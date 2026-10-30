// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple S8000 (A9) and T8010 (A10) Lightning port USB role switch.
 *
 * The A9 and A10 USB complex has a device-only DWC2 and a separate EHCI +
 * OHCI host pair, all behind one AUSB PHY. For a USB device on the port, iOS
 * keeps the device controller and publishes EHCI and OHCI next to it. The
 * USB_CTL register bank moves the PHY to the host pair with bit 0; bit 1 is
 * set while the PHY is powered for the host pair.
 *
 * Device mode is the default. The Lightning accessory policy of the Tristar
 * charger sets the host role while the camera adapter is attached (see
 * <linux/usb/apple-s8000-role.h>). Host entry also powers the accessory
 * through the PMIC's accessory LDO (LDO5) and its switch, after routing the
 * Tristar ACC pin to the power rail, as iOS does.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/usb/apple-s8000-role.h>
#include <linux/usb/gadget.h>
#include <linux/usb/role.h>

#include "apple-s8000-tristar-route.h"

/* USB_CTL */
#define USB_CTL_HOST			0x00
#define USB_CTL_HOST_EN			BIT(0)
/* Set while the PHY is powered for the host pair. */
#define USB_CTL_PHY_ON			BIT(1)

/*
 * USB complex DMA remap words, owned and programmed at probe by the parent
 * usb-complex driver.
 */
#define REMAP_EHCI0			0x3c
#define REMAP_OHCI0			0x5c
#define REMAP_EN			BIT(8)
#define REMAP_UPPER			GENMASK(3, 0)
/* DRAM is at 0x8_0000_0000; the bus dma-ranges hand out 32-bit addresses. */
#define REMAP_TO_DRAM			(REMAP_EN | FIELD_PREP(REMAP_UPPER, 0x8))

/* D2365 accessory LDO (LDO5) */
#define D2365_LDO5_ENABLE		0x0310
#define D2365_LDO5_EN			BIT(0)
/* Keeps the rail up while the system sleeps. */
#define D2365_LDO5_SLEEP_EN		GENMASK(3, 1)
#define D2365_LDO5_SWITCH		0x2603

/* CBTL1610 (Tristar) */
#define TRISTAR_DX_CONTROL		0x01
#define TRISTAR_ACC_CTRL		0x02
#define TRISTAR_STATUS			0x0c
#define TRISTAR_REVISION		0x0f
/* The revision whose ACC and route encoding is implemented here. */
#define TRISTAR_CBTL1610_REVISION	0x8a
#define TRISTAR_ROUTE_SETTLE_MS		5

/*
 * The port has one switch. The accessory API checks a switch handed to it
 * against this instance, under the lock, so that an unbound switch is never
 * dereferenced.
 */
static DEFINE_MUTEX(apple_usb_role_instance_lock);
static struct apple_usb_role *apple_usb_role_instance;

struct apple_usb_role {
	struct device *dev;
	void __iomem *usb_ctl;
	/* Shared, not claimed: the usb-complex driver owns the remap words. */
	void __iomem *remap;

	struct phy *phy;
	struct platform_device *udc;
	struct device_node *ehci_np;
	struct device_node *ohci_np;
	struct regmap *pmic;
	struct i2c_client *tristar;

	struct usb_role_switch *sw;

	/* Serializes role changes, accessory power and the state below. */
	struct mutex lock;
	enum usb_role role;
	struct device *gadget;
	struct platform_device *ehci;
	struct platform_device *ohci;
	bool phy_held;
	bool ctl_host;
	bool ctl_phy_on;
	/* LDO5 and its switch were (perhaps partly) turned on. */
	bool acc_power;
	bool sleep_power_on;
	/* Accessory power was applied before the policy's ID-bus reads. */
	bool accessory_power_prepared;
	/* Tristar 0x01/0x02 were changed for the adapter; the saved bytes restore them. */
	bool acc_applied;
	u8 acc_saved;
	u8 dx_saved;
	bool dying;
	bool monitor_fault;
};

static void apple_usb_role_iounmap(void *addr)
{
	iounmap(addr);
}

/* Map a register range without claiming it: another driver claims it. */
static void __iomem *apple_usb_role_view(struct apple_usb_role *ur,
					 struct device_node *np, int index)
{
	void __iomem *base = of_iomap(np, index);

	if (!base || devm_add_action_or_reset(ur->dev, apple_usb_role_iounmap,
					      (void __force *)base))
		return NULL;
	return base;
}

/* -------------------------------------------------------------------- */
/* Accessory power */

static int apple_usb_role_tristar_read(struct apple_usb_role *ur, u8 reg)
{
	struct i2c_client *client = ur->tristar;
	union i2c_smbus_data data;
	int ret;

	/* Register selector and STOP, then a separate read. */
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE, NULL);
	if (!ret)
		ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				       I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret ?: data.byte;
}

static int apple_usb_role_tristar_write(struct apple_usb_role *ur, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ur->tristar, reg, val);
	if (ret)
		return ret;
	msleep(TRISTAR_ROUTE_SETTLE_MS);
	ret = apple_usb_role_tristar_read(ur, reg);
	if (ret < 0)
		return ret;
	return ret == val ? 0 : -EIO;
}

static int apple_usb_role_ldo5(struct apple_usb_role *ur, bool on)
{
	return regmap_update_bits(ur->pmic, D2365_LDO5_ENABLE, D2365_LDO5_EN,
				  on ? D2365_LDO5_EN : 0);
}

static int apple_usb_role_ldo5_switch(struct apple_usb_role *ur, bool on)
{
	/* iOS writes the whole register: 1 or 0. */
	return regmap_write(ur->pmic, D2365_LDO5_SWITCH, on);
}

static int apple_usb_role_ldo5_sleep(struct apple_usb_role *ur, bool on)
{
	return regmap_update_bits(ur->pmic, D2365_LDO5_ENABLE,
				  D2365_LDO5_SLEEP_EN,
				  on ? D2365_LDO5_SLEEP_EN : 0);
}

/*
 * iOS applies the ACC and route configuration when the accessory attaches
 * and again before it enables accessory power, so that the ACC pin carries
 * the power rail (apwr) before LDO5 comes on. The values and the dxctrl
 * write order are in apple-s8000-tristar-route.h.
 */
static int apple_usb_role_acc_config(struct apple_usb_role *ur)
{
	struct tristar_route_plan plan;
	int ret, status;

	if (ur->acc_applied)
		return 0;
	if (!ur->tristar)
		return -ENODEV;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_REVISION);
	if (ret < 0)
		return ret;
	if (ret != TRISTAR_CBTL1610_REVISION)
		return -ENODEV;
	status = apple_usb_role_tristar_read(ur, TRISTAR_STATUS);
	if (status < 0)
		return status;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_DX_CONTROL);
	if (ret < 0)
		return ret;
	ur->dx_saved = ret;
	ret = apple_usb_role_tristar_read(ur, TRISTAR_ACC_CTRL);
	if (ret < 0)
		return ret;
	ur->acc_saved = ret;
	if (!tristar_route_plan_for(status, ur->dx_saved, &plan)) {
		dev_warn(ur->dev, "Tristar reports no plug (status %#04x)\n", status);
		return -ENOLINK;
	}
	/* From the first write on, teardown restores both registers. */
	ur->acc_applied = true;
	ret = 0;
	if (plan.dx_write) {
		if (plan.dx_two_steps)
			ret = apple_usb_role_tristar_write(ur, TRISTAR_DX_CONTROL, plan.dx_first);
		if (!ret)
			ret = apple_usb_role_tristar_write(ur, TRISTAR_DX_CONTROL, plan.dx_target);
		if (ret)
			return ret;
	}
	if (ur->acc_saved != plan.acc)
		ret = apple_usb_role_tristar_write(ur, TRISTAR_ACC_CTRL, plan.acc);
	dev_dbg(ur->dev, "orientation %u: dxctrl %#04x -> %#04x, ACC_CTRL %#04x -> %#04x (%d)\n",
		plan.orientation, ur->dx_saved, plan.dx_target, ur->acc_saved,
		plan.acc, ret);
	return ret;
}

static void apple_usb_role_acc_restore_reg(struct apple_usb_role *ur, u8 reg, u8 saved)
{
	int ret = apple_usb_role_tristar_read(ur, reg);

	if (ret == saved)
		return;
	if (ret >= 0)
		ret = apple_usb_role_tristar_write(ur, reg, saved);
	if (ret < 0)
		dev_err(ur->dev, "Tristar register %#04x restore: %d\n", reg, ret);
}

static void apple_usb_role_acc_restore(struct apple_usb_role *ur)
{
	if (!ur->acc_applied)
		return;
	ur->acc_applied = false;
	/* Reverse of the apply order: ACC_CTRL, then dxctrl. */
	apple_usb_role_acc_restore_reg(ur, TRISTAR_ACC_CTRL, ur->acc_saved);
	apple_usb_role_acc_restore_reg(ur, TRISTAR_DX_CONTROL, ur->dx_saved);
}

static void apple_usb_role_acc_power_off(struct apple_usb_role *ur)
{
	int ret;

	if (ur->acc_power) {
		if (ur->sleep_power_on) {
			ret = apple_usb_role_ldo5_sleep(ur, false);
			if (ret)
				dev_err(ur->dev, "LDO5 sleep power off: %d\n", ret);
			ur->sleep_power_on = false;
		}
		ret = apple_usb_role_ldo5_switch(ur, false);
		if (ret)
			dev_err(ur->dev, "LDO5 switch off: %d\n", ret);
		ret = apple_usb_role_ldo5(ur, false);
		if (ret)
			dev_err(ur->dev, "LDO5 off: %d\n", ret);
		ur->acc_power = false;
	}
	apple_usb_role_acc_restore(ur);
}

/*
 * The power iOS gives the camera adapter: ACC pin routing first, then the
 * rail, its switch and its sleep enable, before the hub enumerates. Without
 * a PMIC there is nothing to switch.
 */
static int apple_usb_role_acc_power_on(struct apple_usb_role *ur)
{
	int ret;

	if (!ur->pmic)
		return 0;
	ret = apple_usb_role_acc_config(ur);
	if (ret) {
		dev_err(ur->dev, "Tristar ACC configuration: %d\n", ret);
		return ret;
	}
	ur->acc_power = true;
	ret = apple_usb_role_ldo5(ur, true);
	if (!ret)
		ret = apple_usb_role_ldo5_switch(ur, true);
	if (!ret) {
		ret = apple_usb_role_ldo5_sleep(ur, true);
		if (!ret)
			ur->sleep_power_on = true;
	}
	if (ret)
		dev_err(ur->dev, "accessory power (LDO5): %d\n", ret);
	return ret;
}

/* -------------------------------------------------------------------- */
/* Role changes */

static int apple_usb_role_is_gadget(struct device *dev, const void *data)
{
	return dev->bus && !strcmp(dev->bus->name, "gadget");
}

/*
 * iOS leaves the device controller published while its host controllers
 * are active. Keep the gadget driver bound and drop only its pull-up.
 */
static int apple_usb_role_release_gadget(struct apple_usb_role *ur)
{
	struct device *gadget;
	int ret;

	gadget = device_find_child(&ur->udc->dev, NULL, apple_usb_role_is_gadget);
	if (!gadget)
		return 0;
	if (!gadget->driver) {
		/* Nothing to give back later either. */
		put_device(gadget);
		return 0;
	}
	ret = usb_gadget_disconnect(dev_to_usb_gadget(gadget));
	if (ret) {
		dev_err(ur->dev, "gadget soft disconnect: %d\n", ret);
		put_device(gadget);
		return ret;
	}
	ur->gadget = gadget;
	return 0;
}

static void apple_usb_role_restore_gadget(struct apple_usb_role *ur)
{
	int ret;

	if (!ur->gadget)
		return;
	ret = usb_gadget_connect(dev_to_usb_gadget(ur->gadget));
	if (ret)
		dev_err(ur->dev, "gadget soft reconnect: %d\n", ret);
	put_device(ur->gadget);
	ur->gadget = NULL;
}

/* Only repairs a value lost since the usb-complex driver's probe. */
static void apple_usb_role_remap(struct apple_usb_role *ur, unsigned int offset)
{
	if (readl(ur->remap + offset) != REMAP_TO_DRAM)
		writel(REMAP_TO_DRAM, ur->remap + offset);
}

static struct platform_device *apple_usb_role_add_hc(struct apple_usb_role *ur,
						     struct device_node *np)
{
	struct platform_device *pdev = of_platform_device_create(np, NULL, ur->dev);

	if (!pdev) {
		dev_err(ur->dev, "%pOFn: could not create the controller\n", np);
		return NULL;
	}
	/* ehci/ohci-platform prefer asynchronous probing. */
	wait_for_device_probe();
	if (!pdev->dev.driver) {
		dev_err(ur->dev, "%s: no driver bound\n", dev_name(&pdev->dev));
		of_platform_device_destroy(&pdev->dev, NULL);
		return NULL;
	}
	return pdev;
}

static int apple_usb_role_add_hcs(struct apple_usb_role *ur)
{
	/* EHCI before its OHCI companion, as for any companion pair. */
	ur->ehci = apple_usb_role_add_hc(ur, ur->ehci_np);
	if (!ur->ehci)
		return -ENODEV;
	ur->ohci = apple_usb_role_add_hc(ur, ur->ohci_np);
	if (!ur->ohci)
		dev_warn(ur->dev, "no OHCI: low/full-speed devices will not work\n");
	return 0;
}

static void apple_usb_role_ctl_clear(struct apple_usb_role *ur)
{
	/* As iOS leaves host mode: host select first, then bit 1. */
	if (ur->ctl_host) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_HOST_EN,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_host = false;
	}
	if (ur->ctl_phy_on) {
		writel(readl(ur->usb_ctl + USB_CTL_HOST) & ~USB_CTL_PHY_ON,
		       ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_phy_on = false;
	}
}

/* Undo whatever enter_host() got to, in reverse order. Idempotent. */
static void apple_usb_role_host_off(struct apple_usb_role *ur)
{
	int ret;

	if (ur->ohci) {
		of_platform_device_destroy(&ur->ohci->dev, NULL);
		ur->ohci = NULL;
	}
	if (ur->ehci) {
		of_platform_device_destroy(&ur->ehci->dev, NULL);
		ur->ehci = NULL;
	}
	apple_usb_role_ctl_clear(ur);
	if (ur->phy_held) {
		ret = phy_set_mode(ur->phy, PHY_MODE_USB_DEVICE);
		if (ret)
			dev_err(ur->dev, "PHY device mode: %d\n", ret);
		phy_power_off(ur->phy);
		phy_exit(ur->phy);
		ur->phy_held = false;
	}
	apple_usb_role_acc_power_off(ur);
	ur->accessory_power_prepared = false;
}

static int apple_usb_role_enter_host(struct apple_usb_role *ur)
{
	u32 ctl;
	int ret;

	ret = apple_usb_role_release_gadget(ur);
	if (ret)
		goto fail;

	ret = apple_usb_role_acc_power_on(ur);
	if (ret)
		goto fail;

	apple_usb_role_remap(ur, REMAP_EHCI0);
	apple_usb_role_remap(ur, REMAP_OHCI0);

	ret = phy_init(ur->phy);
	if (ret)
		goto fail;
	ret = phy_power_on(ur->phy);
	if (ret) {
		phy_exit(ur->phy);
		goto fail;
	}
	ur->phy_held = true;

	/*
	 * iOS creates EHCI and OHCI before it selects host mode, so their
	 * power domains are on when USB_CTL bit 1 is written: with the domains
	 * still off the hardware discards the bit.
	 */
	ret = apple_usb_role_add_hcs(ur);
	if (ret)
		goto fail;
	ctl = readl(ur->usb_ctl + USB_CTL_HOST);
	if (!(ctl & USB_CTL_PHY_ON)) {
		writel(ctl | USB_CTL_PHY_ON, ur->usb_ctl + USB_CTL_HOST);
		ur->ctl_phy_on = true;
	}

	/* Host select, then the PHY's host tunables and reset. */
	ctl = readl(ur->usb_ctl + USB_CTL_HOST);
	writel(ctl | USB_CTL_PHY_ON | USB_CTL_HOST_EN, ur->usb_ctl + USB_CTL_HOST);
	ur->ctl_host = true;
	ret = phy_set_mode(ur->phy, PHY_MODE_USB_HOST);
	if (ret)
		goto fail;

	/* If something dropped bit 1 on the way, assert it once more. */
	ctl = readl(ur->usb_ctl + USB_CTL_HOST);
	if (!(ctl & USB_CTL_PHY_ON))
		writel(ctl | USB_CTL_PHY_ON, ur->usb_ctl + USB_CTL_HOST);

	dev_dbg(ur->dev, "host mode\n");
	return 0;

fail:
	apple_usb_role_host_off(ur);
	apple_usb_role_restore_gadget(ur);
	return ret;
}

static void apple_usb_role_enter_device(struct apple_usb_role *ur)
{
	apple_usb_role_host_off(ur);
	apple_usb_role_restore_gadget(ur);
	dev_dbg(ur->dev, "device mode\n");
}

static int apple_usb_role_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct apple_usb_role *ur = usb_role_switch_get_drvdata(sw);
	int ret = 0;

	/* There is no idle state: the gadget is the resting role. */
	if (role == USB_ROLE_NONE)
		role = USB_ROLE_DEVICE;

	mutex_lock(&ur->lock);
	if (ur->dying) {
		ret = -ESHUTDOWN;
	} else if (role == USB_ROLE_HOST && ur->monitor_fault) {
		ret = -EUCLEAN;
	} else if (role == USB_ROLE_HOST && ur->role != USB_ROLE_HOST) {
		ret = apple_usb_role_enter_host(ur);
		if (!ret) {
			WRITE_ONCE(ur->role, USB_ROLE_HOST);
			ur->accessory_power_prepared = false;
		}
	} else if (role == USB_ROLE_DEVICE && ur->role != USB_ROLE_DEVICE) {
		apple_usb_role_enter_device(ur);
		WRITE_ONCE(ur->role, USB_ROLE_DEVICE);
	}
	mutex_unlock(&ur->lock);
	return ret;
}

static enum usb_role apple_usb_role_get(struct usb_role_switch *sw)
{
	struct apple_usb_role *ur = usb_role_switch_get_drvdata(sw);

	return READ_ONCE(ur->role);
}

/* Caller holds apple_usb_role_instance_lock, which keeps the instance alive. */
static struct apple_usb_role *apple_usb_role_from_switch(struct usb_role_switch *sw)
{
	struct apple_usb_role *ur;

	lockdep_assert_held(&apple_usb_role_instance_lock);
	if (IS_ERR_OR_NULL(sw) || !apple_usb_role_instance)
		return NULL;
	ur = usb_role_switch_get_drvdata(sw);
	return ur == apple_usb_role_instance ? ur : NULL;
}

/**
 * apple_s8000_usb_role_accessory_power - apply or release accessory power
 * @sw: the Lightning role switch
 * @on: apply before ID-bus identification, or unwind after a failed sequence
 *
 * iOS powers the accessory before it reads the accessory's information or
 * changes the USB mode. This only sets up the local PMIC and Tristar state;
 * it sends nothing on the ID bus. A successful host entry takes the power
 * over, and leaving host mode turns it off.
 *
 * Return: 0, -EBUSY outside the device role, -ESHUTDOWN while the switch
 * goes away, -ENODEV if @sw is not this driver's switch, or the error of a
 * failed PMIC or Tristar access.
 */
int apple_s8000_usb_role_accessory_power(struct usb_role_switch *sw, bool on)
{
	struct apple_usb_role *ur;
	int ret = -ENODEV;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		if (ur->dying) {
			ret = -ESHUTDOWN;
		} else if (ur->role != USB_ROLE_DEVICE) {
			ret = -EBUSY;
		} else if (on && !ur->accessory_power_prepared) {
			ret = apple_usb_role_acc_power_on(ur);
			if (ret)
				apple_usb_role_acc_power_off(ur);
			else
				ur->accessory_power_prepared = true;
		} else if (!on && ur->accessory_power_prepared) {
			apple_usb_role_acc_power_off(ur);
			ur->accessory_power_prepared = false;
			ret = 0;
		} else {
			ret = 0;
		}
		mutex_unlock(&ur->lock);
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_accessory_power);

/**
 * apple_s8000_usb_role_monitor - serialize a Tristar sample with role changes
 * @sw: the Lightning role switch
 * @sample: sleepable callback; must not query or change the USB role
 * @data: callback argument
 *
 * Lock order: instance, role, then the caller's locks taken in @sample. The
 * callback must restore DX_CONTROL before it returns. If it returns -EUCLEAN
 * (route left broken), host entry is refused until a later callback succeeds.
 * No role change can overlap the callback.
 *
 * Return: the callback's result, -EBUSY outside the device role, or -ENODEV
 * if @sw is not this driver's switch.
 */
int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
				 int (*sample)(void *data), void *data)
{
	struct apple_usb_role *ur;
	int ret = -ENODEV;

	mutex_lock(&apple_usb_role_instance_lock);
	ur = apple_usb_role_from_switch(sw);
	if (ur) {
		mutex_lock(&ur->lock);
		if (ur->dying || ur->role != USB_ROLE_DEVICE) {
			ret = -EBUSY;
		} else {
			ret = sample(data);
			ur->monitor_fault = ret == -EUCLEAN;
		}
		mutex_unlock(&ur->lock);
	}
	mutex_unlock(&apple_usb_role_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_s8000_usb_role_monitor);

/* -------------------------------------------------------------------- */
/* Probe */

static void apple_usb_role_put_device(void *data)
{
	put_device(data);
}

static void apple_usb_role_put_node(void *data)
{
	of_node_put(data);
}

static void apple_usb_role_unregister(void *data)
{
	usb_role_switch_unregister(data);
}

static int apple_usb_role_get_pmic(struct apple_usb_role *ur)
{
	struct device_node *np;
	struct i2c_client *client;
	int ret;

	np = of_parse_phandle(ur->dev->of_node, "apple,pmic", 0);
	if (!np)
		return 0;
	client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!client)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_device, &client->dev);
	if (ret)
		return ret;
	ur->pmic = dev_get_regmap(&client->dev, NULL);
	if (!ur->pmic)
		return -EPROBE_DEFER;
	if (!device_link_add(ur->dev, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;
	return 0;
}

static int apple_usb_role_get_tristar(struct apple_usb_role *ur)
{
	struct device_node *np;

	np = of_parse_phandle(ur->dev->of_node, "apple,tristar", 0);
	if (!np)
		return 0;
	ur->tristar = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!ur->tristar)
		return -EPROBE_DEFER;
	return devm_add_action_or_reset(ur->dev, apple_usb_role_put_device,
					&ur->tristar->dev);
}

/* The remap words belong to the parent usb-complex, whose driver claims them. */
static int apple_usb_role_get_complex(struct apple_usb_role *ur)
{
	struct device_node *complex = of_get_parent(ur->dev->of_node);

	if (complex && of_device_is_compatible(complex, "apple,s5l8960x-usb-complex"))
		ur->remap = apple_usb_role_view(ur, complex, 0);
	of_node_put(complex);
	return ur->remap ? 0 : -ENODEV;
}

static int apple_usb_role_get_udc(struct apple_usb_role *ur)
{
	struct device_node *np;

	np = of_parse_phandle(ur->dev->of_node, "apple,usb-device", 0);
	if (!np)
		return -EINVAL;
	ur->udc = of_find_device_by_node(np);
	of_node_put(np);
	if (!ur->udc)
		return -EPROBE_DEFER;
	return devm_add_action_or_reset(ur->dev, apple_usb_role_put_device, &ur->udc->dev);
}

static int apple_usb_role_get_hcs(struct apple_usb_role *ur)
{
	struct device_node *np = ur->dev->of_node;
	int ret;

	ur->ehci_np = of_get_compatible_child(np, "generic-ehci");
	if (!ur->ehci_np)
		return -ENODEV;
	ret = devm_add_action_or_reset(ur->dev, apple_usb_role_put_node, ur->ehci_np);
	if (ret)
		return ret;
	ur->ohci_np = of_get_compatible_child(np, "generic-ohci");
	if (!ur->ohci_np)
		return -ENODEV;
	return devm_add_action_or_reset(ur->dev, apple_usb_role_put_node, ur->ohci_np);
}

static void apple_usb_role_teardown(void *data)
{
	struct apple_usb_role *ur = data;

	mutex_lock(&apple_usb_role_instance_lock);
	if (apple_usb_role_instance == ur)
		apple_usb_role_instance = NULL;
	mutex_unlock(&apple_usb_role_instance_lock);

	mutex_lock(&ur->lock);
	ur->dying = true;
	if (ur->role == USB_ROLE_HOST) {
		apple_usb_role_enter_device(ur);
		WRITE_ONCE(ur->role, USB_ROLE_DEVICE);
	}
	mutex_unlock(&ur->lock);
}

static int apple_usb_role_probe(struct platform_device *pdev)
{
	struct usb_role_switch_desc desc = {};
	struct device *dev = &pdev->dev;
	struct apple_usb_role *ur;
	int ret;

	ur = devm_kzalloc(dev, sizeof(*ur), GFP_KERNEL);
	if (!ur)
		return -ENOMEM;
	ur->dev = dev;
	ur->role = USB_ROLE_DEVICE;
	platform_set_drvdata(pdev, ur);
	ret = devm_mutex_init(dev, &ur->lock);
	if (ret)
		return ret;

	ur->usb_ctl = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ur->usb_ctl))
		return PTR_ERR(ur->usb_ctl);
	ret = apple_usb_role_get_complex(ur);
	if (ret)
		return dev_err_probe(dev, ret, "usb-complex registers\n");

	ur->phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(ur->phy))
		return dev_err_probe(dev, PTR_ERR(ur->phy), "PHY\n");

	ret = apple_usb_role_get_udc(ur);
	if (ret)
		return dev_err_probe(dev, ret, "device controller\n");
	ret = apple_usb_role_get_hcs(ur);
	if (ret)
		return dev_err_probe(dev, ret, "host controller nodes\n");
	ret = apple_usb_role_get_pmic(ur);
	if (ret)
		return dev_err_probe(dev, ret, "PMIC\n");
	ret = apple_usb_role_get_tristar(ur);
	if (ret)
		return dev_err_probe(dev, ret, "Tristar\n");

	/* No register access here: device mode must work as without the switch. */
	desc.fwnode = dev_fwnode(dev);
	desc.name = "lightning";
	desc.set = apple_usb_role_set;
	desc.get = apple_usb_role_get;
	desc.driver_data = ur;
	ur->sw = usb_role_switch_register(dev, &desc);
	if (IS_ERR(ur->sw))
		return dev_err_probe(dev, PTR_ERR(ur->sw), "role switch\n");
	ret = devm_add_action_or_reset(dev, apple_usb_role_unregister, ur->sw);
	if (ret)
		return ret;
	/* Runs before the switch goes away: no role change can race it. */
	ret = devm_add_action_or_reset(dev, apple_usb_role_teardown, ur);
	if (ret)
		return ret;

	mutex_lock(&apple_usb_role_instance_lock);
	apple_usb_role_instance = ur;
	mutex_unlock(&apple_usb_role_instance_lock);
	dev_info(dev, "Lightning port in device role%s\n",
		 ur->pmic ? ", accessory power available" : "");
	return 0;
}

/*
 * The accessory rail and the Tristar mux keep their state across an SoC
 * reset, and the boot loader brings the port up as a device again: put the
 * shared hardware back. The host controllers, children of this device, are
 * already shut down.
 */
static void apple_usb_role_shutdown(struct platform_device *pdev)
{
	struct apple_usb_role *ur = platform_get_drvdata(pdev);

	mutex_lock(&ur->lock);
	ur->dying = true;
	apple_usb_role_ctl_clear(ur);
	if (ur->phy_held)
		phy_set_mode(ur->phy, PHY_MODE_USB_DEVICE);
	apple_usb_role_acc_power_off(ur);
	mutex_unlock(&ur->lock);
}

static int apple_usb_role_prepare(struct device *dev)
{
	struct apple_usb_role *ur = dev_get_drvdata(dev);

	/* The role cannot change mid-suspend. */
	if (READ_ONCE(ur->role) == USB_ROLE_HOST) {
		dev_warn(dev, "host mode: return to device mode before suspending\n");
		return -EBUSY;
	}
	return 0;
}

static const struct dev_pm_ops apple_usb_role_pm_ops = {
	.prepare = pm_sleep_ptr(apple_usb_role_prepare),
};

static const struct of_device_id apple_usb_role_of_match[] = {
	{ .compatible = "apple,s8000-usb-role-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_usb_role_of_match);

static struct platform_driver apple_usb_role_driver = {
	.probe = apple_usb_role_probe,
	.shutdown = apple_usb_role_shutdown,
	.driver = {
		.name = "apple-s8000-usb-role",
		.of_match_table = apple_usb_role_of_match,
		.pm = pm_ptr(&apple_usb_role_pm_ops),
	},
};
module_platform_driver(apple_usb_role_driver);

MODULE_DESCRIPTION("Apple S8000 Lightning port USB role switch");
MODULE_LICENSE("GPL");
