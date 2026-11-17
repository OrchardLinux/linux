// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Driver for backlight controllers attached via Apple DWI 2-wire interface
 *
 * Copyright (c) 2024 Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DWI_BL_CTL			0x0
#define DWI_BL_CTL_SEND1		BIT(0)
#define DWI_BL_CTL_SEND2		BIT(4)
#define DWI_BL_CTL_SEND3		BIT(5)
#define DWI_BL_CTL_LE_DATA		BIT(6)
/* Only used on Apple A9 and later */
#define DWI_BL_CTL_SEND4		BIT(12)

#define DWI_BL_CMD			0x4
#define DWI_BL_CMD_TYPE			GENMASK(31, 28)
#define DWI_BL_CMD_TYPE_SET_BRIGHTNESS	0xa
#define DWI_BL_CMD_DATA			GENMASK(10, 0)

#define DWI_BL_CTL_SEND			(DWI_BL_CTL_SEND1 | \
					 DWI_BL_CTL_SEND2 | \
					 DWI_BL_CTL_SEND3 | \
					 DWI_BL_CTL_LE_DATA | \
					 DWI_BL_CTL_SEND4)

#define DWI_BL_MAX_BRIGHTNESS		2047

/* A command takes well under a millisecond on the wire. */
#define DWI_BL_SEND_TIMEOUT_US		1000

/*
 * TI LP8559 command register: the LED outputs are on while bits 0 and 3
 * are set. A brightness of 0 sent over DWI leaves them lit at the lowest
 * current; iOS (AppleARMBacklight, "bklE") turns the backlight off by
 * clearing both bits and on by setting them, then waits 600 us before the
 * first level.
 */
#define LP8559_COMMAND			0x00
#define LP8559_COMMAND_ON		(BIT(0) | BIT(3))
#define LP8559_ON_DELAY_US		600

struct apple_dwi_bl {
	void __iomem *base;
	struct i2c_client *led;	/* the LED driver's own enable, if separate */
	bool led_usable;
	bool led_on;
};

static int dwi_bl_led_enable(struct backlight_device *bl, bool on)
{
	struct apple_dwi_bl *dwi_bl = bl_get_data(bl);
	int ret;

	if (!dwi_bl->led_usable || dwi_bl->led_on == on)
		return 0;

	ret = i2c_smbus_read_byte_data(dwi_bl->led, LP8559_COMMAND);
	if (ret >= 0)
		ret = i2c_smbus_write_byte_data(dwi_bl->led, LP8559_COMMAND,
						on ? ret | LP8559_COMMAND_ON :
						ret & ~LP8559_COMMAND_ON);
	if (ret < 0) {
		dev_err(&bl->dev, "cannot turn the LED driver %s: %d\n",
			on ? "on" : "off", ret);
		return ret;
	}
	dwi_bl->led_on = on;
	if (on)
		fsleep(LP8559_ON_DELAY_US);
	return 0;
}

static int dwi_bl_update_status(struct backlight_device *bl)
{
	struct apple_dwi_bl *dwi_bl = bl_get_data(bl);

	int brightness = backlight_get_brightness(bl);

	u32 cmd = 0;
	u32 ctl;
	int ret;

	cmd |= FIELD_PREP(DWI_BL_CMD_DATA, brightness);
	cmd |= FIELD_PREP(DWI_BL_CMD_TYPE, DWI_BL_CMD_TYPE_SET_BRIGHTNESS);

	/* On before the first level, off after the last one. */
	if (brightness) {
		ret = dwi_bl_led_enable(bl, true);
		if (ret)
			return ret;
	}

	/* A command written while the previous one is still sent is lost. */
	ret = readl_poll_timeout(dwi_bl->base + DWI_BL_CTL, ctl,
				 !(ctl & DWI_BL_CTL_SEND1), 10,
				 DWI_BL_SEND_TIMEOUT_US);
	if (ret) {
		dev_err(&bl->dev, "previous command still pending: %#x\n", ctl);
		return ret;
	}

	writel(cmd, dwi_bl->base + DWI_BL_CMD);
	writel(DWI_BL_CTL_SEND, dwi_bl->base + DWI_BL_CTL);

	if (!brightness)
		return dwi_bl_led_enable(bl, false);
	return 0;
}

static int dwi_bl_get_brightness(struct backlight_device *bl)
{
	struct apple_dwi_bl *dwi_bl = bl_get_data(bl);

	u32 cmd = readl(dwi_bl->base + DWI_BL_CMD);

	return FIELD_GET(DWI_BL_CMD_DATA, cmd);
}

static const struct backlight_ops dwi_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = dwi_bl_get_brightness,
	.update_status	= dwi_bl_update_status
};

static void dwi_bl_put_led(void *data)
{
	struct apple_dwi_bl *dwi_bl = data;

	put_device(&dwi_bl->led->dev);
}

static void dwi_bl_unlink_led(void *data)
{
	device_link_del(data);
}

/*
 * The LED driver the DWI commands reach can have an enable of its own on
 * I2C. Only a TI LP8559 is driven here. Sleep must suspend this device
 * before that bus, and resume it after.
 */
static int dwi_bl_get_led(struct platform_device *pdev, struct apple_dwi_bl *dwi_bl)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct device_link *link;
	int ret;

	np = of_parse_phandle(dev->of_node, "apple,led-driver", 0);
	if (!np)
		return 0;
	/* Only the LP8559's command register is known; leave others alone. */
	if (!of_device_is_compatible(np, "ti,lp8559")) {
		of_node_put(np);
		return 0;
	}
	dwi_bl->led = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!dwi_bl->led)
		return dev_err_probe(dev, -EPROBE_DEFER, "LED driver not available yet\n");
	ret = devm_add_action_or_reset(dev, dwi_bl_put_led, dwi_bl);
	if (ret)
		return ret;

	link = device_link_add(dev, &dwi_bl->led->dev, DL_FLAG_STATELESS);
	if (!link)
		return dev_err_probe(dev, -EINVAL, "cannot order sleep after the LED driver\n");
	ret = devm_add_action_or_reset(dev, dwi_bl_unlink_led, link);
	if (ret)
		return ret;

	/* Without its enable, the backlight still works; level 0 stays lit. */
	ret = i2c_smbus_read_byte_data(dwi_bl->led, LP8559_COMMAND);
	if (ret < 0) {
		dev_warn(dev, "LED driver does not answer (%d); it stays on\n", ret);
		return 0;
	}
	dwi_bl->led_usable = true;
	dwi_bl->led_on = (ret & LP8559_COMMAND_ON) == LP8559_COMMAND_ON;
	return 0;
}

static int dwi_bl_probe(struct platform_device *dev)
{
	struct apple_dwi_bl *dwi_bl;
	struct backlight_device *bl;
	struct backlight_properties props;
	struct resource *res;
	int ret;

	dwi_bl = devm_kzalloc(&dev->dev, sizeof(*dwi_bl), GFP_KERNEL);
	if (!dwi_bl)
		return -ENOMEM;

	dwi_bl->base = devm_platform_get_and_ioremap_resource(dev, 0, &res);
	if (IS_ERR(dwi_bl->base))
		return PTR_ERR(dwi_bl->base);

	ret = dwi_bl_get_led(dev, dwi_bl);
	if (ret)
		return ret;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = DWI_BL_MAX_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	bl = devm_backlight_device_register(&dev->dev, dev->name, &dev->dev,
					dwi_bl, &dwi_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	platform_set_drvdata(dev, dwi_bl);

	bl->props.brightness = dwi_bl_get_brightness(bl);
	/* A level from the boot loader or a previous kernel is meant to be seen. */
	if (bl->props.brightness)
		dwi_bl_led_enable(bl, true);

	return 0;
}

static const struct of_device_id dwi_bl_of_match[] = {
	{ .compatible = "apple,dwi-bl" },
	{},
};

MODULE_DEVICE_TABLE(of, dwi_bl_of_match);

static struct platform_driver dwi_bl_driver = {
	.driver		= {
		.name	= "apple-dwi-bl",
		.of_match_table = dwi_bl_of_match
	},
	.probe		= dwi_bl_probe,
};

module_platform_driver(dwi_bl_driver);

MODULE_DESCRIPTION("Apple DWI Backlight Driver");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
