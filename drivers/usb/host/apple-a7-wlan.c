// SPDX-License-Identifier: GPL-2.0-only
/* Apple A7 (S5L8960X) HSIC Wi-Fi transport owner: PHY, radio reset, HOST_READY
 * and the EHCI root port of the BCM43241.
 *
 * Where the radio sits is board wiring and comes from the device tree: the
 * apple,s5l8960x-hsic-wlan node names the HSIC PHY and EHCI registers, the
 * EHCI interrupt, the HSIC PHY power state (whose parent is the USB2HOST
 * one), the EHCI root port and the HOST_READY/DEVICE_READY pins. The iPad
 * Air's radio is on EHCI0 port 3, the iPad mini 2's and 3's on EHCI1 port 1.
 *
 * The register sequence was measured on an iPad Air (J72): PMU GPIO3
 * 09->0b->09, HOST_READY through a standard exclusive gpiod consumer claim
 * that retains direction/mux and returns low before reset/PHY stop, standard
 * EHCI enumeration and a strict ROM query; optional persistent owner for the
 * separately loaded firmware driver. The iPad mini 2 (J85) ran it too. Every
 * board gets one policy: a value a following write depends on must match the
 * J72 measurement or the write is refused; a value that only describes the
 * starting state is logged when it differs, and the sequence continues.
 */
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/dma-direct.h>
#include <linux/of_device.h>
#include <linux/irqdomain.h>
#include <linux/ioport.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "ehci.h"

#define DRV_NAME "apple-a7-wlan"
#define PMGR "/soc/power-management@20e000000/"
#define PMGR_BASE 0x20e000000ULL
#define USB_COMPLEX 0x20c900000ULL	/* ADT usb-complex reg, usb_widget at +1c/3c/5c/7c */
#define GPIO_BASE 0x20e300000ULL
#define PMU_GPIO3 0x403		/* D2089 GPIO3 configuration: radio reset */
/* The BCM43241 boot ROM: AppleBCMWLANBusInterfaceHSICShim's "4324" personality */
#define RADIO_PID 0xbd1c

/* Measured on J72 (iPad Air 1); the reference for every board. */
#define J72_PHY_CONTROL 4
#define J72_PHY_CONFIG 0x153
#define J72_WIDGET 0x108
#define J72_PMU_RESET 0x09
#define J72_HOST_READY_CFG 0x00072602
#define J72_DEVICE_READY_CFG 0x00070200
#define J72_EHCI_CAPLENGTH 0x01000010
#define J72_EHCI_HCSPARAMS 0x1313
#define J72_EHCI_HCCPARAMS 0x2a026
#define J72_ROM_CHIP 0x4324
#define J72_ROM_REV 6
#define J72_ROM_RAMSIZE 0x90000
#define J72_ROM_REMAP 0x60000000

/* The radio's wiring, from its apple,s5l8960x-hsic-wlan device tree node. */
struct a7_wlan_board {
	const char *name;		/* the machine's first compatible */
	u32 phy_domain_reg, host_domain_reg;	/* PMGR: HSIC PHY, USB2HOST */
	u64 phy_base;			/* HSIC PHY control (+0) / config (+8) */
	u64 ehci_base;
	u32 ehci_irq;
	unsigned int port;		/* EHCI root port of the radio */
	u32 host_ready_gpio, device_ready_gpio;
	u16 usb_pid;			/* 0a5c:xxxx boot ROM identity */
};

static struct a7_wlan_board radio;
static const struct a7_wlan_board *board;

/* A J72 measurement that only describes the starting state: a board that
 * differs logs it and continues.
 */
static bool observe(const char *what, u32 seen, u32 j72)
{
	if (seen != j72)
		pr_warn(DRV_NAME ": %s %s=%08x, J72 measured %08x; continuing\n",
			board->name, what, seen, j72);
	return true;
}

/* A J72 measurement that a following write depends on: every board must
 * match it, so a board that differs only logs and never writes on a guess.
 */
static bool gate(const char *what, u32 seen, u32 j72)
{
	if (seen == j72)
		return true;
	pr_warn(DRV_NAME ": %s %s=%08x, J72 measured %08x; refusing the dependent write\n",
		board->name, what, seen, j72);
	return false;
}

static u32 before[2], active[2], after[2], control, config;
static u32 widgets_before[4], widgets_active[4], widgets_after[4];
static int result = -EAGAIN, suspend_result, detach_result;
static bool observed, restored, consumer_removed;
static bool confirm_native, phy_touched, phy_restored;
static u32 original_control, original_config, running_control;
static struct device *pmic;
static struct regmap *pmu_map;
static bool reset_touched, reset_restored;
static u32 pmu_before, pmu_released, pmu_after, host_ready_before, device_ready_before;
static u32 ready_samples, ready_high_samples, last_device_ready;
static struct gpio_device *ready_gdev;
static struct gpio_desc *host_ready;
static bool lookup_installed, host_touched, host_restored, gpio_released;
static u32 host_high_cfg, host_after, high_window_samples, high_window_ready;
static u32 low_window_samples, low_window_ready;
static u32 ehci_caps[3], ehci_initial[7], ehci_high[7], ehci_low[7], ehci_reset_state[7];
static bool ehci_valid;
static bool retain_controller;
static bool consumer_driver_registered;
static int consumer_probe(struct platform_device *pdev) { return 0; }
static struct platform_driver consumer_driver = {
 .probe = consumer_probe,
 .driver = { .name = DRV_NAME, .owner = THIS_MODULE },
};

#include "apple-a7-wlan-hcd.h"

static void ehci_state(void __iomem *base, u32 *values)
{
 static const u32 offsets[] = {0, 4, 8, 0x14, 0x18, 0x40, 0x4c};
 int i;
 /* EHCI caplength is checked before using these standard operational words.
  * Reads only: no controller run/reset, IRQ, port-power or DMA writes.
  */
 for (i = 0; i < 7; i++) values[i] = readl(base + 0x10 + offsets[i]);
}

static struct gpiod_lookup_table ready_lookup = {
 .dev_id = DRV_NAME,
 .table = {
  { .key = NULL, .chip_hwnum = 0, .con_id = "host-ready", .idx = 0, .flags = GPIO_ACTIVE_HIGH },
  { }
 }
};

static int acquire_host_ready(struct device *consumer)
{
 struct device_node *node = of_find_node_by_path("/soc/pinctrl@20e300000");
 int ret;
 if (!node) return -ENODEV;
 ready_gdev = gpio_device_find_by_fwnode(of_fwnode_handle(node));
 of_node_put(node);
 if (IS_ERR(ready_gdev)) { ret = PTR_ERR(ready_gdev); ready_gdev = NULL; return ret; }
 if (!ready_gdev) return -ENODEV;
 ready_lookup.table[0].key = gpio_device_get_label(ready_gdev);
 if (!ready_lookup.table[0].key) return -ENODEV;
 ready_lookup.table[0].chip_hwnum = board->host_ready_gpio;
 gpiod_add_lookup_table(&ready_lookup);
 lookup_installed = true;
 host_ready = gpiod_get(consumer, "host-ready", GPIOD_ASIS);
 if (IS_ERR(host_ready)) { ret = PTR_ERR(host_ready); host_ready = NULL; return ret; }
 ret = gpiod_get_direction(host_ready);
 return ret < 0 ? ret : (ret == 0 ? 0 : -EPROTO);
}

static u32 host_ready_reg(void __iomem *gpio)
{
 return readl(gpio + board->host_ready_gpio * 4);
}

static int ready_window(void __iomem *gpio, u32 expected_host, u32 *samples, u32 *high)
{
 unsigned int i;
 for (i = 0; i < 50; i++) {
  last_device_ready = readl(gpio + board->device_ready_gpio * 4);
  (*samples)++;
  if (last_device_ready & 1) (*high)++;
  if ((last_device_ready & ~1U) != (device_ready_before & ~1U) ||
      host_ready_reg(gpio) != expected_host) return -EIO;
  usleep_range(2000, 2500);
 }
 return 0;
}


static int exact_pmic(struct device *dev, const void *expected)
{
 struct i2c_client *client = i2c_verify_client(dev);
 return client && client->addr == 0x3c && dev->of_node == expected &&
        of_device_is_compatible(dev->of_node, "apple,anya-pmic");
}

static int acquire_pmu(void)
{
 struct device_node *node = of_find_node_by_path("/soc/i2c@20a110000/pmic@3c");
 if (!node) return -ENODEV;
 pmic = bus_find_device(&i2c_bus_type, NULL, node, exact_pmic);
 of_node_put(node);
 if (!pmic) return -ENODEV;
 pmu_map = dev_get_regmap(pmic, NULL);
 if (!pmic->driver || !pmu_map || regmap_get_val_bytes(pmu_map) != 1)
  return -ENODEV;
 return regmap_read_bypassed(pmu_map, PMU_GPIO3, &pmu_before);
}
module_param(confirm_native, bool, 0400);

static void states(void __iomem *phy, void __iomem *host, u32 *values)
{
	values[0] = readl(phy);
	values[1] = readl(host);
}

static bool domain_ready(u32 state)
{
 u32 actual = (state >> 4) & 15;
 /* Match the owning driver's auto-PM semantics, restricted to clock-gated
  * or fully active states. Never accept power-gated or target-off states.
  */
 return (state & 15) == 15 &&
        (actual == 15 || ((state & (1U << 28)) && actual == 4));
}

static bool same_states(const u32 *a, const u32 *b)
{
	/* Only request/actual state nibbles, not PMGR sticky diagnostic flags. */
	return !((a[0] ^ b[0]) & 0xff) && !((a[1] ^ b[1]) & 0xff);
}

static void widgets(void __iomem *base, u32 *values)
{
	int i;
	/* Four exact USB-complex configuration offsets in the owner-local tree
	 * and native arbitrator. No guessed offset or register write.
	 */
	for (i = 0; i < 4; i++)
		values[i] = readl(base + 0x1c + i * 0x20);
}

static int ports_get(char *buf, int len, const char *name, const u32 *ports)
{
	unsigned int i, n = max(n_ports, 3U);

	len += scnprintf(buf + len, PAGE_SIZE - len, "%s=", name);
	for (i = 0; i < n && i < HCS_N_PORTS_MAX; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s%08x",
				 i ? "," : "", ports[i]);
	return len + scnprintf(buf + len, PAGE_SIZE - len, "\n");
}

static int status_get(char *buf, const struct kernel_param *unused)
{
	int len = scnprintf(buf, PAGE_SIZE,
		"board=%s\n"
		"result=%d\nobserved=%u\nrestored=%u\nconsumer_removed=%u\n"
        "phy_touched=%u\nphy_restored=%u\ncontrol_before=%08x\ncontrol_running=%08x\n"
        "reset_touched=%u\nreset_restored=%u\npmu_before=%02x\npmu_released=%02x\npmu_after=%02x\n"
        "host_ready_before=%08x\ndevice_ready_before=%08x\nready_samples=%u\nready_high_samples=%u\nlast_device_ready=%08x\n"
        "host_touched=%u\nhost_restored=%u\ngpio_released=%u\nhost_high_cfg=%08x\nhost_gpio_after=%08x\n"
        "high_window_samples=%u\nhigh_window_ready=%u\nlow_window_samples=%u\nlow_window_ready=%u\n"
        "dma_checked=%u\nhcd_attempted=%u\nhcd_stopped=%u\ncontroller_restored=%u\nretain_controller=%u\n"
        "enumerate_result=%d\nradio_vid=%04x\nradio_pid=%04x\nradio_speed=%u\nrom_result=%d\n"
        "rom_identity=%08x,%08x,%08x,%08x,%08x,%08x\n",
		board ? board->name : "none",
		result, observed, restored, consumer_removed, phy_touched, phy_restored, original_control, running_control,
        reset_touched, reset_restored, pmu_before, pmu_released, pmu_after,
        host_ready_before, device_ready_before, ready_samples, ready_high_samples, last_device_ready,
        host_touched, host_restored, gpio_released, host_high_cfg, host_after,
        high_window_samples, high_window_ready, low_window_samples, low_window_ready,
        dma_checked, hcd_attempted, hcd_stopped, controller_restored, retain_controller,
        enumerate_result, radio_vid, radio_pid, radio_speed, rom_result,
        rom_words[0],rom_words[1],rom_words[2],rom_words[3],rom_words[4],rom_words[5]);

	len = ports_get(buf, len, "saved_ports", saved_ports);
	len = ports_get(buf, len, "live_ports", live_ports);
	len += scnprintf(buf + len, PAGE_SIZE - len,
        "ehci_valid=%u\nehci_caps=%08x,%08x,%08x\n"
        "ehci_initial=%08x,%08x,%08x,%08x,%08x,%08x,%08x\n"
        "ehci_high=%08x,%08x,%08x,%08x,%08x,%08x,%08x\n"
        "ehci_low=%08x,%08x,%08x,%08x,%08x,%08x,%08x\n"
        "ehci_reset=%08x,%08x,%08x,%08x,%08x,%08x,%08x\n"
		"suspend_result=%d\ndetach_result=%d\n"
		"phy_before=%08x\nphy_active=%08x\nphy_after=%08x\n"
		"host_before=%08x\nhost_active=%08x\nhost_after=%08x\n"
		"control=%08x\nconfig=%08x\n"
		"widget_1c=%08x,%08x,%08x\nwidget_3c=%08x,%08x,%08x\n"
		"widget_5c=%08x,%08x,%08x\nwidget_7c=%08x,%08x,%08x\n",
        ehci_valid, ehci_caps[0], ehci_caps[1], ehci_caps[2],
        ehci_initial[0], ehci_initial[1], ehci_initial[2], ehci_initial[3], ehci_initial[4], ehci_initial[5], ehci_initial[6],
        ehci_high[0], ehci_high[1], ehci_high[2], ehci_high[3], ehci_high[4], ehci_high[5], ehci_high[6],
        ehci_low[0], ehci_low[1], ehci_low[2], ehci_low[3], ehci_low[4], ehci_low[5], ehci_low[6],
        ehci_reset_state[0], ehci_reset_state[1], ehci_reset_state[2], ehci_reset_state[3], ehci_reset_state[4], ehci_reset_state[5], ehci_reset_state[6],

        suspend_result,
		detach_result, before[0], active[0], after[0], before[1],
		active[1], after[1], control, config,
		widgets_before[0], widgets_active[0], widgets_after[0],
		widgets_before[1], widgets_active[1], widgets_after[1],
		widgets_before[2], widgets_active[2], widgets_after[2],
		widgets_before[3], widgets_active[3], widgets_after[3]);
	return len;
}
static const struct kernel_param_ops ops = { .get = status_get };
module_param_cb(status, &ops, NULL, 0400);

static struct device_node *np = NULL, *parent = NULL, *host_np = NULL;
static struct of_phandle_args spec = { .args_count = 0 };
static struct platform_device *pdev = NULL;
static struct resource resource;
static u32 provider_reg[2], host_reg[2];
static void __iomem *phy = NULL, *host = NULL, *base = NULL, *complex = NULL, *gpio = NULL, *ehci = NULL;
static bool attached = false, pm_enabled = false, resumed = false, host_pending = false;
static bool keep_active, session_active, stopping;
static DEFINE_MUTEX(ready_lock);
static DEFINE_MUTEX(lifecycle_lock);
module_param(keep_active, bool, 0400);
module_param(session_active, bool, 0400);
static int cleanup_session(void);

/* Raise (cycle: lower, then raise) HOST_READY for the retained radio. */
int apple_a7_wlan_host_ready(struct usb_device *udev, bool cycle);
int apple_a7_wlan_host_ready(struct usb_device *udev, bool cycle)
{
 int ret = -ENODEV, high_ret;
 mutex_lock(&ready_lock);
 if (!session_active || stopping || !host_ready || !radio_hcd || !hcd_added ||
     !udev || udev->bus != &radio_hcd->self ||
     udev->parent != radio_hcd->self.root_hub || udev->portnum != board->port ||
     le16_to_cpu(udev->descriptor.idVendor) != 0x0a5c ||
     le16_to_cpu(udev->descriptor.idProduct) != board->usb_pid ||
     udev->speed != USB_SPEED_HIGH) goto out;
 ret = 0;
 if (cycle) {
  ret = gpiod_set_raw_value_cansleep(host_ready, 0);
  if (!ret && host_ready_reg(gpio) != host_ready_before) ret = -EIO;
  msleep(20);
 }
 /* Always attempt HIGH after a possibly ambiguous LOW write. */
 high_ret = gpiod_set_raw_value_cansleep(host_ready, 1);
 if (!ret) ret = high_ret;
 if (!ret && host_ready_reg(gpio) != (host_ready_before | 1)) ret = -EIO;
 msleep(20);
out:
 mutex_unlock(&ready_lock);
 return ret;
}
EXPORT_SYMBOL_GPL(apple_a7_wlan_host_ready);

static int stop_set(const char *value, const struct kernel_param *unused)
{
 bool requested;
 int ret = kstrtobool(value, &requested);
 if (ret || !requested) return -EINVAL;
 mutex_lock(&lifecycle_lock);
 if (!session_active) ret = -EALREADY;
 else {
  cleanup_session();
  ret = retain_controller || !consumer_removed || !restored ||
        !gpio_released ? -EIO : 0;
 }
 mutex_unlock(&lifecycle_lock);
 return ret;
}
static const struct kernel_param_ops stop_ops = { .set = stop_set };
module_param_cb(stop, &stop_ops, NULL, 0200);

static bool rom_identity_ok(void)
{
	if (rom_result || radio_vid != 0x0a5c || radio_pid != board->usb_pid)
		return false;
	/* The ADT names the chipset ('wifi-chipset' 4324); the rest is J72's. */
	if (rom_words[0] != J72_ROM_CHIP) {
		pr_warn(DRV_NAME ": %s ROM chip %04x is not the ADT 4324\n",
			board->name, rom_words[0]);
		return false;
	}
	observe("ROM revision", rom_words[1], J72_ROM_REV);
	observe("ROM ramsize", rom_words[2], J72_ROM_RAMSIZE);
	observe("ROM remap", rom_words[3], J72_ROM_REMAP);
	return true;
}

/* The ready pins are pinctrl GPIOs: <&pinctrl N flags>. */
static int radio_pin(struct device_node *node, const char *name, u32 *pin)
{
	struct of_phandle_args args;
	int ret = of_parse_phandle_with_args(node, name, "#gpio-cells", 0, &args);

	if (ret)
		return ret;
	ret = of_node_name_eq(args.np, "pinctrl") && args.args_count >= 1 ? 0 : -EINVAL;
	*pin = args.args[0];
	of_node_put(args.np);
	return ret;
}

/* Fill the radio's wiring from its device tree node. */
static int radio_from_dt(void)
{
	struct device_node *node = of_find_compatible_node(NULL, NULL, "apple,s5l8960x-hsic-wlan");
	struct resource phy_res, ehci_res;
	u32 cell[2];
	int ret = -ENODEV;

	if (!node || !of_device_is_available(node))
		goto out;
	if (of_property_read_string_index(of_root, "compatible", 0, &radio.name) ||
	    of_address_to_resource(node, 0, &phy_res) ||
	    of_address_to_resource(node, 1, &ehci_res) ||
	    resource_size(&phy_res) < 0x10 || resource_size(&ehci_res) < 0x10000 ||
	    of_property_read_u32_array(node, "interrupts", cell, 2) || cell[0] != 0 ||
	    of_property_read_u32(node, "apple,ehci-port", &radio.port) ||
	    radio_pin(node, "host-ready-gpios", &radio.host_ready_gpio) ||
	    radio_pin(node, "device-ready-gpios", &radio.device_ready_gpio))
		goto out;
	/* The HSIC PHY power state; its parent is the EHCI's USB2HOST one. */
	np = of_parse_phandle(node, "power-domains", 0);
	host_np = np ? of_parse_phandle(np, "power-domains", 0) : NULL;
	if (!np || !host_np)
		goto out;
	radio.phy_base = phy_res.start;
	radio.ehci_base = ehci_res.start;
	radio.ehci_irq = cell[1];
	radio.usb_pid = RADIO_PID;
	ret = 0;
out:
	of_node_put(node);
	return ret;
}

static int __init observer_init(void)
{
 int ret, i;
 u32 highest;

	if (!confirm_native)
        return -EPERM;
	/* No radio described: nothing to own on this machine. */
	if (radio_from_dt()) {
		of_node_put(host_np);
		of_node_put(np);
		np = host_np = NULL;
		return -ENODEV;
	}
	if (!of_device_is_compatible(np, "apple,pmgr-pwrstate") ||
	    np->parent != host_np->parent ||
	    of_property_read_u32_array(np, "reg", provider_reg, 2) ||
	    of_property_read_u32_array(host_np, "reg", host_reg, 2) ||
	    provider_reg[1] != 4 || host_reg[1] != 4) {
		result = -ENODEV;
		goto out;
	}
	radio.phy_domain_reg = provider_reg[0];
	radio.host_domain_reg = host_reg[0];
	board = &radio;
	pr_info(DRV_NAME ": %s: radio on EHCI %#llx port %u, HSIC PHY %#llx\n",
		board->name, board->ehci_base, board->port, board->phy_base);
	highest = max(board->phy_domain_reg, board->host_domain_reg) + 4;
	if (of_address_to_resource(np->parent, 0, &resource) ||
	    resource.start != PMGR_BASE || resource_size(&resource) < highest) {
		result = -ENODEV;
		goto out;
	}
	parent = of_node_get(host_np);
	phy = ioremap(PMGR_BASE + board->phy_domain_reg, 4);
	host = ioremap(PMGR_BASE + board->host_domain_reg, 4);
	base = ioremap(board->phy_base, 0x10);
	complex = ioremap(USB_COMPLEX, 0x80);
    gpio = ioremap(GPIO_BASE, 0x1000);
    ehci = ioremap(board->ehci_base, 0x10000);
	if (!phy || !host || !base || !complex || !gpio || !ehci) {
		result = -ENOMEM;
		goto out;
	}
	widgets(complex, widgets_before);
	states(phy, host, before);
	/* J72: both domains are off before the owner. Genpd owns any change. */
	if (!observe("HSIC PHY domain state", before[0] & 0xff, 0) ||
	    !observe("USB2HOST domain state", before[1] & 0xff, 0)) {
		result = -EBUSY;
		goto out;
	}
	pdev = platform_device_register_simple(DRV_NAME, -1, NULL, 0);
	if (IS_ERR(pdev)) {
		result = PTR_ERR(pdev);
		pdev = NULL;
		goto out;
	}
	spec.np = np;
	ret = of_genpd_add_device(&spec, &pdev->dev);
	if (ret) {
		result = ret;
		goto cleanup;
	}
	attached = true;
	pm_runtime_enable(&pdev->dev);
	pm_enabled = true;
	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret < 0) {
		result = ret;
		goto cleanup;
	}
	resumed = true;
	states(phy, host, active);
	widgets(complex, widgets_active);
	if (!domain_ready(active[0]) || !domain_ready(active[1])) {
		result = -EIO;
		goto cleanup;
	}
    result = platform_driver_register(&consumer_driver);
    if (result) goto cleanup;
    consumer_driver_registered = true;
	/* Only the control/config offsets used by the exact native port driver.
	 * Do not sweep the unknown +4/+12 words or any operational EHCI state.
	 */
	control = readl(base);
	config = readl(base + 8);
	observed = true;
    original_control = control;
    original_config = config;
    phy_restored = true;
    /* The native PHY transition below was qualified from this baseline. */
    if (!gate("PHY control", control, J72_PHY_CONTROL) ||
        !gate("PHY config", config, J72_PHY_CONFIG)) {
        result = -EPROTO;
        goto cleanup;
    }
    /* The usb_widget remap words decide where EHCI DMA lands. */
    for (i = 0; i < 4; i++) {
        if (!gate("usb_widget", widgets_before[i], J72_WIDGET) ||
            !gate("usb_widget active", widgets_active[i], J72_WIDGET)) {
            result = -EPROTO;
            goto cleanup;
        }
    }
    result = acquire_pmu();
    if (result) goto cleanup;
    host_ready_before = host_ready_reg(gpio);
    device_ready_before = readl(gpio + board->device_ready_gpio * 4);
    /* The reset write 0b is only known from the 09 baseline. */
    if (!gate("PMU GPIO3", pmu_before, J72_PMU_RESET) ||
        !observe("HOST_READY config", host_ready_before, J72_HOST_READY_CFG) ||
        !observe("DEVICE_READY config", device_ready_before, J72_DEVICE_READY_CFG)) {
        result = -EPROTO;
        goto cleanup;
    }
    /* Both lines must start low: HOST_READY is restored to this word. */
    if ((host_ready_before & 1) || (device_ready_before & 1)) {
        pr_warn(DRV_NAME ": %s HOST_READY/DEVICE_READY not low (%08x/%08x)\n",
                board->name, host_ready_before, device_ready_before);
        result = -EPROTO;
        goto cleanup;
    }
    reset_restored = true;
    host_restored = true;
    result = acquire_host_ready(&pdev->dev);
    if (host_ready) host_restored = host_ready_reg(gpio) == host_ready_before;
    if (result) goto cleanup;
    if (!host_restored) { result = -EPROTO; goto cleanup; }
    /* Exact native transition: no cfg property => leave config unchanged.
     * PHY settling precedes the bounded release below; no host registration.
     */
    phy_touched = true;
    phy_restored = false;
    writel(readl(base) | 3, base);
    writel(readl(base) & ~0x26U, base);
    udelay(20);
    writel(readl(base) & ~1U, base);
    udelay(1000);
    running_control = readl(base);
    result = (running_control == 0 && readl(base + 8) == original_config) ? 0 : -EIO;
    if (result) goto cleanup;
    for (i = 0; i < 3; i++) ehci_caps[i] = readl(ehci + i * 4);
    /* Operational registers are addressed at caplength 0x10, EHCI 1.0. */
    n_ports = HCS_N_PORTS(ehci_caps[1]);
    if (ehci_caps[0] != J72_EHCI_CAPLENGTH ||
        !observe("EHCI HCSPARAMS", ehci_caps[1], J72_EHCI_HCSPARAMS) ||
        !observe("EHCI HCCPARAMS", ehci_caps[2], J72_EHCI_HCCPARAMS) ||
        board->port < 1 || board->port > n_ports) {
        pr_warn(DRV_NAME ": %s EHCI caps %08x,%08x,%08x, J72 measured %08x,%08x,%08x\n",
                board->name, ehci_caps[0], ehci_caps[1], ehci_caps[2],
                J72_EHCI_CAPLENGTH, J72_EHCI_HCSPARAMS, J72_EHCI_HCCPARAMS);
        result = -EPROTO;
        goto cleanup;
    }
    ehci_state(ehci, ehci_initial);
    if ((ehci_initial[0] & 3) || !(ehci_initial[1] & 0x1000) || ehci_initial[2]) {
        result = -EBUSY;
        goto cleanup;
    }
    ehci_valid = true;
    pr_info(DRV_NAME ": phase prepare bound HCD\n");
    result = hcd_prepare(pdev, ehci);
    if (result) goto cleanup;
    /* Mark before attempting the write: an I2C error may follow a write. */
    reset_touched = true;
    reset_restored = false;
    /* Known full byte avoids consulting a potentially cached RMW value.
     * Exact native function yields0b from our required raw baseline09.
     */
    result = regmap_write(pmu_map, PMU_GPIO3, 0x0b);
    if (result) goto cleanup;
    result = regmap_read_bypassed(pmu_map, PMU_GPIO3, &pmu_released);
    if (result) goto cleanup;
    if (pmu_released != 0x0b) {
        result = -EIO;
        goto cleanup;
    }
    /* Compare low/high/low ready phases without registering a USB host.
     * Each window retains 50 samples; a low response is not a fault verdict.
     */
    result = ready_window(gpio, host_ready_before, &ready_samples, &ready_high_samples);
    if (result) goto cleanup;
    host_touched = true;
    host_pending = true;
    host_restored = false;
    result = gpiod_set_raw_value_cansleep(host_ready, 1);
    if (result) goto cleanup;
    host_high_cfg = host_ready_reg(gpio);
    if (host_high_cfg != (host_ready_before | 1)) { result = -EIO; goto cleanup; }
    result = ready_window(gpio, host_ready_before | 1, &high_window_samples, &high_window_ready);
    if (result) goto cleanup;
    pr_info(DRV_NAME ": %s DEVICE_READY high in %u/%u samples after HOST_READY (J72: 45/50)\n",
            board->name, high_window_ready, high_window_samples);
    result = hcd_enumerate(ehci);
    ehci_state(ehci, ehci_high);
    if (!result && keep_active) {
        if (!rom_identity_ok()) {
            result = -EPROTO;
            goto cleanup;
        }
        session_active = true;
        pr_info(DRV_NAME ": %s ROM %04x rev %u ram %#x remap %#x; HCD retained ACTIVE for opt-in firmware driver; root-only stop available\n",
                board->name, rom_words[0], rom_words[1], rom_words[2], rom_words[3]);
        return 0;
    }
cleanup:
    return cleanup_session();
out:
    return cleanup_session();
}

static int cleanup_session(void)
{
    int i;
    mutex_lock(&ready_lock);
    stopping = true;
    session_active = false;
    mutex_unlock(&ready_lock);
    if (!pdev) goto out;
    if (!hcd_finish(ehci, ehci_initial)) {
        result = -EIO;
        retain_controller = true;
        if (pdev && pm_enabled) pm_runtime_disable(&pdev->dev);
        goto out;
    }
    if (host_pending) {
        int host_ret = gpiod_set_raw_value_cansleep(host_ready, 0);
        host_after = host_ready_reg(gpio);
        host_restored = !host_ret && host_after == host_ready_before;
        if (!host_restored) {
            result = -EIO;
            /* One cleanup retry covers an ambiguous first GPIO write. */
            host_ret = gpiod_set_raw_value_cansleep(host_ready, 0);
            host_after = host_ready_reg(gpio);
            host_restored = !host_ret && host_after == host_ready_before;
        }
        if (host_restored) {
            int low_ret = ready_window(gpio, host_ready_before, &low_window_samples, &low_window_ready);
            if (low_ret) result = low_ret;
            ehci_state(ehci, ehci_low);
        }
    }
    if (host_ready && !host_restored) {
        result = -EIO;
        retain_controller = true;
        goto out;
    }
    if (reset_touched) {
        int reset_ret = regmap_write(pmu_map, PMU_GPIO3, pmu_before);
        int read_ret = regmap_read_bypassed(pmu_map, PMU_GPIO3, &pmu_after);
        reset_restored = !reset_ret && !read_ret && pmu_after == pmu_before;
        if (!reset_restored) {
            result = -EIO;
            /* Keep PHY and domain owner intact if radio reset is uncertain. */
            pm_runtime_disable(&pdev->dev);
            retain_controller = true;
            goto out;
        }
    }
    if (ehci_valid) ehci_state(ehci, ehci_reset_state);
    if (phy_touched) {
        /* Native OR7 is an off policy, not our measured iBoot baseline4. */
        writel(original_control, base);
        phy_restored = readl(base) == original_control && readl(base + 8) == original_config;
        if (!phy_restored) {
            result = -EIO;
            pm_runtime_disable(&pdev->dev);
            retain_controller = true;
            goto out;
        }
    }
	if (resumed) {
		suspend_result = pm_runtime_put_sync_suspend(&pdev->dev);
		if (suspend_result < 0) {
			if (!result)
				result = suspend_result;
			/* Keep the failed consumer attached, with no async retries.
			 * Do not discard a still-active domain owner on error.
			 */
			pm_runtime_disable(&pdev->dev);
            retain_controller = true;
			goto out;
		}
	}
	if (pm_enabled)
		pm_runtime_disable(&pdev->dev);
	if (attached) {
		detach_result = pm_genpd_remove_device(&pdev->dev);
		if (detach_result) {
			/* Preserve a still-attached device if detaching fails. */
			if (!result)
				result = detach_result;
            retain_controller = true;
			goto out;
		}
	}
    if (pdev) platform_device_unregister(pdev);
    pdev = NULL;
    consumer_removed = true;
	for (i = 0; i < 200; i++) {
		states(phy, host, after);
		if (same_states(before, after))
			break;
		usleep_range(500, 1000);
	}
	restored = same_states(before, after);
	widgets(complex, widgets_after);
	if (!restored && !result)
		result = -EIO;
out:
    if (retain_controller) {
        pr_err(DRV_NAME ": halted/restoration check failed; retaining DMA, GPIO, PHY and domain owners until reboot\n");
        return 0;
    }
    if (consumer_driver_registered && (!pdev || consumer_removed)) {
        platform_driver_unregister(&consumer_driver);
        consumer_driver_registered = false;
    }
    if (host_ready && host_restored) { gpiod_put(host_ready); host_ready = NULL; }
    if (lookup_installed) { gpiod_remove_lookup_table(&ready_lookup); lookup_installed = false; }
    if (ready_gdev && !host_ready) { gpio_device_put(ready_gdev); ready_gdev = NULL; }
    gpio_released = !host_ready && !ready_gdev && !lookup_installed;
    if (pmic) { put_device(pmic); pmic = NULL; pmu_map = NULL; }
	pr_info(DRV_NAME ": %s result=%d observed=%u restored=%u removed=%u\n",
		board ? board->name : "none", result, observed, restored, consumer_removed);
	if (base)
		iounmap(base);
	if (complex)
		iounmap(complex);
    if (gpio) {
        iounmap(gpio);
    }
    if (ehci) {
        iounmap(ehci);
    }
	if (host)
		iounmap(host);
	if (phy)
		iounmap(phy);
	of_node_put(parent);
	of_node_put(host_np);
	of_node_put(np);
	/* Keep diagnostic status available on hardware failure. No retry API. */
	return 0;
}
static void __exit observer_exit(void) { }
module_init(observer_init);
module_exit(observer_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple A7 HSIC Wi-Fi owner with synchronized stop and strict ROM gate");
