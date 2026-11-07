// SPDX-License-Identifier: GPL-2.0-only
/* Exact J72/7.3 pw1 native PHY/reset/HSIC transport owner.
 * GPIO3 PMIC bit1:09->0b->09. HOST_READY uses a standard exclusive gpiod
 * consumer claim, retains direction/mux, and returns low before reset/PHY stop.
 * Standard EHCI enumeration and strict ROM query; optional persistent owner for separately loaded firmware driver.
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
#include <linux/utsname.h>
#include <linux/dma-direct.h>
#include <linux/of_device.h>
#include <linux/irqdomain.h>
#include <linux/ioport.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "ehci.h"

#define DOMAIN "/soc/power-management@20e000000/power-controller@20120"
#define HOST "/soc/power-management@20e000000/power-controller@20170"
#define PHY_BASE 0x20c000050ULL
#define PHY_STATE 0x20e020120ULL
#define HOST_STATE 0x20e020170ULL
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
 .driver = { .name = "j72-wlan-transport-v3", .owner = THIS_MODULE },
};

#include "j72-wlan-hcd.h"

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
 .dev_id = "j72-wlan-transport-v3",
 .table = {
  { .key = NULL, .chip_hwnum = 163, .con_id = "host-ready", .idx = 0, .flags = GPIO_ACTIVE_HIGH },
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
 gpiod_add_lookup_table(&ready_lookup);
 lookup_installed = true;
 host_ready = gpiod_get(consumer, "host-ready", GPIOD_ASIS);
 if (IS_ERR(host_ready)) { ret = PTR_ERR(host_ready); host_ready = NULL; return ret; }
 ret = gpiod_get_direction(host_ready);
 return ret < 0 ? ret : (ret == 0 ? 0 : -EPROTO);
}

static int ready_window(void __iomem *gpio, u32 expected_host, u32 *samples, u32 *high)
{
 unsigned int i;
 for (i = 0; i < 50; i++) {
  last_device_ready = readl(gpio + 164 * 4);
  (*samples)++;
  if (last_device_ready & 1) (*high)++;
  if ((last_device_ready & ~1U) != device_ready_before ||
      readl(gpio + 163 * 4) != expected_host) return -EIO;
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
 return regmap_read_bypassed(pmu_map, 0x403, &pmu_before);
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

static int status_get(char *buf, const struct kernel_param *unused)
{
	return scnprintf(buf, PAGE_SIZE,
		"result=%d\nobserved=%u\nrestored=%u\nconsumer_removed=%u\n"
        "phy_touched=%u\nphy_restored=%u\ncontrol_before=%08x\ncontrol_running=%08x\n"
        "reset_touched=%u\nreset_restored=%u\npmu_before=%02x\npmu_released=%02x\npmu_after=%02x\n"
        "host_ready_before=%08x\ndevice_ready_before=%08x\nready_samples=%u\nready_high_samples=%u\nlast_device_ready=%08x\n"
        "host_touched=%u\nhost_restored=%u\ngpio_released=%u\nhost_high_cfg=%08x\nhost_gpio_after=%08x\n"
        "high_window_samples=%u\nhigh_window_ready=%u\nlow_window_samples=%u\nlow_window_ready=%u\n"
        "dma_checked=%u\nhcd_attempted=%u\nhcd_stopped=%u\ncontroller_restored=%u\nretain_controller=%u\n"
        "enumerate_result=%d\nradio_vid=%04x\nradio_pid=%04x\nradio_speed=%u\nrom_result=%d\n"
        "rom_identity=%08x,%08x,%08x,%08x,%08x,%08x\n"
        "saved_ports=%08x,%08x,%08x\nlive_ports=%08x,%08x,%08x\n"
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
		result, observed, restored, consumer_removed, phy_touched, phy_restored, original_control, running_control,
        reset_touched, reset_restored, pmu_before, pmu_released, pmu_after,
        host_ready_before, device_ready_before, ready_samples, ready_high_samples, last_device_ready,
        host_touched, host_restored, gpio_released, host_high_cfg, host_after,
        high_window_samples, high_window_ready, low_window_samples, low_window_ready,
        dma_checked, hcd_attempted, hcd_stopped, controller_restored, retain_controller,
        enumerate_result, radio_vid, radio_pid, radio_speed, rom_result,
        rom_words[0],rom_words[1],rom_words[2],rom_words[3],rom_words[4],rom_words[5],
        saved_ports[0],saved_ports[1],saved_ports[2],live_ports[0],live_ports[1],live_ports[2],
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

int j72_wlan_transport_ready(struct usb_device *udev, bool cycle);
int j72_wlan_transport_ready(struct usb_device *udev, bool cycle)
{
 int ret = -ENODEV, high_ret;
 mutex_lock(&ready_lock);
 if (!session_active || stopping || !host_ready || !radio_hcd || !hcd_added ||
     !udev || udev->bus != &radio_hcd->self ||
     udev->parent != radio_hcd->self.root_hub || udev->portnum != 3 ||
     le16_to_cpu(udev->descriptor.idVendor) != 0x0a5c ||
     le16_to_cpu(udev->descriptor.idProduct) != 0xbd1c ||
     udev->speed != USB_SPEED_HIGH) goto out;
 ret = 0;
 if (cycle) {
  ret = gpiod_set_raw_value_cansleep(host_ready, 0);
  if (!ret && readl(gpio + 163 * 4) != host_ready_before) ret = -EIO;
  msleep(20);
 }
 /* Always attempt HIGH after a possibly ambiguous LOW write. */
 high_ret = gpiod_set_raw_value_cansleep(host_ready, 1);
 if (!ret) ret = high_ret;
 if (!ret && readl(gpio + 163 * 4) != (host_ready_before | 1)) ret = -EIO;
 msleep(20);
out:
 mutex_unlock(&ready_lock);
 return ret;
}
EXPORT_SYMBOL_GPL(j72_wlan_transport_ready);

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

static int __init observer_init(void)
{
 int ret, i;

	if (!confirm_native)
        return -EPERM;
    if (!of_machine_is_compatible("apple,j72") ||
	    strcmp(utsname()->release, "7.3.0-rc1-j72-pw1-orchard-apple-4k"))
		return -ENODEV;
	np = of_find_node_by_path(DOMAIN);
	host_np = of_find_node_by_path(HOST);
	if (!np || !host_np || !of_device_is_compatible(np, "apple,pmgr-pwrstate") ||
	    np->parent != host_np->parent ||
	    of_property_read_u32_array(np, "reg", provider_reg, 2) ||
	    of_property_read_u32_array(host_np, "reg", host_reg, 2) ||
	    provider_reg[0] != 0x20120 || provider_reg[1] != 4 ||
	    host_reg[0] != 0x20170 || host_reg[1] != 4 ||
	    of_address_to_resource(np->parent, 0, &resource) ||
	    resource.start != 0x20e000000ULL || resource_size(&resource) < 0x20174) {
		result = -ENODEV;
		goto out;
	}
	parent = of_parse_phandle(np, "power-domains", 0);
	if (parent != host_np) {
		result = -ENODEV;
		goto out;
	}
	phy = ioremap(PHY_STATE, 4);
	host = ioremap(HOST_STATE, 4);
	base = ioremap(PHY_BASE, 0x10);
	complex = ioremap(0x20c900000ULL, 0x80);
    gpio = ioremap(0x20e300000ULL, 0x1000);
    ehci = ioremap(0x20c200000ULL, 0x10000);
	if (!phy || !host || !base || !complex || !gpio || !ehci) {
		result = -ENOMEM;
		goto out;
	}
	widgets(complex, widgets_before);
	states(phy, host, before);
	if ((before[0] & 0xff) || (before[1] & 0xff)) {
		result = -EBUSY;
		goto out;
	}
	pdev = platform_device_register_simple("j72-wlan-transport-v3", -1, NULL, 0);
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
    if (control != 4 || config != 0x153) {
        result = -EPROTO;
        goto cleanup;
    }
    for (i = 0; i < 4; i++) {
        if (widgets_before[i] != 0x108 || widgets_active[i] != 0x108) {
            result = -EPROTO;
            goto cleanup;
        }
    }
    result = acquire_pmu();
    if (result) goto cleanup;
    host_ready_before = readl(gpio + 163 * 4);
    device_ready_before = readl(gpio + 164 * 4);
    if (pmu_before != 0x09 || host_ready_before != 0x00072602 ||
        device_ready_before != 0x00070200) {
        result = -EPROTO;
        goto cleanup;
    }
    reset_restored = true;
    host_restored = true;
    result = acquire_host_ready(&pdev->dev);
    if (host_ready) host_restored = readl(gpio + 163 * 4) == host_ready_before;
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
    if (ehci_caps[0] != 0x01000010 || ehci_caps[1] != 0x1313 || ehci_caps[2] != 0x2a026) {
        result = -EPROTO;
        goto cleanup;
    }
    ehci_state(ehci, ehci_initial);
    if ((ehci_initial[0] & 3) || !(ehci_initial[1] & 0x1000) || ehci_initial[2]) {
        result = -EBUSY;
        goto cleanup;
    }
    ehci_valid = true;
    pr_info("j72-wlan-transport-v3: phase prepare bound HCD\n");
    result = hcd_prepare(pdev, ehci);
    if (result) goto cleanup;
    /* Mark before attempting the write: an I2C error may follow a write. */
    reset_touched = true;
    reset_restored = false;
    /* Known full byte avoids consulting a potentially cached RMW value.
     * Exact native function yields0b from our required raw baseline09.
     */
    result = regmap_write(pmu_map, 0x403, 0x0b);
    if (result) goto cleanup;
    result = regmap_read_bypassed(pmu_map, 0x403, &pmu_released);
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
    host_high_cfg = readl(gpio + 163 * 4);
    if (host_high_cfg != (host_ready_before | 1)) { result = -EIO; goto cleanup; }
    result = ready_window(gpio, host_ready_before | 1, &high_window_samples, &high_window_ready);
    if (result) goto cleanup;
    result = hcd_enumerate(ehci);
    ehci_state(ehci, ehci_high);
    if (!result && keep_active) {
        if (rom_result || radio_vid != 0x0a5c || radio_pid != 0xbd1c ||
            rom_words[0] != 0x4324 || rom_words[1] != 6 ||
            rom_words[2] != 0x90000 || rom_words[3] != 0x60000000) {
            result = -EPROTO;
            goto cleanup;
        }
        session_active = true;
        pr_info("j72-wlan-transport-v3: exact ROM verified; HCD retained ACTIVE for opt-in firmware driver; root-only stop available\n");
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
        host_after = readl(gpio + 163 * 4);
        host_restored = !host_ret && host_after == host_ready_before;
        if (!host_restored) {
            result = -EIO;
            /* One cleanup retry covers an ambiguous first GPIO write. */
            host_ret = gpiod_set_raw_value_cansleep(host_ready, 0);
            host_after = readl(gpio + 163 * 4);
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
        int reset_ret = regmap_write(pmu_map, 0x403, pmu_before);
        int read_ret = regmap_read_bypassed(pmu_map, 0x403, &pmu_after);
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
        pr_err("j72_wlan_transport_v3: halted/restoration check failed; retaining DMA, GPIO, PHY and domain owners until reboot\n");
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
	pr_info("j72_wlan_transport_v3: result=%d observed=%u restored=%u removed=%u\n",
		result, observed, restored, consumer_removed);
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
MODULE_DESCRIPTION("J72 opt-in persistent HSIC owner with synchronized stop and strict ROM gate");
