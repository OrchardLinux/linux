// SPDX-License-Identifier: GPL-2.0-only
/*
 * Panel power over system sleep on Apple A12 phones whose display the boot
 * loader set up (Synopsys DesignWare MIPI DSI host, 1.32a on the iPhone XR).
 *
 * The sequence is iOS's (ApplePinotLCD::_lcdEnable and
 * AppleSynopsysMIPIDSIController in the iOS 17.7 kernelcache for iPhone11,8):
 * on sleep DCS display off and sleep in while video still runs, 134 ms, the
 * high-speed clock off, then panel reset asserted and its rail off. iOS never
 * wakes the panel from a soft sleep in: wake is the rail on, 10 ms, reset
 * released, 7 ms, DCS sleep out in low-power command mode, the high-speed
 * clock on, 100 ms, DCS display on, then video mode. No vendor commands.
 *
 * The display pipe's timing generator (disp0 0x48024, bit 31) is stopped
 * once the panel is off and started again right after the high-speed clock
 * comes back, 100 ms before display on (UnifiedPipeline::
 * enable_timings_gated); without that, video starts mid-frame and the
 * picture is garbled. The host is soft reset (PWR_UP) over the sleep as on
 * iOS, which also rewrites all its registers from the boot loader's copy and
 * shuts the PHY down; here the PHY stays up in low-power stop state, which
 * needs no PLL relock. Wake runs in the background so resume does not wait
 * the ~120 ms.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define DSI_PWR_UP		0x04
#define DSI_MODE_CFG		0x34
#define  MODE_CMD		BIT(0)
#define DSI_GEN_HDR		0x6c
#define DSI_CMD_PKT_STATUS	0x74
#define  GEN_CMD_EMPTY		BIT(0)
#define DSI_LPCLK_CTRL		0x94
#define  LPCLK_HS_REQ		BIT(0)

#define DISP_TG_CTRL		0x24	/* disp0 reg[0] + 0x48000 */
#define  TG_ENABLE		BIT(31)
#define  TG_MODE		BIT(29)
#define  TG_LATCH		BIT(15)

#define DCS_SHORT_WRITE		0x05
#define DCS_SLEEP_IN		0x10
#define DCS_SLEEP_OUT		0x11
#define DCS_DISPLAY_OFF		0x28
#define DCS_DISPLAY_ON		0x29

struct apple_dsi_pm {
	struct device *dev;
	void __iomem *base;
	void __iomem *tg;
	struct gpio_desc *reset;
	struct regulator *lcd;
	int reset_released;	/* level the boot loader left: panel running */
	bool off;
	struct work_struct wake;
	struct mutex lock;
};

static int apple_dsi_dcs(struct apple_dsi_pm *p, u8 cmd)
{
	u32 st;
	int ret;

	writel((cmd << 8) | DCS_SHORT_WRITE, p->base + DSI_GEN_HDR);
	ret = readl_poll_timeout(p->base + DSI_CMD_PKT_STATUS, st,
				 st & GEN_CMD_EMPTY, 10, 20000);
	if (ret)
		dev_warn(p->dev, "DCS %#04x not sent (status %#x)\n", cmd, st);
	return ret;
}

static void apple_dsi_panel_off(struct apple_dsi_pm *p)
{
	guard(mutex)(&p->lock);
	if (p->off)
		return;
	apple_dsi_dcs(p, DCS_DISPLAY_OFF);
	apple_dsi_dcs(p, DCS_SLEEP_IN);
	msleep(134);
	writel(readl(p->base + DSI_LPCLK_CTRL) & ~LPCLK_HS_REQ,
	       p->base + DSI_LPCLK_CTRL);
	writel(readl(p->base + DSI_MODE_CFG) | MODE_CMD, p->base + DSI_MODE_CFG);
	writel((readl(p->tg + DISP_TG_CTRL) & ~TG_ENABLE) | TG_LATCH,
	       p->tg + DISP_TG_CTRL);
	msleep(20);	/* a frame; iOS polls a pipe status for the stop */
	writel(0, p->base + DSI_PWR_UP);
	gpiod_set_value_cansleep(p->reset, !p->reset_released);
	regulator_disable(p->lcd);
	p->off = true;
}

static void apple_dsi_panel_on(struct apple_dsi_pm *p)
{
	int ret;

	guard(mutex)(&p->lock);
	if (!p->off)
		return;
	ret = regulator_enable(p->lcd);
	if (ret) {
		dev_err(p->dev, "panel rail not on: %d\n", ret);
		return;
	}
	writel(1, p->base + DSI_PWR_UP);
	msleep(10);
	gpiod_set_value_cansleep(p->reset, p->reset_released);
	msleep(7);
	apple_dsi_dcs(p, DCS_SLEEP_OUT);
	udelay(12);
	writel(readl(p->base + DSI_LPCLK_CTRL) | LPCLK_HS_REQ,
	       p->base + DSI_LPCLK_CTRL);
	writel(readl(p->tg + DISP_TG_CTRL) | TG_ENABLE | TG_MODE | TG_LATCH,
	       p->tg + DISP_TG_CTRL);
	msleep(100);
	apple_dsi_dcs(p, DCS_DISPLAY_ON);
	writel(readl(p->base + DSI_MODE_CFG) & ~MODE_CMD, p->base + DSI_MODE_CFG);
	p->off = false;
}

static void apple_dsi_wake_work(struct work_struct *work)
{
	apple_dsi_panel_on(container_of(work, struct apple_dsi_pm, wake));
}

static int apple_dsi_suspend(struct device *dev)
{
	struct apple_dsi_pm *p = dev_get_drvdata(dev);

	flush_work(&p->wake);
	apple_dsi_panel_off(p);
	return 0;
}

static int apple_dsi_resume(struct device *dev)
{
	struct apple_dsi_pm *p = dev_get_drvdata(dev);

	queue_work(system_dfl_wq, &p->wake);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_dsi_pm_ops, apple_dsi_suspend,
				apple_dsi_resume);

/* For bring-up: write 1 to switch the panel off as for sleep, 0 to wake it. */
static ssize_t panel_off_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t n)
{
	struct apple_dsi_pm *p = dev_get_drvdata(dev);
	bool off;
	int ret;

	ret = kstrtobool(buf, &off);
	if (ret)
		return ret;
	if (off)
		apple_dsi_panel_off(p);
	else
		apple_dsi_panel_on(p);
	return n;
}

static ssize_t panel_off_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct apple_dsi_pm *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", p->off);
}
static DEVICE_ATTR_RW(panel_off);

static struct attribute *apple_dsi_attrs[] = {
	&dev_attr_panel_off.attr,
	NULL
};
ATTRIBUTE_GROUPS(apple_dsi);

static void apple_dsi_wake_cancel(void *data)
{
	struct apple_dsi_pm *p = data;

	cancel_work_sync(&p->wake);
	apple_dsi_panel_on(p);
}

static int apple_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_dsi_pm *p;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	mutex_init(&p->lock);
	INIT_WORK(&p->wake, apple_dsi_wake_work);

	p->base = devm_platform_ioremap_resource_byname(pdev, "dsi");
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	p->tg = devm_platform_ioremap_resource_byname(pdev, "timing");
	if (IS_ERR(p->tg))
		return PTR_ERR(p->tg);

	/* Leave the line as the boot loader set it, and learn which level that is. */
	p->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(p->reset))
		return dev_err_probe(dev, PTR_ERR(p->reset), "no panel reset\n");
	ret = gpiod_get_value_cansleep(p->reset);
	if (ret < 0)
		return dev_err_probe(dev, ret, "panel reset unreadable\n");
	p->reset_released = ret;
	ret = gpiod_direction_output(p->reset, p->reset_released);
	if (ret)
		return ret;

	p->lcd = devm_regulator_get(dev, "lcd");
	if (IS_ERR(p->lcd))
		return dev_err_probe(dev, PTR_ERR(p->lcd), "no panel rail\n");
	/* The panel is running: claim the rail so it is not switched off as unused. */
	ret = regulator_enable(p->lcd);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, p);
	ret = devm_add_action_or_reset(dev, apple_dsi_wake_cancel, p);
	if (ret)
		return ret;

	dev_info(dev, "panel sleep over suspend; reset released at level %d\n",
		 p->reset_released);
	return 0;
}

static const struct of_device_id apple_dsi_of_match[] = {
	{ .compatible = "apple,t8020-mipi-dsi" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_dsi_of_match);

static struct platform_driver apple_dsi_driver = {
	.probe = apple_dsi_probe,
	.driver = {
		.name = "apple-mipi-dsi-panel-pm",
		/* Unbinding would drop the rail the running panel needs. */
		.suppress_bind_attrs = true,
		.of_match_table = apple_dsi_of_match,
		.pm = pm_sleep_ptr(&apple_dsi_pm_ops),
		.dev_groups = apple_dsi_groups,
	},
};
module_platform_driver(apple_dsi_driver);

MODULE_DESCRIPTION("Apple MIPI DSI panel power over system sleep");
MODULE_LICENSE("GPL");
