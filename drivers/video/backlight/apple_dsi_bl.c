// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Brightness of the OLED panels of Apple iPhones over the MIPI DSI host.
 *
 * The OLED iPhones (iPhone X) have no DWI backlight controller: the panel
 * takes the MIPI DCS "set display brightness" command (0x51) with an 11-bit
 * display brightness value (DBV), as iOS sends it. The SoC's Synopsys
 * DesignWare MIPI DSI host (version 1.31a on the A11) runs the display in
 * video mode with low-power commands enabled, so a generic long write
 * through its command FIFO reaches the panel between frames. Only the
 * command registers are used; the host itself stays as iBoot set it up.
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <video/mipi_display.h>

#define DSI_VERSION			0x00
#define DSI_GEN_HDR			0x6c
#define DSI_GEN_HDR_DT			GENMASK(5, 0)
#define DSI_GEN_HDR_WC			GENMASK(23, 8)
#define DSI_GEN_PLD_DATA		0x70
#define DSI_CMD_PKT_STATUS		0x74
#define DSI_GEN_CMD_EMPTY		BIT(0)
#define DSI_GEN_CMD_FULL		BIT(1)
#define DSI_GEN_PLD_W_FULL		BIT(3)

#define DSI_BL_MAX_DBV			2047U
/* A short command goes out within a frame (16.7 ms at 60 Hz). */
#define DSI_BL_TIMEOUT_US		50000

struct apple_dsi_bl {
	void __iomem *base;
	u32 min_dbv;
};

static int dsi_bl_update_status(struct backlight_device *bl)
{
	struct apple_dsi_bl *dsi_bl = bl_get_data(bl);
	u32 dbv = backlight_get_brightness(bl);
	u32 status;
	int ret;

	if (!backlight_is_blank(bl))
		dbv = max(dbv, dsi_bl->min_dbv);

	ret = readl_poll_timeout(dsi_bl->base + DSI_CMD_PKT_STATUS, status,
				 !(status & (DSI_GEN_CMD_FULL | DSI_GEN_PLD_W_FULL)),
				 100, DSI_BL_TIMEOUT_US);
	if (ret) {
		dev_err(&bl->dev, "DSI command FIFO full: %#x\n", status);
		return ret;
	}

	/*
	 * 51, DBV[7:0], DBV[10:8]: the panel takes the low byte first. The
	 * payload FIFO sends the word's bytes from the lowest.
	 */
	writel(MIPI_DCS_SET_DISPLAY_BRIGHTNESS | (dbv & 0xff) << 8 | (dbv >> 8) << 16,
	       dsi_bl->base + DSI_GEN_PLD_DATA);
	writel(FIELD_PREP(DSI_GEN_HDR_DT, MIPI_DSI_DCS_LONG_WRITE) |
	       FIELD_PREP(DSI_GEN_HDR_WC, 3), dsi_bl->base + DSI_GEN_HDR);

	ret = readl_poll_timeout(dsi_bl->base + DSI_CMD_PKT_STATUS, status,
				 status & DSI_GEN_CMD_EMPTY, 100, DSI_BL_TIMEOUT_US);
	if (ret)
		dev_err(&bl->dev, "DSI brightness command not sent: %#x\n", status);
	return ret;
}

static const struct backlight_ops dsi_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = dsi_bl_update_status,
};

static int dsi_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = { };
	struct apple_dsi_bl *dsi_bl;
	struct backlight_device *bl;
	u32 max = DSI_BL_MAX_DBV, level;
	u32 version;

	dsi_bl = devm_kzalloc(dev, sizeof(*dsi_bl), GFP_KERNEL);
	if (!dsi_bl)
		return -ENOMEM;
	dsi_bl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi_bl->base))
		return PTR_ERR(dsi_bl->base);

	/* iBoot powers the host up for the boot display; nothing works otherwise. */
	version = readl(dsi_bl->base + DSI_VERSION);
	if (!version || version == U32_MAX)
		return dev_err_probe(dev, -ENODEV, "DSI host not powered (%#x)\n", version);

	props.type = BACKLIGHT_RAW;
	props.scale = BACKLIGHT_SCALE_NON_LINEAR;
	device_property_read_u32(dev, "max-brightness", &max);
	max = min(max, DSI_BL_MAX_DBV);
	device_property_read_u32(dev, "apple,min-dbv", &dsi_bl->min_dbv);
	dsi_bl->min_dbv = min(dsi_bl->min_dbv, max);
	level = max / 2;
	device_property_read_u32(dev, "default-brightness-level", &level);
	props.max_brightness = max;
	props.brightness = min(level, max);

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, dsi_bl,
					    &dsi_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	platform_set_drvdata(pdev, bl);

	/* The panel's level is not readable; set the boot level we report. */
	return backlight_update_status(bl);
}

static const struct of_device_id dsi_bl_of_match[] = {
	{ .compatible = "apple,dsi-bl" },
	{ }
};
MODULE_DEVICE_TABLE(of, dsi_bl_of_match);

static struct platform_driver dsi_bl_driver = {
	.driver = {
		.name = "apple-dsi-bl",
		.of_match_table = dsi_bl_of_match,
	},
	.probe = dsi_bl_probe,
};
module_platform_driver(dsi_bl_driver);

MODULE_DESCRIPTION("Apple OLED panel brightness over the MIPI DSI host");
MODULE_LICENSE("Dual MIT/GPL");
