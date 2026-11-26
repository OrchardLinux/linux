// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Mesa (Touch ID) sensor
 *
 * The sensor sits on its own SPI bus with two supplies the SMC switches.
 * Doing anything useful with it needs firmware that only the Secure
 * Enclave hands out, so this driver just checks that the sensor answers
 * from its boot ROM and then switches it off, which keeps it from drawing
 * power while Linux runs.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

/*
 * Get status: four command bytes, three bytes of padding, then the
 * 16-byte reply, all in one full-duplex transfer.
 */
#define MESA_STATUS_LEN		23
#define MESA_STATUS_REPLY	7

/* Power sequencing delays from Apple's device tree, in milliseconds */
#define MESA_ON_1V8_TO_3V0_MS	3
#define MESA_ON_SETTLE_MS	20
#define MESA_OFF_3V0_TO_1V8_MS	4
#define MESA_OFF_SETTLE_MS	6

struct apple_mesa {
	struct spi_device *spi;
	struct regulator *vdd18;
	struct regulator *vdd30;
};

static int apple_mesa_power_on(struct apple_mesa *mesa)
{
	int ret;

	ret = regulator_enable(mesa->vdd18);
	if (ret)
		return ret;
	msleep(MESA_ON_1V8_TO_3V0_MS);

	ret = regulator_enable(mesa->vdd30);
	if (ret) {
		regulator_disable(mesa->vdd18);
		return ret;
	}
	msleep(MESA_ON_SETTLE_MS);

	return 0;
}

static void apple_mesa_power_off(struct apple_mesa *mesa)
{
	regulator_disable(mesa->vdd30);
	msleep(MESA_OFF_3V0_TO_1V8_MS);
	regulator_disable(mesa->vdd18);
	msleep(MESA_OFF_SETTLE_MS);
}

static int apple_mesa_identify(struct apple_mesa *mesa)
{
	static const u8 cmd[] = { 0x80, 0x10, 0x00, 0x07 };
	struct device *dev = &mesa->spi->dev;
	struct spi_transfer xfer = { };
	const u8 *reply;
	u8 *buf;
	int ret;

	buf = kzalloc(2 * MESA_STATUS_LEN, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy(buf, cmd, sizeof(cmd));

	xfer.tx_buf = buf;
	xfer.rx_buf = buf + MESA_STATUS_LEN;
	xfer.len = MESA_STATUS_LEN;
	ret = spi_sync_transfer(mesa->spi, &xfer, 1);
	if (ret)
		goto out;

	reply = buf + MESA_STATUS_LEN + MESA_STATUS_REPLY;
	if (reply[0] != 0x80 || reply[1] != 0x20 || reply[3] != 0x10) {
		dev_err(dev, "no sensor status: %*ph\n", 16, reply);
		ret = -ENODEV;
		goto out;
	}

	dev_info(dev, "Touch ID sensor %04x%s\n", reply[12] | reply[13] << 8,
		 reply[7] ? "" : ", in boot ROM");
out:
	kfree(buf);
	return ret;
}

static int apple_mesa_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_mesa *mesa;
	int ret;

	mesa = devm_kzalloc(dev, sizeof(*mesa), GFP_KERNEL);
	if (!mesa)
		return -ENOMEM;
	mesa->spi = spi;

	mesa->vdd18 = devm_regulator_get(dev, "vdd18");
	if (IS_ERR(mesa->vdd18))
		return dev_err_probe(dev, PTR_ERR(mesa->vdd18), "no 1.8 V supply\n");

	mesa->vdd30 = devm_regulator_get(dev, "vdd30");
	if (IS_ERR(mesa->vdd30))
		return dev_err_probe(dev, PTR_ERR(mesa->vdd30), "no 3 V supply\n");

	ret = apple_mesa_power_on(mesa);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	ret = apple_mesa_identify(mesa);
	apple_mesa_power_off(mesa);

	return ret;
}

static const struct of_device_id apple_mesa_of_match[] = {
	{ .compatible = "apple,mesa" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_mesa_of_match);

static const struct spi_device_id apple_mesa_spi_id[] = {
	{ "mesa" },
	{ }
};
MODULE_DEVICE_TABLE(spi, apple_mesa_spi_id);

static struct spi_driver apple_mesa_driver = {
	.driver = {
		.name = "apple-mesa",
		.of_match_table = apple_mesa_of_match,
	},
	.id_table = apple_mesa_spi_id,
	.probe = apple_mesa_probe,
};
module_spi_driver(apple_mesa_driver);

MODULE_DESCRIPTION("Apple Mesa (Touch ID) sensor power-off driver");
MODULE_LICENSE("Dual MIT/GPL");
