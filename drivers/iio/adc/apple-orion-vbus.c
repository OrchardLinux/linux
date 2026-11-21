// SPDX-License-Identifier: GPL-2.0-only
/* Lightning VBUS ADC on the Apple Orion Bellatrix. */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>

#define ORION_CONTROL		0x00
#define ORION_STATUS		0x0c
#define ORION_READY		0x17
#define ORION_ADC_COMMAND	0xa0
#define ORION_ADC_STATUS	0xa2
#define ORION_ADC_LOW		0xa4
#define ORION_ADC_HIGH		0xa5
#define ORION_CONTROL_IDLE	0x81
#define ORION_CONTROL_ADC	(ORION_CONTROL_IDLE | BIT(4))
#define ORION_ADC_VBUS		2
#define ORION_ADC_DONE		BIT(0)

struct orion_vbus {
	struct i2c_client *client;
	struct mutex lock;
	bool restore_failed;
};

static int orion_restore(struct i2c_client *client)
{
	int ret, state;

	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state < 0)
		return state;
	if (state == ORION_CONTROL_IDLE)
		return 0;
	if (state != ORION_CONTROL_ADC)
		return -EIO;

	ret = i2c_smbus_write_byte_data(client, ORION_CONTROL, ORION_CONTROL_IDLE);
	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state == ORION_CONTROL_IDLE)
		return ret;
	return state < 0 ? state : -EIO;
}

static int orion_convert(struct i2c_client *client)
{
	int state, ret, low, high, attempt;

	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state != ORION_CONTROL_ADC)
		return state < 0 ? state : -EIO;
	ret = i2c_smbus_read_byte_data(client, ORION_ADC_STATUS);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, ORION_ADC_COMMAND, ORION_ADC_VBUS);
	if (ret < 0)
		return ret;

	for (attempt = 0; ; attempt++) {
		ret = i2c_smbus_read_byte_data(client, ORION_ADC_STATUS);
		if (ret < 0)
			return ret;
		if (ret & ORION_ADC_DONE)
			break;
		if (attempt == 5)
			return -ETIMEDOUT;
		msleep(10);
	}

	low = i2c_smbus_read_byte_data(client, ORION_ADC_LOW);
	if (low < 0)
		return low;
	high = i2c_smbus_read_byte_data(client, ORION_ADC_HIGH);
	if (high < 0)
		return high;
	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state != ORION_CONTROL_ADC)
		return state < 0 ? state : -EIO;

	ret = low | ((high & 3) << 8);
	return ret == 1023 ? -ERANGE : ret;
}

static int orion_sample(struct orion_vbus *vbus)
{
	struct i2c_client *client = vbus->client;
	int state, ret, restore;

	if (vbus->restore_failed)
		return -EIO;
	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state != ORION_CONTROL_IDLE)
		return state < 0 ? state : -EBUSY;
	state = i2c_smbus_read_byte_data(client, ORION_STATUS);
	if (state < 0)
		return state;
	if (!(state & BIT(2))) {
		state = i2c_smbus_read_byte_data(client, ORION_READY);
		if (state < 0)
			return state;
		if (state & (BIT(1) | BIT(0)))
			return -EBUSY;
	}
	state = i2c_smbus_read_byte_data(client, ORION_CONTROL);
	if (state != ORION_CONTROL_IDLE)
		return state < 0 ? state : -EBUSY;

	ret = i2c_smbus_write_byte_data(client, ORION_CONTROL, ORION_CONTROL_ADC);
	if (!ret)
		ret = orion_convert(client);
	restore = orion_restore(client);
	if (restore) {
		vbus->restore_failed = true;
		dev_err(&client->dev, "ADC control restore failed: %d\n", restore);
		return restore;
	}

	return ret;
}

static int orion_vbus_read_raw(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       int *val, int *val2, long mask)
{
	struct orion_vbus *vbus = iio_priv(indio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_PROCESSED)
		return -EINVAL;

	mutex_lock(&vbus->lock);
	ret = orion_sample(vbus);
	mutex_unlock(&vbus->lock);
	if (ret < 0)
		return ret;

	/* Orion Bellatrix ADC2 is scaled in 16 mV steps. */
	*val = ret * 16;
	return IIO_VAL_INT;
}

static const struct iio_info orion_vbus_info = {
	.read_raw = orion_vbus_read_raw,
};

static const struct iio_chan_spec orion_vbus_channel = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.datasheet_name = "vbus",
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
};

static int orion_vbus_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct orion_vbus *vbus;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*vbus));
	if (!indio_dev)
		return -ENOMEM;

	vbus = iio_priv(indio_dev);
	vbus->client = client;
	mutex_init(&vbus->lock);
	indio_dev->name = "orion-vbus";
	indio_dev->info = &orion_vbus_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &orion_vbus_channel;
	indio_dev->num_channels = 1;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id orion_vbus_of_match[] = {
	{ .compatible = "apple,orion-bellatrix-vbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, orion_vbus_of_match);

static struct i2c_driver orion_vbus_driver = {
	.driver = {
		.name = "apple-orion-vbus",
		.of_match_table = orion_vbus_of_match,
	},
	.probe = orion_vbus_probe,
};
module_i2c_driver(orion_vbus_driver);

MODULE_DESCRIPTION("Apple Orion Bellatrix Lightning VBUS ADC");
MODULE_LICENSE("GPL");
