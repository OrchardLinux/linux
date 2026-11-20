// SPDX-License-Identifier: GPL-2.0-only
/* CT709 four-channel ambient light sensor. */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include "ams-ct709.h"
#include "ams-ct709-lux.h"

struct ct709_data {
	struct i2c_client *client;
	struct mutex lock;
	struct ct709_lux_cal calibration;
	u64 deadline;
	bool stopping;
	bool faulted;
};
static int ct709_read(void *ctx, u8 reg, u8 *value)
{
	struct ct709_data *d = ctx;
	int ret = i2c_smbus_read_byte_data(d->client, reg);

	if (ret < 0)
		return ret;
	*value = ret;
	return 0;
}
static int ct709_write(void *ctx, u8 reg, u8 value)
{
	struct ct709_data *d = ctx;

	if (!((reg == 0x80 && (value == 0 || value == 1 || value == 3)) ||
	      (reg == 0x81 && (value == 0xff || value == 0xf6)) ||
	      (reg == 0x8f && (value == 0 || value == 2))))
		return -EPERM;
	return i2c_smbus_write_byte_data(d->client, reg, value);
}
static int ct709_block(void *ctx, u8 reg, u8 *bytes, unsigned int count)
{
	struct ct709_data *d = ctx;
	struct i2c_msg messages[] = {
	    {.addr = d->client->addr, .len = 1, .buf = &reg},
	    {.addr = d->client->addr,
	     .flags = I2C_M_RD,
	     .len = 8,
	     .buf = bytes},
	};
	int ret;

	if (reg != 0x94 || count != 8)
		return -EINVAL;
	ret = i2c_transfer(d->client->adapter, messages, ARRAY_SIZE(messages));
	return ret < 0 ? ret : ret == ARRAY_SIZE(messages) ? 0 : -EIO;
}
static int ct709_delay(void *ctx, unsigned int us)
{
	(void)ctx;
	usleep_range(us, us + 500);
	return 0;
}
static bool ct709_expired(void *ctx)
{
	struct ct709_data *d = ctx;

	return ktime_get_ns() >= d->deadline;
}
static const struct als_ops ct709_ops = {
	.read = ct709_read,
	.write = ct709_write,
	.block = ct709_block,
	.delay = ct709_delay,
	.expired = ct709_expired,
};
static int ct709_read_raw(struct iio_dev *indio,
			  const struct iio_chan_spec *channel, int *value,
			  int *value2, long mask)
{
	struct ct709_data *d = iio_priv(indio);
	struct als_result result;
	int ret;

	if (mask == IIO_CHAN_INFO_HARDWAREGAIN) {
		*value = 16;
		return IIO_VAL_INT;
	}
	if (mask == IIO_CHAN_INFO_INT_TIME) {
		*value = 0;
		*value2 = 27800;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	if (mask != IIO_CHAN_INFO_PROCESSED &&
	    (mask != IIO_CHAN_INFO_RAW || channel->address >= 4))
		return -EINVAL;
	mutex_lock(&d->lock);
	if (d->stopping || d->faulted) {
		ret = -ESHUTDOWN;
		goto out;
	}
	d->deadline = ktime_get_ns() + 5ULL * NSEC_PER_SEC;
	ret = als_once(&ct709_ops, d, &result);
	if (ret == -EOPNOTSUPP && !result.changed)
		goto out;
	if (ret || !result.valid || !result.restored) {
		d->faulted = true;
		if (!ret)
			ret = -EIO;
		dev_err(&d->client->dev,
			"capture failed %d, restore %d, verified %u\n", ret,
			result.restore_error, result.restored);
		goto out;
	}
	if (mask == IIO_CHAN_INFO_PROCESSED) {
		int millilux;

		ret = ct709_millilux(&d->calibration, result.channels, &millilux);
		if (ret)
			goto out;
		*value = millilux / 1000;
		*value2 = (millilux % 1000) * 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		goto out;
	}
	*value = result.channels[channel->address];
	ret = IIO_VAL_INT;
out:
	mutex_unlock(&d->lock);
	return ret;
}
#define CT709_CHANNEL(color, index)                                            \
	{                                                                      \
	    .type = IIO_INTENSITY,                                             \
	    .modified = 1,                                                     \
	    .channel2 = IIO_MOD_LIGHT_##color,                                 \
	    .address = index,                                                  \
	    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                      \
	    .info_mask_shared_by_type =                                        \
		BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_INT_TIME), \
	}
static const struct iio_chan_spec ct709_channels[] = {
	CT709_CHANNEL(CLEAR, 0),
	CT709_CHANNEL(RED, 1),
	CT709_CHANNEL(GREEN, 2),
	CT709_CHANNEL(BLUE, 3),
	{ .type = IIO_LIGHT, .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) },
};
static const struct iio_info ct709_info = {.read_raw = ct709_read_raw};
/* J120 stores both orientations in the factory record on the als0 node. */
static bool ct709_read_calibration(struct i2c_client *client,
				   struct ct709_lux_cal *cal)
{
	struct device_node *owner;
	const u8 *blob, *record;
	unsigned int orientation, other, i;
	int len;

	if (!client->dev.of_node || !of_machine_is_compatible("apple,j120"))
		return false;
	owner = of_find_node_by_path("als0");
	if (!owner)
		return false;
	if (!of_device_is_compatible(owner, "apple,ct709") ||
	    owner->parent != client->dev.of_node->parent)
		goto out;
	blob = of_get_property(owner, "apple,als-calibration", &len);
	if (!blob || len < 6696 || memcmp(blob, "\x02\x01\x02\x08", 4) ||
	    get_unaligned_le16(blob + 4) != 6696)
		goto out;
	if (client->addr != 0x29 && client->addr != 0x39)
		goto out;
	orientation = client->addr == 0x29 ? 6 : 5;
	other = orientation == 6 ? 5 : 6;
	record = NULL;
	for (i = 0; i < 2; i++) {
		const u8 *p = blob + 8 + 3344 * i;

		if (p[0] != 2 || p[1] != 1 || p[3] != 1 ||
		    get_unaligned_le32(p + 4) != 3344 || p[0x68] != 3)
			goto out;
		if (p[2] == orientation && !record)
			record = p;
		else if (p[2] != other)
			goto out;
	}
	if (record && !ct709_lux_parse(record, 3344, cal)) {
		of_node_put(owner);
		return true;
	}
out:
	of_node_put(owner);
	return false;
}
static int ct709_probe(struct i2c_client *client)
{
	struct iio_dev *indio;
	struct ct709_data *d;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;
	ret = i2c_smbus_read_byte_data(client, 0x92);
	if (ret < 0)
		return ret;
	if (ret != 0x8f)
		return -ENODEV;
	ret = i2c_smbus_read_byte_data(client, 0x91);
	if (ret < 0)
		return ret;
	if (ret != 0x31)
		return -ENODEV;
	indio = devm_iio_device_alloc(&client->dev, sizeof(*d));
	if (!indio)
		return -ENOMEM;
	d = iio_priv(indio);
	d->client = client;
	mutex_init(&d->lock);
	i2c_set_clientdata(client, indio);
	indio->name = "ct709";
	indio->info = &ct709_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = ct709_channels;
	indio->num_channels = ARRAY_SIZE(ct709_channels) - 1;
	if (ct709_read_calibration(client, &d->calibration))
		indio->num_channels++;
	return devm_iio_device_register(&client->dev, indio);
}
static void ct709_remove(struct i2c_client *client)
{
	struct ct709_data *d = iio_priv(i2c_get_clientdata(client));

	mutex_lock(&d->lock);
	d->stopping = true;
	mutex_unlock(&d->lock);
}
static const struct of_device_id ct709_of_match[] = {
	{.compatible = "apple,ct709"}, {}};
MODULE_DEVICE_TABLE(of, ct709_of_match);
static const struct i2c_device_id ct709_ids[] = {{"ct709"}, {}};
MODULE_DEVICE_TABLE(i2c, ct709_ids);
static struct i2c_driver ct709_driver = {
	.driver = {.name = "ct709", .of_match_table = ct709_of_match},
	.probe = ct709_probe,
	.remove = ct709_remove,
	.id_table = ct709_ids,
};
module_i2c_driver(ct709_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple CT709 ambient light sensor");
