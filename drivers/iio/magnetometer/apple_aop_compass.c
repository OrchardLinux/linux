// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Magnetometer served by the Apple Always-On Processor (AOP)
 *
 * The A11's AOP firmware samples the compass (an Alps HSCDTD601A) itself.
 * Once it has its batch, raw and report intervals (EPIC set-property tags
 * 0x16, 0x5c and 0, which its sensor base class handles as for the
 * gyroscope), it sends one report (EPIC subtype 0xc3) per report interval,
 * built by the same routine as the gyroscope's and laid out the same way:
 *
 *   0  u16 sequence
 *   2  u32 status
 *   6  s32 x, y, z: the field less the firmware's offsets, in 1/65536 uT
 *  18  s32 zero
 *
 * Before that it takes the unit's factory calibration, as iOS sends it:
 * compass-calibration as it is (tag 0x0b), compass-orientation (0x18: a
 * version word and a 3x3 matrix of s8, one row per word) and
 * compass-sens-calibration (0x1d: a version word and a 3x3 sensitivity
 * matrix). iOS rebuilds the last two from a version 3 record holding one
 * entry per location ID; version 2 records, as on the iPhone 8 Plus, go to
 * the firmware unchanged, and so does any record here. Without the
 * sensitivity matrix the firmware reports the field uncorrected.
 */

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/aop.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#define AOP_PROP_REPORT_INTERVAL	0x00
#define AOP_PROP_BATCH_INTERVAL		0x16
#define AOP_PROP_RAW_INTERVAL		0x5c

#define AOP_REPORT_COMPASS		0xc3
#define AOP_COMPASS_LEN			22
#define AOP_COMPASS_XYZ			6

#define AOP_COMPASS_DEFAULT_US		100000	/* 10 Hz */
#define AOP_COMPASS_MIN_US		10000	/* 100 Hz */
#define AOP_COMPASS_MAX_US		1000000	/* 1 Hz */

/* 1/65536 microtesla = 1/6553600 gauss */
#define AOP_COMPASS_SCALE_DIV		6553600

struct apple_aop_compass {
	struct device *dev;
	const struct apple_aop_service *service;
	struct iio_mount_matrix orientation;

	struct mutex lock; /* interval */
	u32 interval_us;

	spinlock_t sample_lock; /* sample, have_sample */
	s32 sample[3];
	bool have_sample;

	IIO_DECLARE_BUFFER_WITH_TS(s32, scan, 3);
};

static int apple_aop_compass_set_u32(struct apple_aop_compass *compass, u32 key, u32 value)
{
	__le32 le = cpu_to_le32(value);

	return apple_aop_set_property(compass->service, key, &le, sizeof(le));
}

/* No separate raw sample stream: raw interval 0. */
static int apple_aop_compass_set_interval(struct apple_aop_compass *compass, u32 interval_us)
{
	int ret;

	ret = apple_aop_compass_set_u32(compass, AOP_PROP_BATCH_INTERVAL, interval_us);
	if (!ret)
		ret = apple_aop_compass_set_u32(compass, AOP_PROP_RAW_INTERVAL, 0);
	if (!ret)
		ret = apple_aop_compass_set_u32(compass, AOP_PROP_REPORT_INTERVAL, interval_us);
	if (!ret)
		compass->interval_us = interval_us;
	return ret;
}

static void apple_aop_compass_report(void *cookie, u16 subtype, const void *data, size_t len)
{
	struct iio_dev *indio_dev = cookie;
	struct apple_aop_compass *compass = iio_priv(indio_dev);
	unsigned long flags;
	int i;

	if (subtype != AOP_REPORT_COMPASS || len < AOP_COMPASS_LEN)
		return;

	spin_lock_irqsave(&compass->sample_lock, flags);
	for (i = 0; i < 3; i++)
		compass->sample[i] = get_unaligned_le32(data + AOP_COMPASS_XYZ + 4 * i);
	compass->have_sample = true;
	spin_unlock_irqrestore(&compass->sample_lock, flags);

	if (iio_buffer_enabled(indio_dev)) {
		memcpy(compass->scan, compass->sample, sizeof(compass->sample));
		iio_push_to_buffers_with_ts(indio_dev, compass->scan, sizeof(compass->scan),
					    iio_get_time_ns(indio_dev));
	}
}

static int apple_aop_compass_read_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2, long mask)
{
	struct apple_aop_compass *compass = iio_priv(indio_dev);
	unsigned long flags;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		spin_lock_irqsave(&compass->sample_lock, flags);
		*val = compass->sample[chan->scan_index];
		ret = compass->have_sample ? IIO_VAL_INT : -EAGAIN;
		spin_unlock_irqrestore(&compass->sample_lock, flags);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = AOP_COMPASS_SCALE_DIV;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&compass->lock);
		*val = USEC_PER_SEC / compass->interval_us;
		*val2 = (u64)(USEC_PER_SEC % compass->interval_us) * USEC_PER_SEC /
			compass->interval_us;
		mutex_unlock(&compass->lock);
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int apple_aop_compass_write_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int val, int val2, long mask)
{
	struct apple_aop_compass *compass = iio_priv(indio_dev);
	u64 micro_hz;
	u32 interval;
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val < 0 || val2 < 0)
		return -EINVAL;

	micro_hz = (u64)val * USEC_PER_SEC + val2;
	if (!micro_hz)
		return -EINVAL;
	interval = clamp_t(u64, div64_u64((u64)USEC_PER_SEC * USEC_PER_SEC, micro_hz),
			   AOP_COMPASS_MIN_US, AOP_COMPASS_MAX_US);

	mutex_lock(&compass->lock);
	ret = apple_aop_compass_set_interval(compass, interval);
	mutex_unlock(&compass->lock);
	return ret;
}

static const struct iio_mount_matrix *
apple_aop_compass_get_mount_matrix(const struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct apple_aop_compass *compass = iio_priv(indio_dev);

	return &compass->orientation;
}

static const struct iio_chan_spec_ext_info apple_aop_compass_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, apple_aop_compass_get_mount_matrix),
	{ }
};

#define APPLE_AOP_COMPASS_CHANNEL(axis, index) {			\
	.type = IIO_MAGN,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_CPU,					\
	},								\
	.ext_info = apple_aop_compass_ext_info,				\
}

/* Firmware reports always contain all three axes. */
static const unsigned long apple_aop_compass_scan_masks[] = { GENMASK(2, 0), 0 };

static const struct iio_chan_spec apple_aop_compass_channels[] = {
	APPLE_AOP_COMPASS_CHANNEL(X, 0),
	APPLE_AOP_COMPASS_CHANNEL(Y, 1),
	APPLE_AOP_COMPASS_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_info apple_aop_compass_info = {
	.read_raw = apple_aop_compass_read_raw,
	.write_raw = apple_aop_compass_write_raw,
};

/* iOS marks the calibration complete once the orientation is applied */
static const struct apple_aop_calibration apple_aop_compass_calibration[] = {
	{ "apple,compass-calibration", 0x0b, 0 },
	{ "apple,compass-orientation", 0x18, 1 },
	{ "apple,compass-sens-calibration", 0x1d, 0 },
};

static void apple_aop_compass_stop(void *data)
{
	struct apple_aop_compass *compass = data;

	apple_aop_compass_set_u32(compass, AOP_PROP_REPORT_INTERVAL, 0);
	apple_aop_unlisten(compass->service);
}

static int apple_aop_compass_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct apple_aop_service *service = dev_get_platdata(dev);
	struct apple_aop_compass *compass;
	struct iio_dev *indio_dev;
	int ret;

	if (!service)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*compass));
	if (!indio_dev)
		return -ENOMEM;
	compass = iio_priv(indio_dev);
	compass->dev = dev;
	compass->service = service;
	mutex_init(&compass->lock);
	spin_lock_init(&compass->sample_lock);

	ret = iio_read_mount_matrix(dev, &compass->orientation);
	if (ret)
		return ret;

	indio_dev->name = "apple-aop-compass";
	indio_dev->info = &apple_aop_compass_info;
	indio_dev->available_scan_masks = apple_aop_compass_scan_masks;
	indio_dev->channels = apple_aop_compass_channels;
	indio_dev->num_channels = ARRAY_SIZE(apple_aop_compass_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev, NULL);
	if (ret)
		return ret;

	ret = apple_aop_listen(service, apple_aop_compass_report, indio_dev);
	if (ret)
		return ret;

	ret = apple_aop_calibrate(service, dev, apple_aop_compass_calibration,
				  ARRAY_SIZE(apple_aop_compass_calibration));
	if (ret)
		dev_warn(dev, "calibration state not set: %d\n", ret);

	ret = apple_aop_compass_set_interval(compass, AOP_COMPASS_DEFAULT_US);
	if (ret) {
		apple_aop_unlisten(service);
		return dev_err_probe(dev, ret, "failed to start reports\n");
	}

	ret = devm_add_action_or_reset(dev, apple_aop_compass_stop, compass);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id apple_aop_compass_of_match[] = {
	{ .compatible = "apple,t8015-aop-compass" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_aop_compass_of_match);

static struct platform_driver apple_aop_compass_driver = {
	.driver = {
		.name = "apple-aop-compass",
		.of_match_table = apple_aop_compass_of_match,
	},
	.probe = apple_aop_compass_probe,
};
module_platform_driver(apple_aop_compass_driver);

MODULE_DESCRIPTION("Apple AOP magnetometer driver");
MODULE_LICENSE("Dual MIT/GPL");
