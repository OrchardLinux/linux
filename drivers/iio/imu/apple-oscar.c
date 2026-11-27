// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple M7 motion coprocessor ("Oscar") on the A7 iPads
 *
 * The accelerometer, gyroscope and compass of the A7 iPads sit behind the
 * M7, an NXP LPC18xx-class microcontroller on UART4 (Apple's
 * /arm-io/uart4/oscar). It has no flash for its firmware: its boot ROM
 * takes CoreMotionOscarFirmware.image over the UART at every power-up.
 *
 * Boot: three supplies on (core, I/O, sensors, in that order), reset
 * released; the ROM sends '?', takes the whole image and answers "OK\r\n";
 * the firmware then sends a ready report.
 *
 * Packets ("ICPA"), little endian: "ICPA", u8 type, u8 total length (0 means
 * 256), u32 options, u16 checksum, u16 sequence number, payload. The checksum
 * makes the 16-bit sum of the packet's little-endian words 0xffff.
 * Commands: 0x04 set property [dev u8][prop u32][len u8][value],
 * 0x0a get property [dev u8][prop u32]. Replies are the command type | 0x80
 * with the command's sequence number and a payload of u32 status plus data.
 * 0xba reports a packet the M7 could not read, echoing what it received.
 * Reports: 0xc0 ready, 0xd0 log text, 0xc1 gyroscope, 0xc2 accelerometer,
 * 0xc3 compass. A sensor report holds 32-byte records: u8 kind, u8 length,
 * u64 time (us), u16 sequence, u32 subtype, s32 x, y, z as 16.16 fixed
 * point (g, degrees per second, microtesla), s32 temperature (16.16 degrees
 * Celsius, 0x7fffffff when absent). Units are inferred from the J71's
 * data: 1 g at rest, a 0.07 degree/s step (L3GD20H-class gyroscope at
 * 2000 degrees/s) and a 25-30 C temperature word. Property 0 is the report interval in
 * microseconds (0 stops the sensor), property 0x16 the batch interval.
 *
 * All of this is from iOS 12.5.8's AppleOscar and the firmware, and was
 * checked on a J71. The M7 drops the tail of packets sent back to back, so
 * commands go out in short pieces.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define OSCAR_BAUD		1500000
#define OSCAR_HDR		14
#define OSCAR_MAX_PKT		256
#define OSCAR_MAX_PAYLOAD	0x72	/* iOS's limit for a command */
#define OSCAR_TX_PIECE		8

#define OSCAR_CMD_SET_PROP	0x04
#define OSCAR_CMD_GET_PROP	0x0a
#define OSCAR_REP_BAD_PACKET	0xba
#define OSCAR_REP_READY		0xc0
#define OSCAR_REP_GYRO		0xc1
#define OSCAR_REP_ACCEL		0xc2
#define OSCAR_REP_COMPASS	0xc3
#define OSCAR_REP_LOG		0xd0

#define OSCAR_DEV_GYRO		0
#define OSCAR_DEV_ACCEL		1
#define OSCAR_DEV_COMPASS	2
#define OSCAR_DEV_SYSTEM	3

#define OSCAR_PROP_INTERVAL	0x00
#define OSCAR_PROP_BUILD	0x14
#define OSCAR_PROP_BATCH	0x16

#define OSCAR_REC_LEN		32
#define OSCAR_TEMP_NONE		0x7fffffff

#define OSCAR_IDLE_MS		3000

enum oscar_state {
	OSCAR_OFF,
	OSCAR_ROM_WAIT,		/* waiting for the boot ROM's '?' */
	OSCAR_ROM_LOAD,		/* image sent, waiting for "OK\r\n" */
	OSCAR_RUN,
};

enum oscar_kind {
	OSCAR_ACCEL,
	OSCAR_GYRO,
	OSCAR_MAGN,
	OSCAR_NUM_SENSORS,
};

struct oscar;

struct oscar_sensor {
	struct oscar *oscar;
	struct iio_dev *indio_dev;
	struct mutex lock;		/* streaming state */
	u8 dev;
	u8 report;
	u32 interval_us;
	bool streaming;
	bool fresh;
	unsigned long last_use;
	s32 xyz[3];
	s32 temp;
	struct iio_mount_matrix orientation;
};

struct oscar {
	struct serdev_device *serdev;
	struct device *dev;
	struct regulator *core, *io, *sensors;
	struct gpio_desc *reset;
	bool core_on, io_on, sensors_on;
	const char *fw_name;

	spinlock_t lock;		/* rx state, samples */
	enum oscar_state state;
	u8 rx[2 * OSCAR_MAX_PKT];
	size_t rx_len;
	u8 line[4];			/* boot ROM answer window */

	struct completion rom_ready, rom_done, fw_ready, cmd_done;
	bool rom_ok;

	struct mutex cmd_lock;		/* one command at a time */
	u16 seq;
	u16 cmd_seq;
	u8 cmd_reply_type;
	bool cmd_failed;
	u32 cmd_status;
	u8 cmd_data[OSCAR_MAX_PKT];
	size_t cmd_data_len;

	wait_queue_head_t sample_wq;
	struct work_struct boot_work;
	struct delayed_work idle_work;
	bool registered;

	struct oscar_sensor sensor[OSCAR_NUM_SENSORS];
};

static u16 oscar_checksum(const u8 *buf, size_t len)
{
	u32 sum = 0;
	size_t i;

	for (i = 0; i + 1 < len; i += 2)
		sum += get_unaligned_le16(buf + i);
	if (i < len)
		sum += buf[i];
	return ~sum & 0xffff;
}

/* ---- receive side, called from the serdev receive path ---- */

static void oscar_sensor_report(struct oscar *o, struct oscar_sensor *s, const u8 *p, size_t len)
{
	size_t off;

	for (off = 0; off + OSCAR_REC_LEN <= len; ) {
		const u8 *r = p + off;
		u8 rec_len = r[1];

		if (r[0] <= 1) {
			s->xyz[0] = get_unaligned_le32(r + 0x10);
			s->xyz[1] = get_unaligned_le32(r + 0x14);
			s->xyz[2] = get_unaligned_le32(r + 0x18);
			s->temp = get_unaligned_le32(r + 0x1c);
			s->fresh = true;
		}
		if (rec_len < 10)
			break;
		off += rec_len;
	}
}

static void oscar_packet(struct oscar *o, const u8 *pkt, size_t len)
{
	u8 type = pkt[4];
	u16 seq = get_unaligned_le16(pkt + 12);
	const u8 *payload = pkt + OSCAR_HDR;
	size_t plen = len - OSCAR_HDR;
	int i;

	switch (type) {
	case OSCAR_REP_READY:
		complete(&o->fw_ready);
		return;
	case OSCAR_REP_LOG:
		dev_dbg(o->dev, "M7: %.*s\n", (int)plen, payload);
		return;
	case OSCAR_REP_BAD_PACKET:
		dev_dbg(o->dev, "M7 rejected a packet\n");
		if (o->cmd_seq) {
			o->cmd_failed = true;
			complete(&o->cmd_done);
		}
		return;
	}

	for (i = 0; i < OSCAR_NUM_SENSORS; i++) {
		if (type == o->sensor[i].report) {
			oscar_sensor_report(o, &o->sensor[i], payload, plen);
			wake_up(&o->sample_wq);
			return;
		}
	}

	if (type >= 0x80 && type < 0xc0 && o->cmd_seq && seq == o->cmd_seq &&
	    type == o->cmd_reply_type && plen >= 4) {
		o->cmd_status = get_unaligned_le32(payload);
		o->cmd_data_len = plen - 4;
		memcpy(o->cmd_data, payload + 4, plen - 4);
		o->cmd_failed = false;
		complete(&o->cmd_done);
	}
}

static void oscar_parse(struct oscar *o)
{
	size_t i, len;

	for (;;) {
		for (i = 0; i + 4 <= o->rx_len; i++)
			if (!memcmp(o->rx + i, "ICPA", 4))
				break;
		if (i + 4 > o->rx_len) {
			/* keep a possible start of the magic */
			i = o->rx_len > 3 ? o->rx_len - 3 : 0;
		}
		if (i) {
			memmove(o->rx, o->rx + i, o->rx_len - i);
			o->rx_len -= i;
		}
		if (o->rx_len < OSCAR_HDR)
			return;
		len = o->rx[5] ? o->rx[5] : 256;
		if (len < OSCAR_HDR) {
			memmove(o->rx, o->rx + 1, --o->rx_len);
			continue;
		}
		if (o->rx_len < len)
			return;
		if (oscar_checksum(o->rx, len)) {
			dev_dbg_ratelimited(o->dev, "packet with a bad checksum\n");
			memmove(o->rx, o->rx + 1, --o->rx_len);
			continue;
		}
		oscar_packet(o, o->rx, len);
		memmove(o->rx, o->rx + len, o->rx_len - len);
		o->rx_len -= len;
	}
}

static size_t oscar_receive_buf(struct serdev_device *serdev, const u8 *data, size_t count)
{
	struct oscar *o = serdev_device_get_drvdata(serdev);
	unsigned long flags;
	size_t i = 0, n;

	spin_lock_irqsave(&o->lock, flags);
	switch (o->state) {
	case OSCAR_OFF:
		i = count;
		break;
	case OSCAR_ROM_WAIT:
		if (memchr(data, '?', count))
			complete(&o->rom_ready);
		i = count;
		break;
	case OSCAR_ROM_LOAD:
		/* the two bytes before "\r\n" are the answer: "OK" or not */
		for (; i < count; i++) {
			memmove(o->line, o->line + 1, 3);
			o->line[3] = data[i];
			if (o->line[2] == '\r' && o->line[3] == '\n') {
				o->rom_ok = o->line[0] == 'O' && o->line[1] == 'K';
				o->state = OSCAR_RUN;
				o->rx_len = 0;
				complete(&o->rom_done);
				i++;
				break;
			}
		}
		if (o->state != OSCAR_RUN)
			break;
		fallthrough;
	case OSCAR_RUN:
		while (i < count) {
			n = min(count - i, sizeof(o->rx) - o->rx_len);
			memcpy(o->rx + o->rx_len, data + i, n);
			o->rx_len += n;
			i += n;
			oscar_parse(o);
			if (o->rx_len == sizeof(o->rx))
				o->rx_len = 0;	/* cannot happen with valid lengths */
		}
		break;
	}
	spin_unlock_irqrestore(&o->lock, flags);

	return count;
}

static const struct serdev_device_ops oscar_serdev_ops = {
	.receive_buf = oscar_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/* ---- commands ---- */

static int oscar_send(struct oscar *o, const u8 *buf, size_t len)
{
	size_t off, n;
	ssize_t ret;

	for (off = 0; off < len; off += n) {
		n = min_t(size_t, len - off, OSCAR_TX_PIECE);
		ret = serdev_device_write(o->serdev, buf + off, n, msecs_to_jiffies(100));
		if (ret < 0)
			return ret;
		serdev_device_wait_until_sent(o->serdev, msecs_to_jiffies(10));
		usleep_range(200, 300);
	}
	return 0;
}

static int oscar_command(struct oscar *o, u8 type, const u8 *payload, size_t len,
			 u8 *reply, size_t *reply_len)
{
	u8 pkt[OSCAR_HDR + OSCAR_MAX_PAYLOAD];
	unsigned long flags;
	int tries, ret = -ETIMEDOUT;

	if (len > OSCAR_MAX_PAYLOAD)
		return -EINVAL;

	mutex_lock(&o->cmd_lock);
	for (tries = 0; tries < 3; tries++) {
		spin_lock_irqsave(&o->lock, flags);
		if (++o->seq == 0)
			o->seq = 1;
		o->cmd_seq = o->seq;
		o->cmd_reply_type = type | 0x80;
		o->cmd_failed = false;
		reinit_completion(&o->cmd_done);
		spin_unlock_irqrestore(&o->lock, flags);

		memcpy(pkt, "ICPA", 4);
		pkt[4] = type;
		pkt[5] = OSCAR_HDR + len;
		memset(pkt + 6, 0, 6);
		put_unaligned_le16(o->cmd_seq, pkt + 12);
		memcpy(pkt + OSCAR_HDR, payload, len);
		put_unaligned_le16(oscar_checksum(pkt, OSCAR_HDR + len), pkt + 10);

		ret = oscar_send(o, pkt, OSCAR_HDR + len);
		if (ret)
			break;
		if (!wait_for_completion_timeout(&o->cmd_done, msecs_to_jiffies(500))) {
			ret = -ETIMEDOUT;
			continue;
		}
		spin_lock_irqsave(&o->lock, flags);
		if (o->cmd_failed) {
			ret = -EIO;
		} else if (o->cmd_status) {
			ret = -EREMOTEIO;
		} else {
			ret = 0;
			if (reply && reply_len) {
				*reply_len = min(*reply_len, o->cmd_data_len);
				memcpy(reply, o->cmd_data, *reply_len);
			}
		}
		spin_unlock_irqrestore(&o->lock, flags);
		if (ret != -EIO)
			break;
	}
	spin_lock_irqsave(&o->lock, flags);
	o->cmd_seq = 0;
	spin_unlock_irqrestore(&o->lock, flags);
	mutex_unlock(&o->cmd_lock);

	if (ret)
		dev_dbg(o->dev, "command 0x%02x failed: %d (status 0x%x)\n",
			type, ret, o->cmd_status);
	return ret;
}

static int oscar_set_prop(struct oscar *o, u8 dev, u32 prop, u32 value)
{
	u8 p[10];

	p[0] = dev;
	put_unaligned_le32(prop, p + 1);
	p[5] = 4;
	put_unaligned_le32(value, p + 6);
	return oscar_command(o, OSCAR_CMD_SET_PROP, p, sizeof(p), NULL, NULL);
}

static int oscar_get_prop(struct oscar *o, u8 dev, u32 prop, u8 *val, size_t *len)
{
	u8 p[5], reply[OSCAR_MAX_PKT];
	size_t n = sizeof(reply);
	int ret;

	p[0] = dev;
	put_unaligned_le32(prop, p + 1);
	ret = oscar_command(o, OSCAR_CMD_GET_PROP, p, sizeof(p), reply, &n);
	if (ret)
		return ret;
	if (!n || reply[0] + 1 > n)
		return -EPROTO;
	*len = min_t(size_t, *len, reply[0]);
	memcpy(val, reply + 1, *len);
	return 0;
}

/* ---- streaming on demand ---- */

static int oscar_stream(struct oscar_sensor *s, bool on)
{
	struct oscar *o = s->oscar;
	int ret;

	if (s->streaming == on)
		return 0;
	if (on) {
		ret = oscar_set_prop(o, s->dev, OSCAR_PROP_BATCH, 0);
		if (ret)
			return ret;
	}
	ret = oscar_set_prop(o, s->dev, OSCAR_PROP_INTERVAL, on ? s->interval_us : 0);
	if (ret)
		return ret;
	s->streaming = on;
	if (on)
		s->fresh = false;
	return 0;
}

static void oscar_idle_work(struct work_struct *work)
{
	struct oscar *o = container_of(to_delayed_work(work), struct oscar, idle_work);
	bool active = false;
	int i;

	for (i = 0; i < OSCAR_NUM_SENSORS; i++) {
		struct oscar_sensor *s = &o->sensor[i];

		mutex_lock(&s->lock);
		if (s->streaming) {
			if (time_after(jiffies, s->last_use + msecs_to_jiffies(OSCAR_IDLE_MS)))
				oscar_stream(s, false);
			else
				active = true;
		}
		mutex_unlock(&s->lock);
	}
	if (active)
		schedule_delayed_work(&o->idle_work, msecs_to_jiffies(OSCAR_IDLE_MS));
}

static int oscar_sample(struct oscar_sensor *s, int axis, int *val)
{
	struct oscar *o = s->oscar;
	unsigned long flags;
	int ret;

	if (o->state != OSCAR_RUN)
		return -EAGAIN;

	mutex_lock(&s->lock);
	s->last_use = jiffies;
	ret = oscar_stream(s, true);
	mutex_unlock(&s->lock);
	if (ret)
		return ret;
	schedule_delayed_work(&o->idle_work, msecs_to_jiffies(OSCAR_IDLE_MS));

	ret = wait_event_interruptible_timeout(o->sample_wq, READ_ONCE(s->fresh),
					       msecs_to_jiffies(500));
	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;

	spin_lock_irqsave(&o->lock, flags);
	*val = axis < 3 ? s->xyz[axis] : s->temp;
	spin_unlock_irqrestore(&o->lock, flags);
	if (axis == 3 && *val == OSCAR_TEMP_NONE)
		return -ENODATA;
	return IIO_VAL_INT;
}

/* ---- IIO ---- */

static const struct iio_mount_matrix *
oscar_get_mount_matrix(const struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	struct oscar_sensor *s = *(struct oscar_sensor **)iio_priv(indio_dev);

	return &s->orientation;
}

static const struct iio_chan_spec_ext_info oscar_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, oscar_get_mount_matrix),
	{ }
};

#define OSCAR_AXIS(_type, _axis, _index, _ext) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.address = _index,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = _ext,						\
}

#define OSCAR_TEMP {							\
	.type = IIO_TEMP,						\
	.address = 3,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
}

static const struct iio_chan_spec oscar_accel_channels[] = {
	OSCAR_AXIS(IIO_ACCEL, X, 0, oscar_ext_info),
	OSCAR_AXIS(IIO_ACCEL, Y, 1, oscar_ext_info),
	OSCAR_AXIS(IIO_ACCEL, Z, 2, oscar_ext_info),
};

static const struct iio_chan_spec oscar_gyro_channels[] = {
	OSCAR_AXIS(IIO_ANGL_VEL, X, 0, NULL),
	OSCAR_AXIS(IIO_ANGL_VEL, Y, 1, NULL),
	OSCAR_AXIS(IIO_ANGL_VEL, Z, 2, NULL),
	OSCAR_TEMP,
};

static const struct iio_chan_spec oscar_magn_channels[] = {
	OSCAR_AXIS(IIO_MAGN, X, 0, NULL),
	OSCAR_AXIS(IIO_MAGN, Y, 1, NULL),
	OSCAR_AXIS(IIO_MAGN, Z, 2, NULL),
	OSCAR_TEMP,
};

static int oscar_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct oscar_sensor *s = *(struct oscar_sensor **)iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return oscar_sample(s, chan->address, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:		/* 1/65536 g in m/s^2 */
			*val = 0;
			*val2 = 149637;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ANGL_VEL:	/* 1/65536 degree/s in rad/s */
			*val = 1;
			*val2 = 3754936;
			return IIO_VAL_FRACTIONAL;
		case IIO_MAGN:		/* 1/65536 uT in gauss */
			*val = 1;
			*val2 = 6553600;
			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:		/* 1/65536 degree C in millidegrees */
			*val = 1000;
			*val2 = 65536;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_info oscar_iio_info = {
	.read_raw = oscar_read_raw,
};

static int oscar_register(struct oscar *o)
{
	static const struct {
		const char *name;
		const struct iio_chan_spec *channels;
		int num_channels;
	} kinds[OSCAR_NUM_SENSORS] = {
		[OSCAR_ACCEL] = { "apple-oscar-accel", oscar_accel_channels,
				  ARRAY_SIZE(oscar_accel_channels) },
		[OSCAR_GYRO] = { "apple-oscar-gyro", oscar_gyro_channels,
				 ARRAY_SIZE(oscar_gyro_channels) },
		[OSCAR_MAGN] = { "apple-oscar-magn", oscar_magn_channels,
				 ARRAY_SIZE(oscar_magn_channels) },
	};
	int i, ret;

	for (i = 0; i < OSCAR_NUM_SENSORS; i++) {
		struct iio_dev *indio_dev = o->sensor[i].indio_dev;

		indio_dev->name = kinds[i].name;
		indio_dev->info = &oscar_iio_info;
		indio_dev->modes = INDIO_DIRECT_MODE;
		indio_dev->channels = kinds[i].channels;
		indio_dev->num_channels = kinds[i].num_channels;
		ret = devm_iio_device_register(o->dev, indio_dev);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---- power and boot ---- */

static void oscar_power_off(struct oscar *o)
{
	unsigned long flags;

	spin_lock_irqsave(&o->lock, flags);
	o->state = OSCAR_OFF;
	spin_unlock_irqrestore(&o->lock, flags);
	gpiod_set_value_cansleep(o->reset, 1);
	if (o->sensors_on)
		regulator_disable(o->sensors);
	if (o->io_on)
		regulator_disable(o->io);
	if (o->core_on)
		regulator_disable(o->core);
	o->sensors_on = false;
	o->io_on = false;
	o->core_on = false;
}

static int oscar_boot(struct oscar *o, const struct firmware *fw)
{
	unsigned long flags;
	u8 build[0x80];
	size_t n = sizeof(build) - 1;
	ssize_t ret;
	int i;

	/* iOS: all off, reset held, 100 ms, core, 5 ms, I/O, 10 ms, sensors, 20 ms */
	oscar_power_off(o);
	msleep(100);

	spin_lock_irqsave(&o->lock, flags);
	o->state = OSCAR_ROM_WAIT;
	reinit_completion(&o->rom_ready);
	reinit_completion(&o->rom_done);
	reinit_completion(&o->fw_ready);
	memset(o->line, 0, sizeof(o->line));
	spin_unlock_irqrestore(&o->lock, flags);

	ret = regulator_enable(o->core);
	if (ret)
		goto fail;
	o->core_on = true;
	usleep_range(5000, 6000);
	ret = regulator_enable(o->io);
	if (ret)
		goto fail;
	o->io_on = true;
	usleep_range(10000, 11000);
	ret = regulator_enable(o->sensors);
	if (ret)
		goto fail;
	o->sensors_on = true;
	msleep(20);
	gpiod_set_value_cansleep(o->reset, 0);

	if (!wait_for_completion_timeout(&o->rom_ready, msecs_to_jiffies(1000))) {
		dev_err(o->dev, "no answer from the M7 boot ROM\n");
		ret = -ETIMEDOUT;
		goto fail;
	}

	spin_lock_irqsave(&o->lock, flags);
	o->state = OSCAR_ROM_LOAD;
	spin_unlock_irqrestore(&o->lock, flags);

	ret = serdev_device_write(o->serdev, fw->data, fw->size, msecs_to_jiffies(5000));
	if (ret < 0)
		goto fail;
	if (ret != fw->size) {
		ret = -EIO;
		goto fail;
	}
	if (!wait_for_completion_timeout(&o->rom_done, msecs_to_jiffies(20000))) {
		dev_err(o->dev, "the M7 boot ROM did not answer the image\n");
		ret = -ETIMEDOUT;
		goto fail;
	}
	if (!o->rom_ok) {
		dev_err(o->dev, "the M7 boot ROM refused the image\n");
		ret = -EINVAL;
		goto fail;
	}
	if (!wait_for_completion_timeout(&o->fw_ready, msecs_to_jiffies(2000)))
		dev_warn(o->dev, "no ready report from the M7 firmware\n");

	if (!oscar_get_prop(o, OSCAR_DEV_SYSTEM, OSCAR_PROP_BUILD, build, &n)) {
		build[n] = 0;
		dev_info(o->dev, "M7 firmware %s\n", build);
	}

	for (i = 0; i < OSCAR_NUM_SENSORS; i++)
		o->sensor[i].streaming = false;
	return 0;

fail:
	oscar_power_off(o);
	return ret;
}

static void oscar_boot_work(struct work_struct *work)
{
	struct oscar *o = container_of(work, struct oscar, boot_work);
	const struct firmware *fw;
	int ret, tries;

	for (tries = 0; tries < 10; tries++) {
		ret = request_firmware(&fw, o->fw_name, o->dev);
		if (ret != -ENOENT)
			break;
		msleep(3000);	/* the root file system may not be there yet */
	}
	if (ret) {
		dev_err(o->dev, "firmware %s: %d\n", o->fw_name, ret);
		return;
	}

	for (tries = 0; tries < 2; tries++) {
		ret = oscar_boot(o, fw);
		if (!ret)
			break;
	}
	release_firmware(fw);
	if (ret) {
		dev_err(o->dev, "M7 boot failed: %d\n", ret);
		return;
	}

	if (!o->registered) {
		ret = oscar_register(o);
		if (ret) {
			dev_err(o->dev, "IIO registration failed: %d\n", ret);
			return;
		}
		o->registered = true;
	}
}

static int oscar_probe(struct serdev_device *serdev)
{
	static const struct {
		u8 dev, report;
		u32 interval_us;
	} sensors[OSCAR_NUM_SENSORS] = {
		[OSCAR_ACCEL] = { OSCAR_DEV_ACCEL, OSCAR_REP_ACCEL, 20000 },
		[OSCAR_GYRO] = { OSCAR_DEV_GYRO, OSCAR_REP_GYRO, 20000 },
		[OSCAR_MAGN] = { OSCAR_DEV_COMPASS, OSCAR_REP_COMPASS, 50000 },
	};
	struct device *dev = &serdev->dev;
	struct oscar *o;
	int i, ret;

	o = devm_kzalloc(dev, sizeof(*o), GFP_KERNEL);
	if (!o)
		return -ENOMEM;
	o->serdev = serdev;
	o->dev = dev;
	spin_lock_init(&o->lock);
	mutex_init(&o->cmd_lock);
	init_completion(&o->rom_ready);
	init_completion(&o->rom_done);
	init_completion(&o->fw_ready);
	init_completion(&o->cmd_done);
	init_waitqueue_head(&o->sample_wq);
	INIT_WORK(&o->boot_work, oscar_boot_work);
	INIT_DELAYED_WORK(&o->idle_work, oscar_idle_work);
	serdev_device_set_drvdata(serdev, o);

	o->fw_name = "apple/s5l8960x-oscar.bin";
	device_property_read_string(dev, "firmware-name", &o->fw_name);

	o->core = devm_regulator_get(dev, "core");
	if (IS_ERR(o->core))
		return dev_err_probe(dev, PTR_ERR(o->core), "core supply\n");
	o->io = devm_regulator_get(dev, "io");
	if (IS_ERR(o->io))
		return dev_err_probe(dev, PTR_ERR(o->io), "io supply\n");
	o->sensors = devm_regulator_get(dev, "sensors");
	if (IS_ERR(o->sensors))
		return dev_err_probe(dev, PTR_ERR(o->sensors), "sensors supply\n");
	o->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(o->reset))
		return dev_err_probe(dev, PTR_ERR(o->reset), "reset GPIO\n");

	for (i = 0; i < OSCAR_NUM_SENSORS; i++) {
		struct oscar_sensor *s = &o->sensor[i];
		struct iio_dev *indio_dev;

		indio_dev = devm_iio_device_alloc(dev, sizeof(s));
		if (!indio_dev)
			return -ENOMEM;
		*(struct oscar_sensor **)iio_priv(indio_dev) = s;
		mutex_init(&s->lock);
		s->oscar = o;
		s->indio_dev = indio_dev;
		s->dev = sensors[i].dev;
		s->report = sensors[i].report;
		s->interval_us = sensors[i].interval_us;
		s->temp = OSCAR_TEMP_NONE;
	}
	/*
	 * The board's mount-matrix is the accelerometer's; the gyroscope and
	 * compass are other chips whose axes are not known yet, so they
	 * report in their own frame without a matrix.
	 */
	ret = iio_read_mount_matrix(dev, &o->sensor[OSCAR_ACCEL].orientation);
	if (ret)
		return ret;

	serdev_device_set_client_ops(serdev, &oscar_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot open the UART\n");
	if (serdev_device_set_baudrate(serdev, OSCAR_BAUD) != OSCAR_BAUD)
		dev_warn(dev, "UART not at %u baud\n", OSCAR_BAUD);
	serdev_device_set_flow_control(serdev, false);

	schedule_work(&o->boot_work);
	return 0;
}

static void oscar_remove(struct serdev_device *serdev)
{
	struct oscar *o = serdev_device_get_drvdata(serdev);

	cancel_work_sync(&o->boot_work);
	cancel_delayed_work_sync(&o->idle_work);
	oscar_power_off(o);
}

static int oscar_suspend(struct device *dev)
{
	struct oscar *o = dev_get_drvdata(dev);
	int i;

	cancel_delayed_work_sync(&o->idle_work);
	if (o->state != OSCAR_RUN)
		return 0;
	for (i = 0; i < OSCAR_NUM_SENSORS; i++) {
		mutex_lock(&o->sensor[i].lock);
		oscar_stream(&o->sensor[i], false);
		mutex_unlock(&o->sensor[i].lock);
	}
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(oscar_pm_ops, oscar_suspend, NULL);

static const struct of_device_id oscar_of_match[] = {
	{ .compatible = "apple,s5l8960x-oscar" },
	{ }
};
MODULE_DEVICE_TABLE(of, oscar_of_match);

static struct serdev_device_driver oscar_driver = {
	.probe = oscar_probe,
	.remove = oscar_remove,
	.driver = {
		.name = "apple-oscar",
		.of_match_table = oscar_of_match,
		.pm = pm_sleep_ptr(&oscar_pm_ops),
	},
};
module_serdev_device_driver(oscar_driver);

MODULE_DESCRIPTION("Apple M7 motion coprocessor (accelerometer, gyroscope, compass)");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_FIRMWARE("apple/s5l8960x-oscar.bin");
