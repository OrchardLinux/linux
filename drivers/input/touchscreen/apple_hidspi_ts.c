// SPDX-License-Identifier: GPL-2.0
/*
 * Apple HIDSPI touchscreen (iPhone X, "hid-transport,k1").
 *
 * The controller starts in its HBPP bootloader. The host loads the firmware
 * with HBPP packets in SPI mode 3, following the steps of Apple's MTFW
 * personality (converted by Orchard into an op list, see below). The firmware
 * then speaks HIDSPI in SPI mode 0: every transfer is 4 bytes of padding and a
 * 512-byte packet framed as in Asahi's spi-hid-apple (flags, device, offset,
 * remain, length, data, CRC-16). Touches arrive on interface 0 as report 0x44
 * with apple_z2 finger records.
 *
 * Apple's firmware comes in variants ("personalities"), and each unit's ADT
 * names the one it needs. m1n1 passes that on as apple,hid-merge-personality;
 * the file is <firmware-name>-<personality>.bin. If it isn't there, the
 * touchscreen stays off and the log names the variant.
 *
 * Firmware file ("D221OPS1" + ops, little endian):
 *   'T' u32 len, label       'S' u16 ack (0: none), u32 len, HBPP packet
 *   'P' u32 addr, mask, value, timeout_ms        'R' u32 addr
 *   'C' u32 addr, u32 max, u32 len, DT property name: the unit's calibration,
 *       copied from the ADT by m1n1, sent as an HBPP blob at addr
 *
 * The CS-to-clock delay of iOS (200 us, "CS Delay") is a zero-length transfer
 * with a delay in front of each message.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/cleanup.h>
#include <linux/firmware.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define HIDSPI_PAD		4
#define HIDSPI_PKT		512
#define HIDSPI_XFER		(HIDSPI_PAD + HIDSPI_PKT)
#define HIDSPI_READ		0x20
#define HIDSPI_WRITE		0x40
#define HIDSPI_DEV_INFO		0xd0
#define HIDSPI_DEV_TOUCH	0x00
#define HIDSPI_MSG_INPUT	0x10
#define HIDSPI_MSG_GET_REPORT	0x32
#define TOUCH_REPORT		0x44
#define SURFACE_REPORT		0xd9
#define NUM_FINGERS_OFFSET	16
#define FINGERS_OFFSET		24
#define TOUCH_STARTED		3
#define TOUCH_MOVED		4
#define MAX_SLOTS		10

#define HBPP_ACK_TRIES		5
#define HBPP_READ_STATUS	0x4c39
#define HBPP_ERROR		0x4f81
#define HIDSPI_GAP_US		250	/* MTFW "Inter Transaction Us" */
#define RECOVER_AFTER		20

static const u8 hidspi_booted[4] = { 0xa0, 0x80, 0x00, 0x00 };
static const u8 hbpp_nop[16] = {
	0x1a, 0xa1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
	0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
};

struct apple_hidspi_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_hidspi_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct regulator *ana;
	struct clk *clk;
	struct gpio_desc *reset;
	const char *fw_name;
	u8 *tx;
	u8 *rx;
	u8 *irq_tx;
	u8 *irq_rx;
	u8 *pkt;
	size_t pkt_size;
	struct mutex lock;	/* the bootloader exchanges and requests */
	u8 msg_id;
	bool powered;
	bool input_ready;
	unsigned int bootloader_seen;
	struct work_struct recover;
	struct completion booted;
	struct completion surface;
	s16 min_x, max_x, min_y, max_y;
};

static int ahts_set_mode(struct apple_hidspi_ts *ts, u32 mode)
{
	ts->spi->mode = (ts->spi->mode & ~SPI_MODE_X_MASK) | mode;
	return spi_setup(ts->spi);
}

/* One CS-asserted message: the iOS CS-to-clock delay, then @len bytes. */
static int ahts_xfer(struct apple_hidspi_ts *ts, const void *tx, void *rx,
		     size_t len)
{
	struct spi_transfer t[2] = {
		{ .delay = { .value = 200, .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = tx, .rx_buf = rx, .len = len },
	};

	return spi_sync_transfer(ts->spi, t, ARRAY_SIZE(t));
}

/* HBPP bootloader (mode 3) */

static int ahts_hbpp_ack(struct apple_hidspi_ts *ts, u16 *reply)
{
	static const u8 ack[2] = { 0x1a, 0xa1 };
	int ret;

	ret = ahts_xfer(ts, ack, ts->rx, sizeof(ack));
	if (!ret)
		*reply = get_unaligned_be16(ts->rx);
	return ret;
}

static int ahts_hbpp_send(struct apple_hidspi_ts *ts, const u8 *pkt,
			  size_t len, u16 want)
{
	u16 reply = 0;
	int i, ret = 0;

	for (i = 0; i < HBPP_ACK_TRIES; i++) {
		if (i)
			usleep_range(2000, 3000);
		ret = ahts_xfer(ts, pkt, NULL, len);
		if (ret)
			return ret;
		if (!want)
			return 0;
		ret = ahts_hbpp_ack(ts, &reply);
		if (!ret && reply == want)
			return 0;
	}
	dev_err(&ts->spi->dev, "HBPP packet of %zu bytes: ACK %#06x, want %#06x\n",
		len, reply, want);
	return ret ?: -EPROTO;
}

/*
 * HBPP read (iOS 0xfffffff0059216b8): the reply is 4c39, the value (BE16 low
 * half, then the high bytes) and the BE16 sum of the four value bytes. It can
 * arrive shifted, so look for it at every bit offset of a 16-byte exchange.
 */
static int ahts_hbpp_read(struct apple_hidspi_ts *ts, u32 addr, u32 *value)
{
	u8 req[8] = { 0x1c, 0x73, addr >> 8, addr, addr >> 24, addr >> 16 };
	u8 *rx = ts->rx;
	int ret, sh, k;
	u16 sum;

	sum = req[2] + req[3] + req[4] + req[5];
	put_unaligned_be16(sum, req + 6);
	ret = ahts_xfer(ts, req, NULL, sizeof(req));
	if (ret)
		return ret;
	ret = ahts_xfer(ts, hbpp_nop, rx, sizeof(hbpp_nop));
	if (ret)
		return ret;

	for (sh = 0; sh <= 64; sh++) {
		u8 b[8];

		for (k = 0; k < 8; k++) {
			int bit = sh + 8 * k, by = bit / 8, o = bit % 8;

			b[k] = o ? (rx[by] << o | rx[by + 1] >> (8 - o)) : rx[by];
		}
		if (get_unaligned_be16(b) != HBPP_READ_STATUS)
			continue;
		if ((u16)(b[2] + b[3] + b[4] + b[5]) != get_unaligned_be16(b + 6))
			continue;
		*value = get_unaligned_be16(b + 2) | (u32)b[4] << 24 |
			 (u32)b[5] << 16;
		return 0;
	}
	return -EPROTO;
}

static bool ahts_hbpp_word(u16 w)
{
	switch (w) {
	case 0x18e1: case 0x1aa1: case 0x1f01: case 0x4879:
	case 0x4969: case 0x4ad1: case 0x4bc1:
		return true;
	}
	return false;
}

static int ahts_hbpp_detect(struct apple_hidspi_ts *ts)
{
	int ret;

	ret = ahts_xfer(ts, hbpp_nop, ts->rx, sizeof(hbpp_nop));
	if (ret)
		return ret;
	if (ahts_hbpp_word(get_unaligned_be16(ts->rx)) &&
	    ahts_hbpp_word(get_unaligned_be16(ts->rx + 2)))
		return 0;
	dev_dbg(&ts->spi->dev, "no HBPP bootloader: %*ph\n", 4, ts->rx);
	return -ENODEV;
}

/*
 * HBPP blob loading @len bytes at @addr (z2fw.encode_blob): 18 e1 30 01, the
 * BE16 word count minus one, the address (BE16 halves, low first), the BE16
 * sum of those six bytes, the payload with each byte pair swapped, and the
 * sum of the payload bytes as the address is encoded.
 */
static size_t ahts_build_blob(u8 *out, u32 addr, const u8 *data, size_t len)
{
	size_t padded = ALIGN(len, 4), k;
	u32 sum = 0;

	out[0] = 0x18;
	out[1] = 0xe1;
	out[2] = 0x30;
	out[3] = 0x01;
	put_unaligned_be16(padded / 4 - 1, out + 4);
	put_unaligned_be16(addr, out + 6);
	put_unaligned_be16(addr >> 16, out + 8);
	put_unaligned_be16(out[4] + out[5] + out[6] + out[7] + out[8] + out[9], out + 10);
	memset(out + 12, 0, padded);
	for (k = 0; k < len; k++) {
		out[12 + (k ^ 1)] = data[k];
		sum += data[k];
	}
	put_unaligned_be16(sum, out + 12 + padded);
	put_unaligned_be16(sum >> 16, out + 14 + padded);
	return 16 + padded;
}

static int ahts_send_calibration(struct apple_hidspi_ts *ts, u32 addr,
				 u32 max, const char *name)
{
	struct device *dev = &ts->spi->dev;
	int count = device_property_count_u8(dev, name);
	size_t len;
	int ret;

	if (count <= 0) {
		dev_info(dev, "no %s; the controller keeps its default\n", name);
		return 0;
	}
	len = min_t(size_t, count, max);
	if (16 + ALIGN(len, 4) > ts->pkt_size)
		return -EINVAL;
	u8 *cal __free(kfree) = kmalloc(count, GFP_KERNEL);
	if (!cal)
		return -ENOMEM;
	ret = device_property_read_u8_array(dev, name, cal, count);
	if (ret)
		return ret;
	return ahts_hbpp_send(ts, ts->pkt, ahts_build_blob(ts->pkt, addr, cal, len),
			      0x4bc1);
}

static int ahts_run_ops(struct apple_hidspi_ts *ts, const u8 *d, size_t n)
{
	struct device *dev = &ts->spi->dev;
	size_t i = 8;
	int ret;

	if (n < 8 || memcmp(d, "D221OPS1", 8))
		return dev_err_probe(dev, -EINVAL, "bad firmware header\n");

	while (i < n) {
		u8 op = d[i++];
		u32 len, addr, mask, val, tmo, r = 0;
		unsigned long end;

		switch (op) {
		case 'T':
			if (n - i < 4)
				return -EINVAL;
			len = get_unaligned_le32(d + i);
			if (len > n - i - 4)
				return -EINVAL;
			dev_dbg(dev, "%.*s\n", (int)len, d + i + 4);
			i += 4 + len;
			break;
		case 'S':
			if (n - i < 6)
				return -EINVAL;
			val = get_unaligned_le16(d + i);
			len = get_unaligned_le32(d + i + 2);
			i += 6;
			if (len > n - i || len > ts->pkt_size)
				return -EINVAL;
			memcpy(ts->pkt, d + i, len);
			ret = ahts_hbpp_send(ts, ts->pkt, len, val);
			if (ret)
				return ret;
			i += len;
			break;
		case 'P':
			if (n - i < 16)
				return -EINVAL;
			addr = get_unaligned_le32(d + i);
			mask = get_unaligned_le32(d + i + 4);
			val = get_unaligned_le32(d + i + 8);
			tmo = get_unaligned_le32(d + i + 12);
			i += 16;
			end = jiffies + msecs_to_jiffies(tmo);
			for (;;) {
				ret = ahts_hbpp_read(ts, addr, &r);
				if (!ret && (r & mask) == val)
					break;
				if (time_after(jiffies, end))
					return dev_err_probe(dev, -ETIMEDOUT,
						"poll %#x & %#x != %#x (last %#x)\n",
						addr, mask, val, r);
				usleep_range(5000, 6000);
			}
			break;
		case 'C': {
			char name[64];

			if (n - i < 12)
				return -EINVAL;
			addr = get_unaligned_le32(d + i);
			mask = get_unaligned_le32(d + i + 4);
			len = get_unaligned_le32(d + i + 8);
			i += 12;
			if (len > n - i || len >= sizeof(name))
				return -EINVAL;
			memcpy(name, d + i, len);
			name[len] = '\0';
			i += len;
			ret = ahts_send_calibration(ts, addr, mask, name);
			if (ret)
				return ret;
			break;
		}
		case 'R':
			if (n - i < 4)
				return -EINVAL;
			addr = get_unaligned_le32(d + i);
			i += 4;
			if (!ahts_hbpp_read(ts, addr, &r))
				dev_dbg(dev, "read %#x = %#x\n", addr, r);
			break;
		default:
			return dev_err_probe(dev, -EINVAL, "bad op %#x at %zu\n",
					     op, i - 1);
		}
	}
	return 0;
}

static void ahts_power_off(struct apple_hidspi_ts *ts)
{
	gpiod_set_value_cansleep(ts->reset, 1);
	if (!ts->powered)
		return;
	clk_disable_unprepare(ts->clk);
	regulator_disable(ts->ana);
	ts->powered = false;
}

/* Reset held, supply and clock on, reset released: the HBPP bootloader. */
static int ahts_power_on(struct apple_hidspi_ts *ts)
{
	int ret;

	gpiod_set_value_cansleep(ts->reset, 1);
	ret = clk_prepare_enable(ts->clk);
	if (ret)
		return ret;
	ret = regulator_enable(ts->ana);
	if (ret) {
		clk_disable_unprepare(ts->clk);
		return ret;
	}
	ts->powered = true;
	msleep(50);
	gpiod_set_value_cansleep(ts->reset, 0);
	msleep(100);
	return 0;
}

static int ahts_boot(struct apple_hidspi_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	const struct firmware *fw;
	int ret, try;

	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "no firmware for this unit's touch variant (%s): touchscreen left off\n",
				     ts->fw_name);
	ts->pkt_size = SZ_256K;
	ts->pkt = kmalloc(ts->pkt_size, GFP_KERNEL);
	if (!ts->pkt) {
		release_firmware(fw);
		return -ENOMEM;
	}

	for (try = 0; try < 3; try++) {
		if (try)
			ahts_power_off(ts);
		ret = ahts_power_on(ts);
		if (ret)
			break;
		ret = ahts_set_mode(ts, SPI_MODE_3);
		if (ret)
			break;
		ret = ahts_hbpp_detect(ts);
		if (ret)
			continue;
		ret = ahts_run_ops(ts, fw->data, fw->size);
		if (!ret)
			break;
	}
	release_firmware(fw);
	kfree(ts->pkt);
	ts->pkt = NULL;
	if (ret)
		return dev_err_probe(dev, ret, "firmware load failed\n");
	dev_info(dev, "firmware loaded (try %d)\n", try + 1);
	msleep(10);
	return ahts_set_mode(ts, SPI_MODE_0);
}

/* HIDSPI (mode 0) */

static int ahts_request(struct apple_hidspi_ts *ts, u8 dev, u8 u0, u8 u1,
			u8 u2, u16 rsplen)
{
	struct spi_transfer t[3] = {
		{ .delay = { .value = 200, .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = ts->tx, .len = HIDSPI_XFER,
		  .delay = { .value = 200, .unit = SPI_DELAY_UNIT_USECS } },
		{ .rx_buf = ts->rx, .len = 8 },
	};
	u8 *pkt = ts->tx + HIDSPI_PAD, *h = pkt + 8;
	u16 crc;
	int ret;

	guard(mutex)(&ts->lock);
	memset(ts->tx, 0, HIDSPI_XFER);
	pkt[0] = HIDSPI_WRITE;
	pkt[1] = dev;
	put_unaligned_le16(10, pkt + 6);
	h[0] = u0;
	h[1] = u1;
	h[2] = u2;
	h[3] = ts->msg_id++;
	put_unaligned_le16(rsplen, h + 4);
	crc = crc16(0, h, 8);
	put_unaligned_le16(crc, h + 8);
	crc = crc16(0, pkt, HIDSPI_PKT - 2);
	put_unaligned_le16(crc, pkt + HIDSPI_PKT - 2);
	ret = spi_sync_transfer(ts->spi, t, ARRAY_SIZE(t));
	usleep_range(HIDSPI_GAP_US, HIDSPI_GAP_US + 50);
	return ret;
}

/* Report 0xd9: LE32 width, height (0.01 mm), s16 left, bottom, right, top. */
static void ahts_surface(struct apple_hidspi_ts *ts, const u8 *d, size_t len)
{
	s16 l, b, r, t;

	if (len < 16)
		return;
	l = get_unaligned_le16(d + 8);
	b = get_unaligned_le16(d + 10);
	r = get_unaligned_le16(d + 12);
	t = get_unaligned_le16(d + 14);
	if (r <= l || t <= b)
		return;
	ts->min_x = l;
	ts->max_x = r;
	ts->min_y = b;
	ts->max_y = t;
	dev_info(&ts->spi->dev, "surface %u x %u (0.01 mm), x %d..%d, y %d..%d\n",
		 get_unaligned_le32(d), get_unaligned_le32(d + 4), l, r, b, t);
	complete(&ts->surface);
}

static unsigned int ahts_scale(int v, int lo, int hi, unsigned int max)
{
	long n = (long)(v - lo) * max / (hi - lo);

	return clamp(n, 0L, (long)max);
}

static void ahts_touch(struct apple_hidspi_ts *ts, const u8 *pl, size_t len)
{
	const struct apple_hidspi_finger *f;
	struct input_dev *input = ts->input;
	int i, nf, slot;

	if (!ts->input_ready || len <= NUM_FINGERS_OFFSET)
		return;
	nf = pl[NUM_FINGERS_OFFSET];
	if (len < FINGERS_OFFSET || nf > (len - FINGERS_OFFSET) / sizeof(*f))
		return;
	f = (const struct apple_hidspi_finger *)(pl + FINGERS_OFFSET);
	for (i = 0; i < nf; i++) {
		bool on = f[i].state == TOUCH_STARTED || f[i].state == TOUCH_MOVED;
		unsigned int x, y;

		slot = input_mt_get_slot_by_key(input, f[i].finger);
		if (slot < 0)
			continue;
		input_mt_slot(input, slot);
		if (!input_mt_report_slot_state(input, MT_TOOL_FINGER, on))
			continue;
		x = ahts_scale((s16)le16_to_cpu(f[i].abs_x), ts->min_x, ts->max_x,
			       ts->props.max_x);
		y = ahts_scale((s16)le16_to_cpu(f[i].abs_y), ts->min_y, ts->max_y,
			       ts->props.max_y);
		touchscreen_report_pos(input, &ts->props, x, ts->props.max_y - y,
				       true);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, le16_to_cpu(f[i].touch_major));
		input_report_abs(input, ABS_MT_TOUCH_MINOR, le16_to_cpu(f[i].touch_minor));
		input_report_abs(input, ABS_MT_WIDTH_MAJOR, le16_to_cpu(f[i].tool_major));
		input_report_abs(input, ABS_MT_WIDTH_MINOR, le16_to_cpu(f[i].tool_minor));
		input_report_abs(input, ABS_MT_ORIENTATION,
				 (s16)le16_to_cpu(f[i].orientation));
	}
	input_mt_sync_frame(input);
	input_sync(input);
}

static irqreturn_t ahts_irq(int irq, void *data)
{
	struct apple_hidspi_ts *ts = data;
	u8 *p = ts->irq_rx + HIDSPI_PAD, *m = p + 8;
	size_t len, mlen;

	if (ahts_xfer(ts, ts->irq_tx, ts->irq_rx, HIDSPI_XFER))
		return IRQ_HANDLED;
	usleep_range(HIDSPI_GAP_US, HIDSPI_GAP_US + 50);
	if (crc16(0, p, HIDSPI_PKT - 2) != get_unaligned_le16(p + HIDSPI_PKT - 2)) {
		/* Back in the HBPP bootloader: the firmware is gone. */
		if (get_unaligned_be16(ts->irq_rx) == HBPP_ERROR &&
		    ++ts->bootloader_seen == RECOVER_AFTER) {
			disable_irq_nosync(ts->spi->irq);
			schedule_work(&ts->recover);
			dev_warn(&ts->spi->dev, "controller reset to its bootloader; reloading\n");
			return IRQ_HANDLED;
		}
		dev_warn_ratelimited(&ts->spi->dev, "packet CRC mismatch: %*ph\n",
				     8, ts->irq_rx);
		return IRQ_HANDLED;
	}
	ts->bootloader_seen = 0;
	len = get_unaligned_le16(p + 6);
	if (len > HIDSPI_PKT - 10)
		return IRQ_HANDLED;

	if (p[1] == HIDSPI_DEV_INFO && len == sizeof(hidspi_booted) &&
	    !memcmp(m, hidspi_booted, len)) {
		complete(&ts->booted);
		return IRQ_HANDLED;
	}
	if (len < 10)
		return IRQ_HANDLED;
	mlen = get_unaligned_le16(m + 6);
	if (mlen + 10 > len || !mlen)
		return IRQ_HANDLED;

	if (p[0] == HIDSPI_WRITE && m[0] == HIDSPI_MSG_GET_REPORT &&
	    m[1] == SURFACE_REPORT && m[8] == SURFACE_REPORT)
		ahts_surface(ts, m + 9, mlen - 1);
	else if (p[0] == HIDSPI_READ && p[1] == HIDSPI_DEV_TOUCH &&
		 m[0] == HIDSPI_MSG_INPUT && m[1] == TOUCH_REPORT)
		ahts_touch(ts, m + 8, mlen);
	return IRQ_HANDLED;
}

static void ahts_recover_work(struct work_struct *work)
{
	struct apple_hidspi_ts *ts = container_of(work, struct apple_hidspi_ts, recover);

	reinit_completion(&ts->booted);
	ts->bootloader_seen = 0;
	ahts_power_off(ts);
	if (ahts_boot(ts))
		return;		/* the IRQ stays disabled */
	enable_irq(ts->spi->irq);
	if (!wait_for_completion_timeout(&ts->booted, msecs_to_jiffies(1000)))
		dev_warn(&ts->spi->dev, "no boot message after reload\n");
}

static void ahts_power_off_action(void *data)
{
	struct apple_hidspi_ts *ts = data;

	cancel_work_sync(&ts->recover);
	ahts_power_off(ts);
}

static int ahts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_hidspi_ts *ts;
	struct input_dev *input;
	const char *fw_prefix, *personality;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	ts->min_x = -218;
	ts->max_x = 6003;
	ts->min_y = -220;
	ts->max_y = 13270;
	init_completion(&ts->booted);
	init_completion(&ts->surface);
	INIT_WORK(&ts->recover, ahts_recover_work);
	mutex_init(&ts->lock);
	ts->tx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, HIDSPI_XFER + 16, GFP_KERNEL);
	ts->irq_tx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->irq_rx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	if (!ts->tx || !ts->rx || !ts->irq_tx || !ts->irq_rx)
		return -ENOMEM;

	ts->ana = devm_regulator_get(dev, "ana");
	if (IS_ERR(ts->ana))
		return dev_err_probe(dev, PTR_ERR(ts->ana), "no ana supply\n");
	ts->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ts->clk))
		return dev_err_probe(dev, PTR_ERR(ts->clk), "no clock\n");
	ts->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset))
		return dev_err_probe(dev, PTR_ERR(ts->reset), "no reset GPIO\n");
	ret = device_property_read_string(dev, "firmware-name", &fw_prefix);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");
	ret = device_property_read_string(dev, "apple,hid-merge-personality", &personality);
	if (ret)
		return dev_err_probe(dev, ret,
				     "the bootloader passed no touch firmware variant: touchscreen left off\n");
	if (strchr(personality, '/'))
		return dev_err_probe(dev, -EINVAL, "bad touch firmware variant\n");
	ts->fw_name = devm_kasprintf(dev, GFP_KERNEL, "%s-%s.bin", fw_prefix, personality);
	if (!ts->fw_name)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	ts->input = input;
	input->name = "Apple iPhone X Touchscreen";
	input->phys = "apple_hidspi_ts";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x05ac;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 1124, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 2435, 0, 0);
	touchscreen_parse_properties(input, true, &ts->props);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, -32768, 32767, 0, 0);
	ret = input_mt_init_slots(input, MAX_SLOTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ahts_power_off_action, ts);
	if (ret)
		return ret;
	ret = ahts_boot(ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ahts_irq,
					IRQF_ONESHOT, "apple-hidspi-ts", ts);
	if (ret)
		return dev_err_probe(dev, ret, "no IRQ\n");
	if (!wait_for_completion_timeout(&ts->booted, msecs_to_jiffies(1000)))
		dev_warn(dev, "no boot message from the firmware\n");
	if (ahts_request(ts, HIDSPI_DEV_TOUCH, HIDSPI_MSG_GET_REPORT,
			 SURFACE_REPORT, 0x00, 64) ||
	    !wait_for_completion_timeout(&ts->surface, msecs_to_jiffies(500)))
		dev_warn(dev, "no surface descriptor; using defaults\n");

	ret = input_register_device(input);
	if (ret)
		return ret;
	ts->input_ready = true;
	return 0;
}

static const struct of_device_id ahts_of_match[] = {
	{ .compatible = "apple,d221-touchscreen" },
	{ }
};
MODULE_DEVICE_TABLE(of, ahts_of_match);

static const struct spi_device_id ahts_spi_id[] = {
	{ "d221-touchscreen" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ahts_spi_id);

static struct spi_driver ahts_driver = {
	.driver = {
		.name = "apple-hidspi-ts",
		.of_match_table = ahts_of_match,
	},
	.id_table = ahts_spi_id,
	.probe = ahts_probe,
};
module_spi_driver(ahts_driver);

MODULE_DESCRIPTION("Apple HIDSPI touchscreen (iPhone X)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/d221-touch-*.bin");
