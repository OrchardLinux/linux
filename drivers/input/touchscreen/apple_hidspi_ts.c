// SPDX-License-Identifier: GPL-2.0
/*
 * Apple HIDSPI touchscreens: iPhone X ("hid-transport,k1") and iPhone XR
 * ("hid-transport,spi").
 *
 * The controller starts in its HBPP bootloader. The host loads the firmware
 * with HBPP packets, following the steps of Apple's MTFW personality
 * (converted by Orchard into an op list, see below). The firmware then speaks
 * HIDSPI in SPI mode 0: every transfer is 4 bytes of padding and a 512-byte
 * packet framed as in Asahi's spi-hid-apple (flags, device, offset, remain,
 * length, data, CRC-16). Touches arrive on interface 0 as report 0x44 with
 * apple_z2 finger records.
 *
 * What differs between the controllers is in struct ahts_variant: the
 * supplies, the bootloader's SPI mode and HBPP version (before 0x104 a blob
 * counts its words, since then its words minus one), the largest blob and the
 * wait before a packet's acknowledge, and whether HIDSPI has iOS's status
 * stage: 200 us after a packet, with chip select still asserted, 4 bytes of
 * padding and a status word both ways. A controller that wants it also takes
 * an all-zero packet (its CRC is valid) as a command and falls back to its
 * bootloader, so the host clocks out 0xff while it reads.
 *
 * The iPhone X's firmware holds its interrupt line low for every packet and
 * hands each one out once. The iPhone XR's hands out its newest packet on
 * every read, again and again, and its line announces nothing: there a
 * thread reads at the intervals of Project Cascadia's driver and tells a new
 * packet from a repeated one by comparing.
 *
 * Apple's firmware comes in variants ("personalities"), and each unit's ADT
 * names the one it needs. m1n1 passes that on as apple,hid-merge-personality;
 * the file is <firmware-name>-<personality>.bin. If it isn't there, the
 * touchscreen stays off and the log names the variant.
 *
 * Firmware file ("D221OPS2" + ops, little endian):
 *   'T' u32 len, label       'S' u16 ack (0: none, 0xffff: any reply),
 *                                u32 len, HBPP packet
 *   'P' u32 addr, mask, value, timeout_ms        'R' u32 addr
 *   'C' u32 addr, u32 max, u32 len, DT property name, u32 len, default: the
 *       unit's calibration, copied from the ADT by m1n1, sent as an HBPP blob
 *       at addr; Apple's default from the firmware if the unit has none
 *
 * The CS-to-clock delay of iOS ("CS Delay") is a zero-length transfer with a
 * delay in front of each message.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/bitmap.h>
#include <linux/cleanup.h>
#include <linux/firmware.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/lockdep.h>
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
#define TOUCH_LIFTED		7
#define TOUCH_GONE		0
#define MAX_SLOTS		10

#define HBPP_ACK_TRIES		5
#define HBPP_ACK_BLOB		0x4bc1
#define HBPP_ACK_ANY		0xffff
#define HBPP_READ_STATUS	0x4c39
#define HBPP_ERROR		0x4f81
#define HIDSPI_STAGE_US		200	/* MTFW "Inter Stage Us" */
#define HIDSPI_GAP_US		250	/* MTFW "Inter Transaction Us" */
#define HIDSPI_STATUS_LEN	(HIDSPI_PAD + 4)
#define HIDSPI_STATUS_OK	0xd56827ac
#define HIDSPI_STATUS_NAK	0x0e4fb139
#define HIDSPI_PAD_WORD		0xdeadbeef	/* in front of the firmware's packets */
#define RECOVER_AFTER		20
/* Where the host reads unasked, as Project Cascadia's driver does: */
#define POLL_IDLE_US		8000	/* between reads; a report takes 8 ms */
#define POLL_ACTIVE_US		4000	/* the same, for ... */
#define POLL_ACTIVE_MS		300	/* ... this long after a new packet */
#define POLL_DRAIN		8	/* new packets taken without a pause */
#define RELEASE_MS		60	/* no report for this long: fingers are up */
#define MAX_SUPPLIES		3

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

/**
 * struct ahts_variant - what differs between the controllers
 * @name: name of the input device
 * @supplies: the supplies, in the order they turn on
 * @clk: the touch clock is a clock of the device tree
 * @hbpp_mode: SPI mode of the bootloader
 * @hbpp_version: the ADT's hbpp-version
 * @hbpp_chunk: most payload bytes of a blob the driver builds; 0: no limit
 * @hbpp_ack_delay_ms: wait between a packet and its acknowledge
 * @reset_settle_ms: wait after releasing reset; 0: 100 ms
 * @cs_delay_us: MTFW "CS Delay"
 * @status_stage: HIDSPI packets are followed by the status stage
 * @idle_fill: what the host clocks out while it only reads
 * @read_ack: every packet read is answered with a status word (no padding)
 *	in the same chip-select window; without it the firmware hands out the
 *	same packet again and keeps its interrupt line asserted
 * @polled: the firmware repeats its newest packet on every read and its
 *	interrupt line announces none: no interrupt is used, a thread reads
 * @max_x: largest X reported, unless the device tree gives the size
 * @max_y: largest Y reported, unless the device tree gives the size
 * @min_sx: left edge of the sensor, if the firmware sends no surface descriptor
 * @max_sx: right edge; equal to @min_sx if the descriptor is required
 * @min_sy: bottom edge
 * @max_sy: top edge
 */
struct ahts_variant {
	const char *name;
	const char *supplies[MAX_SUPPLIES];
	bool clk;
	u32 hbpp_mode;
	u16 hbpp_version;
	unsigned int hbpp_chunk;
	unsigned int hbpp_ack_delay_ms;
	unsigned int reset_settle_ms;
	u16 cs_delay_us;
	bool status_stage;
	u8 idle_fill;
	bool read_ack;
	bool polled;
	unsigned int max_x, max_y;
	s16 min_sx, max_sx, min_sy, max_sy;
};

struct apple_hidspi_ts {
	struct spi_device *spi;
	const struct ahts_variant *variant;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct regulator *supplies[MAX_SUPPLIES];
	unsigned int num_supplies;
	struct clk *clk;
	struct gpio_desc *reset;
	const char *fw_name;
	u8 *tx;
	u8 *rx;
	u8 *irq_tx;
	u8 *irq_rx;
	u8 *last;		/* the transfer before, where the host polls */
	u8 *status;
	u8 *pkt;
	size_t pkt_size;
	struct mutex lock;	/* requests, polled reads and their reload */
	bool polled;		/* no interrupt: read on a timer */
	u8 msg_id;
	bool powered;
	bool input_ready;
	struct task_struct *thread;
	unsigned int fingers;	/* down in the last report */
	DECLARE_BITMAP(prev, 256);	/* finger ids of the last report */
	unsigned int bootloader_seen;
	struct work_struct recover;
	struct work_struct resume_work;
	bool irq_on;		/* interrupt variants: enabled */
	struct completion booted;
	struct completion surface;
	s16 min_x, max_x, min_y, max_y;
};

static void ahts_resume_work(struct work_struct *work);

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
		{ .delay = { .value = ts->variant->cs_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = tx, .rx_buf = rx, .len = len },
	};

	return spi_sync_transfer(ts->spi, t, ARRAY_SIZE(t));
}

/* HBPP bootloader */

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
		/*
		 * Polling the acknowledge instead of waiting the whole delay
		 * loads the XR's firmware in 0.34 s instead of 2.2 s. The last
		 * packet, which starts the firmware and takes any answer,
		 * still gets the full wait.
		 */
		if (ts->variant->hbpp_ack_delay_ms && want != HBPP_ACK_ANY) {
			unsigned long end = jiffies + msecs_to_jiffies(
				2 * ts->variant->hbpp_ack_delay_ms + 20);

			do {
				usleep_range(200, 300);
				ret = ahts_hbpp_ack(ts, &reply);
				if (!ret && (want == HBPP_ACK_ANY || reply == want))
					return 0;
			} while (time_before(jiffies, end));
			continue;
		}
		if (ts->variant->hbpp_ack_delay_ms)
			msleep(ts->variant->hbpp_ack_delay_ms);
		ret = ahts_hbpp_ack(ts, &reply);
		if (!ret && (want == HBPP_ACK_ANY || reply == want))
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
 * BE16 word count (minus one since HBPP 0x104), the address (BE16 halves, low
 * first), the BE16 sum of those six bytes, the payload with each byte pair
 * swapped, and the sum of the payload bytes as the address is encoded.
 */
static size_t ahts_build_blob(struct apple_hidspi_ts *ts, u8 *out, u32 addr,
			      const u8 *data, size_t len)
{
	size_t padded = ALIGN(len, 4), k;
	u32 sum = 0;

	out[0] = 0x18;
	out[1] = 0xe1;
	out[2] = 0x30;
	out[3] = 0x01;
	put_unaligned_be16(padded / 4 - (ts->variant->hbpp_version >= 0x104),
			   out + 4);
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

/* @len bytes at @addr, as one blob or as many as the bootloader's limit asks. */
static int ahts_send_blob(struct apple_hidspi_ts *ts, u32 addr, const u8 *data,
			  size_t len)
{
	size_t chunk = ts->variant->hbpp_chunk ?: len, off, n;
	int ret;

	for (off = 0; off < len; off += n) {
		n = min(len - off, chunk);
		if (16 + ALIGN(n, 4) > ts->pkt_size)
			return -EINVAL;
		ret = ahts_hbpp_send(ts, ts->pkt,
				     ahts_build_blob(ts, ts->pkt, addr + off,
						     data + off, n),
				     HBPP_ACK_BLOB);
		if (ret)
			return ret;
	}
	return 0;
}

static int ahts_send_calibration(struct apple_hidspi_ts *ts, u32 addr,
				 u32 max, const char *name,
				 const u8 *def, size_t def_len)
{
	struct device *dev = &ts->spi->dev;
	int count = device_property_count_u8(dev, name);
	size_t len;
	int ret;

	if (count <= 0) {
		if (!def_len) {
			dev_info(dev, "no %s; the controller keeps its default\n", name);
			return 0;
		}
		dev_info(dev, "no %s; using the firmware's default\n", name);
		return ahts_send_blob(ts, addr, def, min_t(size_t, def_len, max));
	}
	len = min_t(size_t, count, max);
	u8 *cal __free(kfree) = kmalloc(count, GFP_KERNEL);
	if (!cal)
		return -ENOMEM;
	ret = device_property_read_u8_array(dev, name, cal, count);
	if (ret)
		return ret;
	return ahts_send_blob(ts, addr, cal, len);
}

static int ahts_run_ops(struct apple_hidspi_ts *ts, const u8 *d, size_t n)
{
	struct device *dev = &ts->spi->dev;
	size_t i = 8;
	int ret;

	if (n < 8 || memcmp(d, "D221OPS2", 8))
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
			if (n - i < 4)
				return -EINVAL;
			len = get_unaligned_le32(d + i);
			i += 4;
			if (len > n - i)
				return -EINVAL;
			ret = ahts_send_calibration(ts, addr, mask, name, d + i, len);
			i += len;
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

static void ahts_supplies_off(struct apple_hidspi_ts *ts, unsigned int n)
{
	while (n--)
		regulator_disable(ts->supplies[n]);
}

static void ahts_power_off(struct apple_hidspi_ts *ts)
{
	gpiod_set_value_cansleep(ts->reset, 1);
	if (!ts->powered)
		return;
	clk_disable_unprepare(ts->clk);
	ahts_supplies_off(ts, ts->num_supplies);
	ts->powered = false;
}

/* Reset held, clock and supplies on, reset released: the HBPP bootloader. */
static int ahts_power_on(struct apple_hidspi_ts *ts)
{
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(ts->reset, 1);
	ret = clk_prepare_enable(ts->clk);
	if (ret)
		return ret;
	for (i = 0; i < ts->num_supplies; i++) {
		ret = regulator_enable(ts->supplies[i]);
		if (ret) {
			ahts_supplies_off(ts, i);
			clk_disable_unprepare(ts->clk);
			return ret;
		}
	}
	ts->powered = true;
	msleep(50);
	gpiod_set_value_cansleep(ts->reset, 0);
	msleep(ts->variant->reset_settle_ms ?: 100);
	return 0;
}

static int ahts_boot(struct apple_hidspi_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	const struct firmware *fw;
	ktime_t t0 = ktime_get();
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
		ret = ahts_set_mode(ts, ts->variant->hbpp_mode);
		if (ret)
			break;
		/*
		 * On the XR the first power-up after the supplies were off
		 * answers 3e 02 instead of the bootloader, whatever the delays;
		 * the second always works.
		 */
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
	dev_info(dev, "firmware loaded in %lld ms (try %d)\n",
		 ktime_ms_delta(ktime_get(), t0), try + 1);
	memset(ts->last, 0, HIDSPI_XFER);
	msleep(10);
	return ahts_set_mode(ts, SPI_MODE_0);
}

/* HIDSPI (mode 0) */

/*
 * A packet whose CRC holds. Where the host sends status words, its packets
 * leave their bytes behind the device's shorter ones, and only the CRC of the
 * message is good; a 4-byte message has none.
 */
static bool ahts_packet_ok(struct apple_hidspi_ts *ts, const u8 *p)
{
	size_t len, mlen;

	if (crc16(0, p, HIDSPI_PKT - 2) == get_unaligned_le16(p + HIDSPI_PKT - 2))
		return true;
	if (!ts->variant->status_stage)
		return false;
	len = get_unaligned_le16(p + 6);
	if (len == 4)
		return p[0] == HIDSPI_READ;
	if (len < 10 || len > HIDSPI_PKT - 10)
		return false;
	mlen = get_unaligned_le16(p + 14);
	return mlen + 10 <= len &&
	       crc16(0, p + 8, 8 + mlen) == get_unaligned_le16(p + 16 + mlen);
}

/*
 * A packet each way, then the status stage in the same chip-select window:
 * the host's word says whether the device's packet arrived whole.
 */
static int ahts_exchange(struct apple_hidspi_ts *ts, const u8 *tx, u8 *rx,
			 bool *ok)
{
	struct spi_transfer t[2] = {
		{ .delay = { .value = ts->variant->cs_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = tx, .rx_buf = rx, .len = HIDSPI_XFER, .cs_change = 1 },
	};
	struct spi_transfer st = {
		.tx_buf = ts->status,
		.rx_buf = ts->status + HIDSPI_STATUS_LEN,
		.len = HIDSPI_STATUS_LEN,
	};
	int ret;

	lockdep_assert_held(&ts->lock);
	ret = spi_sync_transfer(ts->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;
	*ok = ahts_packet_ok(ts, rx + HIDSPI_PAD);
	usleep_range(HIDSPI_STAGE_US, HIDSPI_STAGE_US + 50);
	memset(ts->status, ts->variant->idle_fill, HIDSPI_PAD);
	put_unaligned_le32(*ok ? HIDSPI_STATUS_OK : HIDSPI_STATUS_NAK,
			   ts->status + HIDSPI_PAD);
	ret = spi_sync_transfer(ts->spi, &st, 1);
	usleep_range(HIDSPI_GAP_US, HIDSPI_GAP_US + 50);
	return ret;
}

static int ahts_request(struct apple_hidspi_ts *ts, u8 dev, u8 u0, u8 u1,
			u8 u2, u16 rsplen)
{
	struct spi_transfer t[3] = {
		{ .delay = { .value = ts->variant->cs_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = ts->tx, .len = HIDSPI_XFER,
		  .delay = { .value = HIDSPI_STAGE_US,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .rx_buf = ts->rx, .len = HIDSPI_STATUS_LEN },
	};
	u8 *pkt = ts->tx + HIDSPI_PAD, *h = pkt + 8;
	bool ok;
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
	if (ts->variant->status_stage)
		return ahts_exchange(ts, ts->tx, ts->rx, &ok);
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

static bool ahts_finger_on(const struct apple_hidspi_finger *f)
{
	return f->state == TOUCH_STARTED || f->state == TOUCH_MOVED;
}

static void ahts_finger(struct apple_hidspi_ts *ts,
			const struct apple_hidspi_finger *f, bool on)
{
	struct input_dev *input = ts->input;
	unsigned int x, y;
	int slot;

	slot = input_mt_get_slot_by_key(input, f->finger);
	if (slot < 0)
		return;
	input_mt_slot(input, slot);
	if (!input_mt_report_slot_state(input, MT_TOOL_FINGER, on))
		return;
	x = ahts_scale((s16)le16_to_cpu(f->abs_x), ts->min_x, ts->max_x,
		       ts->props.max_x);
	y = ahts_scale((s16)le16_to_cpu(f->abs_y), ts->min_y, ts->max_y,
		       ts->props.max_y);
	touchscreen_report_pos(input, &ts->props, x, ts->props.max_y - y, true);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, le16_to_cpu(f->touch_major));
	input_report_abs(input, ABS_MT_TOUCH_MINOR, le16_to_cpu(f->touch_minor));
	input_report_abs(input, ABS_MT_WIDTH_MAJOR, le16_to_cpu(f->tool_major));
	input_report_abs(input, ABS_MT_WIDTH_MINOR, le16_to_cpu(f->tool_minor));
	input_report_abs(input, ABS_MT_ORIENTATION, (s16)le16_to_cpu(f->orientation));
}

/*
 * A finger that is already up (lifted or gone) in the first report read of
 * it: where reports are read rather than handed out one by one, a tap's down
 * reports can pass unread.
 */
static bool ahts_finger_missed(struct apple_hidspi_ts *ts,
			       const struct apple_hidspi_finger *f)
{
	int slot;

	if ((f->state != TOUCH_LIFTED && f->state != TOUCH_GONE) ||
	    test_bit(f->finger, ts->prev))
		return false;
	slot = input_mt_get_slot_by_key(ts->input, f->finger);
	return slot >= 0 && !input_mt_is_active(&ts->input->mt->slots[slot]);
}

static void ahts_touch(struct apple_hidspi_ts *ts, const u8 *pl, size_t len)
{
	const struct apple_hidspi_finger *f;
	struct input_dev *input = ts->input;
	unsigned int on = 0;
	bool missed = false;
	int i, nf;

	if (!ts->input_ready || len <= NUM_FINGERS_OFFSET)
		return;
	nf = pl[NUM_FINGERS_OFFSET];
	if (len < FINGERS_OFFSET || nf > (len - FINGERS_OFFSET) / sizeof(*f))
		return;
	f = (const struct apple_hidspi_finger *)(pl + FINGERS_OFFSET);
	if (nf)
		dev_dbg(&ts->spi->dev, "frame: %d fingers, id %u state %u x %d y %d\n",
			nf, f->finger, f->state, (s16)le16_to_cpu(f->abs_x),
			(s16)le16_to_cpu(f->abs_y));
	else
		dev_dbg(&ts->spi->dev, "frame: no fingers\n");

	if (ts->polled) {
		/* Such a tap goes down where it was lifted, then up below. */
		for (i = 0; i < nf; i++)
			missed |= ahts_finger_missed(ts, &f[i]);
		if (missed) {
			for (i = 0; i < nf; i++)
				ahts_finger(ts, &f[i], ahts_finger_on(&f[i]) ||
					    ahts_finger_missed(ts, &f[i]));
			input_mt_sync_frame(input);
			input_sync(input);
		}
		bitmap_zero(ts->prev, 256);
		for (i = 0; i < nf; i++)
			set_bit(f[i].finger, ts->prev);
	}

	for (i = 0; i < nf; i++) {
		ahts_finger(ts, &f[i], ahts_finger_on(&f[i]));
		on += ahts_finger_on(&f[i]);
	}
	input_mt_sync_frame(input);
	input_sync(input);
	ts->fingers = on;
}

/* The packet the interrupt announces, into irq_rx. */
static int ahts_read(struct apple_hidspi_ts *ts, bool *ok)
{
	struct spi_controller *ctlr = ts->spi->controller;
	struct spi_transfer t[2] = {
		{ .delay = { .value = ts->variant->cs_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = ts->irq_tx, .rx_buf = ts->irq_rx,
		  .len = HIDSPI_XFER, .cs_change = 1 },
	};
	struct spi_transfer st = {
		.tx_buf = ts->status,
		.rx_buf = ts->status + HIDSPI_STATUS_LEN,
		.len = 4,
	};
	struct spi_message m;
	bool pad;
	int ret;

	if (!ts->variant->read_ack) {
		ret = ahts_xfer(ts, ts->irq_tx, ts->irq_rx, HIDSPI_XFER);
		if (ret)
			return ret;
		usleep_range(HIDSPI_GAP_US, HIDSPI_GAP_US + 50);
		*ok = ahts_packet_ok(ts, ts->irq_rx + HIDSPI_PAD);
		return 0;
	}

	/*
	 * As iOS (AppleHIDTransportSPI): the packet, "Inter Stage Us", then
	 * the host's status word with chip select still asserted. A packet
	 * of the firmware (0xdeadbeef padding) is acknowledged even when the
	 * check below cannot verify it, as its boot message.
	 */
	spi_bus_lock(ctlr);
	spi_message_init_with_transfers(&m, t, ARRAY_SIZE(t));
	ret = spi_sync_locked(ts->spi, &m);
	if (!ret) {
		*ok = ahts_packet_ok(ts, ts->irq_rx + HIDSPI_PAD);
		pad = get_unaligned_le32(ts->irq_rx) == HIDSPI_PAD_WORD;
		usleep_range(HIDSPI_STAGE_US, HIDSPI_STAGE_US + 50);
		put_unaligned_le32((*ok || pad) ? HIDSPI_STATUS_OK : HIDSPI_STATUS_NAK,
				   ts->status);
		spi_message_init_with_transfers(&m, &st, 1);
		ret = spi_sync_locked(ts->spi, &m);
		*ok = *ok || pad;
	}
	spi_bus_unlock(ctlr);
	usleep_range(HIDSPI_GAP_US, HIDSPI_GAP_US + 50);
	return ret;
}

/* The valid packet in irq_rx. */
static void ahts_handle(struct apple_hidspi_ts *ts)
{
	u8 *p = ts->irq_rx + HIDSPI_PAD, *m = p + 8;
	size_t len, mlen;

	len = get_unaligned_le16(p + 6);
	if (len > HIDSPI_PKT - 10)
		return;
	dev_dbg(&ts->spi->dev, "packet %*ph\n", 24, p);

	if (p[1] == HIDSPI_DEV_INFO && len == sizeof(hidspi_booted) &&
	    !memcmp(m, hidspi_booted, len)) {
		complete(&ts->booted);
		return;
	}
	if (len < 10)
		return;
	mlen = get_unaligned_le16(m + 6);
	if (mlen + 10 > len || !mlen)
		return;

	if (p[0] == HIDSPI_WRITE && m[0] == HIDSPI_MSG_GET_REPORT &&
	    m[1] == SURFACE_REPORT && m[8] == SURFACE_REPORT)
		ahts_surface(ts, m + 9, mlen - 1);
	else if (p[0] == HIDSPI_READ && p[1] == HIDSPI_DEV_TOUCH &&
		 m[0] == HIDSPI_MSG_INPUT && m[1] == TOUCH_REPORT)
		ahts_touch(ts, m + 8, mlen);
}

/*
 * One read where the host polls: 1 if it brought a new packet, 0 if the one
 * before again, an error if what came is not a packet of the firmware (its
 * padding is 0xdeadbeef; the bootloader answers with its status words there).
 */
static int ahts_poll(struct apple_hidspi_ts *ts)
{
	bool ok;
	int ret;

	guard(mutex)(&ts->lock);
	ret = ahts_read(ts, &ok);
	if (ret)
		return ret;
	if (!ok || get_unaligned_le32(ts->irq_rx) != HIDSPI_PAD_WORD) {
		dev_dbg(&ts->spi->dev, "read rejected: pad %*ph, packet %*ph\n",
			HIDSPI_PAD, ts->irq_rx, 8, ts->irq_rx + HIDSPI_PAD);
		return -EBADMSG;
	}
	ts->bootloader_seen = 0;
	if (!memcmp(ts->irq_rx, ts->last, HIDSPI_XFER))
		return 0;
	memcpy(ts->last, ts->irq_rx, HIDSPI_XFER);
	ahts_handle(ts);
	return 1;
}

/* No report although fingers were down: their last reports passed unread. */
static void ahts_release(struct apple_hidspi_ts *ts)
{
	guard(mutex)(&ts->lock);
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	ts->fingers = 0;
}

/* The firmware stopped answering: load it again, every 5 s until it runs. */
static void ahts_reload(struct apple_hidspi_ts *ts)
{
	dev_warn(&ts->spi->dev, "controller stopped answering; reloading\n");
	while (!kthread_should_stop()) {
		scoped_guard(mutex, &ts->lock) {
			ts->bootloader_seen = 0;
			ahts_power_off(ts);
			if (!ahts_boot(ts))
				return;
		}
		msleep_interruptible(5000);
	}
}

/*
 * Without an interrupt in the device tree the reports are read on a timer,
 * which costs these reads while nothing touches the screen.
 */
static int ahts_poll_thread(void *data)
{
	struct apple_hidspi_ts *ts = data;
	unsigned long last = jiffies;
	unsigned int n, us;
	int ret = 0;

	while (!kthread_should_stop()) {
		for (n = 0; n < POLL_DRAIN; n++) {
			ret = ahts_poll(ts);
			if (ret <= 0)
				break;
			last = jiffies;
		}
		if (ret < 0 && ++ts->bootloader_seen >= RECOVER_AFTER) {
			ahts_reload(ts);
			continue;
		}
		if (ts->fingers &&
		    time_after(jiffies, last + msecs_to_jiffies(RELEASE_MS)))
			ahts_release(ts);
		us = time_before(jiffies, last + msecs_to_jiffies(POLL_ACTIVE_MS)) ?
		     POLL_ACTIVE_US : POLL_IDLE_US;
		usleep_range(us, us + 500);
	}
	return 0;
}

static irqreturn_t ahts_irq(int irq, void *data)
{
	struct apple_hidspi_ts *ts = data;
	bool ok;

	if (ahts_read(ts, &ok))
		return IRQ_HANDLED;
	if (!ok) {
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
	ahts_handle(ts);
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

	cancel_work_sync(&ts->resume_work);
	if (ts->thread)
		kthread_stop(ts->thread);
	cancel_work_sync(&ts->recover);
	ahts_power_off(ts);
}

static int ahts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_hidspi_ts *ts;
	const struct ahts_variant *v;
	struct input_dev *input;
	const char *fw_prefix, *personality;
	unsigned int i;
	int ret;

	v = spi_get_device_match_data(spi);
	if (!v)
		return -ENODEV;
	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	spi_set_drvdata(spi, ts);
	ts->variant = v;
	ts->min_x = v->min_sx;
	ts->max_x = v->max_sx;
	ts->min_y = v->min_sy;
	ts->max_y = v->max_sy;
	init_completion(&ts->booted);
	init_completion(&ts->surface);
	INIT_WORK(&ts->recover, ahts_recover_work);
	INIT_WORK(&ts->resume_work, ahts_resume_work);
	mutex_init(&ts->lock);
	ts->tx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, HIDSPI_XFER + 16, GFP_KERNEL);
	ts->irq_tx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->irq_rx = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->last = devm_kzalloc(dev, HIDSPI_XFER, GFP_KERNEL);
	ts->status = devm_kzalloc(dev, 2 * HIDSPI_STATUS_LEN, GFP_KERNEL);
	if (!ts->tx || !ts->rx || !ts->irq_tx || !ts->irq_rx || !ts->last ||
	    !ts->status)
		return -ENOMEM;
	memset(ts->irq_tx, v->idle_fill, HIDSPI_XFER);

	for (i = 0; i < MAX_SUPPLIES && v->supplies[i]; i++) {
		ts->supplies[i] = devm_regulator_get(dev, v->supplies[i]);
		if (IS_ERR(ts->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(ts->supplies[i]),
					     "no %s supply\n", v->supplies[i]);
	}
	ts->num_supplies = i;
	if (v->clk) {
		ts->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(ts->clk))
			return dev_err_probe(dev, PTR_ERR(ts->clk), "no clock\n");
	}
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
	input->name = v->name;
	input->phys = "apple_hidspi_ts";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x05ac;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, v->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, v->max_y, 0, 0);
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

	ts->polled = v->polled || spi->irq <= 0;
	if (ts->polled) {
		ts->thread = kthread_run(ahts_poll_thread, ts, "apple-hidspi-ts");
		if (IS_ERR(ts->thread)) {
			ret = PTR_ERR(ts->thread);
			ts->thread = NULL;
			return ret;
		}
	} else {
		ret = devm_request_threaded_irq(dev, spi->irq, NULL, ahts_irq,
						IRQF_ONESHOT, "apple-hidspi-ts", ts);
		if (ret)
			return dev_err_probe(dev, ret, "no IRQ\n");
		ts->irq_on = true;
	}
	if (!wait_for_completion_timeout(&ts->booted, msecs_to_jiffies(1000)))
		dev_warn(dev, "no boot message from the firmware\n");
	if (ahts_request(ts, HIDSPI_DEV_TOUCH, HIDSPI_MSG_GET_REPORT,
			 SURFACE_REPORT, 0x00, 64) ||
	    !wait_for_completion_timeout(&ts->surface, msecs_to_jiffies(500))) {
		if (ts->max_x <= ts->min_x)
			return dev_err_probe(dev, -ENODEV,
					     "no surface descriptor: touchscreen left off\n");
		dev_warn(dev, "no surface descriptor; using defaults\n");
	}

	ret = input_register_device(input);
	if (ret)
		return ret;
	ts->input_ready = true;
	return 0;
}

/*
 * The controller is switched off for sleep and its firmware loaded again on
 * resume, as on the iPads' Z2 controllers. Reading over a suspended SPI
 * controller takes an SError, so the poll thread or the interrupt stops
 * first.
 */
static int ahts_suspend(struct device *dev)
{
	struct apple_hidspi_ts *ts = dev_get_drvdata(dev);

	cancel_work_sync(&ts->resume_work);
	if (ts->thread) {
		kthread_stop(ts->thread);
		ts->thread = NULL;
	} else if (ts->irq_on) {
		disable_irq(ts->spi->irq);
		ts->irq_on = false;
	}
	cancel_work_sync(&ts->recover);
	if (ts->fingers)
		ahts_release(ts);
	ahts_power_off(ts);
	return 0;
}

/* Loading the firmware takes about 2 s; resume does not wait for it. */
static void ahts_resume_work(struct work_struct *work)
{
	struct apple_hidspi_ts *ts = container_of(work, struct apple_hidspi_ts,
						  resume_work);
	struct device *dev = &ts->spi->dev;
	int ret;

	reinit_completion(&ts->booted);
	ts->bootloader_seen = 0;
	ret = ahts_boot(ts);
	if (ret) {
		dev_err(dev, "firmware not loaded after resume: %d\n", ret);
		return;
	}
	if (ts->polled) {
		ts->thread = kthread_run(ahts_poll_thread, ts, "apple-hidspi-ts");
		if (IS_ERR(ts->thread))
			ts->thread = NULL;
	} else {
		enable_irq(ts->spi->irq);
		ts->irq_on = true;
	}
}

static int ahts_resume(struct device *dev)
{
	struct apple_hidspi_ts *ts = dev_get_drvdata(dev);

	queue_work(system_dfl_wq, &ts->resume_work);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ahts_pm_ops, ahts_suspend, ahts_resume);

/*
 * iPhone X. HBPP 0x105 in mode 3 and the 200 us CS delay of its MTFW, as
 * measured on a d221; the sizes are the panel's pixels and the surface
 * descriptor that unit's firmware sent.
 */
static const struct ahts_variant ahts_d221 = {
	.name = "Apple iPhone X Touchscreen",
	.supplies = { "ana" },
	.clk = true,
	.hbpp_mode = SPI_MODE_3,
	.hbpp_version = 0x105,
	.cs_delay_us = 200,
	.max_x = 1124,
	.max_y = 2435,
	.min_sx = -218,
	.max_sx = 6003,
	.min_sy = -220,
	.max_sy = 13270,
};

/*
 * iPhone XR. The ADT's multi-touch node: hbpp-version 0x103, and
 * function-power_ana (hv), function-power_ldo (core) and function-clock_enable
 * (clken), the last two switched by the SMC. From Project Cascadia's driver,
 * which runs this controller: blobs of at most 0x3f0 bytes, 20 ms before each
 * acknowledge, a 300 us CS delay, 0xff while reading and the status stage.
 * The bootloader answers in SPI mode 3, as the iPhone X's; in mode 0 its
 * words arrive shifted by one bit. The panel has 828 x 1792 pixels. As iOS
 * (AppleHIDTransportSPI) it answers every packet read with a bare status word
 * in the same chip-select window; unanswered, the firmware returns the same
 * packet again and holds its interrupt (AOP GPIO 19, level low) asserted.
 */
static const struct ahts_variant ahts_n841 = {
	.name = "Apple iPhone XR Touchscreen",
	.supplies = { "hv", "core", "clken" },
	.hbpp_mode = SPI_MODE_3,
	.hbpp_version = 0x103,
	.hbpp_chunk = 0x3f0,
	.hbpp_ack_delay_ms = 20,
	.reset_settle_ms = 30,
	.cs_delay_us = 300,
	.status_stage = true,
	.idle_fill = 0xff,
	.read_ack = true,
	.max_x = 827,
	.max_y = 1791,
	/* What the firmware's surface descriptor gives on an iPhone XR. */
	.min_sx = -226,
	.max_sx = 6223,
	.min_sy = -228,
	.max_sy = 13739,
};

static const struct of_device_id ahts_of_match[] = {
	{ .compatible = "apple,d221-touchscreen", .data = &ahts_d221 },
	{ .compatible = "apple,n841-touchscreen", .data = &ahts_n841 },
	{ }
};
MODULE_DEVICE_TABLE(of, ahts_of_match);

static const struct spi_device_id ahts_spi_id[] = {
	{ "d221-touchscreen", (kernel_ulong_t)&ahts_d221 },
	{ "n841-touchscreen", (kernel_ulong_t)&ahts_n841 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ahts_spi_id);

static struct spi_driver ahts_driver = {
	.driver = {
		.name = "apple-hidspi-ts",
		.of_match_table = ahts_of_match,
		.pm = pm_sleep_ptr(&ahts_pm_ops),
	},
	.id_table = ahts_spi_id,
	.probe = ahts_probe,
};
module_spi_driver(ahts_driver);

MODULE_DESCRIPTION("Apple HIDSPI touchscreen (iPhone X, iPhone XR)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/d221-touch-*.bin");
MODULE_FIRMWARE("apple/n841-touch-*.bin");
