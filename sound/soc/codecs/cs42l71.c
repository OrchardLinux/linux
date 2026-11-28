// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Cirrus Logic CS42L71 headset codec driver
 *
 * Copyright (C) 2026 ricktav
 *
 * The CS42L71 is an Apple-specific Cirrus Logic headset codec on the
 * iPhone 6s and 6s Plus (N66/N71). There is no public datasheet. The
 * control port is AppleCSSPIv2Audio's framed "SPIv2" protocol (24-bit
 * little-endian addresses), reconstructed from iOS 15.8.8 (19H422)
 * AppleCS42L71Audio / AppleEmbeddedAudio. It is not the CS42L81's
 * 6C/6D 5-byte page framing: that map must not be written on this chip.
 *
 * The probe runs iOS's cold enableAudioHW: digital LDO pulldown low, MCLK
 * on, soft reset (0x050000), 0x000005 bit 1 low around the SPI port
 * bring-up, the ID at 0x000000..05, then iOS's register init without the
 * ASP. Playback to the headphones and capture from the headset mic or
 * one of the four built-in mics ("Capture Source") run on the main port
 * (XSP) on pad 0 / MCA0, with the codec as the clock provider. A plug
 * raises the codec interrupt (or is polled without one); the codec then
 * tells headphones from a headset with a mic, and reports the headset's
 * centre button. The receiver, Apple remote volume keys and runtime
 * power management are not done yet.
 *
 * The codec's ASP (pad 2) is shared with the CS35L21 loudspeaker that
 * already works on Linux (MCA2 as clock provider). Never set 0x02000b
 * bit 7, and keep the ASP held and powered off. D2255 PMU GPIO 12 (the
 * digital LDO pulldown), if the device tree gives it, is driven low and
 * left low. MCLK, if the device tree gives one, stays on while the
 * driver is bound.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define CS42L71_OP_PORT		0xa1
#define CS42L71_OP_POLL		0x90
#define CS42L71_OP_ADDR		0x80
#define CS42L71_OP_COUNT	0x81
#define CS42L71_OP_RD8		0xc6
#define CS42L71_OP_WR8		0xb5
#define CS42L71_OP_WR32		0xb2

#define CS42L71_PORT_BRINGUP	0x50
#define CS42L71_PORT_RUN	0x40
#define CS42L71_PORT_ABORT	0x01

#define CS42L71_ST_SPI_READY	0x90	/* bits 7 and 4: write / port ready */
#define CS42L71_ST_READ_READY	0x50	/* bits 6 and 4: read data ready */

#define CS42L71_BRINGUP_POLLS	49
#define CS42L71_READ_TRIES	32
#define CS42L71_WRITE_POLLS	10	/* 90 00 pairs per write frame */
#define CS42L71_WRITE_TRIES	5

/*
 * 0x000000..02 read 42 a7 10 on the iPhone 6s Plus: "42A71" with the
 * Cirrus convention of the part number in the ID registers.
 */
#define CS42L71_DEVID		0x000000
#define CS42L71_ID_REGS		6	/* 0x000000..05 */
#define CS42L71_REVID		0x000005	/* bit 1: iOS clears it for the bring-up */
#define CS42L71_REVID_BRINGUP	BIT(1)
#define CS42L71_SOFT_RESET	0x050000
#define CS42L71_SOFT_RESET_KEY	0xa5

struct cs42l71 {
	struct device *dev;
	struct spi_device *spi;
	struct clk *mclk;
	struct gpio_desc *ldo_pulldown;
	struct mutex lock;	/* all SPI access and the state below */
	unsigned int hp_vol;	/* 0..CS42L71_HP_VOL_MAX, 1 dB steps from -50 dB */
	bool hp_on;
	bool play_on;		/* XSP RX (playback) running */
	bool cap_on;		/* XSP TX (capture) running */
	unsigned int mic_vol;	/* 0..CS42L71_MIC_VOL_MAX, 0.5 dB steps from -6 dB */
	unsigned int mic_src;	/* CS42L71_SRC_*: the Mic Source control */
	int cap_mic;		/* capturing from: -1 headset, else built-in index */
	struct snd_soc_jack jack;
	struct delayed_work jack_work;
	int jack_state;		/* -1: not read yet */
	int jack_type;		/* SND_JACK_HEADPHONE, SND_JACK_HEADSET or 0 */
	struct dentry *dbg_redetect, *dbg_reg;
	int irq;		/* requested interrupt, 0 for none */
	bool polling;		/* poll 0x100077 (no interrupt, or it stormed) */
	unsigned long irq_window;	/* storm guard: start of the window */
	unsigned int irq_count;		/* interrupts in it */
};

/*
 * One full-duplex transfer as its own message. hold_cs on the last (only)
 * transfer of a message leaves chip select asserted, so the next message
 * continues the same SPIv2 sequence. Every caller holds spi_bus_lock()
 * for the whole sequence, so this must use spi_sync_locked(): spi_sync()
 * would take the bus lock again and deadlock. The last transfer of a
 * sequence must not hold CS.
 */
static int cs42l71_xfer(struct cs42l71 *cs, const u8 *tx, u8 *rx,
			unsigned int len, bool hold_cs)
{
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = len,
		.cs_change = hold_cs,
	};
	struct spi_message m;

	spi_message_init_with_transfers(&m, &t, 1);
	return spi_sync_locked(cs->spi, &m);
}

static int cs42l71_port_cmd(struct cs42l71 *cs, u8 val, bool hold_cs)
{
	u8 tx[2] = { CS42L71_OP_PORT, val };

	return cs42l71_xfer(cs, tx, NULL, 2, hold_cs);
}

static int cs42l71_release_cs(struct cs42l71 *cs)
{
	u8 tx[2] = { CS42L71_OP_POLL, 0 };

	return cs42l71_xfer(cs, tx, NULL, 2, false);
}

/*
 * Port bring-up after a cold or soft reset, CS held for the whole
 * sequence: A1 50, 1 ms, poll 90 00 until status has 0x90 (at most
 * 49 x 1 ms), then A1 40. A timeout sends A1 01.
 */
static int cs42l71_port_bringup(struct cs42l71 *cs)
{
	u8 poll_tx[2] = { CS42L71_OP_POLL, 0 };
	u8 poll_rx[2];
	int i, ret;

	spi_bus_lock(cs->spi->controller);

	ret = cs42l71_port_cmd(cs, CS42L71_PORT_BRINGUP, true);
	if (ret)
		goto abort;
	fsleep(1000);

	for (i = 0; i < CS42L71_BRINGUP_POLLS; i++) {
		memset(poll_rx, 0, sizeof(poll_rx));
		ret = cs42l71_xfer(cs, poll_tx, poll_rx, 2, true);
		if (ret)
			goto abort;
		if ((poll_rx[1] & CS42L71_ST_SPI_READY) == CS42L71_ST_SPI_READY) {
			ret = cs42l71_port_cmd(cs, CS42L71_PORT_RUN, false);
			spi_bus_unlock(cs->spi->controller);
			return ret;
		}
		fsleep(1000);
	}

	dev_err(cs->dev,
		"SPIv2 port not ready (status %#x, want bits 7+4)\n",
		poll_rx[1]);
	cs42l71_port_cmd(cs, CS42L71_PORT_ABORT, false);
	spi_bus_unlock(cs->spi->controller);
	return -ETIMEDOUT;

abort:
	cs42l71_port_cmd(cs, CS42L71_PORT_ABORT, false);
	spi_bus_unlock(cs->spi->controller);
	return ret;
}

static void cs42l71_fill_header(u8 *hdr, u32 addr, u16 count)
{
	hdr[0] = CS42L71_OP_ADDR;
	hdr[1] = addr;
	hdr[2] = addr >> 8;
	hdr[3] = addr >> 16;
	hdr[4] = CS42L71_OP_COUNT;
	hdr[5] = count;
	hdr[6] = count >> 8;
}

/*
 * 8-bit read: header 80 aaaaaa 81 nnnn, then C6 00 (00 00)xn C6 00
 * until the last rx byte has 0x50 (up to 32 tries, header sent once),
 * then n+2 dummy clocks. The data is the first n rx bytes of that
 * last transfer.
 */
static int cs42l71_read8(struct cs42l71 *cs, u32 addr, u8 *data, unsigned int n)
{
	u8 hdr[7];
	u8 *payload_tx, *payload_rx, *dummy_tx, *dummy_rx;
	unsigned int payload_len = 2 * n + 4;
	unsigned int dummy_len = n + 2;
	unsigned int i;
	int ret, try;
	bool held = false;

	if (!n || n > 16)
		return -EINVAL;

	payload_tx = kzalloc(payload_len + payload_len + dummy_len + dummy_len,
			     GFP_KERNEL);
	if (!payload_tx)
		return -ENOMEM;
	payload_rx = payload_tx + payload_len;
	dummy_tx = payload_rx + payload_len;
	dummy_rx = dummy_tx + dummy_len;

	payload_tx[0] = CS42L71_OP_RD8;
	payload_tx[payload_len - 2] = CS42L71_OP_RD8;

	cs42l71_fill_header(hdr, addr, n);

	spi_bus_lock(cs->spi->controller);
	held = true;

	ret = cs42l71_xfer(cs, hdr, NULL, sizeof(hdr), true);
	if (ret)
		goto out;

	ret = -ETIMEDOUT;
	for (try = 0; try < CS42L71_READ_TRIES; try++) {
		memset(payload_rx, 0, payload_len);
		ret = cs42l71_xfer(cs, payload_tx, payload_rx, payload_len, true);
		if (ret)
			goto out;
		if ((payload_rx[payload_len - 1] & CS42L71_ST_READ_READY) ==
		    CS42L71_ST_READ_READY) {
			ret = 0;
			break;
		}
		ret = -ETIMEDOUT;
	}
	if (ret) {
		dev_err(cs->dev, "SPIv2 read %#08x never ready (last status %#04x, want bits 6+4)\n",
			addr, payload_rx[payload_len - 1]);
		goto out;
	}

	ret = cs42l71_xfer(cs, dummy_tx, dummy_rx, dummy_len, false);
	held = false;
	if (!ret) {
		for (i = 0; i < n; i++)
			data[i] = dummy_rx[i];
	}

out:
	if (held)
		cs42l71_release_cs(cs);
	spi_bus_unlock(cs->spi->controller);
	kfree(payload_tx);
	return ret;
}

/* True if any 90 00 pair in rx[] answered with both bits of @want. */
static bool cs42l71_polls_ready(const u8 *rx, unsigned int pairs, u8 want,
				u8 *last)
{
	unsigned int i;

	for (i = 0; i < pairs; i++) {
		*last = rx[2 * i + 1];
		if ((*last & want) == want)
			return true;
	}
	return false;
}

/*
 * Write of one register, as AppleCSSPIv2Audio does it, CS held:
 * 80 aaaaaa 81 0100 (90 00)x10; if no status byte has 0x90, the 20 poll
 * bytes again, up to 4 more times. Then B5 vv (8-bit) or B2 vvvvvvvv
 * (32-bit, LE) and (90 00)x10, which must answer 0x90 too. Nothing
 * reaches the register unless the codec said its bus is ready.
 */
static int cs42l71_write_data(struct cs42l71 *cs, u32 addr, u8 op,
			      const u8 *data, unsigned int dlen, u16 count)
{
	u8 tx[7 + 2 * CS42L71_WRITE_POLLS] = { };
	u8 rx[7 + 2 * CS42L71_WRITE_POLLS];
	u8 dtx[5 + 2 * CS42L71_WRITE_POLLS] = { };
	u8 drx[5 + 2 * CS42L71_WRITE_POLLS];
	unsigned int i, dpoll = 1 + dlen;
	u8 last = 0;
	int ret, try;
	bool ready;

	if (dlen > 4)
		return -EINVAL;

	cs42l71_fill_header(tx, addr, count);
	for (i = 0; i < CS42L71_WRITE_POLLS; i++)
		tx[7 + 2 * i] = CS42L71_OP_POLL;
	dtx[0] = op;
	memcpy(dtx + 1, data, dlen);
	for (i = 0; i < CS42L71_WRITE_POLLS; i++)
		dtx[dpoll + 2 * i] = CS42L71_OP_POLL;

	spi_bus_lock(cs->spi->controller);

	memset(rx, 0, sizeof(rx));
	ret = cs42l71_xfer(cs, tx, rx, sizeof(tx), true);
	if (ret)
		goto out;
	ready = cs42l71_polls_ready(rx + 7, CS42L71_WRITE_POLLS,
				    CS42L71_ST_SPI_READY, &last);
	for (try = 1; !ready && try < CS42L71_WRITE_TRIES; try++) {
		memset(rx, 0, sizeof(rx));
		ret = cs42l71_xfer(cs, tx + 7, rx, 2 * CS42L71_WRITE_POLLS, true);
		if (ret)
			goto out;
		ready = cs42l71_polls_ready(rx, CS42L71_WRITE_POLLS,
					    CS42L71_ST_SPI_READY, &last);
	}
	if (!ready) {
		dev_err(cs->dev, "SPIv2 write %#08x: bus not ready (status %#04x), not written\n",
			addr, last);
		ret = -ETIMEDOUT;
		goto out;
	}

	memset(drx, 0, sizeof(drx));
	ret = cs42l71_xfer(cs, dtx, drx, dpoll + 2 * CS42L71_WRITE_POLLS, true);
	if (ret)
		goto out;
	if (!cs42l71_polls_ready(drx + dpoll, CS42L71_WRITE_POLLS,
				 CS42L71_ST_SPI_READY, &last)) {
		dev_err(cs->dev, "SPIv2 write %#08x: no ready after the data (status %#04x)\n",
			addr, last);
		ret = -EIO;
	}

out:
	cs42l71_release_cs(cs);
	spi_bus_unlock(cs->spi->controller);
	return ret;
}

static int cs42l71_write8(struct cs42l71 *cs, u32 addr, u8 val)
{
	return cs42l71_write_data(cs, addr, CS42L71_OP_WR8, &val, 1, 1);
}

/* 32-bit word registers (0x020001..04 and others): B2 and one LE word */
static int cs42l71_write32(struct cs42l71 *cs, u32 addr, u32 val)
{
	u8 b[4] = { val, val >> 8, val >> 16, val >> 24 };

	return cs42l71_write_data(cs, addr, CS42L71_OP_WR32, b, 4, 1);
}

static int cs42l71_update8(struct cs42l71 *cs, u32 addr, u8 mask, u8 val)
{
	u8 old;
	int ret;

	ret = cs42l71_read8(cs, addr, &old, 1);
	if (ret)
		return ret;
	if (((old & ~mask) | (val & mask)) == old)
		return 0;
	return cs42l71_write8(cs, addr, (old & ~mask) | (val & mask));
}

/*
 * The steps of iOS's cold enableAudioHW between MCLK and the port
 * bring-up: soft reset, then 0x000005 bit 1 low. Each step logs and the
 * probe goes on, so one boot shows how far the codec answers.
 */
static void cs42l71_soft_reset(struct cs42l71 *cs, u8 *ctl5, bool *have_ctl5)
{
	int ret;

	ret = cs42l71_write8(cs, CS42L71_SOFT_RESET, CS42L71_SOFT_RESET_KEY);
	if (!ret) {
		fsleep(10000);
		ret = cs42l71_write8(cs, CS42L71_SOFT_RESET, 0);
		fsleep(1000);
	}
	dev_info(cs->dev, "soft reset %s (%d)\n", ret ? "failed" : "done", ret);

	*have_ctl5 = false;
	ret = cs42l71_read8(cs, CS42L71_REVID, ctl5, 1);
	if (ret) {
		dev_info(cs->dev, "0x000005 not readable before the bring-up (%d)\n", ret);
		return;
	}
	*have_ctl5 = true;
	ret = cs42l71_write8(cs, CS42L71_REVID, *ctl5 & ~CS42L71_REVID_BRINGUP);
	dev_info(cs->dev, "0x000005 was %#04x, bit 1 cleared (%d)\n", *ctl5, ret);
}

static int cs42l71_read_id(struct cs42l71 *cs, u8 id[CS42L71_ID_REGS])
{
	unsigned int i;
	int ret;

	for (i = 0; i < CS42L71_ID_REGS; i++) {
		ret = cs42l71_read8(cs, CS42L71_DEVID + i, &id[i], 1);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Playback to the headphones, from iOS 15.8.8's AppleCS42L71Audio: XSP is
 * the main port on pad 0, the codec provides its clocks (SCLK = MCLK x
 * N / M = 96 fs, LRCK high for 48), and XSP RX slots 0 and 1 feed mixers
 * A and B, which drive HP left and right.
 *
 * The init below is iOS's register init without everything on the ASP:
 * the ASP (pad 2) carries the CS35L21 speaker, which Linux clocks from
 * MCA2. 0x02000b..0c and 0x020009..0a are never written, the ASP clock
 * stays held (0x040002 bit 1) and powered down (0x010005 bit 1).
 */
struct cs42l71_upd {
	u32 reg;
	u8 mask;
	u8 val;
};

static const struct cs42l71_upd cs42l71_init_seq[] = {
	{ 0x000005, 0x0e, 0x08 },	/* [3:1] = 100b */
	{ 0x03002c, 0x08, 0x00 },
	{ 0x040002, 0x02, 0x02 },	/* ASP clock held */
	{ 0x010005, 0x02, 0x02 },	/* ASP powered down */
	{ 0x000008, 0x02, 0x02 },
	{ 0x330010, 0x0f, 0x09 },
	{ 0x330018, 0x0f, 0x09 },
	{ 0x000008, 0x01, 0x01 },
	{ 0x040002, 0x04, 0x04 },	/* MSP clock held */
	{ 0x020005, 0x80, 0x80 },	/* XSP: the codec provides the clocks */
	{ 0x020006, 0x07, 0x02 },	/* XSP format 2, as iOS */
	{ 0x040002, 0x01, 0x01 },	/* XSP clock held until a stream */
	{ 0x020006, 0x80, 0x80 },
	{ 0x000008, 0x1c, 0x00 },
	{ 0x32001a, 0x3f, 0x3a },	/* output sources: mixer A/B */
	{ 0x32001b, 0x3f, 0x3b },
	{ 0x32001c, 0x3f, 0x3a },
	{ 0x32001d, 0x3f, 0x3b },
	{ 0x470001, 0x03, 0x01 },
	{ 0x210004, 0x10, 0x00 },
	{ 0x320020, 0x3f, 0x1c },	/* mixer A: XSP RX ch1, sidetone */
	{ 0x320021, 0x3f, 0x19 },
	{ 0x320022, 0x3f, 0x1d },	/* mixer B: XSP RX ch2, sidetone */
	{ 0x320023, 0x3f, 0x19 },
	{ 0x300000, 0x3f, 0x3f },	/* mixer attenuators muted */
	{ 0x300001, 0x3f, 0x3f },
	{ 0x300002, 0x3f, 0x3f },
	{ 0x300003, 0x3f, 0x3f },
	{ 0x000009, 0x80, 0x80 },
	{ 0x220001, 0x0f, 0x02 },
	{ 0x200003, 0x7f, 0x40 },	/* HP gain parked (-64 dB) */
	{ 0x200004, 0x7f, 0x40 },
	{ 0x110001, 0x0c, 0x00 },
	{ 0x010000, 0xc1, 0x00 },	/* power up (enableAudioHW) */
	{ 0x010000, 0x02, 0x00 },	/* no DSP */
	{ 0x200005, 0xf4, 0x00 },	/* HP clamp released */
};

#define CS42L71_XSP_FRAME	96
#define CS42L71_HP_VOL_MIN_DB	-50
#define CS42L71_HP_VOL_MAX	50	/* -50 dB + 50 = 0 dB: the cap */
#define CS42L71_HP_VOL_DEFAULT	30	/* -20 dB */

static int cs42l71_run_seq(struct cs42l71 *cs, const struct cs42l71_upd *seq,
			   unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = cs42l71_update8(cs, seq[i].reg, seq[i].mask, seq[i].val);
		if (ret) {
			dev_err(cs->dev, "%#08x mask %#04x = %#04x failed: %d\n",
				seq[i].reg, seq[i].mask, seq[i].val, ret);
			return ret;
		}
	}
	return 0;
}

static int cs42l71_init_regs(struct cs42l71 *cs)
{
	int ret;

	ret = cs42l71_run_seq(cs, cs42l71_init_seq, ARRAY_SIZE(cs42l71_init_seq));
	if (ret)
		return ret;
	/* XSP frame: LRCK high 48, frame 96 (32-bit words, [10:0]) */
	ret = cs42l71_write32(cs, 0x020003, CS42L71_XSP_FRAME / 2 - 1);
	if (!ret)
		ret = cs42l71_write32(cs, 0x020004, CS42L71_XSP_FRAME - 1);
	return ret;
}

/* SCLK = 12 MHz x N / M = 96 fs (ConfigureXSPClocks) */
static const struct {
	unsigned int rate;
	u16 n, m;
} cs42l71_xsp_nm[] = {
	{ 48000, 0x030, 0x07d },
	{ 44100, 0x1b9, 0x4e2 },
	{ 32000, 0x020, 0x07d },
	{ 24000, 0x018, 0x07d },
	{ 22050, 0x1b9, 0x9c4 },
	{ 16000, 0x010, 0x07d },
	{ 12000, 0x00c, 0x07d },
	{ 11025, 0x1b9, 0x1388 },
	{ 8000, 0x008, 0x07d },
};

static u8 cs42l71_hp_reg(unsigned int vol)
{
	return (CS42L71_HP_VOL_MIN_DB + (int)vol) & 0x7f;
}

/* bd6de0: gain, HP power-up, 55 ms, output power-up, mixers open */
static int cs42l71_hp_on(struct cs42l71 *cs)
{
	u8 g = cs42l71_hp_reg(cs->hp_vol);
	int ret;

	ret = cs42l71_update8(cs, 0x200003, 0x7f, g);
	if (!ret)
		ret = cs42l71_update8(cs, 0x200004, 0x7f, g);
	if (!ret)
		ret = cs42l71_update8(cs, 0x010001, 0x02, 0x00);
	if (ret)
		return ret;
	msleep(55);
	ret = cs42l71_update8(cs, 0x010007, 0x01, 0x00);
	if (!ret)
		ret = cs42l71_update8(cs, 0x300000, 0x3f, 0x00);
	if (!ret)
		ret = cs42l71_update8(cs, 0x300002, 0x3f, 0x00);
	if (!ret)
		cs->hp_on = true;
	return ret;
}

/* dba34 with enable = 0, up to the XSP clocks */
static void cs42l71_hp_off(struct cs42l71 *cs)
{
	static const struct cs42l71_upd off[] = {
		{ 0x200003, 0x7f, 0x40 },
		{ 0x200004, 0x7f, 0x40 },
		{ 0x300000, 0x3f, 0x3f },
		{ 0x300002, 0x3f, 0x3f },
		{ 0x010007, 0x01, 0x01 },
		{ 0x010001, 0x0e, 0x0e },
	};

	cs42l71_run_seq(cs, off, ARRAY_SIZE(off));
	cs->hp_on = false;
}

/* XSP clocks for @rate, with RX and TX powered down (bd67d8 order) */
static int cs42l71_xsp_clocks(struct cs42l71 *cs, unsigned int rate)
{
	static const struct cs42l71_upd hold[] = {
		{ 0x420014, 0x04, 0x00 },	/* XSP TX block off */
		{ 0x010004, 0x02, 0x02 },	/* XSP TX down */
		{ 0x010004, 0x01, 0x01 },	/* XSP RX down while reprogrammed */
		{ 0x040002, 0x01, 0x01 },
		{ 0x040002, 0x01, 0x00 },	/* XSP clock released (iOS order) */
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(cs42l71_xsp_nm); i++)
		if (cs42l71_xsp_nm[i].rate == rate)
			break;
	if (i == ARRAY_SIZE(cs42l71_xsp_nm))
		return -EINVAL;

	ret = cs42l71_run_seq(cs, hold, ARRAY_SIZE(hold));
	if (!ret)
		ret = cs42l71_write32(cs, 0x020001, cs42l71_xsp_nm[i].n);
	if (!ret)
		ret = cs42l71_write32(cs, 0x020002, cs42l71_xsp_nm[i].m);
	return ret;
}

/* Nothing on the XSP any more: hold its clock (iOS's idle state) */
static void cs42l71_xsp_idle(struct cs42l71 *cs)
{
	cs42l71_update8(cs, 0x010004, 0x01, 0x01);
	cs42l71_update8(cs, 0x040002, 0x01, 0x01);
}

static int cs42l71_rx_start(struct cs42l71 *cs)
{
	static const struct cs42l71_upd rx[] = {
		{ 0x010004, 0x01, 0x00 },	/* XSP RX up */
		{ 0x450000, 0xff, 0x00 },	/* RX ch1 at bit 0 */
		{ 0x45000a, 0x07, 0x05 },	/* 2 bytes, enabled */
		{ 0x450001, 0xff, 0x10 },	/* RX ch2 at bit 16 */
		{ 0x45000b, 0x07, 0x05 },
	};

	return cs42l71_run_seq(cs, rx, ARRAY_SIZE(rx));
}

static void cs42l71_rx_stop(struct cs42l71 *cs)
{
	static const struct cs42l71_upd stop[] = {
		{ 0x010004, 0x01, 0x01 },
		{ 0x45000a, 0x04, 0x00 },
		{ 0x45000b, 0x04, 0x00 },
	};

	cs42l71_run_seq(cs, stop, ARRAY_SIZE(stop));
}

/*
 * Capture from the headset mic, from iOS's 'adex' path (bd816c, bd86c8):
 * the headset-input ADC (source 0x16) goes to XSP TX slots 0 and 1, which
 * land in MCA0 RX slots 0 and 1. The ADC's sample-rate code is
 * 0x330000/08; the gain is 0x140009 (coarse 0/+10/+20 dB in bits 7:6, a
 * signed 0.5 dB fine step in bits 5:0); 0x01000a bit 2 powers the headset
 * mic path. Nothing here touches the ASP.
 */
#define CS42L71_MIC_VOL_MAX	75	/* -6 dB + 75 x 0.5 dB = +31.5 dB */
#define CS42L71_MIC_VOL_DEFAULT	24	/* +6 dB */
#define CS42L71_MIC_SRC_HPIN	0x16
#define CS42L71_TX_SLOTS	10

/*
 * The four built-in mics (iOS imic, lmic, smic, fmic): each has an ADC
 * power-down bit in 0x010002 (1 = down), a bias block whose bits 3:2 set
 * the level (iOS uses 2, 0 is off) and a source code for the XSP slots.
 * The ADC goes up before the bias, then 50 ms settle (iOS order).
 */
static const struct {
	u8 pd;
	u32 bias;
	u8 src;
} cs42l71_builtin[] = {
	{ 0x01, 0x110001, 0x0e },	/* ADC1A, imic */
	{ 0x02, 0x11000a, 0x0f },	/* ADC1B, lmic */
	{ 0x04, 0x110013, 0x10 },	/* ADC2A, smic */
	{ 0x08, 0x11001c, 0x11 },	/* ADC2B, fmic */
};

enum {
	CS42L71_SRC_AUTO,	/* the headset mic when one is in, else built-in */
	CS42L71_SRC_HEADSET,
	CS42L71_SRC_MIC1,	/* .. CS42L71_SRC_MIC1 + 3 */
};

#define CS42L71_BUILTIN_DEFAULT	0	/* imic, iOS's first mic; likely the bottom one */

static const struct {
	unsigned int rate;
	u8 code;
} cs42l71_adc_rate[] = {
	{ 8000, 1 }, { 11025, 2 }, { 12000, 3 }, { 16000, 4 }, { 22050, 5 },
	{ 24000, 6 }, { 32000, 7 }, { 44100, 8 }, { 48000, 9 },
};

static u8 cs42l71_mic_reg(unsigned int v)
{
	if (v <= 36)
		return (v - 12) & 0x3f;		/* -6 .. +12 dB */
	if (v <= 52)
		return 0x40 | (v - 32);		/* +10 dB coarse */
	return 0x80 | (v - 52);			/* +20 dB coarse */
}

static int cs42l71_tx_start(struct cs42l71 *cs, unsigned int rate)
{
	static const struct cs42l71_upd adc_on[] = {
		{ 0x010002, 0xc0, 0xc0 },	/* no DSP decimator (bd921c) */
		{ 0x000008, 0x18, 0x00 },
		{ 0x010003, 0x08, 0x00 },	/* headset-input ADC up */
	};
	static const struct cs42l71_upd sidetone_off[] = {
		{ 0x310007, 0x03, 0x03 },
		{ 0x300001, 0x3f, 0x3f },
		{ 0x300003, 0x3f, 0x3f },
		{ 0x310005, 0x01, 0x00 },
		{ 0x010001, 0x01, 0x01 },
	};
	unsigned int i;
	u8 code = 0, src;
	int ret;

	for (i = 0; i < ARRAY_SIZE(cs42l71_adc_rate); i++)
		if (cs42l71_adc_rate[i].rate == rate)
			code = cs42l71_adc_rate[i].code;
	if (!code)
		return -EINVAL;

	if (cs->mic_src == CS42L71_SRC_HEADSET ||
	    (cs->mic_src == CS42L71_SRC_AUTO &&
	     cs->jack_type == SND_JACK_HEADSET))
		cs->cap_mic = -1;
	else if (cs->mic_src == CS42L71_SRC_AUTO)
		cs->cap_mic = CS42L71_BUILTIN_DEFAULT;
	else
		cs->cap_mic = cs->mic_src - CS42L71_SRC_MIC1;

	ret = cs42l71_update8(cs, 0x330000, 0x0f, code);
	if (!ret)
		ret = cs42l71_update8(cs, 0x330008, 0x0f, code);
	if (!ret)
		ret = cs42l71_run_seq(cs, adc_on, ARRAY_SIZE(adc_on));
	if (cs->cap_mic < 0) {
		if (!ret)
			ret = cs42l71_write8(cs, 0x140009,
					     cs42l71_mic_reg(cs->mic_vol));
		if (!ret)
			ret = cs42l71_update8(cs, 0x01000a, 0x04, 0x00);
		src = CS42L71_MIC_SRC_HPIN;
	} else {
		if (!ret)
			ret = cs42l71_update8(cs, 0x010002,
					      cs42l71_builtin[cs->cap_mic].pd,
					      0x00);
		if (!ret)
			ret = cs42l71_update8(cs,
					      cs42l71_builtin[cs->cap_mic].bias,
					      0x0c, 0x08);
		src = cs42l71_builtin[cs->cap_mic].src;
	}
	for (i = 0; !ret && i < CS42L71_TX_SLOTS; i++) {
		bool on = i < 2;

		ret = cs42l71_update8(cs, 0x420000 + i, 0xff, 16 * i);
		if (!ret)
			ret = cs42l71_update8(cs, 0x42000a + i, 0x03, 0x01);
		if (!ret && on)
			ret = cs42l71_update8(cs, 0x320006 + i, 0x3f, src);
		if (!ret)
			ret = cs42l71_update8(cs, 0x42000a + i, 0x04,
					      on ? 0x04 : 0x00);
	}
	if (!ret)
		ret = cs42l71_update8(cs, 0x010004, 0x02, 0x00);	/* TX up */
	if (!ret)
		ret = cs42l71_update8(cs, 0x420014, 0x04, 0x04);	/* TX on */
	if (!ret)
		ret = cs42l71_run_seq(cs, sidetone_off,
				      ARRAY_SIZE(sidetone_off));
	if (!ret)
		msleep(50);	/* mic and ADC settle; iOS has no wait here */
	return ret;
}

static void cs42l71_adc_off(struct cs42l71 *cs)
{
	static const struct cs42l71_upd off[] = {
		{ 0x010003, 0x08, 0x08 },
		{ 0x140009, 0xc0, 0x00 },
		{ 0x140009, 0x3f, 0x00 },
		{ 0x01000a, 0x04, 0x04 },	/* headset mic path down */
	};

	cs42l71_run_seq(cs, off, ARRAY_SIZE(off));
}

static void cs42l71_tx_stop(struct cs42l71 *cs)
{
	unsigned int i;

	for (i = 0; i < 2; i++)
		cs42l71_update8(cs, 0x42000a + i, 0x04, 0x00);
	cs42l71_update8(cs, 0x420014, 0x04, 0x00);
	cs42l71_update8(cs, 0x010004, 0x02, 0x02);
	if (cs->cap_mic >= 0) {
		cs42l71_update8(cs, cs42l71_builtin[cs->cap_mic].bias, 0x0c,
				0x00);
		cs42l71_update8(cs, 0x010002, cs42l71_builtin[cs->cap_mic].pd,
				cs42l71_builtin[cs->cap_mic].pd);
	}
	cs42l71_adc_off(cs);
}

static void cs42l71_stream_stop(struct cs42l71 *cs, bool playback)
{
	if (playback && cs->play_on) {
		if (cs->hp_on)
			cs42l71_hp_off(cs);
		cs42l71_rx_stop(cs);
		cs->play_on = false;
	} else if (!playback && cs->cap_on) {
		cs42l71_tx_stop(cs);
		cs->cap_on = false;
	}
	if (!cs->play_on && !cs->cap_on)
		cs42l71_xsp_idle(cs);
}

static int cs42l71_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct cs42l71 *cs = snd_soc_component_get_drvdata(dai->component);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int rate = params_rate(params);
	int ret = 0;

	mutex_lock(&cs->lock);
	cs42l71_stream_stop(cs, playback);	/* hw_params may repeat */
	/* the other direction already runs the clocks at this rate */
	if (!cs->play_on && !cs->cap_on)
		ret = cs42l71_xsp_clocks(cs, rate);
	if (!ret && playback)
		ret = cs42l71_rx_start(cs);
	else if (!ret)
		ret = cs42l71_tx_start(cs, rate);
	if (!ret) {
		if (playback)
			cs->play_on = true;
		else
			cs->cap_on = true;
	} else {
		if (!playback)
			cs42l71_adc_off(cs);
		if (!cs->play_on && !cs->cap_on)
			cs42l71_xsp_idle(cs);
	}
	mutex_unlock(&cs->lock);
	if (ret)
		dev_err(cs->dev, "XSP %s start at %u Hz failed: %d\n",
			playback ? "playback" : "capture", rate, ret);
	return ret;
}

static int cs42l71_hw_free(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct cs42l71 *cs = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&cs->lock);
	cs42l71_stream_stop(cs,
			    substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	mutex_unlock(&cs->lock);
	return 0;
}

/* Called from prepare (unmute) and hw_free (mute), in process context */
static int cs42l71_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct cs42l71 *cs = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	mutex_lock(&cs->lock);
	if (!mute && !cs->hp_on)
		ret = cs42l71_hp_on(cs);
	else if (mute && cs->hp_on)
		cs42l71_hp_off(cs);
	mutex_unlock(&cs->lock);
	return ret;
}

static int cs42l71_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The XSP format is iOS's fixed one; only the clock roles must fit. */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBP_CFP)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops cs42l71_xsp_ops = {
	.hw_params = cs42l71_hw_params,
	.hw_free = cs42l71_hw_free,
	.mute_stream = cs42l71_mute_stream,
	.set_fmt = cs42l71_set_fmt,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver cs42l71_dai = {
	.name = "cs42l71-xsp",
	.playback = {
		.stream_name = "XSP Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "XSP Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l71_xsp_ops,
	.symmetric_rate = 1,
};

static const DECLARE_TLV_DB_SCALE(cs42l71_hp_tlv, CS42L71_HP_VOL_MIN_DB * 100,
				  100, 0);

static int cs42l71_hp_vol_info(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = CS42L71_HP_VOL_MAX;
	return 0;
}

static int cs42l71_hp_vol_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = cs->hp_vol;
	return 0;
}

static int cs42l71_hp_vol_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);
	long v = ucontrol->value.integer.value[0];
	int ret = 0;

	if (v < 0 || v > CS42L71_HP_VOL_MAX)
		return -EINVAL;

	mutex_lock(&cs->lock);
	if (v == cs->hp_vol) {
		mutex_unlock(&cs->lock);
		return 0;
	}
	cs->hp_vol = v;
	if (cs->hp_on) {
		u8 g = cs42l71_hp_reg(v);

		ret = cs42l71_update8(cs, 0x200003, 0x7f, g);
		if (!ret)
			ret = cs42l71_update8(cs, 0x200004, 0x7f, g);
	}
	mutex_unlock(&cs->lock);
	return ret ?: 1;
}

static const DECLARE_TLV_DB_SCALE(cs42l71_mic_tlv, -600, 50, 0);

static int cs42l71_mic_vol_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = CS42L71_MIC_VOL_MAX;
	return 0;
}

static int cs42l71_mic_vol_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = cs->mic_vol;
	return 0;
}

static int cs42l71_mic_vol_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);
	long v = ucontrol->value.integer.value[0];
	int ret = 0;

	if (v < 0 || v > CS42L71_MIC_VOL_MAX)
		return -EINVAL;

	mutex_lock(&cs->lock);
	if (v == cs->mic_vol) {
		mutex_unlock(&cs->lock);
		return 0;
	}
	cs->mic_vol = v;
	if (cs->cap_on && cs->cap_mic < 0)
		ret = cs42l71_write8(cs, 0x140009, cs42l71_mic_reg(v));
	mutex_unlock(&cs->lock);
	return ret ?: 1;
}

static const char * const cs42l71_mic_src_texts[] = {
	"Auto", "Headset", "Mic 1", "Mic 2", "Mic 3", "Mic 4",
};

static int cs42l71_mic_src_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	return snd_ctl_enum_info(uinfo, 1, ARRAY_SIZE(cs42l71_mic_src_texts),
				 cs42l71_mic_src_texts);
}

static int cs42l71_mic_src_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);

	ucontrol->value.enumerated.item[0] = cs->mic_src;
	return 0;
}

/* Takes effect at the next capture start. */
static int cs42l71_mic_src_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l71 *cs = snd_soc_component_get_drvdata(c);
	unsigned int v = ucontrol->value.enumerated.item[0];
	bool changed;

	if (v >= ARRAY_SIZE(cs42l71_mic_src_texts))
		return -EINVAL;

	mutex_lock(&cs->lock);
	changed = v != cs->mic_src;
	cs->mic_src = v;
	mutex_unlock(&cs->lock);
	return changed;
}

static const struct snd_kcontrol_new cs42l71_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Capture Source",
		.info = cs42l71_mic_src_info,
		.get = cs42l71_mic_src_get,
		.put = cs42l71_mic_src_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Headphone Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = cs42l71_hp_vol_info,
		.get = cs42l71_hp_vol_get,
		.put = cs42l71_hp_vol_put,
		.tlv.p = cs42l71_hp_tlv,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Headset Mic Capture Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = cs42l71_mic_vol_info,
		.get = cs42l71_mic_vol_get,
		.put = cs42l71_mic_vol_put,
		.tlv.p = cs42l71_mic_tlv,
	},
};

/* Jack pins: endpoints only, so the jack gets the usual control names */
static const struct snd_soc_dapm_widget cs42l71_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Headphone"),
	SND_SOC_DAPM_INPUT("Headset Mic"),
};

static struct snd_soc_jack_pin cs42l71_jack_pins[] = {
	{ .pin = "Headphone", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Headset Mic", .mask = SND_JACK_MICROPHONE },
};

/*
 * Jack detection, as iOS's AppleCS42L71Mikey does it: the "HTD" block at
 * 0x100070 is powered on after the register init, and 0x100077 bit 7 says
 * whether a plug is in. Plug events (0x10007b bit 6 in, bit 5 out, clear
 * on read, unmasked in 0x100079) raise the codec interrupt, AP GPIO 139,
 * level low. The contacts bounce for about 50 ms, so 0x100077 is read
 * 200 ms after the last event, and a new plug is then classified. Without
 * the interrupt, or if it storms, 0x100077 is polled instead.
 */
#define CS42L71_HTD_IRQ_MASK1	0x100079	/* bits 7:5, 1 = masked */
#define CS42L71_HTD_IRQ_MASK2	0x10007a
#define CS42L71_HTD_IRQ_STAT1	0x10007b	/* clear on read */
#define CS42L71_HTD_IRQ_STAT2	0x10007c
#define CS42L71_HTD_PLUG	(BIT(6) | BIT(5))	/* 6: in, 5: out */
#define CS42L71_JACK_DEBOUNCE_MS	200	/* contacts bounce for ~50 ms */
#define CS42L71_IRQ_STORM	50	/* interrupts per second */
#define CS42L71_HTD_PRESENT	0x100077
#define CS42L71_JACK_POLL_MS	500

static int cs42l71_htd_on(struct cs42l71 *cs)
{
	int ret;

	ret = cs42l71_update8(cs, 0x100075, 0xc0, 0x80);
	if (!ret)
		ret = cs42l71_write8(cs, 0x100073, 0xc0);	/* no invert */
	if (!ret)
		ret = cs42l71_update8(cs, 0x100071, 0x21, 0x20);
	if (!ret)
		ret = cs42l71_write8(cs, 0x100076, 0x8f);
	if (!ret)
		ret = cs42l71_update8(cs, 0x01000a, 0x04, 0x04);	/* mic path off */
	return ret;
}

/* c6e98: write an HTD mode, wait as iOS does for it */
static void cs42l71_htd_mode(struct cs42l71 *cs, u8 mode, u8 htd70)
{
	cs42l71_write8(cs, 0x100070, 0x01);
	cs42l71_update8(cs, 0x100071, 0xc0, 0xc0);
	cs42l71_update8(cs, 0x100074, 0xff, mode);
	msleep(mode & 4 ? 35 : mode & 1 ? 10 : 15);
	if (htd70 != 0x01)
		cs42l71_write8(cs, 0x100070, htd70);	/* bit 7: after the wait */
}

static void cs42l71_htd_masks(struct cs42l71 *cs, u8 mask1, u8 mask2)
{
	cs42l71_update8(cs, CS42L71_HTD_IRQ_MASK1, 0xe0, mask1);
	cs42l71_update8(cs, CS42L71_HTD_IRQ_MASK2, 0xfb, mask2);
}

static void cs42l71_htd_clear(struct cs42l71 *cs)
{
	u8 v;

	cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT1, &v, 1);
	cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT2, &v, 1);
	cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT2, &v, 1);
}

/* Detached or headphones only: mode 3, plug events only */
static void cs42l71_htd_idle(struct cs42l71 *cs)
{
	cs42l71_htd_mode(cs, 0x03, 0x01);
	cs42l71_htd_masks(cs, 0x80, 0xff);
}

/*
 * Headset detection after a plug (iOS c85dc/c86dc, then c45f8): with the
 * HP output clamped, sense which ring carries the mic (CTIA or OMTP),
 * switch the mic matrix 0x01000b to it, then let the HTD classify the
 * plug in 0x100078: bit 0 a 3-pole plug, bit 1 a mic, bits 6:4 = 1..4 an
 * Apple remote. The class is polled; iOS waits for status2 bits 4:3.
 */
static int cs42l71_mic_detect(struct cs42l71 *cs)
{
	unsigned int pass, t;
	u8 done = 0, r = 0, q = 0;
	bool omtp;

	cs42l71_write8(cs, 0x010009, 0xd7);
	cs42l71_write8(cs, 0x01000b, 0xf3);
	cs42l71_update8(cs, 0x01000a, 0xfb, 0x00);
	cs42l71_update8(cs, 0x110025, 0x03, 0x00);
	cs42l71_htd_mode(cs, 0x06, 0x01);
	cs42l71_htd_masks(cs, 0x80, 0xff);
	cs42l71_read8(cs, 0x03000c, &done, 1);

	for (pass = 0; pass < 2; pass++) {
		cs42l71_update8(cs, 0x200005, 0xf4, 0x84);	/* HP clamp */
		udelay(100);
		cs42l71_update8(cs, 0x01000a, 0xfb, 0xc0);
		usleep_range(1000, 1200);
		cs42l71_read8(cs, 0x03000c, &done, 1);
		cs42l71_read8(cs, 0x01000c, &r, 1);
		dev_dbg(cs->dev, "mic sense %u: 0x03000c = %#04x, 0x01000c = %#04x\n",
			pass, done, r);
		r = done & BIT(1) ? r & 0x03 : 0;
		if (r == 3 && pass == 0) {
			cs42l71_write8(cs, 0x01000b, 0xf3);
			cs42l71_update8(cs, 0x01000a, 0xfb, 0x00);
			cs42l71_update8(cs, 0x110025, 0x03, 0x00);
			cs42l71_update8(cs, 0x200005, 0xf4, 0x00);
			udelay(100);
			continue;
		}
		cs42l71_update8(cs, 0x200005, 0xf4, 0x00);
		udelay(100);
		break;
	}
	omtp = r == 1;

	cs42l71_write8(cs, 0x01000b, omtp ? 0x59 : 0xa6);
	cs42l71_update8(cs, 0x01000a, 0xfb, 0x00);
	cs42l71_update8(cs, 0x110025, 0x03, 0x00);
	cs42l71_htd_mode(cs, 0x03, 0x01);
	cs42l71_update8(cs, 0x100075, 0x3f, 0x36);
	cs42l71_htd_clear(cs);
	/* mode 0x9e: bit 7 is written with a 0x100076 pulse */
	cs42l71_update8(cs, 0x100074, 0x7f, 0x1e);
	cs42l71_write8(cs, 0x100076, 0x80);
	usleep_range(1000, 1200);
	cs42l71_update8(cs, 0x100074, 0xff, 0x9e);
	cs42l71_write8(cs, 0x100076, 0x8f);
	cs42l71_htd_clear(cs);
	cs42l71_write8(cs, 0x100070, 0x01);
	msleep(35);

	/* iOS classifies on status2 bits 4:3, "detection done" */
	for (t = 0; t < 50; t++) {
		u8 st2 = 0, q0 = q;

		cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT2, &st2, 1);
		if (cs42l71_read8(cs, 0x100078, &q, 1))
			break;
		if (st2 || q != q0)
			dev_dbg(cs->dev, "classify %u ms: 0x10007c = %#04x, 0x100078 = %#04x\n",
				35 + 20 * t, st2, q);
		if (st2 & 0x18)
			break;
		msleep(20);
	}
	dev_info(cs->dev, "plug: mic sense %u (%s), 0x100078 = %#04x %s after %u ms\n",
		 r, omtp ? "OMTP" : "CTIA", q, t < 50 ? "done" : "timeout",
		 35 + 20 * t);

	if (q & BIT(0))
		return SND_JACK_HEADPHONE;		/* 3-pole */
	if (q & BIT(1) || (((q >> 4) & 7) >= 1 && ((q >> 4) & 7) <= 4))
		return SND_JACK_HEADSET;
	return SND_JACK_HEADPHONE;			/* mic open */
}

/* SingleButton (c4bac state 7): centre button and mic changes unmasked */
static void cs42l71_htd_headset(struct cs42l71 *cs)
{
	cs42l71_update8(cs, 0x100075, 0x3f, 0x36);
	cs42l71_htd_mode(cs, 0x1f, 0xc1);
	cs42l71_htd_clear(cs);
	cs42l71_htd_masks(cs, 0x00, 0xfc);
}

static void cs42l71_unplugged(struct cs42l71 *cs)
{
	if (cs->cap_on)
		cs42l71_adc_off(cs);	/* userspace stops the stream */
	cs42l71_htd_idle(cs);
	cs42l71_write8(cs, 0x01000b, 0x00);
	cs42l71_update8(cs, 0x01000a, 0xfb, 0x00);
	cs42l71_update8(cs, 0x110025, 0x03, 0x00);
}

static void cs42l71_jack_update(struct cs42l71 *cs)
{
	u8 v;
	int present, type = 0;

	mutex_lock(&cs->lock);
	if (cs42l71_read8(cs, CS42L71_HTD_PRESENT, &v, 1))
		goto out;
	present = !!(v & BIT(7));
	if (present == cs->jack_state)
		goto out;
	cs->jack_state = present;

	if (!present) {
		cs42l71_unplugged(cs);
	} else if (cs->hp_on) {
		/* iOS's clamp-free polarity test is not done here */
		type = SND_JACK_HEADPHONE;
	} else {
		type = cs42l71_mic_detect(cs);
		if (type == SND_JACK_HEADSET)
			cs42l71_htd_headset(cs);
		else
			cs42l71_htd_idle(cs);
	}
	dev_info(cs->dev, "%s (0x100077 = %#04x)\n",
		 !present ? "unplugged" :
		 type == SND_JACK_HEADSET ? "headset with mic" : "headphones",
		 v);
	cs->jack_type = type;
	snd_soc_jack_report(&cs->jack, type, SND_JACK_HEADSET | SND_JACK_BTN_0);
out:
	mutex_unlock(&cs->lock);
}

static void cs42l71_jack_work(struct work_struct *work)
{
	struct cs42l71 *cs = container_of(to_delayed_work(work), struct cs42l71,
					  jack_work);

	cs42l71_jack_update(cs);
	if (cs->polling)
		schedule_delayed_work(&cs->jack_work,
				      msecs_to_jiffies(CS42L71_JACK_POLL_MS));
}

static irqreturn_t cs42l71_irq(int irq, void *data)
{
	struct cs42l71 *cs = data;
	u8 st1 = 0, st2 = 0;

	mutex_lock(&cs->lock);
	cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT1, &st1, 1);
	cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT2, &st2, 1);
	mutex_unlock(&cs->lock);
	dev_dbg(cs->dev, "irq: 0x10007b = %#04x, 0x10007c = %#04x\n", st1, st2);

	/* a wrong trigger or a status that does not clear: back to polling */
	if (time_after(jiffies, cs->irq_window + HZ)) {
		cs->irq_window = jiffies;
		cs->irq_count = 0;
	}
	if (++cs->irq_count > CS42L71_IRQ_STORM) {
		dev_warn(cs->dev, "interrupt storm (0x10007b = %#04x), polling the jack instead\n",
			 st1);
		disable_irq_nosync(irq);
		cs->polling = true;
		schedule_delayed_work(&cs->jack_work, 0);
		return IRQ_HANDLED;
	}

	if (!st1 && !st2)
		return IRQ_NONE;
	/* read 0x100077 once the contacts have settled */
	if (st1 & CS42L71_HTD_PLUG)
		mod_delayed_work(system_percpu_wq, &cs->jack_work,
				 msecs_to_jiffies(CS42L71_JACK_DEBOUNCE_MS));
	/* the centre button (status2 bit 0 pressed, bit 1 released) */
	if (cs->jack_type == SND_JACK_HEADSET && (st2 & 0x03)) {
		bool down = st2 & BIT(0) && !(st2 & BIT(1));

		dev_dbg(cs->dev, "button %s\n", down ? "down" : "up");
		snd_soc_jack_report(&cs->jack, down ? SND_JACK_BTN_0 : 0,
				    SND_JACK_BTN_0);
	}
	if (st1 & BIT(7))
		dev_dbg(cs->dev, "mic change (0x10007b bit 7)\n");
	return IRQ_HANDLED;
}

/* Read-only view of the detect registers, to decode the plug bits. */
static int cs42l71_jack_regs_show(struct seq_file *m, void *unused)
{
	struct cs42l71 *cs = dev_get_drvdata(m->private);
	static const u32 regs[] = { 0x010009, 0x01000a, 0x01000b, 0x01000c,
				    0x03000c, 0x03000d, 0x110025, 0x140009,
				    0x330000, 0x330008 };
	unsigned int i;
	u8 v;

	mutex_lock(&cs->lock);
	for (i = 0x100070; i <= 0x10007c; i++) {
		if (i == 0x10007b || i == 0x10007c)
			continue;	/* IRQ status: clear on read */
		if (!cs42l71_read8(cs, i, &v, 1))
			seq_printf(m, "%06x: %02x\n", i, v);
	}
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		if (!cs42l71_read8(cs, regs[i], &v, 1))
			seq_printf(m, "%06x: %02x\n", regs[i], v);
	mutex_unlock(&cs->lock);
	return 0;
}

/*
 * Bring-up aids: "cs42l71-redetect" classifies the plug again, and
 * "cs42l71-reg" reads ("addr") or writes ("addr val") one register. Writes
 * to the ASP (page 0x41, 0x02000b, 0x010005, 0x040002 bit 1) are refused.
 */
static u32 cs42l71_dbg_addr;

static ssize_t cs42l71_redetect_write(struct file *f, const char __user *buf,
				      size_t len, loff_t *ppos)
{
	struct cs42l71 *cs = f->private_data;

	mutex_lock(&cs->lock);
	cs->jack_state = -1;
	mutex_unlock(&cs->lock);
	cs42l71_jack_update(cs);
	return len;
}

static const struct file_operations cs42l71_redetect_fops = {
	.open = simple_open,
	.write = cs42l71_redetect_write,
};

static ssize_t cs42l71_reg_write(struct file *f, const char __user *ubuf,
				 size_t len, loff_t *ppos)
{
	struct cs42l71 *cs = f->private_data;
	char buf[32];
	u32 addr, val;
	int n, ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	n = sscanf(buf, "%x %x", &addr, &val);
	if (n < 1 || addr > 0xffffff)
		return -EINVAL;
	cs42l71_dbg_addr = addr;
	if (n == 1)
		return len;
	if (val > 0xff || (addr >> 16) == 0x41 || addr == 0x02000b ||
	    addr == 0x010005)
		return -EPERM;
	mutex_lock(&cs->lock);
	if (addr == 0x040002)
		ret = cs42l71_update8(cs, addr, 0xfd, val);	/* keep bit 1 */
	else
		ret = cs42l71_write8(cs, addr, val);
	mutex_unlock(&cs->lock);
	dev_info(cs->dev, "debugfs write %06x = %02x (%d)\n", addr, val, ret);
	return ret ?: len;
}

static ssize_t cs42l71_reg_read(struct file *f, char __user *ubuf,
				size_t len, loff_t *ppos)
{
	struct cs42l71 *cs = f->private_data;
	char buf[24];
	u8 v = 0;
	int ret, n;

	if (*ppos)
		return 0;
	mutex_lock(&cs->lock);
	ret = cs42l71_read8(cs, cs42l71_dbg_addr, &v, 1);
	mutex_unlock(&cs->lock);
	if (ret)
		return ret;
	n = scnprintf(buf, sizeof(buf), "%06x: %02x\n", cs42l71_dbg_addr, v);
	return simple_read_from_buffer(ubuf, len, ppos, buf, n);
}

static const struct file_operations cs42l71_reg_fops = {
	.open = simple_open,
	.read = cs42l71_reg_read,
	.write = cs42l71_reg_write,
};

static int cs42l71_component_probe(struct snd_soc_component *component)
{
	struct cs42l71 *cs = snd_soc_component_get_drvdata(component);
	int ret;

	ret = snd_soc_card_jack_new_pins(component->card, "Headset",
					 SND_JACK_HEADSET | SND_JACK_BTN_0,
					 &cs->jack, cs42l71_jack_pins,
					 ARRAY_SIZE(cs42l71_jack_pins));
	if (ret)
		return ret;
	snd_jack_set_key(cs->jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);

	cs->jack_state = -1;
	INIT_DELAYED_WORK(&cs->jack_work, cs42l71_jack_work);
	cs->dbg_redetect = debugfs_create_file("cs42l71-redetect", 0200, NULL,
					       cs, &cs42l71_redetect_fops);
	cs->dbg_reg = debugfs_create_file("cs42l71-reg", 0600, NULL, cs,
					  &cs42l71_reg_fops);

	cs->polling = true;
	if (cs->spi->irq > 0) {
		u8 v;

		/* old events out, then the plug events unmasked */
		mutex_lock(&cs->lock);
		cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT1, &v, 1);
		cs42l71_read8(cs, CS42L71_HTD_IRQ_STAT2, &v, 1);
		mutex_unlock(&cs->lock);
		cs->irq_window = jiffies;
		ret = request_threaded_irq(cs->spi->irq, NULL, cs42l71_irq,
					   IRQF_ONESHOT, "cs42l71", cs);
		if (ret) {
			dev_warn(cs->dev, "no jack interrupt (%d), polling\n", ret);
		} else {
			cs->irq = cs->spi->irq;
			mutex_lock(&cs->lock);
			ret = cs42l71_update8(cs, CS42L71_HTD_IRQ_MASK1,
					      CS42L71_HTD_PLUG, 0);
			mutex_unlock(&cs->lock);
			if (ret)
				dev_warn(cs->dev, "plug events not unmasked (%d), polling\n",
					 ret);
			else
				cs->polling = false;
		}
	}

	/* the first state is read here; the interrupt reports changes */
	schedule_delayed_work(&cs->jack_work, 0);
	return 0;
}

static void cs42l71_component_remove(struct snd_soc_component *component)
{
	struct cs42l71 *cs = snd_soc_component_get_drvdata(component);

	if (cs->irq) {
		mutex_lock(&cs->lock);
		cs42l71_htd_masks(cs, 0xe0, 0xff);
		mutex_unlock(&cs->lock);
		free_irq(cs->irq, cs);
		cs->irq = 0;
	}
	cs->polling = false;
	debugfs_remove(cs->dbg_redetect);
	debugfs_remove(cs->dbg_reg);
	cancel_delayed_work_sync(&cs->jack_work);
}

static const struct snd_soc_component_driver cs42l71_component = {
	.name = "cs42l71",
	.probe = cs42l71_component_probe,
	.remove = cs42l71_component_remove,
	.controls = cs42l71_controls,
	.num_controls = ARRAY_SIZE(cs42l71_controls),
	.dapm_widgets = cs42l71_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs42l71_widgets),
	.endianness = 1,
};

static void cs42l71_mclk_off(void *data)
{
	clk_disable_unprepare(data);
}

static int cs42l71_probe(struct spi_device *spi)
{
	struct cs42l71 *cs;
	u8 id[CS42L71_ID_REGS] = { };
	bool have_ctl5;
	u8 ctl5;
	int ret;

	cs = devm_kzalloc(&spi->dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = &spi->dev;
	cs->spi = spi;
	mutex_init(&cs->lock);
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	spi_set_drvdata(spi, cs);

	/*
	 * iOS's enableAudioHW starts by driving the PMU's digital LDO pulldown
	 * line low. The state the boot loader left it in is logged first.
	 */
	cs->ldo_pulldown = devm_gpiod_get_optional(cs->dev, "cirrus,ldo-pulldown",
						   GPIOD_ASIS);
	if (IS_ERR(cs->ldo_pulldown))
		return dev_err_probe(cs->dev, PTR_ERR(cs->ldo_pulldown),
				     "no LDO pulldown line\n");
	if (cs->ldo_pulldown) {
		dev_info(cs->dev, "LDO pulldown was %d, driving it low\n",
			 gpiod_get_raw_value_cansleep(cs->ldo_pulldown));
		ret = gpiod_direction_output(cs->ldo_pulldown, 0);
		if (ret)
			return dev_err_probe(cs->dev, ret,
					     "LDO pulldown does not switch\n");
		fsleep(10000);
	}

	/* iOS has the 12 MHz MCLK running from here on; here only for the probe. */
	cs->mclk = devm_clk_get_optional(cs->dev, "mclk");
	if (IS_ERR(cs->mclk))
		return dev_err_probe(cs->dev, PTR_ERR(cs->mclk), "no MCLK\n");
	ret = clk_prepare_enable(cs->mclk);
	if (ret)
		return dev_err_probe(cs->dev, ret, "MCLK does not start\n");
	if (cs->mclk)
		dev_info(cs->dev, "MCLK %lu Hz\n", clk_get_rate(cs->mclk));
	fsleep(10000);

	cs42l71_soft_reset(cs, &ctl5, &have_ctl5);

	ret = cs42l71_port_bringup(cs);
	if (!ret && have_ctl5) {
		ret = cs42l71_write8(cs, CS42L71_REVID, ctl5 | CS42L71_REVID_BRINGUP);
		dev_info(cs->dev, "0x000005 bit 1 set again (%d)\n", ret);
	}
	if (!ret)
		ret = cs42l71_read_id(cs, id);
	if (ret) {
		clk_disable_unprepare(cs->mclk);
		return ret;
	}
	/* The codec runs from MCLK from here on (no runtime PM yet). */
	ret = devm_add_action_or_reset(cs->dev, cs42l71_mclk_off, cs->mclk);
	if (ret)
		return ret;

	dev_info(cs->dev, "0x000000..05 = %*ph\n", CS42L71_ID_REGS, id);

	if (id[0] != 0x42 || id[1] != 0xa7 || (id[2] & 0xf0) != 0x10) {
		dev_err(cs->dev, "not a CS42L71 (ID %*ph)\n", 3, id);
		return -ENODEV;
	}

	dev_info(cs->dev, "CS42L71 (%02x%02x%x)\n", id[0], id[1], id[2] >> 4);

	ret = cs42l71_init_regs(cs);
	if (!ret)
		ret = cs42l71_htd_on(cs);
	if (ret)
		return dev_err_probe(cs->dev, ret, "register init failed\n");

	debugfs_create_devm_seqfile(cs->dev, "cs42l71-jack-regs", NULL,
				    cs42l71_jack_regs_show);

	cs->hp_vol = CS42L71_HP_VOL_DEFAULT;
	cs->mic_vol = CS42L71_MIC_VOL_DEFAULT;
	cs->mic_src = CS42L71_SRC_AUTO;
	cs->cap_mic = -1;

	return devm_snd_soc_register_component(cs->dev, &cs42l71_component,
					       &cs42l71_dai, 1);
}

static const struct of_device_id cs42l71_of_match[] = {
	{ .compatible = "cirrus,cs42l71" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l71_of_match);

static struct spi_driver cs42l71_driver = {
	.driver = {
		.name = "cs42l71",
		.of_match_table = cs42l71_of_match,
	},
	.probe = cs42l71_probe,
};
module_spi_driver(cs42l71_driver);

MODULE_DESCRIPTION("ASoC CS42L71 headset codec driver");
MODULE_AUTHOR("ricktav <82380971+ricktav@users.noreply.github.com>");
MODULE_LICENSE("Dual MIT/GPL");
