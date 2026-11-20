// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple S8000/S8003 (A9) MCA, "mca-version 4", as an ASoC CPU DAI
 *
 * Copyright (C) 2026 Berkay Kösebay
 *
 * The register layout and the programming sequence follow iOS 16.7.10's
 * AppleAE2MCA::_setIISConfig_v4. The driver also handles the two
 * neighbours iOS drives from the MCA driver: the per-MCA MCLK control word,
 * exported as a clock (the codec's MCLK), and the MCA's route in the shared
 * i2s-switch (AppleAE2I2SSwitch3).
 *
 * The T7001 (A8X) MCA is "mca-version 1": the same frame, slot, sample and
 * channel config fields as version 4 in the A5-era register layout, with
 * one interrupt status word and two interrupt enables instead of a status
 * and a mask register (iOS 15.8 19H422's AppleAE2MCA::_setIISConfig_v1 and its
 * v0/v1 stop and interrupt paths). The offsets that differ are in a
 * per-version table, struct s8000_mca_regs. The S5L8960X (A7) MCA is version
 * 1 as well.
 *
 * Samples are moved by SmartIO DMA (endpoint 0x28 for MCA0 TX, 0x29 for
 * MCA0 RX) through the generic dmaengine PCM. The SmartIO shim record of
 * the endpoint is passed as the slave config's peripheral_config.
 *
 * TX and RX share the frame (clocks, slot width, run bit) and the IRQ: the
 * first prepared direction programs the whole block, the other one only
 * resets and programs its own half, and the frame stops with the last.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/soc/apple/sio.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define MCA_VERSION		0x00
#define MCA_FIFO_SIZE		0x04
#define  MCA_FIFO_SIZE_TX	GENMASK(9, 0)
#define MCA_SAMPLE		0x08
#define MCA_RESET		0x0c
#define  MCA_RESET_TX		BIT(0)
#define  MCA_RESET_RX		BIT(1)
#define MCA_FRAME		0x10
#define  MCA_FRAME_MASTER	BIT(0)
#define  MCA_FRAME_RUN		BIT(1)
#define  MCA_FRAME_POL		BIT(2)	/* set if format flag bit 0 != provider */
#define  MCA_FRAME_FMT		BIT(3)	/* set unless format flag bit 1 */
#define  MCA_FRAME_DUTY		BIT(6)
#define  MCA_FRAME_MCLK_DIV	GENMASK(11, 8)
#define  MCA_FRAME_SLOTS	GENMASK(15, 12)
#define  MCA_FRAME_WIDTH	GENMASK(23, 20)
#define  MCA_FRAME_HALF		GENMASK(31, 24)
#define MCA_PAD			0x14
#define MCA_IRQ_STATUS		0x1c
#define MCA_IRQ_MASK		0x20
#define  MCA_IRQ_TX_UNDERFLOW	BIT(0)
#define  MCA_IRQ_TX_OVERFLOW	BIT(1)
#define  MCA_IRQ_TX_LOW_WATER	BIT(2)
#define  MCA_IRQ_RX_UNDERFLOW	BIT(3)
#define  MCA_IRQ_RX_OVERFLOW	BIT(4)
#define  MCA_IRQ_RX_HIGH_WATER	BIT(5)
#define  MCA_IRQ_FRAME_ERROR	BIT(6)
#define  MCA_IRQ_BCLK		BIT(7)
#define  MCA_IRQ_NO_BCLK	BIT(8)
#define  MCA_IRQ_FSYNC		BIT(9)
#define  MCA_IRQ_NO_FSYNC	BIT(10)
#define  MCA_IRQ_SHORT_FSYNC	BIT(11)
#define  MCA_IRQ_RATE		BIT(12)
#define  MCA_IRQ_ALL		GENMASK(12, 0)
#define  MCA_IRQ_ERRORS		(MCA_IRQ_TX_UNDERFLOW | MCA_IRQ_TX_OVERFLOW | \
				 MCA_IRQ_RX_UNDERFLOW | MCA_IRQ_RX_OVERFLOW | \
				 MCA_IRQ_FRAME_ERROR)
/*
 * iOS v4 unmasks only the four FIFO error causes (mask 0x1fe4); split per
 * direction so a direction's causes are only open while it is prepared.
 */
#define  MCA_IRQ_TX_CAUSES	(MCA_IRQ_TX_UNDERFLOW | MCA_IRQ_TX_OVERFLOW)
#define  MCA_IRQ_RX_CAUSES	(MCA_IRQ_RX_UNDERFLOW | MCA_IRQ_RX_OVERFLOW)
#define  MCA_IRQ_TX_ALL		GENMASK(2, 0)
#define  MCA_IRQ_RX_ALL		GENMASK(5, 3)
#define MCA_FLUSH		0x24
#define MCA_TX_CFG		0x60
#define MCA_TX_FIFO_CFG		0x64
#define MCA_TX_SLOTS		0x68
#define MCA_TX_SLOTS_HI		0x6c
#define MCA_RX_CFG		0x80
#define MCA_RX_FIFO_CFG		0x84
#define MCA_RX_SLOTS		0x88
#define MCA_RX_SLOTS_HI		0x8c
#define  MCA_XX_CFG_EN		BIT(0)

/*
 * Version 1 (T7001), from iOS 15.8 _setIISConfig_v1: VERSION, FIFO_SIZE,
 * SAMPLE and PAD as above (FIFO_SIZE reads 0x80), FRAME at 0x0c with the
 * version 4 fields except the MCLK divider (SCLK = NCO / (2 * (div + 1))),
 * TX/RX_CFG with their own bit encoding (0x9 base instead of 0x33, the
 * clock edge bit at 1 instead of 2, data delay in bits 16:8 as before).
 * RESET (TX bit 0, RX bit 1) and FLUSH (bit 0) clear themselves.
 */
#define MCA_V1_FRAME		0x0c
#define MCA_V1_IRQ_EN		0x10
#define  MCA_V1_IRQ_EN_FIFO	BIT(0)	/* both directions' FIFO errors */
#define  MCA_V1_IRQ_EN_FRAME	BIT(1)	/* frame error (fixed frame length) */
#define MCA_V1_TX_CFG		0x18
#define MCA_V1_TX_CTL		0x1c
#define MCA_V1_TX_FIFO_CFG	0x20
#define MCA_V1_TX_SLOTS		0x24
#define MCA_V1_RX_CFG		0x2c
#define MCA_V1_RX_CTL		0x30
#define  MCA_V1_XX_CTL_IOS	0xd	/* written by iOS on T7001, meaning unknown */
#define  MCA_V1_XX_CTL_A7	0xf	/* the same, on S5L8960X */
#define MCA_V1_RX_FIFO_CFG	0x34
#define MCA_V1_RX_SLOTS		0x38
/* causes latch whether they are enabled or not; write 1 to clear */
#define MCA_V1_IRQ_STATUS	0x40
#define  MCA_V1_IRQ_TX_UNDERFLOW	BIT(12)
#define  MCA_V1_IRQ_TX_OVERFLOW	BIT(13)
#define  MCA_V1_IRQ_RX_UNDERFLOW	BIT(28)
#define  MCA_V1_IRQ_RX_OVERFLOW	BIT(29)
#define  MCA_V1_IRQ_FRAME_ERROR	BIT(31)
#define MCA_V1_RESET		0x44
#define MCA_V1_FLUSH		0x48
/* the version 4 causes MCA_V1_IRQ_EN_FIFO stands for */
#define MCA_IRQ_V1_FIFO		(MCA_IRQ_TX_CAUSES | MCA_IRQ_RX_CAUSES)

/* iOS _setIISConfig_v4 values for MCA0 */
#define MCA_TX_CFG_I2S		(BIT(8) | 0x33 | 0x4)	/* enabled: 0x137 */
#define MCA_RX_CFG_I2S		(BIT(8) | 0x33)		/* enabled: 0x133 */
#define MCA_RX_FIFO_CFG_IOS	0x00030004	/* RX depth 4 (mcaKeepDMABusy) */

#define MCA_POLL_US		10
#define MCA_TIMEOUT_US		10000

/*
 * i2s-switch v3 (AppleAE2I2SSwitch3): one register per node, pinN at N,
 * audN (= MCA N) at 5 + N, dspN at 10 + N, dspc at 14, then one output
 * enable register per pin at 15 + N. A node register holds the node code
 * of its clock source in [4:0] and of its data source in [12:8] and
 * [20:16] (route direction 0 and 1). Node codes: audN = N, pinN = 8 + N,
 * 0x18 = none. Reset values from the switch's +0x580 method.
 */
#define I2S_SW_PIN(n)		(4 * (n))
#define I2S_SW_AUD(n)		(4 * (5 + (n)))
#define I2S_SW_PIN_OE(n)	(4 * (15 + (n)))
#define I2S_SW_NODES		5
#define  I2S_SW_CLK_SRC		GENMASK(4, 0)
#define  I2S_SW_DATA_SRC0	GENMASK(12, 8)
#define  I2S_SW_DATA_SRC1	GENMASK(20, 16)
#define  I2S_SW_PIN_OE_CLK	BIT(0)
#define  I2S_SW_PIN_OE_DATA	BIT(4)
#define I2S_SW_CODE_AUD(n)	(n)
#define I2S_SW_CODE_PIN(n)	(8 + (n))
#define I2S_SW_CODE_NONE	0x18
#define I2S_SW_NODE_DEFAULT	0x00181818

/*
 * The AOP's i2s-switch (ADT aop-i2s-switch,t8010) passes the headphone link
 * on to the AOP pads the codec is wired to. Its layout is not decoded: the
 * DT gives the words to write (see apple,aop-i2s-route).
 */
#define S8000_MCA_AOP_WRITES	4

#define MCA_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000)
#define MCA_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* The registers whose offset or meaning depends on the MCA version */
struct s8000_mca_regs {
	u32 reset;
	u32 flush;
	u32 frame;
	u32 tx_cfg, tx_fifo_cfg, tx_slots, tx_slots_hi;	/* _hi: 0 if absent */
	u32 rx_cfg, rx_fifo_cfg, rx_slots, rx_slots_hi;
	/*
	 * Version 1: MCA_V1_IRQ_* instead of MCA_IRQ_STATUS/MASK, the TX/RX
	 * CTL words, and iOS waits for the flush only, not for the reset, when
	 * it sets up the whole block.
	 */
	bool v1;
	unsigned int nco_per_sclk;	/* bitclock NCO rate / SCLK rate */
};

static const struct s8000_mca_regs mca_v4_regs = {
	.reset = MCA_RESET,
	.flush = MCA_FLUSH,
	.frame = MCA_FRAME,
	.tx_cfg = MCA_TX_CFG,
	.tx_fifo_cfg = MCA_TX_FIFO_CFG,
	.tx_slots = MCA_TX_SLOTS,
	.tx_slots_hi = MCA_TX_SLOTS_HI,
	.rx_cfg = MCA_RX_CFG,
	.rx_fifo_cfg = MCA_RX_FIFO_CFG,
	.rx_slots = MCA_RX_SLOTS,
	.rx_slots_hi = MCA_RX_SLOTS_HI,
	.nco_per_sclk = 1,
};

/* iOS forces the MCLK/SCLK ratio of version 0/1 to at least 2 */
static const struct s8000_mca_regs mca_v1_regs = {
	.reset = MCA_V1_RESET,
	.flush = MCA_V1_FLUSH,
	.frame = MCA_V1_FRAME,
	.tx_cfg = MCA_V1_TX_CFG,
	.tx_fifo_cfg = MCA_V1_TX_FIFO_CFG,
	.tx_slots = MCA_V1_TX_SLOTS,
	.rx_cfg = MCA_V1_RX_CFG,
	.rx_fifo_cfg = MCA_V1_RX_FIFO_CFG,
	.rx_slots = MCA_V1_RX_SLOTS,
	.v1 = true,
	.nco_per_sclk = 2,
};

/*
 * The variants are _setIISConfig_v4 (J81: _v1) evaluated for the board's
 * iisConfig (the "reg" of the ADT's audio-data node below the MCA):
 *
 *   J71t mca0 10 00 00 00 02 00 01 00 ...    consumer, I2S, 2 x 16 bit
 *   J71b mca2 01 13 00 00 08 00 01 00 ...    provider, 8 x 16 bit, slots 0-3
 *   J171 mca4 01 13 00 00 02 00 01 00 ...    provider, 2 slots, slots 0-1
 *   J81  mca3 01 12 00 00 08 00 00 00 ...    provider, 8 x 16 bit in a
 *                                            256-bit frame, slots 0-1
 *
 * Bytes 0xc/0xe of the J71b record are the 128-bit frame and its 1-bit sync
 * pulse: the amplifiers run in TDM mode, as MAX98357 calls it.
 */
struct s8000_mca_hw {
	u32 version;
	const struct s8000_mca_regs *regs;
	bool clock_provider;
	/*
	 * Run the provider clocks from prepare to hw_free, not from trigger
	 * to trigger: a codec that clocks itself from SCLK (CS42L42 family)
	 * locks its PLL when ASoC unmutes it, which is after prepare and
	 * before trigger, and runs from SCLK until ASoC mutes it again,
	 * which is after trigger stop and before hw_free. The MCA only
	 * drives SCLK while TX is enabled, so TX stays enabled as well.
	 */
	bool clocks_at_prepare;
	unsigned int dai_format;	/* SND_SOC_DAIFMT_* */
	u32 v1_ctl;			/* version 1: TX/RX_CTL word */
	unsigned int channels;
	unsigned int slots;
	unsigned int frame_bits;	/* 0: slots x slot width, no pad */
	u32 tx_slots;
	u32 tx_cfg;
	u32 rx_cfg;
	u32 rx_slots;
	u64 formats;
	u32 frame_flags;		/* FRAME bits 6:2 */
	unsigned int sync_bits;		/* 0: half a frame */
	u32 stream_irqs;		/* error causes open while TX is prepared */
	/*
	 * Leave the interrupt line alone. On the iPad mini 2 (J85), A7 MCA1's
	 * line stayed asserted with both version 1 enables clear and no known
	 * cause in the status register: 100000 unhandled interrupts during
	 * the first prepare, which starved the touchscreen's SPI firmware
	 * upload.
	 */
	bool no_irq;
	/*
	 * Divisor of every sample on its way to the DMA buffer, 1 for none.
	 * A board without volume hardware bounds its output here, where no
	 * client can bypass it; the PCM then has no mmap access.
	 */
	unsigned int attenuation;
	/* AppleAE2MCA _getMCLKClkCon: nclk / 2 (0x18) | output enable (0x40) */
	u32 mclk_ctl;
	unsigned long mclk_rate;
	/*
	 * SmartIO shim words of the MCA endpoints (ADT dma-channels; TX and
	 * RX carry the same words): transfer control for 16- and 32-bit
	 * samples, timeout, FIFO and trigger level in bytes.
	 */
	u32 sio_xfer16;
	u32 sio_xfer32;
	u32 sio_timeout;
	u32 sio_fifo;
	u32 sio_trigger;
};

struct s8000_mca {
	struct device *dev;
	const struct s8000_mca_hw *hw;
	const struct s8000_mca_regs *regs;
	unsigned int slot_width;
	void __iomem *base;
	void __iomem *mclk_reg;
	struct clk_hw mclk_hw;
	struct clk *bitclk;
	bool bitclk_enabled;

	struct regmap *sw;	/* i2s-switch, NULL if not routed */
	u32 sw_aud;
	u32 sw_pin;

	/*
	 * AOP i2s-switch writes (apple,aop-i2s-route): the pads of a codec
	 * wired to the AOP are reached through the AOP's own switch.
	 */
	struct {
		struct regmap *map;
		u32 reg, val;
	} aop[S8000_MCA_AOP_WRITES];
	unsigned int n_aop;

	int irq;
	u32 fifo_tx;
	bool capture;		/* SmartIO "rx" channel present */
	struct snd_soc_dai_driver dai_drv;
	struct snd_pcm_hardware pcm_hw;
	struct snd_dmaengine_pcm_config pcm_config;
	struct snd_dmaengine_dai_dma_data dma_tx, dma_rx;
	struct apple_sio_dma_config sio_cfg[2];	/* per SNDRV_PCM_STREAM_* */

	/* computed in hw_params, written in prepare */
	u32 sample, frame, pad, tx_cfg, tx_fifo_cfg, tx_slots, rx_slots;

	/*
	 * BIT(SNDRV_PCM_STREAM_*): prepared (prepare .. hw_free) and running
	 * (trigger start .. stop). Under hw_lock with frame RUN and irq_mask.
	 */
	spinlock_t hw_lock;
	unsigned int prep_mask, run_mask;
	u32 irq_mask;		/* current MCA_IRQ_MASK value (v1: its causes) */
	u32 prep_frame;		/* frame the prepared block runs */
};

static inline void mca_write(struct s8000_mca *mca, u32 off, u32 val)
{
	writel_relaxed(val, mca->base + off);
}

static inline u32 mca_read(struct s8000_mca *mca, u32 off)
{
	return readl_relaxed(mca->base + off);
}

/*
 * Interrupt causes are handled as version 4 MCA_IRQ_* bits; on version 1
 * they are translated to MCA_V1_IRQ_STATUS bits (low and high water have
 * no version 1 cause) and the mask to the two enables.
 */
static u32 mca_v1_irq_bits(u32 causes)
{
	return (causes & MCA_IRQ_TX_UNDERFLOW ? MCA_V1_IRQ_TX_UNDERFLOW : 0) |
	       (causes & MCA_IRQ_TX_OVERFLOW ? MCA_V1_IRQ_TX_OVERFLOW : 0) |
	       (causes & MCA_IRQ_RX_UNDERFLOW ? MCA_V1_IRQ_RX_UNDERFLOW : 0) |
	       (causes & MCA_IRQ_RX_OVERFLOW ? MCA_V1_IRQ_RX_OVERFLOW : 0) |
	       (causes & MCA_IRQ_FRAME_ERROR ? MCA_V1_IRQ_FRAME_ERROR : 0);
}

static u32 mca_irq_status(struct s8000_mca *mca)
{
	u32 raw;

	if (!mca->regs->v1)
		return mca_read(mca, MCA_IRQ_STATUS);

	raw = mca_read(mca, MCA_V1_IRQ_STATUS);
	return (raw & MCA_V1_IRQ_TX_UNDERFLOW ? MCA_IRQ_TX_UNDERFLOW : 0) |
	       (raw & MCA_V1_IRQ_TX_OVERFLOW ? MCA_IRQ_TX_OVERFLOW : 0) |
	       (raw & MCA_V1_IRQ_RX_UNDERFLOW ? MCA_IRQ_RX_UNDERFLOW : 0) |
	       (raw & MCA_V1_IRQ_RX_OVERFLOW ? MCA_IRQ_RX_OVERFLOW : 0) |
	       (raw & MCA_V1_IRQ_FRAME_ERROR ? MCA_IRQ_FRAME_ERROR : 0);
}

static void mca_irq_ack(struct s8000_mca *mca, u32 causes)
{
	if (mca->regs->v1)
		mca_write(mca, MCA_V1_IRQ_STATUS, mca_v1_irq_bits(causes));
	else
		mca_write(mca, MCA_IRQ_STATUS, causes);
}

static void mca_irq_set_mask(struct s8000_mca *mca, u32 mask)
{
	u32 en = 0;

	if (!mca->regs->v1) {
		mca_write(mca, MCA_IRQ_MASK, mask);
		return;
	}

	if (~mask & MCA_IRQ_V1_FIFO)
		en |= MCA_V1_IRQ_EN_FIFO;
	if (~mask & MCA_IRQ_FRAME_ERROR)
		en |= MCA_V1_IRQ_EN_FRAME;
	mca_write(mca, MCA_V1_IRQ_EN, en);
}

static struct s8000_mca *hw_to_mca(struct clk_hw *hw)
{
	return container_of(hw, struct s8000_mca, mclk_hw);
}

static int mca_mclk_prepare(struct clk_hw *hw)
{
	return pm_runtime_resume_and_get(hw_to_mca(hw)->dev);
}

static void mca_mclk_unprepare(struct clk_hw *hw)
{
	pm_runtime_put(hw_to_mca(hw)->dev);
}

static int mca_mclk_enable(struct clk_hw *hw)
{
	struct s8000_mca *mca = hw_to_mca(hw);

	writel(mca->hw->mclk_ctl, mca->mclk_reg);
	return 0;
}

static void mca_mclk_disable(struct clk_hw *hw)
{
	writel(0, hw_to_mca(hw)->mclk_reg);
}

static unsigned long mca_mclk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	return hw_to_mca(hw)->hw->mclk_rate;
}

static const struct clk_ops mca_mclk_ops = {
	.prepare = mca_mclk_prepare,
	.unprepare = mca_mclk_unprepare,
	.enable = mca_mclk_enable,
	.disable = mca_mclk_disable,
	.recalc_rate = mca_mclk_recalc_rate,
};

/*
 * Route the MCA to its pad with the codec as the clock provider: the pad
 * takes its data from the MCA and drives data only, the MCA takes clocks
 * and data from the pad. For MCA0 on pin 0 this is 0x18, 0x00080808 and
 * 0x10, the state the live iOS ADT reports as "AUD0 - PIN0".
 */
static void mca_route(struct s8000_mca *mca, bool on)
{
	u32 pin = I2S_SW_NODE_DEFAULT, aud = I2S_SW_NODE_DEFAULT, oe = 0;

	if (!mca->sw)
		return;

	if (on) {
		pin = FIELD_PREP(I2S_SW_CLK_SRC, mca->hw->clock_provider ?
				 I2S_SW_CODE_AUD(mca->sw_aud) : I2S_SW_CODE_NONE) |
		      FIELD_PREP(I2S_SW_DATA_SRC0, I2S_SW_CODE_AUD(mca->sw_aud)) |
		      FIELD_PREP(I2S_SW_DATA_SRC1, I2S_SW_CODE_AUD(mca->sw_aud));
		aud = FIELD_PREP(I2S_SW_CLK_SRC, mca->hw->clock_provider ?
				 I2S_SW_CODE_NONE : I2S_SW_CODE_PIN(mca->sw_pin)) |
		      FIELD_PREP(I2S_SW_DATA_SRC0, I2S_SW_CODE_PIN(mca->sw_pin)) |
		      FIELD_PREP(I2S_SW_DATA_SRC1, I2S_SW_CODE_PIN(mca->sw_pin));
		oe = I2S_SW_PIN_OE_DATA;
		if (mca->hw->clock_provider)
			oe |= I2S_SW_PIN_OE_CLK;
	}

	regmap_write(mca->sw, I2S_SW_PIN(mca->sw_pin), pin);
	regmap_write(mca->sw, I2S_SW_AUD(mca->sw_aud), aud);
	regmap_write(mca->sw, I2S_SW_PIN_OE(mca->sw_pin), oe);
}

static int mca_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	unsigned int provider = mca->hw->clock_provider ? SND_SOC_DAIFMT_BP_FP :
						       SND_SOC_DAIFMT_BC_FC;

	/* S8000 consumes codec clocks; T8010 drives the digital speaker amps. */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != provider)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != mca->hw->dai_format)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

/* AppleAE2MCA::_getFIFODepth, TX direction */
static u32 mca_tx_depth(struct s8000_mca *mca, unsigned int rate,
			unsigned int channels)
{
	u32 per_ms = round_up(rate * channels / 2000, 4);
	u32 min = (channels * 2 + 3) & 0x3fc;
	u32 depth;

	if (per_ms < min)
		depth = min;
	else
		depth = min_t(u32, per_ms, mca->fifo_tx - 3);

	depth &= ~3;
	return max_t(u32, depth, 4);
}

static int mca_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	struct snd_dmaengine_dai_dma_data *dma =
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &mca->dma_tx :
								 &mca->dma_rx;
	unsigned int bits = params_width(params);
	unsigned int pcm_width = params_physical_width(params);
	unsigned int slot = mca->slot_width ?: pcm_width;
	unsigned int channels = params_channels(params);
	unsigned int slots = mca->hw->slots, depth;
	unsigned int frame_bits = mca->hw->frame_bits ?: slot * slots;
	unsigned int sync_bits = mca->hw->sync_bits ?: frame_bits / 2;
	u32 width_code;

	if (channels != mca->hw->channels)
		return -EINVAL;
	/* Qualify one clock family first; no unverified v5 capture or TDM. */
	if (mca->hw->clock_provider && params_rate(params) != 48000)
		return -EINVAL;

	switch (slot) {
	case 16:
		width_code = 1;
		break;
	case 32:
		width_code = 4;
		break;
	default:
		return -EINVAL;
	}
	if (frame_bits < slot * slots)
		return -EINVAL;

	/*
	 * _setIISConfig_v4 for the board's iisConfig. The DAI is symmetric,
	 * so both directions compute the same values for both halves; prepare
	 * writes the half (or the block) it owns.
	 */
	mca->sample = (32 - bits) << 8 | (32 - bits);
	/* clocks after the last slot; split in two for a 2-slot I2S frame */
	mca->pad = frame_bits - slot * slots;
	if (!mca->hw->sync_bits && slots == 2)
		mca->pad = (mca->pad / 2) << 16 | mca->pad / 2;
	depth = mca_tx_depth(mca, params_rate(params), channels);
	mca->tx_fifo_cfg = depth | (depth - 1) << 16;
	mca->tx_slots = mca->hw->tx_slots;
	mca->rx_slots = mca->hw->rx_slots;
	/* bits 15:8 = data delay (1); 0x33 base; 0x4 for iisConfig[1] bit 4 clear */
	mca->tx_cfg = mca->hw->tx_cfg;
	mca->frame = FIELD_PREP(MCA_FRAME_HALF, sync_bits - 1) |
		     FIELD_PREP(MCA_FRAME_WIDTH, width_code) |
		     FIELD_PREP(MCA_FRAME_SLOTS, slots - 1) |
		     FIELD_PREP(MCA_FRAME_MCLK_DIV, 0) |
		     mca->hw->frame_flags;

	if (mca->hw->clock_provider) {
		unsigned long bitclk = params_rate(params) * frame_bits *
				       mca->regs->nco_per_sclk;
		int ret;

		ret = clk_set_rate(mca->bitclk, bitclk);
		if (ret)
			return ret;
		if (clk_get_rate(mca->bitclk) != bitclk)
			return -EINVAL;
		mca->frame |= MCA_FRAME_MASTER;
	}

	dma->addr_width = pcm_width == 16 ? DMA_SLAVE_BUSWIDTH_2_BYTES :
				       DMA_SLAVE_BUSWIDTH_4_BYTES;
	/* informational: the SmartIO limit comes from the shim record */
	dma->maxburst = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? depth :
			MCA_RX_FIFO_CFG_IOS & 0xffff;

	return 0;
}

static void mca_write_tx(struct s8000_mca *mca)
{
	const struct s8000_mca_regs *regs = mca->regs;

	if (regs->v1)
		mca_write(mca, MCA_V1_TX_CTL, mca->hw->v1_ctl);
	mca_write(mca, regs->tx_fifo_cfg, mca->tx_fifo_cfg);
	mca_write(mca, regs->tx_slots, mca->tx_slots);
	if (regs->tx_slots_hi)
		mca_write(mca, regs->tx_slots_hi, 0);
	mca_write(mca, regs->tx_cfg, mca->tx_cfg & ~MCA_XX_CFG_EN);
}

static void mca_write_rx(struct s8000_mca *mca)
{
	const struct s8000_mca_regs *regs = mca->regs;

	if (regs->v1)
		mca_write(mca, MCA_V1_RX_CTL, mca->hw->v1_ctl);
	mca_write(mca, regs->rx_fifo_cfg, MCA_RX_FIFO_CFG_IOS);
	mca_write(mca, regs->rx_slots, mca->rx_slots);
	if (regs->rx_slots_hi)
		mca_write(mca, regs->rx_slots_hi, 0);
	mca_write(mca, regs->rx_cfg, mca->hw->rx_cfg & ~MCA_XX_CFG_EN);
}

/*
 * The whole block, as _setIISConfig_v4 (or _v1): only while no direction is
 * prepared
 */
static int mca_init_block(struct s8000_mca *mca)
{
	const struct s8000_mca_regs *regs = mca->regs;
	unsigned long flags;
	u32 val;
	int ret;

	spin_lock_irqsave(&mca->hw_lock, flags);
	mca->irq_mask = MCA_IRQ_ALL;
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	mca_write(mca, regs->reset, MCA_RESET_TX | MCA_RESET_RX);
	/* _setIISConfig_v1 resets and then waits for the flush only */
	if (!regs->v1) {
		ret = readl_relaxed_poll_timeout(mca->base + regs->reset, val,
						 !(val & (MCA_RESET_TX | MCA_RESET_RX)),
						 MCA_POLL_US, MCA_TIMEOUT_US);
		if (ret) {
			dev_err(mca->dev, "reset timed out (%#x)\n", val);
			return ret;
		}
	}

	mca_write(mca, regs->flush, 1);
	ret = readl_relaxed_poll_timeout(mca->base + regs->flush, val,
					 !(val & 1), MCA_POLL_US,
					 MCA_TIMEOUT_US);
	if (ret) {
		dev_err(mca->dev, "flush timed out (%#x)\n", val);
		return ret;
	}

	mca_irq_set_mask(mca, MCA_IRQ_ALL);
	mca_irq_ack(mca, MCA_IRQ_ALL);
	if (regs->v1) {
		mca_write(mca, MCA_V1_TX_CTL, mca->hw->v1_ctl);
		mca_write(mca, MCA_V1_RX_CTL, mca->hw->v1_ctl);
	}
	mca_write(mca, MCA_SAMPLE, mca->sample);
	mca_write(mca, regs->tx_fifo_cfg, mca->tx_fifo_cfg);
	mca_write(mca, regs->rx_fifo_cfg, MCA_RX_FIFO_CFG_IOS);
	mca_write(mca, MCA_PAD, mca->pad);
	mca_write(mca, regs->tx_slots, mca->tx_slots);
	if (regs->tx_slots_hi)
		mca_write(mca, regs->tx_slots_hi, 0);
	mca_write(mca, regs->rx_slots, mca->rx_slots);
	if (regs->rx_slots_hi)
		mca_write(mca, regs->rx_slots_hi, 0);
	mca_write(mca, regs->tx_cfg, mca->tx_cfg & ~MCA_XX_CFG_EN);
	mca_write(mca, regs->rx_cfg, mca->hw->rx_cfg & ~MCA_XX_CFG_EN);
	return 0;
}

/* One direction while the other is prepared or running: leave the frame */
static int mca_init_dir(struct s8000_mca *mca, bool tx)
{
	u32 bit = tx ? MCA_RESET_TX : MCA_RESET_RX, val;
	int ret;

	mca_write(mca, mca->regs->reset, bit);
	ret = readl_relaxed_poll_timeout(mca->base + mca->regs->reset, val,
					 !(val & bit), MCA_POLL_US,
					 MCA_TIMEOUT_US);
	if (ret) {
		dev_err(mca->dev, "%s reset timed out (%#x)\n",
			tx ? "TX" : "RX", val);
		return ret;
	}

	mca_write(mca, MCA_SAMPLE, mca->sample);
	if (tx)
		mca_write_tx(mca);
	else
		mca_write_rx(mca);
	mca_irq_ack(mca, tx ? MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL);
	return 0;
}

/*
 * The AOP is always on and Linux does not use its side of the switch, so
 * the route is (re)written at every start and never taken down.
 */
static void mca_aop_route(struct s8000_mca *mca)
{
	unsigned int i;

	for (i = 0; i < mca->n_aop; i++)
		regmap_write(mca->aop[i].map, mca->aop[i].reg, mca->aop[i].val);
}

static int mca_prepare(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	bool tx = dir == SNDRV_PCM_STREAM_PLAYBACK, first;
	unsigned long flags;
	int ret;

	/* ASoC serializes prepare/hw_free of the card's streams (pcm_mutex) */
	spin_lock_irqsave(&mca->hw_lock, flags);
	first = !(mca->prep_mask & ~BIT(dir));
	/*
	 * Prepared again (XRUN recovery, rewind): the codec may be running
	 * from SCLK, which resetting the block would stop. While the block
	 * still runs the prepared frame (not after a suspend), keep the
	 * clocks; trigger restarts the DMA.
	 */
	if (mca->hw->clocks_at_prepare && (mca->prep_mask & BIT(dir)) &&
	    mca->prep_frame == mca->frame &&
	    mca_read(mca, mca->regs->frame) == (mca->frame | MCA_FRAME_RUN)) {
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	if (first && mca->bitclk && !mca->bitclk_enabled) {
		ret = clk_prepare_enable(mca->bitclk);
		if (ret)
			return ret;
		mca->bitclk_enabled = true;
	}
	if (first)
		mca_aop_route(mca);
	ret = first ? mca_init_block(mca) : mca_init_dir(mca, tx);
	if (ret) {
		if (first && mca->bitclk_enabled) {
			clk_disable_unprepare(mca->bitclk);
			mca->bitclk_enabled = false;
		}
		return ret;
	}

	spin_lock_irqsave(&mca->hw_lock, flags);
	mca->prep_mask |= BIT(dir);
	mca->irq_mask &= ~(tx ? MCA_IRQ_TX_CAUSES | mca->hw->stream_irqs :
			   MCA_IRQ_RX_CAUSES);
	mca_irq_set_mask(mca, mca->irq_mask);
	if (first && tx && mca->hw->clocks_at_prepare)
		mca_write(mca, mca->regs->tx_cfg, mca->tx_cfg | MCA_XX_CFG_EN);
	if (first)
		mca_write(mca, mca->regs->frame, mca->frame |
			  (mca->hw->clocks_at_prepare ? MCA_FRAME_RUN : 0));
	mca->prep_frame = mca->frame;
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	if (first)
		mca_route(mca, true);
	return 0;
}

/* Called with hw_lock held */
static void mca_stop_tx(struct s8000_mca *mca)
{
	u32 val;

	mca_write(mca, mca->regs->tx_cfg, 0);
	mca_write(mca, mca->regs->reset, MCA_RESET_TX);
	if (readl_relaxed_poll_timeout_atomic(mca->base + mca->regs->reset, val,
					      !(val & MCA_RESET_TX), 1, 1000))
		dev_warn(mca->dev, "TX reset timed out\n");
}

static int mca_hw_free(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	unsigned long flags;
	bool last;

	spin_lock_irqsave(&mca->hw_lock, flags);
	if (!(mca->prep_mask & BIT(dir))) {
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		return 0;
	}
	mca->prep_mask &= ~BIT(dir);
	last = !mca->prep_mask;
	if (last) {
		/* back to the idle state runtime resume sets up: all masked */
		mca->irq_mask = MCA_IRQ_ALL;
		if (mca->hw->clocks_at_prepare)
			mca_stop_tx(mca);
		mca_write(mca, mca->regs->frame, 0);
	} else {
		mca->irq_mask |= dir == SNDRV_PCM_STREAM_PLAYBACK ?
				 MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL;
	}
	mca_irq_set_mask(mca, mca->irq_mask);
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	if (last) {
		mca_route(mca, false);
		if (mca->bitclk_enabled) {
			clk_disable_unprepare(mca->bitclk);
			mca->bitclk_enabled = false;
		}
	}
	return 0;
}

static int mca_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	bool tx = dir == SNDRV_PCM_STREAM_PLAYBACK;
	u32 reset = tx ? MCA_RESET_TX : MCA_RESET_RX, val;
	unsigned long flags;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&mca->hw_lock, flags);
		/* the other direction's pending causes are not ours to clear */
		mca_irq_ack(mca, !mca->run_mask ? MCA_IRQ_ALL :
			    tx ? MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL);
		if (tx)
			mca_write(mca, mca->regs->tx_cfg,
				  mca->tx_cfg | MCA_XX_CFG_EN);
		else
			mca_write(mca, mca->regs->rx_cfg,
				  mca->hw->rx_cfg | MCA_XX_CFG_EN);
		mca->run_mask |= BIT(dir);
		mca_write(mca, mca->regs->frame, mca->frame | MCA_FRAME_RUN);
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&mca->hw_lock, flags);
		mca->run_mask &= ~BIT(dir);
		if (tx && mca->hw->clocks_at_prepare) {
			/* SCLK runs until hw_free: the codec may run from it */
			spin_unlock_irqrestore(&mca->hw_lock, flags);
			return 0;
		}
		mca_write(mca, tx ? mca->regs->tx_cfg : mca->regs->rx_cfg, 0);
		/*
		 * MAX98357 asks not to lose LRCLK while BCLK runs and the amp
		 * is on, and the amp stops after this DAI: a provider keeps
		 * the frame running until hw_free.
		 */
		if (!mca->run_mask && !mca->hw->clock_provider)
			mca_write(mca, mca->regs->frame, mca->frame);
		/* TX stops with RESET = 1, RX with RESET = 2 (iOS, v1 and v4) */
		mca_write(mca, mca->regs->reset, reset);
		ret = readl_relaxed_poll_timeout_atomic(mca->base +
							mca->regs->reset,
							val, !(val & reset), 1,
							1000);
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		if (ret)
			dev_warn(mca->dev, "%s reset timed out\n",
				 tx ? "TX" : "RX");
		return 0;
	}

	return -EINVAL;
}

static int mca_dai_probe(struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &mca->dma_tx,
				  mca->capture ? &mca->dma_rx : NULL);
	return 0;
}

static const struct snd_soc_dai_ops mca_dai_ops = {
	.probe = mca_dai_probe,
	.set_fmt = mca_set_fmt,
	.hw_params = mca_hw_params,
	.prepare = mca_prepare,
	.hw_free = mca_hw_free,
	.trigger = mca_trigger,
};

/* Template; probe drops .capture without a SmartIO "rx" channel. */
static const struct snd_soc_dai_driver mca_dai = {
	.name = "s8000-mca",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MCA_RATES,
		.formats = MCA_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MCA_RATES,
		.formats = MCA_FORMATS,
	},
	.ops = &mca_dai_ops,
	/* one frame (clocks, slot width) for both directions */
	.symmetric_rate = 1,
	.symmetric_sample_bits = 1,
};

static const struct snd_soc_component_driver mca_component = {
	.name = "s8000-mca",
};

/*
 * SmartIO queues one command per period (at most 16, it has 31 tags per
 * endpoint) and reports the position per completed period. The generic
 * dmaengine PCM would flag that as BATCH, but skips that when the driver
 * supplies its own hardware limits, so set it here: PipeWire and
 * PulseAudio need it to keep a period of headroom.
 */
static const struct snd_pcm_hardware mca_dma_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
	.formats = MCA_FORMATS,
	.rates = MCA_RATES,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 512,
	.period_bytes_max = 32 * 1024,
	.periods_min = 2,
	.periods_max = 16,
};

static int mca_prepare_slave_config(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct dma_slave_config *sc)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct s8000_mca *mca =
		snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));
	struct apple_sio_dma_config *cfg = &mca->sio_cfg[substream->stream];
	const struct s8000_mca_hw *hw = mca->hw;
	unsigned int bytes = params_physical_width(params) / 8;
	unsigned int per_ms;
	int ret;

	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, sc);
	if (ret)
		return ret;

	/*
	 * The shim record as AppleAE2MCA sets it up: the transfer width from
	 * the dma-channels entry, and as the FIFO depth for either direction
	 * one millisecond of samples, rounded to 4. The DMA driver applies
	 * SmartIO's validFIFODepth to it: TX at least trigger + 4, RX at most
	 * the trigger.
	 */
	per_ms = round_up(params_rate(params) * params_channels(params) / 1000,
			  4);
	*cfg = (struct apple_sio_dma_config) {
		.xfer = bytes == 2 ? hw->sio_xfer16 : hw->sio_xfer32,
		.timeout = hw->sio_timeout,
		.fifo = hw->sio_fifo,
		.trigger = hw->sio_trigger,
		.limit = bytes * per_ms,
	};
	sc->peripheral_config = cfg;
	sc->peripheral_size = sizeof(*cfg);
	return 0;
}

/* Only for S16_LE PCMs: the variants with an attenuation allow nothing else. */
static int mca_attenuate(struct snd_pcm_substream *substream, int channel,
			 unsigned long hwoff, unsigned long bytes)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct s8000_mca *mca =
		snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));
	s16 *sample = (s16 *)(substream->runtime->dma_area + hwoff);
	unsigned long i;

	for (i = 0; i < bytes / sizeof(*sample); i++)
		sample[i] /= (int)mca->hw->attenuation;
	return 0;
}

static const struct snd_dmaengine_pcm_config mca_dma_config = {
	.prepare_slave_config = mca_prepare_slave_config,
	.prealloc_buffer_size = 128 * 1024,
};

static irqreturn_t mca_irq(int irq, void *data)
{
	struct s8000_mca *mca = data;
	u32 raw, status;

	spin_lock(&mca->hw_lock);
	raw = mca_irq_status(mca);
	/*
	 * Only bits 12:0 are interrupt causes (iOS tests status & 0x1fff and
	 * reports anything else as spurious). Bit 31 also reads as set while
	 * a cause is pending, so it must not count as handled, or a cause
	 * open in the hardware mask but not in irq_mask would be "handled"
	 * forever.
	 */
	status = raw & MCA_IRQ_ALL & ~mca->irq_mask;
	if (!status) {
		/* resync the hardware mask, so a stale open mask cannot storm */
		mca_irq_set_mask(mca, mca->irq_mask);
		mca_irq_ack(mca, raw & MCA_IRQ_ALL);
		spin_unlock(&mca->hw_lock);
		return IRQ_NONE;
	}

	mca_irq_ack(mca, status);
	if (status & MCA_IRQ_ERRORS) {
		/*
		 * Error causes re-latch every frame while the condition lasts
		 * (e.g. a TX underflow): mask them until the direction's next
		 * prepare instead of taking an interrupt per frame.
		 */
		mca->irq_mask |= status & MCA_IRQ_ERRORS;
		/* v1 has one enable for both directions' FIFO errors */
		if (mca->regs->v1 && (status & MCA_IRQ_V1_FIFO))
			mca->irq_mask |= MCA_IRQ_V1_FIFO;
		mca_irq_set_mask(mca, mca->irq_mask);
		dev_dbg_ratelimited(mca->dev, "irq status %#x, masked\n",
				    status);
	}
	spin_unlock(&mca->hw_lock);

	return IRQ_HANDLED;
}

/*
 * The IRQ is only enabled while the block is powered: the handler reads
 * MCA registers, and an MMIO access to a power-gated Apple block can hang
 * the CPU.
 */
static int mca_runtime_suspend(struct device *dev)
{
	struct s8000_mca *mca = dev_get_drvdata(dev);

	if (!mca->hw->no_irq)
		disable_irq(mca->irq);
	mca_irq_set_mask(mca, MCA_IRQ_ALL);
	return 0;
}

static int mca_runtime_resume(struct device *dev)
{
	struct s8000_mca *mca = dev_get_drvdata(dev);

	/*
	 * The power domain was just switched on, so the MCA may be at its
	 * reset state, with causes like no-BCLK open in the hardware mask.
	 * Mask and clear everything before the IRQ is enabled; prepare
	 * unmasks what a stream needs.
	 */
	mca->irq_mask = MCA_IRQ_ALL;
	mca_irq_set_mask(mca, MCA_IRQ_ALL);
	mca_irq_ack(mca, MCA_IRQ_ALL);
	if (!mca->hw->no_irq)
		enable_irq(mca->irq);
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mca_pm_ops, mca_runtime_suspend,
				 mca_runtime_resume, NULL);

static int s8000_mca_get_aop_route(struct s8000_mca *mca)
{
	struct device_node *np = mca->dev->of_node;
	struct of_phandle_args args;
	struct regmap *map;
	unsigned int i;

	for (i = 0; ; i++) {
		if (of_parse_phandle_with_fixed_args(np, "apple,aop-i2s-route",
						     2, i, &args))
			break;
		if (i >= S8000_MCA_AOP_WRITES) {
			of_node_put(args.np);
			return dev_err_probe(mca->dev, -EINVAL,
					     "too many apple,aop-i2s-route entries\n");
		}
		map = syscon_node_to_regmap(args.np);
		of_node_put(args.np);
		if (IS_ERR(map))
			return dev_err_probe(mca->dev, PTR_ERR(map),
					     "failed to get an AOP i2s-switch register\n");
		mca->aop[i].map = map;
		mca->aop[i].reg = args.args[0];
		mca->aop[i].val = args.args[1];
	}
	mca->n_aop = i;
	return 0;
}

static int s8000_mca_get_route(struct s8000_mca *mca)
{
	struct device_node *np = mca->dev->of_node;
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, "apple,i2s-switch", 2, 0,
					       &args);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return dev_err_probe(mca->dev, ret,
				     "invalid apple,i2s-switch\n");

	mca->sw = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(mca->sw))
		return dev_err_probe(mca->dev, PTR_ERR(mca->sw),
				     "failed to get the i2s-switch\n");

	mca->sw_aud = args.args[0];
	mca->sw_pin = args.args[1];
	if (mca->sw_aud >= I2S_SW_NODES || mca->sw_pin >= I2S_SW_NODES)
		return dev_err_probe(mca->dev, -EINVAL,
				     "invalid i2s-switch route %u/%u\n",
				     mca->sw_aud, mca->sw_pin);
	return 0;
}

/*
 * The T8010 headphone link: MCA4 to the CS42L83 jack codec, the ADT's
 * /arm-io/mca4/audio-codec (iPadOS 17.7.x, identical on J71b, J72b, J171 and
 * J172): provider (its i2s-switch route has the flags of the speaker MCA2,
 * not the consumer MCA0), flags 0x13, two slots carrying data, 32-bit samples,
 * and SmartIO endpoint 0x30 with a 0x180-byte FIFO and a 0xc0 trigger.
 *
 * Linux's CS42L42-family driver only takes standard I2S as a clock consumer
 * and derives its internal MCLK from SCLK with its PLL, so this variant runs
 * a plain 64 fs I2S frame (two 32-bit slots, LRCLK at half the frame, one bit
 * of data delay) with the frame bits the S8000 I2S link uses. iOS's frame
 * length for this link (byte 0xc of the record) has not been decoded and may
 * differ; the codec does not need it to.
 */
static const struct s8000_mca_hw t8010_i2s_mca_hw = {
	.version = 5,
	.regs = &mca_v4_regs,
	.clock_provider = true,
	.clocks_at_prepare = true,
	.dai_format = SND_SOC_DAIFMT_I2S,
	.channels = 2,
	.slots = 2,
	.tx_slots = 0x3,
	.tx_cfg = 0x133,
	.rx_cfg = 0x137,
	.rx_slots = 0x3,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.frame_flags = MCA_FRAME_DUTY | MCA_FRAME_FMT,
	.attenuation = 1,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0x180,
	.sio_trigger = 0xc0,
};

static int s8000_mca_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {};
	const char *clk_name = "mca-mclk";
	struct s8000_mca *mca;
	u32 ver, fifo;
	int ret;

	mca = devm_kzalloc(dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;
	mca->dev = dev;
	mca->hw = device_get_match_data(dev);
	if (!mca->hw)
		return -ENODEV;
	mca->regs = mca->hw->regs;
	/* J120 iisConfig spans 256 clocks over eight slots with S16 samples. */
	if (of_device_is_compatible(dev->of_node, "apple,t8011-mca"))
		mca->slot_width = 32;
	spin_lock_init(&mca->hw_lock);
	mca->irq_mask = MCA_IRQ_ALL;
	platform_set_drvdata(pdev, mca);

	mca->base = devm_platform_ioremap_resource_byname(pdev, "mca");
	if (IS_ERR(mca->base))
		return PTR_ERR(mca->base);
	mca->mclk_reg = devm_platform_ioremap_resource_byname(pdev, "mclk");
	if (IS_ERR(mca->mclk_reg))
		return PTR_ERR(mca->mclk_reg);

	if (mca->hw->clock_provider) {
		mca->bitclk = devm_clk_get(dev, "bitclk");
		if (IS_ERR(mca->bitclk))
			return dev_err_probe(dev, PTR_ERR(mca->bitclk),
					     "failed to get AP NCO bitclock\n");
	}

	ret = s8000_mca_get_route(mca);
	if (ret)
		return ret;

	ret = s8000_mca_get_aop_route(mca);
	if (ret)
		return ret;

	if (!mca->hw->no_irq) {
		mca->irq = platform_get_irq(pdev, 0);
		if (mca->irq < 0)
			return mca->irq;
		/* enabled by runtime resume, once the hardware mask is set */
		ret = devm_request_irq(dev, mca->irq, mca_irq, IRQF_NO_AUTOEN,
				       dev_name(dev), mca);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request IRQ\n");
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power up\n");
	ver = mca_read(mca, MCA_VERSION);
	fifo = mca_read(mca, MCA_FIFO_SIZE);
	pm_runtime_put(dev);

	/* Version 1 keeps the version in the upper half (0x00010002 on a J81) */
	if (mca->hw->version == 1)
		ver >>= 16;
	mca->fifo_tx = FIELD_GET(MCA_FIFO_SIZE_TX, fifo);
	if (ver != mca->hw->version || mca->fifo_tx < 8)
		dev_warn(dev, "unexpected MCA version %u, TX FIFO %u\n", ver,
			 mca->fifo_tx);
	if (mca->fifo_tx < 8)
		mca->fifo_tx = 0x40;

	/* The generic PCM fails the card if a DAI stream has no channel. */
	mca->capture = of_property_match_string(dev->of_node, "dma-names",
						"rx") >= 0;

	/* MCLK control word -> codec MCLK */
	of_property_read_string(dev->of_node, "clock-output-names", &clk_name);
	init.name = clk_name;
	init.ops = &mca_mclk_ops;
	mca->mclk_hw.init = &init;
	ret = devm_clk_hw_register(dev, &mca->mclk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register MCLK\n");
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &mca->mclk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add MCLK provider\n");

	mca->dai_drv = mca_dai;
	mca->dai_drv.playback.channels_min = mca->hw->channels;
	mca->dai_drv.playback.channels_max = mca->hw->channels;
	mca->dai_drv.playback.formats = mca->hw->formats;
	if (mca->hw->clock_provider)
		mca->dai_drv.playback.rates = SNDRV_PCM_RATE_48000;
	if (!mca->capture)
		memset(&mca->dai_drv.capture, 0, sizeof(mca->dai_drv.capture));

	ret = devm_snd_soc_register_component(dev, &mca_component,
					      &mca->dai_drv, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	mca->pcm_hw = mca_dma_hw;
	mca->pcm_hw.channels_min = mca->hw->channels;
	mca->pcm_hw.channels_max = mca->hw->channels;
	mca->pcm_hw.formats = mca->hw->formats;
	mca->pcm_config = mca_dma_config;
	mca->pcm_config.pcm_hardware = &mca->pcm_hw;
	if (mca->hw->attenuation > 1) {
		mca->pcm_hw.info &= ~(SNDRV_PCM_INFO_MMAP |
				      SNDRV_PCM_INFO_MMAP_VALID);
		mca->pcm_config.process = mca_attenuate;
	}

	ret = devm_snd_dmaengine_pcm_register(dev, &mca->pcm_config, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register PCM\n");

	return 0;
}

/* MCA0 on S8000/S8003, from iOS 16.7.10 AppleAE2MCA and the J71t ADT */
static const struct s8000_mca_hw s8000_mca_hw = {
	.version = 4,
	.regs = &mca_v4_regs,
	.dai_format = SND_SOC_DAIFMT_I2S,
	.channels = 2,
	.slots = 2,
	.tx_slots = 0x3,
	.tx_cfg = MCA_TX_CFG_I2S,
	.rx_cfg = MCA_RX_CFG_I2S,
	.rx_slots = 0x3,
	.formats = MCA_FORMATS,
	.frame_flags = MCA_FRAME_DUTY | MCA_FRAME_FMT,
	.mclk_ctl = 0x58,
	.mclk_rate = 12000000,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0x300,
	.sio_trigger = 0x180,
};

/*
 * N66 (iPhone 6s Plus) MCA2 TX to the CS35L19 speaker amplifier, from the
 * ADT's /arm-io/mca2/audio-speaker iisConfig (the phone's live ADT, iOS 15):
 *
 *   10 12 00 00 08 00 01 00  00 1b b7 00 00 00 10 00
 *   30 00 01 00 02 00 00 00  0c 00 00 00 01 02 10 10  00 00 00 00
 *
 * 8 slots of 16 bits (a 128-bit frame) with a 16-bit sync pulse and one bit
 * of data delay, format flags 0x12, the speaker in slot 1, a 12 MHz MCLK
 * output. iOS runs this MCA as a clock consumer of the CS42L71 codec's ASP,
 * which Linux does not drive yet, so the MCA provides the clocks itself from
 * NCO 2 (6.144 MHz at 48 kHz), with the polarity bit _setIISConfig_v4 sets
 * for flags 0x12 in that role (as on J81). The SmartIO shim words are the
 * ADT's dma-channels for endpoint 0x2c (FIFO 0x60, trigger 0x30).
 *
 * The output is capped at -12 dB (attenuation 4), unconditionally: there is
 * no speaker protection yet. iOS reads the amplifier's voltage and current
 * sense back and keeps the speaker in its limits; Linux does not. The cap
 * lives in this table only, so no ALSA control or device tree property can
 * lift it, and with mmap off every sample passes through mca_attenuate().
 */
static const struct s8000_mca_hw s8000_tdm_mca_hw = {
	.version = 4,
	.regs = &mca_v4_regs,
	.clock_provider = true,
	.dai_format = SND_SOC_DAIFMT_DSP_A,
	.channels = 1,
	.slots = 8,
	.tx_slots = 0x2,
	.tx_cfg = 0x133,
	.rx_cfg = 0x137,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.frame_flags = MCA_FRAME_POL,
	.sync_bits = 16,
	.stream_irqs = MCA_IRQ_FRAME_ERROR,
	.attenuation = 4,
	.mclk_ctl = 0x58,
	.mclk_rate = 12000000,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0x60,
	.sio_trigger = 0x30,
};

/*
 * J71b MCA2 TX: iPadOS 17.7.11 audio-speaker iisConfig and dma-channels, not
 * the A9 MCA0 values. Slots 0-3 carry the four amplifiers; which speaker sits
 * in which slot is not in the ADT. FRAME has no bits 6:2 set and the IRQ mask
 * opens the frame error, as iOS does for a fixed frame length.
 *
 * The attenuation bounds full scale to -12 dB: iOS protects the speakers from
 * its side of the amplifiers (speaker protection), which Linux does not have.
 */
static const struct s8000_mca_hw t8010_mca_hw = {
	.version = 5,
	.regs = &mca_v4_regs,
	.clock_provider = true,
	.dai_format = SND_SOC_DAIFMT_DSP_A,
	.channels = 4,
	.slots = 8,
	.tx_slots = 0x0f,
	.tx_cfg = 0x133,
	.rx_cfg = 0x137,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.sync_bits = 1,
	.stream_irqs = MCA_IRQ_FRAME_ERROR,
	.attenuation = 4,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0x180,
	.sio_trigger = 0xcc,
};

/*
 * J81 (iPad Air 2) MCA3 TX, from the ADT's /arm-io/mca3/audio-speaker0
 * iisConfig (iOS 15.8 19H422):
 *
 *   01 12 00 00 08 00 00 00  00 00 00 00 00 01 10 00
 *   00 00 00 00 03 00 00 00  ff 00 00 00 02 08 10 10  00 00 00 00
 *
 * provider, format flags 0x12, 8 slots in a fixed 256-bit frame with a
 * 16-bit sync pulse, two TX channels in slots 0-1 (the two MAX98721
 * amplifiers), eight RX channels (not used here), 16-bit samples. Under
 * _setIISConfig_v1 that is TX_CFG 0x109 (data delay 1), RX_CFG 0x00b,
 * FRAME 0x0f107005 + RUN, PAD 128, TX_SLOTS 0x3, and a bitclock NCO
 * (function-mclk_frequency, NCO 3) at twice SCLK: 24.576 MHz at 48 kHz.
 * The MCLK word stays 0: iisConfig[0] bit 4 (MCLK output) is clear. The
 * SmartIO shim words are the ADT's dma-channels (FIFO 0xcc, trigger 0xac).
 *
 * As on J71b, the output is bounded to -12 dB until the amplifiers have
 * a driver and speaker protection.
 */
static const struct s8000_mca_hw t7001_mca_hw = {
	.version = 1,
	.regs = &mca_v1_regs,
	.clock_provider = true,
	.dai_format = SND_SOC_DAIFMT_DSP_A,
	.v1_ctl = MCA_V1_XX_CTL_IOS,
	.channels = 2,
	.slots = 8,
	.frame_bits = 256,
	.tx_slots = 0x3,
	.tx_cfg = 0x109,
	.rx_cfg = 0x00b,
	.rx_slots = 0xff,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.frame_flags = MCA_FRAME_POL,
	.sync_bits = 16,
	.stream_irqs = MCA_IRQ_FRAME_ERROR,
	.attenuation = 4,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0xcc,
	.sio_trigger = 0xac,
};

/*
 * S5L8960X (A7) MCA0 to the CS42L81 on the iPad Air (J72): the ADT's
 * /arm-io/mca0/audio-codec iisConfig (iOS 12.5.7 16H81)
 *
 *   10 00 00 00 02 00 01 00 ...
 *
 * is the J71t link: consumer, I2S, two 16-bit slots, data delay 1. Under
 * version 1 that is TX_CFG 0x10b, RX_CFG 0x109, FRAME 0x0f101048 + RUN,
 * PAD 0 and slots 0-1, with 0xf in TX/RX_CTL; the MCLK word 0x58 gives the
 * codec 12 MHz. Measured against iOS on a J72 at 48 kHz, 16 bit. The
 * SmartIO shim words are the ADT's dma-channels (FIFO 0xcc, trigger 0xac).
 * The codec has its own volume, so there is no attenuation.
 */
static const struct s8000_mca_hw s5l8960x_mca_hw = {
	.version = 1,
	.regs = &mca_v1_regs,
	.dai_format = SND_SOC_DAIFMT_I2S,
	.v1_ctl = MCA_V1_XX_CTL_A7,
	.channels = 2,
	.slots = 2,
	.tx_slots = 0x3,
	.tx_cfg = 0x10b,
	.rx_cfg = 0x109,
	.rx_slots = 0x3,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.frame_flags = MCA_FRAME_DUTY | MCA_FRAME_FMT,
	.mclk_ctl = 0x58,
	.mclk_rate = 12000000,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0xcc,
	.sio_trigger = 0xac,
};

/*
 * S5L8960X (A7) MCA1 TX, the speaker link of the iPad mini 2 (J85), from
 * the ADT's /arm-io/mca1/audio-speaker0 iisConfig (iOS 12.5.8 16H88):
 *
 *   10 12 00 00 08 00 01 00  00 1b b7 00 00 00 10 00
 *   30 00 01 00 03 00 00 00  ff 00 00 00 02 08 10 10  00 00 00 00
 *
 * The MCA is a consumer (iisConfig[0] bit 0 clear): the primary CS35L19
 * amplifier drives the frame, from the 12 MHz MCLK this MCA outputs
 * (iisConfig[0] bit 4, bytes 8-11 = 12000000; control word 0x58 as on
 * S8000 and J72). Format flags 0x12 as on J81 (DSP A, one bit of data
 * delay: TX_CFG 0x109, and no clock edge bit), 8 slots of 16 bits with no
 * fixed frame length (bytes 0xc-0xd zero) and a 16-bit sync pulse, TX in
 * slots 0-1 (one per amplifier). No POL bit: format flag bit 0 equals the
 * (consumer) role. The SmartIO shim words are the ADT's dma-channels (FIFO
 * 0xcc, trigger 0xac), the same as on J81; TX/RX_CTL 0xf as on MCA0.
 *
 * As on J71b and J81, full scale is bounded to -12 dB here, where no client
 * can bypass it: Linux has no speaker protection.
 */
static const struct s8000_mca_hw s5l8960x_tdm_mca_hw = {
	.version = 1,
	.no_irq = true,
	.regs = &mca_v1_regs,
	.dai_format = SND_SOC_DAIFMT_DSP_A,
	.v1_ctl = MCA_V1_XX_CTL_A7,
	.channels = 2,
	.slots = 8,
	.tx_slots = 0x3,
	.tx_cfg = 0x109,
	.rx_cfg = 0x00b,
	.rx_slots = 0xff,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.sync_bits = 16,
	.attenuation = 4,
	.mclk_ctl = 0x58,
	.mclk_rate = 12000000,
	.sio_xfer16 = 0x21,
	.sio_xfer32 = 0x22,
	.sio_timeout = 0,
	.sio_fifo = 0xcc,
	.sio_trigger = 0xac,
};

static const struct of_device_id s8000_mca_of_match[] = {
	{ .compatible = "apple,s5l8960x-mca", .data = &s5l8960x_mca_hw },
	{ .compatible = "apple,s5l8960x-mca-tdm", .data = &s5l8960x_tdm_mca_hw },
	{ .compatible = "apple,s8000-mca", .data = &s8000_mca_hw },
	{ .compatible = "apple,s8000-mca-tdm", .data = &s8000_tdm_mca_hw },
	{ .compatible = "apple,t7001-mca", .data = &t7001_mca_hw },
	{ .compatible = "apple,t8010-mca", .data = &t8010_mca_hw },
	{ .compatible = "apple,t8010-mca-i2s", .data = &t8010_i2s_mca_hw },
	{ .compatible = "apple,t8011-mca", .data = &t8010_mca_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, s8000_mca_of_match);

static struct platform_driver s8000_mca_driver = {
	.driver = {
		.name = "s8000-mca",
		.of_match_table = s8000_mca_of_match,
		.pm = pm_ptr(&mca_pm_ops),
	},
	.probe = s8000_mca_probe,
};
module_platform_driver(s8000_mca_driver);

MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_DESCRIPTION("Apple S8000 (A9) MCA I2S driver");
MODULE_LICENSE("Dual MIT/GPL");
