// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8010 (A10) PDM microphone controller, as an ASoC CPU DAI
 *
 * The built-in microphones of the iPad 6 and iPad 7 are PDM microphones.
 * Their bit streams go to the application processor's PDM controller, which
 * has one HiFi decimator per microphone; the decimators filter the streams
 * down to PCM and the controller hands the samples to SmartIO DMA. The PDM
 * clock comes from the Always-On Processor's audio service.
 *
 * The register sequences follow iPadOS 17.7.11's AppleT8010PDMAudio
 * (ApplePDMControllerT8010 and AppleHPDecimator); the decimator filter for
 * 48 kHz is the one in the "dmic" node of the J71b, J72b and J171 Apple
 * device trees (identical on all three).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/sio.h>
#include <linux/spinlock.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* Controller */
#define PDM_FIFO_INFO		0x04
#define PDM_FIFO_INFO_DEPTH	GENMASK(25, 16)
#define PDM_SAMPLE		0x08
#define PDM_SAMPLE_SHIFT	GENMASK(12, 8)
#define PDM_RESET		0x0c
#define PDM_RESET_FIFO		BIT(1)
#define PDM_CTRL		0x10
#define PDM_CTRL_RUN		BIT(0)
#define PDM_CTRL_STOP		BIT(1)
#define PDM_CTRL_BASE		BIT(3)
#define PDM_CTRL_MIC(n)		BIT(4 + (n))
#define PDM_CTRL_24BIT		BIT(31)
#define PDM_STATUS		0x18
#define PDM_STATUS_BUSY		BIT(28)
#define PDM_IRQ_STATUS		0x1c
#define PDM_IRQ_UNDERFLOW	BIT(3)
#define PDM_IRQ_OVERFLOW	BIT(4)
#define PDM_IRQ_MASK		0x20
#define PDM_IRQ_MASK_RUN	0x20
#define PDM_IRQ_MASK_STOP	0x38
#define PDM_FLUSH		0x24
#define PDM_FLUSH_GO		BIT(0)
#define PDM_FIFO_CFG		0x80
#define PDM_FIFO_CFG_IOS	0x22
#define PDM_WATERMARK		0x84
#define PDM_WATERMARK_LEVEL	GENMASK(24, 16)
#define PDM_WATERMARK_BASE	0x4
#define PDM_START		0x110

/* HiFi decimator */
#define DEC_LENGTHS		0x00
#define DEC_LOAD		0x04
#define DEC_CONFIG		0x08
#define DEC_GATE		0x10
#define DEC_GATE_ALL		0x1111
#define DEC_RATIOS		0x14
#define DEC_RATIOS_PRODUCT	GENMASK(7, 0)
#define DEC_RATIOS_R0		GENMASK(15, 10)
#define DEC_RATIOS_R1		GENMASK(21, 16)
#define DEC_RATIOS_R2		GENMASK(27, 22)
#define DEC_16BIT		0x30
#define DEC_TIMING0		0x3c
#define DEC_TIMING1		0x40
#define DEC_COEFF		0x1000

#define PDM_MAX_MICS		2
#define PDM_TIMEOUT_US		1000
/* "mic-startup-delay" of the hifi-dec nodes */
#define PDM_MIC_STARTUP_MS	50
/* without the "busy-dma" property iOS uses this watermark */
#define PDM_WATERMARK_WORDS	4

/*
 * The sample shift: 16 (iOS's value for 16-bit samples) passes the
 * decimators' output as it is, each step below it adds 6 dB. The capture
 * volume runs from 0 to +42 dB; +18 dB puts speech near -14 dBFS and keeps
 * headroom for the microphones' DC offset, which the gain scales too.
 */
#define PDM_SHIFT_UNITY		16
#define PDM_GAIN_MAX		7
#define PDM_GAIN_DEFAULT	3

#define PDM_RATES		SNDRV_PCM_RATE_48000
#define PDM_FORMATS		SNDRV_PCM_FMTBIT_S16_LE

struct t8010_pdm_filter {
	unsigned int rate;
	unsigned int pdm_rate;		/* "pdmfreq-<rate>" */
	u8 ratios[3];			/* "ratios-<rate>" */
	u8 lengths[3];			/* "lengths-<rate>" */
	const s64 *coeffs;		/* "coeff-<rate>", lengths[0..2] summed */
};

static const s64 t8010_pdm_coeffs_48000[] = {
	-14612, -202381, -1130814, -4069816,
	-10880924, -22898738, -38556964, -50626684,
	-45404441, -6814836, 73217110, 181006512,
	271997539, 276738469, 128362809, -193102755,
	-618087258, -973086517, -1015691696, -519574836,
	617919115, 2293215803, 4185384485, 5836128129,
	6798862616, 6798862616, 5836128129, 4185384485,
	2293215803, 617919115, -519574836, -1015691696,
	-973086517, -618087258, -193102755, 128362809,
	276738469, 271997539, 181006512, 73217110,
	-6814836, -45404441, -50626684, -38556964,
	-22898738, -10880924, -4069816, -1130814,
	-202381, -14612, -60318, -301869,
	-900716, -1949222, -3188570, -3686307,
	-1680839, 5090919, 18339672, 37516505,
	58044280, 70582391, 62560464, 22670899,
	-52295366, -150599862, -242366197, -283739834,
	-230136791, -56540454, 221618660, 536549872,
	774412313, 804362052, 526400105, -75347863,
	-893748135, -1693841487, -2153409686, -1945357667,
	-841724919, 1192792220, 3951638589, 7004674201,
	9786552588, 11734175981, 12433984993, 11734175981,
	9786552588, 7004674201, 3951638589, 1192792220,
	-841724919, -1945357667, -2153409686, -1693841487,
	-893748135, -75347863, 526400105, 804362052,
	774412313, 536549872, 221618660, -56540454,
	-230136791, -283739834, -242366197, -150599862,
	-52295366, 22670899, 62560464, 70582391,
	58044280, 37516505, 18339672, 5090919,
	-1680839, -3686307, -3188570, -1949222,
	-900716, -301869, -60318, 37697,
	741944, 3213342, 7258688, 9588835,
	6155121, -1773829, -6271437, -1789514,
	5594133, 4572191, -4416527, -7400071,
	2107081, 9968333, 1702385, -11546629,
	-6966254, 11280868, 13195920, -8402236,
	-19474171, 2419790, 24531075, 6698607,
	-26891445, -18384428, 25094070, 31397616,
	-17965298, -43855101, 4911784, 53380283,
	13811163, -57375700, -36907409, 53401117,
	61976071, -39621646, -85611820, 15263275,
	103695638, 18990746, -111866005, -60744378,
	106139828, 105841202, -83615383, -148577694,
	43165531, 182200816, 13984294, -199670092,
	-83906552, 194624626, 159948681, -162453562,
	-233102365, 101330033, 292790106, -13053006,
	-328034186, -96460775, 328911190, 217173335,
	-288141778, -335497675, 202611798, 435475429,
	-74603462, -500502112, -87489426, 515460375,
	269154278, -469042850, -450584290, 355976260,
	608374336, -178831151, -718008086, -50904472,
	756943020, 312760787, -707983722, -578418248,
	562541307, 814099991, -323324727, -984131458,
	6003163, 1055403175, 360574827, -1002315448,
	-735898204, 811632870, 1071179007, -486585933,
	-1314522168, 49526832, 1417542005, 457519342,
	-1342845489, -974922894, 1071553321, 1430440425,
	-609843631, -1747122647, -6615095, 1853751314,
	711496481, -1696990497, -1410949995, 1253973078,
	1991805428, -543589524, -2335172723, -365651957,
	2335107432, 1353619373, -1921138342, -2254931419,
	1082340339, 2876024465, 110735503, -3024548267,
	-1491523911, 2551297279, 2797232986, -1402398482,
	-3691350716, -324606025, 3817915712, 2333362698,
	-2895125292, -4111464719, 849816031, 4976276272,
	2020653272, -4228475582, -4903902678, 1461900929,
	6448221520, 2948308796, -5054614829, -7270304651,
	-207946250, 8164556802, 7769621721, -1900028559,
	-11231935075, -10661277904, 289059290, 13767811519,
	21429997078, 20682388156, 14613693055, 7871289330,
	3219368673, 961029538, 189896415, 18987337,
};

static const struct t8010_pdm_filter t8010_pdm_filters[] = {
	{
		.rate = 48000,
		.pdm_rate = 2400000,
		.ratios = { 5, 5, 2 },
		.lengths = { 50, 73, 161 },
		.coeffs = t8010_pdm_coeffs_48000,
	},
};

/* Per-decimator timing words of AppleHPDecimator, indexed by decimator */
static const u32 t8010_pdm_dec_timing0[] = {
	0x80000057, 0x80000062, 0x8000007a, 0x80000092,
	0x800000b9, 0x800000ba, 0x80000106, 0x80000114,
};

static const u32 t8010_pdm_dec_timing1[] = {
	0x08000004, 0x08000029, 0x0800003b, 0x08000070,
	0x080000b3, 0x080000b9, 0x080000ef, 0x08000100,
};

/* The "dma-channels" record of the pdm node */
#define PDM_SIO_XFER16		0x21
#define PDM_SIO_FIFO		0x398
#define PDM_SIO_TRIGGER		0x190

struct t8010_pdm_dma_hw {
	u32 fifo;
	u32 trigger;
};

struct t8010_pdm {
	struct device *dev;
	const struct t8010_pdm_dma_hw *dma_hw;
	void __iomem *base;
	void __iomem *dec[PDM_MAX_MICS];
	unsigned int num_mics;
	struct clk *clk;
	spinlock_t lock;	/* controller registers */

	const struct t8010_pdm_filter *filter;
	unsigned int channels;
	unsigned int gain;	/* 6 dB steps */
	bool clk_on;

	struct snd_soc_dai_driver dai_drv;
	struct snd_dmaengine_dai_dma_data dma_rx;
	struct apple_sio_dma_config sio_cfg;
	struct snd_dmaengine_pcm_config pcm_config;
};

static int t8010_pdm_wait_clear(struct t8010_pdm *pdm, u32 reg, u32 bits)
{
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout_atomic(pdm->base + reg, val,
						!(val & bits), 10,
						PDM_TIMEOUT_US);
	if (ret)
		dev_warn(pdm->dev, "register %#x bits %#x stuck\n", reg, bits);
	return ret;
}

/* AppleHPDecimator: the filter for the stream's rate and width */
static void t8010_pdm_dec_configure(struct t8010_pdm *pdm, unsigned int idx)
{
	const struct t8010_pdm_filter *f = pdm->filter;
	void __iomem *dec = pdm->dec[idx];
	unsigned int i, n = f->lengths[0] + f->lengths[1] + f->lengths[2];

	writel_relaxed(1, dec + DEC_16BIT);
	writel_relaxed(idx, dec + DEC_CONFIG);
	writel_relaxed(FIELD_PREP(DEC_RATIOS_PRODUCT,
				  f->ratios[0] * f->ratios[1] * f->ratios[2] - 1) |
		       FIELD_PREP(DEC_RATIOS_R0, f->ratios[0]) |
		       FIELD_PREP(DEC_RATIOS_R1, f->ratios[1]) |
		       FIELD_PREP(DEC_RATIOS_R2, f->ratios[2]),
		       dec + DEC_RATIOS);
	writel_relaxed(f->lengths[2] << 16 | f->lengths[1] << 8 | f->lengths[0],
		       dec + DEC_LENGTHS);
	for (i = 0; i < n; i++) {
		writel_relaxed(lower_32_bits(f->coeffs[i]), dec + DEC_COEFF + 8 * i);
		writel_relaxed(upper_32_bits(f->coeffs[i]), dec + DEC_COEFF + 8 * i + 4);
	}
	writel_relaxed(1, dec + DEC_LOAD);
	writel_relaxed(t8010_pdm_dec_timing0[idx], dec + DEC_TIMING0);
	writel_relaxed(t8010_pdm_dec_timing1[idx], dec + DEC_TIMING1);
}

static void t8010_pdm_dec_gate(struct t8010_pdm *pdm, bool gate)
{
	unsigned int i;

	for (i = 0; i < pdm->channels; i++)
		writel_relaxed(gate ? DEC_GATE_ALL : 0, pdm->dec[i] + DEC_GATE);
}

/* The PDM clock and the decimators' clocks, as iOS's setClockEnable */
static int t8010_pdm_clock_on(struct t8010_pdm *pdm)
{
	int ret;

	ret = clk_set_rate(pdm->clk, pdm->filter->pdm_rate);
	if (ret)
		return ret;
	ret = clk_prepare_enable(pdm->clk);
	if (ret)
		return ret;
	t8010_pdm_dec_gate(pdm, false);
	pdm->clk_on = true;
	/* the microphones need this long after the clock starts */
	msleep(PDM_MIC_STARTUP_MS);
	return 0;
}

static void t8010_pdm_clock_off(struct t8010_pdm *pdm)
{
	if (!pdm->clk_on)
		return;
	t8010_pdm_dec_gate(pdm, true);
	clk_disable_unprepare(pdm->clk);
	pdm->clk_on = false;
}

static int t8010_pdm_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);

	return pm_runtime_resume_and_get(pdm->dev);
}

static void t8010_pdm_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);

	t8010_pdm_clock_off(pdm);
	pm_runtime_put(pdm->dev);
}

static int t8010_pdm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);
	const struct t8010_pdm_filter *filter = NULL;
	unsigned int channels = params_channels(params);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(t8010_pdm_filters); i++)
		if (t8010_pdm_filters[i].rate == params_rate(params))
			filter = &t8010_pdm_filters[i];
	if (!filter || channels > pdm->num_mics ||
	    params_width(params) != 16)
		return -EINVAL;

	if (pdm->clk_on && (filter != pdm->filter || channels != pdm->channels))
		t8010_pdm_clock_off(pdm);
	pdm->filter = filter;
	pdm->channels = channels;

	pdm->dma_rx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	pdm->dma_rx.maxburst = round_up(params_rate(params) * channels / 1000, 4);

	if (!pdm->clk_on)
		return t8010_pdm_clock_on(pdm);
	return 0;
}

static int t8010_pdm_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	t8010_pdm_clock_off(snd_soc_dai_get_drvdata(dai));
	return 0;
}

static u32 t8010_pdm_ctrl(struct t8010_pdm *pdm)
{
	u32 ctrl = PDM_CTRL_BASE;
	unsigned int i;

	for (i = 0; i < pdm->channels; i++)
		ctrl |= PDM_CTRL_MIC(i);
	return ctrl;
}

/*
 * ApplePDMControllerT8010's start up to its DMA start: flush and reset,
 * program the controller and the decimators, clear the FIFO errors.
 */
static int t8010_pdm_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;
	unsigned int i;
	int ret = 0;

	spin_lock_irqsave(&pdm->lock, flags);
	writel_relaxed(PDM_FLUSH_GO, pdm->base + PDM_FLUSH);
	t8010_pdm_wait_clear(pdm, PDM_FLUSH, PDM_FLUSH_GO);
	if (readl_relaxed(pdm->base + PDM_STATUS) & PDM_STATUS_BUSY) {
		dev_err(pdm->dev, "still running\n");
		ret = -EBUSY;
		goto out;
	}
	writel_relaxed(PDM_RESET_FIFO, pdm->base + PDM_RESET);
	t8010_pdm_wait_clear(pdm, PDM_RESET, PDM_RESET_FIFO);

	writel_relaxed(t8010_pdm_ctrl(pdm), pdm->base + PDM_CTRL);
	writel_relaxed(PDM_FIFO_CFG_IOS, pdm->base + PDM_FIFO_CFG);
	writel_relaxed(FIELD_PREP(PDM_SAMPLE_SHIFT, PDM_SHIFT_UNITY - pdm->gain),
		       pdm->base + PDM_SAMPLE);
	writel_relaxed(FIELD_PREP(PDM_WATERMARK_LEVEL, PDM_WATERMARK_WORDS - 1) |
		       PDM_WATERMARK_BASE, pdm->base + PDM_WATERMARK);

	for (i = 0; i < pdm->channels; i++)
		t8010_pdm_dec_configure(pdm, i);

	writel_relaxed(PDM_IRQ_UNDERFLOW | PDM_IRQ_OVERFLOW,
		       pdm->base + PDM_IRQ_STATUS);
out:
	spin_unlock_irqrestore(&pdm->lock, flags);
	return ret;
}

/* The rest of the start, after the DMA start (ASoC starts the DMA first) */
static void t8010_pdm_start(struct t8010_pdm *pdm)
{
	u32 ctrl = t8010_pdm_ctrl(pdm);

	writel_relaxed(PDM_IRQ_MASK_RUN, pdm->base + PDM_IRQ_MASK);
	writel_relaxed(ctrl | PDM_CTRL_RUN, pdm->base + PDM_CTRL);
	readl_relaxed(pdm->base + PDM_CTRL);
	writel_relaxed(1, pdm->base + PDM_START);
}

static void t8010_pdm_stop(struct t8010_pdm *pdm)
{
	u32 val;

	writel_relaxed(PDM_IRQ_MASK_STOP, pdm->base + PDM_IRQ_MASK);
	writel_relaxed(PDM_CTRL_BASE | PDM_CTRL_STOP, pdm->base + PDM_CTRL);
	if (readl_relaxed_poll_timeout_atomic(pdm->base + PDM_STATUS, val,
					      !(val & PDM_STATUS_BUSY), 10,
					      PDM_TIMEOUT_US))
		dev_warn(pdm->dev, "did not stop\n");
	writel_relaxed(PDM_FLUSH_GO, pdm->base + PDM_FLUSH);
	t8010_pdm_wait_clear(pdm, PDM_FLUSH, PDM_FLUSH_GO);
	writel_relaxed(PDM_RESET_FIFO, pdm->base + PDM_RESET);
	t8010_pdm_wait_clear(pdm, PDM_RESET, PDM_RESET_FIFO);
}

static int t8010_pdm_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		spin_lock_irqsave(&pdm->lock, flags);
		t8010_pdm_start(pdm);
		spin_unlock_irqrestore(&pdm->lock, flags);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		spin_lock_irqsave(&pdm->lock, flags);
		t8010_pdm_stop(pdm);
		spin_unlock_irqrestore(&pdm->lock, flags);
		return 0;
	}
	return -EINVAL;
}

static int t8010_pdm_dai_probe(struct snd_soc_dai *dai)
{
	struct t8010_pdm *pdm = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, NULL, &pdm->dma_rx);
	return 0;
}

static const struct snd_soc_dai_ops t8010_pdm_dai_ops = {
	.probe = t8010_pdm_dai_probe,
	.startup = t8010_pdm_startup,
	.shutdown = t8010_pdm_shutdown,
	.hw_params = t8010_pdm_hw_params,
	.hw_free = t8010_pdm_hw_free,
	.prepare = t8010_pdm_prepare,
	.trigger = t8010_pdm_trigger,
};

static const struct snd_soc_dai_driver t8010_pdm_dai = {
	.name = "t8010-pdm",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = PDM_MAX_MICS,
		.rates = PDM_RATES,
		.formats = PDM_FORMATS,
	},
	.ops = &t8010_pdm_dai_ops,
};

static int t8010_pdm_gain_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = pdm->gain;
	return 0;
}

/* Takes effect at once: the shift can change while the stream runs. */
static int t8010_pdm_gain_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	long gain = ucontrol->value.integer.value[0];
	unsigned long flags;

	if (gain < 0 || gain > PDM_GAIN_MAX)
		return -EINVAL;
	if (gain == pdm->gain)
		return 0;

	spin_lock_irqsave(&pdm->lock, flags);
	pdm->gain = gain;
	if (pdm->clk_on)
		writel_relaxed(FIELD_PREP(PDM_SAMPLE_SHIFT,
					  PDM_SHIFT_UNITY - pdm->gain),
			       pdm->base + PDM_SAMPLE);
	spin_unlock_irqrestore(&pdm->lock, flags);
	return 1;
}

static const DECLARE_TLV_DB_SCALE(t8010_pdm_gain_tlv, 0, 600, 0);

static const struct snd_kcontrol_new t8010_pdm_controls[] = {
	SOC_SINGLE_EXT_TLV("Mic Capture Volume", SND_SOC_NOPM, 0, PDM_GAIN_MAX,
			   0, t8010_pdm_gain_get, t8010_pdm_gain_put,
			   t8010_pdm_gain_tlv),
};

static const struct snd_soc_component_driver t8010_pdm_component = {
	.name = "t8010-pdm",
	.controls = t8010_pdm_controls,
	.num_controls = ARRAY_SIZE(t8010_pdm_controls),
};

/*
 * SmartIO queues one command per period and reports the position per
 * completed period, which PipeWire needs to know (BATCH).
 */
static const struct snd_pcm_hardware t8010_pdm_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
	.formats = PDM_FORMATS,
	.rates = PDM_RATES,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = PDM_MAX_MICS,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 512,
	.period_bytes_max = 32 * 1024,
	.periods_min = 2,
	.periods_max = 16,
};

static int t8010_pdm_prepare_slave_config(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params,
					  struct dma_slave_config *sc)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct t8010_pdm *pdm =
		snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));
	int ret;

	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, sc);
	if (ret)
		return ret;

	/*
	 * The shim record of the "dma-channels" entry, and as FIFO depth one
	 * millisecond of samples rounded to 4, as ApplePDMControllerT8010
	 * sets it.
	 */
	pdm->sio_cfg = (struct apple_sio_dma_config) {
		.xfer = PDM_SIO_XFER16,
		.fifo = pdm->dma_hw->fifo,
		.trigger = pdm->dma_hw->trigger,
		.limit = 2 * round_up(params_rate(params) *
				      params_channels(params) / 1000, 4),
	};
	sc->peripheral_config = &pdm->sio_cfg;
	sc->peripheral_size = sizeof(pdm->sio_cfg);
	return 0;
}

static const struct snd_dmaengine_pcm_config t8010_pdm_dma_config = {
	.prepare_slave_config = t8010_pdm_prepare_slave_config,
	.pcm_hardware = &t8010_pdm_pcm_hw,
	.prealloc_buffer_size = 128 * 1024,
};

static irqreturn_t t8010_pdm_irq(int irq, void *data)
{
	struct t8010_pdm *pdm = data;
	u32 status;

	spin_lock(&pdm->lock);
	status = readl_relaxed(pdm->base + PDM_IRQ_STATUS);
	writel_relaxed(status, pdm->base + PDM_IRQ_STATUS);
	spin_unlock(&pdm->lock);

	status &= PDM_IRQ_UNDERFLOW | PDM_IRQ_OVERFLOW;
	if (!status)
		return IRQ_NONE;
	dev_warn_ratelimited(pdm->dev, "FIFO %s\n",
			     status & PDM_IRQ_OVERFLOW ? "overflow" : "underflow");
	return IRQ_HANDLED;
}

static int t8010_pdm_runtime_suspend(struct device *dev)
{
	struct t8010_pdm *pdm = dev_get_drvdata(dev);

	writel_relaxed(PDM_IRQ_MASK_STOP, pdm->base + PDM_IRQ_MASK);
	return 0;
}

static int t8010_pdm_runtime_resume(struct device *dev)
{
	struct t8010_pdm *pdm = dev_get_drvdata(dev);

	writel_relaxed(PDM_IRQ_MASK_STOP, pdm->base + PDM_IRQ_MASK);
	return 0;
}

static const struct dev_pm_ops t8010_pdm_pm_ops = {
	RUNTIME_PM_OPS(t8010_pdm_runtime_suspend, t8010_pdm_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static int t8010_pdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct t8010_pdm *pdm;
	unsigned int i;
	int irq, ret;
	u32 fifo;

	pdm = devm_kzalloc(dev, sizeof(*pdm), GFP_KERNEL);
	if (!pdm)
		return -ENOMEM;
	pdm->dev = dev;
	pdm->dma_hw = device_get_match_data(dev);
	if (!pdm->dma_hw)
		return -ENODEV;
	pdm->gain = PDM_GAIN_DEFAULT;
	spin_lock_init(&pdm->lock);
	platform_set_drvdata(pdev, pdm);

	pdm->base = devm_platform_ioremap_resource_byname(pdev, "pdm");
	if (IS_ERR(pdm->base))
		return PTR_ERR(pdm->base);
	for (i = 0; i < PDM_MAX_MICS; i++) {
		char name[8];

		snprintf(name, sizeof(name), "dec%u", i);
		if (platform_get_resource_byname(pdev, IORESOURCE_MEM, name) == NULL)
			break;
		pdm->dec[i] = devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(pdm->dec[i]))
			return PTR_ERR(pdm->dec[i]);
	}
	pdm->num_mics = i;
	if (!pdm->num_mics)
		return dev_err_probe(dev, -EINVAL, "no decimators\n");

	pdm->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pdm->clk))
		return dev_err_probe(dev, PTR_ERR(pdm->clk), "no PDM clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, t8010_pdm_irq, 0, dev_name(dev), pdm);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power up\n");
	fifo = readl_relaxed(pdm->base + PDM_FIFO_INFO);
	writel_relaxed(PDM_IRQ_MASK_STOP, pdm->base + PDM_IRQ_MASK);
	pm_runtime_put(dev);
	dev_dbg(dev, "%u microphones, FIFO depth %lu\n", pdm->num_mics,
		FIELD_GET(PDM_FIFO_INFO_DEPTH, fifo));

	pdm->dai_drv = t8010_pdm_dai;
	pdm->dai_drv.capture.channels_max = pdm->num_mics;
	ret = devm_snd_soc_register_component(dev, &t8010_pdm_component,
					      &pdm->dai_drv, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	pdm->pcm_config = t8010_pdm_dma_config;
	ret = devm_snd_dmaengine_pcm_register(dev, &pdm->pcm_config, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register PCM\n");

	return 0;
}

static const struct t8010_pdm_dma_hw t8010_pdm_dma_hw = {
	.fifo = PDM_SIO_FIFO,
	.trigger = PDM_SIO_TRIGGER,
};

/* J120 PDM 16-bit DMA record, channel 0x5b. */
static const struct t8010_pdm_dma_hw t8011_pdm_dma_hw = {
	.fifo = 0x600,
	.trigger = 0xac,
};

static const struct of_device_id t8010_pdm_of_match[] = {
	{ .compatible = "apple,t8010-pdm", .data = &t8010_pdm_dma_hw },
	{ .compatible = "apple,t8011-pdm", .data = &t8011_pdm_dma_hw },
	{}
};
MODULE_DEVICE_TABLE(of, t8010_pdm_of_match);

static struct platform_driver t8010_pdm_driver = {
	.driver = {
		.name = "apple-t8010-pdm",
		.of_match_table = t8010_pdm_of_match,
		.pm = pm_ptr(&t8010_pdm_pm_ops),
	},
	.probe = t8010_pdm_probe,
};
module_platform_driver(t8010_pdm_driver);

MODULE_DESCRIPTION("Apple T8010 PDM microphone controller");
MODULE_LICENSE("Dual MIT/GPL");
