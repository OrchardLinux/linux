// SPDX-License-Identifier: GPL-2.0-only
/*
 * MAX98721 speaker amplifier, as used in the Apple iPad Air 2
 *
 * The MAX98721 has no public datasheet. Every register that iOS touches
 * matches the MAX98925 map, so this driver uses that map. The sequence
 * follows iOS 15's AppleMAX98721Amp: TDM consumer clocked from BCLK, one
 * playback slot per amplifier, the protection settings Apple's device tree
 * gives (limiter, boost and boost current limit) and the global enable only
 * while a stream runs.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "max98925.h"

/* FORMAT as iOS sets it: TDM, one bit of delay, BCI, EXTBCLK_HIZ. */
#define MAX98721_FORMAT_TDM	(M98925_DAI_EXTBCLK_HIZ_MASK | \
				 M98925_DAI_BCI_MASK | \
				 M98925_DAI_DLY_MASK | \
				 M98925_DAI_TDM_MASK)

/* DAI_CLK_MODE1: clocked from BCLK, MDLL multiplier 6, as iOS sets it. */
#define MAX98721_CLK_MODE1_MASK	(M98925_DAI_CLK_SOURCE_MASK | \
				 M98925_MDLL_MULT_MASK)
#define MAX98721_CLK_MODE1	(M98925_DAI_CLK_SOURCE_MASK | 6)

/* BLOCK_ENABLE: boost, clock monitor and speaker; no IV-sense ADCs. */
#define MAX98721_BLOCK_EN	(M98925_BST_EN_MASK | M98925_CLKMON_EN_MASK | \
				 M98925_SPK_EN_MASK)

/* The gain iOS uses on J81 (speaker-config byte 1); never exceeded. */
#define MAX98721_GAIN_MAX	0x16

struct max98721_priv {
	struct regmap *regmap;
};

/*
 * Written once at probe, with both enables off. The protection values are
 * the iPad Air 2's speaker-config from Apple's device tree; both amplifiers
 * use the same record.
 */
static const struct reg_sequence max98721_init[] = {
	{ MAX98925_GLOBAL_ENABLE,	0x00 },
	{ MAX98925_BLOCK_ENABLE,	0x00 },
	/* Flag-to-DOUT map, as iOS writes it. */
	{ MAX98925_MAP0,		0xc0 },
	{ MAX98925_MAP1,		0x00 },
	{ MAX98925_MAP2,		0x00 },
	{ MAX98925_MAP3,		0xf0 },
	{ MAX98925_MAP4,		0x00 },
	{ MAX98925_MAP5,		0xab },
	{ MAX98925_MAP6,		0x89 },
	{ MAX98925_MAP7,		0x00 },
	{ MAX98925_MAP8,		0x00 },
	{ MAX98925_FORMAT,		M98925_DAI_CHANSZ_16 |
					MAX98721_FORMAT_TDM },
	/* Playback slot 0; DOUT (IV sense) off and Hi-Z in every slot. */
	{ MAX98925_TDM_SLOT_SELECT,	M98925_DAI_DIN_EN_MASK },
	{ MAX98925_DOUT_CFG_VMON,	0x00 },
	{ MAX98925_DOUT_CFG_IMON,	0x00 },
	{ MAX98925_DOUT_CFG_VBAT,	0x00 },
	{ MAX98925_DOUT_CFG_VBST,	0x00 },
	{ MAX98925_DOUT_CFG_FLAG,	0x00 },
	{ MAX98925_DOUT_HIZ_CFG1,	0xff },
	{ MAX98925_DOUT_HIZ_CFG2,	0xff },
	{ MAX98925_DOUT_HIZ_CFG3,	0xff },
	{ MAX98925_DOUT_HIZ_CFG4,	0xff },
	{ MAX98925_DOUT_DRV_STRENGTH,	0x02 },
	/* speaker-config bytes 0 and 2-9 */
	{ MAX98925_FILTERS,		0xd1 },	/* dither, IV DC block, DC-block HPF */
	{ MAX98925_GAIN_RAMPING,	0x00 },
	{ MAX98925_SPK_AMP,		0x00 },
	{ MAX98925_THRESHOLD,		0x2a },	/* ALC on, threshold 0x0a */
	{ MAX98925_ALC_ATTACK,		0x00 },
	{ MAX98925_ALC_ATTEN_RLS,	0x84 },
	{ MAX98925_ALC_HOLD_RLS,	0x00 },
	{ MAX98925_ALC_CONFIGURATION,	0x03 },
	{ MAX98925_BOOST_CONVERTER,	0x01 },
	{ MAX98925_CONFIGURATION,	0x03 },	/* iOS ORs 3 into byte 9 (0) */
	/* Clear stale interrupt flags. */
	{ MAX98925_IRQ_CLEAR0,		0xff },
	{ MAX98925_IRQ_CLEAR1,		0xff },
	{ MAX98925_IRQ_CLEAR2,		0xff },
};

/* 0 mutes; 1 is -6 dB, in 1 dB steps (inferred from iOS writing 0 to mute). */
static const DECLARE_TLV_DB_SCALE(max98721_gain_tlv, -700, 100, 1);

static const struct snd_kcontrol_new max98721_controls[] = {
	SOC_SINGLE_TLV("Speaker Volume", MAX98925_GAIN, M98925_SPK_GAIN_SHIFT,
		       MAX98721_GAIN_MAX, 0, max98721_gain_tlv),
};

/*
 * iOS turns the blocks on first and the global enable last, then waits
 * 1.5 ms; it turns the global enable off first and waits 9 ms before it
 * turns the blocks off.
 */
static int max98721_amp_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct max98721_priv *priv = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return regmap_write(priv->regmap, MAX98925_BLOCK_ENABLE,
				    MAX98721_BLOCK_EN);
	case SND_SOC_DAPM_POST_PMU:
		regmap_write(priv->regmap, MAX98925_GLOBAL_ENABLE,
			     M98925_EN_MASK);
		usleep_range(1500, 2000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_write(priv->regmap, MAX98925_GLOBAL_ENABLE, 0);
		usleep_range(9000, 10000);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_write(priv->regmap, MAX98925_BLOCK_ENABLE, 0);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget max98721_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIFIN", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       max98721_amp_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route max98721_dapm_routes[] = {
	{ "Amp", NULL, "AIFIN" },
	{ "OUT", NULL, "Amp" },
};

/* The sample rate codes iOS uses, which match the MAX98925's. */
static const unsigned int max98721_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
};

static int max98721_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct max98721_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int chansz, sr;
	int i;

	switch (params_width(params)) {
	case 16:
		chansz = M98925_DAI_CHANSZ_16;
		break;
	case 24:
		chansz = M98925_DAI_CHANSZ_24;
		break;
	case 32:
		chansz = M98925_DAI_CHANSZ_32;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(max98721_rates); i++)
		if (max98721_rates[i] == params_rate(params))
			break;
	if (i == ARRAY_SIZE(max98721_rates))
		return -EINVAL;
	sr = i;

	regmap_write(priv->regmap, MAX98925_FORMAT,
		     chansz | MAX98721_FORMAT_TDM);
	regmap_update_bits(priv->regmap, MAX98925_DAI_CLK_MODE1,
			   MAX98721_CLK_MODE1_MASK, MAX98721_CLK_MODE1);
	/*
	 * Consumer (DAI_MAS clear), with the 256-BCLK frame iOS sets for every
	 * word size: the J81's MCA3 pads its eight slots to 256 bits.
	 */
	return regmap_write(priv->regmap, MAX98925_DAI_CLK_MODE2,
			    (sr << M98925_DAI_SR_SHIFT) | M98925_DAI_BSEL_256);
}

static int max98721_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The only mode iOS uses: MCA-clocked TDM, data one bit late. */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC ||
	    (fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

/* The amplifier plays the first slot of rx_mask, on both of its inputs. */
static int max98721_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				 unsigned int rx_mask, int slots, int slot_width)
{
	struct max98721_priv *priv = snd_soc_component_get_drvdata(dai->component);
	unsigned int slot = 0;

	if (slots) {
		if (!rx_mask || __ffs(rx_mask) > 7)
			return -EINVAL;
		slot = __ffs(rx_mask);
	}

	return regmap_write(priv->regmap, MAX98925_TDM_SLOT_SELECT,
			    M98925_DAI_DIN_EN_MASK |
			    (slot << M98925_DAI_INR_SOURCE_SHIFT) |
			    (slot << M98925_DAI_INL_SOURCE_SHIFT));
}

static const struct snd_soc_dai_ops max98721_dai_ops = {
	.hw_params	= max98721_hw_params,
	.set_fmt	= max98721_set_fmt,
	.set_tdm_slot	= max98721_set_tdm_slot,
};

static struct snd_soc_dai_driver max98721_dai = {
	.name = "max98721-aif",
	.playback = {
		.stream_name	= "HiFi Playback",
		.channels_min	= 1,
		.channels_max	= 8,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &max98721_dai_ops,
};

static const struct snd_soc_component_driver max98721_component = {
	.controls		= max98721_controls,
	.num_controls		= ARRAY_SIZE(max98721_controls),
	.dapm_widgets		= max98721_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(max98721_dapm_widgets),
	.dapm_routes		= max98721_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(max98721_dapm_routes),
	.endianness		= 1,
};

static bool max98721_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg <= MAX98925_IRQ_CLEAR2 || reg == MAX98925_REV_VERSION;
}

static bool max98721_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX98925_VBAT_DATA ... MAX98925_IRQ_ENABLE2:
	case MAX98925_MAP0 ... MAX98925_ALC_ATTEN_RLS:
	case MAX98925_ALC_CONFIGURATION ... MAX98925_GLOBAL_ENABLE:
	case MAX98925_BOOST_LIMITER:
	case MAX98925_REV_VERSION:
		return true;
	default:
		return false;
	}
}

static bool max98721_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX98925_IRQ_ENABLE0 ... MAX98925_GLOBAL_ENABLE:
	case MAX98925_BOOST_LIMITER:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config max98721_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= MAX98925_REV_VERSION,
	.volatile_reg	= max98721_volatile_reg,
	.readable_reg	= max98721_readable_reg,
	.writeable_reg	= max98721_writeable_reg,
	.cache_type	= REGCACHE_MAPLE,
};

/* iOS reads and clears the flags but enables no interrupt; report a few. */
static irqreturn_t max98721_irq(int irq, void *data)
{
	struct device *dev = data;
	struct max98721_priv *priv = dev_get_drvdata(dev);
	unsigned int flag[3] = {};
	int i;

	for (i = 0; i < 3; i++)
		regmap_read(priv->regmap, MAX98925_FLAG0 + i, &flag[i]);
	for (i = 0; i < 3; i++)
		regmap_write(priv->regmap, MAX98925_IRQ_CLEAR0 + i, 0xff);

	if (!(flag[0] | flag[1] | flag[2]))
		return IRQ_NONE;

	dev_warn_ratelimited(dev, "interrupt: flags %02x %02x %02x\n",
			     flag[0], flag[1], flag[2]);
	return IRQ_HANDLED;
}

static int max98721_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct max98721_priv *priv;
	unsigned int rev;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	i2c_set_clientdata(i2c, priv);

	priv->regmap = devm_regmap_init_i2c(i2c, &max98721_regmap);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	/* iOS reads the revision only to see the part answer. */
	ret = regmap_read(priv->regmap, MAX98925_REV_VERSION, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read revision\n");
	dev_info(dev, "MAX98721 revision 0x%02x\n", rev);

	ret = regmap_multi_reg_write(priv->regmap, max98721_init,
				     ARRAY_SIZE(max98721_init));
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise\n");

	/* Start muted; the volume control never goes above iOS's gain. */
	regmap_update_bits(priv->regmap, MAX98925_GAIN, M98925_SPK_GAIN_MASK, 0);
	/* Boost current limit from speaker-config byte 10 (0x50). */
	regmap_update_bits(priv->regmap, MAX98925_BOOST_LIMITER,
			   M98925_BST_ILIM_MASK, 0x50);

	if (i2c->irq) {
		ret = devm_request_threaded_irq(dev, i2c->irq, NULL,
						max98721_irq, IRQF_ONESHOT,
						dev_name(dev), dev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to get IRQ\n");

		/* Thermal shutdown/warning, speaker overcurrent, slot errors. */
		regmap_write(priv->regmap, MAX98925_IRQ_ENABLE0,
			     M98925_THERMSHDN_BGN_EN_MASK |
			     M98925_THERMWARN_BGN_EN_MASK);
		regmap_write(priv->regmap, MAX98925_IRQ_ENABLE1,
			     M98925_SPKCURNT_EN_MASK);
		regmap_write(priv->regmap, MAX98925_IRQ_ENABLE2,
			     M98925_SLOTOVRN_EN_MASK | M98925_INVALSLOT_EN_MASK |
			     M98925_SLOTCNFLT_EN_MASK);
	}

	return devm_snd_soc_register_component(dev, &max98721_component,
					       &max98721_dai, 1);
}

static const struct i2c_device_id max98721_i2c_id[] = {
	{ "max98721" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max98721_i2c_id);

static const struct of_device_id max98721_of_match[] = {
	{ .compatible = "maxim,max98721" },
	{ }
};
MODULE_DEVICE_TABLE(of, max98721_of_match);

static struct i2c_driver max98721_i2c_driver = {
	.driver = {
		.name = "max98721",
		.of_match_table = max98721_of_match,
	},
	.probe = max98721_i2c_probe,
	.id_table = max98721_i2c_id,
};
module_i2c_driver(max98721_i2c_driver);

MODULE_DESCRIPTION("ASoC MAX98721 speaker amplifier driver");
MODULE_LICENSE("GPL");
