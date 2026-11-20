// SPDX-License-Identifier: GPL-2.0-only
/* J120 uses four MAX98723 amplifiers sharing one reset line. */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>

struct speaker_bank {
	struct device *dev;
	struct gpio_desc *reset;
	struct regmap *amps[4];
	struct mutex lock;
	unsigned int users;
	unsigned long enabled_mask;
	bool faulted;
};

static const struct regmap_config amp_registers = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0x1ff,
};

static int configure_amp(struct regmap *amp, unsigned int slot)
{
	unsigned int enabled, value, i;
	unsigned int feedback = 3 << (2 * slot);
	const struct reg_sequence serial[] = {
		{ 0x3a, 0 },		/* Keep the speaker muted. */
		{ 0x18, BIT(slot) },	/* PCM input */
		{ 0x21, 0x65 }, { 0x22, 0 }, { 0x25, 0x88 },
		{ 0x1c, 0xff ^ feedback }, { 0x1d, 0xff },
		{ 0x1a, feedback }, { 0x1b, 0 },
		{ 0x1e, 0x10 + 0x22 * slot }, { 0x3e, 3 },
		/* J120 speaker-config from the board device tree. */
		{ 0x28, 0xf ^ BIT(slot) }, { 0x2b, BIT(slot) },
		{ 0x2e, 2 }, { 0x31, 2 }, { 0x37, 0x33 }, { 0x3f, 0xf7 },
	};
	int ret;

	/* Slots 0 and 1 are the connector-side amplifiers. */
	if (slot < 2) {
		ret = regmap_write(amp, 0x100, 1);
		if (ret)
			return dev_err_probe(regmap_get_device(amp), ret,
					     "software reset write failed\n");
		msleep(12);
	}
	ret = regmap_read(amp, 0xff, &enabled);
	if (ret)
		return dev_err_probe(regmap_get_device(amp), ret,
				     "initial global enable read failed\n");
	if (enabled)
		return -EBUSY;

	ret = regmap_multi_reg_write(amp, serial, ARRAY_SIZE(serial));
	if (ret)
		return ret;
	ret = regmap_update_bits(amp, 0x26, 0xcf, slot);
	if (ret)
		return ret;
	ret = regmap_update_bits(amp, 0x23, 0x0f, 8);
	if (ret)
		return ret;
	ret = regmap_update_bits(amp, 0x24, 0x0f, 8);
	if (ret)
		return ret;
	ret = regmap_update_bits(amp, 0x42, 0x3f, 0x28);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(serial); i++) {
		ret = regmap_read(amp, serial[i].reg, &value);
		if (ret)
			return ret;
		if (value != serial[i].def)
			return -EIO;
	}
	ret = regmap_read(amp, 0x42, &value);
	if (ret)
		return ret;
	return (value & 0x3f) == 0x28 ? 0 : -EIO;
}

static int bank_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct speaker_bank *bank = dev_get_drvdata(dai->dev);
	unsigned int i;
	int ret;

	guard(mutex)(&bank->lock);
	if (bank->users && bank->faulted)
		return -EIO;
	if (!bank->users) {
		bank->faulted = false;
		gpiod_set_value_cansleep(bank->reset, 0);
		msleep(100);
		for (i = 0; i < ARRAY_SIZE(bank->amps); i++) {
			ret = configure_amp(bank->amps[i], i);
			if (ret) {
				gpiod_set_value_cansleep(bank->reset, 1);
				dev_err(bank->dev, "amp %u: startup failed: %d\n", i, ret);
				return ret;
			}
		}
	}
	bank->users++;
	return 0;
}

static int set_output(struct speaker_bank *bank, unsigned int slot, bool on)
{
	struct regmap *amp = bank->amps[slot];
	const unsigned int regs[] = { on ? 0x3a : 0xff, on ? 0xff : 0x3a };
	unsigned int i, value;
	int ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_write(amp, regs[i], on);
		if (ret)
			goto reset;
		/* Allow global enable to settle before readback. */
		if (on && regs[i] == 0xff)
			usleep_range(2000, 2500);
		ret = regmap_read(amp, regs[i], &value);
		if (ret)
			goto reset;
		if ((value & 1) != on) {
			ret = -EIO;
			goto reset;
		}
	}
	__assign_bit(slot, &bank->enabled_mask, on);
	return 0;

reset:
	gpiod_set_value_cansleep(bank->reset, 1);
	bank->faulted = true;
	bank->enabled_mask = 0;
	dev_err(bank->dev, "amp %u: output transition failed, reset asserted: %d\n",
		slot, ret);
	return ret;
}

static int mute_outputs(struct speaker_bank *bank)
{
	int ret;

	while (bank->enabled_mask) {
		ret = set_output(bank, __ffs(bank->enabled_mask), false);
		if (ret)
			return ret;
	}
	return 0;
}

static int bank_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct speaker_bank *bank = dev_get_drvdata(dai->dev);

	unsigned int slot;
	int ret;

	guard(mutex)(&bank->lock);
	if (mute)
		return mute_outputs(bank);
	if (bank->faulted)
		return -EIO;
	for (slot = 0; slot < ARRAY_SIZE(bank->amps); slot++) {
		if (test_bit(slot, &bank->enabled_mask))
			continue;
		ret = set_output(bank, slot, true);
		if (ret)
			return ret;
	}
	return 0;
}

static void bank_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct speaker_bank *bank = dev_get_drvdata(dai->dev);

	guard(mutex)(&bank->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mute_outputs(bank);
	if (--bank->users == 0)
		gpiod_set_value_cansleep(bank->reset, 1);
}

static const struct snd_soc_dai_ops bank_ops = {
	.startup = bank_startup,
	.shutdown = bank_shutdown,
	.mute_stream = bank_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver bank_dai = {
	.name = "max98723-bank",
	.playback = {
		.stream_name = "Speakers", .channels_min = 4, .channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000, .formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Feedback", .channels_min = 8, .channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000, .formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &bank_ops,
};

static const struct snd_soc_component_driver bank_component = {
	.name = "j120-max98723",
};

static int bank_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct speaker_bank *bank;
	unsigned int i;

	bank = devm_kzalloc(dev, sizeof(*bank), GFP_KERNEL);
	if (!bank)
		return -ENOMEM;
	bank->dev = dev;
	mutex_init(&bank->lock);
	bank->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(bank->reset))
		return PTR_ERR(bank->reset);
	for (i = 0; i < ARRAY_SIZE(bank->amps); i++) {
		struct device_node *bus;
		struct i2c_adapter *adapter;
		struct i2c_client *amp;

		bus = of_parse_phandle(dev->of_node, "speaker-buses", i / 2);
		if (!bus)
			return -EINVAL;
		adapter = of_get_i2c_adapter_by_node(bus);
		of_node_put(bus);
		if (!adapter)
			return -EPROBE_DEFER;
		amp = devm_i2c_new_dummy_device(dev, adapter, 0x31 + i % 2);
		i2c_put_adapter(adapter);
		if (IS_ERR(amp))
			return PTR_ERR(amp);
		bank->amps[i] = devm_regmap_init_i2c(amp, &amp_registers);
		if (IS_ERR(bank->amps[i]))
			return PTR_ERR(bank->amps[i]);
	}

	platform_set_drvdata(pdev, bank);
	return devm_snd_soc_register_component(dev, &bank_component, &bank_dai, 1);
}

static const struct of_device_id bank_match[] = {
	{ .compatible = "apple,j120-speaker-amplifiers" },
	{}
};
MODULE_DEVICE_TABLE(of, bank_match);

static struct platform_driver bank_driver = {
	.probe = bank_probe,
	.driver = { .name = "j120-max98723", .of_match_table = bank_match },
};
module_platform_driver(bank_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("J120 MAX98723 speaker amplifiers");
