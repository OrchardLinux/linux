// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple AOP audio service: the PDM microphone clock
 *
 * On the A10 the built-in microphones are recorded by the application
 * processor's PDM controller, but their clock comes from the Always-On
 * Processor. iPadOS's PDM driver asks for it through a platform function
 * of the AOP audio driver, which sets property 201 of the audio service's
 * "pdm0" device: a start word and the clock rate to run the clock, a stop
 * word to halt it. The calls are wrapped audio calls, as on the M1's AOP.
 *
 * The firmware talks to its devices only after they were attached, so the
 * driver attaches "pdm0" when it binds on A10. A10X firmware instead
 * accepts a direct clock command with a source index and a rate in Hz.
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/aop.h>

#define AOP_AUDIO_COMMAND		0x20
#define AOP_AUDIO_CALL_ATTACH		0xc3000002
#define AOP_AUDIO_CALL_SET_PROP		0xc3000005

#define AOP_AUDIO_DEV_PDM0		0x70646d30	/* 'pdm0' */
#define AOP_AUDIO_PROP_PDM_CLOCK	201
#define AOP_AUDIO_PDM_CLOCK_START	0x78693020	/* 'xi0 ' */
#define AOP_AUDIO_PDM_CLOCK_STOP	0x6f666620	/* 'off ' */

#define AOP_T8011_CLOCK_DISABLE		0
#define AOP_T8011_CLOCK_XIO		1

struct aop_t8011_clock_config {
	__le32 reserved;	/* EPIC prefix before the clock payload */
	__le32 source;
	__le32 rate;
};

struct aop_audio_call_hdr {
	__le32 zero0;
	__le32 target;		/* 0xffffffff */
	__le32 type;
	u8 zero1[20];
	__le64 len;		/* size of the call minus 4 */
} __packed;

struct aop_audio_attach {
	struct aop_audio_call_hdr hdr;
	__le32 dev;
	__le32 zero;
} __packed;

struct aop_audio_pdm_clock {
	struct aop_audio_call_hdr hdr;
	__le32 dev;
	__le32 prop;
	__le32 size;
	__le32 mode;
	__le32 rate;
} __packed;

struct aop_audio {
	struct device *dev;
	struct apple_aop_service service;
	struct clk_hw hw;
	unsigned long rate;
	bool direct_clock;
};

static void aop_audio_hdr(struct aop_audio_call_hdr *hdr, u32 type, size_t size)
{
	*hdr = (struct aop_audio_call_hdr) {
		.target = cpu_to_le32(0xffffffff),
		.type = cpu_to_le32(type),
		.len = cpu_to_le64(size - 4),
	};
}

static int aop_audio_call(struct aop_audio *audio, const void *call, size_t size,
			  const char *what)
{
	int ret;

	ret = apple_aop_call(&audio->service, AOP_AUDIO_COMMAND, call, size,
			     NULL, 0);
	if (ret > 0) {
		dev_err(audio->dev, "%s: firmware returned %#x\n", what, ret);
		return -EIO;
	}
	if (ret < 0)
		dev_err(audio->dev, "%s: %d\n", what, ret);
	return ret;
}

static int aop_audio_pdm_clock(struct aop_audio *audio, u32 mode, u32 rate)
{
	struct aop_audio_pdm_clock call = {
		.dev = cpu_to_le32(AOP_AUDIO_DEV_PDM0),
		.prop = cpu_to_le32(AOP_AUDIO_PROP_PDM_CLOCK),
		.size = cpu_to_le32(2 * sizeof(__le32)),
		.mode = cpu_to_le32(mode),
		.rate = cpu_to_le32(rate),
	};
	const char *what = mode == AOP_AUDIO_PDM_CLOCK_STOP ?
			   "stop PDM clock" : "start PDM clock";

	if (audio->direct_clock) {
		struct aop_t8011_clock_config config = {
			.source = cpu_to_le32(rate ? AOP_T8011_CLOCK_XIO :
					      AOP_T8011_CLOCK_DISABLE),
			.rate = cpu_to_le32(rate),
		};

		return aop_audio_call(audio, &config, sizeof(config), what);
	}

	aop_audio_hdr(&call.hdr, AOP_AUDIO_CALL_SET_PROP, sizeof(call));
	return aop_audio_call(audio, &call, sizeof(call), what);
}

static struct aop_audio *to_aop_audio(struct clk_hw *hw)
{
	return container_of(hw, struct aop_audio, hw);
}

static int aop_audio_clk_prepare(struct clk_hw *hw)
{
	struct aop_audio *audio = to_aop_audio(hw);

	if (!audio->rate)
		return -EINVAL;
	return aop_audio_pdm_clock(audio, AOP_AUDIO_PDM_CLOCK_START, audio->rate);
}

static void aop_audio_clk_unprepare(struct clk_hw *hw)
{
	aop_audio_pdm_clock(to_aop_audio(hw), AOP_AUDIO_PDM_CLOCK_STOP, 0);
}

static unsigned long aop_audio_clk_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	return to_aop_audio(hw)->rate;
}

static int aop_audio_clk_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	req->rate = clamp_t(unsigned long, req->rate, 1, U32_MAX);
	return 0;
}

static int aop_audio_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	to_aop_audio(hw)->rate = rate;
	return 0;
}

static const struct clk_ops aop_audio_clk_ops = {
	.prepare = aop_audio_clk_prepare,
	.unprepare = aop_audio_clk_unprepare,
	.recalc_rate = aop_audio_clk_recalc_rate,
	.determine_rate = aop_audio_clk_determine_rate,
	.set_rate = aop_audio_clk_set_rate,
};

static int aop_audio_probe(struct platform_device *pdev)
{
	const struct apple_aop_service *service = dev_get_platdata(&pdev->dev);
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {
		.ops = &aop_audio_clk_ops,
		/* the rate goes to the firmware with the start word */
		.flags = CLK_SET_RATE_GATE,
	};
	struct aop_audio_attach attach = {
		.dev = cpu_to_le32(AOP_AUDIO_DEV_PDM0),
	};
	struct aop_audio *audio;
	int ret;

	if (!service)
		return -ENODEV;

	audio = devm_kzalloc(dev, sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return -ENOMEM;
	audio->dev = dev;
	audio->service = *service;
	/* The A10X firmware takes the clock as a direct command. */
	audio->direct_clock = of_device_is_compatible(dev->of_node,
						       "apple,t8011-aop-audio");

	if (!audio->direct_clock) {
		aop_audio_hdr(&attach.hdr, AOP_AUDIO_CALL_ATTACH, sizeof(attach));
		ret = aop_audio_call(audio, &attach, sizeof(attach), "attach pdm0");
		if (ret)
			return ret;
	}

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-pdm", dev_name(dev));
	if (!init.name)
		return -ENOMEM;
	audio->hw.init = &init;
	ret = devm_clk_hw_register(dev, &audio->hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the clock\n");

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &audio->hw);
}

static const struct of_device_id aop_audio_of_match[] = {
	{ .compatible = "apple,t8010-aop-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, aop_audio_of_match);

static struct platform_driver aop_audio_driver = {
	.driver = {
		.name = "apple-aop-audio",
		.of_match_table = aop_audio_of_match,
	},
	.probe = aop_audio_probe,
};
module_platform_driver(aop_audio_driver);

MODULE_DESCRIPTION("Apple AOP audio service PDM clock");
MODULE_LICENSE("Dual MIT/GPL");
