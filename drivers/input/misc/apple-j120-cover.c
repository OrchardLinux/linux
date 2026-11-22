// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define D2400_GPIO_INPUT_BASE	0x186
#define J120_COVER_POLL_MS	200

struct j120_cover {
	struct regmap *regmap;
	int last_sample;
};

static int j120_cover_open(struct input_dev *input)
{
	struct j120_cover *cover = input_get_drvdata(input);

	/* The first poll after reopening must start a new debounce window. */
	cover->last_sample = -1;
	return 0;
}

static void j120_cover_poll(struct input_dev *input)
{
	struct j120_cover *cover = input_get_drvdata(input);
	bool near_hall, far_hall;
	u8 levels[2];
	int ret;

	/* Read GPIO 0 and GPIO 10 without touching the PMIC pin settings. */
	ret = regmap_bulk_read(cover->regmap, D2400_GPIO_INPUT_BASE, levels,
			       ARRAY_SIZE(levels));
	if (ret) {
		cover->last_sample = -1;
		return;
	}

	near_hall = !(levels[0] & BIT(0));
	far_hall = !(levels[1] & BIT(2));
	/* Keep the previous switch state until both inputs agree twice. */
	if (near_hall != far_hall) {
		cover->last_sample = -1;
		return;
	}

	if (cover->last_sample == near_hall) {
		input_report_switch(input, SW_MACHINE_COVER, near_hall);
		input_sync(input);
	}
	cover->last_sample = near_hall;
}

static int j120_cover_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	struct j120_cover *cover;
	int ret;

	cover = devm_kzalloc(dev, sizeof(*cover), GFP_KERNEL);
	if (!cover)
		return -ENOMEM;

	cover->regmap = dev_get_regmap(dev->parent, NULL);
	if (!cover->regmap)
		return dev_err_probe(dev, -ENODEV, "August PMIC register map unavailable\n");
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "Apple J120 Magnetic Cover";
	input->id.bustype = BUS_I2C;
	input->open = j120_cover_open;
	input_set_drvdata(input, cover);
	input_set_capability(input, EV_SW, SW_MACHINE_COVER);

	ret = input_setup_polling(input, j120_cover_poll);
	if (ret)
		return ret;
	input_set_poll_interval(input, J120_COVER_POLL_MS);
	input_set_min_poll_interval(input, J120_COVER_POLL_MS);
	input_set_max_poll_interval(input, J120_COVER_POLL_MS);

	return input_register_device(input);
}

static const struct of_device_id j120_cover_of_match[] = {
	{ .compatible = "apple,j120-cover-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, j120_cover_of_match);

static struct platform_driver j120_cover_driver = {
	.probe = j120_cover_probe,
	.driver = {
		.name = "apple-j120-cover",
		.of_match_table = j120_cover_of_match,
	},
};
module_platform_driver(j120_cover_driver);

MODULE_DESCRIPTION("Apple J120 magnetic cover switch");
MODULE_LICENSE("GPL");
