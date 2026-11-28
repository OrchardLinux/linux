// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC power switches
 *
 * On the A11 and A12 iPhones the SMC switches some peripheral supplies on
 * and off through write-only keys. A plain key takes 1 for on and 0 for off.
 * A function key (the ADT's pKW4 functions) takes the function's code from
 * the ADT with the state in its lowest bit; any other value, a plain 0
 * included, is an invalid code. A read of these keys fails, so the driver
 * keeps the state it last wrote.
 *
 * Which keys exist is a property of the SMC firmware, so the table lives
 * here, selected by compatible, instead of in the device tree.
 */

#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

struct macsmc_rail {
	const char *name;
	smc_key key;
	u32 on;
	u32 off;
};

struct macsmc_regulator {
	struct apple_smc *smc;
	const struct macsmc_rail *rail;
	bool enabled;
};

static int macsmc_regulator_write(struct regulator_dev *rdev, bool on)
{
	struct macsmc_regulator *reg = rdev_get_drvdata(rdev);
	u32 val = on ? reg->rail->on : reg->rail->off;
	int ret;

	ret = apple_smc_write(reg->smc, reg->rail->key, &val, sizeof(val));
	if (ret)
		return ret;

	reg->enabled = on;
	return 0;
}

static int macsmc_regulator_enable(struct regulator_dev *rdev)
{
	return macsmc_regulator_write(rdev, true);
}

static int macsmc_regulator_disable(struct regulator_dev *rdev)
{
	return macsmc_regulator_write(rdev, false);
}

static int macsmc_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct macsmc_regulator *reg = rdev_get_drvdata(rdev);

	return reg->enabled;
}

static const struct regulator_ops macsmc_regulator_ops = {
	.enable = macsmc_regulator_enable,
	.disable = macsmc_regulator_disable,
	.is_enabled = macsmc_regulator_is_enabled,
};

/* iPhone 8 / 8 Plus: the Touch ID sensor's supplies */
static const struct macsmc_rail macsmc_t8015_rails[] = {
	{ .name = "mesa-1v8", .key = SMC_KEY(rLDc), .on = 1 },
	{ .name = "mesa-3v0", .key = SMC_KEY(rLDa), .on = 1 },
	{ }
};

/*
 * iPhone XR: the touch controller's core supply and the enable of its clock,
 * the ADT multi-touch node's function-power_ldo = pKW4('rBK3', 0x5020000)
 * and function-clock_enable = pKW4('gP00', 0x8000000).
 */
static const struct macsmc_rail macsmc_t8020_rails[] = {
	{ .name = "touch-core", .key = SMC_KEY(rBK3),
	  .on = 0x05020001, .off = 0x05020000 },
	{ .name = "touch-clock", .key = SMC_KEY(gP00),
	  .on = 0x08000001, .off = 0x08000000 },
	{ }
};

static int macsmc_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_smc *smc = dev_get_drvdata(dev->parent);
	const struct macsmc_rail *rail = of_device_get_match_data(dev);

	if (!smc || !rail)
		return -ENODEV;

	for (; rail->name; rail++) {
		struct regulator_config config = { .dev = dev };
		struct regulator_desc *desc;
		struct macsmc_regulator *reg;
		struct regulator_dev *rdev;

		if (!apple_smc_key_exists(smc, rail->key)) {
			dev_warn(dev, "%s: SMC key %p4ch missing, skipped\n",
				 rail->name, &rail->key);
			continue;
		}

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		reg = devm_kzalloc(dev, sizeof(*reg), GFP_KERNEL);
		if (!desc || !reg)
			return -ENOMEM;

		reg->smc = smc;
		reg->rail = rail;

		desc->name = rail->name;
		desc->of_match = rail->name;
		desc->type = REGULATOR_VOLTAGE;
		desc->owner = THIS_MODULE;
		desc->ops = &macsmc_regulator_ops;

		config.driver_data = reg;
		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "%s: register failed\n", rail->name);
	}

	return 0;
}

static const struct of_device_id macsmc_regulator_of_match[] = {
	{ .compatible = "apple,t8015-smc-regulator", .data = macsmc_t8015_rails },
	{ .compatible = "apple,t8020-smc-regulator", .data = macsmc_t8020_rails },
	{ }
};
MODULE_DEVICE_TABLE(of, macsmc_regulator_of_match);

static struct platform_driver macsmc_regulator_driver = {
	.driver = {
		.name = "macsmc-regulator",
		.of_match_table = macsmc_regulator_of_match,
	},
	.probe = macsmc_regulator_probe,
};
module_platform_driver(macsmc_regulator_driver);

MODULE_DESCRIPTION("Apple SMC power switches");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_ALIAS("platform:macsmc-regulator");
