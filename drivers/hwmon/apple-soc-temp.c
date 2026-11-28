// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SoC die temperature sensors
 *
 * Copyright (c) 2026 kiki <kieran@lavato.net>
 *
 * The ADT calls these tempsensor0 to tempsensor8 on T8020 and gives every
 * register offset by name. Two kinds, which is why there are two compatibles:
 * the PMGR die sensors are 32 bit and evenly spaced, the CPU cluster ones are
 * 64 bit and are not.
 *
 * Each sensor can also accumulate readings into a sum and a count, but iBoot
 * leaves those at zero, so this reads readBK, a single reading.
 */

#include <linux/bitfield.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/*
 * A read holds two 16 bit fields. The low one is the temperature in 1/64 of a
 * degree, the high one is the raw diode code, which falls as the low one
 * rises. Only the low one is of any use here.
 */
#define TEMP_READ_VALUE		GENMASK(15, 0)
#define TEMP_READ_SHIFT		6

struct apple_temp_sensor {
	const char *label;
	u32 read_bk;
};

struct apple_temp_soc {
	const struct apple_temp_sensor *sensors;
	unsigned int count;
	bool wide;
};

/* /arm-io/tempsensor0 to 4. One block, 0x4000 per sensor. */
static const struct apple_temp_sensor t8020_pmgr_sensors[] = {
	{ "SOC die 0", 0x20000 },
	{ "SOC die 1", 0x24000 },
	{ "SOC die 2", 0x28000 },
	{ "SOC die 3", 0x2c000 },
	{ "SOC die 4", 0x30000 },
};

/* /arm-io/tempsensor5 to 7, the P cluster and its two cores. */
static const struct apple_temp_sensor t8020_acc_sensors[] = {
	{ "ACC",   0x4000 },
	{ "ACCp0", 0x4010 },
	{ "ACCp1", 0x4020 },
};

/* /arm-io/tempsensor8, the E cluster, same offsets at its own base. */
static const struct apple_temp_sensor t8020_acce_sensors[] = {
	{ "ACCe", 0x4000 },
};

static const struct apple_temp_soc t8020_pmgr = {
	.sensors = t8020_pmgr_sensors,
	.count = ARRAY_SIZE(t8020_pmgr_sensors),
};

static const struct apple_temp_soc t8020_acc = {
	.sensors = t8020_acc_sensors,
	.count = ARRAY_SIZE(t8020_acc_sensors),
	.wide = true,
};

static const struct apple_temp_soc t8020_acce = {
	.sensors = t8020_acce_sensors,
	.count = ARRAY_SIZE(t8020_acce_sensors),
	.wide = true,
};

struct apple_temp {
	void __iomem *base;
	const struct apple_temp_soc *soc;
};

static u64 apple_temp_raw(const struct apple_temp *temp, int channel)
{
	u32 offset = temp->soc->sensors[channel].read_bk;

	if (temp->soc->wide)
		return readq_relaxed(temp->base + offset);

	return readl_relaxed(temp->base + offset);
}

static int apple_temp_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
				 u32 attr, int channel, long *val)
{
	struct apple_temp *temp = dev_get_drvdata(dev);

	if (type != hwmon_temp || attr != hwmon_temp_input)
		return -EOPNOTSUPP;

	/* millidegrees, from a reading in 1/64 of a degree */
	*val = (FIELD_GET(TEMP_READ_VALUE, apple_temp_raw(temp, channel)) * 1000)
		>> TEMP_READ_SHIFT;

	return 0;
}

static int apple_temp_hwmon_read_string(struct device *dev, enum hwmon_sensor_types type,
					u32 attr, int channel, const char **str)
{
	struct apple_temp *temp = dev_get_drvdata(dev);

	if (type != hwmon_temp || attr != hwmon_temp_label)
		return -EOPNOTSUPP;

	*str = temp->soc->sensors[channel].label;

	return 0;
}

/*
 * Not every sensor in a block is running. On D331p, PMGR sensor 2 reads zero
 * while the other four read sensible temperatures, so hide whatever reads
 * nothing rather than publish a channel stuck at 0 degrees.
 */
static umode_t apple_temp_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
					   u32 attr, int channel)
{
	const struct apple_temp *temp = data;

	if (type != hwmon_temp || channel >= temp->soc->count ||
	    !apple_temp_raw(temp, channel))
		return 0;

	return 0444;
}

static const struct hwmon_ops apple_temp_hwmon_ops = {
	.is_visible = apple_temp_hwmon_is_visible,
	.read = apple_temp_hwmon_read,
	.read_string = apple_temp_hwmon_read_string,
};

static const struct hwmon_channel_info * const apple_temp_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info apple_temp_chip = {
	.ops = &apple_temp_hwmon_ops,
	.info = apple_temp_info,
};

static int apple_temp_probe(struct platform_device *pdev)
{
	struct apple_temp *temp;
	struct device *hwmon;

	temp = devm_kzalloc(&pdev->dev, sizeof(*temp), GFP_KERNEL);
	if (!temp)
		return -ENOMEM;

	temp->soc = of_device_get_match_data(&pdev->dev);
	if (!temp->soc)
		return -ENODEV;

	temp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(temp->base))
		return PTR_ERR(temp->base);

	hwmon = devm_hwmon_device_register_with_info(&pdev->dev, "apple_soc_temp",
						     temp, &apple_temp_chip, NULL);

	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id apple_temp_of_table[] = {
	{ .compatible = "apple,t8020-pmgr-tempsensor", .data = &t8020_pmgr },
	{ .compatible = "apple,t8020-acc-tempsensor", .data = &t8020_acc },
	{ .compatible = "apple,t8020-acce-tempsensor", .data = &t8020_acce },
	{}
};
MODULE_DEVICE_TABLE(of, apple_temp_of_table);

static struct platform_driver apple_temp_driver = {
	.probe = apple_temp_probe,
	.driver = {
		.name = "apple-soc-temp",
		.of_match_table = apple_temp_of_table,
	},
};
module_platform_driver(apple_temp_driver);

MODULE_DESCRIPTION("Apple SoC die temperature sensors");
MODULE_LICENSE("GPL");
