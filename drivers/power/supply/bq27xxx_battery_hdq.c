// SPDX-License-Identifier: GPL-2.0
/*
 * BQ27xxx battery monitor HDQ/1-wire driver
 *
 * Copyright (C) 2007-2017 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/power/bq27xxx_battery.h>

#include <linux/w1.h>

#define W1_FAMILY_BQ27000	0x01

#define HDQ_CMD_READ	(0 << 7)
#define HDQ_CMD_WRITE	(1 << 7)

static int F_ID;
module_param(F_ID, int, S_IRUSR);
MODULE_PARM_DESC(F_ID, "1-wire slave FID for BQ27xxx device");

static const enum power_supply_property bq27545_iphone7_plus_cache_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/* RemainingCapacity answers, NominalAvailableCapacity does not. */
static const enum power_supply_property bq27540_j120_cache_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

struct bq27xxx_hdq_data {
	enum bq27xxx_chip chip;
	const enum power_supply_property *cache_properties;
	size_t num_cache_properties;
	bool use_remaining_capacity;
	/* Cached telemetry, refreshed periodically (always for the BQ27545) */
	bool cache_only;
	unsigned int read_budget_ms;
};

static int w1_bq27000_read(struct w1_slave *sl, unsigned int reg)
{
	u8 val;
	int ret;

	mutex_lock(&sl->master->bus_mutex);
	w1_write_8(sl->master, HDQ_CMD_READ | reg);
	ret = w1_read_block(sl->master, &val, 1);
	mutex_unlock(&sl->master->bus_mutex);

	return ret == 1 ? val : -EIO;
}

static int bq27xxx_battery_hdq_read(struct bq27xxx_device_info *di, u8 reg,
				    bool single)
{
	struct w1_slave *sl = dev_to_w1_slave(di->dev);
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/*
		 * Make sure the value has not changed in between reading the
		 * lower and the upper part
		 */
		upper = w1_bq27000_read(sl, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = w1_bq27000_read(sl, reg);
			if (lower < 0)
				return lower;

			upper = w1_bq27000_read(sl, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return w1_bq27000_read(sl, reg);
}

static int bq27xxx_battery_hdq_add_slave(struct w1_slave *sl)
{
	const struct of_device_id *match;
	struct bq27xxx_device_info *di;
	int ret;

	di = devm_kzalloc(&sl->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &sl->dev;
	match = of_match_node(sl->family->of_match_table, sl->dev.of_node);
	if (sl->master->dev.parent && !match)
		return -ENODEV;
	if (match) {
		const struct bq27xxx_hdq_data *data = match->data;

		di->chip = data->chip;
		di->name = "battery";
		di->cache_only = di->chip == BQ27545 || data->cache_only;
		di->cache_refresh_ms = di->cache_only ? 30000 : 0;
		if (data->cache_properties) {
			di->cache_properties = data->cache_properties;
			di->num_cache_properties = data->num_cache_properties;
		}
		di->cache_use_remaining_capacity = data->use_remaining_capacity;
		di->cache_read_budget_ms = data->read_budget_ms;
	} else {
		di->chip = BQ27000;
		di->name = "bq27000-battery";
	}
	di->bus.read = bq27xxx_battery_hdq_read;

	ret = bq27xxx_battery_setup(di);
	if (ret)
		return ret;

	/* The W1 core can retain the slave after a failed add callback. */
	dev_set_drvdata(&sl->dev, di);

	return 0;
}

static void bq27xxx_battery_hdq_remove_slave(struct w1_slave *sl)
{
	struct bq27xxx_device_info *di = dev_get_drvdata(&sl->dev);

	if (di)
		bq27xxx_battery_teardown(di);
}

static const struct w1_family_ops bq27xxx_battery_hdq_fops = {
	.add_slave	= bq27xxx_battery_hdq_add_slave,
	.remove_slave	= bq27xxx_battery_hdq_remove_slave,
};

/*
 * Apple's gauge firmware on these boards reports a charge-remaining value
 * that only pairs with FullChargeCapacity when it is read from
 * RemainingCapacity, not NominalAvailableCapacity.
 */
static const struct bq27xxx_hdq_data bq27545_iphone7_plus_data = {
	.chip = BQ27545,
	.cache_properties = bq27545_iphone7_plus_cache_props,
	.num_cache_properties = ARRAY_SIZE(bq27545_iphone7_plus_cache_props),
	.use_remaining_capacity = true,
};

static const struct bq27xxx_hdq_data bq27545_ipad_air_data = {
	.chip = BQ27545,
	.use_remaining_capacity = true,
};

static const struct bq27xxx_hdq_data bq27545_data = {
	.chip = BQ27545,
};

static const struct bq27xxx_hdq_data bq27541_data = {
	.chip = BQ27541,
};

/*
 * The iPad Pro 12.9 (2017)'s BQ27540 (ADT gas-gauge): the BQ27541 register
 * layout, RemainingCapacity instead of NominalAvailableCapacity, cached.
 */
static const struct bq27xxx_hdq_data bq27540_j120_data = {
	.chip = BQ27541,
	.cache_only = true,
	.read_budget_ms = 5000,
	.cache_properties = bq27540_j120_cache_props,
	.num_cache_properties = ARRAY_SIZE(bq27540_j120_cache_props),
	.use_remaining_capacity = true,
};

static const struct of_device_id bq27xxx_battery_hdq_of_match[] = {
	{ .compatible = "apple,j120-bq27540", .data = &bq27540_j120_data },
	{ .compatible = "apple,d11-bq27545", .data = &bq27545_iphone7_plus_data },
	{ .compatible = "apple,d111-bq27545", .data = &bq27545_iphone7_plus_data },
	{ .compatible = "apple,j72-bq27545", .data = &bq27545_ipad_air_data },
	{ .compatible = "ti,bq27545-hdq", .data = &bq27545_data },
	{ .compatible = "ti,bq27540-hdq", .data = &bq27541_data },
	{ }
};
MODULE_DEVICE_TABLE(of, bq27xxx_battery_hdq_of_match);

static struct w1_family bq27xxx_battery_hdq_family = {
	.fid = W1_FAMILY_BQ27000,
	.fops = &bq27xxx_battery_hdq_fops,
	.of_match_table = bq27xxx_battery_hdq_of_match,
};

static int __init bq27xxx_battery_hdq_init(void)
{
	if (F_ID)
		bq27xxx_battery_hdq_family.fid = F_ID;

	return w1_register_family(&bq27xxx_battery_hdq_family);
}
module_init(bq27xxx_battery_hdq_init);

static void __exit bq27xxx_battery_hdq_exit(void)
{
	w1_unregister_family(&bq27xxx_battery_hdq_family);
}
module_exit(bq27xxx_battery_hdq_exit);

MODULE_AUTHOR("Texas Instruments Ltd");
MODULE_DESCRIPTION("BQ27xxx battery monitor HDQ/1-wire driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_FAMILY_BQ27000));
