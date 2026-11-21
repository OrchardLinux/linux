// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only battery and USB charger monitor for the Dialog D2186 PMIC on
 * Apple A8 devices without a dedicated fuel-gauge IC.
 *
 * Reads telemetry from the D2186 housekeeping ADC via IIO and reads charger
 * status from PMIC registers 0x60 and 0x62. Never writes to charger current
 * limits or any charging control register.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/consumer.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* D2186 status block registers (0x60..0x6d), read-only */
#define D2186_STATUS_A			0x60
#define D2186_STATUS_A_USB_PRESENT	BIT(0)
#define D2186_STATUS_A_TEMP_FAULT	BIT(7)

#define D2186_STATUS_C			0x62
#define D2186_STATUS_C_CHG_STATE_MASK	GENMASK(1, 0)
#define D2186_STATUS_C_CHG_END		BIT(2)
#define D2186_STATUS_C_CHG_TIMEOUT	BIT(4)

#define D2186_DEFAULT_CHARGE_FULL_UAH	1080000
#define D2186_DEFAULT_VMIN_UV		3400000
#define D2186_DEFAULT_VMAX_UV		4350000

/*
 * Approximate 4.35 V Li-ion OCV table anchored at 3400 mV (0%) when no
 * simple-battery OCV table is provided in Device Tree.
 */
static const struct power_supply_battery_ocv_table d2186_fallback_ocv[] = {
	{ 4350000, 100 },
	{ 4220000,  90 },
	{ 4100000,  80 },
	{ 3990000,  70 },
	{ 3900000,  60 },
	{ 3840000,  50 },
	{ 3800000,  40 },
	{ 3770000,  30 },
	{ 3740000,  20 },
	{ 3680000,  10 },
	{ 3580000,   5 },
	{ 3400000,   0 },
};

struct apple_d2186_battery {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *bat_psy;
	struct power_supply *chg_psy;
	struct power_supply_battery_info *binfo;
	struct iio_channel *vbat;
	struct iio_channel *ich;
	struct iio_channel *tbat;
	struct iio_channel *vbus;
	struct iio_channel *ibus;
};

static int apple_d2186_read_status_regs(struct apple_d2186_battery *bat,
					unsigned int *status_a,
					unsigned int *status_c)
{
	int ret;

	ret = regmap_read(bat->regmap, D2186_STATUS_A, status_a);
	if (ret)
		return ret;

	return regmap_read(bat->regmap, D2186_STATUS_C, status_c);
}

static int apple_d2186_decode_status(unsigned int status_a,
				     unsigned int status_c, bool is_battery)
{
	if (!(status_a & D2186_STATUS_A_USB_PRESENT))
		return is_battery ? POWER_SUPPLY_STATUS_DISCHARGING :
				    POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (status_c & D2186_STATUS_C_CHG_END)
		return POWER_SUPPLY_STATUS_FULL;

	if (status_c & D2186_STATUS_C_CHG_TIMEOUT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (status_c & D2186_STATUS_C_CHG_STATE_MASK)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int apple_d2186_read_temp_tenth_c(struct apple_d2186_battery *bat,
					 int *tenth_c)
{
	int mdegc, ret;

	if (!bat->tbat)
		return -ENODATA;

	ret = iio_read_channel_processed(bat->tbat, &mdegc);
	if (ret)
		return ret;

	*tenth_c = DIV_ROUND_CLOSEST(mdegc, 100);
	return 0;
}

static int apple_d2186_bat_capacity(struct apple_d2186_battery *bat,
				    int *capacity)
{
	int vbat_uv, tenth_c, temp_c = 20, cap, ret;

	ret = iio_read_channel_processed_scale(bat->vbat, &vbat_uv, 1000);
	if (ret)
		return ret;

	if (!apple_d2186_read_temp_tenth_c(bat, &tenth_c))
		temp_c = tenth_c / 10;

	if (bat->binfo)
		cap = power_supply_batinfo_ocv2cap(bat->binfo, vbat_uv, temp_c);
	else
		cap = -EINVAL;

	if (cap < 0)
		cap = power_supply_ocv2cap_simple(d2186_fallback_ocv,
						  ARRAY_SIZE(d2186_fallback_ocv),
						  vbat_uv);

	*capacity = clamp(cap, 0, 100);
	return 0;
}

static int apple_d2186_bat_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct apple_d2186_battery *bat = power_supply_get_drvdata(psy);
	unsigned int status_a, status_c;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		ret = apple_d2186_read_status_regs(bat, &status_a, &status_c);
		if (ret)
			return ret;
		val->intval = apple_d2186_decode_status(status_a, status_c,
							true);
		return 0;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(bat->regmap, D2186_STATUS_A, &status_a);
		if (ret)
			return ret;
		if (status_a & D2186_STATUS_A_TEMP_FAULT) {
			int tenth_c;

			if (!apple_d2186_read_temp_tenth_c(bat, &tenth_c) &&
			    tenth_c < 0)
				val->intval = POWER_SUPPLY_HEALTH_COLD;
			else
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return iio_read_channel_processed_scale(bat->vbat,
							&val->intval, 1000);

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/*
		 * D2186 ich measures PMIC charge current into the battery
		 * (positive while charging, 0 when discharging; there is no
		 * coulomb counter or discharge current sense on this board).
		 */
		if (!bat->ich)
			return -ENODATA;
		return iio_read_channel_processed_scale(bat->ich,
							&val->intval, 1000);

	case POWER_SUPPLY_PROP_TEMP:
		return apple_d2186_read_temp_tenth_c(bat, &val->intval);

	case POWER_SUPPLY_PROP_CAPACITY:
		return apple_d2186_bat_capacity(bat, &val->intval);

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (bat->binfo && bat->binfo->charge_full_design_uah > 0)
			val->intval = bat->binfo->charge_full_design_uah;
		else
			val->intval = D2186_DEFAULT_CHARGE_FULL_UAH;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (bat->binfo && bat->binfo->voltage_min_design_uv > 0)
			val->intval = bat->binfo->voltage_min_design_uv;
		else
			val->intval = D2186_DEFAULT_VMIN_UV;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (bat->binfo && bat->binfo->voltage_max_design_uv > 0)
			val->intval = bat->binfo->voltage_max_design_uv;
		else
			val->intval = D2186_DEFAULT_VMAX_UV;
		return 0;

	default:
		return -EINVAL;
	}
}

static int apple_d2186_chg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct apple_d2186_battery *bat = power_supply_get_drvdata(psy);
	unsigned int status_a, status_c;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(bat->regmap, D2186_STATUS_A, &status_a);
		if (ret)
			return ret;
		val->intval = !!(status_a & D2186_STATUS_A_USB_PRESENT);
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		ret = apple_d2186_read_status_regs(bat, &status_a, &status_c);
		if (ret)
			return ret;
		val->intval = apple_d2186_decode_status(status_a, status_c,
							false);
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!bat->vbus)
			return -ENODATA;
		return iio_read_channel_processed_scale(bat->vbus,
							&val->intval, 1000);

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!bat->ibus)
			return -ENODATA;
		ret = regmap_read(bat->regmap, D2186_STATUS_A, &status_a);
		if (ret)
			return ret;
		if (!(status_a & D2186_STATUS_A_USB_PRESENT)) {
			val->intval = 0;
			return 0;
		}
		return iio_read_channel_processed_scale(bat->ibus,
							&val->intval, 1000);

	default:
		return -EINVAL;
	}
}

static const enum power_supply_property apple_d2186_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static const struct power_supply_desc apple_d2186_bat_desc = {
	.name = "apple-d2186-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = apple_d2186_bat_props,
	.num_properties = ARRAY_SIZE(apple_d2186_bat_props),
	.get_property = apple_d2186_bat_get_property,
};

static const enum power_supply_property apple_d2186_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc apple_d2186_chg_desc = {
	.name = "apple-d2186-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = apple_d2186_chg_props,
	.num_properties = ARRAY_SIZE(apple_d2186_chg_props),
	.get_property = apple_d2186_chg_get_property,
};

static struct iio_channel *apple_d2186_opt_chan(struct device *dev,
						const char *name)
{
	struct iio_channel *chan;

	chan = devm_iio_channel_get(dev, name);
	if (IS_ERR(chan) && PTR_ERR(chan) == -ENODEV)
		return NULL;

	return chan;
}

static void apple_d2186_put_bat_info(void *data)
{
	struct apple_d2186_battery *bat = data;

	power_supply_put_battery_info(bat->bat_psy, bat->binfo);
}

static int apple_d2186_battery_probe(struct platform_device *pdev)
{
	struct power_supply_config bat_cfg = {};
	struct power_supply_config chg_cfg = {};
	struct device *dev = &pdev->dev;
	struct apple_d2186_battery *bat;
	struct regmap *regmap;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV,
				     "parent PMIC regmap unavailable\n");

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = dev;
	bat->regmap = regmap;

	bat->vbat = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(bat->vbat))
		return dev_err_probe(dev, PTR_ERR(bat->vbat),
				     "failed to get vbat ADC channel\n");

	bat->ich = apple_d2186_opt_chan(dev, "ich");
	if (IS_ERR(bat->ich))
		return dev_err_probe(dev, PTR_ERR(bat->ich),
				     "failed to get ich ADC channel\n");

	bat->tbat = apple_d2186_opt_chan(dev, "tbat");
	if (IS_ERR(bat->tbat))
		return dev_err_probe(dev, PTR_ERR(bat->tbat),
				     "failed to get tbat ADC channel\n");

	bat->vbus = apple_d2186_opt_chan(dev, "vbus");
	if (IS_ERR(bat->vbus))
		return dev_err_probe(dev, PTR_ERR(bat->vbus),
				     "failed to get vbus ADC channel\n");

	bat->ibus = apple_d2186_opt_chan(dev, "ibus");
	if (IS_ERR(bat->ibus))
		return dev_err_probe(dev, PTR_ERR(bat->ibus),
				     "failed to get ibus ADC channel\n");

	bat_cfg.drv_data = bat;
	bat_cfg.fwnode = dev_fwnode(dev);
	bat_cfg.no_wakeup_source = true;

	bat->bat_psy = devm_power_supply_register(dev, &apple_d2186_bat_desc,
						  &bat_cfg);
	if (IS_ERR(bat->bat_psy))
		return dev_err_probe(dev, PTR_ERR(bat->bat_psy),
				     "failed to register battery power supply\n");

	ret = power_supply_get_battery_info(bat->bat_psy, &bat->binfo);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "failed to read battery info\n");
	if (!ret) {
		ret = devm_add_action_or_reset(dev, apple_d2186_put_bat_info,
					       bat);
		if (ret)
			return ret;
	}

	chg_cfg.drv_data = bat;
	chg_cfg.no_wakeup_source = true;

	bat->chg_psy = devm_power_supply_register(dev, &apple_d2186_chg_desc,
						  &chg_cfg);
	if (IS_ERR(bat->chg_psy))
		return dev_err_probe(dev, PTR_ERR(bat->chg_psy),
				     "failed to register USB power supply\n");

	return 0;
}

static const struct of_device_id apple_d2186_battery_of_match[] = {
	{ .compatible = "apple,d2186-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_d2186_battery_of_match);

static struct platform_driver apple_d2186_battery_driver = {
	.probe = apple_d2186_battery_probe,
	.driver = {
		.name = "apple-d2186-battery",
		.of_match_table = apple_d2186_battery_of_match,
	},
};
module_platform_driver(apple_d2186_battery_driver);

MODULE_IMPORT_NS("IIO_CONSUMER");
MODULE_DESCRIPTION("Apple D2186 PMIC read-only battery and charger status driver");
MODULE_LICENSE("GPL");
