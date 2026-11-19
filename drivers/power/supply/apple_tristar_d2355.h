/* SPDX-License-Identifier: GPL-2.0-only */
/* D2355 hardware operations for the shared Tristar charging policy. */
#include "apple_d2355_current.h"

#define D2355_TRISTAR_STATUS_ONLINE	0x42

struct tristar_d2355 {
	struct d2355_calibration calibration;
	struct mutex io_lock;
	s32 pack_limits[6];
	unsigned int requested;
	u8 raw, control;
	bool verified, fault;
};

static int tristar_d2355_disable(struct tristar_charger *chg)
{
	unsigned int control, check;
	int ret;

	ret = regmap_read(chg->regmap, chg->control_base, &control);
	if (!ret)
		ret = regmap_write(chg->regmap, chg->control_base, control | BIT(2));
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base, &check);
	if (!ret && check != (control | BIT(2)))
		ret = -EIO;
	return ret;
}

static int tristar_d2355_read_limit(struct tristar_charger *chg, unsigned int *code)
{
	struct tristar_d2355 *hw = chg->chip_data;
	unsigned int before, raw, after, result;
	int ret;

	mutex_lock(&hw->io_lock);
	ret = regmap_read(chg->regmap, chg->control_base, &before);
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base + 0xb, &raw);
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base, &after);
	if (ret)
		goto out;
	if (before != after) {
		ret = -ESTALE;
		goto out;
	}
	if (after & BIT(2)) {
		result = 0;
	} else if (after & BIT(1)) {
		ret = -EOPNOTSUPP; /* Only verified low-voltage range zero. */
		goto out;
	} else if (hw->verified) {
		if (raw != hw->raw || after != hw->control) {
			ret = -ESTALE;
			goto out;
		}
		result = hw->requested;
	} else {
		/* Initial handoff setting only; never adopt a higher grant. */
		result = max(raw, 8U);
	}
	*code = result;
out:
	mutex_unlock(&hw->io_lock);
	return ret;
}

static unsigned int tristar_d2355_code_ma(struct tristar_charger *chg,
					unsigned int code)
{
	return code * 10;
}

static unsigned int tristar_d2355_ma_code(struct tristar_charger *chg,
					unsigned int ma)
{
	return ma < 80 ? 0 : min(ma / 10, 254U);
}

static int tristar_d2355_write_limit(struct tristar_charger *chg, unsigned int code)
{
	struct tristar_d2355 *hw = chg->chip_data;
	unsigned int control, check, epoch = atomic_read(&chg->epoch);
	unsigned int allowance;
	u8 raw = 8;
	bool disabled;
	int ret, recovery;

	if (code > 254)
		return -ERANGE;
	mutex_lock(&hw->io_lock);
	if (hw->fault && code) {
		ret = -EIO;
		goto out;
	}
	if (code > chg->chip->fallback_code) {
		allowance = max(chg->source_ma, tristar_usb_ma(chg));
		if (!chg->present || epoch != chg->epoch_seen ||
		    code * 10 > min(allowance, chg->max_ma)) {
			ret = -ESTALE;
			goto out;
		}
	}
	ret = regmap_read(chg->regmap, chg->control_base, &control);
	if (ret)
		goto out;
	if (code && (control & BIT(1))) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	if (code) {
		ret = d2355_current_encode(&hw->calibration, 0, code * 10,
					  &raw, &disabled);
		if (ret)
			goto out;
		if (disabled) {
			ret = -ERANGE;
			goto out;
		}
	}
	hw->verified = false;
	ret = tristar_d2355_disable(chg);
	if (ret) {
		hw->fault = true;
		goto out;
	}
	if (!code)
		goto verified;
	ret = regmap_write(chg->regmap, chg->control_base + 0xb, raw);
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base + 0xb, &check);
	if (!ret && check != raw)
		ret = -EIO;
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base, &check);
	if (!ret && check != (control | BIT(2)))
		ret = -ESTALE;
	if (!ret && code > chg->chip->fallback_code &&
	    epoch != atomic_read(&chg->epoch))
		ret = -ESTALE;
	if (!ret)
		ret = regmap_write(chg->regmap, chg->control_base, control & ~BIT(2));
	if (!ret)
		ret = regmap_read(chg->regmap, chg->control_base, &check);
	if (!ret && check != (control & ~BIT(2)))
		ret = -EIO;
	/* Publish before the final epoch check: an IRQ must see the grant and
	 * wait for io_lock, or invalidate this check and force recovery below.
	 */
	if (!ret) {
		WRITE_ONCE(chg->code, code);
		smp_mb(); /* Pairs with invalidation before IRQ withdrawal. */
	}
	if (!ret && code > chg->chip->fallback_code &&
	    epoch != atomic_read(&chg->epoch))
		ret = -ESTALE;
	if (ret) {
		recovery = tristar_d2355_disable(chg);
		if (recovery)
			hw->fault = true;
		else
			WRITE_ONCE(chg->code, 0);
		goto out;
	}
verified:
	hw->requested = code;
	hw->raw = raw;
	hw->control = code ? control & ~BIT(2) : control | BIT(2);
	hw->verified = true;
	WRITE_ONCE(chg->code, code);
out:
	mutex_unlock(&hw->io_lock);
	return ret;
}

static int tristar_d2355_read_status(struct tristar_charger *chg, unsigned int *status)
{
	u8 bytes[10];
	int ret = regmap_bulk_read(chg->regmap, chg->status_base, bytes, sizeof(bytes));

	/*
	 * Attachment starts source identification. Register 0x86 bit 5 is
	 * charger feedback and can be clear while the input limit is set.
	 */
	if (!ret)
		*status = bytes[1] & BIT(7) ? D2355_TRISTAR_STATUS_ONLINE : 0;
	return ret;
}

static bool tristar_d2355_pack_allows(struct tristar_charger *chg,
				     const struct tristar_pack *pack)
{
	struct tristar_d2355 *hw = chg->chip_data;

	return pack->temp >= hw->pack_limits[0] && pack->temp <= hw->pack_limits[1] &&
	       pack->voltage_uv >= hw->pack_limits[2] && pack->voltage_uv <= hw->pack_limits[3] &&
	       pack->current_ua >= hw->pack_limits[4] && pack->current_ua <= hw->pack_limits[5];
}

static int tristar_d2355_prepare(struct tristar_charger *chg)
{
	struct device *dev = chg->dev;
	struct tristar_d2355 *hw;
	struct device_node *np;
	struct i2c_client *adc;
	const struct firmware *firmware;
	const char *name;
	u32 limits[6];
	unsigned int i;
	int ret;

	if (chg->max_ma > 2540 || chg->max_ma < 80)
		return -EINVAL;
	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;
	ret = devm_mutex_init(dev, &hw->io_lock);
	if (ret)
		return ret;
	ret = device_property_read_u32_array(dev, "apple,pack-limits", limits, 6);
	if (ret)
		return ret;
	for (i = 0; i < 6; i++)
		hw->pack_limits[i] = (s32)limits[i];
	for (i = 0; i < 6; i += 2)
		if (hw->pack_limits[i] > hw->pack_limits[i + 1])
			return -EINVAL;
	if (hw->pack_limits[2] <= 0)
		return -EINVAL;
	ret = device_property_read_string(dev, "firmware-name", &name);
	if (ret)
		return ret;
	ret = request_firmware(&firmware, name, dev);
	if (ret)
		return ret;
	ret = d2355_calibration_parse_hv(firmware->data, firmware->size, &hw->calibration);
	release_firmware(firmware);
	if (ret)
		return ret;
	np = of_parse_phandle(dev->of_node, "apple,adc-pmic", 0);
	if (!np)
		return -EINVAL;
	adc = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!adc)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, tristar_put_client, adc);
	if (ret)
		return ret;
	chg->adc_regmap = dev_get_regmap(&adc->dev, NULL);
	if (!chg->adc_regmap)
		return -EPROBE_DEFER;
	if (!device_link_add(dev, &adc->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	chg->chip_data = hw;
	return 0;
}

static const struct tristar_chip_info tristar_d2355_chip = {
	.prepare = tristar_d2355_prepare,
	.pack_allows = tristar_d2355_pack_allows,
	.status_size = 10,
	.control_size = 12,
	.adc_error_mask = 0xc0,
	.strict_limit_errors = true,
	.require_pack_telemetry = true,
	.invalidate_on_line = true,
	.read_limit = tristar_d2355_read_limit,
	.write_limit = tristar_d2355_write_limit,
	.code_ma = tristar_d2355_code_ma,
	.ma_code = tristar_d2355_ma_code,
	.read_status = tristar_d2355_read_status,
	.handoff_code = 0,
	.attach_code = 8,
	.online_status = D2355_TRISTAR_STATUS_ONLINE,
	.ramp_step_ms = 500,
	.fallback_code = 8, /* Preserve measured J120 80 mA baseline until identified. */
};
