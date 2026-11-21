// SPDX-License-Identifier: GPL-2.0-only
/*
 * Housekeeping ADC driver for the Dialog D2186 PMIC used on Apple A8 devices.
 *
 * Conversions are serialized and polled via BIT(7) of register 0x500 so the
 * PMIC event interrupt block (0x50..0x5c / 0x70..0x7c) remains untouched.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

/* On-die thermal sensor trim offsets for channels 0x12..0x19 */
#define D2186_DIE_TEMP_TRIM_BASE	0x4a0
#define D2186_DIE_TEMP_CHAN_BASE	0x12
#define D2186_IBUS_DIE_TEMP_CHAN	0x14

/* Housekeeping ADC control and result registers */
#define D2186_ADC_CTRL			0x500
#define D2186_ADC_CTRL_START		BIT(7)
#define D2186_ADC_CTRL_CHAN_MASK	GENMASK(4, 0)

#define D2186_ADC_CFG			0x501
#define D2186_ADC_CFG_CLEAR_MASK	GENMASK(7, 1) /* preserve BIT(0) accumulator bit */

#define D2186_ADC_RESULT_LO		0x502
#define D2186_ADC_RESULT_INVALID	BIT(7)
#define D2186_ADC_RESULT_LO_MASK	GENMASK(3, 0)

/* Factory two-point ibus calibration registers (16-bit LE low, high) */
#define D2186_IBUS_CAL_BASE		0x51d
#define D2186_IBUS_CAL_DELTA_Q16	5243
#define D2186_IBUS_OFFSET_Q16		0x1999 /* ~0.100 A in Q16.16 */

#define D2186_ADC_POLL_US		1000
#define D2186_ADC_TIMEOUT_US		50000

/* Hardware ADC multiplexer channel numbers */
#define D2186_ADC_CHAN_VCC_MAIN		0
#define D2186_ADC_CHAN_ICH		1
#define D2186_ADC_CHAN_TBAT		2
#define D2186_ADC_CHAN_VBAT		4
#define D2186_ADC_CHAN_IBUS		9
#define D2186_ADC_CHAN_VBUS		15

struct d2186_ntc_point {
	u32 r_ohm;
	int mdegc;
};

/*
 * 10 kOhm NTC thermistor lookup table (-40..125 C in 5 C steps, R at 25 C =
 * 10000 Ohm) matching the D2186 PMIC battery temperature conversion.
 */
static const struct d2186_ntc_point d2186_ntc_lut[] = {
	{ 188500, -40000 }, { 144290, -35000 }, { 111330, -30000 },
	{  86560, -25000 }, {  67790, -20000 }, {  53460, -15000 },
	{  42450, -10000 }, {  33930,  -5000 }, {  27280,      0 },
	{  22070,   5000 }, {  17960,  10000 }, {  14700,  15000 },
	{  12090,  20000 }, {  10000,  25000 }, {   8310,  30000 },
	{   6940,  35000 }, {   5830,  40000 }, {   4910,  45000 },
	{   4160,  50000 }, {   3540,  55000 }, {   3020,  60000 },
	{   2590,  65000 }, {   2230,  70000 }, {   1920,  75000 },
	{   1670,  80000 }, {   1450,  85000 }, {   1270,  90000 },
	{   1110,  95000 }, {    980, 100000 }, {    860, 105000 },
	{    760, 110000 }, {    670, 115000 }, {    600, 120000 },
	{    530, 125000 },
};

/*
 * Q16.16 VBUS (4.50..5.50 V) and die temperature (-40..85 C) axes and
 * relative Ron correction matrix (Q16.16) for USB input current (ibus).
 */
static const s64 d2186_ibus_vbus_axis[9] = {
	294912, 303104, 311296, 316211, 327680, 335872, 344064, 352256, 360448,
};

static const s64 d2186_ibus_temp_axis[6] = {
	-2621440, -1638400, 0, 1703936, 3276800, 5570560,
};

static const s64 d2186_ibus_ron_q16[6][9] = {
	{ -14811, -15597, -15925, -15990, -16515, -16908, -17235, -17498, -17956 },
	{ -11927, -12451, -12910, -13041, -13631, -13959, -14221, -14286, -14680 },
	{  -5505,  -6029,  -6553,  -6815,  -7405,  -7733,  -8126,  -8454,  -8847 },
	{   2031,   1507,    983,    458,      0,   -524,   -917,  -1310,  -1703 },
	{   9568,   9109,   8519,   7995,   7405,   6815,   6488,   6094,   5636 },
	{  23134,  22216,  21692,  21168,  20054,  19660,  19070,  18677,  18153 },
};

struct d2186_adc {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock; /* serializes ADC multiplexer conversions */
	s32 ibus_slope;
	u8 die_temp_trim;
};

static int d2186_adc_read_raw_locked(struct d2186_adc *adc, u8 hw_chan,
				     u32 *raw)
{
	unsigned int ctrl;
	u8 buf[2];
	int ret;

	ret = regmap_write(adc->regmap, D2186_ADC_CTRL,
			   D2186_ADC_CTRL_START |
			   (hw_chan & D2186_ADC_CTRL_CHAN_MASK));
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(adc->regmap, D2186_ADC_CTRL, ctrl,
				       !(ctrl & D2186_ADC_CTRL_START),
				       D2186_ADC_POLL_US,
				       D2186_ADC_TIMEOUT_US);
	if (ret)
		goto cleanup;

	ret = regmap_bulk_read(adc->regmap, D2186_ADC_RESULT_LO, buf,
			       sizeof(buf));
cleanup:
	regmap_update_bits(adc->regmap, D2186_ADC_CFG,
			   D2186_ADC_CFG_CLEAR_MASK, 0);
	regmap_write(adc->regmap, D2186_ADC_CTRL, 0);

	if (ret)
		return ret;

	if (hw_chan == D2186_ADC_CHAN_IBUS &&
	    (buf[0] & D2186_ADC_RESULT_INVALID))
		return -EIO;

	*raw = (buf[0] & D2186_ADC_RESULT_LO_MASK) | ((u32)buf[1] << 4);
	return 0;
}

static int d2186_adc_rntc_to_mdegc(u32 r_ohm)
{
	unsigned int n = ARRAY_SIZE(d2186_ntc_lut);
	unsigned int i;

	if (r_ohm >= d2186_ntc_lut[0].r_ohm)
		return d2186_ntc_lut[0].mdegc;
	if (r_ohm <= d2186_ntc_lut[n - 1].r_ohm)
		return d2186_ntc_lut[n - 1].mdegc;

	for (i = 0; i < n - 1; i++) {
		u32 r1 = d2186_ntc_lut[i].r_ohm;
		u32 r2 = d2186_ntc_lut[i + 1].r_ohm;

		if (r_ohm >= r2) {
			int t1 = d2186_ntc_lut[i].mdegc;
			int t2 = d2186_ntc_lut[i + 1].mdegc;

			return t2 - (int)div_s64((s64)(t2 - t1) * (r_ohm - r2),
						 r1 - r2);
		}
	}

	return d2186_ntc_lut[n - 1].mdegc;
}

static s64 d2186_adc_ibus_ron_q16(s64 vbus_q16, s64 temp_q16)
{
	unsigned int vi = 0, ti = 0;
	s64 v_span, t_span, r0, r1;

	vbus_q16 = clamp(vbus_q16, d2186_ibus_vbus_axis[0],
			 d2186_ibus_vbus_axis[ARRAY_SIZE(d2186_ibus_vbus_axis) - 1]);
	temp_q16 = clamp(temp_q16, d2186_ibus_temp_axis[0],
			 d2186_ibus_temp_axis[ARRAY_SIZE(d2186_ibus_temp_axis) - 1]);

	while (vi < ARRAY_SIZE(d2186_ibus_vbus_axis) - 2 &&
	       vbus_q16 > d2186_ibus_vbus_axis[vi + 1])
		vi++;
	while (ti < ARRAY_SIZE(d2186_ibus_temp_axis) - 2 &&
	       temp_q16 > d2186_ibus_temp_axis[ti + 1])
		ti++;

	v_span = d2186_ibus_vbus_axis[vi + 1] - d2186_ibus_vbus_axis[vi];
	r0 = div64_s64((d2186_ibus_vbus_axis[vi + 1] - vbus_q16) *
		       d2186_ibus_ron_q16[ti][vi] +
		       (vbus_q16 - d2186_ibus_vbus_axis[vi]) *
		       d2186_ibus_ron_q16[ti][vi + 1], v_span);
	r1 = div64_s64((d2186_ibus_vbus_axis[vi + 1] - vbus_q16) *
		       d2186_ibus_ron_q16[ti + 1][vi] +
		       (vbus_q16 - d2186_ibus_vbus_axis[vi]) *
		       d2186_ibus_ron_q16[ti + 1][vi + 1], v_span);

	t_span = d2186_ibus_temp_axis[ti + 1] - d2186_ibus_temp_axis[ti];
	return r0 + div64_s64((temp_q16 - d2186_ibus_temp_axis[ti]) * (r1 - r0),
			      t_span);
}

static int d2186_adc_read_ibus_ma_locked(struct d2186_adc *adc, int *ma)
{
	u32 raw_ibus, raw_vbus, raw_tdie;
	s64 vbus_q16, temp_q16, ron_q16, denom;
	u64 q, i_q16;
	int ret;

	if (adc->ibus_slope <= 0)
		return -ENODATA;

	ret = d2186_adc_read_raw_locked(adc, D2186_ADC_CHAN_IBUS, &raw_ibus);
	if (ret)
		return ret;

	ret = d2186_adc_read_raw_locked(adc, D2186_ADC_CHAN_VBUS, &raw_vbus);
	if (ret)
		return ret;

	ret = d2186_adc_read_raw_locked(adc, D2186_IBUS_DIE_TEMP_CHAN,
					&raw_tdie);
	if (ret)
		return ret;

	vbus_q16 = (s64)raw_vbus * 88;
	temp_q16 = div_s64(((s64)raw_tdie - (s64)adc->die_temp_trim * 16) *
			   -0x6c80000LL + 0xf42400000LL, 10000);
	ron_q16 = d2186_adc_ibus_ron_q16(vbus_q16, temp_q16);
	denom = ron_q16 + 0x10000LL;
	if (denom <= 0)
		return -ERANGE;

	q = div64_u64((u64)raw_ibus << 32, (u32)adc->ibus_slope);
	i_q16 = D2186_IBUS_OFFSET_Q16 + div64_u64(q << 16, (u64)denom);
	*ma = (int)((i_q16 * 1000 + 32768) >> 16);
	return 0;
}

static int d2186_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct d2186_adc *adc = iio_priv(indio_dev);
	u32 raw;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&adc->lock);
		ret = d2186_adc_read_raw_locked(adc, chan->channel, &raw);
		mutex_unlock(&adc->lock);
		if (ret)
			return ret;
		*val = raw;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (chan->channel == D2186_ADC_CHAN_TBAT) {
			u32 r_ohm;

			mutex_lock(&adc->lock);
			ret = d2186_adc_read_raw_locked(adc,
							D2186_ADC_CHAN_TBAT,
							&raw);
			mutex_unlock(&adc->lock);
			if (ret)
				return ret;
			/* Vntc = raw * 40 Q16.16 V, 50 uA drive => R = V * 20000 */
			r_ohm = (u32)(((u64)raw * 40 * 20000) >> 16);
			*val = d2186_adc_rntc_to_mdegc(r_ohm);
			return IIO_VAL_INT;
		}
		if (chan->channel == D2186_ADC_CHAN_IBUS) {
			mutex_lock(&adc->lock);
			ret = d2186_adc_read_ibus_ma_locked(adc, val);
			mutex_unlock(&adc->lock);
			return ret ? ret : IIO_VAL_INT;
		}
		return -EINVAL;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case D2186_ADC_CHAN_VCC_MAIN:
		case D2186_ADC_CHAN_VBAT:
			*val = 2500;
			*val2 = 12;
			return IIO_VAL_FRACTIONAL_LOG2;
		case D2186_ADC_CHAN_VBUS:
			*val = 5500;
			*val2 = 12;
			return IIO_VAL_FRACTIONAL_LOG2;
		case D2186_ADC_CHAN_ICH:
			*val = 1000;
			*val2 = 12;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->channel) {
		case D2186_ADC_CHAN_VCC_MAIN:
		case D2186_ADC_CHAN_VBAT:
			/* 2500 mV offset = 4096 * (2500 / 4096) mV */
			*val = 4096;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int d2186_adc_read_label(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				char *label)
{
	return sysfs_emit(label, "%s\n", chan->datasheet_name);
}

static int d2186_adc_fwnode_xlate(struct iio_dev *indio_dev,
				  const struct fwnode_reference_args *iiospec)
{
	unsigned int i;

	if (iiospec->nargs != 1)
		return -EINVAL;

	for (i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].channel == iiospec->args[0])
			return i;
	}

	return -EINVAL;
}

static const struct iio_info d2186_adc_info = {
	.read_raw = d2186_adc_read_raw,
	.read_label = d2186_adc_read_label,
	.fwnode_xlate = d2186_adc_fwnode_xlate,
};

static const struct iio_chan_spec d2186_adc_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_VCC_MAIN,
		.datasheet_name = "vcc_main",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
	},
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_ICH,
		.datasheet_name = "ich",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_TBAT,
		.datasheet_name = "tbat",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_VBAT,
		.datasheet_name = "vbat",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
	},
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_IBUS,
		.datasheet_name = "ibus",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = D2186_ADC_CHAN_VBUS,
		.datasheet_name = "vbus",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int d2186_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct d2186_adc *adc;
	struct regmap *regmap;
	unsigned int trim;
	u8 cal[4];
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV,
				     "parent PMIC regmap unavailable\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	adc->regmap = regmap;
	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	ret = regmap_bulk_read(regmap, D2186_IBUS_CAL_BASE, cal, sizeof(cal));
	if (!ret) {
		u16 cal_low = cal[0] | ((u16)cal[1] << 8);
		u16 cal_high = cal[2] | ((u16)cal[3] << 8);

		if (cal_high > cal_low) {
			s64 diff_q32 = (s64)(cal_high - cal_low) << 32;

			adc->ibus_slope = (s32)div_s64(diff_q32,
						       D2186_IBUS_CAL_DELTA_Q16);
		}
	} else {
		dev_warn(dev, "ibus calibration read failed (%d); ibus processed disabled\n",
			 ret);
	}

	ret = regmap_read(regmap,
			  D2186_DIE_TEMP_TRIM_BASE +
			  (D2186_IBUS_DIE_TEMP_CHAN - D2186_DIE_TEMP_CHAN_BASE),
			  &trim);
	if (!ret)
		adc->die_temp_trim = trim;

	indio_dev->name = "apple-d2186-adc";
	indio_dev->info = &d2186_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = d2186_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(d2186_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id d2186_adc_of_match[] = {
	{ .compatible = "apple,d2186-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, d2186_adc_of_match);

static struct platform_driver d2186_adc_driver = {
	.probe = d2186_adc_probe,
	.driver = {
		.name = "apple-d2186-adc",
		.of_match_table = d2186_adc_of_match,
	},
};
module_platform_driver(d2186_adc_driver);

MODULE_DESCRIPTION("Apple D2186 PMIC housekeeping ADC driver");
MODULE_LICENSE("GPL");
