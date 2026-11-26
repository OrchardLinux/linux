// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and USB charger monitor for the Dialog D2186 PMIC on Apple A8
 * devices without a dedicated fuel-gauge IC.
 *
 * Reads telemetry from the D2186 housekeeping ADC via IIO and reads charger
 * status from PMIC registers 0x60 and 0x62 (plus optional read-only hardware
 * averaged charge current at 0x4e2). With a board input-current maximum in
 * the device tree, it also sets the USB input-current limit to what the USB
 * host grants (see "USB input-current grant" below); it never writes the
 * battery charge current, the charge voltage or the input-suspend control.
 *
 * State-of-charge estimation follows AppleARMPMUCharger's pmu-measured model
 * for battery-family 6 (N102): 10-sample VBAT burst averaging, the 100-point
 * 0006-default OCV curve (3400..4275 mV), a 6-slot median-filtered OCV ring
 * (4-sample window while charging, 6-sample while discharging), and monotonic
 * rate limiting (at most 1% per 20 s). Because this driver never suspends
 * charging to take a rest-voltage reading,
 * charging mode pairs VBAT with ICH to apply IR compensation
 * (V_ocv = V_bat - I_ch * R_int) and integrates charge current
 * (I_ch * dt / Q_full) with gentle OCV drift anchoring.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/consumer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* D2186 status block registers (0x60..0x6d), read-only */
#define D2186_STATUS_A			0x60
#define D2186_STATUS_A_USB_PRESENT	BIT(0)
#define D2186_STATUS_A_TEMP_FAULT	BIT(7)

#define D2186_STATUS_C			0x62
#define D2186_STATUS_C_CHG_STATE_MASK	GENMASK(1, 0)
#define D2186_STATUS_C_CHG_END		BIT(2)
#define D2186_STATUS_C_CHG_TIMEOUT	BIT(4)

/* Hardware-averaged charge current register (AppleD2186PMU ich_avg, read-only) */
#define D2186_ICH_AVG_REG		0x4e2
#define D2186_ICH_AVG_INVALID		0xff

/*
 * D2186 charger controls, as AppleD2186PMUPowerSource (iOS 12) uses them.
 * Input limit: 70 mA + 10 mA per code, 0xff from 2620 mA; iOS sets the input
 * suspend bit only for a target under 70 mA. Charge current: 30 mA per code.
 */
#define D2186_CHG_CONTROL		0x10
#define D2186_CHG_CONTROL_INPUT_SUSPEND	BIT(2)
#define D2186_CHG_INPUT_LIMIT		0x4c0
#define D2186_CHG_CHARGE_CURRENT	0x4c6
#define D2186_CHG_CHARGE_CURRENT_MASK	GENMASK(5, 0)
#define D2186_CHG_CHARGE_CURRENT_STEP_MA 30

#define D2186_ILIM_HANDOFF_CODE		0x03	/* 100 mA */
#define D2186_ILIM_FALLBACK_CODE	0x2b	/* 500 mA, a USB 2.0 port */
#define D2186_ILIM_MAX_CODE		0xff	/* 2620 mA */
#define D2186_GRANT_POLL_MS		1000
#define D2186_RAMP_HOLD_MS		100
#define D2186_RAMP_STEP_MA		100

#define D2186_DEFAULT_CHARGE_FULL_UAH	1080000
#define D2186_DEFAULT_VMIN_UV		3400000
#define D2186_DEFAULT_VMAX_UV		4350000
#define D2186_DEFAULT_RINT_UOHM		200000

/*
 * AppleARMPMUCharger pmu-measured parameters for battery-family 6 (N102):
 * - 10 consecutive VBAT samples averaged per update
 * - +35 mV discharge load compensation (measured Linux desktop sag on N102)
 * - 6-slot OCV median ring (window 4 while charging, 6 while discharging)
 * - 10 s periodic poll interval; 1% per 20 s (_smoothSOC ui-update-interval)
 * - 99% cap while actively charging until CHG_END (FULL)
 */
#define D2186_VBAT_SAMPLES		10
#define D2186_DISCHARGE_LOAD_COMP_UV	35000
#define D2186_OCV_RING_SIZE		6
#define D2186_OCV_AVG_CHARGING		4
#define D2186_OCV_AVG_DISCHARGING	6
#define D2186_POLL_INTERVAL_MS		10000
#define D2186_STALE_INTERVAL_MS		15000
#define D2186_LONG_GAP_MS		60000
#define D2186_SMOOTH_INTERVAL_MS	20000
#define D2186_ICH_LOW_UA		80000

/* Fixed-point SoC in micro-percent: 1% = 1,000,000, 100% = 100,000,000 */
#define D2186_UPCT_SCALE		1000000
#define D2186_UPCT_MAX			(100 * D2186_UPCT_SCALE)
#define D2186_UPCT_CHARGING_MAX		(99 * D2186_UPCT_SCALE)
#define D2186_RELAX_DEADBAND_UPCT	(2 * D2186_UPCT_SCALE)

/*
 * Exact 0%..99% open-circuit voltage table (in mV) from AppleARMPMUCharger
 * battery-data["0006-default"] for battery-family 6 (iPod touch 6, N102).
 * 100% is reached only on hardware charge termination (CHG_END).
 */
static const u16 d2186_n102_ocv_mv[100] = {
	/*  0.. 9 */ 3400, 3460, 3535, 3595, 3641, 3669, 3678, 3683, 3685, 3687,
	/* 10..19 */ 3689, 3690, 3693, 3701, 3709, 3715, 3720, 3727, 3732, 3737,
	/* 20..29 */ 3741, 3744, 3748, 3752, 3755, 3758, 3761, 3764, 3766, 3768,
	/* 30..39 */ 3771, 3773, 3775, 3778, 3780, 3783, 3786, 3789, 3791, 3794,
	/* 40..49 */ 3797, 3801, 3804, 3807, 3811, 3814, 3818, 3822, 3826, 3831,
	/* 50..59 */ 3835, 3840, 3845, 3850, 3855, 3861, 3867, 3874, 3881, 3889,
	/* 60..69 */ 3897, 3905, 3914, 3923, 3931, 3939, 3947, 3954, 3960, 3965,
	/* 70..79 */ 3970, 3976, 3984, 3995, 4009, 4025, 4043, 4057, 4064, 4069,
	/* 80..89 */ 4075, 4084, 4094, 4104, 4114, 4125, 4135, 4145, 4156, 4166,
	/* 90..99 */ 4176, 4187, 4197, 4208, 4219, 4230, 4241, 4252, 4263, 4275,
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

	struct mutex lock; /* protects SoC estimator state */
	struct delayed_work poll_work;

	int q_full_uah;
	int r_int_uohm;
	int v_shutdown_uv;

	bool soc_valid;
	int last_status;
	bool last_usb_present;
	int last_ich_ua;
	u64 last_sample_ns;
	int ocv_ring[D2186_OCV_RING_SIZE];
	unsigned int ring_idx;
	unsigned int ring_count;
	int est_upct;
	int display_upct;
	int reported_cap;
	unsigned int low_vbat_count;

	/* USB input-current grant; grant_lock protects everything below. */
	struct mutex grant_lock;
	struct delayed_work grant_work;
	unsigned int max_ma;		/* board maximum; 0: no grant */
	unsigned int boot_code;		/* input limit as the boot loader left it */
	unsigned int code;		/* last verified input limit */
	unsigned int target_code;
	unsigned int usb_ma;		/* accepted USB host allowance */
	unsigned int usb_report_ma;	/* report held over system sleep */
	bool usb_report_held;
	bool present;
	bool ramping;
	bool fault;
	bool sleeping;
	bool stopping;
	u64 step_ns;
	u64 step_wait_ns;
	u64 guard_ns;
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
	bool usb_present = !!(status_a & D2186_STATUS_A_USB_PRESENT);

	if (usb_present && (status_c & D2186_STATUS_C_CHG_END))
		return POWER_SUPPLY_STATUS_FULL;

	if (usb_present && (status_c & D2186_STATUS_C_CHG_TIMEOUT))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	/*
	 * D2186 status C bits 1:0 directly report hardware charger active state
	 * (AppleD2186PMUPowerSource::getChargeStatus). Honor it even if status
	 * A bit 0 momentarily glitches.
	 */
	if (status_c & D2186_STATUS_C_CHG_STATE_MASK)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (!usb_present)
		return is_battery ? POWER_SUPPLY_STATUS_DISCHARGING :
				    POWER_SUPPLY_STATUS_NOT_CHARGING;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int apple_d2186_read_ich_avg_ua(struct apple_d2186_battery *bat,
				       int *ich_ua)
{
	unsigned int val;
	int ret;

	ret = regmap_read(bat->regmap, D2186_ICH_AVG_REG, &val);
	if (ret)
		return ret;

	val &= 0xff;
	if (val == D2186_ICH_AVG_INVALID)
		return -ENODATA;

	*ich_ua = (int)DIV_ROUND_CLOSEST_ULL((u64)val * 1000000ULL, 256);
	return 0;
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

static int apple_d2186_table_to_upct(const struct power_supply_battery_ocv_table *tbl,
				     int len, int v_uv)
{
	int i;

	if (v_uv >= tbl[0].ocv)
		return clamp(tbl[0].capacity, 0, 100) * D2186_UPCT_SCALE;
	if (v_uv <= tbl[len - 1].ocv)
		return clamp(tbl[len - 1].capacity, 0, 100) * D2186_UPCT_SCALE;

	for (i = 0; i < len - 1; i++) {
		int v_hi = tbl[i].ocv;
		int v_lo = tbl[i + 1].ocv;
		int c_hi = tbl[i].capacity;
		int c_lo = tbl[i + 1].capacity;

		if (v_uv >= v_lo && v_hi > v_lo) {
			s64 span = (s64)(c_hi - c_lo) * D2186_UPCT_SCALE;

			return c_lo * D2186_UPCT_SCALE +
			       (int)div_s64(span * (v_uv - v_lo), v_hi - v_lo);
		}
	}

	return clamp(tbl[len - 1].capacity, 0, 100) * D2186_UPCT_SCALE;
}

static int apple_d2186_v_to_upct(struct apple_d2186_battery *bat, int v_uv)
{
	const struct power_supply_battery_ocv_table *tbl;
	int table_len = 0, temp_c = 20, tenth_c;
	unsigned int i;

	if (bat->binfo) {
		if (!apple_d2186_read_temp_tenth_c(bat, &tenth_c))
			temp_c = tenth_c / 10;

		tbl = power_supply_find_ocv2cap_table(bat->binfo, temp_c,
						      &table_len);
		/*
		 * Use a DT-provided OCV table only if it has high resolution
		 * (>= 50 points); otherwise prefer the built-in 100-point
		 * Apple 0006-default table.
		 */
		if (tbl && table_len >= 50)
			return apple_d2186_table_to_upct(tbl, table_len, v_uv);
	}

	if (v_uv <= (int)d2186_n102_ocv_mv[0] * 1000)
		return 0;
	if (v_uv >= (int)d2186_n102_ocv_mv[ARRAY_SIZE(d2186_n102_ocv_mv) - 1] * 1000)
		return D2186_UPCT_CHARGING_MAX;

	for (i = 0; i < ARRAY_SIZE(d2186_n102_ocv_mv) - 1; i++) {
		int v_lo = (int)d2186_n102_ocv_mv[i] * 1000;
		int v_hi = (int)d2186_n102_ocv_mv[i + 1] * 1000;

		if (v_uv < v_hi)
			return (int)i * D2186_UPCT_SCALE +
			       (int)div_s64((s64)(v_uv - v_lo) * D2186_UPCT_SCALE,
					    v_hi - v_lo);
	}

	return D2186_UPCT_CHARGING_MAX;
}

static int apple_d2186_sample_burst(struct apple_d2186_battery *bat,
				    bool sample_ich,
				    int *vbat_uv, int *ich_ua)
{
	s64 vbat_sum = 0, ich_sum = 0;
	unsigned int vbat_cnt = 0, ich_cnt = 0;
	unsigned int i;
	int v, cur, ret;

	for (i = 0; i < D2186_VBAT_SAMPLES; i++) {
		ret = iio_read_channel_processed_scale(bat->vbat, &v, 1000);
		if (!ret) {
			vbat_sum += v;
			vbat_cnt++;
		}

		if (sample_ich && bat->ich) {
			ret = iio_read_channel_processed_scale(bat->ich, &cur,
							       1000);
			if (!ret) {
				ich_sum += max(cur, 0);
				ich_cnt++;
			}
		}
	}

	if (!vbat_cnt)
		return -EIO;

	*vbat_uv = (int)div_s64(vbat_sum, vbat_cnt);
	*ich_ua = ich_cnt ? (int)div_s64(ich_sum, ich_cnt) : 0;
	return 0;
}

/*
 * Return the median of the most recent @window samples in the OCV ring buffer.
 * Using the median rather than a raw arithmetic mean rejects transient
 * CPU/GPU voltage-sag spikes that would otherwise depress the average for the
 * entire 60 s ring lifetime.
 */
static int apple_d2186_ring_median(struct apple_d2186_battery *bat,
				   unsigned int window)
{
	int vals[D2186_OCV_RING_SIZE];
	unsigned int count = min(bat->ring_count, window);
	unsigned int i, j, idx;

	if (!count)
		return 0;

	idx = (bat->ring_idx + D2186_OCV_RING_SIZE - 1) % D2186_OCV_RING_SIZE;
	for (i = 0; i < count; i++) {
		vals[i] = bat->ocv_ring[idx];
		idx = (idx + D2186_OCV_RING_SIZE - 1) % D2186_OCV_RING_SIZE;
	}

	for (i = 1; i < count; i++) {
		int key = vals[i];

		for (j = i; j > 0 && vals[j - 1] > key; j--)
			vals[j] = vals[j - 1];
		vals[j] = key;
	}

	if (count & 1)
		return vals[count / 2];

	return (vals[count / 2 - 1] + vals[count / 2]) / 2;
}

static int apple_d2186_update_locked(struct apple_d2186_battery *bat,
				     bool *bat_changed, bool *chg_changed)
{
	unsigned int status_a, status_c, win;
	bool usb_present, prev_chg, now_chg, long_gap;
	int status, vbat_uv, ich_ua, v_ocv_uv, ocv_upct, med_ocv_upct;
	int max_step, new_cap, ret;
	u64 now_ns, dt_ms;

	ret = apple_d2186_read_status_regs(bat, &status_a, &status_c);
	if (ret)
		return ret;

	usb_present = !!(status_a & D2186_STATUS_A_USB_PRESENT);
	status = apple_d2186_decode_status(status_a, status_c, true);

	ret = apple_d2186_sample_burst(bat,
				       usb_present ||
				       status == POWER_SUPPLY_STATUS_CHARGING,
				       &vbat_uv, &ich_ua);
	if (ret)
		return ret;

	/*
	 * Fall back to the D2186 hardware-averaged charge current register
	 * (0x4e2, ich_avg) if instantaneous ADC chan 1 reads 0 while charging.
	 */
	if (status == POWER_SUPPLY_STATUS_CHARGING && ich_ua == 0) {
		int ich_avg_ua;

		if (!apple_d2186_read_ich_avg_ua(bat, &ich_avg_ua) &&
		    ich_avg_ua > 0)
			ich_ua = ich_avg_ua;
	}

	now_ns = ktime_get_boottime_ns();
	dt_ms = bat->soc_valid ?
		div_u64(now_ns - bat->last_sample_ns, NSEC_PER_MSEC) : 0;

	if (status == POWER_SUPPLY_STATUS_FULL) {
		ocv_upct = D2186_UPCT_MAX;
	} else {
		if (status == POWER_SUPPLY_STATUS_DISCHARGING) {
			v_ocv_uv = vbat_uv + D2186_DISCHARGE_LOAD_COMP_UV;
		} else {
			int ir_uv = ich_ua > 0 ?
				    (int)div_s64((s64)ich_ua * bat->r_int_uohm,
						 1000000) : 0;

			v_ocv_uv = vbat_uv - ir_uv;
		}
		ocv_upct = apple_d2186_v_to_upct(bat, v_ocv_uv);
	}

	prev_chg = bat->last_status == POWER_SUPPLY_STATUS_CHARGING ||
		   bat->last_status == POWER_SUPPLY_STATUS_FULL;
	now_chg = status == POWER_SUPPLY_STATUS_CHARGING ||
		  status == POWER_SUPPLY_STATUS_FULL;
	long_gap = bat->soc_valid && dt_ms > D2186_LONG_GAP_MS;

	if (!bat->soc_valid || long_gap || prev_chg != now_chg) {
		bat->ring_idx = 0;
		bat->ring_count = 0;
	}

	bat->ocv_ring[bat->ring_idx] = ocv_upct;
	bat->ring_idx = (bat->ring_idx + 1) % D2186_OCV_RING_SIZE;
	if (bat->ring_count < D2186_OCV_RING_SIZE)
		bat->ring_count++;

	win = status == POWER_SUPPLY_STATUS_CHARGING ?
	      D2186_OCV_AVG_CHARGING : D2186_OCV_AVG_DISCHARGING;
	med_ocv_upct = apple_d2186_ring_median(bat, win);

	if (status == POWER_SUPPLY_STATUS_DISCHARGING &&
	    vbat_uv <= bat->v_shutdown_uv)
		bat->low_vbat_count++;
	else
		bat->low_vbat_count = 0;

	if (!bat->soc_valid) {
		bat->est_upct = med_ocv_upct;
		if (status == POWER_SUPPLY_STATUS_CHARGING)
			bat->est_upct = min(bat->est_upct,
					    D2186_UPCT_CHARGING_MAX);
		bat->display_upct = bat->est_upct;
		bat->soc_valid = true;
	} else if (status == POWER_SUPPLY_STATUS_FULL) {
		bat->est_upct = D2186_UPCT_MAX;
	} else if (status == POWER_SUPPLY_STATUS_CHARGING) {
		if (!long_gap &&
		    bat->last_status == POWER_SUPPLY_STATUS_CHARGING) {
			int ich_mid_ua = (bat->last_ich_ua + ich_ua) / 2;
			int diff;

			if (ich_mid_ua > 0 && dt_ms > 0) {
				int delta_upct = (int)div64_s64(
					(s64)ich_mid_ua * (s64)dt_ms * 1000LL,
					(s64)bat->q_full_uah * 36LL);

				bat->est_upct = min(bat->est_upct + delta_upct,
						    D2186_UPCT_CHARGING_MAX);
			}

			diff = med_ocv_upct - bat->est_upct;
			if (diff > 0) {
				int div = ich_ua < D2186_ICH_LOW_UA ? 8 : 16;

				bat->est_upct += diff / div;
			} else if (diff < -D2186_RELAX_DEADBAND_UPCT) {
				bat->est_upct += diff / 16;
			}
		} else if (long_gap) {
			bat->est_upct = med_ocv_upct;
		} else {
			/* Smooth handoff on charger plug-in without jumping */
			bat->est_upct = bat->display_upct;
		}
		bat->est_upct = clamp(bat->est_upct, 0, D2186_UPCT_CHARGING_MAX);
	} else {
		bat->est_upct = med_ocv_upct;
	}

	/* Apple _smoothSOC rate limiting: max 1% per 20 s (4x faster at UVLO) */
	if (dt_ms > 0) {
		u64 interval_ms = bat->low_vbat_count >= 2 ?
				  D2186_SMOOTH_INTERVAL_MS / 4 :
				  D2186_SMOOTH_INTERVAL_MS;

		max_step = (int)min_t(u64,
				      div_u64(dt_ms * D2186_UPCT_SCALE,
					      interval_ms),
				      D2186_UPCT_MAX);
	} else {
		max_step = 0;
	}

	/*
	 * Enforce iOS _smoothSOC directionality:
	 * - FULL / CHARGING (0x62 bits 1:0) / external USB connected:
	 *   strictly non-decreasing regardless of measured ich.
	 * - DISCHARGING (USB absent and charger inactive):
	 *   strictly non-increasing.
	 */
	if (status == POWER_SUPPLY_STATUS_FULL) {
		if (bat->display_upct < D2186_UPCT_MAX)
			bat->display_upct = min(D2186_UPCT_MAX,
						bat->display_upct + max_step);
	} else if (status == POWER_SUPPLY_STATUS_CHARGING || usb_present) {
		int target = min(bat->est_upct, D2186_UPCT_CHARGING_MAX);

		if (target > bat->display_upct)
			bat->display_upct = min(target,
						bat->display_upct + max_step);
	} else {
		if (bat->est_upct < bat->display_upct)
			bat->display_upct = max(bat->est_upct,
						bat->display_upct - max_step);
	}

	if (status == POWER_SUPPLY_STATUS_FULL &&
	    bat->display_upct >= D2186_UPCT_MAX - D2186_UPCT_SCALE / 2) {
		new_cap = 100;
	} else {
		new_cap = DIV_ROUND_CLOSEST(bat->display_upct, D2186_UPCT_SCALE);
		if (status == POWER_SUPPLY_STATUS_CHARGING)
			new_cap = clamp(new_cap, 1, 99);
		else if (bat->low_vbat_count >= 2 && new_cap == 0)
			new_cap = 0;
		else
			new_cap = clamp(new_cap, 1, 100);
	}

	if (bat_changed)
		*bat_changed = new_cap != bat->reported_cap ||
			       status != bat->last_status;
	if (chg_changed)
		*chg_changed = usb_present != bat->last_usb_present ||
			       status != bat->last_status;

	bat->reported_cap = new_cap;
	bat->last_status = status;
	bat->last_usb_present = usb_present;
	bat->last_ich_ua = ich_ua;
	bat->last_sample_ns = now_ns;
	return 0;
}

static void apple_d2186_poll_work(struct work_struct *work)
{
	struct apple_d2186_battery *bat =
		container_of(work, struct apple_d2186_battery, poll_work.work);
	bool bat_changed = false, chg_changed = false;
	int ret;

	mutex_lock(&bat->lock);
	ret = apple_d2186_update_locked(bat, &bat_changed, &chg_changed);
	mutex_unlock(&bat->lock);

	if (!ret) {
		if (bat_changed)
			power_supply_changed(bat->bat_psy);
		if (chg_changed)
			power_supply_changed(bat->chg_psy);
	}

	queue_delayed_work(system_freezable_wq, &bat->poll_work,
			   msecs_to_jiffies(D2186_POLL_INTERVAL_MS));
}

static int apple_d2186_bat_capacity(struct apple_d2186_battery *bat,
				    int *capacity)
{
	u64 age_ms;
	int ret = 0;

	mutex_lock(&bat->lock);
	age_ms = bat->soc_valid ?
		 div_u64(ktime_get_boottime_ns() - bat->last_sample_ns,
			 NSEC_PER_MSEC) :
		 D2186_STALE_INTERVAL_MS;
	if (!bat->soc_valid || age_ms >= D2186_STALE_INTERVAL_MS)
		ret = apple_d2186_update_locked(bat, NULL, NULL);
	if (!ret)
		*capacity = bat->reported_cap;
	mutex_unlock(&bat->lock);

	return ret;
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
		 * (positive while charging, 0 when discharging or when system
		 * load consumes all USB input current; there is no battery
		 * coulomb counter or discharge current sense on N102).
		 */
		if (!bat->ich)
			return -ENODATA;
		ret = iio_read_channel_processed_scale(bat->ich,
						       &val->intval, 1000);
		if (!ret && val->intval == 0 &&
		    !regmap_read(bat->regmap, D2186_STATUS_C, &status_c) &&
		    (status_c & D2186_STATUS_C_CHG_STATE_MASK)) {
			int ich_avg_ua;

			if (!apple_d2186_read_ich_avg_ua(bat, &ich_avg_ua) &&
			    ich_avg_ua > 0)
				val->intval = ich_avg_ua;
		}
		return ret;

	case POWER_SUPPLY_PROP_TEMP:
		return apple_d2186_read_temp_tenth_c(bat, &val->intval);

	case POWER_SUPPLY_PROP_CAPACITY:
		return apple_d2186_bat_capacity(bat, &val->intval);

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bat->q_full_uah;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->v_shutdown_uv;
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

/*
 * USB input-current grant
 *
 * The same contract as the Lightning charger on the iPad (apple-tristar-
 * charger): the USB device controller reports what the host allows the
 * device to draw (vbus_draw, which includes an Apple host's current grant)
 * as this supply's input current limit. Without that report, or for anything
 * up to 500 mA, an attached source gets the 500 mA of a USB 2.0 port. A
 * larger allowance is granted up to the board maximum, ramped as iOS does
 * (100 mA held for 100 ms, then 100 mA steps 10 ms apart), only while the
 * battery is inside the charging window. Every write is read back; the
 * limit is checked once a second and put back if the PMIC changed it. The
 * end of the input ends the grant.
 *
 * Differences from the iPad: the D2186 encodes the limit as 70 mA + 10 mA
 * per code (iOS: AppleD2186PMUPowerSource), and n102 has no PMU event line
 * for this driver, so attach and detach come from a 1 s poll of 0x60 bit 0.
 * The input limit is the only control written: the battery charge current
 * (0x4c6) and the input-suspend bit stay as iBoot and the PMIC set them, and
 * the limit the boot loader left is restored on remove and shutdown.
 */
static unsigned int d2186_code_ma(unsigned int code)
{
	return code * 10 + 70;
}

/* Round down, as iOS does: quantization must never exceed a permitted current. */
static unsigned int d2186_ma_code(unsigned int ma)
{
	if (ma < 70)
		return 0;
	return min((ma - 70) / 10, D2186_ILIM_MAX_CODE);
}

/* Caller holds grant_lock. */
static int d2186_set_code(struct apple_d2186_battery *bat, unsigned int code)
{
	unsigned int readback;
	int ret;

	ret = regmap_write(bat->regmap, D2186_CHG_INPUT_LIMIT, code);
	if (!ret)
		ret = regmap_read(bat->regmap, D2186_CHG_INPUT_LIMIT, &readback);
	if (!ret && readback != code)
		ret = -EIO;
	if (!ret)
		bat->code = code;
	return ret;
}

/* Caller holds grant_lock. */
static void d2186_withdraw(struct apple_d2186_battery *bat, unsigned int code)
{
	if (bat->code > code && d2186_set_code(bat, code))
		dev_crit(bat->dev, "input-limit withdrawal failed; limit unverified\n");
}

/* Grants above the fallback are allowed only inside these pack limits. */
static bool d2186_pack_allows_grant(struct apple_d2186_battery *bat)
{
	int vbat_uv, tenth_c;

	if (iio_read_channel_processed_scale(bat->vbat, &vbat_uv, 1000) ||
	    apple_d2186_read_temp_tenth_c(bat, &tenth_c))
		return false;
	return tenth_c >= 50 && tenth_c <= 400 &&
	       vbat_uv >= 3200000 && vbat_uv <= 4350000;
}

/* Caller holds grant_lock. */
static unsigned int d2186_target_code(struct apple_d2186_battery *bat)
{
	if (bat->usb_ma <= d2186_code_ma(D2186_ILIM_FALLBACK_CODE))
		return D2186_ILIM_FALLBACK_CODE;
	return d2186_ma_code(min(bat->usb_ma, bat->max_ma));
}

/* Follow a changed allowance. Caller holds grant_lock. */
static void d2186_update_target(struct apple_d2186_battery *bat)
{
	unsigned int old_target = bat->target_code;

	if (!bat->present || bat->fault)
		return;
	bat->target_code = d2186_target_code(bat);
	if (bat->code > bat->target_code) {
		d2186_withdraw(bat, bat->target_code);
		bat->ramping = false;
	} else if (bat->code < bat->target_code &&
		   (!bat->ramping || bat->target_code != old_target)) {
		/* A new ramp, or a new target mid-ramp: check the pack first. */
		bat->ramping = true;
		bat->step_ns = 0;
	}
}

/* Caller holds grant_lock. */
static void d2186_ramp_step(struct apple_d2186_battery *bat, u64 now)
{
	unsigned int next, from_ma = d2186_code_ma(bat->code);
	u64 wait;

	if (bat->step_ns && now - bat->step_ns < bat->step_wait_ns)
		return;
	if (bat->code >= bat->target_code) {
		/* Nothing to ramp: a limit above the target goes down at once. */
		d2186_withdraw(bat, bat->target_code);
		bat->ramping = false;
		return;
	}
	/* The ADC is slow: look at the pack when a ramp starts, not per step. */
	if (!bat->step_ns && bat->target_code > D2186_ILIM_FALLBACK_CODE &&
	    !d2186_pack_allows_grant(bat)) {
		bat->target_code = D2186_ILIM_FALLBACK_CODE;
		if (bat->code >= bat->target_code) {
			d2186_withdraw(bat, bat->target_code);
			bat->ramping = false;
			dev_warn(bat->dev, "battery outside the charging window; holding the fallback\n");
			return;
		}
	}
	if (!bat->step_ns && bat->code <= D2186_ILIM_HANDOFF_CODE) {
		/* From nothing, iOS writes 100 mA and holds it. */
		next = D2186_ILIM_HANDOFF_CODE;
		wait = D2186_RAMP_HOLD_MS * NSEC_PER_MSEC;
	} else {
		next = min(bat->target_code,
			   d2186_ma_code(from_ma + D2186_RAMP_STEP_MA));
		wait = (u64)(d2186_code_ma(next) - from_ma) * (NSEC_PER_MSEC / 10);
	}
	if (d2186_set_code(bat, next)) {
		bat->fault = true;
		bat->ramping = false;
		d2186_withdraw(bat, D2186_ILIM_FALLBACK_CODE);
		dev_err(bat->dev, "input-limit write failed; holding the fallback\n");
		return;
	}
	bat->step_ns = now;
	bat->step_wait_ns = wait;
	if (next >= bat->target_code) {
		bat->ramping = false;
		dev_info(bat->dev, "USB input limit %u mA (host allows %u mA)\n",
			 d2186_code_ma(bat->code), bat->usb_ma);
	}
}

/*
 * Once a second: put back a limit the PMIC changed (an attach default), and
 * hold the fallback while the pack is outside the charging window or the
 * PMIC suspends its input. Caller holds grant_lock.
 */
static void d2186_guard(struct apple_d2186_battery *bat, u64 now)
{
	unsigned int hw, control;
	bool allowed;

	if (now - bat->guard_ns < (u64)D2186_GRANT_POLL_MS * NSEC_PER_MSEC)
		return;
	bat->guard_ns = now;

	if (!regmap_read(bat->regmap, D2186_CHG_INPUT_LIMIT, &hw) &&
	    hw != bat->code) {
		dev_warn_ratelimited(bat->dev,
				     "input limit changed to %u mA by the PMIC; restoring\n",
				     d2186_code_ma(hw));
		if (hw > bat->code) {
			if (d2186_set_code(bat, bat->code))
				dev_crit(bat->dev, "input-limit write failed; limit unverified\n");
			return;
		}
		if (d2186_set_code(bat, min(bat->code, D2186_ILIM_FALLBACK_CODE))) {
			dev_crit(bat->dev, "fallback write failed; input limit unverified\n");
			return;
		}
	}
	if (bat->fault)
		return;

	allowed = d2186_pack_allows_grant(bat);
	if (!regmap_read(bat->regmap, D2186_CHG_CONTROL, &control) &&
	    (control & D2186_CHG_CONTROL_INPUT_SUSPEND))
		allowed = false;

	bat->target_code = d2186_target_code(bat);
	if (!allowed && bat->code > D2186_ILIM_FALLBACK_CODE) {
		bat->ramping = false;
		d2186_withdraw(bat, D2186_ILIM_FALLBACK_CODE);
		dev_warn(bat->dev, "battery outside the charging window or input suspended; holding the fallback\n");
	} else if (allowed && bat->code < bat->target_code && !bat->ramping) {
		/* Back inside the window: no replug needed to charge again. */
		bat->ramping = true;
		bat->step_ns = 0;
	}
}

/* Caller holds grant_lock. Returns true when the supply changed. */
static bool d2186_grant_poll(struct apple_d2186_battery *bat, u64 now)
{
	unsigned int status_a, old_code = bat->code, old_usb_ma = bat->usb_ma;
	bool present, was_present = bat->present;

	if (regmap_read(bat->regmap, D2186_STATUS_A, &status_a)) {
		dev_err_ratelimited(bat->dev, "PMIC status unreadable; holding the fallback\n");
		bat->ramping = false;
		d2186_withdraw(bat, D2186_ILIM_FALLBACK_CODE);
		return bat->code != old_code;
	}
	present = !!(status_a & D2186_STATUS_A_USB_PRESENT);

	if (present && !was_present) {
		/* A new attachment: the fallback, until the host reports. */
		bat->present = true;
		bat->fault = false;
		bat->usb_ma = 0;
		bat->target_code = D2186_ILIM_FALLBACK_CODE;
		bat->ramping = true;
		bat->step_ns = 0;
		bat->guard_ns = now;
	} else if (!present && was_present) {
		/* Only a lost input ends the grant. */
		bat->present = false;
		bat->ramping = false;
		bat->usb_ma = 0;
		d2186_withdraw(bat, D2186_ILIM_FALLBACK_CODE);
	}

	if (bat->present) {
		if (bat->ramping)
			d2186_ramp_step(bat, now);
		else
			d2186_guard(bat, now);
	}

	return present != was_present || bat->usb_ma != old_usb_ma ||
	       (!bat->ramping && bat->code != old_code);
}

static void apple_d2186_grant_work(struct work_struct *work)
{
	struct apple_d2186_battery *bat =
		container_of(work, struct apple_d2186_battery, grant_work.work);
	unsigned long delay = msecs_to_jiffies(D2186_GRANT_POLL_MS);
	bool changed;
	u64 now;

	mutex_lock(&bat->grant_lock);
	if (bat->stopping) {
		mutex_unlock(&bat->grant_lock);
		return;
	}
	now = ktime_get_boottime_ns();
	changed = d2186_grant_poll(bat, now);
	if (bat->ramping) {
		u64 due = bat->step_ns ? bat->step_ns + bat->step_wait_ns : now;

		delay = (s64)(due - now) > 0 ?
			max(usecs_to_jiffies(div_u64(due - now, NSEC_PER_USEC)), 1UL) : 0;
	}
	queue_delayed_work(system_freezable_wq, &bat->grant_work, delay);
	if (bat->sleeping)
		changed = false;
	mutex_unlock(&bat->grant_lock);

	if (changed)
		power_supply_changed(bat->chg_psy);
}

/*
 * The USB device controller reports what the host allows the device to draw
 * (vbus_draw) as the input current limit: 0 without a session, 100 mA
 * unconfigured, the configuration's bMaxPower, 2 mA suspended, or more after
 * an Apple host's current grant. Not writable from user space. A report
 * while no input is present counts as nothing.
 *
 * A system sleep is not the host lowering its grant: a report from prepare
 * to complete is held and applied at complete, as on the iPad.
 */
static int apple_d2186_chg_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct apple_d2186_battery *bat = power_supply_get_drvdata(psy);
	unsigned int old_code, old_ma, report_ma, accept_ma;
	bool changed;

	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;
	if (!bat->max_ma)
		return -EOPNOTSUPP;

	mutex_lock(&bat->grant_lock);
	old_code = bat->code;
	old_ma = bat->usb_ma;
	report_ma = max(val->intval, 0) / 1000;
	accept_ma = bat->present ? min(report_ma, bat->max_ma) : 0;
	if (bat->sleeping) {
		bat->usb_report_ma = accept_ma;
		bat->usb_report_held = accept_ma != old_ma;
		dev_dbg(bat->dev, "USB budget: reported %u mA, %u mA kept over sleep\n",
			report_ma, old_ma);
		mutex_unlock(&bat->grant_lock);
		return 0;
	}
	bat->usb_report_held = false;
	bat->usb_ma = accept_ma;
	dev_dbg(bat->dev, "USB budget: reported %u mA, accepted %u mA, present %u\n",
		report_ma, bat->usb_ma, bat->present);
	if (!bat->stopping) {
		d2186_update_target(bat);
		mod_delayed_work(system_freezable_wq, &bat->grant_work, 0);
	}
	changed = bat->code != old_code || bat->usb_ma != old_ma;
	mutex_unlock(&bat->grant_lock);

	if (changed)
		power_supply_changed(psy);
	return 0;
}

/* Stop granting and put back the limit the boot loader left. */
static void apple_d2186_grant_stop(struct apple_d2186_battery *bat)
{
	if (!bat->max_ma)
		return;

	mutex_lock(&bat->grant_lock);
	bat->stopping = true;
	mutex_unlock(&bat->grant_lock);
	cancel_delayed_work_sync(&bat->grant_work);

	mutex_lock(&bat->grant_lock);
	if (bat->code != bat->boot_code &&
	    d2186_set_code(bat, bat->boot_code))
		dev_err(bat->dev, "cannot restore the boot input limit\n");
	mutex_unlock(&bat->grant_lock);
}

static void apple_d2186_grant_stop_action(void *data)
{
	apple_d2186_grant_stop(data);
}

/* Before the supply registers: set_property may run as soon as it does. */
static int apple_d2186_grant_init(struct apple_d2186_battery *bat)
{
	struct device *dev = bat->dev;
	unsigned int code, control = 0, charge = 0, status_a;
	u32 max_ua;
	int ret;

	ret = devm_mutex_init(dev, &bat->grant_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&bat->grant_work, apple_d2186_grant_work);

	if (device_property_read_u32(dev, "input-current-limit-microamp", &max_ua))
		return 0;

	ret = regmap_read(bat->regmap, D2186_CHG_INPUT_LIMIT, &code);
	if (!ret)
		ret = regmap_read(bat->regmap, D2186_CHG_CONTROL, &control);
	if (!ret)
		ret = regmap_read(bat->regmap, D2186_CHG_CHARGE_CURRENT, &charge);
	if (!ret)
		ret = regmap_read(bat->regmap, D2186_STATUS_A, &status_a);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the charger controls\n");

	bat->max_ma = min(max_ua / 1000, d2186_code_ma(D2186_ILIM_MAX_CODE));
	bat->boot_code = code;
	bat->code = code;
	bat->target_code = D2186_ILIM_FALLBACK_CODE;
	dev_info(dev,
		 "boot loader left: input limit 0x%02x (%u mA), charge current 0x%02x (%u mA), control 0x%02x, status 0x%02x; board maximum %u mA\n",
		 code, d2186_code_ma(code), charge,
		 (unsigned int)(charge & D2186_CHG_CHARGE_CURRENT_MASK) *
		 D2186_CHG_CHARGE_CURRENT_STEP_MA,
		 control, status_a, bat->max_ma);
	if (control & D2186_CHG_CONTROL_INPUT_SUSPEND)
		dev_warn(dev, "PMIC input suspended; holding the fallback\n");
	return 0;
}

static int apple_d2186_chg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct apple_d2186_battery *bat = power_supply_get_drvdata(psy);
	unsigned int status_a, status_c, code;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (bat->max_ma) {
			mutex_lock(&bat->grant_lock);
			code = bat->code;
			mutex_unlock(&bat->grant_lock);
		} else {
			ret = regmap_read(bat->regmap, D2186_CHG_INPUT_LIMIT, &code);
			if (ret)
				return ret;
		}
		val->intval = d2186_code_ma(code) * 1000;
		return 0;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* What the USB host allows, up to the board maximum; 0 when unknown. */
		mutex_lock(&bat->grant_lock);
		val->intval = bat->usb_ma * 1000;
		mutex_unlock(&bat->grant_lock);
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(bat->regmap, D2186_CHG_CHARGE_CURRENT, &code);
		if (ret)
			return ret;
		val->intval = (int)(code & D2186_CHG_CHARGE_CURRENT_MASK) *
			      D2186_CHG_CHARGE_CURRENT_STEP_MA * 1000;
		return 0;

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
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static const struct power_supply_desc apple_d2186_chg_desc = {
	.name = "apple-d2186-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = apple_d2186_chg_props,
	.num_properties = ARRAY_SIZE(apple_d2186_chg_props),
	.get_property = apple_d2186_chg_get_property,
	.set_property = apple_d2186_chg_set_property,
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

static void apple_d2186_cancel_poll(void *data)
{
	struct apple_d2186_battery *bat = data;

	cancel_delayed_work_sync(&bat->poll_work);
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
	bat->q_full_uah = D2186_DEFAULT_CHARGE_FULL_UAH;
	bat->r_int_uohm = D2186_DEFAULT_RINT_UOHM;
	bat->v_shutdown_uv = D2186_DEFAULT_VMIN_UV;
	bat->last_status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = devm_mutex_init(dev, &bat->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&bat->poll_work, apple_d2186_poll_work);

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

		if (bat->binfo->charge_full_design_uah > 0)
			bat->q_full_uah = bat->binfo->charge_full_design_uah;
		if (bat->binfo->voltage_min_design_uv > 0)
			bat->v_shutdown_uv = bat->binfo->voltage_min_design_uv;
		if (bat->binfo->factory_internal_resistance_charging_uohm > 0)
			bat->r_int_uohm = bat->binfo->factory_internal_resistance_charging_uohm;
		else if (bat->binfo->factory_internal_resistance_uohm > 0)
			bat->r_int_uohm = bat->binfo->factory_internal_resistance_uohm;
	}

	ret = apple_d2186_grant_init(bat);
	if (ret)
		return ret;

	chg_cfg.drv_data = bat;
	chg_cfg.no_wakeup_source = true;

	bat->chg_psy = devm_power_supply_register(dev, &apple_d2186_chg_desc,
						  &chg_cfg);
	if (IS_ERR(bat->chg_psy))
		return dev_err_probe(dev, PTR_ERR(bat->chg_psy),
				     "failed to register USB power supply\n");

	if (bat->max_ma) {
		ret = devm_add_action_or_reset(dev, apple_d2186_grant_stop_action,
					       bat);
		if (ret)
			return ret;
		platform_set_drvdata(pdev, bat);
		queue_delayed_work(system_freezable_wq, &bat->grant_work, 0);
	}

	mutex_lock(&bat->lock);
	apple_d2186_update_locked(bat, NULL, NULL);
	mutex_unlock(&bat->lock);

	ret = devm_add_action_or_reset(dev, apple_d2186_cancel_poll, bat);
	if (ret)
		return ret;

	queue_delayed_work(system_freezable_wq, &bat->poll_work,
			   msecs_to_jiffies(D2186_POLL_INTERVAL_MS));

	return 0;
}

static const struct of_device_id apple_d2186_battery_of_match[] = {
	{ .compatible = "apple,d2186-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_d2186_battery_of_match);

static void apple_d2186_battery_shutdown(struct platform_device *pdev)
{
	struct apple_d2186_battery *bat = platform_get_drvdata(pdev);

	if (bat)
		apple_d2186_grant_stop(bat);
}

static int apple_d2186_battery_prepare(struct device *dev)
{
	struct apple_d2186_battery *bat = dev_get_drvdata(dev);

	if (bat) {
		mutex_lock(&bat->grant_lock);
		bat->sleeping = true;
		bat->usb_report_held = false;
		mutex_unlock(&bat->grant_lock);
	}
	return 0;
}

static void apple_d2186_battery_complete(struct device *dev)
{
	struct apple_d2186_battery *bat = dev_get_drvdata(dev);
	bool changed = false;

	if (!bat)
		return;

	mutex_lock(&bat->grant_lock);
	bat->sleeping = false;
	if (bat->usb_report_held) {
		unsigned int old_code = bat->code;

		bat->usb_report_held = false;
		bat->usb_ma = bat->present ? bat->usb_report_ma : 0;
		if (!bat->stopping) {
			d2186_update_target(bat);
			mod_delayed_work(system_freezable_wq, &bat->grant_work, 0);
		}
		changed = true;
		dev_dbg(dev, "USB budget: %u mA applied after sleep (limit %u -> %u mA)\n",
			bat->usb_ma, d2186_code_ma(old_code), d2186_code_ma(bat->code));
	}
	mutex_unlock(&bat->grant_lock);

	if (changed)
		power_supply_changed(bat->chg_psy);
}

static const struct dev_pm_ops apple_d2186_battery_pm_ops = {
	.prepare = apple_d2186_battery_prepare,
	.complete = apple_d2186_battery_complete,
};

static struct platform_driver apple_d2186_battery_driver = {
	.probe = apple_d2186_battery_probe,
	.shutdown = apple_d2186_battery_shutdown,
	.driver = {
		.name = "apple-d2186-battery",
		.of_match_table = apple_d2186_battery_of_match,
		.pm = &apple_d2186_battery_pm_ops,
	},
};
module_platform_driver(apple_d2186_battery_driver);

MODULE_IMPORT_NS("IIO_CONSUMER");
MODULE_DESCRIPTION("Apple D2186 PMIC battery and USB charger driver");
MODULE_LICENSE("GPL");
