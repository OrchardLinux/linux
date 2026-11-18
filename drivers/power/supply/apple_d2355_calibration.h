/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_D2355_CALIBRATION_H
#define _APPLE_D2355_CALIBRATION_H

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define D2355_CAL_POINTS 5
#define D2355_CAL_RANGES 4
#define D2355_CAL_HV_BYTES 84
#define D2355_CAL_ONE 65536

struct d2355_calibration {
	u32 selector;
	s32 measured[D2355_CAL_RANGES][D2355_CAL_POINTS];
};

/* Property words are private per-unit Q16 values, not register settings. */
static inline int d2355_calibration_parse_hv(const void *data, size_t len,
					   struct d2355_calibration *cal)
{
	struct d2355_calibration parsed;
	const u8 *bytes = data;
	unsigned int range, point;
	u32 word;

	if (!data || !cal || len != D2355_CAL_HV_BYTES)
		return -EINVAL;
	parsed.selector = get_unaligned_le32(bytes);
	if (parsed.selector < 1 || parsed.selector > 3)
		return -EINVAL;
	for (range = 0; range < D2355_CAL_RANGES; range++) {
		for (point = 0; point < D2355_CAL_POINTS; point++) {
			word = get_unaligned_le32(bytes + 4 + 20 * range + 4 * point);
			if (!word || word > S32_MAX)
				return -EINVAL;
			if (point && word <= (u32)parsed.measured[range][point - 1])
				return -EINVAL;
			parsed.measured[range][point] = word;
		}
	}
	*cal = parsed;
	return 0;
}

/* Native public nominal landmarks; high ranges depend on format selector. */
static const s32 d2355_calibration_nominal[4][5] = {
	{ 100, 500, 1000, 2100, 2400 },
	{ 100, 480, 500, 1000, 2000 },
	{ 100, 500, 1000, 1500, 2000 },
	{ 100, 480, 1000, 1500, 2000 },
};

/*
 * No hardware or source admission here. The caller must supply a qualified
 * active range; this helper never selects voltage modes or authorizes current.
 * Reject arithmetic outside the representable domain instead of wrapping it.
 */
static inline int d2355_calibration_apply(const struct d2355_calibration *cal,
					unsigned int range, s32 target_q16,
					s32 *result_q16)
{
	const s32 *nominal, *measured;
	s64 numerator, slope, candidate, low, high;
	s32 dx, dy;
	unsigned int point, index = 3, percent, correction = 0;

	if (!cal || !result_q16 || range >= D2355_CAL_RANGES ||
	    cal->selector < 1 || cal->selector > 3)
		return -EINVAL;
	if (target_q16 < 0 || target_q16 > 2549 * D2355_CAL_ONE)
		return -ERANGE;
	nominal = d2355_calibration_nominal[range ? cal->selector : 0];
	measured = cal->measured[range];
	for (point = 0; point < D2355_CAL_POINTS; point++)
		if (measured[point] <= 0 ||
		    (point && measured[point] <= measured[point - 1]))
			return -EINVAL;

	candidate = target_q16;
	if (target_q16 >= nominal[0] * D2355_CAL_ONE) {
		for (point = 1; point < D2355_CAL_POINTS; point++) {
			if (nominal[point] * D2355_CAL_ONE > target_q16) {
				index = point - 1;
				break;
			}
		}
		dx = (nominal[index + 1] - nominal[index]) * D2355_CAL_ONE;
		dy = measured[index + 1] - measured[index];
		numerator = (s64)dx * D2355_CAL_ONE;
		slope = div_s64(numerator, dy);
		if (slope <= 0 || slope > S32_MAX)
			return -ERANGE;
		candidate = (s64)nominal[index] * D2355_CAL_ONE +
			    ((slope * ((s64)target_q16 - measured[index])) >> 16);
		if (candidate < 0 || candidate > S32_MAX)
			return -ERANGE;
	}

	percent = cal->selector == 2 ? 20 : 15;
	/* Preserve native integer-mA rounding before the strict bound checks. */
	low = div_s64(((100 - percent) * (s64)target_q16) >> 16, 100) *
	      D2355_CAL_ONE;
	high = div_s64(((100 + percent) * (s64)target_q16) >> 16, 100) *
	       D2355_CAL_ONE;
	if (candidate <= low || candidate >= high)
		candidate = target_q16;

	if (target_q16 >= 100 * D2355_CAL_ONE && cal->selector != 2 &&
	    candidate > 10 * D2355_CAL_ONE) {
		if ((u32)target_q16 >> 18 > 124) {
			correction = 20;
			if ((u32)target_q16 >> 19 > 124) {
				correction = 40;
				if ((u32)target_q16 >> 18 > 374)
					correction = target_q16 > 2000 * D2355_CAL_ONE ? 70 : 60;
			}
		}
		candidate -= (10 + correction) * D2355_CAL_ONE;
	}
	if (candidate < 0 || candidate > S32_MAX)
		return -ERANGE;
	*result_q16 = candidate;
	return 0;
}
#endif
