/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_D2355_CURRENT_H
#define _APPLE_D2355_CURRENT_H

#include "apple_d2355_calibration.h"

/*
 * Convert an already admitted target, using a separately qualified range.
 * This does not program the input-disable bit or the current-limit register.
 * Both outputs stay unchanged on error; the caller owns hardware sequencing.
 */
static inline int d2355_current_encode(const struct d2355_calibration *cal,
				      unsigned int range, unsigned int target_ma,
				      u8 *code, bool *disable_input)
{
	s32 calibrated_q16;
	unsigned int calibrated_ma;
	u8 encoded;
	int ret;

	if (!code || !disable_input)
		return -EINVAL;
	if (target_ma > 2549)
		return -ERANGE;
	ret = d2355_calibration_apply(cal, range, target_ma * D2355_CAL_ONE,
				       &calibrated_q16);
	if (ret)
		return ret;
	calibrated_ma = calibrated_q16 / D2355_CAL_ONE;
	if (calibrated_ma < 80)
		encoded = 8;
	else if (calibrated_ma > 2549)
		encoded = 255;
	else
		encoded = calibrated_ma / 10;
	*code = encoded;
	*disable_input = calibrated_ma < 80;
	return 0;
}
#endif
