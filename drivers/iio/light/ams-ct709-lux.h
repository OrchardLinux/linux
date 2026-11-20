/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AMS_CT709_LUX_H
#define AMS_CT709_LUX_H

#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/overflow.h>
#include <linux/unaligned.h>

/* Fixed exposure: 16x sensor gain and ten integration cycles. */
struct ct709_lux_cal {
	u32 gain_q20[4];
	u16 offset_milli[4];
	s32 coefficient[4];
	u32 exposure;
};

static u32 ct709_gain_step(u16 word)
{
	/* Factory gain transitions are Q8. Normalize the ratio to [1.75, 2.4]. */
	static const u8 num[] = { 1, 1, 5, 2 };
	static const u8 den[] = { 1, 2, 14, 1 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(num); i++) {
		u32 n = word * num[i], d = 256 * den[i];

		if (4 * n >= 7 * d && 5 * n <= 12 * d)
			return DIV_ROUND_CLOSEST_ULL((u64)n << 20, d);
	}
	return DIV_ROUND_CLOSEST_ULL((u64)word * 14 << 12, 5);
}

/* The caller selects an orientation record validated as native type 3. */
static int ct709_lux_parse(const u8 *p, size_t size, struct ct709_lux_cal *cal)
{
	u32 usec, cycles;
	unsigned int i, j;

	if (size != 3344 || p[0x68] != 3 || p[16] < 4 || p[16] > 8)
		return -EINVAL;
	usec = get_unaligned_le16(p + 10) * 1000;
	cycles = (usec >> 10) > 694 ? 255 : ((u64)(usec >> 2) * 0x05e4be11) >> 36;
	cal->exposure = cycles * get_unaligned_le16(p + 8);
	if (!cal->exposure)
		return -EINVAL;
	for (i = 0; i < 4; i++) {
		const u8 *gains = p + 0x1c + 20 * i;
		u32 gain = 1U << 20;

		for (j = 4; j < p[16]; j++) {
			u32 step = ct709_gain_step(get_unaligned_le16(gains + 2 * j));
			u64 next = ((u64)gain * step + BIT(19)) >> 20;

			if (!next || next > U32_MAX)
				return -ERANGE;
			gain = next;
		}
		cal->gain_q20[i] = gain;
		cal->offset_milli[i] = get_unaligned_le16(gains - 3);
		cal->coefficient[i] = (s32)get_unaligned_le32(p + 0x6c + 4 * i);
	}
	return 0;
}

static int ct709_millilux(const struct ct709_lux_cal *cal,
			  const u16 channels[4], int *value)
{
	const u64 divisor = 2550ULL * 1000000;
	const u64 ceiling = 150000000ULL * divisor;
	s64 count[4], sum = 0;
	unsigned int i;

	/* The frame is CRGB, while the calibration record is RGBC. */
	for (i = 0; i < 4; i++) {
		count[i] = ((u64)channels[(i + 1) % 4] * cal->gain_q20[i] *
			    1000 + BIT(19)) >> 20;
		if (channels[1])
			count[i] += cal->offset_milli[i];
	}
	if (!channels[1])
		count[0] = div_s64(max_t(s64, 0, 108 * count[3] -
					97 * count[1] - 112 * count[2]), 100);
	for (i = 0; i < 4; i++) {
		s64 term;

		if (check_mul_overflow(count[i], (s64)cal->coefficient[i], &term) ||
		    check_add_overflow(sum, term, &sum))
			return -ERANGE;
	}
	if (sum <= 0)
		*value = 0;
	else if ((u64)sum > div_u64(ceiling, cal->exposure))
		*value = 150000000;
	else
		*value = DIV_ROUND_CLOSEST_ULL((u64)sum * cal->exposure, divisor);
	return 0;
}

#endif
