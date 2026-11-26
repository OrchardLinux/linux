// SPDX-License-Identifier: GPL-2.0-or-later

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/string.h>

#include "tsl2583.h"

static void ct821_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 510,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 131072U, factor);
}

static void ct821_gain_factor_zero_test(struct kunit *test)
{
	u32 factor = 1;

	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(65536, 0,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(0, 255,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1U, factor);
}

static void ct821_gain_factor_overflow_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_ASSERT_EQ(test, 1U << 24, factor);
	KUNIT_EXPECT_EQ(test, -ERANGE,
			apple_als_calculate_gain_factor(factor, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
}

static void ct819_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	/*
	 * The gain ratios of the factory record of an iPad 7 fold into the
	 * nominal 8x, 16x and 128x gain steps of the part.
	 */
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 2010,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 514560U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 521,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1047210U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 1862,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 7616816U, factor);
}

/*
 * A calibration with round numbers: 8x gain and 400 ms (148 cycles of
 * 2.7 ms) as the reference, a1/b1 = 1/2 up to a ch1/ch0 ratio of 0.4, a2/b2 =
 * 0.8/1.2 above it, fudge 204/255 = 0.8, and the iPad 7 gain ratios as above.
 */
static const struct apple_als_calibration ct819_test_cal = {
	.lux_fudge = 204,
	.integration_time = 400,
	.gain = 1,
	.a_coeff1 = 65536,
	.b_coeff1 = 131072,
	.upper_range = 26214,
	.a_coeff2 = 52429,
	.b_coeff2 = 78643,
	.gain_factor = {
		{ 65536, 65536 },
		{ 514560, 514560 },
		{ 1047210, 1047210 },
		{ 7616816, 7616816 },
	},
};

#define CT819_TEST_CYCLE_US	2700
#define CT819_TEST_REF_CYCLES	148
#define CT819_TEST_UNITY_TRIM	1000

static int ct819_test_lux(u32 cycles, unsigned int gain, u32 lux_max,
			  u16 ch0, u16 ch1)
{
	return apple_als_calc_lux(&ct819_test_cal, gain, cycles,
				  CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
				  CT819_TEST_UNITY_TRIM, lux_max, ch0, ch1);
}

static void ct819_lux_curve_test(struct kunit *test)
{
	/* ch1/ch0 = 0.1 uses the first segment: (1000 * 1 - 100 * 2) * 0.8 */
	KUNIT_EXPECT_EQ(test, 640,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 100));
	/* ch1/ch0 = 0.5 is past the limit: (1000 * 0.8 - 500 * 1.2) * 0.8 */
	KUNIT_EXPECT_EQ(test, 160,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 500));
	/* Mostly infrared light cannot go negative */
	KUNIT_EXPECT_EQ(test, 0,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 100, 1000));
	KUNIT_EXPECT_EQ(test, 0,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 0, 0));
}

static void ct819_lux_integration_time_test(struct kunit *test)
{
	/* The same light in 50 ms (18 cycles) gives 18/148 of the counts */
	KUNIT_EXPECT_EQ(test, 947,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1480, 148));
	KUNIT_EXPECT_EQ(test, 947,
			ct819_test_lux(18, 1, 5000, 180, 18));
}

static void ct819_lux_gain_test(struct kunit *test)
{
	/* 16x sees 1047210 / 514560 times the counts of 8x */
	KUNIT_EXPECT_EQ(test, 640,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 2, 5000, 2035, 204));
}

static void ct819_lux_saturation_test(struct kunit *test)
{
	/* 1024 counts per cycle, 16 bits at most */
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 65535, 100));
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 65535));
	KUNIT_EXPECT_EQ(test, 1000000, ct819_test_lux(18, 1, 1000000, 18431, 0));
	KUNIT_EXPECT_NE(test, 1000000, ct819_test_lux(18, 1, 1000000, 18430, 0));
	/* One count short of the limit still computes, then clamps */
	KUNIT_EXPECT_EQ(test, 48000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 65535, 60000, 0));
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 60000, 0));
}

static void apple_lux_16bit_limit_test(struct kunit *test)
{
	/* Without a per-cycle limit only the 16 bit range clips (the CT821) */
	KUNIT_EXPECT_EQ(test, 65535,
			apple_als_calc_lux(&ct819_test_cal, 1, 18, CT819_TEST_CYCLE_US,
					   0, CT819_TEST_UNITY_TRIM, 65535, 18431, 0));
	KUNIT_EXPECT_NE(test, 1000000,
			apple_als_calc_lux(&ct819_test_cal, 1, 18, CT819_TEST_CYCLE_US,
					   0, CT819_TEST_UNITY_TRIM, 1000000, 18431, 0));
}

static void ct819_lux_invalid_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, -EINVAL, ct819_test_lux(0, 1, 5000, 100, 10));
	KUNIT_EXPECT_EQ(test, -EINVAL,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 8, 5000, 100, 10));
}

static void ct819_lux_dark_counts_test(struct kunit *test)
{
	struct apple_als_calibration cal = ct819_test_cal;

	cal.ch0_dark_counts = 4;
	cal.ch1_dark_counts = 2;
	/* (1004 - 4) and (102 - 2) are the counts of the first test */
	KUNIT_EXPECT_EQ(test, 640,
			apple_als_calc_lux(&cal, 1, CT819_TEST_REF_CYCLES,
					   CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
					   1000, 5000, 1004, 102));
	/* The dark counts of an unlit sensor leave nothing to report */
	KUNIT_EXPECT_EQ(test, 0,
			apple_als_calc_lux(&cal, 1, CT819_TEST_REF_CYCLES,
					   CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
					   1000, 5000, 3, 2));
}

/*
 * LSCI record sets made up for the tests: the header and each record's
 * placement, orientation, data size and version are set, the record data is
 * a fill byte, and the header's checksum word makes the 16-bit words sum to
 * 0xffff.
 */
#define TEST_LSCI_MAX	256

static size_t test_lsci(u8 *data, size_t len, u8 version, u8 count,
			size_t record_size, u8 record_len,
			const u8 (*position)[2])
{
	size_t rec_len = APPLE_ALS_CAL_HEADER_SIZE + count * record_size;
	u32 sum = 0;
	size_t i;

	memset(data, 0, len);
	data[0] = 1;
	data[1] = version;
	data[2] = count;
	data[3] = APPLE_ALS_CAL_HEADER_SIZE;
	data[4] = rec_len & 0xff;
	data[5] = rec_len >> 8;
	for (i = 0; i < count; i++) {
		u8 *record = data + APPLE_ALS_CAL_HEADER_SIZE + i * record_size;

		memset(record, 0x11 * (i + 1), record_size);
		record[0] = 0;
		record[APPLE_ALS_REC_PLACEMENT] = position[i][0];
		record[APPLE_ALS_REC_ORIENTATION] = position[i][1];
		record[3] = record_len;
		record[4] = 1;
	}
	for (i = 0; i < rec_len; i += 2)
		sum += data[i] | (data[i + 1] << 8);
	sum = (u16)(U16_MAX - (u16)sum);
	data[6] = sum & 0xff;
	data[7] = sum >> 8;

	return rec_len;
}

static const u8 test_one_sensor[][2] = { { 1, 1 } };
/* The iPad Air 2's ADT: als1 (0x49) is 1/6, als2 (0x29) is 1/5 */
static const u8 test_two_sensors[][2] = { { 1, 6 }, { 1, 5 } };

static void lsci_one_record_test(struct kunit *test)
{
	u8 data[TEST_LSCI_MAX];

	/* The iPad mini 2: 72 bytes of records in a 200-byte property */
	KUNIT_ASSERT_EQ(test, 72,
			test_lsci(data, 200, 2, 1, 0x40, 0x40, test_one_sensor));
	KUNIT_EXPECT_EQ(test, 1, apple_als_check_records(data, 200, 2, 0x40));
	KUNIT_EXPECT_EQ(test, 1, apple_als_check_records(data, 72, 2, 0x40));
	/* A single record serves its sensor whatever position it names */
	KUNIT_EXPECT_EQ(test, 0, apple_als_find_record(data, 1, 0x40, 1, 1));
	KUNIT_EXPECT_EQ(test, 0, apple_als_find_record(data, 1, 0x40, 1, 6));

	/* The iPhone 6s's CT821: one 0x54-byte v3 record, 92 bytes */
	KUNIT_ASSERT_EQ(test, 92,
			test_lsci(data, 92, 3, 1, 0x54, 0x52, test_one_sensor));
	KUNIT_EXPECT_EQ(test, 1, apple_als_check_records(data, 92, 3, 0x54));
}

static void lsci_two_records_test(struct kunit *test)
{
	u8 data[TEST_LSCI_MAX];

	/* The iPad Air 2: two records, 136 bytes in a 200-byte property */
	KUNIT_ASSERT_EQ(test, 136,
			test_lsci(data, 200, 2, 2, 0x40, 0x40, test_two_sensors));
	KUNIT_EXPECT_EQ(test, 2, apple_als_check_records(data, 200, 2, 0x40));
	KUNIT_EXPECT_EQ(test, 0, apple_als_find_record(data, 2, 0x40, 1, 6));
	KUNIT_EXPECT_EQ(test, 1, apple_als_find_record(data, 2, 0x40, 1, 5));
	/* iOS's default position matches neither */
	KUNIT_EXPECT_EQ(test, -ENOENT,
			apple_als_find_record(data, 2, 0x40, 1, 1));
	KUNIT_EXPECT_EQ(test, -ENOENT,
			apple_als_find_record(data, 2, 0x40, 2, 5));
}

static void lsci_bad_header_test(struct kunit *test)
{
	u8 data[TEST_LSCI_MAX];

	test_lsci(data, 200, 2, 2, 0x40, 0x40, test_two_sensors);
	/* Too short for the header, the wrong version or record size */
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 7, 2, 0x40));
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 3, 0x40));
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 2, 0x54));
	/* The property ends inside the records */
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 135, 2, 0x40));

	/* One record counted in a two-record length */
	data[2] = 1;
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 2, 0x40));
	data[2] = 0;
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 2, 0x40));
	data[2] = 2;

	/* The records do not start right after the header */
	data[3] = 16;
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 2, 0x40));
	data[3] = APPLE_ALS_CAL_HEADER_SIZE;

	/* Something other than zeros after the records */
	data[150] = 1;
	KUNIT_EXPECT_EQ(test, -EINVAL, apple_als_check_records(data, 200, 2, 0x40));
}

static void lsci_bad_checksum_test(struct kunit *test)
{
	u8 data[TEST_LSCI_MAX];

	test_lsci(data, 200, 2, 2, 0x40, 0x40, test_two_sensors);
	data[8 + 0x40 + 0x20] ^= 1;
	KUNIT_EXPECT_EQ(test, -EBADMSG,
			apple_als_check_records(data, 200, 2, 0x40));
}

static struct kunit_case lsci_record_test_cases[] = {
	KUNIT_CASE(lsci_one_record_test),
	KUNIT_CASE(lsci_two_records_test),
	KUNIT_CASE(lsci_bad_header_test),
	KUNIT_CASE(lsci_bad_checksum_test),
	{}
};

static struct kunit_suite lsci_record_test_suite = {
	.name = "apple-als-lsci-records",
	.test_cases = lsci_record_test_cases,
};

static struct kunit_case ct819_gain_factor_test_cases[] = {
	KUNIT_CASE(ct819_gain_factor_valid_test),
	{}
};

static struct kunit_suite ct819_gain_factor_test_suite = {
	.name = "ct819-gain-factor",
	.test_cases = ct819_gain_factor_test_cases,
};

static struct kunit_case ct819_lux_test_cases[] = {
	KUNIT_CASE(ct819_lux_curve_test),
	KUNIT_CASE(ct819_lux_integration_time_test),
	KUNIT_CASE(ct819_lux_gain_test),
	KUNIT_CASE(ct819_lux_saturation_test),
	KUNIT_CASE(apple_lux_16bit_limit_test),
	KUNIT_CASE(ct819_lux_invalid_test),
	KUNIT_CASE(ct819_lux_dark_counts_test),
	{}
};

static struct kunit_suite ct819_lux_test_suite = {
	.name = "ct819-lux",
	.test_cases = ct819_lux_test_cases,
};

static struct kunit_case ct821_gain_factor_test_cases[] = {
	KUNIT_CASE(ct821_gain_factor_valid_test),
	KUNIT_CASE(ct821_gain_factor_zero_test),
	KUNIT_CASE(ct821_gain_factor_overflow_test),
	{}
};

static struct kunit_suite ct821_gain_factor_test_suite = {
	.name = "ct821-gain-factor",
	.test_cases = ct821_gain_factor_test_cases,
};

kunit_test_suites(&lsci_record_test_suite,
		  &ct819_gain_factor_test_suite,
		  &ct819_lux_test_suite,
		  &ct821_gain_factor_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple ALS calibration records, gain factors and lux computation");
MODULE_LICENSE("GPL");
