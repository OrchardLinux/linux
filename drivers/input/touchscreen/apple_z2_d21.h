/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple Z2 touchscreen: the iPhone 8 Plus (D21, D211) start-up, interrupt
 * and recovery backend of apple_z2.c.
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * This is the D21 driver of the contributor's working D21 kernel ("JADE":
 * OrchardLinux linux c2a38a5db829, blob bce4e6c701a7; iPhOmarchy 7952dce
 * GPU-WIP/jade/kernel/0021, by Dugs), which starts this controller where
 * the iPhone 7 Plus path of apple_z2.c does not. apple_z2.c hands a D21 to
 * it at probe, remove, suspend and resume. Its code is carried as it runs
 * there, with its identifiers prefixed apple_z2_d21 so they live beside
 * the shared driver's, its module parameters named d21_*, its SMC calls
 * through wrappers, and its match tables and registration left to
 * apple_z2.c. The defaults are the ones its working load line sets.
 */
#ifndef _APPLE_Z2_D21_H
#define _APPLE_Z2_D21_H

#include <linux/bitrev.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hex.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mfd/macsmc.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

/*
 * The D21 core rail is an SMC function. Without the SMC driver the rail
 * stays as the bootloader left it.
 */
static int apple_z2_d21_smc_write(struct apple_smc *smc, smc_key key,
				  const void *buf, size_t size)
{
#if IS_ENABLED(CONFIG_MFD_MACSMC)
	return apple_smc_write(smc, key, buf, size);
#else
	return -ENODEV;
#endif
}

static int apple_z2_d21_smc_rw(struct apple_smc *smc, smc_key key,
			       const void *wbuf, size_t wsize, void *rbuf,
			       size_t rsize)
{
#if IS_ENABLED(CONFIG_MFD_MACSMC)
	return apple_smc_rw(smc, key, wbuf, wsize, rbuf, rsize);
#else
	return -ENODEV;
#endif
}

#define APPLE_Z2_D21_NUM_FINGERS_OFFSET      16
#define APPLE_Z2_D21_FINGERS_OFFSET          24
#define APPLE_Z2_D21_TOUCH_STARTED           3
#define APPLE_Z2_D21_TOUCH_MOVED             4
#define APPLE_Z2_D21_CMD_READ_INTERRUPT_DATA 0xEB
#define APPLE_Z2_D21_REPLY_INTERRUPT_DATA    0xE1
#define APPLE_Z2_D21_FRAME_INPUT_REPORT      0x90
#define APPLE_Z2_D21_INTERFACE_GRAPE         BIT(1)
#define APPLE_Z2_D21_HID_REPORT_ID           0x76
#define APPLE_Z2_D21_D111_TOUCH_REPORT       0x44
#define APPLE_Z2_D21_D11_STATUS_REPORT       0x50
#define APPLE_Z2_D21_REPORT_ENABLE           0xaf
#define APPLE_Z2_D21_D11_REPORT_ENABLE       0xe2
/*
 * "Multitouch high level" power state. Its single channel sits at offset 1 of
 * the report and takes Unknown, Active, Ready, Rsv, UILock, FaceDet, Rsv,
 * Rsv, PreReady, Anticip or Diag.
 */
#define APPLE_Z2_D21_REPORT_HIGH_LEVEL       0x72
#define APPLE_Z2_D21_HIGH_LEVEL_ACTIVE       1
#define APPLE_Z2_D21_J172_HEADER_SIZE         32
#define APPLE_Z2_D21_J172_CONTACT_SIZE        48
#define APPLE_Z2_D21_J172_MAX_CONTACTS        32
#define APPLE_Z2_D21_J172_FLOAT_FORMAT        BIT(1)
#define APPLE_Z2_D21_REPORT_SURFACE           0xd9
#define APPLE_Z2_D21_SURFACE_DESCRIPTOR_SIZE  16
#define APPLE_Z2_D21_CMD_DEVICE_INFO         0xE2
#define APPLE_Z2_D21_CMD_REPORT_INFO         0xE3
#define APPLE_Z2_D21_CMD_CTRL_WRITE_SHORT    0xE4
#define APPLE_Z2_D21_CMD_CTRL_READ_SHORT     0xE6
#define APPLE_Z2_D21_CMD_CTRL_READ_LONG      0xE7
#define APPLE_Z2_D21_CMD_LAST                0xE1
#define APPLE_Z2_D21_CMD_WAKE                0xEE
#define APPLE_Z2_D21_CMD_SIZE                16
#define APPLE_Z2_D21_RAW_XFER_MAX_SIZE       64
#define APPLE_Z2_D21_GEN2_MIN_RESULT_SIZE    64
#define APPLE_Z2_D21_HBPP_CMD_BLOB           0x3001
#define APPLE_Z2_D21_FW_MAGIC                0x5746325A
#define APPLE_Z2_D21_RX_BUF_SIZE             4000
#define APPLE_Z2_D21_D111_CAL_SIZE           0x400
#define APPLE_Z2_D21_LOAD_COMMAND_INIT_PAYLOAD        0
#define APPLE_Z2_D21_LOAD_COMMAND_SEND_BLOB           1
#define APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION    2
#define APPLE_Z2_D21_LOAD_COMMAND_WAIT_IRQ            3
#define APPLE_Z2_D21_LOAD_COMMAND_SET_CONFIG          4
#define APPLE_Z2_D21_LOAD_COMMAND_RAW_XFER            5
#define APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION_V2 6
#define APPLE_Z2_D21_RAW_XFER_RX_BIT_REVERSE BIT(0)
#define APPLE_Z2_D21_CAL_PROP_NAME                    "apple,z2-cal-blob"
#define APPLE_Z2_D21_ORB_GAP_CAL_PROP_NAME            "apple,z2-orb-gap-cal-blob"
#define APPLE_Z2_D21_ORB_FORCE_CAL_PROP_NAME          "apple,z2-orb-force-cal-blob"
#define APPLE_Z2_D21_SHAPE_ACCEL_CAL_PROP_NAME        "apple,z2-shape-accel-cal-blob"
#define APPLE_Z2_D21_PROX_CAL_PROP_NAME               "apple,z2-prox-cal-blob"

#define APPLE_Z2_D21_FW_CONFIG_WORDS         11
#define APPLE_Z2_D21_FW_CONFIG_MIN_DMA       BIT(0)
#define APPLE_Z2_D21_FW_CONFIG_Z2_DELAY      BIT(1)
#define APPLE_Z2_D21_FW_CONFIG_CS_DELAY      BIT(2)
#define APPLE_Z2_D21_FW_CONFIG_CPHA          BIT(3)
#define APPLE_Z2_D21_FW_CONFIG_CPOL          BIT(4)
#define APPLE_Z2_D21_FW_CONFIG_BOOT_TIMEOUT  BIT(8)

enum apple_z2_d21_protocol_state {
	APPLE_Z2_D21_STATE_OFF,
	APPLE_Z2_D21_STATE_POWERED,
	APPLE_Z2_D21_STATE_SPI_CONFIGURED,
	APPLE_Z2_D21_STATE_BOOT_IRQ,
	APPLE_Z2_D21_STATE_HBPP_READY,
	APPLE_Z2_D21_STATE_FIRMWARE_READY,
	APPLE_Z2_D21_STATE_DEVICE_INFO,
	APPLE_Z2_D21_STATE_RUNTIME,
	APPLE_Z2_D21_STATE_REPORTS_READY,
};

enum apple_z2_d21_variant {
	APPLE_Z2_D21_VARIANT_GENERIC,
	APPLE_Z2_D21_VARIANT_J172,
	APPLE_Z2_D21_VARIANT_D11,
	APPLE_Z2_D21_VARIANT_D111,
	APPLE_Z2_D21_VARIANT_D21,
};

struct apple_z2_d21_chip_info {
	const char *name;
	enum apple_z2_d21_variant variant;
};

struct apple_z2_d21 {
	struct spi_device *spidev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_ana_gpio;
	struct gpio_desc *power_ldo_gpio;
	struct gpio_desc *display_sync_gpio;
	struct gpio_desc *display_sync1_gpio;
	struct pinctrl *j172_sync_pinctrl;
	struct pinctrl_state *j172_sync_active_state;
	struct regulator *hv_supply;
	struct regulator *core_supply;
	struct regulator *ldo_supply;
	struct apple_smc *smc;
	struct clk *clk;
	struct input_dev *input_dev;
	struct completion boot_irq;
	struct mutex io_lock; /* Serializes command and IRQ transfers. */
	enum apple_z2_d21_variant variant;
	bool no_init_ack;
	bool booted;
	bool clk_enabled;
	bool platform_powered;
	bool upload_irq_masked;
	bool runtime_frame_valid;
	bool runtime_queue_empty;
	bool runtime_control_reply;
	bool irq_type_changed;
	unsigned int saved_irq_type;
	bool surface_descriptor_valid;
	bool j172_sync_pinctrl_active;
	bool hv_enabled;
	bool core_enabled;
	bool ldo_enabled;
	bool smc_rail_up;
	enum apple_z2_d21_protocol_state protocol_state;
	unsigned int bpw16_min_len;
	unsigned int dma_min_len;
	unsigned int z2_inter_packet_delay_us;
	unsigned int z2_cs_delay_us;
	unsigned int boot_timeout_ms;
	s16 sensor_max_x;
	s16 sensor_min_x;
	s16 sensor_max_y;
	s16 sensor_min_y;
	int index_parity;
	struct touchscreen_properties props;
	const char *fw_name;
	u8 *tx_buf;
	u8 *rx_buf;
	u8 ack_buf[2];
	struct delayed_work d21_poll;
	u16 last_blob_ack;
	/* D21 palm rejection: one latch bit per MT slot (slots = 10 for D21). */
	DECLARE_BITMAP(d21_palm_latch, 10);
	/* Consecutive oversized frames per slot (sustained-palm filter). */
	u8 d21_big_frames[10];
	/*
	 * D21 runtime recovery (Orchard 79f5e69/45784d7 port): the rearm
	 * ladder, the no-frames watchdog, and the IRQ-enabled tracker.
	 */
	struct delayed_work d21_rearm;
	struct delayed_work d21_watch;
	unsigned int d21_rearm_attempts;
	unsigned int d21_boot_fails;
	unsigned int d21_dead_restarts;
	unsigned long d21_last_frame;
	/* Last interrupt edge: the controller asking for attention. */
	unsigned long d21_last_irq;
	/* A contact is currently reported down. */
	bool d21_contact_down;
	bool d21_recovering;
	bool stopping;
	bool irq_on;
	/* Boot-edge storm guard + boot deadline (see apple_z2_d21_irq). */
	unsigned int boot_edges;
	unsigned long boot_edge_jiffy;
	unsigned long boot_deadline;
};

static bool apple_z2_d21_is_j172(const struct apple_z2_d21 *z2)
{
	return z2->variant == APPLE_Z2_D21_VARIANT_J172;
}

static bool apple_z2_d21_is_d11(const struct apple_z2_d21 *z2)
{
	return z2->variant == APPLE_Z2_D21_VARIANT_D11;
}

static bool apple_z2_d21_is_d111(const struct apple_z2_d21 *z2)
{
	return z2->variant == APPLE_Z2_D21_VARIANT_D111;
}

static bool apple_z2_d21_is_d21(const struct apple_z2_d21 *z2)
{
	return z2->variant == APPLE_Z2_D21_VARIANT_D21;
}

static bool apple_z2_d21_is_iphone7_plus(const struct apple_z2_d21 *z2)
{
	return apple_z2_d21_is_d11(z2) || apple_z2_d21_is_d111(z2) ||
	       apple_z2_d21_is_d21(z2);
}

struct apple_z2_d21_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_z2_d21_hbpp_blob_hdr {
	__le16 cmd;
	__le16 len;
	__le32 addr;
	__le16 checksum;
};

struct apple_z2_d21_fw_hdr {
	__le32 magic;
	__le32 version;
};

struct apple_z2_d21_fw_calibration {
	__le32 address;
	__le32 max_size;
	__le32 provider;
};

enum apple_z2_d21_calibration_provider {
	APPLE_Z2_D21_CAL_MULTI_TOUCH,
	APPLE_Z2_D21_CAL_ORB_GAP,
	APPLE_Z2_D21_CAL_ORB_FORCE,
	APPLE_Z2_D21_CAL_SHAPE_ACCEL,
};

static void apple_z2_d21_apply_z2_delays(struct apple_z2_d21 *z2);
static void apple_z2_d21_post_z2_xfer_delay(struct apple_z2_d21 *z2);
static void apple_z2_d21_set_gpio(struct gpio_desc *gpio, int value);
static void apple_z2_d21_z2_checksum(u8 *cmd, unsigned int len);
static bool apple_z2_d21_z2_checksum_valid(const u8 *cmd, unsigned int len);
static int apple_z2_d21_boot(struct apple_z2_d21 *z2);
static void apple_z2_d21_platform_power_off(struct apple_z2_d21 *z2);
static void apple_z2_d21_start_recovery(struct apple_z2_d21 *z2, int error);
static void apple_z2_d21_irq_off_nosync(struct apple_z2_d21 *z2);

static bool apple_z2_d21_float_to_integer(const u8 *raw, s64 *value)
{
	u32 bits = get_unaligned_le32(raw);
	u32 exponent = (bits >> 23) & 0xff;
	u64 significand;
	u64 magnitude;
	int shift;

	if (exponent == 0xff)
		return false;
	if (!exponent) {
		*value = 0;
		return true;
	}

	significand = BIT_ULL(23) | (bits & GENMASK(22, 0));
	shift = exponent - 127 - 23;
	if (shift >= 0) {
		if (shift > 7)
			return false;
		magnitude = significand << shift;
	} else if (-shift >= 64) {
		magnitude = 0;
	} else {
		magnitude = DIV_ROUND_CLOSEST_ULL(significand,
						  BIT_ULL(-shift));
	}

	if (magnitude > S32_MAX)
		return false;
	*value = bits & BIT(31) ? -(s64)magnitude : magnitude;
	return true;
}

static bool apple_z2_d21_scale_j172_coord(const u8 *raw, s16 maximum,
				      s16 minimum, unsigned int pixels,
				      unsigned int *position)
{
	s64 coordinate;
	s64 numerator;
	s64 denominator;

	if (maximum <= minimum || !apple_z2_d21_float_to_integer(raw, &coordinate))
		return false;

	/* The report uses micrometres; the descriptor uses 0.01 mm. */
	numerator = coordinate - 10LL * minimum;
	denominator = 10LL * (maximum - minimum);
	if (numerator <= 0) {
		*position = 0;
	} else if (numerator >= denominator) {
		*position = pixels;
	} else {
		*position = DIV_ROUND_CLOSEST_ULL((u64)numerator * pixels,
						  denominator);
	}

	return true;
}

static bool apple_z2_d21_scale_iphone7_coord(s16 coordinate, s16 maximum,
					 s16 minimum, unsigned int pixels,
					 unsigned int *position)
{
	s64 numerator;
	s64 denominator;

	if (maximum <= minimum)
		return false;

	numerator = coordinate - minimum;
	denominator = maximum - minimum;
	if (numerator <= 0)
		*position = 0;
	else if (numerator >= denominator)
		*position = pixels;
	else
		*position = DIV_ROUND_CLOSEST_ULL((u64)numerator * pixels,
						  denominator);

	return true;
}

static void apple_z2_d21_parse_j172_touches(struct apple_z2_d21 *z2,
					const u8 *msg, size_t msg_len)
{
	unsigned int contact_offset;
	unsigned int contact_bytes;
	unsigned int nfingers;
	unsigned int x;
	unsigned int y;
	unsigned int i;
	const u8 *contact;
	int slot;
	bool active;
	bool coords_valid;

	if (msg_len < APPLE_Z2_D21_J172_HEADER_SIZE)
		return;

	contact_offset = msg[2] + msg[14];
	contact_bytes = get_unaligned_le16(msg + 16);
	nfingers = msg[22];
	if (msg[2] < APPLE_Z2_D21_J172_HEADER_SIZE ||
	    !(get_unaligned_le16(msg + 12) & APPLE_Z2_D21_J172_FLOAT_FORMAT) ||
	    nfingers > APPLE_Z2_D21_J172_MAX_CONTACTS ||
	    contact_bytes != nfingers * APPLE_Z2_D21_J172_CONTACT_SIZE ||
	    contact_offset > msg_len ||
	    contact_bytes > msg_len - contact_offset) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid J172 touch packet layout len=%zu header=%u contacts=%u bytes=%u\n",
				     msg_len, msg[2], nfingers, contact_bytes);
		return;
	}

	if (nfingers && !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "J172 touch packet without surface descriptor\n");
		return;
	}

	contact = msg + contact_offset;
	for (i = 0; i < nfingers; i++, contact += APPLE_Z2_D21_J172_CONTACT_SIZE) {
		slot = input_mt_get_slot_by_key(z2->input_dev, contact[0]);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev,
				 "unable to get slot for J172 finger\n");
			continue;
		}

		active = contact[1] == APPLE_Z2_D21_TOUCH_STARTED ||
			 contact[1] == APPLE_Z2_D21_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, active))
			continue;

		coords_valid = apple_z2_d21_scale_j172_coord(contact + 4,
							 z2->sensor_max_x,
							 z2->sensor_min_x,
							 z2->props.max_x, &x);
		if (coords_valid)
			coords_valid = apple_z2_d21_scale_j172_coord(contact + 8,
								 z2->sensor_max_y,
								 z2->sensor_min_y,
								 z2->props.max_y, &y);
		if (!coords_valid) {
			dev_warn_ratelimited(&z2->spidev->dev,
					     "invalid J172 touch coordinates\n");
			continue;
		}

		touchscreen_report_pos(z2->input_dev, &z2->props, x,
				       z2->props.max_y - y, true);
	}

	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

/*
 * Per-frame logging (irq-rx / irq-pkt / irq-pkt valid / touch parse) is pure
 * diagnostics: at ~30 Hz it floods the console and the dmesg ring.  Off by
 * default; runtime-togglable so a stream-death watch can enable it only
 * around the interesting moment.
 */
static bool z2_verbose;
module_param_named(d21_verbose, z2_verbose, bool, 0644);
MODULE_PARM_DESC(d21_verbose, "Per-frame IRQ/packet/touch-parse logging (diagnostic)");

/*
 * D21: minimum pressure (+26) for a 0x44 record to count as an active
 * contact.  Hover/rest/blips sit at 0-59 (occasional one-frame flickers to
 * ~86) while every deliberate press ramps past 100 (taps peak ~338, holds to
 * 1382), so 128 rejects the hover band without losing a tap (a real press's
 * first 1-2 onset frames can dip below 128, but the next frame is well past
 * it).  1 accepts any nonzero pressure; 0 passes everything (raw A/B).
 */
static unsigned int d21_pres_min = 64;
module_param_named(d21_pres_min, d21_pres_min, uint, 0644);
MODULE_PARM_DESC(d21_pres_min,
		 "D21: minimum 0x44 pressure for an active contact (128=default, 0=raw)");

/*
 * D21: palm rejection by contact size.  tool_major (+12) measured on the
 * 2026-10-31 omarchy-a install: a real finger (drag) reads 662-757, a flat
 * pressed palm reads 3000-4360 (median 3319, 205 records) with a 4-record
 * onset fringe at 537-1000.  Any frame of a contact that reaches tool_max
 * latches that contact as a palm for its whole life - a per-frame check
 * alone would let the onset fringe flicker through - and the latch clears
 * when the contact lifts or vanishes from the report.  The finger never
 * approaches the threshold, so a finger stays live while a palm rests on
 * the panel (the iOS behavior).  0 disables the check.
 *
 * Calibration round 2 (raw 0x44 records, same day): a HARD finger press
 * ramps tool_major up to 1812 (x1214 minor, area 2.2M) while a LIGHT palm
 * press lands at 1934x1552 (area 3.0M) and a medium palm at 1455-2174 -
 * major alone cannot separate them.  The record AREA (major*minor) does:
 * every observed palm frame is 2.84M-8.0M, every observed finger <= 2.2M.
 * d21_tool_area_max latches a contact whose record area reaches it; the
 * latches OR together.  Palm fragments (657x555, 1763x549 strips) can
 * still flash a single frame - a resting palm (pressure 0) never reaches
 * the filter at all, so the fragment blips only occur under deliberate
 * mashing.  0 disables the area check.
 */
static unsigned int d21_tool_max = 3000;
module_param_named(d21_tool_max, d21_tool_max, uint, 0644);
MODULE_PARM_DESC(d21_tool_max,
		 "D21: tool_major that latches a contact as a palm (0=off)");
static unsigned int d21_tool_area_max = 5000000;
module_param_named(d21_tool_area_max, d21_tool_area_max, uint, 0644);
MODULE_PARM_DESC(d21_tool_area_max,
		 "D21: tool_major*tool_minor area that latches a palm (0=off)");

/*
 * A real finger press opens with a 3-4 frame size spike (measured on this
 * panel: tool_major 2265-2482, area 3.5-3.7M) before settling to about
 * 1550 / 1.85M.  The latch used to fire on the first oversized frame, so
 * that onset spike latched the entire press and the tap was discarded -
 * "touch works but not all the time".  Require the oversized condition to
 * persist for latch_frames consecutive frames before latching: a finger's
 * onset spike is transient and resets the counter, while a resting palm
 * stays big and latches.  An unambiguous giant contact (tool_area_hard,
 * the recurring ~13M phantom/palm blob) still latches immediately so it
 * never flashes a click.  0 for latch_frames restores the old
 * latch-on-first-frame behavior; 0 for tool_area_hard disables the
 * immediate path.
 */
static unsigned int d21_latch_frames = 8;
module_param_named(d21_latch_frames, d21_latch_frames, uint, 0644);
MODULE_PARM_DESC(d21_latch_frames,
		 "D21: consecutive oversized frames before latching a palm (0=first)");
static unsigned int d21_tool_area_hard = 12000000;
module_param_named(d21_tool_area_hard, d21_tool_area_hard, uint, 0644);
MODULE_PARM_DESC(d21_tool_area_hard,
		 "D21: area that latches a palm immediately (0=off)");

/*
 * D21: the touch digital-core rail is SMC key 'rBK3' (ADT function-power_ldo
 * = 75:pKW4('rBK3', 0x5020000), provider = the smc-pmu nub of the SMC IOP;
 * the Asahi macsmc driver is bound on T8015 and apple_smc_write is exported).
 * power_on / power_off run the SMC FUNCTION call as an RW_KEY transaction
 * with code = (dt_value & 0xffff0000) | enable (decoded from
 * AppleSMCEmbeddedFunctions.cpp, 2026-11-03): ON = 0x05020001,
 * OFF = 0x05020000 (iOS-proven values — the phone does this every screen
 * lock).  A raw 0 write is an INVALID code and trips systemic protection.
 */
static bool d21_smc_cut_rail = true;
module_param_named(d21_smc_cut_rail, d21_smc_cut_rail, bool, 0644);
MODULE_PARM_DESC(d21_smc_cut_rail,
		 "D21: RW smc_off_val (0x05020000 = iOS off) to rBK3 on power_off");
static unsigned int d21_smc_on_val = 0x05020001;
module_param_named(d21_smc_on_val, d21_smc_on_val, uint, 0644);
MODULE_PARM_DESC(d21_smc_on_val,
		 "D21: rBK3 function ON code (0x05020001 = dt_value | enable=1)");
static unsigned int d21_smc_off_val = 0x05020000;
module_param_named(d21_smc_off_val, d21_smc_off_val, uint, 0644);
MODULE_PARM_DESC(d21_smc_off_val,
		 "D21: rBK3 function OFF code (0x05020000 = the raw dt_value, iOS-proven)");
static unsigned int d21_smc_on_retries;
module_param_named(d21_smc_on_retries, d21_smc_on_retries, uint, 0644);
MODULE_PARM_DESC(d21_smc_on_retries,
		 "D21: retry OFF->1s->ON this many times when the post-ON rBK3 state != 01 (0 = probe-and-log only)");

/*
 * D21 runtime recovery knobs (Orchard 79f5e69 port, 2026-11-05).  See the
 * recovery block below apple_z2_d21_poll_work() for the full design.
 */
static bool d21_recover = true;
module_param_named(d21_recover, d21_recover, bool, 0644);
MODULE_PARM_DESC(d21_recover,
		 "D21: in-driver runtime recovery (watchdog + resync ladder + full restart)");
static unsigned int d21_quiet_ms = 15000;
module_param_named(d21_quiet_ms, d21_quiet_ms, uint, 0644);
MODULE_PARM_DESC(d21_quiet_ms,
		 "D21: start recovery after this many ms without a valid frame (0=off)");

/*
 * D21: read the rBK3 FUNCTION state via the RW_KEY transaction.  A bare
 * apple_smc_read() of a function key always fails (SMC result != 0 -> EIO);
 * the RW form's read phase returns the state byte.  Write phase is inert
 * (we pass the ON code).  Returns the state byte, or -1 if the RW itself
 * failed.  Measured 2026-11-04: OFF -> 00, ON -> 01, 01 <=> healthy roll.
 */
static int d21_smc_rail_state(struct apple_z2_d21 *z2)
{
	u32 wv = d21_smc_on_val;
	u8 rb[4] = {};
	int err;

	if (!z2->smc)
		return -1;
	err = apple_z2_d21_smc_rw(z2->smc, SMC_KEY(rBK3), &wv, 4, rb, 4);
	if (err != 4)   /* success returns the read size */
		return -1;
	return rb[0];
}

/*
 * D21: latch RELEASE threshold.  The controller reuses tracking ids, so a
 * latched palm/phantom blob can be immediately followed by a real finger on
 * the same id.  Every palm/ghost frame measures area >= 2.8M while a real
 * finger is ~0.8-1.5M, so when a frame falls below this area the slot's
 * latch is cleared and the finger reports normally.  Without it the reused
 * id stays latched and the real tap is silently dropped (observed
 * 2026-11-02: an 11M phantom at the bottom followed by a "5" tap on the same
 * id, area 1.4M, suppressed).  0 disables the release.
 */
static unsigned int d21_tool_area_release = 2000000;
module_param_named(d21_tool_area_release, d21_tool_area_release, uint, 0644);
MODULE_PARM_DESC(d21_tool_area_release,
		 "D21: area below which a latched contact is released (0=off)");

static void apple_z2_d21_parse_touches(struct apple_z2_d21 *z2,
				   const u8 *msg, size_t msg_len)
{
	s16 abs_x;
	s16 abs_y;
	s16 orientation;
	bool coords_valid;
	bool any_active = false;
	unsigned int x;
	unsigned int y;
	int i;
	int nfingers;
	int slot;
	int slot_valid;
	struct apple_z2_d21_finger *fingers;

	if (msg_len >= APPLE_Z2_D21_J172_HEADER_SIZE &&
	    msg[0] == APPLE_Z2_D21_HID_REPORT_ID && msg[3] == 4) {
		apple_z2_d21_parse_j172_touches(z2, msg, msg_len);
		return;
	}

	if (msg_len <= APPLE_Z2_D21_NUM_FINGERS_OFFSET)
		return;
	nfingers = msg[APPLE_Z2_D21_NUM_FINGERS_OFFSET];
	if (z2_verbose)
		dev_info(&z2->spidev->dev,
			 "touch parse: nfingers=%d msg_len=%zu\n",
			 nfingers, msg_len);
	if (msg_len < APPLE_Z2_D21_FINGERS_OFFSET ||
	    nfingers > (msg_len - APPLE_Z2_D21_FINGERS_OFFSET) /
		       sizeof(*fingers)) {
		dev_warn(&z2->spidev->dev,
			 "invalid touch packet length %zu for %d fingers\n",
			 msg_len, nfingers);
		return;
	}
	if (apple_z2_d21_is_iphone7_plus(z2) && nfingers &&
	    !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "iPhone 7 Plus touch packet without surface descriptor\n");
		return;
	}
	fingers = (struct apple_z2_d21_finger *)(msg + APPLE_Z2_D21_FINGERS_OFFSET);
	/* D21 palm latch: slots seen in THIS frame (vanish sweep below). */
	unsigned long d21_seen = 0;
	for (i = 0; i < nfingers; i++) {
		slot = input_mt_get_slot_by_key(z2->input_dev, fingers[i].finger);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev, "unable to get slot for finger\n");
			continue;
		}
		if (apple_z2_d21_is_d21(z2)) {
			/*
			 * D21 0x44, measured 2026-10-30 with labeled seqtest
			 * runs: the state byte does NOT separate touch from
			 * hover (its meaning even varies with fw state).  A
			 * deliberate sustained press reads state=4 with real
			 * pressure (393-1382), a resting finger reads state=4
			 * pressure=0, newly detected contacts read 3 for one
			 * frame, light contacts read 1/2, lifts read 0.  The
			 * "Touch bit" (bit0) marks the transient edge events,
			 * so filtering on it passes phantom/hand blinks while
			 * dropping every continuous-press record - the driver
			 * literally discarded the user's finger.
			 *
			 * Pressure (+26) is the invariant discriminator.  Measured landscape
			 * (seqtest, 229 contact runs): hover/rest/blips sit at 0-59
			 * (rest band 12-16, one-frame flickers to 59), every deliberate
			 * press and corner tap ramps >= 100 (taps peak ~338, holds to
			 * 1382) - so the default threshold is 64.  state==0
			 * is the explicit lift.  d21_pres_min=0 restores the
			 * raw pass-everything behavior for A/B.
			 */
			unsigned int f_area = (unsigned int)
				le16_to_cpu(fingers[i].tool_major) *
				le16_to_cpu(fingers[i].tool_minor);
			/*
			 * Release a latch when the contact shrinks to finger
			 * scale: a reused tracking id must not keep a real
			 * finger suppressed after a palm/phantom blob.
			 */
			if (d21_tool_area_release &&
			    f_area < d21_tool_area_release) {
				clear_bit(slot, z2->d21_palm_latch);
				z2->d21_big_frames[slot] = 0;
			}
			/*
			 * big_frames must only count frames that would
			 * forward: a slow-onset strong touch ramps its tool
			 * size while the pressure is still below pres_min,
			 * and counting those frames latched the whole tap
			 * before its pressure ever crossed the gate
			 * (measured 2026-11-04: top taps tool_major
			 * 3797-4332 with the pressure ramping, latched after
			 * 8 frames with zero evdev events, while the one
			 * finger-scale tap, tool 736, forwarded instantly).
			 * Hold the counter on sub-gate frames instead of
			 * resetting it, or a long ramp would never reach
			 * latch_frames.  The giant-blob class still
			 * hard-latches on tool_area_hard, whose pressure is
			 * irrelevant.
			 */
			bool big = (d21_tool_max &&
				    le16_to_cpu(fingers[i].tool_major) >=
					    d21_tool_max) ||
				   (d21_tool_area_max &&
				    f_area >= d21_tool_area_max);
			bool forwards = le16_to_cpu(fingers[i].pressure) >=
					d21_pres_min;

			if (d21_tool_area_hard &&
			    f_area >= d21_tool_area_hard) {
				/* Unambiguous giant blob: latch now. */
				set_bit(slot, z2->d21_palm_latch);
				z2->d21_big_frames[slot] = 255;
			} else if (big && forwards) {
				if (z2->d21_big_frames[slot] < 255)
					z2->d21_big_frames[slot]++;
				if (z2->d21_big_frames[slot] >=
				    d21_latch_frames)
					set_bit(slot, z2->d21_palm_latch);
			} else if (!big) {
				z2->d21_big_frames[slot] = 0;
			}
			if (fingers[i].state == 0) {
				clear_bit(slot, z2->d21_palm_latch);
				z2->d21_big_frames[slot] = 0;
			}
			__set_bit(slot, &d21_seen);
			slot_valid = fingers[i].state != 0 &&
				     le16_to_cpu(fingers[i].pressure) >=
					     d21_pres_min &&
				     !test_bit(slot, z2->d21_palm_latch);
		} else {
			slot_valid = fingers[i].state == APPLE_Z2_D21_TOUCH_STARTED ||
				     fingers[i].state == APPLE_Z2_D21_TOUCH_MOVED;
		}
		if (slot_valid)
			any_active = true;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, slot_valid))
			continue;
		if (apple_z2_d21_is_iphone7_plus(z2)) {
			abs_x = (s16)le16_to_cpu(fingers[i].abs_x);
			abs_y = (s16)le16_to_cpu(fingers[i].abs_y);
			coords_valid = apple_z2_d21_scale_iphone7_coord(abs_x,
								    z2->sensor_max_x,
								    z2->sensor_min_x,
								    z2->props.max_x, &x);
			if (coords_valid)
				coords_valid = apple_z2_d21_scale_iphone7_coord(abs_y,
									    z2->sensor_max_y,
									    z2->sensor_min_y,
									    z2->props.max_y,
									    &y);
			if (!coords_valid) {
				dev_warn_ratelimited(&z2->spidev->dev,
						     "invalid iPhone 7 Plus touch coordinates\n");
				continue;
			}
			touchscreen_report_pos(z2->input_dev, &z2->props, x,
					       z2->props.max_y - y,
					       true);
		} else if (apple_z2_d21_is_d21(z2)) {
			/*
			 * D21 0x44 records carry the contact position in
			 * SIGNED surface-descriptor units: the edge "ghost"
			 * contacts sit at exactly min_x (-224 on this panel),
			 * real contacts span min..max (-224..6608 x,
			 * -236..11904 y per the d9 read), so the earlier
			 * per-mille interpretation was a coincidence of
			 * small values.  Scale like the iPhone 7 Plus path;
			 * the controller clamps out-of-surface contacts to
			 * the border, and so do we.
			 */
			s16 sx = (s16)le16_to_cpu(fingers[i].abs_x);
			s16 sy = (s16)le16_to_cpu(fingers[i].abs_y);
			bool coords_valid;

			coords_valid = apple_z2_d21_scale_iphone7_coord(sx,
								    z2->sensor_max_x,
								    z2->sensor_min_x,
								    z2->props.max_x, &x);
			if (coords_valid)
				coords_valid = apple_z2_d21_scale_iphone7_coord(sy,
									    z2->sensor_max_y,
									    z2->sensor_min_y,
									    z2->props.max_y,
									    &y);
			if (!coords_valid) {
				dev_warn_ratelimited(&z2->spidev->dev,
						     "invalid D21 touch coordinates\n");
				continue;
			}
			touchscreen_report_pos(z2->input_dev, &z2->props, x,
					       y, true);
		} else {
		touchscreen_report_pos(z2->input_dev, &z2->props,
				       le16_to_cpu(fingers[i].abs_x),
				       le16_to_cpu(fingers[i].abs_y),
				       true);
	}
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MAJOR,
				 le16_to_cpu(fingers[i].tool_major));
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MINOR,
				 le16_to_cpu(fingers[i].tool_minor));
		orientation = (s16)le16_to_cpu(fingers[i].orientation);
		if (apple_z2_d21_is_iphone7_plus(z2))
			orientation = (s16)(0x4000 - orientation);
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION, orientation);
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 le16_to_cpu(fingers[i].touch_major));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 le16_to_cpu(fingers[i].touch_minor));
	}
	if (apple_z2_d21_is_d21(z2)) {
		/* Contacts that vanished from the report release their latch. */
		for (i = 0; i < 10; i++)
			if (!test_bit(i, &d21_seen)) {
				clear_bit(i, z2->d21_palm_latch);
				z2->d21_big_frames[i] = 0;
			}
		/*
		 * Remember whether this frame carried a contact: the
		 * watchdog uses it to tell "the controller owes us a frame"
		 * from "nobody is touching the screen".
		 */
		z2->d21_contact_down = any_active;
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

static void apple_z2_d21_dispatch_frame(struct apple_z2_d21 *z2, const u8 *payload,
				    size_t payload_len)
{
	const u8 *report;
	u16 report_len;

	/*
	 * D21 is the same HBPP generation as Orchard's working J71b, whose
	 * runtime delivers contacts as 0x90 "input report" frames carrying HID
	 * report 0x76 on the "grape" interface.  The legacy iPhone7 dispatch
	 * below would silently drop those, so handle the HID frame shape for
	 * D21 explicitly and log it -- no such frame has ever been observed on
	 * this controller, so seeing one is itself the result.
	 */
	if (apple_z2_d21_is_d21(z2) && payload_len >= 4 &&
	    payload[0] == APPLE_Z2_D21_FRAME_INPUT_REPORT) {
		report_len = get_unaligned_le16(payload + 2);
		dev_info(&z2->spidev->dev,
			 "D21 HID input frame: iface=%#x len=%u report=%#x\n",
			 payload[1], report_len,
			 payload_len > 4 ? payload[4] : 0);
		if ((payload[1] & APPLE_Z2_D21_INTERFACE_GRAPE) &&
		    report_len && report_len == payload_len - 4) {
			report = payload + 4;
			if (report[0] == APPLE_Z2_D21_HID_REPORT_ID)
				apple_z2_d21_parse_touches(z2, report, report_len);
		}
		return;
	}

	/* iPhone 7 Plus Gen2 packets carry the legacy report header directly. */
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		if (payload_len < APPLE_Z2_D21_FINGERS_OFFSET)
			return;

		/*
		 * D111 hardware traces identify 0x44 as its touch report.  No D11
		 * runtime capture currently establishes a report identifier, so
		 * retain its length-based dispatch until that protocol is verified.
		 */
		if (apple_z2_d21_is_d111(z2) &&
		    payload[0] != APPLE_Z2_D21_D111_TOUCH_REPORT)
			return;

		apple_z2_d21_parse_touches(z2, payload, payload_len);
		return;
	}

	if (payload_len < 4 || payload[0] != APPLE_Z2_D21_FRAME_INPUT_REPORT)
		return;

	report_len = get_unaligned_le16(payload + 2);
	if (report_len != payload_len - 4) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid input report length %u for payload %zu\n",
				     report_len, payload_len);
		return;
	}

	if (!(payload[1] & APPLE_Z2_D21_INTERFACE_GRAPE) || !report_len)
		return;

	report = payload + 4;
	if (report[0] != APPLE_Z2_D21_HID_REPORT_ID)
		return;

	apple_z2_d21_parse_touches(z2, report, report_len);
}

static bool apple_z2_d21_gen2_packet_valid(const u8 *buf, size_t buf_len,
				       u8 counter, bool strict_counter)
{
	bool counter_valid;
	u16 payload_len;
	u16 checksum;
	int i;

	if (buf_len < 7)
		return false;

	counter_valid = strict_counter ? buf[1] == counter :
					buf[1] == 1 || buf[1] == 2;
	payload_len = get_unaligned_le16(buf + 2);
	if (((buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) & 0xff) ||
	    (buf[0] & 0xfe) != 0xea || !counter_valid ||
	    payload_len < 2 || payload_len + 5 > buf_len)
		return false;

	checksum = get_unaligned_le16(buf + 5 + payload_len - 2);
	for (i = 0; i < payload_len - 2; i++)
		checksum -= buf[5 + i];

	return !checksum;
}

/*
 * D21 class controllers (HBPP 0x103, same generation as J71b) do not treat the
 * data phase of a long read as a dummy transfer: the transmitted bytes are the
 * ACK for the queued report, so a plain 0xa5 fill never releases it. Send a
 * well formed command frame instead, and run reads longer than one frame with
 * CPHA cleared.
 */
static int apple_z2_d21_gen1_read_frame(struct apple_z2_d21 *z2, u8 *buf, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = len,
		.bits_per_word = 8,
	};
	u32 saved_mode = z2->spidev->mode;
	bool changed = len > APPLE_Z2_D21_CMD_SIZE;
	int error, restore;

	if (changed) {
		z2->spidev->mode = saved_mode & ~SPI_CPHA;
		error = spi_setup(z2->spidev);
		if (error)
			return error;
	}

	error = spi_sync_transfer(z2->spidev, &xfer, 1);

	if (changed) {
		z2->spidev->mode = saved_mode;
		restore = spi_setup(z2->spidev);
		if (!error)
			error = restore;
	}

	return error;
}

/*
 * Run the data phase of a D21 read: transmit the acknowledgement frame from
 * tx_buf while receiving into rx_buf, at the reduced clock the controller
 * expects once a transfer spans more than one command frame.
 */
static int apple_z2_d21_gen1_exchange(struct apple_z2_d21 *z2, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = len,
		.bits_per_word = 8,
		.speed_hz = 500000,
	};
	u32 saved_mode = z2->spidev->mode;
	bool changed = len > APPLE_Z2_D21_CMD_SIZE;
	int error, restore;

	if (changed) {
		z2->spidev->mode = saved_mode & ~SPI_CPHA;
		error = spi_setup(z2->spidev);
		if (error)
			return error;
	}

	error = spi_sync_transfer(z2->spidev, &xfer, 1);

	if (changed) {
		z2->spidev->mode = saved_mode;
		restore = spi_setup(z2->spidev);
		if (!error)
			error = restore;
	}

	return error;
}

static void apple_z2_d21_gen1_prepare_report_read(struct apple_z2_d21 *z2, u8 report,
					      u16 len)
{
	size_t bytes = (size_t)len + 5;

	memset(z2->rx_buf, 0, bytes);
	z2->rx_buf[0] = APPLE_Z2_D21_CMD_CTRL_READ_LONG;
	z2->rx_buf[1] = report;
	z2->rx_buf[2] = 1;
	put_unaligned_le16(len, z2->rx_buf + 3);
	apple_z2_d21_z2_checksum(z2->rx_buf, bytes);
}

static bool z2_frame_dump;
module_param_named(d21_frame_dump, z2_frame_dump, bool, 0444);
MODULE_PARM_DESC(d21_frame_dump,
		 "Log the full interrupt-frame payload on every packet (diagnostic)");

/*
 * Tag discipline for the interrupt-data pull.  false (default, kext-faithful):
 * the iOS AppleMultitouchZ2SPI keeps the same [0xEB][seq] for the pull and the
 * data read of ONE frame transaction and toggles 1<->2 only after the frame is
 * fully consumed - empty pulls leave it untouched.  true: the legacy
 * toggle-on-every-pull discipline (including empty pulls), kept for A/B.
 */
static bool z2_tag_always;
module_param_named(d21_tag_always, z2_tag_always, bool, 0444);
MODULE_PARM_DESC(d21_tag_always,
		 "Toggle the pull sequence tag on every read (legacy) or only after a consumed frame (kext-faithful, default)");

/*
 * Checksum placement of the D21 interrupt data-read command.  true (default,
 * per the kext disassembly): the [EB][seq][01] command is stretched to the
 * full transfer with its checksum in the LAST two bytes.  false: the legacy
 * 16-byte command checksum at bytes 14-15.
 */
static bool z2_pull_cksum_end = true;
module_param_named(d21_pull_cksum_end, z2_pull_cksum_end, bool, 0444);
MODULE_PARM_DESC(d21_pull_cksum_end,
		 "D21: data-read command checksum at the transfer end (kext) or at byte 14 (legacy)");

static int apple_z2_d21_read_packet(struct apple_z2_d21 *z2)
{
	struct spi_transfer xfer = { };
	bool strict_counter;
	bool d21_resynced = false;
	int error;
	size_t pkt_len;
	size_t wire_len;
	u16 payload_len;
	u8 counter;

	z2->runtime_frame_valid = false;
	z2->runtime_queue_empty = false;
	z2->runtime_control_reply = false;
	memset(z2->tx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_D21_CMD_READ_INTERRUPT_DATA;
	counter = z2->index_parity + 1;
	z2->tx_buf[1] = counter;
	/*
	 * Kext-faithful (AppleMultitouchZ2SPI) interrupt pull: [EB][seq][00]
	 * with 0xAA filler in bytes 3..13.  Byte 2 = 0x00 means "present a
	 * frame"; the data read that follows uses 0x01.  The legacy driver
	 * sent 0x01 here, which the controller parses as a data read for a
	 * frame that was never presented.
	 *
	 * The seq toggles only after a frame is fully consumed (see the end
	 * of this function); empty pulls leave it untouched.
	 */
	z2->tx_buf[2] = 0x00;
	memset(&z2->tx_buf[3], 0xaa, APPLE_Z2_D21_CMD_SIZE - 3 - 2);
	if (z2_tag_always)
		z2->index_parity = !z2->index_parity;
	apple_z2_d21_z2_checksum(z2->tx_buf, APPLE_Z2_D21_CMD_SIZE);
	xfer.tx_buf = z2->tx_buf;
	xfer.rx_buf = z2->rx_buf;
	xfer.len = APPLE_Z2_D21_CMD_SIZE;

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	apple_z2_d21_post_z2_xfer_delay(z2);

	if (z2_verbose)
		dev_info(&z2->spidev->dev, "irq-rx: %*ph\n", 16, z2->rx_buf);

	if (z2->rx_buf[0] != APPLE_Z2_D21_REPLY_INTERRUPT_DATA)
		return 0;

	payload_len = get_unaligned_le16(z2->rx_buf + 1);
	if (apple_z2_d21_is_d21(z2)) {
		/* A zero-length queue header means nothing is pending. */
		if (!payload_len) {
			z2->runtime_queue_empty = true;
			return 0;
		}
		pkt_len = max_t(size_t, APPLE_Z2_D21_CMD_SIZE,
				(size_t)payload_len + 5);
	} else {
		pkt_len = (payload_len + 8) & 0xfffffffc;
	}
	if (pkt_len > APPLE_Z2_D21_RX_BUF_SIZE) {
		dev_warn(&z2->spidev->dev, "packet too large: %zu\n", pkt_len);
		return -EMSGSIZE;
	}
	wire_len = pkt_len;
	if (apple_z2_d21_is_d11(z2))
		wire_len = max_t(size_t, wire_len,
				 APPLE_Z2_D21_GEN2_MIN_RESULT_SIZE);

	if (apple_z2_d21_is_d21(z2)) {
		/*
		 * Data read of the presented frame: [EB][seq][01] with zero
		 * filler, same seq as the pull.  Per the kext disassembly the
		 * command is stretched to the full transfer length with the
		 * checksum in the LAST two bytes (pull_cksum_end); the legacy
		 * placement was bytes 14-15 of a 16-byte command.
		 */
		memset(z2->tx_buf, 0, wire_len);
		z2->tx_buf[0] = APPLE_Z2_D21_CMD_READ_INTERRUPT_DATA;
		z2->tx_buf[1] = counter;
		z2->tx_buf[2] = 1;
		apple_z2_d21_z2_checksum(z2->tx_buf,
				     z2_pull_cksum_end ? wire_len :
							 APPLE_Z2_D21_CMD_SIZE);
		memset(z2->rx_buf, 0, wire_len);
		error = apple_z2_d21_gen1_exchange(z2, wire_len);
	} else if (apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) {
		memset(z2->rx_buf, 0xa5, wire_len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = wire_len;
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
	} else {
		error = spi_read(z2->spidev, z2->rx_buf, pkt_len);
	}
	if (error)
		return error;
	if (apple_z2_d21_is_iphone7_plus(z2))
		apple_z2_d21_post_z2_xfer_delay(z2);

	/*
	 * Orchard 5fb60c0 port (J71b — same HBPP generation as D21): when
	 * the data phase answers with a queue header instead of frame
	 * data, the controller and the driver are a transfer apart.
	 * Ignored, every later read got a header back and touch stayed
	 * dead with the interrupt still firing until the driver was
	 * rebound (omarchy-a#25).  Take it as the next frame's header and
	 * run the data phase for it, once per read.
	 */
	if (apple_z2_d21_is_d21(z2) && !d21_resynced &&
	    z2->rx_buf[0] == APPLE_Z2_D21_REPLY_INTERRUPT_DATA) {
		payload_len = get_unaligned_le16(z2->rx_buf + 1);
		if (payload_len) {
			dev_info(&z2->spidev->dev,
				 "D21: header in the data phase (%u bytes pending): resynchronising\n",
				 payload_len);
			d21_resynced = true;
			pkt_len = max_t(size_t, APPLE_Z2_D21_CMD_SIZE,
					(size_t)payload_len + 5);
			if (pkt_len > APPLE_Z2_D21_RX_BUF_SIZE)
				return -EMSGSIZE;
			wire_len = pkt_len;
			memset(z2->tx_buf, 0, wire_len);
			z2->tx_buf[0] = APPLE_Z2_D21_CMD_READ_INTERRUPT_DATA;
			z2->tx_buf[1] = counter;
			z2->tx_buf[2] = 1;
			apple_z2_d21_z2_checksum(z2->tx_buf,
					     z2_pull_cksum_end ? wire_len :
								 APPLE_Z2_D21_CMD_SIZE);
			memset(z2->rx_buf, 0, wire_len);
			error = apple_z2_d21_gen1_exchange(z2, wire_len);
			if (error)
				return error;
		}
	}

	if (apple_z2_d21_is_d21(z2) && !z2->rx_buf[0]) {
		/* Nothing pending on the queue: keep the tag, emit nothing. */
		z2->runtime_queue_empty = true;
		return 0;
	}
	if (apple_z2_d21_is_d21(z2) && wire_len >= APPLE_Z2_D21_CMD_SIZE &&
	    z2->rx_buf[0] == APPLE_Z2_D21_CMD_LAST &&
	    apple_z2_d21_z2_checksum_valid(z2->rx_buf, APPLE_Z2_D21_CMD_SIZE)) {
		/*
		 * A control reply while service is pending, not a coordinate
		 * frame. Keep the tag and let the bounded drain retry.
		 */
		z2->runtime_control_reply = true;
		return 0;
	}

	if (z2_verbose)
		dev_info(&z2->spidev->dev, "irq-pkt: len=%zu %*ph\n", pkt_len,
			 (int)min_t(size_t, pkt_len, 64U), z2->rx_buf);
	if (z2_frame_dump) {
		size_t off;

		for (off = 0; off < pkt_len; off += 48)
			dev_info(&z2->spidev->dev, "irq-frame+%zu: %*ph\n",
				 off, (int)min_t(size_t, pkt_len - off, 48U),
				 z2->rx_buf + off);
	}

	/* D11 touch reports do not follow the requested alternating tag. */
	strict_counter = !apple_z2_d21_is_d11(z2) ||
			 (pkt_len > 5 && z2->rx_buf[5] ==
					 APPLE_Z2_D21_D11_STATUS_REPORT);
	z2->runtime_frame_valid =
		apple_z2_d21_gen2_packet_valid(z2->rx_buf, pkt_len, counter,
					   strict_counter);
	/*
	 * D21 recovery: remember the last genuinely valid frame (gen2
	 * header + checksum).  The half-boot's status frames never set
	 * this, which is what makes the no-frames watchdog unambiguous.
	 */
	if (z2->runtime_frame_valid) {
		z2->d21_last_frame = jiffies;
		z2->d21_dead_restarts = 0;
	}
	if (!z2->runtime_frame_valid) {
		if (z2_verbose)
			dev_info(&z2->spidev->dev,
				 "irq-pkt invalid: counter=%u strict=%d\n",
				 counter, strict_counter);
		return 0;
	}
	if (z2_verbose)
		dev_info(&z2->spidev->dev, "irq-pkt valid: report=0x%02x\n",
			 z2->rx_buf[5]);

	if (!z2_tag_always)
		z2->index_parity = !z2->index_parity;

	payload_len = get_unaligned_le16(z2->rx_buf + 2);
	apple_z2_d21_dispatch_frame(z2, z2->rx_buf + 5, payload_len - 2);

	return 0;
}

static void apple_z2_d21_reset_protocol(struct apple_z2_d21 *z2)
{
	z2->protocol_state = APPLE_Z2_D21_STATE_OFF;
	z2->index_parity = 0;
	z2->runtime_frame_valid = false;
	z2->surface_descriptor_valid = false;
}

static int apple_z2_d21_advance_protocol(struct apple_z2_d21 *z2,
				     enum apple_z2_d21_protocol_state expected,
				     enum apple_z2_d21_protocol_state next)
{
	if (z2->protocol_state != expected)
		return -EPROTO;

	z2->protocol_state = next;
	return 0;
}

static void apple_z2_d21_z2_checksum(u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];
	put_unaligned_le16(checksum, cmd + len - 2);
}

static bool apple_z2_d21_z2_checksum_valid(const u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	if (len < 2)
		return false;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];

	return get_unaligned_le16(cmd + len - 2) == checksum;
}

static void apple_z2_d21_set_spi_delay(struct spi_delay *delay,
				   unsigned int usecs)
{
	if (usecs > U16_MAX)
		usecs = U16_MAX;

	delay->value = usecs;
	delay->unit = SPI_DELAY_UNIT_USECS;
}

static unsigned int apple_z2_d21_effective_cs_delay_us(struct apple_z2_d21 *z2)
{
	/* J172 firmware expresses this field in ns; Linux SPI uses us here. */
	return apple_z2_d21_is_j172(z2) ? 10 : z2->z2_cs_delay_us;
}

static void apple_z2_d21_apply_z2_delays(struct apple_z2_d21 *z2)
{
	unsigned int cs_delay_us = apple_z2_d21_effective_cs_delay_us(z2);
	unsigned int inactive_us = apple_z2_d21_is_j172(z2) ? 10 :
				  z2->z2_inter_packet_delay_us;

	if (cs_delay_us) {
		apple_z2_d21_set_spi_delay(&z2->spidev->cs_setup, cs_delay_us);
		apple_z2_d21_set_spi_delay(&z2->spidev->cs_hold, cs_delay_us);
	}
	if (inactive_us)
		apple_z2_d21_set_spi_delay(&z2->spidev->cs_inactive, inactive_us);
}

static void apple_z2_d21_post_z2_xfer_delay(struct apple_z2_d21 *z2)
{
	unsigned int delay_us = z2->z2_inter_packet_delay_us;

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static void apple_z2_d21_post_ack_delay(struct apple_z2_d21 *z2)
{
	unsigned int delay_us = apple_z2_d21_effective_cs_delay_us(z2);

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static void apple_z2_d21_avoid_short_dma(struct apple_z2_d21 *z2,
				     struct spi_transfer *xfer)
{
	/*
	 * iPhone 7 Plus firmware describes a target-specific minimum
	 * DMA transfer length, not a property of the SPI controller.  Supplying
	 * an RX buffer makes shorter writes full-duplex, which the T8010
	 * controller handles through PIO without changing the wire transfer.
	 */
	if (!apple_z2_d21_is_iphone7_plus(z2) || !z2->dma_min_len ||
	    !xfer->tx_buf || xfer->rx_buf ||
	    xfer->len >= z2->dma_min_len)
		return;

	memset(z2->rx_buf, 0, xfer->len);
	xfer->rx_buf = z2->rx_buf;
}

static int apple_z2_d21_z2_xfer_locked_delay(struct apple_z2_d21 *z2,
					 const char *tag, bool post_delay)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = APPLE_Z2_D21_CMD_SIZE,
	};
	int error;

	apple_z2_d21_z2_checksum(z2->tx_buf, APPLE_Z2_D21_CMD_SIZE);
	apple_z2_d21_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error) {
		dev_err(&z2->spidev->dev, "%s transfer failed: %d\n",
			tag, error);
		return error;
	}
	if (post_delay)
		apple_z2_d21_post_z2_xfer_delay(z2);

	dev_dbg(&z2->spidev->dev, "%s tx=%*ph rx=%*ph\n", tag,
		APPLE_Z2_D21_CMD_SIZE, z2->tx_buf, APPLE_Z2_D21_CMD_SIZE,
		z2->rx_buf);
	return 0;
}

static int apple_z2_d21_z2_xfer_locked(struct apple_z2_d21 *z2, const char *tag)
{
	return apple_z2_d21_z2_xfer_locked_delay(z2, tag, true);
}

static int apple_z2_d21_z2_cmd_locked_delay(struct apple_z2_d21 *z2, u8 cmd,
					u8 report, const char *tag,
					bool post_delay)
{
	memset(z2->tx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	z2->tx_buf[0] = cmd;
	z2->tx_buf[1] = report;

	return apple_z2_d21_z2_xfer_locked_delay(z2, tag, post_delay);
}

static int apple_z2_d21_z2_cmd_locked(struct apple_z2_d21 *z2, u8 cmd, u8 report,
				  const char *tag)
{
	return apple_z2_d21_z2_cmd_locked_delay(z2, cmd, report, tag, true);
}

static int apple_z2_d21_read_device_info_locked(struct apple_z2_d21 *z2)
{
	u16 max_packet_size;
	u8 interface_version;
	u8 platform_id;
	int error;

	error = apple_z2_d21_z2_cmd_locked_delay(z2, APPLE_Z2_D21_CMD_WAKE, 0,
					     "device-info-wake", false);
	if (error)
		return error;
	usleep_range(2000, 2100);

	error = apple_z2_d21_z2_cmd_locked_delay(z2, APPLE_Z2_D21_CMD_DEVICE_INFO, 0,
					     "device-info", false);
	if (error)
		return error;
	apple_z2_d21_post_z2_xfer_delay(z2);

	error = apple_z2_d21_z2_cmd_locked_delay(z2, APPLE_Z2_D21_CMD_LAST, 0,
					     "device-info-last", false);
	if (error)
		return error;

	if (z2->rx_buf[0] != APPLE_Z2_D21_CMD_DEVICE_INFO ||
	    !apple_z2_d21_z2_checksum_valid(z2->rx_buf, APPLE_Z2_D21_CMD_SIZE))
		return -EPROTO;

	platform_id = z2->rx_buf[1];
	interface_version = z2->rx_buf[2];
	max_packet_size = get_unaligned_le16(z2->rx_buf + 3);

	dev_dbg(&z2->spidev->dev,
		"device info: platform=%u interface=%u max-packet=%u\n",
		 platform_id, interface_version, max_packet_size);
	return apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_FIRMWARE_READY,
					 APPLE_Z2_D21_STATE_DEVICE_INFO);
}

static bool z2_reply_log;
module_param_named(d21_reply_log, z2_reply_log, bool, 0444);
MODULE_PARM_DESC(d21_reply_log,
		 "Log the raw reply frame of every report write (diagnostic)");

static int apple_z2_d21_write_report_locked(struct apple_z2_d21 *z2, u8 report,
					const u8 *data, u8 len,
					const char *tag)
{
	char last_tag[32];
	int error;

	if (len > 11)
		return -EINVAL;

	memset(z2->tx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_D21_CMD_CTRL_WRITE_SHORT;
	z2->tx_buf[1] = report;
	z2->tx_buf[2] = len;
	memcpy(z2->tx_buf + 3, data, len);
	error = apple_z2_d21_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	if (z2_reply_log)
		dev_info(&z2->spidev->dev, "%s: write reply: %*ph\n", tag,
			 APPLE_Z2_D21_CMD_SIZE, z2->rx_buf);

	memset(z2->tx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_D21_CMD_LAST;
	snprintf(last_tag, sizeof(last_tag), "%s-last", tag);
	error = apple_z2_d21_z2_xfer_locked(z2, last_tag);
	if (error)
		return error;

	if (z2_reply_log)
		dev_info(&z2->spidev->dev, "%s: last reply: %*ph\n", tag,
			 APPLE_Z2_D21_CMD_SIZE, z2->rx_buf);

	if (apple_z2_d21_is_d11(z2) &&
	    (z2->rx_buf[0] != APPLE_Z2_D21_CMD_LAST ||
	     !apple_z2_d21_z2_checksum_valid(z2->rx_buf, APPLE_Z2_D21_CMD_SIZE)))
		return -EPROTO;

	return 0;
}

static int apple_z2_d21_enable_d11_reports_locked(struct apple_z2_d21 *z2)
{
	static const u8 enable = 1;

	return apple_z2_d21_write_report_locked(z2, APPLE_Z2_D21_D11_REPORT_ENABLE,
					    &enable, sizeof(enable), "d11-e2");
}

static int apple_z2_d21_read_report_info_locked(struct apple_z2_d21 *z2, u8 report,
					    u8 *status, u16 *len,
					    const char *tag)
{
	char xfer_tag[40];
	unsigned int i;
	int error;

	for (i = 0; i < 2; i++) {
		snprintf(xfer_tag, sizeof(xfer_tag), "%s-info-%u", tag, i + 1);
		error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_REPORT_INFO,
					       report, xfer_tag);
		if (error)
			return error;
	}

	if (z2->rx_buf[0] != APPLE_Z2_D21_CMD_REPORT_INFO ||
	    z2->rx_buf[1] != report ||
	    !apple_z2_d21_z2_checksum_valid(z2->rx_buf, APPLE_Z2_D21_CMD_SIZE))
		return -EPROTO;

	*status = z2->rx_buf[2];
	*len = get_unaligned_le16(z2->rx_buf + 3);
	dev_dbg(&z2->spidev->dev,
		"%s report=%02x status=%u len=%u\n", tag, report,
		*status, *len);

	return 0;
}

static int apple_z2_d21_read_report_locked(struct apple_z2_d21 *z2, u8 report,
				       u16 len, const char *tag)
{
	struct spi_transfer xfer = { };
	char last_tag[32];
	unsigned int data_len;
	int error;

	if (!len || len > APPLE_Z2_D21_RX_BUF_SIZE - 5)
		return -EMSGSIZE;

	memset(z2->tx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_D21_CMD_SIZE);
	z2->tx_buf[0] = len <= 11 ? APPLE_Z2_D21_CMD_CTRL_READ_SHORT :
					 APPLE_Z2_D21_CMD_CTRL_READ_LONG;
	z2->tx_buf[1] = report;
	put_unaligned_le16(len, z2->tx_buf + 3);
	error = apple_z2_d21_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	if (len <= 11) {
		snprintf(last_tag, sizeof(last_tag), "%s-last", tag);
		error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_LAST, 0,
					       last_tag);
		if (!error)
			dev_dbg(&z2->spidev->dev, "%s short read done\n",
				tag);
		return error;
	}

	data_len = len + 5;
	if (apple_z2_d21_is_d21(z2)) {
		apple_z2_d21_gen1_prepare_report_read(z2, report, len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = data_len;
		apple_z2_d21_apply_z2_delays(z2);
		error = apple_z2_d21_gen1_read_frame(z2, z2->rx_buf, data_len);
	} else {
		memset(z2->rx_buf, 0xa5, data_len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = data_len;
		apple_z2_d21_apply_z2_delays(z2);
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
	}
	if (error)
		return error;
	apple_z2_d21_post_z2_xfer_delay(z2);
	dev_info(&z2->spidev->dev, "%s long read len=%u rx=%*ph\n", tag,
		 data_len, min_t(unsigned int, data_len, 32U), z2->rx_buf);

	return 0;
}

static int apple_z2_d21_store_surface_descriptor(struct apple_z2_d21 *z2, u16 len)
{
	const u8 *descriptor = z2->rx_buf + 3;
	unsigned int x_pixels;
	unsigned int y_pixels;
	unsigned int x_res;
	unsigned int y_res;
	s16 max_x;
	s16 min_x;
	s16 max_y;
	s16 min_y;
	u32 width;
	u32 height;

	if (len != APPLE_Z2_D21_SURFACE_DESCRIPTOR_SIZE ||
	    z2->rx_buf[0] != APPLE_Z2_D21_CMD_CTRL_READ_LONG ||
	    z2->rx_buf[1] != APPLE_Z2_D21_REPORT_SURFACE ||
	    (apple_z2_d21_is_d21(z2) ? z2->rx_buf[2] > 1 : z2->rx_buf[2] != 0) ||
	    !apple_z2_d21_z2_checksum_valid(z2->rx_buf, len + 5))
		return -EPROTO;

	width = get_unaligned_le32(descriptor);
	height = get_unaligned_le32(descriptor + 4);
	min_x = (s16)get_unaligned_le16(descriptor + 8);
	max_x = (s16)get_unaligned_le16(descriptor + 12);
	min_y = (s16)get_unaligned_le16(descriptor + 10);
	max_y = (s16)get_unaligned_le16(descriptor + 14);
	if (!width || !height || max_x <= min_x || max_y <= min_y)
		return -EPROTO;

	z2->sensor_max_x = max_x;
	z2->sensor_min_x = min_x;
	z2->sensor_max_y = max_y;
	z2->sensor_min_y = min_y;
	z2->surface_descriptor_valid = true;

	x_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_X) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_X) + 1;
	y_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_Y) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_Y) + 1;
	x_res = max(DIV_ROUND_CLOSEST(x_pixels * 100, width), 1U);
	y_res = max(DIV_ROUND_CLOSEST(y_pixels * 100, height), 1U);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_Y, y_res);
	if (test_bit(ABS_X, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_X, x_res);
	if (test_bit(ABS_Y, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_Y, y_res);

	dev_dbg(&z2->spidev->dev,
		"surface %ux%u, bounds x=%d..%d y=%d..%d, resolution %ux%u units/mm\n",
		 width, height, min_x, max_x, min_y, max_y, x_res, y_res);

	return 0;
}

static int apple_z2_d21_iphone7_plus_init_locked(struct apple_z2_d21 *z2)
{
	static const u8 disable_legacy_reports[] = { 0 };
	u16 surface_len;
	u8 status;
	int error;

	error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_WAKE, 0,
				       "iphone7-plus-wake");
	if (error)
		return error;

	error = apple_z2_d21_read_report_info_locked(z2, APPLE_Z2_D21_REPORT_SURFACE,
						 &status, &surface_len,
						 "iphone7-plus-d9");
	if (error)
		return error;
	if (status || surface_len != APPLE_Z2_D21_SURFACE_DESCRIPTOR_SIZE) {
		if (!apple_z2_d21_is_d21(z2))
			return -EPROTO;
		dev_warn(&z2->spidev->dev,
			 "D21 surface info odd (status=%u len=%u), using DT size\n",
			 status, surface_len);
	} else {
		error = apple_z2_d21_read_report_locked(z2, APPLE_Z2_D21_REPORT_SURFACE,
						    surface_len,
						    "iphone7-plus-d9-read");
		if (error)
			return error;
		error = apple_z2_d21_store_surface_descriptor(z2, surface_len);
		if (error)
			return error;
	}

	error = apple_z2_d21_write_report_locked(z2, APPLE_Z2_D21_REPORT_ENABLE,
					     disable_legacy_reports,
					     sizeof(disable_legacy_reports),
					     "iphone7-plus-af");
	if (error)
		return error;

	dev_dbg(&z2->spidev->dev,
		"iPhone 7 Plus report initialization complete\n");

	return 0;
}

static int apple_z2_d21_j172_init_reads_locked(struct apple_z2_d21 *z2)
{
	u8 status;
	u16 db_len;
	u16 prop_len;
	u16 surface_len;
	unsigned int i;
	int error;

	error = apple_z2_d21_read_report_info_locked(z2, APPLE_Z2_D21_REPORT_SURFACE,
						 &status, &surface_len, "d9");
	if (error)
		return error;
	if (status || surface_len != APPLE_Z2_D21_SURFACE_DESCRIPTOR_SIZE)
		return -EPROTO;

	error = apple_z2_d21_read_report_locked(z2, APPLE_Z2_D21_REPORT_SURFACE,
					    surface_len, "d9-read");
	if (error)
		return error;
	error = apple_z2_d21_store_surface_descriptor(z2, surface_len);
	if (error)
		return error;

	error = apple_z2_d21_read_report_info_locked(z2, 0xdb, &status, &db_len,
						 "db");
	if (error)
		return error;
	if (status || db_len != 0x7d)
		return -EPROTO;

	error = apple_z2_d21_read_report_locked(z2, 0xdb, db_len, "db-read-1");
	if (error)
		return error;

	error = apple_z2_d21_read_report_info_locked(z2, 0x73, &status, &prop_len,
						 "73");
	if (error)
		return error;
	if (status || prop_len != 8)
		return -EPROTO;

	error = apple_z2_d21_read_report_locked(z2, 0x73, prop_len, "73-read-1");
	if (error)
		return error;

	for (i = 2; i <= 3; i++) {
		char db_tag[16];
		char prop_tag[16];

		snprintf(db_tag, sizeof(db_tag), "db-read-%u", i);
		error = apple_z2_d21_read_report_locked(z2, 0xdb, db_len, db_tag);
		if (error)
			return error;

		snprintf(prop_tag, sizeof(prop_tag), "73-read-%u", i);
		error = apple_z2_d21_read_report_locked(z2, 0x73, prop_len,
						    prop_tag);
		if (error)
			return error;
	}

	return 0;
}

static int apple_z2_d21_j172_init_writes_locked(struct apple_z2_d21 *z2)
{
	static const u8 rep9d[] = { 0x01, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00 };
	static const u8 repbf[] = { 0x9d, 0x81, 0x09, 0x00 };
	u8 status;
	u16 len;
	int error;

	error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_WAKE, 0, "j172-wake");
	if (error)
		return error;
	usleep_range(2200, 2300);

	error = apple_z2_d21_read_report_info_locked(z2, 0x9d, &status, &len,
						 "9d");
	if (error)
		return error;
	if (status || len != sizeof(rep9d))
		return -EPROTO;

	error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-1");
	if (error)
		return error;
	usleep_range(50500, 50700);
	error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-2");
	if (error)
		return error;

	error = apple_z2_d21_read_report_info_locked(z2, 0xbf, &status, &len,
						 "bf");
	if (error)
		return error;
	if (status || len != sizeof(repbf))
		return -EPROTO;
	error = apple_z2_d21_write_report_locked(z2, 0xbf, repbf,
					     sizeof(repbf), "j172-bf");
	if (error)
		return error;

	error = apple_z2_d21_read_report_info_locked(z2, 0xaf, &status, &len,
						 "af");
	if (error)
		return error;
	if (status != 2 || len)
		return -EPROTO;

	dev_dbg(&z2->spidev->dev, "J172 report initialization complete\n");
	return apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_RUNTIME,
					 APPLE_Z2_D21_STATE_REPORTS_READY);
}

static irqreturn_t apple_z2_d21_irq(int irq, void *data)
{
	struct apple_z2_d21 *z2 = data;
	unsigned int attempt;
	int error;

	/*
	 * Every edge satisfies a pending wait, including the one the
	 * controller raises after the boot sequence to ask for its
	 * calibration.
	 */
	complete(&z2->boot_irq);
	dev_info_ratelimited(&z2->spidev->dev, "D21: touch IRQ edge, booted=%d\n",
			     z2->booted);

	/*
	 * Liveness for the no-frames watchdog: an interrupt edge means the
	 * controller is asking to be read.  An idle D21 controller does not
	 * ask (measured 2026-11-05: 0 edges in 3 s with no finger), so this
	 * timestamp separates "the controller is quiet" from "the controller
	 * is asking but no frame comes back".
	 */
	z2->d21_last_irq = jiffies;

	if (!z2->booted) {
		/*
		 * Boot-edge storm guard: a dead controller that holds ATN
		 * asserted re-fires this level IRQ ~44k times a second
		 * (measured 2026-11-03), and the handler thread crowds out
		 * the boot thread — a firmware-upload step then never
		 * finishes and the whole boot hangs.  Complete at most once
		 * per jiffy (phase waits still satisfy), and after ~200k
		 * edges (~4.5 s of continuous storm — a healthy boot's
		 * assert phase stays far below) mask the IRQ for good: the
		 * remaining boot waits time out BOUNDED, the boot fails,
		 * and the recovery ladder backs off instead of hanging.
		 */
		z2->boot_edges++;
		if (time_after(jiffies, z2->boot_edge_jiffy)) {
			z2->boot_edge_jiffy = jiffies;
			complete(&z2->boot_irq);
		}
		if (z2->boot_edges == 200000 && z2->irq_on) {
			dev_warn(&z2->spidev->dev,
				 "D21: boot-edge storm (%u edges), masking IRQ until the boot fails\n",
				 z2->boot_edges);
			apple_z2_d21_irq_off_nosync(z2);
		}
		return IRQ_HANDLED;
	} else if (apple_z2_d21_is_d21(z2)) {
		/*
		 * Kext-faithful: EXACTLY ONE frame read per IRQ
		 * (AppleMultitouchSPI::handleInterrupt -> gated
		 * readOneFrameOfData; "the default contract is 1 IRQ = 1
		 * frame").  Do NOT drain in a loop here: the trailing pull
		 * that finds the queue empty is an off-IRQ transaction the
		 * controller never expects - the same transaction class that
		 * wedges it when poll_hz>0 (measured 2026-10-30).  The old
		 * bounded drain survived for hours on quiet boots and killed
		 * the stream in minutes-to-hours otherwise (stochastic).
		 *
		 * The line is LEVEL-LOW: while frames stay queued the line
		 * stays asserted and this handler re-fires, which drains the
		 * queue at the controller's own pace with no extra
		 * transactions.
		 */
		mutex_lock(&z2->io_lock);
		/*
		 * Orchard 79f5e69 frame retries: a failed read is retried
		 * up to 3 attempts, 1-2 ms apart.  These are IRQ-context
		 * reads (the transaction the IRQ asked for), not off-IRQ
		 * pulls.  A persistent failure starts the recovery ladder
		 * (apple_z2_d21_rearm_work): release contacts, mask the
		 * IRQ, bounded resync x3, then a full in-driver restart.
		 */
		for (attempt = 0; attempt < 3; attempt++) {
			if (attempt)
				usleep_range(1000, 2000);
			error = apple_z2_d21_read_packet(z2);
			if (!error)
				break;
		}
		mutex_unlock(&z2->io_lock);
		if (error) {
			dev_warn_ratelimited(&z2->spidev->dev,
					     "runtime packet read failed: %d\n",
					     error);
			apple_z2_d21_start_recovery(z2, error);
		}
	} else {
		mutex_lock(&z2->io_lock);
		error = apple_z2_d21_read_packet(z2);
		mutex_unlock(&z2->io_lock);
		if (error)
			dev_warn_ratelimited(&z2->spidev->dev,
					     "runtime packet read failed: %d\n",
					     error);
	}

	return IRQ_HANDLED;
}

/*
 * Orchard-style runtime poller rate in Hz (0 = off, the default).  See
 * apple_z2_d21_poll_work().  WARNING (measured 2026-10-30): non-zero values
 * WEDGE the D21 controller - off-IRQ pulls return all-zero and the frame
 * stream collapses to ~1 Hz with phantom storms; the wedge survives
 * reload + 45 s rails-off and needs a force-restart, like the dma_min
 * poisoning.  Kept for the record; do not enable on D21.
 */
static unsigned int d21_poll_hz;
module_param_named(d21_poll_hz, d21_poll_hz, uint, 0444);
MODULE_PARM_DESC(d21_poll_hz,
		 "D21: poll the frame queue at N Hz instead of trusting per-frame IRQs "
		 "(0=off). WARNING: non-zero wedges the controller (persists across reloads, "
		 "force-restart to recover)");

/*
 * Orchard-style runtime poller (j71b_z2.ko runs its whole runtime POLLED,
 * with rtpoll callbacks and IRQ_IS_POLLED, rather than trusting per-frame
 * IRQs).
 *
 * The D21 scan engine never stops (0xf8 counters climb continuously), but
 * with the Boot Sequence pokes applied the touch-bit records still arrive on
 * a ~1.93 s delivery gate while idle/hover streams freely - so the gate for
 * touch frames sits somewhere the IRQ path cannot bypass.  When poll_hz>0,
 * re-arm this worker every tick and drain whatever the queue will give us
 * right now, gate or no gate.  An empty pull is one 16-byte transfer; pulls
 * that return nothing leave the seq tag untouched, so this composes with
 * the IRQ path.
 */
static void apple_z2_d21_poll_work(struct work_struct *work)
{
	struct apple_z2_d21 *z2 = container_of(work, struct apple_z2_d21, d21_poll.work);
	unsigned int i;
	int error = 0;

	if (z2->booted) {
		mutex_lock(&z2->io_lock);
		for (i = 0; i < 8; i++) {
			error = apple_z2_d21_read_packet(z2);
			if (error || (!z2->runtime_frame_valid &&
				      !z2->runtime_control_reply))
				break;
		}
		mutex_unlock(&z2->io_lock);
		if (error)
			dev_warn_ratelimited(&z2->spidev->dev,
					     "poll packet read failed: %d\n",
					     error);
	}
	if (z2->booted && d21_poll_hz)
		schedule_delayed_work(&z2->d21_poll,
				      msecs_to_jiffies(1000 / d21_poll_hz));
}

/*
 * ==========================================================================
 * D21 runtime recovery — ported from Orchard 79f5e69 ("Input: apple_z2_d21:
 * Bound and recover the iPad 5 runtime like the N1 driver") for the D21
 * wedges (TOUCH-STATE 1M/1E, 2026-11-04):
 *
 * - Normal service stays kext-faithful: EXACTLY ONE frame read per IRQ on
 *   the level-low line.  Orchard's bounded drain (<=8 frames per IRQ) is
 *   the transaction class measured to WEDGE a healthy D21 controller
 *   (poll_hz, 2026-10-30), so it only runs inside the recovery ladder,
 *   when the runtime is already broken.
 *
 * - Trigger 1 (Orchard): a frame read that still fails after 3 attempts
 *   (1-2 ms apart) starts the ladder.
 * - Trigger 2 (D21-specific): the no-frames watchdog.  The D21 half-boot
 *   is SILENT — rail up, SPI ACKs, preamble 1f 01, pokes err=0, ZERO 0x44
 *   frames — so no read error ever fires; only the ABSENCE of frames
 *   identifies it.  A healthy roll streams 15-25 fps idle, so quiet_ms
 *   (default 15 s) without a single valid frame is unambiguous.
 *   gen2_packet_valid() rejects the half-boot's status frames (first byte
 *   0xe1 fails the 0xea header family), so they never refresh
 *   d21_last_frame.
 *
 * - Ladder: release contacts, mask the IRQ, then a delayed work 1 s later
 *   re-checks liveness with a bounded drain, up to 3 attempts 1 s apart.
 *   If frames return, the IRQ is re-enabled and nothing else happens.
 *   Otherwise: FULL IN-DRIVER RESTART — power_off (which cuts the rBK3
 *   digital-core rail through the SMC: the TRUE power cycle the wedged
 *   core state needs; the old hv/ldo-only rails-off never cut it) then
 *   apple_z2_d21_boot(), reloading firmware and calibration.  This replaces a
 *   module reload without tearing the input device down.
 */
#define APPLE_Z2_D21_REARM_ATTEMPTS	3
#define APPLE_Z2_D21_REARM_DELAY_MS	1000
#define APPLE_Z2_D21_WATCH_PERIOD_MS	5000
#define APPLE_Z2_D21_RESTART_BACKOFF_MS	60000
#define APPLE_Z2_D21_MAX_BOOT_FAILS	5
#define APPLE_Z2_D21_MAX_DEAD_RESTARTS	5

/*
 * The interrupt is requested disabled (IRQF_NO_AUTOEN) and enabled during
 * boot.  Track the enabled state so a path that finds it disabled does not
 * disable it a second time (Orchard 45784d7): the recovery ladder masks it
 * from the IRQ thread itself, and stop/boot/err_stop must stay balanced.
 */
static void apple_z2_d21_irq_on(struct apple_z2_d21 *z2)
{
	if (z2->irq_on)
		return;
	z2->irq_on = true;
	enable_irq(z2->spidev->irq);
}

static void apple_z2_d21_irq_off(struct apple_z2_d21 *z2)
{
	if (!z2->irq_on)
		return;
	z2->irq_on = false;
	disable_irq(z2->spidev->irq);
}

/* Masking from the IRQ thread itself: disable_irq() would wait for us. */
static void apple_z2_d21_irq_off_nosync(struct apple_z2_d21 *z2)
{
	if (!z2->irq_on)
		return;
	z2->irq_on = false;
	disable_irq_nosync(z2->spidev->irq);
}

static void apple_z2_d21_release_contacts(struct apple_z2_d21 *z2)
{
	unsigned int i;

	z2->d21_contact_down = false;
	for (i = 0; i < z2->input_dev->mt->num_slots; i++) {
		input_mt_slot(z2->input_dev, i);
		input_mt_report_slot_inactive(z2->input_dev);
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

/*
 * Enter the ladder: release contacts, mask the IRQ, resync 1 s from now.
 * Callable from the IRQ thread (frame error) and the watchdog (silence).
 */
static void apple_z2_d21_start_recovery(struct apple_z2_d21 *z2, int error)
{
	if (!d21_recover || z2->stopping || !z2->booted || z2->d21_recovering)
		return;

	z2->d21_recovering = true;
	z2->d21_rearm_attempts = 0;
	dev_warn(&z2->spidev->dev,
		 "D21: runtime failure %d, releasing contacts and resynchronising\n",
		 error);
	apple_z2_d21_release_contacts(z2);
	apple_z2_d21_irq_off_nosync(z2);
	schedule_delayed_work(&z2->d21_rearm,
			      msecs_to_jiffies(APPLE_Z2_D21_REARM_DELAY_MS));
}

/*
 * The ladder (Orchard's apple_z2_d21_n1_rearm_work): up to 3 bounded-drain
 * resync attempts 1 s apart, then a full in-driver restart.  Firmware and
 * calibration are only reloaded by that restart.
 */
static void apple_z2_d21_rearm_work(struct work_struct *work)
{
	struct apple_z2_d21 *z2 = container_of(to_delayed_work(work),
					   struct apple_z2_d21, d21_rearm);
	unsigned int i, attempts;
	bool saw_frame;
	int error = 0;

	mutex_lock(&z2->io_lock);
	if (z2->stopping || !z2->booted) {
		z2->d21_recovering = false;
		mutex_unlock(&z2->io_lock);
		return;
	}

	if (z2->d21_rearm_attempts < APPLE_Z2_D21_REARM_ATTEMPTS) {
		z2->d21_rearm_attempts++;
		attempts = z2->d21_rearm_attempts;
		/*
		 * Light rung: a bounded drain, 8 reads at most.  The
		 * controller is already broken here (no valid frame for
		 * quiet_ms, or a hard read error), so the off-IRQ pull
		 * risk that forbids this in the IRQ path does not apply:
		 * a module reload would do the same and worse.
		 *
		 * Track whether ANY read of the rung produced a valid
		 * frame: the queue header/data-phase resync can land a
		 * real frame on one pull and an empty queue on the next,
		 * and runtime_frame_valid only reflects the LAST read
		 * (measured 2026-11-03: a streaming controller was
		 * declared "resync failed" and restart-looped).
		 */
		saw_frame = false;
		for (i = 0; i < 8; i++) {
			error = apple_z2_d21_read_packet(z2);
			if (error)
				break;
			if (z2->runtime_frame_valid)
				saw_frame = true;
			if (!z2->runtime_frame_valid &&
			    !z2->runtime_control_reply)
				break;
		}
		if (!error && saw_frame) {
			/* It came back on its own; no restart needed. */
			z2->d21_last_frame = jiffies;
			z2->d21_rearm_attempts = 0;
			z2->d21_recovering = false;
			apple_z2_d21_irq_on(z2);
			mutex_unlock(&z2->io_lock);
			dev_info(&z2->spidev->dev,
				 "D21: runtime recovered after %u resync attempt(s)\n",
				 attempts);
			schedule_delayed_work(&z2->d21_watch,
					      msecs_to_jiffies(APPLE_Z2_D21_WATCH_PERIOD_MS));
			return;
		}
		dev_warn(&z2->spidev->dev,
			 "D21: resynchronisation %u failed (%d)\n",
			 attempts, error ?: -EIO);
		schedule_delayed_work(&z2->d21_rearm,
				      msecs_to_jiffies(APPLE_Z2_D21_REARM_DELAY_MS));
		mutex_unlock(&z2->io_lock);
		return;
	}

	/* Ladder exhausted: full in-driver restart. */
	z2->booted = false;
	z2->d21_recovering = false;
	mutex_unlock(&z2->io_lock);

	dev_warn(&z2->spidev->dev,
		 "D21: restarting the touch controller in-driver (rBK3 rail cycle + fw/cal reload)\n");
	cancel_delayed_work_sync(&z2->d21_poll);
	gpiod_direction_output(z2->reset_gpio, 1);
	apple_z2_d21_platform_power_off(z2);
	apple_z2_d21_reset_protocol(z2);

	error = apple_z2_d21_boot(z2);
	if (error) {
		if (++z2->d21_boot_fails > APPLE_Z2_D21_MAX_BOOT_FAILS) {
			dev_err(&z2->spidev->dev,
				"D21: giving up auto-recovery after %u failed restarts; reload the module\n",
				z2->d21_boot_fails);
			return;
		}
		dev_err(&z2->spidev->dev,
			"D21: in-driver restart failed: %d (retry in %d s)\n",
			error, APPLE_Z2_D21_RESTART_BACKOFF_MS / 1000);
		schedule_delayed_work(&z2->d21_rearm,
				      msecs_to_jiffies(APPLE_Z2_D21_RESTART_BACKOFF_MS));
		return;
	}
	z2->d21_boot_fails = 0;
	/*
	 * A restart that boots but never delivers a frame (the controller
	 * came up wrong again, e.g. holding ATN asserted) must not cycle
	 * forever: every cycle is a full rail cycle.  Frames clear the
	 * counter (read_packet), so a single healthy frame restarts the
	 * budget.
	 */
	if (++z2->d21_dead_restarts > APPLE_Z2_D21_MAX_DEAD_RESTARTS) {
		dev_err(&z2->spidev->dev,
			"D21: %u in-driver restarts without a frame; giving up auto-recovery, reload the module\n",
			z2->d21_dead_restarts);
		return;
	}
	dev_info(&z2->spidev->dev, "D21: in-driver restart succeeded\n");
}

/*
 * The no-frames watchdog (the D21-specific trigger).  Runs every 5 s while
 * booted; quiet_ms without a single valid frame while the controller owes
 * us one is the half-boot/DEAD signature.
 *
 * "Owes us one" is the whole point of this rework: an idle D21 controller
 * is completely silent - no finger, no frames, no interrupt edges (measured
 * 2026-11-05: 0 edges in 3 s of idleness) - and that silence is normal.
 * Treating it as a failure made the driver recover every 15 s: three
 * off-IRQ resync pulls and a full rBK3 rail cycle, 45 times in one boot
 * (journal of the 2026-11-05 desktop session), which is what froze the
 * phone in the desktop.  A controller that IS talking (interrupt edges
 * inside the window) or that still has a contact down, but delivers no
 * valid frame for quiet_ms, is the real DEAD signature and still recovers.
 */
static void apple_z2_d21_watch_work(struct work_struct *work)
{
	struct apple_z2_d21 *z2 = container_of(to_delayed_work(work),
					   struct apple_z2_d21, d21_watch);

	if (z2->stopping || !z2->booted || !d21_recover || !d21_quiet_ms) {
		dev_info_ratelimited(&z2->spidev->dev,
				     "D21 watch: skipped (stopping=%d booted=%d recover=%d quiet_ms=%u)\n",
				     z2->stopping, z2->booted, d21_recover,
				     d21_quiet_ms);
		return;
	}

	if (z2->d21_recovering)
		return; /* the ladder is active; it re-arms us */

	/*
	 * Nothing is expected: no contact is down and the controller has
	 * not asked to be read inside the window.  Idle is not dead.
	 */
	if (!z2->d21_contact_down &&
	    time_is_before_jiffies(z2->d21_last_irq +
				   msecs_to_jiffies(d21_quiet_ms))) {
		schedule_delayed_work(&z2->d21_watch,
				      msecs_to_jiffies(APPLE_Z2_D21_WATCH_PERIOD_MS));
		return;
	}

	/* Stale = now is PAST last_frame + quiet_ms (time_is_BEFORE_jiffies:
	 * the argument lies before jiffies).  A healthy roll refreshes
	 * d21_last_frame on every frame, so this only fires when the
	 * controller stopped talking. */
	if (time_is_before_jiffies(z2->d21_last_frame +
				   msecs_to_jiffies(d21_quiet_ms))) {
		apple_z2_d21_start_recovery(z2, -ETIMEDOUT);
		return;
	}

	schedule_delayed_work(&z2->d21_watch,
			      msecs_to_jiffies(APPLE_Z2_D21_WATCH_PERIOD_MS));
}

static void apple_z2_d21_set_gpio(struct gpio_desc *gpio, int value)
{
	if (gpio)
		gpiod_set_value_cansleep(gpio, value);
}

static void apple_z2_d21_pulse_gpio_pair(struct gpio_desc *gpio0,
				     struct gpio_desc *gpio1)
{
	apple_z2_d21_set_gpio(gpio0, 1);
	apple_z2_d21_set_gpio(gpio1, 1);
	usleep_range(1000, 2000);
	apple_z2_d21_set_gpio(gpio0, 0);
	apple_z2_d21_set_gpio(gpio1, 0);
	usleep_range(1000, 2000);
}

static int apple_z2_d21_j172_sync_claim_gpios(struct apple_z2_d21 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	if (!z2->display_sync1_gpio) {
		z2->display_sync1_gpio =
			devm_gpiod_get_optional(dev, "display-sync1",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync1_gpio)) {
			error = PTR_ERR(z2->display_sync1_gpio);
			z2->display_sync1_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio) {
		z2->display_sync_gpio =
			devm_gpiod_get_optional(dev, "display-sync",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync_gpio)) {
			error = PTR_ERR(z2->display_sync_gpio);
			z2->display_sync_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio || !z2->display_sync1_gpio)
		return -ENODEV;

	return 0;
}

static int apple_z2_d21_j172_sync_set_inactive(struct apple_z2_d21 *z2)
{
	int error;

	if (z2->j172_sync_pinctrl_active) {
		devm_pinctrl_put(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		z2->j172_sync_active_state = NULL;
		z2->j172_sync_pinctrl_active = false;
	}

	error = apple_z2_d21_j172_sync_claim_gpios(z2);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync1_gpio);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync_gpio);
	if (error)
		return error;

	dev_dbg(&z2->spidev->dev, "display-sync pins inactive\n");
	return 0;
}

static int apple_z2_d21_j172_sync_set_active(struct apple_z2_d21 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	error = apple_z2_d21_j172_sync_set_inactive(z2);
	if (error)
		return error;

	devm_gpiod_put(dev, z2->display_sync1_gpio);
	z2->display_sync1_gpio = NULL;
	devm_gpiod_put(dev, z2->display_sync_gpio);
	z2->display_sync_gpio = NULL;

	z2->j172_sync_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(z2->j172_sync_pinctrl)) {
		error = PTR_ERR(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		goto reclaim_gpios;
	}

	z2->j172_sync_active_state =
		pinctrl_lookup_state(z2->j172_sync_pinctrl,
				     "display-sync-active");
	if (IS_ERR(z2->j172_sync_active_state)) {
		error = PTR_ERR(z2->j172_sync_active_state);
		z2->j172_sync_active_state = NULL;
		goto put_pinctrl;
	}

	error = pinctrl_select_state(z2->j172_sync_pinctrl,
				     z2->j172_sync_active_state);
	if (error)
		goto put_pinctrl;

	z2->j172_sync_pinctrl_active = true;
	dev_dbg(dev, "display-sync pins active\n");
	return 0;

put_pinctrl:
	devm_pinctrl_put(z2->j172_sync_pinctrl);
	z2->j172_sync_pinctrl = NULL;
	z2->j172_sync_active_state = NULL;
reclaim_gpios:
	if (apple_z2_d21_j172_sync_claim_gpios(z2))
		dev_warn(dev, "failed to reclaim display-sync GPIOs\n");
	return error;
}

static void apple_z2_d21_platform_power_off(struct apple_z2_d21 *z2)
{
	int error;

	if (apple_z2_d21_is_iphone7_plus(z2))
		gpiod_set_value_cansleep(z2->reset_gpio, 1);

	if (z2->clk_enabled) {
		clk_disable_unprepare(z2->clk);
		z2->clk_enabled = false;
	}
	if (apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2))
		usleep_range(1000, 2000);

	if (apple_z2_d21_is_iphone7_plus(z2)) {
		if (z2->core_enabled) {
			error = regulator_disable(z2->core_supply);
			if (error) {
				dev_warn(&z2->spidev->dev,
					 "failed to disable core supply: %d\n", error);
			} else {
				z2->core_enabled = false;
			}
			usleep_range(1000, 2000);
		}
	}
	if (z2->hv_enabled) {
		error = regulator_disable(z2->hv_supply);
		if (error) {
			dev_warn(&z2->spidev->dev,
				 "failed to disable HV supply: %d\n", error);
		} else {
			z2->hv_enabled = false;
		}
	}
	/*
	 * Cut the touch digital-core rail (D21 ADT function-power_ldo =
	 * SMC key 'rBK3') FIRST — iOS's off order (kext enable=0 path: LDO
	 * call, then ana) — then the HV regulator, letting the charge stored
	 * on the die drain: the half-booted/blind core state is rail-backed
	 * and survives a plain probe cycle, so a reload must be a TRUE power
	 * cut (measured 2026-11-03, TOUCH-STATE 1E/2H).
	 *
	 * The SMC cut is a PLAIN SMC_MSG_WRITE_KEY of the function code:
	 * OFF = 0x05020000 (iOS-proven, every screen lock).  A raw 0 write is
	 * an INVALID code and trips systemic protection (twice, 2026-11-03).
	 */
	if (z2->smc && d21_smc_cut_rail) {
		u32 v = d21_smc_off_val;

		error = apple_z2_d21_smc_write(z2->smc, SMC_KEY(rBK3), &v, 4);
		if (error)
			dev_warn(&z2->spidev->dev,
				 "smc rBK3 off-write failed: %d\n", error);
		else {
			z2->smc_rail_up = false;
			dev_info(&z2->spidev->dev, "smc rBK3 OFF 0x%08x state=%d\n",
				 v, d21_smc_rail_state(z2));
		}
		usleep_range(3000, 5000);
	}
	if (z2->ldo_enabled) {
		usleep_range(3000, 5000);
		error = regulator_disable(z2->ldo_supply);
		if (error) {
			dev_warn(&z2->spidev->dev,
				 "failed to disable LDO supply: %d\n", error);
		} else {
			z2->ldo_enabled = false;
		}
	}
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		z2->platform_powered = false;
		return;
	}

	if (apple_z2_d21_is_j172(z2)) {
		error = apple_z2_d21_j172_sync_set_inactive(z2);
		if (error)
			dev_warn(&z2->spidev->dev,
				 "failed to set display-sync inactive: %d\n",
				 error);
	} else {
		apple_z2_d21_set_gpio(z2->display_sync1_gpio, 0);
		apple_z2_d21_set_gpio(z2->display_sync_gpio, 0);
	}
	apple_z2_d21_set_gpio(z2->power_ldo_gpio, 0);
	apple_z2_d21_set_gpio(z2->power_ana_gpio, 0);
	z2->platform_powered = false;
}

static int apple_z2_d21_enable_clock(struct apple_z2_d21 *z2)
{
	int error;

	if (!z2->clk || z2->clk_enabled)
		return 0;

	error = clk_prepare_enable(z2->clk);
	if (error)
		return error;

	z2->clk_enabled = true;
	return 0;
}

static int apple_z2_d21_j172_power_on(struct apple_z2_d21 *z2)
{
	int error;

	apple_z2_d21_platform_power_off(z2);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	msleep(50);

	apple_z2_d21_set_gpio(z2->power_ana_gpio, 1);
	apple_z2_d21_set_gpio(z2->power_ldo_gpio, 1);
	error = apple_z2_d21_j172_sync_set_active(z2);
	if (error) {
		apple_z2_d21_platform_power_off(z2);
		return error;
	}

	error = apple_z2_d21_enable_clock(z2);
	if (error) {
		apple_z2_d21_platform_power_off(z2);
		return error;
	}

	usleep_range(15000, 16000);
	z2->platform_powered = true;
	return apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_OFF,
					 APPLE_Z2_D21_STATE_POWERED);
}

/*
 * D21: fetch the SMC (smc-pmu provider of the touch rail) — the Asahi macsmc
 * driver binds the t8015 SMC at smc@236000100; its drvdata is the apple_smc.
 */
static struct apple_smc *apple_z2_d21_smc(struct device *dev)
{
	struct device_node *np;
	struct device *sdev;
	struct apple_smc *smc;

	np = of_find_compatible_node(NULL, NULL, "apple,t8015-smc");
	if (!np)
		return NULL;
	sdev = bus_find_device_by_of_node(&platform_bus_type, np);
	of_node_put(np);
	if (!sdev)
		return NULL;
	smc = dev_get_drvdata(sdev);
	put_device(sdev);
	return smc;
}

static int apple_z2_d21_power_on(struct apple_z2_d21 *z2)
{
	int error;

	/*
	 * The ADT sequence is power_ana -> power_ldo -> clock_enable.  The
	 * power_ana rail is the chestnut display-PMIC touch-HV switch, wired
	 * as the optional "hv" supply; enable it first, then assert reset and
	 * bring up the touch clock on top.
	 */
	if (z2->hv_supply) {
		error = regulator_enable(z2->hv_supply);
		if (error)
			return error;
		z2->hv_enabled = true;
		usleep_range(1000, 2000);
	}

	/*
	 * The ADT sequence is power_ana -> power_ldo -> clock_enable; the
	 * digital-core rail (SMC key 'rBK3') is wired as the optional
	 * "power-ldo" supply.  power_off has already cut it, so enabling
	 * it here completes a true core power cycle on every transition.
	 */
	if (z2->ldo_supply) {
		error = regulator_enable(z2->ldo_supply);
		if (error) {
			if (z2->hv_enabled) {
				regulator_disable(z2->hv_supply);
				z2->hv_enabled = false;
			}
			return error;
		}
		z2->ldo_enabled = true;
		/*
		 * The chestnut HV must be STABLE before the controller's core
		 * rail comes up, or the Z2 boots, scans 1-2 garbage frames and
		 * goes silent (measured 2026-11-03: a 1-2 ms gap produced a
		 * boot-then-quiet roll; a 12 s settle produced continuous
		 * streaming).  iOS inserts power-ana-enable-delay-us here;
		 * the D21 ADT does not carry it; 50 ms still raced (1.5/5
		 * reloads), 500 ms is the tested-safe settle.
		 */
		usleep_range(500000, 600000);
	}

	/*
	 * Re-assert the core rail through the SMC (what iOS does on every
	 * power-up of this function).  power_off below cuts it.
	 *
	 * 2026-11-03 decode + measurement (AppleSMCEmbeddedFunctions.cpp and
	 * live tests): a PLAIN SMC_MSG_WRITE_KEY of the 4-byte code EXECUTES
	 * the function:  code = (dt_value & 0xffff0000) | enable
	 * -> ON = 0x05020001 (boots the controller, preamble 1f 01 1f 01),
	 *    OFF = 0x05020000 (iOS does this on every screen lock),
	 *    raw 0 = INVALID code -> systemic protection reboot (twice).
	 * RW_KEY's write phase is inert; its read phase returns the function
	 * state (00 = off, 01 = on) — use it only as the status readback.
	 *
	 * 2026-11-04 (§1L): apple_smc_read() on a FUNCTION key ALWAYS fails
	 * with EIO (the SMC returns a nonzero result for a bare READ of a
	 * function key), so the old state log was meaningless noise.  The
	 * true state comes from the RW_KEY transaction: write phase inert
	 * (we pass the ON code), read phase = state byte.  Measured live:
	 * OFF -> 00, ON -> 01, 01 <=> healthy roll.
	 *
	 * d21_smc_on_retries: when >0 and the post-ON state reads != 01,
	 * power_on retries OFF -> 1 s drain -> ON up to N times inside the
	 * probe.  Default 0 (probe-and-log only).
	 */
	if (z2->smc) {
		u32 v = d21_smc_on_val;
		int attempt, state = -1;

		for (attempt = 0; attempt <= d21_smc_on_retries; attempt++) {
			error = apple_z2_d21_smc_write(z2->smc, SMC_KEY(rBK3), &v, 4);
			if (error) {
				dev_warn(&z2->spidev->dev,
					 "smc rBK3 on-write failed: %d\n",
					 error);
			} else {
				z2->smc_rail_up = true;
				state = d21_smc_rail_state(z2);
				dev_info(&z2->spidev->dev,
					 "smc rBK3 ON attempt %d 0x%08x state=%d\n",
					 attempt, v, state);
				if (state == 1)
					break;
			}
			if (attempt < d21_smc_on_retries) {
				u32 off = d21_smc_off_val;

				apple_z2_d21_smc_write(z2->smc, SMC_KEY(rBK3), &off, 4);
				msleep(1000);
			}
		}
		usleep_range(2000, 4000);
	}

	gpiod_set_value_cansleep(z2->reset_gpio, 1);

	error = apple_z2_d21_enable_clock(z2);
	if (error)
		return error;

	usleep_range(1000, 2000);
	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_D21_STATE_POWERED;
	return 0;
}

static int apple_z2_d21_iphone7_plus_power_on(struct apple_z2_d21 *z2)
{
	int error;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);

	error = regulator_enable(z2->hv_supply);
	if (error)
		return error;
	z2->hv_enabled = true;
	usleep_range(2000, 5000);

	error = regulator_enable(z2->core_supply);
	if (error)
		goto err_power_off;
	z2->core_enabled = true;
	usleep_range(2000, 5000);

	error = apple_z2_d21_enable_clock(z2);
	if (error)
		goto err_power_off;
	usleep_range(1000, 2000);

	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_D21_STATE_POWERED;
	return 0;

err_power_off:
	apple_z2_d21_platform_power_off(z2);
	return error;
}

static int apple_z2_d21_platform_power_on(struct apple_z2_d21 *z2)
{
	int error;

	if (z2->platform_powered)
		return 0;

	if (apple_z2_d21_is_j172(z2))
		return apple_z2_d21_j172_power_on(z2);
	if (apple_z2_d21_is_d21(z2))
		return apple_z2_d21_power_on(z2);
	if (apple_z2_d21_is_iphone7_plus(z2))
		return apple_z2_d21_iphone7_plus_power_on(z2);

	apple_z2_d21_set_gpio(z2->power_ana_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_d21_set_gpio(z2->power_ldo_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_d21_pulse_gpio_pair(z2->display_sync_gpio,
				 z2->display_sync1_gpio);

	error = apple_z2_d21_enable_clock(z2);
	if (error) {
		apple_z2_d21_platform_power_off(z2);
		return error;
	}

	usleep_range(1000, 2000);
	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_D21_STATE_POWERED;
	return 0;
}

static int apple_z2_d21_pulse_reset(struct apple_z2_d21 *z2)
{
	/*
	 * The ADT gives this controller a long reset delay, and a short pulse
	 * is not enough to return it to a known state: a stale state then
	 * survives into the next initialisation and the report stream comes
	 * up in whatever mode the previous run left it in.
	 */
	unsigned int hold_ms = apple_z2_d21_is_iphone7_plus(z2) ? 25 : 1;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	msleep(hold_ms);
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	usleep_range(1000, 2000);
	return 0;
}

/* Build calibration blob, caller is responsible for freeing the blob data. */
static const char *apple_z2_d21_calibration_property(u32 provider)
{
	switch (provider) {
	case APPLE_Z2_D21_CAL_MULTI_TOUCH:
		return APPLE_Z2_D21_CAL_PROP_NAME;
	case APPLE_Z2_D21_CAL_ORB_GAP:
		return APPLE_Z2_D21_ORB_GAP_CAL_PROP_NAME;
	case APPLE_Z2_D21_CAL_ORB_FORCE:
		return APPLE_Z2_D21_ORB_FORCE_CAL_PROP_NAME;
	case APPLE_Z2_D21_CAL_SHAPE_ACCEL:
		return APPLE_Z2_D21_SHAPE_ACCEL_CAL_PROP_NAME;
	default:
		return NULL;
	}
}

static const u8 *apple_z2_d21_build_cal_blob(struct apple_z2_d21 *z2,
					 const char *property, u32 address,
					 u32 max_size, size_t *size,
					 bool required)
{
	u8 *cal_data;
	int cal_size;
	size_t padded_size;
	size_t blob_size;
	u32 checksum;
	u16 checksum_hdr;
	int i;
	struct apple_z2_d21_hbpp_blob_hdr *hdr;
	int error;

	if (!device_property_present(&z2->spidev->dev, property))
		return required ? ERR_PTR(-ENOENT) : NULL;

	cal_size = device_property_count_u8(&z2->spidev->dev, property);
	if (cal_size <= 0)
		return ERR_PTR(cal_size ?: -EINVAL);
	if (cal_size > max_size)
		return ERR_PTR(-E2BIG);

	padded_size = round_up((size_t)cal_size, sizeof(__le32));
	blob_size = sizeof(struct apple_z2_d21_hbpp_blob_hdr) + padded_size +
		    sizeof(__le32);
	u8 *blob_data __free(kfree) = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data)
		return ERR_PTR(-ENOMEM);

	hdr = (struct apple_z2_d21_hbpp_blob_hdr *)blob_data;
	hdr->cmd = cpu_to_le16(APPLE_Z2_D21_HBPP_CMD_BLOB);
	hdr->len = cpu_to_le16(padded_size / sizeof(__le32));
	hdr->addr = cpu_to_le32(address);

	checksum_hdr = 0;
	for (i = 2; i < 8; i++)
		checksum_hdr += blob_data[i];
	hdr->checksum = cpu_to_le16(checksum_hdr);

	cal_data = blob_data + sizeof(struct apple_z2_d21_hbpp_blob_hdr);
	error = device_property_read_u8_array(&z2->spidev->dev, property,
					      cal_data, cal_size);
	if (error)
		return ERR_PTR(error);

	checksum = 0;
	for (i = 0; i < cal_size; i++)
		checksum += cal_data[i];
	put_unaligned_le32(checksum, cal_data + padded_size);

	*size = blob_size;
	return no_free_ptr(blob_data);
}

/* Build the HBPP14 calibration packet used by iPhone 7 Plus Gen2 firmware. */
/*
 * The per-device calibration is syscfg data. Our loader does not inject it into
 * the device tree yet, so allow the same blob to be read from /lib/firmware.
 * This is device-specific data: it lives only on the device, never in the tree.
 */
static const char *apple_z2_d21_cal_firmware_name(const char *property)
{
	if (!strcmp(property, APPLE_Z2_D21_CAL_PROP_NAME))
		return "apple/z2-mtcl.bin";
	if (!strcmp(property, APPLE_Z2_D21_ORB_GAP_CAL_PROP_NAME))
		return "apple/z2-orbgap.bin";
	if (!strcmp(property, APPLE_Z2_D21_ORB_FORCE_CAL_PROP_NAME))
		return "apple/z2-orbforce.bin";
	if (!strcmp(property, APPLE_Z2_D21_SHAPE_ACCEL_CAL_PROP_NAME))
		return "apple/z2-shapeaccel.bin";
	if (!strcmp(property, APPLE_Z2_D21_PROX_CAL_PROP_NAME))
		return "apple/z2-proxcal.bin";
	return NULL;
}

static const u8 *apple_z2_d21_build_iphone7_cal(struct apple_z2_d21 *z2,
					    const char *property, u32 address,
					    u32 max_size, size_t *size)
{
	const struct firmware *fw = NULL;
	const u8 *src = NULL;
	u8 *cal_data;
	int cal_size;
	size_t padded_size;
	size_t words;
	size_t blob_size;
	u32 checksum;
	u16 header_checksum;
	unsigned int i;
	bool synthesize = false;
	int error;

	if (device_property_present(&z2->spidev->dev, property)) {
		cal_size = device_property_count_u8(&z2->spidev->dev, property);
		if (cal_size <= 0)
			return ERR_PTR(cal_size ?: -EINVAL);
	} else {
		const char *name = apple_z2_d21_cal_firmware_name(property);

		error = name ? request_firmware(&fw, name, &z2->spidev->dev)
			     : -ENOENT;
		if (!error) {
			cal_size = fw->size;
			src = fw->data;
			dev_info(&z2->spidev->dev,
				 "D21: %s from %s (%d bytes)\n",
				 property, name, cal_size);
		} else if (apple_z2_d21_is_d21(z2)) {
			/*
			 * D21: no calibration available yet. Send a zeroed blob
			 * of the declared size so the firmware's calibration
			 * slots are at least well formed.
			 */
			synthesize = true;
			cal_size = max_size;
			dev_warn(&z2->spidev->dev,
				 "D21: no %s, sending %u zeroed bytes\n",
				 property, max_size);
		} else {
			return ERR_PTR(-ENOENT);
		}
	}
	if (cal_size > max_size) {
		release_firmware(fw);
		return ERR_PTR(-E2BIG);
	}

	padded_size = round_up((size_t)cal_size, sizeof(__le32));
	words = padded_size / sizeof(__le32);
	if (words > U16_MAX) {
		release_firmware(fw);
		return ERR_PTR(-E2BIG);
	}
	blob_size = 12 + padded_size + sizeof(__le32);
	u8 *blob_data __free(kfree) = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data) {
		release_firmware(fw);
		return ERR_PTR(-ENOMEM);
	}

	put_unaligned_be16(0x18e1, blob_data);
	put_unaligned_be16(APPLE_Z2_D21_HBPP_CMD_BLOB, blob_data + 2);
	/*
	 * The length field counts the payload words less one. Apple's own
	 * container for this part stores its firmware blobs that way (63592
	 * bytes carries 15897, 17312 carries 4327), and the calibration packet
	 * shares the framing. Sending the plain word count makes the packet one
	 * word too long, so the controller reads into the trailing checksum and
	 * rejects every calibration blob.
	 */
	put_unaligned_be16(words - 1, blob_data + 4);
	put_unaligned_be16(address, blob_data + 6);
	put_unaligned_be16(address >> 16, blob_data + 8);
	header_checksum = 0;
	for (i = 4; i < 10; i++)
		header_checksum += blob_data[i];
	put_unaligned_be16(header_checksum, blob_data + 10);

	cal_data = blob_data + 12;
	if (src) {
		memcpy(cal_data, src, cal_size);
		release_firmware(fw);
		fw = NULL;
	} else if (!synthesize) {
		error = device_property_read_u8_array(&z2->spidev->dev,
						      property, cal_data,
						      cal_size);
		if (error)
			return ERR_PTR(error);
	}

	checksum = 0;
	for (i = 0; i < cal_size; i++)
		checksum += cal_data[i];
	for (i = 0; i < padded_size; i += 2)
		swap(cal_data[i], cal_data[i + 1]);
	put_unaligned_be16(checksum, cal_data + padded_size);
	put_unaligned_be16(checksum >> 16, cal_data + padded_size + 2);

	*size = blob_size;
	return no_free_ptr(blob_data);
}

static int apple_z2_d21_send_firmware_blob(struct apple_z2_d21 *z2, const u8 *data,
				       u32 size, bool init, bool defer_irq_wait)
{
	struct spi_transfer blob_xfer = {
		.tx_buf = data,
		.len = size,
		.bits_per_word = 8,
	};
	struct spi_transfer ack_xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->ack_buf,
		.len = 2,
		.bits_per_word = 8,
	};
	bool ready;
	int error;

	/*
	 * iPhone 7 Plus firmware stores HBPP byte streams in 8-bit wire order.
	 * SmartIO may still DMA these transfers, but using 16-bit SPI words would
	 * swap every byte pair a second time.
	 */
	if (!init && !apple_z2_d21_is_iphone7_plus(z2) &&
	    size >= z2->bpw16_min_len)
		blob_xfer.bits_per_word = 16;

	dev_dbg(&z2->spidev->dev, "firmware blob len=%u bpw=%u\n",
		size, blob_xfer.bits_per_word);

	if ((apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) && init) {
		if (size != 4 || data[0] != 0x1a || data[1] != 0xa1 ||
		    data[2] != 0x18 || data[3] != 0xe1)
			return dev_err_probe(&z2->spidev->dev, -EINVAL,
					     "invalid HBPP14 readiness command\n");
		memset(z2->rx_buf, 0, size);
		blob_xfer.rx_buf = z2->rx_buf;
	}
	apple_z2_d21_avoid_short_dma(z2, &blob_xfer);

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	apple_z2_d21_avoid_short_dma(z2, &ack_xfer);
	reinit_completion(&z2->boot_irq);
	error = spi_sync_transfer(z2->spidev, &blob_xfer, 1);
	if (error)
		return error;

	if ((apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) && init) {
		ready = z2->rx_buf[0] == 0x1f && z2->rx_buf[1] == 0x01;
		dev_info(&z2->spidev->dev, "D21 initial-blob reply: %*ph\n",
			 8, z2->rx_buf);
		if (!ready)
			return dev_err_probe(&z2->spidev->dev, -EPROTO,
					     "touch controller not ready\n");
		dev_dbg(&z2->spidev->dev, "HBPP14 ready\n");
	}

	if (!init || !z2->no_init_ack) {
		error = spi_sync_transfer(z2->spidev, &ack_xfer, 1);
		if (error)
			return error;
		z2->last_blob_ack = get_unaligned_le16(z2->ack_buf);
		dev_info(&z2->spidev->dev, "blob len=%u ack: %*ph\n", size, 2,
			 z2->ack_buf);
		apple_z2_d21_post_ack_delay(z2);
	}
	if ((apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) && init) {
		error = apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_BOOT_IRQ,
						  APPLE_Z2_D21_STATE_HBPP_READY);
		if (error)
			return error;
	}

	if (defer_irq_wait && z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		apple_z2_d21_irq_on(z2);
	}

	if (!defer_irq_wait)
		wait_for_completion_timeout(&z2->boot_irq,
					    msecs_to_jiffies(20));

	return 0;
}

static int apple_z2_d21_send_firmware_raw_xfer(struct apple_z2_d21 *z2,
					   const u8 *data, u32 size)
{
	struct spi_transfer xfer = { };
	const u8 *logical_tx;
	u32 tx_len, rx_len, flags, xfer_len;
	unsigned int i;
	int error;

	if (size < 3 * sizeof(__le32))
		return -EINVAL;

	tx_len = get_unaligned_le32(data);
	rx_len = get_unaligned_le32(data + sizeof(__le32));
	flags = get_unaligned_le32(data + 2 * sizeof(__le32));
	if (tx_len > APPLE_Z2_D21_RAW_XFER_MAX_SIZE ||
	    rx_len > APPLE_Z2_D21_RAW_XFER_MAX_SIZE ||
	    flags & ~APPLE_Z2_D21_RAW_XFER_RX_BIT_REVERSE ||
	    size != 3 * sizeof(__le32) + tx_len ||
	    (!tx_len && !rx_len))
		return -EINVAL;

	xfer_len = max(tx_len, rx_len);
	if ((tx_len && tx_len != xfer_len) ||
	    (rx_len && rx_len != xfer_len))
		return -EINVAL;

	logical_tx = data + 3 * sizeof(__le32);
	if (tx_len)
		xfer.tx_buf = logical_tx;
	if (rx_len) {
		memset(z2->rx_buf, 0, rx_len);
		xfer.rx_buf = z2->rx_buf;
	}
	xfer.len = xfer_len;
	xfer.bits_per_word = 8;
	apple_z2_d21_avoid_short_dma(z2, &xfer);

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	if (flags & APPLE_Z2_D21_RAW_XFER_RX_BIT_REVERSE) {
		for (i = 0; i < rx_len; i++)
			z2->rx_buf[i] = bitrev8(z2->rx_buf[i]);
	}

	dev_dbg(&z2->spidev->dev,
		"firmware raw xfer tx_len=%u rx_len=%u flags=%#x tx=%*ph rx=%*ph\n",
		 tx_len, rx_len, flags, min(tx_len, 16U), logical_tx,
		 min(rx_len, 16U), z2->rx_buf);

	return 0;
}

static bool apple_z2_d21_wait_firmware_irq(struct apple_z2_d21 *z2, u32 timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);

	if (!timeout)
		timeout = msecs_to_jiffies(1);

	/*
	 * Re-arm before waiting. The controller raises an edge for each phase
	 * it wants serviced, and Apple's loader for this part waits for the
	 * one that follows the boot sequence before it hands over calibration.
	 * Without re-arming, a completion left over from an earlier phase
	 * satisfies the wait immediately and the calibration goes out before
	 * the controller asked for it.
	 */
	reinit_completion(&z2->boot_irq);

	if (!wait_for_completion_timeout(&z2->boot_irq, timeout)) {
		dev_warn(&z2->spidev->dev,
			 "firmware IRQ wait timed out after %u ms\n", timeout_ms);
		return false;
	}

	return true;
}

static int apple_z2_d21_wait_ready_irq(struct apple_z2_d21 *z2, u32 timeout_ms)
{
	if (!apple_z2_d21_wait_firmware_irq(z2, timeout_ms)) {
		/*
		 * Missing the edge does not abort an otherwise completed boot,
		 * but it does mean the calibration that follows was not asked
		 * for, so say so rather than failing silently.
		 */
		if (!apple_z2_d21_is_d21(z2))
			return -ETIMEDOUT;
		dev_warn(&z2->spidev->dev,
			 "calibration request IRQ not seen within %u ms\n",
			 timeout_ms);
	}
	if (!apple_z2_d21_is_j172(z2) && !apple_z2_d21_is_iphone7_plus(z2))
		return 0;

	if (apple_z2_d21_is_j172(z2) &&
	    !apple_z2_d21_wait_firmware_irq(z2, timeout_ms))
		return -ETIMEDOUT;

	dev_dbg(&z2->spidev->dev, "firmware ready IRQ%s received\n",
		apple_z2_d21_is_j172(z2) ? "s" : "");
	return apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_HBPP_READY,
					 APPLE_Z2_D21_STATE_FIRMWARE_READY);
}

static u32 apple_z2_d21_fw_config_word(const u8 *data, unsigned int word)
{
	return get_unaligned_le32(data + word * sizeof(__le32));
}

static int apple_z2_d21_apply_fw_config(struct apple_z2_d21 *z2, const u8 *data,
				    size_t size)
{
	struct spi_device *spi = z2->spidev;
	u32 valid, min_dma, z2_delay, cs_delay, cpha, cpol, boot_timeout;
	u32 mode = spi->mode;
	int error;

	if (size < APPLE_Z2_D21_FW_CONFIG_WORDS * sizeof(__le32))
		return -EINVAL;

	valid = apple_z2_d21_fw_config_word(data, 0);
	min_dma = apple_z2_d21_fw_config_word(data, 1);
	z2_delay = apple_z2_d21_fw_config_word(data, 2);
	cs_delay = apple_z2_d21_fw_config_word(data, 3);
	cpha = apple_z2_d21_fw_config_word(data, 4);
	cpol = apple_z2_d21_fw_config_word(data, 5);
	boot_timeout = apple_z2_d21_fw_config_word(data, 9);
	if ((valid & APPLE_Z2_D21_FW_CONFIG_MIN_DMA) && min_dma) {
		if (apple_z2_d21_is_iphone7_plus(z2)) {
			if (min_dma > APPLE_Z2_D21_RX_BUF_SIZE)
				return -EINVAL;
			z2->dma_min_len = min_dma;
		} else {
			z2->bpw16_min_len = min_dma;
		}
	}
	if (valid & APPLE_Z2_D21_FW_CONFIG_Z2_DELAY)
		z2->z2_inter_packet_delay_us = z2_delay;
	/* iPhone 7 Plus uses zero to retain the required 1 ms CS delay. */
	if ((valid & APPLE_Z2_D21_FW_CONFIG_CS_DELAY) &&
	    (!apple_z2_d21_is_iphone7_plus(z2) || cs_delay))
		z2->z2_cs_delay_us = cs_delay;
	if ((valid & APPLE_Z2_D21_FW_CONFIG_BOOT_TIMEOUT) && boot_timeout)
		z2->boot_timeout_ms = boot_timeout;

	if (valid & APPLE_Z2_D21_FW_CONFIG_CPHA) {
		if (cpha)
			mode |= SPI_CPHA;
		else
			mode &= ~SPI_CPHA;
	}
	if (valid & APPLE_Z2_D21_FW_CONFIG_CPOL) {
		if (cpol)
			mode |= SPI_CPOL;
		else
			mode &= ~SPI_CPOL;
	}

	if (mode != spi->mode) {
		u32 old_mode = spi->mode;

		spi->mode = mode;
		error = spi_setup(spi);
		if (error) {
			spi->mode = old_mode;
			return error;
		}
	}

	apple_z2_d21_apply_z2_delays(z2);
	dev_dbg(&spi->dev,
		"firmware SPI config: valid=%#x dma-min-len=%u bpw16-min-len=%u z2-delay=%u cs-delay=%u boot-timeout=%u mode=%#x\n",
		 valid, z2->dma_min_len, z2->bpw16_min_len,
		 z2->z2_inter_packet_delay_us,
		 z2->z2_cs_delay_us, z2->boot_timeout_ms, spi->mode);

	return 0;
}

/*
 * D21 diagnostic walk of the iPad-style report init. The D21 firmware is
 * HBPP 0x103, the same generation as Orchard's J71b, whose verified sequence
 * uses the 0xd9/0xdb/0x73 reads and 0x9d/0xbf writes rather than the iPhone
 * init inherited from D111. Log every step and never fail the probe so one
 * run shows exactly which exchanges the controller accepts.
 */
/*
 * Complete the DeviceInfo handshake that follows the firmware upload. The
 * controller does not report contacts until this has been answered: 0xee wakes
 * it, 0xe2 is the request, and 0xe1 collects the reply, which echoes the command
 * byte and checksums its first fourteen bytes. This is the same three-frame
 * exchange the same-generation J71b firmware requires; the frames our command
 * helper builds are byte-identical to the reference's (the trailer is the sum
 * of the preceding bytes, which for an otherwise zeroed frame is the command
 * byte itself).
 */
static int apple_z2_d21_device_info(struct apple_z2_d21 *z2)
{
	static const u8 commands[] = { APPLE_Z2_D21_CMD_WAKE,
				       APPLE_Z2_D21_CMD_DEVICE_INFO,
				       APPLE_Z2_D21_CMD_LAST };
	struct device *dev = &z2->spidev->dev;
	u16 sum = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		int error;

		error = apple_z2_d21_z2_cmd_locked_delay(z2, commands[i], 0,
						     "d21-device-info", false);
		if (error)
			return error;
		if (i == 0)
			usleep_range(2000, 2100);
		if (i == 1)
			usleep_range(125, 225);
	}

	for (i = 0; i < 14; i++)
		sum += z2->rx_buf[i];
	if (z2->rx_buf[0] != APPLE_Z2_D21_CMD_DEVICE_INFO ||
	    get_unaligned_le16(z2->rx_buf + 14) != sum)
		return dev_err_probe(dev, -EPROTO,
				     "D21 device info reply invalid: %*ph\n",
				     APPLE_Z2_D21_CMD_SIZE, z2->rx_buf);

	dev_info(dev, "D21 device info: platform=%u interface=%u max-packet=%u\n",
		 z2->rx_buf[1], z2->rx_buf[2],
		 get_unaligned_le16(z2->rx_buf + 3));
	return 0;
}

/*
 * Diagnostic: ask the controller for its own state and counter reports.
 * Reading a report takes it out of the firmware's report table, so this
 * consumes anything the controller had queued for the interrupt channel. It
 * stays off unless asked for, to keep an ordinary touch test clean.
 */
static bool apple_z2_d21_probe_reports;
module_param_named(d21_probe_reports, apple_z2_d21_probe_reports, bool, 0444);
MODULE_PARM_DESC(d21_probe_reports, "Sample the controller state and counter reports");

/*
 * Bring-up experiment knobs.  D21 is HBPP 0x103, the same firmware generation
 * as Orchard's verified J71b, yet its report table also exposes the 0x9d/0xbf/
 * 0xaf reports that the J172 path uses.  Rather than guess, let every step of
 * the runtime initialisation be skipped or re-valued without a rebuild:
 *   - 0x9d/0xbf are J172 "mode setup", skipped on J71b;
 *   - 0xaf is a report-enable whose value the J71b image does not implement;
 *   - 0xe2 enables the iPhone-family report set;
 *   - 0x72 offset 1 is the "Multitouch high level" power/UI state.
 * A -1 value skips that write entirely.
 */
static bool d21_skip_j172_writes;
module_param_named(d21_skip_j172_writes, d21_skip_j172_writes, bool, 0444);
static bool d21_prox_cal = true;
module_param_named(d21_prox_cal, d21_prox_cal, bool, 0444);
MODULE_PARM_DESC(d21_prox_cal,
		 "D21: download the in-pocket prox calibration (syscfg PxCl -> 0x00400180) at bootload");
MODULE_PARM_DESC(d21_skip_j172_writes, "Skip the J172 0x9d/0xbf mode-setup writes");
static int d21_af_val = 0;
module_param_named(d21_af_val, d21_af_val, int, 0444);
MODULE_PARM_DESC(d21_af_val, "Value written to report 0xaf, or -1 to skip");
static int d21_e2_val = 1;
module_param_named(d21_e2_val, d21_e2_val, int, 0444);
MODULE_PARM_DESC(d21_e2_val, "Value written to report 0xe2, or -1 to skip");
static int d21_hl_state = 1;
module_param_named(d21_hl_state, d21_hl_state, int, 0444);
MODULE_PARM_DESC(d21_hl_state, "Report 0x72 offset 1 high-level state, or -1 to skip");
static int d21_probe_id = -1;
module_param_named(d21_probe_id, d21_probe_id, int, 0444);
MODULE_PARM_DESC(d21_probe_id, "Read this report id repeatedly after init (diagnostic)");
static int d21_probe_times = 40;
module_param_named(d21_probe_times, d21_probe_times, int, 0444);
MODULE_PARM_DESC(d21_probe_times, "How many times to read probe_id");
static bool d21_poll = true;
module_param_named(d21_poll, d21_poll, bool, 0444);
MODULE_PARM_DESC(d21_poll, "Poll the interrupt-data queue after init (probe_times x 100 ms)");
static char *d21_watch_ids;
module_param_named(d21_watch_ids, d21_watch_ids, charp, 0444);
MODULE_PARM_DESC(d21_watch_ids, "Comma-separated hex report ids to watch with full-payload dumps");
static char *d21_write = "70:08";
module_param_named(d21_write, d21_write, charp, 0444);
MODULE_PARM_DESC(d21_write,
		 "D21: comma-separated id:hexpayload report writes applied after the whole init (e.g. write=c8:01,9d:0200)");
/*
 * Default D21 arming.  The init choreography's legacy bf write carries the
 * MultitouchSupport "touch mode 0" payload (9d 81 09 00 = touch OFF), and the
 * userspace arming that iOS lands from backboardd never happens on Linux.
 * With HV rails powered the controller boots and scans but stays silent
 * until these two writes are made: report 0xc8 bit0 set (one-byte detection
 * options, 08 -> 09) and report 0xbf mode 1 (touch on, 8-byte
 * endianness-tagged form).  arm=0 restores the raw pre-arming behaviour for
 * A/B tests; explicit write= entries always apply on top of the default.
 */
static bool d21_arm = true;
module_param_named(d21_arm, d21_arm, bool, 0444);
MODULE_PARM_DESC(d21_arm,
		 "D21: apply the default touch-arming writes (c8=09, bf mode 1) after init");
static char *d21_read_ids;
module_param_named(d21_read, d21_read_ids, charp, 0444);
MODULE_PARM_DESC(d21_read,
		 "D21: comma-separated id:len raw report reads, run before the writes (feature-space "
		 "properties invisible to GET_REPORT_INFO, e.g. read=d1:4,d3:16,7f:16)");
static bool d21_read_after;
module_param_named(d21_read_after, d21_read_after, bool, 0444);
MODULE_PARM_DESC(d21_read_after,
		 "D21: re-run the read= list after the write/rmw entries to compare readbacks");
static char *d21_rmw;
module_param_named(d21_rmw, d21_rmw, charp, 0444);
MODULE_PARM_DESC(d21_rmw,
		 "D21: comma-separated id:len read-modify-write entries applied after the writes "
		 "(reads len bytes and writes them straight back, e.g. rmw=a6:4)");
static int d21_poll_watch;
module_param_named(d21_poll_watch, d21_poll_watch, int, 0444);
MODULE_PARM_DESC(d21_poll_watch,
		 "D21: read this report id every poll iteration and dump changes (post-arming watch)");
static int d21_poll_watch_len = 512;
module_param_named(d21_poll_watch_len, d21_poll_watch_len, int, 0444);
MODULE_PARM_DESC(d21_poll_watch_len, "D21: cap the poll-watch read length");

/*
 * Orchard's working J71b runtime (same HBPP 0x103 generation as D21) switches
 * the touch IRQ to level-low before enabling it, while this driver keeps the
 * device tree's falling-edge trigger.  Apple's own D21 personality also carries
 * a host-config block (SPI mode, inter-packet delay, minimum DMA size) that the
 * D21 container has no SET_CONFIG record for, so it is never applied.  Expose
 * both as knobs so a single run can be A/B'd on the device.
 */
static bool d21_irq_level = true;
module_param_named(d21_irq_level, d21_irq_level, bool, 0444);
MODULE_PARM_DESC(d21_irq_level, "D21: switch the touch IRQ to level-low at runtime");
static int d21_ip_delay = -1;
module_param_named(d21_ip_delay, d21_ip_delay, int, 0444);
MODULE_PARM_DESC(d21_ip_delay, "D21: inter-packet delay us (Apple profile: 125), -1 keeps");
static int d21_dma_min = -1;
module_param_named(d21_dma_min, d21_dma_min, int, 0444);
MODULE_PARM_DESC(d21_dma_min, "D21: minimum DMA transfer size (Apple profile: 16), -1 keeps");
static int d21_spi_mode = -1;
module_param_named(d21_spi_mode, d21_spi_mode, int, 0444);
MODULE_PARM_DESC(d21_spi_mode, "D21: force SPI mode 0..3 (Apple profile: 1), -1 keeps");

/*
 * Orchard's working J71b runtime (same HBPP 0x103 firmware generation as D21)
 * arms the contact stream with a fixed post-boot choreography, and its
 * container carries no runtime-enable write at all.  Its order is:
 *
 *   DeviceInfo (WAKE,E2,E1) -> one interrupt-data poll -> surface (0xd9)
 *   info+read -> database (0xdb) info + read x3 -> 0x73 info (+reads when it
 *   exists) -> drain the queue -> touch IRQ to level-low -> settle 2260 ms ->
 *   WAKE -> 0x9d info (J71b: unsupported -> no writes at all -> REPORTS_READY)
 *
 * The D21 path did the reads only once, WAKE first, and had no settle, and the
 * write-based A/B (9d/bf/af/e2/72) never changed that.  Mirror the working
 * reference by default; every knob is exposed so each step can be varied on
 * the device without a rebuild.
 */
static bool d21_j71b_order = true;
module_param_named(d21_j71b_order, d21_j71b_order, bool, 0444);
MODULE_PARM_DESC(d21_j71b_order, "D21: use the J71b runtime init choreography");
static int d21_db_reads = 3;
module_param_named(d21_db_reads, d21_db_reads, int, 0444);
MODULE_PARM_DESC(d21_db_reads, "D21: how many times to read report 0xdb (J71b: 3)");
static int d21_settle_ms = 2260;
module_param_named(d21_settle_ms, d21_settle_ms, int, 0444);
MODULE_PARM_DESC(d21_settle_ms, "D21: settle after IRQ enable, before the late WAKE (J71b: 2260)");
static int d21_drain_polls = 4;
module_param_named(d21_drain_polls, d21_drain_polls, int, 0444);
MODULE_PARM_DESC(d21_drain_polls, "D21: interrupt-data drain polls after the init reads (J71b: 4)");
static char *d21_poke =
	"10003060:17d3,1000305c:2,10003058:6,10003000:3,10003518:1";
module_param_named(d21_poke, d21_poke, charp, 0444);
MODULE_PARM_DESC(d21_poke,
		 "D21: pre-calibration memory pokes 'addr:val[,addr:val]' (hex), blob path. "
		 "C1F5E,4 fwconfig Boot Sequence: 10003060:17d3,1000305c:2,10003058:6,10003000:3,10003458:1");
static char *d21_poke_after = "10003458:1";
module_param_named(d21_poke_after, d21_poke_after, charp, 0444);
MODULE_PARM_DESC(d21_poke_after,
		 "D21: post-calibration memory pokes, same format as poke= (C1F5E,4: 10003300:1)");

/*
 * Settle delay in ms before each poke list.  The container's readiness
 * probe completes microseconds before our first blob poke; on some boots
 * the controller is not accepting blobs yet and rejects it (measured
 * 2026-10-30).  50 ms removes the rejection on the boots tested.
 */
static unsigned int d21_poke_delay_ms = 50;
module_param_named(d21_poke_delay_ms, d21_poke_delay_ms, uint, 0444);
MODULE_PARM_DESC(d21_poke_delay_ms,
		 "D21: settle delay before the poke= / poke_after= lists (default 50)");

/*
 * Verify the pokes by reading them back through the kext's register-read
 * packet (0x1c73 + long ATN_ACK).  There is no other memory-read path, so
 * without this a poke that the controller silently dropped is invisible -
 * which is exactly the boot-to-boot variance we are chasing.
 */
static bool d21_poke_readback;
module_param_named(d21_poke_readback, d21_poke_readback, bool, 0444);
MODULE_PARM_DESC(d21_poke_readback,
		 "D21: read back every poke= / poke_after= register and log wrote vs read (default false)");

/*
 * Read a report back and log it. The controller's own configuration for this
 * part declares two host-visible reports: 0xf8 carries the scan timing
 * counters (dirty/clean/active frames per scan phase) and 0x72 carries the
 * "Multitouch high level" state (Unknown/Active/Ready/UILock/PreReady/...).
 * Sampling them tells us whether the sensing engine is actually running and
 * which power state it believes it is in.
 */
static void apple_z2_d21_probe_report(struct apple_z2_d21 *z2, u8 report,
				      const char *tag)
{
	struct device *dev = &z2->spidev->dev;
	u8 status = 0;
	u16 len = 0;
	int error;

	error = apple_z2_d21_read_report_info_locked(z2, report, &status, &len, tag);
	dev_info(dev, "D21 probe %s: info err=%d status=%u len=%u\n", tag,
		 error, status, len);
	if (error || len < 3 || len > APPLE_Z2_D21_RX_BUF_SIZE - 5)
		return;

	error = apple_z2_d21_read_report_locked(z2, report, len, tag);
	if (error) {
		dev_info(dev, "D21 probe %s: read err=%d\n", tag, error);
		return;
	}

	dev_info(dev, "D21 probe %s: %*ph\n", tag,
		 (int)min_t(size_t, len + 5, 48U), z2->rx_buf);
	if (report == 0x72 && len >= 4)
		dev_info(dev, "D21 probe %s: high-level state index=%u\n", tag,
			 z2->rx_buf[4]);
}

/*
 * Run a comma-separated id:len raw feature-space read list (see the read=
 * knob).  Idempotent: called before the writes by default and again after
 * them when read_after=1, to spot readbacks that change once the arming
 * writes have landed.
 */
static void apple_z2_d21_read_list(struct apple_z2_d21 *z2, const char *ids)
{
	char *buf, *entry;
	struct device *dev = &z2->spidev->dev;
	int error;

	buf = kstrdup(ids, GFP_KERNEL);
	if (!buf)
		return;
	for (entry = buf; entry && *entry; ) {
		unsigned int id, len;
		char *comma, *colon;

		comma = strpbrk(entry, ",");
		if (comma)
			*comma++ = '\0';
		colon = strchr(entry, ':');
		if (!colon) {
			dev_err(dev, "D21 read: bad entry '%s'\n", entry);
			entry = comma;
			continue;
		}
		*colon++ = '\0';
		id = simple_strtoul(entry, NULL, 16);
		len = simple_strtoul(colon, NULL, 10);
		if (!len || len > APPLE_Z2_D21_RX_BUF_SIZE - 5) {
			dev_err(dev, "D21 read: bad len %u\n", len);
			entry = comma;
			continue;
		}
		error = apple_z2_d21_read_report_locked(z2, id, len, "d21-read");
		dev_info(dev, "D21 read %02x len %u: err=%d\n", id, len,
			 error);
		if (!error)
			dev_info(dev, "D21 read %02x: %*ph\n", id,
				 (int)min_t(size_t, len + 5U, 96U),
				 z2->rx_buf);
		entry = comma;
	}
	kfree(buf);
}

/*
 * The (single) D21 device, for the runtime read_now=/kick= knobs below.  Set
 * once the D21 probe tail runs; cleared on remove.
 *
 * The sysfs param setters race with module unload: a write can pass the NULL
 * check and then sleep in kstrdup() while remove() clears the pointer and
 * devm frees the struct.  Observed in the field as a NULL deref inside
 * apple_z2_d21_read_list (z2->spidev read as NULL from the zeroed freed
 * chunk).  d21_now_lock closes the window: setters hold it across the whole
 * use, remove clears the pointer under it before anything is freed.
 */
static struct apple_z2_d21 *d21_z2_now;
static DEFINE_MUTEX(d21_now_lock);

static struct apple_z2_d21 *apple_z2_d21_now_get(void)
{
	mutex_lock(&d21_now_lock);
	if (!d21_z2_now)
		mutex_unlock(&d21_now_lock);
	return d21_z2_now;
}

static void apple_z2_d21_now_put(void)
{
	mutex_unlock(&d21_now_lock);
}

static int d21_read_now_set(const char *val, const struct kernel_param *kp)
{
	struct apple_z2_d21 *z2 = apple_z2_d21_now_get();
	char *buf;

	if (!z2)
		return -ENODEV;
	buf = kstrdup(val, GFP_KERNEL);
	if (!buf) {
		apple_z2_d21_now_put();
		return -ENOMEM;
	}

	mutex_lock(&z2->io_lock);
	apple_z2_d21_read_list(z2, buf);
	mutex_unlock(&z2->io_lock);

	kfree(buf);
	apple_z2_d21_now_put();
	return 0;
}

static const struct kernel_param_ops d21_read_now_ops = {
	.set = d21_read_now_set,
};
module_param_cb(d21_read_now, &d21_read_now_ops, NULL, 0644);
MODULE_PARM_DESC(d21_read_now,
		 "D21: write 'id:len,id:len' to read reports immediately, no reload "
		 "(e.g. echo 7f:16 > .../parameters/read_now)");

/*
 * Runtime 0x72 high-level state transition: write the state index to
 * re-run the boot set-state transaction and log the readback.  The boot
 * path (hl_state=) writes Active(1) once; iOS drives lock->unlock
 * transitions every wake and never leaves the state machinery alone, so
 * a controller that latched a guard state (UILock(4), PreReady(8),
 * Anticip(9)) would stay there forever on Linux - this knob is the
 * experiment lever for driving transitions on a live roll.
 */
static int d21_hl_state_set_set(const char *val, const struct kernel_param *kp)
{
	struct apple_z2_d21 *z2 = apple_z2_d21_now_get();
	u8 high_level[8] = { };
	int state, error;

	if (!z2)
		return -ENODEV;

	error = kstrtoint(val, 0, &state);
	if (error || state < 0 || state > 255) {
		apple_z2_d21_now_put();
		return error ?: -EINVAL;
	}

	mutex_lock(&z2->io_lock);
	high_level[1] = state;
	error = apple_z2_d21_write_report_locked(z2, APPLE_Z2_D21_REPORT_HIGH_LEVEL,
					     high_level, sizeof(high_level),
					     "d21-72-set");
	dev_info(&z2->spidev->dev, "D21: 72 set-state %u err=%d\n",
		 state, error);
	if (!error)
		apple_z2_d21_probe_report(z2, APPLE_Z2_D21_REPORT_HIGH_LEVEL,
					  "d21-72-set-after");
	mutex_unlock(&z2->io_lock);

	apple_z2_d21_now_put();
	return 0;
}

static const struct kernel_param_ops d21_hl_state_set_ops = {
	.set = d21_hl_state_set_set,
};
module_param_cb(d21_hl_state_set, &d21_hl_state_set_ops, NULL, 0644);
MODULE_PARM_DESC(d21_hl_state_set,
		 "D21: write the 0x72 high-level state index (0-255) to run the "
		 "set-state transaction now (e.g. echo 1 > .../parameters/hl_state_set)");

/*
 * Runtime HBPP register read: 'echo addr[,addr...] > .../parameters/regread'
 * reads each 32-bit word through the kext's register-read packet and logs
 * 'D21 regread addr=... =... err=...'.  Unlike read_now= these are HBPP
 * command packets, not report pulls, so they do not share the frame-stream
 * pull tag - but they still pause the stream briefly; do not interleave with
 * a frame-timing capture.
 */
static int apple_z2_d21_regread32(struct apple_z2_d21 *z2, u32 address, u32 *value);

static int d21_regread_set(const char *val, const struct kernel_param *kp)
{
	struct apple_z2_d21 *z2 = apple_z2_d21_now_get();
	char *buf, *entry;
	unsigned long addr;
	u32 value;
	int error = 0;

	if (!z2)
		return -ENODEV;
	buf = kstrdup(val, GFP_KERNEL);
	if (!buf) {
		apple_z2_d21_now_put();
		return -ENOMEM;
	}

	mutex_lock(&z2->io_lock);
	for (entry = buf; entry && *entry; ) {
		char *comma = strchr(entry, ',');

		if (comma)
			*comma++ = '\0';
		addr = simple_strtoul(entry, NULL, 16);
		value = 0;
		error = apple_z2_d21_regread32(z2, addr, &value);
		dev_info(&z2->spidev->dev,
			 "D21 regread addr=%08lx =%08x err=%d\n",
			 addr, value, error);
		if (error)
			break;
		entry = comma;
	}
	mutex_unlock(&z2->io_lock);

	kfree(buf);
	apple_z2_d21_now_put();
	return error;
}

static const struct kernel_param_ops d21_regread_ops = {
	.set = d21_regread_set,
};
module_param_cb(d21_regread, &d21_regread_ops, NULL, 0644);
MODULE_PARM_DESC(d21_regread,
		 "D21: write 'addr[,addr]' (hex) to read 32-bit controller registers via the "
		 "HBPP register-read packet (e.g. echo 10003060 > .../parameters/regread)");

/*
 * Diagnostics for the batched-delivery investigation: kick the IRQ line by
 * disable+enable so the interrupt controller re-samples the level.  If frames
 * are queued with the line asserted and the level re-fire was missed, a kick
 * mid-gap releases them immediately.
 */
static int d21_kick_set(const char *val, const struct kernel_param *kp)
{
	struct apple_z2_d21 *z2 = apple_z2_d21_now_get();

	if (!z2)
		return -ENODEV;
	apple_z2_d21_irq_off(z2);
	apple_z2_d21_irq_on(z2);
	apple_z2_d21_now_put();
	return 0;
}

static const struct kernel_param_ops d21_kick_ops = {
	.set = d21_kick_set,
};
module_param_cb(d21_kick, &d21_kick_ops, NULL, 0644);
MODULE_PARM_DESC(d21_kick,
		 "D21: write anything to disable+enable the touch IRQ (AIC level re-sample test)");

static int apple_z2_d21_probe_tail(struct apple_z2_d21 *z2);

static int apple_z2_d21_init_locked(struct apple_z2_d21 *z2)
{
	static const u8 rep9d[] = { 0x01, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00 };
	static const u8 repbf[] = { 0x9d, 0x81, 0x09, 0x00 };
	struct device *dev = &z2->spidev->dev;
	u8 high_level[8] = { };	/* the state channel sits at offset 1 */
	u8 status = 0;
	u16 len = 0;
	int error;

	error = apple_z2_d21_device_info(z2);
	dev_info(dev, "D21 init: device info err=%d\n", error);

	if (!d21_j71b_order) {
		error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_WAKE, 0, "d21-wake");
		dev_info(dev, "D21 init: wake err=%d\n", error);
		usleep_range(2000, 2100);
	} else {
		/*
		 * The J71b runtime performs a single interrupt-data poll right
		 * after the handoff, before any report queries.
		 */
		error = apple_z2_d21_read_packet(z2);
		dev_info(dev, "D21 init: post-handoff poll err=%d valid=%d\n",
			 error, z2->runtime_frame_valid);
	}

	error = apple_z2_d21_read_report_info_locked(z2, 0xd9, &status, &len,
						 "d21-d9");
	dev_info(dev, "D21 init: d9 info err=%d status=%u len=%u\n",
		 error, status, len);
	if (!error && len) {
		error = apple_z2_d21_read_report_locked(z2, 0xd9, len, "d21-d9-read");
		dev_info(dev, "D21 init: d9 read err=%d\n", error);
		if (!error) {
			error = apple_z2_d21_store_surface_descriptor(z2, len);
			dev_info(dev, "D21 init: d9 store err=%d len=%u\n",
				 error, len);
		}
	}

	status = 0; len = 0;
	error = apple_z2_d21_read_report_info_locked(z2, 0xdb, &status, &len,
						 "d21-db");
	dev_info(dev, "D21 init: db info err=%d status=%u len=%u\n",
		 error, status, len);
	if (!error && len) {
		unsigned int db;

		for (db = 0; db < (d21_j71b_order ? d21_db_reads : 1); db++) {
			char tag[16];

			snprintf(tag, sizeof(tag), "d21-db-read-%u", db + 1);
			error = apple_z2_d21_read_report_locked(z2, 0xdb, len, tag);
			dev_info(dev, "D21 init: db read %u err=%d len=%u\n",
				 db + 1, error, len);
		}
	}

	status = 0; len = 0;
	error = apple_z2_d21_read_report_info_locked(z2, 0x73, &status, &len,
						 "d21-73");
	dev_info(dev, "D21 init: 73 info err=%d status=%u len=%u\n",
		 error, status, len);
	if (!error && len) {
		unsigned int pr;

		for (pr = 0; pr < (d21_j71b_order ? 3 : 1); pr++) {
			char tag[16];

			snprintf(tag, sizeof(tag), "d21-73-read-%u", pr + 1);
			error = apple_z2_d21_read_report_locked(z2, 0x73, len, tag);
			dev_info(dev, "D21 init: 73 read %u err=%d len=%u\n",
				 pr + 1, error, len);
		}
	} else if (d21_j71b_order && !error && !len) {
		dev_info(dev,
			 "D21 init: report 0x73 unsupported (status=%u), as on J71b\n",
			 status);
	}

	if (d21_j71b_order) {
		/*
		 * Drain the interrupt-data queue the way the J71b runtime does
		 * before it arms the IRQ: an asserted line must be serviced
		 * through reads, not left pending.
		 */
		unsigned int dp;

		for (dp = 0; dp < d21_drain_polls; dp++) {
			error = apple_z2_d21_read_packet(z2);
			dev_info(dev, "D21 init: drain %u err=%d empty=%d\n",
				 dp + 1, error, z2->runtime_queue_empty);
			if (z2->runtime_queue_empty ||
			    (!z2->runtime_frame_valid &&
			     !z2->runtime_control_reply))
				break;
		}
	}

	if (d21_j71b_order) {
		/*
		 * J71b order: switch the touch IRQ to level-low, then let the
		 * line settle before the final handshake.  The J172 report
		 * writes are not part of this choreography; the reference
		 * probes 0x9d and skips them when the image does not
		 * implement it.
		 */
		if (d21_irq_level && !z2->stopping) {
			z2->saved_irq_type = irq_get_trigger_type(z2->spidev->irq);
			if (!z2->saved_irq_type ||
			    irq_set_irq_type(z2->spidev->irq, IRQ_TYPE_LEVEL_LOW)) {
				dev_warn(dev, "D21: could not set level-low IRQ\n");
			} else {
				z2->irq_type_changed = true;
				dev_info(dev,
					 "D21: touch IRQ -> level-low (was %u)\n",
					 z2->saved_irq_type);
			}
		}

		msleep(d21_settle_ms);

		error = apple_z2_d21_z2_cmd_locked(z2, APPLE_Z2_D21_CMD_WAKE, 0,
					       "d21-late-wake");
		dev_info(dev, "D21 init: late wake err=%d\n", error);
		usleep_range(2200, 2300);

		status = 0; len = 0;
		error = apple_z2_d21_read_report_info_locked(z2, 0x9d, &status, &len,
							 "d21-9d-info");
		dev_info(dev, "D21 init: 9d info err=%d status=%u len=%u\n",
			 error, status, len);
		if (!error && status == 2 && !len) {
			dev_info(dev,
				 "D21 init: 0x9d unsupported -> no J172 9d/bf writes (REPORTS_READY)\n");
		} else if (d21_skip_j172_writes) {
			dev_info(dev, "D21 init: J172 9d/bf writes skipped\n");
		} else {
			error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
							     sizeof(rep9d), "d21-9d-1");
			dev_info(dev, "D21 init: 9d write1 err=%d\n", error);
			usleep_range(50500, 50700);
			error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
							     sizeof(rep9d), "d21-9d-2");
			dev_info(dev, "D21 init: 9d write2 err=%d\n", error);
			error = apple_z2_d21_write_report_locked(z2, 0xbf, repbf,
							     sizeof(repbf), "d21-bf");
			dev_info(dev, "D21 init: bf write err=%d\n", error);
		}

		/*
		 * The report-enable knobs stay available in this order too:
		 * D21 is D11-descended (its status frame is the D11 status
		 * report), so the D11 report-enable (0xe2) and the
		 * high-level state (0x72) may yet be the missing arms - but
		 * now they are written after the reads and the settle, which
		 * is the only ordering never tried.
		 */
		if (d21_af_val >= 0) {
			u8 af_data = d21_af_val & 0xff;

			error = apple_z2_d21_write_report_locked(z2, APPLE_Z2_D21_REPORT_ENABLE,
							     &af_data, sizeof(af_data),
							     "d21-af-write");
			dev_info(dev, "D21 init: af write %#x err=%d\n",
				 af_data, error);
		}
		if (d21_e2_val >= 0) {
			u8 e2_data = d21_e2_val & 0xff;

			error = apple_z2_d21_write_report_locked(z2,
							     APPLE_Z2_D21_D11_REPORT_ENABLE,
							     &e2_data, sizeof(e2_data),
							     "d21-e2-write");
			dev_info(dev, "D21 init: e2 write %#x err=%d\n",
				 e2_data, error);
		}
		if (d21_hl_state >= 0) {
			u8 high_level[8] = { };

			high_level[1] = d21_hl_state & 0xff;
			error = apple_z2_d21_write_report_locked(z2,
							     APPLE_Z2_D21_REPORT_HIGH_LEVEL,
							     high_level,
							     sizeof(high_level),
							     "d21-72-active");
			dev_info(dev, "D21 init: 72 set-state %u err=%d\n",
				 high_level[1], error);
			apple_z2_d21_probe_report(z2, APPLE_Z2_D21_REPORT_HIGH_LEVEL,
						  "d21-72-after");
		}

		dev_info(dev,
			 "D21 init: J71b order complete\n");
		return apple_z2_d21_probe_tail(z2);
	}

	if (d21_skip_j172_writes) {
		dev_info(dev, "D21 init: J172 9d/bf writes skipped\n");
	} else {
		error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
						     sizeof(rep9d), "d21-9d-1");
		dev_info(dev, "D21 init: 9d write1 err=%d\n", error);
		usleep_range(50500, 50700);
		error = apple_z2_d21_write_report_locked(z2, 0x9d, rep9d,
						     sizeof(rep9d), "d21-9d-2");
		dev_info(dev, "D21 init: 9d write2 err=%d\n", error);
		error = apple_z2_d21_write_report_locked(z2, 0xbf, repbf,
						     sizeof(repbf), "d21-bf");
		dev_info(dev, "D21 init: bf write err=%d\n", error);
	}

	status = 0; len = 0;
	error = apple_z2_d21_read_report_info_locked(z2, 0xaf, &status, &len,
						 "d21-af");
	dev_info(dev, "D21 init: af info err=%d status=%u len=%u\n",
		 error, status, len);

	/*
	 * Enable reporting, as the iPhone-family initialisation does: write the
	 * legacy report-enable (0xaf) and then the iPhone-family one (0xe2).
	 * Both values are overridable; -1 skips the write.
	 */
	if (d21_af_val >= 0) {
		u8 af_data = d21_af_val & 0xff;

		error = apple_z2_d21_write_report_locked(z2, APPLE_Z2_D21_REPORT_ENABLE,
						     &af_data, sizeof(af_data),
						     "d21-af-write");
		dev_info(dev, "D21 init: af write %#x err=%d\n", af_data, error);
	}

	if (d21_e2_val >= 0) {
		u8 e2_data = d21_e2_val & 0xff;

		error = apple_z2_d21_write_report_locked(z2,
						     APPLE_Z2_D21_D11_REPORT_ENABLE,
						     &e2_data, sizeof(e2_data),
						     "d21-e2-write");
		dev_info(dev, "D21 init: e2 write %#x err=%d\n", e2_data, error);
	}

	/*
	 * The controller reads its high-level power state back as Unknown until
	 * the host sets it, and nothing is surfaced while the state is
	 * undefined. Declare the UI live, as the host does once it is ready to
	 * consume input.
	 */
	if (d21_hl_state >= 0) {
		high_level[1] = d21_hl_state & 0xff;
		error = apple_z2_d21_write_report_locked(z2,
						     APPLE_Z2_D21_REPORT_HIGH_LEVEL,
						     high_level,
						     sizeof(high_level),
						     "d21-72-active");
		dev_info(dev, "D21 init: 72 set-state %u err=%d\n",
			 high_level[1], error);
	}
	apple_z2_d21_probe_report(z2, APPLE_Z2_D21_REPORT_HIGH_LEVEL,
				  "d21-72-after");

	/*
	 * The controller's interrupt line is level-low in hardware, and the
	 * working J71b (same HBPP generation) runtime switches the trigger to
	 * level-low before enabling it.  With the device tree's falling-edge
	 * trigger an already-asserted line is never serviced, so queued status
	 * frames are never drained.  Mirror the reference here; the original
	 * trigger is restored on stop.
	 */
	if (d21_irq_level) {
		z2->saved_irq_type = irq_get_trigger_type(z2->spidev->irq);
		if (!z2->saved_irq_type ||
		    irq_set_irq_type(z2->spidev->irq, IRQ_TYPE_LEVEL_LOW)) {
			dev_warn(dev, "D21: could not set level-low IRQ\n");
		} else {
			z2->irq_type_changed = true;
			dev_info(dev,
				 "D21: touch IRQ -> level-low (was %u)\n",
				 z2->saved_irq_type);
		}
	}

	return apple_z2_d21_probe_tail(z2);
}

/*
 * Diagnostics that run after either init order: the report-table sweep (what
 * changes under a finger), the focused single-report probe and the
 * interrupt-data poll harness.
 */
static int apple_z2_d21_probe_tail(struct apple_z2_d21 *z2)
{
	struct device *dev = &z2->spidev->dev;
	unsigned int i;
	int error;

	mutex_lock(&d21_now_lock);
	d21_z2_now = z2;
	mutex_unlock(&d21_now_lock);

	/*
	 * Sample the controller's own view of itself: the high-level state
	 * and the scan timing counters, the latter repeatedly so an advancing
	 * counter shows the sensing engine is running.
	 */
	if (apple_z2_d21_probe_reports) {
		struct apple_z2_d21_cand {
			u8 id;
			u16 len;
			u8 prev[400];
		} *cand;
		unsigned int n = 0;
		unsigned int k;

		/*
		 * Find the report that actually carries contacts. Walk the
		 * whole table and then read every report that exists, logging
		 * the ones that change: a contact report changes under a
		 * finger, counters change all the time, and anything that
		 * stays put is inert data.  The comparison and the dumps cover
		 * the whole payload - an 8-byte-prefix diff silently missed
		 * contact data living deeper in a report.
		 */
		cand = kcalloc(0x100, sizeof(*cand), GFP_KERNEL);
		if (cand) {
			for (i = 1; i < 0x100; i++) {
				u8 cand_status = 0;
				u16 cand_len = 0;

				error = apple_z2_d21_read_report_info_locked(z2, i,
									 &cand_status,
									 &cand_len,
									 "d21-cand");
				/*
				 * Keep 1-byte reports too: the iOS userspace
				 * arming toggles a single "detection options"
				 * byte (id 0xc8 on the wire), so a len==1
				 * report is exactly the shape to watch.
				 */
				if (!error && cand_len >= 1 &&
				    cand_len <= sizeof(cand->prev)) {
					/*
					 * Log the table itself: the table is
					 * dynamic (it grew after the J71b
					 * choreography), so every "does id X
					 * exist" question is only valid for
					 * the exact init state that produced
					 * the answer.
					 */
					dev_info(dev,
						 "D21 table %02x: status=%u len=%u\n",
						 i, cand_status, cand_len);
					cand[n].id = i;
					cand[n].len = cand_len;
					memset(cand[n].prev, 0,
					       sizeof(cand[n].prev));
					n++;
				}
			}
			dev_info(dev, "D21: watching %u reports\n", n);

			for (i = 0; i < 120; i++) {
				for (k = 0; k < n; k++) {
					unsigned int seg;
					bool changed;

					if (!cand[k].len)
						continue;
					error = apple_z2_d21_read_report_locked(z2,
									    cand[k].id,
									    cand[k].len,
									    "d21-cand-read");
					if (error) {
						cand[k].len = 0;
						continue;
					}
					changed = memcmp(cand[k].prev,
							 z2->rx_buf + 3,
							 cand[k].len) != 0;
					if (!changed)
						continue;
					dev_info(dev, "D21 cand %02x changed (len %u):\n",
						 cand[k].id, cand[k].len);
					dev_info(dev, "  was: %*ph\n",
						 (int)cand[k].len,
						 cand[k].prev);
					for (seg = 0; seg < cand[k].len;
					     seg += 48) {
						dev_info(dev, "  now: %*ph\n",
							 (int)min(48u,
								  cand[k].len - seg),
							 z2->rx_buf + 3 + seg);
					}
					memcpy(cand[k].prev, z2->rx_buf + 3,
					       cand[k].len);
				}
				msleep(100);
			}
			kfree(cand);
		}
		apple_z2_d21_probe_report(z2, 0x72, "d21-72-state");
	}

	/*
	 * Focused diagnostic: read one report id repeatedly so a counter report
	 * (e.g. 0xf8) can be watched with and without a finger on the panel,
	 * independently of the interrupt channel.
	 */
	if (d21_probe_id >= 0 && d21_probe_id <= 0xff) {
		unsigned int k;

		for (k = 0; k < d21_probe_times; k++) {
			apple_z2_d21_probe_report(z2, d21_probe_id, "d21-probe");
			msleep(100);
		}
	}

	/*
	 * Focused watcher: full-payload dumps of the named reports, logged
	 * only when the payload changes.  The generic sweep floods the ring
	 * buffer so hard that the touch window falls out of it; this keeps a
	 * touch-vs-idle diff readable.
	 */
	if (d21_watch_ids && d21_watch_ids[0]) {
		u8 ids[16];
		u8 (*prev)[400];
		bool seen[16] = { };
		unsigned int n = 0;
		unsigned int k, it, seg;
		char *p = d21_watch_ids;

		while (*p && n < ARRAY_SIZE(ids)) {
			ids[n] = (u8)simple_strtoul(p, &p, 16);
			if (*p == ',')
				p++;
			n++;
		}
		prev = kcalloc(n, sizeof(*prev), GFP_KERNEL);
		if (prev) {
			for (it = 0; it < d21_probe_times; it++) {
				for (k = 0; k < n; k++) {
					u8 st = 0;
					u16 len = 0;

					error = apple_z2_d21_read_report_info_locked(z2, ids[k], &st, &len, "watch");
					if (error || !len || len > 400)
						continue;
					error = apple_z2_d21_read_report_locked(z2, ids[k], len, "watch");
					if (error)
						continue;
					if (seen[k] &&
					    !memcmp(prev[k], z2->rx_buf + 3, len))
						continue;
					dev_info(dev, "D21 watch %02x (len %u)%s:\n",
						 ids[k], len,
						 seen[k] ? " changed" : " base");
					for (seg = 0; seg < len; seg += 48)
						dev_info(dev, "  %*ph\n",
							 (int)min(48u, len - seg),
							 z2->rx_buf + 3 + seg);
					memcpy(prev[k], z2->rx_buf + 3, len);
					seen[k] = true;
				}
				msleep(100);
			}
			kfree(prev);
		}
	}

	/*
	 * Raw feature-space reads, run before the writes (iOS order: the
	 * property reads happen on open, the touch-mode write on unlock):
	 * the property reports AppleMultitouchHIDService reads on every open
	 * (0xd1 family id, 0xd3 rows/cols/bcd/max-packet, 0xd0 region
	 * descriptor, 0xa1 region param, 0x7f critical errors, 0xd9 surface)
	 * answer here even though GET_REPORT_INFO reports them as not valid.
	 */
	if (d21_read_ids && d21_read_ids[0])
		apple_z2_d21_read_list(z2, d21_read_ids);

	/* Default D21 touch arming - see the arm= param comment. */
	if (d21_arm) {
		static const u8 arm_c8[] = { 0x09 };
		static const u8 arm_bf[] = { 0x9f, 0x83, 0x0b, 0x02,
					     0x00, 0x10, 0x00, 0x00 };

		error = apple_z2_d21_write_report_locked(z2, 0xc8, arm_c8,
						     sizeof(arm_c8),
						     "d21-arm-c8");
		dev_info(dev, "D21 arm c8 write: err=%d\n", error);
		error = apple_z2_d21_write_report_locked(z2, 0xbf, arm_bf,
						     sizeof(arm_bf),
						     "d21-arm-bf");
		dev_info(dev, "D21 arm bf write: err=%d\n", error);
	}

	/*
	 * Generic post-init report writes, applied after the whole init
	 * choreography and with the IRQ already live - the position where the
	 * iOS userspace arming lands (backboardd's MultitouchHID toggles a
	 * one-byte "detection options" value and sends the touch mode only
	 * once the device is open and on).  Each entry is id:hexpayload,
	 * at most 11 payload bytes (CTRL_WRITE_SHORT), e.g.:
	 *   insmod apple_z2_d21.ko write=c8:01,c8:03
	 */
	if (d21_write && d21_write[0]) {
		char *buf, *entry;

		buf = kstrdup(d21_write, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		for (entry = buf; entry && *entry; ) {
			u8 data[11];
			unsigned int id, len = 0;
			char *comma, *colon;

			comma = strpbrk(entry, ",");
			if (comma)
				*comma++ = '\0';
			colon = strchr(entry, ':');
			if (!colon) {
				dev_err(dev, "D21 write: bad entry '%s'\n",
					entry);
				entry = comma;
				continue;
			}
			*colon++ = '\0';
			id = simple_strtoul(entry, NULL, 16);
			while (colon[0] && colon[1] && len < sizeof(data)) {
				int hi = hex_to_bin(colon[0]);
				int lo = hex_to_bin(colon[1]);

				if (hi < 0 || lo < 0)
					break;
				data[len++] = (u8)((hi << 4) | lo);
				colon += 2;
			}
			error = apple_z2_d21_write_report_locked(z2, id, data, len,
							     "d21-write");
			dev_info(dev, "D21 write %02x len %u: err=%d\n",
				 id, len, error);
			entry = comma;
		}
		kfree(buf);
	}

	/*
	 * Read-modify-write entries ("rmw=id:len", applied after the writes):
	 * read len bytes of the report and write the same payload straight
	 * back - the pattern of the userspace plugin's ReceiverProximityReset
	 * property (getReport(0xa6) then setReport(0xa6) of the u32 just
	 * read), and a generic "poke" for reports whose write semantics are
	 * unknown.
	 */
	if (d21_rmw && d21_rmw[0]) {
		char *buf, *entry;

		buf = kstrdup(d21_rmw, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		for (entry = buf; entry && *entry; ) {
			u8 data[32];
			unsigned int id, len;
			char *comma, *colon;

			comma = strpbrk(entry, ",");
			if (comma)
				*comma++ = '\0';
			colon = strchr(entry, ':');
			if (!colon) {
				dev_err(dev, "D21 rmw: bad entry '%s'\n",
					entry);
				entry = comma;
				continue;
			}
			*colon++ = '\0';
			id = simple_strtoul(entry, NULL, 16);
			len = simple_strtoul(colon, NULL, 10);
			if (!len || len > sizeof(data)) {
				dev_err(dev, "D21 rmw: bad len %u\n", len);
				entry = comma;
				continue;
			}
			error = apple_z2_d21_read_report_locked(z2, id, len,
							    "d21-rmw-read");
			dev_info(dev, "D21 rmw %02x read len %u: err=%d\n",
				 id, len, error);
			if (error) {
				entry = comma;
				continue;
			}
			if (z2_reply_log)
				dev_info(dev, "d21-rmw-read %02x: %*ph\n",
					 id,
					 (int)min_t(size_t, len + 5U, 40U),
					 z2->rx_buf);
			/* write_report_locked clobbers rx_buf - copy first */
			memcpy(data, z2->rx_buf + 3, len);
			error = apple_z2_d21_write_report_locked(z2, id, data,
							     len,
							     "d21-rmw-write");
			dev_info(dev, "D21 rmw %02x write len %u: err=%d\n",
				 id, len, error);
			entry = comma;
		}
		kfree(buf);
	}

	if (d21_read_after && d21_read_ids && d21_read_ids[0])
		apple_z2_d21_read_list(z2, d21_read_ids);



	/*
	 * Poll the interrupt-data queue directly.  If the controller has frames
	 * queued under a finger but the edge-triggered IRQ never delivers them,
	 * this shows it: apple_z2_d21_read_packet logs irq-rx/irq-pkt per poll.
	 */
	if (d21_poll) {
		unsigned int k;

		for (k = 0; k < d21_probe_times; k++) {
			error = apple_z2_d21_read_packet(z2);
			dev_info(dev, "D21 poll %u: err=%d valid=%d\n", k, error,
				 z2->runtime_frame_valid);
			if (d21_poll_watch) {
				u8 st = 0;
				u16 wlen = 0;
				static u8 prev[512];
				static bool have;

				error = apple_z2_d21_read_report_info_locked(z2,
					 d21_poll_watch, &st, &wlen,
					 "poll-watch");
				if (!error && wlen && wlen <= 512 &&
				    wlen <= d21_poll_watch_len) {
					error = apple_z2_d21_read_report_locked(z2,
						d21_poll_watch, wlen,
						"poll-watch-read");
					if (!error && (!have ||
					    memcmp(prev, z2->rx_buf + 3,
						   wlen))) {
						unsigned int seg;

						dev_info(dev,
							 "poll-watch %02x (len %u)%s:\n",
							 d21_poll_watch, wlen,
							 have ? " changed" :
								" base");
						for (seg = 0; seg < wlen;
						     seg += 48)
							dev_info(dev, "  %*ph\n",
							      (int)min_t(size_t,
								 wlen - seg,
								 48U),
							      z2->rx_buf + 3 +
								      seg);
						memcpy(prev, z2->rx_buf + 3,
						       wlen);
						have = true;
					}
				}
			}
			msleep(100);
		}
	}
	return 0;
}

/*
 * Write one 32-bit word to controller memory via the HBPP blob path, framed
 * exactly like the calibration packets (see apple_z2_d21_build_iphone7_cal):
 * 12-byte header, one payload word byte-swapped for the 16-bit SPI word
 * order (bpw16_min_len is 1 on D21), trailing checksum.
 */
static int apple_z2_d21_poke32(struct apple_z2_d21 *z2, u32 address, u32 value)
{
	u8 buf[12 + 4 + 4];
	u16 header_checksum;
	u32 checksum;
	int i, error;

	put_unaligned_be16(0x18e1, buf);
	put_unaligned_be16(APPLE_Z2_D21_HBPP_CMD_BLOB, buf + 2);
	put_unaligned_be16(0, buf + 4);		/* payload words - 1 */
	put_unaligned_be16(address & 0xffff, buf + 6);
	put_unaligned_be16(address >> 16, buf + 8);
	header_checksum = 0;
	for (i = 4; i < 10; i++)
		header_checksum += buf[i];
	put_unaligned_be16(header_checksum, buf + 10);

	put_unaligned_le32(value, buf + 12);
	checksum = buf[12] + buf[13] + buf[14] + buf[15];
	swap(buf[12], buf[13]);
	swap(buf[14], buf[15]);
	put_unaligned_be16(checksum & 0xffff, buf + 16);
	put_unaligned_be16(checksum >> 16, buf + 18);

	error = apple_z2_d21_send_firmware_blob(z2, buf, sizeof(buf), false, false);
	if (!error && !z2->last_blob_ack)
		dev_warn(&z2->spidev->dev,
			 "D21 poke %08x REJECTED (ack 0000) - boot will run the "
			 "pre-poke metronome; reload to re-roll\n", address);
	return error;
}

/*
 * Read one 32-bit word from controller memory via the HBPP register-read
 * packet, framed exactly like AppleMultitouchZ2SPI::readRegisterValue
 * (kernelcache 20H392, text 0x59b7564): an 8-byte packet 1c 73 + the address
 * split into two 16-bit wire-swapped halves + a BE16 byte-sum of the address
 * bytes, then the long ATN_ACK transaction (1a a1 18 e1 18 e1 18 e1) whose
 * response carries the register value in the same 16-bit wire order the poke
 * packets use.  The 8-byte response to the command packet itself is ignored,
 * again matching the kext.
 */
static int apple_z2_d21_regread32(struct apple_z2_d21 *z2, u32 address, u32 *value)
{
	u8 cmd[8];
	u8 cmd_reply[8];
	struct spi_transfer cmd_xfer = {
		.tx_buf = cmd,
		.rx_buf = cmd_reply,
		.len = 8,
		.bits_per_word = 8,
	};
	struct spi_transfer ack_xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = 8,
		.bits_per_word = 8,
	};
	u16 sum;
	int error;

	cmd[0] = 0x1c;
	cmd[1] = 0x73;
	cmd[2] = address >> 8;
	cmd[3] = address;
	cmd[4] = address >> 24;
	cmd[5] = address >> 16;
	sum = cmd[2] + cmd[3] + cmd[4] + cmd[5];
	cmd[6] = sum >> 8;
	cmd[7] = sum;

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	z2->tx_buf[2] = 0x18;
	z2->tx_buf[3] = 0xe1;
	z2->tx_buf[4] = 0x18;
	z2->tx_buf[5] = 0xe1;
	z2->tx_buf[6] = 0x18;
	z2->tx_buf[7] = 0xe1;

	error = spi_sync_transfer(z2->spidev, &cmd_xfer, 1);
	if (error)
		return error;
	error = spi_sync_transfer(z2->spidev, &ack_xfer, 1);
	if (error)
		return error;

	dev_info(&z2->spidev->dev, "D21 regread %08x cmd-rx: %*ph ack-rx: %*ph\n",
		 address, 8, cmd_reply, 8, z2->rx_buf);

	*value = (z2->rx_buf[2] << 8 | z2->rx_buf[3]) |
		 (u32)(z2->rx_buf[4] << 8 | z2->rx_buf[5]) << 16;
	return 0;
}

/*
 * Apply 'addr:val[,addr:val]' hex pairs via the blob path.  The C1F5E,4
 * fwconfig Boot Sequence carries five register pokes, then the calibration
 * request, then one more (0x10003060/5c/58, 0x10003000, 0x10003458, cal,
 * 0x10003300) - scan-engine configuration the D21 container does not repeat
 * as load records, so it can be replayed with these knobs.
 *
 * Timing note (measured 2026-10-30): on some boots the controller rejects
 * the FIRST poke with an all-zero ack when it is sent within microseconds
 * of the container's readiness probe; rejected boots run the pre-poke
 * ~1.93 s metronome.  A short settle delay before the list removes the
 * rejection.  Do NOT retry a rejected poke: a re-send mid-boot-sequence
 * leaves the delivery engine fully silent (measured - worse than the
 * metronome).  Log the rejection and move on; a reload re-rolls the boot.
 */
static int apple_z2_d21_poke_list(struct apple_z2_d21 *z2, const char *spec,
				  const char *tag)
{
	char *buf, *entry;
	int error = 0;

	if (!spec || !spec[0])
		return 0;
	buf = kstrdup(spec, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (d21_poke_delay_ms)
		msleep(d21_poke_delay_ms);
	for (entry = buf; entry && *entry; ) {
		char *comma = strpbrk(entry, ",");
		char *colon;
		u32 addr, val;

		if (comma)
			*comma++ = '\0';
		colon = strchr(entry, ':');
		if (!colon) {
			dev_err(&z2->spidev->dev, "D21 poke: bad entry '%s'\n",
				entry);
			entry = comma;
			continue;
		}
		*colon++ = '\0';
		addr = simple_strtoul(entry, NULL, 16);
		val = simple_strtoul(colon, NULL, 16);
		error = apple_z2_d21_poke32(z2, addr, val);
		dev_info(&z2->spidev->dev, "D21 poke %s %08x:%08x err=%d\n",
			 tag, addr, val, error);
		if (error)
			break;
		if (d21_poke_readback) {
			u32 got = 0;
			int rerr;

			/*
			 * Known useless in fw mode: the read packet returns the
			 * pull-response echo, not register data (the kext only
			 * reads registers from the bootloader).  Kept for the
			 * record; do not trust got vs val.
			 */
			rerr = apple_z2_d21_regread32(z2, addr, &got);
			dev_info(&z2->spidev->dev,
				 "D21 poke-readback %s %08x wrote=%08x fw-read=%08x err=%d\n",
				 tag, addr, val, got, rerr);
		}
		entry = comma;
	}
	kfree(buf);
	return error;
}

/*
 * Send the container's calibration blobs, but only after the rest of it.
 *
 * The container lists calibration before the boot sequence. The boot sequence is
 * what asks for it though: it ends with a RequestCalibration action that puts
 * the controller into the state where it accepts and retains calibration data.
 * Sending the blobs first, as replaying the container in its written order does,
 * leaves the controller uncalibrated - and an uncalibrated controller does not
 * report contacts. Apple's own configuration for this personality (the
 * C1F5E,3 entry of D21.mtprops, which is what iOS feeds the same controller)
 * lists the boot sequence first and the calibration sequence after it, so match
 * that ordering here.
 */
static int apple_z2_d21_send_late_calibration(struct apple_z2_d21 *z2,
					  const struct firmware *fw)
{
	size_t fw_idx = sizeof(struct apple_z2_d21_fw_hdr);
	u32 load_cmd;
	u32 size;

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;
		if (z2->stopping || time_after(jiffies, z2->boot_deadline)) {
			dev_warn(&z2->spidev->dev,
				 "firmware load aborted (%s at %u/%u)\n",
				 z2->stopping ? "stopping" : "boot deadline",
				 (unsigned int)fw_idx, (unsigned int)fw->size);
			return -ETIMEDOUT;
		}

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		if (load_cmd == APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION_V2) {
			const u8 *data __free(kfree) = NULL;
			const char *property;
			u32 address, max_size, provider;
			size_t len;
			int error;

			if (size != sizeof(struct apple_z2_d21_fw_calibration) ||
			    size > fw->size - fw_idx)
				return -EINVAL;

			address = get_unaligned_le32(fw->data + fw_idx);
			max_size = get_unaligned_le32(fw->data + fw_idx +
						      sizeof(__le32));
			provider = get_unaligned_le32(fw->data + fw_idx +
						      2 * sizeof(__le32));
			property = apple_z2_d21_calibration_property(provider);
			if (!property || !max_size)
				return -EINVAL;

			data = apple_z2_d21_build_iphone7_cal(z2, property, address,
							  max_size, &len);
			if (IS_ERR(data))
				return PTR_ERR(data);

			dev_info(&z2->spidev->dev,
				 "D21: calibration %s -> 0x%08x after boot sequence\n",
				 property, address);
			error = apple_z2_d21_send_firmware_blob(z2, data, len, false,
							    false);
			if (error)
				return error;
		}

		fw_idx = round_up(fw_idx + size, 4);
	}

	/*
	 * D21: iOS's AppleMultitouchZ2SPI also bootloads the in-pocket prox
	 * calibration (syscfg PxCl, 144 B on this unit) to controller SRAM.
	 * The target address comes from the kext's Z2 defaults initializer
	 * (kernelcache 20H392, MTSPI defaults fn): prox_cal_addr = 0x00400180
	 * (in DTCM's calibration region, below orb-gap-cal at 0x400568),
	 * alongside cal_dl_addr = 0x00400200 and fw_execute_addr =
	 * 0x10001800.  The DT properties that could override these
	 * (fll-mval, fll-mval-addr, cal-dl-addr, fw-execute-addr,
	 * prox-cal-addr) are ABSENT from the D21 ADT, so iOS on this unit
	 * runs the defaults, and the container (C1F5E,3) declares no prox
	 * provider - the fw-loop never carries it.  The kext sends PxCl as
	 * HBPP DATA packets (hdr 0xe118, <= 0x3f0 chunks, status 0x4bc1)
	 * BEFORE the EXECUTE packet; post-boot is our proven equivalent
	 * (the four cals land and take effect through the same path).
	 * The earlier target 0x10009600 (the makez2fw GEN_1 slot guess)
	 * acked e1 0e but is a region the firmware never reads - three
	 * fully-calibrated boots still rolled the bottom-region-deaf
	 * mode lottery with the in-pocket baseline effectively absent.
	 * Without PxCl the firmware's in-pocket baseline subsystem runs
	 * on absent data - the lead suspect for the phantom/guard boot
	 * modes and the bottom-region (pocket-gradient) delivery loss
	 * (docs/phantom-pocketguard-research.md).  Optional and non-fatal:
	 * with neither the DT property nor apple/z2-proxcal.bin present,
	 * or on a rejected send, the boot proceeds exactly as before.
	 * Never zero-fills blind into controller SRAM.  A pre-fw attempt
	 * (GEN_1 ordering) acked 1f 01 but derailed the firmware download
	 * (-EPROTO, 2026-11-04), so this must stay after the fw.
	 */
	if (apple_z2_d21_is_d21(z2) && d21_prox_cal) {
		const u8 *px __free(kfree) = NULL;
		bool have = device_property_present(&z2->spidev->dev,
						    APPLE_Z2_D21_PROX_CAL_PROP_NAME);
		size_t pxlen = 0;
		int perr;

		if (!have) {
			const struct firmware *pxfw __free(firmware) = NULL;

			have = !request_firmware(&pxfw, "apple/z2-proxcal.bin",
						 &z2->spidev->dev);
		}
		if (!have) {
			dev_warn(&z2->spidev->dev,
				 "D21: no prox calibration (apple,z2-prox-cal-blob / apple/z2-proxcal.bin), in-pocket baseline not programmed\n");
		} else {
			px = apple_z2_d21_build_iphone7_cal(z2,
							APPLE_Z2_D21_PROX_CAL_PROP_NAME,
							0x00400180, 0x400,
							&pxlen);
			if (IS_ERR(px)) {
				dev_warn(&z2->spidev->dev,
					 "D21: prox calibration build failed (%ld), skipping\n",
					 PTR_ERR(px));
			} else {
				perr = apple_z2_d21_send_firmware_blob(z2, px,
								   pxlen, false,
								   false);
				if (perr)
					dev_warn(&z2->spidev->dev,
						 "D21: prox calibration send failed (%d), continuing\n",
						 perr);
				else
					dev_info(&z2->spidev->dev,
						 "D21: prox calibration -> 0x00400180 (%zu bytes)\n",
						 pxlen);
			}
		}
	}

	return 0;
}

static int apple_z2_d21_upload_firmware(struct apple_z2_d21 *z2)
{
	const struct apple_z2_d21_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	size_t next_idx;
	size_t size;
	u32 load_cmd;
	u32 address;
	u32 timeout_ms;
	bool defer_irq_wait;
	bool init;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return dev_err_probe(&z2->spidev->dev, error,
				     "unable to load firmware\n");
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_d21_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_D21_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return dev_err_probe(&z2->spidev->dev, -EINVAL,
				     "invalid firmware header\n");

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;
		if (z2->stopping || time_after(jiffies, z2->boot_deadline)) {
			dev_warn(&z2->spidev->dev,
				 "firmware load aborted (%s at %u/%u)\n",
				 z2->stopping ? "stopping" : "boot deadline",
				 (unsigned int)fw_idx, (unsigned int)fw->size);
			return -ETIMEDOUT;
		}

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		switch (load_cmd) {
		case APPLE_Z2_D21_LOAD_COMMAND_INIT_PAYLOAD:
		case APPLE_Z2_D21_LOAD_COMMAND_SEND_BLOB: {
			const u8 *blob = fw->data + fw_idx;

			if (size > fw->size - fw_idx)
				return -EINVAL;
			init = load_cmd == APPLE_Z2_D21_LOAD_COMMAND_INIT_PAYLOAD;
			next_idx = round_up(fw_idx + size, 4);
			defer_irq_wait = next_idx <= fw->size &&
				fw->size - next_idx >= 2 * sizeof(__le32) &&
				get_unaligned_le32(fw->data + next_idx) ==
					APPLE_Z2_D21_LOAD_COMMAND_WAIT_IRQ;
			error = apple_z2_d21_send_firmware_blob(z2, blob, size, init,
							    defer_irq_wait);
			if (error)
				return error;
			fw_idx += size;
			break;
		}
		case APPLE_Z2_D21_LOAD_COMMAND_RAW_XFER:
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_d21_send_firmware_raw_xfer(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case APPLE_Z2_D21_LOAD_COMMAND_SET_CONFIG:
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_d21_apply_fw_config(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION: {
			const u8 *data __free(kfree) = NULL;

			address = size;
			data = apple_z2_d21_build_cal_blob(z2, APPLE_Z2_D21_CAL_PROP_NAME, address,
						       U32_MAX, &size, false);
			if (IS_ERR(data))
				return PTR_ERR(data);
			if (data) {
				error = apple_z2_d21_send_firmware_blob(z2, data, size, false, false);
				if (error)
					return error;
			}
			break;
		}
		case APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION_V2: {
			const u8 *data __free(kfree) = NULL;
			const char *property;
			u32 max_size;
			u32 provider;

			if (!apple_z2_d21_is_iphone7_plus(z2))
				return -EINVAL;
			if (size != sizeof(struct apple_z2_d21_fw_calibration) ||
			    size > fw->size - fw_idx)
				return -EINVAL;
			address = get_unaligned_le32(fw->data + fw_idx);
			max_size = get_unaligned_le32(fw->data + fw_idx + sizeof(__le32));
			provider = get_unaligned_le32(fw->data + fw_idx +
						      2 * sizeof(__le32));
			property = apple_z2_d21_calibration_property(provider);
			if (!property || !max_size)
				return -EINVAL;
			fw_idx += size;

			if (apple_z2_d21_is_d21(z2)) {
				/* Sent after the boot sequence requests it. */
				break;
			}

			data = apple_z2_d21_build_iphone7_cal(z2, property, address,
							  max_size, &size);
			if (IS_ERR(data)) {
				/*
				 * D21 gets these from syscfg via the loader; until
				 * that injection exists, run the panel without them
				 * rather than refusing to boot the controller.
				 */
				if (PTR_ERR(data) == -ENOENT &&
				    apple_z2_d21_is_d21(z2)) {
					dev_warn(&z2->spidev->dev,
						 "no %s, skipping\n", property);
					break;
				}
				return PTR_ERR(data);
			}
			error = apple_z2_d21_send_firmware_blob(z2, data, size,
							    false, false);
			if (error)
				return error;
			break;
		}
		case APPLE_Z2_D21_LOAD_COMMAND_WAIT_IRQ:
			if (size != sizeof(__le32) || size > fw->size - fw_idx)
				return -EINVAL;
			timeout_ms = get_unaligned_le32(fw->data + fw_idx);
			fw_idx += size;
			error = apple_z2_d21_wait_ready_irq(z2, timeout_ms);
			if (error)
				return error;
			if (apple_z2_d21_is_d21(z2)) {
				/* Keep ATN masked through the rest of boot. */
				apple_z2_d21_irq_off(z2);
				z2->upload_irq_masked = true;
			}
			if (apple_z2_d21_is_j172(z2)) {
				apple_z2_d21_irq_off(z2);
				error = apple_z2_d21_read_device_info_locked(z2);
				apple_z2_d21_irq_on(z2);
				if (error)
					return error;
			}
			break;
		default:
			return -EINVAL;
		}

		fw_idx = round_up(fw_idx, 4);
	}

	if (apple_z2_d21_is_d21(z2)) {
		mutex_lock(&z2->io_lock);
		error = apple_z2_d21_poke_list(z2, d21_poke, "pre-cal");
		if (!error)
			error = apple_z2_d21_send_late_calibration(z2, fw);
		if (!error)
			error = apple_z2_d21_poke_list(z2, d21_poke_after,
						       "post-cal");
		mutex_unlock(&z2->io_lock);
		if (error)
			return error;

		msleep(50);
		mutex_lock(&z2->io_lock);
		error = apple_z2_d21_init_locked(z2);
		if (!error) {
			z2->booted = true;
			error = apple_z2_d21_read_packet(z2);
			if (!error) {
				z2->upload_irq_masked = false;
				apple_z2_d21_irq_on(z2);
			}
		}
		mutex_unlock(&z2->io_lock);
		if (!error && d21_poll_hz) {
			INIT_DELAYED_WORK(&z2->d21_poll, apple_z2_d21_poll_work);
			schedule_delayed_work(&z2->d21_poll,
					      msecs_to_jiffies(1000 / d21_poll_hz));
			dev_info(&z2->spidev->dev, "D21: runtime poller at %u Hz\n",
				 d21_poll_hz);
		}
		return error;
	}

	if (apple_z2_d21_is_iphone7_plus(z2)) {
		msleep(50);
		mutex_lock(&z2->io_lock);
		error = apple_z2_d21_iphone7_plus_init_locked(z2);
		if (!error) {
			z2->booted = true;
			error = apple_z2_d21_read_packet(z2);
		}
		if (!error && apple_z2_d21_is_d11(z2) &&
		    (!z2->runtime_frame_valid ||
		     z2->rx_buf[5] != APPLE_Z2_D21_D11_STATUS_REPORT))
			error = -EPROTO;
		if (!error && apple_z2_d21_is_d11(z2))
			error = apple_z2_d21_enable_d11_reports_locked(z2);
		mutex_unlock(&z2->io_lock);
		return error;
	}

	z2->booted = true;
	mutex_lock(&z2->io_lock);
	error = apple_z2_d21_read_packet(z2);
	if (!error && apple_z2_d21_is_j172(z2) && !z2->runtime_frame_valid)
		error = -EPROTO;
	if (!error && apple_z2_d21_is_j172(z2))
		error = apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_DEVICE_INFO,
						  APPLE_Z2_D21_STATE_RUNTIME);
	if (!error && apple_z2_d21_is_j172(z2))
		error = apple_z2_d21_j172_init_reads_locked(z2);
	mutex_unlock(&z2->io_lock);
	if (error)
		return error;

	if (!apple_z2_d21_is_j172(z2))
		return 0;

	msleep(2260);
	mutex_lock(&z2->io_lock);
	error = apple_z2_d21_j172_init_writes_locked(z2);
	mutex_unlock(&z2->io_lock);
	return error;
}

static int apple_z2_d21_apply_initial_config(struct apple_z2_d21 *z2)
{
	const struct apple_z2_d21_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	u32 load_cmd;
	size_t size;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return error;
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_d21_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_D21_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return -EINVAL;

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;
		if (z2->stopping || time_after(jiffies, z2->boot_deadline)) {
			dev_warn(&z2->spidev->dev,
				 "firmware load aborted (%s at %u/%u)\n",
				 z2->stopping ? "stopping" : "boot deadline",
				 (unsigned int)fw_idx, (unsigned int)fw->size);
			return -ETIMEDOUT;
		}

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		if (load_cmd == APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION)
			continue;
		if (size > fw->size - fw_idx)
			return -EINVAL;
		if (load_cmd == APPLE_Z2_D21_LOAD_COMMAND_SEND_CALIBRATION_V2) {
			fw_idx = round_up(fw_idx + size, 4);
			continue;
		}
		if (load_cmd == APPLE_Z2_D21_LOAD_COMMAND_SET_CONFIG)
			return apple_z2_d21_apply_fw_config(z2,
						fw->data + fw_idx, size);
		if (load_cmd != APPLE_Z2_D21_LOAD_COMMAND_INIT_PAYLOAD &&
		    load_cmd != APPLE_Z2_D21_LOAD_COMMAND_SEND_BLOB &&
		    load_cmd != APPLE_Z2_D21_LOAD_COMMAND_RAW_XFER &&
		    load_cmd != APPLE_Z2_D21_LOAD_COMMAND_WAIT_IRQ)
			return -EINVAL;

		fw_idx = round_up(fw_idx + size, 4);
	}

	/*
	 * D21 (personality v3) firmware carries no SET_CONFIG command: its SPI
	 * timing comes from the device tree instead. Not having found one is
	 * only an error for the older layouts.
	 */
	if (apple_z2_d21_is_d21(z2))
		return 0;

	return -EINVAL;
}

static int apple_z2_d21_boot_preamble(struct apple_z2_d21 *z2, bool full_duplex)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.len = 4,
		.bits_per_word = 8,
	};
	int error;

	memset(z2->tx_buf, 0, xfer.len);
	if (full_duplex) {
		memset(z2->rx_buf, 0, xfer.len);
		xfer.rx_buf = z2->rx_buf;
	}

	apple_z2_d21_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	dev_info(&z2->spidev->dev, "D21 boot preamble reply: %*ph\n",
		 4, z2->rx_buf);

	return apple_z2_d21_advance_protocol(z2, APPLE_Z2_D21_STATE_POWERED,
					 APPLE_Z2_D21_STATE_SPI_CONFIGURED);
}

static int apple_z2_d21_iphone7_plus_post_boot_preamble(struct apple_z2_d21 *z2)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = 4,
		.bits_per_word = 8,
	};

	memset(z2->tx_buf, 0, xfer.len);
	memset(z2->rx_buf, 0, xfer.len);
	apple_z2_d21_apply_z2_delays(z2);

	{
		int ret = spi_sync_transfer(z2->spidev, &xfer, 1);

		dev_info(&z2->spidev->dev,
			 "D21 post-boot preamble reply: %*ph\n", 4, z2->rx_buf);
		return ret;
	}
}

static int apple_z2_d21_boot(struct apple_z2_d21 *z2)
{
	bool irq_enabled = false;
	int error;

	z2->booted = false;
	if (z2->stopping)
		/* Teardown won the race with a recovery restart. */
		return -ENODEV;
	z2->boot_edges = 0;
	z2->boot_deadline = jiffies + msecs_to_jiffies(60000);
	apple_z2_d21_reset_protocol(z2);
	error = apple_z2_d21_platform_power_on(z2);
	if (error)
		return error;

	if (apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) {
		error = apple_z2_d21_apply_initial_config(z2);
		if (error)
			goto err_stop;
	}
	if (apple_z2_d21_is_j172(z2)) {
		error = apple_z2_d21_boot_preamble(z2, false);
		if (error)
			goto err_stop;
	} else if (apple_z2_d21_is_iphone7_plus(z2)) {
		/* iPhone 7 Plus performs a legacy reset and zero preamble first. */
		error = apple_z2_d21_pulse_reset(z2);
		if (error)
			goto err_stop;
		error = apple_z2_d21_boot_preamble(z2, true);
		if (error)
			goto err_stop;

		/* Hold reset while arming the IRQ to discard the first boot edge. */
		gpiod_set_value_cansleep(z2->reset_gpio, 1);
		usleep_range(1000, 2000);
	}

	reinit_completion(&z2->boot_irq);
	apple_z2_d21_irq_on(z2);
	irq_enabled = true;
	if (apple_z2_d21_is_j172(z2)) {
		error = apple_z2_d21_pulse_reset(z2);
	} else if (apple_z2_d21_is_iphone7_plus(z2)) {
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		usleep_range(1000, 2000);
		error = 0;
	} else {
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		error = 0;
	}
	if (error)
		goto err_stop;

	if (!wait_for_completion_timeout(&z2->boot_irq,
					 msecs_to_jiffies(z2->boot_timeout_ms))) {
		error = -ETIMEDOUT;
		goto err_stop;
	}
	if (apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) {
		error = apple_z2_d21_advance_protocol(z2,
						  APPLE_Z2_D21_STATE_SPI_CONFIGURED,
						  APPLE_Z2_D21_STATE_BOOT_IRQ);
		if (error)
			goto err_stop;
	}
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		error = apple_z2_d21_iphone7_plus_post_boot_preamble(z2);
		if (error)
			goto err_stop;
	}

	if (apple_z2_d21_is_j172(z2) || apple_z2_d21_is_iphone7_plus(z2)) {
		apple_z2_d21_irq_off(z2);
		z2->upload_irq_masked = true;
	}

	if (z2->stopping)
		goto err_stop;

	error = apple_z2_d21_upload_firmware(z2);
	if (error)
		goto err_stop;

	/*
	 * D21: arm the no-frames watchdog (the Orchard 79f5e69 port).
	 * The D21 half-boot is SILENT - rail up, SPI ACKs, ZERO 0x44
	 * frames - so its only detector is the absence of valid frames
	 * while the controller owes us one.  quiet_ms (default 15 s)
	 * without one valid frame starts the recovery ladder, which ends
	 * in a full in-driver restart (SMC rBK3 off -> drain -> on +
	 * reset + fw/cal reload) instead of a module reload.
	 *
	 * "While the controller owes us one": an idle controller owes us
	 * nothing (no contact down, no interrupt edges), so both
	 * timestamps start at the arming point and the watchdog leaves an
	 * idle controller alone.
	 */
	if (apple_z2_d21_is_d21(z2) && !z2->stopping) {
		z2->d21_last_frame = jiffies;
		z2->d21_last_irq = jiffies;
		z2->d21_contact_down = false;
		z2->d21_recovering = false;
		if (d21_recover) {
			dev_info(&z2->spidev->dev,
				 "D21: recovery watchdog armed (quiet_ms=%u)\n",
				 d21_quiet_ms);
			schedule_delayed_work(&z2->d21_watch,
					      msecs_to_jiffies(APPLE_Z2_D21_WATCH_PERIOD_MS));
		}
	}

	return 0;

err_stop:
	z2->booted = false;
	/*
	 * A failed or teardown-raced boot must leave NO trace: the runtime
	 * level-low flip survives rmmod in the pinctrl/GPIO domain, and the
	 * next probe then fails with "type mismatch, failed to map hwirq"
	 * (-EINVAL) - the driver never binds again until a reboot.  This is
	 * the root cause of the "d21-touch auto-start is flaky" symptom
	 * (measured 2026-11-03: every reload after a raced in-driver restart
	 * failed -22 with an unbound module).
	 */
	if (z2->irq_type_changed) {
		irq_set_irq_type(z2->spidev->irq, z2->saved_irq_type);
		z2->irq_type_changed = false;
	}
	if (z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		apple_z2_d21_irq_on(z2);
	}
	if (irq_enabled)
		apple_z2_d21_irq_off(z2);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	apple_z2_d21_platform_power_off(z2);
	apple_z2_d21_reset_protocol(z2);
	return error;
}

static const struct apple_z2_d21_chip_info apple_z2_d21_info = {
	.name = "iPhone 8 Plus Touchscreen",
	.variant = APPLE_Z2_D21_VARIANT_D21,
};

static int apple_z2_d21_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct apple_z2_d21_chip_info *info;
	struct apple_z2_d21 *z2;
	unsigned int mt_flags = INPUT_MT_DIRECT;
	unsigned int slots;
	int cal_size;
	int error;

	info = &apple_z2_d21_info;

	z2 = devm_kzalloc(dev, sizeof(*z2), GFP_KERNEL);
	if (!z2)
		return -ENOMEM;

	/*
	 * The transmit side has to be able to hold a whole acknowledgement
	 * frame, not just a command: a data phase may span several frames.
	 */
	z2->tx_buf = devm_kzalloc(dev, APPLE_Z2_D21_RX_BUF_SIZE, GFP_KERNEL);
	if (!z2->tx_buf)
		return -ENOMEM;
	z2->rx_buf = devm_kzalloc(dev, APPLE_Z2_D21_RX_BUF_SIZE, GFP_KERNEL);
	if (!z2->rx_buf)
		return -ENOMEM;

	z2->spidev = spi;
	z2->variant = info->variant;
	z2->boot_timeout_ms = 20;
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		z2->z2_inter_packet_delay_us = 1000;
		z2->z2_cs_delay_us = 1000;
		z2->boot_timeout_ms = 500;
	}
	if (apple_z2_d21_is_d21(z2)) {
		/*
		 * Apple's configuration for this personality allows five
		 * seconds for the first boot after a reset, well beyond the
		 * half second the other iPhone parts use.
		 */
		z2->boot_timeout_ms = 5000;

		/*
		 * The SPI mode lives in the persistent spi_device across driver
		 * reloads, and this panel only answers in mode 3 (CPOL|CPHA, as
		 * the device tree declares).  Restore it explicitly so a
		 * mistuned experiment cannot wedge the bus; the module parameter
		 * can still override it for a deliberate test.
		 */
		spi->mode = (spi->mode & ~SPI_MODE_X_MASK) |
			    SPI_CPOL | SPI_CPHA;
		if (d21_spi_mode >= 0)
			spi->mode = (spi->mode & ~SPI_MODE_X_MASK) |
				    (d21_spi_mode & SPI_MODE_X_MASK);
		error = spi_setup(spi);
		if (error)
			return dev_err_probe(dev, error,
					     "cannot set SPI mode\n");

		/*
		 * Apple's own D21 profile (C1F5E,4 Firmware Config) carries a
		 * host-config block that the D21 container has no SET_CONFIG
		 * record for: inter-packet delay and minimum DMA transfer size.
		 * Apply them from the module parameters so a run can be
		 * re-valued without a rebuild.
		 */
		if (d21_ip_delay >= 0)
			z2->z2_inter_packet_delay_us = d21_ip_delay;
		if (d21_dma_min >= 0)
			z2->dma_min_len = d21_dma_min;
		dev_info(dev, "D21 host-config: ip-delay=%u dma-min=%u mode=%u\n",
			 z2->z2_inter_packet_delay_us, z2->dma_min_len,
			 (unsigned int)(spi->mode & SPI_MODE_X_MASK));
	}
	z2->bpw16_min_len = 1;
	device_property_read_u32(dev, "apple,z2-bpw16-min-len",
				 &z2->bpw16_min_len);
	if (!z2->bpw16_min_len)
		z2->bpw16_min_len = 1;
	z2->no_init_ack = device_property_read_bool(dev,
						    "apple,z2-no-init-ack");
	init_completion(&z2->boot_irq);
	mutex_init(&z2->io_lock);
	/*
	 * Always initialise the poller work here, even though it is only
	 * queued when poll_hz>0.  apple_z2_d21_stop() unconditionally cancels it,
	 * and cancel_delayed_work_sync() on an uninitialised work_struct
	 * trips a WARN in __flush_work() and can derail module unload - which
	 * leaves the module loaded, the reset line claimed, and the recovery
	 * scripts unable to reload (observed 2026-11-02).
	 */
	INIT_DELAYED_WORK(&z2->d21_poll, apple_z2_d21_poll_work);
	/* Same lesson as above: the recovery works are always initialised,
	 * because stop() unconditionally cancels them. */
	INIT_DELAYED_WORK(&z2->d21_rearm, apple_z2_d21_rearm_work);
	INIT_DELAYED_WORK(&z2->d21_watch, apple_z2_d21_watch_work);
	spi_set_drvdata(spi, z2);

	z2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(z2->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->reset_gpio),
				     "unable to get reset GPIO\n");
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		z2->hv_supply = devm_regulator_get(dev, "hv");
		if (IS_ERR(z2->hv_supply))
			return dev_err_probe(dev, PTR_ERR(z2->hv_supply),
					     "unable to get HV supply\n");

		z2->core_supply = devm_regulator_get(dev, "core");
		if (IS_ERR(z2->core_supply))
			return dev_err_probe(dev, PTR_ERR(z2->core_supply),
					     "unable to get core supply\n");
	}

	if (apple_z2_d21_is_d21(z2)) {
		/*
		 * The D21 ADT names the touch analog rail as
		 * function-power_ana = 98:dpLE(0x302) - a switch inside the
		 * "display-pmu,chestnut" I2C PMIC, the same rail the A10 parts
		 * model as their HV supply.  Optional: when the DT does not
		 * describe it, the rails stay as iBoot/iOS left them.
		 */
		z2->hv_supply = devm_regulator_get_optional(dev, "hv");
		if (IS_ERR(z2->hv_supply))
			z2->hv_supply = NULL;

		/*
		 * function-power_ldo = 75:pKW4('rBK3', 0x5020000) - the touch
		 * digital-core rail, provided by the SMC (smc-pmu).  Wired as
		 * an optional "power-ldo" supply so a reload is a true core
		 * power cycle; without a backing regulator this is a no-op.
		 */
		z2->ldo_supply = devm_regulator_get_optional(dev, "power-ldo");
		if (IS_ERR(z2->ldo_supply))
			z2->ldo_supply = NULL;

		/* the SMC owns the rail (macsmc); direct key writes beat a
		 * DTB regulator for the same purpose */
		z2->smc = apple_z2_d21_smc(dev);
		if (z2->smc)
			dev_info(dev, "SMC bound: rail key rBK3 available\n");
		else
			dev_warn(dev, "SMC not found; rail key writes disabled\n");
	}

	z2->power_ana_gpio =
		devm_gpiod_get_optional(dev, "power-ana", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ana_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ana_gpio),
				     "unable to get analog power GPIO\n");
	z2->power_ldo_gpio =
		devm_gpiod_get_optional(dev, "power-ldo", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ldo_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ldo_gpio),
				     "unable to get LDO power GPIO\n");
	z2->display_sync_gpio =
		devm_gpiod_get_optional(dev, "display-sync", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync_gpio),
				     "unable to get display-sync GPIO\n");
	z2->display_sync1_gpio =
		devm_gpiod_get_optional(dev, "display-sync1", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync1_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync1_gpio),
				     "unable to get display-sync1 GPIO\n");

	z2->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(z2->clk))
		return dev_err_probe(dev, PTR_ERR(z2->clk),
				     "unable to get clock\n");

	error = devm_request_threaded_irq(dev, spi->irq, NULL, apple_z2_d21_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  "apple-z2-irq", z2);
	if (error)
		return dev_err_probe(dev, error, "unable to request IRQ\n");

	error = device_property_read_string(dev, "firmware-name", &z2->fw_name);
	if (error)
		return dev_err_probe(dev, error, "unable to get firmware name\n");
	if (apple_z2_d21_is_iphone7_plus(z2)) {
		static const struct {
			const char *name;
			unsigned int max_size;
		} calibration_properties[] = {
			{ APPLE_Z2_D21_CAL_PROP_NAME, 0x708 },
			{ APPLE_Z2_D21_ORB_GAP_CAL_PROP_NAME, 0x3e0 },
			{ APPLE_Z2_D21_ORB_FORCE_CAL_PROP_NAME, 0x5c4 },
			{ APPLE_Z2_D21_SHAPE_ACCEL_CAL_PROP_NAME, 0xd0 },
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(calibration_properties); i++) {
			cal_size = device_property_count_u8(dev,
							    calibration_properties[i].name);
			if (cal_size <= 0 ||
			    cal_size > calibration_properties[i].max_size ||
			    (apple_z2_d21_is_d111(z2) &&
			     i == APPLE_Z2_D21_CAL_MULTI_TOUCH &&
			     cal_size != APPLE_Z2_D21_D111_CAL_SIZE))
				return dev_err_probe(dev, -EINVAL,
						     "invalid iPhone 7 Plus calibration %s size %d\n",
						     calibration_properties[i].name, cal_size);
		}
	}

	z2->input_dev = devm_input_allocate_device(dev);
	if (!z2->input_dev)
		return -ENOMEM;
	z2->input_dev->name = info->name;
	z2->input_dev->phys = "apple_z2";
	z2->input_dev->id.bustype = BUS_SPI;

	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	touchscreen_parse_properties(z2->input_dev, true, &z2->props);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_ORIENTATION, -32768, 32767,
			     0, 0);

	if (apple_z2_d21_is_j172(z2)) {
		slots = APPLE_Z2_D21_J172_MAX_CONTACTS;
	} else if (apple_z2_d21_is_iphone7_plus(z2)) {
		/*
		 * The iPhone report layout carries a finger key, and contacts
		 * are looked up by it, so the core has to track keys and retire
		 * the slots they were assigned to.
		 */
		slots = 10;
		mt_flags |= INPUT_MT_TRACK;
		/*
		 * The D11 (iPhone 7 Plus) controller and its D21 descendant
		 * (iPhone 8 Plus) omit a released contact from the following
		 * reports instead of always sending a state==0 lift.  Without
		 * DROP_UNUSED the slot stays down and the desktop keeps a stuck
		 * point after a release (observed on D21: a 1.3M contact held
		 * ~36 s at screen ~(617,1130) with no lift).  Orchard carries
		 * this for D11; D21 needs it for the same reason.
		 */
		if (apple_z2_d21_is_d11(z2) || apple_z2_d21_is_d21(z2))
			mt_flags |= INPUT_MT_DROP_UNUSED;
	} else {
		slots = 256;
	}
	error = input_mt_init_slots(z2->input_dev, slots, mt_flags);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to initialize multitouch slots\n");
	error = input_register_device(z2->input_dev);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to register input device\n");

	usleep_range(5000, 10000);
	return apple_z2_d21_boot(z2);
}

static void apple_z2_d21_stop(struct apple_z2_d21 *z2)
{
	/*
	 * The poller re-arms itself only while booted, but a work item may
	 * already be queued or running; drain it before the IRQ/protocol go
	 * away so it can't touch the controller mid-teardown.  The D21
	 * recovery works are drained first so a restart in flight cannot
	 * race the teardown (Orchard 79f5e69 stop ordering), and the IRQ
	 * goes through the tracked off (the ladder may have masked it).
	 */
	if (apple_z2_d21_is_d21(z2)) {
		mutex_lock(&z2->io_lock);
		z2->stopping = true;
		mutex_unlock(&z2->io_lock);
		cancel_delayed_work_sync(&z2->d21_rearm);
		cancel_delayed_work_sync(&z2->d21_watch);
	}
	cancel_delayed_work_sync(&z2->d21_poll);
	apple_z2_d21_irq_off(z2);
	if (z2->irq_type_changed) {
		irq_set_irq_type(z2->spidev->irq, z2->saved_irq_type);
		z2->irq_type_changed = false;
	}
	gpiod_direction_output(z2->reset_gpio, 1);
	z2->booted = false;
	apple_z2_d21_platform_power_off(z2);
	apple_z2_d21_reset_protocol(z2);
}

static void apple_z2_d21_remove(struct spi_device *spi)
{
	struct apple_z2_d21 *z2 = spi_get_drvdata(spi);

	/*
	 * Drop the runtime-knob handle under the lock first: a concurrent
	 * read_now=/kick= write is either rejected (-ENODEV) or has already
	 * finished with the struct by the time we return.
	 */
	mutex_lock(&d21_now_lock);
	if (d21_z2_now == z2)
		d21_z2_now = NULL;
	mutex_unlock(&d21_now_lock);
	apple_z2_d21_stop(z2);
}

static int apple_z2_d21_suspend(struct device *dev)
{
	apple_z2_d21_stop(spi_get_drvdata(to_spi_device(dev)));

	return 0;
}

static int apple_z2_d21_resume(struct device *dev)
{
	struct apple_z2_d21 *z2 = spi_get_drvdata(to_spi_device(dev));

	z2->stopping = false;
	return apple_z2_d21_boot(z2);
}

#endif /* _APPLE_Z2_D21_H */
