// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lightning charging for Apple devices with a Tristar (CBTL1610) port
 * controller and a D2365-family PMIC (D2365, the iPad Air 2's D2207 or the
 * A7 iPads' D2089).
 *
 * iBoot leaves the PMIC input-current limit at 100 mA. On every attachment
 * this driver identifies the source and grants what it advertises, up to the
 * board maximum:
 *   USB-C cable: CC level on the orientation-selected lane (1.5 A / 3 A)
 *   USB-A cable: Apple D+/D- divider (BrickID) on both Tristar mux banks
 *   BC1.2 charging port (CDP or DCP) on the oriented USB pair: 1.5 A
 *   USB host: what the USB device controller reports through vbus_draw
 *             above 500 mA (an Apple host's current grant)
 * Anything unidentified gets the fallback, the 500 mA of a USB 2.0 port.
 *
 * The mux shares the USB data lines, and mux changes have stalled USB data.
 * The USB device controller cannot see VBUS, so this driver is the port's
 * attach detector and owns its data role: USB data stays off from attach
 * until the source is identified, at most TRISTAR_HOLD_MS. A host's
 * advertisement (CDP, USB-C current) is therefore measured before its
 * session begins, and the mux is never sampled while USB data is on. A source event during a session re-reads only
 * the capability frame: the same cable in the same orientation keeps its
 * identification.
 *
 * Once the source is identified, the PMU events that can mean a source change
 * neither lower the input limit nor identify the source again, as in iOS 16:
 * they pause the ramp, and the status is read again 50 ms after the last one.
 * Only a lost input ends the grant. Before identification, such an event
 * restarts it. A grant ramps the way iOS does: 100 mA held for 100 ms, then
 * 100 mA steps 10 ms apart. ADC completion is an event of its own.
 *
 * Copyright The Orchard Linux contributors
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "apple_tristar_accessory.h"

/* PMIC connection state: bit 0 is "connected" on every PMIC */
#define TRISTAR_CHG_CONNECTED_NOW	BIT(0)
#define TRISTAR_CHG_ADC_START		BIT(7)	/* D2207 ADC control */
#define TRISTAR_CHG_ADC_LATCH		GENMASK(7, 3)	/* D2207 ADC control + 1 */

struct tristar_charger;
struct tristar_pack;

/* Keep shared charging policy independent of the PMIC input-limit registers. */
struct tristar_chip_info {
	int (*prepare)(struct tristar_charger *chg);
	bool (*pack_allows)(struct tristar_charger *chg, const struct tristar_pack *pack);
	unsigned int status_size, control_size;
	u8 adc_error_mask;
	bool strict_limit_errors;
	bool require_pack_telemetry;
	bool invalidate_on_line;
	int (*read_limit)(struct tristar_charger *chg, unsigned int *code);
	int (*write_limit)(struct tristar_charger *chg, unsigned int code);
	unsigned int (*code_ma)(struct tristar_charger *chg, unsigned int code);
	unsigned int (*ma_code)(struct tristar_charger *chg, unsigned int ma);
	int (*read_status)(struct tristar_charger *chg, unsigned int *status);
	unsigned int handoff_code;
	unsigned int attach_code;
	unsigned int online_status;
	unsigned int ramp_step_ms;
	unsigned int fallback_code;
};

/*
 * Where a PMIC keeps the charger registers, relative to the banks the
 * device tree gives, and how its ADC is driven. The functions are the same
 * on both PMICs; the addresses are not.
 */
static const struct tristar_chip_info tristar_d2365_chip;
static const struct tristar_chip_info tristar_d2355_chip;

struct tristar_pmic {
	const struct tristar_chip_info *chip;
	u8 connected;		/* status bank: connection state */
	u8 connected_new;	/* the bit that also marks a new connection */
	u8 status;		/* status bank: battery charger state */
	u8 input_limit;		/* control bank: input-current limit */
	u8 adc_start;		/* written to the ADC bank to start a conversion */
	u8 adc_result;		/* ADC bank: two result bytes */
	bool adc_rearm;		/* clear the start bit and latch after a read */
	unsigned int plug_event; /* the event that latches on plug and unplug */
};

/* D2365 (iPad 5, iPad 6), from iOS 16 AppleD2365PMUPowerSource */
static const struct tristar_pmic tristar_d2365 = {
	.chip = &tristar_d2365_chip,
	.connected = 0x09,
	.connected_new = BIT(1),
	.status = 0x0a,
	.input_limit = 0x03,
	.adc_start = 0x01,
	.adc_result = 0x01,
	.plug_event = 72,
};

/*
 * D2207 (iPad Air 2), from iOS 15.8 AppleD2207PMU: connection state 0x60,
 * charger state 0x62, input limit 0x4c0, ADC control 0x500 with the result
 * at 0x502. A conversion starts with bit 7 and the channel, 6 for the
 * Tristar's BrickID line; iOS clears the start bit and the latch in 0x501
 * after each read. The plug event is event 0.
 */
static const struct tristar_pmic tristar_d2207 = {
	.chip = &tristar_d2365_chip,
	.connected = 0x00,
	.connected_new = BIT(3),
	.status = 0x02,
	.input_limit = 0x00,
	.adc_start = TRISTAR_CHG_ADC_START | 6,
	.adc_result = 0x02,
	.adc_rearm = true,
	.plug_event = 0,
};

/*
 * D2089 (A7 iPads), from iOS 12 AppleD2089PMU and AppleD2089PMUPowerSource:
 * the D2207's layout. The power source reads the 13 status bytes at 0x60
 * and takes bit 0 of 0x60 as connected, needing bit 3 as well for a new
 * connection; it writes the input limit to 0x4c0 as (mA - 75) / 12.5. The
 * ADC starts with bit 7 and the channel in 0x500 (6 for the Tristar's
 * BrickID line, the ADT's function-brick_id_voltage), returns 12 bits at
 * 0x502 that iOS scales for channel 6 as this driver does, and is cleared
 * through 0x501 and 0x500 after each read. ADC done is event 89; the plug
 * event is event 0.
 */
static const struct tristar_pmic tristar_d2089 = {
	.chip = &tristar_d2365_chip,
	.connected = 0x00,
	.connected_new = BIT(3),
	.status = 0x02,
	.input_limit = 0x00,
	.adc_start = TRISTAR_CHG_ADC_START | 6,
	.adc_result = 0x02,
	.adc_rearm = true,
	.plug_event = 0,
};

/* D2355 reads connection state via its own status callback and a companion ADC. */
static const struct tristar_pmic tristar_d2355 = {
	.chip = &tristar_d2355_chip,
	.adc_start = 1,
	.adc_result = 1,
	.plug_event = 72,
};

/* Interrupts: the PMU line, ADC completion, then the source-change events. */
#define TRISTAR_IRQ_LINE		0
#define TRISTAR_IRQ_ADC			1
#define TRISTAR_IRQ_SOURCE		2
#define TRISTAR_IRQS_MAX		16

/* Input-current limit encoding: mA = code * 12.5 + 75 */
#define TRISTAR_CHG_HANDOFF_CODE	0x02	/* 100 mA, as iBoot leaves it */
#define TRISTAR_CHG_FALLBACK_CODE	0x22	/* 500 mA */

/* Tristar (CBTL1610) registers */
#define CBTL_MUX_ROUTE			0x01
#define CBTL_MUX_STATE			0x03	/* also the BC1.2 excitation */
#define CBTL_CAP_STATUS			0x0c
#define CBTL_CAP_RESPONSE		0x20
#define CBTL_CAP_LEN			8

#define TRISTAR_POLL_MS			100
/* Identify this long after an attach or a source event. */
#define TRISTAR_LOOK_MS			300
/*
 * USB data waits at most this long for an identification. A half-inserted
 * plug reads as no signature, so until then a missing one is looked for again.
 */
#define TRISTAR_HOLD_MS			1000
/*
 * The input-current ramp of iOS 16 (AppleARMPMUCharger): from nothing it
 * writes 100 mA and holds it for 100 ms, then steps up 100 mA at a time,
 * each step waiting a tenth of a millisecond per mA of the previous one.
 * Steps down go to the new target at once.
 */
#define TRISTAR_RAMP_HOLD_MS		100
#define TRISTAR_RAMP_STEP_MA		100
/* After the last PMU source event, the ramp waits this long, as iOS does. */
#define TRISTAR_EVENT_DEBOUNCE_MS	50
/* More source events than this per second means something is stuck. */
#define TRISTAR_IRQ_STORM		200

enum tristar_phase {
	TRISTAR_EMPTY,
	TRISTAR_SETTLING,
	TRISTAR_RAMPING,
	TRISTAR_CHARGING,
	TRISTAR_FAULT,
};

/* Capability-frame routes, as decoded by the Tristar firmware */
enum cbtl_route {
	CBTL_ROUTE_NONE,
	CBTL_ROUTE_USB0,
	CBTL_ROUTE_UART0,
	CBTL_ROUTE_MBUS,
	CBTL_ROUTE_USBC,
	CBTL_ROUTE_UART2,
};

struct tristar_charger {
	const struct tristar_chip_info *chip;
	void *chip_data;
	bool probe_ready;
	struct device *dev;
	struct regmap *regmap;
	struct regmap *adc_regmap;
	struct i2c_client *tristar;
	struct power_supply *psy;
	struct usb_role_switch *role_sw;
	struct workqueue_struct *wq;	/* frozen across system sleep */
	struct delayed_work poll_work;
	struct completion adc_done;
	int irqs[TRISTAR_IRQS_MAX];
	unsigned int nirqs;
	int plug_irq;			/* wakes the system, 0 without one */
	const struct tristar_pmic *pmic;
	u32 status_base;
	u32 control_base;
	u32 adc_base;
	unsigned int max_ma;
	struct iio_channel *vbus;
	bool vbus_fault;

	/* Serializes policy state and PMIC/Tristar sequences. */
	struct mutex lock;

	/*
	 * Event attribution: a new epoch invalidates an identification in
	 * progress. D2355 line and source handlers can run on separate IRQ
	 * threads and must each invalidate the grant under consideration.
	 */
	atomic_t epoch;
	bool adc_waiting;
	u64 storm_start_ns;
	unsigned int storm_count;
	/* Last source event on the PMU line after identification, 0: none */
	u64 line_ns;

	enum tristar_phase phase;
	bool online;
	bool present;
	bool stopping;
	unsigned int code;	/* last verified input limit */
	unsigned int target_code;
	unsigned int source_ma;
	/* USB data is on for this attachment: the mux must stay untouched. */
	bool usb_on;
	/* What the USB host allows (vbus_draw) */
	unsigned int usb_ma;
	/*
	 * From prepare to complete of a system sleep: the device controller's
	 * reports are its own suspend, not the host's.
	 */
	bool sleeping;
	/* A report held over sleep, applied at complete */
	bool usb_report_held;
	unsigned int usb_report_ma;
	enum power_supply_usb_type usb_type;
	/* The identified cable, to recognize it after a source event. */
	bool cable_known;
	bool usbc;
	unsigned int orientation;
	char source[96];
	u64 attach_ns;		/* next identification at attach_ns + LOOK_MS */
	u64 hold_ns;		/* identification round started */
	u64 step_ns;		/* last ramp step, 0: the ramp has not started */
	u64 step_wait_ns;	/* before the next step up */
	u64 guard_ns;
	unsigned int epoch_seen;
	/* Lightning accessory policy, on boards with a role switch */
	struct tristar_accessory *acc;
	/* USB data is on, but the device controller is held off for a USB host role. */
	bool usb_held;
};

static unsigned int tristar_d2365_code_ma(struct tristar_charger *chg, unsigned int code)
{
	return code * 25 / 2 + 75;
}

/* Round down: quantization must never exceed a permitted current. */
static unsigned int tristar_d2365_ma_code(struct tristar_charger *chg, unsigned int ma)
{
	return ma <= 75 ? 0 : (ma - 75) * 2 / 25;
}

static int tristar_d2365_read_limit(struct tristar_charger *chg,
				    unsigned int *code)
{
	return regmap_read(chg->regmap,
			   chg->control_base + chg->pmic->input_limit, code);
}

static int tristar_d2365_write_limit(struct tristar_charger *chg,
				     unsigned int code)
{
	return regmap_write(chg->regmap,
			    chg->control_base + chg->pmic->input_limit, code);
}

static int tristar_d2365_read_status(struct tristar_charger *chg,
				     unsigned int *status)
{
	return regmap_read(chg->regmap,
			   chg->status_base + chg->pmic->status, status);
}

static const struct tristar_chip_info tristar_d2365_chip = {
	.adc_error_mask = 0x80,
	.read_limit = tristar_d2365_read_limit,
	.write_limit = tristar_d2365_write_limit,
	.code_ma = tristar_d2365_code_ma,
	.ma_code = tristar_d2365_ma_code,
	.read_status = tristar_d2365_read_status,
	.handoff_code = TRISTAR_CHG_HANDOFF_CODE,
	.attach_code = TRISTAR_CHG_HANDOFF_CODE,
	.fallback_code = TRISTAR_CHG_FALLBACK_CODE,
};

static unsigned int tristar_code_ma(struct tristar_charger *chg, unsigned int code)
{
	return chg->chip->code_ma(chg, code);
}

static unsigned int tristar_ma_code(struct tristar_charger *chg, unsigned int ma)
{
	return chg->chip->ma_code(chg, ma);
}

static int tristar_set_code(struct tristar_charger *chg, unsigned int code)
{
	unsigned int readback;
	int ret;

	ret = chg->chip->write_limit(chg, code);
	if (!ret)
		ret = chg->chip->read_limit(chg, &readback);
	if (!ret && readback != code)
		ret = -EIO;
	if (!ret)
		WRITE_ONCE(chg->code, code);
	return ret;
}

static void tristar_withdraw(struct tristar_charger *chg, unsigned int code)
{
	if (READ_ONCE(chg->code) > code && tristar_set_code(chg, code))
		dev_crit(chg->dev, "Withdrawal write failed; input limit unverified\n");
}

/* ---- Battery ---- */

struct tristar_pack {
	int capacity;
	int temp;
	int voltage_uv;
	int current_ua;
	bool full;
};

static int tristar_read_pack(struct tristar_charger *chg, struct tristar_pack *pack)
{
	struct power_supply *battery = power_supply_get_by_name("battery");
	union power_supply_propval val;
	int ret;

	if (!battery)
		return -ENODEV;
	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret && val.intval != 1)
		ret = -ENODEV;
	if (!ret)
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret && (val.intval < 0 || val.intval > 100))
		ret = -ERANGE;
	if (!ret) {
		pack->capacity = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_TEMP, &val);
	}
	if (!ret) {
		pack->temp = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	}
	if (!ret) {
		pack->voltage_uv = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	}
	if (!ret)
		pack->current_ua = val.intval;
	pack->full = false;
	if (!ret && chg->chip->require_pack_telemetry) {
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &val);
		if (!ret)
			pack->full = val.intval == POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	}
	power_supply_put(battery);
	return ret;
}

/* Grants above the fallback are allowed only inside these pack limits. */
static bool tristar_pack_allows_grant(struct tristar_charger *chg, const struct tristar_pack *pack)
{
	if (chg->chip->pack_allows)
		return chg->chip->pack_allows(chg, pack);
	return pack->temp >= 50 && pack->temp <= 400 &&
	       pack->voltage_uv >= 3200000 && pack->voltage_uv <= 4350000 &&
	       pack->current_ua >= -3000000 && pack->current_ua <= 3000000;
}

/* ---- Tristar ---- */

static int cbtl_read(struct tristar_charger *chg, u8 reg, u8 *buf, u16 len)
{
	struct i2c_client *client = chg->tristar;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = 1, .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);

	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static int cbtl_write(struct tristar_charger *chg, u8 reg, u8 value)
{
	struct i2c_client *client = chg->tristar;
	u8 bytes[2] = { reg, value };
	struct i2c_msg msg = { .addr = client->addr, .len = 2, .buf = bytes };
	u8 readback;
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;
	ret = cbtl_read(chg, reg, &readback, 1);
	return ret ?: readback == value ? 0 : -EIO;
}

static int cbtl_route(struct tristar_charger *chg, u8 value)
{
	return cbtl_write(chg, CBTL_MUX_ROUTE, value);
}

/* Forward CRC-8: seed 0xff, reflected polynomial 0x8c. */
static u8 cbtl_crc(const u8 *data, unsigned int len)
{
	unsigned int crc = 0xff, i, bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}
	return crc;
}

/*
 * Decode the routes of the accessory's capability response, as the Tristar
 * firmware's legacy branch and its helper do. Extended layouts and
 * unsuccessful multi-function routes are rejected. Returns false when the
 * layout is not understood.
 */
static bool cbtl_decode(u8 h0, u8 h2, u8 h3, unsigned int orientation,
			enum cbtl_route *first, enum cbtl_route *second)
{
	bool usb = h0 & 0x18, uart = h0 & 0x03, mbus = h0 & 0x04;
	bool usbc = h2 & 0x08, uart2 = false;
	enum cbtl_route tags[2] = { CBTL_ROUTE_NONE, CBTL_ROUTE_NONE };
	unsigned int n = 0;

	if ((h0 & 0x20) || (orientation != 1 && orientation != 2))
		return false;
	/* The helper can replace the legacy flags; byte 2 bit 3 alone is not enough. */
	if (mbus) {
		if (usb && (h0 & 0x03) == 0x01 && (h3 & 0x70) == 0x20) {
			uart = mbus = usbc = false;
		} else if (usbc) {
			if (usb && (h0 & 0x03) == 0x02) {
				uart = mbus = usbc = false;
				uart2 = true;
			} else if ((h0 & 0x1b) == 0x1b) {
				usb = uart = usbc = false;
				uart2 = true;
			} else if ((h0 & 0x1b) == 0x0b) {
				usb = mbus = usbc = false;
				uart2 = true;
			}
		}
	}
	if (usb + uart + mbus + usbc + uart2 >= 3)
		return false;
	if (usb)
		tags[n++] = CBTL_ROUTE_USB0;
	if (uart)
		tags[n++] = CBTL_ROUTE_UART0;
	if (mbus)
		tags[n++] = CBTL_ROUTE_MBUS;
	if (usbc)
		tags[n++] = CBTL_ROUTE_USBC;
	if (uart2)
		tags[1] = CBTL_ROUTE_UART2;
	*first = tags[orientation == 2 ? 1 : 0];
	*second = tags[orientation == 2 ? 0 : 1];
	return true;
}

/*
 * Read the capability frame once. The status byte must be stable around the
 * response, and the response must carry the expected opcode and CRC.
 */
static int cbtl_capability(struct tristar_charger *chg, bool *usbc,
			   unsigned int *orientation)
{
	enum cbtl_route first, second;
	u8 packet[CBTL_CAP_LEN], before, after;
	int ret;

	ret = cbtl_read(chg, CBTL_CAP_STATUS, &before, 1);
	if (ret)
		return ret;
	if ((before & 0x84) != 0x80)
		return -EAGAIN;
	ret = cbtl_read(chg, CBTL_CAP_RESPONSE, packet, sizeof(packet));
	if (!ret)
		ret = cbtl_read(chg, CBTL_CAP_STATUS, &after, 1);
	if (ret)
		goto out;
	if (before != after || packet[0] != 0x75) {
		ret = -EAGAIN;
		goto out;
	}
	if (cbtl_crc(packet, 7) != packet[7]) {
		ret = -EBADMSG;
		goto out;
	}
	*orientation = 1 + !!(after & 0x40);
	if (!cbtl_decode(packet[1] & 0x3f, packet[3] & 0x08, packet[4] & 0x70,
			 *orientation, &first, &second)) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	*usbc = first == CBTL_ROUTE_USBC || second == CBTL_ROUTE_USBC;
out:
	/* The response can carry accessory identity; do not keep it. */
	memzero_explicit(packet, sizeof(packet));
	return ret;
}

/* ---- ADC ---- */

/* One conversion; completion arrives as a PMU event. Caller holds lock. */
static int tristar_adc_mv(struct tristar_charger *chg, unsigned int epoch,
			  unsigned int *mv)
{
	u8 value[2];
	u32 raw;
	int ret;

	reinit_completion(&chg->adc_done);
	WRITE_ONCE(chg->adc_waiting, true);
	ret = regmap_write(chg->adc_regmap, chg->adc_base, chg->pmic->adc_start);
	if (ret) {
		WRITE_ONCE(chg->adc_waiting, false);
		return ret;
	}
	if (!wait_for_completion_timeout(&chg->adc_done, msecs_to_jiffies(500))) {
		WRITE_ONCE(chg->adc_waiting, false);
		return -ETIMEDOUT;
	}
	if (atomic_read(&chg->epoch) != epoch)
		return -ESTALE;
	ret = regmap_bulk_read(chg->adc_regmap, chg->adc_base + chg->pmic->adc_result,
			       value, sizeof(value));
	if (!ret && chg->pmic->adc_rearm) {
		ret = regmap_update_bits(chg->adc_regmap, chg->adc_base,
					 TRISTAR_CHG_ADC_START, 0);
		if (!ret)
			ret = regmap_update_bits(chg->adc_regmap, chg->adc_base + 1,
						 TRISTAR_CHG_ADC_LATCH, 0);
	}
	if (ret)
		return ret;
	if (value[0] & chg->chip->adc_error_mask)
		return -EIO;
	raw = (value[0] & 0x0f) | ((u32)value[1] << 4);
	*mv = (((u64)raw * 5000 * 16 + 40 * 65536) / 1000) * 1000 / 65536;
	return 0;
}

/*
 * Route the Tristar mux, optionally excite the other line of the pair for
 * BC1.2 detection, convert, and always restore the mux and the excitation.
 */
static int tristar_mux_sample(struct tristar_charger *chg, const u8 *routes,
			      unsigned int count, u8 excite, unsigned int epoch,
			      unsigned int *mv)
{
	unsigned int i;
	int ret = 0, restore = 0;
	u8 state;

	for (i = 0; i < count && !ret; i++)
		ret = cbtl_route(chg, routes[i]);
	if (!ret) {
		usleep_range(5000, 6000);
		if (excite) {
			ret = cbtl_write(chg, CBTL_MUX_STATE, excite);
			if (!ret)
				msleep(40);
		}
		if (!ret)
			ret = tristar_adc_mv(chg, epoch, mv);
	}
	if (excite)
		restore = cbtl_write(chg, CBTL_MUX_STATE, 0);
	if (!restore)
		restore = cbtl_route(chg, 0);
	if (!restore)
		restore = cbtl_read(chg, CBTL_MUX_STATE, &state, 1);
	if (!restore && state)
		restore = -EIO;
	if (restore) {
		dev_crit(chg->dev, "Tristar mux restore unverified (%d)\n", restore);
		return -EUCLEAN;
	}
	return ret;
}

/* Apple USB-A chargers: D+ and D- divider bands select the BrickID. */
static unsigned int tristar_brick_ma(unsigned int dp, unsigned int dm,
				     unsigned int *id)
{
	static const unsigned int brick_ma[9] = {
		500, 1000, 1000, 2100, 2400, 3000, 1000, 1000, 1000,
	};
	unsigned int p, m;

	if (dp < 1500 || dm < 1500)
		return 0;
	p = dp < 2184 ? 0 : dp < 2880 ? 1 : 2;
	m = dm < 2184 ? 0 : dm < 2880 ? 1 : 2;
	*id = 1 + 3 * p + m;
	return brick_ma[*id - 1];
}

/* ---- Identification ---- */

struct tristar_ident {
	unsigned int ma;		/* the source's advertisement, 0: none */
	enum power_supply_usb_type type;
	bool sampled;			/* the mux was used */
	bool usbc;			/* capability frame, valid if sampled */
	unsigned int orientation;
	char what[96];
};

/* Mux routes per bank: D- and D+ through the USB-selected bank. */
static const u8 tristar_dminus[2][2] = { { 0x11, 0x1d }, { 0x10, 0xd0 } };
static const u8 tristar_dplus[2][2] = { { 0x11, 0x1c }, { 0x10, 0xc0 } };

/*
 * BC1.2 primary and secondary detection on the oriented USB pair, as iOS
 * does for an ordinary USB source (5de33e4, 5de35dc): with the measured line
 * routed to the ADC, the Tristar excites the pair for 40 ms (register 0x03:
 * 9 while D- is measured, 10 while D+ is, plus 2 on the high bank; 5ff5a08).
 * A primary D- of 326-899 mV means a charging port; the secondary D+ then
 * tells a CDP (below 326 mV) from a DCP. Both may draw 1.5 A.
 */
static int tristar_identify_bc12(struct tristar_charger *chg, unsigned int bank,
				 unsigned int epoch, struct tristar_ident *id)
{
	unsigned int dm, dp;
	int ret;

	ret = tristar_mux_sample(chg, tristar_dminus[bank], 2, 9 + 2 * bank,
				 epoch, &dm);
	if (ret)
		return ret;
	if (dm < 326 || dm > 899) {
		dev_dbg(chg->dev, "BC1.2 primary detection: D- %u mV, no charging port\n", dm);
		return 0;
	}
	ret = tristar_mux_sample(chg, tristar_dplus[bank], 2, 10 + 2 * bank,
				 epoch, &dp);
	if (ret)
		return ret;
	id->ma = 1500;
	id->type = dp < 326 ? POWER_SUPPLY_USB_TYPE_CDP : POWER_SUPPLY_USB_TYPE_DCP;
	snprintf(id->what, sizeof(id->what),
		 "USB charging port (%s, BC1.2 D- %u mV, D+ %u mV)",
		 id->type == POWER_SUPPLY_USB_TYPE_CDP ? "CDP" : "DCP", dm, dp);
	return 0;
}

static int tristar_identify_usbc(struct tristar_charger *chg, unsigned int orientation,
				 unsigned int epoch, struct tristar_ident *id)
{
	/* Lane 0 is sampled through route 0xc0, lane 1 through 0x0c. */
	static const u8 lanes[2] = { 0xc0, 0x0c };
	unsigned int mv[2], sel = orientation == 2 ? 1 : 0, cc_ma;
	int ret, lane;

	/*
	 * The firmware reads only the orientation-selected lane. The other is
	 * logged as evidence; it is not a CC advertisement.
	 */
	for (lane = 0; lane < 2; lane++) {
		ret = tristar_mux_sample(chg, &lanes[lane], 1, 0, epoch, &mv[lane]);
		if (ret)
			return ret;
	}
	cc_ma = mv[sel] >= 1230 ? 3000 : mv[sel] >= 660 ? 1500 : 0;
	if (cc_ma < 1500) {
		/* Default USB power: a charging port may still allow 1.5 A. */
		ret = tristar_identify_bc12(chg, orientation - 1, epoch, id);
		if (ret)
			return ret;
		if (id->ma > cc_ma)
			return 0;
	}
	id->ma = cc_ma;
	id->type = POWER_SUPPLY_USB_TYPE_C;
	if (cc_ma)
		snprintf(id->what, sizeof(id->what),
			 "USB-C power source advertising %u.%u A (CC %u mV, other lane %u mV)",
			 cc_ma / 1000, cc_ma % 1000 / 100, mv[sel], mv[!sel]);
	else
		snprintf(id->what, sizeof(id->what),
			 "USB-C source with default USB power (CC %u mV, other lane %u mV)",
			 mv[sel], mv[!sel]);
	return 0;
}

static int tristar_identify_usba(struct tristar_charger *chg, unsigned int orientation,
				 unsigned int epoch, struct tristar_ident *id)
{
	unsigned int bank_ma[2], bank_id[2] = { 0, 0 }, dp, dm, brick;
	int ret, bank;

	for (bank = 0; bank < 2; bank++) {
		ret = tristar_mux_sample(chg, tristar_dminus[bank], 2, 0, epoch, &dm);
		if (!ret)
			ret = tristar_mux_sample(chg, tristar_dplus[bank], 2, 0, epoch, &dp);
		if (ret)
			return ret;
		bank_ma[bank] = tristar_brick_ma(dp, dm, &bank_id[bank]);
	}
	if (bank_ma[0] && bank_ma[1] && bank_id[0] != bank_id[1]) {
		/* Disagreeing banks: trust the lower advertisement. */
		id->ma = min(bank_ma[0], bank_ma[1]);
		brick = bank_ma[0] < bank_ma[1] ? bank_id[0] : bank_id[1];
	} else {
		bank = bank_ma[0] ? 0 : 1;
		id->ma = bank_ma[bank];
		brick = bank_id[bank];
	}
	if (id->ma) {
		id->type = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
		snprintf(id->what, sizeof(id->what), "Apple USB-A charger, BrickID %u (%u.%u A)",
			 brick, id->ma / 1000, id->ma % 1000 / 100);
		return 0;
	}
	ret = tristar_identify_bc12(chg, orientation - 1, epoch, id);
	if (!ret && !id->ma)
		snprintf(id->what, sizeof(id->what),
			 "USB-A source without charger signature (computer, hub or generic charger)");
	return ret;
}

/* Caller holds lock. */
static int tristar_identify(struct tristar_charger *chg, struct tristar_ident *id)
{
	unsigned int epoch = atomic_read(&chg->epoch), orientation;
	const char *holder;
	bool usbc;
	u8 route, state;
	int ret;

	id->ma = 0;
	id->type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	id->sampled = false;
	holder = tristar_accessory_holds(chg->acc, &id->ma);
	if (holder) {
		/*
		 * The accessory policy holds the port: the mux is not ours to
		 * sample, and no cable is known to compare after a source event.
		 */
		id->type = POWER_SUPPLY_USB_TYPE_ACA;
		id->usbc = false;
		id->orientation = 0;
		strscpy(id->what, holder, sizeof(id->what));
		return 0;
	}
	ret = cbtl_read(chg, CBTL_MUX_ROUTE, &route, 1);
	if (!ret)
		ret = cbtl_read(chg, CBTL_MUX_STATE, &state, 1);
	if (ret)
		return ret;
	if (route || state) {
		snprintf(id->what, sizeof(id->what), "unknown source (mux busy %02x/%02x)",
			 route, state);
		return 0;
	}
	ret = cbtl_capability(chg, &usbc, &orientation);
	if (ret) {
		snprintf(id->what, sizeof(id->what), "unidentified cable (capability frame %d)",
			 ret);
		return 0;
	}
	id->sampled = true;
	id->usbc = usbc;
	id->orientation = orientation;
	if (usbc)
		return tristar_identify_usbc(chg, orientation, epoch, id);
	return tristar_identify_usba(chg, orientation, epoch, id);
}

/* ---- Policy ---- */

/*
 * What the USB host allows. As in iOS 16 (IOUSBDeviceController), only the
 * USB session changes it: a reset, a disconnect or a suspend reported by the
 * device controller, or the end of the attachment. A PMU source event does
 * not. Caller holds lock.
 */
static unsigned int tristar_usb_ma(struct tristar_charger *chg)
{
	return chg->usb_ma;
}

/*
 * Turn USB data on or hold it off. A failed hold counts as on: the mux is
 * then left alone. While the accessory policy holds the port as a USB host,
 * the device controller stays off even with USB data on; the next poll after
 * the policy lets go turns it on. Stopping always turns it on. Caller holds
 * lock.
 */
static void tristar_set_usb(struct tristar_charger *chg, bool on)
{
	unsigned int ma;
	bool held = on && !chg->stopping && tristar_accessory_holds(chg->acc, &ma);
	int ret = usb_role_switch_set_role(chg->role_sw,
					   on && !held ? USB_ROLE_DEVICE : USB_ROLE_NONE);

	chg->usb_held = held && !ret;
	if (ret)
		dev_err(chg->dev, "cannot turn USB data %s (%d)\n", on ? "on" : "off", ret);
	chg->usb_on = on || ret;
	if (!chg->usb_on) {
		chg->usb_ma = 0;
		chg->usb_report_held = false;
	}
}

/* The highest grant the source allows, up to the board maximum. */
static unsigned int tristar_target_code(struct tristar_charger *chg)
{
	unsigned int ma = max(chg->source_ma, tristar_usb_ma(chg));

	if (ma <= tristar_code_ma(chg, chg->chip->fallback_code))
		return chg->chip->fallback_code;
	return tristar_ma_code(chg, min(ma, chg->max_ma));
}

/* Follow a changed allowance once the source is identified. Caller holds lock. */
static void tristar_update_target(struct tristar_charger *chg)
{
	if (chg->phase != TRISTAR_RAMPING && chg->phase != TRISTAR_CHARGING)
		return;
	chg->target_code = tristar_target_code(chg);
	if (chg->code > chg->target_code) {
		tristar_withdraw(chg, chg->target_code);
		chg->phase = TRISTAR_CHARGING;
	} else if (chg->code < chg->target_code && chg->phase == TRISTAR_CHARGING) {
		chg->phase = TRISTAR_RAMPING;
		chg->step_ns = 0;
	}
}

/* Turn on a device controller held off for a host role the policy gave back. Caller holds lock. */
static void tristar_unhold_usb(struct tristar_charger *chg)
{
	unsigned int ma;

	if (chg->usb_held && !tristar_accessory_holds(chg->acc, &ma))
		tristar_set_usb(chg, true);
}

/* Caller holds lock. */
static void tristar_new_attachment(struct tristar_charger *chg, u64 now)
{
	/* Before anything else: a host must not start a session yet. */
	tristar_set_usb(chg, false);
	chg->present = true;
	chg->vbus_fault = false;
	chg->phase = TRISTAR_SETTLING;
	chg->attach_ns = now;
	chg->hold_ns = now;
	chg->source[0] = '\0';
	chg->source_ma = 0;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	chg->cable_known = false;
	chg->epoch_seen = atomic_read(&chg->epoch);
	WRITE_ONCE(chg->line_ns, 0);
	if (tristar_set_code(chg, chg->chip->attach_code))
		dev_crit(chg->dev, "Attach write failed; input limit unverified\n");
	dev_dbg(chg->dev, "power connected, identifying source\n");
}

/* Caller holds lock. */
static void tristar_mux_fault(struct tristar_charger *chg)
{
	chg->phase = TRISTAR_FAULT;
	strscpy(chg->source, "Tristar mux restore failed", sizeof(chg->source));
	tristar_withdraw(chg, chg->chip->fallback_code);
	dev_err(chg->dev, "Tristar mux restore unverified; holding the fallback until reboot\n");
}

/* Start the grant for the identified source: ramp up to it. Caller holds lock. */
static void tristar_grant(struct tristar_charger *chg)
{
	chg->target_code = tristar_target_code(chg);
	if (chg->code > chg->target_code)
		tristar_withdraw(chg, chg->target_code);
	chg->phase = chg->code < chg->target_code ? TRISTAR_RAMPING : TRISTAR_CHARGING;
	chg->step_ns = 0;
	dev_dbg(chg->dev, "%s: up to %u mA\n", chg->source,
		tristar_code_ma(chg, chg->target_code));
}

/*
 * Take an identification and turn USB data on. Even a charger signature does
 * not rule out a host: some ports show one until a device draws current. A
 * BC1.2 dedicated charging port does: it shorts D+ to D-, so no host is behind
 * it, and the controller's pull-up on that pair reads as a bus reset and a
 * source event, which would start the identification over every few seconds.
 * USB data stays off on a DCP; a port that changes its mode later is a new
 * source event and is identified again. Caller holds lock.
 */
static void tristar_accept(struct tristar_charger *chg, const struct tristar_ident *id)
{
	strscpy(chg->source, id->what, sizeof(chg->source));
	chg->source_ma = id->ma;
	chg->usb_type = id->type;
	chg->cable_known = id->sampled;
	chg->usbc = id->usbc;
	chg->orientation = id->orientation;
	tristar_set_usb(chg, chg->chip != &tristar_d2365_chip ||
			id->type != POWER_SUPPLY_USB_TYPE_DCP);
	tristar_grant(chg);
}

/*
 * A source event while USB data is on. The mux must stay untouched, but the
 * capability frame can be read: the same cable in the same orientation means
 * the same source, and its identification still applies. A source swapped
 * between two cables of one kind faster than the poll is not told apart, and
 * neither is a USB-C source lowering its Rp without a detach (usb-c.md).
 * Anything else is a new attachment. Caller holds lock.
 */
static void tristar_reconfirm(struct tristar_charger *chg, u64 now, bool expired)
{
	unsigned int orientation;
	bool usbc;
	int ret;

	if (!chg->cable_known) {
		/* Nothing to compare with: keep the session at the fallback. */
		tristar_grant(chg);
		return;
	}
	ret = cbtl_capability(chg, &usbc, &orientation);
	if (!ret && atomic_read(&chg->epoch) == chg->epoch_seen &&
	    usbc == chg->usbc && orientation == chg->orientation) {
		dev_dbg(chg->dev, "same cable after the source event\n");
		tristar_grant(chg);
		return;
	}
	chg->epoch_seen = atomic_read(&chg->epoch);
	if (!expired)
		return;		/* look again on the next poll */
	dev_dbg(chg->dev, "cable changed during the USB session (%d)\n", ret);
	tristar_new_attachment(chg, now);
}

static void tristar_run_identification(struct tristar_charger *chg, u64 now)
{
	bool expired = (s64)(now - chg->hold_ns) >= (s64)TRISTAR_HOLD_MS * NSEC_PER_MSEC;
	struct tristar_ident id;
	int ret;

	if (chg->usb_on) {
		tristar_reconfirm(chg, now, expired);
		return;
	}
	/*
	 * iOS measures the data lines only on the USB route that the digital
	 * ID selects. While the accessory policy reads that ID, leave the mux
	 * alone, but never beyond the hold.
	 */
	if (!expired && tristar_accessory_identifying(chg->acc))
		return;
	ret = tristar_identify(chg, &id);
	if (ret == -EUCLEAN) {
		tristar_mux_fault(chg);
		return;
	}
	if (ret || atomic_read(&chg->epoch) != chg->epoch_seen) {
		/* The source changed while identifying: look again once it settles. */
		chg->epoch_seen = atomic_read(&chg->epoch);
		if (!expired) {
			chg->attach_ns = now;
			return;
		}
		dev_dbg(chg->dev, "source unstable during identification (%d)\n", ret);
		id.ma = 0;
		id.type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		id.sampled = false;
		strscpy(id.what, "unstable source", sizeof(id.what));
	} else if (!id.ma && !expired) {
		return;		/* no signature yet: look again on the next poll */
	}
	tristar_accept(chg, &id);
}

/* A source event less than TRISTAR_EVENT_DEBOUNCE_MS ago holds the ramp. */
static bool tristar_debouncing(struct tristar_charger *chg, u64 now)
{
	u64 line_ns = READ_ONCE(chg->line_ns);

	return line_ns &&
	       (s64)(now - line_ns) < (s64)TRISTAR_EVENT_DEBOUNCE_MS * NSEC_PER_MSEC;
}

static void tristar_ramp_step(struct tristar_charger *chg, u64 now)
{
	struct tristar_pack pack;
	unsigned int next, from_ma = tristar_code_ma(chg, chg->code);
	u64 wait;

	if (tristar_debouncing(chg, now) ||
	    (chg->step_ns && now - chg->step_ns < chg->step_wait_ns))
		return;
	if (tristar_read_pack(chg, &pack)) {
		if (chg->chip->require_pack_telemetry)
			tristar_withdraw(chg, chg->chip->fallback_code);
		return;
	}
	if (!tristar_pack_allows_grant(chg, &pack)) {
		chg->phase = TRISTAR_CHARGING;
		tristar_withdraw(chg, chg->chip->fallback_code);
		dev_warn(chg->dev, "battery outside the charging window; holding the fallback\n");
		return;
	}
	if (chg->chip->ramp_step_ms) {
		next = min(chg->target_code, tristar_ma_code(chg, from_ma + TRISTAR_RAMP_STEP_MA));
		wait = chg->chip->ramp_step_ms * NSEC_PER_MSEC;
	} else if (!chg->step_ns && chg->code <= chg->chip->handoff_code) {
		/* From nothing, iOS writes 100 mA and holds it. */
		next = chg->chip->handoff_code;
		wait = TRISTAR_RAMP_HOLD_MS * NSEC_PER_MSEC;
	} else {
		next = min(chg->target_code, tristar_ma_code(chg, from_ma + TRISTAR_RAMP_STEP_MA));
		wait = (u64)(tristar_code_ma(chg, next) - from_ma) * (NSEC_PER_MSEC / 10);
	}
	if (tristar_set_code(chg, next)) {
		chg->phase = TRISTAR_FAULT;
		tristar_withdraw(chg, chg->chip->fallback_code);
		dev_err(chg->dev, "input-limit write failed; holding the fallback\n");
		return;
	}
	chg->step_ns = now;
	chg->step_wait_ns = wait;
	if (next == chg->target_code)
		chg->phase = TRISTAR_CHARGING;
}

/*
 * While ramping, the next step or the end of a debounce is due sooner than the
 * next poll. Caller holds lock. Returns 0 when the usual poll interval applies,
 * which includes a step that could not be taken (an unreadable battery).
 */
static unsigned long tristar_ramp_delay(struct tristar_charger *chg, u64 now)
{
	u64 due = chg->step_ns ? chg->step_ns + chg->step_wait_ns : now;
	u64 line_ns = READ_ONCE(chg->line_ns);

	if (chg->stopping || chg->phase != TRISTAR_RAMPING)
		return 0;
	if (line_ns)
		due = max(due, line_ns + TRISTAR_EVENT_DEBOUNCE_MS * NSEC_PER_MSEC);
	if ((s64)(due - now) <= 0)
		return 0;
	return max(usecs_to_jiffies(div_u64(due - now, NSEC_PER_USEC)), 1UL);
}

/*
 * On the iPad 5 the PMIC returns the input limit to its attach default of
 * 100 mA some 25-400 ms after attach, after the fallback has been written.
 * Put the fallback back; a grant above it was set by a later ramp, so drop
 * to the fallback and ramp again under the usual pack checks. A limit above
 * the one written is put back down at once. Caller holds lock.
 */
static void tristar_check_limit(struct tristar_charger *chg)
{
	unsigned int hw, code = chg->code;

	int ret = chg->chip->read_limit(chg, &hw);

	if (ret) {
		if (chg->chip->strict_limit_errors) {
			tristar_set_code(chg, chg->chip->fallback_code);
			chg->phase = TRISTAR_FAULT;
			dev_err_ratelimited(chg->dev, "input limit unverified (%d); holding fallback\n", ret);
		}
		return;
	}
	if (hw == code)
		return;
	dev_warn_ratelimited(chg->dev, "input limit changed to %u mA by the PMIC; restoring\n",
			     tristar_code_ma(chg, hw));
	if (hw > code) {
		if (tristar_set_code(chg, code))
			dev_crit(chg->dev, "Input-limit write failed; limit unverified\n");
		return;
	}
	if (tristar_set_code(chg, min_t(unsigned int, code, chg->chip->fallback_code))) {
		dev_crit(chg->dev, "Fallback write failed; input limit unverified\n");
		return;
	}
	if (code > chg->chip->fallback_code) {
		chg->phase = TRISTAR_RAMPING;
		chg->step_ns = 0;
	}
}

static void tristar_guard(struct tristar_charger *chg, u64 now)
{
	struct tristar_pack pack;

	if (now - chg->guard_ns < NSEC_PER_SEC)
		return;
	chg->guard_ns = now;
	tristar_check_limit(chg);
	if (chg->phase != TRISTAR_CHARGING)
		return;
	if (tristar_read_pack(chg, &pack)) {
		if (chg->chip->require_pack_telemetry)
			tristar_withdraw(chg, chg->chip->fallback_code);
		return;
	}
	if (chg->code > chg->chip->fallback_code && !tristar_pack_allows_grant(chg, &pack)) {
		tristar_withdraw(chg, chg->chip->fallback_code);
		dev_warn(chg->dev, "battery outside the charging window; holding the fallback\n");
	} else if (chg->code < chg->target_code && tristar_pack_allows_grant(chg, &pack)) {
		/* Back inside the window: no replug needed to charge again. */
		chg->phase = TRISTAR_RAMPING;
		chg->step_ns = 0;
	}
}

/*
 * The hold at probe waits for an attachment, and a bootloader can leave the PMIC
 * reporting none with a cable in. Nothing else would then turn USB data on, and
 * the role set at probe has already ended the controller's own fallback, so give
 * up on the source: USB serial and SSH must not depend on charging. An attachment
 * later holds USB data off again and identifies as usual. Caller holds lock.
 */
static void tristar_no_source(struct tristar_charger *chg, unsigned int status, u64 now)
{
	if (chg->usb_on ||
	    (s64)(now - chg->hold_ns) < (s64)TRISTAR_HOLD_MS * NSEC_PER_MSEC)
		return;
	dev_info(chg->dev, "no source identified (status %#x), turning USB data on\n", status);
	tristar_set_usb(chg, true);
}

/*
 * While the accessory policy holds the port (the camera adapter in USB host
 * mode), the source behind the adapter cannot be identified: take the
 * policy's allowance. Once the policy lets go, identify the source again, or
 * turn on the device controller that tristar_set_usb() held off. Caller
 * holds lock.
 */
static void tristar_follow_accessory(struct tristar_charger *chg, u64 now)
{
	bool charging = chg->phase == TRISTAR_RAMPING || chg->phase == TRISTAR_CHARGING;
	const char *holder;
	unsigned int ma;

	if (!IS_ENABLED(CONFIG_APPLE_TRISTAR_ACCESSORY))
		return;
	holder = tristar_accessory_holds(chg->acc, &ma);
	if (!holder) {
		if (charging && chg->usb_type == POWER_SUPPLY_USB_TYPE_ACA)
			tristar_new_attachment(chg, now);
		else
			tristar_unhold_usb(chg);
		return;
	}
	if (!charging || (chg->usb_type == POWER_SUPPLY_USB_TYPE_ACA && chg->source_ma == ma))
		return;
	strscpy(chg->source, holder, sizeof(chg->source));
	chg->source_ma = ma;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_ACA;
	tristar_update_target(chg);
	dev_dbg(chg->dev, "%s: %u mA\n", holder, ma);
}

/*
 * Input presence is the D2365's connection state 0x189, as iOS 16 decides it
 * (AppleD2365PMUPowerSource): bit 0 is "connected", and a new connection also
 * needs bit 1. The D2207's 0x60 has the same meaning, with bit 3 for a new
 * connection (iOS 15.8 AppleD2207PMU). The charger status 0x18a is not presence. Its bits describe
 * the battery charger: the same method takes bits 1:0 as charging, bit 2 as
 * charge end and bit 4 as charge timeout, for the charging and fully charged
 * state only. It can also read 0 with the input still there: behind the
 * powered camera adapter it alternated between 0x40 and 0 every 100-300 ms
 * during the ramp while 0x189 bit 0 stayed set. iOS's input-current ramp
 * (AppleARMPMUCharger) follows the source's allowance and backs off only when
 * the PMIC reports input limiting, which the D2365 never does, so a charger
 * that finished, timed out or does not charge keeps the grant.
 *
 * The D2207's charger state 0x62 has the same bits. An unreadable connection
 * state says nothing: keep the last decision.
 */
static bool tristar_input_present(struct tristar_charger *chg)
{
	unsigned int connected;

	if (regmap_read(chg->regmap, chg->status_base + chg->pmic->connected, &connected)) {
		dev_err_ratelimited(chg->dev, "PMIC connection state unreadable\n");
		return chg->online;
	}
	if (!(connected & TRISTAR_CHG_CONNECTED_NOW))
		return false;
	return chg->present || (connected & chg->pmic->connected_new);
}

static void tristar_poll(struct work_struct *work)
{
	unsigned long ramp_delay = 0;
	struct tristar_charger *chg = container_of(to_delayed_work(work),
						   struct tristar_charger, poll_work);
	unsigned int status, old_code;
	bool old_online, old_present, old_usb_on;
	u64 now = ktime_get_ns(), line_ns;

	mutex_lock(&chg->lock);
	if (chg->stopping)
		goto out;
	old_code = chg->code;
	old_online = chg->online;
	old_present = chg->present;
	old_usb_on = chg->usb_on;
	if (chg->chip->read_status(chg, &status)) {
		tristar_withdraw(chg, chg->chip->fallback_code);
		dev_err_ratelimited(chg->dev, "PMIC status unreadable; holding the fallback\n");
		goto again;
	}
	chg->online = chg->chip->online_status ? status != 0 : tristar_input_present(chg);
	if (!chg->online) {
		/* The hold of the next attachment starts when input returns. */
		chg->hold_ns = now;
		if (chg->present) {
			tristar_set_usb(chg, false);
			chg->present = false;
			chg->phase = TRISTAR_EMPTY;
			chg->source[0] = '\0';
			chg->source_ma = 0;
			chg->usb_ma = 0;
			chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			if (tristar_set_code(chg, chg->chip->handoff_code))
				dev_crit(chg->dev, "Handoff write failed; input limit unverified\n");
			dev_dbg(chg->dev, "disconnected\n");
		}
		tristar_no_source(chg, status, now);
		goto again;
	}
	if (chg->chip->online_status && status != chg->chip->online_status) {
		/* Transitional input state: no grant. */
		tristar_withdraw(chg, chg->chip->fallback_code);
		tristar_no_source(chg, status, now);
		goto again;
	}
	if (!chg->present) {
		tristar_new_attachment(chg, now);
		goto again;
	}
	if (IS_REACHABLE(CONFIG_IIO) && chg->vbus && !chg->vbus_fault &&
	    (chg->phase == TRISTAR_RAMPING || chg->phase == TRISTAR_CHARGING) &&
	    tristar_code_ma(chg, chg->target_code) > 1000) {
		int mv, ret;

		ret = iio_read_channel_processed(chg->vbus, &mv);
		if (ret || mv < 4750 || mv > 5500) {
			chg->vbus_fault = true;
			chg->phase = TRISTAR_FAULT;
			dev_warn(chg->dev, "VBUS guard tripped (sample %d mV, error %d)\n",
				 ret ? 0 : mv, ret);
		}
	}
	if (chg->vbus_fault)
		tristar_withdraw(chg, chg->chip->fallback_code);
	if (chg->phase == TRISTAR_FAULT) {
		/* USB data must not depend on charging working. */
		if (!chg->usb_on)
			tristar_set_usb(chg, true);
		else
			tristar_unhold_usb(chg);
		goto again;
	}
	if (atomic_read(&chg->epoch) != chg->epoch_seen && chg->phase != TRISTAR_SETTLING) {
		/* An event the identification did not see: identify again. */
		chg->epoch_seen = atomic_read(&chg->epoch);
		chg->phase = TRISTAR_SETTLING;
		chg->attach_ns = now;
		chg->hold_ns = now;
		dev_dbg(chg->dev, "source event, re-identifying\n");
		goto again;
	}
	line_ns = READ_ONCE(chg->line_ns);
	if (line_ns && !tristar_debouncing(chg, now)) {
		/* The status was read again after the debounce: the input stayed. */
		cmpxchg(&chg->line_ns, line_ns, 0);
		dev_dbg(chg->dev, "source event debounced, %u mA kept\n",
			tristar_code_ma(chg, chg->code));
	}
	tristar_follow_accessory(chg, now);
	switch (chg->phase) {
	case TRISTAR_SETTLING:
		if ((s64)(now - chg->attach_ns) >= (s64)TRISTAR_LOOK_MS * NSEC_PER_MSEC)
			tristar_run_identification(chg, now);
		break;
	case TRISTAR_RAMPING:
		tristar_ramp_step(chg, now);
		break;
	case TRISTAR_CHARGING:
		tristar_guard(chg, now);
		break;
	default:
		break;
	}
again:
	ramp_delay = tristar_ramp_delay(chg, now);
	tristar_accessory_report(chg->acc, chg->usb_ma, chg->phase == TRISTAR_FAULT);
	mod_delayed_work(chg->wq, &chg->poll_work,
			 msecs_to_jiffies(TRISTAR_POLL_MS));
	if (chg->code != old_code || chg->online != old_online ||
	    chg->present != old_present || chg->usb_on != old_usb_on)
		power_supply_changed(chg->psy);
out:
	mutex_unlock(&chg->lock);
	if (ramp_delay)
		mod_delayed_work(chg->wq, &chg->poll_work, ramp_delay);
}

/* ---- PMU events ---- */

/*
 * Drop a grant to the fallback, from an event handler. No policy lock:
 * regmap serializes the write against any sequence in flight, and a grant
 * must never wait for the policy to finish. Returns true if it lowered one.
 */
static bool tristar_event_withdraw(struct tristar_charger *chg)
{
	unsigned int readback;

	if (READ_ONCE(chg->code) <= chg->chip->fallback_code)
		return false;
	if (!chg->chip->write_limit(chg, chg->chip->fallback_code) &&
	    !chg->chip->read_limit(chg, &readback) &&
	    readback == chg->chip->fallback_code)
		WRITE_ONCE(chg->code, chg->chip->fallback_code);
	else
		dev_crit(chg->dev, "PMU event: withdrawal write failed\n");
	return true;
}

static bool tristar_invalidate_source(struct tristar_charger *chg)
{
	atomic_inc(&chg->epoch);
	/* Publish invalidation before checking for a concurrent current grant. */
	smp_mb__after_atomic();
	return tristar_event_withdraw(chg);
}

/*
 * The PMU line asserted and the event controller has not read the events
 * yet. iOS acts only on the events themselves, so there is nothing to do.
 */
static irqreturn_t tristar_line_irq(int irq, void *data)
{
	struct tristar_charger *chg = data;

	if (!chg->chip->invalidate_on_line)
		return IRQ_HANDLED;

	if (tristar_invalidate_source(chg))
		WRITE_ONCE(chg->line_ns, ktime_get_ns() ?: 1);
	return IRQ_HANDLED;
}

/*
 * iOS 16 subscribes the same D2365 events (AppleD2365PMUPowerSource) and
 * hands each to AppleARMPMUCharger as a power-source change. That neither
 * lowers the input limit nor identifies the source again: it holds the ramp
 * and reads the status again 50 ms after the last event. Do the same once
 * the source is identified. Returns false before then.
 */
static bool tristar_event_debounce(struct tristar_charger *chg)
{
	enum tristar_phase phase = READ_ONCE(chg->phase);

	if (phase != TRISTAR_RAMPING && phase != TRISTAR_CHARGING)
		return false;
	WRITE_ONCE(chg->line_ns, ktime_get_ns() ?: 1);
	/* Never later than a poll already queued: events must not starve it. */
	queue_delayed_work(chg->wq, &chg->poll_work,
			   msecs_to_jiffies(TRISTAR_EVENT_DEBOUNCE_MS));
	return true;
}

/*
 * A PMU event that can mean a source change. Once the source is identified it
 * is debounced. Before that, withdraw any grant, and a new epoch invalidates
 * the identification in progress.
 */
static void tristar_source_event(struct tristar_charger *chg)
{
	unsigned int i;
	u64 now;

	if (!chg->chip->invalidate_on_line && tristar_event_debounce(chg)) {
		now = ktime_get_ns();
		goto count;
	}

	tristar_invalidate_source(chg);
	/* The USB session may continue: ask its controller for a fresh report. */
	power_supply_changed(chg->psy);
	mod_delayed_work(chg->wq, &chg->poll_work, 0);

	now = ktime_get_ns();
count:
	if (now - chg->storm_start_ns > NSEC_PER_SEC) {
		chg->storm_start_ns = now;
		chg->storm_count = 0;
	}
	if (++chg->storm_count > TRISTAR_IRQ_STORM) {
		WRITE_ONCE(chg->phase, TRISTAR_FAULT);
		if (tristar_set_code(chg, chg->chip->fallback_code))
			dev_crit(chg->dev, "PMU source events storming; input limit unverified\n");
		else
			dev_crit(chg->dev, "PMU source events storming; holding the fallback\n");
		for (i = TRISTAR_IRQ_SOURCE; i < chg->nirqs; i++)
			disable_irq_nosync(chg->irqs[i]);
	}
}

static irqreturn_t tristar_source_irq(int irq, void *data)
{
	struct tristar_charger *chg = data;

	dev_dbg(chg->dev, "PMU source event %lu\n",
		irqd_to_hwirq(irq_get_irq_data(irq)));
	tristar_source_event(chg);
	return IRQ_HANDLED;
}

/*
 * The event controller dispatches in ascending event order and ADC
 * completion has the highest number, so a source event latched together
 * with it has already started a new epoch: the waiter then discards its
 * result instead of accepting a conversion that straddled a swap.
 */
static irqreturn_t tristar_adc_irq(int irq, void *data)
{
	struct tristar_charger *chg = data;

	if (READ_ONCE(chg->adc_waiting)) {
		WRITE_ONCE(chg->adc_waiting, false);
		complete(&chg->adc_done);
	} else {
		/* Nobody started that conversion. */
		dev_dbg(chg->dev, "unsolicited ADC completion: source event\n");
		tristar_source_event(chg);
	}
	return IRQ_HANDLED;
}

/* ---- power_supply ---- */

static int tristar_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct tristar_charger *chg = power_supply_get_drvdata(psy);

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg->usb_type;
		if (val->intval == POWER_SUPPLY_USB_TYPE_UNKNOWN && chg->usb_on)
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = tristar_code_ma(chg, chg->code) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* What the identified source or USB host allows; 0 when unknown. */
		val->intval = min(max(chg->source_ma, tristar_usb_ma(chg)),
				  chg->max_ma) * 1000;
		break;
	default:
		mutex_unlock(&chg->lock);
		return -EINVAL;
	}
	mutex_unlock(&chg->lock);
	return 0;
}

/*
 * The USB device controller reports what the host allows the device to draw
 * (vbus_draw) as the input current limit: 0 without a session, 100 mA
 * unconfigured, the configuration's bMaxPower, 2 mA suspended, or more after
 * an Apple host's current grant. Not writable from user space. Only a
 * session this driver turned on counts: without VBUS sensing the controller
 * keeps reporting bus suspend and resume after its host is gone.
 *
 * A system sleep disconnects the device controller from its host: that is
 * not the host lowering its grant, and iOS keeps the granted current across
 * sleep. A report from prepare to complete is held and applied at complete;
 * after the wake the host's enumeration sets the budget as usual. No supply
 * change is reported while the system sleeps: it is a wakeup event, and it
 * would abort the suspend that caused it. A plug or unplug wakes the system
 * and ends the attachment, which drops a held report.
 */
static int tristar_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct tristar_charger *chg = power_supply_get_drvdata(psy);
	unsigned int old_code, old_ma, report_ma, accept_ma;

	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;

	mutex_lock(&chg->lock);
	old_code = chg->code;
	old_ma = tristar_usb_ma(chg);
	report_ma = max(val->intval, 0) / 1000;
	accept_ma = chg->present && chg->usb_on ? min(report_ma, chg->max_ma) : 0;
	if (chg->sleeping) {
		chg->usb_report_ma = accept_ma;
		chg->usb_report_held = accept_ma != old_ma;
		dev_dbg(chg->dev, "USB budget: reported %u mA, %u mA kept over sleep\n",
			report_ma, old_ma);
		mutex_unlock(&chg->lock);
		return 0;
	}
	chg->usb_report_held = false;
	chg->usb_ma = accept_ma;
	dev_dbg(chg->dev,
		"USB budget: reported %u mA, accepted %u mA, present %u, data %u\n",
		report_ma, chg->usb_ma, chg->present, chg->usb_on);
	if (!chg->stopping) {
		tristar_update_target(chg);
		mod_delayed_work(chg->wq, &chg->poll_work, 0);
	}
	mutex_unlock(&chg->lock);

	if (READ_ONCE(chg->code) != old_code || old_ma != READ_ONCE(chg->usb_ma))
		power_supply_changed(psy);
	return 0;
}

static const enum power_supply_property tristar_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc tristar_desc = {
	.name = "lightning",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID) |
		     BIT(POWER_SUPPLY_USB_TYPE_C) |
		     (IS_ENABLED(CONFIG_APPLE_TRISTAR_ACCESSORY) ?
		      BIT(POWER_SUPPLY_USB_TYPE_ACA) : 0),
	.properties = tristar_properties,
	.num_properties = ARRAY_SIZE(tristar_properties),
	.get_property = tristar_get_property,
	.set_property = tristar_set_property,
};

/* ---- Probe ---- */

static int tristar_get_bank(struct device *dev, const char *name, u32 size,
			    u32 *base)
{
	int index = of_property_match_string(dev->of_node, "reg-names", name);
	u64 address, length;
	int ret;

	if (index < 0)
		return dev_err_probe(dev, index, "missing %s register range\n", name);
	ret = of_property_read_reg(dev->of_node, index, &address, &length);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read %s register range\n", name);
	if (length < size || address + size - 1 > U32_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid %s register range\n", name);
	*base = address;
	return 0;
}

static void tristar_put_client(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

static void tristar_put_role_switch(void *data)
{
	usb_role_switch_put(data);
}

#include "apple_tristar_d2355.h"

/* Leave the limit as iBoot did, so a reboot or a kexec'd kernel starts safe. */
static void tristar_stop(struct tristar_charger *chg, const char *reason)
{
	unsigned int i;

	mutex_lock(&chg->lock);
	chg->stopping = true;
	mutex_unlock(&chg->lock);
	for (i = 0; i < chg->nirqs; i++)
		disable_irq(chg->irqs[i]);
	cancel_delayed_work_sync(&chg->poll_work);
	tristar_accessory_stop(chg->acc);
	mutex_lock(&chg->lock);
	if (tristar_set_code(chg, chg->chip->handoff_code))
		dev_crit(chg->dev, "%s: handoff restore failed\n", reason);
	/* Without this driver nothing holds USB data off. */
	tristar_set_usb(chg, true);
	mutex_unlock(&chg->lock);
}

static void tristar_stop_action(void *data)
{
	tristar_stop(data, "remove");
}

/* IRQ resources unwind first; stop work before the power supply is freed. */
static void tristar_abort_probe(void *data)
{
	struct tristar_charger *chg = data;

	if (chg->probe_ready)
		return;
	mutex_lock(&chg->lock);
	chg->stopping = true;
	mutex_unlock(&chg->lock);
	cancel_delayed_work_sync(&chg->poll_work);
	mutex_lock(&chg->lock);
	if (tristar_set_code(chg, chg->chip->handoff_code))
		dev_crit(chg->dev, "probe failure: input handoff failed\n");
	tristar_set_usb(chg, true);
	mutex_unlock(&chg->lock);
}

static int tristar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct tristar_charger *chg;
	struct device_node *np;
	unsigned int code, i;
	u32 max_ua;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;
	chg->dev = dev;
	atomic_set(&chg->epoch, 0);
	chg->pmic = device_get_match_data(dev);
	if (!chg->pmic)
		return -ENODEV;
	chg->chip = chg->pmic->chip;
	chg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	chg->adc_regmap = chg->regmap;
	ret = tristar_get_bank(dev, "status",
			       max_t(unsigned int, chg->chip->status_size,
				     max(chg->pmic->connected, chg->pmic->status) + 1),
			       &chg->status_base);
	if (!ret)
		ret = tristar_get_bank(dev, "control",
				       max_t(unsigned int, chg->chip->control_size,
					     chg->pmic->input_limit + 1),
				       &chg->control_base);
	if (!ret)
		ret = tristar_get_bank(dev, "adc", chg->pmic->adc_result + 2, &chg->adc_base);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "input-current-limit-microamp", &max_ua);
	if (ret)
		return dev_err_probe(dev, ret, "missing board input-current maximum\n");
	chg->max_ma = max_ua / 1000;
	if (device_property_present(dev, "io-channels")) {
		if (!IS_REACHABLE(CONFIG_IIO))
			return -EOPNOTSUPP;
		chg->vbus = devm_iio_channel_get(dev, "vbus");
		if (IS_ERR(chg->vbus))
			return dev_err_probe(dev, PTR_ERR(chg->vbus), "VBUS telemetry unavailable\n");
	}

	if (chg->chip->prepare) {
		ret = chg->chip->prepare(chg);
		if (ret)
			return dev_err_probe(dev, ret, "PMIC preparation failed\n");
	}

	np = of_parse_phandle(dev->of_node, "apple,tristar", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "missing apple,tristar\n");
	chg->tristar = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!chg->tristar)
		return dev_err_probe(dev, -EPROBE_DEFER, "Tristar not available yet\n");
	ret = devm_add_action_or_reset(dev, tristar_put_client, chg->tristar);
	if (ret)
		return ret;
	if (!i2c_check_functionality(chg->tristar->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP, "Tristar bus lacks plain I2C\n");

	chg->role_sw = usb_role_switch_get(dev);
	if (IS_ERR(chg->role_sw))
		return dev_err_probe(dev, PTR_ERR(chg->role_sw),
				     "USB role switch not available\n");
	if (!chg->role_sw)
		return dev_err_probe(dev, -EINVAL, "missing USB role switch connection\n");
	ret = devm_add_action_or_reset(dev, tristar_put_role_switch, chg->role_sw);
	if (ret)
		return ret;

	ret = platform_irq_count(pdev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to count the PMU event interrupts\n");
	if (ret < TRISTAR_IRQ_SOURCE || ret > TRISTAR_IRQS_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "expected the PMU line and ADC interrupts\n");
	chg->nirqs = ret;
	for (i = 0; i < chg->nirqs; i++) {
		chg->irqs[i] = platform_get_irq(pdev, i);
		if (chg->irqs[i] < 0)
			return chg->irqs[i];
		if (i >= TRISTAR_IRQ_SOURCE &&
		    irqd_to_hwirq(irq_get_irq_data(chg->irqs[i])) == chg->pmic->plug_event)
			chg->plug_irq = chg->irqs[i];
	}

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;
	init_completion(&chg->adc_done);
	chg->wq = devm_alloc_workqueue(dev, "tristar-charger",
				       WQ_UNBOUND | WQ_HIGHPRI | WQ_FREEZABLE, 1);
	if (!chg->wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&chg->poll_work, tristar_poll);
	ret = tristar_accessory_probe(dev, chg->tristar, &chg->lock, &chg->acc);
	if (ret)
		return ret;

	/* Never adopt a limit we cannot explain; anything higher drops to the fallback. */
	ret = chg->chip->read_limit(chg, &code);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the input-current limit\n");
	chg->code = code;

	config.drv_data = chg;
	config.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &tristar_desc, &config);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "failed to register power supply\n");

	/* Register before changing limits or requesting IRQs. */
	ret = devm_add_action_or_reset(dev, tristar_abort_probe, chg);
	if (ret)
		return ret;
	if (code > chg->chip->fallback_code) {
		dev_warn(dev, "Input limit %u mA at probe; dropping to %u mA\n",
			 tristar_code_ma(chg, code), tristar_code_ma(chg, chg->chip->fallback_code));
		ret = tristar_set_code(chg, chg->chip->fallback_code);
		if (ret)
			return dev_err_probe(dev, ret, "failed to lower the input-current limit\n");
	}


	for (i = 0; i < chg->nirqs; i++) {
		ret = devm_request_threaded_irq(dev, chg->irqs[i], NULL,
						i == TRISTAR_IRQ_LINE ? tristar_line_irq :
						i == TRISTAR_IRQ_ADC ? tristar_adc_irq :
						tristar_source_irq,
						IRQF_ONESHOT, dev_name(dev), chg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request PMU event %u\n", i);
	}

	platform_set_drvdata(pdev, chg);
	ret = devm_add_action_or_reset(dev, tristar_stop_action, chg);
	if (ret)
		return ret;
	chg->probe_ready = true;

	if (chg->plug_irq) {
		ret = device_init_wakeup(dev, true);
		if (ret)
			return ret;
	}

	/* Whatever is attached gets identified before USB data comes on. */
	mutex_lock(&chg->lock);
	chg->hold_ns = ktime_get_ns();
	tristar_set_usb(chg, false);
	mutex_unlock(&chg->lock);

	mod_delayed_work(chg->wq, &chg->poll_work, 0);
	tristar_accessory_start(chg->acc);
	return 0;
}

static void tristar_shutdown(struct platform_device *pdev)
{
	struct tristar_charger *chg = platform_get_drvdata(pdev);

	if (chg)
		tristar_stop(chg, "shutdown");
}

/*
 * From here until complete, a USB budget report is the device controller's
 * own suspend (it disconnects from its host), not the host's: see
 * tristar_set_property().
 */
static int tristar_prepare(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);

	mutex_lock(&chg->lock);
	chg->sleeping = true;
	mutex_unlock(&chg->lock);
	return 0;
}

static void tristar_complete(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);
	bool changed = false;

	mutex_lock(&chg->lock);
	chg->sleeping = false;
	if (chg->usb_report_held) {
		chg->usb_report_held = false;
		chg->usb_ma = chg->usb_report_ma;
		tristar_update_target(chg);
		mod_delayed_work(chg->wq, &chg->poll_work, 0);
		changed = true;
	}
	mutex_unlock(&chg->lock);

	if (changed)
		power_supply_changed(chg->psy);
}

/*
 * A plug or unplug wakes the system, so sleep hides no source swap and an
 * identified source keeps its grant, as in iOS 16, which keeps a USB host's
 * granted current across sleep too (IOAccessoryUSBCurrentLimit): a tablet
 * left on a charger or a computer charges at the same rate while it sleeps.
 * Anything without a plug event to rely on parks at the handoff and is
 * identified again on resume.
 *
 * Sleep never waits for the charger. iOS 16 (AppleARMPMUCharger) cancels the
 * ramp on sleep and leaves the limit as written; after wake the ramp goes on
 * from there. So does this driver: a ramp in progress keeps the limit it has
 * reached, which is never above the grant, and continues on resume. An
 * identification in progress is abandoned: it runs within one poll, so the
 * mux is back in place here, and the source draws the handoff it already
 * has until it is identified again on resume. The poll work is frozen by
 * now, so waiting for either would wait forever.
 */
static int tristar_suspend(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);
	bool keep;
	int ret = 0;

	mutex_lock(&chg->lock);
	keep = chg->plug_irq && device_may_wakeup(dev) && chg->present &&
	       (chg->phase == TRISTAR_RAMPING || chg->phase == TRISTAR_CHARGING);
	chg->stopping = true;
	if (!keep) {
		chg->present = false;
		chg->phase = TRISTAR_EMPTY;
		ret = tristar_set_code(chg, chg->chip->handoff_code);
		if (ret)
			chg->stopping = false;
	}
	mutex_unlock(&chg->lock);
	if (ret)
		return ret;
	cancel_delayed_work_sync(&chg->poll_work);
	tristar_accessory_stop(chg->acc);
	if (chg->plug_irq && device_may_wakeup(dev)) {
		ret = enable_irq_wake(chg->plug_irq);
		if (ret)
			dev_warn(dev, "cannot wake on plug events: %d\n", ret);
	}

	return 0;
}

static int tristar_resume(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);

	if (chg->plug_irq && device_may_wakeup(dev))
		disable_irq_wake(chg->plug_irq);
	mutex_lock(&chg->lock);
	chg->stopping = false;
	mutex_unlock(&chg->lock);
	mod_delayed_work(chg->wq, &chg->poll_work, 0);
	tristar_accessory_start(chg->acc);
	return 0;
}

static const struct dev_pm_ops tristar_pm_ops = {
	.prepare = pm_sleep_ptr(tristar_prepare),
	.complete = pm_sleep_ptr(tristar_complete),
	SYSTEM_SLEEP_PM_OPS(tristar_suspend, tristar_resume)
};

static const struct of_device_id tristar_of_match[] = {
	{ .compatible = "apple,d2089-tristar-charger", .data = &tristar_d2089 },
	{ .compatible = "apple,d2207-tristar-charger", .data = &tristar_d2207 },
	{ .compatible = "apple,tristar-charger", .data = &tristar_d2365 },
	{ .compatible = "apple,d2355-tristar-charger", .data = &tristar_d2355 },
	{ }
};
MODULE_DEVICE_TABLE(of, tristar_of_match);

static struct platform_driver tristar_driver = {
	.driver = {
		.name = "apple-tristar-charger",
		.of_match_table = tristar_of_match,
		.pm = pm_sleep_ptr(&tristar_pm_ops),
	},
	.probe = tristar_probe,
	.shutdown = tristar_shutdown,
};
module_platform_driver(tristar_driver);

MODULE_DESCRIPTION("Apple Tristar Lightning charging");
MODULE_LICENSE("GPL");
