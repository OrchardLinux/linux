// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lightning accessory policy of the Tristar charger: USB host mode for the
 * Lightning-to-USB camera adapter.
 *
 * The camera adapter announces itself with digital ID 11 f0 (USB, UART and
 * power pins). On it iOS runs the post-ID exchange (interface information,
 * serials, SystemInfo), makes the port a USB host, and then agrees on power
 * with the adapter over the ID bus; the adapter's hub enumerates at high
 * speed. There is no device-mode phase in between. This policy does the
 * same through the port's USB role switch, which owns the host controllers
 * and the Tristar's data and accessory-power routes. When the adapter is
 * unplugged, the port returns to the USB device controller.
 *
 * The powered camera adapter supplies input power while the iPad hosts
 * its hub. After its power handshake, use the 2.1 A profile validated on
 * this adapter with iOS and Linux. Before a successful handshake, keep
 * the unidentified-source fallback. The charger still applies its board
 * maximum and battery guards; no USB data-line sampling is needed here.
 *
 * Every 250 ms the policy reads the Tristar status (0x0c: bits 0-1 VBUS,
 * bit 6 orientation, bit 7 an attached plug). With a plug attached it asks
 * for the digital ID 300 ms after the plug edge, then every 2 s, or every
 * 10 s while a USB host uses the gadget; with the adapter in host mode it
 * checks the ID every 5 s. An unplug is three empty reads in a row; while
 * the adapter is ours, bit 7 alone counts (an unplug in host mode can leave
 * the VBUS bits set) and the wait is 2 s; bit 7 back after 0.5 s or more is
 * a replug and starts over, as iOS identifies on every attach.
 *
 * Sharing the Tristar with the charger: every Tristar access of either side
 * holds the charger's lock, so ID-bus transfers and mux samples never
 * interleave. The charger samples the mux only while the policy does not
 * hold the port: from the request for host mode until it is given back, and
 * around each change of accessory power or USB role, the charger takes the
 * adapter's allowance instead of identifying the source (see
 * tristar_accessory_holds()). The hold is taken under the lock before any
 * role-switch call, so no mux sample is in flight when the role switch
 * rewrites the routes. Role-switch calls are never made with the lock held:
 * the role switch calls back into the charger (the status sample below, the
 * gadget's vbus_draw) with its own lock held.
 *
 * The digital ID comes first. iOS measures the USB data lines only through
 * the route that the decoded ID selects (its pin-voltage operation refuses
 * any other route), so it never drives or samples them before the ID is
 * known. The charger likewise leaves the mux alone from a plug edge until
 * the first GetID after it has finished (see
 * tristar_accessory_identifying()).
 *
 * Copyright The Orchard Linux contributors
 */

#include <linux/array_size.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/usb/apple-s8000-role.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "apple_tristar_accessory.h"
#include "apple_tristar_idbus.h"

/* Tristar status 0x0c: both low bits are strict VBUS, bit 7 an attached plug. */
#define TRISTAR_AUTO_VBUS		0x03
#define TRISTAR_AUTO_PRESENT		0x80
#define TRISTAR_AUTO_ORIENTATION	0x40

#define TRISTAR_ACCESSORY_POLL_MS	250
/* A power handshake needs a digital ID read this recently. */
#define TRISTAR_ACCESSORY_ID_WINDOW_MS	30000
/* The gadget's draw before configuration; more means a configured session. */
#define TRISTAR_ACCESSORY_UNCONFIGURED_MA 100

/*
 * The SystemInfo values iOS sends: the root model number and its OS build.
 * The adapter does not answer them. Without a model number in the device
 * tree, send the values this exchange was validated with.
 */
#define TRISTAR_ACCESSORY_MODEL		"MP2G2"
#define TRISTAR_ACCESSORY_OSVERSION	"20H350"

enum tristar_auto_phase {
	TRISTAR_AUTO_EMPTY,	/* nothing attached */
	TRISTAR_AUTO_DEVICE,	/* attached, device role, GetID pending */
	TRISTAR_AUTO_OTHER,	/* identified, not a host-mode accessory */
	TRISTAR_AUTO_ENTER,	/* switch to host requested */
	TRISTAR_AUTO_HANDSHAKE,	/* host is on; wait for VBUS, then 0x72/0x70 */
	TRISTAR_AUTO_HOST,	/* host role, owned by this policy */
	TRISTAR_AUTO_INHIBIT,	/* role taken over or entry failed; wait for unplug */
};

enum tristar_auto_action {
	TRISTAR_AUTO_NONE,
	TRISTAR_AUTO_PROBE,		/* GetID */
	TRISTAR_AUTO_POWER_HANDSHAKE,
	TRISTAR_AUTO_TO_HOST,
	TRISTAR_AUTO_TO_DEVICE,
};

struct tristar_auto_cfg {
	unsigned int probe_ms;		/* GetID period, no active USB link */
	unsigned int busy_probe_ms;	/* ... while a USB host uses the gadget */
	unsigned int verify_ms;		/* host role: re-check the ID */
	unsigned int settle_ms;		/* after a plug edge */
	unsigned int detach_polls;	/* consecutive empty 0x0c reads */
	unsigned int owned_detach_polls; /* ... adapter ours, bit 7 clear but VBUS bits set */
	unsigned int owned_replug_polls; /* bit 7 clear this long, then back: a replug */
	unsigned int handshake_tries;
	unsigned int handshake_wait_ms;	/* host-on deadline for strict VBUS */
	int input_ua;			/* host role, after a good power handshake */
	int floor_ua;			/* host role, no power handshake */
};

static const struct tristar_auto_cfg tristar_auto_cfg = {
	.probe_ms = 2000,
	.busy_probe_ms = 10000,
	.verify_ms = 5000,
	.settle_ms = 300,
	.detach_polls = 3,
	.owned_detach_polls = 8,
	.owned_replug_polls = 2,
	.handshake_tries = 2,
	.handshake_wait_ms = 15000,
	.input_ua = 2100000,
	.floor_ua = 500000,
};

struct tristar_auto {
	enum tristar_auto_phase phase;
	int status;			/* last good 0x0c, or -1 */
	unsigned int empty;		/* consecutive empty 0x0c reads */
	bool was_empty;			/* the previous read was empty */
	unsigned long next_probe;
	u8 id[TRISTAR_IDBUS_ID_LEN];
	bool id_valid;
	bool adapter;			/* id selects host mode */
	unsigned int ph_tries;
	unsigned long ph_deadline;
	bool ph_ok;
	bool id_pending;		/* no GetID result since the plug edge */
};

struct tristar_auto_in {
	unsigned long now;
	int status;		/* 0x0c or -errno */
	bool host;		/* the role switch is in host mode */
	bool link_active;	/* a USB host configured the gadget, not suspended */
};

/**
 * struct tristar_accessory - Lightning accessory policy
 * @dev: the charger
 * @lock: the charger's lock; serializes every Tristar access and the fields below
 * @role_np: the role switch's node
 * @role_sw: the Lightning port's role switch, once it is registered
 * @work: the policy's poll
 * @ib: ID-bus master
 * @autop: policy state
 * @stopped: no poll is queued and none may queue itself
 * @status: last Tristar status read, or -errno
 * @vbus: strict VBUS was seen since the last detach
 * @detach_role: VBUS went away; give the role back at the next poll
 * @host: the role switch was in host mode at the last poll
 * @busy: an accessory-power or role change is in progress
 * @usb_ma: what the USB host allows the gadget, as the charger last saw it
 * @mux_fault: the charger could not verify a Tristar mux restore
 * @id_ok: the last GetID succeeded, at @id_at (jiffies)
 * @id_at: see @id_ok
 * @source_id: the digital ID of the attached accessory
 * @source_id_valid: @source_id holds one
 * @info_valid: the post-ID exchange ran for this attachment
 * @model: SystemInfo model number
 * @model_len: its length
 */
struct tristar_accessory {
	struct device *dev;
	struct mutex *lock;	/* the charger's: every Tristar access */
	struct device_node *role_np;
	struct usb_role_switch *role_sw;
	struct delayed_work work;
	struct tristar_idbus ib;
	struct tristar_auto autop;
	bool stopped;
	int status;
	bool vbus;
	bool detach_role;
	bool host;
	bool busy;
	unsigned int usb_ma;
	bool mux_fault;
	bool id_ok;
	unsigned long id_at;
	u8 source_id[TRISTAR_IDBUS_ID_LEN];
	bool source_id_valid;
	bool info_valid;
	u8 model[TRISTAR_IDBUS_INFO_MAX];
	unsigned int model_len;
};

/* ---- Decisions: no I/O here; the caller runs the action and reports back ---- */

static bool tristar_auto_due(unsigned long now, unsigned long when)
{
	return (long)(now - when) >= 0;
}

/* The camera adapter, digital ID 11 f0 ..: USB=2 UART=1 PWR=0xf. */
static bool tristar_auto_is_host_adapter(const u8 *id)
{
	return id[0] == 0x11 && id[1] == 0xf0;
}

static bool tristar_auto_attached(int status)
{
	return status >= 0 && (status & (TRISTAR_AUTO_PRESENT | TRISTAR_AUTO_VBUS));
}

/*
 * The camera adapter belongs to us from the host request until it is gone.
 * An unplug in host mode can read 0x0c = 0x25: bit 7 (plug present) clear,
 * VBUS bit 0 still set. Only bit 7 says the plug is there.
 */
static bool tristar_auto_owned(const struct tristar_auto *ta)
{
	return ta->id_valid && ta->adapter &&
		(ta->phase == TRISTAR_AUTO_ENTER || ta->phase == TRISTAR_AUTO_HANDSHAKE ||
		 ta->phase == TRISTAR_AUTO_HOST);
}

/*
 * iOS identifies the accessory again on every attach. Forget the adapter
 * information whenever the policy enters a phase without an owned adapter.
 */
static bool tristar_auto_reidentify(enum tristar_auto_phase old,
				    enum tristar_auto_phase now)
{
	bool was = old == TRISTAR_AUTO_EMPTY || old == TRISTAR_AUTO_DEVICE;
	bool is = now == TRISTAR_AUTO_EMPTY || now == TRISTAR_AUTO_DEVICE;

	return is && !was;
}

/* Host entry can briefly remove strict VBUS while the same adapter is present. */
static bool tristar_auto_hold_source(const struct tristar_auto *ta, int status)
{
	return status >= 0 && (status & TRISTAR_AUTO_PRESENT) && ta->id_valid && ta->adapter &&
		(ta->phase == TRISTAR_AUTO_ENTER || ta->phase == TRISTAR_AUTO_HANDSHAKE ||
		 ta->phase == TRISTAR_AUTO_HOST);
}

static const char *tristar_auto_phase_name(enum tristar_auto_phase phase)
{
	static const char *const names[] = {
		[TRISTAR_AUTO_EMPTY] = "empty",
		[TRISTAR_AUTO_DEVICE] = "device",
		[TRISTAR_AUTO_OTHER] = "other",
		[TRISTAR_AUTO_ENTER] = "enter-host",
		[TRISTAR_AUTO_HANDSHAKE] = "handshake",
		[TRISTAR_AUTO_HOST] = "host",
		[TRISTAR_AUTO_INHIBIT] = "inhibit",
	};

	return (unsigned int)phase < ARRAY_SIZE(names) ? names[phase] : "?";
}

static void tristar_auto_forget(struct tristar_auto *ta)
{
	ta->id_valid = false;
	ta->adapter = false;
	ta->ph_ok = false;
	ta->ph_tries = 0;
}

static void tristar_auto_init(struct tristar_auto *ta)
{
	*ta = (struct tristar_auto){ .phase = TRISTAR_AUTO_EMPTY, .status = -1 };
}

/* One poll. The caller then runs the returned action and reports back. */
static enum tristar_auto_action
tristar_auto_step(struct tristar_auto *ta, const struct tristar_auto_cfg *cfg,
		  const struct tristar_auto_in *in)
{
	const int plug_bits = TRISTAR_AUTO_PRESENT | TRISTAR_AUTO_ORIENTATION;
	bool attached, edge, replug, owned;
	unsigned int limit, was_empty;
	int prev;

	/* An I2C error is no evidence either way. */
	if (in->status < 0)
		return TRISTAR_AUTO_NONE;

	owned = tristar_auto_owned(ta);
	attached = owned ? in->status >= 0 && (in->status & TRISTAR_AUTO_PRESENT) :
			   tristar_auto_attached(in->status);
	was_empty = ta->empty;
	prev = ta->status;
	ta->status = in->status;
	replug = attached && ta->was_empty;
	ta->was_empty = !attached;
	ta->empty = attached ? 0 : ta->empty + 1;
	edge = replug || (prev >= 0 && attached && ((prev ^ in->status) & plug_bits));

	/* A fully empty 0x0c is an unplug at once; bit 7 clear with VBUS lingering waits longer. */
	limit = tristar_auto_owned(ta) && tristar_auto_attached(in->status) ?
		cfg->owned_detach_polls : cfg->detach_polls;
	if (ta->empty >= limit && ta->phase != TRISTAR_AUTO_EMPTY) {
		bool release = ta->adapter && in->host;

		ta->phase = TRISTAR_AUTO_EMPTY;
		tristar_auto_forget(ta);
		return release ? TRISTAR_AUTO_TO_DEVICE : TRISTAR_AUTO_NONE;
	}

	/*
	 * A replug in host mode (bit 7 clear for a second or more, then back)
	 * leaves the adapter unidentified. Release the role and attach it
	 * again, as iOS does on every attach.
	 */
	if (owned && attached && was_empty >= cfg->owned_replug_polls &&
	    ta->phase != TRISTAR_AUTO_EMPTY) {
		bool release = in->host;

		ta->phase = TRISTAR_AUTO_EMPTY;
		tristar_auto_forget(ta);
		return release ? TRISTAR_AUTO_TO_DEVICE : TRISTAR_AUTO_NONE;
	}

	switch (ta->phase) {
	case TRISTAR_AUTO_EMPTY:
		if (attached) {
			ta->phase = TRISTAR_AUTO_DEVICE;
			ta->next_probe = in->now + cfg->settle_ms;
			ta->id_pending = true;
		}
		return TRISTAR_AUTO_NONE;
	case TRISTAR_AUTO_DEVICE:
	case TRISTAR_AUTO_OTHER:
		/* No identification while the port is still a host. */
		if (!attached || in->host)
			return TRISTAR_AUTO_NONE;
		if (edge) {
			ta->phase = TRISTAR_AUTO_DEVICE;
			ta->next_probe = in->now + cfg->settle_ms;
			ta->id_pending = true;
		}
		if (!tristar_auto_due(in->now, ta->next_probe))
			return TRISTAR_AUTO_NONE;
		ta->next_probe = in->now + (in->link_active ? cfg->busy_probe_ms : cfg->probe_ms);
		return TRISTAR_AUTO_PROBE;
	case TRISTAR_AUTO_HANDSHAKE:
		if (!in->host) {
			ta->phase = TRISTAR_AUTO_INHIBIT;
			return TRISTAR_AUTO_NONE;
		}
		/* Without strict VBUS the handshake would fail: do not spend tries on it. */
		if ((in->status & TRISTAR_AUTO_VBUS) == TRISTAR_AUTO_VBUS)
			return TRISTAR_AUTO_POWER_HANDSHAKE;
		if (tristar_auto_due(in->now, ta->ph_deadline))
			ta->phase = TRISTAR_AUTO_HOST;
		return TRISTAR_AUTO_NONE;
	case TRISTAR_AUTO_ENTER:
		if (in->host) {
			ta->phase = TRISTAR_AUTO_HANDSHAKE;
			ta->ph_deadline = in->now + cfg->handshake_wait_ms;
			return TRISTAR_AUTO_NONE;
		}
		return TRISTAR_AUTO_TO_HOST;
	case TRISTAR_AUTO_HOST:
		if (!in->host) {
			ta->phase = TRISTAR_AUTO_INHIBIT;
			return TRISTAR_AUTO_NONE;
		}
		if (!attached)
			return TRISTAR_AUTO_NONE;
		if (edge || tristar_auto_due(in->now, ta->next_probe)) {
			ta->next_probe = in->now + cfg->verify_ms;
			return TRISTAR_AUTO_PROBE;
		}
		return TRISTAR_AUTO_NONE;
	case TRISTAR_AUTO_INHIBIT:
		break;
	}
	return TRISTAR_AUTO_NONE;
}

/* GetID finished: ret 0 with a valid id, or -errno. */
static enum tristar_auto_action
tristar_auto_probed(struct tristar_auto *ta, const struct tristar_auto_cfg *cfg,
		    unsigned long now, int ret, const u8 *id)
{
	bool adapter;

	ta->id_pending = false;
	if (ta->phase != TRISTAR_AUTO_DEVICE && ta->phase != TRISTAR_AUTO_OTHER &&
	    ta->phase != TRISTAR_AUTO_HOST)
		return TRISTAR_AUTO_NONE;
	if (ret)
		return TRISTAR_AUTO_NONE;
	adapter = tristar_auto_is_host_adapter(id);
	memcpy(ta->id, id, TRISTAR_IDBUS_ID_LEN);
	ta->id_valid = true;
	if (ta->phase == TRISTAR_AUTO_HOST) {
		if (adapter)
			return TRISTAR_AUTO_NONE;
		/* A fast swap to another accessory, e.g. a computer's cable. */
		ta->adapter = false;
		ta->ph_ok = false;
		ta->phase = TRISTAR_AUTO_OTHER;
		ta->next_probe = now + cfg->probe_ms;
		return TRISTAR_AUTO_TO_DEVICE;
	}
	ta->adapter = adapter;
	if (!adapter) {
		ta->phase = TRISTAR_AUTO_OTHER;
		return TRISTAR_AUTO_NONE;
	}
	ta->phase = TRISTAR_AUTO_ENTER;
	ta->ph_ok = false;
	ta->ph_tries = 0;
	return TRISTAR_AUTO_TO_HOST;
}

/*
 * Power handshake finished. Host is already on: iOS enumerates the hub
 * before its handshake completes. After the last try, keep the port in host
 * mode at the fallback input budget.
 */
static void tristar_auto_handshake_done(struct tristar_auto *ta,
					const struct tristar_auto_cfg *cfg, int ret)
{
	if (ta->phase != TRISTAR_AUTO_HANDSHAKE)
		return;
	ta->ph_tries++;
	ta->ph_ok = !ret;
	if (ret && ta->ph_tries < cfg->handshake_tries)
		return;
	ta->phase = TRISTAR_AUTO_HOST;
}

/* Host entry finished. No retry on failure: the next unplug starts over. */
static void tristar_auto_host_done(struct tristar_auto *ta,
				   const struct tristar_auto_cfg *cfg,
				   unsigned long now, int ret)
{
	if (ta->phase != TRISTAR_AUTO_ENTER)
		return;
	if (ret) {
		ta->phase = TRISTAR_AUTO_INHIBIT;
		return;
	}
	ta->phase = TRISTAR_AUTO_HANDSHAKE;
	ta->next_probe = now + cfg->verify_ms;
	ta->ph_deadline = now + cfg->handshake_wait_ms;
}

/*
 * Input power while the adapter is ours: use its validated 2.1 A profile
 * only after a successful power handshake and with strict VBUS present.
 * Returns false when the ordinary device-mode rules apply.
 */
static bool tristar_auto_supply(const struct tristar_auto *ta,
				const struct tristar_auto_cfg *cfg, int status,
				int *current_ua)
{
	bool online;

	if (!ta->id_valid || !ta->adapter)
		return false;
	if (ta->phase != TRISTAR_AUTO_HANDSHAKE && ta->phase != TRISTAR_AUTO_ENTER &&
	    ta->phase != TRISTAR_AUTO_HOST)
		return false;
	online = status >= 0 && (status & TRISTAR_AUTO_VBUS) == TRISTAR_AUTO_VBUS;
	*current_ua = !online ? 0 : ta->ph_ok ? cfg->input_ua : cfg->floor_ua;
	return true;
}

/* ---- Policy ---- */

static unsigned long tristar_accessory_now(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

/* A new attachment, or a changed accessory: run the post-ID exchange again. Caller holds lock. */
static void tristar_accessory_new_source(struct tristar_accessory *acc)
{
	acc->source_id_valid = false;
	acc->info_valid = false;
}

/* Caller holds lock. */
static void tristar_accessory_source_id(struct tristar_accessory *acc, const u8 *id)
{
	if (acc->source_id_valid && memcmp(acc->source_id, id, sizeof(acc->source_id)))
		tristar_accessory_new_source(acc);
	memcpy(acc->source_id, id, sizeof(acc->source_id));
	acc->source_id_valid = true;
}

/* Caller holds lock. */
static void tristar_accessory_phase(struct tristar_accessory *acc, enum tristar_auto_phase old)
{
	struct tristar_auto *ta = &acc->autop;

	if (old != TRISTAR_AUTO_EMPTY && old != ta->phase &&
	    (ta->phase == TRISTAR_AUTO_DEVICE || ta->phase == TRISTAR_AUTO_EMPTY)) {
		tristar_accessory_new_source(acc);
		tristar_auto_forget(ta);
	}
	if (tristar_auto_reidentify(old, ta->phase))
		acc->info_valid = false;
	if (old != ta->phase)
		dev_dbg(acc->dev, "accessory: %s -> %s\n", tristar_auto_phase_name(old),
			tristar_auto_phase_name(ta->phase));
}

/* Hold the port for a role-switch call; see the top of this file. */
static void tristar_accessory_busy(struct tristar_accessory *acc, bool busy)
{
	mutex_lock(acc->lock);
	acc->busy = busy;
	mutex_unlock(acc->lock);
}

/*
 * The role switch runs this with its lock held, in the device role only, so
 * that no role change overlaps the read. -EUCLEAN keeps the role switch from
 * entering host mode while the charger has a Tristar mux restore unverified.
 */
static int tristar_accessory_sample(void *data)
{
	struct tristar_accessory *acc = data;
	int ret;

	mutex_lock(acc->lock);
	acc->status = tristar_idbus_status(&acc->ib);
	ret = acc->mux_fault ? -EUCLEAN : 0;
	mutex_unlock(acc->lock);
	return ret;
}

static enum tristar_auto_action tristar_accessory_probe_id(struct tristar_accessory *acc)
{
	struct tristar_auto *ta = &acc->autop;
	u8 id[TRISTAR_IDBUS_ID_LEN] = {};
	enum tristar_auto_phase old;
	enum tristar_auto_action action;
	bool need_info = false, powered = false, same = false;
	int ret;

	mutex_lock(acc->lock);
	ret = tristar_idbus_identify(&acc->ib, id);
	acc->id_ok = !ret;
	if (!ret) {
		acc->id_at = jiffies;
		same = ta->id_valid && !memcmp(ta->id, id, sizeof(id));
		tristar_accessory_source_id(acc, id);
		need_info = tristar_auto_is_host_adapter(id) && !acc->info_valid;
		acc->busy = need_info;
	}
	mutex_unlock(acc->lock);

	/* iOS switches the accessory power on before the post-ID exchange. */
	if (need_info) {
		ret = apple_s8000_usb_role_accessory_power(acc->role_sw, true);
		powered = !ret;
		if (!ret) {
			mutex_lock(acc->lock);
			ret = tristar_idbus_accessory_info(&acc->ib, acc->model, acc->model_len,
							   (const u8 *)TRISTAR_ACCESSORY_OSVERSION,
							   strlen(TRISTAR_ACCESSORY_OSVERSION));
			mutex_unlock(acc->lock);
		}
		if (ret && powered)
			apple_s8000_usb_role_accessory_power(acc->role_sw, false);
	}

	mutex_lock(acc->lock);
	acc->busy = false;
	if (need_info && !ret)
		acc->info_valid = true;
	old = ta->phase;
	action = tristar_auto_probed(ta, &tristar_auto_cfg, tristar_accessory_now(), ret, id);
	if (!ret && !same)
		dev_dbg(acc->dev, "accessory: digital ID %*ph%s\n", (int)sizeof(id), id,
			ta->adapter ? " (camera adapter)" : "");
	else if (ret)
		dev_dbg(acc->dev, "accessory: identification failed (%d)\n", ret);
	tristar_accessory_phase(acc, old);
	mutex_unlock(acc->lock);
	return action;
}

static enum tristar_auto_action
tristar_accessory_act(struct tristar_accessory *acc, enum tristar_auto_action action)
{
	struct tristar_auto *ta = &acc->autop;
	enum tristar_auto_phase old;
	int ret = 0;

	switch (action) {
	case TRISTAR_AUTO_PROBE:
		return tristar_accessory_probe_id(acc);
	case TRISTAR_AUTO_POWER_HANDSHAKE:
		mutex_lock(acc->lock);
		if (!acc->id_ok || time_after(jiffies, acc->id_at +
					      msecs_to_jiffies(TRISTAR_ACCESSORY_ID_WINDOW_MS)))
			ret = -ENOLINK;
		else
			ret = tristar_idbus_power_handshake(&acc->ib);
		old = ta->phase;
		tristar_auto_handshake_done(ta, &tristar_auto_cfg, ret);
		dev_dbg(acc->dev, "accessory: power handshake %s (%d), try %u\n",
			ret ? "failed" : "done", ret, ta->ph_tries);
		tristar_accessory_phase(acc, old);
		mutex_unlock(acc->lock);
		break;
	case TRISTAR_AUTO_TO_HOST:
		/* The policy is in enter-host: the charger leaves the port alone. */
		ret = usb_role_switch_set_role(acc->role_sw, USB_ROLE_HOST);
		mutex_lock(acc->lock);
		old = ta->phase;
		tristar_auto_host_done(ta, &tristar_auto_cfg, tristar_accessory_now(), ret);
		tristar_accessory_phase(acc, old);
		mutex_unlock(acc->lock);
		if (ret)
			dev_warn(acc->dev, "camera adapter: USB host mode failed: %d\n", ret);
		else
			dev_dbg(acc->dev, "camera adapter: USB host mode\n");
		break;
	case TRISTAR_AUTO_TO_DEVICE:
		tristar_accessory_busy(acc, true);
		if (usb_role_switch_get_role(acc->role_sw) == USB_ROLE_HOST)
			ret = usb_role_switch_set_role(acc->role_sw, USB_ROLE_DEVICE);
		tristar_accessory_busy(acc, false);
		if (ret)
			dev_warn(acc->dev, "accessory: USB device mode failed: %d\n", ret);
		else
			dev_dbg(acc->dev, "accessory: USB device mode\n");
		break;
	case TRISTAR_AUTO_NONE:
		break;
	}
	return TRISTAR_AUTO_NONE;
}

static void tristar_accessory_step(struct tristar_accessory *acc)
{
	struct tristar_auto *ta = &acc->autop;
	struct tristar_auto_in in = {};
	enum tristar_auto_action action;
	enum tristar_auto_phase old;
	unsigned int i;
	bool detach;
	int ret;

	/* The charger does not wait for the role switch; the policy does. */
	if (!acc->role_sw) {
		acc->role_sw = usb_role_switch_find_by_fwnode(of_fwnode_handle(acc->role_np));
		if (!acc->role_sw)
			return;
	}

	ret = apple_s8000_usb_role_monitor(acc->role_sw, tristar_accessory_sample, acc);
	mutex_lock(acc->lock);
	/* Not in the device role: the role switch did not sample. */
	if (ret && ret != -EUCLEAN)
		acc->status = tristar_idbus_status(&acc->ib);
	/* VBUS gone, other than during host entry: a detach, whatever the plug bit says. */
	if (acc->status >= 0 && acc->vbus && !(acc->status & TRISTAR_AUTO_VBUS) &&
	    !tristar_auto_hold_source(ta, acc->status)) {
		acc->vbus = false;
		acc->detach_role = true;
		tristar_accessory_new_source(acc);
		tristar_auto_forget(ta);
		ta->phase = TRISTAR_AUTO_EMPTY;
		ta->was_empty = true;
		ta->empty = 0;
	}
	if (acc->status >= 0 && (acc->status & TRISTAR_AUTO_VBUS) == TRISTAR_AUTO_VBUS)
		acc->vbus = true;
	detach = acc->detach_role;
	acc->detach_role = false;
	mutex_unlock(acc->lock);
	if (detach)
		tristar_accessory_act(acc, TRISTAR_AUTO_TO_DEVICE);

	in.host = usb_role_switch_get_role(acc->role_sw) == USB_ROLE_HOST;
	mutex_lock(acc->lock);
	acc->host = in.host;
	in.link_active = acc->usb_ma > TRISTAR_ACCESSORY_UNCONFIGURED_MA;
	in.now = tristar_accessory_now();
	in.status = acc->status;
	old = ta->phase;
	action = tristar_auto_step(ta, &tristar_auto_cfg, &in);
	tristar_accessory_phase(acc, old);
	mutex_unlock(acc->lock);

	/* A probe can lead to host entry; nothing chains further. */
	for (i = 0; i < 4 && action != TRISTAR_AUTO_NONE; i++)
		action = tristar_accessory_act(acc, action);
}

static void tristar_accessory_work(struct work_struct *work)
{
	struct tristar_accessory *acc = container_of(to_delayed_work(work),
						     struct tristar_accessory, work);

	tristar_accessory_step(acc);
	mutex_lock(acc->lock);
	if (!acc->stopped)
		mod_delayed_work(system_power_efficient_wq, &acc->work,
				 msecs_to_jiffies(TRISTAR_ACCESSORY_POLL_MS));
	mutex_unlock(acc->lock);
}

/* ---- Interface to the charger ---- */

/**
 * tristar_accessory_holds - whether the policy holds the Lightning port
 * @acc: accessory policy, or NULL
 * @ma: the input allowance while held, in mA
 *
 * While held, the charger must not sample the Tristar mux and takes @ma as
 * the source's allowance. Caller holds the charger lock.
 *
 * Return: a description of the holder, or NULL when the charger may
 * identify the source itself.
 */
const char *tristar_accessory_holds(struct tristar_accessory *acc, unsigned int *ma)
{
	int ua;

	if (!acc)
		return NULL;
	if (tristar_auto_supply(&acc->autop, &tristar_auto_cfg, acc->status, &ua)) {
		*ma = ua / 1000;
		return "Lightning camera adapter, USB host mode";
	}
	*ma = 0;
	if (acc->busy)
		return "Lightning accessory, changing USB role";
	if (acc->host)
		return "USB host mode";
	return NULL;
}

/**
 * tristar_accessory_identifying - whether the attachment's digital ID is being read
 * @acc: accessory policy, or NULL
 *
 * True from a plug edge until the first GetID after it has finished, with
 * or without an ID. Meanwhile the charger must not route or excite the USB
 * data lines; it still bounds its own wait. Caller holds the charger lock.
 *
 * Return: true while that GetID is pending.
 */
bool tristar_accessory_identifying(struct tristar_accessory *acc)
{
	return acc && acc->autop.phase == TRISTAR_AUTO_DEVICE && acc->autop.id_pending;
}

/**
 * tristar_accessory_report - what the charger knows, once per charger poll
 * @acc: accessory policy, or NULL
 * @usb_ma: what the USB host allows the gadget (vbus_draw), 0 without one
 * @mux_fault: the charger could not verify a Tristar mux restore
 *
 * Caller holds the charger lock.
 */
void tristar_accessory_report(struct tristar_accessory *acc, unsigned int usb_ma,
			      bool mux_fault)
{
	if (!acc)
		return;
	acc->usb_ma = usb_ma;
	acc->mux_fault = mux_fault;
}

/**
 * tristar_accessory_start - start or resume the policy
 * @acc: accessory policy, or NULL
 */
void tristar_accessory_start(struct tristar_accessory *acc)
{
	if (!acc)
		return;
	mutex_lock(acc->lock);
	acc->stopped = false;
	mod_delayed_work(system_power_efficient_wq, &acc->work, 0);
	mutex_unlock(acc->lock);
}

/**
 * tristar_accessory_stop - stop the policy
 * @acc: accessory policy, or NULL
 *
 * Returns once no poll runs; a role change in progress has finished and
 * none starts until tristar_accessory_start(). The port keeps its role.
 * Caller must not hold the charger lock.
 */
void tristar_accessory_stop(struct tristar_accessory *acc)
{
	if (!acc)
		return;
	mutex_lock(acc->lock);
	acc->stopped = true;
	mutex_unlock(acc->lock);
	cancel_delayed_work_sync(&acc->work);
}

/* Unbind: give a host role the policy holds back to the USB device controller. */
static void tristar_accessory_release(void *data)
{
	struct tristar_accessory *acc = data;

	tristar_accessory_stop(acc);
	if (acc->role_sw) {
		if (usb_role_switch_get_role(acc->role_sw) == USB_ROLE_HOST)
			usb_role_switch_set_role(acc->role_sw, USB_ROLE_DEVICE);
		usb_role_switch_put(acc->role_sw);
	}
	of_node_put(acc->role_np);
}

/* SystemInfo model: the root model number, if the boot loader passed one. */
static void tristar_accessory_model(struct tristar_accessory *acc)
{
	struct device_node *root = of_find_node_by_path("/");
	const u8 *prop = NULL;
	int len = 0;

	if (root)
		prop = of_get_property(root, "model-number", &len);
	if (prop && len > 0) {
		acc->model_len = min_t(unsigned int, len, sizeof(acc->model));
		memcpy(acc->model, prop, acc->model_len);
		while (acc->model_len && !acc->model[acc->model_len - 1])
			acc->model_len--;
	}
	of_node_put(root);
	if (!acc->model_len) {
		acc->model_len = strlen(TRISTAR_ACCESSORY_MODEL);
		memcpy(acc->model, TRISTAR_ACCESSORY_MODEL, acc->model_len);
	}
}

/**
 * tristar_accessory_probe - set up the policy if the board has a role switch
 * @dev: the charger
 * @tristar: the Tristar
 * @lock: the charger's lock, which serializes every Tristar access
 * @accp: the policy, or NULL when the device tree names no role switch
 *
 * The policy does nothing until tristar_accessory_start(). It finds the role
 * switch from its own poll, so that charging never waits for the role switch.
 *
 * Return: 0, or -errno.
 */
int tristar_accessory_probe(struct device *dev, struct i2c_client *tristar,
			    struct mutex *lock, struct tristar_accessory **accp)
{
	struct tristar_accessory *acc;
	struct device_node *np;
	bool available;
	int ret;

	np = of_parse_phandle(dev->of_node, "apple,usb-role-switch", 0);
	if (!np)
		return 0;
	available = of_device_is_available(np);
	if (!available || !IS_REACHABLE(CONFIG_USB_ROLES_APPLE_S8000)) {
		of_node_put(np);
		if (available)
			dev_warn(dev, "no Lightning role switch driver; accessories stay unidentified\n");
		return 0;
	}

	acc = devm_kzalloc(dev, sizeof(*acc), GFP_KERNEL);
	if (!acc) {
		of_node_put(np);
		return -ENOMEM;
	}
	acc->dev = dev;
	acc->lock = lock;
	acc->role_np = np;
	acc->ib.client = tristar;
	acc->status = -ENODATA;
	acc->stopped = true;
	tristar_auto_init(&acc->autop);
	tristar_accessory_model(acc);
	INIT_DELAYED_WORK(&acc->work, tristar_accessory_work);
	ret = devm_add_action_or_reset(dev, tristar_accessory_release, acc);
	if (ret)
		return ret;

	*accp = acc;
	dev_info(dev, "Lightning accessory identification and USB host mode enabled\n");
	return 0;
}
