/* SPDX-License-Identifier: MIT */
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/types.h>
#else
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
#endif
struct als_ops {
	int (*read)(void *ctx, u8 reg, u8 *value);
	int (*write)(void *ctx, u8 reg, u8 value);
	int (*block)(void *ctx, u8 reg, u8 *bytes, unsigned int count);
	int (*delay)(void *ctx, unsigned int us);
	bool (*expired)(void *ctx);
};
struct als_result {
	bool changed, restored, valid;
	u8 original, status;
	u16 channels[4];
	int error, restore_error;
};

static int als_read(const struct als_ops *o, void *ctx, u8 reg, u8 *value)
{
	return o->expired(ctx) ? -ETIMEDOUT : o->read(ctx, reg, value);
}

static int als_write(const struct als_ops *o, void *ctx, u8 reg, u8 value)
{
	return o->expired(ctx) ? -ETIMEDOUT : o->write(ctx, reg, value);
}

static int als_block(const struct als_ops *o, void *ctx, u8 reg, u8 *bytes,
		     unsigned int count)
{
	return o->expired(ctx) ? -ETIMEDOUT : o->block(ctx, reg, bytes, count);
}

static int als_delay(const struct als_ops *o, void *ctx, unsigned int us)
{
	return o->expired(ctx) ? -ETIMEDOUT : o->delay(ctx, us);
}

static int als_once(const struct als_ops *o, void *ctx, struct als_result *r)
{
	u8 v, bytes[8];
	int ret = 0, wr, rd, err;
	bool configured = false, disabled, settings;

	*r = (struct als_result){0};

	ret = als_read(o, ctx, 0x92, &v);

	if (ret)

		goto finish;
	if (v != 0x8f) {
		ret = -ENODEV;
		goto finish;
	}
	ret = als_read(o, ctx, 0x91, &v);
	if (ret)
		goto finish;
	if (v != 0x31) {
		ret = -ENODEV;
		goto finish;
	}
	ret = als_read(o, ctx, 0x80, &r->original);
	if (ret)
		goto finish;
	if (r->original != 0) {
		ret = -EBUSY;
		goto finish;
	}
	ret = als_read(o, ctx, 0x81, &v);
	if (ret)
		goto finish;
	if (v != 0xff) {
		ret = -EPERM;
		goto finish;
	}
	ret = als_read(o, ctx, 0x8f, &v);
	if (ret)
		goto finish;
	if (v != 0) {
		ret = -EPERM;
		goto finish;
	}
	/*
	 * Configure only after exact original admission; any write can be
	 * ambiguous.
	 */
	if (o->expired(ctx)) {
		ret = -ETIMEDOUT;
		goto finish;
	}
	r->changed = true;
	configured = true;
	ret = o->write(ctx, 0x81, 0xf6);
	if (ret)
		goto finish;
	ret = als_read(o, ctx, 0x81, &v);
	if (ret)
		goto finish;
	if (v != 0xf6) {
		ret = -EIO;
		goto finish;
	}
	ret = als_write(o, ctx, 0x8f, 2);
	if (ret)
		goto finish;
	ret = als_read(o, ctx, 0x8f, &v);
	if (ret)
		goto finish;
	if (v != 2) {
		ret = -EIO;
		goto finish;
	}
	/*
	 * A failed write can still have taken effect. Restore on every later
	 * exit.
	 */
	if (o->expired(ctx)) {
		ret = -ETIMEDOUT;
		goto finish;
	}
	r->changed = true;
	ret = o->write(ctx, 0x80, 1);
	if (ret)
		goto finish;
	ret = als_read(o, ctx, 0x80, &v);
	if (ret)
		goto finish;
	if (v != 1) {
		ret = -EIO;
		goto finish;
	}
	ret = als_delay(o, ctx, 5000);
	if (ret)
		goto finish;
	ret = als_write(o, ctx, 0x80, 3);
	if (ret)
		goto finish;
	ret = als_read(o, ctx, 0x80, &v);
	if (ret)
		goto finish;
	if (v != 3) {
		ret = -EIO;
		goto finish;
	}
	for (unsigned int i = 0; i < 20; i++) {
		ret = als_delay(o, ctx, 5000);
		if (ret)
			goto finish;
		ret = als_read(o, ctx, 0x93, &r->status);
		if (ret)
			goto finish;
		if (r->status & 0x80) {
			ret = -ERANGE;
			goto finish;
		}
		if (r->status & 1)
			break;
	}
	if (!(r->status & 1)) {
		ret = -ETIMEDOUT;
		goto finish;
	}
	ret = als_block(o, ctx, 0x94, bytes, 8);
	if (ret)
		goto finish;
	for (unsigned int i = 0; i < 4; i++)
		r->channels[i] = (u16)bytes[2 * i] | (u16)bytes[2 * i + 1] << 8;
	r->valid = true;
finish:
	r->error = ret;
	if (r->changed) {
		/*
		 * Restoration is attempted even after deadline/signal; never
		 * retry enable.
		 */
		wr = o->write(ctx, 0x80, r->original);
		rd = o->read(ctx, 0x80, &v);
		disabled = !rd && v == r->original;
		err = wr ? wr : rd ? rd : disabled ? 0 : -EIO;
		settings = disabled;
		if (disabled && configured) {
			wr = o->write(ctx, 0x81, 0xff);
			rd = o->read(ctx, 0x81, &v);
			settings = settings && !rd && v == 0xff;
			if (!err)
				err = wr ? wr : rd ? rd : v == 0xff ? 0 : -EIO;
			wr = o->write(ctx, 0x8f, 0);
			rd = o->read(ctx, 0x8f, &v);
			settings = settings && !rd && v == 0;
			if (!err)
				err = wr ? wr : rd ? rd : v == 0 ? 0 : -EIO;
			rd = o->read(ctx, 0x80, &v);
			settings = settings && !rd && v == 0;
			if (!err)
				err = rd ? rd : v == 0 ? 0 : -EIO;
		}
		r->restored = disabled && settings;
		r->restore_error = err;
		if (!ret)
			ret = r->restore_error;
	}
	return ret;
}
