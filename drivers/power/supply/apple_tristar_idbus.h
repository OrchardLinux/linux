/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Lightning ID-bus master of the Tristar (CBTL1610) port controller.
 *
 * Copyright The Orchard Linux contributors
 */
#ifndef _APPLE_TRISTAR_IDBUS_H
#define _APPLE_TRISTAR_IDBUS_H

#include <linux/types.h>

struct i2c_client;

#define TRISTAR_IDBUS_ID_LEN		6
/* Longest SystemInfo string: 32 data bytes after opcode, type and length */
#define TRISTAR_IDBUS_INFO_MAX		32

/**
 * struct tristar_idbus - ID-bus master state
 * @client: the Tristar
 * @io_error: first I/O error of the current command sequence; every later
 *	      register access of that sequence is refused
 * @break_pending: a SystemInfo packet timed out; the next transfer starts
 *		   with an ID-bus BREAK
 * @last_ctrl: the control register as last read
 * @last_event: the event register as last read
 *
 * The caller serializes all use of one instance with the other users of the
 * Tristar.
 */
struct tristar_idbus {
	struct i2c_client *client;
	int io_error;
	bool break_pending;
	u8 last_ctrl;
	u8 last_event;
};

int tristar_idbus_status(struct tristar_idbus *ib);
int tristar_idbus_identify(struct tristar_idbus *ib, u8 *id);
int tristar_idbus_accessory_info(struct tristar_idbus *ib,
				 const u8 *model, unsigned int model_len,
				 const u8 *osversion, unsigned int osversion_len);
int tristar_idbus_power_handshake(struct tristar_idbus *ib);

#endif /* _APPLE_TRISTAR_IDBUS_H */
