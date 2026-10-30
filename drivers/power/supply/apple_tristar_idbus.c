// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lightning ID-bus master of the Tristar (CBTL1610) port controller.
 *
 * A Lightning accessory announces itself on the ID bus: asked for its
 * digital ID, it answers with six bytes that say what it is and which pins
 * it uses. The Tristar clocks the wire, frames BREAKs and appends the
 * request CRC itself; the host drives it over I2C, as iOS does:
 *
 *   1. prepare the engine: 0x60 = 0x01;
 *   2. write the request, reversed, into the TX window that ends at 0x60,
 *      with the RX length byte ((rx_len - 1) << 1) at 0x60 itself;
 *   3. read the window back, and send nothing if it differs;
 *   4. GO: 0x61 = ((tx_len - 1) << 1) | 1;
 *   5. poll the event register 0x0d every 10 ms, for at most 100 ms;
 *   6. read the response forward from the RX window at 0x20: the opcode,
 *      rx_len - 1 data bytes and a CRC over the first rx_len bytes.
 *
 * The RX window is never written: a write there overwrites the retained
 * response. Only the control register 0x05, the pin-fault register 0x0a and
 * the TX window 0x3c-0x61 are written; the mux registers 0x01-0x03 belong
 * to the charger and the role switch.
 *
 * Copyright The Orchard Linux contributors
 */

#include <linux/array_size.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/string.h>

#include "apple_tristar_idbus.h"

#define TRISTAR_IDBUS_REG_DXCTRL	0x01
#define TRISTAR_IDBUS_REG_ACCMUX	0x02
/* bit 0 VBUS sink (the role switch's), bit 2 BREAK, bit 3 ID-bus reset hold */
#define TRISTAR_IDBUS_REG_CTRL		0x05
#define TRISTAR_IDBUS_CTRL_BREAK	0x04
#define TRISTAR_IDBUS_CTRL_RESET	0x08
#define TRISTAR_IDBUS_REG_FAULT		0x0a
#define TRISTAR_IDBUS_REG_STATUS	0x0c
#define TRISTAR_IDBUS_STATUS_VBUS	0x03
#define TRISTAR_IDBUS_REG_EVENT		0x0d
#define TRISTAR_IDBUS_REG_REVISION	0x0f
#define TRISTAR_IDBUS_REG_AIDSW		0x17
#define TRISTAR_IDBUS_REG_RX		0x20
#define TRISTAR_IDBUS_REG_TX_END	0x60
#define TRISTAR_IDBUS_REG_GO		0x61
#define TRISTAR_IDBUS_TX_PREPARE	0x01
/* The revision whose ID-bus engine this file drives */
#define TRISTAR_IDBUS_CBTL1610_REV	0x8a

#define TRISTAR_IDBUS_EV_RESET		0x04
#define TRISTAR_IDBUS_EV_DIGITAL_ID	0x08
#define TRISTAR_IDBUS_EV_DONE		0x10
#define TRISTAR_IDBUS_EV_ERROR		0x20
#define TRISTAR_IDBUS_EV_TIMEOUT	0x40
#define TRISTAR_IDBUS_EV_FAULT		0x80
#define TRISTAR_IDBUS_EV_DONE_MASK	0x38

/* Commands and their responses */
#define TRISTAR_IDBUS_GET_ID		0x74
#define TRISTAR_IDBUS_ID		0x75
#define TRISTAR_IDBUS_HOSTID		0x0002	/* the host's system ID, as iOS sends it */
#define TRISTAR_IDBUS_POWER		0x70
#define TRISTAR_IDBUS_POWER_ACK		0x71
#define TRISTAR_IDBUS_GET_STATE		0x72
#define TRISTAR_IDBUS_STATE		0x73
#define TRISTAR_IDBUS_GET_INTERFACE	0x76
#define TRISTAR_IDBUS_INTERFACE		0x77
#define TRISTAR_IDBUS_GET_MODULE_SN	0x78
#define TRISTAR_IDBUS_MODULE_SN		0x79
#define TRISTAR_IDBUS_GET_ACCESSORY_SN	0x7a
#define TRISTAR_IDBUS_ACCESSORY_SN	0x7b
#define TRISTAR_IDBUS_SYSTEM_INFO	0x84
#define TRISTAR_IDBUS_INFO_MODEL	0
#define TRISTAR_IDBUS_INFO_OSVERSION	1
/* 0x70 flag that iOS sets only when it retries */
#define TRISTAR_IDBUS_POWER_RETRY	0x08

#define TRISTAR_IDBUS_MAX_TX		(TRISTAR_IDBUS_INFO_MAX + 4)
#define TRISTAR_IDBUS_MAX_FRAME		22	/* a serial: opcode, 20 bytes, CRC */
#define TRISTAR_IDBUS_ID_FRAME_LEN	8

#define TRISTAR_IDBUS_POLL_MS		10
#define TRISTAR_IDBUS_POLL_STEPS	10	/* 100 ms per transfer */
#define TRISTAR_IDBUS_SETTLE_MS		20

/* Read before every GetID, in this order; reading 0x0d also clears stale events. */
static const u8 tristar_idbus_baseline[] = {
	TRISTAR_IDBUS_REG_STATUS, TRISTAR_IDBUS_REG_EVENT, TRISTAR_IDBUS_REG_CTRL,
	TRISTAR_IDBUS_REG_DXCTRL, TRISTAR_IDBUS_REG_ACCMUX, TRISTAR_IDBUS_REG_AIDSW,
	TRISTAR_IDBUS_REG_FAULT,
};

/* CRC-8, polynomial 0x31 reflected (0x8c), LSB first, seed 0xff. */
static u8 tristar_idbus_crc8(const u8 *buf, unsigned int len)
{
	unsigned int i;
	u8 crc = 0xff;

	while (len--) {
		crc ^= *buf++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}
	return crc;
}

/*
 * A response is its opcode, rx_len - 1 data bytes and a CRC over the first
 * rx_len bytes. A zero @type accepts any opcode with a good CRC, as iOS does
 * for the SystemInfo replies.
 */
static int tristar_idbus_check_frame(const u8 *frame, unsigned int rx_len, u8 type)
{
	if (type && frame[0] != type)
		return -EPROTO;
	if (tristar_idbus_crc8(frame, rx_len) != frame[rx_len])
		return -EBADMSG;
	return 0;
}

/* ---- I2C access, in the transaction shapes iOS uses ---- */

/* The register selector and STOP, then a separate read. */
static int tristar_idbus_reg_read(struct i2c_client *client, u8 reg)
{
	union i2c_smbus_data data;
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE, NULL);
	if (!ret)
		ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				       I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret ?: data.byte;
}

/*
 * The first I/O error is kept in ib->io_error and refuses every later access
 * of the sequence, so nothing is written after a failure.
 */
static int tristar_idbus_fail(struct tristar_idbus *ib, int err)
{
	if (!ib->io_error)
		ib->io_error = err;
	return err;
}

static int tristar_idbus_read(struct tristar_idbus *ib, u8 reg)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = tristar_idbus_reg_read(ib->client, reg);
	if (ret < 0)
		return tristar_idbus_fail(ib, ret);
	if (reg == TRISTAR_IDBUS_REG_CTRL)
		ib->last_ctrl = ret;
	return ret;
}

static int tristar_idbus_write(struct tristar_idbus *ib, u8 reg, u8 val)
{
	int ret;

	if (ib->io_error)
		return ib->io_error;
	ret = i2c_smbus_write_byte_data(ib->client, reg, val);
	return ret ? tristar_idbus_fail(ib, ret) : 0;
}

/*
 * Read-modify-write that holds the bus throughout: the role switch writes
 * the VBUS-sink bit of 0x05. Returns the value read back afterwards.
 */
static int tristar_idbus_update(struct tristar_idbus *ib, u8 reg, u8 clear, u8 set)
{
	struct i2c_client *client = ib->client;
	union i2c_smbus_data data;
	int ret;

	if (ib->io_error)
		return ib->io_error;
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE, NULL);
	if (!ret)
		ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				       I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (!ret) {
		data.byte = (data.byte & ~clear) | set;
		ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data);
	}
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret)
		return tristar_idbus_fail(ib, ret);
	return tristar_idbus_read(ib, reg);
}

/* The address pointer and STOP, then a separate read of @len bytes. */
static int tristar_idbus_block_read(struct tristar_idbus *ib, u8 reg, u8 *buf,
				    unsigned int len)
{
	struct i2c_client *client = ib->client;
	struct i2c_msg sel = {
		.addr = client->addr,
		.flags = client->flags & I2C_M_TEN,
		.len = 1,
		.buf = &reg,
	};
	struct i2c_msg rx = {
		.addr = client->addr,
		.flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
		.len = len,
		.buf = buf,
	};
	int ret;

	if (ib->io_error)
		return ib->io_error;
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_transfer(client->adapter, &sel, 1);
	if (ret == 1)
		ret = __i2c_transfer(client->adapter, &rx, 1);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret == 1)
		return 0;
	return tristar_idbus_fail(ib, ret < 0 ? ret : -EIO);
}

/* The address and the data in one write message. */
static int tristar_idbus_block_write(struct tristar_idbus *ib, u8 reg, const u8 *buf,
				     unsigned int len)
{
	u8 msg[1 + TRISTAR_IDBUS_MAX_TX + 1];
	int ret;

	if (ib->io_error)
		return ib->io_error;
	if (len > TRISTAR_IDBUS_MAX_TX + 1)
		return -EINVAL;
	msg[0] = reg;
	memcpy(msg + 1, buf, len);
	ret = i2c_master_send(ib->client, msg, len + 1);
	if (ret == (int)len + 1)
		return 0;
	return tristar_idbus_fail(ib, ret < 0 ? ret : -EIO);
}

static int tristar_idbus_read_event(struct tristar_idbus *ib)
{
	int ev = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_EVENT);

	if (ev >= 0)
		ib->last_event = ev;
	return ev;
}

static int tristar_idbus_check_revision(struct tristar_idbus *ib)
{
	int ret = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_REVISION);

	if (ret < 0)
		return ret;
	return ret == TRISTAR_IDBUS_CBTL1610_REV ? 0 : -ENODEV;
}

/* ---- Transfers ---- */

/* A pin fault (0x0d bit 7): acknowledge it by writing back what 0x0a reads. */
static int tristar_idbus_ack_fault(struct tristar_idbus *ib)
{
	int ret = tristar_idbus_update(ib, TRISTAR_IDBUS_REG_FAULT, 0, 0);

	return ret < 0 ? ret : 0;
}

/* 0x05 |= 0x04; the read-back is the 20 us the BREAK needs. */
static int tristar_idbus_send_break(struct tristar_idbus *ib)
{
	int ret = tristar_idbus_update(ib, TRISTAR_IDBUS_REG_CTRL, 0,
				       TRISTAR_IDBUS_CTRL_BREAK);

	return ret < 0 ? ret : 0;
}

/*
 * One command and its response; see the top of this file. Completion is the
 * transfer-done code or, for GetID, the digital-ID event. Returns 0, the
 * response's header or CRC verdict, or -errno.
 */
static int tristar_idbus_transfer(struct tristar_idbus *ib, const u8 *tx,
				  unsigned int tx_len, unsigned int rx_len, u8 rx_type,
				  bool digital_id, u8 *frame)
{
	u8 win[TRISTAR_IDBUS_MAX_TX + 1], back[TRISTAR_IDBUS_MAX_TX + 1];
	u8 base = TRISTAR_IDBUS_REG_TX_END - tx_len, go = ((tx_len - 1) << 1) | 1;
	unsigned int i, step;
	int ev, ret;

	if (!tx_len || tx_len > TRISTAR_IDBUS_MAX_TX || !rx_len ||
	    rx_len >= TRISTAR_IDBUS_MAX_FRAME)
		return -EINVAL;
	for (i = 0; i < tx_len; i++)
		win[i] = tx[tx_len - 1 - i];
	win[tx_len] = (rx_len - 1) << 1;

	if (ib->break_pending) {
		ib->break_pending = false;
		ret = tristar_idbus_send_break(ib);
		if (ret)
			return ret;
	}
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_TX_END, TRISTAR_IDBUS_TX_PREPARE);
	if (!ret)
		ret = tristar_idbus_block_write(ib, base, win, tx_len + 1);
	if (!ret)
		ret = tristar_idbus_block_read(ib, base, back, tx_len + 1);
	if (ret)
		return ret;
	if (memcmp(back, win, tx_len + 1))
		return -EREMOTEIO;
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_GO, go);
	if (ret)
		return ret;

	for (step = 0; ; step++) {
		ev = tristar_idbus_read_event(ib);
		if (ev < 0)
			return ev;
		if (ev & TRISTAR_IDBUS_EV_FAULT) {
			ret = tristar_idbus_ack_fault(ib);
			if (ret)
				return ret;
		}
		if ((digital_id && (ev & TRISTAR_IDBUS_EV_DIGITAL_ID)) ||
		    (ev & TRISTAR_IDBUS_EV_DONE_MASK) == TRISTAR_IDBUS_EV_DONE)
			break;
		if (ev & TRISTAR_IDBUS_EV_ERROR)
			return -EIO;
		if (ev & TRISTAR_IDBUS_EV_RESET)
			return -ECONNRESET;
		if ((ev & TRISTAR_IDBUS_EV_TIMEOUT) || step == TRISTAR_IDBUS_POLL_STEPS)
			return -ETIMEDOUT;
		fsleep(TRISTAR_IDBUS_POLL_MS * USEC_PER_MSEC);
	}

	ret = tristar_idbus_block_read(ib, TRISTAR_IDBUS_REG_RX, frame, rx_len + 1);
	if (ret)
		return ret;
	/* With the digital ID, iOS also reads the status (orientation). */
	if (digital_id) {
		ret = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
		if (ret < 0)
			return ret;
	}
	return tristar_idbus_check_frame(frame, rx_len, rx_type);
}

/**
 * tristar_idbus_status - read the Tristar status register
 * @ib: ID-bus master
 *
 * Bits 0 and 1 are VBUS, bit 6 the plug orientation, bit 7 an attached plug.
 *
 * Return: the register value, -ENODEV for another Tristar revision, or
 * -errno.
 */
int tristar_idbus_status(struct tristar_idbus *ib)
{
	int ret = tristar_idbus_reg_read(ib->client, TRISTAR_IDBUS_REG_REVISION);

	if (ret < 0)
		return ret;
	if (ret != TRISTAR_IDBUS_CBTL1610_REV)
		return -ENODEV;
	return tristar_idbus_reg_read(ib->client, TRISTAR_IDBUS_REG_STATUS);
}

/**
 * tristar_idbus_identify - ask the accessory for its digital ID
 * @ib: ID-bus master
 * @id: TRISTAR_IDBUS_ID_LEN bytes for the ID
 *
 * Acknowledges a latched pin fault and releases a latched ID-bus reset hold
 * first. One GetID, without a reorientation: a reorientation can move a live
 * USB session to the other lanes.
 *
 * Return: 0 with @id filled, or -errno.
 */
int tristar_idbus_identify(struct tristar_idbus *ib, u8 *id)
{
	static const u8 tx[] = {
		TRISTAR_IDBUS_GET_ID, TRISTAR_IDBUS_HOSTID >> 8, TRISTAR_IDBUS_HOSTID & 0xff,
	};
	u8 frame[TRISTAR_IDBUS_ID_FRAME_LEN];
	unsigned int i;
	int ret, val;

	ib->io_error = 0;
	ib->break_pending = false;
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(tristar_idbus_baseline); i++) {
		u8 reg = tristar_idbus_baseline[i];

		val = reg == TRISTAR_IDBUS_REG_EVENT ? tristar_idbus_read_event(ib) :
						       tristar_idbus_read(ib, reg);
		if (val < 0)
			return val;
	}
	/* Then the RX window as the last exchange left it, read only. */
	ret = tristar_idbus_block_read(ib, TRISTAR_IDBUS_REG_RX, frame, sizeof(frame));
	if (ret)
		return ret;
	if (ib->last_event & TRISTAR_IDBUS_EV_FAULT) {
		ret = tristar_idbus_ack_fault(ib);
		if (ret)
			return ret;
	}
	if (ib->last_ctrl & TRISTAR_IDBUS_CTRL_RESET) {
		val = tristar_idbus_update(ib, TRISTAR_IDBUS_REG_CTRL,
					   TRISTAR_IDBUS_CTRL_RESET, 0);
		if (val < 0)
			return val;
		fsleep(TRISTAR_IDBUS_SETTLE_MS * USEC_PER_MSEC);
		if (val & TRISTAR_IDBUS_CTRL_RESET)
			return -EBUSY;
	}

	ret = tristar_idbus_transfer(ib, tx, sizeof(tx), TRISTAR_IDBUS_ID_FRAME_LEN - 1,
				     TRISTAR_IDBUS_ID, true, frame);
	if (!ret)
		memcpy(id, frame + 1, TRISTAR_IDBUS_ID_LEN);
	return ret;
}

/*
 * One SystemInfo packet. The adapter normally does not answer; iOS then
 * advances when the transfer times out. That timeout runs iOS's recovery:
 * the engine is aborted (0x60 = 0x01) and the next transfer starts with a
 * BREAK. Any other failure, or a reply with a bad CRC, is an error.
 */
static int tristar_idbus_system_info(struct tristar_idbus *ib, u8 type,
				     const u8 *data, unsigned int len)
{
	u8 tx[TRISTAR_IDBUS_MAX_TX], reply[2];
	int ret;

	len = min(len, TRISTAR_IDBUS_INFO_MAX);
	tx[0] = TRISTAR_IDBUS_SYSTEM_INFO;
	tx[1] = type;
	tx[2] = len >> 8;
	tx[3] = len;
	memcpy(tx + 4, data, len);
	ret = tristar_idbus_transfer(ib, tx, len + 4, 1, 0, false, reply);
	if (ret != -ETIMEDOUT || ib->io_error)
		return ret;
	ret = tristar_idbus_write(ib, TRISTAR_IDBUS_REG_TX_END, TRISTAR_IDBUS_TX_PREPARE);
	if (!ret)
		ib->break_pending = true;
	return ret;
}

/**
 * tristar_idbus_accessory_info - the exchange iOS runs after the digital ID
 * @ib: ID-bus master
 * @model: the host's model number, sent as SystemInfo type 0
 * @model_len: its length; at most TRISTAR_IDBUS_INFO_MAX bytes are sent
 * @osversion: the host's OS version, sent as SystemInfo type 1
 * @osversion_len: its length; at most TRISTAR_IDBUS_INFO_MAX bytes are sent
 *
 * For the camera adapter iOS reads the interface information (0x76), the
 * module serial (0x78) and the accessory serial (0x7a), then sends the two
 * SystemInfo packets (0x84). The adapter expects this exchange before the
 * power handshake. The serials are not kept.
 *
 * Return: 0, or -errno.
 */
int tristar_idbus_accessory_info(struct tristar_idbus *ib,
				 const u8 *model, unsigned int model_len,
				 const u8 *osversion, unsigned int osversion_len)
{
	static const struct {
		u8 cmd, rx_len, rx_type;
	} reads[] = {
		{ TRISTAR_IDBUS_GET_INTERFACE, 11, TRISTAR_IDBUS_INTERFACE },
		{ TRISTAR_IDBUS_GET_MODULE_SN, 21, TRISTAR_IDBUS_MODULE_SN },
		{ TRISTAR_IDBUS_GET_ACCESSORY_SN, 21, TRISTAR_IDBUS_ACCESSORY_SN },
	};
	u8 frame[TRISTAR_IDBUS_MAX_FRAME];
	unsigned int i;
	int ret;

	ib->io_error = 0;
	ib->break_pending = false;
	ret = tristar_idbus_check_revision(ib);
	for (i = 0; i < ARRAY_SIZE(reads) && !ret; i++)
		ret = tristar_idbus_transfer(ib, &reads[i].cmd, 1, reads[i].rx_len,
					     reads[i].rx_type, false, frame);
	memzero_explicit(frame, sizeof(frame));
	if (!ret)
		ret = tristar_idbus_system_info(ib, TRISTAR_IDBUS_INFO_MODEL,
						model, model_len);
	if (!ret)
		ret = tristar_idbus_system_info(ib, TRISTAR_IDBUS_INFO_OSVERSION,
						osversion, osversion_len);
	/* A pending BREAK is kept for the power handshake that follows. */
	if (ret)
		ib->break_pending = false;
	return ret;
}

/**
 * tristar_idbus_power_handshake - agree on accessory power
 * @ib: ID-bus master
 *
 * With VBUS present: GetState (0x72), answered by 0x73 and four state
 * bytes, then the power request (0x70) echoing the first two state bytes
 * without the retry flag, answered by 0x71. No retries here.
 *
 * Return: 0, -ENODEV without VBUS, -EBUSY with the ID bus held in reset,
 * or -errno.
 */
int tristar_idbus_power_handshake(struct tristar_idbus *ib)
{
	static const u8 get_state = TRISTAR_IDBUS_GET_STATE;
	u8 state[6], request[3], ack[2];
	int ret, val;

	ib->io_error = 0;
	ret = tristar_idbus_check_revision(ib);
	if (ret)
		return ret;
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_STATUS);
	if (val < 0)
		return val;
	if ((val & TRISTAR_IDBUS_STATUS_VBUS) != TRISTAR_IDBUS_STATUS_VBUS)
		return -ENODEV;
	val = tristar_idbus_read(ib, TRISTAR_IDBUS_REG_CTRL);
	if (val < 0)
		return val;
	if (val & TRISTAR_IDBUS_CTRL_RESET)
		return -EBUSY;
	ret = tristar_idbus_transfer(ib, &get_state, 1, 5, TRISTAR_IDBUS_STATE, false, state);
	if (ret)
		return ret;
	request[0] = TRISTAR_IDBUS_POWER;
	request[1] = state[1] & ~TRISTAR_IDBUS_POWER_RETRY;
	request[2] = state[2];
	return tristar_idbus_transfer(ib, request, sizeof(request), 1,
				      TRISTAR_IDBUS_POWER_ACK, false, ack);
}
