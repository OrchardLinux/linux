/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * iPad Air (J72) N1 report framing, contact layout and recovery budget for
 * apple_z2.c, kept as pure functions so they can be tested on their own.
 */
#ifndef _APPLE_Z2_J72_H
#define _APPLE_Z2_J72_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define APPLE_Z2_J72_MAX_CONTACTS 10
#define APPLE_Z2_J72_MAX_PAYLOAD 2454
#define APPLE_Z2_J72_RECOVERIES 6
#define APPLE_Z2_J72_RECOVERY_MS 60000ULL

struct apple_z2_j72_contact {
	u8 id;
	u8 state;
	s16 x;
	s16 y;
};

struct apple_z2_j72_recovery {
	u64 times[APPLE_Z2_J72_RECOVERIES];
	unsigned int count;
	unsigned int next;
};

static inline bool apple_z2_j72_checksum(const u8 *p, size_t len)
{
	u16 sum = 0;
	size_t i;

	if (len < 2)
		return false;
	for (i = 0; i < len - 2; i++)
		sum += p[i];
	return sum == get_unaligned_le16(p + len - 2);
}

/* A valid E-class control reply is not necessarily an EB length reply. */
static inline int apple_z2_j72_length(const u8 *p, u16 *length)
{
	unsigned int i;
	bool zero = true;

	for (i = 0; i < 14; i++)
		zero &= !p[i];
	if (zero)
		return -EAGAIN;
	if ((p[0] & 0xf0) != 0xe0 || !apple_z2_j72_checksum(p, 16))
		return -EBADMSG;
	if (p[0] != 0xe1)
		return -EAGAIN;
	*length = get_unaligned_le16(p + 1);
	if (*length > APPLE_Z2_J72_MAX_PAYLOAD || (*length && *length < 2))
		return -EMSGSIZE;
	return 0;
}

static inline bool apple_z2_j72_packet(const u8 *p, size_t len)
{
	unsigned int header = 0;
	unsigned int i;
	u16 inner;

	if (len < 7 || len > APPLE_Z2_J72_MAX_PAYLOAD + 5 ||
	    (p[0] != 0xea && p[0] != 0xeb))
		return false;
	for (i = 0; i < 5; i++)
		header += p[i];
	inner = get_unaligned_le16(p + 2);
	return !(header & 0xff) && inner >= 2 && (size_t)inner + 5 == len &&
	       apple_z2_j72_checksum(p + 5, inner);
}

/* Validate the whole native report before the caller changes any input slot. */
static inline int apple_z2_j72_contacts(const u8 *p, size_t len,
				      struct apple_z2_j72_contact *out,
				      unsigned int *count)
{
	unsigned int n, i, j;
	const u8 *contact;

	if (len < 24 || p[0] != 0x44)
		return -EPROTO;
	if (p[2] != 24 || get_unaligned_le16(p + 14) || p[17] != 30)
		return -EPROTO;
	n = p[16];
	if (n > APPLE_Z2_J72_MAX_CONTACTS || n > (len - 24) / 30)
		return -EMSGSIZE;
	for (i = 0; i < n; i++) {
		contact = p + 24 + i * 30;
		switch (contact[1]) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 7:
			break;
		default:
			return -EPROTO;
		}
		for (j = 0; j < i; j++)
			if (out[j].id == contact[0])
				return -EPROTO;
		out[i].id = contact[0];
		out[i].state = contact[1];
		out[i].x = (s16)get_unaligned_le16(contact + 4);
		out[i].y = (s16)get_unaligned_le16(contact + 6);
	}
	*count = n;
	return 0;
}

static inline bool apple_z2_j72_recovery_admit(struct apple_z2_j72_recovery *r,
					     u64 now)
{
	unsigned int last;

	if (r->count > APPLE_Z2_J72_RECOVERIES ||
	    r->next >= APPLE_Z2_J72_RECOVERIES)
		return false;
	last = (r->next + APPLE_Z2_J72_RECOVERIES - 1) % APPLE_Z2_J72_RECOVERIES;
	if (r->count && now < r->times[last])
		return false;
	if (r->count == APPLE_Z2_J72_RECOVERIES &&
	    now - r->times[r->next] < APPLE_Z2_J72_RECOVERY_MS)
		return false;
	r->times[r->next] = now;
	r->next = (r->next + 1) % APPLE_Z2_J72_RECOVERIES;
	if (r->count < APPLE_Z2_J72_RECOVERIES)
		r->count++;
	return true;
}

#endif
