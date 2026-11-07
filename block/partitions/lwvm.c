// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Lightweight Volume Manager (LwVM) partition table.
 *
 * Older iOS releases partition their flash with LwVM.  The first 4 KiB of the disk hold
 * the header: a signature, the media UUID and size, a CRC32 of the header, up
 * to eight partition descriptors and a map of 1024 chunks.  The disk is split
 * into 1024 equal power-of-two chunks; each map entry names the partition and
 * the partition-relative chunk stored at that physical chunk.  A copy of the
 * header sits at the end of the first chunk.
 *
 * A partition can be scattered over the disk in any chunk order.  A Linux
 * partition is one contiguous range, so only partitions whose chunks are
 * stored in order and back to back are added; the others are reported and
 * skipped.  A partition keeps its LwVM index as its number, and its UUID and
 * name become PARTUUID and PARTNAME.
 */

#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/uuid.h>

#include "check.h"

#define LWVM_HEADER_SIZE	4096
#define LWVM_CRC_OFFSET		0x2c
#define LWVM_PARTS_OFFSET	0x200
#define LWVM_MAX_PARTS		8
#define LWVM_MAP_OFFSET		0x800
#define LWVM_CHUNKS		1024
#define LWVM_CHUNK_FREE		0xf
#define LWVM_NAME_CHARS		36

struct lwvm_part {
	uuid_t type;
	uuid_t guid;
	__le64 begin;		/* virtual byte range, [begin, end) */
	__le64 end;
	__le64 attributes;
	__le16 name[LWVM_NAME_CHARS];
} __packed;

struct lwvm_header {
	uuid_t signature;
	uuid_t media_uuid;
	__le64 media_size;
	__le32 part_count;
	__le32 crc32;
	u8 pad[LWVM_PARTS_OFFSET - 0x30];
	struct lwvm_part parts[12];
	__le16 map[LWVM_CHUNKS];
} __packed;

/* The CRC-protected variant; the older "noCRC" one is not handled. */
static const uuid_t lwvm_signature =
	UUID_INIT(0x6a9088cf, 0x8afd, 0x630a, 0xe3, 0x51, 0xe2, 0x48, 0x87, 0xe0, 0xb9, 0x8b);

static bool lwvm_read(struct parsed_partitions *state, sector_t first, u8 *buf)
{
	unsigned int i;

	for (i = 0; i < LWVM_HEADER_SIZE / SECTOR_SIZE; i++) {
		Sector sect;
		u8 *data = read_part_sector(state, first + i, &sect);

		if (!data)
			return false;
		memcpy(buf + i * SECTOR_SIZE, data, SECTOR_SIZE);
		put_dev_sector(sect);
	}
	return true;
}

static bool lwvm_valid(struct lwvm_header *h, u64 media)
{
	u32 crc = le32_to_cpu(h->crc32);
	bool ok;

	if (!uuid_equal(&h->signature, &lwvm_signature))
		return false;
	h->crc32 = 0;
	ok = ~crc32_le(~0, (u8 *)h, LWVM_HEADER_SIZE) == crc;
	h->crc32 = cpu_to_le32(crc);
	return ok && le64_to_cpu(h->media_size) == media &&
	       le32_to_cpu(h->part_count) >= 1 &&
	       le32_to_cpu(h->part_count) <= LWVM_MAX_PARTS;
}

static void lwvm_name(struct partition_meta_info *info, const struct lwvm_part *p)
{
	unsigned int i;

	for (i = 0; i < LWVM_NAME_CHARS && i < PARTITION_META_INFO_VOLNAMELTH - 1; i++) {
		u16 c = le16_to_cpu(p->name[i]);

		if (!c)
			break;
		info->volname[i] = c < 0x80 && isprint(c) ? c : '!';
	}
	info->volname[i] = 0;
}

int lwvm_partition(struct parsed_partitions *state)
{
	u64 media = get_capacity(state->disk) << SECTOR_SHIFT;
	u16 first[LWVM_MAX_PARTS], seen[LWVM_MAX_PARTS];
	struct lwvm_header *h;
	unsigned int count, chunks, i;
	u64 chunk;
	int ret = 0;

	BUILD_BUG_ON(sizeof(struct lwvm_part) != 0x80);
	BUILD_BUG_ON(sizeof(struct lwvm_header) != LWVM_HEADER_SIZE);

	/* The chunk is the smallest power of two that splits the disk in 1024. */
	if (media < 2 * LWVM_HEADER_SIZE * LWVM_CHUNKS)
		return 0;
	chunk = 1ULL << fls64(DIV_ROUND_UP_ULL(media, LWVM_CHUNKS) - 1);
	chunks = div64_u64(media, chunk);

	h = kmalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -1;

	if (!lwvm_read(state, 0, (u8 *)h)) {
		ret = -1;
		goto out;
	}
	if (!lwvm_valid(h, media)) {
		if (!uuid_equal(&h->signature, &lwvm_signature))
			goto out;
		pr_warn("%s: LwVM header is damaged, trying its copy\n", state->name);
		if (!lwvm_read(state, (chunk - LWVM_HEADER_SIZE) >> SECTOR_SHIFT, (u8 *)h)) {
			ret = -1;
			goto out;
		}
		if (!lwvm_valid(h, media))
			goto out;
	}

	count = le32_to_cpu(h->part_count);
	memset(first, 0xff, sizeof(first));
	memset(seen, 0, sizeof(seen));

	/* Find each partition's first chunk and count the chunks stored in order
	 * after it; anything else means the partition is scattered. */
	for (i = 0; i < LWVM_CHUNKS; i++) {
		u16 entry = le16_to_cpu(h->map[i]);
		unsigned int owner = entry >> 12, index = entry & 0xfff;

		if (owner == LWVM_CHUNK_FREE)
			continue;
		if (!i || i >= chunks || owner >= count) {
			pr_warn("%s: LwVM chunk %u is invalid\n", state->name, i);
			goto out;
		}
		if (!index)
			first[owner] = i;
		if (first[owner] != 0xffff && i - first[owner] == index &&
		    seen[owner] == index)
			seen[owner]++;
		else
			seen[owner] = 0xffff;
	}

	strlcat(state->pp_buf, " LwVM:", PAGE_SIZE);
	for (i = 0; i < count; i++) {
		const struct lwvm_part *p = &h->parts[i];
		u64 begin = le64_to_cpu(p->begin), end = le64_to_cpu(p->end);
		struct partition_meta_info *info;
		int slot = i + 1;

		if (end <= begin || (begin | end) & (SECTOR_SIZE - 1))
			continue;
		if (seen[i] == 0xffff ||
		    (u64)seen[i] * chunk < end - begin ||
		    (u64)(seen[i] - 1) * chunk >= end - begin) {
			pr_info("%s: LwVM partition %d is not contiguous, skipped\n",
				state->name, slot);
			continue;
		}
		put_partition(state, slot, first[i] * (chunk >> SECTOR_SHIFT),
			      (end - begin) >> SECTOR_SHIFT);
		if (slot >= state->limit)
			continue;
		info = &state->parts[slot].info;
		snprintf(info->uuid, sizeof(info->uuid), "%pUb", &p->guid);
		lwvm_name(info, p);
		state->parts[slot].has_info = true;
	}
	strlcat(state->pp_buf, "\n", PAGE_SIZE);
	ret = 1;
out:
	kfree(h);
	return ret;
}
