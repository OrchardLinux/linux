/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTD_APPLE_H2FMI_H
#define MTD_APPLE_H2FMI_H

#include <linux/bits.h>
#include <linux/math.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define H2FMI_PARAMS_SIZE	512
#define H2FMI_CHUNKS		4
#define H2FMI_CHUNK_LO		1024
#define H2FMI_CHUNK_META		16
#define H2FMI_CHUNK_HI		3072
#define H2FMI_CHUNK_RAW		4112
#define H2FMI_PAGE_RAW		16448
#define H2FMI_PAGE_DATA		16384
#define H2FMI_PAGE_OOB		64
#define H2FMI_PAGE_SHIFT		14

#define PPN_ST_GOOD		0x40
#define PPN_ST_REFRESH		0x42
#define PPN_ST_RETIRED		0x45
#define PPN_ST_ERASED		0x49

struct h2fmi_geometry {
	u32 caus;
	u32 blocks;
	u32 pages;
	u32 page_bits;
	u32 block_bits;
	u32 cau_bits;
	u32 slc_pages;
};

/* Restrict discovery to the two measured PPN 1.5 package profiles. The
 * parameter response does not identify a PPN protocol revision. The existing
 * platform compatible continues to select that protocol explicitly.
 */
static inline int h2fmi_parse_geometry(const u8 *params, size_t len,
				       struct h2fmi_geometry *geometry)
{
	struct h2fmi_geometry g;
	u32 cell_bits, cells, raw_bytes;

	if (len != H2FMI_PARAMS_SIZE ||
	    memcmp(params, "PPN Device Info", sizeof("PPN Device Info") - 1))
		return -EINVAL;

	g.caus = get_unaligned_le32(params + 0x10);
	g.cau_bits = get_unaligned_le32(params + 0x14);
	g.blocks = get_unaligned_le32(params + 0x18);
	g.block_bits = get_unaligned_le32(params + 0x1c);
	g.pages = get_unaligned_le32(params + 0x20);
	g.slc_pages = get_unaligned_le32(params + 0x24);
	g.page_bits = get_unaligned_le32(params + 0x28);
	cell_bits = get_unaligned_le32(params + 0x2c);
	cells = get_unaligned_le32(params + 0x30);
	raw_bytes = get_unaligned_le32(params + 0x34);
	if (raw_bytes != H2FMI_PAGE_RAW || cell_bits != 1 || cells != 2)
		return -EOPNOTSUPP;
	if (g.caus == 2 && g.cau_bits == 4 && g.blocks == 1064 &&
	    g.block_bits == 11 && g.pages == 256 && g.slc_pages == 128 &&
	    g.page_bits == 8) {
		*geometry = g;
		return 0;
	}
	if (g.caus == 8 && g.cau_bits == 3 && g.blocks == 2088 &&
	    g.block_bits == 12 && g.pages == 128 && g.slc_pages == 64 &&
	    g.page_bits == 7) {
		*geometry = g;
		return 0;
	}
	return -EOPNOTSUPP;
}

static inline int h2fmi_page_row(const struct h2fmi_geometry *g,
				 unsigned int cau, unsigned int block,
				 unsigned int page, u32 *row)
{
	if (cau >= g->caus || block >= g->blocks || page >= g->pages)
		return -EINVAL;
	*row = page | block << g->page_bits |
	       cau << (g->page_bits + g->block_bits);
	return 0;
}

/* SLC-view bootstrap members may occupy any of the last 32 blocks. */
#define H2FMI_BOOTSTRAP_BLOCKS	32

/*
 * A single-row erase clears one block of one CAU: the row of its first
 * normal-view page, and the pages behind it. Both follow from the geometry
 * the package reports, so erase is offered only when that geometry is
 * self-consistent: the page, block and CAU fields fit their address bits,
 * the SLC view aliases half of the normal pages, and there are blocks below
 * the bootstrap window.
 */
static inline bool h2fmi_erase_geometry_valid(const struct h2fmi_geometry *g)
{
	return g->page_bits < 32 && g->block_bits < 32 && g->cau_bits < 32 &&
	       g->page_bits + g->block_bits + g->cau_bits < 32 &&
	       g->pages && g->pages == BIT(g->page_bits) &&
	       g->slc_pages * 2 == g->pages &&
	       g->blocks > H2FMI_BOOTSTRAP_BLOCKS + 1 &&
	       g->blocks <= BIT(g->block_bits) &&
	       g->caus && g->caus <= BIT(g->cau_bits);
}

static inline int h2fmi_erase_row(const struct h2fmi_geometry *g,
				 unsigned int cau, unsigned int block, u32 *row)
{
	if (!h2fmi_erase_geometry_valid(g))
		return -EOPNOTSUPP;
	if (cau >= g->caus || block >= g->blocks)
		return -EINVAL;
	if (!block || block >= g->blocks - H2FMI_BOOTSTRAP_BLOCKS)
		return -EPERM;
	return h2fmi_page_row(g, cau, block, 0, row);
}

/* The VFL bootstrap's packed requests set the bit above the CAU field.
 * Keep this view separate from normal MTD offsets. Its address encoding
 * follows the saved firmware; live context selection is a separate step.
 */
static inline int h2fmi_slc_row(const struct h2fmi_geometry *g,
				unsigned int cau, unsigned int block,
				unsigned int page, u32 *row)
{
	u32 value;
	int ret;

	if (page >= g->slc_pages)
		return -EINVAL;
	ret = h2fmi_page_row(g, cau, block, page, &value);
	if (ret)
		return ret;
	*row = value | 1u << (g->page_bits + g->block_bits + g->cau_bits);
	return 0;
}

/* Unknown statuses must never lead to a data-out command or a good block. */
static inline int h2fmi_page_status(u8 status)
{
	switch (status) {
	case PPN_ST_GOOD:
	case PPN_ST_ERASED:
		return 0;
	case PPN_ST_REFRESH:
		return 1;
	default:
		return -EBADMSG;
	}
}

static inline bool h2fmi_raw_page_erased(u8 status, const u8 *raw)
{
	return status == PPN_ST_ERASED && !memchr_inv(raw, 0xff, H2FMI_PAGE_RAW);
}

/*
 * SDR operation, as Cascadia's PIO driver runs the A5 controller after its own
 * NAND reset: the package is held at the low-power interface timing until
 * feature 0x180 selects the asynchronous interface, then runs at the working
 * timing measured on the P105 (dc 2, rs 3, rh 2, ws 1, wh 0).
 */
#define H2FMI_IF_CTRL_LOW_POWER	0xffff
#define H2FMI_IF_CTRL_SDR	0x23210
#define H2FMI_FEATURE_POWER	0x0180
#define H2FMI_POWER_ASYNC	2
#define H2FMI_FEATURE_FIRMWARE	0x9080
#define H2FMI_FIRMWARE_SIZE	16
#define H2FMI_FEATURE_BAD_BLOCKS	0x84
#define H2FMI_ID_SIZE		8
#define H2FMI_SDR_SECTOR	1028	/* a page unit moves as four sectors */

/* Read ID answers "PPN" and the protocol version; both packages are PPN 1.5. */
static inline bool h2fmi_ppn_id(const u8 *id)
{
	return !memcmp(id, "PPN", 3) && id[3] == 1 && id[4] == 5;
}

/* Feature 0x84 | cau << 8 returns one bit per block of that CAU. */
static inline u32 h2fmi_bad_block_bytes(const struct h2fmi_geometry *g)
{
	return DIV_ROUND_UP(g->blocks, 8);
}

static inline u16 h2fmi_bad_block_feature(unsigned int cau)
{
	return H2FMI_FEATURE_BAD_BLOCKS | cau << 8;
}

/* An SDR page read sends the full address: u16 byte count, u16 column, row. */
static inline void h2fmi_full_address(u32 row, u32 *addr0, u32 *addr1)
{
	*addr0 = H2FMI_PAGE_RAW;
	*addr1 = row;
}

/* CMD 79 reports ready at bit 6 and program failure at bits 0 and 4. */
static inline int h2fmi_program_status(u8 status)
{
	return (status & BIT(6)) && !(status & (BIT(0) | BIT(4))) ? 0 : -EIO;
}

#endif
