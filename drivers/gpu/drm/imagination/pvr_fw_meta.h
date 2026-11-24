/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright (c) 2023 Imagination Technologies Ltd. */

#ifndef PVR_FW_META_H
#define PVR_FW_META_H

#include <linux/types.h>

/* Forward declaration from pvr_device.h */
struct pvr_device;

struct drm_device;

/**
 * struct pvr_meta_ldr_target - Where the contents of a META LDR image go
 * @mem: For a LOADMEM or ZEROMEM (@zero) command: store in @write_addr where
 *       the @size bytes at META address @addr are kept, or %NULL to leave the
 *       command out. Returns 0 or a negative error for an address the target
 *       has no place for.
 * @config: For each register write of a CONFIG command. Returns 0 or a
 *          negative error. May be %NULL.
 * @start_threads: For a START_THREADS command, with its @count words. Returns
 *                 0 or a negative error. May be %NULL: the command is skipped.
 */
struct pvr_meta_ldr_target {
	int (*mem)(void *data, u32 addr, u32 size, bool zero, void **write_addr);
	int (*config)(void *data, u32 reg, u32 value);
	int (*start_threads)(void *data, const u32 *words, u32 count);
};

int pvr_meta_cr_read32(struct pvr_device *pvr_dev, u32 reg_addr, u32 *reg_value_out);

int pvr_meta_ldr_process(struct drm_device *drm_dev, const u8 *fw, u32 fw_size,
			 const struct pvr_meta_ldr_target *target, void *target_data,
			 u32 **boot_conf_ptr);

#endif /* PVR_FW_META_H */
