/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Apple chips: Rogue hardware behind Apple's GFX firmware. */

#ifndef PVR_APPLE_H
#define PVR_APPLE_H

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <uapi/drm/pvr_drm.h>

#include "pvr_rogue_fwif_client.h"

struct dma_fence;
struct pvr_apple_chip;
struct pvr_context;
struct pvr_device;
struct pvr_job;
struct pvr_queue;
struct pvr_power_sequence_ops;
struct pvr_stream_cmd_defs;
struct pvr_vm_context;

extern const struct pvr_power_sequence_ops pvr_power_sequence_ops_apple;
extern const struct pvr_apple_chip pvr_apple_chip_s8003;
extern const struct pvr_apple_chip pvr_apple_chip_t8010;

/*
 * The A10's GPU resolves 38-bit data addresses (a triangle renders with its
 * buffers at 0x20_xxxx_xxxx, not at 0x40_xxxx_xxxx), below powervr's Rogue
 * heaps; Apple devices get the same heaps below 2^38. The firmware's PDS and
 * USC code bases are the code heaps' bases.
 */
#define PVR_APPLE_GENERAL_HEAP_BASE		0x0800000000ull
#define PVR_APPLE_PDSCODEDATA_HEAP_BASE		0x2a00000000ull
#define PVR_APPLE_USCCODE_HEAP_BASE		0x2c00000000ull
#define PVR_APPLE_RGNHDR_HEAP_BASE		0x2ef0000000ull
#define PVR_APPLE_VISTEST_HEAP_BASE		0x2f00000000ull
#define PVR_APPLE_TRANSFER_FRAG_HEAP_BASE	0x3000000000ull

/*
 * Geometry registers of Apple's StartTA that the Rogue command, the HWRT
 * dataset and the free list do not determine, as raw register values
 * (stream extension PVR_STREAM_EXTHDR_TYPE_GEOM_APPLE).
 */
struct pvr_apple_geom_regs {
	u64 pds_ctrl;			/* 0xc90 */
	u64 ctx_state_base[2];		/* 0x428, 0xcc0 */
	u64 vdm_ctx_store_task[3];	/* 0x438, 0x440, 0x448 */
	u64 vdm_ctx_resume_task[3];	/* 0x450, 0x458, 0x460 */
	u64 vdm_ctx_state;		/* 0x470, PDS-relative */
	u64 vdm_call_stack;		/* 0x418 */
	u64 pds_state[6];		/* 0xf058, 0xf060, 0xf068, 0xf070, 0xf038, 0xf040 */
	u64 cluster[6];			/* 0x11030, per cluster */
	u32 r_c78;			/* 0xc78 */
	u32 padding;
};

struct pvr_apple_cmd_geom {
	struct rogue_fwif_cmd_geom cmd;
	struct pvr_apple_geom_regs apple;
};

/*
 * Fragment registers of Apple's Start3D beyond the Rogue command
 * (PVR_STREAM_EXTHDR_TYPE_FRAG_APPLE). The background program takes four
 * registers (PDS_BGRND0..3) where the Rogue command has three.
 */
struct pvr_apple_frag_regs {
	u64 isp_sample_pos;		/* 0xfa8 */
	u64 pds_bgnd[4];		/* 0x6a0, 0x6a8, 0x6b0, 0x6b8 */
	u64 cluster0[6];		/* 0x11020, per cluster */
	u64 cluster1[6];		/* 0x11080, per cluster */
	u64 r_fd0;			/* 0xfd0 */
	u32 event_pixel_pds_code;	/* 0x618 */
	u32 isp_rgn;			/* 0xf28 */
	u32 r_6d0;			/* 0x6d0 */
	u32 r_3e00;			/* 0x3e00 */
	u32 r_fd8;			/* 0xfd8 */
	u32 r_4070;			/* 0x4070 */
	u32 line[6];			/* 0x11d0 ... 0x11f8: line rasterization */
};

struct pvr_apple_cmd_frag {
	struct rogue_fwif_cmd_frag cmd;
	struct pvr_apple_frag_regs apple;
};

/*
 * Apple A9 (G5P). Its firmware takes a job as a work record: the kernel
 * builds the record from the Rogue command, the HWRT dataset and these
 * stream extensions, which carry the record's words that userspace owns.
 * Addresses are GPU addresses in the job's VM; "PDS words" are the firmware's
 * program descriptors as the A9 Vulkan driver builds them, code addresses in
 * them relative to the PDS heap's base.
 *
 * Geometry (PVR_STREAM_EXTHDR_TYPE_GEOM_G5P). Of the Rogue command only
 * regs.vdm_ctrl_stream_base is read.
 */
struct pvr_apple_g5p_geom_regs {
	u64 scratch_base;		/* TA scratch block, 8-byte aligned */
	u64 scratch_ptr[6];		/* TA scratch and marker pointers */
	u64 ta_program;			/* TA state PDS program: byte offset from the PDS heap's base */
	u64 sample_positions;		/* x | y << 4 per sample, 1/16 pixel; 0: the standard pattern */
	u32 context_pds[2][6];		/* context store and load PDS words */
	u32 resume_pds[2][4];		/* resume PDS words */
	u32 ta_pds[4];			/* TA state PDS words; word 3 is zero */
};

struct pvr_apple_g5p_cmd_geom {
	struct rogue_fwif_cmd_geom cmd;
	struct pvr_apple_g5p_geom_regs g5p;
};

/*
 * Fragment (PVR_STREAM_EXTHDR_TYPE_FRAG_G5P). Of the Rogue command
 * regs.isp_scissor_base, isp_dbias_base, isp_oclqry_base, isp_bgobjdepth and
 * isp_bgobjvals are read.
 */
#define PVR_APPLE_G5P_FRAG_MSAA_RESOLVE	BIT(0)	/* four samples only */

struct pvr_apple_g5p_frag_regs {
	u64 background_texture;		/* PDS_BGRND1_BASE: the background object's texture state */
	u64 zls[6];			/* register images 0xf48, 0xf50, 0xf58, 0xf60, 0xf68, 0xfd0 */
	u64 sample_positions;		/* as the geometry job's */
	u32 background[8];		/* background (load or clear) object */
	u32 eot[4];			/* end-of-tile object; word 3 is zero */
	u32 eot_resume[4];		/* partial-render end of tile; word 3 is zero */
	u32 restore[4];			/* partial-render restore words */
	u32 pixel_output_width;		/* 2, 4 or 8 */
	u32 flags;			/* PVR_APPLE_G5P_FRAG_* */
};

struct pvr_apple_g5p_cmd_frag {
	struct rogue_fwif_cmd_frag cmd;
	struct pvr_apple_g5p_frag_regs g5p;
};

/*
 * Compute (PVR_STREAM_EXTHDR_TYPE_COMPUTE_G5P). Of the Rogue command only
 * regs.cdm_ctrl_stream_base is read.
 */
struct pvr_apple_g5p_compute_regs {
	u64 state[4];			/* context state buffers */
	u64 tail;			/* 16-byte aligned */
	u64 terminate;			/* address of the control stream's terminate word */
	u64 pds_store;			/* PDS code | data << 32 */
	u64 pds_resume;
	u64 pds_terminate;
	u32 config[2];
	u32 pds_store1;
	u32 pds_resume1;
};

struct pvr_apple_g5p_cmd_compute {
	struct rogue_fwif_cmd_compute cmd;
	struct pvr_apple_g5p_compute_regs g5p;
};

#if IS_ENABLED(CONFIG_DRM_POWERVR_APPLE)
int pvr_apple_fw_init(struct pvr_device *pvr_dev);
void pvr_apple_fw_fini(struct pvr_device *pvr_dev);
struct pvr_gem_object *pvr_apple_pm_pages(struct pvr_device *pvr_dev, size_t size);
int pvr_apple_reset(struct pvr_device *pvr_dev);
int pvr_apple_suspend(struct pvr_device *pvr_dev);
int pvr_apple_resume(struct pvr_device *pvr_dev);
void pvr_apple_device_info_init(struct pvr_device *pvr_dev);
bool pvr_apple_takes_jobs(struct pvr_device *pvr_dev);
bool pvr_apple_free_list_has_pages(struct pvr_device *pvr_dev);
struct dma_fence *pvr_apple_submit_job(struct pvr_job *job);
struct dma_fence *pvr_apple_submit_compute_job(struct pvr_device *pvr_dev,
					       struct pvr_vm_context *vm_ctx, u64 stream,
					       u64 ctx_state);
struct dma_fence *pvr_apple_submit_render_job(struct pvr_job *job);
void pvr_apple_report_hang(struct pvr_device *pvr_dev, struct dma_fence *fence);
void pvr_apple_wait_file_jobs(struct pvr_device *pvr_dev);
int pvr_apple_job_add_deps(struct pvr_job *job);
void pvr_apple_job_armed(struct pvr_job *job, struct dma_fence *finished);
int pvr_apple_vm_init(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx);
void pvr_apple_mmu_flush(struct pvr_device *pvr_dev);
void pvr_apple_vm_fini(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx);
void pvr_apple_quiesce(struct pvr_device *pvr_dev);
const struct pvr_stream_cmd_defs *pvr_apple_cmd_stream(struct pvr_device *pvr_dev,
						       enum drm_pvr_job_type type);

struct pvr_queue *pvr_apple_queue_create(struct pvr_context *ctx, enum drm_pvr_job_type type);
void pvr_apple_queue_destroy(struct pvr_queue *queue, bool cleanup_queue_entity);
int pvr_apple_queue_job_init(struct pvr_job *job, u64 drm_client_id);
void pvr_apple_queue_job_cleanup(struct pvr_job *job);
#else
static inline int pvr_apple_fw_init(struct pvr_device *pvr_dev) { return -ENODEV; }
static inline void pvr_apple_fw_fini(struct pvr_device *pvr_dev) {}
static inline struct pvr_gem_object *pvr_apple_pm_pages(struct pvr_device *pvr_dev, size_t size)
{
	return ERR_PTR(-E2BIG);
}
static inline int pvr_apple_reset(struct pvr_device *pvr_dev) { return -ENODEV; }
static inline int pvr_apple_suspend(struct pvr_device *pvr_dev) { return 0; }
static inline int pvr_apple_resume(struct pvr_device *pvr_dev) { return 0; }
static inline void pvr_apple_device_info_init(struct pvr_device *pvr_dev) {}
static inline bool pvr_apple_takes_jobs(struct pvr_device *pvr_dev) { return false; }
static inline bool pvr_apple_free_list_has_pages(struct pvr_device *pvr_dev) { return true; }
static inline int pvr_apple_vm_init(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx)
{
	return -ENODEV;
}
static inline struct pvr_queue *pvr_apple_queue_create(struct pvr_context *ctx,
							enum drm_pvr_job_type type)
{
	return ERR_PTR(-ENODEV);
}
static inline void pvr_apple_queue_destroy(struct pvr_queue *queue, bool cleanup_queue_entity) {}
static inline int pvr_apple_queue_job_init(struct pvr_job *job, u64 drm_client_id)
{
	return -ENODEV;
}
static inline void pvr_apple_queue_job_cleanup(struct pvr_job *job) {}
static inline int pvr_apple_job_add_deps(struct pvr_job *job) { return 0; }
static inline void pvr_apple_job_armed(struct pvr_job *job, struct dma_fence *finished) {}
static inline void pvr_apple_mmu_flush(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_quiesce(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_wait_file_jobs(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_report_hang(struct pvr_device *pvr_dev, struct dma_fence *fence) {}
static inline void pvr_apple_vm_fini(struct pvr_device *pvr_dev,
				     struct pvr_vm_context *vm_ctx) {}
static inline const struct pvr_stream_cmd_defs *
pvr_apple_cmd_stream(struct pvr_device *pvr_dev, enum drm_pvr_job_type type)
{
	return NULL;
}
#endif

#endif /* PVR_APPLE_H */
