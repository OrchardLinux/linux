// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A10 (T8010): the GFX firmware's coprocessor, memory and commands.
 *
 * The firmware runs on "gfx-kf", an AppleA7IOP V2 coprocessor, and speaks RTKit
 * through the A7IOP mailbox. It powers the Rogue core through the "Fender"
 * block; the AP may only touch RGX registers once Fender reports a power state
 * above 0.
 * The firmware maps host memory itself: a 1 MiB window at IOP address 0 holds
 * the image and the init page, and a heap is mapped through memory descriptors
 * (see pvr_apple_layout.c).
 *
 * Messages, AGX endpoints:
 *   0x20  init: 0x81 << 48 | physical address of the init page
 *   0x21  doorbell: 0x83 << 48 makes the firmware scan its rings
 */

#include "pvr_apple.h"
#include "pvr_apple_chip.h"
#include "pvr_apple_layout.h"
#include "pvr_device.h"
#include "pvr_free_list.h"
#include "pvr_hwrt.h"
#include "pvr_job.h"
#include "pvr_stream_defs.h"

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/soc/apple/rtkit.h>

#define APPLE_EP_AGX_INIT		0x20
#define APPLE_EP_AGX_DOORBELL		0x21
#define APPLE_MSG_INIT			(0x81ull << 48)
#define APPLE_MSG_KICK			(0x83ull << 48)

/* Fender beyond the power state: the firmware window, the run bit */
#define FENDER_PERF_STATE_MAX		6	/* T8010's fastest */
#define FENDER_WINDOW			0x200c
#define FENDER_RUN			0x2048
#define FENDER_RUN_ENABLE		BIT(0)

/* A7IOP: the IOP's copy of the firmware window */
#define IOP_WINDOW			0x0008
#define IOP_WINDOW_ENABLE		0x0038

#define APPLE_JOB_SLOTS			16	/* matches the layout's per-job slots */

/* Heap: U+0x14 becomes 1 once the firmware accepted the init data */
#define HEAP_U_READY			(0x5000 + 0x14)

struct pvr_apple_t8010 {
	void __iomem *iop;
	struct apple_rtkit *rtk;

	void *win;
	dma_addr_t win_dma;
	size_t win_next;	/* RTKit buffers are carved after the image */
	dma_addr_t heap_dma;
	bool rtk_idle;		/* RTKit's mailbox stopped while the GPU is off */
};

union pvr_apple_t8010_render_job {
	struct pvr_apple_ta_job ta;
	struct pvr_apple_3d_job frag;
};

/*
 * The firmware's debug flags: 1 SKU trace events, 2 cancel-path events, into
 * a debug ring of 511 records that nothing drains. Once it is full the
 * firmware stops the GPU's work and asks the host to drain it (messages of
 * type 2 on the init endpoint, iOS 10's receivedMessageFromAKF()): only for
 * short experiments. Read at each firmware start, so a change applies from
 * the next reset; a hang report then ends with the ring's last records.
 */
static unsigned int pvr_apple_fw_debug;
module_param_named(apple_fw_debug, pvr_apple_fw_debug, uint, 0644);
MODULE_PARM_DESC(apple_fw_debug, "Apple A10: GFX firmware debug flags (1 trace events, 2 cancel events; default 0)");

static int pvr_apple_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct pvr_apple *apple = cookie;
	struct pvr_apple_t8010 *t = apple->chip_data;
	size_t size = ALIGN(bfr->size, SZ_4K);

	if (bfr->iova) {
		/* Firmware-chosen address: must lie inside the window */
		if (bfr->iova + bfr->size > PVR_APPLE_WINDOW_SIZE)
			return -EINVAL;
		bfr->buffer = t->win + bfr->iova;
		bfr->is_mapped = true;
		return 0;
	}
	if (t->win_next + size > pvr_apple_layout_init_offset())
		return -ENOMEM;
	bfr->iova = t->win_next;
	bfr->buffer = t->win + t->win_next;
	t->win_next += size;
	return 0;
}

static void pvr_apple_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* Carved from the window, which lives as long as the device */
}

static void pvr_apple_recv_message(void *cookie, u8 endpoint, u64 message)
{
	struct pvr_apple *apple = cookie;

	dev_dbg(apple_dev(apple), "firmware ep %#x: %#016llx\n", endpoint, message);
	/* none is known to mean completion, but a check costs little */
	pvr_apple_firmware_event(apple);
}

static void pvr_apple_crashed(void *cookie, const void *crashlog, size_t size)
{
	struct pvr_apple *apple = cookie;

	dev_err(apple_dev(apple), "GFX firmware crashed (%zu byte crashlog)\n", size);
	if (crashlog)
		print_hex_dump(KERN_ERR, "crashlog: ", DUMP_PREFIX_OFFSET, 16, 4, crashlog, size,
			       false);
}

static const struct apple_rtkit_ops pvr_apple_rtkit_ops = {
	.crashed = pvr_apple_crashed,
	.recv_message = pvr_apple_recv_message,
	.shmem_setup = pvr_apple_shmem_setup,
	.shmem_destroy = pvr_apple_shmem_destroy,
};

/* The window is programmed in two copies: Fender (4-byte stride) and IOP (8). */
static void pvr_apple_set_window(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;
	const u32 fields[] = {
		lower_32_bits(t->win_dma), upper_32_bits(t->win_dma),
		0, 0, PVR_APPLE_WINDOW_SIZE, 0,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		writel(fields[i], apple->fender + FENDER_WINDOW + 4 * i);
		writel(fields[i], t->iop + IOP_WINDOW + 8 * i);
	}
	writel(1, t->iop + IOP_WINDOW_ENABLE);
}

static int pvr_apple_t8010_init(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	struct platform_device *pdev = to_platform_device(dev);
	struct pvr_apple_layout *layout = &apple->layout;
	struct pvr_apple_t8010 *t;
	int err;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	apple->chip_data = t;

	t->iop = devm_platform_ioremap_resource_byname(pdev, "iop");
	if (IS_ERR(t->iop))
		return PTR_ERR(t->iop);

	/* The firmware addresses host memory physically through its own MMU. */
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (err)
		return err;
	t->win = dmam_alloc_coherent(dev, PVR_APPLE_WINDOW_SIZE, &t->win_dma, GFP_KERNEL);
	apple->heap = dmam_alloc_coherent(dev, PVR_APPLE_HEAP_SIZE, &t->heap_dma, GFP_KERNEL);
	if (!t->win || !apple->heap)
		return -ENOMEM;
	if (!IS_ALIGNED(t->win_dma, SZ_16K))
		return dev_err_probe(dev, -EINVAL, "firmware window %pad not 16 KiB aligned\n",
				     &t->win_dma);
	t->win_next = PVR_APPLE_FW_RUNTIME;

	/* The firmware writes into its image while it runs: the copy is for a reset */
	err = pvr_apple_fw_load(apple, PVR_APPLE_FW_RUNTIME);
	if (err)
		return err;
	memcpy(t->win, apple->fw_image, apple->fw_size);

	t->rtk = devm_apple_rtkit_init(dev, apple, NULL, 0, &pvr_apple_rtkit_ops);
	if (IS_ERR(t->rtk))
		return dev_err_probe(dev, PTR_ERR(t->rtk), "RTKit init failed\n");

	layout->stamp[APPLE_ENGINE_COMPUTE] = pvr_apple_layout_stamp_offset();
	layout->stamp[APPLE_ENGINE_GEOM] = pvr_apple_layout_ta_stamp_offset();
	layout->stamp[APPLE_ENGINE_FRAG] = pvr_apple_layout_3d_stamp_offset();
	layout->producer[APPLE_ENGINE_COMPUTE] = pvr_apple_layout_compute_producer();
	layout->producer[APPLE_ENGINE_GEOM] = pvr_apple_layout_ta_producer();
	layout->producer[APPLE_ENGINE_FRAG] = pvr_apple_layout_3d_producer();
	layout->frag_ring = pvr_apple_layout_ring_indices_offset(1);
	layout->frag_retired = pvr_apple_layout_3d_retired_offset();
	layout->mmu_root = pvr_apple_layout_mmu_root_offset(0);
	layout->mmu_gen = pvr_apple_layout_mmu_gen_offset(0);
	layout->event_consumer = pvr_apple_layout_event_consumer();
	layout->event_producer = pvr_apple_layout_event_producer();
	layout->events = pvr_apple_layout_event_entry(0);
	layout->event_size = pvr_apple_layout_event_entry(1) - layout->events;
	return 0;
}

static int pvr_apple_t8010_boot(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;
	struct device *dev = apple_dev(apple);
	int err;

	/* The mailbox first: the firmware talks as soon as it runs */
	if (t->rtk_idle) {
		err = apple_rtkit_reinit(t->rtk);
		if (err)
			return err;
		t->rtk_idle = false;
	}
	pvr_apple_set_window(apple);
	writel(readl(apple->fender + FENDER_RUN) | FENDER_RUN_ENABLE, apple->fender + FENDER_RUN);

	err = apple_rtkit_boot(t->rtk);
	if (err) {
		dev_err(dev, "GFX firmware boot failed: %d\n", err);
		goto err_stop;
	}
	err = apple_rtkit_start_ep(t->rtk, APPLE_EP_AGX_INIT);
	if (!err)
		err = apple_rtkit_start_ep(t->rtk, APPLE_EP_AGX_DOORBELL);
	if (err)
		goto err_shutdown;

	err = pvr_apple_fender_power_on(apple);
	if (err)
		goto err_shutdown;
	return 0;

err_shutdown:
	apple_rtkit_shutdown(t->rtk);
err_stop:
	writel(readl(apple->fender + FENDER_RUN) & ~FENDER_RUN_ENABLE, apple->fender + FENDER_RUN);
	return err;
}

/* The window and heap layouts with empty rings: see pvr_apple_layout.c */
static int pvr_apple_t8010_start(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;
	struct device *dev = apple_dev(apple);
	u32 init = pvr_apple_layout_init_offset();
	u32 ready;
	int err;

	memset(t->win + init, 0, PVR_APPLE_WINDOW_SIZE - init);
	memset(apple->heap, 0, PVR_APPLE_HEAP_SIZE);
	pvr_apple_layout_build(t->win, t->win_dma, apple->heap, t->heap_dma,
			       apple->vm_root, APPLE_GPU_PDS, APPLE_GPU_USC);
	/* Geometry and fragment queues (work rings 0 and 1) */
	pvr_apple_layout_build_ta(apple->heap);
	pvr_apple_layout_build_3d(apple->heap);
	*(__le32 *)(apple->heap + pvr_apple_layout_debug_flags_offset()) =
		cpu_to_le32(pvr_apple_fw_debug);
	wmb();

	err = apple_rtkit_send_message(t->rtk, APPLE_EP_AGX_INIT,
				       APPLE_MSG_INIT | (t->win_dma + init), NULL, false);
	if (err)
		return err;
	err = read_poll_timeout(heap_read32, ready, ready == 1, 1000, 3000000, false, apple,
				HEAP_U_READY);
	if (err) {
		dev_err(dev, "GFX firmware did not accept init data\n");
		return err;
	}
	dev_info(dev, "GFX firmware initialized (window %pad, heap %pad)\n", &t->win_dma,
		 &t->heap_dma);
	return 0;
}

/*
 * Shut the firmware down and power the GPU off through state 0. A wedged or
 * crashed firmware may not answer: go on regardless. apple_rtkit_shutdown()
 * resets RTKit's state only once the firmware acknowledged; after a crash it
 * stays "crashed" and refuses to boot, so @recover resets it here.
 */
static void pvr_apple_t8010_stop(struct pvr_apple *apple, bool recover)
{
	struct pvr_apple_t8010 *t = apple->chip_data;

	pvr_apple_request_power_state(apple, 0);
	if (READ_ONCE(apple->stopping))
		apple_rtkit_shutdown_terminal(t->rtk);
	else if (apple_rtkit_shutdown(t->rtk) && recover)
		apple_rtkit_reinit(t->rtk);
	writel(readl(apple->fender + FENDER_RUN) & ~FENDER_RUN_ENABLE,
	       apple->fender + FENDER_RUN);
}

static void pvr_apple_t8010_idle(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;

	/* its mailbox would keep the GPU's power domain on */
	if (READ_ONCE(apple->stopping))
		t->rtk_idle = true;
	else
		t->rtk_idle = !apple_rtkit_mbox_stop(t->rtk);
}

static void pvr_apple_t8010_restore_image(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;

	/* The window as at probe: the image, zeroes after it, RTKit buffers anew */
	memset(t->win, 0, pvr_apple_layout_init_offset());
	memcpy(t->win, apple->fw_image, apple->fw_size);
	t->win_next = PVR_APPLE_FW_RUNTIME;
	wmb();
}

static int pvr_apple_t8010_kick(struct pvr_apple *apple)
{
	struct pvr_apple_t8010 *t = apple->chip_data;

	return apple_rtkit_send_message(t->rtk, APPLE_EP_AGX_DOORBELL, APPLE_MSG_KICK, NULL,
					false);
}

static void pvr_apple_t8010_write_compute(struct pvr_apple *apple, u32 seq, u64 stream,
					  u64 ctx_state)
{
	pvr_apple_layout_write_compute_job(apple->heap, seq, stream, ctx_state);
}

static void pvr_apple_geom_job_init(struct pvr_apple_ta_job *ta, const struct pvr_apple_cmd_geom *cmd,
				    struct pvr_hwrt_data *hwrt)
{
	const struct pvr_hwrt_dataset *set = hwrt->hwrt_dataset;
	const struct rogue_fwif_hwrtdata_common *common = &set->common;
	const struct pvr_apple_geom_regs *apple = &cmd->apple;
	struct pvr_free_list *fl = set->free_lists[0];
	u64 offset;

	memset(ta, 0, sizeof(*ta));
	ta->regs32[0] = apple->r_c78;
	ta->regs32[1] = cmd->cmd.regs.ppp_ctrl;
	ta->regs32[2] = common->screen_pixel_max;		/* 0xc98 PPP_SCREEN */
	ta->regs32[3] = common->te_screen;			/* 0xc18 */
	ta->regs32[4] = common->te_mtile1;			/* 0xc08 */
	ta->regs32[5] = common->te_mtile2;			/* 0xc10 */
	ta->regs32[6] = common->mtile_stride;			/* 0xc20 */
	ta->regs32[7] = cmd->cmd.regs.te_psg;			/* 0xc28 */
	ta->regs64[0] = apple->pds_ctrl;			/* 0xc90 */
	ta->regs64[1] = hwrt->data.rgn_header_dev_addr;		/* 0xc38 */
	ta->regs64[2] = hwrt->data.tail_ptrs_dev_addr;		/* 0xc40 */
	ta->regs64[3] = common->teaa;				/* 0xc00 */
	ta->regs64[4] = common->multi_sample_ctl;		/* 0xc80 */
	ta->regs64[5] = apple->ctx_state_base[0];		/* 0x428 */
	ta->regs64[6] = apple->ctx_state_base[1];		/* 0xcc0 */
	memcpy(&ta->regs64[7], apple->vdm_ctx_store_task, sizeof(apple->vdm_ctx_store_task));
	memcpy(&ta->regs64[10], apple->vdm_ctx_resume_task, sizeof(apple->vdm_ctx_resume_task));
	ta->regs64[13] = apple->vdm_ctx_state;			/* 0x470 */
	ta->regs64[14] = apple->vdm_call_stack;			/* 0x418 */
	ta->regs64[15] = cmd->cmd.regs.vdm_ctrl_stream_base;	/* 0x408 */
	memcpy(&ta->regs64[16], apple->pds_state, sizeof(apple->pds_state));
	memcpy(&ta->regs64[22], apple->cluster, sizeof(apple->cluster));

	mutex_lock(&fl->lock);
	offset = (u64)(fl->max_pages - fl->current_pages) * sizeof(u32);
	ta->free_list = fl->free_list_gpu_addr + (offset & ~15ull);
	ta->free_list_pages = fl->current_pages + (offset & 15) / sizeof(u32);
	mutex_unlock(&fl->lock);
	ta->mlist = hwrt->data.pm_mlist_dev_addr;
	ta->mtile_array = hwrt->data.macrotile_array_dev_addr;
	ta->vheap_table = hwrt->data.vheap_table_dev_addr;
}

/* Start3D's register blocks from the Rogue fragment command and the Apple extension */
static void pvr_apple_frag_job_init(struct pvr_apple_3d_job *frag,
				    const struct pvr_apple_cmd_frag *cmd, struct pvr_hwrt_data *hwrt)
{
	const struct rogue_fwif_frag_regs *regs = &cmd->cmd.regs;
	const struct pvr_apple_frag_regs *apple = &cmd->apple;

	memset(frag, 0, sizeof(*frag));
	frag->regs32[0] = hwrt->hwrt_dataset->common.isp_mtile_size;	/* 0xf18 */
	frag->regs32[1] = regs->isp_bgobjdepth;				/* 0xf80 */
	frag->regs32[2] = regs->isp_bgobjvals;				/* 0xf88 */
	frag->regs32[4] = apple->r_6d0;
	frag->regs32[5] = apple->event_pixel_pds_code;			/* 0x618 */
	frag->regs32[6] = regs->event_pixel_pds_data;			/* 0x620 */
	frag->regs32[7] = regs->event_pixel_pds_info;			/* 0x628 */
	frag->regs32[8] = regs->isp_ctl;				/* 0xf38 */
	frag->regs32[9] = apple->r_3e00;
	frag->regs32[10] = apple->r_fd8;
	frag->regs32[11] = apple->r_4070;
	memcpy(&frag->regs32[12], apple->line, sizeof(apple->line));
	frag->regs64[0] = apple->isp_sample_pos;			/* 0xfa8 */
	frag->regs64[1] = regs->isp_scissor_base;			/* 0xfb0 */
	frag->regs64[2] = regs->isp_dbias_base;				/* 0xfb8 */
	frag->regs64[3] = regs->isp_oclqry_base;			/* 0xfc0, pipe 0 */
	frag->regs64[4] = regs->isp_oclqry_base;			/* 0xfc0, pipe 1 */
	/*
	 * Depth load and store bases, then stencil's (measured: the depth
	 * store reaches memory only with the base in 0xf58)
	 */
	frag->regs64[5] = regs->isp_zlsctl;				/* 0xf48 */
	frag->regs64[6] = regs->isp_zload_store_base;			/* 0xf50 Z load */
	frag->regs64[7] = regs->isp_zload_store_base;			/* 0xf58 Z store */
	frag->regs64[8] = regs->isp_stencil_load_store_base;		/* 0xf60 S load */
	frag->regs64[9] = regs->isp_stencil_load_store_base;		/* 0xf68 S store */
	frag->regs64[10] = apple->isp_rgn;				/* 0xf28 */
	memcpy(&frag->regs64[11], apple->pds_bgnd, sizeof(apple->pds_bgnd));
	frag->regs64[15] = regs->isp_aa;				/* 0xf30 */
	memcpy(&frag->regs64[16], apple->cluster0, sizeof(apple->cluster0));
	memcpy(&frag->regs64[22], apple->cluster1, sizeof(apple->cluster1));
	frag->regs64[28] = apple->r_fd0;
}

static void pvr_apple_t8010_prepare_geom(void *hw, struct pvr_job *job)
{
	union pvr_apple_t8010_render_job *render = hw;

	pvr_apple_geom_job_init(&render->ta, job->cmd, job->hwrt);
}

static void pvr_apple_t8010_prepare_frag(void *hw, struct pvr_job *job)
{
	union pvr_apple_t8010_render_job *render = hw;

	pvr_apple_frag_job_init(&render->frag, job->cmd, job->hwrt);
}

static void pvr_apple_t8010_write_geom(struct pvr_apple *apple, u32 seq, const void *hw)
{
	pvr_apple_layout_write_ta_job(apple->heap, seq, hw);
}

static void pvr_apple_t8010_write_frag(struct pvr_apple *apple, u32 seq, const void *hw)
{
	pvr_apple_layout_write_3d_job(apple->heap, seq, hw);
}

/* With debug flags set (apple_fw_debug): the trace ring's last records */
static void pvr_apple_t8010_report_hang(struct pvr_apple *apple)
{
	if (heap_read32(apple, pvr_apple_layout_debug_flags_offset())) {
		u32 next = heap_read32(apple, pvr_apple_layout_sku_index());
		unsigned int i;

		dev_err(apple_dev(apple), "hang: trace ring index %u\n", next);
		for (i = next >= 24 ? next - 24 : 0; i < next; i++)
			dev_err(apple_dev(apple), "hang: trace %u: %*ph\n", i, 36,
				apple->heap + pvr_apple_layout_sku_record(i));
	}
}

const struct pvr_apple_chip pvr_apple_chip_t8010 = {
	.fw_name = "apple/gfx-t8010.bin",
	.perf_state_max = FENDER_PERF_STATE_MAX,
	.compute_slots = APPLE_JOB_SLOTS,
	.render_job_size = sizeof(union pvr_apple_t8010_render_job),
	.geom_stream = &pvr_apple_cmd_geom_stream,
	.frag_stream = &pvr_apple_cmd_frag_stream,
	.geom_ext_type = PVR_STREAM_EXTHDR_TYPE_GEOM_APPLE,
	.geom_ext_mask = PVR_STREAM_EXTHDR_GEOM_APPLE_REGS,
	.frag_ext_type = PVR_STREAM_EXTHDR_TYPE_FRAG_APPLE,
	.frag_ext_mask = PVR_STREAM_EXTHDR_FRAG_APPLE_REGS,
	.init = pvr_apple_t8010_init,
	.boot = pvr_apple_t8010_boot,
	.start = pvr_apple_t8010_start,
	.stop = pvr_apple_t8010_stop,
	.idle = pvr_apple_t8010_idle,
	.restore_image = pvr_apple_t8010_restore_image,
	.kick = pvr_apple_t8010_kick,
	.write_compute = pvr_apple_t8010_write_compute,
	.prepare_geom = pvr_apple_t8010_prepare_geom,
	.prepare_frag = pvr_apple_t8010_prepare_frag,
	.write_geom = pvr_apple_t8010_write_geom,
	.write_frag = pvr_apple_t8010_write_frag,
	.report_hang = pvr_apple_t8010_report_hang,
};
