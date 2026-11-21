/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple's GPU firmware, between pvr_apple.c and a chip: the state every chip
 * shares, and what a chip provides (struct pvr_apple_chip).
 *
 * The chips run one family of firmware: jobs go into 256-entry work rings,
 * one per engine, and are done when the engine's stamp reaches their sequence
 * number; the firmware posts events to a ring the host drains; the Rogue core
 * is powered through Fender's power-state register. A chip differs in the
 * processor the firmware runs on and how it is booted and rung, and in the
 * layout of the memory it shares with the host, commands included.
 */

#ifndef PVR_APPLE_CHIP_H
#define PVR_APPLE_CHIP_H

#include "pvr_apple.h"
#include "pvr_device.h"

#include <linux/atomic.h>
#include <linux/dma-fence.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct devfreq;
struct drm_gpu_scheduler;
struct pvr_gem_object;
struct pvr_stream_cmd_defs;

/*
 * Fender's power-state request and acknowledge. The power state is also the
 * GPU performance state: 0 is off, 1 to N are the GPU's clock and voltage
 * steps, lowest to highest (the firmware's own DVFS writes the same register,
 * fw 0x16d40; ours leaves it alone). The steps come from the bootloader as
 * the GPU's OPP table, the state number as opp-level; without one the GPU
 * runs in the chip's perf_state_max.
 */
#define FENDER_CTRL			0x0000
#define FENDER_POWER_STATE		0x1000
#define FENDER_POWER_STATE_MASK		0x1f
#define FENDER_POWER_REQUEST		0x10	/* | target state */

/* Driver-owned GPU addresses: code at powervr's PDS and USC heap bases */
#define APPLE_GPU_PDS		PVR_APPLE_PDSCODEDATA_HEAP_BASE
#define APPLE_GPU_USC		PVR_APPLE_USCCODE_HEAP_BASE

/* Entries of a work ring and of the event ring */
#define APPLE_RING_ENTRIES	256

enum pvr_apple_engine {
	APPLE_ENGINE_COMPUTE,
	APPLE_ENGINE_GEOM,
	APPLE_ENGINE_FRAG,
	APPLE_ENGINES
};

/*
 * Where pvr_apple.c reads and writes the memory shared with the firmware:
 * offsets into pvr_apple.heap, set by the chip's init().
 */
struct pvr_apple_layout {
	u32 stamp[APPLE_ENGINES];	/* sequence number of the engine's last finished job */
	u32 producer[APPLE_ENGINES];	/* producer index of the engine's work ring */
	u32 frag_ring;			/* fragment work ring's first index: jobs retired */
	u32 frag_retired;		/* fragment retirement stamp */
	u32 mmu_root;			/* GPU context 0: page-table root, 64 bits */
	u32 mmu_gen;			/* GPU context 0: TLB generation */
	u32 event_consumer;		/* event ring indices */
	u32 event_producer;
	u32 events;			/* event ring entries, event_size bytes each */
	u32 event_size;
};

struct seq_file;

struct pvr_apple {
	struct pvr_device *pvr_dev;
	const struct pvr_apple_chip *chip;
	void *chip_data;	/* the chip's own state, set by its init() */
	void __iomem *fender;

	const void *fw_image;	/* the firmware as loaded (pvr_apple_fw_load()) */
	size_t fw_size;
	void *heap;		/* memory shared with the firmware: rings, stamps, commands */
	struct pvr_apple_layout layout;
	bool stopping;		/* terminal shutdown forbids firmware restart */
	bool booted;
	bool fault_reported;		/* once per firmware boot */
	bool fault;			/* the firmware stopped on a fault in this boot */
	bool fw_ready;		/* init data handed over once: runtime PM restarts it */
	ktime_t gate_off;		/* last successful runtime power-off */

	struct mutex submit_lock;	/* job slots, rings, producer index, GPU VM */
	u32 last_seq;			/* last submitted compute job */
	u32 ta_seq;			/* last submitted geometry job */
	u32 frag_seq;			/* last submitted fragment job */
	u32 frag_events;		/* fragment-done events the firmware posted */
	/*
	 * A geometry job reached the firmware since the last fragment job (or
	 * its start): only then does the next Start3D find a free list
	 */
	bool geom_pending;
	spinlock_t event_lock;		/* event ring consumer, frag_events */
	dma_addr_t vm_root;		/* page-table root GPU context 0 runs on */

	/*
	 * Completion: fences of submitted jobs, one timeline per engine,
	 * signalled from the engine's stamp.
	 */
	u64 fence_context;		/* first of APPLE_ENGINES */
	spinlock_t fence_lock;
	struct list_head pending[3];	/* struct pvr_apple_fence, by seq */
	atomic_t fault_pending;		/* a chip latched a new firmware fault */
	struct delayed_work poll_work;
	struct hrtimer poll_timer;	/* rearms poll_work while jobs are pending */
	/*
	 * The firmware's interrupt, if the chip has one the host can take
	 * (0: none). It is enabled while fences are pending (@fw_irq_on,
	 * under fence_lock) and kicks poll_work; poll_work then checks again
	 * every &pvr_apple_chip.irq_poll_us, and every apple_poll_us until
	 * @fast_until, which a completion the firmware announced sets.
	 */
	int fw_irq;
	bool fw_irq_on;
	ktime_t fast_until;
	/*
	 * The last fragment job pushed: the next geometry job waits for it, as
	 * the firmware keeps one parameter-manager scene at a time.
	 */
	struct dma_fence *last_frag;
	/* The last geometry job pushed: the next fragment job consumes its scene */
	struct dma_fence *last_geom;
	/*
	 * Chips with &pvr_apple_chip.scenes of 2: the fragment job before
	 * @last_frag, and the VM @last_frag runs in.
	 */
	struct dma_fence *prev_frag;
	const struct pvr_vm_context *last_frag_vm;

	/*
	 * Performance state (Fender power state) and its scaling: perf_lock
	 * serializes requests; busy_since (0 while idle) and busy_ns, under
	 * fence_lock, give devfreq the time jobs were in flight.
	 */
	struct mutex perf_lock;
	u32 perf_state;
	u32 perf_state_max;
	struct devfreq *devfreq;
	struct mutex devfreq_lock;	/* serializes our devfreq suspend reference */
	bool devfreq_suspended;
	ktime_t busy_since;
	u64 busy_ns;
	u64 retired_total; /* retired or cancelled jobs, fence_lock */

	ktime_t stat_since;

	/* Backs GPU VA 0 in userspace VMs: the end-of-render writes there */
	struct pvr_gem_object *zero_obj;

	/* Parameter-manager pages every free list hands out (pvr_apple_pm_pages()) */
	struct mutex pm_lock;
	struct pvr_gem_object *pm_pages;

	/*
	 * GPU pages of the kernel VM context: data (stream, terminate PDS,
	 * context state, flush scratch; also mapped into every userspace VM),
	 * PDS and USC code at the code bases.
	 */
	struct pvr_apple_gpu_page {
		struct pvr_gem_object *obj;
		u8 *map;
		u64 device_addr;
		size_t size;
	} data, pds, usc;
};

/**
 * struct pvr_apple_chip - One chip's firmware processor, memory layout and jobs
 * @fw_name: Firmware file, unless the device tree names one (firmware-name).
 * @perf_state_max: Fastest Fender power state, used without an OPP table.
 * @compute_slots: Compute jobs the firmware's memory holds at a time.
 * @render_job_size: Bytes @prepare_geom and @prepare_frag fill.
 * @geom_stream: How a geometry job's command stream from userspace is decoded.
 * @frag_stream: The same for a fragment job.
 * @geom_ext_type: Required geometry register extension header type.
 * @geom_ext_mask: Required bits within that geometry extension header.
 * @frag_ext_type: Required fragment register extension header type.
 * @frag_ext_mask: Required bits within that fragment extension header.
 * @init: At probe, with Fender mapped: map and allocate what the chip needs,
 *	load the firmware (pvr_apple_fw_load()), set &pvr_apple.heap,
 *	&pvr_apple.layout and &pvr_apple.chip_data. Device-managed.
 * @boot: Run the firmware's processor and power the core
 *	(pvr_apple_fender_power_on()); the RGX registers work afterwards. Leaves
 *	everything stopped on an error.
 * @start: Hand the booted firmware fresh init data and wait until it takes
 *	jobs. Every stamp and ring starts empty; GPU context 0 runs on the
 *	kernel VM (&pvr_apple.vm_root).
 * @stop: Power the core off (Fender state 0) and stop the firmware's
 *	processor. With @recover the firmware may have crashed or hung: leave
 *	the processor ready for the next @boot regardless.
 * @idle: After @stop for runtime suspend: give up what would keep the GPU's
 *	power domain on.
 * @fini: When the device goes, after @stop, with the kernel VM still there:
 *	give up what @start made in it. Also after a first @start that failed.
 *	Optional.
 * @restore_image: Before @boot, when the firmware ran before: put its memory
 *	back as @init left it.
 * @kick: Make the firmware look at its work rings.
 * @write_compute: Write compute job @seq (CDM control stream and context
 *	state at GPU addresses) and its work-ring entry.
 * @prepare_geom: Translate a geometry job (its decoded command, HWRT data and
 *	free list) into what @write_geom takes. May sleep; no lock held.
 * @prepare_frag: The same for a fragment job and @write_frag.
 * @write_geom: Write geometry job @seq and its work-ring entry.
 * @write_frag: Write fragment job @seq and its work-ring entry.
 * @report_hang: Log what else the firmware recorded about a job that timed out.
 * @heaps: The userspace heaps (DRM_PVR_HEAP_*), if not powervr's Apple heaps
 *	below 2^38. Optional.
 * @read_gpu_id: Read the GPU's ID (BVNC) the chip's own way, with the core
 *	powered by @boot. Optional: the Rogue registers are read as on any
 *	Rogue core.
 *
 * The functions from @kick to @write_frag are for a firmware that takes one
 * command per engine at a time from a fixed place, named by the work ring's
 * index (the A10's). A chip whose firmware takes jobs differently provides
 * these instead, and pvr_apple.c keeps the fences, the order of the jobs and
 * the power management:
 *
 * @compute_stream: How a compute job's command stream is decoded, if not as
 *	the Rogue command alone.
 * @scenes: Scenes (a geometry job and its fragment job) one VM may have in the
 *	firmware at a time; 0 is 1. Above 1 a fragment job is submitted as soon
 *	as its geometry job was, not when that finished: the firmware orders
 *	the two. Scenes of different VMs never overlap.
 * @own_pm_pages: The chip allocates the parameter manager's pages itself; a
 *	free list has none (pvr_apple_free_list_has_pages()).
 * @gate_off_ms: How long the GPU's power domain stays off before the firmware
 *	is started again, and that a reset takes the domain off as well.
 * @wait_deadline: A wait for the GPU to go idle (pvr_apple_wait_idle()) has
 *	one deadline for all engines, not one per engine, and a wait that
 *	gives up says so, from memory only. A client that closes, or a VM
 *	that goes, then waits at most that one timeout under submit_lock. A
 *	job refused before the chip's own submit path says why.
 * @fault_skips_done: A firmware fault resets through the oldest job its
 *	engine has not finished, not one finished but not yet retired.
 * @submit: Hand @job to the firmware. Takes &pvr_apple.submit_lock itself.
 *	Return: the job's sequence number on its engine, which @engine_done
 *	reaches when it is finished, or a negative error.
 * @engine_done: The sequence number of the last finished job of @engine. No
 *	lock held; may be called with interrupts off.
 * @max_power_state: Highest state licensed for fault capture; optional.
 * @report_fault: Capture the MMU fault state after a firmware fault, in place
 *	of the licence-checked bank read (pvr_apple_report_mmu_fault()); optional.
 * @events: Read what the firmware posted. Before @engine_done wherever a job
 *	is waited for; may sleep.
 * @vm_init: A userspace VM was created: map what the firmware needs in it.
 * @vm_fini: A VM is about to go, its jobs retired: take it from the firmware.
 * @vm_scenes: How many scenes of this VM the firmware may hold now, at most
 *	@scenes. Called in atomic context.
 * @mmu_flush: Page tables of some VM changed. May be called in atomic context.
 *
 * A chip with neither @write_compute nor @submit takes no jobs: its firmware
 * is started and stopped, no context is created on it
 * (pvr_apple_takes_jobs()).
 *
 * pvr_apple.c calls @boot, @start, @stop, @restore_image, @kick and the
 * write functions with &pvr_apple.submit_lock held, except @boot and @start
 * at probe and @stop when the device goes. It publishes the ring's producer index
 * after a write function and before @kick.
 */
struct pvr_apple_chip {
	const char *fw_name;
	u32 perf_state_max;
	u32 compute_slots;
	size_t render_job_size;
	const struct pvr_stream_cmd_defs *geom_stream;
	const struct pvr_stream_cmd_defs *frag_stream;
	u32 geom_ext_type;
	u32 geom_ext_mask;
	u32 frag_ext_type;
	u32 frag_ext_mask;
	const struct pvr_stream_cmd_defs *compute_stream;
	unsigned int scenes;
	bool own_pm_pages;
	unsigned int gate_off_ms;
	bool wait_deadline;
	bool fault_skips_done;
	/* With the firmware's interrupt taken: microseconds between checks while jobs are pending */
	unsigned int irq_poll_us;
	const struct drm_pvr_heap *heaps;
	int (*read_gpu_id)(struct pvr_apple *apple, struct pvr_gpu_id *gpu_id);

	int (*init)(struct pvr_apple *apple);
	int (*boot)(struct pvr_apple *apple);
	int (*start)(struct pvr_apple *apple);
	void (*stop)(struct pvr_apple *apple, bool recover);
	void (*idle)(struct pvr_apple *apple);
	void (*fini)(struct pvr_apple *apple);
	void (*restore_image)(struct pvr_apple *apple);
	int (*kick)(struct pvr_apple *apple);
	void (*write_compute)(struct pvr_apple *apple, u32 seq, u64 stream, u64 ctx_state);
	void (*prepare_geom)(void *hw, struct pvr_job *job);
	void (*prepare_frag)(void *hw, struct pvr_job *job);
	void (*write_geom)(struct pvr_apple *apple, u32 seq, const void *hw);
	void (*write_frag)(struct pvr_apple *apple, u32 seq, const void *hw);
	void (*report_hang)(struct pvr_apple *apple);
	void (*report_fault)(struct pvr_apple *apple);

	int (*prepare_job)(struct pvr_apple *apple, struct pvr_job *job);
	int (*submit)(struct pvr_apple *apple, struct pvr_job *job);
	u32 (*engine_done)(struct pvr_apple *apple, enum pvr_apple_engine engine);
	u32 (*max_power_state)(struct pvr_apple *apple);
	void (*events)(struct pvr_apple *apple);
	int (*vm_init)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	void (*vm_fini)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	bool (*vm_evictable)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	unsigned int (*vm_scenes)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	void (*mmu_flush)(struct pvr_apple *apple);
	/* Acknowledge the firmware's interrupt at its source; hard IRQ context */
	void (*irq_ack)(struct pvr_apple *apple);
	/* The chip's own lines of debugfs apple_jobs */
};

/* A new host tag must be nonzero and differ from the firmware's cached tag. */
static inline u32 pvr_apple_tag_bump(u32 tag, u32 step, u32 cached)
{
	tag += step;
	while (!tag || tag == cached)
		tag += step;
	return tag;
}

/* The heap is coherent memory shared with the firmware, not MMIO. */
static inline u32 heap_read32(struct pvr_apple *apple, u32 offset)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(apple->heap + offset)));
}

static inline struct device *apple_dev(struct pvr_apple *apple)
{
	return from_pvr_device(apple->pvr_dev)->dev;
}

void pvr_apple_report_mmu_fault(struct pvr_apple *apple);
void pvr_apple_firmware_fault(struct pvr_apple *apple);
bool pvr_apple_fence_faulted(struct dma_fence *fence);
void pvr_apple_fence_bind(struct pvr_device *pvr_dev, struct dma_fence *fence,
			  struct drm_gpu_scheduler *sched);
int pvr_apple_fw_load(struct pvr_apple *apple, size_t max_size);
int pvr_apple_wait_idle(struct pvr_apple *apple, unsigned int timeout_us);
int pvr_apple_request_power_state(struct pvr_apple *apple, u32 state);
u32 pvr_apple_start_power_state(struct pvr_apple *apple);
int pvr_apple_fender_power_on(struct pvr_apple *apple);
int pvr_apple_firmware_irq_init(struct pvr_apple *apple, int irq);
void pvr_apple_firmware_event(struct pvr_apple *apple);
void pvr_apple_firmware_progress(struct pvr_apple *apple);

#endif /* PVR_APPLE_CHIP_H */
