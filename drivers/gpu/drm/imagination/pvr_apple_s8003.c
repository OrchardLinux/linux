// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A9 (S8003): the GFX firmware on the Rogue core's own META processor.
 *
 * The A9 has no coprocessor for its GPU firmware: Apple's firmware is a META
 * LDR image, as Imagination's is, and runs on the META inside the GT7600. The
 * host does what Apple's kext does:
 *
 *  - unpack the image into one block of GPU memory, with the bootloader's
 *    configuration (the segment MMU and the image's own register writes) at
 *    its fixed place in it;
 *  - map that block, the memory shared with the firmware ("root" and
 *    "shared": its init data, rings and control words) and the registers the
 *    firmware uses (Fender, the interrupt controller) at the firmware's
 *    addresses, in the kernel VM context: the firmware's page tables are the
 *    kernel VM's, and the image and the shared memory lie where powervr puts
 *    IMG's firmware heap;
 *  - store the Rogue registers of a META boot (reset, boot mode, page-table
 *    root, cache setup, reset release) in Fender's boot stream: Fender
 *    replays it into the core when the core is powered up;
 *  - power the core up and wait until the firmware posts its first event.
 *
 * The META slave port is not used: the firmware boots itself from the memory
 * mapped for it.
 *
 * Jobs are written as this firmware takes them, which is not the A10's way:
 * see "Jobs" below. The firmware raises its interrupt to the host through
 * the interrupt controller's software-set register, which that controller's
 * driver never clears: the host clears it itself before it reads the rings,
 * as Apple's kext does. It is taken while jobs are pending; otherwise the
 * rings are read on a timer.
 *
 * The boot sequence, the layout of the shared memory, the job records and
 * every value in them are from Berkay Kösebay's G5P driver, which runs the
 * GPU on the iPad 5 (J71t); the names of the firmware's objects are his
 * readings of Apple's kext and firmware.
 */

#include "pvr_apple.h"
#include "pvr_apple_chip.h"
#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_free_list.h"
#include "pvr_fw_meta.h"
#include "pvr_gem.h"
#include "pvr_hwrt.h"
#include "pvr_job.h"
#include "pvr_mmu.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_meta.h"
#include "pvr_stream_defs.h"
#include "pvr_vm.h"

#include <clocksource/arm_arch_timer.h>
#include <asm/arch_timer.h>
#include <dt-bindings/interrupt-controller/apple-aic.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

/*
 * Fender beyond the power state. The boot stream is a list of Rogue register
 * writes, two words each; Fender runs it when the core's power state leaves 0.
 */
#define FENDER_CTRL_BOOT		0x40000000	/* as iOS sets it before a boot */
#define FENDER_STREAM_STATE		0x0010		/* cleared before the stream is stored */
#define FENDER_WAKE			0x0014
#define FENDER_WAKE_ALL			0x7		/* before every doorbell */
#define FENDER_BOOT_2010		0x2010		/* iOS's values before a boot; */
#define FENDER_BOOT_2014		0x2014		/* their meaning is not known */
#define FENDER_STREAM			0x10000
#define FENDER_STREAM_SIZE		0x200
#define FENDER_STREAM_END		0xffff0000
#define FENDER_CORE			0x20000
#define FENDER_CORE_HOLD		BIT(5)		/* keep the core in reset */

#define S8003_PERF_STATE_MAX		4

/* Rogue registers of the boot stream that powervr has no name for */
#define S8003_CR_E160			0xe160	/* takes the address of a spare page */

/* SOFT_RESET's first two values, as iOS sequences them */
#define S8003_SOFT_RESET_ALL		0x00e7fffffffffc1dull
#define S8003_SOFT_RESET_CORE		0x00e7fe071ffffc1dull
#define S8003_SLC3_SCRAMBLE		0xb42dd24b4bd22db4ull

/* The interrupt controller's registers the firmware uses, from its base */
#define AIC_TIME			0x1020
#define AIC_SW_SET			0x4000
#define AIC_SW_CLR			0x4080	/* the host clears the firmware's interrupt here */

/*
 * The firmware's GPU addresses: all in the GiB powervr's kernel VM keeps for
 * a firmware (IMG's bootloader is at the same address,
 * ROGUE_FW_BOOTLDR_DEVV_ADDR).
 */
#define S8003_IMAGE_VA			0xe1c0000000ull
#define S8003_SHARED_VA			0xe1c0200000ull
#define S8003_ROOT_VA			0xe1c3200000ull
#define S8003_PIO_VA			0xe1c4200000ull

#define S8003_IMAGE_SIZE		0x30000
#define S8003_ROOT_SIZE			0x8000
#define S8003_SHARED_SIZE		0x60000
#define S8003_PIO_FENDER		0x0000	/* offsets from S8003_PIO_VA */
#define S8003_PIO_AIC			0x8000
#define S8003_PIO_SIZE			0x8000	/* of each */

/* The engines' channels: pages of the shared memory, past its objects */
#define S8003_TA_CHANNEL		0x54000
#define S8003_3D_CHANNEL		0x58000
#define S8003_CU_CHANNEL		0x5b000

/*
 * The parameter manager's sets, behind the shared memory: three pages each
 * (control words with the scene's record at S8003_PM_RECORD, block list, free
 * list). As many as fit below the 3 MiB the firmware is told at
 * platform+0x77c; his driver had eight there.
 */
#define S8003_PM_SETS			12
#define S8003_PM_SET_SIZE		(3 * PVR_DEVICE_PAGE_SIZE)
#define S8003_PM_SETS_VA		(S8003_SHARED_VA + S8003_SHARED_SIZE)
#define S8003_PM_RECORD			0x80
#define S8003_PM_BLOCK_SIZE		SZ_128K	/* eight pages of the pool */
#define S8003_PM_BLOCKS_MAX		128u	/* what one page of free list holds */
/*
 * The PM's own tables for a set (macrotile array 0x80 bytes, MList 0x10000,
 * VHEAP 0x600), in the VM that uses the set, below the userspace heaps.
 */
#define S8003_PM_AUX_VA			0x101000000ull
#define S8003_PM_AUX_STRIDE		0x20000
#define S8003_PM_AUX_SIZE		(5 * PVR_DEVICE_PAGE_SIZE)

/* The firmware's slots for VMs: 0 is its own, the first ones Apple's kext keeps */
#define S8003_SLOTS			256
#define S8003_SLOT_FIRST		7
/* A slot's tag: the field whose change drops the slot's translations from the catalogue down */
#define S8003_TAG_CATALOGUE		0x00400000
#define S8003_TAG_FIRST			0x00400000
#define S8003_TAG_PARK			0x00500000

/* The unpacked image: the bootloader's configuration and what it hands over */
#define S8003_IMAGE_CONF		0x200
#define S8003_IMAGE_HANDOVER		0x3ff8	/* META addresses of INIT and CONTROL */
#define S8003_IMAGE_COREMEM		0x24000	/* core memory's contents */
#define S8003_IMAGE_COREMEM_SIZE	0x55dc
#define S8003_LDR_CONF_MAX		27	/* register writes in Apple's image */

/*
 * The objects of the root and shared memory, by the number Apple's kext
 * gives each (it numbers every object twice, for its CPU and its GPU address;
 * these are the CPU-side numbers). Those without a name here are placed and
 * announced to the firmware, and left zero.
 */
enum {
	S8003_INIT		= 0x44,	/* what the firmware reads first */
	S8003_CHANNEL		= 0x47,	/* three: 0x47, 0x48, 0x49 */
	S8003_POLICY		= 0x4a,	/* the power governor's record */
	S8003_PLATFORM		= 0x4d,	/* registers, clocks, interrupt, thread */
	S8003_CONTROL		= 0x5e,
	S8003_WORK_CTL		= 0x60,	/* work rings: indices 0x60, 0x64, 0x68, */
	S8003_WORK_RING		= 0x61,	/* entries 0x61, 0x65, 0x69 */
	S8003_DC_CTL		= 0x6c,	/* device-control ring */
	S8003_DC_RING		= 0x6d,
	S8003_EVENT_CTL		= 0x70,
	S8003_LOG_CTL		= 0x71,	/* four lanes */
	S8003_STATS_CTL		= 0x72,
	S8003_TRACE_CTL		= 0x73,
	S8003_EVENTS		= 0x74,
	S8003_LOGS		= 0x75,
	S8003_STATS		= 0x76,
	S8003_TRACE		= 0x77,
	S8003_TAG_HOST		= 0x84,	/* per slot: the host's tag, */
	S8003_TAG_SEEN		= 0x85,	/* the one the firmware saw last, */
	S8003_MMU_ROOT		= 0x86,	/* the address of the page catalogue */
	S8003_OBJECTS		= 0x90,
};

/* A ring's indices: the consumer's, 16 bytes on the producer's */
#define S8003_RING_CONSUMER		0x00
#define S8003_RING_PRODUCER		0x10
#define S8003_RING_CTL_SIZE		0x20	/* one lane's */
#define S8003_EVENT_SIZE		44
#define S8003_DC_SIZE			20

/* Events: the first word */
#define S8003_EVENT_ALIVE		0
#define S8003_EVENT_STAMP		1

/* The device-control command that carries nothing, and the ring's doorbell */
#define S8003_DC_NONE			8
#define S8003_MTS_DEVICE_CONTROL	(ROGUE_CR_MTS_SCHEDULE_TASK_COUNTED | ROGUE_CR_MTS_SCHEDULE_DM_DM0)

#define S8003_ALIVE_TIMEOUT_US		3000000
#define S8003_DC_TIMEOUT_US		3000000
/*
 * The firmware posts an "alive" event each time its scheduler runs, about 290
 * a second, idle or not (measured on the iPad 5): read the rings well before
 * their 256 entries are used.
 */
#define S8003_DRAIN_MS			100

struct s8003_layout {
	u32 offset[S8003_OBJECTS];
	u8 shared[S8003_OBJECTS];	/* in the shared memory, not the root */
};

/* An object of the root or shared memory and its size, in the order Apple's kext packs them */
struct s8003_object_size {
	u8 object;
	bool shared;
	u32 size;
};

struct pvr_apple_s8003;
struct s8003_vm;

/*
 * What differs between the chips that run this firmware: sizes, counts, and
 * the steps whose structure differs. The A9's values are the constants above;
 * a chip's start, jobs and VMs go through the same functions otherwise.
 */
/*
 * What differs on the A8's boot path between the A8 and the A7 (S5L8960X,
 * G6430, Apple's G3). The A7's AGXG3 runs the A8's AGXG4P code nearly
 * unchanged (16H88 kernelcaches compared function by function: object sizes,
 * image layout, init builders, boot stream, start, stop and kicks); these
 * are the differences, and the A7's power manager's.
 */
struct t7000_soc {
	const char *border_fw;		/* BorderColorTable: Apple's 0x800 bytes */
	u32 ps_gfx;			/* the GPU's power-state word in the PMGR */
	bool ps_domain;			/* that word is a Linux power domain's (A7) */
	u32 pll, pll_lock;		/* the GPU PLL's lock bit in the PMGR */
	bool a8_globals;		/* perf records, voltage manager, SOC_TVM (A8) */
	u32 bridge_reg, bridge_val;	/* the first bridge's one store */
	const u32 *platform_38;		/* PLATFORM +0x38: the CSC block, 30 words */
	const u32 *mhz, *mv;		/* the ADT's perf-states, 7 rows */
	u32 perf_count;			/* PLATFORM +0x310 */
	bool fence;			/* a FenceBuffer named to the firmware */
	bool slc;			/* SLC_CTRL_MISC and 0x3808 in the boot stream */
	u32 rev_major;			/* CORE_REVISION[23:16] */
	u32 chip_info[3];		/* without an ID read */
	/*
	 * The pause before each group of new accesses in the power-up and
	 * boot, in ms (T7000_SETTLE_MS). The A8 keeps its bring-up module's
	 * 100: its first driver probe without them froze, and shorter ones were
	 * never measured there.
	 */
	unsigned int settle_ms;
	/*
	 * The render records' chip words, from the kexts' own fillers (iOS
	 * 12.5.8: AGXG4P 0x847b858 and 0x84760fc, AGXG3 0x8770408 and
	 * 0x876aef0; the rest of both records and both microsequences is
	 * shared code, identical in the two kexts).
	 */
	u32 te_psg;			/* the TA record's +0x3c8 */
	u32 pbe_3d;			/* the 3D record's PBE register words, 16 bytes a target */
	u32 word_3d4;			/* the 3D record's +0x3d4; AGXG3 writes none */
};

/*
 * The A7's firmware writes at GPU VA 0xff_ffff_fff0 at boot, as context 0
 * (an MMU fault on the J71 without a page there: bank 1, tag 3); a scratch
 * page at the top of the 40-bit GPU VA space takes the write.
 */
#define A7_SCRATCH_VA			0xfffffff000ull

/* The A7's power and boot steps, at debug level */
#define a7_dbg(s, fmt, ...)							\
	do {									\
		if ((s)->v->soc && (s)->v->soc->ps_domain)			\
			dev_dbg(apple_dev((s)->apple), "A7: " fmt, ##__VA_ARGS__);	\
	} while (0)

struct s8003_variant {
	const char *name;
	const struct t7000_soc *soc;	/* the A8 path's SoC: A8 or A7 */
	u32 page_size;			/* the GPU page size the firmware's tables have */
	u32 image_size;			/* the unpacked firmware image */
	u32 root_size, shared_size;	/* the root and shared memory as mapped */
	u32 root_objects_end;		/* where the root's objects must end */
	u32 shared_objects_end;		/* where the shared memory's objects must end */
	const struct s8003_object_size *objects;
	unsigned int nr_objects;
	u32 ring_ctl_size;		/* one lane's ring indices */
	u32 slots;			/* the firmware's slots for VMs */
	u32 slot_first;			/* the first slot the host may use */
	unsigned int vm_slots;		/* a VM's slots: 2 (geometry and compute, fragment) or 1 */
	u32 channel[3];			/* the engines' channels: TA, 3D, CU offsets */
	bool channels_root;		/* the channels are in the root memory, not the shared */
	u32 ldr_conf_count;		/* the image's register writes, if a fixed number */
	bool touch;			/* the firmware needs s8003_touch() */
	const struct pvr_meta_ldr_target *ldr;

	int (*init)(struct pvr_apple *apple);
	void (*image_configure)(struct pvr_apple_s8003 *s);
	int (*map)(struct pvr_apple *apple);
	void (*unmap)(struct pvr_apple *apple);
	void (*init_build)(struct pvr_apple *apple, u32 q);
	void (*channels_init)(struct pvr_apple_s8003 *s);
	int (*boot)(struct pvr_apple *apple);
	int (*release)(struct pvr_apple *apple, u32 q);
	int (*core_off)(struct pvr_apple *apple);
	void (*idle)(struct pvr_apple *apple);
	int (*doorbell)(struct pvr_apple *apple, u32 dm);
	int (*first_command)(struct pvr_apple *apple);
	void (*no_alive)(struct pvr_apple *apple);	/* instead of the stop after a boot without alive */
	int (*vm_map)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	void (*vm_put)(struct pvr_apple *apple, struct s8003_vm *vm);
	void (*vm_unmap)(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx);
	u32 (*engine_done)(struct pvr_apple *apple, enum pvr_apple_engine engine);
	int (*prepare_job)(struct pvr_apple *apple, struct pvr_job *job);
	int (*submit_geom)(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job);
	int (*submit_frag)(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job);
	int (*submit_compute)(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job);
	void (*report_hang)(struct pvr_apple *apple);
	/* A fault event @e (S8003_EVENT_SIZE bytes) was posted: say what it is; optional */
	void (*event_fault)(struct pvr_apple *apple, const u8 *e);
};

struct s8003_mem {
	struct pvr_gem_object *obj;
	u8 *map;
	u64 device_addr;
	size_t size;
	bool mapped;
};

/*
 * What pvr_apple.c reads from the memory shared with the firmware: the
 * engines' stamps and ring indices, GPU context 0's MMU words, the event
 * ring's indices. Where the A9's firmware keeps them is settled with its
 * commands; until then they are here, zero: nothing submitted, nothing done.
 */
struct s8003_idle {
	__le32 stamp[APPLE_ENGINES];
	__le32 producer[APPLE_ENGINES];
	__le32 frag_ring;
	__le32 frag_retired;
	__le64 mmu_root;
	__le32 mmu_gen;
	__le32 event_consumer;
	__le32 event_producer;
};

/* A PM set a VM took, and the scene in it */
struct s8003_pm {
	int set;			/* of the firmware's sets; -1: none */
	u32 blocks;			/* of the pool */
	u32 built;			/* the firmware start it was built for */
	u32 rec_seq;			/* last geometry job carrying its PM record */
	struct pvr_gem_object *pool;
	struct pvr_gem_object *aux;	/* the PM's tables, mapped in the VM */
	void *pool_map, *aux_map;
	struct pvr_gem_object *next_pool;
	void *next_pool_map;
	u32 next_blocks;
	bool ready;			/* published under submit_lock and set_lock */
	const struct pvr_hwrt_data *hwrt;	/* the scene's target */
	u32 scene;			/* the firmware start the scene belongs to; 0: none */
	u32 ta_seq;			/* its geometry job */
	u32 td_seq;			/* its fragment job; 0: not submitted */
	bool clean;			/* both of its jobs were handed to the firmware */
};

/*
 * Why a job was refused (s8003_submit_error(), t7000_prepare_job()), for the
 * counters in debugfs apple_jobs. Names as printed there.
 */
enum s8003_refusal {
	S8003_REFUSED_NOT_PREPARED,	/* geometry: no region ring or PM objects */
	S8003_REFUSED_NO_GEOM,		/* fragment: its geometry job failed or was not submitted */
	S8003_REFUSED_RT,		/* the target's geometry */
	S8003_REFUSED_PM,		/* no PM set, or its pool or set not built */
	S8003_REFUSED_SLOT_WAIT,	/* the engine's record slot stayed busy */
	S8003_REFUSED_PUBLISH,		/* the firmware's ring was not ours to publish to */
	S8003_REFUSED_FILL,		/* the 3D record */
	S8003_REFUSED_PREPARE_RT,	/* at the ioctl: the target */
	S8003_REFUSED_PREPARE_SAMPLES,	/* at the ioctl: more than one sample */
	S8003_REFUSED_PREPARE_RING,	/* at the ioctl: region headers not a mirrored ring */
	S8003_REFUSALS
};

static const char *const s8003_refusal_name[S8003_REFUSALS] = {
	[S8003_REFUSED_NOT_PREPARED] = "not_prepared",
	[S8003_REFUSED_NO_GEOM] = "frag_no_geom",
	[S8003_REFUSED_RT] = "rt_derive",
	[S8003_REFUSED_PM] = "pm",
	[S8003_REFUSED_SLOT_WAIT] = "slot_wait",
	[S8003_REFUSED_PUBLISH] = "publish",
	[S8003_REFUSED_FILL] = "fill",
	[S8003_REFUSED_PREPARE_RT] = "prepare_rt",
	[S8003_REFUSED_PREPARE_SAMPLES] = "prepare_samples",
	[S8003_REFUSED_PREPARE_RING] = "prepare_ring",
};

struct s8003_vm {
	struct pvr_vm_context *vm_ctx;
	/* Only ioctl preparers take this lock; the job worker never waits on it. */
	struct mutex prepare_lock;
	u32 slot[2];			/* geometry and compute, fragment; 0: none */
	u32 attached;			/* the firmware start the slots were loaded for */
	struct s8003_pm pm[2];
	u32 fb_blocks;			/* pool size the firmware's feedback asked for */
	ktime_t fb_at;			/* when that feedback was last taken */

	/* The A8's: its PM set (submit_lock), its scene (rings: &pvr_hwrt_dataset.apple_rgn) */
	struct {
		struct pvr_gem_object *aux, *stack, *pool;
		u8 *aux_map, *stack_map, *pool_map;
		u32 blocks;			/* the pool's 128 KiB blocks (32 pages each) */
		u32 want;			/* blocks the feedback or a stall asked for */
		struct pvr_gem_object *next_pool;	/* @want blocks, taken at a geometry job */
		u8 *next_pool_map;
		u32 next_blocks;
		u32 built;			/* the firmware start the set was built for */
		bool open;			/* a geometry job waits for its fragment job */
		const struct pvr_hwrt_data *hwrt;
		u32 ta_seq;
		u64 rgn_addr;			/* the scene's region-header slice */
	} t7000;
};

/*
 * An A8 engine's jobs, phase by phase, in us (debugfs apple_jobs "a8 <engine>
 * phases"), over the jobs whose firmware timestamps were usable, so that
 * host + wait + run + tail = total for each job: from the chip's submit to
 * the fence's signal. Inside host: the PM pool's feedback and build
 * (geometry), the wait for the engine's record slot, the wait for the work
 * ring's slot. Inside wait (doorbell to the firmware's start of the work):
 * the doorbells that found the GPU awake and those that found Fender in
 * state 0 (powered down, woken by the doorbell); a fragment job kicked
 * before its geometry job's end instead: its wait for that end and the
 * firmware's hand-off from there to the 3D's start. Inside tail (the
 * firmware's end to the signal): ended by a check that the firmware's
 * interrupt led to (end to interrupt, interrupt to signal), or by a check
 * without one (the backup poll or a re-check).
 */
struct t7000_phases {
	struct pvr_apple_stat total, host, wait, run, tail;
	struct pvr_apple_stat pm, slot, ring;
	struct pvr_apple_stat wait_awake, wait_woke, ta_wait, ta_to_3d;
	struct pvr_apple_stat end_to_irq, irq_to_signal, end_to_check;
	/* the firmware's stamped start (+8): none, or more than 2 us from end - result */
	u64 start_stamp_none, start_stamp_off;
};

struct pvr_apple_s8003 {
	struct pvr_apple *apple;
	const struct s8003_variant *v;
	struct delayed_work drain_work;	/* reads the rings while the firmware runs */
	struct mutex event_lock;	/* the rings the firmware posts to */
	ktime_t touched;		/* s8003_touch() */
	bool failed;			/* its rings' indices are not ours any more */
	bool stamped;			/* it posted a job's stamp since events() last looked */

	/*
	 * Per VM (struct s8003_vm, by its VM context) and per start of the
	 * firmware: @gen counts the starts, and what the firmware was told
	 * for an earlier one is told again.
	 */
	struct xarray vms;
	u32 gen;
	spinlock_t tag_lock;		/* @tag and the host's tags in the shared memory */
	u32 tag[S8003_SLOTS];		/* 0: slot not loaded */
	u64 tag_steps_past;		/* cached or zero tags skipped, under tag_lock */
	DECLARE_BITMAP(loading, S8003_SLOTS);	/* tag held until DC_SLOT completes */
	DECLARE_BITMAP(bump_pending, S8003_SLOTS); /* flush deferred while loading */
	DECLARE_BITMAP(slots, S8003_SLOTS);
	DECLARE_BITMAP(parked, S8003_SLOTS);	/* left on @park_vm's tables */
	u32 next_slot;
	spinlock_t set_lock;		/* @sets, @next_set, &s8003_pm.set */
	DECLARE_BITMAP(sets, S8003_PM_SETS);
	u32 next_set;
	/*
	 * Which sets the firmware may be writing, for s8003_touch(): per set
	 * the last geometry and fragment job on it (0: none this start), and
	 * the sets of the VMs that submitted the last two scenes. Written
	 * under submit_lock, read without it.
	 */
	u32 set_geom[S8003_PM_SETS];
	u32 set_frag[S8003_PM_SETS];
	unsigned long wb_sets;
	struct s8003_vm *pm_last, *pm_prev; /* last two scenes for cache-line reads */
	struct pvr_vm_context *park_vm;	/* maps what every VM maps, nothing else */

	/* Bus addresses the firmware is told: its registers' and ours */
	phys_addr_t fender_phys;
	phys_addr_t rgx_phys;
	phys_addr_t aic_phys;
	u32 irq;			/* the firmware's interrupt to the host, as the AIC numbers it */
	void __iomem *aic_sw_clr;	/* its word of the AIC's software-clear registers */

	struct s8003_layout layout;
	u8 *image;			/* the unpacked image, as the bootloader finds it */
	u32 conf[S8003_LDR_CONF_MAX][2];
	u32 conf_count;
	u32 thread_sp;
	u32 thread_pc;

	struct s8003_mem fw;		/* the image the firmware runs in */
	struct s8003_mem root;
	struct s8003_mem shared;
	struct s8003_mem pmsets;
	struct s8003_mem zero;		/* at GPU address 0 */
	struct pvr_gem_object *spare;
	bool pio_mapped;

	/* The A8's (T7000_*) */
	void __iomem *pmgr;		/* the power manager: GPU power state, PLL5 */
	void __iomem *bridges;		/* the GPU's two bus bridges */
	u8 *border;			/* BorderColorTable's contents (firmware file) */
	u32 chip_info[3];		/* PLATFORM +0x300..+0x308, from the core's ID */
	struct s8003_mem hwobj[4];	/* the hardware objects at 0x8_00000000 */
	struct s8003_mem a7_scratch;	/* the A7's: GPU VA 0xff_ffff_f000 */
	bool t7000_pio_mapped;
	bool stuck;			/* Fender stayed busy: the GPU is left alone */
	u32 t7000_trace;		/* the records' trace ID, one per job */

	struct s8003_idle idle;

	/* The A8: the smallest PM pool a stall taught; the VM of the last geometry job */
	u32 t7000_pm_floor;
	struct s8003_vm *t7000_geom_vm;
	bool t7000_mmu_arm;		/* a lockup event came from the MMU-fault arm */
	/*
	 * The A8's job timeline (debugfs apple_jobs), per engine: the system
	 * counter at the doorbell; the firmware's own timestamps of the job
	 * (start, end), read once when the job retires; under fence_lock.
	 */
	u64 t7000_kick[APPLE_ENGINES];
	struct pvr_apple_stat t7000_fw_wait_us[APPLE_ENGINES], t7000_fw_run_us[APPLE_ENGINES];
	struct pvr_apple_stat t7000_fw_tail_us[APPLE_ENGINES];
	u64 t7000_fw_bad[APPLE_ENGINES], t7000_pm_builds;
	/*
	 * Where each job's time goes (t7000_phases()), in system counter ticks:
	 * set at submit under submit_lock (the A8 has one job per engine with
	 * the firmware, and an engine's next job is submitted only after this
	 * one retired), read when it retires under fence_lock. The geometry
	 * job's firmware end, for the hand-off to its fragment job.
	 */
	u64 t7000_submit_at[APPLE_ENGINES];
	u32 t7000_pm_ticks[APPLE_ENGINES], t7000_slot_ticks[APPLE_ENGINES];
	u32 t7000_ring_ticks[APPLE_ENGINES];
	bool t7000_kick_woke[APPLE_ENGINES];
	u32 t7000_geom_end;
	struct t7000_phases t7000_ph[APPLE_ENGINES];
	/* The 3D record the last fragment job was written to (T7000_RCH_W or T7000_RCH1_W) */
	u32 t7000_frag_w;
	/*
	 * The A8's firmware interrupts by what the check that took each one
	 * found (t7000_irq_checked()): its job, or nothing after a poll had
	 * retired a job since the previous interrupt, or nothing with a stamp
	 * event posted, or with only log/statistics/trace entries, or with
	 * nothing new at all; under fence_lock.
	 */
	u64 t7000_irq_found, t7000_irq_polled, t7000_irq_stamp_event, t7000_irq_rings, t7000_irq_none;
	/* stamp events posted and jobs retired (statistics), their difference (kept) */
	u64 t7000_stamp_events, t7000_jobs_retired;
	s64 t7000_ev_lead;
	u32 t7000_ev_cursor, t7000_ev_gen;	/* the next event to count, for start @t7000_ev_gen */

	/* Statistics (debugfs apple_jobs): live per-VM states and A8 PM objects, doorbells */
	atomic_t vms_live, pm_objs_live;
	atomic64_t doorbells, doorbell_wakes, doorbell_pll_unlocked, doorbell_refused;
	atomic64_t refused[S8003_REFUSALS];	/* jobs refused, by reason */
	bool bif_read;			/* t7000_report_fault() read the fault registers */
};

static void put32(u8 *p, u32 value)
{
	put_unaligned_le32(value, p);
}

static void put64(u8 *p, u64 value)
{
	put_unaligned_le64(value, p);
}

/*
 * The firmware's 32-bit address of a GPU address in its GiB. The root memory
 * is reached through META's uncached range, the rest through its cached one.
 */
static u32 s8003_meta_addr(u64 va)
{
	u32 range = va >= S8003_ROOT_VA && va < S8003_ROOT_VA + SZ_16M ? 0x40000000 : 0xc0000000;

	return (lower_32_bits(va) + 0x40000000) | range;
}

/* The A9's objects, as Apple's kext packs them */
static const struct s8003_object_size s8003_objects_a9[] = {
	{ S8003_INIT, false, 0x1490 },
	{ S8003_CHANNEL, false, 0x990 },
	{ S8003_CHANNEL + 1, false, 0x8e0 },
	{ S8003_CHANNEL + 2, false, 0x8c0 },
	{ S8003_POLICY, false, 0x938 },
	{ 0x4b, false, 0x28 },
	{ 0x4c, false, 8 },
	{ S8003_PLATFORM, false, 0x78c },
	{ 0x4e, false, 0x84 },
	{ S8003_CONTROL, true, 0x2058 },
	{ S8003_EVENT_CTL, true, S8003_RING_CTL_SIZE },
	{ S8003_LOG_CTL, true, 4 * S8003_RING_CTL_SIZE },
	{ S8003_STATS_CTL, true, S8003_RING_CTL_SIZE },
	{ S8003_TRACE_CTL, true, S8003_RING_CTL_SIZE },
	{ S8003_EVENTS, true, 0x2c00 },
	{ S8003_LOGS, true, 0xb000 },
	{ 0x4f, true, 0x36000 },
	{ S8003_STATS, true, 0x2c00 },
	{ S8003_TRACE, true, 0x2400 },
	{ S8003_DC_CTL, true, S8003_RING_CTL_SIZE },
	{ S8003_DC_RING, true, 0x1400 },
	{ S8003_WORK_CTL, true, S8003_RING_CTL_SIZE },
	{ S8003_WORK_CTL + 4, true, S8003_RING_CTL_SIZE },
	{ S8003_WORK_CTL + 8, true, S8003_RING_CTL_SIZE },
	{ S8003_WORK_RING, true, 0x1400 },
	{ S8003_WORK_RING + 4, true, 0x1400 },
	{ S8003_WORK_RING + 8, true, 0x1400 },
	{ 0x5c, true, 8 },
	{ 0x85, true, 0x400 },
	{ 0x84, true, 0x400 },
	{ S8003_MMU_ROOT, true, 0x800 },
};

/* Apple's kext packs each group in the variant's order, every object on 64 bytes. */
static int s8003_layout_plan(const struct s8003_variant *v, struct s8003_layout *layout)
{
	const struct s8003_object_size *objects = v->objects;
	u32 used[2] = {};
	unsigned int i;

	for (i = 0; i < v->nr_objects; i++) {
		u32 offset = ALIGN(used[objects[i].shared], 64);

		layout->offset[objects[i].object] = offset;
		layout->shared[objects[i].object] = objects[i].shared;
		used[objects[i].shared] = offset + objects[i].size;
	}
	return used[0] <= v->root_objects_end && used[1] <= v->shared_objects_end ? 0 : -EINVAL;
}

static u64 s8003_object_va(const struct s8003_layout *layout, u32 object)
{
	return (layout->shared[object] ? S8003_SHARED_VA : S8003_ROOT_VA) + layout->offset[object];
}

static u32 s8003_object_addr(const struct s8003_layout *layout, u32 object)
{
	return s8003_meta_addr(s8003_object_va(layout, object));
}

static u8 *s8003_object(struct pvr_apple_s8003 *s, u32 object)
{
	return (s->layout.shared[object] ? s->shared.map : s->root.map) + s->layout.offset[object];
}

/*
 * Where the image's sections go in the unpacked image, as Apple's kext lays
 * them out. The local and core memory ranges are not zeroed here: the
 * bootloader does that.
 */
static int s8003_ldr_mem(void *data, u32 addr, u32 size, bool zero, void **write_addr)
{
	static const struct {
		u32 base, size, offset;
	} sections[] = {
		{ 0x10000000, 0x18000, 0x04000 },
		{ 0x20000000, 0x04000, 0x1c000 },
		{ 0xc0000000, 0x04000, 0x00000 },
		{ 0xe0001000, 0x04000, 0x20000 },
		{ 0x80000080, S8003_IMAGE_COREMEM_SIZE, S8003_IMAGE_COREMEM },
	};
	struct pvr_apple_s8003 *s = data;
	unsigned int i;

	if (zero && addr >= 0x80000000 && addr < 0x88000000) {
		*write_addr = NULL;
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(sections); i++) {
		if (addr >= sections[i].base && size <= sections[i].size &&
		    addr - sections[i].base <= sections[i].size - size) {
			*write_addr = s->image + sections[i].offset + addr - sections[i].base;
			return 0;
		}
	}
	return -EINVAL;
}

static int s8003_ldr_config(void *data, u32 reg, u32 value)
{
	struct pvr_apple_s8003 *s = data;

	if (s->conf_count == S8003_LDR_CONF_MAX)
		return -EINVAL;
	s->conf[s->conf_count][0] = reg;
	s->conf[s->conf_count][1] = value;
	s->conf_count++;
	return 0;
}

/* The one thread the image starts: its number, stack pointer and entry */
static int s8003_ldr_start_threads(void *data, const u32 *words, u32 count)
{
	struct pvr_apple_s8003 *s = data;

	if (count < 3 || s->thread_pc)
		return -EINVAL;
	s->thread_sp = words[1];
	s->thread_pc = words[2];
	return 0;
}

/*
 * The bootloader's configuration: the segment MMU's view of the image (code,
 * data, the bootloader itself, one more data range), the image's own register
 * writes, and where the bootloader finds the addresses of INIT and CONTROL.
 */
static void s8003_image_configure(struct pvr_apple_s8003 *s)
{
	static const struct {
		u32 base, size, offset;
	} segments[] = {
		{ 0x10000000, 0x18000, 0x04000 },
		{ 0x20000000, 0x04000, 0x1c000 },
		{ 0x40000000, 0x04000, 0x00000 },
		{ 0x60001000, 0x04000, 0x20000 },
	};
	u8 *conf = s->image + S8003_IMAGE_CONF;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(segments); i++) {
		u64 out = S8003_IMAGE_VA + segments[i].offset;

		/* The bootloader runs in segment 2: only its cache policy is set */
		if (i == 2) {
			out |= ROGUE_FW_SEGMMU_OUTADDR_TOP_VIVT_SLC_UNCACHED(0);
			put32(conf, META_CR_MMCU_SEGMENT_N_OUTA1(i));
			put32(conf + 4, upper_32_bits(out));
			conf += 8;
			continue;
		}
		out |= ROGUE_FW_SEGMMU_OUTADDR_TOP_VIVT_SLC_CACHED(0);
		put32(conf, META_CR_MMCU_SEGMENT_N_BASE(i));
		put32(conf + 4, segments[i].base | ROGUE_FW_SEGMMU_ALLTHRS_WRITEABLE);
		put32(conf + 8, META_CR_MMCU_SEGMENT_N_LIMIT(i));
		put32(conf + 12, segments[i].size - 1);
		put32(conf + 16, META_CR_MMCU_SEGMENT_N_OUTA0(i));
		put32(conf + 20, lower_32_bits(out));
		put32(conf + 24, META_CR_MMCU_SEGMENT_N_OUTA1(i));
		put32(conf + 28, upper_32_bits(out));
		conf += 32;
	}
	for (i = 0; i < s->conf_count; i++, conf += 8) {
		put32(conf, s->conf[i][0]);
		put32(conf + 4, s->conf[i][1]);
	}
	put64(conf, 0);
	put32(conf + 8, s8003_meta_addr(S8003_IMAGE_VA + S8003_IMAGE_HANDOVER));
	put32(s->image + S8003_IMAGE_HANDOVER, s8003_object_addr(&s->layout, S8003_INIT));
	put32(s->image + S8003_IMAGE_HANDOVER + 4, s8003_object_addr(&s->layout, S8003_CONTROL));
}

/*
 * The GPU's performance states for the firmware's governor: its clock table
 * (MHz per Fender power state, iOS's on the iPad 5) and limits that pin the
 * governor to state @q, the state the core is powered up in. Pinned, it never
 * changes the state and never powers the core off while idle: the host does
 * that (runtime PM).
 */
static void s8003_init_perf(struct pvr_apple_s8003 *s, u32 q)
{
	static const u32 mhz[] = { 0, 340, 474, 550, 723 };
	u8 *platform = s8003_object(s, S8003_PLATFORM);
	u8 *control = s8003_object(s, S8003_CONTROL);
	u8 *policy = s8003_object(s, S8003_POLICY);
	u32 top = 100 * (ARRAY_SIZE(mhz) - 1), pin = 100 * q;
	unsigned int i;

	put32(platform + 0x710, ARRAY_SIZE(mhz) - 1);
	put32(platform + 0x714, ARRAY_SIZE(mhz));
	for (i = 0; i < ARRAY_SIZE(mhz); i++)
		put32(platform + 0x718 + 4 * i, mhz[i]);
	put32(control + 0x1028, 3348);		/* power envelope, mW */
	put32(control + 0x102c, pin);		/* floor */
	put32(control + 0x1030, top);
	put32(control + 0x1038, top);		/* overrides of the governor's limits */
	put32(control + 0x1040, pin);
	put32(policy + 0x688, top);
	put32(policy + 0x68c, top);
	put32(policy + 0x6f0, pin);
	put32(policy + 0x6f4, pin);
	/*
	 * Above state 1 the governor's record starts as it would leave it for
	 * this target: the firmware's first sample then finds the core in the
	 * state it wants.
	 */
	if (q > 1) {
		put64(policy + 0x28, (u64)q << 32 | q);
		put32(policy + 0x30, 0);
		put32(policy + 8, 0);
	}
}

/*
 * The root and shared memory as the firmware finds it when it starts: where
 * its objects are, the registers it may use, the interrupt it raises, its
 * thread, and empty rings. What Apple's kext fills from the platform's power
 * policy beyond s8003_init_perf() stays zero.
 */
static void s8003_init_build(struct pvr_apple *apple, u32 q)
{
	static const u8 rings[] = {
		S8003_WORK_CTL, S8003_WORK_RING, S8003_WORK_CTL + 4, S8003_WORK_RING + 4,
		S8003_WORK_CTL + 8, S8003_WORK_RING + 8, S8003_DC_CTL, S8003_DC_RING,
		S8003_EVENT_CTL, S8003_EVENTS, S8003_LOG_CTL, S8003_LOGS,
		S8003_STATS_CTL, S8003_STATS, S8003_TRACE_CTL, S8003_TRACE,
	};
	static const u8 objects[] = {
		S8003_CHANNEL, S8003_CHANNEL + 1, S8003_CHANNEL + 2, S8003_POLICY, 0x4b,
	};
	static const u8 tail[] = { 0x4e, 0x4c, S8003_PLATFORM, 0x85, 0x4f };
	struct pvr_apple_s8003 *s = apple->chip_data;
	const struct s8003_layout *layout = &s->layout;
	const struct {
		u64 phys;
		u32 addr;
		u32 size;
	} pio[] = {
		{ s->fender_phys, s8003_meta_addr(S8003_PIO_VA + S8003_PIO_FENDER), 0x5000 },
		{ s->aic_phys, s8003_meta_addr(S8003_PIO_VA + S8003_PIO_AIC), 0x4000 },
		{ s->aic_phys + AIC_SW_SET,
		  s8003_meta_addr(S8003_PIO_VA + S8003_PIO_AIC + AIC_SW_SET), 0x4000 },
		{ s->rgx_phys, 0, 0x20000 },
	};
	u8 *init, *platform, *policy, *control;
	u64 now = arch_timer_read_counter();
	unsigned int i;

	memset(s->root.map, 0, S8003_ROOT_SIZE);
	memset(s->shared.map, 0, S8003_SHARED_SIZE);
	init = s8003_object(s, S8003_INIT);
	platform = s8003_object(s, S8003_PLATFORM);
	policy = s8003_object(s, S8003_POLICY);
	control = s8003_object(s, S8003_CONTROL);

	for (i = 0; i < ARRAY_SIZE(rings); i++)
		put32(init + 4 * i, s8003_object_addr(layout, rings[i]));
	for (i = 0; i < ARRAY_SIZE(objects); i++)
		put32(init + 0x64 + 4 * i, s8003_object_addr(layout, objects[i]));
	for (i = 0; i < ARRAY_SIZE(tail); i++)
		put32(init + 0x44c + 4 * i, s8003_object_addr(layout, tail[i]));
	for (i = 0; i < 3; i++) {
		u8 *channel = s8003_object(s, S8003_CHANNEL + i);
		u32 addr = s8003_object_addr(layout, S8003_CHANNEL + i);

		put32(channel + 4, addr);
		put32(channel + 12, addr + 8);
	}
	put32(s8003_object(s, S8003_CHANNEL + 1) + 0x778, 0xffffffff);
	/* Core memory's contents: uncached address, GPU address, size */
	put32(init + 0x480,
	      (lower_32_bits(S8003_IMAGE_VA + S8003_IMAGE_COREMEM) + 0x40000000) | 0x40000000);
	put64(init + 0x484, S8003_IMAGE_VA + S8003_IMAGE_COREMEM);
	put32(init + 0x48c, S8003_IMAGE_COREMEM_SIZE);

	put64(platform, 0x600000000ull);
	put64(platform + 8, 0xffc00000);
	/* What PDS and USC code addresses are relative to: powervr's code heaps */
	put64(platform + 0x10, PVR_APPLE_PDSCODEDATA_HEAP_BASE);
	put64(platform + 0x18, PVR_APPLE_USCCODE_HEAP_BASE);
	for (i = 0; i < ARRAY_SIZE(pio); i++) {
		u8 *entry = platform + 0x630 + 16 * i;

		put64(entry, pio[i].phys);
		put32(entry + 8, pio[i].addr);
		put32(entry + 12, pio[i].size);
	}
	put32(platform + 0x6c0, 1);
	put32(platform + 0x6c8, 1);
	put64(platform + 0x6d8, 0x100000000ull);
	put32(platform + 0x6e0, 1);
	put32(platform + 0x6e4, 1);
	/* From the core's revision (15): 1, 5, and its 6 clusters */
	put32(platform + 0x704, 1);
	put32(platform + 0x708, 5);
	put32(platform + 0x70c, 6);
	/* The interrupt to the host: its word and bit among the AIC's software sets */
	put32(platform + 0x758, 4 * (s->irq / 32));
	put32(platform + 0x75c, BIT(s->irq % 32));
	put32(platform + 0x760, AIC_TIME);
	put32(platform + 0x764, s->thread_pc);
	put32(platform + 0x768, s->thread_sp);
	put64(platform + 0x774, S8003_IMAGE_VA);
	put64(platform + 0x77c, 0x300000);
	/* +0x784: what the firmware adds to the AIC's time; Linux counts from it */

	put64(policy + 8, 0x400000000ull);
	put64(policy + 0x10, 0x3f800000);
	put64(policy + 0x28, 0x100000001ull);
	put32(policy + 0x38, 100);
	put64(policy + 0x8f8, now);		/* the governor's last sample */

	put32(control + 0x24, 0x78);
	put32(control + 0x30, 1);
	put32(control + 0x1044, s8003_object_addr(layout, 0x84));
	put32(control + 0x1048, s8003_object_addr(layout, S8003_MMU_ROOT));
	put32(control + 0x104c, 1);
	put64(s8003_object(s, S8003_MMU_ROOT),
	      pvr_vm_get_page_table_root_addr(apple->pvr_dev->kernel_vm_ctx));

	s8003_init_perf(s, q);
}

static void s8003_mem_fini(struct pvr_apple *apple, struct s8003_mem *mem)
{
	if (!mem->obj)
		return;
	if (mem->mapped)
		pvr_vm_unmap_obj(apple->pvr_dev->kernel_vm_ctx, mem->obj, mem->device_addr,
				 mem->size);
	if (mem->map)
		pvr_gem_object_vunmap(mem->obj);
	pvr_gem_object_put(mem->obj);
	memset(mem, 0, sizeof(*mem));
}

static int s8003_mem_init(struct pvr_apple *apple, struct s8003_mem *mem, u64 device_addr,
			  size_t size, u64 flags)
{
	int err;

	mem->obj = pvr_gem_object_create(apple->pvr_dev, size, flags);
	if (IS_ERR(mem->obj)) {
		err = PTR_ERR(mem->obj);
		mem->obj = NULL;
		return err;
	}
	mem->size = size;
	mem->map = pvr_gem_object_vmap(mem->obj);
	if (IS_ERR(mem->map)) {
		err = PTR_ERR(mem->map);
		mem->map = NULL;
		goto err_fini;
	}
	err = pvr_vm_map(apple->pvr_dev->kernel_vm_ctx, mem->obj, 0, device_addr, size);
	if (err)
		goto err_fini;
	mem->device_addr = device_addr;
	mem->mapped = true;
	return 0;

err_fini:
	s8003_mem_fini(apple, mem);
	return err;
}

/* The registers' windows in the kernel VM */
static void s8003_unmap(struct pvr_apple *apple)
{
	struct pvr_vm_context *vm_ctx = apple->pvr_dev->kernel_vm_ctx;
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (s->pio_mapped) {
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + S8003_PIO_FENDER, S8003_PIO_SIZE);
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + S8003_PIO_AIC, S8003_PIO_SIZE);
		s->pio_mapped = false;
	}
}

static void pvr_apple_s8003_fini(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	/* A firmware left running keeps its memory (t7000_stuck()) */
	if (s->stuck)
		return;
	s->v->unmap(apple);
	if (s->park_vm) {
		pvr_vm_context_put(s->park_vm);
		s->park_vm = NULL;
	}
	s8003_mem_fini(apple, &s->zero);
	s8003_mem_fini(apple, &s->pmsets);
	s8003_mem_fini(apple, &s->shared);
	s8003_mem_fini(apple, &s->root);
	s8003_mem_fini(apple, &s->fw);
	if (s->spare) {
		pvr_gem_object_put(s->spare);
		s->spare = NULL;
	}
}

/*
 * The firmware's memory and registers in the kernel VM. Everything is for the
 * firmware only (PM/firmware protect): the image, the root and shared memory
 * cache coherent as Apple's kext maps them, the registers past the cache.
 * One plain zeroed page lies at address 0, which nothing else maps there: an
 * access the firmware's context makes at 0 while it starts lands in it
 * instead of faulting (iOS can map a dummy page for every unmapped address).
 */
static int s8003_map(struct pvr_apple *apple)
{
	struct pvr_vm_context *vm_ctx = apple->pvr_dev->kernel_vm_ctx;
	struct pvr_apple_s8003 *s = apple->chip_data;
	int err;

	err = s8003_mem_init(apple, &s->fw, S8003_IMAGE_VA, S8003_IMAGE_SIZE,
			     DRM_PVR_BO_PM_FW_PROTECT);
	if (!err)
		err = s8003_mem_init(apple, &s->root, S8003_ROOT_VA, S8003_ROOT_SIZE,
				     DRM_PVR_BO_PM_FW_PROTECT | PVR_BO_DEVICE_CACHE_COHERENT);
	if (!err)
		err = s8003_mem_init(apple, &s->shared, S8003_SHARED_VA, S8003_SHARED_SIZE,
				     DRM_PVR_BO_PM_FW_PROTECT | PVR_BO_DEVICE_CACHE_COHERENT);
	if (!err)
		err = s8003_mem_init(apple, &s->pmsets, S8003_PM_SETS_VA,
				     S8003_PM_SETS * S8003_PM_SET_SIZE,
				     DRM_PVR_BO_PM_FW_PROTECT | PVR_BO_DEVICE_CACHE_COHERENT);
	if (!err)
		err = s8003_mem_init(apple, &s->zero, 0, PVR_DEVICE_PAGE_SIZE,
				     PVR_BO_FW_FLAGS_DEVICE_CACHED);
	if (!err) {
		s->spare = pvr_gem_object_create(apple->pvr_dev, PVR_DEVICE_PAGE_SIZE,
						 PVR_BO_FW_FLAGS_DEVICE_CACHED);
		if (IS_ERR(s->spare)) {
			err = PTR_ERR(s->spare);
			s->spare = NULL;
		}
	}
	if (!err)
		err = pvr_vm_map_io(vm_ctx, s->fender_phys, S8003_PIO_VA + S8003_PIO_FENDER,
				    S8003_PIO_SIZE);
	if (!err) {
		err = pvr_vm_map_io(vm_ctx, s->aic_phys, S8003_PIO_VA + S8003_PIO_AIC,
				    S8003_PIO_SIZE);
		if (err)
			pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + S8003_PIO_FENDER, S8003_PIO_SIZE);
		else
			s->pio_mapped = true;
	}
	if (err)
		pvr_apple_s8003_fini(apple);
	return err;
}

static void fender_write(struct pvr_apple *apple, u32 reg, u32 value)
{
	writel(value, apple->fender + reg);
	readl(apple->fender + reg);	/* each write lands before the next */
}

static int fender_wait(struct pvr_apple *apple, u32 reg, u32 mask, u32 value)
{
	u32 val;
	int err;

	err = readl_poll_timeout(apple->fender + reg, val, (val & mask) == value, 10, 100000);
	if (err)
		dev_err(apple_dev(apple), "Fender %#x is %#x, not %#x of %#x\n", reg, val, value,
			mask);
	return err;
}

/* Take the core's reset request back and power it off (Fender state 0) */
static int s8003_core_off(struct pvr_apple *apple)
{
	int err;

	fender_write(apple, FENDER_CORE, 0);
	err = fender_wait(apple, FENDER_POWER_STATE, FENDER_POWER_REQUEST, 0);
	if (!err)
		err = pvr_apple_request_power_state(apple, 0);
	return err;
}

struct s8003_stream {
	struct pvr_apple *apple;
	u32 offset;
	int err;
};

static void stream_word(struct s8003_stream *stream, u32 word)
{
	void __iomem *reg = stream->apple->fender + FENDER_STREAM + stream->offset;

	if (stream->err)
		return;
	if (stream->offset >= FENDER_STREAM_SIZE) {
		stream->err = -ENOSPC;
		return;
	}
	writel(word, reg);
	if (readl(reg) != word)
		stream->err = -EIO;
	stream->offset += 4;
}

/*
 * One 32-bit write. iOS's stream has 0xff above the register's offset in the
 * last (or only) half of a register, and 0 in the upper half before it.
 */
static void stream_write32(struct s8003_stream *stream, u32 reg, u32 value, bool last)
{
	stream_word(stream, reg | (last ? 0xff << 16 : 0));
	stream_word(stream, value);
}

static void stream_write64(struct s8003_stream *stream, u32 reg, u64 value)
{
	stream_write32(stream, reg + 4, upper_32_bits(value), false);
	stream_write32(stream, reg, lower_32_bits(value), true);
}

/*
 * Boot the META: with the core off and held in reset, store the boot stream,
 * then let go and power the core up in state @q. Fender replays the stream,
 * whose last write releases the reset; the bootloader in the image runs from
 * the kernel VM's page tables.
 */
static int s8003_release(struct pvr_apple *apple, u32 q)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_stream stream = { .apple = apple };
	dma_addr_t spare;
	int err;

	err = pvr_gem_get_dma_addr(s->spare, 0, &spare);
	if (err)
		return err;

	err = s8003_core_off(apple);
	if (err)
		return err;
	fender_write(apple, FENDER_CORE, FENDER_CORE_HOLD);
	err = fender_wait(apple, FENDER_CORE, FENDER_CORE_HOLD, FENDER_CORE_HOLD);
	if (!err)
		err = fender_wait(apple, FENDER_POWER_STATE, FENDER_POWER_STATE_MASK, 0);
	if (err)
		return err;

	fender_write(apple, FENDER_STREAM_STATE, 0);
	fender_write(apple, FENDER_CTRL, FENDER_CTRL_BOOT);
	stream_write64(&stream, ROGUE_CR_SOFT_RESET, S8003_SOFT_RESET_ALL);
	stream_write64(&stream, ROGUE_CR_SOFT_RESET, S8003_SOFT_RESET_CORE);
	stream_write64(&stream, ROGUE_CR_SOFT_RESET, ROGUE_CR_SOFT_RESET_GARTEN_EN);
	stream_write32(&stream, ROGUE_CR_META_BOOT, ROGUE_CR_META_BOOT_MODE_EN, true);
	stream_write32(&stream, ROGUE_CR_MMU_CBASE_MAPPING_CONTEXT, 0, true);
	stream_write64(&stream, ROGUE_CR_MMU_CBASE_MAPPING,
		       pvr_vm_get_page_table_root_addr(apple->pvr_dev->kernel_vm_ctx) >> 12);
	stream_write64(&stream, ROGUE_CR_SLC3_CTRL_MISC,
		       ROGUE_CR_SLC3_CTRL_MISC_ADDR_DECODE_MODE_SCRAMBLE_PVR_HASH);
	stream_write64(&stream, ROGUE_CR_SLC3_SCRAMBLE, S8003_SLC3_SCRAMBLE);
	stream_write64(&stream, ROGUE_CR_SLC3_SCRAMBLE2, S8003_SLC3_SCRAMBLE);
	fender_write(apple, FENDER_BOOT_2010, 0xbf);
	fender_write(apple, FENDER_BOOT_2014, 0x233);
	stream_write64(&stream, S8003_CR_E160, spare);
	stream_write64(&stream, ROGUE_CR_GARTEN_SLC, 0);
	stream_write64(&stream, ROGUE_CR_SOFT_RESET, 0);
	stream_word(&stream, FENDER_STREAM_END);
	if (stream.err) {
		dev_err(apple_dev(apple), "Fender did not keep the boot stream: %d\n", stream.err);
		return stream.err;
	}
	wmb();	/* the image and init data are in memory before the META runs */

	fender_write(apple, FENDER_CORE, 0);
	err = fender_wait(apple, FENDER_POWER_STATE, FENDER_POWER_STATE_MASK, 0);
	if (err)
		return err;
	mutex_lock(&apple->perf_lock);
	err = pvr_apple_request_power_state(apple, q);
	mutex_unlock(&apple->perf_lock);
	return err;
}

static u32 ring_read(u8 *ctl, u32 index)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(ctl + index)));
}

static void ring_consume(u8 *ctl, u32 consumer)
{
	/* what was read from the entries, before the firmware may reuse them */
	mb();
	WRITE_ONCE(*(__le32 *)(ctl + S8003_RING_CONSUMER), cpu_to_le32(consumer));
}

/* Give a ring's entries back unread: the firmware's logs, statistics and trace */
static int s8003_ring_discard(u8 *ctl)
{
	u32 consumer = ring_read(ctl, S8003_RING_CONSUMER);
	u32 producer = ring_read(ctl, S8003_RING_PRODUCER);

	if (consumer >= APPLE_RING_ENTRIES || producer >= APPLE_RING_ENTRIES)
		return -EPROTO;
	if (consumer != producer)
		ring_consume(ctl, producer);
	return 0;
}

/*
 * Read what the firmware posted, in the order Apple's kext does: the log
 * lanes, the events, statistics, trace. A full ring stalls the firmware.
 * Events other than "alive" and a job's stamp are a fault or a recovery
 * request: they are logged, and end a wait for a job the firmware gave up. An
 * error is a ring whose indices are not the host's and the firmware's any
 * more.
 */
static int s8003_drain(struct pvr_apple *apple, bool *alive)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_EVENT_CTL);
	u8 *events = s8003_object(s, S8003_EVENTS);
	u32 consumer, producer;
	unsigned int lane;
	int err = 0;

	guard(mutex)(&s->event_lock);
	for (lane = 0; lane < 4 && !err; lane++)
		err = s8003_ring_discard(s8003_object(s, S8003_LOG_CTL) +
					 lane * s->v->ring_ctl_size);
	if (err)
		return err;

	consumer = ring_read(ctl, S8003_RING_CONSUMER);
	producer = ring_read(ctl, S8003_RING_PRODUCER);
	if (consumer >= APPLE_RING_ENTRIES || producer >= APPLE_RING_ENTRIES)
		return -EPROTO;
	rmb();	/* the index before the entries it covers */
	while (consumer != producer) {
		const u8 *e = events + consumer * S8003_EVENT_SIZE;
		u32 type = get_unaligned_le32(e);

		if (type == S8003_EVENT_ALIVE) {
			*alive = true;
		} else if (type == S8003_EVENT_STAMP) {
			WRITE_ONCE(s->stamped, true);
		} else {
			dev_warn_ratelimited(apple_dev(apple), "firmware fault event type=%u slot=%u: %*ph\n",
					     type, get_unaligned_le32(e + 0x0c),
					     S8003_EVENT_SIZE - 4, e + 4);
			if (s->v->event_fault)
				s->v->event_fault(apple, e);
			pvr_apple_firmware_fault(apple);
		}
		consumer = (consumer + 1) % APPLE_RING_ENTRIES;
		ring_consume(ctl, consumer);
	}

	err = s8003_ring_discard(s8003_object(s, S8003_STATS_CTL));
	if (!err)
		err = s8003_ring_discard(s8003_object(s, S8003_TRACE_CTL));
	return err;
}

/* While the firmware runs and until its interrupt is taken */
static void s8003_drain_work(struct work_struct *work)
{
	struct pvr_apple_s8003 *s = container_of(to_delayed_work(work), struct pvr_apple_s8003,
						 drain_work);
	bool alive = false;
	int err;

	if (READ_ONCE(s->apple->stopping))
		return;
	err = s8003_drain(s->apple, &alive);
	if (err) {
		dev_err(apple_dev(s->apple), "GFX firmware's rings are no longer read: %d\n", err);
		return;
	}
	if (!READ_ONCE(s->apple->stopping))
		schedule_delayed_work(&s->drain_work, msecs_to_jiffies(S8003_DRAIN_MS));
}

static int s8003_wait_alive(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	bool alive = false;
	int err, drained = 0;

	err = read_poll_timeout(s8003_drain, drained, drained || alive, 1000,
				S8003_ALIVE_TIMEOUT_US, false, apple, &alive);
	if (!err)
		err = drained;
	if (err && s->v->no_alive)	/* no Rogue or Fender access after a failed boot */
		dev_err(apple_dev(apple), "GFX firmware did not come up: %d\n", err);
	else if (err)
		dev_err(apple_dev(apple), "GFX firmware did not come up: %d (Fender state %#x)\n",
			err, readl(apple->fender + FENDER_POWER_STATE));
	return err;
}

/*
 * The core is powered and no power state request is in flight. The firmware
 * makes requests of its own, so one in flight is waited for, as Apple's kext
 * does before it touches the core.
 */
static bool s8003_core_ready(struct pvr_apple *apple)
{
	u32 state;

	if (readl_poll_timeout(apple->fender + FENDER_POWER_STATE, state,
			       !(state & FENDER_POWER_REQUEST), 10, 5000))
		return false;
	return state & FENDER_POWER_STATE_MASK;
}

/*
 * Jobs.
 *
 * Each engine has one channel, a page of the shared memory: a ring of 16
 * pointers with five watermarks (the host's own among them), two stamps, a
 * timestamp queue and a dependency object. A job is a work record ("W") the
 * firmware loads the engine's registers from, plus a microsequence the
 * firmware runs for it: start, wait for the engine, finish, each step naming
 * the addresses it reads and the stamp it writes. A render job also has a
 * small command ahead of its record: for a geometry job the parameter
 * manager's record to start from, for a fragment job the geometry job's stamp
 * to wait for. The pointer ring names command and record, an entry of the
 * engine's 256-entry work ring names the channel and how far the pointer ring
 * is filled, and the doorbell is the META's scheduler register.
 *
 * A job is done when both stamps of its channel reached its number and all
 * five watermarks its end of the pointer ring.
 *
 * A VM is known to the firmware as a pair of "slots", one for geometry and
 * compute jobs, one for fragment jobs: two device-control commands make the
 * firmware load the VM's page-table root for them. The firmware compares a
 * tag per slot with the one it saw last whenever a job starts and drops the
 * slot's translations if the host changed it.
 *
 * The parameter manager's state for one scene is a "PM set": three pages in
 * the firmware's GiB (control words and the scene's record, the list of the
 * pool's blocks, the free list), a pool of pages and the pages the PM keeps
 * its own tables in. Every VM maps all sets' three pages where the firmware
 * has them, protected as the firmware's are, because the PM reaches them
 * through the job's slot. A VM takes up to two sets, one per scene in the
 * firmware, with its first render.
 */

/* A channel page */
#define S8003_CH_CTL			0x100	/* watermarks, 16 bytes apart */
#define S8003_CH_CTL_HOST		0x40	/* the host's: how far the ring is filled */
#define S8003_CH_CTL_ENTRIES		0x50
#define S8003_CH_RING			0x200
#define S8003_CH_ENTRIES		16
#define S8003_CH_CMD			0x300
#define S8003_CH_W			0x400
#define S8003_CH_TA_USEQ		0xa00
#define S8003_CH_3D_USEQ		0xd00
#define S8003_CH_TQ			0x1800
#define S8003_CH_STAMP			0x1840	/* two words */
#define S8003_CH_DEP			0x1900
#define S8003_CH_SLOT1			0x1c00	/* every second job: command, W, microsequence */
#define S8003_CH_TIMES			0x2800	/* a job's two timestamps, 16 bytes per slot */

#define S8003_CMD_SIZE			0x40
#define S8003_TA_W_SIZE			0x560
#define S8003_3D_W_SIZE			0x8c0
#define S8003_TA_USEQ_SIZE		0xc0
#define S8003_3D_USEQ_SIZE		0x180

/* The compute channel: a quarter page, one job at a time */
#define S8003_CU_W_SIZE			0x290
#define S8003_CU_USEQ			0x800
#define S8003_CU_USEQ_SIZE		0xc0
#define S8003_CU_TQ			0x900
#define S8003_CU_STAMP			0x940	/* two words */
#define S8003_CU_DEP			0x980

/* Commands ahead of a render job's record */
#define S8003_CMD_WAIT			4	/* for a stamp to reach a number */
#define S8003_CMD_PM			6	/* the PM record a geometry job starts from */

#define S8003_DC_SLOT			3	/* (re)load a slot's page-table root */

#define S8003_JOB_TIMEOUT_US		5000000
#define S8003_RING_TIMEOUT_US		1000000

static u32 get32(const u8 *p)
{
	return get_unaligned_le32(p);
}

/* The firmware numbers the engines: geometry, fragment, compute */
static u32 s8003_engine(enum pvr_apple_engine engine)
{
	return engine == APPLE_ENGINE_GEOM ? 0 : engine == APPLE_ENGINE_FRAG ? 1 : 2;
}

static u32 s8003_channel_offset(const struct pvr_apple_s8003 *s, enum pvr_apple_engine engine)
{
	return s->v->channel[s8003_engine(engine)];
}

static u8 *s8003_channel(struct pvr_apple_s8003 *s, enum pvr_apple_engine engine)
{
	return (s->v->channels_root ? s->root.map : s->shared.map) + s8003_channel_offset(s, engine);
}

static u32 s8003_channel_addr(const struct pvr_apple_s8003 *s, enum pvr_apple_engine engine)
{
	return s8003_meta_addr((s->v->channels_root ? S8003_ROOT_VA : S8003_SHARED_VA) +
			       s8003_channel_offset(s, engine));
}

/* A channel before its first job: where its parts are, an empty ring */
static void s8003_channel_init(u8 *p, u32 addr, u32 tq, u32 dep)
{
	u32 off;

	put32(p, addr + S8003_CH_CTL);
	put32(p + 4, addr + S8003_CH_RING);
	put32(p + 8, addr + tq);
	put32(p + 0xc, 4);
	put32(p + 0x18, ~0u);
	put32(p + S8003_CH_CTL + S8003_CH_CTL_ENTRIES, S8003_CH_ENTRIES);
	put32(p + tq + 4, addr + tq);
	put64(p + dep + 0x30, 1);
	put64(p + dep + 0x48, ~0ull);
	put32(p + dep + 0x50, ~0u);
	for (off = 0x70; off < 0x270; off += 0x10)
		put32(p + dep + off + 8, 0xf);
}

static void s8003_channels_init(struct pvr_apple_s8003 *s)
{
	s8003_channel_init(s8003_channel(s, APPLE_ENGINE_GEOM),
			   s8003_channel_addr(s, APPLE_ENGINE_GEOM), S8003_CH_TQ, S8003_CH_DEP);
	s8003_channel_init(s8003_channel(s, APPLE_ENGINE_FRAG),
			   s8003_channel_addr(s, APPLE_ENGINE_FRAG), S8003_CH_TQ, S8003_CH_DEP);
	s8003_channel_init(s8003_channel(s, APPLE_ENGINE_COMPUTE),
			   s8003_channel_addr(s, APPLE_ENGINE_COMPUTE), S8003_CU_TQ, S8003_CU_DEP);
}

/* How often s8003_touch() runs while the firmware works for the host */
#define S8003_TOUCH_US			1000

static u32 pvr_apple_s8003_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine);

/*
 * The PM sets the firmware may be writing: those with a geometry or fragment
 * job not done yet, and those of the VMs whose scenes it handled last and
 * the one before, whose free lists the PM writes back when a later job runs.
 */
static unsigned long s8003_live_sets(struct pvr_apple_s8003 *s)
{
	u32 geom = pvr_apple_s8003_engine_done(s->apple, APPLE_ENGINE_GEOM);
	u32 frag = pvr_apple_s8003_engine_done(s->apple, APPLE_ENGINE_FRAG);
	unsigned long live = READ_ONCE(s->wb_sets);
	unsigned int set;

	BUILD_BUG_ON(S8003_PM_SETS > BITS_PER_LONG);
	for (set = 0; set < S8003_PM_SETS; set++) {
		u32 g = READ_ONCE(s->set_geom[set]), f = READ_ONCE(s->set_frag[set]);

		if ((g && (s32)(geom - g) < 0) || (f && (s32)(frag - f) < 0))
			live |= BIT(set);
	}
	return live & READ_ONCE(s->sets[0]);
}

/*
 * While a job runs the firmware does not see its own progress in the root and
 * shared memory until the host read those cache lines: without the reads a
 * phase waited for the firmware's next tick, seconds in all (measured on the
 * iPad 5: 7.9 s from input to frame, 0.27 s with them every 1 to 2 ms). One
 * load per 64-byte line, the PM sets the firmware may be writing included;
 * reading only those cut the time of the reads by a quarter to a third on
 * the G5P driver, with no slower phase.
 */
static u32 s8003_touch(struct pvr_apple_s8003 *s)
{
	unsigned long set, live = s8003_live_sets(s);
	u32 at;

	for (at = 0; at < S8003_ROOT_SIZE; at += 64)
		(void)READ_ONCE(*(const u32 *)(s->root.map + at));
	for (at = 0; at < S8003_SHARED_SIZE; at += 64)
		(void)READ_ONCE(*(const u32 *)(s->shared.map + at));
	for_each_set_bit(set, &live, S8003_PM_SETS)
		for (at = 0; at < S8003_PM_SET_SIZE; at += 64)
			(void)READ_ONCE(*(const u32 *)(s->pmsets.map + set * S8003_PM_SET_SIZE + at));
	return (S8003_ROOT_SIZE + S8003_SHARED_SIZE + hweight_long(live) * S8003_PM_SET_SIZE) / 64;
}

static unsigned long s8003_vm_sets(const struct s8003_vm *vm)
{
	unsigned long mask = 0;
	unsigned int i;

	for (i = 0; vm && i < ARRAY_SIZE(vm->pm); i++)
		if (vm->pm[i].set >= 0)
			mask |= BIT(vm->pm[i].set);
	return mask;
}

/* After @pm_last or @pm_prev changed; submit_lock held */
static void s8003_wb_sets_update(struct pvr_apple_s8003 *s)
{
	WRITE_ONCE(s->wb_sets, s8003_vm_sets(s->pm_last) | s8003_vm_sets(s->pm_prev));
}

/* What the firmware posted since the last look; wherever a job is waited for */
static void pvr_apple_s8003_events(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	ktime_t now = ktime_get();
	bool alive = false;

	if (!READ_ONCE(apple->booted) || !s->shared.map)
		return;
	/* An eighth early, so that a check every S8003_TOUCH_US always touches */
	if (s->v->touch &&
	    ktime_us_delta(now, READ_ONCE(s->touched)) + S8003_TOUCH_US / 8 >= S8003_TOUCH_US) {
		WRITE_ONCE(s->touched, now);
		s8003_touch(s);
	}
	if (!READ_ONCE(s->failed) && s8003_drain(apple, &alive)) {
		WRITE_ONCE(s->failed, true);
		dev_err(apple_dev(apple), "GFX firmware's rings are no longer read\n");
	}
	/* A job's stamp: its watermarks follow; look again soon */
	if (xchg(&s->stamped, false))
		pvr_apple_firmware_progress(apple);
}

static u32 s8003_submitted(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	return READ_ONCE(*(engine == APPLE_ENGINE_GEOM ? &apple->ta_seq :
			   engine == APPLE_ENGINE_FRAG ? &apple->frag_seq : &apple->last_seq));
}

/*
 * The last job of @engine that is done: both stamps at its number or past it
 * (the firmware only moves a stamp forward), and every watermark between its
 * end of the pointer ring and that of the last job submitted. With the
 * firmware off nothing is in flight.
 */
static u32 s8003_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 latest = s8003_submitted(apple, engine);
	u32 per_job = engine == APPLE_ENGINE_COMPUTE ? 1 : 2;
	u32 stamp = engine == APPLE_ENGINE_COMPUTE ? S8003_CU_STAMP : S8003_CH_STAMP;
	u32 done, end, span, off;
	const u8 *p;
	s32 behind;

	if (!READ_ONCE(apple->booted) || !s->shared.map)
		return latest;
	p = s8003_channel(s, engine);
	behind = min_t(s32, get32(p + stamp) - latest, get32(p + stamp + 4) - latest);
	done = latest + min(behind, 0);
	end = (per_job * done) % S8003_CH_ENTRIES;
	span = (per_job * (latest - done)) % S8003_CH_ENTRIES;
	for (off = 0; off <= S8003_CH_CTL_HOST; off += 0x10) {
		u32 mark = get32(p + S8003_CH_CTL + off);

		if (mark >= S8003_CH_ENTRIES ||
		    (mark + S8003_CH_ENTRIES - end) % S8003_CH_ENTRIES > span)
			return done - 1;
	}
	return done;
}

static u32 pvr_apple_s8003_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return s->v->engine_done(apple, engine);
}

static u32 s8003_done_poll(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	pvr_apple_s8003_events(apple);
	return pvr_apple_s8003_engine_done(apple, engine);
}

static int s8003_wait_done(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq)
{
	u32 done;
	int err;

	err = read_poll_timeout(s8003_done_poll, done,
				(s32)(done - seq) >= 0 || READ_ONCE(apple->fault), 20,
				S8003_JOB_TIMEOUT_US, false, apple, engine);
	if (!err && (s32)(done - seq) < 0) {
		pvr_apple_report_mmu_fault(apple);
		return -EIO;
	}
	return err;
}

/*
 * Wake the firmware's scheduler for @engine, as Apple's kext does after every
 * ring entry: Fender's wake, then the META's scheduler register. A core whose
 * power state is changing is not rung: the firmware looks at its rings when
 * it is done with that.
 */
static int s8003_doorbell(struct pvr_apple *apple, u32 dm)
{
	writel(FENDER_WAKE_ALL, apple->fender + FENDER_WAKE);
	if (readl(apple->fender + FENDER_WAKE) & ~FENDER_WAKE_ALL)
		return -EPROTO;
	if (!s8003_core_ready(apple))
		return 0;
	wmb();	/* what the firmware is rung for is in memory */
	pvr_cr_write32(apple->pvr_dev, ROGUE_CR_MTS_SCHEDULE,
		       ROGUE_CR_MTS_SCHEDULE_TASK_COUNTED | dm);
	return 0;
}

static u32 s8003_ring_consumer(struct pvr_apple *apple, u8 *ctl)
{
	pvr_apple_s8003_events(apple);
	return ring_read(ctl, S8003_RING_CONSUMER);
}

/*
 * Give the firmware @n device-control commands of two words and wait until
 * it took them.
 */
static int s8003_device_control(struct pvr_apple *apple, const u32 (*cmd)[2], unsigned int n)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_DC_CTL);
	u8 *ring = s8003_object(s, S8003_DC_RING);
	u32 producer = ring_read(ctl, S8003_RING_PRODUCER);
	u32 consumer, target;
	unsigned int i;
	int err;

	if (producer >= APPLE_RING_ENTRIES || ring_read(ctl, S8003_RING_CONSUMER) != producer)
		return -EPROTO;
	for (i = 0; i < n; i++) {
		u8 *entry = ring + ((producer + i) % APPLE_RING_ENTRIES) * S8003_DC_SIZE;

		memset(entry, 0, S8003_DC_SIZE);
		put32(entry, cmd[i][0]);
		put32(entry + 4, cmd[i][1]);
	}
	target = (producer + n) % APPLE_RING_ENTRIES;
	wmb();
	WRITE_ONCE(*(__le32 *)(ctl + S8003_RING_PRODUCER), cpu_to_le32(target));
	mb();	/* the index is in memory before the firmware is told */

	err = s->v->doorbell(apple, ROGUE_CR_MTS_SCHEDULE_DM_DM0);
	if (err)
		return err;
	return read_poll_timeout(s8003_ring_consumer, consumer, consumer == target, 100,
				 S8003_DC_TIMEOUT_US, false, apple, ctl);
}

/* Bump a loaded slot's tag with tag_lock held; leave empty slots at zero. */
static u64 s8003_slot_tag_bump(struct pvr_apple_s8003 *s, u32 slot)
{
	u32 tag = s->tag[slot], cached, next;

	lockdep_assert_held(&s->tag_lock);
	if (!tag)
		return 0;
	cached = get32(s8003_object(s, S8003_TAG_SEEN) + 4 * slot);
	next = pvr_apple_tag_bump(tag, S8003_TAG_CATALOGUE, cached);
	s->tag[slot] = next;
	put32(s8003_object(s, S8003_TAG_HOST) + 4 * slot, next);
	return (u32)(next - (tag + S8003_TAG_CATALOGUE)) / S8003_TAG_CATALOGUE;
}

/* Point a VM's slots (a pair, or one) at a page-table root (0: at none) under @tag */
static int s8003_slots_load(struct pvr_apple *apple, const u32 *slot, u64 root, u32 tag)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int e, n = s->v->vm_slots;
	const u32 cmd[2][2] = { { S8003_DC_SLOT, slot[n - 1] }, { S8003_DC_SLOT, slot[0] } };
	unsigned long flags;
	u64 steps = 0, total;
	int err;

	spin_lock_irqsave(&s->tag_lock, flags);
	for (e = 0; e < n; e++) {
		__set_bit(slot[e], s->loading);
		put64(s8003_object(s, S8003_MMU_ROOT) + 8 * slot[e], root);
		put32(s8003_object(s, S8003_TAG_HOST) + 4 * slot[e], tag);
		s->tag[slot[e]] = tag;
	}
	spin_unlock_irqrestore(&s->tag_lock, flags);

	err = s8003_device_control(apple, cmd, n);
	if (err)
		dev_err(apple_dev(apple), "GFX firmware slot load failed: %d\n", err);
	/* The firmware must copy the tag we held stable while loading the root. */
	for (e = 0; e < n && !err; e++) {
		u32 seen = get32(s8003_object(s, S8003_TAG_SEEN) + 4 * slot[e]);

		if (seen != tag) {
			dev_err(apple_dev(apple),
				"GFX firmware did not load slot %u: loaded tag %#x, firmware saw %#x, now %#x\n",
				slot[e], tag, seen, READ_ONCE(s->tag[slot[e]]));
			err = -EIO;
		}
	}
	/* The attach holds submit_lock: deferred flushes precede the VM's job. */
	spin_lock_irqsave(&s->tag_lock, flags);
	for (e = 0; e < n; e++) {
		__clear_bit(slot[e], s->loading);
		if (test_bit(slot[e], s->bump_pending)) {
			steps += s8003_slot_tag_bump(s, slot[e]);
			__clear_bit(slot[e], s->bump_pending);
		}
	}
	s->tag_steps_past += steps;
	total = s->tag_steps_past;
	spin_unlock_irqrestore(&s->tag_lock, flags);
	wmb();
	if (steps)
		dev_dbg_ratelimited(apple_dev(apple),
				    "MMU tag steps-past=%llu total=%llu\n", steps, total);
	return err;
}

/*
 * The slots of a VM for this start of the firmware. Slots are handed out
 * in turn, so that one a VM gave back is used again as late as possible; if
 * it was left on the park tables (pvr_apple_s8003_vm_fini()) it is emptied
 * first, as Apple's kext does when an address space goes.
 */
static int s8003_vm_attach(struct pvr_apple *apple, struct s8003_vm *vm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	const struct s8003_variant *v = s->v;
	bool parked = false;
	unsigned int e;
	int err = 0;

	lockdep_assert_held(&apple->submit_lock);
	if (vm->attached == s->gen)
		return 0;
	for (e = v->vm_slots; e-- > 0 && !vm->slot[e];) {
		u32 slot = find_next_zero_bit(s->slots, v->slots, s->next_slot);

		if (slot >= v->slots)
			slot = find_next_zero_bit(s->slots, v->slots, v->slot_first);
		if (slot >= v->slots) {
			if (vm->slot[1])
				__clear_bit(vm->slot[1], s->slots);
			vm->slot[1] = 0;
			return -EBUSY;
		}
		__set_bit(slot, s->slots);
		s->next_slot = slot + 1 < v->slots ? slot + 1 : v->slot_first;
		vm->slot[e] = slot;
	}
	/* No job runs while a slot changes */
	err = pvr_apple_wait_idle(apple, S8003_JOB_TIMEOUT_US);
	for (e = 0; e < v->vm_slots; e++)
		parked |= test_bit(vm->slot[e], s->parked);
	if (!err && parked) {
		err = s8003_slots_load(apple, vm->slot, 0, 0);
		for (e = 0; e < v->vm_slots; e++)
			__clear_bit(vm->slot[e], s->parked);
	}
	if (!err)
		err = s8003_slots_load(apple, vm->slot,
				       pvr_vm_get_page_table_root_addr(vm->vm_ctx), S8003_TAG_FIRST);
	if (!err)
		vm->attached = s->gen;
	return err;
}

/*
 * Page tables changed: move the tag of every slot in use, in the field that
 * makes the firmware drop the slot's translations down from the catalogue
 * when its next job starts. Never to 0, the tag of an empty slot.
 */
static void pvr_apple_s8003_mmu_flush(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned long flags;
	u64 steps = 0, total;
	u32 slot;

	if (!s->shared.map)
		return;
	spin_lock_irqsave(&s->tag_lock, flags);
	for (slot = s->v->slot_first; slot < s->v->slots; slot++) {
		if (test_bit(slot, s->loading))
			__set_bit(slot, s->bump_pending);
		else
			steps += s8003_slot_tag_bump(s, slot);
	}
	s->tag_steps_past += steps;
	total = s->tag_steps_past;
	spin_unlock_irqrestore(&s->tag_lock, flags);
	wmb();
	if (steps)
		dev_dbg_ratelimited(apple_dev(apple),
				    "MMU tag steps-past=%llu total=%llu\n", steps, total);
}

/* The PM reaches its sets through the job's slot, at the firmware's addresses */
static int s8003_vm_map(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (!s->pmsets.obj)
		return -ENODEV;
	return pvr_vm_map_reserved(vm_ctx, s->pmsets.obj, S8003_PM_SETS_VA, s->pmsets.size);
}

static int pvr_apple_s8003_vm_init(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm;
	int err;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;
	vm->vm_ctx = vm_ctx;
	mutex_init(&vm->prepare_lock);
	vm->pm[0].set = -1;
	vm->pm[1].set = -1;
	err = s->v->vm_map(apple, vm_ctx);
	if (!err)
		err = xa_err(xa_store(&s->vms, (unsigned long)vm_ctx, vm, GFP_KERNEL));
	if (err)
		kfree(vm);
	else
		atomic_inc(&s->vms_live);
	return err;
}

/*
 * How many scenes of a VM the firmware may hold: as many as the VM has PM
 * sets. Reserve the first here, and the second while more than half remain.
 * IOCTL context: arming counts only sets whose resources are fully ready.
 */
static unsigned int s8003_vm_reserve_sets(struct pvr_apple *apple,
					      struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm = xa_load(&s->vms, (unsigned long)vm_ctx);
	unsigned int i, n = 0;
	unsigned long flags;

	if (!vm)
		return 1;
	spin_lock_irqsave(&s->set_lock, flags);
	for (i = 0; i < 2; i++) {
		unsigned long set;

		if (vm->pm[i].set >= 0) {
			n++;
			continue;
		}
		if (i && bitmap_weight(s->sets, S8003_PM_SETS) >= S8003_PM_SETS / 2)
			break;
		/* In turn: the PM may still write a set a VM gave back */
		set = find_next_zero_bit(s->sets, S8003_PM_SETS, s->next_set);
		if (set >= S8003_PM_SETS)
			set = find_first_zero_bit(s->sets, S8003_PM_SETS);
		if (set >= S8003_PM_SETS)
			break;
		__set_bit(set, s->sets);
		s->next_set = (set + 1) % S8003_PM_SETS;
		vm->pm[i].set = set;
		n++;
	}
	spin_unlock_irqrestore(&s->set_lock, flags);
	return max(n, 1u);
}

static unsigned int pvr_apple_s8003_vm_scenes(struct pvr_apple *apple,
					      struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm = xa_load(&s->vms, (unsigned long)vm_ctx);
	unsigned long flags;
	unsigned int n = 0, i;

	if (!vm)
		return 1;
	spin_lock_irqsave(&s->set_lock, flags);
	for (i = 0; i < ARRAY_SIZE(vm->pm); i++)
		if (vm->pm[i].set >= 0 && vm->pm[i].ready)
			n++;
	spin_unlock_irqrestore(&s->set_lock, flags);
	return max(n, 1U);
}

static void s8003_pm_put(struct pvr_apple_s8003 *s, struct s8003_pm *pm)
{
	unsigned long flags;

	spin_lock_irqsave(&s->set_lock, flags);
	if (pm->set >= 0)
		__clear_bit(pm->set, s->sets);
	spin_unlock_irqrestore(&s->set_lock, flags);
	if (pm->pool) {
		pvr_gem_object_vunmap(pm->pool);
		pvr_gem_object_put(pm->pool);
	}
	if (pm->next_pool) {
		pvr_gem_object_vunmap(pm->next_pool);
		pvr_gem_object_put(pm->next_pool);
	}
	if (pm->aux) {
		pvr_gem_object_vunmap(pm->aux);
		pvr_gem_object_put(pm->aux);
	}
}

/*
 * A VM goes; its jobs are done. The PM still writes the free list of the VM
 * whose scene it handled last, or the one before, through that VM's geometry
 * slot when a later job runs (seen on the iPad 5 as page faults in a closed
 * context's slot). So the slots are not emptied but put on the park VM's
 * tables, which map the PM sets and nothing else, and the sets are taken
 * again as late as possible.
 */
static void pvr_apple_s8003_vm_fini(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm = xa_erase(&s->vms, (unsigned long)vm_ctx);
	unsigned int e;

	if (!vm)
		return;
	mutex_lock(&apple->submit_lock);
	if (apple->booted && vm->attached == s->gen && s->park_vm && vm_ctx != s->park_vm &&
	    !pvr_apple_wait_idle(apple, S8003_JOB_TIMEOUT_US) &&
	    !s8003_slots_load(apple, vm->slot, pvr_vm_get_page_table_root_addr(s->park_vm),
			      S8003_TAG_PARK)) {
		for (e = 0; e < s->v->vm_slots; e++)
			__set_bit(vm->slot[e], s->parked);
	}
	if (s->pm_last == vm)
		s->pm_last = NULL;
	if (s->t7000_geom_vm == vm)
		s->t7000_geom_vm = NULL;
	if (s->pm_prev == vm)
		s->pm_prev = NULL;
	s8003_wb_sets_update(s);
	for (e = 0; e < 2; e++) {
		if (vm->slot[e])
			__clear_bit(vm->slot[e], s->slots);
	}
	s->v->vm_put(apple, vm);
	mutex_unlock(&apple->submit_lock);
	if (s->v->vm_unmap)
		s->v->vm_unmap(apple, vm_ctx);
	atomic_dec(&s->vms_live);
	kfree(vm);
}

/* A VM's PM sets go with it */
static void s8003_vm_put(struct pvr_apple *apple, struct s8003_vm *vm)
{
	unsigned int e;

	for (e = 0; e < 2; e++)
		s8003_pm_put(apple->chip_data, &vm->pm[e]);
}

static u64 s8003_pm_set_va(const struct s8003_pm *pm, u32 page)
{
	return S8003_PM_SETS_VA + pm->set * S8003_PM_SET_SIZE + page * PVR_DEVICE_PAGE_SIZE;
}

static u64 s8003_pm_aux_va(const struct s8003_pm *pm)
{
	return S8003_PM_AUX_VA + (u64)pm->set * S8003_PM_AUX_STRIDE;
}

/*
 * A PM set for @vm with a pool of @blocks blocks, built anew for this start
 * of the firmware: the pool's pages are 16 KiB pages of ours, listed per
 * block (eight pages each, the blocks linked by their META addresses) and
 * once more as the free list, four 4 KiB page numbers a page; the scene's
 * record names the PM's own tables (their GPU addresses in the VM) and is
 * otherwise empty. The pool is not rewritten between jobs: each geometry
 * job's command has the firmware rebuild the free list from the block list.
 */
static int s8003_pm_build(struct pvr_apple *apple, struct s8003_vm *vm, struct s8003_pm *pm,
			  u32 blocks)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *control, *list, *free, *record;
	u32 previous = 0, i, j, k;
	u64 list_va, aux_va;
	int err;

	if (!pm->aux || !pm->aux_map)
		return -ENOMEM;
	if (pm->built == s->gen && pm->blocks >= blocks)
		return 0;
	if (pm->blocks < blocks) {
		if (!pm->next_pool || pm->next_blocks < blocks)
			return -ENOMEM;
		/* This set is idle. Its previous pool can now be retired. */
		if (pm->pool) {
			pvr_gem_object_vunmap(pm->pool);
			pvr_gem_object_put(pm->pool);
		}
		pm->pool = pm->next_pool;
		pm->pool_map = pm->next_pool_map;
		pm->blocks = pm->next_blocks;
		pm->next_pool = NULL;
		pm->next_pool_map = NULL;
		pm->next_blocks = 0;
	}
	pm->built = 0;
	pm->rec_seq = 0;
	memset(pm->pool_map, 0, pvr_gem_object_size(pm->pool));
	memset(pm->aux_map, 0, S8003_PM_AUX_SIZE);
	dma_sync_sgtable_for_device(apple_dev(apple), pm->pool->base.sgt, DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_device(apple_dev(apple), pm->aux->base.sgt, DMA_BIDIRECTIONAL);

	control = s->pmsets.map + pm->set * S8003_PM_SET_SIZE;
	list = control + PVR_DEVICE_PAGE_SIZE;
	free = list + PVR_DEVICE_PAGE_SIZE;
	list_va = s8003_pm_set_va(pm, 1);
	aux_va = s8003_pm_aux_va(pm);
	memset(control, 0, S8003_PM_SET_SIZE);
	for (j = 0; j < pm->blocks; j++) {
		u32 at = 0x80 + j * 0x24;

		put32(list + at, previous);
		for (k = 0; k < 8; k++) {
			dma_addr_t page;
			u32 ppn;

			err = pvr_gem_get_dma_addr(pm->pool, (j * 8 + k) * PVR_DEVICE_PAGE_SIZE,
						   &page);
			if (err)
				return err;
			ppn = page >> 12;
			put32(list + at + 4 + k * 4, ppn);
			for (i = 0; i < 4; i++)
				put32(free + ((j * 8 + k) * 4 + i) * 4, ppn + i);
		}
		previous = s8003_meta_addr(list_va + at);
	}
	put32(list + 0x60, 1);
	put32(control + 0x10, previous);
	put64(control + 0x14, s8003_pm_set_va(pm, 2));
	put32(control + 0x1c, pm->blocks * 32 * 4);
	put32(control + 0x20, pm->blocks * 32);
	put32(control + 0x24, pm->blocks * 32 - 1);
	put32(control + 0x34, PVR_DEVICE_PAGE_SIZE);
	put32(control + 0x40, s8003_meta_addr(list_va));
	put32(control + 0x54, pm->blocks * 32);
	put32(control + 0x58, pm->blocks * 32);
	/* The PM's tables: macrotile array, MList, VHEAP */
	record = control + S8003_PM_RECORD;
	put64(record + 0xa8, aux_va);
	put64(record + 0xa0, aux_va + 0x80);
	put64(record + 0xb0, aux_va + 0x80 + 0x10000);
	put32(record + 0xb8, s8003_meta_addr(list_va + 0x40));
	wmb();
	pm->built = s->gen;
	return 0;
}

/*
 * A render target's geometry as the firmware takes it with each job: Apple's
 * kext computes it from the size and the samples (AGXRenderTargetMemoryG5),
 * in the Rogue parameter format with 32x32 tiles, 4x4 macrotiles, five-byte
 * region headers per padded tile and a tail pointer cache of
 * next_pow2(padded tiles)^2 entries of eight bytes. Integer arithmetic for
 * the kext's three pairs of single-precision scale values.
 */
struct s8003_rt {
	u32 header;		/* bytes of region headers */
	u32 region;
	u64 positions;		/* the standard sample positions */
	u32 state, padded;
	u64 scale[3];
	u64 ta_state;
	u64 ta_tiling[2];
	u32 ta_tiling_word, tiles;
	u64 tpc;		/* bytes of the tail pointer cache */
};

#define S8003_RT_MAX_DIM	8192
#define S8003_RT_SCALE_A	0x3e89309cu
#define S8003_RT_SCALE_B	0x3fddb3d9u
#define S8003_POSITIONS_1X	0x88ull
#define S8003_POSITIONS_4X	0xeaa26e26ull

/* The single-precision float of 1 <= n < 2^24 */
static u32 s8003_f32(u32 n)
{
	u32 bits = fls(n);

	return ((126 + bits) << 23) | ((n << (24 - bits)) & 0x7fffff);
}

/* A normal positive single divided by 1 <= n < 2^24, rounded to nearest even */
static u32 s8003_fdiv(u32 a, u32 n)
{
	u32 shift = fls(n) + 3;
	u64 num = (u64)((a & 0x7fffff) | 0x800000) << shift;
	u32 q = div_u64(num, n), sticky = (u64)q * n != num;
	u32 s = fls(q) - 24, m = q >> s, low = q & ((1u << s) - 1);

	if (low > 1u << (s - 1) || (low == 1u << (s - 1) && (sticky || (m & 1))))
		m++;
	if (m >> 24) {
		m >>= 1;
		s++;
	}
	return ((((a >> 23) & 0xff) - shift + s) << 23) | (m & 0x7fffff);
}

static int s8003_rt_derive(const struct pvr_hwrt_dataset *hwrt, struct s8003_rt *rt)
{
	u32 width = hwrt->width, height = hwrt->height;
	u32 tx = (width + 31) / 32, ty = (height + 31) / 32;
	u32 mx = ALIGN((tx + 3) >> 2, 4), my = ALIGN((ty + 3) >> 2, 4);
	u32 s = hwrt->samples == 4 ? 2 : 1;
	u32 px = mx * s * 4, py = my * s * 4, tpc;

	if (!width || !height || width > S8003_RT_MAX_DIM || height > S8003_RT_MAX_DIM ||
	    (hwrt->samples != 1 && hwrt->samples != 4))
		return -EINVAL;
	tpc = roundup_pow_of_two(max(px, py));
	*rt = (struct s8003_rt) {
		.header = px * py * 5,
		.region = mx * my * s * s * 5,
		.positions = s == 2 ? S8003_POSITIONS_4X : S8003_POSITIONS_1X,
		.state = s == 2 ? 2 : 0,
		.padded = (mx & 0x3ff) << 16 | ((my * s) & 0x3ff),
		.scale = {
			s8003_fdiv(S8003_RT_SCALE_A, width) |
				(u64)s8003_fdiv(S8003_RT_SCALE_A, height) << 32,
			s8003_fdiv(S8003_RT_SCALE_B, width) |
				(u64)s8003_fdiv(S8003_RT_SCALE_B, height) << 32,
			(s8003_f32(width) + (4u << 23)) |
				(u64)(s8003_f32(height) + (4u << 23)) << 32,
		},
		.ta_state = s == 2 ? 5 : 0,
		.ta_tiling = {
			(u64)((ty - 1) << 12 | (tx - 1)) << 32 | ((height - 1) << 16 | (width - 1)),
			(u64)(my << 18 | my << 10 | 3 * my) << 32 | (mx << 18 | mx << 10 | 3 * mx),
		},
		.ta_tiling_word = mx * my,
		.tiles = tx * ty,
		.tpc = (u64)tpc * tpc * 8,
	};
	/*
	 * What userspace sized its buffers for: the dataset's region header
	 * size is the ISP region size in dwords (pvr_drm.h), the region
	 * headers of one of the 16 macrotiles; its tail pointer cache size is
	 * in bytes.
	 */
	if (hwrt->common.rgn_header_size < rt->region / 4 || hwrt->common.tpc_size < rt->tpc)
		return -EINVAL;
	return 0;
}

/*
 * A geometry job's work record: the words Apple's kext sets the same for
 * every render, then userspace's (stream extension), then the target's.
 */
static void s8003_fill_ta(u8 *w, const struct pvr_apple_g5p_cmd_geom *cmd,
			  const struct pvr_hwrt_data *hwrt, const struct s8003_rt *rt)
{
	static const u32 fixed[][2] = {
		{ 0x020, 0x1e3ce508 }, { 0x024, 0x1e3ce508 }, { 0x3a4, 0x00000088 },
		{ 0x3a8, 0x00000002 }, { 0x3c0, 0x00020000 }, { 0x514, 0x08002358 },
		{ 0x528, 0x00000001 },
	};
	const struct pvr_apple_g5p_geom_regs *g = &cmd->g5p;
	u64 tpc = hwrt->data.tail_ptrs_dev_addr;
	unsigned int i;

	memset(w, 0, S8003_TA_W_SIZE);
	for (i = 0; i < ARRAY_SIZE(fixed); i++)
		put32(w + fixed[i][0], fixed[i][1]);
	put64(w + 0x48, g->scratch_ptr[4]);
	put64(w + 0x50, g->scratch_ptr[5]);
	for (i = 0; i < 6; i++) {
		put32(w + 0x58 + i * 4, g->context_pds[0][i]);
		put32(w + 0x70 + i * 4, g->context_pds[1][i]);
	}
	put64(w + 0x88, g->ta_program);
	put64(w + 0x90, g->scratch_base & ~7ull);
	put64(w + 0x98, cmd->cmd.regs.vdm_ctrl_stream_base & ~3ull);
	for (i = 0; i < 4; i++) {
		put32(w + 0xa0 + i * 4, g->resume_pds[0][i]);
		put32(w + 0xb0 + i * 4, g->resume_pds[1][i]);
	}
	for (i = 0; i < 3; i++)
		put32(w + 0xc0 + i * 4, g->ta_pds[i]);
	for (i = 0; i < 4; i++)
		put64(w + 0x468 + i * 8, g->scratch_ptr[i]);
	put64(w + 0x488, g->scratch_ptr[4]);

	put64(w + 0x28, hwrt->data.rgn_header_dev_addr);
	put64(w + 0x30, tpc);
	put64(w + 0x38, rt->ta_state);
	put64(w + 0x40, g->sample_positions ?: rt->positions);
	put64(w + 0x444, tpc);
	put32(w + 0x44c, rt->tpc);
	put64(w + 0x3ac, rt->ta_tiling[0]);
	put64(w + 0x3b4, rt->ta_tiling[1]);
	put32(w + 0x3bc, rt->ta_tiling_word);
}

/* The pixel output's partition words for @width registers and @shift (10, 11 with 4 samples) */
static u32 s8003_fill_output(u8 *w, u32 width, u32 shift)
{
	/* Apple's code per width: 2, 4, 8 */
	u32 code = width == 4 ? 0x10 : width == 8 ? 0x20 : 0;
	u32 count = 0x4000 / (width << shift);
	unsigned int i;

	put32(w + 0x3c8, count & 0x1f);
	for (i = 0; i < 6; i++)
		put64(w + 0x108 + i * 8, code | (count - 1) << 7 | 0x43);
	return count;
}

static int s8003_fill_3d(u8 *w, const struct pvr_apple_g5p_cmd_frag *cmd,
			 const struct pvr_hwrt_data *hwrt, const struct s8003_rt *rt)
{
	static const u32 fixed[][2] = {
		{ 0x108, 0x000003c3 }, { 0x110, 0x000003c3 }, { 0x118, 0x000003c3 },
		{ 0x120, 0x000003c3 }, { 0x128, 0x000003c3 }, { 0x130, 0x000003c3 },
		{ 0x3a4, 0x3f800000 }, { 0x3a8, 0x00000300 }, { 0x3b0, 0x0000001f },
		{ 0x3c0, 0x0023000f }, { 0x3c4, 0x00000408 }, { 0x3c8, 0x00000008 },
		{ 0x3cc, 0x000000f8 }, { 0x6a8, 0x3f800000 }, { 0x6ac, 0x00000300 },
		{ 0x6b0, 0x00000002 }, { 0x868, 0x08002358 },
	};
	/* The record's two places for each ZLS register image */
	static const u16 zls[6][2] = {
		{ 0x80, 0x618 }, { 0x88, 0x628 }, { 0x90, 0x630 },
		{ 0x98, 0x648 }, { 0xa0, 0x650 }, { 0x138, 0x6d0 },
	};
	const struct rogue_fwif_frag_regs *regs = &cmd->cmd.regs;
	const struct pvr_apple_g5p_frag_regs *g = &cmd->g5p;
	bool resolve = g->flags & PVR_APPLE_G5P_FRAG_MSAA_RESOLVE;
	bool msaa = hwrt->hwrt_dataset->samples == 4;
	u64 positions = g->sample_positions ?: rt->positions;
	u32 width = g->pixel_output_width ?: 2;
	unsigned int i;

	if ((width != 2 && width != 4 && width != 8) ||
	    (g->flags & ~PVR_APPLE_G5P_FRAG_MSAA_RESOLVE) || (resolve && !msaa))
		return -EINVAL;

	memset(w, 0, S8003_3D_W_SIZE);
	for (i = 0; i < ARRAY_SIZE(fixed); i++)
		put32(w + fixed[i][0], fixed[i][1]);
	for (i = 0; i < 8; i++)
		put32(w + 0x5b8 + i * 4, g->background[i]);
	put32(w + 0xb0, g->background[0]);
	put32(w + 0xb4, g->background[1]);
	for (i = 4; i < 8; i++)
		put32(w + 0xc0 + (i - 4) * 4, g->background[i]);
	/* The kext's upload has the halves of the first end-of-tile pair swapped */
	put32(w + 0x3b4, g->eot[1]);
	put32(w + 0x3b8, g->eot[0]);
	put32(w + 0x3bc, g->eot[2]);
	for (i = 0; i < 3; i++) {
		put32(w + 0x680 + i * 4, g->eot[i]);
		put32(w + 0x694 + i * 4, g->eot_resume[i]);
	}
	put32(w + 0x600, g->restore[0]);
	put32(w + 0x604, g->restore[1]);
	put32(w + 0x610, g->restore[2]);
	put32(w + 0x614, g->restore[3]);

	put64(w + 0x60, regs->isp_scissor_base & ~3ull);
	put64(w + 0x4a0, regs->isp_scissor_base);
	put64(w + 0x68, regs->isp_dbias_base & ~3ull);
	put64(w + 0x490, regs->isp_dbias_base);
	put64(w + 0x70, regs->isp_oclqry_base);
	put64(w + 0x78, regs->isp_oclqry_base);
	put64(w + 0xb8, g->background_texture);
	put64(w + 0x5f0, g->background_texture);
	put32(w + 0x3a4, regs->isp_bgobjdepth);
	put32(w + 0x6a8, regs->isp_bgobjdepth);
	put32(w + 0x3a8, regs->isp_bgobjvals);
	put32(w + 0x6ac, regs->isp_bgobjvals);
	for (i = 0; i < 6; i++) {
		put64(w + zls[i][0], g->zls[i]);
		put64(w + zls[i][1], g->zls[i]);
	}
	put32(w + 0x3cc, (width == 4 ? 1 : width == 8 ? 2 : 0) | 0xf8);
	put32(w + 0x6b0, width);
	s8003_fill_output(w, width, 10);

	put64(w + 0x20, hwrt->data.rgn_header_dev_addr);
	put64(w + 0x28, rt->positions);
	put32(w + 0x30, rt->state);
	put32(w + 0x34, rt->padded);
	put32(w + 0x38, rt->region);
	put64(w + 0x3c, rt->scale[0]);
	put64(w + 0x44, rt->scale[1]);
	put64(w + 0x4c, rt->scale[2]);
	put64(w + 0x58, positions);
	put32(w + 0xa8, rt->region >> 2);
	put32(w + 0x3a0, rt->padded);
	put64(w + 0x3d0, rt->scale[0]);
	put64(w + 0x3d8, rt->scale[1]);
	put32(w + 0x3e0, lower_32_bits(rt->scale[2]));
	put32(w + 0x3e4, lower_32_bits(rt->scale[0]));

	/*
	 * Four samples: the AA mode, ISP_CTL without the last pass (and with
	 * the resolve), and half the output partitions for the width.
	 */
	if (msaa) {
		u32 count = s8003_fill_output(w, width, 11);
		u32 mode = min(count, 2u);

		put32(w + 0x3c0, (0x0023000f & ~(0xfu | BIT(21))) | 0x7 |
				 (resolve ? BIT(17) : 0) | (mode - 1) << 21);
		put32(w + 0x3c4, mode * 0x204);
		put64(w + 0xd0, rt->state);
		put32(w + 0x86c, resolve);
	}
	return 0;
}

/* Where job @seq of a render channel has its command, record and microsequence */
struct s8003_job_slot {
	u32 cmd, w, useq, times;
};

static struct s8003_job_slot s8003_slot_of(u32 seq, bool frag)
{
	if (seq & 1)
		return (struct s8003_job_slot) {
			S8003_CH_CMD, S8003_CH_W, frag ? S8003_CH_3D_USEQ : S8003_CH_TA_USEQ,
			S8003_CH_TIMES,
		};
	return (struct s8003_job_slot) {
		S8003_CH_SLOT1, S8003_CH_SLOT1 + S8003_CMD_SIZE,
		S8003_CH_SLOT1 + S8003_CMD_SIZE + S8003_3D_W_SIZE, S8003_CH_TIMES + 16,
	};
}

/* What the 3D engine is waited for between a fragment job's start and finish */
static const u32 s8003_3d_wait[] = {
	0x201,	    0x138204,  0x10,   0x8388204, 0,	  0x104000a, 0x1f,
	0x1f,	    0x8388204, 1,      0x104000a, 0x1f,	  0x1f,	     0xf48008,
	0x10000015, 4,	       0,      0x14,	  0x109f, 0x13000a,  0x4000,
	0x4000,	    0x138204,  0x4000, 0x8388204, 0,	  0x104000a, 8,
	8,	    0x8388204, 1,      0x104000a, 8,	  8,
};

static u8 *useq_put(u8 *out, const u32 *words, unsigned int n)
{
	for (; n--; out += 4)
		put32(out, *words++);
	return out;
}

/*
 * The parts of render job @seq that are the kernel's, around the record's
 * image at the job's slot of the channel page @p: the record's links (slot,
 * dependency object, PM set and its scene record, microsequence, stamps), the
 * microsequence, the command and both pointer-ring entries. @pm is the PM
 * set's META address, @pm_id its number. Without @pm_record a geometry job
 * waits for its set's last record-carrying TA, already complete. This is
 * AGXTACommandDescriptor_preparePMAndWork (0xfffffff006cb9afc): a steady TA
 * takes the PM stamp set at +0x40; a rebuild takes the full set. Backing is
 * rebuilt only on a generation change (0xfffffff006cb1940). A fragment job
 * waits at @wait_stamp for @wait_seq, its geometry job's stamp.
 */
static void s8003_job_write(u8 *p, u32 addr, u32 global, u32 pm, u32 pm_id, u32 slot, bool frag,
			    u32 seq, u32 wait_stamp, u32 wait_seq, bool pm_record)
{
	struct s8003_job_slot at = s8003_slot_of(seq, frag);
	u32 w = addr + at.w, record = pm + S8003_PM_RECORD;
	u32 stamp = addr + S8003_CH_STAMP, times = addr + at.times;
	/* In the record: the microsequence's result, and where its timestamps go */
	u32 result = w + (frag ? 0x8ac : 0x550), time = w + (frag ? 0x8b4 : 0x558);
	u32 ring = (2 * (seq - 1)) % S8003_CH_ENTRIES;
	u8 *work = p + at.w, *cmd = p + at.cmd, *useq = p + at.useq;
	const u32 time_start[] = { 0x80000018, result, time, time, seq };
	const u32 time_end[] = { 0x18, result, time, time + 4, seq };
	const u32 wait_ta = 0x801, end = 0x40000017;

	put32(work + (frag ? 0x8ac : 0x550), 0);
	put32(work + (frag ? 0x8b0 : 0x554), 0);
	put32(work + (frag ? 0x8b4 : 0x558), times);
	put32(work + (frag ? 0x8b8 : 0x55c), times + 8);
	memset(p + at.times, 0, 16);
	memset(useq, 0, frag ? S8003_3D_USEQ_SIZE : S8003_TA_USEQ_SIZE);
	put32(work, frag);
	put32(work + 4, slot);
	put32(work + (frag ? 0x14 : 0xc), addr + S8003_CH_DEP);
	put32(work + (frag ? 0x18 : 0x14), pm);
	if (!frag)
		put32(work + 0x10, pm_id);
	put32(work + (frag ? 0x1c : 0x18), record);
	put32(work + (frag ? 0xc : 0x454), addr + at.useq);
	put32(work + (frag ? 0x10 : 0x458), frag ? S8003_3D_USEQ_SIZE : S8003_TA_USEQ_SIZE);
	put32(work + (frag ? 0x888 : 0x52c), stamp);
	put32(work + (frag ? 0x88c : 0x530), stamp + 4);
	put32(work + (frag ? 0x890 : 0x534), seq);
	put32(work + (frag ? 0x894 : 0x538), frag);
	put32(work + (frag ? 0x898 : 0x53c), frag);
	put32(work + (frag ? 0x8a0 : 0x544), seq);

	if (frag) {
		const u32 start[] = {
			0x25, w + 0x3a0, w + 0x58, record, global + 0x724, w + 0x7f0, w + 0x7f4,
			w + 0x878, addr, w, slot, 0, pm_id, get32(work + 0x800), 0, 1, w + 0x6d8,
			get32(work + 0x87c), get32(work + 0x884), 0, seq,
		};
		const u32 finish[] = {
			0x26, seq, stamp + 4, seq, record, pm, 1, global + 0x724, w + 0x878,
			w + 0x7f0, addr, w, slot, w + 0x6d8, 0, get32(work + 0x87c),
			get32(work + 0x884), (u32)-0x104,
		};

		useq = useq_put(useq, start, ARRAY_SIZE(start));
		useq = useq_put(useq, time_start, ARRAY_SIZE(time_start));
		useq = useq_put(useq, s8003_3d_wait, ARRAY_SIZE(s8003_3d_wait));
		useq = useq_put(useq, time_end, ARRAY_SIZE(time_end));
		useq = useq_put(useq, finish, ARRAY_SIZE(finish));
	} else {
		const u32 start[] = {
			0x23, w + 0x3a4, w + 0x20, record, global + 0x724, addr, slot, 0, pm_id,
			0, 0, 0, w + 0x464, get32(work + 0x51c), get32(work + 0x520), 0, seq,
		};
		const u32 finish[] = {
			0x24, record, pm, global + 0x724, addr, slot, 0, w + 0x464,
			get32(work + 0x51c), get32(work + 0x520), seq, stamp + 4, seq,
			(u32)-0x70,
		};

		useq = useq_put(useq, start, ARRAY_SIZE(start));
		useq = useq_put(useq, time_start, ARRAY_SIZE(time_start));
		useq = useq_put(useq, &wait_ta, 1);
		useq = useq_put(useq, time_end, ARRAY_SIZE(time_end));
		useq = useq_put(useq, finish, ARRAY_SIZE(finish));
	}
	useq_put(useq, &end, 1);

	memset(cmd, 0, S8003_CMD_SIZE);
	if (frag) {
		put32(cmd, S8003_CMD_WAIT);
		put32(cmd + 4, wait_stamp);
		put32(cmd + 8, wait_seq);
		put32(cmd + 12, wait_seq & 0x7fffffff);
	} else if (pm_record) {
		put32(cmd, S8003_CMD_PM);
		put32(cmd + 4, slot);
		put32(cmd + 8, pm_id);
		put32(cmd + 12, pm);
	} else {
		put32(cmd, S8003_CMD_WAIT);
		put32(cmd + 4, wait_stamp);
		put32(cmd + 8, wait_seq);
		put32(cmd + 12, wait_seq & 0x7fffffff);
	}
	put32(p + S8003_CH_RING + ring * 4, addr + at.cmd);
	put32(p + S8003_CH_RING + ((ring + 1) % S8003_CH_ENTRIES) * 4, w);
}

/*
 * Tell the firmware that @engine's pointer ring is filled up to @end: an
 * entry of the engine's work ring, the host's watermark, the work ring's
 * producer index, the doorbell. The doorbell is rung unless the job's own
 * stamps show it done: ringing for an entry the firmware took already adds
 * no work, and a job that was not rung for waited until something else woke
 * the firmware (a second of silence with two jobs in flight, then a lost
 * job, on the iPad 5).
 */
static int s8003_publish(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq, u32 end)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 fw_engine = s8003_engine(engine);
	u8 *ctl = s8003_object(s, S8003_WORK_CTL + 4 * fw_engine);
	u8 *ring = s8003_object(s, S8003_WORK_RING + 4 * fw_engine);
	u32 stamp = engine == APPLE_ENGINE_COMPUTE ? S8003_CU_STAMP : S8003_CH_STAMP;
	u8 *p = s8003_channel(s, engine), *entry;
	u32 start = ring_read(ctl, S8003_RING_PRODUCER), consumer;
	int err;

	if (start >= APPLE_RING_ENTRIES)
		return -EPROTO;
	/* The firmware takes a work-ring entry at once; the ring never holds two */
	err = read_poll_timeout(s8003_ring_consumer, consumer, consumer == start, 20,
				S8003_RING_TIMEOUT_US, false, apple, ctl);
	if (err)
		return err;
	entry = ring + start * S8003_DC_SIZE;
	memset(entry, 0, S8003_DC_SIZE);
	put32(entry, fw_engine);
	put32(entry + 4, s8003_channel_addr(s, engine));
	put32(entry + 8, end);
	wmb();
	put32(p + S8003_CH_CTL + S8003_CH_CTL_HOST, end);
	wmb();
	WRITE_ONCE(*(__le32 *)(ctl + S8003_RING_PRODUCER),
		   cpu_to_le32((start + 1) % APPLE_RING_ENTRIES));
	mb();
	if ((s32)(get32(p + stamp) - seq) >= 0 && (s32)(get32(p + stamp + 4) - seq) >= 0)
		return 0;
	/* Compute has no doorbell of its own in Apple's kext: the geometry engine's wakes the scheduler */
	return s8003_doorbell(apple, engine == APPLE_ENGINE_FRAG ? ROGUE_CR_MTS_SCHEDULE_DM_DM1 :
								    ROGUE_CR_MTS_SCHEDULE_DM_DM0);
}

/* Whether a PM set holds no scene the firmware still works on */
static bool s8003_pm_idle(struct pvr_apple *apple, const struct s8003_pm *pm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (pm->scene != s->gen)
		return true;
	if (pm->td_seq)
		return (s32)(pvr_apple_s8003_engine_done(apple, APPLE_ENGINE_FRAG) - pm->td_seq) >= 0;
	return false;
}

/*
 * Whether a geometry job on @pm, which needs @blocks, may go without the PM
 * record, which has the firmware rebuild the set's free list from its block
 * list: the set is not built anew for the job, and its last scene ran to its
 * end in this start of the firmware (both jobs handed over, the fragment job
 * done, no fault). A scene without a fragment job gave no pages back.
 */
static bool s8003_pm_clean(struct pvr_apple *apple, const struct s8003_pm *pm, u32 blocks)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return pm->rec_seq && pm->clean && pm->built == s->gen && pm->blocks >= blocks &&
	       pm->scene == s->gen && pm->td_seq && !READ_ONCE(apple->fault) &&
	       (s32)(pvr_apple_s8003_engine_done(apple, APPLE_ENGINE_FRAG) - pm->td_seq) >= 0;
}

/*
 * The firmware reports, at 0x40 of a set's list page, the most pages of the
 * pool a scene used and a margin (4 KiB pages) and a request for more
 * blocks; 0x60 is 1 once the host took them. Apple's kext grows the pool
 * from them between scenes (AGXParameterManagementMeta::
 * growPoolFromFirmwareFeedback): to three times the high-water mark or the
 * margin plus a block, else by the request, at most every 0x3d0901 ticks
 * of 24 MHz.
 */
#define S8003_PM_FEEDBACK		0x40
#define S8003_PM_FEEDBACK_TAKEN		0x60
#define S8003_PM_FEEDBACK_MS		167
#define S8003_PM_GROW_MAX		8	/* blocks per request */

/* Take a VM's idle sets' feedback; submit_lock held */
static void s8003_pm_feedback(struct pvr_apple *apple, struct s8003_vm *vm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	ktime_t now = ktime_get();
	unsigned int i;

	lockdep_assert_held(&apple->submit_lock);
	if (!apple->booted || (vm->fb_at && ktime_ms_delta(now, vm->fb_at) < S8003_PM_FEEDBACK_MS))
		return;
	for (i = 0; i < ARRAY_SIZE(vm->pm); i++) {
		struct s8003_pm *pm = &vm->pm[i];
		u32 high, margin, request, target;
		u8 *list;

		if (pm->set < 0 || pm->built != s->gen || !s8003_pm_idle(apple, pm))
			continue;
		list = s->pmsets.map + pm->set * S8003_PM_SET_SIZE + PVR_DEVICE_PAGE_SIZE;
		if (get32(list + S8003_PM_FEEDBACK_TAKEN) == 1)
			continue;
		rmb();	/* the flag before the words it covers */
		high = get32(list + S8003_PM_FEEDBACK);
		margin = get32(list + S8003_PM_FEEDBACK + 4);
		request = min(get32(list + S8003_PM_FEEDBACK + 12), (u32)S8003_PM_GROW_MAX);
		put32(list + S8003_PM_FEEDBACK_TAKEN, 1);
		wmb();
		vm->fb_at = now;
		/* Not from this pool: ignored */
		if (high > pm->blocks * 32 || margin > S8003_PM_BLOCKS_MAX * 32)
			continue;
		target = DIV_ROUND_UP(max(3 * high, 32 + margin), 32);
		if (target <= pm->blocks && request)
			target = pm->blocks + request;
		target = min(target, S8003_PM_BLOCKS_MAX);
		if (target <= max(vm->fb_blocks, pm->blocks))
			continue;
		vm->fb_blocks = target;
		dev_dbg(apple_dev(apple),
			"PM pool VM=%p grows to %u blocks from feedback: high-water %u pages, margin %u, request %u\n",
			vm->vm_ctx, target, high, margin, request);
	}
}

static bool pvr_apple_s8003_vm_evictable(struct pvr_apple *apple,
					      struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm = xa_load(&s->vms, (unsigned long)vm_ctx);
	unsigned int i;

	lockdep_assert_held(&apple->submit_lock);
	if (!vm || READ_ONCE(apple->fault))
		return false;
	if (vm->attached != s->gen)
		return true;
	if (!apple->booted || s->pm_last == vm || s->pm_prev == vm)
		return false;
	for (i = 0; i < ARRAY_SIZE(vm->pm); i++)
		if (!s8003_pm_idle(apple, &vm->pm[i]))
			return false;
	return true;
}

/* Publish capacity only once its pages, CPU maps and aux VA are ready.
 * Worker readers hold submit_lock; the atomic arming callback holds set_lock.
 */
static void s8003_pm_mark_ready(struct pvr_apple_s8003 *s, struct s8003_pm *pm)
{
	unsigned long flags;

	lockdep_assert_held(&s->apple->submit_lock);
	spin_lock_irqsave(&s->set_lock, flags);
	pm->ready = true;
	spin_unlock_irqrestore(&s->set_lock, flags);
}

static int s8003_prepare_job(struct pvr_apple *apple, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm = xa_load(&s->vms, (unsigned long)job->ctx->vm_ctx);
	struct s8003_rt rt;
	u32 blocks;
	unsigned int i;
	int err;

	if (job->type != DRM_PVR_JOB_TYPE_GEOMETRY)
		return 0;
	if (!vm)
		return -EINVAL;
	err = s8003_rt_derive(job->hwrt->hwrt_dataset, &rt);
	if (err)
		return err;
	blocks = min_t(u32, max(DIV_ROUND_UP(job->hwrt->hwrt_dataset->free_lists[0]->initial_num_pages, 32),
			       DIV_ROUND_UP(50 + rt.tiles, 32)), S8003_PM_BLOCKS_MAX);
	mutex_lock(&vm->prepare_lock);
	mutex_lock(&apple->submit_lock);
	s8003_vm_reserve_sets(apple, vm->vm_ctx);
	/* The firmware's feedback (s8003_pm_feedback()) is a floor for the next pool. */
	blocks = max(blocks, min(vm->fb_blocks, S8003_PM_BLOCKS_MAX));
	mutex_unlock(&apple->submit_lock);
	if (vm->pm[0].set < 0) {
		err = -ENOMEM;
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(vm->pm); i++) {
		struct s8003_pm *pm = &vm->pm[i];
		struct pvr_gem_object *obj;
		void *map;
		bool need_pool;

		if (pm->set < 0)
			continue;
		if (!pm->aux) {
			obj = pvr_gem_object_create(apple->pvr_dev, S8003_PM_AUX_SIZE,
						   DRM_PVR_BO_PM_FW_PROTECT);
			if (IS_ERR(obj)) {
				err = PTR_ERR(obj);
				goto out;
			}
			map = pvr_gem_object_vmap(obj);
			if (IS_ERR(map)) {
				err = PTR_ERR(map);
				pvr_gem_object_put(obj);
				goto out;
			}
			err = pvr_vm_map_reserved(vm->vm_ctx, obj, s8003_pm_aux_va(pm), S8003_PM_AUX_SIZE);
			if (err) {
				pvr_gem_object_vunmap(obj);
				pvr_gem_object_put(obj);
				goto out;
			}
			mutex_lock(&apple->submit_lock);
			pm->aux = obj;
			pm->aux_map = map;
			mutex_unlock(&apple->submit_lock);
		}
		mutex_lock(&apple->submit_lock);
		need_pool = max(pm->blocks, pm->next_blocks) < blocks;
		if (!need_pool)
			s8003_pm_mark_ready(s, pm);
		mutex_unlock(&apple->submit_lock);
		if (!need_pool)
			continue;
		obj = pvr_gem_object_create(apple->pvr_dev, (size_t)blocks * S8003_PM_BLOCK_SIZE,
					   PVR_BO_FW_FLAGS_DEVICE_CACHED);
		if (IS_ERR(obj)) {
			err = PTR_ERR(obj);
			goto out;
		}
		map = pvr_gem_object_vmap(obj);
		if (IS_ERR(map)) {
			err = PTR_ERR(map);
			pvr_gem_object_put(obj);
			goto out;
		}
		mutex_lock(&apple->submit_lock);
		if (pm->next_pool) {
			pvr_gem_object_vunmap(pm->next_pool);
			pvr_gem_object_put(pm->next_pool);
		}
		pm->next_pool = obj;
		pm->next_pool_map = map;
		pm->next_blocks = blocks;
		s8003_pm_mark_ready(s, pm);
		mutex_unlock(&apple->submit_lock);
	}
	err = 0;
out:
	mutex_unlock(&vm->prepare_lock);
	return err;
}

static int s8003_submit_error(struct pvr_apple *apple, struct s8003_vm *vm,
			      const char *engine, int err, enum s8003_refusal why,
			      const char *reason)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	atomic64_inc(&s->refused[why]);
	dev_warn_ratelimited(apple_dev(apple),
			     "%s job not submitted: error=%d VM=%p: %s\n",
			     engine, err, vm->vm_ctx, reason);
	return err;
}

static int s8003_submit_geom(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	const struct pvr_hwrt_dataset *hwrt = job->hwrt->hwrt_dataset;
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *p = s8003_channel(s, APPLE_ENGINE_GEOM);
	struct s8003_pm *pm = NULL;
	struct s8003_rt rt;
	u32 seq, blocks;
	bool pm_record;
	unsigned int i;
	int err;

	err = s8003_rt_derive(hwrt, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_RT,
					  "rt derive");
	/* The VM's sets were taken when the job was ordered (pvr_apple_s8003_vm_scenes()) */
	for (i = 0; i < 2 && !pm; i++)
		if (vm->pm[i].set >= 0 && vm->pm[i].ready &&
		    s8003_pm_idle(apple, &vm->pm[i]))
			pm = &vm->pm[i];
	/*
	 * Each holds a scene still: one whose fragment job never came (a
	 * geometry job alone) is given up once its geometry job is done.
	 */
	for (i = 0; i < 2 && !pm; i++)
		if (vm->pm[i].set >= 0 && vm->pm[i].ready && !vm->pm[i].td_seq &&
		    !s8003_wait_done(apple, APPLE_ENGINE_GEOM, vm->pm[i].ta_seq))
			pm = &vm->pm[i];
	if (!pm)
		return s8003_submit_error(apple, vm, "geometry", -EBUSY, S8003_REFUSED_PM,
					  "no PM set");

	/*
	 * Take the firmware's feedback. The worker does not allocate: a larger
	 * pool is prepared for a later job (s8003_prepare_job()) and switched to
	 * here, between scenes, once it is ready.
	 */
	s8003_pm_feedback(apple, vm);
	/* As many 128 KiB blocks as the free list has 4 KiB pages, and enough for the target */
	blocks = max(DIV_ROUND_UP(hwrt->free_lists[0]->initial_num_pages, 32),
		     DIV_ROUND_UP(50 + rt.tiles, 32));
	blocks = max(blocks, min(vm->fb_blocks, max(pm->blocks, pm->next_blocks)));
	/* Before the build, which may make the set anew */
	pm_record = !s8003_pm_clean(apple, pm, min(blocks, S8003_PM_BLOCKS_MAX));
	pm->clean = false;
	err = s8003_pm_build(apple, vm, pm, min(blocks, S8003_PM_BLOCKS_MAX));
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_PM,
					  "PM pool build failed");
	/* The record slot of the job before the last one is free again */
	if (apple->ta_seq)
		err = s8003_wait_done(apple, APPLE_ENGINE_GEOM, apple->ta_seq - 1);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");

	seq = apple->ta_seq + 1;
	if (!pm_record) {
		u32 done = pvr_apple_s8003_engine_done(apple, APPLE_ENGINE_GEOM);

		/* The last rebuild must be visible; a newer ordinary TA need not be. */
		if ((s32)(done - pm->rec_seq) < 0)
			pm_record = true;
	}
	s8003_fill_ta(p + s8003_slot_of(seq, false).w, job->cmd, job->hwrt, &rt);
	s8003_job_write(p, s8003_channel_addr(s, APPLE_ENGINE_GEOM),
			s8003_object_addr(&s->layout, S8003_CHANNEL),
			s8003_meta_addr(s8003_pm_set_va(pm, 0)), pm->set + 1, vm->slot[0], false,
			seq, s8003_channel_addr(s, APPLE_ENGINE_GEOM) + S8003_CH_STAMP + 4,
			pm_record ? 0 : pm->rec_seq, pm_record);
	if (pm_record)
		pm->rec_seq = seq;
	pm->hwrt = job->hwrt;
	if (s->pm_last != vm) {
		s->pm_prev = s->pm_last;
		s->pm_last = vm;
	}
	s8003_wb_sets_update(s);
	pm->scene = s->gen;
	pm->ta_seq = seq;
	pm->td_seq = 0;
	WRITE_ONCE(s->set_frag[pm->set], 0);
	WRITE_ONCE(s->set_geom[pm->set], seq);
	apple->ta_seq = seq;
	err = s8003_publish(apple, APPLE_ENGINE_GEOM, seq, (2 * seq) % S8003_CH_ENTRIES);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_PUBLISH,
					  "publish");
	return seq;
}

static int s8003_submit_frag(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *p = s8003_channel(s, APPLE_ENGINE_FRAG);
	struct s8003_pm *pm = NULL;
	struct s8003_rt rt;
	unsigned int i;
	u32 seq;
	int err;

	/* The scene of this job's geometry job, which was submitted before it */
	for (i = 0; i < 2; i++)
		if (vm->pm[i].scene == s->gen && vm->pm[i].hwrt == job->hwrt && !vm->pm[i].td_seq)
			pm = &vm->pm[i];
	if (!pm)
		return s8003_submit_error(apple, vm, "fragment", -ECANCELED, S8003_REFUSED_NO_GEOM,
					   "its geometry job failed or was not submitted");
	err = s8003_rt_derive(job->hwrt->hwrt_dataset, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_RT,
					  "rt derive");
	if (apple->frag_seq)
		err = s8003_wait_done(apple, APPLE_ENGINE_FRAG, apple->frag_seq - 1);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");

	seq = apple->frag_seq + 1;
	err = s8003_fill_3d(p + s8003_slot_of(seq, true).w, job->cmd, job->hwrt, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_FILL,
					  "3D record fill");
	s8003_job_write(p, s8003_channel_addr(s, APPLE_ENGINE_FRAG),
			s8003_object_addr(&s->layout, S8003_CHANNEL + 1),
			s8003_meta_addr(s8003_pm_set_va(pm, 0)), pm->set + 1, vm->slot[1], true, seq,
			s8003_channel_addr(s, APPLE_ENGINE_GEOM) + S8003_CH_STAMP + 4, pm->ta_seq, true);
	pm->td_seq = seq;
	WRITE_ONCE(s->set_frag[pm->set], seq);
	apple->frag_seq = seq;
	err = s8003_publish(apple, APPLE_ENGINE_FRAG, seq, (2 * seq) % S8003_CH_ENTRIES);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_PUBLISH,
					  "publish");
	pm->clean = true;
	return seq;
}

/*
 * A compute job runs on its VM's geometry slot. One record: the job before
 * it is done first. The microsequence: start, wait for the engine's
 * "finished" event and its status, finish.
 */
static int s8003_submit_compute(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	const struct pvr_apple_g5p_cmd_compute *cmd = job->cmd;
	const struct pvr_apple_g5p_compute_regs *g = &cmd->g5p;
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 addr = s8003_channel_addr(s, APPLE_ENGINE_COMPUTE);
	u32 global = s8003_object_addr(&s->layout, S8003_CHANNEL + 2);
	u32 w = addr + S8003_CH_W, slot = vm->slot[0], seq;
	u8 *p = s8003_channel(s, APPLE_ENGINE_COMPUTE);
	u8 *work = p + S8003_CH_W;
	unsigned int i;
	int err;

	err = s8003_wait_done(apple, APPLE_ENGINE_COMPUTE, apple->last_seq);
	if (err)
		return s8003_submit_error(apple, vm, "compute", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");
	seq = apple->last_seq + 1;
	{
		const u32 useq[] = {
			0x32, w + 0x10, w + 0x60, global + 0x720, addr, slot, 0, 0, 0, w + 0x1e8,
			0, 0, seq,
			0x80000018, w + 0x278, w + 0x280, w + 0x280, seq,
			0x101, 0x004a000a, 1, 1,
			0x18, w + 0x278, w + 0x280, w + 0x284, seq,
			0x33, global + 0x720, addr, slot, w + 0x1e8, 0, 0, seq,
			addr + S8003_CU_STAMP + 4, seq, (u32)-0x6c,
			0x40000017,
		};

		BUILD_BUG_ON(sizeof(useq) > S8003_CU_USEQ_SIZE);
		memset(p + S8003_CU_USEQ, 0, S8003_CU_USEQ_SIZE);
		useq_put(p + S8003_CU_USEQ, useq, ARRAY_SIZE(useq));
	}
	memset(work, 0, S8003_CU_W_SIZE);
	put32(work, 3);
	put32(work + 8, slot);
	put32(work + 0xc, addr + S8003_CU_DEP);
	put32(work + 0x10, g->config[0]);
	put32(work + 0x14, g->config[1]);
	put64(work + 0x60, g->tail);
	put64(work + 0x68, g->pds_terminate);
	put64(work + 0x70, cmd->cmd.regs.cdm_ctrl_stream_base);
	put32(work + 0x1e0, addr + S8003_CU_USEQ);
	put32(work + 0x1e4, S8003_CU_USEQ_SIZE);
	for (i = 0; i < 4; i++)
		put64(work + 0x1ec + i * 8, g->state[i]);
	put64(work + 0x20c, g->tail);
	put64(work + 0x214, g->terminate);
	put64(work + 0x21c, g->pds_store);
	put32(work + 0x224, g->pds_store1);
	put64(work + 0x22c, g->pds_resume);
	put32(work + 0x234, g->pds_resume1);
	put64(work + 0x23c, g->terminate);
	put32(work + 0x244, 0x00a00004);
	put32(work + 0x258, addr + S8003_CU_STAMP);
	put32(work + 0x25c, addr + S8003_CU_STAMP + 4);
	put32(work + 0x260, seq);
	put32(work + 0x270, seq);
	put32(p + S8003_CH_RING + ((seq - 1) % S8003_CH_ENTRIES) * 4, w);
	apple->last_seq = seq;
	err = s8003_publish(apple, APPLE_ENGINE_COMPUTE, seq, seq % S8003_CH_ENTRIES);
	if (err)
		return s8003_submit_error(apple, vm, "compute", err, S8003_REFUSED_PUBLISH,
					  "publish");
	return seq;
}

static int pvr_apple_s8003_submit(struct pvr_apple *apple, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm;
	int err;

	mutex_lock(&apple->submit_lock);
	vm = xa_load(&s->vms, (unsigned long)job->ctx->vm_ctx);
	if (!apple->booted || !vm)
		err = -ENODEV;
	else
		err = s8003_vm_attach(apple, vm);
	if (err) {
		/* the chip's own refusals say why below; these said nothing */
		if (apple->chip->wait_deadline)
			dev_warn_ratelimited(apple_dev(apple), "job not submitted: %s (%d)\n",
					     !apple->booted ? "the firmware is not running" :
					     !vm ? "its VM is not known to the chip" :
					     err == -EBUSY ? "every VM slot is taken" :
					     "its VM did not attach", err);
		goto out_unlock;
	}
	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY)
		err = s->v->submit_geom(apple, vm, job);
	else if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		err = s->v->submit_frag(apple, vm, job);
	else
		err = s->v->submit_compute(apple, vm, job);
out_unlock:
	mutex_unlock(&apple->submit_lock);
	return err;
}

static void s8003_report_hang(struct pvr_apple *apple)
{
	static const char *const names[] = { "compute", "geometry", "fragment" };
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *dc = s8003_object(s, S8003_DC_CTL);
	int engine;

	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		const u8 *p = s8003_channel(s, engine);
		const u8 *ctl = s8003_object(s, S8003_WORK_CTL + 4 * s8003_engine(engine));
		u32 stamp = engine == APPLE_ENGINE_COMPUTE ? S8003_CU_STAMP : S8003_CH_STAMP;

		dev_err(apple_dev(apple),
			"hang: %s: submitted %u, stamps %u %u, watermarks %u %u %u %u %u, work ring %u/%u\n",
			names[engine], s8003_submitted(apple, engine), get32(p + stamp),
			get32(p + stamp + 4), get32(p + S8003_CH_CTL),
			get32(p + S8003_CH_CTL + 0x10), get32(p + S8003_CH_CTL + 0x20),
			get32(p + S8003_CH_CTL + 0x30), get32(p + S8003_CH_CTL + 0x40),
			get32(ctl + S8003_RING_CONSUMER), get32(ctl + S8003_RING_PRODUCER));
	}
	dev_err(apple_dev(apple),
		"hang: firmware %s, device control %u/%u, Fender state %#x\n",
		s->failed ? "rings unreadable" : READ_ONCE(apple->fault) ? "posted a fault" : "posted no fault",
		get32(dc + S8003_RING_CONSUMER),
		get32(dc + S8003_RING_PRODUCER), readl(apple->fender + FENDER_POWER_STATE));
}

/*
 * The first command: an empty device-control command. The firmware takes it
 * from the ring once its scheduler runs, which is when it takes commands at
 * all.
 */
static int s8003_first_command(struct pvr_apple *apple)
{
	static const u32 cmd[1][2] = { { S8003_DC_NONE, 0 } };
	struct pvr_apple_s8003 *s = apple->chip_data;
	int err;

	put32(s8003_object(s, S8003_CONTROL) + 0x1020, 0);	/* no payload */
	err = s8003_device_control(apple, cmd, 1);
	if (err)
		dev_err(apple_dev(apple), "GFX firmware did not take its first command: %d\n",
			err);
	return err;
}

/* A start of the firmware: it knows no VM, and its channels are empty */
static void s8003_jobs_reset(struct pvr_apple_s8003 *s)
{
	struct s8003_vm *vm;
	unsigned long index, flags;
	unsigned int i;

	spin_lock_irqsave(&s->tag_lock, flags);
	memset(s->tag, 0, sizeof(s->tag));
	spin_unlock_irqrestore(&s->tag_lock, flags);
	bitmap_zero(s->parked, S8003_SLOTS);
	if (!++s->gen)
		s->gen = 1;
	xa_for_each(&s->vms, index, vm)
		for (i = 0; i < ARRAY_SIZE(vm->pm); i++)
			vm->pm[i].rec_seq = 0;
	WRITE_ONCE(s->failed, false);
	WRITE_ONCE(s->apple->fault, false);
	memset(s->set_geom, 0, sizeof(s->set_geom));
	memset(s->set_frag, 0, sizeof(s->set_frag));
	if (s->pmsets.map)
		memset(s->pmsets.map, 0, s->pmsets.size);
	s->v->channels_init(s);
}

/*
 * The registers the firmware is told about, as bus addresses: Fender and the
 * Rogue core from this node, the interrupt controller and the number of the
 * firmware's interrupt from the node's last interrupt.
 */
static int s8003_parse_dt(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	struct platform_device *pdev = to_platform_device(dev);
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct of_phandle_args irq;
	struct resource *res, aic;
	int err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fender");
	if (!res)
		return -EINVAL;
	s->fender_phys = res->start;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rgx");
	if (!res)
		return -EINVAL;
	s->rgx_phys = res->start;

	err = of_irq_parse_one(dev->of_node, 4, &irq);
	if (err)
		return dev_err_probe(dev, err, "no firmware interrupt\n");
	if (irq.args_count < 2 || irq.args[0] != AIC_IRQ)
		err = -EINVAL;
	else
		err = of_address_to_resource(irq.np, 0, &aic);
	of_node_put(irq.np);
	if (err)
		return dev_err_probe(dev, err, "firmware interrupt not on an AIC\n");
	s->irq = irq.args[1];
	s->aic_phys = aic.start;

	if ((s->fender_phys | s->aic_phys) & ~PVR_DEVICE_PAGE_MASK)
		return -EINVAL;
	return 0;
}

static void s8003_free_image(void *data)
{
	kvfree(data);
}

/* Hard IRQ context: clear the software-set source, before the rings are read */
static void pvr_apple_s8003_irq_ack(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	writel(BIT(s->irq % 32), s->aic_sw_clr);
}

/*
 * The firmware's interrupt, the node's fifth. Without it (or its register)
 * the jobs are polled for, as before.
 */
static void s8003_irq_init(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	struct pvr_apple_s8003 *s = apple->chip_data;
	int irq, err;

	s->aic_sw_clr = devm_ioremap(dev, s->aic_phys + AIC_SW_CLR + 4 * (s->irq / 32), 4);
	irq = platform_get_irq_optional(to_platform_device(dev), 4);
	if (!s->aic_sw_clr)
		err = -ENOMEM;
	else if (irq < 0)
		err = irq;
	else
		err = pvr_apple_firmware_irq_init(apple, irq);
	if (err)
		dev_warn(dev, "firmware interrupt not taken (%d): jobs are polled for\n", err);
}

static const struct s8003_variant *s8003_variant_of(const struct pvr_apple_chip *chip);

static int pvr_apple_s8003_init(struct pvr_apple *apple)
{
	const struct s8003_variant *v = s8003_variant_of(apple->chip);
	struct device *dev = apple_dev(apple);
	struct pvr_apple_layout *layout = &apple->layout;
	struct pvr_apple_s8003 *s;
	unsigned int i;
	int err;

	/* The firmware's page tables have the chip's page size, as the kernel's must */
	if (PVR_DEVICE_PAGE_SIZE != v->page_size)
		return dev_err_probe(dev, -ENODEV, "the %s GPU needs a %u KiB page kernel\n",
				     v->name, v->page_size / SZ_1K);

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	apple->chip_data = s;
	s->apple = apple;
	s->v = v;
	INIT_DELAYED_WORK(&s->drain_work, s8003_drain_work);
	spin_lock_init(&s->tag_lock);
	spin_lock_init(&s->set_lock);
	xa_init(&s->vms);
	s->next_slot = v->slot_first;
	err = devm_mutex_init(dev, &s->event_lock);
	if (err)
		return err;

	err = s8003_parse_dt(apple);
	if (err)
		return err;
	if (v->init) {
		err = v->init(apple);
		if (err)
			return err;
	}
	s8003_irq_init(apple);
	err = s8003_layout_plan(v, &s->layout);
	if (err)
		return err;

	s->image = kvzalloc(v->image_size, GFP_KERNEL);
	if (!s->image)
		return -ENOMEM;
	err = devm_add_action_or_reset(dev, s8003_free_image, s->image);
	if (err)
		return err;
	err = pvr_apple_fw_load(apple, SZ_1M);
	if (err)
		return err;
	err = pvr_meta_ldr_process(from_pvr_device(apple->pvr_dev), apple->fw_image,
				   apple->fw_size, v->ldr, s, NULL);
	if (!err && (!s->thread_pc || (v->ldr_conf_count && s->conf_count != v->ldr_conf_count)))
		err = -EINVAL;
	if (err)
		return dev_err_probe(dev, err, "not the %s's GFX firmware image\n", v->name);
	v->image_configure(s);

	apple->heap = &s->idle;
	for (i = 0; i < APPLE_ENGINES; i++) {
		layout->stamp[i] = offsetof(struct s8003_idle, stamp[i]);
		layout->producer[i] = offsetof(struct s8003_idle, producer[i]);
	}
	layout->frag_ring = offsetof(struct s8003_idle, frag_ring);
	layout->frag_retired = offsetof(struct s8003_idle, frag_retired);
	layout->mmu_root = offsetof(struct s8003_idle, mmu_root);
	layout->mmu_gen = offsetof(struct s8003_idle, mmu_gen);
	layout->event_consumer = offsetof(struct s8003_idle, event_consumer);
	layout->event_producer = offsetof(struct s8003_idle, event_producer);
	return 0;
}

/*
 * Wake Fender and power the core in state 1, the META held in reset: the
 * Rogue registers answer (the core's ID is read next), no firmware runs.
 */
static int s8003_boot(struct pvr_apple *apple)
{
	int err;

	writel(FENDER_STREAM_END, apple->fender + FENDER_STREAM);
	writel(FENDER_CTRL_BOOT, apple->fender + FENDER_CTRL);
	err = pvr_apple_request_power_state(apple, 1);
	if (err)
		return err;
	usleep_range(10, 20);
	return 0;
}

static int pvr_apple_s8003_boot(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return s->v->boot(apple);
}

static int pvr_apple_s8003_start(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	const struct s8003_variant *v = s->v;
	u32 q = pvr_apple_start_power_state(apple);
	int err;

	if (!s->shared.obj) {
		err = v->map(apple);
		if (err)
			return err;
	}
	/* The firmware writes into its image while it runs: start from the copy */
	memcpy(s->fw.map, s->image, v->image_size);
	v->init_build(apple, q);
	s8003_jobs_reset(s);

	a7_dbg(s, "booting the firmware (power state %u)\n", q);
	err = v->release(apple, q);
	if (!err) {
		err = s8003_wait_alive(apple);
		if (err && v->no_alive) {
			v->no_alive(apple);
			return err;
		}
	}
	if (!err) {
		dev_dbg(apple_dev(apple), "GFX firmware is up (power state %u)\n", q);
		if (v->first_command)
			err = v->first_command(apple);
	}
	if (err) {
		/* It must not run on once the kernel VM, its page tables, may go */
		v->core_off(apple);
		return err;
	}
	dev_dbg(apple_dev(apple), "GFX firmware takes commands\n");
	/* Once: the tables a VM's slots are left on when the VM goes */
	if (!s->park_vm) {
		s->park_vm = pvr_vm_create_context(apple->pvr_dev, true);
		if (IS_ERR(s->park_vm)) {
			dev_warn(apple_dev(apple), "no park VM: %pe\n", s->park_vm);
			s->park_vm = NULL;
		}
	}
	schedule_delayed_work(&s->drain_work, msecs_to_jiffies(S8003_DRAIN_MS));
	return 0;
}

/*
 * Power the core off through Fender state 0, which stops the META with it;
 * the firmware has no part in it, so a hung one stops the same way. Not with
 * the GPU's power domain off already (a probe that failed after boot()):
 * Fender is not to be touched then, and nothing is left running.
 */
static void pvr_apple_s8003_stop(struct pvr_apple *apple, bool recover)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (READ_ONCE(apple->stopping))
		cancel_delayed_work(&s->drain_work);
	else
		cancel_delayed_work_sync(&s->drain_work);
	if (pm_runtime_status_suspended(apple_dev(apple)))
		return;
	s->v->core_off(apple);
}

static void s8003_idle(struct pvr_apple *apple)
{
	/* nothing of the firmware's keeps the GPU's power domain on */
}

static void pvr_apple_s8003_idle(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	s->v->idle(apple);
}

static void pvr_apple_s8003_restore_image(struct pvr_apple *apple)
{
	/* start() copies the image and builds the init data anew each time */
}

static u32 pvr_apple_s8003_max_power_state(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return get32(s8003_object(s, S8003_PLATFORM) + 0x710);
}

static void pvr_apple_s8003_report_hang(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	s->v->report_hang(apple);
}

static int pvr_apple_s8003_prepare_job(struct pvr_apple *apple, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return s->v->prepare_job(apple, job);
}

/*
 * The A8 (T7000): the GX6450 (Apple's G4P) runs the same family of firmware
 * on its META (AGXFirmwareKextG4P, "METAFW_G4P g4p_b0"), booted the same way.
 * What differs is ported from the standalone bring-up module that boots it
 * on the iPod touch 6, stage by stage (an out-of-tree module that is not
 * part of this series): power-on and ID, firmware boot to the alive event, slot
 * load, compute, and the first TA + 3D render. Addresses in the comments
 * are Apple's iOS 12.4 (16G77) iPod7,1 kernelcache (AGXG4P, unslid) and the
 * firmware's; "P4"/"P5" are the live iOS objects dumped on that device.
 */

/* The core this driver boots: CORE_REVISION major 3, minor 10 (readChipInfo 0x8479d14) */
#define T7000_CORE_REVISION		0x00030a00

/* The power manager's registers, from its base (ApplePMGR) */
#define T7000_PMGR_PLL5			0x5004	/* the GPU's PLL: bit 30 locked */
#define T7000_PLL_LOCKED		BIT(30)
#define T7000_PMGR_GPU_TABLE		0x1c000	/* GPU performance records 1-3 at +0x10 each */
#define T7000_GPU_TABLE_VALID		BIT(21)
#define T7000_PMGR_VM_CTL		0x23000	/* the voltage manager: on with [11:8] clear */
#define T7000_PMGR_PS_GFX		0x20320	/* the GPU's power state */
#define T7000_PMGR_SOC_TVM		0x2d000	/* SoC voltage manager enable (0x849d340) */
#define T7000_PMGR_MISC_ACG		0x32000	/* clock gating enable (0x849d1bc) */
#define T7000_PS_TARGET			GENMASK(3, 0)
#define T7000_PS_ACTUAL			GENMASK(7, 4)
#define T7000_PS_HIST			GENMASK(9, 8)
#define T7000_PS_DEV_DISABLE		BIT(10)
#define T7000_PS_MIN			GENMASK(19, 16)
#define T7000_PS_AUTO_PM		BIT(28)
#define T7000_PS_BIT30			BIT(30)
#define T7000_PS_ON			0xf
#define T7000_PS_POLL_US		200000

#define T7000_FENDER_POLL_US		100000

/*
 * The bring-up module paused this long before each group of new accesses
 * (its log crumbs): before the GPU's power state goes on (after SOC_TVM),
 * before Fender's wake, before the first Rogue register read, before Fender
 * state 0 after it, before the boot stream and before the release. Every
 * run had these pauses, so the driver keeps them.
 */
#define T7000_SETTLE_MS			t7000_settle(apple)
static unsigned int t7000_settle_ms;
module_param_named(apple_settle_ms, t7000_settle_ms, uint, 0644);
MODULE_PARM_DESC(apple_settle_ms,
		 "A8/A7: the pause before each step of the GPU's power-up and firmware boot, in ms (0, the default: the chip's, A8 100, A7 10)");

static unsigned int t7000_settle(struct pvr_apple *apple)
{
	const struct pvr_apple_s8003 *s = apple->chip_data;

	return t7000_settle_ms ?: s->v->soc->settle_ms;
}
#define T7000_FENDER_SNOOP		0x0014	/* FENDER_WAKE: bit 1 snoop armed */
#define T7000_FENDER_CTRL_SNOOP		BIT(29)

/* Rogue registers of the boot stream that powervr has no name for */
#define T7000_CR_SLC_3808		0x3808

/*
 * The unpacked image as 0x84727c8 lays it out with iOS's 4 KiB pages: the
 * bootloader's segment (2) first, then code, data and one more data range;
 * no core memory. The handover is at the end of segment 2.
 */
#define T7000_IMAGE_SIZE		0x24000
#define T7000_IMAGE_HANDOVER		0xff8
#define T7000_SEGMMU_ALLTHRS_WRITEABLE	0xf02	/* 0x8471b0c: BASE | 0xf02 */
#define T7000_SEGMMU_OUTA1_BIFDM	(7u << 8) /* (PLATFORM +0x4bc = 7) << 8 */

/* The firmware's register windows (PLATFORM +0x190, P4): Fender, AIC + 0x1000, AIC + 0x4000 */
#define T7000_PIO_FENDER		0x0000
#define T7000_PIO_FENDER_SIZE		0x5000
#define T7000_PIO_AIC_TIME		0x8000
#define T7000_PIO_AIC_SET		0xc000
#define T7000_PIO_AIC_SIZE		0x1000

/*
 * Root memory: the objects, then the channels and the PM sets' control
 * blocks, all in META's uncached window as Apple's (P5: channel 0x43318200,
 * PM control 0x4331c000). Shared memory: the objects, then the PM sets'
 * lists in the cached window as Apple's (P5b: 0xc0284000).
 */
#define T7000_ROOT_OBJECTS_END		0x18000
#define T7000_CU_CHANNEL		0x18000	/* 0x1000 */
#define T7000_TA_CHANNEL		0x20000	/* 0x3000 each */
#define T7000_3D_CHANNEL		0x24000
#define T7000_PM_CONTROL		0x28000	/* 0x2000 per set */
#define T7000_SHARED_OBJECTS_END	0x60000
#define T7000_PM_LIST			0x60000	/* 0x2000 per set */
#define T7000_PM_SETS			7	/* one per user slot */
#define T7000_PM_SET_SIZE		0x2000
#define T7000_ROOT_SIZE			(T7000_PM_CONTROL + T7000_PM_SETS * T7000_PM_SET_SIZE)
#define T7000_SHARED_SIZE		(T7000_PM_LIST + T7000_PM_SETS * T7000_PM_SET_SIZE)

/* Slots: 8, the host's from 1, one per VM (vt+0x888/0x890/0x898) */
#define T7000_SLOTS			8
#define T7000_SLOT_FIRST		1

/*
 * The hardware-specific objects Apple's kext maps at firmware init
 * (0x847950c) at 0x8_00000000 and names to the firmware (PLATFORM +0x30,
 * +0x188, +0x4a8; object 0x4c +0/+8): the YUV CSC buffer, BRN44455Object
 * (0x8475964), BorderColorTable (0x847586c: 0x800 bytes of the kext's data)
 * and FenceBuffer (0x8475a40). Each GART reserves 0x8_00000000 + 4 MiB
 * (0x8463138), so every VM sees them at these addresses: the firmware loads
 * the FenceBuffer into MCU_FENCE and the MCU writes it through the running
 * job's context. Plain flags: with PM/META protect the firmware never
 * posts its alive event.
 */
#define T7000_HWOBJ_VA			0x800000000ull
#define T7000_HWOBJ_SIZE		0x4000
#define T7000_HWOBJ_WINDOW		SZ_4M
#define T7000_BORDER_SIZE		0x800
#define T7000_BORDER_FW			"apple/gfx-t7000-border.bin"
enum { T7000_HW_YUV, T7000_HW_BRN44455, T7000_HW_BORDER, T7000_HW_FENCE, T7000_HW_OBJS };
static const u32 t7000_hwobj_offset[T7000_HW_OBJS] = { 0x0, 0x4000, 0x8000, 0x10000 };

/* GPU memory of the firmware's: past the device cache, PM/firmware protected */
#define T7000_FW_FLAGS			(DRM_PVR_BO_BYPASS_DEVICE_CACHE | DRM_PVR_BO_PM_FW_PROTECT)
/* What a VM's jobs reach: past the device cache, nothing else */
#define T7000_VM_FLAGS			DRM_PVR_BO_BYPASS_DEVICE_CACHE

/*
 * Objects, as the A8's kext sizes them (allocateSharedRingGroups 0x84653e8,
 * packed-group table 0x712fa38..fab0) and packs them, checked against
 * every live P4 object. A ring lane's indices take 0x30 bytes (A9 0x20).
 */
#define T7000_RING_CTL_SIZE		0x30

static const struct s8003_object_size s8003_objects_a8[] = {
	{ S8003_INIT, false, 0x1118 },
	{ S8003_CHANNEL, false, 0x990 },
	{ S8003_CHANNEL + 1, false, 0x8e0 },
	{ S8003_CHANNEL + 2, false, 0x8c0 },
	{ S8003_POLICY, false, 0x1058 },
	{ 0x4b, false, 0x28 },
	{ 0x4c, false, 0x10 },
	{ S8003_PLATFORM, false, 0x4dc },
	{ 0x4e, false, 0x84 },
	{ S8003_TAG_SEEN, false, 0x20 },
	{ S8003_CONTROL, true, 0x1528 },
	{ S8003_EVENT_CTL, true, T7000_RING_CTL_SIZE },
	{ S8003_LOG_CTL, true, 4 * T7000_RING_CTL_SIZE },
	{ S8003_STATS_CTL, true, T7000_RING_CTL_SIZE },
	{ S8003_TRACE_CTL, true, T7000_RING_CTL_SIZE },
	{ S8003_EVENTS, true, 0x2c00 },
	{ S8003_LOGS, true, 0xb000 },
	{ 0x4f, true, 0x36000 },
	{ S8003_STATS, true, 0x2c00 },
	{ S8003_TRACE, true, 0x2400 },
	{ S8003_WORK_CTL, true, T7000_RING_CTL_SIZE },
	{ S8003_WORK_CTL + 4, true, T7000_RING_CTL_SIZE },
	{ S8003_WORK_CTL + 8, true, T7000_RING_CTL_SIZE },
	{ S8003_DC_CTL, true, T7000_RING_CTL_SIZE },
	{ S8003_WORK_RING, true, 0x1400 },
	{ S8003_WORK_RING + 4, true, 0x1400 },
	{ S8003_WORK_RING + 8, true, 0x1400 },
	{ S8003_DC_RING, true, 0x1400 },
	{ S8003_TAG_HOST, true, 0x20 },
	{ S8003_MMU_ROOT, true, 0x40 },
};

/* The image's segments: META address (bit 31 clear), size, offset in the image */
static const struct {
	u32 base, size, offset;
} t7000_segments[4] = {
	{ 0x10000000, 0x1e000, 0x01000 },
	{ 0x20000000, 0x03000, 0x1f000 },
	{ 0x40000000, 0x01000, 0x00000 },	/* the bootloader's: not configured (0x8471b0c) */
	{ 0x60000000, 0x02000, 0x22000 },
};

/* Where a section of the LDR image goes, as 0x8472a00 reads it */
static int t7000_ldr_mem(void *data, u32 addr, u32 size, bool zero, void **write_addr)
{
	struct pvr_apple_s8003 *s = data;
	u32 seg = (addr >> 29) & 3, a = addr & 0x7fffffff;

	/* core memory (0x80000080..): the A8's image has none */
	if ((u32)(addr + 0x7fffff80) >> 7 < 0x17f || !((addr >> 27) & 0xf))
		return -EINVAL;
	if (a < t7000_segments[seg].base || size > t7000_segments[seg].size ||
	    a - t7000_segments[seg].base > t7000_segments[seg].size - size)
		return -EINVAL;
	*write_addr = s->image + t7000_segments[seg].offset + a - t7000_segments[seg].base;
	return 0;
}

/* Thread 1's stack pointer and entry (0x8472a00 case 3), from entries {thread, sp, pc, catch} */
static int t7000_ldr_start_threads(void *data, const u32 *words, u32 count)
{
	struct pvr_apple_s8003 *s = data;
	u32 i;

	for (i = 0; i + 4 <= count; i += 4) {
		if (words[i] != 1)
			continue;
		if (s->thread_pc)
			return -EINVAL;
		s->thread_sp = words[i + 1];
		s->thread_pc = words[i + 2];
	}
	return 0;
}

static const struct pvr_meta_ldr_target t7000_ldr_target = {
	.mem = t7000_ldr_mem,
	.config = s8003_ldr_config,
	.start_threads = t7000_ldr_start_threads,
};

/*
 * The bootloader's configuration (0x84724c4 + 0x8471b0c): segments 0, 1 and
 * 3 with the BIF data master in OUTA1, the image's 27 writes, a zero u64 and
 * the META address of the handover, which holds INIT's and CONTROL's.
 */
static void t7000_image_configure(struct pvr_apple_s8003 *s)
{
	u8 *conf = s->image + S8003_IMAGE_CONF;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(t7000_segments); i++) {
		u64 out = S8003_IMAGE_VA + t7000_segments[i].offset;
		u32 limit = max(t7000_segments[i].size - 1, 0xfffu);

		if (i == 2)
			continue;
		put32(conf, META_CR_MMCU_SEGMENT_N_BASE(i));
		put32(conf + 4, t7000_segments[i].base | T7000_SEGMMU_ALLTHRS_WRITEABLE);
		put32(conf + 8, META_CR_MMCU_SEGMENT_N_LIMIT(i));
		put32(conf + 12, limit);
		put32(conf + 16, META_CR_MMCU_SEGMENT_N_OUTA0(i));
		put32(conf + 20, lower_32_bits(out));
		put32(conf + 24, META_CR_MMCU_SEGMENT_N_OUTA1(i));
		put32(conf + 28, T7000_SEGMMU_OUTA1_BIFDM | upper_32_bits(out));
		conf += 32;
	}
	for (i = 0; i < s->conf_count; i++, conf += 8) {
		put32(conf, s->conf[i][0]);
		put32(conf + 4, s->conf[i][1]);
	}
	put64(conf, 0);
	put32(conf + 8, s8003_meta_addr(S8003_IMAGE_VA + T7000_IMAGE_HANDOVER));
	put32(s->image + T7000_IMAGE_HANDOVER, s8003_object_addr(&s->layout, S8003_INIT));
	put32(s->image + T7000_IMAGE_HANDOVER + 4, s8003_object_addr(&s->layout, S8003_CONTROL));
}

/* The registers' windows and the hardware objects in the kernel VM */
static void t7000_unmap(struct pvr_apple *apple)
{
	struct pvr_vm_context *vm_ctx = apple->pvr_dev->kernel_vm_ctx;
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int i;

	if (s->t7000_pio_mapped) {
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_FENDER, T7000_PIO_FENDER_SIZE);
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_AIC_TIME, T7000_PIO_AIC_SIZE);
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_AIC_SET, T7000_PIO_AIC_SIZE);
		s->t7000_pio_mapped = false;
	}
	for (i = 0; i < T7000_HW_OBJS; i++)
		s8003_mem_fini(apple, &s->hwobj[i]);
	if (s->a7_scratch.obj)
		s8003_mem_fini(apple, &s->a7_scratch);
}

/*
 * The firmware's memory and registers in the kernel VM, as the bring-up
 * module maps them: the image, root and shared memory and a zeroed page at
 * address 0 for the firmware only and past the device cache (the CPU writes
 * them uncached), the register windows as pvr_vm_map_io() maps registers,
 * and the hardware objects.
 */
static int t7000_map(struct pvr_apple *apple)
{
	struct pvr_vm_context *vm_ctx = apple->pvr_dev->kernel_vm_ctx;
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int i;
	int err;

	err = s8003_mem_init(apple, &s->fw, S8003_IMAGE_VA, T7000_IMAGE_SIZE, T7000_FW_FLAGS);
	if (!err)
		err = s8003_mem_init(apple, &s->root, S8003_ROOT_VA, T7000_ROOT_SIZE, T7000_FW_FLAGS);
	if (!err)
		err = s8003_mem_init(apple, &s->shared, S8003_SHARED_VA, T7000_SHARED_SIZE,
				     T7000_FW_FLAGS);
	if (!err)
		err = s8003_mem_init(apple, &s->zero, 0, PVR_DEVICE_PAGE_SIZE, T7000_FW_FLAGS);
	for (i = 0; !err && i < T7000_HW_OBJS; i++)
		err = s8003_mem_init(apple, &s->hwobj[i], T7000_HWOBJ_VA + t7000_hwobj_offset[i],
				     T7000_HWOBJ_SIZE, T7000_VM_FLAGS);
	if (!err) {
		err = pvr_vm_map_io(vm_ctx, s->fender_phys, S8003_PIO_VA + T7000_PIO_FENDER,
				    T7000_PIO_FENDER_SIZE);
		if (!err) {
			err = pvr_vm_map_io(vm_ctx, s->aic_phys + 0x1000,
					    S8003_PIO_VA + T7000_PIO_AIC_TIME, T7000_PIO_AIC_SIZE);
			if (err)
				pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_FENDER,
						T7000_PIO_FENDER_SIZE);
		}
		if (!err) {
			err = pvr_vm_map_io(vm_ctx, s->aic_phys + AIC_SW_SET,
					    S8003_PIO_VA + T7000_PIO_AIC_SET, T7000_PIO_AIC_SIZE);
			if (err) {
				pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_FENDER,
						T7000_PIO_FENDER_SIZE);
				pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + T7000_PIO_AIC_TIME,
						T7000_PIO_AIC_SIZE);
			}
		}
		if (!err)
			s->t7000_pio_mapped = true;
	}
	/*
	 * The A7's firmware writes at GPU VA 0xff_ffff_fff0 at boot (an MMU
	 * fault on the J71, bank 1, tag 3, context 0). A scratch page there
	 * takes the write; what lands is logged after the alive event.
	 */
	if (!err && s->v->soc->ps_domain &&
	    s8003_mem_init(apple, &s->a7_scratch, A7_SCRATCH_VA, PVR_DEVICE_PAGE_SIZE, T7000_VM_FLAGS))
		dev_warn(apple_dev(apple), "no scratch page at %#llx\n", A7_SCRATCH_VA);
	if (err)
		pvr_apple_s8003_fini(apple);
	return err;
}

/* PLATFORM +0x38..+0xaf: the first of the two 0xa8-byte blocks 0x8466578 copies from acc+0x1798 (P4) */
static const u32 t7000_platform_38[] = {
	0x00000000, 0x00002000, 0x00002000, 0x00000000, 0x20000000, 0x00000000,
	0x00002543, 0xe4233312, 0xf3772543, 0x10f2e5fd, 0x408d2543, 0xdd650000,
	0x00002543, 0xe0fd395e, 0xf92d2543, 0x099ceef2, 0x43962543, 0xdbe10000,
	0x00002000, 0xe9912cdd, 0xf4fd2000, 0x10eee926, 0x38b42000, 0xe3a60000,
	0x00002000, 0xe73b3189, 0xfa022000, 0x0a7df105, 0x3b5e2000, 0xe2510000,
};

/* POLICY +0x40: the 0x160 bytes 0x8467770 copies from fwctl +0x3f0 (P4: the nonzero words) */
static const u32 t7000_policy_40[][2] = {
	{ 0x40, 0x4294fc36 }, { 0x44, 0x40041a37 }, { 0x48, 0x4219ba44 }, { 0xe0, 0x42849d8b },
	{ 0x164, 0x3f800000 }, { 0x168, 0x3faab021 }, { 0x16c, 0x3fc820c5 }, { 0x170, 0x3fe9d495 },
};

/*
 * The governor's words 0x8467770 computes from fwctl +0x598..+0x63c (P4), with
 * the result it stores; where iOS's firmware changed a word later (+0x678,
 * +0x67c, +0x774, +0x78c), Apple's initial value.
 */
static const u32 t7000_policy_gov[][2] = {
	{ 0x634, 0x271 }, { 0x640, 0x3f7f2e9f }, { 0x648, 0x3b516154 }, { 0x650, 0x3ca59586 },
	{ 0x65c, 0x452ba000 }, { 0x660, 0x40a90fdb }, { 0x668, 0xbddfbe5e }, { 0x66c, 0xfffffefc },
	{ 0x674, 0x12c }, { 0x67c, 0xaba }, { 0x694, 0x12c }, { 0x72c, 0x5f }, { 0x730, 0x6 },
	{ 0x734, 0x1 }, { 0x73c, 0x3f4ccccd }, { 0x744, 0x3e4ccccd }, { 0x740, 0x3f7ae148 },
	{ 0x748, 0x3ca3d70a }, { 0x74c, 0x40fca970 }, { 0x750, 0x3e4a2121 }, { 0x758, 0x42be0000 },
	{ 0x75c, 0x416b53d1 }, { 0x760, 0x40db53d0 }, { 0x764, 0x4006bca2 }, { 0x76c, 0xc8 },
	{ 0x770, 0x64 }, { 0x778, 0x5f }, { 0x790, 0x64 }, { 0x7a8, 0x42c80000 }, { 0x7ac, 300 },
	{ 0x7b4, 0x3f4ccccd }, { 0x7b8, 0x3e4ccccd }, { 0x904, 300 },
};

/* MHz and mV per row (kext arrays acc+0x19d4, P4): states 1-3 are 365, 450 and 500 MHz */
static const u32 t7000_mhz[7] = { 0, 365, 450, 500, 182, 225, 250 };
static const u32 t7000_mv[7] = { 600, 871, 896, 912, 871, 896, 912 };

/* AGXG3's CSC block (0xfffffff00876e48c, data 0xfffffff00717ac00), 16H88 J71 */
static const u32 s5l8960x_platform_38[] = {
	0x00000000, 0x00000200, 0x00000200, 0x00000000, 0x02000000, 0x00000000,
	0x00000254, 0x1e420331, 0x0f370254, 0x010f0e60, 0x04090254, 0x1dd60000,
	0x00000254, 0x1e100396, 0x0f930254, 0x009a0eef, 0x04390254, 0x1dbe0000,
	0x00000200, 0x1e9902ce, 0x0f500200, 0x010f0e92, 0x038b0200, 0x1e3a0000,
	0x00000200, 0x1e740319, 0x0fa00200, 0x00a80f10, 0x03b60200, 0x1e250000,
};

/* The J71's live ADT sgx perf-states (7 rows of {Hz, mV}; iBoot fills them) */
static const u32 s5l8960x_mhz[7] = { 0, 330, 390, 450, 165, 195, 225 };
static const u32 s5l8960x_mv[7] = { 600, 871, 937, 1000, 871, 937, 1000 };

static const struct t7000_soc t7000_soc_a8 = {
	.border_fw = T7000_BORDER_FW,
	.ps_gfx = T7000_PMGR_PS_GFX,
	.pll = T7000_PMGR_PLL5,
	.pll_lock = T7000_PLL_LOCKED,
	.a8_globals = true,
	.bridge_reg = 0x18,
	.bridge_val = 0x80010004,
	.platform_38 = t7000_platform_38,
	.mhz = t7000_mhz,
	.mv = t7000_mv,
	.perf_count = 16,
	.fence = true,
	.slc = true,
	.rev_major = 3,
	.chip_info = { 2, 4, 4 },
	.settle_ms = 100,
	.te_psg = 0x00022000,
	.pbe_3d = 0x118,
	.word_3d4 = 0x00001200,
};

/*
 * The A7. Its GPU power state (PMGR +0x20218) is the ps_gfx power domain,
 * behind dwi; MISC_ACG is the A8's word, but there is no SOC_TVM, and the
 * A8's perf records and voltage manager word are not known on it. Its GPU
 * PLL: page 5 +0x0 bit 30 rose with Fender's wake on the J71 (one run;
 * +0x4 did not change). Bridges 11 and 12 from the J71's live ADT: bridge
 * 11 +0x14 = 0x80010004, bridge 12 the A8's 32 words. The border table is
 * AGXG3's clamp_to_zero_border_color_table_noastc; there is no FenceBuffer
 * (AGXG3 vt+0xa28 returns 0) and no SLC setup (its populateInitSequence is
 * empty). CORE_REVISION major 1, minor 0x32, 0x41 (chip info 3) or 0x46
 * (4); the J71's reads 0x00014600, CORE_ID 0x011b0403. PLATFORM +0x310 is
 * the ADT's perf-state-count, 7.
 */
static const struct t7000_soc t7000_soc_a7 = {
	.border_fw = "apple/gfx-s5l8960x-border.bin",
	.ps_gfx = 0x20218,
	.ps_domain = true,
	.pll = 0x5000,
	.pll_lock = BIT(30),
	.bridge_reg = 0x14,
	.bridge_val = 0x80010004,
	.platform_38 = s5l8960x_platform_38,
	.mhz = s5l8960x_mhz,
	.mv = s5l8960x_mv,
	.perf_count = 7,
	.rev_major = 1,
	.chip_info = { 1, 4, 4 },
	/* J71: a boot with 10 ms pauses takes 46 ms, with 100 ms 418 ms */
	.settle_ms = 10,
	/* J71: geometry jobs and a clear-only render ran with these (28 Nov) */
	.te_psg = 0x00020000,
	.pbe_3d = 0xd8,
};

/*
 * The root and shared memory as the A8's firmware finds it when it starts
 * (0x8466578, 0x8467770; values from P4): where its objects are, the heaps'
 * bases, the registers it may use, the interrupt it raises, its thread, the
 * performance states and the hardware objects. @q is the power state the
 * core starts in, to which the power interface's two clock targets are
 * pinned (Apple: state 1, 0x846eb10). CONTROL +0xcd0..+0xcd8, the
 * performance controller's inputs, stay zero as in Apple's live CONTROL.
 */
static void t7000_init_build(struct pvr_apple *apple, u32 q)
{
	static const u8 rings[] = {
		S8003_WORK_CTL, S8003_WORK_RING, S8003_WORK_CTL + 4, S8003_WORK_RING + 4,
		S8003_WORK_CTL + 8, S8003_WORK_RING + 8, S8003_DC_CTL, S8003_DC_RING,
		S8003_EVENT_CTL, S8003_EVENTS, S8003_LOG_CTL, S8003_LOGS,
		S8003_STATS_CTL, S8003_STATS, S8003_TRACE_CTL, S8003_TRACE,
	};
	static const u8 objects[] = {
		S8003_CHANNEL, S8003_CHANNEL + 1, S8003_CHANNEL + 2, S8003_POLICY, 0x4b,
	};
	static const u8 tail[] = { 0x4e, 0x4c, S8003_PLATFORM, S8003_TAG_SEEN, 0x4f };
	struct pvr_apple_s8003 *s = apple->chip_data;
	const struct s8003_layout *layout = &s->layout;
	const struct {
		u64 phys;
		u32 addr;
		u32 size;
	} pio[] = {
		{ s->fender_phys, s8003_meta_addr(S8003_PIO_VA + T7000_PIO_FENDER),
		  T7000_PIO_FENDER_SIZE },
		{ s->aic_phys + 0x1000, s8003_meta_addr(S8003_PIO_VA + T7000_PIO_AIC_TIME),
		  T7000_PIO_AIC_SIZE },
		{ s->aic_phys + AIC_SW_SET, s8003_meta_addr(S8003_PIO_VA + T7000_PIO_AIC_SET),
		  T7000_PIO_AIC_SIZE },
		{ s->rgx_phys, 0, 0x20000 },
	};
	const struct t7000_soc *soc = s->v->soc;
	u8 *init, *platform, *policy, *control, *o4c;
	unsigned int i;

	memset(s->root.map, 0, T7000_ROOT_SIZE);
	memset(s->shared.map, 0, T7000_SHARED_SIZE);
	init = s8003_object(s, S8003_INIT);
	platform = s8003_object(s, S8003_PLATFORM);
	policy = s8003_object(s, S8003_POLICY);
	control = s8003_object(s, S8003_CONTROL);
	o4c = s8003_object(s, 0x4c);

	for (i = 0; i < ARRAY_SIZE(rings); i++)
		put32(init + 4 * i, s8003_object_addr(layout, rings[i]));
	for (i = 0; i < ARRAY_SIZE(objects); i++)
		put32(init + 0x88 + 4 * i, s8003_object_addr(layout, objects[i]));
	for (i = 0; i < ARRAY_SIZE(tail); i++)
		put32(init + 0x470 + 4 * i, s8003_object_addr(layout, tail[i]));
	/* No core memory: the image's end, size 0 (0x84724c4; P4 0x40024000, 0xe1c0024000, 0) */
	put32(init + 0x4a4, (lower_32_bits(S8003_IMAGE_VA + T7000_IMAGE_SIZE) + 0x40000000) |
			    0x40000000);
	put64(init + 0x4a8, S8003_IMAGE_VA + T7000_IMAGE_SIZE);
	/* Channel headers {0, self, 0, self + 8}; no channel 1 +0x778 on the A8 */
	for (i = 0; i < 3; i++) {
		u8 *channel = s8003_object(s, S8003_CHANNEL + i);
		u32 addr = s8003_object_addr(layout, S8003_CHANNEL + i);

		put32(channel + 4, addr);
		put32(channel + 12, addr + 8);
	}

	/* The heaps' bases (DAT 0x712f910): +0x20/+0x28 is the PDS and USC execution base */
	put64(platform + 0x00, 0x500000000ull);
	put64(platform + 0x08, 0x600000000ull);
	put64(platform + 0x10, 0x100000000ull);
	put64(platform + 0x18, 0x100000000ull);
	put64(platform + 0x20, 0x300000000ull);
	put64(platform + 0x28, 0x300000000ull);
	for (i = 0; i < ARRAY_SIZE(t7000_platform_38); i++)
		put32(platform + 0x38 + 4 * i, soc->platform_38[i]);
	for (i = 0; i < ARRAY_SIZE(pio); i++) {
		u8 *entry = platform + 0x190 + 16 * i;

		put64(entry, pio[i].phys);
		put32(entry + 8, pio[i].addr);
		put32(entry + 12, pio[i].size);
	}
	put32(platform + 0x250, 1);	/* Fender boot */
	put32(platform + 0x254, 1);	/* gpu_lockup_detection */
	put32(platform + 0x258, 1);	/* PIO supported */
	put32(platform + 0x274, 1);	/* agf_timeslice */
	put32(platform + 0x278, 1);
	put32(platform + 0x27c, 1);	/* agf_timer */
	put32(platform + 0x280, 1000 * 0x18);	/* agf_timer_period, 24 MHz ticks */
	put32(platform + 0x284, 0x10);
	put32(platform + 0x290, 1);	/* agf_dwb */
	put32(platform + 0x294, 1);	/* agf_itr_cache */
	put32(platform + 0x29c, 1);	/* agf_deferred_opaque */
	put32(platform + 0x2b4, 0x1f);	/* agf_max_tiles_per_pipe */
	put32(platform + 0x2dc, 1);	/* agf_frg_vce_inval */
	/* chip info {major class, minor class, cores}: the A8's {2, 4, 4} */
	for (i = 0; i < 3; i++)
		put32(platform + 0x300 + 4 * i,
		      s->chip_info[0] ? s->chip_info[i] : soc->chip_info[i]);
	put32(platform + 0x30c, 3);	/* the highest performance state */
	put32(platform + 0x310, soc->perf_count);
	for (i = 0; i < ARRAY_SIZE(t7000_mhz); i++) {
		put32(platform + 0x314 + 4 * i, soc->mhz[i]);
		put32(platform + 0x354 + 4 * i, soc->mv[i]);
	}
	/* The interrupt to the host: its word and bit among the AIC's software sets (P4 0x10, 0x400) */
	put32(platform + 0x494, 4 * (s->irq / 32));
	put32(platform + 0x498, BIT(s->irq % 32));
	put32(platform + 0x49c, AIC_TIME & 0xfff);	/* in the time window */
	put32(platform + 0x4a0, s->thread_pc);
	put32(platform + 0x4a4, s->thread_sp);
	put64(platform + 0x4b0, S8003_IMAGE_VA);
	put64(platform + 0x4b8, 0x700000000ull);	/* DAT 0x712fab8 */
	put32(platform + 0x4c8, 0xffff);	/* agf_gtp_ks */
	put32(platform + 0x4cc, 7);		/* agf_frg_ks */
	put32(platform + 0x4d0, 7);		/* agf_compute_ks */

	put32(policy + 0x04, 0x10);
	put64(policy + 0x10, 0x3f800000);
	put64(policy + 0x28, 0x100000001ull);
	put32(policy + 0x3c, 100);
	for (i = 0; i < ARRAY_SIZE(t7000_policy_40); i++)
		put32(policy + t7000_policy_40[i][0], t7000_policy_40[i][1]);
	for (i = 0; i < ARRAY_SIZE(t7000_policy_gov); i++)
		put32(policy + t7000_policy_gov[i][0], t7000_policy_gov[i][1]);
	put64(policy + 0x1008, arch_timer_read_counter());	/* the governor's last sample */

	put32(control + 0x28, 1);
	put32(control + 0x34, 0x78);		/* gpu_temperature_limit */
	put32(control + 0x4a, 1);
	put32(control + 0xc66, 1867);		/* power target */
	put32(control + 0xc6a, 100 * q);	/* the power interface's clock targets */
	put32(control + 0xc6e, 100 * q);
	put32(control + 0xd0e, s8003_object_addr(layout, S8003_TAG_HOST));
	put32(control + 0xd12, s8003_object_addr(layout, S8003_MMU_ROOT));
	put32(control + 0xd16, 1);
	/* GPU context 0 runs on the kernel VM, whose catalogue the boot stream gives the BIF */
	put64(s8003_object(s, S8003_MMU_ROOT),
	      pvr_vm_get_page_table_root_addr(apple->pvr_dev->kernel_vm_ctx));

	/* The hardware objects, fresh, and their names */
	for (i = 0; i < T7000_HW_OBJS; i++)
		memset(s->hwobj[i].map, 0, T7000_HWOBJ_SIZE);
	memcpy(s->hwobj[T7000_HW_BORDER].map, s->border, T7000_BORDER_SIZE);
	if (s->a7_scratch.map)
		memset(s->a7_scratch.map, 0, PVR_DEVICE_PAGE_SIZE);
	put64(platform + 0x30, T7000_HWOBJ_VA + t7000_hwobj_offset[T7000_HW_YUV]);
	put64(platform + 0x188, T7000_HWOBJ_VA + t7000_hwobj_offset[T7000_HW_BORDER]);
	put64(o4c, T7000_HWOBJ_VA + t7000_hwobj_offset[T7000_HW_BRN44455]);
	if (soc->fence) {
		put64(platform + 0x4a8, T7000_HWOBJ_VA + t7000_hwobj_offset[T7000_HW_FENCE]);
		put64(o4c + 8, T7000_HWOBJ_VA + t7000_hwobj_offset[T7000_HW_FENCE]);
	}
}

/*
 * A channel page. The compute channel has the A9's layout (s8003_channel_init()),
 * which the A8's firmware takes. The TA and 3D channels have Apple's header
 * and ring control (P5b chfw_ta/chfw_3d, chring: five watermarks 0x10 apart,
 * 0x144 entries, the ring right behind at +0x60); their dependency objects,
 * stamps and records are where the A9's layout has them.
 */
#define T7000_RCH_CMD			0x300	/* the 3D job's barrier command */
#define T7000_RCH_W			0x400
#define T7000_RCH_TA_USEQ		0xa00
#define T7000_RCH_3D_USEQ		0xd00
#define T7000_RCH_TIMES			0x2800	/* the record's times: end, -, start, - */
#define T7000_RCH_SIZE			0x3000
/*
 * The 3D channel's second record (a8_joint): its barrier command, record,
 * microsequence and times behind the first's, in the 0x1000 the 3D channel
 * has before the PM control blocks (T7000_PM_CONTROL).
 */
#define T7000_RCH_SLOT1			0x3000
#define T7000_RCH1_CMD			(T7000_RCH_SLOT1 + 0x000)
#define T7000_RCH1_W			(T7000_RCH_SLOT1 + 0x100)
#define T7000_RCH1_3D_USEQ		(T7000_RCH_SLOT1 + 0xa00)
#define T7000_RCH1_TIMES		(T7000_RCH_SLOT1 + 0xc00)
#define T7000_RCH_CTL			0x2000	/* Apple's ring control: 0x60 + 0x144 entries */
#define T7000_RCH_RING			(T7000_RCH_CTL + 0x60)
#define T7000_RCH_ENTRIES		0x144
#define T7000_CU_SIZE			0x1000

static void t7000_channel_init(u8 *p, u32 size, u32 addr, u32 tq, u32 dep)
{
	memset(p, 0, size);
	s8003_channel_init(p, addr, tq, dep);
}

/* Apple's TA/3D channel header: @control_knob is CONTROL + 0xc, @object the engine's channel object */
static void t7000_render_channel_init(u8 *p, u32 addr, u32 control_knob, u32 object)
{
	t7000_channel_init(p, T7000_RCH_SIZE, addr, S8003_CH_TQ, S8003_CH_DEP);
	memset(p, 0, 0x80);
	put32(p + 0x00, addr + T7000_RCH_CTL);
	put32(p + 0x04, addr + T7000_RCH_RING);
	put32(p + 0x08, control_knob);
	put32(p + 0x0c, 2);
	put32(p + 0x10, 0xff000000);
	put32(p + 0x18, 0xffffffff);
	put32(p + 0x2c, object);
	put32(p + 0x34, object + 8);
	put32(p + 0x4c, 0x21);
	put32(p + 0x54, 0x640);
	memset(p + S8003_CH_CTL, 0, 0x60);
	put32(p + T7000_RCH_CTL + S8003_CH_CTL_ENTRIES, T7000_RCH_ENTRIES);
}

static void t7000_channels_init(struct pvr_apple_s8003 *s)
{
	u32 knob = s8003_object_addr(&s->layout, S8003_CONTROL) + 0xc;

	t7000_channel_init(s8003_channel(s, APPLE_ENGINE_COMPUTE), T7000_CU_SIZE,
			   s8003_channel_addr(s, APPLE_ENGINE_COMPUTE), S8003_CU_TQ, S8003_CU_DEP);
	t7000_render_channel_init(s8003_channel(s, APPLE_ENGINE_GEOM),
				  s8003_channel_addr(s, APPLE_ENGINE_GEOM), knob,
				  s8003_object_addr(&s->layout, S8003_CHANNEL));
	t7000_render_channel_init(s8003_channel(s, APPLE_ENGINE_FRAG),
				  s8003_channel_addr(s, APPLE_ENGINE_FRAG), knob,
				  s8003_object_addr(&s->layout, S8003_CHANNEL + 1));
}

static u32 pmgr_read(struct pvr_apple_s8003 *s, u32 reg)
{
	return readl(s->pmgr + reg);
}

/*
 * Every access to the Rogue registers waits for this: Fender in state 1, no
 * request in flight, the GPU's PLL locked, all read right before. An access
 * with the PLL unlocked hung the whole SoC in bring-up.
 */
static bool t7000_ps_on(struct pvr_apple_s8003 *s);

static bool t7000_r0_gate(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 state, pll;

	/* The A7's domain may be off (runtime PM): then Fender is not to be read */
	if (s->v->soc->ps_domain && !t7000_ps_on(s)) {
		dev_err_ratelimited(apple_dev(apple),
				    "GPU registers not accessed: the GPU's power domain is off (%#x)\n",
				    pmgr_read(s, s->v->soc->ps_gfx));
		return false;
	}
	state = readl(apple->fender + FENDER_POWER_STATE);
	pll = pmgr_read(s, s->v->soc->pll);
	if ((state & FENDER_POWER_STATE_MASK) == 1 && (pll & s->v->soc->pll_lock))
		return true;
	dev_err_ratelimited(apple_dev(apple),
			    "GPU registers not accessed: Fender state %#x, GPU PLL %#x\n", state, pll);
	return false;
}

/*
 * Where Apple panics (a GPC state transition stuck, no alive event): leave
 * the GPU as it is, as the bring-up module does. No Fender, PMGR or Rogue
 * register is touched again, and the firmware's memory and page tables are
 * kept until a reboot (pvr_apple_s8003_fini()): the META may still run in
 * them.
 */
static void t7000_stuck(struct pvr_apple *apple, const char *why)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (s->stuck)
		return;
	s->stuck = true;
	if (apple->pvr_dev->kernel_vm_ctx)
		pvr_vm_context_get(apple->pvr_dev->kernel_vm_ctx);
	dev_err(apple_dev(apple), "%s: the GPU is left as it is, with its memory, until a reboot\n",
		why);
}

/* Ask Fender for power state @state and wait until no request is in flight */
static int t7000_fender_state(struct pvr_apple *apple, u32 state)
{
	u32 val;
	int err;

	writel(FENDER_POWER_REQUEST | state, apple->fender + FENDER_POWER_STATE);
	err = readl_poll_timeout(apple->fender + FENDER_POWER_STATE, val,
				 !(val & FENDER_POWER_REQUEST), 5, T7000_FENDER_POLL_US);
	if (err) {
		dev_err(apple_dev(apple), "Fender stayed busy going to state %u (%#x)\n", state, val);
		return err;
	}
	apple->perf_state = state;
	return 0;
}

/* Apple's GPU power state writer (0x8497520): on (auto power management) or off */
static int t7000_ps_set(struct pvr_apple *apple, u32 target)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	void __iomem *ps = s->pmgr + s->v->soc->ps_gfx;
	u32 r = readl(ps), val;
	int err;

	if (FIELD_GET(T7000_PS_TARGET, r) == target && !(r & (T7000_PS_BIT30 | T7000_PS_DEV_DISABLE)) &&
	    !!(r & T7000_PS_AUTO_PM) == (target == T7000_PS_ON))
		return 0;
	if (r & T7000_PS_AUTO_PM) {
		r &= 0xeffffcff;
		writel(r, ps);
	} else {
		r &= 0xfffffcff;
	}
	r = (r & 0xfffffcf0) | target;
	if (target < T7000_PS_ON)
		r |= T7000_PS_HIST;
	writel(r, ps);
	err = readl_poll_timeout(ps, val, FIELD_GET(T7000_PS_ACTUAL, val) == target, 5,
				 T7000_PS_POLL_US);
	if (err) {
		dev_err(apple_dev(apple), "GPU power state %#x not reached: %#x\n", target, val);
		return err;
	}
	if (target != T7000_PS_ON)
		return 0;
	r = (r & ~T7000_PS_HIST) | T7000_PS_AUTO_PM;
	writel(r, ps);
	if (r & T7000_PS_BIT30) {
		r &= ~T7000_PS_BIT30;
		writel(r, ps);
	}
	if (r & T7000_PS_DEV_DISABLE)
		writel(r & ~T7000_PS_DEV_DISABLE, ps);
	return 0;
}

static bool t7000_ps_on(struct pvr_apple_s8003 *s)
{
	return FIELD_GET(T7000_PS_ACTUAL, pmgr_read(s, s->v->soc->ps_gfx)) == T7000_PS_ON;
}

/*
 * Apple's bridge settings for the GPU (0x84a0098, from the ADT): one store to
 * bridge 10, 32 to bridge 11, after the GPU powers on.
 */
static const u32 t7000_bridge11[32] = {
	0x007c7c01, 0x007c7c01, 0x007c7c01, 0x007c7901, 0x007c7901, 0x007c7c01, 0x007c7c01,
	0x007c7c01, 0x007c7c01, 0x007c7c01, 0x007c7c01, 0x007c7c01, 0x006c6901, 0x007c7901,
	0x007c7c01, 0x007c7c01, 0x00747201, 0x00747201, 0x00747201, 0x00747201, 0x007c7c01,
	0x007c7c01, 0x007c7c01, 0x006c6901, 0x007c7c01, 0x00797901, 0x00797901, 0x007c7c01,
	0x006c6901, 0x00797901, 0x00797901, 0x00797901,
};

static void t7000_bridges(struct pvr_apple_s8003 *s)
{
	unsigned int i;

	writel(s->v->soc->bridge_val, s->bridges + s->v->soc->bridge_reg);
	for (i = 0; i < ARRAY_SIZE(t7000_bridge11); i++)
		writel(t7000_bridge11[i], s->bridges + 0x80000 + 4 * i);
}

static int t7000_wake_fender(struct pvr_apple *apple);

/*
 * The A7: its power domain (ps_gfx, behind dwi) is on when this runs.
 * MISC_ACG is set first if it is not yet (iOS's power manager sets it at its
 * start; a boot that wants Apple's order sets it before the domain goes on),
 * then the bridge settings, then Fender's wake into state 1 behind an empty
 * boot stream, with the A8's pauses.
 */
static int t7000_boot_domain(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 val;
	int err;

	if (!t7000_ps_on(s)) {
		dev_err(apple_dev(apple), "the GPU's power domain is off: %#x\n",
			pmgr_read(s, s->v->soc->ps_gfx));
		return -EIO;
	}
	val = pmgr_read(s, T7000_PMGR_MISC_ACG);
	if (!(val & 1)) {
		dev_warn(apple_dev(apple), "MISC_ACG was clear: set after the GPU's power\n");
		writel(val | 1, s->pmgr + T7000_PMGR_MISC_ACG);
	}
	a7_dbg(s, "power-up: power state %#x, MISC_ACG %#x; bridges, then Fender's wake\n",
		pmgr_read(s, s->v->soc->ps_gfx), pmgr_read(s, T7000_PMGR_MISC_ACG));
	t7000_bridges(s);
	msleep(T7000_SETTLE_MS);
	err = t7000_wake_fender(apple);
	a7_dbg(s, "power-up done (%d): Fender %#x, GPU PLL %#x\n", err,
		readl(apple->fender + FENDER_POWER_STATE), pmgr_read(s, s->v->soc->pll));
	return err == -EAGAIN ? -ETIMEDOUT : err;
}

/*
 * Power the GPU on as the bring-up module does, in Apple's order: the
 * power manager's clock gating (0x849d1bc) and SoC voltage manager
 * (0x849d340) enables, then the GPU's power state with automatic power
 * management (K3-K5), the bridge settings (K6), and Fender's wake into
 * state 1 with the META held back by an empty boot stream (K7). The core's
 * ID can be read then. A GPU already on (a restart without power-off) is
 * only woken.
 */
static int t7000_boot(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	void __iomem *ps = s->pmgr + s->v->soc->ps_gfx;
	unsigned int n;
	u32 val;
	int err;

	if (s->stuck)
		return -EIO;
	if (s->v->soc->ps_domain)
		return t7000_boot_domain(apple);
	/*
	 * Every stop powers the GPU off (t7000_core_off()): one that is on was
	 * left on by something else. The module refuses such a GPU, and so
	 * does this.
	 */
	if (FIELD_GET(T7000_PS_ACTUAL, pmgr_read(s, T7000_PMGR_PS_GFX))) {
		dev_err(apple_dev(apple), "the GPU is powered already: not touched\n");
		return -EBUSY;
	}
	for (n = 1; n <= 3; n++) {
		if (!(pmgr_read(s, T7000_PMGR_GPU_TABLE + 0x10 * n) & T7000_GPU_TABLE_VALID)) {
			dev_err(apple_dev(apple), "GPU performance record %u not valid\n", n);
			return -EIO;
		}
	}
	val = pmgr_read(s, T7000_PMGR_MISC_ACG);
	if (!(val & 1))
		writel(val | 1, s->pmgr + T7000_PMGR_MISC_ACG);
	if (pmgr_read(s, T7000_PMGR_VM_CTL) & 0xf00) {
		dev_err(apple_dev(apple), "the voltage manager is off: GPU not powered\n");
		return -EIO;
	}
	val = pmgr_read(s, T7000_PMGR_SOC_TVM);
	if (!(val & 1)) {
		writel(val | 1, s->pmgr + T7000_PMGR_SOC_TVM);
		udelay(100);
	}
	msleep(T7000_SETTLE_MS);	/* the SRAM rail SOC_TVM raises, before the GPU powers */
	/* K3 (0x84973ac): minimum state 0, history cleared */
	val = readl(ps);
	writel((val & 0xfff00000) | (val & 0xfcff), ps);
	err = t7000_ps_set(apple, T7000_PS_ON);
	if (err)
		goto ps_off;
	t7000_bridges(s);

	msleep(T7000_SETTLE_MS);
	err = t7000_wake_fender(apple);
	if (err != -EAGAIN)
		return err;
	err = -ETIMEDOUT;
ps_off:
	t7000_ps_set(apple, 0);
	return err;
}

/* Fender's wake into state 1; -EAGAIN after a failed wake that went back to state 0 */
static int t7000_wake_fender(struct pvr_apple *apple)
{
	int err;

	writel(FENDER_STREAM_END, apple->fender + FENDER_STREAM);
	writel(FENDER_CTRL_BOOT, apple->fender + FENDER_CTRL);
	mutex_lock(&apple->perf_lock);
	err = t7000_fender_state(apple, 1);
	mutex_unlock(&apple->perf_lock);
	if (!err) {
		usleep_range(10, 20);
		msleep(T7000_SETTLE_MS);
		return 0;
	}
	if (t7000_fender_state(apple, 0)) {
		/* Apple panics here ("GPC state transition is stuck") */
		t7000_stuck(apple, "Fender stuck at the wake");
		return err;
	}
	return -EAGAIN;
}

static int t7000_core_off(struct pvr_apple *apple);

/* The cores AGXG4P (A8) and AGXG3 (A7, 0xfffffff00876e7cc) accept */
static bool t7000_rev_ok(struct pvr_apple_s8003 *s, u32 rev)
{
	u32 minor = FIELD_GET(GENMASK(15, 8), rev);

	if (s->v->soc->rev_major == 3)
		return rev == T7000_CORE_REVISION;
	return FIELD_GET(GENMASK(23, 16), rev) == 1 && !(rev & 0xff0000ff) &&
	       (minor == 0x32 || minor == 0x41 || minor == 0x46);
}

/* The core's ID, as the bring-up module reads it: two gated 32-bit reads */
static int pvr_apple_t7000_read_gpu_id(struct pvr_apple *apple, struct pvr_gpu_id *gpu_id)
{
	struct pvr_device *pvr_dev = apple->pvr_dev;
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 rev = 0, id = 0;
	int err = -EIO;

	if (t7000_r0_gate(apple)) {
		rev = pvr_cr_read32(pvr_dev, ROGUE_CR_CORE_REVISION);
		if (t7000_r0_gate(apple)) {
			id = pvr_cr_read32(pvr_dev, ROGUE_CR_CORE_ID);
			err = 0;
		}
	}
	/*
	 * As the module: the core goes back to Fender state 0 at once, and
	 * the GPU off if the ID was not read. The release starts from state 0.
	 */
	if (err) {
		t7000_core_off(apple);
		return err;
	}
	msleep(T7000_SETTLE_MS);
	mutex_lock(&apple->perf_lock);
	err = t7000_fender_state(apple, 0);
	mutex_unlock(&apple->perf_lock);
	if (err) {
		t7000_stuck(apple, "Fender stuck after the ID read");
		return err;
	}
	if (!t7000_rev_ok(s, rev)) {
		dev_err(apple_dev(apple), "CORE_REVISION %#x is not the %s's\n", rev, s->v->name);
		return -ENODEV;
	}
	/* readChipInfo: {2, 4} on the A8; {1, 3} or {1, 4} (minor 0x46) on the A7; the cores */
	s->chip_info[0] = s->v->soc->rev_major == 3 ? 2 : 1;
	s->chip_info[1] = s->v->soc->rev_major == 3 ? 4 :
			  FIELD_GET(GENMASK(15, 8), rev) == 0x46 ? 4 : 3;
	s->chip_info[2] = FIELD_GET(GENMASK(15, 8), id);
	dev_info(apple_dev(apple), "%s GPU: CORE_REVISION %#010x CORE_ID %#010x\n", s->v->name,
		 rev, id);
	gpu_id->b = FIELD_GET(GENMASK(23, 16), rev);
	gpu_id->v = FIELD_GET(GENMASK(15, 8), rev);
	gpu_id->n = FIELD_GET(GENMASK(15, 8), id);
	gpu_id->c = FIELD_GET(GENMASK(7, 0), id);
	return 0;
}

/*
 * One 32- or 64-bit write of the boot stream, as Apple's Fender writers
 * (0x847a858, 0x847a8b4) store them: no read-back.
 */
static void t7000_stream_word(struct s8003_stream *stream, u32 word)
{
	if (stream->offset >= FENDER_STREAM_SIZE) {
		stream->err = -ENOSPC;
		return;
	}
	writel(word, stream->apple->fender + FENDER_STREAM + stream->offset);
	stream->offset += 4;
}

static void t7000_stream_write32(struct s8003_stream *stream, u32 reg, u32 value)
{
	t7000_stream_word(stream, reg | 0xff << 16);
	t7000_stream_word(stream, value);
}

static void t7000_stream_write64(struct s8003_stream *stream, u32 reg, u64 value)
{
	t7000_stream_word(stream, reg + 4);
	t7000_stream_word(stream, upper_32_bits(value));
	t7000_stream_word(stream, reg | 0xff << 16);
	t7000_stream_word(stream, lower_32_bits(value));
}

/*
 * Apple's stop (0x846ebf8): Fender's core word 0, then state 0, waiting for
 * each; then the stored boot stream is cut back to its end marker, so that
 * no later power-up replays it, and the GPU's power state goes off: every
 * boot starts from a GPU that is off, in the full power-on order. A Fender that stays busy (Apple panics) is
 * left alone, and so is everything after it.
 */
static int t7000_core_off(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 val;
	int err;

	if (s->stuck || !t7000_ps_on(s)) {
		a7_dbg(s, "stop: nothing to do (%s, power state %#x)\n", s->stuck ? "stuck" : "off",
			pmgr_read(s, s->v->soc->ps_gfx));
		return s->stuck ? -EIO : 0;
	}
	a7_dbg(s, "stop: Fender %#x sleep %#x kick %#x int %#x, power state %#x\n",
		readl(apple->fender + FENDER_POWER_STATE), readl(apple->fender + FENDER_CTRL),
		readl(apple->fender + 0x8), readl(apple->fender + 0x14),
		pmgr_read(s, s->v->soc->ps_gfx));
	writel(0, apple->fender + FENDER_CORE);
	err = readl_poll_timeout(apple->fender + FENDER_POWER_STATE, val,
				 !(val & FENDER_POWER_REQUEST), 5, T7000_FENDER_POLL_US);
	if (!err) {
		mutex_lock(&apple->perf_lock);
		err = t7000_fender_state(apple, 0);
		mutex_unlock(&apple->perf_lock);
	}
	if (err) {
		t7000_stuck(apple, "Fender stuck at the stop");
		return err;
	}
	writel(FENDER_STREAM_END, apple->fender + FENDER_STREAM);
	/*
	 * Apple checks before it gates the GPU (setGFXPower(0), isOff): state 0
	 * and no kick, with TIMER_SLEEP_CONTROL bits 29 and 30 (full off) or
	 * bit 30 alone. Logged on the A7, whose domain goes off next.
	 */
	a7_dbg(s, "stopped: Fender %#x sleep %#x kick %#x int %#x; the power domain goes off next\n",
		readl(apple->fender + FENDER_POWER_STATE), readl(apple->fender + FENDER_CTRL),
		readl(apple->fender + 0x8), readl(apple->fender + 0x14));
	/* and the GPU's power state off, as the module ends every run (the A7's domain: at suspend) */
	return s->v->soc->ps_domain ? 0 : t7000_ps_set(apple, 0);
}

/*
 * Boot the META (Apple's prepareBoot 0x846f044 and start 0x846eb10): with
 * the core in state 0, store the boot stream (reset, boot mode, the page
 * catalogue of GPU context 0 in BIF_CAT_BASE0 as configureMMUForBoot
 * 0x8477e8c does, the SLC setup of populateInitSequence 0x8477eb0; no spare
 * page), then release the core word and power the core up in state @q.
 * Fender replays the stream, whose last write releases the reset.
 */
static int t7000_release(struct pvr_apple *apple, u32 q)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_stream stream = { .apple = apple };
	u32 val;
	int err;

	if (s->stuck)
		return -EIO;
	mutex_lock(&apple->perf_lock);
	err = t7000_fender_state(apple, 0);
	mutex_unlock(&apple->perf_lock);
	if (err) {
		t7000_stuck(apple, "Fender stuck before the boot");
		return err;
	}
	msleep(T7000_SETTLE_MS);
	(void)readl(apple->fender + FENDER_STREAM_STATE);
	writel(0, apple->fender + FENDER_STREAM_STATE);
	(void)readl(apple->fender + FENDER_STREAM_STATE);
	writel(FENDER_CTRL_BOOT, apple->fender + FENDER_CTRL);
	t7000_stream_write64(&stream, ROGUE_CR_SOFT_RESET, S8003_SOFT_RESET_ALL);
	t7000_stream_write64(&stream, ROGUE_CR_SOFT_RESET, S8003_SOFT_RESET_CORE);
	t7000_stream_write64(&stream, ROGUE_CR_SOFT_RESET, ROGUE_CR_SOFT_RESET_GARTEN_EN);
	t7000_stream_write32(&stream, ROGUE_CR_META_BOOT, ROGUE_CR_META_BOOT_MODE_EN);
	t7000_stream_write64(&stream, ROGUE_CR_BIF_CAT_BASE0,
			     pvr_vm_get_page_table_root_addr(apple->pvr_dev->kernel_vm_ctx));
	if (s->v->soc->slc) {
		t7000_stream_write64(&stream, ROGUE_CR_SLC_CTRL_MISC, 0x200001);
		t7000_stream_write64(&stream, T7000_CR_SLC_3808, 1);
	}
	t7000_stream_write64(&stream, ROGUE_CR_SOFT_RESET, 0);
	t7000_stream_word(&stream, FENDER_STREAM_END);
	if (stream.err) {
		dev_err(apple_dev(apple), "Fender did not take the boot stream: %d\n", stream.err);
		return stream.err;
	}
	wmb();	/* the image and init data are in memory before the META runs */

	msleep(T7000_SETTLE_MS);
	writel(0, apple->fender + FENDER_CORE);
	err = readl_poll_timeout(apple->fender + FENDER_POWER_STATE, val,
				 !(val & FENDER_POWER_REQUEST), 5, T7000_FENDER_POLL_US);
	if (err) {
		t7000_stuck(apple, "Fender busy before the release");
		return err;
	}
	/* As Apple's start: the request, then the alive event is waited for in memory */
	mutex_lock(&apple->perf_lock);
	writel(FENDER_POWER_REQUEST | q, apple->fender + FENDER_POWER_STATE);
	apple->perf_state = q;
	mutex_unlock(&apple->perf_lock);
	return 0;
}

/* Apple panics in every caller of its alive wait (0x8471040, 0x846dc1c, 0x846f9a0) */
static void t7000_no_alive(struct pvr_apple *apple)
{
	t7000_stuck(apple, "no alive event from the firmware");
}

/* After the stop, for runtime suspend: the GPU's power state goes off */
static void t7000_idle(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (!s->stuck && !s->v->soc->ps_domain && t7000_ps_on(s))
		t7000_ps_set(apple, 0);
	if (!s->v->soc->ps_domain)
		return;
	/*
	 * The A7: its power domain goes off after this. Fender's state 0
	 * hands the GPU's rail down to the DVFS sequencer, which talks to the
	 * PMIC over DWI: give it time before the power gate (Apple's host idle
	 * timer waits too).
	 */
	a7_dbg(s, "idle: power state %#x, dwi %#x\n", pmgr_read(s, s->v->soc->ps_gfx),
	       pmgr_read(s, 0x20038));
	msleep(20);
}


/*
 * Apple's kick, the same in all five of its MTS_SCHEDULE writers (TA
 * 0x846f220, 3D 0x846f408, CL 0x846f5f0, device control 0x846f9a0; COUNTED
 * and the data master): Fender's wake (+0x14 = 7, 0x847a640), the power
 * state read (+0x1000 & 0xf, 0x8477020), then the META's scheduler
 * register, written in every case. Only in state 0 (the firmware idled the
 * core) does Apple look further: with Fender's control 0 it restarts the
 * firmware (0x846dc1c), with the snoop not armed it warns; with the wake
 * write Fender wakes the core (on the iPod: state 1 and the PLL locked
 * within 1 ms). In state 1 Apple reads no PLL: the firmware's own power
 * and clock changes (its idle gating, DVFS) can leave PLL5 relocking while
 * the state reads 1 (iOS shows that in 30 of 200 draw rounds), and Apple
 * rings then too. So does this driver; such doorbells are counted. It
 * still does not write the register with Fender's control 0 or the snoop
 * not armed in state 0 (never seen here; Apple's restart is not
 * implemented): the job is then in the ring and its wait ends in the reset.
 *
 * Nothing here fails: a job's publish checks first (t7000_publish_check()).
 *
 * Return: whether the doorbell found Fender in state 0 (statistics).
 */
static bool t7000_ring(struct pvr_apple *apple, u32 dm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 state, pll = 0, ctrl = 0, snoop;

	if (s->stuck) {
		atomic64_inc(&s->doorbell_refused);
		dev_err_ratelimited(apple_dev(apple),
				    "doorbell not rung: the GPU is stuck; the job waits for the reset\n");
		return false;
	}
	if (s->v->soc->ps_domain && !t7000_ps_on(s)) {
		atomic64_inc(&s->doorbell_refused);
		dev_err_ratelimited(apple_dev(apple),
				    "doorbell not rung: the GPU's power domain is off\n");
		return false;
	}
	mb();	/* what the firmware is rung for is in memory */
	writel(FENDER_WAKE_ALL, apple->fender + FENDER_WAKE);
	state = readl(apple->fender + FENDER_POWER_STATE);
	atomic64_inc(&s->doorbells);
	if (!(state & 0xf)) {
		atomic64_inc(&s->doorbell_wakes);
		ctrl = readl(apple->fender + FENDER_CTRL);
		snoop = readl(apple->fender + T7000_FENDER_SNOOP);
		if (!ctrl || (!(ctrl & T7000_FENDER_CTRL_SNOOP) && !(snoop & BIT(1)))) {
			atomic64_inc(&s->doorbell_refused);
			dev_err_ratelimited(apple_dev(apple),
					    "doorbell not rung: Fender state %#x, control %#x, snoop %#x; the job waits for the reset\n",
					    state, ctrl, snoop);
			return true;
		}
	} else {
		/* statistics: Apple reads no PLL here */
		pll = pmgr_read(s, s->v->soc->pll);
		if (!(pll & s->v->soc->pll_lock)) {
			atomic64_inc(&s->doorbell_pll_unlocked);
			dev_info_ratelimited(apple_dev(apple),
					     "doorbell rung with PLL5 relocking, as Apple's kext rings: Fender state %#x, PLL5 %#x\n",
					     state, pll);
		}
	}
	pvr_cr_write32(apple->pvr_dev, ROGUE_CR_MTS_SCHEDULE, ROGUE_CR_MTS_SCHEDULE_TASK_COUNTED | dm);
	return !(state & 0xf);
}

/* The variant's doorbell, for device-control commands */
static int t7000_doorbell(struct pvr_apple *apple, u32 dm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (s->stuck)
		return -EIO;
	t7000_ring(apple, dm);
	return 0;
}

/* The hardware objects at their addresses in a VM */
static int t7000_vm_map(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int i;
	int err = 0;

	for (i = 0; !err && i < T7000_HW_OBJS; i++) {
		if (!s->hwobj[i].obj)
			return -ENODEV;
		err = pvr_vm_map_reserved(vm_ctx, s->hwobj[i].obj,
					  T7000_HWOBJ_VA + t7000_hwobj_offset[i], T7000_HWOBJ_SIZE);
	}
	return err;
}

static void t7000_vm_unmap(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int i;

	for (i = 0; i < T7000_HW_OBJS; i++)
		if (s->hwobj[i].obj)
			pvr_vm_unmap_obj(vm_ctx, s->hwobj[i].obj,
					 T7000_HWOBJ_VA + t7000_hwobj_offset[i], T7000_HWOBJ_SIZE);
}

/*
 * The last job of @engine that is done: its finish stamp at its number or
 * past it. The finish stamp is the last word a job's microsequence writes
 * (TA 0x24, 3D 0x26, CU 0x33), and what the bring-up module waits for; the
 * mirror stamp and the stamp event follow it.
 */
static u32 t7000_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 latest = s8003_submitted(apple, engine);
	u32 stamp = engine == APPLE_ENGINE_COMPUTE ? S8003_CU_STAMP : S8003_CH_STAMP;
	s32 behind;

	if (!READ_ONCE(apple->booted) || !s->root.map)
		return latest;
	behind = get32(s8003_channel(s, engine) + stamp + 4) - latest;
	return latest + min(behind, 0);
}

/*
 * Everything that can refuse a job's publish, checked before the job's
 * record, numbers or VM state move: a refused job leaves nothing the
 * firmware could run. Once, a publish refused after the producer index was
 * written (the doorbell's PLL check) left the 3D job in the work ring; the
 * firmware took it un-rung while its fence had already failed, and the 3D
 * engine faulted in the job's context. The work ring's slot is free once
 * the firmware took the previous entry (it takes one at once); submit_lock
 * keeps it so until t7000_publish().
 */
static int t7000_publish_check(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_WORK_CTL + 4 * s8003_engine(engine));
	u32 start = ring_read(ctl, S8003_RING_PRODUCER), consumer;

	lockdep_assert_held(&apple->submit_lock);
	if (s->stuck)
		return -EIO;
	if (start >= APPLE_RING_ENTRIES)
		return -EPROTO;
	return read_poll_timeout(s8003_ring_consumer, consumer, consumer == start, 20,
				 S8003_RING_TIMEOUT_US, false, apple, ctl);
}

/*
 * Tell the firmware that @engine's pointer ring is filled up to @end: an
 * entry of the engine's work ring {engine, channel, end}, with Apple's 1 in
 * its fourth word on a render channel (P5b), the host's watermark, the work
 * ring's producer index, and Apple's kick with the engine's data master (TA
 * DM0, 3D DM1, CL DM2: 0x846f220, 0x846f408, 0x846f5f0). After
 * t7000_publish_check() under the same hold of submit_lock; it does not
 * fail: once the producer index is written the job is the firmware's, and
 * its fence follows the firmware's progress, a fault or the reset.
 */
static void t7000_publish(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 end)
{
	static const u32 dm[] = {
		ROGUE_CR_MTS_SCHEDULE_DM_DM0, ROGUE_CR_MTS_SCHEDULE_DM_DM1, ROGUE_CR_MTS_SCHEDULE_DM_DM2,
	};
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 fw_engine = s8003_engine(engine);
	u8 *ctl = s8003_object(s, S8003_WORK_CTL + 4 * fw_engine);
	u8 *ring = s8003_object(s, S8003_WORK_RING + 4 * fw_engine);
	u32 host = engine == APPLE_ENGINE_COMPUTE ? S8003_CH_CTL : T7000_RCH_CTL;
	u32 start = ring_read(ctl, S8003_RING_PRODUCER);
	u8 *p = s8003_channel(s, engine), *entry;

	entry = ring + start * S8003_DC_SIZE;
	memset(entry, 0, S8003_DC_SIZE);
	put32(entry, fw_engine);
	put32(entry + 4, s8003_channel_addr(s, engine));
	put32(entry + 8, end);
	if (engine != APPLE_ENGINE_COMPUTE)
		put32(entry + 12, 1);
	wmb();
	put32(p + host + S8003_CH_CTL_HOST, end);
	wmb();
	WRITE_ONCE(*(__le32 *)(ctl + S8003_RING_PRODUCER),
		   cpu_to_le32((start + 1) % APPLE_RING_ENTRIES));
	s->t7000_kick[engine] = arch_timer_read_counter();
	s->t7000_kick_woke[engine] = t7000_ring(apple, dm[fw_engine]);
}

/* The A8's compute record: the A9's with every offset from 0x10 on 8 further */
#define T7000_CU_W_SIZE			0x298
#define T7000_CU_TERMINATE1		0x00900004	/* the terminate program's descriptor (A9 0x00a00004) */

/*
 * A compute job on its VM's slot, one at a time: the record and the
 * microsequence word for word as Apple's in P5's CL slots (start 0x32 with
 * the stamp's previous value k twice, the timestamps around the wait for
 * the engine's finished event, finish 0x33), userspace's registers placed
 * as the bring-up module ran Mesa's records: the context store, resume
 * and terminate programs and their descriptors, the configuration word,
 * the context state and the control stream with its terminate word.
 */
static int t7000_submit_compute(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	const struct pvr_apple_g5p_cmd_compute *cmd = job->cmd;
	const struct pvr_apple_g5p_compute_regs *g = &cmd->g5p;
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 addr = s8003_channel_addr(s, APPLE_ENGINE_COMPUTE);
	u32 global = s8003_object_addr(&s->layout, S8003_CHANNEL + 2);
	u32 w = addr + S8003_CH_W, stamp = addr + S8003_CU_STAMP, slot = vm->slot[0], seq, k;
	u8 *p = s8003_channel(s, APPLE_ENGINE_COMPUTE);
	u8 *work = p + S8003_CH_W;
	unsigned int i;
	int err;

	err = s8003_wait_done(apple, APPLE_ENGINE_COMPUTE, apple->last_seq);
	if (err)
		return s8003_submit_error(apple, vm, "compute", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");
	err = t7000_publish_check(apple, APPLE_ENGINE_COMPUTE);
	if (err)
		return s8003_submit_error(apple, vm, "compute", err, S8003_REFUSED_PUBLISH,
					  "publish");
	seq = apple->last_seq + 1;
	k = seq - 1;
	{
		const u32 useq[] = {
			0x32, w + 0x18, w + 0x68, global + 0x720, addr, slot, k, k, 0, w + 0x1f0,
			1, 0, seq,
			0x80000018, w + 0x280, w + 0x288, w + 0x288, seq,
			0x101, 0x004a000a, 1, 1,
			0x18, w + 0x280, w + 0x288, w + 0x28c, seq,
			0x33, global + 0x720, addr, slot, w + 0x1f0, 1, 0, seq,
			stamp + 4, seq, (u32)-0x6c,
			0x40000017,
		};

		BUILD_BUG_ON(sizeof(useq) > S8003_CU_USEQ_SIZE);
		memset(p + S8003_CU_USEQ, 0, S8003_CU_USEQ_SIZE);
		useq_put(p + S8003_CU_USEQ, useq, ARRAY_SIZE(useq));
	}
	memset(work, 0, T7000_CU_W_SIZE);
	put32(work, 3);
	put32(work + 8, slot);
	put32(work + 0xc, addr + S8003_CU_DEP);
	put32(work + 0x18, g->config[0]);
	put32(work + 0x1c, g->config[1]);
	put64(work + 0x68, g->tail);
	put64(work + 0x70, g->pds_resume);
	put64(work + 0x78, cmd->cmd.regs.cdm_ctrl_stream_base);
	put32(work + 0x1e8, addr + S8003_CU_USEQ);
	put32(work + 0x1ec, S8003_CU_USEQ_SIZE);
	for (i = 0; i < 4; i++)
		put64(work + 0x1f4 + i * 8, g->state[i]);
	put64(work + 0x214, g->tail);
	put64(work + 0x21c, g->terminate);
	put64(work + 0x224, g->pds_store);
	put32(work + 0x22c, g->pds_store1);
	put64(work + 0x234, g->pds_resume);
	put32(work + 0x23c, g->pds_resume1);
	put64(work + 0x244, g->pds_terminate);
	put32(work + 0x24c, T7000_CU_TERMINATE1);
	put32(work + 0x260, stamp);
	put32(work + 0x264, stamp + 4);
	put32(work + 0x268, seq);
	put32(work + 0x278, seq);
	put32(work + 0x27c, k);
	put32(p + S8003_CH_RING + ((seq - 1) % S8003_CH_ENTRIES) * 4, w);
	apple->last_seq = seq;
	t7000_publish(apple, APPLE_ENGINE_COMPUTE, seq % S8003_CH_ENTRIES);
	return seq;
}

/*
 * Render jobs. The A8's parameter manager keeps one set per VM: a control
 * block with 36 state records "S" of 0xc0 bytes behind its first 0x80 (root
 * memory), a list of the pool's pages in slices of 32 with a link to the
 * previous slice (shared memory), and in the VM the PM's own tables (36
 * macrotile arrays, 36 MLists, the vheap) and the free-list page stack: a
 * u32 page number array the host fills on the CPU, which the firmware loads
 * into the PM at every TA start (0x1001a418 -> 0x10025a28). Without the
 * stack the TA faulted at VA 0x180 in the PM's context. P5b shows Apple's
 * fresh queue built this way (kext 0x847f58c, 0x8480a28, 0x847fef0); its
 * first frame uses S[1] for the TA and the 3D job. The pool's pages are
 * never mapped: the PM writes their numbers to the hardware.
 */
#define T7000_PM_S_COUNT		36
#define T7000_PM_S_SIZE			0xc0
#define T7000_PM_SLICE			0x84
#define T7000_PM_SLICE_PAGES		32
/*
 * The pool, in 128 KiB blocks (a slice of 32 pages each). Apple's fresh
 * queue starts at 2 and its steady PMs held 8 (0x100 pages), as dumped:
 * iOS grows the pool, from the firmware's feedback between scenes and with
 * tag-5 grow records ("Synchronous PB Grow" in the kext's firmware
 * state dump), and this driver has no grow inside a scene and no partial
 * render. So it starts larger (a8_pm_blocks; on the iPod, 2 blocks held a
 * 256x256 mesh of 8192 small triangles but not 16384, some 20-32 bytes a
 * triangle, so 32 blocks hold about 128k), grows from the feedback
 * between scenes as the A9 does, and doubles after a stall for
 * pages. The most is what one set's list page holds: 61 slices of 0x84
 * after its 0x80 header (Apple sizes its list for 0x300 blocks).
 */
#define T7000_PM_BLOCKS_MAX		((T7000_PM_SET_SIZE - 0x80) / T7000_PM_SLICE)
#define T7000_PM_FEEDBACK		0x40	/* list: high-water pages, margin, -, request */
#define T7000_PM_FEEDBACK_TAKEN		0x60	/* list: 1 once the host took them */
static unsigned int a8_pm_blocks = 32;
module_param(a8_pm_blocks, uint, 0644);
MODULE_PARM_DESC(a8_pm_blocks, "A8: a new VM's PM pool in 128 KiB blocks (2..61, default 32 = 4 MiB)");
/*
 * Growing a live VM's pool rebuilds its set under the same PM id while the
 * firmware may hold that id loaded; whether a TA start reloads the stack
 * then is not read yet (10007eec, 10007fd8), and the A8's usage words are
 * the A9's by analogy. Until a test shows both, the feedback is only
 * logged, and a stall raises the pool of later VMs only.
 */
static bool a8_pm_grow;
module_param(a8_pm_grow, bool, 0644);
MODULE_PARM_DESC(a8_pm_grow, "A8: grow a live VM's PM pool from the firmware's feedback (default 0: log the feedback only)");
#define T7000_PM_MACROTILE		0x80
#define T7000_PM_MLIST			0x10000
#define T7000_PM_VHEAP			0x600
#define T7000_PM_AUX_SIZE		ALIGN(T7000_PM_S_COUNT * T7000_PM_MACROTILE + \
					      T7000_PM_S_COUNT * T7000_PM_MLIST + T7000_PM_VHEAP, SZ_4K)
#define T7000_PM_STACK_SIZE		0x18000	/* 0x300 blocks x 32 pages (PM object +0x28) */
#define T7000_PM_SCENE_S		1
/* In every VM, behind the general heap (t7000_heaps) */
#define T7000_PM_AUX_VA			0x1f0000000ull
#define T7000_PM_STACK_VA		0x1f0400000ull

static u64 t7000_pm_mlist(unsigned int i)
{
	return T7000_PM_AUX_VA + T7000_PM_S_COUNT * T7000_PM_MACROTILE + (u64)i * T7000_PM_MLIST;
}

/* The VM's set at its slot's place in the root and shared memory */
static u32 t7000_pm_control_va(const struct s8003_vm *vm)
{
	return T7000_PM_CONTROL + (vm->slot[0] - T7000_SLOT_FIRST) * T7000_PM_SET_SIZE;
}

static u32 t7000_pm_list_va(const struct s8003_vm *vm)
{
	return T7000_PM_LIST + (vm->slot[0] - T7000_SLOT_FIRST) * T7000_PM_SET_SIZE;
}

/*
 * The VM's PM id: one per set, so one per live VM. The firmware loads a
 * set's free list into the PM only when a job names an id other than the
 * one that PM side last loaded, after saving the running set back to its
 * control block (0x10007fd8). Apple's kext gives each live PM its own id,
 * out of the firmware table's 127 (assignPMId, 0xfffffff008486614).
 */
static u32 t7000_pm_id(const struct s8003_vm *vm)
{
	return vm->slot[0] - T7000_SLOT_FIRST;
}

static u32 t7000_pm_s(const struct s8003_vm *vm)
{
	return s8003_meta_addr(S8003_ROOT_VA + t7000_pm_control_va(vm)) + 0x80 +
	       T7000_PM_SCENE_S * T7000_PM_S_SIZE;
}

/*
 * Build the VM's PM set for this start of the firmware (a8_pm_fill(): P5b's
 * fresh queue, its control fields, the slices, S records with the VM's
 * tables, the stack from index 0, slice 0 first). The usage word's flag
 * (list +0x60) is 1: the host took the firmware's feedback.
 */
static int t7000_pm_build(struct pvr_apple *apple, struct s8003_vm *vm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *control = s->root.map + t7000_pm_control_va(vm);
	u8 *list = s->shared.map + t7000_pm_list_va(vm);
	u32 list_meta = s8003_meta_addr(S8003_SHARED_VA + t7000_pm_list_va(vm));
	u32 blocks = vm->t7000.blocks, pages = blocks * T7000_PM_SLICE_PAGES;
	u8 *stack = vm->t7000.stack_map;
	unsigned int i, j, k;
	int err;

	if (!blocks || blocks > T7000_PM_BLOCKS_MAX)
		return -EINVAL;
	vm->t7000.built = 0;
	s->t7000_pm_builds++;
	memset(stack, 0, T7000_PM_STACK_SIZE);
	/* The stack first: the pool's page numbers from index 0 (0x847fef0) */
	for (i = 0; i < pages; i++) {
		dma_addr_t page;

		err = pvr_gem_get_dma_addr(vm->t7000.pool, i * SZ_4K, &page);
		if (err)
			return err;
		put32(stack + 4 * i, page >> 12);
	}
	memset(vm->t7000.pool_map, 0, (size_t)pages * SZ_4K);
	memset(vm->t7000.aux_map, 0, T7000_PM_AUX_SIZE);
	memset(control, 0, T7000_PM_SET_SIZE);
	memset(list, 0, T7000_PM_SET_SIZE);
	for (j = 0; j < blocks; j++) {
		u8 *slice = list + 0x80 + j * T7000_PM_SLICE;

		put32(slice, j ? list_meta + 0x80 + (j - 1) * T7000_PM_SLICE : 0);
		for (k = 0; k < T7000_PM_SLICE_PAGES; k++)
			put32(slice + 4 + 4 * k, get32(stack + 4 * (j * T7000_PM_SLICE_PAGES + k)));
	}
	put32(control + 0xc, 0x7f);
	put32(control + 0x14, list_meta + 0x80 + (blocks - 1) * T7000_PM_SLICE);
	put64(control + 0x18, T7000_PM_STACK_VA);
	put32(control + 0x20, T7000_PM_STACK_SIZE);
	put32(list + T7000_PM_FEEDBACK_TAKEN, 1);
	put32(control + 0x24, pages);
	put32(control + 0x28, pages - 1);
	put32(control + 0x38, SZ_4K);
	put32(control + 0x44, list_meta);
	put32(control + 0x58, 1);
	put32(control + 0x5c, 0x6000);
	put32(control + 0x60, 0x2000);
	for (i = 0; i < T7000_PM_S_COUNT; i++) {
		u8 *rec = control + 0x80 + i * T7000_PM_S_SIZE;

		put64(rec + 0xa0, t7000_pm_mlist(i));
		put64(rec + 0xa8, T7000_PM_AUX_VA + i * T7000_PM_MACROTILE);
		put64(rec + 0xb0, t7000_pm_mlist(T7000_PM_S_COUNT));
		put32(rec + 0xb8, list_meta + 0x40);
	}
	wmb();
	vm->t7000.built = s->gen;
	return 0;
}

static void t7000_pm_obj_put(struct pvr_apple *apple, struct pvr_gem_object **obj, u8 **map)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (!*obj)
		return;
	atomic_dec(&s->pm_objs_live);
	if (*map)
		pvr_gem_object_vunmap(*obj);
	pvr_gem_object_put(*obj);
	*obj = NULL;
	*map = NULL;
}

/* A VM's PM objects go with it; their VM mappings with the VM */
static void t7000_vm_put(struct pvr_apple *apple, struct s8003_vm *vm)
{
	t7000_pm_obj_put(apple, &vm->t7000.aux, &vm->t7000.aux_map);
	t7000_pm_obj_put(apple, &vm->t7000.stack, &vm->t7000.stack_map);
	t7000_pm_obj_put(apple, &vm->t7000.pool, &vm->t7000.pool_map);
	t7000_pm_obj_put(apple, &vm->t7000.next_pool, &vm->t7000.next_pool_map);
}

static int t7000_pm_obj(struct pvr_apple *apple, size_t size, struct pvr_gem_object **obj, u8 **map)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	*obj = pvr_gem_object_create(apple->pvr_dev, size, T7000_VM_FLAGS);
	if (IS_ERR(*obj)) {
		int err = PTR_ERR(*obj);

		*obj = NULL;
		return err;
	}
	atomic_inc(&s->pm_objs_live);
	*map = pvr_gem_object_vmap(*obj);
	if (IS_ERR(*map)) {
		int err = PTR_ERR(*map);

		*map = NULL;
		t7000_pm_obj_put(apple, obj, map);
		return err;
	}
	return 0;
}

/*
 * Region headers as a mirrored ring (Apple's resource with flag 0x200000,
 * mapped page by page at VA and VA + its length, 0x847d738): a buffer of
 * R bytes mapped twice back to back, of 16 slices of the target's region
 * headers rounded up to 4 KiB at least. Each frame takes the next slice;
 * slices are never cleared. A dataset's ring: where it starts, R, the
 * slice, and the dataset's cursor (Apple's RTM +0x19c), kept in the data set
 * itself (&pvr_hwrt_dataset.apple_rgn) for as long as it exists.
 */

static int t7000_rgn_ring(struct pvr_vm_context *vm_ctx, u64 base, u32 slice, u64 *size)
{
	struct pvr_gem_object *obj, *mirror;
	u64 offset, range, mirror_offset, mirror_range;
	int err = -EINVAL;

	obj = pvr_vm_find_gem_object(vm_ctx, base, &offset, &range);
	if (!obj)
		return -EINVAL;
	mirror = pvr_vm_find_gem_object(vm_ctx, base + range, &mirror_offset, &mirror_range);
	if (mirror == obj && !offset && !mirror_offset && mirror_range == range &&
	    range == pvr_gem_object_size(obj) && is_power_of_2(range) &&
	    range >= roundup_pow_of_two(16 * slice)) {
		*size = range;
		err = 0;
	}
	if (mirror)
		pvr_gem_object_put(mirror);
	pvr_gem_object_put(obj);
	return err;
}

/*
 * Before a geometry job, in the submitting process: the target (one sample
 * so far), its region-header ring, and the VM's PM objects, mapped in the VM
 * once (the stack, the PM's tables) or kept unmapped (the pool).
 */
static int t7000_prepare_job(struct pvr_apple *apple, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm;
	struct pvr_hwrt_dataset *dataset;
	struct s8003_rt rt;
	u64 ring;
	u32 slice, blocks;
	int err;

	if (job->type != DRM_PVR_JOB_TYPE_GEOMETRY)
		return 0;
	vm = xa_load(&s->vms, (unsigned long)job->ctx->vm_ctx);
	if (!vm)
		return -EINVAL;
	dataset = job->hwrt->hwrt_dataset;
	err = s8003_rt_derive(dataset, &rt);
	if (err) {
		atomic64_inc(&s->refused[S8003_REFUSED_PREPARE_RT]);
		return err;
	}
	if (dataset->samples != 1) {
		atomic64_inc(&s->refused[S8003_REFUSED_PREPARE_SAMPLES]);
		return -EOPNOTSUPP;
	}
	slice = ALIGN(rt.header, SZ_4K);
	err = t7000_rgn_ring(vm->vm_ctx, job->hwrt->data.rgn_header_dev_addr, slice, &ring);
	if (err) {
		atomic64_inc(&s->refused[S8003_REFUSED_PREPARE_RING]);
		dev_warn_ratelimited(apple_dev(apple),
				     "geometry job refused: region headers at %#llx are not a mirrored ring of 16 slices of %#x\n",
				     job->hwrt->data.rgn_header_dev_addr, slice);
		return err;
	}

	/*
	 * The submitting client's own context: a holder may wait on the GPU
	 * (up to the job timeout under submit_lock), so a killed client
	 * leaves instead of sleeping uninterruptibly behind it.
	 */
	err = mutex_lock_killable(&vm->prepare_lock);
	if (err)
		return err;
	if (!vm->t7000.stack) {
		struct pvr_gem_object *aux = NULL, *stack = NULL, *pool = NULL;
		u8 *aux_map = NULL, *stack_map = NULL, *pool_map = NULL;

		err = t7000_pm_obj(apple, T7000_PM_AUX_SIZE, &aux, &aux_map);
		if (!err)
			err = t7000_pm_obj(apple, T7000_PM_STACK_SIZE, &stack, &stack_map);
		blocks = clamp(max(READ_ONCE(a8_pm_blocks), READ_ONCE(s->t7000_pm_floor)), 2u,
			       (u32)T7000_PM_BLOCKS_MAX);
		if (!err)
			err = t7000_pm_obj(apple, (size_t)blocks * T7000_PM_SLICE_PAGES * SZ_4K,
					   &pool, &pool_map);
		if (!err)
			err = pvr_vm_map_reserved(vm->vm_ctx, aux, T7000_PM_AUX_VA, T7000_PM_AUX_SIZE);
		if (!err) {
			err = pvr_vm_map_reserved(vm->vm_ctx, stack, T7000_PM_STACK_VA,
						  T7000_PM_STACK_SIZE);
			if (err)
				pvr_vm_unmap_obj(vm->vm_ctx, aux, T7000_PM_AUX_VA, T7000_PM_AUX_SIZE);
		}
		if (err) {
			t7000_pm_obj_put(apple, &aux, &aux_map);
			t7000_pm_obj_put(apple, &stack, &stack_map);
			t7000_pm_obj_put(apple, &pool, &pool_map);
			goto out;
		}
		mutex_lock(&apple->submit_lock);
		vm->t7000.aux = aux;
		vm->t7000.aux_map = aux_map;
		vm->t7000.stack = stack;
		vm->t7000.stack_map = stack_map;
		vm->t7000.pool = pool;
		vm->t7000.pool_map = pool_map;
		vm->t7000.blocks = blocks;
		vm->t7000.want = blocks;
		mutex_unlock(&apple->submit_lock);
	}
	/*
	 * A larger pool the feedback or a stall asked for: allocated here, in
	 * the submitting process, and taken by a geometry job once the scenes
	 * on the old one are done (t7000_submit_geom()).
	 */
	mutex_lock(&apple->submit_lock);
	blocks = max(vm->t7000.want, READ_ONCE(s->t7000_pm_floor));
	blocks = !READ_ONCE(a8_pm_grow) || vm->t7000.next_pool || blocks <= vm->t7000.blocks ? 0 :
		 min_t(u32, blocks, T7000_PM_BLOCKS_MAX);
	mutex_unlock(&apple->submit_lock);
	if (blocks) {
		struct pvr_gem_object *pool = NULL;
		u8 *pool_map = NULL;

		if (!t7000_pm_obj(apple, (size_t)blocks * T7000_PM_SLICE_PAGES * SZ_4K, &pool,
				  &pool_map)) {
			mutex_lock(&apple->submit_lock);
			if (!vm->t7000.next_pool) {
				vm->t7000.next_pool = pool;
				vm->t7000.next_pool_map = pool_map;
				vm->t7000.next_blocks = blocks;
				pool = NULL;
			}
			mutex_unlock(&apple->submit_lock);
			t7000_pm_obj_put(apple, &pool, &pool_map);
		}
	}
	/*
	 * The ring lives with the data set, which this job holds until it is
	 * freed: no other target's job can take it before this one is
	 * submitted. Its cursor starts at 0 with the data set.
	 */
	mutex_lock(&apple->submit_lock);
	dataset->apple_rgn.ring = ring;
	dataset->apple_rgn.slice = slice;
	mutex_unlock(&apple->submit_lock);
out:
	mutex_unlock(&vm->prepare_lock);
	return err;
}

/* The A8's records */
#define T7000_TA_W_SIZE			0x568
#define T7000_3D_W_SIZE			0x8b0
#define T7000_TA_USEQ_SIZE		0xc0
#define T7000_3D_USEQ_SIZE		0x140

/*
 * A geometry job's record: the A9's fill moved to the A8's offsets (+8 from
 * 0x20 and from 0x3a4), with Apple's A8 values where they differ from the
 * A9's (the TA scratch base, the resume and TA PDS words and three scratch
 * pointers are zero, +0x3c8 is 0x22000, +0x530 is 0), and the region
 * headers' slice of this frame. The values are those of the record the
 * first triangle drew with (Mesa's twiddled-triangle job, recorded on
 * the iPod touch 6).
 */
static void t7000_fill_ta(u8 *w, const struct t7000_soc *soc, const struct pvr_apple_g5p_cmd_geom *cmd,
			  const struct pvr_hwrt_data *hwrt, const struct s8003_rt *rt, u64 rgn)
{
	const struct pvr_apple_g5p_geom_regs *g = &cmd->g5p;
	u64 tpc = hwrt->data.tail_ptrs_dev_addr;
	unsigned int i;

	memset(w, 0, T7000_TA_W_SIZE);
	put32(w + 0x28, 0x1e3ce508);
	put32(w + 0x2c, 0x1e3ce508);
	put64(w + 0x30, rgn);
	put64(w + 0x38, tpc);
	put64(w + 0x40, rt->ta_state);
	put64(w + 0x48, g->sample_positions ?: rt->positions);
	put64(w + 0x50, g->scratch_ptr[4]);
	put64(w + 0x58, g->scratch_ptr[5]);
	for (i = 0; i < 6; i++) {
		put32(w + 0x60 + i * 4, g->context_pds[0][i]);
		put32(w + 0x78 + i * 4, g->context_pds[1][i]);
	}
	put64(w + 0x90, g->ta_program);
	put64(w + 0xa0, cmd->cmd.regs.vdm_ctrl_stream_base & ~3ull);
	put32(w + 0x3ac, 0x00000088);
	put32(w + 0x3b0, 0x00000002);
	put64(w + 0x3b4, rt->ta_tiling[0]);
	put64(w + 0x3bc, rt->ta_tiling[1]);
	put32(w + 0x3c4, rt->ta_tiling_word);
	put32(w + 0x3c8, soc->te_psg);
	put64(w + 0x44c, tpc);
	put32(w + 0x454, rt->tpc);
	put64(w + 0x470, g->scratch_ptr[0]);
	put64(w + 0x490, g->scratch_ptr[4]);
}

/*
 * A fragment job's record, the same way: the A9's fill at the A8's offsets,
 * without the scale words (zero in Apple's records), Apple's ISP and output
 * words for one sample and an output width of 2 (+0x3c8..+0x3d4), and the
 * render targets' PBE register words from the Rogue command (set A at
 * +0x118 and +0x4c0, set B at +0x540, two qwords per target, as Apple's
 * fillers 0x8475dbc and 0x844aeb0 write them).
 */
static int t7000_fill_3d(u8 *w, const struct t7000_soc *soc, const struct pvr_apple_g5p_cmd_frag *cmd,
			 const struct s8003_rt *rt, u64 rgn)
{
	/* The register block's ZLS images and their copies further on (as recorded from the first triangle) */
	static const u16 zls[5][2] = {
		{ 0x80, 0x618 }, { 0x88, 0x628 }, { 0x90, 0x630 }, { 0x98, 0x648 }, { 0xa0, 0x650 },
	};
	const struct rogue_fwif_frag_regs *regs = &cmd->cmd.regs;
	const struct pvr_apple_g5p_frag_regs *g = &cmd->g5p;
	u64 positions = g->sample_positions ?: rt->positions;
	unsigned int i;

	if ((g->pixel_output_width && g->pixel_output_width != 2) || g->flags)
		return -EINVAL;

	memset(w, 0, T7000_3D_W_SIZE);
	put64(w + 0x20, rgn);
	put64(w + 0x28, rt->positions);
	put32(w + 0x30, rt->state);
	put32(w + 0x34, rt->padded);
	put32(w + 0x38, rt->region);
	put64(w + 0x60, positions);
	put64(w + 0x68, regs->isp_scissor_base & ~3ull);
	put64(w + 0x70, regs->isp_dbias_base & ~3ull);
	put64(w + 0x78, regs->isp_oclqry_base);
	for (i = 0; i < ARRAY_SIZE(zls); i++) {
		put64(w + zls[i][0], g->zls[i]);
		put64(w + zls[i][1], g->zls[i]);
	}
	put32(w + 0xa8, rt->region >> 2);
	put32(w + 0xb0, g->background[0]);
	put32(w + 0xb4, g->background[1]);
	put64(w + 0xb8, g->background_texture);
	for (i = 4; i < 8; i++)
		put32(w + 0xc0 + (i - 4) * 4, g->background[i]);
	for (i = 0; i < 8; i++) {
		put64(w + soc->pbe_3d + i * 16, regs->pbe_word[i][0]);
		put64(w + soc->pbe_3d + 8 + i * 16, regs->pbe_word[i][1]);
		put64(w + 0x4c0 + i * 16, regs->pbe_word[i][0]);
		put64(w + 0x4c8 + i * 16, regs->pbe_word[i][1]);
		put64(w + 0x540 + i * 16, regs->pbe_word[i][0]);
		put64(w + 0x548 + i * 16, regs->pbe_word[i][1]);
	}
	put32(w + 0x3a8, rt->padded);
	put32(w + 0x3ac, regs->isp_bgobjdepth);
	put32(w + 0x3b0, regs->isp_bgobjvals);
	put32(w + 0x3b8, 0x0000001f);
	/* The kext's upload has the halves of the first end-of-tile pair swapped */
	put32(w + 0x3bc, g->eot[1]);
	put32(w + 0x3c0, g->eot[0]);
	put32(w + 0x3c4, g->eot[2]);
	put32(w + 0x3c8, 0x0001700f | BIT(17));	/* ISP_CTL; +PROCESS_EMPTY_TILES, as the A9 (0x0023000f) */
	put32(w + 0x3cc, 0x000001fc);
	put32(w + 0x3d4, soc->word_3d4);
	put64(w + 0x498, regs->isp_dbias_base);
	put64(w + 0x4a8, regs->isp_scissor_base);
	for (i = 0; i < 8; i++)
		put32(w + 0x5c0 + i * 4, g->background[i]);
	put64(w + 0x5e0, g->background_texture);
	put32(w + 0x608, g->restore[0]);
	put32(w + 0x60c, g->restore[1]);
	for (i = 0; i < 3; i++) {
		put32(w + 0x670 + i * 4, g->eot[i]);
		put32(w + 0x684 + i * 4, g->eot_resume[i]);
	}
	put32(w + 0x698, regs->isp_bgobjdepth);
	put32(w + 0x69c, regs->isp_bgobjvals);
	put32(w + 0x6a0, 2);
	put32(w + 0x858, 1);
	return 0;
}

/* What the 3D engine is waited for between a fragment job's start and finish (P5 ms_3d) */
static const u32 t7000_3d_wait[24] = {
	0x201, 0x138204, 0x10, 0xf, 0 /* the channel object + 0x724 */, 0x14, 0x109f, 0x104000a,
	0x1f, 0x1f, 0xf48008, 0x10000015, 4, 0, 0x14, 0x109f, 0x13000a, 0x4000, 0x4000, 0x138204,
	0x4000, 0x104000a, 8, 8,
};

/*
 * The kernel's part of a render job (a8_ta_write()/a8_3d_write(), Apple's
 * P5 form): the record's links (tag, slot, dependency object, PM set and
 * state record, microsequence, stamps, the trace ID Metal passes per job)
 * and the microsequence: start (TA 0x23, 3D 0x25 with 23 arguments), the
 * timestamps around the wait, finish (0x24, 0x26). @k is the stamp's value
 * before this job; the job's finish writes k + 1.
 *
 * The timestamps are Apple's op 0x18 pair with Apple's operands (TA
 * FUN_fffffff00848d3e0: {W+0x558, W+0x560, W+0x560} after 0x23, {W+0x558,
 * W+0x560, W+0x564} after the wait; 3D {W+0x8a0, W+0x8a8, W+0x8ac}): the
 * result word gets the work's duration, and the firmware stores the
 * absolute start and end where the two pointer words point. Both engines
 * point them at their channel's @times, the start at +8 and the end at +0,
 * as the A9's s8003_job_write() points its own pair at two times per job.
 */
struct t7000_rjob {
	u32 ch, w, useq, object, slot, pm_id, control, s, dep, seq, k, trace, times;
	u32 pr_ch, pr_index;
};

static void t7000_ta_write(u8 *work, u8 *useq, const struct t7000_rjob *j)
{
	const u32 w = j->w, g = j->object + 0x724;
	const u32 mirror = j->ch + S8003_CH_STAMP, finish = mirror + 4;
	const u32 words[] = {
		0x23, w + 0x3ac, w + 0x28, j->s, g, j->ch, j->slot, j->k, j->pm_id, 0, j->k, 0,
		w + 0x46c, 0, 0, 0, j->seq,
		0x80000018, w + 0x558, w + 0x560, w + 0x560, j->seq,
		0x801,
		0x18, w + 0x558, w + 0x560, w + 0x564, j->seq,
		0x24, j->s, j->control, g, j->ch, j->slot, 0, w + 0x46c, 0, 0, j->seq, finish,
		j->k + 1, (u32)-0x70,
		0x40000017,
	};

	BUILD_BUG_ON(sizeof(words) > T7000_TA_USEQ_SIZE);
	put32(work + 0x4, j->slot);
	put32(work + 0xc, j->dep);
	put32(work + 0x10, j->pm_id);
	put32(work + 0x14, j->control);
	put32(work + 0x18, j->s);
	put32(work + 0x45c, j->useq);
	put32(work + 0x460, T7000_TA_USEQ_SIZE);
	put32(work + 0x464, j->pr_ch);
	put32(work + 0x468, j->pr_index);
	put32(work + 0x51c, j->trace);
	put32(work + 0x534, mirror);
	put32(work + 0x538, finish);
	put32(work + 0x53c, j->k + 1);
	put32(work + 0x54c, j->seq);
	put32(work + 0x550, j->k);
	put32(work + 0x560, j->times + 8);
	put32(work + 0x564, j->times);
	memset(useq, 0, T7000_TA_USEQ_SIZE);
	useq_put(useq, words, ARRAY_SIZE(words));
}

static void t7000_3d_write(u8 *work, u8 *useq, const struct t7000_rjob *j)
{
	const u32 w = j->w, g = j->object + 0x724;
	const u32 mirror = j->ch + S8003_CH_STAMP, finish = mirror + 4;
	const u32 start[] = {
		0x25, w + 0x3a8, w + 0x60, j->s, g, w + 0x7d8, w + 0x7dc, w + 0x860, j->ch, w, j->slot,
		j->k, j->pm_id, 0, j->k, 0, w + 0x6c0, w + 0x894, 0, 0, 0, 0, 0, j->seq,
		0x80000018, w + 0x8a0, w + 0x8a8, w + 0x8a8, j->seq,
	};
	const u32 finish_words[] = {
		0x18, w + 0x8a0, w + 0x8a8, w + 0x8ac, j->seq,
		0x26, j->seq, finish, j->k + 1, j->s, j->control, 1, g, w + 0x860, w + 0x7d8, j->ch, w,
		j->slot, w + 0x6c0, 0, 0, 0, (u32)-0xe8,
		0x40000017,
	};
	u32 wait[ARRAY_SIZE(t7000_3d_wait)];
	unsigned int i;
	u8 *out;

	BUILD_BUG_ON(sizeof(start) + sizeof(wait) + sizeof(finish_words) > T7000_3D_USEQ_SIZE);
	for (i = 0; i < ARRAY_SIZE(wait); i++)
		wait[i] = i == 4 ? g : t7000_3d_wait[i];
	put32(work + 0x0, 1);
	put32(work + 0x4, j->slot);
	put32(work + 0xc, j->useq);
	put32(work + 0x10, T7000_3D_USEQ_SIZE);
	put32(work + 0x14, j->dep);
	put32(work + 0x18, j->control);
	put32(work + 0x1c, j->s);
	put32(work + 0x850, j->trace);
	put32(work + 0x870, mirror);
	put32(work + 0x874, finish);
	put32(work + 0x878, j->k + 1);
	put32(work + 0x888, j->seq);
	put32(work + 0x88c, j->k);
	put32(work + 0x8a8, j->times + 8);
	put32(work + 0x8ac, j->times);
	memset(useq, 0, T7000_3D_USEQ_SIZE);
	out = useq_put(useq, start, ARRAY_SIZE(start));
	out = useq_put(out, wait, ARRAY_SIZE(wait));
	useq_put(out, finish_words, ARRAY_SIZE(finish_words));
}

/*
 * A geometry job: one record at a time on the TA channel (one pointer-ring
 * entry, its record; no PM command on the A8), on the VM's PM set, built
 * for this start of the firmware or again after a scene whose fragment
 * job never came, and S[1] for its scene as Apple's first frame. Its
 * partial-render index names the 3D ring entry of the frame's fragment
 * record.
 */
/*
 * The firmware's usage report at list +0x40 after a scene (high-water
 * pages, margin, -, request in blocks; +0x60 is 1 once taken, and the
 * firmware clears both when it starts a scene, 10008150), read as the A9
 * does (s8003_pm_feedback(): Apple's growPoolFromFirmwareFeedback): the
 * pool should hold three times the high-water mark or the margin plus a
 * block, else grow by the request. Memory only; submit_lock held.
 */
static void t7000_pm_feedback(struct pvr_apple *apple, struct s8003_vm *vm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *list = s->shared.map + t7000_pm_list_va(vm);
	u32 high, margin, request, target, blocks = vm->t7000.blocks;

	lockdep_assert_held(&apple->submit_lock);
	if (get32(list + T7000_PM_FEEDBACK_TAKEN) == 1)
		return;
	rmb();	/* the flag before the words it covers */
	high = get32(list + T7000_PM_FEEDBACK);
	margin = get32(list + T7000_PM_FEEDBACK + 4);
	request = min(get32(list + T7000_PM_FEEDBACK + 12), (u32)S8003_PM_GROW_MAX);
	if (!READ_ONCE(a8_pm_grow)) {
		/* read only, at most once a second a VM: what the A8 reports */
		if (!vm->fb_at || ktime_ms_delta(ktime_get(), vm->fb_at) >= 1000) {
			vm->fb_at = ktime_get();
			dev_info(apple_dev(apple),
				 "A8 PM usage (pool %u blocks): %u %u %u %u\n", blocks, high,
				 margin, get32(list + T7000_PM_FEEDBACK + 8),
				 get32(list + T7000_PM_FEEDBACK + 12));
		}
		return;
	}
	/* as the A9: at most every S8003_PM_FEEDBACK_MS */
	if (vm->fb_at && ktime_ms_delta(ktime_get(), vm->fb_at) < S8003_PM_FEEDBACK_MS)
		return;
	vm->fb_at = ktime_get();
	put32(list + T7000_PM_FEEDBACK_TAKEN, 1);
	wmb();
	/* Not from this pool: ignored */
	if (high > blocks * T7000_PM_SLICE_PAGES ||
	    margin > T7000_PM_BLOCKS_MAX * T7000_PM_SLICE_PAGES)
		return;
	target = DIV_ROUND_UP(max(3 * high, T7000_PM_SLICE_PAGES + margin), T7000_PM_SLICE_PAGES);
	if (target <= blocks && request)
		target = blocks + request;
	target = min_t(u32, target, T7000_PM_BLOCKS_MAX);
	if (target > vm->t7000.want) {
		dev_dbg(apple_dev(apple),
			"A8 PM pool feedback: high-water %u pages, margin %u, request %u: %u -> %u blocks\n",
			high, margin, request, blocks, target);
		vm->t7000.want = target;
	}
}

/*
 * A job's submit phases for t7000_phases(), in system counter ticks: from
 * the chip's submit (@t0) to the PM step's end (@t1), to the record slot
 * wait's end (@t2), to now, after the work ring's slot wait. Statistics
 * only; submit_lock held, right before the job's publish.
 */
static void t7000_submit_times(struct pvr_apple_s8003 *s, enum pvr_apple_engine engine,
			       u64 t0, u64 t1, u64 t2)
{
	s->t7000_submit_at[engine] = t0;
	s->t7000_pm_ticks[engine] = t1 - t0;
	s->t7000_slot_ticks[engine] = t2 - t1;
	s->t7000_ring_ticks[engine] = arch_timer_read_counter() - t2;
}

/*
 * A fragment job is submitted once its geometry job is (pvr_apple_job_armed()),
 * not once it finished: the firmware holds the 3D itself. Each 3D record
 * goes behind the barrier command {4, the TA's finish stamp, the value it
 * reaches, the job's number} (t7000_submit_frag()), Apple's form, so
 * the 3D starts when its TA is done without the host's interrupt, check,
 * signal and the fragment job's run_job in between. That barrier is what
 * Apple's A8 kext puts in front of each 3D record (seen in an iOS capture), so iOS
 * needs no host wait there; the A9's kext kicks a command buffer's TA and 3D
 * back to back (submitTAWorkAndKickMTS 0xfffffff006ca00a0 and its 3D twin
 * 0xfffffff006ca01f8), as the A9 here does (pvr_apple_chip_s8003.scenes).
 * A partial render then finds the 3D record its TA names (pr_index) in the
 * ring, as on iOS. 0 waits for the geometry job's fence as before.
 */
static bool a8_joint;
module_param(a8_joint, bool, 0644);
MODULE_PARM_DESC(a8_joint, "A8: submit a fragment job right after its geometry job, the firmware holds the 3D until the TA is done; change it only with the GPU idle (default 0)");

static bool t7000_joint(struct pvr_apple *apple)
{
	return READ_ONCE(a8_joint);
}

/*
 * The 3D record of fragment job @seq: with a8_joint the two alternate. A
 * fragment job is then written as soon as its geometry job is submitted,
 * some tens of microseconds after the job before it was seen done, while
 * the firmware may still retire that job: its interrupt came 80 us after
 * its stamp was visible on average, when measured, and the firmware reads the
 * record for the retirement (the mirror stamp's address and value at
 * W+0x870..0x878) and runs its microsequence to the end op (0x40000017).
 * Apple's kext writes each command buffer's records apart and reclaims
 * them on the firmware's completion event; the A9 here alternates two
 * (s8003_slot_of()). Without a8_joint the next fragment job came a geometry
 * job later (over 600 us), and the one record is kept as it was.
 */
static bool t7000_frag_slot1(struct pvr_apple_s8003 *s)
{
	return READ_ONCE(a8_joint) && s->t7000_frag_w != T7000_RCH1_W;
}

static int t7000_submit_geom(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	struct pvr_hwrt_dataset *dataset = job->hwrt->hwrt_dataset;
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 addr = s8003_channel_addr(s, APPLE_ENGINE_GEOM);
	u8 *p = s8003_channel(s, APPLE_ENGINE_GEOM);
	u64 t0 = arch_timer_read_counter(), t1, t2;
	struct t7000_rjob j;
	struct s8003_rt rt;
	u64 slice_addr;
	u32 seq;
	int err;

	if (!dataset->apple_rgn.ring || !vm->t7000.stack)
		return s8003_submit_error(apple, vm, "geometry", -EINVAL, S8003_REFUSED_NOT_PREPARED,
					  "not prepared");
	err = s8003_rt_derive(dataset, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_RT,
					  "rt derive");
	/*
	 * Every earlier scene is done here (a geometry job waits for the last
	 * fragment job, pvr_apple_job_add_deps()): the pool is the firmware's
	 * no more. Take the last scene's feedback, then a larger pool if one
	 * is ready; the set is built anew on it.
	 */
	if (vm->t7000.built == s->gen && !vm->t7000.open)
		t7000_pm_feedback(apple, vm);
	if (vm->t7000.next_pool && !vm->t7000.open) {
		dev_info(apple_dev(apple), "A8 PM pool: %u -> %u blocks (%u KiB)\n", vm->t7000.blocks,
			 vm->t7000.next_blocks, vm->t7000.next_blocks * 128);
		t7000_pm_obj_put(apple, &vm->t7000.pool, &vm->t7000.pool_map);
		vm->t7000.pool = vm->t7000.next_pool;
		vm->t7000.pool_map = vm->t7000.next_pool_map;
		vm->t7000.blocks = vm->t7000.next_blocks;
		vm->t7000.next_pool = NULL;
		vm->t7000.next_pool_map = NULL;
		vm->t7000.built = 0;
	}
	s->t7000_geom_vm = vm;
	if (vm->t7000.built != s->gen || vm->t7000.open) {
		err = t7000_pm_build(apple, vm);
		if (err)
			return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_PM,
						  "PM set build");
	}
	t1 = arch_timer_read_counter();
	err = s8003_wait_done(apple, APPLE_ENGINE_GEOM, apple->ta_seq);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");
	t2 = arch_timer_read_counter();
	err = t7000_publish_check(apple, APPLE_ENGINE_GEOM);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, S8003_REFUSED_PUBLISH,
					  "publish");
	t7000_submit_times(s, APPLE_ENGINE_GEOM, t0, t1, t2);

	seq = apple->ta_seq + 1;
	slice_addr = job->hwrt->data.rgn_header_dev_addr +
		     (dataset->apple_rgn.cursor & (dataset->apple_rgn.ring - 1));
	t7000_fill_ta(p + T7000_RCH_W, s->v->soc, job->cmd, job->hwrt, &rt, slice_addr);
	memset(p + T7000_RCH_TIMES, 0, 16);
	j = (struct t7000_rjob) {
		.ch = addr, .w = addr + T7000_RCH_W, .useq = addr + T7000_RCH_TA_USEQ,
		.object = s8003_object_addr(&s->layout, S8003_CHANNEL), .slot = vm->slot[0],
		.pm_id = t7000_pm_id(vm),
		.control = s8003_meta_addr(S8003_ROOT_VA + t7000_pm_control_va(vm)),
		.s = t7000_pm_s(vm), .dep = addr + S8003_CH_DEP, .seq = seq, .k = seq - 1,
		.trace = ++s->t7000_trace, .pr_ch = s8003_channel_addr(s, APPLE_ENGINE_FRAG),
		.pr_index = (2 * (apple->frag_seq + 1) - 1) % T7000_RCH_ENTRIES,
		.times = addr + T7000_RCH_TIMES,
	};
	t7000_ta_write(p + T7000_RCH_W, p + T7000_RCH_TA_USEQ, &j);
	put32(p + T7000_RCH_RING + ((seq - 1) % T7000_RCH_ENTRIES) * 4, j.w);
	/*
	 * The 3D ring entry this TA names for a partial render (pr_index) is
	 * written by its fragment job, which with a8_joint is written while
	 * this TA runs. Until then it names the last fragment job's record,
	 * complete and not rewritten (t7000_frag_slot1()), as it did before
	 * a8_joint; never a record being written.
	 */
	if (READ_ONCE(a8_joint) && s->t7000_frag_w)
		put32(s8003_channel(s, APPLE_ENGINE_FRAG) + T7000_RCH_RING + j.pr_index * 4,
		      j.pr_ch + s->t7000_frag_w);

	dataset->apple_rgn.cursor += dataset->apple_rgn.slice;
	vm->t7000.open = true;
	vm->t7000.hwrt = job->hwrt;
	vm->t7000.ta_seq = seq;
	vm->t7000.rgn_addr = slice_addr;
	apple->ta_seq = seq;
	t7000_publish(apple, APPLE_ENGINE_GEOM, seq % T7000_RCH_ENTRIES);
	return seq;
}

/*
 * A fragment job: its scene's geometry job was submitted before it. Two
 * pointer-ring entries, the barrier command {4, the TA's finish stamp, the
 * value it reaches, this job's number} and the record (P5b), the scene's
 * region-header slice, S[1] of the set.
 */
static int t7000_submit_frag(struct pvr_apple *apple, struct s8003_vm *vm, struct pvr_job *job)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 addr = s8003_channel_addr(s, APPLE_ENGINE_FRAG);
	u32 ta_addr = s8003_channel_addr(s, APPLE_ENGINE_GEOM);
	u8 *p = s8003_channel(s, APPLE_ENGINE_FRAG), *cmd;
	u64 t0 = arch_timer_read_counter(), t2;
	u32 cmd_off, w_off, useq_off, times_off;
	struct t7000_rjob j;
	struct s8003_rt rt;
	u32 seq, ring;
	int err;

	BUILD_BUG_ON(T7000_RCH1_W - T7000_RCH1_CMD < S8003_CMD_SIZE);
	BUILD_BUG_ON(T7000_RCH1_3D_USEQ - T7000_RCH1_W < T7000_3D_W_SIZE);
	BUILD_BUG_ON(T7000_RCH1_TIMES - T7000_RCH1_3D_USEQ < T7000_3D_USEQ_SIZE);
	BUILD_BUG_ON(T7000_3D_CHANNEL + T7000_RCH1_TIMES + 16 > T7000_PM_CONTROL);

	if (!vm->t7000.open || vm->t7000.hwrt != job->hwrt || vm->t7000.built != s->gen)
		return s8003_submit_error(apple, vm, "fragment", -ECANCELED, S8003_REFUSED_NO_GEOM,
					   "its geometry job failed or was not submitted");
	err = s8003_rt_derive(job->hwrt->hwrt_dataset, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_RT,
					  "rt derive");
	err = s8003_wait_done(apple, APPLE_ENGINE_FRAG, apple->frag_seq);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_SLOT_WAIT,
					  "record slot wait");
	t2 = arch_timer_read_counter();
	err = t7000_publish_check(apple, APPLE_ENGINE_FRAG);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_PUBLISH,
					  "publish");
	t7000_submit_times(s, APPLE_ENGINE_FRAG, t0, t0, t2);

	seq = apple->frag_seq + 1;
	/*
	 * Which record (t7000_frag_slot1()): with a8_joint the two alternate, so
	 * this job never rewrites the record of the job before it, which the
	 * firmware may still be retiring.
	 */
	if (t7000_frag_slot1(s)) {
		cmd_off = T7000_RCH1_CMD;
		w_off = T7000_RCH1_W;
		useq_off = T7000_RCH1_3D_USEQ;
		times_off = T7000_RCH1_TIMES;
	} else {
		cmd_off = T7000_RCH_CMD;
		w_off = T7000_RCH_W;
		useq_off = T7000_RCH_3D_USEQ;
		times_off = T7000_RCH_TIMES;
	}
	cmd = p + cmd_off;
	err = t7000_fill_3d(p + w_off, s->v->soc, job->cmd, &rt, vm->t7000.rgn_addr);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, S8003_REFUSED_FILL,
					  "3D record fill");
	memset(p + times_off, 0, 16);
	j = (struct t7000_rjob) {
		.ch = addr, .w = addr + w_off, .useq = addr + useq_off,
		.object = s8003_object_addr(&s->layout, S8003_CHANNEL + 1), .slot = vm->slot[0],
		.pm_id = t7000_pm_id(vm),
		.control = s8003_meta_addr(S8003_ROOT_VA + t7000_pm_control_va(vm)),
		.s = t7000_pm_s(vm), .dep = ta_addr + S8003_CH_DEP, .seq = seq, .k = seq - 1,
		.trace = ++s->t7000_trace, .times = addr + times_off,
	};
	t7000_3d_write(p + w_off, p + useq_off, &j);
	memset(cmd, 0, S8003_CMD_SIZE);
	put32(cmd, S8003_CMD_WAIT);
	put32(cmd + 4, ta_addr + S8003_CH_STAMP + 4);
	put32(cmd + 8, vm->t7000.ta_seq);
	put32(cmd + 12, seq);
	/* the whole record before the ring entries that name it (a partial render reads them) */
	wmb();
	ring = (2 * (seq - 1)) % T7000_RCH_ENTRIES;
	put32(p + T7000_RCH_RING + ring * 4, addr + cmd_off);
	put32(p + T7000_RCH_RING + ((ring + 1) % T7000_RCH_ENTRIES) * 4, j.w);
	s->t7000_frag_w = w_off;

	vm->t7000.open = false;
	apple->frag_seq = seq;
	t7000_publish(apple, APPLE_ENGINE_FRAG, (2 * seq) % T7000_RCH_ENTRIES);
	return seq;
}

/*
 * A fault in a job's context can leave the A8's firmware silent with
 * EVENT_STATUS bit 9 (MMU page fault) latched: no event and no watchdog. So
 * after the firmware's state, read EVENT_STATUS once, gated as every access
 * to the Rogue registers, and only with bit 9 set the BIF's two fault banks
 * once each: the faulting address, read or write, the requester's tag and
 * sideband, and the context (CAT_BASE: 0 the firmware's, a slot's, or a
 * PM's). The firmware's fault arm reads the same banks (0x10022c98). Never
 * in a loop: a tight poll of these registers during a job froze the whole
 * SoC. @why prefixes the lines.
 *
 * Return: whether EVENT_STATUS was read (the gate let it).
 */
static bool t7000_report_bif(struct pvr_apple *apple, const char *why)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct pvr_device *pvr_dev = apple->pvr_dev;
	u32 status, bank;
	bool fw_fault;

	if (s->stuck || !t7000_r0_gate(apple))
		return false;
	status = pvr_cr_read32(pvr_dev, ROGUE_CR_EVENT_STATUS);
	/*
	 * A lockup event from the firmware's MMU-fault arm (+8 = 0): the
	 * firmware took bit 9 off already (10005138), so the banks are read
	 * on its word, once, as the bit would have had them read.
	 */
	fw_fault = xchg(&s->t7000_mmu_arm, false);
	dev_err(apple_dev(apple), "%s: EVENT_STATUS %#x%s\n", why, status,
		status & ROGUE_CR_EVENT_STATUS_MMU_PAGE_FAULT_EN ? " (MMU page fault)" :
		fw_fault ? " (bit 9 taken by the firmware's MMU-fault arm)" :
		" (no MMU page fault: BIF banks not read)");
	if (!(status & ROGUE_CR_EVENT_STATUS_MMU_PAGE_FAULT_EN || fw_fault) ||
	    !t7000_r0_gate(apple))
		return true;
	for (bank = 0; bank < 2; bank++) {
		u32 reg = bank ? ROGUE_CR_BIF_FAULT_BANK1_MMU_STATUS : ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS;
		u32 req_reg = bank ? ROGUE_CR_BIF_FAULT_BANK1_REQ_STATUS :
				     ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS;
		u64 mmu = pvr_cr_read32(pvr_dev, reg) | (u64)pvr_cr_read32(pvr_dev, reg + 4) << 32;
		u64 req = pvr_cr_read32(pvr_dev, req_reg) |
			  (u64)pvr_cr_read32(pvr_dev, req_reg + 4) << 32;

		dev_err(apple_dev(apple),
			"GPU MMU fault bank %u: %s, context %llu, VA %#llx %s, tag %llu, sideband %#llx (mmu %#llx req %#llx)\n",
			bank, mmu & ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_EN ? "fault" : "no fault",
			(mmu >> ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_SHIFT) & 0xf,
			req & ~ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_ADDRESS_CLRMSK,
			req & ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_RNW_EN ? "read" : "write",
			(req >> ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_SHIFT) & 0xf,
			(req >> ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_SHIFT) & 0x3f, mmu, req);
	}
	return true;
}

/*
 * The firmware stopped on a fault (a fault event, or a job's stamp that will
 * not come): the gated registers once, at the event, as iOS reads the banks
 * before it resets the firmware. The A10's licence check (the epoch's power
 * states) has no A8 counterpart; Fender state 1 and the PLL lock are the
 * A8's gate. pvr_apple_report_mmu_fault() calls this once per firmware start.
 */
static void t7000_report_fault(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	WRITE_ONCE(s->bif_read, t7000_report_bif(apple, "fault"));
}

/*
 * A type-4 event: the firmware's lockup recovery. The lockup checker
 * 1000f538 recovers once every active engine has its lockup flag; 100290a4
 * builds the event on its stack with 1000f800:
 * +0 type 4, +0xc and +0x18 from 10002ed0, +0x10 and +0x14 zero, +0x1c the
 * checker's argument, and +0x20/+0x24 from 10017c54. +0x28 is not written by
 * the builder: what the stack held (0x1001b5a8 and 0x1001afd8 were seen;
 * neither follows a call, the first is mid-instruction). +8 (written by
 * the undecoded 1000f820) is 0 when the lockup state was set to 2 by the
 * DM handler's MMU-fault arm (10005138: the firmware's status word had bit
 * 0x200, EVENT_STATUS bit 9, which it clears after reading the BIF banks
 * itself, 10022c98) and 1 from the watchdog count in 1000f538 (TA +0x830
 * flags); these are the only two writers of state 2. So arm 0 is an MMU
 * page fault the firmware took, not a stall for pages: a stall has no event.
 *
 * 10017c54 fills +0x24 one byte per group of the object at [GbP+260], each
 * byte the same four fields: bit 0 the group's +0, bit 1 its +4, bit 2 its
 * +0xc non-zero, bits 3-7 its +0x10 (groups at +0x318, +0x30, +0x600 and
 * +0x8e8 for bytes 0-3). The byte with bit 2 set moved with the locked
 * engine: 05 81 81 81 for a TA lockup (one run), 81 05 81 81 for a 3D one
 * (another), so bytes 0-2 are named TA, 3D and CL here [inferred from those
 * two]. The lockup state the firmware keeps in host memory, object 0x4b
 * (INIT +0x98: +0 state 0/1/2, +8/+0xc/+0x10 per-engine lockup counts, +0x14
 * the total, +0x18 the first locked engine; not yet seen on Linux), is
 * printed raw beside it, and each engine whose submitted and done counts
 * differ. Memory only: no register is read here.
 */
static void t7000_event_fault(struct pvr_apple *apple, const u8 *e)
{
	static const char *const groups[4] = { "TA", "3D", "CL", "group3" };
	static const char *const names[APPLE_ENGINES] = {
		[APPLE_ENGINE_GEOM] = "geometry",
		[APPLE_ENGINE_FRAG] = "fragment",
		[APPLE_ENGINE_COMPUTE] = "compute",
	};
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 arm = get_unaligned_le32(e + 8), w = get_unaligned_le32(e + 0x24);
	const u8 *lk = s8003_object(s, 0x4b);
	char locked[32] = "";
	int i, engine;

	if (get_unaligned_le32(e) != 4)
		return;
	/* the fault registers read next (t7000_report_fault()) take the banks */
	if (arm == 0)
		WRITE_ONCE(s->t7000_mmu_arm, true);
	for (i = 0; i < 4; i++) {
		u8 b = w >> (8 * i);

		if (b & BIT(2)) {
			strlcat(locked, groups[i], sizeof(locked));
			strlcat(locked, " ", sizeof(locked));
		}
	}
	dev_err(apple_dev(apple),
		"firmware lockup recovery: arm %u (%s), locked %s(+0x24 %#x: %s %u/%u/%u/%u %s %u/%u/%u/%u %s %u/%u/%u/%u %s %u/%u/%u/%u as +0/+4/+0xc set/+0x10), +0x1c %#x +0x20 %#x\n",
		arm, arm == 0 ? "the DM handler's MMU-fault arm" : arm == 1 ? "the TA/3D watchdog" : "?",
		locked[0] ? locked : "none ", w,
		groups[0], w & 1, (w >> 1) & 1, (w >> 2) & 1, (w >> 3) & 0x1f,
		groups[1], (w >> 8) & 1, (w >> 9) & 1, (w >> 10) & 1, (w >> 11) & 0x1f,
		groups[2], (w >> 16) & 1, (w >> 17) & 1, (w >> 18) & 1, (w >> 19) & 0x1f,
		groups[3], (w >> 24) & 1, (w >> 25) & 1, (w >> 26) & 1, (w >> 27) & 0x1f,
		get_unaligned_le32(e + 0x1c), get_unaligned_le32(e + 0x20));
	dev_err(apple_dev(apple),
		"firmware lockup recovery: object 0x4b %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		get32(lk), get32(lk + 4), get32(lk + 8), get32(lk + 0xc), get32(lk + 0x10),
		get32(lk + 0x14), get32(lk + 0x18), get32(lk + 0x1c), get32(lk + 0x20),
		get32(lk + 0x24));
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		u32 submitted = s8003_submitted(apple, engine);
		u32 done = t7000_engine_done(apple, engine);

		if (submitted != done)
			dev_err(apple_dev(apple), "firmware lockup recovery: %s submitted %u done %u\n",
				names[engine], submitted, done);
	}
}

/*
 * At a hang: the parameter manager's state, for a TA that ran out of pages:
 * the TA object (INIT +0x88; +0x728 1 while the TA is active, +0x72c 2 after the out-of-memory handler 10012b24, the
 * watchdog's words +0x828..+0x840), the last geometry VM's PM control
 * (+0x24 pages, +0x28 top index, +0x3c/+0x40 the saved hardware words),
 * its list's usage report (+0x40..+0x4c, taken flag +0x60) and the stack
 * at the top index; all memory. Then R0+0x348, the PM's live top index,
 * read once through the gate. A stall for pages doubles the pool of the
 * next VM (t7000_pm_floor) and of this one, if it lives on.
 */
static void t7000_report_pm(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	const u8 *ta = s8003_object(s, S8003_CHANNEL);
	struct s8003_vm *vm = s->t7000_geom_vm;
	u32 oom = get32(ta + 0x72c);

	dev_err(apple_dev(apple),
		"hang: TA object +0x728 %#x +0x72c %#x%s; watchdog +0x828.. %08x %08x %08x %08x %08x %08x %08x\n",
		get32(ta + 0x728), oom, oom == 2 ? " (out of parameter memory)" : "",
		get32(ta + 0x828), get32(ta + 0x82c), get32(ta + 0x830), get32(ta + 0x834),
		get32(ta + 0x838), get32(ta + 0x83c), get32(ta + 0x840));
	if (vm && vm->slot[0] && vm->t7000.stack_map) {
		const u8 *control = s->root.map + t7000_pm_control_va(vm);
		const u8 *list = s->shared.map + t7000_pm_list_va(vm);
		u32 top = get32(control + 0x28);

		dev_err(apple_dev(apple),
			"hang: PM pool %u blocks: control pages %u top %u saved %#x %#x; usage %u %u %u %u taken %u; stack[top] %#x\n",
			vm->t7000.blocks, get32(control + 0x24), top, get32(control + 0x3c),
			get32(control + 0x40), get32(list + T7000_PM_FEEDBACK),
			get32(list + T7000_PM_FEEDBACK + 4), get32(list + T7000_PM_FEEDBACK + 8),
			get32(list + T7000_PM_FEEDBACK + 12), get32(list + T7000_PM_FEEDBACK_TAKEN),
			top < T7000_PM_STACK_SIZE / 4 ? get32(vm->t7000.stack_map + 4 * top) : 0);
		if (oom == 2) {
			u32 blocks = min_t(u32, 2 * vm->t7000.blocks, T7000_PM_BLOCKS_MAX);

			WRITE_ONCE(s->t7000_pm_floor, max(s->t7000_pm_floor, blocks));
			if (READ_ONCE(a8_pm_grow))
				vm->t7000.want = max(vm->t7000.want, blocks);
			dev_err(apple_dev(apple), "hang: new VMs' PM pools from now on at least %u blocks\n",
				s->t7000_pm_floor);
		}
	}
	if (s->stuck || !t7000_r0_gate(apple))
		return;
	dev_err(apple_dev(apple), "hang: R0+0x348 (PM top index) %#x\n",
		pvr_cr_read32(apple->pvr_dev, 0x348));
}

/*
 * A job timed out: the firmware's state from memory, then the fault
 * registers (t7000_report_bif()) unless the fault event read them already.
 */
static void t7000_report_hang(struct pvr_apple *apple)
{
	static const char *const names[] = { "compute", "geometry", "fragment" };
	struct pvr_apple_s8003 *s = apple->chip_data;
	int engine;

	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		const u8 *p = s8003_channel(s, engine);
		const u8 *ctl = s8003_object(s, S8003_WORK_CTL + 4 * s8003_engine(engine));
		u32 stamp = engine == APPLE_ENGINE_COMPUTE ? S8003_CU_STAMP : S8003_CH_STAMP;

		dev_err(apple_dev(apple), "hang: %s: submitted %u, stamps %u %u, work ring %u/%u\n",
			names[engine], s8003_submitted(apple, engine), get32(p + stamp),
			get32(p + stamp + 4), get32(ctl + S8003_RING_CONSUMER),
			get32(ctl + S8003_RING_PRODUCER));
	}
	dev_err(apple_dev(apple), "hang: firmware %s, Fender state %#x\n",
		s->failed ? "rings unreadable" : READ_ONCE(apple->fault) ? "posted a fault" :
		"posted no fault",
		s->stuck || (s->v->soc->ps_domain && !t7000_ps_on(s)) ? 0 :
		readl(apple->fender + FENDER_POWER_STATE));
	t7000_report_pm(apple);
	/* once: the fault event's read, if the gate let it, else here */
	if (READ_ONCE(apple->fault_reported) && xchg(&s->bif_read, false)) {
		dev_err(apple_dev(apple), "hang: fault registers were read at the fault (above)\n");
		return;
	}
	t7000_report_bif(apple, "hang");
}

/*
 * A retired job's timeline from its record, which a later job of the
 * engine overwrites only once this one is done (s8003_wait_done()); the
 * record's sequence word tells. Its microsequence times the engine's work:
 * op 0x80000018 {result, start pointer, start pointer, seq} after the start
 * packet, op 0x18 {result, start pointer, end pointer, seq} after the wait,
 * with Apple's operands (TA W+0x558, W+0x560, W+0x564; 3D W+0x8a0, W+0x8a8,
 * W+0x8ac; CL W+0x280, W+0x288, W+0x28c). The result word is the work's
 * duration in the firmware's 24 MHz time (the firmware converts the Rogue
 * timer through its correlation and compares such times against 0x5dbf,
 * 1 ms; PLATFORM +0x280 = 24000; the A9's same op measured 24 ticks/us)
 * [I]: measured raw words were 0x25e..0x2e5 (TA) and 0x35a..0x424 (3D), 25..44
 * us. The pointer words say where the firmware stores the absolute start
 * and end (t7000_ta_write(), t7000_3d_write(): T7000_RCH_TIMES +8 and +0,
 * zeroed per job); measurements showed the 3D's end there in the system counter's
 * clock. Both render engines: start_to_end from the result, kick_to_start
 * and end_to_signal from the end, the phases of the job (t7000_phases()),
 * and the first four whose end is missing or off by more than a second are
 * logged raw. Memory only, under fence_lock.
 */
#define T7000_FW_TICKS_PER_US		24

/* The phases of a job whose timeline is usable (struct t7000_phases), in counter ticks */
static void t7000_phases(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 kick,
			 u32 start, u32 end, u32 now, u32 stamped_start, ktime_t now_kt, u32 freq)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct t7000_phases *ph = &s->t7000_ph[engine];
	u32 submit = (u32)s->t7000_submit_at[engine], host = kick - submit;
	u32 tail_us = (now - end) / freq;
	ktime_t irq_at = READ_ONCE(apple->irq_at);
	s64 irq_us;

	if (!stamped_start)
		ph->start_stamp_none++;
	else if (abs((s32)(stamped_start - start)) > 2 * (s32)freq)
		ph->start_stamp_off++;
	if (!s->t7000_submit_at[engine] || host / freq > USEC_PER_SEC)
		return;
	pvr_apple_stat_add(&ph->total, (now - submit) / freq);
	pvr_apple_stat_add(&ph->host, host / freq);
	pvr_apple_stat_add(&ph->wait, (start - kick) / freq);
	pvr_apple_stat_add(&ph->run, (end - start) / freq);
	pvr_apple_stat_add(&ph->tail, tail_us);
	if (engine == APPLE_ENGINE_GEOM)
		pvr_apple_stat_add(&ph->pm, s->t7000_pm_ticks[engine] / freq);
	pvr_apple_stat_add(&ph->slot, s->t7000_slot_ticks[engine] / freq);
	pvr_apple_stat_add(&ph->ring, s->t7000_ring_ticks[engine] / freq);
	/* a fragment job the firmware held for its TA (a8_joint): that wait, then the hand-off */
	if (engine == APPLE_ENGINE_FRAG && s->t7000_geom_end &&
	    (s32)(s->t7000_geom_end - kick) > 0 && (s32)(start - s->t7000_geom_end) >= 0) {
		pvr_apple_stat_add(&ph->ta_wait, (s->t7000_geom_end - kick) / freq);
		pvr_apple_stat_add(&ph->ta_to_3d, (start - s->t7000_geom_end) / freq);
	} else {
		pvr_apple_stat_add(s->t7000_kick_woke[engine] ? &ph->wait_woke : &ph->wait_awake,
				   (start - kick) / freq);
	}
	/* the check that retires this job: one an interrupt after the end led to, or not */
	irq_us = irq_at && !ktime_after(irq_at, now_kt) ? ktime_us_delta(now_kt, irq_at) : -1;
	if (irq_us >= 0 && irq_us <= tail_us) {
		pvr_apple_stat_add(&ph->end_to_irq, tail_us - irq_us);
		pvr_apple_stat_add(&ph->irq_to_signal, irq_us);
	} else {
		pvr_apple_stat_add(&ph->end_to_check, tail_us);
	}
}

static void t7000_job_retired(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq)
{
	static const struct { u32 w, seq, result; } at[APPLE_ENGINES] = {
		[APPLE_ENGINE_GEOM] = { T7000_RCH_W, 0x54c, 0x558 },
		[APPLE_ENGINE_FRAG] = { T7000_RCH_W, 0x888, 0x8a0 },
		[APPLE_ENGINE_COMPUTE] = { S8003_CH_W, 0x278, 0x280 },
	};
	struct pvr_apple_s8003 *s = apple->chip_data;
	const u8 *p = s8003_channel(s, engine), *w = p + at[engine].w;
	u32 times = T7000_RCH_TIMES;
	ktime_t now_kt = ktime_get();
	u32 freq = arch_timer_get_cntfrq() / 1000000, now = (u32)arch_timer_read_counter();
	u32 kick = (u32)s->t7000_kick[engine], end, start;
	u64 run = get_unaligned_le64(w + at[engine].result);
	s32 wait, tail;

	s->t7000_jobs_retired++;
	s->t7000_ev_lead--;
	if (engine == APPLE_ENGINE_GEOM)
		s->t7000_geom_end = 0;
	/* the record this fragment job was written to: no later one is written before it retires */
	if (engine == APPLE_ENGINE_FRAG && s->t7000_frag_w == T7000_RCH1_W) {
		w = p + T7000_RCH1_W;
		times = T7000_RCH1_TIMES;
	}
	if (get32(w + at[engine].seq) != seq)
		return;
	if (run / T7000_FW_TICKS_PER_US <= USEC_PER_SEC)
		pvr_apple_stat_add(&s->t7000_fw_run_us[engine], run / T7000_FW_TICKS_PER_US);
	if (engine == APPLE_ENGINE_COMPUTE || !freq)
		return;
	end = get32(p + times);
	start = end - (u32)run * (freq / T7000_FW_TICKS_PER_US ?: 1);
	wait = start - kick;
	tail = now - end;
	if (!end || wait < 0 || tail < 0 || wait / freq > USEC_PER_SEC ||
	    tail / freq > USEC_PER_SEC) {
		/* the first few raw, to tell what the firmware stored and in which clock */
		if (s->t7000_fw_bad[engine]++ < 4)
			dev_info(apple_dev(apple),
				 "A8 timeline raw: engine %d seq %u kick %#x now %#x (counter %u MHz); result %#llx; times %08x %08x %08x %08x\n",
				 engine, seq, kick, now, freq, run, get32(p + times),
				 get32(p + times + 4), get32(p + times + 8),
				 get32(p + times + 12));
		return;
	}
	pvr_apple_stat_add(&s->t7000_fw_wait_us[engine], wait / freq);
	pvr_apple_stat_add(&s->t7000_fw_tail_us[engine], tail / freq);
	if (engine == APPLE_ENGINE_GEOM)
		s->t7000_geom_end = end;
	t7000_phases(apple, engine, kick, start, end, now, get32(p + times + 8),
		     now_kt, freq);
}

static void t7000_debugfs_reset(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int i;

	if (!s)
		return;
	memset(s->t7000_fw_wait_us, 0, sizeof(s->t7000_fw_wait_us));
	memset(s->t7000_fw_run_us, 0, sizeof(s->t7000_fw_run_us));
	memset(s->t7000_fw_tail_us, 0, sizeof(s->t7000_fw_tail_us));
	memset(s->t7000_fw_bad, 0, sizeof(s->t7000_fw_bad));
	memset(s->t7000_ph, 0, sizeof(s->t7000_ph));
	s->t7000_pm_builds = 0;
	s->t7000_irq_found = 0;
	s->t7000_irq_polled = 0;
	s->t7000_irq_stamp_event = 0;
	s->t7000_irq_rings = 0;
	s->t7000_irq_none = 0;
	s->t7000_stamp_events = 0;
	s->t7000_jobs_retired = 0;
	atomic64_set(&s->doorbells, 0);
	atomic64_set(&s->doorbell_wakes, 0);
	atomic64_set(&s->doorbell_pll_unlocked, 0);
	atomic64_set(&s->doorbell_refused, 0);
	for (i = 0; i < S8003_REFUSALS; i++)
		atomic64_set(&s->refused[i], 0);
}

/*
 * Which A8 interrupts find their job (&pvr_apple_chip.irq_checked), from
 * memory only, before the rings are drained. A private cursor over the
 * event ring counts every stamp event the firmware posts; t7000_job_retired()
 * counts every retired job. Their running difference (never reset) tells,
 * at an interrupt whose check found nothing, whether the firmware had posted
 * more stamp events than jobs retired (stamp_event_ahead: the event came
 * before the stamp it reports; the first four are logged with the event and
 * the stamps) or not (for_retired_job: the interrupt of a job a check
 * without an interrupt had retired first, when one had since the previous
 * interrupt; else one with only log/statistics/trace entries, or with
 * nothing new). Meaningful while the firmware posts one stamp event per job:
 * stamp_events against jobs_retired shows it. Under fence_lock.
 */
static void t7000_irq_checked(struct pvr_apple *apple, unsigned int retired, bool polled)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_EVENT_CTL);
	const u8 *events = s8003_object(s, S8003_EVENTS), *stamp_event = NULL;
	u32 producer = ring_read(ctl, S8003_RING_PRODUCER), cursor = s->t7000_ev_cursor;
	unsigned int lane;
	bool rings = false;

	if (producer >= APPLE_RING_ENTRIES)
		return;
	/* a new start of the firmware: its rings begin anew; count from here */
	if (s->t7000_ev_gen != s->gen || cursor >= APPLE_RING_ENTRIES) {
		s->t7000_ev_gen = s->gen;
		s->t7000_ev_lead = 0;
		cursor = producer;
	}
	rmb();	/* the index before the entries it covers */
	for (; cursor != producer; cursor = (cursor + 1) % APPLE_RING_ENTRIES) {
		const u8 *e = events + cursor * S8003_EVENT_SIZE;

		if (get_unaligned_le32(e) == S8003_EVENT_STAMP) {
			s->t7000_stamp_events++;
			s->t7000_ev_lead++;
			stamp_event = e;
		}
	}
	s->t7000_ev_cursor = cursor;
	for (lane = 0; lane < 4 && !rings; lane++) {
		u8 *log = s8003_object(s, S8003_LOG_CTL) + lane * s->v->ring_ctl_size;

		rings = ring_read(log, S8003_RING_CONSUMER) != ring_read(log, S8003_RING_PRODUCER);
	}
	rings |= ring_read(s8003_object(s, S8003_STATS_CTL), S8003_RING_CONSUMER) !=
		 ring_read(s8003_object(s, S8003_STATS_CTL), S8003_RING_PRODUCER);
	rings |= ring_read(s8003_object(s, S8003_TRACE_CTL), S8003_RING_CONSUMER) !=
		 ring_read(s8003_object(s, S8003_TRACE_CTL), S8003_RING_PRODUCER);

	if (retired) {
		s->t7000_irq_found++;
	} else if (s->t7000_ev_lead > 0) {
		if (s->t7000_irq_stamp_event++ < 4) {
			const u8 *ta = s8003_channel(s, APPLE_ENGINE_GEOM);
			const u8 *td = s8003_channel(s, APPLE_ENGINE_FRAG);

			dev_info(apple_dev(apple),
				 "A8 interrupt before its stamp: events ahead of retirements by %lld, last new stamp event %*ph; TA submitted %u stamps %u %u; 3D submitted %u stamps %u %u\n",
				 s->t7000_ev_lead, S8003_EVENT_SIZE - 4,
				 stamp_event ? stamp_event + 4 : events, s8003_submitted(apple, APPLE_ENGINE_GEOM),
				 get32(ta + S8003_CH_STAMP), get32(ta + S8003_CH_STAMP + 4),
				 s8003_submitted(apple, APPLE_ENGINE_FRAG), get32(td + S8003_CH_STAMP),
				 get32(td + S8003_CH_STAMP + 4));
		}
	} else if (polled) {
		s->t7000_irq_polled++;
	} else if (rings) {
		s->t7000_irq_rings++;
	} else {
		s->t7000_irq_none++;
	}
}

/*
 * The backup completion check's period while jobs are pending (with the
 * interrupt taken): the firmware raises its interrupt some time after it
 * writes a job's stamp, and in one measurement the 250 us check found 71% of
 * the jobs first. A tunable, to measure the trade against CPU wake-ups.
 */
static unsigned int a8_poll_us = 250;
module_param(a8_poll_us, uint, 0644);
MODULE_PARM_DESC(a8_poll_us, "A8: backup completion check period in us while jobs are pending (20..1000, default 250)");

static unsigned int t7000_poll_us(struct pvr_apple *apple)
{
	return clamp(READ_ONCE(a8_poll_us), 20u, 1000u);
}

/*
 * An interrupt whose check finds its job not done in memory yet: measurements show
 * that is most of them (interrupts that found nothing ~5200 against ~1950
 * that found their job), and the 100 us fast period then found the job.
 * The first re-check comes sooner; the fast period follows.
 */
static unsigned int a8_recheck_us = 20;
module_param(a8_recheck_us, uint, 0644);
MODULE_PARM_DESC(a8_recheck_us, "A8: first re-check after an interrupt that found nothing, in us (5..250, default 20)");

static unsigned int t7000_recheck_us(struct pvr_apple *apple)
{
	return clamp(READ_ONCE(a8_recheck_us), 5u, 250u);
}

static u64 t7000_avg(const struct pvr_apple_stat *st)
{
	return st->n ? div64_u64(st->sum, st->n) : 0;
}

/*
 * debugfs apple_jobs: an engine's phases (struct t7000_phases), averages in
 * us over the same jobs, so the first line adds up to its total; the second
 * splits its parts, each with its own count.
 */
static void t7000_phases_show(struct seq_file *m, const char *name, const struct t7000_phases *ph)
{
	seq_printf(m, "a8 %s phases_us n %llu total %llu = submit_to_doorbell %llu + doorbell_to_start %llu + start_to_end %llu + end_to_signal %llu\n",
		   name, ph->total.n, t7000_avg(&ph->total), t7000_avg(&ph->host),
		   t7000_avg(&ph->wait), t7000_avg(&ph->run), t7000_avg(&ph->tail));
	seq_printf(m, "a8 %s phases_detail_us submit: pm %llu slot_wait %llu ring_wait %llu; doorbell_to_start: awake %llu (n %llu) after_fender_wake %llu (n %llu) behind_ta %llu + ta_end_to_3d_start %llu (n %llu); end_to_signal: irq %llu + irq_to_signal %llu (n %llu) by_check %llu (n %llu); max total %llu host %llu wait %llu run %llu tail %llu; start_stamp none %llu off %llu\n",
		   name, t7000_avg(&ph->pm), t7000_avg(&ph->slot), t7000_avg(&ph->ring),
		   t7000_avg(&ph->wait_awake), ph->wait_awake.n,
		   t7000_avg(&ph->wait_woke), ph->wait_woke.n,
		   t7000_avg(&ph->ta_wait), t7000_avg(&ph->ta_to_3d), ph->ta_wait.n,
		   t7000_avg(&ph->end_to_irq), t7000_avg(&ph->irq_to_signal), ph->end_to_irq.n,
		   t7000_avg(&ph->end_to_check), ph->end_to_check.n,
		   ph->total.max, ph->host.max, ph->wait.max, ph->run.max, ph->tail.max,
		   ph->start_stamp_none, ph->start_stamp_off);
}

/* debugfs apple_jobs: per-VM state alive and the doorbells, statistics only */
static void t7000_debugfs_show(struct pvr_apple *apple, struct seq_file *m)
{
	static const char *const names[APPLE_ENGINES] = {
		[APPLE_ENGINE_GEOM] = "geometry",
		[APPLE_ENGINE_FRAG] = "fragment",
		[APPLE_ENGINE_COMPUTE] = "compute",
	};
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int r;
	int engine;

	if (!s)
		return;
	/* the park VM is a VM context with a VM state of its own: the idle baseline */
	seq_printf(m, "a8 live vm_states %d pm_objs %d (park VM %d)\n", atomic_read(&s->vms_live),
		   atomic_read(&s->pm_objs_live), s->park_vm ? 1 : 0);
	seq_printf(m, "a8 pm_blocks new VMs %u (a8_pm_blocks %u, stall floor %u, max %u)\n",
		   clamp(max(READ_ONCE(a8_pm_blocks), READ_ONCE(s->t7000_pm_floor)), 2u,
			 (u32)T7000_PM_BLOCKS_MAX),
		   READ_ONCE(a8_pm_blocks), READ_ONCE(s->t7000_pm_floor), (u32)T7000_PM_BLOCKS_MAX);
	seq_printf(m, "a8 doorbells %lld in_fender_state_0 %lld pll5_relocking %lld not_rung %lld\n",
		   (long long)atomic64_read(&s->doorbells),
		   (long long)atomic64_read(&s->doorbell_wakes),
		   (long long)atomic64_read(&s->doorbell_pll_unlocked),
		   (long long)atomic64_read(&s->doorbell_refused));
	/* jobs refused, by reason: all 0 when every job reached the firmware */
	seq_puts(m, "a8 refused");
	for (r = 0; r < S8003_REFUSALS; r++)
		seq_printf(m, " %s %lld", s8003_refusal_name[r],
			   (long long)atomic64_read(&s->refused[r]));
	seq_putc(m, '\n');
	seq_printf(m, "a8 irqs found_job %llu; found nothing: stamp_event_ahead %llu for_retired_job %llu other_rings_only %llu nothing_new %llu; stamp_events %llu jobs_retired %llu lead %lld\n",
		   s->t7000_irq_found, s->t7000_irq_stamp_event, s->t7000_irq_polled,
		   s->t7000_irq_rings, s->t7000_irq_none, s->t7000_stamp_events,
		   s->t7000_jobs_retired, s->t7000_ev_lead);
	seq_printf(m, "a8 pm_builds %llu poll_us %u\n", s->t7000_pm_builds, t7000_poll_us(apple));
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		const struct pvr_apple_stat *st[3] = {
			&s->t7000_fw_wait_us[engine], &s->t7000_fw_run_us[engine],
			&s->t7000_fw_tail_us[engine],
		};
		static const char *const what[3] = { "kick_to_start", "start_to_end", "end_to_signal" };
		int i;

		for (i = 0; i < 3; i++)
			seq_printf(m, "a8 %s fw %s_us last %llu max %llu avg %llu n %llu\n",
				   names[engine], what[i], st[i]->last, st[i]->max,
				   st[i]->n ? div64_u64(st[i]->sum, st[i]->n) : 0, st[i]->n);
		seq_printf(m, "a8 %s fw timestamps unusable %llu\n", names[engine],
			   s->t7000_fw_bad[engine]);
		if (engine != APPLE_ENGINE_COMPUTE)
			t7000_phases_show(m, names[engine], &s->t7000_ph[engine]);
	}
	seq_printf(m, "a8 joint %d (fragment job submitted after its geometry job's submit)\n",
		   READ_ONCE(a8_joint));
}

static void t7000_iounmap(void *addr)
{
	iounmap((void __iomem *)addr);
}

/*
 * Registers another driver owns as well (the power manager's): mapped, not
 * requested, and non-posted as every Apple device's.
 */
static void __iomem *t7000_ioremap(struct device *dev, const char *name)
{
	struct resource *res = platform_get_resource_byname(to_platform_device(dev),
							    IORESOURCE_MEM, name);
	void __iomem *addr;

	if (!res)
		return NULL;
	addr = ioremap_np(res->start, resource_size(res));
	if (!addr || devm_add_action_or_reset(dev, t7000_iounmap, (void __force *)addr))
		return NULL;
	return addr;
}

/*
 * The power manager's and the bridges' registers, and BorderColorTable's
 * contents: 0x800 bytes of the kext's data (0xfffffff0088f1460), shipped
 * beside the firmware. Without them the GPU is not started: the firmware
 * runs only as the bring-up module ran it.
 */
static int t7000_init(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	struct pvr_apple_s8003 *s = apple->chip_data;
	const struct firmware *border;
	int err;

	s->pmgr = t7000_ioremap(dev, "pmgr");
	s->bridges = t7000_ioremap(dev, "bridges");
	if (!s->pmgr || !s->bridges)
		return dev_err_probe(dev, -EINVAL, "no power manager or bridge registers\n");

	err = request_firmware(&border, s->v->soc->border_fw, dev);
	if (err)
		return dev_err_probe(dev, err, "no %s\n", s->v->soc->border_fw);
	if (border->size != T7000_BORDER_SIZE) {
		dev_err(dev, "%s has %zu bytes, not %u\n", s->v->soc->border_fw, border->size,
			T7000_BORDER_SIZE);
		err = -EINVAL;
	} else {
		s->border = devm_kmemdup(dev, border->data, T7000_BORDER_SIZE, GFP_KERNEL);
		if (!s->border)
			err = -ENOMEM;
	}
	release_firmware(border);
	return err;
}

static const struct pvr_meta_ldr_target s8003_ldr_target = {
	.mem = s8003_ldr_mem,
	.config = s8003_ldr_config,
	.start_threads = s8003_ldr_start_threads,
};

/* The A9: its values are the S8003_* constants, its steps the s8003_* functions */
static const struct s8003_variant s8003_variant_a9 = {
	.name = "A9",
	.page_size = SZ_16K,
	.image_size = S8003_IMAGE_SIZE,
	.root_size = S8003_ROOT_SIZE,
	.shared_size = S8003_SHARED_SIZE,
	.root_objects_end = S8003_ROOT_SIZE,
	.shared_objects_end = S8003_TA_CHANNEL,
	.objects = s8003_objects_a9,
	.nr_objects = ARRAY_SIZE(s8003_objects_a9),
	.ring_ctl_size = S8003_RING_CTL_SIZE,
	.slots = S8003_SLOTS,
	.slot_first = S8003_SLOT_FIRST,
	.vm_slots = 2,
	.channel = { S8003_TA_CHANNEL, S8003_3D_CHANNEL, S8003_CU_CHANNEL },
	.touch = true,
	.ldr = &s8003_ldr_target,
	.image_configure = s8003_image_configure,
	.map = s8003_map,
	.unmap = s8003_unmap,
	.init_build = s8003_init_build,
	.channels_init = s8003_channels_init,
	.boot = s8003_boot,
	.release = s8003_release,
	.core_off = s8003_core_off,
	.idle = s8003_idle,
	.doorbell = s8003_doorbell,
	.first_command = s8003_first_command,
	.vm_map = s8003_vm_map,
	.vm_put = s8003_vm_put,
	.engine_done = s8003_engine_done,
	.prepare_job = s8003_prepare_job,
	.submit_geom = s8003_submit_geom,
	.submit_frag = s8003_submit_frag,
	.submit_compute = s8003_submit_compute,
	.report_hang = s8003_report_hang,
};

/* The A8: its own sizes and boot; its jobs as the A9's where the firmware takes them the same way */
static const struct s8003_variant s8003_variant_a8 = {
	.name = "A8",
	.soc = &t7000_soc_a8,
	.page_size = SZ_4K,
	.image_size = T7000_IMAGE_SIZE,
	.root_size = T7000_ROOT_SIZE,
	.shared_size = T7000_SHARED_SIZE,
	.root_objects_end = T7000_ROOT_OBJECTS_END,
	.shared_objects_end = T7000_SHARED_OBJECTS_END,
	.objects = s8003_objects_a8,
	.nr_objects = ARRAY_SIZE(s8003_objects_a8),
	.ring_ctl_size = T7000_RING_CTL_SIZE,
	.slots = T7000_SLOTS,
	.slot_first = T7000_SLOT_FIRST,
	.vm_slots = 1,
	.channel = { T7000_TA_CHANNEL, T7000_3D_CHANNEL, T7000_CU_CHANNEL },
	.channels_root = true,
	.ldr_conf_count = S8003_LDR_CONF_MAX,
	.ldr = &t7000_ldr_target,
	.init = t7000_init,
	.image_configure = t7000_image_configure,
	.map = t7000_map,
	.unmap = t7000_unmap,
	.init_build = t7000_init_build,
	.channels_init = t7000_channels_init,
	.boot = t7000_boot,
	.release = t7000_release,
	.core_off = t7000_core_off,
	.idle = t7000_idle,
	.doorbell = t7000_doorbell,
	.no_alive = t7000_no_alive,
	.vm_map = t7000_vm_map,
	.vm_put = t7000_vm_put,
	.vm_unmap = t7000_vm_unmap,
	.engine_done = t7000_engine_done,
	.prepare_job = t7000_prepare_job,
	.submit_geom = t7000_submit_geom,
	.submit_frag = t7000_submit_frag,
	.submit_compute = t7000_submit_compute,
	.report_hang = t7000_report_hang,
	.event_fault = t7000_event_fault,
};

/* The A7: the A8's variant with the A7's SoC differences (struct t7000_soc) */
static const struct s8003_variant s8003_variant_a7 = {
	.name = "A7",
	.soc = &t7000_soc_a7,
	.page_size = SZ_4K,
	.image_size = T7000_IMAGE_SIZE,
	.root_size = T7000_ROOT_SIZE,
	.shared_size = T7000_SHARED_SIZE,
	.root_objects_end = T7000_ROOT_OBJECTS_END,
	.shared_objects_end = T7000_SHARED_OBJECTS_END,
	.objects = s8003_objects_a8,
	.nr_objects = ARRAY_SIZE(s8003_objects_a8),
	.ring_ctl_size = T7000_RING_CTL_SIZE,
	.slots = T7000_SLOTS,
	.slot_first = T7000_SLOT_FIRST,
	.vm_slots = 1,
	.channel = { T7000_TA_CHANNEL, T7000_3D_CHANNEL, T7000_CU_CHANNEL },
	.channels_root = true,
	.ldr_conf_count = S8003_LDR_CONF_MAX,
	.ldr = &t7000_ldr_target,
	.init = t7000_init,
	.image_configure = t7000_image_configure,
	.map = t7000_map,
	.unmap = t7000_unmap,
	.init_build = t7000_init_build,
	.channels_init = t7000_channels_init,
	.boot = t7000_boot,
	.release = t7000_release,
	.core_off = t7000_core_off,
	.idle = t7000_idle,
	.doorbell = t7000_doorbell,
	.first_command = s8003_first_command,	/* the A7 took it on every boot (J71) */
	.no_alive = t7000_no_alive,
	.vm_map = t7000_vm_map,
	.vm_put = t7000_vm_put,
	.vm_unmap = t7000_vm_unmap,
	.engine_done = t7000_engine_done,
	.prepare_job = t7000_prepare_job,
	.submit_geom = t7000_submit_geom,
	.submit_frag = t7000_submit_frag,
	.submit_compute = t7000_submit_compute,
	.report_hang = t7000_report_hang,
	.event_fault = t7000_event_fault,
};

static const struct s8003_variant *s8003_variant_of(const struct pvr_apple_chip *chip)
{
	if (chip == &pvr_apple_chip_t7000)
		return &s8003_variant_a8;
	if (chip == &pvr_apple_chip_s5l8960x)
		return &s8003_variant_a7;
	return &s8003_variant_a9;
}

const struct pvr_apple_chip pvr_apple_chip_s8003 = {
	.fw_name = "apple/gfx-s8003.bin",
	.perf_state_max = S8003_PERF_STATE_MAX,
	.geom_stream = &pvr_apple_g5p_cmd_geom_stream,
	.frag_stream = &pvr_apple_g5p_cmd_frag_stream,
	.geom_ext_type = PVR_STREAM_EXTHDR_TYPE_GEOM_G5P,
	.geom_ext_mask = PVR_STREAM_EXTHDR_GEOM_G5P_REGS,
	.frag_ext_type = PVR_STREAM_EXTHDR_TYPE_FRAG_G5P,
	.frag_ext_mask = PVR_STREAM_EXTHDR_FRAG_G5P_REGS,
	.compute_stream = &pvr_apple_g5p_cmd_compute_stream,
	.scenes = 2,
	.own_pm_pages = true,
	.gate_off_ms = 10,
	/* As often as s8003_touch() runs: the firmware needs those reads to go on */
	.irq_poll_us = 1000,
	.init = pvr_apple_s8003_init,
	.boot = pvr_apple_s8003_boot,
	.start = pvr_apple_s8003_start,
	.stop = pvr_apple_s8003_stop,
	.idle = pvr_apple_s8003_idle,
	.fini = pvr_apple_s8003_fini,
	.restore_image = pvr_apple_s8003_restore_image,
	.report_hang = pvr_apple_s8003_report_hang,
	.prepare_job = pvr_apple_s8003_prepare_job,
	.submit = pvr_apple_s8003_submit,
	.engine_done = pvr_apple_s8003_engine_done,
	.max_power_state = pvr_apple_s8003_max_power_state,
	.events = pvr_apple_s8003_events,
	.vm_init = pvr_apple_s8003_vm_init,
	.vm_fini = pvr_apple_s8003_vm_fini,
	.vm_scenes = pvr_apple_s8003_vm_scenes,
	.vm_evictable = pvr_apple_s8003_vm_evictable,
	.mmu_flush = pvr_apple_s8003_mmu_flush,
	.irq_ack = pvr_apple_s8003_irq_ack,
};

/*
 * The A8's heaps, as its firmware lays out a VM (heap setup 0x847efb8; the
 * bases PLATFORM tells the firmware): the general heap at 0x1_00000000, the
 * PDS heap at the PDS and USC execution base 0x3_00000000, USC code behind
 * it at 0x3_80000000, then region headers, visibility test and transfer
 * fragments. The general heap ends at 0x1_f0000000: the 256 MiB behind it
 * hold the driver's PM tables in every VM (T7000_PM_*_VA). The hardware
 * objects' 4 MiB at 0x8_00000000, reserved in every GART by Apple's kext,
 * are outside every heap, so userspace cannot map there.
 */
static const struct drm_pvr_heap t7000_heaps[] = {
	[DRM_PVR_HEAP_GENERAL] = {
		.base = 0x100000000ull,
		.size = 0xf0000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_PDS_CODE_DATA] = {
		.base = 0x300000000ull,
		.size = 0x10000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_USC_CODE] = {
		.base = 0x380000000ull,
		.size = 0x10000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_RGNHDR] = {
		.base = 0x500000000ull,
		.size = 0x10000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_VIS_TEST] = {
		.base = 0x580000000ull,
		.size = 0x10000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_TRANSFER_FRAG] = {
		.base = 0x600000000ull,
		.size = 0x10000000ull,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
};

/*
 * The A8 (T7000). Its firmware is booted and runs as the A9's is, with the
 * variant above. It starts in power state 1, Apple's start state and the
 * only one whose PLL lock is proven on Linux; the firmware is told all three
 * of its states, and the host pins it there (PLATFORM, CONTROL). Jobs take
 * the A9's command streams: the A8's records hold the same registers.
 */
const struct pvr_apple_chip pvr_apple_chip_t7000 = {
	.fw_name = "apple/gfx-t7000.bin",
	.perf_state_max = 1,
	.geom_stream = &pvr_apple_g5p_cmd_geom_stream,
	.frag_stream = &pvr_apple_g5p_cmd_frag_stream,
	.geom_ext_type = PVR_STREAM_EXTHDR_TYPE_GEOM_G5P,
	.geom_ext_mask = PVR_STREAM_EXTHDR_GEOM_G5P_REGS,
	.frag_ext_type = PVR_STREAM_EXTHDR_TYPE_FRAG_G5P,
	.frag_ext_mask = PVR_STREAM_EXTHDR_FRAG_G5P_REGS,
	.compute_stream = &pvr_apple_g5p_cmd_compute_stream,
	.own_pm_pages = true,
	.gate_off_ms = 10,
	.wait_deadline = true,
	.fault_skips_done = true,
	.irq_thread = true,
	/*
	 * Between the firmware's interrupts the completion check runs every
	 * 250 us while jobs are pending: the A9's 1 ms was a sixth of an A8
	 * compute job's wall clock. Memory only: the stamps and the rings.
	 */
	.irq_poll_us = 250,
	.heaps = t7000_heaps,
	.read_gpu_id = pvr_apple_t7000_read_gpu_id,
	.report_fault = t7000_report_fault,
	.debugfs_show = t7000_debugfs_show,
	.debugfs_reset = t7000_debugfs_reset,
	.job_retired = t7000_job_retired,
	.poll_us = t7000_poll_us,
	.recheck_us = t7000_recheck_us,
	.irq_checked = t7000_irq_checked,
	.joint = t7000_joint,
	.init = pvr_apple_s8003_init,
	.boot = pvr_apple_s8003_boot,
	.start = pvr_apple_s8003_start,
	.stop = pvr_apple_s8003_stop,
	.idle = pvr_apple_s8003_idle,
	.fini = pvr_apple_s8003_fini,
	.restore_image = pvr_apple_s8003_restore_image,
	.report_hang = pvr_apple_s8003_report_hang,
	.prepare_job = pvr_apple_s8003_prepare_job,
	.submit = pvr_apple_s8003_submit,
	.engine_done = pvr_apple_s8003_engine_done,
	.events = pvr_apple_s8003_events,
	.vm_init = pvr_apple_s8003_vm_init,
	.vm_fini = pvr_apple_s8003_vm_fini,
	.mmu_flush = pvr_apple_s8003_mmu_flush,
	.irq_ack = pvr_apple_s8003_irq_ack,
};

/*
 * The A7 (S5L8960X): the A8's chip with the A7's firmware. Its firmware
 * runs the A8's host interface; jobs are wired as the A8's but untested on
 * the A7 (the A7's shader encodings are not the A8's).
 */
const struct pvr_apple_chip pvr_apple_chip_s5l8960x = {
	.fw_name = "apple/gfx-s5l8960x.bin",
	.perf_state_max = 1,
	.geom_stream = &pvr_apple_g5p_cmd_geom_stream,
	.frag_stream = &pvr_apple_g5p_cmd_frag_stream,
	.geom_ext_type = PVR_STREAM_EXTHDR_TYPE_GEOM_G5P,
	.geom_ext_mask = PVR_STREAM_EXTHDR_GEOM_G5P_REGS,
	.frag_ext_type = PVR_STREAM_EXTHDR_TYPE_FRAG_G5P,
	.frag_ext_mask = PVR_STREAM_EXTHDR_FRAG_G5P_REGS,
	.compute_stream = &pvr_apple_g5p_cmd_compute_stream,
	.own_pm_pages = true,
	.gate_off_ms = 10,
	.wait_deadline = true,
	.fault_skips_done = true,
	.irq_thread = true,
	/*
	 * Between the firmware's interrupts the completion check runs every
	 * 250 us while jobs are pending: the A9's 1 ms was a sixth of an A8
	 * compute job's wall clock. Memory only: the stamps and the rings.
	 */
	.irq_poll_us = 250,
	.heaps = t7000_heaps,
	.read_gpu_id = pvr_apple_t7000_read_gpu_id,
	.report_fault = t7000_report_fault,
	.debugfs_show = t7000_debugfs_show,
	.debugfs_reset = t7000_debugfs_reset,
	.job_retired = t7000_job_retired,
	.poll_us = t7000_poll_us,
	.recheck_us = t7000_recheck_us,
	.irq_checked = t7000_irq_checked,
	.joint = t7000_joint,
	.init = pvr_apple_s8003_init,
	.boot = pvr_apple_s8003_boot,
	.start = pvr_apple_s8003_start,
	.stop = pvr_apple_s8003_stop,
	.idle = pvr_apple_s8003_idle,
	.fini = pvr_apple_s8003_fini,
	.restore_image = pvr_apple_s8003_restore_image,
	.report_hang = pvr_apple_s8003_report_hang,
	.prepare_job = pvr_apple_s8003_prepare_job,
	.submit = pvr_apple_s8003_submit,
	.engine_done = pvr_apple_s8003_engine_done,
	.events = pvr_apple_s8003_events,
	.vm_init = pvr_apple_s8003_vm_init,
	.vm_fini = pvr_apple_s8003_vm_fini,
	.mmu_flush = pvr_apple_s8003_mmu_flush,
	.irq_ack = pvr_apple_s8003_irq_ack,
};
