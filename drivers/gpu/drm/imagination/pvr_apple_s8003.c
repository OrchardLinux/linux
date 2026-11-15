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
#include <dt-bindings/interrupt-controller/apple-aic.h>
#include <linux/bitmap.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
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
	const struct pvr_hwrt_data *hwrt;	/* the scene's target */
	u32 scene;			/* the firmware start the scene belongs to; 0: none */
	u32 ta_seq;			/* its geometry job */
	u32 td_seq;			/* its fragment job; 0: not submitted */
	bool clean;			/* both of its jobs were handed to the firmware */
};

struct s8003_vm {
	struct pvr_vm_context *vm_ctx;
	u32 slot[2];			/* geometry and compute, fragment; 0: none */
	u32 attached;			/* the firmware start the slots were loaded for */
	struct s8003_pm pm[2];
	bool pm_logged;			/* first pool sizing report for this VM */
	u32 fb_blocks;			/* pool size the firmware's feedback asked for */
	ktime_t fb_at;			/* when that feedback was last taken */
};

struct pvr_apple_s8003 {
	struct pvr_apple *apple;
	struct delayed_work drain_work;	/* reads the rings while the firmware runs */
	struct mutex event_lock;	/* the rings the firmware posts to */
	ktime_t touched;		/* s8003_touch() */
	bool failed;			/* its rings' indices are not ours any more */
	bool fault;			/* it posted an event that is neither "alive" nor a stamp */
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

	/* For debugfs: the reads s8003_touch() made, and the waits for the firmware */
	atomic64_t touches, touch_ns, touch_lines;
	atomic64_t done_polls, ring_polls;
	/* Under submit_lock: PM record or none per geometry job, pool feedback */
	u64 pm_full, pm_clean, pm_clean_queued, pm_rec_unreached;
	u64 fb_reads, fb_grows;
	u32 fb_max_blocks;
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

	struct s8003_idle idle;
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

/* Apple's kext packs each group in this order, every object on 64 bytes. */
static int s8003_layout_plan(struct s8003_layout *layout)
{
	static const struct {
		u8 object;
		bool shared;
		u32 size;
	} objects[] = {
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
	u32 used[2] = {};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(objects); i++) {
		u32 offset = ALIGN(used[objects[i].shared], 64);

		layout->offset[objects[i].object] = offset;
		layout->shared[objects[i].object] = objects[i].shared;
		used[objects[i].shared] = offset + objects[i].size;
	}
	return used[0] <= S8003_ROOT_SIZE && used[1] <= S8003_TA_CHANNEL ? 0 : -EINVAL;
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

static const struct pvr_meta_ldr_target s8003_ldr_target = {
	.mem = s8003_ldr_mem,
	.config = s8003_ldr_config,
	.start_threads = s8003_ldr_start_threads,
};

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

static void pvr_apple_s8003_fini(struct pvr_apple *apple)
{
	struct pvr_vm_context *vm_ctx = apple->pvr_dev->kernel_vm_ctx;
	struct pvr_apple_s8003 *s = apple->chip_data;

	if (s->pio_mapped) {
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + S8003_PIO_FENDER, S8003_PIO_SIZE);
		pvr_vm_unmap_io(vm_ctx, S8003_PIO_VA + S8003_PIO_AIC, S8003_PIO_SIZE);
		s->pio_mapped = false;
	}
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
					 lane * S8003_RING_CTL_SIZE);
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
			if (!READ_ONCE(s->fault)) {
				WRITE_ONCE(s->fault, true);
				pvr_apple_report_mmu_fault(apple);
				pvr_apple_firmware_fault(apple);
			}
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
	bool alive = false;
	int err, drained = 0;

	err = read_poll_timeout(s8003_drain, drained, drained || alive, 1000,
				S8003_ALIVE_TIMEOUT_US, false, apple, &alive);
	if (!err)
		err = drained;
	if (err)
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

static u32 s8003_channel_offset(enum pvr_apple_engine engine)
{
	static const u32 offset[] = { S8003_TA_CHANNEL, S8003_3D_CHANNEL, S8003_CU_CHANNEL };

	return offset[s8003_engine(engine)];
}

static u8 *s8003_channel(struct pvr_apple_s8003 *s, enum pvr_apple_engine engine)
{
	return s->shared.map + s8003_channel_offset(engine);
}

static u32 s8003_channel_addr(enum pvr_apple_engine engine)
{
	return s8003_meta_addr(S8003_SHARED_VA + s8003_channel_offset(engine));
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
			   s8003_channel_addr(APPLE_ENGINE_GEOM), S8003_CH_TQ, S8003_CH_DEP);
	s8003_channel_init(s8003_channel(s, APPLE_ENGINE_FRAG),
			   s8003_channel_addr(APPLE_ENGINE_FRAG), S8003_CH_TQ, S8003_CH_DEP);
	s8003_channel_init(s8003_channel(s, APPLE_ENGINE_COMPUTE),
			   s8003_channel_addr(APPLE_ENGINE_COMPUTE), S8003_CU_TQ, S8003_CU_DEP);
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
	if (ktime_us_delta(now, READ_ONCE(s->touched)) + S8003_TOUCH_US / 8 >= S8003_TOUCH_US) {
		WRITE_ONCE(s->touched, now);
		atomic64_add(s8003_touch(s), &s->touch_lines);
		atomic64_add(ktime_to_ns(ktime_sub(ktime_get(), now)), &s->touch_ns);
		atomic64_inc(&s->touches);
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
static u32 pvr_apple_s8003_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine)
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

static u32 s8003_done_poll(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	atomic64_inc(&s->done_polls);
	pvr_apple_s8003_events(apple);
	return pvr_apple_s8003_engine_done(apple, engine);
}

static int s8003_wait_done(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 done;
	int err;

	err = read_poll_timeout(s8003_done_poll, done,
				(s32)(done - seq) >= 0 || READ_ONCE(s->fault), 20,
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
	struct pvr_apple_s8003 *s = apple->chip_data;

	atomic64_inc(&s->ring_polls);
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

	err = s8003_doorbell(apple, ROGUE_CR_MTS_SCHEDULE_DM_DM0);
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

/* Point a pair of slots at a page-table root (0: at none) under @tag */
static int s8003_slots_load(struct pvr_apple *apple, const u32 *slot, u64 root, u32 tag)
{
	const u32 cmd[2][2] = { { S8003_DC_SLOT, slot[1] }, { S8003_DC_SLOT, slot[0] } };
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned long flags;
	u64 steps = 0, total;
	unsigned int e;
	int err;

	spin_lock_irqsave(&s->tag_lock, flags);
	for (e = 0; e < 2; e++) {
		__set_bit(slot[e], s->loading);
		put64(s8003_object(s, S8003_MMU_ROOT) + 8 * slot[e], root);
		put32(s8003_object(s, S8003_TAG_HOST) + 4 * slot[e], tag);
		s->tag[slot[e]] = tag;
	}
	spin_unlock_irqrestore(&s->tag_lock, flags);

	err = s8003_device_control(apple, cmd, 2);
	if (err)
		dev_err(apple_dev(apple), "GFX firmware slot load failed: %d\n", err);
	/* The firmware must copy the tag we held stable while loading the root. */
	for (e = 0; e < 2 && !err; e++) {
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
	for (e = 0; e < 2; e++) {
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
 * The slot pair of a VM for this start of the firmware. Slots are handed out
 * in turn, so that one a VM gave back is used again as late as possible; if
 * it was left on the park tables (pvr_apple_s8003_vm_fini()) it is emptied
 * first, as Apple's kext does when an address space goes.
 */
static int s8003_vm_attach(struct pvr_apple *apple, struct s8003_vm *vm)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	unsigned int e;
	int err = 0;

	lockdep_assert_held(&apple->submit_lock);
	if (vm->attached == s->gen)
		return 0;
	for (e = 2; e-- > 0 && !vm->slot[e];) {
		u32 slot = find_next_zero_bit(s->slots, S8003_SLOTS, s->next_slot);

		if (slot >= S8003_SLOTS)
			slot = find_next_zero_bit(s->slots, S8003_SLOTS, S8003_SLOT_FIRST);
		if (slot >= S8003_SLOTS) {
			if (vm->slot[1])
				__clear_bit(vm->slot[1], s->slots);
			vm->slot[1] = 0;
			return -EBUSY;
		}
		__set_bit(slot, s->slots);
		s->next_slot = slot + 1 < S8003_SLOTS ? slot + 1 : S8003_SLOT_FIRST;
		vm->slot[e] = slot;
	}
	/* No job runs while a slot changes */
	err = pvr_apple_wait_idle(apple, S8003_JOB_TIMEOUT_US);
	if (!err && (test_bit(vm->slot[0], s->parked) || test_bit(vm->slot[1], s->parked))) {
		err = s8003_slots_load(apple, vm->slot, 0, 0);
		__clear_bit(vm->slot[0], s->parked);
		__clear_bit(vm->slot[1], s->parked);
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
	for (slot = S8003_SLOT_FIRST; slot < S8003_SLOTS; slot++) {
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

static int pvr_apple_s8003_vm_init(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	struct s8003_vm *vm;
	int err;

	if (!s->pmsets.obj)
		return -ENODEV;
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;
	vm->vm_ctx = vm_ctx;
	vm->pm[0].set = -1;
	vm->pm[1].set = -1;
	/* The PM reaches its sets through the job's slot, at the firmware's addresses */
	err = pvr_vm_map_reserved(vm_ctx, s->pmsets.obj, S8003_PM_SETS_VA, s->pmsets.size);
	if (!err)
		err = xa_err(xa_store(&s->vms, (unsigned long)vm_ctx, vm, GFP_KERNEL));
	if (err)
		kfree(vm);
	return err;
}

/*
 * How many scenes of a VM the firmware may hold: as many as the VM has PM
 * sets. It takes its first here, and its second while the firmware has more
 * than half of them left. Called when a job is ordered, in atomic context.
 */
static unsigned int pvr_apple_s8003_vm_scenes(struct pvr_apple *apple,
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

static void s8003_pm_put(struct pvr_apple_s8003 *s, struct s8003_pm *pm)
{
	unsigned long flags;

	spin_lock_irqsave(&s->set_lock, flags);
	if (pm->set >= 0)
		__clear_bit(pm->set, s->sets);
	spin_unlock_irqrestore(&s->set_lock, flags);
	if (pm->pool)
		pvr_gem_object_put(pm->pool);
	if (pm->aux)
		pvr_gem_object_put(pm->aux);
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
		__set_bit(vm->slot[0], s->parked);
		__set_bit(vm->slot[1], s->parked);
	}
	if (s->pm_last == vm)
		s->pm_last = NULL;
	if (s->pm_prev == vm)
		s->pm_prev = NULL;
	s8003_wb_sets_update(s);
	for (e = 0; e < 2; e++) {
		if (vm->slot[e])
			__clear_bit(vm->slot[e], s->slots);
		s8003_pm_put(s, &vm->pm[e]);
	}
	mutex_unlock(&apple->submit_lock);
	kfree(vm);
}

static int s8003_obj_zero(struct pvr_gem_object *obj)
{
	void *map = pvr_gem_object_vmap(obj);

	if (IS_ERR(map))
		return PTR_ERR(map);
	memset(map, 0, pvr_gem_object_size(obj));
	pvr_gem_object_vunmap(obj);
	return 0;
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

	if (!pm->aux) {
		struct pvr_gem_object *aux;

		aux = pvr_gem_object_create(apple->pvr_dev, S8003_PM_AUX_SIZE,
					    DRM_PVR_BO_PM_FW_PROTECT);
		if (IS_ERR(aux))
			return PTR_ERR(aux);
		err = pvr_vm_map_reserved(vm->vm_ctx, aux, s8003_pm_aux_va(pm), S8003_PM_AUX_SIZE);
		if (err) {
			pvr_gem_object_put(aux);
			return err;
		}
		pm->aux = aux;
	}
	if (pm->built == s->gen && pm->blocks >= blocks)
		return 0;
	if (pm->blocks < blocks) {
		struct pvr_gem_object *pool;

		pool = pvr_gem_object_create(apple->pvr_dev, (size_t)blocks * S8003_PM_BLOCK_SIZE,
					     PVR_BO_FW_FLAGS_DEVICE_CACHED);
		if (IS_ERR(pool))
			return PTR_ERR(pool);
		if (pm->pool)
			pvr_gem_object_put(pm->pool);
		pm->pool = pool;
		pm->blocks = blocks;
	}
	pm->built = 0;
	pm->rec_seq = 0;
	err = s8003_obj_zero(pm->pool);
	if (!err)
		err = s8003_obj_zero(pm->aux);
	if (err)
		return err;

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
	put32(entry + 4, s8003_channel_addr(engine));
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
	       pm->scene == s->gen && pm->td_seq && !READ_ONCE(s->fault) &&
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
		s->fb_reads++;
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
		s->fb_grows++;
		s->fb_max_blocks = max(s->fb_max_blocks, target);
		dev_info(apple_dev(apple),
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
	if (!vm || READ_ONCE(s->fault))
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

static int s8003_submit_error(struct pvr_apple *apple, struct s8003_vm *vm,
			      const char *engine, int err, const char *reason)
{
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
		return s8003_submit_error(apple, vm, "geometry", err, "rt derive");
	/* The VM's sets were taken when the job was ordered (pvr_apple_s8003_vm_scenes()) */
	for (i = 0; i < 2 && !pm; i++)
		if (vm->pm[i].set >= 0 && s8003_pm_idle(apple, &vm->pm[i]))
			pm = &vm->pm[i];
	/*
	 * Each holds a scene still: one whose fragment job never came (a
	 * geometry job alone) is given up once its geometry job is done.
	 */
	for (i = 0; i < 2 && !pm; i++)
		if (vm->pm[i].set >= 0 && !vm->pm[i].td_seq &&
		    !s8003_wait_done(apple, APPLE_ENGINE_GEOM, vm->pm[i].ta_seq))
			pm = &vm->pm[i];
	if (!pm)
		return s8003_submit_error(apple, vm, "geometry", -EBUSY, "no PM set");

	/* Process context, submit_lock held: grow through the existing pool allocator. */
	s8003_pm_feedback(apple, vm);
	/* As many 128 KiB blocks as the free list has 4 KiB pages, and enough for the target */
	blocks = max(DIV_ROUND_UP(hwrt->free_lists[0]->initial_num_pages, 32),
		     DIV_ROUND_UP(50 + rt.tiles, 32));
	/* The existing build path allocates a larger pool while this set is idle. */
	blocks = max(blocks, vm->fb_blocks);
	/* Before the build, which may make the set anew */
	pm_record = !s8003_pm_clean(apple, pm, min(blocks, S8003_PM_BLOCKS_MAX));
	pm->clean = false;
	err = s8003_pm_build(apple, vm, pm, min(blocks, S8003_PM_BLOCKS_MAX));
	if (!err && !vm->pm_logged) {
		dev_info(apple_dev(apple),
			 "PM pool VM=%p blocks=%u clamped=%u tiles=%u initial_num_pages=%u\n",
			 vm->vm_ctx, pm->blocks, blocks > S8003_PM_BLOCKS_MAX, rt.tiles,
			 hwrt->free_lists[0]->initial_num_pages);
		vm->pm_logged = true;
	}
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, "PM pool build failed");
	/* The record slot of the job before the last one is free again */
	if (apple->ta_seq)
		err = s8003_wait_done(apple, APPLE_ENGINE_GEOM, apple->ta_seq - 1);
	if (err)
		return s8003_submit_error(apple, vm, "geometry", err, "record slot wait");

	seq = apple->ta_seq + 1;
	if (!pm_record) {
		u32 done = pvr_apple_s8003_engine_done(apple, APPLE_ENGINE_GEOM);

		/* The last rebuild must be visible; a newer ordinary TA need not be. */
		if ((s32)(done - pm->rec_seq) < 0) {
			pm_record = true;
			s->pm_rec_unreached++;
		} else if ((s32)(done - apple->ta_seq) < 0) {
			s->pm_clean_queued++;
		}
	}
	if (pm_record)
		s->pm_full++;
	else
		s->pm_clean++;
	s8003_fill_ta(p + s8003_slot_of(seq, false).w, job->cmd, job->hwrt, &rt);
	s8003_job_write(p, s8003_channel_addr(APPLE_ENGINE_GEOM),
			s8003_object_addr(&s->layout, S8003_CHANNEL),
			s8003_meta_addr(s8003_pm_set_va(pm, 0)), pm->set + 1, vm->slot[0], false,
			seq, s8003_channel_addr(APPLE_ENGINE_GEOM) + S8003_CH_STAMP + 4,
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
		return s8003_submit_error(apple, vm, "geometry", err, "publish");
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
		return s8003_submit_error(apple, vm, "fragment", -ECANCELED,
					   "its geometry job failed or was not submitted");
	err = s8003_rt_derive(job->hwrt->hwrt_dataset, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, "rt derive");
	if (apple->frag_seq)
		err = s8003_wait_done(apple, APPLE_ENGINE_FRAG, apple->frag_seq - 1);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, "record slot wait");

	seq = apple->frag_seq + 1;
	err = s8003_fill_3d(p + s8003_slot_of(seq, true).w, job->cmd, job->hwrt, &rt);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, "3D record fill");
	s8003_job_write(p, s8003_channel_addr(APPLE_ENGINE_FRAG),
			s8003_object_addr(&s->layout, S8003_CHANNEL + 1),
			s8003_meta_addr(s8003_pm_set_va(pm, 0)), pm->set + 1, vm->slot[1], true, seq,
			s8003_channel_addr(APPLE_ENGINE_GEOM) + S8003_CH_STAMP + 4, pm->ta_seq, true);
	pm->td_seq = seq;
	WRITE_ONCE(s->set_frag[pm->set], seq);
	apple->frag_seq = seq;
	err = s8003_publish(apple, APPLE_ENGINE_FRAG, seq, (2 * seq) % S8003_CH_ENTRIES);
	if (err)
		return s8003_submit_error(apple, vm, "fragment", err, "publish");
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
	u32 addr = s8003_channel_addr(APPLE_ENGINE_COMPUTE);
	u32 global = s8003_object_addr(&s->layout, S8003_CHANNEL + 2);
	u32 w = addr + S8003_CH_W, slot = vm->slot[0], seq;
	u8 *p = s8003_channel(s, APPLE_ENGINE_COMPUTE);
	u8 *work = p + S8003_CH_W;
	unsigned int i;
	int err;

	err = s8003_wait_done(apple, APPLE_ENGINE_COMPUTE, apple->last_seq);
	if (err)
		return s8003_submit_error(apple, vm, "compute", err, "record slot wait");
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
		return s8003_submit_error(apple, vm, "compute", err, "publish");
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
	if (err)
		goto out_unlock;
	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY)
		err = s8003_submit_geom(apple, vm, job);
	else if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		err = s8003_submit_frag(apple, vm, job);
	else
		err = s8003_submit_compute(apple, vm, job);
out_unlock:
	mutex_unlock(&apple->submit_lock);
	return err;
}

static void pvr_apple_s8003_report_hang(struct pvr_apple *apple)
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
		s->failed ? "rings unreadable" : s->fault ? "posted a fault" : "posted no fault",
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
	WRITE_ONCE(s->fault, false);
	memset(s->set_geom, 0, sizeof(s->set_geom));
	memset(s->set_frag, 0, sizeof(s->set_frag));
	memset(s->pmsets.map, 0, s->pmsets.size);
	s8003_channels_init(s);
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

static int pvr_apple_s8003_init(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	struct pvr_apple_layout *layout = &apple->layout;
	struct pvr_apple_s8003 *s;
	unsigned int i;
	int err;

	/* The firmware's page tables have 16 KiB pages, as the kernel's must */
	if (PVR_DEVICE_PAGE_SIZE != SZ_16K)
		return dev_err_probe(dev, -ENODEV, "the A9 GPU needs a 16 KiB page kernel\n");

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	apple->chip_data = s;
	s->apple = apple;
	INIT_DELAYED_WORK(&s->drain_work, s8003_drain_work);
	spin_lock_init(&s->tag_lock);
	spin_lock_init(&s->set_lock);
	xa_init(&s->vms);
	s->next_slot = S8003_SLOT_FIRST;
	err = devm_mutex_init(dev, &s->event_lock);
	if (err)
		return err;

	err = s8003_parse_dt(apple);
	if (err)
		return err;
	s8003_irq_init(apple);
	err = s8003_layout_plan(&s->layout);
	if (err)
		return err;

	s->image = kvzalloc(S8003_IMAGE_SIZE, GFP_KERNEL);
	if (!s->image)
		return -ENOMEM;
	err = devm_add_action_or_reset(dev, s8003_free_image, s->image);
	if (err)
		return err;
	err = pvr_apple_fw_load(apple, SZ_1M);
	if (err)
		return err;
	err = pvr_meta_ldr_process(from_pvr_device(apple->pvr_dev), apple->fw_image,
				   apple->fw_size, &s8003_ldr_target, s, NULL);
	if (!err && !s->thread_pc)
		err = -EINVAL;
	if (err)
		return dev_err_probe(dev, err, "not the A9's GFX firmware image\n");
	s8003_image_configure(s);

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
static int pvr_apple_s8003_boot(struct pvr_apple *apple)
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

static int pvr_apple_s8003_start(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u32 q = pvr_apple_start_power_state(apple);
	int err;

	if (!s->shared.obj) {
		err = s8003_map(apple);
		if (err)
			return err;
	}
	/* The firmware writes into its image while it runs: start from the copy */
	memcpy(s->fw.map, s->image, S8003_IMAGE_SIZE);
	s8003_init_build(apple, q);
	s8003_jobs_reset(s);

	err = s8003_release(apple, q);
	if (!err)
		err = s8003_wait_alive(apple);
	if (!err) {
		dev_dbg(apple_dev(apple), "GFX firmware is up (power state %u)\n", q);
		err = s8003_first_command(apple);
	}
	if (err) {
		/* It must not run on once the kernel VM, its page tables, may go */
		s8003_core_off(apple);
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
	s8003_core_off(apple);
}

static void pvr_apple_s8003_idle(struct pvr_apple *apple)
{
	/* nothing of the firmware's keeps the GPU's power domain on */
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

static bool pvr_apple_s8003_fault(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	return READ_ONCE(s->fault);
}

static void pvr_apple_s8003_debugfs_show(struct pvr_apple *apple, struct seq_file *m)
{
	struct pvr_apple_s8003 *s = apple->chip_data;

	seq_printf(m, "touch count %lld ns %lld lines %lld\n", atomic64_read(&s->touches),
		   atomic64_read(&s->touch_ns), atomic64_read(&s->touch_lines));
	seq_printf(m, "pm_record full %llu clean %llu clean_queued %llu rec_unreached %llu\n",
		   READ_ONCE(s->pm_full), READ_ONCE(s->pm_clean),
		   READ_ONCE(s->pm_clean_queued), READ_ONCE(s->pm_rec_unreached));
	seq_printf(m, "pm_pool feedback %llu grows %llu max_blocks %u\n", READ_ONCE(s->fb_reads),
		   READ_ONCE(s->fb_grows), READ_ONCE(s->fb_max_blocks));
	seq_printf(m, "s8003_wait_polls done %lld ring %lld\n", atomic64_read(&s->done_polls),
		   atomic64_read(&s->ring_polls));
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
	.submit = pvr_apple_s8003_submit,
	.engine_done = pvr_apple_s8003_engine_done,
	.fault = pvr_apple_s8003_fault,
	.max_power_state = pvr_apple_s8003_max_power_state,
	.events = pvr_apple_s8003_events,
	.vm_init = pvr_apple_s8003_vm_init,
	.vm_fini = pvr_apple_s8003_vm_fini,
	.vm_scenes = pvr_apple_s8003_vm_scenes,
	.vm_evictable = pvr_apple_s8003_vm_evictable,
	.mmu_flush = pvr_apple_s8003_mmu_flush,
	.irq_ack = pvr_apple_s8003_irq_ack,
	.debugfs_show = pvr_apple_s8003_debugfs_show,
};
