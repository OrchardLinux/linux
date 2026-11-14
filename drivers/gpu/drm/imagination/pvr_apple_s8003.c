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
 * Not here yet: the firmware's commands. No job is written, so the chip takes
 * none (pvr_apple_takes_jobs()), and the firmware's interrupt to the host is
 * not taken: its rings are read on a timer while it runs.
 *
 * The boot sequence, the layout of the shared memory and every value in it
 * are from Berkay Kösebay's G5P driver, which runs the GPU on the iPad 5
 * (J71t); the names of the firmware's objects are his readings of Apple's
 * kext and firmware.
 */

#include "pvr_apple.h"
#include "pvr_apple_chip.h"
#include "pvr_device.h"
#include "pvr_fw_meta.h"
#include "pvr_gem.h"
#include "pvr_mmu.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_meta.h"
#include "pvr_vm.h"

#include <clocksource/arm_arch_timer.h>
#include <dt-bindings/interrupt-controller/apple-aic.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/*
 * Fender beyond the power state. The boot stream is a list of Rogue register
 * writes, two words each; Fender runs it when the core's power state leaves 0.
 */
#define FENDER_CTRL			0x0000
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

struct pvr_apple_s8003 {
	struct pvr_apple *apple;
	struct delayed_work drain_work;	/* reads the rings while the firmware runs */

	/* Bus addresses the firmware is told: its registers' and ours */
	phys_addr_t fender_phys;
	phys_addr_t rgx_phys;
	phys_addr_t aic_phys;
	u32 irq;			/* the firmware's interrupt to the host, as the AIC numbers it */

	struct s8003_layout layout;
	u8 *image;			/* the unpacked image, as the bootloader finds it */
	u32 conf[S8003_LDR_CONF_MAX][2];
	u32 conf_count;
	u32 thread_sp;
	u32 thread_pc;

	struct s8003_mem fw;		/* the image the firmware runs in */
	struct s8003_mem root;
	struct s8003_mem shared;
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
	return used[0] <= S8003_ROOT_SIZE && used[1] <= S8003_SHARED_SIZE ? 0 : -EINVAL;
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
	put64(platform + 0x10, 0x300000000ull);
	put64(platform + 0x18, 0x300000000ull);
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
	s8003_mem_fini(apple, &s->zero);
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
 * Events other than "alive" and a job's stamp mean the firmware stopped (a
 * fault, a recovery request): none is expected before a job ran.
 */
static int s8003_drain(struct pvr_apple *apple, bool *alive)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_EVENT_CTL);
	u8 *events = s8003_object(s, S8003_EVENTS);
	u32 consumer, producer;
	unsigned int lane;
	int err = 0;

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
		} else if (type != S8003_EVENT_STAMP) {
			dev_err(apple_dev(apple), "firmware event %u: %*ph\n", type,
				S8003_EVENT_SIZE - 4, e + 4);
			err = -EIO;
		}
		consumer = (consumer + 1) % APPLE_RING_ENTRIES;
		ring_consume(ctl, consumer);
	}
	if (err)
		return err;

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

	err = s8003_drain(s->apple, &alive);
	if (err) {
		dev_err(apple_dev(s->apple), "GFX firmware's rings are no longer read: %d\n", err);
		return;
	}
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
 * Make the firmware look at the device-control ring: Fender's wake first, as
 * Apple's kext does before every doorbell, then the META's scheduler.
 */
static int s8003_dc_doorbell(struct pvr_apple *apple)
{
	writel(FENDER_WAKE_ALL, apple->fender + FENDER_WAKE);
	if (readl(apple->fender + FENDER_WAKE) & ~FENDER_WAKE_ALL)
		return -EPROTO;
	if (!s8003_core_ready(apple))
		return -EHOSTDOWN;
	pvr_cr_write32(apple->pvr_dev, ROGUE_CR_MTS_SCHEDULE, S8003_MTS_DEVICE_CONTROL);
	return 0;
}

static u32 s8003_dc_consumer(struct pvr_apple *apple, int *err)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	bool alive = false;

	*err = s8003_drain(apple, &alive);
	return ring_read(s8003_object(s, S8003_DC_CTL), S8003_RING_CONSUMER);
}

/*
 * The first command: an empty device-control command. The firmware takes it
 * from the ring once its scheduler runs, which is when it takes commands at
 * all.
 */
static int s8003_first_command(struct pvr_apple *apple)
{
	struct pvr_apple_s8003 *s = apple->chip_data;
	u8 *ctl = s8003_object(s, S8003_DC_CTL);
	u8 *ring = s8003_object(s, S8003_DC_RING);
	u32 producer = ring_read(ctl, S8003_RING_PRODUCER);
	u32 consumer, target;
	int err, drained = 0;

	if (producer >= APPLE_RING_ENTRIES || ring_read(ctl, S8003_RING_CONSUMER) != producer)
		return -EPROTO;
	target = (producer + 1) % APPLE_RING_ENTRIES;

	put32(s8003_object(s, S8003_CONTROL) + 0x1020, 0);	/* no payload */
	memset(ring + producer * S8003_DC_SIZE, 0, S8003_DC_SIZE);
	put32(ring + producer * S8003_DC_SIZE, S8003_DC_NONE);
	wmb();
	WRITE_ONCE(*(__le32 *)(ctl + S8003_RING_PRODUCER), cpu_to_le32(target));
	mb();	/* the index is in memory before the firmware is told */

	err = s8003_dc_doorbell(apple);
	if (!err) {
		err = read_poll_timeout(s8003_dc_consumer, consumer, drained || consumer == target,
					1000, S8003_DC_TIMEOUT_US, false, apple, &drained);
		if (!err)
			err = drained;
	}
	if (err)
		dev_err(apple_dev(apple), "GFX firmware did not take its first command: %d\n",
			err);
	return err;
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

	err = s8003_parse_dt(apple);
	if (err)
		return err;
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

	err = s8003_release(apple, q);
	if (!err)
		err = s8003_wait_alive(apple);
	if (!err) {
		dev_info(apple_dev(apple), "GFX firmware is up (power state %u)\n", q);
		err = s8003_first_command(apple);
	}
	if (err) {
		/* It must not run on once the kernel VM, its page tables, may go */
		s8003_core_off(apple);
		return err;
	}
	dev_info(apple_dev(apple), "GFX firmware takes commands\n");
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

const struct pvr_apple_chip pvr_apple_chip_s8003 = {
	.fw_name = "apple/gfx-s8003.bin",
	.perf_state_max = S8003_PERF_STATE_MAX,
	.init = pvr_apple_s8003_init,
	.boot = pvr_apple_s8003_boot,
	.start = pvr_apple_s8003_start,
	.stop = pvr_apple_s8003_stop,
	.idle = pvr_apple_s8003_idle,
	.fini = pvr_apple_s8003_fini,
	.restore_image = pvr_apple_s8003_restore_image,
};
