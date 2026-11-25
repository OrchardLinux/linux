// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple 32-bit S5L SoC support (S5L8940X / S5L8942X, "A5").
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/sizes.h>
#include <linux/irq.h>
#include <linux/bits.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sched/task.h>
#include <asm/cputype.h>
#include <asm/barrier.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/memory.h>

void apple_aic1_arm_cpu0(void);
void apple_aic1_hw_quiesce(bool read_back);
pid_t __init apple_s5l_rest_spawn_init(int (*fn)(void *), void *arg,
				       unsigned long flags);

/*
 * Lab IRQ stub forced SPSR.I on return against sticky nIRQ.  Keep true through
 * first enables; apple_aic1_release_escape() clears it after 0x5004 drain +
 * successful irq_thaw (aic1-lab v30: I0 HOLD OK without force).
 */
bool apple_aic1_early_irq_escape = true;

void __init apple_s5l_pmccntr_enable_counter(void);
void __init apple_s5l_pmccntr_init(unsigned long rate);
unsigned long __init apple_s5l_calibrate_cpu_hz(void);
void __init apple_s5l_wdt_clocksource_init(void);
void __init apple_s5l_pmu_clkevt_init(unsigned long cpu_hz);

/* P105AP recovery scanout address. */
#define P105_FB_PHYS	0x9f6fc000

#define P105_AIC_PHYS	0x3f200000
#define P105_AIC_VIRT	0xF0500000UL

/* aic1-lab v30: EVENT is per-CPU @ 0x5004; 0x2004 stays idle on aic,1 */
#define P105_AIC_CONFIG		0x0010
#define P105_AIC_EVENT		0x2004
#define P105_AIC_EVENT_CPU0	0x5004
#define P105_AIC_IPI_ACK	0x200c
#define P105_AIC_IPI_MASK_SET	0x2024
#define P105_AIC_SW_CLR		0x4080
#define P105_AIC_MASK_SET	0x4100
#define P105_AIC_TARGET_CPU	0x3000
#define P105_AIC_NR_IRQ		192
#define P105_AIC_NR_WORDS	6

/*
 * For the early quiesce/rearm code only.  The AIC driver maps the block again
 * itself, strongly-ordered -- see aic1_of_init() in irq-apple-aic1.c.  (Made
 * MT_UNCACHED here, this static map came out without page tables and the
 * driver, reusing it, faulted at the first read.)
 */
static struct map_desc apple_aic_desc __initdata = {
	.virtual	= P105_AIC_VIRT,
	.pfn		= __phys_to_pfn(P105_AIC_PHYS),
	.length		= SZ_1M,
	.type		= MT_DEVICE,
};

/*
 * Lab: CBAR phys 0x3E100000 R/W is fine bare-metal.  Under Linux, both
 * ioremap and static MT_DEVICE map of that window hang (aic1q33 stuck after
 * printing CP15 CBAR).  Skip GIC MMIO; rely on AIC EVENT@0x5004 drain.
 */
void apple_a9_gic_drain(void)
{
	/* no CBAR map — AIC drain only */
}

/*
 * Early aic,1 quiesce (aic1-lab v30 P0 + E5 drain):
 * TARGET=0, MASK, IPI, drain EVENT@0x5004 (and 0x2004), CFG.enable=0.
 * Never PMGR gate 0x4D while AIC must live.
 */
void apple_aic1_hw_quiesce(bool read_back)
{
	void __iomem *base = (void __iomem *)P105_AIC_VIRT;
	unsigned int i, n, cpu;
	u32 cfg;

	for (i = 0; i < P105_AIC_NR_WORDS; i++) {
		writel_relaxed(~0U, base + P105_AIC_MASK_SET + i * 4);
		writel_relaxed(~0U, base + P105_AIC_SW_CLR + i * 4);
	}

	for (i = 0; i < P105_AIC_NR_IRQ; i++)
		writel_relaxed(0, base + P105_AIC_TARGET_CPU + i * 4);

	writel_relaxed(~0U, base + P105_AIC_IPI_MASK_SET);
	writel_relaxed(BIT(31) | BIT(0), base + P105_AIC_IPI_ACK);
	writel_relaxed(~0U, base + P105_AIC_IPI_MASK_SET);
	for (i = 0; i < 2; i++) {
		writel_relaxed(BIT(31) | BIT(0), base + 0x500c + (i << 7));
		writel_relaxed(~0U, base + 0x5024 + (i << 7));
	}

	for (n = 0; n < 512; n++) {
		u32 any = readl_relaxed(base + P105_AIC_EVENT);

		for (cpu = 0; cpu < 2; cpu++)
			any |= readl_relaxed(base + P105_AIC_EVENT_CPU0 +
					     (cpu << 7));
		if (!any)
			break;
	}

	for (i = 0; i < P105_AIC_NR_WORDS; i++)
		writel_relaxed(~0U, base + P105_AIC_MASK_SET + i * 4);

	cfg = readl_relaxed(base + P105_AIC_CONFIG);
	cfg |= 0xE0000000U;
	cfg &= ~BIT(0);
	writel_relaxed(cfg, base + P105_AIC_CONFIG);
	dsb(sy);

	if (read_back) {
		/*
		 * Reading EVENT consumes a pending event. Keep the readback
		 * sequence used during the first quiesce.
		 */
		readl_relaxed(base + P105_AIC_CONFIG);
		readl_relaxed(base + P105_AIC_EVENT_CPU0);
		readl_relaxed(base + P105_AIC_TARGET_CPU);
	}
}

/* Affinity to CPU0 after IRQs are on; drain per-CPU EVENT. */
void apple_aic1_arm_cpu0(void)
{
	void __iomem *base = (void __iomem *)P105_AIC_VIRT;
	unsigned int i, n, cpu;

	for (i = 0; i < P105_AIC_NR_WORDS; i++) {
		writel_relaxed(~0U, base + P105_AIC_MASK_SET + i * 4);
		writel_relaxed(~0U, base + P105_AIC_SW_CLR + i * 4);
	}
	for (i = 0; i < P105_AIC_NR_IRQ; i++)
		writel_relaxed(BIT(0), base + P105_AIC_TARGET_CPU + i * 4);
	for (i = 0; i < P105_AIC_NR_WORDS; i++)
		writel_relaxed(~0U, base + P105_AIC_MASK_SET + i * 4);
	for (n = 0; n < 512; n++) {
		u32 any = readl_relaxed(base + P105_AIC_EVENT);

		for (cpu = 0; cpu < 2; cpu++)
			any |= readl_relaxed(base + P105_AIC_EVENT_CPU0 +
					     (cpu << 7));
		if (!any)
			break;
	}
}

pid_t __init apple_s5l_rest_spawn_init(int (*fn)(void *), void *arg,
				       unsigned long flags)
{
	return user_mode_thread(fn, arg, flags | CLONE_FILES);
}

static void __init apple_s5l_reserve(void)
{
	phys_addr_t phys = P105_FB_PHYS & ~(SZ_1M - 1);

	memblock_remove(phys, P105_FB_PHYS - phys);
}

static void __init apple_s5l_map_io(void)
{
	iotable_init(&apple_aic_desc, 1);
}

/*
 * Cortex-A9 errata that multi_v7 cannot apply for us -- the workaround is a
 * write to the diagnostic register, which only secure code may make, and
 * this kernel runs secure.  The A5's cores are r2p8:
 *   743622 (r2p*): faulty hazard checking in the store buffer can corrupt
 *                  data -- diagnostic bit 6;
 *   751472 (before r3p0, SMP): an interrupted ICIALLUIS may never complete,
 *                  and the core waiting on it hangs -- bit 11.
 * CPU1 sets the same bits in headsmp.S; cpu_resume restores the register
 * after power-down idle.
 */
static void __init apple_s5l_a9_errata(void)
{
	u32 midr = read_cpuid_id(), variant = (midr >> 20) & 0xf, diag;

	if (read_cpuid_part() != ARM_CPU_PART_CORTEX_A9)
		return;
	asm volatile("mrc p15, 0, %0, c15, c0, 1" : "=r" (diag));
	if (variant == 2)
		diag |= BIT(6);
	if (variant < 3)
		diag |= BIT(11);
	asm volatile("mcr p15, 0, %0, c15, c0, 1" : : "r" (diag));
	pr_info("Cortex-A9 r%up%u: errata 743622/751472, diagnostic register %#x\n",
		variant, midr & 0xf, diag);
}

/*
 * The L2's other half.  The ADT's pl310 node has two register blocks: the
 * PL310 itself and Apple's CIF at 0x3fd00000.  iOS 6.1's AppleS5L8940XPL310,
 * enabling the L2 (its 'pmtc' platform function), configures the PL310 and
 * then, right before the enable bit, writes CIF+0x1020 = 1 and CIF+0x1120 =
 * 0x80000100.  Without them one core ran fine with the L2 on, and two cores
 * under load froze the whole machine, silently, within a minute.
 * init_IRQ() enables the L2 as soon as the machine's init_irq returns.
 */
static void __init apple_s5l_cif_l2_on(void)
{
	struct device_node *np;
	void __iomem *cif;

	np = of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
	if (!np || !of_device_is_available(np))
		goto out;
	cif = of_iomap(np, 1);
	if (!cif) {
		pr_warn("L2 CIF: no second reg in the cache-controller node\n");
		goto out;
	}
	pr_info("L2 CIF: +0x1020 %#x -> 0x1, +0x1120 %#x -> 0x80000100\n",
		readl_relaxed(cif + 0x1020), readl_relaxed(cif + 0x1120));
	writel(1, cif + 0x1020);
	writel(0x80000100, cif + 0x1120);
	iounmap(cif);
out:
	of_node_put(np);
}

static void __init apple_s5l_init_irq(void)
{
	irqchip_init();
	apple_s5l_cif_l2_on();
}

static void __init apple_s5l_init_time(void)
{
	unsigned long cpu_hz;

	/* Enable the cycle counter first, then measure it against the watchdog's
	 * exact 24 MHz reference, and only then register the clocksource -- its
	 * rate used to be a hardcoded 1 GHz guess, which made all wall-clock time
	 * wrong.  The same measured figure is what the PMU tick is scaled by. */
	apple_s5l_pmccntr_enable_counter();
	cpu_hz = apple_s5l_calibrate_cpu_hz();

	/* Wall time comes from the 24 MHz watchdog counter, NOT from PMCCNTR:
	 * PMCCNTR stops in WFI, so with it as the clocksource ktime only advanced
	 * while the CPU was busy and sleep(1) took ~13 real seconds. */
	apple_s5l_wdt_clocksource_init();
	apple_s5l_pmccntr_init(cpu_hz);

	/* Tick from the PMU overflow interrupt; it self-tests at late_initcall and
	 * leaves the system exactly as tickless as before if the IRQ never lands. */
	apple_s5l_pmu_clkevt_init(cpu_hz);
}

static const char * const apple_s5l_dt_compat[] __initconst = {
	"apple,s5l8940x",
	"apple,s5l8942x",
	NULL
};

DT_MACHINE_START(APPLE_S5L, "Apple S5L (Device Tree)")
	.dt_compat	= apple_s5l_dt_compat,
	.reserve	= apple_s5l_reserve,
	.map_io		= apple_s5l_map_io,
	.init_early	= apple_s5l_a9_errata,
	.init_irq	= apple_s5l_init_irq,
	.init_time	= apple_s5l_init_time,
	/* Keep iBoot's AUX (16 x 64 KB); a mask at all is what makes
	 * init_IRQ() bring the PL310 up from the DT (dts: cache-controller). */
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
MACHINE_END
