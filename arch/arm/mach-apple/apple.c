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
#include <asm/mach/arch.h>
#include <asm/memory.h>

pid_t __init apple_s5l_rest_spawn_init(int (*fn)(void *), void *arg,
				       unsigned long flags);

void __init apple_s5l_pmccntr_enable_counter(void);
void __init apple_s5l_pmccntr_init(unsigned long rate);
unsigned long __init apple_s5l_calibrate_cpu_hz(void);
void __init apple_s5l_wdt_clocksource_init(void);
void __init apple_s5l_pmu_clkevt_init(unsigned long cpu_hz);

/* P105AP recovery scanout address. */
#define P105_FB_PHYS	0x9f6fc000

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
	.init_early	= apple_s5l_a9_errata,
	.init_irq	= apple_s5l_init_irq,
	.init_time	= apple_s5l_init_time,
	/* Keep iBoot's AUX (16 x 64 KB); a mask at all is what makes
	 * init_IRQ() bring the PL310 up from the DT (dts: cache-controller). */
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
MACHINE_END
