// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/mm.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/io.h>
#include <asm/smp_plat.h>
#include <asm/sysreg.h>

extern void secondary_holding_pen(void);
volatile unsigned long __section(".mmuoff.data.read")
secondary_holding_pen_release = INVALID_HWID;

static phys_addr_t cpu_release_addr[NR_CPUS];

/*
 * Apple SoCs started by m1n1 through spin tables can take a core offline:
 * the PMGR block named by /cpus/apple,cpu-start stops and starts cores
 * (stop request at +0x0, system bit at +0x4, start at +0x8 + 4 * cluster),
 * and a stopped core powers down in a WFI without state retention. Started
 * again, it comes out of reset at its IORVBAR, which points into m1n1, and
 * m1n1 hands it to the holding pen through its spin table as at boot. The
 * PMGR takes only non-posted writes; a posted one raises an SError.
 */
static phys_addr_t apple_cpu_start_phys;
static void __iomem *apple_cpu_start;
static bool apple_cpu_stopped[NR_CPUS];

#define SYS_IMP_APL_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define CYC_OVRD_FIQ_MODE		GENMASK(21, 20)
#define CYC_OVRD_IRQ_MODE		GENMASK(23, 22)
#define CYC_OVRD_WFI_MODE		GENMASK(25, 24)
#define CYC_OVRD_DISABLE_WFI_RET	BIT(0)

static u32 apple_cpu_bit(unsigned int cpu, bool flat)
{
	u64 mpidr = cpu_logical_map(cpu);
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	unsigned int core = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	return flat ? BIT(4 * cluster + core) : BIT(core);
}

/*
 * Write secondary_holding_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_holding_pen_release;
	unsigned long size = sizeof(secondary_holding_pen_release);

	secondary_holding_pen_release = val;
	dcache_clean_inval_poc((unsigned long)start, (unsigned long)start + size);
}


static int smp_spin_table_cpu_init(unsigned int cpu)
{
	struct device_node *dn;
	int ret;

	dn = of_get_cpu_node(cpu, NULL);
	if (!dn)
		return -ENODEV;

	/*
	 * Determine the address from which the CPU is polling.
	 */
	ret = of_property_read_u64(dn, "cpu-release-addr",
				   &cpu_release_addr[cpu]);
	if (!apple_cpu_start_phys && dn->parent)
		of_property_read_u64(dn->parent, "apple,cpu-start",
				     &apple_cpu_start_phys);
	if (ret)
		pr_err("CPU %d: missing or invalid cpu-release-addr property\n",
		       cpu);

	of_node_put(dn);

	return ret;
}

static int smp_spin_table_cpu_prepare(unsigned int cpu)
{
	__le64 __iomem *release_addr;
	phys_addr_t pa_holding_pen = __pa_symbol(secondary_holding_pen);

	if (!cpu_release_addr[cpu])
		return -ENODEV;

	/*
	 * The cpu-release-addr may or may not be inside the linear mapping.
	 * As ioremap_cache will either give us a new mapping or reuse the
	 * existing linear mapping, we can use it to cover both cases. In
	 * either case the memory will be MT_NORMAL.
	 */
	release_addr = ioremap_cache(cpu_release_addr[cpu],
				     sizeof(*release_addr));
	if (!release_addr)
		return -ENOMEM;

	/*
	 * We write the release address as LE regardless of the native
	 * endianness of the kernel. Therefore, any boot-loaders that
	 * read this address need to convert this address to the
	 * boot-loader's endianness before jumping. This is mandated by
	 * the boot protocol.
	 */
	writeq_relaxed(pa_holding_pen, release_addr);
	dcache_clean_inval_poc((__force unsigned long)release_addr,
			    (__force unsigned long)release_addr +
				    sizeof(*release_addr));

	/*
	 * Send an event to wake up the secondary CPU.
	 */
	sev();

	iounmap(release_addr);

	return 0;
}

static int smp_spin_table_cpu_boot(unsigned int cpu)
{
	if (apple_cpu_stopped[cpu]) {
		unsigned int cluster = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);

		writel_relaxed(apple_cpu_bit(cpu, true), apple_cpu_start + 0x4);
		writel(apple_cpu_bit(cpu, false), apple_cpu_start + 0x8 + 4 * cluster);
		apple_cpu_stopped[cpu] = false;
	}

	/*
	 * Update the pen release flag.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send an event, causing the secondaries to read pen_release.
	 */
	sev();

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static bool smp_spin_table_cpu_can_disable(unsigned int cpu)
{
	return apple_cpu_start_phys;
}

static int smp_spin_table_cpu_disable(unsigned int cpu)
{
	if (!apple_cpu_start_phys)
		return -EOPNOTSUPP;
	if (!apple_cpu_start) {
		apple_cpu_start = ioremap_np(apple_cpu_start_phys, 0x10);
		if (!apple_cpu_start)
			return -ENOMEM;
	}
	return 0;
}

static void smp_spin_table_cpu_die(unsigned int cpu)
{
	u64 cyc;

	/*
	 * As iOS (AppleARMCPU::quiesceCPU, then arm64_prepare_for_sleep): the
	 * PMGR stop request while the core still runs, then a WFI that may
	 * power it down without state retention. The cluster-wide ACC_OVRD
	 * deep-sleep bits are left alone: the other cores keep running.
	 */
	writel(apple_cpu_bit(cpu, true), apple_cpu_start + 0x0);
	cyc = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);
	cyc &= ~(CYC_OVRD_FIQ_MODE | CYC_OVRD_IRQ_MODE | CYC_OVRD_WFI_MODE);
	cyc |= FIELD_PREP(CYC_OVRD_FIQ_MODE, 2) | FIELD_PREP(CYC_OVRD_IRQ_MODE, 2) |
	       FIELD_PREP(CYC_OVRD_WFI_MODE, 3) | CYC_OVRD_DISABLE_WFI_RET;
	write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
	isb();
	for (;;) {
		dsb(sy);
		wfi();
	}
}

static int smp_spin_table_cpu_kill(unsigned int cpu)
{
	/* No status register on the A12: give it the time m1n1 gives it. */
	msleep(10);
	apple_cpu_stopped[cpu] = true;
	return 0;
}
#endif

const struct cpu_operations smp_spin_table_ops = {
	.name		= "spin-table",
	.cpu_init	= smp_spin_table_cpu_init,
	.cpu_prepare	= smp_spin_table_cpu_prepare,
	.cpu_boot	= smp_spin_table_cpu_boot,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_can_disable = smp_spin_table_cpu_can_disable,
	.cpu_disable	= smp_spin_table_cpu_disable,
	.cpu_die	= smp_spin_table_cpu_die,
	.cpu_kill	= smp_spin_table_cpu_kill,
#endif
};
