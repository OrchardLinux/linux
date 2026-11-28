// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU idle for Apple SoCs, which have no PSCI.
 *
 * Without a cpuidle driver the idle loop never reaches the s2idle path, so
 * suspend-to-idle keeps the tick running and does not suspend timekeeping:
 * CLOCK_MONOTONIC counts the sleep, and every service watchdog armed before a
 * long sleep expires on wake. The core only enters a state above index 0 for
 * s2idle, and only one with a non-zero exit latency, hence the second state.
 *
 * State 0 is WFI as m1n1 sets it up (CYC_OVRD WFI mode 2): the core is only
 * clock gated. On cores with architectural state retention (AIDR_EL1 bit 2;
 * the A12 and later, HAS_RETENTION_STATE in XNU) state 1 lets WFI power the
 * core down (WFI mode 3), as iOS's idle does. The core keeps its system
 * registers and SP but may lose its general-purpose and FP/SIMD registers,
 * which XNU detects by LR reading zero after the WFI; here the integer
 * registers are kept on the stack and FP/SIMD state goes through cpu_pm.
 * Older cores keep plain WFI in state 1: powered down they would restart at
 * their reset vector.
 */

#include <linux/bitfield.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#include <asm/barrier.h>
#include <asm/sysreg.h>

#define SYS_IMP_APL_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define CYC_OVRD_WFI_MODE		GENMASK(25, 24)
#define CYC_OVRD_DISABLE_WFI_RET	BIT(0)
#define AIDR_EL1_ARCH_RETENTION		BIT(2)

/* Returns true when the core lost its registers in the WFI. */
asmlinkage bool apple_cpuidle_retention_wfi(void);

static bool retention = true;
module_param(retention, bool, 0644);
MODULE_PARM_DESC(retention, "let idle cores power down where they retain state");

static atomic_long_t apple_cpuidle_lost = ATOMIC_LONG_INIT(0);

static int lost_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%ld\n", atomic_long_read(&apple_cpuidle_lost));
}

static const struct kernel_param_ops lost_ops = { .get = lost_get };
module_param_cb(lost, &lost_ops, NULL, 0444);
MODULE_PARM_DESC(lost, "WFIs in which a core powered down and lost its registers");

static __cpuidle int apple_cpuidle_wfi(struct cpuidle_device *dev,
				       struct cpuidle_driver *drv, int index)
{
	dsb(sy);
	wfi();
	return index;
}

static __cpuidle int apple_cpuidle_retention(struct cpuidle_device *dev,
					     struct cpuidle_driver *drv,
					     int index)
{
	bool lost;
	u64 cyc;

	if (!READ_ONCE(retention)) {
		ct_cpuidle_enter();
		apple_cpuidle_wfi(dev, drv, index);
		ct_cpuidle_exit();
		return index;
	}

	/* Saves FP/SIMD state, which the core may lose. */
	if (cpu_pm_enter()) {
		ct_cpuidle_enter();
		apple_cpuidle_wfi(dev, drv, index);
		ct_cpuidle_exit();
		return index;
	}

	ct_cpuidle_enter();
	cyc = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);
	write_sysreg_s((cyc & ~(CYC_OVRD_WFI_MODE | CYC_OVRD_DISABLE_WFI_RET)) |
		       FIELD_PREP(CYC_OVRD_WFI_MODE, 3), SYS_IMP_APL_CYC_OVRD);
	isb();
	lost = apple_cpuidle_retention_wfi();
	write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
	isb();
	ct_cpuidle_exit();

	if (lost)
		atomic_long_inc(&apple_cpuidle_lost);
	cpu_pm_exit();
	return index;
}

static struct cpuidle_driver apple_cpuidle_driver = {
	.name = "apple_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = apple_cpuidle_wfi,
			.exit_latency = 1,
			.target_residency = 1,
			.name = "WFI",
			.desc = "WFI, clock gated",
		},
		{
			.enter = apple_cpuidle_wfi,
			.enter_s2idle = apple_cpuidle_wfi,
			.exit_latency = 1,
			.target_residency = 1000,
			.name = "WFI-S2IDLE",
			.desc = "WFI, for suspend-to-idle",
		},
	},
	.state_count = 2,
};

static const char *const apple_cpuidle_socs[] = {
	"apple,s5l8960x",
	"apple,t7000",
	"apple,t7001",
	"apple,s8000",
	"apple,s8003",
	"apple,s8001",
	"apple,t8010",
	"apple,t8011",
	"apple,t8015",
	"apple,t8020",
	NULL
};

static int __init apple_cpuidle_init(void)
{
	struct cpuidle_state *s = &apple_cpuidle_driver.states[1];

	if (!of_machine_compatible_match(apple_cpuidle_socs))
		return -ENODEV;

	if (read_sysreg(aidr_el1) & AIDR_EL1_ARCH_RETENTION) {
		/*
		 * On the iPhone XR a 2 ms sleep overshoots by 15 us more than
		 * with clock gating; the residency is a guess on the safe side.
		 */
		s->enter = apple_cpuidle_retention;
		s->enter_s2idle = apple_cpuidle_retention;
		s->flags |= CPUIDLE_FLAG_RCU_IDLE;
		s->exit_latency = 20;
		s->target_residency = 500;
		strscpy(s->name, "RET", CPUIDLE_NAME_LEN);
		strscpy(s->desc, "WFI, core powered down, state retained",
			CPUIDLE_DESC_LEN);
		pr_info("apple-cpuidle: cores power down in idle (state retention)\n");
	}

	return cpuidle_register(&apple_cpuidle_driver, NULL);
}
module_init(apple_cpuidle_init);

static void __exit apple_cpuidle_exit(void)
{
	cpuidle_unregister(&apple_cpuidle_driver);
}
module_exit(apple_cpuidle_exit);

MODULE_DESCRIPTION("CPU idle for Apple SoCs");
MODULE_LICENSE("GPL");
