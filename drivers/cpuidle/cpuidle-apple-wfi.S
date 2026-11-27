/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WFI that may power the core down with state retention. The core keeps SP,
 * PC and its system registers but may come back with its general-purpose
 * registers zeroed (XNU's arm64_retention_wfi tests LR for zero). Keep what
 * the caller relies on, and SP_EL0 (current) to be safe, on the stack.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

SYM_FUNC_START(apple_cpuidle_retention_wfi)
	stp	x29, x30, [sp, #-112]!
	mov	x29, sp
	stp	x18, x19, [sp, #16]
	stp	x20, x21, [sp, #32]
	stp	x22, x23, [sp, #48]
	stp	x24, x25, [sp, #64]
	stp	x26, x27, [sp, #80]
	mrs	x9, sp_el0
	stp	x28, x9, [sp, #96]
	mov	x30, #1
	dsb	sy
	wfi
	cmp	x30, #0
	cset	x0, eq
	ldp	x28, x9, [sp, #96]
	msr	sp_el0, x9
	ldp	x26, x27, [sp, #80]
	ldp	x24, x25, [sp, #64]
	ldp	x22, x23, [sp, #48]
	ldp	x20, x21, [sp, #32]
	ldp	x18, x19, [sp, #16]
	ldp	x29, x30, [sp], #112
	ret
SYM_FUNC_END(apple_cpuidle_retention_wfi)
