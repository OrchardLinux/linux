/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_IRQCHIP_APPLE_AIC1_H
#define _LINUX_IRQCHIP_APPLE_AIC1_H

#include <linux/init.h>

#ifdef CONFIG_APPLE_AIC1
void __init apple_aic1_prepare_boot(void);
#else
static inline void apple_aic1_prepare_boot(void) {}
#endif

#endif
