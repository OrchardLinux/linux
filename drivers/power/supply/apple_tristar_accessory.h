/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Lightning accessory policy of the Tristar charger: the interface between
 * the charger and the policy. See apple_tristar_accessory.c.
 *
 * Copyright The Orchard Linux contributors
 */
#ifndef _APPLE_TRISTAR_ACCESSORY_H
#define _APPLE_TRISTAR_ACCESSORY_H

#include <linux/kconfig.h>
#include <linux/types.h>

struct device;
struct i2c_client;
struct mutex;
struct tristar_accessory;

#if IS_ENABLED(CONFIG_APPLE_TRISTAR_ACCESSORY)
int tristar_accessory_probe(struct device *dev, struct i2c_client *tristar,
			    struct mutex *lock, struct tristar_accessory **accp);
void tristar_accessory_start(struct tristar_accessory *acc);
void tristar_accessory_stop(struct tristar_accessory *acc);
void tristar_accessory_report(struct tristar_accessory *acc, unsigned int usb_ma,
			      bool mux_fault);
const char *tristar_accessory_holds(struct tristar_accessory *acc, unsigned int *ma);
bool tristar_accessory_identifying(struct tristar_accessory *acc);
#else
static inline int tristar_accessory_probe(struct device *dev, struct i2c_client *tristar,
					  struct mutex *lock,
					  struct tristar_accessory **accp)
{
	return 0;
}

static inline void tristar_accessory_start(struct tristar_accessory *acc)
{
}

static inline void tristar_accessory_stop(struct tristar_accessory *acc)
{
}

static inline void tristar_accessory_report(struct tristar_accessory *acc,
					    unsigned int usb_ma, bool mux_fault)
{
}

static inline const char *tristar_accessory_holds(struct tristar_accessory *acc,
						  unsigned int *ma)
{
	return NULL;
}

static inline bool tristar_accessory_identifying(struct tristar_accessory *acc)
{
	return false;
}
#endif

#endif /* _APPLE_TRISTAR_ACCESSORY_H */
