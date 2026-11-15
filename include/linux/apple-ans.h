/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_APPLE_ANS_H
#define _LINUX_APPLE_ANS_H

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#define APPLE_ANS_NVMMU_MAX_REQS	36
#define APPLE_ANS_NVMMU_MAX_PAGES	256
#define APPLE_ANS_NVMMU_PAGE_SIZE	4096

/*
 * Map @npages pages for FlatDMA request @tag, or retire the tag if @npages is
 * zero. @to_memory: the controller writes the pages (an NVMe read).
 */
#if IS_REACHABLE(CONFIG_PCIE_APPLE_H9P)
int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
			const u64 *pages, unsigned int npages,
			bool to_memory, dma_addr_t *iova);
/*
 * System sleep, in which the storage port loses power.
 * apple_ans_nvmmu_drain(): no request is mapped and the NVMMU has no
 * invalidation left to do. apple_ans_stopped(): the controller is disabled
 * and off the bus and its translations are retired, so the host may power
 * the port off.
 */
int apple_ans_nvmmu_drain(struct device *dev);
int apple_ans_stopped(struct device *dev);
#else
static inline int apple_ans_nvmmu_drain(struct device *dev)
{
	return -ENODEV;
}

static inline int apple_ans_stopped(struct device *dev)
{
	return -ENODEV;
}

static inline int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
				      const u64 *pages, unsigned int npages,
				      bool to_memory, dma_addr_t *iova)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_APPLE_ANS_H */
