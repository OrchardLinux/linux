// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Idle VM residency for faultless PowerVR devices. */
#include "pvr_device.h"
#include "pvr_gem.h"
#include "pvr_vm.h"

#include <drm/drm_debugfs.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <linux/debugfs.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/swap.h>

static bool evict = true;
module_param(evict, bool, 0644);
MODULE_PARM_DESC(evict, "Evict idle native BOs (default on)");
static unsigned int evict_idle_ms = 1000;
module_param(evict_idle_ms, uint, 0644);
MODULE_PARM_DESC(evict_idle_ms, "Quiet time after VM retirement in milliseconds");
static bool evict_log;
module_param(evict_log, bool, 0644);
MODULE_PARM_DESC(evict_log, "Log BO eviction");

void pvr_gem_lru_update_locked(struct pvr_gem_object *bo)
{
	struct drm_gem_object *obj = gem_from_pvr_gem(bo);
	struct pvr_device *dev = to_pvr_device(obj->dev);
	struct drm_gem_lru *lru = &dev->gem_reclaim.evictable;

	dma_resv_assert_held(obj->resv);
	if (!bo->base.sgt || drm_gem_is_imported(obj))
		return;
	if ((bo->flags & (PVR_BO_KERNEL_PINNED | DRM_PVR_BO_PM_FW_PROTECT)) ||
	    bo->exported || obj->dma_buf || refcount_read(&bo->base.vmap_use_count) ||
	    refcount_read(&bo->base.pages_pin_count))
		lru = &dev->gem_reclaim.pinned;
	drm_gem_lru_move_tail(lru, obj);
}

static bool pvr_gem_evict(struct drm_gem_object *obj, struct ww_acquire_ctx *unused)
{
	struct pvr_device *dev = to_pvr_device(obj->dev);
	struct pvr_gem_object *bo = gem_to_pvr_gem(obj);
	struct drm_gem_shmem_object *shmem = &bo->base;
	unsigned int reason;

	if (drm_gem_is_imported(obj))
		reason = 0;
	else if (bo->exported || obj->dma_buf)
		reason = 1;
	else if (bo->flags & (PVR_BO_KERNEL_PINNED | DRM_PVR_BO_PM_FW_PROTECT))
		reason = 2;
	else if (refcount_read(&shmem->vmap_use_count) || refcount_read(&shmem->pages_pin_count))
		reason = 3;
	else if (!shmem->sgt || refcount_read(&shmem->pages_use_count) != 1)
		reason = 4;
	else if (!dma_resv_test_signaled(obj->resv, DMA_RESV_USAGE_BOOKKEEP))
		reason = 5;
	else if (pvr_vm_evict_bo(obj, READ_ONCE(evict_idle_ms), &reason)) {
		pvr_gem_release_pages_locked(bo);
		atomic64_inc(&dev->gem_reclaim.evicts);
		if (READ_ONCE(evict_log))
			drm_info(&dev->base, "evicted BO size=%zu\n", obj->size);
		return true;
	}
	atomic64_inc(&dev->gem_reclaim.skip[reason]);
	return false;
}

static unsigned long pvr_gem_reclaim(struct pvr_device *dev, unsigned long pages)
{
	unsigned long remaining = 0, freed = 0;

	if (!READ_ONCE(evict) || !mutex_trylock(&dev->gem_reclaim.scan_lock))
		return SHRINK_STOP;
	if (dev->gem_reclaim.stopped) {
		mutex_unlock(&dev->gem_reclaim.scan_lock);
		return SHRINK_STOP;
	}
	atomic64_inc(&dev->gem_reclaim.scans);
	if (get_nr_swap_pages() > 0)
		freed = drm_gem_lru_scan(&dev->base, &dev->gem_reclaim.evictable,
					min(pages, (unsigned long)UINT_MAX), &remaining,
					pvr_gem_evict, NULL);
	mutex_unlock(&dev->gem_reclaim.scan_lock);
	return freed ? freed : SHRINK_STOP;
}

/* IMG's KCCB invalidation takes power/CCB locks and can reset firmware.
 * Run that path outside direct reclaim; Apple publishes a RAM-only tag.
 */
static void pvr_gem_reclaim_work(struct work_struct *work)
{
	struct pvr_device *dev = container_of(work, struct pvr_device, gem_reclaim.work);
	unsigned long pages = atomic_long_xchg(&dev->gem_reclaim.requested, 0);

	if (pages)
		pvr_gem_reclaim(dev, pages);
}

static unsigned long pvr_gem_count(struct shrinker *shrinker, struct shrink_control *sc)
{
	struct pvr_device *dev = shrinker->private_data;

	if (!READ_ONCE(evict) || get_nr_swap_pages() <= 0)
		return 0;
	return READ_ONCE(dev->gem_reclaim.evictable.count) +
	       READ_ONCE(dev->gem_reclaim.dontneed.count);
}

static unsigned long pvr_gem_scan(struct shrinker *shrinker, struct shrink_control *sc)
{
	struct pvr_device *dev = shrinker->private_data;

	if (!current_is_kswapd() &&
	    (sc->gfp_mask & (__GFP_FS | __GFP_IO)) != (__GFP_FS | __GFP_IO))
		return SHRINK_STOP;
	if (!pvr_device_is_apple(dev)) {
		atomic_long_set(&dev->gem_reclaim.requested, sc->nr_to_scan);
		schedule_work(&dev->gem_reclaim.work);
		return SHRINK_STOP;
	}
	return pvr_gem_reclaim(dev, sc->nr_to_scan);
}

static int pvr_gem_evict_show(struct seq_file *m, void *unused)
{
	struct pvr_device *dev = m->private;
	static const char *const reasons[] = {
		"imported", "exported", "protected", "pinned", "pages", "bo-busy",
		"vm-trylock", "vm-busy", "young", "chip-veto", "op-alloc", "mmu-flush",
	};
	unsigned long resident, dontneed;
	unsigned int i;

	mutex_lock(&dev->base.gem_lru_mutex);
	dontneed = dev->gem_reclaim.dontneed.count << PAGE_SHIFT;
	resident = (dev->gem_reclaim.evictable.count + dev->gem_reclaim.pinned.count +
		    dev->gem_reclaim.dontneed.count) << PAGE_SHIFT;
	mutex_unlock(&dev->base.gem_lru_mutex);
	seq_printf(m, "resident=%lu evicted=%lld dontneed=%lu\n", resident,
		   atomic64_read(&dev->gem_reclaim.evicted_bytes), dontneed);
	seq_printf(m, "scan=%lld evict=%lld purge=%lld restore=%lld restore-fail=%lld\n",
		   atomic64_read(&dev->gem_reclaim.scans), atomic64_read(&dev->gem_reclaim.evicts),
		   atomic64_read(&dev->gem_reclaim.purges), atomic64_read(&dev->gem_reclaim.restores),
		   atomic64_read(&dev->gem_reclaim.restore_fail));
	for (i = 0; i < ARRAY_SIZE(reasons); i++)
		seq_printf(m, "skip-%s=%lld\n", reasons[i], atomic64_read(&dev->gem_reclaim.skip[i]));
	return 0;
}

static int pvr_gem_evict_open(struct inode *inode, struct file *file)
{
	return single_open(file, pvr_gem_evict_show, inode->i_private);
}

static ssize_t pvr_gem_evict_write(struct file *file, const char __user *buf,
				 size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct pvr_device *dev = m->private;
	unsigned long mib;
	int err = kstrtoul_from_user(buf, len, 0, &mib);

	if (err)
		return err;
	if (mib > (UINT_MAX >> (20 - PAGE_SHIFT)))
		return -ERANGE;
	pvr_gem_reclaim(dev, mib << (20 - PAGE_SHIFT));
	return len;
}

static const struct file_operations pvr_gem_evict_fops = {
	.owner = THIS_MODULE,
	.open = pvr_gem_evict_open,
	.read = seq_read,
	.write = pvr_gem_evict_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void pvr_gem_shrinker_debugfs_init(struct pvr_device *dev, struct dentry *root)
{
	debugfs_create_file("gem_evict", 0600, root, dev, &pvr_gem_evict_fops);
}

void pvr_gem_shrinker_fini(struct pvr_device *dev)
{
	if (!dev->gem_reclaim.shrinker)
		return;
	mutex_lock(&dev->gem_reclaim.scan_lock);
	dev->gem_reclaim.stopped = true;
	mutex_unlock(&dev->gem_reclaim.scan_lock);
	shrinker_free(dev->gem_reclaim.shrinker);
	dev->gem_reclaim.shrinker = NULL;
	cancel_work_sync(&dev->gem_reclaim.work);
}

static void pvr_gem_shrinker_release(struct drm_device *drm, void *data)
{
	pvr_gem_shrinker_fini(data);
}

int pvr_gem_shrinker_init(struct pvr_device *dev)
{
	struct shrinker *shrinker;

	drm_gem_lru_init(&dev->gem_reclaim.evictable);
	drm_gem_lru_init(&dev->gem_reclaim.dontneed);
	drm_gem_lru_init(&dev->gem_reclaim.pinned);
	mutex_init(&dev->gem_reclaim.scan_lock);
	INIT_WORK(&dev->gem_reclaim.work, pvr_gem_reclaim_work);
	shrinker = shrinker_alloc(0, "drm-powervr");
	if (!shrinker)
		return -ENOMEM;
	dev->gem_reclaim.shrinker = shrinker;
	shrinker->private_data = dev;
	shrinker->count_objects = pvr_gem_count;
	shrinker->scan_objects = pvr_gem_scan;
	shrinker_register(shrinker);
	return drmm_add_action_or_reset(&dev->base, pvr_gem_shrinker_release, dev);
}
