// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple's GPU firmware: Rogue fixed-function hardware driven by Apple's GFX
 * firmware instead of Imagination's.
 *
 * This file is what the chips share: job submission through the firmware's
 * work rings, completion from its stamps and events, the order of geometry
 * and fragment jobs, the GPU's power and performance state (Fender), runtime
 * PM, sleep and recovery. The processor the firmware runs on, its memory
 * layout and its commands belong to the chip (struct pvr_apple_chip,
 * pvr_apple_chip.h): pvr_apple_t8010.c for the A10, pvr_apple_s8003.c for the
 * A9.
 */

#include "pvr_apple.h"
#include "pvr_apple_chip.h"
#include "pvr_apple_layout.h"
#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_drv.h"
#include "pvr_free_list.h"
#include "pvr_gem.h"
#include "pvr_hwrt.h"
#include "pvr_job.h"
#include "pvr_mmu.h"
#include "pvr_power.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_heap_config.h"
#include "pvr_stream_defs.h"
#include "pvr_vm.h"

#include <drm/drm_print.h>
#include <drm/gpu_scheduler.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>

struct pvr_apple_fence {
	struct dma_fence base;
	struct list_head node;
	struct drm_gpu_scheduler *sched;
	ktime_t submitted;
	bool firmware_fault;
};

/*
 * How often the stamps are checked while jobs are pending. A jiffy-based
 * delay (1-2 ms per check) cost a render about 4 ms: a geometry and a
 * fragment wait each.
 */
static unsigned int pvr_apple_poll_us = 100;
module_param_named(apple_poll_us, pvr_apple_poll_us, uint, 0644);
MODULE_PARM_DESC(apple_poll_us, "Apple A10: microseconds between job completion checks (default 100)");

/*
 * With the firmware's interrupt taken, checks stay apple_poll_us apart for
 * this long after the firmware announced a completion: the stamps and ring
 * watermarks that make a job done are not all in memory at once.
 */
#define PVR_APPLE_FAST_US	1000

/* Idle time before runtime PM powers the GPU off; tunable in sysfs. */
#define PVR_APPLE_AUTOSUSPEND_MS	1000

static unsigned int pvr_apple_perf_state;
module_param_named(apple_perf_state, pvr_apple_perf_state, uint, 0444);
MODULE_PARM_DESC(apple_perf_state,
		 "Apple A10: fixed GPU performance state, 1 (slowest) up; 0 (default) scales it with the load (devfreq), or runs the fastest without an OPP table");

static struct pvr_apple *to_apple(struct pvr_device *pvr_dev)
{
	return pvr_dev->apple;
}

/* Ask Fender for power state @state and wait until it reports it */
int pvr_apple_request_power_state(struct pvr_apple *apple, u32 state)
{
	u32 val;
	int err;

	writel(FENDER_POWER_REQUEST | state, apple->fender + FENDER_POWER_STATE);
	err = readl_poll_timeout(apple->fender + FENDER_POWER_STATE, val,
				 (val & FENDER_POWER_STATE_MASK) == state, 100, 100000);
	if (err)
		dev_err(apple_dev(apple), "Fender power state %u not reached (%#x)\n", state, val);
	else
		apple->perf_state = state;
	return err;
}

/**
 * pvr_apple_start_power_state() - The power state the core starts in
 * @apple: Apple firmware state.
 *
 * Return: the fixed state if one is set, else the fastest. Never state 0:
 * that is off, and the firmware is about to start.
 */
u32 pvr_apple_start_power_state(struct pvr_apple *apple)
{
	return clamp(pvr_apple_perf_state ?: apple->perf_state_max, 1u,
		     apple->perf_state_max ?: 1u);
}

/**
 * pvr_apple_fender_power_on() - Power the Rogue core up
 * @apple: Apple firmware state.
 *
 * For a chip's boot(), once its firmware runs far enough to serve Fender.
 *
 * Return: 0 or a negative error.
 */
int pvr_apple_fender_power_on(struct pvr_apple *apple)
{
	u32 state;
	int err;

	/* A core left on by a previous boot is reset through state 0 first. */
	state = readl(apple->fender + FENDER_POWER_STATE) & FENDER_POWER_STATE_MASK;
	if (state != 0) {
		err = pvr_apple_request_power_state(apple, 0);
		if (err)
			return err;
	}
	/*
	 * Powering up and changing the clock are the same request: start in
	 * the fixed state if one is set, else the fastest; devfreq takes it
	 * from there.
	 */
	mutex_lock(&apple->perf_lock);
	err = pvr_apple_request_power_state(apple, pvr_apple_start_power_state(apple));
	mutex_unlock(&apple->perf_lock);
	return err;
}

static void pvr_apple_poll_work(struct work_struct *work);
static enum hrtimer_restart pvr_apple_poll_timer(struct hrtimer *timer);
static u32 pvr_apple_frag_retired(struct pvr_apple *apple);

/*
 * GPU frequency scaling: devfreq picks an OPP from the time jobs were in
 * flight, and the OPP's level is the performance state to request. The
 * firmware is left out: its own DVFS is off (FENDER.md section 9).
 */
/* The platform device's driver data is the DRM device (pvr_probe()) */
static struct pvr_apple *pvr_apple_from_dev(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	return to_apple(to_pvr_device(drm_dev));
}

static int pvr_apple_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct pvr_apple *apple = pvr_apple_from_dev(dev);
	struct dev_pm_opp *opp;
	u32 level;
	int err = 0;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	level = dev_pm_opp_get_level(opp);
	*freq = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	mutex_lock(&apple->perf_lock);
	if (apple->booted && level != apple->perf_state) {
		dev_dbg(dev, "performance state %u -> %u\n", apple->perf_state, level);
		err = pvr_apple_request_power_state(apple, level);
	}
	mutex_unlock(&apple->perf_lock);
	return err;
}

static int pvr_apple_devfreq_status(struct device *dev, struct devfreq_dev_status *status)
{
	struct pvr_apple *apple = pvr_apple_from_dev(dev);
	ktime_t now = ktime_get();
	unsigned long flags;
	u64 busy;

	spin_lock_irqsave(&apple->fence_lock, flags);
	busy = apple->busy_ns;
	if (apple->busy_since) {
		busy += ktime_to_ns(ktime_sub(now, apple->busy_since));
		apple->busy_since = now;
	}
	apple->busy_ns = 0;
	spin_unlock_irqrestore(&apple->fence_lock, flags);

	status->busy_time = busy;
	status->total_time = ktime_to_ns(ktime_sub(now, apple->stat_since));
	apple->stat_since = now;
	status->current_frequency = apple->devfreq ? apple->devfreq->previous_freq : 0;
	return 0;
}

static struct devfreq_simple_ondemand_data pvr_apple_ondemand = {
	/*
	 * A compositor frame must finish well inside its 16.7 ms: go fast
	 * early (above 40% busy) rather than near saturation.
	 */
	.upthreshold = 40,
	.downdifferential = 10,
};

/* The GPU's OPP table, if the bootloader filled one: its fastest state */
static void pvr_apple_opp_init(struct pvr_apple *apple)
{
	struct device *dev = apple_dev(apple);
	unsigned long freq = ULONG_MAX;
	struct dev_pm_opp *opp;

	apple->perf_state_max = apple->chip->perf_state_max;
	if (devm_pm_opp_of_add_table(dev) || dev_pm_opp_get_opp_count(dev) <= 0)
		return;
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return;
	apple->perf_state_max = dev_pm_opp_get_level(opp);
	dev_pm_opp_put(opp);
	dev_dbg(dev, "OPP table: %d states, fastest %lu Hz is state %u\n",
		dev_pm_opp_get_opp_count(dev), freq, apple->perf_state_max);
}

/* Scale the clock once the firmware runs: no state changes while it starts */
static void pvr_apple_devfreq_start(struct pvr_apple *apple)
{
	static struct devfreq_dev_profile profile = {
		.polling_ms = 20,
		.target = pvr_apple_devfreq_target,
		.get_dev_status = pvr_apple_devfreq_status,
	};
	struct device *dev = apple_dev(apple);
	unsigned long freq = ULONG_MAX;
	struct dev_pm_opp *opp;

	if (apple->devfreq || pvr_apple_perf_state || dev_pm_opp_get_opp_count(dev) <= 0)
		return;
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return;
	dev_pm_opp_put(opp);

	profile.initial_freq = freq;
	apple->stat_since = ktime_get();
	apple->devfreq = devm_devfreq_add_device(dev, &profile, DEVFREQ_GOV_SIMPLE_ONDEMAND,
						 &pvr_apple_ondemand);
	if (IS_ERR(apple->devfreq)) {
		dev_warn(dev, "no GPU frequency scaling: %pe\n", apple->devfreq);
		apple->devfreq = NULL;
	}
}

/* devm action: a booted firmware is stopped when the device goes (probe failure) */
static void pvr_apple_stop_firmware(void *data)
{
	struct pvr_apple *apple = data;

	if (!apple->booted)
		return;
	apple->chip->stop(apple, false);
	cancel_delayed_work_sync(&apple->poll_work);
	hrtimer_cancel(&apple->poll_timer);
	apple->booted = false;
}

/**
 * pvr_apple_fw_load() - Load the chip's firmware file
 * @apple: Apple firmware state.
 * @max_size: Largest image the chip takes.
 *
 * For a chip's init(). The image stays in &pvr_apple.fw_image for as long as
 * the device, for every later start of the firmware.
 *
 * Return: 0 or a negative error.
 */
int pvr_apple_fw_load(struct pvr_apple *apple, size_t max_size)
{
	struct device *dev = apple_dev(apple);
	const char *name = apple->chip->fw_name;
	const struct firmware *fw;
	int err;

	of_property_read_string(dev->of_node, "firmware-name", &name);
	err = request_firmware(&fw, name, dev);
	if (err)
		return dev_err_probe(dev, err, "cannot load %s\n", name);
	if (fw->size > max_size) {
		release_firmware(fw);
		return dev_err_probe(dev, -EINVAL, "%s too large\n", name);
	}
	apple->fw_image = devm_kmemdup(dev, fw->data, fw->size, GFP_KERNEL);
	apple->fw_size = fw->size;
	release_firmware(fw);
	if (!apple->fw_image)
		return -ENOMEM;
	return 0;
}

/* Track our one suspend reference, including jobless resumes and failed starts.
 * devfreq changes its suspend count even when its governor callback fails.
 * Do not take submit_lock here: devfreq callbacks have their own lock order.
 */
static void pvr_apple_devfreq_suspend(struct pvr_apple *apple)
{
	mutex_lock(&apple->devfreq_lock);
	if (apple->devfreq && !apple->devfreq_suspended) {
		devfreq_suspend_device(apple->devfreq);
		WRITE_ONCE(apple->devfreq_suspended, true);
	}
	mutex_unlock(&apple->devfreq_lock);
}

static void pvr_apple_devfreq_resume(struct pvr_apple *apple)
{
	mutex_lock(&apple->devfreq_lock);
	if (apple->devfreq && apple->devfreq_suspended) {
		devfreq_resume_device(apple->devfreq);
		WRITE_ONCE(apple->devfreq_suspended, false);
	}
	mutex_unlock(&apple->devfreq_lock);
}

static int pvr_apple_power_init(struct pvr_device *pvr_dev)
{
	struct device *dev = from_pvr_device(pvr_dev)->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct pvr_apple *apple;
	unsigned int i;
	int err;

	apple = devm_kzalloc(dev, sizeof(*apple), GFP_KERNEL);
	if (!apple)
		return -ENOMEM;
	apple->pvr_dev = pvr_dev;
	apple->chip = pvr_dev->device_data->apple_chip;
	pvr_dev->apple = apple;
	err = devm_mutex_init(dev, &apple->submit_lock);
	if (!err)
		err = devm_mutex_init(dev, &apple->perf_lock);
	if (!err)
		err = devm_mutex_init(dev, &apple->devfreq_lock);
	if (!err)
		err = devm_mutex_init(dev, &apple->pm_lock);
	if (err)
		return err;
	atomic_set(&apple->fault_pending, 0);
	spin_lock_init(&apple->fence_lock);
	spin_lock_init(&apple->event_lock);
	for (i = 0; i < APPLE_ENGINES; i++)
		INIT_LIST_HEAD(&apple->pending[i]);
	INIT_DELAYED_WORK(&apple->poll_work, pvr_apple_poll_work);
	hrtimer_setup(&apple->poll_timer, pvr_apple_poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	apple->fence_context = dma_fence_context_alloc(APPLE_ENGINES);

	apple->fender = devm_platform_ioremap_resource_byname(pdev, "fender");
	if (IS_ERR(apple->fender))
		return PTR_ERR(apple->fender);

	err = apple->chip->init(apple);
	if (err)
		return err;

	/*
	 * Reading the GPU ID powers the GPU on, which boots the firmware: a probe
	 * failing after that (the support check without exp_hw_support) left it
	 * running, and the next probe could not boot it (-ETIME) until a reboot.
	 * Stop it on the way out, as pvr_apple_fini() does for a bound device.
	 */
	err = devm_add_action(dev, pvr_apple_stop_firmware, apple);
	if (err)
		return err;

	/* Before powering on: power_on() requests the fastest state */
	pvr_apple_opp_init(apple);

	/*
	 * Each runtime resume boots the firmware again: idle for longer than
	 * a frame or two before powering it off.
	 */
	pm_runtime_set_autosuspend_delay(dev, PVR_APPLE_AUTOSUSPEND_MS);

	return 0;
}

static int pvr_apple_boot(struct pvr_apple *apple)
{
	int err;

	if (apple->booted)
		return 0;
	err = apple->chip->boot(apple);
	if (!err)
		apple->booted = true;
	return err;
}

static int pvr_apple_fw_restart(struct pvr_apple *apple);
static void pvr_apple_fw_stop(struct pvr_apple *apple);

/*
 * Runtime PM. Every job the firmware runs holds a runtime-PM reference from
 * pvr_apple_run_job() until its fence leaves the pending list, so the GPU
 * goes idle only with nothing in flight. Then the firmware is shut down and
 * the GPU powered off; the next job boots it again with fresh init data, as
 * after a reset. At probe, before the init data exists, power on only boots
 * the firmware: pvr_apple_fw_init() follows.
 */
static void pvr_apple_stat_add(struct pvr_apple_stat *stat, s64 us)
{
	stat->last = max(us, 0LL);
	stat->max = max(stat->max, stat->last);
	stat->total += stat->last;
}

static int pvr_apple_power_on(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	ktime_t start = ktime_get();
	unsigned long flags;

	WRITE_ONCE(apple->power_on_at, start);
	if (apple->chip->gate_off_ms && apple->gate_off) {
		s64 remaining = (s64)apple->chip->gate_off_ms * USEC_PER_MSEC -
				ktime_us_delta(start, apple->gate_off);

		if (remaining > 0) {
			msleep(DIV_ROUND_UP(remaining, USEC_PER_MSEC));
			spin_lock_irqsave(&apple->fence_lock, flags);
			pvr_apple_stat_add(&apple->rpm_gate_wait, ktime_us_delta(ktime_get(), start));
			spin_unlock_irqrestore(&apple->fence_lock, flags);
		}
	}
	/* The driver core resumes a device before .remove(). Powering the
	 * domain must not boot an idle firmware just to tear it down again.
	 * The first job after resume starts it in pvr_apple_prepare_job().
	 */
	if (READ_ONCE(apple->stopping))
		return -ENODEV;
	return apple->fw_ready ? 0 : pvr_apple_boot(apple);
}

int pvr_apple_prepare_job(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	ktime_t start = ktime_get(), on;
	unsigned long flags;
	bool resumed;
	int err;

	mutex_lock(&apple->submit_lock);
	resumed = !apple->booted;
	err = READ_ONCE(apple->stopping) ? -ENODEV :
		apple->booted ? 0 : pvr_apple_fw_restart(apple);
	mutex_unlock(&apple->submit_lock);
	if (resumed) {
		/* A start after a reset has no power-on of its own */
		on = xchg(&apple->power_on_at, 0);
		spin_lock_irqsave(&apple->fence_lock, flags);
		apple->rpm_resumes++;
		if (err) {
			apple->rpm_resume_failed++;
		} else {
			pvr_apple_stat_add(&apple->rpm_boot, ktime_us_delta(ktime_get(), start));
			if (on)
				pvr_apple_stat_add(&apple->rpm_resume, ktime_us_delta(ktime_get(), on));
			apple->first_job_from = on ?: start;
		}
		spin_unlock_irqrestore(&apple->fence_lock, flags);
	}
	if (err)
		dev_err(apple_dev(apple), "GPU did not start: %d\n", err);
	else if (READ_ONCE(apple->devfreq_suspended))
		pvr_apple_devfreq_resume(apple);
	return err;
}

static int pvr_apple_power_off(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	ktime_t start = ktime_get();
	unsigned long flags;
	int err;

	/* Terminal shutdown already stopped firmware; never wait on submitters. */
	if (READ_ONCE(apple->stopping))
		return 0;
	if (!apple->fw_ready)
		return 0;

	pvr_apple_devfreq_suspend(apple);
	mutex_lock(&apple->submit_lock);
	/*
	 * Nothing holds a job reference, so this does not wait in practice.
	 * A firmware that a reset stopped has nothing to wait for.
	 */
	err = apple->booted ? pvr_apple_wait_idle(apple, 1000000) : 0;
	if (!err) {
		pvr_apple_fw_stop(apple);
		apple->chip->idle(apple);
	}
	mutex_unlock(&apple->submit_lock);
	if (err) {
		pvr_apple_devfreq_resume(apple);
		return -EBUSY;
	}
	/* the pollers run only while fences are pending; none are */
	hrtimer_cancel(&apple->poll_timer);
	cancel_delayed_work_sync(&apple->poll_work);
	if (apple->chip->gate_off_ms)
		apple->gate_off = ktime_get();
	spin_lock_irqsave(&apple->fence_lock, flags);
	apple->rpm_suspends++;
	pvr_apple_stat_add(&apple->rpm_suspend, ktime_us_delta(ktime_get(), start));
	spin_unlock_irqrestore(&apple->fence_lock, flags);
	return 0;
}

const struct pvr_power_sequence_ops pvr_power_sequence_ops_apple = {
	.init = pvr_apple_power_init,
	.power_on = pvr_apple_power_on,
	.power_off = pvr_apple_power_off,
};

static void wr32(u8 *base, u32 offset, u32 value)
{
	*(__le32 *)(base + offset) = cpu_to_le32(value);
}

static int pvr_apple_gpu_buf_init_vm(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx,
				     struct pvr_apple_gpu_page *page, u64 device_addr, size_t size)
{
	struct pvr_device *pvr_dev = apple->pvr_dev;
	int err;

	page->obj = pvr_gem_object_create(pvr_dev, size, PVR_BO_FW_FLAGS_DEVICE_UNCACHED);
	if (IS_ERR(page->obj))
		return PTR_ERR(page->obj);
	page->map = pvr_gem_object_vmap(page->obj);
	if (IS_ERR(page->map)) {
		err = PTR_ERR(page->map);
		goto err_put;
	}
	memset(page->map, 0, size);
	err = pvr_vm_map(vm_ctx, page->obj, 0, device_addr, size);
	if (err)
		goto err_vunmap;
	page->device_addr = device_addr;
	page->size = size;
	return 0;

err_vunmap:
	pvr_gem_object_vunmap(page->obj);
err_put:
	pvr_gem_object_put(page->obj);
	page->obj = NULL;
	return err;
}

static int pvr_apple_gpu_page_init_vm(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx,
				      struct pvr_apple_gpu_page *page, u64 device_addr)
{
	return pvr_apple_gpu_buf_init_vm(apple, vm_ctx, page, device_addr, PVR_DEVICE_PAGE_SIZE);
}

static int pvr_apple_gpu_page_init(struct pvr_apple *apple, struct pvr_apple_gpu_page *page,
				   u64 device_addr)
{
	return pvr_apple_gpu_page_init_vm(apple, apple->pvr_dev->kernel_vm_ctx, page, device_addr);
}

static void pvr_apple_gpu_page_fini_vm(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx,
				       struct pvr_apple_gpu_page *page)
{
	if (!page->obj)
		return;
	pvr_vm_unmap_obj(vm_ctx, page->obj, page->device_addr, page->size);
	pvr_gem_object_vunmap(page->obj);
	pvr_gem_object_put(page->obj);
	page->obj = NULL;
}

static void pvr_apple_gpu_page_fini(struct pvr_apple *apple, struct pvr_apple_gpu_page *page)
{
	pvr_apple_gpu_page_fini_vm(apple, apple->pvr_dev->kernel_vm_ctx, page);
}

static void pvr_apple_gpu_pages_fini(struct pvr_apple *apple)
{
	if (apple->zero_obj) {
		pvr_gem_object_put(apple->zero_obj);
		apple->zero_obj = NULL;
	}
	pvr_apple_gpu_page_fini(apple, &apple->usc);
	pvr_apple_gpu_page_fini(apple, &apple->pds);
	pvr_apple_gpu_page_fini(apple, &apple->data);
}

static int pvr_apple_gpu_pages_init(struct pvr_apple *apple)
{
	int err;

	err = pvr_apple_gpu_page_init(apple, &apple->data, PVR_APPLE_GPU_DATA);
	if (!err)
		err = pvr_apple_gpu_page_init(apple, &apple->pds, APPLE_GPU_PDS);
	if (!err)
		err = pvr_apple_gpu_page_init(apple, &apple->usc, APPLE_GPU_USC);
	if (!err) {
		apple->zero_obj = pvr_gem_object_create(apple->pvr_dev, PVR_DEVICE_PAGE_SIZE,
							PVR_BO_FW_FLAGS_DEVICE_UNCACHED);
		if (IS_ERR(apple->zero_obj)) {
			err = PTR_ERR(apple->zero_obj);
			apple->zero_obj = NULL;
		}
	}
	if (err) {
		pvr_apple_gpu_pages_fini(apple);
		return err;
	}

	/* Terminate-only stream; HALT for the terminate and context PDS programs */
	wr32(apple->data.map, 0x0000, 0x80000000);
	wr32(apple->data.map, 0x1000, 0xd3000000);
	wr32(apple->pds.map, 0x0000, 0xd3000000);
	return 0;
}

static void pvr_apple_switch_vm(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx)
{
	struct pvr_device *pvr_dev = apple->pvr_dev;
	dma_addr_t root = pvr_vm_get_page_table_root_addr(vm_ctx);
	__le32 *gen;

	lockdep_assert_held(&apple->submit_lock);
	if (root == apple->vm_root)
		return;
	*(__le64 *)(apple->heap + apple->layout.mmu_root) = cpu_to_le64(root);
	pvr_cr_write32(pvr_dev, ROGUE_CR_MMU_CBASE_MAPPING_CONTEXT, 0);
	pvr_cr_write32(pvr_dev, ROGUE_CR_MMU_CBASE_MAPPING, (u32)(root >> 12));
	gen = (__le32 *)(apple->heap + apple->layout.mmu_gen);
	*gen = cpu_to_le32(le32_to_cpu(*gen) + 0x400000);	/* full invalidation */
	wmb();
	apple->vm_root = root;
}

static u32 pvr_apple_completed_seq(struct pvr_apple *apple)
{
	return heap_read32(apple, apple->layout.stamp[APPLE_ENGINE_COMPUTE]);
}

static int pvr_apple_wait(struct pvr_apple *apple, u32 seq, unsigned int timeout_us)
{
	u32 done;

	return read_poll_timeout(pvr_apple_completed_seq, done, (s32)(done - seq) >= 0, 1000,
				 timeout_us, false, apple);
}

static u32 pvr_apple_engine_done(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	if (apple->chip->engine_done)
		return apple->chip->engine_done(apple, engine);
	return heap_read32(apple, apple->layout.stamp[engine]);
}

/* For a wait: what the firmware posted meanwhile is read first */
static u32 pvr_apple_engine_poll(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	atomic64_inc(&apple->wait_polls);
	if (!READ_ONCE(apple->stopping) && apple->chip->events)
		apple->chip->events(apple);
	return pvr_apple_engine_done(apple, engine);
}

static u32 pvr_apple_engine_last(struct pvr_apple *apple, enum pvr_apple_engine engine)
{
	switch (engine) {
	case APPLE_ENGINE_GEOM:
		return apple->ta_seq;
	case APPLE_ENGINE_FRAG:
		return apple->frag_seq;
	default:
		return apple->last_seq;
	}
}

static void pvr_apple_stat_show(struct seq_file *s, const char *name,
				const struct pvr_apple_stat *stat)
{
	seq_printf(s, "%s last %llu max %llu total %llu\n", name, stat->last, stat->max,
		   stat->total);
}

static int pvr_apple_jobs_show(struct seq_file *s, void *unused)
{
	struct pvr_apple *apple = s->private;
	struct device *dev = apple_dev(apple);
	static const char *const names[APPLE_ENGINES] = {
		[APPLE_ENGINE_GEOM] = "geometry",
		[APPLE_ENGINE_FRAG] = "fragment",
		[APPLE_ENGINE_COMPUTE] = "compute",
	};
	struct pvr_apple_stat gate_wait, boot, resume, first_job, suspend;
	u32 submitted[APPLE_ENGINES], done[APPLE_ENGINES];
	unsigned int pending[APPLE_ENGINES] = {};
	u64 resumes, resume_failed, suspends, poll_runs;
	struct pvr_apple_fence *fence;
	u64 retired_total, busy_ns;
	bool irq_on;
	unsigned long flags;
	bool booted, fault = false;
	const char *runtime_status;
	int engine;

	mutex_lock(&apple->submit_lock);
	booted = apple->booted;
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		submitted[engine] = pvr_apple_engine_last(apple, engine);
		done[engine] = pvr_apple_engine_done(apple, engine);
	}
	if (apple->chip->fault)
		fault = apple->chip->fault(apple);
	spin_lock_irqsave(&apple->fence_lock, flags);
	for (engine = 0; engine < APPLE_ENGINES; engine++)
		list_for_each_entry(fence, &apple->pending[engine], node)
			pending[engine]++;
	retired_total = apple->retired_total;
	busy_ns = apple->busy_ns;
	gate_wait = apple->rpm_gate_wait;
	boot = apple->rpm_boot;
	resume = apple->rpm_resume;
	first_job = apple->rpm_first_job;
	suspend = apple->rpm_suspend;
	resumes = apple->rpm_resumes;
	resume_failed = apple->rpm_resume_failed;
	suspends = apple->rpm_suspends;
	poll_runs = apple->poll_runs;
	irq_on = apple->fw_irq_on;
	spin_unlock_irqrestore(&apple->fence_lock, flags);
	mutex_unlock(&apple->submit_lock);

	switch (READ_ONCE(dev->power.runtime_status)) {
	case RPM_ACTIVE:
		runtime_status = "active";
		break;
	case RPM_RESUMING:
		runtime_status = "resuming";
		break;
	case RPM_SUSPENDED:
		runtime_status = "suspended";
		break;
	default:
		runtime_status = "suspending";
		break;
	}
	seq_printf(s, "booted %u\n", booted);
	for (engine = 0; engine < APPLE_ENGINES; engine++)
		seq_printf(s, "%s submitted %u done %u pending %u\n", names[engine],
			   submitted[engine], done[engine], pending[engine]);
	seq_printf(s, "retired_total %llu\nbusy_ns %llu\n", retired_total, busy_ns);
	if (apple->chip->fault)
		seq_printf(s, "fault %u\n", fault);
	seq_printf(s, "runtime_pm_status %s\n", runtime_status);
	seq_printf(s, "fw_irq %s irqs %lld\n", !apple->fw_irq ? "none" : irq_on ? "on" : "off",
		   atomic64_read(&apple->fw_irqs));
	seq_printf(s, "poll_runs %llu wait_polls %lld\n", poll_runs,
		   atomic64_read(&apple->wait_polls));
	seq_printf(s, "rpm_resumes %llu failed %llu\n", resumes, resume_failed);
	pvr_apple_stat_show(s, "rpm_gate_wait_us", &gate_wait);
	pvr_apple_stat_show(s, "rpm_boot_us", &boot);
	pvr_apple_stat_show(s, "rpm_resume_us", &resume);
	pvr_apple_stat_show(s, "rpm_first_job_us", &first_job);
	seq_printf(s, "rpm_suspends %llu\n", suspends);
	pvr_apple_stat_show(s, "rpm_suspend_us", &suspend);
	if (apple->chip->debugfs_show)
		apple->chip->debugfs_show(apple, s);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pvr_apple_jobs);

void pvr_apple_debugfs_init(struct pvr_device *pvr_dev, struct dentry *root)
{
	if (pvr_dev->apple)
		debugfs_create_file("apple_jobs", 0400, root, pvr_dev->apple,
				    &pvr_apple_jobs_fops);
}

static int pvr_apple_wait_engine(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq,
				 unsigned int timeout_us)
{
	u32 done;

	/*
	 * 20 us steps: this runs before every render job is written and, at a
	 * switch of GPU VM, for every engine; 1 ms steps cost a scene-heavy
	 * WebGL page several milliseconds per frame
	 */
	return read_poll_timeout(pvr_apple_engine_poll, done, (s32)(done - seq) >= 0, 20,
				 timeout_us, false, apple, engine);
}

/* Wait until every engine retired its submitted jobs; submit_lock held */
int pvr_apple_wait_idle(struct pvr_apple *apple, unsigned int timeout_us)
{
	int engine, err = 0;

	lockdep_assert_held(&apple->submit_lock);
	for (engine = 0; engine < APPLE_ENGINES && !err; engine++)
		err = pvr_apple_wait_engine(apple, engine, pvr_apple_engine_last(apple, engine),
					    timeout_us);
	return err;
}

/*
 * Queue a compute job: CDM control stream at @stream, context-state buffer at
 * @ctx_state (GPU virtual addresses in @vm_ctx). Returns its sequence number,
 * which the firmware writes to the completion stamp when the job retires, or
 * a negative error. May sleep: for a free job slot, and before switching GPU
 * context 0 to another VM until that VM's jobs have retired.
 */
static int pvr_apple_submit_compute(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx,
				    u64 stream, u64 ctx_state)
{
	u32 seq;
	int err;

	mutex_lock(&apple->submit_lock);
	err = pvr_apple_wait(apple, apple->last_seq - (apple->chip->compute_slots - 1), 5000000);
	if (!err && pvr_vm_get_page_table_root_addr(vm_ctx) != apple->vm_root)
		err = pvr_apple_wait_idle(apple, 5000000);
	if (err) {
		dev_err(apple_dev(apple), "GPU did not retire job %u\n",
			pvr_apple_completed_seq(apple) + 1);
		goto out_unlock;
	}
	pvr_apple_switch_vm(apple, vm_ctx);
	seq = ++apple->last_seq;
	apple->chip->write_compute(apple, seq, stream, ctx_state);
	wmb();
	/* ring indices wrap at the ring size, sequence numbers do not */
	WRITE_ONCE(*(__le32 *)(apple->heap + apple->layout.producer[APPLE_ENGINE_COMPUTE]),
		   cpu_to_le32(seq % APPLE_RING_ENTRIES));
	wmb();
	err = apple->chip->kick(apple);
	if (err)
		dev_err(apple_dev(apple), "doorbell for job %u failed: %d\n", seq, err);
	else
		err = seq;
out_unlock:
	mutex_unlock(&apple->submit_lock);
	return err;
}

static const char *pvr_apple_fence_driver_name(struct dma_fence *f)
{
	return PVR_DRIVER_NAME;
}

static const char *pvr_apple_fence_timeline_name(struct dma_fence *f)
{
	return "apple-gfx";
}

static const struct dma_fence_ops pvr_apple_fence_ops = {
	.get_driver_name = pvr_apple_fence_driver_name,
	.get_timeline_name = pvr_apple_fence_timeline_name,
};

bool pvr_apple_fence_faulted(struct dma_fence *base)
{
	struct pvr_apple_fence *fence;

	if (!base || base->ops != &pvr_apple_fence_ops)
		return false;
	fence = container_of(base, struct pvr_apple_fence, base);
	return READ_ONCE(fence->firmware_fault);
}

/* Bind only hardware fences returned by a successful Apple submission. */
void pvr_apple_fence_bind(struct pvr_device *pvr_dev, struct dma_fence *base,
			  struct drm_gpu_scheduler *sched)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct pvr_apple_fence *fence;
	unsigned long flags;

	if (!base || base->ops != &pvr_apple_fence_ops)
		return;
	fence = container_of(base, struct pvr_apple_fence, base);
	spin_lock_irqsave(&apple->fence_lock, flags);
	fence->sched = sched;
	fence->submitted = ktime_get();
	spin_unlock_irqrestore(&apple->fence_lock, flags);
}

/* MMU status fields and requestor names from the G5P fault decoder. */
struct pvr_apple_mmu_fault {
	bool valid, read;
	u8 type, level, requester, sideband, context;
	u64 va;
};

static struct pvr_apple_mmu_fault pvr_apple_mmu_fault_decode(u64 s)
{
	struct pvr_apple_mmu_fault f = {
		.valid = s & 1,
		.type = (s >> 1) & 3,
		.read = (s >> 3) & 1,
		.level = (s >> 4) & 3,
		.requester = (s >> 6) & 0x3f,
		.sideband = (s >> 12) & 0xff,
		.context = (s >> 20) & 0xff,
		.va = (s >> 24) & 0xfffffffff0ULL,
	};

	return f;
}

static const char *const pvr_apple_mmu_fault_types[4] = {
	"page-invalid",
	"type1",
	"read-only",
	"pm-meta-protected",
};

/* Requester names, iOS kext table 6ff5ed8 (16-byte {name, class} entries; read from the j71t
 * 16.7.10 kernelcache). A larger index prints its number only.
 */
static const char *const pvr_apple_mmu_requesters[] = {
	"META",
	"TLA",
	"HOST",
	"VDMM",
	"CDM",
	"IPP",
	"PM",
	"TILING",
	"MCU_TEXAS0",
	"PDS_TEXAS0",
	"PBE0_TEXAS0",
	"PBE1_TEXAS0",
	"VDMS_BLACKPEARL0",
	"IPF_BLACKPEARL0",
	"ISP_BLACKPEARL0",
	"TPF_BLACKPEARL0",
	"USCS_BLACKPEARL0",
	"PPP_BLACKPEARL0",
	"VCE_BLACKPEARL0",
	"FBCDC_BLACKPEARL0",
	"MCU_TEXAS1",
	"PDS_TEXAS1",
	"PBE0_TEXAS1",
	"PBE1_TEXAS1",
	"MCU_TEXAS2",
	"PDS_TEXAS2",
	"PBE0_TEXAS2",
	"PBE1_TEXAS2",
	"VDMS_BLACKPEARL1",
	"IPF_BLACKPEARL1",
	"ISP_BLACKPEARL1",
	"TPF_BLACKPEARL1",
	"USCS_BLACKPEARL1",
	"PPP_BLACKPEARL1",
	"VCE_BLACKPEARL1",
	"FBCDC_BLACKPEARL1",
	"MCU_TEXAS3",
	"PDS_TEXAS3",
	"PBE0_TEXAS3",
	"PBE1_TEXAS3",
	"MCU_TEXAS4",
	"PDS_TEXAS4",
	"PBE0_TEXAS4",
	"PBE1_TEXAS4",
	"VDMS_BLACKPEARL2",
	"IPF_BLACKPEARL2",
	"ISP_BLACKPEARL2",
	"TPF_BLACKPEARL2",
	"USCS_BLACKPEARL2",
	"PPP_BLACKPEARL2",
	"VCE_BLACKPEARL2",
	"FBCDC_BLACKPEARL2",
	"MCU_TEXAS5",
	"PDS_TEXAS5",
	"PBE0_TEXAS5",
	"PBE1_TEXAS5",
};

static const char *pvr_apple_mmu_requester(u32 requester)
{
	if (requester >= ARRAY_SIZE(pvr_apple_mmu_requesters))
		return "?";
	return pvr_apple_mmu_requesters[requester];
}

/* The firmware stopped on a fault: iOS reads these banks before resetting it. */
void pvr_apple_report_mmu_fault(struct pvr_apple *apple)
{
	u32 ctrl, power, state, highest;
	const char *reason;
	unsigned int bank;

	if (xchg(&apple->fault_reported, true))
		return;
	if (!apple->chip->max_power_state) {
		dev_warn(apple_dev(apple), "GPU MMU fault licence refused: no licence for this chip\n");
		return;
	}
	ctrl = readl(apple->fender + FENDER_CTRL);
	power = readl(apple->fender + FENDER_POWER_STATE);
	state = power & 0xf;
	highest = apple->chip->max_power_state(apple);
	if (ctrl & BIT(29))
		reason = "power-down in progress";
	else if (power & FENDER_POWER_REQUEST)
		reason = "power request in flight";
	else if (!state || state > highest)
		reason = "state outside epoch range";
	else
		reason = NULL;
	if (reason) {
		dev_warn(apple_dev(apple),
			 "GPU MMU fault licence refused: %s (ctrl=%#x power=%#x max=%u)\n",
			 reason, ctrl, power, highest);
		return;
	}

	for (bank = 0; bank < 2; bank++) {
		u64 status = pvr_cr_read64(apple->pvr_dev, 0xe150 + bank * 8);
		struct pvr_apple_mmu_fault f = pvr_apple_mmu_fault_decode(status);

		dev_warn(apple_dev(apple),
			 "GPU MMU fault bank=%u va=0x%llx %s type=%s level=%u requestor=%u/%s context=%u valid=%u sideband=%u\n",
			 bank, f.va, f.read ? "read" : "write", pvr_apple_mmu_fault_types[f.type],
			 f.level, f.requester, pvr_apple_mmu_requester(f.requester), f.context,
			 f.valid, f.sideband);
	}
}

/* Called once when a chip latches a fault, after capturing its MMU state. */
void pvr_apple_firmware_fault(struct pvr_apple *apple)
{
	atomic_set(&apple->fault_pending, 1);
	pvr_apple_firmware_event(apple);
}

/* A reset cancels every engine: fault only the oldest bound scheduler. */
static void pvr_apple_fault_pending(struct pvr_apple *apple)
{
	static const char *const names[APPLE_ENGINES] = {
		[APPLE_ENGINE_GEOM] = "geometry",
		[APPLE_ENGINE_FRAG] = "fragment",
		[APPLE_ENGINE_COMPUTE] = "compute",
	};
	struct pvr_apple_fence *fence, *oldest = NULL;
	struct drm_gpu_scheduler *sched = NULL;
	unsigned long flags;
	u64 seqno = 0;
	int engine, selected = 0;

	spin_lock_irqsave(&apple->fence_lock, flags);
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		fence = list_first_entry_or_null(&apple->pending[engine],
					 struct pvr_apple_fence, node);
		if (!fence || !fence->sched)
			continue;
		if (!oldest || ktime_before(fence->submitted, oldest->submitted)) {
			oldest = fence;
			selected = engine;
		}
	}
	if (oldest) {
		WRITE_ONCE(oldest->firmware_fault, true);
		sched = oldest->sched;
		seqno = oldest->base.seqno;
	}
	spin_unlock_irqrestore(&apple->fence_lock, flags);
	if (sched) {
		dev_warn(apple_dev(apple), "firmware fault: resetting via %s job seqno=%llu\n",
			 names[selected], seqno);
		drm_sched_fault(sched);
	}
}

/*
 * Consume the firmware's events. Completion is taken from the stamps, so the
 * events are only acknowledged: a full event ring stalls the firmware.
 */
static void pvr_apple_ack_events(struct pvr_apple *apple)
{
	unsigned long flags;
	u32 cons, prod;

	spin_lock_irqsave(&apple->event_lock, flags);
	cons = heap_read32(apple, apple->layout.event_consumer);
	prod = heap_read32(apple, apple->layout.event_producer);
	for (; cons != prod; cons = (cons + 1) % APPLE_RING_ENTRIES) {
		const u32 *e = apple->heap + apple->layout.events +
			       apple->layout.event_size * (cons % APPLE_RING_ENTRIES);

		/* type 1, 4: a fragment job finished (1, 2: a geometry job) */
		if (e[0] == 1 && e[1] == 4)
			apple->frag_events++;
		/*
		 * Others are not handled yet; iOS 10's AGXFirmware::
		 * drainFirmwareEventRing() answers type 8 (channel, value) with a
		 * device-control command {10, 2, value}, and type 4 can request a
		 * restart. Log them, so a job that waits on one is visible.
		 */
		else if (e[0] != 1)
			dev_warn_ratelimited(apple_dev(apple),
					     "firmware event %u: %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
					     cons, e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8]);
	}
	WRITE_ONCE(*(__le32 *)(apple->heap + apple->layout.event_consumer),
		   cpu_to_le32(prod));
	spin_unlock_irqrestore(&apple->event_lock, flags);
}

/*
 * The last fragment job whose retirement, free-list unregister included, is
 * done: the retirement stamp every fragment command names (+0x89c), which the
 * firmware writes before the type-1 event. One event may announce several
 * retirements, so the stamp is read as a watermark, never counted per event.
 * Its low byte is the job's sequence number; fewer than 256 are outstanding.
 */
static u32 pvr_apple_frag_retired_seq(struct pvr_apple *apple)
{
	u32 seq = READ_ONCE(apple->frag_seq), mark;

	rmb();	/* after the events that announced it */
	mark = heap_read32(apple, apple->layout.frag_retired);
	return seq - (u8)(seq - mark);
}

/* The retirement watermark, taking new events off the ring first */
static u32 pvr_apple_frag_events(struct pvr_apple *apple)
{
	pvr_apple_ack_events(apple);
	return pvr_apple_frag_retired_seq(apple);
}

/* Drop the runtime-PM references of @n jobs that left the pending lists. */
static void pvr_apple_jobs_retired(struct pvr_apple *apple, unsigned int n)
{
	struct device *dev = apple_dev(apple);
	unsigned long flags;

	if (!n)
		return;
	spin_lock_irqsave(&apple->fence_lock, flags);
	apple->retired_total += n;
	spin_unlock_irqrestore(&apple->fence_lock, flags);
	pm_runtime_mark_last_busy(dev);
	while (n--)
		pm_runtime_put_autosuspend(dev);
}

/**
 * pvr_apple_firmware_event() - The firmware posted something: look now
 * @apple: Apple firmware state.
 *
 * Any context, hard IRQ included.
 */
void pvr_apple_firmware_event(struct pvr_apple *apple)
{
	/* After a terminal shutdown nothing may run the poll work again */
	if (!READ_ONCE(apple->stopping))
		mod_delayed_work(system_highpri_wq, &apple->poll_work, 0);
}

/**
 * pvr_apple_firmware_progress() - The firmware announced a finished job
 * @apple: Apple firmware state.
 *
 * For a chip's events(): check again soon, until the job is done in memory.
 */
void pvr_apple_firmware_progress(struct pvr_apple *apple)
{
	WRITE_ONCE(apple->fast_until, ktime_add_us(ktime_get(), PVR_APPLE_FAST_US));
}

static irqreturn_t pvr_apple_firmware_irq(int irq, void *data)
{
	struct pvr_apple *apple = data;

	atomic64_inc(&apple->fw_irqs);
	/* At its source first, as Apple's kext does before it reads the rings */
	apple->chip->irq_ack(apple);
	pvr_apple_firmware_event(apple);
	return IRQ_HANDLED;
}

/**
 * pvr_apple_firmware_irq_init() - Take the firmware's interrupt
 * @apple: Apple firmware state.
 * @irq: Its Linux interrupt number.
 *
 * For a chip's init(), with &pvr_apple_chip.irq_ack set. The interrupt is
 * enabled only while jobs are pending: an idle firmware raises it too.
 *
 * Return: 0 or a negative error; on error the jobs are polled for.
 */
int pvr_apple_firmware_irq_init(struct pvr_apple *apple, int irq)
{
	struct device *dev = apple_dev(apple);
	int err;

	if (!apple->chip->irq_ack || !apple->chip->irq_poll_us)
		return -EINVAL;
	err = devm_request_irq(dev, irq, pvr_apple_firmware_irq, IRQF_NO_AUTOEN,
			       "apple-gfx-fw", apple);
	if (err)
		return err;
	apple->fw_irq = irq;
	return 0;
}

/* Enable the firmware's interrupt while jobs are pending; fence_lock held */
static void pvr_apple_fw_irq_set_locked(struct pvr_apple *apple, bool on)
{
	lockdep_assert_held(&apple->fence_lock);
	if (!apple->fw_irq || apple->fw_irq_on == on)
		return;
	apple->fw_irq_on = on;
	if (on)
		enable_irq(apple->fw_irq);
	else
		disable_irq_nosync(apple->fw_irq);
}

/* Signal the fences of retired jobs; check again while any are pending. */
static void pvr_apple_poll_work(struct work_struct *work)
{
	struct pvr_apple *apple = container_of(work, struct pvr_apple, poll_work.work);
	struct pvr_apple_fence *fence, *tmp;
	unsigned int retired = 0, us;
	bool pending = false, irq_on;
	int engine;

	if (READ_ONCE(apple->stopping))
		return;
	if (apple->chip->events)
		apple->chip->events(apple);
	else
		pvr_apple_ack_events(apple);
	if (atomic_xchg(&apple->fault_pending, 0))
		pvr_apple_fault_pending(apple);
	spin_lock_irq(&apple->fence_lock);
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		u32 done = pvr_apple_engine_done(apple, engine);

		/*
		 * A fragment job's stamp comes before the firmware retires it, so
		 * its target may not be complete in memory yet for readers that
		 * do not go through the GPU, such as the display. Its fence also
		 * waits for the retirement stamp.
		 */
		if (engine == APPLE_ENGINE_FRAG && !apple->chip->engine_done) {
			u32 retired = pvr_apple_frag_retired_seq(apple);

			if ((s32)(retired - done) < 0)
				done = retired;
		}
		list_for_each_entry_safe(fence, tmp, &apple->pending[engine], node) {
			if ((s32)(done - (u32)fence->base.seqno) < 0)
				break;
			list_del_init(&fence->node);
			dma_fence_signal_locked(&fence->base);
			dma_fence_put(&fence->base);
			retired++;
		}
		pending |= !list_empty(&apple->pending[engine]);
	}
	apple->poll_runs++;
	if (retired && apple->first_job_from) {
		pvr_apple_stat_add(&apple->rpm_first_job,
				   ktime_us_delta(ktime_get(), apple->first_job_from));
		apple->first_job_from = 0;
	}
	if (!pending && apple->busy_since) {
		apple->busy_ns += ktime_to_ns(ktime_sub(ktime_get(), apple->busy_since));
		apple->busy_since = 0;
	}
	pvr_apple_fw_irq_set_locked(apple, pending);
	irq_on = apple->fw_irq_on;
	spin_unlock_irq(&apple->fence_lock);
	pvr_apple_jobs_retired(apple, retired);

	if (!pending || READ_ONCE(apple->stopping))
		return;
	/*
	 * Without the firmware's interrupt (the A10's firmware sends no
	 * completion message in this setup) poll. With it, check again
	 * soon only after an announced completion; otherwise the slower
	 * period backs the interrupt up.
	 */
	us = READ_ONCE(pvr_apple_poll_us);
	if (irq_on && !ktime_before(ktime_get(), READ_ONCE(apple->fast_until)))
		us = apple->chip->irq_poll_us;
	hrtimer_start(&apple->poll_timer, us_to_ktime(max(us, 10U)), HRTIMER_MODE_REL);
}

static enum hrtimer_restart pvr_apple_poll_timer(struct hrtimer *timer)
{
	struct pvr_apple *apple = container_of(timer, struct pvr_apple, poll_timer);

	if (!READ_ONCE(apple->stopping))
		mod_delayed_work(system_highpri_wq, &apple->poll_work, 0);
	return HRTIMER_NORESTART;
}

/**
 * pvr_apple_report_hang() - Log the firmware's job state for a timed-out job
 * @pvr_dev: Target device.
 * @fence: The job's hardware fence (its context names the engine).
 *
 * Tells a job the GPU never finished (its engine's stamp below the job's
 * sequence number) from one whose completion was missed (stamp at or past
 * it), with the submission counters and ring indices to compare.
 */
void pvr_apple_report_hang(struct pvr_device *pvr_dev, struct dma_fence *fence)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	static const char *const names[] = { "compute", "geometry", "fragment" };
	unsigned int engine = fence ? (unsigned int)(fence->context - apple->fence_context) : 3;

	if (!fence || engine >= APPLE_ENGINES) {
		dev_err(apple_dev(apple), "hang: job without an Apple fence\n");
		return;
	}
	dev_err(apple_dev(apple),
		"hang: %s job %llu%s; stamps geometry %u fragment %u compute %u; submitted geometry %u fragment %u compute %u; fragment retired %#x; rings ta %#x 3d %#x; events %#x/%#x\n",
		engine < ARRAY_SIZE(names) ? names[engine] : "?", fence->seqno,
		dma_fence_is_signaled(fence) ? " (signalled)" : "",
		pvr_apple_engine_done(apple, APPLE_ENGINE_GEOM),
		pvr_apple_engine_done(apple, APPLE_ENGINE_FRAG), pvr_apple_completed_seq(apple),
		apple->ta_seq, apple->frag_seq, apple->last_seq, pvr_apple_frag_retired(apple),
		heap_read32(apple, apple->layout.producer[APPLE_ENGINE_GEOM]),
		heap_read32(apple, apple->layout.producer[APPLE_ENGINE_FRAG]),
		heap_read32(apple, apple->layout.event_producer),
		heap_read32(apple, apple->layout.event_consumer));
	if (apple->chip->report_hang)
		apple->chip->report_hang(apple);
}

/* No job is ordered after the scenes submitted so far any more */
static void pvr_apple_forget_scenes(struct pvr_apple *apple)
{
	dma_fence_put(apple->last_frag);
	apple->last_frag = NULL;
	dma_fence_put(apple->prev_frag);
	apple->prev_frag = NULL;
	dma_fence_put(apple->last_geom);
	apple->last_geom = NULL;
}

static unsigned int pvr_apple_cancel_fences(struct pvr_apple *apple, int error)
{
	struct pvr_apple_fence *fence, *tmp;
	unsigned int cancelled = 0;
	int engine;

	if (READ_ONCE(apple->stopping)) {
		/* Terminal: retain storage and never wait for blocked worker activity. */
		hrtimer_try_to_cancel(&apple->poll_timer);
		cancel_delayed_work(&apple->poll_work);
	} else {
		/* Each rearms the other. Normal teardown joins both pollers. */
		hrtimer_cancel(&apple->poll_timer);
		cancel_delayed_work_sync(&apple->poll_work);
		hrtimer_cancel(&apple->poll_timer);
	}
	spin_lock_irq(&apple->fence_lock);
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		list_for_each_entry_safe(fence, tmp, &apple->pending[engine], node) {
			list_del_init(&fence->node);
			dma_fence_set_error(&fence->base, error);
			dma_fence_signal_locked(&fence->base);
			dma_fence_put(&fence->base);
			cancelled++;
		}
	}
	pvr_apple_fw_irq_set_locked(apple, false);
	spin_unlock_irq(&apple->fence_lock);
	pvr_apple_jobs_retired(apple, cancelled);
	return cancelled;
}

/**
 * pvr_apple_submit_compute_job() - Queue a compute job on the GFX firmware
 * @pvr_dev: Target device.
 * @vm_ctx: GPU VM the job runs in.
 * @stream: GPU address of the CDM control stream.
 * @ctx_state: GPU address of the CDM context-state buffer.
 *
 * Return: a fence signalled when the job retires, or an ERR_PTR().
 */
/* The fence of job @seq on @engine, which the poll work signals; @seq < 0 is an error. */
static struct dma_fence *pvr_apple_job_fence(struct pvr_apple *apple, struct pvr_apple_fence *fence,
					     enum pvr_apple_engine engine, int seq)
{
	if (seq < 0) {
		kfree(fence);
		return ERR_PTR(seq);
	}
	dma_fence_init(&fence->base, &pvr_apple_fence_ops, &apple->fence_lock,
		       apple->fence_context + engine, (u32)seq);
	/* One reference for the pending list, one for the caller */
	dma_fence_get(&fence->base);
	spin_lock_irq(&apple->fence_lock);
	if (!apple->busy_since)
		apple->busy_since = ktime_get();
	list_add_tail(&fence->node, &apple->pending[engine]);
	pvr_apple_fw_irq_set_locked(apple, true);
	spin_unlock_irq(&apple->fence_lock);
	pvr_apple_firmware_event(apple);
	return &fence->base;
}

struct dma_fence *pvr_apple_submit_compute_job(struct pvr_device *pvr_dev,
					       struct pvr_vm_context *vm_ctx, u64 stream,
					       u64 ctx_state)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct pvr_apple_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);
	return pvr_apple_job_fence(apple, fence, APPLE_ENGINE_COMPUTE,
				   pvr_apple_submit_compute(apple, vm_ctx, stream, ctx_state));
}

/**
 * pvr_apple_wait_file_jobs() - Retire submitted jobs before a file's objects go
 * @pvr_dev: Target device.
 *
 * A client that exits with a render in flight closes its file while the
 * firmware still runs the render's fragment job. File close unmaps the
 * file's VM (pvr_destroy_contexts_for_file()); doing that under the job
 * left it unfinished and the firmware stuck, no later job ran (a client
 * exiting right after its submit shows it). Called once the file's contexts are destroyed, so no new
 * job of the file can start.
 */
void pvr_apple_wait_file_jobs(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	mutex_lock(&apple->submit_lock);
	err = pvr_apple_wait_idle(apple, 5000000);
	mutex_unlock(&apple->submit_lock);
	if (err)
		dev_err(apple_dev(apple), "GPU did not retire its jobs before a file closed\n");
}

/**
 * pvr_apple_quiesce() - Retire every submitted job before the device goes away
 * @pvr_dev: Target device.
 *
 * Userspace may exit before its last job's fence is signalled, leaving the
 * job to be freed later from the scheduler. Wait for the GPU, signal the
 * fences now, and run the scheduler's free_job work and the context releases
 * it queues, so that no job outlives pvr_remove().
 */
void pvr_apple_quiesce(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	mutex_lock(&apple->submit_lock);
	err = apple->booted ? pvr_apple_wait_idle(apple, 5000000) : 0;
	mutex_unlock(&apple->submit_lock);

	if (err) {
		dev_err(apple_dev(apple), "GPU did not retire its jobs before removal\n");
		pvr_apple_cancel_fences(apple, -ETIMEDOUT);
	} else if (apple->booted) {
		mod_delayed_work(system_highpri_wq, &apple->poll_work, 0);
		flush_delayed_work(&apple->poll_work);
	}
	/* free_job first, then the context releases it queued */
	flush_workqueue(pvr_dev->sched_wq);
	flush_workqueue(pvr_dev->sched_wq);
}

/**
 * pvr_apple_mmu_flush() - Invalidate GPU context 0's TLB before the next job
 * @pvr_dev: Target device.
 *
 * Page-table updates of the VM GPU context 0 runs on must not be hidden by
 * stale TLB entries. The firmware compares each context's generation with
 * the one it acknowledged and invalidates on a change (fw 0x18530).
 * Called from pvr_mmu_flush_exec().
 */
void pvr_apple_mmu_flush(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	__le32 *gen = (__le32 *)(apple->heap + apple->layout.mmu_gen);

	if (apple->chip->mmu_flush) {
		apple->chip->mmu_flush(apple);
		return;
	}

	/*
	 * May run in atomic context, so no lock: only the host writes the
	 * generation, and any change makes the firmware invalidate. A bump
	 * racing with pvr_apple_switch_vm() still changes the value.
	 */
	WRITE_ONCE(*gen, cpu_to_le32(le32_to_cpu(READ_ONCE(*gen)) + 0x400000));
	wmb();
}

/*
 * Switch GPU context 0 back to the kernel VM once its jobs have retired, so
 * that the firmware never walks page tables that are about to be freed.
 */
static void pvr_apple_switch_to_kernel_vm(struct pvr_apple *apple, struct pvr_vm_context *only_if)
{
	struct pvr_device *pvr_dev = apple->pvr_dev;

	mutex_lock(&apple->submit_lock);
	/* Powered off, the firmware starts again on the kernel VM. */
	if (apple->booted &&
	    (!only_if || pvr_vm_get_page_table_root_addr(only_if) == apple->vm_root)) {
		if (pvr_apple_wait_idle(apple, 5000000))
			dev_err(apple_dev(apple), "jobs did not retire before a VM switch\n");
		pvr_apple_switch_vm(apple, pvr_dev->kernel_vm_ctx);
	}
	mutex_unlock(&apple->submit_lock);
}

/**
 * pvr_apple_vm_fini() - A VM context is about to be freed
 * @pvr_dev: Target device.
 * @vm_ctx: The VM context.
 */
void pvr_apple_vm_fini(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple *apple = to_apple(pvr_dev);

	if (vm_ctx == pvr_dev->kernel_vm_ctx)
		return;
	if (apple->chip->vm_fini)
		apple->chip->vm_fini(apple, vm_ctx);
	if (!apple->booted)
		return;
	pvr_apple_switch_to_kernel_vm(apple, vm_ctx);
}

/*
 * Render jobs. The firmware takes a geometry job's parameter-manager state
 * (one slot, one free-list descriptor) and keeps the scene in the parameter
 * manager until the fragment job on the same slot has run, so scenes are
 * serialized: each geometry job waits for the previous fragment job
 * (pvr_apple_job_add_deps()). Each engine has one command area, so a job
 * also waits for the engine's previous job before it is written.
 */
/* Fragment jobs the firmware has retired: work ring 1's first index word */
static u32 pvr_apple_frag_retired(struct pvr_apple *apple)
{
	return heap_read32(apple, apple->layout.frag_ring);
}

static int pvr_apple_queue_render(struct pvr_apple *apple, struct pvr_vm_context *vm_ctx,
				  enum pvr_apple_engine engine, const void *job,
				  struct pvr_free_list *fl)
{
	u32 seq;
	int err;

	mutex_lock(&apple->submit_lock);
	err = pvr_apple_wait_engine(apple, engine, pvr_apple_engine_last(apple, engine), 5000000);
	if (!err && pvr_vm_get_page_table_root_addr(vm_ctx) != apple->vm_root)
		err = pvr_apple_wait_idle(apple, 5000000);
	if (err) {
		dev_err(apple_dev(apple), "GPU did not retire its %s job\n",
			engine == APPLE_ENGINE_GEOM ? "geometry" : "fragment");
		goto out_unlock;
	}
	/*
	 * A fragment job's stamp is written before the firmware retires it, and
	 * the retirement unregisters the free list (fw 0x13144: table[id] = 0,
	 * current ids -1). The next geometry job registers it again when it is
	 * dispatched, so it must not be dispatched before that: otherwise its
	 * Start3D finds no free list and the firmware crashes (fw 0x1703c).
	 */
	if (!err && engine == APPLE_ENGINE_GEOM) {
		u32 retired;

		err = read_poll_timeout(pvr_apple_frag_retired, retired,
					!((retired ^ apple->frag_seq) & 0xff), 20, 1000000, false,
					apple);
		if (err)
			dev_err(apple_dev(apple), "firmware did not retire fragment job %u\n",
				apple->frag_seq);
		/*
		 * The ring index advances before the retirement has unregistered
		 * the free list: a geometry job dispatched in between had its
		 * registration cleared, and the next Start3D loaded a null entry and
		 * crashed the firmware (fw 0x16634, Hyprland every few thousand
		 * renders). The retirement stamp is written after the unregister.
		 */
		if (!err) {
			u32 events;

			err = read_poll_timeout(pvr_apple_frag_events, events,
						(s32)(events - apple->frag_seq) >= 0, 20, 1000000, false,
						apple);
			if (err) {
				dev_err(apple_dev(apple),
					"fragment job %u not retired (retirement stamp %u)\n",
					apple->frag_seq, events);
				err = 0;
			}
		}
	}
	if (err)
		goto out_unlock;
	/*
	 * A fragment job without its geometry job crashes the firmware: its
	 * Start3D loads a null free-list entry (fw 0x1703c -> 0x16634). That
	 * happens when the geometry job failed, as after a reset, whose
	 * cancelled geometry fences still release the fragment jobs waiting
	 * on them. End such a job instead.
	 */
	if (engine == APPLE_ENGINE_FRAG && !apple->geom_pending) {
		dev_warn_ratelimited(apple_dev(apple),
				     "fragment job without a geometry job: not run\n");
		err = -ECANCELED;
		goto out_unlock;
	}
	pvr_apple_switch_vm(apple, vm_ctx);
	apple->geom_pending = engine == APPLE_ENGINE_GEOM;
	if (engine == APPLE_ENGINE_GEOM) {
		seq = ++apple->ta_seq;
		apple->chip->write_geom(apple, seq, job);
	} else {
		seq = ++apple->frag_seq;
		apple->chip->write_frag(apple, seq, job);
	}
	wmb();
	WRITE_ONCE(*(__le32 *)(apple->heap + apple->layout.producer[engine]),
		   cpu_to_le32(seq % APPLE_RING_ENTRIES));
	wmb();
	err = apple->chip->kick(apple);
	if (!err)
		err = seq;
out_unlock:
	mutex_unlock(&apple->submit_lock);
	return err;
}

/**
 * pvr_apple_submit_render_job() - Queue a geometry or fragment job
 * @job: The job, its command decoded with pvr_apple_cmd_{geom,frag}_stream.
 *
 * Return: a fence signalled when the job retires, or an ERR_PTR().
 */
struct dma_fence *pvr_apple_submit_render_job(struct pvr_job *job)
{
	struct pvr_apple *apple = to_apple(job->pvr_dev);
	struct pvr_apple_fence *fence;
	enum pvr_apple_engine engine;
	void *hw;
	int seq;

	if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT &&
	    job->fw_ccb_cmd_type == ROGUE_FWIF_CCB_CMD_TYPE_FRAG_PR)
		return ERR_PTR(-EOPNOTSUPP);	/* partial renders: not yet */
	if (apple->chip->submit)
		return pvr_apple_submit_job(job);

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	hw = kmalloc(apple->chip->render_job_size, GFP_KERNEL);
	if (!fence || !hw) {
		kfree(fence);
		kfree(hw);
		return ERR_PTR(-ENOMEM);
	}
	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY) {
		engine = APPLE_ENGINE_GEOM;
		apple->chip->prepare_geom(hw, job);
	} else {
		engine = APPLE_ENGINE_FRAG;
		apple->chip->prepare_frag(hw, job);
	}
	seq = pvr_apple_queue_render(apple, job->ctx->vm_ctx, engine, hw,
				     engine == APPLE_ENGINE_GEOM ?
					     job->hwrt->hwrt_dataset->free_lists[0] : NULL);
	kfree(hw);
	return pvr_apple_job_fence(apple, fence, engine, seq);
}

/**
 * pvr_apple_submit_job() - Queue a job on a chip that writes its jobs itself
 * @job: The job, its command decoded with the chip's stream definitions.
 *
 * Return: a fence signalled when the job retires, or an ERR_PTR().
 */
struct dma_fence *pvr_apple_submit_job(struct pvr_job *job)
{
	struct pvr_apple *apple = to_apple(job->pvr_dev);
	enum pvr_apple_engine engine = job->type == DRM_PVR_JOB_TYPE_GEOMETRY ? APPLE_ENGINE_GEOM :
				       job->type == DRM_PVR_JOB_TYPE_FRAGMENT ? APPLE_ENGINE_FRAG :
				       APPLE_ENGINE_COMPUTE;
	struct pvr_apple_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);
	return pvr_apple_job_fence(apple, fence, engine, apple->chip->submit(apple, job));
}

/**
 * pvr_apple_job_add_deps() - Order a job against the parameter manager's scene
 * @job: Job about to be armed.
 *
 * The firmware holds one scene at a time, built by a geometry job and
 * consumed by the next fragment job. A geometry job waits for the last
 * fragment job pushed, on any context; a fragment job waits for the last
 * geometry job. IMG's firmware orders a geometry/fragment pair itself; the
 * HWRT reservation does not, as a job's fence is added to it only after all
 * jobs of a submission collected their dependencies.
 *
 * A chip may hold two scenes of one VM (&pvr_apple_chip.scenes,
 * &pvr_apple_chip.vm_scenes): a geometry job then waits for the fragment job
 * before the last one if the last one is its own VM's, and a fragment job
 * for its geometry job to be submitted rather than finished
 * (pvr_apple_job_armed()).
 *
 * Return: 0 or a negative error.
 */
int pvr_apple_job_add_deps(struct pvr_job *job)
{
	struct pvr_apple *apple = to_apple(job->pvr_dev);
	struct dma_fence *last;
	unsigned int scenes = 1;

	if (job->type != DRM_PVR_JOB_TYPE_GEOMETRY && job->type != DRM_PVR_JOB_TYPE_FRAGMENT)
		return 0;
	spin_lock_irq(&apple->fence_lock);
	/* Take this VM's first scene even when another VM rendered last. */
	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY && apple->chip->vm_scenes)
		scenes = apple->chip->vm_scenes(apple, job->ctx->vm_ctx);
	if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		last = apple->last_geom;
	else if (apple->chip->scenes > 1 && apple->last_frag_vm == job->ctx->vm_ctx &&
		 scenes > 1)
		last = apple->prev_frag;
	else
		last = apple->last_frag;
	last = dma_fence_get(last);
	spin_unlock_irq(&apple->fence_lock);
	return last ? drm_sched_job_add_dependency(&job->base, last) : 0;
}

/**
 * pvr_apple_job_armed() - Record an armed render job for pvr_apple_job_add_deps()
 * @job: The job.
 * @finished: Its scheduler finished fence.
 */
void pvr_apple_job_armed(struct pvr_job *job, struct dma_fence *finished)
{
	struct pvr_apple *apple = to_apple(job->pvr_dev);
	bool joint = apple->chip->scenes > 1;
	struct dma_fence **last, *old;

	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY)
		last = &apple->last_geom;
	else if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		last = &apple->last_frag;
	else
		return;
	/* The firmware orders a fragment job after its geometry job itself */
	if (joint && job->type == DRM_PVR_JOB_TYPE_GEOMETRY)
		finished = &job->base.s_fence->scheduled;
	spin_lock_irq(&apple->fence_lock);
	old = *last;
	*last = dma_fence_get(finished);
	if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT) {
		apple->last_frag_vm = job->ctx->vm_ctx;
		if (joint)
			swap(old, apple->prev_frag);
	}
	spin_unlock_irq(&apple->fence_lock);
	dma_fence_put(old);
}

/**
 * pvr_apple_vm_init() - Map the driver's reserved data page into a VM
 * @pvr_dev: Target device.
 * @vm_ctx: New userspace VM context.
 *
 * Every compute job names the driver's terminate and context-store PDS
 * programs and the flush scratch address; they live in the data page at
 * %PVR_APPLE_GPU_DATA, outside the userspace heaps. Fragment jobs also write
 * (without data) to GPU VA 0 at the end of a render, which iPadOS keeps
 * mapped (its code heap starts there); back it with a zeroed page.
 */
int pvr_apple_vm_init(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	err = pvr_vm_map_reserved(vm_ctx, apple->data.obj, PVR_APPLE_GPU_DATA,
				  PVR_DEVICE_PAGE_SIZE);
	if (!err)
		err = pvr_vm_map_reserved(vm_ctx, apple->zero_obj, 0, PVR_DEVICE_PAGE_SIZE);
	if (!err && apple->chip->vm_init)
		err = apple->chip->vm_init(apple, vm_ctx);
	return err;
}

/*
 * Hand the firmware fresh init data: empty rings, GPU context 0 on the kernel
 * VM. Every stamp starts at 0 again, so do the sequence numbers. The firmware
 * has just booted.
 */
static int pvr_apple_fw_start(struct pvr_apple *apple)
{
	apple->vm_root = pvr_vm_get_page_table_root_addr(apple->pvr_dev->kernel_vm_ctx);
	apple->last_seq = 0;
	apple->ta_seq = 0;
	apple->frag_seq = 0;
	apple->frag_events = 0;
	apple->geom_pending = false;
	apple->fault_reported = false;
	return apple->chip->start(apple);
}

/*
 * Shut the firmware down and power the GPU off through state 0. A wedged or
 * crashed firmware may not answer: go on regardless.
 */
static void pvr_apple_fw_stop(struct pvr_apple *apple)
{
	lockdep_assert_held(&apple->submit_lock);
	if (!apple->booted)
		return;
	apple->chip->stop(apple, true);
	apple->booted = false;
}

/* Boot the firmware from the kept image with fresh init data, as at probe. */
static int pvr_apple_fw_restart(struct pvr_apple *apple)
{
	int err;

	lockdep_assert_held(&apple->submit_lock);
	if (READ_ONCE(apple->stopping))
		return -ENODEV;
	apple->chip->restore_image(apple);
	err = pvr_apple_boot(apple);
	if (!err)
		err = pvr_apple_fw_start(apple);
	return err;
}

int pvr_apple_fw_init(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	err = pvr_apple_gpu_pages_init(apple);
	if (err)
		return err;
	err = pvr_apple_fw_start(apple);
	if (err) {
		if (apple->chip->fini)
			apple->chip->fini(apple);
		pvr_apple_gpu_pages_fini(apple);
		return err;
	}
	pvr_apple_devfreq_start(apple);
	apple->fw_ready = true;
	return 0;
}

/**
 * pvr_apple_reset() - Restart the GFX firmware after a job hung
 * @pvr_dev: Target device.
 *
 * Every job in flight ends with -ETIMEDOUT, the hung one and those queued
 * behind it on the firmware's rings, so that nothing waits on the GPU
 * forever. The firmware is shut down, the GPU powered off through state 0,
 * and both started again from the kept firmware image with fresh init data,
 * as at probe. The driver's GPU pages stay: userspace VMs map the data page.
 *
 * Return: 0 once the GPU runs again, or a negative error; then it stays off.
 */
int pvr_apple_reset(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct device *dev = apple_dev(apple);
	int err;

	mutex_lock(&apple->submit_lock);
	dev_warn(dev, "resetting the GPU\n");
	pvr_apple_cancel_fences(apple, -ETIMEDOUT);
	pvr_apple_forget_scenes(apple);
	spin_lock_irq(&apple->fence_lock);
	apple->busy_since = 0;
	spin_unlock_irq(&apple->fence_lock);

	pvr_apple_fw_stop(apple);
	/*
	 * A chip whose core keeps state while its power domain is on: take
	 * the domain off through runtime PM and leave it off for a while. The
	 * next job starts the firmware (pvr_apple_power_on()). If another job
	 * holds the GPU powered at this moment, the domain stays on and the
	 * firmware is started here.
	 */
	if (apple->chip->gate_off_ms) {
		mutex_unlock(&apple->submit_lock);
		if (pm_runtime_suspend(dev) >= 0) {
			msleep(apple->chip->gate_off_ms);
			return 0;
		}
		mutex_lock(&apple->submit_lock);
	}
	err = READ_ONCE(apple->stopping) ? -ENODEV :
		apple->booted ? 0 : pvr_apple_fw_restart(apple);
	mutex_unlock(&apple->submit_lock);
	if (err)
		dev_err(dev, "GPU reset failed: %d\n", err);
	return err;
}

/**
 * pvr_apple_suspend() - Stop the GPU for system sleep
 * @pvr_dev: Target device.
 *
 * The firmware keeps its state only while the GPU is powered, and the GPU
 * power domain goes off in sleep. Retire every job (userspace and the job
 * scheduler are frozen by now), then shut the firmware down and power the
 * GPU off through state 0, as before a reset. Nothing the firmware held
 * needs saving: pvr_apple_resume() hands it fresh init data, and each job
 * carries the rest.
 *
 * Return: 0. A GPU that does not retire its jobs is stopped regardless and
 * those jobs end with -ETIMEDOUT.
 */
int pvr_apple_suspend(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct device *dev = apple_dev(apple);
	unsigned int n;
	int err;

	/*
	 * Runtime-suspended: nothing runs and the firmware is off, so there is
	 * nothing to stop. The PM core holds a runtime reference across sleep,
	 * so pvr_apple_resume() sees the same status and leaves it off too.
	 */
	if (pm_runtime_status_suspended(dev))
		return 0;

	pvr_apple_devfreq_suspend(apple);

	mutex_lock(&apple->submit_lock);
	err = pvr_apple_wait_idle(apple, 5000000);
	mutex_unlock(&apple->submit_lock);
	if (err)
		dev_err(dev, "GPU did not retire its jobs before sleep\n");
	/* Signal what retired, then end anything left */
	mod_delayed_work(system_highpri_wq, &apple->poll_work, 0);
	flush_delayed_work(&apple->poll_work);

	mutex_lock(&apple->submit_lock);
	n = pvr_apple_cancel_fences(apple, -ETIMEDOUT);
	if (n)
		dev_warn(dev, "%u GPU jobs ended unfinished before sleep\n", n);
	pvr_apple_forget_scenes(apple);
	spin_lock_irq(&apple->fence_lock);
	apple->busy_since = 0;
	spin_unlock_irq(&apple->fence_lock);
	pvr_apple_fw_stop(apple);
	mutex_unlock(&apple->submit_lock);

	cancel_delayed_work_sync(&apple->poll_work);
	hrtimer_cancel(&apple->poll_timer);
	return 0;
}

/**
 * pvr_apple_resume() - Start the GPU again after system sleep
 * @pvr_dev: Target device.
 *
 * Boot the firmware from the kept image with fresh init data, as at probe.
 * If the GPU was runtime-suspended before sleep, it stays off until the next
 * job powers it on.
 *
 * Return: 0 once the GPU runs again, or a negative error. The GPU then stays
 * off until a job times out and pvr_apple_reset() boots it again.
 */
int pvr_apple_resume(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	/* Runtime-suspended before sleep: the next job starts it. */
	if (pm_runtime_status_suspended(apple_dev(apple)))
		return 0;

	mutex_lock(&apple->submit_lock);
	err = pvr_apple_fw_restart(apple);
	mutex_unlock(&apple->submit_lock);
	if (err)
		dev_err(apple_dev(apple), "GPU did not start after sleep: %d\n", err);
	else
		pvr_apple_devfreq_resume(apple);
	return err;
}

/**
 * pvr_apple_pm_pages() - The parameter-manager pages every free list hands out
 * @pvr_dev: Target device.
 * @size: Bytes a free list needs.
 *
 * The firmware runs one geometry scene at a time, and the kernel serialises
 * them across all clients (pvr_apple_job_add_deps()), so the free list of
 * every Vulkan device can point at the same pages instead of pinning its own
 * (32 MiB each from Mesa: 36 MB per GPU client, 2026-11-03).
 *
 * The pages are as many as Mesa's G9P parameter buffer (32 MiB): a device's
 * smaller lists, created first, must not size them.
 *
 * Return: a new reference to the shared pages, ERR_PTR(-E2BIG) if a list
 * needs more than they hold (it then gets its own), or another ERR_PTR().
 */
#define APPLE_PM_PAGES_SIZE	SZ_32M

struct pvr_gem_object *pvr_apple_pm_pages(struct pvr_device *pvr_dev, size_t size)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct pvr_gem_object *obj;

	mutex_lock(&apple->pm_lock);
	if (!apple->pm_pages) {
		obj = pvr_gem_object_create(pvr_dev, max_t(size_t, size, APPLE_PM_PAGES_SIZE),
					    PVR_BO_FW_FLAGS_DEVICE_CACHED);
		if (IS_ERR(obj)) {
			mutex_unlock(&apple->pm_lock);
			return obj;
		}
		apple->pm_pages = obj;
	}
	obj = apple->pm_pages;
	if (pvr_gem_object_size(obj) < size)
		obj = ERR_PTR(-E2BIG);
	else
		drm_gem_object_get(gem_from_pvr_gem(obj));
	mutex_unlock(&apple->pm_lock);
	return obj;
}

/*
 * Shutdown is terminal: wait at most one second for a submitter and at most
 * one second total for all engines. Chip stop has bounded hardware/RTKit
 * polls (A9: 200 ms; A10: 100 ms plus two 1 s ACKs). Never flush the job
 * scheduler here: it may be waiting on a job that the GPU cannot finish.
 */
void pvr_apple_shutdown(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	struct device *dev = apple_dev(apple);
	unsigned long deadline = jiffies + msecs_to_jiffies(1000);
	int engine, err = 0;

	WRITE_ONCE(apple->stopping, true);
	while (!mutex_trylock(&apple->submit_lock)) {
		if (time_after_eq(jiffies, deadline)) {
			dev_err(dev, "shutdown: submitter did not stop within 1 s; continuing\n");
			pvr_apple_cancel_fences(apple, -ESHUTDOWN);
			return;
		}
		usleep_range(1000, 2000);
	}
	deadline = jiffies + msecs_to_jiffies(1000);
	if (apple->booted) {
		for (engine = 0; engine < APPLE_ENGINES && !err; engine++) {
			unsigned int remaining;

			if (time_after_eq(jiffies, deadline)) {
				err = -ETIMEDOUT;
				break;
			}
			remaining = jiffies_to_usecs(deadline - jiffies);
			err = pvr_apple_wait_engine(apple, engine,
						   pvr_apple_engine_last(apple, engine), remaining);
		}
		apple->chip->stop(apple, false);
		apple->booted = false;
		apple->chip->idle(apple);
	}
	mutex_unlock(&apple->submit_lock);
	pvr_apple_cancel_fences(apple, -ESHUTDOWN);
	if (err)
		dev_warn(dev, "shutdown: GPU jobs timed out; firmware forced off\n");
	/* No resume, including for a firmware that has never run. */
	/* device_shutdown() pins runtime PM before invoking this callback. */
	pm_runtime_put_noidle(dev);
	err = __pm_runtime_suspend(dev, RPM_ASYNC | RPM_NOWAIT);
	pm_runtime_get_noresume(dev);
	if (err < 0)
		dev_warn(dev, "shutdown: runtime suspend returned %d; continuing\n", err);
	else
		dev_info(dev, "shutdown: Apple GPU firmware stopped, runtime suspend requested\n");
}

void pvr_apple_fw_fini(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);

	/* Stop the DMA writer while every firmware page is still owned. */
	mutex_lock(&apple->submit_lock);
	pvr_apple_fw_stop(apple);
	apple->fw_ready = false;
	mutex_unlock(&apple->submit_lock);
	hrtimer_cancel(&apple->poll_timer);
	cancel_delayed_work_sync(&apple->poll_work);
	pvr_apple_forget_scenes(apple);
	pvr_apple_cancel_fences(apple, -ENODEV);
	pvr_apple_gpu_pages_fini(apple);
	if (apple->pm_pages) {
		/* free lists still holding the pages keep their own references */
		pvr_gem_object_put(apple->pm_pages);
		apple->pm_pages = NULL;
	}
	if (apple->chip->fini)
		apple->chip->fini(apple);
}

/*
 * Static device information. Apple's firmware carries no IMG header, so the
 * values come from the hardware (CORE_ID: 6 clusters) and the Rogue lineage.
 */
void pvr_apple_device_info_init(struct pvr_device *pvr_dev)
{
	pvr_dev->features.has_compute = true;
	pvr_dev->features.has_cdm_control_stream_format = true;
	pvr_dev->features.cdm_control_stream_format = 1;
	pvr_dev->features.has_num_clusters = true;
	pvr_dev->features.num_clusters = 6;
	pvr_dev->features.has_phys_bus_width = true;
	pvr_dev->features.phys_bus_width = 40;
	pvr_dev->features.has_virtual_address_space_bits = true;
	pvr_dev->features.virtual_address_space_bits = 40;	/* page tables; data paths: 38 bits */
	pvr_dev->features.has_tile_size_x = true;
	pvr_dev->features.tile_size_x = 32;
	pvr_dev->features.has_tile_size_y = true;
	pvr_dev->features.tile_size_y = 32;
	/* TE_AA as the iOS 10 kext sets it: 1, 5, 7 for 2, 4, 8 samples */
	pvr_dev->features.has_isp_samples_per_pixel = true;
	pvr_dev->features.isp_samples_per_pixel = 2;
}

/**
 * pvr_apple_takes_jobs() - Whether the chip's firmware is given jobs
 * @pvr_dev: Target device.
 *
 * Return: %false for a chip whose firmware is started and stopped, but whose
 * commands are not written yet: it gets no contexts.
 */
bool pvr_apple_takes_jobs(struct pvr_device *pvr_dev)
{
	const struct pvr_apple_chip *chip = pvr_dev->device_data->apple_chip;

	return chip->write_compute || chip->submit;
}

/**
 * pvr_apple_free_list_has_pages() - Whether a free list holds parameter-manager pages
 * @pvr_dev: Target device.
 *
 * Return: %false on a chip that allocates those pages itself, for each VM: a
 * free list then only says how many a render may take.
 */
bool pvr_apple_free_list_has_pages(struct pvr_device *pvr_dev)
{
	return !pvr_dev->device_data->apple_chip->own_pm_pages;
}

/**
 * pvr_apple_cmd_stream() - How a render job's command stream is decoded
 * @pvr_dev: Target device.
 * @type: %DRM_PVR_JOB_TYPE_GEOMETRY, %DRM_PVR_JOB_TYPE_FRAGMENT or
 *	%DRM_PVR_JOB_TYPE_COMPUTE.
 *
 * Return: the chip's stream definition: the Rogue command plus the registers
 * its firmware takes beyond it.
 */
const struct pvr_stream_cmd_defs *pvr_apple_cmd_stream(struct pvr_device *pvr_dev,
						       enum drm_pvr_job_type type)
{
	const struct pvr_apple_chip *chip = pvr_dev->device_data->apple_chip;

	if (type == DRM_PVR_JOB_TYPE_COMPUTE)
		return chip->compute_stream ?: &pvr_cmd_compute_stream;
	return type == DRM_PVR_JOB_TYPE_GEOMETRY ? chip->geom_stream : chip->frag_stream;
}

/**
 * pvr_apple_stream_musthave() - Required register extension for a chip's stream
 * @pvr_dev: Target device.
 * @type: Geometry or fragment stream type.
 * @ext_type: Extension header type to which the returned mask applies.
 *
 * Return: the chip's required register bits, or zero for other stream types.
 */
u32 pvr_apple_stream_musthave(struct pvr_device *pvr_dev, enum pvr_stream_type type,
			      u32 *ext_type)
{
	const struct pvr_apple_chip *chip = pvr_dev->device_data->apple_chip;

	if (type == PVR_STREAM_TYPE_GEOM) {
		*ext_type = chip->geom_ext_type;
		return chip->geom_ext_mask;
	}
	if (type == PVR_STREAM_TYPE_FRAG) {
		*ext_type = chip->frag_ext_type;
		return chip->frag_ext_mask;
	}

	*ext_type = 0;
	return 0;
}
