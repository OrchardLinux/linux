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
 * pvr_apple_chip.h): pvr_apple_t8010.c for the A10.
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
#include "pvr_vm.h"

#include <drm/drm_print.h>
#include <drm/gpu_scheduler.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>

struct pvr_apple_fence {
	struct dma_fence base;
	struct list_head node;
};

/*
 * How often the stamps are checked while jobs are pending. A jiffy-based
 * delay (1-2 ms per check) cost a render about 4 ms: a geometry and a
 * fragment wait each.
 */
static unsigned int pvr_apple_poll_us = 100;
module_param_named(apple_poll_us, pvr_apple_poll_us, uint, 0644);
MODULE_PARM_DESC(apple_poll_us, "Apple A10: microseconds between job completion checks (default 100)");

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
	/* never state 0 here: that is off, and the firmware is about to start */
	err = pvr_apple_request_power_state(apple, clamp(pvr_apple_perf_state ?: apple->perf_state_max,
							 1u, apple->perf_state_max ?: 1u));
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
		err = devm_mutex_init(dev, &apple->pm_lock);
	if (err)
		return err;
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
static int pvr_apple_wait_idle(struct pvr_apple *apple, unsigned int timeout_us);

/*
 * Runtime PM. Every job the firmware runs holds a runtime-PM reference from
 * pvr_apple_run_job() until its fence leaves the pending list, so the GPU
 * goes idle only with nothing in flight. Then the firmware is shut down and
 * the GPU powered off; the next job boots it again with fresh init data, as
 * after a reset. At probe, before the init data exists, power on only boots
 * the firmware: pvr_apple_fw_init() follows.
 */
static int pvr_apple_power_on(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	if (!apple->fw_ready)
		return pvr_apple_boot(apple);

	mutex_lock(&apple->submit_lock);
	err = apple->booted ? 0 : pvr_apple_fw_restart(apple);
	mutex_unlock(&apple->submit_lock);
	if (err)
		dev_err(apple_dev(apple), "GPU did not start: %d\n", err);
	else if (apple->devfreq)
		devfreq_resume_device(apple->devfreq);
	return err;
}

static int pvr_apple_power_off(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);
	int err;

	if (!apple->fw_ready)
		return 0;

	if (apple->devfreq)
		devfreq_suspend_device(apple->devfreq);
	mutex_lock(&apple->submit_lock);
	/* Nothing holds a job reference, so this does not wait in practice. */
	err = pvr_apple_wait_idle(apple, 1000000);
	if (!err) {
		pvr_apple_fw_stop(apple);
		apple->chip->idle(apple);
	}
	mutex_unlock(&apple->submit_lock);
	if (err) {
		if (apple->devfreq)
			devfreq_resume_device(apple->devfreq);
		return -EBUSY;
	}
	/* the pollers run only while fences are pending; none are */
	hrtimer_cancel(&apple->poll_timer);
	cancel_delayed_work_sync(&apple->poll_work);
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
	return heap_read32(apple, apple->layout.stamp[engine]);
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

static int pvr_apple_wait_engine(struct pvr_apple *apple, enum pvr_apple_engine engine, u32 seq,
				 unsigned int timeout_us)
{
	u32 done;

	/*
	 * 20 us steps: this runs before every render job is written and, at a
	 * switch of GPU VM, for every engine; 1 ms steps cost a scene-heavy
	 * WebGL page several milliseconds per frame
	 */
	return read_poll_timeout(pvr_apple_engine_done, done, (s32)(done - seq) >= 0, 20,
				 timeout_us, false, apple, engine);
}

/* Wait until every engine retired its submitted jobs; submit_lock held */
static int pvr_apple_wait_idle(struct pvr_apple *apple, unsigned int timeout_us)
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

	if (!n)
		return;
	pm_runtime_mark_last_busy(dev);
	while (n--)
		pm_runtime_put_autosuspend(dev);
}

/* Signal the fences of retired jobs; poll again while any are pending. */
static void pvr_apple_poll_work(struct work_struct *work)
{
	struct pvr_apple *apple = container_of(work, struct pvr_apple, poll_work.work);
	struct pvr_apple_fence *fence, *tmp;
	unsigned int retired = 0;
	bool pending = false;
	int engine;

	pvr_apple_ack_events(apple);
	spin_lock_irq(&apple->fence_lock);
	for (engine = 0; engine < APPLE_ENGINES; engine++) {
		u32 done = pvr_apple_engine_done(apple, engine);

		/*
		 * A fragment job's stamp comes before the firmware retires it, so
		 * its target may not be complete in memory yet for readers that
		 * do not go through the GPU, such as the display. Its fence also
		 * waits for the retirement stamp.
		 */
		if (engine == APPLE_ENGINE_FRAG) {
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
	if (!pending && apple->busy_since) {
		apple->busy_ns += ktime_to_ns(ktime_sub(ktime_get(), apple->busy_since));
		apple->busy_since = 0;
	}
	spin_unlock_irq(&apple->fence_lock);
	pvr_apple_jobs_retired(apple, retired);

	/* The firmware sends no completion message in this setup: poll. */
	if (pending)
		hrtimer_start(&apple->poll_timer, us_to_ktime(max(pvr_apple_poll_us, 10U)),
			      HRTIMER_MODE_REL);
}

static enum hrtimer_restart pvr_apple_poll_timer(struct hrtimer *timer)
{
	struct pvr_apple *apple = container_of(timer, struct pvr_apple, poll_timer);

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
	apple->chip->report_hang(apple);
}

static unsigned int pvr_apple_cancel_fences(struct pvr_apple *apple, int error)
{
	struct pvr_apple_fence *fence, *tmp;
	unsigned int cancelled = 0;
	int engine;

	/* each rearms the other: the timer, the work, then a timer it started */
	hrtimer_cancel(&apple->poll_timer);
	cancel_delayed_work_sync(&apple->poll_work);
	hrtimer_cancel(&apple->poll_timer);
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
	spin_unlock_irq(&apple->fence_lock);
	mod_delayed_work(system_highpri_wq, &apple->poll_work, 0);
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
	err = pvr_apple_wait_idle(apple, 5000000);
	mutex_unlock(&apple->submit_lock);

	if (err) {
		dev_err(apple_dev(apple), "GPU did not retire its jobs before removal\n");
		pvr_apple_cancel_fences(apple, -ETIMEDOUT);
	} else {
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

	if (vm_ctx == pvr_dev->kernel_vm_ctx || !apple->booted)
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
 * Return: 0 or a negative error.
 */
int pvr_apple_job_add_deps(struct pvr_job *job)
{
	struct pvr_apple *apple = to_apple(job->pvr_dev);
	struct dma_fence *last;

	if (job->type != DRM_PVR_JOB_TYPE_GEOMETRY && job->type != DRM_PVR_JOB_TYPE_FRAGMENT)
		return 0;
	spin_lock_irq(&apple->fence_lock);
	last = dma_fence_get(job->type == DRM_PVR_JOB_TYPE_GEOMETRY ? apple->last_frag :
								     apple->last_geom);
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
	struct dma_fence **last, *old;

	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY)
		last = &apple->last_geom;
	else if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		last = &apple->last_frag;
	else
		return;
	spin_lock_irq(&apple->fence_lock);
	old = *last;
	*last = dma_fence_get(finished);
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
	dma_fence_put(apple->last_frag);
	apple->last_frag = NULL;
	dma_fence_put(apple->last_geom);
	apple->last_geom = NULL;
	spin_lock_irq(&apple->fence_lock);
	apple->busy_since = 0;
	spin_unlock_irq(&apple->fence_lock);

	pvr_apple_fw_stop(apple);
	err = pvr_apple_fw_restart(apple);
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

	if (apple->devfreq)
		devfreq_suspend_device(apple->devfreq);

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
	dma_fence_put(apple->last_frag);
	apple->last_frag = NULL;
	dma_fence_put(apple->last_geom);
	apple->last_geom = NULL;
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
	else if (apple->devfreq)
		devfreq_resume_device(apple->devfreq);
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

void pvr_apple_fw_fini(struct pvr_device *pvr_dev)
{
	struct pvr_apple *apple = to_apple(pvr_dev);

	apple->fw_ready = false;
	dma_fence_put(apple->last_frag);
	apple->last_frag = NULL;
	dma_fence_put(apple->last_geom);
	apple->last_geom = NULL;
	if (apple->booted)
		pvr_apple_switch_to_kernel_vm(apple, NULL);
	pvr_apple_cancel_fences(apple, -ENODEV);
	pvr_apple_gpu_pages_fini(apple);
	if (apple->pm_pages) {
		/* free lists still holding the pages keep their own references */
		pvr_gem_object_put(apple->pm_pages);
		apple->pm_pages = NULL;
	}
	if (!apple->booted)
		return;
	apple->chip->stop(apple, false);
	/* the firmware's messages until its stop queued completion checks */
	cancel_delayed_work_sync(&apple->poll_work);
	hrtimer_cancel(&apple->poll_timer);
	apple->booted = false;
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
 * pvr_apple_cmd_stream() - How a render job's command stream is decoded
 * @pvr_dev: Target device.
 * @type: %DRM_PVR_JOB_TYPE_GEOMETRY or %DRM_PVR_JOB_TYPE_FRAGMENT.
 *
 * Return: the chip's stream definition: the Rogue command plus the registers
 * its firmware takes beyond it.
 */
const struct pvr_stream_cmd_defs *pvr_apple_cmd_stream(struct pvr_device *pvr_dev,
						       enum drm_pvr_job_type type)
{
	const struct pvr_apple_chip *chip = pvr_dev->device_data->apple_chip;

	return type == DRM_PVR_JOB_TYPE_GEOMETRY ? chip->geom_stream : chip->frag_stream;
}
