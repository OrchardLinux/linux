// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS driver for the Apple A9/A10 display pipe, disp0.
 *
 * iBoot leaves the eDP link, the panel and the timing generator running.
 * This driver adopts that mode and never programs timings or clocks; it
 * trains the link only after a sleep with the panel off (see below), with the
 * boot loader's link settings. It offers one XRGB8888 primary plane. A new
 * framebuffer reaches the pipe through the ADP parameter FIFO, which holds
 * the packet until the next frame boundary and applies it there; the pipe
 * then raises the "packet applied" status bit, which completes the page flip.
 * A flip event therefore never arrives before the new buffer is being
 * scanned out.
 *
 * The block is the ADBE back end of iOS's AppleH8ADBE0 kext
 * (AppleMobileADBE0, AppleH7DisplayPipe). Its FIFO port and layer registers
 * (0x10c0, 0x402c, 0x4030, 0x4038, 0x4068) sit at the same offsets as in the
 * H7 pipe driven by drivers/gpu/drm/adp.
 *
 * Disabling the CRTC scans out the boot framebuffer, cleared to black,
 * instead of clearing the layer enable or touching the link: neither has
 * been verified on hardware.
 *
 * The pipe's bus is 32 bits wide with DRAM at bus address 0. iBoot leaves
 * the display DART in bypass, so the plane base register holds the low 32
 * bits of the boot framebuffer's physical address; the bus offset is taken
 * from that live scanout. Probe confirms the bypass by reading the DART,
 * never writing it, and leaves the display to simpledrm otherwise.
 * Scanout buffers are ordinary DMA allocations that must fall inside the
 * same 4 GiB window.
 *
 * With dart_scanout=1 the pipe reads through dart-disp0 instead, as on iOS,
 * where IOMobileFramebuffer maps every swap surface through the display's
 * IOMapper as one DART segment. Any buffer, including another device's
 * scattered dma-buf, can then be scanned out. Probe maps the live boot
 * framebuffer at its bypass bus address before translation starts, keeps
 * every buffer it handed to the pipe mapped until the pipe is seen reading a
 * newer one, and returns the DART to bypass, on the boot framebuffer, at
 * shutdown.
 *
 * Where the device tree gives the panel supply, it goes off in system sleep,
 * as on iOS, where AppleDPDisplayPowerController switches the panel's
 * lcd_ldo and the DisplayPort transmitter (an Analogix DP core) trains the
 * link again on every wake. The transmitter's AUX channel is a drm_dp_aux,
 * so DPCD access goes through the shared DP helpers. The pipe and the transmitter keep running as iBoot set them
 * up: only the sink loses power. Before the supply is cut, the link must
 * read back as trained over AUX and the supply must read on; otherwise the
 * panel stays on for that sleep. On wake the link gets the boot loader's
 * configuration and a no-handshake (fast) training, as iOS does, and the
 * backlight comes on only after the link reads back trained.
 */

#include <linux/aperture.h>
#include <linux/apple-dart.h>
#include <linux/backlight.h>
#include <linux/console.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_dumb_buffers.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

/* Offsets in the ADP window, reg[0]. */
#define ADP_SPDS		0x1014	/* [15:0] SPDS version */
#define ADP_SPDS_VERSION	0x13
#define ADP_PARMST		0x101c	/* parameter FIFO state */
#define ADP_PARMST_STATE	GENMASK(7, 4)
#define ADP_INTEN		0x1028
#define ADP_STATUS		0x102c	/* W1C; reads are unmasked */
#define ADP_CRC			0x2030	/* blend CRC, read only */
#define ADP_CRC_CTL		0x1074	/* iOS: 1 enables blend CRC, 0 disables */
#define ADP_CRC_CTL_EN		BIT(0)
#define ADP_BLEND_BYPASS	0x2020	/* logged only, never written here */
#define ADP_FRAME		0x1034	/* [15:0] frame counter */
#define ADP_VALID		0x1038	/* bit 0: pipe state usable */
#define ADP_FIFO_PTR		0x1040
#define ADP_FIFO_PTR_BUSY	GENMASK(12, 0)	/* != 0: packet in flight */
#define ADP_PKT			0x10c0	/* parameter FIFO push port */

/* Pipe 0, layer 0. */
#define ADP_FORMAT		0x402c
#define ADP_FORMAT_FLAGS	(BIT(0) | BIT(21) | BIT(22) | GENMASK(27, 24))
#define ADP_BASE		0x4030
#define ADP_STRIDE		0x4038
#define ADP_LAYERS		0x4068

#define ADP_ST_VBI		BIT(0)
/*
 * Packet completion. Every parameter packet raises bit 10 at the frame
 * boundary where it is applied: the FIFO holds it (ParmSt 3, 0x1040 = 1)
 * until that boundary, then ParmSt returns to 1, 0x1040 to 0 and bit 10
 * sets together with VBI. iOS waits on 0x408; bit 3 ("packet synced") was
 * never seen on this pipe, but is accepted as well.
 */
#define ADP_ST_SYNCED		BIT(3)
#define ADP_ST_APPLIED		BIT(10)
#define ADP_ST_DONE		(ADP_ST_SYNCED | ADP_ST_APPLIED)
#define ADP_ST_ERRORS		0xa42	/* OutUnder, PFAlign, PFOver, RdErr */

/*
 * Parameter FIFO packets: a preamble (bit 31, armed bit 30, word count in
 * [28:16], token in [15:0]) followed by register entries, each a header
 * (bit 29, one data word in [28:16], register in [15:0]) and the value.
 * The pipe loads the token into its frame counter when it takes the packet,
 * so the driver sends the next frame number to keep the counter continuous.
 */
#define ADP_PKT_PREAMBLE	BIT(31)
#define ADP_PKT_ARMED		BIT(30)
#define ADP_PKT_COUNT		GENMASK(28, 16)
#define ADP_PKT_TOKEN		GENMASK(15, 0)
#define ADP_PKT_ENTRY		BIT(29)
#define ADP_PKT_REG		GENMASK(15, 0)
#define ADP_PKT_MAX_WORDS	4

/*
 * Display DART (dart-disp0, "dart,s5l8960x" family), read only: TCR at 0xc
 * (one byte per stream, bit 7 = translate), TTBRs at 0x40 (valid bit 31,
 * PA >> 12). Page tables: 4 KiB granule, TTBR index = bus[31:30],
 * L1 = bus[29:21], L2 = bus[20:12], PTE valid bit 0, address bits [35:12].
 */
#define DART_WINDOWS		2
#define DART_STREAMS		4
#define DART_TTBRS		4
#define DART_TCR		0x0c
#define DART_TCR_TRANSLATE	BIT(7)
#define DART_TTBR(sid, idx)	(0x40 + 4 * ((sid) * DART_TTBRS + (idx)))
#define DART_TTBR_VALID		BIT(31)
#define DART_TTBR_ADDR		GENMASK(23, 0)
#define DART_PTE_VALID		BIT_ULL(0)
#define DART_PTE_ADDR		GENMASK_ULL(35, 12)

/*
 * DisplayPort transmitter ("displayport,s8003", AppleANXDPTX), the "dp"
 * window: an Analogix DP core, but AUX takes the request nibble next to
 * address bits 19:16 and keeps its data at 0xc00, as iOS's readDPCDGated and
 * writeDPCDGated use it. Status 0x3dc: bit 1 reply received, bit 0 AUX
 * error (cause in 0x780[3:0]); iOS unmasks both in 0x3f8 for a transaction.
 */
#define DPTX_INT_STA		0x3dc
#define DPTX_INT_STA_MASK	0x3f8
#define DPTX_INT_AUX_ERR	BIT(0)
#define DPTX_INT_RPLY		BIT(1)
#define DPTX_SYS_CTL_3		0x608
#define DPTX_SYS_CTL_3_HPD	BIT(6)
#define DPTX_ASSR		0x614	/* bit 0: alternate scrambler seed (setASSREnabledGated) */
#define DPTX_ASSR_EN		BIT(0)
#define DPTX_TRAINING_PTN	0x688	/* [1:0] pattern, bit 5 scrambling off */
#define DPTX_TRAINING_PTN_MASK	GENMASK(1, 0)
#define DPTX_TRAINING_NO_SCRAMBLE BIT(5)
#define DPTX_AUX_CH_STA		0x780
#define DPTX_AUX_STATUS		GENMASK(3, 0)
#define DPTX_AUX_STATUS_NACK	1
#define DPTX_AUX_STATUS_TIMEOUT	2
#define DPTX_AUX_STATUS_NACK_M	7
#define DPTX_AUX_RX_COMM	0x78c
#define DPTX_AUX_BUF_CTL	0x790
#define DPTX_AUX_BUF_CLR	BIT(9)
#define DPTX_AUX_BUF_COUNT	GENMASK(8, 0)
#define DPTX_AUX_LEN		0x794	/* length - 1 */
#define DPTX_AUX_ADDR_7_0	0x798
#define DPTX_AUX_ADDR_15_8	0x79c
#define DPTX_AUX_ADDR_19_16	0x7a0	/* [7:4] request, [3:0] address 19:16 */
#define DPTX_AUX_REQUEST	GENMASK(7, 4)
#define DPTX_AUX_CTL_2		0x7a4
#define DPTX_AUX_EN		BIT(0)
#define DPTX_AUX_ADDR_ONLY	BIT(1)
#define DPTX_AUX_DATA(i)	(0xc00 + 4 * (i))
#define DPTX_AUX_WAIT_US	20000

/*
 * Panel power timing, AppleDPDisplayPowerController's defaults (no ADT
 * override on J71t): the supply stays off at least 125 ms, hot-plug detect
 * is expected 67 ms after it comes on, and the backlight waits 193 ms.
 */
#define ADBE_PANEL_OFF_MIN_MS	125
#define ADBE_PANEL_HPD_MS	67
#define ADBE_PANEL_BL_MS	193
#define ADBE_PANEL_HPD_WAIT_US	(500 * USEC_PER_MSEC)
#define ADBE_PANEL_AUX_WAIT_US	(500 * USEC_PER_MSEC)
#define ADBE_PANEL_TRIES	3
/* Fast training: each pattern for at least 500 us. */
#define ADBE_TPS_US		1000
/* DPCD 0x100..0x10a: the link the boot loader trained, replayed on wake. */
#define ADBE_LINK_CFG_LEN	(DP_EDP_CONFIGURATION_SET - DP_LINK_BW_SET + 1)

#define ADBE_ALIGN		64	/* plane base and stride */
#define ADBE_MIN_SIZE		32	/* panel width and height */
#define ADBE_MAX_SIZE		4096
#define ADBE_ADDR_TAG		(ADBE_ALIGN - 1)	/* low bits of base/stride */
#define ADBE_BUS_LIMIT		BIT_ULL(32)
#define ADBE_KICK_WAIT_US	(20 * USEC_PER_MSEC)	/* iOS allows 1 s */
#define ADBE_PARK_WAIT_US	(100 * USEC_PER_MSEC)
/* Translated scanout: buffers held for the pipe, and when they are released. */
#define ADBE_HOLD_MAX		8
#define ADBE_HOLD_FRAMES	1	/* boundaries after a newer entry is seen live */
#define ADBE_HOLD_POLL_MS	8
#define ADBE_FRAME_MS		17	/* one 60 Hz frame, rounded up */
/* Smallest IOVA window left below the boot framebuffer for allocations. */
#define ADBE_DART_MIN_WINDOW	SZ_256M
#define ADBE_VERIFY_PAGE	SZ_16K
#define ADBE_VERIFY_IOVA_START	SZ_16M

static bool dart_scanout = true;
module_param(dart_scanout, bool, 0444);
MODULE_PARM_DESC(dart_scanout,
		 "Scan out through the display DART, so any dma-buf can be shown (reads back the active mode)");
#define ADBE_FLIP_VBLANKS	3	/* complete on VBI if bit 10 never comes */
#define ADBE_FLIP_TIMEOUT_MS	100	/* complete anyway without an interrupt */
#define ADBE_INTEN_VBLANK	(ADP_ST_VBI | ADP_ST_APPLIED)

struct adbe {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *regs;
	void __iomem *boot_va;
	struct resource boot;
	u64 bus_offset;
	/* The panel mode iBoot left running, and its boot framebuffer. */
	struct drm_display_mode mode;
	u32 pitch;
	size_t fb_size;
	u32 boot_base;
	u32 boot_stride;
	u32 saved_inten;
	bool vbl_enabled;

	/* Staged by the plane update, pushed by the CRTC flush. */
	u32 next_base;
	u32 next_stride;
	bool next_dirty;
	/* Commit tail only. */
	u32 cur_base;
	bool boot_black;

	/* Protected by drm.event_lock. */
	struct drm_pending_vblank_event *event;
	unsigned int event_vbls;
	bool event_applied;
	u64 kick_seq;
	struct timer_list flip_timer;

	/* Translated scanout only (dart_scanout); scanout is NULL otherwise. */
	struct device_node *scanout_np;
	struct platform_device *scanout;
	struct platform_device *dart;
	void __iomem *dart_regs[DART_WINDOWS];
	struct iommu_domain *domain;
	u32 boot_iova_len;
	/*
	 * Framebuffers handed to the pipe, oldest first. One is released only
	 * after the pipe has shown a newer one and ADBE_HOLD_FRAMES frame
	 * boundaries have passed, so no buffer is unmapped while it is still
	 * fetched.
	 */
	spinlock_t hold_lock;
	struct {
		struct drm_framebuffer *fb;
		u32 base;
		bool seen;	/* live base has shown this entry */
		u16 seen_frame;	/* ADP_FRAME when it was first seen */
	} hold[ADBE_HOLD_MAX];
	unsigned int nhold;
	struct delayed_work hold_work;

	/*
	 * Panel power over system sleep. The supply and the backlight are
	 * looked up before the first sleep, not at probe, so that probe does
	 * not wait for the PMU or the backlight.
	 */
	void __iomem *dp;
	struct drm_dp_aux aux;
	struct notifier_block pm_nb;
	struct gpio_desc *lcd_ldo;
	struct backlight_device *backlight;
	struct device_link *ldo_link, *bl_link;
	bool panel_ready;	/* supply and ordering in place */
	bool panel_off;		/* the supply is off for this sleep */
	bool panel_lost;	/* a wake did not bring the panel back */
	bool bl_held;		/* backlight held off after a failed wake */
	ktime_t panel_off_at;
	u8 dpcd_caps[DP_RECEIVER_CAP_SIZE];
	u8 link_cfg[ADBE_LINK_CFG_LEN];
};

static struct adbe *to_adbe(struct drm_device *drm)
{
	return container_of(drm, struct adbe, drm);
}

/*
 * Physical size of the known panels, matched by the boot framebuffer. Any
 * other panel is adopted with its size unknown.
 */
static const struct {
	u16 width, height;	/* pixels */
	u16 width_mm, height_mm;
} adbe_panels[] = {
	{ 1536, 2048, 148, 197 },	/* 9.7-inch iPad 5 and iPad 6 */
	{ 1620, 2160, 155, 207 },	/* 10.2-inch iPad 7 */
};

/* The fixed mode of the panel iBoot left running, at its boot size. */
static void adbe_init_mode(struct adbe *adbe, u32 width, u32 height)
{
	const struct drm_display_mode mode = {
		DRM_MODE_INIT(60, width, height, 0, 0),
	};
	unsigned int i;

	adbe->mode = mode;
	for (i = 0; i < ARRAY_SIZE(adbe_panels); i++) {
		if (adbe_panels[i].width == width &&
		    adbe_panels[i].height == height) {
			adbe->mode.width_mm = adbe_panels[i].width_mm;
			adbe->mode.height_mm = adbe_panels[i].height_mm;
			break;
		}
	}
}

static u32 adbe_tag_word(u32 value, u32 tag)
{
	return (value & ~ADBE_ADDR_TAG) | (tag & ADBE_ADDR_TAG);
}

/* Bus address of [cpu, cpu + len); false if not reachable or misaligned. */
static bool adbe_cpu_to_bus(struct adbe *adbe, u64 cpu, u64 len, u32 *bus)
{
	u64 offset = adbe->bus_offset;

	if (!len || cpu < offset || (cpu & ADBE_ADDR_TAG) ||
	    cpu - offset >= ADBE_BUS_LIMIT ||
	    len > ADBE_BUS_LIMIT - (cpu - offset))
		return false;
	*bus = cpu - offset;
	return true;
}

/*
 * readCurrentParamFIFOState() in iOS: ready when no packet is in flight
 * and ParmSt is not 2; ParmSt above 5 is undefined.
 */
static int adbe_fifo_state(struct adbe *adbe)
{
	u32 state = FIELD_GET(ADP_PARMST_STATE, readl(adbe->regs + ADP_PARMST));

	if (readl(adbe->regs + ADP_FIFO_PTR) & ADP_FIFO_PTR_BUSY)
		return 0;
	if (state > 5)
		return -EIO;
	return state != 2;
}

static int adbe_fifo_wait_ready(struct adbe *adbe)
{
	int state, ret;

	ret = read_poll_timeout(adbe_fifo_state, state, state, 1000,
				ADBE_KICK_WAIT_US, false, adbe);
	if (ret)
		return ret;
	return state < 0 ? state : 0;
}

/* Caller holds event_lock; the port has no per-word handshake. */
static void adbe_pkt_push(struct adbe *adbe, const u32 *words,
			  unsigned int count)
{
	u16 token = readl(adbe->regs + ADP_FRAME) + 1;
	unsigned int i;

	writel(ADP_PKT_PREAMBLE | ADP_PKT_ARMED |
	       FIELD_PREP(ADP_PKT_COUNT, count) |
	       FIELD_PREP(ADP_PKT_TOKEN, token), adbe->regs + ADP_PKT);
	for (i = 0; i < count; i++)
		writel(words[i], adbe->regs + ADP_PKT);
}

static unsigned int adbe_pkt_add(u32 *words, unsigned int count, u32 reg,
				 u32 value)
{
	words[count++] = ADP_PKT_ENTRY | FIELD_PREP(ADP_PKT_COUNT, 1) |
			 FIELD_PREP(ADP_PKT_REG, reg);
	words[count++] = value;
	return count;
}

static void adbe_send_event(struct adbe *adbe,
			    struct drm_pending_vblank_event *event)
{
	unsigned long flags;

	if (!event)
		return;
	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	drm_crtc_send_vblank_event(&adbe->crtc, event);
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	drm_crtc_vblank_put(&adbe->crtc);
}

static irqreturn_t adbe_irq(int irq, void *data)
{
	struct adbe *adbe = data;
	struct drm_pending_vblank_event *done = NULL;
	u32 status;

	/*
	 * Read and acknowledge under event_lock, so that a flip armed by
	 * adbe_kick() never completes on a status read from before its push.
	 */
	spin_lock(&adbe->drm.event_lock);
	status = readl(adbe->regs + ADP_STATUS);
	if (!status) {
		spin_unlock(&adbe->drm.event_lock);
		return IRQ_NONE;
	}
	writel(status, adbe->regs + ADP_STATUS);
	if (adbe->event) {
		if (status & ADP_ST_DONE)
			adbe->event_applied = true;
		/*
		 * Complete once the boundary's vblank has been counted (VBI in
		 * this read, or already handled), so the event carries that
		 * frame's sequence and timestamp even if bit 10 arrives just
		 * before VBI.
		 */
		if (adbe->event_applied &&
		    ((status & ADP_ST_VBI) ||
		     drm_crtc_vblank_count(&adbe->crtc) != adbe->kick_seq)) {
			done = adbe->event;
			adbe->event = NULL;
		} else if (!adbe->event_applied && (status & ADP_ST_VBI) &&
			   ++adbe->event_vbls >= ADBE_FLIP_VBLANKS) {
			done = adbe->event;
			adbe->event = NULL;
		}
	}
	spin_unlock(&adbe->drm.event_lock);

	if (status & ADP_ST_ERRORS)
		dev_warn_ratelimited(adbe->drm.dev, "pipe error bits %#x\n",
				     status & ADP_ST_ERRORS);
	/* VBI is sticky: only count it while vblank is enabled. */
	if ((status & ADP_ST_VBI) && READ_ONCE(adbe->vbl_enabled))
		drm_crtc_handle_vblank(&adbe->crtc);
	adbe_send_event(adbe, done);

	return IRQ_HANDLED;
}

/* Safety net: never leave a flip pending if no interrupt completes it. */
static void adbe_flip_timeout(struct timer_list *t)
{
	struct adbe *adbe = timer_container_of(adbe, t, flip_timer);
	struct drm_pending_vblank_event *done;
	unsigned long flags;

	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	done = adbe->event;
	adbe->event = NULL;
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	if (!done)
		return;

	dev_warn_ratelimited(adbe->drm.dev, "flip not acknowledged in %u ms\n",
			     ADBE_FLIP_TIMEOUT_MS);
	adbe_send_event(adbe, done);
}

/*
 * Push one plane update: base and, if it changed, stride. Sleeps for at most
 * ADBE_KICK_WAIT_US on the FIFO; a busy FIFO skips the update. If @event is
 * given and vblank is available, it completes when the pipe applies the
 * packet and *@event is cleared.
 */
static int adbe_kick(struct adbe *adbe, u32 base, u32 stride,
		     struct drm_pending_vblank_event **event)
{
	u32 words[ADP_PKT_MAX_WORDS];
	unsigned int count = 0;
	unsigned long flags;
	bool armed = false;
	u32 status;
	int ret;

	if (readl(adbe->regs + ADP_STRIDE) != stride)
		count = adbe_pkt_add(words, count, ADP_STRIDE, stride);
	count = adbe_pkt_add(words, count, ADP_BASE, base);

	ret = adbe_fifo_wait_ready(adbe);
	if (ret) {
		dev_warn_ratelimited(adbe->drm.dev,
				     "parameter FIFO busy, update skipped\n");
		return ret;
	}

	if (event && *event && !drm_crtc_vblank_get(&adbe->crtc))
		armed = true;

	spin_lock_irqsave(&adbe->drm.event_lock, flags);
	/* A completion latched before this push belongs to an earlier packet. */
	status = readl(adbe->regs + ADP_STATUS) & ADP_ST_DONE;
	if (status)
		writel(status, adbe->regs + ADP_STATUS);
	adbe_pkt_push(adbe, words, count);
	if (armed) {
		adbe->event = *event;
		adbe->event_vbls = 0;
		adbe->event_applied = false;
		adbe->kick_seq = drm_crtc_vblank_count(&adbe->crtc);
		*event = NULL;
	}
	spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	if (armed)
		mod_timer(&adbe->flip_timer,
			  jiffies + msecs_to_jiffies(ADBE_FLIP_TIMEOUT_MS));

	adbe->cur_base = base;
	return 0;
}

/*
 * Scan out the boot framebuffer, cleared to black. If the packet is not
 * applied in time, store the base directly and push an empty packet, as the
 * H7 pipe driver does.
 */
static void adbe_park_pipe(struct adbe *adbe)
{
	unsigned long flags;
	u32 base;

	if (!adbe->boot_black) {
		memset_io(adbe->boot_va, 0, adbe->fb_size);
		wmb();	/* publish the write-combined stores before the flip */
		adbe->boot_black = true;
	}
	if (adbe->cur_base == adbe->boot_base &&
	    readl(adbe->regs + ADP_BASE) == adbe->boot_base)
		return;

	if (!adbe_kick(adbe, adbe->boot_base, adbe->boot_stride, NULL) &&
	    !readl_poll_timeout(adbe->regs + ADP_BASE, base,
				base == adbe->boot_base, 1000,
				ADBE_PARK_WAIT_US))
		return;

	dev_warn(adbe->drm.dev, "park packet not applied, storing the boot base\n");
	writel(adbe->boot_stride, adbe->regs + ADP_STRIDE);
	writel(adbe->boot_base, adbe->regs + ADP_BASE);
	if (!adbe_fifo_wait_ready(adbe)) {
		spin_lock_irqsave(&adbe->drm.event_lock, flags);
		adbe_pkt_push(adbe, NULL, 0);
		spin_unlock_irqrestore(&adbe->drm.event_lock, flags);
	}
	adbe->cur_base = adbe->boot_base;
}

/*
 * Translated scanout: drop the held framebuffers that the pipe can no
 * longer read. The parameter FIFO applies packets in order, one at a frame
 * boundary, and a new packet is only pushed once the previous one is out of
 * the FIFO. Once the live base has shown an entry and ADBE_HOLD_FRAMES
 * further boundaries have passed, every older entry is therefore finished,
 * including ones whose update was skipped. A park is recorded as an entry
 * without a framebuffer, so it orders against later updates like any
 * buffer. A base that matches nothing releases nothing. @all drops every
 * entry; callers use it only once the DART is back in bypass.
 */
static void adbe_hold_release(struct adbe *adbe, bool all)
{
	struct drm_framebuffer *drop[ADBE_HOLD_MAX];
	unsigned int ndrop = 0, keep, i;
	unsigned long flags;
	u16 frame;
	u32 hw;

	hw = readl(adbe->regs + ADP_BASE) & ~ADBE_ADDR_TAG;
	frame = readl(adbe->regs + ADP_FRAME);

	spin_lock_irqsave(&adbe->hold_lock, flags);
	if (all) {
		ndrop = adbe->nhold;
	} else {
		/* The newest entry with the live base is the one on screen. */
		for (i = adbe->nhold; i-- > 0;) {
			if (adbe->hold[i].base == hw) {
				if (!adbe->hold[i].seen) {
					adbe->hold[i].seen = true;
					adbe->hold[i].seen_frame = frame;
				}
				break;
			}
		}
		/* Everything older than the newest entry seen long enough. */
		for (i = adbe->nhold; i-- > 0;) {
			if (adbe->hold[i].seen &&
			    (u16)(frame - adbe->hold[i].seen_frame) >=
			    ADBE_HOLD_FRAMES) {
				ndrop = i;
				break;
			}
		}
	}
	keep = adbe->nhold - ndrop;
	for (i = 0; i < ndrop; i++)
		drop[i] = adbe->hold[i].fb;
	for (i = 0; i < keep; i++)
		adbe->hold[i] = adbe->hold[ndrop + i];
	adbe->nhold = keep;
	spin_unlock_irqrestore(&adbe->hold_lock, flags);

	for (i = 0; i < ndrop; i++)
		if (drop[i])
			drm_framebuffer_put(drop[i]);
}

static void adbe_hold_work(struct work_struct *work)
{
	struct adbe *adbe = container_of(to_delayed_work(work), struct adbe,
					 hold_work);

	adbe_hold_release(adbe, false);
	/* The newest entry is the one on screen: it stays until replaced. */
	if (READ_ONCE(adbe->nhold) > 1)
		schedule_delayed_work(&adbe->hold_work,
				      msecs_to_jiffies(ADBE_HOLD_POLL_MS));
}

/*
 * Hold @fb, at bus address @bus, until the pipe has moved past it. A NULL
 * @fb records a park on the boot framebuffer.
 */
static void adbe_hold_push(struct adbe *adbe, struct drm_framebuffer *fb,
			   u32 bus)
{
	struct drm_framebuffer *old = NULL;
	unsigned long flags;
	int tries;

	/* Full only if the pipe ignores updates; give it a few frames. */
	for (tries = 0; tries < 8 && READ_ONCE(adbe->nhold) == ADBE_HOLD_MAX;
	     tries++) {
		msleep(ADBE_HOLD_POLL_MS);
		adbe_hold_release(adbe, false);
	}

	if (fb)
		drm_framebuffer_get(fb);
	spin_lock_irqsave(&adbe->hold_lock, flags);
	if (!fb && adbe->nhold && !adbe->hold[adbe->nhold - 1].fb) {
		/* Already parked. */
		spin_unlock_irqrestore(&adbe->hold_lock, flags);
		return;
	}
	if (adbe->nhold == ADBE_HOLD_MAX) {
		old = adbe->hold[0].fb;
		memmove(&adbe->hold[0], &adbe->hold[1],
			sizeof(adbe->hold[0]) * (ADBE_HOLD_MAX - 1));
		adbe->nhold--;
	}
	adbe->hold[adbe->nhold].fb = fb;
	adbe->hold[adbe->nhold].base = bus;
	adbe->hold[adbe->nhold].seen = false;
	adbe->nhold++;
	spin_unlock_irqrestore(&adbe->hold_lock, flags);

	if (old) {
		dev_warn_ratelimited(adbe->drm.dev,
				     "pipe not following updates, releasing the oldest held buffer\n");
		drm_framebuffer_put(old);
	}
	mod_delayed_work(system_wq, &adbe->hold_work,
			 msecs_to_jiffies(ADBE_HOLD_POLL_MS));
}

static void adbe_park(struct adbe *adbe)
{
	adbe_park_pipe(adbe);
	if (adbe->scanout)
		adbe_hold_push(adbe, NULL, adbe->boot_base & ~ADBE_ADDR_TAG);
}

static int adbe_plane_atomic_check(struct drm_plane *plane,
				   struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct adbe *adbe = to_adbe(plane->dev);
	struct drm_crtc_state *crtc_state = NULL;
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;
	u32 bus;
	u64 len;
	int ret;

	if (new_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret || !new_state->visible)
		return ret;

	if (fb->pitches[0] % ADBE_ALIGN || fb->pitches[0] < adbe->pitch ||
	    fb->pitches[0] > SZ_32K) {
		drm_dbg_kms(plane->dev, "pitch %u unsupported\n", fb->pitches[0]);
		return -EINVAL;
	}
	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);
	len = (u64)fb->pitches[0] * (adbe->mode.vdisplay - 1) + adbe->pitch;
	if (!adbe_cpu_to_bus(adbe, addr, len, &bus)) {
		drm_dbg_kms(plane->dev, "buffer %pad not reachable by the pipe\n",
			    &addr);
		return -EINVAL;
	}

	return 0;
}

static void adbe_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct adbe *adbe = to_adbe(plane->dev);
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;
	u32 bus;

	if (!new_state->visible)
		return;
	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);
	/* Checked in atomic_check. */
	if (!adbe_cpu_to_bus(adbe, addr, ADBE_ALIGN, &bus))
		return;
	/* Keep the tag bits iBoot left in the low bits of both words. */
	adbe->next_base = adbe_tag_word(bus, adbe->boot_base);
	adbe->next_stride = adbe_tag_word(fb->pitches[0], adbe->boot_stride);
	adbe->next_dirty = true;
	/* DRM drops its reference at flip completion, which may precede the
	 * boundary the pipe applies the base at; keep it mapped until then.
	 */
	if (adbe->scanout)
		adbe_hold_push(adbe, fb, bus);
}

static const struct drm_plane_helper_funcs adbe_plane_helper_funcs = {
	.atomic_check = adbe_plane_atomic_check,
	.atomic_update = adbe_plane_atomic_update,
};

static const struct drm_plane_funcs adbe_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const u32 adbe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const u64 adbe_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int adbe_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);
	struct adbe *adbe = to_adbe(crtc->dev);

	if (!crtc_state->enable)
		return 0;
	if (crtc_state->mode.hdisplay != adbe->mode.hdisplay ||
	    crtc_state->mode.vdisplay != adbe->mode.vdisplay)
		return -EINVAL;

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void adbe_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_commit *state)
{
	struct adbe *adbe = to_adbe(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	crtc->state->event = NULL;
	if (adbe->next_dirty) {
		adbe->next_dirty = false;
		adbe_kick(adbe, adbe->next_base, adbe->next_stride, &event);
	}
	if (!event)
		return;

	/* Nothing pushed: complete at the next vblank, or now if it is off. */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (!drm_crtc_vblank_get(crtc))
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void adbe_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_commit *state)
{
	drm_crtc_vblank_on(crtc);
}

static void adbe_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_commit *state)
{
	struct adbe *adbe = to_adbe(crtc->dev);
	struct drm_pending_vblank_event *done;
	unsigned long flags;

	timer_delete_sync(&adbe->flip_timer);
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	done = adbe->event;
	adbe->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	adbe_send_event(adbe, done);

	drm_crtc_vblank_off(crtc);
	adbe->next_dirty = false;
	adbe_park(adbe);

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int adbe_enable_vblank(struct drm_crtc *crtc)
{
	struct adbe *adbe = to_adbe(crtc->dev);

	/*
	 * VBI stays latched while vblank is off; unmasking it would report a
	 * stale boundary as a vblank now.
	 */
	writel(ADP_ST_VBI, adbe->regs + ADP_STATUS);
	WRITE_ONCE(adbe->vbl_enabled, true);
	writel(adbe->saved_inten | ADBE_INTEN_VBLANK, adbe->regs + ADP_INTEN);

	return 0;
}

static void adbe_disable_vblank(struct drm_crtc *crtc)
{
	struct adbe *adbe = to_adbe(crtc->dev);

	/* Restore the exact mask iBoot left (OutUnder only). */
	writel(adbe->saved_inten, adbe->regs + ADP_INTEN);
	WRITE_ONCE(adbe->vbl_enabled, false);
}

static const struct drm_crtc_helper_funcs adbe_crtc_helper_funcs = {
	.atomic_check = adbe_crtc_atomic_check,
	.atomic_flush = adbe_crtc_atomic_flush,
	.atomic_enable = adbe_crtc_atomic_enable,
	.atomic_disable = adbe_crtc_atomic_disable,
};

static const struct drm_crtc_funcs adbe_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = adbe_enable_vblank,
	.disable_vblank = adbe_disable_vblank,
};

static int adbe_connector_get_modes(struct drm_connector *connector)
{
	struct adbe *adbe = to_adbe(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &adbe->mode);
}

static const struct drm_connector_helper_funcs adbe_connector_helper_funcs = {
	.get_modes = adbe_connector_get_modes,
};

static const struct drm_connector_funcs adbe_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs adbe_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Enable the CRTC before the planes, so a flip always has vblank. */
static const struct drm_mode_config_helper_funcs adbe_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static int adbe_dumb_create(struct drm_file *file, struct drm_device *drm,
			    struct drm_mode_create_dumb *args)
{
	int ret;

	ret = drm_mode_size_dumb(drm, args, ADBE_ALIGN, 0);
	if (ret)
		return ret;

	return drm_gem_dma_dumb_create_internal(file, drm, args);
}

DEFINE_DRM_GEM_DMA_FOPS(adbe_fops);

static const struct drm_driver adbe_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &adbe_fops,
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(adbe_dumb_create),
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "adbe",
	.desc = "Apple A9 display pipe",
	.major = 1,
	.minor = 0,
};

/*
 * The simple-framebuffer the bootloader describes for the running scanout.
 * Returns it with a reference held, or an ERR_PTR.
 */
static struct device_node *adbe_find_boot_fb(struct adbe *adbe)
{
	struct device_node *np;
	const char *format = "";
	u32 width = 0, height = 0, stride = 0;
	int ret;

	for_each_child_of_node(of_chosen, np) {
		if (!of_device_is_compatible(np, "simple-framebuffer") ||
		    !of_device_is_available(np))
			continue;

		ret = of_reserved_mem_region_to_resource(np, 0, &adbe->boot);
		if (ret)
			ret = of_address_to_resource(np, 0, &adbe->boot);
		if (!ret)
			ret = of_property_read_u32(np, "width", &width) ?:
			      of_property_read_u32(np, "height", &height) ?:
			      of_property_read_u32(np, "stride", &stride) ?:
			      of_property_read_string(np, "format", &format);
		/*
		 * The driver never programs timings: it adopts whatever panel
		 * iBoot left running. iBoot may pad rows, e.g. a 1620-pixel
		 * row to 6528 bytes.
		 */
		if (ret || width < ADBE_MIN_SIZE || width > ADBE_MAX_SIZE ||
		    height < ADBE_MIN_SIZE || height > ADBE_MAX_SIZE ||
		    stride < width * 4 || stride > SZ_32K ||
		    stride % ADBE_ALIGN ||
		    strcmp(format, "x8r8g8b8") ||
		    resource_size(&adbe->boot) < (u64)stride * height) {
			dev_err(adbe->drm.dev,
				"boot framebuffer %ux%u, stride %u, %s not supported\n",
				width, height, stride, format);
			of_node_put(np);
			return ERR_PTR(-EINVAL);
		}
		adbe_init_mode(adbe, width, height);
		adbe->pitch = stride;
		adbe->fb_size = (size_t)stride * height;
		return np;
	}

	return ERR_PTR(-ENODEV);
}

/*
 * Retire the firmware framebuffer. The aperture helpers unplug simpledrm if
 * it is bound; a simple-framebuffer device it has not bound yet (probe
 * order, or simpledrm built as a module) is destroyed, so it cannot take
 * the boot framebuffer back later.
 */
static int adbe_take_boot_fb(struct adbe *adbe, struct device_node *fb_np)
{
	struct platform_device *fb;
	int ret;

	ret = aperture_remove_conflicting_devices(adbe->boot.start,
						  resource_size(&adbe->boot),
						  adbe_drm_driver.name);
	if (ret)
		return ret;

	fb = of_find_device_by_node(fb_np);
	if (fb) {
		of_platform_device_destroy(&fb->dev, NULL);
		put_device(&fb->dev);
	}

	return 0;
}

/* Table pages are only read if they lie inside a /memory range. */
static bool adbe_pa_is_dram(phys_addr_t pa)
{
	struct device_node *np;
	struct resource res;
	bool found = false;
	int i;

	for_each_node_by_type(np, "memory") {
		for (i = 0; !found && !of_address_to_resource(np, i, &res); i++)
			found = pa >= res.start && pa + SZ_4K - 1 <= res.end;
		if (found) {
			of_node_put(np);
			break;
		}
	}

	return found;
}

static int adbe_dart_pte(phys_addr_t table, unsigned int index, u64 *pte)
{
	u64 *page;

	if (!adbe_pa_is_dram(table))
		return -EFAULT;
	page = memremap(table, SZ_4K, MEMREMAP_WB);
	if (!page)
		return -ENOMEM;
	*pte = READ_ONCE(page[index]);
	memunmap(page);

	return 0;
}

/*
 * Whether one translating stream maps the live scanout page: 1 if it does,
 * 0 if it leaves that page unmapped or maps it elsewhere (so it is not the
 * stream the pipe reads through), <0 if its tables cannot be read.
 */
static int adbe_dart_maps_scanout(void __iomem *dart, unsigned int sid,
				  u32 bus, phys_addr_t pa)
{
	u32 ttbr = readl(dart + DART_TTBR(sid, bus >> 30));
	u64 pte;
	int ret;

	if (!(ttbr & DART_TTBR_VALID))
		return 0;
	ret = adbe_dart_pte((phys_addr_t)FIELD_GET(DART_TTBR_ADDR, ttbr) << 12,
			    (bus >> 21) & 0x1ff, &pte);
	if (ret || !(pte & DART_PTE_VALID))
		return ret;
	ret = adbe_dart_pte(pte & DART_PTE_ADDR, (bus >> 12) & 0x1ff, &pte);
	if (ret || !(pte & DART_PTE_VALID))
		return ret;

	return (pte & DART_PTE_ADDR) == (pa & ~(phys_addr_t)(SZ_4K - 1));
}

/*
 * Confirm that no display DART stream translates the live scanout (bus
 * address @bus, physical @pa). Buffers are plain DMA allocations, which
 * the pipe can only reach through bypass. An unreadable page table counts
 * as translating: an unknown state is never taken for bypass.
 */
static int adbe_check_dart_bypass(struct adbe *adbe, u32 bus, phys_addr_t pa)
{
	struct platform_device *pdev = to_platform_device(adbe->drm.dev);
	static const char * const names[DART_WINDOWS] = { "dart0", "dart1" };
	struct device *dev = adbe->drm.dev;
	struct device_node *scanout_np = of_get_child_by_name(dev->of_node, "scanout");
	bool has_scanout = !!scanout_np;
	unsigned int win, sid;

	of_node_put(scanout_np);

	for (win = 0; win < DART_WINDOWS; win++) {
		struct resource *res;
		void __iomem *dart;
		u32 tcr;
		int ret = 0;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   names[win]);
		if (!res)
			return dev_err_probe(dev, -ENODEV,
					     "no %s window: cannot check the display DART\n",
					     names[win]);
		/* Not requested: the DART is not this device's to own. */
		if (res->flags & IORESOURCE_MEM_NONPOSTED)
			dart = ioremap_np(res->start, resource_size(res));
		else
			dart = ioremap(res->start, resource_size(res));
		if (!dart)
			return -ENOMEM;

		tcr = readl(dart + DART_TCR);
		for (sid = 0; sid < DART_STREAMS && !ret; sid++) {
			if (!((tcr >> (8 * sid)) & DART_TCR_TRANSLATE))
				continue;
			/* A scanout DMA child identifies dart1 as an SMMU. */
			if (has_scanout && win == 1) {
				dev_err(dev, "display SMMU stream %u translates: unknown scanout state\n", sid);
				ret = -ENODEV;
				break;
			}
			ret = adbe_dart_maps_scanout(dart, sid, bus, pa);
			if (ret)
				dev_err(dev, "display DART %s stream %u %s the scanout\n",
					names[win], sid,
					ret < 0 ? "may translate" : "translates");
		}
		iounmap(dart);
		if (ret)
			return ret < 0 ? ret : -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_IOMMU_API
/* Non-requesting views. Only an owned, blocked rollback may force-clear
 * residual translate bits, after the IOMMU driver has attempted the detach.
 */
static void adbe_dart_iounmap(void *map)
{
	iounmap(map);
}

static int adbe_dart_map_windows(struct adbe *adbe)
{
	struct platform_device *pdev = to_platform_device(adbe->drm.dev);
	static const char * const names[] = { "dart0", "dart1" };
	unsigned int win;

	for (win = 0; win < ARRAY_SIZE(names); win++) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, names[win]);
		int ret;

		if (!res)
			return -ENODEV;
		if (res->flags & IORESOURCE_MEM_NONPOSTED)
			adbe->dart_regs[win] = ioremap_np(res->start, resource_size(res));
		else
			adbe->dart_regs[win] = ioremap(res->start, resource_size(res));
		if (!adbe->dart_regs[win])
			return -ENOMEM;
		ret = devm_add_action_or_reset(adbe->drm.dev, adbe_dart_iounmap, adbe->dart_regs[win]);
		if (ret)
			return ret;
	}
	return 0;
}

static void adbe_dart_capture(struct adbe *adbe, const char *phase)
{
	void __iomem *d = adbe->dart_regs[0], *s = adbe->dart_regs[1];

	dev_info(adbe->drm.dev,
		 "display DART %s: 0c=%08x 10=%08x 1c=%08x 20=%08x 24=%08x 2c=%08x 30=%08x\n",
		 phase, readl(d + 0x0c), readl(d + 0x10), readl(d + 0x1c),
		 readl(d + 0x20), readl(d + 0x24), readl(d + 0x2c), readl(d + 0x30));
	dev_info(adbe->drm.dev,
		 "display SMMU %s: 0c=%08x 10=%08x 14=%08x 1c=%08x 20=%08x 24=%08x 2c=%08x 1000=%08x\n",
		 phase, readl(s + 0x0c), readl(s + 0x10), readl(s + 0x14),
		 readl(s + 0x1c), readl(s + 0x20), readl(s + 0x24), readl(s + 0x2c), readl(s + 0x1000));
}

/* Only the primary DART has TTBRs; both front ends must translate. */
static int adbe_check_dart_translates(struct adbe *adbe, u32 bus, phys_addr_t pa)
{
	void __iomem *dart = adbe->dart_regs[0], *smmu = adbe->dart_regs[1];
	u32 tcr = readl(dart + DART_TCR), smmu_tcr = readl(smmu + DART_TCR);
	unsigned int sid;
	int ret;

	for (sid = 0; sid < DART_STREAMS; sid++) {
		if (!((tcr >> (8 * sid)) & DART_TCR_TRANSLATE) ||
		    !((smmu_tcr >> (8 * sid)) & DART_TCR_TRANSLATE))
			ret = -EIO;
		else
			ret = adbe_dart_maps_scanout(dart, sid, bus, pa);
		if (ret != 1) {
			dev_err(adbe->drm.dev, "display DART/SMMU stream %u does not map %#x to %pa\n",
				sid, bus, &pa);
			return ret < 0 ? ret : -EIO;
		}
	}
	return 0;
}

static bool adbe_dart_is_bypass(struct adbe *adbe)
{
	u32 d = readl(adbe->dart_regs[0] + DART_TCR);
	u32 s = readl(adbe->dart_regs[1] + DART_TCR);

	if (d || s) {
		dev_err(adbe->drm.dev, "display DART/SMMU still translating (%08x/%08x)\n", d, s);
		return false;
	}
	return true;
}

/* Claiming the blocking domain must precede any fallback TCR write. No
 * scanout allocations exist yet, and the pipe is back on its identity image.
 */
static int adbe_dart_rollback(struct adbe *adbe, struct device *sdev)
{
	u32 mask = 0x80808080, d, s;
	int ret;

	ret = iommu_device_claim_dma_owner(sdev, adbe);
	d = readl(adbe->dart_regs[0] + DART_TCR);
	s = readl(adbe->dart_regs[1] + DART_TCR);
	if (ret) {
		dev_err(adbe->drm.dev, "cannot return the display DART to bypass (%d)\n", ret);
	} else if ((d | s) & mask) {
		dev_err(adbe->drm.dev,
			"display DART rollback left translate bits set (%08x/%08x); clearing DART then SMMU\n",
			d, s);
		writel(d & ~mask, adbe->dart_regs[0] + DART_TCR);
		readl(adbe->dart_regs[0] + DART_TCR);
		writel(s & ~mask, adbe->dart_regs[1] + DART_TCR);
		readl(adbe->dart_regs[1] + DART_TCR);
		d = readl(adbe->dart_regs[0] + DART_TCR);
		s = readl(adbe->dart_regs[1] + DART_TCR);
	}
	dev_info(adbe->drm.dev, "display DART after-rollback dart0c=%08x smmu0c=%08x\n", d, s);
	return ret || !adbe_dart_is_bypass(adbe) ? -EIO : 0;
}

static u32 adbe_verify_errors(struct adbe *adbe)
{
	u32 errors = readl(adbe->regs + ADP_STATUS) & ADP_ST_ERRORS;

	if (errors)
		writel(errors, adbe->regs + ADP_STATUS);
	return errors;
}

/* No IRQ is requested yet. Require this packet's applied bit, two further
 * VBIs, then two samples on successive VBIs. Preserve errors for each step.
 */
static int adbe_verify_show(struct adbe *adbe, u32 base, u32 stride,
			    u32 *crc, u32 *errors)
{
	u32 status, frame, previous, samples[2];
	unsigned int i;
	int ret;

	ret = adbe_kick(adbe, base, stride, NULL);
	if (ret)
		goto out;
	ret = readl_poll_timeout(adbe->regs + ADP_STATUS, status,
				 status & ADP_ST_APPLIED, 1000, ADBE_PARK_WAIT_US);
	if (ret)
		goto out;
	writel(status & ADP_ST_DONE, adbe->regs + ADP_STATUS);
	for (i = 0; i < 3; i++) {
		previous = (u16)readl(adbe->regs + ADP_FRAME);
		ret = readl_poll_timeout(adbe->regs + ADP_FRAME, frame,
					 (u16)frame != previous, 1000,
					 3 * ADBE_FRAME_MS * USEC_PER_MSEC);
		if (ret)
			goto out;
		if (i)
			samples[i - 1] = readl(adbe->regs + ADP_CRC);
	}
	*crc = samples[1];
	if (readl(adbe->regs + ADP_BASE) != base ||
	    readl(adbe->regs + ADP_STRIDE) != stride)
		ret = -EIO;
	else if (samples[0] != samples[1])
		ret = -EAGAIN;
out:
	*errors |= adbe_verify_errors(adbe);
	return ret;
}

static bool adbe_verify_overlap(u64 a, size_t len, u64 b, size_t blen)
{
	return a < b + blen && b < a + len;
}

/* All manual mappings are made before the DMA allocator sees this domain.
 * Exclude both the boot mapping and scratch's bypass/identity mapping.
 */
static int adbe_verify_aliases(u32 boot, u32 scratch, size_t len, u32 alias[2])
{
	u64 at;
	unsigned int n = 0;

	for (at = ADBE_VERIFY_IOVA_START; at + len <= boot; at += ALIGN(len, SZ_16M)) {
		if (adbe_verify_overlap(at, len, boot, len) ||
		    adbe_verify_overlap(at, len, scratch, len))
			continue;
		alias[n++] = at;
		if (n == 2)
			return 0;
	}
	return -ENOSPC;
}

/* Same allocator as adbe_dumb_create(), before drm_dev_set_dma_dev(): the
 * parent is still in direct-DMA mode. Verify physical contiguity from its
 * exported sg table instead of treating an opaque DMA handle as a PA.
 */
static struct drm_gem_dma_object *adbe_verify_pattern(struct adbe *adbe,
						    size_t len, phys_addr_t *pa, u32 *bus)
{
	struct drm_gem_dma_object *obj;
	struct sg_table *sgt;
	struct scatterlist *sg;
	phys_addr_t next;
	size_t bytes = 0, off;
	unsigned int i;
	__le32 *pixels;
	int ret = -ERANGE;

	obj = drm_gem_dma_create(&adbe->drm, len);
	if (IS_ERR(obj))
		return obj;
	sgt = drm_gem_dma_get_sg_table(obj);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto put;
	}
	*pa = sg_phys(sgt->sgl);
	next = *pa;
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		if (sg_phys(sg) != next)
			break;
		next += sg->length;
		bytes += sg->length;
	}
	sg_free_table(sgt);
	kfree(sgt);
	if (bytes < len || !IS_ALIGNED(*pa, ADBE_VERIFY_PAGE) ||
	    !adbe_cpu_to_bus(adbe, *pa, len, bus))
		goto put;
	pixels = obj->vaddr;
	for (off = 0; off < len; off += sizeof(*pixels)) {
		u32 x = (off % adbe->pitch) / sizeof(*pixels);
		u32 y = off / adbe->pitch;
		u32 value = (x * 0x9e3779b1U) ^ (y * 0x85ebca77U);

		/* A distinct opaque marker at every 16 KiB page's first pixel
		 * guarantees that no two test pages contain the same bytes.
		 */
		if (!(off % ADBE_VERIFY_PAGE))
			value = off / ADBE_VERIFY_PAGE + 1;
		pixels[off / sizeof(*pixels)] = cpu_to_le32(0xff000000 | (value & 0xffffff));
	}
	wmb(); /* publish the allocator's write-combined stores */
	dma_sync_single_for_device(drm_dev_dma_dev(&adbe->drm), obj->dma_addr,
				   len, DMA_TO_DEVICE);
	return obj;
put:
	drm_gem_object_put(&obj->base);
	return ERR_PTR(ret);
}

/* A is a real bypass reference, before deferred attach; B/C cannot validate
 * each other if both fetch unrelated RAM. Every exit restores the old frame
 * before releasing scratch pages or IOVAs, and verdict dumps follow rollback.
 */
static int adbe_dart_verify(struct adbe *adbe, struct device *sdev,
			    struct iommu_domain *domain, u32 iova, size_t len, int f0)
{
	struct drm_gem_dma_object *pattern = NULL;
	const char *result = "INCONCLUSIVE", *reason = "fault count unavailable";
	u32 crc[3] = {}, errors[4] = {}, alias[2], bus = 0, ignored_crc;
	u32 crcctl[4] = {}; /* before write, after write, after A, after restore */
	u32 blend_bypass = 0;
	bool crcctl_written = false;
	u32 previous_base = readl(adbe->regs + ADP_BASE);
	u32 previous_stride = readl(adbe->regs + ADP_STRIDE);
	u32 stride = adbe_tag_word(adbe->pitch, previous_stride);
	size_t mapped_p = 0;
	phys_addr_t pa;
	bool mapped_id = false, mapped_a = false, restore_failed = false;
	int f1, delta = 0, ret;

	if (f0 < 0)
		goto verdict;
	if (!adbe_dart_is_bypass(adbe)) {
		result = "FAIL";
		reason = "reference is not bypass";
		goto verdict;
	}
	reason = "pattern allocation failed";
	pattern = adbe_verify_pattern(adbe, len, &pa, &bus);
	if (IS_ERR(pattern)) {
		pattern = NULL;
		goto verdict;
	}
	reason = "no alias IOVA window";
	if (adbe_verify_overlap(bus, len, iova, len) ||
	    adbe_verify_aliases(iova, bus, len, alias))
		goto cleanup;
	reason = "alias mapping failed";
	ret = iommu_map(domain, bus, pa, len, IOMMU_READ, GFP_KERNEL);
	if (ret)
		goto cleanup;
	mapped_id = true;
	ret = iommu_map(domain, alias[0], pa, len, IOMMU_READ, GFP_KERNEL);
	if (ret)
		goto cleanup;
	mapped_a = true;
	/* Swap exactly the first two 16 KiB pages, not the image halves. */
	ret = iommu_map(domain, alias[1], pa + ADBE_VERIFY_PAGE,
			ADBE_VERIFY_PAGE, IOMMU_READ, GFP_KERNEL);
	if (ret)
		goto cleanup;
	mapped_p = ADBE_VERIFY_PAGE;
	ret = iommu_map(domain, alias[1] + ADBE_VERIFY_PAGE, pa,
			ADBE_VERIFY_PAGE, IOMMU_READ, GFP_KERNEL);
	if (ret)
		goto cleanup;
	mapped_p += ADBE_VERIFY_PAGE;
	ret = iommu_map(domain, alias[1] + mapped_p, pa + mapped_p,
			len - mapped_p, IOMMU_READ, GFP_KERNEL);
	if (ret)
		goto cleanup;
	mapped_p = len;

	console_lock();
	/* Discard inherited errors, especially a warm boot's output underrun. */
	adbe_verify_errors(adbe);
	/* iOS setBlendCRCEnable (ExportCRCAtVBI path): direct write of 1. */
	crcctl[0] = readl(adbe->regs + ADP_CRC_CTL);
	blend_bypass = readl(adbe->regs + ADP_BLEND_BYPASS);
	writel(ADP_CRC_CTL_EN, adbe->regs + ADP_CRC_CTL);
	crcctl_written = true;
	crcctl[1] = readl(adbe->regs + ADP_CRC_CTL);
	if (!(crcctl[1] & ADP_CRC_CTL_EN)) {
		reason = "pipe CRC enable not live";
		goto restore_crc;
	}
	ret = adbe_verify_show(adbe, adbe_tag_word(bus, previous_base), stride,
			       &crc[0], &errors[0]);
	crcctl[2] = readl(adbe->regs + ADP_CRC_CTL);
	if (ret || !crc[0]) {
		reason = "pipe CRC not live";
		goto restore;
	}
	ret = iommu_deferred_attach(sdev, domain);
	if (ret) {
		result = "FAIL";
		reason = "translation attach failed";
		goto restore;
	}
	if (adbe_check_dart_translates(adbe, bus, pa) ||
	    adbe_check_dart_translates(adbe, alias[0], pa) ||
	    adbe_check_dart_translates(adbe, alias[1], pa + ADBE_VERIFY_PAGE)) {
		result = "FAIL";
		reason = "translation read-back changed";
		goto restore;
	}
	ret = adbe_verify_show(adbe, adbe_tag_word(alias[0], previous_base), stride,
			       &crc[1], &errors[1]);
	if (ret == -EAGAIN) {
		reason = "translated pattern CRC unstable";
		goto restore;
	}
	if (ret || crc[1] != crc[0]) {
		result = "FAIL";
		reason = ret ? "pipe update failed" : "wrong pixels through translation";
		goto restore;
	}
	ret = adbe_verify_show(adbe, adbe_tag_word(alias[1], previous_base), stride,
			       &crc[2], &errors[2]);
	if (ret) {
		reason = ret == -EAGAIN ? "permuted pattern CRC unstable" : "pipe update failed";
		if (ret != -EAGAIN)
			result = "FAIL";
	} else if (crc[2] == crc[0]) {
		reason = "pipe CRC not live";
	} else {
		reason = NULL;
	}
restore:
	/* R: previous image, including its exact base/stride tag bits. Its CRC
	 * need not be live: applied/base/stride and the frame waits qualify it.
	 */
	ret = adbe_verify_show(adbe, previous_base, previous_stride,
			       &ignored_crc, &errors[3]);
	restore_failed = ret && ret != -EAGAIN;
restore_crc:
	if (crcctl_written) {
		writel(crcctl[0], adbe->regs + ADP_CRC_CTL);
		crcctl[3] = readl(adbe->regs + ADP_CRC_CTL);
	}
	console_unlock();
	if (restore_failed) {
		result = "FAIL";
		reason = "previous framebuffer restore failed";
		dev_err(adbe->drm.dev,
			"display DART cannot restore previous scanout; retaining scratch and IOVAs\n");
	}
cleanup:
	/* Never free or unmap anything the pipe might still be fetching. */
	if (!restore_failed) {
		bool failed = false;

		if (mapped_p && iommu_unmap(domain, alias[1], mapped_p) != mapped_p)
			failed = true;
		if (mapped_a && iommu_unmap(domain, alias[0], len) != len)
			failed = true;
		if (mapped_id && iommu_unmap(domain, bus, len) != len)
			failed = true;
		if (failed) {
			result = "FAIL";
			reason = "alias unmap failed";
		}
	}
verdict:
	/* Both IRQ sources and their sticky/masked errors feed this counter. */
	f1 = apple_dart_fault_count(&adbe->dart->dev);
	if (f0 >= 0 && f1 >= f0)
		delta = f1 - f0;
	if (delta) {
		result = "FAIL";
		reason = "DART/SMMU fault";
	} else if (f0 < 0 || f1 < f0) {
		if (result[0] != 'F') {
			result = "INCONCLUSIVE";
			reason = "fault count unavailable";
		}
	}
	/* Bit 1 is OutUnder (reported only); bits 6/9/11 are PFAlign/PFOver/RdErr. */
	if ((errors[1] | errors[2]) & (ADP_ST_ERRORS & ~BIT(1))) {
		result = "FAIL";
		reason = "translated pipe error";
	}
	if (!reason &&
	    (adbe_check_dart_translates(adbe, iova, adbe->boot.start) ||
	     adbe_check_dart_translates(adbe, iova + len - SZ_4K,
				 adbe->boot.start + len - SZ_4K))) {
		result = "FAIL";
		reason = "translation read-back changed";
	}
	ret = 0;
	if (reason) {
		if (restore_failed || adbe_dart_rollback(adbe, sdev))
			ret = -EIO;
		else
			ret = -ENODEV;
	}
	if (pattern && !restore_failed && ret != -EIO)
		drm_gem_object_put(&pattern->base);
	/* This capture is AFTER rollback and any forced clear, never before. */
	adbe_dart_capture(adbe, "after-verdict");
	if (!reason) {
		dev_info(adbe->drm.dev,
			 "display DART self-check passed: pattern crc %08x translated %08x permuted %08x faults +%d step=A errbits=%#x step=B errbits=%#x step=C errbits=%#x step=R errbits=%#x crcctl %08x>%08x>%08x restored %08x blendbypass %08x\n",
			 crc[0], crc[1], crc[2], delta, errors[0], errors[1], errors[2], errors[3],
			 crcctl[0], crcctl[1], crcctl[2], crcctl[3], blend_bypass);
	} else {
		dev_info(adbe->drm.dev,
			 "display DART self-check %s (%s: pattern crc %08x translated %08x permuted %08x faults +%d step=A errbits=%#x step=B errbits=%#x step=C errbits=%#x step=R errbits=%#x crcctl %08x>%08x>%08x restored %08x blendbypass %08x); %s\n",
			result, reason, crc[0], crc[1], crc[2], delta,
			errors[0], errors[1], errors[2], errors[3],
			crcctl[0], crcctl[1], crcctl[2], crcctl[3], blend_bypass,
			ret == -EIO ? "rollback unconfirmed; leaving scanout unmanaged" : "back to bypass");
	}
	return ret;
}

/*
 * Enable the display DART node, bind its driver, and order this device
 * after it for system sleep: the DART's resume resets and restores its
 * streams, which must happen before the pipe is given an IOVA again.
 */
static int adbe_dart_bind(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	struct platform_device *pdev = NULL;
	struct of_phandle_args args;
	int ret = 0;

	if (!IS_ENABLED(CONFIG_OF_DYNAMIC))
		return -EOPNOTSUPP;

	if (of_parse_phandle_with_args(adbe->scanout_np, "iommus",
				       "#iommu-cells", 0, &args))
		return -ENODEV;

#ifdef CONFIG_OF_DYNAMIC
	if (!of_device_is_available(args.np)) {
		struct of_changeset cs;

		of_changeset_init(&cs);
		ret = of_changeset_update_prop_string(&cs, args.np, "status",
						      "okay") ?:
		      of_changeset_apply(&cs);
		of_changeset_destroy(&cs);
	}
#endif

	if (!ret) {
		pdev = of_find_device_by_node(args.np);
		if (!pdev) {
			pdev = of_platform_device_create(args.np, NULL, NULL);
			if (pdev)
				get_device(&pdev->dev);
		}
		if (!pdev) {
			ret = -ENODEV;
		} else if (!pdev->dev.driver) {
			dev_err(dev, "%pOF: no DART driver bound\n", args.np);
			ret = -ENODEV;
		} else if (!device_link_add(dev, &pdev->dev,
					    DL_FLAG_STATELESS)) {
			ret = -EINVAL;
		}
		if (ret && pdev) {
			put_device(&pdev->dev);
			pdev = NULL;
		}
	}
	adbe->dart = pdev;
	of_node_put(args.np);

	return ret;
}

/*
 * Switch the pipe from bypass to translated scanout. The boot framebuffer
 * stays where the pipe reads it: its bypass bus address is mapped to its
 * physical address in the scanout device's default domain while that
 * domain is not yet attached (apple,dart-deferred-attach), and only then
 * is it attached. New allocations stay below the boot framebuffer.
 *
 * Returns 0 in translated mode, -EIO if the DART state is unknown (the
 * caller must leave the display alone), or another error with the pipe
 * still reading through bypass.
 */
static int adbe_dart_init(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	u32 iova = adbe->boot_base & ~ADBE_ADDR_TAG;
	size_t len = PAGE_ALIGN(adbe->fb_size);
	struct platform_device *spdev;
	struct iommu_domain *domain;
	struct device *sdev;
	int ret, f0;

	if (iova < ADBE_DART_MIN_WINDOW || (u64)iova + len > ADBE_BUS_LIMIT ||
	    !IS_ALIGNED(adbe->boot.start, PAGE_SIZE) ||
	    !IS_ALIGNED(iova, PAGE_SIZE)) {
		dev_warn(dev, "boot framebuffer at bus %#x leaves no IOVA window\n",
			 iova);
		return -ERANGE;
	}

	adbe->scanout_np = of_get_child_by_name(dev->of_node, "scanout");
	if (!adbe->scanout_np)
		return -ENODEV;

	ret = adbe_dart_map_windows(adbe);
	if (ret)
		return ret;
	adbe_dart_capture(adbe, "before-attach");

	ret = adbe_dart_bind(adbe);
	if (ret)
		return ret;

	spdev = of_platform_device_create(adbe->scanout_np, NULL, dev);
	if (!spdev)
		return -ENODEV;
	sdev = &spdev->dev;
	if (!device_iommu_mapped(sdev))
		of_dma_configure(sdev, adbe->scanout_np, true);
	if (!device_iommu_mapped(sdev)) {
		dev_err(dev, "scanout device has no display DART\n");
		return -ENODEV;
	}
	if (!sdev->iommu->attach_deferred)
		dev_err(dev, "display DART attached before the boot framebuffer was mapped\n");

	ret = dma_set_mask_and_coherent(sdev, DMA_BIT_MASK(32));
	if (ret)
		goto bypass;
	/* Allocations come from below the boot framebuffer. */
	sdev->bus_dma_limit = iova - 1;

	domain = iommu_get_domain_for_dev(sdev);
	if (!domain || (domain->type != IOMMU_DOMAIN_DMA &&
			domain->type != IOMMU_DOMAIN_DMA_FQ)) {
		ret = -ENODEV;
		goto bypass;
	}
	ret = iommu_map(domain, iova, adbe->boot.start, len, IOMMU_READ,
			GFP_KERNEL);
	if (ret)
		goto bypass;
	if (iommu_iova_to_phys(domain, iova) != adbe->boot.start ||
	    iommu_iova_to_phys(domain, iova + len - 1) !=
	    adbe->boot.start + len - 1) {
		ret = -EIO;
		goto bypass;
	}

	f0 = apple_dart_fault_count(&adbe->dart->dev);
	ret = adbe_dart_verify(adbe, sdev, domain, iova, len, f0);
	if (ret)
		return ret;

	adbe->scanout = spdev;
	adbe->domain = domain;
	adbe->boot_iova_len = len;
	adbe->bus_offset = 0;
	drm_dev_set_dma_dev(&adbe->drm, sdev);
	dev_info(dev, "scanout through the display DART, boot framebuffer at %#x\n",
		 iova);

	return 0;

bypass:
	/* An unconfirmed boot base must retain its alias mappings and domain. */
	if ((readl(adbe->regs + ADP_BASE) & ~ADBE_ADDR_TAG) != iova) {
		dev_err(dev, "display DART cannot restore boot scanout; leaving mappings alive\n");
		return -EIO;
	}
	/* The blocking domain of this DART is bypass, as iBoot left it. */
	if (adbe_dart_rollback(adbe, sdev))
		return -EIO;
	adbe_dart_capture(adbe, "after-verdict");
	return ret == -EIO ? -ENODEV : ret;
}

/*
 * Return the display DART to bypass with the pipe on the boot framebuffer,
 * the state iBoot leaves and the next kernel's probe checks for.
 */
static void adbe_dart_exit(struct adbe *adbe)
{
	u32 base;

	if (!adbe->scanout)
		return;

	cancel_delayed_work_sync(&adbe->hold_work);
	base = readl(adbe->regs + ADP_BASE);
	if (base != adbe->boot_base) {
		dev_err(adbe->drm.dev,
			"pipe reads %#x, not the boot framebuffer: display DART left translating\n",
			base);
		return;
	}
	/* Let the frame in flight finish on the boot framebuffer. */
	msleep(2 * ADBE_FRAME_MS);
	if (adbe_dart_rollback(adbe, &adbe->scanout->dev))
		return;
	adbe_hold_release(adbe, true);
	adbe->scanout = NULL;
	dev_info(adbe->drm.dev, "display DART back in bypass\n");
}
#else
static int adbe_dart_init(struct adbe *adbe)
{
	return -EOPNOTSUPP;
}

static void adbe_dart_exit(struct adbe *adbe)
{
}
#endif

/* Check that the pipe runs as iBoot leaves it; reads only. */
static int adbe_adopt(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	u32 base, stride, format;
	u16 frame;
	int ret;

	format = readl(adbe->regs + ADP_FORMAT) & ~ADP_FORMAT_FLAGS;
	if ((readl(adbe->regs + ADP_SPDS) & 0xffff) != ADP_SPDS_VERSION ||
	    (format != 0xc6000 && format != 0xe4000) ||
	    !(readl(adbe->regs + ADP_VALID) & BIT(0)) ||
	    (readl(adbe->regs + ADP_LAYERS) & 0xf) != 1 ||
	    (readl(adbe->regs + ADP_STATUS) & ADP_ST_ERRORS))
		return dev_err_probe(dev, -ENODEV,
				     "pipe is not running as iBoot leaves it\n");

	frame = readl(adbe->regs + ADP_FRAME);
	msleep(50);
	if ((u16)readl(adbe->regs + ADP_FRAME) == frame)
		return dev_err_probe(dev, -ENODEV,
				     "frame counter is not advancing\n");

	base = readl(adbe->regs + ADP_BASE);
	stride = readl(adbe->regs + ADP_STRIDE);
	/*
	 * With the display DART in bypass the plane base is the low half of
	 * the boot framebuffer's physical address. Anything else means the
	 * pipe reads through a translation this driver does not manage.
	 */
	if ((stride & ~ADBE_ADDR_TAG) != adbe->pitch ||
	    (base & ~ADBE_ADDR_TAG) != lower_32_bits(adbe->boot.start))
		return dev_err_probe(dev, -ENODEV,
				     "plane %#x/%#x is not the boot framebuffer %pa\n",
				     base, stride, &adbe->boot.start);

	ret = adbe_check_dart_bypass(adbe, base & ~ADBE_ADDR_TAG,
				     adbe->boot.start);
	if (ret)
		return ret;

	adbe->boot_base = base;
	adbe->boot_stride = stride;
	adbe->cur_base = base;
	adbe->bus_offset = adbe->boot.start - (base & ~ADBE_ADDR_TAG);
	adbe->saved_inten = readl(adbe->regs + ADP_INTEN);

	return 0;
}

static int adbe_modeset_init(struct adbe *adbe)
{
	struct drm_device *drm = &adbe->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 32;
	drm->mode_config.min_height = 32;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	/* Write-combined buffers: CPU readback is slow. */
	drm->mode_config.prefer_shadow = 1;
	drm->mode_config.funcs = &adbe_mode_config_funcs;
	drm->mode_config.helper_private = &adbe_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &adbe->plane, BIT(0),
				       &adbe_plane_funcs, adbe_formats,
				       ARRAY_SIZE(adbe_formats), adbe_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&adbe->plane, &adbe_plane_helper_funcs);

	ret = drmm_crtc_init_with_planes(drm, &adbe->crtc, &adbe->plane, NULL,
					 &adbe_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&adbe->crtc, &adbe_crtc_helper_funcs);

	ret = drmm_encoder_init(drm, &adbe->encoder, NULL,
				DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	adbe->encoder.possible_crtcs = drm_crtc_mask(&adbe->crtc);

	ret = drmm_connector_init(drm, &adbe->connector, &adbe_connector_funcs,
				  DRM_MODE_CONNECTOR_eDP, NULL);
	if (ret)
		return ret;
	drm_connector_helper_add(&adbe->connector, &adbe_connector_helper_funcs);
	ret = drm_connector_attach_encoder(&adbe->connector, &adbe->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);

	return 0;
}

static struct adbe *aux_to_adbe(struct drm_dp_aux *aux)
{
	return container_of(aux, struct adbe, aux);
}

/*
 * One native AUX transaction, the transfer hook of the shared DP helpers,
 * which do the retries. The request nibble is the DisplayPort request code.
 */
static ssize_t adbe_aux_transfer(struct drm_dp_aux *aux,
				 struct drm_dp_aux_msg *msg)
{
	struct adbe *adbe = aux_to_adbe(aux);
	void __iomem *dp = adbe->dp;
	size_t i, len = msg->size;
	u8 *buf = msg->buffer;
	u32 ctl, mask, st, status;
	int ret;

	if (msg->request != DP_AUX_NATIVE_READ &&
	    msg->request != DP_AUX_NATIVE_WRITE)
		return -EOPNOTSUPP;
	if (!len || len > DP_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	writel(len - 1, dp + DPTX_AUX_LEN);
	writel(msg->address & 0xff, dp + DPTX_AUX_ADDR_7_0);
	writel((msg->address >> 8) & 0xff, dp + DPTX_AUX_ADDR_15_8);
	writel(FIELD_PREP(DPTX_AUX_REQUEST, msg->request) |
	       ((msg->address >> 16) & 0xf), dp + DPTX_AUX_ADDR_19_16);
	ctl = readl(dp + DPTX_AUX_CTL_2) & ~(DPTX_AUX_ADDR_ONLY | DPTX_AUX_EN);
	writel(ctl, dp + DPTX_AUX_CTL_2);
	writel(DPTX_AUX_BUF_CLR, dp + DPTX_AUX_BUF_CTL);
	if (msg->request == DP_AUX_NATIVE_WRITE)
		for (i = 0; i < len; i++)
			writel(buf[i], dp + DPTX_AUX_DATA(i));

	writel(DPTX_INT_RPLY | DPTX_INT_AUX_ERR, dp + DPTX_INT_STA);
	mask = readl(dp + DPTX_INT_STA_MASK);
	writel(mask | DPTX_INT_RPLY | DPTX_INT_AUX_ERR, dp + DPTX_INT_STA_MASK);
	writel(ctl | DPTX_AUX_EN, dp + DPTX_AUX_CTL_2);
	ret = readl_poll_timeout(dp + DPTX_INT_STA, st,
				 st & (DPTX_INT_RPLY | DPTX_INT_AUX_ERR), 20,
				 DPTX_AUX_WAIT_US);
	writel(st & (DPTX_INT_RPLY | DPTX_INT_AUX_ERR), dp + DPTX_INT_STA);
	writel(mask, dp + DPTX_INT_STA_MASK);

	if (ret || (st & DPTX_INT_AUX_ERR)) {
		/* iOS drops AUX_EN itself after a failed transaction. */
		writel(readl(dp + DPTX_AUX_CTL_2) & ~DPTX_AUX_EN,
		       dp + DPTX_AUX_CTL_2);
		status = readl(dp + DPTX_AUX_CH_STA) & DPTX_AUX_STATUS;
		dev_dbg(adbe->drm.dev,
			"AUX %#x at %#x: %s, AUX_CH_STA %#x AUX_RX_COMM %#x\n",
			msg->request, msg->address, ret ? "no reply" : "error",
			status, readl(dp + DPTX_AUX_RX_COMM));
		if (ret || status == DPTX_AUX_STATUS_TIMEOUT)
			return -ETIMEDOUT;
		if (status == DPTX_AUX_STATUS_NACK ||
		    status == DPTX_AUX_STATUS_NACK_M) {
			msg->reply = DP_AUX_NATIVE_REPLY_NACK;
			return 0;
		}
		return -EIO;
	}

	if (msg->request == DP_AUX_NATIVE_READ) {
		len = min_t(size_t, len, FIELD_GET(DPTX_AUX_BUF_COUNT,
						   readl(dp + DPTX_AUX_BUF_CTL)));
		for (i = 0; i < len; i++)
			buf[i] = readl(dp + DPTX_AUX_DATA(i));
	}
	msg->reply = DP_AUX_NATIVE_REPLY_ACK;

	return len;
}

/*
 * The transmitter's scrambler seed mode. The sink must match it: a mismatch
 * still trains and locks, but every scrambled pixel is noise.
 */
static u8 adbe_source_assr(struct adbe *adbe)
{
	return readl(adbe->dp + DPTX_ASSR) & DPTX_ASSR_EN ?
	       DP_ALTERNATE_SCRAMBLER_RESET_ENABLE : 0;
}

static unsigned int adbe_link_lanes(struct adbe *adbe)
{
	return adbe->link_cfg[DP_LANE_COUNT_SET - DP_LINK_BW_SET] &
	       DP_LANE_COUNT_MASK;
}

/* The sink reports clock recovery, equalisation, symbol lock and alignment. */
static int adbe_link_trained(struct adbe *adbe)
{
	unsigned int lanes = adbe_link_lanes(adbe);
	u8 status[DP_LINK_STATUS_SIZE];
	int ret;

	ret = drm_dp_dpcd_read_link_status(&adbe->aux, status);
	if (ret)
		return ret;
	if (!drm_dp_clock_recovery_ok(status, lanes) ||
	    !drm_dp_channel_eq_ok(status, lanes)) {
		dev_dbg(adbe->drm.dev, "eDP lane status %*ph\n",
			DP_LINK_STATUS_SIZE, status);
		return -EIO;
	}

	return 0;
}

/*
 * Train the link again with the configuration the boot loader left, as iOS
 * does on every wake (its event log: TrainLink, 4 lanes, RBR, enhanced
 * framing, Fast, no downspread, swing 0 / pre-emphasis 0; ASSR on). The
 * sink advertises NO_AUX_HANDSHAKE_LINK_TRAINING, so the source sends TPS1
 * then TPS2 for a fixed time and the sink locks without AUX handshakes; the
 * shared DP helpers have no sequence for that, so it is done here. The
 * transmitter's own link rate, lane count and drive levels are as the boot
 * loader set them and are not touched.
 */
static int adbe_link_train(struct adbe *adbe)
{
	struct drm_dp_aux *aux = &adbe->aux;
	unsigned int lanes = adbe_link_lanes(adbe);
	const u8 *cfg = adbe->link_cfg;
	u8 tps[1 + 4] = { 0 };
	u32 ptn;
	int ret;

	ptn = readl(adbe->dp + DPTX_TRAINING_PTN);
	if (ptn & DPTX_TRAINING_PTN_MASK) {
		dev_warn(adbe->drm.dev,
			 "eDP transmitter left in training pattern %#x\n", ptn);
		ptn &= ~(DPTX_TRAINING_PTN_MASK | DPTX_TRAINING_NO_SCRAMBLE);
	}

	ret = drm_dp_link_power_up(aux, adbe->dpcd_caps[DP_DPCD_REV]);
	if (!ret)
		ret = drm_dp_dpcd_write_byte(aux, DP_EDP_CONFIGURATION_SET,
				(cfg[DP_EDP_CONFIGURATION_SET - DP_LINK_BW_SET] &
				 ~DP_ALTERNATE_SCRAMBLER_RESET_ENABLE) |
				adbe_source_assr(adbe));
	if (!ret)
		ret = drm_dp_dpcd_write_byte(aux, DP_DOWNSPREAD_CTRL,
				cfg[DP_DOWNSPREAD_CTRL - DP_LINK_BW_SET]);
	if (!ret)
		ret = drm_dp_dpcd_write_byte(aux, DP_LINK_BW_SET, cfg[0]);
	if (!ret)
		ret = drm_dp_dpcd_write_byte(aux, DP_LANE_COUNT_SET,
				cfg[DP_LANE_COUNT_SET - DP_LINK_BW_SET]);
	if (ret) {
		dev_err(adbe->drm.dev, "eDP link configuration failed: %d\n", ret);
		return ret;
	}

	writel(ptn | DPTX_TRAINING_NO_SCRAMBLE | DP_TRAINING_PATTERN_1,
	       adbe->dp + DPTX_TRAINING_PTN);
	tps[0] = DP_TRAINING_PATTERN_1 | DP_LINK_SCRAMBLING_DISABLE;
	ret = drm_dp_dpcd_write_data(aux, DP_TRAINING_PATTERN_SET, tps,
				     1 + lanes);
	usleep_range(ADBE_TPS_US, 2 * ADBE_TPS_US);
	if (!ret) {
		writel(ptn | DPTX_TRAINING_NO_SCRAMBLE | DP_TRAINING_PATTERN_2,
		       adbe->dp + DPTX_TRAINING_PTN);
		tps[0] = DP_TRAINING_PATTERN_2 | DP_LINK_SCRAMBLING_DISABLE;
		ret = drm_dp_dpcd_write_data(aux, DP_TRAINING_PATTERN_SET, tps,
					     1 + lanes);
		usleep_range(ADBE_TPS_US, 2 * ADBE_TPS_US);
	}
	/* Back to the normal (scrambled) stream whatever happened. */
	writel(ptn, adbe->dp + DPTX_TRAINING_PTN);
	if (!ret)
		ret = drm_dp_dpcd_write_byte(aux, DP_TRAINING_PATTERN_SET,
					     DP_TRAINING_PATTERN_DISABLE);
	if (!ret)
		ret = adbe_link_trained(adbe);
	if (ret)
		dev_err(adbe->drm.dev, "eDP link training failed: %d\n", ret);

	return ret;
}

/*
 * Look up the panel supply and the backlight once, before the first sleep,
 * and order system sleep: the display pipe suspends before the PMU GPIO
 * (behind I2C) and resumes after it, and the backlight resumes only after
 * the pipe has brought the panel back.
 */
static void adbe_panel_attach(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	struct device_node *np;
	struct gpio_desc *ldo;

	if (adbe->panel_ready || !adbe->dp)
		return;

	if (!adbe->lcd_ldo) {
		ldo = devm_gpiod_get(dev, "apple,lcd-ldo", GPIOD_ASIS);
		if (IS_ERR(ldo)) {
			if (PTR_ERR(ldo) == -EPROBE_DEFER)
				return;	/* the PMU is not there yet: next sleep */
			dev_warn(dev, "no panel supply (%pe): panel stays on in sleep\n",
				 ldo);
			adbe->dp = NULL;
			return;
		}
		adbe->lcd_ldo = ldo;
	}
	if (!adbe->ldo_link) {
		adbe->ldo_link = device_link_add(dev,
			gpio_device_to_device(gpiod_to_gpio_device(adbe->lcd_ldo)),
			DL_FLAG_STATELESS);
		if (!adbe->ldo_link) {
			dev_warn(dev, "cannot order sleep against the panel supply\n");
			return;
		}
	}

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np && !adbe->bl_link) {
		adbe->backlight = of_find_backlight_by_node(np);
		if (!adbe->backlight) {
			of_node_put(np);
			return;	/* not registered yet: next sleep */
		}
		adbe->bl_link = device_link_add(&adbe->backlight->dev, dev,
						DL_FLAG_STATELESS);
		if (!adbe->bl_link) {
			dev_warn(dev, "cannot order the backlight after the panel\n");
			put_device(&adbe->backlight->dev);
			adbe->backlight = NULL;
			of_node_put(np);
			return;
		}
	}
	of_node_put(np);
	adbe->panel_ready = true;
}

static void adbe_panel_detach(void *data)
{
	struct adbe *adbe = data;

	if (adbe->bl_link)
		device_link_del(adbe->bl_link);
	if (adbe->backlight)
		put_device(&adbe->backlight->dev);
	if (adbe->ldo_link)
		device_link_del(adbe->ldo_link);
}

static int adbe_pm_notify(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct adbe *adbe = container_of(nb, struct adbe, pm_nb);

	if (action == PM_SUSPEND_PREPARE)
		adbe_panel_attach(adbe);

	return NOTIFY_DONE;
}

static void adbe_pm_notify_remove(void *data)
{
	struct adbe *adbe = data;

	unregister_pm_notifier(&adbe->pm_nb);
}

/*
 * Before the supply goes off, everything the wake needs must already work
 * on the live link: AUX, the sink's capabilities and its trained link
 * configuration, the lane status check, and a supply that reads on. Any
 * miss keeps the panel on for this sleep.
 */
static const char *adbe_panel_gate(struct adbe *adbe)
{
	u8 caps[DP_RECEIVER_CAP_SIZE], cfg[ADBE_LINK_CFG_LEN];
	unsigned int lanes;

	if (adbe->panel_lost)
		return "a wake did not bring it back earlier";
	if (gpiod_get_direction(adbe->lcd_ldo) != GPIO_LINE_DIRECTION_OUT)
		return "lcd_ldo is not an output";
	if (gpiod_get_value_cansleep(adbe->lcd_ldo) != 1)
		return "lcd_ldo does not read on";
	if (readl(adbe->dp + DPTX_TRAINING_PTN) & DPTX_TRAINING_PTN_MASK)
		return "the transmitter is sending a training pattern";
	if (drm_dp_read_dpcd_caps(&adbe->aux, caps))
		return "AUX read of the receiver capabilities failed";
	if (!(caps[DP_MAX_LANE_COUNT] & DP_MAX_LANE_COUNT_MASK))
		return "the receiver capabilities read back empty";
	if (!(caps[DP_MAX_DOWNSPREAD] & DP_NO_AUX_HANDSHAKE_LINK_TRAINING))
		return "the sink needs handshake link training";
	if (adbe->dpcd_caps[DP_DPCD_REV] &&
	    memcmp(caps, adbe->dpcd_caps, sizeof(caps)))
		return "the receiver capabilities changed";
	if (drm_dp_dpcd_read_data(&adbe->aux, DP_LINK_BW_SET, cfg, sizeof(cfg)))
		return "AUX read of the link configuration failed";
	lanes = cfg[DP_LANE_COUNT_SET - DP_LINK_BW_SET] & DP_LANE_COUNT_MASK;
	if (!cfg[0] || !lanes || lanes > 4)
		return "the sink reports no trained link";
	if ((cfg[DP_EDP_CONFIGURATION_SET - DP_LINK_BW_SET] &
	     DP_ALTERNATE_SCRAMBLER_RESET_ENABLE) != adbe_source_assr(adbe))
		return "the transmitter and the sink disagree on ASSR";

	memcpy(adbe->dpcd_caps, caps, sizeof(caps));
	memcpy(adbe->link_cfg, cfg, sizeof(cfg));
	if (adbe_link_trained(adbe))
		return "the live link does not read back as trained";

	return NULL;
}

static void adbe_panel_off(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	const char *why;

	if (!adbe->dp || pm_suspend_target_state == PM_SUSPEND_ON)
		return;

	why = adbe->panel_ready ? adbe_panel_gate(adbe) :
				  "supply or backlight not found yet";
	if (why) {
		dev_dbg(dev, "panel stays on in sleep: %s\n", why);
		return;
	}

	gpiod_set_value_cansleep(adbe->lcd_ldo, 0);
	if (gpiod_get_value_cansleep(adbe->lcd_ldo) != 0) {
		gpiod_set_value_cansleep(adbe->lcd_ldo, 1);
		dev_warn(dev, "lcd_ldo did not switch off: panel stays on in sleep\n");
		return;
	}
	drm_dp_dpcd_set_powered(&adbe->aux, false);
	adbe->panel_off = true;
	adbe->panel_off_at = ktime_get_boottime();
}

/* Supply on, wait for the sink, train. Returns 0 once the link reads trained. */
static int adbe_panel_power_up(struct adbe *adbe, ktime_t *on_at)
{
	struct device *dev = adbe->drm.dev;
	u8 caps[DP_RECEIVER_CAP_SIZE];
	s64 off_ms;
	u32 v;
	int ret, err;

	off_ms = ktime_ms_delta(ktime_get_boottime(), adbe->panel_off_at);
	if (off_ms < ADBE_PANEL_OFF_MIN_MS)
		msleep(ADBE_PANEL_OFF_MIN_MS - off_ms);

	gpiod_set_value_cansleep(adbe->lcd_ldo, 1);
	*on_at = ktime_get();
	if (gpiod_get_value_cansleep(adbe->lcd_ldo) != 1) {
		dev_err(dev, "lcd_ldo does not read on\n");
		return -EIO;
	}
	drm_dp_dpcd_set_powered(&adbe->aux, true);

	msleep(ADBE_PANEL_HPD_MS);
	ret = readl_poll_timeout(adbe->dp + DPTX_SYS_CTL_3, v,
				 v & DPTX_SYS_CTL_3_HPD, 1000,
				 ADBE_PANEL_HPD_WAIT_US);
	if (ret)
		dev_dbg(dev, "no eDP hot-plug detect (SYS_CTL_3 %#x), trying AUX\n",
			v);

	/* The TCON answers AUX once it is up; iOS reads the caps first too. */
	ret = read_poll_timeout(drm_dp_read_dpcd_caps, err, !err,
				10 * USEC_PER_MSEC, ADBE_PANEL_AUX_WAIT_US,
				false, &adbe->aux, caps);
	if (ret) {
		dev_err(dev, "eDP sink does not answer AUX: %d\n", err);
		return err;
	}
	if (memcmp(caps, adbe->dpcd_caps, sizeof(caps))) {
		dev_err(dev, "eDP receiver capabilities %*ph differ from before sleep\n",
			(int)sizeof(caps), caps);
		return -EIO;
	}

	ret = adbe_link_train(adbe);
	if (ret)
		ret = adbe_link_train(adbe);

	return ret;
}

/*
 * Bring the panel back before the CRTC is restored: the pipe still shows the
 * black boot framebuffer while the link trains. A failed attempt switches
 * the supply off again and retries; after the last one the backlight is held
 * off, so the panel is never lit without a trained link.
 */
static void adbe_panel_on(struct adbe *adbe)
{
	struct device *dev = adbe->drm.dev;
	ktime_t on_at = ktime_get();
	s64 lit_ms;
	int tries, ret = -EIO;

	if (!adbe->panel_off && !adbe->panel_lost)
		return;

	for (tries = 0; tries < ADBE_PANEL_TRIES; tries++) {
		if (tries || (!adbe->panel_off && adbe->panel_lost)) {
			gpiod_set_value_cansleep(adbe->lcd_ldo, 0);
			drm_dp_dpcd_set_powered(&adbe->aux, false);
			adbe->panel_off_at = ktime_get_boottime();
		}
		ret = adbe_panel_power_up(adbe, &on_at);
		if (!ret)
			break;
	}
	adbe->panel_off = false;

	if (ret) {
		/* Leave the supply on: a later wake or a reboot may still recover. */
		gpiod_set_value_cansleep(adbe->lcd_ldo, 1);
		drm_dp_dpcd_set_powered(&adbe->aux, true);
		adbe->panel_lost = true;
		if (adbe->backlight && !adbe->bl_held) {
			backlight_disable(adbe->backlight);
			adbe->bl_held = true;
		}
		dev_err(dev, "eDP panel did not come back after %d attempts: backlight held off\n",
			ADBE_PANEL_TRIES);
		return;
	}

	/* iOS lets the backlight on no earlier than 193 ms after the supply. */
	lit_ms = ktime_ms_delta(ktime_get(), on_at);
	if (lit_ms < ADBE_PANEL_BL_MS)
		msleep(ADBE_PANEL_BL_MS - lit_ms);
	if (adbe->bl_held) {
		backlight_enable(adbe->backlight);
		adbe->bl_held = false;
	}
	adbe->panel_lost = false;
}

/*
 * Powering disp0 or dp off resets the pipe and the eDP link, which this
 * driver cannot bring back. Hold every listed domain while bound.
 */
static int adbe_power_init(struct device *dev)
{
	struct dev_pm_domain_list *pd_list;
	int ret;

	/* The driver core attaches a single domain itself. */
	if (!dev->pm_domain) {
		ret = devm_pm_domain_attach_list(dev, NULL, &pd_list);
		if (ret < 0)
			return ret;
	}
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	return pm_runtime_resume_and_get(dev);
}

static void adbe_power_put(void *data)
{
	pm_runtime_put_noidle(data);
}

static int adbe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *fb_np;
	struct adbe *adbe;
	int irq, ret;

	adbe = devm_drm_dev_alloc(dev, &adbe_drm_driver, struct adbe, drm);
	if (IS_ERR(adbe))
		return PTR_ERR(adbe);
	platform_set_drvdata(pdev, adbe);
	timer_setup(&adbe->flip_timer, adbe_flip_timeout, 0);
	spin_lock_init(&adbe->hold_lock);
	INIT_DELAYED_WORK(&adbe->hold_work, adbe_hold_work);

	adbe->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adbe->regs))
		return PTR_ERR(adbe->regs);

	/*
	 * With a panel supply, the panel goes off in system sleep and the
	 * DisplayPort transmitter's AUX channel brings it back on wake.
	 */
	if (of_property_present(dev->of_node, "apple,lcd-ldo-gpios")) {
		adbe->dp = devm_platform_ioremap_resource_byname(pdev, "dp");
		if (IS_ERR(adbe->dp)) {
			dev_warn(dev, "no DisplayPort transmitter window (%pe): panel stays on in sleep\n",
				 adbe->dp);
			adbe->dp = NULL;
		} else {
			adbe->aux.name = "adbe eDP";
			adbe->aux.dev = dev;
			adbe->aux.drm_dev = &adbe->drm;
			adbe->aux.transfer = adbe_aux_transfer;
			drm_dp_aux_init(&adbe->aux);
		}
	}

	ret = adbe_power_init(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot hold the power domains\n");
	ret = devm_add_action_or_reset(dev, adbe_power_put, dev);
	if (ret)
		return ret;

	fb_np = adbe_find_boot_fb(adbe);
	if (IS_ERR(fb_np))
		return dev_err_probe(dev, PTR_ERR(fb_np),
				     "no supported boot framebuffer\n");

	ret = adbe_adopt(adbe);
	if (ret)
		goto put_fb;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		goto put_fb;

	adbe->boot_va = devm_ioremap_wc(dev, adbe->boot.start, adbe->fb_size);
	if (!adbe->boot_va) {
		ret = -ENOMEM;
		goto put_fb;
	}

	/* Before any allocation: buffers then come from the DART's domain. */
	if (dart_scanout) {
		struct device_node *scanout;

		scanout = of_get_child_by_name(dev->of_node, "scanout");
		if (!scanout) {
			dart_scanout = false;
		} else {
			of_node_put(scanout);
			ret = adbe_dart_init(adbe);
			if (ret == -EIO)
				goto put_fb;	/* unknown DART state: leave it to simpledrm */
			if (ret)
				dev_warn(dev, "translated scanout unavailable (%d), using bypass\n",
					 ret);
			dart_scanout = !ret;
		}
	}

	ret = adbe_modeset_init(adbe);
	if (ret)
		goto put_fb;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto put_fb;
	}
	/* No packet of ours yet: a latched completion is not ours. */
	writel(ADP_ST_DONE, adbe->regs + ADP_STATUS);
	ret = devm_request_irq(dev, irq, adbe_irq, 0, dev_name(dev), adbe);
	if (ret)
		goto put_fb;

	/* From here on simpledrm is gone; the pipe still shows its buffer. */
	ret = adbe_take_boot_fb(adbe, fb_np);
	of_node_put(fb_np);
	if (ret) {
		adbe_dart_exit(adbe);
		return dev_err_probe(dev, ret,
				     "cannot take over the boot framebuffer\n");
	}

	ret = drm_dev_register(&adbe->drm, 0);
	if (ret) {
		adbe_dart_exit(adbe);
		return ret;
	}

	if (adbe->dp) {
		ret = devm_add_action(dev, adbe_panel_detach, adbe);
		if (!ret) {
			adbe->pm_nb.notifier_call = adbe_pm_notify;
			ret = register_pm_notifier(&adbe->pm_nb);
			if (!ret)
				ret = devm_add_action_or_reset(dev,
							       adbe_pm_notify_remove,
							       adbe);
		}
		if (ret) {
			dev_warn(dev, "panel stays on in sleep: %d\n", ret);
			adbe->dp = NULL;
		}
	}

	drm_client_setup(&adbe->drm, NULL);

	return 0;

put_fb:
	/* Nothing was flipped yet: the pipe is still on the boot framebuffer. */
	adbe_dart_exit(adbe);
	of_node_put(fb_np);
	return ret;
}

static void adbe_remove(struct platform_device *pdev)
{
	struct adbe *adbe = platform_get_drvdata(pdev);

	drm_dev_unplug(&adbe->drm);
	drm_atomic_helper_shutdown(&adbe->drm);
	timer_delete_sync(&adbe->flip_timer);
	if (adbe->scanout)
		adbe_park(adbe);
	adbe_dart_exit(adbe);
	writel(adbe->saved_inten, adbe->regs + ADP_INTEN);
}

static void adbe_shutdown(struct platform_device *pdev)
{
	struct adbe *adbe = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&adbe->drm);
	if (adbe->scanout) {
		timer_delete_sync(&adbe->flip_timer);
		adbe_park(adbe);
	}
	/* A kexec'd kernel then finds the pipe as iBoot leaves it. */
	adbe_dart_exit(adbe);
	writel(adbe->saved_inten, adbe->regs + ADP_INTEN);
}

static int adbe_suspend(struct device *dev)
{
	struct adbe *adbe = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_suspend(&adbe->drm);
	if (ret)
		return ret;
	/* The pipe now shows the black boot framebuffer. */
	adbe_panel_off(adbe);

	return 0;
}

static int adbe_resume(struct device *dev)
{
	struct adbe *adbe = dev_get_drvdata(dev);

	adbe_panel_on(adbe);

	return drm_mode_config_helper_resume(&adbe->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(adbe_pm_ops, adbe_suspend, adbe_resume);

static const struct of_device_id adbe_of_match[] = {
	{ .compatible = "apple,s8000-display-pipe" },
	{ }
};
MODULE_DEVICE_TABLE(of, adbe_of_match);

static struct platform_driver adbe_platform_driver = {
	.probe = adbe_probe,
	.remove = adbe_remove,
	.shutdown = adbe_shutdown,
	.driver = {
		.name = "apple-adbe",
		.of_match_table = adbe_of_match,
		.pm = pm_sleep_ptr(&adbe_pm_ops),
	},
};
module_platform_driver(adbe_platform_driver);

MODULE_DESCRIPTION("DRM driver for the Apple A9 display pipe");
MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_LICENSE("GPL");
