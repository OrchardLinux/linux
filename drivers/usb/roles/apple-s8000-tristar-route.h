/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Tristar (CBTL1610) transport route and ACC pin configuration for the
 * Lightning camera adapter, digital ID 11 f0 00 00 00 00 (USB 2, UART 1,
 * power f). Pure arithmetic, no I/O.
 *
 * As iOS does for this ID:
 *  - The ID decodes to the transports {usb0, urt0} and the ACC roles
 *    {idbs, apwr}. Both pairs swap when the plug is in orientation 2.
 *  - Orientation (status register 0x0c): 0 unless bit 7, else 1 + bit 6.
 *  - dxctrl (register 0x01): the low nibble routes the first lane pair, the
 *    high nibble the second. usb0 is 1, urt0 is 2 (low) or 0x20 (high).
 *    Bit 3 / bit 7 mark a pair that differs from the default, which is USB
 *    on the oriented pair and the other pair open. The current byte is
 *    first normalised to that default; when a pair changes, an
 *    intermediate byte with that pair's override bit is written before
 *    the final one.
 *  - ACC_CTRL (register 0x02): ACC1 nibble idbs 0, urt1 1, jtag 2, apwr 3;
 *    ACC2 the same, shifted left by 4.
 *
 * For the adapter in orientation 1 this gives ACC_CTRL 0x30 and dxctrl 0xa1.
 */
#ifndef APPLE_S8000_TRISTAR_ROUTE_H
#define APPLE_S8000_TRISTAR_ROUTE_H

#include <linux/types.h>

#define TRISTAR_ROUTE_STATUS_PRESENT	0x80
#define TRISTAR_ROUTE_STATUS_ORIENT2	0x40

#define TRISTAR_ROUTE_ACC_O1		0x30	/* ACC1 idbs, ACC2 apwr */
#define TRISTAR_ROUTE_ACC_O2		0x03	/* ACC1 apwr, ACC2 idbs */
#define TRISTAR_ROUTE_DX_O1		0xa1	/* usb0 | urt0 << 4 | override of pair 2 */
#define TRISTAR_ROUTE_DX_O2		0x1a	/* urt0 | usb0 << 4 | override of pair 1 */

/**
 * struct tristar_route_plan - Tristar writes for the camera adapter
 * @orientation: 0 without a plug, else 1 or 2
 * @acc: ACC_CTRL value (register 0x02)
 * @dx_first: intermediate dxctrl value (register 0x01)
 * @dx_target: final dxctrl value
 * @dx_write: register 0x01 differs from @dx_target
 * @dx_two_steps: @dx_first is written before @dx_target
 */
struct tristar_route_plan {
	unsigned int orientation;
	u8 acc;
	u8 dx_first;
	u8 dx_target;
	bool dx_write;
	bool dx_two_steps;
};

static inline unsigned int tristar_route_orientation(u8 status)
{
	if (!(status & TRISTAR_ROUTE_STATUS_PRESENT))
		return 0;
	return status & TRISTAR_ROUTE_STATUS_ORIENT2 ? 2 : 1;
}

/* @status is register 0x0c and @dx_cur register 0x01. False without a plug. */
static inline bool tristar_route_plan_for(u8 status, u8 dx_cur,
					  struct tristar_route_plan *plan)
{
	u8 norm = dx_cur, lo, hi;

	plan->orientation = tristar_route_orientation(status);
	if (!plan->orientation)
		return false;
	plan->acc = plan->orientation == 1 ? TRISTAR_ROUTE_ACC_O1 : TRISTAR_ROUTE_ACC_O2;
	plan->dx_target = plan->orientation == 1 ? TRISTAR_ROUTE_DX_O1 : TRISTAR_ROUTE_DX_O2;
	plan->dx_write = dx_cur != plan->dx_target;
	/* A pair without its override bit is at the default: USB on the oriented pair. */
	if (plan->orientation == 1) {
		if (!(norm & 0x08))
			norm = (norm & ~0x07) | 0x01;
	} else if (!(norm & 0x80)) {
		norm = (norm & ~0x70) | 0x10;
	}
	lo = (plan->dx_target ^ norm) & 0x07 ? 0x08 : plan->dx_target & 0x0f;
	hi = (plan->dx_target ^ norm) & 0x70 ? 0x80 : plan->dx_target & 0xf0;
	plan->dx_first = lo | hi;
	plan->dx_two_steps = plan->dx_write && plan->dx_first != plan->dx_target;
	return true;
}

#endif /* APPLE_S8000_TRISTAR_ROUTE_H */
