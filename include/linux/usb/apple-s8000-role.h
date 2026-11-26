/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Apple S8000 (A9) and T8010 (A10) Lightning port role switch: the interface
 * of the Lightning accessory policy (apple_tristar_charger).
 *
 * The policy is the only user that sets the host role. It powers the
 * accessory before it identifies it, and it samples the Tristar through the
 * switch so that no role change overlaps the sample.
 */
#ifndef __LINUX_USB_APPLE_S8000_ROLE_H
#define __LINUX_USB_APPLE_S8000_ROLE_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct usb_role_switch;

#if IS_REACHABLE(CONFIG_USB_ROLES_APPLE_S8000)
int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
				 int (*sample)(void *data), void *data);
int apple_s8000_usb_role_accessory_power(struct usb_role_switch *sw, bool on);
#else
static inline int apple_s8000_usb_role_monitor(struct usb_role_switch *sw,
					       int (*sample)(void *data), void *data)
{
	return -EOPNOTSUPP;
}

static inline int apple_s8000_usb_role_accessory_power(struct usb_role_switch *sw, bool on)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* __LINUX_USB_APPLE_S8000_ROLE_H */
