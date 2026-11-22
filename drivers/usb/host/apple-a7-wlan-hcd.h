/* SPDX-License-Identifier: GPL-2.0-only */
/* Board EHCI and Wi-Fi root port. Standard kernel EHCI/DMA/USB APIs; no firmware. */
static struct usb_hcd *radio_hcd;
static struct hc_driver radio_driver;
static int (*standard_hub_control)(struct usb_hcd *, u16, u16, u16, char *, u16);
static struct device_node *usb_bus_node, *aic_node;
static struct resource *host_region;
static unsigned int host_irq;
static bool irq_owned, hcd_added, hcd_attempted, hcd_stopped, controller_restored;
static bool dma_checked;
static unsigned int n_ports;
static u32 saved_ports[HCS_N_PORTS_MAX], live_ports[HCS_N_PORTS_MAX], dma_test_address;
static int enumerate_result = -EAGAIN, rom_result = -EAGAIN;
static u32 radio_vid, radio_pid, radio_speed, rom_words[6];
static struct usb_device *radio_device;

/* Leave unused root ports unpowered and unenumerated. Hub requests still use
 * the standard EHCI implementation; only the Wi-Fi port is exposed.
 */
static int radio_hub_control(struct usb_hcd *h, u16 request, u16 value,
                            u16 index, char *buf, u16 len)
{
 unsigned int port = index & 0xff;
 if ((request == GetPortStatus || request == SetPortFeature ||
      request == ClearPortFeature) && port != board->port) {
  if (port < 1 || port > n_ports) return -EPIPE;
  if (request == GetPortStatus) {
   if (len < 4) return -EINVAL;
   memset(buf, 0, 4);
  }
  return 0;
 }
 return standard_hub_control(h, request, value, index, buf, len);
}

static int find_radio(struct usb_device *udev, void *unused)
{
 if (udev->bus != &radio_hcd->self || !udev->parent ||
     udev->parent != radio_hcd->self.root_hub || udev->portnum != board->port)
  return 0;
 if (udev->state != USB_STATE_CONFIGURED) return 0;
 radio_vid = le16_to_cpu(udev->descriptor.idVendor);
 radio_pid = le16_to_cpu(udev->descriptor.idProduct);
 radio_speed = udev->speed;
 radio_device = usb_get_dev(udev);
 return 1;
}

static int hcd_prepare(struct platform_device *consumer, void __iomem *ehci)
{
 struct irq_fwspec spec = { .param_count = 3,
  .param = {0, board->ehci_irq, IRQ_TYPE_LEVEL_HIGH} };
 struct irq_domain *domain;
 irq_hw_number_t hwirq;
 unsigned int irq_type;
 const struct bus_dma_region *ranges;
 u32 cells[6];
 void *memory;
 dma_addr_t dma;
 int ret, i;

 /* USB hub code needs a real bound controller/sysdev driver, even for
  * a temporary HCD. Never assign dev->driver by hand.
  */
 if (consumer->dev.driver != &consumer_driver.driver) return -ENODEV;
 pr_info(DRV_NAME ": phase bound controller verified\n");
 usb_bus_node = of_find_node_by_path("/soc/bus@20c000000");
 aic_node = of_find_node_by_path("/soc/interrupt-controller@20e100000");
 if (!usb_bus_node || !aic_node ||
     !of_device_is_compatible(usb_bus_node, "apple,s5l8960x-usb-complex") ||
     !of_device_is_compatible(aic_node, "apple,s5l8960x-aic")) return -ENODEV;
 if (of_property_read_u32_array(usb_bus_node, "dma-ranges", cells, 6) ||
     cells[0] || cells[1] || cells[2] != 8 || cells[3] ||
     cells[4] != 1 || cells[5] ||
     of_find_property(usb_bus_node, "iommus", NULL) ||
     of_find_property(usb_bus_node, "iommu-map", NULL)) return -EPROTO;
 /* The loader-configured remap word is independently checked by the caller.
  * No guessed DMA offset: let OF parse the exact live bus dma-ranges.
  */
 ret = of_dma_configure(&consumer->dev, usb_bus_node, true);
 if (ret) return ret;
 ranges = consumer->dev.dma_range_map;
 if (!ranges || ranges[0].cpu_start != 0x800000000ULL ||
     ranges[0].dma_start || ranges[0].size != 0x100000000ULL ||
     ranges[1].size || dev_dma_coherent(&consumer->dev) || consumer->dev.iommu)
  return -EPROTO;
 ret = dma_set_mask_and_coherent(&consumer->dev, DMA_BIT_MASK(32));
 if (ret) return ret;
 /* Allocation alone starts no DMA. Verify inverse range translation and the
  * entire page address before the HCD may allocate/submit descriptors.
  */
 memory = dma_alloc_coherent(&consumer->dev, PAGE_SIZE, &dma, GFP_KERNEL);
 if (!memory) return -ENOMEM;
 ret = (dma <= 0xffffffffULL - PAGE_SIZE + 1 &&
        dma_to_phys(&consumer->dev, dma) == 0x800000000ULL + dma) ? 0 : -EPROTO;
 dma_test_address = (u32)dma;
 dma_free_coherent(&consumer->dev, PAGE_SIZE, memory, dma);
 if (ret) return ret;
 dma_checked = true;
 pr_info(DRV_NAME ": phase DMA mapping verified\n");
 for (i = 0; i < n_ports; i++) {
  saved_ports[i] = readl(ehci + 0x54 + i * 4);
  /* No existing device, powered port, reset or OHCI transaction to disturb. */
  if (saved_ports[i] == PORT_OWNER) continue;
  if (board->measured) return -EBUSY;
  pr_warn(DRV_NAME ": %s port%d=%08x, J72 measured %08x on every port (unverified on %s)\n",
          board->name, i + 1, saved_ports[i], PORT_OWNER, board->name);
  /* Never take over an enabled or resetting port on an unmeasured board. */
  if (saved_ports[i] & (PORT_PE | PORT_RESET)) return -EBUSY;
 }
 host_region = request_mem_region(board->ehci_base, 0x10000, DRV_NAME);
 if (!host_region) return -EBUSY;
 spec.fwnode = of_fwnode_handle(aic_node);
 domain = irq_find_matching_fwspec(&spec, DOMAIN_BUS_WIRED);
 if (!domain || !domain->ops || !domain->ops->translate) return -ENODEV;
 ret = domain->ops->translate(domain, &spec, &hwirq, &irq_type);
 if (ret || irq_type != IRQ_TYPE_LEVEL_HIGH) return -EPROTO;
 host_irq = irq_find_mapping(domain, hwirq);
 if (!host_irq) {
  host_irq = irq_create_fwspec_mapping(&spec);
  if (!host_irq) return -ENXIO;
  irq_owned = true;
 }
 ehci_init_driver(&radio_driver, NULL);
 standard_hub_control = radio_driver.hub_control;
 radio_driver.hub_control = radio_hub_control;
 radio_hcd = usb_create_hcd(&radio_driver, &consumer->dev, DRV_NAME);
 if (!radio_hcd) return -ENOMEM;
 radio_hcd->regs = ehci;
 radio_hcd->rsrc_start = board->ehci_base;
 radio_hcd->rsrc_len = 0x10000;
 hcd_to_ehci(radio_hcd)->caps = ehci;
 return 0;
}

static int hcd_enumerate(void __iomem *ehci)
{
 int i;
 __le32 identity[6] = {0};
 pr_info(DRV_NAME ": phase usb_add_hcd begin\n");
 hcd_attempted = true;
 enumerate_result = usb_add_hcd(radio_hcd, host_irq, 0);
 pr_info(DRV_NAME ": phase usb_add_hcd result=%d\n", enumerate_result);
 if (enumerate_result) return enumerate_result;
 hcd_added = true;
 usb_disable_autosuspend(radio_hcd->self.root_hub);
 enumerate_result = -ETIMEDOUT;
 for (i = 0; i < 100; i++) {
  usb_for_each_dev(NULL, find_radio);
  if (radio_device) { enumerate_result = 0; break; }
  msleep(100);
 }
 for (i = 0; i < n_ports; i++) live_ports[i] = readl(ehci + 0x54 + i * 4);
 if (radio_device) {
  pr_info(DRV_NAME ": %s radio %04x:%04x speed %u on port %u\n", board->name,
          radio_vid, radio_pid, radio_speed, board->port);
  /* Exact observed owner-local HSIC identity only. IN request, no firmware,
   * RAM download, reset/config vendor command or calibration transfer.
   * usb_control_msg_recv rejects short responses rather than decoding junk.
   */
  if (radio_vid == 0x0a5c && radio_pid == board->usb_pid && radio_speed == USB_SPEED_HIGH) {
   pr_info(DRV_NAME ": phase strict ROM query\n");
   usb_lock_device(radio_device);
   rom_result = usb_control_msg_recv(radio_device, 0, 5, 0xc1, 1, 0,
                                     identity, sizeof(identity), 1000, GFP_KERNEL);
   usb_unlock_device(radio_device);
   if (!rom_result) for (i = 0; i < 6; i++) rom_words[i] = le32_to_cpu(identity[i]);
  } else {
   if (!board->measured)
    pr_warn(DRV_NAME ": %s radio is not 0a5c:%04x high speed as measured on J72 (unverified on %s); no ROM query\n",
            board->name, board->usb_pid, board->name);
   rom_result = -ENODEV;
  }
  usb_put_dev(radio_device);
  radio_device = NULL;
 }
 return enumerate_result;
}

/* Called BEFORE lowering HOST_READY, resetting radio or releasing PHY/power.
 * Halt before removing HCD/buffers. A failed halt retains the complete owner,
 * IRQ, GPIO and clocks until reboot; never free an active DMA schedule.
 */
static bool hcd_finish(void __iomem *ehci, const u32 *initial)
{
 int i;
 if (hcd_attempted) {
  pr_info(DRV_NAME ": phase HCD halt/remove\n");
  /* The standard owner updates cached RUN under its lock and cancels
   * timers. Raw command writes race the EHCI watchdog and can restart DMA.
   * No transfer buffers are freed until the independent HALT read below.
   */
  if (hcd_added) radio_driver.shutdown(radio_hcd);
  for (i = 0; i < 100; i++) {
   if (readl(ehci + 0x14) & STS_HALT) break;
   usleep_range(1000, 1500);
  }
  if (i == 100) return false;
  if (hcd_added) {
   usb_remove_hcd(radio_hcd);
   hcd_added = false;
  }
  if ((readl(ehci + 0x10) & (CMD_RUN | CMD_RESET)) ||
      !(readl(ehci + 0x14) & STS_HALT) || readl(ehci + 0x18)) return false;
  hcd_stopped = true;
  pr_info(DRV_NAME ": phase HCD stopped\n");
  /* Restore only saved RW configuration, never replay W1C status bits. */
  writel(initial[0], ehci + 0x10);
  writel(initial[3], ehci + 0x24);
  writel(initial[4], ehci + 0x28);
  writel(initial[5], ehci + 0x50);
  for (i = 0; i < n_ports; i++) writel(saved_ports[i], ehci + 0x54 + i * 4);
  controller_restored = readl(ehci + 0x10) == initial[0] &&
      readl(ehci + 0x24) == initial[3] && readl(ehci + 0x28) == initial[4] &&
      readl(ehci + 0x50) == initial[5];
  for (i = 0; i < n_ports; i++)
   controller_restored &= (readl(ehci + 0x54 + i * 4) & (PORT_OWNER | PORT_POWER | PORT_SUSPEND | PORT_RESET | PORT_PE | PORT_WKOC_E | PORT_WKDISC_E | PORT_WKCONN_E)) == saved_ports[i];
  if (!controller_restored) return false;
 }
 if (radio_hcd) { usb_put_hcd(radio_hcd); radio_hcd = NULL; }
 if (irq_owned) { irq_dispose_mapping(host_irq); irq_owned = false; }
 if (host_region) { release_mem_region(board->ehci_base, 0x10000); host_region = NULL; }
 of_node_put(aic_node); aic_node = NULL;
 of_node_put(usb_bus_node); usb_bus_node = NULL;
 return true;
}
