// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8020 PMP (power management processor)
 *
 * The A12's PMP sits in an ASC wrapper, as the SMC does, where the A11's
 * (pmp_t8015_v1.c) had an AKF wrapper. The boot loader leaves it stopped;
 * iOS loads its firmware, a raw RTKit image that runs from IOP address 0,
 * from the kernelcache's AppleT8020PMPFirmware. Booting it is what lets the
 * SoC lower its power: on the A11 that alone saved about half a watt.
 *
 * The start follows iOS (AppleASCWrapV2, RTBuddy and ApplePMP in the iOS
 * 17.7 kernelcache for iPhone11,8): the image is copied into the PMP's SRAM,
 * the remap register in front of the ASC is set to the SRAM's address | 1 so
 * that IOP address 0 reaches it, and the CPU is started. On endpoint 0x20
 * the PMP then reports ready, and the AP answers with a 0xe400-byte shared
 * buffer: the board's DRAM configuration at 0x2000 and, at 0xe000, a table
 * of {DVA, size} for each MMIO window the PMP reaches through its DART. The
 * PMP acknowledges it, and needs nothing more to run.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/apple/rtkit.h>

#define ASC_CPU_CONTROL		0x44
#define  ASC_CPU_RUN		BIT(4)
#define RVBAR_ENABLE		BIT(0)

#define PMP_EP			0x20
#define PMP_MSG_TYPE		GENMASK_ULL(55, 52)
#define PMP_MSG_SUB		GENMASK_ULL(51, 48)
#define PMP_MSG_DATA		GENMASK_ULL(47, 0)
#define PMP_SUB_READY		0
#define PMP_SUB_SHMEM		1
#define PMP_SUB_SHMEM_ACK	2

#define PMP_SHMEM_SIZE		0xe400
#define PMP_SHMEM_DRAM		0x2000
#define PMP_SHMEM_DRAM_SIZE	0x80
#define PMP_SHMEM_PIO		0xe000

struct apple_pmp {
	struct device *dev;
	void __iomem *asc;
	void __iomem *rvbar;
	void __iomem *sram;
	phys_addr_t sram_phys;
	resource_size_t sram_size;
	const char *fw_name;
	struct apple_rtkit *rtk;
	void *shmem;
	dma_addr_t shmem_dva;
	struct work_struct ready;
};

/* Each window as iOS passes it: {DVA, size}, in the ADT's reg order. */
static int apple_pmp_fill_pio(struct apple_pmp *pmp)
{
	struct device *dev = pmp->dev;
	__le64 *table = pmp->shmem + PMP_SHMEM_PIO;
	int n, i;

	n = of_property_count_u64_elems(dev->of_node, "apple,pio-windows");
	if (n < 0)
		return 0;
	if (n % 2 || n / 2 * 16 > PMP_SHMEM_SIZE - PMP_SHMEM_PIO)
		return -EINVAL;
	for (i = 0; i < n / 2; i++) {
		u64 addr, size;
		dma_addr_t dva;

		of_property_read_u64_index(dev->of_node, "apple,pio-windows", 2 * i, &addr);
		of_property_read_u64_index(dev->of_node, "apple,pio-windows", 2 * i + 1, &size);
		if (!size)
			continue;
		dva = dma_map_resource(dev, addr, size, DMA_BIDIRECTIONAL, 0);
		if (dma_mapping_error(dev, dva))
			return -ENOMEM;
		table[2 * i] = cpu_to_le64(dva);
		table[2 * i + 1] = cpu_to_le64(size);
	}
	return 0;
}

static void apple_pmp_ready_work(struct work_struct *work)
{
	struct apple_pmp *pmp = container_of(work, struct apple_pmp, ready);
	struct device *dev = pmp->dev;
	int ret;

	pmp->shmem = dmam_alloc_coherent(dev, PMP_SHMEM_SIZE, &pmp->shmem_dva,
					 GFP_KERNEL);
	if (!pmp->shmem) {
		dev_err(dev, "no shared buffer\n");
		return;
	}
	memset(pmp->shmem, 0, PMP_SHMEM_SIZE);
	of_property_read_u8_array(dev->of_node, "apple,dram-config",
				  pmp->shmem + PMP_SHMEM_DRAM, PMP_SHMEM_DRAM_SIZE);
	ret = apple_pmp_fill_pio(pmp);
	if (ret) {
		dev_err(dev, "PMP windows not mapped: %d\n", ret);
		return;
	}
	ret = apple_rtkit_send_message(pmp->rtk, PMP_EP,
				       FIELD_PREP(PMP_MSG_SUB, PMP_SUB_SHMEM) |
				       FIELD_PREP(PMP_MSG_DATA, pmp->shmem_dva),
				       NULL, false);
	if (ret)
		dev_err(dev, "shared buffer not sent: %d\n", ret);
}

static void apple_pmp_recv(void *cookie, u8 ep, u64 msg)
{
	struct apple_pmp *pmp = cookie;

	if (ep != PMP_EP)
		return;
	if (FIELD_GET(PMP_MSG_TYPE, msg) == 0) {
		switch (FIELD_GET(PMP_MSG_SUB, msg)) {
		case PMP_SUB_READY:
			dev_info(pmp->dev, "PMP ready (%#llx)\n", msg);
			schedule_work(&pmp->ready);
			return;
		case PMP_SUB_SHMEM_ACK:
			dev_info(pmp->dev, "PMP took the shared buffer\n");
			return;
		}
	}
	dev_dbg(pmp->dev, "PMP message %#llx\n", msg);
}

static const struct apple_rtkit_ops apple_pmp_rtkit_ops = {
	.recv_message = apple_pmp_recv,
};

static int apple_pmp_load(struct apple_pmp *pmp)
{
	const struct firmware *fw __free(firmware) = NULL;
	int ret;

	ret = request_firmware(&fw, pmp->fw_name, pmp->dev);
	if (ret)
		return dev_err_probe(pmp->dev, ret, "no firmware %s\n", pmp->fw_name);
	if (fw->size > pmp->sram_size)
		return dev_err_probe(pmp->dev, -ENOSPC,
				     "firmware larger than SRAM: %zu\n", fw->size);

	memcpy_toio(pmp->sram, fw->data, fw->size);
	wmb();
	writeq(pmp->sram_phys | RVBAR_ENABLE, pmp->rvbar);
	writel(readl(pmp->asc + ASC_CPU_CONTROL) | ASC_CPU_RUN,
	       pmp->asc + ASC_CPU_CONTROL);
	return 0;
}

static const struct dev_pm_domain_attach_data pd_data = {
	.pd_flags = PD_FLAG_DEV_LINK_ON,
};

static int apple_pmp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *domains;
	struct apple_pmp *pmp;
	struct resource *res;
	int ret;

	ret = dev_pm_domain_attach_list(dev, &pd_data, &domains);
	if (ret < 0)
		return ret;

	pmp = devm_kzalloc(dev, sizeof(*pmp), GFP_KERNEL);
	if (!pmp)
		return -ENOMEM;
	pmp->dev = dev;
	INIT_WORK(&pmp->ready, apple_pmp_ready_work);

	pmp->rvbar = devm_platform_ioremap_resource_byname(pdev, "rvbar");
	if (IS_ERR(pmp->rvbar))
		return PTR_ERR(pmp->rvbar);

	pmp->asc = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(pmp->asc))
		return PTR_ERR(pmp->asc);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!res)
		return -EINVAL;
	pmp->sram_size = resource_size(res);
	pmp->sram_phys = res->start;
	pmp->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(pmp->sram))
		return PTR_ERR(pmp->sram);

	ret = device_property_read_string(dev, "firmware-name", &pmp->fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");

	if (readl(pmp->asc + ASC_CPU_CONTROL) & ASC_CPU_RUN)
		return dev_err_probe(dev, -EBUSY, "PMP already running\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	pmp->rtk = devm_apple_rtkit_init(dev, pmp, NULL, 0, &apple_pmp_rtkit_ops);
	if (IS_ERR(pmp->rtk))
		return dev_err_probe(dev, PTR_ERR(pmp->rtk), "RTKit init failed\n");

	ret = apple_pmp_load(pmp);
	if (ret)
		return ret;

	ret = apple_rtkit_wake(pmp->rtk);
	if (ret)
		return dev_err_probe(dev, ret, "PMP did not boot\n");
	ret = apple_rtkit_start_ep(pmp->rtk, PMP_EP);
	if (ret)
		return dev_err_probe(dev, ret, "PMP endpoint not started\n");

	dev_info(dev, "PMP running\n");
	return 0;
}

static const struct of_device_id apple_pmp_of_match[] = {
	{ .compatible = "apple,t8020-pmp" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pmp_of_match);

static struct platform_driver apple_pmp_driver = {
	.probe = apple_pmp_probe,
	.driver = {
		.name = "apple-pmp-t8020",
		.of_match_table = apple_pmp_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(apple_pmp_driver);

MODULE_DESCRIPTION("Apple T8020 PMP");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_FIRMWARE("apple/pmp-t8020.bin");
