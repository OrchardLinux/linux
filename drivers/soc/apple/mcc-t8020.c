// SPDX-License-Identifier: GPL-2.0-only
/*
 * System level cache of the A12's memory controller (MCC).
 *
 * iBoot hands over with the cache off; iOS (AppleT8020MemCacheController in
 * the iOS 17.7 kernelcache for iPhone11,8) turns it on when its driver starts,
 * with everything else running: broadcast on (non-plane block +0xc), the way
 * count from the ADT's config-data written once to the plane block (+0x1c00),
 * broadcast off, then each plane's +0x1c04 is polled until its data and tag
 * way counts show. Before a restart iOS sets the ways to 0 again, so the boot
 * loader finds the cache off as it expects.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define MCC_BROADCAST		0x0c
#define MCC_PLANE_WAYS		0x1c00
#define MCC_PLANE_STATUS	0x1c04
#define  STATUS_DATA_WAYS	GENMASK(14, 10)
#define  STATUS_TAG_WAYS	GENMASK(9, 5)
#define MCC_PLANES		4
#define MCC_PLANE_STRIDE	0x40000

struct apple_mcc {
	struct device *dev;
	void __iomem *global;
	void __iomem *plane;
	u32 ways;
};

static int apple_mcc_set_ways(struct apple_mcc *mcc, u32 ways)
{
	int i, ret = 0;
	u32 st;

	writel(1, mcc->global + MCC_BROADCAST);
	writel(ways, mcc->plane + MCC_PLANE_WAYS);
	writel(0, mcc->global + MCC_BROADCAST);

	for (i = 0; i < MCC_PLANES; i++) {
		int err = readl_poll_timeout_atomic(
			mcc->plane + i * MCC_PLANE_STRIDE + MCC_PLANE_STATUS, st,
			FIELD_GET(STATUS_DATA_WAYS, st) == ways &&
			FIELD_GET(STATUS_TAG_WAYS, st) == ways, 10, 100000);
		if (err) {
			dev_err(mcc->dev, "plane %d: %#x, want %u ways\n", i, st, ways);
			ret = err;
		}
	}
	return ret;
}

static int apple_mcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_mcc *mcc;
	int ret;

	mcc = devm_kzalloc(dev, sizeof(*mcc), GFP_KERNEL);
	if (!mcc)
		return -ENOMEM;
	mcc->dev = dev;

	mcc->global = devm_platform_ioremap_resource_byname(pdev, "global");
	if (IS_ERR(mcc->global))
		return PTR_ERR(mcc->global);
	mcc->plane = devm_platform_ioremap_resource_byname(pdev, "plane");
	if (IS_ERR(mcc->plane))
		return PTR_ERR(mcc->plane);
	if (of_property_read_u32(dev->of_node, "apple,cache-ways", &mcc->ways))
		return dev_err_probe(dev, -EINVAL, "no apple,cache-ways\n");

	platform_set_drvdata(pdev, mcc);
	ret = apple_mcc_set_ways(mcc, mcc->ways);
	if (ret)
		return ret;
	dev_info(dev, "system level cache on (%u ways)\n", mcc->ways);
	return 0;
}

static void apple_mcc_shutdown(struct platform_device *pdev)
{
	struct apple_mcc *mcc = platform_get_drvdata(pdev);

	if (mcc)
		apple_mcc_set_ways(mcc, 0);
}

static const struct of_device_id apple_mcc_of_match[] = {
	{ .compatible = "apple,t8020-mcc" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_mcc_of_match);

static struct platform_driver apple_mcc_driver = {
	.probe = apple_mcc_probe,
	.shutdown = apple_mcc_shutdown,
	.driver = {
		.name = "apple-mcc-t8020",
		.of_match_table = apple_mcc_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_mcc_driver);
