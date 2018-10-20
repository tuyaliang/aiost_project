/**
 * dwc3-sn.c : Support for dwc3 platform devices on Socionext platforms
 *
 * Copyright (C) 2016 Socionext Inc.
 *
 * Author: Kenji Fujikawa <fujikawa.kenji@socionext.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Inspired by dwc3-st.c.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include "core.h"
#include "io.h"

/* usb30 glue registers */
#define USB3_CKCTL		0x0
#define USB3_RCTL		0x4
#define USB3_RCV_SNS	0x8
#define USB3_FSEL		0xC
#define USB3_RFC_CTL	0x10
#define USB3_HO_IF		0x14
#define USB3_JT_ADJ		0x18
#define USB3_IDVBUSCTL	0x20

#define USB3_OVRCUR_EN  0x10000  /* bit field of over current en */

#define RAM_PD			0x0  /* offset */
#define USB30_RAM_PD	0x80 /* bit field of dwc3 */

/**
 * struct sn_dwc3 - dwc3-sn driver private structure
 * @dev:		device pointer
 * @glue_base:		ioaddr for the glue registers
 */
struct sn_dwc3 {
	struct device *dev;
	void __iomem *glue_base;
};

#ifdef CONFIG_PM_WARP
struct sn_dwc3 *warp_dwc3_data;
#endif

static inline u32 sn_dwc3_readl(void __iomem *base, u32 offset)
{
	return readl_relaxed(base + offset);
}

static inline void sn_dwc3_writel(void __iomem *base, u32 offset, u32 value)
{
	writel_relaxed(value, base + offset);
}

/**
 * sn_dwc3_init: init the controller via glue logic
 * @dwc3_data: driver private structure
 */
static void sn_dwc3_reset(struct sn_dwc3 *dwc3_data)
{
	// for now.
	int counter=0xFFFF;

	/* clock supply & reset seqence */
	/* usb30 core clock en */
	sn_dwc3_writel(dwc3_data->glue_base, USB3_CKCTL, 0x1);

	/* wait stable */
	while (1) {
		// for now. refer to dwc3's reg? */
		if (!--counter) break;
	}

	/* usb30 ref clock enable */
	sn_dwc3_writel(dwc3_data->glue_base, USB3_RFC_CTL, 0x1);

	/* reset */
	sn_dwc3_writel(dwc3_data->glue_base, USB3_RCTL, 0x101);
}

#if 0
static void sn_dwc3_mlb01_init(struct sn_dwc3 *dwc3_data)
{
	/* errata 1 */

	/* errata 2 */

	/* errata 3 */
}
#endif

extern enum usb_dr_mode sn_dr_mode;
static int hostmode = 2;

static int sn_dwc3_probe(struct platform_device *pdev)
{
	struct sn_dwc3 *dwc3_data;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *child;
	struct platform_device *child_pdev;
	int ret;
	u32 reg_val;

	if (hostmode == 1)
		sn_dr_mode = USB_DR_MODE_HOST;
	else if (hostmode == 0)
		sn_dr_mode = USB_DR_MODE_PERIPHERAL;
	else
		sn_dr_mode = USB_DR_MODE_UNKNOWN;

	dwc3_data = devm_kzalloc(dev, sizeof(*dwc3_data), GFP_KERNEL);
	if (!dwc3_data)
		return -ENOMEM;

	dwc3_data->dev = dev;

	/* SNI glue */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg-glue");
	dwc3_data->glue_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dwc3_data->glue_base))
		return PTR_ERR(dwc3_data->glue_base);

#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	warp_dwc3_data = dwc3_data;
#endif

	/* Socionext glue logic init */
	sn_dwc3_reset(dwc3_data);

	/* for dwc3 core */
	child = of_get_child_by_name(node, "dwc3");
	if (!child) {
		dev_err(&pdev->dev, "failed to find dwc3 core node\n");
		ret = -ENODEV;
		goto undo_softreset;
	}

	/* Allocate and initialize the core */
	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to add dwc3 core\n");
		goto undo_softreset;
	}

	child_pdev = of_find_device_by_node(child);
	if (!child_pdev) {
		dev_err(dev, "failed to find dwc3 core device\n");
		ret = -ENODEV;
		goto undo_softreset;
	}

	/* over current disable. device mode only */ 
	if (sn_dr_mode == USB_DR_MODE_PERIPHERAL) { /* device mode */
		reg_val = sn_dwc3_readl(dwc3_data->glue_base, USB3_JT_ADJ);
		reg_val &= ~USB3_OVRCUR_EN;
		sn_dwc3_writel(dwc3_data->glue_base, USB3_JT_ADJ, reg_val);
	}

	platform_set_drvdata(pdev, dwc3_data);

#if 0
/* for mlb01 errata */
sn_dwc3_mlb01_init(dwc3_data);
#endif

undo_softreset:
#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_put_sync(&pdev->dev);
#endif
	;

	return ret;
}


static int sn_dwc3_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);

#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM_WARP
static int sn_dwc3_warp_suspend(struct device *dev)
{
	pm_runtime_put_sync(dev);

	/* reset */
	sn_dwc3_writel(warp_dwc3_data->glue_base, USB3_RCTL, 0x0);
	/* usb30 ref clock disable */
	sn_dwc3_writel(warp_dwc3_data->glue_base, USB3_RFC_CTL, 0x0);
	/* usb30 core clock disable */
	sn_dwc3_writel(warp_dwc3_data->glue_base, USB3_CKCTL, 0x0);
	
	return 0;
}

static int sn_dwc3_warp_resume(struct device *dev)
{
// for now.
void __iomem *rampd_reg;
u32 rampd_val;

	pm_runtime_get_sync(dev);

	/* Socionext glue logic init */
	sn_dwc3_reset(warp_dwc3_data);

// set rampd reg.for now.
rampd_reg = ioremap(0x1b110040, 0x100);
rampd_val = sn_dwc3_readl(rampd_reg, RAM_PD);
rampd_val &= ~USB30_RAM_PD;
sn_dwc3_writel(rampd_reg, RAM_PD, rampd_val);
iounmap(rampd_reg);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sn_dwc3_dev_pm_ops, sn_dwc3_warp_suspend, sn_dwc3_warp_resume);
#define DEV_PM_OPS	(&sn_dwc3_dev_pm_ops)

#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_WARP */

static const struct of_device_id sn_dwc3_match[] = {
	{ .compatible = "socionext,usb3mlb01,f_usb30dr_fp" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sn_dwc3_match);

static struct platform_driver sn_dwc3_driver = {
	.probe = sn_dwc3_probe,
	.remove = sn_dwc3_remove,
	.driver = {
		.name = "usb-sn-dwc3",
		.of_match_table = sn_dwc3_match,
		.pm = DEV_PM_OPS,
	},
};

module_platform_driver(sn_dwc3_driver);

module_param(hostmode, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hostmode, "Override for host mode");

MODULE_AUTHOR("Kenji Fujikawa <fujikawa.kenji@socionext.com>");
MODULE_DESCRIPTION("DesignWare USB3 SNI Glue Layer");
MODULE_LICENSE("GPL v2");
