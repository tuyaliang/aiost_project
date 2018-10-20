/*
 * NAND Flash Controller Device Driver for DT
 *
 * Copyright © 2011, Picochip.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "denali.h"

struct denali_dt {
	struct denali_nand_info	denali;
	struct clk *clk[5];
	int count_clk;
};

static const struct of_device_id denali_nand_dt_ids[] = {
	{ .compatible = "denali,denali-nand-dt", .data = (void*)DENALI_ORIG },
	{ .compatible = "denali,denali-nand-dt-2011", .data = (void *)DENALI_2011 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, denali_nand_dt_ids);

static u64 denali_dma_mask;

static int denali_dt_probe(struct platform_device *ofdev)
{
	const struct of_device_id *of_id =
		of_match_device(denali_nand_dt_ids, &ofdev->dev);
	struct resource *denali_reg, *nand_data;
	struct denali_dt *dt;
	struct denali_nand_info *denali;
	int ret;
	int n;

	dt = devm_kzalloc(&ofdev->dev, sizeof(*dt), GFP_KERNEL);
	if (!dt)
		return -ENOMEM;
	denali = &dt->denali;

	denali->platform = DT;
	denali->variant = (enum denali_variants)of_id->data;
	denali->dev = &ofdev->dev;
	denali->irq = platform_get_irq(ofdev, 0);
	if (denali->irq < 0) {
		dev_err(&ofdev->dev, "no irq defined\n");
		return denali->irq;
	}

	denali_reg = platform_get_resource_byname(ofdev, IORESOURCE_MEM, "denali_reg");
	denali->flash_reg = devm_ioremap_resource(&ofdev->dev, denali_reg);
	if (IS_ERR(denali->flash_reg))
		return PTR_ERR(denali->flash_reg);

	nand_data = platform_get_resource_byname(ofdev, IORESOURCE_MEM, "nand_data");
	denali->flash_mem = devm_ioremap_resource(&ofdev->dev, nand_data);
	if (IS_ERR(denali->flash_mem))
		return PTR_ERR(denali->flash_mem);

	if (!of_property_read_u32(ofdev->dev.of_node,
		"dma-mask", (u32 *)&denali_dma_mask)) {
		denali->dev->dma_mask = &denali_dma_mask;
	} else {
		denali->dev->dma_mask = NULL;
	}

	if (of_property_read_u32(ofdev->dev.of_node,
		"ecc-page0-strength", (u32 *)&denali->ecc_page0_strength))
		denali->ecc_page0_strength = ~0; /* default to generic strength */

	while (!IS_ERR_OR_NULL(dt->clk[dt->count_clk] =
			of_clk_get(ofdev->dev.of_node, dt->count_clk))) {
		clk_prepare_enable(dt->clk[dt->count_clk]);
		dev_dbg(&ofdev->dev, "%s: clk%d: rate %ld\n", __func__,
			dt->count_clk, clk_get_rate(dt->clk[dt->count_clk]));
		dt->count_clk++;
	}

	if (!IS_ERR(dt->clk[0]))
		denali->clk_ns = 1000000000L / (clk_get_rate(dt->clk[0]) / 2);

	ret = denali_init(denali);
	if (ret)
		goto out_disable_clk;

	platform_set_drvdata(ofdev, dt);
	return 0;

out_disable_clk:
	n = 0;
	while (!IS_ERR_OR_NULL(dt->clk[n]) && n < dt->count_clk) {
		clk_disable_unprepare(dt->clk[n]);
		clk_put(dt->clk[n++]);
	}
	return ret;
}

static int denali_dt_remove(struct platform_device *ofdev)
{
	struct denali_dt *dt = platform_get_drvdata(ofdev);
	int n;

	denali_remove(&dt->denali);
	n = 0;
	while (!IS_ERR_OR_NULL(dt->clk[n]) && n < dt->count_clk) {
		clk_disable_unprepare(dt->clk[n]);
		clk_put(dt->clk[n++]);
	}

	return 0;
}

static struct platform_driver denali_dt_driver = {
	.probe		= denali_dt_probe,
	.remove		= denali_dt_remove,
	.driver		= {
		.name	= "denali-nand-dt",
		.of_match_table	= denali_nand_dt_ids,
	},
};

module_platform_driver(denali_dt_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jamie Iles");
MODULE_DESCRIPTION("DT driver for Denali NAND controller");
