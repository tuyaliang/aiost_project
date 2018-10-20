/*
 * NAND Flash Controller Device Driver for DT
 *
 * Copyright © 2015, Cadence.
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
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "cadence_hpnfc.h"

struct cadence_hpnfc_dt {
    struct hpnfc_state_t hpnfc;
};

static const struct of_device_id cadence_hpnfc_dt_ids[] = {
    { .compatible = "cdns,hpnfc-dt" },
    {/* cadence */}
};

MODULE_DEVICE_TABLE(of, cadence_hpnfc_dt_ids);

static u64 cadence_hpnfc_dma_mask;

static int cadence_hpnfc_dt_probe(struct platform_device *ofdev)
{
    struct resource *res;
    struct cadence_hpnfc_dt *dt;
    struct hpnfc_state_t *hpnfc;
    int ret;
    const struct of_device_id *of_id;
    static u32 tmp;

    of_id = of_match_device(cadence_hpnfc_dt_ids, &ofdev->dev);
    if (of_id) {
        ofdev->id_entry = of_id->data;
    } else {
        pr_err("Failed to find the right device id.\n");
        return -ENOMEM;
    }

    dt = devm_kzalloc(&ofdev->dev, sizeof(*dt), GFP_KERNEL);
    if (!dt) {
        return -ENOMEM;
    }

    hpnfc = &dt->hpnfc;

    hpnfc->dev = &ofdev->dev;
    hpnfc->irq = platform_get_irq(ofdev, 0);
    if (hpnfc->irq < 0) {
        dev_err(&ofdev->dev, "no irq defined\n");
        return hpnfc->irq;
    }
    dev_info(hpnfc->dev, "IRQ: nr %d\n", hpnfc->irq );

    hpnfc->rst = devm_reset_control_get_optional(&ofdev->dev, NULL);
    ret = reset_control_deassert(hpnfc->rst); 

    if (ret) {
        return ret;
    } 

    res = platform_get_resource(ofdev, IORESOURCE_MEM, 2);
    hpnfc->exstop_nfwpx_reg = devm_ioremap_resource(hpnfc->dev, res);
    IOWR_32(hpnfc->exstop_nfwpx_reg, (uint32_t )0x00000001);

    res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
    hpnfc->reg = devm_ioremap_resource(hpnfc->dev, res);
    if (IS_ERR(hpnfc->reg)){
        dev_err(&ofdev->dev, "devm_ioremap_resource res 0 failed\n");
        return PTR_ERR(hpnfc->reg);
    }

    res = platform_get_resource(ofdev, IORESOURCE_MEM, 1);
    hpnfc->slave_dma = devm_ioremap_resource(&ofdev->dev, res);
    if (IS_ERR(hpnfc->reg)){
        dev_err(hpnfc->dev, "devm_ioremap_resource res 1 failed\n");
        return PTR_ERR(hpnfc->reg);
    }

    if (!of_property_read_u32(ofdev->dev.of_node,
                              "dma-mask", (u32 *)&cadence_hpnfc_dma_mask)){

        hpnfc->dev->dma_mask = &cadence_hpnfc_dma_mask;
    } else {
        hpnfc->dev->dma_mask = NULL;
    }

    if (!of_property_read_u32(ofdev->dev.of_node,
                              "ecc-sec-size", &tmp)){
        hpnfc->sector_size = tmp;
    } else {
        hpnfc->sector_size = 0;
        dev_warn(hpnfc->dev, "ecc-sec-size not found in device tree\n");
    }

    if (!of_property_read_u32(ofdev->dev.of_node,
                              "ecc-corr-cap", &tmp)){

        hpnfc->corr_cap = tmp;
    } else {

        hpnfc->corr_cap = 0;
        dev_warn(hpnfc->dev, "ecc-corr-cap not found in device tree\n");
    }

    ret = cadence_hpnfc_init(hpnfc);
    if (ret) {
        return ret;
    }

    platform_set_drvdata(ofdev, dt);

    return 0;
}

static int cadence_hpnfc_dt_remove(struct platform_device *ofdev)
{
    struct cadence_hpnfc_dt *dt = platform_get_drvdata(ofdev);

    cadence_hpnfc_remove(&dt->hpnfc);

    return 0;
}

static int cadence_hpnfc_dt_suspend(struct device *dev)
{
	struct cadence_hpnfc_dt *dt = dev_get_drvdata(dev);

	cadence_hpnfc_suspend(&dt->hpnfc);
	pm_runtime_put_sync(dev);

	return 0;
}

static int cadence_hpnfc_dt_resume(struct device *dev)
{
	struct cadence_hpnfc_dt *dt = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	cadence_hpnfc_resume(&dt->hpnfc);

	return 0;
}

static SIMPLE_DEV_PM_OPS(cadence_hpnfc_dt_pm_ops, cadence_hpnfc_dt_suspend, cadence_hpnfc_dt_resume);


static struct platform_driver cadence_hpnfc_dt_driver = {
    .probe          = cadence_hpnfc_dt_probe,
    .remove         = cadence_hpnfc_dt_remove,
    .driver         = {
        .name   = "cdns-hpnfc-dt",
		.owner	= THIS_MODULE,
        .of_match_table = cadence_hpnfc_dt_ids,
		.pm	= &cadence_hpnfc_dt_pm_ops,
    },
};

module_platform_driver(cadence_hpnfc_dt_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cadence");
MODULE_DESCRIPTION("DT driver for Cadence NAND flash controller");

