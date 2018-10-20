/*
 * linux/sound/soc/m8m/jmilau8m-rstc.c - I2S IP Reset Controller driver
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * This IP has a variable number of I2S units mapped at offsets from a common
 * reset register.  This driver abstracts the reset register in a way that DT
 * alone can bind however many I2S were instantiated in the hardware to the
 * common reset register at the appropriate bit offsets.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/reset-controller.h>
#include <linux/of_address.h>

static int jmilau8m_reset_change(struct device_node *of_node, unsigned long id,
				 bool state)
{
	void __iomem *regs = of_iomap(of_node, 0);

	if (IS_ERR_OR_NULL(regs))
		return -ENODEV;

	pr_info("%s: Reset id %ld state %d\n", __func__, id, state);

	if (state)
		writel(readl(regs) | (1 << id), regs);
	else
		writel(readl(regs) & ~(1 << id), regs);

	iounmap(regs);

	return 0;
}

static int jmilau8m_reset_assert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return jmilau8m_reset_change(rcdev->of_node, id, true);
}

static int jmilau8m_reset_deassert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	return jmilau8m_reset_change(rcdev->of_node, id, false);
}


static struct reset_control_ops jmilau8m_rstc_ops = {
	.assert = jmilau8m_reset_assert,
	.deassert = jmilau8m_reset_deassert,
};

static struct reset_controller_dev jmilau8m_reset_controller = {
	.ops = &jmilau8m_rstc_ops,
	.nr_resets = 32,
};

static int jmilau8m_rstc_probe(struct platform_device *pdev)
{
	jmilau8m_reset_controller.of_node = pdev->dev.of_node;
	return reset_controller_register(&jmilau8m_reset_controller);
}

static int jmilau8m_rstc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id jmilau8m_rstc_dt_ids[] = {
	{ .compatible = "socionext,jmilau8m-rstc" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, jmilau8m_rstc_dt_ids);

static struct platform_driver jmilau8m_rstc_driver = {
	.probe = jmilau8m_rstc_probe,
	.remove = jmilau8m_rstc_remove,
	.driver = {
		.name = "jmilau8m-rstc",
		.owner = THIS_MODULE,
		.of_match_table = jmilau8m_rstc_dt_ids,
	},
};

module_platform_driver(jmilau8m_rstc_driver);

MODULE_DESCRIPTION("Socionext JMILAU8M I2S reset controller driver");
MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_LICENSE("GPL");

