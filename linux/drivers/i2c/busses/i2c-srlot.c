/*
 * Socionext SRLOT controller driver
 *
 * Copyright (C) 2015 Socionext Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SRLCTL		0x0
#define SCKSL		BIT(16)
#define MSBSL		BIT(21)
#define SCSI		BIT(23)
#define TRGSEL		BIT(24)
#define SRLTRG_MASK	0x3
#define SRLTRG_SHFT	30
#define SRLDATA0 	0x4
#define SRLDATA(n) 	(SRLDATA0 + (n) * 4)
#define SRLCNT		0x18
#define SRLCNT_MASK	0x1f
#define SRLCNT_SHFT	0x0
#define SRLST		0x1c
#define SRF		BIT(31)

struct srlot_i2c_dev {
	struct device *dev;
	struct i2c_adapter adapter;
	struct clk *clk, *senclk;
	void __iomem *base;
};

static int srlot_i2c_xfer_msg(struct srlot_i2c_dev *srlot, struct i2c_msg *msg,
			       int last_msg)
{
	u32 cnt, val, n, avail;
	int len, total = msg->len;
	const u8 *buf = msg->buf;

	if (msg->flags & I2C_M_RD)
		return -EINVAL;

	len = total;
	while (len) {
		avail = 18;
		n = 0;
		while (avail && len) {
			val = 0;
			val |= (buf[total - len] << 24);
			len--; avail--;
			if (!len || !avail)
				goto wr_data;
			val |= (buf[total - len] << 16);
			len--; avail--;
			if (!len || !avail)
				goto wr_data;
			val |= (buf[total - len] << 8);
			len--; avail--;
			if (!len || !avail)
				goto wr_data;
			val |= (buf[total - len] << 0);
			len--; avail--;
wr_data:
			writel_relaxed(val, srlot->base + SRLDATA(n++));
		}

		writel(18 - avail, srlot->base + SRLCNT);

		pr_debug("wrote %d\n", 18 - avail);

		cnt = 1000;
		do {
			cpu_relax();
			val = readl(srlot->base + SRLST);
		} while ((val & SRF) && --cnt);

		if (!cnt)
			pr_err("%s: srlot timeout!\n", __func__);

	}

	return 0;
}

static int srlot_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num)
{
	int i, ret;
	struct srlot_i2c_dev *srlot = i2c_get_adapdata(adap);

	for (i = 0; i < num; i++) {
		ret = srlot_i2c_xfer_msg(srlot, &msgs[i], i == num - 1);
		if (ret != 0)
			return ret;
	}

	return num;
}

static u32 srlot_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm srlot_i2c_algo = {
	.master_xfer = srlot_i2c_xfer,
	.functionality = srlot_i2c_functionality,
};

static int srlot_i2c_probe(struct platform_device *pdev)
{
	struct srlot_i2c_dev *srlot;
	struct resource *res;
	int err = 0;
	u32 val;

	srlot = devm_kzalloc(&pdev->dev, sizeof(*srlot), GFP_KERNEL);
	if (!srlot)
		return -ENOMEM;

	srlot->clk = devm_clk_get(&pdev->dev, "senap");
	if (IS_ERR(srlot->clk)) {
		dev_err(&pdev->dev, "Unable to request clock\n");
		return PTR_ERR(srlot->clk);
	}

	err = clk_prepare_enable(srlot->clk);
	if (err)
		return err;

	srlot->senclk = devm_clk_get(&pdev->dev, "senclk");
	if (IS_ERR(srlot->senclk)) {
		dev_err(&pdev->dev, "Unable to request clock\n");
		return PTR_ERR(srlot->clk);
	}

	err = clk_prepare_enable(srlot->senclk);
	if (err)
		return err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	srlot->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(srlot->base))
		return PTR_ERR(srlot->base);

	/* stop ourselves */
	val = readl_relaxed(srlot->base + SRLCTL);
	val &= ~(SRLTRG_MASK << SRLTRG_SHFT);
	writel_relaxed(val, srlot->base + SRLCTL);

	/* set manual trigger */
	val = readl_relaxed(srlot->base + SRLCTL);
	val &= ~TRGSEL;
	writel_relaxed(val, srlot->base + SRLCTL);

	/* init controller */
	val = readl_relaxed(srlot->base + SRLCTL);
	val &= ~(SRLTRG_MASK << SRLTRG_SHFT);
	val |= 1 << SRLTRG_SHFT;
	writel_relaxed(val, srlot->base + SRLCTL);

	/* configure params */
	val = readl_relaxed(srlot->base + SRLCTL);
	val &= ~SCSI;
	val &= ~MSBSL;
	writel_relaxed(val, srlot->base + SRLCTL);

	srlot->adapter.dev.parent = &pdev->dev;
	srlot->adapter.algo = &srlot_i2c_algo;
	srlot->adapter.dev.of_node = pdev->dev.of_node;
	srlot->dev = &pdev->dev;

	snprintf(srlot->adapter.name, sizeof(srlot->adapter.name), "srlot-i2c");
	i2c_set_adapdata(&srlot->adapter, srlot);

	err = i2c_add_adapter(&srlot->adapter);
	if (err) {
		dev_err(&pdev->dev, "failed to add I2C adapter!\n");
		return err;
	}

	platform_set_drvdata(pdev, srlot);
	dev_info(&pdev->dev, "I2C bus:%d added\n", srlot->adapter.nr);

	return 0;
}

static int srlot_i2c_remove(struct platform_device *pdev)
{
	struct srlot_i2c_dev *srlot = platform_get_drvdata(pdev);

	i2c_del_adapter(&srlot->adapter);
	return 0;
}

static const struct of_device_id srlot_i2c_of_match[] = {
	{ .compatible = "socionext,m8m-srlot", },
	{ /* sentinel */ },
};

static struct platform_driver srlot_i2c_driver = {
	.probe = srlot_i2c_probe,
	.remove = srlot_i2c_remove,
	.driver = {
		.name = "srlot-i2c",
		.of_match_table = srlot_i2c_of_match,
	},
};
module_platform_driver(srlot_i2c_driver);

MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
MODULE_DESCRIPTION("Socionext SRLOT SPI driver");
MODULE_LICENSE("GPL v2");
