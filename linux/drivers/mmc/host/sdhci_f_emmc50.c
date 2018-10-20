/*
* eMMC host controller driver for Socionext SoCs
*
* Copyright (C) 2016 Socionext Inc.
*		http://www.socionext.com
*
* Author: Haraguchi Munehiro <haraguchi.munehiro@socionext.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include "sdhci-pltfm.h"

#define F_EMMC_MIN_CLOCK 40000

/* HRS register */
#define REG_HRS0     0x00
#define REG_HRS4     0x10
#define   UIS_ACK    BIT(26)
#define   UIS_RD     BIT(25)
#define   UIS_WR     BIT(24)
#define REG_HRS6     0x18
#define   ETR        BIT(15)
#define   ETV        0x00003F00
#define   EMM        0x00000007
#define    MMC_MODE_LEGACY 1
#define    MMC_MODE_HS_SDR 2
#define    MMC_MODE_HS_DDR 3
#define    MMC_MODE_HS_200 4
#define    MMC_MODE_HS_400 5
#define REG_CRS63    0xFC

/* PHY register */
#define REG_DELAY_MMC_LEGACY  0x06
#define REG_DELAY_MMC_SDR     0x07
#define REG_DELAY_MMC_DDR     0x08
#define REG_DLL_LOCK          0x09
#define   DLL_LOCK            BIT(7)
#define REG_IFM               0x0E

enum f_emmc_variants {
	F_EMMC50_VARIANT
};

struct f_emmc_priv {
	enum f_emmc_variants variant;
	struct clk *clk_iface;
	struct clk *clk;
	struct device *dev;
	void __iomem *ioaddr_hrs;
	struct resource *res_exs;
	u32 transfer_data;
	struct reset_control *rst;
};

static void f_emmc_tuning_set_phase(void __iomem *ioaddr, u8 phase)
{
	u32 hrs6;

	hrs6 = readl(ioaddr + REG_HRS6);
	hrs6 &= ~(ETR | ETV);
	hrs6 |= (0x8000 | (phase << 8));
	writel(hrs6, ioaddr + REG_HRS6);
}

/*
 * Find the middle value of the longest continuous sequence of phase numbers.
 * Note that since the phase number works like a ring (cyclic array), the
 * sequence ended at 39 (the end of array) may continue to the sequence started
 * from 0.
 */
static u32 f_emmc_tuning_find_phase(u32 *phases, u32 phase_cnt)
{
	u32 val, i, j;
	u32 top_num;          /* top number                 */
	u32 cnt_num;          /* continus phase number      */
	u32 max_top_num = 0;  /* max top number             */
	u32 max_cnt_num = 0;  /* max continus  phase number */
	u32 zero_cnt_num = 0; /* zero continus phase number */

	for (i = 0; i < phase_cnt;)
	{
		val = phases[i];

		top_num = val;
		cnt_num = 1;

		for (j = i + 1; j < phase_cnt && phases[j] == (val + 1); j++) {
			val++;
			cnt_num++;
		}

		if (cnt_num > max_cnt_num) {
			max_top_num = top_num;
			max_cnt_num = cnt_num;
		}

		if (top_num == 0)
			zero_cnt_num = cnt_num;

		i += cnt_num;
	}

	/* All tuning result is pass */
	if (max_cnt_num == 40) {
		return 20;
	}

	/* Check boundary between 39 and 0 */
	if (zero_cnt_num && val == 39) {
		zero_cnt_num += cnt_num;

		if (zero_cnt_num > max_cnt_num) {
			max_top_num = top_num;
			max_cnt_num = zero_cnt_num;
		}
	}

	val = max_top_num + (max_cnt_num - 1) / 2;
	if (val > 39)
		val -= 40;

	return val;
}

static int f_emmc_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	int tuning_seq_cnt = 3;
	u32 phase;
	u32 tuned_phases[40];
	u32 tuned_phase_cnt = 0;
	int ret;
	struct mmc_host *mmc = host->mmc;
	struct f_emmc_priv *priv = sdhci_priv(host);
	void __iomem *ioaddr = priv->ioaddr_hrs;

retry:
	phase = 0;
	do {
		/* Set the phase in delay line hw block */
		f_emmc_tuning_set_phase(ioaddr, phase);

		ret = mmc_send_tuning(mmc, opcode, NULL);

		if (!ret) {
			/* Tuning is successful at this tuning point */
			tuned_phases[tuned_phase_cnt++] = phase;
			dev_dbg(mmc_dev(mmc), "%s: Found good phase = %d\n",
				mmc_hostname(mmc), phase);
		}
	} while (++phase < ARRAY_SIZE(tuned_phases));

	if (tuned_phase_cnt) {
		phase = f_emmc_tuning_find_phase(tuned_phases, tuned_phase_cnt);

		f_emmc_tuning_set_phase(ioaddr, phase);

		dev_dbg(mmc_dev(mmc), "%s: Setting the tuning phase to %d\n",
			mmc_hostname(mmc), phase);

		ret = 0;
	}
	else {
		if (--tuning_seq_cnt)
			goto retry;
		/* Tuning failed */
		dev_dbg(mmc_dev(mmc), "%s: No tuning point found\n",
			mmc_hostname(mmc));
		ret = -EIO;
	}

	return ret;
}

static u8 f_emmc_phy_data(struct sdhci_host *host, u32 addr, u32 data, u32 wr)
{
	u32 val;
	u8 rdata = 0;
	u32 to = 100;
	u32 enable;
	struct f_emmc_priv *priv = sdhci_priv(host);

	if (wr) {
		val = (data << 8) | addr;
		enable = UIS_WR;
	}
	else {
		val = addr;
		enable = UIS_RD;
	}

	writel(val, priv->ioaddr_hrs + REG_HRS4);
	val |= enable;
	writel(val, priv->ioaddr_hrs + REG_HRS4);

	/* wait ack */
	while (--to && !(readl(priv->ioaddr_hrs + REG_HRS4) & UIS_ACK))
		udelay(1);

	if (!to)
		pr_err("%s: f_emmc: phy read failed.\n", mmc_hostname(host->mmc));

	if (!wr)
		rdata = (readl(priv->ioaddr_hrs + REG_HRS4) >> 16) & 0xFF;

	to = 100;
	val &= ~enable;
	writel(val, priv->ioaddr_hrs + REG_HRS4);

	/* check clear ack */
	while (--to && (readl(priv->ioaddr_hrs + REG_HRS4) & UIS_ACK))
		udelay(1);

	if (!to)
		pr_err("%s: f_emmc: phy read failed.\n", mmc_hostname(host->mmc));

	return rdata;
}

static void f_emmc_phy_config(struct sdhci_host *host)
{
	f_emmc_phy_data(host, REG_DELAY_MMC_LEGACY, 0x0, 1);
	f_emmc_phy_data(host, REG_DELAY_MMC_SDR, 0x0, 1);
	f_emmc_phy_data(host, REG_DELAY_MMC_DDR, 0x3, 1);
}

static void f_emmc_set_uhs_signaling(struct sdhci_host *host, unsigned int uhs)
{
	u32 data;
	struct f_emmc_priv *priv = sdhci_priv(host);

	switch (uhs){
	case MMC_TIMING_MMC_HS:
		data = MMC_MODE_HS_SDR;
		break;
	case MMC_TIMING_MMC_DDR52:
		data = MMC_MODE_HS_DDR;
		break;
	case MMC_TIMING_MMC_HS200:
		data = MMC_MODE_HS_200;
		break;
	case MMC_TIMING_MMC_HS400:
		data = MMC_MODE_HS_400;
		break;
	default:
		data = MMC_MODE_LEGACY;
	}

	data = (readl(priv->ioaddr_hrs + REG_HRS6) & ~0x7) | data;
	writel(data, priv->ioaddr_hrs + REG_HRS6);
}

static unsigned int f_emmc_get_min_clock(struct sdhci_host *host)
{
	return F_EMMC_MIN_CLOCK;
}

static int f_emmc_set_version_quirk(struct sdhci_host *host)
{
	struct f_emmc_priv *priv = sdhci_priv(host);

	return ((readl(priv->ioaddr_hrs + REG_CRS63) >> 16) & 0xFF);
}

static int f_emmc_clock_change_quirk(struct sdhci_host *host)
{
	int to = 100;
	int ret = 0;

	/* check clear ack */
	while (--to && !(f_emmc_phy_data(host, REG_DLL_LOCK, 0, 0) & DLL_LOCK))
		udelay(1);

	if (!to)
		ret = -ETIMEDOUT;

	return ret;
}

static void f_emmc_reset(struct sdhci_host *host, u8 mask)
{
	int to = 1000;

	sdhci_writew(host,
		(sdhci_readw(host, SDHCI_CLOCK_CONTROL) & ~BIT(2)) |
		BIT(0), SDHCI_CLOCK_CONTROL);
	mmiowb();

	sdhci_reset(host, mask);

	sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) |
		BIT(2) | BIT(0), SDHCI_CLOCK_CONTROL);

	while (--to && !(sdhci_readw(host, SDHCI_CLOCK_CONTROL) & BIT(1)))
		udelay(10);

	if (!to)
		pr_err("%s: clock failed to be stable.\n", mmc_hostname(host->mmc));

	f_emmc_clock_change_quirk(host);
}

inline u16 f_emmc_read_w(struct sdhci_host *host, int reg)
{
	u32 data;
	u32 shift;

	if (reg == SDHCI_TRANSFER_MODE) {
		struct f_emmc_priv *priv = sdhci_priv(host);

		return priv->transfer_data;
	}

	data = readl(host->ioaddr + (reg & 0xFFFFFFFC));
	shift = 8 * (reg & 0x2);
	data >>= shift;

	return (u16)data;
}

inline u8 f_emmc_read_b(struct sdhci_host *host, int reg)
{
	u32 data;
	u32 shift;

	data = readl(host->ioaddr + (reg & 0xFFFFFFFC));
	shift = 8 * (reg & 0x3);
	data >>= shift;

	return (u8)data;
}

inline void f_emmc_write_w(struct sdhci_host *host, u16 val, int reg)
{
	u32 addr32;
	u32 data;
	u32 shift;
	u32 mask;

	if (reg == SDHCI_TRANSFER_MODE) {
		struct f_emmc_priv *priv = sdhci_priv(host);

		priv->transfer_data = val;
		return;
	}

	addr32 = reg & 0xFFFFFFFC;

	if (reg == SDHCI_COMMAND) {

		struct f_emmc_priv *priv = sdhci_priv(host);

		data = priv->transfer_data;
		priv->transfer_data = 0;
	}
	else
		data = readl(host->ioaddr + addr32);

	shift = 8 * (reg & 0x2);
	mask = ~(0xFFFF << shift);
	data = (data & mask) | ((u32)val << shift);
	writel(data, host->ioaddr + addr32);
}

inline void f_emmc_write_b(struct sdhci_host *host, u8 val, int reg)
{
	u32 addr32;
	u32 data;
	u32 shift;
	u32 mask;

	addr32 = reg & 0xFFFFFFFC;
	data = readl(host->ioaddr + addr32);
	shift = 8 * (reg & 0x3);
	mask = ~(0xFF << shift);
	data = (data & mask) | ((u32)val << shift);
	writel(data, host->ioaddr + addr32);
}

static const struct sdhci_ops f_emmc_ops = {
	.get_min_clock           = f_emmc_get_min_clock,
	.reset                   = f_emmc_reset,
	.set_clock               = sdhci_set_clock,
	.set_bus_width           = sdhci_set_bus_width,
	.set_uhs_signaling       = f_emmc_set_uhs_signaling,
	.set_version_quirk       = f_emmc_set_version_quirk,
	.clock_change_quirk2     = f_emmc_clock_change_quirk,
	.platform_execute_tuning = f_emmc_execute_tuning,
	.read_w                  = f_emmc_read_w,
	.read_b                  = f_emmc_read_b,
	.write_w                 = f_emmc_write_w,
	.write_b                 = f_emmc_write_b,
};

static const struct of_device_id f_emmc_dt_ids[] = {
	{
		.compatible = "socionext,mlb01-emmc-5.0",
		.data       = (void *)F_EMMC50_VARIANT
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_emmc_dt_ids);

static int f_emmc_exs_init(struct sdhci_host *host)
{
	struct f_emmc_priv *priv = sdhci_priv(host);
	int ret = 0;

	ret = reset_control_deassert(priv->rst);
	if (ret) {
		dev_err(priv->dev, "unable to do Initialization\n");
		return ret;
	}

	f_emmc_phy_config(host);

	return ret;
}

static int f_emmc_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
		of_match_device(f_emmc_dt_ids, &pdev->dev);
	struct sdhci_host *host;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq, ret = 0;
	struct f_emmc_priv *priv;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "%s: no irq specified\n", __func__);
		return irq;
	}

	host = sdhci_alloc_host(dev, sizeof(struct f_emmc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	priv = sdhci_priv(host);
	priv->dev = dev;
	priv->variant = (int)of_id->data;

	host->quirks =	SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
			SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
			SDHCI_QUIRK_CLOCK_BEFORE_RESET |
			SDHCI_QUIRK_DELAY_AFTER_POWER;

	host->quirks2 = SDHCI_QUIRK2_SUPPORT_SINGLE |
			SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
			SDHCI_QUIRK2_BROKEN_SDR50_TUNING |
			SDHCI_QUIRK2_NEED_DELAY_AFTER_INT_CLK_RST;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, host);

	sdhci_get_of_property(pdev);
	host->hw_name = "f_emmc50";
	host->ops = &f_emmc_ops;
	host->irq = irq;

	priv->rst = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(dev, "unable to get Rest\n");
		return PTR_ERR(priv->rst);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "srs");
	host->ioaddr = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hrs");
	priv->ioaddr_hrs = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->ioaddr_hrs)) {
		ret = PTR_ERR(priv->ioaddr_hrs);
		goto err;
	}

	priv->clk_iface = devm_clk_get(dev, "iface");
	if (IS_ERR(priv->clk_iface)) {
		ret = PTR_ERR(priv->clk_iface);
		goto err;
	}

	ret = clk_prepare_enable(priv->clk_iface);
	if (ret)
		goto err;

	priv->clk = devm_clk_get(dev, "core");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		goto err;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto err1;

	ret = f_emmc_exs_init(host);
	if (ret)
		goto err2;

	ret = sdhci_add_host(host);
	if (ret)
		goto err2;

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err2:
	clk_disable_unprepare(priv->clk);
err1:
	clk_disable_unprepare(priv->clk_iface);
err:
	sdhci_free_host(host);
	return ret;
}

static int f_emmc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);

	sdhci_free_host(host);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int f_emmc_suspend(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

        return sdhci_suspend_host(host);
}

static int f_emmc_resume(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

#ifdef CONFIG_PM_WARP
	if (pm_device_down) {
		f_emmc_exs_init(host);
	}
#endif

        return sdhci_resume_host(host);
}
#endif

static int f_emmc_runtime_suspend(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

        return sdhci_runtime_suspend_host(host);
}

static int f_emmc_runtime_resume(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

        return sdhci_runtime_resume_host(host);
}

#ifdef CONFIG_PM
static const struct dev_pm_ops f_emmc_pmops = {
        SET_SYSTEM_SLEEP_PM_OPS(f_emmc_suspend, f_emmc_resume)
        SET_RUNTIME_PM_OPS(f_emmc_runtime_suspend,
                           f_emmc_runtime_resume, NULL)
};

#define F_EMMC_PMOPS (&f_emmc_pmops)

#else
#define F_EMMC_PMOPS NULL
#endif

static struct platform_driver f_emmc_driver = {
	.driver = {
		.name = "f_emmc50",
		.of_match_table = f_emmc_dt_ids,
		.pm	= F_EMMC_PMOPS,
	},
	.probe = f_emmc_probe,
	.remove = f_emmc_remove,
};

module_platform_driver(f_emmc_driver);

MODULE_DESCRIPTION("F_EMMC50 Card Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SOCIONEXT INC.");
MODULE_ALIAS("platform:f_emmc");
