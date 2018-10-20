/*
 * linux/drivers/mmc/host/sdhci_f_sdh30.c
 *
 * Copyright (C) 2013 - 2015 Fujitsu Semiconductor, Ltd
 *              Vincent Yang <vincent.yang@tw.fujitsu.com>
 * Copyright (C) 2015 Linaro Ltd  Andy Green <andy.green@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>

#include "sdhci-pltfm.h"

/* F_SDH30 extended Controller registers */
#define F_SDH30_AHB_CONFIG		0x100
#define  F_SDH30_AHB_BIGED		0x00000040
#define  F_SDH30_BUSLOCK_DMA		0x00000020
#define  F_SDH30_BUSLOCK_EN		0x00000010
#define  F_SDH30_SIN			0x00000008
#define  F_SDH30_AHB_INCR_16		0x00000004
#define  F_SDH30_AHB_INCR_8		0x00000002
#define  F_SDH30_AHB_INCR_4		0x00000001

#define F_SDH30_TUNING_SETTING		0x108
#define  F_SDH30_CMD_CHK_DIS		0x00010000

#define F_SDH30_IO_CONTROL2		0x114
#define  F_SDH30_CRES_O_DN		0x00080000
#define  F_SDH30_MSEL_O_1_8		0x00040000

#define F_SDH30_ESD_CONTROL		0x124
#define  F_SDH30_CMD_DAT_DELAY		0x00000200
#define  F_SDH30_EMMC_HS200		0x01000000


#define F_SDH30_MIN_CLOCK		400000

#define IO_SDIO_BCLKSEL(adr)		((void __iomem *)((adr) + 0x01E0))
#define IO_SDIO_CS_PROT_CONTROL(adr) ((void __iomem *)((adr) + 0x01E4))
#define IO_SDIO_WP_CD_LED_SET(adr)    ((void __iomem *)(adr + 0x0210))
#define IO_SDIO_SOFT_RESET(adr)  ((void __iomem *)((adr) + 0x0200))
#define IO_SDIO_CR_SET(adr)      ((void __iomem *)((adr) + 0x0220))
#define  SDIO_CR_SET_CR_TOCLKUNIT       BIT(24)
#define  SDIO_CR_SET_CR_TOCLKFREQ_SFT   (16)
#define  SDIO_CR_SET_CR_TOCLKFREQ_MASK  (0x3F << SDIO_CR_SET_CR_TOCLKFREQ_SFT)
#define  SDIO_CR_SET_CR_BCLKFREQ_SFT    (8)
#define  SDIO_CR_SET_CR_BCLKFREQ_MASK   (0xFF << SDIO_CR_SET_CR_BCLKFREQ_SFT)
#define  SDIO_CR_SET_CR_RTUNTIMER_SFT   (4)
#define IO_SDIO_CDR_SET(adr)      ((void __iomem *)((adr) + 0x0230))

/* Extended Controller registers for UHS-II */
#define F_SDH40_MIN_CLOCK	26000000
#define IO_SDIO_CR_SET2(adr)	((void __iomem *)((adr)+0x0224))
#define  SDIO_CR_SET2_CR_18VDD2SUP	(0x1 << 28)
#define  SDIO_CR_SET2_CR_UHS2GAP	(0x1 << 24)
#define  SDIO_CR_SET2_CR_UHS2DAP	(0x1 << 20)
#define  SDIO_CR_SET2_CR_UHS2MAXBLEN	(0x200 << 8)
#define  SDIO_CR_SET2_CR_UHS2NFCU	0x04
#define IO_SDIO_PV_SET5(adr)      ((void __iomem *)((adr) + 0x0250))

/* F_SDH40 extended Controller registers */
#define HRS32(adr)         ((void __iomem *)((adr)+0x0060))
#define  HRS32_LOCK        (BIT(2))
#define HRS38(adr)         ((void __iomem *)((adr)+0x0078))
#define  HRS38_UHS2PC      (BIT(0))
#define HRS39(adr)         ((void __iomem *)((adr)+0x007C))
#define  HRS39_SDBCLKSTP   (0x1)
#define HRS45(adr)         ((void __iomem *)((adr)+0x0094))
#define  HRS45_MASK        (0xFFFF008E)
#define  HRS45_CT_CT       (0x40 << 8)
#define  HRS45_RCLK2EN_TST (0x1 << 6)
#define  HRS45_CT_NRST     (0x1 << 4)
#define  HRS45_CTSEL       (0x1)
#define CRS06(adr)         ((void __iomem *)((adr)+0x00FC))

enum f_sdh30_variants {
	FSDH30_VARIANT_MB86S7X,
	FSDH30_VARIANT_M8M,
	FSDH30_VARIANT_MLB01_SD30,
	FSDH30_VARIANT_MLB01_SD40
};

struct f_sdhost_priv {
	enum f_sdh30_variants variant;
	struct clk *clk_iface;
	struct clk *clk;
	u32 vendor_hs200;
	struct device *dev;
	unsigned enabled;
	int reset_gpio;
	u16 transfer_data;
	u32 sd40_macro;
	u32 sd40_exe;
	u32 sd40_enable;
	u32 irq_sd30;
	u32 irq_sd40;
	void __iomem *ioaddr_sd30;
	void __iomem *ioaddr_sd40;
	void __iomem *ioaddr_sd40_sn;
	u32 actual_clk30;
	u32 actual_clk40;
};

static void sdhci_f_sdh40_set_clock(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);

	sdhci_writew(host, sdhci_readw(host, SDHCI_HOST_CONTROL2) |
		SDHCI_CTRL_UHS2_INTERFACE_EN, SDHCI_HOST_CONTROL2);
	udelay(100);

	writel(HRS38_UHS2PC, HRS38(priv->ioaddr_sd40_sn));
}

static int sdhci_f_sdh40_set_version_quirk(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	int reg;

	if (priv->sd40_exe)
		reg = readl(CRS06(priv->ioaddr_sd40_sn)) >> 16;
	else
		reg = sdhci_readw(host, SDHCI_HOST_VERSION);

	return reg;
}

static int sdhci_f_sdh30_clock_change_quirk(struct sdhci_host *host)
{
	int to = 100;
	int ret = 0;

	writel(BIT(16) | BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));

	/* If we need to wait, it's nonzero now (some IPs don't need wait) */
	if (!readl(IO_SDIO_BCLKSEL(host->ioaddr)))
		return ret;

	/* wait for 1ms it to become ready */
	while (--to && !(readl(IO_SDIO_BCLKSEL(host->ioaddr)) & BIT(0)))
		udelay(10);

	if (!to)
		ret = -ETIMEDOUT;

	/* even if timeout, clear BIT(16) */
	writel(BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));

	return ret;
}

static void sdhci_f_sdh30_soft_voltage_switch(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	u32 ctrl = 0;

	usleep_range(5000, 6000);
	ctrl = sdhci_readl(host, F_SDH30_IO_CONTROL2);
	ctrl |= F_SDH30_CRES_O_DN;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);
	ctrl |= F_SDH30_MSEL_O_1_8;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);

	ctrl &= ~F_SDH30_CRES_O_DN;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);
	usleep_range(5000, 6000);

	if (priv->vendor_hs200) {
		dev_info(priv->dev, "%s: setting hs200\n", __func__);
		ctrl = sdhci_readl(host, F_SDH30_ESD_CONTROL);
		ctrl |= priv->vendor_hs200;
		sdhci_writel(host, ctrl, F_SDH30_ESD_CONTROL);
	}

	ctrl = sdhci_readl(host, F_SDH30_TUNING_SETTING);
	ctrl |= F_SDH30_CMD_CHK_DIS;
	sdhci_writel(host, ctrl, F_SDH30_TUNING_SETTING);
}

static unsigned int sdhci_f_sdh30_get_min_clock(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);

	if (priv->sd40_exe)
		return F_SDH40_MIN_CLOCK;
	else
		return F_SDH30_MIN_CLOCK;
}

static void sdhci_f_sdh30_reset(struct sdhci_host *host, u8 mask)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	int to = 1000;

	sdhci_writew(host,
		(sdhci_readw(host, SDHCI_CLOCK_CONTROL) & ~BIT(2)) |
		     BIT(0), SDHCI_CLOCK_CONTROL);
	mmiowb();

	sdhci_reset(host, mask);

	if (mask & SDHCI_RESET_ALL) {
		if (priv->sd40_exe)
			sdhci_f_sdh40_set_clock(host);
		else
			sdhci_f_sdh30_clock_change_quirk(host);
	}

	sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) |
			   BIT(2) | BIT(0), SDHCI_CLOCK_CONTROL);

	while (--to && !(sdhci_readw(host, SDHCI_CLOCK_CONTROL) & BIT(1)))
		udelay(10);

	if (!to)
		pr_err("%s: clock failed to be stable.\n",
			mmc_hostname(host->mmc));

	if (mask & SDHCI_RESET_ALL) {
		writel(BIT(16) | BIT(10) | BIT(8),
		       IO_SDIO_BCLKSEL(host->ioaddr));
		writel((0xf << 16) | BIT(8) | BIT(0),
		       IO_SDIO_CS_PROT_CONTROL(host->ioaddr));
		writel(BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));

		sdhci_writel(host, F_SDH30_CMD_DAT_DELAY, F_SDH30_ESD_CONTROL);
	}
}

inline u16 sdhci_f_sdh30_read_w(struct sdhci_host *host, int reg)
{
	u32 data;
	u32 shift;

	if (reg == SDHCI_TRANSFER_MODE || reg == SDHCI_UHS2_TRANS_MODE) {
		struct f_sdhost_priv *priv = sdhci_priv(host);

		return priv->transfer_data;
	}

	data = readl(host->ioaddr + (reg & 0xFFFFFFFC));
	shift = 8 * (reg & 0x2);
	data >>= shift;

	return (u16)data;
}

inline u8 sdhci_f_sdh30_read_b(struct sdhci_host *host, int reg)
{
	u32 data;
	u32 shift;

	data = readl(host->ioaddr + (reg & 0xFFFFFFFC));
	shift = 8 * (reg & 0x3);
	data >>= shift;

	return (u8)data;
}

inline void sdhci_f_sdh30_write_w(struct sdhci_host *host, u16 val, int reg)
{
	u32 addr32;
	u32 data;
	u32 shift;
	u32 mask;

	if (reg == SDHCI_TRANSFER_MODE || reg == SDHCI_UHS2_TRANS_MODE) {
		struct f_sdhost_priv *priv = sdhci_priv(host);

		priv->transfer_data = val;
		return;
	}

	addr32 = reg & 0xFFFFFFFC;

	if (reg == SDHCI_COMMAND || reg == SDHCI_UHS2_COMMAND) {

		struct f_sdhost_priv *priv = sdhci_priv(host);

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

inline void sdhci_f_sdh30_write_b(struct sdhci_host *host, u8 val, int reg)
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

static void sdhci_f_sdh30_vendor_spec(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	u32 val;

	/* turn the "LED" on (power the card) */
	writel(BIT(2), IO_SDIO_WP_CD_LED_SET(host->ioaddr));

	val = sdhci_readl(host, F_SDH30_IO_CONTROL2);
	val |= F_SDH30_CRES_O_DN;
	sdhci_writel(host, val, F_SDH30_IO_CONTROL2);
	val &= ~F_SDH30_MSEL_O_1_8;
	sdhci_writel(host, val, F_SDH30_IO_CONTROL2);
	val &= ~F_SDH30_CRES_O_DN;
	sdhci_writel(host, val, F_SDH30_IO_CONTROL2);

	val = sdhci_readw(host, F_SDH30_AHB_CONFIG);
	val |= F_SDH30_SIN | F_SDH30_AHB_INCR_16 | F_SDH30_AHB_INCR_8 |
		F_SDH30_AHB_INCR_4;
	val &= ~(F_SDH30_AHB_BIGED | F_SDH30_BUSLOCK_EN);
	sdhci_writew(host, val, F_SDH30_AHB_CONFIG);

	sdhci_writel(host, F_SDH30_CMD_DAT_DELAY, F_SDH30_ESD_CONTROL);

	val = sdhci_readl(host, SDHCI_CAPABILITIES);
	if (val & SDHCI_CAN_DO_8BIT)
		priv->vendor_hs200 = F_SDH30_EMMC_HS200;
}

static int sdhci_f_sdh40_uhs2_to_uhs1(struct sdhci_host *host,
					irq_handler_t fn0, irq_handler_t fn1)
{
	int val;
	int ret;
	int to;
	struct f_sdhost_priv *priv = sdhci_priv(host);
	struct mmc_host *mmc = host->mmc;

	/* PHY control */
	val = readl(HRS45(priv->ioaddr_sd40_sn)) & HRS45_MASK;
	val |= HRS45_CT_CT | HRS45_RCLK2EN_TST | HRS45_CT_NRST;
	writel(val, HRS45(priv->ioaddr_sd40_sn));
	writel(val | HRS45_CTSEL, HRS45(priv->ioaddr_sd40_sn));
	
	to = 100;
	while (--to && (readl(HRS32(priv->ioaddr_sd40_sn)) & HRS32_LOCK))
		udelay(10);

	if (!to) {
		ret = -ETIMEDOUT;
		pr_err("%s: HRS32 Lock timeout failed %d\n",
		       mmc_hostname(host->mmc), ret);
		return ret;
	}

	writel(0, HRS38(priv->ioaddr_sd40_sn));
	udelay(100); /* TODO:check rtos */

	sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
			~SDHCI_CLOCK_INT_EN, SDHCI_CLOCK_CONTROL);

	to = 100;
	while (--to && !(readl(HRS39(priv->ioaddr_sd40_sn)) & HRS39_SDBCLKSTP))
		udelay(10);
	if (!to) {
		ret = -ETIMEDOUT;
		pr_err("%s: HRS39 SDBCLKSTP timeout failed %d\n",
		       mmc_hostname(host->mmc), ret);
		return ret;
	}

	sdhci_writew(host, sdhci_readw(host, SDHCI_HOST_CONTROL2) &
			~SDHCI_CTRL_UHS2_INTERFACE_EN, SDHCI_HOST_CONTROL2);

	/* Change SD40->SD30 */
	free_irq(host->irq, host);
	host->irq = priv->irq_sd30;

	ret = request_threaded_irq(host->irq, fn0, fn1,
				   IRQF_SHARED,	mmc_hostname(mmc), host);

	if (ret) {
		pr_err("%s: Failed to request IRQ %d: %d\n",
		       mmc_hostname(mmc), host->irq, ret);
		return ret;
	}

	host->ioaddr = priv->ioaddr_sd30;
	priv->sd40_exe = 0;

	writel(0, IO_SDIO_SOFT_RESET(host->ioaddr));
	udelay(1);
	/*TODO: SET TOCLKFREQ BCLKFREQ  of uhs1*/
	writel(BIT(0), IO_SDIO_SOFT_RESET(host->ioaddr));

	/* Set clock for SD30 */
	writel(BIT(16) | BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));
	/* Clear after reset. set for UHS1 */ 
	writel((0xf << 16) | BIT(8) | BIT(0),IO_SDIO_CS_PROT_CONTROL(host->ioaddr));
	writel(BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));

	sdhci_f_sdh30_vendor_spec(host);

	mmc->caps &= ~MMC_CAP_UHS2;
	mmc->flags &= ~(MMC_UHS2_SUPPORT | MMC_UHS2_INITIALIZED);
	mmc->ios.timing = MMC_TIMING_LEGACY;
	mmc->f_min = F_SDH30_MIN_CLOCK;
	mmc->f_max = priv->actual_clk30;
	host->max_clk = mmc->f_max;

	sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);

	return 0;
}

static int sdhci_f_sdh40_uhs1_to_uhs2(struct sdhci_host *host,
				irq_handler_t fn0, irq_handler_t fn1)
{
	int ret;
	struct f_sdhost_priv *priv = sdhci_priv(host);
	struct mmc_host *mmc = host->mmc;

	if (priv->sd40_macro == 0 || priv->sd40_enable == 1)
		return 0;

	free_irq(host->irq, host);
	host->irq = priv->irq_sd40;

	ret = request_threaded_irq(host->irq, fn0, fn1,
				   IRQF_SHARED,	mmc_hostname(mmc), host);

	if (ret) {
		pr_err("%s: Failed to request IRQ %d: %d\n",
		       mmc_hostname(mmc), host->irq, ret);
		return ret;
	}

	writel(0, IO_SDIO_SOFT_RESET(host->ioaddr));
	udelay(1);
	/*TODO: SET TOCLKFREQ BCLKFREQ  of uhs2*/
	writel(BIT(16), IO_SDIO_SOFT_RESET(host->ioaddr));

	/* turn the "LED" on (power the card) */
	writel(BIT(2), IO_SDIO_WP_CD_LED_SET(host->ioaddr));

	host->ioaddr = priv->ioaddr_sd40;
	priv->sd40_exe = 1;

	sdhci_f_sdh40_set_clock(host);

	mmc->caps |= MMC_CAP_UHS2;
	mmc->flags |= MMC_UHS2_SUPPORT;
	mmc->flags &= ~MMC_UHS2_INITIALIZED;
	mmc->ios.timing = MMC_TIMING_UHS2;
	mmc->f_min = F_SDH40_MIN_CLOCK;
	mmc->f_max = priv->actual_clk40;
	host->max_clk = mmc->f_max;

	sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);

	return 0;
}

void f_sdh30_sd_power_off(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	gpio_set_value(priv->reset_gpio, 0);
	udelay(1000);
}

void f_sdh30_sd_power_on(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	gpio_set_value(priv->reset_gpio, 1);
}

static const struct sdhci_ops sdhci_f_sdh30_ops = {
	.voltage_switch = sdhci_f_sdh30_soft_voltage_switch,
	.get_min_clock = sdhci_f_sdh30_get_min_clock,
	.reset = sdhci_f_sdh30_reset,
	.set_clock = sdhci_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.clock_change_quirk = sdhci_f_sdh30_clock_change_quirk,
	.read_w = sdhci_f_sdh30_read_w,
	.read_b = sdhci_f_sdh30_read_b,
	.write_w = sdhci_f_sdh30_write_w,
	.write_b = sdhci_f_sdh30_write_b,
	.set_version_quirk = sdhci_f_sdh40_set_version_quirk,
	.uhs2_to_uhs1 = sdhci_f_sdh40_uhs2_to_uhs1,
	.uhs1_to_uhs2 = sdhci_f_sdh40_uhs1_to_uhs2,
	.sd_power_on = f_sdh30_sd_power_on,
	.sd_power_off = f_sdh30_sd_power_off,
};

static const struct of_device_id f_sdh30_dt_ids[] = {
	{
		.compatible = "fujitsu,mb86s70-sdhci-3.0",
		.data = (void *)FSDH30_VARIANT_MB86S7X
	},
	{
		.compatible = "socionext,m8m-esdhci-3.0",
		.data = (void *)FSDH30_VARIANT_M8M
	},
	{
		.compatible = "socionext,mlb01-esdhci-3.0",
		.data = (void *)FSDH30_VARIANT_MLB01_SD30
	},
	{
		.compatible = "socionext,mlb01-esdhci-4.0",
		.data = (void *)FSDH30_VARIANT_MLB01_SD40
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_sdh30_dt_ids);

static void f_sdh30_m8m_init(struct sdhci_host *host)
{
	u32 val;
	struct f_sdhost_priv *priv = sdhci_priv(host);
	int rate = clk_get_rate(priv->clk);

	writel(BIT(0), IO_SDIO_SOFT_RESET(host->ioaddr));

	if (priv->sd40_macro) {
		priv->actual_clk40 = rate;
		rate *= 4; // for SD40
		priv->actual_clk30 = rate;
		if (!priv->sd40_exe)
			rate *= 4; // for SD30
	}

	sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
		~(BIT(0) | BIT(2)), SDHCI_CLOCK_CONTROL);

	/* reset the sdhci IP */
	writel(0, IO_SDIO_SOFT_RESET(host->ioaddr));

	if (priv->reset_gpio >= 0)
		f_sdh30_sd_power_off(host);

	msleep(20); /* need for card power off duration */

	/* IO_SDIO_CR_SET should be set while reset */
	val = readl(IO_SDIO_CR_SET(host->ioaddr));
	val &= ~(SDIO_CR_SET_CR_TOCLKFREQ_MASK | SDIO_CR_SET_CR_BCLKFREQ_MASK);
	if (rate >= 16000000)
		val |= SDIO_CR_SET_CR_TOCLKUNIT | 
		       ((rate / 16 / 1000000) << SDIO_CR_SET_CR_TOCLKFREQ_SFT);
	else
		val |= (rate / 16 / 1000) << SDIO_CR_SET_CR_TOCLKFREQ_SFT;

	val |= ((rate / 4 / 1000000) << SDIO_CR_SET_CR_BCLKFREQ_SFT);
	val &= ~(0xF << SDIO_CR_SET_CR_RTUNTIMER_SFT);
	writel(val, IO_SDIO_CR_SET(host->ioaddr));
	writel(0x3, IO_SDIO_CDR_SET(host->ioaddr));

	if (priv->sd40_exe) {
		val = SDIO_CR_SET2_CR_18VDD2SUP | SDIO_CR_SET2_CR_UHS2GAP |
			SDIO_CR_SET2_CR_UHS2DAP | SDIO_CR_SET2_CR_UHS2MAXBLEN |
			SDIO_CR_SET2_CR_UHS2NFCU;
		writel(val, IO_SDIO_CR_SET2(host->ioaddr));
		writel(0x00003200, IO_SDIO_PV_SET5(host->ioaddr)); // PV_DTIMEC
		writel(BIT(16), IO_SDIO_SOFT_RESET(host->ioaddr));
	}
	else
		writel(BIT(0), IO_SDIO_SOFT_RESET(host->ioaddr));

	writel(BIT(16) | BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));
	writel(BIT(10) | BIT(8), IO_SDIO_BCLKSEL(host->ioaddr));

	sdhci_f_sdh30_vendor_spec(host);
}

static int f_sdh30_init(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct f_sdhost_priv *priv = sdhci_priv(host);
	int ret;

	ret = clk_prepare_enable(priv->clk_iface);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto err1;

	if (priv->variant)
		f_sdh30_m8m_init(host);

	if (priv->sd40_exe) {
		host->ioaddr = priv->ioaddr_sd40;
		sdhci_f_sdh40_set_clock(host);

		host->irq = priv->irq_sd40;
		host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
			SDHCI_QUIRK_MISSING_CAPS;
		host->caps = 0x21EA25B2;
		host->caps1 = 0x1000200B;
	}
	else
		host->irq = priv->irq_sd30;

	ret = sdhci_add_host(host);

	if (ret)
		goto err2;

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;
err1:
	clk_disable_unprepare(priv->clk_iface);
err2:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int f_sdh30_deinit(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct f_sdhost_priv *priv = sdhci_priv(host);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	sdhci_remove_host(host,
		readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	clk_disable_unprepare(priv->clk_iface);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct f_sdhost_priv *priv = sdhci_priv(host);
	unsigned int enable;
	int ret;

	if (kstrtouint(buf, 0, &enable) < 0)
		return -EINVAL;

	if (enable)
		enable = 1;
	else
		enable = 0;

	if (enable == priv->enabled)
		return count;

	if (enable) {
		if (of_get_property(dev->of_node, "no-init", NULL)) {
			// set GPIO
			priv->reset_gpio = of_get_named_gpio(dev->of_node,
							     "sni,mmc-power-gpio", 0);
			if (priv->reset_gpio >= 0) {
				ret = devm_gpio_request_one(dev, priv->reset_gpio,
						GPIOF_OUT_INIT_LOW, "mmc_reset");
				if (!ret)
					host->quirks2 |= SDHCI_QUIRK2_OPS_SD_POWER_CONTROL;
				else
					priv->reset_gpio = -1;
			}
		}

		ret = f_sdh30_init(dev);
	}
	else
		ret = f_sdh30_deinit(dev);

	if (ret)
		return -EINVAL;

	priv->enabled = enable;
	return count;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct f_sdhost_priv *priv = sdhci_priv(host);

	return sprintf(buf, "%d\n", priv->enabled);
}

static DEVICE_ATTR(enable, 0664, enable_show, enable_store);

static int sdhci_f_sdh30_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
		of_match_device(f_sdh30_dt_ids, &pdev->dev);
	struct sdhci_host *host;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq, ret = 0;
	struct f_sdhost_priv *priv;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "%s: no irq specified\n", __func__);
		return irq;
	}

	host = sdhci_alloc_host(dev, sizeof(struct f_sdhost_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	priv = sdhci_priv(host);
	priv->dev = dev;
	priv->variant = (int)of_id->data;

	host->quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
		       SDHCI_QUIRK_INVERTED_WRITE_PROTECT;

	if (priv->variant)
		host->quirks |=
			SDHCI_QUIRK_CLOCK_BEFORE_RESET |
			SDHCI_QUIRK_DELAY_AFTER_POWER;

	host->quirks2 = SDHCI_QUIRK2_SUPPORT_SINGLE |
			SDHCI_QUIRK2_TUNING_WORK_AROUND;

	if (of_get_property(dev->of_node, "quirk-host-off-card-on", NULL)) {
		host->quirks2 |=
		 SDHCI_QUIRK2_HOST_OFF_CARD_ON;
	}

	if (priv->variant == FSDH30_VARIANT_MLB01_SD30 ||
			priv->variant == FSDH30_VARIANT_MLB01_SD40)
		host->quirks2 |= SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
			SDHCI_QUIRK2_BROKEN_SDR50_TUNING |
			SDHCI_QUIRK2_NEED_DELAY_AFTER_INT_CLK_RST;
	else if (priv->variant)
		host->quirks2 |= SDHCI_QUIRK2_LED_ABUSED_AS_CARD_POWER |
			SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
			SDHCI_QUIRK2_BROKEN_SDR50_TUNING |
			0; //SDHCI_QUIRK2_NO_1_8_V;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, host);

	sdhci_get_of_property(pdev);
	host->hw_name = "f_sdh30";
	host->ops = &sdhci_f_sdh30_ops;
	host->irq = irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->ioaddr = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		goto err;
	}

	priv->ioaddr_sd30 = host->ioaddr;
	priv->irq_sd30    = host->irq;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uhs2");
	if (res) {
		priv->sd40_macro = 1;
		priv->ioaddr_sd40 = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->ioaddr_sd40)) {
			ret = PTR_ERR(priv->ioaddr_sd40);
			goto err;
		}

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uhs2_sn");
		priv->ioaddr_sd40_sn = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->ioaddr_sd40_sn)) {
			ret = PTR_ERR(priv->ioaddr_sd40);
			goto err;
		}

		priv->irq_sd40 = platform_get_irq_byname(pdev, "uhs2");
		if (priv->irq_sd40 < 0) {
			dev_err(dev, "%s: no irq specified for sd40\n", __func__);
			return priv->irq_sd40;
		}

		if (of_get_property(dev->of_node, "sd40_enable", NULL)) {
			priv->sd40_exe = 1;
			priv->sd40_enable = 0;
		}
		else {
			priv->sd40_enable = 1;
			priv->sd40_exe = 0;
		}
	}
	else {
		priv->sd40_macro = 0;
		priv->sd40_exe = 0;
	}

	priv->clk_iface = devm_clk_get(dev, "iface");
	if (IS_ERR(priv->clk_iface)) {
		ret = PTR_ERR(priv->clk_iface);
		goto err;
	}

	priv->clk = devm_clk_get(dev, "core");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		goto err;
	}

	ret = device_create_file(dev, &dev_attr_enable);
	if (ret) {
		dev_err(dev, "%s: sysfs_create_group failed\n", __func__);
		goto err;
	}

	if (!of_get_property(dev->of_node, "no-init", NULL)) {
		priv->reset_gpio = of_get_named_gpio(dev->of_node,
						     "sni,mmc-power-gpio", 0);
		if (priv->reset_gpio >= 0) {
			ret = devm_gpio_request_one(dev, priv->reset_gpio,
					GPIOF_OUT_INIT_LOW, "mmc_reset");
			if (!ret)
				host->quirks2 |= SDHCI_QUIRK2_OPS_SD_POWER_CONTROL;
			else
				priv->reset_gpio = -1;
		}

		dev_err(dev, "%s: starting host\n", __func__);
		f_sdh30_init(dev);
		priv->enabled = 1;
	}

	return 0;
err:
	sdhci_free_host(host);
	return ret;
}

static int sdhci_f_sdh30_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);

	sdhci_free_host(host);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_f_sdh30_suspend(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

	if (!of_get_property(dev->of_node, "no-init", NULL))
	        return sdhci_suspend_host(host);
	else
		return 0;
}

static int sdhci_f_sdh30_resume(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

	if (!of_get_property(dev->of_node, "no-init", NULL)) {
#ifdef CONFIG_PM_WARP
		if (pm_device_down) {
			struct f_sdhost_priv *priv = sdhci_priv(host);

			if (priv->variant)
				f_sdh30_m8m_init(host);
		}
#endif

	        return sdhci_resume_host(host);
	}
	else
		return 0;
}
#endif
static int sdhci_f_sdh30_runtime_suspend(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

	if (!of_get_property(dev->of_node, "no-init", NULL))
	        return sdhci_runtime_suspend_host(host);
	else
		return 0;
}
static int sdhci_f_sdh30_runtime_resume(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);

	if (!of_get_property(dev->of_node, "no-init", NULL))
	        return sdhci_runtime_resume_host(host);
	else
		return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops sdhci_f_sdh30_pmops = {
        SET_SYSTEM_SLEEP_PM_OPS(sdhci_f_sdh30_suspend, sdhci_f_sdh30_resume)
        SET_RUNTIME_PM_OPS(sdhci_f_sdh30_runtime_suspend,
                           sdhci_f_sdh30_runtime_resume, NULL)
};

#define SDHCI_F_SDH30_PMOPS (&sdhci_f_sdh30_pmops)

#else
#define SDHCI_F_SDH30_PMOPS NULL
#endif

static struct platform_driver sdhci_f_sdh30_driver = {
	.driver = {
		.name = "f_sdh30",
		.of_match_table = f_sdh30_dt_ids,
		.pm	= SDHCI_F_SDH30_PMOPS,
	},
	.probe	= sdhci_f_sdh30_probe,
	.remove	= sdhci_f_sdh30_remove,
};

module_platform_driver(sdhci_f_sdh30_driver);

MODULE_DESCRIPTION("F_SDH30 SD Card Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("FUJITSU SEMICONDUCTOR LTD.");
MODULE_ALIAS("platform:f_sdh30");
