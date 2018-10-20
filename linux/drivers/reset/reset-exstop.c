/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

enum { /* Must match with sc2000a-resets.h */
	EHCI_RESET = 0,
	USB2_UDC_RESET,
	NSEC_RESET,
	F_EMMC50_RESET,
	NAND_RESET,

	MLB01_MAX_RESETS, /* keep at last */
};

#define	SOFTRESET		0x00C
#define	RAM_PD			0x040
#define	NSEC_CKSTP		0x070
#define	USB2_CKCTL		0x200
#define	USB2_RCTL		0x204
#define	USB2_ANPD		0x208
#define	USB2_HFSEL		0x20C
#define	USB2_FSEL		0x210
#define	USB2_EHCI_SB	0x214
#define	USB2_EHCI_STRAP	0x218
#define	USB2_OHCI_IF	0x21C
#define	USB2_AHB_SLVIF	0x220
#define	USB2_LMODSET	0x224
#define	USB2_PMODSET	0x228
#define	USB2_HPT_CTL	0x22C
#define	USB2_IDVBUSCTL	0x230
#define D_RAMPD_NETSEC	BIT(5)
#define D_RAMPD_USB2	BIT(6)

struct mlb01usbh_data {
	struct reset_controller_dev rcdev;
	void __iomem *base;
};

#define to_data(p) container_of(p, struct mlb01usbh_data, rcdev)

static int mlb01usbh_restart(struct notifier_block *this, unsigned long mode,
			       void *cmd)
{
	pr_emerg("%s: unable to restart system\n", __func__);
	return NOTIFY_DONE;
}

static struct notifier_block mlb01usbh_restart_nb = {
	.notifier_call = mlb01usbh_restart,
	.priority = 192,
};

#define CLK_BASE	0x1d021000
#define PIN_BASE	0x1d022000

static int mlb01_ehci_reset(struct reset_controller_dev *rcdev)
{
	struct mlb01usbh_data *rc = to_data(rcdev);
	void __iomem *clkbase, *pinbase;
	u32 val;

	clkbase = ioremap(CLK_BASE, 0x1000);
	pinbase = ioremap(PIN_BASE, 0x1000);

	writel_relaxed(2 << 16, clkbase + 0x88); /* enable EXSAP */
	writel_relaxed(2 << 14, clkbase + 0x88); /* enable EXSAH */
	writel_relaxed(2 << 18, clkbase + 0x58); /* enable EXSAX */

	/* Wait1us */
	udelay(1);

	/* Host/Function Select(=1 is host) */
	writel_relaxed(1, rc->base + USB2_HFSEL);

	/* ID set(0) is host */
	val = readl_relaxed(rc->base + USB2_IDVBUSCTL);
	val &= ~0x1000;
	val |= 0x100;
	writel_relaxed(val, rc->base + USB2_IDVBUSCTL);

	/* PUDCR PM */
	val = readl_relaxed(pinbase + 0x42c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x42c);
	val = readl_relaxed(pinbase + 0x42c);
	val &= ~BIT(3);
	writel_relaxed(val, pinbase + 0x42c);
	val = readl_relaxed(pinbase + 0x42c);
	val &= ~BIT(4);
	writel_relaxed(val, pinbase + 0x42c);

	/* PUDER PM */
	val = readl_relaxed(pinbase + 0x32c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x32c);
	val = readl_relaxed(pinbase + 0x32c);
	val |= BIT(3);
	writel_relaxed(val, pinbase + 0x32c);
	val = readl_relaxed(pinbase + 0x32c);
	val &= ~BIT(4);
	writel_relaxed(val, pinbase + 0x32c);

	/* EPCR PM */
	val = readl_relaxed(pinbase + 0x22c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x22c);
	val = readl_relaxed(pinbase + 0x22c);
	val &= ~BIT(3);
	writel_relaxed(val, pinbase + 0x22c);

	/* DDR PM */
	val = readl_relaxed(pinbase + 0x12c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x12c);

	/* RAM PowerDown release */
	val = readl_relaxed(rc->base + RAM_PD);
	val &= ~D_RAMPD_USB2;
	writel_relaxed(val, rc->base + RAM_PD);

	/* Wait1us */
	udelay(1);

	//HSANDPCTL High Speed PHY PowerDownReset(1->0)
	writel_relaxed(0, rc->base + USB2_ANPD);

	//USB2_CKCTL Clock Run(1)
	val = readl_relaxed(rc->base + USB2_CKCTL);
	val |= 0x100;
	writel_relaxed(val, rc->base + USB2_CKCTL);

	/* Wait250ns + margin */
	udelay(1);

	writel_relaxed(4, rc->base + USB2_LMODSET);

	/* Usb UnReset */
	val = readl_relaxed(rc->base + USB2_RCTL);
	val |= 0x100;
	writel_relaxed(val, rc->base + USB2_RCTL);

	iounmap(clkbase);
	iounmap(pinbase);
	return 0;
}

/* exstop of mlb01 usb20 reg offset */
#define MLB01_USB2_CKCTL		0x200
#define MLB01_USB2_RCTL		0x204
#define MLB01_USB2_ANPD		0x208
#define MLB01_USB2_HFSEL		0x20c
#define MLB01_USB2_FSEL		0x210
#define MLB01_USB2_EHCI_SB	0x214
#define MLB01_USB2_EHCI_STRAP	0x218
#define MLB01_USB2_OHCI_IF	0x21c
#define MLB01_USB2_AHB_SLVIF	0x220
#define MLB01_USB2_LMODSET	0x224
#define MLB01_USB2_PMODSET	0x228
#define MLB01_USB2_HPT_CTL	0x22c
#define MLB01_USB2_IDVBUSCTL	0x230
#define MLB01_USB2_HDMAC1		0x238
#define MLB01_USB2_HDMAC2		0x23C
#define MLB01_USB2_DMAFSM1	0x240
#define MLB01_USB2_DMAFSM2	0x244

static int mlb01_udc2_reset(struct reset_controller_dev *rcdev)
{
	struct mlb01usbh_data *rc = to_data(rcdev);
	void __iomem *clkbase, *pinbase;
	u32 val;

	/*exstop setting for mlb01 */
	/* clock supply */
	writel_relaxed(0x0, rc->base + MLB01_USB2_ANPD);
	writel_relaxed(0x1, rc->base + MLB01_USB2_CKCTL);
	writel_relaxed(0x1, rc->base + MLB01_USB2_RCTL);

	/* dma */
	writel_relaxed(0x300, rc->base + MLB01_USB2_HDMAC1);
	writel_relaxed(0x300, rc->base + MLB01_USB2_HDMAC2);

	clkbase = ioremap(CLK_BASE, 0x1000);
	pinbase = ioremap(PIN_BASE, 0x1000);

	writel_relaxed(2 << 16, clkbase + 0x88); /* enable EXSAP */
	writel_relaxed(2 << 14, clkbase + 0x88); /* enable EXSAH */
	writel_relaxed(2 << 18, clkbase + 0x58); /* enable EXSAX */

	/* Wait1us */
	udelay(1);

	/* Host/Function Select(=1 is host) */
	writel_relaxed(0, rc->base + USB2_HFSEL);
	/* device only */
	writel_relaxed(0x01000000, rc->base + MLB01_USB2_IDVBUSCTL);

	/* PUDCR PM */
	val = readl_relaxed(pinbase + 0x42c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x42c);
	val = readl_relaxed(pinbase + 0x42c);
	val &= ~BIT(3);
	writel_relaxed(val, pinbase + 0x42c);
	val = readl_relaxed(pinbase + 0x42c);
	val &= ~BIT(4);
	writel_relaxed(val, pinbase + 0x42c);

	/* PUDER PM */
	val = readl_relaxed(pinbase + 0x32c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x32c);
	val = readl_relaxed(pinbase + 0x32c);
	val |= BIT(3);
	writel_relaxed(val, pinbase + 0x32c);
	val = readl_relaxed(pinbase + 0x32c);
	val &= ~BIT(4);
	writel_relaxed(val, pinbase + 0x32c);

	/* EPCR PM */
	val = readl_relaxed(pinbase + 0x22c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x22c);
	val = readl_relaxed(pinbase + 0x22c);
	val &= ~BIT(3);
	writel_relaxed(val, pinbase + 0x22c);

	/* DDR PM */
	val = readl_relaxed(pinbase + 0x12c);
	val |= BIT(1);
	writel_relaxed(val, pinbase + 0x12c);

	/* RAM PowerDown release */
	val = readl_relaxed(rc->base + RAM_PD);
	val &= ~D_RAMPD_USB2;
	writel_relaxed(val, rc->base + RAM_PD);

	/* Wait1us */
	udelay(1);

	//HSANDPCTL High Speed PHY PowerDownReset(1->0)
	writel_relaxed(0, rc->base + USB2_ANPD);

	//USB2_CKCTL Clock Run(1)
	val = readl_relaxed(rc->base + USB2_CKCTL);
	val |= 0x100;
	writel_relaxed(val, rc->base + USB2_CKCTL);

	/* Wait250ns + margin */
	udelay(1);

	writel_relaxed(4, rc->base + USB2_LMODSET);

	/* Usb UnReset */
	val = readl_relaxed(rc->base + USB2_RCTL);
	val |= 0x100;
	writel_relaxed(val, rc->base + USB2_RCTL);

	iounmap(clkbase);
	iounmap(pinbase);
	return 0;
}

static int mlb01_nsec_reset(struct reset_controller_dev *rcdev)
{
	struct mlb01usbh_data *rc = to_data(rcdev);
	u32 val;

	/* RAM PowerDown release */
	val = readl_relaxed(rc->base + RAM_PD);
	val &= ~D_RAMPD_NETSEC;
	writel_relaxed(val, rc->base + RAM_PD);

	/* NETSEC Bus Clock Supply */
	writel_relaxed(0, rc->base + NSEC_CKSTP);

	/* NETSEC SoftReset Deassert */
	val = readl_relaxed(rc->base + SOFTRESET);
	val |= BIT(8);
	writel_relaxed(val, rc->base + SOFTRESET);

	return 0;
}

/* TOP:clock */
/* #define CLK_BASE  0x1d021000 */
#define   CLKSEL1     0x0000
#define   EMMCCLK       (BIT(30) | BIT(29) | BIT(28))
#define   EMMCCLK_V     4
#if (EMMCCLK_V == 4)
#define MLB01_F_EMMC50_CLK 188
#elif (EMMCCLK_V == 5)
#define MLB01_F_EMMC50_CLK 167
#elif (EMMCCLK_V == 6)
#define MLB01_F_EMMC50_CLK 150
#else
#define MLB01_F_EMMC50_CLK 100
#endif
#define   CLKSTOP2    0x0058
#define     EMMCCK      (BIT(13) | BIT(12))
#define     EMMCCK_V    2


/* Other Controll */
#define OTHER_CONT_BASE 0x1D020000
#define   PUDCNT          0x0028
#define   EMMMASK         0x001F
#define   EMMMASK_V       0x001D

/* EXS:0x1B110000 */
/* #define SOFTRESET      0x000C */
#define  EMCRSTX         BIT(16)
#define  EMCRSTX_ON      0
#define  EMCRSTX_OFF     1

/* #define RAM_PD         0x0040 */
#define  RAMPD_EMMC      BIT(10)
#define BUSCKSTP       0x0060
#define  EMM_CKSTP       BIT(5)
#define EMMC_CKSTP     0x0074
#define  EMCLKSTP        BIT(0)
#define EM_DRVSTR      0x0090
#define  EM_DSEL         (BIT(1) | BIT(0))
#define EM_PV_DTIMEC   0x00B4
#define  PV_DTIMEC       0x00FFFFFF
#define  PV_DTIMEC_V     0
#define EM_PV_AMPBL    0x00B8
#define  PV_AMPBL        0xF
#define  PV_AMPBL_V      0xF
#define EM_CR_SLOTTYPE 0x00BC
#define  CR_SLOTTYPE     BIT(0)
#define  CR_SLOTTYPE_V   1
#define EM_CR_BCLKFREQ 0x00C0
#define  CR_BCLKFREQ     0xFF
#define EM_CDDET       0x00C4
#define  EMPHYLK         BIT(8)
#define  EMCD            BIT(0)
#define  EMCD_V          0

static void f_emmc_dset(void __iomem *base, u32 offset, u32 mask, u32 in)
{
	u32 data;
	u32 shift = 1;

	if (in) {
		while (!(mask & shift))
			shift <<= 1;
		data = (readl(base + offset) & ~mask) | (in * shift);
	}
	else
		data = readl(base + offset) & ~mask;

	writel(data, base + offset);
}

static int mlb01_f_emmc50_reset(struct reset_controller_dev *rcdev)
{
	struct mlb01usbh_data *rc = to_data(rcdev);
	int ret = 0;
	void __iomem *clk_base;
	void __iomem *io_base;
	void __iomem *exs_base;

	clk_base = ioremap(CLK_BASE, 0x100);
	io_base = ioremap(OTHER_CONT_BASE, 0x100);
	exs_base = rc->base;

	f_emmc_dset(clk_base, CLKSEL1, EMMCCLK, EMMCCLK_V);
	f_emmc_dset(clk_base, CLKSTOP2, EMMCCK, EMMCCK_V);

	f_emmc_dset(exs_base, RAM_PD, RAMPD_EMMC, 0);
	udelay(1);
	f_emmc_dset(exs_base, SOFTRESET, EMCRSTX, 0);
	f_emmc_dset(exs_base, EMMC_CKSTP, EMCLKSTP, 0);
	f_emmc_dset(io_base, PUDCNT, EMMMASK, EMMMASK_V);

	f_emmc_dset(exs_base, EM_CR_SLOTTYPE, CR_SLOTTYPE, CR_SLOTTYPE_V);
	f_emmc_dset(exs_base, EM_CR_BCLKFREQ, CR_BCLKFREQ,
				MLB01_F_EMMC50_CLK);
	f_emmc_dset(exs_base, EM_PV_DTIMEC, PV_DTIMEC, PV_DTIMEC_V);
	f_emmc_dset(exs_base, EM_PV_AMPBL, PV_AMPBL, PV_AMPBL_V);
	f_emmc_dset(exs_base, EM_CDDET, EMCD, EMCD_V);
	udelay(1);
	f_emmc_dset(exs_base, SOFTRESET, EMCRSTX, 1);

	iounmap(clk_base);
	iounmap(io_base);

	return ret;
}

static int mlb01_nand_reset(struct reset_controller_dev *rcdev)
{
	struct mlb01usbh_data *rc = to_data(rcdev);
	u32 val;

	/* NAND SoftReset Deassert */
	val = readl_relaxed(rc->base + SOFTRESET);
	val &= ~BIT(0);
	writel_relaxed(val, rc->base + SOFTRESET);
	val &= ~BIT(1);
	writel_relaxed(val, rc->base + SOFTRESET);
	val &= ~BIT(2);
	writel_relaxed(val, rc->base + SOFTRESET);
	
	udelay(2000);

	val |= BIT(0);
	writel_relaxed(val, rc->base + SOFTRESET);
	val |= BIT(1);
	writel_relaxed(val, rc->base + SOFTRESET);
	val |= BIT(2);
	writel_relaxed(val, rc->base + SOFTRESET);

	return 0;
}

static int mlb01usbh_reset(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	switch (id) {
	case EHCI_RESET:
		return mlb01_ehci_reset(rcdev);
	case USB2_UDC_RESET:
		return mlb01_udc2_reset(rcdev);
	case NSEC_RESET:
		return mlb01_nsec_reset(rcdev);
	case F_EMMC50_RESET:
		return mlb01_f_emmc50_reset(rcdev);
	case NAND_RESET:
		return mlb01_nand_reset(rcdev);
	default:
		pr_err("%s: RESET(%lu) not implemented yet\n", __func__, id);
		return -EINVAL;
	}
}

static struct reset_control_ops mlb01usbh_ops = {
	.deassert = mlb01usbh_reset,
	.reset = mlb01usbh_reset,
};

static int mlb01usbh_probe(struct platform_device *pdev)
{
	struct mlb01usbh_data *rc;
	struct resource *res;
	int ret;

	rc = devm_kzalloc(&pdev->dev, sizeof(*rc), GFP_KERNEL);
	if (!rc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rc->base))
		return PTR_ERR(rc->base);

	rc->rcdev.owner = THIS_MODULE;
	rc->rcdev.nr_resets = MLB01_MAX_RESETS;
	rc->rcdev.ops = &mlb01usbh_ops;
	rc->rcdev.of_node = pdev->dev.of_node;

	platform_set_drvdata(pdev, rc);

	ret = reset_controller_register(&rc->rcdev);
	if (ret) {
		dev_err(&pdev->dev, "unable to register device\n");
		return ret;
	}

	ret = register_restart_handler(&mlb01usbh_restart_nb);
	if (ret)
		dev_warn(&pdev->dev, "failed to register restart handler\n");

	return 0;
}

static int mlb01usbh_remove(struct platform_device *pdev)
{
	struct mlb01usbh_data *rc = platform_get_drvdata(pdev);
	int ret;

	ret = unregister_restart_handler(&mlb01usbh_restart_nb);
	if (ret)
		dev_warn(&pdev->dev, "failed to unregister restart handler\n");

	reset_controller_unregister(&rc->rcdev);

	return 0;
}

static const struct of_device_id mlb01usbh_match[] = {
	{ .compatible = "socionext,milbeaut-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, mlb01usbh_match);

static struct platform_driver mlb01usbh_driver = {
	.probe	= mlb01usbh_probe,
	.remove	= mlb01usbh_remove,
	.driver	= {
		.name		= "mlb01usbh-reset",
		.of_match_table	= mlb01usbh_match,
	},
};
module_platform_driver(mlb01usbh_driver);
MODULE_LICENSE("GPL v2");
