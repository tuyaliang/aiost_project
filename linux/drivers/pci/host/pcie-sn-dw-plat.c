/*
 * PCIe host controller driver for Socionext SoCs
 *
 * Copyright (C) 2016 Socionext Inc.
 *		http://www.socionext.com
 *
 * Author: Haraguchi Munehiro <haraguchi.munehiro@socionext.com>
 * Implementation based on pcie-designware-plat.c and pcie-designware.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>

#include "pcie-designware.h"

/* CTRL PCIE0=0x19150000 PCIE1=0x19160000 */
#define TYPE1_DEV_ID_VEND_ID_REG 0x000
#define   DEVICE_ID                0xFFFF0000
#define   VENDER_ID                0x0000FFFF

#define TYPE1_CLASS_CODE_REV_ID_REG 0x008
#define   BASE_CLASS_CODE             0xFF000000
#define     BASE_CLASS_CODE_VALUE       0x06
#define   SUBCLASS_CODE               0x00FF0000
#define     SUBCLASS_CODE_VALUE         0x04
#define   PROGRAM_INTERFACE           0x0000FF00
#define     PROGRAM_INTERFACE_VALUE     0x00

#define BAR0_REG 0x010
#define BAR1_REG 0x014

#define DEVICE_CONTROL_DEVICE_STATUS 0x078
#define   PCIE_CAP_MAX_READ_REQ_SIZE   (BIT(14) | BIT(13) | BIT(12))
#define   PCIE_CAP_MAX_PAYLOAD_SIZE_CS (BIT(7)  | BIT(6)  | BIT(5))

#define LINK_CAPABILITIES_REG 0x07C
#define   PCIE_CAP_MAX_LINK_WIDTH (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define   PCIE_CAP_MAX_LINK_SPEED (BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define LINK_CONTROL_LINK_STATUS_REG 0x080
#define   PCIE_CAP_NEGO_LINK_WIDTH (BIT(23) | BIT(22) | BIT(21) | BIT(20))
#define   PCIE_CAP_LINK_SPEED      (BIT(19) | BIT(18) | BIT(17) | BIT(16))

#define MISC_CONTROL_1_OFF 0x8BC
#define   DBI_RO_WR_EN       BIT(0)

#define DBI2_OFFSET 0x1000
#define BAR0_SHADOW_REG (BAR0_REG + DBI2_OFFSET)
#define BAR1_SHADOW_REG (BAR1_REG + DBI2_OFFSET)

/* EXS PCIE0=0x1B111000 PCIE1=0x1B112000 */
#define CORE_CONTROL       0x000
#define   APP_LTSSM_ENABLE    BIT(4)
#define   DEVICE_TYPE         (BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define AXI_CLK_STOP       0x004
#define   DBI_ACLK_STOP       BIT(8)
#define   SLV_ACLK_STOP       BIT(4)
#define   MSTR_ACLK_STOP      BIT(0)

#define RESET_CONTROL_1    0x00C
#define   PERST_N_O_REG       BIT(5)
#define   PERST_N_I_REG       BIT(4)
#define   BUTTON_RST_N_REG    BIT(1)
#define   PWUP_RST_N_REG      BIT(0)

#define RESET_CONTROL_2    0x010

#define RESET_SELECT_1     0x014
#define   SQU_RST_SEL         BIT(29)
#define   PHY_RST_SEL         BIT(28)
#define   PWR_RST_SEL         BIT(24)
#define   STI_RST_SEL         BIT(20)
#define   N_STI_RST_SEL       BIT(16)
#define   CORE_RST_SEL        BIT(12)
#define   PERST_SEL           BIT(4)
#define   BUTTON_RST_SEL      BIT(1)
#define   PWUP_RST_SEL        BIT(0)

#define RESET_SELECT_2     0x018
#define   DBI_ARST_SEL        BIT(8)
#define   SLV_ARST_SEL        BIT(4)
#define   MSTR_ARST_SEL       BIT(0)

#define PM_CONTROL_2       0x050
#define   SYS_AUX_PWR_DET     BIT(8)

#define PHY_CONFIG_COM_6   0x114
#define   PIPE_PORT_SEL       (BIT(1) | BIT(0))

#define LINK_MONITOR       0x210
#define   SMLH_LINK_UP        BIT(0)

struct sn_dw_plat_pcie {
	void __iomem		*ctrl_base;
	void __iomem		*exs_base;
	struct pcie_port	pp;
};

static void sn_pcie_set_data(void __iomem *base, u32 offset, u32 mask, u32 in)
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

static u32 sn_pcie_read_data(void __iomem *base, u32 offset, u32 mask)
{
	u32 data;
	u32 shift = 0;

	while (!(mask & 1)) {
		mask >>= 1;
		shift++;
	}

	data = (readl(base + offset) >> shift) & mask;

	return data;
}

static void sn_dbi_ro_wr_en(void __iomem *ctrl_base)
{

	sn_pcie_set_data(ctrl_base, MISC_CONTROL_1_OFF, DBI_RO_WR_EN, 1);
#ifdef SPEED1_LANE1
	sn_pcie_set_data(ctrl_base, LINK_CAPABILITIES_REG,
				PCIE_CAP_MAX_LINK_WIDTH, 1);
	sn_pcie_set_data(ctrl_base, LINK_CAPABILITIES_REG,
				PCIE_CAP_MAX_LINK_SPEED, 1);
#endif
	sn_pcie_set_data(ctrl_base, TYPE1_CLASS_CODE_REV_ID_REG,
				BASE_CLASS_CODE, BASE_CLASS_CODE_VALUE);
	sn_pcie_set_data(ctrl_base, TYPE1_CLASS_CODE_REV_ID_REG,
				SUBCLASS_CODE, SUBCLASS_CODE_VALUE);
	sn_pcie_set_data(ctrl_base, TYPE1_CLASS_CODE_REV_ID_REG,
				PROGRAM_INTERFACE, PROGRAM_INTERFACE_VALUE);
	sn_pcie_set_data(ctrl_base, MISC_CONTROL_1_OFF, DBI_RO_WR_EN, 0);
}

static int sn_pcie_init(void __iomem *ctrl_base, void __iomem *exs_base)
{
	/* 1: Assert all PHY / LINK resets */
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , PERST_SEL     , 0);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PERST_N_I_REG , 0);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PERST_N_O_REG , 0);
	/* Device Reset(PERST#) is effective afrer Set device_type (RC) */
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , PWUP_RST_SEL  , 0);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PWUP_RST_N_REG, 0);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , BUTTON_RST_SEL  , 0);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, BUTTON_RST_N_REG, 0);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , PWR_RST_SEL     , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_2 , MSTR_ARST_SEL   , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_2 , SLV_ARST_SEL    , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_2 , DBI_ARST_SEL    , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , CORE_RST_SEL    , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , STI_RST_SEL     , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , N_STI_RST_SEL   , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , SQU_RST_SEL     , 1);
	sn_pcie_set_data(exs_base, RESET_SELECT_1 , PHY_RST_SEL     , 1);

	/* 2: Set P<n>_app_ltssm_enable='0' for reprogramming before linkup. */
	sn_pcie_set_data(exs_base, CORE_CONTROL, APP_LTSSM_ENABLE, 0);
	/* 3: Set device_type (RC) */
	sn_pcie_set_data(exs_base, CORE_CONTROL, DEVICE_TYPE, 4);
	/* 4: Set Bifurcation  1=disable  4=able */
	sn_pcie_set_data(exs_base, PHY_CONFIG_COM_6, PIPE_PORT_SEL, 1);
	/* 5: Supply Reference (It has executed) */
	/* 6: Wait for 10usec (Reference Clocks is stable) */
	udelay(10);
	/* 7 De assert PERST# */
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PERST_N_I_REG, 1);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PERST_N_O_REG, 1);
	/* 8 Assert SYS_AUX_PWR_DET */
	sn_pcie_set_data(exs_base, PM_CONTROL_2, SYS_AUX_PWR_DET, 1);
	/* 9 Supply following clocks */
	sn_pcie_set_data(exs_base, AXI_CLK_STOP, MSTR_ACLK_STOP, 0);
	sn_pcie_set_data(exs_base, AXI_CLK_STOP, SLV_ACLK_STOP, 0);
	sn_pcie_set_data(exs_base, AXI_CLK_STOP, DBI_ACLK_STOP, 0);
	/* 10 De assert PHY reset */
	/* 11 De assert LINK's PMC reset */
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, PWUP_RST_N_REG, 1);
	sn_pcie_set_data(exs_base, RESET_CONTROL_1, BUTTON_RST_N_REG, 1);
	/* 12 PHY auto */
	/* 13 Wrapper auto */
	/* 14-17 PHY auto */
	/* 18 Wrapper auto */
	/* 19 Update registers through DBI AXI Slave interface */
	sn_dbi_ro_wr_en(ctrl_base);
	sn_pcie_set_data(ctrl_base, PCI_COMMAND, PCI_COMMAND_IO, 1);
	sn_pcie_set_data(ctrl_base, PCI_COMMAND, PCI_COMMAND_MEMORY, 1);
	sn_pcie_set_data(ctrl_base, PCI_COMMAND, PCI_COMMAND_MASTER, 1);
	/* Clear BAR0, BAR1 because RC driver accesses these regs */
	writel(0,ctrl_base + BAR0_SHADOW_REG);
	writel(0,ctrl_base + BAR1_SHADOW_REG);
	/* Max Read Request Size=1(256byte) Max Payload Size=2(512byte) */
	sn_pcie_set_data(ctrl_base, DEVICE_CONTROL_DEVICE_STATUS,
			PCIE_CAP_MAX_READ_REQ_SIZE, 1);
	sn_pcie_set_data(ctrl_base, DEVICE_CONTROL_DEVICE_STATUS,
			PCIE_CAP_MAX_PAYLOAD_SIZE_CS, 2);
	/* 20 Set P<n>_app_ltssm_enable='1' */
	sn_pcie_set_data(exs_base, CORE_CONTROL, APP_LTSSM_ENABLE, 1);

	return 0;
}

static irqreturn_t sn_dw_plat_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

static void sn_dw_plat_pcie_host_init(struct pcie_port *pp)
{
	dw_pcie_setup_rc(pp);
	dw_pcie_wait_for_link(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);
}

static int sn_plat_pcie_link_up(struct pcie_port *pp)
{
	struct sn_dw_plat_pcie *sn_dw_plat_pcie;
	void __iomem *exs_base;

	sn_dw_plat_pcie = container_of(pp, struct sn_dw_plat_pcie, pp);
	exs_base = sn_dw_plat_pcie->exs_base;

	return readl(exs_base + LINK_MONITOR) & SMLH_LINK_UP;
}

static struct pcie_host_ops sn_dw_plat_pcie_host_ops = {
	.host_init = sn_dw_plat_pcie_host_init,
	.link_up = sn_plat_pcie_link_up,
};

static int sn_dw_plat_add_pcie_port(struct pcie_port *pp,
				 struct platform_device *pdev)
{
	int ret;

	pp->irq = platform_get_irq(pdev, 1);
	if (pp->irq < 0)
		return pp->irq;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0)
			return pp->msi_irq;

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					sn_dw_plat_pcie_msi_irq_handler,
					IRQF_SHARED, "sn-dw-plat-pcie-msi", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &sn_dw_plat_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int sn_dw_plat_pcie_probe(struct platform_device *pdev)
{
	struct sn_dw_plat_pcie *sn_dw_plat_pcie;
	struct pcie_port *pp;
	struct resource *res;
	int ret;
	u32 width;
	u32 speed;

	sn_dw_plat_pcie = devm_kzalloc(&pdev->dev, sizeof(*sn_dw_plat_pcie),
					GFP_KERNEL);
	if (!sn_dw_plat_pcie)
		return -ENOMEM;

	pp = &sn_dw_plat_pcie->pp;
	pp->dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrlreg");
	if (!res)
		return -ENODEV;

	sn_dw_plat_pcie->ctrl_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sn_dw_plat_pcie->ctrl_base))
		return PTR_ERR(sn_dw_plat_pcie->ctrl_base);

	pp->dbi_base = sn_dw_plat_pcie->ctrl_base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "exsreg");
	if (!res)
		return -ENODEV;
	sn_dw_plat_pcie->exs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sn_dw_plat_pcie->exs_base))
		return PTR_ERR(sn_dw_plat_pcie->exs_base);

	ret = sn_pcie_init(sn_dw_plat_pcie->ctrl_base,
				sn_dw_plat_pcie->exs_base);
	if (ret < 0)
		return -ENODEV;

	ret = sn_dw_plat_add_pcie_port(pp, pdev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, sn_dw_plat_pcie);

#ifdef SPEED1_LANE1
	dev_info(pp->dev, " Set <link width=1> <link speed=GEN1>\n");
#endif
	width = sn_pcie_read_data(sn_dw_plat_pcie->ctrl_base,
		LINK_CONTROL_LINK_STATUS_REG, PCIE_CAP_NEGO_LINK_WIDTH);
	speed = sn_pcie_read_data(sn_dw_plat_pcie->ctrl_base,
		LINK_CONTROL_LINK_STATUS_REG, PCIE_CAP_LINK_SPEED);

	dev_info(pp->dev, " <link width = %d> <link speed = %d>\n",
			width, speed);

	return 0;
}

static const struct of_device_id sn_dw_plat_pcie_of_match[] = {
	{ .compatible = "socionext,mlb01-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, sn_dw_plat_pcie_of_match);

static struct platform_driver sn_dw_plat_pcie_driver = {
	.driver = {
		.name	= "sn-pcie",
		.of_match_table = sn_dw_plat_pcie_of_match,
	},
	.probe = sn_dw_plat_pcie_probe,
};

module_platform_driver(sn_dw_plat_pcie_driver);

MODULE_AUTHOR("Haraguchi Munehiro <haraguchi.munehiro@socionext.com>");
MODULE_DESCRIPTION("Socionext PCIe host controller driver");
MODULE_LICENSE("GPL v2");
