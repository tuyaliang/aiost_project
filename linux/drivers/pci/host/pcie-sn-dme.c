/*
 * PCIe host controller driver for Socionext pcie2dme IP
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * Based on -->
 *
 * Socionext IP driver by Slash Huang <slash.huang@socionext.com>
 *				      <slash.linux.c@gmail.com>
 *
 * and
 *
 * PCIe host controller driver for Xilinx AXI PCIe Bridge
 *
 * Copyright (c) 2012 - 2014 Xilinx, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pci_ids.h>
#include <linux/completion.h>

enum sn_pcie2_dme_regs {

	/* General Control Register area */

	SNP_REG_GCR_SYSCON1	= 0,
		GCR_SYSCON1_HP_PRSNTX_VAL	= BIT(28), /* prsnt VAL */
		GCR_SYSCON1_HP_PRSNTX_SEL	= BIT(24), /* 0=ext 1= VAL */
		GCR_SYSCON1_PEX_SEL		= BIT(8), /* 0 = use RST_PEX */
		GCR_SYSCON1_PRV_SUPP_LINK_SPD	= BIT(4), /* 1 = GEN2 */
		GCR_SYSCON1_PRV_RE_TYPE		= BIT(0), /* 1 = Rootcomplex */

	SNP_REG_GCR_SYSCON2	= 4,
		GCR_SYSCON2_PHY_RSTX		= BIT(16), /* nPHY_RSTX val */
		GCR_SYSCON2_RST_PEX		= BIT(8), /* nRST_PEX val */
		GCR_SYSCON2_RST_PONX		= BIT(0), /* nRST_PONX val */

	SNP_REG_GCR_SYS_STATUS	= 8,
		GCR_SYS_STATUS_CPPEX_I_RO		= BIT(24),
		GCR_SYS_STATUS_PERSTX_I_RO		= BIT(16),
		GCR_SYS_STATUS_RST_SWRSTX_RO		= BIT(0),

	SNP_REG_GCR_PM_STATUS	= 0x100,
		GCR_PM_STATUS_PM_AUXEN_RO		= BIT(12),
		GCR_PM_STATUS_PM_PMERQ_DET_RO		= BIT(8),
		GCR_PM_STATUS_PM_TOFF_RO		= BIT(4),
		GCR_PM_STATUS_PM_F0_PMST_RO_MASK	= GENMASK(1, 0),
		GCR_PM_STATUS_PM_F0_PMST_RO_SHIFT	= 0,

	SNP_REG_GCR_ST_STATUS1	= 0x200,
		GCR_ST_STATUS1_ST_RE_TYPE_RO		= BIT(24),
		GCR_ST_STATUS1_ST_F0_MST_EN_RO		= BIT(23),
		GCR_ST_STATUS1_ST_DL_UP_RO		= BIT(20),
		GCR_ST_STATUS1_ST_GR_GMX_RO		= BIT(16),
		GCR_ST_STATUS1_ST_LTSSM_RO_MASK		= GENMASK(3, 0),
		GCR_ST_STATUS1_ST_LTSSM_RO_SHIFT	= 8,
		GCR_ST_STATUS1_ST_LTSSM_SUB_RO_MASK	= GENMASK(2, 0),
		GCR_ST_STATUS1_ST_LTSSM_SUB_RO_SHIFT	= 0,

	SNP_REG_GCR_ST_STATUS2	= 0x204,
		GCR_ST_STATUS2_ST_MSIX_ABORT_RO		= BIT(28),
		GCR_ST_STATUS2_ST_MSI_ABORT_RO		= BIT(24),
		GCR_ST_STATUS2_ST_TRS0_EN_RO		= BIT(21),
		GCR_ST_STATUS2_ST_TRS_INT_RO		= BIT(20),
		GCR_ST_STATUS2_ST_ICS_EN_RO		= BIT(17),
		GCR_ST_STATUS2_ST_ICS_INT_RO		= BIT(16),
		GCR_ST_STATUS2_ST_DMA_INT_RO_MASK	= GENMASK(1, 0),
		GCR_ST_STATUS2_ST_DMA_INT_RO_SHIFT	= BIT(8),
		GCR_ST_STATUS2_ST_EXPCAP_INT_RO		= BIT(4),
		GCR_ST_STATUS2_ST_ERR_INT_RO		= BIT(0),

	SNP_REG_GCR_ST_STATUS3	= 0x208,
		GCR_ST_STATUS3_ST_SERR_EN		= BIT(28), /* 1 =use */
		GCR_ST_STATUS3_ST_SERR_OUT_EN		= BIT(24), /* irqen */
		GCR_ST_STATUS3_ST_SERR_INT		= BIT(16), /* irqst */
		GCR_ST_STATUS3_ST_TRM_INT_RO		= BIT(8),
		GCR_ST_STATUS3_ST_SERR_RO		= BIT(0),

	SNP_REG_GCR_HP_STATUS	= 0x300,
		GCR_HP_STATUS_HP_PRSNTX_RO		= BIT(24),
		GCR_HP_STATUS_HP_EMIL_RO		= BIT(16),
		GCR_HP_STATUS_HP_PWR_ENX_RO		= BIT(12),
		GCR_HP_STATUS_HP_ATN_LEDX_RO		= BIT(8),
		GCR_HP_STATUS_HP_PWR_LEDX_RO		= BIT(4),
		GCR_HP_STATUS_HP_CLKENX_RO		= BIT(0),

	SNP_REG_GCR_HP_CONTROL	= 0x304,
		GCR_HP_CONTROL_HP_EMILS			= BIT(24),
		GCR_HP_CONTROL_HP_MRLX			= BIT(16),
		GCR_HP_CONTROL_HP_PWR_FLTX		= BIT(8),
		GCR_HP_CONTROL_HP_BUTTONX		= BIT(0),

	/* Message Purpose Register area */

	SNP_REG_MPR_INTCOM_CON1	= 0x4000,
		MPR_INTCOM_CON1_MSG_F0			= BIT(0),
	SNP_REG_MPR_INTCOM_CON2	= 0x4004,
	SNP_REG_MPR_INTCOM_STAT1	= 0x4008,
		MPR_INTCOM_STAT1_MSIX_MASK_RO		= BIT(12),
		MPR_INTCOM_STAT1_MSIX_F0_EN_RO		= BIT(8),
		MPR_INTCOM_STAT1_MSI_F0_EN_RO		= BIT(4),
		MPR_INTCOM_STAT1_MSI_MM_EN_RO_MASK	= GENMASK(2, 0),
		MPR_INTCOM_STAT1_MSI_MM_EN_RO_SHIFT	= 0,
	
	SNP_REG_MPR_INTCOM_STAT2	= 0x400c,
	SNP_REG_MPR_INTCOM_STAT3	= 0x4010,
	SNP_REG_MPR_INTCOM_STAT4	= 0x4014,
	SNP_REG_MPR_INTCOM_STAT5	= 0x4018,
	SNP_REG_MPR_INTX_STATUS	= 0x4100,
		MPR_INTX_STATUS_MSG_INTD_RO		= BIT(12),
		MPR_INTX_STATUS_MSG_INTC_RO		= BIT(8),
		MPR_INTX_STATUS_MSG_INTB_RO		= BIT(4),
		MPR_INTX_STATUS_MSG_INTA_RO		= BIT(0),

	SNP_REG_MPR_MSIINT_EN		= 0x4200,
	SNP_REG_MPR_MSIINT_OUTEN	= 0x4204,
	SNP_REG_MPR_MSIINT_STAT		= 0x4208,
	SNP_REG_MPR_MSI_DATA_CLR	= 0x420c,
		MPR_MSI_DATA_CLR_CLEAR			= BIT(0),
	SNP_REG_MPR_MDI_DATA0		= 0x6000,

	SNP_CORE_OFS			= 0x2000,

	SNP_REG_CORE_CSR		= SNP_CORE_OFS + 0x408,
	SNP_REG_CORE_AXI_MODE		= SNP_CORE_OFS + 0x880,
	SNP_REG_CORE_AXI_SLAVE_TXFER_EN	= SNP_CORE_OFS + 0x884,
	SNP_REG_CORE_AXI_SLAVE_CONF_INT	= SNP_CORE_OFS + 0x1000,
	SNP_REG_CORE_AXI_SLAVE_CONF_CTL	= SNP_CORE_OFS + 0x1008,

	SNP_REG_CORE_SLAVE_BASE_ADS	= SNP_CORE_OFS + 0x1100,
	SNP_REG_CORE_SLAVE_TARGET	= SNP_CORE_OFS + 0x1110,
	SNP_REG_CORE_SLAVE_TGT_STATUS	= SNP_CORE_OFS + 0x100C,

	SNP_REG_CORE_SLAVE_REMAPM_CTL	= SNP_CORE_OFS + 0x1808,

	/* there are 16 sets of these registers, each set offset by +0x10 */

	SNP_REG_CORE_SLAVE_REMAPM_BASE	= SNP_CORE_OFS + 0x1820,
	SNP_REG_CORE_SLAVE_REMAPM_SIZE	= SNP_CORE_OFS + 0x1824,
	SNP_REG_CORE_SLAVE_REMAPM_PCAL	= SNP_CORE_OFS + 0x1828,
	SNP_REG_CORE_SLAVE_REMAPM_PCAH	= SNP_CORE_OFS + 0x182c,

	/* there are 16 sets of these registers, each set offset by +0x10 */

	SNP_REG_CORE_SLAVE_REMAPI_BASE	= SNP_CORE_OFS + 0x1200,
	SNP_REG_CORE_SLAVE_REMAPI_SIZE	= SNP_CORE_OFS + 0x1204,
	SNP_REG_CORE_SLAVE_REMAPI_PCA	= SNP_CORE_OFS + 0x1208,
};

#define XILINX_NUM_MSI_IRQS		32
#define XILINX_MAX_NUM_RESOURCES	3

/* helper for offsetting between register sets */
#define SL(n) (n * 16)


/**
 * struct sn_pcie2_dme - PCIe port information
 * @base: IO Mapped Register Bases
 * @irq: Interrupt number
 * @msi_pages: MSI pages
 * @root_busno: Root Bus number
 * @dev: Device pointer
 * @irq_domain: IRQ domain pointer
 * @bus_range: Bus range
 * @resources: Bus Resources
 */
struct sn_pcie2_dme {
	void __iomem *base, *base_cs;
	struct of_pci_range range_cs;
	u32 irq[5];
	void __iomem *msi_pages;
	u8 root_busno;
	struct device *dev;
	struct irq_domain *irq_domain;
	struct resource bus_range;
	struct list_head resources;
	struct resource mem[XILINX_MAX_NUM_RESOURCES];
	int mem_count;
	struct completion link;
	DECLARE_BITMAP(msi_irq_in_use, XILINX_NUM_MSI_IRQS);
	u32 virq[XILINX_NUM_MSI_IRQS];
};

static inline struct sn_pcie2_dme *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static inline bool sn_pcie_dme_link_is_up(struct sn_pcie2_dme *port)
{
	return !!(readl(port->base + SNP_REG_GCR_ST_STATUS1) &
		  GCR_ST_STATUS1_ST_DL_UP_RO);
}

static int sn_pcie_dme_reset(struct sn_pcie2_dme *port)
{
	/* if the core says there's no card, then no card / clock/ power */
	if (readl(port->base + SNP_REG_GCR_HP_STATUS) &
	    GCR_HP_STATUS_HP_PRSNTX_RO)
		return -ENODEV;

	writel(GCR_SYSCON2_RST_PONX, port->base + SNP_REG_GCR_SYSCON2);

	writel(GCR_SYSCON1_PRV_SUPP_LINK_SPD |
	       GCR_SYSCON1_PRV_RE_TYPE,
	       port->base + SNP_REG_GCR_SYSCON1);

	writel(GCR_SYSCON2_RST_PEX |
	       GCR_SYSCON2_RST_PONX,
	       port->base + SNP_REG_GCR_SYSCON2);

	/* set required registers before linkup... */

	writel(readl(port->base + SNP_REG_CORE_AXI_MODE) | BIT(8) | BIT(3),
	       port->base + SNP_REG_CORE_AXI_MODE);

	writel((0x1234 << 16) | PCI_VENDOR_ID_FUJITSU_ME,
	       port->base + SNP_CORE_OFS + PCI_VENDOR_ID);
	writew(PCI_CLASS_BRIDGE_PCI,
	       port->base + SNP_CORE_OFS + PCI_CLASS_DEVICE);
	writew(BIT(12), /* reference clock is used */
	       port->base + SNP_CORE_OFS + 0x92);

	writel(GCR_SYSCON2_RST_PEX |
	       GCR_SYSCON2_RST_PONX |
	       GCR_SYSCON2_PHY_RSTX,
	       port->base +  SNP_REG_GCR_SYSCON2);

	/* GEN2 */
	writel(2, port->base + SNP_CORE_OFS + 0x840);

	writel((readl(port->base + SNP_CORE_OFS + 0x88) &
		~(BIT(14) | BIT(13) | BIT(12) | BIT(7) | BIT(6) | BIT(5))) |
		(3 << 12), port->base + SNP_CORE_OFS + 0x88);

	writeb(BIT(1) | BIT(0), port->base + SNP_CORE_OFS + 0x820);

	writel(0xfff80000, port->base + SNP_CORE_OFS + 0x804);
	writel(0xfff80000, port->base + SNP_CORE_OFS + 0x808);

	writel(0, port->base + SNP_CORE_OFS + 0x10);
	writel(0, port->base + SNP_CORE_OFS + 0x14);

	writel(0x147, port->base + SNP_CORE_OFS + 0x4);
	/* release the core reset */
	writel(readl(port->base + SNP_REG_CORE_CSR) | BIT(0),
	       port->base + SNP_REG_CORE_CSR);


	reinit_completion(&port->link);

	/* release the core reset */
	writel(readl(port->base + SNP_REG_CORE_CSR) & ~BIT(0),
	       port->base + SNP_REG_CORE_CSR);

	writel(0, port->base + SNP_CORE_OFS + 0x804);
	writel(0, port->base + SNP_CORE_OFS + 0x808);
	writel(0, port->base + SNP_CORE_OFS + 0x820);

	/* enable MSI */
	writew(0x81 | (5 << 1), port->base + SNP_CORE_OFS + 0xc2);

	/* disable MSI-X */
	writew(readw(port->base + SNP_CORE_OFS + 0xe2) | BIT(15),
		port->base + SNP_CORE_OFS + 0xe2);

	/* enable hotplug interrupts */
	writew(readw(port->base + SNP_CORE_OFS + 0x98) | BIT(5) | BIT(12),
		port->base + SNP_CORE_OFS + 0x98);

	/* VC enable */
	writew(readw(port->base + SNP_CORE_OFS + 0x214) | BIT(31),
		port->base + SNP_CORE_OFS + 0x214);

	/* INTx interrupt disable off */
	writew(readw(port->base + SNP_CORE_OFS + 0x4) & ~BIT(10),
		port->base + SNP_CORE_OFS + 0x4);

	writel(~0, port->base + SNP_REG_MPR_MSIINT_EN);
	writel(~0, port->base + SNP_REG_MPR_MSIINT_OUTEN);

	/* exit DL config mode */
	writel(readl(port->base + SNP_REG_CORE_AXI_MODE) & ~BIT(3),
	       port->base + SNP_REG_CORE_AXI_MODE);

	if (!wait_for_completion_timeout(&port->link, msecs_to_jiffies(500)))
		dev_err(port->dev, "Link didn't come up\n");

	writel(GCR_SYSCON2_RST_PONX |
	       GCR_SYSCON2_PHY_RSTX,
	       port->base +  SNP_REG_GCR_SYSCON2);

	udelay(1);

	writel(GCR_SYSCON2_RST_PEX |
	       GCR_SYSCON2_RST_PONX |
	       GCR_SYSCON2_PHY_RSTX,
	       port->base +  SNP_REG_GCR_SYSCON2);

	return 0;
}

#if 0
/**
 * sn_pcie_dme_clear_err_interrupts - Clear Error Interrupts
 * @port: PCIe port information
 */
static void sn_pcie_dme_clear_err_interrupts(struct sn_pcie2_dme *port)
{
	unsigned long val = readl(port->base + SNP_REG_GCR_ST_STATUS3);

	if (val & GCR_ST_STATUS3_ST_SERR_INT)
		writel(GCR_ST_STATUS3_ST_SERR_INT,
		       port->base + SNP_REG_GCR_ST_STATUS3);
}
#endif

/**
 * sn_pcie_dme_valid_device - Check if a valid device is present on bus
 * @bus: PCI Bus structure
 * @devfn: device/function
 *
 * Return: 'true' on success and 'false' if invalid device is found
 */
static bool sn_pcie_dme_valid_device(struct pci_bus *bus, unsigned int devfn)
{
	struct sn_pcie2_dme *port = sys_to_pcie(bus->sysdata);

	/* if the core says there's no card, then it's invalid */
	if (readl(port->base + SNP_REG_GCR_HP_STATUS) &
	    GCR_HP_STATUS_HP_PRSNTX_RO)
		return false;

	/* Check if link is up when trying to access downstream ports */
	if (bus->number != port->root_busno && !sn_pcie_dme_link_is_up(port)) {
		dev_err(port->dev, "Link down\n");
		return false;
	}

	/* Only one device down on each root port */
	if (bus->number == port->root_busno && devfn > 0)
		return false;

	/*
	 * Do not read more than one device on the bus directly attached
	 * to RC.
	 */
	if (bus->primary == port->root_busno && devfn > 0)
		return false;

	return true;
}

static struct pci_bus *card_bus;

/**
 * sn_pcie_dme_map_bus - Get configuration base
 * @bus: PCI Bus structure
 * @devfn: Device/function
 * @where: Offset from base
 *
 * Return: Base address of the configuration space needed to be
 *	   accessed.
 */
static void __iomem *sn_pcie_dme_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct sn_pcie2_dme *port = sys_to_pcie(bus->sysdata);
	u32 type = 0;
	int timeout = 1000;

	if (!sn_pcie_dme_valid_device(bus, devfn))
		return NULL;

	if (!bus->number)
		return port->base + SNP_CORE_OFS + (devfn << 13) + where;

	dev_dbg(port->dev, "%s: bus %d, devfn %d, where %d %p\n", __func__,
		bus->number, devfn, where, port->base);

	if (bus->number > 1)
		type = 1;

	writew(readw(port->base + SNP_CORE_OFS + PCI_COMMAND) |
	       PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
	       port->base + SNP_CORE_OFS + PCI_COMMAND);

	/* we use slave remapper 0 to gain acess to pcie bus config space */

	writel(port->range_cs.cpu_addr,
	       port->base + SNP_REG_CORE_SLAVE_BASE_ADS);
	writel(BIT(1) | BIT(0), port->base + SNP_REG_CORE_AXI_SLAVE_CONF_CTL);
	/* Slave Transfer (ICS and TRS) Enable */
	writel(BIT(8) | BIT(0), port->base + SNP_REG_CORE_AXI_SLAVE_TXFER_EN);

	writel((type << 16) | (bus->number << 8) | (PCI_SLOT(devfn) << 3) |
	       PCI_FUNC(devfn), port->base + SNP_REG_CORE_SLAVE_TARGET);

	while (--timeout &&
	       (readb(port->base + SNP_REG_CORE_SLAVE_TGT_STATUS) & BIT(0)))
		if (timeout > 1500)
			udelay(1);
		else
			usleep_range(100, 500);

	if (!timeout) {
		dev_err(port->dev, "Timeout waiting for slave config\n");
		return NULL;
	}

	card_bus = bus;

	return port->base_cs + where;
}

/* PCIe operations */
static struct pci_ops sn_pcie_dme_ops = {
	.map_bus = sn_pcie_dme_map_bus,
	.read	= pci_generic_config_read,
	.write	= pci_generic_config_write,
};

/* MSI functions */

/**
 * sn_pcie_dme_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void sn_pcie_dme_destroy_msi(unsigned int irq)
{
	struct irq_desc *desc;
	struct msi_desc *msi;
	struct sn_pcie2_dme *port;

	pr_err("%s\n", __func__);

	desc = irq_to_desc(irq);
	msi = irq_desc_get_msi_desc(desc);
	port = sys_to_pcie(msi_desc_to_pci_sysdata(msi));

	if (!test_bit(irq, port->msi_irq_in_use))
		dev_err(port->dev, "Trying to free unused MSI#%d\n", irq);
	else
		clear_bit(irq, port->msi_irq_in_use);
}

/**
 * sn_pcie_dme_assign_msi - Allocate MSI number
 * @port: PCIe port structure
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int sn_pcie_dme_assign_msi(struct sn_pcie2_dme *port)
{
	int pos;

	pr_err("%s\n", __func__);

	pos = find_first_zero_bit(port->msi_irq_in_use, XILINX_NUM_MSI_IRQS);
	if (pos < XILINX_NUM_MSI_IRQS)
		set_bit(pos, port->msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

/**
 * sn_msi_teardown_irq - Destroy the MSI
 * @chip: MSI Chip descriptor
 * @irq: MSI IRQ to destroy
 */
static void sn_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	struct irq_desc *desc;
	struct msi_desc *msi;
	struct sn_pcie2_dme *port;

	pr_err("%s\n", __func__);

	desc = irq_to_desc(irq);
	msi = irq_desc_get_msi_desc(desc);
	port = sys_to_pcie(msi_desc_to_pci_sysdata(msi));

	sn_pcie_dme_destroy_msi(irq);

	writel(readl(port->base + SNP_REG_MPR_MSIINT_EN) & ~(1 <<irq),
		port->base + SNP_REG_MPR_MSIINT_EN);
	writel(readl(port->base + SNP_REG_MPR_MSIINT_OUTEN) & ~(1 <<irq),
		port->base + SNP_REG_MPR_MSIINT_OUTEN);
}

/**
 * sn_pcie_dme_msi_setup_irq - Setup MSI request
 * @chip: MSI chip pointer
 * @pdev: PCIe device pointer
 * @desc: MSI descriptor pointer
 *
 * Return: '0' on success and error value on failure
 */
static int sn_pcie_dme_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct sn_pcie2_dme *port = sys_to_pcie(pdev->bus->sysdata);
	unsigned int irq;
	int hwirq;
	struct msi_msg msg;
	phys_addr_t msg_addr;

	pr_err("%s\n", __func__);

	hwirq = sn_pcie_dme_assign_msi(port);
	if (hwirq < 0)
		return hwirq;

	writel(readl(port->base + SNP_REG_MPR_MSIINT_EN) | (1 << hwirq),
		port->base + SNP_REG_MPR_MSIINT_EN);
	writel(readl(port->base + SNP_REG_MPR_MSIINT_OUTEN) | (1 << hwirq),
		port->base + SNP_REG_MPR_MSIINT_OUTEN);

	irq = irq_create_mapping(port->irq_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg_addr = virt_to_phys((void *)port->msi_pages);

	msg.address_hi = 0;
	msg.address_lo = msg_addr;
	msg.data = irq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

/* MSI Chip Descriptor */
static struct msi_controller sn_pcie_dme_msi_chip = {
	.setup_irq = sn_pcie_dme_msi_setup_irq,
	.teardown_irq = sn_msi_teardown_irq,
};

/* HW Interrupt Chip Descriptor */
static struct irq_chip sn_msi_irq_chip = {
	.name = "Socionext PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/**
 * sn_pcie_dme_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int sn_pcie_dme_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	//pr_err("%s: irq %u hwirq %u\n", __func__, irq, (u32)hwirq);
	irq_set_chip_and_handler(irq, &sn_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
	.map = sn_pcie_dme_msi_map,
};

/**
 * sn_pcie_dme_enable_msi - Enable MSI support
 * @port: PCIe port information
 */
static void sn_pcie_dme_enable_msi(struct sn_pcie2_dme *port)
{
	phys_addr_t msg_addr;

	port->msi_pages = ioremap(0x28000000, 0x10000);
	msg_addr = 0x28000000;
	writew(0x81 | (5 << 1), port->base + SNP_CORE_OFS + 0xc2);

	/* 32 MSI */
	writel(msg_addr, port->base + SNP_CORE_OFS + 0xc4);
	writel((u64)msg_addr >> 32, port->base + SNP_CORE_OFS + 0xc8);
}

/* INTx Functions */

/**
 * sn_pcie_dme_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int sn_pcie_dme_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct sn_pcie2_dme *port = domain->host_data;

	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	writel(~0, port->base + SNP_CORE_OFS + 0x1800);

	return 0;
}


/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = sn_pcie_dme_intx_map,
};

/* PCIe HW Functions */

static irqreturn_t sn_pcie_dme_intr_handler(int irq, void *data)
{
	struct sn_pcie2_dme *port = data;
	u32 status2 = readl(port->base + SNP_REG_GCR_ST_STATUS2);
	u32 status = readl(port->base + SNP_REG_MPR_INTX_STATUS);

	writel(status2, port->base + SNP_REG_GCR_ST_STATUS2);

//	pr_err("%s: 0x%x, INTX status 0x%x\n", __func__, status2, status);

	if (status2 & GCR_ST_STATUS2_ST_EXPCAP_INT_RO) {
		status = readw(port->base + SNP_CORE_OFS + 0x9a);
		if (status & BIT(8)) {
			pcie_update_link_speed(to_pci_bus(port->dev),
				readw(port->base + SNP_CORE_OFS + 0x92));
			if (sn_pcie_dme_link_is_up(port))
				complete(&port->link);
		}
		writew(status, port->base + SNP_CORE_OFS + 0x9a);

//		pr_err("post-service status2 0x%x\n", readl(port->base + SNP_REG_GCR_ST_STATUS2));
	}
	if (status2 & GCR_ST_STATUS2_ST_ERR_INT_RO) {
		dev_err(port->dev, "%s: ERR_INT\n", __func__);
	}

	if (status & 1)
		generic_handle_irq(irq_find_mapping(port->irq_domain, 1));

#if 0
	if (status & XILINX_PCIE_INTR_INTX) {
		/* INTx interrupt received */
		val = readl(port, XILINX_PCIE_REG_RPIFR1);

		/* Check whether interrupt valid */
		if (!(val & XILINX_PCIE_RPIFR1_INTR_VALID)) {
			dev_warn(port->dev, "RP Intr FIFO1 read error\n");
			return IRQ_HANDLED;
		}

		/* Clear interrupt FIFO register 1 */
		writel(port, XILINX_PCIE_RPIFR1_ALL_MASK,
			   XILINX_PCIE_REG_RPIFR1);

		/* Handle INTx Interrupt */
		val = ((val & XILINX_PCIE_RPIFR1_INTR_MASK) >>
			XILINX_PCIE_RPIFR1_INTR_SHIFT) + 1;
		generic_handle_irq(irq_find_mapping(port->irq_domain, val));
	}

	if (status & XILINX_PCIE_INTR_MSI) {
		/* MSI Interrupt */
		val = readl(port, XILINX_PCIE_REG_RPIFR1);

		if (!(val & XILINX_PCIE_RPIFR1_INTR_VALID)) {
			dev_warn(port->dev, "RP Intr FIFO1 read error\n");
			return IRQ_HANDLED;
		}

		if (val & XILINX_PCIE_RPIFR1_MSI_INTR) {
			msi_data = readl(port, XILINX_PCIE_REG_RPIFR2) &
				   XILINX_PCIE_RPIFR2_MSG_DATA;

			/* Clear interrupt FIFO register 1 */
			writel(port, XILINX_PCIE_RPIFR1_ALL_MASK,
				   XILINX_PCIE_REG_RPIFR1);

			if (IS_ENABLED(CONFIG_PCI_MSI)) {
				/* Handle MSI Interrupt */
				generic_handle_irq(msi_data);
			}
		}
	}
#endif

	return IRQ_HANDLED;
}



/**
 * sn_pcie_dme_free_irq_domain - Free IRQ domain
 * @port: PCIe port information
 */
static void sn_pcie_dme_free_irq_domain(struct sn_pcie2_dme *port)
{
	int i;
	u32 irq, num_irqs;

	/* Free IRQ Domain */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		iounmap(port->msi_pages);

		num_irqs = XILINX_NUM_MSI_IRQS;
	} else {
		/* INTx */
		num_irqs = 4;
	}

	for (i = 0; i < num_irqs; i++) {
		irq = irq_find_mapping(port->irq_domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(port->irq_domain);
}

/**
 * sn_pcie_dme_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int sn_pcie_dme_init_irq_domain(struct sn_pcie2_dme *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pcie_intc_node;

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return PTR_ERR(pcie_intc_node);
	}

	port->irq_domain = irq_domain_add_linear(pcie_intc_node, 4,
						 &intx_domain_ops,
						 port);
	if (!port->irq_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return PTR_ERR(port->irq_domain);
	}

	/* Setup MSI */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		port->irq_domain = irq_domain_add_linear(node,
							 XILINX_NUM_MSI_IRQS,
							 &msi_domain_ops,
							 &sn_pcie_dme_msi_chip);
		if (!port->irq_domain) {
			dev_err(dev, "Failed to get a MSI IRQ domain\n");
			return PTR_ERR(port->irq_domain);
		}

		sn_pcie_dme_enable_msi(port);
	}

	return 0;
}

/**
 * sn_pcie_dme_init_port - Initialize hardware
 * @port: PCIe port information
 */
static void sn_pcie_dme_init_port(struct sn_pcie2_dme *port)
{
	/* enable "own" EXPCAP and ERR interrupt sources */
	writel(readl(port->base + SNP_CORE_OFS + 4) & ~BIT(10),
	       port->base + SNP_CORE_OFS + 4);
}

/**
 * sn_pcie_dme_setup - Setup memory resources
 * @nr: Bus number
 * @sys: Per controller structure
 *
 * Return: '1' on success and error value on failure
 */
static int sn_pcie_dme_setup(int nr, struct pci_sys_data *sys)
{
	struct sn_pcie2_dme *port = sys_to_pcie(sys);

	writeb(sys->busnr, port->base + SNP_CORE_OFS + PCI_PRIMARY_BUS);

	writew(readw(port->base + SNP_CORE_OFS + PCI_COMMAND) |
	       PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
	       port->base + SNP_CORE_OFS + PCI_COMMAND);

	list_splice_init(&port->resources, &sys->resources);

	return 1;
}

/**
 * sn_pcie_dme_scan_bus - Scan PCIe bus for devices
 * @nr: Bus number
 * @sys: Per controller structure
 *
 * Return: Valid Bus pointer on success and NULL on failure
 */
static struct pci_bus *sn_pcie_dme_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct sn_pcie2_dme *port = sys_to_pcie(sys);
	struct pci_bus *bus;

	port->root_busno = sys->busnr;
	bus = pci_scan_root_bus(port->dev, sys->busnr, &sn_pcie_dme_ops,
				sys, &sys->resources);

	return bus;
}

/**
 * sn_pcie_dme_parse_and_add_res - Add resources by parsing ranges
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int sn_pcie_dme_parse_and_add_res(struct sn_pcie2_dme *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct resource *mem;
	resource_size_t offset;
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	struct resource_entry *win;
	int err = 0, mem_resno = 0, n = 0;

	/* Get the ranges */
	if (of_pci_range_parser_init(&parser, node)) {
		dev_err(dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}

	/* Parse the ranges and add the resources found to the list */
	for_each_of_pci_range(&parser, &range) {

		if (mem_resno >= XILINX_MAX_NUM_RESOURCES) {
			dev_err(dev, "Maximum memory resources exceeded\n");
			return -EINVAL;
		}

		mem = devm_kmalloc(dev, sizeof(*mem), GFP_KERNEL);
		if (!mem) {
			err = -ENOMEM;
			goto free_resources;
		}

		of_pci_range_to_resource(&range, node, mem);

		switch (mem->flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_MEM:
			offset = range.cpu_addr - range.pci_addr;
			mem_resno++;
			break;
		case 0: /* config */
			port->range_cs = range;
			port->base_cs = devm_ioremap(dev, range.cpu_addr,
						     range.size - 1);
			if (IS_ERR(port->base_cs))
				return PTR_ERR(port->base_cs);
			break;
		default:
			err = -EINVAL;
			break;
		}

		if (err < 0) {
			dev_warn(dev, "Invalid resource found %pR\n", mem);
			continue;
		}

		err = request_resource(&iomem_resource, mem);
		if (err)
			goto free_resources;

		pci_add_resource_offset(&port->resources, mem, offset);

		writel(range.cpu_addr,
		       port->base + SNP_REG_CORE_SLAVE_REMAPI_BASE + SL(n));
		writel(range.size,
		       port->base + SNP_REG_CORE_SLAVE_REMAPI_SIZE + SL(n));
		writel(range.pci_addr >> 32,
		       port->base + SNP_REG_CORE_SLAVE_REMAPI_PCA);

		writel(range.cpu_addr,
		       port->base + SNP_REG_CORE_SLAVE_REMAPM_BASE + SL(n));
		writel(range.size,
		       port->base + SNP_REG_CORE_SLAVE_REMAPM_SIZE + SL(n));
		writel(range.pci_addr,
		       port->base + SNP_REG_CORE_SLAVE_REMAPM_PCAL + SL(n));
		writel(range.pci_addr >> 32,
		       port->base + SNP_REG_CORE_SLAVE_REMAPM_PCAH + SL(n));
	}

	/* Get the bus range */
	if (of_pci_parse_bus_range(node, &port->bus_range)) {
		port->bus_range = (struct resource) {
			.name	= node->name,
			.start	= 0,
			.end	= 0xff,
			.flags	= IORESOURCE_BUS,
		};
	}

	/* Register bus resource */
	pci_add_resource(&port->resources, &port->bus_range);

	return 0;

free_resources:
	release_child_resources(&iomem_resource);
	resource_list_for_each_entry(win, &port->resources)
		devm_kfree(dev, win->res);
	pci_free_resource_list(&port->resources);

	return err;
}

/**
 * sn_pcie_dme_parse_dt - Parse Device tree
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int sn_pcie_dme_parse_dt(struct sn_pcie2_dme *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct resource regs;
	const char *type;
	int err, n = 0;

	type = of_get_property(node, "device_type", NULL);
	if (!type || strcmp(type, "pci")) {
		dev_err(dev, "invalid \"device_type\" %s\n", type);
		return -EINVAL;
	}

	err = of_address_to_resource(node, 0, &regs);
	if (err) {
		dev_err(dev, "missing \"reg\" property\n");
		return err;
	}

	port->base = devm_ioremap_resource(dev, &regs);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	do {
		port->irq[n] = irq_of_parse_and_map(node, n);
		err = devm_request_irq(dev, port->irq[n],
				       sn_pcie_dme_intr_handler, IRQF_SHARED,
				       "sn-pcie", port);
		if (err)
			break;
		n++;
	} while (n < ARRAY_SIZE(port->irq));

	return 0;
}

/**
 * sn_pcie_dme_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int sn_pcie_dme_probe(struct platform_device *pdev)
{
	struct sn_pcie2_dme *port;
	struct hw_pci hw;
	struct device *dev = &pdev->dev;
	int err;
	static int nprobes;

	if (nprobes++ < 4)
		return -EPROBE_DEFER;

	if (!dev->of_node)
		return -ENODEV;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->dev = dev;
	init_completion(&port->link);

	err = sn_pcie_dme_parse_dt(port);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	err = sn_pcie_dme_reset(port);
	if (err)
		return err;
	sn_pcie_dme_init_port(port);

	err = sn_pcie_dme_init_irq_domain(port);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return err;
	}

	/*
	 * Parse PCI ranges, configuration bus range and
	 * request their resources
	 */
	INIT_LIST_HEAD(&port->resources);
	err = sn_pcie_dme_parse_and_add_res(port);
	if (err) {
		dev_err(dev, "Failed adding resources\n");
		return err;
	}

	platform_set_drvdata(pdev, port);

	/* Register the device */
	memset(&hw, 0, sizeof(hw));
	hw = (struct hw_pci) {
		.nr_controllers	= 1,
		.private_data	= (void **)&port,
		.setup		= sn_pcie_dme_setup,
		.map_irq	= of_irq_parse_and_map_pci,
		.scan		= sn_pcie_dme_scan_bus,
		.ops		= &sn_pcie_dme_ops,
	};

#ifdef CONFIG_PCI_MSI
	sn_pcie_dme_msi_chip.dev = port->dev;
	hw.msi_ctrl = &sn_pcie_dme_msi_chip;
#endif
	pci_common_init_dev(dev, &hw);

	return 0;
}

/**
 * sn_pcie_dme_remove - Remove function
 * @pdev: Platform device pointer
 *
 * Return: '0' always
 */
static int sn_pcie_dme_remove(struct platform_device *pdev)
{
	struct sn_pcie2_dme *port = platform_get_drvdata(pdev);

	sn_pcie_dme_free_irq_domain(port);

	return 0;
}

static void quirk_sn_pcie_dme_pci_fix_res(struct pci_dev *dev)
{
	struct pci_sys_data *sys = dev->sysdata;
	void __iomem *p;

	if (!card_bus)
		return;

	p = sn_pcie_dme_map_bus(card_bus, 0, 0);

	if (sys && sys->map_irq)
		pci_fixup_irqs(pci_common_swizzle, sys->map_irq);

	writel(0x2a000000, p + 0x10);

	if (pci_has_flag(PCI_PROBE_ONLY))
		return;

	pci_bus_size_bridges(dev->bus);
	pci_bus_assign_resources(dev->bus);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_FUJITSU_ME, PCI_ANY_ID,
			quirk_sn_pcie_dme_pci_fix_res);

static struct of_device_id sn_pcie_dme_of_match[] = {
	{ .compatible = "socionext,pcie2-dme-1-m8m", },
	{}
};

static struct platform_driver sn_pcie_dme_driver = {
	.driver = {
		.name = "sn-pcie2-dme-pcie",
		.of_match_table = sn_pcie_dme_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = sn_pcie_dme_probe,
	.remove = sn_pcie_dme_remove,
};
module_platform_driver(sn_pcie_dme_driver);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("Socionext PCIE2DME driver");
MODULE_LICENSE("GPL v2");

