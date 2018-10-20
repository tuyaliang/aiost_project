/*
 * m8m generic powerdomain support
 * Copyright (C) 2014-2015 Linaro, Ltd  Andy Green <andy.green@linaro.org>
 *
 * based on -->
 *
 * Exynos Generic power domain support.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Implementation of Exynos specific power domain control which is used in
 * conjunction with runtime-pm. Support for both device-tree and non-device-tree
 * based power domain support is included.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
/* #define DEBUG */
#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/clk.h>

enum domains {
	M8M_DOM_ALWAYS_ON,
	M8M_DOM_SD0,
	M8M_DOM_NETSEC,
	M8M_DOM_NF,
	M8M_DOM_USB,
	M8M_DOM_PCIE0,
	M8M_DOM_PCIE1,
	M8M_DOM_HDMI,
	M8M_DOM_SD1,
};

enum integration_ips {
	INTEG_TOP_JTSCR,
	INTEG_GPV,
	INTEG_NIC,
	INTEG_EXS,

	/* always last */
	INTEG_COUNT
};

enum regs {
	REG_EXS_NFBOOT		= 8,
	REG_EXS_SOFRESET	= 0xc,
	REG_EXS_RAMPD		= 0x40,
	REG_EXS_PCIE		= 0x60,
	REG_EXS_NETSEC		= 0x70
};

struct m8m_pm_domain_context {
	void __iomem *integration[INTEG_COUNT];
};

struct m8m_pm_domain {
	struct generic_pm_domain pd;
	char const *name;
	enum domains domain;
	bool is_off;
	struct m8m_pm_domain_context *context;
	struct clk *clks[8];
	int count_clks;
};

static void m8m_writel(struct m8m_pm_domain *pd,
			enum integration_ips n, u32 ofs, u32 reset, u32 set)
{
	writel((readl(pd->context->integration[n] + ofs) & ~(reset)) | set,
	       pd->context->integration[n] + ofs);
}

static int m8m_pd_power_on(struct generic_pm_domain *domain)
{
	struct m8m_pm_domain *pd;
	int n;

	pd = container_of(domain, struct m8m_pm_domain, pd);

	pr_debug("%s: domain %s\n", __func__, domain->name);

	for (n = 0; n < pd->count_clks; n++)
		clk_prepare_enable(pd->clks[n]);

	switch (pd->domain) {
	case M8M_DOM_SD0:
		/* sd0 card detect out of reset */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, BIT(8), 0);
		/* EXS_SRAM_PD b0 = 0 = enable SD0 SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(0), 0);
		udelay(1);
		/* Drive strength (replace when pinmux avail) */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x20, 0xf, 5);
		break;
	case M8M_DOM_NETSEC:
		/* enable NETSEC PTPSTP clock */
		m8m_writel(pd, INTEG_EXS, REG_EXS_NETSEC, BIT(0), 0);
		/* EXS_SRAM_PD b4 = 0 = enable NETSEC SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(4), 0);
		udelay(1);
		/* bring it out of reset */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, BIT(2), 0);
		break;
	case M8M_DOM_NF:
		/* XDMAC operational mode */
		m8m_writel(pd, INTEG_EXS, 0, 0, BIT(0));
		/* EXS_SRAM_PD b2 = 0 = enable NF SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(2), 0);
		udelay(1); /* required in sect 5.2 of EXS pdf */
		/* NANDflash bootstrap */
		m8m_writel(pd, INTEG_EXS, REG_EXS_NFBOOT,
				       BIT(3) | BIT(2) | BIT(1) | BIT(0),
				       BIT(4));
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(1) | BIT(0));
		udelay(2000);
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, BIT(1) | BIT(0), 0);
		udelay(1);
		break;
	case M8M_DOM_USB:
		/* EXS_SRAM_PD b5 = 0 = enable USB SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(5), 0);
		udelay(1);
		break;
	case M8M_DOM_PCIE0:
		/* select PCI mode not AU2 */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0xc, 0, BIT(31));
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(6) | BIT(5));
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(7) | BIT(6), 0);
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, BIT(6) | BIT(5), 0);
		/* b0 is bifurication enable when set */
		m8m_writel(pd, INTEG_EXS, REG_EXS_PCIE, BIT(8) | BIT(0), 0);

		/* set pu0-3 to managed by pcie ip */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x2244, 0, BIT(3) | BIT(2) | BIT(1) | BIT(0));
		break;
	case M8M_DOM_PCIE1:
		break;
	case M8M_DOM_HDMI:
		/* hdmi phy reset */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x1044, BIT(1), 0);
		break;
	case M8M_DOM_SD1:
		clk_set_rate(pd->clks[0], 1512000000 / 40);
		/* sd0 card detect out of reset */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, BIT(9), 0);
		/* EXS_SRAM_PD b0 = 0 = enable SD0 SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, BIT(1), 0);
		udelay(1);
		/* Drive strength (replace when pinmux avail) */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x20, 0xf, 5);
		break;

	default:
		break;
	}

	pd->is_off = false;

	return 0;
}

static int m8m_pd_power_off(struct generic_pm_domain *domain)
{
	struct m8m_pm_domain *pd;
	int n;

	pd = container_of(domain, struct m8m_pm_domain, pd);

	pr_debug("%s: domain %s\n", __func__, domain->name);

	switch (pd->domain) {
	case M8M_DOM_SD0:
		/* Drive strength (replace when pinmix avail) */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x20, 0xf, 5);
		/* disable  SD0 SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(0));
		udelay(1);
		/* sd0 card detect reset */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(8));
		break;
	case M8M_DOM_NETSEC:
		/*
		 * ogma has a problem dealing with restart after reset
		 * that's a big pain because otherwise we could load the
		 * microcode when the netdev is opened (after userland is
		 * up and we can load firmware...)
		 */
		/* hold it in reset */
//		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(2));
		/* disable NETSEC PTPSTP clock */
		m8m_writel(pd, INTEG_EXS, REG_EXS_NETSEC, 0, BIT(0));
		/* EXS_SRAM_PD b4 = 0 = disable NETSEC SRAM */
		//m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(4));
		break;
	case M8M_DOM_NF:
		/* reset it */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(1) | BIT(0));
		/* XDMAC operational mode */
		m8m_writel(pd, INTEG_EXS, 0, BIT(0), 0);
		/* EXS_SRAM_PD b2 = 0 = disable NF SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(2));
		udelay(1);
		break;
	case M8M_DOM_USB:
		/* EXS_SRAM_PD b5 = 0 = enable USB SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(5));
		udelay(1);
		break;
	case M8M_DOM_PCIE0:
		m8m_writel(pd, INTEG_EXS, REG_EXS_PCIE, 0, BIT(8));
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(6) | BIT(5));
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(7) | BIT(6));
		/* set pu0-3 to not be pcie */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x2244, BIT(3) | BIT(2) | BIT(1) | BIT(0), 0);
		break;
	case M8M_DOM_PCIE1:
		break;
	case M8M_DOM_HDMI:
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x1044, 0, BIT(1));
		break;
	case M8M_DOM_SD1:
		/* Drive strength (replace when pinmix avail) */
		m8m_writel(pd, INTEG_TOP_JTSCR, 0x20, 0xf, 5);
		/* disable  SD0 SRAM */
		m8m_writel(pd, INTEG_EXS, REG_EXS_RAMPD, 0, BIT(1));
		udelay(1);
		/* sd0 card detect reset */
		m8m_writel(pd, INTEG_EXS, REG_EXS_SOFRESET, 0, BIT(9));
		break;

	default:
		break;
	}
	if (pd->domain)
		for (n = pd->count_clks - 1; n >= 0; n--)
			clk_disable_unprepare(pd->clks[n]);

	pd->is_off = true;

	return 0;
}

static struct genpd_onecell_data m8m_pd;

static int m8m_add_sub_domain(struct device_node *master,
			struct device_node *sub)
{
	struct m8m_pm_domain *master_pd, *sub_pd;
	int ret;

	sub_pd = platform_get_drvdata(of_find_device_by_node(sub));
	master_pd = platform_get_drvdata(of_find_device_by_node(master));

	pr_debug("sub-domain:%d, master-domain:%d.\n"
		, sub_pd->domain, master_pd->domain);

	while (1) {
		ret = pm_genpd_add_subdomain(&master_pd->pd, &sub_pd->pd);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}

	return 0;
}


int m8m_pm_suspend_noirq(struct device *dev)
{
	if (dev->driver && dev->driver->pm && dev->driver->pm->suspend_noirq)
		return dev->driver->pm->suspend_noirq(dev);

	return 0;
}
int m8m_pm_resume_noirq(struct device *dev)
{
	if (dev->driver && dev->driver->pm && dev->driver->pm->resume_noirq)
		return dev->driver->pm->resume_noirq(dev);

	return 0;
}

int m8m_attach_dev(struct generic_pm_domain *domain,
			  struct device *dev)
{
	dev->pm_domain->ops.suspend_noirq = m8m_pm_suspend_noirq;
	dev->pm_domain->ops.resume_noirq = m8m_pm_resume_noirq;

	return 0;
}

static __init int m8m_pm_init_power_domain(void)
{
	struct device_node *np, *child = NULL, *master;
	int ret, pd_num, n;
	struct platform_device *pdev;
	struct m8m_pm_domain_context *context;

	pr_debug("%s\n", __func__);

	np = of_find_compatible_node(NULL, NULL, "socionext,m8m-pd");
	if (np == NULL) {
		pr_err("find no power domain device.\n");
		return -ENOMEM;
	}	

	context = kzalloc(sizeof(*context), GFP_KERNEL);
	for (n = 0; n < INTEG_COUNT; n++)
		context->integration[n] = of_iomap(np, n);

	/* remapNAND, XDMAC, AHB_BM assets + NETSEC */
	writel(0xff, context->integration[INTEG_GPV]);
	/* remap internal sram to address zero. */
	writel(1, context->integration[INTEG_NIC]);

	/* populate power-domain cell device */
	pdev = of_find_device_by_node(np);
	ret = of_platform_populate(np, NULL, NULL, &pdev->dev);
	if (ret) {
		pr_err("find no power-domain-cell device.\n");
		return -ENOMEM;
	}

	/* count for power domain number */
	pd_num = 0;
	for (child = of_get_next_child(np, NULL); child;
			child = of_get_next_child(np, child))
		pd_num++;

	pr_debug("m8m power domain count is %d.\n", pd_num);
	m8m_pd.domains = kmalloc(pd_num*sizeof(struct generic_pm_domain *),
			GFP_KERNEL);
	m8m_pd.num_domains = pd_num;

	pd_num = 0;
	for (child = of_get_next_child(np, NULL); child;
					child = of_get_next_child(np, child)) {
		struct m8m_pm_domain *pd;
		struct platform_device *child_pdev =
				of_find_device_by_node(child);

		pd = kzalloc(sizeof(*pd), GFP_KERNEL);
		if (!pd) {
			pr_err("%s: failed to allocate memory for domain-cell.\n"
				, __func__);
			return -ENOMEM;
		}
		pd->pd.name = kstrdup(child->name, GFP_KERNEL);
		pd->pd.power_off = m8m_pd_power_off;
		pd->pd.power_on = m8m_pd_power_on;
		pd->pd.attach_dev = m8m_attach_dev;
		pd->name = pd->pd.name;
		pd->context = context;
		if (of_property_read_u32(child, "index"
					, &pd->domain))
			pd->domain = -1;
		m8m_pd.domains[pd_num] = &pd->pd;

		pd->count_clks = 0;
		while (!IS_ERR_OR_NULL(pd->clks[pd->count_clks] = 
				       of_clk_get(child, pd->count_clks)) &&
		       pd->count_clks < ARRAY_SIZE(pd->clks))
			pd->count_clks++;

		pm_genpd_init(&pd->pd, NULL, true);
		platform_set_drvdata(child_pdev, pd);

		if (!pd->domain)
			m8m_pd_power_on(&pd->pd);

		pr_debug("power domain %s starting.\n", child->name);
		pd_num++;
	}

	/* register sub-domain if necessary */
	for  (child = of_get_next_child(np, NULL); child;
				child = of_get_next_child(np, child)) {
		master = of_parse_phandle(child, "master-domain-cell", 0);
		if (master == NULL)
			continue;

		m8m_add_sub_domain(master, child);
	}

	if (of_genpd_add_provider_onecell(np, &m8m_pd)) {
		pr_err("m8m power domain initialization failed.\n");
		return -1;
	}

	return 0;
}
arch_initcall(m8m_pm_init_power_domain);
