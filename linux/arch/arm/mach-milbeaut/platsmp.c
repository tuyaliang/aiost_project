/*
 * arch/arm/mach-milbeaut/platsmp.c
 * Copyright:	(C) 2013-2015 Socionext
 * Copyright:	(C) 2015 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/arm-cci.h>
#include <linux/clkdev.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/platform_device.h>

#include <asm/cp15.h>
#include <asm/cputype.h>
#include <asm/suspend.h>
#include <asm/idmap.h>
#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>

#define MLB01_MAX_CPU	4

#define KERNEL_UNBOOT_FLAG	0x12345678
#define CPU_FINISH_SUSPEND_FLAG 0x56784321

#define CPU_RESUME_ADDRESS_OFFSET   (MLB01_MAX_CPU * 4)
#define CPU_FINISH_SUSPEND_OFFSET   (CPU_RESUME_ADDRESS_OFFSET * 2 + 8)

u32 trampoline_phys;
static void __iomem *trampoline;

extern void mlb01_cpu_entry(unsigned long secondary_entry);

static int mlb01_boot_secondary(unsigned int l_cpu, struct task_struct *idle)
{
	unsigned int mpidr, cpu, cluster;
	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	if (cpu >= MLB01_MAX_CPU)
		return -EINVAL;

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);

	writel(virt_to_phys(secondary_startup), trampoline + cpu * 4);
	arch_send_wakeup_ipi_mask(cpumask_of(l_cpu));

	return 0;
}

static void mlb01_cpu_die(unsigned int l_cpu)
{
	unsigned int mpidr, cpu;
	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	/* Set data $ to invalid */
	asm volatile
		(".arch	armv7-a \n\t"
		 "stmfd	sp!, {fp, ip} \n\t"
		 "mrc	p15, 0, r0, c1, c0, 0	@ get SCTLR \n\t"
		 "bic	r0, r0, #"__stringify(CR_C)" \n\t"
		 "mcr	p15, 0, r0, c1, c0, 0	@ set SCTLR \n\t"
		 "isb	\n\t"
		 "bl	v7_flush_dcache_louis \n\t"
		 "ldmfd	sp!, {fp, ip}"
		 : : : "r0","r1","r2","r3","r4","r5","r6","r7",
		 "r9","r10","lr","memory" );

	writel(KERNEL_UNBOOT_FLAG, trampoline + cpu * 4);

	/* Now we are prepared for power-down, do it: */
	wfi();
	/* Enable data $ */
	asm volatile
		("mrc	p15, 0, r0, c1, c0, 0	@ get SCTLR \n\t"
		 "orr	r0, r0, #"__stringify(CR_C)" \n\t"
		 "mcr	p15, 0, r0, c1, c0, 0	@ set SCTLR");
}

static int mlb01_cpu_kill(unsigned int l_cpu)
{
	unsigned int mpidr, cpu;

	mpidr = cpu_logical_map(l_cpu);
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	writel(KERNEL_UNBOOT_FLAG,
		trampoline + cpu * 4);

	/* we also need to send some signal to M0/RTOS to power off this cpu */

	return 1;
}

static struct smp_operations __initdata mlb01_smp_ops = {
	.smp_boot_secondary	= mlb01_boot_secondary,
	.cpu_die		= mlb01_cpu_die,
	.cpu_kill		= mlb01_cpu_kill,
};

static int __init mlb01_smp_init(void)
{
	unsigned int mpidr, cpu, cluster;
	struct device_node *np;
	struct resource res;

	np = of_find_compatible_node(NULL, NULL, "socionext,milbeaut-evb");
	if (!np || !of_device_is_available(np))
		return -ENODEV;
	of_node_put(np);

	np = of_find_compatible_node(NULL, NULL, "socionext,smp-trampoline");
	if (!np)
		return -ENODEV;

	if (of_address_to_resource(np, 0, &res))
		return -ENODEV;
	trampoline_phys = res.start;

	trampoline = of_iomap(np, 0);
	if (!trampoline)
		return -ENODEV;
	of_node_put(np);

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	printk("MCPM boot on cpu_%u cluster_%u\n", cpu, cluster);

	for (cpu = 0; cpu < MLB01_MAX_CPU; cpu++)
		writel(KERNEL_UNBOOT_FLAG, trampoline + cpu * 4);

	smp_set_ops(&mlb01_smp_ops);

	return 0;
}
early_initcall(mlb01_smp_init);

static int mlb01_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM);
}

#ifdef CONFIG_PM_WARP
static int mlb01_die(unsigned long arg)
{
	unsigned int mpidr, cpu, cluster;
	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	writel(virt_to_phys(cpu_resume), trampoline + CPU_RESUME_ADDRESS_OFFSET + cpu * 4);
	flush_cache_all();

	/* Notify that can enter suspend mode*/
	writel(CPU_FINISH_SUSPEND_FLAG, trampoline + CPU_FINISH_SUSPEND_OFFSET);

	/* M0 or RTOS should power-off, and then reset upon resume */
	while (1)
		asm("wfi");

	/* never reach here*/
	return -1;
}

#else	// Emulated suspend/resume
typedef void (*phys_reset_t)(unsigned long);
static phys_reset_t phys_reset;

static int mlb01_die(unsigned long arg)
{
	setup_mm_for_reboot();
	asm("wfi");
	/* Boot just like a secondary */
	phys_reset = (phys_reset_t)(unsigned long)virt_to_phys(cpu_reset);
	phys_reset(virt_to_phys(cpu_resume));

	return 0;
}
#endif

static int mlb01_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		pr_err("STANDBY\n");
		asm("wfi");
		break;
	case PM_SUSPEND_MEM:
		pr_err("SUSPEND\n");
		cpu_pm_enter();
		cpu_suspend(0, mlb01_die);
		cpu_pm_exit();
		break;
	}
	return 0;
}

static const struct platform_suspend_ops mlb01_pm_ops = {
	.valid		= mlb01_pm_valid,
	.enter		= mlb01_pm_enter,
};

struct clk *mlb01_clclk_register(struct device *cpu_dev);

static int __init mlb01evb_pm_init(void)
{
	suspend_set_ops(&mlb01_pm_ops);

	return 0;
}
late_initcall(mlb01evb_pm_init);
