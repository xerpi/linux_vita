/*
 * SMP support for the PlayStation Vita
 *
 * Copyright (C) 2018 Sergi Granell
 *
 * Cloned from arch/arm/mach-sti/platsmp.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/memory.h>
#include <linux/smp.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#define SECONDARY_STARTUP_ADDR	0x1F007F00

int vita_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *boot_addr;
	const u32 secondary_boot_addr = SECONDARY_STARTUP_ADDR + cpu * 4;

	boot_addr = ioremap((phys_addr_t)secondary_boot_addr, sizeof(phys_addr_t));
	if (!boot_addr) {
		pr_warn("unable to ioremap boot address for cpu %u\n", cpu);
		return -ENOMEM;
	}

	writel_relaxed(__pa_symbol(secondary_startup), boot_addr);

	/* Ensure the write is visible to the secondary core */
	smp_wmb();
	/* Wakeup secondary cores */
	sev();

	iounmap(boot_addr);

	return 0;
}

static void __init vita_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *scu_base;

	scu_base = ioremap(scu_a9_get_base(), SZ_256);
	scu_enable(scu_base);
	iounmap(scu_base);
}

static const struct smp_operations vita_smp_ops __initconst = {
	.smp_prepare_cpus	= vita_smp_prepare_cpus,
	.smp_boot_secondary	= vita_smp_boot_secondary,
};
CPU_METHOD_OF_DECLARE(vita_smp, "vita,smp", &vita_smp_ops);
