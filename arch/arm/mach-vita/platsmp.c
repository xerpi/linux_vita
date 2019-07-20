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
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#define SECONDARY_STARTUP_ADDR	0x1F007F00

/* XXX vita_pen_release is cargo culted code - DO NOT COPY XXX */
volatile int vita_pen_release = -1;

extern void vita_secondary_startup(void);

/*
 * XXX CARGO CULTED CODE - DO NOT COPY XXX
 *
 * Write vita_pen_release in a way that is guaranteed to be visible to
 * all observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void vita_write_pen_release(int val)
{
	vita_pen_release = val;
	smp_wmb();
	sync_cache_w(&vita_pen_release);
}

static DEFINE_SPINLOCK(boot_lock);

void vita_smp_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	vita_write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int vita_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting vita_pen_release.
	 *
	 * Note that "vita_pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	vita_write_pen_release(cpu);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (vita_pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return vita_pen_release != -1 ? -ENOSYS : 0;
}


static void __init vita_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *boot_addr;
	void __iomem *scu_base;

	scu_base = ioremap(scu_a9_get_base(), SZ_256);
	scu_enable(scu_base);
	iounmap(scu_base);

	boot_addr = ioremap((phys_addr_t)SECONDARY_STARTUP_ADDR, SZ_256);
	__raw_writel(__pa_symbol(vita_secondary_startup),
		boot_addr);
	iounmap(boot_addr);
}


static const struct smp_operations vita_smp_ops __initconst = {
	.smp_prepare_cpus	= vita_smp_prepare_cpus,
	.smp_secondary_init	= vita_smp_secondary_init,
	.smp_boot_secondary	= vita_smp_boot_secondary,
};
CPU_METHOD_OF_DECLARE(vita_smp, "vita,smp", &vita_smp_ops);
