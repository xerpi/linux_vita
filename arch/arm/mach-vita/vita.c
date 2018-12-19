/*
 * Device Tree support for PlayStation Vita
 *
 * Copyright (C) 2018 Sergi Granell
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>

#include <asm/mach/arch.h>

static const char * const vita_dt_compat[] = {
	"vita,vita",
	NULL,
};

DT_MACHINE_START(VITA_DT, "PlayStation Vita (Device Tree Support)")
	.dt_compat	= vita_dt_compat,
MACHINE_END
