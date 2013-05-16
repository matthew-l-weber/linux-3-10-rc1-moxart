/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>

static const char * const moxart_dt_compat[] = {
	"moxa,moxart-uc-7112-lx",
	NULL,
};

DT_MACHINE_START(MOXART, "MOXA UC-7112-LX")
	.dt_compat		= moxart_dt_compat,
MACHINE_END

