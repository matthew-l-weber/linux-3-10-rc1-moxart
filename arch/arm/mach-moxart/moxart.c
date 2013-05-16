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

#include "moxart.h"

static void __init moxart_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char * const moxart_board_dt_compat[] = {
	"moxa,moxart-uc-7112-lx",
	NULL,
};

DT_MACHINE_START(MOXART, "MOXA UC-7112-LX")
	.init_irq		= moxart_init_irq,
	.init_time		= moxart_timer_init,
	.init_machine	= moxart_init,
	.handle_irq		= moxart_handle_irq,
	.dt_compat		= moxart_board_dt_compat,
	.nr_irqs		= 32,
MACHINE_END

