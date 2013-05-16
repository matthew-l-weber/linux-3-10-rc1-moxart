/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/init.h>
#include <asm/system.h>
#include <asm/proc-fns.h>

static void moxart_idle(void)
{	
}

static int __init moxart_idle_init(void)
{
	arm_pm_idle = moxart_idle;
	return 0;
}

arch_initcall(moxart_idle_init);
