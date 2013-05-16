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
	/*
	 * Because of broken hardware we have to enable interrupts or the CPU
	 * will never wakeup... Acctualy it is not very good to enable
	 * interrupts first since scheduler can miss a tick, but there is
	 * no other way around this. Platforms that needs it for power saving
	 * should call enable_hlt() in init code, since by default it is
	 * disabled.
	 */
		
	/*	
	comment two lines below from gemini (text above also from gemini).
	leaving this an empty function is the only way the kernel boots all the way to init. 
	( http://lists.infradead.org/pipermail/linux-arm-kernel/2013-May/168472.html )
	
	local_irq_enable();
	cpu_do_idle();
	*/
}

static int __init moxart_idle_init(void)
{
	arm_pm_idle = moxart_idle;
	return 0;
}

arch_initcall(moxart_idle_init);
