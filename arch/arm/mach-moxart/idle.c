/*
 * arch/arm/mach-moxart/idle.c
 *
 * (C) Copyright 2013, Jonas Jensen <jonas.jensen@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/of.h>

#include <asm/system.h>
#include <asm/proc-fns.h>
#include <asm/system_misc.h>

static void moxart_idle(void)
{
}

static const struct of_device_id moxart_match[] = {
	   { .compatible = "moxa,moxart" },
	   { }
};

static int __init moxart_idle_init(void)
{
	struct device_node *node;

	node = of_find_matching_node(NULL, moxart_match);
	if (!node)
		return -ENODEV;

	arm_pm_idle = moxart_idle;
	return 0;
}

/*
arch_initcall(moxart_idle_init);
*/
