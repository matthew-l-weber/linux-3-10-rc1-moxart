/*
 * arch/arm/mach-moxart/moxart.c
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

#include <asm/mach/arch.h>

static const char * const moxart_dt_compat[] = {
	"moxa,moxart",
	NULL,
};

DT_MACHINE_START(MOXART, "MOXA UC-7112-LX")
	.dt_compat		= moxart_dt_compat,
MACHINE_END

