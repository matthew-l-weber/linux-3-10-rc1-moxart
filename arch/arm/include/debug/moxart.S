/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#define MOXART_UART_DEBUG_PHYS_BASE		0x98200000

.macro	addruart, rp, rv, tmp
	ldr	\rp, =MOXART_UART_DEBUG_PHYS_BASE	@ physical
.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
