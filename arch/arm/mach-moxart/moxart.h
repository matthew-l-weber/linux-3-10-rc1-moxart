/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

extern __iomem void *reg_wdt;
extern void moxart_timer_init(void);
extern void moxart_init_irq(void);
extern void moxart_handle_irq(struct pt_regs *regs);

