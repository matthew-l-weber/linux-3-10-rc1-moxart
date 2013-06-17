/*
 * MOXA ART SoCs timer handling.
 *
 * Copyright (C) 2013 Jonas Jensen
 *
 * Jonas Jensen <jonas.jensen@gmail.com>
 *
 * Based on code from
 * Moxa Technology Co., Ltd. <www.moxa.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clocksource.h>

#include <asm/mach/time.h>

#define APB_CLK 48000000

#define TIMER_1_COUNT(base_addr)        (base_addr + 0x00)
#define TIMER_1_LOAD(base_addr)         (base_addr + 0x04)
#define TIMER_1_MATCH1(base_addr)       (base_addr + 0x08)
#define TIMER_1_MATCH2(base_addr)       (base_addr + 0x0C)

#define TIMER_2_COUNT(base_addr)        (base_addr + 0x10)
#define TIMER_2_LOAD(base_addr)         (base_addr + 0x14)
#define TIMER_2_MATCH1(base_addr)       (base_addr + 0x18)
#define TIMER_2_MATCH2(base_addr)       (base_addr + 0x1C)

#define TIMER_3_COUNT(base_addr)        (base_addr + 0x20)
#define TIMER_3_LOAD(base_addr)         (base_addr + 0x24)
#define TIMER_3_MATCH1(base_addr)       (base_addr + 0x28)
#define TIMER_3_MATCH2(base_addr)       (base_addr + 0x2C)

#define TIMER_CR(base_addr)             (base_addr + 0x30)

#define TIMER_1_CR_ENABLE(base_addr)    (base_addr + 0x30)
#define TIMER_1_CR_EXTCLK(base_addr)    (base_addr + 0x34)
#define TIMER_1_CR_FLOWIN(base_addr)    (base_addr + 0x38)

#define TIMER_2_CR_ENABLE(base_addr)    (base_addr + 0x42)
#define TIMER_2_CR_EXTCLK(base_addr)    (base_addr + 0x46)
#define TIMER_2_CR_FLOWIN(base_addr)    (base_addr + 0x50)

#define TIMER_3_CR_ENABLE(base_addr)    (base_addr + 0x54)
#define TIMER_3_CR_EXTCLK(base_addr)    (base_addr + 0x58)
#define TIMER_3_CR_FLOWIN(base_addr)    (base_addr + 0x62)

#define TIMER_INTR_STATE(base_addr)     (base_addr + 0x34)

#define TIMEREG_1_CR_ENABLE         (1 << 0)
#define TIMEREG_1_CR_CLOCK          (1 << 1)
#define TIMEREG_1_CR_INT            (1 << 2)
#define TIMEREG_2_CR_ENABLE         (1 << 3)
#define TIMEREG_2_CR_CLOCK          (1 << 4)
#define TIMEREG_2_CR_INT            (1 << 5)
#define TIMEREG_3_CR_ENABLE         (1 << 6)
#define TIMEREG_3_CR_CLOCK          (1 << 7)
#define TIMEREG_3_CR_INT            (1 << 8)
#define TIMEREG_COUNT_UP            (1 << 9)
#define TIMEREG_COUNT_DOWN          (0 << 9)

#define MAX_TIMER   2
#define USED_TIMER  1

#define TIMER1_COUNT                0x0
#define TIMER1_LOAD                 0x4
#define TIMER1_MATCH1               0x8
#define TIMER1_MATCH2               0xC
#define TIMER2_COUNT                0x10
#define TIMER2_LOAD                 0x14
#define TIMER2_MATCH1               0x18
#define TIMER2_MATCH2               0x1C
#define TIMER3_COUNT                0x20
#define TIMER3_LOAD                 0x24
#define TIMER3_MATCH1               0x28
#define TIMER3_MATCH2               0x2C
#define TIMER_INTR_MASK     0x38

static void __iomem *timer_base;

static irqreturn_t moxart_timer_interrupt(int irq, void *dev_id)
{
	timer_tick();
	return IRQ_HANDLED;
}

static struct irqaction moxart_timer_irq = {
	.name = "moxart-timer",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = moxart_timer_interrupt,
};

static void __init moxart_timer_init(struct device_node *node)
{
	int ret, irq;

	timer_base = of_iomap(node, 0);
	if (!timer_base)
		panic("%s: failed to map base\n", node->full_name);

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("%s: can't parse IRQ\n", node->full_name);

	ret = setup_irq(irq, &moxart_timer_irq);
	if (ret)
		pr_warn("%s: failed to setup IRQ %d\n", node->full_name, irq);


	writel(APB_CLK / HZ, TIMER_1_COUNT(timer_base));
	writel(APB_CLK / HZ, TIMER_1_LOAD(timer_base));

	writel(1, TIMER_1_CR_ENABLE(timer_base));
	writel(0, TIMER_1_CR_EXTCLK(timer_base));
	writel(1, TIMER_1_CR_FLOWIN(timer_base));

	pr_info("%s: count/load (APB_CLK=%d/HZ=%d) IRQ=%d\n",
		node->full_name, APB_CLK, HZ, irq);
}
CLOCKSOURCE_OF_DECLARE(moxart, "moxa,moxart-timer", moxart_timer_init);

