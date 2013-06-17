/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>

#include <asm/exception.h>

#include "irqchip.h"

#define IRQ_SOURCE_REG      0
#define IRQ_MASK_REG        0x04
#define IRQ_CLEAR_REG       0x08
#define IRQ_MODE_REG        0x0c
#define IRQ_LEVEL_REG       0x10
#define IRQ_STATUS_REG      0x14

#define FIQ_SOURCE_REG      0x20
#define FIQ_MASK_REG        0x24
#define FIQ_CLEAR_REG       0x28
#define FIQ_MODE_REG        0x2c
#define FIQ_LEVEL_REG       0x30
#define FIQ_STATUS_REG      0x34

#define IRQ_SOURCE(base_addr)   (base_addr + 0x00)
#define IRQ_MASK(base_addr)     (base_addr + 0x04)
#define IRQ_CLEAR(base_addr)    (base_addr + 0x08)
#define IRQ_TMODE(base_addr)    (base_addr + 0x0C)
#define IRQ_TLEVEL(base_addr)   (base_addr + 0x10)
#define IRQ_STATUS(base_addr)   (base_addr + 0x14)
#define FIQ_SOURCE(base_addr)   (base_addr + 0x20)
#define FIQ_MASK(base_addr)     (base_addr + 0x24)
#define FIQ_CLEAR(base_addr)    (base_addr + 0x28)
#define FIQ_TMODE(base_addr)    (base_addr + 0x2C)
#define FIQ_TLEVEL(base_addr)   (base_addr + 0x30)
#define FIQ_STATUS(base_addr)   (base_addr + 0x34)

static void __iomem *moxart_irq_base;
static struct irq_domain *moxart_irq_domain;
static unsigned int interrupt_mask;

asmlinkage void __exception_irq_entry moxart_handle_irq(struct pt_regs *regs);

void moxart_irq_ack(struct irq_data *irqd)
{
	unsigned int irq = irqd_to_hwirq(irqd);

	writel(1 << irq, IRQ_CLEAR(moxart_irq_base));
}

static void moxart_irq_mask(struct irq_data *irqd)
{
	unsigned int irq = irqd_to_hwirq(irqd);
	unsigned int mask;

	mask = readl(IRQ_MASK(moxart_irq_base));
	mask &= ~(1 << irq);
	writel(mask, IRQ_MASK(moxart_irq_base));
}

static void moxart_irq_unmask(struct irq_data *irqd)
{
	unsigned int irq = irqd_to_hwirq(irqd);
	unsigned int mask;

	mask = readl(IRQ_MASK(moxart_irq_base));
	mask |= (1 << irq);
	writel(mask, IRQ_MASK(moxart_irq_base));
}

static struct irq_chip moxart_irq_chip = {
	.name		= "moxart_irq",
	.irq_ack	= moxart_irq_ack,
	.irq_mask	= moxart_irq_mask,
	.irq_unmask	= moxart_irq_unmask,
	.irq_set_wake = NULL,
};

static int moxart_irq_map(struct irq_domain *d, unsigned int virq,
			 irq_hw_number_t hw)
{
	if ((1 << hw) && interrupt_mask) {
		irq_set_chip_and_handler(virq, &moxart_irq_chip,
			handle_edge_irq);
		pr_info("%s: irq_set_chip_and_handler edge virq=%d hw=%d\n",
			__func__, virq, (unsigned int) hw);
	} else {
		irq_set_chip_and_handler(virq, &moxart_irq_chip,
			handle_level_irq);
		pr_info("%s: irq_set_chip_and_handler level virq=%d hw=%d\n",
			__func__, virq, (unsigned int) hw);
	}

	set_irq_flags(virq, IRQF_VALID);

	return 0;
}

static struct irq_domain_ops moxart_irq_ops = {
	.map = moxart_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int __init moxart_of_init(struct device_node *node,
				struct device_node *parent)
{
	interrupt_mask = be32_to_cpup(of_get_property(node,
		"interrupt-mask", NULL));
	pr_debug("%s: interrupt-mask=%x\n", node->full_name, interrupt_mask);

	moxart_irq_base = of_iomap(node, 0);
	if (!moxart_irq_base)
		panic("%s: unable to map IC registers\n", node->full_name);

	moxart_irq_domain = irq_domain_add_linear(node,
		32, &moxart_irq_ops, NULL);

	if (!moxart_irq_domain)
		panic("%s: unable to create IRQ domain\n", node->full_name);

	writel(0, IRQ_MASK(moxart_irq_base));
	writel(0xffffffff, IRQ_CLEAR(moxart_irq_base));

	writel(interrupt_mask, IRQ_TMODE(moxart_irq_base));
	writel(interrupt_mask, IRQ_TLEVEL(moxart_irq_base));
	
	set_handle_irq(moxart_handle_irq);

	pr_info("%s: %s finished\n", node->full_name, __func__);

	return 0;
}
IRQCHIP_DECLARE(moxa_moxart_ic, "moxa,moxart-interrupt-controller",
	moxart_of_init);

asmlinkage void __exception_irq_entry moxart_handle_irq(struct pt_regs *regs)
{
	u32 irqstat;
	int hwirq;

	irqstat = readl(moxart_irq_base + IRQ_STATUS_REG);

	while (irqstat) {
		hwirq = ffs(irqstat) - 1;
		handle_IRQ(irq_find_mapping(moxart_irq_domain, hwirq), regs);
		irqstat &= ~(1 << hwirq);
	}
}


