/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/clkdev.h>

static DEFINE_SPINLOCK(_lock);

struct clk_device {
    struct clk_hw	hw;
	void __iomem	*reg_pmu;
    spinlock_t		*lock;
};

/*
static int moxart_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct moxart_clk *dev_clk = container_of(hw, struct moxart_clk,
						  fll_hw);
}
*/

static long moxart_round_rate(struct clk_hw *c_hw, unsigned long parent_rate, unsigned long *p)
{
	unsigned int mul, val, div, ret;
	struct clk_device *dev_clk = container_of(c_hw, struct clk_device, hw);

	mul = (readl(dev_clk->reg_pmu + 0x30) >> 3) & 0x1ff;
	val = (readl(dev_clk->reg_pmu + 0x0c) >> 4) & 0x7;
	switch (val) {
	case 0:
		div = 2;
		break;
	case 1:
		div = 3;
		break;
	case 2:
		div = 4;
		break;
	case 3:
		div = 6;
		break;
	case 4:
		div = 8;
		break;
	default:                                                                                                                                   
		div = 2;                                                                                                                               
		break;
	}
	ret = (38684 * mul + 10000) / (div * 10000);
	ret = (ret * 1000000) / 2;
	pr_info("%s: ret=%d mul=%d div=%d val=%d\n",
		__func__, ret, mul, div, val);
	return ret;	
}

static const struct clk_ops moxart_clk_ops = {
/*	.set_rate = moxart_set_rate,*/
	.round_rate = moxart_round_rate,
};


static __init void moxart_device_clk_init(struct device_node *node)
{
	struct clk *clk;
	struct clk_device *dev_clk;
	const char *clk_name = node->name;
	struct clk_init_data init;
	struct resource res_pmu;
	int err;
	
	pr_info("start %s\n", __func__);
   
	dev_clk = kzalloc(sizeof(*dev_clk), GFP_KERNEL);
	if (WARN_ON(!dev_clk))
		return;

	dev_clk->lock = &_lock;

	err = of_address_to_resource(node, 0, &res_pmu);
	if (err) {
		pr_err("could not get power management unit base resource\n");
		return;
	}
   
	dev_clk->reg_pmu = ioremap(res_pmu.start, resource_size(&res_pmu));
	if (IS_ERR(dev_clk->reg_pmu)) {
		pr_err("%s: ioremap res_pmu failed\n",
			__func__);
		return;
	}

	init.name = clk_name;
	init.ops = &moxart_clk_ops;
    init.flags = 0;
    init.num_parents = 0;

	dev_clk->hw.init = &init;

	clk = clk_register(NULL, &dev_clk->hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(dev_clk);
		return;
	}
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	clk_register_clkdev(clk, clk_name, NULL);

	pr_info("finished %s\n", __func__);
}
CLK_OF_DECLARE(moxart_clk_device, "moxa,moxart-device-clock", moxart_device_clk_init);

MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");
MODULE_DESCRIPTION("MOXA ART clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:moxart-clk");

