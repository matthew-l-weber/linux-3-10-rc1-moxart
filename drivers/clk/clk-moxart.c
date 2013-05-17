/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
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

static int moxart_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct clk *clk;
	struct clk_device *dev_clk;
	const char *clk_name;
	const char *parent_name;
	struct clk_init_data init;
	struct resource res_pmu;
	int err;
	
	dev_info(&pdev->dev, "start %s\n", __func__);
   
	dev_clk = devm_kzalloc(&pdev->dev, sizeof(*dev_clk), GFP_KERNEL);
	if (WARN_ON(!dev_clk))
		return -ENOMEM;

	dev_clk->lock = &_lock;

	err = of_address_to_resource(node, 0, &res_pmu);
	if (err) {
		dev_err(&pdev->dev, "could not get power management unit base resource\n");
		return err;
	}
   
	dev_clk->reg_pmu = devm_ioremap_resource(&pdev->dev, &res_pmu);
	if (IS_ERR(dev_clk->reg_pmu)) {
		dev_err(&pdev->dev, "devm_ioremap_resource res_pmu failed\n");
		return PTR_ERR(dev_clk->reg_pmu);
	}

/*	parent_name = of_clk_get_parent_name(pdev->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;*/

	clk_name = pdev->dev.of_node->name;
	of_property_read_string(pdev->dev.of_node, "clock-output-names",
		&clk_name);
	clk_name = "sys_clk";
	
	dev_info(&pdev->dev, "clk_name=%s\n", clk_name);

	init.name = clk_name;
	init.ops = &moxart_clk_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 0;
	
	dev_clk->hw.init = &init;

	clk = devm_clk_register(&pdev->dev, &dev_clk->hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(dev_clk);
		return PTR_ERR(clk);
	}

	err = of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, clk);

	dev_info(&pdev->dev, "finished %s\n", __func__);

	return err;
}
/*CLK_OF_DECLARE(moxart_clk_device, "moxa,moxart-device-clock", moxart_device_clk_init);*/

static int moxart_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
   
	return 0;
}

static const struct of_device_id moxart_clk_match[] = {
	{ .compatible = "moxa,moxart-pmu" },
	{ },
};
/*MODULE_DEVICE_TABLE(of, moxart_clk_ids);*/

static struct platform_driver moxart_clk_driver = {
	.probe = moxart_clk_probe,
	.remove = moxart_clk_remove,
	.driver = {
		.name			= "moxart-sysclk",
		.owner			= THIS_MODULE,
		.of_match_table	= moxart_clk_match,
	},
};

static int __init moxart_clk_init(void)
{
	return platform_driver_register(&moxart_clk_driver);
}

static void __exit moxart_clk_exit(void)
{
	platform_driver_unregister(&moxart_clk_driver);
}

postcore_initcall(moxart_clk_init);
module_exit(moxart_clk_exit)

module_platform_driver(moxart_clk_driver);

MODULE_ALIAS("platform:moxart-sysclk");
MODULE_DESCRIPTION("MOXA ART clock driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");

