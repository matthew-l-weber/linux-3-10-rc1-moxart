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

static long moxart_round_rate(struct clk_hw *c_hw,
	unsigned long parent_rate, unsigned long *p)
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
	pr_debug("%s: ret=%d mul=%d div=%d val=%d\n",
		__func__, ret, mul, div, val);
	/* usually host->sysclk=77500000 mul=80 div=2 val=0 */
	return ret;
}

static const struct clk_ops moxart_clk_ops = {
	.round_rate = moxart_round_rate,
};

static const struct of_device_id moxart_pmu_match[] = {
	{ .compatible = "moxa,moxart-pmu" },
	{ },
};

static const struct of_device_id moxart_sysclk_match[] = {
	{ .compatible = "moxa,moxart-sysclk" },
	{ }
};

static int moxart_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *clk_node;
	struct clk *clk;
	struct clk_device *dev_clk;
	const char *clk_name;
	struct clk_init_data init;
	struct resource res_pmu;
	int err;

	dev_clk = devm_kzalloc(&pdev->dev, sizeof(*dev_clk), GFP_KERNEL);
	if (WARN_ON(!dev_clk))
		return -ENOMEM;

	dev_clk->lock = &_lock;

	err = of_address_to_resource(node, 0, &res_pmu);
	if (err) {
		dev_err(&pdev->dev, "could not get PMU base resource\n");
		return err;
	}

	dev_clk->reg_pmu = devm_ioremap_resource(&pdev->dev, &res_pmu);
	if (IS_ERR(dev_clk->reg_pmu)) {
		dev_err(&pdev->dev, "devm_ioremap_resource res_pmu failed\n");
		return PTR_ERR(dev_clk->reg_pmu);
	}

	clk_node = of_find_matching_node(NULL, moxart_sysclk_match);
	if (!clk_node)
		pr_err("%s: can't find DT node\n", clk_node->full_name);

	clk_name = clk_node->name;
	of_property_read_string(clk_node, "clock-output-names",
		&clk_name);

	init.name = clk_name;
	init.ops = &moxart_clk_ops;
	init.flags = 0;
	init.num_parents = 0;

	dev_clk->hw.init = &init;

	clk = devm_clk_register(&pdev->dev, &dev_clk->hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(dev_clk);
		return PTR_ERR(clk);
	}

	err = of_clk_add_provider(clk_node, of_clk_src_simple_get, clk);

	/*	calling of_clk_init here means it can be removed from
		init_machine which can then itself be removed entirely.
		this needs to happen or fixed-clock "apb_clk" will not
		get registered */
	/*	of_clk_init(NULL); unfortunately this means __init
		section mismatch */

	dev_info(&pdev->dev, "finished %s\n", __func__);

	return err;
}

static int moxart_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static struct platform_driver moxart_clk_driver = {
	.probe = moxart_clk_probe,
	.remove = moxart_clk_remove,
	.driver = {
		.name			= "moxart-sysclk",
		.owner			= THIS_MODULE,
		.of_match_table	= moxart_pmu_match,
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

MODULE_ALIAS("platform:moxart-sysclk");
MODULE_DESCRIPTION("MOXA ART clock driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");

