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

static __iomem void *reg_pmu;

struct moxart_clk {
	struct clk_hw fll_hw;
	struct clk *fll;
};

/*
static int moxart_fll_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct moxart_clk *clkdata = container_of(hw, struct moxart_clk,
						  fll_hw);
}
*/

static long moxart_fll_round_rate(struct clk_hw *hw, unsigned long parent_rate, unsigned long *p)
{
	unsigned int mul, val, div, ret;
    mul = (readl(reg_pmu + 0x30) >> 3) & 0x1ff;
    val = (readl(reg_pmu + 0x0c) >> 4) & 0x7;
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

static const struct clk_ops moxart_fll_ops = {
/*	.set_rate = moxart_fll_set_rate,*/
	.round_rate = moxart_fll_round_rate,
};

static struct clk_init_data moxart_fll_init = {
	.name = "fll",
	.ops = &moxart_fll_ops,
	.flags = CLK_SET_RATE_GATE,
};

/*static int moxart_clk_probe(struct platform_device *pdev)*/
static __init void moxart_device_clk_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	struct moxart_clk *clkdata;
/*	struct device *dev = &pdev->dev;
    struct device_node *node = dev->of_node;*/
    struct resource res_pmu;
    int err;
   
    err = of_address_to_resource(node, 0, &res_pmu);
    if (err) {
/*        dev_err(dev, "could not get power management unit base resource\n");*/
		pr_err("could not get power management unit base resource\n");
/*        return err;*/
    }
   
    /*reg_pmu = devm_ioremap_resource(dev, &res_pmu);
    if (IS_ERR(reg_pmu)) {
        dev_err(dev, "%s: devm_ioremap_resource res_pmu failed\n",
            __func__);
        return PTR_ERR(reg_pmu);
    }*/

	/*clkdata = devm_kzalloc(&pdev->dev, sizeof(*clkdata), GFP_KERNEL);*/
	clkdata = kzalloc(sizeof(*clkdata), GFP_KERNEL);
	if (WARN_ON(!clkdata))
        return;

/*	if (!clkdata)
		return -ENOMEM;*/

	clkdata->fll_hw.init = &moxart_fll_init;
	
	/*clkdata->fll = devm_clk_register(&pdev->dev, &clkdata->fll_hw);
	if (IS_ERR(clkdata->fll))
		return PTR_ERR(clkdata->fll);*/

	clk = clk_register(NULL, &clkdata->fll_hw);
    if (WARN_ON(IS_ERR(clk))) {
        kfree(clkdata);
        return;
    }
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
    clk_register_clkdev(clk, clk_name, NULL);

/*	dev_set_drvdata(&pdev->dev, clkdata);*/

/*	dev_info(&pdev->dev, "finished %s\n", __func__);*/

	pr_info("finished %s\n", __func__);

/*	return 0;*/
}
CLK_OF_DECLARE(moxart_clk_device, "moxa,moxart-device-clock", moxart_device_clk_init);

/*static int moxart_clk_remove(struct platform_device *pdev)
{
	return 0;
}*/

/*static struct platform_driver moxart_clk_driver = {
	.probe = moxart_clk_probe,
	.remove = moxart_clk_remove,
	.driver		= {
		.name	= "moxart-clk",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(moxart_clk_driver);*/

MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");
MODULE_DESCRIPTION("MOXA ART clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:moxart-clk");
