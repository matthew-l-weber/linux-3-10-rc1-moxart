/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/timer.h>

#define SW_READY_GPIO   (27)
#define SW_RESET_GPIO   (25)

#define GPIO_DATA_OUT(base_addr)            (base_addr + 0x00)
#define GPIO_DATA_IN(base_addr)             (base_addr + 0x04)
#define GPIO_PIN_DIRECTION(base_addr)       (base_addr + 0x08)

static void __iomem *moxart_pmu_base;
static void __iomem *moxart_gpio_base;

void moxart_gpio_enable(u32 gpio)
{
	writel(readl(moxart_pmu_base) | gpio, moxart_pmu_base);
}

void moxart_gpio_disable(u32 gpio)
{
	writel(readl(moxart_pmu_base) & ~gpio, moxart_pmu_base);
}

static int moxart_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	moxart_gpio_enable(1 << offset);
	return pinctrl_request_gpio(offset);
}

static void moxart_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(offset);
	moxart_gpio_disable(1 << offset);
}

static int moxart_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *ioaddr = GPIO_PIN_DIRECTION(moxart_gpio_base);

	writel(readl(ioaddr) & ~(1 << offset), ioaddr);
	return 0;
}

static int moxart_gpio_direction_output(struct gpio_chip *chip,
	unsigned offset, int value)
{
	void __iomem *ioaddr = GPIO_PIN_DIRECTION(moxart_gpio_base);

	writel(readl(ioaddr) | (1 << offset), ioaddr);
	return 0;
}

static void moxart_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 reg;
	void __iomem *ioaddr = GPIO_DATA_OUT(moxart_gpio_base);

	reg = readl(ioaddr);
	pr_debug("%s: offset=%x value=%d ioaddr=%p reg=%x\n",
		__func__, offset, value, ioaddr, reg);
	(value) ?	writel(reg | (1 << offset), ioaddr) :
				writel(reg & ~(1 << offset), ioaddr);
}

static int moxart_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 ret;

	ret = readl(GPIO_PIN_DIRECTION(moxart_gpio_base));
	if (ret & (1 << offset)) {
		pr_debug("%s: readl(%p)=%x\n",
			__func__, GPIO_DATA_OUT(moxart_gpio_base),
		readl(GPIO_DATA_OUT(moxart_gpio_base)));
		ret = readl(GPIO_DATA_OUT(moxart_gpio_base)) & (1 << offset);
	} else {
		ret = readl(GPIO_DATA_IN(moxart_gpio_base)) & (1 << offset);
	}

	return ret;
}

static struct gpio_chip moxart_gpio_chip = {
	.label				= "moxart-gpio",
	.request			= moxart_gpio_request,
	.free				= moxart_gpio_free,
	.direction_input	= moxart_gpio_direction_input,
	.direction_output	= moxart_gpio_direction_output,
	.set				= moxart_gpio_set,
	.get				= moxart_gpio_get,
	.base				= 0,
	.ngpio				= 32,
	.can_sleep			= 0,
};

static int moxart_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource res_gpio, res_pmu;
	int err;

	err = of_address_to_resource(node, 0, &res_gpio);
	if (err) {
		dev_err(dev, "could not get GPIO base resource\n");
		return err;
	}
	err = of_address_to_resource(node, 1, &res_pmu);
	if (err) {
		dev_err(dev, "could not get PMU base resource\n");
		return err;
	}

	moxart_gpio_base = devm_ioremap_resource(dev, &res_gpio);
	if (IS_ERR(moxart_gpio_base)) {
		dev_err(dev, "%s: devm_ioremap_resource res_gpio failed\n",
			__func__);
		return PTR_ERR(moxart_gpio_base);
	}
	moxart_pmu_base = devm_ioremap_resource(dev, &res_pmu);
	if (IS_ERR(moxart_pmu_base)) {
		dev_err(dev, "%s: devm_ioremap_resource res_pmu failed\n",
			__func__);
		return PTR_ERR(moxart_pmu_base);
	}

	gpiochip_add(&moxart_gpio_chip);

	moxart_gpio_enable(SW_READY_GPIO | SW_RESET_GPIO);
	moxart_gpio_direction_output(&moxart_gpio_chip, SW_READY_GPIO, 0);
	moxart_gpio_direction_input(&moxart_gpio_chip, SW_RESET_GPIO);
	moxart_gpio_set(&moxart_gpio_chip, SW_READY_GPIO, 0);
	
	/* change I/O multiplexing to SD (not needed) */
	/* moxart_gpio_disable(0xff << 10);*/ 

	/*	readyled is lit on SW_READY_GPIO=0, use SW_READY_GPIO=1 to
		shut it off after boot (bootloader switches it on) */

	dev_info(&pdev->dev, "finished %s gpio_base=%p pmu_base=%p\n",
		__func__, moxart_gpio_base, moxart_pmu_base);

	return 0;
}

static const struct of_device_id moxart_gpio_match[] = {
	{ .compatible = "moxa,moxart-gpio" },
	{ }
};

static struct platform_driver moxart_gpio_driver = {
	.driver     = {
		.name			= "moxart-gpio",
		.owner			= THIS_MODULE,
		.of_match_table	= moxart_gpio_match,
	},
	.probe      = moxart_gpio_probe,
};

static int __init moxart_gpio_init(void)
{
	return platform_driver_register(&moxart_gpio_driver);
}

postcore_initcall(moxart_gpio_init);
/*module_platform_driver(moxart_gpio_driver);*/

MODULE_ALIAS("platform:moxart-gpio");
MODULE_DESCRIPTION("MOXART GPIO chip driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");

