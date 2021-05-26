// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Vita Reset Controller driver
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/of_device.h>

struct vita_reset {
	void __iomem *reg_base;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
};

static int vita_reset_reset(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct vita_reset *data =
		container_of(rcdev, struct vita_reset, rcdev);
	/*unsigned int bank = id / BITS_PER_REG;
	unsigned int offset = id % BITS_PER_REG;
	void __iomem *reg_addr = data->reg_base + (bank << 2);*/

	printk("vita_reset_reset: %d\n", id);

	//writel(BIT(offset), reg_addr);

	return 0;
}

static int vita_reset_level(struct reset_controller_dev *rcdev,
			    unsigned long id, bool assert)
{
	struct vita_reset *data =
		container_of(rcdev, struct vita_reset, rcdev);
	/*unsigned int bank = id / BITS_PER_REG;
	unsigned int offset = id % BITS_PER_REG;*/
	void __iomem *reg_addr;
	unsigned long flags;
	u32 reg;

	printk("vita_reset_level: id: %d, assert: %d\n", id, assert);

	//reg_addr = data->reg_base + data->param->level_offset + (bank << 2);

	spin_lock_irqsave(&data->lock, flags);

	/*reg = readl(reg_addr);
	if (assert)
		writel(reg & ~BIT(offset), reg_addr);
	else
		writel(reg | BIT(offset), reg_addr);*/

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int vita_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return vita_reset_level(rcdev, id, true);
}

static int vita_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return vita_reset_level(rcdev, id, false);
}

static int vita_reset_status(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	printk("vita_reset_status: id: %d\n", id);

	return 0;
}

static const struct reset_control_ops vita_reset_ops = {
	.reset		= vita_reset_reset,
	.assert		= vita_reset_assert,
	.deassert	= vita_reset_deassert,
	.status		= vita_reset_status,
};

static int vita_reset_probe(struct platform_device *pdev)
{
	struct vita_reset *data;
	struct resource *res;

	printk("vita_reset_probe\n");

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->reg_base))
		return PTR_ERR(data->reg_base);

	platform_set_drvdata(pdev, data);

	spin_lock_init(&data->lock);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 1024; //data->param->reg_count;
	data->rcdev.ops = &vita_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	printk("vita_reset_probe done\n");

	return devm_reset_controller_register(&pdev->dev, &data->rcdev);
}

static const struct of_device_id vita_reset_of_match[] = {
	{ .compatible = "vita,reset" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vita_reset_of_match);

static struct platform_driver vita_reset_driver = {
	.probe	= vita_reset_probe,
	.driver = {
		.name		= "vita-reset",
		.of_match_table	= vita_reset_of_match,
	},
};
module_platform_driver(vita_reset_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita reset driver");
MODULE_LICENSE("GPL");

