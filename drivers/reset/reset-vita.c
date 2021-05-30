// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/types.h>
#include <linux/of_device.h>

struct vita_reset {
	void __iomem *reg_base;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
};

static int vita_reset_level(struct reset_controller_dev *rcdev,
			    unsigned long id, bool assert)
{
	struct vita_reset *vreset =
		container_of(rcdev, struct vita_reset, rcdev);
	void __iomem *reg_addr = vreset->reg_base + id * 4;
	unsigned long flags;
	u32 val;
	const u32 mask = 1;

	pr_info("vita_reset_level: id: %ld, assert: %d\n", id, assert);

	spin_lock_irqsave(&vreset->lock, flags);

	val = readl(reg_addr);
	if (assert)
		writel(val | mask, reg_addr);
	else
		writel(val & ~mask, reg_addr);

	spin_unlock_irqrestore(&vreset->lock, flags);

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

static const struct reset_control_ops vita_reset_ops = {
	.assert		= vita_reset_assert,
	.deassert	= vita_reset_deassert,
};

static int vita_reset_probe(struct platform_device *pdev)
{
	struct vita_reset *data;
	struct resource *res;

	pr_info("vita_reset_probe\n");

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
	data->rcdev.nr_resets = resource_size(res) / 4;
	data->rcdev.ops = &vita_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

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

