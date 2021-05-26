// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/rohm-generic.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/regmap.h>

struct vita_clk_gate {
	struct clk_hw hw;
	void __iomem *reg;
};

#define to_vita_clk_gate(_hw) container_of(_hw, struct vita_clk_gate, hw)

static int vita_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct vita_clk_gate *gate = to_vita_clk_gate(hw);
	u32 reg;

	pr_info("Vita clock gate en/disable, reg: 0x%X, enable: %d\n", (u32)gate->reg, enable);

	reg = ioread32(gate->reg);
	reg &= ~1;
	reg |= enable;
	iowrite32(reg, gate->reg);

	return 0;
}

static int vita_clk_gate_enable(struct clk_hw *hw)
{
	return vita_clk_gate_endisable(hw, 1);
}

static void vita_clk_gate_disable(struct clk_hw *hw)
{
	vita_clk_gate_endisable(hw, 0);
}

static int vita_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct vita_clk_gate *gate = to_vita_clk_gate(hw);

	return ioread32(gate->reg) & 1;
}

static const struct clk_ops vita_clk_gate_ops = {
	.enable = vita_clk_gate_enable,
	.disable = vita_clk_gate_disable,
	.is_enabled = vita_clk_gate_is_enabled,
};

static int vita_clk_gate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct vita_clk_gate *data;
	struct clk_hw *hw;
	struct clk_init_data init;
	struct resource *res;
	int num_parents;
	int ret;

	pr_info("vita_clk_gate_probe\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->reg))
		return PTR_ERR(data->reg);

	num_parents = of_clk_get_parent_count(node);
	if (num_parents > 1) {
		dev_err(dev, "clock supports at most one parent\n");
		return -EINVAL;
	}

	init.name = dev->of_node->name;
	init.ops = &vita_clk_gate_ops;
	init.flags = 0;
	init.num_parents = 0;

	data->hw.init = &init;
	hw = &data->hw;

	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
}

static const struct of_device_id vita_clk_gate_of_match[] = {
	{ .compatible = "vita,clock-gate" },
	{}
};

MODULE_DEVICE_TABLE(of, vita_clk_gate_of_match);

static struct platform_driver vita_clk_gate = {
	.probe = vita_clk_gate_probe,
	.driver = {
		.name = "vita-clk",
		.of_match_table = vita_clk_gate_of_match,
	},
};

module_platform_driver(vita_clk_gate);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita clock gate driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:vita-clk");
