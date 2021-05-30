// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/gpio/driver.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#define VITA_GPIO_DIRECTION		0x00
#define  VITA_GPIO_DIRECTION_OUT	0
#define  VITA_GPIO_DIRECTION_IN		1
#define VITA_GPIO_READ			0x04
#define VITA_GPIO_SET			0x08
#define VITA_GPIO_CLEAR			0x0C
#define VITA_GPIO_INT_MODE_0_15		0x14
#define VITA_GPIO_INT_MODE_16_31	0x18
#define  VITA_GPIO_INT_MODE_HLEVEL_SENS	0
#define  VITA_GPIO_INT_MODE_LLEVEL_SENS	1
#define  VITA_GPIO_INT_MODE_REDGE	2
#define  VITA_GPIO_INT_MODE_FEDGE	3
#define VITA_GPIO_INT_MASK_GATE0	0x1C
#define VITA_GPIO_INT_MASK_GATE1	0x20
#define VITA_GPIO_INT_MASK_GATE2	0x24
#define VITA_GPIO_INT_MASK_GATE3	0x28
#define VITA_GPIO_INT_MASK_GATE4	0x2C
#define VITA_GPIO_READ_LATCH		0x34
#define VITA_GPIO_INT_STATUS_GATE0	0x38
#define VITA_GPIO_INT_STATUS_GATE1	0x3C
#define VITA_GPIO_INT_STATUS_GATE2	0x40
#define VITA_GPIO_INT_STATUS_GATE3	0x44
#define VITA_GPIO_INT_STATUS_GATE4	0x48

struct vita_gpio_chip {
	struct gpio_chip	gc;
	void __iomem		*regs;
	struct clk 		*clk;
	struct reset_control    *rst;
};

static void vita_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	//pr_info("vita_gpio_irq_mask: id: %d\n", d->hwirq);

	raw_spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val |= BIT(d->hwirq);
	iowrite32(val, vgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	raw_spin_unlock_irqrestore(&chip->bgpio_lock, flags);
}

static void vita_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	//pr_info("vita_gpio_irq_unmask: id: %d\n", d->hwirq);

	raw_spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val &= ~BIT(d->hwirq);
	iowrite32(val, vgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	raw_spin_unlock_irqrestore(&chip->bgpio_lock, flags);
}

static int vita_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	int mode;
	u32 val;
	u32 reg;
	u32 shift;

	if (d->hwirq < 16) {
		reg = VITA_GPIO_INT_MODE_0_15;
		shift = d->hwirq * 2;
	} else {
		reg = VITA_GPIO_INT_MODE_16_31;
		shift = (d->hwirq - 16) * 2;
	}

	if (type == IRQ_TYPE_EDGE_RISING) {
		mode = VITA_GPIO_INT_MODE_REDGE;
	} else if (type == IRQ_TYPE_EDGE_FALLING) {
		mode = VITA_GPIO_INT_MODE_FEDGE;
	} else if (type == IRQ_TYPE_LEVEL_HIGH) {
		mode = VITA_GPIO_INT_MODE_HLEVEL_SENS;
	} else if (type == IRQ_TYPE_LEVEL_LOW) {
		mode = VITA_GPIO_INT_MODE_LLEVEL_SENS;
	}

	//pr_info("vita_gpio_irq_set_type: id: %d, type: %d (mode: %d)\n", d->hwirq, type, mode);

	raw_spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(vgpio->regs + reg);
	val &= ~(3 << shift);
	val |= mode << shift;
	iowrite32(val, vgpio->regs + reg);

	raw_spin_unlock_irqrestore(&chip->bgpio_lock, flags);

	return 0;
}

static void vita_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long masked, status, valid;
	int hwirq;

	masked = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	status = ioread32(vgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
	valid = status & ~masked;

	//pr_info("vita_gpio_irq_handler, int status: 0x%08X\n", status);

	chained_irq_enter(irqchip, desc);

	for_each_set_bit(hwirq, &valid, chip->ngpio) {
		generic_handle_irq(irq_find_mapping(chip->irq.domain, hwirq));
		iowrite32(BIT(hwirq), vgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
	}

	chained_irq_exit(irqchip, desc);
}

static struct irq_chip vita_gpio_irqchip = {
	.name		= "vita-gpio",
	.irq_mask	= vita_gpio_irq_mask,
	.irq_unmask	= vita_gpio_irq_unmask,
	.irq_set_type	= vita_gpio_irq_set_type
};

static int vita_gpio_probe(struct platform_device *pdev)
{
	struct vita_gpio_chip *vgpio;
	struct gpio_irq_chip *girq;
	struct device *dev = &pdev->dev;
	int ret, irq;
	u32 num_gpios = 32;

	pr_info("vita_gpio_probe\n");

	vgpio = devm_kzalloc(dev, sizeof(*vgpio), GFP_KERNEL);
	if (!vgpio)
		return -ENOMEM;

	vgpio->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vgpio->regs))
		return PTR_ERR(vgpio->regs);

	of_property_read_u32(pdev->dev.of_node, "ngpios", &num_gpios);
	if (num_gpios > 32) {
		dev_err(dev, "ngpios must be less or equal 32\n");
		return -EINVAL;
	}

	/* Reset GPIO status here? */

	ret = bgpio_init(&vgpio->gc, dev, 4,
			 vgpio->regs + VITA_GPIO_READ,
			 vgpio->regs + VITA_GPIO_SET,
			 vgpio->regs + VITA_GPIO_CLEAR,
			 vgpio->regs + VITA_GPIO_DIRECTION,
			 NULL,
			 0 /* BGPIOF_READ_OUTPUT_REG_SET */);
	if (ret) {
		dev_err(dev, "Failed to register generic gpio, %d\n",
			ret);
		return ret;
	}

	vgpio->gc.label = dev_name(dev);
	vgpio->gc.ngpio = num_gpios;
	vgpio->gc.parent = dev;
	vgpio->gc.base = -1;
	vgpio->gc.owner = THIS_MODULE;

	vgpio->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(vgpio->clk)) {
		ret = PTR_ERR(vgpio->clk);
		dev_err(dev,
			"Failed to retrieve peripheral clock, %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(vgpio->clk);
	if (ret) {
		dev_err(dev,
			"Failed to enable the peripheral clock, %d\n", ret);
		return ret;
	}

	vgpio->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(vgpio->rst)) {
		ret = PTR_ERR(vgpio->rst);
		goto err_disable_clk;
	}
	reset_control_deassert(vgpio->rst);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return irq ? irq : -ENODEV;

	girq = &vgpio->gc.irq;
	girq->chip = &vita_gpio_irqchip;
	girq->parent_handler = vita_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1,
				     sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents) {
		ret = -ENOMEM;
		goto err_reset;
	}
	girq->parents[0] = irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_level_irq;

	ret = devm_gpiochip_add_data(dev, &vgpio->gc, vgpio);
	if (ret < 0) {
		dev_err(dev, "Could not register gpiochip, %d\n", ret);
		goto err_reset;
	}

	platform_set_drvdata(pdev, vgpio);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_reset:
	reset_control_assert(vgpio->rst);

err_disable_clk:
	clk_disable_unprepare(vgpio->clk);

	return ret;
}

static int vita_gpio_remove(struct platform_device *pdev)
{
	struct vita_gpio_chip *vgpio = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);

	reset_control_assert(vgpio->rst);

	clk_disable_unprepare(vgpio->clk);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);

	return 0;
}

static const struct of_device_id vita_of_ids[] = {
	{ .compatible = "vita,gpio" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vita_of_ids);

static struct platform_driver vita_gpio_driver = {
	.driver = {
		.name = "vita-gpio",
		.of_match_table = vita_of_ids,
	},
	.probe = vita_gpio_probe,
	.remove = vita_gpio_remove,
};
module_platform_driver(vita_gpio_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita GPIO driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-gpio");
