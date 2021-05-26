#include <linux/gpio/driver.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
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
	struct gpio_chip gc;
	struct clk *pclk;
	void __iomem *regs;
};

static void vita_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *cgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	//printk("vita_gpio_irq_mask: id: %d\n", d->hwirq);

	spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(cgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val |= BIT(d->hwirq);
	iowrite32(val, cgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	spin_unlock_irqrestore(&chip->bgpio_lock, flags);
}

static void vita_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *cgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	//printk("vita_gpio_irq_unmask: id: %d\n", d->hwirq);

	spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(cgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val &= ~BIT(d->hwirq);
	iowrite32(val, cgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	spin_unlock_irqrestore(&chip->bgpio_lock, flags);
}

static int vita_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *cgpio = gpiochip_get_data(chip);
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

	//printk("vita_gpio_irq_set_type: id: %d, type: %d (mode: %d)\n", d->hwirq, type, mode);

	spin_lock_irqsave(&chip->bgpio_lock, flags);

	val = ioread32(cgpio->regs + reg);
	val &= ~(3 << shift);
	val |= mode << shift;
	iowrite32(val, cgpio->regs + reg);

	spin_unlock_irqrestore(&chip->bgpio_lock, flags);

	return 0;
}

static void vita_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct vita_gpio_chip *cgpio = gpiochip_get_data(chip);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long masked, status, valid;
	int hwirq;

	masked = ioread32(cgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	status = ioread32(cgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
	valid = status & ~masked;

	//printk("vita_gpio_irq_handler, int status: 0x%08X\n", status);

	chained_irq_enter(irqchip, desc);

	for_each_set_bit(hwirq, &valid, chip->ngpio) {
		generic_handle_irq(irq_find_mapping(chip->irq.domain, hwirq));
		iowrite32(BIT(hwirq), cgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
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
	struct vita_gpio_chip *cgpio;
	int ret, irq;
	u32 num_gpios = 32;

	printk("vita_gpio_probe\n");

	cgpio = devm_kzalloc(&pdev->dev, sizeof(*cgpio), GFP_KERNEL);
	if (!cgpio)
		return -ENOMEM;

	cgpio->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cgpio->regs))
		return PTR_ERR(cgpio->regs);

	of_property_read_u32(pdev->dev.of_node, "ngpios", &num_gpios);

	if (num_gpios > 32) {
		dev_err(&pdev->dev, "ngpios must be less or equal 32\n");
		return -EINVAL;
	}

	/* Reset GPIO status here? */

	ret = bgpio_init(&cgpio->gc, &pdev->dev, 4,
			 cgpio->regs + VITA_GPIO_READ,
			 cgpio->regs + VITA_GPIO_SET,
			 cgpio->regs + VITA_GPIO_CLEAR,
			 cgpio->regs + VITA_GPIO_DIRECTION,
			 NULL,
			 0 /* BGPIOF_READ_OUTPUT_REG_SET */);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register generic gpio, %d\n",
			ret);
		return ret;
	}

	cgpio->gc.label = dev_name(&pdev->dev);
	cgpio->gc.ngpio = num_gpios;
	cgpio->gc.parent = &pdev->dev;
	cgpio->gc.base = -1;
	cgpio->gc.owner = THIS_MODULE;

	cgpio->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(cgpio->pclk)) {
		ret = PTR_ERR(cgpio->pclk);
		dev_err(&pdev->dev,
			"Failed to retrieve peripheral clock, %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(cgpio->pclk);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to enable the peripheral clock, %d\n", ret);
		return ret;
	}

	/*
	 * Optional irq_chip support
	 */
	irq = platform_get_irq(pdev, 0);
	printk("vita_gpio_probe irq: %d\n", irq);
	if (irq >= 0) {
		struct gpio_irq_chip *girq;

		girq = &cgpio->gc.irq;
		girq->chip = &vita_gpio_irqchip;
		girq->parent_handler = vita_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(&pdev->dev, 1,
					     sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents) {
			ret = -ENOMEM;
			goto err_disable_clk;
		}
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &cgpio->gc, cgpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err_disable_clk;
	}

	platform_set_drvdata(pdev, cgpio);
	return 0;

err_disable_clk:
	clk_disable_unprepare(cgpio->pclk);

	return ret;
}

static int vita_gpio_remove(struct platform_device *pdev)
{
	struct vita_gpio_chip *cgpio = platform_get_drvdata(pdev);

	clk_disable_unprepare(cgpio->pclk);

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
