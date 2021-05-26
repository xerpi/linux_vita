// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/spi/spi.h>

struct vita_spi {
	struct spi_master	*master;
	void __iomem		*base_addr;
	struct clk		*pclk;
	struct gpio_desc	*mosi;
	struct gpio_desc	*miso;
};

static inline u32 vita_spi_read(struct vita_spi *sspi, u32 reg)
{
	u32 val = readl(sspi->base_addr + reg);
	//printk("vita_spi_read: reg: 0x%02X, value: 0x%X\n", reg, val);
	return val;
}

static inline void vita_spi_write(struct vita_spi *sspi, u32 reg, u32 value)
{
	//printk("vita_spi_write: reg: 0x%02X, value: 0x%X\n", reg, value);
	writel(value, sspi->base_addr + reg);
}

static inline void vita_spi_read_fifo_wait(struct vita_spi *sspi, int n)
{
	while (vita_spi_read(sspi, 0xA * 4) < n)
		udelay(100);
}

static inline u32 vita_spi_read_fifo_available(struct vita_spi *sspi)
{
	return vita_spi_read(sspi, 0xA * 4) & 0x7F;
}

static inline void vita_spi_set_mosi(struct vita_spi *sspi, int is_on)
{
        gpiod_set_value_cansleep(sspi->mosi, is_on);
}

static inline int vita_spi_get_miso(struct vita_spi *sspi)
{
        return gpiod_get_value_cansleep(sspi->miso);
}

static void vita_spi_transfer_send(struct vita_spi *sspi, const u8 *tx_buf, int len)
{
	u32 val;
	unsigned int i = 0;
	
	vita_spi_set_mosi(sspi, true);

	/* Flush pending data to be read from the FIFO */
	while (vita_spi_read_fifo_available(sspi) > 0)
		vita_spi_read(sspi, 0 * 4);

	vita_spi_read(sspi, 0xB * 4);
	vita_spi_write(sspi, 9 * 4, 0x600);

	/* Transfer data */
	while (len >= 2) {
		val = tx_buf[i] | ((u32)tx_buf[i + 1] << 8);
		vita_spi_write(sspi, 1 * 4, val);
		i += 2;
		len -= 2;
	}

	if (len > 0)
		vita_spi_write(sspi, 1 * 4, tx_buf[i]);
	
	/* Transfer end */
	vita_spi_write(sspi, 2 * 4,  0);
	vita_spi_write(sspi, 4 * 4,  1);
	vita_spi_read(sspi, 4 * 4);
}

static void vita_spi_transfer_recv(struct vita_spi *sspi, u8 *rx_buf, int len)
{
	u32 val, avail;
	int i = 0;
	
	vita_spi_set_mosi(sspi, false);
	
	do {
		val = vita_spi_get_miso(sspi);
		//printk("miso: %d\n", val);
		udelay(100);
	} while (val == 1);
	
	do {
		avail = vita_spi_read_fifo_available(sspi);
		if (!avail)
			break;

		val = vita_spi_read(sspi, 0 * 4);
		
		if (avail >= 2 && ((i + 2) <= len)) {
			rx_buf[i] = val & 0xFF;
			rx_buf[i + 1] = (val >> 8) & 0xFF;
			i += 2;
		} else if (avail >= 1) {
			rx_buf[i] = val & 0xFF;
			i++;
		}
	} while (i < len);
	
	/*while (len >= 2) {
		vita_spi_read_fifo_wait(sspi, 2);
		val = vita_spi_read(sspi, 0 * 4);
		rx_buf[i] = val & 0xFF;
		rx_buf[i + 1] = (val >> 8) & 0xFF;
		i += 2;
		len -= 2;
	} 

	if (len > 0) {
		vita_spi_read_fifo_wait(sspi, 1);
		rx_buf[i] = vita_spi_read(sspi, 0 * 4) & 0xFF;
	}*/

	/* Transfer end */
	vita_spi_write(sspi, 4 * 4,  0);
	vita_spi_read(sspi, 4 * 4);
}


static int vita_spi_transfer_one(struct spi_master *master,
				 struct spi_device *spi,
				 struct spi_transfer *tfr)
{

	struct vita_spi *sspi = spi_master_get_devdata(master);

	//printk("vita_spi_transfer_one. tx_buf: 0x%X, rx_buf: 0x%X, len: 0x%X\n", tfr->tx_buf, tfr->rx_buf, tfr->len);

	/* Transfer data */	
	if (tfr->rx_buf) {
		vita_spi_transfer_recv(sspi, tfr->rx_buf, tfr->len);
	} else {
		vita_spi_transfer_send(sspi, tfr->tx_buf, tfr->len);
	}
	
	return 0;
}

static irqreturn_t vita_spi_irq_handler(int irq, void *dev_id)
{
	struct vita_spi *sspi = dev_id;
	//u32 status = vita_spi_read(sspi, vita_INT_STA_REG);

	printk("vita_spi_irq_handler: %d, status: 0x%X\n", irq, 0 /* status */);

	return IRQ_NONE;
}

static int vita_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct vita_spi *sspi;
	int ret = 0, irq;
	
	printk("vita_spi_probe\n");

	master = spi_alloc_master(&pdev->dev, sizeof(struct vita_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);
	sspi = spi_master_get_devdata(master);

	sspi->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sspi->base_addr)) {
		ret = PTR_ERR(sspi->base_addr);
		goto err_free_master;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENXIO;
		goto err_free_master;
	}

	ret = devm_request_irq(&pdev->dev, irq, vita_spi_irq_handler,
			       0, "vita-spi", sspi);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ\n");
		goto err_free_master;
	}

	// printk("Vita SPI MOSI count: %d\n", gpiod_count(&pdev->dev, "mosi"));

	sspi->mosi = devm_gpiod_get_optional(&pdev->dev, "mosi", GPIOD_OUT_HIGH);
	if (IS_ERR(sspi->mosi))
		return PTR_ERR(sspi->mosi);

	sspi->miso = devm_gpiod_get_optional(&pdev->dev, "miso", GPIOD_IN);
	if (IS_ERR(sspi->miso))
		return PTR_ERR(sspi->miso);

	sspi->master = master;
	master->transfer_one = vita_spi_transfer_one;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->dev.of_node = pdev->dev.of_node;
	master->auto_runtime_pm = true;

	sspi->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sspi->pclk)) {
		ret = PTR_ERR(sspi->pclk);
		dev_err(&pdev->dev,
			"Failed to retrieve peripheral clock, %d\n", ret);
		goto err_free_master;
	}

	ret = clk_prepare_enable(sspi->pclk);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to enable the peripheral clock, %d\n", ret);
		return ret;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);
	
	vita_spi_write(sspi, 8 * 4,  0);
	vita_spi_read(sspi, 8 * 4);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		goto err_disable_clk;
	}

	return 0;

err_disable_clk:
	clk_disable_unprepare(sspi->pclk);
err_free_master:
	spi_master_put(master);
	return ret;
}

static const struct of_device_id vita_spi_match[] = {
	{ .compatible = "vita,spi", },
	{}
};
MODULE_DEVICE_TABLE(of, vita_spi_match);

static struct platform_driver vita_spi_driver = {
	.probe	= vita_spi_probe,
	.driver	= {
		.name		= "vita-spi",
		.of_match_table	= vita_spi_match,
	},
};
module_platform_driver(vita_spi_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:vita-spi");

