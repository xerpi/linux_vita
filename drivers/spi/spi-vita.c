// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>

#define SPI_RXFIFO	0x00
#define SPI_TXFIFO	0x04
#define SPI_CONTROL	0x08
#define SPI_INT_CTL	0x0C
#define SPI_STATUS	0x10
#define SPI_UNK20	0x20
#define SPI_INT_STAT	0x24
#define SPI_RXFIFO_STAT	0x28
#define SPI_TXFIFO_STAT	0x2C

struct vita_spi {
	struct spi_controller	*ctlr;
	void __iomem		*regs;
	struct clk		*clk;
	struct reset_control	*rst;
};

static inline u32 vita_spi_read(struct vita_spi *vspi, u32 reg)
{
	u32 val = readl(vspi->regs + reg);
	//pr_info("vita_spi_read: reg: 0x%02X, value: 0x%X\n", reg, val);
	return val;
}

static inline void vita_spi_write(struct vita_spi *vspi, u32 reg, u32 value)
{
	//pr_info("vita_spi_write: reg: 0x%02X, value: 0x%X\n", reg, value);
	writel(value, vspi->regs + reg);
}

static inline u32 vita_spi_read_fifo_available(struct vita_spi *vspi)
{
	return vita_spi_read(vspi, SPI_RXFIFO_STAT) & 0x7F;
}

static void vita_spi_transfer_send(struct vita_spi *vspi, const u8 *tx_buf, int len)
{
	u32 val;
	unsigned int i = 0;

	/* Flush pending data in the RX FIFO */
	while (vita_spi_read_fifo_available(vspi) > 0)
		vita_spi_read(vspi, SPI_RXFIFO);

	vita_spi_read(vspi, SPI_TXFIFO_STAT);
	vita_spi_write(vspi, SPI_INT_STAT, 0x600);

	/* Transfer data */
	while (len >= 2) {
		val = tx_buf[i] | ((u32)tx_buf[i + 1] << 8);
		vita_spi_write(vspi, SPI_TXFIFO, val);
		i += 2;
		len -= 2;
	}

	if (len > 0)
		vita_spi_write(vspi, SPI_TXFIFO, tx_buf[i]);

	/* Transfer end */
	vita_spi_write(vspi, SPI_CONTROL, 0);
	vita_spi_write(vspi, SPI_STATUS, 1);
	vita_spi_read(vspi, SPI_STATUS);
}

static void vita_spi_transfer_recv(struct vita_spi *vspi, u8 *rx_buf, int len)
{
	u32 val, avail;
	int i = 0;

	while (i < len) {
		avail = vita_spi_read_fifo_available(vspi);
		if (!avail)
			break;

		val = vita_spi_read(vspi, SPI_RXFIFO);

		if (avail >= 2 && ((i + 2) <= len)) {
			rx_buf[i] = val & 0xFF;
			rx_buf[i + 1] = (val >> 8) & 0xFF;
			i += 2;
		} else if (avail >= 1) {
			rx_buf[i] = val & 0xFF;
			i++;
		}
	}

	/* Transfer end */
	vita_spi_write(vspi, SPI_STATUS, 0);
	vita_spi_read(vspi, SPI_STATUS);
}

static int vita_spi_transfer_one(struct spi_controller *ctlr,
				 struct spi_device *spi,
				 struct spi_transfer *tfr)
{

	struct vita_spi *vspi = spi_controller_get_devdata(ctlr);

	//pr_info("vita_spi_transfer_one. tx_buf: 0x%X, rx_buf: 0x%X, len: 0x%X\n",
	//       tfr->tx_buf, tfr->rx_buf, tfr->len);

	if (tfr->rx_buf) {
		vita_spi_transfer_recv(vspi, tfr->rx_buf, tfr->len);
	} else {
		vita_spi_transfer_send(vspi, tfr->tx_buf, tfr->len);
	}

	return 0;
}

static int vita_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct vita_spi *vspi;
	struct device *dev = &pdev->dev;
	int ret = 0;

	pr_info("vita_spi_probe\n");

	ctlr = devm_spi_alloc_master(dev, sizeof(struct vita_spi));
	if (!ctlr)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctlr);

	ctlr->transfer_one = vita_spi_transfer_one;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->auto_runtime_pm = true;

	vspi = spi_controller_get_devdata(ctlr);
	vspi->ctlr = ctlr;

	vspi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vspi->regs))
		return PTR_ERR(vspi->regs);

	vspi->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(vspi->clk))
		return dev_err_probe(dev, PTR_ERR(vspi->clk),
				     "could not get clk\n");

	clk_prepare_enable(vspi->clk);

	vspi->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(vspi->rst)) {
		ret = PTR_ERR(vspi->rst);
		goto err_disable_clk;
	}
	reset_control_deassert(vspi->rst);

	vita_spi_write(vspi, SPI_UNK20, 0);
	vita_spi_read(vspi, SPI_UNK20);

	pm_runtime_enable(dev);
	pm_runtime_set_active(dev);

	ret = spi_register_controller(ctlr);
	if (ret) {
		dev_err(dev, "could not register SPI controller: %d\n",
			ret);
		goto err_reset;
	}

	return 0;

err_reset:
	reset_control_assert(vspi->rst);

err_disable_clk:
	clk_disable_unprepare(vspi->clk);

	return ret;
}

static int vita_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct vita_spi *vspi = spi_controller_get_devdata(ctlr);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);

	reset_control_assert(vspi->rst);

	clk_disable_unprepare(vspi->clk);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);

	spi_unregister_controller(ctlr);

	return 0;
}

static void vita_spi_shutdown(struct platform_device *pdev)
{
	int ret;

	ret = vita_spi_remove(pdev);
	if (ret)
		dev_err(&pdev->dev, "failed to shutdown\n");
}

static const struct of_device_id vita_spi_match[] = {
	{ .compatible = "vita,spi", },
	{}
};
MODULE_DEVICE_TABLE(of, vita_spi_match);

static struct platform_driver vita_spi_driver = {
	.driver	= {
		.name		= "vita-spi",
		.of_match_table	= vita_spi_match,
	},
	.probe		= vita_spi_probe,
	.remove		= vita_spi_remove,
	.shutdown	= vita_spi_shutdown,
};
module_platform_driver(vita_spi_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:vita-spi");
