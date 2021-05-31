// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/vita-syscon.h>

static const struct mfd_cell vita_syscon_devs[] = {
	{
		.name = "vita-syscon-ts",
		.of_compatible = "vita,syscon-ts"
	},
	{
		.name = "vita-syscon-buttons",
		.of_compatible = "vita,syscon-buttons"
	},
	{
		.name = "vita-syscon-rtc",
		.of_compatible = "vita,syscon-rtc"
	},
};

static inline void syscon_set_tx_gpio(struct vita_syscon *syscon, int is_on)
{
	gpiod_set_value_cansleep(syscon->tx_gpio, is_on);
}

static inline void syscon_calc_checksum(u8 *tx, int data_size)
{
	int i;
	u32 hash = 0;

	for (i = 0; i < data_size; i++)
		hash += tx[i];

	tx[data_size] = ~hash;
}

static int vita_syscon_transfer(struct vita_syscon *syscon, u8 *tx, void *rx, int rx_size)
{
	struct spi_message msg;
	struct spi_transfer tx_xfer, rx_xfer;
	struct spi_device *spi = syscon->spi;
	unsigned int timeout;
	int ret = 0;
	int tx_size = SYSCON_TX_HEADER_SIZE + tx[SYSCON_TX_LENGTH];
	u8 res;

	memset(&tx_xfer, 0, sizeof(tx_xfer));
	memset(&rx_xfer, 0, sizeof(rx_xfer));

	tx_xfer.tx_buf = tx;
	tx_xfer.len = tx_size;
	rx_xfer.rx_buf = rx;
	rx_xfer.len = rx_size;

	syscon_calc_checksum(tx, tx_size - 1);

	spi_bus_lock(spi->master);

	do {
		reinit_completion(&syscon->rx_irq);

		syscon_set_tx_gpio(syscon, true);

		/* Send data */
		spi_message_init(&msg);
		spi_message_add_tail(&tx_xfer, &msg);
		ret = spi_sync_locked(spi, &msg);
		if (ret < 0) {
			syscon_set_tx_gpio(syscon, false);
			goto out;
		}

		syscon_set_tx_gpio(syscon, false);

		/* Wait RX interrupt */
		timeout = wait_for_completion_timeout(&syscon->rx_irq,
						      msecs_to_jiffies(250));
		if (!timeout) {
			dev_warn(&spi->dev, "%s: RX IRQ timeout", dev_name(&spi->dev));
			ret = -ETIMEDOUT;
			goto out;
		}

		/* Receive data */
		spi_message_init(&msg);
		spi_message_add_tail(&rx_xfer, &msg);
		ret = spi_sync_locked(spi, &msg);
		if (ret < 0)
			goto out;
		res = ((u8 *)rx)[SYSCON_RX_RESULT];
	} while (res == 0x80 || res == 0x81);

out:
	spi_bus_unlock(spi->master);

	return ret;
}

static int vita_syscon_command_read(struct vita_syscon *syscon, u16 cmd, void *rx, int rx_size)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 1];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = 1;

	return vita_syscon_transfer(syscon, tx, rx, rx_size);
}

static int vita_syscon_short_command_write(struct vita_syscon *syscon, u16 cmd, u32 data, int cmd_len)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + sizeof(u32) + 1];
	u8 rx[32];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = cmd_len;

	tx[SYSCON_TX_DATA(0)] = data & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (data >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = (data >> 16) & 0xFF;
	tx[SYSCON_TX_DATA(3)] = (data >> 24) & 0xFF;

	return vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
}

static int vita_syscon_scratchpad_read(struct vita_syscon *syscon, u16 offset, void *buffer, int size)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 3 + 1];
	u8 rx[32];
	int ret;

	if ((size - 1 >= 0x18) || ((size + offset) >= 0x101))
		return -EINVAL;

	tx[SYSCON_TX_CMD_LO] = 0x90;
	tx[SYSCON_TX_CMD_HI] = 0;
	tx[SYSCON_TX_LENGTH] = 4;

	tx[SYSCON_TX_DATA(0)] = offset & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (offset >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = size;

	ret = vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	memcpy(buffer, &rx[SYSCON_RX_DATA], size);

	return 0;
}

static int vita_syscon_scratchpad_write(struct vita_syscon *syscon, u16 offset, const void *buffer, int size)
{
	u8 tx[32];
	u8 rx[32];

	if ((size - 1 >= 0x18) || ((size + offset) >= 0x101))
		return -EINVAL;

	tx[SYSCON_TX_CMD_LO] = 0x91;
	tx[SYSCON_TX_CMD_HI] = 0;
	tx[SYSCON_TX_LENGTH] = size + 4;

	tx[SYSCON_TX_DATA(0)] = offset & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (offset >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = size;
	memcpy(&tx[SYSCON_TX_DATA(3)], buffer, size);

	return vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
}

static irqreturn_t vita_syscon_rx_gpio_irq_handler(int irq, void *dev_id)
{
	struct vita_syscon *syscon = dev_id;

	complete(&syscon->rx_irq);

	return IRQ_HANDLED;
}

static int vita_syscon_probe(struct spi_device *spi)
{
	struct vita_syscon *syscon;
	u8 baryon_version[SYSCON_RX_HEADER_SIZE + sizeof(syscon->baryon_version)];
	u8 hw_info[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_info)];
	u8 hw_flags[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_flags)];
	int ret, irq;

	pr_info("vita_syscon_probe\n");

	syscon = devm_kzalloc(&spi->dev, sizeof(struct vita_syscon),
				GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;

	irq = of_irq_get(spi->dev.of_node, 0);
	if (irq <= 0)
		return irq ? irq : -ENODEV;

	ret = devm_request_irq(&spi->dev, irq, vita_syscon_rx_gpio_irq_handler, 0,
			       "vita-syscon-gpio-rx", syscon);
	if (ret) {
		dev_err(&spi->dev, "could not request IRQ: %d\n", ret);
		return ret;
	}

	syscon->tx_gpio = devm_gpiod_get(&spi->dev, "tx", GPIOD_OUT_LOW);
	if (IS_ERR(syscon->tx_gpio))
		return PTR_ERR(syscon->tx_gpio);

	init_completion(&syscon->rx_irq);

	spi_set_drvdata(spi, syscon);
	syscon->dev = &spi->dev;
	syscon->spi = spi;
	syscon->transfer = vita_syscon_transfer;
	syscon->command_read = vita_syscon_command_read;
	syscon->short_command_write = vita_syscon_short_command_write;
	syscon->scratchpad_read = vita_syscon_scratchpad_read;
	syscon->scratchpad_write = vita_syscon_scratchpad_write;

	ret = vita_syscon_command_read(syscon, 1, baryon_version, sizeof(baryon_version));
	if (ret < 0) {
		return ret;
	}
	memcpy(&syscon->baryon_version, &baryon_version[SYSCON_RX_DATA],
	       sizeof(syscon->baryon_version));

	pr_info("Vita Syscon Baryon version: 0x%X\n", syscon->baryon_version);

	if (syscon->baryon_version > 0x1000003)
		ret = vita_syscon_short_command_write(syscon, 0x80, 0x12, 3);
	else if (syscon->baryon_version > 0x70501)
		ret = vita_syscon_short_command_write(syscon, 0x80, 2, 3);
	if (ret < 0) {
		return ret;
	}

	ret = vita_syscon_command_read(syscon, 5, hw_info, sizeof(hw_info));
	if (ret < 0) {
		return ret;
	}
	memcpy(&syscon->hardware_info, &hw_info[SYSCON_RX_DATA], sizeof(syscon->hardware_info));

	pr_info("Vita Syscon HW info: 0x%X\n", syscon->hardware_info);

	ret = vita_syscon_command_read(syscon, 6, hw_flags, sizeof(hw_flags));
	if (ret < 0) {
		return ret;
	}
	memcpy(syscon->hardware_flags, &hw_flags[SYSCON_RX_DATA], sizeof(syscon->hardware_flags));

	return devm_mfd_add_devices(syscon->dev, PLATFORM_DEVID_NONE,
				    vita_syscon_devs, ARRAY_SIZE(vita_syscon_devs),
				    NULL, 0, NULL);
}

static const struct of_device_id vita_syscon_of_match[] = {
	{ .compatible = "vita,syscon" },
	{}
};

MODULE_DEVICE_TABLE(of, vita_syscon_of_match);

static struct spi_driver vita_syscon_driver = {
	.driver = {
		.name   = "vita-syscon",
		.of_match_table = vita_syscon_of_match,
	},
	.probe = vita_syscon_probe,
};

module_spi_driver(vita_syscon_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita's Syscon (Ernie) driver");
MODULE_LICENSE("GPL");
