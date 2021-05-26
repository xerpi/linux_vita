// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
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
};

static inline void syscon_calc_checksum(u8 *packet, int data_length)
{
	int i;
	unsigned char hash = 0;

	for (i = 0; i < SYSCON_TX_HEADER_SIZE + data_length; i++)
		hash += packet[i];

	packet[SYSCON_TX_HEADER_SIZE + data_length] = ~hash;
}

static int vita_syscon_command_read(struct vita_syscon *syscon, u16 cmd, void *buffer, int length)
{
	int ret;
	struct spi_transfer xfers[2] = {0};
	u8 tx[SYSCON_TX_HEADER_SIZE + 1];
	u8 res;

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = 1;

	syscon_calc_checksum(tx, 0);

	xfers[0].tx_buf = tx;
	xfers[0].len = sizeof(tx);
	xfers[1].rx_buf = buffer;
	xfers[1].len = length;

	do {
		ret = spi_sync_transfer(syscon->spi, xfers, 2);
		if (ret < 0)
			return ret;
		res = ((u8 *)buffer)[SYSCON_RX_RESULT];
	} while (res == 0x80 || res == 0x81);

        return 0;
}

static int vita_syscon_short_command_write(struct vita_syscon *syscon, u16 cmd, u32 data, int length)
{
	int ret;
	struct spi_transfer xfers[2] = {0};
	u8 tx[SYSCON_TX_HEADER_SIZE + sizeof(data) + 1];
	u8 rx[16];
	u8 res;

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = length + 1;

	tx[SYSCON_TX_DATA(0)] = data & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (data >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = (data >> 16) & 0xFF;
	tx[SYSCON_TX_DATA(3)] = (data >> 24) & 0xFF;

	syscon_calc_checksum(tx, length);

	xfers[0].tx_buf = tx;
	xfers[0].len = SYSCON_TX_HEADER_SIZE + length + 1;
	xfers[1].rx_buf = rx;
	xfers[1].len = sizeof(rx);

	do {
		ret = spi_sync_transfer(syscon->spi, xfers, 2);
		if (ret < 0)
			return ret;
		res = rx[SYSCON_RX_RESULT];
	} while (res == 0x80 || res == 0x81);

        return 0;
}

static int vita_syscon_probe(struct spi_device *spi)
{
	int error;
	struct vita_syscon *syscon;
	u8 baryon_version[SYSCON_RX_HEADER_SIZE + sizeof(syscon->baryon_version)];
	u8 hw_info[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_info)];
	u8 hw_flags[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_flags)];

	printk("vita_syscon_probe\n");

	syscon = devm_kzalloc(&spi->dev, sizeof(struct vita_syscon),
				GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;

	spi_set_drvdata(spi, syscon);
	syscon->dev = &spi->dev;
	syscon->spi = spi;
	syscon->command_read = vita_syscon_command_read;
	syscon->short_command_write = vita_syscon_short_command_write;

	printk("vita_syscon_probe: before reading baryon\n");

	vita_syscon_command_read(syscon, 1, baryon_version, sizeof(baryon_version));
	memcpy(&syscon->baryon_version, &baryon_version[SYSCON_RX_DATA], sizeof(syscon->baryon_version));

	printk("Vita Syscon Baryon version: 0x%X\n", syscon->baryon_version);

	if (syscon->baryon_version > 0x1000003)
		vita_syscon_short_command_write(syscon, 0x80, 0x12, 2);
	else if (syscon->baryon_version > 0x70501)
		vita_syscon_short_command_write(syscon, 0x80, 2, 2);

	vita_syscon_command_read(syscon, 5, hw_info, sizeof(hw_info));
	memcpy(&syscon->hardware_info, &hw_info[SYSCON_RX_DATA], sizeof(syscon->hardware_info));

	printk("Vita Syscon HW info: 0x%X\n", syscon->hardware_info);

	vita_syscon_command_read(syscon, 6, hw_flags, sizeof(hw_flags));
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

