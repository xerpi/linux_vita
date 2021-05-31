/* linux/mfd/vita-syscon.h
 *
 * Functions to access PlayStation Vita's Syscon (Ernie)
 *
 * Copyright (c) 2021 Sergi Granell
 *
 *
 *  For licencing details see kernel-base/COPYING
 */

#ifndef __LINUX_MFD_VITA_SYSCON_H
#define __LINUX_MFD_VITA_SYSCON_H

#define SYSCON_TX_HEADER_SIZE	3
#define SYSCON_TX_CMD_LO	0
#define SYSCON_TX_CMD_HI	1
#define SYSCON_TX_LENGTH	2
#define SYSCON_TX_DATA(i)	(3 + (i))

#define SYSCON_RX_HEADER_SIZE	4
#define SYSCON_RX_STATUS_LO	0
#define SYSCON_RX_STATUS_HI	1
#define SYSCON_RX_LENGTH	2
#define SYSCON_RX_RESULT	3
#define SYSCON_RX_DATA		4

#define SYSCON_RESET_TYPE_POWEROFF	0
#define SYSCON_RESET_TYPE_SUSPEND	1
#define SYSCON_RESET_TYPE_COLD_RESET	2
#define SYSCON_RESET_TYPE_SOFT_RESET	17

struct vita_syscon {
	struct device *dev;
	struct spi_device *spi;
	struct gpio_desc *tx_gpio;
	struct completion rx_irq;
	int (*transfer)(struct vita_syscon *syscon, u8 *tx, void *rx, int rx_size);
	int (*command_read)(struct vita_syscon *syscon, u16 cmd, void *rx, int rx_size);
	int (*short_command_write)(struct vita_syscon *syscon, u16 cmd, u32 data, int cmd_len);
	int (*scratchpad_read)(struct vita_syscon *syscon, u16 offset, void *buffer, int size);
	int (*scratchpad_write)(struct vita_syscon *syscon, u16 offset, const void *buffer, int size);
	/* Syscon info */
	u32 baryon_version;
	u32 hardware_info;
	u8 hardware_flags[16];
};

#endif /*  __LINUX_MFD_VITA_SYSCON_H */
