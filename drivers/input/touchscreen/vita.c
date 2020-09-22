// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PlayStation Vita touchscreen support
 *
 * Copyright (C) 2020 Sergi Granell
 *
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DEFAULT_POLL_PERIOD 10 /* ms */
#define MAX_8BIT ((1 << 8) - 1)

struct vita_ts {
	struct device *dev;
	struct input_dev *input;
	void __iomem *base;
};

//////////////////////////////////////////////////////

#define PERVASIVE_RESET_BASE_ADDR	0xE3101000
#define PERVASIVE_GATE_BASE_ADDR	0xE3102000
#define PERVASIVE_BASECLK_BASE_ADDR	0xE3103000
#define PERVASIVE_MISC_BASE_ADDR	0xE3100000
#define PERVASIVE2_BASE_ADDR		0xE3110000

static void __iomem  *g_pervasive_gate;
static void __iomem  *g_pervasive_reset;

static inline void pervasive_mask_or(void __iomem *addr, unsigned int val)
{
	uint32_t tmp;
	tmp = ioread32(addr);
	tmp |= val;
	iowrite32(tmp, addr);
	smp_mb();
	tmp = ioread32(addr);
	smp_mb();
}

static inline void pervasive_mask_and_not(void __iomem *addr, unsigned int val)
{
	uint32_t tmp;
	tmp = ioread32(addr);
	tmp &= ~val;
	iowrite32(tmp, addr);
	smp_mb();
	tmp = ioread32(addr);
	smp_mb();
}

void pervasive_clock_enable_spi(int bus)
{
	pervasive_mask_or((char __iomem *)g_pervasive_gate + 0x104 + 4 * bus, 1);
}

void pervasive_clock_disable_spi(int bus)
{
	pervasive_mask_and_not((char __iomem *)g_pervasive_gate + 0x104 + 4 * bus, 1);
}

void pervasive_reset_exit_spi(int bus)
{
	pervasive_mask_and_not((char __iomem *)g_pervasive_reset + 0x104 + 4 * bus, 1);
}

//////////////////////////////////////////////////////////

#define GPIO0_BASE_ADDR			0xE20A0000
#define GPIO1_BASE_ADDR			0xE0100000

#define GPIO_REGS(i)			((void *)((i) == 0 ? g_gpio0_addr : g_gpio1_addr))

#define GPIO_PORT_MODE_INPUT	0
#define GPIO_PORT_MODE_OUTPUT	1

#define GPIO_PORT_OLED_LCD	0
#define GPIO_PORT_SYSCON_OUT	3
#define GPIO_PORT_SYSCON_IN	4
#define GPIO_PORT_GAMECARD_LED	6
#define GPIO_PORT_PS_LED	7
#define GPIO_PORT_HDMI_BRIDGE	15

static void __iomem  *g_gpio0_addr;
static void __iomem  *g_gpio1_addr;

void gpio_set_port_mode(int bus, int port, int mode)
{
	uint32_t tmp;
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	tmp = ioread32((char __iomem *)gpio_regs + 0);
	tmp &= ~(1 << port);
	tmp |= (mode << port);
	iowrite32(tmp, (char __iomem *)gpio_regs + 0);

	dmb();
}

int gpio_port_read(int bus, int port)
{
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	ioread32((char __iomem *)gpio_regs + 4);

	return (ioread32((char __iomem *)gpio_regs + 4) >> port) & 1;
}

void gpio_port_set(int bus, int port)
{
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	iowrite32((1 << port), (char __iomem *)gpio_regs + 2 * 4);

	ioread32((char __iomem *)gpio_regs + 4 * 0xD);

	dsb();
}

void gpio_port_clear(int bus, int port)
{
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	iowrite32((1 << port), (char __iomem *)gpio_regs + 3 * 4);

	ioread32((char __iomem *)gpio_regs + 4 * 0xD);

	dsb();
}

void gpio_set_intr_mode(int bus, int port, int mode)
{
	uint32_t tmp;
	void __iomem  *gpio_regs = GPIO_REGS(bus);
	unsigned int reg = 5 + port / 15;
	unsigned int off = 2 * (port % 15);

	tmp = ioread32((char __iomem *)gpio_regs + 4 * reg);
	tmp &= ~(3 << off);
	tmp |= (mode << off);
	iowrite32(tmp, (char __iomem *)gpio_regs + 4 * reg);

	dmb();
}

int gpio_query_intr(int bus, int port)
{
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	return (1 << port) & ((ioread32((char __iomem *)gpio_regs + 4 * 0x0E) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x07)) |
			      (ioread32((char __iomem *)gpio_regs + 4 * 0x0F) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x08)) |
			      (ioread32((char __iomem *)gpio_regs + 4 * 0x10) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x09)) |
			      (ioread32((char __iomem *)gpio_regs + 4 * 0x11) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x0A)) |
			      (ioread32((char __iomem *)gpio_regs + 4 * 0x12) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x0B)));
}

int gpio_acquire_intr(int bus, int port)
{
	unsigned int ret;
	unsigned int mask = 1 << port;
	void __iomem  *gpio_regs = GPIO_REGS(bus);

	ret = mask & ((ioread32((char __iomem *)gpio_regs + 4 * 0x0E) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x07)) |
		      (ioread32((char __iomem *)gpio_regs + 4 * 0x0F) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x08)) |
		      (ioread32((char __iomem *)gpio_regs + 4 * 0x10) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x09)) |
		      (ioread32((char __iomem *)gpio_regs + 4 * 0x11) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x0A)) |
		      (ioread32((char __iomem *)gpio_regs + 4 * 0x12) & ~ioread32((char __iomem *)gpio_regs + 4 * 0x0B)));

	iowrite32(mask, (char __iomem *)gpio_regs + 4 * 0x0E);
	iowrite32(mask, (char __iomem *)gpio_regs + 4 * 0x0F);
	iowrite32(mask, (char __iomem *)gpio_regs + 4 * 0x10);
	iowrite32(mask, (char __iomem *)gpio_regs + 4 * 0x11);
	iowrite32(mask, (char __iomem *)gpio_regs + 4 * 0x12);
	dsb();

	return ret;
}

//////////////////////////////////////////////////////

#define SPI_BASE_ADDR	0xE0A00000
#define SPI_REGS(i)	(g_spi0_addr)

static void __iomem  *g_spi0_addr;

int spi_init(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	pervasive_clock_enable_spi(bus);
	pervasive_reset_exit_spi(bus);

	if (bus == 2) {
		iowrite32(0x30001, (char __iomem *)spi_regs + 4 * 2);
		iowrite32(0xF,  (char __iomem *)spi_regs + 4 * 5);
		iowrite32(3, (char __iomem *)spi_regs + 4 * 3);
	}

	iowrite32(0, (char __iomem *)spi_regs + 4 * 8);
	ioread32((char __iomem *)spi_regs + 4 * 8);

	dsb();

	return 0;
}

void spi_write_start(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	/*
	 * Flush pending data to be read from the FIFO
	 */
	while (ioread32((char __iomem *)spi_regs + 4 * 0xA))
		ioread32((char __iomem *)spi_regs + 4 * 0);

	ioread32((char __iomem *)spi_regs + 4 * 0xB);
	iowrite32(0x600, (char __iomem *)spi_regs + 4 * 9);
}

void spi_write_end(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	iowrite32(0, (char __iomem *)spi_regs + 4 * 2);
	iowrite32(1, (char __iomem *)spi_regs + 4 * 4);
	ioread32((char __iomem *)spi_regs + 4 * 4);

	dsb();
}

void spi_write(int bus, unsigned int data)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	iowrite32(data, (char __iomem *)spi_regs + 4 * 1);
}

int spi_read_available(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	return ioread32((char __iomem *)spi_regs + 4 * 0xA);
}

int spi_read(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	return ioread32((char __iomem *)spi_regs + 4 * 0);
}

void spi_read_end(int bus)
{
	void __iomem  *spi_regs = SPI_REGS(bus);

	iowrite32(0, (char __iomem *)spi_regs + 4 * 4);
	ioread32((char __iomem *)spi_regs + 4 * 4);

	dsb();
}

//////////////////////////////////////////////////////

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

struct syscon_touchpanel_device_info {
	uint16_t front_vendor_id;
	uint16_t front_fw_version;
	uint16_t back_vendor_id;
	uint16_t back_fw_version;
};

struct syscon_touchpanel_device_info_ext {
	uint16_t front_vendor_id;
	uint16_t front_fw_version;
	uint16_t front_unk1;
	uint8_t front_unk2;
	uint8_t front_unk3;
	uint16_t unused1;
	uint16_t back_vendor_id;
	uint16_t back_fw_version;
	uint16_t back_unk1;
	uint8_t back_unk2;
	uint8_t back_unk3;
	uint16_t unused2;
};

int syscon_init(void);
void syscon_transfer(const uint8_t *tx, int tx_size, uint8_t *rx, int max_rx_size);
void syscon_command_read(uint16_t cmd, void *buffer, int max_length);
void syscon_short_command_write(uint16_t cmd, uint32_t data, int length);
int syscon_get_baryon_version(void);
int syscon_get_hardware_info(void);
void syscon_reset_device(int type, int mode);
void syscon_ctrl_device_reset(unsigned int param_1, unsigned int param_2);
void syscon_get_touchpanel_device_info(struct syscon_touchpanel_device_info *info);
void syscon_get_touchpanel_device_info_ext(struct syscon_touchpanel_device_info_ext *info);
void syscon_get_touchpanel_unk_info_front(uint16_t *data);
void syscon_get_touchpanel_unk_info_back(uint16_t *data);
void syscon_touch_set_sampling_cycle(int cycles_front, int cycles_back);

static uint32_t g_baryon_version;

int syscon_init(void)
{
	uint8_t version[SYSCON_RX_HEADER_SIZE + sizeof(g_baryon_version)];

	spi_init(0);

	gpio_set_port_mode(0, GPIO_PORT_SYSCON_OUT, GPIO_PORT_MODE_OUTPUT);
	gpio_set_port_mode(0, GPIO_PORT_SYSCON_IN, GPIO_PORT_MODE_INPUT);
	gpio_set_intr_mode(0, GPIO_PORT_SYSCON_IN, 3);

	syscon_command_read(1, version, sizeof(version));
	memcpy(&g_baryon_version, &version[SYSCON_RX_DATA], sizeof(g_baryon_version));

	if (g_baryon_version > 0x1000003)
		syscon_short_command_write(0x80, 0x12, 2);
	else if (g_baryon_version > 0x70501)
		syscon_short_command_write(0x80, 2, 2);

	printk("VITA: Baryon: %x\n", g_baryon_version);

	return 0;
}


static void syscon_raw_write(const uint8_t *buffer, int length)
{
	uint32_t hash = 0;
	int i = 0;

	gpio_port_clear(0, GPIO_PORT_SYSCON_OUT);
	spi_write_start(0);

	while (length >= 2) {
		uint8_t lo = buffer[i];
		uint8_t hi = buffer[i + 1];
		spi_write(0, lo | ((uint32_t)hi << 8));
		hash += lo + hi;
		i += 2;
		length -= 2;
	}

	if (length) {
		hash = ~(hash + buffer[i]) & 0xFF;
		spi_write(0, buffer[i] | ((uint32_t)hash << 8));
	} else {
		spi_write(0, ~hash & 0xFF);
	}

	spi_write_end(0);
	gpio_port_set(0, GPIO_PORT_SYSCON_OUT);
}

static void syscon_raw_read(uint8_t *buffer, int max_length)
{
	int i = 0;

	while (!gpio_query_intr(0, GPIO_PORT_SYSCON_IN))
		;

	gpio_acquire_intr(0, GPIO_PORT_SYSCON_IN);

	while (spi_read_available(0)) {
		uint32_t data = spi_read(0);
		if (i < max_length)
			buffer[i] = data & 0xFF;
		if (i + 1 < max_length)
			buffer[i + 1] = (data >> 8) & 0xFF;
		i += 2;
	}

	spi_read_end(0);
	gpio_port_clear(0, GPIO_PORT_SYSCON_OUT);
}

void syscon_transfer(const uint8_t *tx, int tx_size, uint8_t *rx, int max_rx_size)
{
	uint8_t ret;

	do {
		syscon_raw_write(tx, tx_size);
		syscon_raw_read(rx, max_rx_size);
		ret = rx[SYSCON_RX_RESULT];
	} while (ret == 0x80 || ret == 0x81);
}

void syscon_command_read(uint16_t cmd, void *buffer, int max_length)
{
	uint8_t tx[SYSCON_TX_HEADER_SIZE];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = 1;

	syscon_transfer(tx, sizeof(tx), buffer, max_length);
}

void syscon_short_command_write(uint16_t cmd, uint32_t data, int length)
{
	uint8_t tx[SYSCON_TX_HEADER_SIZE + 4];
	uint8_t rx[16];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = length + 1;

	tx[SYSCON_TX_DATA(0)] = data & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (data >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = (data >> 16) & 0xFF;
	tx[SYSCON_TX_DATA(3)] = (data >> 24) & 0xFF;

	syscon_transfer(tx, SYSCON_TX_HEADER_SIZE + length, rx, sizeof(rx));
}

int syscon_get_baryon_version(void)
{
	return g_baryon_version;
}
/*
From my PSTV:
DCD 0x1030603           ; syscon_version
DCD 0x703030            ; sysroot_hw_info
From my Vita 1000:
DCD 0x100060D           ; syscon_version
DCD 0x406000            ; sysroot_hw_info
*/
int syscon_get_hardware_info(void)
{
	return 0x406000; //sysroot_get_hw_info();
}

void syscon_ctrl_device_reset(unsigned int param_1, unsigned int param_2)
{
	syscon_short_command_write(0x88F, param_2 | (param_1 << 8), 2);
}

void syscon_get_touchpanel_device_info(struct syscon_touchpanel_device_info *info)
{
	unsigned char buffer[SYSCON_RX_HEADER_SIZE + sizeof(*info)];
	uint8_t *data = &buffer[SYSCON_RX_DATA];

	syscon_command_read(0x380, buffer, sizeof(buffer));

	info->front_vendor_id  = (data[0] << 8) | data[1];
	info->front_fw_version = (data[2] << 8) | data[3];
	info->back_vendor_id   = (data[4] << 8) | data[5];
	info->back_fw_version  = (data[6] << 8) | data[7];
}

void syscon_get_touchpanel_device_info_ext(struct syscon_touchpanel_device_info_ext *info)
{
	uint8_t buffer[SYSCON_RX_HEADER_SIZE + sizeof(*info)];
	uint8_t *data = &buffer[SYSCON_RX_DATA];

	syscon_command_read(0x390, buffer, sizeof(buffer));

	info->front_vendor_id  = (data[0] << 8) | data[1];
	info->front_fw_version = (data[2] << 8) | data[3];
	info->front_unk1       = (data[4] << 8) | data[5];
	info->front_unk2       = data[6];
	info->front_unk3       = data[7];
	info->unused1          = (data[9] << 8) | data[8];
	info->back_vendor_id   = (data[10] << 8) | data[11];
	info->back_fw_version  = (data[12] << 8) | data[13];
	info->back_unk1        = (data[14] << 8) | data[15];
	info->back_unk2        = data[16];
	info->back_unk3        = data[17];
	info->unused2          = (data[19] << 8) | data[18];
}

void syscon_get_touchpanel_unk_info_front(uint16_t *data)
{
	uint8_t buff[SYSCON_RX_HEADER_SIZE + 2];
	syscon_command_read(0x3a7, buff, sizeof(buff));
	*data = buff[SYSCON_RX_DATA] | ((uint16_t)buff[SYSCON_RX_DATA + 1] << 8);
}

void syscon_get_touchpanel_unk_info_back(uint16_t *data)
{
	uint8_t buff[SYSCON_RX_HEADER_SIZE + 2];
	syscon_command_read(0x3b7, buff, sizeof(buff));
	*data = buff[SYSCON_RX_DATA] | ((uint16_t)buff[SYSCON_RX_DATA + 1] << 8);
}

void syscon_touch_set_sampling_cycle(int cycles_front, int cycles_back)
{
	uint8_t buffer[SYSCON_TX_HEADER_SIZE + 8];
	uint8_t rx[16];

	buffer[0] = 0x87;
	buffer[1] = 3;
	buffer[2] = 9;

	if (cycles_front >= 0) {
		buffer[3] = 1;
		buffer[4] = cycles_front & 0xFF;
	} else {
		buffer[3] = 0;
		buffer[4] = 0;
	}

	buffer[5] = 0;
	buffer[6] = 0;

	if (cycles_back >= 0) {
		buffer[7] = 1;
		buffer[8] = cycles_back & 0xFF;
	} else {
		buffer[7] = 0;
		buffer[8] = 0;
	}

	buffer[9] = 0;
	buffer[10] = 0;

	syscon_transfer(buffer, sizeof(buffer), rx, sizeof(rx));
}

//////////////////////////////////////////////////////

#define CTRL_BUTTON_HELD(ctrl, button)		((ctrl) & (button))
#define CTRL_BUTTON_PRESSED(ctrl, old, button)	(((ctrl) & ~(old)) & (button))

#define CTRL_UP		(1 << 0)
#define CTRL_RIGHT	(1 << 1)
#define CTRL_DOWN	(1 << 2)
#define CTRL_LEFT	(1 << 3)
#define CTRL_TRIANGLE	(1 << 4)
#define CTRL_CIRCLE	(1 << 5)
#define CTRL_CROSS	(1 << 6)
#define CTRL_SQUARE	(1 << 7)
#define CTRL_SELECT	(1 << 8)
#define CTRL_L		(1 << 9)
#define CTRL_R		(1 << 10)
#define CTRL_START	(1 << 11)
#define CTRL_PSBUTTON	(1 << 12)
#define CTRL_POWER	(1 << 14)
#define CTRL_VOLUP	(1 << 16)
#define CTRL_VOLDOWN	(1 << 17)
#define CTRL_HEADPHONE	(1 << 27)

struct ctrl_data {
	uint32_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
};

// Packet 0x101 is 10 bytes long
// Packet 0x104 is 14 bytes long, includes arrows pressure

void ctrl_read(struct ctrl_data *data)
{
	uint8_t buffer[SYSCON_RX_HEADER_SIZE + 10];

	syscon_command_read(0x101, buffer, sizeof(buffer));

	data->buttons = (buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) |
	                (buffer[6] << 24)) ^ 0x4037fcf9;
	data->lx = buffer[8];
	data->ly = buffer[9];
	data->rx = buffer[10];
	data->ry = buffer[11];
}

void ctrl_set_analog_sampling(int enable)
{
	uint8_t data;

	if (enable) {
		if (syscon_get_baryon_version() < 0x90202)
			data = 1;
		else
			data = 3;
	} else {
		data = 0;
	}

	syscon_short_command_write(0x180, data, 1);
}

//////////////////////////////////////////////////////

#define TOUCH_PORT_FRONT (1 << 0)
#define TOUCH_PORT_BACK  (1 << 1)

#define TOUCH_MAX_REPORT_FRONT 6
#define TOUCH_MAX_REPORT_BACK  4

#define TOUCH_REPORT_DATA_SIZE      (0x7a - 2)
#define TOUCH_REPORT_PORT_ENTRIES   10

struct touch_report {
	uint8_t id; // bit 8: invalid
	uint8_t force;
	uint16_t x; // 11 bits
	uint16_t y; // 11 bits
};

struct touch_data {
	uint32_t num_front;
	uint32_t num_back;
	struct touch_report front[TOUCH_MAX_REPORT_FRONT];
	struct touch_report back[TOUCH_MAX_REPORT_FRONT];
};

static uint16_t g_front_vendor_id;
static uint16_t g_front_fw_version;
static uint16_t g_front_unk1;
static uint8_t g_front_unk2;
static uint8_t g_front_unk3;
static uint16_t g_back_vendor_id;
static uint16_t g_back_fw_version;
static uint16_t g_back_unk1;
static uint8_t g_back_unk2;
static uint8_t g_back_unk3;
static uint8_t g_front_vendor_id_non_dependant;
static uint8_t g_back_vendor_id_dependant;
static int g_syscon_hw_version_dependant;

static void touch_read_panel_info_1(void)
{
	struct syscon_touchpanel_device_info info;

	syscon_get_touchpanel_device_info(&info);

	g_front_vendor_id = info.front_vendor_id;
	g_front_fw_version = info.front_fw_version;
	g_back_vendor_id = info.back_vendor_id;
	g_back_fw_version = info.back_fw_version;

	if (syscon_get_baryon_version() > 0x90002) {
		uint16_t data;

		syscon_get_touchpanel_unk_info_front(&data);
		g_front_unk1 = (data >> 8) | (data << 8);

		syscon_get_touchpanel_unk_info_back(&data);
		g_back_unk1 = (data >> 8) | (data << 8);
	}
}

static void touch_read_panel_info_2(void)
{
	struct syscon_touchpanel_device_info_ext info;

	syscon_get_touchpanel_device_info_ext(&info);

	g_front_vendor_id = info.front_vendor_id;
	g_front_fw_version = info.front_fw_version;
	g_front_unk1 = info.front_unk1;
	g_front_unk2 = info.front_unk2;
	g_front_unk3 = info.front_unk3;
	g_back_vendor_id = info.back_vendor_id;
	g_back_fw_version = info.back_fw_version;
	g_back_unk1 = info.back_unk1;
	g_back_unk2 = info.back_unk2;
	g_back_unk3 = info.back_unk3;
}

void touch_init(void)
{
	uint32_t baryon_version = syscon_get_baryon_version();

	//syscon_ctrl_device_reset(0xC, 0);
	//delay(1s)
	syscon_ctrl_device_reset(0xC, 1);

	if (baryon_version < 0x1000600) {
		touch_read_panel_info_1();
	} else {
		touch_read_panel_info_2();
	}

	g_front_vendor_id_non_dependant = 0xe0;
	if (g_back_vendor_id == 0x800a)
		g_back_vendor_id_dependant = 0xd0;
	else
		g_back_vendor_id_dependant = 0xb0;

	if (baryon_version > 0x90002)
		syscon_touch_set_sampling_cycle(0, 0);

	if (syscon_get_hardware_info() & 0x10000)
		g_syscon_hw_version_dependant = 0;
	else
		g_syscon_hw_version_dependant = 1;
}

void touch_configure(int port_mask, uint8_t max_report_front, uint8_t max_report_back)
{
	uint8_t buffer[SYSCON_TX_HEADER_SIZE + 4];
	uint8_t rx[16];
	uint8_t front_cfg, back_cfg;

	port_mask &= 3;

	if (port_mask == 0) {
		front_cfg = 0;
		back_cfg = 0;
	} else {
		if ((port_mask == 2) || (port_mask == 3)) {
			front_cfg = 0;
			back_cfg = g_back_vendor_id_dependant;
		} else if (port_mask == 1) {
			front_cfg = g_front_vendor_id_non_dependant;
			back_cfg = 0;
		} else {
			front_cfg = 0;
			back_cfg = 0;
		}
	}

	if (max_report_front > TOUCH_MAX_REPORT_FRONT)
		max_report_front = TOUCH_MAX_REPORT_FRONT;

	if (max_report_back > TOUCH_MAX_REPORT_BACK)
		max_report_back = TOUCH_MAX_REPORT_BACK;

	buffer[SYSCON_TX_CMD_LO] = 0x81;
	buffer[SYSCON_TX_CMD_HI] = 3;
	buffer[SYSCON_TX_LENGTH] = 5;
	buffer[SYSCON_TX_DATA(0)] = port_mask & 1;
	buffer[SYSCON_TX_DATA(1)] = front_cfg | max_report_front;
	buffer[SYSCON_TX_DATA(2)] = (port_mask >> 1) & 1;
	buffer[SYSCON_TX_DATA(3)] = back_cfg | max_report_back;

	syscon_transfer(buffer, sizeof(buffer), rx, sizeof(rx));
}

void touch_set_sampling_cycle(int port_mask, uint8_t cycles_front, uint8_t cycles_back)
{
	uint8_t buffer[SYSCON_TX_HEADER_SIZE + 8];
	uint8_t rx[16];

	buffer[SYSCON_TX_CMD_LO] = 0x87;
	buffer[SYSCON_TX_CMD_HI] = 3;
	buffer[SYSCON_TX_LENGTH] = 9;
	buffer[SYSCON_TX_DATA(0)] = port_mask & 1;
	buffer[SYSCON_TX_DATA(1)] = cycles_front;
	buffer[SYSCON_TX_DATA(2)] = 0;
	buffer[SYSCON_TX_DATA(3)] = 0;
	buffer[SYSCON_TX_DATA(4)] = (port_mask >> 1) & 1;
	buffer[SYSCON_TX_DATA(5)] = cycles_back;
	buffer[SYSCON_TX_DATA(6)] = 0;
	buffer[SYSCON_TX_DATA(7)] = 0;

	syscon_transfer(buffer, sizeof(buffer), rx, sizeof(rx));
}

void touch_read(int port_mask, struct touch_data *data)
{
	static uint8_t buffer[SYSCON_RX_HEADER_SIZE + TOUCH_REPORT_DATA_SIZE];
	uint8_t *raw_data = &buffer[SYSCON_RX_DATA];
	int offset;
	uint8_t cmd_lo;
	int i;

	if ((port_mask & 3) == 3)
		cmd_lo = 0;
	else
		cmd_lo = port_mask & 3;

	syscon_command_read(0x300 + cmd_lo, buffer, sizeof(buffer));

	data->num_front = 0;
	if (port_mask & TOUCH_PORT_FRONT) {
		offset = 0;
		for (i = 0; i < TOUCH_MAX_REPORT_FRONT; i++) {
			uint8_t id = raw_data[offset];
			if (!(id & 0x80)) {
				struct touch_report *report = &data->front[data->num_front++];
				report->id = id;
				report->force = raw_data[offset + 5];
				report->x = raw_data[offset + 1] | (((uint16_t)raw_data[offset + 2] & 7) << 8);
				report->y = raw_data[offset + 3] | (((uint16_t)raw_data[offset + 4] & 7) << 8);
			}
			offset += 6;
		}
	}

	data->num_back = 0;
	if (port_mask & TOUCH_PORT_BACK) {
		offset = TOUCH_REPORT_PORT_ENTRIES * 6;
		for (i = 0; i < TOUCH_MAX_REPORT_BACK; i++) {
			uint8_t id = raw_data[offset];
			if (!(id & 0x80)) {
				struct touch_report *report = &data->back[data->num_back++];
				report->id = id;
				report->force = raw_data[offset + 5];
				report->x = raw_data[offset + 1] | (((uint16_t)raw_data[offset + 2] & 7) << 8);
				report->y = raw_data[offset + 3] | (((uint16_t)raw_data[offset + 4] & 7) << 8);
			}
			offset += 6;
		}
	}
}

/////////////////////////////////////////////////////

static void vita_ts_poll(struct input_dev *input_dev)
{
	struct vita_ts *ts = input_get_drvdata(input_dev);
	struct ctrl_data ctrl;
	struct touch_data touch;
	static bool down = false;

	/*ctrl_read(&ctrl);

	if (CTRL_BUTTON_HELD(ctrl.buttons, CTRL_CROSS) || CTRL_BUTTON_HELD(ctrl.buttons, CTRL_L))
		input_report_key(input_dev, BTN_LEFT, 1);
	if (CTRL_BUTTON_HELD(ctrl.buttons, CTRL_SQUARE) || CTRL_BUTTON_HELD(ctrl.buttons, CTRL_R))
		input_report_key(input_dev, BTN_RIGHT, 1);
	if (CTRL_BUTTON_HELD(ctrl.buttons, CTRL_TRIANGLE))
		input_report_key(input_dev, BTN_MIDDLE, 1);

	input_report_rel(input_dev, REL_X, (int8_t)ctrl.rx - 128);
	input_report_rel(input_dev, REL_Y, (int8_t)ctrl.ry - 128);*/

	touch_read(TOUCH_PORT_FRONT | TOUCH_PORT_BACK, &touch);

	if (touch.num_front > 0) {
		if (!down) {
			input_report_key(input_dev, BTN_TOUCH, 1);
			down = true;
		}
		input_report_abs(input_dev, ABS_X, touch.front[0].x);
		input_report_abs(input_dev, ABS_Y, touch.front[0].y);
		input_report_abs(input_dev, ABS_PRESSURE, touch.front[0].force);
		input_sync(input_dev);
	} else {
		if (down) {
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_sync(input_dev);
			down = false;
		}
	}
}

static int vita_ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_ts *ts;
	struct input_dev *input_dev;
	int error;

	ts = devm_kzalloc(&pdev->dev, sizeof(struct vita_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->dev = dev;

	ts->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ts->base))
		return PTR_ERR(ts->base);

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev)
		return -ENOMEM;

	ts->input = input_dev;
	input_set_drvdata(input_dev, ts);

	/*input_set_capability(input_dev, EV_REL, REL_X);
	input_set_capability(input_dev, EV_REL, REL_Y);
	input_set_capability(input_dev, EV_KEY, BTN_LEFT);
	input_set_capability(input_dev, EV_KEY, BTN_MIDDLE);
	input_set_capability(input_dev, EV_KEY, BTN_RIGHT);*/

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, 1920, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1080, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_8BIT, 0, 0);

	input_dev->name = "PlayStation Vita Touchscreen";
	input_dev->phys = "vita_ts0";
	input_dev->dev.parent = ts->dev;
	input_dev->id.bustype = BUS_SPI;

	error = input_setup_polling(input_dev, vita_ts_poll);
	if (error)
		return error;


	g_pervasive_gate = ioremap(PERVASIVE_GATE_BASE_ADDR, 0x1000);
	g_pervasive_reset = ioremap(PERVASIVE_RESET_BASE_ADDR, 0x1000);
	g_gpio0_addr = ioremap(GPIO0_BASE_ADDR, 0x1000);
	g_gpio1_addr = ioremap(GPIO1_BASE_ADDR, 0x1000);
	g_spi0_addr = ioremap(SPI_BASE_ADDR, 0x1000);

	syscon_init();

	ctrl_set_analog_sampling(1);

	touch_init();
	touch_configure(TOUCH_PORT_FRONT | TOUCH_PORT_BACK,
	                TOUCH_MAX_REPORT_FRONT,
	                TOUCH_MAX_REPORT_BACK);
	touch_set_sampling_cycle(TOUCH_PORT_FRONT | TOUCH_PORT_BACK, 0xFF, 0xFF);

	input_set_poll_interval(input_dev, DEFAULT_POLL_PERIOD);

	error = input_register_device(input_dev);
	if (error)
		return error;

	return 0;
}

static const struct of_device_id vita_ts_of_match[] = {
	{ .compatible = "vita,touchscreen", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vita_ts_of_match);

static struct platform_driver vita_ts_driver = {
	.driver = {
		.name	= "vita-ts",
		.of_match_table = of_match_ptr(vita_ts_of_match),
	},
	.probe	= vita_ts_probe,
};

module_platform_driver(vita_ts_driver);

MODULE_DESCRIPTION("PlayStation Vita touchscreen support driver");
MODULE_AUTHOR("Sergi Granell");
MODULE_LICENSE("GPL");
