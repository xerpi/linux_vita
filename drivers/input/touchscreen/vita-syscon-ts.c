// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/input/touchscreen.h>

#define TOUCH_PORT_FRONT (1 << 0)
#define TOUCH_PORT_BACK  (1 << 1)

#define TOUCH_MAX_REPORT_FRONT 6
#define TOUCH_MAX_REPORT_BACK  4

#define TOUCH_REPORT_DATA_SIZE      (0x7a - 2)
#define TOUCH_REPORT_PORT_ENTRIES   10

struct syscon_touchpanel_device_info {
	u16 front_vendor_id;
	u16 front_fw_version;
	u16 back_vendor_id;
	u16 back_fw_version;
};

struct syscon_touchpanel_device_info_ext {
	u16 front_vendor_id;
	u16 front_fw_version;
	u16 front_unk1;
	u8 front_unk2;
	u8 front_unk3;
	u16 unused1;
	u16 back_vendor_id;
	u16 back_fw_version;
	u16 back_unk1;
	u8 back_unk2;
	u8 back_unk3;
	u16 unused2;
};

struct vita_ts_vendor_data {
	u16 vendor_id;
	u16 fw_version;
	u16 unk1;
	u8 unk2;
	u8 unk3;
};

struct vita_vendor_data {
	struct vita_ts_vendor_data front;
	struct vita_ts_vendor_data back;
	u8 front_vendor_id_non_dependant;
	u8 back_vendor_id_dependant;
	u32 syscon_hw_version_dependant;
};

struct vita_syscon_ts {
	struct vita_syscon *syscon;
	struct input_dev *input;
	struct device *dev;
	struct vita_vendor_data vendor;
	struct touchscreen_properties prop;
};

struct touch_finger {
	u8 id; // bit 8: invalid
	u8 force;
	u16 x; // 11 bits
	u16 y; // 11 bits
};

struct touch_report {
	int num_front;
	int num_back;
	struct touch_finger front[TOUCH_MAX_REPORT_FRONT];
	struct touch_finger back[TOUCH_MAX_REPORT_BACK];
};

static int syscon_ctrl_device_reset(struct vita_syscon *syscon, unsigned int param_1,
				    unsigned int param_2)
{
	return syscon->short_command_write(syscon, 0x88F, param_2 | (param_1 << 8), 3);
}

static int syscon_get_touchpanel_device_info(struct vita_syscon *syscon,
					     struct syscon_touchpanel_device_info *info)
{
	unsigned char rx[SYSCON_RX_HEADER_SIZE + sizeof(*info) + 1];
	u8 *data = &rx[SYSCON_RX_DATA];
	int ret;

	ret = syscon->command_read(syscon, 0x380, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	info->front_vendor_id  = (data[0] << 8) | data[1];
	info->front_fw_version = (data[2] << 8) | data[3];
	info->back_vendor_id   = (data[4] << 8) | data[5];
	info->back_fw_version  = (data[6] << 8) | data[7];

	return 0;
}

static int syscon_get_touchpanel_device_info_ext(struct vita_syscon *syscon,
						 struct syscon_touchpanel_device_info_ext *info)
{
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(*info) + 1];
	u8 *data = &rx[SYSCON_RX_DATA];
	int ret;

	ret = syscon->command_read(syscon, 0x390, rx, sizeof(rx));
	if (ret < 0)
		return ret;

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

	return 0;
}

static int syscon_get_touchpanel_unk_info_front(struct vita_syscon *syscon, u16 *data)
{
	int ret;
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(u16) + 1];

	ret = syscon->command_read(syscon, 0x3a7, rx, sizeof(rx));

	*data = rx[SYSCON_RX_DATA] | ((u16)rx[SYSCON_RX_DATA + 1] << 8);

	return ret;
}

static int syscon_get_touchpanel_unk_info_back(struct vita_syscon *syscon, u16 *data)
{
	int ret;
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(u16) + 1];

	ret = syscon->command_read(syscon, 0x3b7, rx, sizeof(rx));

	*data = rx[SYSCON_RX_DATA] | ((u16)rx[SYSCON_RX_DATA + 1] << 8);

	return ret;
}

static int syscon_touch_set_sampling_cycle(struct vita_syscon *syscon, int cycles_front,
					   int cycles_back)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 8 + 1];
	u8 rx[16];

	tx[SYSCON_TX_CMD_LO] = 0x87;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 9;
	tx[SYSCON_TX_DATA(0)] = (cycles_front >= 0) ? 1 : 0;
	tx[SYSCON_TX_DATA(1)] = (cycles_front >= 0) ? (cycles_front & 0xFF) : 0;
	tx[SYSCON_TX_DATA(2)] = 0;
	tx[SYSCON_TX_DATA(3)] = 0;
	tx[SYSCON_TX_DATA(4)] = (cycles_back >= 0) ? 1 : 0;
	tx[SYSCON_TX_DATA(5)] = (cycles_back >= 0) ? (cycles_back & 0xFF) : 0;
	tx[SYSCON_TX_DATA(6)] = 0;
	tx[SYSCON_TX_DATA(7)] = 0;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static int touch_read_panel_info_1(struct vita_syscon *syscon,
				   struct vita_ts_vendor_data *front,
				   struct vita_ts_vendor_data *back)
{
	struct syscon_touchpanel_device_info info;
	u16 data;
	int ret;

	ret = syscon_get_touchpanel_device_info(syscon, &info);
	if (ret < 0)
		return ret;

	front->vendor_id = info.front_vendor_id;
	front->fw_version = info.front_fw_version;
	back->vendor_id = info.back_vendor_id;
	back->fw_version = info.back_fw_version;

	if (syscon->baryon_version > 0x90002) {
		ret = syscon_get_touchpanel_unk_info_front(syscon, &data);
		if (ret < 0)
			return ret;

		front->unk1 = (data >> 8) | (data << 8);

		ret = syscon_get_touchpanel_unk_info_back(syscon, &data);
		if (ret < 0)
			return ret;

		back->unk1 = (data >> 8) | (data << 8);
	}

	return 0;
}

static int touch_read_panel_info_2(struct vita_syscon *syscon,
				   struct vita_ts_vendor_data *front,
				   struct vita_ts_vendor_data *back)
{
	struct syscon_touchpanel_device_info_ext info;
	int ret;

	ret = syscon_get_touchpanel_device_info_ext(syscon, &info);
	if (ret < 0)
		return ret;

	front->vendor_id = info.front_vendor_id;
	front->fw_version = info.front_fw_version;
	front->unk1 = info.front_unk1;
	front->unk2 = info.front_unk2;
	front->unk3 = info.front_unk3;
	back->vendor_id = info.back_vendor_id;
	back->fw_version = info.back_fw_version;
	back->unk1 = info.back_unk1;
	back->unk2 = info.back_unk2;
	back->unk3 = info.back_unk3;

	return 0;
}

static int touch_init(struct vita_syscon_ts *ts)
{
	struct vita_syscon *syscon = ts->syscon;
	uint32_t baryon_version = syscon->baryon_version;
	uint32_t hw_info = syscon->hardware_info;
	int ret;

	ret = syscon_ctrl_device_reset(syscon, 0xC, 0);
	if (ret < 0)
		return ret;

	udelay(1000);

	ret = syscon_ctrl_device_reset(syscon, 0xC, 1);
	if (ret < 0)
		return ret;

	mdelay(100);

	if (baryon_version < 0x1000600) {
		ret = touch_read_panel_info_1(syscon, &ts->vendor.front, &ts->vendor.back);
	} else {
		ret = touch_read_panel_info_2(syscon, &ts->vendor.front, &ts->vendor.back);
	}
	if (ret < 0)
		return ret;

	ts->vendor.front_vendor_id_non_dependant = 0xe0;
	if (ts->vendor.back.vendor_id == 0x800a)
		ts->vendor.back_vendor_id_dependant = 0xd0;
	else
		ts->vendor.back_vendor_id_dependant = 0xb0;

	if (baryon_version > 0x90002) {
		ret = syscon_touch_set_sampling_cycle(syscon, 0, 0);
		if (ret < 0)
			return ret;
	}

	if (hw_info & 0x10000)
		ts->vendor.syscon_hw_version_dependant = 0;
	else
		ts->vendor.syscon_hw_version_dependant = 1;

	return 0;
}

static int touch_configure(struct vita_syscon_ts *ts, int port_mask,
			   u8 max_report_front, u8 max_report_back)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 tx[SYSCON_TX_HEADER_SIZE + 4 + 1];
	u8 rx[16];
	u8 front_cfg, back_cfg;

	port_mask &= 3;

	if (port_mask == 0) {
		front_cfg = 0;
		back_cfg = 0;
	} else {
		if ((port_mask == 2) || (port_mask == 3)) {
			front_cfg = 0;
			back_cfg = ts->vendor.back_vendor_id_dependant;
		} else if (port_mask == 1) {
			front_cfg = ts->vendor.front_vendor_id_non_dependant;
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

	tx[SYSCON_TX_CMD_LO] = 0x81;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 5;
	tx[SYSCON_TX_DATA(0)] = port_mask & 1;
	tx[SYSCON_TX_DATA(1)] = front_cfg | max_report_front;
	tx[SYSCON_TX_DATA(2)] = (port_mask >> 1) & 1;
	tx[SYSCON_TX_DATA(3)] = back_cfg | max_report_back;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static int touch_set_sampling_cycle(struct vita_syscon_ts *ts, int port_mask,
				    u8 cycles_front, u8 cycles_back)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 tx[SYSCON_TX_HEADER_SIZE + 8 + 1];
	u8 rx[16];

	tx[SYSCON_TX_CMD_LO] = 0x87;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 9;
	tx[SYSCON_TX_DATA(0)] = port_mask & 1;
	tx[SYSCON_TX_DATA(1)] = cycles_front;
	tx[SYSCON_TX_DATA(2)] = 0;
	tx[SYSCON_TX_DATA(3)] = 0;
	tx[SYSCON_TX_DATA(4)] = (port_mask >> 1) & 1;
	tx[SYSCON_TX_DATA(5)] = cycles_back;
	tx[SYSCON_TX_DATA(6)] = 0;
	tx[SYSCON_TX_DATA(7)] = 0;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static inline bool touch_parse_finger(struct touch_finger *finger, const u8 *data)
{
	u8 id = data[0];
	if (id & 0x80)
		return false;

	finger->id = id;
	finger->force = data[5];
	finger->x = data[1] | (((u16)data[2] & 7) << 8);
	finger->y = data[3] | (((u16)data[4] & 7) << 8);

	return true;
}

static int touch_read(struct vita_syscon_ts *ts, int port_mask, struct touch_report *report)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 buffer[SYSCON_RX_HEADER_SIZE + TOUCH_REPORT_DATA_SIZE + 1];
	u8 *raw_data = &buffer[SYSCON_RX_DATA];
	struct touch_finger *finger;
	int offset;
	u8 cmd_lo;
	int i, ret;

	if ((port_mask & 3) == 3)
		cmd_lo = 0;
	else
		cmd_lo = port_mask & 3;

	ret = syscon->command_read(syscon, 0x300 + cmd_lo, buffer, sizeof(buffer));
	if (ret < 0)
		return ret;

	report->num_front = 0;
	if (port_mask & TOUCH_PORT_FRONT) {
		offset = 0;
		for (i = 0; i < TOUCH_MAX_REPORT_FRONT; i++) {
			finger = &report->front[report->num_front];

			if (touch_parse_finger(finger, &raw_data[offset]))
				report->num_front++;

			offset += 6;
		}
	}

	report->num_back = 0;
	if (port_mask & TOUCH_PORT_BACK) {
		offset = TOUCH_REPORT_PORT_ENTRIES * 6;
		for (i = 0; i < TOUCH_MAX_REPORT_BACK; i++) {
			finger = &report->back[report->num_back];

			if (touch_parse_finger(finger, &raw_data[offset]))
				report->num_back++;

			offset += 6;
		}
	}

	return 0;
}

static void vita_syscon_ts_poll(struct input_dev *input_dev)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input_dev);
	struct touch_report touch;
	int i, ret;

	ret = touch_read(ts, TOUCH_PORT_FRONT, &touch);
	if (ret < 0)
		return;

	for (i = 0; i < touch.num_front; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
		input_report_abs(input_dev, ABS_MT_POSITION_X, touch.front[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, touch.front[i].y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, touch.front[i].force);
	}

	input_mt_sync_frame(input_dev);
	input_sync(input_dev);
}

static int vita_syscon_ts_open(struct input_dev *input)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input);

	//pr_info("vita_syscon_ts_open\n");

	syscon_ctrl_device_reset(ts->syscon, 0xC, 1);
	mdelay(100);

	return 0;
}

static void vita_syscon_ts_close(struct input_dev *input)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input);

	//pr_info("vita_syscon_ts_close\n");

	syscon_ctrl_device_reset(ts->syscon, 0xC, 0);
	udelay(1000);
}

static int vita_syscon_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct vita_syscon_ts *ts;
	struct input_dev *input_dev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	pr_info("vita_syscon_ts_probe\n");

	ts = devm_kzalloc(dev, sizeof(struct vita_syscon_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->syscon = syscon;
	ts->dev = dev;

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;

	ts->input = input_dev;
	input_set_drvdata(input_dev, ts);

	input_dev->name = "PlayStation Vita Touchscreen (Syscon)";
	input_dev->phys = "vita_syscon_ts";
	input_dev->dev.parent = ts->dev;
	input_dev->id.bustype = BUS_SPI;
	input_dev->open = vita_syscon_ts_open;
	input_dev->close = vita_syscon_ts_close;

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 1920, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1080, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(input_dev, true, &ts->prop);

	ret = input_mt_init_slots(input_dev, TOUCH_MAX_REPORT_FRONT,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "Failed to initialize MT slots: %d\n", ret);
		return ret;
	}

	ret = input_setup_polling(input_dev, vita_syscon_ts_poll);
	if (ret)
		return ret;

	input_set_poll_interval(input_dev, 16);

	ret = touch_init(ts);
	if (ret < 0) {
		return ret;
	}

	ret = touch_configure(ts, TOUCH_PORT_FRONT, TOUCH_MAX_REPORT_FRONT, 0);
	if (ret < 0) {
		return ret;
	}

	ret = touch_set_sampling_cycle(ts, TOUCH_PORT_FRONT, 0xFF, 0);
	if (ret < 0) {
		return ret;
	}

	ret = input_register_device(input_dev);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id vita_syscon_ts_of_match[] = {
	{ .compatible = "vita,syscon-ts" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_ts_of_match);

static struct platform_driver vita_syscon_ts_driver = {
	.driver = {
		.name = "vita-syscon-ts",
		.of_match_table = vita_syscon_ts_of_match,
	},
	.probe = vita_syscon_ts_probe,
};
module_platform_driver(vita_syscon_ts_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita Touchscreen Driver (Syscon)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-syscon-ts");
