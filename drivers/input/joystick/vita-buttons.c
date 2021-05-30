// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mfd/vita-syscon.h>

#define CTRL_UP		BIT(0)
#define CTRL_RIGHT	BIT(1)
#define CTRL_DOWN	BIT(2)
#define CTRL_LEFT	BIT(3)
#define CTRL_TRIANGLE	BIT(4)
#define CTRL_CIRCLE	BIT(5)
#define CTRL_CROSS	BIT(6)
#define CTRL_SQUARE	BIT(7)
#define CTRL_SELECT	BIT(8)
#define CTRL_LTRIGGER	BIT(9)
#define CTRL_RTRIGGER	BIT(10)
#define CTRL_START	BIT(11)
#define CTRL_PSBUTTON	BIT(12)
#define CTRL_POWER	BIT(14)
#define CTRL_VOLUP	BIT(16)
#define CTRL_VOLDOWN	BIT(17)
#define CTRL_HEADPHONE	BIT(27)

struct vita_syscon_buttons {
	struct vita_syscon *syscon;
	struct input_dev *input;
	struct device *dev;
};

static void vita_syscon_buttons_poll(struct input_dev *input)
{
	struct vita_syscon_buttons *pad = input_get_drvdata(input);
	u8 data[SYSCON_RX_HEADER_SIZE + 10 + 1];
	u32 buttons;
	int ret;

	// Packet 0x101 is 10 bytes long
	// Packet 0x104 is 14 bytes long, includes arrows pressure
	ret = pad->syscon->command_read(pad->syscon, 0x101, data, sizeof(data));
	if (ret < 0)
		return;

	buttons = (data[4] | (data[5] << 8) | (data[6] << 16) |
		  (data[6] << 24)) ^ 0x4037fcf9;

	input_report_abs(input, ABS_X, data[8]);
	input_report_abs(input, ABS_Y, data[9]);
	input_report_abs(input, ABS_RX, data[10]);
	input_report_abs(input, ABS_RY, data[11]);
	input_report_key(input, BTN_DPAD_UP, buttons & CTRL_UP);
	input_report_key(input, BTN_DPAD_DOWN, buttons & CTRL_DOWN);
	input_report_key(input, BTN_DPAD_LEFT, buttons & CTRL_LEFT);
	input_report_key(input, BTN_DPAD_RIGHT, buttons & CTRL_RIGHT);
	input_report_key(input, BTN_X, buttons & CTRL_SQUARE);
	input_report_key(input, BTN_A, buttons & CTRL_CROSS);
	input_report_key(input, BTN_B, buttons & CTRL_CIRCLE);
	input_report_key(input, BTN_Y, buttons & CTRL_TRIANGLE);
	input_report_key(input, BTN_TL, buttons & CTRL_LTRIGGER);
	input_report_key(input, BTN_TR, buttons & CTRL_RTRIGGER);
	input_report_key(input, BTN_SELECT, buttons & CTRL_SELECT);
	input_report_key(input, BTN_START, buttons & CTRL_START);
	input_report_key(input, BTN_MODE, buttons & CTRL_PSBUTTON);
	input_report_key(input, KEY_VOLUMEUP, buttons & CTRL_VOLUP);
	input_report_key(input, KEY_VOLUMEDOWN, buttons & CTRL_VOLDOWN);
	input_report_key(input, KEY_POWER, buttons & CTRL_POWER);

	input_sync(input);
}

static int vita_syscon_buttons_probe(struct platform_device *pdev)
{
	int err;
	struct vita_syscon_buttons *pad;
	struct input_dev *idev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	pr_info("vita_syscon_buttons_probe\n");

	pad = devm_kzalloc(dev, sizeof(struct vita_syscon_buttons), GFP_KERNEL);
	if (!pad)
		return -ENOMEM;

	pad->syscon = syscon;
	pad->dev = dev;

	idev = devm_input_allocate_device(dev);
	if (!idev)
		return -ENOMEM;

	pad->input = idev;
	input_set_drvdata(idev, pad);
	idev->name = "PlayStation Vita Buttons (Syscon)";
	idev->phys = "vita_syscon_buttons";
	idev->dev.parent = pad->dev;
	idev->id.bustype = BUS_SPI;

	input_set_abs_params(idev, ABS_X, 0, 255, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 255, 0, 0);
	input_set_abs_params(idev, ABS_RX, 0, 255, 0, 0);
	input_set_abs_params(idev, ABS_RY, 0, 255, 0, 0);
	input_set_capability(idev, EV_KEY, BTN_DPAD_UP);
	input_set_capability(idev, EV_KEY, BTN_DPAD_DOWN);
	input_set_capability(idev, EV_KEY, BTN_DPAD_LEFT);
	input_set_capability(idev, EV_KEY, BTN_DPAD_RIGHT);
	input_set_capability(idev, EV_KEY, BTN_A);
	input_set_capability(idev, EV_KEY, BTN_B);
	input_set_capability(idev, EV_KEY, BTN_X);
	input_set_capability(idev, EV_KEY, BTN_Y);
	input_set_capability(idev, EV_KEY, BTN_TL);
	input_set_capability(idev, EV_KEY, BTN_TR);
	input_set_capability(idev, EV_KEY, BTN_SELECT);
	input_set_capability(idev, EV_KEY, BTN_START);
	input_set_capability(idev, EV_KEY, BTN_MODE);
	input_set_capability(idev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(idev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(idev, EV_KEY, KEY_POWER);

	err = input_setup_polling(idev, vita_syscon_buttons_poll);
	if (err) {
		dev_err(pad->dev, "failed to set up polling: %d\n", err);
		return err;
	}

	input_set_poll_interval(idev, 16);

	err = input_register_device(idev);
	if (err) {
		dev_err(pad->dev,
			"failed to register input device: %d\n", err);
		return err;
	}

	return 0;
}

static const struct of_device_id vita_syscon_buttons_of_match[] = {
	{ .compatible = "vita,syscon-buttons" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_buttons_of_match);

static struct platform_driver vita_syscon_buttons_driver = {
	.driver = {
		.name = "vita-syscon-buttons",
		.of_match_table = vita_syscon_buttons_of_match,
	},
	.probe = vita_syscon_buttons_probe,
};
module_platform_driver(vita_syscon_buttons_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita buttons driver");
MODULE_LICENSE("GPL");
