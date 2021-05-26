// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DEFAULT_POLL_PERIOD 10 /* ms */
#define MAX_8BIT ((1u << 8) - 1)

struct vita_syscon_ts {
	struct vita_syscon *syscon;
	struct input_dev *input;
	struct device *dev;
};

#if 0
static void vita_syscon_ts_poll(struct input_dev *input_dev)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input_dev);
	u16 last_x = readw(ts->base + X_OFFSET);
	u16 last_y = readw(ts->base + Y_OFFSET);
	bool pendown = last_x & PENDOWN_MASK;

	if (pendown) {
		if (ts->debounce) {
			ts->debounce--;
			return;
		}

		if (!ts->pendown) {
			input_report_key(input_dev, BTN_TOUCH, 1);
			ts->pendown = true;
		}

		last_x = ((~last_x) >> 4) & MAX_12BIT;
		last_y = ((~last_y) >> 4) & MAX_12BIT;

		input_report_abs(input_dev, ABS_X, last_x);
		input_report_abs(input_dev, ABS_Y, last_y);
		input_sync(input_dev);
	} else if (ts->pendown) {
		ts->pendown = false;
		ts->debounce = DEBOUNCE_COUNT;
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_sync(input_dev);
	}
}
#endif

static void vita_syscon_ts_poll(struct input_dev *input_dev)
{
#if 0
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
#endif
}

static int vita_syscon_ts_probe(struct platform_device *pdev)
{
	int error;
	struct vita_syscon_ts *ts;
	struct input_dev *input_dev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	printk("vita_syscon_ts_probe\n");

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

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, 1920, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1080, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_8BIT, 0, 0);

	input_dev->name = "PlayStation Vita Touchscreen (Syscon)";
	input_dev->phys = "vita_ts0";
	input_dev->dev.parent = ts->dev;
	input_dev->id.bustype = BUS_SPI;

	//input_dev->open = vita_syscon_ts_open;
	//input_dev->close = vita_syscon_ts_close;

	error = input_setup_polling(input_dev, vita_syscon_ts_poll);
	if (error)
		return error;
		
		
	//syscon->read_dev(syscon, 0, 0, NULL);

	/*touch_init();
	touch_configure(TOUCH_PORT_FRONT | TOUCH_PORT_BACK,
	                TOUCH_MAX_REPORT_FRONT,
	                TOUCH_MAX_REPORT_BACK);
	touch_set_sampling_cycle(TOUCH_PORT_FRONT | TOUCH_PORT_BACK, 0xFF, 0xFF);*/

	input_set_poll_interval(input_dev, DEFAULT_POLL_PERIOD);

	error = input_register_device(input_dev);
	if (error)
		return error;

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
