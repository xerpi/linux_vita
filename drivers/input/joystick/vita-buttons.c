#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mfd/vita-syscon.h>

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

struct vita_syscon_buttons {
	struct vita_syscon *syscon;
	struct input_dev *input;
	struct device *dev;
};

// Packet 0x101 is 10 bytes long
// Packet 0x104 is 14 bytes long, includes arrows pressure

static void vita_syscon_buttons_poll(struct input_dev *input)
{
	struct vita_syscon_buttons *pad = input_get_drvdata(input);
	uint8_t buffer[SYSCON_RX_HEADER_SIZE + 10];
	u32 buttons;

	//printk("vita_syscon_buttons_poll\n");

	pad->syscon->command_read(pad->syscon, 0x101, buffer, sizeof(buffer));

	buttons = (buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) |
	          (buffer[6] << 24)) ^ 0x4037fcf9;
/*	data->lx = buffer[8];
	data->ly = buffer[9];
	data->rx = buffer[10];
	data->ry = buffer[11];*/

	//printk("  buttons: 0x%0X\n", buttons);

	input_report_key(input, KEY_POWER, buttons & CTRL_POWER);
	input_report_abs(input, ABS_X, buffer[8]);
	input_report_abs(input, ABS_Y, buffer[9]);
	input_report_abs(input, ABS_RX, buffer[10]);
	input_report_abs(input, ABS_RY, buffer[11]);
	input_report_key(input, BTN_DPAD_UP, buttons & CTRL_UP);
	input_report_key(input, BTN_DPAD_DOWN, buttons & CTRL_DOWN);
	input_report_key(input, BTN_DPAD_LEFT, buttons & CTRL_LEFT);
	input_report_key(input, BTN_DPAD_RIGHT, buttons & CTRL_RIGHT);
	input_report_key(input, BTN_X, buttons & CTRL_SQUARE);
	input_report_key(input, BTN_A, buttons & CTRL_CROSS);
	input_report_key(input, BTN_B, buttons & CTRL_CIRCLE);
	input_report_key(input, BTN_Y, buttons & CTRL_TRIANGLE);
	/*input_report_key(input, BTN_TL, buttons & BIT(5));
	input_report_key(input, BTN_TR, buttons & BIT(4));
	input_report_key(input, BTN_TL2, buttons & BIT(7));
	input_report_key(input, BTN_TR2, buttons & BIT(6));
	input_report_key(input, BTN_THUMBL, buttons & BIT(6));
	input_report_key(input, BTN_THUMBR, buttons & BIT(5));*/
	input_report_key(input, BTN_SELECT, buttons & CTRL_SELECT);
	input_report_key(input, BTN_START, buttons & CTRL_START);

	input_sync(input);
}

static int vita_syscon_buttons_probe(struct platform_device *pdev)
{
	int err;
	struct vita_syscon_buttons *pad;
	struct input_dev *idev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	printk("vita_syscon_buttons_probe\n");

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
	input_set_capability(idev, EV_KEY, BTN_TL2);
	input_set_capability(idev, EV_KEY, BTN_TR2);
	input_set_capability(idev, EV_KEY, BTN_THUMBL);
	input_set_capability(idev, EV_KEY, BTN_THUMBR);
	input_set_capability(idev, EV_KEY, BTN_SELECT);
	input_set_capability(idev, EV_KEY, BTN_START);
	input_set_capability(idev, EV_KEY, KEY_POWER);

	idev->name = "PlayStation Vita Buttons (Syscon)";
	idev->phys = "vita_buttons0";
	idev->dev.parent = pad->dev;
	idev->id.bustype = BUS_SPI;

	err = input_setup_polling(idev, vita_syscon_buttons_poll);
	if (err) {
		dev_err(pad->dev, "failed to set up polling: %d\n", err);
		return err;
	}

	input_set_poll_interval(idev, 10);

	/* register input poll device */
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
