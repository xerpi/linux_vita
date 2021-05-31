// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/mfd/vita-syscon.h>

#define SECONDS_FROM_YEAR_1_TO_1970 62135596800

struct vita_rtc {
	struct rtc_device *rtc;
	struct vita_syscon *syscon;
};

static int vita_syscon_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct vita_rtc *vrtc = dev_get_drvdata(dev);
	struct vita_syscon *syscon = vrtc->syscon;
	u8 syscon_on_hsec_cmd_rx[SYSCON_RX_HEADER_SIZE + 4 + 1];
	u32 current_tick[2];
	u32 syscon_on_hsec;
	time64_t usec;
	time64_t time;
	int ret;

	ret = syscon->scratchpad_read(syscon, 0x10, current_tick, 8);
	if (ret < 0)
		return ret;

	ret = syscon->command_read(syscon, 0x11, &syscon_on_hsec_cmd_rx,
				   sizeof(syscon_on_hsec_cmd_rx));
	if (ret < 0)
		return ret;

	syscon_on_hsec = *(u32 *)&syscon_on_hsec_cmd_rx[SYSCON_RX_DATA];

	usec = (current_tick[0] << 19) |
	       (((u64)current_tick[0] >> 13) << 32) |
	       (((u64)current_tick[1] & 0xFF) << 51);

	time = div_u64(usec + (u64)syscon_on_hsec * 500000, 1000000);
	if (time < SECONDS_FROM_YEAR_1_TO_1970)
		return -EINVAL;

	rtc_time64_to_tm(time - SECONDS_FROM_YEAR_1_TO_1970, tm);

	return 0;
}

static const struct rtc_class_ops vita_rtc_ops = {
	.read_time = vita_syscon_rtc_read_time,
};

static int vita_syscon_rtc_probe(struct platform_device *pdev)
{
	struct vita_rtc *vrtc;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	vrtc = devm_kzalloc(&pdev->dev, sizeof(*vrtc), GFP_KERNEL);
	if (!vrtc)
		return -ENOMEM;

	vrtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(vrtc->rtc))
		return PTR_ERR(vrtc->rtc);

	vrtc->syscon = syscon;
	vrtc->rtc->ops = &vita_rtc_ops;
	vrtc->rtc->range_max = U64_MAX;

	platform_set_drvdata(pdev, vrtc);

	return devm_rtc_register_device(vrtc->rtc);
}

static const struct of_device_id vita_syscon_rtc_of_match[] = {
	{ .compatible = "vita,syscon-rtc" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_rtc_of_match);

static struct platform_driver vita_syscon_rtc_driver = {
	.driver = {
		.name = "vita-syscon-rtc",
		.of_match_table = vita_syscon_rtc_of_match,
	},
	.probe = vita_syscon_rtc_probe,
};
module_platform_driver(vita_syscon_rtc_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita Syscon RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-syscon-rtc");
