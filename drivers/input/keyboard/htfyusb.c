/*
 * Copyright (c) 2021, htfyun Ltd
 * Author: Tower
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#define DEVNAME "htfy-customusb"


typedef struct {
    struct platform_device *dev;
    struct input_dev *input;
    struct wake_lock lock;
    int detect_gpio;
    int det_irq;
    bool state;
} custom_usb_info;


extern bool fb_power_off(void);

static void repo_key(custom_usb_info *info)
{
    input_report_key(info->input, KEY_WAKEUP, 1);
    input_sync(info->input);
    input_report_key(info->input, KEY_WAKEUP, 0);
    input_sync(info->input);
}

static irqreturn_t customusb_det_irq_isr(int irq, void *data)
{
    custom_usb_info *info = data;

    info->state = !gpio_get_value(info->detect_gpio);
    if (info->state) {
        //prevent system to suspend
        wake_lock(&info->lock);
        repo_key(info);
    } else {
        //when timeout will unlock auto, allow system to suspend
        wake_lock_timeout(&info->lock, 1 * HZ);
        repo_key(info);
    }

    return IRQ_HANDLED;
}

static int htfy_customusb_probe(struct platform_device *pdev)
{
    int ret = 0;
    custom_usb_info *info;
    enum of_gpio_flags flags;

    info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
    if (!info) {
        return -ENOMEM;
    }

    info->detect_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "gpio_detect", 0, &flags);
    if (!gpio_is_valid(info->detect_gpio)) {
        dev_err(&pdev->dev, "no gpio_detect pin available\n");
        goto error;
    }

    ret = devm_gpio_request_one(&pdev->dev, info->detect_gpio, GPIOF_IN, "htfy_customusb_det");
    if (ret < 0) {
        goto gpio_free;
    }

    info->det_irq = gpio_to_irq(info->detect_gpio);
    if (info->det_irq < 0) {
        dev_err(&pdev->dev, "Unable to get irq number for GPIO %d, error %d\n", info->detect_gpio, info->det_irq);
        goto gpio_free;
    }

    ret = devm_request_threaded_irq(&pdev->dev, info->det_irq, NULL, customusb_det_irq_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "htcustomusb-det", info);
    if (ret < 0) {
        dev_err(&pdev->dev, "Could not register for %s interrupt, irq=%d, ret=%d\n", "htcustomusb-det", info->det_irq, ret);
        goto gpio_free;
    }

    info->input = devm_input_allocate_device(&pdev->dev);
    info->input->name = "htfyusb";
    info->input->dev.parent = &pdev->dev;
    info->input->evbit[0] = BIT_MASK(EV_KEY);
    info->input->keybit[BIT_WORD(KEY_WAKEUP)] = BIT_MASK(KEY_WAKEUP);

    ret = input_register_device(info->input);
    if(ret) {
        dev_err(&pdev->dev, "register input device failed!!!\n");
        input_free_device(info->input);
        goto gpio_free;
    }

    platform_set_drvdata(pdev, info);
    device_init_wakeup(&pdev->dev, true);
    wake_lock_init(&info->lock, WAKE_LOCK_SUSPEND, "htfy-customusb-lock");

    return 0;

gpio_free:
    devm_gpio_free(&pdev->dev, info->detect_gpio);

error:
    if (info) {
        kfree(info);
    }

    return ret;
}

static void htfy_customusb_shutdown(struct platform_device *pdev)
{
}

#ifdef CONFIG_PM
static int htfy_customusb_suspend(struct device *dev)
{
    custom_usb_info *info = dev_get_drvdata(dev);

    printk("entering %s\n", __func__);

    if (device_may_wakeup(dev)) {
        enable_irq_wake(info->det_irq);
    }

    return 0;
}

static int htfy_customusb_resume(struct device *dev)
{
    custom_usb_info *info = dev_get_drvdata(dev);

    printk("entering %s\n", __func__);

    if (device_may_wakeup(dev)) {
        disable_irq_wake(info->det_irq);
    }

    return 0;
}

static SIMPLE_DEV_PM_OPS(htfy_customusb_pm_ops, htfy_customusb_suspend, htfy_customusb_resume);
#endif

static const struct of_device_id of_htfy_customusb_match[] = {
    { .compatible = DEVNAME, },
    {},
};

MODULE_DEVICE_TABLE(of, of_htfy_customusb_match);

static struct platform_driver htfy_customusb_driver = {
    .probe = htfy_customusb_probe,
    .shutdown = htfy_customusb_shutdown,
    .driver = {
        .name = DEVNAME,
#ifdef CONFIG_PM
        .pm = &htfy_customusb_pm_ops,
#endif
        .of_match_table = of_htfy_customusb_match,
    },
};

module_platform_driver(htfy_customusb_driver);

MODULE_AUTHOR("Tower");
MODULE_DESCRIPTION("HtFy custom usb driver.");
MODULE_LICENSE("GPL");
