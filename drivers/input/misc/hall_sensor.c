/*
 * drivers/input/misc/hall_sensor.c
 *
 * Copyright (c) 2020 Ratta.
 *
 * This file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 * Either version of 2 of the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define DEVICE_NAME		"hall_sensor"
#define CONVERSION_TIME_MS	50

struct hall_sensor  {
	struct input_dev *input;
	struct delayed_work hall_work;
	spinlock_t mutex;
	int irq;
	int gpio;
};

static inline void hall_wake_worker(struct hall_sensor *hall)
{
	if (!hall)
		return;

	spin_lock(&hall->mutex);
	cancel_delayed_work(&hall->hall_work);
	flush_delayed_work(&hall->hall_work);
	schedule_delayed_work(&hall->hall_work,
			      msecs_to_jiffies(100));
	spin_unlock(&hall->mutex);
}

static irqreturn_t hall_sensor_irq(int irq, void *dev_id)
{
	struct hall_sensor *hall = dev_id;

	hall_wake_worker(hall);

	return IRQ_HANDLED;
}

static void hall_work_func(struct work_struct *work)
{
	struct hall_sensor *hall = container_of((struct delayed_work *)work,
						struct hall_sensor, hall_work);
	struct device *dev = &hall->input->dev;
	int value;

	value = gpio_get_value(hall->gpio);

	if (value == 0) {
		dev_info(dev, "Hall report sleep key.\n");
		input_report_key(hall->input, KEY_SLEEP, 1);
		input_sync(hall->input);
		input_report_key(hall->input, KEY_SLEEP, 0);
		input_sync(hall->input);
	} else if (value == 1) {
		dev_info(dev, "Hall report wakeup key.\n");
		input_report_key(hall->input, KEY_WAKEUP, 1);
		input_sync(hall->input);
		input_report_key(hall->input, KEY_WAKEUP, 0);
		input_sync(hall->input);
	}
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct hall_sensor *hall;
	int ret = 0, gpio = 0;

	if (!np)
		return -ENODEV;

	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	ret = devm_gpio_request_one(&pdev->dev, gpio, GPIOF_IN, "hall-irq");
	if (ret < 0) {
		dev_err(&pdev->dev, "request gpio failed, %d\n", ret);
		return ret;
	}

	/* Allocate memory */
	hall = devm_kzalloc(&pdev->dev, sizeof(struct hall_sensor), GFP_KERNEL);
	if (!hall)
		return -ENOMEM;

	/* Save the stuctures to be used in the driver */
	hall->gpio = gpio;
	hall->irq = gpio_to_irq(gpio);

	hall->input = devm_input_allocate_device(&pdev->dev);
	if (!hall->input)
		return -ENOMEM;

	hall->input->name = "Hall Sensor";
	hall->input->id.bustype = BUS_HOST;
	hall->input->dev.parent = &pdev->dev;
	input_set_capability(hall->input, EV_KEY, KEY_WAKEUP);
	input_set_capability(hall->input, EV_KEY, KEY_SLEEP);
	input_set_drvdata(hall->input, hall);
	INIT_DELAYED_WORK(&hall->hall_work, hall_work_func);
	spin_lock_init(&hall->mutex);

	platform_set_drvdata(pdev, hall);
	device_init_wakeup(&pdev->dev, true);

	ret = input_register_device(hall->input);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to register input device, %d\n", ret);
		goto err_register_input;
	}

	/*
	 * Request the interrupt on a falling trigger
	 * and only one trigger per falling edge
	 */
	ret = devm_request_threaded_irq(&pdev->dev,
					hall->irq, NULL,
					hall_sensor_irq,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT,
					"hall_sensor", hall);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ, %d\n", ret);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	input_unregister_device(hall->input);
err_register_input:
	device_init_wakeup(&pdev->dev, false);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_sensor *hall = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, hall->irq, hall);
	input_unregister_device(hall->input);
	device_init_wakeup(&pdev->dev, false);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id hall_dt_ids[] = {
	{
		.compatible = "ratta,hall",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, hall_dt_ids);

static int __maybe_unused hall_sensor_suspend(struct device *dev)
{
	struct hall_sensor *hall = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(hall->irq);

	return 0;
}

static int __maybe_unused hall_sensor_resume(struct device *dev)
{
	struct hall_sensor *hall = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(hall->irq);

	//hall_wake_worker(hall);

	return 0;
}

static SIMPLE_DEV_PM_OPS(hall_sensor_pm,
			 hall_sensor_suspend, hall_sensor_resume);

static struct platform_driver hall_sensor_driver = {
	.probe	= hall_sensor_probe,
	.remove	= hall_sensor_remove,
	.driver = {
		.name	= "hall_sensor",
		.owner	= THIS_MODULE,
		.pm	= &hall_sensor_pm,
		.of_match_table = hall_dt_ids,
	},
};

module_platform_driver(hall_sensor_driver);

MODULE_AUTHOR("Ratta");
MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL");
