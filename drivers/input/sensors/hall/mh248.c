 /*
 * drivers/input/sensors/hall/mh248.c
 *
 * Copyright (C) 2012-2016 Rockchip Co.,Ltd.
 * Author: Bin Yang <yangbin@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#include <linux/sensor-dev.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/rk_keys.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>

#ifdef CONFIG_EBC
#include "../../../gpu/drm/rockchip/eink/ebc.h"
#endif

/*
* define SUPPORT_ULTRA_SLEEP for px30 eink platform,
* if your board support ultra sleep, you should open this define
*/
//#define SUPPORT_ULTRA_SLEEP

struct mh248_para {
	struct device *dev;
	struct notifier_block fb_notif;
#ifdef CONFIG_EBC
	struct notifier_block ebc_notif;
#endif
	struct mutex ops_lock;
	int is_suspend;
	int gpio_pin;
	int irq;
	int active_value;
#ifdef SUPPORT_ULTRA_SLEEP
	void __iomem	*pmu_gpio_regs;
	int is_pmu_wakeup;
#endif
};

#ifdef SUPPORT_ULTRA_SLEEP
#define PMU_GPIO_REG_BASE 0xFF040000 //GPIO0_A7, PMIC INT PIN
#define PMU_GPIO_REG_SIZE 0x100
#define PMU_GPIO_INT_STATUS_REG 0x40
#define PMU_GPIO_INT_MASK 0x80
#endif

struct mh248_para *g_mh248 = NULL;

static int hall_fb_notifier_callback(struct notifier_block *self,
				     unsigned long action, void *data)
{
	struct mh248_para *mh248;
	struct fb_event *event = data;

	mh248 = container_of(self, struct mh248_para, fb_notif);

	mutex_lock(&mh248->ops_lock);
	if (action == FB_EARLY_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			mh248->is_suspend = 1;
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			mh248->is_suspend = 0;
			break;
		default:
			break;
		}
	}
	mutex_unlock(&mh248->ops_lock);

	return NOTIFY_OK;
}

#ifdef CONFIG_EBC
static int hall_ebc_notifier_callback(struct notifier_block *self,
				     unsigned long action, void *data)
{
	struct mh248_para *mh248;

	mh248 = container_of(self, struct mh248_para, ebc_notif);

	mutex_lock(&mh248->ops_lock);

	if (action == EBC_FB_BLANK) {
		printk("%s: ebc fb blank\n", __func__);
		mh248->is_suspend = 1;
	}
	else if (action == EBC_FB_UNBLANK) {
		printk("%s: ebc fb unblank\n", __func__);
		mh248->is_suspend = 0;
	}

	mutex_unlock(&mh248->ops_lock);

	return NOTIFY_OK;
}
#endif

static irqreturn_t hall_mh248_interrupt(int irq, void *dev_id)
{
	struct mh248_para *mh248 = (struct mh248_para *)dev_id;
	int gpio_value = 0;

	printk("--------%s: --------\n", __func__);

	gpio_value = gpio_get_value(mh248->gpio_pin);

	if ((gpio_value != mh248->active_value) &&
	    (mh248->is_suspend == 0)) {
		rk8xx_send_power_key(1);
		rk8xx_send_power_key(0);
		printk("%s: send power key\n", __func__);
	} else if ((gpio_value == mh248->active_value) &&
		   (mh248->is_suspend == 1)) {
		rk8xx_send_wakeup_key();
		printk("%s: send wakeup key\n", __func__);
	}

	return IRQ_HANDLED;
}

#ifdef SUPPORT_ULTRA_SLEEP
static int mh248_syscore_suspend(void)
{
	return 0;
}

static void mh248_syscore_resume(void)
{
	u32 intr_status;

	intr_status = readl_relaxed(g_mh248->pmu_gpio_regs + PMU_GPIO_INT_STATUS_REG);
	if (intr_status & PMU_GPIO_INT_MASK)
		g_mh248->is_pmu_wakeup = 1;
	else
		g_mh248->is_pmu_wakeup = 0;
	printk("%s: GPIO0 INT status =  %x\n", __func__, intr_status);
}

static struct syscore_ops mh248_syscore_ops = {
	.suspend = mh248_syscore_suspend,
	.resume = mh248_syscore_resume,
};
#endif

static int hall_mh248_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mh248_para *mh248;
	enum of_gpio_flags irq_flags;
	int hallactive = 0;
	int gpio_value = 0;
	int ret = 0;

	mh248 = devm_kzalloc(&pdev->dev, sizeof(*mh248), GFP_KERNEL);
	if (!mh248)
		return -ENOMEM;

	mh248->dev = &pdev->dev;

	mh248->gpio_pin = of_get_named_gpio_flags(np, "irq-gpio",
					0, &irq_flags);
	if (!gpio_is_valid(mh248->gpio_pin)) {
		dev_err(mh248->dev, "Can not read property irq-gpio\n");
		return mh248->gpio_pin;
	}
	mh248->irq = gpio_to_irq(mh248->gpio_pin);

	of_property_read_u32(np, "hall-active", &hallactive);
	mh248->active_value = hallactive;
	mh248->is_suspend = 0;
	mutex_init(&mh248->ops_lock);

	ret = devm_gpio_request_one(mh248->dev, mh248->gpio_pin,
				    GPIOF_DIR_IN, "hall_mh248");
	if (ret < 0) {
		dev_err(mh248->dev, "fail to request gpio:%d\n",
			mh248->gpio_pin);
		return ret;
	}
	gpio_value = gpio_get_value(mh248->gpio_pin);

	ret = devm_request_threaded_irq(mh248->dev, mh248->irq,
					NULL, hall_mh248_interrupt,
					irq_flags | IRQF_NO_SUSPEND | IRQF_ONESHOT,
					"hall_mh248", mh248);
	if (ret < 0) {
		dev_err(mh248->dev, "request irq(%d) failed, ret=%d\n",
			mh248->irq, ret);
		return ret;
	}

	enable_irq_wake(mh248->irq);
	mh248->fb_notif.notifier_call = hall_fb_notifier_callback;
	fb_register_client(&mh248->fb_notif);

#ifdef CONFIG_EBC
	mh248->ebc_notif.notifier_call = hall_ebc_notifier_callback;
	rkebc_register_notifier(&mh248->ebc_notif);
#endif

#ifdef SUPPORT_ULTRA_SLEEP
	mh248->pmu_gpio_regs = ioremap(PMU_GPIO_REG_BASE, PMU_GPIO_REG_SIZE);
	register_syscore_ops(&mh248_syscore_ops);
#endif

	g_mh248 = mh248;
	dev_info(mh248->dev, "hall_mh248_probe success.\n");

	return 0;
}

#ifdef SUPPORT_ULTRA_SLEEP
static int mh248_suspend(struct device *dev)
{
	suspend_state_t suspend_state = get_suspend_state();

	if (suspend_state == PM_SUSPEND_ULTRA)
		disable_irq(g_mh248->irq);

	return 0;
}

static int mh248_resume(struct device *dev)
{
	int gpio_value = 0;
	suspend_state_t suspend_state = get_suspend_state();

	if (suspend_state == PM_SUSPEND_ULTRA) {
		if (!g_mh248->is_pmu_wakeup) {
			gpio_value = gpio_get_value(g_mh248->gpio_pin);
			if ((gpio_value == g_mh248->active_value) &&
				   (g_mh248->is_suspend == 1)) {
				rk8xx_send_wakeup_key();
				printk("%s: send wakeup key\n", __func__);
			}
		}
		enable_irq(g_mh248->irq);
	}
	return 0;
}

static const struct dev_pm_ops mh248_pm = {
	.resume = mh248_resume,
	.suspend = mh248_suspend,
};
#endif

static const struct of_device_id hall_mh248_match[] = {
	{ .compatible = "hall-mh248" },
	{ /* Sentinel */ }
};

static struct platform_driver hall_mh248_driver = {
	.probe = hall_mh248_probe,
	.driver = {
		.name = "mh248",
		.owner = THIS_MODULE,
		.of_match_table	= hall_mh248_match,
#ifdef SUPPORT_ULTRA_SLEEP
		.pm = &mh248_pm,
#endif
	},
};

module_platform_driver(hall_mh248_driver);

MODULE_ALIAS("platform:mh248");
MODULE_AUTHOR("Bin Yang <yangbin@rock-chips.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hall Sensor MH248 driver");
