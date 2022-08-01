/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LTE_H
#define __LTE_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/fb.h>

struct lte_data {
	struct device *dev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct gpio_desc *vbat_gpio;
	struct gpio_desc *vbus_gpio;
	struct gpio_desc *wakeup_gpio;
	struct gpio_desc *disable_gpio;
	//struct class *cls_node;
	//struct class_attribute wakup;

    // 20210408: 增加lock用于防止系统休眠，目前系统休眠之后4G就不能使用了。
    // 另外为了加快唤醒速度，我们通过 work 来进行GPIO操作(因为需要延迟)
	struct wake_lock 		suspend_lock;
	struct delayed_work 	ctrl_work;
	struct delayed_work 	check_work;
	struct notifier_block	fb_notify;
	// 20210701: 我们在三个地方会控制LTE的GPIO: 1. ctrl-work; 2. suspend; 3. sys-fs.
	// 因此需要增加 mutex 保护。
	struct mutex			ctrl_mutex;

	int 					usb_check_fail_cnt;
	int 					usb_dev_num;
	//bool 					found_dev;
	bool                    power_on;
	bool                    active;
	bool 					active_when_suspend;
	bool 					operating;

	// 20210726: 是否休眠的时候保持 4G 模块处于连接状态。
	bool 					keep_active_suspend;

	//20210807: 休眠是否拉低 VBUS，如果不拉低，可以让4G USB 一直保持连接。
	bool 					keep_vbus_when_suspend;
};

#endif /* __LTE_H */
