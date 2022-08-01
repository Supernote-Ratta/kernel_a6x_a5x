/*
 * Rockchip EBOOK waveform driver.
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Yakir Yang <ykk@rock-chips.com>
 *	   Dai Lunxue <dlx@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef HTFY_DBG_H
#define HTFY_DBG_H
#include <linux/input.h>
#include <linux/fb.h>

// 20211122: always define.
//#ifdef CONFIG_HTFY_DEBUG
/**
 * 20191031:主要用来定义动态的打印信息，用于EBC的打印。因为EBC是打包的.
 */
extern int ht_ebc_dbg_bits;

#define 	EDBG_LUT 			(1<<0)
#define 	EDBG_USER 			(1<<1)		//user callback,like ioctrl/read/
#define 	EDBG_BUF 			(1<<2)
#define 	EDBG_IQRFRAME		(1<<3)
#define 	EDBG_AUTO 			(1<<4)
#define 	EDBG_XALIGN			(1<<5)		// align msg.
#define 	EDBG_POWER 			(1<<6)
#define 	EDBG_DRAW 			(1<<7)
#define 	EDBG_WF 			(1<<8)		//for waveform.

#define 	EDBG_INIT			(1<<9)
#define 	EDBG_TCON 			(1<<10)
#define 	EDBG_LCDC 			(1<<11)
#define 	EDBG_VIDEO 			(1<<12)
#define 	EDBG_SWITCH			(1<<13)
#define 	EDBG_WARN 			(1<<14)		// warning msg.
#define 	EDBG_CONFLICT   	(1<<15)	

#define 	IDBG_WACOM 			(1<<16)

// __func__,  __LINE__,
#define dbg_printk(bit, fmt, arg...) do {   \
	  if (ht_ebc_dbg_bits & bit)	   \
	  printk("*EDBG*" fmt, ## arg); } while (0)
#else
#define dbg_printk(bit, fmt, arg...)	do {} while(0);
#endif


static inline bool htfy_gdb_enable(int bits) {
	return (ht_ebc_dbg_bits & bits)?1:0;
}

/*---------------------------------------------------------------*/
// HAHA: 临时用来定义和注册 电磁笔的输入回调函数，有 DETECT 和 TOUCH两种.
typedef void (*touch_detect) (/*struct input_dev *dev,*/bool pen_on);
typedef void (*touch_input) (struct input_dev *dev, int x, int y, int press, bool touch);  // touch/erase

void register_touch_detect(touch_detect det_fun);
void register_touch_input(touch_input input_fun);

// 20200615: call directly instead of register_touch_detect.
void ebc_set_tp_power(/*struct input_dev *dev,*/bool pen_on);

// 20191207: 我们在suspend的时候通过这个函数来判断是灭屏休眠还是亮屏休眠.
bool fb_eink(struct fb_info *info);
bool fb_power_off(void);

// 20201214: 增加深度休眠后是否允许 alarmtimer 唤醒系统的配置。设置代码为了方便，放在
// rk817_battery.c 的 sys 接口上面.
bool alarmtimer_rtc_wakeup_enabled(void);

//#endif // HTFY_DBG_H

