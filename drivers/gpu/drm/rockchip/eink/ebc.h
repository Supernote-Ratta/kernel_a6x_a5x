/*
 * RK29 ebook control driver rk29_ebc.h
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: <Dai Lunxue> dlx@rock-chips.com
 *	   <Hunag Lin> hl@rock-chips.com
 *	   <Yang Kuankuan> ykk@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef RK29_EBC_H
#define RK29_EBC_H

#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include "epdlut/epd_lut.h" 
#include "bufmanage/buf_manage.h"
#include "epdlut/epdtemperature/epd_temperature.h"
#include "ebc_dbg/rk29_ebc_dbg.h"

#include "tcon.h"
#include "panel.h"

#define RKEBC_DRV_VERSION		"1.03"

#define EBC_SUCCESS			(0)
#define EBC_ERROR			(-1)

//#define DIRECT_MODE 
#define AUTO_MODE_ENABLE

/* Select WAVEFORM from nand or spi flash */
#define NAND_WAVEFORM			(0)
#define SPI_WAVEFORM			(1)

/*SET END DISPLAY*/
#define END_RESET			(0)
#define END_PICTURE			(1)

// ebc ioctl command
#define GET_EBC_BUFFER (0x7000)
#define SET_EBC_SEND_BUFFER (0x7001)
#define GET_EBC_DRIVER_SN (0x7002)
#define GET_EBC_BUFFER_INFO (0x7003)

#define EBC_DRIVER_SN "RK29_EBC_DRIVER_VERSION_1.00"
#define EBC_DRIVER_SN_LEN sizeof(EBC_DRIVER_SN)

#define ebc_printk(dir_of_file,lev,fmt) ebc_dbg_printk(dir_of_file,lev,fmt)

/*
 * IMPORTANT: Those values is corresponding to android hardware program,
 * so *FORBID* to changes bellow values, unless you know what you're doing.
 * And if you want to add new refresh modes, please appended to the tail.
 */
enum epd_refresh_mode {
	EPD_AUTO	= 0,
	EPD_FULL	= 1,
	EPD_A2		= 2,
	EPD_PART	= 3,
	EPD_FULL_DITHER = 4,
	EPD_RESET	= 5,
	EPD_BLACK_WHITE = 6,
	EPD_TEXT	= 7,
	EPD_BLOCK	= 8,
	EPD_FULL_WIN	= 9,
	EPD_OED_PART	= 10,
	EPD_DIRECT_PART	= 11,
	EPD_DIRECT_A2	= 12,
};

#define EBC_OFF      (0)
#define EBC_ON        (1)
struct logo_info
{
	int logo_pic_offset;
	int logo_end_offset;
	int logo_power_pic_offset;
};

struct ebc_platform_data{
	int (*io_init)(void);
	int (*io_deinit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int (*vcom_power_on)(void);
	int (*vcom_power_off)(void);
	int (*suspend)(struct ebc_platform_data *ebc_data);
	int (*resume)(struct ebc_platform_data *ebc_data);
	const char *regulator;
};
struct ebc_pwr_ops
{
	int (*power_on)(void);
	int (*power_down)(void);
	int (*vcom_set)(int vcom_mv);
};
struct ebc_temperateure_ops
{
	int (*temperature_get)(int *temp);
};


/*android use struct*/
struct ebc_buf_info{
	int offset; 
	int epd_mode;
	int height;
	int width;
	int vir_height;
	int vir_width;
	int fb_width;
	int fb_height;
	int color_panel;
	int win_x1;
	int win_y1;
	int win_x2;
	int win_y2;
	int rotate;
}__packed;

// ebc sn
struct ebc_sn_info{
	u32 key;
	u32 sn_len;
	char cip_sn[EBC_DRIVER_SN_LEN];
};

struct rk29_ebc_info{
	int is_busy_now;
	char frame_total;
	char frame_bw_total;
	short int auto_need_refresh;
	int frame_left;
	int ebc_send_count;
	int ebc_mode;
	int pix_num_left;
	int height;
	int width;
	int *lut_addr;
	short int buffer_need_update;
	short int one_pix_end;
	int  buffer_need_check;
	int bits_per_pixel;
	int ebc_irq_status;
	int ebc_auto_work;
	int ebc_dsp_buf_status;// 1:have display buffer 0:no display buffer
	struct device *dev;
	struct fb_info *ebc_fb;
	struct epd_lut_ops ebc_lut_ops;
	struct ebc_pwr_ops ebc_pwr_ops;
	struct ebc_temperateure_ops ebc_temp_ops;
	struct ebc_ops *ebc_ops;
	struct epd_lut_data lut_data;
	struct task_struct *ebc_task;
	struct ebc_platform_data *mach_info;

	int    *auto_image_new;
	int    *auto_image_old;
	char   *auto_frame_buffer;
	int   	*auto_image_fb;
	void *auto_direct_buffer0;
	void *auto_direct_buffer1;

	int ebc_power_status;
	int ebc_last_display;
	char *lut_ddr_vir;
	struct ebc_buf_s  *prev_dsp_buf;
	struct ebc_buf_s  *curr_dsp_buf;

	struct wake_lock suspend_lock;
	int wake_lock_is_set;

	struct logo_info logo_info;
	int first_in;//first frame buf flag.
	/* timer to power off vdd */
	struct timer_list	vdd_timer;

	/* bootup animations timer. */
	struct timer_list	boot_logo_timer;

	/* timeout when start frame. */
	struct timer_list	frame_timer;
	struct work_struct work;
	struct ebc_buf_info buf_info;

	/*work*/
	struct work_struct	auto_buffer_work;
	struct work_struct	bootup_ani;
	struct workqueue_struct *bootup_ani_wq;
};

struct eink {
	struct device *dev;
	struct eink_tcon *tcon;
	struct eink_panel panel;

	struct rk29_ebc_info info;
};

/* modiy for open ebc ioctl */
extern int rkebc_register_notifier(struct notifier_block *nb);
extern int rkebc_unregister_notifier(struct notifier_block *nb);
extern int register_ebc_pwr_ops(struct ebc_pwr_ops *ops);
extern int register_ebc_temp_ops(struct ebc_temperateure_ops *ops);
#if 0 //CONFIG_BATTERY_RK30_ADC_FAC
extern int rk30_adc_battery_get_bat_vol(void);
extern void rk30_adc_ebc_battery_check(void);
#endif
extern int rkebc_notify(unsigned long event);
extern int rk29ebc_notify(unsigned long event);
extern long ebc_io_ctl(struct file *file, unsigned int cmd, unsigned long arg);	


/* public function */
extern int set_logo_info(struct logo_info *plogo_info);

/* hooks for customs */
void ebc_io_ctl_hook(unsigned int cmd, struct ebc_buf_info *info);

extern int get_waveform_type(void);
extern int get_waveform_position(void);
extern int set_end_display(void);
extern int get_bootup_logo_cycle(void);
extern int is_bootup_ani_loop(void);
extern int is_need_show_lowpower_pic(void);
extern int support_bootup_ani(void);
extern int support_double_thread_calcu(void);
extern int get_bootup_ani_mode(void);
extern int support_tps_3v3_always_alive(void);
extern int map_auto_mode(void);
extern int map_gray16_mode(void);

#endif
