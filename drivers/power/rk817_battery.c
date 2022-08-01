/*
 * rk817 battery  driver
 *
 * Copyright (C) 2018 Rockchip Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "rk817-bat: " fmt

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/rk_usbbc.h>
#include <linux/regmap.h>
#include <linux/rk_keys.h>

// 20201212: 如果我们直接用 rtc 的接口，会导致上层设置的 RTC_WAKE alarm 失效。
// --我们需要保留两种方式。因为 alarm-timer 的 RTC 可能会被关闭。
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/htfy_dbg.h>  // 20201214,hsl add for fb/alarm_timer.

#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#define BAT_DEBUGX       0

static int dbg_enable = BAT_DEBUGX;

module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)

#define BAT_INFO(fmt, args...) pr_info(fmt, ##args)

// 20201209: V1.0.1: support charging sleep (for FT).
// 20210414: V1.0.3: fix for remain_cap > fcc.
#define DRIVER_VERSION	"1.03"

//#define CHARGING_SLEEP		1


#define DIV(x)	((x) ? (x) : 1)
#define ENABLE	0x01
#define DISABLE	0x00
#define MAX_INTERPOLATE		1000
#define MAX_PERCENTAGE		100
#define MAX_INT			0x7FFF

/* RK818_GGCON */
#define OCV_SAMP_MIN_MSK	0x0c
#define OCV_SAMP_8MIN		(0x00 << 2)

#define ADC_CAL_8MIN		0x00
#define RELAX_VOL12_UPD_MSK	(RELAX_VOL1_UPD | RELAX_VOL2_UPD)
#define MINUTE(x)	\
	((x) * 60)

#define ADC_TO_CURRENT(adc_value, samp_res)	\
	(adc_value * 172 / 1000 / samp_res)
#define CURRENT_TO_ADC(current, samp_res)	\
	(current * 1000 * samp_res / 172)

#define ADC_TO_CAPACITY(adc_value, samp_res)	\
	(adc_value / 1000 * 172 / 3600 / samp_res)
#define CAPACITY_TO_ADC(capacity, samp_res)	\
	(capacity * samp_res * 3600 / 172 * 1000)

#define ADC_TO_CAPACITY_UAH(adc_value, samp_res)	\
	(adc_value / 3600 * 172 / samp_res)
#define ADC_TO_CAPACITY_MAH(adc_value, samp_res)	\
	(adc_value / 1000 * 172 / 3600 / samp_res)

/* THREAML_REG */
#define TEMP_85C		(0x00 << 2)
#define TEMP_95C		(0x01 << 2)
#define TEMP_105C		(0x02 << 2)
#define TEMP_115C		(0x03 << 2)

#define ZERO_LOAD_LVL1			1400
#define ZERO_LOAD_LVL2			600

/* zero algorithm */
//#define PWROFF_THRESD			3400
#define MIN_ZERO_DSOC_ACCURACY		10	/*0.01%*/
#define MIN_ZERO_OVERCNT		100
#define MIN_ACCURACY			1
#define DEF_PWRPATH_RES			50
#define WAIT_DSOC_DROP_SEC		15
#define WAIT_SHTD_DROP_SEC		30
#define MIN_ZERO_GAP_XSOC1		10
#define MIN_ZERO_GAP_XSOC2		5
#define MIN_ZERO_GAP_XSOC3		3
#define MIN_ZERO_GAP_CALIB		5

#define ADC_CALIB_THRESHOLD		4
#define ADC_CALIB_LMT_MIN		3
#define ADC_CALIB_CNT			5

/* default param */
#define DEFAULT_BAT_RES			    135     //电池内阻，毫欧
#define DEFAULT_SLP_ENTER_CUR		120
#define DEFAULT_SLP_EXIT_CUR		120
#define DEFAULT_SLP_FILTER_CUR		100
#define DEFAULT_PWROFF_VOL_THRESD	3400 //3500 //20210225,hsl fix.
#define DEFAULT_MONITOR_SEC		    7
#define DEFAULT_ALGR_VOL_THRESD1	3850
#define DEFAULT_ALGR_VOL_THRESD2	3950
#define DEFAULT_CHRG_VOL_SEL		CHRG_VOL4200MV
#define DEFAULT_CHRG_CUR_SEL		CHRG_CUR1400MA
#define DEFAULT_CHRG_CUR_INPUT		INPUT_CUR2000MA
#define DEFAULT_POFFSET			    42
#define DEFAULT_MAX_SOC_OFFSET		60
#define DEFAULT_FB_TEMP				TEMP_115C
#define DEFAULT_ENERGY_MODE			0
#define DEFAULT_ZERO_RESERVE_DSOC	10
#define DEFAULT_SAMPLE_RES		    20  //采样电阻，毫欧

/* sample resistor and division */
#define SAMPLE_RES_10MR			10
#define SAMPLE_RES_20MR			20
#define SAMPLE_RES_DIV1			1
#define SAMPLE_RES_DIV2			2

/* sleep */
/* 20211022: 如果是带4G 模块的项目，休眠的时候4G是不关闭的，休眠电流是 30ma左右，LOG：
[  182.175382] Suspended for 593.247 seconds
[  182.675195] rk817-bat: v_avg=3562,c_avg=-29,rsoc=1001,dsoc=1222,remain_cap=507228,use_fcc=2720,real_fcc=3200,chrg=0
如果是不带4G的项目，则应该是 2.5MA 左右。这里设置得稍微大一点问题不大。
*/
#ifdef CONFIG_LTE
#define SLP_CURR_MAX			30 
#else 
#define SLP_CURR_MAX			4  
#endif 

//#define SLP_CURR_MIN			2 // 6 --20200530,hsl fix for px30.
#define DISCHRG_TIME_STEP1		MINUTE(10)
#define DISCHRG_TIME_STEP2		MINUTE(30)
#define SLP_DSOC_VOL_THRESD		3600
//#define REBOOT_PERIOD_SEC		180
//#define REBOOT_MAX_CNT			80

// 20210225: 电量是0的时候，电池保留的FCC电量百分比。比如FCC=4000,
// 则 remain_cap = 800的时候，电量就是0.因为电池不能完全放电，此时的电压已经是3.6V左右了。
#define RESEVED_FCC_PERCENT     15

// 20211025: 当出现电池过放（由于电量调整导致remain_cap 错误），此时电量还比较高，但是电池电压
// 已经到达了关机电压(3.5V),此时电池里面剩余电量。
#define EMPTY_FCC_PERCENT     	12	// power-off volt=3500.

#define TIMER_MS_COUNTS		1000

/* fcc */
#define MIN_FCC				500
#define CAP_INVALID			0x80

/* virtual params */
#define VIRTUAL_CURRENT			1000
#define VIRTUAL_VOLTAGE			3888
#define VIRTUAL_SOC				66
#define VIRTUAL_PRESET			1
#define VIRTUAL_TEMPERATURE		188
#define VIRTUAL_STATUS			POWER_SUPPLY_STATUS_CHARGING

#define FINISH_CHRG_CUR1		1000
#define FINISH_CHRG_CUR2		1500
#define FINISH_MAX_SOC_DELAY		20
#define TERM_CHRG_DSOC			88
#define TERM_CHRG_CURR			600
#define TERM_CHRG_K			650
#define SIMULATE_CHRG_INTV		8
#define SIMULATE_CHRG_CURR		400
#define SIMULATE_CHRG_K			1500
#define FULL_CHRG_K			400

enum work_mode {
	MODE_ZERO = 0,
	MODE_FINISH,
	MODE_SMOOTH_CHRG,
	MODE_SMOOTH_DISCHRG,
	MODE_SMOOTH,
};

enum charge_status {
	CHRG_OFF,
	DEAD_CHRG,
	TRICKLE_CHRG,
	CC_OR_CV_CHRG,
	CHARGE_FINISH,
	USB_OVER_VOL,
	BAT_TMP_ERR,
	BAT_TIM_ERR,
};

enum bat_mode {
	MODE_BATTARY = 0,
	MODE_VIRTUAL,
};

enum rk817_sample_time {
	S_8_MIN,
	S_16_MIN,
	S_32_MIN,
	S_48_MIN,
};

enum rk817_output_mode {
	AVERAGE_MODE,
	INSTANT_MODE,
};


enum charge_current {
	CHRG_CUR_1000MA,
	CHRG_CUR_1500MA,
	CHRG_CUR_2000MA,
	CHRG_CUR_2500MA,
	CHRG_CUR_2750MA,
	CHRG_CUR_3000MA,
	CHRG_CUR_3500MA,
	CHRG_CUR_500MA,
};

enum charge_voltage {
	CHRG_VOL_4100MV,
	CHRG_VOL_4150MV,
	CHRG_VOL_4200MV,
	CHRG_VOL_4250MV,
	CHRG_VOL_4300MV,
	CHRG_VOL_4350MV,
	CHRG_VOL_4400MV,
	CHRG_VOL_4450MV,
};

enum rk817_battery_fields {
	ADC_SLP_RATE, BAT_CUR_ADC_EN, BAT_VOL_ADC_EN,
	USB_VOL_ADC_EN, TS_ADC_EN, SYS_VOL_ADC_EN, GG_EN, /*ADC_CONFIG0*/
	CUR_ADC_DITH_SEL, CUR_ADC_DIH_EN, CUR_ADC_CHOP_EN,
	CUR_ADC_CHOP_SEL, CUR_ADC_CHOP_VREF_EN, /*CUR_ADC_CFG0*/
	CUR_ADC_VCOM_SEL, CUR_ADC_VCOM_BUF_INC, CUR_ADC_VREF_BUF_INC,
	CUR_ADC_BIAS_DEC, CUR_ADC_IBIAS_SEL,/*CUR_ADC_CFG1*/
	VOL_ADC_EXT_VREF_EN, VOL_ADC_DITH_SEL, VOL_ADC_DITH_EN,
	VOL_ADC_CHOP_EN, VOL_ADC_CHOP_SEL, VOL_ADC_CHOP_VREF_EN,
	VOL_ADC_VCOM_SEL, VOL_ADC_VCOM_BUF_INC, VOL_ADC_VREF_BUF_INC,
	VOL_ADC_IBIAS_SEL, /*VOL_ADC_CFG1*/
	RLX_CUR_FILTER, TS_FUN, VOL_ADC_TSCUR_SEL,
	VOL_CALIB_UPD, CUR_CALIB_UPD, /*ADC_CONFIG1*/
	CUR_OUT_MOD, VOL_OUT_MOD, FRAME_SMP_INTERV,
	ADC_OFF_CAL_INTERV, RLX_SPT, /*GG_CON*/
	OCV_UPD, RELAX_STS, RELAX_VOL2_UPD, RELAX_VOL1_UPD, BAT_CON,
	QMAX_UPD_SOFT, TERM_UPD, OCV_STS, /*GG_STS*/
	RELAX_THRE_H, RELAX_THRE_L, /*RELAX_THRE*/
	RELAX_VOL1_H, RELAX_VOL1_L,
	RELAX_VOL2_H, RELAX_VOL2_L,
	RELAX_CUR1_H, RELAX_CUR1_L,
	RELAX_CUR2_H, RELAX_CUR2_L,
	OCV_THRE_VOL,
	OCV_VOL_H, OCV_VOL_L,
	OCV_VOL0_H, OCV_VOL0_L,
	OCV_CUR_H, OCV_CUR_L,
	OCV_CUR0_H, OCV_CUR0_L,
	PWRON_VOL_H, PWRON_VOL_L,
	PWRON_CUR_H, PWRON_CUR_L,
	OFF_CNT,
	Q_INIT_H3, Q_INIT_H2, Q_INIT_L1, Q_INIT_L0,
	Q_PRESS_H3, Q_PRESS_H2, Q_PRESS_L1, Q_PRESS_L0,
	BAT_VOL_H, BAT_VOL_L,
	BAT_CUR_H, BAT_CUR_L,
	BAT_TS_H, BAT_TS_L,
	USB_VOL_H, USB_VOL_L,
	SYS_VOL_H, SYS_VOL_L,
	Q_MAX_H3, Q_MAX_H2, Q_MAX_L1, Q_MAX_L0,
	Q_TERM_H3, Q_TERM_H2, Q_TERM_L1, Q_TERM_L0,
	Q_OCV_H3, Q_OCV_H2, Q_OCV_L1, Q_OCV_L0,
	OCV_CNT,
	SLEEP_CON_SAMP_CUR_H, SLEEP_CON_SAMP_CUR_L,
	CAL_OFFSET_H, CAL_OFFSET_L,
	VCALIB0_H, VCALIB0_L,
	VCALIB1_H, VCALIB1_L,
	IOFFSET_H, IOFFSET_L,
	BAT_R0, SOC_REG0, SOC_REG1, SOC_REG2,
	REMAIN_CAP_REG2, REMAIN_CAP_REG1, REMAIN_CAP_REG0,
	NEW_FCC_REG2, NEW_FCC_REG1, NEW_FCC_REG0,
	RESET_MODE,
	FG_INIT, HALT_CNT_REG, CALC_REST_REGL, CALC_REST_REGH,
	VOL_ADC_B3,  VOL_ADC_B2, VOL_ADC_B1, VOL_ADC_B0,
	VOL_ADC_K3, VOL_ADC_K2, VOL_ADC_K1, VOL_ADC_K0,
	BAT_EXS, CHG_STS, BAT_OVP_STS, CHRG_IN_CLAMP,
	CHIP_NAME_H, CHIP_NAME_L,
	PLUG_IN_STS, CHRG_EN, CHRG_CUR_SEL, CHRG_VOL_SEL, BAT_LTS_TS,
	F_MAX_FIELDS
};

static const struct reg_field rk817_battery_reg_fields[] = {
	[ADC_SLP_RATE] = REG_FIELD(0x50, 0, 0),
	[BAT_CUR_ADC_EN] = REG_FIELD(0x50, 2, 2),
	[BAT_VOL_ADC_EN] = REG_FIELD(0x50, 3, 3),
	[USB_VOL_ADC_EN] = REG_FIELD(0x50, 4, 4),
	[TS_ADC_EN] = REG_FIELD(0x50, 5, 5),
	[SYS_VOL_ADC_EN] = REG_FIELD(0x50, 6, 6),
	[GG_EN] = REG_FIELD(0x50, 7, 7),/*ADC_CONFIG0*/

	[CUR_ADC_DITH_SEL] = REG_FIELD(0x51, 1, 3),
	[CUR_ADC_DIH_EN] = REG_FIELD(0x51, 4, 4),
	[CUR_ADC_CHOP_EN] = REG_FIELD(0x51, 5, 5),
	[CUR_ADC_CHOP_SEL] = REG_FIELD(0x51, 6, 6),
	[CUR_ADC_CHOP_VREF_EN] = REG_FIELD(0x51, 7, 7), /*CUR_ADC_COFG0*/

	[CUR_ADC_VCOM_SEL] = REG_FIELD(0x52, 0, 1),
	[CUR_ADC_VCOM_BUF_INC] = REG_FIELD(0x52, 2, 2),
	[CUR_ADC_VREF_BUF_INC] = REG_FIELD(0x52, 3, 3),
	[CUR_ADC_BIAS_DEC] = REG_FIELD(0x52, 4, 4),
	[CUR_ADC_IBIAS_SEL] = REG_FIELD(0x52, 5, 6), /*CUR_ADC_COFG1*/

	[VOL_ADC_EXT_VREF_EN] = REG_FIELD(0x53, 0, 0),
	[VOL_ADC_DITH_SEL]  = REG_FIELD(0x53, 1, 3),
	[VOL_ADC_DITH_EN] = REG_FIELD(0x53, 4, 4),
	[VOL_ADC_CHOP_EN] = REG_FIELD(0x53, 5, 5),
	[VOL_ADC_CHOP_SEL] = REG_FIELD(0x53, 6, 6),
	[VOL_ADC_CHOP_VREF_EN] = REG_FIELD(0x53, 7, 7),/*VOL_ADC_COFG0*/

	[VOL_ADC_VCOM_SEL] = REG_FIELD(0x54, 0, 1),
	[VOL_ADC_VCOM_BUF_INC] = REG_FIELD(0x54, 2, 2),
	[VOL_ADC_VREF_BUF_INC] = REG_FIELD(0x54, 3, 3),
	[VOL_ADC_IBIAS_SEL] = REG_FIELD(0x54, 5, 6), /*VOL_ADC_COFG1*/

	[RLX_CUR_FILTER] = REG_FIELD(0x55, 0, 1),
	[TS_FUN] = REG_FIELD(0x55, 3, 3),
	[VOL_ADC_TSCUR_SEL] = REG_FIELD(0x55, 4, 5),
	[VOL_CALIB_UPD] = REG_FIELD(0x55, 6, 6),
	[CUR_CALIB_UPD] = REG_FIELD(0x55, 7, 7), /*ADC_CONFIG1*/

	[CUR_OUT_MOD] = REG_FIELD(0x56, 0, 0),
	[VOL_OUT_MOD] = REG_FIELD(0x56, 1, 1),
	[FRAME_SMP_INTERV] = REG_FIELD(0x56, 2, 3),
	[ADC_OFF_CAL_INTERV] = REG_FIELD(0x56, 4, 5),
	[RLX_SPT] = REG_FIELD(0x56, 6, 7), /*GG_CON*/

	[OCV_UPD] = REG_FIELD(0x57, 0, 0),
	[RELAX_STS] = REG_FIELD(0x57, 1, 1),
	[RELAX_VOL2_UPD] = REG_FIELD(0x57, 2, 2),
	[RELAX_VOL1_UPD] = REG_FIELD(0x57, 3, 3),
	[BAT_CON] = REG_FIELD(0x57, 4, 4),
	[QMAX_UPD_SOFT] = REG_FIELD(0x57, 5, 5),
	[TERM_UPD] = REG_FIELD(0x57, 6, 6),
	[OCV_STS] = REG_FIELD(0x57, 7, 7), /*GG_STS*/

	[RELAX_THRE_H] = REG_FIELD(0x58, 0, 7),
	[RELAX_THRE_L] = REG_FIELD(0x59, 0, 7),

	[RELAX_VOL1_H] = REG_FIELD(0x5A, 0, 7),
	[RELAX_VOL1_L] = REG_FIELD(0x5B, 0, 7),
	[RELAX_VOL2_H] = REG_FIELD(0x5C, 0, 7),
	[RELAX_VOL2_L] = REG_FIELD(0x5D, 0, 7),

	[RELAX_CUR1_H] = REG_FIELD(0x5E, 0, 7),
	[RELAX_CUR1_L] = REG_FIELD(0x5F, 0, 7),
	[RELAX_CUR2_H] = REG_FIELD(0x60, 0, 7),
	[RELAX_CUR2_L] = REG_FIELD(0x61, 0, 7),

	[OCV_THRE_VOL] = REG_FIELD(0x62, 0, 7),

	[OCV_VOL_H] = REG_FIELD(0x63, 0, 7),
	[OCV_VOL_L] = REG_FIELD(0x64, 0, 7),
	[OCV_VOL0_H] = REG_FIELD(0x65, 0, 7),
	[OCV_VOL0_L] = REG_FIELD(0x66, 0, 7),
	[OCV_CUR_H] = REG_FIELD(0x67, 0, 7),
	[OCV_CUR_L] = REG_FIELD(0x68, 0, 7),
	[OCV_CUR0_H] = REG_FIELD(0x69, 0, 7),
	[OCV_CUR0_L] = REG_FIELD(0x6A, 0, 7),
	[PWRON_VOL_H] = REG_FIELD(0x6B, 0, 7),
	[PWRON_VOL_L] = REG_FIELD(0x6C, 0, 7),
	[PWRON_CUR_H] = REG_FIELD(0x6D, 0, 7),
	[PWRON_CUR_L] = REG_FIELD(0x6E, 0, 7),
	[OFF_CNT] = REG_FIELD(0x6F, 0, 7),
	[Q_INIT_H3] = REG_FIELD(0x70, 0, 7),
	[Q_INIT_H2] = REG_FIELD(0x71, 0, 7),
	[Q_INIT_L1] = REG_FIELD(0x72, 0, 7),
	[Q_INIT_L0] = REG_FIELD(0x73, 0, 7),

	[Q_PRESS_H3] = REG_FIELD(0x74, 0, 7),
	[Q_PRESS_H2] = REG_FIELD(0x75, 0, 7),
	[Q_PRESS_L1] = REG_FIELD(0x76, 0, 7),
	[Q_PRESS_L0] = REG_FIELD(0x77, 0, 7),

	[BAT_VOL_H] = REG_FIELD(0x78, 0, 7),
	[BAT_VOL_L] = REG_FIELD(0x79, 0, 7),

	[BAT_CUR_H] = REG_FIELD(0x7A, 0, 7),
	[BAT_CUR_L] = REG_FIELD(0x7B, 0, 7),

	[BAT_TS_H] = REG_FIELD(0x7C, 0, 7),
	[BAT_TS_L] = REG_FIELD(0x7D, 0, 7),
	[USB_VOL_H] = REG_FIELD(0x7E, 0, 7),
	[USB_VOL_L] = REG_FIELD(0x7F, 0, 7),

	[SYS_VOL_H] = REG_FIELD(0x80, 0, 7),
	[SYS_VOL_L] = REG_FIELD(0x81, 0, 7),
	[Q_MAX_H3] = REG_FIELD(0x82, 0, 7),
	[Q_MAX_H2] = REG_FIELD(0x83, 0, 7),
	[Q_MAX_L1] = REG_FIELD(0x84, 0, 7),
	[Q_MAX_L0] = REG_FIELD(0x85, 0, 7),

	[Q_TERM_H3] = REG_FIELD(0x86, 0, 7),
	[Q_TERM_H2] = REG_FIELD(0x87, 0, 7),
	[Q_TERM_L1] = REG_FIELD(0x88, 0, 7),
	[Q_TERM_L0] = REG_FIELD(0x89, 0, 7),
	[Q_OCV_H3] = REG_FIELD(0x8A, 0, 7),
	[Q_OCV_H2] = REG_FIELD(0x8B, 0, 7),

	[Q_OCV_L1] = REG_FIELD(0x8C, 0, 7),
	[Q_OCV_L0] = REG_FIELD(0x8D, 0, 7),
	[OCV_CNT] = REG_FIELD(0x8E, 0, 7),
	[SLEEP_CON_SAMP_CUR_H] = REG_FIELD(0x8F, 0, 7),
	[SLEEP_CON_SAMP_CUR_L] = REG_FIELD(0x90, 0, 7),
	[CAL_OFFSET_H] = REG_FIELD(0x91, 0, 7),
	[CAL_OFFSET_L] = REG_FIELD(0x92, 0, 7),
	[VCALIB0_H] = REG_FIELD(0x93, 0, 7),
	[VCALIB0_L] = REG_FIELD(0x94, 0, 7),
	[VCALIB1_H] = REG_FIELD(0x95, 0, 7),
	[VCALIB1_L] = REG_FIELD(0x96, 0, 7),
	[IOFFSET_H] = REG_FIELD(0x97, 0, 7),
	[IOFFSET_L] = REG_FIELD(0x98, 0, 7),

	[BAT_R0] = REG_FIELD(0x99, 0, 7),
	[SOC_REG0] = REG_FIELD(0x9A, 0, 7),
	[SOC_REG1] = REG_FIELD(0x9B, 0, 7),
	[SOC_REG2] = REG_FIELD(0x9C, 0, 7),

	[REMAIN_CAP_REG0] = REG_FIELD(0x9D, 0, 7),
	[REMAIN_CAP_REG1] = REG_FIELD(0x9E, 0, 7),
	[REMAIN_CAP_REG2] = REG_FIELD(0x9F, 0, 7),
	[NEW_FCC_REG0] = REG_FIELD(0xA0, 0, 7),
	[NEW_FCC_REG1] = REG_FIELD(0xA1, 0, 7),
	[NEW_FCC_REG2] = REG_FIELD(0xA2, 0, 7),
	[RESET_MODE] = REG_FIELD(0xA3, 0, 3),
	[FG_INIT] = REG_FIELD(0xA5, 7, 7),

	[HALT_CNT_REG] = REG_FIELD(0xA6, 0, 7),
	[CALC_REST_REGL] = REG_FIELD(0xA7, 0, 7),
	[CALC_REST_REGH] = REG_FIELD(0xA8, 0, 7),

	[VOL_ADC_B3] = REG_FIELD(0xA9, 0, 7),
	[VOL_ADC_B2] = REG_FIELD(0xAA, 0, 7),
	[VOL_ADC_B1] = REG_FIELD(0xAB, 0, 7),
	[VOL_ADC_B0] = REG_FIELD(0xAC, 0, 7),

	[VOL_ADC_K3] = REG_FIELD(0xAD, 0, 7),
	[VOL_ADC_K2] = REG_FIELD(0xAE, 0, 7),
	[VOL_ADC_K1] = REG_FIELD(0xAF, 0, 7),
	[VOL_ADC_K0] = REG_FIELD(0xB0, 0, 7),
	[BAT_EXS] = REG_FIELD(0xEB, 7, 7),
	[CHG_STS] = REG_FIELD(0xEB, 4, 6),
	[BAT_OVP_STS] = REG_FIELD(0xEB, 3, 3),
	[CHRG_IN_CLAMP] = REG_FIELD(0xEB, 2, 2),
	[CHIP_NAME_H] = REG_FIELD(0xED, 0, 7),
	[CHIP_NAME_L] = REG_FIELD(0xEE, 0, 7),
	[PLUG_IN_STS] = REG_FIELD(0xF0, 6, 6),
	[CHRG_CUR_SEL] = REG_FIELD(0xE4, 0, 2),
	[CHRG_VOL_SEL] = REG_FIELD(0xE4, 4, 6),
	[BAT_LTS_TS] = REG_FIELD(0xE9, 0, 7),
};

struct battery_platform_data {
	u32 *ocv_table;
	u32 *zero_table;

	u32 table_t[4][21];
	int temp_t[4];
	u32 temp_t_num;

	u32 *ntc_table;
	u32 ocv_size;
	u32 ntc_size;
	int ntc_degree_from;
	u32 ntc_factor;
	u32 max_input_current;
	u32 max_chrg_current;
	u32 max_chrg_voltage;
	u32 lp_input_current;
	u32 lp_soc_min;
	u32 lp_soc_max;
	u32 pwroff_vol;
	u32 monitor_sec;
	u32 zero_algorithm_vol;
	u32 zero_reserve_dsoc;
	u32 bat_res;
	u32 design_capacity;
	u32 design_qmax;
	u32 sleep_enter_current;
	u32 sleep_exit_current;
	u32 sleep_filter_current;

	u32 power_dc2otg;
	u32 max_soc_offset;
	u32 bat_mode;
	u32 fb_temp;
	u32 energy_mode;
	u32 cccv_hour;
	u32 dc_det_adc;
	int dc_det_pin;
	u8  dc_det_level;
	u32 sample_res;
	u32 bat_res_up;
	u32 bat_res_down;
	u32 design_max_voltage;
	bool extcon;
};

struct rk817_battery_device {
	struct platform_device		*pdev;
	struct device				*dev;
	struct i2c_client			*client;
	struct rk808			*rk817;
	struct power_supply			*bat;
	struct power_supply		*chg_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*ac_psy;
	struct regmap			*regmap;
	struct regmap_field		*rmap_fields[F_MAX_FIELDS];
	struct battery_platform_data	*pdata;
	struct workqueue_struct		*bat_monitor_wq;
	struct delayed_work		bat_delay_work;
	struct delayed_work		calib_delay_work;
	struct wake_lock		wake_lock;
	struct timer_list		caltimer;

	int				res_div;
	int				bat_res;
	bool			is_first_power_on;
	int				chrg_status;
	int 			low_current_chg_count;
	int 			finish_chg_count;
	int 			low_cap_chg_count;
	int				res_fac;
	int				over_20mR;
	bool			is_initialized;
	bool			bat_first_power_on;
	u8				ac_in;
	u8				usb_in;
	u8				otg_in;
	u8				dc_in;
	u8				prop_status;
	int				cvtlmt_irq;
	int				current_avg;
	int				current_relax;
	int				voltage_usb;
	int				voltage_sys;
	int				voltage_avg;
	int				voltage_ocv;
	int				voltage_relax;
	int				voltage_k;/* VCALIB0 VCALIB1 */
	int				voltage_b;
	int				remain_cap;	// 单位是 uah.
	int				design_cap;
	int				nac;

	// 20210414: 下面几个 fcc 的单位都是 mah. 不需要 *1000.
	int				fcc;
	int				reserved_fcc; 
	int				max_used_fcc;   
	int             real_fcc; // 20210414: 实际电池会出现充电后容量大于 FCC 的情况。我们需要处理这种情况下的 电量显示。
	int				qmax;   // mah
	
	int				dsoc;   // 百分比*1000,比如 90%= 90*1000.
	int				rsoc;
	int				poffset;
	int				fake_offline;
	int				age_ocv_soc;
	bool			age_allow_update;
	int				age_level;
	int				age_ocv_cap;
	int				pwron_voltage;
	int				age_voltage;
	int				age_adjust_cap;
	unsigned long	age_keep_sec;
	int				zero_timeout_cnt;
	int				zero_remain_cap;
	int				zero_dsoc;
	int				zero_linek;
	u64				zero_drop_sec;
	u64				shtd_drop_sec;

	int				powerpatch_res;
	int				zero_voltage_avg;
	int				zero_current_avg;
	int				zero_vsys;
	int				zero_dead_voltage;
	int				zero_dead_soc;
	int				zero_dead_cap;
	int				zero_batvol_to_ocv;
	int				zero_batocv_to_soc;
	int				zero_batocv_to_cap;
	int				zero_xsoc;
	unsigned long	finish_base;
	time_t			rtc_base;
	int				sm_remain_cap;
	int				sm_linek;
	int				sm_chrg_dsoc;
	int				sm_dischrg_dsoc;
	int				smooth_soc;
	int				algo_rest_val;
	int				algo_rest_mode;
	int				sleep_sum_cap;
	int				sleep_remain_cap;
	unsigned long	sleep_dischrg_sec;
	unsigned long	sleep_sum_sec;
	bool			sleep_chrg_online;
	u8				sleep_chrg_status;
	bool			adc_allow_update;
	int             fb_blank;
	bool			s2r; /*suspend to resume --表示是否可以大范围的更新电量百分比。*/
	u32				work_mode;
	int				temperature;
	int				chrg_cur_lp_input;
	int				chrg_vol_sel;
	int				chrg_cur_input;
	int				chrg_cur_sel;
	u32				monitor_ms;
	u32				pwroff_min;
	u32				adc_calib_cnt;
	unsigned long	chrg_finish_base;
	unsigned long	boot_base;
	unsigned long	flat_match_sec;
	unsigned long	plug_in_base;
	unsigned long	plug_out_base;
	u8				halt_cnt;
	bool			is_halt;
	bool			is_max_soc_offset;
	bool			is_sw_reset;
	bool			is_ocv_calib;
	bool			is_first_on;
	bool			is_force_calib;
	//int				last_dsoc;
	u8				cvtlmt_int_event;
	u8				slp_dcdc_en_reg;
	int				ocv_pre_dsoc;
	int				ocv_new_dsoc;
	int				max_pre_dsoc;
	int				max_new_dsoc;
	int				force_pre_dsoc;
	int				force_new_dsoc;

	int				dbg_cap_low0;
	int				dbg_pwr_dsoc;
	int				dbg_pwr_rsoc;
	int				dbg_pwr_vol;
	int				dbg_chrg_min[10];
	int				dbg_meet_soc;
	int				dbg_calc_dsoc;
	int				dbg_calc_rsoc;
	int				is_charging;
	unsigned long	charge_count;
	u8				plugin_trigger;
	u8				plugout_trigger;
	int				plugin_irq;
	int				plugout_irq;
	int				chip_id;
	int				is_register_chg_psy;


	// 20191229,HSL,add led ctrl
	int     		ledr_gpio;
	int     		ledr_on_level;

	int     		ledg_gpio;
	int     		ledg_on_level;

    struct delayed_work		led_work;

#ifdef CONFIG_LEDS_AW2013
// 20210727,tanlq add.aw2013支持三种颜色的灯光控制.控制规则： < [0]: 显示红色，[0]~[1]:黄色
// [1]~[2]: 显示绿色，>[2] 闪绿色 
// <[3]:闪红色（使用中）
	u32 	tri_led_table[4]; 
#else
// 20210412,hsl add.我们支持三种颜色的灯光控制.控制规则： < [0]: 显示红色，
// > [1]: 显示绿色， 其他显示橙色(红和绿同时亮)。默认 [0]=98, [1]=97,只有红和绿。
	u32     tri_led_table[2]; 
#endif
	int 	notify_dsoc;
	int 	notify_charging;

	bool 	charging_sleep; // true: 允许充电进入休眠(用于FT); false: 充电时不休眠。
	bool 	cust_ctrl_led;	// 是否有上层应用控制LED,主要用于 FT测试。
	bool 	resume_adjust;  // true: 系统休眠唤醒了，需要在 work 里面进行 charge_adjust。
	bool    force_uncharge; // 20210415: 是否需要上报非充电状态来触发Android关机防止电池过放。
};

// static
u64 get_boot_sec(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return ts.tv_sec;
}

static u32 interpolate(int value, u32 *table, int size)
{
	u8 i;
	u16 d;

	for (i = 0; i < size; i++) {
		if (value < table[i])
			break;
	}

	if ((i > 0) && (i < size)) {
		d = (value - table[i - 1]) * (MAX_INTERPOLATE / (size - 1));
		d /= table[i] - table[i - 1];
		d = d + (i - 1) * (MAX_INTERPOLATE / (size - 1));
	} else {
		d = i * ((MAX_INTERPOLATE + size / 2) / size);
	}

	if (d > 1000)
		d = 1000;

	return d;
}

/* (a * b) / c */
static int32_t ab_div_c(u32 a, u32 b, u32 c)
{
	bool sign;
	u32 ans = MAX_INT;
	int tmp;

	sign = ((((a ^ b) ^ c) & 0x80000000) != 0);
	if (c != 0) {
		if (sign)
			c = -c;
		tmp = (a * b + (c >> 1)) / c;
		if (tmp < MAX_INT)
			ans = tmp;
	}

	if (sign)
		ans = -ans;

	return ans;
}

static int rk817_bat_field_read(struct rk817_battery_device *battery,
				enum rk817_battery_fields field_id)
{
	int val;
	int ret;

	ret = regmap_field_read(battery->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int rk817_bat_field_write(struct rk817_battery_device *battery,
				 enum rk817_battery_fields field_id,
				 unsigned int val)
{
	return regmap_field_write(battery->rmap_fields[field_id], val);
}

/*cal_offset: current offset value*/
static int rk817_bat_get_coffset(struct rk817_battery_device *battery)
{
	int  coffset_value = 0;

	coffset_value |= rk817_bat_field_read(battery, CAL_OFFSET_H) << 8;
	coffset_value |= rk817_bat_field_read(battery, CAL_OFFSET_L);

	return coffset_value;
}

static void rk817_bat_set_coffset(struct rk817_battery_device *battery, int val)
{
	u8  buf = 0;

	buf = (val >> 8) & 0xff;
	rk817_bat_field_write(battery, CAL_OFFSET_H, buf);
	buf = (val >> 0) & 0xff;
	rk817_bat_field_write(battery, CAL_OFFSET_L, buf);
}

/* current offset value calculated */
static int rk817_bat_get_ioffset(struct rk817_battery_device *battery)
{
	int  ioffset_value = 0;

	ioffset_value |= rk817_bat_field_read(battery, IOFFSET_H) << 8;
	ioffset_value |= rk817_bat_field_read(battery, IOFFSET_L);

	return ioffset_value;
}

static int rk817_bat_get_temp(struct rk817_battery_device *battery)
{
	int  temp_value = 0,temp_value1=0;
	int adc_to_vol;

	temp_value |= rk817_bat_field_read(battery, BAT_TS_H) << 8;
	temp_value |= rk817_bat_field_read(battery, BAT_TS_L);
	temp_value1 |= rk817_bat_field_read(battery, BAT_LTS_TS);
	adc_to_vol = temp_value * 1200 / 65536;
	printk("rk817_bat_get_temp:adc=%d adcv=%d templimit:0x%2x\n",temp_value,adc_to_vol,temp_value1);
	//if(battery->current_avg>0){
	//	adc_to_vol -= 7; //
	//}
	//if(battery->current_avg >0){//103充电时adc会比不插充电器小,78插充电器adc会升高
		
	//}else{
	//	adc_to_vol+=10;
	//}
	if(adc_to_vol > battery->pdata->temp_t[0]){ // <0
		return 0;
	}
	if(adc_to_vol > battery->pdata->temp_t[0]-30){ // 0-3
		return 3;
	}
	if(adc_to_vol > battery->pdata->temp_t[1]+10){ // 3-13
		return 13;
	}
	if(adc_to_vol > battery->pdata->temp_t[1]){ // 13-15
		return 15;
	}
	if(adc_to_vol > battery->pdata->temp_t[1]-10){ // 15-17// adc 55会自动不充电
		return 17;
	}
	if(adc_to_vol > battery->pdata->temp_t[2]+5){ // 17-43
		return 43;
	}
	if(adc_to_vol > battery->pdata->temp_t[2]){ // 43-45
		return 45;
	}
	if(adc_to_vol > battery->pdata->temp_t[2]-5){ // 45-47
		return 47;
	}
	if(adc_to_vol > battery->pdata->temp_t[3]+5){ // 47-55
		return 55;
	}
	if(adc_to_vol > battery->pdata->temp_t[3]){ // 55-60
		return 59;
	}
	else{ //60
		return 60;
	}
//printk("rk817_bat_get_temp:%d \n",temp_value);
	return adc_to_vol;
}

#if 0
static int rk817_bat_update_temperature(struct rk817_battery_device *battery)
{
	u32 ntc_size, *ntc_table;
	int i, res;

	ntc_table = battery->pdata->ntc_table;
	ntc_size = battery->pdata->ntc_size;

	if (ntc_size) {
		res = rk817_bat_get_nts_res(battery);
		if (res == 0)
			return 0;

		if (res < ntc_table[ntc_size - 1]) {
			battery->temperature = (ntc_size + battery->pdata->ntc_degree_from) * 10;
			rkbat_dbg(battery->dev,
				"bat ntc upper max degree: R=%d\n",
				res);
		} else if (res > ntc_table[0]) {
			battery->temperature = battery->pdata->ntc_degree_from * 10;
			rkbat_dbg(battery->dev,
				"bat ntc lower min degree: R=%d\n",
				res);
		} else {
			for (i = 0; i < ntc_size; i++) {
				if (res >= ntc_table[i])
					break;
			}

			if (i <= 0)
				battery->temperature =
				    (battery->pdata->ntc_degree_from) * 10;
			else
				battery->temperature =
				    (i + battery->pdata->ntc_degree_from) * 10;
		}
		rk817_bat_temperature_chrg(battery, battery->temperature / 10);
	}

	return 0;
}
#endif
static void rk817_bat_current_calibration(struct rk817_battery_device *battery)
{
	int pwron_value, ioffset, cal_offset;

	pwron_value = rk817_bat_field_read(battery, PWRON_CUR_H) << 8;
	pwron_value |= rk817_bat_field_read(battery, PWRON_CUR_L);

	ioffset = rk817_bat_get_ioffset(battery);

    //20210221-LOG: Caloffset: 0x80ea / IOFFSET: 0x80ea / Caloffset: 0x80ea
	//DBG("Caloffset: 0x%x\n", rk817_bat_get_coffset(battery));
	//DBG("IOFFSET: 0x%x\n", ioffset);
	if (0)
		cal_offset = pwron_value + ioffset;
	else
		cal_offset = ioffset;

	rk817_bat_set_coffset(battery, cal_offset);
	DBG("Caloffset: 0x%x\n", rk817_bat_get_coffset(battery));

}

static int rk817_bat_get_vaclib0(struct rk817_battery_device *battery)
{
	int vcalib_value = 0;

	vcalib_value |= rk817_bat_field_read(battery, VCALIB0_H) << 8;
	vcalib_value |= rk817_bat_field_read(battery, VCALIB0_L);

	return vcalib_value;
}

static int rk817_bat_get_vaclib1(struct rk817_battery_device *battery)
{
	int vcalib_value = 0;

	vcalib_value |= rk817_bat_field_read(battery, VCALIB1_H) << 8;
	vcalib_value |= rk817_bat_field_read(battery, VCALIB1_L);

	return vcalib_value;
}

static void rk817_bat_init_voltage_kb(struct rk817_battery_device *battery)
{
	int vcalib0, vcalib1;

	vcalib0 = rk817_bat_get_vaclib0(battery);
	vcalib1 =  rk817_bat_get_vaclib1(battery);
	if (battery->chip_id == RK809_ID) {
		battery->voltage_k = (1050 - 600) * 1000 / DIV(vcalib1 - vcalib0);
		battery->voltage_b = 1050 - (battery->voltage_k * vcalib1) / 1000;
	} else {
		battery->voltage_k = (4025 - 2300) * 1000 / DIV(vcalib1 - vcalib0);
		battery->voltage_b = 4025 - (battery->voltage_k * vcalib1) / 1000;
	}
}

static void rk817_bat_restart_relax(struct rk817_battery_device *battery)
{
	rk817_bat_field_write(battery, RELAX_VOL1_UPD, 0x00);
	rk817_bat_field_write(battery, RELAX_VOL2_UPD, 0x00);
}

static bool is_rk817_bat_relax_mode(struct rk817_battery_device *battery)
{
	u8 relax_sts, relax_vol1_upd, relax_vol2_upd;

	relax_sts = rk817_bat_field_read(battery, RELAX_STS);
	relax_vol1_upd = rk817_bat_field_read(battery, RELAX_VOL1_UPD);
	relax_vol2_upd = rk817_bat_field_read(battery, RELAX_VOL2_UPD);

    // 20210221-LOG: RELAX_STS: 1 / RELAX_VOL1_UPD: 0 / RELAX_VOL2_UPD: 0 
	//DBG("RELAX_STS: %d\n", relax_sts);
	//DBG("RELAX_VOL1_UPD: %d\n", relax_vol1_upd);
	//DBG("RELAX_VOL2_UPD: %d\n", relax_vol2_upd);
	if (relax_sts && relax_vol1_upd && relax_vol2_upd)
		return true;
	else
		return false;
}

static u16 rk817_bat_get_relax_vol1(struct rk817_battery_device *battery)
{
	u16 vol, val = 0;

	val = rk817_bat_field_read(battery, RELAX_VOL1_H) << 8;
	val |= rk817_bat_field_read(battery, RELAX_VOL1_L);
	vol = battery->voltage_k * val / 1000 + battery->voltage_b;

	return vol;
}

static u16 rk817_bat_get_relax_vol2(struct rk817_battery_device *battery)
{
	u16 vol, val = 0;

	val = rk817_bat_field_read(battery, RELAX_VOL2_H) << 8;
	val |= rk817_bat_field_read(battery, RELAX_VOL2_L);
	vol = battery->voltage_k * val / 1000 + battery->voltage_b;

	return vol;
}

static u16 rk817_bat_get_relax_voltage(struct rk817_battery_device *battery)
{
	u16 relax_vol1, relax_vol2;

	if (!is_rk817_bat_relax_mode(battery))
		return 0;

	relax_vol1 = rk817_bat_get_relax_vol1(battery);
	relax_vol2 = rk817_bat_get_relax_vol2(battery);

	return relax_vol1 > relax_vol2 ? relax_vol1 : relax_vol2;
}

static void rk817_bat_set_relax_sample(struct rk817_battery_device *battery)
{
	u8 buf;
	int enter_thres, filter_thres;
	struct battery_platform_data *pdata = battery->pdata;

	filter_thres = pdata->sleep_filter_current * 1000 / 1506;

	enter_thres = CURRENT_TO_ADC(pdata->sleep_enter_current,
				     battery->res_div);
	filter_thres = CURRENT_TO_ADC(pdata->sleep_filter_current,
				      battery->res_div);

	/* set relax enter and exit threshold */
	buf = (enter_thres >> 8) & 0xff;
	rk817_bat_field_write(battery, RELAX_THRE_H, buf);
	buf = enter_thres & 0xff;
	rk817_bat_field_write(battery, RELAX_THRE_L, buf);
	/* set sample current threshold */
	buf = (filter_thres >> 8) & 0xff;
	rk817_bat_field_write(battery, SLEEP_CON_SAMP_CUR_H, buf);
	buf = filter_thres & 0xff;
	rk817_bat_field_write(battery, SLEEP_CON_SAMP_CUR_L, buf);

	/* reset relax update state */
	rk817_bat_restart_relax(battery);
	DBG("<%s>. sleep_enter_current = %d, sleep_exit_current = %d\n",
	    __func__, pdata->sleep_enter_current, pdata->sleep_exit_current);
}

/* runtime OCV voltage,  |RLX_VOL2 - RLX_VOL1| < OCV_THRE,
 * the OCV reg update every 120s
 */
static void rk817_bat_ocv_thre(struct rk817_battery_device *battery, int value)
{
	rk817_bat_field_write(battery, OCV_THRE_VOL, value);
}

static int rk817_bat_get_ocv_voltage(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;

	val = rk817_bat_field_read(battery, OCV_VOL_H) << 8;
	val |= rk817_bat_field_read(battery, OCV_VOL_L);
	vol = battery->voltage_k * val / 1000 + battery->voltage_b;

	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

static int rk817_bat_get_ocv0_voltage0(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;

	val = rk817_bat_field_read(battery, OCV_VOL0_H) << 8;
	val |= rk817_bat_field_read(battery, OCV_VOL0_L);
	vol = battery->voltage_k * val / 1000 + battery->voltage_b;
	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

/* power on battery voltage */
static int rk817_bat_get_pwron_voltage(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;

	val = rk817_bat_field_read(battery, PWRON_VOL_H) << 8;
	val |= rk817_bat_field_read(battery, PWRON_VOL_L);
	vol = battery->voltage_k * val / 1000 + battery->voltage_b;
	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

static int rk817_bat_get_battery_voltage(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;
	int vcalib0, vcalib1;

	vcalib0 = rk817_bat_get_vaclib0(battery);
	vcalib1 =  rk817_bat_get_vaclib1(battery);


	val = rk817_bat_field_read(battery, BAT_VOL_H) << 8;
	val |= rk817_bat_field_read(battery, BAT_VOL_L) << 0;

	vol = battery->voltage_k * val / 1000 + battery->voltage_b;

	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

static int rk817_bat_get_USB_voltage(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;

	rk817_bat_field_write(battery, USB_VOL_ADC_EN, 0x01);

	val = rk817_bat_field_read(battery, USB_VOL_H) << 8;
	val |= rk817_bat_field_read(battery, USB_VOL_L) << 0;

	vol = (battery->voltage_k * val / 1000 + battery->voltage_b) * 60 / 46;

	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

static int rk817_bat_get_sys_voltage(struct rk817_battery_device *battery)
{
	int vol, val = 0, vol_temp;

	val = rk817_bat_field_read(battery, SYS_VOL_H) << 8;
	val |= rk817_bat_field_read(battery, SYS_VOL_L) << 0;

	vol = (battery->voltage_k * val / 1000 + battery->voltage_b) * 60 / 46;

	if (battery->chip_id == RK809_ID) {
		vol_temp = vol * battery->pdata->bat_res_up /
			   battery->pdata->bat_res_down + vol;
		vol = vol_temp;
	}

	return vol;
}

static int rk817_bat_get_avg_current(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val = rk817_bat_field_read(battery, BAT_CUR_H) << 8;
	val |= rk817_bat_field_read(battery, BAT_CUR_L);

	if (val & 0x8000)
		val -= 0x10000;

	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static int rk817_bat_get_relax_cur1(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val = rk817_bat_field_read(battery, RELAX_CUR1_H) << 8;
	val |= rk817_bat_field_read(battery, RELAX_CUR1_L);

	if (val & 0x8000)
		val -= 0x10000;

	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static int rk817_bat_get_relax_cur2(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val |= rk817_bat_field_read(battery, RELAX_CUR2_H) << 8;
	val = rk817_bat_field_read(battery, RELAX_CUR2_L);

	if (val & 0x8000)
		val -= 0x10000;

	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static int rk817_bat_get_relax_current(struct rk817_battery_device *battery)
{
	int relax_cur1, relax_cur2;

	if (!is_rk817_bat_relax_mode(battery))
		return 0;

	relax_cur1 = rk817_bat_get_relax_cur1(battery);
	relax_cur2 = rk817_bat_get_relax_cur2(battery);

	return (relax_cur1 < relax_cur2) ? relax_cur1 : relax_cur2;
}

static int rk817_bat_get_ocv_current(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val = rk817_bat_field_read(battery, OCV_CUR_H) << 8;
	val |= rk817_bat_field_read(battery, OCV_CUR_L);

	if (val & 0x8000)
		val -= 0x10000;

	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static int rk817_bat_get_ocv_current0(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val = rk817_bat_field_read(battery, OCV_CUR0_H) << 8;
	val |= rk817_bat_field_read(battery, OCV_CUR0_L);

	if (val & 0x8000)
		val -= 0x10000;

	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static int rk817_bat_get_pwron_current(struct rk817_battery_device *battery)
{
	int cur, val = 0;

	val = rk817_bat_field_read(battery, PWRON_CUR_H) << 8;
	val |= rk817_bat_field_read(battery, PWRON_CUR_L);

	if (val & 0x8000)
		val -= 0x10000;
	cur = ADC_TO_CURRENT(val, battery->res_div);

	return cur;
}

static bool rk817_bat_remain_cap_is_valid(struct rk817_battery_device *battery)
{
	return !(rk817_bat_field_read(battery, Q_PRESS_H3) & CAP_INVALID);
}

static u32 rk817_bat_get_capacity_uah(struct rk817_battery_device *battery)
{
	u32 val = 0, capacity = 0;

	if (rk817_bat_remain_cap_is_valid(battery)) {
		val = rk817_bat_field_read(battery, Q_PRESS_H3) << 24;
		val |= rk817_bat_field_read(battery, Q_PRESS_H2) << 16;
		val |= rk817_bat_field_read(battery, Q_PRESS_L1) << 8;
		val |= rk817_bat_field_read(battery, Q_PRESS_L0) << 0;

		capacity = ADC_TO_CAPACITY_UAH(val, battery->res_div);
	} else {
	    // 20210529: remain_cap is Not-valid,Q_PRESS_H3=0xff
	    printk("get_capacity_uah: remain_cap is Not-valid,Q_PRESS_H3=0x%x,remain_cap=%d,nac=%d\n", 
	        rk817_bat_field_read(battery, Q_PRESS_H3), battery->remain_cap, battery->nac);
	    capacity = 0; //battery->remain_cap;
	}

    // 20210225: get_capacity_uah capacity = 1191444
	// DBG("get_capacity_uah capacity = %d\n", capacity);
	return  capacity;
}

static u32 rk817_bat_get_capacity_mah(struct rk817_battery_device *battery)
{
	u32 val, capacity = 0;

	if (rk817_bat_remain_cap_is_valid(battery)) {
		val = rk817_bat_field_read(battery, Q_PRESS_H3) << 24;
		val |= rk817_bat_field_read(battery, Q_PRESS_H2) << 16;
		val |= rk817_bat_field_read(battery, Q_PRESS_L1) << 8;
		val |= rk817_bat_field_read(battery, Q_PRESS_L0) << 0;

		capacity = ADC_TO_CAPACITY(val, battery->res_div);
	} else {
	    // 20210529: remain_cap is Not-valid,Q_PRESS_H3=0xff
	    printk("get_capacity_mah: remain_cap is Not-valid,Q_PRESS_H3=0x%x,remain_cap=%d\n", 
	        rk817_bat_field_read(battery, Q_PRESS_H3), battery->remain_cap);
	    capacity = battery->remain_cap/1000;
	}
	//DBG("Q_PRESS_H3 = 0x%x\n", rk817_bat_field_read(battery, Q_PRESS_H3));
	//DBG("Q_PRESS_H2 = 0x%x\n", rk817_bat_field_read(battery, Q_PRESS_H2));
	//DBG("Q_PRESS_H1 = 0x%x\n", rk817_bat_field_read(battery, Q_PRESS_L1));
	//DBG("Q_PRESS_H0 = 0x%x\n", rk817_bat_field_read(battery, Q_PRESS_L0));

	// DBG("get_capacity_mah capacity = %d\n", capacity);
	return  capacity;
}

/*
static void  fuel_gauge_q_init_info(struct rk817_battery_device *battery)
{
	BAT_INFO("Q_INIT_H3 = 0x%x\n", rk817_bat_field_read(battery, Q_INIT_H3));
	BAT_INFO("Q_INIT_H2 = 0x%x\n", rk817_bat_field_read(battery, Q_INIT_H2));
	BAT_INFO("Q_INIT_L1 = 0x%x\n", rk817_bat_field_read(battery, Q_INIT_L1));
	BAT_INFO("Q_INIT_L0 = 0x%x\n", rk817_bat_field_read(battery, Q_INIT_L0));
}
*/

static void rk817_bat_init_coulomb_cap(struct rk817_battery_device *battery,
				       u32 capacity)
{
	u8 buf;
	u32 cap;
	u32 val,temp;

	// 20211029：disable GG before write。
	// rk817_bat_field_write(battery, GG_EN, DISABLE);
	
	//fuel_gauge_q_init_info(battery);
	cap = CAPACITY_TO_ADC(capacity, battery->res_div);
	//DBG("new cap: 0x%x\n", cap);

	//battery->nac = capacity; // 20211102: 记录下来，如果电量读取错误了，可以重新再初始化。
#if 1	
	// 20211027: UBOOT 里面不修改电池的 电量和百分比后，到了内核里面出现读取电量错误的异常，LOG
	// 如下。说明 当前这个函数有可能写电量写不进去（可能adc正在更新中）。
	// [    1.027506] get_capacity_uah: remain_cap is Not-valid,Q_PRESS_H3=0xff,remain_cap=0
	// [    1.031430] rk817-bat: not_first_pwron: force_calib by ocv_vol=4586(volt=4348,c_avg=421),pre_soc=100,now_cap=3200
	// [    1.044880] get_capacity_uah: remain_cap is Not-valid,Q_PRESS_H3=0xff,remain_cap=3200000
	// [   43.177169] get_capacity_uah: remain_cap is Not-valid,Q_PRESS_H3=0xff,remain_cap=3200000
	do {
		buf = (cap >> 24) & 0xff;
		rk817_bat_field_write(battery, Q_INIT_H3, buf);
		buf = (cap >> 16) & 0xff;
		rk817_bat_field_write(battery, Q_INIT_H2, buf);
		buf = (cap >> 8) & 0xff;
		rk817_bat_field_write(battery, Q_INIT_L1, buf);
		buf = (cap >> 0) & 0xff;

		/* 20211102: XSF:  
		许盛飞.手机.PMU  16:14:15
		rk817_bat_init_coulomb_cap 的设置
		
		许盛飞.手机.PMU  16:14:24
		需要保证init0这个寄存器
		
		许盛飞.手机.PMU  16:14:37
		必须保证前后写进去的值不一样
		
		许盛飞.手机.PMU  16:14:40
		才更新
		*/
		temp = rk817_bat_field_read(battery, Q_INIT_L0);
		if (temp == buf) {
			buf += 1;
		}
		rk817_bat_field_write(battery, Q_INIT_L0, buf);

		val = rk817_bat_field_read(battery, Q_INIT_H3) << 24;
		val |= rk817_bat_field_read(battery, Q_INIT_H2) << 16;
		val |= rk817_bat_field_read(battery, Q_INIT_L1) << 8;
		val |= rk817_bat_field_read(battery, Q_INIT_L0) << 0;
		if(cap == val) break;
		BAT_INFO("init_coulomb_cap: capacity=%d,write cap=%d(0x%x),read value=%d(0x%x)\n", 
			capacity, cap, cap, val, val);
	} while (true);
#else 

	buf = (cap >> 24) & 0xff;
	rk817_bat_field_write(battery, Q_INIT_H3, buf);
	buf = (cap >> 16) & 0xff;
	rk817_bat_field_write(battery, Q_INIT_H2, buf);
	buf = (cap >> 8) & 0xff;
	rk817_bat_field_write(battery, Q_INIT_L1, buf);
	buf = (cap >> 0) & 0xff;
	rk817_bat_field_write(battery, Q_INIT_L0, buf);
#endif 
	// 20211029: enabe GG again.
	// rk817_bat_field_write(battery, GG_EN, ENABLE);
	
	//battery->rsoc = capacity * 1000 * 100 / battery->fcc;
	battery->remain_cap = capacity * 1000;
	//DBG("init_coulomb_cap: %d\n", battery->remain_cap);
	//fuel_gauge_q_init_info(battery);

    // 20210227: 此处增加一个会读操作，只是确认 Q_PRESS 里面内容是否已经根据 Q_INIT 更新了。
    // 20211102: 异常的LOG: rk817-bat: init_coulomb_cap: check new capacity=0mah, write cap=3200mah
    // --看下面的LOG,写进去的电量是 3200，结果读出来的是344，不是错误值，也不是0.这个怎么处理呢？
    /*
[    1.062539] rk817-bat: init_coulomb_cap: check new capacity=0mah, write cap=3200mah
[    1.063327] rk817-bat: bat_init_fg: rsoc = 0, fcc = 3200/0,nac=3200
[    1.066670] probe_init: dsoc=100000,rsoc=0,remain_cap=344,volt_avg=4348,ocv_volt=16,qmax=3400,realfcc=0,rvsfcc=0
[    1.076222] rk817-bat: v_avg=4348,c_avg=334,rsoc=0,dsoc=100000,remain_cap=344,use_fcc=0,real_fcc=0,chrg=3    
    */
	//val = rk817_bat_get_capacity_mah(battery);
	//BAT_INFO("init_coulomb_cap: check new capacity=%dmah, write cap=%dmah\n", val, capacity);
}

static void rk817_bat_save_cap(struct rk817_battery_device *battery,
			       int capacity)
{
	u8 buf;
	static u32 old_cap;

	if (capacity >= battery->qmax)
		capacity = battery->qmax;
	if (capacity <= 0)
		capacity = 0;
	if (old_cap == capacity)
		return;

	old_cap = capacity;
	buf = (capacity >> 16) & 0xff;
	rk817_bat_field_write(battery, REMAIN_CAP_REG2, buf);
	buf = (capacity >> 8) & 0xff;
	rk817_bat_field_write(battery, REMAIN_CAP_REG1, buf);
	buf = (capacity >> 0) & 0xff;
	rk817_bat_field_write(battery, REMAIN_CAP_REG0, buf);
}

static void rk817_bat_update_qmax(struct rk817_battery_device *battery,
				  u32 capacity)
{
	u8 buf;
	u32 cap_adc;

	cap_adc = CAPACITY_TO_ADC(capacity, battery->res_div);
	buf = (cap_adc >> 24) & 0xff;
	rk817_bat_field_write(battery, Q_MAX_H3, buf);
	buf = (cap_adc >> 16) & 0xff;
	rk817_bat_field_write(battery, Q_MAX_H2, buf);
	buf = (cap_adc >> 8) & 0xff;
	rk817_bat_field_write(battery, Q_MAX_L1, buf);
	buf = (cap_adc >> 0) & 0xff;
	rk817_bat_field_write(battery, Q_MAX_L0, buf);
	//battery->qmax = capacity;
}

/*
static int rk817_bat_get_qmax(struct rk817_battery_device *battery)
{
	u32 capacity;
	int val = 0;

	val = rk817_bat_field_read(battery, Q_MAX_H3) << 24;
	val |= rk817_bat_field_read(battery, Q_MAX_H2) << 16;
	val |= rk817_bat_field_read(battery, Q_MAX_L1) << 8;
	val |= rk817_bat_field_read(battery, Q_MAX_L0) << 0;
	capacity = ADC_TO_CAPACITY(val, battery->res_div);
	battery->qmax = capacity;
	return capacity;
}
*/

static int rk817_bat_get_real_fcc(struct rk817_battery_device *battery)
{
	u32 fcc = 0;

	fcc |= rk817_bat_field_read(battery, NEW_FCC_REG2) << 16;
	fcc |= rk817_bat_field_read(battery, NEW_FCC_REG1) << 8;
	fcc |= rk817_bat_field_read(battery, NEW_FCC_REG0) << 0;

	/*
	if (fcc < MIN_FCC) {
		DBG("invalid fcc(%d), use design cap", fcc);
		fcc = battery->pdata->design_capacity;
		rk817_bat_save_real_fcc(battery, fcc);
	} else if (fcc > battery->pdata->design_qmax) {
		DBG("invalid fcc(%d), use qmax", fcc);
		fcc = battery->pdata->design_qmax;
		rk817_bat_save_real_fcc(battery, fcc);
	}
	*/
	return fcc;
}

// 20210225: 这个方式统计电量有问题。从上次放电的LOG看，最终低电关机打印出来的LOG如下：
// rk817-bat: v_avg=3605,v_sys=3606,c_avg=-1007,v_relx=0,rsoc=19676,dsoc=19749,remain_cap=787072,fcc=4000,chrg=0,c_pwon=23
// rk817-bat: v_avg=3606,v_sys=3556,c_avg=-1001,v_relx=0,rsoc=19608,dsoc=19676,remain_cap=784320,fcc=4000,chrg=0,c_pwon=23
// 从LOG看出，电池电量还有 18%左右的时候，电压就已经掉到 3.8V，此时就需要关机了. --保留电量按20%来计算，否则大电流放电时候，电压可能
// 会拉得很低，比如下面的LOG就是打开CAMERA时候打印出来的:
// v_avg=3469,v_sys=3436,c_avg=-1423,v_relx=0,rsoc=12284,dsoc=12734,remain_cap=1211396,fcc=4000,chrg=0,c_pwon=19
// v_avg=3449,v_sys=3429,c_avg=-1498,v_relx=0,rsoc=9408,dsoc=9734,remain_cap=1096328,fcc=4000,chrg=0,c_pwon=19
// v_avg=3672,v_sys=3619,c_avg=-511,v_relx=0,rsoc=9352,dsoc=1000,remain_cap=1094092,fcc=4000,chrg=0,c_pwon=15 --低电流情况下，电压回复。
static int rk817_bat_get_rsoc(struct rk817_battery_device *battery)
{
	int remain_cap;
	remain_cap = battery->remain_cap; //rk817_bat_get_capacity_uah(battery);

    // 20210414: 我们考虑  remain_cap > fcc 的情况： remain_cap=4187340,fcc=4000。如果不调整，会导致
    // 100% 的电量维持很长时间。rsv_fcc 和 used_fcc 的单位都是 mah. remain_cap 的单位是 uah, 所以返回值是
    // 百分比*1000, 这样可以反馈比较微小的变化，即 万分之几。
	return (remain_cap-battery->reserved_fcc*1000) * 100/ battery->max_used_fcc; // DIV(battery->real_fcc-battery->reserved_fcc); 
}


// 20211025: 由于我们获取电池比分比的算法不一样，所以当我们知道电池的电量的时候，通过电量来获取实际
// 的百分比。
static int rk817_bat_get_cap_by_rsoc(struct rk817_battery_device *battery, int rsoc)
{
	return battery->max_used_fcc*rsoc/100 + battery->reserved_fcc;
}

static void rk817_bat_save_real_fcc_and_update_used_fcc(struct rk817_battery_device *battery)
{
	u8 buf;
    int fcc = battery->real_fcc;

    // 20210416: 是否需要限制最大电池容量??有些标称是 4000mah的电池，实际冲完了，统计电量可以达到
    // remain_cap=4805164.
    // v_avg=4599,v_sys=4592,c_avg=3,v_relx=4599,rsoc=123676,dsoc=100000,remain_cap=4804992,use_fcc=3400,rv_fcc=600,chrg=4
    if( fcc > battery->qmax ) {
        battery->real_fcc = fcc = battery->qmax;

        // 20211102: 由于rk817_bat_init_coulomb_cap 函数会导致 电量计异常，怎么处理呢？由于电量错误，
    	// 使得电量计里面的电量大于最大电量了,
        rk817_bat_init_coulomb_cap(battery, battery->real_fcc);
    }
	buf = (fcc >> 16) & 0xff;
	rk817_bat_field_write(battery, NEW_FCC_REG2, buf);
	buf = (fcc >> 8) & 0xff;
	rk817_bat_field_write(battery, NEW_FCC_REG1, buf);
	buf = (fcc >> 0) & 0xff;
	rk817_bat_field_write(battery, NEW_FCC_REG0, buf);
	
    // 20210416: fcc/real_fcc/rsv_fcc/nac/qmax/used_fcc 单位都是 mah, remain_cap 的单位是 uah.所以计算出来的
    // rsoc 单位是 100% * 1000.
    battery->reserved_fcc = battery->real_fcc*RESEVED_FCC_PERCENT/100; 
	battery->max_used_fcc = (battery->real_fcc-battery->reserved_fcc);
	battery->rsoc = rk817_bat_get_rsoc(battery);
	BAT_INFO("save_real_fcc: real_fcc=%d,reserved_fcc=%d,max_used_fcc=%d,save fcc=%d,fcc=%d,rsoc=%d\n", battery->real_fcc, 
	    battery->max_used_fcc, battery->reserved_fcc, fcc, battery->fcc, battery->rsoc);
}

static int rk817_get_100percent_soc(struct rk817_battery_device *battery, int soc) 
{
    return ( soc + 500)/1000; // battery->dsoc
}

/*
static int rk817_bat_get_off_count(struct rk817_battery_device *battery)
{
	return rk817_bat_field_read(battery, OFF_CNT);
}
*/

static int rk817_bat_get_ocv_count(struct rk817_battery_device *battery)
{
	return rk817_bat_field_read(battery, OCV_CNT);
}

static int rk817_bat_vol_to_soc(struct rk817_battery_device *battery,
				int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, ocv_soc;

	ocv_table = battery->pdata->ocv_table;
	ocv_size = battery->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	ocv_soc = ab_div_c(temp, MAX_PERCENTAGE, MAX_INTERPOLATE);

	return ocv_soc;
}

static int rk817_bat_vol_to_cap(struct rk817_battery_device *battery,
				int voltage)
{
	u32 *ocv_table, temp;
	int ocv_size, capacity;

	ocv_table = battery->pdata->ocv_table;
	ocv_size = battery->pdata->ocv_size;
	temp = interpolate(voltage, ocv_table, ocv_size);
	capacity = ab_div_c(temp, battery->fcc, MAX_INTERPOLATE);

	return capacity;
}

static void rk817_bat_save_dsoc(struct rk817_battery_device *battery,
				int save_soc)
{
	static int last_soc = -1;

	if (last_soc != save_soc) {
		rk817_bat_field_write(battery, SOC_REG0,
				      save_soc & 0xff);
		rk817_bat_field_write(battery, SOC_REG1,
				      (save_soc >> 8) & 0xff);
		rk817_bat_field_write(battery, SOC_REG2,
				      (save_soc >> 16) & 0xff);

		last_soc = save_soc;
	}
}

static int rk817_bat_get_prev_dsoc(struct rk817_battery_device *battery)
{
	int soc_save;

	soc_save = rk817_bat_field_read(battery, SOC_REG0);
	soc_save |= (rk817_bat_field_read(battery, SOC_REG1) << 8);
	soc_save |= (rk817_bat_field_read(battery, SOC_REG2) << 16);

	return soc_save;
}

static bool is_rk817_bat_first_pwron(struct rk817_battery_device *battery)
{
	if (rk817_bat_field_read(battery, BAT_CON)) {
		rk817_bat_field_write(battery, BAT_CON, 0x00);
		return true;
	}

	return false;
}

static int rk817_bat_get_charge_status(struct rk817_battery_device *battery)
{
	int status;

	if (battery->chip_id == RK809_ID) {
		if ((battery->voltage_avg > battery->pdata->design_max_voltage) &&
		    (battery->current_avg > 0) &&
		    ((battery->current_avg < 500) ||
		     (battery->rsoc / 1000 == 100)))
			return CHARGE_FINISH;

		if (battery->plugin_trigger)
			return CC_OR_CV_CHRG;
		else
			return CHRG_OFF;
	}
	
	status = rk817_bat_field_read(battery, CHG_STS);

	if (status == CC_OR_CV_CHRG) {
	    // 20210222: 此处不需要调整，我们需要真正的 CHARGE_FINISH 来校准电量计里面
	    // 的 remain_cap 计数。
		/*if (battery->rsoc == 100 * 1000 ) {
			DBG("charge to finish\n");
			status = CHARGE_FINISH;
		}*/
	}

	switch (status) {
	case CHRG_OFF:
		DBG("charge off...\n");
		break;
	case DEAD_CHRG:
		DBG("dead charge...\n");
		break;
	case TRICKLE_CHRG:
		DBG("trickle charge...\n");
		break;
	case CC_OR_CV_CHRG:
		DBG("CC or CV charge...\n");
		break;
	case CHARGE_FINISH:
		DBG("charge finish...\n");
		break;
	case USB_OVER_VOL:
		DBG("USB over voltage...\n");
		break;
	case BAT_TMP_ERR:
		DBG("battery temperature error...\n");
		break;
	case BAT_TIM_ERR:
		DBG("battery timer error..\n");
		break;
	default:
		break;
	}

	return status;
}


static int get_charge_status(struct rk817_battery_device *battery)
{
	return rk817_bat_get_charge_status(battery);
}

/*
static bool is_rk817_bat_ocv_valid(struct rk817_battery_device *battery)
{
	return (!battery->is_initialized && battery->pwroff_min >= 30);
}
*/

static void rk817_bat_gas_gaugle_enable(struct rk817_battery_device *battery)
{
		rk817_bat_field_write(battery, GG_EN, ENABLE);
}

static void rk817_bat_gg_con_init(struct rk817_battery_device *battery)
{
	rk817_bat_field_write(battery, RLX_SPT, S_8_MIN);
	rk817_bat_field_write(battery, ADC_OFF_CAL_INTERV, S_8_MIN);
	rk817_bat_field_write(battery, VOL_OUT_MOD, AVERAGE_MODE);
	rk817_bat_field_write(battery, CUR_OUT_MOD, AVERAGE_MODE);
}

static void  rk817_bat_adc_init(struct rk817_battery_device *battery)
{
	rk817_bat_field_write(battery, SYS_VOL_ADC_EN, ENABLE);
	rk817_bat_field_write(battery, TS_ADC_EN, ENABLE);
	rk817_bat_field_write(battery, USB_VOL_ADC_EN, ENABLE);
	rk817_bat_field_write(battery, BAT_VOL_ADC_EN, ENABLE);
	rk817_bat_field_write(battery, BAT_CUR_ADC_EN, ENABLE);
}

static void rk817_bat_init_info(struct rk817_battery_device *battery)
{
	battery->design_cap = battery->pdata->design_capacity;
	battery->qmax = battery->pdata->design_qmax;
	battery->bat_res = battery->pdata->bat_res;
	battery->monitor_ms = battery->pdata->monitor_sec * TIMER_MS_COUNTS;
	battery->res_div = (battery->pdata->sample_res == SAMPLE_RES_20MR) ?
		       SAMPLE_RES_DIV2 : SAMPLE_RES_DIV1;
	//DBG("battery->qmax :%d\n", battery->qmax);
}

static int rk817_bat_get_prev_cap(struct rk817_battery_device *battery)
{
	int val = 0;

	val = rk817_bat_field_read(battery, REMAIN_CAP_REG2) << 16;
	val |= rk817_bat_field_read(battery, REMAIN_CAP_REG1) << 8;
	val |= rk817_bat_field_read(battery, REMAIN_CAP_REG0) << 0;

	return val;
}

#if 0
static u8 rk817_bat_get_halt_cnt(struct rk817_battery_device *battery)
{
	return rk817_bat_field_read(battery, HALT_CNT_REG);
}

static void rk817_bat_inc_halt_cnt(struct rk817_battery_device *battery)
{
	u8 cnt;

	cnt =  rk817_bat_field_read(battery, HALT_CNT_REG);
	rk817_bat_field_write(battery, HALT_CNT_REG, ++cnt);
}

static bool is_rk817_bat_last_halt(struct rk817_battery_device *battery)
{
	int pre_cap = rk817_bat_get_prev_cap(battery);
	int now_cap = rk817_bat_get_capacity_mah(battery);

	/* over 10%: system halt last time */
	if (abs(now_cap - pre_cap) > (battery->fcc / 10)) {
		rk817_bat_inc_halt_cnt(battery);
		return true;
	} else {
		return false;
	}
}
#endif 

static u8 is_rk817_bat_initialized(struct rk817_battery_device *battery)
{
	u8 val = rk817_bat_field_read(battery, FG_INIT);

	if (val) {
		rk817_bat_field_write(battery, FG_INIT, 0x00);
		return true;
	} else {
		return false;
	}
}

#if 0
static void rk817_bat_calc_sm_linek(struct rk817_battery_device *battery)
{
	int linek;
	int diff, delta;
	int current_avg = rk817_bat_get_avg_current(battery);

	delta = abs(battery->dsoc - battery->rsoc);
	diff = delta * 3;/* speed:3/4 */

	if (current_avg > 0) {
		if (battery->dsoc < battery->rsoc)
			linek = 1000 * (delta + diff) / DIV(diff);
		else if (battery->dsoc > battery->rsoc)
			linek = 1000 * diff / DIV(delta + diff);
		else
			linek = 1000;
	} else {
		if (battery->dsoc < battery->rsoc)
			linek = -1000 * diff / DIV(delta + diff);
		else if (battery->dsoc > battery->rsoc)
			linek = -1000 * (delta + diff) / DIV(diff);
		else
			linek = -1000;
	}

	battery->dbg_meet_soc = (battery->dsoc >= battery->rsoc) ?
		(battery->dsoc - diff) : (battery->rsoc - diff);

	battery->sm_linek = linek;
	battery->sm_remain_cap = battery->remain_cap;
	battery->dbg_calc_dsoc = battery->dsoc;
	battery->dbg_calc_rsoc = battery->rsoc;
}

static void rk817_bat_smooth_algo_prepare(struct rk817_battery_device *battery)
{
	battery->smooth_soc = battery->dsoc;

	DBG("<%s>. dsoc=%d, dsoc:smooth_soc=%d\n",
	    __func__, battery->dsoc, battery->smooth_soc);
	rk817_bat_calc_sm_linek(battery);
}

static void rk817_bat_finish_algo_prepare(struct rk817_battery_device *battery)
{
	battery->finish_base = get_boot_sec();

	if (!battery->finish_base)
		battery->finish_base = 1;
}
#endif 

static void rk817_bat_init_dsoc_algorithm(struct rk817_battery_device *battery)
{
	if (battery->dsoc >= 100 * 1000)
		battery->dsoc = 100 * 1000;
	else if (battery->dsoc <= 0)
		battery->dsoc = 0;
	/* init current mode */
	battery->voltage_avg = rk817_bat_get_battery_voltage(battery);
	battery->current_avg = rk817_bat_get_avg_current(battery);

    // 20210226,hsl: no needs.
    #if 0
	if (get_charge_status(battery) == CHARGE_FINISH) {
		rk817_bat_finish_algo_prepare(battery);
		battery->work_mode = MODE_FINISH;
	} else {
		rk817_bat_smooth_algo_prepare(battery);
		battery->work_mode = MODE_SMOOTH;
	}
	DBG("%s, sm_remain_cap = %d, smooth_soc = %d\n",
	    __func__, battery->sm_remain_cap, battery->smooth_soc);
	#endif 
}

static void rk817_bat_first_pwron(struct rk817_battery_device *battery)
{
	battery->rsoc =
		rk817_bat_vol_to_soc(battery,
				     battery->pwron_voltage) * 1000;/* uAH */
	
	battery->fcc	= battery->pdata->design_capacity;
	battery->nac = rk817_bat_vol_to_cap(battery, battery->pwron_voltage);

	battery->real_fcc = battery->fcc;
	rk817_bat_save_real_fcc_and_update_used_fcc(battery);
	
	rk817_bat_update_qmax(battery, battery->qmax);

	battery->dsoc = battery->rsoc;
	//rk817_bat_save_fcc(battery, battery->fcc);
	BAT_INFO("first_pwron:rsoc = %d,fcc = %d,nac = %d,pwo_volt=%d,remain_cap=%d\n",
	    battery->rsoc, battery->fcc, battery->nac, battery->pwron_voltage, battery->remain_cap);
}

static void rk817_bat_not_first_pwron(struct rk817_battery_device *battery)
{
	int now_cap, pre_soc, pre_cap,vol_cap;
    bool need_save_real_fcc = false;
    bool need_update_cap = false;
    int   ocv_vol,c_avg, c_volt;
	pre_soc = rk817_bat_get_prev_dsoc(battery);
	pre_cap = rk817_bat_get_prev_cap(battery);
	now_cap = rk817_bat_get_capacity_mah(battery);
	battery->real_fcc = rk817_bat_get_real_fcc(battery);
	vol_cap = rk817_bat_vol_to_cap(battery, battery->pwron_voltage);
	//battery->remain_cap = pre_cap * 1000;
	battery->is_halt = false; //is_rk817_bat_last_halt(battery);
	battery->halt_cnt = 0; //rk817_bat_get_halt_cnt(battery);
	battery->is_initialized = is_rk817_bat_initialized(battery);
	battery->is_ocv_calib = 0; //is_rk817_bat_ocv_valid(battery);

	// 20211025-LOG: not_first_pwron:pre_soc=54263,pre_cap=1963,now_cap=1973,halt=0,inited=1,ovc_valid=0
	// 拔掉电池后再插上： not_first_pwron:pre_soc=0,pre_cap=0,now_cap=0,real_fcc=3200
    BAT_INFO("not_first_pwron0:pwo_volt=%d pre_soc=%d,pre_cap=%d,now_cap=%d,vol_cap=%d,real_fcc=%d\n",
        battery->pwron_voltage, pre_soc, pre_cap, now_cap, vol_cap,battery->real_fcc);

#if 0      
	if (battery->is_halt) {
		// 20210918-LOG: rk817-bat: system halt last time... cap: pre=3400, now=4608 (qmax=3400,realfcc=3400)
		BAT_INFO("system halt last time... cap: pre=%d, now=%d\n",
			 pre_cap, now_cap);
		if (now_cap < 0)
			now_cap = 0;
		if(now_cap > (battery->qmax)){
			now_cap = (battery->qmax);
		}
		
		rk817_bat_init_coulomb_cap(battery, now_cap); // 20211025: will update remain_cap.
		pre_cap = now_cap;
		pre_soc = battery->rsoc;
		goto finish;
	} else if (battery->is_ocv_calib || !battery->is_initialized) {
		/* not initialized and poweroff_cnt above 30 min */
		ocv_vol = rk817_bat_get_ocv_voltage(battery);
		ocv_soc = rk817_bat_vol_to_soc(battery, ocv_vol);
		ocv_cap = rk817_bat_vol_to_cap(battery, ocv_vol);
		pre_cap = ocv_cap;
		battery->ocv_pre_dsoc = pre_soc;
		battery->ocv_new_dsoc = ocv_soc;
		if (abs(ocv_soc - pre_soc) >= battery->pdata->max_soc_offset) {
			battery->ocv_pre_dsoc = pre_soc;
			battery->ocv_new_dsoc = ocv_soc;
			battery->is_max_soc_offset = true;
			BAT_INFO("trigger max soc offset, dsoc: %d -> %d\n",
				 pre_soc, ocv_soc);
			pre_soc = ocv_soc;
		}
		BAT_INFO("OCV calib: cap=%d, rsoc=%d\n", ocv_cap, ocv_soc);
	} else if (battery->pwroff_min > 0) {
		ocv_vol = rk817_bat_get_ocv_voltage(battery);
		ocv_soc = rk817_bat_vol_to_soc(battery, ocv_vol);
		ocv_cap = rk817_bat_vol_to_cap(battery, ocv_vol);
		battery->force_pre_dsoc = pre_soc;
		battery->force_new_dsoc = ocv_soc;
		if (abs(ocv_soc - pre_soc) >= 80) {
			battery->is_force_calib = true;
			BAT_INFO("dsoc force calib: %d -> %d\n",
				 pre_soc, ocv_soc);
			pre_soc = ocv_soc;
			pre_cap = ocv_cap;
		}
	} 
finish:
	
	battery->dsoc = pre_soc;
	battery->nac = pre_cap;
	if (battery->nac < 0)
		battery->nac = 0;
	else if(battery->nac > battery->qmax ) {
	    battery->nac = battery->qmax;
	}
#else 
#ifdef CONFIG_PROC_RATTA
	if(battery->real_fcc < (battery->fcc*5/10) || battery->real_fcc > battery->qmax) {
#else
	if(battery->real_fcc < (battery->fcc*7/10) || battery->real_fcc > battery->qmax) {
#endif
		BAT_INFO("not_first_pwron: adjust real_fcc %d to %d qmax=%d\n", battery->real_fcc,
			battery->fcc, battery->qmax);
		battery->real_fcc = battery->fcc;
		need_save_real_fcc = true;
	}
	
	if(need_save_real_fcc) {
    	rk817_bat_save_real_fcc_and_update_used_fcc(battery);
    } else {
    	// 20211025: 我们更具实际的 fcc 来更新 保留的电量。百分比还是固定 RESEVED_FCC_PERCENT.
		// battery->reserved_fcc = battery->fcc*RESEVED_FCC_PERCENT/100; // fcc*1000* (18/100)
		battery->reserved_fcc = battery->real_fcc*RESEVED_FCC_PERCENT/100; 
		battery->max_used_fcc = (battery->real_fcc-battery->reserved_fcc);
	}
	// 20210414-LOG:rsoc_init1:realfcc=4200,fcc=4000,rev_fcc=600,max_used_fcc=3600
    BAT_INFO("not_first_pwron: realfcc=%d,fcc=%d,rev_fcc=%d,max_used_fcc=%d\n",
        battery->real_fcc, battery->fcc, battery->reserved_fcc, 
        battery->max_used_fcc);

	pre_soc = -1;
	if(now_cap ==0 || pre_cap == 0 ||  now_cap < battery->reserved_fcc/2 ){
		need_update_cap = true;
	}

	// 20211103: 此处主要调整原来记录的电量和开机后检测到电池电压之间发生巨大差异的情况。产生的原因
	// 可能是上次启动更新 cap的时候I2C出错，导致电量计里面电量基本是0.但是电压却有 4.3V的情况。但是这个
	// 需要和低电插入充电器开机分开。低电关机的时候，电量是0，插入充电器，充电电流接近2A的时候，电池的
	// 电压也有可能达到 4.1/4.2V.此时不能直接调整电量到30%左右。这种情况我们判断 
	// now_cap < battery->fcc/(RESEVED_FCC_PERCENT*2) 是否可以过滤？？
	#if 0
	if(!need_update_cap) {
		// 20210524: 这个时候，只能获取到有效的 BAT-VOLT. ocv/sys/ocv0 都不准确。如果没有插拔电池，ocv
        // 会更加准确。只有在 ovc 无效的情况下，才采用 bat_vol.
        ocv_vol = rk817_bat_get_ocv_voltage(battery);
        c_avg = rk817_bat_get_avg_current(battery);
        c_volt = rk817_bat_get_battery_voltage(battery);  
        if(ocv_vol < 2200) {
            // 20211025: 由于没有办法获取到 OCV,此处需要根据电流进行修正，比如下面的LOG：
            // rk817-bat: v_avg=3948,c_avg=1569,rsoc=-17185,dsoc=-17647,remain_cap=12556,use_fcc=2720,real_fcc=3200,chrg=3
            // 我们按电池内阻 100毫欧的幅度来调整.
            ocv_vol = c_volt - c_avg/100;
        }
		pre_soc = rk817_bat_vol_to_soc(battery, ocv_vol);
		if(pre_soc - pre_soc > 50*1000) {
			need_update_cap = true;
		}
	}		
	#endif 
	// 20211025：下面的情况是处理拔电池异常，或者第一次上电(由于uboot里面做了处理，所以内核里面无法
	// 识别到 first_power_on). 正常情况下，我们用到容量的 15% 之后就会关机了。
	if(need_update_cap) {
		// 20210522: 拔掉电池重新接上，由于UBOOT里面会初始化，所以此处 init = true,但是电量
	    // 和电压严重不匹配，LOG 如下。所以此处还是需要判断 电压和当前电量的关系。
	    // 20210525: 如果是大电流烤机情况下低电关机，然后插入充电器开机，会出现此时电量是0，但是
	    // 电压很高的情况。所以这里不能判断 dsoc < 5 就进行调整，而是看上次记录的电量( nac ).如果中间
	    // 拔掉了电池或者进入MASKROM烧写，此时的 nac 是个很小的值。正常使用的情况下，nac 不会比 
	    // reserved_fcc 低很多。
	    /*
	    [    0.839892] rk817-bat: get_capacity_mah capacity = 6
	    [    0.840489] rk817-bat: not_first_pwron:pre_soc=0,pre_cap=7,now_cap=6,halt=0,inited=1,ovc_valid=0
	    [    0.840515] rk817-bat: initialized yet..
	    [    0.843062] rk817-bat: not_first_pwron:dsoc=0 cap=7 v=4341 ov=4388 rv=0 min=0 psoc=0 pcap=7
	    */
		if(pre_soc == -1) {
	        ocv_vol = rk817_bat_get_ocv_voltage(battery);
	        c_avg = rk817_bat_get_avg_current(battery);
	        c_volt = rk817_bat_get_battery_voltage(battery);  
	        if(ocv_vol < 2200) {
	            // 20211025: 由于没有办法获取到 OCV,此处需要根据电流进行修正，比如下面的LOG：
	            // rk817-bat: v_avg=3948,c_avg=1569,rsoc=-17185,dsoc=-17647,remain_cap=12556,use_fcc=2720,real_fcc=3200,chrg=3
	            // 我们按电池内阻 100毫欧的幅度来调整.
	            ocv_vol = c_volt - c_avg/battery->bat_res;
	        }
			pre_soc = rk817_bat_vol_to_soc(battery, ocv_vol);
		}
		
		now_cap = rk817_bat_get_cap_by_rsoc(battery, pre_soc);
		rk817_bat_init_coulomb_cap(battery, now_cap);

		/*
[    1.032889] rk817-bat: not_first_pwron:pre_soc=0,pre_cap=4,now_cap=12
[    1.033813] rk817-bat: not_first_pwron: force_calib by ocv_vol=15		
		adjust by volt/current: force_calib by ocv_vol=3945(volt=3958,c_avg=1586)
		*/
		BAT_INFO("not_first_pwron: force_calib by ocv_vol=%d(volt=%d,c_avg=%d),pre_soc=%d,now_cap=%d\n", ocv_vol,
			c_volt, c_avg, pre_soc, now_cap);
	 	pre_soc *= 1000;
	}
	
	battery->dsoc = pre_soc;
	battery->nac = now_cap;

	// 20211102: 我们通过LOG发现，写进去的 cap是 3200mah，结果读出来的是 344uah.没有提示错误。
	// 所以此处设置 remain_cap 为 now_cap,这样可以通过前后电量的对比，判断当前817里面电量计的读数是否
	// 有问题。
	battery->remain_cap = now_cap*1000;
	//battery->remain_cap = rk817_bat_get_capacity_uah(battery);
#endif
	
    // 20211025: not_first_pwron:dsoc=39536 nac=1572 v=3925 ov=3720 rv=0,remain_cap=1572000
	BAT_INFO("not_first_pwron:dsoc=%d nac=%d v=%d ov=%d rv=%d,remain_cap=%d\n",
	    battery->dsoc, battery->nac, rk817_bat_get_battery_voltage(battery),
	    rk817_bat_get_ocv_voltage(battery),
	    rk817_bat_get_relax_voltage(battery),
	    battery->remain_cap);
}

static void rk817_bat_rsoc_init(struct rk817_battery_device *battery)
{
    //int ocv_cap, ocv_soc, ocv_vol;
    //bool 	need_save_real_fcc = false;
	battery->is_first_power_on = is_rk817_bat_first_pwron(battery);
	battery->pwroff_min = 0; //rk817_bat_get_off_count(battery);
	battery->pwron_voltage = rk817_bat_get_pwron_voltage(battery);

	// 20210304: fcc 以 DTS里面的为准。否则烧错固件了不能及时更新回来。其实FCC是固定值，没有必要
    // 保存/加载。
	battery->fcc = battery->pdata->design_capacity; //rk817_bat_get_fcc(battery);
	
    //20210221-LOG: rk817_bat_rsoc_init, is_first_power_on = 0, pwroff_min = 0, pwron_voltage = 4278
    // 如果是插着USB开机，这个 pwron_voltage 可能会比较高。--20210415: 即使拔掉电池重新开机，此处 first-power-on
    // 也是0，可能是 uboot 里面做了处理了。
	DBG("%s, is_first_power_on = %d,pwron_voltage = %d\n",
	    __func__, battery->is_first_power_on,
	    battery->pwron_voltage);

	// 20211102: 拔掉电池开机，uboot里面会初始化，但是可能I2C会出错: rk8xx_write: write reg 0x57 failed, ret=-121
	// 所以到了内核这里，还是出现 is_first_power_on = true的情况。
	if (battery->is_first_power_on) {
		rk817_bat_first_pwron(battery);
		rk817_bat_init_coulomb_cap(battery, battery->nac);
		battery->remain_cap = rk817_bat_get_capacity_uah(battery);
	} else {
		rk817_bat_not_first_pwron(battery);
	}
	//rk817_bat_save_dsoc(battery, battery->dsoc);
	
	// 20210225:这种情况下，重新开机，允许电池电量突变更新。
	battery->s2r = true; 
}

static void rk817_bat_caltimer_isr(unsigned long data)
{
	struct rk817_battery_device *battery =
		(struct rk817_battery_device *)data;

	mod_timer(&battery->caltimer, jiffies + MINUTE(8) * HZ);
	queue_delayed_work(battery->bat_monitor_wq,
			   &battery->calib_delay_work,
			   msecs_to_jiffies(10));
}

static void rk817_bat_internal_calib(struct work_struct *work)
{
	struct rk817_battery_device *battery = container_of(work,
			struct rk817_battery_device, calib_delay_work.work);

	return;

	rk817_bat_current_calibration(battery);
	/* calib voltage kb */
	rk817_bat_init_voltage_kb(battery);

	DBG("caltimer:coffset=0x%x\n", rk817_bat_get_coffset(battery));
}

static void rk817_bat_init_caltimer(struct rk817_battery_device *battery)
{
	setup_timer(&battery->caltimer,
		    rk817_bat_caltimer_isr,
		    (unsigned long)battery);
	battery->caltimer.expires = jiffies + MINUTE(8) * HZ;
	add_timer(&battery->caltimer);
	INIT_DELAYED_WORK(&battery->calib_delay_work, rk817_bat_internal_calib);
}

static void rk817_bat_init_fg(struct rk817_battery_device *battery)
{
	rk817_bat_adc_init(battery);
	rk817_bat_gas_gaugle_enable(battery);
	rk817_bat_gg_con_init(battery);
	rk817_bat_init_voltage_kb(battery);
	rk817_bat_set_relax_sample(battery);
	rk817_bat_ocv_thre(battery, 0xff);
	rk817_bat_init_caltimer(battery);
	rk817_bat_rsoc_init(battery);
	//DBG("bat_init_fg0:capactiy = %d\n", rk817_bat_get_capacity_mah(battery));
	
    battery->rsoc = rk817_bat_get_rsoc(battery);
	// 20210416: bat_init_fg: rsoc = 105000, fcc = 4000/4200,nac=4200
	BAT_INFO("bat_init_fg: rsoc = %d, fcc = %d/%d,nac=%d\n", battery->rsoc, battery->fcc, 
		battery->real_fcc, battery->nac);

    // 20210227: 设置了 init_coulomb_cap 后，此处读取到的 mah 就是已经更新后的电量。但是为啥在 work 里面更新不能生效呢?
	//DBG("bat_init_fg1:capactiy = %d\n", rk817_bat_get_capacity_mah(battery));
	
	rk817_bat_init_dsoc_algorithm(battery); 

	// 20210304: rk817_bat_init_info will set qmax.
	//battery->qmax = battery->pdata->design_qmax; //rk817_bat_get_qmax(battery);
	battery->voltage_avg = rk817_bat_get_battery_voltage(battery);
	battery->voltage_sys = rk817_bat_get_sys_voltage(battery);

	battery->voltage_ocv = rk817_bat_get_ocv_voltage(battery);
	battery->voltage_relax = rk817_bat_get_relax_voltage(battery);
	battery->current_avg = rk817_bat_get_avg_current(battery);
	battery->dbg_pwr_dsoc = battery->dsoc;
	battery->dbg_pwr_rsoc = battery->rsoc;
	battery->dbg_pwr_vol = battery->voltage_avg;
	battery->temperature = 25;//VIRTUAL_TEMPERATURE;
	
	// 20210818: 为了方便后面排查问题，我们总是把开机的电池重要信息打印出来。
	// 20210918: 无法进行OTA升级的LOG:
	// rk817-bat: system halt last time... cap: pre=3400, now=4608
	// probe init: dsoc=0,rsoc=100000,remain_cap=3400000,volt_avg=4333,ocv_volt=4306,qmax=3400,realfcc=3400
	
	printk("probe_init: dsoc=%d,rsoc=%d,"
	    "remain_cap=%d,volt_avg=%d,ocv_volt=%d,qmax=%d,realfcc=%d,rvsfcc=%d,temp=%d\n",
	    battery->dsoc, battery->rsoc, battery->remain_cap,
	    battery->voltage_avg,
	    battery->voltage_ocv, 
	    battery->qmax, battery->real_fcc,battery->reserved_fcc,battery->temperature);

	// 20210221-LOG: OCV_THRE_VOL: 0xff
	// DBG("OCV_THRE_VOL: 0x%x", rk817_bat_field_read(battery, OCV_THRE_VOL));
	if(battery->dsoc == 0 || (battery->dsoc < 10000 && battery->rsoc > 20000) ){
		battery->dsoc = battery->rsoc;
		printk("init-fg: reset dsoc to rsoc(%d),s2r=%d", battery->rsoc, battery->s2r);
	}
}

static int rk817_bat_parse_dt(struct rk817_battery_device *battery)
{
	u32 out_value;
	int length, ret;
	size_t size;
	struct battery_platform_data *pdata;
	struct device *dev = battery->dev;
	struct device_node *np = battery->dev->of_node;
	enum of_gpio_flags flags;

	pdata = devm_kzalloc(battery->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	battery->pdata = pdata;
	/* init default param */
	pdata->bat_res = DEFAULT_BAT_RES;
	pdata->monitor_sec = DEFAULT_MONITOR_SEC;
	pdata->pwroff_vol = DEFAULT_PWROFF_VOL_THRESD;
	pdata->sleep_exit_current = DEFAULT_SLP_EXIT_CUR;
	pdata->sleep_enter_current = DEFAULT_SLP_ENTER_CUR;

	pdata->sleep_filter_current = DEFAULT_SLP_FILTER_CUR;
	pdata->bat_mode = MODE_BATTARY;
	pdata->max_soc_offset = DEFAULT_MAX_SOC_OFFSET;
	pdata->fb_temp = DEFAULT_FB_TEMP;
	pdata->energy_mode = DEFAULT_ENERGY_MODE;
	pdata->zero_reserve_dsoc = DEFAULT_ZERO_RESERVE_DSOC * 1000;

	pdata->sample_res = DEFAULT_SAMPLE_RES;

	/* parse necessary param */
	if (!of_find_property(np, "ocv_table", &length)) {
		dev_err(dev, "ocv_table not found!\n");
		return -EINVAL;
	}

	pdata->ocv_size = length / sizeof(u32);
	if (pdata->ocv_size <= 0) {
		dev_err(dev, "invalid ocv table\n");
		return -EINVAL;
	}

	size = sizeof(*pdata->ocv_table) * pdata->ocv_size;
	pdata->ocv_table = devm_kzalloc(battery->dev, size, GFP_KERNEL);
	if (!pdata->ocv_table)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "ocv_table", pdata->ocv_table,
					 pdata->ocv_size);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "design_capacity", &out_value);
	if (ret < 0) {
		dev_err(dev, "design_capacity not found!\n");
		return ret;
	}
	pdata->design_capacity = out_value;

	ret = of_property_read_u32(np, "design_qmax", &out_value);
	if (ret < 0) {
		dev_err(dev, "design_qmax not found!\n");
		return ret;
	}
	pdata->design_qmax = out_value;

	/* parse unnecessary param */
	ret = of_property_read_u32(np, "sample_res", &pdata->sample_res);
	if (ret < 0)
		dev_err(dev, "sample_res missing!\n");

	ret = of_property_read_u32(np, "fb_temperature", &pdata->fb_temp);
	if (ret < 0)
		dev_err(dev, "fb_temperature missing!\n");

	ret = of_property_read_u32(np, "energy_mode", &pdata->energy_mode);
	if (ret < 0)
		dev_err(dev, "energy_mode missing!\n");

	ret = of_property_read_u32(np, "max_soc_offset",
				   &pdata->max_soc_offset);
	if (ret < 0)
		dev_err(dev, "max_soc_offset missing!\n");

	ret = of_property_read_u32(np, "monitor_sec", &pdata->monitor_sec);
	if (ret < 0)
		dev_err(dev, "monitor_sec missing!\n");

	ret = of_property_read_u32(np, "zero_algorithm_vol",
				   &pdata->zero_algorithm_vol);
	if (ret < 0)
		dev_err(dev, "zero_algorithm_vol missing!\n");

	ret = of_property_read_u32(np, "zero_reserve_dsoc",
				   &pdata->zero_reserve_dsoc);
	if (ret < 0)
		dev_err(dev, "zero_reserve_dsoc missing!\n");
	pdata->zero_reserve_dsoc *= 1000;

	ret = of_property_read_u32(np, "virtual_power", &pdata->bat_mode);
	if (ret < 0)
		dev_err(dev, "virtual_power missing!\n");

	ret = of_property_read_u32(np, "bat_res", &pdata->bat_res);
	if (ret < 0)
		dev_err(dev, "bat_res missing!\n");

	ret = of_property_read_u32(np, "sleep_enter_current",
				   &pdata->sleep_enter_current);
	if (ret < 0)
		dev_err(dev, "sleep_enter_current missing!\n");

	ret = of_property_read_u32(np, "sleep_exit_current",
				   &pdata->sleep_exit_current);
	if (ret < 0)
		dev_err(dev, "sleep_exit_current missing!\n");

	ret = of_property_read_u32(np, "sleep_filter_current",
				   &pdata->sleep_filter_current);
	if (ret < 0)
		dev_err(dev, "sleep_filter_current missing!\n");

    // 20210225: 目前项目里面定义的都是3.5V,其实 v_sys >= 3.3V系统完全可以正常工作。定义为 3300也是可以的。
    // 后面有项目需求的时候可以修改过来.
	ret = of_property_read_u32(np, "power_off_thresd", &pdata->pwroff_vol);
	if (ret < 0)
		dev_err(dev, "power_off_thresd missing!\n");

	if (battery->chip_id == RK809_ID) {
		ret = of_property_read_u32(np, "bat_res_up",
					   &pdata->bat_res_up);
		if (ret < 0)
			dev_err(dev, "battery res_up missing\n");

		ret = of_property_read_u32(np, "bat_res_down",
					   &pdata->bat_res_down);
		if (ret < 0)
			dev_err(dev, "battery res_down missing!\n");

		ret = of_property_read_u32(np, "design_max_voltage",
					   &pdata->design_max_voltage);
		if (ret < 0)
			dev_err(dev, "battery design_max_voltage missing!\n");

		ret = of_property_read_u32(np, "register_chg_psy",
					   &battery->is_register_chg_psy);
		if (ret < 0 || !battery->is_register_chg_psy)
			dev_err(dev, "not have to register chg psy!\n");
	}


	// 20180213,hsl add led ctrl
	battery->ledr_gpio = of_get_named_gpio_flags(np, "rled_gpios", 0, &flags);
	if (battery->ledr_gpio < 0 ){
		printk("%s	ledr_gpio error--that's ok\n", __func__);
	} else {
		battery->ledr_on_level = (flags & OF_GPIO_ACTIVE_LOW)?  0 : 1;
		ret = gpio_request(battery->ledr_gpio, "bat-rled");
		if( ret ){
			printk("%s:request gpio %d Failed!!!\n",__func__ , battery->ledr_gpio);
		} else {
			//20171019,if the uboot already reset,we don't need
			// here.
			gpio_direction_output(battery->ledr_gpio, !battery->ledr_on_level);
		}
	}

	battery->ledg_gpio = of_get_named_gpio_flags(np, "gled_gpios", 0, &flags);
	if (battery->ledg_gpio < 0 ){
		printk("%s	ledg_gpio error--that's ok\n", __func__);
	} else {
		battery->ledg_on_level = (flags & OF_GPIO_ACTIVE_LOW)?  0 : 1;
		ret = gpio_request(battery->ledg_gpio, "bat-gled");
		if( ret ){
			printk("%s:request gpio %d Failed!!!\n",__func__ , battery->ledg_gpio);
		} else {
			//20171019,if the uboot already reset,we don't need
			// here.
			gpio_direction_output(battery->ledg_gpio, !battery->ledg_on_level);
		}
	}

	// 20210412,增加 tri_led_table 的支持.
	if (!of_find_property(np, "tri-led", &length)) {
	    // 20220713: R2 反馈99%就显示绿灯，需要100%。此处修改。
	    // < 0: 显示红灯， > [1]: 显示绿灯； 两个之间，显示黄灯（红绿同时亮）。
		battery->tri_led_table[0] = 100;  // < 100% 红灯，即 0--99%
		battery->tri_led_table[1] = 99;   // >99% 绿灯，即 100%。
	} else {
	    ret = of_property_read_u32_array(np, "tri-led", battery->tri_led_table, length / sizeof(u32));
	    if (ret < 0) {
	        printk("%s:read tri-led array Failed(%d)!\n", __func__ , ret);
		    return ret;
		}
		// 20210412-LOG: rk817_bat_parse_dt:tri-led[10 95]
		//printk("%s:tri-led[%d %d]\n", __func__, battery->tri_led_table[0], battery->tri_led_table[1]);
		//battery->tri_led_table[0] = 1000;
		//battery->tri_led_table[1] *= 1000;
	}
	if (!of_find_property(np, "temperature_table", &length)) {
		battery->pdata->temp_t[0] = 0;
		dev_err(dev, "temperature_table not found!\n");
		//return -EINVAL;
	}else{
		ret = of_property_read_u32_array(np, "temperature_table", battery->pdata->temp_t, length / sizeof(int));
		if (ret < 0) {
			battery->pdata->temp_t[0] = 0;
	        printk("%s:read temperature_table array Failed(%d)!\n", __func__ , ret);
		    return ret;
		}else{
			printk("temperature_table:%d,%d,%d,%d\n", battery->pdata->temp_t[0], battery->pdata->temp_t[1], battery->pdata->temp_t[2], battery->pdata->temp_t[3]);
		}
	}

    // 20210227-LOG: 
    /*
[    0.846379] bat_res:100
[    0.846379] res_sample:10
[    0.846379] design_capacity:4000
[    0.846379] design_qmax :4200
[    0.846379] sleep_enter_current:300
[    0.846379] sleep_exit_current:300
[    0.846379] sleep_filter_current:100
[    0.846379] zero_algorithm_vol:3950
[    0.846379] zero_reserve_dsoc:10000000
[    0.846379] monitor_sec:10
[    0.846379] max_soc_offset:60
[    0.846379] virtual_power:0
[    0.846379] pwroff_vol:3300
    */
	DBG("DTS:"
	    "bat_res:%d,"
	    "res_sample:%d,"
	    "design_capacity:%d,"
	    "design_qmax :%d,"
	    "sleep_enter_current:%d,"
	    "sleep_exit_current:%d,"
	    "sleep_filter_current:%d,"
	    "zero_algorithm_vol:%d,"
	    "zero_reserve_dsoc:%d,"
	    "monitor_sec:%d,"
	    "max_soc_offset:%d,"
	    "virtual_power:%d,"
	    "pwroff_vol:%d\n",
	    pdata->bat_res,
	    pdata->sample_res,
	    pdata->design_capacity,
	    pdata->design_qmax,
	    pdata->sleep_enter_current,
	    pdata->sleep_exit_current,
	    pdata->sleep_filter_current,
	    pdata->zero_algorithm_vol,
	    pdata->zero_reserve_dsoc,
	    pdata->monitor_sec,
	    pdata->max_soc_offset,
	    pdata->bat_mode,
	    pdata->pwroff_vol);

	return 0;
}

static enum power_supply_property rk817_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

static int rk817_bat_get_usb_psy(struct device *dev, void *data)
{
	struct rk817_battery_device *battery = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_USB) {
		battery->usb_psy = psy;
		return 1;
	}

	return 0;
}

static int rk817_bat_get_ac_psy(struct device *dev, void *data)
{
	struct rk817_battery_device *battery = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS) {
		battery->ac_psy = psy;
		return 1;
	}

	return 0;
}

static void rk817_bat_get_chrg_psy(struct rk817_battery_device *battery)
{
	if (!battery->usb_psy)
		class_for_each_device(power_supply_class, NULL, (void *)battery,
				      rk817_bat_get_usb_psy);
	if (!battery->ac_psy)
		class_for_each_device(power_supply_class, NULL, (void *)battery,
				      rk817_bat_get_ac_psy);
}

static int rk817_bat_get_charge_state(struct rk817_battery_device *battery)
{
	union power_supply_propval val;
	int ret;
	struct power_supply *psy;

	if (!battery->usb_psy || !battery->ac_psy)
		rk817_bat_get_chrg_psy(battery);

	psy = battery->usb_psy;
	if (psy) {
		ret = psy->desc->get_property(psy, POWER_SUPPLY_PROP_ONLINE,
					      &val);
		if (!ret)
			battery->usb_in = val.intval;
	}

	psy = battery->ac_psy;
	if (psy) {
		ret = psy->desc->get_property(psy, POWER_SUPPLY_PROP_ONLINE,
					      &val);
		if (!ret)
			battery->ac_in = val.intval;
	}

	DBG("%s: ac_online=%d, usb_online=%d,vol=%d\n",
	    __func__, battery->ac_in, battery->usb_in, battery->voltage_avg);

	return (battery->usb_in || battery->ac_in);
}

static void rk817_bat_set_force_uncharge(struct rk817_battery_device *battery)
{
	union power_supply_propval val;
	struct power_supply *psy;

	if (!battery->usb_psy || !battery->ac_psy)
		return;
		
	psy = battery->usb_psy;
	if(psy->desc->set_property) {
	    val.intval = battery->force_uncharge; // set force-uncharge
	    psy->desc->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	}
	psy = battery->ac_psy;
	if(psy->desc->set_property) {
	    psy->desc->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	}
	
	DBG("%s: force_uncharge=%d\n",__func__, battery->force_uncharge);
}

// 20191229,hsl add. 20210222: 使用 bat_delay_work 可能会导致 bat_work 被多次频繁调用。我们使用
// 单独的 led_work.
static void rk817_update_battery_for_leds(struct rk817_battery_device *battery)
{
	//cancel_delayed_work(&battery->bat_delay_work);
	//queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work, msecs_to_jiffies(0));
	queue_delayed_work(battery->bat_monitor_wq, &battery->led_work, 0);
}


static int rk817_battery_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct rk817_battery_device *battery = power_supply_get_drvdata(psy);

	// 20191229:when plug/unplug,we will Got psp: 42/12/17/24/28/46/0/2;
	// printk("%s:psp=%d(STATUS=%d)\n", __func__, psp, POWER_SUPPLY_PROP_STATUS);
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battery->current_avg * 1000;/*uA*/
		if (battery->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_CURRENT * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery->voltage_avg * 1000;/*uV*/
		if (battery->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_VOLTAGE * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		
		if (battery->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_SOC;
		else {
		    val->intval = rk817_get_100percent_soc(battery, battery->dsoc); // (battery->dsoc  + 500) / 1000;
		    // if(val->intval > 100) val->intval = 100; // 20210222,hsl fix.
		}
		rk817_update_battery_for_leds(battery); // 20191229,hsl add.用于及时更新LED 的显示。即插拔DC/USB后能够马上更新。
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery->temperature;
		if (battery->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_TEMPERATURE;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (battery->pdata->bat_mode == MODE_VIRTUAL)
			val->intval = VIRTUAL_STATUS;
		else if (battery->dsoc >= 100 * 1000)  // 20210222: dsoc有可能会比 100*1000大一点。
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else {
		    if(battery->force_uncharge) {  // 20210415: 电池电压过低了，需要强制充电.
		        val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		    } else if ((battery->chip_id != RK809_ID) &&
			    rk817_bat_get_charge_state(battery))
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (battery->chip_id == RK809_ID &&
				 battery->plugin_trigger)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		DBG("get-PROP_STATUS:val=%d,foceuncharg=%d\n", val->intval, battery->force_uncharge);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = battery->charge_count;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery->pdata->design_capacity * 1000;/* uAh */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 4500 * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 5000 * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc rk817_bat_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= rk817_bat_props,
	.num_properties	= ARRAY_SIZE(rk817_bat_props),
	.get_property	= rk817_battery_get_property,
};

static int rk817_bat_init_power_supply(struct rk817_battery_device *battery)
{
	struct power_supply_config psy_cfg = { .drv_data = battery, };

	battery->bat = devm_power_supply_register(battery->dev,
						  &rk817_bat_desc,
						  &psy_cfg);
	if (IS_ERR(battery->bat)) {
		dev_err(battery->dev, "register bat power supply fail\n");
		return PTR_ERR(battery->bat);
	}

	return 0;
}

static enum power_supply_property rk809_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
};

static int rk809_chg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct rk817_battery_device *battery = power_supply_get_drvdata(psy);
	int online = 0;
	int ret = 0;

	if (battery->plugin_trigger)
		online = 1;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = online;
		dev_dbg(battery->dev, "report online: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (online)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		dev_dbg(battery->dev, "report prop: %d\n", val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct power_supply_desc rk809_chg_desc = {
	.name		= "charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= rk809_chg_props,
	.num_properties	= ARRAY_SIZE(rk809_chg_props),
	.get_property	= rk809_chg_get_property,
};

static int rk809_chg_init_power_supply(struct rk817_battery_device *battery)
{
	struct power_supply_config psy_cfg = { .drv_data = battery, };

	battery->chg_psy =
		devm_power_supply_register(battery->dev, &rk809_chg_desc,
					   &psy_cfg);
	if (IS_ERR(battery->chg_psy)) {
		dev_err(battery->dev, "register chg psy power supply fail\n");
		return PTR_ERR(battery->chg_psy);
	}

	return 0;
}

static void rk817_bat_power_supply_changed(struct rk817_battery_device *battery)
{
	static int old_soc = -1;

    int     soc100 = rk817_get_100percent_soc(battery, battery->dsoc); 
    
	/*if (battery->dsoc > 100 * 1000)
		battery->dsoc = 100 * 1000;
	else if (battery->dsoc < 0)
		battery->dsoc = 0;
    
	if (battery->dsoc == old_soc)
		return;
    */
    if (soc100 == old_soc && !battery->force_uncharge)
		return;
	old_soc = soc100; //battery->dsoc;
	//battery->last_dsoc = battery->dsoc;
	power_supply_changed(battery->bat);
	DBG("changed: dsoc=%d, rsoc=%d, v=%d, ov=%d c=%d, cap=%d, fcc=%d,funcharg=%d\n",
	    battery->dsoc, battery->rsoc, battery->voltage_avg,
	    battery->voltage_ocv, battery->current_avg,
	    battery->remain_cap, battery->real_fcc, battery->force_uncharge);

    /*
	DBG("dl=%d, rl=%d, v=%d, halt=%d, halt_n=%d, max=%d\n"
	    "init=%d, sw=%d, calib=%d, below0=%d, force=%d\n",
	    battery->dbg_pwr_dsoc, battery->dbg_pwr_rsoc,
	    battery->dbg_pwr_vol,
	    battery->is_halt, battery->halt_cnt,
	    battery->is_max_soc_offset,
	    battery->is_initialized, battery->is_sw_reset,
	    battery->is_ocv_calib,
	    battery->dbg_cap_low0, battery->is_force_calib);
	*/
}

static void rk817_battery_debug_info(struct rk817_battery_device *battery)
{
    // 20210227: 此处读取的cap 已经修改过来了: debug_info0:capactiy = 4199
    // DBG("debug_info0:capactiy = %d\n", rk817_bat_get_capacity_mah(battery));
	rk817_bat_get_battery_voltage(battery);
	rk817_bat_get_sys_voltage(battery);
	rk817_bat_get_USB_voltage(battery);
	rk817_bat_get_pwron_voltage(battery);
	rk817_bat_get_ocv_voltage(battery);
	rk817_bat_get_ocv0_voltage0(battery);

	rk817_bat_current_calibration(battery);
	rk817_bat_get_avg_current(battery);
	rk817_bat_get_relax_cur1(battery);
	rk817_bat_get_relax_cur2(battery);
	rk817_bat_get_relax_current(battery);
	rk817_bat_get_ocv_current(battery);
	rk817_bat_get_ocv_current0(battery);
	rk817_bat_get_pwron_current(battery);
	rk817_bat_get_ocv_count(battery);
	rk817_bat_save_dsoc(battery, battery->dsoc);
	DBG("debug_info1:capactiy = %d\n", rk817_bat_get_capacity_mah(battery));
}

static void
rk817_bat_update_charging_status(struct rk817_battery_device *battery)
{
	int is_charging;

	is_charging = rk817_bat_get_charge_state(battery);
	if (is_charging == battery->is_charging)
		return;

	battery->is_charging = is_charging;
	if (is_charging)
		battery->charge_count++;

    // 20210415: 充电状态发生改变的时候，我们清除这个标志,不管是变为充电还是变为不充电。
	battery->force_uncharge = false;
}

void rk817_bat_enable_charge(struct rk817_battery_device *battery)
{
	rk817_bat_field_write(battery, BAT_LTS_TS, 0xFA);
	power_supply_changed(battery->bat);
}

void rk817_bat_disable_charge(struct rk817_battery_device *battery)
{
	rk817_bat_field_write(battery, BAT_LTS_TS, 0x05);
	power_supply_changed(battery->bat);
}
static int charge_disable_flag = 0;
static int charge_low_flag = 0;
//static int charge_high_flag = 0;

static int rk817_bat_temperature_chrg(struct rk817_battery_device *battery, int temp)
{

// 小于0 大于60 不充电
// 0-10 45-60 低电流充电 低电压充电
// 10-45 正常充电
	if (battery->bat ==NULL) {
		dev_err(battery->dev, "register bat power supply fail\n");
		return 0;
	}

	if ((temp <= 0)||(temp >= 60)){
		if(charge_disable_flag == 0) {
			charge_disable_flag = 1;
			charge_low_flag = 0;
			rk817_bat_disable_charge(battery);
			printk("rk817_bat_temperature_chrg:rk817_bat_disable_charge \n");
		}
		return 0;
	}

	if ((temp > 3) && (temp < 59)) {
		if(charge_disable_flag == 1){
			charge_disable_flag = 0;
			charge_low_flag = 0;
			rk817_bat_enable_charge(battery);
			printk("rk817_bat_temperature_chrg:rk817_bat_enable_charge \n");
		}
	}

	if ((temp <= 13)||(temp >= 47)){
		if(charge_low_flag == 0) {
			charge_low_flag = 1;
			rk817_bat_field_write(battery, CHRG_CUR_SEL, CHRG_CUR_500MA);
			rk817_bat_field_write(battery, CHRG_VOL_SEL, CHRG_VOL_4200MV);
			printk("rk817_bat_temperature_chrg:current500,vol4200 \n");
		}
		return 0;
	}

	if ((temp >= 17)&&(temp <= 43)){ 
		if(charge_low_flag == 1) {
			charge_low_flag = 0;
			rk817_bat_field_write(battery, CHRG_CUR_SEL, CHRG_CUR_2000MA);
			rk817_bat_field_write(battery, CHRG_VOL_SEL, CHRG_VOL_4350MV);
			printk("rk817_bat_temperature_chrg:current2000,vol4350 \n");
		}
		return 0;
	}

	return 0;
	#if 0
	for (i = 0; i < battery->pdata->tc_count; i++) {
		up_temp = battery->pdata->tc_table[i].temp_up;
		down_temp = battery->pdata->tc_table[i].temp_down;
		cfg_current = battery->pdata->tc_table[i].chrg_current;

		if (now_temp >= down_temp && now_temp <= up_temp) {
			/* Temp range or charger are not update, return */
			if (config_index == i)
				return 0;

			config_index = i;

			if ((battery->pdata->tc_table[i].chrg_current != 0) &&
				(battery->pdata->tc_table[i].chrg_current_index != 0xff))
				rk817_bat_field_write(battery, CHRG_CUR_SEL,
					battery->pdata->tc_table[i].chrg_current_index);
			else
				rk817_bat_disable_charge(battery);
			#if 0
			if ((battery->pdata->tc_table[i].chrg_voltage != 0) &&
				(battery->pdata->tc_table[i].chrg_voltage_index != 0xff))
				rk817_bat_field_write(battery, CHRG_CUR_SEL,
					battery->pdata->tc_table[i].chrg_voltage_index);
			else
				rk817_bat_enable_charge(battery);
			#endif

			return 0;
		}
	}

	return 0;
	#endif
}

static void rk817_bat_update_info(struct rk817_battery_device *battery)
{
	int  capacity_uah;
	battery->voltage_avg = rk817_bat_get_battery_voltage(battery);
	battery->voltage_sys = rk817_bat_get_sys_voltage(battery);
	battery->current_avg = rk817_bat_get_avg_current(battery);
	battery->voltage_relax = rk817_bat_get_relax_voltage(battery);
	
	capacity_uah = rk817_bat_get_capacity_uah(battery);
	// 20211102: 我们按 10A 充放电的最大电流来计算。 10AH*1000*1000*10/3600 = 27778UAH.就是说两次采样之间的
	// 电量变化不能超过 27778uah. --20211102: 修改了 rk817_bat_init_coulomb_cap 里面的bug之后，应该不会出现
	// 写不进去的情况。由于 cap 是 u32,所以比较大小需要转为int。
	if(capacity_uah == 0 /*|| battery->remain_cap - capacity_uah > 28000*/) { 
		// 20211102: read error.这种情况下，我们不能更新remain_cap 和 rsoc,否则就会出现电量 0% 关机。
		// 我们尝试用 nac 重新更新 cap 试试。
		// rk817-bat: update_info: Error new capacity=3201608, remain_cap=3200000,reset coulomb!
		BAT_INFO("update_info: Error new capacity=%d, remain_cap=%d,reset coulomb!\n", 
			capacity_uah, battery->remain_cap);
		rk817_bat_init_coulomb_cap(battery, battery->remain_cap/1000);
		dump_stack();
	} else {
		battery->remain_cap = capacity_uah;
		battery->rsoc = rk817_bat_get_rsoc(battery);
	}
	//battery->remain_cap = rk817_bat_get_capacity_uah(battery);
	battery->voltage_usb = rk817_bat_get_USB_voltage(battery);

	battery->chrg_status = get_charge_status(battery);
	rk817_bat_update_charging_status(battery);
	if(battery->pdata->temp_t[0]!=0){
		battery->temperature = rk817_bat_get_temp(battery);//temperature;//
		rk817_bat_temperature_chrg(battery,battery->temperature);
	}
	// 20210221-LOG: rk817_bat_update_info:valtage usb: 4815 / valtage usb: 18 / 4823
	// --这个应该是USB插入的时候，USB 端测量到的电压.
	// DBG("rk817_bat_update_info:valtage usb: %d\n", battery->voltage_usb);

	// 20201209-PX30-LOG:  BAT_INFO  / DBG
	// rk817-bat: v_avg=4532,v_sys=4248,c_avg=-2,v_relx=4601,rsoc=99893,dsoc=99867,remain_cap=3995732,fcc=4000,chrg=0,c_pwon=28
	// v_avg=4259,v_sys=4244,c_avg=-63,v_relx=4601,rsoc=99867,dsoc=99867,remain_cap=3994700,fcc=4000,chrg=0,c_pwon=28
	// 20210221-LOG: v_avg=3896,v_sys=3913,c_avg=484,v_relx=0,rsoc=10130,dsoc=99775,
	// remain_cap=405232,fcc=4000,chrg=3,c_pwon=23
	// v_avg=4194,v_sys=4202,c_avg=494,v_relx=0,rsoc=100740,dsoc=99728,remain_cap=4029616,
	// fcc=4000,chrg=3,c_pwon=23  --这个是充满电的情况。
	// 20210903: v_avg=4311,c_avg=1392,rsoc=79191,dsoc=79389,remain_cap=2634008,use_fcc=2720,rv_fcc=480,chrg=3
	BAT_INFO("v_avg=%d,c_avg=%d,rsoc=%d,dsoc=%d,remain_cap=%d,use_fcc=%d,real_fcc=%d,chrg=%d,fc=%d,temperature=%d \n",
	    battery->voltage_avg,
	   // battery->voltage_sys,
	    battery->current_avg,
	    //battery->voltage_relax,
	    battery->rsoc, battery->dsoc,
	    battery->remain_cap, battery->max_used_fcc,
	    //battery->reserved_fcc,
	    battery->real_fcc,
	    battery->chrg_status,
	    battery->finish_chg_count,
	    battery->temperature);

	/*if(battery->resume_adjust) {
		battery->resume_adjust = false;
		if (rk817_bat_sleep_dischrg(battery)) {
			battery->sleep_dischrg_sec = 0;
		}
	}*/

#if 0
	/* smooth charge */
	if (battery->remain_cap / 1000 > battery->fcc) {
		/*battery->sm_remain_cap -=*/
		/*(battery->remain_cap - battery->fcc * 1000);*/
		battery->sm_remain_cap = battery->fcc * 1000;
		DBG("<%s>. cap: remain=%d, sm_remain=%d,fcc=%d\n",
		    __func__, battery->remain_cap, battery->sm_remain_cap, battery->fcc);
		//DBG("fcc: %d\n", battery->fcc);
		// rk817_bat_init_coulomb_cap(battery, battery->fcc + 100);
		rk817_bat_init_coulomb_cap(battery, battery->fcc);
		rk817_bat_get_capacity_mah(battery);
	}

	if (battery->chrg_status != CHARGE_FINISH)
		battery->finish_base = get_boot_sec();
#else
#endif 
}

static void rk817_bat_save_data(struct rk817_battery_device *battery)
{
	rk817_bat_save_dsoc(battery, battery->dsoc);
	rk817_bat_save_cap(battery, battery->remain_cap / 1000);
	DBG("save_data: save dsoc=%d,remain_cap=%d\n", battery->dsoc, battery->remain_cap);
}

#if 0

static unsigned long base2sec(unsigned long x)
{
	if (x)
		return (get_boot_sec() > x) ? (get_boot_sec() - x) : 0;
	else
		return 0;
}

/*
 * cccv and finish switch all the time will cause dsoc freeze,
 * if so, do finish chrg, 100ma is less than min finish_ma.
 */
static bool rk817_bat_fake_finish_mode(struct rk817_battery_device *battery)
{
	if ((battery->rsoc == 100) &&
	    (rk817_bat_get_charge_status(battery) == CC_OR_CV_CHRG) &&
	    (abs(battery->current_avg) <= 100))
		return true;
	else
		return false;
}

/* high load: current < 0 with charger in.
 * System will not shutdown while dsoc=0% with charging state(ac_in),
 * which will cause over discharge, so oppose status before report states.
 */
static void rk817_bat_lowpwr_check(struct rk817_battery_device *battery)
{
	static u64 time;
	int pwr_off_thresd = battery->pdata->pwroff_vol;

	if (battery->current_avg < 0 && battery->voltage_avg < pwr_off_thresd) {
		if (!time)
			time = get_boot_sec();

		if ((base2sec(time) > MINUTE(1)) ||
		    (battery->voltage_avg <= pwr_off_thresd - 50)) {
			battery->fake_offline = 1;
			if (battery->voltage_avg <= pwr_off_thresd - 50)
				battery->dsoc -= 1000;
			DBG("low power, soc=%d, current=%d\n",
			    battery->dsoc, battery->current_avg);
		}
	} else {
		time = 0;
		battery->fake_offline = 0;
	}

	DBG("<%s>. t=%lu, dsoc=%d, current=%d, fake_offline=%d\n",
	    __func__, base2sec(time), battery->dsoc,
	    battery->current_avg, battery->fake_offline);
}

static void rk817_bat_calc_smooth_dischrg(struct rk817_battery_device *battery)
{
	int tmp_soc = 0;

	/* check new dsoc */
	if (battery->smooth_soc < 0)
		battery->smooth_soc = 0;

	tmp_soc = battery->smooth_soc / 1000;

	if (tmp_soc != battery->dsoc / 1000) {
		if (battery->smooth_soc > battery->dsoc)
			return;

		if (battery->smooth_soc + 1000 > battery->dsoc)
			battery->dsoc = battery->smooth_soc;
		else
			battery->dsoc -= 1000;

		if (battery->dsoc <= 0)
			battery->dsoc = 0;
	}
}

static void rk817_bat_smooth_algorithm(struct rk817_battery_device *battery)
{
	int ydsoc = 0, delta_cap = 0, old_cap = 0, tmp_soc;
	/*int linek;*/
	int diff, delta;
	/*int current_avg = rk817_bat_get_avg_current(battery);*/

	delta = abs(battery->dsoc - battery->rsoc);
	diff = delta * 3;/* speed:3/4 */

	/* charge and discharge switch */
	if ((battery->sm_linek * battery->current_avg <= 0)) {
		DBG("<%s>. linek mode, retinit sm linek..\n", __func__);
		rk817_bat_calc_sm_linek(battery);
	}

	/*battery->sm_linek = linek;*/

	battery->remain_cap = rk817_bat_get_capacity_uah(battery);

	old_cap = battery->sm_remain_cap;
	DBG("smooth: smooth_soc = %d, dsoc = %d, battery->sm_linek = %d\n",
	    battery->smooth_soc, battery->dsoc, battery->sm_linek);

	/* discharge status: sm_remain_cap > remain_cap, delta_cap > 0 */
	/* from charge to discharge:
	 * remain_cap may be above sm_remain_cap, delta_cap <= 0
	 */
	delta_cap = battery->remain_cap - battery->sm_remain_cap;
	DBG("smooth: sm_remain_cap = %d, remain_cap = %d\n",
	    battery->sm_remain_cap, battery->remain_cap);
	DBG("smooth: delta_cap = %d, dsoc = %d\n",
	    delta_cap, battery->dsoc);

	if (delta_cap == 0) {
		DBG("<%s>. delta_cap = 0\n", __func__);
		return;
	}

	/* discharge: sm_linek < 0, if delate_cap <0, ydsoc > 0 */
	ydsoc = battery->sm_linek * abs(delta_cap / DIV(battery->fcc)) / 10;

	DBG("smooth: ydsoc = %d, fcc = %d\n", ydsoc, battery->fcc);
	if (ydsoc == 0) {
		DBG("<%s>. ydsoc = 0\n", __func__);
		return;
	}
	battery->sm_remain_cap = battery->remain_cap;

	DBG("<%s>. k=%d, ydsoc=%d; cap:old=%d, new:%d; delta_cap=%d\n",
	    __func__, battery->sm_linek, ydsoc, old_cap,
	    battery->sm_remain_cap, delta_cap);

	/* discharge mode */
	/* discharge mode, but ydsoc > 0,
	 * from charge status to dischrage
	 */
	battery->smooth_soc += ydsoc;
	if (ydsoc < 0) {
		rk817_bat_calc_smooth_dischrg(battery);
	} else {
		if (battery->smooth_soc < 0)
			battery->smooth_soc = 0;

		tmp_soc = battery->smooth_soc / 1000;

		if (tmp_soc != battery->dsoc / 1000) {
			if (battery->smooth_soc < battery->dsoc)
				return;

			battery->dsoc = battery->smooth_soc;
			if (battery->dsoc <= 0)
				battery->dsoc = 0;
		}
	}

	if (battery->s2r) {
		battery->s2r = false;
		rk817_bat_calc_sm_linek(battery);
	}

	DBG("smooth: smooth_soc = %d, dsoc = %d\n",
	    battery->smooth_soc, battery->dsoc);
	DBG("smooth: delta_cap = %d, dsoc = %d\n",
	    delta_cap, battery->dsoc);
}

static void rk817_bat_calc_zero_linek(struct rk817_battery_device *battery)
{
	int dead_voltage, ocv_voltage;
	int voltage_avg, current_avg, vsys;
	int ocv_cap, dead_cap, xsoc;
	int ocv_soc, dead_soc;
	int pwroff_vol;
	int min_gap_xsoc;
	int powerpatch_res;

	if ((abs(battery->current_avg) < 400) && (battery->dsoc / 1000 > 5))
		pwroff_vol = battery->pdata->pwroff_vol + 50;
	else
		pwroff_vol = battery->pdata->pwroff_vol;

	/* calc estimate ocv voltage */
	voltage_avg = rk817_bat_get_battery_voltage(battery);
	current_avg = rk817_bat_get_avg_current(battery);
	vsys = voltage_avg + (current_avg * DEF_PWRPATH_RES) / 1000;

	powerpatch_res = (voltage_avg - vsys) * 1000 / current_avg;

	battery->zero_voltage_avg = voltage_avg;
	battery->zero_current_avg = current_avg;
	battery->zero_vsys = vsys;

	DBG("Zero: voltage_avg = %d, Vsys = %d\n", voltage_avg, vsys);
	DBG("Zero: powerpatch_res = %d\n", powerpatch_res);
	DBG("ZERO0: shtd_vol: poweroff_vol(usr) = %d\n"
	    "pwroff_vol = %d\n"
	    "zero_reserve_dsoc = %d\n",
	    battery->pdata->pwroff_vol,
	    pwroff_vol,
	    battery->pdata->zero_reserve_dsoc);

	/* get the dead ocv voltage, pwroff_vol is vsys */
	dead_voltage = pwroff_vol - current_avg *
				(battery->bat_res + DEF_PWRPATH_RES) / 1000;

	ocv_voltage = voltage_avg - (current_avg * battery->bat_res) / 1000;
	DBG("ZERO0: dead_voltage(shtd) = %d, ocv_voltage(now) = %d\n",
	    dead_voltage, ocv_voltage);

	/* calc estimate soc and cap */
	dead_soc = rk817_bat_vol_to_soc(battery, dead_voltage);
	dead_cap = rk817_bat_vol_to_cap(battery, dead_voltage);
	DBG("ZERO0: dead_soc = %d, dead_cap = %d\n",
	    dead_soc, dead_cap);

	ocv_soc = rk817_bat_vol_to_soc(battery, ocv_voltage);
	ocv_cap = rk817_bat_vol_to_cap(battery, ocv_voltage);
	DBG("ZERO0: ocv_soc = %d, ocv_cap = %d\n",
	    ocv_soc, ocv_cap);

	/* xsoc: available rsoc */
	xsoc = ocv_soc - dead_soc;

	battery->zero_dead_voltage = dead_voltage;
	battery->zero_dead_soc = dead_soc;
	battery->zero_dead_cap = dead_cap;

	battery->zero_batvol_to_ocv = ocv_voltage;
	battery->zero_batocv_to_soc = ocv_soc;
	battery->zero_batocv_to_cap = ocv_cap;

	battery->zero_xsoc = xsoc;

	DBG("Zero: xsoc = %d\n", xsoc);
	/* min_gap_xsoc: reserve xsoc */
	if (abs(current_avg) > ZERO_LOAD_LVL1)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC3;
	else if (abs(current_avg) > ZERO_LOAD_LVL2)
		min_gap_xsoc = MIN_ZERO_GAP_XSOC2;
	else
		min_gap_xsoc = MIN_ZERO_GAP_XSOC1;

	if ((xsoc <= 30) &&
	    (battery->dsoc >= battery->pdata->zero_reserve_dsoc))
		min_gap_xsoc = min_gap_xsoc + MIN_ZERO_GAP_CALIB;

	battery->zero_remain_cap = battery->remain_cap;
	battery->zero_timeout_cnt = 0;
	if ((battery->dsoc / 1000 <= 1) && (xsoc > 0)) {
		battery->zero_linek = 400;
		battery->zero_drop_sec = 0;
	} else if (xsoc >= 0) {
		battery->zero_drop_sec = 0;
		battery->zero_linek =
			(battery->zero_dsoc + xsoc / 2) / DIV(xsoc);
		/* battery energy mode to use up voltage */
		if ((battery->pdata->energy_mode) &&
		    (xsoc - battery->dsoc / 1000 >= MIN_ZERO_GAP_XSOC3) &&
		    (battery->dsoc  / 1000 <= 10) && (battery->zero_linek < 300)) {
			battery->zero_linek = 300;
			DBG("ZERO-new: zero_linek adjust step0...\n");
		/* reserve enough power yet, slow down any way */
		} else if ((xsoc - battery->dsoc / 1000 >= min_gap_xsoc) ||
			   ((xsoc - battery->dsoc / 1000 >= MIN_ZERO_GAP_XSOC2) &&
			    (battery->dsoc / 1000 <= 10) && (xsoc > 15))) {
			if (xsoc <= 20 &&
			    battery->dsoc / 1000 >= battery->pdata->zero_reserve_dsoc)
				battery->zero_linek = 1200;
			else if (xsoc - battery->dsoc / 1000 >= 2 * min_gap_xsoc)
				battery->zero_linek = 400;
			else if (xsoc - battery->dsoc / 1000 >= 3 + min_gap_xsoc)
				battery->zero_linek = 600;
			else
				battery->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step1...\n");
		/* control zero mode beginning enter */
		} else if ((battery->zero_linek > 1800) &&
			   (battery->dsoc / 1000 > 70)) {
			battery->zero_linek = 1800;
			DBG("ZERO-new: zero_linek adjust step2...\n");
		/* dsoc close to xsoc: it must reserve power */
		} else if ((battery->zero_linek > 1000) &&
			   (battery->zero_linek < 1200)) {
			battery->zero_linek = 1200;
			DBG("ZERO-new: zero_linek adjust step3...\n");
		/* dsoc[5~15], dsoc < xsoc */
		} else if ((battery->dsoc / 1000 <= 15 && battery->dsoc > 5) &&
			   (battery->zero_linek <= 1200)) {
			/* slow down */
			if ((xsoc - battery->dsoc / 1000) >= min_gap_xsoc)
				battery->zero_linek = 800;
			/* reserve power */
			else
				battery->zero_linek = 1200;
			DBG("ZERO-new: zero_linek adjust step4...\n");
		/* dsoc[5, 100], dsoc < xsoc */
		} else if ((battery->zero_linek < 1000) &&
			   (battery->dsoc / 1000 >= 5)) {
			if ((xsoc - battery->dsoc / 1000) < min_gap_xsoc) {
				/* reserve power */
				battery->zero_linek = 1200;
			} else {
				if (abs(battery->current_avg) > 500)/* heavy */
					battery->zero_linek = 900;
				else
					battery->zero_linek = 1000;
			}
			DBG("ZERO-new: zero_linek adjust step5...\n");
		/* dsoc[0~5], dsoc < xsoc */
		} else if ((battery->zero_linek < 1000) &&
			   (battery->dsoc  / 1000 <= 5)) {
			if ((xsoc - battery->dsoc / 1000) <= 3)
				battery->zero_linek = 1200;
			else
				battery->zero_linek = 800;
			DBG("ZERO-new: zero_linek adjust step6...\n");
		}
	} else {
		/* xsoc < 0 */
		battery->zero_linek = 1000;
		if (!battery->zero_drop_sec)
			battery->zero_drop_sec = get_boot_sec();
		if (base2sec(battery->zero_drop_sec) >= WAIT_DSOC_DROP_SEC) {
			DBG("ZERO0: t=%lu\n", base2sec(battery->zero_drop_sec));
			battery->zero_drop_sec = 0;
			battery->dsoc -= 1000;
			if (battery->dsoc < 0)
				battery->dsoc = 0;
			battery->zero_dsoc = battery->dsoc;
		}
	}

	if (voltage_avg < pwroff_vol - 70) {
		if (!battery->shtd_drop_sec)
			battery->shtd_drop_sec = get_boot_sec();
		if (base2sec(battery->shtd_drop_sec) > WAIT_SHTD_DROP_SEC) {
			DBG("voltage extreme low...soc:%d->0\n", battery->dsoc);
			battery->shtd_drop_sec = 0;
			battery->dsoc = 0;
		}
	} else {
		battery->shtd_drop_sec = 0;
	}

	DBG("Zero: zero_linek = %d\n", battery->zero_linek);
}

static void rk817_bat_zero_algo_prepare(struct rk817_battery_device *battery)
{
	int tmp_dsoc;

	tmp_dsoc = battery->zero_dsoc / 1000;

	if (tmp_dsoc != battery->smooth_soc / 1000)
		battery->zero_dsoc = battery->smooth_soc;

	DBG("zero_smooth: zero_dsoc = %d\n", battery->zero_dsoc);

	rk817_bat_calc_zero_linek(battery);
}

static void rk817_bat_calc_zero_algorithm(struct rk817_battery_device *battery)
{
	int tmp_soc;

	tmp_soc = battery->zero_dsoc / 1000;

	if (tmp_soc == battery->dsoc / 1000)
		return;

	if (battery->zero_dsoc > battery->dsoc)
		return;

	if (battery->zero_dsoc < battery->dsoc - 1000)
		battery->dsoc -= 1000;
	else
		battery->dsoc = battery->zero_dsoc;
}

static void rk817_bat_zero_algorithm(struct rk817_battery_device *battery)
{
	int delta_cap = 0, delta_soc = 0;

	battery->zero_timeout_cnt++;
	delta_cap = battery->zero_remain_cap - battery->remain_cap;
	delta_soc = battery->zero_linek * delta_cap / DIV(battery->fcc) / 10;

	DBG("zero algorithm start\n");
	DBG("DEAD: dead_voltage: %d\n"
	    "dead_soc: %d\n"
	    "dead_cap: %d\n"
	    "powoff_vol: %d\n",
	    battery->zero_dead_voltage,
	    battery->zero_dead_soc,
	    battery->zero_dead_cap,
	    battery->pdata->pwroff_vol);
	DBG("DEAD: bat_voltage: %d\n"
	    "bat_current: %d\n"
	    "batvol_to_ocv: %d\n"
	    "batocv_to_soc: %d\n"
	    "batocv_to_cap: %d\n",
	    battery->zero_voltage_avg,
	    battery->zero_current_avg,
	    battery->zero_batvol_to_ocv,
	    battery->zero_batocv_to_soc,
	    battery->zero_batocv_to_cap);
	DBG("DEAD: Xsoc: %d, zero_reserve_dsoc: %d\n",
	    battery->zero_xsoc, battery->pdata->zero_reserve_dsoc);
	DBG("CAP: zero_remain_cap = %d, remain_cap = %d\n",
	    battery->zero_remain_cap, battery->remain_cap);
	DBG("Zero: zero_delta_cap = %d, zero_link = %d, delta_soc = %d\n",
	    delta_cap, battery->zero_linek, delta_soc);
	DBG("zero algorithm end\n");

	if ((delta_soc >= MIN_ZERO_DSOC_ACCURACY) ||
	    (battery->zero_timeout_cnt > MIN_ZERO_OVERCNT) ||
	    (battery->zero_linek == 0)) {
		DBG("ZERO1:--------- enter calc -----------\n");
		battery->zero_timeout_cnt = 0;
		battery->zero_dsoc -= delta_soc;
		rk817_bat_calc_zero_algorithm(battery);
		DBG("Zero: dsoc: %d\n", battery->dsoc);
		rk817_bat_calc_zero_linek(battery);
	}
}

static void rk817_bat_finish_algorithm(struct rk817_battery_device *battery)
{
	unsigned long finish_sec, soc_sec;
	int plus_soc, finish_current, rest = 0;

	/* rsoc */
	if ((battery->remain_cap != battery->fcc) &&
	    (get_charge_status(battery) == CHARGE_FINISH)) {
		battery->age_adjust_cap +=
			(battery->fcc * 1000 - battery->remain_cap);
		rk817_bat_init_coulomb_cap(battery, battery->fcc);
		rk817_bat_get_capacity_mah(battery);
	}

	/* dsoc */
	if (battery->dsoc < 100 * 1000) {
		if (!battery->finish_base)
			battery->finish_base = get_boot_sec();

		finish_current = (battery->rsoc - battery->dsoc) > FINISH_MAX_SOC_DELAY ?
					FINISH_CHRG_CUR2 : FINISH_CHRG_CUR1;
		finish_sec = base2sec(battery->finish_base);

		soc_sec = battery->fcc * 3600 / 100 / DIV(finish_current);
		plus_soc = finish_sec / DIV(soc_sec);
		if (finish_sec > soc_sec) {
			rest = finish_sec % soc_sec;
			battery->dsoc += plus_soc * 1000;
			battery->finish_base = get_boot_sec();
			if (battery->finish_base > rest)
				battery->finish_base = get_boot_sec() - rest;
		}
		DBG("CHARGE_FINISH:dsoc<100,dsoc=%d\n"
		    "soc_time=%lu, sec_finish=%lu, plus_soc=%d, rest=%d\n",
		    battery->dsoc, soc_sec, finish_sec, plus_soc, rest);
		DBG("battery->age_adjust_cap = %d\n", battery->age_adjust_cap);
	}
}

static void rk817_bat_display_smooth(struct rk817_battery_device *battery)
{
	/* discharge: reinit "zero & smooth" algorithm to avoid handling dsoc */
	if (battery->s2r && !battery->sleep_chrg_online) {
		DBG("s2r: discharge, reset algorithm...\n");
		battery->s2r = false;
		rk817_bat_zero_algo_prepare(battery);
		rk817_bat_smooth_algo_prepare(battery);
		return;
	}

	if (battery->work_mode == MODE_FINISH) {
		DBG("step1: charge finish...\n");
		rk817_bat_finish_algorithm(battery);

		if ((get_charge_status(battery) != CHARGE_FINISH) &&
		    !rk817_bat_fake_finish_mode(battery)) {
			if ((battery->current_avg < 0) &&
			    (battery->voltage_avg < battery->pdata->zero_algorithm_vol)) {
				DBG("step1: change to zero mode...\n");
				rk817_bat_zero_algo_prepare(battery);
				battery->work_mode = MODE_ZERO;
			} else {
				DBG("step1: change to smooth mode...\n");
				rk817_bat_smooth_algo_prepare(battery);
				battery->work_mode = MODE_SMOOTH;
			}
		}
	} else if (battery->work_mode == MODE_ZERO) {
		DBG("step2: zero algorithm...\n");
		rk817_bat_zero_algorithm(battery);
		if ((battery->voltage_avg >=
		    battery->pdata->zero_algorithm_vol + 50) ||
		    (battery->current_avg >= 0)) {
			DBG("step2: change to smooth mode...\n");
			rk817_bat_smooth_algo_prepare(battery);
			battery->work_mode = MODE_SMOOTH;
		} else if ((get_charge_status(battery) == CHARGE_FINISH) ||
			   rk817_bat_fake_finish_mode(battery)) {
			DBG("step2: change to finish mode...\n");
			rk817_bat_finish_algo_prepare(battery);
			battery->work_mode = MODE_FINISH;
		}
	} else {
		DBG("step3: smooth algorithm...\n");
		rk817_bat_smooth_algorithm(battery);
		if ((battery->current_avg < 0) &&
		    (battery->voltage_avg <
		     battery->pdata->zero_algorithm_vol)) {
			DBG("step3: change to zero mode...\n");
			rk817_bat_zero_algo_prepare(battery);
			battery->work_mode = MODE_ZERO;
		} else if ((get_charge_status(battery) == CHARGE_FINISH) ||
			   rk817_bat_fake_finish_mode(battery)) {
			DBG("step3: change to finish mode...\n");
			rk817_bat_finish_algo_prepare(battery);
			battery->work_mode = MODE_FINISH;
		}
	}
}
#endif 

// 20210221: 我们定义自己的电量显示算法。主要支持开机，长时间休眠唤醒的电量修正。采用 RK 的 
// smooth_algorithm 不能解决这个问题，且显得太过复杂了。我们只需要关注 rsoc/dsoc 就可以了。
static void rk817_bat_display_soc_htfy(struct rk817_battery_device *battery)
{
    // 20210225: 用来统计平均电压，防止某一次电压低于 pwroff_vol 之后就进行了关机.
    static int volt_avg[4];
    static unsigned int volt_num = 0;

    // 20210227: 放电电流太大的时候，进行一定的补偿，否则后面容易出现低电压关机（其实电量还有10%但是就关机了).
    int cur_volt = battery->voltage_avg;
    if (battery->current_avg < -800) {
    		cur_volt += 80;     // 电池内阻大概是 100毫欧。 battery->bat_res
    } else if (battery->current_avg < -500) {
    		cur_volt += 50;     // 电池内阻大概是 100毫欧。
    }
    
    volt_avg[volt_num&0X3] = cur_volt; //battery->voltage_avg;
    volt_num++;
    //DBG("display_soc_htfy: rsoc=%d,dsoc=%d,s2r=%d,monitor_ms=%d\n", battery->rsoc,battery->dsoc, 
    //    battery->s2r, battery->monitor_ms);
    
    // 实际电量和显示电量一样，不需要处理. 由于精度是 0.001,所以两个值一样的可能性基本没有.
	//if( battery->rsoc == battery->dsoc ) return;

	//如果不一样，区分充电和非充电的状态??也不行。比如USB充电的情况下，电量也有可能是降低的，
	// 因为放电的电流大于充电的电流。所以这里只需要简单的让 dsoc 靠近 rsoc就可以了？
	// 20210221: battery->s2r 用来表示是否允许电量有突变。比如长时间待机以及开机.
	// 20211029: 增加充电和非充电状态判断。只有充电状态，电量才能增加.
	if(battery->s2r) {
	    battery->dsoc = battery->rsoc;
	    battery->s2r = false;
	} else if(battery->is_charging &&   // battery->current_avg > 0 
		(battery->dsoc < battery->rsoc - 500
	    || (battery->dsoc < battery->rsoc && battery->rsoc >= 100*1000))) {
	    // 20211029: 此处不能判断 充电电流 > 0，因为 CHARGE-FINISH 之后，current_avg 会小于0.
	    battery->dsoc += 1000;
	} else {
		// 20211029: 充电状态，电量也有可能减少，比如USB充电。所以此处我们判断电流。
		// 电流小于0表示放电(需要排除charge-finish 情况)
		if(battery->current_avg < -50 && CHARGE_FINISH != battery->chrg_status
			&& battery->dsoc >= battery->rsoc + 500) {
	    	battery->dsoc -= 1000;
	    }
	}

    // 20210222: 要限定一个最大最小值，以免出错.
	if(battery->dsoc > 100*1000) battery->dsoc = 100*1000;
	else if( battery->dsoc < 0) {
	    battery->dsoc = 0;
	} 

	// 20211103: 明明已经充电完成，但是这里判断 is_charging 返回的值却是0. 原因是DTS里面的 extcon = <&u2phy>;
	// 改为了 extcon = <&fusb0>; --TYPE-C 充电（可能需要真正的 TYPE-C 接口才行）。
	// display_soc_htfy:volt_avg[4572 4572 4572 4572],pwroff_vol=3500,dsoc=99101,rsoc=99132,cur_avg=26,charging=0(4)
    DBG("soc_htfy:volt_avg[%d %d %d %d],pwroff_vol=%d,dsoc=%d,rsoc=%d,cur_avg=%d,charging=%d(%d)\n", volt_avg[0], volt_avg[1], 
        volt_avg[2], volt_avg[3], battery->pdata->pwroff_vol, battery->dsoc, 
        battery->rsoc, battery->current_avg, battery->is_charging, battery->chrg_status);

    // 20210416:此处代码：1. 需要防止电池电压过低但是系统不会自动关机，有两种情况：a. 此时电量是0，但是插着充电器；b.
    // 此时电压很低了，但是电量还比较高。 此处增加一个 10% 电量的判断，是防止当短时间系统电流比较大，导致电压偏低，
    // 然后系统就关机了，实际电量还有不少。我们现在设置的 pwroff 电压是 3500/3550.算是比较高的。由于上面增加了 80/50 mv
    // 的调整，此处可以改为 5%. --20210525: 去除 dsoc < 5 的判断。原因是如果 dsoc 是通过 ocv 来调整出来的，有可能
    // 偏差很大。此时电压已经低至 3.5v,但是 dsoc 还有 31%。这种情况下，必须上报电量 0% 关机。
	if( cur_volt < battery->pdata->pwroff_vol /*&& battery->dsoc < 5000 */&& volt_num > 3 ) {
	    // 20210415: 插着USB的情况下播放视频，会出现下面的LOG: 可惜上面的 else if 语句做了过滤。
	    // v_avg=2998,v_sys=3054,c_avg=-879,v_relx=3603,rsoc=-13397,dsoc=0,remain_cap=144480,use_fcc=3400,rv_fcc=600,chrg=3
	    int pwroff_avg = (volt_avg[0] + volt_avg[1] + volt_avg[2] + volt_avg[3]) >> 2;
    	
    	if (pwroff_avg < (battery->pdata->pwroff_vol)/*pwroff_vol*/) {
    	    // 20210225-LOG: display_soc_htfy: volt_avg[3503 3491 3491 3498],pwroff_vol=3500,dsoc=1146
    		battery->dsoc = 0;

    		// 20211025: 这种情况下，其实电池接近过放了，剩余电量应该没有到 reserved电量。我们按 
    		// 10% 来计算。主要是拔掉电池又未进行充放电之后会出现这种情况。 20211025: pwroff_vol 的
    		// 值不一样，剩余的电量就不一样，此处怎么处理呢？ 
    		//battery->remain_cap = battery->reserved_fcc*1000;
    			//battery->real_fcc*(1000*EMPTY_FCC_PERCENT/100);

            rk817_bat_init_coulomb_cap(battery,/*battery->reserved_fcc*/
            	battery->real_fcc*EMPTY_FCC_PERCENT/100); // this will update the remain_cap.
            
            BAT_INFO("display_soc_htfx:set dsoc=0,remain cap=%d,pwroff_avg=%d,pwroff_vol=%d\n", battery->remain_cap,
                pwroff_avg, battery->pdata->pwroff_vol);
    		// 20210415: 如果在充电的情况下达到这个很低的电压，说明充电电流要比放电电流小（比如在USB充电
    		// 的情况下播放视频），由于充电情况下android不会自动关机，我们在此处需要强制关机，否则电池会过放).
    		// 比较好的处理办法就是上报电池电量是0，且充电状态是不充电。这样能否显示关机画面。
    		if(battery->is_charging && battery->current_avg < 0) {
    		    DBG("display_soc_htfx:force shutdown!pwroff_avg=%d, pwroff_vol=%d\n", pwroff_avg, battery->pdata->pwroff_vol);
    		    battery->force_uncharge = true;
    		    rk817_bat_set_force_uncharge(battery);
    		}
    	}
	}
}

static void rk817_bat_output_info(struct rk817_battery_device *battery)
{
    // 202102211-LOG: info: voltage_k = 70 / info: voltage_b = 14 
    // info: voltage = 3896 / info: voltage_sys = 3913 / remain_cap = 405232
    // sm_remain_cap = 405000 / sm_link = 750 / smooth_soc = 99775 
    // zero_remain_cap = 0 / zero_link = 0 / zero_dsoc = 0 
    // dsoc = 99775, dsoc/1000 = 99 / rsoc = 10130
	DBG("info start:\n");
	DBG("info: voltage_k = %d\n", battery->voltage_k);
	DBG("info: voltage_b = %d\n", battery->voltage_b);
	DBG("info: voltage = %d\n", battery->voltage_avg);
	DBG("info: voltage_sys = %d\n", battery->voltage_sys);
	DBG("info: current = %d\n", battery->current_avg);

	DBG("info: FCC = %d\n", battery->fcc);
	DBG("info: remain_cap = %d\n", battery->remain_cap);
	DBG("info: sm_remain_cap = %d\n", battery->sm_remain_cap);
	DBG("info: sm_link = %d\n", battery->sm_linek);
	DBG("info: smooth_soc = %d\n", battery->smooth_soc);

	DBG("info: zero_remain_cap = %d\n", battery->zero_remain_cap);
	DBG("info: zero_link = %d\n", battery->zero_linek);
	DBG("info: zero_dsoc = %d\n", battery->zero_dsoc);

	//DBG("info: remain_cap = %d\n", battery->remain_cap);
	DBG("info: dsoc = %d, dsoc/1000 = %d\n",
	    battery->dsoc, battery->dsoc / 1000);
	DBG("info: rsoc = %d\n", battery->rsoc);
	//DBG("info END.\n");
}


#define CHG_LED_RED             1
#define CHG_LED_GREEN           2
#define CHG_LED_BLUE           	4

#define CHG_LED_ALL             (CHG_LED_RED|CHG_LED_GREEN)

// which_led:1 -- ledr , 2: ledg, 3: ledr & ledg.
#ifdef CONFIG_LEDS_AW2013
extern void aw2013_ht_set_brightness(int color,enum led_brightness brightnessr,enum led_brightness brightnessg,enum led_brightness brightnessb);
//extern void aw2013_ht_led_blink_set(int color, unsigned long blinking);
#else
static void rk817_bat_set_led(struct rk817_battery_device *pdata,
    int which_led, bool on)
{
    if( which_led & CHG_LED_RED ){
        if( gpio_is_valid(pdata->ledr_gpio) ){
            gpio_set_value(pdata->ledr_gpio,
                on?pdata->ledr_on_level:(!pdata->ledr_on_level));
        }
    }
    if( which_led & CHG_LED_GREEN ){
        if( gpio_is_valid(pdata->ledg_gpio) ){
            gpio_set_value(pdata->ledg_gpio,
                on?pdata->ledg_on_level:(!pdata->ledg_on_level));
        }
    }
}
#endif
static void rk817_update_leds(struct rk817_battery_device *bat)
{
    // 20210412-LOG: rk817_update_leds: soc=38605,charging=1,cust_ctrl_led=0
	int dsoc_percent =  rk817_get_100percent_soc(bat, bat->dsoc); 

	DBG("rk817_update_leds: soc=%d,charging=%d,cust_ctrl_led=%d\n", bat->dsoc, bat->is_charging,
		bat->cust_ctrl_led);
	if( bat->is_charging ) {
	    // 20210412: 如果是FT 测试模式(FT 通过 sys 接口来控制 LED，则此处我们不进行设置)。
		if(!bat->cust_ctrl_led) {
		    // 20210412: 我们增加根据电量来控制 LED 的显示的功能. 
		    // dsoc 是 1000 为单位，不知道为什么要这样自己搞自己.
			#ifdef CONFIG_LEDS_AW2013
			if( dsoc_percent < bat->tri_led_table[0] ) {
		        // 20210412: <[0]: 亮红灯;
		        aw2013_ht_set_brightness(CHG_LED_RED,255,0,0);
				//aw2013_ht_set_brightness(CHG_LED_GREEN,0);
		    } else if( dsoc_percent < bat->tri_led_table[1] ) {  
		        // 20210727: <[1]: 亮黄灯;
				aw2013_ht_set_brightness(CHG_LED_RED|CHG_LED_GREEN,255,200,0);
				//aw2013_ht_set_brightness(CHG_LED_GREEN,200);
		    } else if( dsoc_percent < bat->tri_led_table[2] ) {  
		        // 20211027: <[2]: 亮黄灯;
				aw2013_ht_set_brightness(CHG_LED_RED|CHG_LED_GREEN,255,200,0);
				//aw2013_ht_set_brightness(CHG_LED_GREEN,200);

		    } else{
				// 20210727 >[2]:亮绿灯
				aw2013_ht_set_brightness(CHG_LED_GREEN,0,255,0);
				//aw2013_ht_set_brightness(CHG_LED_RED,0);
				//aw2013_ht_set_brightness(CHG_LED_RED,0);
				//aw2013_ht_led_blink_set(CHG_LED_GREEN,1);
		    }
			#else
		    if( dsoc_percent < bat->tri_led_table[0] ) {
		        // 20210412: <[0]: 亮红灯;
		        rk817_bat_set_led(bat, CHG_LED_GREEN, false);
		        rk817_bat_set_led(bat, CHG_LED_RED, true);
		    } else if( dsoc_percent > bat->tri_led_table[1] ) {  
		        // 20210412: >[1]: 亮绿灯;
		        rk817_bat_set_led(bat, CHG_LED_GREEN,true);
		        rk817_bat_set_led(bat, CHG_LED_RED,false);
		    } else {
		        // 20210412: [0] - [1] 之间：亮橙色，也就是两个灯都亮。
		        rk817_bat_set_led(bat, CHG_LED_GREEN, true);
		        rk817_bat_set_led(bat, CHG_LED_RED, true);
		    }
			#endif
	    }

		// 20191230:don't sleep if charging!!
		if(!bat->charging_sleep){
	    	wake_lock(&bat->wake_lock);
	    }
    } else {
    	if(!bat->cust_ctrl_led) {
#ifdef CONFIG_LEDS_AW2013
			if( dsoc_percent < bat->tri_led_table[3] ) { 
				//aw2013_ht_set_brightness(CHG_LED_GREEN,0);
				//aw2013_ht_led_blink_set(CHG_LED_RED,1);
				//aw2013_ht_set_brightness(CHG_LED_GREEN,0);
				aw2013_ht_set_brightness(CHG_LED_RED,255,0,0);
			}else{
				//aw2013_ht_set_brightness(CHG_LED_RED,0);
				//aw2013_ht_set_brightness(CHG_LED_GREEN,0);
				aw2013_ht_set_brightness(0,0,0,0);
			}
#else
    		rk817_bat_set_led(bat, CHG_LED_ALL, false);
#endif
    	}

    	if(!bat->charging_sleep){
    		wake_unlock(&bat->wake_lock);
    	}
    }
}

static void rk817_notify_battery_change(struct rk817_battery_device *battery)
{
	if(battery->dsoc == battery->notify_dsoc &&
		battery->is_charging == battery->notify_charging )
		return;
	battery->notify_charging = battery->is_charging;
	battery->notify_dsoc = battery->dsoc;
	rk817_update_leds(battery);
}

static void rk817_update_real_fcc_from_remain_cap_and_save(
	struct rk817_battery_device *bat)
{
	// 4000x0.5%=20.
	bat->real_fcc = (bat->remain_cap/1000) - 20; // 为了保证显示100%，预留 0.5%
	rk817_bat_save_real_fcc_and_update_used_fcc(bat); 
}

// 20211022：是否充满电的充电电流判断标志(门槛)
#define CHARGE_FINISH_SWICH_CUR				120

static void rk817_bat_update_charge_finish(struct rk817_battery_device *battery)
{
	static int charge_status = 0; 
    static int update_remain_cap = 0;
    
	if( battery->chrg_status != charge_status ) {
	    charge_status = battery->chrg_status;
	    battery->low_current_chg_count = 0;
	    battery->finish_chg_count = 0;
	    battery->low_cap_chg_count = 0;
	}

	// 20211022: 我们会碰到异常的 CHARGE_FINISH,LOG 如下： 正常情况下， c_avg 应该是 -1 到 -4.此处需要判断是否是真正的charge-finish,且
	// 增加一些计数.
	// v_avg=4337,c_avg=156,rsoc=86934,dsoc=86962,remain_cap=3732572,use_fcc=3570,real_fcc=4199,chrg=4
	// v_avg=4278,c_avg=83,rsoc=-176,dsoc=76962,remain_cap=-562395296,use_fcc=3172687,real_fcc=3732572,chrg=4
	// v_avg=4603,c_avg=5,rsoc=100391,dsoc=100000,remain_cap=3376188,use_fcc=2861,real_fcc=3365,chrg=4  --USB充电FINISH的电流。
	// 20211102,在拔掉电池再接上，然后插入USB开机，会出现下面的LOG: 状态反馈 4,但是电量却很低。
	/*
	[   23.152690] rk817-bat: v_avg=4323,c_avg=-22,rsoc=0,dsoc=0,remain_cap=1032,use_fcc=0,real_fcc=0,chrg=4
	[   33.178692] rk817-bat: v_avg=4324,c_avg=6,rsoc=0,dsoc=0,remain_cap=860,use_fcc=0,real_fcc=0,chrg=4
	*/
	if(charge_status == CHARGE_FINISH){
		if(battery->current_avg < CHARGE_FINISH_SWICH_CUR && 

			// 20211102: 必须 > 80% 的 FCC.主要防止初始 cap 出错了，电池一直工作在低容量状态。比如
			// 实际容量是4000，结果 remain_cap 里面记录到 3000的时候就 CHARGE-FINISH 充不进去了，之后
			// 放电到3000，电池上报电量是0，导致一直有1000mah的电量无法使用。用户只有把电量放到真正的
			// reserve-fcc，才能完全使用这些电量。可能需要在 FT测试里面做一下校准？
			//sn 电量很多只有2300/2900。怎么都到不了80%降低点标准
#ifdef CONFIG_PROC_RATTA
			battery->remain_cap > battery->fcc*500 ) {
#else
			battery->remain_cap > battery->fcc*750 ) {
#endif
			battery->finish_chg_count++;
		} else {
			// 20211022-LOG: DC充电切换到USB充电，会偶然打印一下下面的信息：
			// charge_finish-A: v_avg=858,finish_chg_cnt=0
			// charge_finish-A: v_avg=27,finish_chg_cnt=0
			// charge_finish-A: v_avg=43,finish_chg_cnt=0
			BAT_INFO("charge_finish-A: big v_avg=%d,finish_chg_cnt=%d\n",
    		    battery->current_avg, battery->finish_chg_count);
		}
	} else if(battery->rsoc > 99*1000) {
		// 20211022: 如果充电状态没有到达 CHARGE_FINISH ，但是 通过 remain_cap 计算出来的 rsoc 已经到达
		// 100 了，怎么处理？这种情况可能:1. 原来的 remain_cap 经过了调整，比实际的高；2. 充电器有问题，
		// 一直没有上报 CHAGE_FINISH 的状态；3. 电池的实际容量比标称的 FCC 要搞。
		if(battery->current_avg > CHARGE_FINISH_SWICH_CUR) {
			// 20211022: 如果此时充电电流还很大，我们设置为 99%。等到 CHARGE-FINISH 的时候，我们会调整 real_fcc。
			battery->rsoc = 99*1000;
		} // 否则就让 rsoc 显示为 100%
	}
	
    // 20210416: 充电完成或者计算出来的电量已经大于 102%了，我们就需要调整 real_fcc/max_used_fcc,否则放电的时候
    // 就会导致前面 100% 的电量要放很长时间。
	if(/*charge_status == CHARGE_FINISH*/	// 20211022: oneloop 10sec
		battery->finish_chg_count > 12) {
        // 20210222: 如果是真正的充满了，我们必须让 remain_cap 接近 FCC，否则就会导致
        // 100%的电量变化到99%的时候时间很慢或者很快。如果已经充电完成，但是实际统计电量少于
        // FCC或者 > qmax,则进行调整. 充电一个晚上，会看到下面的LOG: (充电可休眠的时候)
        // charge_finish: remain cap=4700244, fcc=4000
        // --刚刚充满的时候，还是可以进行小电流充电的，此时 cap 还会继续增加。但是如果充满后很长一段时间（比如
        // 一个小时),cap 超出了范围，那么就需要调整. --CHARGE_FINISH的时候，最大可能还有200MA的充电.此处判断充电电流就可以了.
        // 看LOG: 当还是 CC or CV charge. 的时候，电流可以达到 300,但是出现 charge finish... 之后，电流就变为个位数。
        // 20211015: 实际上当充电状态是charg-finish的时候，充电电流经常是-1/-4，remain_cap 不可能再增加了，我们需要调整
        // real_fcc.保证电量达到 100%。尤其是电池使用了一段时间之后。
        // v_avg=4589,c_avg=-1,rsoc=95797,dsoc=95720,remain_cap=3085680,use_fcc=2720,real_fcc=3200,chrg=4
        // v_avg=4588,c_avg=-4,rsoc=95797,dsoc=95720,remain_cap=3085680,use_fcc=2720,real_fcc=3200,chrg=4
        #if 0
        if(battery->remain_cap < battery->real_fcc*1000 && battery->rsoc < 100*1000) {
        	// 20211103: 这种情况就是 原来的 real_fcc 的值太高了，导致现在充电一直充不满.
        	// 我们需要根据充电完成的条件重新调整 real_fcc，是指可以冲到 100%。
        	if(battery->remain_cap - update_remain_cap > 5000) {
            	BAT_INFO("charge_finish-X: remain cap=%d,s_remain_cap=%d,real_fcc=%d\n",
    		    	battery->remain_cap, update_remain_cap, battery->real_fcc);
    		    update_remain_cap = battery->remain_cap;
    		    rk817_update_real_fcc_from_remain_cap_and_save(battery);
    		}
        } else if (battery->remain_cap < battery->fcc*1000 ) { // 防止一直充电充不到 100%.
        	// 20211015-LOG: 
        	// charge_finish-0: remain cap=3086196, fcc=3200,s_remain_cap=0,real_fcc=3200
        	// charge_finish-0: remain cap=3085680, fcc=3200,s_remain_cap=3085680,real_fcc=3200
            if(battery->remain_cap - update_remain_cap > 5000) {
            	BAT_INFO("charge_finish-0: remain cap=%d, fcc=%d,s_remain_cap=%d,real_fcc=%d\n",
    		    	battery->remain_cap, battery->fcc, update_remain_cap, battery->real_fcc);
    		    update_remain_cap = battery->remain_cap;
    		    rk817_update_real_fcc_from_remain_cap_and_save(battery);
    		}
    		//rk817_bat_get_capacity_mah(battery);
    	} else if (battery->remain_cap > battery->fcc*1000 + 50000) {  //防止初始电量计里面内容不准确，导致统计电量偏大很多。
    	    //202010227-LOG: charge_finish-1: remain cap=4205744, fcc=4000,qmax=4200,s_remain_cap=4205056
    	    // 20210416: charge_finish-1: remain cap=4801724, fcc=4000,qmax=4200,s_remain_cap=0,real_fcc=4000
    	    /* 20211022-LOG:
[ 2453.077323] rk817-bat: v_avg=4601,c_avg=1,rsoc=101967,dsoc=100000,remain_cap=3365352,use_fcc=2814,real_fcc=3310,chrg=4
[ 2463.104273] rk817-bat: v_avg=4601,c_avg=1,rsoc=101967,dsoc=100000,remain_cap=3365352,use_fcc=2814,real_fcc=3310,chrg=4
[ 2463.104425] rk817-bat: charge_finish-1: remain cap=3365352, fcc=3200,qmax=3400,s_remain_cap=3310140,real_fcc=3310
[ 2463.105596] rk817-bat: save_real_fcc: real_fcc=3365,reserved_fcc=2861,max_used_fcc=504,save fcc=3365,fcc=3200,rsoc=100012
[ 2473.131876] rk817-bat: v_avg=4601,c_avg=1,rsoc=100012,dsoc=100000,remain_cap=3365352,use_fcc=2861,real_fcc=3365,chrg=4
[ 2473.132026] rk817-bat: charge_finish-1: remain cap=3365352, fcc=3200,qmax=3400,s_remain_cap=3365352,real_fcc=3365    	    
    	    */
    		// 20210227: init_coulomb_cap 修改的是 Q_INIT 寄存器，而 rk817_bat_get_capacity_uah 函数读取的是
    		// Q_PRESS 寄存器，这两个是不一样的。如果才能 同步呢？开机的时候会同步一下，不知道是什么操作可以触发。
    		if(battery->remain_cap - update_remain_cap > 5000) {
    			BAT_INFO("charge_finish-1: remain cap=%d, fcc=%d,qmax=%d,s_remain_cap=%d,real_fcc=%d\n",
    		    	battery->remain_cap, battery->fcc, battery->qmax, update_remain_cap, battery->real_fcc);
    		    update_remain_cap = battery->remain_cap;
    		    
    		    rk817_update_real_fcc_from_remain_cap_and_save(battery);
    		}
       	}
       	#else 
       	// 20211103: 在 charging-finish的情况下，让 rsoc 接近100%。 无论是大于还是小于都进行调整。
       	if( battery->rsoc > 102*1000 || battery->rsoc < 100*1000) {
        	// 20211103: 这种情况就是 原来的 real_fcc 的值太高了，导致现在充电一直充不满.
        	// 我们需要根据充电完成的条件重新调整 real_fcc，是指可以冲到 100%。
        	if(battery->remain_cap - update_remain_cap > 5000
        		|| battery->remain_cap - update_remain_cap < -5000) {
            	BAT_INFO("charge_finish-X: remain cap=%d,s_remain_cap=%d,real_fcc=%d,rsoc=%d\n",
    		    	battery->remain_cap, update_remain_cap, battery->real_fcc,  battery->rsoc);

    		    // 20211103: 这里还需要考虑一种情况，就是电池基本满电的情况下拔掉电池，重新初始化的
    		    // 时候写入电量异常，导致此时电池电量计里面的容量是接近0.充电一会后 CHARGE-FINISH，但是
    		    // 此时的 remain_cap 还是很少的。怎么处理呢？这种情况也有可能是很差的充电器误触发 CHARGE-FINISH.
    		    update_remain_cap = battery->remain_cap;
    		    rk817_update_real_fcc_from_remain_cap_and_save(battery);
    		}
        } 
       	#endif
    }else {
    	// 20210903: 此处处理 一直没有charge-finish的情况下，电池充电充不满的问题。主要从X1上面移植过来。
    	// 可能需要产品使用 一两年之后才会出现。
    	if(CC_OR_CV_CHRG == charge_status && (battery->ac_in || battery->dc_in) 
    		&& battery->current_avg < CHARGE_FINISH_SWICH_CUR ) {
    		battery->low_current_chg_count++;
    		
    		// 20210903: 10sec --oneloop. 5min.
    		if(battery->low_current_chg_count > 30 && battery->rsoc < 100*1000) {
    			BAT_INFO("charge_finish-2: remain cap=%d, fcc=%d,qmax=%d,s_remain_cap=%d,real_fcc=%d,rsoc=%d\n",
    		    	battery->remain_cap, battery->fcc, battery->qmax, update_remain_cap, battery->real_fcc, battery->rsoc);
    		    if(battery->remain_cap - update_remain_cap > 5000
        		|| battery->remain_cap - update_remain_cap < -5000) {
	    		    update_remain_cap = battery->remain_cap;
	    		    rk817_update_real_fcc_from_remain_cap_and_save(battery);
	    		}
    		}
    	} else if(charge_status == CHARGE_FINISH 
    		&& battery->current_avg < CHARGE_FINISH_SWICH_CUR
    		&& battery->voltage_avg > 4500) {
    		battery->low_cap_chg_count++;
    		
    		// 20210903: 10sec --oneloop. 5min.
    		if(battery->low_cap_chg_count > 30 && battery->rsoc < 100*1000) {
    			BAT_INFO("charge_finish-3: remain cap=%d, fcc=%d,qmax=%d,s_remain_cap=%d,real_fcc=%d,rsoc=%d\n",
    		    	battery->remain_cap, battery->fcc, battery->qmax, update_remain_cap, battery->real_fcc, battery->rsoc);
    		    if(battery->remain_cap - update_remain_cap > 5000
        		|| battery->remain_cap - update_remain_cap < -5000) {

        			// 20211103: 强制调增当前的电量是 FCC(init_coulomb_cap 会更新 remain_cap,保证同步)。
        			rk817_bat_init_coulomb_cap(battery, battery->real_fcc);
        			//battery->remain_cap = battery->real_fcc; 
	    		    update_remain_cap = battery->remain_cap;

	    		    // 20211103：这种情况下 real_fcc 不需要更新，更新的是电量。
	    		    //rk817_update_real_fcc_from_remain_cap_and_save(battery);
	    		}
    		}
    	}
    }
}


static void rk817_battery_work(struct work_struct *work)
{
	struct rk817_battery_device *battery =
		container_of(work,
			     struct rk817_battery_device,
			     bat_delay_work.work);

	rk817_bat_update_info(battery);
	rk817_bat_update_charge_finish(battery);
	//rk817_bat_lowpwr_check(battery);
	
	//rk817_bat_display_smooth(battery);
	rk817_bat_display_soc_htfy(battery);
	rk817_bat_save_data(battery);  // save before notify for shutdown.
	
	rk817_bat_power_supply_changed(battery);
	
	//rk817_bat_output_info(battery);
	rk817_notify_battery_change(battery);

	if (rk817_bat_field_read(battery, CUR_CALIB_UPD)) {
		rk817_bat_current_calibration(battery);
		rk817_bat_init_voltage_kb(battery);
		rk817_bat_field_write(battery, CUR_CALIB_UPD, 0x01);
	}

	queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work,
			   msecs_to_jiffies(battery->monitor_ms));

    // 20210222: battery_work-done: rsoc=24583,dsoc=23620,s2r=0,monitor_ms=10000 
    // 整个 work 运行时间大概是 8ms 左右(主界面待机的情况下)。
    //DBG("battery_work-done: rsoc=%d,dsoc=%d,remain_cap=%d,monitor_ms=%d\n", battery->rsoc,battery->dsoc, 
    //    battery->remain_cap, battery->monitor_ms);
}

static void rk817_battery_led_work(struct work_struct *work)
{
	struct rk817_battery_device *battery =
		container_of(work,
			     struct rk817_battery_device,
			     led_work.work);

	rk817_bat_update_charging_status(battery);
    rk817_update_leds(battery);
}

// 20201209: hsl add for FT.主要用于控制 LED/充电休眠.
// 20201214: 增加黑屏休眠后是否支持 alarmtimer 唤醒的设置。主要用于 FT和自动关机等场合。休眠时候
// RTC 唤醒会影响到待机时间。所以需要根据需要，由 PMS 上面动态配置。
// 20210414: 应用(设置)上面获取电池容量，通过这个接口。所以必须保证 fcc=%d 后面的数据就是电池容量。
static bool alarmtimer_rtc_wakeup_enable = false;
static bool wakelock_lock_when_charging = true;
bool charging_needs_wakelock(void) 
{
	return wakelock_lock_when_charging;
}


// 20210412: cat /sys/bus/platform/devices/rk817-battery/bat
static ssize_t bat_info_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rk817_battery_device *bat = dev_get_drvdata(dev);

    // 20210416-LOG: v_avg=4364,v_sys=4361,c_avg=229,v_relx=4601,chg_sleep=0,alarm_rtc=0
    // rsoc=124946,dsoc=100000,remain_cap=4848164,fcc=4000/4832684/600,chrg_status=3,v_pwroff=3400,tri-led=10/97
	return sprintf(buf,"v_avg=%d,v_sys=%d,c_avg=%d,v_relx=%d,chg_sleep=%d,alarm_rtc=%d,rsoc=%d,dsoc=%d,\n"
	    "remain_cap=%d,fcc=%d/%d/%d/%d,chrg_status=%d,v_pwroff=%d,tri-led=%d/%d\n",
	    bat->voltage_avg,
	    bat->voltage_sys,
	    bat->current_avg,
	    bat->voltage_relax, bat->charging_sleep,
	    alarmtimer_rtc_wakeup_enable,
	    bat->rsoc, bat->dsoc,
	    bat->remain_cap, bat->fcc, bat->real_fcc, 
	    bat->max_used_fcc, bat->reserved_fcc,
	    bat->chrg_status, bat->pdata->pwroff_vol,
	    bat->tri_led_table[0], bat->tri_led_table[1]
		);
}

static ssize_t bat_info_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	char cmd;
	struct rk817_battery_device *bat = dev_get_drvdata(dev);

	sscanf(buf, "%c", &cmd);
	BAT_INFO("Cmd(%c[%ld]):v_avg=%d,v_sys=%d,c_avg=%d,v_relx=%d,v_ocv=%d,chg_sleep=%d\n"
	    "rsoc = %d,dsoc=%d,remain_cap=%d,fcc=%d,chrg_status=%d,real_fcc=%d\n",
	    cmd, count, bat->voltage_avg,
	    bat->voltage_sys,
	    bat->current_avg,
	    bat->voltage_relax,
	    bat->voltage_ocv,
	    bat->charging_sleep,
	    bat->rsoc, bat->dsoc,
	    bat->remain_cap, bat->fcc,
	    bat->chrg_status,
	    bat->real_fcc); // rk817_bat_get_pwron_current(bat)

	// 20211104: 增加更新电池容量的接口。将来可以用于 FT 出厂校准电量.
	// echo 3300 > /sys/bus/platform/devices/rk817-battery/bat
	if(count > 3) {
		int set_cap = 0;
		int ret = sscanf(buf, "%d", &set_cap);
		if( ret != 1) {
			return -EINVAL;
		}
		printk("SetNewCap: new_cap=%d,remain_cap=%d,fcc=%d,qmax=%d\n", set_cap, 
			bat->remain_cap, bat->fcc, bat->qmax);
		if(set_cap < bat->fcc*9/10 || set_cap > bat->qmax) {
			printk("SetNewCap: Invaid set_cap(%d),must >= 90%%fcc and < qmax!\n", set_cap);
			return -EINVAL;
		}

		bat->s2r = true; // 20220301: allow update dsoc right-now.
		// remain_cap: uah, set_cap: mah.
		rk817_bat_init_coulomb_cap(bat, set_cap);
		rk817_update_real_fcc_from_remain_cap_and_save(bat);

		// copy from battery-work to save the data(to avoid lost if reboot now)
		rk817_bat_update_info(bat);
		rk817_bat_display_soc_htfy(bat);
		rk817_bat_save_data(bat);  // save before notify for shutdown.
		
		return count;
	}

	
	// 20190708,hsl add. sys file: /sys/bus/platform/devices/rk817-battery/bat
	// echo p > /sys/bus/platform/devices/rk817-battery/bat
	// echo s > /sys/bus/platform/devices/rk817-battery/bat
	// echo r > /sys/bus/platform/devices/rk817-battery/bat
	// echo l > /sys/bus/platform/devices/rk817-battery/bat
	if(cmd == 'l') {
	    // 20210605-LOG: CheckLowPower:dsoc=99976
		printk("CheckLowPower:dsoc=%d\n", bat->dsoc);
		if(bat->dsoc > 15000) {
			bat->dsoc = 12000;  // 触发一个低电告警,用于测试对话框的显示问题.
		} else if(bat->dsoc > 2000 ) {
			bat->dsoc -= 1000;
		}
		rk817_bat_power_supply_changed(bat);
	} else if (cmd == 'r') {  // led red on
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(CHG_LED_RED,255,0,0);
		#else
		rk817_bat_set_led(bat, CHG_LED_RED, true);
		#endif
	} else if (cmd == 'R') { // led red off
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(0,0,0,0);
		#else
		rk817_bat_set_led(bat, CHG_LED_RED, false);
		#endif
	} else if (cmd == 'g') { // led green on
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(CHG_LED_GREEN,0,255,0);
		#else
		rk817_bat_set_led(bat, CHG_LED_GREEN, true);
		#endif
	}  else if (cmd == 'G') { // led green off
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(0,0,0,0);
		#else
		rk817_bat_set_led(bat, CHG_LED_GREEN, false);
		#endif
	}else if (cmd == 'b') { // led green on
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(CHG_LED_BLUE,0,0,255);
		#endif
	}  else if (cmd == 'B') { // led green off
		bat->cust_ctrl_led = true;
		#ifdef CONFIG_LEDS_AW2013
		aw2013_ht_set_brightness(0,0,0,0);
		#endif
	} else if (cmd == 'x') { // don't ctrl the led.
		bat->cust_ctrl_led = false;
		rk817_update_leds(bat);
	} else if (cmd == 's') { // enable charging-sleep(use for FT).
		bat->charging_sleep = true;
		wakelock_lock_when_charging = false;	// 20210717:充电时允许休眠。
		if(wake_lock_active(&bat->wake_lock)) {
			wake_unlock(&bat->wake_lock);
		}
	} else if (cmd == 'S') {
		bat->charging_sleep = false; // 充电时不允许休眠。
		wakelock_lock_when_charging = true;
		if(!wake_lock_active(&bat->wake_lock)) {
			rk817_update_leds(bat);
		}
	} else if (cmd == 'a') { // enable charging-sleep(use for PMS).
		alarmtimer_rtc_wakeup_enable = true;
		BAT_INFO("set alarmtimer_rtc_wakeup_enable = true!\n");
	} else if (cmd == 'A') {
		alarmtimer_rtc_wakeup_enable = false;
		BAT_INFO("set alarmtimer_rtc_wakeup_enable = false!\n");
	} else {
		BAT_INFO("command error:%c\n", cmd);
	}
	return count;
}

bool alarmtimer_rtc_wakeup_enabled(void)
{
	return alarmtimer_rtc_wakeup_enable;
}


static struct device_attribute rk817_bat_attr[] = {
	__ATTR(bat, 0664, bat_info_show, bat_info_store),
};

static void rk818_bat_init_sysfs(struct rk817_battery_device *di)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rk817_bat_attr); i++) {
		ret = sysfs_create_file(&di->dev->kobj,
					&rk817_bat_attr[i].attr);
		if (ret)
			dev_err(di->dev, "create bat node(%s) error\n",
				rk817_bat_attr[i].attr.name);
	}
}


static irqreturn_t rk809_plug_in_isr(int irq, void *cg)
{
	struct rk817_battery_device *battery;
	battery = (struct rk817_battery_device *)cg;

	//printk("%s:charging=%d\n", __func__, battery->is_charging);
	battery->plugin_trigger = 1;
	battery->plugout_trigger = 0;
	power_supply_changed(battery->bat);
	if (battery->is_register_chg_psy)
		power_supply_changed(battery->chg_psy);

	// 20191229,for fast update leds. --don't have this irq.
	//queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work,
	//		   msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static irqreturn_t rk809_plug_out_isr(int irq, void *cg)
{
	struct rk817_battery_device *battery;

	battery = (struct rk817_battery_device *)cg;
	//printk("%s:charging=%d\n", __func__, battery->is_charging);
	battery->plugin_trigger = 0;
	battery->plugout_trigger = 1;
	power_supply_changed(battery->bat);
	if (battery->is_register_chg_psy)
		power_supply_changed(battery->chg_psy);

	//queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work,
	//		   msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static int rk809_charge_init_irqs(struct rk817_battery_device *battery)
{
	struct rk808 *rk817 = battery->rk817;
	struct platform_device *pdev = battery->pdev;
	int ret, plug_in_irq, plug_out_irq;

	battery->plugin_trigger = 0;
	battery->plugout_trigger = 0;

	plug_in_irq = regmap_irq_get_virq(rk817->irq_data, RK817_IRQ_PLUG_IN);
	if (plug_in_irq < 0) {
		dev_err(battery->dev, "plug_in_irq request failed!\n");
		return plug_in_irq;
	}

	plug_out_irq = regmap_irq_get_virq(rk817->irq_data, RK817_IRQ_PLUG_OUT);
	if (plug_out_irq < 0) {
		dev_err(battery->dev, "plug_out_irq request failed!\n");
		return plug_out_irq;
	}

	ret = devm_request_threaded_irq(battery->dev, plug_in_irq, NULL,
					rk809_plug_in_isr,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"rk817_plug_in", battery);
	if (ret) {
		dev_err(&pdev->dev, "plug_in_irq request failed!\n");
		return ret;
	}

	ret = devm_request_threaded_irq(battery->dev, plug_out_irq, NULL,
					rk809_plug_out_isr,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"rk817_plug_out", battery);
	if (ret) {
		dev_err(&pdev->dev, "plug_out_irq request failed!\n");
		return ret;
	}

	if (rk817_bat_field_read(battery, PLUG_IN_STS)) {
		battery->plugin_trigger = 1;
		battery->plugout_trigger = 0;
	}

	return 0;
}


extern bool fb_power_off( void );

// 20181215: Android 上层会启动很多RTC定时器，导致系统被频繁唤醒。休眠时间大大缩短。
// 我们此处通过修改 RTC ALARM ,屏蔽 android 上层的 RTC，同时可以保证低电关机.
#define RTCALARM_FOR_BATTERY

#ifdef RTCALARM_FOR_BATTERY
#include <linux/rtc.h>

#define DISCHRG_TIME_FORFTTEST		MINUTE(2)


static struct rtc_device *rtc_dev = NULL;
//bool 	enable_normal_rtc_alarm = 0; // 是否在休眠的时候，允许普通的RTC ALARM 继续起作用。
#define RTC_CLASS_NAME  "rtc0"

static struct alarm rk817_alarm;

static enum alarmtimer_restart	rk817_alarm_function(struct alarm *xalarm, ktime_t now)
{
	printk("rk817_alarm_function: type=%d,state=%d\n", rk817_alarm.type, rk817_alarm.state);
	return ALARMTIMER_NORESTART;
}


static void rk817_alarm_init( void )
{
	alarm_init(&rk817_alarm, ALARM_REALTIME, rk817_alarm_function);
	printk("rk817_alarm_init: type=%d,state=%d,time.fun=%ph\n", rk817_alarm.type,
		rk817_alarm.state, rk817_alarm.timer.function);
}

//extern int rtc_set_alarm_force(struct rtc_device *rtc, struct rtc_wkalrm *alarm);
static int set_rtc_alarm_by_battery_capacity(struct rk817_battery_device *di, struct rtc_wkalrm *tmp)
{
    time64_t   	rtc_time64;
    int        	ret;
    int 		sec;

    // 20211022: 这个是休眠时候的电流。应该还要根据 4G 模块是否关闭做调整。
	int 		c_sleep = SLP_CURR_MAX;
	
	int     	valid_cap =  (di->remain_cap- di->reserved_fcc*1000);
	
	// 20211022: 如果是亮屏情况下，还需要增加 TP/EMR 的耗电，平均大概是 15MA 左右。其实还有WIFI
	// 模块和4G模块静态下的功耗。4G的已经预算在 SLP_CURR_MAX 里面了。 
	if(!fb_power_off()){
		c_sleep += 15;
	}
    
    sec = ((valid_cap/1000)*3600)/(c_sleep);  // valid_cap的单位是 MAH*1000.
    // 20190610: 增加FT测试，5min唤醒系统做休眠测试. --20201214：我们通过命令来设置是否支持休眠的时候RTC
    // 唤醒系统（FT和自动关机功能需要）。
	if(fb_power_off() && sec < DISCHRG_TIME_STEP2 ) {
        sec = DISCHRG_TIME_STEP2;  // sleep 10min
    }

	// 20201212: 主要测试 alarm 接口是否可以正常工作。--alarm接口可以正常工作。原来设置无效，是因为
	// 在 alarmtimer_suspend 函数里面增加了 if(fb_power_off()) return 0;  的判断。把深休设置RTC的功能屏蔽了。
    // sec = 10;

    ret=rtc_read_time(rtc_dev, &tmp->time);
    if( ret < 0 ){
        return ret;
    }

    rtc_time64 = rtc_tm_to_time64(&tmp->time);
    rtc_time64 += sec;
    rtc_time64_to_tm(rtc_time64, &tmp->time);

    // 20191229: set_rtc_alarm_by_battery_capacity:RTC alarm:2020-01-29 12:36:27,remain_cap=3997796/4000,wakeup sec=2880000,en=0
    // BATTERY-RTC alarm:2020-02-09 15:27:58,remain_cap=3998828/4000,wakeup sec=3840000,soc=100  -- 大概是 44天。
    printk("BATTERY-RTC alarm:%4d-%02d-%02d %02d:%02d:%02d,remain_cap=%d/%d,wakeup sec=%d,c_sleep=%d,alarm_wake=%d\n",
        1900 + tmp->time.tm_year, tmp->time.tm_mon + 1, tmp->time.tm_mday,
        tmp->time.tm_hour, tmp->time.tm_min, tmp->time.tm_sec,
        di->remain_cap, di->design_cap, sec, 
        c_sleep, alarmtimer_rtc_wakeup_enabled());
    return ret;
}

static int set_rtc_alarm_for_battery(struct rk817_battery_device *di)
{
    int ret = 0;
	ktime_t alarm_expire;
    struct rtc_wkalrm tmp;

    if (!rtc_dev) {
        rtc_dev = rtc_class_open(RTC_CLASS_NAME);
        if (!rtc_dev) {
            printk("%s:open %s Failed!!\n", __func__, RTC_CLASS_NAME);
            return -ENODEV;
        }
    }
    ret = set_rtc_alarm_by_battery_capacity(di, &tmp);
    if( ret < 0 ){
        printk("%s: init rtc_wkalrm Failed,ret=%d\n", __func__, ret);
        return ret;
    }

	// 20201214: 如果 alarm 的唤醒功能是打开的，我们就用 alarm 的接口。否则需要直接
	// 设置 RTC-ALARM. 20211022：如果上层打开了定时关机我们需要定时唤醒系统。
	if(alarmtimer_rtc_wakeup_enabled() || !fb_power_off()) {
		alarm_try_to_cancel(&rk817_alarm);
		alarm_expire = rtc_tm_to_ktime(tmp.time);
		alarm_start(&rk817_alarm, alarm_expire);
	} else {
    	tmp.enabled = 1;
    	tmp.pending = 0;
    	ret = rtc_set_alarm(rtc_dev, &tmp);
    }

    return ret;
}

/*
static bool battery_is_rtc_wakeup(struct htfyun_adc_battery_data *bat)
{
    struct rtc_time tm;
    int ret;
    unsigned long now;
    if( bat->rtc_wkup_second == 0 || !rtc_dev ){
        return false;
    }
    ret=rtc_read_time(rtc_dev, &tm);
    if (ret)
    {
        printk("%s:error rtc_read_time\n",__func__);
        return false;
    }
    rtc_tm_to_time(&tm, &now);
    DBG("%s:now=%ld,rtc wakeup sec=%ld\n",__func__, now, bat->rtc_wkup_second);
    if( now >= bat->rtc_wkup_second-3 && now <= bat->rtc_wkup_second+3 ) {
        return true;
    }
    return false;
}
*/
static time_t rk817_get_rtc_sec(void)
{
	int err;
	struct rtc_time tm;
	struct timespec tv = { .tv_nsec = NSEC_PER_SEC >> 1, };
	//struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	time_t sec;

	if (!rtc_dev) {
        rtc_dev = rtc_class_open(RTC_CLASS_NAME);
        if (!rtc_dev) {
            printk("%s:open %s Failed!!\n", __func__, RTC_CLASS_NAME);
            return 0;
        }
    }

	err = rtc_read_time(rtc_dev, &tm);
	if (err) {
		dev_err(rtc_dev->dev.parent, "read hardware clk failed\n");
		return 0;
	}

	/*err = rtc_valid_tm(&tm);
	if (err) {
		dev_err(rtc->dev.parent, "invalid date time\n");
		return 0;
	}*/

	rtc_tm_to_time(&tm, &tv.tv_sec);
	sec = tv.tv_sec;

	return sec;
}

#else
#define set_rtc_alarm_for_battery(di)
#define rk817_get_rtc_sec()			0
#define rk817_alarm_init()
#endif

static int  rk817_bat_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk817_battery_device *battery = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&battery->bat_delay_work);

#if 0
	battery->s2r = false;
	battery->sleep_chrg_status = get_charge_status(battery);
	battery->current_avg = rk817_bat_get_avg_current(battery);
	if (battery->current_avg > 0 ||
	    (battery->sleep_chrg_status == CC_OR_CV_CHRG) ||
	    (battery->sleep_chrg_status == CHARGE_FINISH))
		battery->sleep_chrg_online = 1;
	else
		battery->sleep_chrg_online = 0;

	battery->remain_cap = rk817_bat_get_capacity_uah(battery);
	battery->rsoc = rk817_bat_get_rsoc(battery);

	battery->rtc_base = rk817_get_rtc_sec();
	rk817_bat_save_data(battery);

	if (battery->sleep_chrg_status != CHARGE_FINISH)
		battery->finish_base = get_boot_sec();

	if ((battery->work_mode == MODE_ZERO) &&
	    (battery->current_avg >= 0)) {
		DBG("suspend: MODE_ZERO exit...\n");
		/* it need't do prepare for mode finish and smooth, it will
		 * be done in display_smooth
		 */
		if (battery->sleep_chrg_status == CHARGE_FINISH) {
			battery->work_mode = MODE_FINISH;
			battery->finish_base = get_boot_sec();
		} else {
			battery->work_mode = MODE_SMOOTH;
			rk817_bat_smooth_algo_prepare(battery);
		}
	}

	DBG("suspend get_boot_sec: %lld\n", get_boot_sec());
#endif 
	DBG("suspend: dl=%d rl=%d c=%d v=%d cap=%d at=%ld ch=%d\n",
	    battery->dsoc, battery->rsoc, battery->current_avg,
	    rk817_bat_get_battery_voltage(battery),
	    //rk817_bat_get_capacity_uah(battery),
	    battery->remain_cap,
	    battery->sleep_dischrg_sec, battery->sleep_chrg_online);
	// DBG("battery->sleep_chrg_status=%d\n", battery->sleep_chrg_status);

	// 20191229,hsl add. 20211022：我们亮屏的情况也需要启动 ALARM 唤醒系统，因为有可能用户
	// 设置了30分钟或者永不休眠，此时电池的电量只剩下1-2%,长时间休眠可能会导致电池过放。
	if(true /*fb_power_off()*/) {
		rk817_bat_field_write(battery, FRAME_SMP_INTERV, 1);//tanlq add 200403
		set_rtc_alarm_for_battery(battery);
	}

	// 20211104: add for resume calc sleep_sec.
	battery->rtc_base = rk817_get_rtc_sec();
	return 0;
}

static int rk817_bat_rtc_sleep_sec(struct rk817_battery_device *battery)
{
	int interval_sec;

	interval_sec = rk817_get_rtc_sec() - battery->rtc_base;

	return (interval_sec > 0) ? interval_sec : 0;
}


#ifdef CONFIG_OF
static const struct of_device_id rk817_bat_of_match[] = {
	{ .compatible = "rk817,battery", },
	{ },
};
MODULE_DEVICE_TABLE(of, rk817_bat_of_match);
#else
static const struct of_device_id rk817_bat_of_match[] = {
	{ },
};
#endif

static int rk817_battery_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(rk817_bat_of_match, &pdev->dev);
	struct rk817_battery_device *battery;
	struct rk808 *rk817 = dev_get_drvdata(pdev->dev.parent);
	struct i2c_client *client = rk817->i2c;
	int i,  ret;

	if (!of_id) {
		dev_err(&pdev->dev, "Failed to find matching dt id\n");
		return -ENODEV;
	}

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -EINVAL;

	battery->rk817 = rk817;
	battery->client = client;
	battery->dev = &pdev->dev;
	platform_set_drvdata(pdev, battery);
	battery->chip_id = rk817->variant;

	battery->regmap = rk817->regmap;
	if (IS_ERR(battery->regmap)) {
		dev_err(battery->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rk817_battery_reg_fields); i++) {
		const struct reg_field *reg_fields = rk817_battery_reg_fields;

		battery->rmap_fields[i] =
			devm_regmap_field_alloc(battery->dev,
						battery->regmap,
						reg_fields[i]);
		if (IS_ERR(battery->rmap_fields[i])) {
			dev_err(battery->dev, "cannot allocate regmap field\n");
			return PTR_ERR(battery->rmap_fields[i]);
		}
	}

	ret = rk817_bat_parse_dt(battery);
	if (ret < 0) {
		dev_err(battery->dev, "battery parse dt failed!\n");
		return ret;
	}

	rk817_bat_init_info(battery);
	rk817_bat_init_fg(battery);
    
	rk817_battery_debug_info(battery);
	rk817_bat_update_info(battery);
	rk817_bat_output_info(battery);
	rk817_notify_battery_change(battery);

	battery->bat_monitor_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "rk817-bat-monitor-wq");
	INIT_DELAYED_WORK(&battery->bat_delay_work, rk817_battery_work);
	
	INIT_DELAYED_WORK(&battery->led_work, rk817_battery_led_work);
    
	ret = rk817_bat_init_power_supply(battery);
	if (ret) {
		dev_err(battery->dev, "rk817 power supply register failed!\n");
		return ret;
	}
	if (battery->is_register_chg_psy) {
		ret = rk809_chg_init_power_supply(battery);
		if (ret) {
			dev_err(battery->dev, "rk809 chg psy init failed!\n");
			return ret;
		}
	}

	if (battery->chip_id == RK809_ID)
		rk809_charge_init_irqs(battery);

	wake_lock_init(&battery->wake_lock, WAKE_LOCK_SUSPEND, "rk817_bat_lock");

	rk818_bat_init_sysfs(battery); // 20201209,hsl add.
	rk817_alarm_init();  // 20201212,hsl add.

	DBG("name: 0x%x", rk817_bat_field_read(battery, CHIP_NAME_H));
	DBG("%x\n", rk817_bat_field_read(battery, CHIP_NAME_L));
	DBG("driver version %s\n", DRIVER_VERSION);

	// 20210918: 此处提前调用电池更新函数，主要解决 recovery 上报电量是0的问题。在 recovery
	// 系统下，OTA升级前需要判断电池电量，此时开机时间大概是4.8秒。如果不能及时更新 dsoc，可能会
	// 出现上报的电量是0的问题。由于recovery里面只读取一次，就会导致OTA升级失败（电量太低不给升级）。
	queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work, 2*HZ);
	return 0;
}

static void rk817_battery_shutdown(struct platform_device *dev)
{
}


#ifdef CONFIG_PM_SLEEP


#if 0

static void rk817_bat_relife_age_flag(struct rk817_battery_device *battery)
{
	u8 ocv_soc, ocv_cap, soc_level;

	if (battery->voltage_relax <= 0)
		return;

	ocv_soc = rk817_bat_vol_to_soc(battery, battery->voltage_relax);
	ocv_cap = rk817_bat_vol_to_cap(battery, battery->voltage_relax);
	DBG("<%s>. ocv_soc=%d, min=%lu, vol=%d\n", __func__,
	    ocv_soc, battery->sleep_dischrg_sec / 60, battery->voltage_relax);

	/* sleep enough time and ocv_soc enough low */
	if (!battery->age_allow_update && ocv_soc <= 10) {
		battery->age_voltage = battery->voltage_relax;
		battery->age_ocv_cap = ocv_cap;
		battery->age_ocv_soc = ocv_soc;
		battery->age_adjust_cap = 0;

		if (ocv_soc <= 1)
			battery->age_level = 100;
		else if (ocv_soc < 5)
			battery->age_level = 90;
		else
			battery->age_level = 80;

		/*soc_level = rk818_bat_get_age_level(battery);*/
		soc_level = 0;
		if (soc_level > battery->age_level) {
			battery->age_allow_update = false;
		} else {
			battery->age_allow_update = true;
			battery->age_keep_sec = get_boot_sec();
		}

		BAT_INFO("resume: age_vol:%d, age_ocv_cap:%d, age_ocv_soc:%d, "
			 "soc_level:%d, age_allow_update:%d, "
			 "age_level:%d\n",
			 battery->age_voltage, battery->age_ocv_cap,
			 ocv_soc, soc_level,
			 battery->age_allow_update, battery->age_level);
	}
}



static void rk817_bat_init_capacity(struct rk817_battery_device *battery,
				    u32 cap)
{
	int delta_cap;

	delta_cap = cap - battery->remain_cap;
	if (!delta_cap)
		return;

	battery->age_adjust_cap += delta_cap;
	rk817_bat_init_coulomb_cap(battery, cap);
	rk817_bat_smooth_algo_prepare(battery);
	rk817_bat_zero_algo_prepare(battery);
}



static void rk817_bat_relax_vol_calib(struct rk817_battery_device *battery)
{
	int soc, cap, vol;

	vol = battery->voltage_relax;
	soc = rk817_bat_vol_to_soc(battery, vol);
	cap = rk817_bat_vol_to_cap(battery, vol);
	rk817_bat_init_capacity(battery, cap);
	BAT_INFO("sleep ocv calib: rsoc=%d, cap=%d\n", soc, cap);
}


static int rk817_bat_sleep_dischrg(struct rk817_battery_device *battery)
{
	bool ocv_soc_updated = false;
	int tgt_dsoc, gap_soc, sleep_soc = 0;
	int pwroff_vol = battery->pdata->pwroff_vol;
	unsigned long sleep_sec = battery->sleep_dischrg_sec;

	DBG("<%s>. enter: dsoc=%d, rsoc=%d, rv=%d, v=%d, sleep_min=%lu\n",
	    __func__, battery->dsoc, battery->rsoc, battery->voltage_relax,
	    battery->voltage_avg, sleep_sec / 60);

	if (battery->voltage_relax >= battery->voltage_avg) {
		rk817_bat_relax_vol_calib(battery);
		rk817_bat_restart_relax(battery);
		rk817_bat_relife_age_flag(battery);
		ocv_soc_updated = true;
	}

	/* handle dsoc */
	if (battery->dsoc <= battery->rsoc) {
		battery->sleep_sum_cap = (SLP_CURR_MIN * sleep_sec / 3600);
		sleep_soc = battery->sleep_sum_cap * 100 / DIV(battery->fcc);
		tgt_dsoc = battery->dsoc - sleep_soc * 1000;
		if (sleep_soc > 0) {
			BAT_INFO("calib0: rl=%d, dl=%d, intval=%d\n",
				 battery->rsoc, battery->dsoc, sleep_soc);
			if (battery->dsoc / 1000 < 5) {
				battery->dsoc -= 1000;
			} else if ((tgt_dsoc / 1000 < 5) &&
				   (battery->dsoc  / 1000 >= 5)) {
				if (battery->dsoc / 1000 == 5)
					battery->dsoc -= 1000;
				else
					battery->dsoc = 5 * 1000;
			} else if (tgt_dsoc / 1000 > 5) {
				battery->dsoc = tgt_dsoc;
			}
		}

		DBG("%s: dsoc<=rsoc, sum_cap=%d==>sleep_soc=%d, tgt_dsoc=%d\n",
		    __func__, battery->sleep_sum_cap, sleep_soc, tgt_dsoc);
	} else {
		/* di->dsoc > di->rsoc */
		battery->sleep_sum_cap = (SLP_CURR_MAX * sleep_sec / 3600);
		sleep_soc = battery->sleep_sum_cap / DIV(battery->fcc / 100);
		gap_soc = battery->dsoc - battery->rsoc;

		DBG("calib1: rsoc=%d, dsoc=%d, intval=%d\n",
		    battery->rsoc, battery->dsoc, sleep_soc);
		if (gap_soc > sleep_soc) {
			if ((gap_soc - 5000) > (sleep_soc * 2 * 1000))
				battery->dsoc -= (sleep_soc * 2 * 1000);
			else
				battery->dsoc -= sleep_soc * 1000;
		} else {
			battery->dsoc = battery->rsoc;
		}

		DBG("%s: dsoc>rsoc, sum_cap=%d=>sleep_soc=%d, gap_soc=%d\n",
		    __func__, battery->sleep_sum_cap, sleep_soc, gap_soc);
	}

	if (battery->voltage_avg <= pwroff_vol - 70) {
		battery->dsoc = 0;
		DBG("low power sleeping, shutdown... %d\n", battery->dsoc);
	}

	if (ocv_soc_updated && sleep_soc &&
	    (battery->rsoc - battery->dsoc) < 5000 &&
	    battery->dsoc < 40 * 1000) {
		battery->dsoc -= 1000;
		DBG("low power sleeping, reserved... %d\n", battery->dsoc);
	}

	if (battery->dsoc <= 0) {
		battery->dsoc = 0;
		DBG("sleep dsoc is %d...\n", battery->dsoc);
	}

	DBG("<%s>. out: dsoc=%d, rsoc=%d, sum_cap=%d\n",
	    __func__, battery->dsoc, battery->rsoc, battery->sleep_sum_cap);

	return sleep_soc;
}
#endif

static int rk817_bat_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk817_battery_device *battery = dev_get_drvdata(&pdev->dev);

#if 0
	int interval_sec = 0, time_step, pwroff_vol;

	battery->s2r = true;
	battery->current_avg = rk817_bat_get_avg_current(battery);
	battery->voltage_relax = rk817_bat_get_relax_voltage(battery);
	battery->voltage_avg = rk817_bat_get_battery_voltage(battery);
	battery->remain_cap = rk817_bat_get_capacity_uah(battery);
	battery->rsoc = rk817_bat_get_rsoc(battery);
	interval_sec = rk817_bat_rtc_sleep_sec(battery);
	battery->sleep_sum_sec += interval_sec;
	pwroff_vol = battery->pdata->pwroff_vol;

	if (!battery->sleep_chrg_online) {
		/* only add up discharge sleep seconds */
		battery->sleep_dischrg_sec += interval_sec;
		if (battery->voltage_avg <= pwroff_vol + 50)
			time_step = DISCHRG_TIME_STEP1;
		else
			time_step = DISCHRG_TIME_STEP2;
	}

	// DBG
	printk("resume: dl=%d rl=%d c=%d v=%d rv=%d "
	    "cap=%d dt=%d at=%ld ch=%d, sec = %d\n",
	    battery->dsoc, battery->rsoc, battery->current_avg,
	    battery->voltage_avg, battery->voltage_relax,
	    rk817_bat_get_capacity_uah(battery), interval_sec,
	    battery->sleep_dischrg_sec, battery->sleep_chrg_online,
	    interval_sec);

	/* sleep: enough time and discharge */
	if ((!battery->sleep_chrg_online) &&
	    (battery->sleep_dischrg_sec > time_step)) {
		if (rk817_bat_sleep_dischrg(battery))
			battery->sleep_dischrg_sec = 0;
	}

	rk817_bat_save_data(battery);

	/* charge/lowpower lock: for battery work to update dsoc and rsoc */
	if ((battery->sleep_chrg_online) ||
	    (!battery->sleep_chrg_online &&
	    battery->voltage_avg < battery->pdata->pwroff_vol))
		wake_lock_timeout(&battery->wake_lock, msecs_to_jiffies(2000));

	queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work,
			   msecs_to_jiffies(1000));
#else
	int interval_sec = 0;
	rk817_bat_field_write(battery, FRAME_SMP_INTERV, 0); //tanlq add 200403
	interval_sec = rk817_bat_rtc_sleep_sec(battery);
	if(interval_sec > MINUTE(30)) { // 20210221: battery->s2r 用来表示是否允许电量有突变。比如长时间待机以及开机.
	    battery->s2r = true;
	}
	

	// 20201209: 测试发现 817的电量计在低放电电流的情况下，检测也很准确。比如PX30 SDK 待机
	// 电流可以识别到是 2MA。所以休眠的时候不需要进行任何调整，有内部电量计来进行统计就可以了。
	#if 0
	if (!battery->sleep_chrg_online) {
		battery->sleep_dischrg_sec += interval_sec;
	}
	battery->current_avg = rk817_bat_get_avg_current(battery);
	battery->voltage_relax = rk817_bat_get_relax_voltage(battery);
	battery->voltage_avg = rk817_bat_get_battery_voltage(battery);
	battery->remain_cap = rk817_bat_get_capacity_uah(battery);
	battery->rsoc = rk817_bat_get_rsoc(battery);

	BAT_INFO("RESUME:v_avg=%d,v_sys=%d,c_avg=%d,v_relx=%d"
			",rsoc=%d,dsoc=%d,remain_cap=%d,sleep_sec=%d/%ld\n",
			battery->voltage_avg,
			battery->voltage_sys,
			battery->current_avg,
			battery->voltage_relax,
			battery->rsoc, battery->dsoc,
			battery->remain_cap, interval_sec,
			battery->sleep_dischrg_sec);

	if (!battery->sleep_chrg_online) {
		int time_step time_step = DISCHRG_TIME_STEP2;
		/* sleep: enough time and discharge */
		if(battery->sleep_dischrg_sec > time_step) {
			battery->resume_adjust = true;
			if (rk817_bat_sleep_dischrg(battery))
				battery->sleep_dischrg_sec = 0;
		}
	}
	#else
	//battery->resume_adjust = true;
	#endif

    // 20210506: 增加LOG用于打印确认817采样到的平均放电电流是否正确.
    // 20210529-LOG: resume: interval_sec=1622266126,s2r=1,c_avg=-5 ,rm for fast-resume.
	//battery->current_avg = rk817_bat_get_avg_current(battery);
    //printk("resume: interval_sec=%d,s2r=%d,c_avg=%d\n", interval_sec, battery->s2r, battery->current_avg);
	    
	// 20191204: for eink fast wakeup,we do this at work.
	// wake_lock_timeout(&battery->wake_lock, msecs_to_jiffies(2000)); // kernel-suspend thread will do this.
	queue_delayed_work(battery->bat_monitor_wq, &battery->bat_delay_work,
			   msecs_to_jiffies(500));
#endif
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(rk817_bat_pm_ops,
			 rk817_bat_pm_suspend,
			 rk817_bat_pm_resume);

static struct platform_driver rk817_battery_driver = {
	.probe = rk817_battery_probe,
	.shutdown = rk817_battery_shutdown,
	.driver = {
		.name = "rk817-battery",
		.pm = &rk817_bat_pm_ops,
		.of_match_table = of_match_ptr(rk817_bat_of_match),
	},
};

static int __init rk817_battery_init(void)
{
	return platform_driver_register(&rk817_battery_driver);
}
fs_initcall_sync(rk817_battery_init);

static void __exit rk817_battery_exit(void)
{
	platform_driver_unregister(&rk817_battery_driver);
}
module_exit(rk817_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("shengfeixu <xsf@rock-chips.com>");
MODULE_DESCRIPTION("rk817 battery Charger Driver");

