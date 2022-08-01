/*
 * PMIC management for epaper power control HAL
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef PMIC_H
#define PMIC_H

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/workqueue.h>

struct pmic_sess;

#define PMIC_DEFAULT_DWELL_TIME_MS	1111
#define PMIC_DEFAULT_VCOMOFF_TIME_MS	20
#define PMIC_DEFAULT_V3P3OFF_TIME_MS	-1

#if defined(FB_OMAP3EP_PAPYRUS_PM_VZERO)
  #define PAPYRUS_STANDBY_DWELL_TIME	4 /*sec*/
#else
  #define PAPYRUS_STANDBY_DWELL_TIME	0
#endif

#define	SEQ_VDD(index)		((index % 4) << 6)
#define SEQ_VPOS(index)		((index % 4) << 4)
#define SEQ_VEE(index)		((index % 4) << 2)
#define SEQ_VNEG(index)		((index % 4) << 0)

/* power up seq delay time */
#define UDLY_3ms(index)		(0x00 << ((index%4) * 2))
#define UDLY_6ms(index)		(0x01 << ((index%4) * 2))
#define UDLY_9ms(index)		(0x10 << ((index%4) * 2))
#define UDLY_12ms(index)	(0x11 << ((index%4) * 2))

/* power down seq delay time */
#define DDLY_6ms(index)		(0x00 << ((index%4) * 2))
#define DDLY_12ms(index)	(0x01 << ((index%4) * 2))
#define DDLY_24ms(index)	(0x10 << ((index%4) * 2))
#define DDLY_48ms(index)	(0x11 << ((index%4) * 2))


#define NUMBER_PMIC_REGS	10

struct pmic_driver {
	const char *id;

	int vcom_min;
	int vcom_max;
	int vcom_step;

	int (*hw_read_temperature)(struct pmic_sess *sess, int *t);
	bool (*hw_power_ack)(struct pmic_sess *sess);
	void (*hw_power_req)(struct pmic_sess *sess, bool up);

	int (*set_enable)(struct pmic_sess *sess, int enable);
	int (*set_vcom_voltage)(struct pmic_sess *sess, int vcom_mv);
	int (*set_vcom1)(struct pmic_sess *sess, uint8_t vcom1);
	int (*set_vcom2)(struct pmic_sess *sess, uint8_t vcom2);
	int (*set_vadj)(struct pmic_sess *sess, uint8_t vadj);
	int (*set_int_en1)(struct pmic_sess *sess, uint8_t int_en1);
	int (*set_int_en2)(struct pmic_sess *sess, uint8_t int_en2);
	int (*set_upseq0)(struct pmic_sess *sess, uint8_t upseq0);
	int (*set_upseq1)(struct pmic_sess *sess, uint8_t upseq1);
	int (*set_dwnseq0)(struct pmic_sess *sess, uint8_t dwnseq0);
	int (*set_dwnseq1)(struct pmic_sess *sess, uint8_t dwnseq1);
	int (*set_tmst1)(struct pmic_sess *sess, uint8_t tmst1);
	int (*set_tmst2)(struct pmic_sess *sess, uint8_t tmst2);

	int (*set_vp_adjust)(struct pmic_sess *sess, uint8_t vp_adjust);
	int (*set_vn_adjust)(struct pmic_sess *sess, uint8_t vn_adjust);
	int (*set_vcom_adjust)(struct pmic_sess *sess, uint8_t vcom_adjust);
	int (*set_pwr_seq0)(struct pmic_sess *sess, uint8_t pwr_seq0);
	int (*set_pwr_seq1)(struct pmic_sess *sess, uint8_t pwr_seq1);
	int (*set_pwr_seq2)(struct pmic_sess *sess, uint8_t pwr_seq2);
	int (*set_tmst_config)(struct pmic_sess *sess, uint8_t tmst_config);
	int (*set_tmst_os)(struct pmic_sess *sess, uint8_t tmst_os);
	int (*set_tmst_hyst)(struct pmic_sess *sess, uint8_t tmst_hyst);

	int (*hw_vcom_switch)(struct pmic_sess *sess, bool state);

	int (*hw_set_dvcom)(struct pmic_sess *sess, int state);

	int (*hw_init)(struct pmic_sess *sess ,struct i2c_client *);
	void (*hw_cleanup)(struct pmic_sess *sess);

	bool (*hw_standby_dwell_time_ready)(struct pmic_sess *sess);
	int (*hw_pm_sleep)(struct pmic_sess *sess);
	int (*hw_pm_resume)(struct pmic_sess *sess);


};

struct pmic_sess {
	bool powered;
	struct delayed_work powerdown_work;
	unsigned int dwell_time_ms;
	struct delayed_work vcomoff_work;
	unsigned int vcomoff_time_ms;
	int v3p3off_time_ms;
	const struct pmic_driver *drv;
	void *drvpar;
	int temp_man_offset;
	int revision;
	int is_inited;
};

extern int pmic_probe(struct pmic_sess **sess, const char *id,
					unsigned int dwell_time_ms,
					unsigned int vcomoff_time_ms,
					int v3p3_time_ms);

extern void pmic_remove(struct pmic_sess **sess);

extern int pmic_set_registers_papyrus_1(struct pmic_sess *sess, uint8_t *vals);
extern int pmic_set_registers_papyrus_2(struct pmic_sess *sess, uint8_t *vals);
/*
 * Set VCOM voltage, in millivolts.
 * NB! Change will take effect with the next pmic power up!
 */
extern int pmic_set_vcom_voltage(struct pmic_sess *sess, int vcom_mv);

/*Runtame Change VCOM state */
extern int pmic_set_dvcom(struct pmic_sess *sess,int state);

/* Request asynchronous power up. */
extern int pmic_req_powerup(struct pmic_sess *sess);

/* Ensure that power is OK. Called after a pmic_req_powerup(). */
extern int pmic_sync_powerup(struct pmic_sess *sess);

/* Get pmic temperature sensor measurement. */
extern int pmic_get_temperature(struct pmic_sess *sess, int *t);

/*
 * Asynchronously release the power up requirement. Power may go down
 * up to a few seconds after this call in order to avoid power up/down
 * cycles with frequent screen updates.
 */
extern void pmic_release_powerup_req(struct pmic_sess *sess);

/* Get that enough delay between standby and sleep*/
extern bool pmic_standby_dwell_time_ready(struct pmic_sess *sess);

extern void pmic_pm_sleep(struct pmic_sess *sess);

extern void pmic_pm_resume(struct pmic_sess *sess);

//int tps65185_vcom_get(void);
//int tps65185_vcom_set(int vcom_mv);


// 20190906: EINK 电源操作函数.电源模块提供实现函数，且提供 htfy_register_ebc_pwr_ops 供本模块调用.
// 返回值: 0 --操作OK, < 0: 出错。
struct htfy_pwr_ops
{
	void 	*priv;
	int 	(*power_on)(void*);
	int 	(*power_down)(void*);
	int 	(*vcom_set)(void*, int);

    // 20210512: sometime,power-on failed,add check status.return:1 power good, 0: power fail,
    // needs repower。
    int     (*power_check)(void*, int timeout);

    // 20210513: 在 WAKEUP 拉高的时候，我们需要重新初始化 65185（因为内部的寄存器被复位了).
    // 我们增加了这个函数。其实很多设置直接用默认的值也是可以的。
    // 20210727: 增加一个新的参数: epd_type 来做新旧 EPD的兼容。这个参数来源于 ebc-driver,所以
    // 我们在 65185 里面的初始化要延后一点，通过这个 reinit 来进行。
    int     (*reinit)(void*, int epd_type);
    
	// 20200101：如果调用了 sleep函数，唤醒的时候会显示异常。
	//int 	(*power_hw_sleep)(void*);
	//int 	(*power_hw_resume)(void*);
};

// 20190906: EINK 温度传感器读取温度操作，温控模块需提供 htfy_register_ebc_temp_ops 供本模块调用.
// 返回值: 0 --操作OK, < 0: 出错。
struct htfy_gettemp_ops
{
	void 	*priv;
	int 	(*temperature_get)(void*, int *);
};


#endif  /* PMIC_H */
