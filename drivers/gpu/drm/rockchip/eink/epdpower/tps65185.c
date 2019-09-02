/*
 * Papyrus epaper power control HAL
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * TPS6518x power control is facilitated using I2C control and WAKEUP GPIO
 * pin control. The other VCC GPIO Papyrus' signals must be tied to ground.
 *
 * TODO:
 * 	- Instead of polling, use interrupts to signal power up/down
 * 	  acknowledge.
 */
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/suspend.h>

#define INVALID_GPIO -1

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31))
#include <linux/i2c/pmic-tps65185-i2c.h>
#else
#define PAPYRUS2_1P0_I2C_ADDRESS		0x48
#define PAPYRUS2_1P1_I2C_ADDRESS		0x68
extern void papyrus_set_i2c_address(int address);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
  #include <asm/gpio.h>
#else
  #include <linux/gpio.h>
#endif

#include "pmic.h"
#include "../ebc.h"
#define TPS65185_I2C_NAME "tps65185"

#define PAPYRUS_VCOM_MAX_MV		0
#define PAPYRUS_VCOM_MIN_MV		-5110

#if 1
#define tps65185_printk(fmt, args...) printk(KERN_INFO "[tps] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define tps65185_printk(fmt, args...) 
#endif

/* After waking up from sleep, Papyrus
   waits for VN to be discharged and all
   voltage ref to startup before loading
   the default EEPROM settings. So accessing
   registers too early after WAKEUP could
   cause the register to be overridden by
   default values */
#define PAPYRUS_EEPROM_DELAY_MS 50
/* Papyrus WAKEUP pin must stay low for
   a minimum time */
#define PAPYRUS_SLEEP_MINIMUM_MS 110
/* Temp sensor might take a little time to
   settle eventhough the status bit in TMST1
   state conversion is done - if read too early
   0C will be returned instead of the right temp */
#define PAPYRUS_TEMP_READ_TIME_MS 10

/* Powerup sequence takes at least 24 ms - no need to poll too frequently */
#define HW_GET_STATE_INTERVAL_MS 24

struct papyrus_sess {
	struct i2c_adapter *adap;
	struct i2c_client *client;
	uint8_t enable_reg_shadow;
	uint8_t enable_reg;
	uint8_t vadj;
	uint8_t vcom1;
	uint8_t vcom2;
	uint8_t vcom2off;
	uint8_t int_en1;
	uint8_t int_en2;
	uint8_t upseq0;
	uint8_t upseq1;
	uint8_t dwnseq0;
	uint8_t dwnseq1;
	uint8_t tmst1;
	uint8_t tmst2;

	/* Custom power up/down sequence settings */
	struct {
		/* If options are not valid we will rely on HW defaults. */
		bool valid;
		unsigned int dly[8];
	} seq;
	struct timeval standby_tv;
	unsigned int v3p3off_time_ms;
	int pwr_up_pin;
	int wake_up_pin;
	int vcom_ctl_pin;
	/* True if a high WAKEUP brings Papyrus out of reset. */
	int poweron_active_high;
	int wakeup_active_high;
	int vcomctl_active_high;
};

//#define tps65185_SPEED	(400*1000)


#define PAPYRUS_ADDR_TMST_VALUE		0x00
#define PAPYRUS_ADDR_ENABLE		0x01
#define PAPYRUS_ADDR_VADJ		0x02
#define PAPYRUS_ADDR_VCOM1_ADJUST	0x03
#define PAPYRUS_ADDR_VCOM2_ADJUST	0x04
#define PAPYRUS_ADDR_INT_ENABLE1	0x05
#define PAPYRUS_ADDR_INT_ENABLE2	0x06
#define PAPYRUS_ADDR_INT_STATUS1	0x07
#define PAPYRUS_ADDR_INT_STATUS2	0x08
#define PAPYRUS_ADDR_UPSEQ0		0x09
#define PAPYRUS_ADDR_UPSEQ1		0x0a
#define PAPYRUS_ADDR_DWNSEQ0		0x0b
#define PAPYRUS_ADDR_DWNSEQ1		0x0c
#define PAPYRUS_ADDR_TMST1		0x0d
#define PAPYRUS_ADDR_TMST2		0x0e
#define PAPYRUS_ADDR_PG_STATUS		0x0f
#define PAPYRUS_ADDR_REVID		0x10

// INT_ENABLE1
#define PAPYRUS_INT_ENABLE1_ACQC_EN	1
#define PAPYRUS_INT_ENABLE1_PRGC_EN 0

// INT_STATUS1
#define PAPYRUS_INT_STATUS1_ACQC	1
#define PAPYRUS_INT_STATUS1_PRGC	0

// VCOM2_ADJUST
#define PAPYRUS_VCOM2_ACQ	7
#define PAPYRUS_VCOM2_PROG	6
#define PAPYRUS_VCOM2_HIZ	5



#define PAPYRUS_MV_TO_VCOMREG(MV)	((MV) / 10)

#define V3P3_EN_MASK	0x20
#define PAPYRUS_V3P3OFF_DELAY_MS 10//100

struct papyrus_hw_state {
	uint8_t tmst_value;
	uint8_t int_status1;
	uint8_t int_status2;
	uint8_t pg_status;
};

static uint8_t papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;

static int papyrus_hw_setreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t val)
{
	int stat;
	uint8_t txbuf[2] = { regaddr, val };
	struct i2c_msg msgs[] = {
		{
			.addr = sess->client->addr,//papyrus2_i2c_addr,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
			//.scl_rate = tps65185_SPEED,
		}
	};

	stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));

	if (stat < 0)
		pr_err("papyrus: I2C send error: %d\n", stat);
	else if (stat != ARRAY_SIZE(msgs)) {
		pr_err("papyrus: I2C send N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}


static int papyrus_hw_getreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t *val)
{
	int stat;
	struct i2c_msg msgs[] = {
		{
			.addr = sess->client->addr,////papyrus2_i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &regaddr,
			//.scl_rate = tps65185_SPEED,
		},
		{
			.addr = sess->client->addr,//papyrus2_i2c_addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
			//.scl_rate = tps65185_SPEED,
		}
	};

	stat = i2c_transfer(sess->adap, msgs, ARRAY_SIZE(msgs));

	if (stat < 0)
    {
		pr_err("papyrus: I2C read error: %d\n", stat);
        pr_err("Papyrus i2c addr %x %s\n",sess->client->addr,__FILE__); 
    }
	else if (stat != ARRAY_SIZE(msgs)) {
		pr_err("papyrus: I2C read N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}


static void papyrus_hw_get_pg(struct papyrus_sess *sess,
							  struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}

/*
static void papyrus_hw_get_state(struct papyrus_sess *sess, struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST_VALUE, &hwst->tmst_value);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS1, &hwst->int_status1);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS2, &hwst->int_status2);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}
*/

static void papyrus_hw_send_powerup(struct papyrus_sess *sess)
{
	int stat = 0;

	// set VADJ 
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);

	// set UPSEQs & DWNSEQs 
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);

	// commit it, so that we can adjust vcom through "Rk_ebc_power_control_Release_v1.1" 
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, sess->vcom1);
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, sess->vcom2);

#if 1
	/* Enable 3.3V switch to the panel */
	sess->enable_reg_shadow |= V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	msleep(sess->v3p3off_time_ms);
#endif

	/* switch to active mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive, 
	 * don't enable vcom buffer
	 */
	sess->enable_reg_shadow = (0x80 | 0x30 | 0x0F);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	return;
}

#if 1
static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
	int stat;

	/* switch to standby mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive, 
	 * don't enable vcom buffer
	 */
	sess->enable_reg_shadow = (0x40 | 0x20 | 0x0F);
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	
	do_gettimeofday(&sess->standby_tv);

#if 1
	/* 3.3V switch must be turned off last */
	msleep(sess->v3p3off_time_ms);
	sess->enable_reg_shadow &= ~V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
#endif

	return;
}
#endif

static int papyrus_hw_read_temperature(struct pmic_sess *pmsess, int *t)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;
	int ntries = 50;
	uint8_t tb;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST1, 0x80);

	do {
		stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST1, &tb);
	} while (!stat && ntries-- && (((tb & 0x20) == 0) || (tb & 0x80)));

	if (stat)
		return stat;

	msleep(PAPYRUS_TEMP_READ_TIME_MS);
	stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_TMST_VALUE, &tb);
	*t = (int)(int8_t)tb;

	//tps65185_printk("current temperature is %d\n",*t);

	return stat;
}
static int papyrus_hw_get_revid(struct papyrus_sess *sess)
{
	int stat;
	uint8_t revid;

	stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_REVID, &revid);
	if (stat) {
		pr_err("papyrus: I2C error: %d\n", stat);
		return stat;
	} else
		return revid;
}

void papyrus_set_i2c_address(int address)
{
    if    (address == PAPYRUS2_1P0_I2C_ADDRESS)
    {
        papyrus2_i2c_addr = PAPYRUS2_1P0_I2C_ADDRESS;
    }
    else if (address == PAPYRUS2_1P1_I2C_ADDRESS)
    {
        papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;
    } else {
        pr_err("papyrus: Invalid i2c address: %d\n", address);
    }
    printk("papyrus i2c addr set to %x\n",papyrus2_i2c_addr);
}

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
#if 1
	sess->vadj = 0x03;
	
	sess->upseq0 = SEQ_VNEG(0) | SEQ_VEE(1) | SEQ_VPOS(2) | SEQ_VDD(3);
	sess->upseq1 = UDLY_3ms(0) | UDLY_3ms(1) | UDLY_3ms(2) | UDLY_3ms(3);
	
	sess->dwnseq0 = SEQ_VDD(0) | SEQ_VPOS(1) | SEQ_VEE(2) | SEQ_VNEG(3);
	sess->dwnseq1 = DDLY_6ms(0) | DDLY_6ms(1) | DDLY_6ms(2) | DDLY_6ms(3);

	sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(2500) & 0x00FF);
	sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(2500) & 0x0100) >> 8);
#else
	sess->vadj = 0x03;

	sess->upseq0 = 0xE1;
	sess->upseq1 = 0x55;

	sess->dwnseq0 = 0x1B;
	sess->dwnseq1 = 0xC0;
#endif

	return 0;
}


static int papyrus_hw_init(struct papyrus_sess *sess, const char *chip_id)
{
	int stat = 0;

        if((sess->pwr_up_pin!= INVALID_GPIO))
                stat |= gpio_request(sess->pwr_up_pin, "papyrus-power-on");
	if((sess->wake_up_pin!= INVALID_GPIO))
		stat |= gpio_request(sess->wake_up_pin, "papyrus-wake_up");
	if((sess->vcom_ctl_pin!= INVALID_GPIO))
		stat |= gpio_request(sess->vcom_ctl_pin, "papyrus-vcom-ctl");
	if (stat) {
		pr_err("papyrus: cannot reserve GPIOs\n");
		stat = -ENODEV;
		return stat;
	}
	sess->poweron_active_high = 1;
	sess->wakeup_active_high = 1;
	sess->vcomctl_active_high = 1;
	if((sess->wake_up_pin != INVALID_GPIO)){
		gpio_direction_output(sess->wake_up_pin, !sess->wakeup_active_high);
		/* wait to reset papyrus */
		msleep(PAPYRUS_SLEEP_MINIMUM_MS);
		gpio_direction_output(sess->wake_up_pin, sess->wakeup_active_high);
		if (sess->pwr_up_pin != INVALID_GPIO)
			gpio_direction_output(sess->pwr_up_pin, sess->poweron_active_high);
		gpio_direction_output(sess->vcom_ctl_pin, sess->vcomctl_active_high);
		msleep(PAPYRUS_EEPROM_DELAY_MS);
	}

	stat = papyrus_hw_get_revid(sess);
	if (stat < 0)
		goto cleanup_i2c_adapter;
	pr_info("papyrus: detected device with ID=%02x (TPS6518%dr%dp%d)\n",
			stat, stat & 0xF, (stat & 0xC0) >> 6, (stat & 0x30) >> 4);
	stat = 0;

	return stat;

cleanup_i2c_adapter:
	i2c_put_adapter(sess->adap);
//free_gpios:
	gpio_free(sess->wake_up_pin);
	gpio_free(sess->vcom_ctl_pin);
	if (sess->pwr_up_pin != INVALID_GPIO)
		gpio_free(sess->pwr_up_pin);
	pr_err("papyrus: ERROR: could not initialize I2C papyrus!\n");
	return stat;
}

static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
	struct papyrus_sess *sess = pmsess->drvpar;

	pr_debug("papyrus: i2c pwr req: %d\n", up);
	if (up){
		papyrus_hw_send_powerup(sess);
	} else {
		papyrus_hw_send_powerdown(sess);
	}
	return;
}


static bool papyrus_hw_power_ack(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct papyrus_hw_state hwst;
	int st;
	int retries_left = 10;

	do {
		papyrus_hw_get_pg(sess, &hwst);

		pr_debug("hwst: tmst_val=%d, ist1=%02x, ist2=%02x, pg=%02x\n",
				hwst.tmst_value, hwst.int_status1,
				hwst.int_status2, hwst.pg_status);
		hwst.pg_status &= 0xfa;
		if (hwst.pg_status == 0xfa)
			st = 1;
		else if (hwst.pg_status == 0x00)
			st = 0;
		else {
			st = -1;	/* not settled yet */
			msleep(HW_GET_STATE_INTERVAL_MS);
		}
		retries_left--;
	} while ((st == -1) && retries_left);

	if ((st == -1) && !retries_left)
		pr_err("papyrus: power up/down settle error (PG = %02x)\n", hwst.pg_status);

	return !!st;
}


static void papyrus_hw_cleanup(struct papyrus_sess *sess)
{
	gpio_free(sess->wake_up_pin);
	gpio_free(sess->vcom_ctl_pin);
	if (sess->pwr_up_pin != INVALID_GPIO)
		gpio_free(sess->pwr_up_pin);
	i2c_put_adapter(sess->adap);
}

/* -------------------------------------------------------------------------*/

static int papyrus_set_enable(struct pmic_sess *pmsess, int enable)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->enable_reg = enable;
	return 0;
}

static int papyrus_set_vcom_voltage(struct pmic_sess *pmsess, int vcom_mv)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(vcom_mv) & 0x00FF);
	sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(vcom_mv) & 0x0100) >> 8);
	return 0;
}

static int papyrus_set_vcom1(struct pmic_sess *pmsess, uint8_t vcom1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = vcom1;
	return 0;
}

static int papyrus_set_vcom2(struct pmic_sess *pmsess, uint8_t vcom2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	// TODO; Remove this temporary solution to set custom vcom-off mode
	//       Add PMIC setting when this is to be a permanent feature
	pr_debug("papyrus_set_vcom2 vcom2off 0x%02x\n", vcom2);
	sess->vcom2off = vcom2;
	return 0;
}

static int papyrus_set_vadj(struct pmic_sess *pmsess, uint8_t vadj)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vadj = vadj;
	return 0;
}

static int papyrus_set_int_en1(struct pmic_sess *pmsess, uint8_t int_en1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en1 = int_en1;
	return 0;
}

static int papyrus_set_int_en2(struct pmic_sess *pmsess, uint8_t int_en2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en2 = int_en2;
	return 0;
}

static int papyrus_set_upseq0(struct pmic_sess *pmsess, uint8_t upseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq0 = upseq0;
	return 0;
}

static int papyrus_set_upseq1(struct pmic_sess *pmsess, uint8_t upseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq1 = upseq1;
	return 0;
}

static int papyrus_set_dwnseq0(struct pmic_sess *pmsess, uint8_t dwnseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq0 = dwnseq0;
	return 0;
}

static int papyrus_set_dwnseq1(struct pmic_sess *pmsess, uint8_t dwnseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq1 = dwnseq1;
	return 0;
}

static int papyrus_set_tmst1(struct pmic_sess *pmsess, uint8_t tmst1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst1 = tmst1;
	return 0;
}

static int papyrus_set_tmst2(struct pmic_sess *pmsess, uint8_t tmst2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst2 = tmst2;
	return 0;
}

static int papyrus_vcom_switch(struct pmic_sess *pmsess, bool state)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;

	sess->enable_reg_shadow &= ~((1u << 4) | (1u << 6) | (1u << 7));
	sess->enable_reg_shadow |= (state ? 1u : 0) << 4;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* set VCOM off output */
	if (!state && sess->vcom2off != 0) {
		stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,
						sess->vcom2off);
	}

	return stat;
}

static bool papyrus_standby_dwell_time_ready(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct timeval current_tv;
	long total_secs;

	do_gettimeofday(&current_tv);
	mb();
	total_secs = current_tv.tv_sec - sess->standby_tv.tv_sec;

	if (total_secs < PAPYRUS_STANDBY_DWELL_TIME)
		return false;

	return true;
}

static int nWakeFormIdleMode = 0;

static int papyrus_pm_sleep(struct pmic_sess *sess)
{
	struct papyrus_sess *s = sess->drvpar;

#ifdef CONFIG_IDLE
	if (PM_SUSPEND_IDLE == get_suspend_state()){                           
		nWakeFormIdleMode = 1;												
		return 0;
	}                                                                      
#endif 

	nWakeFormIdleMode = 0;
	tps65185_printk("%s\n", __func__);

	if(support_tps_3v3_always_alive()){
		//papyrus_hw_send_powerdown(s);
		return 0;
	}
	gpio_direction_output(s->vcom_ctl_pin, !s->vcomctl_active_high);
	gpio_direction_output(s->wake_up_pin, !s->wakeup_active_high);

	return 0;
}

static int papyrus_pm_resume(struct pmic_sess *sess)
{
	struct papyrus_sess *s = sess->drvpar;

#ifdef CONFIG_IDLE
	if(nWakeFormIdleMode == 1){
		return 0;
	}
#endif

	tps65185_printk("%s\n", __func__);

	if(support_tps_3v3_always_alive()){
		//papyrus_hw_send_powerup(s);
		return 0;
	}
	gpio_direction_output(s->wake_up_pin, s->wakeup_active_high);
	gpio_direction_output(s->vcom_ctl_pin, s->vcomctl_active_high);
	
	return 0;
}

static int papyrus_probe(struct pmic_sess *pmsess,struct i2c_client *client)
{
	struct papyrus_sess *sess;
	int stat;
	enum of_gpio_flags flags;
	struct device_node *node = client->dev.of_node;

	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess) {
		pr_err("%s:%d: kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	sess->client = client;
	sess->adap = client->adapter;

	papyrus_hw_arg_init(sess);

	//if (pmsess->v3p3off_time_ms == -1)
		sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;
	//else
	//	sess->v3p3off_time_ms = pmsess->v3p3off_time_ms;

	do_gettimeofday(&sess->standby_tv);

	sess->wake_up_pin = of_get_named_gpio_flags(node, "wakeup_pin", 0, &flags);
	if (!gpio_is_valid(sess->wake_up_pin)) {
		pr_err("tsp65185: failed to find wakeup pin\n");
		goto free_sess;
	}

	sess->vcom_ctl_pin = of_get_named_gpio_flags(node, "vcomctl_pin", 0, &flags);
	if (!gpio_is_valid(sess->vcom_ctl_pin)) {
		pr_err("tsp65185: failed to find vcom_ctl pin\n");
		goto free_sess;
	}

	sess->pwr_up_pin = of_get_named_gpio_flags(node, "powerup_pin", 0, &flags);
	if (!gpio_is_valid(sess->pwr_up_pin)) {
		sess->pwr_up_pin = INVALID_GPIO;
		pr_err("tsp65185: failed to find pwr_up pin\n");
	}

	stat = papyrus_hw_init(sess, pmsess->drv->id);
	if (stat)
		goto free_sess;

	sess->enable_reg_shadow = 0;
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	if (stat)
		goto free_sess;

	pmsess->drvpar = sess;
	pmsess->revision = papyrus_hw_get_revid(sess);
	return 0;

free_sess:
	kfree(sess);

	return stat;
}

static void papyrus_remove(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;

	papyrus_hw_cleanup(sess);

	kfree(sess);
	pmsess->drvpar = 0;
}

const struct pmic_driver pmic_driver_tps65185_i2c = {
	.id = "tps65185-i2c",

	.vcom_min = PAPYRUS_VCOM_MIN_MV,
	.vcom_max = PAPYRUS_VCOM_MAX_MV,
	.vcom_step = 10,

	.hw_read_temperature = papyrus_hw_read_temperature,
	.hw_power_ack = papyrus_hw_power_ack,
	.hw_power_req = papyrus_hw_power_req,

	.set_enable = papyrus_set_enable,
	.set_vcom_voltage = papyrus_set_vcom_voltage,
	.set_vcom1 = papyrus_set_vcom1,
	.set_vcom2 = papyrus_set_vcom2,
	.set_vadj = papyrus_set_vadj,
	.set_int_en1 = papyrus_set_int_en1,
	.set_int_en2 = papyrus_set_int_en2,
	.set_upseq0 = papyrus_set_upseq0,
	.set_upseq1 = papyrus_set_upseq1,
	.set_dwnseq0 = papyrus_set_dwnseq0,
	.set_dwnseq1 = papyrus_set_dwnseq1,
	.set_tmst1 = papyrus_set_tmst1,
	.set_tmst2 = papyrus_set_tmst2,

	.hw_vcom_switch = papyrus_vcom_switch,

	.hw_init = papyrus_probe,
	.hw_cleanup = papyrus_remove,

	.hw_standby_dwell_time_ready = papyrus_standby_dwell_time_ready,
	.hw_pm_sleep = papyrus_pm_sleep,
	.hw_pm_resume = papyrus_pm_resume,
};

struct pmic_sess pmic_sess_data;

static int vcom_convertint(const char s[])  
{  
    int i;  
    int n = 0;  
    for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i)  
    {  
        n = 10 * n + (s[i] - '0');  
    }  
    return n;  
} 

static ssize_t vcom_mv_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value = tps65185_vcom_get();
	if (value < 0)
		return -EINVAL;
	return sprintf(buf, "%d\n", value);
}

static ssize_t vcom_mv_set(struct device *dev, struct device_attribute *attr, const char *_buf, size_t _count)
{
	int value;

	value = vcom_convertint(_buf);
	if (value < 0 || value > 5110) {
		printk("value should be 0~5110\n");
		return _count;
	}
	printk("set vcom to: %dmV\n", value);

	tps65185_vcom_set(value);

	return _count;
}

static struct kobject *tps65185_kobj;
static struct device_attribute tps65185_attrs =
		__ATTR(vcom_mv, 0660, vcom_mv_get, vcom_mv_set);

static int tps65185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	tps65185_printk("I2C addr:%x\n", client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk("I2C check functionality failed.");
		return -ENODEV;
	}

	if(0 != pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data,client))
	{
		printk("pmic_driver_tps65185_i2c hw_init failed.");
		return -ENODEV;
	}
	//pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);

	pmic_sess_data.is_inited = 1;

	tps65185_kobj = kobject_create_and_add("tps65185", NULL);
	if (tps65185_kobj) {
		ret = sysfs_create_file(tps65185_kobj, &tps65185_attrs.attr);
		if (ret)
			dev_err(&client->dev, "create tps65185 sysfs error\n");
	}

	tps65185_printk("tps65185_probe ok.\n");

	return 0;
}

static int tps65185_remove(struct i2c_client *client)
{
	pmic_driver_tps65185_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
	memset(&pmic_sess_data,0,sizeof(struct pmic_sess));
	return 0;
}

#if 0
static int tps65185_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return pmic_driver_tps65185_i2c.hw_pm_sleep((struct pmic_sess *)&pmic_sess_data);
}
static int tps65185_resume(struct i2c_client *client)
{
	return pmic_driver_tps65185_i2c.hw_pm_resume((struct pmic_sess *)&pmic_sess_data);
}
#endif

static const struct i2c_device_id tps65185_id[] = {
	{ TPS65185_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id tps65185_dt_ids[] = {
	{ .compatible = "ti,tps65185", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, tps65185_dt_ids);
static struct i2c_driver tps65185_driver = {
	.probe	= tps65185_probe,
	.remove 	= tps65185_remove,
	//.suspend = tps65185_suspend,
	//.resume  = tps65185_resume,
	.id_table	= tps65185_id,
	.driver = {
		.of_match_table = tps65185_dt_ids,
		.name	  = TPS65185_I2C_NAME,
		.owner	  = THIS_MODULE,
	},
};

static int __init tps65185_init(void)
{
	int ret;

	ret = i2c_add_driver(&tps65185_driver);
	if (ret)
		printk("Register tps65185 driver failed.\n");

	return ret;
}

static void __exit tps65185_exit(void)
{
	return i2c_del_driver(&tps65185_driver);
}

fs_initcall(tps65185_init);
module_exit(tps65185_exit);

MODULE_DESCRIPTION("ti tps65185 pmic");
MODULE_LICENSE("GPL");

int tps65185_vcom_get(void)
{
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
	uint8_t rev_val = 0;
	int stat = 0;
	int read_vcom_mv = 0;

	tps65185_printk("tps65185_vcom_set enter.\n");
	if(!tpmic_sess_data.is_inited)
		return -1;
	// VERIFICATION
	gpio_direction_output(sess->wake_up_pin, 0);
	msleep(10);
	gpio_direction_output(sess->wake_up_pin, 1);
	msleep(10);
	read_vcom_mv = 0;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val);
	tps65185_printk("rev_val = 0x%x\n",rev_val);
	read_vcom_mv += rev_val;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val);
	tps65185_printk("rev_val = 0x%x\n",rev_val);
	read_vcom_mv += ((rev_val & 0x0001)<<8);
	tps65185_printk("read_vcom_mv = %d\n",read_vcom_mv);

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	return read_vcom_mv * 10;
}

int tps65185_vcom_set(int vcom_mv)
{
	//struct i2c_client *client=NULL; 
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
	uint8_t rev_val = 0;
	int stat = 0;
	int read_vcom_mv = 0;

	tps65185_printk("tps65185_vcom_set enter.\n");
	if(!tpmic_sess_data.is_inited)
		return -1;
	gpio_direction_output(sess->wake_up_pin, 1);
	msleep(10);
	// Set vcom voltage
	pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&tpmic_sess_data,vcom_mv);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST,sess->vcom1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,sess->vcom2);
	tps65185_printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);

	// PROGRAMMING
	sess->vcom2 |= 1<<PAPYRUS_VCOM2_PROG;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,sess->vcom2);
	rev_val = 0;
	while(!(rev_val & (1<<PAPYRUS_INT_STATUS1_PRGC)))
	{
		stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_INT_STATUS1, &rev_val);
		tps65185_printk("PAPYRUS_ADDR_INT_STATUS1 = 0x%x\n",rev_val);
		msleep(50);
	}
	
	// VERIFICATION
	tps65185_printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);
	gpio_direction_output(sess->wake_up_pin, 0);
	msleep(10);
	gpio_direction_output(sess->wake_up_pin, 1);
	msleep(10);
	read_vcom_mv = 0;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val);
	tps65185_printk("rev_val = 0x%x\n",rev_val);
	read_vcom_mv += rev_val;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val);
	tps65185_printk("rev_val = 0x%x\n",rev_val);
	read_vcom_mv += ((rev_val & 0x0001)<<8);
	tps65185_printk("read_vcom_mv = %d\n",read_vcom_mv);

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	return 0;
}

static int tps65185_set_vcom_voltage(int vcom_mv)
{
	if (vcom_mv < 0 || vcom_mv > 5110) {
		printk("tps: err vcom value, vcom value shoule be 0~5110\n");
		return -1;
	}

        if(pmic_sess_data.is_inited)
                pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&pmic_sess_data, vcom_mv);
        return 0;
}

static int tps65185_power_on(void)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);
	return 0;
}
static int tps65185_power_down(void)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,0);
	return 0;
}
static int tps65185_temperature_get(int *temp)
{
	if(pmic_sess_data.is_inited)
		return pmic_driver_tps65185_i2c.hw_read_temperature((struct pmic_sess *)&pmic_sess_data,temp);
	else
		return 0;
}
int register_ebc_pwr_ops(struct ebc_pwr_ops *ops)
{
	ops->power_on = tps65185_power_on;
	ops->power_down = tps65185_power_down;
	ops->vcom_set = tps65185_set_vcom_voltage;
	return 0;
}
#ifdef CONFIG_EPD_TPS65185_SENSOR
int register_ebc_temp_ops(struct ebc_temperateure_ops *ops)
{
	ops->temperature_get = tps65185_temperature_get;
	return 0;
}


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
	u32 value;
	tps65185_temperature_get(&value);
	seq_printf(s, "%d\n", value);

	return 0;
}

static int proc_lm_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_lm_show, NULL);
}

static const struct file_operations proc_lm_fops = {
	.open		= proc_lm_open,
	.read		= seq_read,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int __init lm_proc_init(void)
{
	proc_create("epdsensor", 0, NULL, &proc_lm_fops);
	return 0;

}
late_initcall(lm_proc_init);
#endif
#endif



