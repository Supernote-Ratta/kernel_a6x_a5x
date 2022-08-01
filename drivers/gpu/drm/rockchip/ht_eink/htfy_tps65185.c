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
#include <linux/htfy_dbg.h>
#include <linux/uaccess.h>
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

#include "htfy_pmic.h"
//#include "htfy_ebc.h" //rm for open-source.
#define TPS65185_I2C_NAME "tps65185"

#define PAPYRUS_VCOM_MAX_MV		0
#define PAPYRUS_VCOM_MIN_MV		-5110


#define BGB_PRINT              0

#if BGB_PRINT
#define tps65185_printk(fmt, args...) printk(KERN_INFO "[tps] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
//#define tps65185_printk(fmt, args...) dbg_printk(EDBG_POWER,"%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define tps65185_printk(fmt, args...)
#endif
static struct class *pmu_class;
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
	//struct timeval standby_tv;
	unsigned int v3p3off_time_ms;

	int 	epd_type;
	int 	power_fault_cnt;

	// 20201116: 在PX30上面，我们不需要 wakeup/enable/int GPIO.这些是以前 8951 上面用的。
	int pwr_up_pin;
	int error_pin;  // 20210511: 我们有时候会发现界面不显示，需要判断是不是65185供电异常.
	                // when error, output low.
	
	int vcom_ctl_pin;

	// 20210724: wakeup_pin 类似与 reset/sleep。可以复位 IC或者让IC进入
	// 休眠。 新的10寸屏需要修改65185的默认时序，我们改版用 I2C命令来控制，
	// 而不是用 pwr_up GPIO。新的策略一直保持 pwr_up 是低。
	int wake_up_pin;
	
	/* True if a high WAKEUP brings Papyrus out of reset. */
	int poweron_active_high;
	
	int wakeup_active_high;
	int vcomctl_active_high;
    //int eink3v3_active_high;
    //int einkint_active_high;

	// 20210724: 开机或者拉低 wakeup 之后，我们需要重新 初始化寄存器。
    bool 	need_init;
};

//#define tps65185_SPEED	(400*1000)

extern struct pmic_sess pmic_sess_data;

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
#define PAPYRUS_V3P3OFF_DELAY_MS 10  //100

struct papyrus_hw_state {
	uint8_t tmst_value;
	uint8_t int_status1;
	uint8_t int_status2;
	uint8_t pg_status;
};

static uint8_t papyrus2_i2c_addr = PAPYRUS2_1P1_I2C_ADDRESS;
int pmic_id = 0;
extern int sy7636a_power_on(void *priv);
extern int sy7636a_power_down(void *priv);
extern int sy7636a_set_vcom_voltage(void *priv,int vcom_mv);
extern int sy7636a_power_check(void *priv, int dump);
extern int sy7636a_reinit(void *priv, int epd_type);
extern int sy7636a_temperature_get(void *priv,int *temp);

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

#if 1
static int papyrus_hw_reset(struct papyrus_sess *sess)
{

  // 20210817: 如果出错了，我们需要重新初始化 power-up 寄存器，修改延迟。所以此处需要设置
  // need_init 标志。
  sess->need_init = true;
  
  if (gpio_is_valid(sess->vcom_ctl_pin))
	  gpio_set_value(sess->vcom_ctl_pin, !sess->vcomctl_active_high);
  if (gpio_is_valid(sess->pwr_up_pin))
	  gpio_set_value(sess->pwr_up_pin, !sess->poweron_active_high);
	  
  if (gpio_is_valid(sess->wake_up_pin)) {
	  gpio_set_value(sess->wake_up_pin, !sess->wakeup_active_high);
	  msleep(PAPYRUS_SLEEP_MINIMUM_MS);

	  gpio_set_value(sess->wake_up_pin, sess->wakeup_active_high);
	  msleep(PAPYRUS_EEPROM_DELAY_MS);
  }

  return 0;
}

#endif 

static void papyrus_hw_dump_reg(const char* stage, struct papyrus_sess *sess)
{
    int stat = 0;
	uint8_t	upseq0, upseq1, dwnseq0, dwnseq1, enx;//, vadj;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ0, &upseq0);
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ1, &upseq1);
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_DWNSEQ0, &dwnseq0);
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_DWNSEQ1, &dwnseq1);
	//stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VADJ, &vadj);
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_ENABLE, &enx);

    tps65185_printk("%s:upseq0=0x%x/0x%x,upseq1=0x%x/0x%x,dwseq0=0x%x/0x%x,dwseq1=0x%x/0x%x,epd=0x%x,enx=0x%x/0x%x,failed=%d\n", 
        stage, upseq0, sess->upseq0, upseq1, sess->upseq1, 
        dwnseq0, sess->dwnseq0, dwnseq1, sess->dwnseq1, sess->epd_type, 
        enx, sess->enable_reg_shadow, sess->power_fault_cnt);
}

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
    int stat = 0;
    //uint8_t ups0,ups1, ints1, ints2, pwrg;
    //int     cnt = 0;

    // 20210513-LOG: upseq0=0xe4,upseq1=0x55,dwseq0=0x1e,dwseq1=0xe0,adj=0x3,stat=0
    // 20210608-LOG: hw_arg_init:upseq0=0xe4,upseq1=0x0,dwseq0=0x1e,dwseq1=0x0,adj=0x3,stat=0
    // 20210727: hwinit1:upseq0=0xe1/0xe1,upseq1=0x0/0x0,dwseq0=0x1e/0x1e,dwseq1=0x0/0x0,epd=0x1,enx=0x0/0x40
	//if(BGB_PRINT) papyrus_hw_dump_reg(sess);
	sess->vadj = 0x03;
	
	sess->upseq0 = SEQ_VEE(0) | SEQ_VNEG(1) | SEQ_VPOS(2) | SEQ_VDD(3);

	// 20210727：实际测试发现，在高压电路上面加上下拉电阻之后，POWER-FAULT 的现象就不会
	// 发生了。即使这个时候用 0X00也是OK的。如果不加电阻，配置为 0XFF 也很快就会上电异常。
	#if 1

	sess->upseq1 = 0x00;
			// 0xFF;	// 20210727: 0X0F test OK(OK是因为没有进行上下电)
		//UDLY_12ms(0) | UDLY_3ms(1) | UDLY_3ms(2) | UDLY_3ms(3); // 0XFF; //
	#else
	if(sess->power_fault_cnt == 0) {
		sess->upseq1 =  0x00;
	} else if( sess->power_fault_cnt == 1) {
		sess->upseq1 =  0x55;
	} else if( sess->power_fault_cnt == 2) {
		sess->upseq1 =  0xAA;
	} else {
		sess->upseq1 =  0xFF;
	}
	#endif 
	
	// 20210722: 下电时序不动，只改上电。
	sess->dwnseq0 = SEQ_VDD(0) | SEQ_VPOS(1)  | SEQ_VNEG(2) | SEQ_VEE(3);
	sess->dwnseq1 = DDLY_6ms(0) | DDLY_6ms(1) | DDLY_6ms(2) | DDLY_6ms(3);

	//sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(2500) & 0x00FF);
	//sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(2500) & 0x0100) >> 8);
	//sess->enable_reg_shadow = 0;

	//if(BGB_PRINT && htfy_gdb_enable(EDBG_POWER)) papyrus_hw_dump_reg("hwinit0",sess);
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	//msleep(50);
	
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);

	// 20210727: 修改流程，在 standby 的情况下设置寄存器，没有发现 写失败的情况。
	// standy 状态通过在 WAKEUP 是高的情况下，保持 pwr-on 是低来进入。
#if 0
	while(true) {
		// set UPSEQs & DWNSEQs
		stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
		stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
		stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
		stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);
		stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);

		stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ0, &ups0);
		stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_UPSEQ1, &ups1);

		if(stat || sess->upseq0 != ups0 || sess->upseq1 != ups1) {
			//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
			if(BGB_PRINT) papyrus_hw_dump_reg("hwinitX",sess);
			stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS1, &ints1);
			stat |= papyrus_hw_getreg(sess,
					PAPYRUS_ADDR_INT_STATUS2, &ints2);
			stat |= papyrus_hw_getreg(sess,
					PAPYRUS_ADDR_PG_STATUS, &pwrg); // power-good-status.
			printk("int1=0x%x,int2=0x%x,pwrg=0x%x--reset by wakeup!!\n", ints1, ints2, pwrg);
			
			// 20210724：此处需要复位 65185 再试试。
			// papyrus_hw_reset(sess);
			if(cnt > 3) {
				dump_stack();
				break;
			}
			
		} else {
			// 20210724: 初始化 OK. 
			sess->need_init = false;
			break;
		}
	}
#else
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);
#endif 
    if (stat) {
		pr_err("papyrus: set UPSEQ-DOWNSEQ failed,i2c error!\n");
	} 
    // 20210513-LOG: upseq0=0xe4,upseq1=0x55,dwseq0=0x1e,dwseq1=0x0,adj=0x3,stat=0
    // 20210608-LOG: upseq0=0xe4,upseq1=0x0,dwseq0=0x1e,dwseq1=0x0,adj=0x3,stat=0
	if(BGB_PRINT) papyrus_hw_dump_reg("hwinit1", sess); // && htfy_gdb_enable(EDBG_POWER)

	// gpio_direction_output(sess->pwr_up_pin, !sess->poweron_active_high);
	return stat;
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

    //tps65185_printk("%s\n", __func__);
	if(sess->need_init) {
		papyrus_hw_arg_init(sess);
	}
	
	if (sess->vcom_ctl_pin != INVALID_GPIO) {
	   gpio_direction_output(sess->vcom_ctl_pin, sess->vcomctl_active_high);
	}

	// 20210513: TPS65185是一直供电的，只有深度休眠的时候，WAKEUP 才会拉低，其他都是维持为高。
	// POWER-UP 接到的是 65185的 PWRUP 管脚。所以此处I2C随时可以访问。从深度休眠唤醒的时候，WAKEUP
	// 自动就拉高。 深度休眠 WAKEUP 会自动拉低，此时 65185 进入 SLEEP 模式，此时不能接收I2C命令，内部寄存器
	// 会回复到默认值。
	// STANDY: WAKEUP 拉高，PWR 拉低或者 或者 STANDBY bit 设置。 这种模式可以接收I2C命令，但是没有电压输出。
	//  出现异常情况（比如输出电压异常），65185也会自动从 ACTIVE 变为 STANDBY 模式。
	// ACTIVE: The device is in ACTIVE mode when any of the output rails are enabled and no fault condition is present
	// 模式转换： 当 WAKEUP/PWR 一起拉高，可以从 SLEEP 直接进入 ACTIVE.
	//  WAKUP 高， PWR 低， 从 SLEEP 进入 STANDBY;  WAKEUP 高的情况下，拉高 PWR(上升沿)或者设置ACTIVE bit，就可以进入ACTIVE
	// 模式，Output rails will power up in the order defined by the UPSEQx registers。
	// WAKEUP 高， PWR 从高到低，则 从 ACTIVE 进入 STANDY 模式。如果发生其他的 电源异常，也会进入 STANDY 模式。
	// WAKEUP 第，设备从当前模式（ACTIVE/STANDY)进入 SLEEP模式。
	// 出现电压异常的条件： FAULT = UVLO || TSD || BOOST UV|| VCOM fault 
	//msleep(2);

    // 20210513: 我们通过 PWR IO 的上升沿来打开电源就可以了，不需要写寄存器。
    // 20210608: 不跑下面的流程，会导致手写的功耗偏大，本来EPD的耗电是30ma，变为了 150ma。
	//if (sess->eink_3v3_pin != INVALID_GPIO){
	//    gpio_direction_output(sess->eink_3v3_pin, sess->eink3v3_active_high);
	//}

#if 0
	// set VADJ
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);

	// set UPSEQs & DWNSEQs
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);

	// commit it, so that we can adjust vcom through "Rk_ebc_power_control_Release_v1.1"
	// 20191005: update from RK source code. --20210513: 这个 VCOM值已经写到了65185的内部ROM里面，会自动加载。
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, sess->vcom1);
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, sess->vcom2);
#endif 

#if 1
	/* switch to active mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive,
	 * don't enable vcom buffer -- 20210608：核心是下面的语句！！
	 */
	sess->enable_reg_shadow = (0x80 | 0x30 | 0x0F);  // 0x80: STANDY TO ACTIVE, POWER-UP.
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error at pwr up: %d\n", stat);
#else 
	
	/* Enable 3.3V switch to the panel */
	sess->enable_reg_shadow |= V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	msleep(sess->v3p3off_time_ms);

	/* switch to active mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive,
	 * don't enable vcom buffer -- 20210608：核心是下面的语句！！
	 */
	sess->enable_reg_shadow = (0x80 | 0x30 | 0x0F);  // 0x80: STANDY TO ACTIVE, POWER-UP.
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
#endif

    //20220424: 用于示波器抓波形分析 POWER-GOOD 拉高的时间.--示波器对比目前配置，从 pweron 拉高
    // 到 power-good 拉高，大概是 28ms。即在 30ms之内。
    //if (sess->pwr_up_pin != INVALID_GPIO) {
	//   gpio_direction_output(sess->pwr_up_pin, sess->poweron_active_high);
	//}
	
    // 20210512: 从示波器看看，CLK有信号的时候，VCOM还没有输出，需要延迟 5MS.
    // 20210608: 我们在 ebc 里面等待 POWER-GOOD,此处需要进行延迟了。
    // msleep(5);
	// if(BGB_PRINT) papyrus_hw_dump_reg("power-upx",sess);
	return;
}

static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
	
    // 20210513: 我们通过 PWR IO 的下降沿来打开电源就可以了，不需要写寄存器。
    // 20210611: 打开下面的设置寄存器的代码，似乎会导致设备放置一段时间后变为白屏。直接
    // 用 GPIO 口来控制就可以了。
#if 1
	int stat;
	/* switch to standby mode, keep 3.3V & VEE & VDDH & VPOS & VNEG alive,
	 * don't enable vcom buffer
	 */
	sess->enable_reg_shadow = (0x40 | 0x00 | 0x0F);  // 20191005,update from RK source code.
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	//printk("===========papyrus_hw_send_powerdown \n");
	//do_gettimeofday(&sess->standby_tv);
	if (stat)
		pr_err("papyrus: I2C error at pwr down: %d\n", stat);

	//if (sess->pwr_up_pin != INVALID_GPIO){
	//   gpio_direction_output(sess->pwr_up_pin, !sess->poweron_active_high);
	//}

    // 20210513: datasheet 里面提到，即使在下点的时候需要重新上电，IC内部也会控制，不会有问题。
	//msleep(30); // 20210513: wait powr-down-seq done.

	// 20210724: 外面直接关闭 vcom 是不是可能会造成残影？
	if (sess->vcom_ctl_pin != INVALID_GPIO){
	  gpio_direction_output(sess->vcom_ctl_pin, !sess->vcomctl_active_high);
	}
	
	//if (sess->eink_3v3_pin != INVALID_GPIO){
	//    gpio_direction_output(sess->eink_3v3_pin, !sess->eink3v3_active_high);
	//}

	// 20210817：下面的改动会导致正压的下电有点异常。
	//msleep(10);
	//sess->enable_reg_shadow &= ~(V3P3_EN_MASK|0x10);
	//stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	//if (stat)
	//	pr_err("papyrus: I2C error: %d\n", stat);
		
#else 
	int stat;
	sess->enable_reg_shadow = (0x40 | 0x20 | 0x0F);
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	
#if 1
	/* 3.3V switch must be turned off last */
	msleep(sess->v3p3off_time_ms);
	sess->enable_reg_shadow &= ~V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE, sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
#endif
#endif 

    //20220424: 用于示波器抓波形分析 POWER-GOOD 拉高的时间. --参考 papyrus_hw_send_powerup 里面的说明.
    //if (sess->pwr_up_pin != INVALID_GPIO) {
	//   gpio_direction_output(sess->pwr_up_pin, !sess->poweron_active_high);
	//}
	
	//msleep(100); // 20210513: wait for discharging done.
	//if(BGB_PRINT) papyrus_hw_dump_reg("power-down",sess);
}

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
	//temperature = tb;

	tps65185_printk("current temperature is %d\n",*t);

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

// class node
// cat /sys/class/pmu/temperature
static ssize_t papyrus_class_temperature_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
    struct pmic_sess *psess = (struct pmic_sess *)&pmic_sess_data;

    int temperature = 25;
	ssize_t len = 0;
	//u8 val = 0;
	papyrus_hw_read_temperature(psess,&temperature);
    len += sprintf(_buf, "%d:\n",temperature);
	return len;
}


static ssize_t papyrus_class_temperature_store(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
{

	return 0;
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

static CLASS_ATTR(temperature, 0644, papyrus_class_temperature_show, papyrus_class_temperature_store);

static int papyrus_hw_init(struct papyrus_sess *sess, const char *chip_id)
{
	int stat = 0;
	int ret;
    if((sess->pwr_up_pin!= INVALID_GPIO)) {
        stat |= gpio_request(sess->pwr_up_pin, "papyrus-power-on");
    }

	if((sess->vcom_ctl_pin!= INVALID_GPIO))
			stat |= gpio_request(sess->vcom_ctl_pin, "papyrus-vcom-ctl");

    if((sess->error_pin != INVALID_GPIO))
			stat |= gpio_request(sess->error_pin, "papyrus-error");
			

	if((sess->wake_up_pin!= INVALID_GPIO))
		stat |= gpio_request(sess->wake_up_pin, "papyrus-wake_up");
	
	/*
    if((sess->eink_3v3_pin!= INVALID_GPIO))
		stat |= gpio_request(sess->eink_3v3_pin, "eink3v3-power-on");
    if((sess->eink_int_pin!= INVALID_GPIO))
		stat |= gpio_request(sess->eink_int_pin, "eink-int");
	*/

	if (stat) {
		pr_err("papyrus: cannot reserve GPIOs\n");
		stat = -ENODEV;
		return stat;
	}
	sess->poweron_active_high = 1;
	sess->vcomctl_active_high = 1;
	sess->wakeup_active_high = 1;
    //sess->eink3v3_active_high = 1;
    //sess->einkint_active_high = 0;
	
	
	if((sess->wake_up_pin != INVALID_GPIO)) {
		//gpio_direction_output(sess->wake_up_pin, !sess->wakeup_active_high);
		//msleep(PAPYRUS_SLEEP_MINIMUM_MS);
		gpio_direction_output(sess->wake_up_pin, sess->wakeup_active_high);
		sess->need_init = true;
	}

	
	gpio_direction_output(sess->vcom_ctl_pin, !sess->vcomctl_active_high);

    if (sess->error_pin != INVALID_GPIO)
		gpio_direction_input(sess->error_pin);
		
    /*if (sess->eink_3v3_pin != INVALID_GPIO)
        gpio_direction_output(sess->eink_3v3_pin, sess->eink3v3_active_high);
    if (sess->eink_int_pin != INVALID_GPIO) {
        gpio_direction_output(sess->eink_int_pin, sess->einkint_active_high);
    }*/

	msleep(PAPYRUS_EEPROM_DELAY_MS);
	
	if (sess->pwr_up_pin != INVALID_GPIO)
		gpio_direction_output(sess->pwr_up_pin, !sess->poweron_active_high);


	stat = papyrus_hw_get_revid(sess);
	if (stat < 0)
		goto cleanup_i2c_adapter;
	pr_info("papyrus: detected device with ID=%02x (TPS6518%dr%dp%d)\n",
			stat, stat & 0xF, (stat & 0xC0) >> 6, (stat & 0x30) >> 4);
    if((stat == 0x65)||(stat == 0x66)){
        pmic_id = 0x6518;
    }
	stat = 0;
	pmu_class = class_create(THIS_MODULE, "pmu");
	ret =  class_create_file(pmu_class, &class_attr_temperature);
	if (ret)
		pr_info("Fail to create class pmu_temperature.\n");

	return stat;

cleanup_i2c_adapter:
	i2c_put_adapter(sess->adap);
//free_gpios:
	if (sess->wake_up_pin != INVALID_GPIO)
		gpio_free(sess->wake_up_pin);
    if (sess->vcom_ctl_pin != INVALID_GPIO)
	    gpio_free(sess->vcom_ctl_pin);
	if (sess->pwr_up_pin != INVALID_GPIO)
		gpio_free(sess->pwr_up_pin);
    if (sess->error_pin != INVALID_GPIO)
		gpio_free(sess->error_pin);
    /*if (sess->eink_3v3_pin != INVALID_GPIO)
		gpio_free(sess->eink_3v3_pin);
    if (sess->eink_int_pin != INVALID_GPIO)
		gpio_free(sess->eink_int_pin);
	*/
	pr_err("papyrus: ERROR: could not initialize I2C papyrus!\n");
	return stat;
}

static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
	struct papyrus_sess *sess = pmsess->drvpar;
    // tps65185_printk("powerup=%d\n", up);

	//pr_debug("papyrus: i2c pwr req: %d\n", up);
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
	if (sess->vcom_ctl_pin != INVALID_GPIO)
	    gpio_free(sess->vcom_ctl_pin);
	if (sess->pwr_up_pin != INVALID_GPIO)
		gpio_free(sess->pwr_up_pin);

    if (sess->error_pin != INVALID_GPIO)
            gpio_free(sess->error_pin);
            
	if (sess->wake_up_pin != INVALID_GPIO)
		gpio_free(sess->wake_up_pin);

    //if (sess->error_pin != INVALID_GPIO)
	//	gpio_free(sess->error_pin);
	/*
    if (sess->eink_3v3_pin != INVALID_GPIO)
		gpio_free(sess->eink_3v3_pin);
    if (sess->eink_int_pin != INVALID_GPIO)
		gpio_free(sess->eink_int_pin);
	*/
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

#if 0
static bool papyrus_standby_dwell_time_ready(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct timeval current_tv;
	long total_secs = 0;

	do_gettimeofday(&current_tv);
	mb();
	//total_secs = current_tv.tv_sec - sess->standby_tv.tv_sec;

	if (total_secs < PAPYRUS_STANDBY_DWELL_TIME)
		return false;

	return true;
}
#endif 

// 20210513: 每次拉高 WAKEUP,我们需要重新初始化。目前由于WAKEUP 和 VCCIO一起，调用的场合只有两个:
// 开机的时候和灭屏休眠唤醒的时候。
static int tps65185_reinit(void *priv, int epd_type)
{
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
    sess->epd_type = epd_type;
    return papyrus_hw_arg_init(sess);
}


//static int nWakeFormIdleMode = 0;

static int papyrus_pm_sleep(struct pmic_sess *sess)
{
	struct papyrus_sess *s = sess->drvpar;


	//nWakeFormIdleMode = 0;
	// 20210817-LOG: *EDBG*papyrus_pm_sleep(929):  enter sleep,wakeup gpio=-1
	tps65185_printk(" enter sleep,wakeup gpio=%d,fb off=%d\n", s->wake_up_pin, fb_power_off());

	//if(support_tps_3v3_always_alive()){
		//papyrus_hw_send_powerdown(s);
	//	return 0;
	//}

	gpio_direction_output(s->vcom_ctl_pin, !s->vcomctl_active_high);
	
	// 20210724: 拉低 wake_up, 65185 直接进入 sleep 状态。
	if (s->wake_up_pin != INVALID_GPIO) {
		gpio_direction_output(s->wake_up_pin, !s->wakeup_active_high);
		s->need_init = true;
	} else {
		s->need_init = fb_power_off();
	}
	
    //if (s->pwr_up_pin != INVALID_GPIO){
    //    gpio_direction_output(s->pwr_up_pin, !s->poweron_active_high);
    //}
    
    //if (s->eink_3v3_pin != INVALID_GPIO){
    //    gpio_direction_output(s->eink_3v3_pin, !s->eink3v3_active_high);
    //}

	return 0;
}

static int papyrus_pm_resume(struct pmic_sess *sess)
{
	struct papyrus_sess *s = sess->drvpar;
    tps65185_printk(" exit sleep,re-init=%d!\n", s->need_init);

	//if(support_tps_3v3_always_alive()){
		//papyrus_hw_send_powerup(s);
	//	return 0;
	//}
	if (s->wake_up_pin != INVALID_GPIO) {
		gpio_direction_output(s->wake_up_pin, s->wakeup_active_high);
	}

	// 20210817: 我们在开始显示的时候再进行初始化，因为这里拉高 wakeup 后可能需要
	// 等待一点时间 I2C 通信才正常。
	//if(s->need_init) {
	//	return papyrus_hw_arg_init(s);
	//}
	//gpio_direction_output(s->vcom_ctl_pin, s->vcomctl_active_high);

	// 20210724：休眠唤醒我们也让 pwr_up 是低，只需要拉高 wakeup 让 65185 进入 standy。此时需要
	// 重新配置寄存器。
    //if (s->pwr_up_pin != INVALID_GPIO){
    //    gpio_direction_output(s->pwr_up_pin, s->poweron_active_high);
    //}
    
    /*if (s->eink_3v3_pin != INVALID_GPIO){
        gpio_direction_output(s->eink_3v3_pin, s->eink3v3_active_high);
    }*/

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

	//papyrus_hw_arg_init(sess); // //20210513: call by tps65185_reinit

	//if (pmsess->v3p3off_time_ms == -1)
		sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;
	//else
	//	sess->v3p3off_time_ms = pmsess->v3p3off_time_ms;

	// do_gettimeofday(&sess->standby_tv);

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


	sess->wake_up_pin = of_get_named_gpio_flags(node, "wakeup_pin", 0, &flags);
	if (!gpio_is_valid(sess->wake_up_pin)) {
		sess->wake_up_pin = INVALID_GPIO;
		pr_err("tsp65185: failed to find wakeup pin\n");
		//goto free_sess;
	}

#if 0
	sess->eink_3v3_pin = of_get_named_gpio_flags(node, "eink_3v3_pin", 0, &flags);
	if (!gpio_is_valid(sess->eink_3v3_pin)) {
		sess->eink_3v3_pin = INVALID_GPIO;
		pr_err("tsp65185: failed to find eink_3v3 pin\n");
	}
#endif	


    sess->error_pin = of_get_named_gpio_flags(node, "error_pin", 0, &flags);
	if (!gpio_is_valid(sess->error_pin)) {
		sess->error_pin = INVALID_GPIO;
		pr_err("tsp65185: failed to find error_pin\n");
	}

	// 20210727: 下面的函数主要控制 GPIO 口。
	stat = papyrus_hw_init(sess, pmsess->drv->id);
	if (stat)
		goto free_sess;

	// 20210723: enter standy.
	sess->enable_reg_shadow = 0x40;
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	if (stat)
		goto free_sess;

	pmsess->drvpar = sess;
	pmsess->revision = papyrus_hw_get_revid(sess);
    printk("papyrus_probe 65185:%d \n",pmsess->revision);

	// 20210727: 由于我们需要根据不同的屏幕来调整时序，所以有 ebc 驱动传递 epd_type 的时候
	// 再进行初始化。
	// tps65185_reinit(pmsess); 
	return 0;

free_sess:
	if (gpio_is_valid(sess->vcom_ctl_pin)){
		sess->vcom_ctl_pin = INVALID_GPIO;
	}
    if (gpio_is_valid(sess->pwr_up_pin)){
        sess->pwr_up_pin = INVALID_GPIO;
    }
    if (gpio_is_valid(sess->wake_up_pin)){
        sess->wake_up_pin = INVALID_GPIO;
    }
	if (gpio_is_valid(sess->error_pin)){
        sess->error_pin = INVALID_GPIO;
	}
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

	//.hw_standby_dwell_time_ready = papyrus_standby_dwell_time_ready,
	.hw_pm_sleep = papyrus_pm_sleep,
	.hw_pm_resume = papyrus_pm_resume,
};

#if 0
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
#endif 

static int tps65185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	// int ret;

	tps65185_printk("tps65185_probe: I2C addr:%x\n", client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("I2C check functionality failed.");
		return -ENODEV;
	}

	if(0 != pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data,client))
	{
		pmic_id = 0;
		printk("pmic_driver_tps65185_i2c hw_init failed.");
		return -ENODEV;
	}
	//pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);

	pmic_sess_data.is_inited = 1;

	// 20210727: 我们通过 privdata 命令来设置 VCOM,此处不需要 sys.
#if 0
	tps65185_kobj = kobject_create_and_add("tps65185", NULL);
	if (tps65185_kobj) {
		ret = sysfs_create_file(tps65185_kobj, &tps65185_attrs.attr);
		if (ret)
			dev_err(&client->dev, "create tps65185 sysfs error\n");
	}
#endif 
	//tps65185_printk("tps65185_probe ok.\n");

	return 0;
}

static int tps65185_remove(struct i2c_client *client)
{
	pmic_driver_tps65185_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
	memset(&pmic_sess_data,0,sizeof(struct pmic_sess));
	return 0;
}


static int tps65185_suspend(struct device *dev/*struct i2c_client *client, pm_message_t mesg*/)
{
	// 20210817: 系统进入休眠的时候，目前的硬件会自动拉低 wakeup,从而让 65185进入休眠状态。
	// 但是 SNX的设备可能控制了 wakeup. 进入休眠需要我们控制 wakeup。此处统一控制可以兼容。
	return pmic_driver_tps65185_i2c.hw_pm_sleep((struct pmic_sess *)&pmic_sess_data);
}
static int tps65185_resume(struct device *dev/*struct i2c_client *client*/)
{
	return pmic_driver_tps65185_i2c.hw_pm_resume((struct pmic_sess *)&pmic_sess_data);
}


static const struct i2c_device_id tps65185_id[] = {
	{ TPS65185_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id tps65185_dt_ids[] = {
	{ .compatible = "ti,tps65185", },
	{ /* sentinel */ }
};


static const struct dev_pm_ops tps65185_pm_ops = {
	.resume = tps65185_resume,
	.suspend = tps65185_suspend,

	// 20191224,hsl add.
	//.suspend_noirq = htfy_eink_suspend_noirq,
	//.resume_noirq = htfy_eink_resume_noirq,

	// 20201026: for screen-on suspend.
	// .prepare = htfy_pm_prepare,
};


MODULE_DEVICE_TABLE(of, tps65185_dt_ids);
static struct i2c_driver tps65185_driver = {
	.probe	= tps65185_probe,
	.remove 	= tps65185_remove,
	.id_table	= tps65185_id,
	.driver = {
		.of_match_table = tps65185_dt_ids,
		.name	  = TPS65185_I2C_NAME,
		.owner	  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &tps65185_pm_ops,
#endif
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

	tps65185_printk("tps65185_vcom_get enter.\n");
	if(!tpmic_sess_data.is_inited)
		return -1;
	// VERIFICATION
	/*if (sess->wake_up_pin != INVALID_GPIO) {
		gpio_direction_output(sess->wake_up_pin, 0);
		msleep(10);
		gpio_direction_output(sess->wake_up_pin, 1);
		msleep(10);
	}*/
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

static int tps65185_vcom_read(struct papyrus_sess *sess)
{
    int stat = 0;
    uint8_t rev_val1=0, rev_val2=0;
    int read_vcom_mv;
    
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST, &rev_val1);
	//tps65185_printk("rev_val1 = 0x%x\n",rev_val1);
	read_vcom_mv = rev_val1;
	stat |= papyrus_hw_getreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST, &rev_val2);
	//tps65185_printk("rev_val2 = 0x%x\n",rev_val2);
	read_vcom_mv += ((rev_val2 & 0x0001)<<8);

	read_vcom_mv *= 10; //20210512:unit is 10mv.
	tps65185_printk("read_vcom_mv = %d(vcom1=0x%x,vcom2=0x%x)\n", read_vcom_mv, 
	    rev_val1, rev_val2); 

	if (stat) {
		pr_err("papyrus: I2C error: %d\n", stat);
	}
	return read_vcom_mv;
}

// 20210513: 由于我们会编程到 ROM里面，所以只有VCOM改变的时候才需要重新编程。
int tps65185_vcom_set(int vcom_mv)
{
	//struct i2c_client *client=NULL;
	struct pmic_sess tpmic_sess_data = pmic_sess_data;
	struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;
	uint8_t rev_val = 0;
	int stat = 0;
	int read_vcom_mv;

	
	if(!tpmic_sess_data.is_inited) {
	    tps65185_printk("tps65185_vcom_set enter,vcom_mv=%d,but Not Init!!\n", vcom_mv);
		return -1;
	}
	/*if (sess->wake_up_pin != INVALID_GPIO)
		gpio_direction_output(sess->wake_up_pin, 1);
	msleep(10);
	*/

	// 20210513: read back first. if equal,need nothing.
	read_vcom_mv = tps65185_vcom_read(sess);

	// 20210513-LOG: *EDBG*tps65185_vcom_set(1187): tps65185_vcom_set,new vcom_mv=1550,read vcom=1550!!
	tps65185_printk("tps65185_vcom_set,new vcom_mv=%d,read vcom=%d!!\n", vcom_mv, read_vcom_mv);
	if(read_vcom_mv == vcom_mv) {
	    return 0;
	}
	
	// Set vcom voltage
	pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&tpmic_sess_data, vcom_mv);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM1_ADJUST,sess->vcom1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,sess->vcom2);
	tps65185_printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);

	// PROGRAMMING  -- 20210513: 下面的代码把 VCOM 写到 65185 内部的ROM里面，每次上电会重置。
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
	// tps65185_printk("sess->vcom1 = 0x%x sess->vcom2 = 0x%x\n",sess->vcom1,sess->vcom2);
	/*if (sess->wake_up_pin != INVALID_GPIO) {
		gpio_direction_output(sess->wake_up_pin, 0);
		msleep(10);
		gpio_direction_output(sess->wake_up_pin, 1);
		msleep(10);
	}*/
	read_vcom_mv = tps65185_vcom_read(sess);
	tps65185_printk("set vcomd done,read_vcom_mv = %d,set vcom=%d\n",read_vcom_mv, vcom_mv); 

	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);

	return 0;
}

static int tps65185_set_vcom_voltage(void *priv, int vcom_mv)
{
	if (vcom_mv < 0 || vcom_mv > 5110) {
		printk("tps: err vcom value, vcom value shoule be 0~5110\n");
		return -1;
	}
	return tps65185_vcom_set(vcom_mv);
    //if(pmic_sess_data.is_inited)
    //        pmic_driver_tps65185_i2c.set_vcom_voltage((struct pmic_sess *)&pmic_sess_data, vcom_mv);
   //return 0;
}

static int tps65185_power_on(void *priv)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);
	return 0;
}
static int tps65185_power_down(void *priv)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,0);
	return 0;
}

// 20210512: BITS OF power-good-satus
#define VNEG_PG         (1<<1)
#define VEE_PG          (1<<3)
#define VPOS_PG         (1<<4)
#define VN_PG           (1<<5)
#define VDDH_PG         (1<<6)
#define VB_PG           (1<<7)
#define ALL_POWER_GOOD      (VNEG_PG|VEE_PG|VPOS_PG|VN_PG|VDDH_PG|VB_PG)

// 20210512: BITS OF INT1-STATUS
#define INTS1_TSD       (1<<6)      // Thermal shutdown interrupt
#define INTS1_UVLO      (1<<2)      // VIN under voltage detect interrupt

// 20210512: BITS OF INT2-STATUS
#define INTS2_VB_UV         (1<<7)      // under-voltage on DCDC1 detected
#define INTS2_VDDH_UV       (1<<6)      // under-voltage on VDDH charge pump detected
#define INTS2_VN_UV         (1<<5)      // under-voltage on DCDC2 detected
#define INTS2_VPOS_UV       (1<<4)      // under-voltage on LDO1(VPOS) detected
#define INTS2_VEE_UV        (1<<3)      // under-voltage on VEE charge pump detected
#define INTS2_VCOMF         (1<<2)      // fault on VCOM detected (VCOM is outside normal operating range)
#define INTS2_VNEG_UV       (1<<1)      // under-voltage on LDO2(VNEG) detected

#define ANY_PWR_FAULT   (INTS2_VB_UV|INTS2_VDDH_UV|INTS2_VN_UV|INTS2_VPOS_UV|INTS2_VEE_UV|INTS2_VCOMF|INTS2_VNEG_UV)


// return 1: check POWER GOOD; 0: check power fail but Not-Fault.
// -1: power Fault, needs reset. only check after power-on.
static int tps65185_power_check(void *priv, int timeout)
{
    int stat;
    uint8_t ints1, ints2, pwrg;
    int err_pin_value = -1;
    struct papyrus_sess *sess = (struct papyrus_sess *)pmic_sess_data.drvpar;

    // Once a fault is detected,the PWR_GOOD and nINT pins are pulled low and the corresponding 
    // interrupt bit is set in the interrupt register.
    if (sess->error_pin != INVALID_GPIO){
        err_pin_value = gpio_get_value(sess->error_pin);
        //pr_err("power_check: power-good(gpio%d)=%d\n", sess->error_pin, err_pin_value);
	   if(err_pin_value == 1) { // if high, means power-good.
	        return 1;
	   }

	   // 20211227：我们在 SNX A6X 上面发现上电异常，原因是在 65185上电的时候，不能通过I2C访问
	   // 里面的寄存器（尤其是 INT1/INT2),否则容易造成上电异常(FAULT)。所以此处如果有GPIO，直接返回，除非TIMEOUT
	   // 了才进行访问,判断是否是真正的异常。
	   if(!timeout) return 0;
	}
	
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &pwrg); // power-good-status.
	if (stat) {
		pr_err("papyrus: I2C error: stat = %d\n", stat);
		return -1;
	} else {
		if((pwrg & 0X000000FA) == 0X000000FA) {
			return 1;
		}
		// 20210724: 如果已经出现 falt了，则不需要再等待了。
	    if(timeout) {
	    	// 20210512: 如果电源异常，我们通过读取 INT_STATUS1 和 INT_STATUS2 来清除异常。
			stat |= papyrus_hw_getreg(sess,
						PAPYRUS_ADDR_INT_STATUS1, &ints1);
			stat |= papyrus_hw_getreg(sess,
						PAPYRUS_ADDR_INT_STATUS2, &ints2);
	    	pr_err("papyrus: Power_Fault,fcnt=%d,PG=0X%02X,INT1=0X%02X,INT2=0X%02X\n", 
	    		sess->power_fault_cnt, pwrg, ints1, ints2);
	    	sess->power_fault_cnt++;
	    	
			 if(ints2 & ANY_PWR_FAULT) {
		    	// 20211227: don't reset,just power-up again!!
		    	papyrus_hw_reset(sess);
		    	//msleep(PAPYRUS_EEPROM_DELAY_MS);

		    	papyrus_hw_send_powerup(sess);
	    	}
	    } 
	}
	return 0;
}


static int tps65185_temperature_get(void *priv, int *temp)
{
#if 1  // 20191006,hsl just for compare RK-HT driver.
	if(pmic_sess_data.is_inited)
		return pmic_driver_tps65185_i2c.hw_read_temperature((struct pmic_sess *)&pmic_sess_data,temp);
	else
		return 0;
#else
	*temp = 27;
	return 0;
#endif
}

//int register_ebc_pwr_ops(struct ebc_pwr_ops *ops)
__weak
int htfy_register_ebc_pwr_ops(struct htfy_pwr_ops *ops)
{
    printk("===============htfy_register_ebc_pwr_ops:%d\n",pmic_id);
	if(pmic_id != 0x6518){
	    ops->priv = &pmic_sess_data;
		ops->power_on = sy7636a_power_on;
		ops->power_down = sy7636a_power_down;
		ops->vcom_set = sy7636a_set_vcom_voltage;
		ops->power_check = sy7636a_power_check;
		ops->reinit = sy7636a_reinit;
	}else{
		ops->priv = &pmic_sess_data;
    	ops->power_on = tps65185_power_on;
    	ops->power_down = tps65185_power_down;
    	ops->vcom_set = tps65185_set_vcom_voltage;
    	ops->power_check = tps65185_power_check;
    	ops->reinit = tps65185_reinit;
	}
	return 0;
}


//int register_ebc_temp_ops(struct ebc_temperateure_ops *ops)
__weak
int htfy_register_ebc_temp_ops(struct htfy_gettemp_ops *ops)
{
	ops->priv = &pmic_sess_data;
	if(pmic_id != 0x6518){
		ops->priv = &pmic_sess_data;
		ops->temperature_get = sy7636a_temperature_get;
	}
	else{
		ops->priv = &pmic_sess_data;
	ops->temperature_get = tps65185_temperature_get;
	}
	return 0;
}


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
	u32 value;
	if(pmic_id != 0x6518){
		sy7636a_temperature_get(&pmic_sess_data,&value);
	}
	else{
	tps65185_temperature_get(&pmic_sess_data, &value);
	}
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


