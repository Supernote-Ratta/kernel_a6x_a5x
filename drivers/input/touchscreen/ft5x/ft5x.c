/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *  note: only support mulititouch  Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>

#include <linux/interrupt.h>
#include <linux/delay.h>


#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>

#include <linux/init.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <linux/fb.h>
#include <linux/notifier.h>

#include "ft5x_ts.h"
#include <linux/pm_wakeirq.h>

#include <linux/htfy_dbg.h>  // 20191031,hsl add.20191107--for function-call and  fb_power_off.

//---------------------------------------------
//#define NO_ELINK
enum{
    DEBUG_INIT = 1U << 0,
    DEBUG_SUSPEND = 1U << 1,
    DEBUG_INT_INFO = 1U << 2,
    DEBUG_X_Y_INFO = 1U << 3,
    DEBUG_KEY_INFO = 1U << 4,
    DEBUG_WAKEUP_INFO = 1U << 5,
    DEBUG_OTHERS_INFO = 1U << 6,
};

static u32 debug_mask = DEBUG_SUSPEND;

#define dprintk(level_mask,fmt,arg...)   if( debug_mask & level_mask ) { \
	printk("***ft5x_ts_sunty***"fmt, ## arg); }

module_param_named(debug_mask,debug_mask,int,S_IRUGO | S_IWUSR | S_IWGRP);

//---------------------------------------------

#define TOUCH_POINT_NUM (5)
//#define CONFIG_SUPPORT_FTS_CTP_UPG
//#define FOR_TSLIB_TEST
//#define TOUCH_KEY_SUPPORT
#ifdef TOUCH_KEY_SUPPORT
#define TOUCH_KEY_FOR_EVB13
//#define TOUCH_KEY_FOR_ANGDA
#ifdef TOUCH_KEY_FOR_ANGDA
#define TOUCH_KEY_X_LIMIT           (60000)
#define TOUCH_KEY_NUMBER            (4)
#endif
#ifdef TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_LOWER_X_LIMIT         (848)
#define TOUCH_KEY_HIGHER_X_LIMIT    (852)
#define TOUCH_KEY_NUMBER            (5)
#endif
#endif


//FT5X02_CONFIG
#define FT5X02_CONFIG_NAME "fttpconfig_5x02public.ini"
extern int ft5x02_Init_IC_Param(struct i2c_client * client);
extern int ft5x02_get_ic_param(struct i2c_client * client);
extern int ft5x02_Get_Param_From_Ini(char *config_name);

struct Upgrade_Info {
    u16 delay_aa;       /*delay of write FT_UPGRADE_AA */
    u16 delay_55;       /*delay of write FT_UPGRADE_55 */
    u8 upgrade_id_1;    /*upgrade id 1 */
    u8 upgrade_id_2;    /*upgrade id 2 */
    u16 delay_readid;   /*delay of read id */
};

static struct i2c_client *this_client;

#ifdef TOUCH_KEY_SUPPORT
static int key_tp  = 0;
static int key_val = 0;
#endif

/*********************************************************************************************/
#define CTP_NAME             "ft5x_ts_sunty"

#define DEFAULT_SCREEN_MAX_X            (800)
#define DEFAULT_SCREEN_MAX_Y            (1280)
#define PRESS_MAX           (255)


/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS    5

#define FT_PRESS    0x08

#define FT_MAX_ID   0x0F
#define FT_TOUCH_STEP   6
#define FT_TOUCH_POINT_NUM      2
#define FT_TOUCH_X_H_POS        3
#define FT_TOUCH_X_L_POS        4
#define FT_TOUCH_Y_H_POS        5
#define FT_TOUCH_Y_L_POS        6
#define FT_TOUCH_XY_POS         7
#define FT_TOUCH_EVENT_POS      3
#define FT_TOUCH_ID_POS         5

#define POINT_READ_BUF  (3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT5x0x_REG_FW_VER       0xA6
#define FT5x0x_REG_POINT_RATE   0x88
#define FT5X0X_REG_THGROUP  0x80


/*********************************************************************************************/
/*------------------------------------------------------------------------------------------*/
/* Addresses to scan */
static const unsigned short normal_i2c[2] = {0x38,I2C_CLIENT_END};
static const int chip_id_value[] = {0x55,0x06,0x08,0x02,0xa3};
static int chip_id = 0;

/*------------------------------------------------------------------------------------------*/

int fts_ctpm_fw_upgrade_with_i_file(void);

static int ft5x_i2c_rxdata(char *rxdata, int length);

struct ts_event {
    u16 au16_x[CFG_MAX_TOUCH_POINTS];   /*x coordinate */
    u16 au16_y[CFG_MAX_TOUCH_POINTS];   /*y coordinate */
    u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];   /*touch event:
                    0 -- down; 1-- up; 2 -- contact(MOVE) */
    u8 au8_finger_id[CFG_MAX_TOUCH_POINTS]; /*touch ID */
    u8 au8_xy[CFG_MAX_TOUCH_POINTS];
    u16 pressure;
    u8 touch_point;
    int touchs;
    u8 touch_point_num;

};

struct ft5x_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;

    struct workqueue_struct *ts_workqueue;
    struct work_struct pen_event_work;
    struct work_struct init_events_work;
    struct work_struct resume_events_work;

    struct ts_event event;

    //dts node : power-supply
    //supply = devm_regulator_get(dev, "power");
    struct regulator *supply;
    struct gpio_desc *enable_gpio; //dts node: enable-gpios
    struct gpio_desc *reset_gpio; //dts node: reset-gpios
    struct gpio_desc *irq_gpio;// dts node: irq-gpios
    int irq;
    bool irq_enabled;
    struct mutex mutex_lock;

    // 20190225,hsl add dbg-class for debug.
    struct class *debug_class;

    // 20190225,hsl: add fb notify to shut down TP when fb off.
    struct notifier_block fb_notif;
    /*
    dts node:
        screen_max_x = <1536>;
        screen_max_y = <2048>;
        exchange_x_y_flag = <1>;
        revert_x_flag = <0>;
        revert_y_flag = <0>;
    */
    int screen_max_x;
    int screen_max_y;
    int revert_x_flag;
    int revert_y_flag;
    int exchange_x_y_flag;

	bool is_enable;
	bool is_fboff;
	bool after_resume;

};

//for we use  devm_gpiod_xxx
#define ENABLE_GPIO_ACTIVE_VALUE (1)
#define RESET_GPIO_ACTIVE_VALUE (0)
#define IRQ_GPIO_ACTIVE_VALUE (0)

struct ft5x_ts_data *g_data;

/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

static void delay_ms(FTS_WORD  w_ms)
{
    //platform related, please implement this function
    msleep( w_ms );
}

static void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
             udelay(1);
        }
    }
}


static int ft5x_reset(struct ft5x_ts_data *data)
{
    if (!data) return -ENOMEM;

    if (!data->reset_gpio) return -ENOMEM;

    gpiod_set_value(data->reset_gpio, RESET_GPIO_ACTIVE_VALUE);
    delay_ms(40);
    gpiod_set_value(data->reset_gpio, !RESET_GPIO_ACTIVE_VALUE);
    delay_ms(40);

    return 0;
}

static int ft5x_set_power_enabled(struct ft5x_ts_data *data, bool enabled)
{
    int ret = 0;
    if (!data) return -ENOMEM;

    if (enabled) {
        if (data->supply) {
            ret = regulator_enable(data->supply);
            if (!ret) {
                dprintk(DEBUG_OTHERS_INFO, "%s[%d]: OK to regulator_enable", __func__, __LINE__);
            } else {
                dprintk(DEBUG_OTHERS_INFO, "%s[%d]: Fail to regulator_enable", __func__, __LINE__);
            }
        }
        if (data->enable_gpio) {
            gpiod_set_value(data->enable_gpio, ENABLE_GPIO_ACTIVE_VALUE);
        }
    } else {
        if (data->supply) {
            ret = regulator_disable(data->supply);
        }
        if (data->enable_gpio) {
            gpiod_set_value(data->enable_gpio, !ENABLE_GPIO_ACTIVE_VALUE);
        }
    }

    return ret;
}

static void ft5x_set_irq_enabled(struct ft5x_ts_data *data, bool enabled)
{
    bool en;
    if (!data) return;
    if (!(data->irq_gpio && data->irq > 0)) return;


	mutex_lock(&data->mutex_lock);
	en = !!enabled;
	dprintk(DEBUG_INT_INFO, "%s, irq_enabled = %d, en = %d\n", __func__, data->irq_enabled, en);
	if (data->irq_enabled == en) {
		goto _out;
	}
	data->irq_enabled = en;

	if (en) {
		enable_irq(data->irq);
	} else {
		disable_irq(data->irq);
	}

_out:
	mutex_unlock(&data->mutex_lock);
}


/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
static int i2c_read_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
    int ret;

    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret != dw_lenth){
        printk("ret = %d. \n", ret);
        printk("i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret != dw_lenth){
        printk("i2c_write_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]:
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
u8 fts_register_read(u8 e_reg_name, u8* pbt_buf, u8 bt_len)
{
    u8 read_cmd[3]= {0};
    u8 cmd_len     = 0;

    read_cmd[0] = e_reg_name;
    cmd_len = 1;

    /*call the write callback function*/
    //    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
    //    {
    //        return FTS_FALSE;
    //    }


    if(!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))   {//change by zhengdixu
        return FTS_FALSE;
    }

    /*call the read callback function to get the register value*/
    if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len)){
        return FTS_FALSE;
    }
    return FTS_TRUE;
}

/*
[function]:
    write a value to register.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[in]        :the returned register value;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int fts_register_write(u8 e_reg_name, u8 bt_value)
{
    FTS_BYTE write_cmd[2] = {0};

    write_cmd[0] = e_reg_name;
    write_cmd[1] = bt_value;

    /*call the write callback function*/
    //return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, 2);
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2); //change by zhengdixu
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    //return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, num);
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);//change by zhengdixu
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_write(u8* pbt_buf, u16 dw_len)
{
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_read(u8* pbt_buf, u8 bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
    //ft5x_i2c_rxdata
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH       128 //2//4//8//16//32//64//128//256

static unsigned char CTPM_FW[]=
{
        #include "ft_app.i"
};
unsigned char fts_ctpm_get_i_file_ver(void)
{
        unsigned int ui_sz;
        ui_sz = sizeof(CTPM_FW);
        if (ui_sz > 2){
                return CTPM_FW[ui_sz - 2];
        }else{
                //TBD, error handling?
                return 0xff; //default value
        }
}

/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info *upgrade_info)
{
    switch (chip_id) {
    case 0x55:    //IC_FT5X06:
        upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
        upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
        upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
        upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
        upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
        break;
    case 0x08:    //IC_FT5606或者IC_FT5506
        upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
        upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
        upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
        upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
        upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
        break;
    case 0x00:    //IC FT5316
    case 0x0a:    //IC FT5316
        upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
        upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
        upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
        upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
        upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
        break;
    default:
        break;
    }
}

E_UPGRADE_ERR_TYPE  ft5x06_ctpm_fw_upgrade(u8* pbt_buf, u16 dw_lenth)
{
    u8 reg_val[2] = {0};
    FTS_BOOL i_ret = 0;
    u16 i = 0;


    u16  packet_number;
    u16  j;
    u16  temp;
    u16  lenght;
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    //u8  auc_i2c_write_buf[10];
    u8 bt_ecc;

    struct  Upgrade_Info upgradeinfo = {0, 0, 0, 0 , 0};


    fts_get_upgrade_info(&upgradeinfo);

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    //delay_ms(100);//最新的源码去掉延时
    fts_register_write(0xfc,0xaa);
    delay_ms(upgradeinfo.delay_aa);

    /*write 0x55 to register 0xfc*/
    fts_register_write(0xfc,0x55);
    printk("Step 1: Reset CTPM test\n");
    delay_ms(upgradeinfo.delay_55);

    /*********Step 2:Enter upgrade mode *****/
    //auc_i2c_write_buf[0] = 0x55;
    //auc_i2c_write_buf[1] = 0xaa;
    i = 0;
    do{
            i++;
            //i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
    cmd_write(0x55,0xaa,0x00,0x00,2);
            printk("Step 2: Enter update mode. \n");
            delay_ms(5);
    }while((FTS_FALSE == i_ret) && i<5);

    /*********Step 3:check READ-ID***********************/
    /*send the opration head*/
    msleep(upgradeinfo.delay_readid);
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == upgradeinfo.upgrade_id_1&& reg_val[1] == upgradeinfo.upgrade_id_2) {
        printk("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else {
        printk("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        return ERR_READID;
    }
    cmd_write(0xcd,0x00,0x00,0x00,1);
    byte_read(reg_val,1);

    /*Step 4:erase app and panel paramenter area*/
    cmd_write(0x61,0x00,0x00,0x00,1);
    msleep(2000);
    cmd_write(0x63,0x00,0x00,0x00,1);
    msleep(100);
    printk("Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j = 0; j < packet_number; j++){
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++){
                packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
                bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        //delay_ms(FTS_PACKET_LENGTH/6 + 1);
        msleep(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0){
                printk("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0){
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++){
                packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
                bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        //delay_ms(20);
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++){
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        //delay_ms(20);
        msleep(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    //cmd_write(0xcc,0x00,0x00,0x00,1);//把0xcc当作寄存器地址，去读出一个字节
    // byte_read(reg_val,1);//change by zhengdixu

    fts_register_read(0xcc, reg_val,1);
    printk("Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc){
            //cmd_write(0x07,0x00,0x00,0x00,1);
    printk("ecc error! \n");
    return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);
    msleep(300);
    return ERR_OK;
}

E_UPGRADE_ERR_TYPE  ft5x02_ctpm_fw_upgrade(u8* pbt_buf, u32 dw_lenth)
{

    u8 reg_val[2] = {0};
    u32 i = 0;

    u32  packet_number;
    u32  j;
    u32  temp;
    u32  lenght;
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    //u8    auc_i2c_write_buf[10];
    u8  bt_ecc;

    //struct timeval begin_tv, end_tv;
    //do_gettimeofday(&begin_tv);

    for (i=0; i<16; i++) {
        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register 0xfc*/
        fts_register_write(0xfc,0xaa);
        msleep(30);
        /*write 0x55 to register 0xfc*/
        fts_register_write(0xfc,0x55);
        //delay_qt_ms(18);
        delay_qt_ms(25);
        /*********Step 2:Enter upgrade mode *****/
        #if 0
        //auc_i2c_write_buf[0] = 0x55;
        //auc_i2c_write_buf[1] = 0xaa;
        do
        {
            i ++;
            //i_ret = ft5x02_i2c_Write(client, auc_i2c_write_buf, 2);
            //i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
            cmd_write(0x55,0xaa,0x00,0x00,2);
            delay_qt_ms(5);
        }while(i_ret <= 0 && i < 5 );
        #else
        //auc_i2c_write_buf[0] = 0x55;
        //ft5x02_i2c_Write(client, auc_i2c_write_buf, 1);
        cmd_write(0x55,0x00,0x00,0x00,1);
        delay_qt_ms(1);
        //auc_i2c_write_buf[0] = 0xaa;
        //ft5x02_i2c_Write(client, auc_i2c_write_buf, 1);
        cmd_write(0xaa,0x00,0x00,0x00,1);
        #endif

        /*********Step 3:check READ-ID***********************/
        delay_qt_ms(1);

        //ft5x02_upgrade_send_head(client);
        cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
        //auc_i2c_write_buf[0] = 0x90;
        //auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        //ft5x02_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);

        if (reg_val[0] == 0x79
            && reg_val[1] == 0x02) {
            //dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            break;
        } else {
            printk("[FTS] Step 3 ERROR: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            //delay_qt_ms(1);
        }
    }
    if (i >= 6)
        return ERR_READID;
    /********Step 4:enable write function*/
    //ft5x02_upgrade_send_head(client);
    cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
    //auc_i2c_write_buf[0] = 0x06;
    //ft5x02_i2c_Write(client, auc_i2c_write_buf, 1);
    cmd_write(0x06,0x00,0x00,0x00,1);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;

    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;

    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0; j<packet_number; j++) {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(lenght>>8);
        packet_buf[5] = (u8)lenght;
        if(temp>=0x4c00 && temp <(0x4c00+512))
            continue;

        for (i=0; i<FTS_PACKET_LENGTH; i++) {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }
        //ft5x02_upgrade_send_head(client);
        cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
        //ft5x02_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH+6);
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(2);
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;

        for (i=0; i<temp; i++) {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }
        //ft5x02_upgrade_send_head(client);
        cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
        //ft5x02_i2c_Write(client, packet_buf, temp+6);
        byte_write(&packet_buf[0],temp + 6);
        delay_qt_ms(2);
    }

    /********Disable write function*/
    //ft5x02_upgrade_send_head(client);
    cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
    //auc_i2c_write_buf[0] = 0x04;
    //ft5x02_i2c_Write(client, auc_i2c_write_buf, 1);
    cmd_write(0x04,0x00,0x00,0x00,1);

    delay_qt_ms(1);
    /*********Step 6: read out checksum***********************/
    //ft5x02_upgrade_send_head(client);
    cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
    //auc_i2c_write_buf[0] = 0xcc;
    //ft5x02_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);

    if (reg_val[0] != bt_ecc) {
        printk("[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
        //return -EIO;
        return ERR_READID;
    }

    /*********Step 7: reset the new FW***********************/
    //ft5x02_upgrade_send_head(client);
    cmd_write(0xFA,0xFA,0x00,0x00,2);//ft5x02_upgrade_send_head
    //auc_i2c_write_buf[0] = 0x07;
    //ft5x02_i2c_Write(client, auc_i2c_write_buf, 1);
    cmd_write(0x07,0x00,0x00,0x00,1);
    msleep(200);  /*make sure CTP startup normally*/
    //DBG("-------upgrade successful-----\n");

    //do_gettimeofday(&end_tv);
    //DBG("cost time=%lu.%lu\n", end_tv.tv_sec-begin_tv.tv_sec,
    //      end_tv.tv_usec-begin_tv.tv_usec);
    return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    fts_register_write(0, 0x40);
    //delay_ms(100);                       //make sure already enter factory mode
    msleep(100);
    fts_register_write(2, 0x4);               //write command to start calibration
    //delay_ms(300);
    msleep(300);
    for(i=0;i<100;i++){
            fts_register_read(0,&uc_temp,1);
            if (((uc_temp&0x70)>>4) == 0x0){    //return to normal mode, calibration finish
                    break;
            }
            //delay_ms(200);
    msleep(200);
            printk("[FTS] waiting calibration %d\n",i);
    }

    printk("[FTS] calibration OK.\n");

    msleep(300);
    fts_register_write(0, 0x40);          //goto factory mode
    delay_ms(100);                       //make sure already enter factory mode
    fts_register_write(2, 0x5);          //store CLB result
    delay_ms(300);
    fts_register_write(0, 0x0);          //return to normal mode
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

void getVerNo(u8* buf, int len)
{
    u8 start_reg = FT5x0x_REG_FW_VER;
    int ret = -1;
    int i = 0;

    ret = fts_register_read(start_reg, buf, len);
    //et = ft5406_read_regs(ft5x0x_ts_data_test->client,start_reg, buf, 2);
    if (ret < 0) {
        printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return;
    }
    for (i = 0; i < 2; i++) {
        printk("=========buf[%d] = 0x%x \n", i, buf[i]);
    }
    return;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
    FTS_BYTE*     pbt_buf = FTS_NULL;
    int i_ret = 0;
    unsigned char a;
    unsigned char b;
#define BUFFER_LEN (2)            //len == 2
    unsigned char buf[BUFFER_LEN] = {0};

    //=========FW upgrade========================*/
    printk("%s. \n", __func__);

    pbt_buf = CTPM_FW;
    msleep(100);
    getVerNo(buf, BUFFER_LEN);
    a = buf[0];
    b = fts_ctpm_get_i_file_ver();
    printk("a == %hu,  b== %hu \n",a, b);
    /*
     * when the firmware in touch panel maybe corrupted,
     * or the firmware in host flash is new, need upgrade
     */
    if ( 0xa6 == a || a != b ){
        /*call the upgrade function*/
        if(chip_id == 0x55 || chip_id == 0x08 || chip_id == 0x00 || chip_id == 0x0a){
            i_ret =  ft5x06_ctpm_fw_upgrade(&pbt_buf[0],sizeof(CTPM_FW));
            if (i_ret != 0){
                printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
            }
            else {
                printk("[FTS] upgrade successfully.\n");
#ifdef AUTO_CLB
                fts_ctpm_auto_clb();  //start auto CLB
#endif
            }
        }
    }
    return i_ret;

}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2){
        return CTPM_FW[0];
    }
    else{
        return 0xff; //default value
    }
}

static int ft5x_i2c_rxdata(char *rxdata, int length)
{
    int ret;

    struct i2c_msg msgs[] = {
        {
            .addr   = this_client->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = rxdata,
        },
        {
            .addr   = this_client->addr,
            .flags  = I2C_M_RD,
            .len    = length,
            .buf    = rxdata,
        },
    };
    ret = i2c_transfer(this_client->adapter, msgs, 2);
    if (ret < 0)
        printk("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static int ft5x_i2c_txdata(char *txdata, int length)
{
    int ret;

    struct i2c_msg msg[] = {
        {
            .addr   = this_client->addr,
            .flags  = 0,
            .len    = length,
            .buf    = txdata,
        },
    };

    //msleep(1);
    ret = i2c_transfer(this_client->adapter, msg, 1);
    if (ret < 0)
        pr_err("%s i2c write error: %d\n", __func__, ret);

    return ret;
}

static int ft5x_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }

    return 0;
}

/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(struct ft5x_ts_data *data)
{
    struct ts_event *event = &data->event;
    u8 buf[POINT_READ_BUF] = { 0 };
    int ret = -1;
    int i = 0;
    u8 pointid = FT_MAX_ID;

    ret = ft5x_i2c_rxdata(buf, POINT_READ_BUF);
   if (ret < 0) {
        dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
        return ret;
    }

	// 20200527: 这个 memset 会把 touchs 数据清除，导致 修正的 release 操作无法实现。
    //memset(event, 0, sizeof(struct ts_event));

    event->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;

    dprintk(DEBUG_X_Y_INFO, "event->touch_point_num = %d,touchs=0x%x\n", event->touch_point_num, event->touchs);

    // 20200527: 在亮屏休眠的情况下，快速点击TP唤醒系统，但是获取不到 TP信息，原因是系统 resume之后再去读取TP
    // 信息，此时获取到的 touch_point_num 是 0.
    #if 0
	if(!data->is_fboff) {
		// ft5x:point num=0,buf=0x00 0x00 0x00 0x40 0x75 0x00   -- 无消息
		// ft5x:point num=1,buf=0x00 0x00 0x01 0x80 0x72 0x00	-- 有消息
		// ft5x: 00 00 01 01 4b 00 23 3f 30 ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff
		//printk("ft5x:point num=%d,buf=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", event->touch_point_num,
		//	buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] );
		printk("ft5x: %*ph\n", (int)sizeof(buf), buf);
	}
	#endif

	// 20200526: 如果 TP 被 disable 了，我们也需要先把数据读回来，否则可能 TP 的 irq 会一直保持
    // 高电平。我们此处 模拟所有的触摸点弹起状态上报。否则 inputFlinger 会发生异常。
    event->touch_point = 0;
    if( !data->is_enable ) {
    	dev_err(&data->client->dev, "%s: fix data for release,touch=0x%x,cur points=%d\n", __func__,
    		event->touchs, event->touch_point_num);
    	if(event->touchs == 0) { // 如果所有的点都已经 release,则不需要处理。
    		return -2;
    	}
    } else {
	    for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {

	        pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	        if (pointid >= FT_MAX_ID) {
	            break;
	        }

	        event->touch_point++;
	        event->au16_x[i] =
	            (((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
	            8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
	        event->au16_y[i] =
	            (((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
	            8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
	        event->au8_touch_event[i] =
	            buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
	        event->au8_finger_id[i] =
	            (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	        event->au8_xy[i] = (unsigned char)buf[FT_TOUCH_XY_POS + FT_TOUCH_STEP * i];

	        dprintk(DEBUG_X_Y_INFO, "id=%d event=%d x=%d y=%d,points=%d\n", event->au8_finger_id[i],
	            event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->touch_point);

	    }

	    event->pressure = FT_PRESS;
    }

    return 0;
}

/*
*report the point information
*/
static void ft5x_report_value(struct ft5x_ts_data *data)
{
    struct ts_event *event = &data->event;
    int i;
    int uppoint = 0;
    int touchs = 0;
    int x,y;
    /*protocol B*/

    dprintk(DEBUG_X_Y_INFO, "event->touch_point = %d\n", event->touch_point);

    for (i = 0; i < event->touch_point; i++)
    {
        input_mt_slot(data->input_dev, event->au8_finger_id[i]);

        if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);

            x = event->au16_x[i];
            y = event->au16_y[i];
            if( data->exchange_x_y_flag ) {
                int t = x;
                x = y;
                y = t;
            }
            if( data->revert_x_flag) {
                x = data->screen_max_x - x;
            }
            if( data->revert_y_flag) {
                y = data->screen_max_y - y;
            }


            input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);

            touchs |= BIT(event->au8_finger_id[i]);
            event->touchs |= BIT(event->au8_finger_id[i]);

            dprintk(DEBUG_X_Y_INFO, "au8_finger_id[%d] = %d:x = %d,y=%d,max_x=%d,may_y=%d,touchs=0x%x/0x%x\n", i ,
                event->au8_finger_id[i], x, y, data->screen_max_x, data->screen_max_y, event->touchs, touchs);

        } else {

            uppoint++;
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            event->touchs &= ~BIT(event->au8_finger_id[i]);
            dprintk(DEBUG_X_Y_INFO, "here release finger_id0(%d).\n", event->au8_finger_id[i]);
        }
    }

    if(unlikely(event->touchs ^ touchs)){
        for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++){
            // here 'i' is equal finger_id
            if(BIT(i) & (event->touchs ^ touchs)){
                dprintk(DEBUG_X_Y_INFO, "release finger_id1(%d).\n", i);
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    event->touchs = touchs;

    if(event->touch_point == uppoint) {
        input_report_key(data->input_dev, BTN_TOUCH, 0);
    } else {
        input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
    }

    input_sync(data->input_dev);

}

static void ft5x_ts_pen_irq_work(struct work_struct *work)
{
    struct ft5x_ts_data * ts = container_of(work, struct ft5x_ts_data, pen_event_work);

    dprintk(DEBUG_INT_INFO,"%s:is_enable=%d,fb_off=%d\n", __func__, ts->is_enable, ts->is_fboff);
    if (ft5x0x_read_Touchdata(ts) == 0) {
        ft5x_report_value(ts);
    }
    //if(ts->is_enable ) ft5x_set_irq_enabled(ts, true);
}


irqreturn_t ft5x_ts_interrupt(int irq, void *dev_id)
{
    struct ft5x_ts_data *ts = (struct ft5x_ts_data *)dev_id;

    dprintk(DEBUG_INT_INFO,"==========Interrupt,irq=%d======\n", ts->irq);

	// 20200526: 我们 设置 irq 的时候，如果采用 LEVEL 模式，此处需要 disable irq.如果
	// 采用 RISING/FALLING 模式，则此处不需要 disable irq.
    //disable_irq_nosync(ts->irq);
    //ts->irq_enabled = false;

#if 1
    //if (!work_pending(&ts->pen_event_work)) {
    	// 20200526: must make sure work is run after irq,or the ts->irq will be disable.and
    	// queue_work function will check pending flag.
	if(!ts->after_resume){ //201013 tanlq ,when resume, cpu reg make a fake interrupt
    	queue_work(ts->ts_workqueue, &ts->pen_event_work);
	}
	ts->after_resume = false;
    //}
#endif
    return IRQ_HANDLED;
}

static void ft5x_resume_events(struct work_struct *work)
{
    int i = 0;
    struct ft5x_ts_data * data = container_of(work, struct ft5x_ts_data, resume_events_work);
	dprintk(DEBUG_INIT,"ft5x_resume_events \n");
    ft5x_set_power_enabled(data, true);
    delay_ms(10);
    ft5x_reset(data);

    if(chip_id == 0x02 ){
#ifdef FT5X02_CONFIG_INI
        if (ft5x02_Get_Param_From_Ini(FT5X02_CONFIG_NAME) >= 0)
            ft5x02_Init_IC_Param(this_client);
        else
            printk("Get ft5x02 param from INI file failed\n");
#else
        msleep(200);    /*wait...*/
        while(i<5){
            dprintk(DEBUG_INIT,"-----------------------------------------Init ic param\r\n");
            if (ft5x02_Init_IC_Param(this_client) >=0 ){
                dprintk(DEBUG_INIT,"---------------------------------------get ic param\r\n");
                if(ft5x02_get_ic_param(this_client) >=0)
                    break;
            }
            i++;
        }
#endif
        }

    ft5x_set_irq_enabled(data, true);

}

static void ft5x_ts_power_ctrl(struct ft5x_ts_data *data, bool fb_off)
{
	if( fb_off){
		// 20190306：进入真正休眠模式（功耗大概 40ua），只有复位才能唤醒。
		//ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
		//delay_ms(10);
		ft5x_set_irq_enabled(data, false);
		ft5x_set_power_enabled(data, false);
	} else {
		ft5x_set_power_enabled(data, true);
		data->after_resume = true;
		//		printk("%s:irq before gpio value=%d!\n", __func__ ,
		//	gpio_get_value(desc_to_gpio(data->irq_gpio)));
		//ft5x_reset(data);//tanlq add 201012 for int_irq up too late
		//delay_ms(400);//tanlq add 201012 for int_irq up too late

		//printk("%s:irq after gpio value=%d!\n", __func__ ,
		//	gpio_get_value(desc_to_gpio(data->irq_gpio)));
		ft5x_set_irq_enabled(data, true);
	}
}

static int ft5x_ts_suspend(struct ft5x_ts_data *data)
{

    dprintk(DEBUG_SUSPEND,"==ft5x_ts_suspend= fb_power_off=%d,is_fboff=%d\n",fb_power_off(), data->is_fboff);
    //dprintk(DEBUG_SUSPEND,"CONFIG_PM: write FT5X0X_REG_PMODE .\n");
#if 1  // 20191207: we do this at fb-notify.
		if( !data->is_fboff /*fb_power_off()*/) {
			enable_irq_wake(data->irq);
		}
#else

    cancel_work_sync(&data->pen_event_work);
    flush_workqueue(data->ts_workqueue);

    // 20190306：进入真正休眠模式（功耗大概 40ua），只有复位才能唤醒。
    //ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
    //delay_ms(10);

    ft5x_set_irq_enabled(data, false);
    ft5x_set_power_enabled(data, false);
#endif
    return 0;
}

static int ft5x_ts_resume(struct ft5x_ts_data *data)
{
    dprintk(DEBUG_SUSPEND,"=ft5x_ts_resume== fb_power_off=%d,is_suspend=%d\n",fb_power_off(), data->is_fboff);
#if 1
		if(!data->is_fboff /*fb_power_off()*/) {
			disable_irq_wake(data->irq);
		}
#else
    queue_work(data->ts_workqueue, &data->resume_events_work);
#endif
    return 0;
}

static int ft5x_init_irq(struct ft5x_ts_data * data)
{
    int gpio;
    int ret = 0;
    if (!data->irq_gpio) {
        dev_err(&data->client->dev, "%s[%d], irq gpio is null.", __func__, __LINE__);
        return -ENOMEM;
    }
    gpio = desc_to_gpio(data->irq_gpio);
    if (gpio_is_valid(gpio)) {
        data->irq = gpio_to_irq(gpio);
    }

    // 20190301:出现一种情况：刚刚 request irq的时候就马上进入中断，LOG 如下：
    // 所以在 注册之前先设置 data->irq_enabled = true。否则会导致 irq_disable.但是TP 失灵的问题
    // 似乎不是这个原因，根本原因是 GPIO7 控制寄存器里面的数据被 显示数据覆盖了才导致终端很早发生。
    /*
Line 646: [    0.931993] ***ft5x_ts_sunty***==========ft5x_ts TS Interrupt,irq=301======
Line 647: [    0.932012] ft5x_ts_sunty 4-0011: ft5x_init_irq[301], ok to request irq, trigger is [falling].
    */
    data->irq_enabled = true;
    ret = devm_request_irq(&data->client->dev, data->irq,
                ft5x_ts_interrupt,
                IRQ_GPIO_ACTIVE_VALUE ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING,
                //IRQ_GPIO_ACTIVE_VALUE ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW,
                dev_name(&data->client->dev),
                data);
    if (ret) {
        dev_err(&data->client->dev, "%s[%d], Failed to request irq.\n", __func__, __LINE__);
    } else {
        //dev_err(&data->client->dev, "%s[%d], ok to request irq, trigger is [%s].\n", __func__, data->irq, IRQ_GPIO_ACTIVE_VALUE ? "rising" : "falling");
        dev_err(&data->client->dev, "%s[%d], ok to request irq, trigger is [%s].\n", __func__, data->irq, IRQ_GPIO_ACTIVE_VALUE ? "High" : "Low");
        //data->irq_enabled = true;
    }

    return ret;
}

static int ft5x_get_chip_id(struct i2c_client *client)
{
    int i = 0;
    int ret = 0;
    if (!client) {
        return -ENOMEM;
    }

    while((ret == 0x00) || (ret == 0xa3)) {
        ret = i2c_smbus_read_byte_data(client, 0xA3);
        dev_warn(&client->dev, "%s[%d]: addr is 0x%x, chip_id value:0x%x (%d)\n", __func__, __LINE__, client->addr, ret, ret);
        if((i++) > 10) {
            break;
        }
        delay_ms(5);
    }
    dprintk(DEBUG_INIT, "read chip_id timers,timers=%d,ret=%d\n", i, ret);

    return ret;

}

// 20180213,hsl add fb_notifier to shut down hid-i2c.
static int ft5x_fb_notifier_callback(struct notifier_block *self,
				     unsigned long action, void *data)
{
	struct ft5x_ts_data *ts;
	struct fb_event *event = data;
    int     cmd;
	ts = container_of(self, struct ft5x_ts_data, fb_notif);

//tanlq add 190529 for system goon
    //return NOTIFY_DONE;

	if (!event)
		return NOTIFY_DONE;

#ifndef NO_ELINK
		if( !fb_eink(event->info) ){
			return NOTIFY_DONE;
		}
#endif

    cmd = *((int *)event->data);
    //dprintk(DEBUG_INIT,"%s:cmd=%d,action=%ld\n", __func__, cmd, action);
    if(action == FB_EARLY_EVENT_BLANK) {
		switch (cmd) {
		case FB_BLANK_VSYNC_SUSPEND:
			if(ts->is_enable){
				ts->is_enable = false;
				ft5x_set_irq_enabled(ts, ts->is_enable);
				dprintk(DEBUG_SUSPEND,"%s:disable touch\n", __func__ );

				// 20200527: 我们通过一个 work 来上报已经按下的案件。
				// 20200615: 我们在 inputDispatch 上面做了处理，此处不能模式 TP 触摸已经弹起的事件。
				//queue_work(ts->ts_workqueue, &ts->pen_event_work);
			}
			break;
		case FB_BLANK_NORMAL:
			if(!ts->is_enable){
				ts->is_enable = true;
				dprintk(DEBUG_SUSPEND,"%s:enable touch\n", __func__ );
				ft5x_set_irq_enabled(ts, ts->is_enable);
			}
			break;
		default:
			break;
		}
	} else if( action == FB_EVENT_BLANK ) {
		switch (cmd) {
		case FB_BLANK_POWERDOWN:
			if(!ts->is_fboff){
				ts->is_fboff = true;
				ft5x_ts_power_ctrl(ts, true);
			}
			break;
		case FB_BLANK_UNBLANK:
			if(ts->is_fboff) {
				ft5x_ts_power_ctrl(ts, false);
				ts->is_fboff = false;
			}
			break;

		default:
			break;
		}
	}
	return NOTIFY_OK;
}



static ssize_t ft5x_dbg_mode_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
    struct ft5x_ts_data *data = i2c_get_clientdata(this_client);

    int gpio = desc_to_gpio(data->irq_gpio);
    int len = sprintf(_buf,"ft5x:irq=%d,enabled=%d,irq gpio=%d,level=%d\n",
        data->irq, data->irq_enabled, gpio, gpio_get_value(gpio));
    printk("%s\n", _buf);
    return len;
}

static ssize_t ft5x_dbg_mode_store(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
{
    struct ft5x_ts_data *ts = i2c_get_clientdata(this_client);
	int cmd = simple_strtol(buf,NULL,10);
	printk(KERN_INFO "ft5x:cmd=%d\n", cmd);
	switch( cmd ) {
	    case 0:
	        // disable the irq.
	        ft5x_set_irq_enabled(ts, false);
	    break;
	    case 1:
	        // enable the irq.
	        ft5x_set_irq_enabled(ts, true);
	    break;
	    case 2:
            ft5x_reset(ts); // just reset by GPIO.
	    break;
	    case 3:
	        // power off the chip.
	        ft5x_set_power_enabled(ts, false);
	        delay_ms(200);
	        // reset the chip.
	        ft5x_set_power_enabled(ts, true);
	        ft5x_reset(ts);
	    break;
	    case 4:
	        // read the chip to check I2C.
	        ft5x_get_chip_id(this_client);
	    break;
	    case 5:
	        ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	    break;
	    case 6:
	        queue_work(ts->ts_workqueue, &ts->pen_event_work);
	    break;
	    default: break;
	}

    return _count;
}
static CLASS_ATTR(debug, 0664, ft5x_dbg_mode_show, ft5x_dbg_mode_store);

static int ft5x_dbg_sys_init(struct ft5x_ts_data * data)
{
	int ret ;
	data->debug_class = class_create(THIS_MODULE, "ft_dbg");
	if( IS_ERR_OR_NULL(data->debug_class) ) {
	    printk("Fail to creat class ft_dbg.\n");
	    return PTR_ERR(data->debug_class);
	}
   	ret =  class_create_file(data->debug_class, &class_attr_debug);
    if (ret)
    {
       printk("Fail to creat class ft_dbg file!!\n");
    }
   return ret;
}

static void ft5x_init_events (struct work_struct *work)
{
    int i = 0;
    struct ft5x_ts_data * data = container_of(work, struct ft5x_ts_data, init_events_work);

    dprintk(DEBUG_INIT, "====%s begin=====.  \n", __func__);

    ft5x_set_power_enabled(data, true);
    ft5x_reset(data);

    chip_id = ft5x_get_chip_id(data->client);

	// 20190531,hsl add.
	if( chip_id < 0 ) {
		// 20190531-LOG: ft5x_get_chip_id=-6--No-Tp?
		printk("ft5x_get_chip_id=%d--No-Tp?\n", chip_id);
		return ;
	}

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
  fts_ctpm_fw_upgrade_with_i_file();
#endif

    if(chip_id == 0x02 ) {
#ifdef FT5X02_CONFIG_INI
        if (ft5x02_Get_Param_From_Ini(FT5X02_CONFIG_NAME) >= 0)
            ft5x02_Init_IC_Param(this_client);
        else
            printk("Get ft5x02 param from INI file failed\n");
#else
        msleep(1000);   /*wait...*/
        while(i<5){
            dprintk(DEBUG_INIT,"-----------------------------------------Init ic param\r\n");
            if (ft5x02_Init_IC_Param(this_client) >=0 ) {
                dprintk(DEBUG_INIT,"---------------------------------------get ic param\r\n");
                if(ft5x02_get_ic_param(this_client) >=0)
                    break;
            }
            i++;
        }
#endif
    }

    //songsayit, here to init irq(irq enabled defautly)
    ft5x_init_irq(data);
	data->is_enable = true;
	data->is_fboff = false;
	data->after_resume = false;

    ft5x_dbg_sys_init(data);

    data->fb_notif.notifier_call = ft5x_fb_notifier_callback;
	fb_register_client(&data->fb_notif);
}

#if defined(CONFIG_OF)
static int ft5x_parse_dt(struct ft5x_ts_data *ft5x_ts)
{
    struct device *dev = NULL;

    u32 val;
    int err = 0;

    if (!ft5x_ts) {
        printk("%s[%d]: ft5x_ts is null.\n", __func__, __LINE__);
        return -ENOMEM;
    }
    if (!ft5x_ts->client) {
        printk("%s[%d]: ft5x_ts->client is null.\n", __func__, __LINE__);
        return -ENOMEM;
    }
    dev = &ft5x_ts->client->dev;

    if (!dev->of_node) {
        dev_err(dev, "%s[%d]: no device node found.\n", __func__, __LINE__);
        return -ENOMEM;
    }

    ft5x_ts->supply = devm_regulator_get(dev, "power");
    if (IS_ERR(ft5x_ts->supply)) {
        dev_err(dev, "%s[%d]: failed to get supply of power.\n", __func__, __LINE__);
        ft5x_ts->supply = NULL;
    }
    ft5x_ts->enable_gpio = devm_gpiod_get_optional(dev, "enable", ENABLE_GPIO_ACTIVE_VALUE ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
    if (IS_ERR(ft5x_ts->enable_gpio)) {
        err = PTR_ERR(ft5x_ts->enable_gpio);
        dev_err(dev, "%s[%d]:failed to request enable GPIO: %d.\n", __func__, __LINE__, err);
    }
    ft5x_ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", RESET_GPIO_ACTIVE_VALUE ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
    if (IS_ERR(ft5x_ts->reset_gpio)) {
        err = PTR_ERR(ft5x_ts->reset_gpio);
        dev_err(dev, "%s[%d]:failed to request reset GPIO: %d.n", __func__, __LINE__, err);
    }
    ft5x_ts->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
    if (IS_ERR(ft5x_ts->irq_gpio)) {
        err = PTR_ERR(ft5x_ts->irq_gpio);
        dev_err(dev, "%s[%d]:failed to request irq GPIO: %d.\n", __func__, __LINE__, err);
    }

    if (!of_property_read_u32(dev->of_node, "screen_max_x", &val)) {
        ft5x_ts->screen_max_x = val;
    } else {
        ft5x_ts->screen_max_x = DEFAULT_SCREEN_MAX_X;
    }

    if (!of_property_read_u32(dev->of_node, "screen_max_y", &val)) {
        ft5x_ts->screen_max_y = val;
    } else {
        ft5x_ts->screen_max_y = DEFAULT_SCREEN_MAX_Y;
    }

    ft5x_ts->revert_x_flag = 0;
    if (!of_property_read_u32(dev->of_node, "revert_x_flag", &val))
        ft5x_ts->revert_x_flag = !!val;
    ft5x_ts->revert_y_flag = 0;
    if (!of_property_read_u32(dev->of_node, "revert_y_flag", &val))
        ft5x_ts->revert_y_flag = !!val;
    ft5x_ts->exchange_x_y_flag = 0;
    if (!of_property_read_u32(dev->of_node, "exchange_x_y_flag", &val))
        ft5x_ts->exchange_x_y_flag = !!val;


    dprintk(DEBUG_INIT, "screen_max_x = %d, screen_max_y = %d.\n", ft5x_ts->screen_max_x, ft5x_ts->screen_max_y);

    return 0;
}
#else
static int ft5x_parse_dt(struct ft5x_ts_data *ft5x_ts)
{
    return 0;
}

#endif

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ft5x_ts_data *ft5x_ts;
    struct input_dev *input_dev;
    int err = 0;

#ifdef TOUCH_KEY_SUPPORT
    int i = 0;
#endif

    dprintk(DEBUG_INIT, "====%s songshtian begin=====.  \n", __func__);

    //printk( "====%s begin===songshtian==.  \n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        dev_err(&client->dev, "i2c_check_functionality failed\n");
        return err;
    }

    ft5x_ts = devm_kzalloc(&client->dev, sizeof(*ft5x_ts), GFP_KERNEL);
    if (!ft5x_ts)   {
        err = -ENOMEM;
        dev_err(&client->dev, "alloc data failed\n");
        return err;
    }

    this_client = client;
    ft5x_ts->client = client;
    i2c_set_clientdata(client, ft5x_ts);

    ft5x_parse_dt(ft5x_ts);

    ft5x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!ft5x_ts->ts_workqueue) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to create ts_workqueue.\n");
        goto failed_create_singlethread_workqueue;
    }

    mutex_init(&ft5x_ts->mutex_lock);

    INIT_WORK(&ft5x_ts->pen_event_work, ft5x_ts_pen_irq_work);
    INIT_WORK(&ft5x_ts->init_events_work, ft5x_init_events);
    INIT_WORK(&ft5x_ts->resume_events_work, ft5x_resume_events);


    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }

    ft5x_ts->input_dev = input_dev;
    input_dev->name = dev_name(&client->dev);
    input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_set_drvdata(input_dev, ft5x_ts);


    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(EV_REP, input_dev->evbit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    input_mt_init_slots(input_dev, TOUCH_POINT_NUM, INPUT_MT_DIRECT);

#ifdef CONFIG_FT5X0X_MULTITOUCH
    set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft5x_ts->screen_max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft5x_ts->screen_max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

    #ifdef FOR_TSLIB_TEST
    set_bit(BTN_TOUCH, input_dev->keybit);
    #endif

    #ifdef TOUCH_KEY_SUPPORT
    key_tp = 0;
    input_dev->evbit[0] = BIT_MASK(EV_KEY);
    for (i = 1; i < TOUCH_KEY_NUMBER; i++)
        set_bit(i, input_dev->keybit);
    #endif
#else
    set_bit(ABS_X, input_dev->absbit);
    set_bit(ABS_Y, input_dev->absbit);
    set_bit(ABS_PRESSURE, input_dev->absbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    input_set_abs_params(input_dev, ABS_X, 0, ft5x_ts->screen_max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, ft5x_ts->screen_max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

    err = input_register_device(input_dev);
    if (err) {
        dev_err(&client->dev,"ft5x_ts_probe: failed to register input device: %s\n",
                dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    queue_work(ft5x_ts->ts_workqueue, &ft5x_ts->init_events_work);

#ifdef CONFIG_FT5X0X_MULTITOUCH
    dprintk(DEBUG_INIT,"CONFIG_FT5X0X_MULTITOUCH is defined. \n");
#endif
    dprintk(DEBUG_INIT, "==%s over =\n", __func__);

    return 0;

exit_input_register_device_failed:
    input_free_device(input_dev);
exit_input_dev_alloc_failed:
    i2c_set_clientdata(client, NULL);
    if (ft5x_ts) {
        destroy_workqueue(ft5x_ts->ts_workqueue);
    }
failed_create_singlethread_workqueue:
    if (ft5x_ts) {
        kfree(ft5x_ts);
    }
    return err;
}

static int ft5x_ts_remove(struct i2c_client *client)
{

    struct ft5x_ts_data *data = i2c_get_clientdata(client);
    //ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);

    printk("==ft5x_ts_remove=\n");

    cancel_work_sync(&data->init_events_work);
    cancel_work_sync(&data->resume_events_work);
    cancel_work_sync(&data->pen_event_work);
    destroy_workqueue(data->ts_workqueue);

    mutex_destroy(&data->mutex_lock);

    input_unregister_device(data->input_dev);
    input_free_device(data->input_dev);

    ft5x_set_irq_enabled(data, false);
    ft5x_set_power_enabled(data, false);

    if (data->supply) {
        devm_regulator_put(data->supply);
    }
    if (data->enable_gpio) {
        devm_gpiod_put(&client->dev, data->enable_gpio);
    }
    if (data->reset_gpio) {
        devm_gpiod_put(&client->dev, data->reset_gpio);
    }

    if (data->irq_gpio) {
        if (data->irq > 0) {
            free_irq(data->irq, data);
        }
        devm_gpiod_put(&client->dev, data->irq_gpio);
    }

    kfree(data);

    i2c_set_clientdata(client, NULL);

    return 0;

}


#ifdef CONFIG_PM
static int ft5x_suspend(struct device *dev)
{
    struct ft5x_ts_data *data = dev_get_drvdata(dev);

    ft5x_ts_suspend(data);

    return 0;
}

static int ft5x_resume(struct device *dev)
{
    struct ft5x_ts_data *data = dev_get_drvdata(dev);

    ft5x_ts_resume(data);

    return 0;
}

static const struct dev_pm_ops ft5x_pm_ops = {
    .suspend	= ft5x_suspend,
    .resume		= ft5x_resume,
};
#endif


static const struct i2c_device_id ft5x_ts_id[] = {
    { CTP_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);



#if defined(CONFIG_OF)
static struct of_device_id ft5x_dt_ids[] = {
	{ .compatible = CTP_NAME },
};

MODULE_DEVICE_TABLE(of, ft5x_dt_ids);
#endif


static struct i2c_driver ft5x_ts_driver = {
    .driver = {
        .name = CTP_NAME,
        .owner = THIS_MODULE,
#if defined(CONFIG_OF)
        .of_match_table = of_match_ptr(ft5x_dt_ids),
#endif

#ifdef CONFIG_PM
        .pm = &ft5x_pm_ops,
#endif
    },
    .probe      = ft5x_ts_probe,
    .remove     = ft5x_ts_remove,
    .id_table   = ft5x_ts_id,

};
//module_i2c_driver(ft5x_ts_driver);
static int __init ft5x_ts_init(void)
{
	return i2c_add_driver(&ft5x_ts_driver);
}

static void __exit ft5x_ts_exit(void)
{
	i2c_del_driver(&ft5x_ts_driver);
}

// 20200509: must init after wacom_i2c,cause must power on wacom first.
late_initcall(ft5x_ts_init);
//module_exit(ft5x_ts_init);


MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");

