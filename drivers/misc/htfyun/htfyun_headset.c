/*
 * Copyright (C) 2014 Rockchip Corporation.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/platform_device.h>
#include <linux/input.h>

#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

//#include "../../sound/soc/codecs/rt5640.h"
#ifdef CONFIG_DEBUG_BUILD
static int debug = 1;
#else
static int debug = 0;
#endif
module_param(debug, int, S_IRUGO|S_IWUSR);
#define dprintk(level, fmt, arg...) do {   \
 if (debug >= level)      \
 printk(KERN_WARNING"htfyun_headset: " fmt , ## arg); } while (0)
#define DBG(format, ...) dprintk(1, format, ## __VA_ARGS__)


// 20160331,we need the hardware to support!!current disable.
// 20160331,we use DTS to ctrl,no need to define here.
//#define SUPPORT_IPHONE_IC_POWER_CTRL        1

// if we want to report the media KEYCODE_HEADSETHOOK,
// define the follow value.
//#define HSDET_REPORT_KEY_MEDIA

// 20150908,HSL,add a switch for HEADSET gpio detect.
// 20150908: the code need to support both,we remove
// gpio irq from DTS.
//#define HSDET_GPIO_INT_ENABLE                   0   // 1 :ENABLE, 0: disable.

// 20170608,200ms is too low,we may miss some press,so set to 100ms.
#define HSDET_ADC_SAMPLE_TIME_MS        100

// 定义ADC的值允许的最大误差.
#define HSDET_DRIFT_ADVALUE				32 // 100

#define HSDET_BOUNCE_CNT                2

#define BIT_HEADSET             (1 << 0)
#define BIT_HEADSET_NO_MIC      (1 << 1)

#define SUPPORT_FIX_HEADSET     0

struct rk_headset_adc_det{
    struct input_dev *input_dev;
	struct iio_channel *chan;
	struct delayed_work adc_check;
	struct delayed_work gpio_check;
	struct switch_dev sdev;

	struct wake_lock	headset_wl;

	// we can use just a adc to detect the headset and mic.
	// see rk3288-cs1.dts. diff headset make diff adc value.
	int         adc_headset;
	int         plugin_debounce; // the adc check count when detect headset is plug.

	int         adc_mic_min;

	int         adc_vol_up; // adc value when press vol up key.
	int         adc_vol_down; //
	int         adc_media;

    // 20181206: add for fix headset report for hardware-bug.
    int         fix_headset;

    // 20170817,add debug key for FT test.
	int         adc_dbg_key;

	int         micdet_gpio;    // the gpio that if it is a mic.
	int         micdet_irq;
	int         mic_level_micdet;

	int         headset_gpio;   // if headset insert or not.
	int         headset_irq;
	int         headset_in_level;

	int         gpio_irq_active_flag; //BIT 0: for micdet gpio,1:bit headset gpio.

	int         ihone_ic_pwr_gpio;
	int         ihone_ic_pwr_active_flag; // when low,the IC is pwr on.

    int         adc_sample_ms;
    bool        report_media_unplug; // weather report media key when headset unplug.
    //bool        dbg_adc;    // weather we need to print the adc value( only for debug).


	// 20191227,hsl add.
	bool		suspended;

    // status variants.
    bool        key_media_reported;
    bool        key_volup_reported;
    bool        key_voldown_reported;
    bool        key_debug_reported;

    bool         need_to_check_Mic ; // only headset plug insert and don't nkow the Mic ,we need to check it.
	bool         isMic ;
	bool         headset_plugin;

	int         headset_status;
};

#define IRQ_FLAG_MICDET      (1<<0)
#define IRQ_FLAG_HEADSET    (1<<1)
static  struct rk_headset_adc_det       *p_hsdet_data;

#define HOOK_KEY_CODE KEY_MEDIA

#define HP_DEBUG_CODE KEY_RO

/*-------------------------------------------------------*/
//extern int fm1188_set_mic_volume( int mic , int vol );
extern bool fb_power_off( void );
/*-------------------------------------------------------*/

static void set_iphone_IC_power(struct rk_headset_adc_det* data, int en )
{
    //printk("IPHONE-IC:gpio=%d,en=%d/%d\n" ,data->ihone_ic_pwr_gpio, en ,
    //    data->ihone_ic_pwr_active_flag );

    if( data->ihone_ic_pwr_gpio >= 0 ) {
        if( en ) {
            gpio_direction_output( data->ihone_ic_pwr_gpio , data->ihone_ic_pwr_active_flag );
        } else {
            gpio_direction_output( data->ihone_ic_pwr_gpio , !data->ihone_ic_pwr_active_flag );
        }
    }
}
static inline bool hsdet_adc_at( int adc_val , int use_val )
{
    if(  adc_val <= use_val+HSDET_DRIFT_ADVALUE &&
            adc_val >= use_val-HSDET_DRIFT_ADVALUE){

        return true;
    }
    return false;
}


static inline bool hsdet_adc_big_then( int adc_val , int use_val )
{
    if(  adc_val >= use_val &&
            adc_val < 900 ){
        return true;
    }
    return false;
}

static ssize_t hsdet_h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

static int hsdet_key_open(struct input_dev *dev)
{
	return 0;
}


static void hsdet_key_close(struct input_dev *dev)
{

}


static void hsedt_report_status( struct rk_headset_adc_det *pdata )
{
    int headset_status = 0;
    if( pdata->headset_plugin || pdata->isMic ){
       if( pdata->isMic ){
            headset_status = BIT_HEADSET;   // with mic.
       } else {
            headset_status = BIT_HEADSET_NO_MIC;
       }
    }
    if( headset_status != pdata->headset_status ) {
        DBG("hs_det:headset status changed,old=%d,new=%d(%s)\n", pdata->headset_status ,
        headset_status,pdata->isMic?"with MIC":"no MIC" );

        // if we change from 2 to 1: BIT_HEADSET_NO_MIC --> BIT_HEADSET. the android show
        // failed. so we need to change a state 0 here.
        if( headset_status >0 && pdata->headset_status > 0 ) {
            #if 0
            if( pdata->report_media_unplug ) {
                input_report_key(pdata->input_dev,HOOK_KEY_CODE, 0);
                input_sync(pdata->input_dev);
            }
            #endif
            switch_set_state(&pdata->sdev, 0);
            DBG("switch_set_state state is 0.\n");
            msleep(50);
        }

        if( pdata->report_media_unplug && headset_status == 0 ) {
            // 20170608,we need key down and up.and only report when
            // unplug.
            input_report_key(pdata->input_dev,HOOK_KEY_CODE,1); // key press.
            input_sync(pdata->input_dev);
            msleep(10);
            input_report_key(pdata->input_dev,HOOK_KEY_CODE,0); // key press.
            input_sync(pdata->input_dev);
        }
        switch_set_state(&pdata->sdev, headset_status);
        DBG("switch_set_state state is headset_status = %d.\n", headset_status);
        pdata->headset_status = headset_status;
    }
}
static void adc_check_work_callback(struct work_struct *work)
{
	int ret,val;
	struct rk_headset_adc_det *pdata = p_hsdet_data;
    bool    adc_consumed = false;

    if( pdata->chan == NULL ){
        return ;
    }

    if( (pdata->headset_irq > 0 || pdata->micdet_irq > 0) && !pdata->headset_plugin ){
        return ; // we use the GPIO irq to trigger
    }

    //rt5640_set_micbias_enabled(true); // 20191226,don't need at px30.

    ret = iio_read_channel_raw(pdata->chan, &val);
    if (ret < 0) {
        pr_err("hs_det:read adc channel() error: %d\n", ret);
	    return ;
    }
    else {
        /* 
       打开调试信息：echo 3 >   /sys/module/htfyun_headset/parameters/debug
        */
        if( /*pdata->dbg_adc || */debug > 2) {
	        DBG("hs_det:read adc value=%d\n",val);
    	}
    }

    /* 20170510,LOG:
<4>[  214.046614] hs_det:read adc value=7
<4>[  214.050094] hs_det: detect headset plugoff,adc value=7/576
<4>[  214.055562] IPHONE-IC:gpio=-2,en=0/0
<4>[  214.059449] hs_det:headset status changed,old=1,new=2(no MIC)
<4>[  214.322065] hs_det:read adc value=561
<4>[  214.325787] hs_det:headset status changed,old=2,new=1(with MIC)
    */
    // 20170510,the order is important.
    // if we have GPIO,then disable the adc check!!
    if( pdata->headset_irq < 0 && pdata->adc_headset >= 0  ) {
        if( hsdet_adc_at(val, pdata->adc_headset ) ) {
                pdata->plugin_debounce ++;
                if( pdata->plugin_debounce >= HSDET_BOUNCE_CNT  ) {
                    // headset plugin.
                    // 20150821,we use isMic as for micdet_gpio active.
                    if( !pdata->headset_plugin ) {
                            pdata->headset_plugin = true;
                        DBG("headset_plugin is true. %s:%d.\n", __func__, __LINE__);
                            DBG("hs_det: detect headset plugin,adc value=%d/%d\n", val , pdata->adc_headset );
                            pdata->need_to_check_Mic = true;
                    }
                }
                adc_consumed = true;
        } else {
                pdata->plugin_debounce = 0;
                if( pdata->headset_plugin ) {
                        pdata->headset_plugin = false;
                        pdata->need_to_check_Mic = true;
                        DBG("hs_det: detect headset plugoff,adc value=%d/%d\n", val , pdata->adc_headset );
                }
        }
    }

    // 20170510,handle key.
    if( pdata->adc_media >= 0 && !adc_consumed ){
        // adc_mic
        if( hsdet_adc_at(val, pdata->adc_media ) ) {
            if( !pdata->key_media_reported ){
                pdata->key_media_reported = true;
                input_report_key(pdata->input_dev,KEY_MEDIA,1);
                input_sync(pdata->input_dev);
            }
            adc_consumed = true;
        } else {
            if( pdata->key_media_reported ){
                pdata->key_media_reported = false;
                input_report_key(pdata->input_dev,KEY_MEDIA,0);
                input_sync(pdata->input_dev);
            }
        }
    }

    if( pdata->adc_vol_up >= 0 && !adc_consumed ){
        // adc_mic
        if( hsdet_adc_at(val, pdata->adc_vol_up ) ) {
            if( !pdata->key_volup_reported ){
                pdata->key_volup_reported = true;
                input_report_key(pdata->input_dev,KEY_VOLUMEUP,1);
                input_sync(pdata->input_dev);
            }
            adc_consumed = true;
        } else {
            if( pdata->key_volup_reported ){
                pdata->key_volup_reported = false;
                input_report_key(pdata->input_dev,KEY_VOLUMEUP,0);
                input_sync(pdata->input_dev);
            }
        }
    }

    if( pdata->adc_vol_down >= 0 && !adc_consumed ){
        // adc_mic
        if( hsdet_adc_at(val, pdata->adc_vol_down ) ) {
            if( !pdata->key_voldown_reported ){
                pdata->key_voldown_reported = true;
                input_report_key(pdata->input_dev,KEY_VOLUMEDOWN,1);
                input_sync(pdata->input_dev);
            }
            adc_consumed = true;
        } else {
            if( pdata->key_voldown_reported ){
                pdata->key_voldown_reported = false;
                input_report_key(pdata->input_dev,KEY_VOLUMEDOWN,0);
                input_sync(pdata->input_dev);
            }
        }
    }

    if( pdata->adc_dbg_key >= 0 && !adc_consumed ){
        // adc_mic
        if( hsdet_adc_at(val, pdata->adc_dbg_key ) ) {
            if( !pdata->key_debug_reported ){
                pdata->key_debug_reported = true;
                input_report_key(pdata->input_dev,HP_DEBUG_CODE,1);
                input_sync(pdata->input_dev);
            }
            adc_consumed = true;
        } else {
            if( pdata->key_debug_reported ){
                pdata->key_debug_reported = false;
                input_report_key(pdata->input_dev,HP_DEBUG_CODE,0);
                input_sync(pdata->input_dev);
            }
        }
    }

    /* 20170510,when one key press,may affect the adc_mic.and we only need to check
     * once when headset plug in.
    */
    if( /*pdata->adc_mic_min >= 0 && */ pdata->micdet_irq < 0 && !adc_consumed
        && pdata->need_to_check_Mic)  {

        // we only need to check Mic when headset plug or No headset/micdet irq.
        // (if no headset/micdet irq,we need pure adc weather to define weather
        // is a headset/mic or not ).
        if( pdata->headset_irq > 0 ) {
            // if we have headset gpio,so only need to check mic when head set is plug.
            if( pdata->headset_plugin ) {
                if( pdata->adc_mic_min > 0) {
                    if( /*hsdet_adc_at*/hsdet_adc_big_then(val, pdata->adc_mic_min ) ) {
                        pdata->isMic = true;
                        DBG("isMic is true. %s: %d. val = %d,adc_min=%d\n", __func__, __LINE__, val, pdata->adc_mic_min);
                    } // NO pdata->isMic = false needed!!
                } else if( pdata->adc_mic_min == 0) {
                    // 20180827: 如果 pdata->adc_mic_min == 0,表示只要 > 0 就有MIC。
                    if( val >= HSDET_DRIFT_ADVALUE) {
                        DBG("isMic is true. %s: %d. val = %d(0)\n", __func__, __LINE__, val);
                        pdata->isMic = true;
                    }
                } else {
                    // < 0,表示不支持MIC？？
                }
            }
        } else {
            // we need to define headset/mic all by adc.
            if( hsdet_adc_at(val, pdata->adc_mic_min ) ) {
                // with mic plugin.
                pdata->isMic = true;
                DBG("isMic is true. %s: %d.\n", __func__, __LINE__);

                // we handle at hsedt_report_status.
                // 20170510,the headset_plugin is maintain bt headset adc/gpio.
                //pdata->headset_plugin = 1;  // the headset must plugin!!
            } else {
                pdata->plugin_debounce = 0;
                if( pdata->isMic ) {
                    pdata->isMic = false;
                    DBG("hs_det: detect MIC plugoff,adc value=%d/%d\n", val , pdata->adc_mic_min );
                    // 20160330,NO MIC headset,disable ihoneIC power
                    set_iphone_IC_power(pdata,0);
                }
            }
        }
        pdata->need_to_check_Mic = false;
    }

    hsedt_report_status( pdata );

    // 20160330,if NOT IRQ( USE ADC detect headset insert),we need to
    // continue start adc check work.
    // 20170509,if we have vol_up,vol_down,media(stop/start) key,we also need to
    // continue adc check work.
    if( (pdata->headset_irq < 0 && pdata->micdet_irq < 0) ||
        (pdata->adc_media >= 0 ||
        pdata->adc_vol_up>= 0 ||
        pdata->adc_vol_down>= 0 ||
        pdata->adc_dbg_key >= 0 ) ||
        pdata->need_to_check_Mic ){
        schedule_delayed_work(&pdata->adc_check,
            msecs_to_jiffies(HSDET_ADC_SAMPLE_TIME_MS));
    }
}

static void gpio_check_work_callback(struct work_struct *work)
{
	struct rk_headset_adc_det *pdata = p_hsdet_data;
	DBG("gpio_check_work_callback,headset gpio=%d,mic gpio=%d,suspend=%d\n", pdata->headset_gpio,
		pdata->micdet_gpio, pdata->suspended);
	if( pdata->suspended ) return;

#if SUPPORT_FIX_HEADSET
    if( pdata->fix_headset >= 0 ){
        printk("htfyun-headset: report by fix_headset=%d\n", pdata->fix_headset);
        cancel_delayed_work(&pdata->adc_check);
        //rt5640_set_micbias_enabled(false);
        pdata->isMic = pdata->fix_headset?true:false;
        pdata->headset_plugin = true;
        hsedt_report_status( pdata );
        return ;
    }
#endif
       // NOT SUPPORT!
	if( gpio_is_valid(pdata->micdet_gpio) ){
	    DBG("micdet gpio level=%d,active level=%d\n",gpio_get_value(pdata->micdet_gpio),
	        pdata->mic_level_micdet );
	    if( gpio_get_value(pdata->micdet_gpio) == pdata->mic_level_micdet ) {
                pdata->isMic = true;
                DBG("isMic is true. %s: %d.\n", __func__, __LINE__);

                // we only set the input mic,no change the vol.
                // we let the FM1188 ctrl the volume.
                // 20151113,HSL,we set the mic at APP.not the driver!!
                //fm1188_set_mic_volume( 1 , 0xffff );

                //switch_set_state(&pdata->sdev, headset_status);
       } else {
                pdata->isMic = false;
                //pdata->headset_plugin = 1;  // let this to make triger a adc exchanged.
                //fm1188_set_mic_volume( 0 , 0xffff);

       }

       //pdata->gpio_irq_active_flag &= ~IRQ_FLAG_MICDET;
       enable_irq( pdata->micdet_irq );
	}

	if(gpio_is_valid(pdata->headset_gpio )) {
	    int gpio_value = gpio_get_value(pdata->headset_gpio);
	    if( gpio_value == pdata->headset_in_level ) {
                pdata->headset_plugin = true;
                //switch_set_state(&pdata->sdev, headset_status);
       } else {
                pdata->headset_plugin = false;
                pdata->isMic = false;
                set_iphone_IC_power(pdata,1); // when NO headset,enable the IC power.
       }

       // 20210412: 根据当前的 GPIO-LEVEL 来设置中断触发类型。如果当前GPIO是 高，就设置为 下降沿，
       // 否则就设置为上升沿。
        if(gpio_value)
			irq_set_irq_type(pdata->headset_irq, IRQ_TYPE_EDGE_FALLING);
	    else
			irq_set_irq_type(pdata->headset_irq, IRQ_TYPE_EDGE_RISING);
        //DBG("headset_plugin is true. %s:%d.\n", __func__, __LINE__);
                
       #if 0
       if(gpio_get_value(pdata->headset_gpio)){
            DBG("%s:irq_set_irq_type IRQF_TRIGGER_LOW\n" , __func__ );
            irq_set_irq_type(pdata->headset_irq, IRQ_TYPE_LEVEL_LOW/* IRQF_TRIGGER_LOW*/);
        }else{
            DBG("%s:irq_set_irq_type IRQF_TRIGGER_HIGH\n" , __func__ );
            irq_set_irq_type(pdata->headset_irq, IRQ_TYPE_LEVEL_HIGH /*IRQF_TRIGGER_HIGH*/);
        }
       #endif
       //pdata->gpio_irq_active_flag &= ~IRQ_FLAG_HEADSET;
       enable_irq(pdata->headset_irq);

	   //DBG("gpio_check_work_callback enable_irq %d\n", pdata->headset_irq);
	   DBG("headset-check: gpio value=%d, plugin level=%d, irq=%d, plug-in=%d\n", gpio_value,
	        pdata->headset_in_level, pdata->headset_irq, pdata->headset_plugin);
	}

	if( pdata->chan != NULL ) {
	    if(pdata->headset_plugin ){
	        pdata->need_to_check_Mic = true;
	        //rt5640_set_micbias_enabled(true); // 20180928: ENABLE BIAS here.
		    // check the mic or adc value.
		    schedule_delayed_work(&pdata->adc_check,
	            msecs_to_jiffies(100)); // for fully insert!!
	        //}
	    } else {
	        cancel_delayed_work(&pdata->adc_check);
	        //rt5640_set_micbias_enabled(false);
	    }
    }
    hsedt_report_status( pdata );

}
static irqreturn_t headset_gpio_irq(int irq, void *dev_id)
{
        struct rk_headset_adc_det *pdata = dev_id;
		//dump_stack();

        #if 0
        if( !delayed_work_pending(&pdata->gpio_check) ){
                schedule_delayed_work(&pdata->gpio_check,
                    msecs_to_jiffies(500) );  // 50ms for gpio debounce.
                // 20160102,we use the adc to check weather had mic.so when headset is insert
                // by gpio detect,we need to wait some time for the adc to check weather with mic

		}
		#else
		//cancel_delayed_work(&pdata->gpio_check);
		schedule_delayed_work(&pdata->gpio_check, msecs_to_jiffies(100) );  // 50ms for gpio debounce.
		#endif

        /*if( irq == pdata->micdet_irq){
            pdata->gpio_irq_active_flag |= IRQ_FLAG_MICDET;
        } else {
            pdata->gpio_irq_active_flag |= IRQ_FLAG_HEADSET;
        }*/
        disable_irq_nosync( irq );

		DBG("%s:headset gpio %d value=%d,disable irq %d!\n", __func__ ,
			pdata->headset_gpio, gpio_get_value(pdata->headset_gpio), irq);

		wake_lock_timeout(&pdata->headset_wl, msecs_to_jiffies(2000));
        // 20170510,when unplug/plug,we maygot a error adc ,so cancel the work.
        return IRQ_HANDLED;
}

static int rockchip_headset_adcdet_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct rk_headset_adc_det *pdata;
	enum of_gpio_flags flags;
	int ret;

    DBG("====enter %s====\n" , __func__ );
	pdata = kzalloc(sizeof(struct rk_headset_adc_det), GFP_KERNEL);
	if (pdata == NULL) {
		printk("%s failed to allocate driver data\n",__FUNCTION__);
		return -ENOMEM;
	}
	memset(pdata,0,sizeof(struct rk_headset_adc_det));
	p_hsdet_data = pdata;

	// init our default value.
    pdata->isMic = false;
    pdata->headset_plugin = false;
    pdata->headset_status = 0;
    pdata->adc_sample_ms = HSDET_ADC_SAMPLE_TIME_MS;
    pdata->report_media_unplug = false;
    //pdata->dbg_adc = false;

    pdata->chan = iio_channel_get(&pdev->dev, NULL);
    if (IS_ERR(pdata->chan))
    {
        pdata->chan = NULL;
        printk("%s() have not set adc chan\n", __FUNCTION__);
        // goto err_free;
    } else {
        ret = of_property_read_u32(node, "headset-adc", &pdata->adc_headset );
        if (ret < 0 ) {
                printk("%s: get headset-adc failed!\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_headset = -1;
        }
        ret = of_property_read_u32(node, "mic-adc", &pdata->adc_mic_min );
        if (ret < 0) {
                printk("%s: get mic-adc failed\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_mic_min = -1;
        }
        /*if( pdata->adc_headset < 0 && pdata->adc_mic_min < 0 ){
            printk("%s:we got adc channel,but no adc value set(for headset or mic)\n", __FUNCTION__);
            // goto err_free;
        }*/

        // media adc??
        ret = of_property_read_u32(node, "media-adc", &pdata->adc_media );
        if (ret < 0) {
                printk("%s: get media-adc failed\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_media = -1;
        }

        // vol-up adc??
        ret = of_property_read_u32(node, "volup-adc", &pdata->adc_vol_up);
        if (ret < 0) {
                printk("%s: get volup-adc failed\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_vol_up = -1;
        }

        // vol-down adc??
        ret = of_property_read_u32(node, "voldown-adc", &pdata->adc_vol_down );
        if (ret < 0) {
                printk("%s: get voldown-adc failed\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_vol_down = -1;
        }

        ret = of_property_read_u32(node, "debug-adc", &pdata->adc_dbg_key );
        if (ret < 0) {
                printk("%s: get hp-debug-adc failed\n", __FUNCTION__);
                //goto err_free;
                pdata->adc_dbg_key = -1;
        }

        // read some flag.
        // vol-down adc??
        ret = of_property_read_u32(node, "adc_sample_ms", &pdata->adc_sample_ms);
        if (ret < 0) {
                printk("%s: get adc_sample_ms failed,default=%d\n", __FUNCTION__, pdata->adc_sample_ms);
        }

        /*if( of_get_property(node, "dbg_adc", NULL) ){
    	    printk("%s:dbg_adc!!\n",__func__);
    	    pdata->dbg_adc = true;
    	} */
    }

#if SUPPORT_FIX_HEADSET
    ret = of_property_read_u32(node, "fix_headset", &pdata->fix_headset);
    if (ret < 0) {
       pdata->fix_headset = -1;
    }
#else
    pdata->fix_headset = -1;
#endif

    printk("adc_headset=%d,adc_mic=%d,media-adc=%d,fix_headset=%d\n", pdata->adc_headset,pdata->adc_mic_min,
        pdata->adc_media, pdata->fix_headset );


    if( of_get_property(node, "unplug_report_key", NULL) ){
	    printk("%s:unplug_report_key!!\n",__func__);
	    pdata->report_media_unplug = true;
	}


    pdata->micdet_gpio = of_get_named_gpio_flags(node, "micdet_gpio", 0, &flags);
 	if (pdata->micdet_gpio < 0 ) {
 	    // 20150908,NO GPIO INT support is also OK.
 	    printk("%s: Failed to get micdet gpio,ret=%d!\n" , __FUNCTION__ , pdata->micdet_gpio );
 	    pdata->micdet_irq = -1;
 	} else {
     	if (gpio_request(pdata->micdet_gpio, "micdet") != 0) {
     		printk("%s: Failed to request gpio %d\n" , __FUNCTION__ , pdata->micdet_gpio);
     	} else {
     	        pdata->micdet_irq = gpio_to_irq( pdata->micdet_gpio );

     	}
   }

    pdata->ihone_ic_pwr_gpio = of_get_named_gpio_flags(node, "pwron_gpio", 0, &flags);
 	if (pdata->ihone_ic_pwr_gpio < 0 ) {
 	    // 20150908,NO GPIO INT support is also OK.
 	    printk("%s: Failed to get ihone_ic_pwr_gpio,ret=%d!\n" , __FUNCTION__ , pdata->micdet_gpio );
 	} else {
     	if (gpio_request(pdata->ihone_ic_pwr_gpio, "headset-pwr") != 0) {
     		printk("%s: Failed to request gpio %d\n" , __FUNCTION__ , pdata->ihone_ic_pwr_gpio);
     	} else {
     	    pdata->ihone_ic_pwr_active_flag = !(flags & OF_GPIO_ACTIVE_LOW);
     	    set_iphone_IC_power(pdata,0); // disable the IC power.
     	}
   }

    pdata->headset_gpio = of_get_named_gpio_flags(node, "headset_gpio", 0, &flags);
	if (pdata->headset_gpio < 0 ) {
 	    // 20150908,NO GPIO INT support is also OK.
 	    printk("%s: Failed to get headset gpio,ret=%d!\n" , __FUNCTION__ , pdata->headset_gpio );
 	    pdata->headset_irq = -1;
 	    if( pdata->chan == NULL ){
 	        goto err_free;
 	    }
 	} else {
     	if (gpio_request(pdata->headset_gpio, "headset-det") != 0) {
     		printk("%s: Failed to request gpio %d\n" , __FUNCTION__ , pdata->headset_gpio);
     	} else {
     	    pdata->headset_irq = gpio_to_irq( pdata->headset_gpio );
     	}
   }

	pdata->sdev.name = "h2w";
	pdata->sdev.print_name = hsdet_h2w_print_name;
	ret = switch_dev_register(&pdata->sdev);
	if (ret < 0) {
	        printk("%s: switch_dev_register failed\n", __FUNCTION__);
		goto err_free;
    }

    switch_set_state(&pdata->sdev, 0);
        // Create and register the input driver.
	pdata->input_dev = input_allocate_device();
	if (!pdata->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_free;
	}
	pdata->input_dev->name = pdev->name;
	pdata->input_dev->open = hsdet_key_open;
	pdata->input_dev->close = hsdet_key_close;
	pdata->input_dev->dev.parent = &pdev->dev;
	//input_dev->phys = KEY_PHYS_NAME;
	pdata->input_dev->id.vendor = 0x0001;
	pdata->input_dev->id.product = 0x0001;
	pdata->input_dev->id.version = 0x0100;

    if( pdata->report_media_unplug || pdata->adc_media >= 0 ||
        pdata->adc_vol_up >= 0 || pdata->adc_vol_down >= 0 ) {
    	// Register the input device
    	ret = input_register_device(pdata->input_dev);
    	if (ret) {
    		dev_err(&pdev->dev, "failed to register input device\n");
    		goto err_free;
    	}

        if( pdata->report_media_unplug || pdata->adc_media >= 0 ) {
    	    input_set_capability(pdata->input_dev, EV_KEY, HOOK_KEY_CODE);
    	}
    	if( pdata->adc_vol_up >= 0 ) {
    	    input_set_capability(pdata->input_dev, EV_KEY, KEY_VOLUMEUP);
    	}
    	if( pdata->adc_vol_down >= 0 ) {
    	    input_set_capability(pdata->input_dev, EV_KEY, KEY_VOLUMEDOWN);
    	}
    }

    if( pdata->adc_dbg_key >= 0 ) {
        input_set_capability(pdata->input_dev, EV_KEY, HP_DEBUG_CODE);

    }
    if( pdata->chan != NULL ) {
	    INIT_DELAYED_WORK(&pdata->adc_check, adc_check_work_callback);

	    //20160102,HSL,if we have gpio,WE DON'T NEED start the adc check work.
	    // only the phone is insert that we need to start the adc check work.
	    if( pdata->headset_irq < 0 &&
	        pdata->micdet_irq < 0 ){
    	    schedule_delayed_work(&pdata->adc_check, msecs_to_jiffies(3000));
        }
    }

    wake_lock_init(&pdata->headset_wl, WAKE_LOCK_SUSPEND, "htfy_headset_wl");

    if( pdata->micdet_irq > 0 || pdata->headset_irq > 0 ){
		INIT_DELAYED_WORK(&pdata->gpio_check,gpio_check_work_callback);
		if(pdata->micdet_irq > 0){
			 ret =  request_irq(pdata->micdet_irq,
     	            headset_gpio_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
     	            "micdet" , pdata);
                    if (ret < 0) {
                        printk("%s: Failed to request irq for gpio %d\n" , __FUNCTION__ , pdata->micdet_gpio);
                   } else {
     	             gpio_direction_input( pdata->micdet_gpio );
     	             disable_irq( pdata->micdet_irq );
     	        }
     	        pdata->mic_level_micdet = !(flags & OF_GPIO_ACTIVE_LOW);
		}
		if(pdata->headset_irq > 0 && pdata->fix_headset < 0 ){
		    
			pdata->headset_in_level = !(flags & OF_GPIO_ACTIVE_LOW);
			if(pdata->headset_in_level){
			ret =  request_irq(pdata->headset_irq,
                    headset_gpio_irq, IRQF_TRIGGER_RISING,
     	            "headset-det" , pdata);
			}else{
				ret =  request_irq(pdata->headset_irq,
                    headset_gpio_irq, IRQF_TRIGGER_FALLING, //IRQF_TRIGGER_NONE ,IRQF_ONESHOT|
     	            "headset-det" , pdata);
			}
			if (ret < 0) {
				printk("%s: Failed to request irq for gpio %d\n" , __FUNCTION__ , pdata->headset_gpio);
			} else {
                //Songsayit.太早产生 耳机中断，在插着耳机开机的情况可能会产生异常。
				disable_irq_nosync( pdata->headset_irq );
                gpio_direction_input( pdata->headset_gpio );
			}

            // 20210514: can only enable at screen-on-sleep.
            //device_init_wakeup(&pdev->dev, true);
		    // enable_irq_wake(pdata->headset_irq);
		}

		// 20150821,HSL,schedul for first time( when micdet low at power on ).
		schedule_delayed_work(&pdata->gpio_check,
				msecs_to_jiffies(5000));  // 5s for gpio check.		
	}
	return 0;

err_free:
	kfree(pdata);
	return ret;
}

static int rockchip_headset_adcdet_remove(struct platform_device *pdev)
{
	struct rk_headset_adc_det *pdata = p_hsdet_data;
	if(pdata->headset_irq > 0 ) free_irq(pdata->headset_irq, pdata);
	kfree(pdata);
	return 0;
}

static int rockchip_headset_adcdet_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rk_headset_adc_det *pdata = p_hsdet_data;
	printk("htfy_headset: suspend,headset irq=%d,fb off=%d\n", pdata->headset_irq, fb_power_off());
	if(fb_power_off()) {
		pdata->suspended = true;
		cancel_delayed_work(&pdata->gpio_check);
		if(pdata->headset_irq > 0 ) {
		    //disable_irq_wake(pdata->headset_irq); // Unbalanced IRQ 56 wake disable
			disable_irq( pdata->headset_irq );
			// 20191227,we will shut off the GPIO-POWER,so set to high.
			//irq_set_irq_type(pdata->headset_irq, IRQF_TRIGGER_HIGH);
		}
		if(pdata->micdet_irq > 0) disable_irq( pdata->micdet_irq );
	} else {
		if(pdata->headset_irq > 0) enable_irq_wake(pdata->headset_irq);
	}
	return 0;
}

static int rockchip_headset_adcdet_resume(struct platform_device *pdev)
{
	struct rk_headset_adc_det *pdata = p_hsdet_data;
	printk("htfy_headset: resume,headset irq=%d\n", pdata->headset_irq);
	pdata->suspended = false;
	//if(pdata->headset_irq > 0 ) {
	//	irq_set_irq_type(pdata->headset_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
		//enable_irq( pdata->headset_irq );
	//}
	//if(pdata->micdet_irq > 0 ) enable_irq( pdata->micdet_irq );
	if(fb_power_off()) {
		schedule_delayed_work(&pdata->gpio_check, msecs_to_jiffies(1567) );
	} else {
		if(pdata->headset_irq > 0) disable_irq_wake(pdata->headset_irq);
	}
	return 0;
}

static const struct of_device_id rockchip_headset_adcdet_of_match[] = {
        { .compatible = "htfyun,headset", },
        {},
};
MODULE_DEVICE_TABLE(of, rockchip_headset_adcdet_of_match);

static struct platform_driver rockchip_headset_adcdet_driver = {
	.probe	= rockchip_headset_adcdet_probe,
	.remove = rockchip_headset_adcdet_remove,
	.resume = 	rockchip_headset_adcdet_resume,
	.suspend = 	rockchip_headset_adcdet_suspend,
	.driver	= {
		.name	= "htfyun_headset",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rockchip_headset_adcdet_of_match),
	},
};

static int __init rockchip_headset_adcdet_init(void)
{
	platform_driver_register(&rockchip_headset_adcdet_driver);
	return 0;
}

static void __exit rockchip_headset_adcdet_exit(void)
{
	platform_driver_unregister(&rockchip_headset_adcdet_driver);
}
late_initcall(rockchip_headset_adcdet_init);
MODULE_DESCRIPTION("Htfyun Headset Core Driver");
MODULE_LICENSE("GPL");

