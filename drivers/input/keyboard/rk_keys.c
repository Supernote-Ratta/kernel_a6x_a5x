/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/rk_keys.h>

// 20210806: 定义HALL 按键是否需要唤醒系统。X1上面 HALL 只需要盒盖的时候进入
// 休眠就可以了。但是 T2 要求开盖的时候要唤醒系统。另外盒盖的时候按 POWER 等
// 按键不能唤醒系统。此处增加编译开关来区分.
#define HALL_WAKUP 					1

#define EMPTY_DEFAULT_ADVALUE		1024

// 20210806: 我们现在使用 1.8V IO,误差可以改小一点。
#define DRIFT_DEFAULT_ADVALUE		50	// 105mv
#define INVALID_ADVALUE			-1
#define EV_ENCALL			KEY_F4
#define EV_MENU				KEY_F1

#if 0
#define key_dbg(bdata, format, arg...)		\
	dev_info(&bdata->input->dev, format, ##arg)
#else
#define key_dbg(bdata, format, arg...)
#endif

#define DEBOUNCE_JIFFIES	(30 / (MSEC_PER_SEC / HZ) + 1)	/* 60ms */
#define ADC_SAMPLE_JIFFIES	(100 / (MSEC_PER_SEC / HZ))	/* 100ms */

// 20220711: change longer for SNX slow wakeup.
#ifdef CONFIG_SUPPORT_ULTRU_SLEEP
#define WAKE_LOCK_JIFFIES	(12 * HZ)			/* 12s */
#else 
#define WAKE_LOCK_JIFFIES	(2 * HZ)			/* 1s */
#endif 

// 20191227: 开机后多久时间轮询案件状态。
#define TRIGER_ACTIVE_KEY_JIFFIES (25*HZ) // ( (30*1000) / (MSEC_PER_SEC / HZ))

enum rk_key_type {
	TYPE_GPIO = 1,
	TYPE_ADC
};

struct rk_keys_button {
	struct device *dev;
	u32 type;		/* TYPE_GPIO, TYPE_ADC */
	u32 code;		/* key code */
	const char *desc;	/* key label */
	u32 state;		/* key up & down state */
	int gpio;		/* gpio only */
	int irq;        /* the irq of this gpio.*/
	int adc_value;		/* adc only */
	int adc_state;		/* adc only */
	int active_low;		/* gpio only */
	int wakeup;		/* gpio only */
	int wake_enabled;

	int disabled; // 20210802:考虑按键的兼容问题（比如T2 HALL 按键）。
	struct timer_list timer;
};

struct rk_keys_drvdata {
	int nbuttons;
	/* flag to indicate if we're suspending/resuming */
	bool in_suspend;

	bool need_adc;
	bool check_hwver;
	int result;
	int rep;
	int drift_advalue;
	struct wake_lock wake_lock;
	struct input_dev *input;
	struct delayed_work adc_poll_work;
	struct iio_channel *chan;

	// 20180627,hsl add for HALL-key.
	struct rk_keys_button* hall_button;
	struct rk_keys_button button[0];
};

static struct input_dev *sinput_dev;

extern bool fb_power_off( void );


#define HWVER0_EARLY					0
#define HWVER1_HALL						1
#define HWVER2_FUTURE					2

// adc 是 0.48V, (0.48/1.8)*1023
#define HWVER_ADC_HALL 					273

static int px30_hw_version = HWVER0_EARLY;
int htfyun_get_hw_version(void)
{
    return px30_hw_version;
}
//EXPORT_SYMBOL(htfyun_get_hw_version);


static bool gpio_key_active(struct rk_keys_button *button)
{
    int state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^
			   button->active_low);
    return state != 0;
}

#if !HALL_WAKUP
// call when system wakeup.
static void update_hall_button( struct rk_keys_drvdata *pdata)
{
    if( pdata->hall_button ) {
        pdata->hall_button->state = !!((gpio_get_value(pdata->hall_button->gpio) ? 1 : 0) ^
			   pdata->hall_button->active_low);
    }
}
#endif 

static bool is_hall_covered( struct rk_keys_drvdata *pdata)
{
    if( pdata->hall_button && pdata->hall_button->state ) {
        return true;
    }
    return false;
}
static void key_send_wakeup_key(struct input_dev *input)
{
	input_report_key(input, KEY_WAKEUP, 1);
	input_sync(input);
	input_report_key(input, KEY_WAKEUP, 0);
	input_sync(input);
}


bool rk_key_hall_covered(void)
{
	if (!sinput_dev)
		return false;
	return is_hall_covered(input_get_drvdata(sinput_dev));
}

// 20180930:需求:盒盖的情况下，插拔 USB不能唤醒系统。
// 我们在这里增加是否盒盖的判断就好了。 --这个函数会在外部被调用，比如USB检测。
void rk_send_wakeup_key(void)
{
    struct rk_keys_drvdata *pdata;

	if (!sinput_dev)
		return;

    // 20180930: hsl add.
    pdata = input_get_drvdata(sinput_dev);
    // printk("%s: hall_button=%p!\n", __func__, pdata->hall_button);
    if( is_hall_covered(pdata) ) {
        // 20180930： 当插拔 USB的时候，打印下面的LOG:
        // rk_send_wakeup_key: Abort because HallCover!
        // 但是系统还是被唤醒了，应该是android 上面的配置。
        printk("%s: Abort because HallCover!\n", __func__);
        return ;
    }
	key_send_wakeup_key(sinput_dev);
}
EXPORT_SYMBOL(rk_send_wakeup_key);


// 20210630: 主要用来处理 T2 皮套盖上后进入休眠的问题.
void rk_send_sleep_key(void)
{
    struct input_dev *input = sinput_dev;

	if (!input)
		return;

	// pr_err("rk_send_sleep_key!!\n");
    input_report_key(input, KEY_SLEEP, 1);
	input_sync(input);
	input_report_key(input, KEY_SLEEP, 0);
	input_sync(input);
}
EXPORT_SYMBOL(rk_send_sleep_key);

void rk_send_power_key(int state)
{
    struct rk_keys_drvdata *pdata;
	if (!sinput_dev)
		return;

	// 20180930: hsl add.
    pdata = input_get_drvdata(sinput_dev);
    if( is_hall_covered(pdata) ) {
        printk("%s: Abort because HallCover!\n", __func__);
        return ;
    }
	if (state) {
		input_report_key(sinput_dev, KEY_POWER, 1);
		input_sync(sinput_dev);
	} else {
		input_report_key(sinput_dev, KEY_POWER, 0);
		input_sync(sinput_dev);
	}
}
EXPORT_SYMBOL(rk_send_power_key);

static void key_send_key(struct input_dev *input, struct rk_keys_button *button)
{
    input_event(input, EV_KEY, button->code, button->state);
    input_sync(input);
    if( button->state ) {  // 20180212,hsl add.only wakeup on Key-Down.
        //sm8951_fast_wakeup();
        if(fb_power_off()) {
            struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
		    key_send_wakeup_key(input); // 20190313: no HallCover Check!
            wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);
        } 
    }
}

static void keys_timer(unsigned long _data)
{
	struct rk_keys_button *button = (struct rk_keys_button *)_data;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;
	int state;

	if (button->type == TYPE_GPIO) {
		state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^
			   button->active_low);
	    //printk("%s: key=%s,state=%d,old state=%d\n", __func__, button->desc,
	    //   state, button->state);
	    enable_irq(button->irq);
    }
	else
		state = !!button->adc_state;
	key_dbg(pdata,"%s: key=%s,state=%d/%d,hall-cover=%d\n", __func__,
        button->desc, state, button->state, is_hall_covered(pdata));
        
	if (button->state != state) {
		button->state = state;
		if( button == pdata->hall_button ) {
			// 20210806: hall 按键比较特殊，一个电平就要发送 DOWN/UP 按键，需要模拟。
			if(state) {
				// 20210806: 如果是盒盖的情况下产生的HALL中断，我们需要发送WAKEUP。
		    	// hall 本身的按键是： KEY_SLEEP。
		    	key_dbg(pdata,"%s: send Sleep for hall_button on!\n", __func__);
		    	rk_send_sleep_key();
			} else {
				// 20210806: 如果是盒盖的情况下产生的HALL中断，我们需要发送WAKEUP。
		    	// hall 本身的按键是： KEY_SLEEP。
		    	key_dbg(pdata,"%s: send WakeUp for hall_button off!\n", __func__);
		    	key_send_wakeup_key(input);
			}
		} else {
			 // 20190313：由于面板在背面的时候也有可能导致 HALL 感应生效，所以需求即使HALL盖着，按HOME按键
		     // 也要唤醒系统。此处增加判断. --我们在 keys_isr 里面已经判断了盒盖和开盖的情况，但是考虑到 ADC 按键也会
		     // 进入这个流程，所以此处还需要做判断。
		     if( is_hall_covered(pdata) ) {
		        //盒盖的情况下，只能发送什么按键呢?
		        #if 0
		        if( (KEY_SLEEP == button->code) ) {
			        printk("%s: Send %s because HallCover!\n", __func__, button->desc);
			        // 此处发送一个完整的 SLEEP按键。
			        input_event(input, EV_KEY, button->code, button->state);
	    		    input_sync(input);

			        input_event(input, EV_KEY, button->code, 0/*button->state*/);
	    	        input_sync(input);
			    } else if(KEY_HOME == button->code) {
	                key_send_key(input, button);
			    } else
			    #endif 
			    if(KEY_POWER == button->code && !fb_power_off() ) {
	                key_send_key(input, button);
			    } else {
			        printk("%s: Don't send %s because HallCover!\n", __func__, button->desc);
			    }
		     } else {
		        // 开盖情况下，可以发送任意按键。普通的按钮会根据状态有按下/弹起两个按键。
		        // printk("%s: send %s keyEvent!\n", __func__, button->desc);
		        key_send_key(input, button);
		     }
	     }
	}

    //20191227：我们使用双 edge 触发，此处不需要启动定时器。
	/*if (state)
		mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);
	*/

}

static irqreturn_t keys_isr(int irq, void *dev_id)
{
	struct rk_keys_button *button = (struct rk_keys_button *)dev_id;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;

    int state;

	//BUG_ON(irq != button->irq/* gpio_to_irq(button->gpio)*/);
	state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^
			   button->active_low);

    key_dbg(pdata,"%s: irq=%d,key=%s,state=%d,hall-cover=%d,insuspend=%d\n", __func__,
        irq, button->desc, state, is_hall_covered(pdata), pdata->in_suspend);
        
#if !HALL_WAKUP	
    if( button == pdata->hall_button ) {
        if( state ) {
            // 此处是盒盖，通过 timer 来发送消息，防止抖动。此处不能 调用 update_hall_button，否则 keys_timer 里面
    		// 由于 button->state != state 的判断导致无按键上报。
            goto star_timer;
        } else {
            // 此处是开盖，目前不做处理。但是需要设置新的 state,否则下次再盒盖的时候就无法判断了。
            button->state = state;
            return IRQ_HANDLED;
        }
    } else if(pdata->in_suspend) {
		// 20190323：由于HALL按键不会唤醒系统，且HALL按键具有状态属性，所以有可能在系统休眠的时候，HALL按键发生了状态变化。
		// 且 is_hall_covered 也需要判断 HALL 的state，所以此处需要更新。--更新了HALL state会导致 HALL 按下按键无法派发。
		update_hall_button(pdata);
	}


    if(is_hall_covered(pdata)) {
        // 20190323: 我们允许盒盖的情况下，发送 POWER 按键进入休眠，但是不能唤醒.
	    if( button->code == KEY_POWER && !fb_power_off() ) {
	        printk("%s: Don't send %s because HallCover & FB ON!\n", __func__, button->desc);
	        goto star_timer;
	    }

	    // 如果 HALL 开关是按着的，表示盖子盖上了，所以不需要唤醒.
	    // 20190313：由于面板在背面的时候也有可能导致 HALL 感应生效，所以需求即使HALL盖着，按HOME按键
	    // 也要唤醒系统。此处增加判断.
	    if( button->code != KEY_HOME) {
	        printk("%s: Don't send %s because HallCover!\n", __func__, button->desc);
	        return IRQ_HANDLED;
	    }
	}

	// 20180918: 此处，HALL 开关是打开的，即盖子是打开的.如果按键是可以唤醒系统，判断
	// 是否需要发送 wakeup_key 和 按键码(一般情况下，按键码都是在 keys_timer 里面发送)
	if (button->wakeup && fb_power_off()) {

        key_dbg(pdata,"%s: send wakeup_key by %s!\n", __func__, button->desc);
        //rk_send_wakeup_key();
        //key_send_wakeup_key(input); // 20190313: no HallCover Check!
        //wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);

        // 20180922: 对于 power按键，功能就是休眠的时候唤醒系统。此处不能在发送power 按键，否则
        // 休眠唤醒会有问题。20190626：要求在休眠状态下，按 HOME键只唤醒系统，不要退回到HOME的功能。
        if( button->code == KEY_POWER || button->code == KEY_HOME) {
            key_send_wakeup_key(input); // 20190313: no HallCover Check!
            wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);
            return IRQ_HANDLED;
        }

        //20180918: 启动一个定时器发送按键。但是如果是suspend状态，先发送按键，否则
        // 由于系统延迟较大，按键可能会丢失。
        if( pdata->in_suspend ) {
            button->state = 1; /* state;*/
            key_dbg(pdata,
        			"wakeup: %skey[%s]: report event[%d] state[%d]\n",
        			(button->type == TYPE_ADC) ? "adc" : "gpio",
        			button->desc, button->code, button->state);
        	key_send_key(input, button);
		}
    }
star_timer:    
#else
	// 20210806: 休眠情况下，按键先发送。担心如果休眠唤醒需要处理时间比较长，在
	// timer 里面再判断按键的状态的时候，已经变了。比如用户快速按按钮唤醒系统。
	// 由于PX30做了优化，此处先不做处理看看.
	// 20210811: 处理之后发现 JYT 项目按HOME键无法唤醒系统，此处需要把 HOME 改为 wakeup,
	// 用来唤醒系统，且屏蔽掉原来的HOME功能。灭屏情况下，只需要按下的时候亮屏就可以了。
	if ( fb_power_off() && state) {  // button->wakeup &&
        key_dbg(pdata,"%s: send wakeup_key by %s,state=%d,code=%d!\n", __func__, 
        	button->desc, state, button->code);
        if( button->code == KEY_POWER || button->code == KEY_HOME) {
            key_send_wakeup_key(input); // 20190313: no HallCover Check!
            wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);
            return IRQ_HANDLED;
        }
    }
#endif 

    //20180922: 由于 HOME GPIO在按下的过程中抖动比较厉害（产生多次边缘触发），此处临时disable irq ？
    disable_irq_nosync(irq);

	mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);

	return IRQ_HANDLED;
}

/*
static ssize_t adc_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "adc_value: %d\n", ddata->result);
}
static DEVICE_ATTR(get_adc_value, S_IRUGO | S_IWUSR, adc_value_show, NULL);
*/

static const struct of_device_id rk_key_match[] = {
	{ .compatible = "rockchip,key", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, rk_key_match);

static int rk_key_adc_iio_read(struct rk_keys_drvdata *data)
{
	struct iio_channel *channel = data->chan;
	int val, ret;

	if (!channel)
		return INVALID_ADVALUE;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val;
}

static void adc_key_poll(struct work_struct *work)
{
	struct rk_keys_drvdata *ddata;
	int i, result = -1;

	ddata = container_of(work, struct rk_keys_drvdata, adc_poll_work.work);
	if (!ddata->in_suspend) {
		result = rk_key_adc_iio_read(ddata);
		if (result > INVALID_ADVALUE &&
		    result < EMPTY_DEFAULT_ADVALUE )
			ddata->result = result;
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = &ddata->button[i];

            // 20180918: 有些adc按键按下后接地，ADC的值是0.另外dup key里面把一个非法的key
            // type 置为 -1，这里也要过来。
			if (button->type != TYPE_ADC /*!button->adc_value*/)
				continue;

			if (result < button->adc_value + ddata->drift_advalue &&
			    result > button->adc_value - ddata->drift_advalue)
				button->adc_state = 1;
			else
				button->adc_state = 0;
			if (button->state != button->adc_state)
				mod_timer(&button->timer,
					  jiffies + DEBOUNCE_JIFFIES);
		}
	}

    if( ddata->need_adc ) {
	    schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
	}
}

static int rk_key_type_get(struct device_node *node,
			   struct rk_keys_button *button)
{
	u32 adc_value;

	if (!of_property_read_u32(node, "rockchip,adc_value", &adc_value))
		return TYPE_ADC;
	else if (of_get_gpio(node, 0) >= 0)
		return TYPE_GPIO;
	else
		return -1;
}

static int rk_keys_parse_dt(struct rk_keys_drvdata *pdata,
			    struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child_node;
	struct iio_channel *chan;
	int ret, gpio, i = 0;
	u32 code, adc_value, flags, drift;

	if (of_property_read_u32(node, "adc-drift", &drift))
		pdata->drift_advalue = DRIFT_DEFAULT_ADVALUE;
	else
		pdata->drift_advalue = (int)drift;

	pdata->need_adc = false;
	chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "no io-channels defined\n");
		chan = NULL;
	} else {
#ifdef CONFIG_SUPPORT_HALL_BUTTON	
		// 20210802: 目前只有T2的项目需要做这个硬件版本兼容判断。编译开关不好控制，此处在 DTS 
		// 上面增加标识来表示。20210816: 在menuconfig 里面增加了功能开关。
		pdata->check_hwver =
			    !!of_get_property(node, "hwver_check", NULL);
#endif 			    
	}
	pdata->chan = chan;
	// dev_info(&pdev->dev, "check_hwver=%d,chan=%p\n", pdata->check_hwver, pdata->chan);
	
	 // 20191225,hsl add.
	for_each_child_of_node(node, child_node) {
		if (of_property_read_u32(child_node, "linux,code", &code)) {
			dev_err(&pdev->dev,
				"Missing linux,code property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		pdata->button[i].code = code;
		pdata->button[i].desc =
		    of_get_property(child_node, "label", NULL);
		pdata->button[i].type =
		    rk_key_type_get(child_node, &pdata->button[i]);
		switch (pdata->button[i].type) {
		case TYPE_GPIO:
			gpio = of_get_gpio_flags(child_node, 0, &flags);
			if (gpio < 0) {
				ret = gpio;
				if (ret != -EPROBE_DEFER)
					dev_err(&pdev->dev,
						"Failed to get gpio flags, error: %d\n",
						ret);
				goto error_ret;
			}

			pdata->button[i].gpio = gpio;
			pdata->button[i].active_low =
			    flags & OF_GPIO_ACTIVE_LOW;
			pdata->button[i].wakeup =
			    !!of_get_property(child_node, "gpio-key,wakeup", NULL);
			break;

		case TYPE_ADC:
			if (of_property_read_u32
			    (child_node, "rockchip,adc_value", &adc_value)) {
				dev_err(&pdev->dev,
					"Missing rockchip,adc_value property in the DT.\n");
				ret = -EINVAL;
				goto error_ret;
			}
			pdata->button[i].adc_value = adc_value;
			if( pdata->chan ) pdata->need_adc = true;
			break;

		default:
			dev_err(&pdev->dev,
				"Error rockchip,type property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}

		// 20210802: 此处提前设置 hall_button, 主要考虑兼容问题。如果不支持 hall 的版本，
		// 我们不希望 getevent -i 显示这个 hall 按键，否则FT测试可能会有异常。
		if( pdata->button[i].code == KEY_SLEEP){
		    pdata->hall_button = &pdata->button[i];
		}
		i++;
	}

	return 0;

error_ret:
	return ret;
}

#ifdef CONFIG_SUPPORT_HALL_BUTTON
static void probe_hw_version_by_adc_channel(struct rk_keys_drvdata *ddata)
{
	int result;
	//if(ddata->chan == NULL) return;
	result = rk_key_adc_iio_read(ddata);

	// 20210802-LOG: hwv1: rk_Keys:probe hw-version,adc=273,drift=60
	// hwv0: rk_Keys:probe hw-version,adc=1016,drift=60
	//		 rk_Keys:disable hall_button for HWVER0_EARLY!adc=1016
	printk("rk_Keys:probe hw-version,adc=%d,drift=%d,need-adc=%d\n", result, 
		ddata->drift_advalue, ddata->need_adc);
	if(result < 0) {
		return;
	} else if( result < HWVER_ADC_HALL + ddata->drift_advalue &&
			    result > HWVER_ADC_HALL - ddata->drift_advalue ) {
		px30_hw_version = HWVER1_HALL;
	}

	// 20210802: 如果是早期版本，不具有 HALL 开关，需要屏蔽掉。
	if(px30_hw_version == HWVER0_EARLY) {
		if(ddata->hall_button) {
			// disable hall.
			ddata->hall_button->disabled = true;
			ddata->hall_button->wake_enabled = false;
			//__clear_bit(ddata->hall_button->code, ddata->input->keybit);
			//__clear_bit(EV_KEY, ddata->input->evbit);
			ddata->hall_button = NULL;
			printk("rk_Keys:disable hall_button for HWVER0_EARLY!adc=%d\n", result);
		}
	}
}
#endif 

static void trigger_active_gpio_key(struct rk_keys_drvdata *ddata)
{
    int  i;

    for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *bi = &ddata->button[i];
		//printk("Key %s:type=%d,gpio value=%d,act_low=%d,need_adc=%d\n", bi->desc,
		//    bi->type, gpio_get_value(bi->gpio), bi->active_low, ddata->need_adc);
		if( bi->type != TYPE_GPIO ) {
		    continue; // already be delete.
		}

		// 20210802: 被 disabled 的GPIO 按键不能产生中断。主要用于兼容考虑。
		if( bi->disabled ) {
			disable_irq(bi->irq);
			continue;
		}
		
		if(gpio_key_active(bi) ) {
			// 20200716: 由于在 keys_timer 里面会调用 enable_irq, 所以为了 balance，此处需要调用 disable_irq.
			disable_irq(bi->irq);
		    mod_timer(&bi->timer, jiffies + TRIGER_ACTIVE_KEY_JIFFIES+i);
		}
	}
}

static int keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rk_keys_drvdata *ddata = NULL;
	struct input_dev *input = NULL;
	int i, error = 0;
	int wakeup, key_num = 0;

	key_num = of_get_child_count(np);
	if (key_num == 0)
		dev_info(&pdev->dev, "no key defined\n");

	ddata = devm_kzalloc(dev, sizeof(struct rk_keys_drvdata) +
			     key_num * sizeof(struct rk_keys_button),
			     GFP_KERNEL);

	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		error = -ENOMEM;
		return error;
	}
	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(&pdev->dev, ddata);

	input->name = "rk29-keypad";	/* pdev->name; */
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	ddata->input = input;

	/* parse info from dt */
	ddata->nbuttons = key_num;
	error = rk_keys_parse_dt(ddata, pdev);
	if (error)
		goto fail0;

	/* Enable auto repeat feature of Linux input subsystem */
	if (ddata->rep)
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, error: %d\n",
		       error);
		goto fail0;
	}

	// 20180930,hsl add for rk_send_wakeup_key.
	input_set_drvdata(input, ddata);

#ifdef CONFIG_SUPPORT_HALL_BUTTON
	// 20210802: 在设置 Input capability 之前进判断。
	if(ddata->check_hwver) {
		probe_hw_version_by_adc_channel(ddata);
	}
#endif 

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];

		if( button->disabled ) continue; 
		
		if (button->code) {
			setup_timer(&button->timer,
				    keys_timer, (unsigned long)button);
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, EV_KEY, button->code);
	}

	// 20210806: hall 需要唤醒的话，要同时具有 SLEEP/WAKEUP key。
	// 20210811: 即使没有 hall按键，也有可能需要上报 WAKEUP 按键。比如休眠情况下按 HOME
	// 按键，这个时候需要唤醒系统，就发送 WAKEUP.所以此处增加 KEY_WAKEUP 的capability,否则
	// 就无法上报这个 WAKEUP 按键。
	if(wakeup || (ddata->hall_button && HALL_WAKUP)) {
		input_set_capability(input, EV_KEY, KEY_WAKEUP);
	}

	wake_lock_init(&ddata->wake_lock, WAKE_LOCK_SUSPEND, input->name);
	device_init_wakeup(dev, wakeup);

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];
	
		button->dev = &pdev->dev;
		
		if( button->disabled ) continue; 
		
		if (button->type == TYPE_GPIO) {
			//int irq;

			error =
			    devm_gpio_request(dev, button->gpio,
					      button->desc ? button->desc : "gpio-keys");
			if (error < 0) {
				pr_err("gpio-keys: failed to request GPIO %d, error %d\n",
				       button->gpio, error);
				goto fail1;
			}

			error = gpio_direction_input(button->gpio);
			if (error < 0) {
				pr_err("gpio-keys: failed to configure input direction for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			button->irq = gpio_to_irq(button->gpio);
			if (button->irq < 0) {
				error = button->irq;
				pr_err("gpio-keys: Unable to get irq number for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			error = devm_request_irq(dev, button->irq, keys_isr,
						 /*button->active_low ?
						 IRQF_TRIGGER_FALLING :
						 IRQF_TRIGGER_RISING*/
						 IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
						 button->desc ?
						 button->desc : "keys",
						 button);
			if (error) {
				pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				       button->irq, error);
				gpio_free(button->gpio);
				goto fail1;
			}
			/* if( button->code == KEY_SLEEP){
			    ddata->hall_button = button;
			}*/
		}
	}

	// 20210630:hsl add for touch-cover. 20210802:这里如果定义不存在的 SLEEP 按键，会导致
	// FT 测试通不过。如果需要增加，也要按项目区分。
	// input_set_capability(input, EV_KEY, KEY_SLEEP); 
	// input_set_capability(input, EV_KEY, KEY_WAKEUP);

	
	/* adc polling work */
	if (ddata->chan && ddata->need_adc) {
		INIT_DELAYED_WORK(&ddata->adc_poll_work, adc_key_poll);
		schedule_delayed_work(&ddata->adc_poll_work,
					  ADC_SAMPLE_JIFFIES);
	}
	
    // 20180918: hsl add:主要用于合着盖子开机的情况（有一个GPIO按键开机就处于按下状态）。
    // 由于中断采用边缘触发，所以无法检测到这种按键状态，需要手工触发。
    trigger_active_gpio_key(ddata);

	sinput_dev = input; // 20220110: set only probe OK.
	
	return error;

fail1:
	while (--i >= 0)
		del_timer_sync(&ddata->button[i].timer);
	device_init_wakeup(dev, 0);
	wake_lock_destroy(&ddata->wake_lock);
fail0:
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int keys_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(dev, 0);
	for (i = 0; i < ddata->nbuttons; i++)
		del_timer_sync(&ddata->button[i].timer);
	if (ddata->chan)
		cancel_delayed_work_sync(&ddata->adc_poll_work);
	input_unregister_device(input);
	wake_lock_destroy(&ddata->wake_lock);

	sinput_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int keys_suspend(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	ddata->in_suspend = true;
	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->disabled) {
				continue;
			}
			// 20191227: hall 按键在 灭屏休眠的时候不需要唤醒系统。但是在亮屏休眠的时候需要唤醒系统并进入sleep模式。
			// 20210806：需要在休眠时候开盖也一样唤醒系统.
			if (button->wakeup) {
				#if HALL_WAKUP
				enable_irq_wake(button->irq);
				button->wake_enabled = true;
				#else 
				if( button != ddata->hall_button || !fb_power_off() ) {
					//printk("%s:enable irq wake for %s,irq=%d\n", __func__, button->desc, button->irq);
					enable_irq_wake(button->irq);
					button->wake_enabled = true;
				}
				#endif 
			}
		}
	}

	return 0;
}
extern unsigned int pm_wakeup_irq;

static int keys_resume(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

// 20211123: 下面的代码回到导致 T2 出现 下面的异常：
// Internal error: Accessing user space memory outside uaccess.h routines: 96000005 [#1] PREEMPT SMP
// PC is at keys_resume+0x28/0x104
// LR is at platform_pm_resume+0x24/0x50 
// 原因还不清楚。增加的判断是用来解决 超低功耗下 HALL 开关唤醒的问题，目前只有 SNX项目是这个硬件电路。
#ifdef CONFIG_SUPPORT_ULTRU_SLEEP
#if HALL_WAKUP
	int state;
	struct input_dev *input = ddata->input;
	if(ddata->hall_button != NULL) {
		state = !!((gpio_get_value(ddata->hall_button->gpio) ? 1 : 0) ^
					   ddata->hall_button->active_low);

		printk("hall status = %x fb_power_off:%d  pm_wakeup_irq:%d \n",state, fb_power_off(),pm_wakeup_irq);
		if (pm_wakeup_irq == 0) {
			if (fb_power_off() && !state) {  // button->wakeup &&
	        	key_send_wakeup_key(input); // 20190313: no HallCover Check!
	        	wake_lock_timeout(&ddata->wake_lock, WAKE_LOCK_JIFFIES);
	    	}
		}
	}
#endif
#endif 
	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->wake_enabled /*button->wakeup*/) {
				disable_irq_wake(button->irq/*gpio_to_irq(button->gpio)*/);
				button->wake_enabled = false;
			}
		}

		#if 0
		preempt_disable();
		/* for call resend_irqs, which may call keys_isr */
		if (local_softirq_pending())
			do_softirq();
		preempt_enable_no_resched();
		#endif
	}

	ddata->in_suspend = false;

	return 0;
}

static const struct dev_pm_ops keys_pm_ops = {
	.suspend	= keys_suspend,
	.resume		= keys_resume,
};
#endif

static struct platform_driver keys_device_driver = {
	.probe		= keys_probe,
	.remove		= keys_remove,
	.driver		= {
		.name	= "rk-keypad",
		.owner	= THIS_MODULE,
		.of_match_table = rk_key_match,
#ifdef CONFIG_PM
		.pm	= &keys_pm_ops,
#endif
	}
};

static int __init rk_keys_driver_init(void)
{
	return platform_driver_register(&keys_device_driver);
}

static void __exit rk_keys_driver_exit(void)
{
	platform_driver_unregister(&keys_device_driver);
}

late_initcall_sync(rk_keys_driver_init);
module_exit(rk_keys_driver_exit);
