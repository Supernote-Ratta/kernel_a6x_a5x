// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019, Fuzhou Rockchip Electronics Co., Ltd.
 *
 * Rockchip rm310 driver for 4g modem
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/lte.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

//#define DONT_ALLOW_SUSPEND 
//#define ONLY_DEEP_SLEEP
#define KEEP_ACTIVE_WHEN_SUSPEN 		0	// 1: 深度休眠不断开4G-USB, 0:休眠断开USB.
#define KEEP_VBUSON_WHEN_SUSPEN 		0	// 1: 深度休眠断开4G网络，但是不断开USB, 0:休眠断开USB.

#define LOG(x...)   pr_info("[4g_modem]: " x)

// 20210716: 由于VBUS的原因，有些模组需要 12-13秒才能识别到USB。哪怕是只拉
// VBUS 也是这样。--20210717: 工厂新回来的板子，VBUS提高到3.5V，正常识别
// USB 的时间再 1.5S左右。 -- 20210809：2.5sec有时会太快超时。3sec比较合适(usb driver retry 也需要一点时间).
#define WAIT_FOR_USB_TIMEOUTS 	(3*HZ+HZ/2) 	//short		/* msecs_to_jiffies(100)*/
#define WAIT_FOR_USB_TIMEOUTL 	(14*HZ)	//long.

// 20210717: 目前 4G寻网时间大概时 13秒。另外还需要多遗留几秒（USB连接上了之后可能还需要
// 延迟一段时间才开始寻网操作）。
#define WAKELOCK_TIMEOUTX 		((13+3)*HZ)	

static struct lte_data *gpdata;
static struct class *modem_class;
//static int modem_status = 1;
//static int modem_wakeup = 1;

bool lte_ready = 0;

// 20210716: 增加一个判断，记录这个设备是否有USB模块。只要出现过一次USB-4G 设备，就必然
// 有 USB 模块。
bool lte_model_exist = 0;

bool lte_pwr_off_by_ft = 0;

/*
  20210407: 美格模组 MC302 GPIO 以及上电和休眠的时序说明如下：
  1. vbat: 电源开关，高的时候给4G 模组供电；给模组供电但是模组不会自动运行。
  2. power: 模组的开机按键，拉高一秒中再拉低，4G模组就会开机运行；
  3. reset: 模组的  reset管教，我们拉高模组处于复位状态； 复位时间要 200ms。
  4. vbus:  模组的USB 控制开关，我们拉高，模组就会把USB打开，这样HOST 才会检测到USB设备；
  5. wakeup: 模组的 wakeup, 模组检测到上升沿，处于正常工作(拉高)，下降沿就处于休眠状态(拉低)；
  6. disable：飞行模式控制IO，我们拉高，模组正常工作，拉低则进入飞行模式。在模组休眠状态下，
             如果不进入飞行模式，功耗10几毫安，如果进入飞行模式，则可以达到2ma。
  上电时序：
    1. 拉高 VBAT,供电； 2.拉高 power 1秒 再拉低； 3. 控制复位，拉高复位200ms 然后拉低；
  进入休眠时序： 
    1. 拉低 VBUS, 2. 把 disable 拉低， 3. 把 wakeup 拉低；
  退出休眠时许： 
    1. 先把wakeup 拉高， 2. 把 disable 拉高； 3. 拉高VBUS,打开USB；
 
*/
extern bool fb_power_off(void);
extern int ehci_wakeup_ctrl(bool wakeup);
extern int ehci_reset_ehci_host(void);
//extern int option_get_usb_dev_num(void);

void option_dev_change(bool add) 
{
	struct lte_data *pdata = gpdata;

	
	// pdata->found_dev = true;
	if(add) {
	    pdata->usb_dev_num++;
	    
	    // 20210707: 只要有设备发生变化了，就说明 4G 已经连接上了USB.
	    lte_model_exist = true;
	}else {
	    pdata->usb_dev_num--;
	    if(pdata->usb_dev_num==0) {
	        //20220521: make sure lte_ctrl_work dosomething.
	        pdata->active_when_suspend = false;
	        pdata->active = false; 
	    }
	}
    LOG(" lte_dev_change: active=%d,active_when_suspend=%d,dev_num=%d\n", pdata->active, 
        pdata->active_when_suspend, pdata->usb_dev_num);	
}

bool lte_keep_active_when_suspend(void) 
{	
	struct lte_data *pdata = gpdata;
	if(!pdata ) return false;
	return pdata->keep_active_suspend;
}

int lte_wakeup(bool wakeup)
{
	struct lte_data *pdata = gpdata;
	if(lte_ready ==0)
		return 0;
		
	mutex_lock(&pdata->ctrl_mutex);

	LOG(" lte_wakeup: active=%d,wakeup=%d,keep-vbus=%d\n", pdata->active, wakeup, pdata->keep_vbus_when_suspend);
 	if(pdata->active != wakeup) {

#ifdef CONFIG_DEBUG_BUILD 	
 	//dump_stack(); // 20210802: to check-suspend failed.(call from mmc/core.c)
#endif  	

	if(wakeup){
	    if (pdata->wakeup_gpio) {
			gpiod_direction_output(pdata->wakeup_gpio, 1);
			msleep(150);
		}

		if (pdata->disable_gpio) {
			gpiod_direction_output(pdata->disable_gpio, 1);
			//msleep(2500);	// 20210806: more time wait LTE active.
		}
		
		if (pdata->vbus_gpio) {
			gpiod_direction_output(pdata->vbus_gpio, 1);
		}
		
#ifdef CONFIG_LTE_YIYUAN //yiyuan 4g never disconnect
#else
		ehci_wakeup_ctrl(true);
#endif
		pdata->active = wakeup;
	} else{
		
#ifdef CONFIG_LTE_YIYUAN //yiyuan 4g never disconnect
#else
		pdata->active = wakeup;
		ehci_wakeup_ctrl(true);
		if(!pdata->keep_vbus_when_suspend || !fb_power_off()){
			if (pdata->vbus_gpio) {
				gpiod_direction_output(pdata->vbus_gpio, 0);
				msleep(20);
				ehci_wakeup_ctrl(false);
			}
		}
#endif
		if (pdata->disable_gpio) {
			gpiod_direction_output(pdata->disable_gpio, 0);
			//msleep(20);
		}
		
		if (pdata->wakeup_gpio) {
			gpiod_direction_output(pdata->wakeup_gpio, 0);
			//msleep(20);
		}
		
		//pdata->found_dev = false;
		pdata->operating = false;
	}

    LOG(" lte_wakeup: active=%d,keep-vbus=%d\n", wakeup, pdata->keep_vbus_when_suspend);
	}

	mutex_unlock(&pdata->ctrl_mutex);
	return 0;
}

/*
  20210407: 美格模组 MC302 GPIO 以及上电和休眠的时序说明如下：
  1. vbat: 电源开关，高的时候给4G 模组供电；给模组供电但是模组不会自动运行。
  2. power: 模组的开机按键，拉高一秒中再拉低，4G模组就会开机运行；
  3. reset: 模组的  reset管教，我们拉高模组处于复位状态； 复位时间要 200ms。
  4. vbus:  模组的USB 控制开关，我们拉高，模组就会把USB打开，这样HOST 才会检测到USB设备；
  5. wakeup: 模组的 wakeup, 模组检测到上升沿，处于正常工作(拉高)，下降沿就处于休眠状态(拉低)；
  6. disable：飞行模式控制IO，我们拉高，模组正常工作，拉低则进入飞行模式。在模组休眠状态下，
             如果不进入飞行模式，功耗10几毫安，如果进入飞行模式，则可以达到2ma。
  上电时序：
    1. 拉高 VBAT,供电； 2.拉高 power 1秒 再拉低； 3. 控制复位，拉高复位200ms 然后拉低；
  进入休眠时序： 
    1. 拉低 VBUS, 2. 把 disable 拉低， 3. 把 wakeup 拉低；
  退出休眠时许： 
    1. 先把wakeup 拉高， 2. 把 disable 拉高； 3. 拉高VBUS,打开USB；
 
*/
//never use this function for lte powerdown,only for factory test!!! 
//If you want to power down lte,use lte_wakeup
static int modem_poweron_off(bool pwr_on)
{
	struct lte_data *pdata = gpdata;
    mutex_lock(&pdata->ctrl_mutex);
	if(pdata->power_on != pwr_on) {
		LOG("modem_poweron_off: pwr_on=%d\n", pwr_on);
		if (pwr_on) {
            //if (pdata->power_gpio) {
        	//	gpiod_direction_output(pdata->power_gpio, 0);
        	//}
            
			if (pdata->vbat_gpio) {
				gpiod_direction_output(pdata->vbat_gpio, 1);
				msleep(100);
			}

			// 20210719: 这个类似于我们的POWER-按键，按下 1秒钟，模组开机。
			if (pdata->power_gpio) {
				gpiod_direction_output(pdata->power_gpio, 0);
				msleep(50);
				gpiod_direction_output(pdata->power_gpio, 1);
				msleep(1000);
				gpiod_direction_output(pdata->power_gpio, 0);
				msleep(50);
			}
			
			if (pdata->reset_gpio) {
			    gpiod_direction_output(pdata->reset_gpio, 0);
				msleep(10);
				gpiod_direction_output(pdata->reset_gpio, 1);
				msleep(200);
				gpiod_direction_output(pdata->reset_gpio, 0);

				// 20210719: 增加时间再 拉高 wakeup. --500改为1000无效。
				msleep(500); 
			}
			
		    if (pdata->wakeup_gpio) {
				gpiod_direction_output(pdata->wakeup_gpio, 1);

				//20210719，多预留一点时间再拉高VBUS，看看是否改善USB无法识别的问题。
				// --从延迟 50ms 变为 850ms，效果一样，开关 433次，无法识别的有 37次。
				// --加大 reset 之后的延迟和 disable 之后的延迟无效。开关54次有8次无法识别。
				// --wakeup 延迟改为2000ms，开关54次有4次无法识别。以上测试都是同一个样机。
				// 20210720: 从 00013设备看，时间长一点识别不到的概率低一点。
				msleep(200);  
			}

			if (pdata->disable_gpio) {
				gpiod_direction_output(pdata->disable_gpio, 1);
				msleep(50);  // 20210806: wait more time for wakeup.
			}
			
			if (pdata->vbus_gpio) {
				gpiod_direction_output(pdata->vbus_gpio, 1);
			}

			pdata->active = 1; // 20210701: already active,set flag.
			LOG("%s: 4g modem power up,set active=1.\n", __func__);
		} else {
			//lte_wakeup(0);

#ifdef CONFIG_LTE_YIYUAN //yiyuan 4g never disconnect
#else
            if (pdata->vbus_gpio) {
				gpiod_direction_output(pdata->vbus_gpio, 0);
			}
#endif
			if (pdata->disable_gpio) {
				gpiod_direction_output(pdata->disable_gpio, 0);
			}
			if (pdata->wakeup_gpio) {
				gpiod_direction_output(pdata->wakeup_gpio, 0);
				// msleep(20);
				pdata->active = false;
			}
			
			if (pdata->power_gpio) {
				#if 0
				// 20210719: 下电的时候先拉低 POWER-KEY 3秒，重新上电的时候还是有 10%概率无法识别
				// 到 USB 设备。
				//gpiod_direction_output(pdata->power_gpio, 0);
				//msleep(100);
				gpiod_direction_output(pdata->power_gpio, 1);
				msleep(3000);
				gpiod_direction_output(pdata->power_gpio, 0);
				msleep(500);
				#else
				gpiod_direction_output(pdata->power_gpio, 0);
				#endif 
			}
			
			if (pdata->reset_gpio) {
				gpiod_direction_output(pdata->reset_gpio, 0);
			}

			if (pdata->vbat_gpio) {
				gpiod_direction_output(pdata->vbat_gpio, 0);
			}

			// 20210407: 模组掉电的时候，我们所有的GPIO都拉低，防止GPIO上面有漏电.
			LOG("%s: 4g modem power down.\n", __func__);
			//pdata->found_dev = false;
		}

		//modem_status = on_off;
		pdata->power_on = pwr_on;
	}

	mutex_unlock(&pdata->ctrl_mutex);
	return 0;
}

static int modem_reset(void)
{
	struct lte_data *pdata = gpdata;
    LOG("modem_reset:cur pwr_on=%d,active=%d\n",pdata->power_on, pdata->active);
    mutex_lock(&pdata->ctrl_mutex);
	if(pdata->active) {
		if (pdata->reset_gpio) {
		    //gpiod_direction_output(pdata->reset_gpio, 0);
			//msleep(10);

			// 20210717: 我们这里拉高，实际出去的信号是低，才起到复位模组的作用。
			gpiod_direction_output(pdata->reset_gpio, 1);
			msleep(200);
			gpiod_direction_output(pdata->reset_gpio, 0);
			//msleep(500);

			//pdata->found_dev = false;
		}
	}

	mutex_unlock(&pdata->ctrl_mutex);
	return 0;
}


static void lte_ctrl_work(struct work_struct *work)
{
	struct lte_data *pdata = gpdata;
	int 	delayx = 0;
    LOG("lte_ctrl:power_on=%d,active=%d,exist=%d,dev_num=%d,fail_cnt=%d,active_when_suspend=%d\n", pdata->power_on, 
        pdata->active, lte_model_exist, pdata->usb_dev_num, 
        pdata->usb_check_fail_cnt, pdata->active_when_suspend);
	lte_ready = 1;

	//20210717: 防止在执行了 prepare 准备进入深度休眠的时候上层通过 wakeup 命令又打开4G。
	if(pdata->active_when_suspend) return;
	
#ifdef DONT_ALLOW_SUSPEND	
	// 20210408: 如果系统休眠之后4G会异常，可以调用这个语句防止系统进入深度休眠.
	// 20210716: 此处先枷锁，防止在 POWER-ON/active 的时候系统进入休眠。--在 lte_active_if_need 里面设置了。
	wake_lock(&pdata->suspend_lock);
#endif 

	if(!pdata->power_on) {
		// 20210717: 在操作之前先设置 operating,防止和 JAVA 命令冲突。
		pdata->operating = true;
	    modem_poweron_off(1);
		delayx = WAIT_FOR_USB_TIMEOUTL; // 这里是第一次上电，LOCK 时间更长一些。
	} else if(!pdata->active) {
		pdata->operating = true;
	    lte_wakeup(1);
	    delayx = WAIT_FOR_USB_TIMEOUTS;  // 其他情况重连大概需要 13秒左右。
	} else if(pdata->usb_dev_num == 0) {
		if(pdata->usb_check_fail_cnt == 1) { // pdata->usb_check_fail_cnt > 0 && 
			pdata->operating = true;
			//modem_reset();

			// 20210807: 如果是复位 ehci-host, 看LOG 500MS 左右就可以重新连上USB了。
			ehci_reset_ehci_host();
			delayx = WAIT_FOR_USB_TIMEOUTL 
				+ pdata->usb_check_fail_cnt*WAIT_FOR_USB_TIMEOUTS; 
		}  else if (pdata->usb_check_fail_cnt == 2){
			// 20210816: 多增加一次 reset 处理。
			pdata->operating = true;
			modem_reset();
			delayx = WAIT_FOR_USB_TIMEOUTL;
		} else {
			return;
		}
	}
#ifndef DONT_ALLOW_SUSPEND	
	wake_unlock(&pdata->suspend_lock); 
	if(delayx) {
		//pdata->found_dev = false;
		cancel_delayed_work(&pdata->check_work);

		// 20210706: 工厂带回来的新的美格4G模组，第一次开机上电信息如下： 
		// [    3.560721] [4g_modem]: modem_poweron_off: 4g modem power up.
		// [   15.330739] usb 2-1: new high-speed USB device number 2 using ehci-platform
		// [   15.458415] usb 2-1: GSM modem (1-port) converter now attached to ttyUSB0
		// 需要 12秒多之后才能识别到USB设备。由于 RILD 跑起来之后会断开USB一次，15秒又刚好
		// 在断开的时候检测。所以需要另外一个机制。--20210715: 老化测试发现 序列号尾号 0013的样机，
		// 
        schedule_delayed_work(&pdata->check_work, delayx);
    }

	// 20210701：每次上电都需要等待，防止系统进入亮屏休眠或者深度休眠。
	// 20210717: 由于上面调用了 unlock,这里即使没有操作，也调用 lock_timeout. ctrl-work 有三个地方
	// 调用：1. 开机初始化，此时没有 lock; 2. 通过 wakeup 接口，此时也没有lock，3. 休眠唤醒，此时会有
	// lock. 不管如何，这里都进行 lock_timeout,主要是防止寻网其间系统进入休眠了。
	wake_lock_timeout(&pdata->suspend_lock, delayx+WAKELOCK_TIMEOUTX);
#endif 		

}

static void lte_check_usb_work(struct work_struct *work)
{
	struct lte_data *pdata = gpdata;
	int usb_dev_num = pdata->usb_dev_num;//option_get_usb_dev_num();
    LOG("check_work: active=%d,usb_dev_num=%d,fail_cnt=%d,active_when_suspend=%d\n", pdata->active,
    	usb_dev_num, pdata->usb_check_fail_cnt, pdata->active_when_suspend);

	if(pdata->active_when_suspend) return;
	
	if(pdata->active && usb_dev_num == 0) {
		// 20210705: 认证固件上面发现开机之后识别不到USB,再次拉 VBUS无效，重新上下电后才有效。
		// 此处改为先拉VBUS,如果还不能识别，则重新上下电。
		#if 0	// 20210807,为了测试 RESET ehci 试试是否有效，此处不进行过多处理。
		LOG("USB-CONNECT failed,check_fail_cnt=%d,re-active!!\n", pdata->usb_check_fail_cnt);
		if(pdata->usb_check_fail_cnt == 0) {
	    	lte_wakeup(0);
	    	schedule_delayed_work(&pdata->ctrl_work, HZ/3 /*msecs_to_jiffies(350)*/);
	    	//modem_reset();  // 20210715: reset 操作不需要ctrl work。
	    } else if(pdata->usb_check_fail_cnt == 1) {
	    	// 20210715: reset 操作不需要ctrl work。--我们需要wake-lock控制，所以统一在 ctrl-work里面处理.
	    	schedule_delayed_work(&pdata->ctrl_work, HZ/10 /*msecs_to_jiffies(350)*/);
	    	//modem_reset();  
	    } else if(pdata->usb_check_fail_cnt == 2) {
	    	modem_poweron_off(0);
	    	schedule_delayed_work(&pdata->ctrl_work, HZ/2 /*msecs_to_jiffies(350)*/);
		} else if(pdata->usb_check_fail_cnt == 3) {
	    	// 20210726: 我们再多做一次复位看看。通过命令行调试，发现下电再上电有时候认不到模组，
	    	// 但是再做一次下电上电就可以了。
	    	schedule_delayed_work(&pdata->ctrl_work, HZ/10 /*msecs_to_jiffies(350)*/);
	    	//modem_reset();  
	    } else {
			// 20210705: keep power on or off??只要有USB 模块存在，我们就保持模块上电状态。
			// 有可能有些模块上电到认到USB需要更长的时间？
			if(!lte_model_exist) {
				modem_poweron_off(0);
			} 
			pdata->operating = false;
		}
		#else
		LOG("USB-CONNECT failed,check_fail_cnt=%d,sche ctrl-work,exist=%d!!\n",
			pdata->usb_check_fail_cnt, lte_model_exist);
		if(pdata->usb_check_fail_cnt < 2) {
			schedule_delayed_work(&pdata->ctrl_work, HZ/10 /*msecs_to_jiffies(350)*/);
		} else {
			// 20210817: 经过多次尝试，还是无法识别USB 设备，则被认为是没有4G模块。
			if(!lte_model_exist){
				modem_poweron_off(0);
				pdata->operating = false;
			}
		}
		#endif 
		// 20210701: 有可能这个设备没有带4G 模块，此处就会出现多次上下电也识别不到USB的情况。我们需要
		// 增加错误次数判断，防止一直进行循环。
		pdata->usb_check_fail_cnt++;
	} else if(usb_dev_num > 0) {
		pdata->usb_check_fail_cnt = 0;
		pdata->operating = false;
	}
}

// 20210719: FT 测试开关4G，到底是上下电还是 只控制 WAKEUP/VBUS GPIO？
// 1： 开关电源， 0：只控制GPIO。 第二种方式更加接近实际使用的情况。但是在休眠测试里面
// 已经覆盖了这个测试模式。
#define LTE_FT_POWER 		1

// echo 0 > /sys/class/rk_modem/modem_status       echo 1 > /sys/class/rk_modem/modem_status
// 复位模组： echo 2 > /sys/class/rk_modem/modem_status
// echo A > /sys/class/rk_modem/modem_status 	echo S > /sys/class/rk_modem/modem_status 
// echo V > /sys/class/rk_modem/modem_status 	echo v > /sys/class/rk_modem/modem_status 
static ssize_t modem_status_write(struct class *cls,
				  struct class_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef CONFIG_LTE_YIYUAN
	LOG("%s: abort(%s) by YIYUAN 4G-LTE\n", __func__, buf);
	return -EINVAL;
#else 
	int new_state, ret;
	struct lte_data *pdata = gpdata;
	// 20210726: 需要增加休眠时候是否断开 4G USB的设置。T2 上面会在设置菜单里面增加用户
	// 选择。 'A': 表示深度休眠不关闭 4G,保持 active 状态. 'S' 表示深度休眠关闭4G（默认状态）。
	if (buf[0] == 'A') {
		LOG("%s: cmd=%s, keep-4G-model active!\n", __func__, buf);
		pdata->keep_active_suspend = true;
		return count;
	} else if (buf[0] == 'S') {
		LOG("%s: cmd=%s, keep-4G-model sleep!\n", __func__, buf);
		pdata->keep_active_suspend = false;
		return count;
	} else if (buf[0] == 'V') {
		LOG("%s: cmd=%s, keep-vbus-high!\n", __func__, buf);
		pdata->keep_vbus_when_suspend = true;
		return count;
	} else if (buf[0] == 'v') {
		LOG("%s: cmd=%s, dont keep-vbus-high!\n", __func__, buf);
		pdata->keep_vbus_when_suspend = false;
		return count;
	}
	
	ret = kstrtoint(buf, 10, &new_state);
	if (ret) {
		LOG("%s: kstrtoint error return %d\n", __func__, ret);
		return ret;
	}

	// 22010701: will check at modem_poweron_off with mutex-lock.
	//if (new_state == pdata->power_on)
	//	return count;
	#if LTE_FT_POWER
	if (new_state == 1) {
		LOG("%s, c(%d), open modem.\n", __func__, new_state);
		modem_poweron_off(1);
		lte_pwr_off_by_ft = false;
	} else if (new_state == 0) {
		LOG("%s, c(%d), close modem.\n", __func__, new_state);
		modem_poweron_off(0);
		lte_pwr_off_by_ft = true;
	} 
	#else 
	if (new_state == 1) {
		LOG("%s, c(%d), wakeup modem.\n", __func__, new_state);
		//modem_poweron_off(1);
		lte_wakeup(1);
		lte_pwr_off_by_ft = false;
	} else if (new_state == 0) {
		LOG("%s, c(%d), sleep modem.\n", __func__, new_state);
		//modem_poweron_off(0);
		lte_wakeup(0);
		lte_pwr_off_by_ft = true;
	} else if (new_state == 11) {
		LOG("%s, c(%d), open modem.\n", __func__, new_state);
		modem_poweron_off(1);
		lte_pwr_off_by_ft = false;
	} else if (new_state == 10) {
		LOG("%s, c(%d), close modem.\n", __func__, new_state);
		modem_poweron_off(0);
		lte_pwr_off_by_ft = true;
	} 
	#endif 
	else if (new_state == 2) {
		 LOG("%s, c(%d), reset modem.\n", __func__, new_state);
		 modem_reset();
	 } else if (new_state == 3) {
		LOG("%s, c(%d), unlock wake-lock\n", __func__, new_state);
		wake_unlock(&pdata->suspend_lock);
		return count;
	} else if (new_state == 4) {
		LOG("%s, c(%d), lock wake-lock.\n", __func__, new_state);
		wake_lock(&pdata->suspend_lock);
		return count;
	} else {
		LOG("%s, invalid parameter.\n", __func__);
	}
	//modem_status = new_state;
	return count;
#endif 	
}

// cat /sys/class/rk_modem/modem_status
static ssize_t modem_status_read(struct class *cls,
				 struct class_attribute *attr,
				 char *_buf)
{
    struct lte_data *pdata = gpdata;
 	return sprintf(_buf, "%s", pdata->keep_active_suspend?"Active":"Sleep");
}

// cat /sys/class/rk_modem/modem_wakeup
static ssize_t modem_wakeup_read(struct class *cls,struct class_attribute *attr, char *_buf)
{
	struct lte_data *pdata = gpdata;
	return sprintf(_buf, "pwr:%d,act:%d,dev_num:%d,exist:%d,err:%d,oper:%d,keep-active:%d,keep-vbus:%d\n", pdata->power_on,
		pdata->active, pdata->usb_dev_num, lte_model_exist, pdata->usb_check_fail_cnt,
		pdata->operating, pdata->keep_active_suspend, pdata->keep_vbus_when_suspend);
}

 // echo 0 > /sys/class/rk_modem/modem_wakeup       echo 1 > /sys/class/rk_modem/modem_wakeup
 // reset: echo 2 > /sys/class/rk_modem/modem_wakeup
 static ssize_t modem_wakeup_write(struct class *cls,struct class_attribute *attr, const char *buf, size_t _count)
 {
	 
#ifdef CONFIG_LTE_YIYUAN
		 LOG("%s: abort(%s) by YIYUAN 4G-LTE\n", __func__, buf);
		 return -EINVAL;
#else 
	 int new_state, ret;
     struct lte_data *pdata = gpdata;
     if(lte_ready == 0)
		return -EBUSY;
	 // 20210701: 当系统进入深度休眠的时候，我们要把4G模块关掉，并且确保 USB 已经断开了再进入
	 // 休眠（否则会影响休眠唤醒后USB的连接）。但是上层 ServiceStateTracker.java 会在休眠退出
	 // 的时候不定时的打开 4G模块的USB.导致内核里面的控制流程进入死循环： kernel suspend loop start
	 // --> lte prepar --> SHUTDOWN 4G model --> return -EBUSY --> exit suspend --> SST enable 4G
	 // --> entern suspend.
	 if(fb_power_off()) {
	 	LOG("%s: abort cmd=%s,active=%d cause fb_power_off!\n", __func__, buf, pdata->active);
	 	return -EINVAL;
	 }

	 if(lte_pwr_off_by_ft) {
	 	LOG("%s: abort cmd=%s, cause pwr_off_by_ft!\n", __func__, buf);
	 	return -EINVAL;
	 }

	 if(!lte_model_exist) {
	 	LOG("%s: abort cmd=%s, cause lte not exist!\n", __func__, buf);
	 	return -EINVAL;
	 }
	 
	 new_state = -1;
	 ret = kstrtoint(buf, 10, &new_state);
	 if (ret) {
		 LOG("%s: kstrtoint error return %d\n", __func__, ret);
		 return ret;
	 }

	 // 22010701: will check at lte_wakeup with mutex-lock.
	 //if (new_state == pdata->active)
	 //	 return _count;
	 if (new_state == 1) {
	     // 20210715: 我们通过 shell 命令来关闭 4G，会看到很快又被打开，应该是 JAVA 上面做的
	     // 操作。把 设备置为飞行模式就OK了。lte_active_if_need 里面 active 的时候设置 usb_check_fail_cnt = 1，
	     // 所以此处不能判断 usb_check_fail_cnt.
		 LOG("%s, new_state=%d, wakeup modem,operating=%d,active=%d\n", __func__, new_state,
		 	pdata->operating, pdata->active);
		 /*if(pdata->usb_check_fail_cnt > 0){
		 	modem_reset();		//20210716: 如果USB检测失败，复位更可靠。
		 } else */
		 if(!pdata->operating) {

		 	// 20210717: 1.为何用 ctrl_work: 主要是可以启动USB-CHECK功能，USB识别失败可以做更多操作；
		 	// 2. 为何要延迟 HZ/4: 通过串口 echo 0 > /sys/class/rk_modem/modem_wakeup 来测试模块下电，
		 	// 在 lte_wakeup(0) 操作还没有完成(只拉低了VBUS，此时USB就断开了)的时候，JAVA上层已经收到
		 	// USB 断开的信息，就会很快调用到此处。ctrl-work 里面打印出来的信息是：
		 	// power_on=1,active=1,model exist=1,dev_num=7 ,导致 没有正确调用 lte_wakeup(1); 操作。我们必须
		 	// 等待 active = 0 之后才能再执行 ctrl-work.
		 	//lte_wakeup(1);
		 	cancel_delayed_work(&pdata->ctrl_work);
			schedule_delayed_work(&pdata->ctrl_work, HZ/3);
		 }
		 //cancel_delayed_work(&pdata->check_work);
		 //dump_stack();
	 } else if (new_state == 0) {
		 LOG("%s, c(%d), sleep modem.\n", __func__, new_state);
		 // 20210717: 防止 check-work 自动又把 4G 模组进行 ACTIVE.
		 cancel_delayed_work(&pdata->ctrl_work);
		 cancel_delayed_work(&pdata->check_work);
		 lte_wakeup(0);
	 } else if (new_state == 2) {
	 	//20210717: 用命令进行reset操作，会发现被reset两次，原因是此处进行reset的时候，会触发JAVA上面
	 	// 发送 wakeup(1)操作，然后在 ctrl-work 里面判断 pwr=1,active=1,从而再次进行reset操作。
	 	// 但是休眠唤醒的操作中应该不会存在这个问题(因为 pdata->operating = true).
		 LOG("%s, c(%d), reset modem.\n", __func__, new_state);
		 modem_reset();
	 } else {
		 LOG("%s, invalid parameter.\n", __func__);
	 }

	 //modem_wakeup = new_state;
	 return _count;
#endif 	 
 }

static CLASS_ATTR(modem_status, 0644, modem_status_read, modem_status_write);
static CLASS_ATTR(modem_wakeup, 0644, modem_wakeup_read, modem_wakeup_write);

static int modem_platdata_parse_dt(struct device *dev,
				   struct lte_data *data)
{
	struct device_node *node = dev->of_node;
	int ret;

	if (!node)
		return -ENODEV;
	//memset(data, 0, sizeof(*data));
	LOG("%s: LTE modem power controlled by gpio.\n", __func__);
	data->vbat_gpio = devm_gpiod_get_optional(dev, "4G,vbat",
						  GPIOD_OUT_LOW);
	if (IS_ERR(data->vbat_gpio)) {
		ret = PTR_ERR(data->vbat_gpio);
		dev_err(dev, "failed to request 4G,vbat GPIO: %d\n", ret);
		return ret;
	}
	data->power_gpio = devm_gpiod_get_optional(dev, "4G,power",
						   GPIOD_OUT_LOW);
	if (IS_ERR(data->power_gpio)) {
		ret = PTR_ERR(data->power_gpio);
		dev_err(dev, "failed to request 4G,power GPIO: %d\n", ret);
		return ret;
	}
	data->reset_gpio = devm_gpiod_get_optional(dev, "4G,reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpio)) {
		ret = PTR_ERR(data->reset_gpio);
		dev_err(dev, "failed to request 4G,reset GPIO: %d\n", ret);
		return ret;
	}

	data->vbus_gpio = devm_gpiod_get_optional(dev, "4G,vbus",
						   GPIOD_OUT_LOW);
	if (IS_ERR(data->vbus_gpio)) {
		ret = PTR_ERR(data->vbus_gpio);
		dev_err(dev, "failed to request 4G,vbus GPIO: %d\n", ret);
		return ret;
	}
	
	data->wakeup_gpio = devm_gpiod_get_optional(dev, "4G,wakeup",
						   GPIOD_OUT_LOW);
	if (IS_ERR(data->wakeup_gpio)) {
		ret = PTR_ERR(data->wakeup_gpio);
		dev_err(dev, "failed to request 4G,wakeup GPIO: %d\n", ret);
		return ret;
	}

	data->disable_gpio = devm_gpiod_get_optional(dev, "4G,disable",
						   GPIOD_OUT_LOW);
	if (IS_ERR(data->disable_gpio)) {
		ret = PTR_ERR(data->disable_gpio);
		dev_err(dev, "failed to request 4G,disable GPIO: %d\n", ret);
		return ret;
	}
	return 0;
}

#if 0
static int modem_power_on_thread(void *data)
{
	modem_poweron_off(1);
	return 0;
}


static int modem_resume_thread(void *data)
{
	msleep(10000);

	modem_poweron_off(1);
	return 0;
}
#endif
static void lte_active_if_need(struct lte_data *pdata) 
{
	pdata->operating = false;
	//  20210716: 如果上层已经通过接口关闭了 4G 模块的电源，此处不能再打开。
	// 20210717: 在系统进入休眠已经调用了prepare的时候马上进行唤醒，这个时候有可能USB
	// 还没有完全断开(usb_dev_num > 0),但是我们还是需要进行 active 操作，否则 4G就不在网了。
	// (此时的上层可能操作会被 operating 或者 fb_power_off 丢弃)。
	// 为什么需要判断 usb_dev_num？就是当 4G经过 ACTIVE/RESET/POWER-OFF-ON 还无法连接上的时候，
	// 休眠唤醒时再做一次尝试。 20210813: power_on 判断其实就是判断 4G模式是否存在。
	// 20210817: ctrl 里面我们在没有识别到4G-USB的时候，并没有把power拉低（防止有些模块很长时间
	// 才能认到USB）。此处判断 power-on 不好.
	if(lte_model_exist // pdata->power_on 
		&& (pdata->active_when_suspend || !pdata->usb_dev_num)) {
		
		// 20210716: 防止马上进入休眠，导致USB 识别出现异常。
		wake_lock(&pdata->suspend_lock);
		
		// 20210716: 次数设置是1，防止识别不到USB的时候再做一次VBUS操作，无效。做reset才有效。
		pdata->usb_check_fail_cnt = 0;

		if(!pdata->usb_dev_num) {
		    pdata->active = false; //make sure lte_ctrl_work dosomething.
		}

		// 20210716: 有时候上次一active的check_work没有来得及调用就进入深度休眠了。由于我们下面还会
		// 打开 模块，此处可以把原来的 check_work 删除掉。 --没有必要，lte_pm_prepare 已经做了处理。
		// cancel_delayed_work(&pdata->check_work);
		
		// 22010701: 如果此处太早给 4G 模块进行active，会导致USB 无法识别。
		// --20210716: 从休眠老化的结果看，延迟2秒 4G-MODEL USB 识别的概率会大大提高。 1秒
		// 还是很容易经过 active/reset/power-off 之后USB也无法识别的情况。
		cancel_delayed_work(&pdata->ctrl_work);
		schedule_delayed_work(&pdata->ctrl_work, 1.8*HZ /*msecs_to_jiffies(350)*/);
	}

	pdata->active_when_suspend = false;
}
static int lte_pm_prepare(struct device *dev)
{
	struct lte_data *pdata = gpdata;
    LOG("lte_pm_prepare: fb-off=%d,4G active=%d,active_when_suspend=%d,dev_num=%d,sus-keep-act=%d\n",
    	fb_power_off(), 
    	pdata->active, pdata->active_when_suspend, pdata->usb_dev_num,
    	pdata->keep_active_suspend);
    if(fb_power_off()){
    	if(!pdata->keep_active_suspend) {
    	    // 20210701：如果休眠的时候 4G 模块是 ACTIVE(USE 正在连接)，则 需要先断开USB连接，
    		// 然后系统才能进入休眠。 此处预留3秒钟。但是也有可能由于检测不到 SIM,4G 模块已经断开
    		// 了，这种情况下不需要处理。 --20210708: 如果识别不到USB设备，则不需要做处理。
    		if(pdata->active && pdata->usb_dev_num) {
    			pdata->active_when_suspend = true;
    			// 20210717: 增加标志防止此时上层又下发 active 命令。
    			pdata->operating = true;

    			cancel_delayed_work(&pdata->ctrl_work);
    			cancel_delayed_work(&pdata->check_work);
    			lte_wakeup(0);

    			// 等待USB断开之后才能进入休眠。 大概200MS USB 就会断开。
    			wake_lock_timeout(&pdata->suspend_lock, 1*HZ);  
    			return -EBUSY;
    		} 
    	} else {
    	    if(pdata->active && pdata->usb_dev_num) {
    	        pdata->active_when_suspend = true;
    	        LOG("lte_pm_prepare: fb-off=%d,set active_when_suspend=%d,dev_num=%d\n",
                	fb_power_off(), pdata->active_when_suspend, pdata->usb_dev_num);
    	    }
    	}
	}
	return 0;
}

static void lte_pm_complete(struct device *dev)
{
	struct lte_data *pdata = gpdata;
    LOG("lte_pm_complete: fb-off=%d, active_when_suspend=%d,pwr_on=%d,dev_num=%d,sus-keep-act=%d\n", 
    	fb_power_off(), pdata->active_when_suspend, pdata->power_on, 
    	pdata->usb_dev_num, pdata->keep_active_suspend);

    // 20210726: 此处可以不增加 if(fb_power_off() && !pdata->keep_active_suspend) 的判断。
    lte_active_if_need(pdata);
}


// 20210701: 增加FB-NOTIFY 来确保亮屏的时候 4G 模块被重新上电。原因：系统进入灭屏待机 -- suspend loop -- 
// lte_prepare -- 断开USB,返回 -EBUSY.如果此时用户按了 POWER 按键，则系统不会进入休眠，就无法调用
// lte_complete,从而导致4G 模块没有连接上。我们在亮屏的时候需要做判断，是否 设置了 active_when_suspend。
static int lte_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct lte_data *pdata = gpdata;
	int blank;
	struct fb_event *evdata = data;

	// 系统会发送两个 EVENT: 先 FB_EARLY_EVENT_BLANK ， 后 FB_EVENT_BLANK。
	if (event != FB_EVENT_BLANK /*FB_EARLY_EVENT_BLANK FB_EVENT_BLANK*/ || !evdata) {
		// 20210701: drop event=9, 4G active=1,act_suspend=0
		//printk("lte_fb_notifier: drop event=%ld, 4G active=%d,act_suspend=%d\n", 
		//	event, pdata->active, pdata->active_when_suspend);
		goto exit;
	}

	blank = *((int*)evdata->data);

    LOG("lte_fb_notifier: 4G active=%d,act_suspend=%d,blank=%d\n", 
			pdata->active, pdata->active_when_suspend, blank);
			
	// 20210701: 
	// 黑屏： event=16, blank=4,4G active=1,act_suspend=0
	// 亮屏： event=16, blank=0,4G active=1,act_suspend=0
	// 统一改为用 EVENT 9:
	// 黑屏： lte_fb_notifier: event=9, blank=4,4G active=1,act_suspend=0
	// 亮屏： lte_fb_notifier: event=9, blank=0,4G active=1,act_suspend=0

	if(pdata->active_when_suspend &&
		/* FB_BLANK_NORMAL == blank || */ FB_BLANK_UNBLANK == blank) {
		LOG("lte_fb_notifier: lte_active_if_need,4G active=%d,act_suspend=%d\n", 
			pdata->active, pdata->active_when_suspend);
    	lte_active_if_need(pdata);
	} 
exit:
	return 0;
}

static void lte_setup_fb_notifier(struct lte_data *pdata)
{
	int rc;

	pdata->fb_notify.notifier_call = lte_fb_notifier_callback;

	rc = fb_register_client(&pdata->fb_notify);
	if (rc)
		dev_err(pdata->dev, "Unable to register fb_notifier: %d\n", rc);
}

static int lte_probe(struct platform_device *pdev)
{
	struct lte_data *pdata;
	//struct task_struct *kthread;
	int ret = -1;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	ret = modem_platdata_parse_dt(&pdev->dev, pdata);
	if (ret < 0) {
		LOG("%s: No lte platform data specified\n", __func__);
		goto err;
	}
	gpdata = pdata;
	pdata->dev = &pdev->dev;

#if 0
	//if (pdata->reset_gpio) gpiod_direction_output(pdata->reset_gpio, 0);
	kthread = kthread_run(modem_power_on_thread, NULL,
			      "modem_power_on_thread");
	if (IS_ERR(kthread)) {
		LOG("%s: create modem_power_on_thread failed.\n",  __func__);
		ret = PTR_ERR(kthread);
		goto err;
	}
#else
	mutex_init(&pdata->ctrl_mutex);
    INIT_DELAYED_WORK(&pdata->ctrl_work, lte_ctrl_work);

    INIT_DELAYED_WORK(&pdata->check_work, lte_check_usb_work);
    lte_setup_fb_notifier(pdata);
    
    wake_lock_init(&pdata->suspend_lock, WAKE_LOCK_SUSPEND, "lte");

#ifdef CONFIG_LTE_YIYUAN
	pdata->keep_active_suspend = 1;
    pdata->keep_vbus_when_suspend = 1;
    ehci_wakeup_ctrl(true);  // always keep wakeup for yiyuan.
#else 
    pdata->keep_active_suspend = KEEP_ACTIVE_WHEN_SUSPEN;
    pdata->keep_vbus_when_suspend = KEEP_VBUSON_WHEN_SUSPEN;
#endif 
	// 20210701: 目前不知道为什么，开机 6秒左右连上了USB，但是到 16秒又会断开，19秒左右重连。
	// 所以此处 延迟上电。我们启动时间一般需要 28秒。 -- 开机总是要断开连接一次，难道是 RILD 里面
	// 控制的？
    schedule_delayed_work(&pdata->ctrl_work, 1*HZ /* msecs_to_jiffies(100)*/);
#endif 

	return 0;
err:
	gpdata = NULL;
	return ret;
}

#if 0
static int lte_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct lte_data *pdata = gpdata;
    printk("lte_suspend: fb-off=%d,4G active=%d\n", fb_power_off(), pdata->active);
#ifdef ONLY_DEEP_SLEEP
	if(fb_power_off())
#endif
	{
		// 20210701：如果休眠的时候 4G 模块是 ACTIVE(USE 正在连接)，则 需要先断开USB连接，
		// 然后系统才能进入休眠。 此处预留3秒钟。但是也有可能由于检测不到 SIM,4G 模块已经断开
		// 了，这种情况下不需要处理。
		if(pdata->active) {
			cancel_delayed_work(&pdata->ctrl_work);
			lte_wakeup(0);
			pdata->active_when_suspend = true;
			wake_lock_timeout(&pdata->suspend_lock, 3*HZ);
			return -EBUSY;
		} 
	}
	return 0;
}

static int lte_resume(struct platform_device *pdev)
{
    struct lte_data *pdata = gpdata;
    printk("lte_resume: fb-off=%d,4G active=%d\n", fb_power_off(), pdata->active);
	//printk("4g lte_resume\n");
	//lte_wakeup(1);
#ifdef ONLY_DEEP_SLEEP	
	if(fb_power_off())
#endif
	{
		if(pdata->active_when_suspend) {
			pdata->active_when_suspend = false;
			schedule_delayed_work(&pdata->ctrl_work, msecs_to_jiffies(250));
		}
	}else{
	}
	return 0;
}
#else 

#if 0
static int lte_pm_suspend(struct device *dev)
{
    struct lte_data *pdata = gpdata;
    printk("lte_pm_suspend: fb-off=%d,4G active=%d,act_sus=%d\n", fb_power_off(), 
    	pdata->active, pdata->active_when_suspend);
#ifdef ONLY_DEEP_SLEEP
	if(fb_power_off())
#endif
	{
		// 20210701：如果休眠的时候 4G 模块是 ACTIVE(USE 正在连接)，则 需要先断开USB连接，
		// 然后系统才能进入休眠。 此处预留3秒钟。但是也有可能由于检测不到 SIM,4G 模块已经断开
		// 了，这种情况下不需要处理。
		if(pdata->active) {
			cancel_delayed_work(&pdata->ctrl_work);
			lte_wakeup(0);
			pdata->active_when_suspend = true;
			wake_lock_timeout(&pdata->suspend_lock, 3*HZ);
			return -EBUSY;
		} 
	}
	return 0;
}

static int lte_pm_resume(struct device *dev)
{
    struct lte_data *pdata = gpdata;
    printk("lte_pm_resume: fb-off=%d,4G active=%d,act_sus=%d\n", fb_power_off(), 
    	pdata->active, pdata->active_when_suspend);
	//printk("4g lte_resume\n");
	//lte_wakeup(1);
#ifdef ONLY_DEEP_SLEEP	
	//if(fb_power_off())
#endif
	{
		if(pdata->active_when_suspend) {
			pdata->active_when_suspend = false;
			schedule_delayed_work(&pdata->ctrl_work, msecs_to_jiffies(250));
		}
	}
	return 0;
}
#endif 
#endif 
static int lte_remove(struct platform_device *pdev)
{
	struct lte_data *pdata = gpdata;
    printk("lte_remove \n");
	if (pdata->power_gpio) {
		msleep(100);
		gpiod_direction_output(pdata->power_gpio, 1);
		msleep(750);
		gpiod_direction_output(pdata->power_gpio, 0);
	}
	if (pdata->vbat_gpio)
		gpiod_direction_output(pdata->vbat_gpio, 0);
	gpdata = NULL;
	return 0;
}

static void lte_shutdown(struct platform_device *pdev)
{
	struct lte_data *pdata = gpdata;

	// 20210727: disable ctrl-work.
	pdata->active_when_suspend = true;
	
	printk("lte_shutdown: pwr=%d,act=%d,opering=%d\n", pdata->power_on,
		pdata->active, pdata->operating);

	pdata->operating = true;  // disable /sys interface.
	cancel_delayed_work(&pdata->ctrl_work);
	cancel_delayed_work(&pdata->check_work);

	// needs power off??
	modem_poweron_off(0);
}


static const struct dev_pm_ops lte_pm_ops = {
	// 20210701: for shutdown USB before suspend.
	.prepare = lte_pm_prepare,
	.complete = lte_pm_complete,
	//.suspend	= lte_pm_suspend,
	//.resume		= lte_pm_resume,
};


static const struct of_device_id modem_platdata_of_match[] = {
	{ .compatible = "4g-modem-platdata" },
	{ }
};
MODULE_DEVICE_TABLE(of, modem_platdata_of_match);

static struct platform_driver rm310_driver = {
	.probe		= lte_probe,
	.remove		= lte_remove,
	//.suspend	= lte_suspend,
	//.resume		= lte_resume,
	.shutdown = lte_shutdown,
	.driver	= {
		.name	= "4g-modem-platdata",
		.pm = &lte_pm_ops,
		.of_match_table = of_match_ptr(modem_platdata_of_match),
	},
};

static int __init rm310_init(void)
{
	int ret;

	modem_class = class_create(THIS_MODULE, "rk_modem");
	ret =  class_create_file(modem_class, &class_attr_modem_status);
	if (ret)
		LOG("Fail to create class modem_status.\n");
	ret =  class_create_file(modem_class, &class_attr_modem_wakeup);
	if (ret)
		LOG("Fail to create class modem_status.\n");
	return platform_driver_register(&rm310_driver);
}

static void __exit rm310_exit(void)
{
	platform_driver_unregister(&rm310_driver);
}

late_initcall(rm310_init);
module_exit(rm310_exit);

MODULE_AUTHOR("xuxuehui <xxh@rock-chips.com>");
MODULE_AUTHOR("Alex Zhao <zzc@rock-chips.com>");
MODULE_DESCRIPTION("RM310 lte modem driver");
MODULE_LICENSE("GPL");
