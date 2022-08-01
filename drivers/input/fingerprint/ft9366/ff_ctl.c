/**
 * The device control driver for FocalTech's fingerprint sensor.
 *
 * Copyright (C) 2016-2017 FocalTech Systems Co., Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/signal.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/bug.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/stddef.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#if !defined(CONFIG_MTK_CLKMGR)
# include <linux/clk.h>
#else
# include <mach/mt_clkmgr.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_fingerprint_info;
static int read_id = 9362;

#endif

#include "ff_log.h"
#include "ff_err.h"
#include "ff_ctl.h"
#include "ff_spi.h"

/*
 * Define the driver version string.
 */
#define FF_DRV_VERSION "v2.1.2"

/*
 * Define the driver name.
 */
#define FF_DRV_NAME "focaltech_fp"

/*
 * Driver context definition and its singleton instance.
 */

typedef struct {
    struct miscdevice miscdev;
    int irq_num;
    struct work_struct work_queue;
    struct fasync_struct *async_queue;
    struct input_dev *input;
    struct notifier_block fb_notifier;
#ifdef CONFIG_PM_WAKELOCKS
    struct wakeup_source wake_lock;
#else 
    struct wake_lock wake_lock;
#endif
    bool b_driver_inited;
    bool b_config_dirtied;
	u32 cs_gpio;
	u32 reset_gpio;
	u32 irq_gpio;
} ff_ctl_context_t;
static ff_ctl_context_t *g_context = NULL;

/*
 * Driver configuration.
 */
static ff_driver_config_t driver_config;
ff_driver_config_t *g_config = NULL;
static int init_flag=0;
static char hal_version[64] = {0};
////////////////////////////////////////////////////////////////////////////////
/// Logging driver to logcat through uevent mechanism.

# undef LOG_TAG
#define LOG_TAG "ff_ctl"




///////////////////////////////////////////////////////////////////////////////
/**
 * plat-rk.c
 *
**/

#define CONFIG_RK_PLATFORM "rockchip px30"

int ff_ctl_enable_power(bool on);

/* TODO: */
#define FF_COMPATIBLE_NODE_1 "focaltech,fingerprint-spidev"
//"focaltech,focaltech_fp"
//#define FF_COMPATIBLE_NODE_2 "mediatek,mt6765-fpc"
//#define FF_COMPATIBLE_NODE_1 "mediatek,focal-fp"
//#define FF_COMPATIBLE_NODE_2 "mediatek,fpc1145"
#define FF_COMPATIBLE_NODE_3 "rockchip,px30-spi"
//"mediatek,mt6765-spi"

/* Define pinctrl state types. */
#if 0
typedef enum {
    FF_PINCTRL_STATE_SPI_CS_ACT,
    FF_PINCTRL_STATE_SPI_CK_ACT,
    FF_PINCTRL_STATE_SPI_MOSI_ACT,
    FF_PINCTRL_STATE_SPI_MISO_ACT,
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_CS_SET,
    FF_PINCTRL_STATE_CLK_SET,
    FF_PINCTRL_STATE_MI_SET,
    FF_PINCTRL_STATE_MO_SET,
    FF_PINCTRL_STATE_MI_ACT,
    FF_PINCTRL_STATE_MI_CLR,
    FF_PINCTRL_STATE_MO_ACT,
    FF_PINCTRL_STATE_MO_CLR,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

#else
typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;
#endif
/* Define pinctrl state names. */
#if 0
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "csb_spi", "clk_spi", "mosi_spi", "miso_spi",
    "power_on", "power_off", "reset_low", "reset_high", "irq_gpio",
};

static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpc_pins_pwr_high", "fpc_pins_pwr_low", "fpc_pins_rst_low", "fpc_pins_rst_high",
    "fpc_eint_as_int", "fpc_mode_as_cs", "fpc_mode_as_ck", "fpc_mode_as_mi",
    "fpc_mode_as_mo", "fpc_miso_pull_up", "fpc_miso_pull_down",
    "fpc_mosi_pull_up", "fpc_mosi_pull_down",
};

static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpc_pins_pwr_high", "fpc_pins_pwr_low", "fpc_pins_rst_low", "fpc_pins_rst_high",
    "fpc_eint_as_int", 
};
#else
//static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
//    "fpsensor_finger_power_high","fpsensor_finger_power_low","fpsensor_finger_rst_low","fpsensor_finger_rst_high","fpsensor_eint_as_int"
//};
#endif

/* Native context and its singleton instance. */
typedef struct {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
#if !defined(CONFIG_MTK_CLKMGR)
    struct clk *spiclk;
#endif
    bool b_spiclk_enabled;
} ff_mt6762_context_t;
//static ff_mt6762_context_t ff_mt6762_context;//, *g_context = &ff_mt6762_context;
/*GPIO pins reference.*/
int ff_get_gpio_dts_info(void)
{
	int rc = 0;
	struct device_node *dev_node = NULL;
	    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_1);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        printk("zg502 of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        return (-ENODEV);
    }
      printk("dev_node :%s",dev_node->name);
    /*get pwr resource*/
	g_context->cs_gpio = of_get_named_gpio(dev_node, "fingerprint,en-gpio", 0);
	if (!gpio_is_valid(g_context->cs_gpio)) {
		FF_LOGE("%s, PWR GPIO is invalid.\n", __func__);
		//return -1;
	}
	FF_LOGE("%s, gf:goodix_pwr:%d\n", __func__, g_context->cs_gpio);
	if (gpio_is_valid(g_context->cs_gpio)) {
		rc = gpio_request(g_context->cs_gpio, "fingerprint_pwr");
		if (rc) {
			FF_LOGE("%s, Failed to request PWR GPIO. rc = %d\n", __func__, rc);
			//return -1;
		}
	}
    /*get reset resource*/
	g_context->reset_gpio = of_get_named_gpio(dev_node, "fingerprint,gpio_reset", 0);
	if (!gpio_is_valid(g_context->reset_gpio)) {
		FF_LOGE("%s, RESET GPIO is invalid.\n", __func__);
		//return -1;
	}
	if (gpio_is_valid(g_context->reset_gpio)) {
		rc = gpio_request(g_context->reset_gpio, "fingerprint_reset");
		if (rc) {
			FF_LOGE("%s, Failed to request RESET GPIO. rc = %d\n", __func__, rc);
			//return -1;
		}
		gpio_direction_output(g_context->reset_gpio, 1);
	}
    /*get irq resourece*/
	g_context->irq_gpio = of_get_named_gpio(dev_node, "fingerprint,touch-int-gpio", 0);
	FF_LOGE("%s, gf:irq_gpio:%d\n", __func__, g_context->irq_gpio);
	if (!gpio_is_valid(g_context->irq_gpio)) {
		FF_LOGE("%s, IRQ GPIO is invalid.\n", __func__);
		return -1;
	}

	rc = gpio_request(g_context->irq_gpio, "fingerprint_irq");
	if (rc) {
		FF_LOGE("%s, Failed to request IRQ GPIO. rc = %d\n", __func__, rc);
		return -1;
	}
	gpio_direction_input(g_context->irq_gpio);

	return 0;
}

int ff_ctl_init_pins(ff_ctl_context_t *g_context)
{
    int err = 0;
	//int irq_num1 = 0;
    struct device_node *dev_node = NULL;
    struct platform_device *pdev = NULL;

    printk("'%s' zg502 enter.", __func__);
#if 0
    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_1);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        printk("zg502 of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        return (-ENODEV);
    }
      printk("dev_node :%s",dev_node->name);
	irq_num1 = irq_of_parse_and_map(dev_node, 0);
	*irq_num = irq_num1;
    printk("pzp zg502  irq number is %d.", irq_num1);
    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGE("of_find_device_by_node(..) failed.");
        printk("zg502 of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        FF_LOGE("devm_pinctrl_get(..) failed.");
        printk("zg502 devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

        printk("zg502 register pins.");
    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            FF_LOGE("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            printk("zg502 can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }

    /* init spi,sunch as cs clck miso mosi mode, gpio pullup pulldown */
	/*
        for (i = FF_PINCTRL_STATE_INT_ACT + 1; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
            err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[i]);

            if (err) {
                printk("%s() pinctrl_select_state(%s) failed.\n", __FUNCTION__, g_pinctrl_state_names[i]);
                break;
            }

            printk("pinctrl_select_state(%s) ok.\n", g_pinctrl_state_names[i]);
        }
        */
    /* Initialize the INT pin. */
        printk("zg502 init int pin.");
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT_ACT]);

    /* Retrieve the irq number. 
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_2);
    if (!dev_node) {
        printk("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_2);
        return (-ENODEV);
    }
    *irq_num = irq_of_parse_and_map(dev_node, 0);
    printk("irq number is %d.", *irq_num);*/

    pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
#endif	
#if 1
//#if !defined(CONFIG_MTK_CLKMGR)
    //
    // Retrieve the clock source of the SPI controller.
    //

    /* 3-1: Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_3);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_3);
        printk("zg502 of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_3);
        return (-ENODEV);
    }

        FF_LOGE("of_find_device_by_node(node:%s) .",dev_node->name);
    /* 3-2: Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGE("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    } else {
        //u32 frequency, div;
        //err = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &frequency);
        //err = of_property_read_u32(pdev->dev.of_node, "clock-div", &div);
        FF_LOGE("spi controller(#%d) name: %s.", pdev->id, pdev->name);
        printk("zg502 spi controller(#%d) name: %s.", pdev->id, pdev->name);
        //FF_LOGD("spi controller(#%d) clk : %dHz.", pdev->id, frequency / div);
    }
#if 0
    /* 3-3: Retrieve the SPI clk handler. */
    g_context->spiclk = devm_clk_get(&pdev->dev, "spi-clk");
    if (!g_context->spiclk) {
        FF_LOGE("devm_clk_get(..) failed.");
        return (-ENODEV);
    }
    #endif
#endif


	ff_get_gpio_dts_info();
    ff_ctl_enable_power(true);

    printk("'%s' zg502  leave.", __func__);
    FF_LOGE("'%s' zg502  leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    printk("'%s' enter.", __func__);
#if 0
    // TODO:
	if (g_context->pinctrl) {
        pinctrl_put(g_context->pinctrl);
        g_context->pinctrl = NULL;
    }
    printk("'%s' leave.", __func__);
	#endif
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter. clock: '%s'. \n", __func__,on ? "enable" : "disabled");
#if 0
    if (unlikely(!g_context->spiclk)) {
        return (-ENOSYS);
    }
	printk("focal '%s' b_spiclk_enabled = %d. \n", __func__, g_context->b_spiclk_enabled);

    /* Control the clock source. */
    if (on && !g_context->b_spiclk_enabled) {
        printk("clk_prepare_enable");
        err = clk_prepare_enable(g_context->spiclk);
        if (err) {
            FF_LOGE("clk_prepare_enable(..) = %d.", err);
        }
        g_context->b_spiclk_enabled = true;
    } else if (!on && g_context->b_spiclk_enabled) {
        printk("clk_disable_unprepare");
		clk_disable_unprepare(g_context->spiclk);
        g_context->b_spiclk_enabled = false;
    }
#endif
    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
	static int enable = 1;
    FF_LOGD("'%s' enter. power: '%s'.", __func__, on ? "on" : "off");
	
	if (g_context == NULL) {
		FF_LOGE("%s, Input buff is NULL.\n", __func__);
		return (-ENOSYS);;
	}

	if (on && enable)
		enable = 0;
	else if (!on && !enable)
		enable = 1;
	if (gpio_is_valid(g_context->cs_gpio)) {
		if(on){
			gpio_direction_output(g_context->cs_gpio, 1);
			gpio_set_value(g_context->cs_gpio, 1);
		}
		else{
			gpio_direction_output(g_context->cs_gpio, 1);
			gpio_set_value(g_context->cs_gpio, 0);
		}
	}

    FF_LOGD("'%s' leave.", __func__);
    return 0;
}


int ff_ctl_reset_device(void)
{
    //int err = 0;
    printk("'%s' enter.", __func__);
	if (g_context == NULL) {
		FF_LOGE("%s, Input buff is NULL.\n", __func__);
		return (-ENOSYS);
	}
	if (gpio_is_valid(g_context->reset_gpio)) {
		gpio_direction_output(g_context->reset_gpio, 1);
		gpio_set_value(g_context->reset_gpio, 0);
		mdelay(1);
		gpio_set_value(g_context->reset_gpio, 0);
		mdelay(50);
		gpio_set_value(g_context->reset_gpio, 1);
	}else
		mdelay(10);
    printk("'%s' leave.", __func__);
    return 0;
}

const char *ff_ctl_arch_str(void)
{
	return (CONFIG_RK_PLATFORM);
}

//
/*
 * Log level can be runtime configurable by 'FF_IOC_SYNC_CONFIG'.
 */
ff_log_level_t g_log_level = __FF_EARLY_LOG_LEVEL;

int ff_log_printf(ff_log_level_t level, const char *tag, const char *fmt, ...)
{
    /* Prepare the storage. */
    va_list ap;
    static char uevent_env_buf[128];
    char *ptr = uevent_env_buf;
    int n, available = sizeof(uevent_env_buf);

    /* Fill logging level. */
    available -= sprintf(uevent_env_buf, "FF_LOG=%1d", level);
    ptr += strlen(uevent_env_buf);

    /* Fill logging message. */
    va_start(ap, fmt);
    vsnprintf(ptr, available, fmt, ap);
    va_end(ap);

    /* Send to ff_device. */
    if (likely(g_context) && likely(g_config) && unlikely(g_config->logcat_driver)) {
        char *uevent_env[2] = {uevent_env_buf, NULL};
        kobject_uevent_env(&g_context->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);
    }

    /* Native output. */
    switch (level) {
    case FF_LOG_LEVEL_ERR:
        n = printk(KERN_ERR FF_DRV_NAME": %s\n", ptr);
        break;
    case FF_LOG_LEVEL_WRN:
        n = printk(KERN_WARNING FF_DRV_NAME": %s\n", ptr);
        break;
    case FF_LOG_LEVEL_INF:
        n = printk(KERN_INFO FF_DRV_NAME": %s\n", ptr);
        break;
    case FF_LOG_LEVEL_DBG:
    case FF_LOG_LEVEL_VBS:
    default:
        n = printk(KERN_ERR FF_DRV_NAME": %s\n", ptr);
        break;
    }
    return n;
}

const char *ff_err_strerror(int err)
{
    static char errstr[32] = {'\0', };

    switch (err) {
    case FF_SUCCESS     : return "Success";
    case FF_ERR_INTERNAL: return "Internal error";

    /* Base on unix errno. */
    case FF_ERR_NOENT: return "No such file or directory";
    case FF_ERR_INTR : return "Interrupted";
    case FF_ERR_IO   : return "I/O error";
    case FF_ERR_AGAIN: return "Try again";
    case FF_ERR_NOMEM: return "Out of memory";
    case FF_ERR_BUSY : return "Resource busy / Timeout";

    /* Common error. */
    case FF_ERR_BAD_PARAMS  : return "Bad parameter(s)";
    case FF_ERR_NULL_PTR    : return "Null pointer";
    case FF_ERR_BUF_OVERFLOW: return "Buffer overflow";
    case FF_ERR_BAD_PROTOCOL: return "Bad protocol";
    case FF_ERR_SENSOR_SIZE : return "Wrong sensor dimension";
    case FF_ERR_NULL_DEVICE : return "Device not found";
    case FF_ERR_DEAD_DEVICE : return "Device is dead";
    case FF_ERR_REACH_LIMIT : return "Up to the limit";
    case FF_ERR_REE_TEMPLATE: return "Template store in REE";
    case FF_ERR_NOT_TRUSTED : return "Untrusted enrollment";

    default:
        sprintf(errstr, "%d", err);
        break;
    }

    return (const char *)errstr;
}

////////////////////////////////////////////////////////////////////////////////

/* See plat-xxxx.c for platform dependent implementation. */
//extern int ff_ctl_init_pins(struct ff_ctl_context_t *g_context);
int ff_ctl_free_pins(void);
int ff_ctl_enable_spiclk(bool on);
int ff_ctl_enable_power(bool on);
int ff_ctl_reset_device(void);
const char *ff_ctl_arch_str(void);

/* See chip-xxxx.c for chip dependent implementation. */
extern int ff_chip_init(void);

static int ff_ctl_enable_irq(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("irq: '%s'.", on ? "enable" : "disabled");

    if (unlikely(!g_context)) {
        return (-ENOSYS);
    }

    if (on) {
        enable_irq(g_context->irq_num);
    } else {
        disable_irq(g_context->irq_num);
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static void ff_ctl_device_event(struct work_struct *ws)
{
    ff_ctl_context_t *ctx = container_of(ws, ff_ctl_context_t, work_queue);
    char *uevent_env[2] = {"FF_INTERRUPT", NULL};
    FF_LOGV("'%s' enter.", __func__); 
    FF_LOGD("%s(irq = %d, ..) toggled.", __func__, ctx->irq_num);
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(&g_context->wake_lock, jiffies_to_msecs(2*HZ));
#else
    wake_lock_timeout(&g_context->wake_lock, 2 * HZ); // 2 seconds.
#endif
    kobject_uevent_env(&ctx->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);

    FF_LOGV("'%s' leave.", __func__);
}

static irqreturn_t ff_ctl_device_irq(int irq, void *dev_id)
{
    ff_ctl_context_t *ctx = (ff_ctl_context_t *)dev_id;
	FF_LOGD("==============ff_ctl_device_irq\n");
    disable_irq_nosync(irq);

    if (likely(irq == ctx->irq_num)) {
        if (g_config && g_config->enable_fasync && g_context->async_queue) {
            kill_fasync(&g_context->async_queue, SIGIO, POLL_IN);
        } else {
            schedule_work(&ctx->work_queue);
        }
    }

    enable_irq(irq);
    return IRQ_HANDLED;
}

static int ff_ctl_report_key_event(struct input_dev *input, ff_key_event_t *kevent)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    input_report_key(input, kevent->code, kevent->value);
    input_sync(input);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static const char *ff_ctl_get_version(void)
{
    static char version[FF_DRV_VERSION_LEN] = {'\0', };
    FF_LOGV("'%s' enter.", __func__);

    /* Version info. */
    version[0] = '\0';
    strcat(version, FF_DRV_VERSION);
#ifdef __FF_SVN_REV
    if (strlen(__FF_SVN_REV) > 0) {
        sprintf(version, "%s-r%s", version, __FF_SVN_REV);
    }
#endif
#ifdef __FF_BUILD_DATE
    strcat(version, "-"__FF_BUILD_DATE);
#endif
    sprintf(version, "%s-%s", version, ff_ctl_arch_str());
    FF_LOGD("version: '%s'.", version);

    FF_LOGV("'%s' leave.", __func__);
    return (const char *)version;
}

static int ff_ctl_fb_notifier_callback(struct notifier_block *nb, unsigned long action, void *data)
{
    struct fb_event *event;
    int blank;
    char *uevent_env[2];

	/* If we aren't interested in this event, skip it immediately ... */
	if (action != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
		return NOTIFY_DONE;
	}

	event = (struct fb_event *)data;
	blank = *(int *)event->data;
	FF_LOGD("'%s' enter. action:%ld blank:%d", __func__,action,blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		uevent_env[0] = "FF_SCREEN_ON";
		if (gpio_is_valid(g_context->cs_gpio)) {
			gpio_direction_output(g_context->cs_gpio, 1);
			gpio_set_value(g_context->cs_gpio, 1);
		}
		break;
	case FB_BLANK_POWERDOWN:
        uevent_env[0] = "FF_SCREEN_OFF";
		if (gpio_is_valid(g_context->cs_gpio)) {
			gpio_direction_output(g_context->cs_gpio, 1);
			gpio_set_value(g_context->cs_gpio, 0);
		}
		break;
	default:
	    uevent_env[0] = "FF_SCREEN_??";
		break;
	}
	uevent_env[1] = NULL;
	kobject_uevent_env(&g_context->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);

	FF_LOGV("'%s' leave.", __func__);
	return NOTIFY_OK;
}

static int ff_ctl_register_input(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    /* Allocate the input device. */
    g_context->input = input_allocate_device();
    if (!g_context->input) {
        FF_LOGE("input_allocate_device() failed.");
        return (-ENOMEM);
    }

    /* Register the key event capabilities. */
    //if (g_config) {
        input_set_capability(g_context->input, EV_KEY, KEY_LEFT);//g_config->keycode_nav_left    );
        input_set_capability(g_context->input, EV_KEY, KEY_RIGHT);//g_config->keycode_nav_right   );
        input_set_capability(g_context->input, EV_KEY, KEY_UP);//g_config->keycode_nav_up      );
        input_set_capability(g_context->input, EV_KEY, KEY_DOWN);//g_config->keycode_nav_down    );
        //input_set_capability(g_context->input, EV_KEY, g_config->keycode_double_click);
        input_set_capability(g_context->input, EV_KEY, KEY_F1);//g_config->keycode_click       );
        input_set_capability(g_context->input, EV_KEY, KEY_F2);//g_config->keycode_long_press  );
        //input_set_capability(g_context->input, EV_KEY, g_config->keycode_simulation  );
        input_set_capability(g_context->input, EV_KEY, 558); // Added by xuanfeng.ye for task7065065 on 2018-10-25
    //}

    /* Register the allocated input device. */
    g_context->input->name = "ff_key";
    err = input_register_device(g_context->input);
    if (err) {
        FF_LOGE("input_register_device(..) = %d.", err);
        input_free_device(g_context->input);
        g_context->input = NULL;
        return (-ENODEV);
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static int ff_ctl_free_driver(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    /* Unregister framebuffer event notifier. */
    err = fb_unregister_client(&g_context->fb_notifier);

    /* Disable SPI clock. */
    err = ff_ctl_enable_spiclk(0);

    /* Disable the fingerprint module's power. */
    //err = ff_ctl_enable_power(0);

    /* Unregister the spidev device. */
    //err = ff_spi_free();

    /* De-initialize the input subsystem. */
    if (g_context->input) {
        /*
         * Once input device was registered use input_unregister_device() and
         * memory will be freed once last reference to the device is dropped.
         */
        input_unregister_device(g_context->input);
        g_context->input = NULL;
    }

    /* Release IRQ resource. */
    if (g_context->irq_num > 0) {
        err = disable_irq_wake(g_context->irq_num);
        if (err) {
            FF_LOGE("disable_irq_wake(%d) = %d.", g_context->irq_num, err);
        }
        free_irq(g_context->irq_num, (void*)g_context);
        g_context->irq_num = -1;
    }

    /* Release pins resource. */
    err = ff_ctl_free_pins();

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static int ff_ctl_init_driver(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    printk("'%s' zg502 enter.", __func__);

    if (unlikely(!g_context)) {
        return (-ENOSYS);
    }

    do {
        /* Initialize the PWR/SPI/RST/INT pins resource. */
    	printk("'%s' zg502 ff_ctl_init_pins enter.", __func__);

        err = ff_ctl_init_pins(g_context);
        if (err > 0) {
            g_context->b_config_dirtied = true;
        } else 
 
        if (err) {
            FF_LOGE("ff_ctl_init_pins(..) = %d.", err);
            break;
        }

        /* Register IRQ. */
		g_context->irq_num = gpio_to_irq(g_context->irq_gpio);

        err = request_irq(g_context->irq_num, ff_ctl_device_irq,
                IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ff_irq", (void*)g_context);
        if (err) {
            FF_LOGE("request_irq(..) = %d.", err);
            break;
        }

        /* Wake up the system while receiving the interrupt. */
        err = enable_irq_wake(g_context->irq_num);
        if (err) {
            FF_LOGE("enable_irq_wake(%d) = %d.", g_context->irq_num, err);
        }

        /* Register spidev device. For REE-Emulation solution only. */
//        if (g_config && g_config->enable_spidev) {
        if(init_flag){
            err = ff_spi_init();
            if (err) {
                FF_LOGE("ff_spi_init(..) = %d.", err);
                break;
            }
            init_flag=0;
        }
    } while (0);

    if (err) {
        ff_ctl_free_driver();
        return err;
    }

    /* Initialize the input subsystem. */
    err = ff_ctl_register_input();
    if (err) {
        FF_LOGE("ff_ctl_init_input() = %d.", err);
        //return err;
    }

    printk("'%s'  52 enable power.", __func__);
    /* Enable the fingerprint module's power at system startup. */
    err = ff_ctl_enable_power(1);

    /* Enable SPI clock. */
    //Begin Modified by yuduan.xie for Task 7108178 on 2018.11.12
    err = ff_ctl_enable_spiclk(1);
    //End Modified by yuduan.xie for Task 7108178 on 2018.11.12

    /* Register screen on/off callback. */
    g_context->fb_notifier.notifier_call = ff_ctl_fb_notifier_callback;
    err = fb_register_client(&g_context->fb_notifier);

    g_context->b_driver_inited = true;
    FF_LOGV("'%s' leave.", __func__);
    printk("'%s'  52 leave.", __func__);
    return err;
}
#if 1
static int ff_dev_cleanup(void) {
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    /* Release the HW resources if needs. */
    if (g_context->b_driver_inited) {
        err = ff_ctl_free_driver();
        g_context->b_driver_inited = false;
    }

    /* De-init the wake lock. */
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_trash(&g_context->wake_lock);
#else
    wake_lock_destroy(&g_context->wake_lock);
#endif

    /* Unregister the miscellaneous device. */
    misc_deregister(&g_context->miscdev);

    /* 'g_context' could not use any more. */
    g_context = NULL;

    FF_LOGI("FocalTech fingerprint device control driver released.");
    FF_LOGV("'%s' leave.", __func__);
	return err;
}
#endif
////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static int ff_ctl_fasync(int fd, struct file *filp, int mode)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    FF_LOGD("%s: mode = 0x%08x.", __func__, mode);
    err = fasync_helper(fd, filp, mode, &g_context->async_queue);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static long ff_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    struct miscdevice *dev = (struct miscdevice *)filp->private_data;
    ff_ctl_context_t *ctx = container_of(dev, ff_ctl_context_t, miscdev);
    FF_LOGV("'%s' enter.", __func__);

#if 1
    if (g_log_level <= FF_LOG_LEVEL_DBG) {
        static const char *cmd_names[] = {
                "FF_IOC_INIT_DRIVER", "FF_IOC_FREE_DRIVER",
                "FF_IOC_RESET_DEVICE",
                "FF_IOC_ENABLE_IRQ", "FF_IOC_DISABLE_IRQ",
                "FF_IOC_ENABLE_SPI_CLK", "FF_IOC_DISABLE_SPI_CLK",
                "FF_IOC_ENABLE_POWER", "FF_IOC_DISABLE_POWER",
                "FF_IOC_REPORT_KEY_EVENT", "FF_IOC_SYNC_CONFIG",
                "FF_IOC_GET_VERSION", "unknown",
        };
        unsigned int _cmd = _IOC_NR(cmd);
        if (_cmd > _IOC_NR(FF_IOC_GET_VERSION)) {
            _cmd = _IOC_NR(FF_IOC_GET_VERSION) + 1;
        }
        FF_LOGD("%s(.., %s, ..) invoke.", __func__, cmd_names[_cmd]);
    }
#endif

    switch (cmd) {
    case FF_IOC_INIT_DRIVER: {
        if (!g_context->b_driver_inited) {
            err = ff_ctl_init_driver();
        }
        break;
    }
    case FF_IOC_FREE_DRIVER:
        if (g_context->b_driver_inited) {
            err = ff_ctl_free_driver();
            g_context->b_driver_inited = false;
        }
        break;
    case FF_IOC_RESET_DEVICE:
        err = ff_ctl_reset_device();
        break;
    case FF_IOC_ENABLE_IRQ:
        err = ff_ctl_enable_irq(1);
        break;
    case FF_IOC_DISABLE_IRQ:
        err = ff_ctl_enable_irq(0);
        break;
    case FF_IOC_ENABLE_SPI_CLK:
        err = ff_ctl_enable_spiclk(1);
        break;
    case FF_IOC_DISABLE_SPI_CLK:
        err = ff_ctl_enable_spiclk(0);
        break;
    case FF_IOC_ENABLE_POWER:
        err = ff_ctl_enable_power(1);
        break;
    case FF_IOC_DISABLE_POWER:
        err = ff_ctl_enable_power(0);
        break;
    case FF_IOC_REPORT_KEY_EVENT: {
        ff_key_event_t kevent;
        if (copy_from_user(&kevent, (ff_key_event_t *)arg, sizeof(ff_key_event_t))) {
            FF_LOGE("copy_from_user(..) failed.");
            err = (-EFAULT);
            break;
        }
        err = ff_ctl_report_key_event(ctx->input, &kevent);
        break;
    }
    case FF_IOC_SYNC_CONFIG: {
        if (copy_from_user(&driver_config, (ff_driver_config_t *)arg, sizeof(ff_driver_config_t))) {
            FF_LOGE("copy_from_user(..) failed.");
            err = (-EFAULT);
            break;
        }
        g_config = &driver_config;

        /* Take logging level effect. */
        g_log_level = g_config->log_level;
        break;
    }
    case FF_IOC_GET_VERSION: {
        if (copy_to_user((void *)arg, ff_ctl_get_version(), FF_DRV_VERSION_LEN)) {
            FF_LOGE("copy_to_user(..) failed.");
            err = (-EFAULT);
            break;
        }
        break;
	}
	case FF_IOC_WRITE_HAL_VERSION: {
        if (copy_from_user(hal_version, (void *)arg, FF_HAL_VERSION_LEN)) {
            FF_LOGE("copy_to_user(..) failed.");
            err = (-EFAULT);
            break;
        }
		#if defined(CONFIG_PRIZE_HARDWARE_INFO)
		sprintf(current_fingerprint_info.id, "%s",&hal_version[0]);
		#endif
        break;
	}
	case FF_IOC_GET_HAL_VERSION: {
        if (copy_to_user((void *)arg, hal_version, FF_HAL_VERSION_LEN)) {
            FF_LOGE("copy_to_user(..) failed.");
            err = (-EFAULT);
            break;
        }
        break;
    }
    default:
        err = (-EINVAL);
        break;
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static int ff_ctl_open(struct inode *inode, struct file *filp)
{
    FF_LOGD("'%s' nothing to do.", __func__);
    return 0;
}

static int ff_ctl_release(struct inode *inode, struct file *filp)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    /* Remove this filp from the asynchronously notified filp's. */
    err = ff_ctl_fasync(-1, filp, 0);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

#ifdef CONFIG_COMPAT
static long ff_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    FF_LOGV("focal '%s' enter.\n", __func__);

    err = ff_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));

    FF_LOGV("'%s' leave.", __func__);
    return err;
}
#endif
///////////////////////////////////////////////////////////////////////////////

static struct file_operations ff_ctl_fops = {
    .owner          = THIS_MODULE,
    .fasync         = ff_ctl_fasync,
    .unlocked_ioctl = ff_ctl_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = ff_ctl_compat_ioctl,
#endif
    .open           = ff_ctl_open,
    .release        = ff_ctl_release,
};

static ff_ctl_context_t ff_ctl_context = {
    .miscdev = {
        .minor = MISC_DYNAMIC_MINOR,
        .name  = FF_DRV_NAME,
        .fops  = &ff_ctl_fops,
    }, 0,
};

static int __init ff_ctl_driver_init(void)
{
    int err = 0;
    FF_LOGV("'===========%s' enter.", __func__);
    printk(" ==========zg502 enter.");
	
    /* Register as a miscellaneous device. */
    err = misc_register(&ff_ctl_context.miscdev);
    if (err) {
        FF_LOGE("misc_register(..) = %d.", err);
        return err;
    }

    printk(" zg502 init work.");
    /* Init the interrupt workqueue. */
    INIT_WORK(&ff_ctl_context.work_queue, ff_ctl_device_event);

        /* Init the wake lock. */
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&ff_ctl_context.wake_lock, "ff_wake_lock");
#else
    wake_lock_init(&ff_ctl_context.wake_lock, WAKE_LOCK_SUSPEND, "ff_wake_lock");
#endif

    /* Assign the context instance. */
    g_context = &ff_ctl_context;
    init_flag=1;

#if 1
    /* Initialize the chip. */
    if (ff_ctl_init_driver() == 0) {
	    FF_LOGI("Focaltech init driver success.");
	    if (ff_chip_init() == 0x9362) {
	        FF_LOGI("FocalTech get chip id success.");
			FF_LOGI("FocalTech fingerprint free gpio.");
            //err = ff_ctl_free_driver();
            //g_context->b_driver_inited = false;
		} else {
		    FF_LOGI("FocalTech get chip id failed.");
			ff_dev_cleanup();
			return -1;
		}
    } else {
	    FF_LOGI("Focaltech init driver failed.");
		ff_dev_cleanup();
		return -1;
	}
#endif
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	//id=mas_connect();
	sprintf(current_fingerprint_info.chip,"ft9362");
	sprintf(current_fingerprint_info.id,"0x%x",read_id);
	strcpy(current_fingerprint_info.vendor,"focaltech");
	strcpy(current_fingerprint_info.more,"fingerprint");
#endif
    FF_LOGI("FocalTech fingerprint device control driver registered.");
    FF_LOGV("'%s' leave.", __func__);
    return err;
}

static void __exit ff_ctl_driver_exit(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    /* Release the HW resources if needs. */
    if (g_context->b_driver_inited) {
        err = ff_ctl_free_driver();
        g_context->b_driver_inited = false;
    }

    /* De-init the wake lock. */
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_trash(&g_context->wake_lock);
#else
    wake_lock_destroy(&g_context->wake_lock);
#endif

    /* Unregister the miscellaneous device. */
    misc_deregister(&g_context->miscdev);

    /* 'g_context' could not use any more. */
    g_context = NULL;

    FF_LOGI("FocalTech fingerprint device control driver released.");
    FF_LOGV("'%s' leave.", __func__);
}

module_init(ff_ctl_driver_init);
module_exit(ff_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for FocalTech's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("FocalTech Fingerprint R&D department");

