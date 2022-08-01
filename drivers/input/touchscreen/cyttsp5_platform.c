/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/cyttsp5_platform.h>
#include <linux/regulator/driver.h>  //20180510,hsl add.

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
	int dl = 300+cd->i2c_err_cnt*10;
	printk("cyttsp5_xres,reset delay=%dms,error=%d\n", dl, cd->i2c_err_cnt);
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(80+cd->i2c_err_cnt*5);  // 40.
	gpio_set_value(rst_gpio, 1);

	// 20210806: 如果I2C 通信出现问题，会调用 reset,即进入这个函数。此处加大 reset
	// 后的延迟，否则可能会有概率性的 I2C 出错。
	msleep(dl);
	dev_info(dev,
		"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		pdata->rst_gpio, rc);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int power_gpio = pdata->power_gpio;
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
		if(power_gpio != INVALID_GPIO){
			printk("cyttsp5_init power up:%d\n",power_gpio);
			gpio_set_value(power_gpio, 1);
			msleep(5);
			rc = gpio_request(power_gpio, "cytt_pwr");
			if (rc < 0) {
				gpio_free(power_gpio);
				rc = gpio_request(power_gpio, "cytt_pwr");
			}
			if (rc < 0) {
				dev_err(dev,
					"%s: Fail request gpio=%d\n", __func__,
					power_gpio);
			} else {
				rc = gpio_direction_output(power_gpio, 1);
				if (rc < 0) {
					pr_err("%s: Fail set output gpio=%d\n",
						__func__, power_gpio);
					gpio_free(power_gpio);
				}
			}
	    }
		rc = gpio_request(rst_gpio, "cytt_rst");
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, "cytt_rst");
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			printk("cyttsp5_init rst_gpio:%d 1",rst_gpio);
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, "cytt_irq");
				if (rc < 0) {
					gpio_free(irq_gpio);
					rc = gpio_request(irq_gpio, "cytt_irq");
				}
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}
	} else {
		if(power_gpio != INVALID_GPIO) gpio_free(power_gpio);
		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
	}

	dev_info(dev, "%s: INIT CYTTSP PWR gpio=%d RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, power_gpio, rst_gpio, irq_gpio, rc);
	return rc;
}

const char * regulator_tp_name = "vcc_tp";
bool        regulator_boot_on = true; //20180523,regulator already disable when boot,
                    // regulator_is_enabled return true,and we got unbalanced disables for vcc_tp
static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
    int rst_gpio = pdata->rst_gpio;
	int power_gpio = pdata->power_gpio;
    struct regulator *rgl = regulator_get(dev, regulator_tp_name);
    if( IS_ERR(rgl) ){
        dev_err(dev, "failed to get %s regulator for power on!\n", regulator_tp_name);
    } else {
        if( true/* !regulator_is_enabled(rgl)*/) {
            int ret = regulator_enable(rgl);
            if( ret ){
                dev_err(dev, "enable %s regulator failed,ret=%d!\n", regulator_tp_name, ret);
            }
		    regulator_put(rgl);
		    //gpio_set_value(rst_gpio, 1);
		    msleep(5);
		}
    }
    printk("%s:enable regulator %s,set rst_gpio=1!\n", __func__, regulator_tp_name);
    //dump_stack();
    if(power_gpio != INVALID_GPIO){
		gpio_set_value(power_gpio, 1);
		msleep(5);
    }
	gpio_set_value(rst_gpio, 0);
	msleep(80);
	gpio_set_value(rst_gpio, 1);

	// 20210719: 机器容易出现 I2C错误导致触摸无消息上报，改大一点试试. 200ms
	// 还是有 cyttsp5_read_input: Error getting report 错误。改更大一点试试。
	msleep(300);  // delay 200ms:ok. 100ms: rk3x_i2c_stop with Error: -6. 150ms: OK.
	gpio_direction_input(pdata->irq_gpio);
	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
    int rst_gpio = pdata->rst_gpio;
	int power_gpio = pdata->power_gpio;

    struct regulator *rgl = regulator_get(dev, regulator_tp_name);
    if( IS_ERR(rgl) ){
        dev_err(dev, "failed to get %s regulator for power off!\n", regulator_tp_name);
    } else {
        //20180523: unbalanced disables for vcc_tp
        if(!regulator_boot_on && regulator_is_enabled(rgl)) {
            regulator_disable(rgl);
		    regulator_put(rgl);
		}
		regulator_boot_on = false;
    }
    printk("%s:disable regulator %s,set rst_gpio=0!\n", __func__, regulator_tp_name );
	if(power_gpio != INVALID_GPIO){
		gpio_set_value(power_gpio, 0);
	}
	gpio_set_value(rst_gpio, 0);
	gpio_direction_output(pdata->irq_gpio, 0);
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		parade_debug(dev, DEBUG_LEVEL_2, "%s: Performing a reset\n",
			__func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		parade_debug(dev, DEBUG_LEVEL_2, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif
