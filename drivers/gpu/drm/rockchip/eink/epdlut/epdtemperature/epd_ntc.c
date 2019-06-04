/*
 * RK29 ebook temperature  epd_temperature.c
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: dlx@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*include*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "epd_temperature.h"
#include <linux/adc.h>
#include "../../ebc.h"
/*define.*/
#if  EPD_TEMP_DEBUG
#define temp_printk(fmt, args...)  printk(KERN_INFO "ebc " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define temp_printk(fmt, args...)
#endif


int  epd_ad_value[50] = {3292,3127,2972,2825,2686,2555,2431,2314,2203,2098,
						1999,1904,1815,1731,1651,1575,1503,1435,1371,1309,
						1251,1195,1143,1093,1045,1000,957,916,877,840,
						805,771,739,709,680,652,625,600,576,553,
						531,510,490,471,453,435,419,403,387,373};
struct adc_client 	*epd_adc_client; 
#define  EPD_ADC_NUM 	1

int epd_temp_sensor_shutdown(void)
{
	return 0;
}

int epd_temp_sensor_wakeup(void)
{
	return 0;
}
int epd_get_temperature(int *temperature)
{
	int temp = 27;
	int rc;
	int ad_result;
	int r_value;
	int i;
	if(epd_adc_client == NULL)
		epd_adc_client = adc_register(EPD_ADC_NUM, NULL, NULL);
	ad_result = adc_sync_read(epd_adc_client);
	r_value = (ad_result*1000)/(1024-ad_result);
	for(i = 50; i >0; i--){
		if(r_value < epd_ad_value[i])
			break;
	}
	temp = i + 1;
	temp_printk("ntc temperature = %d\n",temp);
	*temperature = temp;
	return 0;
}

int register_ebc_temp_ops(struct ebc_temperateure_ops *ops)
{
	ops->temperature_get = epd_get_temperature;
	return 0;
}

