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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>

#include "epd_temperature.h"
#include "../../ebc.h"

#define EPD_TEMP_DEBUG (0)

#if EPD_TEMP_DEBUG
#define temp_printk(fmt, args...)  printk(KERN_INFO "[ebc] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define temp_printk(fmt, args...)
#endif


#define LM75_I2C_NAME		  "epd_lm75"
#define LM75_I2C_ADDR		  0x48


struct epd_lm75_sensor {
	struct	i2c_client	 *client;
};

static struct epd_lm75_sensor g_lm75_data;



int epd_temp_sensor_shutdown(void)
{
	struct i2c_client* client = g_lm75_data.client;   
	struct i2c_msg msg[1];
	int ret;
	u8 buf[2];
	
	temp_printk("\n ");
	if(!g_lm75_data.client){ 
		return ;
	}
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	msg->scl_rate = 400*1000;
	buf[0] = 0x01;
	buf[1] = 0x01;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != 1) {
		printk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return;
	}
	
	return;
}
int epd_temp_sensor_wakeup(void)
{
	struct i2c_client* client = g_lm75_data.client;   
	struct i2c_msg msg[1];	
	int ret;
	u8 buf[2];
	
	temp_printk("\n ");
	if(!g_lm75_data.client){ 
		return -1;
	}
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	msg->scl_rate = 400*1000;
	buf[0] = 0x01;
	buf[1] = 0x00;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != 1) {
		printk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return ret;
	}
	
	return 0;	
}
int epd_get_temperature(int *temperature)
{
	struct i2c_msg msgs[2];
	int temp = 27;
	s32 retries = 0;
	char buf[2] = {0,27};
	int ret;
	temp = buf[1];
	if(!g_lm75_data.client) 
		return temp;
	buf[0] = 0;
	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = g_lm75_data.client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];
	msgs[0].scl_rate = 400 * 1000;
	
	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = g_lm75_data.client->addr;
	msgs[1].len   = 1;
	msgs[1].buf   = &buf[1];
	msgs[1].scl_rate = 400 * 1000;
	
	while(retries < 5)
	{
		ret = i2c_transfer(g_lm75_data.client->adapter, msgs, 2);
		if (ret == 2)
		{
			temp = buf[1];
			break;
		}
		else
			printk("epd get temperature error!!!\n");
		retries++;
	}
	temp_printk("temperature = %d\n",temp);
	*temperature = temp;
	return 0;
}

static int epd_lm75_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	temp_printk("I2C addr:%x", client->addr);
	
	g_lm75_data.client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk("I2C check functionality failed.");
		return -ENODEV;
	}

	return 0;
}

static int epd_lm75_remove(struct i2c_client *client)
{
	g_lm75_data.client = NULL;
	return 0;
}


static const struct i2c_device_id epd_lm75_id[] = {
	{ LM75_I2C_NAME, 0 },
	{ }
};
static struct i2c_driver epd_lm75_driver = {
	.probe	= epd_lm75_probe,
	.remove 	= epd_lm75_remove,
	.id_table	= epd_lm75_id,
	.driver = {
		.name	  = LM75_I2C_NAME,
		.owner	  = THIS_MODULE,
	},
};


static int __init epd_lm75_sensor_init(void)
{
	return i2c_add_driver(&epd_lm75_driver);
}

static void __exit epd_lm75_sensor_exit(void)
{
	return i2c_del_driver(&epd_lm75_driver);
}

module_init(epd_lm75_sensor_init);
module_exit(epd_lm75_sensor_exit);

MODULE_DESCRIPTION("LM75 sensor objects for PowerMacs thermal control");
MODULE_LICENSE("GPL");


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
	u32 value;
	epd_get_temperature(&value);
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

int register_ebc_temp_ops(struct ebc_temperateure_ops *ops)
{
	ops->temperature_get = epd_get_temperature;
	return 0;
}