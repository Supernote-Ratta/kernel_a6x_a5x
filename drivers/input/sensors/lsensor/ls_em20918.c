/* drivers/input/sensors/access/kxtik.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>

#include <linux/sensor-dev.h>

/*
* this driver can use for EM30918
*/
static int debug = 5;
module_param(debug, int, S_IRUGO | S_IWUSR);

#define dprintk(level, fmt, arg...) do {   \
 if (debug > level) printk("ls_em20918: " fmt , ## arg); } while (0)

#define EM20918_DEBUG(format, ...) dprintk(0, format, ## __VA_ARGS__)

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
#define REG_CONFIG              (0x01)
#define CONFIG_PS_EN_BIT        (1<<7)
#define CONFIG_PS_SLP_800MS     (1<<6)
#define CONFIG_PS_DR_BIT        (7<<3)
#define CONFIG_PS_DR_200MA      (7<<3)
#define CONFIG_PS_DR_100MA      (6<<3)
#define CONFIG_PS_DR_50MA       (5<<3)
#define CONFIG_PS_DR_25MA       (4<<3)
#define CONFIG_PS_DR_120MA      (3<<3)
#define CONFIG_PS_DR_60MA       (2<<3)
#define CONFIG_PS_DR_30MA       (1<<3)
#define CONFIG_PS_DR_15MA       (0<<3)
#define CONFIG_ALS_EN_BIT       (1<<2)
#define CONFIG_ALS_RANGE_BIT    (1<<1)
#define CONFIG_ALS_MODE_BIT     (1<<0)
//als
#define CONFIG_ALS_MASK         (CONFIG_ALS_EN_BIT | CONFIG_ALS_RANGE_BIT | CONFIG_ALS_MODE_BIT)
#define CONFIG_ALS_ON           (CONFIG_ALS_EN_BIT | CONFIG_ALS_RANGE_BIT)
//ps
#define CONFIG_PS_MASK          (CONFIG_PS_EN_BIT | CONFIG_PS_DR_BIT)
#define CONFIG_PS_ON            (CONFIG_PS_EN_BIT | CONFIG_PS_DR_200MA)

//ps data reg
#define REG_PS_DATA             (0x08)
// alsa data (12bit)
// low 8 bit (7:0)
#define REG_ALS_DATA_L          (0x09)
// high 4 bit (3:0)
#define REG_ALS_DATA_H          (0x0A)

#define ALS_DATA_PRECISION      (0xFFF)


struct em20918_data {
    struct i2c_client *client;
    int sensor_value;
    int report_index;

    struct class *cls_node;

    struct class_attribute s_sensor_value;
    struct class_attribute s_report_index;
    struct class_attribute s_ctrl_data;
};

static struct em20918_data *g_em20918_data = NULL;
extern bool fb_power_off(void);


/****************operate according to sensor chip:start************/

static int ls_em20918_active(struct i2c_client *client, int enable, int rate)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);

    sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
    sensor->ops->ctrl_data &= (~(CONFIG_ALS_MASK));

    if (enable) {
        sensor->ops->ctrl_data |= (CONFIG_ALS_ON & CONFIG_ALS_MASK);;
    }

    return sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
}

/*
in hardware/rockchip/sensor/st/LightSensor.cpp
 static const float luxValues[8] = {
    10.0, 160.0, 225.0, 320.0,
    640.0, 1280.0, 2600.0, 10240.0
};

*/
//跟上层流明值对应
#define LIGHT_SENSOR_LUX_SIZE (8)
static unsigned int light_array[LIGHT_SENSOR_LUX_SIZE + 1] = {
    0,
    100,     180,    250,   400,
    1000,   2000,   4000,  0xffffffff
};

static int light_report_value(struct input_dev *input, int data)
{
    unsigned char index = 0;
    int i ;
    for (i = 0; i < ARRAY_SIZE(light_array) - 1; i++) {
        if (data >= light_array[i] && data < light_array[i + 1]) {
            index = i;
            break;
        }
    }

    input_report_abs(input, ABS_MISC, index);
    input_sync(input);

    return index;
}

static int ls_em20918_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    struct em20918_data *ls_data = g_em20918_data;
    int result = 0;
    u8 als_value_low[1], als_value_high[1];

    int ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
    if (!(ctrl_data & CONFIG_ALS_ON) && sensor->ops->active) {
        sensor->ops->active(client, true, 0);
        msleep(120);
    }

    als_value_low[0] = sensor_read_reg(client, REG_ALS_DATA_L);
    als_value_high[0] = sensor_read_reg(client, REG_ALS_DATA_H);

    ls_data->sensor_value = als_value_low[0] | ((als_value_high[0]) << 8);

    ls_data->report_index = light_report_value(sensor->input_dev, ls_data->sensor_value);

    EM20918_DEBUG("%s:%s sensor_value = %d, report index = %d\n",
        __func__, sensor->ops->name, ls_data->sensor_value, ls_data->report_index);

    return result;
}

static ssize_t show_sensor_value(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct em20918_data *ls_data = container_of(attr, struct em20918_data, s_sensor_value);
	struct i2c_client *client = ls_data->client;
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    ssize_t len = 0;
	u8 als_value_low[1], als_value_high[1];
	
		//int ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
		//if (!(ctrl_data & CONFIG_ALS_ON) && sensor->ops->active) {
		//	sensor->ops->active(client, true, 0);
		//	msleep(120);
		//}
	
		als_value_low[0] = sensor_read_reg(client, REG_ALS_DATA_L);
		als_value_high[0] = sensor_read_reg(client, REG_ALS_DATA_H);
	
		ls_data->sensor_value = als_value_low[0] | ((als_value_high[0]) << 8);
	
		ls_data->report_index = light_report_value(sensor->input_dev, ls_data->sensor_value);

    len += snprintf(_buf + len, PAGE_SIZE - len, "%d\n", ls_data->sensor_value);

    return len;
}

static ssize_t store_report_index(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    return _count;
}

static ssize_t show_report_index(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct em20918_data *ls_data = container_of(attr, struct em20918_data, s_report_index);
    ssize_t len = 0;

    len += snprintf(_buf + len, PAGE_SIZE - len, "%d\n", ls_data->report_index);

    return len;
}

static ssize_t store_sensor_value(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    return _count;
}

static ssize_t show_ctrl_data(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct em20918_data *ls_data = container_of(attr, struct em20918_data, s_ctrl_data);
    struct i2c_client *client = ls_data->client;
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    ssize_t len = 0;

    len += snprintf(_buf + len, PAGE_SIZE - len, "0x%x\n", sensor_read_reg(client, sensor->ops->ctrl_reg));

    return len;
}

static ssize_t store_ctrl_data(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct em20918_data *ls_data = container_of(attr, struct em20918_data, s_ctrl_data);
    struct i2c_client *client = ls_data->client;
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    int value;

    sscanf(buf, "%x", &value);
    sensor_write_reg(client, sensor->ops->ctrl_reg, value);

    return _count;
}


#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define CLASS_CREATE_FILE(data,_name_) do{ \
        data->s_##_name_.attr.mode = 0666;\
        data->s_##_name_.attr.name = STR(_name_);\
        data->s_##_name_.show = show_##_name_;\
        data->s_##_name_.store = store_##_name_;\
        if (class_create_file(data->cls_node, &data->s_##_name_)) {\
            printk("%s: Fail to creat class file %s\n", __func__, data->s_##_name_.attr.name);\
        }\
    } while(0)

#define CLASS_REMOVE_FILE(data,_name_)       do{ \
        class_remove_file(data->cls_node, &data->s_##_name_);\
    } while(0)


static int ls_em20918_init(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);

    struct em20918_data *ls_data;
    int err = 0;
    EM20918_DEBUG("-----------sensor_init----ls_em20918\n");

    ls_data = kzalloc(sizeof(struct em20918_data), GFP_KERNEL);
    if (!ls_data) {
       dev_err(&client->dev, "%s: failed to allocate em20918_data\n", __func__);
       return -1;
    }
    ls_data->client = client;
	if (!IS_ERR_OR_NULL(sensor->supply)) {
		if(!sensor->is_poweron){
			if(!regulator_is_enabled(sensor->supply)) {
				err = regulator_enable(sensor->supply);
			}
			sensor->is_poweron = true;
		}
	}

    err = sensor->ops->active(client, false, 0);
    if(err) {
        printk("%s:line=%d, sensor active error\n", __func__, __LINE__);
        goto error_free;
    }

    sensor->status_cur = SENSOR_OFF;

    ls_data->cls_node = class_create(THIS_MODULE, "htfyun-ls");
    if ( IS_ERR_OR_NULL(ls_data->cls_node)) {
        dev_err(&client->dev, "failed to class_create ls_em20918 err=%d\n", err);
    } else {
        CLASS_CREATE_FILE(ls_data, sensor_value);
        CLASS_CREATE_FILE(ls_data, report_index);
        CLASS_CREATE_FILE(ls_data, ctrl_data);
    }

    g_em20918_data = ls_data;

    return 0;
error_free:
    if (ls_data) {
        kfree(ls_data);
        ls_data = NULL;
    }
    return err;

}
#ifdef CONFIG_PM
static int ls_em20918_suspend(struct i2c_client *client)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);
	int ret = 0;
	printk("ls_em20918_suspend \n");
	if (!IS_ERR_OR_NULL(sensor->supply)) {
		printk("ls_em20918_suspend 111\n");
		if(fb_power_off()&&(sensor->is_poweron)){
			ret = regulator_disable(sensor->supply);
			if (ret < 0) {
				dev_err(&sensor->client->dev, "%s:failed to enable supply: %d\n", __func__, ret);
			}
			sensor->is_poweron = false;
		}
	}
	return ret;
}

static int ls_em20918_resume(struct i2c_client *client)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);
	int ret = 0;
	if (!IS_ERR_OR_NULL(sensor->supply)) {
		if(!sensor->is_poweron){
			if(!regulator_is_enabled(sensor->supply)) {
				ret = regulator_enable(sensor->supply);
			}
			sensor->is_poweron = true;
		}
	}
	return ret;
}

#endif

struct sensor_operate light_em20918_ops = {
    .name               = "ls_em20918",
    .type               = SENSOR_TYPE_LIGHT,    //sensor type and it should be correct
    .id_i2c             = LIGHT_ID_EM20918, //i2c id number
    .read_reg           = REG_ALS_DATA_L, //read data
    .read_len           = 2,            //data length
    .id_reg             = SENSOR_UNKNOW_DATA,   //read device id from this register
    .id_data            = SENSOR_UNKNOW_DATA,   //device id
    .precision          = 16,           //8 bits
    .ctrl_reg           = REG_CONFIG,      //enable or disable
    .int_status_reg     = SENSOR_UNKNOW_DATA,      //intterupt status register
    .range              = {100, 4095},      //range
    .brightness         = {10, 255},     //brightness
    .trig               = IRQF_TRIGGER_NONE,
    .active             = ls_em20918_active,
    .init               = ls_em20918_init,
    .report             = ls_em20918_report_value,
    .suspend 			= ls_em20918_suspend,
    .resume				= ls_em20918_resume,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *light_get_ops(void)
{
    return &light_em20918_ops;
}


static int __init light_em20918_init(void)
{
    return sensor_register_slave(SENSOR_TYPE_LIGHT, NULL, NULL, light_get_ops);
}

static void __exit light_em20918_exit(void)
{
    struct em20918_data *ls_data = g_em20918_data;
    sensor_unregister_slave(SENSOR_TYPE_LIGHT, NULL, NULL, light_get_ops);
    if (ls_data) {
        if (ls_data->cls_node) {
            CLASS_REMOVE_FILE(ls_data, sensor_value);
            CLASS_REMOVE_FILE(ls_data, report_index);
            CLASS_REMOVE_FILE(ls_data, ctrl_data);
            class_destroy(ls_data->cls_node);
            ls_data->cls_node = NULL;
        }
        kfree(ls_data);
        ls_data = NULL;
        g_em20918_data = NULL;
    }
}



module_init(light_em20918_init);
module_exit(light_em20918_exit);


