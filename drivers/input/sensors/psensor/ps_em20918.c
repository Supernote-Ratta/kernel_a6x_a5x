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
 if (debug > level) printk("ps_em20918: " fmt , ## arg); } while (0)

#define EM20918_DEBUG(format, ...) dprintk(0, format, ## __VA_ARGS__)

/*
vendor operation
//rk reserved id, 0~15, in include/linux/soc/rockchip/rk_vendor_storage.h
#define RSV_ID      0
#define SN_ID       1
#define WIFI_MAC_ID 2
#define LAN_MAC_ID  3
#define BT_MAC_ID   4
#define SENSOR_CALIBRATION_ID 7

//ours defined from 64
#define BSN_ID           64
#define IMEI_ID          65
#define UID_ID           66
#define USERDATA_ID      67
#define ALL_INFO_ID      68

*/
extern bool is_rk_vendor_ready(void);
extern int rk_vendor_read(u32 id, void *pbuf, u32 size);
extern int rk_vendor_write(u32 id, void *pbuf, u32 size);

#define PS_SAMPLE_30CM_ID (30) //VENDOR ID

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
#define CONFIG_PS_ON            (CONFIG_PS_EN_BIT | CONFIG_PS_DR_120MA)

//ps data reg
#define REG_PS_DATA             (0x08)
// alsa data (12bit)
// low 8 bit (7:0)
#define REG_ALS_DATA_L          (0x09)
// high 4 bit (3:0)
#define REG_ALS_DATA_H          (0x0A)


struct reg_default {
	unsigned int reg;
	unsigned int def;
};

#define PS_MAX_VALUE (255)

/*
1. 取采样值作为该sensor的距离依据, 相当于经验数据
2. dts 里配置的 poll_delay_ms 为 1000 ms,
   每次读取数据时, 开关sensor等待时间是300ms.所以获取一次sensor数据是1300ms
*/
#define DISTANCE_30CM_DEFAULT (150)
#define DISTANCE_30CM_DELTA (20)

#define SAMPLE_TIME_MS (1000) //采样间隔时间
#define SLEEP_MS_BETWEEN_PS_ON_AND_OFF (300)

#define PS_SAMPLE_VALUES_COUNT (7) // 去除一个最大值, 一个最小值
struct em20918_sample {
    int values[PS_SAMPLE_VALUES_COUNT];
    int values_count;
};

struct em20918_data {
    struct i2c_client *client;

    bool is_read_no_delay;
    bool is_stop_read_value;
    int sensor_value;

    int sample_values_30cm;
    struct em20918_sample sample;

    struct delayed_work dwork_sample;
    struct delayed_work dwork_vendor_read;

    struct class *cls_node;
    struct class_attribute s_sample_30cm;
    struct class_attribute s_sensor_value;
};

static struct em20918_data *g_em20918_data = NULL;

static const struct reg_default em20918_init_reg[] = {

    { 0x01, 0x0 },
    //soft reset(When Register 0x0E = 0x9C and Register 0x0F=0xE1, the soft reset is triggered)
    //{ 0x0E, 0x9C },
    //{ 0x0F, 0xE1 },

};

int em20918_init_all_reg(struct i2c_client *client)
{
    int ret = 0;
    int i = 0;
    for (i = 0; i < ARRAY_SIZE(em20918_init_reg); i++) {
        ret = sensor_write_reg(client, em20918_init_reg[i].reg, em20918_init_reg[i].def);
        if (ret) {
            dev_err(&client->dev, "%s:%d, failed to sensor_write_reg, reg(0x%02x):value(0x%02x) \n",
                __func__, __LINE__, em20918_init_reg[i].reg, em20918_init_reg[i].def);
        }
    }

    return ret;
}

static int em20918_get_sensor_value(struct em20918_data *ps_data)
{
    struct sensor_private_data *sensor =
            (struct sensor_private_data *) i2c_get_clientdata(ps_data->client);
    int ctrl_data;
    if (ps_data->is_stop_read_value) {
        ps_data->sensor_value = 0;
        return -1;
    }

    ctrl_data = sensor_read_reg(ps_data->client, sensor->ops->ctrl_reg);
    if (!(ctrl_data & CONFIG_PS_ON) && sensor->ops->active) {
        sensor->ops->active(ps_data->client, true, 0);
        msleep(120);
    }

    ps_data->sensor_value = sensor_read_reg(ps_data->client, REG_PS_DATA);

    return ps_data->sensor_value;
}

static int em20918_get_nf_value(struct em20918_data *ps_data)
{
    int value = em20918_get_sensor_value(ps_data);

    if (value < 0) {
        return -1;
    }

    if (value <= ps_data->sample_values_30cm - DISTANCE_30CM_DELTA) {
        return 40;
    } else if (value <= ps_data->sample_values_30cm + DISTANCE_30CM_DELTA) {
        return 30;
    } else if (value <= PS_MAX_VALUE - 1) {
        return 1;
    }

    return 0;
}

static int em20918_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    struct em20918_data *ps_data = g_em20918_data;
    int value = em20918_get_nf_value(ps_data);

    EM20918_DEBUG("%s:  psensor value = %d, report value = %d\n", __func__,
            ps_data->sensor_value, value);

    if (value < 0) {
        return 0;
    }

    input_report_abs(sensor->input_dev, ABS_DISTANCE, value);
    input_sync(sensor->input_dev);

    return 0;
}

static int em20918_active(struct i2c_client *client, int enable, int rate)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);

    sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
    sensor->ops->ctrl_data &= (~(CONFIG_PS_MASK));

    if (enable) {
        sensor->ops->ctrl_data |= (CONFIG_PS_ON & CONFIG_PS_MASK);
    }

    return sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
}

////////////////////////////////////////////////////////////////

static void sample_work_start(struct em20918_data *ps_data)
{
    if (!delayed_work_pending(&ps_data->dwork_sample)) {
        schedule_delayed_work(&ps_data->dwork_sample, msecs_to_jiffies(SAMPLE_TIME_MS));
    }

}

static void sample_start(struct em20918_data *ps_data)
{

    ps_data->sample.values_count = 0;
    memset(ps_data->sample.values, 0, sizeof(ps_data->sample.values));

    sample_work_start(ps_data);

}

static int sample_stop(struct em20918_data *ps_data)
{
    int64_t total = 0;
    int max = 0;
    int min = 0xffff;
    int i = 0;
    bool is_error_value = false;

    cancel_delayed_work_sync(&ps_data->dwork_sample);
    em20918_active(ps_data->client, false, 0);

    if (ps_data->sample.values_count < ARRAY_SIZE(ps_data->sample.values) / 2) {
        dev_err(&ps_data->client->dev, "sample count(%d) is not enough, at least count = %d.\n",
                    ps_data->sample.values_count, (int)(ARRAY_SIZE(ps_data->sample.values) / 2));

        ps_data->sample_values_30cm = 0;
        goto out;
    }

    if (ps_data->sample.values_count > ARRAY_SIZE(ps_data->sample.values)) {
        ps_data->sample.values_count = ARRAY_SIZE(ps_data->sample.values);
    }

    for (i = 0; i < ps_data->sample.values_count; i++) {
        total += ps_data->sample.values[i];
        if (max < ps_data->sample.values[i]) {
            max = ps_data->sample.values[i];
        }
        if (min > ps_data->sample.values[i]) {
            min = ps_data->sample.values[i];
        }
        if (ps_data->sample.values[i] <= 0) {
            is_error_value = true;
            break;
        }
    }

    if (is_error_value) {
        ps_data->sample_values_30cm = 0;
        goto out;
    }
    //去掉最大值和最小值
    total -= (max + min);

    ps_data->sample_values_30cm
        = (int)(total / (ps_data->sample.values_count - 2));

    dev_warn(&ps_data->client->dev, "%s: ps_data->sample_values_30cm = %d\n",
            __func__, ps_data->sample_values_30cm);

    i = rk_vendor_write(PS_SAMPLE_30CM_ID, &ps_data->sample_values_30cm, sizeof(ps_data->sample_values_30cm));
    dev_warn(&ps_data->client->dev, "songvendor, rk_vendor_write ret = %d, sample_values_30cm = %d.\n", i, ps_data->sample_values_30cm);

out:
    ps_data->sample.values_count = 0;
    memset(ps_data->sample.values, 0, sizeof(ps_data->sample.values));

    return 0;
}

static void sample_store_sensor_value(struct em20918_data *ps_data)
{

    int count = (++ps_data->sample.values_count);

    if (count >= ARRAY_SIZE(ps_data->sample.values)) {
        count %= ARRAY_SIZE(ps_data->sample.values);
    }
    printk("sample_store_sensor_value count = %d.\n", count);
    ps_data->sample.values[count] = em20918_get_sensor_value(ps_data);

}

static void sample_work_func(struct work_struct *work)
{

    struct em20918_data *ps_data = container_of(to_delayed_work(work), struct em20918_data, dwork_sample);

    sample_store_sensor_value(ps_data);

    sample_work_start(ps_data);
}

static void sample_vendor_read_work_func(struct work_struct *work)
{

    struct em20918_data *ps_data = container_of(to_delayed_work(work), struct em20918_data, dwork_vendor_read);
    int count = 10;
    int ret = -1;

    while (count-- > 0) {
        if (is_rk_vendor_ready())
            break;
        /* sleep 500ms wait rk vendor driver ready */
        msleep(500);
    }
    ret = rk_vendor_read(PS_SAMPLE_30CM_ID, &ps_data->sample_values_30cm, sizeof(ps_data->sample_values_30cm));
    dev_warn(&ps_data->client->dev, "rk_vendor_read, ret = %d, sample_values_30cm = %d.\n", ret, ps_data->sample_values_30cm);
    if (ret != sizeof(ps_data->sample_values_30cm) || 0 == ps_data->sample_values_30cm) {
        ps_data->sample_values_30cm = DISTANCE_30CM_DEFAULT;
    }

}

static ssize_t show_sample_30cm(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct em20918_data *ps_data = container_of(attr, struct em20918_data, s_sample_30cm);
    ssize_t len = 0;

    len += snprintf(_buf + len, PAGE_SIZE - len, "%d\n", ps_data->sample_values_30cm);

    return len;
}

static ssize_t store_sample_30cm(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct em20918_data *ps_data = container_of(attr, struct em20918_data, s_sample_30cm);
    int ret;
    char key[64];

    ret = sscanf(buf, "%s", key);
    if (!ret) {
        dev_err(&ps_data->client->dev, "%s:sscanf failed! cmd buf:%s\n", __func__, buf);
        return -EINVAL;
    }

    dev_warn(&ps_data->client->dev, "%s:key = %s\n", __func__, key);

    if (strcasecmp("start", key) == 0) {
        sample_start(ps_data);
    } else if (strcasecmp("stop", key) == 0) {
        sample_stop(ps_data);
    } else if (strcasecmp("on", key) == 0) {
        em20918_init_all_reg(ps_data->client);
        ps_data->is_stop_read_value = false;
    } else if (strcasecmp("off", key) == 0) {
        sensor_write_reg(ps_data->client, REG_CONFIG, 0);
        ps_data->is_stop_read_value = true;
    } else if (strcasecmp("read_no_delay_on", key) == 0) {
        ps_data->is_read_no_delay = true;
    } else if (strcasecmp("read_no_delay_off", key) == 0) {
        ps_data->is_read_no_delay = false;
    } else {
        dev_err(&ps_data->client->dev, "%s:error cmd(%s). support cmds: start, stop; eg, echo start > this_class_node.\n", __func__, buf);
        return -EINVAL;
    }

    return _count;
}

static ssize_t show_sensor_value(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct em20918_data *ps_data = container_of(attr, struct em20918_data, s_sensor_value);
    ssize_t len = 0;

    len += snprintf(_buf + len, PAGE_SIZE - len, "%d\n", ps_data->sensor_value);

    return len;
}

static ssize_t store_sensor_value(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
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

/////////////////////////////////////////////////////////////////////////////////////

static int em20918_init(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    struct em20918_data *ps_data;
    int err = 0;
	//struct regulator_dev *rdev = sensor->supply->rdev;
    EM20918_DEBUG("-----------sensor_init----ps_em20918\n");

    ps_data = kzalloc(sizeof(struct em20918_data), GFP_KERNEL);
    if (!ps_data) {
        dev_err(&client->dev, "%s: failed to allocate em20918_data\n", __func__);
        return -1;
    }
    ps_data->client = client;
	if (!IS_ERR_OR_NULL(sensor->supply)) {
		if(!sensor->is_poweron){
		EM20918_DEBUG("-----------sensor_init----ps_em20918 power on \n");
			if(!regulator_is_enabled(sensor->supply)) {
				err = regulator_enable(sensor->supply);
			}
			sensor->is_poweron = true;
		}
	}

    err = em20918_init_all_reg(client);
    if (err) {
        dev_err(&client->dev, "%s:em20918_init_all_reg fail %d\n", __func__, err);
        goto error_free;
    }

    err = sensor->ops->active(client, false, 0);
    if (err) {
        dev_err(&client->dev, "%s:%d, sensor->ops->active error\n", __func__, __LINE__);
        goto error_free;
    }

    sensor->status_cur = SENSOR_OFF;

    INIT_DELAYED_WORK(&ps_data->dwork_sample, sample_work_func);
    INIT_DELAYED_WORK(&ps_data->dwork_vendor_read, sample_vendor_read_work_func);
    schedule_delayed_work(&ps_data->dwork_vendor_read, msecs_to_jiffies(3000));

    ps_data->cls_node = class_create(THIS_MODULE, "htfyun-ps");
    if ( IS_ERR_OR_NULL(ps_data->cls_node)) {
        dev_err(&client->dev, "failed to class_create ps_em20918 err=%d\n", err);
    } else {
        CLASS_CREATE_FILE(ps_data, sample_30cm);
        CLASS_CREATE_FILE(ps_data, sensor_value);
    }

    ps_data->is_stop_read_value = false;
    g_em20918_data = ps_data;

    return 0;
error_free:
    if (ps_data) {
        kfree(ps_data);
        ps_data = NULL;
    }
    return err;
}


#ifdef CONFIG_PM
static int em20918_suspend(struct i2c_client *client)
{
    struct sensor_private_data *sensor =
        (struct sensor_private_data *) i2c_get_clientdata(client);
    sensor_write_reg(client, REG_CONFIG, 0);
    sensor->status_cur = SENSOR_OFF;
    return 0;
}

static int em20918_resume(struct i2c_client *client)
{
    em20918_init_all_reg(client);
    return 0;
}
#endif

struct sensor_operate proximity_em20918_ops = {
    .name               = "ps_em20918",
    .type               = SENSOR_TYPE_PROXIMITY,    //sensor type and it should be correct
    .id_i2c             = PROXIMITY_ID_EM20918,     //i2c id number
    .read_reg           = SENSOR_UNKNOW_DATA,           //read data
    .read_len           = 1,                //data length
    .id_reg             = SENSOR_UNKNOW_DATA,       //read device id from this register
    .id_data            = SENSOR_UNKNOW_DATA,       //device id
    .precision          = 8,                //8 bits
    .ctrl_reg           = REG_CONFIG,          //enable or disable
    .int_status_reg     = SENSOR_UNKNOW_DATA,            //intterupt status register
    .range              = {0, 100},           //range
    .trig               = IRQF_TRIGGER_NONE,
    .active             = em20918_active,
    .init               = em20918_init,
    .report             = em20918_report_value,
#ifdef CONFIG_PM
    .suspend            = em20918_suspend,
    .resume             = em20918_resume,
#endif
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *proximity_get_ops(void)
{
    return &proximity_em20918_ops;
}

static int __init proximity_em20918_init(void)
{

    return sensor_register_slave(SENSOR_TYPE_PROXIMITY, NULL, NULL, proximity_get_ops);

}

static void __exit proximity_em20918_exit(void)
{
    struct em20918_data *ps_data = g_em20918_data;
    sensor_unregister_slave(SENSOR_TYPE_PROXIMITY, NULL, NULL, proximity_get_ops);
    if (ps_data) {
        if (ps_data->cls_node) {
            CLASS_REMOVE_FILE(ps_data, sample_30cm);
            CLASS_REMOVE_FILE(ps_data, sensor_value);
            class_destroy(ps_data->cls_node);
            ps_data->cls_node = NULL;
        }
        kfree(ps_data);
        ps_data = NULL;
        g_em20918_data = NULL;
    }
}


module_init(proximity_em20918_init);
module_exit(proximity_em20918_exit);

