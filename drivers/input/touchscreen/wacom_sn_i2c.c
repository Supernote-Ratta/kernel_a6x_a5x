/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/suspend.h>

#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/proc_ratta.h>
#include <linux/fb.h>
#include <linux/input/ratta_touch.h>

int wacom_touch_action_down = 0;
volatile int wacom_last_pressure = 0;
static RAW_NOTIFIER_HEAD(wacom_touch_event_notifier);
int register_wacom_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_register(&wacom_touch_event_notifier, nb);
}

int unregister_wacom_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_unregister(&wacom_touch_event_notifier, nb);
}

EXPORT_SYMBOL(register_wacom_touch_event_notifier);
EXPORT_SYMBOL(unregister_wacom_touch_event_notifier);

/*-----------------------------------*/
/*-----------------------------------*/
/*------Wacom specific items---------*/
/*-----------------------------------*/
/*-----------------------------------*/
#define I2C_DEVICE          "/dev/i2c-1"
#define I2C_TARGET          0x09

#define MPU_W9021            0x45
#define FLASH_BLOCK_SIZE     256
#define FLASH_DATA_SIZE            (65536 * 5)
#define BLOCK_NUM            63
#define W9021_START_ADDR     0x3000
#define W9021_END_ADDR       0x3efff

#define BOOT_CMD_SIZE        (0x010c + 0x02)//78
#define BOOT_RSP_SIZE        6

#define BOOT_WRITE_FLASH     1
#define BOOT_EXIT            3
#define BOOT_BLVER           4
#define BOOT_MPU             5
#define BOOT_QUERY           7
#define ERS_ALL_CMD           0x10

#define ERS_ECH2              0x03
#define QUERY_RSP             0x06

#define PROCESS_INPROGRESS    0xff
#define PROCESS_COMPLETED     0x00
#define PROCESS_CHKSUM1_ERR   0x81
#define PROCESS_CHKSUM2_ERR   0x82
#define PROCESS_TIMEOUT_ERR   0x87
#define RETRY_COUNT           5
#define EXIT_FAIL             1
#define HEX_READ_ERR          -1

/*-----------------------------------*/
/*-----------------------------------*/
/*------ HID requiring items---------*/
/*-----------------------------------*/
/*-----------------------------------*/
#define RTYPE_FEATURE          0x03 /*: Report type -> feature(11b)*/
#define CMD_GET_FEATURE        2
#define CMD_SET_FEATURE        3

#define GFEATURE_SIZE          6
#define SFEATURE_SIZE          8

/*Report IDs for Wacom device*/
#define REPORT_ID_1             0x07
#define REPORT_ID_2             0x08
#define FLASH_CMD_REPORT_ID     2
#define BOOT_CMD_REPORT_ID      7

//#define	ORIGIN_COORD

#ifdef ORIGIN_COORD
static int exchange_x_y_flag 	= 0;//1
static int revert_x_flag 		= 0;//1
static int revert_y_flag 		= 0;
#else
static int exchange_x_y_flag 	= 0;
static int revert_x_flag 		= 0;
static int revert_y_flag 		= 0;

#endif
int screen_max_x = 20280;
int screen_max_y = 13942;

#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	21
#define WACOM_REGULAR_INPUT	10

struct feature_support {
	bool height;
	bool tilt;
};

typedef struct hid_descriptor {
	u16 wHIDDescLength;
	u16 bcdVersion;
	u16 wReportDescLength;
	u16 wReportDescRegister;
	u16 wInputRegister;
	u16 wMaxInputLength;
	u16 wOutputRegister;
	u16 wMaxOutputLength;
	u16 wCommandRegister;
	u16 wDataRegister;
	u16 wVendorID;
	u16 wProductID;
	u16 wVersion;
	u16 RESERVED_HIGH;
	u16 RESERVED_LOW;
} HID_DESC;

struct wacom_features {
	struct feature_support support;
	int x_max;
	int y_max;
	int pressure_max;
	int height_max;
	int tilt_x_max;
	int tilt_y_max;
	int fw_version;
	HID_DESC hid_desc;
};

/*HID specific register*/
#define HID_DESC_REGISTER       1
#define COMM_REG                0x04
#define DATA_REG                0x05

struct wacom_event {
	struct list_head list;
	int code;
};

struct wacom_i2c {
	struct wacom_features *features;
	struct i2c_client *client;
	struct input_dev *input;
	struct mutex fw_lock;
	struct mutex pwr_mutex;
	struct task_struct *notifier;
	struct list_head notifier_head;
	struct kmem_cache *notifier_cache;
	struct semaphore notifier_sema;
	const char *fw_name;
	spinlock_t notifier_lock;
	bool active;
	bool   fw_done;
	u8 data[WACOM_QUERY_SIZE];
	volatile u8 data3;
	bool prox;
	int tool;
	bool 	irq_enabled;
	int 	irq_gpio;
	//int 	detect_gpio;
	//int 	detect_irq;
	int reset_gpio;
	int pwren_gpio;
	struct notifier_block	fb_notify;
};
extern bool fb_power_off(void);
static int g_irq_gpio = -1;
static int wacom_i2c_add_event(struct wacom_i2c *wac_i2c,
			       struct wacom_event *event)
{
	spin_lock(&wac_i2c->notifier_lock);
	list_add_tail(&event->list, &wac_i2c->notifier_head);
	spin_unlock(&wac_i2c->notifier_lock);

	return 0;
}

static struct wacom_event *wacom_i2c_get_event(struct wacom_i2c *wac_i2c)
{
	struct wacom_event *tmp = NULL;

	spin_lock(&wac_i2c->notifier_lock);
	if (!list_empty(&wac_i2c->notifier_head)) {
		tmp = list_first_entry(&wac_i2c->notifier_head,
				       typeof(*tmp),
				       list);
		list_del(&tmp->list);
	}
	spin_unlock(&wac_i2c->notifier_lock);

	return tmp;
}

static int wacom_touch_event_notifier_call_chain(unsigned long val, void *v)
{
    struct wacom_i2c *wac_i2c = v;
    struct wacom_event *tmp = NULL;

    if (!wac_i2c->active)
	    return 0;

    tmp = kmem_cache_alloc(wac_i2c->notifier_cache, GFP_KERNEL);
    if (tmp) {
	memset(tmp, 0, sizeof(*tmp));
        tmp->code = val & 0x1;
        wacom_i2c_add_event(wac_i2c, tmp);
        up(&wac_i2c->notifier_sema);
    }

    return 0;
}
static void wacom_power_switch(struct wacom_i2c *wacom, bool keep_on)
{

		int ret = 0;
		printk("%s on_off:%d \n", __func__, keep_on);
		if(keep_on) {
			if (gpio_is_valid(wacom->pwren_gpio)) {
				gpio_set_value(wacom->pwren_gpio, 1);
				printk("%s power18_gpio on \n", __func__);
			}
			msleep(100);
			gpio_set_value(wacom->reset_gpio, 1);

			gpio_direction_input(wacom->irq_gpio);
			//gpio_direction_input(wacom->detect_gpio);
			printk("%s:regulater on wacom,ret=%d\n", __func__, ret);
		} else {
			if (gpio_is_valid(wacom->pwren_gpio)) {
				gpio_set_value(wacom->pwren_gpio, 0);
				printk("%s pwren_gpio off \n", __func__);
			}

			gpio_set_value(wacom->reset_gpio, 0);

			gpio_direction_output(wacom->irq_gpio, 0);
			//gpio_direction_output(wacom->detect_gpio, 0);
			printk("%s:regulater off wacom,ret=%d\n", __func__, ret);
		}
		if( ret ) {
			printk("%s: failed, powr_on=%d,ret=%d\n", __func__, keep_on, ret);
		}

}
static int wacom_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct wacom_i2c *cd =
		container_of(self, struct wacom_i2c, fb_notify);
	struct fb_event *evdata = data;
	int blank;

    // 20180327,the touch only handle FB_EARLY_EVENT_BLANK.
	if (event != FB_EARLY_EVENT_BLANK/*FB_EVENT_BLANK*/ || !evdata)
		goto exit;

    //if( !fb_eink(evdata->info) ){ //20200109,hsl.
    //    return NOTIFY_DONE;
    //}
	blank = *((int*)evdata->data);

	// 20191207-LOG: wacom_fb_notifier: event=16, blank=0
	
	printk("wacom_fb_notifier event:%lu,blank:%d\n", event, blank);
    //mutex_lock(&cd->pwr_mutex);
	if (blank == FB_BLANK_UNBLANK) {
		if(!cd->irq_enabled) {
			// 20191207: fb ´¦ÓÚÕý³£×´Ì¬.
			wacom_power_switch(cd, true);
			//enable_irq_wake(cd->detect_irq); // 20191207,for balance
			enable_irq(cd->client->irq);
			//enable_irq(cd->detect_irq);
			cd->irq_enabled = true;
		}
	} else if (blank == FB_BLANK_POWERDOWN) {
		// 20191207: fb½øÈë´ý»ú£¬´ý»ú½çÃæÏÔÊ¾³öÀ´ÁË.
		if(cd->irq_enabled) {
			disable_irq(cd->client->irq);
            cd->irq_enabled = false;
			//disable_irq(cd->detect_irq);
			//disable_irq_wake(cd->detect_irq);
			//msleep(5);
			wacom_power_switch(cd, false);
		}
	}
    //mutex_unlock(&cd->pwr_mutex);
exit:
	return 0;
}

static void wacom_setup_fb_notifier(struct wacom_i2c *wacom)
{
	int rc;
	wacom->fb_notify.notifier_call = wacom_fb_notifier_callback;
	rc = fb_register_client(&wacom->fb_notify);
	if (rc) {
		dev_err(&wacom->client->dev, "Unable to register fb_notifier: %d\n", rc);
		return ;
	}
	device_init_wakeup(&wacom->client->dev, true);
	//disable_irq_wake(wacom->detect_irq); // 20191207,for balance
	//disable_irq_wake(wacom->client->irq); // 20191207,for balance
	//printk("wacom_setup_fb_notifier: enable_irq_wake %d!\n", wacom->detect_irq);
}

int get_hid_desc(struct i2c_client *client,
			      struct hid_descriptor *hid_desc)
{
	int ret = -1;
	char cmd[] = {HID_DESC_REGISTER, 0x00};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd),
			.buf = cmd,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(HID_DESC),
			.buf = (char *)hid_desc,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	printk("******************************\n");
	printk("wacom firmware vesrsion:0x%x\n",hid_desc->wVersion);
	printk("******************************\n");

	ret = 0;
//out:
	return ret;
}


static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 cmd_hid_desc[] = {HID_DESC_REGISTER, 0x00};
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	features->height_max = data[15];
	features->tilt_x_max = get_unaligned_le16(&data[17]);
	features->tilt_y_max =  get_unaligned_le16(&data[19]);

	if (features->height_max)
		features->support.height = true;
	else
		features->support.height = false;

	if (features->tilt_x_max && features->tilt_y_max)
		features->support.tilt = true;
	else
		features->support.tilt = false;

	ret = i2c_master_send(client, cmd_hid_desc, sizeof(cmd_hid_desc));
	if (ret < 0) {
		dev_err(&client->dev, "cannot send register info\n");
		return ret;
	}

	ret = i2c_master_recv(client, (char *)&features->hid_desc, sizeof(HID_DESC));
	if (ret < 0) {
		dev_err(&client->dev, "cannot receive register info\n");
		return ret;
	}

	dev_info(&client->dev, "wacom source screen x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		 features->x_max, features->y_max,
		 features->pressure_max, features->fw_version);

	if (1 == exchange_x_y_flag) {
		swap(features->x_max, features->y_max);
	}
	screen_max_x = features->x_max;
	screen_max_y = features->y_max;
	dev_info(&client->dev, "wacom desc screen x_max:%d, y_max:%d\n", features->x_max, features->y_max);

	dev_info(&client->dev, "height:%d, tilt_x:%d, tilt_y:%d\n",
		 features->height_max, features->tilt_x_max, features->tilt_y_max);

	return 0;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	//struct wacom_features *features = wac_i2c->features;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;
	int error;
	int data_len = wac_i2c->features->hid_desc.wMaxInputLength;
	int tilt_x, tilt_y;
	volatile bool need_report = true;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, sizeof(wac_i2c->data));
	if (error < 0)
		goto out;

	pressure = le16_to_cpup((__le16 *)&data[8]);
	if (!pressure && !wacom_last_pressure && (wac_i2c->data3 == data[3]))
		need_report = false;
	if (!wacom_last_pressure != !pressure)
		wacom_last_pressure = pressure;

#if 0
	dev_info(&wac_i2c->client->dev, "pressure=%d,data[3]=0x%02x\n",
			 pressure, data[3]);
#endif

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = !!(data[3] & 0x20);

	if (pressure != 0 ||
		((wac_i2c->tool == BTN_TOOL_PEN) &&
		 (wac_i2c->prox == 0x1))) {
		if (wacom_touch_action_down == 0) {
			wacom_touch_action_down = 1;
			if (wacom_touch_event_notifier_call_chain(1, wac_i2c)) {
				printk("wacom_touch_event_notifier_call_chain error\n");
			}
		}
	} else {
		if (wacom_touch_action_down == 1) {
			wacom_touch_action_down = 0;
			if (wacom_touch_event_notifier_call_chain(0, wac_i2c)) {
				printk("wacom_touch_event_notifier_call_chain error\n");
			}
		}
	}

	if (!need_report)
		goto out;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);

	ratta_set_raw_pen_type(data[2]);
	if (data[2] == 2)
		/* G12 */
		ratta_set_pen_type(12);
	else
		/* G14 */
		ratta_set_pen_type(14);

	if (1 == exchange_x_y_flag) {
		swap(x, y);
	}
	if (1 == revert_x_flag) {
		x = screen_max_x - x - 1;
	}
	if (1 == revert_y_flag) {
		y = screen_max_y - y - 1;
	}

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);

	if (data_len > WACOM_REGULAR_INPUT) {
		if (wac_i2c->features->support.height)
			input_report_abs(input, ABS_DISTANCE, data[10]);

		if (wac_i2c->features->support.tilt) {
			tilt_x = (int)le16_to_cpup((__le16 *)&data[11]);
			tilt_y = (int)le16_to_cpup((__le16 *)&data[13]);
			input_report_abs(input, ABS_TILT_X, tilt_x);
			input_report_abs(input, ABS_TILT_Y, tilt_y);
		}
	}

	input_sync(input);

out:
	wac_i2c->data3 = data[3];

	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);
    wac_i2c->irq_enabled = true;
	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static ssize_t wacom_fw_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

        return sprintf(buf,"%04X.%04X\n",
		       wac_i2c->input->id.product,
		       wac_i2c->features->fw_version);
}
static DEVICE_ATTR(fw_version, S_IRUGO, wacom_fw_version_show, NULL);

static unsigned long __maybe_unused read_hex(const char *buf, int size)
{
	unsigned long sum = 0;
	int i, j;

	for (i = size - 1, j = 1; i >= 0; i--) {
		if ((buf[i]  <= '9') && (buf[i] >= 0))
			sum += (buf[i] - '0') * j;
		else if ((buf[i] <= 'Z') && (buf[i] >= 'A'))
			sum += (buf[i] - 'A' + 10) * j;
		else if ((buf[i] <= 'z') && (buf[i] >= 'a'))
			sum += (buf[i] - 'a' + 10) * j;
		else
			continue;
		j <<= 4;
	}

	return sum;
}

static int wacom_i2c_read_hex(const struct firmware *fw, u8 *data, int size,
			      unsigned long *max_address)
{
	u8 *fix = (u8 *)(fw->data), *var = fix;
	int cnt = 0, cnts, s;
	unsigned long expand_address = 0;
	unsigned long startLinearAddress = 0;

	while ((*var != ':') && ((var - fix) < fw->size))
		var++;
	if ((var - fix) >= fw->size) {
		printk(KERN_ERR "wacom: no ':' in firmware file.\n");
		return -EINVAL;
	}

	cnts = fw->size - (var - fix);

	while (cnt < cnts) {
		unsigned int address = 0, total = 0;
		unsigned int sum = 0, byte_count = 0;
		unsigned int record_type = 0;
		int cr = 0, lf = 0;
		unsigned int i = 0;
		unsigned int tmp = 0;

		s = var[cnt];
		cnt++;

		if (s != ':') {
			if (s == 0x1A) {
				printk(KERN_ERR "wacom: got hex end.\n");
				return cnt;
			}
			printk(KERN_ERR "wacom: invalid hex file.\n");
			return -EINVAL;
		}

		//sscanf(&var[cnt], "%02X", &byte_count);
		byte_count = read_hex(&var[cnt], 2);
		cnt += 2;

		//sscanf(&var[cnt], "%04X", (unsigned int *)&address);
		address = read_hex(&var[cnt], 4);
		cnt += 4;

		//sscanf(&var[cnt], "%02X", &record_type);
		record_type = read_hex(&var[cnt], 2);
		cnt += 2;

		switch (record_type) {
		case 0:
			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;
			address += expand_address;
			if (address > *max_address) {
				*max_address = address;
				*max_address += (byte_count-1);
			}

			for (i = 0; i < byte_count; i++){
				//sscanf(&var[cnt], "%2X", &tmp);
				tmp = read_hex(&var[cnt], 2);
				cnt += 2;
				total += tmp;

				if ((address + i) < size)
					data[address + i] = (unsigned char)tmp;
			}

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;

			total += sum;
			if ((unsigned char)(total & 0xff) != 0x00) {
				printk("wacom: HEX_READ_ERR 4(%d)\n", cnt);
				return -EINVAL; /* check sum error */
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 5(%d)\n", cnt);
				return -EINVAL;
			}

			break;
		case 1:
			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;

			total += sum;
			if ((unsigned char)(total & 0xff) != 0x00) {
				printk("wacom: HEX_READ_ERR 6(%d)\n", cnt);
				return -EINVAL; /* check sum error */
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 7(%d)\n", cnt);
				return -EINVAL;
			}

			break;
		case 2:
			//sscanf(&var[cnt], "%4lX", &expand_address);
			expand_address = read_hex(&var[cnt], 4);
			cnt += 4;

			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;
			total += (unsigned char)(expand_address);
			total += (unsigned char)(expand_address >> 8);

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;

			total += sum;
			if ((unsigned char)(total & 0xff) != 0x00) {
				printk("wacom: HEX_READ_ERR 8(%d)\n", cnt);
				return -EINVAL; /* check sum error */
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 9(%d)\n", cnt);
				return -EINVAL;
			}

			expand_address <<= 4;

			break;
		case 3:
		{
			unsigned long cs=0, ip=0;

			//sscanf(&var[cnt], "%4lX", &cs);
			cs = read_hex(&var[cnt], 4);
			cnt += 4;

			//sscanf(&var[cnt], "%4lX", &ip);
			ip = read_hex(&var[cnt], 4);
			cnt += 4;

			expand_address = (cs << 4) + ip;

			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;
			total += (unsigned char)(cs);
			total += (unsigned char)(cs >> 8);
			total += (unsigned char)(ip);
			total += (unsigned char)(ip >> 8);

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;
			total += sum;

			if ((unsigned char)(total & 0x0f) != 0x00) {
				printk("wacom: HEX_READ_ERR 10(%d)\n", cnt);
				return -EINVAL;
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 11(%d)\n", cnt);
				return -EINVAL;
			}

			expand_address <<= 16;

			break;
		}
		case 4:
			//sscanf(&var[cnt], "%4lX", &expand_address);
			expand_address = read_hex(&var[cnt], 4);
			cnt += 4;

			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;
			total += (unsigned char)(expand_address);
			total += (unsigned char)(expand_address >> 8);

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;

			total += sum;

			if ((unsigned char)(total & 0xff) != 0x00) {
				printk("wacom: HEX_READ_ERR 12(%d)\n", cnt);
				return -EINVAL; /* check sum error */
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 13(%d)\n", cnt);
				return -EINVAL;
			}

			expand_address <<= 16;

			break;
		case 5:
			//sscanf(&var[cnt], "%8lX", &startLinearAddress);
			startLinearAddress = read_hex(&var[cnt], 8);
			cnt += 8;

			total = byte_count;
			total += (unsigned char)(address);
			total += (unsigned char)(address >> 8);
			total += record_type;
			total += (unsigned char)(startLinearAddress);
			total += (unsigned char)(startLinearAddress >> 8);
			total += (unsigned char)(startLinearAddress >> 16);
			total += (unsigned char)(startLinearAddress >> 24);

			//sscanf(&var[cnt], "%2X", &sum);
			sum = read_hex(&var[cnt], 2);
			cnt += 2;
			total += sum;

#if 0
			printf("byte_count: %d\n", byte_count);
			printf("Address: %d\n", address);
			printf("record_type %d\n", record_type);
			printf("startLinearAddress: %d\n", startLinearAddress);
			printf("total: %d \n\n\n", total);
#endif
			if ((unsigned char)(total & 0x0f) != 0x00) {
				printk("wacom: HEX_READ_ERR 14(%d)\n", cnt);
				return -EINVAL; /* check sum error */
			}

			cr = var[cnt];
			cnt++;

			lf = var[cnt];
			cnt++;

			if (cr != '\r' || lf != '\n') {
				printk("wacom: HEX_READ_ERR 15(%d)\n", cnt);
				return -EINVAL;
			}

			break;

		default:
			printk("wacom: HEX_READ_ERR 16(%d)\n", cnt);
			return -EINVAL;
		}
	}

	return cnt;
}

static int wacom_i2c_write(struct wacom_i2c *wac_i2c, u8 *buf, int len)
{
	struct i2c_client *client = wac_i2c->client;
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return len;
}

static bool wacom_i2c_set_feature(struct wacom_i2c *wac_i2c,
				  u8 report_id, unsigned int buf_size, u8 *data,
				  u16 cmdreg, u16 datareg)
{
	int i, ret = -1;
	int total = SFEATURE_SIZE + buf_size;
	u8 *sFeature = NULL;
	bool bRet = false;

	sFeature = kmalloc(sizeof(u8) * total, GFP_KERNEL);
	if (!sFeature) {
		printk("wacom: cannot preserve memory\n");
		goto out;
	}

	memset(sFeature, 0, sizeof(u8) * total);

	sFeature[0] = (u8)(cmdreg & 0x00ff);
	sFeature[1] = (u8)((cmdreg & 0xff00) >> 8);
	sFeature[2] = (RTYPE_FEATURE << 4) | report_id;
	sFeature[3] = CMD_SET_FEATURE;
	sFeature[4] = (u8)(datareg & 0x00ff);
	sFeature[5] = (u8)((datareg & 0xff00) >> 8);

	if ( (buf_size + 2) > 255) {
		sFeature[6] = (u8)((buf_size + 2) & 0x00ff);
		sFeature[7] = (u8)(( (buf_size + 2) & 0xff00) >> 8);
	} else {
		sFeature[6] = (u8)(buf_size + 2);
		sFeature[7] = (u8)(0x00);
	}

	for (i = 0; i < buf_size; i++)
		sFeature[i + SFEATURE_SIZE] = *(data + i);

	ret = wacom_i2c_write(wac_i2c, sFeature, total);
	if (ret != total) {
		printk("wacom: set feature failed sent bytes: %d\n", ret);
		goto err;
	}

	bRet = true;
err:
	kfree(sFeature);
	sFeature = NULL;

out:
	return bRet;
}

static bool wacom_i2c_get_feature(struct wacom_i2c *wac_i2c, u8 report_id,
				  unsigned int buf_size, u8 *data,
				  u16 cmdreg, u16 datareg)
{
	struct i2c_client *client = wac_i2c->client;
	/*"+ 2", adding 2 more spaces for organizeing again later in the passed data, "data"*/
	unsigned int total = buf_size + 2;
	char *recv = NULL;
	bool bRet = false;
	u8 gFeature[] = {
		(u8)(cmdreg & 0x00ff),
		(u8)((cmdreg & 0xff00) >> 8),
		(RTYPE_FEATURE << 4) | report_id,
		CMD_GET_FEATURE,
		(u8)(datareg & 0x00ff),
		(u8)((datareg & 0xff00) >> 8)
	};
	int ret = 0;

	recv = kmalloc(sizeof(char) * total, GFP_KERNEL);
	if (recv == NULL) {
		printk("wacom: %s: cannot preserve memory \n", __func__);
		goto out;
	}
	memset(recv, 0, sizeof(char) * total);

	{
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = GFEATURE_SIZE,
				.buf = (char *)gFeature,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = total,
				.buf = recv,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret != ARRAY_SIZE(msgs)) {
			printk("wacom: get feature transfer failed.\n");
			goto err;
		}
		/*First two bytes in recv are length of
		  the report and data doesn't need them*/
		memcpy(data, (unsigned char *)(recv + 2), buf_size);
	}

	bRet = true;
err:
	kfree(recv);
	recv = NULL;
out:
	return bRet;
}

static bool wacom_i2c_flash_cmd(struct wacom_i2c *wac_i2c)
{
	int len = 0;
	u8 cmd[2] = {0};
	bool bRet = false;

	cmd[len++] = 0x02;
	cmd[len++] = 0x02;

	bRet = wacom_i2c_set_feature(wac_i2c, FLASH_CMD_REPORT_ID,
				     len, cmd, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: Sending flash command failed.\n");
		return bRet;
	}

	msleep(300);

	return true;
}

static bool flash_query_w9021(struct wacom_i2c *wac_i2c)
{
	bool bRet = false;
	u8 command[BOOT_CMD_SIZE] = {0};
	u8 response[BOOT_RSP_SIZE] = {0};
	int ECH = 0, len = 0;

	command[len++] = BOOT_CMD_REPORT_ID;                    /* Report:ReportID */
	command[len++] = BOOT_QUERY;                            /* Report:Boot Query command */
	command[len++] = ECH = 7;                               /* Report:echo */

	bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1, len,
				     command, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature \n", __func__);
		return bRet;
	}

	bRet = wacom_i2c_get_feature(wac_i2c,
				     REPORT_ID_2, BOOT_RSP_SIZE,
				     response, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to get feature \n", __func__);
		return bRet;
	}

	if ((response[1] != BOOT_CMD_REPORT_ID) ||
	    (response[2] != ECH) ) {
		printk("wacom: %s: res1:%x res2:%x \n",
		       __func__, response[1], response[2]);
		return false;
	}

	if (response[3] != QUERY_RSP) {
		printk("wacom: %s: res3:%x \n", __func__, response[3]);
		return false;
	}

	return true;
}

static bool flash_blver_w9021(struct wacom_i2c *wac_i2c, int *blver)
{
	bool bRet = false;
	u8 command[BOOT_CMD_SIZE] = {0};
	u8 response[BOOT_RSP_SIZE] = {0};
	int ECH = 0, len = 0;

	command[len++] = BOOT_CMD_REPORT_ID;    /* Report:ReportID */
	command[len++] = BOOT_BLVER;            /* Report:Boot Version command */
	command[len++] = ECH = 7;               /* Report:echo */

	bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1, len,
				     command, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature1\n", __func__);
		return bRet;
	}

	bRet = wacom_i2c_get_feature(wac_i2c, REPORT_ID_2, BOOT_RSP_SIZE,
				     response, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature2\n", __func__);
		return bRet;
	}

	if ( (response[1] != BOOT_BLVER) ||
	     (response[2] != ECH) ) {
		printk("wacom: %s: res1:%x res2:%x \n",
		       __func__, response[1], response[2]);
		return false;
	}

	*blver = (int)response[3];

	return true;
}

static bool flash_mputype_w9021(struct wacom_i2c *wac_i2c, int* pMpuType)
{
	bool bRet = false;
	u8 command[BOOT_CMD_SIZE] = {0};
	u8 response[BOOT_RSP_SIZE] = {0};
	int ECH = 0, len = 0;

	command[len++] = BOOT_CMD_REPORT_ID;        /* Report:ReportID */
	command[len++] = BOOT_MPU;                  /* Report:Boot Query command */
	command[len++] = ECH = 7;                   /* Report:echo */

	bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1, len,
				     command, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature\n", __func__);
		return bRet;
	}

	bRet = wacom_i2c_get_feature(wac_i2c, REPORT_ID_2, BOOT_RSP_SIZE,
				     response, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to get feature\n", __func__);
		return bRet;
	}

	if ( (response[1] != BOOT_MPU) ||
	     (response[2] != ECH) ) {
		printk("wacom: %s: res1:%x res2:%x \n",
		       __func__, response[1], response[2]);
		return false;
	}

	*pMpuType = (int)response[3];
	return true;
}

static int check_progress(u8 *data, size_t size, u8 cmd, u8 ech)
{
	if (data[0] != cmd || data[1] != ech) {
		printk("wacom: %s: failed to erase\n", __func__);
		return -EFAULT;
	}

	switch (data[2]) {
	case PROCESS_CHKSUM1_ERR:
	case PROCESS_CHKSUM2_ERR:
	case PROCESS_TIMEOUT_ERR:
		printk("wacom: %s: error: %x\n", __func__, data[2]);
		return -EFAULT;
	}

	return data[2];
}

static bool flash_erase_all(struct wacom_i2c *wac_i2c)
{
	bool bRet = false;
	u8 command[BOOT_CMD_SIZE] = {0};
	u8 response[BOOT_RSP_SIZE] = {0};
	int i = 0, len = 0;
	int ECH = 0, sum = 0;
	int ret = -1;

	command[len++] = 7;
	command[len++] = ERS_ALL_CMD;
	command[len++] = ECH = 2;
	command[len++] = ERS_ECH2;

	//Preliminarily stored data that cannnot appear here, but in wacom_set_feature()
	sum += 0x05;
	sum += 0x07;
	for (i = 0; i < len; i++)
		sum += command[i];

	command[len++] = ~sum + 1;

	bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1, len,
				     command, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature\n", __func__);
		return bRet;
	}

	do {
		bRet = wacom_i2c_get_feature(wac_i2c, REPORT_ID_2,
					     BOOT_RSP_SIZE, response,
					     COMM_REG, DATA_REG);
		if (!bRet) {
			printk("wacom: %s: failed to set feature\n", __func__);
			return bRet;
		}

		ret = check_progress(&response[1], (BOOT_RSP_SIZE - 3),
				     ERS_ALL_CMD, ECH);
		if (ret < 0)
			return false;

	} while (ret == PROCESS_INPROGRESS);

	return true;
}

static bool flash_write_block_w9021(struct wacom_i2c *wac_i2c, char *flash_data,
                                    unsigned long ulAddress,
				    u8 *pcommand_id, int *ECH)
{
	const int MAX_COM_SIZE = (8 + FLASH_BLOCK_SIZE + 2); //8: num of command[0] to command[7]
	//FLASH_BLOCK_SIZE: unit to erase the block
	//Num of Last 2 checksums
	bool bRet = false;
	u8 command[300] = {0};
	unsigned char sum = 0;
	int i = 0;

	command[0] = BOOT_CMD_REPORT_ID;                /* Report:ReportID */
	command[1] = BOOT_WRITE_FLASH;                  /* Report:program  command */
	command[2] = *ECH = ++(*pcommand_id);           /* Report:echo */
	command[3] = ulAddress & 0x000000ff;
	command[4] = (ulAddress & 0x0000ff00) >> 8;
	command[5] = (ulAddress & 0x00ff0000) >> 16;
	command[6] = (ulAddress & 0xff000000) >> 24;    /* Report:address(4bytes) */
	command[7] = 0x20;

	/*Preliminarily stored data that cannnot appear here, but in wacom_set_feature()*/
	sum = 0;
	sum += 0x05; sum += 0x0c; sum += 0x01;
	for (i = 0; i < 8; i++)
		sum += command[i];
	command[MAX_COM_SIZE - 2] = ~sum + 1;           /* Report:command checksum */

	sum = 0;
	for (i = 8; i < (FLASH_BLOCK_SIZE + 8); i++){
		command[i] = flash_data[ulAddress+(i - 8)];
		sum += flash_data[ulAddress+(i - 8)];
	}

	command[MAX_COM_SIZE - 1] = ~sum+1;             /* Report:data checksum */

	/*Subtract 8 for the first 8 bytes*/
	bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1,
				     (BOOT_CMD_SIZE + 4 - 8),
				     command, COMM_REG, DATA_REG);
	if (!bRet) {
		printk("wacom: %s: failed to set feature at addr: %x\n",
		       __func__, (unsigned int)ulAddress);
		return bRet;
	}

	usleep_range(50, 100);

	return true;
}

static bool flash_write_w9021(struct wacom_i2c *wac_i2c,
			      unsigned char *flash_data,
                              unsigned long start_address,
			      unsigned long *max_address)
{
	bool bRet = false;
	u8 command_id = 0;
	u8 response[BOOT_RSP_SIZE] = {0};
	int i = 0, j = 0, ECH = 0, ECH_len = 0;
	int ECH_ARRAY[3] = {0};
	int ret = -1;
	unsigned long ulAddress = 0;

	j = 0;
	for (ulAddress = start_address; ulAddress < *max_address;
	     ulAddress += FLASH_BLOCK_SIZE) {
		for (i = 0; i < FLASH_BLOCK_SIZE; i++) {
			if (flash_data[ulAddress+i] != 0xFF)
				break;
		}

		if (i == (FLASH_BLOCK_SIZE))
			continue;

		bRet = flash_write_block_w9021(wac_i2c, flash_data, ulAddress,
					       &command_id, &ECH);
		if(!bRet)
			return bRet;
		if (ECH_len == 3)
			ECH_len = 0;

		ECH_ARRAY[ECH_len++] = ECH;
		if (ECH_len == 3) {
			for (j = 0; j < 3; j++) {
				do {

					bRet = wacom_i2c_get_feature(wac_i2c,
								     REPORT_ID_2,
								     BOOT_RSP_SIZE,
								     response,
								     COMM_REG,
								     DATA_REG);
					if (!bRet) {
						printk("wacom: %s: failed to set feature\n", __func__);
						return bRet;
					}

					ret = check_progress(&response[1],
							     (BOOT_RSP_SIZE - 3),
							     0x01,
							     ECH_ARRAY[j]);
					if (ret < 0) {
						printk("wacom: addr: %x res:%x \n",
						       (unsigned int)ulAddress,
						       response[1]);
						return false;
					}
				} while (ret == PROCESS_INPROGRESS);
			}
		}
	}

	return true;
}

static bool wacom_i2c_flash_w9021(struct wacom_i2c *wac_i2c,
				  unsigned char *data)
{
	bool bRet = false;
	int iBLVer = 0, iMpuType = 0;
	unsigned long max_address = W9021_END_ADDR;             /* Max.address of Load data */
	unsigned long start_address = W9021_START_ADDR;         /* Start.address of Load data */

	/*Obtain boot loader version*/
	if (!flash_blver_w9021(wac_i2c, &iBLVer)) {
		printk("wacom: %s: failed to get BL version\n", __func__);
		return false;
	}
	printk("wacom: BL version: %x\n", iBLVer);

	/*Obtain MPUtype: this can be manually done in user space*/
	if (!flash_mputype_w9021(wac_i2c, &iMpuType)) {
		printk("wacom: %s: failed to get MPU type \n", __func__);
		return false;
	}

	if (iMpuType != MPU_W9021) {
		printk("wacom: MPU is not for W9021 : %x\n", iMpuType);
		return false;
	}
	printk("wacom: MPU type: %x\n", iMpuType);

	/*-----------------------------------*/
	/*Flashing operation starts from here*/

	/*Erase the current loaded program*/
	printk("wacom: %s erasing the current firmware\n", __func__);
	bRet = flash_erase_all(wac_i2c);
	if (!bRet) {
		printk("wacom: %s: failed to erase user program\n", __func__);
		return bRet;
	}

	/*Write the new program*/
	printk("wacom: %s: writing new firmware \n", __func__);
	bRet = flash_write_w9021(wac_i2c, data, start_address, &max_address);
	if (!bRet) {
		printk("wacom: %s: failed to write firmware \n", __func__);
		return bRet;
	}

	printk("wacom: %s: write and verify completed \n", __func__);

	return true;
}

static bool flash_end_w9021(struct wacom_i2c *wac_i2c)
{
        bool bRet = false;
        u8 command[BOOT_CMD_SIZE] = {0};
        int len = 0;

        command[len++] = BOOT_CMD_REPORT_ID;
        command[len++] = BOOT_EXIT;
        command[len++] = 0;

        bRet = wacom_i2c_set_feature(wac_i2c, REPORT_ID_1, len,
				     command, COMM_REG, DATA_REG);
        if (!bRet) {
                printk("wacom: %s: failed to set feature\n", __func__);
                return bRet;
        }

        return true;
}

static bool do_update(struct wacom_i2c *wac_i2c, u8 *data)
{
	bool bRet = false;

	wacom_i2c_flash_cmd(wac_i2c);

	bRet = flash_query_w9021(wac_i2c);
	if (!bRet) {
		printk("wacom: %s: cannot send query\n", __func__);
		goto err;
	}

	bRet = wacom_i2c_flash_w9021(wac_i2c, data);
	if (!bRet) {
		printk("wacom: %s: flash failed \n", __func__);
		goto err;
	}

	bRet = true;
err:
	/*Return to the user mode*/
	printk("wacom: %s: closing the boot mode\n", __func__);
	bRet = flash_end_w9021(wac_i2c);
	if (!bRet) {
		printk("wacom: %s: closing boot mode failed\n", __func__);
	}

	return bRet;
}

static int __maybe_unused wacom_i2c_really_update(struct wacom_i2c *wac_i2c,
						  const struct firmware *fw)
{
	struct i2c_client *client = wac_i2c->client;
	struct wacom_features *features = wac_i2c->features;
	HID_DESC hid_desc = { 0 };
	int i, ret = 0, cnt;
	long version = 0;
	char ver_str[8] = {0};
	u8 *flash_data = NULL;
	unsigned long max_address = 0;

	if (!fw || (fw->size < 5) || (fw->data[0] != '!')) {
		dev_err(&client->dev, "No validate firmware found!\n");
		goto done;
	}

	for (i = 0; i < 4; i++)
		ver_str[i] = fw->data[i + 1];
	ret = kstrtol(ver_str, 16, &version);
	if (ret) {
		dev_err(&client->dev,
			"Convert version failed,%04X\n", (unsigned int)version);
		goto done;
	}

	dev_info(&client->dev,
		 "current version: %04X, new version: %04X\n",
		 features->fw_version, (unsigned int)version);

	if ((unsigned int)version == (unsigned int)(features->fw_version))
		goto done;

	flash_data = kmalloc(FLASH_DATA_SIZE, GFP_KERNEL);
	if (!flash_data)
		goto done;
	memset(flash_data, 0xff, FLASH_DATA_SIZE);
	cnt = wacom_i2c_read_hex(fw, flash_data, FLASH_DATA_SIZE, &max_address);
	dev_info(&client->dev, "firmware size: %d bytes\n", cnt);
	if (cnt <= 0)
		goto done;
	do_update(wac_i2c, flash_data);

	msleep(100);
	/* power down */
	gpio_set_value(wac_i2c->reset_gpio, 0);
	gpio_set_value(wac_i2c->pwren_gpio, 0);
	msleep(200);
	/* power up */
	gpio_set_value(wac_i2c->pwren_gpio, 1);
	msleep(100);
	gpio_set_value(wac_i2c->reset_gpio, 0);
	msleep(150);
	gpio_set_value(wac_i2c->reset_gpio, 1);
	msleep(50);

	ret = wacom_query_device(client, features);
	if (ret)
		goto done;

	ret = get_hid_desc(client, &hid_desc);
	if (ret)
		goto done;
	wac_i2c->input->id.version = hid_desc.wVersion;
done:
	if (flash_data) {
		kfree(flash_data);
		flash_data = NULL;
	}
	release_firmware(fw);

	return ret;
}

static void __maybe_unused wacom_i2c_update(const struct firmware *fw, void *ctx)
{
	struct wacom_i2c *wac_i2c = ctx;
	struct i2c_client *client = wac_i2c->client;
	char buf[256];
	int ret = 0;

	dev_info(&client->dev, "%s enter.\n", __func__);
	mutex_lock(&wac_i2c->fw_lock);
	if (wac_i2c->fw_done)
		goto err;

	ret = wacom_i2c_really_update(wac_i2c, fw);
	if (ret)
		goto err;

	ret = input_register_device(wac_i2c->input);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register wacom input device, %d\n", ret);
		goto err;
	}

	ret = device_create_file(&client->dev, &dev_attr_fw_version);
	if (ret) {
		dev_err(&client->dev, "Create fw version failed,%d\n", ret);
		goto err_unregister_input;
	}

	memset(buf, 0, sizeof(buf));
	snprintf(buf, sizeof(buf) - 1,
		 "/sys/bus/i2c/devices/%s/fw_version",
		 kobject_name(&client->dev.kobj));
	proc_symlink("ratta/emr_fw_version", NULL, buf);

	wac_i2c->fw_done = true;
	mutex_unlock(&wac_i2c->fw_lock);
	return;

err_unregister_input:
	input_unregister_device(wac_i2c->input);
err:
	mutex_unlock(&wac_i2c->fw_lock);
	return;
}

static int wacom_i2c_notifier_thread(void *data)
{
	struct wacom_i2c *wac_i2c = data;
	struct wacom_event *tmp = NULL;

	while (wac_i2c->active &&
	       (down_interruptible(&wac_i2c->notifier_sema) == 0)) {
		tmp = wacom_i2c_get_event(wac_i2c);
		if (!tmp)
			continue;
		if (tmp->code == 1) {
			raw_notifier_call_chain(&wacom_touch_event_notifier,
						1, NULL);
		} else if (tmp->code == 0) {
			raw_notifier_call_chain(&wacom_touch_event_notifier,
						0, NULL);
		}
		kmem_cache_free(wac_i2c->notifier_cache, tmp);
	}

	wac_i2c->active = false;

	return 0;
}

static void wacom_i2c_create_notifier(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;

	wac_i2c->active = true;
	wac_i2c->notifier = kthread_run(wacom_i2c_notifier_thread,
					wac_i2c, "wacom_notifier");
	if (IS_ERR(wac_i2c->notifier)) {
		dev_err(&client->dev, "create notifier thread failed, %ld\n",
			PTR_ERR(wac_i2c->notifier));
		wac_i2c->notifier = NULL;
	}

	return;
}

static void wacom_i2c_destroy_notifier(struct wacom_i2c *wac_i2c)
{
	wac_i2c->active = false;
	up(&wac_i2c->notifier_sema);
	msleep(10);
}
extern int ratta_touch_ic;
static int wacom_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	static struct wacom_features features = { 0 };
	HID_DESC hid_desc = { 0 };
	struct device_node *wac_np;
	int reset_gpio, irq_gpio = -1, pen_detect_gpio, pwren_gpio;
	int error;

	printk("wacom_i2c_probe\n");
	wac_np = client->dev.of_node;
	if (!wac_np) {
		dev_err(&client->dev, "get device node error\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	if (of_property_read_bool(wac_np, "x-flip"))
		revert_x_flag = 1;
	if (of_property_read_bool(wac_np, "y-flip"))
		revert_y_flag = 1;

	reset_gpio = of_get_named_gpio(wac_np, "gpio_rst", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(&client->dev, "no gpio_rst pin available\n");
		return -EINVAL;
	}

	error = devm_gpio_request_one(&client->dev, reset_gpio,
				      GPIOF_OUT_INIT_HIGH, "gpio-rst");
	if (error < 0) {
		dev_err(&client->dev, "request reset gpio failed,%d\n", error);
		return error;
	}

	pwren_gpio = of_get_named_gpio(wac_np, "gpio_pwren", 0);
	if (!gpio_is_valid(pwren_gpio)) {
		dev_err(&client->dev, "no gpio_pwren pin available\n");
		return -EINVAL;
	}

	error = devm_gpio_request_one(&client->dev, pwren_gpio,
				      GPIOF_OUT_INIT_HIGH, "gpio-pwren");
	if (error < 0) {
		dev_err(&client->dev, "request pwren gpio failed,%d\n", error);
		return error;
	}
	msleep(100);

	gpio_direction_output(reset_gpio, 0);
	msleep(150);
	gpio_direction_output(reset_gpio, 1);
	msleep(50);

	error = wacom_query_device(client, &features);
	if (error) {
		return error;
	}

	error = get_hid_desc(client, &hid_desc);
	if (error)
		return error;

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_pwren_gpio;
	}
	if(ratta_touch_ic== RATTA_TOUCH_IC_FT5XX){
		of_property_read_string(wac_np, "wacom,fw_name_ft", &wac_i2c->fw_name);
	}else{
		of_property_read_string(wac_np, "wacom,fw_name", &wac_i2c->fw_name);
	}
	pen_detect_gpio = of_get_named_gpio(wac_np, "gpio_detect", 0);
	if (!gpio_is_valid(pen_detect_gpio)) {
		dev_err(&client->dev, "no pen_detect_gpio pin available\n");
		goto err_free_mem;
	}
	error = devm_gpio_request_one(&client->dev, pen_detect_gpio, GPIOF_IN, "gpio_detect");
	if (error < 0) {
		dev_err(&client->dev, "request gpio detect failed,%d\n", error);
		goto err_free_mem;
	}

	irq_gpio = of_get_named_gpio(wac_np, "gpio_intr", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(&client->dev, "no gpio_intr pin available\n");
		goto err_free_pen_detect_gpio;
	}
	g_irq_gpio = irq_gpio;
#if 1
	error = devm_gpio_request_one(&client->dev, irq_gpio, GPIOF_IN, "gpio_intr");
	if (error < 0) {
		dev_err(&client->dev, "request intr gpio failed,%d\n", error);
		goto err_free_pen_detect_gpio;
	}
#endif
	client->irq = gpio_to_irq(irq_gpio);
	//printk("wacom_i2c_probe irq=%d, irq_gpio=%d\n",client->irq, irq_gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", irq_gpio, client->irq);
		goto err_free_irq_gpio;
	}
	wac_i2c->pwren_gpio = pwren_gpio;
	wac_i2c->reset_gpio = reset_gpio;
    wac_i2c->irq_gpio = irq_gpio;
	wac_i2c->features = &features;
	wac_i2c->client = client;
	wac_i2c->input = input;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	//input->id.version = features.fw_version;
	input->id.version = hid_desc.wVersion;
	input->hint_events_per_packet = 8192;

	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, features.pressure_max, 0, 0);
	//INIT_DELAYED_WORK(&wac_i2c->pd_work, wacom_i2c_detect_worker);

	if (features.support.height)
		input_set_abs_params(input, ABS_DISTANCE, 0, features.height_max, 0, 0);
	if (features.support.tilt) {
		input_set_abs_params(input, ABS_TILT_X, -features.tilt_x_max,
				     features.tilt_x_max, 0, 0);
		input_set_abs_params(input, ABS_TILT_Y, -features.tilt_y_max,
				     features.tilt_y_max, 0, 0);
	}

	input_set_drvdata(input, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_irq_gpio;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);
	wac_i2c->irq_enabled = false;
   // wac_i2c->detect_gpio = pen_detect_gpio;


	i2c_set_clientdata(client, wac_i2c);
	wac_i2c->active = true;
	INIT_LIST_HEAD(&wac_i2c->notifier_head);
	spin_lock_init(&wac_i2c->notifier_lock);
	sema_init(&wac_i2c->notifier_sema, 0);
	mutex_init(&wac_i2c->pwr_mutex);
	wac_i2c->notifier_cache = kmem_cache_create("wacom_notifier",
						    sizeof(struct wacom_event),
						    roundup_pow_of_two(sizeof(struct wacom_event)),
						    0, NULL);
	wacom_i2c_create_notifier(wac_i2c);
	wacom_setup_fb_notifier(wac_i2c);

	mutex_init(&wac_i2c->fw_lock);
	request_firmware_nocache(THIS_MODULE,
				 wac_i2c->fw_name ? wac_i2c->fw_name : "wacom.fw",
				 &client->dev, GFP_KERNEL, wac_i2c, wacom_i2c_update);

	return 0;

err_free_irq_gpio:
	devm_gpio_free(&client->dev, irq_gpio);
err_free_pen_detect_gpio:
	devm_gpio_free(&client->dev, pen_detect_gpio);
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);
err_free_pwren_gpio:
	gpio_set_value(pwren_gpio, 0);
	devm_gpio_free(&client->dev, pwren_gpio);
//err_free_reset_gpio:
	gpio_set_value(pwren_gpio, 0);
	devm_gpio_free(&client->dev, reset_gpio);

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	wacom_i2c_destroy_notifier(wac_i2c);
	if (wac_i2c->notifier_cache) {
		kmem_cache_destroy(wac_i2c->notifier_cache);
		wac_i2c->notifier_cache = NULL;
	}
	device_remove_file(&client->dev, &dev_attr_fw_version);
	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);

	return 0;
}

static int __maybe_unused wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	printk("wacom_i2c_suspend fb_power_off:%d\n",fb_power_off());
	mutex_lock(&wac_i2c->pwr_mutex);
	if(fb_power_off()){
	}else{
		//if ((get_suspend_state() == PM_SUSPEND_IDLE) &&
		//    ratta_has_idle()) {
			if (!irqd_is_wakeup_set(irq_get_irq_data(client->irq))) {
				enable_irq_wake(client->irq);
				//wac_i2c->irq_enabled = true;
				dev_info(&client->dev, "enable irq wake.\n");
			}
			mutex_unlock(&wac_i2c->pwr_mutex);
			return 0;
		//}
	}
//	disable_irq(client->irq);
//	wac_i2c->irq_enabled = false;
	wacom_power_switch(wac_i2c,false);
	mutex_unlock(&wac_i2c->pwr_mutex);

	return 0;
}

static int __maybe_unused wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	printk("wacom_i2c_resume fb_power_off:%d\n",fb_power_off());

	mutex_lock(&wac_i2c->pwr_mutex);
	if(fb_power_off()){
	}else{

		//if ((get_suspend_state() == PM_SUSPEND_IDLE) &&
		 //   ratta_has_idle()) {
			if (irqd_is_wakeup_set(irq_get_irq_data(client->irq))) {
				disable_irq_wake(client->irq);
				//wac_i2c->irq_enabled = false;
				dev_info(&client->dev, "disable irq wake.\n");
			}
			mutex_unlock(&wac_i2c->pwr_mutex);
			return 0;
		//}
	}
	wacom_power_switch(wac_i2c,true);
//	enable_irq(client->irq);
//	wac_i2c->irq_enabled = true;
	mutex_unlock(&wac_i2c->pwr_mutex);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "wacom", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static const struct of_device_id wacom_dt_ids[] = {
	{
		.compatible = "wacom,w9013",
		.data = (void *) &wacom_i2c_id[0],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, wacom_dt_ids);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom",
		.owner	= THIS_MODULE,
		.of_match_table = wacom_dt_ids,
		.pm	= &wacom_i2c_pm,
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};

static int __init wacom_init(void)
{
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

/*
 * Module entry points
 */
subsys_initcall(wacom_init);
//late_initcall(wacom_init);
module_exit(wacom_exit);

//module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
