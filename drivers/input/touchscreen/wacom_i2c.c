
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

#include <linux/notifier.h>
#include <linux/fb.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#include <linux/pm_wakeirq.h>
#include <linux/htfy_dbg.h>  // 20191031,hsl add.20191107--for function-call and  fb_power_off.


int wacom_touch_action_down = 0;
static RAW_NOTIFIER_HEAD(wacom_touch_event_notifier);
int register_wacom_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_register(&wacom_touch_event_notifier, nb);
}
int unregister_wacom_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_unregister(&wacom_touch_event_notifier, nb);
}
int wacom_touch_event_notifier_call_chain(unsigned long val, void *v) {
    return raw_notifier_call_chain(&wacom_touch_event_notifier, val, v);
}
EXPORT_SYMBOL(register_wacom_touch_event_notifier);
EXPORT_SYMBOL(unregister_wacom_touch_event_notifier);


// 20191107: hsl add for eink.
#if 0
static touch_detect		wacom_detect_fun = NULL;
static touch_input		wacom_input_fun = NULL;
void register_touch_detect(touch_detect det_fun)
{
	wacom_detect_fun = det_fun;
}
void register_touch_input(touch_input input_fun)
{
	wacom_input_fun = input_fun;
}
#else
extern void ebc_set_tp_power(/*struct input_dev *dev,*/bool pen_on); // 20200716,hsl add.
#endif 

#if 0
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
#endif  // #if 0

int screen_max_x = 20280;
int screen_max_y = 13942;

#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	19

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	char fw_version;
};

/*HID specific register*/
#define HID_DESC_REGISTER       1
#define COMM_REG                0x04
#define DATA_REG                0x05

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

struct wacom_i2c {
	struct wacom_features *features;
	struct i2c_client *client;
	struct input_dev *input;
	u8 data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;

	// 2019107,hsl add.
	bool 	irq_enabled;
	bool 	suspended;
	int 	suspend_irq_cnt;
	int 	irq_gpio;
	int 	detect_gpio;
	int 	detect_irq;
	int 	reset_gpio;
	int 	power18_gpio;
	struct delayed_work pd_work; // pen detect work to rebound gpio.
	struct notifier_block	fb_notify;
	struct regulator 	*vdd_regulator;
	int revert_x_flag;
    int revert_y_flag;
    int exchange_x_y_flag;
};

// 20191209: if set to 0, we need set regulator-always-on; at DTS FOR vdd1v5_dvp.
#define  CTRL_REGULATOR 	1
#if CTRL_REGULATOR
static void wacom_power_switch(struct wacom_i2c *wacom, bool keep_on)
{
	if(wacom->vdd_regulator) {
		int ret = 0;
		printk("%s on_off:%d \n", __func__, keep_on);
		if(keep_on) {
			if (gpio_is_valid(wacom->power18_gpio)) {
				gpio_set_value(wacom->power18_gpio, 1);
				printk("%s power18_gpio on \n", __func__);
			}
			if(CTRL_REGULATOR) ret = regulator_enable(wacom->vdd_regulator);
			gpio_set_value(wacom->reset_gpio, 1);

			gpio_direction_input(wacom->irq_gpio);
			gpio_direction_input(wacom->detect_gpio);
			printk("%s:regulater on wacom,ret=%d\n", __func__, ret);
		} else {
			if (gpio_is_valid(wacom->power18_gpio)) {
				gpio_set_value(wacom->power18_gpio, 0);
				printk("%s power18_gpio off \n", __func__);
			}
			if(CTRL_REGULATOR) ret = regulator_disable(wacom->vdd_regulator);

			gpio_set_value(wacom->reset_gpio, 0);

			gpio_direction_output(wacom->irq_gpio, 0);
			gpio_direction_output(wacom->detect_gpio, 0);
			printk("%s:regulater off wacom,ret=%d\n", __func__, ret);
		}
		if( ret ) {
			printk("%s: failed, powr_on=%d,ret=%d\n", __func__, keep_on, ret);
		}
	}
}
#else
#define wacom_power_switch(wacom,on)
#endif

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
	dbg_printk(IDBG_WACOM,"wacom_fb_notifier: event=%ld, blank=%d\n", event, blank); //dbg_printk(IDBG_WACOM,

	if (blank == FB_BLANK_UNBLANK) {
		if(!cd->irq_enabled) {
			// 20191207: fb 处于正常状态.
			wacom_power_switch(cd, true);
			//enable_irq_wake(cd->detect_irq); // 20191207,for balance
			enable_irq(cd->client->irq);
			enable_irq(cd->detect_irq);
			cd->irq_enabled = true;
		}
	} else if (blank == FB_BLANK_POWERDOWN) {
		// 20191207: fb进入待机，待机界面显示出来了.
		if(cd->irq_enabled) {
			cd->irq_enabled = false;
			disable_irq(cd->client->irq);
			disable_irq(cd->detect_irq);
			//disable_irq_wake(cd->detect_irq);
			//msleep(5);
			wacom_power_switch(cd, false);
		}
	}
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


static int wacom_query_device(struct wacom_i2c *wac_i2c,struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
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
	if (ret < 0) {
		dev_err(&client->dev, "i2c_transfer msg Failed,ret=%d\n", ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "i2c_transfer msg ret=%d,msgs Size=%d\n", ret, (int)ARRAY_SIZE(msgs));
		return -EIO;
	}
	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	printk("Wacom source screen x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);

	if (1 == wac_i2c->exchange_x_y_flag) {
		swap(features->x_max, features->y_max);
	}
	screen_max_x = features->x_max;
	screen_max_y = features->y_max;
	printk("Wacom desc screen x_max:%d, y_max:%d\n", features->x_max, features->y_max);

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

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, sizeof(wac_i2c->data));
	if (error < 0)
		goto out;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	//printk("%s:x=%d,y=%d,rvt_y=%d,max x=%d,max y=%d\n", __func__, x, y, revert_y_flag, screen_max_x, screen_max_y);
	if (1 == wac_i2c->exchange_x_y_flag) {
		swap(x, y);
	}
	if (1 == wac_i2c->revert_x_flag) {
		x = screen_max_x - x;
	}
	if (1 == wac_i2c->revert_y_flag) {
		y = screen_max_y - y;
	}

	#if 0  //20191207-hsl,we don't need call back.
	if (pressure != 0 || (wac_i2c->tool == BTN_TOOL_PEN && wac_i2c->prox == 0x1)) {
		if (wacom_touch_action_down == 0) {
			wacom_touch_action_down = 1;
			if (wacom_touch_event_notifier_call_chain(1, NULL)) {
				printk("wacom_touch_event_notifier_call_chain error\n");
			}
		}
	} else {
		if (wacom_touch_action_down == 1) {
			wacom_touch_action_down = 0;
			if (wacom_touch_event_notifier_call_chain(0, NULL)) {
				printk("wacom_touch_event_notifier_call_chain error\n");
			}
		}
	}
	#endif
	if(wac_i2c->suspended) {
		wac_i2c->suspend_irq_cnt++;
		if( wac_i2c->suspend_irq_cnt > 5 && wac_i2c->suspend_irq_cnt < 8 && !pressure) {
			// wacom_irq:x=838,y=7530,press=1480,tsw=1,suspended=0
			printk("wacom_irq:fix suspend touch:press=%d,tsw=%x,cnt=%d\n", pressure,
				tsw, wac_i2c->suspend_irq_cnt);
			pressure = 1480;
			tsw = 1;
		}

		// 20191208: drop some msg until resume.or we may lost the valid input.
		else if( wac_i2c->suspend_irq_cnt > 10) {
			printk("wacom_irq:drop suspend touch:press=%d,tsw=%x,cnt=%d\n", pressure,
				tsw, wac_i2c->suspend_irq_cnt);
			goto out;
		}
	}

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_sync(input);


	 // 20191014,hsl add.  dbg_printk(IDBG_WACOM,
	 // *EDBG*wacom_irq:x=15914,y=12419,press=0,f1=0x0,f2=0x0,prox=0x1,tsw=0,ers=0,tool=0x140
	 // *EDBG*wacom_irq:x=15914,y=12418,press=3256,f1=0x0,f2=0x0,prox=0x1,tsw=1,ers=0,tool=0x140
	dbg_printk(IDBG_WACOM,"wacom_irq:x=%d,y=%d,press=%d,tsw=%x,suspended=%d,cnt=%d\n", x, y, pressure,
		tsw, wac_i2c->suspended, wac_i2c->suspend_irq_cnt);
	// 20191107,hsl add.
	/*if( wacom_input_fun != NULL && tsw) {
		wacom_input_fun(input, x, y, pressure, true);
	}*/

out:
	return IRQ_HANDLED;
}

#if 0
static irqreturn_t wacom_i2c_hard_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	printk("wacom_i2c_hard_irq: suspened=%d\n", wac_i2c->suspended);
	return IRQ_WAKE_THREAD;
}
#endif

static void wacom_i2c_detect_worker(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c = container_of(work,
			struct wacom_i2c, pd_work.work);
	//struct input_dev *input = wac_i2c->input;

    int pendet_gpio_value = gpio_get_value(wac_i2c->detect_gpio);
	bool penOn = (pendet_gpio_value == 0)?true:false;

	// 20191107,hsl add.for power-on eink before writing.
	#if 0
	if( wacom_detect_fun != NULL) {
		wacom_detect_fun(/*input, */penOn);
	}
	#else
	ebc_set_tp_power(penOn);
	#endif
}

static irqreturn_t wacom_i2c_detect_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	// 20191107-LOG:*EDBG*wacom_detect_irq: gpio[13]=0,det_fun=          (null)
	dbg_printk(IDBG_WACOM,"wacom_detect_irq: gpio[%d]=%d\n", wac_i2c->detect_gpio,
		gpio_get_value(wac_i2c->detect_gpio));  // , wacom_detect_fun
	schedule_delayed_work(&wac_i2c->pd_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{

	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	//wacom_power_switch(wac_i2c, true);
	//msleep(5);
	//enable_irq(client->irq);
	//enable_irq(wac_i2c->detect_irq);
	dbg_printk(IDBG_WACOM,"wacom_i2c_open: irq=%d\n", client->irq);
	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	//disable_irq(client->irq);
	//disable_irq(wac_i2c->detect_irq);
	dbg_printk(IDBG_WACOM,"wacom_i2c_close: irq=%d\n", client->irq);
}

//static int g_irq_gpio = -1;
static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct wacom_features features = { 0 };
	HID_DESC hid_desc = { 0 };
	struct device_node *wac_np;
	int reset_gpio, irq_gpio = -1, pen_detect_gpio,gpio_power18;
	int error,ret;
	u32 val;
	printk("----wacom_i2c_probe,dev name=%s\n", dev_name(&client->dev));

	wac_np = client->dev.of_node;
	if (!wac_np) {
		dev_err(&client->dev, "get device node error\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		dev_err(&client->dev, "input_allocate_device or alloc wac_i2c failed\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	gpio_power18 = of_get_named_gpio(wac_np, "gpio_power18", 0);
	printk("gpio_power18 is:%d \n",gpio_power18);
	if (!gpio_is_valid(gpio_power18)) {
		dev_err(&client->dev, "no gpio_power18 pin available\n");
	//	goto err_free_mem;
	}else{
		error = devm_gpio_request_one(&client->dev, gpio_power18, GPIOF_OUT_INIT_HIGH, "gpio-pw18");
		if (error < 0) {
			dev_err(&client->dev, "devm_gpio_request_one gpio_power18 Failed\n");
			goto err_free_mem;
		}
		gpio_direction_output(gpio_power18, 1);
		msleep(10);
		printk("wacom_i2c_probe,gpio_power18 is:%d \n",gpio_power18);
	}

#if CTRL_REGULATOR
		wac_i2c->vdd_regulator = devm_regulator_get(&client->dev, "power"); //devm_regulator_get_optional
		if (IS_ERR(wac_i2c->vdd_regulator) /*== -ENODEV*/) {
			error = PTR_ERR(wac_i2c->vdd_regulator);
			dev_err(&client->dev, "power not specified for wacom,ignore power ctrl,err=%d\n", error);
			wac_i2c->vdd_regulator = NULL;
		} else {
			ret = regulator_enable(wac_i2c->vdd_regulator);
			printk("wacom_i2c_probe,vdd_regulator enable \n");
		}
#else
		wac_i2c->vdd_regulator = NULL;
#endif

	reset_gpio = of_get_named_gpio(wac_np, "gpio_rst", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(&client->dev, "no gpio_rst pin available\n");
		goto err_free_mem;
	}

	error = devm_gpio_request_one(&client->dev, reset_gpio, GPIOF_OUT_INIT_LOW, "gpio-rst");
	if (error < 0) {
		dev_err(&client->dev, "devm_gpio_request_one gpio_rst Failed\n");
		goto err_free_mem;
	}
	gpio_direction_output(reset_gpio, 0);
	msleep(100);
	gpio_direction_output(reset_gpio, 1);

	pen_detect_gpio = of_get_named_gpio(wac_np, "gpio_detect", 0);
	if (!gpio_is_valid(pen_detect_gpio)) {
		dev_err(&client->dev, "no pen_detect_gpio pin available\n");
		goto err_free_reset_gpio;
	}
	error = devm_gpio_request_one(&client->dev, pen_detect_gpio, GPIOF_IN, "gpio_detect");
	if (error < 0) {
		dev_err(&client->dev, "devm_gpio_request_one gpio_detect Failed\n");
		goto err_free_reset_gpio;
	}

	irq_gpio = of_get_named_gpio(wac_np, "gpio_intr", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(&client->dev, "no gpio_intr pin available\n");
		goto err_free_pen_detect_gpio;
	}

	wac_i2c->revert_x_flag = 0;
    if (!of_property_read_u32(wac_np, "revert_x_flag", &val)){
        wac_i2c->revert_x_flag = !!val;
    }
    wac_i2c->revert_y_flag = 0;
    if (!of_property_read_u32(wac_np, "revert_y_flag", &val)){
        wac_i2c->revert_y_flag = !!val;
    }
    wac_i2c->exchange_x_y_flag = 0;
    if (!of_property_read_u32(wac_np, "exchange_x_y_flag", &val)){
        wac_i2c->exchange_x_y_flag = !!val;
    }
	//g_irq_gpio = irq_gpio;
	msleep(100);


	//wac_i2c->client = client;

	error = wacom_query_device(wac_i2c,client, &features);
	if (error) {
		printk("wacom_query_device error");
		if (gpio_is_valid(gpio_power18)) {
			gpio_direction_output(gpio_power18, 0);
			//gpio_free(gpio_power18);
		}
		return error;
	}

	error = get_hid_desc(client, &hid_desc);
	if (error) {
				printk("get_hid_desc error");
		if (gpio_is_valid(gpio_power18)) {
			gpio_direction_output(gpio_power18, 0);
			//gpio_free(gpio_power18);
		}
		return error;
	}
#if 1
	error = devm_gpio_request_one(&client->dev, irq_gpio, GPIOF_IN, "gpio_intr");
	if (error < 0) {
		dev_err(&client->dev, "devm_gpio_request_one gpio_intr Failed\n");
		goto err_free_pen_detect_gpio;
	}
#endif
	client->irq = gpio_to_irq(irq_gpio);
	//printk("wacom_i2c_probe irq=%d, irq_gpio=%d\n",client->irq, irq_gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", irq_gpio, client->irq);
		goto err_free_irq_gpio;
	}

	wac_i2c->features = &features;
	wac_i2c->client = client;
	wac_i2c->input = input;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;

	// 20210804: 我们改为用标准的 WACOM 的 vid. native-手写的 demo 上面需要判断这个vid来过滤。
	input->id.vendor = 0x2d1f; // hid_desc.wVendorID; // 0x56a;
	input->id.product = hid_desc.wProductID;//0x0120; // 
	
	//input->id.version = features.fw_version;
	input->id.version = hid_desc.wVersion;

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

	// 20200227:我们需要提前准保好数据再注册中断，否则有可能 request_threaded_irq
	// 之后马上进入 wacom_i2c_detect_irq ，导致出现data异常。--概率性无法开机。
	input_set_drvdata(input, wac_i2c);
	wac_i2c->reset_gpio = reset_gpio; //20191121,need output low when sleep.
	wac_i2c->irq_enabled = false; // 20191219: default is disabled.
	wac_i2c->irq_gpio = irq_gpio;
	wac_i2c->power18_gpio = gpio_power18;
	INIT_DELAYED_WORK(&wac_i2c->pd_work, wacom_i2c_detect_worker);
	i2c_set_clientdata(client, wac_i2c);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_reset_gpio;
	}

	error = request_threaded_irq(client->irq, NULL /*wacom_i2c_hard_irq*/, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_input;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);
	wac_i2c->irq_enabled = false;

	// 2019107:register the detect irq. when detect-pen, become low.
	wac_i2c->detect_gpio = pen_detect_gpio;
	wac_i2c->detect_irq = gpio_to_irq(pen_detect_gpio);
	error = request_threaded_irq(wac_i2c->detect_irq, NULL, wacom_i2c_detect_irq,
				     //IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     "wacom-det", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_irq;
	}

	disable_irq(wac_i2c->detect_irq);

	wacom_setup_fb_notifier(wac_i2c);
	dev_err(&client->dev, "probe Ok,irq gpio=%d,irq=%d,reset_gpio=%d,vdd_regulator=%p\n", wac_i2c->irq_gpio,
		client->irq, wac_i2c->reset_gpio, wac_i2c->vdd_regulator);
	return 0;

err_free_irq:
	free_irq(client->irq, wac_i2c);

err_free_input:
	input_free_device(input);

err_free_reset_gpio:
	gpio_free(reset_gpio);
err_free_pen_detect_gpio:
	gpio_free(pen_detect_gpio);
err_free_irq_gpio:
	gpio_free(irq_gpio);
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);

	if (gpio_is_valid(gpio_power18)) {
		gpio_direction_output(gpio_power18, 0);
		gpio_free(gpio_power18);
	}

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, wac_i2c);
	if (gpio_is_valid(wac_i2c->power18_gpio)) {
		gpio_set_value(wac_i2c->power18_gpio, 0);
	}
	input_unregister_device(wac_i2c->input);
	printk("%s:pw18 gpio level=%d \n", __func__,gpio_get_value(wac_i2c->power18_gpio));

	kfree(wac_i2c);

	return 0;
}

static int __maybe_unused wacom_i2c_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

#if 1  // 20191207: we do this at fb-notify.
	if(fb_power_off()) {
		//disable_irq(wac_i2c->detect_irq);
		//disable_irq(client->irq);
		gpio_set_value(wac_i2c->reset_gpio, 0);
		if (gpio_is_valid(wac_i2c->power18_gpio)) {
			gpio_set_value(wac_i2c->power18_gpio, 0);
		}
		//wacom_power_switch(wac_i2c, false);
	} else {
		// do nothing.let the wacom continue working. if (device_may_wakeup(dev))
		enable_irq_wake(wac_i2c->detect_irq);
	}
#endif
	wac_i2c->suspended = true;
	wac_i2c->suspend_irq_cnt = 0;

	// 20200428: power18_gpio maybe -2(Not Valid), so don't printk it.
	printk("%s:reset gpio level=%d,fb_power_off=%d\n", __func__, gpio_get_value(wac_i2c->reset_gpio),fb_power_off());
	return 0;
}

static int __maybe_unused wacom_i2c_resume_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	wac_i2c->suspended = false;
	printk("%s:reset gpio level=%d,fb_power_off=%d\n", __func__, gpio_get_value(wac_i2c->reset_gpio),
		fb_power_off());

#if 1  // 20191207: we do this at fb-notify.
	if(fb_power_off()) {
		//wacom_power_switch(wac_i2c, true);
		if (gpio_is_valid(wac_i2c->power18_gpio)) {
			gpio_set_value(wac_i2c->power18_gpio, 1);
		}
		gpio_set_value(wac_i2c->reset_gpio, 1);
		//enable_irq(wac_i2c->detect_irq);
		//enable_irq(client->irq);
	} else {
		disable_irq_wake(wac_i2c->detect_irq);
	}
#endif
	return 0;
}

//static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

const static struct dev_pm_ops wacom_i2c_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(wacom_i2c_suspend_noirq,
				      wacom_i2c_resume_noirq)
};

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
 * 20191224: we need init after RK808,cause we need to get regulator.
 */
//subsys_initcall(wacom_init);
device_initcall_sync(wacom_init);
module_exit(wacom_exit);

//module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
