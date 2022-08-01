
/*
 * hanvon Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, hanvon.
 * <tobita.tatsunosuke@hanvon.co.jp>
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

static int dbg_enable=0;
module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG_HV(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)


int hanvon_touch_action_down = 0;
static RAW_NOTIFIER_HEAD(hanvon_touch_event_notifier);
int register_hanvon_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_register(&hanvon_touch_event_notifier, nb);
}
int unregister_hanvon_touch_event_notifier(struct notifier_block *nb) {
    return raw_notifier_chain_unregister(&hanvon_touch_event_notifier, nb);
}
int hanvon_touch_event_notifier_call_chain(unsigned long val, void *v) {
    return raw_notifier_call_chain(&hanvon_touch_event_notifier, val, v);
}
EXPORT_SYMBOL(register_hanvon_touch_event_notifier);
EXPORT_SYMBOL(unregister_hanvon_touch_event_notifier);


// 20191107: hsl add for eink.
static touch_detect		hanvon_detect_fun = NULL;
static touch_input		hanvon_input_fun = NULL;
void register_touch_detect(touch_detect det_fun)
{
	hanvon_detect_fun = det_fun;
}
void register_touch_input(touch_input input_fun)
{
	hanvon_input_fun = input_fun;
}
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
#define MAX_EVENTS					10
#define SCR_X						1280
#define SCR_Y						800
#define MAX_X						0x27de
#define MAX_Y						0x1cfe
#define MAX_PRESSURE				1024

#define MAX_PACKET_SIZE             7


int screen_max_x = 20280;
int screen_max_y = 13942;

#define HW0868_CMD_RESET				0x08680000
#define HW0868_CMD_CONFIG_HIGH				0x08680001
#define HW0868_CMD_CONFIG_LOW				0x08680002
#define HW0868_CMD_UPDATE				0x08680003
#define HW0868_CMD_GET_VERSION				0x08680004
#define HW0868_CMD_CALIBRATE				0x08680005

/* define pen flags, 7-bytes protocal. */
#define PEN_POINTER_UP					0xa0
#define PEN_POINTER_DOWN				0xa1
#define PEN_BUTTON_UP					0xa2
#define PEN_BUTTON_DOWN					0xa3
#define PEN_RUBBER_UP					0xa4
#define PEN_RUBBER_DOWN					0xa5
#define PEN_ALL_LEAVE					0xe0
static volatile int isPenDetected = 0;

/* DEBUG micro, for user interface. */

/* version number buffer */
static unsigned char ver_info[9] = {0};
static int ver_size = 9;
bool get_version;

struct hanvon_data {
	int x;
	int y;
	int pressure;
	char fw_version;
	u8 	flag;
};

/*HID specific register*/
//#define HID_DESC_REGISTER       1
#define COMM_REG                0x04
#define DATA_REG                0x05


struct hanvon_i2c {
	struct hanvon_data *data;
	struct i2c_client *client;
	struct input_dev *input;
	//u8 data[hanvon_QUERY_SIZE];
	bool prox;
	int tool;

	// 2019107,hsl add.
	bool 	irq_enabled;
	bool 	suspended;
	int 	suspend_irq_cnt;
	int 	irq_gpio;
	int 	reset_gpio;
	int     gpio_reset_active_flag;
	int 	power18_gpio;
	struct notifier_block	fb_notify;
	struct regulator 	*vdd_regulator;
	int revert_x_flag;
    int revert_y_flag;
    int exchange_x_y_flag;
	struct workqueue_struct *hw_wq;
	struct work_struct work_irq;
	struct mutex mutex_wq;
};

// 20191209: if set to 0, we need set regulator-always-on; at DTS FOR vdd1v5_dvp.
#define  CTRL_REGULATOR 	1
#if CTRL_REGULATOR
static void hanvon_power_switch(struct hanvon_i2c *hanvon, bool keep_on)
{
	if(hanvon->vdd_regulator) {
		int ret = 0;
		printk("%s on_off:%d \n", __func__, keep_on);
		if(keep_on) {
			if (gpio_is_valid(hanvon->power18_gpio)) {
				gpio_set_value(hanvon->power18_gpio, 1);
				printk("%s power18_gpio on \n", __func__);
			}
			if(CTRL_REGULATOR) ret = regulator_enable(hanvon->vdd_regulator);
			//gpio_set_value(hanvon->reset_gpio, 1);
			gpio_direction_output(hanvon->reset_gpio, !hanvon->gpio_reset_active_flag);
			msleep(150);

			gpio_direction_input(hanvon->irq_gpio);
			printk("%s:regulater on hanvon,ret=%d\n", __func__, ret);
		} else {
			if (gpio_is_valid(hanvon->power18_gpio)) {
				gpio_set_value(hanvon->power18_gpio, 0);
				printk("%s power18_gpio off \n", __func__);
			}
			if(CTRL_REGULATOR) ret = regulator_disable(hanvon->vdd_regulator);

			//gpio_set_value(hanvon->reset_gpio, 0);
			gpio_direction_output(hanvon->reset_gpio, hanvon->gpio_reset_active_flag);
			gpio_direction_output(hanvon->irq_gpio, 0);
			printk("%s:regulater off hanvon,ret=%d\n", __func__, ret);
		}
		if( ret ) {
			printk("%s: failed, powr_on=%d,ret=%d\n", __func__, keep_on, ret);
		}
	}
}
#else
#define hanvon_power_switch(hanvon,on)
#endif

static int hanvon_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct hanvon_i2c *cd =
		container_of(self, struct hanvon_i2c, fb_notify);
	struct fb_event *evdata = data;
	int blank;

    // 20180327,the touch only handle FB_EARLY_EVENT_BLANK.
	if (event != FB_EARLY_EVENT_BLANK/*FB_EVENT_BLANK*/ || !evdata)
		goto exit;

    //if( !fb_eink(evdata->info) ){ //20200109,hsl.
    //    return NOTIFY_DONE;
    //}
	blank = *((int*)evdata->data);

	// 20191207-LOG: hanvon_fb_notifier: event=16, blank=0
	DBG_HV("hanvon_fb_notifier: event=%ld, blank=%d\n", event, blank); //DBG_HV(

	if (blank == FB_BLANK_UNBLANK) {
		if(!cd->irq_enabled) {
			// 20191207: fb 处于正常状态.
			hanvon_power_switch(cd, true);
			//enable_irq_wake(cd->detect_irq); // 20191207,for balance
			enable_irq(cd->client->irq);
			cd->irq_enabled = true;
		}
	} else if (blank == FB_BLANK_POWERDOWN) {
		// 20191207: fb进入待机，待机界面显示出来了.
		if(cd->irq_enabled) {
			cd->irq_enabled = false;
			disable_irq(cd->client->irq);
			//disable_irq_wake(cd->detect_irq);
			//msleep(5);
			hanvon_power_switch(cd, false);
		}
	}
exit:
	return 0;
}

static void hanvon_setup_fb_notifier(struct hanvon_i2c *hanvon)
{
	int rc;
	hanvon->fb_notify.notifier_call = hanvon_fb_notifier_callback;
	rc = fb_register_client(&hanvon->fb_notify);
	if (rc) {
		dev_err(&hanvon->client->dev, "Unable to register fb_notifier: %d\n", rc);
		return ;
	}
	device_init_wakeup(&hanvon->client->dev, true);
	//disable_irq_wake(hanvon->detect_irq); // 20191207,for balance
	//disable_irq_wake(hanvon->client->irq); // 20191207,for balance
	//printk("hanvon_setup_fb_notifier: enable_irq_wake %d!\n", hanvon->detect_irq);
}

static struct hanvon_data hanvon_get_packet(struct hanvon_i2c *hanvon)
{
	struct hanvon_data data = {0};
	struct i2c_client *client = hanvon->client;

	u8 x_buf[MAX_PACKET_SIZE];
	u8 buf[9];
	int count;

	do {
			if(get_version)
			{
				count = i2c_master_recv(client, buf, 9);
				get_version = false;
			}else
			{
				//mdelay(2);
				count = i2c_master_recv(client, x_buf, MAX_PACKET_SIZE);
			}
	}
	while(count == EAGAIN);
	DBG_HV("Reading data. %.2x %.2x %.2x %.2x %.2x %.2x %.2x, count=%d\n", 
				x_buf[0], x_buf[1], x_buf[2], x_buf[3], x_buf[4], 
				x_buf[5], x_buf[6], count);

	if (buf[0] == 0x80)
	{
		printk(KERN_INFO "Get version number ok!\n");
		printk(KERN_INFO "Reading Version. %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x, count=%d\n", 
				buf[0], buf[1], buf[2], buf[3], buf[4], 
				buf[5], buf[6], buf[7], buf[8],count);		
		memcpy(ver_info, buf, ver_size);
	}

	data.flag = x_buf[0];
	data.x |= ((x_buf[1]&0x7f) << 9) | (x_buf[2] << 2) | (x_buf[6] >> 5); // x
	data.y |= ((x_buf[3]&0x7f) << 9) | (x_buf[4] << 2) | ((x_buf[6] >> 3)&0x03); // y
	data.pressure |= ((x_buf[6]&0x07) << 7) | (x_buf[5]);  // pressure

	return data;
}

static int hanvon_report_event(struct hanvon_i2c *hanvon)
{
	struct hanvon_data data = {0};
	struct input_dev *input = hanvon->input;
	data = hanvon_get_packet(hanvon);

	if (data.flag == 0x80)
	{
		return 0;
	}
	if(hanvon->exchange_x_y_flag){
		int t = data.x;
		data.x = data.y;
		data.y = t;
	}
	if(hanvon->revert_x_flag){
		data.x = MAX_X - data.x;
	}
	if(hanvon->revert_y_flag){
		data.y = MAX_Y - data.y;
	}
	
//	hw0868_dbg_coord(KERN_INFO "x=%d\ty=%d\tpressure=%d\tflag=%d\n", data.x, data.y, data.pressure, data.flag);
//	printk(KERN_INFO "x=%d\ty=%d\tpressure=%d\tflag=%d\n", data.x, data.y, data.pressure, data.flag);
	switch(data.flag)
	{
		case PEN_BUTTON_DOWN:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 1);
			input_report_key(input, BTN_TOOL_PEN, 1);
			input_report_key(input, BTN_STYLUS, 1);
			break;
		}
		case PEN_BUTTON_UP:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 0);
			input_report_key(input, BTN_TOOL_PEN, 0);
			input_report_key(input, BTN_STYLUS, 0);
			break;
		}
		case PEN_RUBBER_DOWN:
		{   
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 1);
			input_report_key(input, BTN_TOOL_RUBBER, 1);
			break;
		}
		case PEN_RUBBER_UP:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 0);
			input_report_key(input, BTN_TOOL_RUBBER, 0);
			break;
		}
		case PEN_POINTER_DOWN:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 1);
			input_report_key(input, BTN_TOOL_PEN, 1);
			break;
		}
		case PEN_POINTER_UP:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 0);
			if (isPenDetected == 0)
				input_report_key(input, BTN_TOOL_PEN, 1);
			isPenDetected = 1;
			break;
		}
		case PEN_ALL_LEAVE:
		{
			input_report_abs(input, ABS_X, data.x);
			input_report_abs(input, ABS_Y, data.y);
			input_report_abs(input, ABS_PRESSURE, data.pressure);
			input_report_key(input, BTN_TOUCH, 0);
			input_report_key(input, BTN_TOOL_PEN, 0);
			input_report_key(input, BTN_STYLUS, 0);
			isPenDetected = 0;
			break;
		}
		default:
			printk(KERN_ERR "Hanvon stylus device[0868,I2C]: Invalid input event.\n");
	}
	input_sync(input);
	return 0;
}

static void hanvon_i2c_wq(struct work_struct *work)
{
	struct hanvon_i2c *hanv_i2c = container_of(work, struct hanvon_i2c, work_irq);
	struct i2c_client *client = hanv_i2c->client;

	mutex_lock(&hanv_i2c->mutex_wq);
	hanvon_report_event(hanv_i2c);
	schedule();
	mutex_unlock(&hanv_i2c->mutex_wq);
	enable_irq(client->irq);
}

static irqreturn_t hanvon_i2c_irq(int irq, void *dev_id)
{
	struct hanvon_i2c *hanv_i2c = dev_id;
	//struct input_dev *input = hanv_i2c->input;
	//struct hanvon_data *data = hanv_i2c->data;
	//unsigned int x, y, pressure;
	//unsigned char tsw, f1, f2, ers;
	//int error;


	disable_irq_nosync(irq);
	queue_work(hanv_i2c->hw_wq, &hanv_i2c->work_irq);
	DBG_HV("%s:Interrupt handled.\n", __func__);

	return IRQ_HANDLED;
}

#if 0
static irqreturn_t hanvon_i2c_hard_irq(int irq, void *dev_id)
{
	struct hanvon_i2c *hanv_i2c = dev_id;
	printk("hanvon_i2c_hard_irq: suspened=%d\n", hanv_i2c->suspended);
	return IRQ_WAKE_THREAD;
}


static void hanvon_i2c_detect_worker(struct work_struct *work)
{
	struct hanvon_i2c *hanv_i2c = container_of(work,
			struct hanvon_i2c, pd_work.work);
	struct input_dev *input = hanv_i2c->input;

    int pendet_gpio_value = gpio_get_value(hanv_i2c->detect_gpio);
	bool penOn = (pendet_gpio_value == 0)?true:false;

	// 20191107,hsl add.for power-on eink before writing.
	if( hanvon_detect_fun != NULL) {
		hanvon_detect_fun(input, penOn);
	}
}


static irqreturn_t hanvon_i2c_detect_irq(int irq, void *dev_id)
{
	struct hanvon_i2c *hanv_i2c = dev_id;

	// 20191107-LOG:*EDBG*hanvon_detect_irq: gpio[13]=0,det_fun=          (null)
	DBG_HV("hanvon_detect_irq: gpio[%d]=%d,det_fun=%p\n", hanv_i2c->detect_gpio,
		gpio_get_value(hanv_i2c->detect_gpio), hanvon_detect_fun);
	schedule_delayed_work(&hanv_i2c->pd_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}
#endif

static int hanvon_i2c_open(struct input_dev *dev)
{

	struct hanvon_i2c *hanv_i2c = input_get_drvdata(dev);
	struct i2c_client *client = hanv_i2c->client;

	//hanvon_power_switch(hanv_i2c, true);
	//msleep(5);
	//enable_irq(client->irq);
	//enable_irq(hanv_i2c->detect_irq);
	DBG_HV("hanvon_i2c_open: irq=%d\n", client->irq);
	return 0;
}

static void hanvon_i2c_close(struct input_dev *dev)
{
	struct hanvon_i2c *hanv_i2c = input_get_drvdata(dev);
	struct i2c_client *client = hanv_i2c->client;

	//disable_irq(client->irq);
	//disable_irq(hanv_i2c->detect_irq);
	DBG_HV("hanvon_i2c_close: irq=%d\n", client->irq);
}
#if 1
static int hanvon_get_version(struct i2c_client *client)
{
	int ret = -1;

	unsigned char ver_cmd[] = {0xcd, 0x5f};
	//hw0868_reset();
	printk( "hanvon_get_version\n");
	get_version = true;
	ret = i2c_master_send(client, ver_cmd, 2);
	if (ret < 0)
	{
		get_version = false;
		printk(KERN_INFO "Get version ERROR!\n");
		return ret;
	}

	return ret;
}
#endif
//static int g_irq_gpio = -1;
static int hanvon_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct hanvon_i2c *hanv_i2c;
	struct input_dev *input;
	//struct hanvon_data data = { 0 };
	//HID_DESC hid_desc = { 0 };
	struct device_node *hanv_np;
	int reset_gpio, irq_gpio = -1, gpio_power18;
	int error,ret;
	u32 val;
	enum of_gpio_flags flags;
	printk("----hanvon_i2c_probe222,dev name=%s\n", dev_name(&client->dev));

	hanv_np = client->dev.of_node;
	if (!hanv_np) {
		dev_err(&client->dev, "get device node error\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	hanv_i2c = kzalloc(sizeof(*hanv_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!hanv_i2c || !input) {
		dev_err(&client->dev, "input_allocate_device or alloc hanv_i2c failed\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	gpio_power18 = of_get_named_gpio(hanv_np, "gpio_power18", 0);
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
		printk("hanvon_i2c_probe,gpio_power18 is:%d \n",gpio_power18);
	}

#if CTRL_REGULATOR
		hanv_i2c->vdd_regulator = devm_regulator_get(&client->dev, "power"); //devm_regulator_get_optional
		if (IS_ERR(hanv_i2c->vdd_regulator) /*== -ENODEV*/) {
			error = PTR_ERR(hanv_i2c->vdd_regulator);
			dev_err(&client->dev, "power not specified for hanvon,ignore power ctrl,err=%d\n", error);
			hanv_i2c->vdd_regulator = NULL;
		} else {
			ret = regulator_enable(hanv_i2c->vdd_regulator);
			printk("hanvon_i2c_probe,vdd_regulator enable \n");
		}
#else
		hanv_i2c->vdd_regulator = NULL;
#endif

	reset_gpio = of_get_named_gpio_flags(hanv_np, "gpio_rst", 0, &flags);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(&client->dev, "no gpio_rst pin available\n");
		goto err_free_mem;
	}

	error = gpio_request(reset_gpio, "gpio-rst");//devm_gpio_request_one(&client->dev, reset_gpio, GPIOF_OUT_INIT_LOW, "gpio-rst");
	if (error < 0) {
		dev_err(&client->dev, "devm_gpio_request_one gpio_rst Failed\n");
		goto err_free_mem;
	}
	hanv_i2c->gpio_reset_active_flag = !(flags & OF_GPIO_ACTIVE_LOW);
	gpio_direction_output(reset_gpio, hanv_i2c->gpio_reset_active_flag);
	msleep(100);
	gpio_direction_output(reset_gpio, !hanv_i2c->gpio_reset_active_flag);

	irq_gpio = of_get_named_gpio(hanv_np, "gpio_intr", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(&client->dev, "no gpio_intr pin available\n");
		goto err_free_irq_gpio;
	}

	hanv_i2c->revert_x_flag = 0;
    if (!of_property_read_u32(hanv_np, "revert_x_flag", &val)){
        hanv_i2c->revert_x_flag = !!val;
    }
    hanv_i2c->revert_y_flag = 0;
    if (!of_property_read_u32(hanv_np, "revert_y_flag", &val)){
        hanv_i2c->revert_y_flag = !!val;
    }
    hanv_i2c->exchange_x_y_flag = 0;
    if (!of_property_read_u32(hanv_np, "exchange_x_y_flag", &val)){
        hanv_i2c->exchange_x_y_flag = !!val;
    }
	//g_irq_gpio = irq_gpio;
	msleep(100);


	//hanv_i2c->client = client;

	//error = hanvon_query_device(hanv_i2c,client, &features);
	if (error) {
		printk("hanvon_query_device error");
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
		goto err_free_irq_gpio;
	}
#endif
	client->irq = gpio_to_irq(irq_gpio);
	//printk("hanvon_i2c_probe irq=%d, irq_gpio=%d\n",client->irq, irq_gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", irq_gpio, client->irq);
		goto err_free_irq_gpio;
	}

	//hanv_i2c->features = &features;
	hanv_i2c->client = client;
	hanv_i2c->input = input;

	input->name = "Hanvon electromagnetic pen";
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;
	
	set_bit(EV_ABS, input->evbit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);

	input->dev.parent = &client->dev;
	input->open = hanvon_i2c_open;
	input->close = hanvon_i2c_close;

	if(hanv_i2c->exchange_x_y_flag){
		input_set_abs_params(input, ABS_X, 0, MAX_Y, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, MAX_X, 0, 0);
	}else{
		input_set_abs_params(input, ABS_X, 0, MAX_X, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, MAX_Y, 0, 0);
	}
	input_set_abs_params(input, ABS_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_events_per_packet(input, MAX_EVENTS);

	// 20200227:我们需要提前准保好数据再注册中断，否则有可能 request_threaded_irq
	// 之后马上进入 hanvon_i2c_detect_irq ，导致出现data异常。--概率性无法开机。
	input_set_drvdata(input, hanv_i2c);
	hanv_i2c->reset_gpio = reset_gpio; //20191121,need output low when sleep.
	hanv_i2c->irq_enabled = false; // 20191219: default is disabled.
	hanv_i2c->irq_gpio = irq_gpio;
	hanv_i2c->power18_gpio = gpio_power18;
	//INIT_DELAYED_WORK(&hanv_i2c->pd_work, hanvon_i2c_detect_worker);
	hanv_i2c->hw_wq = create_singlethread_workqueue("hw_wq");
	mutex_init(&hanv_i2c->mutex_wq);
	INIT_WORK(&hanv_i2c->work_irq, hanvon_i2c_wq);
	i2c_set_clientdata(client, hanv_i2c);
	if(hanvon_get_version(client)< 0)
		goto err_free_mem;

	error = input_register_device(hanv_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_reset_gpio;
	}

	error = request_threaded_irq(client->irq, NULL /*hanvon_i2c_hard_irq*/, hanvon_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "hanvon", hanv_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_input;
	}

	/* Disable the IRQ, we'll enable it in hanv_i2c_open() */
	disable_irq(client->irq);
	hanv_i2c->irq_enabled = false;

	// 2019107:register the detect irq. when detect-pen, become low.
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_irq;
	}

	hanvon_setup_fb_notifier(hanv_i2c);
	dev_err(&client->dev, "probe Ok,irq gpio=%d,irq=%d,reset_gpio=%d,vdd_regulator=%p\n", hanv_i2c->irq_gpio,
		client->irq, hanv_i2c->reset_gpio, hanv_i2c->vdd_regulator);
	return 0;

err_free_irq:
	free_irq(client->irq, hanv_i2c);

err_free_input:
	input_free_device(input);

err_free_reset_gpio:
	gpio_free(reset_gpio);
err_free_irq_gpio:
	gpio_free(irq_gpio);
err_free_mem:
	input_free_device(input);
	kfree(hanv_i2c);

	if (gpio_is_valid(gpio_power18)) {
		gpio_direction_output(gpio_power18, 0);
		gpio_free(gpio_power18);
	}

	return error;
}

static int hanvon_i2c_remove(struct i2c_client *client)
{
	struct hanvon_i2c *hanv_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, hanv_i2c);
	if (gpio_is_valid(hanv_i2c->power18_gpio)) {
		gpio_set_value(hanv_i2c->power18_gpio, 0);
	}
	input_unregister_device(hanv_i2c->input);
	printk("%s:pw18 gpio level=%d \n", __func__,gpio_get_value(hanv_i2c->power18_gpio));

	kfree(hanv_i2c);

	return 0;
}

static int __maybe_unused hanvon_i2c_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hanvon_i2c *hanv_i2c = i2c_get_clientdata(client);

#if 1  // 20191207: we do this at fb-notify.
	if(fb_power_off()) {
		//disable_irq(hanv_i2c->detect_irq);
		//disable_irq(client->irq);
		//gpio_set_value(hanv_i2c->reset_gpio, 0);
		
		gpio_direction_output(hanv_i2c->reset_gpio, hanv_i2c->gpio_reset_active_flag);
		if (gpio_is_valid(hanv_i2c->power18_gpio)) {
			gpio_set_value(hanv_i2c->power18_gpio, 0);
		}
		//hanvon_power_switch(hanv_i2c, false);
	} else {
		// do nothing.let the hanvon continue working. if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);
	}
#endif
	hanv_i2c->suspended = true;
	hanv_i2c->suspend_irq_cnt = 0;

	// 20200428: power18_gpio maybe -2(Not Valid), so don't printk it.
	printk("%s:reset gpio level=%d,fb_power_off=%d\n", __func__, gpio_get_value(hanv_i2c->reset_gpio),fb_power_off());
	return 0;
}

static int __maybe_unused hanvon_i2c_resume_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hanvon_i2c *hanv_i2c = i2c_get_clientdata(client);
	hanv_i2c->suspended = false;
	printk("%s:reset gpio level=%d,fb_power_off=%d\n", __func__, gpio_get_value(hanv_i2c->reset_gpio),
		fb_power_off());

#if 1  // 20191207: we do this at fb-notify.
	if(fb_power_off()) {
		//hanvon_power_switch(hanv_i2c, true);
		if (gpio_is_valid(hanv_i2c->power18_gpio)) {
			gpio_set_value(hanv_i2c->power18_gpio, 1);
		}
		//gpio_set_value(hanv_i2c->reset_gpio, 1);
		
		gpio_direction_output(hanv_i2c->reset_gpio, !hanv_i2c->gpio_reset_active_flag);
		//enable_irq(hanv_i2c->detect_irq);
		//enable_irq(client->irq);
	} else {
		disable_irq_wake(client->irq);
	}
#endif
	return 0;
}

//static SIMPLE_DEV_PM_OPS(hanvon_i2c_pm, hanvon_i2c_suspend, hanvon_i2c_resume);

const static struct dev_pm_ops hanvon_i2c_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(hanvon_i2c_suspend_noirq,
				      hanvon_i2c_resume_noirq)
};

static const struct i2c_device_id hanvon_i2c_id[] = {
	{ "hanvon_0868_i2c", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, hanvon_i2c_id);

static const struct of_device_id hanvon_dt_ids[] = {
	{
		.compatible = "hanvon_0868_i2c",
		.data = (void *) &hanvon_i2c_id[0],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, hanvon_dt_ids);

static struct i2c_driver hanvon_i2c_driver = {
	.driver	= {
		.name	= "hanvon",
		.owner	= THIS_MODULE,
		.of_match_table = hanvon_dt_ids,
		.pm	= &hanvon_i2c_pm,
	},

	.probe		= hanvon_i2c_probe,
	.remove		= hanvon_i2c_remove,
	.id_table	= hanvon_i2c_id,
};

static int __init hanvon_init(void)
{
	return i2c_add_driver(&hanvon_i2c_driver);
}

static void __exit hanvon_exit(void)
{
	i2c_del_driver(&hanvon_i2c_driver);
}

/*
 * Module entry points
 * 20191224: we need init after RK808,cause we need to get regulator.
 */
//subsys_initcall(hanvon_init);
device_initcall_sync(hanvon_init);
module_exit(hanvon_exit);

//module_i2c_driver(hanvon_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@hanvon.co.jp>");
MODULE_DESCRIPTION("hanvon EMR I2C Driver");
MODULE_LICENSE("GPL");
