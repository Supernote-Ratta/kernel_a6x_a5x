/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x26_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/irq.h>

#include <linux/notifier.h>
static int home_key_flag = 0;
static int home_key_send = 0;
//extern int register_wacom_touch_event_notifier(struct notifier_block*);
//extern int unregister_wacom_touch_event_notifier(struct notifier_block*);

#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define PRESS_MAX				(255)
#define TP_QUERY_SIZE			(32)

static int screen_max_x 		= 1200;//1404;//1872;
static int screen_max_y 		= 825;//1872;//1440;
static int revert_x_flag 		= 0;
static int revert_y_flag 		= 0;
static int exchange_x_y_flag 	= 0;

#define	HOME_KEY_X			1300
#define	HOME_KEY_Y			1300
#define	HOME_KEY_W			80
#define	HOME_KEY_H			80
#define	HOME_KEY				KEY_HOME

struct ts_event {
	u16		x1;
	u16		y1;
	u16		x2;
	u16		y2;
	u16		x3;
	u16		y3;
	u16		x4;
	u16		y4;
	u16		x5;
	u16		y5;
	u16		pressure;
	s16     touch_ID1;
	s16     touch_ID2;
	s16     touch_ID3;
	s16     touch_ID4;
	s16     touch_ID5;
    u8      touch_point;
};

struct ft5x_ts_data {
	u8 data[TP_QUERY_SIZE];
	struct i2c_client 		*client;
	struct input_dev		*input;
	struct ts_event			event;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

static struct ft5x_ts_data *priv_data;

/*2019-04-18 cw2015 full led control*/
static int cw2015_full_led_gpio = -1;

int cw2015_get_full_led(void)
{
	int io_value = -1;
	if (gpio_is_valid(cw2015_full_led_gpio)) {
		io_value = gpio_get_value(cw2015_full_led_gpio);
		printk("cw2015_get_full_led io_value=%d\n", io_value);
	}
	return io_value;
}
int cw2015_full_led_onoff(int on)
{
	int ret = 0;

	if (gpio_is_valid(cw2015_full_led_gpio)) {
		printk("cw2015_full_led_gpio on=%d\n", on);
		gpio_direction_output(cw2015_full_led_gpio, on);
	}else{
		printk("cw2015_full_led_gpio is invalid\n");
		ret = -1;
	}
	return ret;
}

static int ft5x_i2c_rxdata(struct i2c_client *client, char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		printk("=====ft5x_i2c_rxdata i2c read error: %d\n", ret);
	}
	
	return ret;
}

static int ft5x_detect_tp(struct i2c_client *client)
{
	unsigned char buf[32] = { 0 };
	int ret = -1;
        
	ret = ft5x_i2c_rxdata(client, buf, 31);
	return ret;
}
static void ft5x_ts_release(struct ft5x_ts_data *data)
{
	//printk("=====ft5x_ts_release\n");
	input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 0);
	input_report_key(data->input, BTN_TOUCH, 0);
	if(home_key_flag)
	{
		input_report_key(data->input, HOME_KEY, 0);
		home_key_flag = 0;
		home_key_send = 0;
	}
	input_sync(data->input);
	
	return;
}

static int ft5x_read_data(struct ft5x_ts_data *data)
{
	struct ts_event *event = &data->event;
	unsigned char buf[32] = { 0 };
	int ret = -1;
        
	ret = ft5x_i2c_rxdata(data->client, buf, 31);
	if (ret < 0) {
		printk("=====ft5x_read_data read_data i2c_rxdata failed: %d\n", ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[2] & 0x07;// 000 0111
	//printk("=====ft5x_read_data touch point = %d\n",event->touch_point);

	if (event->touch_point == 0) {
		ft5x_ts_release(data);
		return 1; 
	}

	switch (event->touch_point) {
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			//printk("=====ft5x_read_data source data:event->x5 = %d, event->y5 = %d. \n", event->x5, event->y5);
			if(1 == exchange_x_y_flag) {
				swap(event->x5, event->y5);
			}
			if(1 == revert_x_flag) {
				event->x5 = SCREEN_MAX_X - event->x5;
			}
			if(1 == revert_y_flag) {
				event->y5 = SCREEN_MAX_Y - event->y5;
			}
			event->touch_ID5=(s16)(buf[0x1d] & 0xF0)>>4;
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			//printk("=====ft5x_read_data source data:event->x4 = %d, event->y4 = %d. \n", event->x4, event->y4);
			if(1 == exchange_x_y_flag) {
				swap(event->x4, event->y4);
			}
			if(1 == revert_x_flag) {
				event->x4 = SCREEN_MAX_X - event->x4;
			}
			if(1 == revert_y_flag) {
				event->y4 = SCREEN_MAX_Y - event->y4;
			}	
			event->touch_ID4=(s16)(buf[0x17] & 0xF0)>>4;
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			//printk("=====ft5x_read_data source data:event->x3 = %d, event->y3 = %d. \n", event->x3, event->y3);
			if(1 == exchange_x_y_flag) {
				swap(event->x3, event->y3);
			}
			if(1 == revert_x_flag) {
				event->x3 = SCREEN_MAX_X - event->x3;
			}
			if(1 == revert_y_flag) {
				event->y3 = SCREEN_MAX_Y - event->y3;
			}
			event->touch_ID3=(s16)(buf[0x11] & 0xF0)>>4;
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			//printk("=====ft5x_read_data source data:event->x2 = %d, event->y2 = %d. \n", event->x2, event->y2);
			if(1 == exchange_x_y_flag) {
				swap(event->x2, event->y2);
			}
			if(1 == revert_x_flag) {
				event->x2 = SCREEN_MAX_X - event->x2;
			}
			if(1 == revert_y_flag) {
				event->y2 = SCREEN_MAX_Y - event->y2;
			}
			event->touch_ID2=(s16)(buf[0x0b] & 0xF0)>>4;
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			//printk("=====ft5x_read_data source data:event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
			if(event->x1 >= HOME_KEY_X && event->x1 < (HOME_KEY_X + HOME_KEY_W))
			{
				if(event->y1 >= HOME_KEY_Y && event->y1 < (HOME_KEY_Y + HOME_KEY_H))
						home_key_flag = 1;
			}
			else
			{
				if(1 == exchange_x_y_flag) {
					swap(event->x1, event->y1);
				}
				if(1 == revert_x_flag) {
					event->x1 = SCREEN_MAX_X - event->x1;
				}
				if(1 == revert_y_flag) {
					event->y1 = SCREEN_MAX_Y - event->y1;
				}
				//printk("=====ft5x_read_data dest data:event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
				event->touch_ID1=(s16)(buf[0x05] & 0xF0)>>4;	
			}
			break;
		default:
			return -1;
	}
	event->pressure = 20;
	
	return 0;
}

static void ft5x_report_value(struct ft5x_ts_data *data)
{
	struct ts_event *event = &data->event;

	switch(event->touch_point) {
		case 5:
			input_report_abs(data->input, ABS_MT_TRACKING_ID, event->touch_ID5);	
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 30);
			input_report_key(data->input, BTN_TOUCH, 1); 
			input_mt_sync(data->input);
			//printk("=====ft5x_report_multitouch report data:===x5 = %d,y5 = %d ====\n",event->x5,event->y5);
		case 4:
			input_report_abs(data->input, ABS_MT_TRACKING_ID, event->touch_ID4);	
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 30);
			input_report_key(data->input, BTN_TOUCH, 1); 
			input_mt_sync(data->input);
			//printk("=====ft5x_report_multitouch report data:===x4 = %d,y4 = %d ====\n",event->x4,event->y4);
		case 3:
			input_report_abs(data->input, ABS_MT_TRACKING_ID, event->touch_ID3);	
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 30);
			input_report_key(data->input, BTN_TOUCH, 1); 
			input_mt_sync(data->input);
			//printk("=====ft5x_report_multitouch report data:===x3 = %d,y3 = %d ====\n",event->x3,event->y3);
		case 2:
			input_report_abs(data->input, ABS_MT_TRACKING_ID, event->touch_ID2);	
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 30);
			input_report_key(data->input, BTN_TOUCH, 1); 
			input_mt_sync(data->input);
			//printk("=====ft5x_report_multitouch report data:===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 1:
			input_report_abs(data->input, ABS_MT_TRACKING_ID, event->touch_ID1);	
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 30);
			input_report_key(data->input, BTN_TOUCH, 1);
			input_mt_sync(data->input);
			if(home_key_flag)
			{
					if(home_key_send == 0)
					{
							home_key_send = 1;
							input_report_key(data->input, HOME_KEY, 1);
							//printk("send tp home key\n");
					}
			}
			//printk("=====ft5x_report_multitouch report data:===x1 = %d,y1 = %d ====\n",event->x1,event->y1);
			break;
		default:
			//printk("=====ft5x_report_multitouch report data:==touch_point default =\n");
			break;
	}
	
	input_sync(data->input);
	return;
}	

static irqreturn_t ft5x_ts_interrupt(int irq, void *dev_id)
{
	//printk("=====ft5x_ts_interrupt irq: %d\n", irq);
	struct ft5x_ts_data *ft5x_ts = dev_id;
	int ret = -1;
	ret = ft5x_read_data(ft5x_ts);
	if (ret == 0) {
		ft5x_report_value(ft5x_ts);
	}
	
	return IRQ_HANDLED;
}

#if 0
static int wacom_touch_event(struct notifier_block *this, unsigned long event, void *ptr) {
    //printk("wacom_touch_action_event: %d\n", event);
	if (event == 1) {
		disable_irq(priv_data->client->irq);
	} else {
		enable_irq(priv_data->client->irq);
	}
    return 0;
}

static struct notifier_block wacom_touch_event_notifier = {
    .notifier_call = wacom_touch_event,
};
#endif

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x_ts_data *ft5x_ts;
	struct input_dev *input;
	struct device_node *ft_np;
	int reset_gpio, irq_gpio = -1;
	int err = 0;       

	printk("ft5x_ts_probe\n");

	ft_np = client->dev.of_node;
	if (!ft_np) {
		dev_err(&client->dev, "get device node error\n");
		return -ENODEV;
	}

	/*2019-04-18 cw2015 full led control*/
/*	cw2015_full_led_gpio = of_get_named_gpio(ft_np, "gpio_full_led", 0);
	if (!gpio_is_valid(cw2015_full_led_gpio)) {
		dev_err(&client->dev, "no gpio_full_led_gpio pin available\n");
		return -ENODEV;
	}

	printk("ft5x_ts_probe cw2015_full_led_gpio=%d\n", cw2015_full_led_gpio);
	err = devm_gpio_request_one(&client->dev, cw2015_full_led_gpio, GPIOF_OUT_INIT_LOW, "cw2015_full_led_gpio");
	if (err < 0) {
		return -ENODEV;
	}
	gpio_direction_output(cw2015_full_led_gpio, 0);
*/

	reset_gpio = of_get_named_gpio(ft_np, "gpio_rst", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(&client->dev, "no gpio_rst pin available\n");
		return -ENODEV;
	}
	
	err = devm_gpio_request_one(&client->dev, reset_gpio, GPIOF_OUT_INIT_LOW, "ft-gpio-rst");
	if (err < 0) {
		return -ENODEV;
	}
	gpio_direction_output(reset_gpio, 0);
	msleep(100);
	gpio_direction_output(reset_gpio, 1);

	irq_gpio = of_get_named_gpio(ft_np, "gpio_intr", 0);
	if (!gpio_is_valid(irq_gpio)) {
		err = -ENODEV;
		dev_err(&client->dev, "no gpio_intr pin available\n");
		goto err_free_gpio_reset;
	}

	err = devm_gpio_request_one(&client->dev, irq_gpio, GPIOF_IN, "ft-gpio_intr");
	if (err < 0) {
		goto err_free_gpio_reset;
	}
	
	client->irq = gpio_to_irq(irq_gpio);
	if (client->irq < 0) {
		err = -ENOMEM;
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n", irq_gpio, client->irq);
		goto err_free_gpio_irq;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -EIO;
		dev_err(&client->dev, "i2c_check_functionality error\n");
		goto err_free_gpio_irq;
	}

	if(ft5x_detect_tp(client) < 0){
		printk("[ft5x] %s: ft5x_detect_tp fail\n", __func__);
		goto err_free_gpio_irq;
	}

	ft5x_ts = kzalloc(sizeof(*ft5x_ts), GFP_KERNEL);
	if (!ft5x_ts)	{
		err = -ENOMEM;
		dev_err(&client->dev, "alloc_data_failed error\n");
		goto err_free_gpio_irq;
	}

	input = input_allocate_device();
	if (!input) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto err_free_mem;
	}
	home_key_send = 0;

	ft5x_ts->client = client;
	ft5x_ts->input = input;
	input->name	= "Focaltech I2C TP";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(KEY_HOME, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	
	input_set_drvdata(input, ft5x_ts);

	err = request_threaded_irq(client->irq, NULL, ft5x_ts_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, 
		"ft5x26", ft5x_ts);
	if (err) {
		dev_err(&client->dev, "Failed to enable IRQ, error: %d\n", err);
		goto err_free_input;
	}
	
	err = input_register_device(input);
	if (err) {
		dev_err(&client->dev,"ft5x_ts_probe: failed to register input device: %s\n", dev_name(&client->dev));
		goto err_free_irq;
	}

	i2c_set_clientdata(client, ft5x_ts);

	if (!priv_data) {
		priv_data = ft5x_ts;
	}
//	if (register_wacom_touch_event_notifier(&wacom_touch_event_notifier)) {
//		printk("register wacom_touch_event_notifier error\n");
//	}

	printk("ft5x_ts_probe end\n");

	return 0;

err_free_irq:
	free_irq(client->irq, ft5x_ts);
err_free_input:
	kfree(input);
err_free_mem:
	kfree(ft5x_ts);
err_free_gpio_irq:
	gpio_free(irq_gpio);
err_free_gpio_reset:
	gpio_free(reset_gpio);
	printk("ft5x_ts_probe fail\n");
	
	return err;
}

static int ft5x_ts_remove(struct i2c_client *client)
{
	struct ft5x_ts_data *ft5x_ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x_ts->early_suspend);
#endif
	input_unregister_device(ft5x_ts->input);
	input_free_device(ft5x_ts->input);
	kfree(ft5x_ts);
    
	i2c_set_clientdata(ft5x_ts->client, NULL);

//	unregister_wacom_touch_event_notifier(&wacom_touch_event_notifier);
	priv_data = NULL;

	return 0;
}

static int __maybe_unused ft5x_ts_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);

	return 0;
}

static int __maybe_unused ft5x_ts_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ft5x_ts_i2c_pm, ft5x_ts_i2c_suspend, ft5x_ts_i2c_resume);

static const struct i2c_device_id ft5x_ts_id[] = {
	{ "ft5x26", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);

static const struct of_device_id ft5x_dt_ids[] = {
	{
		.compatible = "focaltech,ft5x26",
		.data = (void *) &ft5x_ts_id[0],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, ft5x_dt_ids);

static struct i2c_driver ft5x_ts_driver = {
	.driver	= {
		.name	= "ft5x26",
		.owner	= THIS_MODULE,
		.of_match_table = ft5x_dt_ids,
		.pm	= &ft5x_ts_i2c_pm,
	},
	.probe		= ft5x_ts_probe,
	.remove		= ft5x_ts_remove,
	.id_table	= ft5x_ts_id,
};

static int __init ft5x_ts_init(void)
{ 
	return i2c_add_driver(&ft5x_ts_driver);
}

static void __exit ft5x_ts_exit(void)
{
	i2c_del_driver(&ft5x_ts_driver);
}

late_initcall(ft5x_ts_init);
module_exit(ft5x_ts_exit);
MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");

