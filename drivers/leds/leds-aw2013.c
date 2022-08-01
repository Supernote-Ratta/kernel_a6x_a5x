/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-aw2013.h>
#include <linux/workqueue.h>

/* register address */
#define AW_REG_RESET			0x00
#define AW_REG_GLOBAL_CONTROL		0x01
#define AW_REG_LED_STATUS		0x02
#define AW_REG_LED_ENABLE		0x30
#define AW_REG_LED_CONFIG_BASE		0x31
#define AW_REG_LED_BRIGHTNESS_BASE	0x34
#define AW_REG_TIMESET0_BASE		0x37
#define AW_REG_TIMESET1_BASE		0x38
/* register bits */
#define AW2013_CHIP_STANDBY	(0x00)
#define AW2013_CHIPID			0x33
#define AW2013_CHIPID_2			0x9
#define AW_LED_MOUDLE_ENABLE_MASK	0x01
#define AW_LED_FADE_OFF_MASK		0x40
#define AW_LED_FADE_ON_MASK		0x20
#define AW_LED_BREATHE_MODE_MASK	0x10
#define AW_LED_RESET_MASK		0x55
#define AW_LED_RESET_DELAY		8
#define AW2013_VDD_MIN_UV		2600000
#define AW2013_VDD_MAX_UV		3300000
#define AW2013_VI2C_MIN_UV		1800000
#define AW2013_VI2C_MAX_UV		1800000
#define MAX_RISE_TIME_MS		7
#define MAX_HOLD_TIME_MS		5
#define MAX_FALL_TIME_MS		7
#define MAX_OFF_TIME_MS			5

#define CHG_LED_RED             1
#define CHG_LED_GREEN           2
#define CHG_LED_BLUE           	4

struct aw2013_led {
	struct i2c_client *client;
	//struct led_classdev cdev;
	struct aw2013_platform_data pdata[3];
	struct delayed_work power_on_work;
	struct mutex lock;
	struct regulator *vdd;
	//int num_leds;
	//int id;
	bool suspended;
	bool poweron;
	bool is_working;
	bool need_refresh;
};
struct aw2013_led *gled_data;
struct i2c_client *AW2013_i2c_client;
extern bool fb_power_off(void);

static int aw2013_write(struct aw2013_led *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}
static int aw2013_read(struct aw2013_led *led, u8 reg, u8 *val)
{
	s32 ret;
	ret = i2c_smbus_read_byte_data(led->client, reg);
	if (ret < 0)
		return ret;
	*val = ret;
	return 0;
}
static int aw2013_power_on(struct aw2013_led *led, bool on)
{
	int rc;
	if (on) {
		rc = regulator_enable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		led->poweron = true;
	} else {
		rc = regulator_disable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		led->poweron = false;
	}
	return rc;
}
static int aw2013_power_init(struct aw2013_led *led, bool on)
{
	int rc;
	if (on) {
		led->vdd = regulator_get(&led->client->dev, "vdd");
		if (IS_ERR(led->vdd)) {
			rc = PTR_ERR(led->vdd);
			dev_err(&led->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(led->vdd) > 0) {
			rc = regulator_set_voltage(led->vdd, AW2013_VDD_MIN_UV,
						   AW2013_VDD_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}
	} else {
		if (regulator_count_voltages(led->vdd) > 0)
			regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);
		regulator_put(led->vdd);
	}
	return 0;
reg_vdd_put:
	regulator_put(led->vdd);
	return rc;
}

static void aw2013_brightness_work(int color,enum led_brightness brightnessr,enum led_brightness brightnessg,enum led_brightness brightnessb)
{
	u8 val;
	if(gled_data == NULL){
		return;
	}
	/* enable regulators if they are disabled */
	if (!gled_data->poweron) {
		
		//if (aw2013_power_on(gled_data, true)) {
		gled_data->pdata[0].brightness = brightnessr;
		gled_data->pdata[1].brightness = brightnessg;
		gled_data->pdata[2].brightness = brightnessb;
		gled_data->need_refresh = true;
			dev_err(&gled_data->client->dev, "power not on ");
			return;
		//}
	}
	color&=0x07;
	//if((led->is_working == false)&&(led->cdev.brightness==0))
	//	return;
	printk("aw2013_brightness_work coler:%d r:%d g:%d b:%d\n",color,brightnessr,brightnessg,brightnessb);
	mutex_lock(&gled_data->lock);
	gled_data->need_refresh = false;
	if ((brightnessr == 0) && (brightnessg == 0) && (brightnessb == 0)){
		aw2013_read(gled_data, AW_REG_LED_ENABLE, &val);
		aw2013_write(gled_data, AW_REG_LED_ENABLE, val & 0xF8);
	}else{
		if(brightnessr>255){
			brightnessr = 255;
		}
		if(brightnessg>255){
			brightnessg = 255;
		}
		if(brightnessb>255){
			brightnessb = 255;
		}
		aw2013_write(gled_data, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);
		
			//printk("aw2013_brightness_work coler111111\n");
		aw2013_write(gled_data, AW_REG_LED_CONFIG_BASE,
			1);
		aw2013_write(gled_data, AW_REG_LED_CONFIG_BASE+1,
			1);
		aw2013_write(gled_data, AW_REG_LED_CONFIG_BASE+2,
			1);
		
			//printk("aw2013_brightness_work coler2222222222\n");
		aw2013_write(gled_data, AW_REG_LED_BRIGHTNESS_BASE,
			brightnessr);
		aw2013_write(gled_data, AW_REG_LED_BRIGHTNESS_BASE+1,
			brightnessg);
		aw2013_write(gled_data, AW_REG_LED_BRIGHTNESS_BASE+2,
			brightnessb);
		
			//printk("aw2013_brightness_work coler33333\n");
		aw2013_read(gled_data, AW_REG_LED_ENABLE, &val);
		
		//printk("aw2013_brightness_work coler444444\n");
		val &=0xF8;
		aw2013_write(gled_data, AW_REG_LED_ENABLE, val | color);
		
		//printk("aw2013_brightness_work coler55555555\n");
		//led->is_working = true;
	}
	mutex_unlock(&gled_data->lock);
}
static void aw2013_power_on_work(struct work_struct *work)
{
	int rc;
	struct aw2013_led *led = gled_data;
	int color = 0x00;
	enum led_brightness brightnessr,brightnessg,brightnessb;
	printk("aw2013_power_on_work \n");
	led->suspended = false;
	rc = regulator_enable(led->vdd);
	if (rc) {
		dev_err(&led->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return;
	}
	led->poweron = true;

	if(led->need_refresh == true){
		if(led->pdata[0].brightness!=0)
			color = 0x01;
		if(led->pdata[1].brightness!=0)
			color |= 0x02;
		if(led->pdata[2].brightness!=0)
			color |= 0x04;
		brightnessr = led->pdata[0].brightness;
		brightnessg = led->pdata[1].brightness;
		brightnessb = led->pdata[2].brightness;
		aw2013_brightness_work(color, brightnessr, brightnessg, brightnessb);
	}
	return;
}

#if 0

static void aw2013_led_blink_set(struct aw2013_led *led, unsigned long blinking)
{
	u8 val;
	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			return;
		}
	}
	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
	if (blinking > 0) {
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
			AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
			AW_LED_BREATHE_MODE_MASK | led->pdata->max_current);
		aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,
			led->cdev.brightness);
		aw2013_write(led, AW_REG_TIMESET0_BASE + led->id * 3,
			led->pdata->rise_time_ms << 4 |
			led->pdata->hold_time_ms);
		aw2013_write(led, AW_REG_TIMESET1_BASE + led->id * 3,
			led->pdata->fall_time_ms << 4 |
			led->pdata->off_time_ms);
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
		led->is_working = true;
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));
		led->is_working = false;
	}
}
void aw2013_ht_led_blink_set(int color, unsigned long blinking)
{
	struct aw2013_led *led;
	int led_num = 0;
	switch(color){
		case CHG_LED_RED:
			led_num = 0;
		break;
		case CHG_LED_GREEN:
			led_num = 1;
		break;
		case CHG_LED_BLUE:
			led_num = 2;
		break;
		default:
			break;
	}
	if(gled_array!= NULL){
		led = &gled_array[led_num];
		aw2013_led_blink_set(led,blinking);
	}
}

static void aw2013_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
	led->cdev.brightness = brightness;
	schedule_work(&led->brightness_work);
}
#endif
void aw2013_ht_set_brightness(int color,enum led_brightness brightnessr,enum led_brightness brightnessg,enum led_brightness brightnessb)
{

	if(gled_data!= NULL){
		//schedule_work(&led->brightness_work);
		aw2013_brightness_work(color,brightnessr,brightnessg,brightnessb);
	}
}
#if 0
static ssize_t aw2013_store_blink(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	ssize_t ret = -EINVAL;
	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	mutex_lock(&led->pdata->led->lock);
	aw2013_led_blink_set(led, blinking);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}
static ssize_t aw2013_led_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			led->pdata->rise_time_ms, led->pdata->hold_time_ms,
			led->pdata->fall_time_ms, led->pdata->off_time_ms);
}
static ssize_t aw2013_led_time_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	int rc, rise_time_ms, hold_time_ms, fall_time_ms, off_time_ms;
	rc = sscanf(buf, "%d %d %d %d",
			&rise_time_ms, &hold_time_ms,
			&fall_time_ms, &off_time_ms);
	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = (rise_time_ms > MAX_RISE_TIME_MS) ?
				MAX_RISE_TIME_MS : rise_time_ms;
	led->pdata->hold_time_ms = (hold_time_ms > MAX_HOLD_TIME_MS) ?
				MAX_HOLD_TIME_MS : hold_time_ms;
	led->pdata->fall_time_ms = (fall_time_ms > MAX_FALL_TIME_MS) ?
				MAX_FALL_TIME_MS : fall_time_ms;
	led->pdata->off_time_ms = (off_time_ms > MAX_OFF_TIME_MS) ?
				MAX_OFF_TIME_MS : off_time_ms;
	aw2013_led_blink_set(led, 1);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}
static DEVICE_ATTR(blink, 0664, NULL, aw2013_store_blink);
static DEVICE_ATTR(led_time, 0664, aw2013_led_time_show, aw2013_led_time_store);
static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_blink.attr,
	&dev_attr_led_time.attr,
	NULL,
};
static struct attribute_group aw2013_led_attr_group = {
	.attrs = aw2013_led_attributes
};
#endif
static int aw_2013_check_chipid(struct aw2013_led *led)
{
	u8 val;
	int i ;
	for (i=0;i<5;i++) {
		aw2013_write(led, AW_REG_RESET, AW_LED_RESET_MASK);
		msleep(i+5);
		aw2013_read(led, AW_REG_RESET, &val);
		if (val == AW2013_CHIPID || val == AW2013_CHIPID_2 ) 
			return 0;
		else
			pr_info("aw2013 check id error, id=%d \n ",val);
	}
	return -1;
}

static void aw2013_shutdown(struct i2c_client *client)
{
	struct aw2013_led *led = i2c_get_clientdata(client);
	mutex_lock(&led->lock);
	aw2013_write(led, AW_REG_GLOBAL_CONTROL, AW2013_CHIP_STANDBY);
	mutex_unlock(&led->lock);
}

#ifdef CONFIG_PM
static int aw2013_led_suspend(struct device *dev)
{
	struct aw2013_led *led = dev_get_drvdata(dev);
	int ret = 0;
	//u8 val;
	if (led->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}
	if(fb_power_off()){
		printk("aw2013_led_suspend \n");
		cancel_delayed_work(&led->power_on_work);
		mutex_lock(&led->lock);
		ret = aw2013_power_on(led, false);
		if (ret) {
			dev_err(dev, "power off failed");
			mutex_unlock(&led->lock);
			return ret;
		}
		led->suspended = true;
		mutex_unlock(&led->lock);
	}
	#if 0
	mutex_lock(&led->lock);
	//if(led->is_working){
		printk("aw2013_led_suspend ,led is working\n");
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		/*
		 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
		 * all off. So we need to power it off.
		 * If value in AW_REG_LED_ENABLE is not 0, that means LEDs are
		 * already turned on by upper layer, we keep them alive during
		 * suspend so as to support screen-off notification LED.
		 */
		if (val == 0) {
			ret = aw2013_power_on(led, false);
			if (ret) {
				dev_err(dev, "power off failed");
				mutex_unlock(&led->lock);
				return ret;
			}
		}
	//}
	led->suspended = true;
	mutex_unlock(&led->lock);
	#endif
	return ret;

}
static int aw2013_led_resume(struct device *dev)
{
	struct aw2013_led *led = dev_get_drvdata(dev);
	int ret = 0;
	if (!led->suspended) {
		dev_info(dev, "Already in awake state\n");
		return 0;
	}
	//mutex_lock(&led->lock);
	if (led->poweron) {
		led->suspended = false;
	//	mutex_unlock(&led->lock);
		return 0;
	}
	printk("aw2013_led_resume \n");
	schedule_delayed_work(&led->power_on_work,2*HZ);
	//ret = aw2013_power_on(led, true);
	//if (ret) {
	//	dev_err(dev, "power on failed");
	//	mutex_unlock(&led->lock);
	//	return ret;
	//}
	//led->suspended = false;
	//mutex_unlock(&led->lock);
	return ret;
}
static const struct dev_pm_ops aw2013_led_pm_ops = {
	.suspend = aw2013_led_suspend,
	.resume = aw2013_led_resume,
};
#else
static int aw2013_led_suspend(struct device *dev)
{
	return 0;
}
static int aw2013_led_resume(struct device *dev)
{
	return 0;
}
static const struct dev_pm_ops aw2013_led_pm_ops = {
};

static int AW2013_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);

	return err;
}


static int aw2013_led_err_handle(struct aw2013_led *led_array,
				int parsed_leds)
{
	int i;
	/*
	 * If probe fails, cannot free resource of all LEDs, only free
	 * resources of LEDs which have allocated these resource really.
	 */
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}

static int aw2013_led_parse_child_node(struct aw2013_led *led_array,
				struct device_node *node)
{
	struct aw2013_led *led;
	struct device_node *temp;
	struct aw2013_platform_data *pdata;
	int rc = 0, parsed_leds = 0;
	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->client = led_array->client;
		pdata = devm_kzalloc(&led->client->dev,
				sizeof(struct aw2013_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&led->client->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = led_array;
		led->pdata = pdata;
		rc = of_property_read_string(temp, "aw2013,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,id",
			&led->id);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,max-current",
			&led->pdata->max_current);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-current, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "aw2013,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}
		of_property_read_string(temp, "linux,default-trigger",
			&led->cdev.default_trigger);
		INIT_DELAYED_WORK(&led->power_on_work, aw2013_power_on_work);
		led->cdev.brightness_set = aw2013_set_brightness;
		rc = led_classdev_register(&led->client->dev, &led->cdev);
		if (rc) {
			dev_err(&led->client->dev,
				"unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_pdata;
		}
		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw2013_led_attr_group);
		if (rc) {
			dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		led->is_working = false;
		parsed_leds++;
	}
	return 0;
free_class:
	aw2013_led_err_handle(led_array, parsed_leds);
	led_classdev_unregister(&led_array[parsed_leds].cdev);
	cancel_work_sync(&led_array[parsed_leds].brightness_work);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	led_array[parsed_leds].pdata = NULL;
	return rc;
free_pdata:
	aw2013_led_err_handle(led_array, parsed_leds);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	return rc;
free_err:
	aw2013_led_err_handle(led_array, parsed_leds);
	return rc;
}
#endif
static int aw2013_led_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct aw2013_led *data;
	//const struct aw2013_platform_data *pdata;
	int ret;
	printk("%s start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}
	data = kzalloc(sizeof(struct aw2013_led), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	gled_data = data;
	data->client = client;
	//data->dev = client->dev;
	mutex_init(&gled_data->lock);
	ret = aw_2013_check_chipid(gled_data);
	if (ret) {
		dev_err(&client->dev, "Check chip id error\n");
	//	goto free_led_arry;
	}

	i2c_set_clientdata(client, data);
	ret = aw2013_power_init(data, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto free_led_arry;
	}
	//AW2013_create_sysfs(client);
	ret = aw2013_power_on(data, true);
	if (ret) {
		dev_err(&client->dev, "power on failed");
		goto pwr_deinit;
	}
	INIT_DELAYED_WORK(&data->power_on_work, aw2013_power_on_work);
	data->need_refresh = false;
	return 0;
pwr_deinit:
	aw2013_power_init(data, false);
//fail_parsed_node:
//	aw2013_led_err_handle(data);
free_led_arry:
	mutex_destroy(&data->lock);
	devm_kfree(&client->dev, data);
	gled_data = NULL;
	return ret;
}
static int aw2013_led_remove(struct i2c_client *client)
{
	struct aw2013_led *led_array = i2c_get_clientdata(client);
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return 0;
}
static const struct i2c_device_id aw2013_led_id[] = {
	{"aw2013_led", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw2013_led_id);
static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "awinic,aw2013",},
	{ },
};
static struct i2c_driver aw2013_led_driver = {
	.probe = aw2013_led_probe,
	.remove = aw2013_led_remove,
	.shutdown = aw2013_shutdown, 
	.driver = {
		.name = "aw2013_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw2013_match_table),
#ifdef CONFIG_PM
		.pm = &aw2013_led_pm_ops,
#endif
	},
	.id_table = aw2013_led_id,
};
static int __init aw2013_led_init(void)
{
	return i2c_add_driver(&aw2013_led_driver);
}
module_init(aw2013_led_init);
static void __exit aw2013_led_exit(void)
{
	i2c_del_driver(&aw2013_led_driver);
}
module_exit(aw2013_led_exit);
MODULE_DESCRIPTION("AWINIC aw2013 LED driver");
MODULE_LICENSE("GPL v2");
