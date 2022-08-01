/*
 * SGM37604A LED Driver
 */
#include <linux/moduleparam.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include "sgm37604a.h"

static int debug = 1;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define _dprintk(level, fmt, arg...) do {   \
  if (debug >= level)      \
  printk("**sgm37604a**%s[%d]: " fmt, __func__, __LINE__, ## arg); } while (0)

#define dprintk(format, ...) _dprintk(1, format, ## __VA_ARGS__)

#define AW87519_NAME "sgm37604a"

#define SGM37604A_MAX_BRIGHTNESS		0x13ff

struct sgm37604a_priv {
	struct i2c_client *client;
	//struct regmap *regmap;
	//struct backlight_device *backlight;
	struct sgm37604a_platform_data *pdata;
	struct gpio_desc *enable_gpio;
	struct workqueue_struct *led_workqueue;
    struct delayed_work led_read_work;
	bool   enabled;
	//struct class *cls_node;
	//int brightness_white;
	//int brightness_yellow;
	unsigned int group;
};
//static struct sgm37604a_priv *g_sgm37604a;
unsigned int regs[]={0x10,0x11,0x19,0x1a,0x1b,0x1f};

#if 0
static const struct reg_sequence sgm37604a_reg_patch[] = {
	{ 0x01, 0x00 },
	{ 0x10, 0x0f },
	{ 0x11, 0x00 },
	{ 0x19, 0x00 },
	{ 0x1a, 0x00 },
	{ 0x1b, 0x00 },
	{ 0x1f, 0x00 },
};

static bool sgm37604a_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0x00 && reg <= 0x1f);
}

static bool sgm37604a_readable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0x00 && reg <= 0x0A);
}

static bool sgm37604a_volatile_register(struct device *dev, unsigned int reg)
{
	//switch(reg) {
	//	case 0x64:
	//	return true;
	//}

	return false;
}

static const struct regmap_config sgm37604a_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.use_single_rw = true,
	.max_register = 0x1f,
	.cache_type = REGCACHE_NONE,
	.readable_reg = sgm37604a_readable_reg,
	.writeable_reg = sgm37604a_writeable_reg,
	.volatile_reg = sgm37604a_volatile_register,
};
#endif

static int sgm37604a_write(struct sgm37604a_priv *sgm, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(sgm->client, reg, data);
}
#if 0
static int sgm37604a_read(struct sgm37604a_priv *sgm, u8 reg)
{
	//printk("sgm37604a_read 0x%x \n",reg);
	return i2c_smbus_read_byte_data(sgm->client, reg);
}
#endif
static int sgm37604a_parse_dt(struct sgm37604a_priv *data)
{
	struct device *dev = &data->client->dev;
	int val = 0;

	data->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(data->enable_gpio)) {
		dev_warn(dev, "%s: No enable GPIO found! continue...\n", __func__);
	}
	if (!of_property_read_u32(dev->of_node, "group", &val)) {
        data->group = val;
    } else {
        data->group = 0x01;
    }
	return 0;
}
static void sgm37604a_backlight_power_ctl(struct sgm37604a_priv *sgm,bool onoff)
{	

	//if(onoff == sgm->enabled)
	//	return ;
	
	dprintk("onoff=%d \n",onoff);
	if(onoff){
		if (!IS_ERR_OR_NULL(sgm->enable_gpio)) {
			gpiod_set_value(sgm->enable_gpio, 1);
			sgm->enabled = true;
		sgm37604a_write(sgm,0x01,0x01);
		sgm37604a_write(sgm,0x10,0x07);
		sgm37604a_write(sgm,0x11,0x00);
		}
	}else{
		//if (!IS_ERR_OR_NULL(sgm->enable_gpio)) {
		//	gpiod_set_value(sgm->enable_gpio, 0);
		//	sgm->enabled = false;
		//}
		sgm37604a_write(sgm,0x01,0x01);
		sgm37604a_write(sgm,0x10,0x00);
		sgm37604a_write(sgm,0x11,0x00);
	}
		

}

static int sgm37604a_backlight_update_status(struct backlight_device *backlight)
{
	struct sgm37604a_priv *sgm = bl_get_data(backlight);
	bool onoff;
	unsigned int ledmod = 0;
	unsigned int brightness = backlight->props.brightness;

	dprintk("brightness=0x%04x \n",brightness);
	if (backlight->props.power != FB_BLANK_UNBLANK ||
	    backlight->props.fb_blank != FB_BLANK_UNBLANK ||
	    backlight->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	if(brightness!=0){
			if(brightness & 0x1000){
				brightness |= 0x3000;
			}else
				brightness |= 0x1000;
	}
		
	onoff = (bool)brightness;
	dprintk("brightness last =0x%04x \n",brightness);

	sgm37604a_backlight_power_ctl(sgm,onoff);

	if (brightness) {
		//ledmod = (brightness&0x3000) >>11;
		ledmod = sgm->group;
		sgm37604a_write(sgm,0x11,0x00);
	
		if(ledmod !=0){
			sgm37604a_write(sgm, SGM37604A_ENABLE, ledmod|0x0f);
		}else
			sgm37604a_write(sgm, SGM37604A_ENABLE, 0x00);
		//sgm37604a_write(sgm, SGM37604A_BRIGHTNESS_CONTROL, 0x10);
		//sgm37604a_write(sgm, SGM37604A_BRIGHTNESS_L, brightness&0x000f);
		sgm37604a_write(sgm, SGM37604A_BRIGHTNESS_M, (brightness&0x03f0)>>4);
		sgm37604a_write(sgm, SGM37604A_BRIGHTNESS_L, brightness&0x000f);
	} else {
		//sgm37604a_write(sgm, SGM37604A_ENABLE, 0);
	}

	return 0;
}

static int sgm37604a_backlight_check_fb(struct backlight_device *backlight,
				       struct fb_info *info)
{
	struct sgm37604a_priv *sgm = bl_get_data(backlight);
	dprintk(" \n");
	return sgm->pdata->fbdev == NULL || sgm->pdata->fbdev == info->dev;
}

static const struct backlight_ops sgm37604a_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= sgm37604a_backlight_update_status,
	.check_fb	= sgm37604a_backlight_check_fb,
};
#if 0
static void sgm_read_work(struct work_struct *work)
{

	struct sgm37604a_priv *sgm = container_of(work, struct sgm37604a_priv, led_read_work.work);
	//sgm37604a_read(sgm,0x10);

	int val;
	//for(i=0;i<6;i++)
	{
		val = sgm37604a_read(sgm,0x10);
		printk("sgm37604a_read 0x%x=0x%x \n",0x10,val);
	}
	queue_delayed_work(sgm->led_workqueue, &sgm->led_read_work, msecs_to_jiffies(1000));

}
#endif
static int sgm37604a_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct sgm37604a_platform_data *pdata;
	struct backlight_device *backlight;
	struct backlight_properties props;
	struct sgm37604a_priv *sgm;
	//int ret;
	//int val = 0;
	dprintk("sgm37604a_probe \n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&client->dev,
			 "I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}

	sgm = devm_kzalloc(&client->dev, sizeof(*sgm), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sgm))
		return -ENOMEM;
	#if 0
	sgm->regmap = devm_regmap_init_i2c(client, &sgm37604a_regmap);
	if (IS_ERR(sgm->regmap)) {
		 ret = PTR_ERR(sgm->regmap);
		 dev_err(&client->dev, "Failed to allocate register map: %d\n", ret);
		 return ret;
	}
	#endif
	i2c_set_clientdata(client, sgm);
	sgm->client = client;
	if (client->dev.of_node) {
		sgm37604a_parse_dt(sgm);
		pdata = devm_kzalloc(&client->dev,
            sizeof(struct sgm37604a_platform_data), GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(&client->dev, "No platform data supplied\n");
			return -EINVAL;
		}
	}
	pdata->def_value = 0x01f;

	sgm37604a_backlight_power_ctl(sgm,true);
	//regmap_register_patch(sgm->regmap, sgm37604a_reg_patch, ARRAY_SIZE(sgm37604a_reg_patch));

	//sgm->client = client;
	sgm->pdata = pdata;
	//regmap_write(sgm->regmap, 0x01, 0x01);//reset
	
	sgm37604a_write(sgm,0x01,0x01);
	//sgm37604a_write(sgm,0x10,0x1f);
	//regmap_write(sgm->regmap, 0x10, 0x1f);
	//for(i=0;i<6;i++)
	//{
	//	val = sgm37604a_read(sgm,regs[i]);	
	//	printk("sgm37604a_read 0x%x=0x%x \n",regs[i],val);
	//}
	//0x10 必须关闭没贴的灯，否则促发保护，灯不会亮
	//0x10=0x03 开机亮白灯
	//0x10=0x05 开机亮黄灯
	//0x10=0x03 0x11=0x00
	//sgm37604a_write(sgm,0x10,0x05);
	//sgm37604a_write(sgm,0x11,0x00);
	//sgm37604a_write(sgm,SGM37604A_BRIGHTNESS_M,0x3f);
	//sgm37604a_write(sgm,SGM37604A_BRIGHTNESS_L,0x0f);
	//val = sgm37604a_read(sgm,SGM37604A_FLAG);	
	//	printk("sgm37604a_read 0x%x=0x%x \n",0x1f,val);
	//regmap_read(sgm->regmap, 0x10, &val);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = SGM37604A_MAX_BRIGHTNESS;
	props.brightness = sgm->pdata->def_value;//clamp_t(unsigned int, pdata->def_value, 0,props.max_brightness);

	backlight = devm_backlight_device_register(&client->dev,
				dev_name(&client->dev), &sgm->client->dev,
				sgm, &sgm37604a_backlight_ops, &props);
	if (IS_ERR(backlight)) {
		dev_err(&client->dev, "failed to register backlight\n");
		return PTR_ERR(backlight);
	}

	backlight_update_status(backlight);
	i2c_set_clientdata(client, backlight);

    sgm->led_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!sgm->led_workqueue) {
        //err = -ENOMEM;
        dev_err(&client->dev, "failed to create led_workqueue.\n");
		return -ENOMEM;
        //goto failed_create_singlethread_workqueue;
    }

	//INIT_DELAYED_WORK(&sgm->led_read_work, sgm_read_work);
	//queue_delayed_work(sgm->led_workqueue, &sgm->led_read_work,
	//		   msecs_to_jiffies(500));

	return 0;
}

static int sgm37604a_remove(struct i2c_client *client)
{
	struct backlight_device *backlight = i2c_get_clientdata(client);

	backlight->props.brightness = 0;
	backlight_update_status(backlight);

	return 0;
}

static const struct i2c_device_id sgm37604a_ids[] = {
	{ "sgm37604a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgm37604a_ids);

static struct i2c_driver sgm37604a_driver = {
	.driver = {
		.name = "sgm37604a",
	},
	.probe = sgm37604a_probe,
	.remove = sgm37604a_remove,
	.id_table = sgm37604a_ids,
};

module_i2c_driver(sgm37604a_driver);

MODULE_DESCRIPTION("SGM37604A Backlight Driver");
MODULE_AUTHOR("HTfyun <tanluqiang@htfyun.com>");
MODULE_LICENSE("GPL");
