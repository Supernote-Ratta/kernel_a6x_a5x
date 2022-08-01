/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio/consumer.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define STR(x) #x
#define DEF_BRIGHTNESS 0x1f
#define MAX_BRIGHTNESS 0x200 //512--->max 15mA

#define MASK_MODE_BRIGHTNESS 0x3000
#define MAX_BRIGHTNESS_LEVEL 0xFFF //total two chanle the cold light brightneww is high 6bit, warm light is low 6bit

// 20220601-hsl: 用低6BIT表示暖光的亮度等级，高6BIT表示冷光的亮度等级；
// BIT 12 表示 暖光的开关；BIT13表示冷光的开关。合起来一共是 14BIT.
#if 1
#define BL_MASK             0X3FFF  //有效的 brightness 的值.
#define BRIGHTNESS_MASK     0X3F
// 暖光: ch1 最低6BIT, 冷光: ch2. 11-6BIT.
#define WARN_BL(x)          ((x&BRIGHTNESS_MASK)*8)     //*8 把上层背光等级转为真正的寄存器的值。
#define COLD_BL(x)          (((x>>6)&BRIGHTNESS_MASK)*8)
#define WARM_ON(x)          (x&(1<<12))
#define COLD_ON(x)          (x&(1<<13))
#else 
#define COLD_OFF  0x00
#define WARM_OFF  0x01
#define COLD_MODE 0x02
#define WARM_MODE 0x03
#endif 

#define LEDCH_CTL_REG  0x00
#define COLD_CH_LSBREG 0x04
#define COLD_CH_MSBREG 0x05
#define WARM_CH_LSBREG 0x06
#define WARM_CH_MSBREG 0x07

#define CLASS_CREATE_FILE(data, _name_) \
    do { \
        data->_name_.attr.mode = 0666; \
        data->_name_.attr.name = STR(_name_); \
        data->_name_.show = fp9966_##_name_##_show; \
        data->_name_.store = fp9966_##_name_##_store; \
        if (class_create_file(data->cls_node, &data->_name_)) { \
            printk("%s: Fail to creat class file %s\n", __func__, data->_name_.attr.name); \
        } \
    } while(0)

#define CLASS_REMOVE_FILE(data, _name_) \
    do { \
        class_remove_file(data->cls_node, &data->_name_); \
    } while(0)


struct fl_data {
    struct i2c_client *i2c;
    struct backlight_device *fldev;
    bool enabled;
    struct regmap *regmap;
    struct regulator *power_supply;
    struct gpio_desc *enable_gpio;
    struct gpio_desc *bl1_adj_gpio;
    struct gpio_desc *bl2_adj_gpio;
    struct class *cls_node;
    struct class_attribute reg;
    struct class_attribute gpio;
    unsigned int def_brightness;
    int light_mode;
    struct mutex update_mutex;
};

static struct reg_sequence fp9966_reg_def[] = {
    {0x00, 0xdd},
    {0x01, 0x88},
    {0x02, 0x00},
    {0x03, 0x99},
    {0x04, 0x00},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x00},
    {0x08, 0x00},
    {0x09, 0x00},
    {0x0A, 0x00},
    {0x0B, 0x00},
    {0x0C, 0x00},
    {0x0D, 0x00},
    {0x0E, 0x00},
    {0x10, 0x00},
};

static ssize_t fp9966_gpio_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct fl_data *fl = container_of(attr, struct fl_data, gpio);
    ssize_t len = 0;
    char *buf = _buf;

    if (IS_ERR_OR_NULL(fl->enable_gpio)) {
        len += snprintf(buf + len, PAGE_SIZE - len, "No enable gpio found.\n");
    } else {
        len += snprintf(buf + len, PAGE_SIZE - len, "enable_gpio = %d\n", gpiod_get_value(fl->enable_gpio));
    }

    if (IS_ERR_OR_NULL(fl->bl1_adj_gpio)) {
        len += snprintf(buf + len, PAGE_SIZE - len, "No adj1 gpio found.\n");
    } else {
        len += snprintf(buf + len, PAGE_SIZE - len, "adj1_gpio = %d\n", gpiod_get_value(fl->bl1_adj_gpio));
    }

    if (IS_ERR_OR_NULL(fl->bl2_adj_gpio)) {
        len += snprintf(buf + len, PAGE_SIZE - len, "No adj2 gpio found.\n");
    } else {
        len += snprintf(buf + len, PAGE_SIZE - len, "adj2_gpio = %d\n", gpiod_get_value(fl->bl2_adj_gpio));
    }

    return len;
}

static ssize_t fp9966_gpio_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct fl_data *fl = container_of(attr, struct fl_data, gpio);
    int ret;
    char name[64];
    int val;

    ret = sscanf(buf, "%s %d", name, &val);
    printk("%s:cmd buf:%s\n", __func__, buf );

    if (ret != 2) {
        dev_err(&fl->i2c->dev, "%s:param is error, echo enable_gpio 1 > this_node.\n", __func__);
        return _count;
    }
    if (0 == strcmp(name, "enable_gpio")) {
        if (!IS_ERR_OR_NULL(fl->enable_gpio)) {
            gpiod_set_value(fl->enable_gpio, !!val);
        }
    } else if (0 == strcmp(name, "adj1_gpio")) {
        if (!IS_ERR_OR_NULL(fl->bl1_adj_gpio)) {
            printk("set adj1 gpio val = %d.\n", val);
            gpiod_set_value(fl->bl1_adj_gpio, !!val);
        }
    } else if (0 == strcmp(name, "adj2_gpio")) {
        if (!IS_ERR_OR_NULL(fl->bl2_adj_gpio)) {
            printk("set adj2 gpio val = %d.\n", val);
            gpiod_set_value(fl->bl2_adj_gpio, !!val);
        }
    }

    return _count;
}

static ssize_t fp9966_reg_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct fl_data *fl = container_of(attr, struct fl_data, reg);
    ssize_t len = 0;
    char *buf = _buf;
    unsigned int i;
    int ret, val = 0;

    for (i = 0; i < ARRAY_SIZE(fp9966_reg_def); i++) {
        ret = regmap_read(fl->regmap, fp9966_reg_def[i].reg, &val);
        printk("%s:read ret: %d reg: [%02x: %02x]\n", __func__, ret, fp9966_reg_def[i].reg, val);
        if (ret < 0) {
            continue;
        }

        len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x: 0x%02x\n", fp9966_reg_def[i].reg, val);
    }
    ret = regmap_read(fl->regmap, 0x0F, &val);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x0f: 0x%02x\n", val);
    ret = regmap_read(fl->regmap, 0x11, &val);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x11: 0x%02x\n", val);
    return len;
}

static ssize_t fp9966_reg_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct fl_data *fl = container_of(attr, struct fl_data, reg);
    int ret, reg, val;

    ret = sscanf(buf, "%x %x", &reg, &val);

    printk("%s:cmd buf:%s\n", __func__, buf);

    if (ret != 2) {
        dev_err(&fl->i2c->dev, "%s:param is error, echo reg val > this_node.\n", __func__);
        return _count;
    }
    regmap_write(fl->regmap, reg, val);

    return _count;
}

static void frontlight_power(struct fl_data *fl, bool enable)
{
    int err;

    if (enable) {
        if (fl->enabled) {
            return;
        }

        if (fl->power_supply) {
            err = regulator_enable(fl->power_supply);
            if (err < 0) {
                dev_err(&fl->i2c->dev, "failed to enable power supply\n");
            }
        }

        if (fl->enable_gpio) {
            gpiod_set_value(fl->enable_gpio, 1);
        }
        msleep(1);
        regmap_register_patch(fl->regmap, fp9966_reg_def, ARRAY_SIZE(fp9966_reg_def));

        fl->enabled = true;
    } else {
        if (fl->enable_gpio) {
            gpiod_set_value(fl->enable_gpio, 0);
        }

        if (fl->power_supply) {
            regulator_disable(fl->power_supply);
        }
        fl->enabled = false;
    }
}

static bool fp9966_readable_reg(struct device *dev, unsigned int reg)
{
    return (reg >= 0x00 && reg <= 0x11);
}

static bool fp9966_writeable_reg(struct device *dev, unsigned int reg)
{
    return (reg >= 0x00 && reg <= 0x11);
}

static bool fp9966_volatile_reg(struct device *dev, unsigned int reg)
{
    return true;
}

static const struct regmap_config fp9966_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .use_single_rw = true,
    //.max_register = 0x69,
    .cache_type = REGCACHE_NONE,
    .readable_reg = fp9966_readable_reg,
    .writeable_reg = fp9966_writeable_reg,
    .volatile_reg = fp9966_volatile_reg,
};

static int frontlight_update_status(struct backlight_device *backlight)
{
    struct fl_data *fl = bl_get_data(backlight);
    unsigned int value = 0, cold_v = 0, warm_v = 0, reg_v = 0;
    unsigned int brightness = backlight->props.brightness;
    bool        warm_on = 0, cold_on = 0;

    mutex_lock(&fl->update_mutex);
    // 20220604-LOG: brightness=0x0061 backlight->props.power: 0x0 backlight->props.fb_blank: 0x0 backlight->props.state: 0x0
    //printk("backlight->props.power: 0x%x backlight->props.fb_blank: 0x%x backlight->props.state: 0x%x\n", 
    //    backlight->props.power, backlight->props.fb_blank, backlight->props.state);
    if (backlight->props.fb_blank == FB_BLANK_POWERDOWN && backlight->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) {
        printk("frontlight_update_status: FB_BLANK_POWERDOWN or BL_CORE_FBBLANK,power off!\n");
        regmap_read(fl->regmap, fp9966_reg_def[0].reg, &fp9966_reg_def[0].def);
        regmap_read(fl->regmap, fp9966_reg_def[4].reg, &fp9966_reg_def[4].def);
        regmap_read(fl->regmap, fp9966_reg_def[5].reg, &fp9966_reg_def[5].def);
        regmap_read(fl->regmap, fp9966_reg_def[6].reg, &fp9966_reg_def[6].def);
        regmap_read(fl->regmap, fp9966_reg_def[7].reg, &fp9966_reg_def[7].def);

        regmap_write(fl->regmap, WARM_CH_LSBREG, 0);
        regmap_write(fl->regmap, WARM_CH_MSBREG, 0);
        regmap_write(fl->regmap, COLD_CH_LSBREG, 0);
        regmap_write(fl->regmap, COLD_CH_MSBREG, 0);
        frontlight_power(fl, false);
        mutex_unlock(&fl->update_mutex);
        return 0;
    } else if (!backlight->props.power && !backlight->props.fb_blank && !backlight->props.state) {
        // 20220601: normal go here.
        //printk("frontlight_update_status:No Power/fb_blank/state,power on!\n");
        frontlight_power(fl, true);
    }

#if 0
    value = brightness & MAX_BRIGHTNESS_LEVEL;
    fl->light_mode = (brightness >> 0x0C) & 0x03;
    printk("value= 0x%x mode= 0x%x\n", value, fl->light_mode);

    if (value) {
        switch (fl->light_mode) {
            case COLD_OFF:
                regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
                reg_v &= 0xF7;
                regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
                break;
            case WARM_OFF:
                regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
                reg_v &= 0x7F;
                regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
                break;
            case COLD_MODE:
                regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
                reg_v |= 0x8;
                regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
                // brightness: 0->63 and fp9966 curent set 0->MAX, setps is 8, curent = brightness * 8
                cold_v = ((value >> 0x06) & 0x3F) * 8;
                if (MAX_BRIGHTNESS < cold_v) {
                    cold_v = MAX_BRIGHTNESS;
                }
                regmap_write(fl->regmap, COLD_CH_LSBREG, cold_v & 0x07);
                regmap_write(fl->regmap, COLD_CH_MSBREG, (cold_v >> 0x03));
                break;
            case WARM_MODE:
                regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
                reg_v |= 0x80;
                regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
                warm_v = (value & 0x3F) * 8;
                if (MAX_BRIGHTNESS < warm_v) {
                    warm_v = MAX_BRIGHTNESS;
                }
                regmap_write(fl->regmap, WARM_CH_LSBREG, warm_v & 0x07);
                regmap_write(fl->regmap, WARM_CH_MSBREG, (warm_v >> 0x03));
                break;
            default:
                break;
        }
    }
#else 
    value = brightness & BL_MASK;
    //printk("frontlight_update_status:brightness=%d,value=%d,lm=%d!\n", brightness,
    //    value, fl->light_mode);
    if(fl->light_mode == value){
        mutex_unlock(&fl->update_mutex);
        return 0;
    }
    
    fl->light_mode = value;
    warm_v = WARN_BL(value);
    if (MAX_BRIGHTNESS < warm_v) {
        warm_v = MAX_BRIGHTNESS;
    }
                
    cold_v = COLD_BL(value);
    if (MAX_BRIGHTNESS < cold_v) {
        cold_v = MAX_BRIGHTNESS;
    }

    
    if(WARM_ON(value) && warm_v > 0){
        warm_on = 1;
    }

    if(COLD_ON(value) && cold_v > 0){
        cold_on = 1;
    }

    if(cold_on) { //CH2 --冷光。
        regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
        reg_v |= 0x80;
        regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);

        regmap_write(fl->regmap, WARM_CH_LSBREG, cold_v & 0x07);
        regmap_write(fl->regmap, WARM_CH_MSBREG, (cold_v >> 0x03));
    } else {
        regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
        reg_v &= ~0x80;
        regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
    }

    if(warm_on) { //CH1  --暖光
        regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
        reg_v |= 0x08;
        regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
        
        regmap_write(fl->regmap, COLD_CH_LSBREG, warm_v & 0x07);
        regmap_write(fl->regmap, COLD_CH_MSBREG, (warm_v >> 0x03));
    } else {
        regmap_read(fl->regmap, LEDCH_CTL_REG, &reg_v);
        reg_v &= ~0x08; //0xF7;
        regmap_write(fl->regmap, LEDCH_CTL_REG, reg_v);
    }

    
    
    printk("value= 0x%x,warm_on=%d,cold_on=%d, cold=0x%04x warm=0x%04x\n", value,
        warm_on, cold_on, cold_v, warm_v);
#endif 
    //printk("set brightness cold=0x%04x warm=0x%04x\n", cold_v, warm_v);

    mutex_unlock(&fl->update_mutex);

    return 0;
}

static int frontlight_check_fb(struct backlight_device *backlight, struct fb_info *info)
{
    return true;
}

static const struct backlight_ops fp9966_frontlight_ops = {
    .options = BL_CORE_SUSPENDRESUME,
    .update_status = frontlight_update_status,
    .check_fb = frontlight_check_fb,
};

#ifdef CONFIG_OF
static int frontlight_parse_dt(struct fl_data *fl)
{
    struct device *dev = &fl->i2c->dev;

    fl->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
    if (IS_ERR(fl->enable_gpio)) {
        dev_err(dev, "%s: No enable GPIO found! continue...\n", __func__);
        return -ENODEV;
    }

    fl->bl1_adj_gpio = devm_gpiod_get_optional(dev, "adj1", GPIOD_OUT_LOW);
    if (IS_ERR(fl->bl1_adj_gpio)) {
        dev_warn(dev, "%s: No adj1 GPIO found! continue...\n", __func__);
    }

    fl->bl2_adj_gpio = devm_gpiod_get_optional(dev, "adj2", GPIOD_OUT_LOW);
    if (IS_ERR(fl->bl2_adj_gpio)) {
        dev_warn(dev, "%s: No adj2 GPIO found! continue...\n", __func__);
    }

    fl->power_supply = devm_regulator_get(dev, "power");
    if (IS_ERR(fl->power_supply)) {
        dev_warn(dev, "%s: No adj2 GPIO found! continue...\n", __func__);
    }

    fl->light_mode = -1; //COLD_MODE;
    return 0;
}

static struct of_device_id frontlight_of_match[] = {
    { .compatible = "fp9966-frontlight" },
    { }
};

MODULE_DEVICE_TABLE(of, frontlight_of_match);
#else
static int frontlight_parse_dt(struct fl_data *fl)
{
    return -ENODEV;
}
#endif

static int frontlight_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct fl_data *fl;
    struct backlight_properties props;
    int ret;

    dev_err(&i2c->dev, "%s\n", __func__);

    fl = devm_kzalloc(&i2c->dev, sizeof(*fl), GFP_KERNEL);
    if (IS_ERR_OR_NULL(fl)) {
        return -ENOMEM;
    }

    i2c_set_clientdata(i2c, fl);
    fl->i2c = i2c;

    ret = frontlight_parse_dt(fl);
    if (ret < 0) {
        dev_err(&i2c->dev, "failed to find dev data\n");
        goto free_mem;
    }

    fl->regmap = devm_regmap_init_i2c(i2c, &fp9966_regmap);
    if (IS_ERR(fl->regmap)) {
        dev_err(&i2c->dev, "Failed to allocate register map!!!\n");
        goto free_gpio;
    }

    fl->def_brightness = DEF_BRIGHTNESS;
    memset(&props, 0, sizeof(props));
    props.type = BACKLIGHT_RAW;
    props.max_brightness = BL_MASK; //MAX_BRIGHTNESS_LEVEL | MASK_MODE_BRIGHTNESS;
    props.brightness = fl->def_brightness;

    fl->fldev = devm_backlight_device_register(&i2c->dev, "ht_eink_fl", &i2c->dev, fl, &fp9966_frontlight_ops, &props);
    if (IS_ERR(fl->fldev)) {
        dev_err(&i2c->dev, "failed to register backlight!!!\n");
        goto free_gpio;
    }

    mutex_init(&fl->update_mutex);
    frontlight_power(fl, true);
    fl->cls_node = class_create(THIS_MODULE, "fp9966");
    if (IS_ERR_OR_NULL(fl->cls_node)) {
        dev_err(&i2c->dev, "failed to create class htfyun fp9966!!!\n");
    } else {
        CLASS_CREATE_FILE(fl, gpio);
        CLASS_CREATE_FILE(fl, reg);
    }

    return 0;
free_gpio:
    devm_gpiod_put(&i2c->dev, fl->enable_gpio);
    devm_gpiod_put(&i2c->dev, fl->bl1_adj_gpio);
    devm_gpiod_put(&i2c->dev, fl->bl2_adj_gpio);
free_mem:
    if (fl) {
        kfree(fl);
        fl = NULL;
    }
    return ret;
}

static int frontlight_remove(struct i2c_client *i2c)
{
    struct fl_data *fl = i2c_get_clientdata(i2c);

    frontlight_power(fl, false);

    CLASS_REMOVE_FILE(fl, gpio);
    //CLASS_REMOVE_FILE(fl, reg);
    class_destroy(fl->cls_node);

    return 0;
}

static void frontlight_shutdown(struct i2c_client *i2c)
{
    struct fl_data *fl = i2c_get_clientdata(i2c);

    frontlight_power(fl, false);
}

#ifdef CONFIG_PM_SLEEP
static int frontlight_suspend(struct device *dev)
{
    return 0;
}

static int frontlight_resume(struct device *dev)
{
    return 0;
}
#endif

static const struct dev_pm_ops frontlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = frontlight_suspend,
    .resume = frontlight_resume,
#endif
};

static const struct i2c_device_id frontlight_i2c_id[] = {
    { "fp9966-frontlight", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, frontlight_i2c_id);

static struct i2c_driver frontlight_driver = {
    .driver = {
        .name = "frontlight",
        .owner = THIS_MODULE,
        .pm = &frontlight_pm_ops,
        .of_match_table = of_match_ptr(frontlight_of_match),
    },
    .probe = frontlight_probe,
    .remove = frontlight_remove,
    .shutdown = frontlight_shutdown,
    .id_table = frontlight_i2c_id,
};

module_i2c_driver(frontlight_driver);

MODULE_DESCRIPTION("Frontlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("htfyun frontlight");
