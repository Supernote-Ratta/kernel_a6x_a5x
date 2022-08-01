#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>

#include "htfyun_aw87xxx.h"

static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);

#define _dprintk(level, fmt, arg...) \
    do { \
        if (debug >= level) { \
            printk("**htfyun-aw87xxx**%s[%d]: " fmt, __func__, __LINE__, ## arg); \
        } \
    } while (0)

#define dprintk(format, ...) _dprintk(1, format, ## __VA_ARGS__)


#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define CLASS_CREATE_FILE(data,_name_) \
    do{ \
        data->s_##_name_.attr.mode = 0666; \
        data->s_##_name_.attr.name = STR(_name_); \
        data->s_##_name_.show = aw87xxx_class_##_name_##_show; \
        data->s_##_name_.store = aw87xxx_class_##_name_##_store; \
        if (class_create_file(data->cls_node, &data->s_##_name_)) { \
            printk("%s: Fail to creat class file %s\n", __func__, data->s_##_name_.attr.name); \
        } \
    } while(0)

#define CLASS_REMOVE_FILE(data,_name_) \
    do{ \
        class_remove_file(data->cls_node, &data->s_##_name_); \
    } while(0)


#define AW87xxx_NAME "htfyun-aw87xxx"
#define AW87519_NAME "htfyun-aw87519"
#define AW87559_NAME "htfyun-aw87559"

/* private data */
struct aw87xxx_priv {
    struct i2c_client *i2c;
    struct regmap *regmap;

    struct regulator *supply;
    struct gpio_desc *enable_gpio;
    struct gpio_desc *reset_gpio;

    struct class *cls_node;
    struct class_attribute s_reg;//used for apps, which regmap is not permissive
    struct class_attribute s_gpio;
    struct class_attribute s_mode;
    int id;
};

static struct aw87xxx_priv *g_aw87xxx;

static const struct reg_sequence aw87519_reg_patch[] = {
    { 0x69, 0x80 },
    { 0x69, 0xB7 },
    { 0x01, 0xF0 },
    { 0x02, 0x09 },
    { 0x03, 0xE8 }, // BOOST OUTPUT TO 8.5V, MAX value.
    { 0x04, 0x19 }, // BOOST PEAK current: 4.0A.  from 0x11
    { 0x05, 0x12 }, // Setting Class D Amplifying Gain, RCV_MODE=0, MAX 27DB
    { 0x06, 0x47 }, // AGC3 set to 47:1.2W, 4A: 1.5W@8 OM  from 0x43
    { 0x07, 0x4E },
    { 0x08, 0x05 }, // from 0x03
    { 0x09, 0x08 },
    { 0x0A, 0x4A },
    { 0x60, 0x16 }, // WHAT-REG??
    { 0x61, 0x20 },
    { 0x62, 0x01 },
    { 0x63, 0x0B },
    { 0x64, 0xC5 },
    { 0x65, 0xA4 },
    { 0x66, 0x78 },
    { 0x67, 0xC4 },
    { 0x68, 0XD0 },
};

static const struct reg_sequence aw87559_reg_patch[] = {
    {0x62, 0xB5},
    {0x78, 0x39},
    {0x79, 0xE5},
    {0x77, 0xC1},
    {0x77, 0xC1},
    {0x78, 0x7A},
    {0x79, 0x6C},
    {0x77, 0x81},
    {0x66, 0x69},
    {0x58, 0xBC},
    {0x02, 0x49},
    {0x03, 0x06},
    {0x04, 0x80},
    {0x05, 0x08},
    {0x06, 0x11}, //0X0C: 18DB, 0X0E: 21DB 0x11: 25.5DB
    {0x07, 0x84},
    {0x08, 0x92},
    {0x09, 0x03},
    {0x0A, 0x08},
    {0x0B, 0x4A},
    {0x0C, 0x02},
    {0x0D, 0x77},
    {0x0E, 0x7A},
    {0x0F, 0x51},
    {0x10, 0x58},
    {0x60, 0x26},
    {0x61, 0x15},
    {0x63, 0x5A},
    {0x64, 0xD5},
    {0x65, 0x57},
    {0x67, 0x28},
    {0x68, 0x35},
    {0x69, 0x98},
    {0x70, 0x1C},
    {0x71, 0x9C},
    {0x72, 0x33},
    {0x73, 0x40},
    {0x74, 0x6C},
    {0x01, 0x78},
};

static int aw87xxx_test_i2c(struct aw87xxx_priv *data)
{
    int reg = 0x0;//chip id register is 0x00
    int val = 0;
    int ret = regmap_read(data->regmap, reg, &val);
    dprintk("aw87xx chip id is 0x%x", val);
    if (ret < 0) {
        dev_err(&data->i2c->dev, "i2c is failed. please check hard.\n");
    }
    return ret;
}

static int aw87xxx_set_enabled(struct aw87xxx_priv *data, bool enabled)
{
    int ret;
    if (!IS_ERR_OR_NULL(data->supply)) {
        if (enabled) {
            ret = regulator_enable(data->supply);
        } else {
            ret = regulator_disable(data->supply);
        }
        if (ret < 0) {
            dev_err(&data->i2c->dev, "%s:failed to enable supply: %d\n", __func__, ret);
        }
        msleep(10);
    }
    if (!IS_ERR_OR_NULL(data->enable_gpio)) {
        gpiod_set_value(data->enable_gpio, enabled);
        msleep(10);
    }
    return 0;
}

static int aw87xxx_set_hw_on(struct aw87xxx_priv *data, bool on)
{
    if (IS_ERR_OR_NULL(data->reset_gpio)) {
        return -ENODEV;
    }
    if (on) {
        gpiod_set_value_cansleep(data->reset_gpio, 0);
        usleep_range(2000, 2500);
        gpiod_set_value_cansleep(data->reset_gpio, 1);
        usleep_range(2000, 2500);
        regmap_write(data->regmap, 0x64, 0x2C);
    } else {
        regmap_write(data->regmap, REG_SYSCTRL, 0x00);
        gpiod_set_value_cansleep(data->reset_gpio, 0);
        usleep_range(2000, 2500);
    }

    return 0;
}

static int aw87xxx_parse_dt(struct aw87xxx_priv *data)
{
    struct device *dev = &data->i2c->dev;

    data->supply = devm_regulator_get(dev, "power");

    if (IS_ERR(data->supply)) {
        dev_warn(dev, "%s: No supply found! continue...\n", __func__);
    }
    data->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
    if (IS_ERR(data->enable_gpio)) {
        dev_warn(dev, "%s: No enable GPIO found! continue...\n", __func__);
    }
    data->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(data->reset_gpio)) {
        dev_warn(dev, "%s: No reset GPIO found! continue...\n", __func__);
    }
    return 0;
}

static bool aw87xxx_writeable_reg(struct device *dev, unsigned int reg)
{
    return (reg >= 0x00 && reg <= 0x10) || (reg >= 0x58 && reg <= 0x79);
}

static bool aw87xxx_readable_reg(struct device *dev, unsigned int reg)
{
    return (reg >= 0x00 && reg <= 0x10) || (reg >= 0x58 && reg <= 0x79);
}

static bool aw87xxx_volatile_register(struct device *dev, unsigned int reg)
{
    switch (reg) {
        case 0x64:
            return true;
    }

    return false;
}

static const struct regmap_config aw87xxx_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .use_single_rw = true,
    .max_register = 0x79,
    .cache_type = REGCACHE_NONE,
    .readable_reg = aw87xxx_readable_reg,
    .writeable_reg = aw87xxx_writeable_reg,
    .volatile_reg = aw87xxx_volatile_register,
};

int aw87xxx_set_audio_amplifier_enable(bool enabled)
{
    struct aw87xxx_priv *data = g_aw87xxx;

    if (IS_ERR_OR_NULL(data)) {
        return -ENODEV;
    }

    dprintk("enabled = %d.\n", enabled);
    if (!enabled) {
        aw87xxx_set_hw_on(data, enabled);
    }

    aw87xxx_set_enabled(data, enabled);

    if (enabled) {
        aw87xxx_set_hw_on(data, enabled);
        dprintk("regmap_register_patch enabled = %d.\n", enabled);
        switch (data->id) {
            case 0:
                regmap_register_patch(data->regmap, aw87519_reg_patch, ARRAY_SIZE(aw87519_reg_patch));
                break;
            case 1:
                regmap_register_patch(data->regmap, aw87559_reg_patch, ARRAY_SIZE(aw87559_reg_patch));
                break;
            default:
                break;
        }
    }

    return 0;
}

static ssize_t aw87xxx_class_reg_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct aw87xxx_priv *data = container_of(attr, struct aw87xxx_priv, s_reg);
    ssize_t len = 0;
    char *buf = _buf;
    unsigned int i;
    int ret = 0, val = 0, size = 0;
    const struct reg_sequence *preg;

    switch (data->id) {
        case 0:
            size = ARRAY_SIZE(aw87519_reg_patch);
            preg = aw87519_reg_patch;
            break;
        case 1:
            size = ARRAY_SIZE(aw87559_reg_patch);
            preg = aw87559_reg_patch;
            break;
        default:
            break;
    }

    for (i = 0; i < size; i++) {
        ret = regmap_read(data->regmap, preg[i].reg, &val);
        printk("%s:read ret: %d reg: [%02x: %02x]\n", __func__, ret, preg[i].reg, val);
        if (ret < 0) {
            continue;
        }

        len += snprintf(buf + len, PAGE_SIZE - len, "%02x: %02x\n", preg[i].reg, val);
    }
    return len;
}

static ssize_t aw87xxx_class_reg_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct aw87xxx_priv *data = container_of(attr, struct aw87xxx_priv, s_reg);
    int ret;
    int reg;
    int val;
    ret = sscanf(buf, "%x %x", &reg, &val);

    printk("%s:cmd buf:%s\n", __func__, buf );

    if (ret != 2) {
        dev_err(&data->i2c->dev, "%s:param is error, echo reg val > this_node.\n", __func__);
        return _count;
    }
    regmap_write(data->regmap, reg, val);

    return _count;
}

static ssize_t aw87xxx_class_gpio_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    struct aw87xxx_priv *data = container_of(attr, struct aw87xxx_priv, s_gpio);
    ssize_t len = 0;
    char *buf = _buf;

    if (IS_ERR_OR_NULL(data->enable_gpio)) {
        len += snprintf(buf + len, PAGE_SIZE - len, "No enable_gpio found.\n");
    } else {
        len += snprintf(buf + len, PAGE_SIZE - len, "enable_gpio = %d\n", gpiod_get_value(data->enable_gpio));
    }

    if (IS_ERR_OR_NULL(data->reset_gpio)) {
        len += snprintf(buf + len, PAGE_SIZE - len, "No reset_gpio found.\n");
    } else {
        len += snprintf(buf + len, PAGE_SIZE - len, "reset_gpio = %d\n", gpiod_get_value(data->reset_gpio));
    }

    return len;
}

static ssize_t aw87xxx_class_gpio_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct aw87xxx_priv *data = container_of(attr, struct aw87xxx_priv, s_gpio);
    int ret;
    char name[64];
    int val;

    ret = sscanf(buf, "%s %d", name, &val);
    printk("%s:cmd buf:%s\n", __func__, buf );

    if (ret != 2) {
        dev_err(&data->i2c->dev, "%s:param is error, echo enable_gpio 1 > this_node.\n", __func__);
        return _count;
    }
    if (0 == strcmp(name, "enable_gpio")) {
        if (!IS_ERR_OR_NULL(data->enable_gpio)) {
            gpiod_set_value(data->enable_gpio, !!val);
        }
    } else if (0 == strcmp(name, "reset_gpio")) {
        if (!IS_ERR_OR_NULL(data->reset_gpio)) {
            dprintk("set reset gpio val = %d.\n", val);
            gpiod_set_value(data->reset_gpio, !!val);
        }
    }

    return _count;
}

static ssize_t aw87xxx_class_mode_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
    ssize_t len = 0;
    char *buf = _buf;

    len += snprintf(buf + len, PAGE_SIZE - len, "0: off mode\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "1: spk mode\n");

    return len;
}

static ssize_t aw87xxx_class_mode_store(struct class *cls, struct class_attribute *attr, const char *buf, size_t _count)
{
    struct aw87xxx_priv *data = container_of(attr, struct aw87xxx_priv, s_mode);

    ssize_t ret;
    unsigned int mode;

    ret = kstrtouint(buf, 10, &mode);
    if (ret) {
        goto out_strtoint;
    }

    switch (mode) {
        case 0:
            aw87xxx_set_audio_amplifier_enable(false);
            break;
        case 1:
            aw87xxx_set_audio_amplifier_enable(true);
            break;
        default: break;
    }

    return _count;
out_strtoint:
    dev_err(&data->i2c->dev, "%s: fail to change str to int\n", __func__);
    return ret;
}

static int aw87xxx_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct aw87xxx_priv *data;
    int ret;

    dev_warn(&i2c->dev, " v20190927 enter %s, the device id: %ld.\n", __func__, id->driver_data);

    data = devm_kzalloc(&i2c->dev, sizeof(*data), GFP_KERNEL);
    if (IS_ERR_OR_NULL(data)) {
        return -ENOMEM;
    }

    data->regmap = devm_regmap_init_i2c(i2c, &aw87xxx_regmap);
    if (IS_ERR(data->regmap)) {
        ret = PTR_ERR(data->regmap);
        dev_err(&i2c->dev, "Failed to allocate register map: %d\n", ret);
        return ret;
    }

    i2c_set_clientdata(i2c, data);
    data->i2c = i2c;
    data->id = (int)id->driver_data;

    aw87xxx_parse_dt(data);

    aw87xxx_set_enabled(data, true);
    aw87xxx_set_hw_on(data, true);
    ret = aw87xxx_test_i2c(data);
    aw87xxx_set_enabled(data, false);
    aw87xxx_set_hw_on(data, false);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw87xxx i2c is failed. please check hard.\n", __func__);
        return -ENODEV;
    }

    g_aw87xxx = data;

    data->cls_node = class_create(THIS_MODULE, AW87xxx_NAME);
    if (IS_ERR_OR_NULL(data->cls_node)) {
        dev_err(&i2c->dev, "failed to class_create htfyun-aw87xxx\n");
    } else {
        CLASS_CREATE_FILE(data, gpio);
        CLASS_CREATE_FILE(data, reg);
        CLASS_CREATE_FILE(data, mode);
    }

    return 0;
}

static int aw87xxx_i2c_remove(struct i2c_client *i2c)
{
    struct aw87xxx_priv *data = i2c_get_clientdata(i2c);

    aw87xxx_set_enabled(data, false);

    if (!IS_ERR(data->cls_node)) {
        CLASS_REMOVE_FILE(data, gpio);
        CLASS_REMOVE_FILE(data, reg);
        CLASS_REMOVE_FILE(data, mode);
        class_destroy(data->cls_node);
    }

    g_aw87xxx = NULL;
    return 0;
}

static void aw87xxx_i2c_shutdown(struct i2c_client *i2c)
{
    struct aw87xxx_priv *data = dev_get_drvdata(&i2c->dev);
    aw87xxx_set_enabled(data, false);
}

#ifdef CONFIG_PM
static int aw87xxx_suspend(struct device *dev)
{
    return 0;
}

static int aw87xxx_resume(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops aw87xxx_pm_ops = {
    .suspend = aw87xxx_suspend,
    .resume = aw87xxx_resume,
};
#endif

static const struct i2c_device_id aw87xxx_i2c_id[] = {
    { AW87519_NAME, 0 },
    { AW87559_NAME, 1 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw87xxx_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id aw87xxx_of_match[] = {
    { .compatible = AW87519_NAME, },
    { .compatible = AW87559_NAME, },
    {},
};
MODULE_DEVICE_TABLE(of, aw87xxx_of_match);
#endif

static struct i2c_driver aw87xxx_i2c_driver = {
    .driver = {
        .name = AW87xxx_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw87xxx_of_match),
#ifdef CONFIG_PM
        //.pm = &aw87xxx_pm_ops,
#endif
    },
    .probe = aw87xxx_i2c_probe,
    .remove = aw87xxx_i2c_remove,
    .shutdown = aw87xxx_i2c_shutdown,
    .id_table = aw87xxx_i2c_id,
};

module_i2c_driver(aw87xxx_i2c_driver);

MODULE_DESCRIPTION("Audio ADC aw87xxx driver");
MODULE_AUTHOR("songshitian <songshitian@htfyun.com>");
MODULE_LICENSE("GPL v2");


