#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/irq.h>
#include <linux/interrupt.h>


/**
******* dts config ************
*
    htfyun-hall_observe {
        compatible = "htfyun-hall_observe";
        hall-gpio = <&gpio3 RK_PB0 GPIO_ACTIVE_HIGH>;
        status = "okay";
    };

*
*/
#define HALL_OBSERVE_NAME "htfyun-hall_observe"

static int debug = 0;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define dprintk(level, fmt, arg...) do {   \
 if (debug >= level)      \
 printk("**htfyun-hall_observe**%s[%d]: " fmt"\n", __func__, __LINE__, ## arg); } while (0)

#define DebugLog(format, ...) dprintk(1, format, ## __VA_ARGS__)

struct hall_data {
    struct device *dev;
	struct switch_dev sdev;

	struct delayed_work dwork;

    int gpio;
    int on_level;
    int gpio2;
    int on_level2;

    unsigned int irq;
    unsigned int irq2;
    spinlock_t irq_lock;
    bool irq_is_disable;
    spinlock_t irq_lock2;
    bool irq_is_disable2;

    struct class        *cls_node;
    struct class_attribute s_gpio;

};

static int read_gpio(int gpio)
{
	int i,level;
	for(i = 0; i < 3; i++) {
		level = gpio_get_value(gpio);
		if (level >= 0) {
			break;
		}

		printk("%s:get pin level again,pin=%d,i=%d\n", __FUNCTION__, gpio, i);
		msleep(1);

	}
	if(level < 0)
		printk("%s:get pin level  err!\n",__FUNCTION__);

	return level;
}


static void hall_irq_disable(struct hall_data *data)
{
    unsigned long irqflags;

    spin_lock_irqsave(&data->irq_lock, irqflags);
    if (!data->irq_is_disable) {
        data->irq_is_disable = 1;
        disable_irq_nosync(data->irq);
    }
    spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

static void hall_irq_enable(struct hall_data *data)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&data->irq_lock, irqflags);
    if (data->irq_is_disable) {
        enable_irq(data->irq);
        data->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

static void hall_irq_disable2(struct hall_data *data)
{
    unsigned long irqflags;

    spin_lock_irqsave(&data->irq_lock2, irqflags);
    if (!data->irq_is_disable2) {
        data->irq_is_disable2 = 1;
        disable_irq_nosync(data->irq2);
    }
    spin_unlock_irqrestore(&data->irq_lock2, irqflags);
}

static void hall_irq_enable2(struct hall_data *data)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&data->irq_lock2, irqflags);
    if (data->irq_is_disable2) {
        enable_irq(data->irq2);
        data->irq_is_disable2 = 0;
    }
    spin_unlock_irqrestore(&data->irq_lock2, irqflags);
}

static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{
    struct hall_data *data = (struct hall_data *) dev_id;

    hall_irq_disable(data);
    hall_irq_disable2(data);

   if (!delayed_work_pending(&data->dwork)) {
       schedule_delayed_work(&data->dwork, msecs_to_jiffies(50));
   }

    return IRQ_HANDLED;
}

static irqreturn_t hall_irq_handler2(int irq, void *dev_id)
{
    struct hall_data *data = (struct hall_data *) dev_id;

    hall_irq_disable(data);
    hall_irq_disable2(data);

   if (!delayed_work_pending(&data->dwork)) {
       schedule_delayed_work(&data->dwork, msecs_to_jiffies(50));
   }

    return IRQ_HANDLED;
}

static void hall_work(struct work_struct *work)
{
    struct hall_data *data = container_of(to_delayed_work(work), struct hall_data, dwork);
    int level = read_gpio(data->gpio);
    int level2 = read_gpio(data->gpio2);
    if((level == data->on_level)==(level2 == data->on_level2)){
        switch_set_state(&data->sdev, (level == data->on_level));
    }
    irq_set_irq_type(data->irq, level ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
    irq_set_irq_type(data->irq2, level2 ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
    hall_irq_enable(data);
    hall_irq_enable2(data);
}

#ifdef CONFIG_OF
static int hall_parse_dt(struct hall_data *data)
{
    struct device_node *np = data->dev->of_node;
    enum of_gpio_flags flags;
    int level;

    int ret = 0;

    if (!np) {
        dev_err(data->dev, "error: node is NULL!!!\n");
        return -ENODEV;
    }

    data->gpio = of_get_named_gpio_flags(np, "hall-gpio", 0, &flags);
    if (gpio_is_valid(data->gpio)) {

        data->on_level = !(flags & OF_GPIO_ACTIVE_LOW);

        ret = devm_gpio_request_one(data->dev, data->gpio,
                                        GPIOF_DIR_IN,
                                        "hall-gpio");

        if (unlikely(ret < 0)) {

           dev_err(data->dev, "%s:%d: gpio(%d) Failed to request hall-gpio.\n",
                       __func__, __LINE__, data->gpio);

       } else {

            level = read_gpio(data->gpio);
            dev_warn(data->dev, "data->on_level = %d. gpio val = %d.\n", data->on_level, level);
            data->irq = gpio_to_irq(data->gpio);
            ret = devm_request_irq(data->dev,
                    data->irq,
                    hall_irq_handler,
                    level ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING,
                    "hall-irq", data);

            if (unlikely(ret)) {

                dev_err(data->dev, "%s:%d: Failed to request irq.\n",
                       __func__, __LINE__);

            } else {

                hall_irq_disable(data);

            }
       }
    }
    data->gpio2 = of_get_named_gpio_flags(np, "hall2-gpio", 0, &flags);
    if (gpio_is_valid(data->gpio2)) {

        data->on_level2 = !(flags & OF_GPIO_ACTIVE_LOW);

        ret = devm_gpio_request_one(data->dev, data->gpio2,
                                        GPIOF_DIR_IN,
                                        "hall2-gpio");

        if (unlikely(ret < 0)) {

           dev_err(data->dev, "%s:%d: gpio(%d) Failed to request hall2-gpio.\n",
                       __func__, __LINE__, data->gpio2);

       } else {

            level = read_gpio(data->gpio2);
            dev_warn(data->dev, "data->on_level = %d. gpio val = %d.\n", data->on_level2, level);
            data->irq2 = gpio_to_irq(data->gpio2);
            ret = devm_request_irq(data->dev,
                    data->irq2,
                    hall_irq_handler2,
                    level ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING,
                    "hall2-irq", data);

            if (unlikely(ret)) {

                dev_err(data->dev, "%s:%d: Failed to request irq.\n",
                       __func__, __LINE__);

            } else {

                hall_irq_disable2(data);

            }
       }
    }

    return ret;
}

#else
static int hall_parse_dt(struct hall_data *data)
{
    return -ENODEV;
}
#endif

static ssize_t hall_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "hall\n");
}

static ssize_t show_gpio(struct class *cls, struct class_attribute *attr, char *buf)
{
    struct hall_data *data = container_of(attr, struct hall_data, s_gpio);
    ssize_t len = 0;

    len += snprintf(buf + len, PAGE_SIZE - len, "%d %d\n", read_gpio(data->gpio),read_gpio(data->gpio2));

    return len;
}


#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define CLASS_CREATE_FILE(data,_name_) do{ \
        data->s_##_name_.attr.mode = 0444;\
        data->s_##_name_.attr.name = STR(_name_);\
        data->s_##_name_.show = show_##_name_;\
        if (class_create_file(data->cls_node, &data->s_##_name_)) {\
            printk("%s: Fail to creat class file %s\n", __func__, data->s_##_name_.attr.name);\
        }\
    } while(0)

#define CLASS_REMOVE_FILE(data,_name_)       do{ \
        class_remove_file(data->cls_node, &data->s_##_name_);\
    } while(0)


static int hall_probe(struct platform_device *pdev)
{
    struct hall_data *data;
    int ret = 0;

    dev_err(&pdev->dev, "%s: enter !!\n" , __func__ );
    data = (struct hall_data *)devm_kzalloc(&pdev->dev, sizeof(struct hall_data), GFP_KERNEL);
    if (data == NULL) {
        dev_err(&pdev->dev, "leds_data kzalloc failed\n" );
        return -ENOMEM;;
    }

    data->dev = &pdev->dev;

    ret = hall_parse_dt(data);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to parse DTS\n");
        goto error;
    }

    data->sdev.name = "hall";
    data->sdev.print_name = hall_print_name;
    ret = switch_dev_register(&data->sdev);
    if (ret < 0)
        goto error;

    INIT_DELAYED_WORK(&data->dwork, hall_work);
    spin_lock_init(&data->irq_lock);
    spin_lock_init(&data->irq_lock2);

    platform_set_drvdata(pdev, data);

    data->cls_node = class_create(THIS_MODULE, HALL_OBSERVE_NAME);
    if(IS_ERR_OR_NULL(data->cls_node)) {
        dev_err(data->dev, "%s:failed to class_create %s\n" ,__func__ , HALL_OBSERVE_NAME);
    } else {
        CLASS_CREATE_FILE(data, gpio);
    }

    if (!delayed_work_pending(&data->dwork)) {
       schedule_delayed_work(&data->dwork, msecs_to_jiffies(50));
    }

    return 0;

error:

    return ret;

}

static int hall_remove(struct platform_device *pdev)
{
    struct hall_data *data = platform_get_drvdata(pdev);
    if(IS_ERR_OR_NULL(data->cls_node)) {
        CLASS_REMOVE_FILE(data, gpio);
        class_destroy(data->cls_node);
    }
    switch_dev_unregister(&data->sdev);
    platform_set_drvdata(pdev, NULL);
    return 0;
}


static struct of_device_id hall_of_match[] = {
    { .compatible = HALL_OBSERVE_NAME },
    { }
};

MODULE_DEVICE_TABLE(of, hall_of_match);


static struct platform_driver hall_driver = {
    .driver = {
        .name = HALL_OBSERVE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(hall_of_match),
    },
    .probe = hall_probe,
    .remove = hall_remove,
};

module_platform_driver(hall_driver);

MODULE_DESCRIPTION("hall observe DRIVER");
MODULE_LICENSE("GPL");

