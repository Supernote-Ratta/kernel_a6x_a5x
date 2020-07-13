/*
 * driver for tusb320
 * Copyright (c) 2020  RATTA
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define TUSB320_SOURCE_PREF_REG  0x0A
#define TUSB320_SOURCE_PREF_MASK  (0b11 << 1)
/*
controls the TUSB320 behavior when configured as a
DRP.
00 – Standard DRP (default)
01 – DRP will perform Try.SNK.
10 – Reserved.
11 – DRP will perform Try.SRC.
*/
#define  TUSB320_SOURCE_PREF_VAL  (0b01 << 1)

struct tusb320 {
	struct i2c_client *client;
	struct regmap *regmap;
};

static bool tusb320_is_volatile_reg(struct device *dev, unsigned int reg);
static const struct regmap_config tusb320_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xA0,
	.cache_type = REGCACHE_NONE,
	.volatile_reg = tusb320_is_volatile_reg,
};

static bool tusb320_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... 0xA0 :
		return true;
	}

	return false;
}

static irqreturn_t tusb320_i2c_irq(int irq, void *dev_id)
{
	pr_info("%s--%d\n", __func__,__LINE__);
	return IRQ_HANDLED;
}

static int tusb320_source_pref_set(struct tusb320 *tusb320 )
{
	struct regmap *map = tusb320->regmap;
	int ret;

	ret = regmap_update_bits(map,TUSB320_SOURCE_PREF_REG,
				    TUSB320_SOURCE_PREF_MASK,
				    TUSB320_SOURCE_PREF_VAL);
	return ret;
}

static int tusb320_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct tusb320 *tusb320;
	unsigned char chip_id[8]={0}; 
	int ret, i, j;
	int buf;
	int irq_gpio,en_n;

	dev_info(&client->dev, "%s--%d\n", __func__,__LINE__);
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA |
				I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "smbus data not supported!\n");
		return -EIO;
	}

	tusb320 = devm_kzalloc(&client->dev, sizeof(*tusb320), GFP_KERNEL);
	if (!tusb320){
		dev_err(&client->dev, "ENOMEM\n");
		return -ENOMEM;
	}
	tusb320->client = client;

	en_n = of_get_named_gpio(np, "en_n", 0);
	if (!gpio_is_valid(en_n)) {
		dev_err(&client->dev, "no en_n pin available\n");
	}
	ret = devm_gpio_request_one(&client->dev, en_n, GPIOF_OUT_INIT_LOW,
				    "tusb320_en_n");
	if (ret < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, ret= %d\n",
			en_n, ret);
	}

	irq_gpio = of_get_named_gpio(np, "typec_int", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(&client->dev, "no en_n pin available\n");
	}

	ret = devm_gpio_request_one(&client->dev, irq_gpio, GPIOF_IN, "typec_int");
	if (ret < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, ret= %d\n", irq_gpio, ret);
	}
#if 0
	gpio_direction_output(en_n, 1);
	msleep(50);
	gpio_direction_output(en_n, 0);
	msleep(10);
#endif
	tusb320->regmap = devm_regmap_init_i2c(client, &tusb320_regmap_config);
	if (IS_ERR(tusb320->regmap)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(tusb320->regmap);
	}

	/* reg 7~0, chip_id ascii "\0TUSB320" */
	for (j=0; j < 3; j++) {
		for(i = 6; i>=0 ;i--) {
			ret = regmap_read(tusb320->regmap,i,&buf);
			if (ret) {
				dev_err(&client->dev, "read reg 0x%x failed ret=%d\n",
					i,ret); 
			}
			chip_id[6-i] = (unsigned char)buf;
		}
		dev_info(&client->dev, "tusb320 chip id = %s\n",chip_id);
		if( !strcmp(chip_id,"TUSB320") )
			break;
	}

	if (j == 3) {
		dev_err(&client->dev, "tusb320 checkid error \n");
		return -ENXIO;
	}

	client->irq = gpio_to_irq(irq_gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n",
			irq_gpio, client->irq);
	}

	ret = request_threaded_irq(client->irq, NULL, tusb320_i2c_irq,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     "tusb320",tusb320);
	if (ret) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", ret);
	}

	ret = tusb320_source_pref_set(tusb320);
	if (ret != 0)
		dev_err(&client->dev, "error set mode ret = %d\n", ret);
	i2c_set_clientdata(client, tusb320);

	return 0;
}

static int tusb320_suspend(struct device *dev)
{
	return 0;
}

static int tusb320_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tusb320_pm_ops = {
	.suspend = tusb320_suspend,
	.resume =  tusb320_resume,
};

static int tusb320_remove(struct i2c_client *client)
{

	return 0;
};

static const struct of_device_id tusb320_of_match[] = {
	{ .compatible = "ti,tusb320", },
	{ },
};
MODULE_DEVICE_TABLE(of, tusb320_of_match);

static const struct i2c_device_id tusb320_id[] = {
	{ "tusb320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb320_id);

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.of_match_table = of_match_ptr(tusb320_of_match),
		.pm = &tusb320_pm_ops,
	},
	.probe    = tusb320_probe,
	.remove   = tusb320_remove,
	.id_table = tusb320_id,
};

module_i2c_driver(tusb320_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jason Tian <jasontian@ratta.com>");
MODULE_DESCRIPTION("TUSB320  driver");
