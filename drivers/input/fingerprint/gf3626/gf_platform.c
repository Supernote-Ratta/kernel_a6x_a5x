#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "gf_common.h"

extern u8 g_debug_level;

/*GPIO pins reference.*/
int gf_get_gpio_dts_info(struct gf_device *gf_dev)
{
	int rc = 0;
    /*get pwr resource*/
	gf_dev->cs_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_pwr", 0);
	if (!gpio_is_valid(gf_dev->cs_gpio)) {
		gf_debug(ERR_LOG, "%s, PWR GPIO is invalid.\n", __func__);
		//return -1;
	}
	gf_debug(DEBUG_LOG, "%s, gf:goodix_pwr:%d\n", __func__, gf_dev->cs_gpio);
	if (gpio_is_valid(gf_dev->cs_gpio)) {
		rc = gpio_request(gf_dev->cs_gpio, "goodix_pwr");
		if (rc) {
			gf_debug(ERR_LOG, "%s, Failed to request PWR GPIO. rc = %d\n", __func__, rc);
			//return -1;
		}
	}
    /*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		gf_debug(ERR_LOG, "%s, RESET GPIO is invalid.\n", __func__);
		return -1;
	}
	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		gf_debug(ERR_LOG, "%s, Failed to request RESET GPIO. rc = %d\n", __func__, rc);
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

    /*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	gf_debug(DEBUG_LOG, "%s, gf:irq_gpio:%d\n", __func__, gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		gf_debug(ERR_LOG, "%s, IRQ GPIO is invalid.\n", __func__);
		return -1;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		gf_debug(ERR_LOG, "%s, Failed to request IRQ GPIO. rc = %d\n", __func__, rc);
		return -1;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	return 0;
}

void gf_cleanup_info(struct gf_device *gf_dev)
{
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		gf_debug(DEBUG_LOG, "%s, remove irq_gpio success\n", __func__);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		gf_debug(DEBUG_LOG, "%s, remove reset_gpio success\n", __func__);
	}
/*	if (gpio_is_valid(gf_dev->cs_gpio)) {
		gpio_free(gf_dev->cs_gpio);
		gf_debug(DEBUG_LOG, "%s, remove reset_gpio success\n", __func__);
	}
*/
}

void gf_hw_power_enable(struct gf_device *gf_dev,u8 onoff)
{
	/* TODO: LDO configure */
	static int enable = 1;
	if (gf_dev == NULL) {
		gf_debug(ERR_LOG, "%s, Input buff is NULL.\n", __func__);
		return;
	}

	if (onoff && enable)
		enable = 0;
	else if (!onoff && !enable)
		enable = 1;
	if (gpio_is_valid(gf_dev->cs_gpio)) {
		if(onoff){
			gpio_direction_output(gf_dev->cs_gpio, 1);
			gpio_set_value(gf_dev->cs_gpio, 1);
		}
		else{
			gpio_direction_output(gf_dev->cs_gpio, 1);
			gpio_set_value(gf_dev->cs_gpio, 0);
		}
	}
}

void gf_hw_reset(struct gf_device *gf_dev, u8 delay)
{
	if (gf_dev == NULL) {
		gf_debug(ERR_LOG, "%s, Input buff is NULL.\n", __func__);
		return;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay);
}

void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
	if (gf_dev == NULL) {
		gf_debug(ERR_LOG, "%s, Input buff is NULL.\n", __func__);
		return;
	}
	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
}

void gf_spi_clk_enable(struct gf_device *gf_dev, u8 bonoff)
{
	static int count;

	if (bonoff && (count == 0))
		count = 1;
	else if ((count > 0) && (bonoff == 0))
		count = 0;
}
