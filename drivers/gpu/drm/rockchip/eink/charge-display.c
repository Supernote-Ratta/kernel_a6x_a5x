/*
eink charge display
 */

#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/mfd/rk808.h>
#include "ebc.h"

enum ebc_reboot_mode {
	EPD_REBOOT	= 0,
	EPD_POWEROFF	= 1,
};

struct ebc_bat_info{
	int capacity;
	int voltage;
	int exit_charge_level;
};

struct ebc_charge_display {
	struct delayed_work charge_delay_work;
	struct workqueue_struct *charge_wq;
	int reboot_mode;

	struct power_supply *usb_psy;
	struct power_supply *ac_psy;
	struct power_supply *bat_psy;

	struct ebc_bat_info bat;
};

static int ebc_get_usb_psy(struct device *dev, void *data)
{
	struct ebc_charge_display *chr_disp_info = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_USB) {
		chr_disp_info->usb_psy = psy;
		return 1;
	}

	return 0;
}

static int ebc_get_ac_psy(struct device *dev, void *data)
{
	struct ebc_charge_display *chr_disp_info = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS) {
		chr_disp_info->ac_psy = psy;
		return 1;
	}

	return 0;
}

static int ebc_get_bat_psy(struct device *dev, void *data)
{
	struct ebc_charge_display *chr_disp_info = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_BATTERY) {
		chr_disp_info->bat_psy = psy;
		return 1;
	}

	return 0;
}

static void ebc_get_chrg_psy(struct ebc_charge_display *chr_disp_info)
{
	if (!chr_disp_info->usb_psy)
		class_for_each_device(power_supply_class, NULL, (void *)chr_disp_info,
				      ebc_get_usb_psy);
	if (!chr_disp_info->ac_psy)
		class_for_each_device(power_supply_class, NULL, (void *)chr_disp_info,
				      ebc_get_ac_psy);
	if (!chr_disp_info->bat_psy)
		class_for_each_device(power_supply_class, NULL, (void *)chr_disp_info,
				      ebc_get_bat_psy);
}

static int ebc_get_charge_state(struct ebc_charge_display *chr_disp_info)
{
	union power_supply_propval val;
	int ret;
	int usb_in = 0, ac_in = 0;

	if (chr_disp_info->usb_psy) {
		ret = chr_disp_info->usb_psy->desc->get_property(chr_disp_info->usb_psy,
						      POWER_SUPPLY_PROP_ONLINE,
						      &val);
		if (!ret)
			usb_in = val.intval;
	}

	if (chr_disp_info->ac_psy) {
		ret = chr_disp_info->ac_psy->desc->get_property(chr_disp_info->ac_psy,
						     POWER_SUPPLY_PROP_ONLINE,
						     &val);
		if (!ret)
			ac_in = val.intval;
	}

	pr_debug("%s: ac_online=%d, usb_online=%d\n", __func__, ac_in, usb_in);

	return (usb_in || ac_in);
}

static void ebc_get_bat_info(struct ebc_charge_display *chr_disp_info)
{
	int ret;
	union power_supply_propval val;

	if (!chr_disp_info->bat_psy)
		return;

	ret = chr_disp_info->bat_psy->desc->get_property(chr_disp_info->bat_psy,
					     POWER_SUPPLY_PROP_CAPACITY,
					     &val);
	if (!ret)
		chr_disp_info->bat.capacity = val.intval;

	ret = chr_disp_info->bat_psy->desc->get_property(chr_disp_info->bat_psy,
					     POWER_SUPPLY_PROP_VOLTAGE_NOW,
					     &val);
	if (!ret)
		chr_disp_info->bat.voltage = val.intval / 1000;
}

static int ebc_charge_check(struct ebc_charge_display *chr_disp_info)
{
/*
	if (!strstr(saved_command_line, "charger"))
		return 0;
	if (!ebc_get_charge_state(chr_disp_info))
		return 0;
*/
	if (chr_disp_info->bat.exit_charge_level <= 0 ||
	    chr_disp_info->bat.capacity >= chr_disp_info->bat.exit_charge_level)
	    return 0;

	return 1;
}

static void ebc_charge_delay_work(struct work_struct *work)
{
	struct ebc_charge_display *chr_disp_info =
		container_of(work, struct ebc_charge_display, charge_delay_work.work);

	if (chr_disp_info->reboot_mode == EPD_POWEROFF)
		kernel_power_off();
	else
		kernel_restart(NULL);
}

int ebc_charge_disp(void)
{
	struct device_node *temp_np;
	int i = 0;
	int ret = 0;
	u32 out_value;
	struct ebc_charge_display *ebc_chr_disp_info;

	ebc_chr_disp_info = kzalloc(sizeof(*ebc_chr_disp_info), GFP_KERNEL);
	if (!ebc_chr_disp_info) {
		pr_err("%s: kzalloc ebc_chr_disp_info failed\n", __func__);
		return -ENOMEM;
	}
	ebc_chr_disp_info->charge_wq = alloc_ordered_workqueue("%s",
		WQ_MEM_RECLAIM | WQ_FREEZABLE, "ebc-charge-wq");
	if (!ebc_chr_disp_info->charge_wq) {
		pr_err("%s: alloc_ordered_workqueue failed\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	INIT_DELAYED_WORK(&ebc_chr_disp_info->charge_delay_work, ebc_charge_delay_work);

	temp_np = of_find_node_by_name(NULL, "charge-animation");
	if (temp_np) {
		ret = of_property_read_u32(temp_np, "rockchip,uboot-exit-charge-level", &out_value);
		if (ret >= 0) {
			ebc_chr_disp_info->bat.exit_charge_level = out_value;
			pr_info("exit charge level: %d\n", out_value);
		}
	}

	ebc_get_chrg_psy(ebc_chr_disp_info);
	ebc_get_bat_info(ebc_chr_disp_info);
	if (ebc_charge_check(ebc_chr_disp_info)) {
		if (!ebc_get_charge_state(ebc_chr_disp_info)) {
			pr_info("%s: low power and no charger, power off......\n", __func__);
			ebc_chr_disp_info->reboot_mode = EPD_POWEROFF;
			queue_delayed_work(ebc_chr_disp_info->charge_wq,
					&ebc_chr_disp_info->charge_delay_work,
					msecs_to_jiffies(1000));
			return -1;
		}
		rk29_ebc_lowpower_pic_show();
		while (1) {
			ebc_get_bat_info(ebc_chr_disp_info);
			if (ebc_chr_disp_info->bat.capacity >= ebc_chr_disp_info->bat.exit_charge_level)
				goto exit;

			if (!ebc_get_charge_state(ebc_chr_disp_info)) {
				pr_info("%s: low power and charger plug out, poweroff\n", __func__);
				rk29_ebc_show_white_background();
				ebc_chr_disp_info->reboot_mode = EPD_POWEROFF;
				queue_delayed_work(ebc_chr_disp_info->charge_wq,
						   &ebc_chr_disp_info->charge_delay_work,
						   msecs_to_jiffies(1000));
				return -1;
			}
			/*
			if (rk8xx_is_pwrkey_long_press(2000)) {
				dev_info(ebc_info->dev, "long press power key, reboot\n");
				ebc_info->reboot_mode = EPD_REBOOT;
				queue_delayed_work(ebc_info->charge_wq,
						   &ebc_info->charge_delay_work,
						   msecs_to_jiffies(100 * 5));
				ebc_charge_show_background(ebc_info);
				return 1;
			}
			*/
			msleep(500);
			//wake_up_process(ebc_auto_task);
			if ((i++) % 40 == 0)
				pr_info("capacity:%d%%, voltage:%dmV, charging...\n",
					 ebc_chr_disp_info->bat.capacity, ebc_chr_disp_info->bat.voltage);
		}
	}

exit:
	destroy_workqueue(ebc_chr_disp_info->charge_wq);
	kfree(ebc_chr_disp_info);
	return 0;
}
EXPORT_SYMBOL_GPL(ebc_charge_disp);
