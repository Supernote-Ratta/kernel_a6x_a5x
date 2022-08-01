/*
 * RK29 ebook control driver rk29_ebc.c
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: Dai Lunxue <dlx@rock-chips.com>
 *         Huang Lin <hl@rock-chips.com>
 *         Yang Kuankuan <ykk@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
//#include <linux/debugfs.h>
//#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/device.h>
//#include <linux/delay.h>
//#include <linux/init.h>
//#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
//#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
//#include <linux/nvmem-consumer.h>
//#include <linux/firmware.h>

#include <linux/htfy_dbg.h>

// 20191106:需要的时候我们直接通过命令 echo 65535 > /sys/module/htfy_dbg/parameters/ht_ebc_dbg_bits 来设置就可以了。
//  cat /sys/module/htfy_dbg/parameters/ht_ebc_dbg_bits
#define DEFUALT_DBG_BITS 		0 //	EDBG_USER|IDBG_WACOM|EDBG_BUF|EDBG_AUTO //(0X3F|EDBG_INIT)
	//(EDBG_LUT|EDBG_INIT|EDBG_IQRFRAME|EDBG_WARN|EDBG_USER|EDBG_LCDC|EDBG_AUTO|EDBG_POWER)

int ht_ebc_dbg_bits = DEFUALT_DBG_BITS;
module_param(ht_ebc_dbg_bits, int, S_IRUGO|S_IWUSR);

/*------------------------------------------------------------------------*/
#if 0
static int htfy_dbg_probe(struct platform_device *pdev)
{
	struct device 				*dev = &pdev->dev;
	printk("%s:dev=%s,do nothing\n", __func__, dev_name(dev));
	return 0;
}


static const struct of_device_id htfy_dbg_match[] = {
	{ .compatible = "htfy_dbg", },
	{ },
};
MODULE_DEVICE_TABLE(of, htfy_dbg_match);

static struct platform_driver htfy_dbg_driver = {
	.probe  = htfy_dbg_probe,
	//.remove = htfy_dbg_remove,
	.driver = {
		.name = "htfy_dbg",
		//.pm = &rk_eink_pm,
		.of_match_table = htfy_dbg_match,
	},
};

static int __init htfy_dbg_init(void)
{
	return platform_driver_register(&htfy_dbg_driver);
}

static void __exit htfy_dbg_exit(void)
{
	return platform_driver_unregister(&htfy_dbg_driver);
}

device_initcall(htfy_dbg_init);  // early then ebc-init.
//device_initcall_sync(htfy_dump_init);

module_exit(htfy_dbg_exit);
#endif

