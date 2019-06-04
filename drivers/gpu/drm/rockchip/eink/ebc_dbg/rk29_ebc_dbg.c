/*
 * RK29 ebook control driver rk29_ebc_dbg.c
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: dlx@rock-chips.com
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

#include <linux/kernel.h>

#include "rk29_ebc_dbg.h"

 int gebc_dbg_lev = EBC_ALL;
 int gprint_dir_or_file = 0;


int rk29ebc_dbg_lve_set(const char *buf,int conut)
{
	int pos = 0;

#if 0
	printk("buf conut = %d\n",conut);
	pos = 0;
	while(pos<conut)
	{
		printk(" %x",buf[pos]);
		pos++;
	}
	printk("\n");
#endif

	if(conut >= 5)
	{
		pos = 0;
		while(pos<5)
		{
			if(buf[pos] == '0')
				EBC_CLR_BIT(gprint_dir_or_file, pos);
			else
				EBC_SET_BIT(gprint_dir_or_file, pos);
			pos++;
		}
	}
	
	return 0;
}

int rk29ebc_dbg_lve_show(void)
{
	printk("===============================================\n");
	printk("gebc_dbg_lev = %d, gprint_dir_or_file = 0x%x\n",gebc_dbg_lev,gprint_dir_or_file);
	printk("echo 5 chars, 0:disable 1:enable.\n");
	printk("  bit0:bootani_dir debug info.\n");
	printk("  bit1:bufmanage_dir debug info.\n");
	printk("  bit2:dither_dir debug info.\n");
	printk("  bit3:epdlut_dir debug info.\n");
	printk("  bit4:ebc_c debug info.\n");
	printk("e.g. echo 11101 >\n");  
	printk("epdlut_dir is disable others enable. \n");
	printk("===============================================\n");
	
	return 0;
}


