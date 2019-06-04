/*
 * RK29 ebook control driver rk29_ebc_dbg.h
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

#ifndef RK29_EBC_DBG_H
#define RK29_EBC_DBG_H

extern int gebc_dbg_lev;
extern int gprint_dir_or_file;

#define 	_module_bootani_dir_ 		(0)
#define 	_module_bufmanage_dir_		(1)
#define 	_module_dither_dir_ 		(2)
#define 	_module_epdlut_dir_			(3)
#define 	_module_ebc_c_ 				(4)

#define EBC_BIT(nr) (1UL << (nr))
#define EBC_SET_BIT(addr,nr) (addr |= EBC_BIT(nr))
#define EBC_CLR_BIT(addr,nr) (addr &= ~(EBC_BIT(nr)))

enum msg_level_e{
	EBC_NO_INFO=0,
	EBC_INFO,
	EBC_ERR,
	EBC_ALL
};

#define ebc_dbg_printk(dir_of_file,lev,fmt)\
				do {\
					if((EBC_BIT(dir_of_file) & gprint_dir_or_file) && (lev <= gebc_dbg_lev)) {\
						printk("ebc " "%s(%d): ", __FUNCTION__, __LINE__);\
						printk fmt;\
					}\
				}while(0)

extern int rk29ebc_dbg_lve_show(void);
extern int rk29ebc_dbg_lve_set(const char *buf,int conut);
#endif //#ifndef RK29_EBC_DBG_H
