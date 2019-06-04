/*
 * RK29 ebook zip epd_zip.h
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

#ifndef EPD_ZIP_H
#define EPD_ZIP_H

void zip_decode(unsigned char *pdbuf,unsigned char * psbuf,int n);

 void zip_init(unsigned int * table,int n);
#endif
