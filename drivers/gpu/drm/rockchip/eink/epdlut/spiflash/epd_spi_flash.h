/*
 * RK29 ebook spi falsh driver epd_spi_falsh.h
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

#ifndef EPD_SPI_FLASH_H
#define EPD_SPI_FLASH_H

/*define */
 #define EPD_SPI_DEBUG (0)
#define SPI_PAGE_SIZE   16    //一次写16B
#define FLASH_PAGE_SIZE  65536//65536  //spi flash 一页的大小 256B
#define FLASH_GLOBAl_SIZE 32*65536
#define FLASH_PAGE_EARSE 4096

#define     FPAGE_SIZE                256
#define     NOR_PAGE_SIZE          256
#define     NOR_SECTOR_SIZE     4096
#define     NOR_BLOCK_SIZE       1024*64
#define     READ_DATA              0x03
#define     BYTE_WRITE             0x02
#define     WRITE_ENABLE         0x06
#define     WRITE_DISABLE       0x04
#define     READ_AD                  0x90
#if defined(CONFIG_PVI_WAVEFORM)
#define WFM_ADDR             0x00886     // See AM300_MMC_IMAGE_X03a/source/broadsheet_soft/bs60_init/run_bs60_init.sh.
#define CMD_ADDR             0x00000     // Base of flash holds the commands (0x00000...(WFM_ADDR - 1)).
#define PNL_ADDR             0x30000     // Start of panel data.
#define TST_ADDR_128K        0x1E000     // Test area (last 8K of 128K).
#define TST_ADDR_256K        0x3E000     // Test area (last 8K of 256K).

#define CMD_SIZE             (WFM_ADDR - CMD_ADDR)
#define WFM_SIZE             (PNL_ADDR - WFM_ADDR)
#define WFM_HDR_SIZE         (0x30)

#define PNL_BASE_PART_NUMBER 0x00
#define PNL_SIZE_PART_NUMBER 16

#define PNL_BASE_VCOM        0x10
#define PNL_SIZE_VCOM        5
#define PNL_SIZE_VCOM_STR    (PNL_SIZE_VCOM + 1)

#define PNL_BASE_WAVEFORM    0x20
#define PNL_SIZE_WAVEFORM    23

#define PNL_BASE_FPL         0x40
#define PNL_SIZE_FPL         3

#define PNL_BASE_BCD         0x50
#define PNL_SIZE_BCD         32
#define PNL_SIZE_BCD_STR     (PNL_SIZE_BCD + 1)

#define PNL_BASE             0x00
#define PNL_SIZE             256
#define PNL_LAST             (PNL_SIZE - 1)

#define PNL_FLASH_BASE       PNL_ADDR

#define PNL_CHAR_UNKNOWN     '!'
#endif
struct epd_spi_flash_info{
	struct class *my_class;
	struct cdev *cdev;

	struct spi_device *dev;
};

#endif
