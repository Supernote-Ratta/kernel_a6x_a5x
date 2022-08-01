/*
 * Rockchip EBOOK waveform driver.
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Yakir Yang <ykk@rock-chips.com>
 *	   Dai Lunxue <dlx@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef HTFY_DUMP_H
#define HTFY_DUMP_H

// 20190909: 主要用来 dump 内核里面的数据。调试使用.
// 我们申请一块 RV-MEM,用来dump 内核的buffer。思路就是把
// 需要dump的 buffer copy 到我们这里的保留分区，记录好数据，
// 然后应用上面可以通过 read 接口来获取到对应的数据。

#define HTFY_DUMP_NAME_LEN 			64

#ifdef CONFIG_HTFY_DUMP
// 返回：0：OK, -ENOMEM：没有空间.
int htfy_dump_put_data(const char* comment, void* data, int size);

int htfy_dump_clear_all_data( void );

void htfy_dump_buff_hex16(const char* prefix, const void * ptr, const int data_len);

bool htfy_dump_record_frame_data(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data);
bool htfy_dump_record_frame_data_8bit(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data);

int htfy_dump_frame_data(void);
#else
#if 0
static inline int htfy_dump_put_data(const char* comment, void* data, int size) {
	return -ENOTSUPP;
}

static inline int htfy_dump_clear_all_data( void ) {
	return 0;
}

static inline void htfy_dump_buff_hex16(const char* prefix, const void * ptr, const int data_len) {}

static inline bool htfy_dump_record_frame_data(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data) {
	return false;
}
static inline bool htfy_dump_record_frame_data_8bit(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data) {
	return false;
}

static inline int htfy_dump_frame_data(void) { return -ENOTSUPP; }
#else 
#define htfy_dump_put_data(comment, data, size)  (-ENOTSUPP)

#define htfy_dump_clear_all_data() 	

#define htfy_dump_buff_hex16(prefix, ptr, data_len)	

#define htfy_dump_record_frame_data(frame, gray_new, gray_old, frame_lut, lcd_data) (false)


#define htfy_dump_record_frame_data_8bit(frame,gray_new,gray_old, frame_lut, lcd_data) (false)
#define htfy_dump_frame_data() 				(-ENOTSUPP)
#endif 
#endif

#endif // HTFY_DUMP_H

