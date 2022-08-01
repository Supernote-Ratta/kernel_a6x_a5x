/*
 * sgm37604a.h - SGM37604A LEDs Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SGM37604A_H__
#define __SGM37604A_H__

#define SGM37604A_RST 0x01
#define SGM37604A_ENABLE 0x10
#define SGM37604A_BRIGHTNESS_CONTROL 0x11
#define SGM37604A_BRIGHTNESS_L 0x1A
#define SGM37604A_BRIGHTNESS_M 0x19
#define SGM37604A_FLAG 0x1F
#define SGM37604A_LED_CURRENT 0x1B
struct sgm37604a_platform_data {
	struct device *fbdev;
	unsigned int max_value;
	unsigned int def_value;
};


#endif
