/*
 * RK29 ebook temperature get  epd_temperature.h
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

#ifndef EPD_TEMPERATURE_H
#define EPD_TEMPERATURE_H

struct epd_ts_ops {
	void (*wakeup)(void);
	void (*shutdown)(void);
	struct module *owner;
};

struct epd_ts {
	struct epd_ts_ops	*ops;
	char *name;
};

/*api*/
extern int epd_get_temperature(int *temperature);
extern int epd_temp_sensor_shutdown(void);
extern int epd_temp_sensor_wakeup(void);

 
#endif

