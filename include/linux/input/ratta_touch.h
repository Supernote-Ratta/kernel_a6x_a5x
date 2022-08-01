#ifndef __RATTA_TOUCH_H__
#define __RATTA_TOUCH_H__

enum {
	RATTA_TOUCH_IC_NONE,
	RATTA_TOUCH_IC_ATMEL,
	RATTA_TOUCH_IC_GOODIX,
	RATTA_TOUCH_IC_CYTTSP5,
	RATTA_TOUCH_IC_PARADE,
    RATTA_TOUCH_IC_FT5XX,
};

bool ratta_touch_exist(int ic);
void ratta_touch_set_ic(int ic);

#endif
