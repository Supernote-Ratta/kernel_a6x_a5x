#ifndef __PROC_RATTA_H__
#define __PROC_RATTA_H__

enum {
	RATTA_MODE_NORMAL,
	RATTA_MODE_FACTORY,
};

int ratta_get_bootmode(void);

#endif
