#ifndef __RATTA_MT_H__
#define __RATTA_MT_H__
#include <linux/device.h>

int ratta_mt_probe(struct device *dev);
void ratta_mt_remove(void);
int ratta_mt_record(int id, int x, int y, bool state);

#endif
