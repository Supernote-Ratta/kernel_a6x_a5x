#ifndef __PROC_RATTA_H__
#define __PROC_RATTA_H__
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

enum {
	RATTA_MODE_NORMAL,
	RATTA_MODE_FACTORY,
};

enum {
	BOARD_A6X = 0,
	BOARD_A5X,
	BOARD_A6X2,
};

int ratta_get_bootmode(void);
void ratta_set_pen_type(int);
void ratta_set_raw_pen_type(int);
int ratta_get_board(void);
bool ratta_has_ultra(void);
bool ratta_has_idle(void);

#define RATTA_PROC_ATTR(_name, _show, _store) \
	static int _name##_proc_open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show, NULL); \
	} \
	static const struct file_operations _name##_proc_fops = { \
		.open = _name##_proc_open, \
		.read = seq_read, \
		.write = _store, \
		.llseek = seq_lseek, \
		.release = single_release, \
	};

#define RATTA_PROC_RO(name) RATTA_PROC_ATTR(name, name##_proc_show, NULL)
#define RATTA_PROC_WO(name) RATTA_PROC_ATTR(name, NULL, name##_proc_write)
#define RATTA_PROC_RW(name) RATTA_PROC_ATTR(name, name##_proc_show, name##_proc_write)

#endif
