#ifndef __PROC_HT_H__
#define __PROC_HT_H__
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

enum {
	HT_MODE_NORMAL,
	HT_MODE_FACTORY,
};

enum {
	BOARD_A6X = 0,
	BOARD_A5X,
	BOARD_A6X2,
	BOARD_HT,
};

int ratta_get_bootmode(void);
void ratta_set_pen_type(int);
void ratta_set_raw_pen_type(int);
int ratta_get_board(void);
bool ratta_has_ultra(void);
bool ratta_has_idle(void);

#define HT_PROC_ATTR(_name, _show, _store) \
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

#define HT_PROC_RO(name) HT_PROC_ATTR(name, name##_proc_show, NULL)
#define HT_PROC_WO(name) HT_PROC_ATTR(name, NULL, name##_proc_write)
#define HT_PROC_RW(name) HT_PROC_ATTR(name, name##_proc_show, name##_proc_write)

#endif
