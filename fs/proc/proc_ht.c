#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include <linux/proc_ht.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


struct ht_proc {
	const char *name;
	int mode;
	const struct file_operations *ops;
};

static int boardid = 0;
char board_str[128] = {0};

static int board_proc_show(struct seq_file *m, void *v)
{

	seq_printf(m, "%s\n", board_str);

	return 0;
}
HT_PROC_RO(board);

#define MAKE_PROC(_name, _mode) \
	{ .name = __stringify(_name), .mode = _mode, .ops = &_name##_proc_fops }

static struct ht_proc proc_array[] = {
	MAKE_PROC(board, 0444),
};

static int __init proc_ht_init(void)
{
	struct proc_dir_entry *proc_ht_root, *tmp;
	struct device_node *np = NULL;
	const char *id = NULL;
	int i;

	np = of_find_node_by_path("/hardware");
	if (np) {
		of_property_read_string(np, "id", &id);
		if (id) {
			if (!strcmp(id, "a5x"))
				boardid = BOARD_A5X;
			else if (!strcmp(id, "a6x2"))
				boardid = BOARD_A6X2;
			else if (!strcmp(id, "a6x"))
				boardid = BOARD_A6X;
		}
	} else{
		boardid = BOARD_HT;
		printk(KERN_WARNING "no board id found.\n");
		return 0;
	}
	//strcp(board_str, id);
	strncpy(board_str, id, sizeof(board_str) - 1);
	printk(KERN_INFO "Ht board: 0x%02x\n", boardid);

	proc_ht_root = proc_mkdir("ht", NULL);
	if (!proc_ht_root) {
		printk(KERN_ERR "create ht proc dir failed\n");
		goto done;
	}

	for (i = 0; i < (sizeof(proc_array) / sizeof(proc_array[0])); i++) {
		tmp = proc_create(proc_array[i].name, proc_array[i].mode,
				  proc_ht_root, proc_array[i].ops);
		if (!tmp)
			printk(KERN_ERR "create proc %s failed.\n", proc_array[i].name);
	}

done:
	return 0;
}
fs_initcall(proc_ht_init);

