#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define RATTA_KERNEL_VERSION "debug-20200529a"

static int version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", RATTA_KERNEL_VERSION);

	return 0;
}

static int version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_proc_show, NULL);
}

static const struct file_operations version_proc_fops = {
	.open		= version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_ratta_init(void)
{
	struct proc_dir_entry *proc_ratta_root;

	proc_ratta_root = proc_mkdir("ratta", NULL);
	if (!proc_ratta_root) {
		printk(KERN_ERR "create ratta proc dir failed\n");
		goto done;
	}

	proc_create("version", S_IRUSR | S_IRGRP | S_IROTH,
		    proc_ratta_root, &version_proc_fops);

done:
	return 0;
}
fs_initcall(proc_ratta_init);
