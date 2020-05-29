#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>

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

static int aid_proc_show(struct seq_file *m, void *v)
{
	char aid_str[64];

	if (!is_rk_vendor_ready())
		return 0;

	memset(aid_str, 0, sizeof(aid_str));
	rk_vendor_read(RATTA_ANDROID_ID, aid_str, 16);
	if (strlen(aid_str))
		seq_printf(m, "%s\n", aid_str);

	return 0;
}

static int aid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, aid_proc_show, NULL);
}

static ssize_t aid_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *off)
{
	char aid_str[64], *pstr, *tmp;

	if (!is_rk_vendor_ready())
		return 0;

	memset(aid_str, 0, sizeof(aid_str));
	if (copy_from_user(aid_str, buf, (count < sizeof(aid_str)) ?
			   count : sizeof(aid_str) - 1))
		return 0;

	pstr = strchr(aid_str, '\n');
	if (pstr)
		*pstr = 0;
	pstr = strchr(aid_str, '\r');
	if (pstr)
		*pstr = 0;

	pstr = aid_str;
	while (((pstr - aid_str) < (sizeof(aid_str) - 1)) &&
	       ((*pstr == ' ') || (*pstr == '\t'))) pstr++;
	tmp = strchr(pstr, ' ');
	if (tmp)
		*tmp = 0;
	if (strlen(pstr) != 16)
		return -EINVAL;

	rk_vendor_write(RATTA_ANDROID_ID, pstr, strlen(pstr));

	return strlen(pstr);
}

static const struct file_operations aid_proc_fops = {
	.open		= aid_proc_open,
	.read		= seq_read,
	.write		= aid_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sn_proc_show(struct seq_file *m, void *v)
{
	char sn_str[64];

	if (!is_rk_vendor_ready())
		return 0;

	memset(sn_str, 0, sizeof(sn_str));
	rk_vendor_read(SN_ID, sn_str, sizeof(sn_str) - 1);
	if (strlen(sn_str))
		seq_printf(m, "%s\n", sn_str);

	return 0;
}

static int sn_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sn_proc_show, NULL);
}

static const struct file_operations sn_proc_fops = {
	.open		= sn_proc_open,
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
	proc_create("aid", 0660, proc_ratta_root, &aid_proc_fops);
	proc_create("sn", 0444, proc_ratta_root, &sn_proc_fops);

done:
	return 0;
}
fs_initcall(proc_ratta_init);
