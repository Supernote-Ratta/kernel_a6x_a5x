#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>

#include <linux/proc_ratta.h>

#define RATTA_KERNEL_VERSION "debug-20200724a"

static int bootmode = 0;
static int volatile pen_type = 0;
static int volatile raw_pen_type = 0;
static DEFINE_SPINLOCK(proc_mutex);
static DEFINE_MUTEX(aid_mutex);

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

static int aid_common_write(const char __user *buf, size_t count, int id)
{
	char aid_str[72], *pstr, *tmp;
	int i, ret;

	if (!is_rk_vendor_ready())
		return -ENODEV;

	mutex_lock(&aid_mutex);

	memset(aid_str, 0, sizeof(aid_str));
	rk_vendor_read(id, aid_str, sizeof(aid_str) - 1);
	if ((strlen(aid_str) == 16) ||
	    (strlen(aid_str) == 32) ||
	    (strlen(aid_str) == 64)) {
		mutex_unlock(&aid_mutex);

		return 0;
	}

	memset(aid_str, 0, sizeof(aid_str));
	ret = copy_from_user(aid_str, buf, (count < sizeof(aid_str)) ?
			     count : sizeof(aid_str) - 1);
	if (ret) {
		mutex_unlock(&aid_mutex);
		printk(KERN_ERR "copy aid(%d) from user failed,%d\n", id, ret);

		return ret;
	}

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
	if ((strlen(pstr) != 16) &&
	    (strlen(pstr) != 32) &&
	    (strlen(pstr) != 64)) {
		mutex_unlock(&aid_mutex);

		return -EINVAL;
	}

	for (i = 0; i < strlen(pstr); i++) {
		if (!(((pstr[i] <= '9') && (pstr[i] >= '0')) ||
		    ((pstr[i] <= 'f') && (pstr[i] >= 'a')) ||
		    ((pstr[i] <= 'F') && (pstr[i] >= 'A')))) {
			mutex_unlock(&aid_mutex);

			return -EINVAL;
		}
	}

	rk_vendor_write(id, pstr, strlen(pstr));

	mutex_unlock(&aid_mutex);

	return 0;
}

static int aid_proc_show(struct seq_file *m, void *v)
{
	char aid_str[72];

	if (!is_rk_vendor_ready())
		return 0;

	memset(aid_str, 0, sizeof(aid_str));
	rk_vendor_read(RATTA_AID_ID, aid_str, sizeof(aid_str) - 1);
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
	if (aid_common_write(buf, count, RATTA_AID_ID))
		return 0;

	return count;
}

static const struct file_operations aid_proc_fops = {
	.open		= aid_proc_open,
	.read		= seq_read,
	.write		= aid_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int usrkey_proc_show(struct seq_file *m, void *v)
{
	char aid_str[72];

	if (!is_rk_vendor_ready())
		return 0;

	memset(aid_str, 0, sizeof(aid_str));
	rk_vendor_read(RATTA_USRKEY_ID, aid_str, sizeof(aid_str) - 1);
	if (strlen(aid_str))
		seq_printf(m, "%s\n", aid_str);

	return 0;
}

static int usrkey_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, usrkey_proc_show, NULL);
}

static ssize_t usrkey_proc_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *off)
{
	if (aid_common_write(buf, count, RATTA_USRKEY_ID))
		return 0;

	return count;
}

static const struct file_operations usrkey_proc_fops = {
	.open		= usrkey_proc_open,
	.read		= seq_read,
	.write		= usrkey_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int said_proc_show(struct seq_file *m, void *v)
{
	char aid_str[72];

	if (!is_rk_vendor_ready())
		return 0;

	memset(aid_str, 0, sizeof(aid_str));
	rk_vendor_read(RATTA_SAID_ID, aid_str, sizeof(aid_str) - 1);
	if (strlen(aid_str))
		seq_printf(m, "%s\n", aid_str);

	return 0;
}

static int said_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, said_proc_show, NULL);
}

static ssize_t said_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *off)
{
	if (aid_common_write(buf, count, RATTA_SAID_ID))
		return 0;

	return count;
}

static const struct file_operations said_proc_fops = {
	.open		= said_proc_open,
	.read		= seq_read,
	.write		= said_proc_write,
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

static int pen_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "G%d\n", pen_type);

	return 0;
}

static int pen_type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pen_type_proc_show, NULL);
}

static const struct file_operations pen_type_proc_fops = {
	.open		= pen_type_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int raw_pen_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%02x\n", (unsigned char)raw_pen_type);

	return 0;
}

static int raw_pen_type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, raw_pen_type_proc_show, NULL);
}

static const struct file_operations raw_pen_type_proc_fops = {
	.open		= raw_pen_type_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int boot_mode_proc_show(struct seq_file *m, void *v)
{
	switch (bootmode) {
	case RATTA_MODE_NORMAL:
		seq_printf(m, "%s\n", "normal");
		break;
	case RATTA_MODE_FACTORY:
		seq_printf(m, "%s\n", "factory");
		break;
	default:
		seq_printf(m, "%s\n", "unknown");
		break;
	}

	return 0;
}

static int boot_mode_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_mode_proc_show, NULL);
}

static ssize_t boot_mode_proc_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *off)
{
	int ret;
	char tmp[64];

	memset(tmp, 0, sizeof(tmp));
	ret = copy_from_user(tmp, buf, (count < sizeof(tmp)) ?
			     count : sizeof(tmp) - 1);
	if (ret) {
		printk(KERN_ERR "Copy boot mode from user failed,%d\n", ret);
		return 0;
	}

	spin_lock(&proc_mutex);
	if (!strncmp(tmp, "normal", strlen("normal")))
		bootmode = RATTA_MODE_NORMAL;
	else if (!strncmp(tmp, "factory", strlen("factory")))
		bootmode = RATTA_MODE_FACTORY;
	spin_unlock(&proc_mutex);

	return count;
}

static const struct file_operations boot_mode_proc_fops = {
	.open		= boot_mode_proc_open,
	.read		= seq_read,
	.write		= boot_mode_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_ratta_init(void)
{
	struct proc_dir_entry *proc_ratta_root, *tmp;

	proc_ratta_root = proc_mkdir("ratta", NULL);
	if (!proc_ratta_root) {
		printk(KERN_ERR "create ratta proc dir failed\n");
		goto done;
	}

	tmp = proc_create("version", S_IRUSR | S_IRGRP | S_IROTH,
			  proc_ratta_root, &version_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc version failed\n");
	tmp = proc_create("aid", 0660, proc_ratta_root, &aid_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc aid failed\n");
	tmp = proc_create("usrkey", 0660, proc_ratta_root, &usrkey_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc usrkey failed\n");
	tmp = proc_create("said", 0660, proc_ratta_root, &said_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc said failed\n");
	tmp = proc_create("sn", 0444, proc_ratta_root, &sn_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc sn failed\n");
	tmp = proc_create("pen_type", 0444, proc_ratta_root, &pen_type_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc pen_type failed\n");
	tmp = proc_create("raw_pen_type", 0444, proc_ratta_root, &raw_pen_type_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc raw_pen_type failed\n");
	tmp = proc_create("boot_mode", 0644, proc_ratta_root, &boot_mode_proc_fops);
	if (!tmp)
		printk(KERN_ERR "Create proc boot_mode failed\n");

done:
	return 0;
}
fs_initcall(proc_ratta_init);

int ratta_get_bootmode(void)
{
	return bootmode;
}

void ratta_set_pen_type(int pen)
{
	spin_lock(&proc_mutex);
	if (pen_type != pen)
		pen_type = pen;
	spin_unlock(&proc_mutex);
}

void ratta_set_raw_pen_type(int pen)
{
	spin_lock(&proc_mutex);
	if (raw_pen_type != pen)
		raw_pen_type = pen;
	spin_unlock(&proc_mutex);
}

static int ratta_bootmode_setup(char *options)
{
	if (!strncmp(options, "normal", strlen("normal"))) {
		bootmode = RATTA_MODE_NORMAL;
	} else if (!strncmp(options, "factory", strlen("factory"))) {
		bootmode = RATTA_MODE_FACTORY;
	}

	return 0;
}

__setup("ratta.bootmode=", ratta_bootmode_setup);
