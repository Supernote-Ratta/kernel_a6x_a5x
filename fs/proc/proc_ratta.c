#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>

#include <linux/proc_ratta.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

//#define RATTA_KERNEL_VERSION "release-20210423a"
#define RATTA_KERNEL_VERSION "release-20210730a"

struct ratta_proc {
	const char *name;
	int mode;
	const struct file_operations *ops;
};

static int bootmode = 0, boardid = 0;
static int volatile pen_type = 0;
static int volatile raw_pen_type = 0;
static DEFINE_SPINLOCK(proc_mutex);
static DEFINE_MUTEX(aid_mutex);
static char uboot_version[64];
static int ratta_init_waveform(void);

static int version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", RATTA_KERNEL_VERSION);

	return 0;
}
RATTA_PROC_RO(version);

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

static ssize_t aid_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *off)
{
	if (aid_common_write(buf, count, RATTA_AID_ID))
		return 0;

	return count;
}
RATTA_PROC_RW(aid);

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

static ssize_t usrkey_proc_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *off)
{
	if (aid_common_write(buf, count, RATTA_USRKEY_ID))
		return 0;

	return count;
}
RATTA_PROC_RW(usrkey);

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

static ssize_t said_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *off)
{
	if (aid_common_write(buf, count, RATTA_SAID_ID))
		return 0;

	return count;
}
RATTA_PROC_RW(said);

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
RATTA_PROC_RO(sn);

static int pen_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "G%d\n", pen_type);

	return 0;
}
RATTA_PROC_RO(pen_type);

static int raw_pen_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%02x\n", (unsigned char)raw_pen_type);

	return 0;
}
RATTA_PROC_RO(raw_pen_type);

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
RATTA_PROC_RW(boot_mode);

static int epd_version_proc_show(struct seq_file *m, void *v)
{
	char vcom_str[64], epd_str[64];
	int tmp = 0, base = 0, tail = 0;

	if (!is_rk_vendor_ready())
		return 0;

	memset(vcom_str, 0, sizeof(vcom_str));
	rk_vendor_read(EINK_VCOM_ID, vcom_str, sizeof(vcom_str) - 1);
	printk("epd waveform version: %s\n", vcom_str);
	if (strlen(vcom_str) > 12) {
		memset(epd_str, 0, sizeof(epd_str));
		switch (vcom_str[5]) {
		case '6':
			epd_str[0] = 'C';
			break;
		case '9':
			epd_str[0] = 'G';
			break;
		case 'R':
			epd_str[0] = 'R';
			break;
		default:
			epd_str[0] = '?';
			break;
		}

		if ((vcom_str[7] >= '0') && (vcom_str[7] <= '9'))
			tail = vcom_str[7] - '0';
		else if ((vcom_str[7] >= 'A') && (vcom_str[7] <= 'Z')) {
			goto v2;
		} else
			return 0;

		if ((vcom_str[6] <= '9') && (vcom_str[6] >= '0')) {
			/* 00 ~ 99 */
			tmp = (vcom_str[6] - '0') * 10 + tail;
			snprintf(&epd_str[1], sizeof(epd_str) - 2, "%02d", tmp);
		} else if ((vcom_str[6] <= 'H') && (vcom_str[6] >= 'A')) {
			/* 100 ~ 170 */
			tmp = (vcom_str[6] - 'A') * 10 + 100 + tail;
			snprintf(&epd_str[1], sizeof(epd_str) - 2, "%03d", tmp);
		} else if ((vcom_str[6] <= 'N') && (vcom_str[6] >= 'J')) {
			/* 180 ~ 220 */
			tmp = (vcom_str[6] - 'J') * 10 + 180 + tail;
			snprintf(&epd_str[1], sizeof(epd_str) - 2, "%03d", tmp);
		} else if ((vcom_str[6] <= 'Z') && (vcom_str[6] >= 'Q')) {
			/* 230 ~ 320 */
			tmp = (vcom_str[6] - 'Q') * 10 + 230 + tail;
			snprintf(&epd_str[1], sizeof(epd_str) - 2, "%03d", tmp);
		} else {
			snprintf(&epd_str[1], sizeof(epd_str) - 2, "???");
		}

		goto done;
v2:
		printk("parse new epd bar code.\n");
		tmp = vcom_str[6];
		if ((tmp >= 'Q') && (tmp <= 'Z')) {
			//[Q, Z]
			base = 629 + (tmp - 'Q') * 23;
		} else if (tmp > 'N') {
			//(N, Q)
			return 0;
		} else if (tmp >= 'J') {
			//[J, N]
			base = 514 + (tmp - 'J') * 23;
		} else if (tmp >= 'A') {
			//[A, H)
			base = 330 + (tmp - 'A') * 23;
		} else
			return 0;

		tmp = vcom_str[7];
		if ((tmp >= 'Q') && (tmp <= 'Z')) {
			//[Q, Z]
			tail = tmp - 'A' - 3;
		} else if (tmp > 'N') {
			//(N, Q)
			return 0;
		} else if (tmp >= 'J') {
			//[J, N]
			tail = tmp - 'A' - 1;
		} else if (tmp >= 'A') {
			//[A, H)
			tail = tmp - 'A';
		} else
			return 0;

		snprintf(&epd_str[1], sizeof(epd_str) - 2, "%03d", base + tail);

done:
		seq_printf(m, "%s\n", epd_str);
	}

	return 0;
}
RATTA_PROC_RO(epd_version);

static int board_proc_show(struct seq_file *m, void *v)
{
	char board_str[128] = {0};

	switch (boardid) {
	case 0:
	case 2:
		strncpy(board_str, "Supernote A6 X", sizeof(board_str) - 1);
		break;
	case 1:
		strncpy(board_str, "Supernote A5 X", sizeof(board_str) - 1);
		break;
	default:
		strncpy(board_str, "unknown", sizeof(board_str) - 1);
		break;
	}

	seq_printf(m, "%s\n", board_str);

	return 0;
}
RATTA_PROC_RO(board);

static int idle_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", ratta_has_idle() ? "true" : "false");

	return 0;
}
RATTA_PROC_RO(idle);

static int ultra_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", ratta_has_ultra() ? "true" : "false");

	return 0;
}
RATTA_PROC_RO(ultra);

static int uboot_version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", uboot_version);

	return 0;
}
RATTA_PROC_RO(uboot_version);

#define MAKE_PROC(_name, _mode) \
	{ .name = __stringify(_name), .mode = _mode, .ops = &_name##_proc_fops }

static struct ratta_proc proc_array[] = {
	MAKE_PROC(version, 0444),
	MAKE_PROC(aid, 0660),
	MAKE_PROC(usrkey, 0660),
	MAKE_PROC(said, 0660),
	MAKE_PROC(sn, 0444),
	MAKE_PROC(pen_type, 0444),
	MAKE_PROC(raw_pen_type, 0444),
	MAKE_PROC(boot_mode, 0664),
	MAKE_PROC(epd_version, 0444),
	MAKE_PROC(board, 0444),
	MAKE_PROC(idle, 0444),
	MAKE_PROC(ultra, 0444),
	MAKE_PROC(uboot_version, 0444),
};

static int __init proc_ratta_init(void)
{
	struct proc_dir_entry *proc_ratta_root, *tmp;
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
	} else
		printk(KERN_WARNING "no board id found.\n");

	printk(KERN_INFO "Ratta board: 0x%02x\n", boardid);
	printk(KERN_INFO "Ratta uboot version: %s\n", uboot_version);
	printk(KERN_INFO "Ratta kernel version: %s\n", RATTA_KERNEL_VERSION);

	ratta_init_waveform();

	proc_ratta_root = proc_mkdir("ratta", NULL);
	if (!proc_ratta_root) {
		printk(KERN_ERR "create ratta proc dir failed\n");
		goto done;
	}

	for (i = 0; i < (sizeof(proc_array) / sizeof(proc_array[0])); i++) {
		tmp = proc_create(proc_array[i].name, proc_array[i].mode,
				  proc_ratta_root, proc_array[i].ops);
		if (!tmp)
			printk(KERN_ERR "create proc %s failed.\n", proc_array[i].name);
	}

done:
	return 0;
}
fs_initcall(proc_ratta_init);

void ratta_save_bootlog(void *addr, int len)
{
	char *ptr = addr;

	ptr[len - 1] = 0;

	printk(KERN_INFO "------------------------------uboot battery log------------------------------\n");
	printk(KERN_INFO "%s\n", ptr);
	printk(KERN_INFO "-----------------------------------------------------------------------------\n");
}

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

int ratta_get_board(void)
{
	return boardid;
}

struct ratta_waveform {
	const char *no;
	const char *filename;
};

static const struct ratta_waveform a5x_waveforms[] = {
	{ "R137", "waveform_a5x_r137.bin" },
	{ "R171", "waveform_a5x_r171.bin" },
	{ "R244", "waveform_a5x_r244.bin" },
	{ "R371", "waveform_a5x_r371.bin" },
	{ "R245", "waveform_a5x_r245.bin" },
	{ "R250", "waveform_a5x_r250.bin" },
	{ "R346", "waveform_a5x_r346.bin" },
	{ "R350", "waveform_a5x_r350.bin" },
	{ "R360", "waveform_a5x_r360.bin" },
};

static const struct ratta_waveform a6x_waveforms[] = {
	{ "R239", "waveform_a6x_r239.bin" },
	{ "R248", "waveform_a6x_r248.bin" },
	{ "R281", "waveform_a6x_r281.bin" },
	{ "R305", "waveform_a6x_r305.bin" },
	{ "R317", "waveform_a6x_r317.bin" },
	{ "R348", "waveform_a6x_r348.bin" },
	{ "R354", "waveform_a6x_r354.bin" },
};

static const char *DEFAULT_A5X_WAVEFORM = "waveform_a5x_r244.bin";
static const char *DEFAULT_A6X_WAVEFORM = "waveform_a6x_r239.bin";
static const char *DEFAULT_XXX_WAVEFORM = "";
static const char *g_waveform = NULL;
static const char *EPD_BATCH = NULL;
static char EPD_BATCH_ROOM[64] = {0};

static int ratta_init_waveform(void)
{
	const struct ratta_waveform *ptr = NULL;
	int size = 0, i;

	switch (boardid) {
	case BOARD_A5X:
		ptr = &(a5x_waveforms[0]);
		size = sizeof(a5x_waveforms) / sizeof(a5x_waveforms[0]);
		g_waveform = DEFAULT_A5X_WAVEFORM;
		break;
	case BOARD_A6X:
	case BOARD_A6X2:
		ptr = &(a6x_waveforms[0]);
		size = sizeof(a6x_waveforms) / sizeof(a6x_waveforms[0]);
		g_waveform = DEFAULT_A6X_WAVEFORM;
		break;
	default:
		g_waveform = DEFAULT_XXX_WAVEFORM;
		break;
	}

	if (!EPD_BATCH)
		return 0;

	for (i = 0; i < size; i++) {
		if (!strcmp(ptr[i].no, EPD_BATCH)) {
			g_waveform = ptr[i].filename;
			break;
		}
	}

	return 0;
}

static int ratta_setup_waveform(char *options)
{
	printk("[EINK]: epd='%s'\n", options);
	strncpy(EPD_BATCH_ROOM, options, sizeof(EPD_BATCH_ROOM) - 1);
	EPD_BATCH = EPD_BATCH_ROOM;

	return 0;
}
__setup("epd=", ratta_setup_waveform);

const char *ratta_get_waveform(void)
{
	printk("[EINK]: use %s\n", g_waveform);

	return g_waveform ? g_waveform : DEFAULT_XXX_WAVEFORM;
}

bool ratta_has_ultra(void)
{
	return (boardid == BOARD_A5X) ||
		(boardid == BOARD_A6X2);
}

bool ratta_has_idle(void)
{
	return (boardid == BOARD_A5X) ||
		(boardid == BOARD_A6X2);
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

static int ratta_uboot_version_setup(char *options)
{
	memset(uboot_version, 0, sizeof(uboot_version));
	strncpy(uboot_version, options, sizeof(uboot_version) - 1);

	return 0;
}
__setup("ratta.uboot.version=", ratta_uboot_version_setup);
