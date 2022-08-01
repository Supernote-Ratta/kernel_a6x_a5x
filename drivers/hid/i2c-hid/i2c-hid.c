/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/rk_keys.h>
#include <linux/wakelock.h>  //20210727,add for fw-updating.

//20180205,hsl add for gpio.
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/htfy_dbg.h>
#include <linux/i2c/i2c-hid.h>

/* flags */
#define I2C_HID_STARTED		0
#define I2C_HID_RESET_PENDING	1
#define I2C_HID_READ_PENDING	2

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

/* debug option */
// 20210601: echo 1 > /sys/module/hid/parameters/debug
static bool debug = 0;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define i2c_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug)							  \
		dev_printk(KERN_DEBUG, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
	} __packed c;
};

#define I2C_HID_CMD(opcode_) \
	.opcode = opcode_, .length = 4, \
	.registerIndex = offsetof(struct i2c_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct i2c_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
static const struct i2c_hid_cmd hid_report_descr_cmd = {
		.registerIndex = offsetof(struct i2c_hid_desc,
			wReportDescRegister),
		.opcode = 0x00,
		.length = 2 };
/* commands */
static const struct i2c_hid_cmd hid_reset_cmd =		{ I2C_HID_CMD(0x01),
							  .wait = true };
static const struct i2c_hid_cmd hid_get_report_cmd =	{ I2C_HID_CMD(0x02) };
static const struct i2c_hid_cmd hid_set_report_cmd =	{ I2C_HID_CMD(0x03) };
static const struct i2c_hid_cmd hid_set_power_cmd =	{ I2C_HID_CMD(0x08) };
static const struct i2c_hid_cmd hid_no_cmd =		{ .length = 0 };

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

static DEFINE_MUTEX(i2c_hid_open_mut);

/* The main device structure */
struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */
	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* i2c buffer size */
	u8			*inbuf;		/* Input buffer */
	u8			*rawbuf;	/* Raw Input buffer */
	u8			*cmdbuf;	/* Command buffer */
	u8			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */

	wait_queue_head_t	wait;		/* For waiting the interrupt */
	struct gpio_desc	*desc;
	int			irq;

	struct i2c_hid_platform_data pdata;

	struct notifier_block fb_notif;

	// 20210727: 防止正在更新EMR-FW 的时候用户按了 Power 按键进入休眠导致升级失败。
	struct wake_lock 	fw_lock;
	bool     is_suspend;  // 202010427: 用来表示电源状态.true: 下电，中断disabled.

	// 20210726,hsl add for fw-update.
	bool 	is_fwupdating;

	bool	irq_wake_enabled;
    // 20210601: 亮屏休眠的时候会出现第一次点击无效的情况，看打印出来的信息，没有收到
    // TOUCH 信息(都是HOVER信息)。我们需要模拟一个 HOVER 变为 TOUCH.
    int     hover_event_num;
    int     screen_on_suspend;
    
	// 20180205,add hid gpio ctrl:pwr-gpio/reset-gpio
	int 	i2cpower_gpio;
	int     i2cpower_level;
	int     pwr_gpio;
    int     pwron_level;

    int     reset_gpio; // reset level is always 0!
	int 	reset_level;

    int     pendet_gpio;
    int     penon_level;
    int     pendet_irq;

    //20180315,hsl,add irq gpio ctrl(NOT use i2c_client data).
    int     irq_gpio;
    int     irq_level;
	struct regulator 	*vdd_regulator;
	int revert_x_flag;
    int revert_y_flag;
    int exchange_x_y_flag;

    struct delayed_work pd_work; // pen detect work to rebound gpio.
    struct delayed_work resume_work; // 20200919: 休眠唤醒的时候重新上电需要延迟120ms，这个会降低唤醒时间，我们
    							// 安排在另外 的 work 里面完成。
};

extern void ebc_set_tp_power(/*struct input_dev *dev,*/bool pen_on); // 20200716,hsl add.

static int i2c_hid_set_power(struct i2c_client *client, int power_state);
static void i2c_hid_chip_power(struct i2c_client *client,
    bool suspend, bool irq_remain);

static int ihid_fb_notifier_callback(struct notifier_block *self,
				     unsigned long action, void *data)
{
	struct i2c_hid *ihid;
	struct fb_event *event = data;
#if 1
	//int wake_status;

	int blank;
	blank = *((int*)event->data);

	ihid = container_of(self, struct i2c_hid, fb_notif);

	if (action != FB_EARLY_EVENT_BLANK/*FB_EVENT_BLANK*/ || !event)
		return 0;

    // ihid_fb_notifier_callback: blank=4,wake=0,suspend=0
    // ihid_fb_notifier_callback: blank=0,wake=0,suspend=1
    //printk("ihid_fb_notifier_callback: blank=%d,wake=%d,suspend=%d\n", blank, 
    //    ihid->irq_wake_enabled, ihid->is_suspend);
    
	// hid_fb_notifier_callback: action=16
	// printk("ihid_fb_notifier_callback: action=%ld\n", action);
	if (blank == FB_BLANK_UNBLANK) {
	    if(ihid->is_suspend) { //20210427: resume will schedule first.but we can do again.
	        schedule_delayed_work(&ihid->resume_work, 0);
	    }
	} else if (blank == FB_BLANK_POWERDOWN) {
		if(ihid->is_fwupdating) {
			printk("ihid_fb_notifier: fb POWERDOWN,abort cause fwupdating!\n");
		} else {
			// 20191207: fb进入待机，待机界面显示出来了.
			cancel_delayed_work_sync(&ihid->resume_work);
			i2c_hid_set_power(ihid->client, I2C_HID_PWR_SLEEP);
			i2c_hid_chip_power(ihid->client, true, false);
		}
	}
#else
	ihid = container_of(self, struct i2c_hid, fb_notif);
	blank = *((int*)event->data);
	printk("ihid_fb_notifier_callback: action=%ld, blank=%d\n", action, blank);

	if (action == FB_EARLY_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			ihid->is_suspend = 1;
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			ihid->is_suspend = 0;
			break;
		default:
			break;
		}
	}
#endif
	return NOTIFY_OK;
}

static int __i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	union command *cmd = (union command *)ihid->cmdbuf;
	int ret;
	struct i2c_msg msg[2];
	int msg_num = 1;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = cmd->data;
	if (data_len > 0) {
		msg[1].addr = client->addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = data_len;
		msg[1].buf = buf_recv;
		msg_num = 2;
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(I2C_HID_RESET_PENDING, &ihid->flags);

	ret = i2c_transfer(client->adapter, msg, msg_num);

	if (data_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != msg_num)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait) {
		i2c_hid_dbg(ihid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(ihid->wait,
				!test_bit(I2C_HID_RESET_PENDING, &ihid->flags),
				msecs_to_jiffies(5000)))
			ret = -ENODATA;
		i2c_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command,
		unsigned char *buf_recv, int data_len)
{
	return __i2c_hid_command(client, command, 0, 0, NULL, 0,
				buf_recv, data_len);
}

static int i2c_hid_get_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __i2c_hid_command(client, &hid_get_report_cmd, reportID,
		reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&client->dev,
			"failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

/**
 * i2c_hid_set_or_send_report: forward an incoming report to the device
 * @client: the i2c_client of the device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int i2c_hid_set_or_send_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *args = ihid->argsbuf;
	const struct i2c_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(ihid->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(ihid->hdesc.wMaxOutputLength);
	u16 size;
	int args_len;
	int index = 0;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (data_len > ihid->bufsize)
		return -EINVAL;

	size =		2			/* size */ +
			(reportID ? 1 : 0)	/* reportID */ +
			data_len		/* buf */;
	args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
			2			/* dataRegister */ +
			size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

	if (reportID >= 0x0F) {
		args[index++] = reportID;
		reportID = 0x0F;
	}

	/*
	 * use the data register for feature reports or if the device does not
	 * support the output register
	 */
	if (use_data) {
		args[index++] = dataRegister & 0xFF;
		args[index++] = dataRegister >> 8;
		hidcmd = &hid_set_report_cmd;
	} else {
		args[index++] = outputRegister & 0xFF;
		args[index++] = outputRegister >> 8;
		hidcmd = &hid_no_cmd;
	}

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	memcpy(&args[index], buf, data_len);

	ret = __i2c_hid_command(client, hidcmd, reportID,
		reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}

static void i2c_hid_set_chip_power(struct i2c_client *client,
    bool pwr_on)
{
    struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret = 0;

	// SDK103S: i2c_hid_set_chip_power:pwr_on=1,pwr_gpio=-2,vdd=ffffffc07ba02000 / i2c_hid_set_chip_power:pwr_on=1,pwr_gpio=-2,i2cgpio=-2
	printk("%s:pwr_on=%d,pwr_gpio=%d,i2cgpio=%d\n", __func__, pwr_on, ihid->pwr_gpio, ihid->i2cpower_gpio);
	if(gpio_is_valid(ihid->i2cpower_gpio)){
		if( pwr_on ){
            gpio_direction_output(ihid->i2cpower_gpio,ihid->i2cpower_level);
		}
	}

    if( gpio_is_valid(ihid->pwr_gpio) ){
	    if( pwr_on ){
            gpio_direction_output(ihid->pwr_gpio,ihid->pwron_level);
        } else {
            gpio_direction_output(ihid->pwr_gpio,!ihid->pwron_level);
        }
    }
    
    if( pwr_on ){
		if(ihid->vdd_regulator) {
			ret = regulator_enable(ihid->vdd_regulator);
		}
        if( gpio_is_valid(ihid->reset_gpio) ) {
            gpio_direction_output(ihid->reset_gpio,!ihid->reset_level);
		    msleep(50);
		    gpio_direction_output(ihid->reset_gpio,ihid->reset_level);
		    msleep(80);
        }

        gpio_direction_input(ihid->pendet_gpio);
        gpio_direction_input(ihid->irq_gpio);
    } else {
        if( gpio_is_valid(ihid->reset_gpio) ) {
            // 20180205,chip is power off,so reset must be low.
            gpio_direction_output(ihid->reset_gpio, 0);
        }
		if(ihid->vdd_regulator) {
			ret = regulator_disable(ihid->vdd_regulator);
		}

        // 202010427：不拉低这几个GPIO，似乎有点漏电。
		if( gpio_is_valid(ihid->irq_gpio) ) {
            // 20180205,chip is power off,so reset must be low.
            gpio_direction_output(ihid->irq_gpio, 0);
        }
        if( gpio_is_valid(ihid->pendet_gpio) ) {
            // 20180205,chip is power off,so reset must be low.
            gpio_direction_output(ihid->pendet_gpio, 0);
        }
    }

	if(gpio_is_valid(ihid->i2cpower_gpio)){
		if( !pwr_on ){
            gpio_direction_output(ihid->i2cpower_gpio,!ihid->i2cpower_level);
		}
	}
		
	if( ret ) {
		printk("%s: failed, powr_on=%d,ret=%d\n", __func__, pwr_on, ret);
	}
}

static int i2c_hid_set_power(struct i2c_client *client, int power_state)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	ret = __i2c_hid_command(client, &hid_set_power_cmd, power_state,
		0, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&client->dev, "failed to change power setting.\n");

	return ret;
}

static int i2c_hid_hwreset(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
	if (ret)
		return ret;

	/*
	 * The HID over I2C specification states that if a DEVICE needs time
	 * after the PWR_ON request, it should utilise CLOCK stretching.
	 * However, it has been observered that the Windows driver provides a
	 * 1ms sleep between the PWR_ON and RESET requests and that some devices
	 * rely on this.
	 */
	usleep_range(1000, 5000);

	i2c_hid_dbg(ihid, "resetting...\n");

	ret = i2c_hid_command(client, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to reset device.\n");
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
		return ret;
	}

	return 0;
}

// 20210112: 增加打印信息;
static void i2c_hid_dump_input(struct i2c_hid *ihid) 
{
    int x,y,press,tx,ty,hi;

    // 20210601: 0f 00 02 21 ca 22 66 36 ff 0f 00 00 00 c0 f9
    // 02: CP pen, 1a: DY pen.
    //if(!(ihid->inbuf[3] & 1) ) return;
    x = ihid->inbuf[4] | ihid->inbuf[5] << 8;
    y = ihid->inbuf[6] | ihid->inbuf[7] << 8;
    press = ihid->inbuf[8] | ihid->inbuf[9] << 8;
    tx = ihid->inbuf[11] | ihid->inbuf[12] << 8;
    ty = ihid->inbuf[13] | ihid->inbuf[14] << 8;
    hi = ihid->inbuf[15] | ihid->inbuf[16] << 8;
    printk("flag=0x%02x(%s),x=%d,y=%d,press=0x%x,tx=0x%x,ty=0x%x,hi=%d\n", ihid->inbuf[3],
        (ihid->inbuf[3] & 1)?"Touch":"Hover", x, y, press, tx, ty, hi);
}

// 20210601: 休眠时候快速点击，返回的数据个数可能只有 13个或者更小。必须保证 HOVER结束。否则
// 不会上报 sync 事件给应用。
#define MAX_FIX_TOUCH_EVENT_NUM           10
// 20210601: 此处允许上班的事件不是越多越好，太多了会导致 SYN_DROPPED,反而让前面的有效信息
// 被全部丢弃。如果太小了，在休眠时候落笔手写又会丢失部分笔迹。此处保留太多事件无意义，手写
// 上面还是会丢弃掉。
#define MAX_REPORT_EVENT_NUM              30

// 20210601: return true: need to report the event, return false: drop the event.
static bool i2c_hid_fix_input_for_screen_suspend(struct i2c_hid *ihid) 
{
    int press;
    if(!ihid->screen_on_suspend)  return true;

    //printk("flag=0x%02x(%s),hvr num=%d,scn-on-suspend=%d\n", ihid->inbuf[3],
    //    (ihid->inbuf[3] & 1)?"Touch":"Hover", ihid->hover_event_num, ihid->screen_on_suspend);
        
    ihid->hover_event_num++;
    if(ihid->hover_event_num > MAX_REPORT_EVENT_NUM) {
        if(ihid->inbuf[3] & 1) {
            printk("Drop Touch,num=%d\n", ihid->hover_event_num);
            ihid->hover_event_num--;
            return false;
        }
        if(ihid->hover_event_num < MAX_REPORT_EVENT_NUM + 2)  return true;
        return false;
    }
    // 20210601: touch 事件必须到达一定的次数，否则 InputReader: 
    // Detected input event buffer overrun for device hid-over-i2c 2D1F:0123.but continue!!
    // 之后事件被丢弃了。
    if(ihid->inbuf[3] & 1) {
        //ihid->hover_event_num = 100;
        return true;
    }
    
    if(ihid->hover_event_num < 2 || ihid->hover_event_num > MAX_FIX_TOUCH_EVENT_NUM-1 ) return true;
    
    // 20210601: 此处进行 HOVER/TOUCH 的处理. 最多处理 3-8 几个msg.这个处理在手写界面会留下
    // 不想要的笔迹（笔尖还没有接触就有手写内容了），为了解决这个问题，我们上报一个很小的press的值。
    // 20210602: 正常情况下手写，台笔的时候看到 press=6也有1.但是落笔看到的还比较大。
    press = 1; //MAN_FIX_TOUCH_EVENT_NUM + (ihid->hover_event_num-(MAN_FIX_TOUCH_EVENT_NUM/2))*1;
    //if(press < 1) {
    //    press = 1;
    //}
    ihid->inbuf[3] |= 1;
    ihid->inbuf[8] = press & 0XFF;
    ihid->inbuf[9] = (press >> 8) & 0XFF;
    
    ihid->inbuf[11] = 0x74;  // titlx
    ihid->inbuf[12] = 0xf5;
    ihid->inbuf[13] = 0xc8;  // tilt y.
    ihid->inbuf[14] = 0;

    ihid->inbuf[15] = ihid->inbuf[16] = 0; // hover heigh = 0

    printk("fix hover to Touch,flag=0x%02x(%s),press=%d,num=%d\n", ihid->inbuf[3],
        (ihid->inbuf[3] & 1)?"Touch":"Hover", press, ihid->hover_event_num);
    return true;
}

static void i2c_hid_report_hover_if_need(struct i2c_hid *ihid) 
{
    u32 ret_size;
    // 20210601:如果不是休眠或者最后上报的事件已经是HOVER，则不需要处理.
    if(!ihid->screen_on_suspend)  return;

    //printk("report_hover:flag=0x%02x(%s),num=%d\n", ihid->inbuf[3],
    //    (ihid->inbuf[3] & 1)?"Touch":"Hover", ihid->hover_event_num);
    if(!(ihid->inbuf[3] & 1) ) return;

    ihid->inbuf[3] &= ~1;
    ihid->inbuf[8] = 0; //press 
    ihid->inbuf[9] = 0;
    
    ihid->inbuf[11] = 0;  // titlx
    ihid->inbuf[12] = 0;
    ihid->inbuf[13] = 0;  // tilt y.
    ihid->inbuf[14] = 0;

    ihid->inbuf[15] = ihid->inbuf[16] = 0; // hover heigh = 0
    ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;
    printk("report_hover:flag=0x%02x(%s),num=%d,ret_size=%d\n", ihid->inbuf[3],
        (ihid->inbuf[3] & 1)?"Touch":"Hover",ihid->hover_event_num, ret_size);

    
    hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2,
		ret_size - 2, 1);
}

static void i2c_hid_get_input(struct i2c_hid *ihid)
{
	int ret;
	u32 ret_size;
	int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);

	if (size > ihid->bufsize)
		size = ihid->bufsize;

	ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &ihid->flags))
			wake_up(&ihid->wait);
		return;
	}

	if ((ret_size > size) || (ret_size <= 2)) {
		dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n",
			__func__, size, ret_size);
		return;
	}

	i2c_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);
	if (debug) i2c_hid_dump_input(ihid);

	// 20210601: 防止信息太多溢出之后被丢弃反而点击无效了。
 	if(!i2c_hid_fix_input_for_screen_suspend(ihid))
 	    return ;

	if (test_bit(I2C_HID_STARTED, &ihid->flags))
		hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2,
				ret_size - 2, 1);

	return;
}

static irqreturn_t i2c_hid_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

	if (test_bit(I2C_HID_READ_PENDING, &ihid->flags))
		return IRQ_HANDLED;

	i2c_hid_get_input(ihid);

/*
	if (device_may_wakeup(&ihid->client->dev) && ihid->is_suspend == 1){
		//rk_send_wakeup_key();

		printk("i2c_hid_irq rk_send_wakeup_key\n");
	}
*/

	return IRQ_HANDLED;
}

static int i2c_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

static void i2c_hid_init_report(struct hid_report *report, u8 *buffer,
	size_t bufsize)
{
	struct hid_device *hid = report->device;
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	unsigned int size, ret_size;

	size = i2c_hid_get_report_length(report);
	if (i2c_hid_get_report(client,
			report->type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report->id, buffer, size))
		return;

	i2c_hid_dbg(ihid, "report (len=%d): %*ph\n", size, size, buffer);

	ret_size = buffer[0] | (buffer[1] << 8);

	if (ret_size != size) {
		dev_err(&client->dev, "error in %s size:%d / ret_size:%d\n",
			__func__, size, ret_size);
		return;
	}

	/* hid->driver_lock is held as we are in probe function,
	 * we just need to setup the input fields, so using
	 * hid_report_raw_event is safe. */
	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}

/*
 * Initialize all reports
 */
static void i2c_hid_init_reports(struct hid_device *hid)
{
	struct hid_report *report;
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *inbuf = kzalloc(ihid->bufsize, GFP_KERNEL);

	if (!inbuf) {
		dev_err(&client->dev, "can not retrieve initial reports\n");
		return;
	}

	/*
	 * The device must be powered on while we fetch initial reports
	 * from it.
	 */
	pm_runtime_get_sync(&client->dev);

	list_for_each_entry(report,
		&hid->report_enum[HID_FEATURE_REPORT].report_list, list)
		i2c_hid_init_report(report, inbuf, ihid->bufsize);

	pm_runtime_put(&client->dev);

	kfree(inbuf);
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void i2c_hid_find_max_report(struct hid_device *hid, unsigned int type,
		unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = i2c_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void i2c_hid_free_buffers(struct i2c_hid *ihid)
{
	kfree(ihid->inbuf);
	kfree(ihid->rawbuf);
	kfree(ihid->argsbuf);
	kfree(ihid->cmdbuf);
	ihid->inbuf = NULL;
	ihid->rawbuf = NULL;
	ihid->cmdbuf = NULL;
	ihid->argsbuf = NULL;
	ihid->bufsize = 0;
}

static int i2c_hid_alloc_buffers(struct i2c_hid *ihid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	int args_len = sizeof(__u8) + /* ReportID */
		       sizeof(__u8) + /* optional ReportID byte */
		       sizeof(__u16) + /* data register */
		       sizeof(__u16) + /* size of the report */
		       report_size; /* report */

	ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->rawbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!ihid->inbuf || !ihid->rawbuf || !ihid->argsbuf || !ihid->cmdbuf) {
		i2c_hid_free_buffers(ihid);
		return -ENOMEM;
	}

	ihid->bufsize = report_size;

	return 0;
}

static int i2c_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t)ihid->bufsize);

	ret = i2c_hid_get_report(client,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, ihid->rawbuf, ask_count);

	if (ret < 0)
		return ret;

	ret_count = ihid->rawbuf[0] | (ihid->rawbuf[1] << 8);

	if (ret_count <= 2)
		return 0;

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, ihid->rawbuf + 2, count);

	return count;
}

static int i2c_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
		size_t count, unsigned char report_type, bool use_data)
{
	struct i2c_client *client = hid->driver_data;
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	if (report_id) {
		buf++;
		count--;
	}

	ret = i2c_hid_set_or_send_report(client,
				report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
				report_id, buf, count, use_data);

	if (report_id && ret >= 0)
		ret++; /* add report_id to the number of transfered bytes */

	return ret;
}

static int i2c_hid_output_report(struct hid_device *hid, __u8 *buf,
		size_t count)
{
	return i2c_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT,
			false);
}

static int i2c_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
			       __u8 *buf, size_t len, unsigned char rtype,
			       int reqtype)
{
	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return i2c_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return i2c_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

static int i2c_hid_parse(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int rsize;
	char *rdesc;
	int ret;
	int tries = 3;

	i2c_hid_dbg(ihid, "entering %s\n", __func__);

	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("weird size of report descriptor (%u)\n", rsize);
		return -EINVAL;
	}

	do {
		ret = i2c_hid_hwreset(client);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret)
		return ret;

	rdesc = kzalloc(rsize, GFP_KERNEL);

	if (!rdesc) {
		dbg_hid("couldn't allocate rdesc memory\n");
		return -ENOMEM;
	}

	i2c_hid_dbg(ihid, "asking HID report descriptor\n");

	ret = i2c_hid_command(client, &hid_report_descr_cmd, rdesc, rsize);
	if (ret) {
		hid_err(hid, "reading report descriptor failed\n");
		kfree(rdesc);
		return -EIO;
	}

	i2c_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

	ret = hid_parse_report(hid, rdesc, rsize);
	kfree(rdesc);
	if (ret) {
		dbg_hid("parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int i2c_hid_start(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;

	i2c_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

	if (bufsize > ihid->bufsize) {
		i2c_hid_free_buffers(ihid);

		ret = i2c_hid_alloc_buffers(ihid, bufsize);

		if (ret)
			return ret;
	}

	if (!(hid->quirks & HID_QUIRK_NO_INIT_REPORTS))
		i2c_hid_init_reports(hid);

	return 0;
}

static void i2c_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int i2c_hid_open(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&i2c_hid_open_mut);
	if (!hid->open++) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			hid->open--;
			goto done;
		}
		set_bit(I2C_HID_STARTED, &ihid->flags);
	}
done:
	mutex_unlock(&i2c_hid_open_mut);
	return ret < 0 ? ret : 0;
}

static void i2c_hid_close(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	/* protecting hid->open to make sure we don't restart
	 * data acquistion due to a resumption we no longer
	 * care about
	 */
	mutex_lock(&i2c_hid_open_mut);
	if (!--hid->open) {
		clear_bit(I2C_HID_STARTED, &ihid->flags);

		/* Save some power */
		pm_runtime_put(&client->dev);
	}
	mutex_unlock(&i2c_hid_open_mut);
}

static int i2c_hid_power(struct hid_device *hid, int lvl)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_dbg(ihid, "%s lvl:%d\n", __func__, lvl);

	switch (lvl) {
	case PM_HINT_FULLON:
		pm_runtime_get_sync(&client->dev);
		break;
	case PM_HINT_NORMAL:
		pm_runtime_put(&client->dev);
		break;
	}
	return 0;
}

static struct hid_ll_driver i2c_hid_ll_driver = {
	.parse = i2c_hid_parse,
	.start = i2c_hid_start,
	.stop = i2c_hid_stop,
	.open = i2c_hid_open,
	.close = i2c_hid_close,
	.power = i2c_hid_power,
	.output_report = i2c_hid_output_report,
	.raw_request = i2c_hid_raw_request,
};

static int i2c_hid_init_irq(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	dev_dbg(&client->dev, "Requesting IRQ: %d\n", ihid->irq);

	ret = request_threaded_irq(ihid->irq, NULL, i2c_hid_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			client->name, ihid);
	if (ret < 0) {
		dev_warn(&client->dev,
			"Could not register for %s interrupt, irq = %d,"
			" ret = %d\n",
			client->name, ihid->irq, ret);

		return ret;
	}

	return 0;
}

static int i2c_hid_fetch_hid_descriptor(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int dsize;
	int ret;

	/* i2c hid fetch using a fixed descriptor size (30 bytes) */
	i2c_hid_dbg(ihid, "Fetching the HID descriptor\n");
	ret = i2c_hid_command(client, &hid_descr_cmd, ihid->hdesc_buffer,
				sizeof(struct i2c_hid_desc));
	if (ret) {
		dev_err(&client->dev, "hid_descr_cmd failed\n");
		return -ENODEV;
	}

	/* Validate the length of HID descriptor, the 4 first bytes:
	 * bytes 0-1 -> length
	 * bytes 2-3 -> bcdVersion (has to be 1.00) */
	/* check bcdVersion == 1.0 */
	if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
		dev_err(&client->dev,
			"unexpected HID descriptor bcdVersion (0x%04hx)\n",
			le16_to_cpu(hdesc->bcdVersion));
		return -ENODEV;
	}

	/* Descriptor length should be 30 bytes as per the specification */
	dsize = le16_to_cpu(hdesc->wHIDDescLength);
	if (dsize != sizeof(struct i2c_hid_desc)) {
		dev_err(&client->dev, "weird size of HID descriptor (%u)\n",
			dsize);
		return -ENODEV;
	}

	// 20181103-LOG: i2c_hid 1-0009: HID Descriptor:
	// 1e 00 00 01 1f 03 02 00 03 00 11 00 00 00 00 00 04 00 05 00 1f 2d 7a 00 31 05 00 00 00 00
	i2c_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);

	//20210720-更新FW前后的LOG: 
	// hid_hdesc:pid=0x2d1f,vid=0x123,fwVer=0x1241
	// hid_hdesc:pid=0x2d1f,vid=0x149,fwVer=0x1702
	//printk("hid_hdesc:pid=0x%x,vid=0x%x,fwVer=0x%x\n", hdesc->wVendorID,
	//	hdesc->wProductID, hdesc->wVersionID);
	return 0;
}

// 20180315,hsl.shut down the chip power.
// if irq_remain: remain the irq wake up function.
static void i2c_hid_chip_power(struct i2c_client *client,
    bool suspend, bool irq_remain)
{
    struct i2c_hid *ihid = i2c_get_clientdata(client);
    if(ihid->is_suspend == suspend) return ;
    if( suspend ){
        //20180205,hsl,add power ctrl.
        // 20180301,hsl,change the irq type.
        //20180301,disable the pendet riq.
        disable_irq(ihid->irq);
        disable_irq(ihid->pendet_irq);

    	irq_set_irq_type(ihid->irq, IRQ_TYPE_LEVEL_HIGH/*IRQF_TRIGGER_HIGH*/);
    	irq_set_irq_type(ihid->pendet_irq, IRQ_TYPE_LEVEL_HIGH/*IRQF_TRIGGER_HIGH*/);
        i2c_hid_set_chip_power(client,false);
    } else {
        //20180205,hsl,add power ctrl.
        i2c_hid_set_chip_power(client,true);
        irq_set_irq_type(ihid->irq, IRQ_TYPE_LEVEL_LOW/*IRQF_TRIGGER_LOW*/);
        irq_set_irq_type(ihid->irq, IRQ_TYPE_LEVEL_LOW/*IRQF_TRIGGER_LOW*/);

        enable_irq(ihid->pendet_irq);
    	enable_irq(ihid->irq);
    }

    ihid->is_suspend = suspend;
}

static void i2c_hid_pd_worker(struct work_struct *work)
{
	struct i2c_hid *ihid = container_of(work,
			struct i2c_hid, pd_work.work);

    int pendet_gpio_value = gpio_get_value(ihid->pendet_gpio);
	bool penOn = (pendet_gpio_value == ihid->penon_level)?true:false;
	//printk("%s:hid penOn=%d,gpio=%d,on-level=%d\n", __func__, penOn,
	//    pendet_gpio_value, ihid->penon_level);

    // 20180213,reset the type for next irq.
	/*if (pendet_gpio_value)
		irq_set_irq_type(ihid->pendet_irq, IRQF_TRIGGER_LOW);
	else
		irq_set_irq_type(ihid->pendet_irq, IRQF_TRIGGER_HIGH);
    */

	ebc_set_tp_power(penOn/*poweroff*/);
}

static irqreturn_t i2c_hid_pendet_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

    int pendet_gpio_value = gpio_get_value(ihid->pendet_gpio);
    bool penOn = (pendet_gpio_value == ihid->penon_level)?true:false;
    //printk("%s:pd gpio value=%d,penOn=%d,scon_suspend=%d,ev_num=%d\n", __func__, pendet_gpio_value,
    //    penOn, ihid->screen_on_suspend, ihid->hover_event_num);

    if(!penOn) i2c_hid_report_hover_if_need(ihid);
    
    if (pendet_gpio_value)
		irq_set_irq_type(ihid->pendet_irq, IRQ_TYPE_LEVEL_LOW/*IRQF_TRIGGER_LOW*/);
	else
		irq_set_irq_type(ihid->pendet_irq, IRQ_TYPE_LEVEL_HIGH/*IRQF_TRIGGER_HIGH*/);

    if( !delayed_work_pending(&ihid->pd_work) ){
        //cancel_delayed_work(&ihid->pd_work);
        schedule_delayed_work(&ihid->pd_work, msecs_to_jiffies(50));
    }

	return IRQ_HANDLED;
}

static int i2c_hid_init_pen_det_irq(struct i2c_hid *ihid)
{
	int ret;
    unsigned long flag = IRQF_ONESHOT;

    if( gpio_get_value(ihid->pendet_gpio) ){
        flag |= IRQF_TRIGGER_LOW;
    } else {
        flag |= IRQF_TRIGGER_HIGH;
    }

    INIT_DELAYED_WORK(&ihid->pd_work, i2c_hid_pd_worker);

    ihid->pendet_irq = gpio_to_irq(ihid->pendet_gpio);
	//printk("Requesting pen-det IRQ: %d\n", ihid->pendet_irq);

	ret = request_threaded_irq(ihid->pendet_irq, NULL, i2c_hid_pendet_irq,
			flag, "hid-pendet", ihid);
	if (ret < 0) {
		printk("%s:request irq failed, irq=%d,ret=%d\n", __func__,
		    ihid->pendet_irq, ret );

		return ret;
	}

	return 0;
}

static void i2c_hid_resume_worker(struct work_struct *work)
{
	int ret;
	struct i2c_hid *ihid = container_of(work,
			struct i2c_hid, resume_work.work);
	struct i2c_client *client = ihid->client;
	struct hid_device	*hid = ihid->hid;

	printk("%s: suspend=%d\n", __func__, ihid->is_suspend);
	if(ihid->is_suspend) {
    	i2c_hid_chip_power(client, false, false);
    	i2c_hid_set_power(client, I2C_HID_PWR_ON);
    	//i2c_hid_set_chip_power(client,true);
    	if (!device_may_wakeup(&client->dev)) {
    		ret = i2c_hid_hwreset(client);
    		if (ret)
    			return;
    	}
    	if (hid->driver && hid->driver->reset_resume) {
    		ret = hid->driver->reset_resume(hid);
    	}
	}
}



#ifdef CONFIG_ACPI

/* Default GPIO mapping */
static const struct acpi_gpio_params i2c_hid_irq_gpio = { 0, 0, true };
static const struct acpi_gpio_mapping i2c_hid_acpi_gpios[] = {
	{ "gpios", &i2c_hid_irq_gpio, 1 },
	{ },
};

static int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	static u8 i2c_hid_guid[] = {
		0xF7, 0xF6, 0xDF, 0x3C, 0x67, 0x42, 0x55, 0x45,
		0xAD, 0x05, 0xB3, 0x0A, 0x3D, 0x89, 0x38, 0xDE,
	};
	union acpi_object *obj;
	struct acpi_device *adev;
	acpi_handle handle;
	int ret;

	handle = ACPI_HANDLE(&client->dev);
	if (!handle || acpi_bus_get_device(handle, &adev))
		return -ENODEV;

	obj = acpi_evaluate_dsm_typed(handle, i2c_hid_guid, 1, 1, NULL,
				      ACPI_TYPE_INTEGER);
	if (!obj) {
		dev_err(&client->dev, "device _DSM execution failed\n");
		return -ENODEV;
	}

	pdata->hid_descriptor_address = obj->integer.value;
	ACPI_FREE(obj);

	/* GPIOs are optional */
	ret = acpi_dev_add_driver_gpios(adev, i2c_hid_acpi_gpios);
	return ret < 0 && ret != -ENXIO ? ret : 0;
}

static const struct acpi_device_id i2c_hid_acpi_match[] = {
	{"ACPI0C50", 0 },
	{"PNP0C50", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, i2c_hid_acpi_match);
#else
static inline int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef CONFIG_OF
static int i2c_hid_of_probe(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata, struct i2c_hid * ihid)
{
	struct device *dev = &client->dev;
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;
	//struct i2c_hid *ihid = i2c_get_clientdata(client);
	dev_err(&client->dev, "i2c_hid_of_probe,i2c addr=0x%x,irq=%d\n",
	    client->addr, client->irq);
	//pr_info

    if( client->irq <= 0 ) {
        ihid->irq_gpio = of_get_named_gpio_flags(np, "irq_gpio", 0, &flags);
    	if (ihid->irq_gpio < 0 ){
    		printk("%s  irq_gpio error!!\n", __func__);
    		return -EINVAL;
        } else {
    		ihid->irq_level = (flags & OF_GPIO_ACTIVE_LOW)
    		    ?  0 : 1;
    		ret = gpio_request(ihid->irq_gpio, "hid-i2c-irq");
    		if( ret ){
    		    printk("%s:request hid-i2c-irq gpio %d Failed!!!\n",__func__ , ihid->irq_gpio);
    		} else {
    		    // the pendet is input.
    		    gpio_direction_input(ihid->irq_gpio);
    		}
        }
        client->irq = gpio_to_irq(ihid->irq_gpio);
    }

	ret = of_property_read_u32(dev->of_node, "hid-descr-addr", &val);
	if (ret) {
		dev_err(&client->dev, "HID register address not provided\n");
		return -ENODEV;
	}
	if (val >> 16) {
		dev_err(&client->dev, "Bad HID register address: 0x%08x\n",
			val);
		return -EINVAL;
	}
	pdata->hid_descriptor_address = val;

	ihid->i2cpower_gpio = of_get_named_gpio_flags(np, "i2cpower_gpio", 0, &flags);
	if (ihid->i2cpower_gpio < 0 ){	// may got -2. can not use smop->pwr_gpio == -EPROBE_DEFER
		printk("%s	i2cpower_gpio error--that's ok\n", __func__);
	} else {
		ihid->i2cpower_level = (flags & OF_GPIO_ACTIVE_LOW)
			?  0 : 1;
		ret = gpio_request(ihid->i2cpower_gpio, "hid-i2c-all-pwr");
		if( ret ){
			printk("%s:request i2cpower_gpio %d Failed!!!\n",__func__ , ihid->i2cpower_gpio);
		} else {
			//pwron now,because we need to i2c_hid_fetch_hid_descriptor
			gpio_direction_output(ihid->i2cpower_gpio,ihid->i2cpower_level);
		}
		printk("%s:i2cpower_gpio(%d) on-level=%d!!\n",__func__ ,
			ihid->i2cpower_gpio,ihid->i2cpower_level);
	}

    ihid->pwr_gpio = of_get_named_gpio_flags(np, "pwr_gpio", 0, &flags);
	if (ihid->pwr_gpio < 0 ){   // may got -2. can not use smop->pwr_gpio == -EPROBE_DEFER
		printk("%s  pwr_gpio error--that's ok\n", __func__);
    } else {
		ihid->pwron_level = (flags & OF_GPIO_ACTIVE_LOW)
		    ?  0 : 1;
		ret = gpio_request(ihid->pwr_gpio, "hid-i2c-pwr");
		if( ret ){
		    printk("%s:request pwr_gpio %d Failed!!!\n",__func__ , ihid->pwr_gpio);
		} else {
		    //pwron now,because we need to i2c_hid_fetch_hid_descriptor
		    gpio_direction_output(ihid->pwr_gpio,ihid->pwron_level);
		}
		printk("%s:pwr_gpio(%d) on-level=%d!!\n",__func__ ,
		    ihid->pwr_gpio,ihid->pwron_level);
    }

	ihid->vdd_regulator = devm_regulator_get(&client->dev, "vcc"); //devm_regulator_get_optional
	if (IS_ERR(ihid->vdd_regulator) /*== -ENODEV*/) {
		ret = PTR_ERR(ihid->vdd_regulator);
		dev_err(&client->dev, "power not specified for wacom,ignore power ctrl,err=%d\n", ret);
		ihid->vdd_regulator = NULL;
	} else {
		ret = regulator_enable(ihid->vdd_regulator);
		printk("i2c_hid_of_probe,vdd_regulator enable \n");
	}

    ihid->reset_gpio = of_get_named_gpio_flags(np, "reset_gpio", 0, &flags);
	if (ihid->reset_gpio < 0 ){
		printk("%s  reset_gpio error--that's ok\n", __func__);
    } else {
    	ihid->reset_level = (flags & OF_GPIO_ACTIVE_LOW);
		ret = gpio_request(ihid->reset_gpio, "hid-i2c-rst");
		if( ret ){
		    printk("%s:request reset_gpio %d Failed!!!\n",__func__ , ihid->reset_gpio);
		}
		gpio_direction_output(ihid->reset_gpio, ihid->reset_level);
				printk("%s:i2creset_gpio(%d) reset-level=%d!!\n",__func__ ,
			ihid->reset_gpio,ihid->reset_level);
    }

    ihid->pendet_gpio= of_get_named_gpio_flags(np, "pendet_gpio", 0, &flags);
	if (ihid->pendet_gpio < 0 ){
		printk("%s pendet_gpio error--that's ok\n", __func__);
    } else {
		ihid->penon_level = (flags & OF_GPIO_ACTIVE_LOW)
		    ?  0 : 1;
		ret = gpio_request(ihid->pendet_gpio, "hid-i2c-pendet");
		if( ret ){
		    printk("%s:request pendet_gpio %d Failed!!!\n",__func__ , ihid->pendet_gpio);
		} else {
		    // the pendet is input.
		    gpio_direction_input(ihid->pendet_gpio);
		}
    }
	ihid->revert_x_flag = 0;
	if (!of_property_read_u32(np, "revert_x_flag", &val)){
		ihid->revert_x_flag = !!val;
	}
	ihid->revert_y_flag = 0;
	if (!of_property_read_u32(np, "revert_y_flag", &val)){
		ihid->revert_y_flag = !!val;
	}
	ihid->exchange_x_y_flag = 0;
	if (!of_property_read_u32(np, "exchange_x_y_flag", &val)){
		ihid->exchange_x_y_flag = !!val;
	}

	return 0;
}


// cat /sys/bus/i2c/devices/1-0009/ver
static ssize_t hid_info_show(struct device *dev,
	 struct device_attribute *attr,
	 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	return sprintf(buf,"0x%x", hdesc->wVersionID);
}

// 20210726: 用来设置固件升级标志。在升级的时候，不能进入休眠(不能掉电)。
// echo 1 > /sys/bus/i2c/devices/1-0009/ver
static ssize_t hid_info_store(struct device *dev, 
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	if(buf[0] == '1') {
		ihid->is_fwupdating = true;
		wake_lock(&ihid->fw_lock);
	} else {
		ihid->is_fwupdating = false;
		wake_unlock(&ihid->fw_lock);
	}
	dev_err(&client->dev, "set fwupdating=%d\n", ihid->is_fwupdating);
	return count;
}


static struct device_attribute hid_dev_attr[] = {
	__ATTR(ver, 0664, hid_info_show, hid_info_store),
};

static void hid_info_init_sysfs(struct i2c_client *client)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(hid_dev_attr); i++) {
		ret = sysfs_create_file(&client->dev.kobj,
					&hid_dev_attr[i].attr);
		if (ret)
			dev_err(&client->dev, "create hid-info node(%s) error\n",
				hid_dev_attr[i].attr.name);
	}
}

static const struct of_device_id i2c_hid_of_match[] = {
	{ .compatible = "hid-over-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_hid_of_match);
#else
static inline int i2c_hid_of_probe(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int i2c_hid_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	int ret;
	struct i2c_hid *ihid;
	struct hid_device *hid;
	__u16 hidRegister;
	struct i2c_hid_platform_data *platform_data = client->dev.platform_data;
	bool    agained = false;

	dbg_hid("HID probe called for i2c 0x%02x\n", client->addr);

	ihid = kzalloc(sizeof(struct i2c_hid), GFP_KERNEL);
	if (!ihid)
		return -ENOMEM;

	if (client->dev.of_node) {
		ret = i2c_hid_of_probe(client, &ihid->pdata, ihid);
		if (ret)
			goto err;
	} else if (!platform_data) {
		ret = i2c_hid_acpi_pdata(client, &ihid->pdata);
		if (ret) {
			dev_err(&client->dev,
				"HID register address not provided\n");
			goto err;
		}
	} else {
		ihid->pdata = *platform_data;
	}

	if (client->irq > 0) {
		ihid->irq = client->irq;
	} else if (ACPI_COMPANION(&client->dev)) {
	    int irq_gpio;
	    int ret;
		ihid->desc = gpiod_get(&client->dev, NULL, GPIOD_IN);
		if (IS_ERR(ihid->desc)) {
			dev_err(&client->dev, "Failed to get GPIO interrupt\n");
			return PTR_ERR(ihid->desc);
		}

		ihid->irq = gpiod_to_irq(ihid->desc);
		if (ihid->irq < 0) {
			gpiod_put(ihid->desc);
			dev_err(&client->dev, "Failed to convert GPIO to IRQ\n");
			return ihid->irq;
		}

		irq_gpio = desc_to_gpio(ihid->desc);
		ret = gpio_request(irq_gpio,"hid-i2c-irq");
		gpiod_direction_input(ihid->desc);
		printk("%s:irq_gpio=%d,value=%d,ret=%d\n",__func__, irq_gpio,
		    gpiod_get_value(ihid->desc), ret);
	}

	i2c_set_clientdata(client, ihid);

	ihid->client = client;

	hidRegister = ihid->pdata.hid_descriptor_address;
	ihid->wHIDDescRegister = cpu_to_le16(hidRegister);

	init_waitqueue_head(&ihid->wait);
	
	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
	 * real computation later. */
	ret = i2c_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
	if (ret < 0)
		goto err;

    // 20181103: power on again when probe Failed!
again:

	pm_runtime_get_noresume(&client->dev);
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);


    //20180205,set power after i2c_set_clientdata.
    i2c_hid_set_chip_power(client,true);
    i2c_hid_init_pen_det_irq(ihid);//20180213,init after poweron.

	ret = i2c_hid_fetch_hid_descriptor(ihid);
	if (ret < 0)
		goto err_pm;

	ret = i2c_hid_init_irq(client);
	if (ret < 0)
		goto err_pm;

	if (client->dev.of_node) {
		ret = of_property_read_bool(client->dev.of_node, "hid-support-wakeup");
		if (ret) {
			device_init_wakeup(&client->dev, true);
			ihid->is_suspend = 0;
			ihid->fb_notif.notifier_call = ihid_fb_notifier_callback;
			fb_register_client(&ihid->fb_notif);
		}
	}

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret = PTR_ERR(hid);
		goto err_irq;
	}

	ihid->hid = hid;

	hid->driver_data = client;
	hid->ll_driver = &i2c_hid_ll_driver;
	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->version = le16_to_cpu(ihid->hdesc.bcdVersion);
	hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
	hid->product = le16_to_cpu(ihid->hdesc.wProductID);

	snprintf(hid->name, sizeof(hid->name), "%s %04hX:%04hX",
		 client->name, hid->vendor, hid->product);
	strlcpy(hid->phys, dev_name(&client->dev), sizeof(hid->phys));

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(client, "can't add hid device: %d\n", ret);
		goto err_mem_free;
	}

	pm_runtime_put(&client->dev);
	
	INIT_DELAYED_WORK(&ihid->resume_work, i2c_hid_resume_worker);
	wake_lock_init(&ihid->fw_lock, WAKE_LOCK_SUSPEND, "hid-fw");
	hid_info_init_sysfs(client);
	return 0;

err_mem_free:
	hid_destroy_device(hid);

err_irq:
	free_irq(ihid->irq, ihid);

err_pm:
	pm_runtime_put_noidle(&client->dev);
	pm_runtime_disable(&client->dev);

    // 20181103: when error,free the irq for again!
    free_irq(ihid->pendet_irq, ihid);
err:
	if (ihid->desc)
		gpiod_put(ihid->desc);

    if( !agained ) {
        printk("%s: probe Failed,ret=%d,agained=%d\n", __func__, ret, agained);
        i2c_hid_set_chip_power(client,false);
        msleep(100);
        agained = true;
        goto again;
    }
	i2c_hid_free_buffers(ihid);
	kfree(ihid);
	return ret;
}

static int i2c_hid_remove(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid;

	pm_runtime_get_sync(&client->dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	hid = ihid->hid;
	hid_destroy_device(hid);

	free_irq(ihid->irq, ihid);

	if (ihid->bufsize)
		i2c_hid_free_buffers(ihid);

	if (ihid->desc)
		gpiod_put(ihid->desc);

	kfree(ihid);

	acpi_dev_remove_driver_gpios(ACPI_COMPANION(&client->dev));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int i2c_hid_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int ret = 0;
	int wake_status;

	printk("i2c_hid_suspend ihid->irq_wake_enabled:%d fb_power_off=%d\n",ihid->irq_wake_enabled,fb_power_off());
	//printk("%s:reset gpio level=%d,fb_power_off=%d\n", __func__, gpio_get_value(wac_i2c->reset_gpio),fb_power_off());

	if (hid->driver && hid->driver->suspend)
		ret = hid->driver->suspend(hid, PMSG_SUSPEND);
#if 1 //tanlq

	//disable_irq(ihid->irq);
	if(fb_power_off()) {
	    // 20210427: do nothing, already done at fb-notify.
		//disable_irq(ihid->irq);
		//i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
		//i2c_hid_chip_power(client, true, false);
	} else {
		if (device_may_wakeup(&client->dev)) {
			wake_status = enable_irq_wake(ihid->irq);
			if (!wake_status) {
				ihid->irq_wake_enabled = true;
			} else
				hid_warn(hid, "Failed to enable irq wake: %d\n",
					wake_status);
		}

		ihid->screen_on_suspend = 1;
		ihid->hover_event_num = 0;
	}
#else
	disable_irq(ihid->irq);
	if (device_may_wakeup(&client->dev)) {
		wake_status = enable_irq_wake(ihid->irq);
		if (!wake_status)
			ihid->irq_wake_enabled = true;
		else
			hid_warn(hid, "Failed to enable irq wake: %d\n",
				wake_status);
	}

	/* Save some power */
	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);

#endif
	return ret;
}

static int i2c_hid_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int wake_status;
	printk("%s: irq_wake=%d fb_power_off=%d,hover_num=%d\n", __func__, ihid->irq_wake_enabled, 
	    fb_power_off(), ihid->hover_event_num);
	if(fb_power_off()) {
		#if 0
		i2c_hid_chip_power(client, false, false);
		i2c_hid_set_power(client, I2C_HID_PWR_ON);
    	//i2c_hid_set_chip_power(client,true);
			if (!device_may_wakeup(&client->dev)) {
				ret = i2c_hid_hwreset(client);
				if (ret)
					return ret;
			}
		if (hid->driver && hid->driver->reset_resume) {
			ret = hid->driver->reset_resume(hid);
			return ret;
		}
		#else
		// 20200919: 我们 HID(WACOM)和TP共用同一个 电源控制，所以HID延后初始化，那么 TP也要做相应的延后。
		// 20210427: 我们统一在 fb-notify 里面给 TP上电，因为存在系统休眠后被 817的GPIO唤醒，但是此时并没有亮屏。
		// 如果我们在这里 给 TP 上电，必须在 suspend 里面给TP 下点。不如改为统一在 fb-notify 里面上下电。
		// schedule_delayed_work(&ihid->resume_work, 0); // msecs_to_jiffies(20)
		ret = 0;
		#endif
	}
	//enable_irq(ihid->irq);
	else{
		if (device_may_wakeup(&client->dev) && ihid->irq_wake_enabled) {
			wake_status = disable_irq_wake(ihid->irq);
			if (!wake_status)
				ihid->irq_wake_enabled = false;
			else
				hid_warn(hid, "Failed to disable irq wake: %d\n",
					wake_status);
		}

		ihid->screen_on_suspend = 0;
	}
	//if (hid->driver && hid->driver->reset_resume) {
	//	ret = hid->driver->reset_resume(hid);
	//	return ret;
	//}

	return 0;
}
#endif

#ifdef CONFIG_PM
static int i2c_hid_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	disable_irq(ihid->irq);
	return 0;
}

static int i2c_hid_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	enable_irq(ihid->irq);
	i2c_hid_set_power(client, I2C_HID_PWR_ON);
	return 0;
}
#endif

static const struct dev_pm_ops i2c_hid_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(i2c_hid_suspend, i2c_hid_resume)
	SET_RUNTIME_PM_OPS(i2c_hid_runtime_suspend, i2c_hid_runtime_resume,
			   NULL)
};

static const struct i2c_device_id i2c_hid_id_table[] = {
	{ "hid", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, i2c_hid_id_table);


static struct i2c_driver i2c_hid_driver = {
	.driver = {
		.name	= "i2c_hid",
		.owner	= THIS_MODULE,
		.pm	= &i2c_hid_pm,
		.acpi_match_table = ACPI_PTR(i2c_hid_acpi_match),
		.of_match_table = of_match_ptr(i2c_hid_of_match),
	},

	.probe		= i2c_hid_probe,
	.remove		= i2c_hid_remove,

	.id_table	= i2c_hid_id_table,
};

module_i2c_driver(i2c_hid_driver);

MODULE_DESCRIPTION("HID over I2C core driver");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_LICENSE("GPL");
