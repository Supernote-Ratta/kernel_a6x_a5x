/*
 * RK29 ebook control driver rk29_ebc.c
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: Dai Lunxue <dlx@rock-chips.com>
 *         Huang Lin <hl@rock-chips.com>
 *         Yang Kuankuan <ykk@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
//#include <linux/debugfs.h>
//#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/device.h>
//#include <linux/delay.h>
//#include <linux/init.h>
//#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
//#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
//#include <linux/nvmem-consumer.h>
//#include <linux/firmware.h>

#include "htfy_dump.h"

// 20190909:目前这个功能主要用于分析 EINK-软解的时候数据是否正确，我们定义 3个 region，是为了
// 方便调控。一个是 lcdc-初始化数据， 一个是 lcdc0 显示第一帧数据， 一个是 lcdc1 显示第二帧数据.
// 后续的帧就会被丢弃。 --我们通过 debug 命令来保持数据，所以这里可以修改大一点。
#define HTFY_DUMP_REGION_CNT 		100
struct htfy_dump_region {
	char 		comment[HTFY_DUMP_NAME_LEN];
	int 		offset;
	int 		size;
};

struct htfy_dump_info {
	struct device 		*dev;
	char				*vir_addr;
	unsigned long 		phy_addr;
	int					total_size;
	int					free_offset;

	int 				region_num;
	struct htfy_dump_region		reginos[HTFY_DUMP_REGION_CNT];
};

static struct htfy_dump_info	*global_dump_info;


///20190911,hsl:增加这个buffer用来调试数据.
#define DEBUGFRAME_MAX_CNT 			96
struct debug_data_frame {
	char		frame_name[16];
	u8			frame_lut[64];
	u8			gray_new[16];
	u8			gray_old[16];
	u8			lcd_data[16];
};
// 25度以及以上，数据更新没有超过 96 帧的。
static struct debug_data_frame	ebc_debug_frames[DEBUGFRAME_MAX_CNT];
static int 						ebc_record_frame_num;
bool 							ebc_frame_8bit = false;


/*-----------------------------------------------------------------*/
// 外部接口函数.
int htfy_dump_put_data(const char* comment, void* data, int size)
{
	int 						free_space;
	struct htfy_dump_region		*region;
	if( !global_dump_info ) return -ENODATA;
	if( global_dump_info->region_num == HTFY_DUMP_REGION_CNT) {
		printk("%s: Reach Max region num(%d)\n", __func__, global_dump_info->region_num);
		return -ENOMEM;
	}

	free_space = global_dump_info->total_size - global_dump_info->free_offset;
	if( free_space < size ){
		printk("%s: buffer Full!free_space=%d,size = %d\n", __func__, free_space, size);
		return -ENOMEM;
	}

	// printk("%s:free_space=%d,size=%d,region num=%d\n", __func__, free_space, size, global_dump_info->region_num);
	region = &global_dump_info->reginos[global_dump_info->region_num++];
	region->offset = global_dump_info->free_offset;
	region->size = size;
	strncpy(region->comment, comment, sizeof(region->comment)-1);

	// copy the data!!
	memcpy(global_dump_info->vir_addr + global_dump_info->free_offset, data, size);
	global_dump_info->free_offset += ALIGN(size, 4);

	return 0;
}

int htfy_dump_clear_all_data( void )
{
	if( !global_dump_info ) return -ENODATA;
	if( global_dump_info->region_num == 0) {
		return 0;
	}

	global_dump_info->free_offset = 0;
	global_dump_info->region_num = 0;
	memset(global_dump_info->reginos, 0, sizeof(global_dump_info->reginos));
	return 0;
}

// 20190911:这是一个独立的接口，用于打印 buffer 数据。
void htfy_dump_buff_hex16(const char* prefix, const void * ptr, const int data_len) {
	const u8*  data = (const u8*)ptr;
	int i;
	//printf("\n[buf]:07 06 05 04 03 02 01 00\n");
	for(i=0; i < data_len; i+= 16)
    {
        printk("%s[%02d]:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
        	prefix, i,
            data[i+0],
            data[i+1],
            data[i+2],
            data[i+3],
            data[i+4],
            data[i+5],
            data[i+6],
            data[i+7],
            data[i+8],
            data[i+9],
            data[i+10],
            data[i+11],
            data[i+12],
            data[i+13],
            data[i+14],
            data[i+15]
            );
     }
}

bool htfy_dump_record_frame_data(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data)
{
	if(frame >= DEBUGFRAME_MAX_CNT ) {
		printk("%s:cur frame=%d,max-frame=%d\n", __func__, frame, DEBUGFRAME_MAX_CNT);
		return false;
	}
	sprintf(ebc_debug_frames[frame].frame_name, "Frame4Bit[%02d]", frame);
	memcpy(ebc_debug_frames[frame].gray_new, gray_new,
		sizeof(ebc_debug_frames[frame].gray_new));
	memcpy(ebc_debug_frames[frame].gray_old, gray_old,
		sizeof(ebc_debug_frames[frame].gray_old));
	memcpy(ebc_debug_frames[frame].frame_lut, frame_lut,
		sizeof(ebc_debug_frames[frame].frame_lut));
	memcpy(ebc_debug_frames[frame].lcd_data, lcd_data,
		sizeof(ebc_debug_frames[frame].lcd_data));

	ebc_frame_8bit = false;
	ebc_record_frame_num = frame + 1;
	return true;
}

bool htfy_dump_record_frame_data_8bit(int frame, void* gray_new, void* gray_old, void* frame_lut, void* lcd_data)
{
	int 	i;
	u8		*temp;
	if(frame >= DEBUGFRAME_MAX_CNT ) {
		printk("%s:cur frame=%d,max-frame=%d\n", __func__, frame, DEBUGFRAME_MAX_CNT);
		return false;
	}
	sprintf(ebc_debug_frames[frame].frame_name, "Frame8Bit[%02d]", frame);

	// 20190911:保持两者(4BIT/8BIT)数据一样.
	temp = (u8*)gray_new;
	for(i = 0; i < sizeof(ebc_debug_frames[frame].gray_new); i++) {
		ebc_debug_frames[frame].gray_new[i] = temp[i*2] | (temp[i*2+1] << 4);
	}

	temp = (u8*)gray_old;
	for(i = 0; i < sizeof(ebc_debug_frames[frame].gray_old); i++) {
		ebc_debug_frames[frame].gray_old[i] = temp[i*2] | (temp[i*2+1] << 4);
	}

	//memcpy(ebc_debug_frames[frame].gray_new, gray_new,
	//	sizeof(ebc_debug_frames[frame].gray_new));
	//memcpy(ebc_debug_frames[frame].gray_old, gray_old,
	//	sizeof(ebc_debug_frames[frame].gray_old));
	memcpy(ebc_debug_frames[frame].frame_lut, frame_lut,
		sizeof(ebc_debug_frames[frame].frame_lut));
	memcpy(ebc_debug_frames[frame].lcd_data, lcd_data,
		sizeof(ebc_debug_frames[frame].lcd_data));

	ebc_record_frame_num = frame + 1;
	ebc_frame_8bit = true;
	return true;
}


int htfy_dump_frame_data(void)
{
	char frame_name[HTFY_DUMP_NAME_LEN];
	int frame_size = (ebc_record_frame_num)*sizeof(struct debug_data_frame);
	sprintf(frame_name, "FrameData_%02d", ebc_record_frame_num);  // ebc_frame_8bit?"HT":"RK",
	htfy_dump_put_data(frame_name, ebc_debug_frames, frame_size);
	return frame_size;
}

/*-----------------------------------------------------------------*/

// 增加读函数，用于获取 更新后的 status。堵塞，只有一个或者多个显示区域完成的时候，才会返回.
ssize_t htfy_dump_ctrl_read (struct file *file, char __user *buf, size_t size, loff_t *pos) {
	struct htfy_dump_info 		*info = file->private_data;
	struct htfy_dump_region		*region;
	int 						ret = 0;
	char						*data;
	int 						i;
	int 						offset = (int)*pos;

	data = info->vir_addr + info->free_offset;
	ret = sprintf(data, "Region Num:%d,Total Size=0x%x,Free Offset=0x%x\n", info->region_num,
		info->total_size, info->free_offset);
	for(i = 0; i < info->region_num; i++){
		region = &info->reginos[i];
		ret += sprintf(data+ret, "  Regions[%d]:%s,size=%d,offset=%d(0x%x)\n", i,
			region->comment, region->size, region->offset, region->offset);
	}

	// 20190909-LOG: htfy_dump_ctrl_read:offset=0x0, ret=101, size=4096
	printk("%s:offset=0x%x, ret=%d, size=%ld\n", __func__, offset, ret, size);
	if( offset >= ret) {
		ret = 0;
	} else {
		ret -= offset;
		if (copy_to_user(buf, &data[offset], ret)) {
		    printk("%s:copy_to_user Failed,count=%d\n",__func__, ret);
			ret = -EFAULT;
		} else {
			*pos = offset+ret; // 20190909:我们需要自己维护这个 pos。
		}
	}
	return ret;
}

long htfy_dump_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct htfy_dump_info 		*info = file->private_data;
	if( cmd == 0x4854CCEA ){
		printk("%s:Clear All dump data, free offset=%d,regions=%d\n", __func__, info->free_offset, info->region_num);
		htfy_dump_clear_all_data();
		return 0;
	} else {
		printk("%s:Unsupport Cmd=0x%08x\n", __func__, cmd);
	}
	return -ENOTSUPP;
}

static int htfy_dump_ctrl_open(struct inode *inode, struct file *file)
{
	file->f_pos = 0;
	file->private_data = global_dump_info;
	return 0;
}

static const struct file_operations htfy_dump_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = htfy_dump_ctrl_open,
	.read = htfy_dump_ctrl_read,
	.unlocked_ioctl = htfy_dump_ctrl_ioctl,
	#ifdef CONFIG_COMPAT
	.compat_ioctl = htfy_dump_ctrl_ioctl,
	#endif
};


ssize_t htfy_dump_data_read (struct file *file, char __user *buf, size_t size, loff_t *pos) {
	struct htfy_dump_info 		*info = file->private_data;
	//struct htfy_dump_region		*region;
	int 		len = 0;
	int 		offset = (int)*pos;

	len = info->free_offset - offset;
	printk("%s:offset=0x%x, free_offset=0x%x,len=%d, size=%ld\n", __func__, offset, info->free_offset, len, size);
	if( len > 0 ) {
		if( len > (int)size) {
			len = size;
		}
		if (copy_to_user(buf, info->vir_addr+offset, len)) {
		    printk("%s:copy_to_user Failed,count=%d\n",__func__, len);
			len = -EFAULT;
		} else {
			*pos = offset+len; // 20190909:我们需要自己维护这个 pos。
		}
	} else if( len < 0 ) {
		len = 0;
	}
	return len;
}

static int htfy_dump_data_open(struct inode *inode, struct file *file)
{
	file->f_pos = 0;
	file->private_data = global_dump_info;
	return 0;
}

static const struct file_operations htfy_dump_data_ops = {
	.owner = THIS_MODULE,
	.open = htfy_dump_data_open,
	.read = htfy_dump_data_read,
	.llseek = generic_file_llseek,
};

static struct miscdevice htfy_dump_ctrl_misc = {
	.minor = 130,
	.name = "htdump_ctrl",
	.fops = &htfy_dump_ctrl_ops,
};

static struct miscdevice htfy_dump_data_misc = {
	.minor = 131,
	.name = "htdump_data",
	.fops = &htfy_dump_data_ops,
};

static int htfy_dump_sysfs_init(struct htfy_dump_info 		*dump_info)
{
	int ret = misc_register(&htfy_dump_ctrl_misc);
	if( ret ) {
		printk("htfy_dump: regist ctrl_misc Failed,ret=%d", ret);
		return ret;
	}
	ret = misc_register(&htfy_dump_data_misc);
	if( ret ) {
		printk("htfy_dump: regist data_misc Failed,ret=%d", ret);
	}

	htfy_dump_ctrl_misc.parent = dump_info->dev;
	htfy_dump_data_misc.parent = dump_info->dev;
	//return device_create_file(eink->dev, &dev_attr_waveform_version);
	return ret;
}
static int htfy_dump_buffer_init(struct htfy_dump_info 	*dump_info)
{
	struct device 			*dev = dump_info->dev;
	struct device_node 		*memory;
	struct resource r;
	int ret;

	/* alloc display buffer for buf_manage  */
	memory = of_parse_phandle(dev->of_node, "dumpmemory-region", 0);
	if (!memory)
		return -ENODEV;

	ret = of_address_to_resource(memory, 0, &r);
	of_node_put(memory);
	if (ret)
		return ret;

	dump_info->phy_addr = r.start;
	dump_info->total_size = resource_size(&r);

	dump_info->vir_addr = devm_memremap(dev, dump_info->phy_addr, dump_info->total_size, MEMREMAP_WB);

	// htfy_dump:total_size=5242880,vir_addr=ffffffc000a7d8c0,phy_addr=0xa7d8c0
	printk("htfy_dump:total_size=%d,vir_addr=%p,phy_addr=0x%lx\n", dump_info->total_size,
		dump_info->vir_addr, dump_info->phy_addr);
	if (IS_ERR_OR_NULL(dump_info->vir_addr) )
		return -ENOMEM;

	dump_info->free_offset = 0;
	dump_info->region_num = 0;
	return 0;
}


static int htfy_dump_probe(struct platform_device *pdev)
{
	struct device 				*dev = &pdev->dev;
	struct htfy_dump_info 		*dump_info;

	int ret;
	dump_info = devm_kzalloc(dev, sizeof(*dump_info), GFP_KERNEL);
	if (!dump_info)
		return -ENOMEM;

	dump_info->dev = dev;
	global_dump_info = dump_info;

	/* 0: 先做 reserved mem 的初始化，因为我们需要的mem都是从这个空间申请的。  */
	ret = htfy_dump_buffer_init(dump_info);
	if (ret) {
		dev_err(dev, "htfy_dump_buffer_init failed!!\n");
		goto free_fb;
	}

	/* 11. sysfs debug node register register */
	htfy_dump_sysfs_init(dump_info);
	return 0;

free_fb:
	kfree(dump_info);
	global_dump_info = NULL;
	return ret;
}

static int htfy_dump_remove(struct platform_device *pdev)
{
	struct htfy_dump_info 	*info = dev_get_drvdata(&pdev->dev);
	//struct eink_panel 	*panel = &eink->panel;

	misc_deregister(&htfy_dump_ctrl_misc);
	misc_deregister(&htfy_dump_data_misc);
	kfree(info);
	global_dump_info = NULL;
	return 0;
}


static const struct of_device_id htfy_dump_match[] = {
	{ .compatible = "htfy_dump", },
	{ },
};
MODULE_DEVICE_TABLE(of, htfy_dump_match);

static struct platform_driver htfy_dump_driver = {
	.probe  = htfy_dump_probe,
	.remove = htfy_dump_remove,
	.driver = {
		.name = "htfy_dump",
		//.pm = &rk_eink_pm,
		.of_match_table = htfy_dump_match,
	},
};

static int __init htfy_dump_init(void)
{
	return platform_driver_register(&htfy_dump_driver);
}

static void __exit htfy_dump_exit(void)
{
	return platform_driver_unregister(&htfy_dump_driver);
}

device_initcall(htfy_dump_init);  // early then ebc-init.
//device_initcall_sync(htfy_dump_init);

module_exit(htfy_dump_exit);

