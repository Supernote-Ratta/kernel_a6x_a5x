/*
 * RK29 ebook spi falsh driver epd_spi_falsh.c
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: dlx@rock-chips.com
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
/*include*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "epd_spi_flash.h"
#include "../epd_lut.h"

/*define.*/
#if EPD_SPI_DEBUG
#define epd_spi_printk(fmt, args...)  printk(KERN_INFO "ebc " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define epd_spi_printk(fmt, args...)
#endif
#if defined(CONFIG_PVI_WAVEFORM)
/* waveform */
#define EINK_WAVEFORM_FILESIZE          262144  // 256K..
#define WF_BUFF_SIZE        EINK_WAVEFORM_FILESIZE
#define PNL_LAST             (PNL_SIZE - 1)
#define PNL_BASE_WAVEFORM    0x20
#define PNL_SIZE_WAVEFORM    23

#define PNL_SIZE                256
#define PNL_BASE_PART_NUMBER    0x00
#define PNL_SIZE_PART_NUMBER    16
#define PNL_SIZE_ID_STR         32
static char panel_id[PNL_SIZE_ID_STR]   = { 0 };

#define PNL_BASE_FPL            0x40
#define PNL_BASE_WAVEFORM       0x20
#define WFM_ADDR                0x00886
#define PNL_CHAR_UNKNOWN        '!'
#define PANEL_ID_UNKNOWN        "????_???_??_???"
typedef struct tagPANEL_TYPE{
    u8  name[4];
    u32 offset_addr; 
}PANEL_TYPE;
static PANEL_TYPE panel_offset_addr[] = {
    {"XC3", (0x30000 - WFM_ADDR)},
    {"XD4", (0x70000 - WFM_ADDR)},
};


#endif


/*struct.*/

static struct epd_spi_flash_info spi_flash_info;

static void Delay100cyc(u32 count)
{
	u16 i;

	while (count--)
		for (i = 0; i < 23; i++);
}

static u32 SpiFlashWaitBusy(struct spi_device *flashdev)
{
	u8 cmd[1];
	u8 status=0xff;
	u32 i;
	
	for (i=0; i<500000; i++)
	{
		Delay100cyc(100);
		cmd[0] = 0x05;
		spi_write_then_read(flashdev, cmd, 1, &status, 1);
		if ((status & 0x01) == 0)		
			return 0;
	}
	return 1;
}

static u32 SPIFlashRead(void *flashdev, u32 addr, u8 *pdata, u32 len) 
{    
	u8 cmd[4];
	u32 ReadLen;
	u32 ret = 0;
	u32 data = (u32)pdata;
   
	epd_spi_printk("enter SPIFlashRead flashdev = %x addr = %d pData = %x, len = %d.\n",flashdev, addr, pdata, len);

	while (len > 0)
	{
		ReadLen = (len > SPI_PAGE_SIZE)? SPI_PAGE_SIZE : len;     

		cmd[0] = READ_DATA;
		cmd[1] = addr>>16 & 0xff;
		cmd[2] = addr>>8 & 0xff;
		cmd[3] = addr & 0xff;
		ret = spi_write_then_read((struct spi_device *)flashdev, cmd, 4, (u8*)data, ReadLen);
		if( ret )
		{
			epd_spi_printk("spi_write_then_read err.\n");
			return 1;
		}
		data += ReadLen;
		len -= ReadLen;       
		addr += ReadLen;
	}

	return 0;
}

static u32 SPIFlashWrite(void *flashdev, u32 addr, u8 *pdata, u32 len) 
{   
	u8 data[20];
	u32 writeLen; 
	u32 ret=0;

	epd_spi_printk("enter SPIFlashWrite.\n");

	while (len > 0)      
	{      
		writeLen = SPI_PAGE_SIZE - (addr % SPI_PAGE_SIZE);	
		writeLen = (len > writeLen)? writeLen : len;
		data[0] = WRITE_ENABLE;    //write enable	    

		ret = spi_write_then_read((struct spi_device *)flashdev, (u8*)data, 1, NULL, 0);

		if (0!=SpiFlashWaitBusy((struct spi_device *)flashdev))
		{
			epd_spi_printk("SpiFlashWaitBusy err.\n");
			ret=1;
		}

		data[0] = BYTE_WRITE;    //byte program
		data[1] = addr>>16 & 0xff;
		data[2] = addr>>8 & 0xff;
		data[3] = addr & 0xff;

		memcpy(&data[4], pdata, writeLen);

		ret = spi_write_then_read((struct spi_device *)flashdev, (u8*)data, writeLen+4, NULL, 0 );      
		if(ret)
		{
			epd_spi_printk("spi_write_then_read err.\n");
			return 1;
		}
		pdata = (u8*)((u32)pdata + writeLen);
		addr = addr+writeLen;      
		len -= writeLen;

		Delay100cyc(30);  //大于100ns  4.333us*30=130us
		if (0!=SpiFlashWaitBusy((struct spi_device *)flashdev))
		{
			epd_spi_printk("SpiFlashWaitBusy err.\n");
			ret=1;
		}
	}

	data[0] = WRITE_DISABLE;    //write disable
	spi_write_then_read((struct spi_device *)flashdev, data, 1, NULL, 0);

	return 0;
}

static u32 Sector_Erase(void *flashdev, u32 addr) 
{    
	u8 data[4];

	epd_spi_printk("enter Sector_Erase.\n");

	data[0] = 0x06;    //write enable     
	spi_write_then_read((struct spi_device *)flashdev, data, 1, NULL, 0);

	data[0] = 0xd8;   // 块擦除
	data[1] = addr>>16 & 0xff;
	data[2] = addr>>8 & 0xff;
	data[3] = addr & 0xff;

	spi_write_then_read((struct spi_device *)flashdev, data, 4, NULL, 0 );

	if (0!=SpiFlashWaitBusy((struct spi_device *)flashdev))
	{
		epd_spi_printk("SpiFlashWaitBusy err.\n");
		return 1;
	}

	data[0] = 0x04;    //write disable
	spi_write_then_read((struct spi_device *)flashdev, data, 1, NULL, 0);

	return 0;

}

static int spi_write_data(void *flashdev, int addr,char *buf,int len)
{
	char *pagedata=NULL;
	int earse_count=0;
	u32 pageaddr=0;
	u32 pagelen=0;
	int temp_addr;
	int ret;

	pagedata = (char*)kmalloc(FLASH_PAGE_SIZE, GFP_KERNEL);
	if(pagedata<0)
	{
		epd_spi_printk("spi_flash_write kmalloc failed. \n");
		return -1;
	} 
	
	while(len>=FLASH_PAGE_SIZE){  	
		pagelen = addr % FLASH_PAGE_SIZE;
		if(pagelen)
			pageaddr=addr - pagelen;
		else
			pageaddr=addr;
		temp_addr=pageaddr;
		ret = SPIFlashRead(flashdev, pageaddr, pagedata, FLASH_PAGE_SIZE);
		if(ret != 0)
		{
			epd_spi_printk("SPIFlashRead err.\n");  
			return -1;
		}
		for(earse_count=0;earse_count<FLASH_PAGE_SIZE/FLASH_PAGE_EARSE;earse_count++){
		Sector_Erase(flashdev, pageaddr);  
		pageaddr=pageaddr+FLASH_PAGE_EARSE;
		}    
		SPIFlashWrite(flashdev, addr, (u8*)buf, FLASH_PAGE_SIZE); 
		if(pagelen)	
		SPIFlashWrite(flashdev, temp_addr, pagedata, pagelen); 
		buf=buf+FLASH_PAGE_SIZE;
		len -= FLASH_PAGE_SIZE;
		addr = temp_addr+FLASH_PAGE_SIZE;        
	}	
	
	if(len){ 	
		pagelen = addr % FLASH_PAGE_SIZE;
		if(pagelen)
			pageaddr=addr -pagelen;
		else
			pageaddr=addr;
		
		temp_addr=pageaddr;
		ret = SPIFlashRead(flashdev, pageaddr, pagedata, FLASH_PAGE_SIZE);
		if(ret != 0)
		{
			epd_spi_printk("SPIFlashRead err.\n");    
			return -1;
		}
		
		for(earse_count=0;earse_count<FLASH_PAGE_SIZE/FLASH_PAGE_EARSE;earse_count++){
			Sector_Erase(flashdev, pageaddr);  
			pageaddr=pageaddr+FLASH_PAGE_EARSE;
		} 
		SPIFlashWrite(flashdev, addr, (u8*)buf, len); 
		if(pagelen)	
			SPIFlashWrite(flashdev, temp_addr, pagedata, pagelen); 
		SPIFlashWrite(flashdev, addr+len, pagedata+pagelen+len, FLASH_PAGE_SIZE-pagelen-len); 
	}
	
	kfree(pagedata);
	return 0;
}
#if defined(CONFIG_PVI_WAVEFORM)
static bool panel_data_valid(char *panel_data)
{
	bool result = false;

	if ( panel_data )
	{
		if ( strchr(panel_data, PNL_CHAR_UNKNOWN) )
		{
			printk(KERN_ERR "Unrecognized values in panel data\n");
			pr_debug("panel data = %s\n", panel_data);
		}
		else
			result = true;
	}

	return ( result );
}

enum panel_data_characters
{
	zero = 0x0, one, two, three, four, five, six, seven, eight, nine,
	underline = 0x0a, dot = 0x0b, negative = 0x0c,
	_a = 0xcb, _b, _c, _d, _e, _f, _g, _h, _i, _j, _k, _l, _m, _n,
	_o, _p, _q, _r, _s, _t, _u, _v, _w, _x, _y, _z,

	_A = 0xe5, _B, _C, _D, _E, _F, _G, _H, _I, _J, _K, _L, _M, _N,
	_O, _P, _Q, _R, _S, _T, _U, _V, _W, _X, _Y, _Z
};
typedef enum panel_data_characters panel_data_characters;
static void panel_data_translate(u8 *buffer, int to_read) {
	int i = 0;

	for (i = 0; i < to_read; i++) {
		if (buffer[i] >= _a && buffer[i] <= _z) {
			buffer[i] = 'a' + (buffer[i] - _a);
		} else if (buffer[i] >= _A && buffer[i] <= _Z) {
			buffer[i] = 'A' + (buffer[i] - _A);
		} else if (/* buffer[i] >= zero && */ buffer[i] <= nine) {
			buffer[i] = '0' + (buffer[i] - zero);
		} else if (buffer[i] == underline) {
			buffer[i] = '_';
		} else if (buffer[i] == dot) {
			buffer[i] = '.';
		} else if (buffer[i] == negative) {
			buffer[i] = '-';
		} else {
			buffer[i] = PNL_CHAR_UNKNOWN;
		}
	}
}

char *panel_get_id(void)
{

	// If the panel ID hasn't already been read in, then read it in now.
	//
	if ( !(('_' == panel_id[4]) && ('_' == panel_id[8]) && ('_' == panel_id[11])) )
	{
        u8 pnl_index = 0;
        u8 pnl_type = 0;
        u8 panel_type_name[4] = {0};
        u8 panel_buffer[PNL_SIZE] = {0};
        char *part_number;
        int cur;

		// Waveform file names are of the form PPPP_XLLL_DD_TTVVSS_B, and
		// panel IDs are of the form PPPP_LLL_DD_MMM.
		//
        SPIFlashRead(spi_flash_info.dev, 0x56, panel_type_name, 4);
        if(panel_type_name[1] == 'X'){
            panel_type_name[0] = panel_type_name[1];
            panel_type_name[1] = panel_type_name[2];
            panel_type_name[2] = panel_type_name[3];
        }
        
        for(pnl_index = 0; pnl_index < sizeof(panel_offset_addr) / sizeof(panel_offset_addr[0]); pnl_index++){
            if((panel_offset_addr[pnl_index].name[0] == panel_type_name[0]) &&
               (panel_offset_addr[pnl_index].name[1] == panel_type_name[1]) &&
               (panel_offset_addr[pnl_index].name[2] == panel_type_name[2])){
                break;
            }
        }
        
        SPIFlashRead(spi_flash_info.dev, panel_offset_addr[pnl_index].offset_addr, panel_buffer, PNL_SIZE);
        panel_data_translate(panel_buffer, PNL_SIZE);
        epd_spi_printk("\npanel_buffer  =  %s\n", panel_buffer);
        
		// The platform is (usually) the PPPP substring.  And, in those cases, we copy
		// the platform data from the EEPROM's waveform name.  However, we must special-case
		// the V220E waveforms since EINK isn't using the same convention as they did in
		// the V110A case (i.e., they named V110A waveforms 110A but they are just
		// calling the V220E waveforms V220 with a run-type of E; run-type is the X
		// field in the PPPP_XLLL_DD_TTVVSS_B part of waveform file names).
		//
		switch ( panel_buffer[PNL_BASE_WAVEFORM+5] )
		{
			case 'E':
				panel_id[0] = '2';
				panel_id[1] = '2';
				panel_id[2] = '0';
				panel_id[3] = 'E';
				break;

			default:
				panel_id[0] = panel_buffer[PNL_BASE_WAVEFORM+0];
				panel_id[1] = panel_buffer[PNL_BASE_WAVEFORM+1];
				panel_id[2] = panel_buffer[PNL_BASE_WAVEFORM+2];
				panel_id[3] = panel_buffer[PNL_BASE_WAVEFORM+3];
				break;
		}

		panel_id[ 4] = '_';

		// The lot number (aka FPL) is the the LLL substring:  Just
		// copy the number itself, skipping the batch (X) designation.
		//
		panel_id[ 5] = panel_buffer[PNL_BASE_FPL+1];
		panel_id[ 6] = panel_buffer[PNL_BASE_FPL+2];
		panel_id[ 7] = panel_buffer[PNL_BASE_FPL+3];

		panel_id[ 8] = '_';

		// The display size is the the DD substring.
		//
		panel_id[ 9] = panel_buffer[PNL_BASE_WAVEFORM+10];
		panel_id[10] = panel_buffer[PNL_BASE_WAVEFORM+11];
		panel_id[11] = '_';

		/* Copy in the full part number */
		part_number = &panel_buffer[PNL_BASE_PART_NUMBER];
		cur = 0;
		while (cur < PNL_SIZE_PART_NUMBER && part_number[cur] != PNL_CHAR_UNKNOWN) {
			panel_id[12+cur] = part_number[cur];
			cur++;
		}

		panel_id[12+cur] = 0;

		if ( !panel_data_valid(panel_id) )
			strcpy(panel_id, PANEL_ID_UNKNOWN);
	}
	epd_spi_printk("%s: panel id=%s\n", __FUNCTION__, panel_id);

	return ( panel_id );
}
#endif
static int spi_flash_open(struct inode *inode, struct file *file)
{
	epd_spi_printk("enter spi flash open.\n");
	file->f_pos = 0;
	return 0;
}
static ssize_t spi_flash_write(struct file *file, const char __user *data,
			      size_t len, loff_t * ppos)
{
	char *write_data=NULL;
	u32 addr = 0;

	if(ppos)
		addr = *ppos;   
	write_data = (char*)kmalloc(len, GFP_KERNEL);

	if(copy_from_user(write_data, data, len))
	{
		kfree(write_data);
		return -EFAULT;
	}
	spi_write_data(spi_flash_info.dev,addr,(char*)write_data,len);

	kfree(write_data);
	return len;

}


static ssize_t spi_flash_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	u32 addr = 0;
	u8* kbuf = NULL;
	int i;
	u32 ret=0;
	
	epd_spi_printk("enter spi flash read.\n");
	
#if defined(CONFIG_PVI_WAVEFORM)

	kbuf = (u8*)kmalloc(EINK_WAVEFORM_FILESIZE, GFP_KERNEL);
	if(SPIFlashRead(spi_flash_info.dev, addr+WFM_ADDR,kbuf , EINK_WAVEFORM_FILESIZE)){
		printk("%s:fail\n",__func__);
		kfree(kbuf);
		return -1;
	}
	buf = panel_get_id();
	for(i=0;i<PNL_SIZE_ID_STR;i++)
	kbuf[EINK_WAVEFORM_FILESIZE+i]=buf[i];
	return kbuf;
#else
	if(SPIFlashRead(spi_flash_info.dev, addr ,kbuf , count))
	{
		kfree(kbuf);
		return -1;
	}
	
	else
	{
		if(copy_to_user(buf, (char *)kbuf, count))
		{
			kfree(kbuf);
			return -EFAULT;
		}
		return count;
	}
#endif

}


/* seek文件定位函数 */
static loff_t spi_flash_seek(struct file *filp, loff_t offset, int orig)
{
	loff_t ret = 0;
	
	switch (orig)
	{
		case 0:   /*相对文件开始位置偏移*/
			if (offset < 0)
			{
				ret =  - EINVAL;
				break;
			}
			if ((unsigned int)offset > FLASH_GLOBAl_SIZE)
			{
				ret =  - EINVAL;
				break;
			}
			filp->f_pos = (unsigned int)offset;
			ret = filp->f_pos;
			break;
			
		case 1:   /*相对文件当前位置偏移*/
			if ((filp->f_pos + offset) > FLASH_GLOBAl_SIZE)
			{
				ret =  - EINVAL;
				break;
			}
			if ((filp->f_pos + offset) < 0)
			{
				ret =  - EINVAL;
				break;
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;
			
		default:
			ret =  - EINVAL;
			break;
	}
	
	return ret;
}

static struct file_operations spi_flash_fops = {
	.owner	= THIS_MODULE,
	.open   = spi_flash_open,
	.write  = spi_flash_write,
	.read   = spi_flash_read,
	.llseek = spi_flash_seek
};

static int  __devinit spi_flash_probe(struct spi_device *spi)
{        
	short flash_major = 0;
	dev_t devno; 
	int err =0;

	epd_spi_printk("enter spi flash probe.\n");

	spi_flash_info.dev = spi;
	
	//  register_chrdev(0, "spi_flash", &spi_flash_fops);
	/* create your own class under /sysfs */
	 devno = MKDEV(flash_major, 0);
	 if(flash_major)
	 {
		register_chrdev_region(devno, 1, "spi_flash");
	 }
	 else
	 {
		alloc_chrdev_region(&devno,0, 1, "spi_flash");
		flash_major = MAJOR(devno);
	 }

	spi_flash_info.cdev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
	if(NULL == spi_flash_info.cdev)
	{
		epd_spi_printk("no mem.\n");
		return ENOMEM;
	}
	cdev_init(spi_flash_info.cdev, &spi_flash_fops);
	spi_flash_info.cdev->owner = THIS_MODULE;
	spi_flash_info.cdev->ops = &spi_flash_fops;
	err = cdev_add(spi_flash_info.cdev, devno, 1);
	if(err)
		epd_spi_printk("adding spi flash error.\n");

	spi_flash_info.my_class = class_create(THIS_MODULE, "spi_flash");
	if(IS_ERR(spi_flash_info.my_class)) 
	{
		epd_spi_printk("Err: failed in creating spi flash class.\n");
		return -1; 
	} 
	device_create(spi_flash_info.my_class, NULL, devno,NULL, "spi_flash"); 

	epd_spi_flash_register(&spi_flash_info);

	epd_spi_printk("spi flash probe ok.\n");
		
	return 0;
}

static  int __devexit spi_flash_remove(struct spi_device *pdev)
{  
	epd_spi_printk("enter spi flash remove.\n");

	if(spi_flash_info.cdev)
	{
		cdev_del(spi_flash_info.cdev);
		kfree(spi_flash_info.cdev);
	}
	if(spi_flash_info.my_class)
	{
		device_destroy(spi_flash_info.my_class, 0);         //delete device node under /dev
		class_destroy(spi_flash_info.my_class);               //delete class created by us
	}

	return 0;
}

#ifdef CONFIG_PM
int spi_flash_suspend(struct spi_device *spi, pm_message_t state)
{
    epd_spi_printk("suspend.\n");
     
    return 0;
}

int spi_flash_resume(struct spi_device *spi)
{
    epd_spi_printk("resume.\n");
  
    return 0;
}
#endif 

static struct spi_driver spi_flash_driver = 
{
	.driver = {
		.name = "epd_spi_flash",
		.bus	  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = spi_flash_probe,
	.remove = __devexit_p(spi_flash_remove),
#ifdef CONFIG_PM
	.suspend = spi_flash_suspend,
	.resume = spi_flash_resume
#endif
};

static int __init spi_flash_init(void)
{
	return spi_register_driver(&spi_flash_driver);
}

static void __exit spi_flash_exit(void)
{
	return spi_unregister_driver(&spi_flash_driver);
}

 subsys_initcall(spi_flash_init);
module_exit(spi_flash_exit);
MODULE_AUTHOR("dlx@rock-chips.com");
MODULE_DESCRIPTION("rockchip rk29 extern spi flash");
MODULE_LICENSE("GPL");
