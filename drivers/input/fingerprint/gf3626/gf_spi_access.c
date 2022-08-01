/* Goodix's GF516/GF318/GF516M/GF318M/GF518M/GF3118M/GF5118M
 *  fingerprint sensor linux driver for REE and factory mode
 *
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "gf_common.h"

extern u8 g_debug_level;

#ifdef SUPPORT_REE_SPI
void gf_spi_setup_conf(struct gf_device *gf_dev, u32 speed)
{
	u32 max_speed_hz;

	switch (speed) {
	case 1:
	case 4:
		max_speed_hz = 2400000;//48
		break;
	case 6:
	case 8:
		max_speed_hz = 2400000; //96
		break;
	default:
		max_speed_hz = 2400000;//48
	}

	gf_dev->spi->mode = SPI_MODE_0;
	gf_dev->spi->max_speed_hz = max_speed_hz;
	gf_dev->spi->bits_per_word = 8;

	if (spi_setup(gf_dev->spi))
		gf_debug(ERR_LOG, "%s, failed to setup spi conf\n", __func__);
}

int gf_spi_read_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	tmp_buf = gf_dev->spi_buffer;

	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* memset((tmp_buf + 4), 0x00, data_len + 1); */
	/* 4 bytes align */
	*(tmp_buf + 4) = 0xF1;
	xfer[1].tx_buf = tmp_buf + 4;
	xfer[1].rx_buf = tmp_buf + 4;
	xfer[1].len = data_len + 1;
	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	memcpy(rx_buf, (tmp_buf + 5), data_len);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gf_spi_write_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}
	tmp_buf = gf_dev->spi_buffer;

	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	memcpy(tmp_buf + 3, tx_buf, data_len);
	xfer[0].len = data_len + 3;
	xfer[0].tx_buf = tmp_buf;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gf_spi_read_byte(struct gf_device *gf_dev, u16 addr, u8 *value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* 4 bytes align */
	*(gf_dev->spi_buffer + 4) = 0xF1;
	xfer[1].tx_buf = gf_dev->spi_buffer + 4;
	xfer[1].rx_buf = gf_dev->spi_buffer + 4;
	xfer[1].len = 2;
	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	*value = *(gf_dev->spi_buffer + 5);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}


int gf_spi_write_byte(struct gf_device *gf_dev, u16 addr, u8 value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);
	*(gf_dev->spi_buffer + 3) = value;

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 3 + 1;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

static int gf_spi_transfer_raw(struct gf_device *gf_dev, u8 *tx_buf, u8 *rx_buf, u32 len)
{
	struct spi_message msg;
	struct spi_transfer xfer;

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(struct spi_transfer));

	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.len = len;
	//printk("gf_spi_transfer_raw len:%d tx=%s rx=%s\n",len,tx_buf,rx_buf);
	spi_message_add_tail(&xfer, &msg);
	spi_sync(gf_dev->spi, &msg);

	return 0;
}

int gf_ioctl_transfer_raw_cmd(struct gf_device *gf_dev, unsigned long arg, unsigned int bufsiz)
{
	struct gf_ioc_transfer_raw ioc_xraw;
	//struct gf_ioc_transfer_raw_k ioc_xraw_k;
	int retval = 0;
	//int *buf = NULL;
//int i;
	do {
		u8 *tx_buf;
		u8 *rx_buf;
		uint32_t len;

		if (copy_from_user(&ioc_xraw, (void __user *)arg, sizeof(struct gf_ioc_transfer_raw))) {
			gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer_raw from user to kernel\n", __func__);
			retval = -EFAULT;
			break;
		}

		if ((ioc_xraw.len > bufsiz) || (ioc_xraw.len == 0)) {
			gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
			retval = -EINVAL;
			break;
		}

		if (ioc_xraw.read_buf == NULL || ioc_xraw.write_buf == NULL) {
			gf_debug(ERR_LOG, "%s: read buf and write buf can not equal to NULL simultaneously.\n", __func__);
			retval = -EINVAL;
			break;
		}

		/* change speed and set transfer mode */
		gf_spi_setup_conf(gf_dev, ioc_xraw.high_time);

		len = ioc_xraw.len;

		tx_buf = kzalloc(len, GFP_KERNEL);
		if (tx_buf == NULL) {
			gf_debug(ERR_LOG, "%s: failed to allocate raw tx buffer\n", __func__);
			retval = -EMSGSIZE;
			break;
		}

		rx_buf = kzalloc(len, GFP_KERNEL);
		if (rx_buf == NULL) {
			kfree(tx_buf);
			gf_debug(ERR_LOG, "%s: failed to allocate raw rx buffer\n", __func__);
			retval = -EMSGSIZE;
			break;
		}
		//(const u8 __user *)(uintptr_t)

		if (copy_from_user(tx_buf, (void __user *)ioc_xraw.write_buf, ioc_xraw.len)) {
			kfree(tx_buf);
			kfree(rx_buf);
			gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from user to kernel\n");
			retval = -EFAULT;
			break;
		}
//		printk("gf_spi_transfer_raw1 len:%d tx=%s \n",len,tx_buf);
//for(i=0;i<len;i++)
//{
//if(i<10)
//	printk("tx%d=0x%x ",i,tx_buf[i]);
//}
//printk("\n");
		//if(len <100){

		gf_spi_transfer_raw(gf_dev, tx_buf, rx_buf, len);
		//	}
		//printk("gf_spi_transfer_raw2 len:%d rx=%s \n",len,rx_buf);
		//for(i=0;i<len;i++)
		//{
		//if(i<10)
		//	printk("rx%d=0x%x ",i,rx_buf[i]);
		//}
		//printk("\n");

//if(len <100){
		if (copy_to_user((void __user *)ioc_xraw.read_buf, rx_buf, ioc_xraw.len)) {
			kfree(tx_buf);
			kfree(rx_buf);
			gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer_raw from kernel to user\n");
			retval = -EFAULT;
		}

		kfree(tx_buf);
		kfree(rx_buf);
	} while (0);

	return retval;
}

int gf_ioctl_spi_init_cfg_cmd(struct gf_device *gf_dev, unsigned long arg)
{
	return 0;
}
#endif
