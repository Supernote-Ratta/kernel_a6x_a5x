#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include "einkwf.h"
#include "eink_waveform.h"
#include "panel_flash.h"


#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define IN_RANGE(n, m, M)	(((n) >= (m)) && ((n) <= (M)))
#define EINK_CHECKSUM(c1, c2)	(((c2) << 16) | (c1))

#define WFM_DEBUG
#ifdef WFM_DEBUG
#define wfm_debug(fmt, args...) do{printk("[WFM] "fmt, ##args);} while (0)
#else
#define wfm_debug(fmt, args...) do{}while(0)
#endif

/*
 * SPI Flash API
 */

#define SFM_WRSR              0x01
#define SFM_PP                0x02
#define SFM_READ              0x03
#define SFM_WRDI              0x04
#define SFM_RDSR              0x05
#define SFM_WREN              0x06
#define SFM_FAST_READ         0x0B
#define SFM_SE                0x20
#define SFM_BE                0xD8
#define SFM_RES               0xAB
#define SFM_ID                0x9F
#define SFM_WIP_MASK          BIT(0)
#define SFM_BP0_MASK          BIT(2)
#define SFM_BP1_MASK          BIT(3)

#define PANEL_FLASH_SIZE           (1024 * 128)
#define PANEL_FLASH_PAGE_SIZE      (256)
#define PANEL_FLASH_SECTOR_SIZE    (1024 * 4)
#define PANEL_FLASH_BLOCK_SIZE     (1024 * 64)

#define PANEL_ID_UNKNOWN      "????_???_??_???"
#define PNL_SIZE_ID_STR       32

#define GPIO_SPI_MAX_CHARS    28
#define SFM_READ_CMD_LEN      4
#define SFM_WRITE_CMD_LEN     4

static char panel_bcd[PNL_SIZE_BCD_STR] = { 0 };
static char panel_id[PNL_SIZE_ID_STR]   = { 0 };
static char panel_vcom_str[PNL_SIZE_VCOM_STR] = { 0 };
static int  panel_vcom = 0;

static u8   panel_waveform_buffer[EINK_WAVEFORM_FILESIZE] = { 0 };
static bool panel_waveform_header_only = false;
static int  panel_waveform_size = 0;

//static struct spi_device *panel_flash_spi = NULL;
static panel_flash_select eink_rom_select = panel_flash_waveform;
static bool panel_rom_select_locked = false;
static bool   spi_registered = false;
static int panel_sfm_size = SFM_SIZE_256K;
static int panel_tst_addr = TST_ADDR_256K;

/* waveform */
#define WF_BUFF_SIZE        EINK_WAVEFORM_FILESIZE
static char wf_buff[WF_BUFF_SIZE] = { 0 };
static char *wf_buffer      = wf_buff;
static int   wf_buffer_size = WF_BUFF_SIZE;

static unsigned char  *buffer_byte  = NULL;
static unsigned short *buffer_short = NULL;
static unsigned long  *buffer_long  = NULL;
static struct epd_spi_flash_info spi_flash_info;
extern int epd_spi_flash_register(struct epd_spi_flash_info *epd_spi_flash);

//static char *wf_to_use = NULL;

// CRC-32 algorithm from:
//  <http://glacier.lbl.gov/cgi-bin/viewcvs.cgi/dor-test/crc32.c?rev=HEAD>

/* Table of CRCs of all 8-bit messages. */
static unsigned crc_table[256];

/* Flag: has the table been computed? Initially false. */
static int crc_table_computed = 0;

/* Make the table for a fast CRC. */
static void make_crc_table(void) {
	unsigned c;
	int n, k;
	for (n = 0; n < 256; n++) {
		c = (unsigned) n;
		for (k = 0; k < 8; k++) {
			if (c & 1) {
				c = 0xedb88320L ^ (c >> 1);
			}
			else {
				c = c >> 1;
			}
		}
		crc_table[n] = c;
	}
	crc_table_computed = 1;
}

/*
   Update a running crc with the bytes buf[0..len-1] and
   the updated crc. The crc should be initialized to zero. Pre-
   post-conditioning (one's complement) is performed within
   function so it shouldn't be done by the caller. Usage example

   unsigned crc = 0L

   while (read_buffer(buffer, length) != EOF) {
   crc = update_crc(crc, buffer, length);
   }
   if (crc != original_crc) error();
 */
static unsigned update_crc(unsigned crc,
		unsigned char *buf, int len) {
	unsigned c = crc ^ 0xffffffffL;
	int n;

	if (!crc_table_computed) make_crc_table();

	for (n = 0; n < len; n++) {
		c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
	}

	return c ^ 0xffffffffL;
}

/* Return the CRC of the bytes buf[0..len-1]. */
unsigned crc32(unsigned char *buf, int len) {
	return update_crc(0L, buf, len);
}

/* Return the sum of the bytes buf[0..len-1]. */
unsigned char sum8(unsigned char *buf, int len) {
	unsigned char c = 0;
	int n;

	for (n = 0; n < len; n++)
		c += buf[n];

	return c;
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

bool panel_flash_present(void)
{
	return ( (NULL != spi_flash_info.dev) && spi_registered );
}

static panel_flash_select get_flash_select(void)
{
	return ( eink_rom_select );
}

static void set_flash_select(panel_flash_select flash_select)
{
	/* BEN TODO - mutex here */
	if ( !panel_rom_select_locked )
	{
		switch ( flash_select )
		{
			case panel_flash_waveform:
			case panel_flash_commands:
			case panel_flash_test:
				eink_rom_select = flash_select;
				break;

				// Prevent the compiler from complaining.
				//
			default:
				break;
		}
	}
}

static unsigned long get_flash_base(void)
{
	unsigned long result;

	switch ( get_flash_select() )
	{
		case panel_flash_waveform:
			result = WFM_ADDR;
			break;

		case panel_flash_commands:
			result = CMD_ADDR;
			break;

		case panel_flash_test:
		default:
			result = panel_tst_addr;
			break;
	}

	return ( result );
}

static int panel_read_from_flash(unsigned long addr, unsigned char *data, unsigned long size)
{
	struct spi_device *spi = spi_flash_info.dev;
	unsigned long start = get_flash_base() + addr;
	struct spi_transfer t;
	struct spi_message m;
	u32 len, xfer_len, xmit_len, extra;
	u8 *tx_buf, *rx_buf, *xmit_buf;
	int ret, i;
	u32 *rcv_buf;

	if (spi == NULL) {
		pr_debug("uninitialized!\n");
		return -1;
	}

	pr_debug("%s: start = 0x%lx, size = %ld dest=0x%x\n", __func__, start, size, (u32) data);

	if ( panel_sfm_size < (start + size) )
	{
		pr_debug("Attempting to read off the end of flash, start = %ld, length %ld\n",
				start, size);
		return -1;
	}

	tx_buf = kzalloc(SFM_READ_CMD_LEN, GFP_KERNEL);
	if (!tx_buf) {
		pr_debug("Can't alloc spi tx buffer, length %d\n", SFM_READ_CMD_LEN);
		return -1;
	}

	len = size;
	xmit_buf = data;

	if (len > 0) {
		xfer_len = len;
		/* handle small reads */
		if (xfer_len % 4) {
			extra = (4 - (xfer_len % 4));
			xfer_len += extra;
		} else {
			extra = 0;
		}

		rx_buf = kzalloc(xfer_len, GFP_KERNEL);
		if (!rx_buf) {
			pr_debug("Can't alloc spi rx buffer, length %d\n", xfer_len);

			kfree(tx_buf);

			return -1;
		}

		spi->mode = SPI_MODE_0;
		spi->is_fast_read = 1;
		spi_setup(spi);

		/* command is 1 byte, addr is 3 bytes, MSB first */
		tx_buf[3] = SFM_READ;
		tx_buf[2] = (start >> 16) & 0xFF;
		tx_buf[1] = (start >> 8) & 0xFF;
		tx_buf[0] = start & 0xFF;

		memset(&t, 0, sizeof(t));
		t.tx_buf = (const void *) tx_buf;
		t.rx_buf = (void *) rx_buf;
		t.len = xfer_len / 4;

		spi_message_init(&m);

		spi_message_add_tail(&t, &m);
		if (spi_sync(spi, &m) != 0 || m.status != 0) {
			printk(KERN_ERR "err on cmd %d\n", m.status);
			ret = -1;
			goto free_txrx;
		}

		if ((xfer_len - (m.actual_length * 4)) != 0) {
			printk(KERN_ERR "only %d bytes sent\n", (m.actual_length * 4));
			ret = -1;
			goto free_txrx;
		}

		//xmit_len = (xfer_len - SFM_READ_CMD_LEN - extra);
		xmit_len = (xfer_len - extra);
		//rcv_buf = (u32 *)(rx_buf + SFM_READ_CMD_LEN);
		rcv_buf = (u32 *)rx_buf;

		/* need to byteswap */
		for (i = 0; i < (xmit_len / 4); i++) {
			*((__u32 *) xmit_buf) = __swab32p(rcv_buf);
			xmit_buf += 4;
			rcv_buf++;
		}

		/* handle requests smaller than 4 bytes */
		if (extra) {
			for (i = 0; i < (4 - extra); i++) {
				((u8 *) xmit_buf)[i] = (rcv_buf[0] >> ((3 - i) * 8)) & 0xFF;
			}
		}

		start += xmit_len;
		len -= xmit_len;

		ret = 0;
	}

free_txrx:
	kfree(tx_buf);
	kfree(rx_buf);

	return ret;
}

static bool supports_panel_data_read(void) {
    return panel_flash_present();
}

static int panel_data_read(u32 start_addr, u8 *buffer, int to_read)
{
	int result = -1;

	if (supports_panel_data_read() && buffer && IN_RANGE(to_read, 1, PNL_SIZE)) {
		panel_flash_select saved_flash_select = get_flash_select();
		set_flash_select(panel_flash_commands);

		panel_read_from_flash((PNL_FLASH_BASE+start_addr), buffer, to_read);
		set_flash_select(saved_flash_select);

		panel_data_translate((u8 *)buffer, to_read);
		result = to_read;
	}

	return result;
}

void einkwf_set_buffer(char *buffer)
{
	// Use the internal buffer by default.
	//
	wf_buffer = buffer ? buffer : wf_buff;
}

char *einkwf_get_buffer(void)
{
	return ( wf_buffer );
}

void einkwf_set_buffer_size(int size)
{
	// Clip to WF_BUFF_SIZE if we're using wf_buff.
	//
	if ( wf_buffer == wf_buff )
		wf_buffer_size = (WF_BUFF_SIZE >= size) ? size : WF_BUFF_SIZE;
	else
		wf_buffer_size = size;
}

int einkwf_get_buffer_size(void)
{
	return ( wf_buffer_size );
}

static void einkwf_buffers_set(void)
{
	buffer_byte  = einkwf_get_buffer();
	buffer_short = (unsigned short *)buffer_byte;
	buffer_long  = (unsigned long *)buffer_byte;
}

static void einkwf_buffers_clr(void)
{
	buffer_byte  = NULL;
	buffer_short = NULL;
	buffer_long  = NULL;
}

int eink_read_byte(unsigned long addr, unsigned char *data)
{
    if ( data && buffer_byte )
        *data = buffer_byte[addr];

    return ( NULL != buffer_byte );
}

int eink_read_short(unsigned long addr, unsigned short *data)
{
	if ( data && buffer_short && buffer_byte )
	{
		// Read 16 bits if we're 16-bit aligned.
		//
		if ( addr == ((addr >> 1) << 1) )
			*data = buffer_short[addr >> 1];
		else
			*data = (buffer_byte[addr + 0] << 0) |
				(buffer_byte[addr + 1] << 8);
	}

	return ( NULL != buffer_short );
}

int eink_read_long(unsigned long addr, unsigned long *data)
{
	if ( data && buffer_long && buffer_byte )
	{
		// Read 32 bits if we're 32-bit aligned.
		//
		if ( addr == ((addr >> 2) << 2) )
			*data = buffer_long[addr >> 2];
		else
			*data = (buffer_byte[addr + 0] <<  0) |
				(buffer_byte[addr + 1] <<  8) |
				(buffer_byte[addr + 2] << 16) |
				(buffer_byte[addr + 3] << 24);
	}

	return ( NULL != buffer_long );
}

void eink_get_waveform_info(struct eink_waveform_info_t *info)
{
	if ( info )
	{
		unsigned char checksum1, checksum2;

		eink_read_byte(EINK_ADDR_WAVEFORM_VERSION,     &info->waveform.version);
		eink_read_byte(EINK_ADDR_WAVEFORM_SUBVERSION,  &info->waveform.subversion);
		eink_read_byte(EINK_ADDR_WAVEFORM_TYPE,        &info->waveform.type);
		eink_read_byte(EINK_ADDR_RUN_TYPE,             &info->waveform.run_type);
		eink_read_byte(EINK_ADDR_FPL_PLATFORM,         &info->fpl.platform);
		eink_read_byte(EINK_ADDR_FPL_SIZE,             &info->fpl.size);
		eink_read_byte(EINK_ADDR_ADHESIVE_RUN_NUM,     &info->fpl.adhesive_run_number);
		eink_read_byte(EINK_ADDR_MODE_VERSION,         &info->waveform.mode_version);
		eink_read_byte(EINK_ADDR_MFG_CODE,             &info->waveform.mfg_code);
		eink_read_byte(EINK_ADDR_CHECKSUM1,            &checksum1);
		eink_read_byte(EINK_ADDR_CHECKSUM2,            &checksum2);

		if (info->waveform.type == EINK_WAVEFORM_TYPE_WR) {
			/* WR spec changes the definition of the byte at 0x16 */
			eink_read_byte(EINK_ADDR_WAVEFORM_REV, &info->waveform.revision);

			/* VCOM shift only on WR panels */
			eink_read_byte(EINK_ADDR_VCOM_SHIFT, &info->waveform.vcom_shift);
		} else {
			eink_read_byte(EINK_ADDR_WAVEFORM_TUNING_BIAS, &info->waveform.tuning_bias);
			info->waveform.revision = 0;
			info->waveform.vcom_shift = 0;
		}

		eink_read_byte(EINK_ADDR_FPL_RATE,             &info->waveform.fpl_rate);

		eink_read_short(EINK_ADDR_FPL_LOT,             &info->fpl.lot);

		eink_read_long(EINK_ADDR_CHECKSUM,             &info->checksum);
		eink_read_long(EINK_ADDR_FILESIZE,             &info->filesize);
		eink_read_long(EINK_ADDR_SERIAL_NUMBER,        &info->waveform.serial_number);

		/* XWIA is only 3 bytes */
		eink_read_long(EINK_ADDR_XWIA,		       &info->waveform.xwia);
		info->waveform.xwia &= 0xFFFFFF;

		if ( 0 == info->filesize ) {
			info->checksum = EINK_CHECKSUM(checksum1, checksum2);
			info->waveform.parse_wf_hex  = false;
		} else {
			info->waveform.parse_wf_hex  = false;
		}

		wfm_debug(   "\n"
				" Waveform version:  0x%02X\n"
				"       subversion:  0x%02X\n"
				"             type:  0x%02X (v%02d)\n"
				"         run type:  0x%02X\n"
				"     mode version:  0x%02X\n"
				"      tuning bias:  0x%02X\n"
				"       frame rate:  0x%02X\n"
				"       vcom shift:  0x%02X\n"
				"\n"
				"     FPL platform:  0x%02X\n"
				"              lot:  0x%04X\n"
				"             size:  0x%02X\n"
				" adhesive run no.:  0x%02X\n"
				"\n"
				"        File size:  0x%08lX\n"
				"         Mfg code:  0x%02X\n"
				"       Serial no.:  0x%08lX\n"
				"         Checksum:  0x%08lX\n",

				info->waveform.version,
				info->waveform.subversion,
				info->waveform.type,
				info->waveform.revision,
				info->waveform.run_type,
				info->waveform.mode_version,
				info->waveform.tuning_bias,
				info->waveform.fpl_rate,
				info->waveform.vcom_shift,

				info->fpl.platform,
				info->fpl.lot,
				info->fpl.size,
				info->fpl.adhesive_run_number,

				info->filesize,
				info->waveform.mfg_code,
				info->waveform.serial_number,
				info->checksum);
	}
}

bool eink_waveform_valid(eink_waveform_info_t *info)
{
	if (info) {
		if (info->filesize <= WFM_HDR_SIZE || info->filesize > EINK_WAVEFORM_FILESIZE) {
			printk(KERN_ERR "eink_fb_waveform: E invalid:Invalid filesize in waveform header:\n");
			return false;
		}
	} else {
		printk(KERN_ERR "eink_fb_waveform: E invalid: info is NULL\n");
		return false;
	}

	return (true);
}

void einkwf_get_waveform_info(eink_waveform_info_t *info)
{
    einkwf_buffers_set();

    eink_get_waveform_info(info);

    einkwf_buffers_clr();
}

unsigned long eink_get_computed_waveform_checksum(unsigned char *buffer)
{
	unsigned long checksum = 0;

	if ( buffer )
	{
		unsigned long *long_buffer = (unsigned long *)buffer,
			      filesize = long_buffer[EINK_ADDR_FILESIZE >> 2];

		if ( filesize )
		{
			unsigned long saved_embedded_checksum;

			// Save the buffer's embedded checksum and then set it zero.
			//
			saved_embedded_checksum = long_buffer[EINK_ADDR_CHECKSUM >> 2];
			long_buffer[EINK_ADDR_CHECKSUM >> 2] = 0;

			// Compute the checkum over the entire buffer, including
			// the zeroed-out embedded checksum area, and then restore
			// the embedded checksum.
			//
			checksum = crc32((unsigned char *)buffer, filesize);
			long_buffer[EINK_ADDR_CHECKSUM >> 2] = saved_embedded_checksum;
		}
		else
		{
			unsigned char checksum1, checksum2;
			int start, length;

			// Checksum bytes 0..(EINK_ADDR_CHECKSUM1 - 1).
			//
			start     = 0;
			length    = EINK_ADDR_CHECKSUM1;
			checksum1 = sum8(&buffer[start], length);

			// Checksum bytes (EINK_ADDR_CHECKSUM1 + 1)..(EINK_ADDR_CHECKSUM2 - 1).
			//
			start     = EINK_ADDR_CHECKSUM1 + 1;
			length    = EINK_ADDR_CHECKSUM2 - start;
			checksum2 = sum8(&buffer[start], length);

			checksum  = EINK_CHECKSUM(checksum1, checksum2);
		}
	}

	return ( checksum );
}

/* Waveform */
extern u8 *panel_get_waveform_from_flash(int offset, u8 *buffer, int buffer_size)
{
	if ( panel_flash_present() )
	{
		panel_flash_select saved_flash_select = get_flash_select();

		wfm_debug("Reading waveform.. (%d bytes)\n", buffer_size);
		set_flash_select(panel_flash_waveform);
		panel_read_from_flash(offset, buffer, buffer_size);

		set_flash_select(saved_flash_select);
	}
	else {
		printk("%s: no flash!\n", __func__);
		memset(&buffer[offset], 0, buffer_size);
	}
	return ( buffer );
}

extern void panel_get_waveform(u8 *buffer, int buffer_size)
{
	wfm_debug("%s, enter...size=%ld\n", __func__,panel_waveform_size);
int i;
	// If we haven't read the waveform in before (or we need to read it again), read it now
	if (0 == panel_waveform_size) {
		u8 *which_buffer = NULL;
		int which_size = 0;
		eink_waveform_info_t waveform_info;

		which_buffer = panel_get_waveform_from_flash(0, panel_waveform_buffer, WFM_HDR_SIZE);

		which_size = WFM_HDR_SIZE;

		einkwf_set_buffer_size(which_size);
		einkwf_set_buffer(panel_waveform_buffer);

		einkwf_get_waveform_info(&waveform_info);

		if (eink_waveform_valid(&waveform_info)) {

			// If we're actually reading from flash itself, we won't have been passed a
			// buffer to use.  And, in that case, we only read in the header to validate
			// with.  Now that we know it's valid, we read in the rest of it unless
			// we've been told not to.
			//
			if (!panel_waveform_header_only) {
				if ( !buffer ) {
					wfm_debug("%s: reading waveform from flash begin...\n", __FUNCTION__);
					panel_get_waveform_from_flash(WFM_HDR_SIZE,
							(which_buffer + WFM_HDR_SIZE),
							(waveform_info.filesize - WFM_HDR_SIZE));
					wfm_debug("%s: reading waveform from flash end...\n", __FUNCTION__);
				}
				pr_debug("%s: verifying waveform checksum\n", __FUNCTION__);
				// Verify waveform checksum
				if (eink_get_computed_waveform_checksum(which_buffer) != waveform_info.checksum) {
					printk(KERN_ERR "eink_fb_waveform: E invalid:Invalid waveform checksu     m:\n");
					return;
				}
			}

			printk("%s: read waveform size %ld\n", __FUNCTION__, waveform_info.filesize);
			panel_waveform_size = waveform_info.filesize;
			einkwf_set_buffer_size(panel_waveform_size);
		}
	} else {
		printk(KERN_DEBUG"%s: verifying waveform checksum\n", __FUNCTION__);
	}
}

void panel_get_waveform_info(eink_waveform_info_t *info)
{
	if (info) {
		panel_get_waveform(NULL, 0);
		einkwf_get_waveform_info(info);
	}
}

/* BCD */
bool supports_panel_bcd(void)
{
	return (panel_flash_present());
}

char *panel_get_bcd(void)
{
	if (!supports_panel_bcd()) {
		return NULL;
	}

	// If the panel ID hasn't already been read in, then read it in now.
	//
	if (panel_bcd[0] == 0)
	{
		u8 bcd[PNL_SIZE_BCD] = { 0 };

		panel_data_read(PNL_BASE_BCD, bcd, PNL_SIZE_BCD);
		strncpy(panel_bcd, bcd, PNL_SIZE_BCD);
		panel_bcd[PNL_SIZE_BCD] = '\0';
	}

	wfm_debug("%s: panel bcd=%s\n", __FUNCTION__, panel_bcd);

	return (panel_bcd);
}

static bool supports_panel_id(void)
{
	return (panel_flash_present());
}

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

char *panel_get_id(void)
{
	if ( ! supports_panel_id() )
		return NULL;

	// If the panel ID hasn't already been read in, then read it in now.
	//
	if ( !(('_' == panel_id[4]) && ('_' == panel_id[8]) && ('_' == panel_id[11])) )
	{
		u8 panel_buffer[PNL_SIZE] = { 0 };
		char *part_number;
		int cur;

		// Waveform file names are of the form PPPP_XLLL_DD_TTVVSS_B, and
		// panel IDs are of the form PPPP_LLL_DD_MMM.
		//
		panel_data_read(PNL_BASE, panel_buffer, PNL_SIZE);

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

	wfm_debug("%s: panel id=%s\n", __FUNCTION__, panel_id);

	return ( panel_id );
}

/* VCOM */
#define VCOM_INT_TO_STR(i, s)   \
    sprintf((s), "-%1d.%02d", ((i)/100)%10, (i)%100)

#define VCOM_STR_READ()      \
    (('-' == panel_vcom_str[0]) && ('.' == panel_vcom_str[2]))

static char *panel_get_vcom_str(void)
{
	pr_debug("%s begin\n", __FUNCTION__);

	// If the VCOM hasn't already been read in, read it in now.
	//
	if ( !VCOM_STR_READ() )
	{
		u8 vcom_str[PNL_SIZE_VCOM] = { 0 };

		panel_data_read(PNL_BASE_VCOM, vcom_str, PNL_SIZE_VCOM);
		strncpy(panel_vcom_str, vcom_str, PNL_SIZE_VCOM);

		// If the VCOM string returned from the panel data is invalid, then
		// use the default one instead.
		//
		panel_vcom_str[PNL_SIZE_VCOM] = '\0';
		wfm_debug("%s, vcom_str: %s\n", __func__, panel_vcom_str);

		if ( !panel_data_valid(panel_vcom_str) )
		{
			return NULL;
		}
	}

	pr_debug("%s vcom=%s\n", __FUNCTION__, panel_vcom_str);

	return ( panel_vcom_str );
}

#define BCD_MODULE_MFG_OFFSET  9

#define VCOM_READ() \
	(0 != panel_vcom)

static int panel_get_vcom(void)
{
	pr_debug("%s begin\n", __FUNCTION__);

	if ( !VCOM_READ() )
	{
		char *vcom_str = panel_get_vcom_str();
		int i;
		char mfg_code = 0;
		eink_waveform_info_t info;

		if (vcom_str == NULL) {
			panel_vcom = 0;
			return false;
		}

		// Parse the VCOM value.
		//
		if ('-' == (char)vcom_str[0])
		{
			// Skip the negative sign (i.e., i = 1, instead of i = 0).
			//
			for( i = 1; i < PNL_SIZE_VCOM; i++ )
			{
				// Skip the dot.
				//
				if ( '.' == (char)vcom_str[i] )
					continue;

				if ( (vcom_str[i] >= '0') && (vcom_str[i] <= '9') )
				{
					panel_vcom *= 10;
					panel_vcom += ((char)vcom_str[i] - '0');
				}
			}
		}

		// For Celeste: Check to see if the mfg code indicates a 400 mV VCOM shift applied to VCOM value
#if 1
		panel_get_waveform_info(&info);

		panel_data_read(PNL_BASE_BCD + BCD_MODULE_MFG_OFFSET, &mfg_code, 1);
		if (mfg_code == 'A' || mfg_code == 'B' ||
				mfg_code == 'C' || mfg_code == 'D' ||
				mfg_code == 'Q' || mfg_code == 'U')
		{
			// We are using an old waveform on a new panel, need to add VCOM shift
			if (!info.waveform.vcom_shift) {
				panel_vcom += 40;
				printk(KERN_INFO "eink_fb_waveform: I vcom:waveform not tuned for 400mV VCOM shift:\n");
			}

		} else {
			// We are using a -400mV tuned waveform on an old panel
			if (info.waveform.vcom_shift) {
				panel_vcom -= 40;
				printk(KERN_INFO "eink_fb_waveform: I vcom:override waveform tuned for 400mV VCOM shift:\n");
			}
		}
#endif

	}

	pr_debug("%s vcom=%d\n", __FUNCTION__, panel_vcom);

	return ( panel_vcom );
}


#define panel_spi_sync(spi, message, result, fmt, args...) \
({ \
	int _ret = 0; \
	if (spi_sync((spi), (message)) != 0 || (message)->status != 0) { \
		printk(KERN_ERR "%s: SPI Error (%d): "fmt, __func__, (message)->status, ##args); \
		*(result) = (message)->status; \
		_ret = 1; \
	} \
	_ret; \
})
static int spi_flash_open(struct inode *inode, struct file *file)
{
	printk("enter spi flash open.\n");
	file->f_pos = 0;
	return 0;
}


u8 *spi_flash_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	u32 addr = 0;
	u8* kbuf = NULL;
	int i;
	printk("enter spi flash read.\n");

	kbuf=panel_get_waveform_from_flash(0, panel_waveform_buffer, EINK_WAVEFORM_FILESIZE);
	//for(i=0;i<0x30;i++)
	//	printk("kbuf[%d]=0x%02x--\n",i,kbuf[i]);
	// get waveform id
	buf = panel_get_id();
	for(i=0;i<PNL_SIZE_ID_STR;i++)
	kbuf[EINK_WAVEFORM_FILESIZE+i]=buf[i];
	/*
	if(copy_to_user(buf, kbuf, EINK_WAVEFORM_FILESIZE))
	{
		printk("enter spi flash copy fail.\n");
		kfree(kbuf);
		return -EFAULT;
	}
	*/
	return kbuf;
}



static struct file_operations spi_flash_fops = {
	.owner	= THIS_MODULE,
	.open	= spi_flash_open,
	//.write	= spi_flash_write,
	.read	= spi_flash_read,
	//.llseek = spi_flash_seek
};

/*!
 * This function is called whenever the SPI slave device is detected.
 * @param	spi	the SPI slave device
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int panel_flash_probe(struct spi_device *spi)
{
	u32 cmd, res;
	struct spi_transfer t;
	struct spi_message m;
	u8 mfg;
	int ret;
	int i;
	short flash_major = 0;
	dev_t devno; 
	int err =0;

	/* Setup the SPI slave */
	spi_flash_info.dev = spi;
	
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
		pr_err("no mem.\n");
		return ENOMEM;
	}
	cdev_init(spi_flash_info.cdev, &spi_flash_fops);
	spi_flash_info.cdev->owner = THIS_MODULE;
	spi_flash_info.cdev->ops = &spi_flash_fops;
	err = cdev_add(spi_flash_info.cdev, devno, 1);
	if(err)
		pr_err("adding spi flash error.\n");

	spi_flash_info.my_class = class_create(THIS_MODULE, "spi_flash");
	if(IS_ERR(spi_flash_info.my_class)) 
	{
		pr_err("Err: failed in creating spi flash class.\n");
		return -1; 
	} 
	device_create(spi_flash_info.my_class, NULL, devno,NULL, "spi_flash"); 

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 32;
	spi_setup(spi);

	/* command is 1 byte, addr is 3 bytes */
	cmd = (SFM_ID << 24);
	res = 0;

	memset(&t, 0, sizeof(t));
	t.tx_buf = (const void *) &cmd;
	t.rx_buf = (void *) &res;
	t.len = 4;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (panel_spi_sync(spi, &m, &ret, "reading flash ID"))
		return ret;

	// see if we received a valid response
	mfg = (res >> 16) & 0xFF;
	wfm_debug("%s, mfg: 0x%02x\n", __func__, mfg);
	switch (mfg) {
		case 0x20:
			pr_debug("M25P20 flash detected\n");
			break;
		case 0xc2:
			pr_debug("MX25L2005 flash detected\n");
			break;
		case 0xef:
			pr_debug("MX25U4035 flash detected\n");
			break;

		case 0x00:
			/* BEN TODO - fix this */
			printk(KERN_ERR "Bad flash signature: 0x%x\n", res);
			spi_flash_info.dev = NULL;
			return -1;

		default:
			pr_debug("Unrecognized flash: 0x%x\n", mfg);
			break;
	}

	/* get what we need */
	/*
	if (wf_to_use == NULL) {
		wf_to_use = kzalloc(WF_PATH_LEN, GFP_KERNEL);
	}
	*/

	if (!panel_flash_present()) {
		printk(KERN_ERR "no panel present, use builtin waveform\n");
		// TODO: use builtin waveform
		return -EINVAL;
	}

	panel_get_bcd();
	panel_get_id();

	panel_get_waveform(NULL, 0);
#if 1
	/* see the first/last 4 byte */
	for (i = 0; i<4; i++)
		printk("[%d]:%02x\n", i, panel_waveform_buffer[i]);
	for (i = panel_waveform_size-4; i< panel_waveform_size; i++)
		printk("[%d]:%02x\n", i, panel_waveform_buffer[i]);
#endif

	wfm_debug("%s, vcom voltage: %d\n", __func__, panel_get_vcom());

	epd_spi_flash_register(&spi_flash_info);

	return 0;
}

/*!
 * This function is called whenever the SPI slave device is removed.
 * @param   spi - the SPI slave device
 * @return  Returns 0 on SUCCESS and error on FAILURE.
 */
static int panel_flash_remove(struct spi_device *spi)
{
	printk(KERN_INFO "Device %s removed\n", dev_name(&spi->dev));
	spi_flash_info.dev = NULL;

	return 0;
}

static struct spi_driver panel_flash_driver = {
	.driver = {
		.name = "panel_flash_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = panel_flash_probe,
	.remove = panel_flash_remove,
};

#if 0
bool panel_flash_init(void)
{
	int ret = spi_register_driver(&panel_flash_driver);

	pr_debug("%s: begin\n", __func__);

	if (ret) {
		/* BEN TODO - fix logging */
		printk(KERN_ERR "spi driver registration failed: %d\n", ret);
		spi_registered = false;
	} else {
		spi_registered = true;
	}

	return spi_registered;
}

void panel_flash_exit(void)
{
	pr_debug("%s: begin\n", __func__);

	if (spi_registered) {
		spi_unregister_driver(&panel_flash_driver);
		spi_registered = false;
	}
}
#endif

static int __init panel_flash_init(void)
{
	int ret = spi_register_driver(&panel_flash_driver);

	pr_debug("%s: begin\n", __func__);

	if (ret) {
		/* BEN TODO - fix logging */
		printk(KERN_ERR "spi driver registration failed: %d\n", ret);
		spi_registered = false;
	} else {
		spi_registered = true;
	}

	return spi_registered;
}

static void __exit panel_flash_exit(void)
{
	pr_debug("%s: begin\n", __func__);

	if (spi_registered) {
		spi_unregister_driver(&panel_flash_driver);
		spi_registered = false;
	}
}
subsys_initcall(panel_flash_init);

//module_init(panel_flash_init);
module_exit(panel_flash_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("kindle");
