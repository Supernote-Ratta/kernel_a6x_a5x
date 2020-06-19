/*
 * buf_manage.h
 *
 *author:dlx@rock-chips.com
 *date:2012/7/3
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 */


#ifndef _BUF_MANAGE_H_
#define _BUF_MANAGE_H_
/*define*/
#define BUF_ERROR ((buf_int)-1)
#define BUF_SUCCESS ((buf_int)0)

#define BUSY ((buf_int)1)
#define IDLE ((buf_int)0)

/* typedef*/
typedef unsigned int buf_uint;
typedef int               buf_int;
typedef char            buf_char;
typedef void            buf_void;

/*enum*/
// buffer status.
enum ebc_buf_status_e{
	buf_idle,
	buf_busy,

	buf_error,
};


/*struct*/
struct ebc_buf_s{
	enum ebc_buf_status_e status; //buffer status.
	buf_char *phy_addr; //buffer physical address.
	buf_char *virt_addr; //buffer virtual address.

	int buf_mode;
	buf_int len; //buffer length
	int win_x1;
	int win_y1;
	int win_x2;
	int win_y2;
};

/*function*/


buf_int ebc_buf_release(struct ebc_buf_s *release_buf);
buf_int ebc_remove_from_dsp_buf_list(struct ebc_buf_s *remove_buf);
buf_int ebc_add_to_dsp_buf_list(struct ebc_buf_s  *dsp_buf, int add_direct);
buf_int ebc_get_dsp_list_enum_num(void);
struct ebc_buf_s *ebc_dsp_buf_get(void);
struct ebc_buf_s *ebc_find_buf_by_phy_addr(buf_char *phy_addr);
struct ebc_buf_s *ebc_empty_buf_get(void);
buf_char *ebc_phy_buf_base_get(void);
buf_char *ebc_virt_buf_base_get(void);

buf_int ebc_buf_uninit(void);
buf_int ebc_buf_init(buf_char *phy_start,buf_char *mem_start,buf_int men_len,buf_int dest_buf_len);

#endif

