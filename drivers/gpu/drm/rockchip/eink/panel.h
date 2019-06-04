#ifndef _EINK_PANEL_H_
#define _EINK_PANEL_H_

struct eink_panel {
	struct device *dev;
	u32 width;
	u32 height;
	u32 vir_width;
	u32 vir_height;
	u32 fb_width;
	u32 fb_height;

	u32 sdck;
	u32 pixels_per_sdck;
	u32 lsl;
	u32 lbl;
	u32 ldl;
	u32 lel;
	u32 gdck_sta;
	u32 lgonl;
	u32 fsl;
	u32 fbl;
	u32 fdl;
	u32 fel;

	struct {
		u32 width;
		u32 height;
		u32 bytes_per_pixel;
		u32 bytes_per_row;
	} mcu;

	u32 h_count;
	u32 v_count;
	u32 line_w;
	u32 line_h;
	u32 top_frame;
	u32 eink_end;
	u32 top_data;
	u32 eink_totle;
	u32 line_rel;
	u8 *ink_group;
	u8 *ink_group_next;
	int color_panel;
	int rotate;
	int vcom_mv;
};

#endif	/* _EINK_TCON_H_ */
