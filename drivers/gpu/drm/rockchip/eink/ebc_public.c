#include "ebc.h"

void ebc_io_ctl_hook(unsigned int cmd, struct ebc_buf_info *info)
{
	switch (cmd) {
	case GET_EBC_BUFFER:
		/*
		 * Here you can check the struct ebc_buf_info.
		 *   buf_info.offset,
		 *   buf_info.height,      buf_info.width,
		 *   buf_info.vir_width,   buf_info.vir_height,
		 *   buf_info.fb_width,    buf_info.fb_height,
		 *   buf_info.color_panel, buf_info.rotate,
		 */
		break;
	case SET_EBC_SEND_BUFFER:
		/*
		 * Here you can translate struct ebc_buf_info to struct ebc_buf_s
		 *   struct ebc_buf_s *buf;
		 *   char *temp_addr;
		 *   temp_addr = ebc_phy_buf_base_get() + buf_info.offset;
		 *   buf = ebc_find_buf_by_phy_addr(temp_addr);
		 * then check the number of struct ebc_buf_s
		 *   buf->buf_mode,
		 *   buf->win_x1, buf->win_x2,
		 *   buf->win_y1, buf->win_y2,
		 */
		break;
	default:
		break;
	}
}

/*
 * The bellow functions are provided the personalized configuration,
 * customs could adjust them though the kernel make menuconfig.
 *
 *	get_waveform_type()
 *	get_waveform_position()
 *
 *	set_end_display()
 *
 *	support_bootup_ani()
 *	is_bootup_ani_loop()
 *	get_bootup_ani_mode()
 *	get_bootup_logo_cycle()
 *
 *	is_need_show_lowpower_pic()
 *
 *	support_tps_3v3_always_alive()
 */
int get_waveform_type()
{
#ifdef CONFIG_RKF_WAVEFORM
	return RKF_WAVEFORM;
#elif defined(CONFIG_PVI_WAVEFORM)
	return PVI_WAVEFORM;
#elif defined(CONFIG_OED_WAVEFORM)
	return OED_WAVEFORM;
#else
	return RKF_WAVEFORM;
#endif
}

int get_waveform_position()
{
#ifdef CONFIG_WAVEFORM_FROM_GPIO_SPI
	return LUT_FROM_GPIO_SPI_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_RK_SPI)
	return LUT_FROM_RK_SPI_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_NAND_FLASH)
	return LUT_FROM_NAND_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_WAVEFORM_FILE)
	return LUT_FROM_WAVEFORM_FILE;
#endif
}

int set_end_display()
{
#ifdef CONFIG_END_PICTURE
	return END_PICTURE;
#else
	return END_RESET;
#endif
}

int get_bootup_logo_cycle(void)
{
	return CONFIG_EBC_ANI_CYC_TIME;
}

int is_bootup_ani_loop(void)
{
#ifdef CONFIG_EBC_BOOT_ANI_LOOP
	return 1;
#else
	return 0;
#endif
}

int is_need_show_lowpower_pic(void)
{
#ifdef CONFIG_SHOW_BATLOW_PIC
	return 1;
#else
	return 0;
#endif
}

int support_bootup_ani(void)
{
#ifdef CONFIG_EBC_BOOT_ANI
	return 1;
#else
	return 0;
#endif
}

int support_double_thread_calcu(void)
{
#ifdef CONFIG_DOUBLE_THREAD_CALCU
	return 1;
#else
	return 0;
#endif
}

int get_bootup_ani_mode(void)
{
#ifdef CONFIG_EBC_BOOT_ANI_A2
	return EPD_A2;
#elif defined(CONFIG_EBC_BOOT_ANI_PART)
	return EPD_PART;
#elif defined(CONFIG_EBC_BOOT_ANI_FULL)
	return EPD_FULL;
#elif defined(CONFIG_EBC_BOOT_ANI_DIR)
	return EPD_DIRECT_PART;
#else
	return EPD_FULL;
#endif
}

int support_tps_3v3_always_alive(void)
{
#ifdef CONFIG_EPD_PMIC_VccEink_ALWAYS_ALIVE
	return 1;
#else
	return 0;
#endif
}

int map_auto_mode(void)
{
#ifdef CONFIG_AUTO_TO_GC16
	return WF_TYPE_GC16;
#elif defined(CONFIG_AUTO_TO_GL16)
	return WF_TYPE_GL16;
#elif defined(CONFIG_AUTO_TO_GLR16)
	return WF_TYPE_GLR16;
#elif defined(CONFIG_AUTO_TO_GLD16)
	return WF_TYPE_GLD16;
#else
	return WF_TYPE_GL16;
#endif
}

int map_gray16_mode(void)
{
#ifdef CONFIG_GRAY16_TO_GC16
	return WF_TYPE_GC16;
#elif defined(CONFIG_GRAY16_TO_GL16)
	return WF_TYPE_GL16;
#elif defined(CONFIG_GRAY16_TO_GLR16)
	return WF_TYPE_GLR16;
#elif defined(CONFIG_GRAY16_TO_GLD16)
	return WF_TYPE_GLD16;
#else
	return WF_TYPE_GL16;
#endif
}
