#ifndef _EINKWF_H
#define _EINKWF_H

#include <linux/types.h>

struct eink_waveform_t
{
    unsigned char   version,			// EINK_ADDR_WAVEFORM_VERSION
                    subversion,			// EINK_ADDR_WAVEFORM_SUBVERSION
                    type,			// EINK_ADDR_WAVEFORM_TYPE
                    run_type,			// EINK_ADDR_RUN_TYPE
                    mode_version,		// EINK_ADDR_MODE_VERSION
                    mfg_code,			// EINK_ADDR_MFG_CODE
                    tuning_bias,		// EINK_ADDR_WAVEFORM_TUNING_BIAS
                    revision,			// EINK_ADDR_WAVEFORM_REV (WR-spec only)
		    fpl_rate,			// EINK_ADDR_FPL_RATE
                    vcom_shift;			// EINK_ADDR_VCOM_SHIFT
    unsigned long   serial_number;		// EINK_ADDR_SERIAL_NUMBER
    unsigned long   xwia;			// EINK_ADDR_XWIA

    bool            parse_wf_hex;
};
typedef struct eink_waveform_t eink_waveform_t;

struct eink_fpl_t
{
    unsigned char   platform,			// EINK_ADDR_FPL_PLATFORM
                    size,			// EINK_ADDR_FPL_SIZE
                    adhesive_run_number;	// EINK_ADDR_ADHESIVE_RUN_NUM

    unsigned short  lot;			// EINK_ADDR_FPL_LOT
};
typedef struct eink_fpl_t eink_fpl_t;

struct eink_waveform_info_t
{
    struct eink_waveform_t waveform;
    struct eink_fpl_t fpl;

    unsigned long   filesize,                   // EINK_ADDR_FILESIZE
                    checksum;                   // EINK_ADDR_FILESIZE ? EINK_ADDR_CHECKSUM : (EINK_ADDR_CHECKSUM2 << 16) | EINK_ADDR_CHECKSUM1
};
typedef struct eink_waveform_info_t eink_waveform_info_t;

#endif /* _EINKWF_H */
