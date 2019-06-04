/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	bootanim.h
Desc	:	boot animation data 
Author	:	DLX
Date	:	2013-1-5
Notes	:	
********************************************************************/
#ifndef _BOOTANIM_H_
#define _BOOTANIM_H_

/*include*/
#define BOOT_ANI_ERROR (-1)
#define BOOT_ANI_SUCCESS (0)

/*struct*/
// Header.
typedef struct sAniHeader {
	int  mTotalSize;
	int  mScreenW;
	int  mScreenH;
	int  mImageCount;
	int  mImageInfoOffset;
	int  mCompressType;
	char mVersion[4];
	int  mReserve[8];
} tAniHeader;

// Image type.
typedef enum ImageType {
	ANIMATION_IMAGE = 0,
	LOWPOWER_IMAGE	= 1
} tImageType;

// Image.
typedef struct sImage_info {
	int 		mX;
	int 		mY;
	int 		mW;
	int 		mH;
	tImageType	mType;
	int 		mDataSize;
	int 		mDataOffset;
	int 		mOrignalSize;
	int 		mReserver[8];
} __packed tImage_info;

// Image.
typedef struct sImage {
	tImage_info	mImage_info;
	char *		mData;
} tImage;

void boot_ani_deinit(void);
int boot_ani_init(void);
int boot_ani_next_frame(struct eink_panel *panel_info,struct ebc_buf_s *dest_buf);
int boot_ani_get_low_power_frame(struct eink_panel *panel_info,struct ebc_buf_s *dest_buf);
int boot_ani_low_power_frame_rm(void);

#endif//#ifndef _BOOTANIM_H_


