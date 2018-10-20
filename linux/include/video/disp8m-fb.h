/*
 * include/video/mb86s70-fb.h
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 */

#ifndef ___LINUX_VIDEO_F_IRIS_H__
#define ___LINUX_VIDEO_F_IRIS_H__

#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#include <video/fdb.h>

enum {
	/*
	 * APB region
	 */
	DISP8M_APB_MAIN_OFS	= 0,
	DISP8M_APB_OUT_OFS	= 0x1000,
	DISP8M_APB_OSD0_OFS	= 0x2000,
	DISP8M_APB_OSD1_OFS	= 0x3000,

	/*
	 * AHB region
	 */
	DISP8M_AHB_MAIN_OFS	= 0,
	DISP8M_AHB_OSD0_OFS	= 0x8000,
	DISP8M_AHB_OSD1_OFS	= 0xc000,
};

/* Layout of APB MAIN register block */

enum disp8m_apb_main_reg {
	DISP8M_APB_MAIN_LRST_OFS	= DISP8M_APB_MAIN_OFS + 0,
		DISP8M_APB_MAIN_LRST__SR__SHIFT = 0,
		DISP8M_APB_MAIN_LRST__SR__MASK = 1,

	DISP8M_APB_MAIN_LTRG_OFS	= DISP8M_APB_MAIN_OFS + 0x100,
		DISP8M_APB_MAIN_LTRG__TRG__SHIFT = 0,
		DISP8M_APB_MAIN_LTRG__TRG__MASK = 3,
			LTRG_FORCED_STOP = 0,
			LTRG_START = 1,
			LTRG_FRAME_STOP = 2,
			LTRG_NOP = 3,

	DISP8M_APB_MAIN_LRPGCTL_OFS	= DISP8M_APB_MAIN_OFS + 0x110,
		DISP8M_APB_MAIN_LRPGCTL__RPGTMG__SHIFT = 0,
		DISP8M_APB_MAIN_LRPGCTL__RPGTMG__MASK = 1,
			RPGTMG_SHADOW_SOF = 0,
			RPGTMG_SHADOW_VSYNC = 1,

	DISP8M_APB_MAIN_LIDT_OFS	= DISP8M_APB_MAIN_OFS + 0x200,
	DISP8M_APB_MAIN_LISIZE_OFS	= DISP8M_APB_MAIN_OFS + 0x204,
	DISP8M_APB_MAIN_LYSA0_OFS	= DISP8M_APB_MAIN_OFS + 0x210,
	DISP8M_APB_MAIN_LYSA1_OFS	= DISP8M_APB_MAIN_OFS + 0x214,
	DISP8M_APB_MAIN_LYSA2_OFS	= DISP8M_APB_MAIN_OFS + 0x218,
	DISP8M_APB_MAIN_LYSA3_OFS	= DISP8M_APB_MAIN_OFS + 0x21c,
	DISP8M_APB_MAIN_LCSA0_OFS	= DISP8M_APB_MAIN_OFS + 0x220,
	DISP8M_APB_MAIN_LCSA1_OFS	= DISP8M_APB_MAIN_OFS + 0x224,
	DISP8M_APB_MAIN_LCSA2_OFS	= DISP8M_APB_MAIN_OFS + 0x228,
	DISP8M_APB_MAIN_LCSA3_OFS	= DISP8M_APB_MAIN_OFS + 0x22c,

	DISP8M_APB_MAIN_LYHGA_OFS	= DISP8M_APB_MAIN_OFS + 0x240,
	DISP8M_APB_MAIN_LCHGA_OFS	= DISP8M_APB_MAIN_OFS + 0x250,

	DISP8M_APB_MAIN_LIBCTL_OFS	= DISP8M_APB_MAIN_OFS + 0x260,
	DISP8M_APB_MAIN_LERCV_OFS	= DISP8M_APB_MAIN_OFS + 0x264,

	DISP8M_APB_MAIN_LHRSZ0_OFS	= DISP8M_APB_MAIN_OFS + 0x300,
	DISP8M_APB_MAIN_LHRSZ1_OFS	= DISP8M_APB_MAIN_OFS + 0x304,
	DISP8M_APB_MAIN_LVRSZ_OFS	= DISP8M_APB_MAIN_OFS + 0x308,

	DISP8M_APB_MAIN_LYWTH_OFS	= DISP8M_APB_MAIN_OFS + 0x310,
	DISP8M_APB_MAIN_LYWHS0_OFS	= DISP8M_APB_MAIN_OFS + 0x318,
	DISP8M_APB_MAIN_LYWHS1_OFS	= DISP8M_APB_MAIN_OFS + 0x31c,
	DISP8M_APB_MAIN_LYWLS0_OFS	= DISP8M_APB_MAIN_OFS + 0x320,
	DISP8M_APB_MAIN_LYWLS1_OFS	= DISP8M_APB_MAIN_OFS + 0x324,
	DISP8M_APB_MAIN_LBLTMR_OFS	= DISP8M_APB_MAIN_OFS + 0x328,

	DISP8M_APB_MAIN_LDRECTL_OFS	= DISP8M_APB_MAIN_OFS + 0x330,
	DISP8M_APB_MAIN_LDREYCAL_OFS	= DISP8M_APB_MAIN_OFS + 0x334,
	DISP8M_APB_MAIN_LDRECBCAL_OFS	= DISP8M_APB_MAIN_OFS + 0x338,
	DISP8M_APB_MAIN_LDRECRCAL_OFS	= DISP8M_APB_MAIN_OFS + 0x33c,
	DISP8M_APB_MAIN_LY2R0_OFS	= DISP8M_APB_MAIN_OFS + 0x340,
	DISP8M_APB_MAIN_LY2R1_OFS	= DISP8M_APB_MAIN_OFS + 0x344,
	DISP8M_APB_MAIN_LY2R2_OFS	= DISP8M_APB_MAIN_OFS + 0x348,

	DISP8M_APB_MAIN_LTBLASET_OFS	= DISP8M_APB_MAIN_OFS + 0x350,

	DISP8M_APB_MAIN_LCC0_OFS	= DISP8M_APB_MAIN_OFS + 0x360,
	DISP8M_APB_MAIN_LCC1_OFS	= DISP8M_APB_MAIN_OFS + 0x364,
	DISP8M_APB_MAIN_LCC2_OFS	= DISP8M_APB_MAIN_OFS + 0x368,

	DISP8M_APB_MAIN_LTCYCAL_OFS	= DISP8M_APB_MAIN_OFS + 0x370,
	DISP8M_APB_MAIN_LTCCTL_OFS	= DISP8M_APB_MAIN_OFS + 0x374,
	DISP8M_APB_MAIN_LDISPEN_OFS	= DISP8M_APB_MAIN_OFS + 0x380,

	DISP8M_APB_MAIN_LDSTA_OFS	= DISP8M_APB_MAIN_OFS + 0x400,
	DISP8M_APB_MAIN_LREVDISP_OFS	= DISP8M_APB_MAIN_OFS + 0x404,
};

enum disp8m_apb_out_reg {
	DISP8M_APB_OUT_RESET_OFS	= DISP8M_APB_OUT_OFS + 0,
	DISP8M_APB_OUT_IFS_OFS		= DISP8M_APB_OUT_OFS + 0x100,
	DISP8M_APB_OUT_TRG_OFS		= DISP8M_APB_OUT_OFS + 0x200,
	DISP8M_APB_OUT_TOCTL_OFS	= DISP8M_APB_OUT_OFS + 0x204,
	DISP8M_APB_OUT_INTC_OFS		= DISP8M_APB_OUT_OFS + 0x208,
	DISP8M_APB_OUT_INTE_OFS		= DISP8M_APB_OUT_OFS + 0x210,
	DISP8M_APB_OUT_INTF_OFS		= DISP8M_APB_OUT_OFS + 0x214,
	DISP8M_APB_OUT_AXISTS_OFS	= DISP8M_APB_OUT_OFS + 0x218,
	DISP8M_APB_OUT_RPGCTL_OFS	= DISP8M_APB_OUT_OFS + 0x220,
	DISP8M_APB_OUT_RPGEN_OFS	= DISP8M_APB_OUT_OFS + 0x224,

	DISP8M_APB_OUT_POLSEL_OFS	= DISP8M_APB_OUT_OFS + 0x300,
	DISP8M_APB_OUT_TSL_OFS		= DISP8M_APB_OUT_OFS + 0x304,
	DISP8M_APB_OUT_VCYC_OFS		= DISP8M_APB_OUT_OFS + 0x308,
	DISP8M_APB_OUT_HCYC_OFS		= DISP8M_APB_OUT_OFS + 0x30c,
	DISP8M_APB_OUT_OVPW_OFS		= DISP8M_APB_OUT_OFS + 0x310,
		DISP8M_APB_OUT_OVPW__OVPWU__SHIFT = 16,
		DISP8M_APB_OUT_OVPW__OVPWU__MASK = 1,



	DISP8M_APB_OUT_HPW_OFS		= DISP8M_APB_OUT_OFS + 0x314,
	DISP8M_APB_OUT_VBLK_OFS		= DISP8M_APB_OUT_OFS + 0x318,
	DISP8M_APB_OUT_HBLK_OFS		= DISP8M_APB_OUT_OFS + 0x31c,
	DISP8M_APB_OUT_VDLY_OFS		= DISP8M_APB_OUT_OFS + 0x320,
	DISP8M_APB_OUT_HDLY_OFS		= DISP8M_APB_OUT_OFS + 0x324,
	DISP8M_APB_OUT_OVSIZE_OFS	= DISP8M_APB_OUT_OFS + 0x328,
	DISP8M_APB_OUT_OHSIZE_OFS	= DISP8M_APB_OUT_OFS + 0x32c,
	DISP8M_APB_OUT_VRFCTL_OFS	= DISP8M_APB_OUT_OFS + 0x330,
	DISP8M_APB_OUT_HRFCTL_OFS	= DISP8M_APB_OUT_OFS + 0x338,

	DISP8M_APB_OUT_DOMD_OFS		= DISP8M_APB_OUT_OFS + 0x400,
	DISP8M_APB_OUT_FDOEN_OFS	= DISP8M_APB_OUT_OFS + 0x410,
	DISP8M_APB_OUT_FODATA_OFS	= DISP8M_APB_OUT_OFS + 0x414,

	DISP8M_APB_OUT_BLANKDT_OFS	= DISP8M_APB_OUT_OFS + 0x420,
	DISP8M_APB_OUT_CLBHSIZE_OFS	= DISP8M_APB_OUT_OFS + 0x430,
	DISP8M_APB_OUT_CLBDT_OFS	= DISP8M_APB_OUT_OFS + 0x440,

	DISP8M_APB_OUT_BLDCTL_OFS	= DISP8M_APB_OUT_OFS + 0x480,
	DISP8M_APB_OUT_R2RCTL_OFS	= DISP8M_APB_OUT_OFS + 0x488,
	DISP8M_APB_OUT_R2Y0_OFS		= DISP8M_APB_OUT_OFS + 0x490,
	DISP8M_APB_OUT_R2Y1_OFS		= DISP8M_APB_OUT_OFS + 0x494,
	DISP8M_APB_OUT_R2Y2_OFS		= DISP8M_APB_OUT_OFS + 0x498,

	DISP8M_APB_OUT_YCTL_OFS		= DISP8M_APB_OUT_OFS + 0x49c,
	DISP8M_APB_OUT_YCAL_OFS		= DISP8M_APB_OUT_OFS + 0x4a0,
	DISP8M_APB_OUT_YCLIP_OFS	= DISP8M_APB_OUT_OFS + 0x4a4,
	DISP8M_APB_OUT_CBCAL_OFS	= DISP8M_APB_OUT_OFS + 0x4a8,
	DISP8M_APB_OUT_CBCLIP_OFS	= DISP8M_APB_OUT_OFS + 0x4ac,
	DISP8M_APB_OUT_CRCAL_OFS	= DISP8M_APB_OUT_OFS + 0x4b0,
	DISP8M_APB_OUT_CRCLIP_OFS	= DISP8M_APB_OUT_OFS + 0x4b4,

	DISP8M_APB_OUT_DOCTL0_OFS	= DISP8M_APB_OUT_OFS + 0x4c0,
	DISP8M_APB_OUT_DOCTL1_OFS	= DISP8M_APB_OUT_OFS + 0x4c4,
	DISP8M_APB_OUT_DOCTL2_OFS	= DISP8M_APB_OUT_OFS + 0x4c8,

	DISP8M_APB_OUT_TRSCODE0_OFS	= DISP8M_APB_OUT_OFS + 0x4d0,
	DISP8M_APB_OUT_TRSCODE1_OFS	= DISP8M_APB_OUT_OFS + 0x4d4,

	DISP8M_APB_OUT_GHDSTA_OFS	= DISP8M_APB_OUT_OFS + 0x500,
	DISP8M_APB_OUT_GVDSTA_OFS	= DISP8M_APB_OUT_OFS + 0x504,
	DISP8M_APB_OUT_GLENGTH_OFS	= DISP8M_APB_OUT_OFS + 0x508,
	DISP8M_APB_OUT_GWIDTH_OFS	= DISP8M_APB_OUT_OFS + 0x50c,
	DISP8M_APB_OUT_GITVL_OFS	= DISP8M_APB_OUT_OFS + 0x510,
	DISP8M_APB_OUT_GNUM_OFS		= DISP8M_APB_OUT_OFS + 0x514,
	DISP8M_APB_OUT_GDCTL_OFS	= DISP8M_APB_OUT_OFS + 0x518,

	DISP8M_APB_OUT_GDISPEN_OFS	= DISP8M_APB_OUT_OFS + 0x520,

	DISP8M_APB_OUT_FFDSTA_OFS	= DISP8M_APB_OUT_OFS + 0x600,
	DISP8M_APB_OUT_FFSIZE_OFS	= DISP8M_APB_OUT_OFS + 0x640,
	DISP8M_APB_OUT_FFWIDTH_OFS	= DISP8M_APB_OUT_OFS + 0x680,
	DISP8M_APB_OUT_FFCLR_OFS	= DISP8M_APB_OUT_OFS + 0x6c0,
	DISP8M_APB_OUT_FFDISPEN_OFS	= DISP8M_APB_OUT_OFS + 0x700,
};

#define DISP8M_APB_OSD1_OFFSET	(DISP8M_APB_OSD1_OFS - DISP8M_APB_OSD0_OFS)
#define DISP8M_AHB_OSD1_OFFSET	(DISP8M_AHB_OSD1_OFS - DISP8M_AHB_OSD0_OFS)

enum disp8m_apb_osd0_reg {
	DISP8M_APB_OSD_GRRST_OFS	= DISP8M_APB_OSD0_OFS + 0,
	DISP8M_APB_OSD_GRTRG_OFS	= DISP8M_APB_OSD0_OFS + 0x100,
	DISP8M_APB_OSD_GRRPGCTL_OFS	= DISP8M_APB_OSD0_OFS + 0x110,
	DISP8M_APB_OSD_GRIDT_OFS	= DISP8M_APB_OSD0_OFS + 0x200,
	DISP8M_APB_OSD_GRTISIZE_OFS	= DISP8M_APB_OSD0_OFS + 0x204,
	DISP8M_APB_OSD_GRTDSTA_OFS	= DISP8M_APB_OSD0_OFS + 0x208,
	DISP8M_APB_OSD_GRIPO_OFS	= DISP8M_APB_OSD0_OFS + 0x210,
	DISP8M_APB_OSD_GRSCCTL_OFS	= DISP8M_APB_OSD0_OFS + 0x214,
	DISP8M_APB_OSD_GRERCV_OFS	= DISP8M_APB_OSD0_OFS + 0x218,
	DISP8M_APB_OSD_GRISIZE_OFS	= DISP8M_APB_OSD0_OFS + 0x400,
	DISP8M_APB_OSD_GRSAO_OFS	= DISP8M_APB_OSD0_OFS + 0x440,
	DISP8M_APB_OSD_GRSA_OFS		= DISP8M_APB_OSD0_OFS + 0x480,
	DISP8M_APB_OSD_GRHGA_OFS	= DISP8M_APB_OSD0_OFS + 0x4c0,
	DISP8M_APB_OSD_GRDSTA_OFS	= DISP8M_APB_OSD0_OFS + 0x500,
	DISP8M_APB_OSD_GRAREN_OFS	= DISP8M_APB_OSD0_OFS + 0x540,
	DISP8M_APB_OSD_GRBSL_OFS	= DISP8M_APB_OSD0_OFS + 0x544,
	DISP8M_APB_OSD_GRBLINK_OFS	= DISP8M_APB_OSD0_OFS + 0x560,
	DISP8M_APB_OSD_GRHRSZ0_OFS	= DISP8M_APB_OSD0_OFS + 0x610,
	DISP8M_APB_OSD_GRHRSZ1_OFS	= DISP8M_APB_OSD0_OFS + 0x614,
	DISP8M_APB_OSD_GRVRSZ_OFS	= DISP8M_APB_OSD0_OFS + 0x618,
	DISP8M_APB_OSD_GRR2Y0_OFS	= DISP8M_APB_OSD0_OFS + 0x620,
	DISP8M_APB_OSD_GRR2Y1_OFS	= DISP8M_APB_OSD0_OFS + 0x624,
	DISP8M_APB_OSD_GRR2Y2_OFS	= DISP8M_APB_OSD0_OFS + 0x628,
	DISP8M_APB_OSD_GRDRECTL_OFS	= DISP8M_APB_OSD0_OFS + 0x630,
	DISP8M_APB_OSD_GRDREYCAL_OFS	= DISP8M_APB_OSD0_OFS + 0x634,
	DISP8M_APB_OSD_GRDRECBCAL_OFS	= DISP8M_APB_OSD0_OFS + 0x638,
	DISP8M_APB_OSD_GRDRECRCAL_OFS	= DISP8M_APB_OSD0_OFS + 0x63c,
	DISP8M_APB_OSD_GRY2R0_OFS	= DISP8M_APB_OSD0_OFS + 0x640,
	DISP8M_APB_OSD_GRY2R1_OFS	= DISP8M_APB_OSD0_OFS + 0x644,
	DISP8M_APB_OSD_GRY2R2_OFS	= DISP8M_APB_OSD0_OFS + 0x648,
	DISP8M_APB_OSD_GRTBLASET_OFS	= DISP8M_APB_OSD0_OFS + 0x650,
	DISP8M_APB_OSD_GRCC0_OFS	= DISP8M_APB_OSD0_OFS + 0x660,
	DISP8M_APB_OSD_GRCC1_OFS	= DISP8M_APB_OSD0_OFS + 0x664,
	DISP8M_APB_OSD_GRCC2_OFS	= DISP8M_APB_OSD0_OFS + 0x668,
	DISP8M_APB_OSD_GRTCYCAL_OFS	= DISP8M_APB_OSD0_OFS + 0x670,
	DISP8M_APB_OSD_GRTCCTL_OFS	= DISP8M_APB_OSD0_OFS + 0x674,
	DISP8M_APB_OSD_GRALP_OFS	= DISP8M_APB_OSD0_OFS + 0x680,
};

/*
 * Layout of AHB tables
 */

enum disp8m_ahb_map {
	DISP8M_AHB_IGTBLR_OFS		= 0,		/* 512 bytes */
	DISP8M_AHB_IGTBLG_OFS		= 0x200,	/* 512 bytes */
	DISP8M_AHB_IGTBLB_OFS		= 0x400,	/* 512 bytes */
	DISP8M_AHB_TCTBL_OFS		= 0x1000,	/* 512 bytes */
	DISP8M_AHB_TCEP_OFS		= 0x1200,	/*   8 bytes */
	DISP8M_AHB_DGTBLFR_OFS		= 0x2000,	/* 256 bytes */
	DISP8M_AHB_DGTBLFG_OFS		= 0x2100,	/* 256 bytes */
	DISP8M_AHB_DGTBLGB_OFS		= 0x2200,	/* 256 bytes */
	DISP8M_AHB_DGTBLDR_OFS		= 0x2400,	/*  1K bytes */
	DISP8M_AHB_DGTBLDG_OFS		= 0x2800,	/*  1K bytes */
	DISP8M_AHB_DGTBLDB_OFS		= 0x2c00,	/*  1K bytes */
};

//	IRIS_GBLC_LOCK_UNLOCK__LOCK_UNLOCK__SHIFT = 0,
//	IRIS_GBLC_LOCK_UNLOCK__LOCK_UNLOCK__MASK = 0xffffffff,
#endif

