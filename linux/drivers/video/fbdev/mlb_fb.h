#ifndef MLB_FB_H
#define MLB_FB_H
#include <linux/mlb_fb_user.h>
typedef enum {
	GRTISIZE = 1,
	GRTDSTA,
	GRSCCTL,
	GRRSZ1,
	GRRSZ2,
	GRISIZE,
	GRDSTA,
	GRHGA,
	GRSA0,
	GRSA,
	GRAREN,
	GRTRG,
	GRIDT,
	GR_CODE_END
} REQUEST_GR_CODE;
union dcore_ifs {
	unsigned long	word;
	struct {
		unsigned long	ifs	:2;
		unsigned long		:30;
	} bit;
};
union core_trg {
	unsigned long       word;
	struct {
		unsigned long   trg     :2;
		unsigned long           :2;
		unsigned long   tgkmd   :1;
		unsigned long           :27;
	} bit;
};
/*  structure of OVSIZE (2890_1328h)    */
union dcore_ovsize {
	unsigned long       word;
	struct {
		unsigned long   ovsize  :14;
		unsigned long           :18;
	} bit;
};

/*  structure of OHSIZE (2890_132Ch)    */
union dcore_ohsize {
	unsigned long       word;
	struct {
		unsigned long   ohsize  :16;
		unsigned long           :16;
	} bit;
};
struct save_gr_reg {
	/* Dcore regsister */
	union core_trg dcore_trig;	/* dcore's triger */
	union dcore_ifs ifs;/* dcore's IFS */
	u32 dcore_control1[4];/* From TGKST to INTE */
	u32 dcore_control2[2];/* From RPGCTL to RPGEN */
	u32 dcore_timing[13];/* From POLSEL to VRFCTL */
	u32 dcore_hrfctl;	/* HRFCTL */
	u32 dcore_hablk;	/* HABLK */
	u32 dcore_domd;		/* DOMD */
	u32 dcore_force_data[2];	/* FDOEN and FODATA */
	u32 dcore_blankdt[2];	/* BLANKDT */
	u32 dcore_clbhsize;	/* CLBHSIZE */
	u32 dcore_clbdt[17];	/* CLBDT and BLDCTL */
	u64 dcore_r2y[3];	/* R2Y */
	u64 dcore_ycal;	/* YCAL */
	u32	dcore_clip_cal[5];	/* From YCLIP to CRCLIP */
	u32 dcore_doctl[3];	/* From DOCTL0 to DOCTL2 */
	u32	dcore_trscode[2];	/* TRSCODE0/1	*/
	u64 dcore_y2r[3];	/* Y2R */
	u32 dcore_tblaset;	/* TBLASET */
	u32	dcore_grid[7];	/* Grid */
	u32	dcore_gdispen;	/* GDISPEN */
	u32 dcore_facesta[42];	/* FFDSTA */
	u32 dcore_facesize[42];	/* FFSIZE */
	u32 dcore_facewidth[42];	/* FFWIDTH */
	u32 dcore_faceclr[42];	/* FFCLR */
	u64	dcore_ffdispen;	/* FFDISPEN */
	u32	dcore_ffdo;	/* FFDO */

	/* GR */
	u32 gr_enable;
	u32 ipo;
	u32 scctl;
	u32 rsz0;
	struct fb_disp disp;
};
/*  structure of GRRSZ0 (2890_2810h)    */
struct fb_mlb_priv {
	void __iomem *gr_reg_base;
	void __iomem *dcore_reg_base;
	u16 block_no;/*DISP's block No. 0:HDMI. 1:LCD */
	u16 gr_no;
	struct fb_disp *disp;
	u32 disp_num;/* disp's pattern num */
	u32 current_pattern;
	/* Save GR's register for resume before suspend */
	struct save_gr_reg suspend_reg;

};
union request_command_head {
	unsigned long       word;
	struct {
		unsigned short  request_num;
		unsigned short  gr_no;
	} bit;
};
union request_head {
	unsigned long       word;
	struct {
		unsigned short  request_size;
		unsigned short  request_code;
	} bit;
};
#endif
