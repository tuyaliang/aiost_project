/*
 * driver/video/fbdev/mlb_ipcu_fb.c
 * Copyright:	(C) 2017-2019 Socionext
 * Copyright:	(C) 2017 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/delay.h>
/* for FB */
#include <linux/fb.h>
#include "mlb_fb_disp.h"

#include <video/fdb.h>
#include "mlb_fb.h"
#include <linux/mlb_fb_user.h>
#include <uapi/linux/sni_ipcu_parts.h>
#include <uapi/linux/ipcu_userland.h>
/* #define TEST_DISP */
/* #define TEST_IPCU */
#define DISP_DCORE_RESET_OFFSET (0x00)
#define DISP_DCORE_IFS_OFFSET (0x100)
#define DISP_DCORE_TRIG_OFFSET (0x200)
#define DISP_DCORE_TGKST_OFFSET (0x204)
#define DISP_DCORE_TOCTL_OFFSET (0x208)
#define DISP_DCORE_INTC_OFFSET (0x20C)
#define DISP_DCORE_INTE_OFFSET (0x210)
#define DISP_DCORE_INTF_OFFSET (0x214)
#define DISP_DCORE_RPGCTL_OFFSET (0x220)
#define DISP_DCORE_RPGEN_OFFSET (0x224)
#define DISP_DCORE_POLSEL_OFFSET (0x300)
#define DISP_DCORE_TSL_OFFSET (0x304)
#define DISP_DCORE_VCYC_OFFSET (0x308)
#define DISP_DCORE_HCYC_OFFSET (0x30c)
#define DISP_DCORE_OVPW_OFFSET (0x310)
#define DISP_DCORE_HPW_OFFSET (0x314)
#define DISP_DCORE_VBLK_OFFSET (0x318)
#define DISP_DCORE_HBLB_OFFSET (0x31C)
#define DISP_DCORE_VDLY_OFFSET (0x320)
#define DISP_DCORE_HDLY_OFFSET (0x324)
#define DISP_DCORE_OVSIZE_OFFSET (0x328)
#define DISP_DCORE_OHSIZE_OFFSET (0x32C)
#define DISP_DCORE_VRFCTL_OFFSET (0x330)
#define DISP_DCORE_HRFCTL_OFFSET (0x338)
#define DISP_DCORE_HABLK_OFFSET (0x340)
#define DISP_DCORE_DOMD_OFFSET (0x400)
#define DISP_DCORE_FDOEN_OFFSET (0x410)
#define DISP_DCORE_FODATA_OFFSET (0x414)
#define DISP_DCORE_BLANKDT_OFFSET (0x420)
#define DISP_DCORE_CLBHSIZE_OFFSET (0x430)
#define DISP_DCORE_CLBDT_OFFSET (0x440)
#define DISP_DCORE_BLDCTL_OFFSET (0x480)
#define DISP_DCORE_R2Y_OFFSET (0x490)
#define DISP_DCORE_YCAL_OFFSET (0x4B0)
#define DISP_DCORE_YCLIP_OFFSET (0x4B8)
#define DISP_DCORE_CBCAL_OFFSET (0x4BC)
#define DISP_DCORE_CBCLIP_OFFSET (0x4C0)
#define DISP_DCORE_CRCAL_OFFSET (0x4C4)
#define DISP_DCORE_CRCLIP_OFFSET (0x4C8)
#define DISP_DCORE_DOCT0_OFFSET (0x4D0)
#define DISP_DCORE_DOCT1_OFFSET (0x4D4)
#define DISP_DCORE_DOCT2_OFFSET (0x4D8)
#define DISP_DCORE_TRSCODE_OFFSET (0x4E0)
#define DISP_DCORE_TRSCODE1_OFFSET (0x4E4)
#define DISP_DCORE_Y2R_OFFSET (0x4F0)
#define DISP_DCORE_TBLASET_OFFSET (0x510)
#define DISP_DCORE_GRID_OFFSET (0x600)
#define DISP_DCORE_GDISPEN_OFFSET (0x620)
#define DISP_DCORE_FACEPOS_OFFSET (0x800)
#define DISP_DCORE_FACEPOS_SIZE (41)
#define DISP_DCORE_FACESIZE_OFFSET (0x900)
#define DISP_DCORE_FACESIZE_SIZE (41)
#define DISP_DCORE_FACEWIDTH_OFFSET (0xA00)
#define DISP_DCORE_FACESIZE_SIZE (41)
#define DISP_DCORE_FACEFCLR_OFFSET (0xB00)
#define DISP_DCORE_FACEFCLR_SIZE (41)
#define DISP_DCORE_FFDISPEN_OFFSET (0xC00)
#define DISP_DCORE_FFDO_OFFSET (0xC08)

#define DISP_GRRST_OFFSET (0)
#define DISP_GRTRG_OFFSET (0x100)
#define DISP_GRIDT_OFFSET (0x200)
#define DISP_GRTISIZE_OFFSET (0x204)
#define DISP_GRTDSTA_OFFSET (0x208)
#define DISP_GRIPO_OFFSET (0x210)
#define DISP_GRSCCTL_OFFSET (0x214)
#define DISP_GRISIZE_OFFSET (0x400)

#define DISP_GRSA0_OFFSET (0x440)
#define DISP_GRSA_OFFSET (0x480)
#define DISP_GRASA_OFFSET (0x4C0)
#define DISP_GRHGA_OFFSET (0x500)
#define DISP_GRAHGA_OFFSET (0x540)
#define DISP_GRDSTA_OFFSET (0x580)
#define DISP_GRAREN_OFFSET (0x5C0)
#define DISP_GRRSZ0_OFFSET (0x810)
#define DISP_GRRSZ1_OFFSET (0x814)
#define DISP_GRRSZ2_OFFSET (0x818)
#define DISP_GRRSZ3_OFFSET (0x81c)

#define IFMT_RGBA8888	(0) /* IFMT : RGBA8888 */
#define IFMT_RGBA4444	(2) /* IFMT : RGBA4444 */


#define RETRY_OPEN_IPCU (10) /* Max of retry open IPCU */

#define INITIALIZED	(1)
#define UNINITIALIZED	(0)
#define DEVICE_NAME "sni-gr"
static struct fb_info fb_mlb_info[4];
const char AREA_NAME[10][7] = {
"area00", "area01", "area02", "area03", "area04",
"area05", "area06", "area07", "area08", "area09"
};
typedef enum {
	BLOCK_HDMI = 0,
	BLOCK_LCD,
	BLCOK_END,
} ENUM_BLCOK;

typedef enum {
	BLOCK_NONE = 0,
	BLCOK_DONE_SUSPEND,
	BLOCK_DONE_RESUME,
} ENUM_BLOCK_SUS_RESUM_STATUS;

typedef enum {
	ENUM_IPCU_CH_SEND = 0,
	ENUM_IPCU_CH_RECEIVE,
	ENUM_IPCU_CH_END
} ENUM_IPCU_CH;
typedef enum {
	ENUM_IPCU_STATUS_NONE = 0,
	ENUM_IPCU_STATUS_INIT = 1,
	ENUM_IPCU_STATUS_OPEN = 2,
	ENUM_IPCU_STATUS_ALL = ENUM_IPCU_STATUS_INIT | ENUM_IPCU_STATUS_OPEN,
} ENUM_IPCU_STATUS;

/**
* Main description for T_POS.<br>
* It's as coordinate.
*/
typedef struct {
	int StartX;	/**< Coordinate of X	*/
	int StartY;	/**< Coordinate of Y	*/
} T_POS;

typedef struct {
	void __iomem *ipcu_command_buffer;
	u32 ipcu_command_buffer_ph;
	u32 ipcu_command_buffer_ph_len;
	u32 ipcu_unit;
	u32	ipcu_ch[ENUM_IPCU_CH_END];/* 0:send 1:receive */
	struct mutex mlock;
	struct work_struct work;
	struct workqueue_struct *ipcu_wq;
	struct sni_ipcu_device ipcu_dev[ENUM_IPCU_CH_END];
	ENUM_IPCU_STATUS status[ENUM_IPCU_CH_END];
	t_ipcu_mail_box mail_box_data[ENUM_IPCU_CH_END];
	int initialize_flag;
} ipcu_communicate;

const struct fb_bitfield argb8888[4] = {
	{.offset = 16, .length = 8, .msb_right = 0},
	{.offset = 8, .length = 8, .msb_right = 0},
	{.offset = 0, .length = 8, .msb_right = 0},
	{.offset = 24, .length = 8, .msb_right = 0}
};
const struct fb_bitfield argb4444[4] = {
	{.offset = 8, .length = 4, .msb_right = 0},
	{.offset = 4, .length = 4, .msb_right = 0},
	{.offset = 0, .length = 4, .msb_right = 0},
	{.offset = 12, .length = 4, .msb_right = 0}
};

static ipcu_communicate gr_ipcu = {
	.ipcu_command_buffer = NULL,
	.status = {ENUM_IPCU_STATUS_NONE, ENUM_IPCU_STATUS_NONE},
	.mail_box_data = { {0, 0}, {0, 0} },
	.initialize_flag = UNINITIALIZED, /* Set uninit flag */
};

static ENUM_BLOCK_SUS_RESUM_STATUS block_suspend_resume_status[BLCOK_END]
	= {BLOCK_NONE, BLOCK_NONE};
/**
* Main description for T_Rect.<br>
* It's a rectangle's coordinate and size.
*/
typedef struct {
	T_POS Postion;	/**< Rectangle's coordinate	*/
	int Width;	/**< Rectangle's Width		*/
	int Height;	/**< Rectangle's Height		*/
} T_Rect;
/**
* Local static fuction declear
*/
static int switch_draw_buffer(struct fb_info *info);
static int initialize_disp(const struct fb_info *info);
static u32 get_every_area_address(struct fb_info *fb_mlb_info,
			u32 one_pixel_byes);
static int get_every_pattern(struct device_node *node,
					const char *pattern_name,
					struct fb_info *info);

/**
* Local static function
*/
#ifndef TEST_DISP
/**
* End the IPCU
*/
static int end_ipcu(const u32 direction[ENUM_IPCU_CH_END])
{
	int i;
	int ret = 0;

	for (i = 0; i < ENUM_IPCU_CH_END; i++) {
		/* Initialize IPCU */
		if ((gr_ipcu.status[i] & ENUM_IPCU_STATUS_OPEN) ==
			ENUM_IPCU_STATUS_OPEN) {
			ret = sni_ipcu_closech(gr_ipcu.ipcu_unit,
				  gr_ipcu.ipcu_ch[i], direction[i]);
		}
		if ((gr_ipcu.status[i] & ENUM_IPCU_STATUS_INIT) ==
			ENUM_IPCU_STATUS_INIT) {
			sni_ipcu_ch_exit(gr_ipcu.ipcu_unit,
			      gr_ipcu.ipcu_ch[i], (void *)&gr_ipcu.ipcu_dev[i]);
		}
		gr_ipcu.status[i] = ENUM_IPCU_STATUS_NONE;
	}
	return ret;
}
/**
* Open ICPU's channel for sending/receiving
*/
static int open_ipcu(void)
{
	int i;
	u32 direction[ENUM_IPCU_CH_END] = {
		SNI_IPCU_DIR_SEND, SNI_IPCU_DIR_RECV
	};
	int ret = 0;

	for (i = 0; i < ENUM_IPCU_CH_END; i++) {
		/* Initialize IPCU */
		gr_ipcu.ipcu_dev[i].dest_unit =
			gr_ipcu.ipcu_unit;
		gr_ipcu.ipcu_dev[i].dest_channel =
			gr_ipcu.ipcu_ch[i];
		if ((gr_ipcu.status[i] & ENUM_IPCU_STATUS_INIT) == 0) {
			ret = sni_ipcu_ch_init(gr_ipcu.ipcu_unit,
				  gr_ipcu.ipcu_ch[i], direction[i],
					(void *)&gr_ipcu.ipcu_dev[i]);
			if (ret == 0) {
				gr_ipcu.status[i] |=
					ENUM_IPCU_STATUS_INIT;
			} else {
				pr_warn("%s:%d [WARN] cannot init ipcu channel.\n"
					, __func__,
				   __LINE__);
				break;
			}
		}
		if ((gr_ipcu.status[i] & ENUM_IPCU_STATUS_OPEN) == 0) {
			ret = sni_ipcu_opench(gr_ipcu.ipcu_unit,
				  gr_ipcu.ipcu_ch[i], direction[i]);
			if (ret == 0) {
				gr_ipcu.status[i] |= ENUM_IPCU_STATUS_OPEN;
			} else {
				pr_err("%s:%d [ERROR] cannot open ipcu channel.\n"
					, __func__,
				   __LINE__);
				break;
			}
		}

	}
	if (ret != 0) {
		end_ipcu(direction);
		/* Set probe do again */
		ret = -EPROBE_DEFER;
	}
	return ret;
}
/**
* Receive IPCU command response from RTOS
*/
static void ipcu_work_handler(struct work_struct *work)
{
	int ret = 0;
	u32 recv_buf[9];
#ifdef TEST_IPCU
	pr_err("%s %s %d\n", __FILE__, __func__, __LINE__);
#endif
	ret = sni_ipcu_recv_msg_kernel(gr_ipcu.ipcu_unit,
				       gr_ipcu.ipcu_ch[ENUM_IPCU_CH_RECEIVE],
				       recv_buf, sizeof(recv_buf),
						FLAG_RECV_WAIT);

#ifdef TEST_IPCU
	pr_err("%s %s %d,recv_buf[0]/[1]:%u,%x\n"
	, __FILE__, __func__, __LINE__
	, recv_buf[0], recv_buf[1]);
#endif
	if (ret != 0) {
		pr_err("%s:%d [ERROR] mlb_ipcu_recv_msg_kernel(): %d\n",
		       __func__, __LINE__, ret);
	} else {
		gr_ipcu.mail_box_data[ENUM_IPCU_CH_RECEIVE].sequence_no
			= recv_buf[0];
		gr_ipcu.mail_box_data[ENUM_IPCU_CH_RECEIVE].data
			= recv_buf[1];
	}
	ret = sni_ipcu_ack_send(gr_ipcu.ipcu_unit,
			gr_ipcu.ipcu_ch[ENUM_IPCU_CH_RECEIVE]);
	if (ret != 0) {
		pr_err("%s:%d [ERROR] mlb_ipcu_send_msg(): %d\n",
		__func__, __LINE__, ret);
	}
}
/**
*  Use IPCU to send a request to RTOS
*	p: Data send to RTOS
*	len: The data's length
*	return 0 if send OK
*/
static int ipcu_send_request(void *p, int len)
{
	int ret = 0;
	int i;

	memcpy_toio(gr_ipcu.ipcu_command_buffer, p, len);
	gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].sequence_no++;
	gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].data =
		gr_ipcu.ipcu_command_buffer_ph;

	wmb();/* Ensure sending data to RTOS by IPCU */

#ifdef TEST_IPCU
	pr_err("sequence_no:%d,data:%x\n",
		gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].sequence_no,
		gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].data);
	for (i = 0; i < len/4; i++) {
		u32 *pP = (u32 *)p;
	pr_err("[%d]:%x\n", i, pP[i]);
	}
	pr_err("mail_box_data:%x %x\n",
	gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].sequence_no,
	gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND].data);
#endif
	if (((gr_ipcu.status[0] & ENUM_IPCU_STATUS_OPEN) !=
			ENUM_IPCU_STATUS_OPEN)  ||
		((gr_ipcu.status[1] & ENUM_IPCU_STATUS_OPEN) !=
			ENUM_IPCU_STATUS_OPEN)) {
		for (i = 0; i < RETRY_OPEN_IPCU; i++) {
			ret = open_ipcu();
			if (ret == 0) {
				break;
			}
			/* Wait RTOS start-up */
			msleep(2);
		}
	}
	if (ret == 0) {
		queue_work(gr_ipcu.ipcu_wq, &gr_ipcu.work);
		ret = sni_ipcu_send_msg_kernel(gr_ipcu.ipcu_unit,
			   gr_ipcu.ipcu_ch[ENUM_IPCU_CH_SEND],
			   &gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND],
				sizeof(gr_ipcu.mail_box_data[ENUM_IPCU_CH_SEND]),
				 FLAG_SEND_NOTIFY);

		if (ret != 0) {
			pr_err("%s:%d [ERROR] mlb_ipcu_send_msg_kernel(): %d\n",
				   __func__, __LINE__, ret);
		} else {
			flush_workqueue(gr_ipcu.ipcu_wq);

			rmb(); /* Ensure gr_ipcu's received content*/
	#ifdef TEST_IPCU
			pr_err("%s %s %d sequence_no:%u,Data:%u\n", __FILE__, __func__, __LINE__,
			gr_ipcu.mail_box_data[ENUM_IPCU_CH_RECEIVE].sequence_no,
			gr_ipcu.mail_box_data[ENUM_IPCU_CH_RECEIVE].data
			);
	#endif
		}
	}
	return ret;
}
#endif
/**
*  Get greatest common divisor
*	u32 a:
*	u32 b:
*	return a and b's greatest common divisor
*/
static u32 get_greatest_common_divisor(u32 a,  u32 b)
{
	u32 ret;

	if (b == 0) {
		/* Cannot 0 division. */
		ret = 1;
	} else {
		while (1) {
			a %= b;
			if (a == 0) {
				ret = b;
				break;
			}
			b %= a;
			if (b == 0) {
				ret = a;
				break;
			}
		}
	}
	return ret;
}
/* Get a fraction of a and b */
static void general_fraction(u32 *a,  u32 *b)
{
	u32 greatest_common_divisor = get_greatest_common_divisor(*a, *b);
	*a /= greatest_common_divisor;
	*b /= greatest_common_divisor;
}

/** Check input format setting
* user_disp: The setting from user land.
* return: 0 will be return if the checking is OK
*/
static int check_ifmt(const struct fb_disp *user_disp)
{
	int ret = -EINVAL;

	switch (user_disp->iput_trans_setting.word) {
		/* IFMT can set only */
	case IFMT_RGBA8888:
	case IFMT_RGBA4444:
		ret = 0;
		break;
	default:
		pr_err("%s:%d [ERROR] : IFMT%lu\n",
				__func__, __LINE__, user_disp->iput_trans_setting.word);
		break;
	}
	return ret;
}
/**
* Check the GR's position which should be satisfied the DISP's stipulating
* user_disp: The setting from user land.
* return: 0 will be return if the checking is OK
*/
static int check_tdsta(const struct fb_disp *user_disp)
{
	int ret = -EINVAL;

	if (((user_disp->start_pos.bit.dsh & 1) == 0) &&
		((user_disp->start_pos.bit.dsv & 1) == 0)) {
		/* The position must be even */
		ret = 0;
	} else {
		pr_err("%s:%d [ERROR] : (dsh,dsv)=(%u,%u)\n",
			__func__, __LINE__,
			user_disp->start_pos.bit.dsh,
			user_disp->start_pos.bit.dsv);
	}
	return ret;
}
/** Check resize parameter of pich offset.
* Make the M/N to reduction of a fraction.
* resize: resize parameter set from user land.
* isize: Iput size. It will be resized by isize * M/N.
* return: 0 will be return if the checking is OK
*/
static int check_pich_offset2(union grrsz *resize, u32 isize)
{
	int ret = -EINVAL;
	u32 rszm = resize->bit.rszm;
	u32 rszn = resize->bit.rszn;
	u32 after_resize;

	/* Get fraction of resize rate */
	general_fraction(&rszm, &rszn);
	after_resize = isize * rszm / rszn;
	if ((isize * rszm - (after_resize - 1) * rszn
		 > resize->bit.rszof) &&
		rszm > resize->bit.rszof) {
		if ((2 * rszm > rszn) &&
			(rszm < rszn * 8)) {
			resize->bit.rszm = rszm;
			resize->bit.rszn = rszn;
			ret = 0;
		} else {
			pr_err("%s:%d [ERROR] : rszm/rszn=%u/%u\n",
				__func__, __LINE__, rszm, rszn);
		}
	} else {
		pr_err("%s:%d [ERROR] : rszm/rszn=%u/%u\n",
			__func__, __LINE__, rszm, rszn);
	}

	return ret;
}
/** Check resize parameter of pich offset.
* They should be satisfied the DISP's stipulating
* gr_rzsl: Resize method. 1: By linear. 0: Inflation.
* resize: resize parameter set from user land.
* isize: Iput size. It will be resized by isize * M/N.
* return: 0 will be return if the checking is OK
*/
static int check_pich_offset(u32 gr_rzsl, union grrsz *resize, u32 isize)
{
	int ret = -EINVAL;

	if (gr_rzsl == 0 ||
		(resize->bit.rszm ==
			resize->bit.rszn)) {
		if (resize->bit.rszof == 0)
			ret = check_pich_offset2(resize, isize);
	} else {
		ret = check_pich_offset2(resize, isize);
	}
	return ret;
}
/** Check GR's start position.
* It should be satisfied the DISP's stipulating
* start: Start position.
* isize: Iput size. It will be resized by isize * M/N.
* osize: Display's size.
* resize: resize parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_input_size(u32 start, u32 isize, u32 osize, union grrsz resize)
{
	int ret = -EINVAL;

	if (start + isize * resize.bit.rszm / resize.bit.rszn <= osize)
		ret = 0;
	else
		pr_err("%s:%d [ERROR] : input_size\n",
				__func__, __LINE__);

	return ret;
}
/** Check all size which want to set.
* They should be satisfied the DISP's stipulating
* info: FB information of the device.
* user_disp: parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_size(struct fb_info *info, struct fb_disp *user_disp)
{
	int ret = -EINVAL;
	union dcore_ovsize ovsize;
	union dcore_ohsize ohsize;
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	u32 gr_rzsl = 1; /* The resize must be linear */

	/* Check pich offset */
	if ((check_pich_offset(gr_rzsl,
		&user_disp->horizonal_resize,
		user_disp->input_area_size.bit.ihsize) == 0) &&
		(check_pich_offset(gr_rzsl,
		&user_disp->vertical_resize,
		user_disp->input_area_size.bit.ivsize) == 0)) {
		/* Check input size */
		if (fb_mlb_par->block_no == 1) {
				/* HDMI can't check because DCORE may be not set */
			ohsize.word = readl(fb_mlb_par->dcore_reg_base +
							DISP_DCORE_OHSIZE_OFFSET);
			ovsize.word = readl(fb_mlb_par->dcore_reg_base +
							DISP_DCORE_OVSIZE_OFFSET);
			if ((check_input_size(user_disp->start_pos.bit.dsh,
				user_disp->input_area_size.bit.ihsize,
				ohsize.bit.ohsize,
				user_disp->horizonal_resize) == 0) &&
				(check_input_size(user_disp->start_pos.bit.dsv,
				user_disp->input_area_size.bit.ivsize,
				ovsize.bit.ovsize,
				user_disp->vertical_resize) == 0)) {
				ret = 0;
			}
		} else {
			ret = 0;
		}
		
	} else {
		dev_err(info->dev, "Resize data error\n");
	}
	return ret;

}
/** Compare 2 rectangle's position.
* a's X < b'x then a<b.
* a's X = b'x and a's Y < b's Y then
* a<b
* a=b then return 0
* a<b then return -1
* a>b then return 1
*/
static int compare_rect(union grdsta a, union grdsta b)
{
	int ret = 0;

	if (a.bit.dsh < b.bit.dsh)
		ret = -1;
	else if (a.bit.dsh == b.bit.dsh)
		if (a.bit.dsv == b.bit.dsv)
			ret = 0;
		else if (a.bit.dsv < b.bit.dsv)
			ret = -1;
		else
			ret = 1;
	else
		ret = 1;
	return ret;
}
/** Check 2 drawing area so that ensure the draw-i < draw-i+1
* user_disp: parameter set from user land.
* index: chceked rect's index with others.
* return: 0 will be return if the checking is OK.
*/
static int check_2rect_order(const struct fb_disp *user_disp, int index)
{
	int j;
	int ret = 0;

	for (j = index + 1; j < MAX_DRAW_AREA_NUM; j++) {
		if (user_disp->enable_show_area[j]) {
			if (compare_rect(
				user_disp->input_image_position[index],
				user_disp->input_image_position[j])
				>= 0) {
				pr_err("%s %d error.\n", __func__, __LINE__);
				ret = -EINVAL;
				break;
			}
		}
	}
	return ret;
}
/** Check all drawing area so that ensure the draw-i < draw-i+1
* user_disp: parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_rect_order(const struct fb_disp *user_disp)
{
	int i;
	int ret = 0;

	for (i = 0; i < MAX_DRAW_AREA_NUM - 1; i++) {
		if (user_disp->enable_show_area[i]) {
			ret = check_2rect_order(user_disp, i);
			if (ret != 0) {
				break;
			}
		}
	}
	return 0;
}
/**
 *	@brief Get the overlap rectangle of 2 rectangles
 *	@param  rect1:Rectangle1
 *			rect2:Rectangle2
 *			overlap_rect:  overlap rect(O)
 *	@return FALSE not any part of overlap
 *	@note		None
 *	@attention	None
 */
static int get_overlap_rect(T_Rect rect1, T_Rect rect2, T_Rect *overlap_rect)
{
	int	overlap = 1;
	int	rect1_right_X  = rect1.Postion.StartX + rect1.Width - 1;
	int	rect2_right_X  = rect2.Postion.StartX + rect2.Width - 1;
	int	rect1_bottom_Y = rect1.Postion.StartY + rect1.Height - 1;
	int	rect2_bottom_Y = rect2.Postion.StartY + rect2.Height - 1;
	int	right, bottom;

	if ((rect1.Postion.StartX > rect2_right_X)  ||
		(rect1_right_X < rect2.Postion.StartX)  ||
		(rect1_bottom_Y < rect2.Postion.StartY) ||
		(rect1.Postion.StartY > rect2_bottom_Y)) {
		overlap = 0;
	} else {

		if ((rect1_right_X <= rect2_right_X) &&
			(rect1_right_X >= rect2.Postion.StartX)) {
			/*      +---------+
			*		|  rect2  |
			*		|         |
			*	--------+
			*	rect1	|
			*/
			right = rect1_right_X;
		} else {
			/*       +---------+
			 *       |  rect2  |
			 *       |         |
			 *  -----------------+
			 *   rect1           |
			*/
			right = rect2_right_X;
		}

		if ((rect1.Postion.StartX <= rect2_right_X) &&
			(rect1.Postion.StartX >= rect2.Postion.StartX)) {
			/*       +---------+
			 *       |  rect2  |
			 *       |         |
			 *          +---------
			 *          | rect1
			*/
			overlap_rect->Postion.StartX = rect1.Postion.StartX;
		} else {
			/*      +---------+
			*		|  rect2  |
			*		|         |
			*      +---------
			*      | rect1
			*/
			overlap_rect->Postion.StartX = rect2.Postion.StartX;
		}

		if ((rect1_bottom_Y <= rect2_bottom_Y) &&
			(rect1_bottom_Y >= rect2.Postion.StartY)) {
			/*      +--------+
			*		|   rect2|
			*   rect1| |      |
			*  --------+      |
			*          |
			*/
			bottom = rect1_bottom_Y;
		} else {
			/*        +--------+
			 *        |   rect2|
			 *        |        |
			 *  rect1 +-|------+
			 *  --------+
			*/
			bottom = rect2_bottom_Y;
		}

		if ((rect1.Postion.StartY <= rect2_bottom_Y) &&
			(rect1.Postion.StartY >= rect2.Postion.StartY)) {
			/*       +---------+
			 *       |  rect2  |
			 *       |         |
			 *          +---------
			 *          | rect1
			*/
			overlap_rect->Postion.StartY = rect1.Postion.StartY;
		} else {
			/*        +---------
			 *       +--|------+
			 *       |  | rect2|
			 *       |  |      |
			 *          | rect1
			*/
			overlap_rect->Postion.StartY = rect2.Postion.StartY;
		}
		overlap_rect->Width =
			right - overlap_rect->Postion.StartX + 1;
		overlap_rect->Height =
			bottom - overlap_rect->Postion.StartY + 1;
	}
	return overlap;
}
/** Check all drawing area if they are not overlap
* user_disp: parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_rect_overlap(const struct fb_disp *user_disp)
{
	T_Rect rect1, rect2, overlap_rect;
	int i, j;

	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if (user_disp->enable_show_area[i]) {
			rect1.Postion.StartX =
				user_disp->input_image_position[i].bit.dsh;
			rect1.Postion.StartY =
			user_disp->input_image_position[i].bit.dsv;
			rect1.Width =
				user_disp->input_image_size[i].bit.ihsize;
			rect1.Height =
				user_disp->input_image_size[i].bit.ivsize;
			for (j = i + 1; j < MAX_DRAW_AREA_NUM; j++) {
				if (user_disp->enable_show_area[j]) {
					rect2.Postion.StartX =
						user_disp->
						input_image_position[j].bit.dsh;
					rect2.Postion.StartY =
						user_disp->
						input_image_position[j].bit.dsv;
					rect2.Width =
						user_disp->
						input_image_size[j].bit.ihsize;
					rect2.Height =
						user_disp->
						input_image_size[j].bit.ivsize;
		/* check the 2 rectangle if they are overlap */
					if (get_overlap_rect(rect1,
						rect2, &overlap_rect) == 1) {
						pr_err("%s %d error. Rect[%d](%u,%u,%u,%u),Rect[%d](%u,%u,%u,%u)\n"
						, __func__, __LINE__, i,
						rect1.Postion.StartX,
						rect1.Postion.StartY,
						rect1.Width, rect1.Height, j,
						rect2.Postion.StartX,
						rect2.Postion.StartY,
						rect2.Width, rect2.Height);
						return -EINVAL;
					}
				}
			}
		}
	}
	return 0;
}
/** Check a drawing area's size.
* It should be satisfied the DISP's stipulating.
* user_disp: parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_draw_rect_val(
	union grisize input_area_size,
	union grdsta input_image_position,
	union grisize input_image_size,
	int block_no)
{
	u32 size_bit_mask;
	u32 min_pixel_num;
	int ret = 0;

	if (block_no == BLOCK_HDMI) {
		size_bit_mask = 0x07;/* Must be over 8 pixles */
		min_pixel_num = 16;
	} else {
		size_bit_mask = 0x03;/* Must be over 4 pixles */
		min_pixel_num = 8;
	}
	if (((input_image_position.bit.dsh & 1) != 0) ||
		((input_image_position.bit.dsv & 1) != 0) ||
		((input_image_size.bit.ihsize & size_bit_mask) != 0) ||
		(input_image_size.bit.ihsize < min_pixel_num) ||
		((input_image_size.bit.ivsize & 1) != 0) ||
		(input_image_size.bit.ivsize  < 8) ||
		((input_image_position.bit.dsh +
		input_image_size.bit.ihsize) > input_area_size.bit.ihsize) ||
		((input_image_position.bit.dsv +
		input_image_size.bit.ivsize) > input_area_size.bit.ivsize)) {
		ret = -EINVAL;
		pr_err("%s %d error. size_bit_mask:%x min_pixel_num:%x position:%lx,input_image_size:%lx,input_area_size:%lx):\n"
		, __func__, __LINE__,
		size_bit_mask,
		min_pixel_num,
		input_image_position.word,
		input_image_size.word,
		input_area_size.word
		);
	}
	return ret;
}
/** Check all drawing area's size.
* It should be satisfied the DISP's stipulating.
* user_disp: parameter set from user land.
* block_no : 0: HDMI 1:LCD
* return: 0 will be return if the checking is OK.
*/
static int check_draw_rect(const struct fb_disp *user_disp, int block_no)
{
	int ret = 0;
	int i;

	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if ((user_disp->enable_show_area[i]) &&
			(check_draw_rect_val(
			user_disp->input_area_size,
			user_disp->input_image_position[i],
			user_disp->input_image_size[i],
			block_no) != 0)) {
			ret = -EINVAL;
			pr_err("%s %d error. ret:%d):\n"
				, __func__, __LINE__, ret);
			break;
		}
	}
	return ret;
}
/**
* Check one address of drawing areas.
* user_disp: parameter set from user land.
* byte_bit_mask : bit mask
* index: Drawing area No.
* return: 0 will be return if the checking is OK.
*/
static int check_one_address_bytes(
		const struct fb_disp *user_disp,
		u32 byte_bit_mask, int index)
{
	int ret = 0, j;

	if ((user_disp->input_image_horizonal_bytes[index]
		& byte_bit_mask) != 0) {
		ret = -EINVAL;
		pr_err("%s %d error. ret:%d):\n"
		, __func__, __LINE__, ret);
	} else {
		for (j = 0; j < 4; j++) {
			if (
				(user_disp->
				input_image_ph_address[index].address[j]
				& byte_bit_mask) != 0) {
				pr_err("%s %d error.\n",
					__func__, __LINE__);
				ret = -EINVAL;
				break;
			} else if (j == 0 &&
				(user_disp->
				input_image_ph_address[index].address[j]
				== 0)) {
				ret = -EINVAL;
				pr_err("%s:%d [ERROR] : check_one_address[%d]\n",
				__func__, __LINE__, j);
				break;
			}
		}
	}
	return ret;
}
/** Check input address and horizonal bytes
* It should be satisfied the DISP's stipulating.
* user_disp: parameter set from user land.
* block_no : 0: HDMI 1:LCD
* return: 0 will be return if the checking is OK.
*/
static int check_add_bytes(const struct fb_disp *user_disp,
		int block_no)
{
	int ret = 0;
	int i;
	u32 byte_bit_mask;

	if (block_no == BLOCK_HDMI) {
		byte_bit_mask = 0x0F;/* Must be over 16 bytes */
	} else {
		byte_bit_mask = 0x07;/* Must be over 4 bytes */
	}

	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if (user_disp->enable_show_area[i]) {
			ret = check_one_address_bytes(user_disp,
				byte_bit_mask, i);
			if (ret != 0) {
				break;
			}
		}
	}
	return ret;
}
/**
* Check drawing area's control function.
* It should be satisfied the DISP's stipulating.
* user_disp: parameter set from user land.
* block_no : 0: HDMI 1:LCD
* return: 0 will be return if the checking is OK.
*/
static int check_draw_area(const struct fb_disp *user_disp, int block_no)
{
	int ret = -EINVAL;

	if ((check_rect_order(user_disp) == 0) &&
		(check_draw_rect(user_disp, block_no) == 0 &&
		(check_rect_overlap(user_disp) == 0)) &&
		(check_add_bytes(user_disp, block_no) == 0)) {
		ret = 0;
	}
	return ret;
}
/**
* Check fb_mlb_par's value so that decide they can be set to GR.
* info: FB's information.
* user_disp: The parameter set from user land.
* return: 0 will be return if the checking is OK.
*/
static int check_parameter(struct fb_info *info,
			struct fb_disp *user_disp)
{
	int ret = -EINVAL;
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;

	/* Check format */
	if ((check_ifmt(user_disp) == 0) &&
		(check_tdsta(user_disp) == 0) &&
		(check_size(info, user_disp) == 0) &&
		(check_draw_area(user_disp,
			fb_mlb_par->block_no) == 0)) {
		ret = 0;
	}
	return ret;
}
/**
* Set FB's fb_fix_screeninfo and fb_var_screeninfo from disp parameter.
* info: FB's information of the device.
* user_disp: parameter set from user-land or DTS.
* return: 0 will be return if the setting is successful.
*/
static int set_var_fix(struct fb_info *info,
	struct fb_disp *user_disp)
{
	u32 one_pixel_byes;

	info->var.xres =
		user_disp->input_area_size.bit.ihsize; /* horizonal size */
	info->var.yres =
		user_disp->input_area_size.bit.ivsize; /* vertiacal size */
	info->fix.smem_start = user_disp->drawing_buffer_top_address_size[0];

	info->screen_base = (char __iomem *)info->fix.smem_start;

	info->fix.smem_len = user_disp->drawing_buffer_top_address_size[1];

	info->fix.xpanstep = user_disp->pan_step[0]; /* Pan X step */
	info->fix.ypanstep = user_disp->pan_step[1]; /* Pan Y step */
	/* virtual horizonal size */
	info->var.xres_virtual =
		user_disp->res_virtual[0];
	/* virtual vertiacal size */
	info->var.yres_virtual =
			 user_disp->res_virtual[1];

	switch (user_disp->iput_trans_setting.bit.ifmt) {
	case 0:/* RGBA8888 */
		info->var.bits_per_pixel = 32;
		info->var.red = argb8888[0];
		info->var.green = argb8888[1];
		info->var.blue = argb8888[2];
		info->var.transp  = argb8888[3];
		break;
	case 2: /* RGBA4444 */
		info->var.bits_per_pixel = 16;
		info->var.red = argb4444[0];
		info->var.green = argb4444[1];
		info->var.blue = argb4444[2];
		info->var.transp  = argb4444[3];
		break;
	default:
		info->var.bits_per_pixel = 32;
		info->var.red = argb8888[0];
		info->var.green = argb8888[1];
		info->var.blue = argb8888[2];
		info->var.transp  = argb8888[3];
		break;
	}
	one_pixel_byes = info->var.bits_per_pixel / 8;
	info->fix.line_length =
		info->var.xres_virtual * one_pixel_byes;
	info->screen_size = (
				info->var.xres_virtual *
				info->var.yres_virtual * one_pixel_byes);
	if (info->screen_size *
		user_disp->valid_buffer_number
		> info->fix.smem_len) {
		pr_err("screen_size:%lu,smem_len:%d,valid_buffer_number:%d\n",
				info->screen_size,
				info->fix.smem_len,
				user_disp->valid_buffer_number);
		user_disp->valid_buffer_number =
			info->fix.smem_len / info->screen_size;
		pr_err("The draw_buffer_num is forced to set to %d as the drawing buffer size is not enough.\n",
		user_disp->valid_buffer_number);
	}
	return 0;
}
/**
* Set data to all drawing area by FBIOGET_DRAWAREA.
* info: FB's information of the device.
* user_disp: parameter set from user-land or DTS.
* return: 0 will be return if the setting is successful.
*/
static int set_drawing_area(struct fb_info *info,
			const draw_area *user_draw_area)
{
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	struct fb_disp user_disp;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int i, ret;

	user_disp = fb_mlb_par->disp[fb_mlb_par->current_pattern];
	memcpy(user_disp.input_image_size,
		user_draw_area->input_image_size,
		sizeof(user_disp.input_image_size));
	memcpy(user_disp.input_image_position,
		user_draw_area->input_image_position,
		sizeof(user_disp.input_image_position));
	memcpy(user_disp.enable_show_area,
		user_draw_area->enable_show_area,
		sizeof(user_disp.enable_show_area));
	memcpy(user_disp.input_image_horizonal_bytes,
		user_draw_area->input_image_horizonal_bytes,
		sizeof(user_disp.input_image_horizonal_bytes));

	ret = check_parameter(info, &user_disp);
	if (ret == 0) {
		*disp = user_disp;
		disp->valid_input_area = 0;
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->enable_show_area[i]) {
				disp->valid_input_area |= (1<<i);
			}
		}
		disp->current_buffer_index = 0;
		/* Get every areas drawing buffer address */
		get_every_area_address(info, info->var.bits_per_pixel / 8);
		set_var_fix(info, disp);
		ret = initialize_disp(info);
		if (ret != 0) {
			dev_err(info->dev,
				"DISP initialize fail. Setting area fail.\n");
		}
	}
	return ret;
}
/**
* Set all data for GR by FBIOFULL_SETTING.
* We don't suggest to use this fuction. Insteal of using it,
* we suggest to use FBIOSWITCH_GR_SETTING which switch setting from DTS.
* info: FB's information of the device.
* user_disp: parameter set from user-land or DTS.
* return: 0 will be return if the setting is successful.
*/
static int set_full_gr(struct fb_info *info,
		const struct fb_disp *user_disp)
{
	struct fb_mlb_priv *fb_mlb_par =
			(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int i, ret;
	struct fb_disp recovery_disp = *disp;/* for error recovery */

	*disp = *user_disp;
	disp->valid_input_area = 0;
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if (disp->enable_show_area[i]) {
			disp->valid_input_area |= (1 << i);
		}
	}
	disp->current_buffer_index = 0;
	for (i = 0; i < 4; i++) {
		if (disp->input_image_ph_address[0].address[i] == 0) {
			disp->valid_buffer_number = i;
			break;
		}
	}
	if (disp->gri_trg != 1) {
		disp->gri_trg = 0;
	}
	set_var_fix(info, disp);
	ret = initialize_disp(info);
	if (ret != 0) {
	/* recovery */
		*disp = recovery_disp;
		set_var_fix(info, disp);
		initialize_disp(info);
		if (ret != 0) {
			dev_err(info->dev,
			"DISP initialize fail. Full setting NG. GR setting was recovery\n");
		}
	}
	return ret;
}
#ifndef TEST_DISP
/**
* Initialize command of IPCU
* send_command: Set to RTOS's command
* block_no: Block No.
* gr_no: GR0/1
* set_command: Send comannd body
*/
static int initial_ipcu_command_head(t_ipcu_command *send_command,
			u32 block_no,
			u32 gr_no)
{
	memset(send_command, 0, sizeof(*send_command));
	send_command->head.bit.command_num = 0;
	send_command->head.bit.block_no
		= block_no;
	send_command->head.bit.gr_no
		= gr_no;
	send_command->command_offset[0] =
		offsetof(t_ipcu_command, buffer);

	return 0;
}
/**
* Set next command to IPCU command
* send_command: ICPU command
* command_code: command code sending
* command_size: command size
* command: sending comand
* return: sendind command szie
*/
static int set_next_command(
				t_ipcu_command *send_command,
				e_command_code command_code,
				int command_size, const u32 *command)
{
	int i;
	int current_index =
		send_command->head.bit.command_num;
	/*Get command's head top */
	int offset =
		send_command->command_offset[current_index];
	u32 command_address =
		(u32)send_command + offset;
	t_request *set_command =
		 (t_request *)(command_address);

	if (current_index < D_MAX_COMMAND_NUM) {
		set_command->command_head.bit.command_code =
			command_code;
		set_command->command_head.bit.size = command_size;
		for (i = 0; i < command_size; i++) {
			set_command->command[i] = command[i];
		}
		/* Get next command offset */
		offset = (u32)(&set_command->command[i])
				- (u32)send_command;
		if (offset >= sizeof(t_ipcu_command) ||
			offset >= gr_ipcu.ipcu_command_buffer_ph_len) {
			pr_err("%s:%d [Error] GR ICPU command buffer is not enough.used:%u.P buffer :%u,D:%u\n"
				, __func__, __LINE__,
			offset, sizeof(t_ipcu_command),
			gr_ipcu.ipcu_command_buffer_ph_len);
		}
		send_command->command_offset[current_index + 1] =
			offset;
		/* Get command num */
		send_command->head.bit.command_num++;
	} else {
		pr_err("%s:%d [Error] GR IPCU commands must less than %u.\n"
			, __func__,
		   __LINE__, D_MAX_COMMAND_NUM);
	}
	return offset;
}
#endif
/**
* Set GR's triger to on/off by FBIOSET_TRIGER.
* Usually, it is called after FBIOFULL_SETTING setting.
* triger: Triger to set to GR.
* return: 0 will be return if the setting is successful.
*/
static int set_gr_triger(struct fb_info *info, unsigned long triger)
{
	struct fb_mlb_priv *fb_mlb_par
		= (struct fb_mlb_priv *)info->par;
	int ret = 0;
#ifdef TEST_DISP
	u32 current_trig =
		readl(
			fb_mlb_par->
			gr_reg_base + DISP_GRTRG_OFFSET);

	if (current_trig == 2) {
		if (triger == 1) {
			writel(1,
				fb_mlb_par->
				gr_reg_base + DISP_GRTRG_OFFSET);
		} else {
			ret = -EPERM;
		}
	} else if (current_trig == 3) {
		if (triger != 1) {
			writel(triger,
				fb_mlb_par->
				gr_reg_base + DISP_GRTRG_OFFSET);
		} else {
			ret = -EPERM;
		}
	}
#else
	t_ipcu_command send_command;
	u32 current_trig =
		readl(fb_mlb_par->gr_reg_base
		+ DISP_GRTRG_OFFSET);

	initial_ipcu_command_head(&send_command,
			fb_mlb_par->block_no,
			fb_mlb_par->gr_no);
	if (current_trig == 2) {
		if (triger != 1) {
			ret = -EPERM;
		}
	} else if (current_trig == 3) {
		if (triger == 1) {
			ret = -EPERM;
		}
	}
	if (ret == 0) {
		int total_send_size =
			set_next_command(&send_command,
				eGRTRG,
				1, (u32 *)(&triger));
		ret = ipcu_send_request(&send_command,
				total_send_size);
		if (ret != 0) {
			dev_err(info->dev,
				"GR on/off fail.\n");
		}
	}
#endif
	return ret;
}
/**
* Set GR's triger to on/off by FBIOSET_TRIGER.
* Usually, it is called after FBIOFULL_SETTING setting.
* triger: Triger to set to GR.
* return: 0 will be return if the setting is successful.
*/
static int set_triger(struct fb_info *info, unsigned long triger)
{
	int ret = 0;

	switch (triger) {
	case 0:/* Force stop */
	case 1:/* Start */
	case 2:/* Frame stop */
		{
			ret = set_gr_triger(info, triger);
			break;
		}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
/**
* Switch a setting at DTS defined by FBIOSWITCH_GR_SETTING.
* info: FB's information of the device.
* switch_index: The index Switch to.
* return: 0 will be return if the setting is successful.
*/
static int switch_gr_setting(struct fb_info *info, u32 switch_index)
{
	int ret = 0;
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	u32 one_pixel_byes;

	fb_mlb_par->current_pattern = switch_index;
	set_var_fix(info,
		&fb_mlb_par->disp[switch_index]);
	one_pixel_byes = info->var.bits_per_pixel / 8;
	/* Get every areas drawing buffer address */
	get_every_area_address(info, one_pixel_byes);
	ret = initialize_disp(info);
	if (ret != 0) {
		dev_err(info->dev,
			"DISP initialize fail. Switching NG.\n");
	}
	return ret;
}
/**
* Switch a buffer to GR.
* It is called by FBIOSWICH_DROW_BUFFER.
* return: 0 will be return if the setting is successful.
*/
static int switch_draw_buffer(struct fb_info *info)
{
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	struct fb_disp *disp = &fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int current_buffer_index =		/* Drawing buffer index */
		 disp->current_buffer_index;
	int i, ret = 0;
	unsigned int enable_show = 0;


#ifdef TEST_DISP
	/* Set drawing address to DISP */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if (disp->enable_show_area[i]) {
			enable_show |= (1 << i);
			WARN_ON(!disp->
				input_image_ph_address[i].address[current_buffer_index]);
			if (i == 0) {
				int j;
				union grscctl scctl;

				scctl.word = 0;
				/* Set all image data buffer address */
				for (j = 0;
					j < disp->valid_buffer_number;
					j++) {
					writel_relaxed(
					 disp->input_image_ph_address[i].address[j],
					 fb_mlb_par->gr_reg_base
					 +DISP_GRSA0_OFFSET + j * 4);
				}
				/* Set showing bank index */
				scctl.bit.idset = current_buffer_index;
				writel_relaxed(scctl.word,
					 fb_mlb_par->gr_reg_base + DISP_GRSCCTL_OFFSET);
			} else {
				writel_relaxed(
				disp->
				input_image_ph_address[i].address[current_buffer_index],
				 fb_mlb_par->gr_reg_base
					+ DISP_GRSA_OFFSET
					+ (i - 1) * 4);
			}
#ifdef SUPORT_YUV_FORMAT
			/* Set A data buffer address */
			writel_relaxed(
			disp->
			input_A_data_ph_address[i].address[current_buffer_index],
			 fb_mlb_par->gr_reg_base + DISP_GRASA_OFFSET + i * 4);
#endif
		}
		writel_relaxed(enable_show,
				 fb_mlb_par->gr_reg_base + DISP_GRAREN_OFFSET);
	}
#else
	/* Use IPCU */
	t_ipcu_command send_command;
	int total_send_size;
	u32 image_address[MAX_DRAW_AREA_NUM - 1];

	initial_ipcu_command_head(&send_command,
			fb_mlb_par->block_no,
			fb_mlb_par->gr_no);


	/* Set drawing address to DISP */
	if (disp->enable_show_area[0]) {
		u32 scctl;

		enable_show = 1;
		WARN_ON(!disp->
			input_image_ph_address[0].address[current_buffer_index]);

		/* Set all image data buffer address */
		/* Set GRSA0 */
		set_next_command(&send_command,
				eGRSA0,
			disp->valid_buffer_number,
		 (u32 *)(disp->input_image_ph_address[0].address));

		/* Set showing bank index */
		scctl = current_buffer_index;

		/* Set GRSCCTL */
		set_next_command(&send_command,
				eGRSCCTL,
				1, &scctl);
	}
	/* Set every area's address */
	for (i = 1; i < MAX_DRAW_AREA_NUM; i++) {
		if (disp->enable_show_area[i]) {
			enable_show |= (1 << i);
			WARN_ON(!disp->
				input_image_ph_address[i].address[current_buffer_index]);
			image_address[i - 1] = (u32)disp->
			input_image_ph_address[i].address[current_buffer_index];
		} else {
			image_address[i - 1] = 0;
		}
	}
	set_next_command(&send_command,
				eGRSA,
				MAX_DRAW_AREA_NUM - 1,
				 image_address);
	/* Set enable area */
	total_send_size =
		set_next_command(&send_command,
				eGRAREN,
				1,
				&enable_show);

	ret = ipcu_send_request(&send_command,
				total_send_size);

#endif
	/* Update current_buffer_index */
	if (ret == 0) {
		disp->current_buffer_index++;
		if (disp->current_buffer_index >= disp->valid_buffer_number) {
			disp->current_buffer_index = 0;
		}
		/* Update drawing buffer */
		info->fix.smem_start = disp->
		input_image_ph_address[0].address[disp->current_buffer_index];
		info->screen_base =
			(char __iomem *)info->fix.smem_start;
	} else {
		dev_err(info->dev,
			"Switching drawing buffer NG.\n");
	}

	return ret;
}
/**
* Get every drawing area's address from their positon.
* fb_mlb_par: Device setting.
* one_pixel_byes: One pixel's bytes
* Return: valid drawing area num.
*/
static u32 get_every_area_address(struct fb_info *fb_mlb_info,
			u32 one_pixel_byes)
{
	struct fb_mlb_priv *fb_mlb_par = fb_mlb_info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int i, j;
	u32 valid_drawing_number = 0;

	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		if (disp->enable_show_area[i] == 1) {
			for (j = 0; j < disp->valid_buffer_number; j++) {
			u32 buffer_address =
				fb_mlb_info->fix.smem_start +
				j * fb_mlb_info->screen_size;
				disp->input_image_ph_address[i].address[j] =
				buffer_address +
				disp->
				input_image_position[i].bit.dsv
					* disp->byte_num_line +
				disp->
				input_image_position[i].bit.dsh
					* one_pixel_byes;
			}
		}
		if (disp->input_image_size[i].word != 0) {
			/* A valid drawing area must be none-zero */
			valid_drawing_number++;
		}
		disp->input_image_horizonal_bytes[i] = disp->byte_num_line;
	}
	return valid_drawing_number;
}
/**
* Set every drawing area's address to GR.
* fb_mlb_par: Device setting.
* send_command: Send IPCU command
* valid_drawing_number: valid drawing area num.
* total_send_size: IPCU command's total size.
* Return: Always 0.
*/
static int set_show_buffer_address(struct fb_mlb_priv *fb_mlb_par,
			t_ipcu_command *send_command,
			u32 valid_drawing_number,
			int *total_send_size)
{

	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int current_buffer_index =
		disp->current_buffer_index;/* Drawing buffer index */
	int i, ret = 0;
#ifdef TEST_DISP

	/* Set GRSCCTL */
	writel_relaxed(current_buffer_index,
				 fb_mlb_par->gr_reg_base + DISP_GRSCCTL_OFFSET);
	/* Set GRSA0 */
	for (i = 0; i < disp->valid_buffer_number; i++) {
		writel_relaxed(disp->input_image_ph_address[0].address[i],
		 fb_mlb_par->gr_reg_base + DISP_GRSA0_OFFSET + i * 4);
	}
	/* Set every area's address */
	for (i = 1; i < MAX_DRAW_AREA_NUM; i++) {
		writel_relaxed(
		disp->
		input_image_ph_address[i].address[current_buffer_index],
		 fb_mlb_par->gr_reg_base + DISP_GRSA_OFFSET
		+ (i - 1) * 4);
	}
#ifdef SUPORT_YUV_FORMAT
	/* Set every area's A data address */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel_relaxed(
		disp->
		input_A_data_ph_address[i].address[current_buffer_index],
		fb_mlb_par->gr_reg_base +
		DISP_GRASA_OFFSET + i * 4);
	}
#endif
	/* Set every area's bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel_relaxed(
			disp->input_image_horizonal_bytes[i],
			fb_mlb_par->gr_reg_base +
			DISP_GRHGA_OFFSET + i * 4);
	}

#ifdef SUPORT_YUV_FORMAT
	/* Set every area's A data bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel_relaxed(
			disp->A_data_horizonal_bytes[i],
			fb_mlb_par->gr_reg_base
			+ DISP_GRAHGA_OFFSET + i * 4);
	}
#endif
#else
	/* Use IPCU to set GR */
	t_ipcu_command send_ipcu_command;
	u32 image_address[MAX_DRAW_AREA_NUM - 1];
	u32 send_reguest_flag = 0;

	if (send_command == NULL) {
		send_command = &send_ipcu_command;
		initial_ipcu_command_head(send_command,
				fb_mlb_par->block_no,
				fb_mlb_par->gr_no);
		send_reguest_flag = 1;/* Need to send command */
	}
	/* Set GRSCCTL */
	set_next_command(send_command,
			eGRSCCTL,
			1, (u32 *)(&current_buffer_index));
	/* Set GRSA0 */
	set_next_command(send_command,
				eGRSA0,
				disp->valid_buffer_number,
			 (u32 *)(disp->input_image_ph_address[0].address));
	/* Set every area's address */
	for (i = 1; i < valid_drawing_number; i++) {
		image_address[i - 1] = (u32)disp->
			input_image_ph_address[i].address[current_buffer_index];
	}
	if (valid_drawing_number > 1) {
		set_next_command(send_command,
				eGRSA,
				valid_drawing_number - 1,
				 image_address);
	}
	/* Set every area's bytes num of one line */
	(*total_send_size) =
		set_next_command(send_command,
			eGRHGA,
			valid_drawing_number,
			 (u32 *)disp->input_image_horizonal_bytes);
	if (send_reguest_flag) {
		/*　Need to send command */
		ret = ipcu_send_request(send_command,
				*total_send_size);
	}


#endif
	return ret;
}
/**
* Set GR's parameter to GR register.
* fb_mlb_par: Device parameter.
* return: return 0 if the reading is successful.
*/
static int initialize_disp(const struct fb_info *info)
{
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	int i, ret = 0;
	struct fb_disp *disp = &fb_mlb_par->disp[fb_mlb_par->current_pattern];
	u32 enable_show = 0;

	if ((readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_TRIG_OFFSET)
		& 0x01) == 1) {
	#ifdef TEST_DISP
		u32 dummy;

		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->enable_show_area[i]) {
				enable_show |= (1<<i);
			}
		}
		/* Set Triger */
		writel_relaxed(0,
			fb_mlb_par->gr_reg_base + DISP_GRTRG_OFFSET);
		for (i = 0; i < 33; i++) {
			u32 trig =
				readl(fb_mlb_par->gr_reg_base
					+ DISP_GRTRG_OFFSET);

			if (trig == 2) {
				/* Set SR */
				writel_relaxed(1,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRRST_OFFSET);
				break;
			}
			msleep(1);
		}
		/* Clear SR */
		writel_relaxed(0,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRRST_OFFSET);
		/* Set format */
		writel_relaxed(disp->iput_trans_setting.word,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRIDT_OFFSET);
		/* Set GR size */
		writel_relaxed(disp->input_area_size.word,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRTISIZE_OFFSET);
		/* Set Position */
		writel_relaxed(disp->start_pos.word,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRTDSTA_OFFSET);
		/* Set IPO */
		writel_relaxed(0x03020100,
					 fb_mlb_par->gr_reg_base
					+ DISP_GRIPO_OFFSET);
		/* Set GRISIZE */
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			writel_relaxed(disp->input_image_size[i].word,
				fb_mlb_par->gr_reg_base
				+ DISP_GRISIZE_OFFSET + i * 4);
		}
		/*Set buffer address */
		set_show_buffer_address(fb_mlb_par, NULL, MAX_DRAW_AREA_NUM, &dummy);
		/* Set every area's position */
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			writel_relaxed(
				disp->input_image_position[i].word,
				fb_mlb_par->gr_reg_base
				+ DISP_GRDSTA_OFFSET + i * 4);
		}
		/* Set bilinear */
		writel_relaxed(
			1, fb_mlb_par->gr_reg_base + DISP_GRRSZ0_OFFSET);
		/* Set horizonal Resize */
		writel_relaxed(disp->horizonal_resize.word,
			fb_mlb_par->gr_reg_base + DISP_GRRSZ1_OFFSET);
		/* Set vertical Resize */
		writel_relaxed(disp->vertical_resize.word,
			fb_mlb_par->gr_reg_base + DISP_GRRSZ2_OFFSET);
		/* Set enable area */
		writel_relaxed(enable_show,
			fb_mlb_par->gr_reg_base + DISP_GRAREN_OFFSET);

		/* Set Triger */
		writel_relaxed(disp->gri_trg,
			fb_mlb_par->gr_reg_base + DISP_GRTRG_OFFSET);

	#else
		/* Use IPCU */
		t_ipcu_command send_command;
		int total_send_size;
		u32 triger = 0, sr = 1, ipo = 0x03020100;
		u32 resize[4];
		u32 valid_drawing_number = 0;

		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->enable_show_area[i]) {
				enable_show |= (1<<i);
			}
		}
		while (valid_drawing_number < MAX_DRAW_AREA_NUM &&
				disp->input_image_size[valid_drawing_number].word != 0) {
				valid_drawing_number++;
		}
		initial_ipcu_command_head(&send_command,
				fb_mlb_par->block_no,
				fb_mlb_par->gr_no);

		/* Set Triger */
		set_next_command(&send_command,
					eGRTRG,
					1, (u32 *)(&triger));
		/* Set SR */
		set_next_command(&send_command,
					eGRRST,
					1, (u32 *)(&sr));
		/* Clear SR */
		sr = 0;
		set_next_command(&send_command,
					eGRRST,
					1, (u32 *)(&sr));
		/* Set format */
		set_next_command(&send_command,
					eGRIDT,
					1, (u32 *)(&disp->iput_trans_setting));
		/* Set GR size */
		set_next_command(&send_command,
					eGRTISIZE,
					1, (u32 *)(&disp->input_area_size));
		/* Set Position */
		set_next_command(&send_command,
					eGRTDSTA,
					1, (u32 *)(&disp->start_pos.word));

		/* Set IPO */
		set_next_command(&send_command,
					eGRIPO,
					1, &ipo);
		/* Set GRISIZE */
		set_next_command(&send_command,
					eGRISIZE,
					valid_drawing_number,
				(u32 *)(&disp->input_image_size));
		/*Set buffer address */
		set_show_buffer_address(fb_mlb_par, &send_command,
			valid_drawing_number, &total_send_size);
		/* Set every area's position */
		set_next_command(&send_command,
					eGRDSTA,
					valid_drawing_number,
				(u32 *)(&disp->input_image_position));
		/* Set Resize */
		resize[0] = 1;
		resize[1] = (u32)disp->horizonal_resize.word;
		resize[2] = (u32)disp->vertical_resize.word;
		resize[3] = 0;

		set_next_command(&send_command,
					eGRRSZ,
					4,
					resize);

		/* Set enable area */
		set_next_command(&send_command,
					eGRAREN,
					1,
					&enable_show);

		/* Set Triger */
		total_send_size =
			set_next_command(&send_command,
					eGRTRG,
					1, (u32 *)(&disp->gri_trg));
		/* Send IPCU command */
		ret = ipcu_send_request(&send_command,
				total_send_size);

	#endif
	} else {
		dev_err(info->dev, "DISP's Dcore has not valid\n");
	}
	return ret;
}
#ifdef CONFIG_PM_SLEEP
/**
* Get GR register setting while suspend.
* fb_mlb_par: Device parameter.
* return: return 0 if the reading is successful.
*/
static int get_gr(struct fb_mlb_priv *fb_mlb_par)
{
	int i;

	/* Get Triger */
	fb_mlb_par->suspend_reg.disp.gri_trg =
		readl(fb_mlb_par->gr_reg_base + DISP_GRTRG_OFFSET) & 0x1;
	/* Get format */
	fb_mlb_par->suspend_reg.disp.iput_trans_setting.word =
		readl(fb_mlb_par->gr_reg_base +
		DISP_GRIDT_OFFSET);
	/* Get GR size */
	fb_mlb_par->suspend_reg.disp.input_area_size.word =
		readl(fb_mlb_par->gr_reg_base +
		DISP_GRTISIZE_OFFSET);
	/* Get Position */
	fb_mlb_par->suspend_reg.disp.start_pos.word =
	readl(fb_mlb_par->gr_reg_base +
		DISP_GRTDSTA_OFFSET);
	/* Get IPO */
	fb_mlb_par->suspend_reg.ipo =
		readl(fb_mlb_par->gr_reg_base +
		DISP_GRIPO_OFFSET);
	/* Get GRISIZE */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->suspend_reg.disp.input_image_size[i].word =
			readl(fb_mlb_par->gr_reg_base
			+ DISP_GRISIZE_OFFSET + i * 4);
	}
	/* Get GRSCCTL */
	fb_mlb_par->suspend_reg.scctl =
		readl(fb_mlb_par->gr_reg_base + DISP_GRSCCTL_OFFSET);
	/* Get GRSA0 */
	for (i = 0; i < 4; i++) {
		fb_mlb_par->
			suspend_reg.disp.input_image_ph_address[0].address[i] =
			 readl(fb_mlb_par->gr_reg_base
			+ DISP_GRSA0_OFFSET + i * 4);
	}
	/* Get every area's address */
	for (i = 1; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->
			suspend_reg.disp.input_image_ph_address[i].address[0] =
			readl(fb_mlb_par->gr_reg_base
			+ DISP_GRSA_OFFSET
			+ (i - 1) * 4);
	}
#ifdef SUPORT_YUV_FORMAT
	/* Get every area's A data address */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->
			suspend_reg.disp.input_A_data_ph_address[i].address[0] =
			readl(fb_mlb_par->gr_reg_base
				+ DISP_GRASA_OFFSET
				+ i * 4);
	}
	/* Get every area's A data bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->
			suspend_reg.disp.A_data_horizonal_bytes[i] =
			readl(fb_mlb_par->gr_reg_base
			+ DISP_GRAHGA_OFFSET + i * 4);
	}
#endif
	/* Get every area's bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->
			suspend_reg.disp.input_image_horizonal_bytes[i] =
			readl(fb_mlb_par->gr_reg_base
			+ DISP_GRHGA_OFFSET + i * 4);
	}
	/* Get every area's position */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		fb_mlb_par->
			suspend_reg.disp.input_image_position[i].word =
			readl(fb_mlb_par->gr_reg_base
				+ DISP_GRDSTA_OFFSET + i * 4);
	}
	/* Get bilinear */
	fb_mlb_par->suspend_reg.rsz0 =
		readl(fb_mlb_par->gr_reg_base
		+ DISP_GRRSZ0_OFFSET);
	/* Get horizonal Resize */
	fb_mlb_par->suspend_reg.disp.horizonal_resize.word =
		readl(fb_mlb_par->gr_reg_base + DISP_GRRSZ1_OFFSET);
	/* Get vertical Resize */
	fb_mlb_par->suspend_reg.disp.vertical_resize.word =
		readl(fb_mlb_par->gr_reg_base + DISP_GRRSZ2_OFFSET);
	/* Get enable area */
	fb_mlb_par->suspend_reg.gr_enable =
		readl(fb_mlb_par->gr_reg_base + DISP_GRAREN_OFFSET);

	return 0;
}
/**
* Get dcore register setting while suspend.
* fb_mlb_par: Device parameter.
* return: return 0 if the reading is successful.
*/
static int get_dcore(struct fb_mlb_priv *fb_mlb_par)
{
	if (block_suspend_resume_status[fb_mlb_par->block_no]
		!= BLCOK_DONE_SUSPEND) {
		int i;

		/* Get IFS */
		fb_mlb_par->suspend_reg.ifs.word =
			readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_IFS_OFFSET);
		/* Get Triger */
		fb_mlb_par->suspend_reg.dcore_trig.word =
			readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_TRIG_OFFSET);
		fb_mlb_par->suspend_reg.dcore_trig.bit.trg &= 0x01;
		/* Get From TGKST to INTE */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_control1)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_control1[i] =
				readl(fb_mlb_par->dcore_reg_base +
					DISP_DCORE_TGKST_OFFSET + i*4);
		}
		/* Get RPGCTL and RPGEN */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_control2)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_control2[i] =
				readl(fb_mlb_par->dcore_reg_base +
					DISP_DCORE_RPGCTL_OFFSET + i*4);
		}
		/* Get From POLSEL to VRFCTL */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_timing)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_timing[i] =
				readl(fb_mlb_par->dcore_reg_base +
					DISP_DCORE_POLSEL_OFFSET + i * 4);
		}
		/* Get HRFCTL */
		fb_mlb_par->suspend_reg.dcore_hrfctl =
			readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_HRFCTL_OFFSET);
		/* Get HABLK */
		fb_mlb_par->suspend_reg.dcore_hablk =
			readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_HABLK_OFFSET);
		/* Get DOMD */
		fb_mlb_par->suspend_reg.dcore_domd =
			readl(fb_mlb_par->dcore_reg_base + DISP_DCORE_DOMD_OFFSET);
		/* Get FDOEN and FODATA */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_force_data)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_force_data[i] =
				readl(fb_mlb_par->dcore_reg_base +
				DISP_DCORE_FDOEN_OFFSET+i * 4);
		}
		/* Get BLANKDT */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_blankdt)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_blankdt[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_BLANKDT_OFFSET + i * 4);
		}
		/* Get CLBHSIZE */
		fb_mlb_par->suspend_reg.dcore_clbhsize =
			readl(fb_mlb_par->dcore_reg_base
				+ DISP_DCORE_CLBHSIZE_OFFSET);
		/* Get CLBDT */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_clbdt)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_clbdt[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_CLBDT_OFFSET + i * 4);
		}
		/* Get R2Y */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_r2y)
			/ sizeof(u64);
			i++) {
			fb_mlb_par->suspend_reg.dcore_r2y[i] =
				readq(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_R2Y_OFFSET + i * 8);
		}
		/* Get YCAL */
		fb_mlb_par->suspend_reg.dcore_ycal =
			readq(fb_mlb_par->dcore_reg_base + DISP_DCORE_YCAL_OFFSET);
		/* Get From YCLIP to CRCLIP */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_clip_cal)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_clip_cal[i] =
				readl(fb_mlb_par->dcore_reg_base
				+ DISP_DCORE_YCLIP_OFFSET + i * 4);
		}
		/* Get From DOCTL0 to DOCTL2 */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_doctl)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_doctl[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_DOCT0_OFFSET + i * 4);
		}
		/* Get TRSCODE0/1	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_trscode)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_trscode[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_TRSCODE_OFFSET + i * 4);
		}
		/* Get Y2R */
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_y2r)
			/ sizeof(u64);
			i++) {
			fb_mlb_par->suspend_reg.dcore_y2r[i] =
				readq(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_Y2R_OFFSET + i * 8);
		}
		/* Get TBLASET */
		fb_mlb_par->suspend_reg.dcore_tblaset =
			readl(fb_mlb_par->dcore_reg_base +
			DISP_DCORE_TBLASET_OFFSET);
		/* Get GRID	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_grid)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_grid[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_GRID_OFFSET + i * 4);
		}
		/* Get GDISPEN */
		fb_mlb_par->suspend_reg.dcore_gdispen =
			readl(fb_mlb_par->dcore_reg_base
				+ DISP_DCORE_GDISPEN_OFFSET);
		/* Get FFDSTA	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_facesta)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_facesta[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_FACEPOS_OFFSET + i * 4);
		}
		/* Get FFSIZE	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_facesize)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_facesize[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_FACESIZE_OFFSET + i * 4);
		}
		/* Get FFWIDTH	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_facewidth)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_facewidth[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_FACEWIDTH_OFFSET + i * 4);
		}
		/* Get FFCLR	*/
		for (i = 0;
			i < sizeof(fb_mlb_par->suspend_reg.dcore_faceclr)
			/ sizeof(u32);
			i++) {
			fb_mlb_par->suspend_reg.dcore_faceclr[i] =
				readl(fb_mlb_par->dcore_reg_base
				 + DISP_DCORE_FACEFCLR_OFFSET + i * 4);
		}
		/* Get FFDISPEN */
		fb_mlb_par->suspend_reg.dcore_ffdispen =
			readq(fb_mlb_par->dcore_reg_base
				+ DISP_DCORE_FFDISPEN_OFFSET);
		/* Get FFDO */
		fb_mlb_par->suspend_reg.dcore_ffdo =
			readl(fb_mlb_par->dcore_reg_base
				+ DISP_DCORE_FFDO_OFFSET);
		/* Set done dcore complite suspend */
		block_suspend_resume_status[fb_mlb_par->block_no]
			= BLCOK_DONE_SUSPEND;
	}
	return 0;
}
#ifndef TEST_DISP
/**
* Initiualize IPCU for send commannd to RTOS
* node: parent node
*/
static int initialize_ipcu(struct device_node *node)
{
	int ret = 0;

	if (gr_ipcu.ipcu_command_buffer == NULL) {
		struct device_node *ipcu_node =
			of_get_child_by_name(node, "fb_common");

		if (ipcu_node) {
			u32 temp[2];

			of_property_read_u32_array(ipcu_node,
				"reg", temp,
				2);
			gr_ipcu.ipcu_command_buffer = of_iomap(ipcu_node, 0);
			gr_ipcu.ipcu_command_buffer_ph = temp[0];
			gr_ipcu.ipcu_command_buffer_ph_len = temp[1];
			of_property_read_u32(ipcu_node,
				"ipcu_unit", &gr_ipcu.ipcu_unit);
			of_property_read_u32_array(ipcu_node,
				"ipcu_ch", gr_ipcu.ipcu_ch,
					2);
		} else {
			pr_err("IPCU not exist err\n");
			ret = -EINVAL;
		}
	}
	if (ret == 0) {
		ret = open_ipcu();
		if (ret == 0) {
			gr_ipcu.ipcu_wq =
				create_singlethread_workqueue("ipcu_disp_recv");
			INIT_WORK(&gr_ipcu.work, ipcu_work_handler);
		}
	}
	return ret;
}
#endif

/**
* Read all drawing area's setting from DTS
* node: device node
* fb_mlb_par: Device parameter.
* return: return 0 if the reading is successful.
*/
static int read_drawing_area(struct device_node *node,
	struct fb_mlb_priv *fb_mlb_par)
{
	struct device_node *child;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	u32 read_buffer[2];
	int i;

	disp->valid_input_area = 0;
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		child = of_get_child_by_name(node, AREA_NAME[i]);
		if (child) {
			of_property_read_u32(child, "show_enable",
				&disp->enable_show_area[i]);
			if (disp->enable_show_area[i]) {
				disp->valid_input_area |= (1<<i);
				/* Get area's size at GR */
				of_property_read_u32_array(child, "size",
							 read_buffer, 2);
				disp->input_image_size[i].bit.ihsize =
					read_buffer[0]; /* width */
				disp->input_image_size[i].bit.ivsize =
					read_buffer[1]; /* height */
				/* Get area's position at GR */
				of_property_read_u32_array(child, "positon",
							 read_buffer, 2);
				/* horizonal coordinate */
				disp->input_image_position[i].bit.dsh =
					read_buffer[0];
				/* vertical coordinate */
				disp->input_image_position[i].bit.dsv =
					read_buffer[1];
			}
		} else {
			break;
		}
	}
	return 0;
}
/**
* Read all setting pattern from DTS
* node: device node
* pattern_name: pattern name get from pattern-name.
* index: GR's index.
* return: return 0 if the reading is successful.
*/
static int get_every_pattern(struct device_node *node,
					const char *pattern_name,
					struct fb_info *info)
{
	int ret = 0;
	u32 read_buffer[2];
	u32 dis_format;
#ifdef SUPORT_YUV_FORMAT
	u32 a_data_byte_num_line = 0;/* One line bytes of A data */
#endif
	int one_pixel_byes = 4;
	struct fb_mlb_priv *fb_mlb_par =
			(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
			&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	struct device_node *child =
			of_get_child_by_name(node, pattern_name);

	if (child) {
		/* Get input size */
		of_property_read_u32_array(child, "dis_size",
					 read_buffer, 2);

		disp->input_area_size.bit.ihsize =
			read_buffer[0]; /* horizonal size */
		disp->input_area_size.bit.ivsize =
			read_buffer[1]; /* vertiacal size */

		/* Get virtual screen size */
		of_property_read_u32_array(child, "dis_virtual_size",
					 disp->res_virtual, 2);

		/* Get resize of horizonal */
		of_property_read_u32_array(child, "dis_expand_horizonal",
					 read_buffer, 2);
		disp->horizonal_resize.bit.rszm =
			read_buffer[0]; /* horizonal resize numerator */
		disp->horizonal_resize.bit.rszn =
			read_buffer[1]; /* horizonal resize denominator */

		/* Get resize of vertical */
		of_property_read_u32_array(child, "dis_expand_vertical",
					 read_buffer, 2);
		disp->vertical_resize.bit.rszm =
			read_buffer[0]; /* vertical resize numerator */
		disp->vertical_resize.bit.rszn =
			read_buffer[1]; /* vertical resize denominator */
		/* Get GR's position at window */
		of_property_read_u32_array(child, "dis_position",
					 read_buffer, 2);
		disp->start_pos.bit.dsh =
			read_buffer[0]; /* horizonal coordinate */
		disp->start_pos.bit.dsv =
			read_buffer[1]; /* vertical coordinate */

		/* Get Pan display step */
		of_property_read_u32_array(child, "dis_pan_step",
					 disp->pan_step, 2);

		/* Get GR data format */
		of_property_read_u32(child, "dis_format",
			&dis_format);
		disp->iput_trans_setting.bit.ifmt = dis_format;
		/* Get GR one line bytes */
		of_property_read_u32(child,
				"byte_num_line",
				&disp->byte_num_line);

		/* Get draw buffer address */
		of_property_read_u32(child, "draw_buffer_num",
			&(disp->valid_buffer_number));
		if (disp->valid_buffer_number > 4) {
			disp->valid_buffer_number = 4;
			pr_err("Max draw_buffer_num is 4. It is forced to set to 4.\n");
		}
		of_property_read_u32_array(child,
				"draw_addrss",
				 disp->drawing_buffer_top_address_size,
					2);

		set_var_fix(info,
			disp);
		one_pixel_byes =
			info->var.bits_per_pixel / 8;

		of_property_read_u32(child, "dis_triger", &disp->gri_trg);

#ifdef SUPORT_YUV_FORMAT
		/* Get GR one line A data bytes */
		of_property_read_u32(child,
			"a_data_byte_num_line",
			&a_data_byte_num_line);
#endif
		/* Get every drwa area */
		read_drawing_area(child, fb_mlb_par);

		/* Get every areas drawing buffer address */
		get_every_area_address(info, one_pixel_byes);

		/* here need modify */
#ifdef SUPORT_YUV_FORMAT
		if (a_data_byte_num_line > 0) {
			/* Get draw A buffer address */
			for (i = 0;
				i < fb_mlb_par->valid_buffer_number; i++) {
				disp->input_A_data_address[0].address[i] =
				(u32)devm_ioremap(dev,
				disp->input_A_data_ph_address[0].address[i],
				disp->input_area_size.bit.ivsize *
				a_data_byte_num_line);
			}
		}
#endif
		ret = check_parameter(info, disp);
	} else {
		pr_err("%s node err\n", pattern_name);
		ret = -EINVAL;
	}
	return ret;
}
/**
* Set GR's data to GR register when resume.
* fb_mlb_par: Device parameter.
* return: return 0 if the reading is successful.
*/
static int resume_gr(const struct fb_mlb_priv *fb_mlb_par)
{
	int i;

	/* Clear SR */
	writel(0, fb_mlb_par->gr_reg_base + DISP_GRRST_OFFSET);

	/* Set triger to 0 */
	writel(0,
		fb_mlb_par->gr_reg_base + DISP_GRTRG_OFFSET);
	/* Set format */
	writel(
		fb_mlb_par->suspend_reg.disp.iput_trans_setting.word,
		fb_mlb_par->gr_reg_base + DISP_GRIDT_OFFSET);
	/* Set GR size */
	writel(
		fb_mlb_par->suspend_reg.disp.input_area_size.word,
		fb_mlb_par->gr_reg_base + DISP_GRTISIZE_OFFSET);
	/* Set Position */
	writel(
		fb_mlb_par->suspend_reg.disp.start_pos.word,
		fb_mlb_par->gr_reg_base + DISP_GRTDSTA_OFFSET);
	/* Set IPO */
	writel(
		fb_mlb_par->suspend_reg.ipo,
		fb_mlb_par->gr_reg_base + DISP_GRIPO_OFFSET);
	/* Set GRISIZE */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->suspend_reg.disp.input_image_size[i].word,
			fb_mlb_par->gr_reg_base + DISP_GRISIZE_OFFSET + i * 4);
	}
	/* Set GRSCCTL */
	writel(fb_mlb_par->suspend_reg.scctl,
		fb_mlb_par->gr_reg_base + DISP_GRSCCTL_OFFSET);
	/* Set GRSA0 */
	for (i = 0; i < 4; i++) {
		writel(
		fb_mlb_par->
			suspend_reg.disp.input_image_ph_address[0].address[i],
		 fb_mlb_par->
			gr_reg_base + DISP_GRSA0_OFFSET + i * 4);
	}
	/* Set every area's address */
	for (i = 1; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->
			suspend_reg.disp.input_image_ph_address[i].address[0],
			fb_mlb_par->
				gr_reg_base + DISP_GRSA_OFFSET + (i - 1) * 4);
	}
#ifdef SUPORT_YUV_FORMAT
	/* Set every area's A data address */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->
			suspend_reg.disp.input_A_data_ph_address[i].address[0],
			fb_mlb_par->
				gr_reg_base + DISP_GRASA_OFFSET + i * 4);
	}
	/* Set every area's A data bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->
				suspend_reg.disp.A_data_horizonal_bytes[i],
			fb_mlb_par->
				gr_reg_base + DISP_GRAHGA_OFFSET + i * 4);
	}
#endif
	/* Set every area's bytes num of one line */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->
				suspend_reg.disp.input_image_horizonal_bytes[i],
			fb_mlb_par->
				gr_reg_base + DISP_GRHGA_OFFSET + i * 4);
	}
	/* Set every area's position */
	for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
		writel(
			fb_mlb_par->
				suspend_reg.disp.input_image_position[i].word,
			fb_mlb_par->
				gr_reg_base + DISP_GRDSTA_OFFSET + i * 4);
	}
	/* Set bilinear */
	writel(fb_mlb_par->suspend_reg.rsz0,
		fb_mlb_par->gr_reg_base + DISP_GRRSZ0_OFFSET + i * 4);
	/* Set horizonal Resize */
	writel(fb_mlb_par->suspend_reg.disp.horizonal_resize.word,
		 fb_mlb_par->gr_reg_base + DISP_GRRSZ1_OFFSET);
	/* Set vertical Resize */
	writel(fb_mlb_par->suspend_reg.disp.vertical_resize.word,
		fb_mlb_par->gr_reg_base + DISP_GRRSZ2_OFFSET);
	/* Set enable area */
	writel(fb_mlb_par->suspend_reg.gr_enable,
		fb_mlb_par->gr_reg_base + DISP_GRAREN_OFFSET);

	for (i = 0; i < 10; i++) {
		u32 dcore_trig =
			readl(fb_mlb_par->dcore_reg_base +
				DISP_DCORE_TRIG_OFFSET);

		if (dcore_trig == 3) {
			/* Set Triger */
			writel(fb_mlb_par->suspend_reg.disp.gri_trg,
				fb_mlb_par->gr_reg_base
				+ DISP_GRTRG_OFFSET);
			break;
		}
		msleep(10);
	}
	return 0;
}
#ifndef TEST_DISP
/**
* Send enable/disable command to RTOS by IPCU.
* enable_show: Enable/disable bit map
* return: 0 = OK
*/
static int send_enable_area_by_ipcu(
				const struct fb_mlb_priv *fb_mlb_par,
				u32 enable_show)
{
	t_ipcu_command send_command;
	int total_send_size;

	initial_ipcu_command_head(&send_command,
			fb_mlb_par->block_no,
			fb_mlb_par->gr_no);

	/* Set enable area */
	total_send_size =
		set_next_command(&send_command,
				eGRAREN,
				1,
				&enable_show);

	return ipcu_send_request(&send_command,
				total_send_size);
}
#endif
/**
* Set GR's drawing area to enable/disable
* info: FB's information
* enable_show: Enable/disable bit map
* return: 0 = OK
*/
static int set_enable_area(struct fb_info *info,
			u32 enable_show)
{
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int i, ret = 0;

	if ((~disp->valid_input_area) & enable_show) {
		/* Want to set invalid area */
		dev_err(info->dev,
			"Attempt to enable invalid drawing area Error.\n");
		dev_err(info->dev,
			"Valid input_area:%x. Enable show:%x\n",
			disp->valid_input_area, enable_show);
	} else {
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (enable_show & (1<<i)) {
				disp->enable_show_area[i] = 1;
			} else {
				disp->enable_show_area[i] = 0;
			}
		}
	}
#ifdef TEST_DISP
		writel_relaxed(enable_show,
				 fb_mlb_par->gr_reg_base
				+ DISP_GRAREN_OFFSET);
#else
	ret = send_enable_area_by_ipcu(fb_mlb_par, enable_show);
#endif
	return ret;

}
/**
* fb_ops function
*/
/**
* mmap fuction
*/
static int milb_disp_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long start = 0, off;
	int ret;
	struct fb_mlb_priv *fb_mlb_par = (struct fb_mlb_priv *)info->par;
	int current_buffer_index;		/* Drawing buffer index */
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];

	mutex_lock(&gr_ipcu.mlock);
	current_buffer_index =
		disp->current_buffer_index;
	start =
		disp->input_image_ph_address[0].address[current_buffer_index];

	len = PAGE_ALIGN(vma->vm_end - vma->vm_start);

	off = vma->vm_pgoff << PAGE_SHIFT;

	if ((vma->vm_end - vma->vm_start + off) > len) {
		ret = -EINVAL;
	} else {
		off += start;
		vma->vm_pgoff = off >> PAGE_SHIFT;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		vma->vm_flags |= VM_IO;

		ret = io_remap_pfn_range(vma,
			vma->vm_start, off >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

		if (ret) {
			pr_err("failed to mmap[%d] vm = 0x%08X/%dbyte, pm = 0x%08lX\n",
			ret, (int)vma, len, start);
			ret = -ENOBUFS;
		}
	}
	WARN_ON(ret);
	mutex_unlock(&gr_ipcu.mlock);
	return ret;
}
/**
* Pan dispaly fuction
*/
static int set_pan_dispay(
		const struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	int i, ret = 0;
	u32 write_value;
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int one_pixel_bytes =
		info->var.bits_per_pixel / 8;
	u32 enable_show = 0;
	u32 byte_bit_mask;

	if (fb_mlb_par->block_no == BLOCK_HDMI) {
		byte_bit_mask = 0x0F;/* Must be over 16 bytes */
	} else {
		byte_bit_mask = 0x07;/* Must be over 4 bytes */
	}
	if (((var->xoffset * one_pixel_bytes)
		& byte_bit_mask) == 0) {
#ifdef TEST_DISP

		/* Set drawing address to DISP */
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->enable_show_area[i]) {
				u32 value =
					var->yoffset *
					disp->input_image_horizonal_bytes[i] +
						var->xoffset * one_pixel_bytes;

				write_value = value +
					disp->input_image_ph_address[i].address[0];
				if (i == 0) {
					union grscctl scctl;

					scctl.word = 0;
					writel_relaxed(write_value,
						 fb_mlb_par->gr_reg_base +
						DISP_GRSA0_OFFSET);
					writel_relaxed(scctl.word,
						 fb_mlb_par->gr_reg_base +
						DISP_GRSCCTL_OFFSET);
				} else {
					writel_relaxed(write_value,
						 fb_mlb_par->gr_reg_base +
							DISP_GRSA_OFFSET
							+ (i - 1) * 4);
				}
#ifdef SUPORT_YUV_FORMAT
				/* Set A data buffer address */
				value = var->yoffset *
					disp->A_data_horizonal_bytes[i] +
					var->xoffset * one_pixel_bytes;

				write_value = value +
					disp->input_A_data_ph_address[i].address[0];
				writel_relaxed(write_value,
					 fb_mlb_par->gr_reg_base +
					DISP_GRASA_OFFSET + i * 4);
#endif
				enable_show |= (1<<i);
			}
		}
		writel_relaxed(enable_show,
				 fb_mlb_par->gr_reg_base + DISP_GRAREN_OFFSET);
#else
		u32 image_address[MAX_DRAW_AREA_NUM - 1];
		t_ipcu_command send_command;
		int total_send_size;

		initial_ipcu_command_head(&send_command,
				fb_mlb_par->block_no,
				fb_mlb_par->gr_no);

		/* Set drawing address to DISP */
		if (disp->enable_show_area[0]) {
			u32 scctl = 0;
			u32 value =
					var->yoffset *
					disp->input_image_horizonal_bytes[0] +
						var->xoffset * one_pixel_bytes;

				write_value = value +
					disp->input_image_ph_address[0].address[0];

			/* Set all image data buffer address */
			/* Set GRSA0 */
			set_next_command(&send_command,
					eGRSA0,
					1, &write_value);

			/* Set showing bank index */
			/* Set GRSCCTL */
			set_next_command(&send_command,
					eGRSCCTL,
					1, &scctl);

			enable_show = 1;
		}
		/* Set every area's address */
		for (i = 1; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->enable_show_area[i]) {
				u32 value =
					var->yoffset *
					disp->input_image_horizonal_bytes[i] +
						var->xoffset * one_pixel_bytes;

				image_address[i - 1] = value +
					disp->input_image_ph_address[i].address[0];
				enable_show |= (1 << i);
			} else {
				image_address[i - 1] = 0;
			}
		}
		set_next_command(&send_command,
					eGRSA,
					MAX_DRAW_AREA_NUM - 1,
					image_address);
		/* Set enable area */
		total_send_size =
			set_next_command(&send_command,
					eGRAREN,
					1,
					&enable_show);

		ret = ipcu_send_request(&send_command,
					total_send_size);
		if (ret != 0) {
			dev_err(info->dev,
				"Address setting fail.\n");
		}

#endif
	} else {
		dev_err(info->dev,
			"Error. Xoffset is invalid:%u):\n"
			, var->xoffset);
		ret = -EINVAL;
	}
	return ret;
}

/**
* Pan dispaly fuction
*/
static int milb_disp_pan_display(
		struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	int ret = -EINVAL;

	mutex_lock(&gr_ipcu.mlock);
	ret = set_pan_dispay(var, info);
	if (ret == 0) {
		info->var.xoffset = var->xoffset;
		info->var.yoffset = var->yoffset;
	} else {
		dev_err(info->dev,
			"Error:%d. %s %d\n"
			, ret, __func__, __LINE__);
	}
	mutex_unlock(&gr_ipcu.mlock);
	return ret;
}
/**
* ioctl function
*/
static int milb_disp_ioctl(struct fb_info *info,
		unsigned int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];

	mutex_lock(&gr_ipcu.mlock);
	switch (cmd) {
	case FBIOSWICH_DROW_BUFFER: {
		ret = switch_draw_buffer(info);
		break;
	}
	case FBIOENABLE_DROW_AREA:
		/* max 10 areas */
		ret = set_enable_area(info, arg & 0x3FF);
		break;
	case FBIOFULL_GETTING: {
		/* Get all setting */
		void __user *argp = (void __user *)arg;

		if (copy_to_user(argp, disp,
			sizeof(*disp)))
			ret = -EFAULT;
		else
			ret = 0;
	}
		break;
	case FBIOFULL_SETTING: {
		void __user *argp = (void __user *)arg;
		struct fb_disp user_disp;

		if (copy_from_user(&user_disp, argp,
			sizeof(user_disp))) {
			ret = -EFAULT;
		} else {
			ret = check_parameter(info, &user_disp);
			if (ret == 0) {
				ret = set_full_gr(info, &user_disp);
			} else {
				dev_err(info->dev,
					"Error. command code:%u,ret:%d):\n"
					, cmd, ret);
			}
		}
		break;
	}
	case FBIOSET_TRIGER:
		ret = set_triger(info, arg);
		break;
	case FBIOSWITCH_GR_SETTING:
		if (arg < fb_mlb_par->disp_num) {
			ret = switch_gr_setting(info, arg);
		} else {
			dev_err(info->dev,
				"Error. Switvhing index is too large:%lu):\n"
				, arg);
			ret = -EINVAL;
		}
		break;
	case FBIOGET_DRAWAREA:{
		void __user *argp = (void __user *)arg;
		draw_area user_draw_area;

		memcpy(
			user_draw_area.input_image_size,
			disp->input_image_size,
			sizeof(disp->input_image_size));
		memcpy(
			user_draw_area.input_image_position,
			disp->input_image_position,
			sizeof(disp->input_image_position));
		memcpy(
			user_draw_area.enable_show_area,
			disp->enable_show_area,
			sizeof(disp->enable_show_area));
		memcpy(
			user_draw_area.input_image_horizonal_bytes,
			disp->input_image_horizonal_bytes,
			sizeof(disp->input_image_horizonal_bytes));
		ret = copy_to_user(argp,
			&user_draw_area, sizeof(user_draw_area));
		break;
	}
	case FBIOSET_DRAWAREA:{
		void __user *argp = (void __user *)arg;
		draw_area user_draw_area;

		if (copy_from_user(&user_draw_area,
			argp, sizeof(user_draw_area))) {
			ret = -EFAULT;
		} else {
			ret = set_drawing_area(info, &user_draw_area);
			if (ret != 0) {
				dev_err(info->dev,
				"Error. command code:%u,ret:%d):\n"
				, cmd, ret);
			}
		}
		break;
	}
	default:
		break;
	}
	mutex_unlock(&gr_ipcu.mlock);
	return ret;
}
/**
* Blank function.
* It set all drawing area to on/off.
* return: Allways 0.
*/
static int milb_disp_enable(int blank, struct fb_info *info)
{
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	u32 enable_show = 0;
	int i, ret = 0;

	mutex_lock(&gr_ipcu.mlock);
	if (blank) {
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			disp->enable_show_area[i] = 0;
		}
	} else {
		enable_show = disp->valid_input_area;
		for (i = 0; i < MAX_DRAW_AREA_NUM; i++) {
			if (disp->valid_input_area & (1<<i)) {
				disp->enable_show_area[i] = 1;
			}
		}
	}
#ifdef TEST_DISP
	writel_relaxed(enable_show,
			 fb_mlb_par->gr_reg_base +
			DISP_GRAREN_OFFSET);
#else
	ret = send_enable_area_by_ipcu(fb_mlb_par, enable_show);

#endif
	mutex_unlock(&gr_ipcu.mlock);
	if (ret != 0) {
		dev_err(info->dev,
			"DISP setting fail.\n");
	}
	return ret;
}
/**
* fb_set_par's function
* It can set xres_virtual and yres_virtual only.
*/
static int milb_disp_set_par(struct fb_info *info)
{
	int ret = 0;
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int one_pixel_bytes = info->var.bits_per_pixel / 8;
	u32 dummy = 0;
	u32 valid_drawing_number;

	mutex_lock(&gr_ipcu.mlock);
	disp->byte_num_line = info->var.xres_virtual *
		one_pixel_bytes;
	/* Get drawing area address */
	valid_drawing_number = get_every_area_address(info, one_pixel_bytes);

	/* Reset current index */
	disp->current_buffer_index = 0;
	/* Change pattern data */
	disp->res_virtual[0] = info->var.xres_virtual;
	disp->res_virtual[1] = info->var.yres_virtual;

	set_show_buffer_address(fb_mlb_par, NULL, valid_drawing_number, &dummy);

	mutex_unlock(&gr_ipcu.mlock);
	return ret;
}
/**
* fb_check_par's function
* It can set xres_virtual and yres_virtual only.
*/

static int milb_disp_check_par(
		struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	int ret = -EINVAL;
	struct fb_var_screeninfo checking_var = *var;
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)info->par;
	struct fb_disp *disp =
		&fb_mlb_par->disp[fb_mlb_par->current_pattern];
	int one_pixel_bytes = info->var.bits_per_pixel / 8;
	u32 screen_size = (
				var->xres_virtual *
				var->yres_virtual * one_pixel_bytes);

	if (var->xres != info->var.xres ||
		var->yres != info->var.yres) {
		/* The size of GR is not permit to change */
		dev_err(info->dev,
			"%s %d error. old res(%d,%d) new(%d,%d):\n"
			, __func__, __LINE__,
		var->xres, var->yres,
		info->var.xres, info->var.yres);
	} else if (
		((var->xres_virtual < info->var.xres) ||
		(var->yres_virtual < info->var.yres))) {
		dev_err(
		info->dev,
		"%s %d error. virtual(%d,%d) var(%d,%d)\n",
		__func__,
		__LINE__,
		var->xres_virtual,
		var->yres_virtual,
		info->var.xres,
		info->var.xres);
	} else if (screen_size *
		disp->valid_buffer_number
		> info->fix.smem_len) {
		dev_err(
		info->dev,
		"%s %d error. screen_size:%lu,smem_len:%d,valid_buffer_number:%d,virtual(%u,%u)\n",
		__func__,
		__LINE__,
		info->screen_size,
		info->fix.smem_len,
		disp->valid_buffer_number,
		var->xres_virtual,
		var->yres_virtual);
	} else {
		/* Skip this checking after */
		checking_var.xres_virtual =
			info->var.xres_virtual;
		checking_var.yres_virtual =
			info->var.yres_virtual;
		if (memcmp(&checking_var,
			&info->var, sizeof(checking_var)) != 0) {
			int i;
			u32 *p1 = (u32 *)(&checking_var);
			u32 *p2 = (u32 *)(&info->var);

			dev_err(info->dev,
				"%s %d error\n", __func__, __LINE__);
			for (i = 0; i < sizeof(checking_var); i +=4) {
				dev_err(info->dev,
					"[%d] %X:%X\n", i, *p1, *p2);
				p1++;
				p2++;
			}
		} else {
			ret = 0;
		}
	}

	return ret;
}
static struct fb_ops milb_disp_ops = {
	.owner          = THIS_MODULE,
	.fb_ioctl = milb_disp_ioctl,
	.fb_compat_ioctl = milb_disp_ioctl,
	.fb_mmap = milb_disp_mmap,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_pan_display = milb_disp_pan_display,
	.fb_check_var = milb_disp_check_par,
	.fb_set_par = milb_disp_set_par,
	.fb_blank = milb_disp_enable,
};

struct milb_disp_par {
	struct fb_disp disp;
};
/**
*
*/
static int milb_disp_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct resource *res_dcore;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int index = 0;
	int i;
	u32 temp;
	struct fb_mlb_priv *fb_mlb_par;

	fb_mlb_par = devm_kzalloc(dev,
			sizeof(*fb_mlb_par), GFP_KERNEL);
	memset(fb_mlb_par, 0, sizeof(*fb_mlb_par));
	/* Get GR channel No. */
	of_property_read_u32(node, "index", &index);
	fb_mlb_info[index].par = fb_mlb_par;
	platform_set_drvdata(pdev, &fb_mlb_info[index]);
	fb_mlb_info[index].fbops = &milb_disp_ops;
	fb_mlb_info[index].flags = FBINFO_FLAG_DEFAULT;
	fb_mlb_info[index].dev = dev;
	fb_mlb_par->gr_no = (u16)(index & 0x01);

	/* Get DCORE base */
	/* Get DCORE's IO address */
	res_dcore = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "dcore");
	fb_mlb_par->dcore_reg_base =
		devm_ioremap_resource(dev, res_dcore);
	/* Get Block No. */
	of_property_read_u32(node, "block_no", &temp);
	fb_mlb_par->block_no = (u16)temp;

	/* Get GR's IO address */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gr");
	fb_mlb_par->gr_reg_base = devm_ioremap_resource(dev, res);
	fb_mlb_info[index].fix.mmio_start = res->start;
	fb_mlb_info[index].fix.mmio_len = res->end - res->start;

	/* Get setting pattern num */
	of_property_read_u32(node, "num-set-pattern",
		&fb_mlb_par->disp_num);
	fb_mlb_par->disp = devm_kzalloc(dev,
		sizeof(struct fb_disp) * (fb_mlb_par->disp_num),
		GFP_KERNEL);
	for (i = 0; i < fb_mlb_par->disp_num; i++) {
		const char *pattern_name;

		if (of_property_read_string_index(node,
				"pattern-name",
				i, &pattern_name) == 0) {
			fb_mlb_par->current_pattern = i;
			strncpy(
				fb_mlb_par->disp[i].setting_name,
				pattern_name,
				sizeof(
				fb_mlb_par->disp[i].setting_name));
			ret = get_every_pattern(node,
					pattern_name,
					&fb_mlb_info[index]);
			if (ret != 0) {
				dev_err(
					&pdev->dev,
					"DTS property err. Patten:%d=%s\n"
					, i, fb_mlb_par->disp[i].setting_name);
				break;
			}
		} else {
			dev_err(&pdev->dev, "pattern-name err\n");
			ret = -EINVAL;
		}
	}
#ifndef TEST_DISP
	if (ret == 0) {
		/*Not set yet */
		struct device_node *parent_node =
			of_get_parent(node);
		ret = initialize_ipcu(parent_node);
	}
#endif
	if (ret == 0) {
		const char *device_name;
		fb_mlb_info[index].var.height = 61;/*?*/
		fb_mlb_info[index].var.width = 46;/* ? */
		fb_mlb_info[index].var.grayscale
			= 0;/* 0:color, 1:grayscale, 2:forcc */
		fb_mlb_info[index].var.colorspace
			= 0x41424752;/* RGBA 32bit or 16 bit */
		fb_mlb_info[index].var.activate  = FB_ACTIVATE_NOW;
		fb_mlb_info[index].var.vmode  = FB_VMODE_NONINTERLACED;

		fb_mlb_info[index].var.left_margin = 0;
		fb_mlb_info[index].var.right_margin = 0;
		fb_mlb_info[index].var.upper_margin = 0;
		fb_mlb_info[index].var.lower_margin = 0;
		fb_mlb_info[index].var.sync = 0;
		fb_mlb_info[index].pseudo_palette = NULL;
		of_property_read_string(node, "device-name", &device_name);
		strncpy(fb_mlb_info[index].fix.id,
			device_name,
			sizeof(fb_mlb_info[index].fix.id));
		fb_mlb_info[index].fix.type = FB_TYPE_PACKED_PIXELS;
		fb_mlb_info[index].fix.visual = FB_VISUAL_TRUECOLOR;
		fb_mlb_info[index].fix.ywrapstep = 0;
		fb_mlb_info[index].fix.accel = FB_ACCEL_NONE;
		fb_alloc_cmap(&fb_mlb_info[index].cmap, 256, 0);


		ret = register_framebuffer(&fb_mlb_info[index]);
		if (gr_ipcu.initialize_flag == UNINITIALIZED) {
			/* Only initialize onetime for all GR */
			mutex_init(&gr_ipcu.mlock);
			gr_ipcu.initialize_flag = INITIALIZED;
		}
		mutex_lock(&gr_ipcu.mlock);
		if (fb_mlb_par->block_no == BLOCK_LCD) {
			/* HDMI don't set GR at probe.
			* It set while plug detected 
			*/
			if (fb_mlb_par->current_pattern == 0) {
				/* Only one pattern, so initialize directly */
				if (initialize_disp(&fb_mlb_info[index]) != 0) {
					dev_err(&pdev->dev,
						"DISP initialize fail\n");
				}
			} else {
				switch_gr_setting(
				&fb_mlb_info[index], 0);/* Set default pattern */
			}
		}
		dev_info(&pdev->dev,
			"Loaded M10V OSD display driver\n");
		mutex_unlock(&gr_ipcu.mlock);
	}
	return ret;
}
static int milb_disp_remove(struct platform_device *pdev)
{
	struct fb_info *pfb_mlb_info = platform_get_drvdata(pdev);
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)pfb_mlb_info->par;

	fb_dealloc_cmap(&fb_mlb_info[fb_mlb_par->current_pattern].cmap);
	pr_info("Goodby...\n");
	destroy_workqueue(gr_ipcu.ipcu_wq);
	return 0;
}
/**
* suspend fuction
*/

static int milb_disp_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info *pfb_mlb_info = platform_get_drvdata(pdev);
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)pfb_mlb_info->par;
	int i;
	u32 direction[ENUM_IPCU_CH_END] = {
		SNI_IPCU_DIR_SEND, SNI_IPCU_DIR_RECV
	};

	mutex_lock(&gr_ipcu.mlock);
	if (fb_mlb_par->block_no == BLOCK_LCD) {
		/* HDMI need to be redetected and reconnected.
		 * So it do not suspend/resume
		*/
		/* Get dcore */
		get_dcore(fb_mlb_par);
		get_gr(fb_mlb_par);
	}
	/* Close IPCU channel */
	for (i = 0; i < ENUM_IPCU_CH_END; i++) {
		if ((gr_ipcu.status[i] & ENUM_IPCU_STATUS_OPEN) ==
			ENUM_IPCU_STATUS_OPEN) {
			sni_ipcu_closech(gr_ipcu.ipcu_unit,
				  gr_ipcu.ipcu_ch[i], direction[i]);
			gr_ipcu.status[i] &= (~ENUM_IPCU_STATUS_OPEN);
		}
	}
	mutex_unlock(&gr_ipcu.mlock);
	return 0;
}
/**
* resume fuction
*/
static int milb_disp_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info *pfb_mlb_info = platform_get_drvdata(pdev);
	struct fb_mlb_priv *fb_mlb_par =
		(struct fb_mlb_priv *)pfb_mlb_info->par;

	mutex_lock(&gr_ipcu.mlock);
	if (pm_device_down) {
		if (fb_mlb_par->block_no == BLOCK_LCD) {
			/* HDMI need to be redetected and reconnected.
			 *	So it do not suspend/resume
			*/
			/* Set GR */
			resume_gr(fb_mlb_par);
		}
#ifdef CONFIG_PM_WARP
#endif /* CONFIG_PM_WARP */
	}
	mutex_unlock(&gr_ipcu.mlock);
	return 0;
}
static const struct dev_pm_ops mlb_fb_pm_ops = {
	.suspend = milb_disp_suspend,
	.resume = milb_disp_resume,
	.freeze = milb_disp_suspend,
	.thaw = milb_disp_resume,
	.restore = milb_disp_resume,
};
#define MILB_DISP_PM_OPS (&mlb_fb_pm_ops)
#endif				/* CONFIG_PM_SLEEP */
static const struct of_device_id milb_disp_fb_of_match[] = {
		{.compatible = "socionext,"DEVICE_NAME },
		{}
};
MODULE_DEVICE_TABLE(of, milb_disp_fb_of_match);
static struct platform_driver milb_disp_fb_driver = {
	.probe = milb_disp_probe,
	.remove = milb_disp_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(milb_disp_fb_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm = MILB_DISP_PM_OPS,
#endif
	},
};
module_platform_driver(milb_disp_fb_driver);
MODULE_DESCRIPTION("Milbeaut OSD Framebuffer driver");
MODULE_AUTHOR("sho.ritsugun <sho.ritsugun_s@aa.socionext.com>");
MODULE_LICENSE("GPL");

