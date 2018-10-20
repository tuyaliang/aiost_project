#ifndef MLB_FB_USER_H
#define MLB_FB_USER_H
#define FBIOSWICH_DROW_BUFFER (0x5000) /* Switch drawing buffer by ioctl */
/* Enable/Disable drawing area  */
#define FBIOENABLE_DROW_AREA	(0x5001)
#define FBIOFULL_SETTING	(0x5002)	/* Set full GR  */
#define FBIOSET_TRIGER	(0x5003)	/* Set GR's triger ON/OFF */
/* Switch GR's setting */
#define FBIOSWITCH_GR_SETTING	(0x5004)
/* Set every drawing area's position and size */
#define FBIOSET_DRAWAREA	(0x5005)
/* Get every drawing area's position and size */
#define FBIOGET_DRAWAREA	(0x5006)
#define FBIOFULL_GETTING	(0x5007)	/* Get full GR  */
#define MAX_DRAW_AREA_NUM	(10)
union grrsz {
	unsigned long       word;
	struct {
		unsigned long   rszm	:5;
		unsigned long           :3;
		unsigned long   rszn	:5;
		unsigned long           :3;
		unsigned long           :7;
		unsigned long           :1;
		unsigned long   rszof	:5;
		unsigned long           :3;
	} bit;
};

union grisize {
	unsigned long       word;
	struct {
		unsigned long   ihsize      :16;
		unsigned long   ivsize      :14;
		unsigned long               :2;
	} bit;
};

union grdsta {
	unsigned long       word;
	struct {
		unsigned long   dsh     :16;
		unsigned long   dsv     :14;
		unsigned long           :2;
	} bit;
};

/*  structure of GRSCCTL    (2890_2214h)    */
union grscctl {
	unsigned long       word;
	struct {
		unsigned long   scen    :2;
		unsigned long           :2;
		unsigned long           :1;
		unsigned long           :3;
		unsigned long   idset   :2;
		unsigned long           :2;
		unsigned long           :1;
		unsigned long           :3;
		unsigned long   idm     :2;
		unsigned long           :14;
	} bit;
};
/*  structure of GRIDT  (2890_2200h)    */
union gridt {
	unsigned long       word;
	struct {
		/* Image data format. 0: RGBA8888, 2: RGBA4444 */
		unsigned long   ifmt    :3;
		unsigned long           :1;
		unsigned long   nbt     :2;/* Set it to 0 */
		unsigned long           :2;
		unsigned long           :8;
		unsigned long   cache   :4;/* Set it to 0 */
		unsigned long   prot    :3;/* Set it to 0 */
		unsigned long           :1;
		unsigned long   slvsl   :1;/* Set it to 0 */
		unsigned long   aslvsl  :1;/* Set it to 0 */
		unsigned long           :2;
		unsigned long   ifbtmu  :1;/* Set it to 0 */
		unsigned long           :3;
	} bit;
};

typedef struct {
	/* Every input image area's size */
	union grisize	input_image_size[MAX_DRAW_AREA_NUM];
	/* Every input image area's position */
	union grdsta	input_image_position[MAX_DRAW_AREA_NUM];
	int	enable_show_area[MAX_DRAW_AREA_NUM];
	/* Every input image area's horizonal width */
	unsigned long	input_image_horizonal_bytes[MAX_DRAW_AREA_NUM];

} draw_area;


struct grimage_address {
	unsigned int       address[4];
};
struct fb_disp {
	/* Image data format. 0: RGBA8888, 2: RGBA4444, 5: Ycc+A4 6: Ycc+A8 */
	union gridt		iput_trans_setting;
	union grisize	input_area_size;	/* Input Area Size */
	union grdsta	start_pos;	/* Area Display Start Position */
	union grrsz		horizonal_resize;	/* Horizontal Resize */
	union grrsz		vertical_resize;	/* Vertical Resize */
	/* Every input image area's size */
	union grisize	input_image_size[MAX_DRAW_AREA_NUM];
	/* Every input image area's position */
	union grdsta	input_image_position[MAX_DRAW_AREA_NUM];
	int	enable_show_area[MAX_DRAW_AREA_NUM];
	u32 drawing_buffer_top_address_size[2];/* [0]: Address, [1]:size */
	u32 gri_trg;/* GR's trig*/
	int valid_input_area;	/* Valid input area bit map */
	int current_buffer_index; /* Drawing buffer index */
	u32 valid_buffer_number;
	/* It is same as fb_info's var x/yres_virtual. [0]:x [1]:y
		It always should be same as them. In particularly of switching pattern */
	u32 res_virtual[2];
	u32 byte_num_line;	/* One line's bytes */
	/* [0]: Pan step of X. [1]:Pan step of Y.*/
	/* They are as same as xpanstep/ypanstep */
	u32 pan_step[2];
	char setting_name[10];

	/* Every input image area's horizonal width */
	unsigned long	input_image_horizonal_bytes[MAX_DRAW_AREA_NUM];
#ifdef SUPORT_YUV_FORMAT
	/* A data's  horizonal width. It is valid at YCC+A4/A8 only */
	unsigned long	A_data_horizonal_bytes[MAX_DRAW_AREA_NUM];
	/* Every input A data's physical address  */
	struct grimage_address	input_A_data_ph_address[MAX_DRAW_AREA_NUM];
	/* Every input A data's address  */
	struct grimage_address	input_A_data_address[MAX_DRAW_AREA_NUM];
#endif
	/* Every input image's physical address  */
	struct grimage_address	input_image_ph_address[MAX_DRAW_AREA_NUM];
};


#endif
