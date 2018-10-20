/*
 * drivers/video/fbdev/socionext/disp8m-fb.c
 *
 * Copyright (C) 2013 - 2015 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 *
 * This always instantiates an fdb child representing the framebuffer, if the
 * "simple" DT attribute is nonzero then it also registers as a simple Linux
 * framebuffer.  If !simple, the fdb child may be adopted by, eg, fdb-drm
 * driver and exposed that way.
 */

#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <video/disp8m-fb.h>
#include <video/fdb.h>

enum disp8m_fbs {
	D8MFB_OSD0,
	D8MFB_OSD1,
	D8MFB_MAIN,

	D8MFB_COUNT_FBS
};

/* there is one CRTC here, so one timings, vsync etc */
struct disp8m_par;

struct disp8m_priv {
	struct f_fdb_child *source; /* our owner, eg, drm driver, if any */
	int id;

	int irq[2];
	char irq_name[2][64];
	struct device *dev;

	/* CRTC register access */
	phys_addr_t base_pa;
	void __iomem *base;

	struct mutex lock; /* serialize api accesses */
	int clocks_enabled;
	struct clk *clocks[10];

	struct completion vsync_completion;
	struct completion flip_completion;

	u32 frame_index;

	unsigned int mode_valid:1;
	unsigned int hdmi:1;
	unsigned int last_hdmi:1;
	struct fdb_video_timings last_timings;

	void (*flip_cb)(void *);
	void *flip_cb_arg;
	struct disp8m_par *flip_par;

	struct fb_info *info[D8MFB_COUNT_FBS];
};

/* but there are multiple overlays */

struct disp8m_par {
	struct disp8m_priv *priv;
	struct f_fdb_child fdb_child;

	struct fb_info *info;
	u32 pseudo_palette[16];
	dma_addr_t fbpaddr;
	void *fb_va;
	u32 framesize;
	unsigned int pixel_format; /* fourcc */
	int bits_per_pixel;

	enum disp8m_fbs fbs; /* which fb / overlay are we? */
};

#define f_par_from_fdb_child(_fc) (_fc->priv)
#define f_priv_from_fdb_child(_fc) (((struct disp8m_par *)_fc->priv)->priv)

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)

static const struct fdb_format formats[][2] = {
	[D8MFB_OSD0] = {
		{ DRM_FORMAT_ARGB8888, { { 4, 1 } }, false },
		{ DRM_FORMAT_XRGB8888, { { 4, 1 } }, false },
	},
	[D8MFB_OSD1] = {
		{ DRM_FORMAT_ARGB8888, { { 4, 1 } }, false },
		{ DRM_FORMAT_XRGB8888, { { 4, 1 } }, false },
	},
	[D8MFB_MAIN] = {
		{ DRM_FORMAT_YVYU, { { 4, 1 } }, true },
		{ DRM_FORMAT_YVYU, { { 4, 1 } }, true },
	},
};

static void _disp8m_fb_set_paddr(struct disp8m_par *par, dma_addr_t pa)
{
	struct disp8m_priv *priv = par->priv;

//	dev_info(priv->dev, "%s: 0x%x (fb = %d)\n", __func__, pa, par->fbs);

	switch (par->fbs) {
	case D8MFB_OSD1:
		writel(pa, priv->base +
		       DISP8M_APB_OSD_GRSAO_OFS + DISP8M_APB_OSD1_OFFSET);
		writel(pa, priv->base +
		       DISP8M_APB_OSD_GRSA_OFS + DISP8M_APB_OSD1_OFFSET);
		break;
	case D8MFB_OSD0:
		writel(pa, priv->base + DISP8M_APB_OSD_GRSAO_OFS);
		writel(pa, priv->base + DISP8M_APB_OSD_GRSA_OFS);
		break;
	case D8MFB_MAIN:
		writel(pa, priv->base + DISP8M_APB_MAIN_LYSA0_OFS);
		writel(pa, priv->base + DISP8M_APB_MAIN_LCSA0_OFS);
		break;
	default:
		break;
	}
}

/*
 * under some conditions it's needed to tell Linux one hactive / fb width
 * for memory and something different for the panel / timings.  This gets
 * the panel / timings or if not given, returns the main hactive value.
 */
static u32 effective_hactive_px(struct fdb_video_timings *timings)
{
	if (!timings->dt.hactive.max || (int)timings->dt.hactive.max < 0)
		return timings->dt.hactive.typ;

	return timings->dt.hactive.max;
}

static void
_disp8m_fb_raster_timing_reconfig(struct disp8m_par *par,
			    struct fdb_video_timings *timings)
{
	struct disp8m_priv *priv = par->priv;
	void __iomem *base = priv->base;
	struct display_timing *dt = &timings->dt;
	int fields = (!!(dt->flags & DISPLAY_FLAGS_INTERLACED)) + 1;
	int clk_per_px = 1;

	/* because we're sending 24-bit px down an 8-bit pipe */
	if (!priv->hdmi)
		clk_per_px = 3;

	/*
	 * Sync polarities
	 */ 

	if (priv->hdmi)
		writel(0/* BIT(12) | BIT(8) | */
			/* h + v data enables 0 = positive */
		       /* BIT(4) | BIT(0) */ /* force hdmi syncs positive */, 
		       base + DISP8M_APB_OUT_POLSEL_OFS);
	else
		writel(/*BIT(12) | BIT(8) | */
			/* h + v data enables 0 = positive */
		       ((!(timings->dt.flags & DISPLAY_FLAGS_HSYNC_HIGH)) << 4) |
		       ((!(timings->dt.flags & DISPLAY_FLAGS_VSYNC_HIGH)) << 0),
		       base + DISP8M_APB_OUT_POLSEL_OFS);

	/*
	 * Vertical timings
	 */

	/* VCYC ( >= vblkx + ovsize + 1) */
	writel((BIT(31) * (fields == 2)) |
	       (((dt->vactive.typ + dt->vfront_porch.typ +
                 dt->vsync_len.typ + dt->vback_porch.typ) / fields) << 0) |
	       ((((dt->vactive.typ + dt->vfront_porch.typ +
                 dt->vsync_len.typ + dt->vback_porch.typ) / fields) +
		 (fields - 1)) << 16),
				base + DISP8M_APB_OUT_VCYC_OFS);
	/* OVSIZE - Output size, vertical */
	writel(dt->vactive.typ, base + DISP8M_APB_OUT_OVSIZE_OFS);

	/* VOVPW - VSYNC width */
	writel(BIT(DISP8M_APB_OUT_OVPW__OVPWU__SHIFT) | /* counted in lines */
	       dt->vsync_len.typ / fields, base + DISP8M_APB_OUT_OVPW_OFS);

	/* VBLK - VBLANK period */
	writel(((((dt->vsync_len.typ + dt->vback_porch.typ) / fields) +
		(fields - 1)) << 8) |
	       (((dt->vsync_len.typ + dt->vback_porch.typ) / fields) << 0),
					base + DISP8M_APB_OUT_VBLK_OFS);
	/* VDLY - VSYNC holdoff */
	writel(0, base + DISP8M_APB_OUT_VDLY_OFS);


	/*
	 * Horizontal timings
	 */

	/* HCYC (hcyc >= HBLK + hactive + 8, and, hcyc >= hactive + 28) */
	writel((effective_hactive_px(timings) * clk_per_px) +
	       dt->hfront_porch.typ +
	       dt->hsync_len.typ +
	       dt->hback_porch.typ, base + DISP8M_APB_OUT_HCYC_OFS);

	/* OHSIZE - Output size, horizontal */
	writel(effective_hactive_px(timings), base + DISP8M_APB_OUT_OHSIZE_OFS);

	dev_info(priv->dev, "%s: %d max %d eff %d %d max %d %d %d\n", __func__,
		dt->hactive.typ, dt->hactive.max, effective_hactive_px(timings),dt->hfront_porch.typ,
		dt->vactive.max,
		dt->hsync_len.typ, dt->hback_porch.typ);

	/* HPW - HSYNC width */
	writel(dt->hsync_len.typ, base + DISP8M_APB_OUT_HPW_OFS);

	/* HBLK - HBLANK period  - 16 <= hblk <= (hcyc - hactive - 8) */
	writel(((dt->hsync_len.typ + dt->hback_porch.typ) << 0),
					base + DISP8M_APB_OUT_HBLK_OFS);

	/* HDLY - HSYNC holdoff */
	writel(0, base + DISP8M_APB_OUT_HDLY_OFS);

	_disp8m_fb_set_paddr(par, par->fbpaddr);

	//Write Interlace or progressive
	writel((!!(dt->flags & DISPLAY_FLAGS_INTERLACED)),
		base + DISP8M_APB_OUT_TSL_OFS);
}

static void
_disp8m_fb_set_timings(struct disp8m_par *par,
		       struct fdb_video_timings *timings)
{
	struct disp8m_priv *priv= par->priv;
	void __iomem *base = priv->base;
	struct display_timing *dt  = &timings->dt;
	int pixeldata_hdmi_routing = BIT(8);
	unsigned int osd_vertical_size=dt->vactive.typ;
	int fields = (!!(dt->flags & DISPLAY_FLAGS_INTERLACED)) + 1;
	int n;

#define F(y,b,r) { y, y + (b - 128), y + (r - 128) }

#if 0
	static const u8 ycbcr_bars[][3] = {
		/* Y   Cb   Cr */
		{ 235, 128, 128 }, /* white */
		{ 210, 16,  146 }, /* yellow */
		{ 170, 166, 16  }, /* cyan */
		{ 145,  54,  34 }, /* green */
		{ 107, 202, 221 }, /* magenta */
		{  82,  90, 240 }, /* red */
		{  41, 240, 110 }, /* blue */
		{  16, 128, 128 }, /* black */
	};
#else

	/* colourbars are RGB ^^ */

	static const u8 ycbcr_bars[][3] = {
		/* Y   Cb   Cr */
		{ 255, 255, 255 }, /* white */
		{ 255, 255, 0 }, /* yellow */
		{ 0, 255, 255  }, /* cyan */
		{ 0, 255, 0 }, /* green */
		{ 255, 0, 255 }, /* magenta */
		{ 255, 0, 0 }, /* red */
		{ 0, 0, 255 }, /* blue */
		{ 0, 0, 0 }, /* black */
	};
#endif
	/*
	 * MAIN Data input unit
	 */	

	/* reset the main input unit... */
	writel(BIT(DISP8M_APB_MAIN_LRST__SR__SHIFT),
	       base + DISP8M_APB_MAIN_LRST_OFS);
	/* ... the output unit... */
	writel(BIT(0), base + DISP8M_APB_OUT_RESET_OFS);
	/* ... the OSD0 unit */
	writel(BIT(0), base + DISP8M_APB_OSD_GRRST_OFS);
	/* ... the OSD1 unit */
	writel(BIT(0), base + DISP8M_APB_OSD_GRRST_OFS + DISP8M_APB_OSD1_OFFSET);

	udelay(1);
	writel(0x333, base + DISP8M_APB_MAIN_LTBLASET_OFS);

	/* select output HEAD... has to be done in reset + clocks off */

	writel(0, base + DISP8M_APB_OUT_IFS_OFS);

	if (!priv->hdmi) {
		/* select DPI head */
		writel(1 << 0, base + DISP8M_APB_OUT_IFS_OFS);
	} else {
		/* select the HDMI head */
		writel(2 << 0, base + DISP8M_APB_OUT_IFS_OFS);
		pixeldata_hdmi_routing = BIT(9) | BIT(0);
	}

	/* let the main input unit out of reset */
	writel(0, base + DISP8M_APB_MAIN_LRST_OFS);
	udelay(1);

	/* set YCrCb422 format */
	writel((2 << 0) |
	       (6 << 16) |
	       (3 << 20), base + DISP8M_APB_MAIN_LIDT_OFS);

	writel(((timings->dt.vactive.typ / fields) << 16) |
	       ((timings->dt.hactive.typ) << 0),
				base + DISP8M_APB_MAIN_LISIZE_OFS); 

	/* this is actually the stride in pixels */
	writel(timings->stride_px, base + DISP8M_APB_MAIN_LYHGA_OFS);
	writel(timings->stride_px, base + DISP8M_APB_MAIN_LCHGA_OFS);

	writel(dt->hactive.typ != effective_hactive_px(timings),
		base + DISP8M_APB_MAIN_LHRSZ0_OFS);
	writel((0  << 24) | (1 << 0) |
		((dt->hactive.typ / effective_hactive_px(timings)) << 8),
		base + DISP8M_APB_MAIN_LHRSZ1_OFS);

#if 0
	/* turn on the main input unit */
	writel(1, base + DISP8M_APB_MAIN_LDISPEN_OFS);
#else
	/* turn off the main input unit */
	writel(0, base + DISP8M_APB_MAIN_LDISPEN_OFS);
#endif

	/*
	 * OSD 0 + 1
	 */

	for (n = 0; n < DISP8M_APB_OSD1_OFFSET; n += DISP8M_APB_OSD1_OFFSET) {

		/* reset the OSD0 input unit */
		writel(0, base + n + DISP8M_APB_OSD_GRRST_OFS);
		udelay(1);

		writel(BIT(0), base + n +DISP8M_APB_OSD_GRRPGCTL_OFS);

		/* OSD0: INPUT transfer settings - 0 = RGBA8888, 1 = YCbCr422 */
		writel((0 << 24) |
		       (3 << 20) |
		       (0x6 << 16) |
		       (0 << 4) |
		       (0 << 0),
			base + n + DISP8M_APB_OSD_GRIDT_OFS);

		/* OSD0 INPUT Area size */
		writel((osd_vertical_size << 16) | dt->hactive.typ,
		       base + n + DISP8M_APB_OSD_GRTISIZE_OFS);

		/* OSD0: Area size 0 */
		writel((osd_vertical_size << 16) | dt->hactive.typ,
		       base + n + DISP8M_APB_OSD_GRISIZE_OFS);

		/* OSD0: Stride in BYTES */
		writel(timings->stride_px * 4,
		       base + n + DISP8M_APB_OSD_GRHGA_OFS);

		writel(dt->hactive.typ != effective_hactive_px(timings),
			base + DISP8M_APB_OSD_GRHRSZ0_OFS);
		writel((0  << 24) | (1 << 0) |
			((dt->hactive.typ / effective_hactive_px(timings)) << 8),
			base + DISP8M_APB_OSD_GRHRSZ1_OFS);

		/* RGB888 pixel layout */
		writel(0x03000102, base + n + DISP8M_APB_OSD_GRIPO_OFS);

		/* OSD0: enable display only area 0 */
		writel(BIT(0), base + n + DISP8M_APB_OSD_GRAREN_OFS);

		/* OSD0: enable AXI error recovery */
		writel(BIT(0), base + n + DISP8M_APB_OSD_GRERCV_OFS);

		/* default alpha level */
		writel(0xff, base + n + DISP8M_APB_OSD_GRALP_OFS);
	}

	/*
	 * Data output unit
	 */

	{ void *p = ioremap(0x18000000, 0x2000); writel(0, p + 4); iounmap(p); }

	/* let the output unit out of reset */
	writel(0 << 0, base + DISP8M_APB_OUT_RESET_OFS);
	udelay(1);

	/* allow shadow updates on VSYNC */
	writel(BIT(0), base + DISP8M_APB_OUT_RPGCTL_OFS);

	if (priv->hdmi)
		/* Data output mode (YCbCr422 16-bit) */
		writel(BIT(16) | /* MSB-justify the output data for HDMI */
		       0x41, /* YCbCr422 16-bit */
		       base + DISP8M_APB_OUT_DOMD_OFS);
	else
		/* Data output mode (rgb888 / 8-bit v2) */
		writel(5, base + DISP8M_APB_OUT_DOMD_OFS);

	/* enable the right clock + data routing */
	writel(0,  base + DISP8M_APB_OUT_TOCTL_OFS);
	writel(pixeldata_hdmi_routing & BIT(0),
				base + DISP8M_APB_OUT_TOCTL_OFS);
	writel(pixeldata_hdmi_routing,
				base + DISP8M_APB_OUT_TOCTL_OFS);

	/* enable vsync, Input unit Error, Main AXI error */
	writel(BIT(0) | BIT(4) | BIT(5) | BIT(8) | BIT(12) | BIT(16),
		base + DISP8M_APB_OUT_INTE_OFS);

	/* enable vsync irq on both fields */
	writel(BIT(1) | BIT(0), base + DISP8M_APB_OUT_INTC_OFS);

	/* Start off in forced mode, first VSYNC irq will unforce us */
	writel(BIT(0), base + DISP8M_APB_OUT_FDOEN_OFS);

	/* forced colour */
#if 0
	writel(0xffffffff, base + DISP8M_APB_OUT_FODATA_OFS);
	writel(0xffffffff, base + DISP8M_APB_OUT_BLANKDT_OFS);
	writel(0xffff, base + DISP8M_APB_OUT_BLANKDT_OFS + 4);
#else
	writel(0, base + DISP8M_APB_OUT_FODATA_OFS);
	writel(0, base + DISP8M_APB_OUT_BLANKDT_OFS);
	writel(0, base + DISP8M_APB_OUT_BLANKDT_OFS + 4);
#endif
	writel(1, base + DISP8M_APB_OUT_HRFCTL_OFS);

	/* colourbars */
	writel(effective_hactive_px(timings) / 8,
		base + DISP8M_APB_OUT_CLBHSIZE_OFS);
	for (n = 0; n < 8; n++)
		writel((ycbcr_bars[n][2] << 16) | /* B / Cr */
		       (ycbcr_bars[n][1] << 8) |  /* G / Cb */
		       ycbcr_bars[n][0],	  /* R / Y */
		       base + DISP8M_APB_OUT_CLBDT_OFS + (n << 2));

	if (priv->hdmi) {
		/*
		 * configure output sequencing for YCbCr422
		 * 
		 *    b1b0 = "High order 8 bits of output order 1"
		 *    b5b4 = "Low order 8 bits of output order 1"
		 *    b9b8 = "High order 8 bits of output order 2"
		 *  b13b12 = "Low order 8 bits of output order 2"
		 *
		 *  Coding 00 = Y0, 01 = Cb, 10 = Cr, 11 = Y1
		 *
		 *  HDMI IP requires packing MSB-aligned
		 *    px[31..28] = Chroma
		 *    px[27..20] = Luma
		 */
		writel((1 << 0) | /* High 1 = Cb */
		       (0 << 4) | /* Low 1  = Y0 */
		       (2 << 8) | /* High 2 = Cr */
		       (3 << 12), /* Low 2  = Y1 */
		       base + DISP8M_APB_OUT_DOCTL0_OFS);
		writel(0, base + DISP8M_APB_OUT_DOCTL1_OFS);
		/* 1 = issue colour difference */
		writel(0, base + DISP8M_APB_OUT_DOCTL2_OFS);
	} else {
#if 0 /* true scanout order */
		writel((1 <<  0) | /* R odd line */
		       (2 <<  4) | /* G */
		       (0 <<  8) | /* B */
		       (1 << 12) | /* G even line */
		       (2 << 16) | /* B */
		       (0 << 20),  /* R */
		       base + DISP8M_APB_OUT_DOCTL0_OFS);
#else
	/* flipped scanout order */
		writel((2 <<  0) | /* R odd line */
		       (0 <<  4) | /* G */
		       (1 <<  8) | /* B */
		       (2 << 12) | /* G even line */
		       (0 << 16) | /* B */
		       (1 << 20),  /* R */
		       base + DISP8M_APB_OUT_DOCTL0_OFS);
#endif
		writel(/*BIT(12) |*/ BIT(8) /*| BIT(4)*/,
			base + DISP8M_APB_OUT_DOCTL1_OFS);
		writel(0, base + DISP8M_APB_OUT_DOCTL2_OFS);
	}
	priv->last_timings = *timings;
	priv->last_hdmi = priv->hdmi;
	priv->mode_valid = 1;

	dev_info(priv->dev, "%s: (%d x %d, flags=0x%x)\n",
		__func__, timings->dt.hactive.typ, timings->dt.vactive.typ,
		timings->dt.flags);

	_disp8m_fb_raster_timing_reconfig(par, timings);

	clk_set_rate(priv->clocks[0], dt->pixelclock.typ * 1000);
	dev_info(priv->dev, "lcdclk_gated =%lld (set to %lld)\n",
				(u64)clk_get_rate(priv->clocks[0]),
				(u64)dt->pixelclock.typ * 1000);
	if (dt->pixelclock.typ == 74250) {
		clk_set_parent(clk_get_parent(priv->clocks[0]),
			       priv->clocks[5]);
		clk_set_parent(clk_get_parent(priv->clocks[1]),
			       priv->clocks[5]);
	}
	if (dt->pixelclock.typ == 27000) {
		clk_set_parent(clk_get_parent(priv->clocks[0]),
			       priv->clocks[6]);
		clk_set_parent(clk_get_parent(priv->clocks[1]),
			       priv->clocks[6]);
	}
	dev_info(priv->dev, "hifclk_gated =%lld (set to %lld)\n",
				(u64)clk_get_rate(priv->clocks[1]),
				(u64)dt->pixelclock.typ * 1000);

	dev_info(priv->dev, "%s: %d x %d, rate = %u (current = %lu)\n",
			__func__, dt->hactive.typ, dt->vactive.typ,
		      dt->pixelclock.typ * 1000, clk_get_rate(priv->clocks[0]));

	/* enable loading shadows (on next VSYNC) */
	writel(BIT(0), base + DISP8M_APB_OUT_RPGEN_OFS);

//	/* start the MAIN input unit */
//	writel(LTRG_START << 0, base + DISP8M_APB_MAIN_LTRG_OFS);

	/* start the OSD0 input unit */
	writel(LTRG_START << 0, base + DISP8M_APB_OSD_GRTRG_OFS);

//	/* start the OSD1 input unit */
//	writel(LTRG_START << 0, base + DISP8M_APB_OSD1_OFFSET +
//				DISP8M_APB_OSD_GRTRG_OFS);

	/* start the output unit */
	writel(LTRG_START << 0, base + DISP8M_APB_OUT_TRG_OFS);
}


static void
disp8m_fb_set_timings(struct f_fdb_child *fdb_child,
		      struct fdb_video_timings *timings)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);
	struct disp8m_priv *priv = par->priv;

	mutex_lock(&priv->lock);
	pm_runtime_get_sync(priv->dev);

	if (priv->hdmi == priv->last_hdmi &&
	    timings->dt.hactive.typ == priv->last_timings.dt.hactive.typ &&
	    timings->dt.vactive.typ == priv->last_timings.dt.vactive.typ)
		timings = &priv->last_timings;

	_disp8m_fb_set_timings(par, timings);

	pm_runtime_put(priv->dev);
	mutex_unlock(&priv->lock);
}

static void
disp8m_fb_fdb_set_paddr(struct f_fdb_child *fdb_child, dma_addr_t pa)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);

	mutex_lock(&par->priv->lock);

	_disp8m_fb_set_paddr(par, pa);
	par->fbpaddr = pa;

	mutex_unlock(&par->priv->lock);

}

static int
disp8m_fb_fdb_enable(struct f_fdb_child *fdb_child)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);

	pm_runtime_get_sync(par->priv->dev);

	mutex_lock(&par->priv->lock);

//	writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1),
//	       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

	mutex_unlock(&par->priv->lock);
	dev_dbg(par->priv->dev, "%s ------ +++++\n", __func__);

	return 0;
}

static void
disp8m_fb_fdb_disable(struct f_fdb_child *fdb_child)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);
#if 0
	if (__raw_readl(par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS) &
	    fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1)) {
		init_completion(&par->dispeng_seq_completion);
		mutex_lock(&par->lock);
		writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 0),
		       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

		mutex_unlock(&par->lock);

		if (wait_for_completion_timeout(&par->dispeng_seq_completion,
						msecs_to_jiffies(60)) <= 0)
			dev_info(priv->dev, "%s: dispeng stop timeout\n",
				 __func__);
	}
#endif
	pm_runtime_put(par->priv->dev);
}


static int
disp8m_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
		    unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno >= 16)
		return -EINVAL;

	if (info->var.grayscale)
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;

	red = CNVT_TOHW(red, info->var.red.length);
	green = CNVT_TOHW(green, info->var.green.length);
	blue = CNVT_TOHW(blue, info->var.blue.length);
	transp = CNVT_TOHW(transp, info->var.transp.length);
#if 0
	((u32 *)(info->pseudo_palette))[regno] =
			(red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
					(transp << info->var.transp.offset);
#endif
	return 0;
}

static int
disp8m_fb_blank(int blank_mode, struct fb_info *info)
{
	struct disp8m_par *par = info->par;

	dev_dbg(info->dev, "disp8m_fb_blank: %d\n", blank_mode);

	if (blank_mode == FB_BLANK_UNBLANK)
		par->fdb_child.ops->enable(&par->fdb_child);
	else
		par->fdb_child.ops->disable(&par->fdb_child);

	return 0;
}

void
disp8m_fb_get_timings(struct f_fdb_child *fdb_child,
		      struct fdb_video_timings *timings)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);
	struct fb_var_screeninfo var;
	const char *mode;
	int ret;

	mutex_lock(&par->priv->lock);

	if (of_property_read_string(fdb_child->dev->of_node, "mode", &mode)) {
		dev_err(fdb_child->dev, "Missing mode\n");
		goto bail;
	}

	dev_dbg(fdb_child->dev, "fdb_fb_get_timings: mode = %s\n", mode);

	ret = fdb_get_of_var(mode, &var);
	if (ret) {
		dev_err(fdb_child->dev, "Failed to get of var for %s\n", mode);
		goto bail;
	}

	fdb_var_to_video_timings(&var, timings);
	dev_dbg(fdb_child->dev, "disp8m_fb_get_timings result ->\n");
	fdb_dump_video_timings(timings);

bail:
	mutex_unlock(&par->priv->lock);
}

static void
disp8m_fb_set_gamma(struct f_fdb_child *fdb_child,
		    u16 red, u16 green, u16 blue, int regno)
{
}
static void
disp8m_fb_get_gamma(struct f_fdb_child *fdb_child,
		    u16 *red, u16 *green, u16 *blue, int regno)
{
}

static int
disp8m_fb_check_timings(struct f_fdb_child *fdb_child,
			struct fdb_video_timings *timings)
{
	return 0;
}

static const struct fdb_format *
disp8m_fb_get_fdb_formats(struct f_fdb_child *fdb_child, int *count)
{
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);

	*count = ARRAY_SIZE(formats[0]);
	return &formats[par->fbs][0];
}

int
disp8m_fb_bind_to_source(struct f_fdb_child *fdb_child,
			 struct f_fdb_child *fdb_bound)
{
	struct disp8m_priv *priv = f_priv_from_fdb_child(fdb_child);

	priv->source = fdb_bound;

	return 0;
}

#if defined(CONFIG_DEBUG_FS)

struct dumps {
	const char *name;
	unsigned int start;
	unsigned int len;
};

struct dumps dumps[] = {
	{ "Main-LRST",		0,	4 },
	{ "Main-LTRG",		0x100,	4 },
	{ "Main-LRPCCR",	0x110,	4 },
	{ "Main-LIDT",		0x200,	4 },
	{ "Main-LISIZE",	0x204,	4 },
	{ "Main-LYSA0",		0x210,	4 },
	{ "Main-LYSA1",		0x214,	4 },
	{ "Main-LYSA2",		0x218,	4 },
	{ "Main-LYSA3",		0x21c,	4 },
	{ "Main-LCSA0",		0x220,	4 },
	{ "Main-LCSA1",		0x224,	4 },
	{ "Main-LCSA2",		0x228,	4 },
	{ "Main-LCSA3",		0x22c,	4 },

	{ "Main-LYHGA",		0x240,	4 },
	{ "Main-LCHGA",		0x250,	4 },
	{ "Main-LIBCTL",	0x260,	4 },
	{ "Main-LETCV",		0x264,	4 },
	{ "GR0-GRTRG",		0x2000+0x100,	4 },
	{ "GR0-GRIDT",		0x2000+0x200,	4 },
	{ "GR0-GRTISIZE",	0x2000+0x204,	4 },
	{ "GR0-GRTDSTA",	0x2000+0x208,	4 },
	{ "GR0-GRIPO",		0x2000+0x210,	4 },
	{ "GR0-GRSCCTL",	0x2000+0x214,	4 },
	{ "GR0-GRISIZE",	0x2000+0x400,	4 },
	{ "GR0-GRSA0",		0x2000+0x440,	4 },

	{ "GR0-GRHGA",		0x2000+0x4C0,	4 },
	{ "GR0-GRDSTA",		0x2000+0x500,	4 },
	{ "GR0-RAREN",		0x2000+0x540,	4 },
	{ "GR0-GRHRSZ0",	0x2000+0x610,	4 },
	{ "GR0-GRHRSZ1",	0x2000+0x614,	4 },
	{ "GR0-GRVRSZ",		0x2000+0x618,	4 },
	
	{ "GR1-GRTRG",		0x3000+0x100,	4 },
	{ "GR1-GRIDT",		0x3000+0x200,	4 },
	{ "GR1-GRTISIZE",	0x3000+0x204,	4 },
	{ "GR1-GRTDSTA",	0x3000+0x208,	4 },
	{ "GR1-GRIPO",		0x3000+0x210,	4 },
	{ "GR1-GRSCCTL",	0x3000+0x214,	4 },
	{ "GR1-GRISIZE",	0x3000+0x400,	4 },
	{ "GR1-GRSA0",		0x3000+0x440,	4 },

	{ "GR1-GRHGA",		0x3000+0x4C0,	4 },
	{ "GR1-GRDSTA",		0x3000+0x500,	4 },
	{ "GR1-RAREN",		0x3000+0x540,	4 },
	{ "GR1-GRHRSZ0",	0x3000+0x610,	4 },
	{ "GR1-GRHRSZ1",	0x3000+0x614,	4 },
	{ "GR1-GRVRSZ",		0x3000+0x618,	4 },

	{ "DATA-TRG",		0x1000+0x200,	4 },
	{ "DATA-TOCTL",		0x1000+0x204,	4 },
	{ "DATA-RPGCTL",	0x1000+0x220,	4 },
	{ "DATA-POLSEL",	0x1000+0x300,	4 },
	{ "DATA-TSL",		0x1000+0x304,	4 },
	{ "DATA-VCYC",		0x1000+0x308,	4 },
	{ "DATA-HCYC",		0x1000+0x30C,	4 },
	{ "DATA-OVPW",		0x1000+0x310,	4 },
	{ "DATA-HPW",		0x1000+0x314,	4 },
	{ "DATA-VBLK",		0x1000+0x318,	4 },
	{ "DATA-HBLK",		0x1000+0x31C,	4 },
	{ "DATA-VDLY",		0x1000+0x320,	4 },
	{ "DATA-HDLY",		0x1000+0x324,	4 },
	{ "DATA-OVSIZE",	0x1000+0x328,	4 },
	{ "DATA-OHSIZE",	0x1000+0x32C,	4 },
	{ "DATA-DOMD",		0x1000+0x400,	4 },
	{ "DATA-CLBHSIZE",	0x1000+0x430,	4 },
	{ "DATA-VRFCTL",	0x1000+0x330,	4 },
	{ "DATA-HRFCTL",	0x1000+0x338,	4 },
	

};

static int
disp8m_fdb_debugfs(struct f_fdb_child *fdb_child, struct seq_file *m, int type)
{
	struct disp8m_priv *priv = f_priv_from_fdb_child(fdb_child);
	int i, n;	

	switch (type) {
	case FDB_DEBUGFS_DUMP:
		pm_runtime_get_sync(priv->dev);
		seq_printf(m, "struct dump dump_disp8m_0x%lx[] = {\n",
			   (unsigned long)priv->base_pa);
		for (i = 0; i < ARRAY_SIZE(dumps); i++)
			for (n = 0; n < dumps[i].len; n += 4)
				seq_printf(m,
					   "/* %s: +0x%04x */ "
					   "{ 0x%lx, 0x%08X },\n",
					   dumps[i].name,
					   dumps[i].start + n,
					   (unsigned long)priv->base_pa +
					   dumps[i].start + n,
					   __raw_readl(priv->base +
						       dumps[i].start + n));
		seq_puts(m, "};\n");
		pm_runtime_put(priv->dev);
		break;
	}
	return 0;
}
#endif
static void disp8m_fb_flip_next_vsync(struct f_fdb_child *fdb_child,
				       dma_addr_t pa, void (cb)(void *),
				       void *cb_arg)
{
	struct disp8m_priv *priv = f_priv_from_fdb_child(fdb_child);
	struct disp8m_par *par = f_par_from_fdb_child(fdb_child);

	mutex_lock(&priv->lock);
	priv->flip_cb = cb;
	priv->flip_cb_arg = cb_arg;
	priv->flip_par = par;
	_disp8m_fb_set_paddr(par, pa);
	priv->flip_par->fbpaddr = pa;
	mutex_unlock(&priv->lock);

	init_completion(&priv->flip_completion);
	wait_for_completion_timeout(&priv->flip_completion,
						  msecs_to_jiffies(50));
}

static void disp8m_hdmi_presence_update(struct f_fdb_child *fdb_child, bool present)
{
	struct disp8m_priv *priv = f_priv_from_fdb_child(fdb_child);

	priv->hdmi = present;
}

struct f_fdb_ops disp8m_fb_fdb_ops = {
	.enable = disp8m_fb_fdb_enable,
	.disable = disp8m_fb_fdb_disable,
	.set_paddr = disp8m_fb_fdb_set_paddr,
	.get_timings = disp8m_fb_get_timings,
	.set_timings = disp8m_fb_set_timings,
	.check_timings = disp8m_fb_check_timings,
	.get_gamma = disp8m_fb_get_gamma,
	.set_gamma = disp8m_fb_set_gamma,
	.get_fdb_formats = disp8m_fb_get_fdb_formats,
	.bind_to_source = disp8m_fb_bind_to_source,
	.hdmi_presence_update = disp8m_hdmi_presence_update,
	.flip_next_vsync = disp8m_fb_flip_next_vsync,
#if defined(CONFIG_DEBUG_FS)
	.fdb_debugfs = disp8m_fdb_debugfs,
#endif
};

static struct fb_fix_screeninfo disp8m_fb_fix = {
	.id =		"disp8m_fb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,
};

static int
disp8m_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct disp8m_par *par = info->par;
	struct disp8m_priv *priv = par->priv;
	int ret;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (!completion_done(&priv->vsync_completion))
			return 0;

		init_completion(&priv->vsync_completion);
		ret = wait_for_completion_timeout(&priv->vsync_completion,
						  msecs_to_jiffies(50));
		if (ret <= 0) {
			dev_err(priv->dev, "WAITFORVSYNC timeout\n");
			return -ETIME;
		}
		return 0;
	}

	return -ENOTTY;
}

static int
disp8m_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct disp8m_par *par = info->par;
	struct disp8m_priv *priv = par->priv;
	int bytes_pp = (var->bits_per_pixel + 7) / 8;
	dma_addr_t paddr;
pr_err("%s\n", __func__);
	if (!completion_done(&priv->vsync_completion)) {
		dev_err(priv->dev, "flip overrun\n");
		return 0;
	}

	paddr = par->fbpaddr + (var->xoffset * bytes_pp) +
				(var->yoffset * var->xres_virtual * bytes_pp);
	disp8m_fb_fdb_set_paddr(&par->fdb_child, par->fbpaddr);

	return 0;
}

static struct fb_ops disp8m_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= disp8m_fb_setcolreg,
	.fb_blank	= disp8m_fb_blank,
	.fb_pan_display = disp8m_fb_pan_display,
	.fb_ioctl	= disp8m_fb_ioctl,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/*
 * runtime_pm: the fact that these irqs were enabled to get this interrupt
 * means that it is OK to access the hardware
 */

static irqreturn_t disp8m_isr(int irq, void *arg)
{
	struct disp8m_priv *priv = arg;
	void __iomem *base = priv->base;
	u32 status;

	status = readl(base + DISP8M_APB_OUT_INTF_OFS);
	writel(status, base + DISP8M_APB_OUT_INTF_OFS);

	if (status & BIT(16)) {
		priv->frame_index++;

		/* disable any forcing */
		writel(0, base + DISP8M_APB_OUT_FDOEN_OFS);
		if (priv->source && priv->source->ops->sync)
			priv->source->ops->sync(priv->source, priv->id,
							priv->frame_index);
		/* do any pending flip */
		if (priv->flip_cb) {
			priv->flip_cb(priv->flip_cb_arg);
			priv->flip_cb = NULL;
			complete(&priv->flip_completion);
		}

		complete(&priv->vsync_completion);
	}
	if (status & BIT(0))
		dev_info(priv->dev, "Main Input Data Transfer ERROR\n");

	if (status & BIT(4))
		dev_info(priv->dev, "OSD0 Input Data Transfer ERROR 0x%x\n",
			 status);

	if (status & BIT(5))
		dev_info(priv->dev, "OSD1 Input Unit Data Transfer ERROR\n");

	if (status & BIT(8))
		dev_info(priv->dev, "Main Input Unit AXI ERR (AXISTS=0x%x)\n",
			 readl(base + DISP8M_APB_OUT_AXISTS_OFS));

	if (status & BIT(12))
		dev_info(priv->dev, "OSD0 Input Unit AXI ERROR %d\n",
			 (readl(base + DISP8M_APB_OUT_AXISTS_OFS) >> 16) & 3);

	return IRQ_HANDLED;
}

static const char const *disp8m_irq_reason[] = {
	"DISP-LCD",
	"DISP-HDMI",
};

static int
disp8m_fb_probe(struct platform_device *pdev)
{
	struct disp8m_priv *priv;
	struct disp8m_par *par;
	struct resource *res;
	char clkname[8];
	int ret = 0;
	int n;

	/* per-CRTC setup (one) */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto bail;
	}

	dev_set_drvdata(&pdev->dev, priv);
	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing base resource\n");
		ret = -EINVAL;
		goto bail;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	priv->base_pa = res->start;

	of_property_read_u32(pdev->dev.of_node, "id", &priv->id);

	init_completion(&priv->vsync_completion);
	mutex_init(&priv->lock);

	for (n = 0; n < ARRAY_SIZE(priv->irq); n++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, n);
		if (!res) {
			dev_err(&pdev->dev, "Missing interrupt resource\n");
			ret = -EINVAL;
			goto bail1;
		}
		sprintf(priv->irq_name[n], "%s:%s", dev_name(&pdev->dev),
			disp8m_irq_reason[n]);
		priv->irq[n] = res->start;
		ret = request_irq(priv->irq[n], disp8m_isr, IRQF_TRIGGER_RISING,
				  priv->irq_name[n], priv);
		if (ret) {
			dev_err(&pdev->dev, "failed to allocate irq\n");
			goto bail1;
		}
		disable_irq(priv->irq[n]);
	}
	ret = 0;
	priv->clocks_enabled = 0;
	while (!ret) {
		sprintf(clkname, "clk%d", priv->clocks_enabled + 1);
		priv->clocks[priv->clocks_enabled] = clk_get(&pdev->dev, clkname);
		if (IS_ERR(priv->clocks[priv->clocks_enabled])) {
			ret = 1;
			continue;
		}
		priv->clocks_enabled++;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* per framebuffer / overlay setup (three) */

	for (n = 0; n < ARRAY_SIZE(priv->info); n++) {
		priv->info[n] = framebuffer_alloc(sizeof(*par), &pdev->dev);
		if (!priv->info[n])
			goto unwind_fbs;

		par = priv->info[n]->par;
		memset(par, 0, sizeof(*par));
		par->info = priv->info[n];

		priv->info[n]->fbops = &disp8m_fb_ops;
		priv->info[n]->fix = disp8m_fb_fix;
		priv->info[n]->dev = &pdev->dev;
		par->priv = priv;
		par->fbs = n;

		priv->info[n]->pseudo_palette = par->pseudo_palette;
		ret = fb_alloc_cmap(&priv->info[n]->cmap, 256, 0);
		if (ret) {
			dev_err(&pdev->dev, "Failed to alloc cmap\n");
			goto unwind_fbs;
		}
		fb_set_cmap(&priv->info[n]->cmap, priv->info[n]);

		/* register our fdb_child with f_fdb bus */
		par->fdb_child.priv = par;

		ret = fdb_register(&pdev->dev, &par->fdb_child,
				   &disp8m_fb_fdb_ops);
		if (ret < 0) {
			dev_err(&pdev->dev, "fdb registration failed\n");
			fb_dealloc_cmap(&priv->info[n]->cmap);
			framebuffer_release(priv->info[n]);
			goto unwind_fbs;
		}
	}

	dev_info(&pdev->dev, "disp8m framebuffers registered on fdb\n");

	return 0;

unwind_fbs:
	while (--n >= 0) {
		par = priv->info[n]->par;
		fdb_unregister(&pdev->dev, &par->fdb_child);
		fb_dealloc_cmap(&priv->info[n]->cmap);
		framebuffer_release(priv->info[n]);
	}
	pm_runtime_put_sync_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	iounmap(priv->base);
	while (--priv->clocks_enabled >= 0)
		clk_put(priv->clocks[priv->clocks_enabled]);

bail1:
	for (n = 0; n < ARRAY_SIZE(priv->irq); n++)
		if (priv->irq[n])
			free_irq(priv->irq[n], priv);

bail:
	kfree(priv);

	return ret;
}

static int
disp8m_fb_remove(struct platform_device *pdev)
{
	struct disp8m_priv *priv = dev_get_drvdata(&pdev->dev);
	struct disp8m_par *par;
	int n;

	for (n = 0; n < ARRAY_SIZE(priv->irq); n++)
		if (priv->irq[n])
			free_irq(priv->irq[n], priv);

	for (n = 0; n < ARRAY_SIZE(priv->info); n++) {
		par = priv->info[n]->par;

		fdb_unregister(&pdev->dev, &par->fdb_child);

//		dma_free_writecombine(info->dev, par->framesize,
//				      par->fb_va, par->fbpaddr);

		fb_dealloc_cmap(&priv->info[n]->cmap);
		framebuffer_release(priv->info[n]);
	}

	pm_runtime_put_sync_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	iounmap(priv->base);

	while (--priv->clocks_enabled >= 0)
		clk_put(priv->clocks[priv->clocks_enabled]);

	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int
disp8m_runtime_suspend(struct device *dev)
{
	struct disp8m_priv *priv = dev_get_drvdata(dev);
	int n;

	dev_info(dev, "%s\n", __func__);

	for (n = 0; n < ARRAY_SIZE(priv->irq); n++)
		if (priv->irq[n])
			disable_irq(priv->irq[n]);

	for (n = priv->clocks_enabled - 1; n >= 0; n--)
		clk_disable_unprepare(priv->clocks[n]);

	return 0;
}

static int
disp8m_runtime_resume(struct device *dev)
{
	struct disp8m_priv *priv = dev_get_drvdata(dev);
	struct disp8m_par *par;
	int n;

	dev_info(dev, "%s\n", __func__);

	/* first let the clocks back on */

	for (n = 0; n < priv->clocks_enabled; n++)
		clk_prepare_enable(priv->clocks[n]);

	/* let the interrupts back on */

	for (n = 0; n < ARRAY_SIZE(priv->irq); n++)
		if (priv->irq[n])
			enable_irq(priv->irq[n]);

	if (!priv->mode_valid)
		return 0;

	/*
	 * although drm will re-set the mode if we are in Xorg, it won't do
	 * anything if we are in framebuffer console... so re-set the mode
	 * ourselves, if we ever had a mode set
	 */

//	for (n = 0; n < ARRAY_SIZE(priv->info); n++) {
		par = priv->info[0]->par;
		_disp8m_fb_set_timings(par, &priv->last_timings);
//	}
	dev_info(dev, "%s end\n", __func__);

	return 0;
}

static int
disp8m_pm_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return disp8m_runtime_suspend(dev);
}

static int
disp8m_pm_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return disp8m_runtime_resume(dev);
}
#endif

static const struct of_device_id disp8m_fb_dt_ids[] = {
	{ .compatible = "socionext,jdsdisp2b" },
	{ /* sentinel */ }
};

#ifdef CONFIG_PM
static const struct dev_pm_ops disp8m_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(disp8m_pm_suspend, disp8m_pm_resume)
	SET_RUNTIME_PM_OPS(disp8m_runtime_suspend, disp8m_runtime_resume, NULL)
};
#endif

static struct platform_driver disp8m_fb_driver = {
	.probe = disp8m_fb_probe,
	.remove = disp8m_fb_remove,
	.driver = {
		.name = "disp8m_fb",
		.of_match_table = disp8m_fb_dt_ids,
#ifdef CONFIG_PM
		.pm = &disp8m_pm_ops,
#endif
	},
};


MODULE_DEVICE_TABLE(of, disp8m_fb_dt_ids);

static int __init disp8m_fb_init(void)
{
	return platform_driver_register(&disp8m_fb_driver);
}

static void __exit disp8m_fb_exit(void)
{
	platform_driver_unregister(&disp8m_fb_driver);
}

module_init(disp8m_fb_init);
module_exit(disp8m_fb_exit);

MODULE_LICENSE("GPL");
