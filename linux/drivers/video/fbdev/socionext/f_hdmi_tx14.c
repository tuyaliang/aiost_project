/*
 * f_hdmi_tx14 HDMI 14 PHY driver
 * Copyright (C) 2013-2015 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/of_address.h>
#include <sound/asound.h>
#include <sound/asoundef.h>
#include <video/f_hdmi_tx14.h>
#include <video/fdb.h>

struct f_hdmi_tx14 {
	struct f_fdb_child fdb_child;
	struct device *dev;
	void __iomem *base;
	int irq;
	struct drm_device *drm_dev;
	u32 source_crtc_bitfield;
	enum hdmi_dvi_mode mode;
	struct f_fdb_child *bound;
	u8 edid[512];
	bool private_edid_valid;

	struct clk *clk[3];
	int clocks;

	struct mutex lock;
	struct f_hdmi_config cfg;
	struct f_hdmi_infoframe_avi avi_cfg;

	struct workqueue_struct *wq;
	struct work_struct worker;
};

/*
 * reference list of CEA timings we can use HDMI audio with
 */

struct f_video_timings {
	u16 x_res;
	u16 y_res;
	u32 pixel_clock;
	u16 hsw;        /* Horizontal synchronization pulse width */
	u16 hfp;        /* Horizontal front porch */
	u16 hbp;        /* Horizontal back porch */
	u16 vsw;        /* Vertical synchronization pulse width */
	u16 vfp;        /* Vertical front porch */
	u16 vbp;        /* Vertical back porch */
	bool vsync_level;
	bool hsync_level;
	bool interlace;
	unsigned char cea_code;
};

static const struct f_video_timings cea_timings[] = {
	{
		640, 480, 25200, 96, 16, 48, 2, 10, 33,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 1
	}, {
		720, 480, 27027, 62, 16, 60, 6, 9, 30,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 2
	}, {
		1280, 720, 74250, 40, 110, 220, 5, 5, 20,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 4,
	}, {
		1920, 540, 74250, 44, 88, 148, 5, 2, 15,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			true, 5,
	}, {
		1440, 240, 27027, 124, 38, 114, 3, 4, 15,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			true, 6,
	}, {
		1920, 1080, 148500, 44, 88, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 16,
	}, {
		720, 576, 27000, 64, 12, 68, 5, 5, 39,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 17,
	}, {
		1280, 720, 74250, 40, 440, 220, 5, 5, 20,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 19,
	}, {
		1920, 540, 74250, 44, 528, 148, 5, 2, 15,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			true, 20,
	}, {
		1440, 288, 27000, 126, 24, 138, 3, 2, 19,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			true, 21,
	}, {
		1440, 576, 54000, 128, 24, 136, 5, 5, 39,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 29,
	}, {
		1920, 1080, 148500, 44, 528, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 31,
	}, {
		1920, 1080, 74250, 44, 638, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 32,
	},
};

static void f_hdmi_core_aux_infoframe_avi_config(struct f_hdmi_tx14 *priv)
{
	char sum = 0;
	u8 tx[16];
	int n;

	tx[0] = HDMI_INFOFRAME_TYPE_AVI;
	tx[1] = 0x02,
	tx[2] = 0x0d;
	tx[3] = (priv->avi_cfg.db1_format << 5) |
		(priv->avi_cfg.db1_active_info << 4) |
		(priv->avi_cfg.db1_bar_info_dv << 2) |
		priv->avi_cfg.db1_scan_info;
	tx[4] = (priv->avi_cfg.db2_colorimetry << 6) |
		(priv->avi_cfg.db2_aspect_ratio << 4) |
		priv->avi_cfg.db2_active_fmt_ar;
	tx[5] = (priv->avi_cfg.db3_itc << 7) |
		(priv->avi_cfg.db3_ec << 4) |
		(priv->avi_cfg.db3_q_range << 2) |
		priv->avi_cfg.db3_nup_scaling;
	tx[6] = priv->avi_cfg.db4_videocode;
	tx[7] = priv->avi_cfg.db5_pixel_repeat;
	tx[8] = priv->avi_cfg.db6_7_line_eoftop;
	tx[9] = priv->avi_cfg.db6_7_line_eoftop >> 8;
	tx[10] = priv->avi_cfg.db8_9_line_sofbottom;
	tx[11] = priv->avi_cfg.db8_9_line_sofbottom >> 8;
	tx[12] = priv->avi_cfg.db10_11_pixel_eofleft;
	tx[13] = priv->avi_cfg.db10_11_pixel_eofleft >> 8;
	tx[14] = priv->avi_cfg.db12_13_pixel_sofright;
	tx[15] = priv->avi_cfg.db12_13_pixel_sofright >> 8;

	for (n = 0; n < ARRAY_SIZE(tx); n++) {
		sum += tx[n];
		writeb(tx[n], priv->base + FHT14_PKT_AVI_TYPE_OFS + n);
	}

	writeb(0x100 - sum, priv->base + FHT14_PKT_AVI_CHSUM_OFS);
}

static void f_hdmi_core_av_packet_config(struct f_hdmi_tx14 *priv,
		struct f_hdmi_packet_enable_repeat *repeat_cfg)
{
	/* enable/repeat the infoframe */
	writeb((repeat_cfg->audio_pkt << 5) |
	       (repeat_cfg->audio_pkt_repeat << 4) |
	       (repeat_cfg->avi_infoframe << 1) |
	       repeat_cfg->avi_infoframe_repeat,
	       priv->base + FHT14_PKT_PB_CTRL1_OFS);
}

int f_hdmi_wp_video_start(struct f_hdmi_tx14 *priv)
{
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_GEN, 1);

	return 0;
}

void f_hdmi_wp_video_stop(struct f_hdmi_tx14 *priv)
{
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_GEN, 0);
}

static void f_hdmi_wp_video_config_timing(struct f_hdmi_tx14 *priv)
{
	int flags = priv->cfg.timings.dt.flags;
	int fields = (!!(flags & DISPLAY_FLAGS_INTERLACED)) + 1;
	u32 resolution_h = priv->cfg.timings.dt.hactive.typ;
	u32 resolution_v = priv->cfg.timings.dt.vactive.typ / fields;
	u32 timing_h = priv->cfg.timings.dt.hback_porch.typ +
		       priv->cfg.timings.dt.hsync_len.typ;
	u32 timing_v = (priv->cfg.timings.dt.vback_porch.typ +
		        priv->cfg.timings.dt.vsync_len.typ) / fields;

	if (!priv->cfg.timings.dt.pixelclock.typ)
		return;

	dev_info(priv->dev, "%s: %d x %d (%d fields)\n", __func__,
		 priv->cfg.timings.dt.hactive.typ,
		 priv->cfg.timings.dt.vactive.typ, fields);

	fdb_setb(priv->base, FHT14_VDE_DE_DLY, DE_DLY_L, timing_h & 0xff);
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_DLY_H,
						(timing_h >> 8) & 0xf);
	fdb_setb(priv->base, FHT14_VDE_DE_TOP, DE_TOP, timing_v & 0x7f);

	fdb_setb(priv->base, FHT14_VDE_DE_CNTL, DE_CNTL, resolution_h & 0xff);
	fdb_setb(priv->base, FHT14_VDE_DE_CNTH, DE_CNTH,
			((resolution_h >> 8) & 0xf));
	fdb_setb(priv->base, FHT14_VDE_DE_LINL, DE_LINL,
			resolution_v & 0xff);
	fdb_setb(priv->base, FHT14_VDE_DE_LINH, DE_LINH,
			((resolution_v >> 8) & 0x7));
/*
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, HS_POL,
					!(flags & DISPLAY_FLAGS_HSYNC_HIGH));
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, VS_POL,
					!(flags & DISPLAY_FLAGS_VSYNC_HIGH));
*/
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, HS_POL, 0);
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, VS_POL, 0);

	fdb_setb(priv->base, FHT14_VDE_HWIDTH_H, HIGH2,
				priv->cfg.timings.dt.hsync_len.typ >> 8);
	fdb_setb(priv->base, FHT14_VDE_HWIDTH_L, LOW,
				priv->cfg.timings.dt.hsync_len.typ & 0xff);
	fdb_setb(priv->base, FHT14_VDE_VWIDTH, WIDTH,
				priv->cfg.timings.dt.vsync_len.typ / fields);
}
static void f_hdmi_tx14_init_irqs(struct f_hdmi_tx14 *priv)
{
	disable_irq(priv->irq);

	writeb(0x60, priv->base + FHT14_INTR_INT_UNMASK1_OFS);
	writeb(0, priv->base + FHT14_INTR_INT_UNMASK2_OFS);
	writeb(0, priv->base + FHT14_INTR_INT_UNMASK3_OFS);
	writeb(0, priv->base + FHT14_INTR_INT_UNMASK4_OFS);
	writeb(0xff, priv->base + FHT14_INTR_INTR1_OFS);
	writeb(0xff, priv->base + FHT14_INTR_INTR2_OFS);
	writeb(0xff, priv->base + FHT14_INTR_INTR3_OFS);
	writeb(0xff, priv->base + FHT14_INTR_INTR4_OFS);

	/* interrupt should be deasserted then */

	/* Active High: deasserted = 0, asserted = 1 */
	fdb_setb(priv->base, FHT14_INTR_INT_CTRL, NPOLARITY, 0);

	enable_irq(priv->irq);
}

static bool _hdmi_tx14_detect(struct f_hdmi_tx14 *priv)
{
	return !!(fdb_getb(priv->base, FHT14_BASE_SYS_STAT, HPD) |
		  fdb_getb(priv->base, FHT14_BASE_SYS_STAT, RSEN));
}

static void hdmi_tx14_hpd_worker(struct work_struct *w)
{
	struct f_hdmi_tx14 *priv = container_of(w, struct f_hdmi_tx14, worker);

	drm_helper_hpd_irq_event(priv->drm_dev);
}

static irqreturn_t f_hdmi_tx14_interrupt(int irq, void *dev)
{
	struct f_hdmi_tx14 *priv = dev;
	u8 stat[4];
	int n;
	u8 good = 0;

	for (n = 0; n < 4; n++) {
		stat[n] = readb(priv->base + FHT14_INTR_INTR1_OFS + n);
		good |= stat[n] &
			readb(priv->base + FHT14_INTR_INT_UNMASK1_OFS + n);
	}

	if (!good)
		/* hum... we got an interrupt without a valid source
		 * we take it to mean the INT polarity is wrong.
		 *
		 * This IP sets it "wrong" (active low) every reset.
		 */
		fdb_setb(priv->base, FHT14_INTR_INT_CTRL, NPOLARITY,
			!(readb(priv->base + FHT14_INTR_INT_CTRL_OFS) & 2));

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, SOFT))
		dev_info(priv->dev, "IRQ: SOFT\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, HPD) ||
	    fdb_getval(stat[0], FHT14_INTR_INTR1, RSEN)) {
		dev_info(priv->dev, "IRQ: HPD / RSEN\n");
		if (!_hdmi_tx14_detect(priv))
			priv->private_edid_valid = false;
		if (priv->drm_dev)
			queue_work(system_power_efficient_wq, &priv->worker);
	}

	for (n = 0; n < 4; n++)
		writeb(stat[n], priv->base + FHT14_INTR_INTR1_OFS + n);

	return IRQ_HANDLED;
}

static int f_hdmi_tx14_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_HDMIA;
}

static int f_hdmi_tx14_is_hdmi_mode(struct f_hdmi_tx14 *priv)
{
	u8 val = fdb_getb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE);

	dev_info(priv->dev, "[%s] Is HDMI mode:%d\n", __func__, val);

	return !!val;
}
#if 0
static void f_hdmi_tx14_mode_enable(struct f_hdmi_tx14 *priv, u8 enable)
{
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE, enable);
}
#endif
static void f_hdmi_tx14_sendcp_packet(struct f_hdmi_tx14 *priv, u8 on)
{
	u8 val;
	int timeout = 1280;
	/* Send CP packets only if in HDMI mode */
	if (!f_hdmi_tx14_is_hdmi_mode(priv))
		return;
	val = fdb_getb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN);
	if (val & 0x08) { /* CP Enable */
		val = fdb_getb(priv->base, FHT14_PKT_GP_BYTE1, AVM);
		if (on) {
			if (val == 0x01) /* Already Mute set */
				return;
		} else {
			if (val == 0x10) /* Already Mute cleared */
				return;
		}
	}

	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN, 0);
	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_REPEAT, 0);
	while (timeout > 0) {
		val = fdb_getb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN);
		if (!(val & 0x08))
			break;
		udelay(50);
		timeout--;
	}

	if (!timeout) {
		dev_err(priv->dev, "timeout INF_CTRL2 %x time %d\n",
							val, timeout);
		return;
	}

	if (on)
		fdb_setb(priv->base, FHT14_PKT_GP_BYTE1, SETAVM, 1);
	else
		fdb_setb(priv->base, FHT14_PKT_GP_BYTE1, CLRAVM, 1);

	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN, 1);
	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_REPEAT, 1);
}

static void f_hdmi_tx14_phy_pwr_ctrl(struct f_hdmi_tx14 *priv, u8 power_state)
{
	/* Power Down the Phy(PD# assert) */
	if (power_state == 0) {
		f_hdmi_tx14_sendcp_packet(priv, 1);
#if 0
		fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 0);
#endif
		dev_info(priv->dev, "PHY powered down\n");
	} else {
		disable_irq(priv->irq);
		fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 1);
		dev_info(priv->dev, "mute off by phy power up\n");
		f_hdmi_tx14_sendcp_packet(priv, 0);
		dev_info(priv->dev, "PHY powered up\n");

		f_hdmi_tx14_init_irqs(priv);
		enable_irq(priv->irq);
	}
}

static void f_hdmi_tx14_pwr_ctrl(struct f_hdmi_tx14 *priv, u8 power_state)
{
	if (power_state == 0) {
		fdb_setb(priv->base, FHT14_ACR_DPD, NPOWERDOWN_TOTAL, 0);
		dev_info(priv->dev, "ip powered down\n");
	} else {
		fdb_setb(priv->base, FHT14_ACR_DPD, NPOWERDOWN_TOTAL, 1);
		dev_info(priv->dev, "ip powered up\n");
		f_hdmi_tx14_init_irqs(priv);
	}
}

static void f_hdmi_tx14_mddc_init(struct f_hdmi_tx14 *priv)
{
	dev_info(priv->dev, "Master DDC Init\n");
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
		 DDC_CMD__ABORT_TRANSACTION);

	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLEAR_FIFO);
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLOCK_SCL);
}

static void f_hdmi_tx14_wakeup(struct f_hdmi_tx14 *priv)
{
	disable_irq(priv->irq);
	fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 1);
	/* Interrupt pin polarity: Assertion HIGH */
	f_hdmi_tx14_init_irqs(priv);
	enable_irq(priv->irq);
}

static void f_hdmi_tx14_swreset(struct f_hdmi_tx14 *priv)
{
	int timeout = 255;
	int phy_pwr_on;
	u8 val;

	disable_irq(priv->irq);

	phy_pwr_on = fdb_getb(priv->base, FHT14_BASE_SYS_CTRL1, NPD);
	while (timeout > 0) { /* wait for input pixel clock to stabilized */
		val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
		if (val & 0x1)
			break;
		timeout--;
	}

	/* Assert SW Reset */
	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 1);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 1);
	val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
	dev_info(priv->dev, "STAT after RST=1: %02x\n", val);

	f_hdmi_tx14_phy_pwr_ctrl(priv, 0); /* Phy Power Down */
	usleep_range(1000, 2000);
	if (phy_pwr_on) {
		dev_info(priv->dev, "swreset(), phy_pwr_on=1\n");
		f_hdmi_tx14_phy_pwr_ctrl(priv, 1); /* Phy Power On */
	}

	/* Release SW Reset */
	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 0);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 0);

	val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
	dev_info(priv->dev, "STAT after RST=0: %02x\n", val);

	f_hdmi_tx14_init_irqs(priv);

	/* allow TCLK (sent to Rx across the link) to stabilize*/
	usleep_range(64000, 100000);

	enable_irq(priv->irq);
}

static void f_hdmi_tx14_no_hdcp(struct f_hdmi_tx14 *priv)
{
	/* Send zero in audio packet */
	fdb_setb(priv->base, FHT14_BASE_SYS_DCTL, AUD_MUTE, 0);
	f_hdmi_tx14_sendcp_packet(priv, 0);
}

static void f_hdmi_tx14_hdmitx_init(struct f_hdmi_tx14 *priv)
{
	dev_info(priv->dev, "HDMI TX Init - PD# %02x\n",
			fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE));
	fdb_setb(priv->base, FHT14_TMDS_TMDS_CTRL, TCLKSEL, 1);
	f_hdmi_tx14_wakeup(priv);
	#if 0
	f_hdmi_tx14_swreset(priv);
	#endif
}

static int f_hdmi_tx14_hw_init(struct f_hdmi_tx14 *priv)
{
	int n;
	u8 val;

	/* Check IDCK to TMDS Clock Stable */
	for (n = 0; n < 200; n++) {
		val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
		if (val & 0x1)
			break;
		udelay(1);
	}
	f_hdmi_tx14_phy_pwr_ctrl(priv, 1); /* Phy Power On */
	f_hdmi_tx14_pwr_ctrl(priv, 1); /* Power On */
	f_hdmi_tx14_hdmitx_init(priv);

	return 0;
}

static void f_hdmi_tx14_basic_configure(struct f_hdmi_tx14 *priv)
{
	/* Before transmmit, basic setting for video timing and info-frame */
	struct f_hdmi_video_config v_core_cfg;
	struct f_hdmi_packet_enable_repeat repeat_cfg;
	struct f_hdmi_infoframe_avi avi_cfg = priv->avi_cfg;

	f_hdmi_tx14_init_irqs(priv);

	/* video config */
	v_core_cfg.tclk_sel_clkmult = HDMI_FPLL10IDCK;
	v_core_cfg.pkt_mode = HDMI_PACKETMODE24BITPERPIXEL;
	v_core_cfg.deep_color = HDMI_DEEPCOLORPACKECTDISABLE;
	v_core_cfg.hdmi_dvi = HDMI_HDMI;
	v_core_cfg.buswidth = HDMI_INPUT_8BIT;
	v_core_cfg.clipcs = HDMI_OUTPUT_RGB;
	v_core_cfg.rangclip = HDMI_RANGECLIP_DISABLE;
	v_core_cfg.rgb2ycbcr = HDMI_RBG2YCBCR_DISABLE;
	v_core_cfg.rangcmps = HDMI_RANGECMPS_DISABLE;
	v_core_cfg.downsmp = HDMI_DOWNSAMPLE_DISABLE;
	v_core_cfg.dithmode = HDMI_OUTPUTDITHER_8BIT;
	v_core_cfg.dither = HDMI_DITHER_ENABLE;
	v_core_cfg.range = HDMI_RANGE_ENABLE;
	v_core_cfg.csc = HDMI_YCBCR2RGB_ENABLE;
	v_core_cfg.upsmp = HDMI_UPSAMPLE_ENABLE;
	v_core_cfg.demux = HDMI_DEMUX_DISABLE;
	v_core_cfg.syncext = HDMI_SYNCEXTRAT_DISABLE;

	/* info frame */
	memset(&avi_cfg, 0, sizeof(avi_cfg));

	/* packet enable and repeat */
	memset(&repeat_cfg, 0, sizeof(repeat_cfg));

	f_hdmi_wp_video_config_timing(priv);

	/*
	 * configure core video part
	 * set software reset in the core
	 */
	v_core_cfg.pkt_mode = HDMI_PACKETMODE24BITPERPIXEL;
	v_core_cfg.hdmi_dvi = priv->cfg.cm.mode;

	fdb_setb(priv->base, FHT14_VDE_VID_MODE, VALUE,
		 v_core_cfg.syncext | v_core_cfg.demux | v_core_cfg.upsmp |
		 v_core_cfg.csc | v_core_cfg.range | v_core_cfg.dither |
		 v_core_cfg.dithmode);

	/* Latch input on rising edge */
	fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, EDGE, 1);
	fdb_setb(priv->base, FHT14_VDE_VID_ACEN, VALUE,
		 v_core_cfg.downsmp | v_core_cfg.rangcmps |
		 v_core_cfg.rgb2ycbcr | v_core_cfg.rangclip |
		 v_core_cfg.clipcs | v_core_cfg.buswidth);

	/* Color Space */
	fdb_setb(priv->base, FHT14_VDE_VID_CTRL, CSCSEL,
		 priv->cfg.timings.dt.vactive.typ >= 720);

	fdb_setb(priv->base, FHT14_VDE_VID_CTRL, EXTN, 1);
	fdb_setb(priv->base, FHT14_VDE_VID_CTRL, ICLK, 0);

	/* Packet Mode */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, PACKET_MODE,
		 v_core_cfg.pkt_mode);
	/* Deep Color */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, DC_EN,
		 v_core_cfg.deep_color);
	/* HDMI Mode */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE,
		 v_core_cfg.hdmi_dvi);
	/* TMDS_CTRL */
	fdb_setb(priv->base, FHT14_TMDS_TMDS_CTRL, TCLKSEL,
		 v_core_cfg.tclk_sel_clkmult);

	f_hdmi_tx14_swreset(priv);

	/*
	 * configure packet
	 * info frame video see doc CEA861-D page 65
	 */
	avi_cfg.db1_format = HDMI_INFOFRAME_AVI_DB1Y_RGB;
	avi_cfg.db1_active_info =
		HDMI_INFOFRAME_AVI_DB1A_ACTIVE_FORMAT_OFF;
	avi_cfg.db1_bar_info_dv = HDMI_INFOFRAME_AVI_DB1B_NO;
	avi_cfg.db1_scan_info = HDMI_INFOFRAME_AVI_DB1S_0;
	avi_cfg.db2_colorimetry = HDMI_INFOFRAME_AVI_DB2C_NO;
	avi_cfg.db2_aspect_ratio = HDMI_INFOFRAME_AVI_DB2M_NO;
	avi_cfg.db2_active_fmt_ar = HDMI_INFOFRAME_AVI_DB2R_SAME;
	avi_cfg.db3_itc = HDMI_INFOFRAME_AVI_DB3ITC_NO;
	avi_cfg.db3_ec = HDMI_INFOFRAME_AVI_DB3EC_XVYUV601;
	avi_cfg.db3_q_range = HDMI_INFOFRAME_AVI_DB3Q_DEFAULT;
	avi_cfg.db3_nup_scaling = HDMI_INFOFRAME_AVI_DB3SC_NO;
	avi_cfg.db4_videocode = priv->cfg.cm.code;
	avi_cfg.db5_pixel_repeat = HDMI_INFOFRAME_AVI_DB5PR_NO;
	avi_cfg.db6_7_line_eoftop = 0;
	avi_cfg.db8_9_line_sofbottom = 0;
	avi_cfg.db10_11_pixel_eofleft = 0;
	avi_cfg.db12_13_pixel_sofright = 0;

	f_hdmi_core_aux_infoframe_avi_config(priv);

	/* enable/repeat the infoframe */
	repeat_cfg.avi_infoframe = HDMI_PACKETENABLE;
	repeat_cfg.avi_infoframe_repeat = HDMI_PACKETREPEATON;
	repeat_cfg.audio_pkt = HDMI_PACKETENABLE;
	repeat_cfg.audio_pkt_repeat = HDMI_PACKETREPEATON;

	f_hdmi_core_av_packet_config(priv, &repeat_cfg);
}

static struct f_hdmi_cm f_hdmi_get_code(struct fdb_video_timings *timing)
{
	u32 pixel_clk = timing->dt.pixelclock.typ; /* in kHz */
	static struct f_hdmi_cm cm = { -1, HDMI_DVI };
	int i;

	for (i = 0; i < ARRAY_SIZE(cea_timings); i++) {
		/* pr_info("%d: %d/%d %d/%d %d/%d %d/%d\n", i,
		 timing->dt.hsync_len.typ, cea_timings[i].hsw,
		 timing->dt.hback_porch.typ, cea_timings[i].hbp,
		 timing->dt.hfront_porch.typ, cea_timings[i].hfp,
			    pixel_clk, cea_timings[i].pixel_clock); */

		if (
		 timing->dt.hsync_len.typ == cea_timings[i].hsw &&
		 timing->dt.hback_porch.typ == cea_timings[i].hbp &&
		 timing->dt.hfront_porch.typ == cea_timings[i].hfp &&
			    pixel_clk == cea_timings[i].pixel_clock) {
			cm.mode = HDMI_HDMI;
			cm.code = cea_timings[i].cea_code;
			return cm;
		}
	}

	return cm;
}

static int f_hdmi_tx14_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_hdmi_cm cm;
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int ret = 0;

	mutex_lock(&priv->lock);

	/* we can only support integer division of 148.5MHz... */
	if (
#if 0
	    timings->dt.pixelclock.typ != 148500 &&
#endif
	    //timings->dt.hactive.typ == 1920 ||
	    (timings->dt.pixelclock.typ != 74250 &&
	     timings->dt.pixelclock.typ != 27000)) {
#if 0
		dev_info(priv->dev, "%s: ---- REJECT on clock %u\n",
					  __func__, timings->dt.pixelclock.typ);
#endif
		ret = -EINVAL;
		goto bail;
	}

	/* with CEA timings we can use audio, so let's prefer them */
	cm = f_hdmi_get_code(timings);
	if (cm.code == -1) {
		dev_dbg(priv->dev, "%s: REJECT non CEA\n", __func__);
		ret = -EINVAL;
		goto bail;
	}
	if (!priv->bound)
		goto bail;
	if (!priv->bound->ops->check_timings)
		goto bail;

	ret = priv->bound->ops->check_timings(priv->bound, timings);
bail:
	mutex_unlock(&priv->lock);

	return ret;
}

static bool f_hdmi_tx14_detect(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	u8 val;

	pm_runtime_get_sync(priv->dev);

	val = _hdmi_tx14_detect(priv);

	pm_runtime_put(priv->dev);

	if (priv->bound && priv->bound->ops->hdmi_presence_update)
		priv->bound->ops->hdmi_presence_update(priv->bound, !!val);

	return !!val;
}

static int f_hdmi_tx14_ddc_edid(struct f_fdb_child *fdb_child,
							u8 *edid, int ext)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	u32 i;
	int n;
	u8 csum = 0;
	u32 offset = 0;
	int timeout = 100;

	while (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, IN_PROG) && --timeout)
		udelay(100);

	if (!timeout)
		dev_info(priv->dev, "DDC problem status=0x%x\n",
			readb(priv->base + FHT14_DDC_DDC_STATUS_OFS));

	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLEAR_FIFO);

	if (ext & 1)
		offset = 0x80;

	fdb_setb(priv->base, FHT14_DDC_DDC_ADDR, ADDR, EDID_SLV);
	fdb_setb(priv->base, FHT14_DDC_DDC_SEGM, SEGM, ext >> 1);
	fdb_setb(priv->base, FHT14_DDC_DDC_OFFSET, OFFSET, offset);
	fdb_setb(priv->base, FHT14_DDC_DDC_COUNT_LOW, COUNT1, 0x80);
	fdb_setb(priv->base, FHT14_DDC_DDC_COUNT_UP, COUNT2, 0);

	if (ext)
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
			 DDC_CMD__ENHANCED_DDC_READ_NO_ACK_ON_LAST);
	else
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD,
			 DDC_CMD, DDC_CMD__SEQ_READ_NO_ACK_ON_LAST);

	/* DDC_STATUS_BUS_LOW */
	if (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, BUS_LOW)) {
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
			 DDC_CMD__ABORT_TRANSACTION);

		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
			 DDC_CMD__CLOCK_SCL);

		fdb_setb(priv->base, FHT14_DDC_DDC_MAN, SCL, 0);
		fdb_setb(priv->base, FHT14_DDC_DDC_MAN, SDA, 0);
		dev_err(priv->dev, "I2C Bus Low?\n");
		return -EIO;
	}
	/* DDC_STATUS_NO_ACK */
	if (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, NO_ACK)) {
		dev_err(priv->dev, "I2C No ACK\n");
		return -EIO;
	}

	timeout = 300;
	i = 0;
	while (timeout && i < 0x80) {
		n = readb(priv->base + FHT14_DDC_DDC_FIFOCNT_OFS);
		n = fdb_const(FHT14_DDC_DDC_FIFOCNT, FIFOCONT, n);
		if (n) {
			while (n-- && i != 0x80)
				edid[i++] = fdb_getb(priv->base,
						FHT14_DDC_DDC_DATA, DDC_DATA);
			continue;
		}
		udelay(100);
		timeout--;
	}

	if (!timeout) {
		dev_info(priv->dev, "DDC BUSY status=0x%x\n",
			readb(priv->base + FHT14_DDC_DDC_STATUS_OFS));
		return -EIO;
	}

	print_hex_dump(KERN_INFO, "",
			DUMP_PREFIX_NONE, 16, 1, edid, 0x80, true);

	csum = 0;
	for (i = 0; i < 0x80; i++)
		csum += edid[i];

	if (!csum)
		return 0;

	dev_err(priv->dev, "E-EDID checksum failed!!\n");

	return -EIO;
}

static int f_hdmi_tx14_read_edid(struct f_fdb_child *fdb_child,
							u8 *buf, int len)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int r, l, j = 0, valid_extensions = 0;
	u8 ext_num;
	int ret = 0;

	mutex_lock(&priv->lock);
	pm_runtime_get_sync(priv->dev);

	if (priv->private_edid_valid)
		goto done;

	if (len < 128) {
		ret = -EINVAL;
		goto bail;
	}

	f_hdmi_tx14_mddc_init(priv);
	r = f_hdmi_tx14_ddc_edid(fdb_child, priv->edid, 0);
	if (r) {
		ret = r;
		goto bail;
	}

	l = 128;
	ext_num = priv->edid[0x7e];

	if (len >= 128 * ext_num && ext_num > 0)
		for (j = 1; j <= ext_num; j++) {
			r = f_hdmi_tx14_ddc_edid(fdb_child,
				priv->edid + ((valid_extensions + 1) * 128), j);
			if (r) {
				ret = r;
				goto bail;
			}

			valid_extensions++;
			l += 128;
		}

	priv->private_edid_valid = true;
done:
	memcpy(buf, priv->edid, len);
	ret = len;
bail:

	pm_runtime_put(priv->dev);

	mutex_unlock(&priv->lock);

	return ret;
}

static u32 f_hdmi_tx14_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int f_hdmi_tx14_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;

	return 0;
}

static int f_hdmi_tx14_hdmi_enable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int ret = 0;

	if (!priv->cfg.timings.dt.pixelclock.typ)
		return 0;

	mutex_lock(&priv->lock);

	ret = f_hdmi_tx14_hw_init(priv);
	if (ret)
		goto bail;

	f_hdmi_tx14_basic_configure(priv);
	f_hdmi_wp_video_start(priv);
	f_hdmi_tx14_no_hdcp(priv);
	f_hdmi_tx14_init_irqs(priv);

bail:
	mutex_unlock(&priv->lock);

	return ret;
}

static void f_hdmi_tx14_hdmi_disable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	mutex_lock(&priv->lock);

	f_hdmi_wp_video_stop(priv);

	mutex_unlock(&priv->lock);
}

static int f_hdmi_tx14_set_hdmi_dvi_mode(struct f_fdb_child *fdb_child, u8 mode)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->mode = mode;

	return 0;
}

int f_hdmi_tx14_audio_enable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, SD0_EN, 1);
	fdb_setb(priv->base, FHT14_ACR_ACR_CTRL, NCTS_PKT_EN, 1);

	return 0;
}

void f_hdmi_tx14_audio_disable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, SD0_EN, 0);
	fdb_setb(priv->base, FHT14_ACR_ACR_CTRL, NCTS_PKT_EN, 0);

}

int f_hdmi_tx14_audio_start(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, AUD_EN, 1);

	return 0;
}

static void f_hdmi_tx14_audio_stop(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, AUD_EN, 0);
}

static void f_hdmi_tx14_audio_format_cfg(struct f_hdmi_tx14 *priv,
					struct f_hdmi_audio_format *aud_fmt,
					struct f_hdmi_audio_config *aud_cfg)
{
	void __iomem *base = priv->base;
	int n, N;

	/* send forced N/CTS numbers */
	writeb(1, base + FHT14_ACR_ACR_CTRL_OFS);

	N = 6144;
	/* ie, 148500 for 1080p, 74250 for 720p etc */
	n = priv->cfg.timings.dt.pixelclock.typ;
	switch (aud_fmt->sample_rate) {
	case 3: /* 32kHz */
		N = 4096;
		break;
	case 0: /* 44.1kHz */
		n = 165000 / (148500 / n);
		N = 6272;
		break;
	case 2: /* 48kHz */
	default:
		break;
	}

	writeb(n, base + FHT14_ACR_CTS_SVAL_LOW_OFS);
	writeb(n >> 8, base + FHT14_ACR_CTS_SVAL_MID_OFS);
	writeb(n >> 16, base + FHT14_ACR_CTS_SVAL_UP_OFS);

	writeb(N, base + FHT14_ACR_N_SVAL_LOW_OFS);
	writeb(N >> 8, base + FHT14_ACR_N_SVAL_MID_OFS);
	writeb(N >> 16, base + FHT14_ACR_N_SVAL_UP_OFS);

	writeb(0xd0, priv->base + FHT14_ACRIN_SPDIF_ERTH_OFS);
	writeb(aud_cfg->mclk_mode, base + FHT14_ACR_FREQ_SVAL_OFS);
	writeb(readb(base + FHT14_ACRIN_AUD_MODE_OFS) |
	       aud_cfg->i2s_cfg.active_sds, base + FHT14_ACRIN_AUD_MODE_OFS);
	writeb(2, base + FHT14_ACRIN_I2S_CHST3_OFS);
	writeb(aud_fmt->sample_rate, base + FHT14_ACRIN_I2S_CHST4_OFS);
	writeb((aud_fmt->sample_rate << 4) | aud_fmt->sample_size,
	       base + FHT14_ACRIN_I2S_CHST5_OFS);
	writeb((readb(base + FHT14_ACRIN_I2S_IN_LEN_OFS) & 0xf0) |
	       aud_fmt->sample_size, base + FHT14_ACRIN_I2S_IN_LEN_OFS);
	writeb(0, base + FHT14_ACRIN_ASRC_OFS);

	fdb_setb(base, FHT14_ACR_HDMI_CTRL, LAYOUT,
		 aud_fmt->stereo_channels != HDMI_AUDIO_STEREO_ONECHANNEL);
	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, I2S_JUST,
		 aud_fmt->justification != HDMI_AUDIO_JUSTIFY_LEFT);
	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, VBIT,
		 aud_fmt->type != HDMI_AUDIO_TYPE_LPCM);
	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, I2S_SHIFT, 0);
	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, SCK_EDGE, 0);
}

static void f_hdmi_tx14_audio_infoframe_cfg(struct f_hdmi_tx14 *priv,
		struct snd_cea_861_aud_if *info_aud)
{
	char sum = 0;
	u8 tx[13];
	int n;

	info_aud->db1_ct_cc = BIT(4) | BIT(0);
	info_aud->db2_sf_ss = 3 << 2;
	info_aud->db3 = 0;
	info_aud->db4_ca = 0;
	info_aud->db5_dminh_lsv = 0;

	tx[0] = HDMI_INFOFRAME_TYPE_AUDIO;
	tx[1] = 0x01,
	tx[2] = 0x0a;
	tx[3] = info_aud->db1_ct_cc;
	tx[4] = info_aud->db2_sf_ss;
	tx[5] = info_aud->db3;
	tx[6] = info_aud->db4_ca;
	tx[7] = info_aud->db5_dminh_lsv;
	tx[8] = 0;
	tx[9] = 0;
	tx[10] = 0;
	tx[11] = 0;
	tx[12] = 0;

	for (n = 0; n < ARRAY_SIZE(tx); n++) {
		sum += tx[n];
		writeb(tx[n], priv->base + FHT14_PKT_AUD_TYPE_OFS + n);
	}

	writeb(0x100 - sum, priv->base + FHT14_PKT_AUD_CHSUM_OFS);
}

int f_hdmi_tx14_audio_config(struct f_fdb_child *fdb_child,
		struct f_fdb_audio *audio)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	struct f_hdmi_audio_format audio_format;
	struct f_hdmi_audio_config audio_config;
	int channel_count;
	bool word_length_16b = false;

	if (!audio || !audio->cea)
		return -EINVAL;

	if (!(audio->iec->status[4] & IEC958_AES4_CON_MAX_WORDLEN_24))
		if (audio->iec->status[4] & IEC958_AES4_CON_WORDLEN_20_16)
			word_length_16b = true;

#if 1
	word_length_16b = true;
	channel_count = 2;
#endif
	audio_config.mclk_mode = HDMI_AUDIO_MCLK_512FS;
	audio_format.sample_rate = HDMI_AUDIO_SAMPLE_RATE_48K;

	if (word_length_16b)
		audio_config.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_LEFT;
	else
		audio_config.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;

	if (channel_count == 2) {
		audio_format.stereo_channels = HDMI_AUDIO_STEREO_ONECHANNEL;
		audio_config.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN;
		audio_config.layout = HDMI_AUDIO_LAYOUT_2CH;
	} else {
		audio_format.stereo_channels = HDMI_AUDIO_STEREO_FOURCHANNELS;
		audio_config.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN |
						  HDMI_AUDIO_I2S_SD1_EN |
						  HDMI_AUDIO_I2S_SD2_EN |
						  HDMI_AUDIO_I2S_SD3_EN;
		audio_config.layout = HDMI_AUDIO_LAYOUT_8CH;
	}

	/* use sample frequency from channel status word */
	audio_config.fs_override = false;
	/* enable ACR packets */
	audio_config.en_acr_pkt = true;
	/* disable direct streaming digital audio */
	audio_config.en_dsd_audio = false;
	/* use parallel audio interface */
	audio_config.en_parallel_aud_input = true;

	if (word_length_16b) {
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_LEFT;
	} else {
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
	}
	audio_format.type = HDMI_AUDIO_TYPE_LPCM;
	audio_format.sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;

	f_hdmi_tx14_audio_format_cfg(priv, &audio_format, &audio_config);
	f_hdmi_tx14_audio_infoframe_cfg(priv, audio->cea);

	return 0;
}

void f_hdmi_tx14_initialized_by_drm(
	struct f_fdb_child *fdb_child, struct drm_device *drm_dev)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->drm_dev = drm_dev;
}



static void f_hdmi_tx14_set_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	mutex_lock(&priv->lock);

	priv->cfg.cm = f_hdmi_get_code(timings);
	priv->cfg.timings = *timings;

	f_hdmi_tx14_basic_configure(priv);
	f_hdmi_wp_video_start(priv);
	f_hdmi_tx14_no_hdcp(priv);
	f_hdmi_tx14_init_irqs(priv);

	dev_info(priv->dev, "%s: %d x %d\n", __func__,
		 priv->cfg.timings.dt.hactive.typ,
		 priv->cfg.timings.dt.vactive.typ);

	mutex_unlock(&priv->lock);
}

#if defined(CONFIG_DEBUG_FS)
static int
f_hdmi_tx14_fdb_debugfs(struct f_fdb_child *fdb_child,
			struct seq_file *m, int type)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	

	switch (type) {
	case FDB_DEBUGFS_DUMP:
		pm_runtime_get_sync(priv->dev);
		seq_printf(m, "pol%x, enabled %x %x %x %x, irq_state %x\n",
		   readb(priv->base + FHT14_INTR_INT_CTRL_OFS),
		   readb(priv->base + FHT14_INTR_INT_UNMASK1_OFS),
		   readb(priv->base + FHT14_INTR_INT_UNMASK2_OFS),
		   readb(priv->base + FHT14_INTR_INT_UNMASK3_OFS),
		   readb(priv->base + FHT14_INTR_INT_UNMASK4_OFS),
		   readb(priv->base + FHT14_INTR_INTR_STATE_OFS)
		);
		pm_runtime_put(priv->dev);

		break;
	}
	return 0;
}
#endif



struct f_fdb_ops f_hdmi_tx14_fdb_ops = {
	.initialized_by_drm = f_hdmi_tx14_initialized_by_drm,
	.get_connector_type = f_hdmi_tx14_get_connector_type,
	.check_timings = f_hdmi_tx14_check_timings,
	.set_timings = f_hdmi_tx14_set_timings,
	.detect = f_hdmi_tx14_detect,
	.get_source_crtc_bitfield = f_hdmi_tx14_get_source_crtc_bitfield,
	.bind_to_source = f_hdmi_tx14_bind_to_source,
	.read_edid = f_hdmi_tx14_read_edid,
	.enable = f_hdmi_tx14_hdmi_enable,
	.disable = f_hdmi_tx14_hdmi_disable,
	.set_hdmi_dvi_mode = f_hdmi_tx14_set_hdmi_dvi_mode,
	.audio_enable	= f_hdmi_tx14_audio_enable,
	.audio_disable	= f_hdmi_tx14_audio_disable,
	.audio_start = f_hdmi_tx14_audio_start,
	.audio_stop = f_hdmi_tx14_audio_stop,
	.audio_config = f_hdmi_tx14_audio_config,
	/*.audio_supported = f_hdmi_tx14_audio_supported,*/
#if defined(CONFIG_DEBUG_FS)
	.fdb_debugfs = f_hdmi_tx14_fdb_debugfs,
#endif
};


static int
f_hdmi_tx14_probe(struct platform_device *pdev)
{
	struct f_hdmi_tx14 *priv;
	int ret = 0;
	u8 devid[3];
	int n;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	INIT_WORK(&priv->worker, hdmi_tx14_hpd_worker);
	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	priv->base = of_iomap(pdev->dev.of_node, 0);
	if (!priv->base) {
		ret = -EINVAL;
		goto fail1;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "no irq resource\n");
		ret = -ENODEV;
		goto fail2;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		ret = -EINVAL;
		goto fail2;
	}

	do {
		priv->clk[priv->clocks] =
			of_clk_get(pdev->dev.of_node, priv->clocks);
		if (IS_ERR(priv->clk[priv->clocks]))
			break;
		ret = clk_prepare_enable(priv->clk[priv->clocks]);
		if (ret < 0)
			goto fail3;
		priv->clocks++;
	} while (priv->clocks < ARRAY_SIZE(priv->clk));
	if (!priv->clocks) {
		dev_err(&pdev->dev, "%s(): clock not found.\n", __func__);
		goto fail3;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* magic phy block init for m8m */
	writeb(0x24, priv->base + 0x801);
	writeb(0xeb, priv->base + 0x80c);

	/* set OCLK ratio + enable DDC clock */
	writeb(0xc | 1, priv->base + 0x808);
	writeb(0x42, priv->base + 0x809);
	writeb(1, priv->base + 0x80d);

	f_hdmi_tx14_init_irqs(priv);

	for (n = 0; n < 3; n++)
		devid[n] = readb(priv->base + FHT14_BASE_DEV_IDL_OFS + n);

	dev_info(priv->dev, "HDMI IP: Id 0x%02X.0x%02X. Rev %02i\n",
						devid[0], devid[1], devid[2]);

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child, &f_hdmi_tx14_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "fdb registration failed\n");
		goto fail3;
	}
	f_hdmi_tx14_init_irqs(priv);

	ret = devm_request_irq(&pdev->dev, priv->irq,
		f_hdmi_tx14_interrupt, IRQF_TRIGGER_HIGH,
						pdev->dev.driver->name, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto fail4;
	}

	dev_info(&pdev->dev, "f_hdmi_tx14 initialized\n");

	return 0;

fail4:
	fdb_unregister(&pdev->dev, &priv->fdb_child);
fail3:
	while (--priv->clocks) {
		clk_disable_unprepare(priv->clk[priv->clocks]);
		clk_put(priv->clk[priv->clocks]);
	}
fail2:
	iounmap(priv->base);
fail1:
	kfree(priv);

	return ret;
}

static int f_hdmi_tx14_remove(struct platform_device *pdev)
{
	struct f_hdmi_tx14 *priv = platform_get_drvdata(pdev);

	while (--priv->clocks) {
		clk_disable_unprepare(priv->clk[priv->clocks]);
		clk_put(priv->clk[priv->clocks]);
	}
	fdb_unregister(&pdev->dev, &priv->fdb_child);
	devm_free_irq(&pdev->dev, priv->irq, priv);
	pm_runtime_disable(&pdev->dev);
	iounmap(priv->base);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int
f_hdmi_tx14_runtime_suspend(struct device *dev)
{
	return 0;
}
static int
f_hdmi_tx14_runtime_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM
static const struct dev_pm_ops f_hdmi_tx14_pm_ops = {
	SET_RUNTIME_PM_OPS(f_hdmi_tx14_runtime_suspend,
			   f_hdmi_tx14_runtime_resume, NULL)
};
#endif

static const struct of_device_id f_hdmi_tx14_fb_dt_ids[] = {
	{ .compatible = "socionext,f_hdmi_tx14" },
	{ /* sentinel */ }
};

static struct platform_driver f_hdmi_tx14_driver = {
	.probe = f_hdmi_tx14_probe,
	.remove = f_hdmi_tx14_remove,
	.driver = {
		.name = "f_hdmi_tx14",
		.of_match_table = f_hdmi_tx14_fb_dt_ids,
#ifdef CONFIG_PM
		.pm = &f_hdmi_tx14_pm_ops,
#endif
	},
};

MODULE_DEVICE_TABLE(of, f_hdmi_tx14_fb_dt_ids);

static int __init f_hdmi_tx14_init(void)
{
	return platform_driver_register(&f_hdmi_tx14_driver);
}

static void __exit f_hdmi_tx14_exit(void)
{
	platform_driver_unregister(&f_hdmi_tx14_driver);
}

module_init(f_hdmi_tx14_init);
module_exit(f_hdmi_tx14_exit);

MODULE_LICENSE("GPL");
