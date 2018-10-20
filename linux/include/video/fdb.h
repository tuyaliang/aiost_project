/*
 * include/video/fdb.h
 *
 * Copyright (C) 2013 - 2015 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 *
 * much of the content here based on Rob Clark's omapdrm and dss work
 */

#ifndef __VIDEO_SOCIONEXT_FDB_H__
#define __VIDEO_SOCIONEXT_FDB_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>

/* some fdb settings are harmonized with DRM ones even when not using DRM */
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include <uapi/linux/fb.h>
#include <uapi/linux/fdb.h>

#include <linux/io.h>

#define FDB_MAX_BUS_CHILDREN 10

struct f_fdb_child;

struct f_fdb {
	struct device *dev;
	char name[10];
	int id;
	struct f_fdb_child *child[FDB_MAX_BUS_CHILDREN];
	int count_fdb_children;
};

struct fdb_video_timings {
	struct display_timing dt;
	int bits_per_pixel;
	unsigned int fourcc;
	int stride_px;

	int scanout_rotation;

	int xres_virtual;
	int yres_virtual;

	int width_mm;
	int height_mm;

	u8 red_offset;
	u8 red_length;
	u8 green_offset;
	u8 green_length;
	u8 blue_offset;
	u8 blue_length;
	u8 alpha_offset;
	u8 alpha_length;
};

/* per-format info: */
struct fdb_format {
	uint32_t pixel_format;
	struct {
		int stride_bpp;           /* this times width is stride */
		int sub_y;                /* sub-sample in y dimension */
	} planes[4];
	bool yuv;
};

struct f_fdb_audio {
	struct snd_aes_iec958 *iec;
	struct snd_cea_861_aud_if *cea;
};

struct fdb_dumps {
	const char *name;
	unsigned int start;
	unsigned int len;
};

enum {
	FDB_DEBUGFS_INFO,
	FDB_DEBUGFS_DUMP
};

struct f_fdb_ops {
	void (*initialized_by_drm)(struct f_fdb_child *, struct drm_device *);
	int (*probe)(struct f_fdb_child *);
	void (*remove)(struct f_fdb_child *);
	int (*enable)(struct f_fdb_child *fdb_child);
	void (*disable)(struct f_fdb_child *fdb_child);
	int (*run_test)(struct f_fdb_child *fdb_child, int test);
	int (*update)(struct f_fdb_child *fdb_child,
		      u16 x, u16 y, u16 w, u16 h);
	int (*sync)(struct f_fdb_child *fdb_child, int crtc_id, u32 count);
	int (*enable_te)(struct f_fdb_child *fdb_child, bool enable);
	int (*get_te)(struct f_fdb_child *fdb_child);
	u32 (*get_rotate)(struct f_fdb_child *fdb_child);
	int (*set_rotate)(struct f_fdb_child *fdb_child, struct fdb_blit *blit);
	void (*get_resolution)(struct f_fdb_child *fdb_child, u16 *x, u16 *y);
	void (*get_dimensions)(struct f_fdb_child *fdb_child, u32 *w, u32 *h);
	int (*get_recommended_bpp)(struct f_fdb_child *fdb_child);
	int (*check_timings)(struct f_fdb_child *fdb_child,
			     struct fdb_video_timings *timings);
	void (*set_timings)(struct f_fdb_child *fdb_child,
			    struct fdb_video_timings *timings);
	void (*get_timings)(struct f_fdb_child *fdb_child,
			    struct fdb_video_timings *timings);
	int (*read_edid)(struct f_fdb_child *fdb_child, u8 *buf, int len);
	bool (*detect)(struct f_fdb_child *fdb_child);
	void (*set_paddr)(struct f_fdb_child *fdb_child, dma_addr_t pa);
	void (*flip_next_vsync)(struct f_fdb_child *fdb_child, dma_addr_t pa,
		void (cb)(void *), void *cb_arg);
	int (*get_connector_type)(struct f_fdb_child *fdb_child);
	void (*set_gamma)(struct f_fdb_child *fdb_child,
			  u16 red, u16 green, u16 blue, int regno);
	void (*get_gamma)(struct f_fdb_child *fdb_child,
			  u16 *red, u16 *green, u16 *blue, int regno);

	u32 (*get_source_crtc_bitfield)(struct f_fdb_child *fdb_child);
	int (*bind_to_source)(struct f_fdb_child *fdb_child,
			      struct f_fdb_child *fdb_bound);
	void (*hdmi_presence_update)(struct f_fdb_child *fdb_child, bool present);
	const struct fdb_format *
		(*get_fdb_formats)(struct f_fdb_child *fdb_child, int *count);
	int (*set_hdmi_dvi_mode)(struct f_fdb_child *fdb_child, u8 mode);
	int (*blit_blocking)(struct f_fdb_child *fdb_child,
			     struct fdb_blit *blit);
	int (*fdb_debugfs)(struct f_fdb_child *fdb_child,
			   struct seq_file *m, int type);

	/*
	 * For display drivers that support audio. This encompasses
	 * HDMI and DisplayPort at the moment.
	 */
	/*
	 * Note: These functions might sleep. Do not call while
	 * holding a spinlock/readlock.
	 */
	int (*audio_enable)(struct f_fdb_child *fdb_child);
	void (*audio_disable)(struct f_fdb_child *fdb_child);
	bool (*audio_supported)(struct f_fdb_child *fdb_child);
	int (*audio_config)(struct f_fdb_child *fdb_child,
			    struct f_fdb_audio *audio);
	/* Note: These functions may not sleep */
	int (*audio_start)(struct f_fdb_child *fdb_child);
	void (*audio_stop)(struct f_fdb_child *fdb_child);
};

struct f_fdb_child {
	struct device *dev;
	struct f_fdb_ops *ops;
	struct mutex lock; /* protect child operations */
	void *priv; /* optional for use by driver priv owning child
			for cases where container_of() isn't enough */
};

/* works for devices that are children on an fdb bus */
#define f_fdb_from_dev(_dev) ((struct f_fdb *)dev_get_drvdata((_dev)->parent))
#define f_fdb_child_from_nb(_nb) container_of((_nb), struct f_fdb_child, nb)

#define dev_seq_printf(d, x, f, ...) \
	seq_printf(m, "%s: "f, dev_name(d), __VA_ARGS__)

static inline void
_set_fdb_bitfield(void __iomem *base, int shift, int mask, u32 val)
{
#ifdef DEBUG
	if (val & ~mask)
		pr_err("set_fdb_bf: inavlid 0x%x, mask = 0x%x\n", val, mask);
#endif
	__raw_writel(
		(__raw_readl(base) & ~(mask << shift)) | (val << shift), base);
}

static inline u32
_get_fdb_bitfield(void __iomem *base, int shift, int mask)
{
	return (__raw_readl(base) >> shift) & mask;
}

static inline void
_set_fdb_bitfield_w(void __iomem *base, int shift, u16 mask, u16 val)
{
#ifdef DEBUG
	if (val & ~mask)
		pr_err("set_fdb_bf: inavlid 0x%x, mask = 0x%x\n", val, mask);
#endif
	__raw_writew(
		(__raw_readw(base) & ~(mask << shift)) | (val << shift), base);
}

static inline u16
_get_fdb_bitfield_w(void __iomem *base, int shift, u16 mask)
{
	return (__raw_readw(base) >> shift) & mask;
}

static inline void
_set_fdb_bitfield_b(void __iomem *base, int shift, u8 mask, u8 val)
{
#ifdef DEBUG
	if (val & ~mask)
		pr_err("set_fdb_bf: inavlid 0x%x, mask = 0x%x\n", val, mask);
#endif
	__raw_writeb(
		(__raw_readb(base) & ~(mask << shift)) | (val << shift), base);
}

static inline u8
_get_fdb_bitfield_b(void __iomem *base, int shift, u8 mask)
{
	return (__raw_readb(base) >> shift) & mask;
}

static inline u32
_get_fdb_bitfield_from_val(u32 val, int shift, int mask)
{
	return (val >> shift) & mask;
}

static inline u32
_fdb_const(u32 val, int shift, int mask)
{
	return (val & mask) << shift;
}

#define fdb_setb(_base, _reg, _field, _val) \
	_set_fdb_bitfield_b(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK, _val)

#define fdb_getb(_base, _reg, _field) \
	_get_fdb_bitfield_b(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK)

#define fdb_setw(_base, _reg, _field, _val) \
	_set_fdb_bitfield_w(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK, _val)

#define fdb_getw(_base, _reg, _field) \
	_get_fdb_bitfield_w(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK)

#define fdb_setl(_base, _reg, _field, _val) \
	_set_fdb_bitfield(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK, _val)

#define fdb_getl(_base, _reg, _field) \
	_get_fdb_bitfield(_base + _reg##_OFS, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK)

#define fdb_getval(_val, _reg, _field) \
	_get_fdb_bitfield_from_val(_val, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK)

#define fdb_const(_reg, _field, _val) \
	_fdb_const(_val, _reg##__##_field##__SHIFT, \
			_reg##__##_field##__MASK)

int fdb_get_of_var(const char *name, struct fb_var_screeninfo *var);

/* register fdb bus child, call it from child probe */
int fdb_register(struct device *dev, struct f_fdb_child *child,
		 struct f_fdb_ops *ops);
/* unregister, on child remove */
int fdb_unregister(struct device *dev, struct f_fdb_child *child);

int fdb_get_peer_count(struct device *dev);

/* call this in the callback if you don't handle the notification yourself */
int fdb_notification_default(struct notifier_block *nb,
			     unsigned long val, void *data);
/* helper to get a child pointer from a name, like "fb0" */
int fdb_name_to_child(struct device *dev, const char *name,
		      struct f_fdb_child **child);
/* call the callback for every fdb_child */
int fdb_callback_each(struct device *dev,
		      int (*callback)(struct f_fdb_child *child, void *arg),
		      void *arg);


int fdb_video_timings_to_var(struct fdb_video_timings *fvt,
			     struct fb_var_screeninfo *var);
int fdb_var_to_video_timings(struct fb_var_screeninfo *var,
			     struct fdb_video_timings *fvt);

void fdb_dump_video_timings(struct fdb_video_timings *fvt);

#endif
