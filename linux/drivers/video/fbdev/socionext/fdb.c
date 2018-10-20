/*
 * drivers/video/fbdev/socionext/fdb.c - Socionext Display Bus
 * Copyright (C) 2013-2014 Linaro, Ltd for Socionext Semiconductor, Ltd
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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <video/fdb.h>

static struct bus_type bus_type;

static int
dt_int(struct device_node *node, const char *string)
{
	u32 ret = 0;

	if (of_property_read_u32(node, string, &ret))
		ret = -1;
	return ret;
}
static int
dt_int_idx(struct device_node *node, const char *string, int idx)
{
	u32 ret = 0;

	if (of_property_read_u32_index(node, string, idx, &ret))
		ret = -1;
	return ret;
}


void
fdb_dump_video_timings(struct fdb_video_timings *fvt)
{
	struct display_timing *dt = &fvt->dt;

	pr_debug("  hactive %d\n", dt->hactive.typ);
	pr_debug("  xres_virtual %d\n", fvt->xres_virtual);
	pr_debug("  hfront_porch %d\n", dt->hfront_porch.typ);
	pr_debug("  hback_porch %d\n", dt->hback_porch.typ);
	pr_debug("  hsync_len %d\n", dt->hsync_len.typ);
	pr_debug("  vactive %d\n", dt->vactive.typ);
	pr_debug("  yres_virtual %d\n", fvt->yres_virtual);
	pr_debug("  vfront_porch %d\n", dt->vfront_porch.typ);
	pr_debug("  vback_porch %d\n", dt->vback_porch.typ);
	pr_debug("  vsync_len %d\n", dt->vsync_len.typ);
	pr_debug("  bits_per_pixel %d\n", fvt->bits_per_pixel);
	pr_debug("  stride_px %d\n", fvt->stride_px);
	pr_debug("  width_mm %d\n", fvt->width_mm);
	pr_debug("  height_mm %d\n", fvt->height_mm);
	pr_debug("  pixelclock %d\n", dt->pixelclock.typ);
	pr_debug("  scanout_rotation %d\n", fvt->scanout_rotation);
	pr_debug("  flags 0x%x\n", dt->flags);
}
EXPORT_SYMBOL_GPL(fdb_dump_video_timings);

static int
fdb_copy_of_timings(struct device_node *node, struct fdb_video_timings *fvt)
{
	struct display_timing *dt = &fvt->dt;

	dt->hactive.typ = dt_int(node, "hactive");
	dt->vactive.typ = dt_int(node, "vactive");
	dt->hactive.max = dt_int_idx(node, "hactive", 1);
	dt->vactive.max = dt_int_idx(node, "vactive", 1);
	dt->hfront_porch.typ = dt_int(node, "hfront-porch");
	dt->vfront_porch.typ = dt_int(node, "vfront-porch");
	dt->hback_porch.typ = dt_int(node, "hback-porch");
	dt->vback_porch.typ = dt_int(node, "vback-porch");
	dt->hsync_len.typ = dt_int(node, "hsync-len");
	dt->vsync_len.typ = dt_int(node, "vsync-len");
	dt->pixelclock.typ = dt_int(node, "clock");
	dt->flags = dt_int(node, "flags");

	fvt->bits_per_pixel = dt_int(node, "bpp");
	fvt->xres_virtual = dt_int(node, "xres-virtual");
	if (fvt->xres_virtual < 0)
		fvt->xres_virtual = dt->hactive.typ;
	fvt->yres_virtual = dt_int(node, "yres-virtual");
	if (fvt->yres_virtual < 0)
		fvt->yres_virtual = dt->vactive.typ;
	fvt->stride_px = fvt->xres_virtual;

	fvt->scanout_rotation = dt_int(node, "scanout-rotation");
	fvt->width_mm = dt_int(node, "width-mm");
	fvt->height_mm = dt_int(node, "height-mm");

	fvt->red_offset = dt_int(node, "red_offset");
	fvt->red_length = dt_int(node, "red_length");
	fvt->green_offset = dt_int(node, "green_offset");
	fvt->green_length = dt_int(node, "green_length");
	fvt->blue_offset = dt_int(node, "blue_offset");
	fvt->blue_length = dt_int(node, "blue_length");
	fvt->alpha_offset = dt_int(node, "alpha_offset");
	fvt->alpha_length = dt_int(node, "alpha_length");

	return 0;
}

int
fdb_video_timings_to_var(struct fdb_video_timings *fvt,
			 struct fb_var_screeninfo *var)
{
	struct display_timing *dt = &fvt->dt;

	memset(var, 0, sizeof(*var));

	var->xres = dt->hactive.typ;
	var->xres_virtual = fvt->xres_virtual;
	var->reserved[2] = dt->hactive.max;
	var->yres = dt->vactive.typ;
	var->yres_virtual = fvt->yres_virtual;
	var->reserved[3] = dt->vactive.max;
	var->pixclock = dt->pixelclock.typ;
	var->sync = 0;

	if (dt->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		var->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (dt->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		var->sync |= FB_SYNC_VERT_HIGH_ACT;

	var->left_margin = (dt->hback_porch.typ - dt->hfront_porch.typ) * 2;
	var->right_margin = dt->hactive.typ + dt->hfront_porch.typ +
					dt->hsync_len.typ + dt->hback_porch.typ;
	var->upper_margin = dt->vback_porch.typ + dt->vfront_porch.typ + 3;
	var->lower_margin = dt->vactive.typ + dt->vfront_porch.typ +
					dt->vback_porch.typ + dt->vsync_len.typ;

	var->hsync_len = dt->hsync_len.typ;
	var->vsync_len = dt->vsync_len.typ;
	var->sync = 0;
	var->bits_per_pixel = fvt->bits_per_pixel;

	var->grayscale = 0;
	var->width = fvt->width_mm;
	var->height = fvt->height_mm;

	switch (fvt->scanout_rotation) {
	case 90:
		var->rotate = 1;
		break;
	case 180:
		var->rotate = 2;
		break;
	case 270:
		var->rotate = 3;
		break;
	default:
		var->rotate = 0;
		break;
	}

	var->reserved[0] = dt->hfront_porch.typ;
	var->reserved[1] = dt->vfront_porch.typ;

	var->activate = FB_ACTIVATE_NOW;
	var->vmode = FB_VMODE_NONINTERLACED;

	var->red.offset = fvt->red_offset;
	var->red.length = fvt->red_length;
	var->green.offset = fvt->green_offset;
	var->green.length = fvt->green_length;
	var->blue.offset = fvt->blue_offset;
	var->blue.length = fvt->blue_length;
	var->transp.offset = fvt->alpha_offset;
	var->transp.length = fvt->alpha_length;

	return 0;
}

int fdb_var_to_video_timings(struct fb_var_screeninfo *var,
			     struct fdb_video_timings *fvt)
{
	struct display_timing *dt = &fvt->dt;

	memset(fvt, 0, sizeof(*fvt));

	dt->hactive.typ = var->xres;
	dt->vactive.typ = var->yres;
	dt->hactive.max = var->reserved[2];
	dt->vactive.max = var->reserved[3];
	fvt->xres_virtual = var->xres_virtual;
	fvt->yres_virtual = var->yres_virtual;

	fvt->stride_px = var->xres_virtual;

	dt->pixelclock.typ = var->pixclock;
	dt->flags = 0;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		dt->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else
		dt->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		dt->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else
		dt->flags |= DISPLAY_FLAGS_VSYNC_LOW;

	dt->hsync_len.typ = var->hsync_len;
	dt->vsync_len.typ = var->vsync_len;
	fvt->bits_per_pixel = var->bits_per_pixel;
	dt->hfront_porch.typ = var->reserved[0];
	dt->vfront_porch.typ = var->reserved[1];

	dt->hback_porch.typ = (var->left_margin +
					(2 * dt->hfront_porch.typ)) / 2;
	dt->vback_porch.typ = var->upper_margin - (dt->vfront_porch.typ + 3);

	fvt->width_mm = var->width;
	fvt->height_mm = var->height;

	switch (var->rotate) {
	case 1:
		fvt->scanout_rotation = 90;
		break;
	case 2:
		fvt->scanout_rotation = 180;
		break;
	case 3:
		fvt->scanout_rotation = 270;
		break;
	default:
		fvt->scanout_rotation = 0;
		break;
	}

	fvt->red_offset = var->red.offset;
	fvt->red_length = var->red.length;
	fvt->green_offset = var->green.offset;
	fvt->green_length = var->green.length;
	fvt->blue_offset = var->blue.offset;
	fvt->blue_length = var->blue.length;
	fvt->alpha_offset = var->transp.offset;
	fvt->alpha_length = var->transp.length;

	return 0;
}
EXPORT_SYMBOL_GPL(fdb_var_to_video_timings);

int
fdb_get_of_var(const char *name, struct fb_var_screeninfo *var)
{
	struct device_node *node = NULL, *pnode;
	struct fdb_video_timings fvt;
	const char *p;

	pnode = of_find_compatible_node(node, NULL, "video-modes");
	if (!pnode) {
		pr_err("no video-modes node\n");
		return -ENOENT;
	}
	do {
		node = of_get_next_available_child(pnode, node);
		if (!node)
			continue;

		if (of_property_read_string(node, "mode", &p))
			continue;
		pr_debug("fdb_get_of_var: trying mode %s\n", p);
		if (strcmp(p, name))
			continue;
		fdb_copy_of_timings(node, &fvt);
		fdb_dump_video_timings(&fvt);
		fdb_video_timings_to_var(&fvt, var);

		return 0;
	} while (node);

	return -ENOENT;
}
EXPORT_SYMBOL_GPL(fdb_get_of_var);

int
fdb_register(struct device *dev, struct f_fdb_child *child,
	     struct f_fdb_ops *ops)
{
	struct f_fdb *bus = f_fdb_from_dev(dev);

	mutex_init(&child->lock);
	child->dev = dev;

	if (bus->count_fdb_children == FDB_MAX_BUS_CHILDREN) {
		dev_err(dev, "Reached max fdb children, rejecting\n");
		return -ENOMEM;
	}

	bus->child[bus->count_fdb_children++] = child;
	child->ops = ops;

	return 0;
}
EXPORT_SYMBOL_GPL(fdb_register);

int
fdb_unregister(struct device *dev, struct f_fdb_child *child)
{
	struct f_fdb *bus = f_fdb_from_dev(dev);
	int n;

	for (n = 0; n < bus->count_fdb_children; n++)
		if (bus->child[n] == child) {
			bus->child[n] = bus->child[bus->count_fdb_children - 1];
			bus->count_fdb_children--;
		}

	return 0;
}
EXPORT_SYMBOL_GPL(fdb_unregister);

int
fdb_get_peer_count(struct device *dev)
{
	struct f_fdb *bus = f_fdb_from_dev(dev);

	return bus->count_fdb_children;
}
EXPORT_SYMBOL_GPL(fdb_get_peer_count);

int
fdb_name_to_child(struct device *dev, const char *name,
		  struct f_fdb_child **child)
{
	struct f_fdb *bus = f_fdb_from_dev(dev);
	int n;

	for (n = 0; n < bus->count_fdb_children; n++) {
		if (!strcmp(of_node_full_name(bus->child[n]->dev->of_node),
			    name)) {
			*child = bus->child[n];

			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(fdb_name_to_child);

int fdb_callback_each(struct device *dev,
		      int (*callback)(struct f_fdb_child *child, void *arg),
		      void *arg)
{
	struct f_fdb *bus = f_fdb_from_dev(dev);
	int n;

	for (n = 0; n < bus->count_fdb_children; n++)
		if (callback(bus->child[n], arg))
			return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(fdb_callback_each);

static int
f_fdb_bus_match(struct device *dev, struct device_driver *driver)
{
	struct f_fdb *priv = dev_get_drvdata(dev);

	dev_err(dev, "bus_match. dev %s/%s, drv %s\n",
		dev_name(dev), priv->name, driver->name);

	return !strcmp(dev->driver->name, driver->name);
}

static ssize_t
device_name_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	struct f_fdb *priv = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			priv->name ?
			priv->name : "");
}

static struct device_attribute default_dev_attrs[] = {
	__ATTR(name, S_IRUGO, device_name_show, NULL),
	__ATTR_NULL,
};

static int
f_fdb_probe(struct platform_device *pdev)
{
	struct f_fdb *priv;
	const int *p;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	p = of_get_property(pdev->dev.of_node, "id", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Requires DT \"id\" property\n");
		ret = -ENODEV;
		goto bail1;
	}
	priv->id = be32_to_cpu(*p);

	platform_set_drvdata(pdev, priv);

	sprintf(priv->name, "f_fdb%d", priv->id & 31);
	bus_type.name = priv->name;
	bus_type.match = f_fdb_bus_match;
	bus_type.dev_attrs = default_dev_attrs;

	ret = bus_register(&bus_type);
	if (ret) {
		dev_err(&pdev->dev, "Bus registration for %s failed %d\n",
			priv->name, ret);
		goto bail1;
	}

	dev_info(&pdev->dev, "FDB initialized\n");

	return 0;

bail1:
	kfree(priv);

	return ret;
}

static int
f_fdb_remove(struct platform_device *pdev)
{
	struct f_fdb *priv = platform_get_drvdata(pdev);

	bus_unregister(&bus_type);
	kfree(priv);

	return 0;
}

static const struct of_device_id f_fdb_dt_ids[] = {
	{ .compatible = "socionext,f_fdb" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, f_fdb_dt_ids);

static struct platform_driver f_fdb_driver = {
	.probe = f_fdb_probe,
	.remove = f_fdb_remove,
	.driver = {
		.name = "f_fdb",
		.of_match_table = f_fdb_dt_ids,
	},
};

static int __init f_fdb_init(void)
{
	return platform_driver_register(&f_fdb_driver);
}

static void __exit f_fdb_exit(void)
{
	platform_driver_unregister(&f_fdb_driver);
}

module_init(f_fdb_init);
module_exit(f_fdb_exit);

MODULE_LICENSE("GPL");


