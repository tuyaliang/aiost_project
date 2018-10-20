/*
 * f-disp8m-dpi stub dpi fdb head driver
 * Copyright (C) 2013 Linaro, Ltd for Socionext Semi
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

#include <video/fdb.h>

struct disp8m_dpi {
	struct f_fdb_child fdb_child;
	struct device *dev;
	void __iomem *base;
	int irq;
	u32 source_crtc_bitfield;
	struct clk *clk;

	struct f_fdb_child *bound; /* fdfb we are bound to */
};

static int disp8m_dpi_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_LVDS;
}

static int disp8m_dpi_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct disp8m_dpi *priv = dev_get_drvdata(fdb_child->dev);

	if (!priv->bound)
		return 0;

	if (!priv->bound->ops->get_timings)
		return 0;

	return priv->bound->ops->check_timings(priv->bound, timings);
}

static void disp8m_dpi_get_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct fb_var_screeninfo var;
	const char *mode;
	int ret;

	if (of_property_read_string(fdb_child->dev->of_node, "mode", &mode)) {
		dev_err(fdb_child->dev, "Missing mode\n");
		return;
	}

	ret = fdb_get_of_var(mode, &var);
	if (ret) {
		dev_err(fdb_child->dev, "Failed to get of var for %s\n", mode);
		return;
	}

	fdb_var_to_video_timings(&var, timings);
}

static bool disp8m_dpi_detect(struct f_fdb_child *fdb_child)
{
	struct f_fdb *bus = NULL;
	int n;

	bus = f_fdb_from_dev(fdb_child->dev);

	for (n = 0; n < bus->count_fdb_children; n++) {
		if (!bus->child[n]->ops)
			continue;
		if (!bus->child[n]->ops->get_connector_type)
			continue;
		if (bus->child[n]->ops->get_connector_type(bus->child[n]) ==
		    DRM_MODE_CONNECTOR_HDMIA) {
			return !bus->child[n]->ops->detect(bus->child[n]);
		}
	}

	return true;
}

static u32 disp8m_dpi_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct disp8m_dpi *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int disp8m_dpi_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct disp8m_dpi *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;
	dev_info(fdb_child->dev, "binding to source %s\n",
					dev_name(fdb_bound->dev));

	return 0;
}

struct f_fdb_ops disp8m_dpi_fdb_ops = {
	.get_connector_type = disp8m_dpi_get_connector_type,
	.check_timings = disp8m_dpi_check_timings,
	.get_timings = disp8m_dpi_get_timings,
	.detect = disp8m_dpi_detect,
	.get_source_crtc_bitfield = disp8m_dpi_get_source_crtc_bitfield,
	.bind_to_source = disp8m_dpi_bind_to_source,
};

static int
disp8m_dpi_probe(struct platform_device *pdev)
{
	struct disp8m_dpi *priv;
	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child,
					&disp8m_dpi_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "framebuffer registration failed\n");
		goto bail;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		return -EINVAL;
	}

	priv->clk = clk_get(&pdev->dev, "lcdclk");
	if (!IS_ERR(priv->clk))
		clk_prepare_enable(priv->clk);

	dev_info(&pdev->dev, "fdb disp8m DPI head initialized\n");

	return 0;

bail:
	kfree(priv);

	return ret;
}

static int disp8m_dpi_remove(struct platform_device *pdev)
{
	struct disp8m_dpi *priv = platform_get_drvdata(pdev);

	fdb_unregister(&pdev->dev, &priv->fdb_child);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int disp8m_dpi_suspend(struct platform_device *pdev,
							pm_message_t msg)
{
	/* suspend here */
	return 0;
}

static int disp8m_dpi_resume(struct platform_device *pdev)
{
	/* resume here */
	return 0;
}
#else
#define disp8m_dpi_suspend NULL
#define disp8m_dpi_resume NULL
#endif /* CONFIG_PM */

static const struct of_device_id disp8m_dpi_fb_dt_ids[] = {
	{ .compatible = "socionext,jdsdisp2b-dpi" },
	{ /* sentinel */ }
};

static struct platform_driver disp8m_dpi_driver = {
	.probe = disp8m_dpi_probe,
	.remove = disp8m_dpi_remove,
	.suspend = disp8m_dpi_suspend,
	.resume = disp8m_dpi_resume,
	.driver = {
		.name = "disp8m_dpi",
		.of_match_table = disp8m_dpi_fb_dt_ids,
	},
};

MODULE_DEVICE_TABLE(of, disp8m_dpi_fb_dt_ids);

static int __init disp8m_dpi_init(void)
{
	return platform_driver_register(&disp8m_dpi_driver);
}

static void __exit disp8m_dpi_exit(void)
{
	platform_driver_unregister(&disp8m_dpi_driver);
}

module_init(disp8m_dpi_init);
module_exit(disp8m_dpi_exit);

MODULE_LICENSE("GPL");
