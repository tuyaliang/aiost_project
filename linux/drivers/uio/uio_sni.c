/*
 * SNI UIO Driver
 * Copyright (C) 2015 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 *
 * This driver exposes up to 5 physical memory regions set in DT using the
 * UIO subsystem, along with an optional irq.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

static irqreturn_t uiosni_handler(int irq, struct uio_info *info)
{
	/* void __iomem *regs = info->mem[0].internal_addr; */

	/* handle the irq */

	return IRQ_HANDLED;
}

static int uiosni_probe(struct platform_device *pdev)
{
	struct uio_info *info;
	struct resource *r;
	int ret = -EINVAL;
	const char *name;
	int n;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	for (n = 0; n < ARRAY_SIZE(info->mem); n++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, n);
		if (!r) {
			if (!n) {
				dev_err(&pdev->dev, "At least one reg= required\n");
				ret = -ENXIO;
				goto err;
			}
			break;
		}
		if (!of_property_read_string_index(pdev->dev.of_node,
						   "reg-names", n, &name))
			info->mem[n].name = name;
		info->mem[n].addr = r->start;
		if (!info->mem[n].addr)
			goto out_unmap;
		info->mem[n].internal_addr = ioremap_cache(r->start, resource_size(r));
		if (!info->mem[n].internal_addr) {
			dev_err(&pdev->dev, "mapping region 0x%X len 0x%X failed\n",
				r->start, resource_size(r)
			);
			goto out_unmap;
		}

		info->mem[n].size = resource_size(r);
		info->mem[n].memtype = UIO_MEM_PHYS;
	}
	if (!of_property_read_string(pdev->dev.of_node, "uioname", &name))
		info->name = name;
	else
		info->name = "unknown";

	info->version = "0.0.1";

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq > 0) {
		info->irq_flags = IRQF_SHARED;
		info->handler = uiosni_handler;
	} else
		info->irq = 0;

	if (uio_register_device(&pdev->dev, info)) {
		dev_err(&pdev->dev, "registration failed\n");
		goto out_unmap;
	}

	platform_set_drvdata(pdev, info);

	return 0;

out_unmap:
	for (n = 0; n < ARRAY_SIZE(info->mem); n++)
		if (info->mem[n].internal_addr)
			iounmap(info->mem[n].internal_addr);
err:
	kfree(info);

	return ret;
}

static int uiosni_remove(struct platform_device *pdev)
{
	struct uio_info *info = platform_get_drvdata(pdev);
	int n;

	uio_unregister_device(info);

	for (n = 0; n < ARRAY_SIZE(info->mem); n++)
		if (info->mem[n].internal_addr)
			iounmap(info->mem[n].internal_addr);

	kfree(info);
	platform_set_drvdata(pdev, NULL);

	return 0;
};

static const struct of_device_id uiosni_dt_ids[] = {
	{ .compatible = "socionext,uiosni" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, uiosni_dt_ids);

static struct platform_driver uiosni_driver = {
	.probe   = uiosni_probe,
	.remove  = uiosni_remove,
	.driver  = {
		.owner = THIS_MODULE,
		.name = "uiosni",
		.of_match_table = uiosni_dt_ids,
	},
};

module_platform_driver(uiosni_driver);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("Socionext UIO Driver");
MODULE_LICENSE("GPL");
