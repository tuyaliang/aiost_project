/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pm.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/io.h>

#include "dmaengine.h"

#define DMACR	0x0	/* global */
#define DE	31
#define DS	30
#define PR	28
#define DH	24

#define DMACA	0x0	/* channel */
#define EB	31
#define PB	30
#define ST	29
#define IS	24
#define BT	20
#define BC	16
#define TC	0
#define DMACB	0x4
#define TT	30
#define MS	28
#define TW	26
#define FS	25
#define FD	24
#define RC	23
#define RS	22
#define RD	21
#define EI	20
#define CI	19
#define SS	16
#define SP	12
#define DP	8
#define DMACSA	0x8
#define DMACDA	0xc

#define S7X_HDMAC_BUSWIDTHS	BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_4_BYTES)

struct s7x_sg {
	dma_addr_t src;
	dma_addr_t dst;
	unsigned len;
};

struct s7x_desc {
	struct dma_async_tx_descriptor txd;
	struct list_head node;
	unsigned dir;
	unsigned sgl;
	int idx; /* -1 of inactive, index of sg[] active xfer */

	struct s7x_sg sg[0]; /* keep at the end */
};

struct s7x_dchan {
	struct dma_chan chan;
	struct s7x_dmac *sdmac;
	void __iomem *regs;
	unsigned irq, id;
	struct tasklet_struct tasklet;
	struct dma_slave_config	cfg;
	struct list_head descriptors;
	struct s7x_desc *active;
	spinlock_t lock;
};

struct s7x_dmac {
	struct dma_device dmac;
	void __iomem *regs;
	struct clk *clk;
	bool pri_rotate;

	struct s7x_dchan schan[0]; /* always at the end */
};

#define to_s7x_dchan(c)	container_of((c), struct s7x_dchan, chan)
#define to_s7x_desc(t)	container_of((t), struct s7x_desc, txd)

static unsigned hw_left(struct s7x_dchan *schan)
{
	struct s7x_desc *desc = schan->active;
	int idx = desc->idx;
	u32 done;

	if (!desc)
		return 0;

	if (desc->dir == DMA_DEV_TO_MEM)
		done = readl_relaxed(schan->regs + DMACDA) - desc->sg[idx].dst;
	else
		done = readl_relaxed(schan->regs + DMACSA) - desc->sg[idx].src;

	return desc->sg[idx].len - done;
}

/* Trigger the segment schan->active->sg[schan->active->idx] */
static void s7x_trigger(struct s7x_dchan *schan)
{
	struct s7x_desc *desc = schan->active;
	struct s7x_dmac *sdmac = schan->sdmac;
	dma_addr_t src, dst;
	unsigned burst, len;
	u32 width, val;
	int idx;

	if (!desc)
		return;

	idx = desc->idx;
	src = desc->sg[idx].src;
	dst = desc->sg[idx].dst;
	len = desc->sg[idx].len;

	val = BIT(DE);
	if (sdmac->pri_rotate)
		val |= BIT(PR);
	writel_relaxed(val, sdmac->regs + DMACR);

	writel_relaxed(src, schan->regs + DMACSA);
	writel_relaxed(dst, schan->regs + DMACDA);

	val = BIT(CI) | BIT(EI);
	if (desc->dir == DMA_MEM_TO_DEV) {
		val |= BIT(FD);
		width = schan->cfg.dst_addr_width;
		burst = schan->cfg.dst_maxburst;
	} else if (desc->dir == DMA_DEV_TO_MEM) {
		val |= BIT(FS);
		width = schan->cfg.src_addr_width;
		burst = schan->cfg.src_maxburst;
	} else {
		width = 4;
		burst = 1; /* 64byte buffer is shared among all channels */
	}
	val |= ((width >> 1) << TW);
	val |= (2 << MS);
	writel_relaxed(val, schan->regs + DMACB);

	val = schan->id << 24;
	if (burst == 16)
		val |= (0xf << 20);
	else if (burst == 8)
		val |= (0xd << 20);
	else if (burst == 4)
		val |= (0xb << 20);
	else
		val |= (0x0 << 20);
	burst *= width;
	val |= ((len / burst - 1) << TC);
	writel_relaxed(val, schan->regs + DMACA);
	val |= BIT(EB);
	if (desc->dir == DMA_MEM_TO_MEM)
		val |= BIT(ST);
	writel_relaxed(val, schan->regs + DMACA);
}

static void s7x_tasklet(unsigned long data)
{
	struct s7x_dchan *schan = (struct s7x_dchan *)data;
	struct s7x_desc *desc, *tmp;
	unsigned long flags;
	LIST_HEAD(completed);

	spin_lock_irqsave(&schan->lock, flags);

	/* pluck done descriptors */
	list_for_each_entry_safe(desc, tmp, &schan->descriptors, node) {
		if (dma_cookie_status(&schan->chan, desc->txd.cookie, NULL) !=
					DMA_COMPLETE)
			break;
		list_move_tail(&desc->node, &completed);
	}

	spin_unlock_irqrestore(&schan->lock, flags);

	/* now complete the descriptors */
	while (!list_empty(&completed)) {
		dma_async_tx_callback cb;
		void *param;

		desc = list_first_entry(&completed, typeof(*desc), node);
		list_del(&desc->node);
		cb = desc->txd.callback;
		param = desc->txd.callback_param;
		kfree(desc);
		if (cb)
			cb(param);
	}
}

static irqreturn_t s7x_hdmac_irq(int irq, void *data)
{
	struct s7x_dchan *schan = data;
	struct s7x_desc *desc;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&schan->lock, flags);

	/* Ack and Disable irqs */
	val = readl_relaxed(schan->regs + DMACB);
	val &= ~(0x7 << SS);
	writel_relaxed(val, schan->regs + DMACB);
	val &= ~BIT(EI);
	val &= ~BIT(CI);
	writel_relaxed(val, schan->regs + DMACB);

	if (!schan->active)
		goto exit_isr;

	/* Mark this segment done */
	desc = schan->active;
	if (desc->idx == desc->sgl - 1) {

		dma_cookie_complete(&desc->txd);

		if (list_is_last(&desc->node, &schan->descriptors)) {
			desc = NULL;
		} else {
			desc = list_next_entry(desc, node);
			desc->idx = 0;
		}
		tasklet_schedule(&schan->tasklet);
	} else {
		desc->idx++;
	}

	schan->active = desc;
	s7x_trigger(schan);

exit_isr:
	spin_unlock_irqrestore(&schan->lock, flags);
	return IRQ_HANDLED;
}

static void s7x_issue_pending(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	struct s7x_desc *desc;
	unsigned long flags;

	spin_lock_irqsave(&schan->lock, flags);
	if (!schan->active) {
		list_for_each_entry(desc, &schan->descriptors, node)
			if (dma_cookie_status(&schan->chan, desc->
					txd.cookie, NULL) != DMA_COMPLETE) {
				desc->idx = 0;
				schan->active = desc;
				break;
			}
		s7x_trigger(schan);
	}
	spin_unlock_irqrestore(&schan->lock, flags);
}

static dma_cookie_t s7x_tx_submit(struct dma_async_tx_descriptor *txd)
{
	struct s7x_dchan *schan = to_s7x_dchan(txd->chan);
	struct s7x_desc *desc = to_s7x_desc(txd);
	dma_cookie_t cookie;
	unsigned long flags;

	spin_lock_irqsave(&schan->lock, flags);
	cookie = dma_cookie_assign(txd);
	list_add_tail(&desc->node, &schan->descriptors);
	spin_unlock_irqrestore(&schan->lock, flags);

	s7x_issue_pending(txd->chan);

	return cookie;
}

static struct s7x_desc *
get_desc(struct s7x_dchan *schan, int sgl, unsigned long flags)
{
	struct dma_chan *chan = &schan->chan;
	struct s7x_desc *desc;

	desc = kzalloc(sizeof(*desc) +
			sgl * sizeof(desc->sg[0]), GFP_KERNEL);
	if (!desc)
		return NULL;

	INIT_LIST_HEAD(&desc->node);
	dma_async_tx_descriptor_init(&desc->txd, chan);
	desc->txd.tx_submit = s7x_tx_submit;
	desc->txd.callback = NULL;
	desc->txd.flags = flags;
	desc->txd.cookie = 0;
	async_tx_ack(&desc->txd);
	desc->sgl = sgl;
	desc->idx = -1;

	return desc;
}

static struct dma_async_tx_descriptor *
s7x_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	struct s7x_desc *desc;

	desc = get_desc(schan, 1, flags);
	if (!desc)
		return NULL;

	desc->sg[0].src = src;
	desc->sg[0].dst = dst;
	desc->sg[0].len = len;
	desc->dir = DMA_MEM_TO_MEM;
	return &desc->txd;
}

static struct dma_async_tx_descriptor *
s7x_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction dir,
		unsigned long flags, void *context)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	struct scatterlist *sg;
	struct s7x_desc *desc;
	int i;

	if (!sgl || !sg_len)
		return NULL;

	desc = get_desc(schan, sg_len, flags);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {

		if (dir == DMA_MEM_TO_DEV) {
			desc->sg[i].src = sg_dma_address(sg);
			desc->sg[i].dst = schan->cfg.dst_addr;
		} else {
			desc->sg[i].dst = sg_dma_address(sg);
			desc->sg[i].src = schan->cfg.src_addr;
		}
		desc->sg[i].len = sg_dma_len(sg);
	}
	desc->dir = dir;

	return &desc->txd;
}

static unsigned int remaining_len(struct s7x_desc *desc)
{
	struct s7x_dchan *schan = to_s7x_dchan(desc->txd.chan);
	unsigned int remain = 0;
	int i;

	for (i = desc->idx; i < desc->sgl; i++)
		if (schan->active == desc && i == desc->idx)
			remain += hw_left(schan);
		else
			remain += desc->sg[i].len;

	return remain;
}

static struct s7x_desc *
cookie_to_desc(struct s7x_dchan *schan, dma_cookie_t cookie)
{
	struct s7x_desc *desc;

	list_for_each_entry(desc, &schan->descriptors, node)
		if (desc->txd.cookie == cookie)
			return desc;

	return NULL;
}

static enum dma_status s7x_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	struct s7x_desc *desc;
	unsigned long flags;
	enum dma_status ret;
	u32 bytes;

	spin_lock_irqsave(&schan->lock, flags);

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		goto out;

	if (!txstate)
		goto out;

	desc = cookie_to_desc(schan, cookie);
	if (!desc) {
		ret = DMA_ERROR;
		goto out;
	}

	bytes = remaining_len(desc);
	dma_set_residue(txstate, bytes);
out:
	spin_unlock_irqrestore(&schan->lock, flags);
	return ret;
}

static int
s7x_device_config(struct dma_chan *chan, struct dma_slave_config *cfg)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	unsigned long flags;

	spin_lock_irqsave(&schan->lock, flags);
	schan->cfg = *cfg;
	spin_unlock_irqrestore(&schan->lock, flags);

	return 0;
}

static int s7x_device_pause(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&schan->lock, flags);
	val = readl_relaxed(schan->regs + DMACA);
	val |= BIT(PB);
	writel_relaxed(val, schan->regs + DMACA);
	spin_unlock_irqrestore(&schan->lock, flags);

	return 0;
}

static int s7x_device_resume(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&schan->lock, flags);
	val = readl_relaxed(schan->regs + DMACA);
	val &= ~BIT(PB);
	writel_relaxed(val, schan->regs + DMACA);
	spin_unlock_irqrestore(&schan->lock, flags);

	return 0;
}

static int s7x_device_terminate_all(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	struct s7x_desc *desc;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&schan->lock, flags);

	val = readl_relaxed(schan->regs + DMACA);
	val &= ~BIT(EB); /* disable the channel */
	writel_relaxed(val, schan->regs + DMACA);

	if (schan->active)
		desc = schan->active;
	else
		desc = list_first_entry_or_null(&schan->descriptors,
						typeof(*desc), node);
	if (!desc) {
		spin_unlock_irqrestore(&schan->lock, flags);
		return 0;
	}

	list_for_each_entry_from(desc, &schan->descriptors, node) {
		desc->idx = desc->sgl - 1; /* mark all done */
		dma_cookie_complete(&desc->txd);
		desc->txd.callback = NULL;
	}
	schan->active = NULL;

	spin_unlock_irqrestore(&schan->lock, flags);

	s7x_tasklet((unsigned long)schan);

	return 0;
}

static int s7x_alloc_chan_resources(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);
	unsigned long flags;

	spin_lock_irqsave(&schan->lock, flags);
	dma_cookie_init(chan);
	spin_unlock_irqrestore(&schan->lock, flags);

	return request_irq(schan->irq, s7x_hdmac_irq, 0, "HDMAC", schan);
}

static void s7x_free_chan_resources(struct dma_chan *chan)
{
	struct s7x_dchan *schan = to_s7x_dchan(chan);

	free_irq(schan->irq, schan);
}

static struct dma_chan *s7x_dmac_xlate(struct of_phandle_args *dma_spec,
				     struct of_dma *of_dma)
{
	struct s7x_dmac *sdmac = of_dma->of_dma_data;
	struct s7x_dchan *schan;
	struct dma_chan *chan;

	if (dma_spec->args_count != 1)
		return NULL;

	chan = dma_get_any_slave_channel(&sdmac->dmac);
	if (!chan)
		return NULL;

	schan = to_s7x_dchan(chan);
	schan->id = dma_spec->args[0];

	return chan;
}

static const struct of_device_id s7x_hdmac_dt_ids[] = {
	{.compatible = "socionext,mb86s7x_hdmac",},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, s7x_hdmac_dt_ids);

static int s7x_dmac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct s7x_dchan *schan;
	struct s7x_dmac *sdmac;
	struct resource *res;
	int err, i, count;

	count = of_irq_count(np);
	if (!count)
		return -EINVAL;

	sdmac = devm_kzalloc(&pdev->dev, sizeof(*sdmac) +
				sizeof(*schan) * count, GFP_KERNEL);
	if (!sdmac)
		return -ENOMEM;

	if (of_get_property(np, "priority-rotate", NULL))
		sdmac->pri_rotate = true;
	else
		sdmac->pri_rotate = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sdmac->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sdmac->regs))
		return PTR_ERR(sdmac->regs);

	sdmac->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdmac->clk))
		return PTR_ERR(sdmac->clk);

	err = clk_prepare_enable(sdmac->clk);
	if (err)
		return err;

	platform_set_drvdata(pdev, sdmac);

	INIT_LIST_HEAD(&sdmac->dmac.channels);
	for (i = 0; i < count; i++) {
		schan = &sdmac->schan[i];
		schan->irq = irq_of_parse_and_map(np, i);
		schan->sdmac = sdmac;
		schan->chan.device = &sdmac->dmac;
		list_add_tail(&schan->chan.device_node,
				&sdmac->dmac.channels);

		schan->regs = sdmac->regs + (i + 1) * 0x10;
		spin_lock_init(&schan->lock);

		INIT_LIST_HEAD(&schan->descriptors);

		tasklet_init(&schan->tasklet, s7x_tasklet,
				(unsigned long)schan);
	}

	dma_cap_set(DMA_SLAVE, sdmac->dmac.cap_mask);
	dma_cap_set(DMA_MEMCPY, sdmac->dmac.cap_mask);
	dma_cap_set(DMA_PRIVATE, sdmac->dmac.cap_mask);

	sdmac->dmac.device_alloc_chan_resources = s7x_alloc_chan_resources;
	sdmac->dmac.device_free_chan_resources = s7x_free_chan_resources;
	sdmac->dmac.device_prep_dma_memcpy = s7x_prep_dma_memcpy;
	sdmac->dmac.device_prep_slave_sg = s7x_prep_slave_sg;
	sdmac->dmac.device_issue_pending = s7x_issue_pending;
	sdmac->dmac.device_tx_status = s7x_tx_status;
	sdmac->dmac.device_config = s7x_device_config;
	sdmac->dmac.device_pause = s7x_device_pause;
	sdmac->dmac.device_resume = s7x_device_resume;
	sdmac->dmac.device_terminate_all = s7x_device_terminate_all;
	sdmac->dmac.src_addr_widths = S7X_HDMAC_BUSWIDTHS;
	sdmac->dmac.dst_addr_widths = S7X_HDMAC_BUSWIDTHS;
	sdmac->dmac.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	sdmac->dmac.dev = &pdev->dev;

	err = dma_async_device_register(&sdmac->dmac);
	if (err)
		goto err_reg;

	err = of_dma_controller_register(np, s7x_dmac_xlate, sdmac);
	if (err)
		goto err_xlate;

	return 0;

err_xlate:
	dma_async_device_unregister(&sdmac->dmac);
err_reg:
	clk_disable_unprepare(sdmac->clk);

	return err;
}

static int s7x_dmac_remove(struct platform_device *pdev)
{
	struct s7x_dmac *sdmac = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&sdmac->dmac);
	clk_disable_unprepare(sdmac->clk);

	return 0;
}

#ifdef CONFIG_PM
static int s7x_hdmac_rpm_suspend(struct device *dev)
{
	struct s7x_dmac *sdmac = dev_get_drvdata(dev);

	clk_disable_unprepare(sdmac->clk);
	return 0;
}

static int s7x_hdmac_rpm_resume(struct device *dev)
{
	struct s7x_dmac *sdmac = dev_get_drvdata(dev);

	return clk_prepare_enable(sdmac->clk);
}
#endif

static const struct dev_pm_ops s7x_dmac_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(s7x_hdmac_rpm_suspend, s7x_hdmac_rpm_resume, NULL)
};

static struct platform_driver s7x_dmac_driver = {
	.driver = {
		.name = "mb86s7x_hdmac",
		.pm = &s7x_dmac_dev_pm_ops,
		.of_match_table = of_match_ptr(s7x_hdmac_dt_ids),
	},
	.probe = s7x_dmac_probe,
	.remove = s7x_dmac_remove,
};
module_platform_driver(s7x_dmac_driver);

MODULE_DESCRIPTION("Socionext HDMAC DmaEngine driver");
MODULE_AUTHOR("Jassi Brar <jassisinghbrar@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mb86s7x_hdmac");
