/*
 *  linux/arch/arm/mach-mb8ac0300/xdmac.c
 *
 * Copyright (C) 2011 SOCIONEXT SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/sched.h>

#include <linux/clk.h>

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_data/dma-mb8ac0300-xdmac.h>

#include <linux/io.h>

#define DRIVER_NAME		"mb8ac0300-xdmac"
#define DRIVER_DESC		"MB8AC0300 XDMA Controller Driver"

#define STOP_TIMEOUT 2 /* 2s */

/* XDMAC Virtual Base address */
static void *xdmac_base;
static struct xdmac_chan xdmac_chans[MAX_XDMAC_CHANNELS];

static int channels_in_use;

#ifdef CONFIG_PM
static u32 xdmac_xdacs;
#endif
static struct clk *xdmac_clk;

static inline unsigned long xdmac_readl(int reg)
{
	return readl(xdmac_base + reg);
}

static inline void xdmac_writel(unsigned long val, int reg)
{
	writel(val, xdmac_base + reg);
}

/* xdmac_get_channel - get control of a xdmac channel
 * @channel: channel number of xdmac.
 * @autostart_flg: set channel auto start data transfer.
 *
 * return -EINVAL    Invalid channel number,
 *        -BUSY      Channel used by other one,
 *        0          Otherwise.
 */

int xdmac_get_channel(u32 channel, u8 autostart_flg)
{
	struct xdmac_chan *chan;
	unsigned long flags;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}

	if (autostart_flg && (autostart_flg != 1)) {
		pr_err("xdmac:autostart_flg err.please set 0 or 1.\n");
		return -EINVAL;
	}

	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state != XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_info("xdmac%d:channel is busy\n", channel);
		return -EBUSY;
	}

	chan->state = XDMAC_IDLE;
	chan->autostart_flg = autostart_flg;
	chan->xdsac = 0;
	chan->xddac = 0;
	chan->xddcc = 0;
	chan->xddes = 0;
	chan->xddpc = 0;

	INIT_LIST_HEAD(&chan->list);

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL(xdmac_get_channel);

/* xdmac_enqueue - place the given request onto the queue
 *                 of operations for the channel.
 * @channel: channel number of xdmac.
 * @xdmac_req: request data of xdmac.
 *
 * retrun -EINVAL    Invalid channel number,
 *                   Invalid state,
 *                   Invalid parameters,
 *        -ENOMEM    Out of memory when alloc request,
 *        0          Otherwise.
 */

int xdmac_enqueue(u32 channel, struct xdmac_req *xdmac_req)
{
	struct xdmac_chan *chan;
	unsigned long flags;
	int ret;
	unsigned long software_enable;
	unsigned long transfer_factor;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:channel is free.\n", channel);
		return -EINVAL;
	}

	software_enable = xdmac_req->xddes & XDDES_SE_MASK;
	transfer_factor = xdmac_req->xddes & XDDES_TF_MASK;
	/* parameters check */
	if ((software_enable == XDDES_SE) &&
			(transfer_factor != XDDES_TF_SOFTWEARE)) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return -EINVAL;
	}

	list_add_tail(&xdmac_req->node, &chan->list);

	if ((chan->state == XDMAC_IDLE) && (chan->autostart_flg) == 1) {
		spin_unlock_irqrestore(&chan->lock, flags);
		ret = xdmac_start(channel);
		if (ret)
			return ret;
	} else
		spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL(xdmac_enqueue);

static inline void __xdmac_start(struct xdmac_chan *chan)
{
	struct xdmac_req *req;

	/* get DMA request from list */
	req = list_entry(chan->list.next, struct xdmac_req, node);

	chan->xdsac = req->xdsac;
	chan->xddac = req->xddac;
	chan->xddcc = req->xddcc;
	chan->xddes = req->xddes;
	chan->xddpc = req->xddpc;

	/* config and start the channel going */
	xdmac_writel((req->req_data.size - 1), XDTBC(chan->number));
	xdmac_writel(req->req_data.src, XDSSA(chan->number));
	xdmac_writel(req->req_data.dst, XDDSA(chan->number));
	xdmac_writel(chan->xdsac, XDSAC(chan->number));
	xdmac_writel(chan->xddac, XDDAC(chan->number));
	xdmac_writel(chan->xddcc, XDDCC(chan->number));
	xdmac_writel(chan->xddpc, XDDPC(chan->number));
	xdmac_writel(chan->xddes | XDDES_CE, XDDES(chan->number));
}

/* xdmac_start - start a xdma channel going
 * @channel: channel number of xdmac.
 *
 * retrun -EINVAL    Invalid channel number,
 *                   Invalid state,
 *        0          Otherwise.
 */

int xdmac_start(u32 channel)
{
	unsigned long flags;
	struct xdmac_chan *chan;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:channel is free.\n", channel);
		return -EINVAL;
	} else if (chan->state != XDMAC_IDLE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_debug("xdmac%d:channel is running\n", channel);
		return 0;
	}

	if (chan->list.next == &chan->list) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_debug("xdmac%d:no request to start\n", channel);
		return 0;
	}

	__xdmac_start(chan);

	chan->state = XDMAC_RUNNING;

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL(xdmac_start);

/* xdmac_getposition -Returns the current transfer points for
 *                    the dma source and destination
 * @channel: channel number of xdmac.
 * @src:     source address buffer pointer.
 * @dst:     destination address buffer pointer.
 *
 * retrun -EINVAL    Invalid channel number
 *        0          Otherwise.
 */

int xdmac_getposition(u32 channel, u32 *src, u32 *dst)
{
	struct xdmac_chan *chan;
	unsigned long flags;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}

	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdma%d:channel is free.\n", channel);
		return -EINVAL;
	}

	if (src != NULL)
		*src = xdmac_readl(XDSSA(channel));
	if (dst != NULL)
		*dst = xdmac_readl(XDDSA(channel));

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL(xdmac_getposition);


/* xdmac_stop(synchronous) -Stop a dma channel transfer.
 * @channel: channel number of xdmac.
 *
 * retrun -EINVAL    Invalid channel number,
 *        0          Otherwise.
 */

int xdmac_stop(u32 channel)
{
	struct xdmac_chan *chan;
	unsigned long tmp, flags, timeout;
	int ret;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	/* stop DMA transfer */

	/* If the DMA transfer mode is BURST mode, even if we set the disable
	 * bit the change of the register during a DMA transfer is reflected
	 * after the DMA transfer is completed.
	 */
	spin_lock_irqsave(&chan->lock, flags);
	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:channel is free.\n", channel);
		return -EINVAL;
	}
	if (chan->state == XDMAC_RUNNING) {
		/* clear CE */
		tmp = xdmac_readl(XDDES(channel));
		if (tmp & XDDES_CE) {
			tmp &= ~((unsigned long)XDDES_CE);
			xdmac_writel(tmp, XDDES(channel));
		}
		chan->state = XDMAC_STOP_REQUEST;
	} else if (chan->state == XDMAC_STOP_REQUEST_NOWAIT) {
		chan->state = XDMAC_STOP_REQUEST;
	} else { /* state is IDLE */
		spin_unlock_irqrestore(&chan->lock, flags);
		return 0;
	}

	reinit_completion(&chan->stop_completion);

	spin_unlock_irqrestore(&chan->lock, flags);

	timeout = HZ * STOP_TIMEOUT;
	ret = wait_for_completion_timeout(&chan->stop_completion, timeout);
	if (!ret) { /* -ETIME */
		pr_err("xdmac%d:err stop time out", channel);
		return -ETIME;
	}
	return 0;
}
EXPORT_SYMBOL(xdmac_stop);

int xdmac_stop_nowait(u32 channel)
{
	struct xdmac_chan *chan;
	unsigned long tmp, flags;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	/* stop DMA transfer */

	/* If the DMA transfer mode is BURST mode, even if we set the disable
	 * bit the change of the register during a DMA transfer is reflected
	 * after the DMA transfer is completed.
	 */
	spin_lock_irqsave(&chan->lock, flags);
	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:channel is free.\n", channel);
		return -EINVAL;
	}
	if (chan->state == XDMAC_RUNNING) {
		/* clear CE */
		tmp = xdmac_readl(XDDES(channel));
		if (tmp & XDDES_CE) {
			tmp &= ~((unsigned long)XDDES_CE);
			xdmac_writel(tmp, XDDES(channel));
		}
		chan->state = XDMAC_STOP_REQUEST_NOWAIT;

	} /* else state is IDLE & STOP_REQUEST */

	spin_unlock_irqrestore(&chan->lock, flags);
	/* do not wait for channel stop */
	return 0;
}
EXPORT_SYMBOL(xdmac_stop_nowait);

static void __xdmac_flush(struct xdmac_chan *chan)
{
	struct xdmac_req *req;

	while (chan->list.next != &chan->list) {
		req = list_entry(chan->list.next, struct xdmac_req, node);
		list_del(chan->list.next);
		if (req->req_data.callback_fn != NULL)
			req->req_data.callback_fn(chan->number,
			       req->req_data.irq_data, XDMAC_REQ_DATA_FLUSHED);
	}
}

int xdmac_flush(u32 channel)
{
	struct xdmac_chan *chan;
	unsigned long flags;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_debug("xdmac%d:Flush free channel.\n", channel);
		return -EINVAL;
	} else if (chan->state != XDMAC_IDLE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:Flush busy channel.\n", channel);
		return -EBUSY;
	}
	__xdmac_flush(chan);

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL(xdmac_flush);

/* xdmac_free - free the xdma channel
 *              (will also abort any outstanding operations)
 * @channel: channel number of xdmac.
 *
 * retrun -EINVAL    Invalid channel number,
 *        0          Otherwise.
 */
int xdmac_free(u32 channel)
{
	struct xdmac_chan *chan;
	int ret;
	unsigned long flags;

	if (channel > MAX_XDMAC_CHANNELS - 1) {
		pr_err("xdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chan = &xdmac_chans[channel];

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == XDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_debug("xdmac%d:freeing free channel.\n", channel);
		return -EINVAL;
	} else if (chan->state == XDMAC_RUNNING) {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_debug("xdmac%d:stop the channel.\n", channel);
		ret = xdmac_stop(channel);
		if (ret)
			return ret;
		spin_lock_irqsave(&chan->lock, flags);
	}

	if (chan->state == XDMAC_IDLE) {
		__xdmac_flush(chan);
		chan->state = XDMAC_PREPARE;
		spin_unlock_irqrestore(&chan->lock, flags);
	} else {
		spin_unlock_irqrestore(&chan->lock, flags);
		pr_err("xdmac%d:free channel fault.\n", channel);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(xdmac_free);

/* irq handler */

static irqreturn_t xdmac_irq(int irq, void *devpw)
{
	struct xdmac_chan *chan = (struct xdmac_chan *)devpw;
	struct xdmac_req *req;
	unsigned long state, tmp, pre_state = 0;

	/* get stop state */
	tmp = xdmac_readl(XDDSD(chan->number));
	state = tmp & XDDSD_IS_MASK;
	tmp &= ~((unsigned long)XDDSD_IS_MASK);
	xdmac_writel(tmp, XDDSD(chan->number));

	spin_lock(&chan->lock);
	if (chan->state == XDMAC_RUNNING)
		chan->state = XDMAC_IDLE;
	else if (chan->state == XDMAC_STOP_REQUEST_NOWAIT) {
		chan->state = XDMAC_IDLE;
		pre_state = XDMAC_STOP_REQUEST_NOWAIT;
	}

	/* modify the channel state */
	if (chan->state == XDMAC_PREPARE) {
		spin_unlock(&chan->lock);
		pr_err("xdmac%d: IRQ in invalid state %d\n",
				chan->number, chan->state);
		return IRQ_HANDLED;
	}

	if (chan->list.next == &chan->list) {
		spin_unlock(&chan->lock);
		pr_err("xdmac%d: No DMA request\n", chan->number);
		return IRQ_HANDLED;
	}

	req = list_entry(chan->list.next, struct xdmac_req, node);

	list_del(chan->list.next);

	if (req->req_data.callback_fn != NULL) {
		spin_unlock(&chan->lock);
		req->req_data.callback_fn(chan->number,
					req->req_data.irq_data, state);
		spin_lock(&chan->lock);
	}

	if (chan->state == XDMAC_STOP_REQUEST)
		complete_all(&chan->stop_completion);

	if ((chan->state == XDMAC_IDLE) && (chan->list.next != &chan->list)
				&& (pre_state != XDMAC_STOP_REQUEST_NOWAIT)) {
		__xdmac_start(chan);
		chan->state = XDMAC_RUNNING;
	} else if (chan->state == XDMAC_STOP_REQUEST)
		chan->state = XDMAC_IDLE;

		/*
		 * else...
		 * XDMAC_RUNNING (xdmac started in callback_fn)
		 * XDMAC_IDLE & list is NULL
		 * XDMAC_IDLE & pre_state == XDMAC_STOP_REQUEST_NOWAIT
		 */


	spin_unlock(&chan->lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

static int xdmac_suspend_noirq(struct device *dev)
{
	struct xdmac_chan *chan;
	u32 channel;
	unsigned long tmp;

	for (channel = 0; channel < MAX_XDMAC_CHANNELS; channel++) {
		chan = &xdmac_chans[channel];
		if (chan->state == XDMAC_RUNNING) {
			dev_err(dev, "%s Channel%d is running\n",
							__func__, channel);
			return -EBUSY;
		}
	}

	tmp = xdmac_readl(XDACS);
	xdmac_xdacs = tmp;
	tmp &= ~((unsigned long)XDACS_XE);
	xdmac_writel(tmp, XDACS);

	clk_disable_unprepare(xdmac_clk);

	return 0;
}

static int xdmac_resume_noirq(struct device *dev)
{
	int ret;

	/* clk_enable */
	ret = clk_prepare_enable(xdmac_clk);
	if (ret) {
		dev_err(dev, "%s failed to enable clock source\n", __func__);
		return ret;
	}
	xdmac_writel(xdmac_xdacs, XDACS);

	return 0;
}
#else
	#define xdmac_suspend_noirq NULL
	#define xdmac_resume_noirq NULL
#endif

static const struct dev_pm_ops mb8ac0300_xdmac_dev_pm_ops = {
	.suspend_noirq = xdmac_suspend_noirq,
	.resume_noirq = xdmac_resume_noirq,
};

static int mb8ac0300_xdmac_probe(struct platform_device *pdev)
{
	struct xdmac_chan *chan;
	unsigned long channel;
	unsigned long tmp = 0;
	int ret;
	int low_power;
	int rotation = 0;
	const int *p;
	struct resource *res;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Requires DT node\n");
		return -ENODEV;
	}

	p = of_get_property(pdev->dev.of_node, "low_power", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Requires DT \"low_power\" property\n");
		return -ENODEV;
	}
	low_power = be32_to_cpu(*p);
	p = of_get_property(pdev->dev.of_node, "rotation", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Requires DT \"rotation\" property\n");
		return -ENODEV;
	}
	rotation = be32_to_cpu(*p);
	p = of_get_property(pdev->dev.of_node, "channels", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Requires DT \"channels\" property\n");
		return -ENODEV;
	}
	channels_in_use = be32_to_cpu(*p);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/* set base address */
	xdmac_base = ioremap(res->start, res->end - res->start + 1);
	if (!xdmac_base) {
		dev_err(&pdev->dev, "unable to map mem region\n");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		ret = -EINVAL;
		goto bail1;
	}

	for (channel = 0; channel < channels_in_use; channel++) {
		chan = &xdmac_chans[channel];
		memset(chan, 0, sizeof(struct xdmac_chan));

		chan->number = channel;

		chan->irq    = res->start + channel;
		chan->state  = XDMAC_PREPARE;
		spin_lock_init(&chan->lock);
		init_completion(&chan->stop_completion);

		ret = request_irq(chan->irq, xdmac_irq, 0,
					"xdmac", (void *)chan);
		if (ret) {
			dev_err(&pdev->dev, "xdmac%d:cannot get IRQ %d\n",
						       chan->irq, chan->number);
			goto bail2;
		}
	}

	xdmac_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(xdmac_clk)) {
		dev_err(&pdev->dev, "Unable to get clock\n");
		ret = -EINVAL;
		goto bail2;
	}

	ret = clk_prepare_enable(xdmac_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock source\n");
		goto bail3;
	}

	if (rotation)
		tmp |= XDACS_CP;
	if (low_power)
		tmp |= XDACS_LP;

	tmp |= XDACS_XE;
	xdmac_writel(tmp, XDACS);

	return 0;

bail3:
	clk_put(xdmac_clk);

bail2:
	while (--channel >= 0) {
		chan = &xdmac_chans[channel];
		free_irq(chan->irq, xdmac_irq);
	}
bail1:
	iounmap(xdmac_base);

	return ret;
}

static int mb8ac0300_xdmac_remove(struct platform_device *pdev)
{
	int channel;

	for (channel = 0; channel < channels_in_use; channel++)
		free_irq(xdmac_chans[channel].irq,
					&xdmac_chans[channel]);

	iounmap(xdmac_base);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mb8ac0300_xdmac_dt_ids[] = {
	{ .compatible = "socionext,mb8ac0300-xdmac" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_xdmac_dt_ids);
#else
#define mb8ac0300_xdmac_dt_ids NULL
#endif

static struct platform_driver mb8ac0300_xdmac_driver = {
	.probe     = mb8ac0300_xdmac_probe,
	.remove    = mb8ac0300_xdmac_remove,
	.driver    = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb8ac0300_xdmac_dt_ids,
		.pm = &mb8ac0300_xdmac_dev_pm_ops,
	},
};

static int __init mb8ac0300_xdmac_driver_init(void)
{
	return platform_driver_register(&mb8ac0300_xdmac_driver);
}
subsys_initcall(mb8ac0300_xdmac_driver_init);

MODULE_AUTHOR("Socionext");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");


