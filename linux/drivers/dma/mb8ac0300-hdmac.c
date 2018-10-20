/*
 *  linux/arch/arm/mach-mb8ac0300/hdmac.c
 *
 *  FASP HDMAC registration and IRQ dispatching
 *
 * Copyright (C) 2011-2013 Socionext Semiconductor, Ltd
 * Copyright (C) 2013-2014 Linaro, Ltd  Andy Green <andy.green@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/* #define DEBUG */
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
#include <linux/platform_data/dma-mb8ac0300-hdmac.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#define STOP_TIMEOUT 2 /* 2s */


static struct mb8ac0300_hdmac_chip hdmac_chips[HDMAC_MAX_CHIPS];

int hdmac_get_channel(u32 channel, u8 autostart_flg)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags;
	struct device *dev;
	int ret;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d\n", channel);
		return -EINVAL;
	}

	if ((autostart_flg != HDMAC_AUTOSTART_ENABLE) &&
	    (autostart_flg != HDMAC_AUTOSTART_DISABLE)) {
		pr_err("hdmac%d:autostart_flg err... please set 0 or 1.\n"
			, channel);
		return -EINVAL;
	}

	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels) {
		pr_err("%s(): CHAN_INDEX[%d] is larger than channels numbers[%d]\n"
			, __func__, CHAN_INDEX(channel), chip->channels);
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "get_sync failed with err %d\n", ret);
		return -EINVAL;
	}

	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;
	dev_dbg(dev, "%s(): CHIP_INDEX(channel) is %d\n",
		__func__, CHIP_INDEX(channel));

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state != HDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		/*
		 * should be nonfatal to caller since he can choose another
		 * channel
		 */
		dev_dbg(dev, "hdmac chip %d, channel %d is busy\n"
			, CHIP_INDEX(channel), CHAN_INDEX(channel));
		return -EBUSY;
	}

	chan->state = HDMAC_IDLE;
	chan->dmaca = 0;
	chan->dmacb = 0;
	chan->autostart_flg = autostart_flg;

	INIT_LIST_HEAD(&chan->list);
	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_get_channel);

int hdmac_enqueue(u32 channel, struct hdmac_req *hdmac_req)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags;
	unsigned long software_trigger;
	unsigned long input_select;
	unsigned long beat;
	int ret = 0;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}

	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;
	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	software_trigger = hdmac_req->dmaca & HDMACA_ST_MASK;
	input_select = hdmac_req->dmaca & HDMACA_IS_MASK;
	beat = hdmac_req->dmaca & HDMACA_BT_MASK;

	if ((hdmac_req->dmacb & HDMACB_MS_MASK) == HDMACB_MS_DEMAND) {
		if (beat != HDMACA_BT_NORMAL && beat != HDMACA_BT_SINGLE)
			return -EINVAL;
		if (software_trigger == HDMACA_ST)
			return -EINVAL;
	}
	if (chan->state == HDMAC_PREPARE) {
		dev_err(dev, "hdmac%d:channel is free.\n", channel);
		return -EINVAL;
	}

	if (software_trigger == HDMACA_ST && input_select != HDMACA_IS_SW)
		return -EINVAL;

	spin_lock_irqsave(&chan->lock, flags);
	list_add_tail(&hdmac_req->node, &chan->list);

	if (chan->state == HDMAC_IDLE &&
	    chan->autostart_flg == HDMAC_AUTOSTART_ENABLE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		ret = hdmac_start(channel);
		if (ret)
			return ret;
	} else {
		spin_unlock_irqrestore(&chan->lock, flags);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_enqueue);

static inline void __hdmac_start(struct hdmac_chan *chan)
{
	struct hdmac_req *req;

	/* get DMA request from list */
	req = list_entry(chan->list.next, struct hdmac_req, node);

	/* config and start the channel going */
	writel(req->req_data.src, chan->chip->base + DMACSA(chan->number));
	writel(req->req_data.dst, chan->chip->base + DMACDA(chan->number));

	chan->dmacb = req->dmacb;
	writel(chan->dmacb, chan->chip->base + DMACB(chan->number));

	chan->dmaca = req->dmaca;
	writel(chan->dmaca | HDMACA_EB, chan->chip->base + DMACA(chan->number));
}

int hdmac_start(u32 channel)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}

	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;
	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	spin_lock_irqsave(&chan->lock, flags);

	switch (chan->state) {
	case HDMAC_PREPARE:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_err(dev, "hdmac%d:channel is free.\n", channel);
		return -EINVAL;

	case HDMAC_IDLE:
		if (chan->list.next == &chan->list) {
			spin_unlock_irqrestore(&chan->lock, flags);
			dev_dbg(dev, "hdmac%d:no request to start\n", channel);
			return 0;
		}
		__hdmac_start(chan);
		chan->state = HDMAC_RUNNING;
		spin_unlock_irqrestore(&chan->lock, flags);
		return 0;

	default:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_dbg(dev, "hdmac%d:channel is running\n", channel);
		return 0;
	}
}
EXPORT_SYMBOL_GPL(hdmac_start);

int hdmac_getposition(u32 channel, dma_addr_t *src, dma_addr_t *dst)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}

	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;

	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->state == HDMAC_PREPARE) {
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_err(dev, "hdmac%d:channel is free.\n", channel);
		return -EINVAL;
	}

	if (src)
		*src = readl(chan->chip->base + DMACSA(chan->number));
	if (dst)
		*dst = readl(chan->chip->base + DMACDA(chan->number));

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_getposition);

int hdmac_stop(u32 channel)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags, timeout, tmp, state;
	struct hdmac_req *req;
	int ret;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;

	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	/*
	 * stop DMA transfer
	 * If the DMA transfer mode is BURST mode, even if we set the disable
	 * bit the change of the register during a DMA transfer is reflected
	 * after the DMA transfer is completed.
	 */

	spin_lock_irqsave(&chan->lock, flags);
	switch (chan->state) {
	case HDMAC_PREPARE:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_err(dev, "hdmac%d:channel is free.\n", channel);
		return -EINVAL;
	case HDMAC_RUNNING:
		tmp = readl(chan->chip->base + DMACA(chan->number));
		if (tmp & HDMACA_EB) {
			tmp &= ~((unsigned long)HDMACA_EB);
			writel(tmp, chan->chip->base + DMACA(chan->number));
		}
		chan->state = HDMAC_STOP_REQUEST;
		tmp = readl(chan->chip->base + DMACA(chan->number));
		if ((tmp & (HDMACA_BC_MASK | HDMACA_TC_MASK)) ==
			(chan->dmaca & (HDMACA_BC_MASK | HDMACA_TC_MASK))) {
			/*
			 * Before the dma transfer started by hardware trigger,
			 * even if we clear the EB bit, the interrupt will
			 * never raise. We check the BC & TC bits, if the
			 * transfer has not begun yet, we do the same pross
			 * as irq handler.
			 * We change the state to HDMAC_IDLE at the end of
			 * process.
			 * Then hdmac_flush & hdmac_start will not work
			 * which is in the callback handler.
			 */
			/* Stop Status */
			tmp = readl(chan->chip->base + DMACB(chan->number));
			state = tmp & HDMACB_SS_MASK;
			tmp &= ~((unsigned long)HDMACB_SS_MASK);
			writel(tmp, chan->chip->base + DMACB(chan->number));

			req = list_entry(chan->list.next, struct hdmac_req,
					node);
			list_del(chan->list.next);
			if (req->req_data.callback_fn != NULL) {
				spin_unlock(&chan->lock);
				req->req_data.callback_fn(
					OUTER_CHAN(chan->chip->chip_index
						, chan->number),
					req->req_data.irq_data, state);
				spin_lock(&chan->lock);
			}
			chan->state = HDMAC_IDLE;
			spin_unlock_irqrestore(&chan->lock, flags);
			return 0;
		}
		break;
	case HDMAC_STOP_REQUEST_NOWAIT:
		chan->state = HDMAC_STOP_REQUEST;
		break;
	default:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_dbg(dev, "hdmac%d:is already stopped\n", channel);
		return 0;
	}

	reinit_completion(&chan->stop_completion);
	spin_unlock_irqrestore(&chan->lock, flags);

	timeout = HZ * STOP_TIMEOUT;
	ret = wait_for_completion_timeout(&chan->stop_completion, timeout);
	if (!ret) { /* -ETIME */
		dev_err(dev, "hdmac%d:err stop time out", channel);
		return -ETIME;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_stop);

int hdmac_stop_nowait(u32 channel)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long tmp, flags, state;
	struct hdmac_req *req;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;

	dev = chip->dev;
	chan = &chip->chans[CHAN_INDEX(channel)];

	/* stop DMA transfer */

	/*
	 * If the DMA transfer mode is BURST mode, even if we set the disable
	 * bit the change of the register during a DMA transfer is reflected
	 * after the DMA transfer is completed.
	 */
	spin_lock_irqsave(&chan->lock, flags);
	switch(chan->state) {
	case HDMAC_PREPARE:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_err(dev, "hdmac%d:channel is free.\n", channel);
		return -EINVAL;
	case HDMAC_RUNNING:
		/* clear EB */
		tmp = readl(chan->chip->base + DMACA(chan->number));
		if (tmp & HDMACA_EB) {
			tmp &= ~((unsigned long)HDMACA_EB);
			writel(tmp, chan->chip->base + DMACA(chan->number));
		}
		chan->state = HDMAC_STOP_REQUEST_NOWAIT;
		tmp = readl(chan->chip->base + DMACA(chan->number));
		if ((tmp & (HDMACA_BC_MASK | HDMACA_TC_MASK)) !=
		    (chan->dmaca & (HDMACA_BC_MASK | HDMACA_TC_MASK)))
			break;
		/*
		 * Before the dma transfer started by hardware trigger,
		 * even if we clear the EB bit, the interrupt will
		 * never raise. We check the BC & TC bits, if the
		 * transfer has not begun yet, we do the same pross
		 * as irq handler.
		 * We change the state to HDMAC_IDLE before call
		 * the callback handler.
		 * Then hdmac_flush & hdmac_start can work which is in
		 * the callback handler.
		 */
		chan->state = HDMAC_IDLE;
		/* Stop Status */
		tmp = readl(chan->chip->base + DMACB(chan->number));
		state = tmp & HDMACB_SS_MASK;
		tmp &= ~((unsigned long)HDMACB_SS_MASK);
		writel(tmp, chan->chip->base + DMACB(chan->number));

		req = list_entry(chan->list.next, struct hdmac_req, node);
		list_del(chan->list.next);

		if (req->req_data.callback_fn == NULL)
			break;

		spin_unlock(&chan->lock);
		req->req_data.callback_fn(OUTER_CHAN(chan->chip->chip_index,
						     chan->number),
					  req->req_data.irq_data, state);
		spin_lock(&chan->lock);
		break;
	}

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_stop_nowait);

static inline void __hdmac_flush(struct hdmac_chan *chan)
{
	struct hdmac_req *req;

	while (chan->list.next != &chan->list) {
		req = list_entry(chan->list.next, struct hdmac_req, node);
		list_del(chan->list.next);
		if (!req->req_data.callback_fn)
			continue;
		req->req_data.callback_fn(
			OUTER_CHAN(chan->chip->chip_index, chan->number),
				req->req_data.irq_data, HDMAC_REQ_DATA_FLUSHED);
	}
}

int hdmac_flush(u32 channel)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	unsigned long flags;
	int ret = 0;
	struct device *dev;

	if (channel >= HDMAC_MAX_CHANNELS) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;

	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	spin_lock_irqsave(&chan->lock, flags);

	switch (chan->state) {
	case HDMAC_PREPARE:
		dev_err(dev, "hdmac%d: Flush free channel.\n", channel);
		ret = -EINVAL;
		break;
	case HDMAC_IDLE:
		__hdmac_flush(chan);
		break;
	default:
		dev_err(dev, "hdmac%d: Flush busy channel.\n", channel);
		ret = -EBUSY;
		break;
	}

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(hdmac_flush);

int hdmac_free(u32 channel)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	int ret = 0;
	unsigned long flags;
	struct device *dev;

	if (channel > HDMAC_MAX_CHANNELS - 1) {
		pr_err("hdmac:Invalid channel num %d.\n", channel);
		return -EINVAL;
	}
	chip = &hdmac_chips[CHIP_INDEX(channel)];
	if (CHAN_INDEX(channel) > chip->channels)
		return -EINVAL;
	chan = &chip->chans[CHAN_INDEX(channel)];
	dev = chip->dev;

	spin_lock_irqsave(&chan->lock, flags);

	switch (chan->state) {
	case HDMAC_PREPARE:
		dev_err(dev, "hdmac%d:freeing free channel.\n", channel);
		ret = -EINVAL;
		break;
	case HDMAC_RUNNING:
		spin_unlock_irqrestore(&chan->lock, flags);
		dev_dbg(dev, "hdmac%d:stop the channel\n", channel);
		ret = hdmac_stop(channel);
		if (ret)
			return ret;
		spin_lock_irqsave(&chan->lock, flags);
		/* zero return means IDLE then need do flush job further */
	case HDMAC_IDLE:
		__hdmac_flush(chan);
		chan->state = HDMAC_PREPARE;
		pm_runtime_put(chip->dev);
		break;
	default:
		dev_err(dev, "hdmac%d:free channel fault.\n", channel);
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&chan->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(hdmac_free);

static irqreturn_t hdmac_irq(int irq, void *devpw)
{
	struct hdmac_chan *chan = (struct hdmac_chan *)devpw;
	struct hdmac_req *req;
	unsigned long state, tmp, pre_state = 0;
	struct device *dev = chan->chip->dev;

	/* Stop Status */
	tmp = readl(chan->chip->base + DMACB(chan->number));
	state = tmp & HDMACB_SS_MASK;
	tmp &= ~((unsigned long)HDMACB_SS_MASK);
	writel(tmp, chan->chip->base + DMACB(chan->number));

	spin_lock(&chan->lock);
	switch (chan->state) {
	case HDMAC_RUNNING:
		chan->state = HDMAC_IDLE;
		break;
	case HDMAC_STOP_REQUEST_NOWAIT:
		chan->state = HDMAC_IDLE;
		pre_state = HDMAC_STOP_REQUEST_NOWAIT;
		break;
	case HDMAC_PREPARE:
		dev_err(dev, "hdmac%d: IRQ in invalid state %d\n",
			OUTER_CHAN(chan->chip->chip_index, chan->number),
								chan->state);
		goto bail;
	}

	if (chan->list.next == &chan->list) {
		dev_err(dev, "hdmac%d: No DMA request\n",
			OUTER_CHAN(chan->chip->chip_index, chan->number));
		goto bail;
	}

	req = list_entry(chan->list.next, struct hdmac_req, node);

	list_del(chan->list.next);

	if (req->req_data.callback_fn != NULL) {
		spin_unlock(&chan->lock);
		req->req_data.callback_fn(
			OUTER_CHAN(chan->chip->chip_index, chan->number),
						req->req_data.irq_data, state);
		spin_lock(&chan->lock);
	}

	switch (chan->state) {
	case HDMAC_STOP_REQUEST:
		complete_all(&chan->stop_completion);
		chan->state = HDMAC_IDLE;
		break;
	case HDMAC_IDLE:
		if (chan->list.next == &chan->list)
			break;
		if (pre_state == HDMAC_STOP_REQUEST_NOWAIT)
			break;
		__hdmac_start(chan);
		chan->state = HDMAC_RUNNING;
		break;
	}

bail:
	spin_unlock(&chan->lock);

	return IRQ_HANDLED;
}

/* return 0 means successful */
static int mb8x_clk_control(struct device *dev, bool on)
{
	int ret, i;
	struct clk *clk;

	dev_dbg(dev, "%s() is started (on:%d).\n", __func__, on);

	if (!on)
		goto clock_off;

	for (i = 0;; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;

		ret = clk_prepare_enable(clk);
		if (ret) {
			dev_err(dev, "failed to enable clock[%d]\n", i);
			goto clock_off;
		}
		dev_info(dev, "enabled_clk_num[%d]\n", i+1);
	}
	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;

clock_off:
	for (i = 0;; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;

		clk_disable_unprepare(clk);
		dev_info(dev, "disabled_clk_num[%d]\n", i+1);
	}
	dev_dbg(dev, "%s() is ended.\n", __func__);
	return on;
}
static int mb8ac0300_hdmac_probe(struct platform_device *pdev)
{
	struct mb8ac0300_hdmac_chip *chip;
	struct hdmac_chan *chan;
	long channel = 0;
	int ret;
	int chip_index;
	int channels;
	int rotation = 0;
	const int *p;
	struct resource *res;
	struct device *dev = &pdev->dev;

	if (!pdev->dev.of_node) {
		dev_err(dev, "Requires DT node\n");
		return -ENODEV;
	}

	p = of_get_property(pdev->dev.of_node, "chip", NULL);
	if (!p) {
		dev_err(dev, "Requires DT \"chip\" property\n");
		return -ENODEV;
	}
	chip_index = be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "rotation", NULL);
	if (!p) {
		dev_err(dev, "Requires DT \"rotation\" property\n");
		return -ENODEV;
	}
	rotation = be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "channels", NULL);
	if (!p) {
		dev_err(dev, "Requires DT \"channels\" property\n");
		return -ENODEV;
	}
	channels = be32_to_cpu(*p);

	if (chip_index >= HDMAC_MAX_CHIPS) {
		dev_err(dev, "Invalid chip num %d.\n", chip_index);
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	chip = &hdmac_chips[chip_index];
	chip->chip_index = chip_index;
	chip->channels = channels;
	chip->dev = dev;
	dev_set_drvdata(&pdev->dev, chip);
	/* set base address */
	chip->base = ioremap(res->start, res->end - res->start + 1);
	if (!chip->base) {
		dev_err(dev, "unable to map mem region\n");
		return -EBUSY;
	}

	/* init chan information */
	for (channel = 0; channel < chip->channels; channel++) {
		chan = &chip->chans[channel];
		memset(chan, 0, sizeof(struct hdmac_chan));
		chan->chip   = chip;
		chan->number = channel;
		chan->state  = HDMAC_PREPARE;

		spin_lock_init(&chan->lock);
		init_completion(&chan->stop_completion);

		res = platform_get_resource(pdev, IORESOURCE_IRQ, channel);
		if (!res) {
			ret = -EINVAL;
			goto bail2;
		}
		chan->irq = res->start;

		sprintf(chan->name, "hdmac-%d-%d", chip_index, (int)channel);
		ret = request_irq(chan->irq, hdmac_irq, IRQF_TRIGGER_HIGH,
						chan->name, (void *)chan);
		if (ret) {
			dev_err(dev, "ret = %d", ret);
			dev_err(dev, "hdmac%d:cannot get IRQ %d\n",
				chan->number, chan->irq);
			goto bail2;
		}
		disable_irq(chan->irq);
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(dev, "HDMAC %d started: channels: %d rotation: %d\n"
		, chip_index, channels, rotation);
	return 0;

bail2:
	while (--channel >= 0) {
		chan = &chip->chans[channel];
		if (chan->irq)
			free_irq(chan->irq, hdmac_irq);
	}

	iounmap(chip->base);

	return ret;
}

static int mb8ac0300_hdmac_remove(struct platform_device *pdev)
{
	struct mb8ac0300_hdmac_chip *chip = dev_get_drvdata(&pdev->dev);
	int channel;
	struct resource *res;

	for (channel = 0; channel < chip->channels; channel++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, channel);
		if (res)
			free_irq(res->start, &chip->chans[channel]);
	}

	iounmap(chip->base);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int mb8x_hdmac_runtime_suspend(struct device *dev)
{
	struct mb8ac0300_hdmac_chip *chip = dev_get_drvdata(dev);
	int channel;
	struct hdmac_chan *chan;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* disable irq */
	for (channel = 0; channel < chip->channels; channel++) {
		chan = &chip->chans[channel];
		disable_irq(chan->irq);
	}

	mb8x_clk_control(dev, false);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

static int mb8x_hdmac_runtime_resume(struct device *dev)
{
	struct mb8ac0300_hdmac_chip *chip = dev_get_drvdata(dev);
	int channel, ret;
	struct hdmac_chan *chan;
	const int *p;
	int rotation = 0;

	dev_dbg(dev, "%s() is started.\n", __func__);

	ret = mb8x_clk_control(dev, true);
	if (ret) {
		dev_err(dev, "%s failed to enable clock source\n", __func__);
		return -EINVAL;
	}

	/* re-configure and re-start dma controller for cold resume */
	p = of_get_property(dev->of_node, "rotation", NULL);
	if (!p) {
		dev_err(dev, "Requires DT \"rotation\" property\n");
		return -ENODEV;
	}
	rotation = be32_to_cpu(*p);

	if (rotation)
		writel(HDMACR_DE | HDMACR_PR, chip->base + DMACR);
	else
		writel(HDMACR_DE, chip->base + DMACR);

	/* enable irq */
	for (channel = 0; channel < chip->channels; channel++) {
		chan = &chip->chans[channel];
		enable_irq(chan->irq);
	}

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

static int mb8x_hdmac_suspend(struct device *dev)
{
	dev_dbg(dev, "%s() is executed.\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return mb8x_hdmac_runtime_suspend(dev);
}

static int mb8x_hdmac_resume(struct device *dev)
{
	dev_dbg(dev, "%s() is executed.\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return mb8x_hdmac_runtime_resume(dev);
}

static const struct dev_pm_ops mb8x_hdmac_ops = {
	.suspend = mb8x_hdmac_suspend,
	.resume = mb8x_hdmac_resume,
	SET_RUNTIME_PM_OPS(mb8x_hdmac_runtime_suspend
		, mb8x_hdmac_runtime_resume, NULL)
};

#define _mb8x_hdmac_ops (&mb8x_hdmac_ops)
#else
#define _mb8x_hdmac_ops NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id mb8ac0300_hdmac_dt_ids[] = {
	{ .compatible = "socionext,mb8ac0300-hdmac" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_hdmac_dt_ids);
#else
#define mb8ac0300_hdmac_dt_ids NULL
#endif

static struct platform_driver mb8ac0300_hdmac_driver = {
	.probe     = mb8ac0300_hdmac_probe,
	.remove    = mb8ac0300_hdmac_remove,
	.driver    = {
		.name  = "mb8ac0300-hdmac",
		.owner = THIS_MODULE,
		.pm = _mb8x_hdmac_ops,
		.of_match_table = mb8ac0300_hdmac_dt_ids,
	},
};

static int __init mb8ac0300_hdmac_driver_init(void)
{
	return platform_driver_register(&mb8ac0300_hdmac_driver);
}
subsys_initcall(mb8ac0300_hdmac_driver_init);

MODULE_AUTHOR("Socionext Semiconductor Limited");
MODULE_DESCRIPTION("MB8AC0300 HDMA Controller Driver");
MODULE_ALIAS("platform:mb8ac0300-hdmac");
MODULE_LICENSE("GPL");

