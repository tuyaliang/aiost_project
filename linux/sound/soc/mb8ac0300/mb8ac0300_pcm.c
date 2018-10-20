/*
 * linux/sound/soc/mb8ac0300/mb8_pcm.c
 *
 * Copyright (C) 2011-2012 SOCIONEXT SEMICONDUCTOR LIMITED
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
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/of.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include "mb8ac0300_pcm.h"

static const struct snd_pcm_hardware mb8_pcm_hardware = {
				/* support function */
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_HALF_DUPLEX,
	.buffer_bytes_max	= 128 * 1024,	/* dma buffer size */
	.period_bytes_min	= 4 * 1024,	/* min value of period bytes */
	.period_bytes_max	= 8 * 1024,	/* max value of period bytes */
	.periods_min		= 2,		/* min value of periods */
	.periods_max		= 128,		/* max value of periods */
	.fifo_size		= 36,		/* not used */
};

static struct dma_async_tx_descriptor *
mb8_pcm_enqueue(struct snd_pcm_substream *substream);

static int mb8_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mb8_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size, &buf->addr,
				       GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	dev_dbg(pcm->card->dev, "dma buffer size %d\n", buf->bytes);

	return 0;
}

static void mb8_pcm_buffdone(void *data)
{
	struct snd_pcm_substream *substream = data;
	struct mb8_pcm_runtime *prtd = substream->runtime->private_data;
	struct dma_chan *dchan = prtd->dchan;

	if (prtd->state & MB8_PCM_ST_RUNNING) {
		if (substream)
			snd_pcm_period_elapsed(substream);

		spin_lock(&prtd->lock);
		/* add dma transfer request */
		prtd->desc[prtd->idx] = mb8_pcm_enqueue(substream);
		dmaengine_submit(prtd->desc[prtd->idx]);
		dma_async_issue_pending(dchan);
		prtd->idx = 1 - prtd->idx;
		spin_unlock(&prtd->lock);
	}
}

static struct dma_async_tx_descriptor *
mb8_pcm_enqueue(struct snd_pcm_substream *substream)
{
	struct mb8_pcm_runtime *prtd = substream->runtime->private_data;
	dma_addr_t pos = prtd->dma_pos;
	unsigned long len = prtd->dma_period;
	struct dma_chan *dchan = prtd->dchan;
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction dir;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dir = DMA_DEV_TO_MEM;
	else
		dir = DMA_MEM_TO_DEV;

	desc = dmaengine_prep_slave_single(dchan, pos, len, dir,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return NULL;

	desc->callback = mb8_pcm_buffdone;
	desc->callback_param = substream;

	pos += prtd->dma_period;
	if (pos >= prtd->dma_end)
		pos = prtd->dma_start;
	prtd->dma_pos = pos;

	return desc;
}

static int mb8_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8_pcm_runtime *prtd = runtime->private_data;
	unsigned long totbytes = params_buffer_bytes(params);
	struct dma_chan *dchan = prtd->dchan;
	struct dma_slave_config slv_cfg;
	unsigned long flags;
	int width;

	/* sample bit width */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		width = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_U16_LE:
		width = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_U32_LE:
		width = 4;
		break;
	default:
		return -EINVAL;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totbytes;

	spin_lock_irqsave(&prtd->lock, flags);
	prtd->dma_period = params_period_bytes(params);
	prtd->dma_start = runtime->dma_addr;
	prtd->dma_pos = prtd->dma_start;
	prtd->dma_end = prtd->dma_start + totbytes;
	spin_unlock_irqrestore(&prtd->lock, flags);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		slv_cfg.direction = DMA_DEV_TO_MEM;
		slv_cfg.src_addr = prtd->dma_addr;
	} else {
		slv_cfg.direction = DMA_MEM_TO_DEV;
		slv_cfg.dst_addr = prtd->dma_addr;
	}

	slv_cfg.src_addr_width = width;
	slv_cfg.dst_addr_width = width;
	slv_cfg.src_maxburst = 1;
	slv_cfg.dst_maxburst = 1;
	slv_cfg.device_fc = false;
	dmaengine_slave_config(dchan, &slv_cfg);

	return 0;
}

static int mb8_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mb8_pcm_runtime *prtd = substream->runtime->private_data;
	struct dma_chan *dchan = prtd->dchan;

	snd_pcm_set_runtime_buffer(substream, NULL);
	dmaengine_terminate_all(dchan);

	return 0;
}

static int mb8_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct mb8_pcm_runtime *prtd = substream->runtime->private_data;

	prtd->dma_pos = prtd->dma_start;
	prtd->desc[0] = mb8_pcm_enqueue(substream);
	prtd->desc[1] = mb8_pcm_enqueue(substream);
	prtd->idx = 0;

	return 0;
}

static int mb8_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mb8_pcm_runtime *prtd = substream->runtime->private_data;
	struct snd_pcm *pcm = substream->pcm;
	struct dma_chan *dchan = prtd->dchan;
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->state |= MB8_PCM_ST_RUNNING;
		spin_unlock_irqrestore(&prtd->lock, flags);
		dmaengine_submit(prtd->desc[0]);
		dmaengine_submit(prtd->desc[1]);
		dma_async_issue_pending(dchan);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->state &= ~MB8_PCM_ST_RUNNING;
		spin_unlock_irqrestore(&prtd->lock, flags);
		dmaengine_terminate_all(dchan);
		break;

	default:
		dev_err(pcm->card->dev, ": invalid command\n");
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t
mb8_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8_pcm_runtime *prtd = runtime->private_data;
	struct dma_async_tx_descriptor *desc = prtd->desc[prtd->idx];
	struct dma_chan *dchan = prtd->dchan;
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int res;

	status = dmaengine_tx_status(dchan, desc->cookie, &state);
	res = prtd->dma_pos - prtd->dma_start +
			prtd->dma_period - state.residue;
	if (res == snd_pcm_lib_buffer_bytes(substream))
		res = 0;
 
	return bytes_to_frames(substream->runtime, res);
}

static int mb8_pcm_open(struct snd_pcm_substream *substream)
{
 	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct platform_device *pdev = to_platform_device(rtd->platform->dev);
	struct snd_pcm *pcm = substream->pcm;
	struct mb8_pcm_runtime *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &mb8_pcm_hardware);
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(pcm->card->dev, ": invalid buffer size: %d\n", ret);
		return ret;
	}

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		prtd->dchan = dma_request_slave_channel(&pdev->dev, "capture");
	else
		prtd->dchan = dma_request_slave_channel(&pdev->dev, "playback");

	spin_lock_init(&prtd->lock);
	runtime->private_data = prtd;

	return 0;
}

static int mb8_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8_pcm_runtime *prtd = runtime->private_data;

	dma_release_channel(prtd->dchan);
	kfree(prtd);

	return 0;
}

static int mb8_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_coherent(substream->pcm->card->dev, vma,
				 runtime->dma_area,
				 runtime->dma_addr,
				 runtime->dma_bytes);
}

static struct snd_pcm_ops mb8_pcm_ops = {
	.open		= mb8_pcm_open,
	.close		= mb8_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mb8_pcm_hw_params,
	.hw_free	= mb8_pcm_hw_free,
	.prepare	= mb8_pcm_prepare,
	.trigger	= mb8_pcm_trigger,
	.pointer	= mb8_pcm_pointer,
	.mmap		= mb8_pcm_mmap,
};

static void mb8_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		/* SNDRV_PCM_STREAM_PLAYBACK:0
		   SNDRV_PCM_STREAM_CAPTURE :1 */
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		/* free dma buffer */
		dma_free_coherent(pcm->card->dev, buf->bytes, buf->area,
				  buf->addr);
		buf->area = NULL;
	}
}

static u64 mb8_pcm_dmamask = DMA_BIT_MASK(32);

static int mb8_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mb8_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		/* allocate dma buffer for playback */
		pr_debug("allocate dma buffer for playback\n");
		ret = mb8_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret) {
			pr_err("failed to alloc dma buf: %d\n", ret);
			goto out;
		}
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		/* allocate dma buffer for capture */
		pr_debug("allocate dma buffer for capture\n");
		ret = mb8_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_CAPTURE);
		if (ret) {
			pr_err(": dma buff alloc failed:%d\n", ret);
			goto out;
		}
	}
out:
	return ret;
}

/* sound soc platform information */
static struct snd_soc_platform_driver mb8ac0300_soc_platform = {
	.ops	= &mb8_pcm_ops,
	.pcm_new	= mb8_pcm_new,
	.pcm_free	= mb8_pcm_free,
};

static int mb8_pcm_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &mb8ac0300_soc_platform);
}

static int  mb8_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id mb8_pcm_dt_ids[] = {
	{ .compatible = "socionext,mb8ac0300_pcm" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8_pcm_dt_ids);

/* PCM driver information of platform device */
static struct platform_driver mb8_pcm_driver = {
	.driver = {
			.name  = "mb8ac0300_pcm",
			.owner = THIS_MODULE,
			.of_match_table = mb8_pcm_dt_ids,
	},
	.probe  = mb8_pcm_probe,
	.remove = mb8_pcm_remove,
};
module_platform_driver(mb8_pcm_driver);

MODULE_AUTHOR("Socionext Semiconductor Limited");
MODULE_DESCRIPTION("Socionext Semiconductor mb8ac0300 PCM DMA module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*mb8ac0300_pcm*");
