/*
 * f_uac1.c -- USB Audio Class 1.0 Function (using u_audio API)
 *
 * Copyright (C) 2016 Ruslan Bilovol <ruslan.bilovol@gmail.com>
 * Copyright (C) 2017 Julian Scheel <julian@jusst.de>
 *
 * This driver doesn't expect any real Audio codec to be present
 * on the device - the audio streams are simply sinked to and
 * sourced from a virtual ALSA sound card created.
 *
 * This file is based on f_uac1.c which is
 *   Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 *   Copyright (C) 2008 Analog Devices, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/usb/audio.h>
#include <linux/module.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <linux/usb/composite.h>
#include <linux/cdev.h>

#include "uac1_mic.h"

struct uac_req {
	struct uac_rtd_params *pp; /* parent param */
	struct usb_request *req;
};

/* Runtime data params for one stream */
struct uac_rtd_params {
	struct snd_uac_chip *uac; /* parent chip */
	bool ep_enabled; /* if the ep is enabled */
	/* Size of the ring buffer */
	size_t dma_bytes;
	unsigned char *dma_area;

	struct snd_pcm_substream *ss;

	/* Ring buffer */
	ssize_t hw_ptr;

	void *rbuf;

	size_t period_size;

	unsigned max_psize;	/* MaxPacketSize of endpoint */
	struct uac_req *ureq;

	spinlock_t lock;
};

struct snd_uac_chip {
	struct g_audio *audio_dev;

	struct uac_rtd_params p_prm;

	struct snd_card *card;
	struct snd_pcm *pcm;

	/* timekeeping for the playback endpoint */
	unsigned int p_interval;
	unsigned int p_residue;

	/* pre-calculated values for playback iso completion */
	unsigned int p_pktsize;
	unsigned int p_pktsize_residue;
	unsigned int p_framesize;
};

#define BUFF_SIZE_MAX	(PAGE_SIZE * 16)
#define PRD_SIZE_MAX	PAGE_SIZE
#define MIN_PERIODS	4

static struct snd_pcm_hardware uac_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		 | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		 | SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_max = BUFF_SIZE_MAX / PRD_SIZE_MAX,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_max = PRD_SIZE_MAX,
	.periods_min = MIN_PERIODS,
};

#define UAC_MAX_RATES 10
struct uac_params {
	/* playback */
	int p_chmask;	/* channel mask */
	int p_srate[UAC_MAX_RATES];	/* rate in Hz */
	int p_srate_active;		/* selected rate in Hz */
	int p_ssize;	/* sample size */

	int req_number; /* number of preallocated requests */
};

enum uac_state
{
	UAC_STATE_DISCONNECTED,
	UAC_STATE_CONNECTED,
	UAC_STATE_STREAMING,
};

struct uac_events;

struct g_audio {
	struct usb_function func;
	struct usb_gadget *gadget;

	struct usb_ep *in_ep;

	/* Max packet size for all in_ep possible speeds */
	unsigned int in_ep_maxpsize;

	/* The ALSA Sound Card it represents on the USB-Client side */
	struct snd_uac_chip *uac;

	struct uac_params params;

	enum uac_state		state;

	struct class		*class;
	int			major;
	int			minor;
	struct cdev		cdev;

	struct uac_events	*events;
	spinlock_t		lock;
};

static inline struct g_audio *func_to_g_audio(struct usb_function *f)
{
	return container_of(f, struct g_audio, func);
}

static inline uint num_channels(uint chanmask)
{
	uint num = 0;

	while (chanmask) {
		num += (chanmask & 1);
		chanmask >>= 1;
	}

	return num;
}

/* --------------------------------------------------------------------------
 * Events
 */

struct uac_kevent {
	struct list_head	list;
	struct uac_event	event;
};

struct uac_events {
	wait_queue_head_t	wait;
	struct list_head	free;
	struct list_head	available;
	unsigned int		navailable;
	unsigned int		nallocated;
	u32			sequence;
};

static int uac_event_init(struct g_audio *audio_dev)
{
	audio_dev->events = kzalloc(sizeof(*audio_dev->events), GFP_KERNEL);
	if (audio_dev->events == NULL)
		return -ENOMEM;

	init_waitqueue_head(&audio_dev->events->wait);

	INIT_LIST_HEAD(&audio_dev->events->free);
	INIT_LIST_HEAD(&audio_dev->events->available);

	audio_dev->events->sequence = -1;

	return 0;
}

static int uac_event_alloc(struct g_audio *audio_dev, unsigned int n)
{
	struct uac_events *events = audio_dev->events;
	unsigned long flags;

	if (!events) {
		WARN_ON(1);
		return -ENOMEM;
	}

	while (events->nallocated < n) {
		struct uac_kevent *kev;

		kev = kzalloc(sizeof(*kev), GFP_KERNEL);
		if (kev == NULL)
			return -ENOMEM;

		spin_lock_irqsave(&audio_dev->lock, flags);
		list_add_tail(&kev->list, &events->free);
		events->nallocated++;
		spin_unlock_irqrestore(&audio_dev->lock, flags);
	}

	return 0;
}

#define list_kfree(list, type, member)				\
	while (!list_empty(list)) {				\
		type *hi;					\
		hi = list_first_entry(list, type, member);	\
		list_del(&hi->member);				\
		kfree(hi);					\
	}

static void uac_event_free(struct g_audio *audio_dev)
{
	struct uac_events *events = audio_dev->events;

	if (!events)
		return;

	list_kfree(&events->free, struct uac_kevent, list);
	list_kfree(&events->available, struct uac_kevent, list);

	kfree(events);
	audio_dev->events = NULL;
}

static int __uac_event_dequeue(struct g_audio *audio_dev, struct uac_event *event)
{
	struct uac_events *events = audio_dev->events;
	struct uac_kevent *kev;
	unsigned long flags;

	spin_lock_irqsave(&audio_dev->lock, flags);

	if (list_empty(&events->available)) {
		spin_unlock_irqrestore(&audio_dev->lock, flags);
		return -ENOENT;
	}

	WARN_ON(events->navailable == 0);

	kev = list_first_entry(&events->available, struct uac_kevent, list);
	list_move(&kev->list, &events->free);
	events->navailable--;

	kev->event.pending = events->navailable;
	*event = kev->event;

	spin_unlock_irqrestore(&audio_dev->lock, flags);

	return 0;
}

static int uac_event_dequeue(struct g_audio *audio_dev, struct uac_event *event,
	int nonblocking)
{
	struct uac_events *events = audio_dev->events;
	int ret;

	if (nonblocking)
		return __uac_event_dequeue(audio_dev, event);

	do {
		ret = wait_event_interruptible(events->wait,
					       events->navailable != 0);
		if (ret < 0)
			break;

		ret = __uac_event_dequeue(audio_dev, event);
	} while (ret == -ENOENT);

	return ret;
}

static void uac_event_queue(struct g_audio *audio_dev, const struct uac_event *event)
{
	struct uac_events *events = audio_dev->events;
	struct uac_kevent *kev;
	unsigned long flags;
	struct timespec timestamp;

	ktime_get_ts(&timestamp);

	spin_lock_irqsave(&audio_dev->lock, flags);

	if (events == NULL) {
		spin_unlock_irqrestore(&audio_dev->lock, flags);
		return;
	}

	events->sequence++;

	if (list_empty(&events->free)) {
		spin_unlock_irqrestore(&audio_dev->lock, flags);
		return;
	}

	kev = list_first_entry(&events->free, struct uac_kevent, list);
	kev->event.type = event->type;
	kev->event.u = event->u;
	kev->event.timestamp = timestamp;
	kev->event.sequence = events->sequence;
	list_move_tail(&kev->list, &events->available);

	events->navailable++;

	wake_up_all(&events->wait);

	spin_unlock_irqrestore(&audio_dev->lock, flags);
}

static int uac_event_pending(struct g_audio *audio_dev)
{
	return audio_dev->events->navailable;
}

/* --------------------------------------------------------------------------
 * Connection / disconnection
 */

static void uac_connect(struct g_audio *audio_dev)
{
	struct usb_composite_dev *cdev = audio_dev->func.config->cdev;
	int ret;

	if ((ret = usb_function_activate(&audio_dev->func)) < 0)
		INFO(cdev, "UAC connect failed with %d\n", ret);
}

static void uac_disconnect(struct g_audio *audio_dev)
{
	struct usb_composite_dev *cdev = audio_dev->func.config->cdev;
	int ret;

	if ((ret = usb_function_deactivate(&audio_dev->func)) < 0)
		INFO(cdev, "UAC disconnect failed with %d\n", ret);
}

/* --------------------------------------------------------------------------
 * Character Device
 */

static unsigned int uac_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct g_audio *audio = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &audio->events->wait, wait);
	if (uac_event_pending(audio))
		mask |= POLLPRI;

	return mask;
}

static long uac_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct g_audio *audio = filp->private_data;
	struct uac_params *params = &audio->params;
	struct uac_event event;
	int ret = 0;

	switch (cmd) {
	case UACIOC_DQEVENT:
		ret = uac_event_dequeue(audio, &event, filp->f_flags & O_NONBLOCK);
		if (ret < 0)
			break;
		if (copy_to_user((void *)arg, &event, sizeof(event)))
			ret = -EFAULT;
		break;

	case UACIOC_SRATE:
		if (copy_to_user((void *)arg, &params->p_srate_active, sizeof(params->p_srate_active)))
			ret = -EFAULT;
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int uac_open(struct inode *inode, struct file *filp)
{
	struct g_audio *audio = container_of(inode->i_cdev, struct g_audio, cdev);
	int ret;

	ret = uac_event_init(audio);
	if (ret < 0)
		return ret;

	ret = uac_event_alloc(audio, 8);
	if (ret < 0) {
		uac_event_free(audio);
		return ret;
	}

	filp->private_data = audio;

	uac_connect(audio);
	return 0;
}

static int uac_release(struct inode *inode, struct file *filp)
{
	struct g_audio *audio = filp->private_data;

	uac_disconnect(audio);

	filp->private_data = NULL;

	uac_event_free(audio);

	return 0;
}

static struct file_operations uac_fops = {
	.owner		= THIS_MODULE,
	.poll		= uac_poll,
	.unlocked_ioctl	= uac_ioctl,
	.open		= uac_open,
	.release	= uac_release,
};

/* --------------------------------------------------------------------------
 * 
 */

static int uac_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm = &uac->p_prm;
	struct g_audio *audio_dev = uac->audio_dev;
	struct uac_params *params = &audio_dev->params;
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&prm->lock, flags);

	/* Reset */
	prm->hw_ptr = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		prm->ss = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		prm->ss = NULL;
		break;
	default:
		err = -EINVAL;
	}

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Clear buffer after Play stops */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && !prm->ss)
		memset(prm->rbuf, 0, prm->max_psize * params->req_number);

	return err;
}

static snd_pcm_uframes_t uac_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm = &uac->p_prm;

	return bytes_to_frames(substream->runtime, prm->hw_ptr);
}

static int uac_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm = &uac->p_prm;
	int err;

	err = snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
	if (err >= 0) {
		prm->dma_bytes = substream->runtime->dma_bytes;
		prm->dma_area = substream->runtime->dma_area;
		prm->period_size = params_period_bytes(hw_params);
	}

	return err;
}

static int uac_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm = &uac->p_prm;

	prm->dma_area = NULL;
	prm->dma_bytes = 0;
	prm->period_size = 0;

	return snd_pcm_lib_free_pages(substream);
}

static int uac_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct g_audio *audio_dev = uac->audio_dev;
	struct uac_params *params = &audio_dev->params;
	int p_ssize = params->p_ssize;
	int p_srate = params->p_srate_active;
	int p_chmask = params->p_chmask;

	uac->p_residue = 0;

	runtime->hw = uac_pcm_hardware;

	spin_lock_init(&uac->p_prm.lock);
	runtime->hw.rate_min = p_srate;
	switch (p_ssize) {
	case 3:
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S24_3LE;
		break;
	case 4:
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	}
	runtime->hw.channels_min = num_channels(p_chmask);
	runtime->hw.period_bytes_min = 2 * uac->p_prm.max_psize
					/ runtime->hw.periods_min;

	runtime->hw.rate_max = runtime->hw.rate_min;
	runtime->hw.channels_max = runtime->hw.channels_min;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	return 0;
}

/* ALSA cries without these function pointers */
static int uac_pcm_null(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_ops uac_pcm_ops = {
	.open = uac_pcm_open,
	.close = uac_pcm_null,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = uac_pcm_hw_params,
	.hw_free = uac_pcm_hw_free,
	.trigger = uac_pcm_trigger,
	.pointer = uac_pcm_pointer,
	.prepare = uac_pcm_null,
};

/* --------------------------------------------------------------------------
 * 
 */

static inline void free_ep(struct uac_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev;
	struct uac_params *params;
	int i;

	if (!prm->ep_enabled)
		return;

	prm->ep_enabled = false;

	audio_dev = uac->audio_dev;
	params = &audio_dev->params;

	for (i = 0; i < params->req_number; i++) {
		if (prm->ureq[i].req) {
			usb_ep_dequeue(ep, prm->ureq[i].req);
			usb_ep_free_request(ep, prm->ureq[i].req);
			prm->ureq[i].req = NULL;
		}
	}

	if (usb_ep_disable(ep))
		dev_err(uac->card->dev, "%s:%d Error!\n", __func__, __LINE__);
}

static void u_audio_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned pending;
	unsigned long flags;
	unsigned int hw_ptr;
	bool update_alsa = false;
	int status = req->status;
	struct uac_req *ur = req->context;
	struct snd_pcm_substream *substream;
	struct uac_rtd_params *prm = ur->pp;
	struct snd_uac_chip *uac = prm->uac;

	/* i/f shutting down */
	if (!prm->ep_enabled || req->status == -ESHUTDOWN)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);

	substream = prm->ss;

	/* Do nothing if ALSA isn't active */
	if (!substream)
		goto exit;

	spin_lock_irqsave(&prm->lock, flags);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * For each IN packet, take the quotient of the current data
		 * rate and the endpoint's interval as the base packet size.
		 * If there is a residue from this division, add it to the
		 * residue accumulator.
		 */
		req->length = uac->p_pktsize;
		uac->p_residue += uac->p_pktsize_residue;

		/*
		 * Whenever there are more bytes in the accumulator than we
		 * need to add one more sample frame, increase this packet's
		 * size and decrease the accumulator.
		 */
		if (uac->p_residue / uac->p_interval >= uac->p_framesize) {
			req->length += uac->p_framesize;
			uac->p_residue -= uac->p_framesize *
					   uac->p_interval;
		}

		req->actual = req->length;
	}

	pending = prm->hw_ptr % prm->period_size;
	pending += req->actual;
	if (pending >= prm->period_size)
		update_alsa = true;

	hw_ptr = prm->hw_ptr;
	prm->hw_ptr = (prm->hw_ptr + req->actual) % prm->dma_bytes;

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Pack USB load in ALSA ring buffer */
	pending = prm->dma_bytes - hw_ptr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (unlikely(pending < req->actual)) {
			memcpy(req->buf, prm->dma_area + hw_ptr, pending);
			memcpy(req->buf + pending, prm->dma_area,
			       req->actual - pending);
		} else {
			memcpy(req->buf, prm->dma_area + hw_ptr, req->actual);
		}
	} else {
		if (unlikely(pending < req->actual)) {
			memcpy(prm->dma_area + hw_ptr, req->buf, pending);
			memcpy(prm->dma_area, req->buf + pending,
			       req->actual - pending);
		} else {
			memcpy(prm->dma_area + hw_ptr, req->buf, req->actual);
		}
	}

exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(uac->card->dev, "%d Error!\n", __LINE__);

	if (update_alsa)
		snd_pcm_period_elapsed(substream);
}

int u_audio_set_playback_srate(struct g_audio *audio_dev, int srate)
{
	struct uac_params *params = &audio_dev->params;
	struct uac_event uac_event;
	int i;

	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (params->p_srate[i] == srate) {
			params->p_srate_active = srate;
			memset(&uac_event, 0x00, sizeof(uac_event));
			uac_event.type = UAC_EVENT_SRATE;
			uac_event.u.srate = srate;
			uac_event_queue(audio_dev, &uac_event);
			return 0;
		}
		if (params->p_srate[i] == 0)
			break;
	}

	return -EINVAL;
}

int u_audio_start_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct usb_gadget *gadget = audio_dev->gadget;
	struct device *dev = &gadget->dev;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac_rtd_params *prm;
	struct uac_params *params =  &audio_dev->params;
	unsigned int factor, rate;
	const struct usb_endpoint_descriptor *ep_desc;
	int req_len, i;

	dev_dbg(dev, "start playback with rate %d\n", params->p_srate_active);
	ep = audio_dev->in_ep;
	prm = &uac->p_prm;
	config_ep_by_speed(gadget, &audio_dev->func, ep);

	ep_desc = ep->desc;

	/* pre-calculate the playback endpoint's interval */
	if (gadget->speed == USB_SPEED_FULL)
		factor = 1000;
	else
		factor = 8000;

	/* pre-compute some values for iso_complete() */
	uac->p_framesize = params->p_ssize *
			    num_channels(params->p_chmask);
	rate = params->p_srate_active * uac->p_framesize;
	uac->p_interval = factor / (1 << (ep_desc->bInterval - 1));
	uac->p_pktsize = min_t(unsigned int, rate / uac->p_interval,
				prm->max_psize);

	if (uac->p_pktsize < prm->max_psize)
		uac->p_pktsize_residue = rate % uac->p_interval;
	else
		uac->p_pktsize_residue = 0;

	req_len = uac->p_pktsize;
	uac->p_residue = 0;

	prm->ep_enabled = true;
	usb_ep_enable(ep);

	for (i = 0; i < params->req_number; i++) {
		if (!prm->ureq[i].req) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->ureq[i].req = req;
			prm->ureq[i].pp = prm;

			req->zero = 0;
			req->context = &prm->ureq[i];
			req->length = req_len;
			req->complete = u_audio_iso_complete;
			req->buf = prm->rbuf + i * prm->max_psize;
		}

		if (usb_ep_queue(ep, prm->ureq[i].req, GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	return 0;
}

void u_audio_stop_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	free_ep(&uac->p_prm, audio_dev->in_ep);
}

/* --------------------------------------------------------------------------
 * 
 */

int g_audio_setup(struct g_audio *g_audio, const char *pcm_name,
					const char *card_name)
{
	struct snd_uac_chip *uac;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct uac_params *params;
	int p_chmask;
	int err;

	if (!g_audio)
		return -EINVAL;

	uac = kzalloc(sizeof(*uac), GFP_KERNEL);
	if (!uac)
		return -ENOMEM;
	g_audio->uac = uac;
	uac->audio_dev = g_audio;

	params = &g_audio->params;
	p_chmask = params->p_chmask;

	if (p_chmask) {
		struct uac_rtd_params *prm = &uac->p_prm;

		uac->p_prm.uac = uac;
		prm->max_psize = g_audio->in_ep_maxpsize;

		prm->ureq = kcalloc(params->req_number, sizeof(struct uac_req),
				GFP_KERNEL);
		if (!prm->ureq) {
			err = -ENOMEM;
			goto fail;
		}

		prm->rbuf = kcalloc(params->req_number, prm->max_psize,
				GFP_KERNEL);
		if (!prm->rbuf) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}
	}

	/* Choose any slot, with no id */
	err = snd_card_new(&g_audio->gadget->dev,
			-1, NULL, THIS_MODULE, 0, &card);
	if (err < 0)
		goto fail;

	uac->card = card;

	/*
	 * Create first PCM device
	 * Create a substream only for non-zero channel streams
	 */
	err = snd_pcm_new(uac->card, pcm_name, 0,
			       p_chmask ? 1 : 0, 0, &pcm);
	if (err < 0)
		goto snd_fail;

	strcpy(pcm->name, pcm_name);
	pcm->private_data = uac;
	uac->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &uac_pcm_ops);

	strcpy(card->driver, card_name);
	strcpy(card->shortname, card_name);
	sprintf(card->longname, "%s %i", card_name, card->dev->id);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
		snd_dma_continuous_data(GFP_KERNEL), 0, BUFF_SIZE_MAX);

	err = snd_card_register(card);

	if (!err)
		return 0;

snd_fail:
	snd_card_free(card);
fail:
	kfree(uac->p_prm.ureq);
	kfree(uac->p_prm.rbuf);
	kfree(uac);

	return err;
}

void g_audio_cleanup(struct g_audio *g_audio)
{
	struct snd_uac_chip *uac;
	struct snd_card *card;

	if (!g_audio || !g_audio->uac)
		return;

	uac = g_audio->uac;
	card = uac->card;
	if (card)
		snd_card_free(card);

	kfree(uac->p_prm.ureq);
	kfree(uac->p_prm.rbuf);
	kfree(uac);
}

/* --------------------------------------------------------------------------
 * 
 */

#define UAC_DEF_CCHMASK		0x3
#define UAC_DEF_CSRATE		48000
#define UAC_DEF_CSSIZE		2
#define UAC_DEF_PCHMASK		0x3
#define UAC_DEF_PSRATE		48000
#define UAC_DEF_PSSIZE		2
#define UAC_DEF_REQ_NUM		2

struct f_uac_opts {
	struct usb_function_instance	func_inst;
	int				p_chmask;
	int				p_srate[UAC_MAX_RATES];
	int				p_srate_active;
	int				p_ssize;
	int				req_number;
	unsigned			bound:1;

	struct mutex			lock;
	int				refcnt;
};

struct f_uac {
	struct g_audio g_audio;
	u8 ac_intf, as_in_intf;
	u8 ac_alt, as_in_alt;	/* needed for get_alt() */
	int ctl_id;
};

static inline struct f_uac *func_to_uac(struct usb_function *f)
{
	return container_of(f, struct f_uac, g_audio.func);
}

static inline
struct f_uac_opts *g_audio_to_uac_opts(struct g_audio *agdev)
{
	return container_of(agdev->func.fi, struct f_uac_opts, func_inst);
}

static inline struct f_uac_opts *to_f_uac_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_uac_opts,
			    func_inst.group);
}

void f_uac_attr_release(struct config_item *item)
{
	struct f_uac_opts *opts = to_f_uac_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

/*
 * DESCRIPTORS ... most are static, but strings and full
 * configuration descriptors are built on demand.
 */

#define UAC1_IN_EP_MAX_PACKET_SIZE 200

/*
 * We have three interfaces - one AudioControl and two AudioStreaming
 *
 * The driver implements a simple UAC_1 topology.
 * ALSA_Playback -> IT_1 -> OT_2 -> USB-IN
 */
#define F_AUDIO_AC_INTERFACE		0
#define F_AUDIO_AS_IN_INTERFACE		1
/* Number of streaming interfaces */
#define F_AUDIO_NUM_INTERFACES		1

static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof(iad_desc),
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface =	0,
	.bInterfaceCount =	1 + F_AUDIO_NUM_INTERFACES,
	.bFunctionClass =	USB_CLASS_AUDIO,
	.bFunctionSubClass =	0,
};

/* B.3.1  Standard AC Interface Descriptor */
static struct usb_interface_descriptor ac_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOCONTROL,
};

/*
 * The number of AudioStreaming and MIDIStreaming interfaces
 * in the Audio Interface Collection
 */
DECLARE_UAC_AC_HEADER_DESCRIPTOR(1);

#define UAC_DT_AC_HEADER_LENGTH	UAC_DT_AC_HEADER_SIZE(F_AUDIO_NUM_INTERFACES)
/* 1 input terminal and 1 output terminal */
#define UAC_DT_TOTAL_LENGTH (UAC_DT_AC_HEADER_LENGTH \
	+ UAC_DT_INPUT_TERMINAL_SIZE + UAC_DT_OUTPUT_TERMINAL_SIZE)
/* B.3.2  Class-Specific AC Interface Descriptor */
static struct uac1_ac_header_descriptor_1 ac_header_desc = {
	.bLength =		UAC_DT_AC_HEADER_LENGTH,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_HEADER,
	.bcdADC =		cpu_to_le16(0x0100),
	.wTotalLength =		cpu_to_le16(UAC_DT_TOTAL_LENGTH),
	.bInCollection =	F_AUDIO_NUM_INTERFACES,
	.baInterfaceNr = {
	/* Interface number of the AudioStream interfaces */
		[0] =		1,
	}
};

#define IO_IN_IT_ID	1
static struct uac_input_terminal_descriptor io_in_it_desc = {
	.bLength		= UAC_DT_INPUT_TERMINAL_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype	= UAC_INPUT_TERMINAL,
	.bTerminalID		= IO_IN_IT_ID,
	.wTerminalType		= cpu_to_le16(UAC_INPUT_TERMINAL_MICROPHONE),
	.bAssocTerminal		= 0,
	.wChannelConfig		= cpu_to_le16(0x3),
};

#define USB_IN_OT_ID	2
static struct uac1_output_terminal_descriptor usb_in_ot_desc = {
	.bLength =		UAC_DT_OUTPUT_TERMINAL_SIZE,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_OUTPUT_TERMINAL,
	.bTerminalID =		USB_IN_OT_ID,
	.wTerminalType =	cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal =	0,
	.bSourceID =		IO_IN_IT_ID,
};

/* B.4.1  Standard AS Interface Descriptor */
static struct usb_interface_descriptor as_in_interface_alt_0_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOSTREAMING,
};

static struct usb_interface_descriptor as_in_interface_alt_1_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	1,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOSTREAMING,
};

/* B.4.2  Class-Specific AS Interface Descriptor */
static struct uac1_as_header_descriptor as_in_header_desc = {
	.bLength =		UAC_DT_AS_HEADER_SIZE,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_AS_GENERAL,
	.bTerminalLink =	USB_IN_OT_ID,
	.bDelay =		1,
	.wFormatTag =		cpu_to_le16(UAC_FORMAT_TYPE_I_PCM),
};

DECLARE_UAC_FORMAT_TYPE_I_DISCRETE_DESC(UAC_MAX_RATES);
#define uac_format_type_i_discrete_descriptor \
	uac_format_type_i_discrete_descriptor_##UAC_MAX_RATES

static struct uac_format_type_i_discrete_descriptor as_in_type_i_desc = {
	.bLength =		0, /* filled on rate setup */
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_FORMAT_TYPE,
	.bFormatType =		UAC_FORMAT_TYPE_I,
	.bSubframeSize =	2,
	.bBitResolution =	16,
	.bSamFreqType =		0, /* filled on rate setup */
};

/* Standard ISO IN Endpoint Descriptor */
static struct usb_endpoint_descriptor as_in_ep_desc  = {
	.bLength =		USB_DT_ENDPOINT_AUDIO_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	.wMaxPacketSize	=	cpu_to_le16(UAC1_IN_EP_MAX_PACKET_SIZE),
	.bInterval =		4,
};

/* Class-specific AS ISO IN Endpoint Descriptor */
static struct uac_iso_endpoint_descriptor as_iso_in_desc = {
	.bLength =		UAC_ISO_ENDPOINT_DESC_SIZE,
	.bDescriptorType =	USB_DT_CS_ENDPOINT,
	.bDescriptorSubtype =	UAC_EP_GENERAL,
	.bmAttributes =		1,
	.bLockDelayUnits =	0,
	.wLockDelay =		0,
};

static struct usb_descriptor_header *f_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,

	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc,

	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,

	(struct usb_descriptor_header *)&as_in_interface_alt_0_desc,
	(struct usb_descriptor_header *)&as_in_interface_alt_1_desc,
	(struct usb_descriptor_header *)&as_in_header_desc,

	(struct usb_descriptor_header *)&as_in_type_i_desc,

	(struct usb_descriptor_header *)&as_in_ep_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

static struct usb_ss_ep_comp_descriptor as_in_ep_comp_desc = {
	.bLength =		USB_DT_SS_EP_COMP_SIZE,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst =		0,
	.bmAttributes =		0,
	.wBytesPerInterval =	cpu_to_le16(UAC1_IN_EP_MAX_PACKET_SIZE),
};

static struct usb_descriptor_header *ss_f_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,

	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc,

	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,

	(struct usb_descriptor_header *)&as_in_interface_alt_0_desc,
	(struct usb_descriptor_header *)&as_in_interface_alt_1_desc,
	(struct usb_descriptor_header *)&as_in_header_desc,

	(struct usb_descriptor_header *)&as_in_type_i_desc,

	(struct usb_descriptor_header *)&as_in_ep_desc,
	(struct usb_descriptor_header *)&as_in_ep_comp_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

enum {
	STR_ASSOC,
};

static struct usb_string strings_uac1[] = {
	[STR_ASSOC].s = "UAC Microphone",
	{ },
};

static struct usb_gadget_strings str_uac1 = {
	.language = 0x0409,	/* en-us */
	.strings = strings_uac1,
};

static struct usb_gadget_strings *uac1_strings[] = {
	&str_uac1,
	NULL,
};

/*
 * This function is an ALSA sound card following USB Audio Class Spec 1.0.
 */

static void uac_cs_attr_sample_rate(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_function *fn = ep->driver_data;
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct g_audio *agdev = func_to_g_audio(fn);
	struct f_uac *uac1 = func_to_uac(fn);
	struct f_uac_opts *opts = g_audio_to_uac_opts(agdev);
	u8 *buf = (u8 *)req->buf;
	u32 val = 0;

	if (req->actual != 3) {
		WARN(cdev, "Invalid data size for UAC_EP_CS_ATTR_SAMPLE_RATE.\n");
		return;
	}

	val = buf[0] | (buf[1] << 8) | (buf[2] << 16);

	if (uac1->ctl_id == (USB_DIR_IN | 1)) {
		opts->p_srate_active = val;
		u_audio_set_playback_srate(agdev, opts->p_srate_active);
	}
}

static int audio_set_endpoint_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = f->config->cdev->req;
	struct f_uac		*uac1 = func_to_uac(f);
	int			value = -EOPNOTSUPP;
	u8			ep = le16_to_cpu(ctrl->wIndex) & 0xff;
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u8			cs = w_value >> 8;

	DBG(cdev, "bRequest 0x%x, w_value 0x%04x, len %d, endpoint %d\n",
			ctrl->bRequest, w_value, len, ep);

	switch (ctrl->bRequest) {
	case UAC_SET_CUR: {
		if (cs == UAC_EP_CS_ATTR_SAMPLE_RATE) {
			cdev->gadget->ep0->driver_data = f;
			uac1->ctl_id = ep;
			req->complete = uac_cs_attr_sample_rate;
		}
		value = len;
		break;
		}
	case UAC_SET_MIN:
		break;

	case UAC_SET_MAX:
		break;

	case UAC_SET_RES:
		break;

	case UAC_SET_MEM:
		break;

	default:
		break;
	}

	return value;
}

static int audio_get_endpoint_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request *req = f->config->cdev->req;
	struct g_audio *agdev = func_to_g_audio(f);
	struct f_uac_opts *opts = g_audio_to_uac_opts(agdev);
	u8 *buf = (u8 *)req->buf;
	int value = -EOPNOTSUPP;
	u8 ep = le16_to_cpu(ctrl->wIndex) & 0xff;
	u16 len = le16_to_cpu(ctrl->wLength);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u8 cs = w_value >> 8;
	u32 val = 0;

	DBG(cdev, "bRequest 0x%x, w_value 0x%04x, len %d, endpoint %d\n",
			ctrl->bRequest, w_value, len, ep);

	switch (ctrl->bRequest) {
	case UAC_GET_CUR: {
		if (cs == UAC_EP_CS_ATTR_SAMPLE_RATE) {
			if (ep == (USB_DIR_IN | 1))
				val = opts->p_srate_active;
			buf[2] = (val >> 16) & 0xff;
			buf[1] = (val >> 8) & 0xff;
			buf[0] = val & 0xff;
		}
		value = len;
		break;
		}
	case UAC_GET_MIN:
	case UAC_GET_MAX:
	case UAC_GET_RES:
		value = len;
		break;
	case UAC_GET_MEM:
		break;
	default:
		break;
	}

	return value;
}

static int
f_audio_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/* composite driver infrastructure handles everything; interface
	 * activation uses set_alt().
	 */
	switch (ctrl->bRequestType) {
	case USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_ENDPOINT:
		value = audio_set_endpoint_req(f, ctrl);
		break;

	case USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT:
		value = audio_get_endpoint_req(f, ctrl);
		break;

	default:
		ERROR(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "audio req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "audio response on err %d\n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int f_audio_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &gadget->dev;
	struct f_uac *uac1 = func_to_uac(f);
	struct uac_event uac_event;
	int ret = 0;

	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (intf == uac1->ac_intf) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}
		if (uac1->g_audio.state == UAC_STATE_DISCONNECTED) {
			memset(&uac_event, 0x00, sizeof(uac_event));
			uac_event.type = UAC_EVENT_CONNECT;
			uac_event_queue(&uac1->g_audio, &uac_event);
			uac1->g_audio.state = UAC_STATE_CONNECTED;
		}
		return 0;
	}

	if (intf == uac1->as_in_intf) {
		uac1->as_in_alt = alt;

		switch (alt) {
		case 0:
			if (uac1->g_audio.state != UAC_STATE_STREAMING)
				return 0;
			u_audio_stop_playback(&uac1->g_audio);
			memset(&uac_event, 0x00, sizeof(uac_event));
			uac_event.type = UAC_EVENT_STREAMOFF;
			uac_event_queue(&uac1->g_audio, &uac_event);
			uac1->g_audio.state = UAC_STATE_CONNECTED;
			break;

		case 1:
			if (uac1->g_audio.state != UAC_STATE_CONNECTED)
				return 0;
			u_audio_start_playback(&uac1->g_audio);
			memset(&uac_event, 0x00, sizeof(uac_event));
			uac_event.type = UAC_EVENT_STREAMON;
			uac_event_queue(&uac1->g_audio, &uac_event);
			uac1->g_audio.state = UAC_STATE_STREAMING;
			break;

		default:
			return -EINVAL;
		}
	} else {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	return ret;
}

static int f_audio_get_alt(struct usb_function *f, unsigned intf)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &gadget->dev;
	struct f_uac *uac1 = func_to_uac(f);

	if (intf == uac1->ac_intf)
		return uac1->ac_alt;
	else if (intf == uac1->as_in_intf)
		return uac1->as_in_alt;
	else
		dev_err(dev, "%s:%d Invalid Interface %d!\n",
			__func__, __LINE__, intf);

	return -EINVAL;
}


static void f_audio_disable(struct usb_function *f)
{
	struct f_uac *uac1 = func_to_uac(f);
	struct uac_event uac_event;

	uac1->as_in_alt = 0;

	if (uac1->g_audio.state == UAC_STATE_STREAMING) {
		u_audio_stop_playback(&uac1->g_audio);
	}
	memset(&uac_event, 0x00, sizeof(uac_event));
	uac_event.type = UAC_EVENT_DISCONNECT;
	uac_event_queue(&uac1->g_audio, &uac_event);
	uac1->g_audio.state = UAC_STATE_DISCONNECTED;
}

/*-------------------------------------------------------------------------*/

/* audio function driver setup/binding */
static int f_audio_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev	*cdev = c->cdev;
	struct usb_gadget		*gadget = cdev->gadget;
	struct f_uac			*uac1 = func_to_uac(f);
	struct g_audio			*audio = func_to_g_audio(f);
	struct f_uac_opts		*audio_opts;
	struct usb_ep			*ep = NULL;
	struct usb_string		*us;
	int				status;
	int				idx, i;
	dev_t				dev;
	struct device			*device;

	audio_opts = container_of(f->fi, struct f_uac_opts, func_inst);

	us = usb_gstrings_attach(cdev, uac1_strings, ARRAY_SIZE(strings_uac1));
	if (IS_ERR(us))
		return PTR_ERR(us);
	iad_desc.iFunction = us[STR_ASSOC].id;

	/* Set channel numbers */
	io_in_it_desc.bNrChannels = num_channels(audio_opts->p_chmask);
	io_in_it_desc.wChannelConfig = cpu_to_le16(audio_opts->p_chmask);
	as_in_type_i_desc.bNrChannels = num_channels(audio_opts->p_chmask);
	as_in_type_i_desc.bSubframeSize = audio_opts->p_ssize;
	as_in_type_i_desc.bBitResolution = audio_opts->p_ssize * 8;

	/* Set sample rates */
	for (i = 0, idx = 0; i < UAC_MAX_RATES; i++) {
		if (audio_opts->p_srate[i] == 0)
			break;
		memcpy(as_in_type_i_desc.tSamFreq[idx++],
				&audio_opts->p_srate[i], 3);
	}
	as_in_type_i_desc.bLength = UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(idx);
	as_in_type_i_desc.bSamFreqType = idx;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	ac_interface_desc.bInterfaceNumber = status;
	uac1->ac_intf = status;
	uac1->ac_alt = 0;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	as_in_interface_alt_0_desc.bInterfaceNumber = status;
	as_in_interface_alt_1_desc.bInterfaceNumber = status;
	uac1->as_in_intf = status;
	uac1->as_in_alt = 0;

	audio->gadget = gadget;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &as_in_ep_desc);
	if (!ep)
		goto fail;
	audio->in_ep = ep;
	audio->in_ep->desc = &as_in_ep_desc;

	/* copy descriptors, and track endpoint copies */
	status = usb_assign_descriptors(f, f_audio_desc, f_audio_desc, ss_f_audio_desc);
	if (status)
		goto fail;

	audio->state = UAC_STATE_DISCONNECTED;

	audio->class = class_create(THIS_MODULE, "uac_gadget");
	if (IS_ERR(audio->class)) {
		status = PTR_ERR(audio->class);
		audio->class = NULL;
		goto err_card_register;
	}

	status = alloc_chrdev_region(&dev, 0, 1, "uac_gadget");
	if (status < 0) {
		class_destroy(audio->class);
		goto err_card_register;
	}
	audio->major = MAJOR(dev);

	spin_lock_init(&audio->lock);

	cdev_init(&audio->cdev, &uac_fops);
	status = cdev_add(&audio->cdev, dev, 1);
	if (status < 0) {
		unregister_chrdev_region(dev, 1);
		class_destroy(audio->class);
		goto err_card_register;
	}

	device = device_create(audio->class, NULL, dev, NULL, "uac_gadget");
	if (IS_ERR(device)) {
		status = PTR_ERR(device);
		cdev_del(&audio->cdev);
		unregister_chrdev_region(dev, 1);
		class_destroy(audio->class);
		goto err_card_register;
	}

	audio->in_ep_maxpsize = le16_to_cpu(as_in_ep_desc.wMaxPacketSize);
	audio->params.p_chmask = audio_opts->p_chmask;
	memcpy(audio->params.p_srate, audio_opts->p_srate,
			sizeof(audio->params.p_srate));
	audio->params.p_srate_active = audio_opts->p_srate_active;
	audio->params.p_ssize = audio_opts->p_ssize;
	audio->params.req_number = audio_opts->req_number;

	status = g_audio_setup(audio, "UAC1_PCM", "UAC1_Gadget");
	if (status)
		goto err_card_register;

	return 0;

err_card_register:
	usb_free_all_descriptors(f);
fail:
	return status;
}

/*-------------------------------------------------------------------------*/

static struct configfs_item_operations f_uac1_item_ops = {
	.release	= f_uac_attr_release,
};

#define UAC_ATTRIBUTE(name)						\
static ssize_t f_uac_opts_##name##_show(				\
					  struct config_item *item,	\
					  char *page)			\
{									\
	struct f_uac_opts *opts = to_f_uac_opts(item);		\
	int result;							\
									\
	mutex_lock(&opts->lock);					\
	result = sprintf(page, "%u\n", opts->name);			\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac_opts_##name##_store(				\
					  struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac_opts *opts = to_f_uac_opts(item);		\
	int ret;							\
	u32 num;							\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	ret = kstrtou32(page, 0, &num);					\
	if (ret)							\
		goto end;						\
									\
	opts->name = num;						\
	ret = len;							\
									\
end:									\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac_opts_, name)

#define UAC_RATE_ATTRIBUTE(name)					\
static ssize_t f_uac_opts_##name##_show(struct config_item *item,	\
					 char *page)			\
{									\
	struct f_uac_opts *opts = to_f_uac_opts(item);			\
	int result = 0;							\
	int i;								\
									\
	mutex_lock(&opts->lock);					\
	page[0] = '\0';							\
	for (i = 0; i < UAC_MAX_RATES; i++) {				\
		if (opts->name[i] == 0)					\
			continue;					\
		result += sprintf(page + strlen(page), "%u,",		\
				opts->name[i]);				\
	}								\
	if (strlen(page) > 0)						\
		page[strlen(page) - 1] = '\n';				\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac_opts_##name##_store(struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac_opts *opts = to_f_uac_opts(item);			\
	char *split_page = NULL;					\
	int ret = -EINVAL;						\
	char *token;							\
	u32 num;							\
	int i;								\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	i = 0;								\
	memset(opts->name, 0x00, sizeof(opts->name));			\
	split_page = kstrdup(page, GFP_KERNEL);				\
	while ((token = strsep(&split_page, ",")) != NULL) {		\
		ret = kstrtou32(token, 0, &num);			\
		if (ret)						\
			goto end;					\
									\
		opts->name[i++] = num;					\
		opts->name##_active = num;				\
		ret = len;						\
	};								\
									\
end:									\
	kfree(split_page);						\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac_opts_, name)

UAC_ATTRIBUTE(p_chmask);
UAC_ATTRIBUTE(p_ssize);
UAC_ATTRIBUTE(req_number);

UAC_RATE_ATTRIBUTE(p_srate);

static struct configfs_attribute *f_uac1_attrs[] = {
	&f_uac_opts_attr_p_chmask,
	&f_uac_opts_attr_p_srate,
	&f_uac_opts_attr_p_ssize,
	&f_uac_opts_attr_req_number,
	NULL,
};

static struct config_item_type f_uac1_func_type = {
	.ct_item_ops	= &f_uac1_item_ops,
	.ct_attrs	= f_uac1_attrs,
	.ct_owner	= THIS_MODULE,
};

static void f_audio_free_inst(struct usb_function_instance *f)
{
	struct f_uac_opts *opts;

	opts = container_of(f, struct f_uac_opts, func_inst);
	kfree(opts);
}

static struct usb_function_instance *f_audio_alloc_inst(void)
{
	struct f_uac_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = f_audio_free_inst;

	config_group_init_type_name(&opts->func_inst.group, "",
				    &f_uac1_func_type);

	opts->p_chmask = UAC_DEF_PCHMASK;
	opts->p_srate[0] = UAC_DEF_PSRATE;
	opts->p_srate_active = UAC_DEF_PSRATE;
	opts->p_ssize = UAC_DEF_PSSIZE;
	opts->req_number = UAC_DEF_REQ_NUM;
	return &opts->func_inst;
}

static void f_audio_free(struct usb_function *f)
{
	struct g_audio *audio;
	struct f_uac_opts *opts;

	audio = func_to_g_audio(f);
	opts = container_of(f->fi, struct f_uac_opts, func_inst);
	kfree(audio);
	mutex_lock(&opts->lock);
	--opts->refcnt;
	mutex_unlock(&opts->lock);
}

static void f_audio_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct g_audio *audio = func_to_g_audio(f);

	device_destroy(audio->class, MKDEV(audio->major, 0));
	cdev_del(&audio->cdev);
	unregister_chrdev_region(MKDEV(audio->major, 0), 1);
	class_destroy(audio->class);

	g_audio_cleanup(audio);
	usb_free_all_descriptors(f);

	audio->gadget = NULL;
}

static struct usb_function *f_audio_alloc(struct usb_function_instance *fi)
{
	struct f_uac *uac1;
	struct f_uac_opts *opts;

	/* allocate and initialize one new instance */
	uac1 = kzalloc(sizeof(*uac1), GFP_KERNEL);
	if (!uac1)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_uac_opts, func_inst);
	mutex_lock(&opts->lock);
	++opts->refcnt;
	mutex_unlock(&opts->lock);

	uac1->g_audio.func.name = "uac1_func";
	uac1->g_audio.func.bind = f_audio_bind;
	uac1->g_audio.func.unbind = f_audio_unbind;
	uac1->g_audio.func.set_alt = f_audio_set_alt;
	uac1->g_audio.func.get_alt = f_audio_get_alt;
	uac1->g_audio.func.setup = f_audio_setup;
	uac1->g_audio.func.disable = f_audio_disable;
	uac1->g_audio.func.free_func = f_audio_free;
	uac1->g_audio.func.bind_deactivated = true;

	return &uac1->g_audio.func;
}

DECLARE_USB_FUNCTION_INIT(uac1, f_audio_alloc_inst, f_audio_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ruslan Bilovol");
