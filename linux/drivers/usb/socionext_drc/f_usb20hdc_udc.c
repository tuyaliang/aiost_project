/*
 * linux/drivers/usb/gadget/f_usb20hdc_udc.c - F_USB20HDC USB function
 * controller driver
 *
 * Copyright (C) SOCIONEXT ELECTRONICS INC. 2011-2012. All rights reserved.
 * Copyright (C) 2012-2015 SOCIONEXT SEMICONDUCTOR LIMITED.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>

#include "f_usb20hdc_udc.h"

static const struct endpont_cb ep_config_data[HDC_UDC_MAX_EP] = {
	/* endpoint 0 */
	[0] = {
		.name					= "ep0",
		.hs_maxpacket				= 64,
		.fs_maxpacket				= 64,
		.buffer_size				= 64,
		.buffers				= 1,
		.pio_auto_change			= 0,
		.trans_end_timing			= 0,
		.dma_ch					= -1,
	},
	[1] = {
		.name					= "ep1-int",
		.hs_maxpacket				= 1024,
		.fs_maxpacket				= 64,
		.buffer_size				= 1024,
		.buffers				= 1,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= -1,
	},
	[2] = {
		.name					= "ep2-int",
		.hs_maxpacket				= 1024,
		.fs_maxpacket				= 64,
		.buffer_size				= 1024,
		.buffers				= 1,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= -1,
	},
	[3] = {
		.name					= "ep3-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= 0,
	},
	[4] = {
		.name					= "ep4-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= 1,
	},
	[5] = {
		.name					= "ep5-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= 0,
	},
	[6] = {
		.name					= "ep6-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.dma_ch					= 1,
	},
};

static inline u32 hdc_readl_cached(void __iomem *base, u32 offset)
{
	u32 reg;

	reg = __raw_readl(base + offset);

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_epcmd_cache_bits_dev_mode(base, offset, &reg);
	else
		hdc_reg_cache_bits(base, offset, &reg);

	return reg;
}

static void hdc_write_bits(void __iomem *base, u32 offset, u8 start_bit,
			   u8 bit_length, u32 value)
{
	u32 reg;

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);

	reg = __raw_readl(base + offset);

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_epcmd_cache_bits_dev_mode(base, offset, &reg);
	else
		hdc_reg_cache_bits(base, offset, &reg);

	reg &= ~(MASK(bit_length) << start_bit);
	reg |= ((value & MASK(bit_length)) << start_bit);

	__raw_writel(reg, base + offset);
	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);
}

/*endpoint cmd reg which will have different function in different mode */
static void hdc_set_epcmd_stop(void __iomem *base_addr, u8 ep_ch)
{
	hdc_write_bits(base_addr, HDC_EPCMD(ep_ch), BIT_STOP, 1, 1);
	hdc_read_bits(base_addr, HDC_DEVS, BIT_CRTSPEED, 1) ?
			udelay(250) : mdelay(2);
}

static void hdc_set_connect_detect(void __iomem *base, u8 connect)
{
	if (connect) {
		/* set bus connect detect */
		hdc_write_bits(base, HDC_OTGSTSFALL, BIT_VBUS_VLD_FEN, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSRISE, BIT_VBUS_VLD_REN, 1, 1);
	} else {
		/* set bus disconnect detect */
		hdc_write_bits(base, HDC_OTGSTSRISE, BIT_VBUS_VLD_REN, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSFALL, BIT_VBUS_VLD_FEN, 1, 1);
	}
}

static u8 is_bus_connected(void __iomem *base)
{
	/* get bus connected */
	return hdc_read_bits(base, HDC_OTGSTS_R_ONLY, BIT_VBUS_VLD, 1) ? 1 : 0;
}

static void hdc_set_dplus_pullup(void __iomem *base, u8 enable)
{
	if (enable) {
		/* pull-up D+ terminal */
		hdc_write_bits(base, HDC_DEVC, BIT_PHYSUSP, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_DISCONNECT, 1, 0);
	} else {
		/* pull-down D+ terminal */
		hdc_write_bits(base, HDC_DEVC, BIT_DISCONNECT, 1, 1);
		hdc_write_bits(base, HDC_DEVC, BIT_PHYSUSP, 1, 1);
	}
}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static u8 set_dma_transfer(struct f_usb20hdc_udc_ep *ep, dma_addr_t source,
			   dma_addr_t destination, u32 bytes, u8 last_transfer)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	s8 dma_ch = ep->dma_ch;
	struct f_usb20hdc_dma_data *dma_data =
					&priv->dma_data[dma_ch];
	int result;
	int dma_mode = MODE_DEMAND;

	if (priv->f_otg->variant == FHDC_VARIANT_LAP)
		dma_mode = MODE_BLOCK;

	/* check DMA channel */
	if (unlikely(dma_ch == -1))
		return 0;

	/* set HDMAC transfer request parameter */
	dma_data->hdmac_req.dmacb = HDMACB_TT_2CYCLE | HDMACB_MS_BLOCK |
					HDMACB_EI | HDMACB_CI | HDMACB_TW_WORD;
	dma_data->hdmac_req.dmacb |= ep->transfer_direction ?
							HDMACB_FD : HDMACB_FS;

	dma_data->hdmac_req.dmaca = dma_data->dreq | HDMACA_BT_INCR16 |
							((bytes + 63) / 64 - 1);
	hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_BLKSIZE,
		       LEN_DMA_BLKSIZE, 64);

// trial! set macro top register.
	if (priv->f_otg->variant == FHDC_VARIANT_LAP) {
		__raw_writel(0x30, base + 0x12000);
		__raw_writel(0x30, base + 0x12004);
	}

	dma_data->hdmac_req.req_data.size = bytes;
	dma_data->hdmac_req.req_data.src = source;
	dma_data->hdmac_req.req_data.dst = destination;
	dma_data->hdmac_req.req_data.irq_data = ep;
	dma_data->hdmac_req.req_data.callback_fn = on_end_dma_transfer;

	result = hdmac_enqueue(dma_data->hdmac_ch, &dma_data->hdmac_req);
	if (result) {
		dev_err(priv->gadget.dev.parent,
			"%s():HDMAC request enqueue failed at 0x%d\n",
							__func__, result);
		return 0;
	}

	/* setup F_USB20HDC controller register */
	hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_MODE, 1, dma_mode);
	if (ep->transfer_direction) {
		if (last_transfer) {
			hdc_write_bits(base,  HDC_DMAC(dma_ch),
				       BIT_DMA_SENDNULL, 1,
				       ep->null_packet ? 1 : 0);
			hdc_write_bits(base, HDC_DMAC(dma_ch),
				       BIT_DMA_INT_EMPTY, 1,
				       ep->in_trans_end_timing ? 1 : 0);
		} else {
			hdc_write_bits(base, HDC_DMAC(dma_ch),
				       BIT_DMA_SENDNULL, 1, 0);
			hdc_write_bits(base, HDC_DMAC(dma_ch),
				       BIT_DMA_INT_EMPTY, 1, 0);
		}
	} else {
		hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_SPR, 1, 0);
	}

	hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_EP, LEN_DMA_EP, ep_ch);
	hdc_writel(base, HDC_DMATCI(dma_ch), bytes);

	return 1;
}

static void enable_dma_transfer(struct f_usb20hdc_udc_ep *ep, u8 enable)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	s8 dma_ch = ep->dma_ch;

	/* check DMA channel */
	if (unlikely(dma_ch == -1))
		return;

	if (enable) {
		/* enable DMA transfer */
		if (hdmac_start(priv->dma_data[ep->dma_ch].hdmac_ch) != 0) {
			ep->dma_transfer = 0;
			dev_err(priv->gadget.dev.parent,
				"%s() hdmac_start() failed\n", __func__);
			return;
		}
		hdc_write_bits(base, HDC_INTEN, BIT_DMA_INTEN(dma_ch), 1, 1);
		hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_ST, 1, 1);
	} else {
		/* disable DMA transfer */
		hdc_write_bits(base, HDC_INTEN, BIT_DMA_INTEN(dma_ch), 1, 0);
		hdc_write_bits(base, HDC_DMAC(dma_ch), BIT_DMA_ST, 1, 0);
		hdc_write_bits(base, HDC_INTS, BIT_DMA_INT(dma_ch), 1, 0);
		ep->dma_transfer = 0;
		priv->dma_data[ep->dma_ch].ep_ch = -1;
	}
}
#endif

static u8 set_in_transfer_dma(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req = ep->req;
	dma_addr_t dma_addr;
	u32 bytes;

	if (unlikely(!ep->ep.desc)) {
		enable_dma_transfer(ep, 0);
		return 0;
	}

	/* check NULL packet IN transfer for last packet of transaction */
	if ((req->req.length) && (!(req->req.length %
		ep->ep.maxpacket)) && (req->req.zero))
		ep->null_packet = 1;

	/* calculate this time transfer byte */
	bytes = req->req.length < HDC_UDC_DMA_TRANS_MAX_BYTES ?
		req->req.length : HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / ep->ep.maxpacket)
		bytes -= bytes % ep->ep.maxpacket;

	/* check DMA transfer buffer mapping */
	if ((req->req.dma == ~(dma_addr_t)0) ||
	    (req->req.dma == (dma_addr_t)0)) {
		/* map DMA transfer buffer and synchronize DMA transfer buffer*/
		req->req.dma = dma_map_single(priv->gadget.dev.parent,
					      req->req.buf, req->req.length,
					      DMA_TO_DEVICE);
	}

	/* check DMA transfer address align */
	if (req->req.dma & 0x3) {
		dev_dbg(priv->dev, "%s() DMA buffer not aligned 0x%llX\n",
			__func__, (u64)req->req.dma);
		enable_dma_transfer(ep, 0);
		if (req->dma_transfer_buffer_map) {
			dma_unmap_single(priv->gadget.dev.parent,
					 req->req.dma, req->req.length,
					 DMA_TO_DEVICE);
			req->req.dma = ~(dma_addr_t)0;
		}
		return 0;
	}

	/* set dma transfer source address */
	dma_addr = req->req.dma;

	/* setup DMA transfer */
	if (!set_dma_transfer(ep, dma_addr,
			      priv->dma_data[ep->dma_ch].epbuf_daddr,
			      bytes, req->req.length <= bytes ? 1 : 0)) {
		dev_err(priv->dev, "%s() set_dma_transfer() failed\n",
			__func__);
		abort_in_transfer_dma(ep, 0);

		return 0;
	}

	/* set request execute */
	req->request_execute = 1;
	req->dmac_int_occurred = 0;
	req->usb_dma_int_occurred = 0;

	/* enable DMA transfer */
	enable_dma_transfer(ep, 1);

	return 1;
#else
	return 0;
#endif
}

static u8 set_out_transfer_dma(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req = ep->req;
	dma_addr_t dma_addr;
	u32 bytes;

	if (unlikely(!ep->ep.desc)) {
		enable_dma_transfer(ep, 0);
		return 0;
	}

	/* calculate this time transfer byte */
	bytes = req->req.length < HDC_UDC_DMA_TRANS_MAX_BYTES ?
		req->req.length : HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / ep->ep.maxpacket)
		bytes -= bytes % ep->ep.maxpacket;

	/* check DMA transfer buffer mapping */
	if ((req->req.dma == ~(dma_addr_t)0) ||
	    (req->req.dma == (dma_addr_t)0)) {
		/* map DMA transfer buffer and synchronize DMA transfer buffer*/
		req->req.dma = dma_map_single(priv->gadget.dev.parent,
					      req->req.buf, req->req.length,
					      DMA_FROM_DEVICE);
	}

	/* check DMA transfer address align and DMA transfer length */
	if ((req->req.dma & 0x3) || (req->req.length & 0x3)) {
		enable_dma_transfer(ep, 0);
		if (req->dma_transfer_buffer_map) {
			dma_unmap_single(priv->gadget.dev.parent,
					 req->req.dma, req->req.length,
					 DMA_FROM_DEVICE);
			req->req.dma = ~(dma_addr_t)0;
		}
		return 0;
	}

	/* set dma transfer destination address */
	dma_addr = req->req.dma;

	/* setup DMA transfer */
	if (!set_dma_transfer(ep, priv->dma_data[
	    ep->dma_ch].epbuf_daddr, dma_addr, bytes, 0)) {
		dev_err(priv->dev, "%s() %d\n", __func__, __LINE__);
		abort_out_transfer_dma(ep, 0);

		return 0;
	}

	/* set request execute */
	req->request_execute = 1;

	/* enable DMA transfer */
	enable_dma_transfer(ep, 1);

	return 1;
#else
	return 0;
#endif
}

static void initialize_endpoint(struct f_usb20hdc_udc_ep *, u8, u8, u8);
static void abort_in_transfer_dma(struct f_usb20hdc_udc_ep *ep, u8 init)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)

	/* disable DMA transfer */
	enable_dma_transfer(ep, 0);

	/* clear NULL packet transfer */
	ep->null_packet = 0;

	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(ep, 1, 0, 0);
#endif
}

static void abort_out_transfer_dma(struct f_usb20hdc_udc_ep *ep, u8 init)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)

	/* disable DMA transfer */
	enable_dma_transfer(ep, 0);

	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(ep, 1, 0, 0);
#endif
}

static u8 end_in_transfer_dma(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req = ep->req;
	s8 dma_ch = ep->dma_ch;
	dma_addr_t dma_addr;
	u32 bytes;
	void __iomem *base = priv->reg_base;

	/* get this time transfer byte */
	bytes = hdc_readl(base, HDC_DMATC_R_ONLY(dma_ch));

	/* update actual bytes */
	req->req.actual += bytes;

	/* check transfer remain byte */
	if (!(req->req.length - req->req.actual)) {
		/* complete request */
		abort_in_transfer_dma(ep, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (req->req.length - req->req.actual) <
			HDC_UDC_DMA_TRANS_MAX_BYTES ?
			(req->req.length - req->req.actual) :
				HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / ep->ep.maxpacket)
		bytes -= bytes % ep->ep.maxpacket;

	/* update dma transfer source address */
	dma_addr = (dma_addr_t)(req->req.dma + req->req.actual);

	/* setup DMA transfer */
	if (!set_dma_transfer(ep, dma_addr,
			      priv->dma_data[dma_ch].epbuf_daddr,
			      bytes, req->req.length <= bytes ? 1 : 0)) {
		dev_err(priv->dev, "%s() set_dma_transfer() failed\n",
			__func__);
		abort_in_transfer_dma(ep, 0);

		return 0;
	}

	/* clear hdmac and usb dma int flag */
	req->dmac_int_occurred = 0;
	req->usb_dma_int_occurred = 0;

	/* enable DMA transfer */
	enable_dma_transfer(ep, 1);
#endif
	return 0;
}

static u8 end_out_transfer_dma(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req = ep->req;
	s8 dma_ch = ep->dma_ch;
	dma_addr_t dma_addr;
	u32 bytes;

	/* get this time transfer byte */
	bytes = hdc_readl(base, HDC_DMATC_R_ONLY(dma_ch));

	/* update actual bytes */
	req->req.actual += bytes;

	/* check transfer request complete */
	if ((req->req.length <= req->req.actual) ||
	    (hdc_read_bits(base, HDC_DMAS_R_ONLY(dma_ch), BIT_DMA_SP, 1))) {
		/* complete request */
		abort_out_transfer_dma(ep, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (req->req.length - req->req.actual) <
			HDC_UDC_DMA_TRANS_MAX_BYTES ?
			(req->req.length - req->req.actual) :
			HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / ep->ep.maxpacket)
		bytes -= bytes % ep->ep.maxpacket;

	/* update dma transfer destination address */
	dma_addr = (dma_addr_t)(req->req.dma + req->req.actual);

	/* setup DMA transfer */
	if (!set_dma_transfer(ep, priv->dma_data[
	    dma_ch].epbuf_daddr, dma_addr, bytes, 0)) {
		dev_err(priv->dev, "%s() set_dma_transfer() failed\n",
			__func__);
		abort_out_transfer_dma(ep, 0);

		return 0;
	}

	/* enable DMA transfer */
	enable_dma_transfer(ep, 1);
#endif
	return 0;
}

static u8 is_dma_transfer_usable(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;

	if ((ep->dma_ch == -1) || (ep->dma_transfer) ||
	    (priv->dma_data[ep->dma_ch].ep_ch != -1))
		return 0;
	priv->dma_data[ep->dma_ch].ep_ch = ep->ep_ch;
	ep->dma_transfer = 1;
	return 1;
#else
	return 0;
#endif
}

static u8 is_pio_transfer_auto_change_usable(struct f_usb20hdc_udc_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	return (ep->pio_auto_change) || (ep->dma_ch == -1);
#else
	return 1;
#endif
}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static void dequeue_all_transfer_request(struct f_usb20hdc_udc_ep *, int);
static void on_end_in_transfer(struct f_usb20hdc_udc_ep *endpoint);
static void on_end_out_transfer(struct f_usb20hdc_udc_ep *endpoint);
static void on_end_dma_transfer(u32 channel, void *data, int state)
{
	struct f_usb20hdc_udc_ep *ep = data;
	struct f_usb20hdc_udc *priv = ep->priv;

	spin_lock(&priv->lock);

	if (unlikely(!ep || !ep->req ||
		     (channel != ep->priv->dma_data[ep->dma_ch].hdmac_ch))) {
		goto end;
	}

	/* DMA transfer end interrupt factor */
	switch (state) {
	case HDMACB_SS_ADD_OVERFLOW:
	case HDMACB_SS_SOURCE_ACCESS_ERROR:
	case HDMACB_SS_DESTINATION_ACCESS_ERROR:
		/* dequeue all previous requests */
		dequeue_all_transfer_request(ep, -EL2HLT);

		break;
	case HDMACB_SS_TRANSFER_STOP_REQUEST:
		ep->transfer_direction ?
			on_end_in_transfer(ep) : on_end_out_transfer(ep);
		break;
	case HDMACB_SS_NORMAL_END:
		if (ep->transfer_direction) {
			ep->req->dmac_int_occurred = 1;
			if (ep->req->usb_dma_int_occurred)
				on_end_in_transfer(ep);
		} else {
			on_end_out_transfer(ep);
		}
		break;
	default:
		break;
	}
end:
	spin_unlock(&priv->lock);
}
#endif

static void initialize_udc_controller(struct f_usb20hdc_udc *priv, u8 preinit)
{
	u32 ch;
	struct device *dev = priv->dev;
	void __iomem *base = priv->reg_base;

	/*
	 * check host mode
	 * [notice]:It is processing nothing, when host_en bit is set
	 */
	if (hdc_read_bits(base, HDC_MODE, BIT_HOST_EN, 1))
		return;

	/* check pre-initialize */
	if (preinit) {
		dev_dbg(dev, "initialize_udc_controller(): preinit\n");
		/* initialize F_USB20HDC system configuration register */
		hdc_core_soft_reset(priv->f_otg);

		/* pre-initialize F_USB20HDC otg register */
		hdc_write_bits(base, HDC_OTGC, BIT_DM_PULL_DOWN, 1, 0);
		hdc_write_bits(base, HDC_OTGC, BIT_DP_PULL_DOWN, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSC, BIT_TMROUT_C, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSC, BIT_VBUS_VLD_C, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSRISE, BIT_TMROUT_REN, 1, 0);
		hdc_write_bits(base, HDC_OTGTC, BIT_START_TMR, 1, 0);
		hdc_write_bits(base, HDC_OTGT, BIT_TMR_INIT_VAL,
			       LEN_TMR_INIT_VAL, 0);
		return;
	}
	dev_dbg(dev, "initialize_udc_controller(): non-preinit\n");

	/*
	 * initialize F_USB20HDC mode register
	 * [notice]:set of dev_int_mode / dev_addr_load_mode bit is prohibition
	 */
	hdc_write_bits(base, HDC_MODE, BIT_DEV_EN, 1, 0);

	/*
	 * initialize F_USB20HDC global interrupt register
	 * [notice]:otg_inten bit is always enable
	 */
	hdc_disable_interrupt(base, HDC_UDC_MAX_EP);

	/* initialize F_USB20HDC device control / status / address register */
	hdc_write_bits(base, HDC_DEVC, BIT_REQSPEED, 1,
		       gadget_is_dualspeed(&priv->gadget) ?
		       REQ_SPEED_HIGH_SPEED : REQ_SPEED_FULL_SPEED);
	hdc_write_bits(base, HDC_DEVC, BIT_REQRESUME, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_RMTWKUP, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDE_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDB_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_SOF_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_SETUP_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_USBRSTE_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_USBRSTB_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_STATUS_OK_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVC, BIT_STATUS_NG_INTEN, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDE_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDB_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_SOF_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_SETUP_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_USBRSTE_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_USBRSTB_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_STATUS_OK_INT, 1, 0);
	hdc_write_bits(base, HDC_DEVS, BIT_STATUS_NG_INT, 1, 0);
	hdc_write_bits(base, HDC_FADDR, BIT_FUNC_ADDR, LEN_FUNC_ADDR, 0);
	hdc_write_bits(base, HDC_FADDR, BIT_DEV_CONFIGURED, 1, 0);

	/* initialize F_USB20HDC dma register */
        if (priv->f_otg->variant == FHDC_VARIANT_LAP)
                hdc_dma_if_dear(base, MODE_BLOCK);
        else
                hdc_dma_if_dear(base, MODE_DEMAND);

	/* initialize F_USB20HDC test control register */
	hdc_write_bits(base, HDC_TESTC, BIT_TEST_P, 1, 0);
	hdc_write_bits(base, HDC_TESTC, BIT_TEST_J, 1, 0);
	hdc_write_bits(base, HDC_TESTC, BIT_TEST_K, 1, 0);
	hdc_write_bits(base, HDC_TESTC, BIT_TEST_SE0NACK, 1, 0);

	/* initialize F_USB20HDC ram register */
	for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++) {
		hdc_writel(base, HDC_EPCTRL(ch), ch == EP0 ? 0x500 : 0x400);
		hdc_writel(base, HDC_EPCONF(ch), 0x00000000);
	}
	for (ch = 0; ch < HDC_UDC_MAX_EP * EP_BUF_CNT; ch++)
		hdc_writel(base, HDC_EPCOUNT(ch), 0x00000000);

	/* wait PHY reset release */
	for (ch = 0xffff;
	     (ch && hdc_read_bits(base, HDC_DEVS, BIT_PHYRESET, 1));
	     ch--)
		;
}

static void initialize_endpoint(struct f_usb20hdc_udc_ep *ep, u8 fifo, u8 stall,
				u8 toggle)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u8 endpoint_enable = hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
		BIT_EP_EN, 1);

	if (ep_ch == EP0) {
		/* check new SETUP transfer */
		if ((hdc_read_bits(base, HDC_DEVS, BIT_SETUP_INT, 1)) ||
		    (hdc_read_bits(base, HDC_DEVS, BIT_USBRSTB_INT, 1)) ||
		    (hdc_read_bits(base, HDC_DEVS, BIT_USBRSTE_INT, 1)) ||
		    (hdc_read_bits(base, HDC_DEVS, BIT_BUSRESET, 1))) {
			fifo = 0;
			stall = 0;
			toggle = 0;
		}

		if (fifo) {
			if (endpoint_enable &&
			    hdc_read_epctrl_bits(base, HDC_EPCTRL(EP0),
						 BIT_EP_EN, 1))
				/* disable endpoint */
				hdc_set_epcmd_stop(base, EP0);

			/* initialize FIFO */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_INIT, 1, 1);
		}

		if (stall) {
			/* initialize endpoint stall */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_STALL_CLR,
				       1, 1);
		}

		if (toggle) {
			/* initialize endpoint data toggle bit */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_TOGGLE_CLR,
				       1, 1);
		}

		/* initialize endpoint control / status register */
		if (fifo) {
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INT_CLR, 1, 1);
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INTEN, 1, 0);
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYO_EMPTY_INTEN, 1, 0);
		}
		hdc_write_bits(base, HDC_EPCMD(EP0), BIT_PING_INTEN, 1, 0);
		hdc_write_bits(base, HDC_EPCMD(EP0), BIT_STALLED_INTEN, 1, 1);
		hdc_write_bits(base, HDC_EPCMD(EP0), BIT_NACK_INTEN, 1, 0);
		hdc_write_bits(base, HDC_INTEN, BIT_DEV_EP_INTEN(EP0), 1, 1);

		if (endpoint_enable &&
		    !hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch), BIT_EP_EN,
					  1))
			/* re-enable endpoint */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_START, 1, 1);
	} else {
		if (fifo) {
			if ((endpoint_enable) &&
			    (hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
						  BIT_EP_EN, 1)))
				/* disable endpoint */
				hdc_set_epcmd_stop(base, ep_ch);

			/* initialize FIFO */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_INIT, 1, 1);
		}

		if (stall) {
			/* initialize endpoint stall */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_STALL_CLR,
				       1, 1);
		}

		if (toggle) {
			/* initialize endpoint data toggle bit */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_TOGGLE_CLR,
				       1, 1);
		}

		/* initialize endpoint control / status register */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYI_RDY_INT_CLR,
			       1, 1);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYO_EMPTY_INT_CLR,
			       1, 1);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYI_RDY_INTEN,
			       1, 0);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYO_EMPTY_INTEN,
			       1, 0);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_PING_INTEN, 1, 0);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_STALLED_INTEN, 1, 1);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_NACK_INTEN, 1, 0);
		hdc_write_bits(base, HDC_INTEN, BIT_DEV_EP_INTEN(ep_ch), 1, 1);

		if ((endpoint_enable) &&
		    (!hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch), BIT_EP_EN,
					   1)))
			/* re-enable endpoint */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
	}
}

static void initialize_endpoint_configure(struct f_usb20hdc_udc *priv)
{
	u32 ep_ch, i;
	struct f_usb20hdc_udc_ep *ep;
	void *base = priv->reg_base;
	u8 high_speed = priv->gadget_driver ?
			priv->gadget_driver->max_speed == USB_SPEED_HIGH ?
			1 : 0 : 1;

	/* initialzie endpoint 0 configure data */
	ep = &priv->ep[EP0];
	ep->ep.name = ep_config_data[EP0].name;
	ep->ep.maxpacket = high_speed ?
			ep_config_data[EP0].hs_maxpacket :
			ep_config_data[EP0].fs_maxpacket;
	ep->ep_ch = EP0;
	ep->transfer_direction = 0;
	ep->transfer_type = USB_ENDPOINT_XFER_CONTROL;
	ep->buf_offset[0] = hdc_get_epbuf_addr_offset();
	ep->buf_offset[1] = ep->buf_offset[0] + ep->ep.maxpacket;
	ep->buf_offset[1] = (ep->buf_offset[1] + 0x3) & ~0x3;
	ep->buffer_size = ep_config_data[EP0].buffer_size;
	ep->buffers = ep_config_data[EP0].buffers;
	ep->pio_auto_change = ep_config_data[EP0].pio_auto_change;
	ep->in_trans_end_timing = ep_config_data[EP0].trans_end_timing;
	ep->dma_ch = ep_config_data[EP0].dma_ch;

	/* configure endpoint 0 */
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_ET, LEN_ET, TYPE_CONTROL);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_DIR, 1, 0);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BNUM, LEN_BNUM, 1 - 1);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_HIBAND, LEN_HIBAND, 0);
	hdc_write_bits(base, HDC_EPCONF(EP0), BIT_BASE, LEN_BASE,
		       (ep->buf_offset[0] & 0x7fff) >> 2);
	hdc_write_bits(base, HDC_EPCONF(EP0), BIT_SIZE, LEN_SIZE,
		       ep->ep.maxpacket);
	hdc_write_bits(base, HDC_EPCONF(EP0), BIT_COUNTIDX, LEN_COUNTIDX, 0);

	for (ep_ch = EP1; ep_ch < HDC_UDC_MAX_EP; ep_ch++) {
		/* initialzie endpoint configure data */
		ep = &priv->ep[ep_ch];
		ep->ep.name = ep_config_data[ep_ch].name;
		ep->ep.maxpacket = high_speed ?
					ep_config_data[ep_ch].hs_maxpacket :
					ep_config_data[ep_ch].fs_maxpacket;
		ep->ep.maxpacket_limit = ep->ep.maxpacket;
		ep->ep_ch = ep_ch;
		ep->transfer_direction = 0;
		ep->transfer_type = 0;
		ep->buffer_size = ep_config_data[ep_ch].buffer_size;
		ep->buffers = ep_config_data[ep_ch].buffers;
		ep->buf_offset[0] = priv->ep[ep_ch - 1].buf_offset[
				priv->ep[ep_ch - 1].buffers - 1] +
				priv->ep[ep_ch - 1].buffer_size;
		ep->buf_offset[0] = (ep->buf_offset[0] + 0x3) & ~0x3;
		for (i = 1; i < ep->buffers; i++) {
			ep->buf_offset[i] = ep->buf_offset[i - 1] +
						ep->buffer_size;
			ep->buf_offset[i] = (ep->buf_offset[i] + 0x3) & ~0x3;
		}
		ep->pio_auto_change = ep_config_data[ep_ch].pio_auto_change;
		ep->in_trans_end_timing = ep_config_data[ep_ch]
						.trans_end_timing;
		ep->dma_ch = ep_config_data[ep_ch].dma_ch;
	}
}

static u8 configure_endpoint(struct f_usb20hdc_udc_ep *ep)
{
	static const u8 transfer_type_register[] = {
		TYPE_CONTROL,		/* control transfer */
		TYPE_ISOCHRONOUS,	/* isochronout transfer */
		TYPE_BULK,		/* bulk transfer */
		TYPE_INTERRUPT,		/* interrupt transfer */
	};
	u32 i;
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	enum usb_device_speed speed = priv->gadget.speed;
	u8 ep_ch = ep->ep.desc->bEndpointAddress &
				USB_ENDPOINT_NUMBER_MASK;
	u8 ep_dir = ep->ep.desc->bEndpointAddress &
				USB_ENDPOINT_DIR_MASK ? 1 : 0;
	u8 ep_type = ep->ep.desc->bmAttributes &
				USB_ENDPOINT_XFERTYPE_MASK;
	u16 maxpacket = usb_endpoint_maxp(ep->ep.desc) & 0x7ff;
	u16 ep_hiband = (usb_endpoint_maxp(ep->ep.desc) >> 11)
				& 0x3;
	u16 ep_buf_offset = ep->buf_offset[0];
	u16 ep_bufs = ep->buffers;

	/* check endpoint transfer buffer size */
	if (ep->buffer_size < maxpacket)
		return 0;

	/* check endpoint transfer packet maximum byte count violation */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		if (((speed == USB_SPEED_FULL) && (maxpacket % 8) &&
		     (maxpacket > 64)) || ((speed == USB_SPEED_HIGH) &&
		     (maxpacket != 64)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (((speed == USB_SPEED_FULL) && (maxpacket > 1023)) ||
		    ((speed == USB_SPEED_HIGH) && (maxpacket > 1024)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (((speed == USB_SPEED_FULL) && (maxpacket > 64)) ||
		    ((speed == USB_SPEED_HIGH) && (maxpacket > 1024)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_BULK:
		if (((speed == USB_SPEED_FULL) &&
		     (maxpacket % 8) && (maxpacket > 64)) ||
		     ((speed == USB_SPEED_HIGH) && (maxpacket != 512)))
			return 0;
		break;
	default:
		return 0;
	}

	/* set endpoint configure data */
	ep->ep.maxpacket = maxpacket;
	ep->transfer_direction = ep_dir;
	ep->transfer_type = ep_type;
	for (i = 1; i < EP_BUF_CNT; i++) {
		ep->buf_offset[i] = ep->buf_offset[i - 1] + maxpacket;
		ep->buf_offset[i] = (ep->buf_offset[i] + 0x3) & ~0x3;
	}

	/* configure endpoint x */
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_ET, LEN_ET,
		       transfer_type_register[ep_type]);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_DIR, 1, ep_dir ? 1 : 0);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BNUM, LEN_BNUM, ep_bufs - 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_HIBAND, LEN_HIBAND,
		       ep_hiband);
	hdc_write_bits(base, HDC_EPCONF(ep_ch), BIT_BASE, LEN_BASE,
		       (ep_buf_offset & 0x7fff) >> 2);
	hdc_write_bits(base, HDC_EPCONF(ep_ch), BIT_SIZE, LEN_SIZE,
		       maxpacket);
	hdc_write_bits(base, HDC_EPCONF(ep_ch), BIT_COUNTIDX, LEN_COUNTIDX,
		       ep_ch * EP_BUF_CNT);

	return 1;
}

static u8 is_endpoint_buffer_usable(struct f_usb20hdc *f_otg)
{
	u32 ep_ch;
	u32 buffer_size = 0;

	/* calculate RAM buffer size */
	buffer_size += 256;
	buffer_size += ep_config_data[EP0].buffer_size * EP_BUF_CNT;
	for (ep_ch = EP1; ep_ch < HDC_UDC_MAX_EP; ep_ch++)
		buffer_size += ep_config_data[ep_ch].buffer_size *
			ep_config_data[ep_ch].buffers;

	if (f_otg->variant == FHDC_VARIANT_LAP)
		return buffer_size <= HDC_UDC_EP_BUFFER_RAM_SIZE_LAP;

	return buffer_size <= HDC_UDC_EP_BUFFER_RAM_SIZE;
}

static void enable_endpoint(struct f_usb20hdc_udc_ep *ep, u8 enable)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	u8 ep_ch = ep->ep_ch;
	void __iomem *base = priv->reg_base;

	if (enable) {
		/* initialize endpoint */
		initialize_endpoint(ep, 1, 1, 1);

		/* set endpoint parameter */
		ep->halt = 0;
		ep->null_packet = 0;

		/* enable endpoint */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
	} else {
		/*
		 * [notice]:use of an abort_xx_transfer() is prohibition,
		 * because cache becomes panic.
		 */
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		/* disable DMA transfer */
		enable_dma_transfer(ep, 0);
#endif
		/* disable endpoint */
		hdc_set_epcmd_stop(base, ep_ch);

		/* initialize endpoint */
		initialize_endpoint(ep, 1, 1, 1);

		/* set endpoint parameter */
		ep->halt = 1;
		ep->force_halt = 0;
		ep->null_packet = 0;
	}
}

static void set_bus_speed(struct f_usb20hdc_udc *priv)
{
	/* set current bus speed */
	priv->gadget.speed =
		hdc_read_bits(priv->reg_base, HDC_DEVS, BIT_CRTSPEED, 1) ==
			CRT_SPEED_HIGH_SPEED ? USB_SPEED_HIGH : USB_SPEED_FULL;

	/* set endpoint 0 max packet */
	priv->ep[EP0].ep.maxpacket = priv->gadget.speed == USB_SPEED_HIGH ?
		ep_config_data[EP0].hs_maxpacket :
		ep_config_data[EP0].fs_maxpacket;
	hdc_write_bits(priv->reg_base, HDC_EPCONF(EP0),
		       BIT_SIZE, LEN_SIZE, priv->ep[EP0].ep.maxpacket);
}

static void set_device_state(struct f_usb20hdc_udc *priv, u8 device_state)
{
	dev_dbg(priv->dev, "%s() is started from %pS\n",
		__func__, __builtin_return_address(0));

	if (priv->device_state == device_state)
		return;

	/* set device state */
	priv->device_state_last = priv->device_state;
	priv->device_state = device_state;
	dev_dbg(priv->dev, "device state:%u, last state:%u\n",
		device_state, priv->device_state_last);
}

static u16 get_fifo_bytes(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u8 index = hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
		BIT_APPPTR, LEN_APPPTR);

	/* get current bytes in FIFO */
	return ep->transfer_direction ?
		hdc_read_bits(base,
			      HDC_EPCOUNT(ep_ch == EP0 ?
					  0 : ep_ch * EP_BUF_CNT + index),
			      BIT_APPCNT, LEN_APPCNT) :
		hdc_read_bits(base,
			      HDC_EPCOUNT(ep_ch == EP0 ?
					  1 : ep_ch * EP_BUF_CNT + index),
			      BIT_PHYCNT, LEN_PHYCNT);
}

static void notify_transfer_request_complete(struct f_usb20hdc_udc_ep *ep,
					     int status)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 halt = ep->halt;

	if (!req)
		return;

	/* delete and initialize list */
	list_del_init(&req->queue);

	/* set request status */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* clear request execute */
	req->request_execute = 0;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	req->dmac_int_occurred = 0;
	req->usb_dma_int_occurred = 0;
#endif
	ep->req = NULL;

	/* check request complete notify for gadget driver */
	if (!req->req.complete)
		return;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		/* check DMA transfer buffer unmap */
		if ((req->req.dma != ~(dma_addr_t)0) &&
		    (req->req.dma != (dma_addr_t)0)) {
			/*
			 * unmap DMA transfer buffer and
			 * synchronize DMA transfer buffer
			 */
			dma_unmap_single(priv->gadget.dev.parent,
					 req->req.dma, req->req.length,
					 ep->transfer_direction ?
					 DMA_TO_DEVICE : DMA_FROM_DEVICE);
			req->req.dma = ~(dma_addr_t)0;
		}
#endif

	/* notify request complete for gadget driver */
	ep->halt = 1;
	spin_unlock(&priv->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&priv->lock);
	ep->halt = halt;
}

static void dequeue_all_transfer_request(struct f_usb20hdc_udc_ep *ep,
					 int status)
{
	u32 i;
	struct f_usb20hdc_udc_req *req;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *priv = ep->priv;

	if (ep->dma_transfer) {
		/* abort in / out DMA transfer */
		ep->transfer_direction ?
		abort_in_transfer_dma(ep, 1) :
		abort_out_transfer_dma(ep, 1);
		spin_unlock(&priv->lock);
		hdmac_stop_nowait(priv->dma_data[ep->dma_ch].hdmac_ch);
		spin_lock(&priv->lock);
	}
#endif
	/* dequeue all transfer request */
	for (i = (u32)-1; i && !list_empty(&ep->queue); i--) {
		req = list_entry(ep->queue.next, struct f_usb20hdc_udc_req,
				 queue);
		ep->req = req;
		notify_transfer_request_complete(ep, status);
	}
}

static u8 is_setup_transferred(struct f_usb20hdc_udc *priv)
{
	return hdc_read_bits(priv->reg_base, HDC_DEVS, BIT_SETUP_INT, 1) ||
		hdc_read_bits(priv->reg_base, HDC_DEVS, BIT_USBRSTB_INT, 1) ||
		hdc_read_bits(priv->reg_base, HDC_DEVS, BIT_USBRSTE_INT, 1) ||
		!!hdc_read_bits(priv->reg_base, HDC_DEVS, BIT_BUSRESET, 1);
}

static u8 set_in_transfer_pio(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 ep_ch = ep->ep_ch;
	u32 *transfer_data;
	u8 index = ep_ch == EP0 ? 0 :
		hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
				     BIT_APPPTR, LEN_APPPTR);
	u32 bytes;

	if (unlikely((ep_ch != EP0) && !ep->ep.desc))
		return 0;

	/* check new SETUP transfer */
	if ((ep_ch == EP0) && (is_setup_transferred(priv))) {
		dev_err(priv->gadget.dev.parent,
			"%s():new SETUP transfer is occurred\n", __func__);
		return 0;
	}

	/* check transfer data setup */
	if (ep_ch == EP0 ?
	    hdc_read_epctrl_bits(base, HDC_EPCTRL(0), BIT_FULLI, 1) :
	    hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch), BIT_FULLO_FULL, 1)) {
		dev_err(priv->gadget.dev.parent,
			"%s():endpoint %u buffer  is full\n",
			__func__, ep_ch);
		return 0;
	}

	/* check NULL packet IN transfer for last packet of transaction */
	if ((req->req.length) && !(req->req.length % ep->ep.maxpacket) &&
	    (req->req.zero))
		ep->null_packet = 1;

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(req->req.buf + req->req.actual);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (req->req.length - req->req.actual) < ep->ep.maxpacket ?
		req->req.length - req->req.actual : ep->ep.maxpacket;

	/* update actual byte */
	req->req.actual = bytes;

	/* set request execute */
	req->request_execute = 1;

	/* check buffer write bytes */
	if (bytes)
		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[index], transfer_data,
				bytes);

	/* notify buffer write bytes */
	hdc_write_bits(base, HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
		       BIT_APPCNT, LEN_APPCNT, bytes);

	if (ep_ch == EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv)) {
			/* enable IN transfer & readyi interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INTEN, 1, 1);
		}
	} else {
		/* check IN transfer end timing and endpoint buffer count */
		if ((ep->in_trans_end_timing) && (ep->buffers >= 2))
			/*
			 * clear empty interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_RDYO_EMPTY_INT_CLR, 1, 1);

		/* check DMA transfer usable */
		if (ep->dma_ch != -1)
			/*
			 * clear ready interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_RDYI_RDY_INT_CLR, 1, 1);

		/* enable IN transfer & ready interrupt */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFWR, 1, 1);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYI_RDY_INTEN,
			       1, 1);
	}

	return 1;
}

static u8 set_out_transfer_pio(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 ep_ch = ep->ep_ch;

	if (unlikely((ep_ch != EP0) && (!ep->ep.desc)))
		return 0;

	/* check new SETUP transfer */
	if ((ep_ch == EP0) && (is_setup_transferred(priv))) {
		dev_err(priv->dev, "%s() new SETUP transfer is occurred\n",
			__func__);
		return 0;
	}

	/* set request execute */
	req->request_execute = 1;

	if (ep_ch == EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv)) {
			/* enable readyo interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYO_EMPTY_INTEN, 1, 1);
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFRD, 1, 1);
		}
	} else {
		/* check DMA transfer usable */
		if (ep->dma_ch != -1) {
			/*
			 * clear ready interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_NACKRESP,
				       1, 1);
			if (hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
						 BIT_EMPTYO_EMPTY, 1))
				hdc_write_bits(base, HDC_EPCMD(ep_ch),
					       BIT_RDYI_RDY_INT_CLR, 1, 1);
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_NACKRESP,
				       1, 0);
		}

		/* enable ready interrupt */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYI_RDY_INTEN,
			       1, 1);
	}

	return 1;
}

static void abort_in_transfer_pio(struct f_usb20hdc_udc_ep *ep, u8 init)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	/* disable endpoint interrupt */
	if (ep_ch == EP0) {
		hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYI_RDY_INTEN,
			       1, 0);
	} else {
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYI_RDY_INTEN,
			       1, 0);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_RDYO_EMPTY_INTEN,
			       1, 0);
	}

	/* clear NULL packet transfer */
	ep->null_packet = 0;

	if (init)
		initialize_endpoint(ep, 1, 0, 0);
}

static void abort_out_transfer_pio(struct f_usb20hdc_udc_ep *ep, u8 init)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	/* disable endpoint interrupt */
	ep_ch == EP0 ?
		hdc_write_bits(base, HDC_EPCMD(EP0),
			       BIT_RDYO_EMPTY_INTEN, 1, 0) :
		hdc_write_bits(base, HDC_EPCMD(ep_ch),
			       BIT_RDYI_RDY_INTEN, 1, 0);

	if (init)
		initialize_endpoint(ep, 1, 0, 0);
}

static u8 end_in_transfer_pio(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 ch = ep->ep_ch;
	u32 *transfer_data;
	u8 i = ch == EP0 ? 0 :
		hdc_read_epctrl_bits(base, HDC_EPCTRL(ch),
				     BIT_APPPTR, LEN_APPPTR);
	u32 bytes;

	/* check empty wait */
	if ((ch != EP0) &&
	    (hdc_read_epctrl_bits(base, HDC_EPCTRL(ch), BIT_EMPTY_INTEN, 1)
	    )) {
		/* complete request */
		abort_in_transfer_pio(ep, 0);
		return 1;
	}

	/* check transfer remain byte */
	if (!(req->req.length - req->req.actual)) {
		/* check NULL packet IN transfer for last packet transaction */
		if (ep->null_packet) {
			if (ch == EP0) {
				/* check new SETUP transfer */
				if (is_setup_transferred(priv)) {
					abort_in_transfer_pio(ep, 0);
					return 0;
				}
				/* notify buffer write bytes */
				hdc_write_bits(base, HDC_EPCOUNT(0),
					       BIT_APPCNT, LEN_APPCNT, 0);

				/* enable IN transfer */
				hdc_write_bits(base, HDC_EPCMD(EP0),
					       BIT_BUFWR, 1, 1);
			} else {
				/* notify buffer write bytes */
				hdc_write_bits(base,
					       HDC_EPCOUNT(ch * EP_BUF_CNT + i),
					       BIT_APPCNT, LEN_APPCNT, 0);

				/* enable IN transfer */
				hdc_write_bits(base, HDC_EPCMD(ch),
					       BIT_BUFWR, 1, 1);
			}
			ep->null_packet = 0;
			return 0;
		}

		/* check IN transfer end timing and ep buffer count */
		if (ep->in_trans_end_timing && (ep->buffers >= 2)) {
			/* enable empty interrupt */
			hdc_write_bits(base, HDC_EPCMD(ch),
				       BIT_RDYO_EMPTY_INTEN, 1, 1);
			return 0;
		}

		/* complete request */
		abort_in_transfer_pio(ep, 0);
		return 1;
	}

	/* check transfer data setup */
	if (ch == EP0 ?
		hdc_read_epctrl_bits(base, HDC_EPCTRL(0), BIT_FULLI, 1) :
		hdc_read_epctrl_bits(base, HDC_EPCTRL(ch), BIT_FULLO_FULL,
				     1)) {
		/* abort IN transfer */
		abort_in_transfer_pio(ep, ch == EP0 ? 0 : 1);
		dev_err(priv->gadget.dev.parent,
			"%s():endpoint %u buffer  is full\n", __func__, ch);
		return 0;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(req->req.buf + req->req.actual);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (req->req.length - req->req.actual) < ep->ep.maxpacket ?
			req->req.length - req->req.actual : ep->ep.maxpacket;

	/* update actual bytes */
	req->req.actual += bytes;

	/* check buffer write bytes */
	if (bytes)
		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[i], transfer_data, bytes);

	/* notify buffer write bytes */
	hdc_write_bits(base, HDC_EPCOUNT(ch * EP_BUF_CNT + i),
		       BIT_APPCNT, LEN_APPCNT, bytes);

	if (ch == EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable IN transfer */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
	} else {
		/* enable IN transfer */
		hdc_write_bits(base, HDC_EPCMD(ch), BIT_BUFWR, 1, 1);
	}

	return 0;
}

static u8 end_out_transfer_pio(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 ep_ch = ep->ep_ch;
	u32 *transfer_data;
	u32 bytes;
	u8 index = ep_ch == EP0 ? 1 :
		hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
				     BIT_APPPTR, LEN_APPPTR);

	/* check transfer data read enable */
	if ((ep_ch != EP0) && hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
						   BIT_EMPTYO_EMPTY, 1)) {
		/* abort OUT transfer */
		abort_out_transfer_pio(ep, 1);
		dev_err(priv->gadget.dev.parent,
			"%s() endpoint %u is empty.%d\n", __func__, ep_ch,
			hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
					     BIT_EPCTRL_NACK_INTEN, 1));

		return 0;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(req->req.buf + req->req.actual);
	prefetch(transfer_data);

	/* get OUT transfer byte */
	bytes = hdc_read_bits(base, HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
			      BIT_PHYCNT, LEN_PHYCNT);
	if (req->req.length < (req->req.actual + bytes))
		bytes = req->req.length - req->req.actual;

	/* update actual bytes */
	req->req.actual += bytes;

	/* check buffer read bytes */
	if (bytes)
		/* read OUT transfer data from buffer */
		hdc_read_epbuf(base, ep->buf_offset[index], transfer_data,
			       bytes);

	if (ep_ch != EP0)
		/* enable next OUT transfer */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFRD, 1, 1);

	/* check transfer request complete */
	if ((req->req.length <= req->req.actual) ||
	    (bytes % ep->ep.maxpacket) || !bytes) {
		/* complete request */
		abort_out_transfer_pio(ep, 0);
		return 1;
	}

	if (ep_ch == EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable next OUT transfer */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFRD, 1, 1);
	}

	return 0;
}

static u8 set_in_transfer(struct f_usb20hdc_udc_ep *ep)
{
	static u8 (* const set_in_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		set_in_transfer_pio,
		set_in_transfer_dma,
	};
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 index = 1, result;

	/* check NULL packet transfer */
	if (!req->req.length)
		return set_in_transfer_pio(ep);

	/* check DMA transfer usable */
	if (!is_dma_transfer_usable(ep)) {
		if (!is_pio_transfer_auto_change_usable(ep))
			return 0;
		index = 0;
	}

	/* get IN transfer process result */
	result = set_in_transfer_function[index](ep);

	/* check PIO transfer auto change usable */
	if (!result && (index == 1) &&
	    is_pio_transfer_auto_change_usable(ep))
		return set_in_transfer_pio(ep);

	return result;
}

static u8 set_out_transfer(struct f_usb20hdc_udc_ep *ep)
{
	static u8 (* const set_out_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		set_out_transfer_pio,
		set_out_transfer_dma,
	};
	struct f_usb20hdc_udc_req *req = ep->req;
	u8 index = 1;
	u8 result;

	/* check NULL packet transfer */
	if (!req->req.length)
		return set_out_transfer_pio(ep);

	/* check DMA transfer usable */
	if (!is_dma_transfer_usable(ep)) {
		if (!is_pio_transfer_auto_change_usable(ep))
			return 0;
		index = 0;
	}

	/* get OUT transfer process result */
	result = set_out_transfer_function[index](ep);

	/* check PIO transfer auto change usable */
	if (!result && (index == 1) &&
	    is_pio_transfer_auto_change_usable(ep))
		return set_out_transfer_pio(ep);

	return result;
}

static void abort_in_transfer(struct f_usb20hdc_udc_ep *ep, u8 initialize)
{
	static void (* const abort_in_transfer_function[])(
					struct f_usb20hdc_udc_ep *, u8) = {
		abort_in_transfer_pio,
		abort_in_transfer_dma,
	};
	return abort_in_transfer_function[!!ep->dma_transfer](ep, initialize);
}

static void abort_out_transfer(struct f_usb20hdc_udc_ep *ep, u8 initialize)
{
	static void (* const abort_out_transfer_function[])(
					struct f_usb20hdc_udc_ep *, u8) = {
		abort_out_transfer_pio,
		abort_out_transfer_dma,
	};
	return abort_out_transfer_function[!!ep->dma_transfer](ep, initialize);
}

static u8 end_in_transfer(struct f_usb20hdc_udc_ep *ep)
{
	static u8 (* const end_in_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		end_in_transfer_pio,
		end_in_transfer_dma,
	};
	return end_in_transfer_function[!!ep->dma_transfer](ep);
}

static u8 end_out_transfer(struct f_usb20hdc_udc_ep *ep)
{
	static u8 (* const end_out_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		end_out_transfer_pio,
		end_out_transfer_dma,
	};
	return end_out_transfer_function[!!ep->dma_transfer](ep);
}

static void halt_transfer(struct f_usb20hdc_udc_ep *ep, u8 halt, u8 force_halt)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_req *req;
	u8 ep_ch = ep->ep_ch;

	/* check isochronous endpoint */
	if (ep->transfer_type == USB_ENDPOINT_XFER_ISOC)
		return;

	if (halt) {
		if (ep_ch == EP0) {
			/* check new SETUP transfer */
			if (is_setup_transferred(priv))
				return;
		}

		/* set transfer halt */
		ep->halt = 1;
		if (force_halt)
			ep->force_halt = 1;

		/* check endpoint x halt clear */
		if (!hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch),
					  BIT_STALL, 1))
			/* halt endpoint x */
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_STALL_SET, 1, 1);
	} else {
		/* check force halt */
		if (ep->force_halt) {
			/* always clear endpoint x data toggle bit */
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_TOGGLE_CLR, 1, 1);
			return;
		}

		/* check endpoint x halt */
		if (hdc_read_epctrl_bits(base, HDC_EPCTRL(ep_ch), BIT_STALL, 1))
			/* clear endpoint x halt */
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_STALL_CLR, 1, 1);

		/* always clear endpoint x data toggle bit */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_TOGGLE_CLR, 1, 1);

		/* clear transfer halt */
		ep->halt = 0;

		/* check next queue empty */
		if (list_empty(&ep->queue))
			return;

		/* get next request */
		req = list_entry(ep->queue.next,
				 struct f_usb20hdc_udc_req, queue);

		/* check the got next request is under current execution */
		if (req->request_execute)
			return;

		/* save request */
		ep->req = req;

		/* set endpoint x transfer request */
		if (!(ep->transfer_direction ? set_in_transfer(ep) :
		    set_out_transfer(ep))) {
			dev_err(priv->dev, "%s():%s of endpoint %u is failed\n",
				__func__, ep->transfer_direction ?
				"set_in_transfer()" : "set_out_transfer()",
				ep_ch);
			dequeue_all_transfer_request(ep, -EL2HLT);
		}
	}
}

static u8 respond_request(struct f_usb20hdc_udc_ep *ep,
			  struct usb_ctrlrequest ctrlreq,
			  int *setup_ret)
{
	int result;
	struct f_usb20hdc_udc *priv = ep->priv;

	/* check SETUP transfer callback to gadget driver */
	if (!priv->gadget_driver || !priv->gadget_driver->setup) {
		dev_err(priv->dev, "%s() no gadget_driver\n", __func__);
		return 0;
	}

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&priv->lock);
	result = priv->gadget_driver->setup(&priv->gadget, &ctrlreq);
	spin_lock(&priv->lock);
	if (setup_ret)
		*setup_ret = result;

	if (result < 0) {
		dev_err(priv->dev, "%s():%s of setup() is failed at %d\n",
			__func__, priv->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result == 0) {
		/* delay NULL packet transfer for status stage */
		priv->ctrl_status_delay = 1;
		priv->ctrl_pri_dir = (ctrlreq.bRequestType & USB_DIR_IN) ||
						!ctrlreq.wLength;
	}

	return 1;
}

static u8 respond_standard_request_get_status(struct f_usb20hdc_udc_ep *ep,
					      struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u16 value;
	struct device *dev = priv->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
		    ctrlreq.wValue || ctrlreq.wIndex ||
		    (ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN transfer data */
		value = priv->selfpowered ?
					(1 << USB_DEVICE_SELF_POWERED) : 0;
		value |= hdc_read_bits(base, HDC_DEVC, BIT_RMTWKUP, 1) ?
					(1 << USB_DEVICE_REMOTE_WAKEUP) : 0x0;

		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[0], (u32 *)&value, 2);

		/* notify buffer write bytes */
		hdc_write_bits(base, HDC_EPCOUNT(0), BIT_APPCNT, LEN_APPCNT, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable IN transfer & readyi interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
		break;
	case USB_RECIP_INTERFACE:
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
		    ctrlreq.wValue || (ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN transfer data */
		value = 0x0000;

		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[0], (u32 *)&value, 2);

		/* notify buffer write bytes */
		hdc_write_bits(base, HDC_EPCOUNT(0), BIT_APPCNT, LEN_APPCNT, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable IN transfer & readyi interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
		break;
	case USB_RECIP_ENDPOINT:
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
		    ctrlreq.wValue || (ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN transfer data */
		value = hdc_read_epctrl_bits(base,
					     HDC_EPCTRL(ctrlreq.wIndex & 0xf),
					     BIT_STALL, 1) ? 0x0001 : 0x0000;

		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[0], (u32 *)&value, 2);

		/* notify buffer write bytes */
		hdc_write_bits(base, HDC_EPCOUNT(0), BIT_APPCNT, LEN_APPCNT, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable IN transfer & readyi interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_clear_feature(struct f_usb20hdc_udc_ep *ep,
						 struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
		    (ctrlreq.wValue != USB_DEVICE_REMOTE_WAKEUP) ||
		    (ctrlreq.wIndex & 0xff) || ctrlreq.wLength)
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* disable remote wakeup */
		hdc_write_bits(priv->reg_base, HDC_DEVC, BIT_RMTWKUP, 1, 0);
		break;
	case USB_RECIP_INTERFACE:
		if ((ctrlreq.bRequestType & USB_DIR_IN) || ctrlreq.wLength)
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		return respond_request(ep, ctrlreq, NULL);

	case USB_RECIP_ENDPOINT:
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
		    (ctrlreq.wValue != USB_ENDPOINT_HALT) || ctrlreq.wLength)
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* clear transfer halt */
		halt_transfer(&priv->ep[ctrlreq.wIndex & 0xf], 0, 0);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_feature(struct f_usb20hdc_udc_ep *ep,
					       struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct device *dev = priv->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
		    (ctrlreq.wIndex & 0xff) || ctrlreq.wLength)
			return 0;

		/* check request parameter of wValue */
		switch (ctrlreq.wValue) {
		case USB_DEVICE_REMOTE_WAKEUP:
			/* check device state */
			switch (priv->device_state) {
			case USB_STATE_ADDRESS:
			case USB_STATE_CONFIGURED:
				break;
			default:
				dev_dbg(dev, "%s (%d): abnormal device_state\n",
					__func__, __LINE__);
				return 0;
			}

			/* enable remote wakeup */
			hdc_write_bits(base, HDC_DEVC, BIT_RMTWKUP, 1, 1);
			break;
		case USB_DEVICE_TEST_MODE:
			/* check device state */
			switch (priv->device_state) {
			case USB_STATE_DEFAULT:
			case USB_STATE_ADDRESS:
			case USB_STATE_CONFIGURED:
				break;
			default:
				dev_dbg(dev, "%s (%d): abnormal device_state\n",
					__func__, __LINE__);
				return 0;
			}

			/* check test selector */
			priv->test_selector = (ctrlreq.wIndex >> 8) & 0xff;
			break;
		default:
			return 0;
		}
		break;
	case USB_RECIP_INTERFACE:
		if ((ctrlreq.bRequestType & USB_DIR_IN) || ctrlreq.wLength)
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		return respond_request(ep, ctrlreq, NULL);

	case USB_RECIP_ENDPOINT:
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
		    (ctrlreq.wValue != USB_ENDPOINT_HALT) || ctrlreq.wLength)
			return 0;

		/* check device state */
		switch (priv->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state\n",
				__func__, __LINE__);
			return 0;
		}

		/* halt transfer */
		halt_transfer(&priv->ep[ctrlreq.wIndex & 0xf], 1, 0);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_address(struct f_usb20hdc_udc_ep *ep,
					       struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
	    (ctrlreq.bRequestType & USB_DIR_IN) ||
	    ctrlreq.wIndex || ctrlreq.wLength)
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_DEFAULT:
	case USB_STATE_ADDRESS:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	/* set address value */
	hdc_write_bits(base, HDC_FADDR, BIT_FUNC_ADDR, LEN_FUNC_ADDR,
		       ctrlreq.wValue & 0xff);

	/* change device state */
	set_device_state(priv, USB_STATE_ADDRESS);

	return 1;
}

static u8 respond_standard_request_get_descriptor(struct f_usb20hdc_udc_ep *ep,
						  struct usb_ctrlrequest
						  ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;
	bool b1, b2, b3, b4;

	b1 = ((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE);
	b2 = ((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE);
	b3 = (ctrlreq.bRequestType & USB_DIR_IN);
	b4 = (((ctrlreq.wValue >> 8) != USB_DT_STRING) && ctrlreq.wIndex);
	if ((b1 && b2) || !b3 || b4)
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_DEFAULT:
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return respond_request(ep, ctrlreq, NULL);
}

static u8 respond_standard_request_set_descriptor(struct f_usb20hdc_udc_ep *ep,
						  struct usb_ctrlrequest
						  ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;
	bool b1, b2, b3, b4;

	b1 = ((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE);
	b2 = ((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE);
	b3 = (ctrlreq.bRequestType & USB_DIR_IN);
	b4 = (((ctrlreq.wValue >> 8) != USB_DT_STRING) && ctrlreq.wIndex);
	if ((b1 && b2) || b3 || b4)
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return respond_request(ep, ctrlreq, NULL);
}

static u8 respond_standard_request_get_configuration(struct f_usb20hdc_udc_ep
						     *ep,
						     struct usb_ctrlrequest
						     ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 value;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
	    (!(ctrlreq.bRequestType & USB_DIR_IN)) ||
	    ctrlreq.wValue || ctrlreq.wIndex || (ctrlreq.wLength != 1))
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_ADDRESS:
		/* create IN transfer data */
		value = 0;

		/* write IN transfer data to buffer */
		hdc_write_epbuf(base, ep->buf_offset[0], (u32 *)&value, 1);

		/* notify buffer write bytes */
		hdc_write_bits(base, HDC_EPCOUNT(0), BIT_APPCNT, LEN_APPCNT, 1);

		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* enable IN transfer & readyi interrupt */
			hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
		break;
	case USB_STATE_CONFIGURED:
		return respond_request(ep, ctrlreq, NULL);
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_configuration(struct f_usb20hdc_udc_ep
						     *ep,
						     struct usb_ctrlrequest
						     ctrlreq)
{
	u32 i;
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	u8 configure_value = ctrlreq.wValue & 0xff;
	u8 configure_value_last = priv->configure_value_last;
	int result;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
	    (ctrlreq.bRequestType & USB_DIR_IN) ||
	    ctrlreq.wIndex || ctrlreq.wLength)
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	/* check configure value */
	if (configure_value) {
		/* check configure value change */
		if ((configure_value != priv->configure_value_last) &&
		    (priv->configure_value_last)) {
			priv->configure_value_last = configure_value;
			for (i = EP1; i < HDC_UDC_MAX_EP; i++)
				/* abort transfer */
				priv->ep[i].transfer_direction ?
					abort_in_transfer(&priv->ep[i], 1) :
					abort_out_transfer(&priv->ep[i], 1);
		}

		/* change device state */
		set_device_state(priv, USB_STATE_CONFIGURED);
	} else {
		/* check configure value change */
		if (configure_value != priv->configure_value_last) {
			priv->configure_value_last = configure_value;
			for (i = EP1; i < HDC_UDC_MAX_EP; i++)
				/* abort transfer */
				priv->ep[i].transfer_direction ?
					abort_in_transfer(&priv->ep[i], 1) :
					abort_out_transfer(&priv->ep[i], 1);
		}

		/* change device state */
		set_device_state(priv, USB_STATE_ADDRESS);
	}

	/* dequeue all previous transfer request */
	for (i = EP0; i < HDC_UDC_MAX_EP; i++)
		dequeue_all_transfer_request(&priv->ep[i],
					     -ECONNABORTED);

	/* set configuration value */
	hdc_write_bits(base, HDC_FADDR, BIT_DEV_CONFIGURED, 1,
		       configure_value ? 1 : 0);

	/* check SETUP transfer callback to gadget driver */
	if (!priv->gadget_driver || !priv->gadget_driver->setup)
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&priv->lock);
	result = priv->gadget_driver->setup(&priv->gadget, &ctrlreq);
	spin_lock(&priv->lock);
	if (result < 0) {
		priv->configure_value_last = configure_value_last;
		set_device_state(priv, configure_value_last ?
				 USB_STATE_CONFIGURED :
				 USB_STATE_ADDRESS);
		hdc_write_bits(base, HDC_FADDR, BIT_DEV_CONFIGURED, 1,
			       !!configure_value_last);
		dev_err(dev, "%s():%s of setup() is failed at %d\n",
			__func__,
			priv->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		priv->ctrl_status_delay = 1;
		priv->ctrl_pri_dir = (ctrlreq.bRequestType & USB_DIR_IN) ||
						!ctrlreq.wLength;
	}

	return 1;
}

static u8 respond_standard_request_get_interface(struct f_usb20hdc_udc_ep *ep,
						 struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE) ||
	    (!(ctrlreq.bRequestType & USB_DIR_IN)) ||
	    ctrlreq.wValue || (ctrlreq.wLength != 1))
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return respond_request(ep, ctrlreq, NULL);
}

static u8 respond_standard_request_set_interface(struct f_usb20hdc_udc_ep *ep,
						 struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE) ||
	    (ctrlreq.bRequestType & USB_DIR_IN) ||
	    ctrlreq.wLength)
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s %d abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return respond_request(ep, ctrlreq, NULL);
}

static u8 respond_standard_request_sync_frame(struct f_usb20hdc_udc_ep *ep,
					      struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct device *dev = priv->dev;

	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_ENDPOINT) ||
	    (!(ctrlreq.bRequestType & USB_DIR_IN)) ||
	    ctrlreq.wValue || (ctrlreq.wLength != 2))
		return 0;

	/* check device state */
	switch (priv->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state\n",
			__func__, __LINE__);
		return 0;
	}

	return respond_request(ep, ctrlreq, NULL);
}

static u8 respond_standard_request_undefined(struct f_usb20hdc_udc_ep *ep,
					     struct usb_ctrlrequest ctrlreq)
{
	/* always error return */
	dev_err(ep->priv->dev, "%s() %d\n",
		__func__, __LINE__);
	return 0;
}

static void enable_host_connect(struct f_usb20hdc_udc *priv, u8 enable)
{
	u32 ch;
	struct f_usb20hdc_udc_ep *ep;
	void __iomem *base = priv->reg_base;

	if (enable) {
		/* initialize F_USB20HDC controller */
		initialize_udc_controller(priv, 0);

		/* enable device mode */
		hdc_write_bits(base, HDC_MODE, BIT_HOST_EN, 1, 0);
		hdc_write_bits(base, HDC_MODE, BIT_DEV_EN, 1, 1);
		hdc_write_bits(base, HDC_MODE, BIT_DEV_INT_MODE, 1, 1);
		hdc_write_bits(base, HDC_MODE, BIT_DEV_ADDR_LOAD_MODE, 1, 1);

		/* enable interrupt factor */
		hdc_write_bits(base, HDC_INTEN, BIT_DEV_INTEN, 1, 1);
		hdc_write_bits(base, HDC_INTEN, BIT_PHY_ERR_INTEN, 1, 1);
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDE_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDB_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDE_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDB_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SETUP_INTEN, 1, 1);
		hdc_write_bits(base, HDC_DEVC, BIT_USBRSTE_INTEN, 1, 1);
		hdc_write_bits(base, HDC_DEVC, BIT_USBRSTB_INTEN, 1, 1);

		/* change device state */
		set_device_state(priv, USB_STATE_POWERED);

		/* initialize endpoint configure data */
		initialize_endpoint_configure(priv);

		/* pull-up D+ terminal */
		dev_dbg(priv->dev, "D+ terminal is pull-up\n");
		hdc_set_dplus_pullup(base, 1);
		dev_dbg(priv->dev, "DP:%d, DM:%d",
			hdc_get_linestate(base) & 0x00000001,
			hdc_get_linestate(base) & 0x00000002);
	} else {
		/* disable interrupt factor */
		hdc_write_bits(base, HDC_INTEN, BIT_DEV_INTEN, 1, 0);
		hdc_write_bits(base, HDC_INTEN, BIT_PHY_ERR_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDE_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SUSPENDB_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_SETUP_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_USBRSTE_INTEN, 1, 0);
		hdc_write_bits(base, HDC_DEVC, BIT_USBRSTB_INTEN, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSFALL, BIT_VBUS_VLD_FEN, 1, 0);
		hdc_write_bits(base, HDC_OTGSTSRISE, BIT_VBUS_VLD_REN, 1, 0);

		/* clear interrupt factor */
		hdc_write_bits(base, HDC_INTS, BIT_PHY_ERR_INT, 1, 0);
		hdc_write_bits(base, HDC_INTS, BIT_CMD_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDE_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDB_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_SOF_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_SETUP_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_USBRSTE_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_USBRSTB_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_STATUS_OK_INT, 1, 0);
		hdc_write_bits(base, HDC_DEVS, BIT_STATUS_NG_INT, 1, 0);

		/* change device state */
		set_device_state(priv, USB_STATE_NOTATTACHED);

		/* pull-down D+ terminal */
		dev_dbg(priv->dev, "D+ terminal is pull-down\n");
		hdc_set_dplus_pullup(base, 0);

		/* abort previous transfer */
		abort_in_transfer(&priv->ep[EP0], 0);
		abort_out_transfer(&priv->ep[EP0], 0);
		enable_endpoint(&priv->ep[EP0], 0);
		for (ch = EP1; ch < HDC_UDC_MAX_EP; ch++) {
			ep = &priv->ep[ch];
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
			if ((ep->dma_ch != -1) && ep->dma_transfer) {
				spin_unlock(&priv->lock);
				hdmac_stop_nowait(priv->dma_data[ep->dma_ch]
						  .hdmac_ch);
				spin_lock(&priv->lock);
			}
#endif
			ep->transfer_direction ?
				abort_in_transfer(ep, 0) :
				abort_out_transfer(ep, 0);
			enable_endpoint(ep, 0);
		}

		/* reset USB OTG controller */
		hdc_core_soft_reset(priv->f_otg);

		/* initial gadget mode */
		initialize_udc_controller(priv, 1);
		initialize_endpoint_configure(priv);

		initialize_udc_controller(priv, 0);

		/* dequeue all previous transfer request */
		for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++)
			dequeue_all_transfer_request(
				&priv->ep[ch], -ESHUTDOWN);

		/* initialize endpoint list data */
		INIT_LIST_HEAD(&priv->gadget.ep0->ep_list);
		INIT_LIST_HEAD(&priv->gadget.ep_list);
		for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++) {
			list_add_tail(&priv->ep[ch].ep.ep_list, ch == EP0 ?
				      &priv->gadget.ep0->ep_list :
				      &priv->gadget.ep_list);
			INIT_LIST_HEAD(&priv->ep[ch].queue);
		}

		/* initialize F_USB20HDC UDC device driver structure data */
		priv->gadget.speed = USB_SPEED_UNKNOWN;
		priv->configure_value_last = 0;
		priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		priv->ctrl_pri_dir = 1;
	}
}

static void enable_communicate(struct f_usb20hdc_udc *priv, u8 enable)
{
	void *base = priv->reg_base;
	struct device *dev = priv->dev;

#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	if (!enable)
		/* delete halt transfer error recovery */
		del_timer_sync(&priv->halt_transfer_error_recovery_timer);
#endif
	if (enable) {
		/* get current bus connect */
		priv->bus_connect = is_bus_connected(base);

		/* check bus connect */
		if (priv->bus_connect) {
			dev_dbg(dev, "%s() bus connected\n", __func__);
			/* set bus disconnect detect */
			hdc_set_connect_detect(base, 0);

			/* enable host connect */
			enable_host_connect(priv, 1);
		} else {
			dev_dbg(dev, "%s() bus isn't connected\n", __func__);
			/* set bus connect detect */
			hdc_set_connect_detect(base, 1);
		}
	} else {
		/* disable host connect */
		enable_host_connect(priv, 0);
	}
}

static void on_detect_bus_connect(u32 data)
{
	struct f_usb20hdc_udc *priv = (struct f_usb20hdc_udc *)data;
	void *base = priv->reg_base;
	struct device *dev = priv->dev;

	/* get current bus connect */
	priv->bus_connect = is_bus_connected(base);

	/* check bus connect */
	if (priv->bus_connect) {
		dev_dbg(dev, "%s() connected to host\n", __func__);
		/* set bus disconnect detect */
		hdc_set_connect_detect(base, 0);

		/* check F_USB20HDC UDC device driver register */
		if (priv->device_add)
			enable_host_connect(priv, 1);
	} else {
		dev_dbg(dev, "%s() no connection to host\n", __func__);
		/* check F_USB20HDC UDC device driver register */
		if (priv->device_add) {
			enable_host_connect(priv, 0);
			hdc_set_connect_detect(base, 1);

			/* notify disconnect to gadget driver */
			if (priv->gadget_driver &&
			    priv->gadget_driver->disconnect) {
				spin_unlock(&priv->lock);
				priv->gadget_driver->disconnect(&priv->gadget);
				spin_lock(&priv->lock);
			}
		} else {
			/* set bus connect detect */
			hdc_set_connect_detect(base, 1);
		}
	}
}

static void on_begin_bus_reset(struct f_usb20hdc_udc *priv)
{
	u32 ch;
	struct f_usb20hdc_udc_ep *ep;

	dev_dbg(priv->dev, "%s() is started\n", __func__);
	/* abort previous transfer,initialize and disable all endpoint */
	abort_in_transfer(&priv->ep[EP0], 0);
	abort_out_transfer(&priv->ep[EP0], 0);
	enable_endpoint(&priv->ep[EP0], 0);
	for (ch = EP1; ch < HDC_UDC_MAX_EP; ch++) {
		ep = &priv->ep[ch];
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		if ((ep->dma_ch != -1) && ep->dma_transfer) {
			spin_unlock(&priv->lock);
			hdmac_stop_nowait(priv->dma_data[ep->dma_ch].hdmac_ch);
			spin_lock(&priv->lock);
		}
#endif
		ep->transfer_direction ?
			abort_in_transfer(ep, 0) : abort_out_transfer(ep, 0);
		enable_endpoint(ep, 0);
	}

	/* re-enable endpoint 0 */
	enable_endpoint(&priv->ep[EP0], 1);

	/* dequeue all previous transfer request */
	for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++)
		dequeue_all_transfer_request(&priv->ep[ch], -ECONNABORTED);

	/* initialize endpoint list data */
	INIT_LIST_HEAD(&priv->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&priv->gadget.ep_list);
	for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++) {
		list_add_tail(&priv->ep[ch].ep.ep_list, ch == EP0 ?
			      &priv->gadget.ep0->ep_list :
			      &priv->gadget.ep_list);
		INIT_LIST_HEAD(&priv->ep[ch].queue);
	}

	if (priv->f_otg->variant != FHDC_VARIANT_LAP) {
		/* check configured */
		if (priv->configure_value_last) {
			/* notify dummy disconnect to gadget driver */
			if (priv->gadget_driver && priv->gadget_driver->disconnect) {
				spin_unlock(&priv->lock);
				priv->gadget_driver->disconnect(&priv->gadget);
				spin_lock(&priv->lock);
			}
		}
	}

	/* initialize F_USB20HDC UDC device driver structure data */
	priv->gadget.speed = USB_SPEED_UNKNOWN;
	priv->configure_value_last = 0;
	priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	priv->ctrl_pri_dir = 1;
	dev_dbg(priv->dev, "%s() is ended\n", __func__);
}

static void on_end_bus_reset(struct f_usb20hdc_udc *priv)
{
	dev_dbg(priv->dev, "%s() is started\n", __func__);
	/* set current bus speed */
	set_bus_speed(priv);

	hdc_write_bits(priv->reg_base, HDC_DEVS, BIT_SUSPENDE_INT, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_DEVS, BIT_SUSPENDB_INT, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_DEVC, BIT_SUSPENDE_INTEN, 1, 1);
	hdc_write_bits(priv->reg_base, HDC_DEVC, BIT_SUSPENDB_INTEN, 1, 1);

	/* change device state */
	set_device_state(priv, USB_STATE_DEFAULT);
	dev_dbg(priv->dev, "%s() is ended\n", __func__);
}

static void on_suspend(struct f_usb20hdc_udc *priv)
{
	if (priv->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(priv->dev, "%s() err\n", __func__);
		return;
	}

	/* change device state */
	set_device_state(priv, USB_STATE_SUSPENDED);

	/* notify suspend to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->suspend) {
		spin_unlock(&priv->lock);
		priv->gadget_driver->suspend(&priv->gadget);
		spin_lock(&priv->lock);
	}
}

static void on_wakeup(struct f_usb20hdc_udc *priv)
{
	if (priv->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(priv->dev, "%s() err\n", __func__);
		return;
	}

	/* change device state */
	set_device_state(priv, priv->device_state_last);

	/* notify resume to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->resume) {
		spin_unlock(&priv->lock);
		priv->gadget_driver->resume(&priv->gadget);
		spin_lock(&priv->lock);
	}
}

static void on_end_transfer_sof(struct f_usb20hdc_udc *priv)
{
}

static void on_end_control_setup_transfer(struct f_usb20hdc_udc_ep *ep)
{
#define	STANDARD_REQUEST_MAXIMUM	13
	static u8 (* const standard_request_respond_function[])(
			struct f_usb20hdc_udc_ep *, struct usb_ctrlrequest) = {
		respond_standard_request_get_status,
		respond_standard_request_clear_feature,
		respond_standard_request_undefined,
		respond_standard_request_set_feature,
		respond_standard_request_undefined,
		respond_standard_request_set_address,
		respond_standard_request_get_descriptor,
		respond_standard_request_set_descriptor,
		respond_standard_request_get_configuration,
		respond_standard_request_set_configuration,
		respond_standard_request_get_interface,
		respond_standard_request_set_interface,
		respond_standard_request_sync_frame,
	};
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;
	struct usb_ctrlrequest ctrlreq;
	u32 bytes;

	/* clear readyi / ready0 interrupt factor */
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYI_RDY_INT_CLR, 1, 1);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYO_EMPTY_INT_CLR, 1, 1);

	/* get SETUP transfer byte */
	bytes = hdc_read_bits(base, HDC_EPCOUNT(1), BIT_PHYCNT, LEN_PHYCNT);

	/* check SETUP transfer byte */
	if (bytes != 8) {
		dev_err(priv->dev, "%s() SETUP transfer byte %u is mismatch\n",
			__func__, (u32)bytes);
		/* check new setup transfer */
		if (!is_setup_transferred(priv))
			/* protocol stall */
			halt_transfer(ep, 1, 0);
		return;
	}

	/* clear status stage delay */
	priv->ctrl_status_delay = 0;

	/* clear transfer halt */
	ep->halt = 0;
	ep->force_halt = 0;
	ep->null_packet = 0;

	/* dequeue all previous control transfer request */
	dequeue_all_transfer_request(ep, -EPROTO);

	/* read SETUP transfer data from buffer */
	hdc_read_epbuf(base, ep->buf_offset[1], (u32 *)&ctrlreq, bytes);

	/* check new setup transfer */
	if (is_setup_transferred(priv))
		return;

	/* enable next OUT transfer */
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFRD, 1, 1);

	/* disable readyi / readyo interrupt */
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYI_RDY_INTEN, 1, 0);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYO_EMPTY_INTEN, 1, 0);

	/* update control transfer stage */
	if (ctrlreq.bRequestType & USB_DIR_IN) {
		priv->ctrl_stage = F_USB20HDC_STAGE_IN_DATA;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is IN data stage\n",
			__func__);
	} else {
		if (ctrlreq.wLength) {
			priv->ctrl_stage = F_USB20HDC_STAGE_OUT_DATA;
			dev_dbg(priv->dev,
				"%s() next control transfer stage is OUT data stage\n",
				__func__);
		} else {
			priv->ctrl_stage = F_USB20HDC_STAGE_IN_STATUS;
			dev_dbg(priv->dev,
				"%s ()next control transfer stage is IN status stage\n",
				__func__);
		}
	}

	/* check request type */
	switch (ctrlreq.bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		/* process standard request respond */
		if ((ctrlreq.bRequest >= STANDARD_REQUEST_MAXIMUM) ||
		    (!standard_request_respond_function[
		    ctrlreq.bRequest](ep, ctrlreq))) {
			if (!is_setup_transferred(priv))
				/* protocol stall */
				halt_transfer(ep, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (priv->ctrl_status_delay)
			return;
		break;
	case USB_TYPE_CLASS:
		/* process class request respond */
		if (!respond_request(ep, ctrlreq, NULL)) {
			if (!is_setup_transferred(priv))
				/* protocol stall */
				halt_transfer(ep, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (priv->ctrl_status_delay)
			return;
		break;
	case USB_TYPE_VENDOR:
		/* process vendor request respond */
		if (!respond_request(ep, ctrlreq, NULL)) {
			if (!is_setup_transferred(priv))
				/* protocol stall */
				halt_transfer(ep, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (priv->ctrl_status_delay)
			return;
		break;
	default:
		if (!is_setup_transferred(priv))
			/* protocol stall */
			halt_transfer(ep, 1, 0);
		break;
	}

	if (is_setup_transferred(priv))
		return;

	/* enable status stage transfer */
	if (ctrlreq.bRequestType & USB_DIR_IN) {
		priv->ctrl_pri_dir = 1;
		hdc_write_bits(base, HDC_EPCMD(EP0),
			       BIT_RDYO_EMPTY_INTEN, 1, 1);
		return;
	}

	priv->ctrl_pri_dir = ctrlreq.wLength ? 0 : 1;
	hdc_write_bits(base, HDC_EPCOUNT(0), BIT_APPCNT, LEN_APPCNT, 0);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_BUFWR, 1, 1);
	hdc_write_bits(base, HDC_EPCMD(EP0), BIT_RDYI_RDY_INTEN, 1, 1);
}

static void on_end_control_in_transfer(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;

	/* process control transfer stage */
	switch (priv->ctrl_stage) {
	case F_USB20HDC_STAGE_IN_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(priv))
			break;

		/* check IN transfer continue */
		if (!end_in_transfer_pio(ep))
			break;

		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_OUT_STATUS;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is OUT status stage\n",
			__func__);
		break;
	case F_USB20HDC_STAGE_OUT_DATA:
	case F_USB20HDC_STAGE_IN_STATUS:
		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is SETUP stage\n",
			__func__);

		/* TEST MODE */
		switch (priv->test_selector) {
		case 1:
			/* set Test_J */
			hdc_write_bits(base, HDC_TESTC, BIT_TEST_J, 1, 1);
			break;
		case 2:
			/* set Test_K */
			hdc_write_bits(base, HDC_TESTC, BIT_TEST_K, 1, 1);
			break;
		case 3:
			/* set Test_SE_NAK */
			hdc_write_bits(base, HDC_TESTC, BIT_TEST_SE0NACK, 1, 1);
			break;
		case 4:
			/* set Test_Packet */
			hdc_write_bits(base, HDC_TESTC, BIT_TEST_P, 1, 1);
			break;
		default:
			break;
		}
		priv->test_selector = 0;
		break;
	case F_USB20HDC_STAGE_OUT_STATUS:
		/* non process */
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* protocol stall */
			halt_transfer(ep, 1, 0);

		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is SETUP stage\n",
			__func__);
		break;
	}
}

static void on_end_control_out_transfer(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	void *base = priv->reg_base;

	/* process control transfer stage */
	switch (priv->ctrl_stage) {
	case F_USB20HDC_STAGE_OUT_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(priv))
			break;

		/* check OUT transfer continue */
		if (!end_out_transfer_pio(ep))
			break;

		/* notify request complete */
		notify_transfer_request_complete(ep, 0);

		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_IN_STATUS;
		if (priv->ctrl_status_delay) {
			priv->ctrl_pri_dir = 1;
			hdc_write_bits(base, HDC_EPCOUNT(0),
				       BIT_APPCNT, LEN_APPCNT, 0);
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_BUFWR, 1, 1);
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INTEN, 1, 1);
		}
		dev_dbg(priv->dev,
			"%s() next control transfer stage is IN status stage\n",
			__func__);
		break;
	case F_USB20HDC_STAGE_IN_DATA:
	case F_USB20HDC_STAGE_OUT_STATUS:
		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is SETUP stage\n",
			__func__);

		/* notify request complete */
		notify_transfer_request_complete(ep, 0);
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(priv))
			/* protocol stall */
			halt_transfer(ep, 1, 0);

		/* update control transfer stage */
		priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(priv->dev,
			"%s() next control transfer stage is SETUP stage\n",
			__func__);
		break;
	}
}

static void on_end_in_transfer(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req;

	/* check IN transfer continue */
	if (!end_in_transfer(ep))
		/* to be continued */
		return;

	/* notify request complete */
	notify_transfer_request_complete(ep, 0);

	/* check next queue empty */
	if (list_empty(&ep->queue))
		return;

	/* get next request */
	req = list_entry(ep->queue.next, struct f_usb20hdc_udc_req, queue);

	/* check the got next request a request under current execution */
	if (req->request_execute)
		return;

	/* save request */
	ep->req = req;

	/* set endpoint x IN trasfer request */
	if (!set_in_transfer(ep)) {
		dev_err(priv->dev, "%s() set_in_transfer() of ep %u is failed\n",
			__func__, ep->ep_ch);
		dequeue_all_transfer_request(ep, -EL2HLT);
	}
}

static void on_end_out_transfer(struct f_usb20hdc_udc_ep *ep)
{
	struct f_usb20hdc_udc *priv = ep->priv;
	struct f_usb20hdc_udc_req *req;

	/* check OUT transfer continue */
	if (!end_out_transfer(ep))
		/* to be continued */
		return;

	/* notify request complete */
	notify_transfer_request_complete(ep, 0);

	/* check next queue empty */
	if (list_empty(&ep->queue))
		return;

	/* get next request */
	req = list_entry(ep->queue.next, struct f_usb20hdc_udc_req, queue);

	/* check the got next request a request under current execution */
	if (req->request_execute)
		return;

	/* save request */
	ep->req = req;

	/* set endpoint x OUT transfer request */
	if (!set_out_transfer(ep)) {
		dev_err(priv->dev, "%s():set_out_transfer() of ep %u is failed\n",
			__func__, ep->ep_ch);
		dequeue_all_transfer_request(ep, -EL2HLT);
	}
}

static void on_halt_transfer(struct f_usb20hdc_udc_ep *ep)
{
#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	struct f_usb20hdc_udc *priv = ep->priv;

	/* check halt transfer error recovery perform need */
	if ((ep->ep_ch != EP0) || ep->halt)
		return;

	dev_err(priv->dev,
		"%s() auto STALL error recovery is pefromed\n", __func__);

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(priv, 1);

	/* disable host connect */
	enable_host_connect(priv, 0);

	/* notify disconnect to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->disconnect) {
		spin_unlock(&priv->lock);
		priv->gadget_driver->disconnect(&priv->gadget);
		spin_lock(&priv->lock);
	}

	/* update halt transfer error recovery timer */
	mod_timer(&priv->halt_transfer_error_recovery_timer,
		  jiffies + msecs_to_jiffies(HDC_UDC_AUTO_STALL_RECOVERY_TIME));
#endif
}

#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
static void on_recovery_halt_transfer_error(unsigned long data)
{
	if (likely(data))
		enable_communicate((struct f_usb20hdc_udc *)data, 1);
}
#endif

static void on_recovery_controller_hungup(struct f_usb20hdc_udc *priv)
{
#if (HDC_UDC_USE_HANGUP_RECOVERY == 1)
	dev_err(priv->dev, "%s() controller hung-up recovery is pefromed\n",
		__func__);

	/* disable host connect */
	enable_host_connect(priv, 0);

	/* notify disconnect to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->disconnect) {
		spin_unlock(&priv->lock);
		priv->gadget_driver->disconnect(&priv->gadget);
		spin_lock(&priv->lock);
	}

	/* check bus connect */
	if (priv->bus_connect)
		/* enable host connect */
		enable_host_connect(priv, 1);
#endif
}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static inline int on_ep_dma_interrupt(struct f_usb20hdc_udc *priv, int ch)
{
	void *base = priv->reg_base;
	struct f_usb20hdc_udc_ep *ep;

	/* clear dma interrupt factor */
	hdc_write_bits(base, HDC_INTS, BIT_DMA_INT(ch), 1, 0);

	ep = &priv->ep[priv->dma_data[ch].ep_ch];
	/* process IN / OUT transfer end */
	if (ep->transfer_direction) {
		ep->req->usb_dma_int_occurred = 1;
		if (ep->req->dmac_int_occurred)
			on_end_in_transfer(ep);
	} else {
		if (hdc_readl(base, HDC_DMATC_R_ONLY(ch)) <
		    hdc_readl(base, HDC_DMATCI(ch)) &&
		    hdc_read_bits(base, HDC_DMAS_R_ONLY(ch),
				  BIT_DMA_SP, 1)) {
			spin_unlock(&priv->lock);
			hdmac_stop_nowait(priv->dma_data[ch].
					  hdmac_ch);
			spin_lock(&priv->lock);
		}
	}

	return IRQ_HANDLED;
}
#endif

static inline int on_ctrl_ep_interrupt(struct f_usb20hdc_udc *priv, int ch)
{
	void *base = priv->reg_base;
	u32 epctrl0, epcmd;

	epctrl0 = hdc_readl(base, HDC_EPCTRL(EP0));

	/* check priority direction */
	if (priv->ctrl_pri_dir) {
		/* priority is given to IN transfer */
		if (VALUE(epctrl0, BIT_RDYI_RDY_INT, 1) &&
		    VALUE(epctrl0, BIT_READY_INTEN, 1)) {
			/* clear readyi interrupt factor */
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INT_CLR, 1, 1);

			/* process control IN transfer end */
			on_end_control_in_transfer(&priv->ep[EP0]);

			return IRQ_HANDLED;
		}

		if (VALUE(epctrl0, BIT_RDYO_EMPTY_INT, 1) &&
		    VALUE(epctrl0, BIT_EMPTY_INTEN, 1)) {
			/* clear readyo interrupt factor */
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYO_EMPTY_INT_CLR, 1, 1);

			/* process control OUT transfer end */
			on_end_control_out_transfer(&priv->ep[EP0]);

			return IRQ_HANDLED;
		}
	} else {
		/* priority is given to OUT transfer */
		if (VALUE(epctrl0, BIT_RDYO_EMPTY_INT, 1) &&
		    VALUE(epctrl0, BIT_EMPTY_INTEN, 1)) {
			/* clear readyo interrupt factor */
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYO_EMPTY_INT_CLR, 1, 1);

			/* process control OUT transfer end */
			on_end_control_out_transfer(&priv->ep[EP0]);

			return IRQ_HANDLED;
		}

		if (VALUE(epctrl0, BIT_RDYI_RDY_INT, 1) &&
		    VALUE(epctrl0, BIT_READY_INTEN, 1)) {
			/* clear readyi interrupt factor */
			hdc_write_bits(base, HDC_EPCMD(EP0),
				       BIT_RDYI_RDY_INT_CLR, 1, 1);

			/* process control IN transfer end */
			on_end_control_in_transfer(&priv->ep[EP0]);

			return IRQ_HANDLED;
		}
	}

	if (VALUE(epctrl0, BIT_STALLED_INT, 1) &&
	    VALUE(epctrl0, BIT_STALL_INTEN, 1)) {
		/* clear stalled interrupt factor */
		hdc_write_bits(base, HDC_EPCMD(EP0),
			       BIT_STALLED_INT_CLR, 1, 1);

		/* process transfer halt */
		on_halt_transfer(&priv->ep[EP0]);

		return IRQ_HANDLED;
	}

	/* clear endpoint 0 interrupt */
	hdc_cmd0_spin(base, HDC_EPCMD(EP0));
	epcmd = hdc_readl_cached(base, HDC_EPCMD(EP0));
	epcmd |= (1 << BIT_RDYO_EMPTY_INT_CLR) | (1 << BIT_RDYI_RDY_INT_CLR) |
		(1 << BIT_PING_INT_CLR) | (1 << BIT_STALLED_INT_CLR) |
		(1 << BIT_NACK_INT_CLR);
	__raw_writel(epcmd, base + HDC_EPCMD(EP0));
	hdc_cmd0_spin(base, HDC_EPCMD(EP0));

	return IRQ_HANDLED;
}

static inline int on_ep_interrupt(struct f_usb20hdc_udc *priv, int ch)
{
	u32 epctrl, epcmd;
	void *base = priv->reg_base;

	epctrl = hdc_readl(base, HDC_EPCTRL(ch));

	if (VALUE(epctrl, BIT_RDYI_RDY_INT, 1) &&
	    VALUE(epctrl, BIT_READY_INTEN, 1)) {
		/* clear ready interrupt factor */
		hdc_write_bits(base, HDC_EPCMD(ch),
			       BIT_RDYI_RDY_INT_CLR, 1, 1);

		/* process IN / OUT transfer end */
		priv->ep[ch].transfer_direction ?
			on_end_in_transfer(&priv->ep[ch]) :
			on_end_out_transfer(&priv->ep[ch]);

		return IRQ_HANDLED;
	}

	if (VALUE(epctrl, BIT_RDYO_EMPTY_INT, 1) &&
	    VALUE(epctrl, BIT_EMPTY_INTEN, 1)) {
		/* clear empty interrupt factor */
		hdc_write_bits(base, HDC_EPCMD(ch),
			       BIT_RDYO_EMPTY_INT_CLR, 1, 1);

		/* process IN transfer end */
		if (priv->ep[ch].transfer_direction)
			on_end_in_transfer(&priv->ep[ch]);

		return IRQ_HANDLED;
	}

	if (VALUE(epctrl, BIT_STALLED_INT, 1) &&
	    VALUE(epctrl, BIT_STALL_INTEN, 1)) {
		/* clear stalled interrupt factor */
		hdc_write_bits(base, HDC_EPCMD(ch),
			       BIT_STALLED_INT_CLR, 1, 1);

		/* process transfer halt */
		on_halt_transfer(&priv->ep[ch]);

		return IRQ_HANDLED;
	}

	/* clear endpoint x interrupt */
	hdc_cmd0_spin(base, HDC_EPCMD(ch));
	epcmd = hdc_readl_cached(base, HDC_EPCMD(ch));
	epcmd |= (1 << BIT_RDYI_RDY_INT_CLR) | (1 << BIT_RDYO_EMPTY_INT_CLR) |
		(1 << BIT_PING_INT_CLR) | (1 << BIT_STALLED_INT_CLR) |
		(1 << BIT_NACK_INT_CLR);
	__raw_writel(epcmd, base + HDC_EPCMD(ch));
	hdc_cmd0_spin(base, HDC_EPCMD(ch));

	return IRQ_HANDLED;
}

static irqreturn_t on_usb_function(int irq, void *dev_id)
{
	u32 ch;
	struct f_usb20hdc_udc *priv = dev_id;
	void *base = priv->reg_base;
	struct device *dev;
	u32 ints, inten, otgstsc, devs, devc;

	if (unlikely(!priv))
		return IRQ_NONE;

	dev = priv->gadget.dev.parent;

	/* check F_USB20HDC controller interrupt request assert */
	if (unlikely(irq != priv->irq)) {
		dev_err(dev, "%s() irq number doesn't match\n", __func__);
		return IRQ_NONE;
	}

	spin_lock(&priv->lock);

	ints = hdc_readl(base, HDC_INTS);
	inten = hdc_readl(base, HDC_INTEN);
	otgstsc = hdc_readl(base, HDC_OTGSTSC);

	/* bus connect interrupt factor */
	if (VALUE(ints, BIT_OTG_INT, 1) && VALUE(inten, BIT_OTG_INTEN, 1) &&
	    VALUE(otgstsc, BIT_VBUS_VLD_C, 1)) {
		/* clear vbus_vld interrupt factor */
		hdc_write_bits(base, HDC_OTGSTSC, BIT_VBUS_VLD_C, 1, 0);

		dev_dbg(dev, "VBUS status change interrupt occurred (vld:%d)\n",
			is_bus_connected(base));

		/* process bus connect detect */
		on_detect_bus_connect((u32)priv);

		goto irq_handled;
	}

	/* check F_USB20HDC controller device mode usage */
	if (!is_device_mode_usage(priv->reg_base))
		goto irq_handled;

	/* PHY hung-up interrupt factor */
	if (VALUE(ints, BIT_PHY_ERR_INT, 1) &&
	    VALUE(inten, BIT_PHY_ERR_INTEN, 1)) {
		/* clear phy_err interrupt factor */
		hdc_write_bits(base, HDC_INTS, BIT_PHY_ERR_INT, 1, 0);

		/* process recovery controller hung-up */
		on_recovery_controller_hungup(priv);

		goto irq_handled;
	}

	devs = hdc_readl(base, HDC_DEVS);
	devc = hdc_readl(base, HDC_DEVC);

	/* bus reset begin interrupt factor */
	if (VALUE(devs, BIT_USBRSTB_INT, 1) &&
	    VALUE(devc, BIT_USBRSTB_INTEN, 1)) {
		/* clear usbrstb interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_USBRSTB_INT, 1, 0);

		dev_dbg(dev, "usbrstb interrupt is occurred\n");

		/* process bus reset begin */
		on_begin_bus_reset(priv);

		goto irq_handled;
	}

	/* bus reset end interrupt factor */
	if (VALUE(devs, BIT_USBRSTE_INT, 1) &&
	    VALUE(devc, BIT_USBRSTE_INTEN, 1)) {
		/* clear usbrste interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_USBRSTE_INT, 1, 0);

		dev_dbg(dev, "usbrste interrupt is occurred\n");

		/* process bus reset end */
		on_end_bus_reset(priv);

		goto irq_handled;
	}

	/* suspend end interrupt factor */
	if (VALUE(devs, BIT_SUSPENDE_INT, 1) &&
	    VALUE(devc, BIT_SUSPENDE_INTEN, 1)) {
		/* clear suspende interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDE_INT, 1, 0);

		dev_dbg(dev, "suspende interrupt is occurred\n");

		/* process bus wakeup */
		on_wakeup(priv);

		goto irq_handled;
	}

	/* SOF interrupt factor */
	if (VALUE(devs, BIT_SOF_INT, 1) &&
	    VALUE(devc, BIT_SOF_INTEN, 1)) {
		/* clear sof interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_SOF_INT, 1, 0);

		/* process SOF transfer end */
		on_end_transfer_sof(priv);

		goto irq_handled;
	}

	/* SETUP transfer interrupt factor */
	if (VALUE(devs, BIT_SETUP_INT, 1) &&
	    VALUE(devc, BIT_SETUP_INTEN, 1)) {
		/* clear setup interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_SETUP_INT, 1, 0);

		/* process control SETUP transfer end */
		on_end_control_setup_transfer(&priv->ep[EP0]);

		goto irq_handled;
	}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	/* endpoint x dma interrupt factor */
	for (ch = HDC_DMA_CH1; ch < HDC_MAX_DMA_CH; ch++) {
		if (VALUE(ints, BIT_DMA_INT(ch), 1) &&
		    VALUE(inten, BIT_DMA_INTEN(ch), 1)) {
			on_ep_dma_interrupt(priv, ch);
			goto irq_handled;
		}
	}
#endif
	/* endpoint x interrupt factor */
	for (ch = EP1; ch < HDC_UDC_MAX_EP; ch++) {
		if (VALUE(ints, BIT_DEV_EP_INT(ch), 1) &&
		    VALUE(inten, BIT_DEV_EP_INTEN(ch), 1)) {
			on_ep_interrupt(priv, ch);
			goto irq_handled;
		}
	}

	/* endpoint 0 interrupt factor */
	if (VALUE(ints, BIT_DEV_EP_INT(EP0), 1) &&
	    VALUE(inten, BIT_DEV_EP_INTEN(EP0), 1)) {
		on_ctrl_ep_interrupt(priv, EP0);
		goto irq_handled;
	}

	/* suspend begin interrupt factor */
	if (VALUE(devs, BIT_SUSPENDB_INT, 1) &&
	    VALUE(devc, BIT_SUSPENDB_INTEN, 1)) {
		/* clear suspendb interrupt factor */
		hdc_write_bits(base, HDC_DEVS, BIT_SUSPENDB_INT, 1, 0);

		dev_dbg(dev, "suspendb interrupt is occurred\n");

		/* process bus suspend */
		on_suspend(priv);

		goto irq_handled;
	}

irq_handled:
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

#define to_fusb20_udc(g)  (container_of((g), struct f_usb20hdc_udc, gadget))

static int hdc_gadget_start(struct usb_gadget *g,
			    struct usb_gadget_driver *driver)
{
	struct f_usb20hdc_udc *priv = to_fusb20_udc(g);
	u8 otg_suspended;
	struct device *dev = priv->gadget.dev.parent;

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	otg_suspended = priv->otg_suspend_state;
#else
	/*OTG role-switch function doesn't exist*/
	otg_suspended = 0;
#endif

	/* entry gadget driver structure */
	priv->gadget_driver = driver;

	if (!otg_suspended) {
		dev_dbg(dev, "%s() nitialize ep data\n", __func__);
		/* initialize endpoint configure data */
		initialize_endpoint_configure(priv);
	}

	/* initialize F_USB20HDC UDC device driver structure data */
	priv->gadget.speed = USB_SPEED_UNKNOWN;
	priv->bus_connect = 0;
	priv->device_state = USB_STATE_ATTACHED;
	priv->device_state_last = USB_STATE_NOTATTACHED;
	priv->configure_value_last = 0;
	priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	priv->ctrl_pri_dir = 1;

	/* set F_USB20HDC UDC device driver register */
	priv->device_add = 1;

	if (!otg_suspended) {
		dev_dbg(dev, "%s() setup recovery timer\n", __func__);
#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&priv->halt_transfer_error_recovery_timer,
			    on_recovery_halt_transfer_error, (u32)priv);
#endif
	}

	if (!otg_suspended) {
		dev_dbg(dev, "%s() enable communicate\n", __func__);
		/* enable communicate */
		enable_communicate(priv, 1);
	}

	dev_info(dev, "%s is registered\n",
		 priv->gadget_driver->driver.name);

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	/*this is for otg resume/suspend */
	priv->gadget_connected = 1;
#endif

	dev_dbg(dev, "%s() gadget is connected to udc function\n", __func__);
	return 0;
}

static int hdc_gadget_stop(struct usb_gadget *g)
{
	struct f_usb20hdc_udc *priv = to_fusb20_udc(g);
	u8 device_add_last;
	unsigned long flags;
	struct device *dev = priv->gadget.dev.parent;

	spin_lock_irqsave(&priv->lock, flags);

	/* clear F_USB20HDC UDC device driver register */
	device_add_last = priv->device_add;
	priv->device_add = 0;

	/* disable communicate */
	enable_communicate(priv, 0);

	spin_unlock_irqrestore(&priv->lock, flags);

	priv->gadget_driver = NULL;

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	/*this is for otg resume/suspend */
	priv->gadget_connected = 0;
#endif

	dev_dbg(dev, "%s() gadget is disconnected from udc function\n",
		__func__);
	return 0;
}

static int hdc_gadget_ep_enable(struct usb_ep *ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!ep || !desc))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (unlikely(endpoint->ep_ch == EP0)) {
		dev_err(dev, "%s() endpoint 0 enable is invalid\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* check endpoint descriptor parameter */
	if (unlikely(desc->bDescriptorType != USB_DT_ENDPOINT)) {
		dev_err(dev, "%s() endpoint %u descriptor type is error\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* check gadget driver parameter */
	if (unlikely(!priv->gadget_driver)) {
		dev_err(dev, "%s() gadget driver parameter is none\n",
			__func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	dev_dbg(dev, "%s() endpoint %u is enabled\n",
		__func__, endpoint->ep_ch);

	/* set endpoint parameter */
	endpoint->ep.desc = desc;

	/* configure endpoint */
	if (!configure_endpoint(endpoint)) {
		endpoint->ep.desc = NULL;
		dev_err(dev, "%s() endpoint %u configure is failed\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* dequeue all previous transfer request */
	dequeue_all_transfer_request(endpoint, -ECONNABORTED);

	/* enable endpoint */
	enable_endpoint(endpoint, 1);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_ep_disable(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!ep))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (unlikely(endpoint->ep_ch == EP0)) {
		dev_err(dev, "%s() endpoint 0 disable is invalid\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	dev_dbg(dev, "%s() endpoint %u is disabled\n", __func__,
		endpoint->ep_ch);

	/* set endpoint parameter */
	endpoint->ep.desc = NULL;

	/* disable endpoint */
	enable_endpoint(endpoint, 0);

	/* dequeue all previous transfer request */
	dequeue_all_transfer_request(endpoint, -ESHUTDOWN);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(dev, "%s() is ended\n", __func__);

	return 0;
}

static struct usb_request *hdc_gadget_ep_alloc_request(struct usb_ep *ep,
						       gfp_t gfp_flags)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	struct device *dev;
	/*
	 * [notice]:Acquisition of a spin lock is prohibition.
	 */

	if (unlikely(!ep))
		return 0;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
#if !defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
		return 0;
#endif
	}

	/* allocate memory and zero clear memory */
	request = kzalloc(sizeof(*request), gfp_flags);
	if (!request)
		return 0;

	dev_dbg(dev, "%s() endpoint %u allocate memory is 0x%p\n",
		__func__, endpoint->ep_ch, request);

	/* initialize list data */
	INIT_LIST_HEAD(&request->queue);
	request->req.dma = ~(dma_addr_t)0;

	dev_dbg(dev, "%s() is ended\n", __func__);

	return &request->req;
}

static void hdc_gadget_ep_free_request(struct usb_ep *ep,
				       struct usb_request *req)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	struct device *dev;

	/*
	 * [notice]:Acquisition of a spin lock is prohibition.
	 */

	if (unlikely(!ep || !req))
		return;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	request = container_of(req, struct f_usb20hdc_udc_req, req);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		return;
	}

	/* free memory */
	WARN_ON(!list_empty(&request->queue));
	kfree(request);
	dev_dbg(dev, "%s() endpoint %u free memory is 0x%p\n",
		__func__, endpoint->ep_ch, request);

	dev_dbg(dev, "%s() is ended\n", __func__);
}

static int hdc_gadget_ep_queue(struct usb_ep *ep, struct usb_request *req,
			       gfp_t gfp_flags)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!ep || !req))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	request = container_of(req, struct f_usb20hdc_udc_req, req);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	if (unlikely(!req->buf || !list_empty(&request->queue))) {
		dev_err(dev, "%s() request parameter is error\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}
	if (unlikely((endpoint->ep_ch != EP0) && !endpoint->ep.desc)) {
		dev_err(dev, "%s() endpoint %u descriptor is none\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* check gadget driver parameter */
	if (unlikely(!priv->gadget_driver)) {
		dev_err(dev, "%s() gadget driver parameter is none\n",
			__func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* check state */
	if (priv->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(dev, "%s() device state is reset\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* initialize request parameter */
	request->req.status = -EINPROGRESS;
	request->req.actual = 0;

	/* check current queue execute */
	if (!list_empty(&endpoint->queue) || endpoint->halt) {
		/* add list tail */
		list_add_tail(&request->queue, &endpoint->queue);
		spin_unlock_irqrestore(&priv->lock, flags);
		return 0;
	}

	/* save request */
	endpoint->req = request;

	if (endpoint->ep_ch == EP0) {
		/* request endpoint 0 */
		switch (priv->ctrl_stage) {
		case F_USB20HDC_STAGE_IN_DATA:
		case F_USB20HDC_STAGE_IN_STATUS:
			if (!set_in_transfer_pio(endpoint))
				goto ERR_EL2HLT;
			break;
		case F_USB20HDC_STAGE_OUT_DATA:
		case F_USB20HDC_STAGE_OUT_STATUS:
			if (!set_out_transfer_pio(endpoint))
				goto ERR_EL2HLT;
			break;
		default:
			dev_dbg(dev, "%s() control transfer stage is changed at %u\n",
				__func__, priv->ctrl_stage);
			goto ERR_EL2HLT;
		}
	} else {
		/* request endpoint x */
		if (endpoint->transfer_direction) {
			if (!set_in_transfer(endpoint))
				goto ERR_EL2HLT;
		} else {
			if (!set_out_transfer(endpoint))
				goto ERR_EL2HLT;
		}
	}

	/* add list tail */
	list_add_tail(&request->queue, &endpoint->queue);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;

ERR_EL2HLT:
	dev_dbg(dev, "%s() %d\n", __func__, __LINE__);
	notify_transfer_request_complete(endpoint, -EL2HLT);

	spin_unlock_irqrestore(&priv->lock, flags);

	return -EL2HLT;
}

static int hdc_gadget_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!ep || !req))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	request = container_of(req, struct f_usb20hdc_udc_req, req);
	priv = endpoint->priv;
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (unlikely((endpoint->ep_ch != EP0) && !endpoint->ep.desc)) {
		dev_err(dev, "%s() ep %u descriptor is none\n",
			__func__, endpoint->ep_ch);
		goto ERR_EINVAL;
	}

	/*
	 * check queue empty
	 * [notice]:It is mandatory processing.
	 */
	if (!list_empty(&endpoint->queue)) {
		/* check list entry */
		list_for_each_entry(request, &endpoint->queue, queue) {
			if ((request) && (&request->req == req))
				break;
		}

		/* check dequeue request mismatch */
		if (unlikely(!request || (&request->req != req))) {
			dev_err(dev, "%s() request is invalid\n", __func__);
			goto ERR_EINVAL;
		}
	} else {
		dev_err(dev, "%s() queue is empty\n", __func__);
		goto ERR_EINVAL;
	}

	/* abort request transfer */
	endpoint->req = request;
	if (endpoint->ep_ch == EP0) {
		abort_in_transfer(endpoint, 0);
		abort_out_transfer(endpoint, 0);
	} else {
		endpoint->transfer_direction ? abort_in_transfer(endpoint, 0) :
						abort_out_transfer(endpoint, 0);
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		if (endpoint->dma_transfer) {
			spin_unlock(&priv->lock);
			hdmac_stop_nowait(priv->dma_data[endpoint->dma_ch].
					  hdmac_ch);
			spin_lock(&priv->lock);
		}
#endif
	}

	notify_transfer_request_complete(endpoint, -ECONNRESET);

	/* check next queue empty */
	if (list_empty(&endpoint->queue))
		goto finish;

	/* get next request */
	request = list_entry(endpoint->queue.next, struct f_usb20hdc_udc_req,
			     queue);

	/* check the got next request a request under current execution */
	if (request->request_execute)
		goto finish;

	/* save request */
	endpoint->req = request;

	/* set endpoint x transfer request */
	if (!(endpoint->transfer_direction ? set_in_transfer(endpoint) :
	    set_out_transfer(endpoint))) {
		dev_err(dev, "%s() %d\n", __func__, __LINE__);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

finish:
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;

ERR_EINVAL:
	spin_unlock_irqrestore(&priv->lock, flags);

	return -EINVAL;
}

static int hdc_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;

	if (unlikely(!ep || ((value != 0) && (value != 1))))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (unlikely((endpoint->ep_ch != EP0) && !endpoint->ep.desc)) {
		dev_err(priv->dev, "%s() endpoint %u descriptor is none\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->transfer_type == USB_ENDPOINT_XFER_ISOC)) {
		dev_dbg(priv->dev, "%s() ep %u is isochoronous transfer\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (value) {
		/* check current queue execute */
		if (!list_empty(&endpoint->queue)) {
			dev_dbg(priv->dev, "%s() endpoint %u is execute queue\n",
				__func__, endpoint->ep_ch);
			spin_unlock_irqrestore(&priv->lock, flags);
			return -EAGAIN;
		}

		/* set halt */
		dev_dbg(priv->dev, "%s() endpoint %u transfer is halted\n",
			__func__, endpoint->ep_ch);
		halt_transfer(endpoint, 1, 0);
	} else {
		/* clear halt */
		dev_dbg(priv->dev, "%s() endpoint %u transfer halt is cleared\n",
			__func__, endpoint->ep_ch);
		halt_transfer(endpoint, 0, 0);
		endpoint->force_halt = 0;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;

	if (unlikely(!ep))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	if (unlikely((endpoint->ep_ch != EP0) && !endpoint->ep.desc)) {
		dev_err(priv->dev, "%s() endpoint %u descriptor is none\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->transfer_type == USB_ENDPOINT_XFER_ISOC)) {
		dev_err(priv->dev, "%s() endpoint %u is isochoronous transfer\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* check current queue execute */
	if (!list_empty(&endpoint->queue)) {
		dev_err(priv->dev, "%s() endpoint %u queue is execute\n",
			__func__, endpoint->ep_ch);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EAGAIN;
	}

	/* set force halt */
	dev_err(priv->dev, "%s() endpoint %u transfer is wedged\n",
		__func__, endpoint->ep_ch);
	halt_transfer(endpoint, 1, 1);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_ep_fifo_status(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	u16 bytes;
	unsigned long flags;

	if (unlikely(!ep))
		return -EINVAL;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* get current bytes in FIFO */
	bytes = get_fifo_bytes(endpoint);
	dev_dbg(priv->dev, "%s() bytes in FIFO is %u\n", __func__, bytes);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return (int)bytes;
}

static void hdc_gadget_ep_fifo_flush(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *priv;
	struct f_usb20hdc_udc_ep *endpoint;
	unsigned long flags;

	if (unlikely(!ep))
		return;

	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, ep);
	priv = endpoint->priv;

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		dev_dbg(priv->dev, "%s() is ended\n", __func__);
		return;
	}

	/* initialize endpoint FIFO */
	dev_dbg(priv->dev, "%s() endpoint %u FIFO is flush\n",
		__func__, endpoint->ep_ch);
	initialize_endpoint(endpoint, 1, 0, 0);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);
}

static int hdc_gadget_get_frame(struct usb_gadget *gadget)
{
	struct f_usb20hdc_udc *priv;
	u16 frame_number;
	unsigned long flags;

	if (unlikely(!gadget))
		return -EINVAL;

	priv = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* get frame number */
	frame_number = hdc_read_bits(priv->reg_base, HDC_TSTAMP_R_ONLY,
				     BIT_TIMESTAMP, LEN_TIMESTAMP);
	dev_dbg(priv->dev, "%s() frame number is %u\n",
		__func__, frame_number);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return (int)frame_number;
}

static int hdc_gadget_wakeup(struct usb_gadget *gadget)
{
	struct f_usb20hdc_udc *priv;
	unsigned long flags;

	if (unlikely(!gadget))
		return -EINVAL;

	priv = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* check device state */
	if (priv->device_state != USB_STATE_SUSPENDED) {
		dev_err(priv->dev, "%s() device state is invalid at %u\n",
			__func__, priv->device_state);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* output resume */
	hdc_write_bits(priv->reg_base, HDC_DEVC, BIT_REQRESUME, 1, 1);
	dev_dbg(priv->dev, "%s() resume is output\n", __func__);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_set_selfpowered(struct usb_gadget *gadget,
				      int is_selfpowered)
{
	struct f_usb20hdc_udc *priv;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!gadget))
		return -EINVAL;

	priv = container_of(gadget, struct f_usb20hdc_udc, gadget);
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
#if !defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
#endif
	}

	/* set self-powered status */
	priv->selfpowered = !!is_selfpowered;
	dev_dbg(dev, "%s() self power status is %s\n",
		__func__, is_selfpowered ? "selfpowered" : "buspowered");

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct f_usb20hdc_udc *priv;
	unsigned long flags;

	if (unlikely(!gadget))
		return -EINVAL;

	priv = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(priv->dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* check bus active */
	if (is_active) {
		/* set bus disconnect detect */
		hdc_set_connect_detect(priv->reg_base, 0);

		/* enable host connect */
		enable_host_connect(priv, 1);
	} else {
		/* disable host connect */
		enable_host_connect(priv, 0);

		/* set bus connect detect */
		hdc_set_connect_detect(priv->reg_base, 1);

		/* notify disconnect to gadget driver */
		spin_unlock(&priv->lock);
		if (priv->gadget_driver && priv->gadget_driver->disconnect)
			priv->gadget_driver->disconnect(&priv->gadget);
		spin_lock(&priv->lock);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_vbus_draw(struct usb_gadget *gadget, u32 ma)
{
	/* it is not implementation */
	return -EOPNOTSUPP;
}

static int hdc_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct f_usb20hdc_udc *priv;
	unsigned long flags;
	struct device *dev;

	if (unlikely(!gadget))
		return -EINVAL;

	priv = container_of(gadget, struct f_usb20hdc_udc, gadget);
	dev = priv->gadget.dev.parent;

	dev_dbg(dev, "%s() is started\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() host mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* check connect request */
	if (is_on) {
		/* check bus connect status */
		if (!priv->bus_connect) {
			dev_err(dev, "%s() D+ can't pull-up\n", __func__);
			spin_unlock_irqrestore(&priv->lock, flags);
			return -EPROTO;
		}

		/* set bus disconnect detect */
		hdc_set_connect_detect(priv->reg_base, 0);

		/* enable host connect */
		enable_host_connect(priv, 1);
	} else {
		/* check bus connect status */
		if (priv->bus_connect) {
			/* disable host connect */
			enable_host_connect(priv, 0);

			/* set bus connect detect */
			hdc_set_connect_detect(priv->reg_base, 1);
		}
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(dev, "%s() is ended\n", __func__);

	return 0;
}

static int hdc_gadget_ioctl(struct usb_gadget *gadget, unsigned code,
			    unsigned long param)
{
	/* it is not implementation */
	return -ENOIOCTLCMD;
}

static void hdc_gadget_release(struct device *dev)
{
}

static const struct usb_ep_ops f_usb20hdc_udc_ep_ops = {
	.enable			= hdc_gadget_ep_enable,
	.disable		= hdc_gadget_ep_disable,
	.alloc_request		= hdc_gadget_ep_alloc_request,
	.free_request		= hdc_gadget_ep_free_request,
	.queue			= hdc_gadget_ep_queue,
	.dequeue		= hdc_gadget_ep_dequeue,
	.set_halt		= hdc_gadget_ep_set_halt,
	.set_wedge		= hdc_gadget_ep_set_wedge,
	.fifo_status		= hdc_gadget_ep_fifo_status,
	.fifo_flush		= hdc_gadget_ep_fifo_flush,
};

static const struct usb_gadget_ops f_usb20hdc_udc_gadget_ops = {
	.get_frame		= hdc_gadget_get_frame,
	.wakeup			= hdc_gadget_wakeup,
	.set_selfpowered	= hdc_gadget_set_selfpowered,
	.vbus_session		= hdc_gadget_vbus_session,
	.vbus_draw		= hdc_gadget_vbus_draw,
	.pullup			= hdc_gadget_pullup,
	.ioctl			= hdc_gadget_ioctl,
	.udc_start		= hdc_gadget_start,
	.udc_stop		= hdc_gadget_stop,
};

int hdc_gadget_probe(struct f_usb20hdc *f_otg)
{
	u32 ch;
	struct f_usb20hdc_udc *priv;
	struct platform_device *pdev = f_otg->pdev;
	void __iomem *reg_base;
	int result;

	if (unlikely(!pdev))
		return -EINVAL;

	/* allocate F_USB20HDC UDC device driver structure data memory */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		result = -ENOMEM;
		goto err_nomem;
	}

	priv->dev = &pdev->dev;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	for (ch = HDC_DMA_CH1; ch < HDC_MAX_DMA_CH; ch++) {
		priv->dma_data[ch] = f_otg->dma_data[ch];
		priv->dma_data[ch].ep_ch = -1;
	}
#endif

	/* get a register base address for a F_USB20HDC device */
	reg_base = ioremap(f_otg->mem_start, f_otg->mem_size);
	if (!reg_base) {
		dev_err(&pdev->dev, "%s() %d\n", __func__, __LINE__);
		result = -ENODEV;
		goto err_res;
	}

	/* check endpoint buffer size */
	if (!is_endpoint_buffer_usable(f_otg)) {
		dev_err(&pdev->dev, "%s() %d\n", __func__, __LINE__);
		result = -ENODEV;
		goto err_irq_buf;
	}

	/* initialize data except for those need to be set to zero */
	priv->f_otg = f_otg;
	priv->gadget.ops = &f_usb20hdc_udc_gadget_ops;
	priv->gadget.ep0 = &priv->ep[0].ep;
	INIT_LIST_HEAD(&priv->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&priv->gadget.ep_list);
	priv->gadget.speed = USB_SPEED_UNKNOWN;
	priv->gadget.max_speed = USB_SPEED_HIGH;
	priv->gadget.name = "f_usb20hdc_udc";
	dev_set_name(&priv->gadget.dev, "gadget");
	priv->gadget.dev.release = hdc_gadget_release;
	spin_lock_init(&priv->lock);
#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	setup_timer(&priv->halt_transfer_error_recovery_timer,
		    on_recovery_halt_transfer_error, (u32)priv);
#endif
	priv->resource = f_otg->mem_res;
	priv->reg_base = reg_base;
	priv->irq =  f_otg->irq;
	initialize_endpoint_configure(priv);
	for (ch = EP0; ch < HDC_UDC_MAX_EP; ch++) {
		priv->ep[ch].ep.ops = &f_usb20hdc_udc_ep_ops;
		list_add_tail(&priv->ep[ch].ep.ep_list, ch == EP0 ?
			      &priv->gadget.ep0->ep_list :
			      &priv->gadget.ep_list);
		priv->ep[ch].priv = priv;
		INIT_LIST_HEAD(&priv->ep[ch].queue);
	}
	priv->selfpowered = 1;
	priv->device_state = USB_STATE_NOTATTACHED;
	priv->device_state_last = USB_STATE_NOTATTACHED;
	priv->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	priv->ctrl_pri_dir = 1;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	result = hdc_dma_attach(priv->dma_data, HDC_MAX_DMA_CH,
				HDC_UDC_DMA_TRANS_MAX_BYTES);
	if (result < 0) {
		dev_err(&pdev->dev, "Failed to attach DMA\n");
		goto err_dma;
	}
#endif

	/* save the private data of a device driver */
	f_otg->f_usb20hdc_udc = priv;

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(priv, 1);

	/* entry a F_USB20HDC device IRQ */
	result = request_irq(priv->irq, on_usb_function, IRQF_SHARED,
			     "f_usb20hdc_udc", priv);
	if (result) {
		dev_err(&pdev->dev, "%s() %d\n", __func__, __LINE__);
		goto err_req_irq;
	}

	result = usb_add_gadget_udc(&pdev->dev, &priv->gadget);
	if (result) {
		dev_err(&pdev->dev, "usb_add_gadget_udc failed!\n");
		goto err_add_gadget;
	}

	/* driver registering log output */
	dev_info(&pdev->dev, "F_USB20HDC UDC driver (version %s) is registered\n",
		 HDC_UDC_DRIVER_VERSION);

	return 0;

err_add_gadget:
	free_irq(priv->irq, priv);
err_req_irq:
	f_otg->f_usb20hdc_udc = NULL;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
err_dma:
	hdc_dma_detach(priv->dma_data, HDC_MAX_DMA_CH,
		       HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif
err_irq_buf:
	iounmap(reg_base);
err_res:
	kfree(priv);
err_nomem:

	return result;
}

#ifdef CONFIG_PM
int hdc_gadget_suspend(struct f_usb20hdc *f_otg)
{
	struct f_usb20hdc_udc *priv = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started\n", __func__);

	if (!priv) {
		dev_dbg(dev, "%s() no udc instance\n", __func__);
		return -EINVAL;
	}

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	if (priv->otg_suspend_state) {
		dev_dbg(dev, "%s() do nothing casue it's otg suspended\n",
			__func__);
		return 0;
	}
#endif

	/* disable dev mode interrupt factor*/
	hdc_write_bits(priv->reg_base, HDC_INTEN, BIT_DEV_INTEN, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_OTGSTSRISE, BIT_VBUS_VLD_REN, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_OTGSTSFALL, BIT_VBUS_VLD_FEN, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_OTGSTSC, BIT_VBUS_VLD_C, 1, 0);

	spin_lock_irqsave(&priv->lock, flags);

	/* disconnect from the host if gadget driver is registered */
	if (priv->device_add)
		enable_communicate(priv, 0);

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(dev, "%s() is ended\n", __func__);

	return 0;
}

int hdc_gadget_resume(struct f_usb20hdc *f_otg)
{
	struct f_usb20hdc_udc *priv = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started\n", __func__);

	if (!priv) {
		dev_dbg(dev, "%s():no udc instance\n", __func__);
		return -EINVAL;
	}

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	if (priv->otg_suspend_state) {
		dev_dbg(dev, "%s() do nothing casue it's otg suspended\n",
			__func__);
		return 0;
	}
#endif

	spin_lock_irqsave(&priv->lock, flags);

#ifdef COLD_RESUME_SUPPORT
	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(priv, 1);
#endif

	/* enable communicate if gadget driver is registered */
	if (priv->device_add) {
		dev_dbg(dev, "%s() class driver were connected\n", __func__);
#ifdef COLD_RESUME_SUPPORT
		/* restore gadget setting : initialize endpoint configure */
		initialize_endpoint_configure(priv);
#endif
#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&priv->halt_transfer_error_recovery_timer,
			    on_recovery_halt_transfer_error, (u32)priv);
#endif
		enable_communicate(priv, 1);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	/* enable dev mode interrupt factor */
	hdc_write_bits(priv->reg_base, HDC_OTGSTSRISE, BIT_VBUS_VLD_REN, 1, 1);
	hdc_write_bits(priv->reg_base, HDC_OTGSTSFALL, BIT_VBUS_VLD_FEN, 1, 1);

	dev_dbg(dev, "%s() is ended\n", __func__);
	return 0;
}
#endif /*CONFIG_PM*/

int hdc_gadget_remove(struct f_usb20hdc *f_otg)
{
	struct f_usb20hdc_udc *priv = f_otg->f_usb20hdc_udc;
	struct platform_device *pdev = f_otg->pdev;

	if (unlikely(!pdev))
		return -EINVAL;

	if (!priv) {
		dev_err(&pdev->dev, "%s() %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	usb_del_gadget_udc(&priv->gadget);

	/* free HDMAC channel and noncachable buffers */
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	hdc_dma_detach(priv->dma_data, HDC_MAX_DMA_CH,
		       HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
	if (!priv->otg_suspend_state)
#endif
		free_irq(priv->irq, priv);

	/* disable communicate */
	enable_communicate(priv, 0);

	/* notify disconnect to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->disconnect)
		priv->gadget_driver->disconnect(&priv->gadget);

	/* notify unbind to gadget driver */
	if (priv->gadget_driver && priv->gadget_driver->unbind)
		priv->gadget_driver->unbind(&priv->gadget);

	/* release device resource */
	f_otg->f_usb20hdc_udc = NULL;
	iounmap(priv->reg_base);

	/* free F_USB20HDC UDC device driver structure data memory */
	kfree(priv);

	/* driver deregistering log output */
	dev_info(&pdev->dev, "F_USB20HDC UDC driver is deregistered\n");

	return 0;
}

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
int hdc_gadget_otg_resume(struct f_usb20hdc *f_otg)
{
	struct f_usb20hdc_udc *priv = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	int ret = 0;

	dev_dbg(dev, "%s() is started from %pS\n", __func__,
		__builtin_return_address(0));

	if (!priv) {
		dev_err(dev, "Socionext udc failed to otg_resume\n");
		return -1;
	}

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(priv, 1);

	/* entry a F_USB20HDC device IRQ */
	ret = request_irq(priv->irq, on_usb_function,
			  IRQF_SHARED, "f_usb20hdc_udc", priv);
	if (ret < 0) {
		dev_err(dev, "Socionext udc failed to resume\n");
		return -1;
	}

	if (priv->gadget_connected) {
		dev_dbg(dev, "Socionext udc's high level gadget connected state : %d",
			priv->gadget_connected);

		/* restore gadget setting : initialize endpoint configure */
		initialize_endpoint_configure(priv);

#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&priv->halt_transfer_error_recovery_timer,
			    on_recovery_halt_transfer_error, (u32)priv);
#endif

		/* restore gadget setting : enable communicate */
		enable_communicate(priv, 1);
	}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	ret = hdc_dma_attach(priv->dma_data, HDC_MAX_DMA_CH,
			     HDC_UDC_DMA_TRANS_MAX_BYTES);
	if (ret < 0) {
		dev_err(dev, "Failed to attach DMA\n");
		return -1;
	}
#endif

	dev_dbg(dev, "Socionext udc resume\n");
	priv->otg_suspend_state = 0;
	return 0;
}

int hdc_gadget_otg_suspend(struct f_usb20hdc *f_otg)
{
	struct f_usb20hdc_udc *priv = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;

	dev_dbg(dev, "%s() is started from %pS\n", __func__,
		__builtin_return_address(0));

	if (!priv) {
		pr_err("Socionext udc failed to otg_suspend\n");
		return -1;
	}

	/* disable communicate */
	enable_communicate(priv, 0);

	/* quarantine udc driver from any hardware event*/
	free_irq(priv->irq, priv);

	/* free HDMAC channel and noncachable buffers */
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	hdc_dma_detach(priv->dma_data, HDC_MAX_DMA_CH,
		       HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif

	dev_dbg(dev, "USB Gadget suspended\n");
	priv->otg_suspend_state = 1;
	return 0;
}
#endif
