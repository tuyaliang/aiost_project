/*
 * Copyright (C) 2015 Linaro Ltd.
 * Author: Jassi Brar <jassisinghbrar@gmail.com>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#define CONF		0x0
#define BYTE_ORDER	0
#define BURST_WAIT	1
#define SOFT_RESET	2

#define MODE		0x4
#define HOST_EN		0
#define DEV_EN		1
#define DEV_INT_MODE	2
#define ADDR_LD_MODE	3

#define INTEN		0x8
#define HOST_INTEN	0
#define DEV_INTEN	1
#define OTG_INTEN	2
#define PHY_ERR_INTEN	3
#define CMD_INTEN	4
#define DMA_INTEN(ch)	(8 + (ch))
#define DEV_EP_INTEN(n)	(16 + (n))
#define HDC_DMA_CH1	0
#define HDC_DMA_CH2	1

#define INTS		0xc
#define OTG_INT		2
#define PHY_ERR_INT	3
#define CMD_INT		4
#define DMA_INT(ch)	(8 + (ch))
#define DEV_EP_INT(ep)	(16 + (ep))

#define EPCMD0		0x40
#define EPCMD(n)	(EPCMD0 + (n) * 4)
#define START		0
#define STOP		1
#define INIT		2
#define BUFWR		3
#define BUFRD		4
#define ET		23
#define LEN_ET		2
#define BNUM		26
#define LEN_BNUM	2
#define DIR_IN		25
#define HIBAND		28
#define LEN_HIBAND	2
#define STALL_CLR	6
#define NULLRESP	9
#define NACKRESP	10
#define WRITE_EN	11
#define RDYI_INTEN	12
#define RDYO_EMPTY_INTEN	13
#define PING_INTEN	14
#define STALLED_INTEN	15
#define NACK_INTEN	16
#define RDYI_INT_CLR	18
#define RDYO_EMPTY_INT_CLR	19
#define TOGGLE_CLR	8
#define TOGGLE_SET	7
#define STALL_SET	5

#define PORTSC		0x100
#define POWER_RHS	2
#define POWER_CTL_REQ	21
#define POWER_REQ	22

#define DEVC		0x200
#define REQRESUME	2
#define RMTWKUP		3
#define DISCONNECT	5
#define PHYSUSP		16
#define SUSPENDE_INTEN	24
#define SUSPENDB_INTEN	25
#define SOF_INTEN	26
#define SETUP_INTEN	27
#define USBRSTE_INTEN	28
#define USBRSTB_INTEN	29
#define STATUS_OK_INTEN	30
#define STATUS_NG_INTEN	31

#define DEVS		0x204
#define SUSPEND		0
#define BUSRESET	1
#define PHYRESET	16
#define CRTSPEED	17
#define FULL_SPEED	1
#define SUSPENDE_INT	24
#define SUSPENDB_INT	25
#define SOF_INT		26
#define SETUP_INT	27
#define USBRSTE_INT	28
#define USBRSTB_INT	29
#define STATUS_OK_INT	30
#define STATUS_NG_INT	31

#define FADDR		0x208
#define FUNC_ADDR	0
#define LEN_FUNC_ADDR	7
#define DEV_CONFIGURED	8

#define TSTAMP		0x20c
#define TIMESTAMP	0
#define LEN_TIMESTAMP	11

#define OTGC		0x300
#define DM_PULL_DOWN	7
#define DP_PULL_DOWN	8

#define OTGSTS		0x310
#define VBUS_VLD	10

#define OTGSTSC		0x314
#define ID_C		6
#define TMROUT_C	0
#define VBUS_VLD_C	10

#define OTGSTSFALL	0x318
#define VBUS_VLD_FEN	10

#define OTGSTSRISE	0x31c
#define TMROUT_REN	0
#define VBUS_VLD_REN	10

#define OTGTC		0x320
#define START_TMR	0

#define OTGT		0x324
#define TMR_INIT_VAL	0
#define LEN_TMR_INIT_VAL	16

#define SN_UDC_DMA(n)	(0x400 + (n) * 0x20)
#define DMA_C(n)	(SN_UDC_DMA(n) + 0x0)
#define DMA_START	0
#define DMA_BUSY	1
#define DMA_MODE	2
#define SEND_NULL	3
#define DMAINT		4
#define SPR		5
#define DMAEP		8
#define LEN_DMAEP	4
#define BLKSIZE		16
#define LEN_BLKSIZE	11

#define DMA_S(n)	(SN_UDC_DMA(n) + 0x4)
#define SHORT_P		1
#define NULL_P		0

#define DMATCI(n)	(SN_UDC_DMA(n) + 0x8)
#define DMATC(n)	(SN_UDC_DMA(n) + 0xc)

#define TESTC		0x500
#define TESTP		0
#define TESTJ		1
#define TESTK		2
#define TESTSE0NAK	3

#define EPCTRL0		0x8000
#define EPCTRL(n)	(EPCTRL0 + (n) * 4)
#define EP_EN		0
#define APPPTR		6
#define LEN_APPPTR	2
#define PHYPTR		8
#define LEN_PHYPTR	2
#define EMPTY		10
#define FULL		11
#define STALL		12
#define EPCTRL_NACKRESP	17
#define READY_INTEN	18
#define EMPTY_INTEN	19
#define STALL_INTEN	21
#define EPCTRL_NACK_INTEN	22
#define RDYI_RDY_INT	26
#define EMPTY_INT	27

#define EPCONF0		0x8040
#define EPCONF(n)	(EPCONF0 + (n) * 4)
#define BASE		0
#define LEN_BASE	13
#define SIZE		13
#define LEN_SIZE	11
#define COUNTIDX	24
#define LEN_COUNTIDX	5

#define EPCOUNT0	0x8080
#define EPCOUNT(n)	(EPCOUNT0 + (n) * 4)
#define APPCNT		0
#define LEN_APPCNT	11
#define PHYCNT		16
#define LEN_PHYCNT	11

/* exstop of m9m usb20 reg offset */
#define M9M_USB2_CKCTL		0x0
#define M9M_USB2_RCTL		0x4
#define M9M_USB2_ANPD		0x8
#define M9M_USB2_HFSEL		0xC
#define M9M_USB2_FSEL		0x10
#define M9M_USB2_EHCI_SB	0x14
#define M9M_USB2_EHCI_STRAP	0x18
#define M9M_USB2_OHCI_IF	0x1C
#define M9M_USB2_AHB_SLVIF	0x20
#define M9M_USB2_LMODSET	0x24
#define M9M_USB2_PMODSET	0x28
#define M9M_USB2_HPT_CTL	0x2C
#define M9M_USB2_IDVBUSCTL	0x30
#define M9M_USB2_HDMAC1		0x38
#define M9M_USB2_HDMAC2		0x3C
#define M9M_USB2_DMAFSM1	0x40
#define M9M_USB2_DMAFSM2	0x44


#define MASK(b)		((u32)-1 >> (32 - (b)))
#define VALUE(r, s, b)	(((r) >> (s)) & MASK(b))

#define SN_EP_NUM		8
#define EP0_MAX_PACKET		64
#define EPn_MAX_PACKET		((EP_SRAM_SZ - EP0_MAX_PACKET * 2) / 2)

#define EP_SRAM_SZ		0x2000 /* 8K */
#define EP_SRAM_LOC		0x8100

#define EPNAME_SIZE	8

#define DMA_OUT	0
#define DMA_IN	1

#define DMA_FIFO	0x8000

#define SS	16
#define CI	19
#define EI	20
#define DE	31
#define PR	28

#define UDC_DREQ	0x0e000000
#define HDMACA_BT_SINGLE	(8 << 20)	/* same as NORMAL*/
#define HDMACA_BT_INCR4		(11 << 20)
#define HDMACA_BT_INCR8		(13 << 20)
#define HDMACA_BT_INCR16	(15 << 20)

#define DMACR		0
#define DMACA(n)	(((n) + 1) * 0x10 + 0x0)
#define DMACB(n)	(((n) + 1) * 0x10 + 0x4)
#define DMASA(n)	(((n) + 1) * 0x10 + 0x8)
#define DMADA(n)	(((n) + 1) * 0x10 + 0xc)

#define BEAT		64
#define LPRC		3
#define HDMACA_BT	HDMACA_BT_INCR16

#define HDMACB_FD		(1 << 24)
#define HDMACB_FS		(1 << 25)
#define HDMACB_TT_2CYCLE	(0 << 30)
#define HDMACB_MS_BLOCK		(0 << 28)
#define HDMACB_EI		(1 << 20)
#define HDMACB_CI		(1 << 19)
#define HDMACB_TW_WORD		(2 << 26)

struct sn_req {
	struct usb_request usb_req; /* keep at top */
	struct list_head queue;
	struct sn_ep *ep;
	bool zlp;
};

struct sn_ep {
	struct usb_ep ep_usb; /* keep at top */
	char name[EPNAME_SIZE];
	unsigned buf_offset, buf_length;
	struct sn_udc *udc;
	unsigned status;
};

enum fhdc_variants {
FHDC_VARIANT_M9M
};

struct sn_udc {
	struct usb_gadget gadget; /* keep at top */
	struct usb_gadget_driver *driver;
	struct sn_ep ep[SN_EP_NUM];
	struct usb_ctrlrequest setup;
	struct sn_req ep0req;
	struct sn_req *ep0_active;
	u8 ep0buf[64];
	struct device *dev;
	struct sn_req *dmarx_busy;
	struct sn_req *dmatx_busy;
	void __iomem *reg_base;
	void __iomem *dma_base;
	dma_addr_t dma_offset;
	spinlock_t lock;
	struct list_head q_in;
	struct list_head q_out;
	u32 remote_wkp;
	u32 buspowered;
	u8 testmode;
	enum fhdc_variants variant;
};

static const char driver_name[] = "f_usb20hdc_drc";//"socionext-udc";

/* Control endpoint configuration.*/
static const struct usb_endpoint_descriptor ep0_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(EP0_MAX_PACKET),
};

static inline int is_in(struct sn_ep *ep)
{
	return !!(ep->ep_usb.address & USB_DIR_IN);
}

static inline int idx_of(struct sn_ep *ep)
{
	return ep->ep_usb.address & USB_ENDPOINT_NUMBER_MASK;
}

static inline bool need_dma(struct sn_req *req)
{
	return idx_of(req->ep);
}

static inline u32 hdc_read_bits(struct sn_udc *udc, u32 offset,
				u8 start_bit, u8 bit_length)
{
	u32 reg = readl_relaxed(udc->reg_base + offset);
	return VALUE(reg, start_bit, bit_length);
}

static inline int is_bus_connected(struct sn_udc *udc)
{
	return hdc_read_bits(udc, OTGSTS, VBUS_VLD, 1);
}

static inline void hdc_cmd0_spin(void __iomem *base, u32 id)
{
	u32 counter = 0xffff;

	while (VALUE(readl_relaxed(base + EPCMD(id)), 31, 1))
		if (!--counter)
			break;
}

static void hdc_write_ep(struct sn_udc *udc, u32 id, u8 start_bit,
			   u8 bit_length, u32 value)
{
	void __iomem *base = udc->reg_base;
	u32 reg, c0, r0, ctrl;
 
	hdc_cmd0_spin(base, id);

	r0 = reg = readl_relaxed(base + EPCMD(id));
	if (id == 0)
		goto do_write;

	reg &= ~((0x1f << 18) | (0x1ff << 0)); /* clear volatile bits */

	c0 = ctrl = readl_relaxed(base + EPCTRL(id));
	reg |= (VALUE(ctrl, 22, 1) << 10); /* nacken */
	reg |= (VALUE(ctrl, 21, 1) << 15); /* stallen */
	reg |= (VALUE(ctrl, 20, 1) << 14); /* pingen */
	reg |= (VALUE(ctrl, 19, 1) << 13); /* emptyen */
	reg |= (VALUE(ctrl, 18, 1) << 12); /* readyen */
	reg |= (VALUE(ctrl, 17, 1) << 10); /* nackresp */
	reg |= (VALUE(ctrl, 16, 1) << 9); /* nullresp */
	reg |= (VALUE(ctrl, 14, 2) << 28); /* HIBAND */
	reg |= (VALUE(ctrl, 4, 2) << 26); /* BNUM */
	reg |= (VALUE(ctrl, 3, 1) << 25); /* DIR */
	reg |= (VALUE(ctrl, 1, 2) << 23); /* ET */

do_write:
	reg |= BIT(WRITE_EN);
	reg &= ~(MASK(bit_length) << start_bit);
	reg |= ((value & MASK(bit_length)) << start_bit);
	writel_relaxed(reg, base + EPCMD(id));

	hdc_cmd0_spin(base, id);
}

static void hdc_write_bits(struct sn_udc *udc, u32 offset, u32 start_bit,
			   u32 bit_length, u32 value)
{
	u32 reg;

	reg = readl_relaxed(udc->reg_base + offset);
	reg &= ~(MASK(bit_length) << start_bit);
	reg |= ((value & MASK(bit_length)) << start_bit);
	writel_relaxed(reg, udc->reg_base + offset);
}

static void hdc_set_epcmd_stop(struct sn_udc *udc, u8 id)
{
	hdc_write_ep(udc, id, STOP, 1, 1);
	hdc_read_bits(udc, DEVS, CRTSPEED, 1) ?
			udelay(250) : mdelay(2);
}

static void hdc_set_vbus(struct sn_udc *udc, u8 enable)
{
	u32 i = 0xffff;

	hdc_write_bits(udc, PORTSC, POWER_REQ, 1, !!enable);

	while (hdc_read_bits(udc, PORTSC,
				POWER_RHS, 1) ^ enable)
		if (--i == 0)
			break;
}

static u32 _hdc_read_id_state(struct sn_udc *udc)
{
	return 0;
}

static void _hdc_soft_reset(struct sn_udc *udc)
{
	int timeout=0xFFFF;

	hdc_write_bits(udc, CONF, SOFT_RESET, 1, 1);
	while (hdc_read_bits(udc, CONF, SOFT_RESET, 1))
		if (!--timeout)
			break;
}

static void hdc_core_soft_reset(struct sn_udc *udc)
{
	/* controller reset*/
	if (hdc_read_bits(udc, PORTSC, POWER_CTL_REQ, 1) &&
	    hdc_read_bits(udc, PORTSC, POWER_REQ, 1)) {
		/* release vbus control before soft reset */
		hdc_write_bits(udc, PORTSC, POWER_CTL_REQ, 1, 0);

		_hdc_soft_reset(udc);

		hdc_set_vbus(udc, 1);

		hdc_write_bits(udc, PORTSC, POWER_CTL_REQ, 1, 1);
	} else {
		_hdc_soft_reset(udc);
	}

	mdelay(50);

	if (!_hdc_read_id_state(udc)) {
		hdc_write_bits(udc, PORTSC, POWER_CTL_REQ, 1, 1);
		hdc_set_vbus(udc, 1);
	}

	hdc_write_bits(udc, INTEN, OTG_INTEN, 1, 1);
}

static void hdc_set_dplus_pullup(struct sn_udc *udc, u8 enable)
{
	if (enable) {
		hdc_write_bits(udc, DEVC, PHYSUSP, 1, 0);
		hdc_write_bits(udc, DEVC, DISCONNECT, 1, 0);
	} else {
		hdc_write_bits(udc, DEVC, DISCONNECT, 1, 1);
		hdc_write_bits(udc, DEVC, PHYSUSP, 1, 1);
	}
}

static void hdc_detect(struct sn_udc *udc, u8 connect)
{
	if (connect) {
		hdc_write_bits(udc, OTGSTSFALL, VBUS_VLD_FEN, 1, 0);
		hdc_write_bits(udc, OTGSTSRISE, VBUS_VLD_REN, 1, 1);
	} else {
		hdc_write_bits(udc, OTGSTSRISE, VBUS_VLD_REN, 1, 0);
		hdc_write_bits(udc, OTGSTSFALL, VBUS_VLD_FEN, 1, 1);
	}
}

static void initialize_endpoint(struct sn_ep *ep)
{
	struct sn_udc *udc = ep->udc;
	unsigned id = idx_of(ep);

	/* initialize FIFO */
	hdc_write_ep(udc, id, INIT, 1, 1);
	hdc_write_ep(udc, id, STALL_CLR, 1, 1);
	hdc_write_ep(udc, id, TOGGLE_CLR, 1, 1);

	/* initialize endpoint control / status register */
	hdc_write_ep(udc, id, RDYI_INT_CLR, 1, 1);
	hdc_write_ep(udc, id, RDYO_EMPTY_INT_CLR, 1, 1);

	hdc_write_ep(udc, id, PING_INTEN, 1, 0);
	hdc_write_ep(udc, id, STALLED_INTEN, 1, 0);
	hdc_write_ep(udc, id, NACK_INTEN, 1, 0);

	if (id) {
		hdc_write_ep(udc, id, RDYO_EMPTY_INTEN, 1, 0);
		if (is_in(ep))
			hdc_write_ep(udc, id, RDYI_INTEN, 1, 0);
		else
			hdc_write_ep(udc, id, RDYI_INTEN, 1, 1);
	} else {
		hdc_write_ep(udc, id, RDYI_INTEN, 1, 1);
		hdc_write_ep(udc, id, RDYO_EMPTY_INTEN, 1, 1);
	}

	hdc_write_bits(udc, INTEN, DEV_EP_INTEN(id), 1, 1);
}

static void sn_req_done(struct sn_req *req, int status)
{
	struct sn_ep *ep = req->ep;
	struct sn_udc *udc = ep->udc;
	struct usb_request *r = &req->usb_req;

	list_del_init(&req->queue);

	if (r->status == -EINPROGRESS)
		r->status = status;
	else
		status = r->status;

	/* unmap request if DMA is present*/
	if (need_dma(req))
		usb_gadget_unmap_request(&udc->gadget, r, is_in(ep));

	if (!status)
		goto do_cb;

	dev_info(udc->dev, "%s done %p, status %d\n",
		ep->ep_usb.name, req, status);

	if (is_in(ep) && (udc->dmatx_busy || ep->status)) {
		if (udc->dmatx_busy == req) {
			hdc_write_bits(udc, INTEN, DMA_INTEN(DMA_IN), 1, 0);
			writel_relaxed(0, udc->dma_base + DMACA(DMA_IN));
			writel_relaxed(0, udc->dma_base + DMACB(DMA_IN));
			hdc_write_bits(udc, DMA_C(DMA_IN), DMA_START, 1, 0);
			writel_relaxed(0, udc->reg_base + DMATCI(DMA_IN));
			hdc_write_ep(udc, idx_of(ep), RDYO_EMPTY_INT_CLR, 1, 1);
			hdc_write_ep(udc, idx_of(ep), RDYI_INT_CLR, 1, 1);
			udc->dmatx_busy = NULL;
			ep->status = 0;
		}
	} else if (!is_in(ep) && udc->dmarx_busy) {
		if (udc->dmarx_busy == req) {
			hdc_write_bits(udc, INTEN, DMA_INTEN(DMA_OUT), 1, 0);
			writel_relaxed(0, udc->dma_base + DMACA(DMA_OUT));
			writel_relaxed(0, udc->dma_base + DMACB(DMA_OUT));
			hdc_write_bits(udc, DMA_C(DMA_OUT), DMA_START, 1, 0);
			writel_relaxed(0, udc->reg_base + DMATCI(DMA_OUT));
			hdc_write_ep(udc, idx_of(ep), RDYO_EMPTY_INT_CLR, 1, 1);
			hdc_write_ep(udc, idx_of(ep), RDYI_INT_CLR, 1, 1);
			udc->dmarx_busy = NULL;
		}
	}

do_cb:
	if (r->complete) {
		spin_unlock(&udc->lock);
		usb_gadget_giveback_request(&ep->ep_usb, r);
		spin_lock(&udc->lock);
	}
}

static inline unsigned ep_buff_addr(struct sn_ep *ep, int index)
{
	unsigned size = (ep->ep_usb.maxpacket + 3) & ~0x3;

	return ep->buf_offset + index * size;
}

static void sn_xfer_ep0_in(struct usb_request *r)
{
	struct sn_req *req = (struct sn_req *)r;
	struct sn_ep *ep = req->ep;
	struct sn_udc *udc = ep->udc;
	int i, count, bytes;
	u32 *src, *dst;
	void *buf;

	if (r->length == r->actual)
		req->zlp = false;

	buf = r->buf + r->actual;
	prefetch(buf);

	src = buf;
	dst = udc->reg_base + ep_buff_addr(ep, 0);

	bytes = min_t(u32, r->length - r->actual, ep->ep_usb.maxpacket);
	count = bytes / 4;
	for (i = 0; i < count; i++)
		*dst++ = *src++;
	memcpy_toio(dst, src, bytes % 4);

	hdc_write_bits(udc, EPCOUNT(0), APPCNT,	LEN_APPCNT, bytes);

	hdc_write_ep(udc, 0, BUFWR, 1, 1);
	if (bytes == ep->ep_usb.maxpacket)
		hdc_write_ep(udc, 0, RDYI_INTEN, 1, 1);
	else
		hdc_write_ep(udc, 0, RDYI_INTEN, 1, 0);

	r->actual += bytes;
}

static void sn_xfer_ep0_out(struct usb_request *r)
{
	struct sn_req *req = (struct sn_req *)r;
	struct sn_ep *ep = req->ep;
	struct sn_udc *udc = ep->udc;
	int i, count, bytes;
	u32 *src, *dst;
	void *buf;

	hdc_write_ep(udc, 0, RDYI_INTEN, 1, 1);
	hdc_write_ep(udc, 0, RDYO_EMPTY_INTEN, 1, 1);

	bytes = hdc_read_bits(udc, EPCOUNT(1), PHYCNT, LEN_PHYCNT);
	if (!bytes)
		return;

	bytes = min_t(u32, r->length - r->actual, bytes);

	buf = r->buf + r->actual;
	dst = buf;
	src = udc->reg_base + ep_buff_addr(ep, 1);

	count = bytes / 4;
	for (i = 0; i < count; i++)
		*dst++ = *src++;
	memcpy_fromio(dst, src, bytes % 4);

	hdc_write_ep(udc, 0, BUFRD, 1, 1);

	r->actual += bytes;

	if (r->actual == r->length) {

		sn_req_done(udc->ep0_active, 0);
		udc->ep0_active = NULL;

		udc->ep0req.usb_req.status = -EINPROGRESS;
		udc->ep0req.usb_req.actual = 0;
		udc->ep0req.usb_req.length = 0;
		udc->ep0_active = &udc->ep0req;
		sn_xfer_ep0_in(&udc->ep0req.usb_req);
	}
}

static void sn_xfer_dma(struct sn_req *req)
{
	struct usb_request *r = (struct usb_request *)req;
	u32 src, dst, bytes, dmaca, dmacb;
	struct sn_ep *ep = req->ep;
	struct sn_udc *udc = ep->udc;
	int dmach, zlp = 0;

	bytes = r->length - r->actual;
	if (is_in(ep)) {
		if (udc->dmatx_busy)
			return;
		dmach = DMA_IN;
		src = r->dma + r->actual;
		dst = udc->dma_offset + DMA_FIFO * dmach;
		/* Need to program ZLP now or after dma_done ? */
		zlp = (r->actual + bytes == r->length) && r->zero;
		hdc_write_bits(udc, DMA_C(dmach), DMAINT, 1, 0);
		dmacb = HDMACB_FD;
		udc->dmatx_busy = req;
	} else {
		if (udc->dmarx_busy)
			return;
		dmach = DMA_OUT;
		dst = r->dma + r->actual;
		src = udc->dma_offset + DMA_FIFO * dmach;
		hdc_write_bits(udc, DMA_C(dmach), SPR, 1, 0);
		dmacb = HDMACB_FS;
		udc->dmarx_busy = req;
		hdc_write_ep(udc, idx_of(ep), RDYI_INTEN, 1, 0);
	}
	ep->status++;

	hdc_write_bits(udc, DMA_C(dmach), BLKSIZE, LEN_BLKSIZE, BEAT);
	hdc_write_bits(udc, DMA_C(dmach), DMA_MODE, 1, 1); /* BLOCK mode */
	hdc_write_bits(udc, DMA_C(dmach), SEND_NULL, 1, zlp ? 1 : 0);
	hdc_write_bits(udc, DMA_C(dmach), DMAEP, LEN_DMAEP, idx_of(ep));
	writel_relaxed(bytes, udc->reg_base + DMATCI(dmach));

	writel_relaxed(BIT(DE) | BIT(PR), udc->dma_base + DMACR);
	writel_relaxed(src, udc->dma_base + DMASA(dmach));
	writel_relaxed(dst, udc->dma_base + DMADA(dmach));

	dmacb |= HDMACB_TT_2CYCLE | HDMACB_MS_BLOCK |
			HDMACB_EI | HDMACB_CI | HDMACB_TW_WORD;
	writel_relaxed(dmacb, udc->dma_base + DMACB(dmach));
	dmaca = UDC_DREQ | HDMACA_BT | (((bytes + BEAT - 1) / BEAT) - 1);
	writel_relaxed(dmaca | BIT(DE),
			udc->dma_base + DMACA(dmach));

	hdc_write_bits(udc, INTEN, DMA_INTEN(dmach), 1, 1);
	hdc_write_bits(udc, DMA_C(dmach), DMA_START, 1, 1);
}

/* Note: USB IN transfer */
static void dma_write(struct sn_udc *udc)
{
	struct list_head *udc_q = &udc->q_in;
	struct sn_req *req;

	if (list_empty(udc_q))
		return;

	req = list_first_entry(udc_q, typeof(*req), queue);
	if (!req)
		return;

	sn_xfer_dma(req);
}

static bool fifo_empty(struct sn_ep *ep)
{
	struct sn_udc *udc = ep->udc;
	u32 ctrl = readl_relaxed(udc->reg_base + EPCTRL(idx_of(ep)));

	return VALUE(ctrl, EMPTY, 1);
}

/* Note: USB OUT transfer */
static void dma_read(struct sn_udc *udc)
{
	struct list_head *udc_q = &udc->q_out;
	struct sn_req *req = NULL;
	u8 ep_checked[16] = {0};

	if (list_empty(udc_q) || udc->dmarx_busy)
		return;

	list_for_each_entry(req, udc_q, queue) {
		if (!ep_checked[idx_of(req->ep)]) {
			if (!fifo_empty(req->ep))
				return sn_xfer_dma(req);
			else
				ep_checked[idx_of(req->ep)] = 1;
		}
	}
}

static void sn_udc_nuke(struct sn_ep *ep, int status)
{
	struct sn_udc *udc = ep->udc;
	struct list_head *udc_q;
	struct sn_req *req, *t;

	if (is_in(ep))
		udc_q = &udc->q_in;
	else
		udc_q = &udc->q_out;

	ep->status = 0;
	list_for_each_entry_safe(req, t, udc_q, queue)
		if (req->ep == ep)
			sn_req_done(req, status);
}

static int sn_udc_ep_set_halt(struct usb_ep *_ep, int stall)
{
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_udc *udc = ep->udc;
	unsigned long flags;
	u32 timeout=0xffff;

	dev_dbg(udc->dev, "%s EP%d\n", stall ? "Stall" : "Unstall", idx_of(ep));
	spin_lock_irqsave(&udc->lock, flags);

	if (is_in(ep) && stall) {
		struct sn_req *req;

		list_for_each_entry(req, &udc->q_in, queue)
			if (req->ep == ep) {
				dev_dbg(udc->dev,
					"requests pending can't halt EP%d\n",
					idx_of(ep));
				spin_unlock_irqrestore(&udc->lock, flags);
				return -EAGAIN;
			}

		/* wait ep empty */
		while (!fifo_empty(ep)) {
			if (!--timeout)
				break;
			udelay(10);
		}
	}

	if (stall) {
		/* Stall the device.*/
		hdc_write_ep(udc, idx_of(ep), STALL_SET, 1, 1);
	} else {
		/* Unstall the device.*/
		hdc_write_ep(udc, idx_of(ep), STALL_CLR, 1, 1);
		if (idx_of(ep) != 0)
			/* Reset the toggle bit.*/
			hdc_write_ep(udc, idx_of(ep), TOGGLE_CLR, 1, 1);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

struct slot {
	unsigned offset;
	unsigned length;
};

static void find_free(struct sn_udc *udc, struct slot sp[])
{
	int i, f;
	struct sn_ep *ep = &udc->ep[0];
	unsigned cur = ep->buf_offset + ep->buf_length; /* EP0 has fixed buffer */
	unsigned diff = EP_SRAM_LOC + EP_SRAM_SZ - cur;

	/* iterate over max possible free slots */
	for (f = 0; f < SN_EP_NUM; f++) {
		/* iterate over EPs */
		for (i = 1; i < SN_EP_NUM; i++) {
			ep = &udc->ep[i];

			if (ep->buf_offset < cur)
				continue;

			if (ep->buf_offset == cur) {
				cur = ep->buf_offset + ep->buf_length;
				diff = EP_SRAM_LOC + EP_SRAM_SZ - cur;
				i = 1; /* rescan */
				continue;
			} else { //if (ep->buf_offset > cur)
				if (diff > ep->buf_offset - cur)
					diff = ep->buf_offset - cur;
			}
		}
		sp[f].offset = cur;
		sp[f].length = diff;

		if (diff == 0 || diff == EP_SRAM_LOC + EP_SRAM_SZ - cur)
			break;
		cur += diff;
	}

	if (f < SN_EP_NUM - 1)
		sp[f + 1].length = 0;
}

static int match_slot(struct slot sp[], unsigned buf_len)
{
	unsigned delta = EP_SRAM_SZ - buf_len;
	int f, ret = -1;

	/* iterate over max possible free slots */
	for (f = 0; f < SN_EP_NUM && sp[f].length; f++) {
		if (sp[f].length < buf_len)
			continue;

		if (sp[f].length == buf_len)
			return f;

		if (sp[f].length - buf_len < delta) {
			delta = sp[f].length - buf_len;
			ret = f;
		}
	}

	return ret;
}

static int __sn_udc_ep_enable(struct sn_ep *ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct sn_udc *udc = ep->udc;
	u32 maxpacket = usb_endpoint_maxp(desc) & 0x7ff;
	u32 ep_hiband = ((usb_endpoint_maxp(desc) >> 11) & 0x3) + 1;
	u32 xfer_type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	unsigned id = idx_of(ep);
	char *types[] = {"CTRL", "ISO", "BULK", "INTR"};
	struct slot s[SN_EP_NUM];
	int i;

	if (id) { /* Only for Non-Zero EPs */
		find_free(udc, s);
		i = match_slot(s, 2 * maxpacket);
		if (i < 0)
			return -ENOMEM;
		ep->buf_offset = s[i].offset;
		ep->buf_length = 2 * maxpacket;
	}

	ep->ep_usb.desc = desc;
	ep->ep_usb.maxpacket = maxpacket;

	hdc_write_ep(udc, id, ET, LEN_ET, xfer_type);
	hdc_write_ep(udc, id, DIR_IN, 1, is_in(ep));
	hdc_write_ep(udc, id, BNUM, LEN_BNUM, !!id);
	hdc_write_ep(udc, id, HIBAND, LEN_HIBAND, ep_hiband);
	hdc_write_bits(udc, EPCONF(id), BASE, LEN_BASE,
			(ep->buf_offset & 0x7fff) >> 2);
	hdc_write_bits(udc, EPCONF(id), SIZE, LEN_SIZE, maxpacket);
	hdc_write_bits(udc, EPCONF(id), COUNTIDX, LEN_COUNTIDX, id * 2);
	initialize_endpoint(ep);
	hdc_write_ep(udc, id, START, 1, 1);

	dev_dbg(udc->dev, "Enable EP%d %s_%s Max_%d\n", id, types[xfer_type],
					is_in(ep) ? "IN" : "OUT", maxpacket);
	return 0;
}

static int sn_udc_ep_enable(struct usb_ep *_ep,
			  const struct usb_endpoint_descriptor *desc)
{
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_udc *udc = ep->udc;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&udc->lock, flags);
	ret = __sn_udc_ep_enable(ep, desc);
	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static int sn_udc_ep_disable(struct usb_ep *_ep)
{
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_udc *udc = ep->udc;
	unsigned long flags;
	int id = idx_of(ep);

	spin_lock_irqsave(&udc->lock, flags);

	hdc_write_bits(udc, INTEN, DEV_EP_INTEN(id), 1, 0);
	sn_udc_nuke(ep, -ESHUTDOWN);
	hdc_set_epcmd_stop(udc, id);
	ep->ep_usb.desc = NULL;

	if (id) { /* Only for Non-Zero EPs */
		ep->buf_offset = 0;
		ep->buf_length = 0;
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static struct usb_request *sn_udc_ep_alloc_request(struct usb_ep *_ep,
						 gfp_t gfp_flags)
{
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_req *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(ep->udc->dev, "%s:not enough memory", __func__);
		return NULL;
	}

	req->ep = ep;
	INIT_LIST_HEAD(&req->queue);

	return &req->usb_req;
}

static void sn_udc_free_request(struct usb_ep *_ep, struct usb_request *r)
{
	kfree((struct sn_req *)r);
}

static void in_done(struct sn_ep *ep)
{
	struct sn_udc *udc = ep->udc;
	struct list_head *udc_q = &udc->q_in;
	struct usb_request *r;
	struct sn_req *req;
	u32 tc, tci;

	if (!ep->status)
		return;

	tc = readl_relaxed(udc->reg_base + DMATC(DMA_IN));
	tci = readl_relaxed(udc->reg_base + DMATCI(DMA_IN));
	if (tc == tci)
		ep->status--;

	list_for_each_entry(req, udc_q, queue)
		if (req->ep == ep)
			break;
	if (req->ep != ep) {
		dev_err(udc->dev, "%s:%d no req\n", __func__, __LINE__);
		return;
	}

	r = &req->usb_req;

	if (r->actual == r->length && !udc->dmatx_busy)
		sn_req_done(req, 0);

	/* program next transfer */
	dma_write(udc);
}

static void hdmac_irq(struct sn_udc *udc, int dmach)
{
	struct list_head *udc_q;
	struct usb_request *r;
	struct sn_req *req;
	struct sn_ep *ep;

	if (dmach == DMA_IN) {
		udc_q = &udc->q_in;
		req = udc->dmatx_busy;
	} else {
		udc_q = &udc->q_out;
		req = udc->dmarx_busy;
	}

	if (!req) {
		dev_err(udc->dev, "%s:DMA%d No Req\n", __func__, dmach);
		return;
	}

	list_del_init(&req->queue); /* remove from queue */

	/* Kill and clear DMA */
	writel_relaxed(0, udc->dma_base + DMACA(dmach));
	writel_relaxed(0, udc->dma_base + DMACB(dmach));

	r = (struct usb_request *)req;
	ep = req->ep;

	r->actual = readl_relaxed(udc->reg_base + DMATC(dmach));

	if (dmach == DMA_OUT) {
		udc->dmarx_busy = NULL;
		hdc_write_ep(udc, idx_of(ep), RDYI_INTEN, 1, 1);
		dma_read(udc);
	}

	sn_req_done(req, 0);

	if (dmach == DMA_IN) {
		udc->dmatx_busy = NULL;
		dma_write(udc);
	}
}

static int sn_udc_ep_queue(struct usb_ep *_ep, struct usb_request *r,
			   gfp_t gfp_flags)
{
	struct sn_req *req = (struct sn_req *)r;
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_udc *udc = ep->udc;
	unsigned long flags;
	int ret = 0;

	r->status = -EINPROGRESS;
	r->actual = 0;

	if (idx_of(ep) == 0) { /* always PIO for EP0 */
		spin_lock_irqsave(&udc->lock, flags);
		udc->ep0_active = req;
		if (udc->setup.bRequestType & USB_DIR_IN || !r->length) {
			if (r->zero || r->length % ep->ep_usb.maxpacket == 0)
				req->zlp = true;
			else
				req->zlp = false;
			sn_xfer_ep0_in(r);
		} else {
			sn_xfer_ep0_out(r);
		}
		spin_unlock_irqrestore(&udc->lock, flags);
		return 0;
	}

	if (need_dma(req))
		ret = usb_gadget_map_request(&udc->gadget,
				&req->usb_req, is_in(ep));
	if (ret)
		return ret;

	spin_lock_irqsave(&udc->lock, flags);

	if (is_in(ep)) {
		list_add_tail(&req->queue, &udc->q_in);
		dma_write(udc);
	} else {
		list_add_tail(&req->queue, &udc->q_out);
		dma_read(udc);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int sn_udc_ep_dequeue(struct usb_ep *_ep, struct usb_request *r)
{
	struct sn_ep *ep = (struct sn_ep *)_ep;
	struct sn_req *t, *req = (struct sn_req *)r;
	struct sn_udc *udc = ep->udc;
	struct list_head *udc_q;
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	if (is_in(ep))
		udc_q = &udc->q_in;
	else
		udc_q = &udc->q_out;

	/* Make sure it's actually queued on this endpoint */
	list_for_each_entry(t, udc_q, queue)
		if (t == req && req->ep == ep) {
			sn_req_done(req, -ECONNRESET);
			ret = 0;
			break;
		}

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static const struct usb_ep_ops sn_ep_ops = {
	.enable		= sn_udc_ep_enable,
	.alloc_request	= sn_udc_ep_alloc_request,
	.queue		= sn_udc_ep_queue,
	.dequeue	= sn_udc_ep_dequeue,
	.free_request	= sn_udc_free_request,
	.set_halt	= sn_udc_ep_set_halt,
	.disable	= sn_udc_ep_disable,
};

static int sn_udc_get_frame(struct usb_gadget *gadget)
{
	struct sn_udc *udc = (struct sn_udc *)gadget;
	unsigned long flags;
	int frame;

	dev_dbg(udc->dev, "%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&udc->lock, flags);
	frame = hdc_read_bits(udc, TSTAMP, TIMESTAMP, LEN_TIMESTAMP);
	spin_unlock_irqrestore(&udc->lock, flags);

	return frame;
}

static int sn_udc_wakeup(struct usb_gadget *gadget)
{
	struct sn_udc *udc = (struct sn_udc *)gadget;
	int status = -EPROTO;
	unsigned long flags;

	dev_dbg(udc->dev, "%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&udc->lock, flags);
	if (udc->remote_wkp) {
		hdc_write_bits(udc, DEVC, REQRESUME, 1, 1);
		status = 0;
	}
	spin_unlock_irqrestore(&udc->lock, flags);

	return status;
}

static void enable_host_connect(struct sn_udc *udc, u8 enable)
{
	int i;

	if (enable) {
		hdc_write_bits(udc, MODE, HOST_EN, 1, 0);
		hdc_write_bits(udc, MODE, DEV_EN, 1, 0);
		hdc_write_bits(udc, FADDR, FUNC_ADDR, LEN_FUNC_ADDR, 0);
		hdc_write_bits(udc, FADDR, DEV_CONFIGURED, 1, 0);

		for (i = 0; i < SN_EP_NUM; i++) {
			writel_relaxed(i == 0 ? 0x500 : 0x400,
						udc->reg_base + EPCTRL(i));
			writel_relaxed(0x0, udc->reg_base + EPCONF(i));
			writel_relaxed(0x0, udc->reg_base + EPCOUNT(i * 2));
			writel_relaxed(0x0, udc->reg_base + EPCOUNT(i * 2 + 1));
		}

		/* wait PHY reset release */
		for (i = 0xffff; i && hdc_read_bits(udc, DEVS, PHYRESET,1); i--)
			;

		/* enable device mode */
		hdc_write_bits(udc, MODE, HOST_EN, 1, 0);
		hdc_write_bits(udc, MODE, DEV_EN, 1, 1);
		hdc_write_bits(udc, MODE, DEV_INT_MODE, 1, 1);
		hdc_write_bits(udc, MODE, ADDR_LD_MODE, 1, 1);

		/* enable interrupt factor */
		hdc_write_bits(udc, INTEN, DEV_INTEN, 1, 1);
		hdc_write_bits(udc, INTEN, OTG_INTEN, 1, 1);
		hdc_write_bits(udc, INTEN, PHY_ERR_INTEN, 1, 1);
		hdc_write_bits(udc, DEVS, SUSPENDE_INT, 1, 0);
		hdc_write_bits(udc, DEVS, SUSPENDB_INT, 1, 0);
		hdc_write_bits(udc, DEVC, SUSPENDE_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, SUSPENDB_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, SETUP_INTEN, 1, 1);
		hdc_write_bits(udc, DEVC, STATUS_OK_INTEN, 1, 1);
		hdc_write_bits(udc, DEVC, STATUS_NG_INTEN, 1, 1);
		hdc_write_bits(udc, DEVC, USBRSTE_INTEN, 1, 1);
		hdc_write_bits(udc, DEVC, USBRSTB_INTEN, 1, 1);

		/* setup EP0 */
		__sn_udc_ep_enable(&udc->ep[0], &ep0_desc);

		hdc_set_dplus_pullup(udc, 1);
	} else {
		/* disable interrupt factor */
		hdc_write_bits(udc, INTEN, DEV_INTEN, 1, 0);
		hdc_write_bits(udc, INTEN, PHY_ERR_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, SUSPENDE_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, SUSPENDB_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, SETUP_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, USBRSTE_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, USBRSTB_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, STATUS_OK_INTEN, 1, 0);
		hdc_write_bits(udc, DEVC, STATUS_NG_INTEN, 1, 0);
		hdc_write_bits(udc, FADDR, DEV_CONFIGURED, 1, 0);

		/* clear interrupt factor */
		hdc_write_bits(udc, INTS, PHY_ERR_INT, 1, 0);
		hdc_write_bits(udc, INTS, CMD_INT, 1, 0);
		hdc_write_bits(udc, DEVS, SUSPENDE_INT, 1, 0);
		hdc_write_bits(udc, DEVS, SUSPENDB_INT, 1, 0);
		hdc_write_bits(udc, DEVS, SOF_INT, 1, 0);
		hdc_write_bits(udc, DEVS, SETUP_INT, 1, 0);
		hdc_write_bits(udc, DEVS, USBRSTE_INT, 1, 0);
		hdc_write_bits(udc, DEVS, USBRSTB_INT, 1, 0);
		hdc_write_bits(udc, DEVS, STATUS_OK_INT, 1, 0);
		hdc_write_bits(udc, DEVS, STATUS_NG_INT, 1, 0);

		hdc_set_dplus_pullup(udc, 0);
	}
}

static int sn_udc_pullup(struct usb_gadget *gadget, int on)
{
	struct sn_udc *udc = (struct sn_udc *)gadget;
	unsigned long flags;

	dev_dbg(udc->dev, "%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&udc->lock, flags);
	hdc_detect(udc, on);
	enable_host_connect(udc, !on);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int sn_udc_start(struct usb_gadget *gadget,
			struct usb_gadget_driver *driver)
{
	struct sn_udc *udc = (struct sn_udc *)gadget;
	unsigned long flags;

	dev_dbg(udc->dev, "%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&udc->lock, flags);

	/* hook up the driver */
	udc->driver = driver;
	udc->gadget.speed = driver->max_speed;

	if (udc->buspowered)
		dev_dbg(udc->dev, "Setup Bus-Powered\n");
	else
		dev_dbg(udc->dev, "Setup Self-Powered\n");

	hdc_write_bits(udc, OTGSTSC, ID_C, 1, 0);
	hdc_core_soft_reset(udc);
	hdc_write_bits(udc, OTGC, DM_PULL_DOWN, 1, 0);
	hdc_write_bits(udc, OTGC, DP_PULL_DOWN, 1, 0);
	hdc_write_bits(udc, OTGSTSC, TMROUT_C, 1, 0);
	hdc_write_bits(udc, OTGSTSC, VBUS_VLD_C, 1, 0);
	hdc_write_bits(udc, OTGSTSRISE, TMROUT_REN, 1, 0);
	hdc_write_bits(udc, OTGTC, START_TMR, 1, 0);
	hdc_write_bits(udc, OTGT, TMR_INIT_VAL,
		       LEN_TMR_INIT_VAL, 0);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int sn_udc_stop(struct usb_gadget *gadget)
{
	struct sn_udc *udc = (struct sn_udc *)gadget;
	unsigned long flags;
	int i;

	dev_dbg(udc->dev, "%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&udc->lock, flags);

	for (i = SN_EP_NUM; i; i--)
		sn_udc_nuke(&udc->ep[i - 1], -ESHUTDOWN);

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->driver = NULL;
	udc->remote_wkp = 0;

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static const struct usb_gadget_ops sn_udc_ops = {
	.get_frame	= sn_udc_get_frame,
	.wakeup		= sn_udc_wakeup,
	.pullup		= sn_udc_pullup,
	.udc_start	= sn_udc_start,
	.udc_stop	= sn_udc_stop,
};

static void handle_ep0_setup(struct sn_udc *udc)
{
	struct sn_ep *ep, *ep0 = &udc->ep[0];
	struct usb_ctrlrequest setup;
	int ret, bytes, status;
	u32 val;
	u16 w_value, w_index;

	/* clear ReadyOut interrupt */
	hdc_write_ep(udc, 0, RDYO_EMPTY_INT_CLR, 1, 1);

	/* read how many bytes are in EP0_Out buffer */
	bytes = hdc_read_bits(udc, EPCOUNT(1), PHYCNT, LEN_PHYCNT);
	if (bytes != sizeof(setup)) {
		dev_err(udc->dev, "%s:%d Bad Length Setup Packet\n",
						__func__, __LINE__);
		goto stall_ep;
	}

	/* copy from EP0_Out buffer */
	memcpy_fromio((void *) &setup, udc->reg_base + ep0->buf_offset +
					ep0->ep_usb.maxpacket, bytes);
	udc->setup = setup;

	/* mark the buffer as read */
	hdc_write_ep(udc, 0, BUFRD, 1, 1);

	/* Since word-sized fields are stored in le16, we have to decode it */
	w_value = le16_to_cpu(setup.wValue);
	w_index = le16_to_cpu(setup.wIndex);

	if ((setup.bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS)
		goto class_request;

	switch (setup.bRequest) {
	case USB_REQ_CLEAR_FEATURE:
		/* remote wakeup */
		if (w_value == USB_DEVICE_REMOTE_WAKEUP) {
			if ((setup.bRequestType & USB_RECIP_MASK) ==
							USB_RECIP_DEVICE) {
				/* disable remote wakeup */
				hdc_write_bits(udc, DEVC, RMTWKUP, 1, 0);
			} else
				goto stall_ep;
		/* endpoint halt */
		} else if (w_value == USB_ENDPOINT_HALT) {
			if ((setup.bRequestType & USB_RECIP_MASK) ==
							USB_RECIP_ENDPOINT) {
				ep = &udc->ep[w_index & USB_ENDPOINT_NUMBER_MASK];
				spin_unlock(&udc->lock);
				sn_udc_ep_set_halt(&ep->ep_usb, 0);
				spin_lock(&udc->lock);
			} else
				goto stall_ep;
		} else
			goto stall_ep;
		udc->ep0req.usb_req.status = -EINPROGRESS;
		udc->ep0req.usb_req.actual = 0;
		udc->ep0req.usb_req.length = 0;
		udc->ep0_active = &udc->ep0req;
		sn_xfer_ep0_in(&udc->ep0req.usb_req);
		break;

	case USB_REQ_SET_FEATURE:
		/* test mode */
		if (w_value == USB_DEVICE_TEST_MODE) {
			udc->testmode = (w_index >> 8) & 0xFF;
		/* remote wakeup */
		} else if (w_value == USB_DEVICE_REMOTE_WAKEUP) {
			if ((setup.bRequestType & USB_RECIP_MASK) ==
							USB_RECIP_DEVICE) {
				/* enable remote wakeup */
				hdc_write_bits(udc, DEVC, RMTWKUP, 1, 1);
			} else {
				goto stall_ep;
			}
		/* endpoint halt */
		} else if (w_value == USB_ENDPOINT_HALT) {
			if ((setup.bRequestType & USB_RECIP_MASK) ==
								USB_RECIP_ENDPOINT) {
				ep = &udc->ep[w_index & USB_ENDPOINT_NUMBER_MASK];
				spin_unlock(&udc->lock);
				sn_udc_ep_set_halt(&ep->ep_usb, 1);
				spin_lock(&udc->lock);
			} else {
				goto stall_ep;
			}
		} else {
			goto stall_ep;
		}
		udc->ep0req.usb_req.status = -EINPROGRESS;
		udc->ep0req.usb_req.actual = 0;
		udc->ep0req.usb_req.length = 0;
		udc->ep0_active = &udc->ep0req;
		sn_xfer_ep0_in(&udc->ep0req.usb_req);
		break;

	case USB_REQ_SET_ADDRESS:
		/* set address value */
		hdc_write_bits(udc, FADDR, FUNC_ADDR, LEN_FUNC_ADDR,
				w_value & 0xff);
		udc->ep0req.usb_req.status = -EINPROGRESS;
		udc->ep0req.usb_req.actual = 0;
		udc->ep0req.usb_req.length = 0;
		udc->ep0_active = &udc->ep0req;
		sn_xfer_ep0_in(&udc->ep0req.usb_req);
		break;

	case USB_REQ_GET_STATUS:
		switch (setup.bRequestType & USB_RECIP_MASK) {
		case USB_RECIP_DEVICE:
			status = hdc_read_bits(udc, DEVC, RMTWKUP, 1) ?
					(1 << USB_DEVICE_REMOTE_WAKEUP) : 0;
			break;
		case USB_RECIP_ENDPOINT:
			ep = &udc->ep[w_index & USB_ENDPOINT_NUMBER_MASK];
			if (w_index & USB_DIR_IN) {
				if (!is_in(ep))
					goto stall_ep;
			} else {
				if (is_in(ep))
					goto stall_ep;
			}
			val = readl_relaxed(udc->reg_base + EPCTRL(idx_of(ep)));
			status = ((val >> 12) & 0x1) ?
					(1 << USB_ENDPOINT_HALT) : 0;
			break;
		case USB_RECIP_INTERFACE:
			if (udc->driver->setup(&udc->gadget, &setup) < 0)
				goto stall_ep;
			return;
		default:
			goto stall_ep;
		}
		*(u16 *)udc->ep0req.usb_req.buf = cpu_to_le16(status);
		udc->ep0req.usb_req.status = -EINPROGRESS;
		udc->ep0req.usb_req.actual = 0;
		udc->ep0req.usb_req.length = 2;
		udc->ep0_active = &udc->ep0req;
		sn_xfer_ep0_in(&udc->ep0req.usb_req);
		break;

	default: /* some STD call */
class_request:
		spin_unlock(&udc->lock);
		if (udc->driver)
			ret = udc->driver->setup(&udc->gadget, &setup);
		else
			ret = -ESHUTDOWN;
		spin_lock(&udc->lock);
		if (ret < 0)
			goto stall_ep;
		if (setup.bRequest == USB_REQ_SET_CONFIGURATION)
			hdc_write_bits(udc, FADDR, DEV_CONFIGURED, 1,
					!!(w_value & 0xff));
		break;
	}
	return;

stall_ep:
	dev_err(udc->dev, "%s:%d STALL EP0\n", __func__, __LINE__);
	spin_unlock(&udc->lock);
	sn_udc_ep_set_halt(&ep0->ep_usb, 1);
	spin_lock(&udc->lock);
	return;
}

static void sn_udc_isr_todo(struct sn_udc *udc, const char *name)
{
	dev_dbg(udc->dev, "IRQ_%s\n", name);
}

static void sn_udc_setup_done(struct sn_udc *udc, int status)
{
	struct usb_request *r;

	if (!udc->ep0_active)
		return;

	/* set test mode */
	if (status == 0 && udc->testmode) {
		switch (udc->testmode) {
		/* test J */
		case 0x1 :
			hdc_write_bits(udc, TESTC, TESTJ, 1, 1);
			break;
		/* test K */
		case 0x2 :
			hdc_write_bits(udc, TESTC, TESTK, 1, 1);
			break;
		/* Test SEO NAK */
		case 0x3 :
			hdc_write_bits(udc, TESTC, TESTSE0NAK, 1, 1);
			break;
		/* Test Packet */
		case 0x4 :
			hdc_write_bits(udc, TESTC, TESTP, 1, 1);
			break;
		default :
			break;
		}
		udc->testmode = 0;
	}

	r = &udc->ep0_active->usb_req;

	if (r->actual == r->length) {
		sn_req_done(udc->ep0_active, status);
		udc->ep0_active = NULL;
	} else {
		dev_err(udc->dev, "%s:%d\t %d != %d\n",
			__func__, __LINE__, r->actual, r->length);
	}
}

static void sn_udc_ep0_data(struct sn_udc *udc)
{
	struct usb_request *r;
	u32 ctrl;

	if (!udc->ep0_active) {
		hdc_write_ep(udc, 0, RDYI_INT_CLR, 1, 1);
		hdc_write_ep(udc, 0, RDYO_EMPTY_INT_CLR, 1, 1);
		dev_dbg(udc->dev, "%s:%d !EP0\n", __func__, __LINE__);
		return;
	}

	r = &udc->ep0_active->usb_req;
	ctrl = readl_relaxed(udc->reg_base + EPCTRL(0));

	if (VALUE(ctrl, RDYI_RDY_INT, 1) && VALUE(ctrl, READY_INTEN, 1)) {
		hdc_write_ep(udc, 0, RDYI_INT_CLR, 1, 1);

		if (!(udc->setup.bRequestType & USB_DIR_IN)) {
			dev_err(udc->dev, "%s:%d !IN\n", __func__, __LINE__);
			return;
		}

		if (r->actual < r->length || udc->ep0_active->zlp)
			sn_xfer_ep0_in(r);

	} else if (VALUE(ctrl, EMPTY_INT, 1) && VALUE(ctrl, EMPTY_INTEN, 1)) {
		hdc_write_ep(udc, 0, RDYO_EMPTY_INT_CLR, 1, 1);

		if (udc->setup.bRequestType & USB_DIR_IN) {
			dev_err(udc->dev, "%s:%d !OUT\n", __func__, __LINE__);
			return;
		}

		sn_xfer_ep0_out(r);
	} else {
		dev_err(udc->dev, "%s:%d %08x\n", __func__, __LINE__, ctrl);
	}
}

static void sn_udc_isr_ep(struct sn_udc *udc, int id)
{
	struct sn_ep *ep = &udc->ep[id];
	u32 ctrl = readl_relaxed(udc->reg_base + EPCTRL(id));

	if (id == 0)
		return sn_udc_ep0_data(udc);

	if (VALUE(ctrl, EMPTY_INT, 1))
		hdc_write_ep(udc, id, RDYO_EMPTY_INT_CLR, 1, 1);

	if (VALUE(ctrl, RDYI_RDY_INT, 1))
		hdc_write_ep(udc, id, RDYI_INT_CLR, 1, 1);

	if (is_in(ep))
		in_done(ep);
	else
		dma_read(udc);
}

static void sn_udc_isr_rste(struct sn_udc *udc)
{
	/* Just re-read the speed at reset */
	udc->gadget.speed = hdc_read_bits(udc, DEVS,
				CRTSPEED, 1) == FULL_SPEED ?
				USB_SPEED_FULL : USB_SPEED_HIGH;
}

static irqreturn_t sn_udc_irq(int irq, void *_udc)
{
	struct sn_udc *udc = _udc;
	u32 ints, inten, otgstsc, devs, devc;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	int i;

	spin_lock_irqsave(&udc->lock, flags);

	ints = readl_relaxed(udc->reg_base + INTS);
	inten = readl_relaxed(udc->reg_base + INTEN);
	otgstsc = readl_relaxed(udc->reg_base + OTGSTSC);
	devs = readl_relaxed(udc->reg_base + DEVS);
	devc = readl_relaxed(udc->reg_base + DEVC);

	/* bus connect interrupt */
	if (VALUE(otgstsc, VBUS_VLD_C, 1)) {
		hdc_write_bits(udc, OTGSTSC, VBUS_VLD_C, 1, 0);
		if (VALUE(inten, OTG_INTEN, 1) &&
			VALUE(ints, OTG_INT, 1)) {

			for (i = SN_EP_NUM; i; i--)
				sn_udc_nuke(&udc->ep[i - 1], -ECONNRESET);

			spin_unlock_irqrestore(&udc->lock, flags);

			if (!is_bus_connected(udc) && udc->driver)
				udc->driver->disconnect(&udc->gadget);

			sn_udc_pullup((struct usb_gadget *)udc,
						!is_bus_connected(udc));

			spin_lock_irqsave(&udc->lock, flags);
			ret = IRQ_HANDLED;
		}
	}

	/* bus reset begin interrupt */
	if (VALUE(devs, USBRSTB_INT, 1)) {
		hdc_write_bits(udc, DEVS, USBRSTB_INT, 1, 0);
		if (udc->driver) {
			spin_unlock_irqrestore(&udc->lock, flags);
			usb_gadget_udc_reset(&udc->gadget, udc->driver);
			spin_lock_irqsave(&udc->lock, flags);
		}
		devs = 0;
		ints = 0;
		ret = IRQ_HANDLED;
	}

	/* bus reset end interrupt */
	if (VALUE(devs, USBRSTE_INT, 1)) {
		hdc_write_bits(udc, DEVS, USBRSTE_INT, 1, 0);
		sn_udc_isr_rste(udc);
		ret = IRQ_HANDLED;
	}

	/* StatusNG interrupt */
	if (VALUE(devs, STATUS_NG_INT, 1)) {
		hdc_write_bits(udc, DEVS, STATUS_NG_INT, 1, 0);
		if (VALUE(devc, STATUS_NG_INTEN, 1)) {
			dev_err(udc->dev, "IRQ_SETUP_NG\n");
			sn_udc_setup_done(udc, -EPROTO);
			ret = IRQ_HANDLED;
		}
	}

	/* StatusOK interrupt */
	if (VALUE(devs, STATUS_OK_INT, 1)) {
		hdc_write_bits(udc, DEVS, STATUS_OK_INT, 1, 0);
		if (VALUE(devc, STATUS_OK_INTEN, 1)) {
			sn_udc_setup_done(udc, 0);
			ret = IRQ_HANDLED;
		}
	}

	/* suspend begin interrupt */
	if (VALUE(devs, SUSPENDB_INT, 1)) {
		hdc_write_bits(udc, DEVS, SUSPENDB_INT, 1, 0);
		if (VALUE(devc, SUSPENDB_INTEN, 1)) {
			sn_udc_isr_todo(udc, "SUSPB");
			ret = IRQ_HANDLED;
		}
	}

	/* suspend end interrupt */
	if (VALUE(devs, SUSPENDE_INT, 1)) {
		hdc_write_bits(udc, DEVS, SUSPENDE_INT, 1, 0);
		if (VALUE(devc, SUSPENDE_INTEN, 1)) {
			sn_udc_isr_todo(udc, "SUSPE");
			ret = IRQ_HANDLED;
		}
	}

	/* PHY hung-up interrupt */
	if (VALUE(ints, PHY_ERR_INT, 1)) {
		hdc_write_bits(udc, INTS, PHY_ERR_INT, 1, 0);
		if (VALUE(inten, PHY_ERR_INTEN, 1)) {
			sn_udc_isr_todo(udc, "PHYERR");
			ret = IRQ_HANDLED;
		}
	}

	/* CMD interrupt */
	if (VALUE(ints, CMD_INT, 1)) {
		hdc_write_bits(udc, INTS, CMD_INT, 1, 0);
		if (VALUE(inten, CMD_INTEN, 1)) {
			sn_udc_isr_todo(udc, "CMD");
			ret = IRQ_HANDLED;
		}
	}

	/* SOF interrupt */
	if (VALUE(devs, SOF_INT, 1)) {
		hdc_write_bits(udc, DEVS, SOF_INT, 1, 0);
		if (VALUE(devc, SOF_INTEN, 1)) {
			sn_udc_isr_todo(udc, "SOF");
			ret = IRQ_HANDLED;
		}
	}

	/* SETUP interrupt */
	if (VALUE(devs, SETUP_INT, 1)) {
		hdc_write_bits(udc, DEVS, SETUP_INT, 1, 0);
		if (VALUE(devc, SETUP_INTEN, 1)) {
			handle_ep0_setup(udc);
			ret = IRQ_HANDLED;
		}
	}

	for (i = 0; i < 2; i++) {
		if (VALUE(ints, DMA_INT(i), 1)) {
			hdc_write_bits(udc, INTS, DMA_INT(i), 1, 0);
			if (VALUE(inten, DMA_INTEN(i), 1)) {
				hdmac_irq(udc, i);
				ret = IRQ_HANDLED;
			}
		}
	}

	/* EP interrupt --- Page 161 for EPCTRL (1-15) */
	for (i = 0; i < SN_EP_NUM; i++)
		if (VALUE(ints, DEV_EP_INT(i), 1)) {
			if (VALUE(inten, DEV_EP_INTEN(i), 1)) {
				sn_udc_isr_ep(udc, i);
				ret = IRQ_HANDLED;
			}
		}

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

/* Match table for of_platform binding */
static const struct of_device_id usb_of_match[] = {
	{ .compatible = "socionext,usb2m9m,f_usb20dc_lfp",
	  .data = (void *)FHDC_VARIANT_M9M },
#if 0
	{ .compatible = "socionext,f_usb20hdc_drc_lap", },
	{ .compatible = "socionext,f_usb20hdc_drc", },
	{ .compatible = "socionext,usb-device", },
#endif
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, usb_of_match);


static int sn_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct resource *res;
	int irq, ret, ep_id, err;
	struct sn_udc *udc;
	void __iomem *base;

	const struct of_device_id *of_id =
		of_match_device(usb_of_match, &pdev->dev);

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	udc->dev = dev;
	udc->dmarx_busy = NULL;
	udc->dmatx_busy = NULL;

	udc->variant = (enum fhdc_variants)of_id->data;

	udc->ep0req.ep = &udc->ep[0];
	udc->ep0req.usb_req.buf = udc->ep0buf;
	INIT_LIST_HEAD(&udc->ep0req.queue);

	/* hdc reg base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "udc");

	/* dma buffer base */
	if (udc->variant == FHDC_VARIANT_M9M) 
		udc->dma_offset = res->start + 0x10000;

	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	udc->reg_base = base;

	/* hdmac reg base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	udc->dma_base = devm_ioremap_resource(dev, res);

	rst = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(dev, "unable to get RST\n");
		rst = NULL;
	} else {
		err = reset_control_deassert(rst);
		if (err)
			dev_err(dev, "unable to do RST\n");
	}

#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
#endif
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "unable to get irq\n");
		return irq;
	}
	ret = devm_request_irq(dev, irq, sn_udc_irq, 0,
			       dev_name(dev), udc);
	if (ret < 0) {
		dev_err(dev, "unable to request irq %d", irq);
		goto fail;
	}

	if (of_get_property(dev->of_node, "bus-powered", NULL))
		udc->buspowered = 1;
	else
		udc->buspowered = 0;

	udc->gadget.ops = &sn_udc_ops;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.ep0 = &udc->ep[0].ep_usb;
	udc->gadget.name = driver_name;

	spin_lock_init(&udc->lock);
	INIT_LIST_HEAD(&udc->q_in);
	INIT_LIST_HEAD(&udc->q_out);
	INIT_LIST_HEAD(&udc->gadget.ep_list);

	for (ep_id = 0; ep_id < SN_EP_NUM; ep_id++) {
		struct sn_ep *ep = &udc->ep[ep_id];

		if (ep_id == 0) {
			usb_ep_set_maxpacket_limit(&ep->ep_usb, EP0_MAX_PACKET);
			ep->buf_offset = EP_SRAM_LOC;
			ep->buf_length = EP0_MAX_PACKET * 2;
			ep->ep_usb.caps.type_control = true;
		} else {
			usb_ep_set_maxpacket_limit(&ep->ep_usb, EPn_MAX_PACKET);
			list_add_tail(&ep->ep_usb.ep_list,
					&udc->gadget.ep_list);
			ep->buf_offset = 0;
			ep->buf_length = 0;
			ep->ep_usb.caps.type_iso = true;
			ep->ep_usb.caps.type_bulk = true;
			ep->ep_usb.caps.type_int = true;
		}

		ep->ep_usb.caps.dir_in = true;
		ep->ep_usb.caps.dir_out = true;

		snprintf(ep->name, EPNAME_SIZE, "ep%d", ep_id);
		ep->ep_usb.name = ep->name;
		ep->ep_usb.ops = &sn_ep_ops;
		ep->udc = udc;
	}

	platform_set_drvdata(pdev, udc);

	for (ret = 0; ret < 4; ret++)
		clk_prepare_enable(of_clk_get(dev->of_node, ret));

	ret = usb_add_gadget_udc(dev, &udc->gadget);
	if (ret)
		goto fail;

	dev_dbg(dev, "%s probed with %d endpoints\n", driver_name, SN_EP_NUM);
	return 0;
fail:
#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_put_sync(&pdev->dev);
#endif
	dev_err(dev, "probe failed, %d\n", ret);
	return ret;
}

static int sn_udc_remove(struct platform_device *pdev)
{
	struct sn_udc *udc = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&udc->gadget);

#ifdef CONFIG_PM_WARP
	/* for CONFIG PM WARP */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM_WARP
/* warp suspend */
static int sn_udc_warp_suspend(struct device *dev)
{
	pm_runtime_put_sync(dev);
        return 0;
}

/* for warp resume */
static int sn_udc_warp_resume(struct device *dev)
{
        struct sn_udc *udc = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);

        /* soft reset */
	hdc_core_soft_reset(udc);

        /* enable device mode */
        hdc_write_bits(udc, MODE, HOST_EN, 1, 0);
        hdc_write_bits(udc, MODE, DEV_EN, 1, 1);
        hdc_write_bits(udc, MODE, DEV_INT_MODE, 1, 1);
        hdc_write_bits(udc, MODE, ADDR_LD_MODE, 1, 1);

        /* set vbus rise & fall interrupt enable */
        hdc_write_bits(udc, OTGSTSFALL, VBUS_VLD_FEN, 1, 1);
        hdc_write_bits(udc, OTGSTSRISE, VBUS_VLD_REN, 1, 1);

        return 0;
}

static SIMPLE_DEV_PM_OPS(sn_udc_pm_ops, sn_udc_warp_suspend, sn_udc_warp_resume);
#endif /* CONFIG_PM_WARP */


static struct platform_driver sn_udc_driver = {
	.driver = {
		.name = driver_name,
		.of_match_table = usb_of_match,
	},
	.probe = sn_udc_probe,
	.remove = sn_udc_remove,
};
module_platform_driver(sn_udc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Socionext udc driver");
MODULE_AUTHOR("Jassi Brar <jassisinghbrar@gmail.com>");
