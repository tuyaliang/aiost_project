/*
 * linux/drivers/usb/host/f_usb20hdc-hcd.c - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) SOCIONEXT ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2012-2015 SOCIONEXT SEMICONDUCTOR LIMITED.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <asm/unaligned.h>

#include "f_usb20hdc-hcd.h"

/* only 0x07ff bits of wMaxPacketSize are for packet size... */
#define max_packet(max_packet_size) ((max_packet_size) & 0x07ff)
#define is_highbandwidth(max_packet_size) ((max_packet_size) & 0x1800)

static const struct endpont_cb ep_config_data[HDC_HCD_MAX_EP] = {
	/* endpoint 0 */
	[0] = {
		.name						= "ep0",
		.hs_maxpacket					= 64,
		.fs_maxpacket					= 64,
		.buffer_size					= 64,
		.buffers					= 2,
		.pio_auto_change				= 0,
		.trans_end_timing				= 0,
		.dma_ch						= -1,
	},
	/* endpoint 1 */
	[1] = {
		.name						= "ep1-bulk",
		.hs_maxpacket					= 512,
		.fs_maxpacket					= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= 0,
	},
	/* endpoint 2 */
	[2] = {
		.name						= "ep2-bulk",
		.hs_maxpacket					= 512,
		.fs_maxpacket					= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= 1,
	},
	/* endpoint 3 */
	[3] = {
		.name						= "ep3-int",
		.hs_maxpacket					= 512,
		.fs_maxpacket					= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= -1,
	},
	/* endpoint 4 */
	[4] = {
		.name						= "ep4-int",
		.hs_maxpacket					= 512,
		.fs_maxpacket					= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= -1,
	},
	/* endpoint 5 */
	[5] = {
		.name						= "ep5-iso",
		.hs_maxpacket					= 1024,
		.fs_maxpacket					= 1023,
		.buffer_size					= 1024,
		.buffers					= 1,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= -1,
	},
	/* endpoint 6 */
	[6] = {
		.name						= "ep6-iso",
		.hs_maxpacket					= 1024,
		.fs_maxpacket					= 1023,
		.buffer_size					= 1024,
		.buffers					= 1,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= -1,
	},
	/* endpoint 7 */
	[7] = {
		.name						= "ep7-int",
		.hs_maxpacket					= 512,
		.fs_maxpacket					= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change				= 1,
		.trans_end_timing				= 1,
		.dma_ch						= -1,
	},
};

#ifdef DEBUG
static void dbg_print_connection(struct device *pdev, void __iomem *base)
{
	if (!hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1)) {
		dev_dbg(pdev, "%s() no connection\n", __func__);
		dev_dbg(pdev, "%s() connection change bit %d\n", __func__,
			hdc_read_bits(base, HDC_PORTSTSC, BIT_CONNECTION_C, 1));
		return;
	}

	dev_dbg(pdev, "%s() connection bit %d line state %d\n",
		__func__,
		hdc_read_bits(base, HDC_PORTSC, BIT_LS_RHS, 1),
		hdc_get_linestate(base));
	/* Error detection */
	if (hdc_read_bits(base, HDC_PORTSC, BIT_LS_RHS, 1)) {
		/* low speed device connection analysis */
		if (LINESTATE_DP_LOW_DM_HIGH != hdc_get_linestate(base))
			dev_dbg(pdev, "%s() speed detection error\n",
				__func__);
	} else {
		/* non-low speed device connection analysis */
		if (LINESTATE_DP_HIGH_DM_LOW != hdc_get_linestate(base))
			dev_dbg(pdev, "%s() speed detection error\n",
				__func__);
	}

	dev_dbg(pdev, "%s() connection change bit %d\n", __func__,
		hdc_read_bits(base, HDC_PORTSTSC, BIT_CONNECTION_C, 1));
}
#endif

static inline u32 hdc_readl_cached(void __iomem *base, u32 offset)
{
	u32 reg;

	reg = __raw_readl(base + offset);

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(7)))
		hdc_epcmd_cache_bits_host_mode(base, offset, &reg);
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

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(7)))
		hdc_epcmd_cache_bits_host_mode(base, offset, &reg);
	else
		hdc_reg_cache_bits(base, offset, &reg);

	reg &= ~(MASK(bit_length) << start_bit);
	reg |= ((value & MASK(bit_length)) << start_bit);

	__raw_writel(reg, base + offset);
	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);
}

static void setup_usb_top_hdmac(struct f_usb20hdc_hcd *priv, s8 dma_ch)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
        u32 val;

	if (priv->f_otg->variant != FHDC_VARIANT_LAP)
		return;

	/* beat type incr16 */
	val = (((priv->dma_data[dma_ch].hdmac_req.dmaca & 0xf0000) >> 16) | 0x30);
	writel(val, priv->reg_base + USB_TOP_HDMAC1 +
		    (priv->dma_data[dma_ch].hdmac_ch << 2));
#endif
}

void stop_usb_top_hdmac(struct f_usb20hdc_hcd_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_hcd *priv = ep->priv;
	s8 dma_ch = ep->dma_ch;
	u32 ofs = USB_TOP_DMAFSM1 + (priv->dma_data[dma_ch].hdmac_ch << 2);

	switch (priv->f_otg->variant) {
	case FHDC_VARIANT_LAP:
		writel(0, priv->reg_base + ofs);
		udelay(1);
		writel(1, priv->reg_base + ofs);
		break;
	case FHDC_VARIANT_ORIG:
		break;
	}
#endif
}

void hdc_dma_if_stop(struct f_usb20hdc_hcd_ep *ep)
{
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_hcd *priv = ep->priv;
	s8 dma_ch = ep->dma_ch;

	hdc_write_bits(priv->reg_base,
			HDC_DMAC(dma_ch), BIT_DMA_ST, 1, 0);
	hdc_write_bits(priv->reg_base, HDC_INTEN,
			BIT_DMA_INTEN(dma_ch), 1, 0);
#endif
}

static void set_all_interrupt_factor(void __iomem *base, bool on,
				     bool include_ep)
{
	int i;

	hdc_write_bits(base, HDC_HOSTINTEN, BIT_SOFSTART_INTEN, 1, 0);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_FRAMEOV_INTEN, 1, 0);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_CONNECT_C_INTEN, 1, on);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_ENABLE_C_INTEN, 1, on);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_SUSPEND_C_INTEN, 1, on);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_OV_CURRENT_C_INTEN, 1, on);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_RESET_C_INTEN, 1, on);
	for (i = EP0; include_ep && (i < HDC_HCD_MAX_EP); i++)
		hdc_write_bits(base, HDC_HOSTINTEN,
			       BIT_TRANS_DONE_INTEN(i), 1, on);
}

static void clear_all_int_status_change(void __iomem *base, bool flag)
{
	int i;

	if (flag) {
		hdc_write_bits(base, HDC_PORTSTSC, BIT_CONNECTION_C, 1, 0);
		hdc_write_bits(base, HDC_PORTSTSC, BIT_SUSPEND_C, 1, 0);
	}
	hdc_write_bits(base, HDC_PORTSTSC, BIT_ENABLE_C, 1, 0);
	hdc_write_bits(base, HDC_PORTSTSC, BIT_OV_CURR_C, 1, 0);
	hdc_write_bits(base, HDC_PORTSTSC, BIT_RESET_C, 1, 0);
	hdc_write_bits(base, HDC_HOSTEVENTSRC, BIT_SOFSTART, 1, 0);
	hdc_write_bits(base, HDC_HOSTEVENTSRC, BIT_FRAMEOV, 1, 0);
	for (i = EP0; i < HDC_HCD_MAX_EP; i++)
		hdc_write_bits(base, HDC_HOSTEVENTSRC, BIT_TRANS_DONE(i), 1, 0);
}

static void initialize_hcd_controller(struct f_usb20hdc_hcd *priv)
{
	void __iomem *base = priv->reg_base;
	u32 i;

	if (is_device_mode_usage(base))
		return;

	/* initialize F_USB20HDC system configuration register */
	hdc_core_soft_reset(priv->f_otg);

	/* initialize F_USB20HDC mode register */
	hdc_write_bits(base, HDC_MODE, BIT_HOST_EN, 1, 0);

	/*
	 * initialize F_USB20HDC global interrupt register
	 * [notice]:otg_inten bit is always enable
	 */
	hdc_disable_interrupt(base, HDC_HCD_MAX_EP);

	/* initialize F_USB20HDC port control / status / event register */
	if (priv->wakeup_from_poweroff == 0) {
		hdc_write_bits(base, HDC_PORTSC, BIT_POWER_CTL_REQ, 1, 1);
		hdc_set_vbus(base, 0);
	}
	hdc_write_bits(base, HDC_PORTSC, BIT_FORCEFS_REQ, 1, !priv->hs_support);
	hdc_write_bits(base, HDC_PORTSC, BIT_ENABLE_REQ, 1, 0);
	hdc_write_bits(base, HDC_PORTSC, BIT_WAKEUP_REQ, 1, 1);

	set_all_interrupt_factor(base, 0, true);
	hdc_write_bits(base, HDC_HOSTINTEN, BIT_SOF_INTERVAL,
		       LEN_SOF_INTERVAL, FRAME_8U);

	clear_all_int_status_change(base, true);

	hdc_frame_set_index(base, 0, 7);
	hdc_frame_set_count(base, HDC_HCD_CONFIG_FRMINIT_BIT_VALUE);

	hdc_set_host_run(base, 0);
	hdc_set_startlink(base, EP0);

	/* initialize F_USB20HDC otg register */
	hdc_write_bits(base, HDC_OTGC, BIT_DM_PULL_DOWN, 1, 0);
	hdc_write_bits(base, HDC_OTGC, BIT_DP_PULL_DOWN, 1, 0);
	hdc_write_bits(base, HDC_OTGSTSC, BIT_TMROUT_C, 1, 0);
	hdc_write_bits(base, HDC_OTGSTSC, BIT_VBUS_VLD_C, 1, 0);

	hdc_write_bits(base, HDC_OTGSTSRISE, BIT_TMROUT_REN, 1, 0);
	hdc_write_bits(base, HDC_OTGTC, BIT_START_TMR, 1, 0);
	hdc_write_bits(base, HDC_OTGT, BIT_TMR_INIT_VAL, LEN_TMR_INIT_VAL, 0);

	/* initialize F_USB20HDC dma register */
	if (priv->f_otg->variant == FHDC_VARIANT_LAP)
		hdc_dma_if_dear(base, MODE_BLOCK);
	else
		hdc_dma_if_dear(base, MODE_DEMAND);

	/* initialize F_USB20HDC ram register */
	for (i = EP0; i < HDC_HCD_MAX_EP; i++) {
		/* empty bit need to be 1 initially */
		hdc_writel(base, HDC_HCEPCTRL1(i), 0x00000400);
		hdc_writel(base, HDC_HCEPCTRL2(i), 0x00000000);
		hdc_writel(base, HDC_EPCONF(i), 0x00000000);
	}
	for (i = 0; i < HDC_HCD_MAX_EP * EP_BUF_CNT; i++)
		hdc_writel(base, HDC_EPCOUNT(i), 0);
}

static void initialize_td_schedule_list(void *base_addr)
{
	hdc_set_startlink(base_addr, EP3);
	hdc_ep_set_nextlink(base_addr, EP3, EP4);
	hdc_ep_set_nextlink(base_addr, EP4, EP5);
	hdc_ep_set_nextlink(base_addr, EP5, EP6);
	hdc_ep_set_nextlink(base_addr, EP6, EP7);
	hdc_ep_set_nextlink(base_addr, EP7, EP0);
	hdc_ep_set_nextlink(base_addr, EP0, EP2);
	hdc_ep_set_nextlink(base_addr, EP2, EP1);
	hdc_ep_set_nextlink(base_addr, EP1, EP0);
}

static void initialize_endpoint_configure(struct f_usb20hdc_hcd *priv)
{
	u32 ep_ch, bcnt;
	struct f_usb20hdc_hcd_ep *ep;

	for (ep_ch = EP0; ep_ch < HDC_HCD_MAX_EP; ep_ch++) {
		ep = &priv->ep[ep_ch];
		ep->ep_ch = ep_ch;
		ep->inuse_devnum = 0;
		ep->inuse_epnum = 0;
		if (ep_ch < EP3)
			ep->max_packet_size = priv->hs_support ?
				ep_config_data[ep_ch].hs_maxpacket :
				ep_config_data[ep_ch].fs_maxpacket;
		ep->buffer_size = ep_ch < EP3 ?
			ep_config_data[ep_ch].buffer_size :
			HDC_HCD_MAX_BUF_SIZE_PER_EP;
		ep->buffers = ep_ch < EP3 ?
			ep_config_data[ep_ch].buffers : EP_BUF_CNT;
		ep->buf_offset[0] = ep_ch == EP0 ?
			hdc_get_epbuf_addr_offset() :
			priv->ep[ep_ch - 1].buf_offset[
			priv->ep[ep_ch - 1].buffers - 1] +
			priv->ep[ep_ch - 1].buffer_size;
		ep->buf_offset[0] = (ep->buf_offset[0] + 0x3) & ~0x3;
		for (bcnt = 1; bcnt < ep->buffers; bcnt++) {
			ep->buf_offset[bcnt] =
				ep->buf_offset[bcnt - 1] + ep->buffer_size;
			ep->buf_offset[bcnt] =
				(ep->buf_offset[bcnt] + 0x3) & ~0x3;
		}
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		ep->dma_ch = ep_config_data[ep_ch].dma_ch;
#endif
	}
}

static u8 is_endpoint_buffer_usable(void)
{
	u32 counter;
	u32 buffer_size = 0;

	/* calculate RAM buffer size */
	buffer_size += 256;
	buffer_size += ep_config_data[EP0].buffer_size * EP_BUF_CNT;
	for (counter = EP1; counter < EP3; counter++)
		buffer_size += ep_config_data[counter].buffer_size *
				ep_config_data[counter].buffers;
	for (; counter < HDC_HCD_MAX_EP; counter++)
		buffer_size += HDC_HCD_MAX_BUF_SIZE_PER_EP * EP_BUF_CNT;

	return buffer_size <= HDC_HCD_EP_BUFFER_RAM_SIZE ? 1 : 0;
}

static u8 is_split_transaction_required(struct f_usb20hdc_hcd_ep *ep)
{
	struct urb *urb = ep->req->urb;
	struct usb_device *parent;
	u8 high_speed_parent_exist;

	for (high_speed_parent_exist = 0, parent =
		urb->dev->parent; parent; parent = parent->parent) {
		if ((parent->level) && (parent->speed == USB_SPEED_HIGH)) {
			high_speed_parent_exist = 1;
			break;
		}
	}

	return urb->dev->speed != USB_SPEED_HIGH &&
					high_speed_parent_exist;
}

#ifdef VIRTUAL_DEVICE_ADDRESS
static int free_all_mapped_addr(struct usb_hcd *hcd)
{
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	int i = 0;

	/* lookup a child logical addr of this hub addr.*/
	for (i = 0; i < MAX_DEV_ADDR; i++)
		priv->dev_addr_table[i].used = 0;

	return 0;
}

static int free_mapped_addr(struct usb_hcd *hcd, u8 log_addr)
{
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	struct dev_addr_mapping *table = priv->dev_addr_table;
	int i, j;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0) {
		dev_err(priv->dev, "%s() zero addr has no mapped addr\n",
			__func__);
		return -1;
	}

	/* lookup a mapped structure and free it*/
	for (i = 0; i < MAX_DEV_ADDR; i++)
		if (table[i].logical_addr == log_addr) {
			/* free the mapped addr*/
			table[i].used = 0;

			/* search any child if removed addr is a parent */
			for (j = 0; j < MAX_DEV_ADDR; j++) {
				if (table[i].hub_addr == log_addr)
					free_mapped_addr(hcd,
							 table[i].logical_addr);
			}
			return 0;
		}

	/*no mapped addr is available*/
	return -1;
}

static int free_hub_mapped_addr(struct usb_hcd *hcd, u8 hub_addr, u8 port_num)
{
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	struct device *pdev = priv->dev;
	struct dev_addr_mapping *table = priv->dev_addr_table;
	int i;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (hub_addr == 0) {
		dev_err(pdev, "%s() zero addr can't be used\n", __func__);
		return -1;
	}

	/* lookup a child logical addr of this hub addr.*/
	for (i = 0; i < MAX_DEV_ADDR; i++)
		if ((table[i].hub_addr == hub_addr) &&
		    (table[i].port_num == port_num)) {
			free_mapped_addr(hcd, table[i].logical_addr);
			return 0;
		}

	return -1;
}

static int create_mapped_addr(struct usb_hcd *hcd, struct urb *urb, u8 log_addr)
{
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	struct device *pdev = hcd->self.controller;
	struct dev_addr_mapping *table = priv->dev_addr_table;
	struct usb_device *parent;
	struct usb_device *children;
	int i;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0) {
		dev_err(pdev, "%s() zero addr can't be used\n", __func__);
		return -1;
	}

	/* check if there is already a mapped physical address */
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if ((table[i].used == 1) &&
		    (table[i].logical_addr == log_addr)) {
			dev_err(pdev, "%s() addr been used (log %d phy %d)\n",
				__func__, log_addr, i + 1);
			return -1;
		}
	}

	/* search to allocate a usable physical address */
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if (table[i].used == 0) {
			table[i].logical_addr = log_addr;
			table[i].used = 1;
			break;
		}
	}

	/* check if any physical addr is available */
	if (i == MAX_DEV_ADDR) {
		dev_err(pdev, "%s() no available physical addr\n", __func__);
		return -1;
	}

	/* fill information which will be used in recycling this mapped addr */
	for (parent = urb->dev->parent, children = urb->dev;
		parent; parent = parent->parent, children = children->parent) {
		if (parent->level) {
			table[i].hub_addr = parent->devnum;
			table[i].port_num = children->portnum;
			return i + 1;
		}
	}

	/*this device is plug into root hub, so no parent device*/
	table[i].hub_addr = 0;
	table[i].port_num = 0;
	return i + 1;
}
#endif
/*
 *return value : >= 0 means successfully
 *< 0 means failed.
*/
static int get_mapped_addr(struct usb_hcd *hcd, u8 log_addr)
{
#ifdef VIRTUAL_DEVICE_ADDRESS
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	struct dev_addr_mapping *table = priv->dev_addr_table;
	int i;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0)
		return 0;

	/* lookup a mapped physical address and return it*/
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if ((table[i].used == 1) && (table[i].logical_addr == log_addr))
			return i + 1;
	}

	/*no phy addr is available*/
	return -1;
#else
	return log_addr;
#endif
}

static void set_endpoint_type(struct f_usb20hdc_hcd *priv, u8 ep_ch,
			      u8 transfer_type)
{
	struct f_usb20hdc_hcd_ep *ep = &priv->ep[ep_ch];

	ep->max_packet_size = priv->hs_support ?
				ep_config_data[transfer_type].hs_maxpacket :
				ep_config_data[transfer_type].fs_maxpacket;
	ep->buffers = ep_config_data[transfer_type].buffers;
	ep->buffer_size = ep_config_data[transfer_type].buffer_size;
	ep->transfer_type = transfer_type;
}

static void hdc_ep_config(struct f_usb20hdc_hcd_ep *ep, int buf_num,
			  int ep_type, int packet_id, int toggle_ctrl,
			  bool is_iso)
{
	struct f_usb20hdc_hcd_req *req = ep->req;
	struct urb *urb = ep->req->urb;
	void *base = ep->priv->reg_base;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(ep->priv);
	u8 ep_ch = ep->ep_ch;
	struct usb_device *p, *child;
	u32 epcmd, speed, epctrl2, epconf;

	speed =  urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		       urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
		       SPEED_LOW_SPEED;

	/* config spcmd register */
	hdc_cmd0_spin(base, HDC_EPCMD(ep_ch));
	epcmd = hdc_readl_cached(base, HDC_EPCMD(ep_ch));
	epcmd &= ~((MASK(LEN_SPEED) << BIT_SPEED) |
		(MASK(LEN_BNUM) << BIT_BNUM) | (1 << BIT_SC) |
		(MASK(LEN_ET) << BIT_ET) | (MASK(LEN_SENDPID) << BIT_SENDPID));

	epcmd |= ((speed & MASK(LEN_SPEED)) << BIT_SPEED) |
		(((buf_num - 1) & MASK(LEN_BNUM)) << BIT_BNUM) |
		(0 << BIT_SC) | ((ep_type & MASK(LEN_ET)) << BIT_ET) |
		((packet_id & MASK(LEN_SENDPID)) << BIT_SENDPID) |
		(1 << BIT_ERRCNT_CLR) | (1 << BIT_STATUS_CLR);

	if (toggle_ctrl)
		epcmd |= (1 << BIT_TOGGLE_SET);
	else
		epcmd |= (1 << BIT_TOGGLE_CLR);

	__raw_writel(epcmd, base + HDC_EPCMD(ep_ch));
	hdc_cmd0_spin(base, HDC_EPCMD(ep_ch));

	/* config HDC_HCEPCTRL2 register */
	epctrl2 = hdc_readl_cached(base, HDC_HCEPCTRL2(ep_ch));
	if (is_split_transaction_required(ep)) {
		for (p = urb->dev->parent, child = urb->dev;
			p; p = p->parent, child = child->parent) {
			if ((p->level) && (p->speed == USB_SPEED_HIGH)) {
				ep->split.hub_addr = p->devnum;
				ep->split.port_addr = child->portnum;
				ep->split.startbit = child->speed !=
					USB_SPEED_FULL;
				break;
			}
		}

		epctrl2 &= ~((0x1 << BIT_STARTBIT) |
			(MASK(LEN_HUB_PORT_NUM) << BIT_HUB_PORT_NUM) |
			(MASK(LEN_HUBADDR) << BIT_HUBADDR));

		epctrl2 |= ((ep->split.startbit & 0x1) << BIT_STARTBIT) |
			((ep->split.port_addr & MASK(LEN_HUB_PORT_NUM))
			 << BIT_HUB_PORT_NUM) |
			((get_mapped_addr(hcd, ep->split.hub_addr) &
			  MASK(LEN_HUBADDR)) << BIT_HUBADDR);
	}
	epctrl2 &= ~((MASK(LEN_FUNCADDR) << BIT_FUNCADDR) |
		(MASK(LEN_EP_NUMBER) << BIT_EP_NUMBER));
	epctrl2 |=
		(get_mapped_addr(hcd, usb_pipedevice(urb->pipe) &
		 MASK(LEN_FUNCADDR)) << BIT_FUNCADDR) |
		((usb_pipeendpoint(urb->pipe) & MASK(LEN_EP_NUMBER)) <<
			BIT_EP_NUMBER);
	__raw_writel(epctrl2, base + HDC_HCEPCTRL2(ep_ch));

	/* config HDC_EPCONF register */
	epconf = hdc_readl_cached(base, HDC_EPCONF(ep_ch));
	epconf &= ~((MASK(LEN_BASE) << BIT_BASE) |
		(MASK(LEN_SIZE) << BIT_SIZE) |
		(MASK(LEN_COUNTIDX) << BIT_COUNTIDX));
	epconf |= ((((ep->buf_offset[0] & 0x7fff) >> 2) & MASK(LEN_BASE))
		    << BIT_BASE) |
		  (((is_iso ? req->maxpacket : urb->ep->desc.wMaxPacketSize) &
		    MASK(LEN_SIZE)) << BIT_SIZE) |
		  (((ep_ch * EP_BUF_CNT) & MASK(LEN_COUNTIDX)) << BIT_COUNTIDX);
	__raw_writel(epconf, base + HDC_EPCONF(ep_ch));
}

static void hdc_ep_frame_interval_setup(struct f_usb20hdc_hcd_ep *ep)
{
	u32 counter;
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 frame_interval;
	int interval = 1;

	/* calculate frame interval */
	if (urb->dev->speed == USB_SPEED_HIGH) {
		if (!(urb->interval / 8)) {
			hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch),
				       BIT_INTERVALSEL, 1, 0);
			hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch),
				       BIT_FMTSEL, 1, 0);
			hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch),
				       BIT_INTERVAL, LEN_INTERVAL, 0);
			return;
		}
		frame_interval = urb->interval / 8;
	} else {
		frame_interval = urb->interval;
	}

	/* search interval register set value */
	for (counter = 0;; counter++) {
		if (frame_interval == (1 << counter))
			break;
		if (((1 << counter) < frame_interval) &&
		    (frame_interval <= (1 << (counter + 1))))
			break;
		if (counter >= 8) {
			counter = 7;
			break;
		}
	}

	/* check Interrupt SPLIT transaction */
	if ((is_split_transaction_required(ep)) &&
	    (usb_pipetype(urb->pipe) == PIPE_INTERRUPT)) {
		counter = 3;
		interval = 0;
	}

	hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch),
		       BIT_INTERVALSEL, 1, interval);
	hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch), BIT_FMTSEL, 1, 0);
	hdc_write_bits(base, HDC_HCEPCTRL2(ep_ch),
		       BIT_INTERVAL, LEN_INTERVAL, counter);
}

static void hdc_ep_stop_and_clear(struct f_usb20hdc_hcd_ep *ep)
{
	u8 ep_ch = ep->ep_ch;
	void *base = ep->priv->reg_base;

	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_STATUS_CLR, 1, 1);
	hdc_ep_complete_inten_set(ep, 0);
	hdc_ep_set_stop(base, ep_ch);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_INIT, 1, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_TOGGLE_CLR, 1, 1);
	ep->transfer_status = -EPIPE;
}

static void notify_transfer_request_complete(struct f_usb20hdc_hcd_ep *ep,
					     struct f_usb20hdc_hcd_req *req,
					     int status)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(priv);
	struct urb *urb = req->urb;

	/* delete and initialize list */
	list_del_init(&req->queue);

	/* set request status */
	if (req->urb->status == -EINPROGRESS)
		req->urb->status = status;
	else
		status = req->urb->status;

	/* clear request execute */
	req->request_execute = 0;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	req->dmac_int_occurred = 0;
	req->usb_dma_int_occurred = 0;
#endif

	/* free request memory */
	WARN_ON(!list_empty(&req->queue));
	kfree(req);

	/* clear current request of the endpoint */
	if (ep->req == req)
		ep->req = NULL;

	/* unlink URB to ep */
	usb_hcd_unlink_urb_from_ep(hcd, urb);

	/* notify request complete for upper */
	spin_unlock(&priv->lock);
	usb_hcd_giveback_urb(hcd, urb, status);
	spin_lock(&priv->lock);
}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static u8 set_dma_transfer(struct f_usb20hdc_hcd_ep *ep, dma_addr_t src,
			   dma_addr_t dest, u32 bytes, u8 ep_int_en,
			   u8 ep_start)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct f_usb20hdc_hcd_req *req = ep->req;
	void *base = priv->reg_base;
	struct device *dev = priv->dev;
	u8 ep_ch = ep->ep_ch;
	s8 dma_ch = ep->dma_ch;
	struct f_usb20hdc_dma_data *dma_data = &priv->dma_data[dma_ch];
	int result;
	u32 dmac;
	int dma_mode = MODE_DEMAND;
	int dma_blksize = 4;

        if (priv->f_otg->variant == FHDC_VARIANT_LAP) {
                dma_mode = MODE_BLOCK;
		dma_blksize = 64;
	}

	/* check DMA transfer address align */
	if ((dest & 0x3) || (src & 0x3)) {
		dev_err(dev, "%s() dma addr not aligned\n", __func__);
		return 0;
	}

	dma_data->ep_ch = ep_ch;
	dma_data->hdmac_req.dmacb = HDMACB_MS_BLOCK | HDMACB_TT_2CYCLE |
					HDMACB_EI | HDMACB_CI | HDMACB_TW_WORD;
	dma_data->hdmac_req.dmacb |= usb_pipein(req->urb->pipe) ?
							HDMACB_FS : HDMACB_FD;
	dma_data->hdmac_req.dmaca = dma_data->dreq | HDMACA_BT_INCR16 |
						(((bytes + 63) / 64) - 1);

	dma_data->hdmac_req.req_data.size = bytes;
	dma_data->hdmac_req.req_data.src = src;
	dma_data->hdmac_req.req_data.dst = dest;
	dma_data->hdmac_req.req_data.irq_data = ep;
	dma_data->hdmac_req.req_data.callback_fn = end_dma_transfer;

	result = hdmac_enqueue(dma_data->hdmac_ch, &dma_data->hdmac_req);
	if (result) {
		dev_err(dev, "%s() HDMAC request enqueue failed at 0x%x\n",
			__func__, result);
		return 0;
	}

	/* set f_usb20hdc controller DMA register */
	dmac = hdc_readl_cached(base, HDC_DMAC(dma_ch));
	dmac &= ~((MASK(LEN_DMA_BLKSIZE) << BIT_DMA_BLKSIZE) |
		(1 << BIT_DMA_MODE) | (MASK(LEN_DMA_EP) << BIT_DMA_EP));
	dmac |= ((dma_blksize & MASK(LEN_DMA_BLKSIZE)) << BIT_DMA_BLKSIZE) |
		(dma_mode << BIT_DMA_MODE) |
		((ep_ch & MASK(LEN_DMA_EP)) << BIT_DMA_EP);

	if (!usb_pipein(ep->req->urb->pipe)) {
		dmac &= ~((1 << BIT_DMA_SENDNULL) | (1 << BIT_DMA_INT_EMPTY));
		dmac |= (1 << BIT_DMA_INT_EMPTY) | (0 << BIT_DMA_SENDNULL);
	} else {
		dmac &= ~(1 << BIT_DMA_SPR);
		dmac |= (0 << BIT_DMA_SPR);
	}
	/* write DMAC later*/
	hdc_writel(base, HDC_DMATCI(dma_ch), bytes);
	hdc_write_bits(base, HDC_INTS, BIT_DMA_INT(dma_ch), 1, 0);
	hdc_write_bits(base, HDC_INTEN, BIT_DMA_INTEN(dma_ch), 1, 1);

	/* set request execute */
	req->request_execute = 1;
	if (ep->transfer_type == TRANSFER_TYPE_BULK_OUT) {
		req->dmac_int_occurred = 0;
		req->usb_dma_int_occurred = 0;
	}

	/* start HDMAC and DMA-IF */
	hdmac_start(priv->dma_data[dma_ch].hdmac_ch);

	setup_usb_top_hdmac(priv, dma_ch);
	dmac |= (1 << BIT_DMA_ST);
	writel(dmac, priv->reg_base + HDC_DMAC(dma_ch));

	if (ep_int_en)
		hdc_ep_complete_inten_set(ep, 1);

	if (!ep_start)
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}
#endif

static void dequeue_all_request(struct f_usb20hdc_hcd_ep *ep, int status)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct f_usb20hdc_hcd_req *req;
	u32 counter;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch = ep->dma_ch;

	priv->bulkin_remain = 0;
	/* stop DMA transfer */
	if ((dma_ch != -1) && (ep->req) && (ep->req->request_execute)) {
		hdc_dma_if_stop(ep);
		hdc_hdmac_stop(ep);
	}
#endif

	/* check dynamic allocated endpoint still necessary */
	if (ep->transfer_type == TRANSFER_TYPE_INTERRUPT_IN ||
	    ep->transfer_type == TRANSFER_TYPE_INTERRUPT_OUT) {
		priv->eps_inuse_map &= ~(0x0001 << ep->ep_ch);
	} else if (ep->transfer_type == TRANSFER_TYPE_ISOCHRONOUS_IN ||
		ep->transfer_type == TRANSFER_TYPE_ISOCHRONOUS_OUT) {
		priv->eps_inuse_map &=
					~(0x0001 << ep->ep_ch);
		ep->inuse_devnum = 0;
		ep->inuse_epnum = 0;
	}

	/* dequeue all transfer request */
	for (counter = (u32)~0;
		counter && !list_empty(&ep->queue); counter--) {
		req = list_entry(ep->queue.next,
				 struct f_usb20hdc_hcd_req, queue);

		notify_transfer_request_complete(ep, req, status);
	}
}

static void copy_urb_to_ep_buf(struct urb *urb, struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	void *base = priv->reg_base;
	struct f_usb20hdc_hcd_req *req = ep->req;
	u8 ep_ch = ep->ep_ch;
	void *transfer_data;
	u32 index, bytes, ep_buf_addr;

	index = hdc_read_epctrl_bits(base, HDC_HCEPCTRL1(ep_ch),
				     BIT_APPPTR, LEN_APPPTR);

	/* set transfer data pointer */
	switch (ep->transfer_type) {
	case TRANSFER_TYPE_CONTROL:
		ep_buf_addr = ep->buf_offset[index];
		if (priv->ctrl_stage == CONTROL_TRANSFER_STAGE_OUT_DATA)
			goto  ctrl_out_data;
		transfer_data = urb->setup_packet;
		bytes = 8;
		break;
	case TRANSFER_TYPE_BULK_OUT:
	case TRANSFER_TYPE_INTERRUPT_OUT:
		ep_buf_addr = ep->buf_offset[0] +
			urb->ep->desc.wMaxPacketSize * index;
ctrl_out_data:
		transfer_data = urb->transfer_buffer + urb->actual_length;
		bytes = (urb->transfer_buffer_length - urb->actual_length) <
			urb->ep->desc.wMaxPacketSize ?
			urb->transfer_buffer_length - urb->actual_length :
			urb->ep->desc.wMaxPacketSize;
		break;
	case TRANSFER_TYPE_ISOCHRONOUS_OUT:
		ep_buf_addr = ep->buf_offset[0] + req->maxpacket * index;
		transfer_data = urb->transfer_buffer +
				urb->iso_frame_desc[ep->iso_packets].offset;
		bytes = urb->iso_frame_desc[ep->iso_packets].length <
			req->maxpacket ?
			urb->iso_frame_desc[ep->iso_packets].length :
			req->maxpacket;
		break;
	}

	prefetch(transfer_data);

	/* write transfer data to buffer */
	if (bytes)
		hdc_write_epbuf(base, ep_buf_addr, transfer_data, bytes);

	/* notify buffer write bytes */
	hdc_write_bits(base, HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
		       BIT_APPCNT, LEN_APPCNT, bytes);

	/* set ep buffer write */
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFWR, 1, 1);
}

static u32 copy_ep_buf_to_urb(struct urb *urb, struct f_usb20hdc_hcd_ep *ep)
{
	void *base = ep->priv->reg_base;
	struct f_usb20hdc_hcd_req *req = ep->req;
	u8 ep_ch = ep->ep_ch;
	void *transfer_data;
	u32 bytes, ep_buf_offset, index =
		hdc_read_epctrl_bits(base, HDC_HCEPCTRL1(ep_ch), BIT_APPPTR,
				     LEN_APPPTR);

	switch (ep->transfer_type) {
	case TRANSFER_TYPE_CONTROL:
		transfer_data = urb->transfer_buffer + urb->actual_length;
		ep_buf_offset = ep->buf_offset[index];
		break;
	case TRANSFER_TYPE_BULK_IN:
	case TRANSFER_TYPE_INTERRUPT_IN:
		transfer_data = urb->transfer_buffer + urb->actual_length;
		ep_buf_offset = ep->buf_offset[0] +
			urb->ep->desc.wMaxPacketSize * index;
		break;
	case TRANSFER_TYPE_ISOCHRONOUS_IN:
		transfer_data = urb->transfer_buffer +
			urb->iso_frame_desc[ep->iso_packets].offset;
		ep_buf_offset = ep->buf_offset[0] + req->maxpacket * index;
		break;
	}
	prefetch(transfer_data);

	/* get IN transfer byte */
	bytes = hdc_read_bits(base, HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
			      BIT_PHYCNT, LEN_PHYCNT);
	if (ep->transfer_type != TRANSFER_TYPE_ISOCHRONOUS_IN) {
		if (urb->transfer_buffer_length < (urb->actual_length + bytes))
			bytes = urb->transfer_buffer_length -
				urb->actual_length;
	} else {
		if (urb->iso_frame_desc[ep->iso_packets].length < bytes)
			bytes = urb->iso_frame_desc[ep->iso_packets].length;
	}

	/* check buffer read bytes */
	if (bytes)
		hdc_read_epbuf(base, ep_buf_offset, transfer_data, bytes);

	/* update actual bytes */
	urb->actual_length += bytes;
	if (ep->transfer_type == TRANSFER_TYPE_ISOCHRONOUS_IN) {
		urb->iso_frame_desc[ep->iso_packets].actual_length = bytes;
		urb->iso_frame_desc[ep->iso_packets].status = 0;
		ep->iso_packets++;
	}

	/* set ep buffer read */
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFRD, 1, 1);

	return bytes;
}

static const u8 set_control_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	hdc_ep_config(ep, 1, TYPE_CONTROL, PID_SETUP, TOGGLE_CLR, false);

	/* update control transfer stage */
	priv->ctrl_stage = CONTROL_TRANSFER_STAGE_SETUP;

	copy_urb_to_ep_buf(urb, ep);

	/* set request execute */
	ep->req->request_execute = 1;

	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_bulk_in_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void __iomem *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 buf_num;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch = ep->dma_ch;
	u32 bytes;
	u8 start_flag = 0;
	u8 hcptr, appptr, bcnt;
	dma_addr_t dma_addr;
	u32 epctrl1;

	priv->bulkin_remain = 0;

	epctrl1 = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep_ch));
	hcptr = VALUE(epctrl1, BIT_HCPTR, LEN_HCPTR);
	appptr = VALUE(epctrl1, BIT_APPPTR, LEN_APPPTR);
	bcnt = VALUE(epctrl1, BIT_EPCTRL_BNUM, LEN_EPCTRL_BNUM) + 1;

	if (hdc_status_trans(epctrl1)) {
		if ((urb->transfer_buffer_length <=
				(urb->ep->desc.wMaxPacketSize * bcnt))) {
			start_flag = 1;
		}
		goto dma_trans;
	}

	if (!hdc_empty_bit(epctrl1)) {
		if ((urb->transfer_buffer_length <=
				(urb->ep->desc.wMaxPacketSize * bcnt))) {
			if (!((hcptr != appptr) && (((hcptr < appptr ?
				bcnt + hcptr - appptr : hcptr - appptr) *
				urb->ep->desc.wMaxPacketSize) <
				urb->transfer_buffer_length))) {
				start_flag = 1;
			}
		}
		goto dma_trans;
	}

	buf_num = (urb->transfer_buffer_length ? ep->buffers : 1);
#else
	buf_num = (urb->transfer_buffer_length >
		urb->ep->desc.wMaxPacketSize ? ep->buffers : 1);
#endif

	hdc_ep_config(ep, buf_num, TYPE_BULK, PID_IN,
		      usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0),
		      false);

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
dma_trans:
	if ((dma_ch != -1) && (urb->transfer_buffer_length != 0)) {
		/* calculate this time transfer byte */
		bytes = (urb->transfer_buffer_length - urb->actual_length) <
			HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			(urb->transfer_buffer_length - urb->actual_length) :
			HDC_HCD_MAX_DMA_TRANSFER_SIZE;

		dma_addr = ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
			priv->f_otg->handle_pio_urb_quirk) ?
			priv->bulk_in_dma + urb->actual_length :
			urb->transfer_dma + urb->actual_length;

		if (set_dma_transfer(ep, priv->dma_data[dma_ch].epbuf_daddr,
				     dma_addr, bytes, true,
				     start_flag))
			return 1;
	}
#endif

	/* set request execute */
	ep->req->request_execute = 1;

	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_bulk_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch = ep->dma_ch;
	u32 bytes;
	dma_addr_t dma_addr;
#endif

	hdc_ep_config(ep, ep->buffers, TYPE_BULK, PID_OUT,
		      usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1),
		      false);

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	if ((dma_ch != -1) && (urb->transfer_buffer_length != 0)) {
		/* calculate this time transfer byte */
		bytes = (urb->transfer_buffer_length - urb->actual_length) <
			HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			(urb->transfer_buffer_length - urb->actual_length) :
			HDC_HCD_MAX_DMA_TRANSFER_SIZE;

		if ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
		    !urb->actual_length &&
		    priv->f_otg->handle_pio_urb_quirk)
			memcpy(priv->bulk_out_buf, urb->transfer_buffer,
			       urb->transfer_buffer_length);

		dma_addr = ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
			priv->f_otg->handle_pio_urb_quirk) ?
			priv->bulk_out_dma + urb->actual_length :
			urb->transfer_dma + urb->actual_length;

		if (set_dma_transfer(ep, dma_addr,
				     priv->dma_data[dma_ch].epbuf_daddr,
				     bytes, true, false))
			return 1;
	}
#endif

	copy_urb_to_ep_buf(urb, ep);

	/* set request execute */
	ep->req->request_execute = 1;

	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_interrupt_in_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u8 split_require = is_split_transaction_required(ep);
	u32 buf_num;

	buf_num = (urb->transfer_buffer_length >
		urb->ep->desc.wMaxPacketSize) || (split_require) ?
		ep->buffers : 1;

	hdc_ep_config(ep, buf_num, TYPE_INTERRUPT, PID_IN,
		      usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0),
		      false);

	hdc_ep_frame_interval_setup(ep);

	/* set request execute */
	ep->req->request_execute = 1;

	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_interrupt_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 buf_num;
	u8 split_require = is_split_transaction_required(ep);

	buf_num = (urb->transfer_buffer_length >
		urb->ep->desc.wMaxPacketSize) || (split_require) ?
		ep->buffers : 1;

	hdc_ep_config(ep, buf_num, TYPE_INTERRUPT, PID_OUT,
		      usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1),
		      false);

	hdc_ep_frame_interval_setup(ep);

	copy_urb_to_ep_buf(urb, ep);

	/* set request execute */
	ep->req->request_execute = 1;

	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_isochronous_in_transfer(
	struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	hdc_ep_config(ep, urb->number_of_packets > 1 ? ep->buffers : 1,
		      TYPE_ISOCHRONOUS, PID_IN, TOGGLE_CLR, true);

	hdc_ep_frame_interval_setup(ep);

	/* initialize transfer packet count */
	ep->iso_packets = 0;

	/* set request execute */
	ep->req->request_execute = 1;
	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 set_isochronous_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	hdc_ep_config(ep, ep->buffers,
		      TYPE_ISOCHRONOUS, PID_OUT, TOGGLE_CLR, true);

	hdc_ep_frame_interval_setup(ep);

	/* initialize transfer packet count */
	ep->iso_packets = 0;

	copy_urb_to_ep_buf(urb, ep);

	/* set request execute */
	ep->req->request_execute = 1;
	hdc_ep_complete_inten_set(ep, 1);
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 1;
}

static const u8 (* const set_transfer_function[])(
					struct f_usb20hdc_hcd_ep *) = {
	set_control_transfer,
	set_bulk_in_transfer,
	set_bulk_out_transfer,
	set_interrupt_in_transfer,
	set_interrupt_out_transfer,
	set_isochronous_in_transfer,
	set_isochronous_out_transfer,
};

static const u8 end_control_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch, pid;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(priv);
	struct device *pdev = hcd->self.controller;
#ifdef VIRTUAL_DEVICE_ADDRESS
	struct usb_ctrlrequest *dev_req;
	u32 data;
	int hub_addr;
#endif
	u8 index = hdc_read_epctrl_bits(base, HDC_HCEPCTRL1(ep_ch),
		BIT_APPPTR, LEN_APPPTR);
	u32 bytes;

	/* process control transfer stage */
	switch (priv->ctrl_stage) {
	case CONTROL_TRANSFER_STAGE_SETUP:
		if (usb_pipein(urb->pipe)) {
			/* update control transfer stage */
			priv->ctrl_stage = CONTROL_TRANSFER_STAGE_IN_DATA;
			pid = PID_IN;
		} else if (urb->transfer_buffer_length) {
			/* update control transfer stage */
			priv->ctrl_stage = CONTROL_TRANSFER_STAGE_OUT_DATA;
			pid = PID_OUT;

			copy_urb_to_ep_buf(urb, ep);
		} else {
			/* update control transfer stage */
			priv->ctrl_stage = CONTROL_TRANSFER_STAGE_IN_STATUS;
			pid = PID_IN;
		}
		hdc_write_bits(base, HDC_EPCMD(ep_ch),
			       BIT_SENDPID, LEN_SENDPID, pid);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_TOGGLE_SET, 1, 1);

		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
		break;
	case CONTROL_TRANSFER_STAGE_IN_DATA:
		/* check STALL response */
		if (hdc_ep_status_stall(ep)) {
			/* clear satus interrupt factor */
			hdc_ep_stop_and_clear(ep);
			dev_dbg(pdev, "%s() end with STALL", __func__);
			return 1;
		}

		bytes = copy_ep_buf_to_urb(urb, ep);

		/* check transfer request complete */
		if ((urb->actual_length >= urb->transfer_buffer_length) ||
		    (bytes % urb->ep->desc.wMaxPacketSize) || !bytes) {
			/* update control transfer stage */
			priv->ctrl_stage = CONTROL_TRANSFER_STAGE_OUT_STATUS;

			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_SENDPID, LEN_SENDPID, PID_OUT);
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_TOGGLE_SET, 1, 1);

			/* notify buffer write bytes */
			hdc_write_bits(base,
				       HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
				       BIT_APPCNT, LEN_APPCNT, 0);

			/* enable control OUT transfer */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFWR, 1, 1);
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* hack into the GetPortStatus' disconnection event */
			dev_req = (struct usb_ctrlrequest *)urb->setup_packet;
			hub_addr = usb_pipedevice(urb->pipe);
			if ((dev_req->bRequestType == 0xa3) &&
			    (dev_req->bRequest == 0x0)) {
				/* read the port connection state*/
				memcpy(&data, urb->transfer_buffer, 4);
				data &= (~0xfffffffe);

				/*release the mapped addr when it's unplug */
				if (!data)
					free_hub_mapped_addr(hcd, hub_addr,
							     dev_req->wIndex);
			}
#endif
		}

		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
		break;
	case CONTROL_TRANSFER_STAGE_OUT_DATA:
		/* check STALL response */
		if (hdc_ep_status_stall(ep)) {
			/* clear satus interrupt factor */
			hdc_ep_stop_and_clear(ep);
			dev_dbg(pdev, "%s() end with STALL", __func__);
			return 1;
		}

		/* update actual bytes */
		urb->actual_length +=
			(urb->transfer_buffer_length - urb->actual_length) <
			urb->ep->desc.wMaxPacketSize ?
			urb->transfer_buffer_length - urb->actual_length :
			urb->ep->desc.wMaxPacketSize;

		/* check transfer request complete */
		if (urb->actual_length >= urb->transfer_buffer_length) {
			/* update control transfer stage */
			priv->ctrl_stage = CONTROL_TRANSFER_STAGE_IN_STATUS;

			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_SENDPID, LEN_SENDPID, PID_IN);
			hdc_write_bits(base, HDC_EPCMD(ep_ch),
				       BIT_TOGGLE_SET, 1, 1);
		} else {
			copy_urb_to_ep_buf(urb, ep);
		}
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
		break;
	case CONTROL_TRANSFER_STAGE_IN_STATUS:
		/* check STALL response */
		if (hdc_ep_status_stall(ep)) {
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* free the mapped addr if SET_ADDRESS request failed */
			dev_req = (struct usb_ctrlrequest *)urb->setup_packet;
			if ((dev_req->bRequestType == 0x0) &&
			    (dev_req->bRequest == 0x5))
				free_mapped_addr(hcd,
						 usb_pipedevice(urb->pipe));
#endif
			/* clear satus interrupt factor */
			hdc_ep_stop_and_clear(ep);
			dev_dbg(pdev, "%s() end with STALL", __func__);
			return 1;
		}

		/* get control IN transfer byte */
		bytes = hdc_read_bits(base,
				      HDC_EPCOUNT(ep_ch * EP_BUF_CNT + index),
				      BIT_PHYCNT, LEN_PHYCNT);

		/* enable next control IN transfer */
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_BUFRD, 1, 1);

		/* complete request */
		hdc_ep_complete_inten_set(ep, 0);
		ep->transfer_status = !bytes ? 0 : -EPROTO;

#ifdef VIRTUAL_DEVICE_ADDRESS
		/* hack into hub port reset request */
		dev_req = (struct usb_ctrlrequest *)urb->setup_packet;
		hub_addr = usb_pipedevice(urb->pipe);
		if ((dev_req->bRequestType == 0x23) &&
		    (dev_req->bRequest == 0x3) && (dev_req->wValue == 0x4)) {
			dev_dbg(pdev,
				"%s() hub port reset request (hub %d port_num %d)",
				__func__, hub_addr, dev_req->wIndex);
			free_hub_mapped_addr(hcd, hub_addr,
					     dev_req->wIndex);
		}
#endif
		return 1;
	case CONTROL_TRANSFER_STAGE_OUT_STATUS:
		/* check STALL response */
		if (hdc_ep_status_stall(ep)) {
			/* clear satus interrupt factor */
			hdc_ep_stop_and_clear(ep);
			dev_dbg(pdev, "%s() end with STALL", __func__);
			return 1;
		}

		/* complete request */
		hdc_ep_complete_inten_set(ep, 0);
		ep->transfer_status = 0;

		return 1;
	default:
		break;
	}

	return 0;
}

static const u8 end_bulk_in_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 bytes;
	u32 max_packet_size = urb->ep->desc.wMaxPacketSize;
	u32 epctrl1;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch = ep->dma_ch;
	dma_addr_t dma_buffer;
	u8 hcptr, appptr, bcnt, result, start_flag = 0;
#endif

	epctrl1 = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep_ch));

	/* check STALL or HALT status */
	if (hdc_status_stall(epctrl1) || hdc_status_halt(epctrl1)) {
		dev_dbg(priv->dev, "%s() STALL or HALT", __func__);
		hdc_ep_stop_and_clear(ep);
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		if (urb->transfer_buffer_length != 0) {
			hdc_hdmac_stop(ep);

			if (dma_ch != -1)
				goto done;
		}
#endif
		return 1;
	}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	if ((dma_ch != -1) && (urb->transfer_buffer_length != 0)) {
		/* get this time transfer byte */
		bytes = hdc_readl(base, HDC_DMATC_R_ONLY(dma_ch));

		/* update actual bytes */
		urb->actual_length += bytes;

		if (urb->actual_length > urb->transfer_buffer_length)
			dev_err(priv->dev, "%s() urb corruption might exist!\n",
				__func__);

		/* check transfer request complete */
		if ((urb->actual_length >= urb->transfer_buffer_length) ||
		    hdc_read_bits(base, HDC_DMAS_R_ONLY(dma_ch),
				  BIT_DMA_SP, 1)) {
			ep->transfer_status = 0;
			hdc_write_bits(base, HDC_HOSTEVENTSRC,
				       BIT_TRANS_DONE(ep_ch), 1, 0);
			hdc_ep_complete_inten_set(ep, 0);

			goto done;
		}

		/* calculate this time transfer byte */
		bytes = (urb->transfer_buffer_length - urb->actual_length) <
			HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			(urb->transfer_buffer_length - urb->actual_length) :
			HDC_HCD_MAX_DMA_TRANSFER_SIZE;

		priv->bulkin_remain = 0;

		hcptr = VALUE(epctrl1, BIT_HCPTR, LEN_HCPTR);
		appptr = VALUE(epctrl1, BIT_APPPTR, LEN_APPPTR);
		bcnt = VALUE(epctrl1, BIT_EPCTRL_BNUM, LEN_EPCTRL_BNUM) + 1;

		if (hdc_status_trans(epctrl1)) {
			if ((bytes <= (max_packet_size * bcnt)))
				start_flag = 1;

			goto dma_trans;
		}

		result = hcptr < appptr ?
				bcnt + hcptr - appptr : hcptr - appptr;
		if (!hdc_empty_bit(epctrl1)) {
			if ((bytes <= (max_packet_size * bcnt))) {
				if (!((hcptr != appptr) &&
				      ((result * max_packet_size) < bytes)))
					start_flag = 1;
			}
			goto dma_trans;
		}
dma_trans:
		dma_buffer = ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
			priv->f_otg->handle_pio_urb_quirk) ?
			priv->bulk_in_dma + urb->actual_length :
			urb->transfer_dma + urb->actual_length;

		if (set_dma_transfer(ep, priv->dma_data[dma_ch].epbuf_daddr,
				     dma_buffer, bytes, false, start_flag))
			return 0;

		ep->transfer_status = -EPIPE;

done:
		hdc_dma_if_stop(ep);
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0,
			      hdc_toggle_bit(epctrl1));
		if (hdc_status_trans(epctrl1) || !hdc_empty_bit(epctrl1))
			priv->bulkin_remain = usb_pipedevice(urb->pipe);
		else
			priv->bulkin_remain = 0;

		if ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
		    priv->f_otg->handle_pio_urb_quirk)
			memcpy(urb->transfer_buffer, priv->bulk_in_buf,
			       urb->actual_length);
		return 1;
	}
#endif

	for (;;) {
		bytes = copy_ep_buf_to_urb(urb, ep);
		epctrl1 = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep_ch));

		/* check transfer request complete */
		if ((urb->actual_length >= urb->transfer_buffer_length) ||
		    (bytes % max_packet_size) || !bytes) {
			/* complete request, sometime it's short packet */
			hdc_ep_complete_inten_set(ep, 0);
			usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0,
				      hdc_toggle_bit(epctrl1));
			ep->transfer_status = 0;
			return 1;
		}

		/* check next bulk IN transfer start */
		if (hdc_empty_bit(epctrl1)) {
			/* wake up ep to receive remain data */
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
			return 0;
		}
	}
}

static const u8 end_bulk_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 epctrl1;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	u32 bytes;
	s8 dma_ch = ep->dma_ch;
	dma_addr_t dma_buffer;
#endif

	epctrl1 = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep_ch));

	/* check STALL or HALT status */
	if (hdc_status_stall(epctrl1) || hdc_status_halt(epctrl1)) {
		dev_err(priv->dev, "%s() STALL or HALT", __func__);
		hdc_ep_stop_and_clear(ep);
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		if (urb->transfer_buffer_length != 0) {
			hdc_hdmac_stop(ep);

			if (dma_ch != -1)
				goto done;
		}
#endif
		return 1;
	}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	if ((dma_ch != -1) &&
	    (urb->transfer_buffer_length != 0)) {
		/* get this time transfer byte */
		bytes = hdc_readl(base, HDC_DMATC_R_ONLY(dma_ch));

		/* update actual bytes */
		urb->actual_length += bytes;

		if (urb->actual_length > urb->transfer_buffer_length)
			dev_err(priv->dev, "%s() urb corruption might exist!\n",
				__func__);

		/* check transfer request complete */
		if (urb->actual_length >= urb->transfer_buffer_length) {
			ep->transfer_status = 0;
			goto done;
		}

		/* calculate this time transfer byte */
		bytes = (urb->transfer_buffer_length - urb->actual_length) <
			HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			(urb->transfer_buffer_length - urb->actual_length) :
			HDC_HCD_MAX_DMA_TRANSFER_SIZE;

		dma_buffer = ((urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) &&
			priv->f_otg->handle_pio_urb_quirk) ?
			priv->bulk_out_dma + urb->actual_length :
			urb->transfer_dma + urb->actual_length;

		if (set_dma_transfer(ep, dma_buffer,
				     priv->dma_data[dma_ch].epbuf_daddr,
				     bytes, false, false))
			return 0;

		ep->transfer_status = -EPIPE;

done:
		hdc_dma_if_stop(ep);
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1,
			      hdc_toggle_bit(epctrl1));

		return 1;
	}
#endif

	/* update actual bytes */
	urb->actual_length +=
			(urb->transfer_buffer_length - urb->actual_length) <
			urb->ep->desc.wMaxPacketSize ?
			urb->transfer_buffer_length - urb->actual_length :
			urb->ep->desc.wMaxPacketSize;

	/* check transfer request complete */
	if (urb->actual_length >= urb->transfer_buffer_length) {
		/* complete request */
		hdc_ep_complete_inten_set(ep, 0);
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1,
			      hdc_toggle_bit(epctrl1));
		ep->transfer_status = 0;
		return 1;
	}

	copy_urb_to_ep_buf(urb, ep);

	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 0;
}

static const u8 end_interrupt_in_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	u32 bytes;

	/* check STALL or HALT status */
	if (hdc_ep_status_stall(ep) || hdc_ep_status_halt(ep)) {
		hdc_ep_stop_and_clear(ep);
		return 1;
	}

	for (;;) {
		bytes = copy_ep_buf_to_urb(urb, ep);

		/* check transfer request complete */
		if ((urb->actual_length >= urb->transfer_buffer_length) ||
		    (bytes % urb->ep->desc.wMaxPacketSize) || !bytes) {
			/* complete request */
			hdc_ep_complete_inten_set(ep, 0);
			usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0,
				      hdc_ep_toggle_bit(ep));
			ep->transfer_status = 0;
			return 1;
		}

		/* check next interrupt IN transfer start */
		if (((urb->actual_length + urb->ep->desc.wMaxPacketSize) <
			urb->transfer_buffer_length) || hdc_ep_empty_bit(ep)) {
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
			return 0;
		}
	}
}

static const u8 end_interrupt_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	/* check STALL or HALT status */
	if (hdc_ep_status_stall(ep) || hdc_ep_status_halt(ep)
		) {
		hdc_ep_stop_and_clear(ep);
		return 1;
	}

	/* update actual bytes */
	urb->actual_length += (urb->transfer_buffer_length -
					urb->actual_length) <
			urb->ep->desc.wMaxPacketSize ?
			urb->transfer_buffer_length - urb->actual_length :
			urb->ep->desc.wMaxPacketSize;

	/* check transfer request complete */
	if (urb->actual_length >= urb->transfer_buffer_length) {
		/* complete request */
		hdc_ep_complete_inten_set(ep, 0);
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1,
			      hdc_ep_toggle_bit(ep));
		ep->transfer_status = 0;
		return 1;
	}

	copy_urb_to_ep_buf(urb, ep);

	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 0;
}

static const u8 end_isochronous_in_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	/* check HALT status */
	if (hdc_ep_status_halt(ep)) {
		hdc_ep_stop_and_clear(ep);
		return 1;
	}

	for (;;) {
		copy_ep_buf_to_urb(urb, ep);

		/* check transfer request complete */
		if (urb->number_of_packets <= ep->iso_packets) {
			/* complete request */
			hdc_ep_complete_inten_set(ep, 0);
			usb_settoggle(urb->dev,
				      usb_pipeendpoint(urb->pipe), 0, 0);
			ep->transfer_status = 0;
			return 1;
		}

		/* check next interrupt IN transfer start */
		if (((urb->actual_length + urb->ep->desc.wMaxPacketSize) <
		    urb->transfer_buffer_length) || hdc_ep_empty_bit(ep)) {
			hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);
			return 0;
		}
	}
}

static const u8 end_isochronous_out_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct urb *urb = ep->req->urb;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;

	/* check HALT status */
	if (hdc_ep_status_halt(ep)) {
		hdc_ep_stop_and_clear(ep);
		return 1;
	}

	/* update actual bytes */
	urb->actual_length += urb->iso_frame_desc[ep->iso_packets].length;
	urb->iso_frame_desc[ep->iso_packets].actual_length =
			urb->iso_frame_desc[ep->iso_packets].length;

	/* set request status */
	urb->iso_frame_desc[ep->iso_packets].status = 0;

	/* update transfer packet count */
	ep->iso_packets++;

	/* check transfer request complete */
	if (urb->number_of_packets <= ep->iso_packets) {
		/* complete request */
		hdc_ep_complete_inten_set(ep, 0);
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1, 0);
		ep->transfer_status = 0;
		return 1;
	}

	copy_urb_to_ep_buf(urb, ep);

	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_START, 1, 1);

	return 0;
}

static void on_end_transfer(struct f_usb20hdc_hcd_ep *ep)
{
	static const u8 (* const end_transfer_function[])(
				struct f_usb20hdc_hcd_ep *) = {
		end_control_transfer,
		end_bulk_in_transfer,
		end_bulk_out_transfer,
		end_interrupt_in_transfer,
		end_interrupt_out_transfer,
		end_isochronous_in_transfer,
		end_isochronous_out_transfer,
	};
	struct f_usb20hdc_hcd *priv = ep->priv;
	void __iomem *base = priv->reg_base;
	struct f_usb20hdc_hcd_req *curr;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct list_head *p;
#endif

	/* check transfer continue */
	if (!end_transfer_function[ep->transfer_type](ep)) {
		/* to be continued */
		return;
	}

	/* check dynamic allocated endpoint still necessary */
	if (usb_pipetype(ep->req->urb->pipe) == PIPE_INTERRUPT) {
		priv->eps_inuse_map &= ~(0x0001 << ep->ep_ch);
	} else if (usb_pipetype(ep->req->urb->pipe) ==
		PIPE_ISOCHRONOUS && list_is_singular(&ep->queue)) {
		priv->eps_inuse_map &= ~(0x0001 << ep->ep_ch);
		ep->inuse_devnum = 0;
		ep->inuse_epnum = 0;
	}

	/* notify request complete */
	notify_transfer_request_complete(ep, ep->req, ep->transfer_status);

	/* check if next queue empty or disconnect*/
	if (list_empty(&ep->queue) ||
	    !hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1))
		return;

	if (ep->req && ep->req->request_execute)
		return;

	/* get next request */
	curr = list_entry(ep->queue.next, struct f_usb20hdc_hcd_req, queue);
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	if (ep->transfer_type == TRANSFER_TYPE_BULK_IN &&
	    (priv->bulkin_remain != 0)) {
		list_for_each(p, &ep->queue) {
			curr = list_entry(p, struct f_usb20hdc_hcd_req, queue);
			if (curr && (priv->bulkin_remain ==
				usb_pipedevice(curr->urb->pipe)))
				break;
		}

		if (priv->bulkin_remain != usb_pipedevice(curr->urb->pipe))
			return;
	}
#endif

	/* save request */
	ep->req = curr;

	/* check the got next request a request under current execution */
	if (!ep->req->request_execute)
		set_transfer_function[ep->transfer_type](ep);
}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static void end_dma_transfer(u32 channel, void *data, int state)
{
	struct f_usb20hdc_hcd_ep *ep = data;
	struct f_usb20hdc_hcd *priv = ep->priv;
	struct device *dev = priv->dev;
	struct f_usb20hdc_hcd_req *req;
	void *base = priv->reg_base;
	u8 ep_ch = ep->ep_ch;
	s8 dma_ch = ep->dma_ch;

	spin_lock(&priv->lock);

	if (unlikely(!ep || !ep->req || channel !=
			priv->dma_data[dma_ch].hdmac_ch)) {
		spin_unlock(&priv->lock);
		return;
	}

	req = ep->req;

	/* DMA transfer end state factor */
	switch (state) {
	case HDMACB_SS_ADD_OVERFLOW:
	case HDMACB_SS_SOURCE_ACCESS_ERROR:
	case HDMACB_SS_DESTINATION_ACCESS_ERROR:
		dev_err(dev, "%s() ep %u aborted at request 0x%p actual_length %u state=0x%x",
			__func__, ep_ch, req, req->urb->actual_length, state);

		/* disable DMA transfer */
		hdc_dma_if_stop(ep);

		/* notify request complete */
		notify_transfer_request_complete(ep, req, -EPIPE);

		/* check if next queue empty or disconnect*/
		if (list_empty(&ep->queue) ||
		    !hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1))
			break;

		/* get next request */
		req = list_entry(ep->queue.next, struct f_usb20hdc_hcd_req,
				 queue);

		/* save request */
		ep->req = req;

		/* check the next request is under current execution */
		if (req->request_execute)
			break;

		/* set transfer request */
		set_transfer_function[ep->transfer_type](ep);

		break;
	case HDMACB_SS_TRANSFER_STOP_REQUEST:
	case HDMACB_SS_NORMAL_END:
		/* process in transfer end */
		if (usb_pipein(req->urb->pipe)) {
			on_end_transfer(ep);
		} else {
			/*
			 * process of transfer end need be done
			 * when usb controller's dma interrupt is handled before
			 * hdmac's interrupt,
			 */
			ep->req->request_execute = 0;
			ep->req->dmac_int_occurred = 1;
			if (ep->req->usb_dma_int_occurred == 1)
				on_end_transfer(ep);
		}
		break;
	default:
		break;
	}

	spin_unlock(&priv->lock);
}

static int on_dma_int(struct f_usb20hdc_hcd *priv, u32 ch)
{
	void *base = priv->reg_base;
	struct f_usb20hdc_hcd_ep *ep;

	if (!hdc_read_bits(base, HDC_INTS, BIT_DMA_INT(ch), 1) ||
	    !hdc_read_bits(base, HDC_INTEN, BIT_DMA_INTEN(ch), 1))
		return IRQ_NONE;

	/* clear trans_done interrupt factor */
	hdc_write_bits(base, HDC_INTS, BIT_DMA_INT(ch), 1, 0);

	ep = &priv->ep[priv->dma_data[ch].ep_ch];

	if (!ep->req)
		return IRQ_HANDLED;

	if (!usb_pipein(ep->req->urb->pipe)) {
		/*
		 * make sure the process of transfer end is done
		 * after hdmac's interrupt occurred,
		 * because the hdmac driver need to change its
		 * state at interrupt handler.
		 */
		ep->req->usb_dma_int_occurred = 1;
		if (ep->req->dmac_int_occurred == 1)
			on_end_transfer(ep);
	} else {
		/* check transfer request complete */
		if ((hdc_readl(base, HDC_DMATC_R_ONLY(ch)) <
		    hdc_readl(base, HDC_DMATCI(ch))) &&
			hdc_read_bits(base, HDC_DMAS_R_ONLY(ch),
				      BIT_DMA_SP, 1)) {
			hdc_hdmac_stop(ep);
		}
	}

	return IRQ_HANDLED;
}
#endif

static int on_ep_int(struct f_usb20hdc_hcd *priv, u32 ch)
{
	void *base = priv->reg_base;
	struct f_usb20hdc_hcd_ep *ep = &priv->ep[ch];
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	u32 epctrl1;
#endif

	if (!hdc_read_bits(base, HDC_HOSTEVENTSRC, BIT_TRANS_DONE(ch), 1) ||
	    !hdc_read_bits(base, HDC_HOSTINTEN, BIT_TRANS_DONE_INTEN(ch), 1)) {
		return IRQ_NONE;
	}

	/* clear trans_done interrupt factor */
	hdc_write_bits(base, HDC_HOSTEVENTSRC, BIT_TRANS_DONE(ch), 1, 0);

	if (!ep->req)
		return IRQ_HANDLED;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	epctrl1 = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep->ep_ch));
	if ((ep->dma_ch != -1) && !hdc_status_stall(epctrl1) &&
	    !hdc_status_halt(epctrl1) && ep->req->urb->transfer_buffer_length) {
		return IRQ_HANDLED;
	}
#endif
	/* process transfer end */
	on_end_transfer(ep);

	return IRQ_HANDLED;
}

static irqreturn_t hdc_host_irq(struct usb_hcd *hcd)
{
	u32 ch, portstsc, hostinten;
	struct f_usb20hdc_hcd *priv;
	void *base;
	struct device *dev = hcd->self.controller;

	if (unlikely(!hcd))
		return IRQ_NONE;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	/* check F_USB20HDC controller interrupt request assert */
	if ((unlikely(hcd->irq != priv->irq)) ||
	    (unlikely(!is_host_mode_usage(base))) ||
		(unlikely((hdc_read_bits(base, HDC_INTS, BIT_HOST_INT, 1)) &&
			(!hdc_read_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1)))))
		return IRQ_NONE;

	/* get spin lock */
	spin_lock(&priv->lock);

	/* queue a HC resume work */
	if (hcd->state == HC_STATE_SUSPENDED) {
		dev_dbg(dev, "%s() port event is going to wake up HC\n",
			__func__);
		usb_hcd_resume_root_hub(hcd);
	}

	/* port status change interrupt factor */
	if (likely(
		!hdc_read_bits(base, HDC_HOSTEVENTSRC, BIT_PORT_STATUS_C, 1))) {
		goto TRANSFER_DONE;
	}

	portstsc = hdc_readl(base, HDC_PORTSTSC);
	hostinten = hdc_readl(base, HDC_HOSTINTEN);

	/* port connection change interrupt factor */
	if (VALUE(portstsc, BIT_CONNECTION_C, 1) &&
	    VALUE(hostinten, BIT_CONNECT_C_INTEN, 1)) {
		dev_dbg(dev, "%s() port connection change\n", __func__);

		/* disable port connection change interrupt */
		dbg_print_connection(hcd->self.controller, base);
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_CONNECT_C_INTEN, 1, 0);
		dbg_print_connection(hcd->self.controller, base);

		goto END_RH_POLL_STATUS;
	}

	/* port enable change interrupt factor */
	if (VALUE(portstsc, BIT_ENABLE_C, 1) &&
	    VALUE(hostinten, BIT_ENABLE_C_INTEN, 1)) {
		dev_dbg(dev, "%s() port enable change\n", __func__);

		/* disable port enable change interrupt */
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_ENABLE_C_INTEN, 1, 0);

		goto END_RH_POLL_STATUS;
	}

	/* port suspend change interrupt factor */
	if (VALUE(portstsc, BIT_SUSPEND_C, 1) &&
	    VALUE(hostinten, BIT_SUSPEND_C_INTEN, 1)) {
		dev_dbg(dev, "%s() suspend change\n", __func__);

		/* disable port suspend change interrupt */
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_SUSPEND_C_INTEN, 1, 0);

		goto END_RH_POLL_STATUS;
	}

	/* port over current change interrupt factor */
	if (VALUE(portstsc, BIT_OV_CURR_C, 1) &&
	    VALUE(hostinten, BIT_OV_CURRENT_C_INTEN, 1)) {
		dev_dbg(dev, "%s() over_current change\n", __func__);
		/* disable port over current change interrupt */
		hdc_write_bits(base, HDC_HOSTINTEN,
			       BIT_OV_CURRENT_C_INTEN, 1, 0);

		goto END_RH_POLL_STATUS;
	}

	/* port reset change interrupt factor */
	if (VALUE(portstsc, BIT_RESET_C, 1) &&
	    VALUE(hostinten, BIT_RESET_C_INTEN, 1)) {
		dev_dbg(dev, "%s() reset change\n", __func__);
		/* disable port reset change interrupt */
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_RESET_C_INTEN, 1, 0);

		goto END_RH_POLL_STATUS;
	}

TRANSFER_DONE:
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		/* DMA channel x interrupt factor */
		for (ch = HDC_DMA_CH1; ch < HDC_MAX_DMA_CH; ch++) {
			if (on_dma_int(priv, ch) == IRQ_HANDLED)
				goto END_UNLOCK;
		}
#endif

		/* endpoint x interrupt factor */
		for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++) {
			if (on_ep_int(priv, ch) == IRQ_HANDLED)
				goto END_UNLOCK;
		}

END_UNLOCK:
	spin_unlock(&priv->lock);
	return IRQ_HANDLED;

END_RH_POLL_STATUS:
	spin_unlock(&priv->lock);
	usb_hcd_poll_rh_status(hcd);
	return IRQ_HANDLED;
}

static int hdc_host_reset(struct usb_hcd *hcd)
{
	u32 i;
	struct f_usb20hdc_hcd *priv;
	void __iomem *base;
	unsigned long flags;

	if (unlikely(!hcd))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(base))) {
		dev_err(priv->dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* halt F_USB20HDC controller */
	hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 0);
	set_all_interrupt_factor(base, 0, true);

	for (i = EP0; i < HDC_HCD_MAX_EP; i++)
		hdc_ep_set_stop(base, i);
	hdc_set_host_run(base, 0);

	/* initialize F_USB20HDC controller */
	initialize_hcd_controller(priv);

	/* disable port power */
	if (priv->wakeup_from_poweroff == 0)
		hdc_set_vbus(base, 0);

	/*
	 * Since a F_USB20HDC controller is a controller corresponding
	 * to dual speed,it sets a transaction translator flag
	 */
	hcd->has_tt = 1;

	priv->next_statechange = jiffies;

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int hdc_host_start(struct usb_hcd *hcd)
{
	u32 ch;
	struct f_usb20hdc_hcd *priv;
	void __iomem *base;
	unsigned long flags;

	if (unlikely(!hcd))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* set hdc parameter */
	hcd->uses_new_polling = 1;
	hcd->state = HC_STATE_RUNNING;
	clear_bit(HCD_FLAG_POLL_RH, &hcd->flags);

	/* initialize endpoint configure */
	initialize_endpoint_configure(priv);

	switch (priv->f_otg->variant) {
	case FHDC_VARIANT_LAP:
		writel(1, base + USB_TOP_IDVBUSCTL);
		break;
	case FHDC_VARIANT_ORIG:
		break;
	}

	/* start F_USB20HDC controller */
	hdc_write_bits(base, HDC_OTGC, BIT_DM_PULL_DOWN, 1, 1);
	hdc_write_bits(base, HDC_OTGC, BIT_DP_PULL_DOWN, 1, 1);
	hdc_write_bits(base, HDC_MODE, BIT_HOST_EN, 1, 1);
	hdc_set_vbus(base, 1);

	/* make the endpoint transfer link list */
	initialize_td_schedule_list(base);
	for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++)
		hdc_write_bits(base, HDC_EPCMD(ch), BIT_NLINKINVALID, 1, 0);

	/* enable interrupt factor */
	hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 1);
	set_all_interrupt_factor(base, 1, false);
	hdc_set_host_run(base, 1);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int hdc_host_get_frame_number(struct usb_hcd *hcd)
{
	struct f_usb20hdc_hcd *priv;
	u16 frame_number;
	unsigned long flags;

	if (unlikely(!hcd))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EPROTO;
	}

	/* get frame number */
	frame_number = hdc_sof_value(priv->reg_base);

	spin_unlock_irqrestore(&priv->lock, flags);

	return (int)frame_number;
}

static void hdc_host_stop(struct usb_hcd *hcd)
{
	u32 ch;
	struct f_usb20hdc_hcd *priv;
	unsigned long flags;
	void __iomem *base;

	if (unlikely(!hcd))
		return;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(base))) {
		dev_err(priv->dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return;
	}

	/* disable interrupt factor */
	hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 0);
	set_all_interrupt_factor(base, 0, true);

	/* clear interrupt factor */
	clear_all_int_status_change(base, true);

	/* stop F_USB20HDC controller */
	hdc_set_host_run(base, 0);

	/* reset F_USB20HDC controller */
	initialize_hcd_controller(priv);

	/* dequeue all previous transfer request */
	for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++)
		dequeue_all_request(&priv->ep[ch], -ESHUTDOWN);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int hdc_host_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
				gfp_t mem_flags)
{
	struct f_usb20hdc_hcd *priv;
	struct f_usb20hdc_hcd_req *req;
	struct f_usb20hdc_hcd_ep *ep;
	struct usb_ctrlrequest *dev_req;
	u16 eps_inuse_mask = 0, maxp = 0;
	u8 ep_ch, dynamic_alloc = 0;
	unsigned long flags;
	int result;
	void __iomem *base;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct list_head *p;
	struct f_usb20hdc_hcd_req *curr;
#endif
	struct device *dev;
#ifdef VIRTUAL_DEVICE_ADDRESS
	int addr;
#endif

	if (unlikely((!hcd) || (!urb)))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);
	dev = hcd->self.controller;
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(base))) {
		dev_err(dev, "%s() device mode is usage\n", __func__);
		result = -ESHUTDOWN;
		goto error_not_linked;
	}

	/* check if bus disconnect*/
	if (!hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1) ||
	    (urb->dev->state == USB_STATE_NOTATTACHED)) {
		dev_err(dev, "%s() device is disconnected\n", __func__);
		result = -ESHUTDOWN;
		goto error_not_linked;
	}

	if (unlikely(!urb->dev || !urb->ep ||
#ifndef CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER
		     (urb->transfer_buffer_length && !urb->transfer_buffer) ||
#endif
			((usb_pipetype(urb->pipe) == PIPE_CONTROL) &&
			!urb->setup_packet))) {
		dev_err(dev, "%s() request parameter is error\n", __func__);
		if (urb->transfer_buffer_length && (!urb->transfer_buffer))
			dev_err(dev, "%s() trans_buf NULL\n", __func__);
		result = -EINVAL;
		goto error_not_linked;
	}

	/* solution for restriction of only 4 bits device address in this IP */
	if (urb->setup_packet) {
		dev_req = (struct usb_ctrlrequest *)urb->setup_packet;
		if ((dev_req->bRequestType == 0x0) &&
		    (dev_req->bRequest == 0x5)) {
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* setup address request */
			addr = create_mapped_addr(hcd, urb, dev_req->wValue);
			if (addr < 0) {
				dev_err(dev, "%s() logical addr %d can't be mapped\n",
					__func__, dev_req->wValue);
				result = -EINVAL;
				goto error_not_linked;
			} else {
				dev_req->wValue = addr;
			}
#else
			if (dev_req->wValue > 15) {
				dev_err(dev, "%s() logical addr %d can't be set\n",
					__func__, dev_req->wValue);
				result = -EINVAL;
				goto error_not_linked;
			}
#endif
		}
	}

	/* set URB parameter */
	urb->status = -EINPROGRESS;
	urb->actual_length = 0;
	urb->error_count = 0;

	/* link URB to ep */
	result = usb_hcd_link_urb_to_ep(hcd, urb);
	if (result)
		goto error_not_linked;

	/* set endpoint parameter */
	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		ep_ch = EP0;
		ep = &priv->ep[ep_ch];
		ep->transfer_type = TRANSFER_TYPE_CONTROL;
		break;
	case PIPE_BULK:
		if (usb_pipein(urb->pipe)) {
			ep_ch = EP1;
			ep = &priv->ep[ep_ch];
			ep->transfer_type = TRANSFER_TYPE_BULK_IN;
		} else {
			ep_ch = EP2;
			ep = &priv->ep[ep_ch];
			ep->transfer_type = TRANSFER_TYPE_BULK_OUT;
		}
		break;
	case PIPE_INTERRUPT:
		for (ep_ch = EP3; ep_ch < HDC_HCD_MAX_EP; ep_ch++) {
			eps_inuse_mask = 0x0001 << ep_ch;
			if (!(priv->eps_inuse_map & eps_inuse_mask)) {
				priv->eps_inuse_map |= eps_inuse_mask;
				dynamic_alloc = 1;
				break;
			}
		}
		if (ep_ch == HDC_HCD_MAX_EP) {
			dev_err(dev, "%s() endpoint is out of use\n", __func__);
			result = -ENOSPC;
			goto error;
		}
		if (usb_pipein(urb->pipe))
			set_endpoint_type(priv, ep_ch,
					  TRANSFER_TYPE_INTERRUPT_IN);
		else
			set_endpoint_type(priv, ep_ch,
					  TRANSFER_TYPE_INTERRUPT_OUT);
		break;
	case PIPE_ISOCHRONOUS:
		for (ep_ch = EP3; ep_ch < HDC_HCD_MAX_EP; ep_ch++) {
			ep = &priv->ep[ep_ch];
			if (!list_empty(&ep->queue) &&
			    (ep->inuse_devnum == usb_pipedevice(urb->pipe)) &&
			    (ep->inuse_epnum == usb_pipeendpoint(urb->pipe))) {
				dynamic_alloc = 1;
				break;
			}
		}
		if (!dynamic_alloc) {
			for (ep_ch = EP3; ep_ch < HDC_HCD_MAX_EP; ep_ch++) {
				eps_inuse_mask = 0x0001 << ep_ch;
				if (!(priv->eps_inuse_map & eps_inuse_mask)) {
					priv->eps_inuse_map |= eps_inuse_mask;
					dynamic_alloc = 1;
					break;
				}
			}
		}
		if (ep_ch == HDC_HCD_MAX_EP) {
			dev_err(dev, "%s() endpoint is out of use\n", __func__);
			result = -ENOSPC;
			goto error;
		}
		priv->ep[ep_ch].inuse_devnum = usb_pipedevice(urb->pipe);
		priv->ep[ep_ch].inuse_epnum = usb_pipeendpoint(urb->pipe);
		if (usb_pipein(urb->pipe))
			set_endpoint_type(priv, ep_ch,
					  TRANSFER_TYPE_ISOCHRONOUS_IN);
		else
			set_endpoint_type(priv, ep_ch,
					  TRANSFER_TYPE_ISOCHRONOUS_OUT);
		break;
	default:
		dev_err(dev, "%s() request parameter is error\n", __func__);
		result = -EINVAL;
		goto error;
	}

	/* get allocated endpoint */
	ep = &priv->ep[ep_ch];

	/* get the value of wMaxPacketSize */
	maxp = usb_maxpacket(urb->dev, urb->pipe, !usb_pipein(urb->pipe));
	if (is_highbandwidth(maxp)) {
		/* hardware does not support high bandwidth */
		dev_err(dev, "%s() Host does not support high bandwidth\n",
			__func__);
		result = -EOPNOTSUPP;
		goto error;
	}

	/* check maximum packet size, only 0x07ff bits are for packet size */
	if (ep->max_packet_size < max_packet(maxp)) {
		dev_err(dev, "%s() MaxPacketSize (%d) is over than (%d)\n",
			__func__, max_packet(maxp), ep->max_packet_size);
		result = -ENOSPC;
		goto error;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	/* allocate request memory and zero clear request memory */
	req = kzalloc(sizeof(*req), mem_flags);

	spin_lock_irqsave(&priv->lock, flags);

	if (!req) {
		result = -ENOMEM;
		goto error;
	}
	INIT_LIST_HEAD(&req->queue);

	/* save URB */
	req->urb = urb;
	req->ep_ch = ep->ep_ch;
	req->maxpacket = max_packet(maxp);
	urb->hcpriv = req;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	/* check current queue execute */
	if (!list_empty(&ep->queue)) {
		if (ep->transfer_type != TRANSFER_TYPE_BULK_IN)
			goto add_list_tail;

		if (ep->req && ep->req->request_execute) {
			goto add_list_tail;
		} else if (priv->bulkin_remain != 0 && priv->bulkin_remain ==
					usb_pipedevice(urb->pipe)) {
			list_for_each(p, &ep->queue) {
				curr = list_entry(p, struct f_usb20hdc_hcd_req,
						  queue);
				if (priv->bulkin_remain ==
				    usb_pipedevice(curr->urb->pipe))
					goto add_list_tail;
			}
		} else {
			goto add_list_tail;
		}
	} else if (ep->transfer_type == TRANSFER_TYPE_BULK_IN) {
		if (priv->bulkin_remain != 0 && priv->bulkin_remain !=
				usb_pipedevice(urb->pipe))
			goto add_list_tail;
	}
#else
	/* check current queue execute */
	if (!list_empty(&ep->queue))
		goto add_list_tail;
#endif

	/* save request */
	ep->req = req;

	/* set transfer */
	set_transfer_function[ep->transfer_type](ep);

add_list_tail:
	/* add list tail */
	list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;

error:
	if (dynamic_alloc) {
		priv->eps_inuse_map &= ~eps_inuse_mask;
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
			ep->inuse_devnum = 0;
			ep->inuse_epnum = 0;
		}
	}

	usb_hcd_unlink_urb_from_ep(hcd, urb);

error_not_linked:
	spin_unlock_irqrestore(&priv->lock, flags);

	dev_err(dev, "%s() is ended with err\n", __func__);

	return result;
}

static int hdc_host_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
				int status)
{
	struct f_usb20hdc_hcd *priv;
	struct f_usb20hdc_hcd_req *req;
	struct f_usb20hdc_hcd_ep *ep;
	void *base;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch;
#endif
	u8 ep_ch;
	unsigned long flags;

	if (unlikely((!hcd) || (!urb)))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(priv->reg_base))) {
		dev_err(priv->dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* ckeck URB unlink */
	if (usb_hcd_check_unlink_urb(hcd, urb, status)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		return 0;
	}

	req = urb->hcpriv;
	ep = &priv->ep[req->ep_ch];
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	dma_ch = ep->dma_ch;
#endif
	ep_ch = ep->ep_ch;

	/* check list entry */
	list_for_each_entry(req, &ep->queue, queue) {
		if ((req) && (req->urb == urb))
			break;
	}

	/* check dequeue request mismatch */
	if (unlikely((!req) || (req->urb != urb))) {
		dev_err(priv->dev, "%s() ep request is mismatch\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	/* abort request transfer */
	if (req->request_execute) {
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
		if (dma_ch != -1) {
			/* abort DMA transfer */
			hdc_dma_if_stop(ep);
			hdc_hdmac_stop(ep);
		}
#endif
		hdc_ep_complete_inten_set(ep, 0);
		hdc_ep_set_stop(base, ep_ch);
		hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_INIT, 1, 1);
	}

	/* check dynamic allocated endpoint still necessary */
	if (usb_pipetype(urb->pipe) == PIPE_INTERRUPT) {
		priv->eps_inuse_map &= ~(0x0001 << ep_ch);
	} else if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS &&
					list_is_singular(&ep->queue)) {
		priv->eps_inuse_map &= ~(0x0001 << ep_ch);
		ep->inuse_devnum = 0;
		ep->inuse_epnum = 0;
	}

	/* notify request complete */
	notify_transfer_request_complete(ep, req, -ESHUTDOWN);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int hdc_host_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct f_usb20hdc_hcd *priv;
	struct device *dev = hcd->self.controller;
	unsigned long flags;
	void __iomem *base;

	if (unlikely((!hcd) || (!buf)))
		return 0;

	dev_dbg(dev, "%s() is started from %pS\n", __func__,
		__builtin_return_address(0));

	/* if !USB_SUSPEND, root hub timers won't get shut down ... */
	if (!HC_IS_RUNNING(hcd->state)) {
		dev_dbg(dev, "%s() HC is not running\n", __func__);
		return 0;
	}

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;

	spin_lock_irqsave(&priv->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(base))) {
		dev_err(dev, "%s() device mode is usage\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return -ESHUTDOWN;
	}

	/* initialize hub status data */
	*buf = 0;

	/* check port status change */
	if (!hdc_read_bits(base, HDC_HOSTEVENTSRC, BIT_PORT_STATUS_C, 1)) {
		dev_dbg(dev, "%s() No port event\n", __func__);
		spin_unlock_irqrestore(&priv->lock, flags);
		return 0;
	}

	/* no hub change reports (bit 0) for now (power, ...) */
	*buf = 1 << 1;

	spin_unlock_irqrestore(&priv->lock, flags);

	return 1;
}

static void do_hub_csc_event(struct usb_hcd *hcd, u32 *status)
{
	int i;
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	void __iomem *base = priv->reg_base;

	if (priv->wakeup_req == 1) {
		priv->wakeup_req = 0;
		/*
		 * disabling port make usb stack can run
		 * configuring device from scratch.
		 */
		hdc_write_bits(base, HDC_PORTSC, BIT_ENABLE_REQ, 1, 0);
		while (hdc_read_bits(base, HDC_PORTSC, BIT_ENABLE_RHS, 1))
			;
	}

	*status |= USB_PORT_STAT_C_CONNECTION << 16;
	if (hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1)) {
		for (i = EP1; i < HDC_HCD_MAX_EP; i++)
			hdc_write_bits(base, HDC_EPCMD(i),
				       BIT_TOGGLE_CLR, 1, 1);
	} else {
#ifdef VIRTUAL_DEVICE_ADDRESS
		free_all_mapped_addr(hcd);
#endif
		/*
		 * abort previous transfer and dequeue
		 * all previous transfer request
		 */
		for (i = EP0; i < HDC_HCD_MAX_EP; i++) {
			hdc_ep_stop_and_clear(&priv->ep[i]);
			dequeue_all_request(&priv->ep[i], -ESHUTDOWN);
		}
	}
}

static int do_hub_clear_port_feature(struct usb_hcd *hcd, u16 w_value)
{
	int ret = 0;
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	void __iomem *base = priv->reg_base;

	switch (w_value) {
	case USB_PORT_FEAT_ENABLE:
		hdc_write_bits(base, HDC_PORTSC, BIT_ENABLE_REQ, 1, 0);
		break;
	case USB_PORT_FEAT_C_ENABLE:
		hdc_write_bits(base, HDC_PORTSTSC, BIT_ENABLE_C, 1, 0);
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_ENABLE_C_INTEN, 1, 1);
		break;
	case USB_PORT_FEAT_SUSPEND:
		break;
	case USB_PORT_FEAT_C_SUSPEND:
		hdc_write_bits(base, HDC_PORTSTSC, BIT_SUSPEND_C, 1, 0);
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_SUSPEND_C_INTEN, 1, 1);
		break;
	case USB_PORT_FEAT_POWER:
		hdc_set_vbus(base, 0);
		break;
	case USB_PORT_FEAT_C_CONNECTION:
		hdc_write_bits(base, HDC_PORTSTSC, BIT_CONNECTION_C, 1, 0);
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_CONNECT_C_INTEN, 1, 1);
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		hdc_write_bits(base, HDC_PORTSTSC, BIT_OV_CURR_C, 1, 0);
		hdc_write_bits(base, HDC_HOSTINTEN,
			       BIT_OV_CURRENT_C_INTEN, 1, 1);
		break;
	case USB_PORT_FEAT_C_RESET:
		break;
	default:
		ret = -EPIPE;
	}

	return ret;
}

static int hdc_host_hub_control(struct usb_hcd *hcd, u16 type_req, u16 w_value,
				u16 w_index, char *buf, u16 w_length)
{
	struct f_usb20hdc_hcd *priv;
	void *base;
	u32 status, selector;
	unsigned long flags;
	struct device *dev;
	int ret;
	struct usb_hub_descriptor *desc = (struct usb_hub_descriptor *)buf;

	if (unlikely((!hcd) || (!buf)))
		return -EINVAL;

	priv = hcd_to_f_usb20hdc(hcd);
	base = priv->reg_base;
	dev = hcd->self.controller;

	spin_lock_irqsave(&priv->lock, flags);

	if (unlikely(is_device_mode_usage(priv->reg_base))) {
		dev_err(dev, "%s() device mode is usage\n", __func__);
		ret = -ESHUTDOWN;
		goto err_unlock;
	}

	switch (type_req) {
	case ClearHubFeature:
		switch (w_value) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			ret = -EPIPE;
			goto err_unlock;
		}
		break;
	case ClearPortFeature:
		if ((!w_index) || (w_index > HDC_HCD_ROOT_HUB_MAX_PORT)) {
			ret = -EPIPE;
			goto err_unlock;
		}
		w_index--;

		ret = do_hub_clear_port_feature(hcd, w_value);
		if (ret == -EPIPE)
			goto err_unlock;

		break;
	case GetHubDescriptor:
		/* create hub descriptor */
		memset(desc, 0, sizeof(*desc));
		desc->bDescLength = 9;
		desc->bDescriptorType = 0x29;
		desc->bNbrPorts = 1;
		desc->wHubCharacteristics = cpu_to_le16(0x0009);
		desc->bPwrOn2PwrGood = 10;
		desc->u.hs.DeviceRemovable[1] = 0xff;
		break;
	case GetHubStatus:
		/* no hub-wide feature/status flags */
		memset(buf, 0, 4);
		break;
	case GetPortStatus:
		if ((!w_index) || (w_index > HDC_HCD_ROOT_HUB_MAX_PORT)) {
			ret = -EPIPE;
			goto err_unlock;
		}
		w_index--;
		status = 0;

		if ((priv->wakeup_req == 1) ||
		    hdc_read_bits(base, HDC_PORTSTSC, BIT_CONNECTION_C, 1))
			do_hub_csc_event(hcd, &status);

		if (hdc_read_bits(base, HDC_PORTSTSC, BIT_ENABLE_C, 1))
			status |= USB_PORT_STAT_C_ENABLE << 16;

		if (hdc_read_bits(base, HDC_PORTSTSC, BIT_OV_CURR_C, 1)) {
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;
			hdc_set_vbus(base, 0);
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if (hdc_read_bits(base, HDC_PORTSTSC, BIT_RESET_C, 1)) {
			hdc_write_bits(base, HDC_PORTSTSC, BIT_RESET_C, 1, 0);
			hdc_write_bits(base, HDC_HOSTINTEN,
				       BIT_RESET_C_INTEN, 1, 1);
			status |= USB_PORT_STAT_C_RESET << 16;
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */
		if (hdc_read_bits(base, HDC_PORTSC, BIT_CONNECTION_RHS, 1)) {
			status |= USB_PORT_STAT_CONNECTION;
			status |=
				hdc_read_bits(base, HDC_PORTSC, BIT_HS_RHS, 1) ?
				USB_PORT_STAT_HIGH_SPEED :
				hdc_read_bits(base, HDC_PORTSC, BIT_LS_RHS, 1) ?
				USB_PORT_STAT_LOW_SPEED : 0;
		}

		if (hdc_read_bits(base, HDC_PORTSC, BIT_ENABLE_RHS, 1)) {
			status |= USB_PORT_STAT_ENABLE;
#ifdef HUB_RESET_WORKAROUND
			if (priv->reset_start == 1) {
				/*reset has been finish*/
				priv->reset_start = 0;
				priv->reset_failed_cnt = 0;
			}
#endif
		} else {
#ifdef HUB_RESET_WORKAROUND
			if (priv->reset_start == 1)
				/*reset hasn't been finish*/
				priv->reset_failed_cnt++;
#endif
		}

		if (hdc_read_bits(base, HDC_PORTSC, BIT_OVER_CURRENT, 1))
			status |= USB_PORT_STAT_OVERCURRENT;

		if (hdc_read_bits(base, HDC_PORTSC, BIT_RESET_RHS, 1))
			status |= USB_PORT_STAT_RESET;

		if (hdc_read_bits(base, HDC_PORTSC, BIT_POWER_RHS, 1))
			status |= USB_PORT_STAT_POWER;

		if (hdc_read_bits(base, HDC_PORTSC, BIT_SUSPENDED_RHS, 1))
			status |= USB_PORT_STAT_C_SUSPEND << 16;

		put_unaligned_le32(status, buf);
		break;
	case SetHubFeature:
		switch (w_value) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			ret = -EPIPE;
			goto err_unlock;
		}
		break;
	case SetPortFeature:
		selector = w_index >> 8;
		w_index &= 0xff;

		if ((!w_index) || (w_index > HDC_HCD_ROOT_HUB_MAX_PORT)) {
			ret = -EPIPE;
			goto err_unlock;
		}
		w_index--;

		switch (w_value) {
		case USB_PORT_FEAT_SUSPEND:
			if (!hcd->driver->bus_resume ||
			    !hcd->driver->bus_suspend) {
				/*
				 * deny usb core request to suspend, HC can't be
				 * waked up without these two function.
				 * note : PM_RUNTIME config may have usb core to
				 * make suspend request when RH have no attached
				 * device or only hub devices are connected.
				*/
				ret = -EPIPE;
				goto err_unlock;
			} else {
				break;
			}
		case USB_PORT_FEAT_POWER:
			hdc_set_vbus(base, 1);
			break;
		case USB_PORT_FEAT_RESET:
			if (hdc_read_bits(base, HDC_PORTSC,
					  BIT_RESUMING_RHS, 1)) {
				ret = -EPIPE;
				goto err_unlock;
			}

#ifdef VIRTUAL_DEVICE_ADDRESS
			free_all_mapped_addr(hcd);
#endif

#ifdef HUB_RESET_WORKAROUND
			if (priv->reset_failed_cnt < 3) {
				/* normal reset procedure */
				hdc_write_bits(base, HDC_PORTSC,
					       BIT_ENABLE_REQ, 1, 1);
				hdc_write_bits(base, HDC_PORTSC,
					       BIT_RESET_REQ, 1, 1);
				priv->reset_start = 1;
			} else {
				/* use power on/off to recovery failed reset
				 * after it's done, hardware can do reset again
				*/
				priv->reset_start = 0;
				priv->reset_failed_cnt = 0;
				dev_info(dev, "do reset recovery\n");

				/* set detection reset */
				hdc_set_vbus(base, 0);
				hdc_set_vbus(base, 1);
			}
#else
			hdc_write_bits(base, HDC_PORTSC, BIT_ENABLE_REQ, 1, 1);
			hdc_write_bits(base, HDC_PORTSC, BIT_RESET_REQ, 1, 1);
#endif
			break;
		/* For downstream facing ports (these):  one hub port is put
		 * into test mode according to USB2 11.24.2.13, then the hub
		 * must be reset (which for root hub now means rmmod+modprobe,
		 * or else system reboot).
		 * Valid Selector Codes are showed below:
		 * 1H : Test_J
		 * 2H : Test_K
		 * 3H : Test_SE0_NAK
		 * 4H : Test_Packet
		 * 5H : Test_Force_Enable
		 * See EHCI 2.3.9 and 4.14 for information
		 * about the EHCI-specific stuff.
		 */
		case USB_PORT_FEAT_TEST:
			if ((!selector) || (selector > 5)) {
				ret = -EPIPE;
				goto err_unlock;
			}
			break;
		default:
			ret = -EPIPE;
			goto err_unlock;
		}
		break;
	default:
		ret = -EPIPE;
		goto err_unlock;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
err_unlock:
	dev_err(dev, "%s() failed when type_req 0x%x w_value 0x%x w_index 0x%x",
		__func__, type_req, w_value, w_index);
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static int hdc_host_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				    gfp_t mem_flags)
{
	/* so far we only implement dma transfer for bulk type */
	if (!usb_pipebulk(urb->pipe))
		return 0;

	return usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
}

static void hdc_host_unmap_urb_for_dma(struct usb_hcd *hcd,
				       struct urb *urb)
{
	/* so far we only implement dma transfer for bulk type */
	if (!usb_pipebulk(urb->pipe))
		return;

	usb_hcd_unmap_urb_for_dma(hcd, urb);
}

#if defined(CONFIG_PM)
static int hdc_host_bus_suspend(struct usb_hcd *hcd)
{
	unsigned long		flags;
	unsigned long		ch;
	struct f_usb20hdc_hcd	*priv = hcd_to_f_usb20hdc(hcd);
	struct device *dev = hcd->self.controller;
	void			*base = priv->reg_base;
	unsigned long		suspended = 0;

	if (time_before(jiffies, priv->next_statechange))
		usleep_range(5 * 1000, 10 * 1000);

	spin_lock_irqsave(&priv->lock, flags);

	if (HC_IS_RUNNING(hcd->state)) {
		/* stop transmition */
		dev_dbg(dev, "%s() HC_IS_RUNNING\n", __func__);
		for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++) {
			/* clear status interrupt factor */
			hdc_ep_stop_and_clear(&priv->ep[ch]);
			dequeue_all_request(&priv->ep[ch], -ESHUTDOWN);
		}

		hcd->state = HC_STATE_QUIESCING;
	}

	/* suspend request */
	if (hdc_read_bits(base, HDC_PORTSC, BIT_ENABLE_RHS, 1) &&
	    !hdc_read_bits(base, HDC_PORTSC, BIT_SUSPENDED_RHS, 1)) {
		hdc_write_bits(base, HDC_PORTSC, BIT_SUSPEND_REQ, 1, 1);
		suspended = 1;
		dev_dbg(dev, "%s() Assert suspend signal\n", __func__);
	}

	/* disable irqs INTEN */
	hdc_disable_interrupt(base, HDC_HCD_MAX_EP);

	/* disable irqs HOSTINTEN */
	set_all_interrupt_factor(base, 0, true);

	/* clear status HOSTINTEN */
	clear_all_int_status_change(base, true);

	hcd->state = HC_STATE_SUSPENDED;

	if (hcd->self.root_hub->do_remote_wakeup) {
		dev_dbg(dev, "%s() set int factor of receive wakup signal\n",
			__func__);

		/* enable wakup source */
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_SUSPEND_C_INTEN, 1, 1);
		hdc_write_bits(base, HDC_HOSTINTEN, BIT_CONNECT_C_INTEN, 1, 1);
		hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 1);
		hdc_write_bits(base, HDC_PORTSC, BIT_WAKEUP_REQ, 1, 1);
	} else {
		/* advanced experiment can be done here for powe-off device */
		dev_dbg(dev, "%s() no need for remote_wakeup\n", __func__);
	}

	/* stop F_USB20HDC controller */
	hdc_set_host_run(base, 0);

	priv->next_statechange = jiffies + msecs_to_jiffies(10);

	spin_unlock_irqrestore(&priv->lock, flags);

	/* wait 10ms for controller to enter low-power mode */
	if (suspended)
		usleep_range(10 * 1000, 20 * 1000);

	return 0;
}

static int hdc_host_bus_resume(struct usb_hcd *hcd)
{
	unsigned long		flags;
	unsigned long		ch;
	struct f_usb20hdc_hcd	*priv = hcd_to_f_usb20hdc(hcd);
	struct device *dev = hcd->self.controller;
	void	__iomem *base = priv->reg_base;

	if (time_before(jiffies, priv->next_statechange))
		usleep_range(5 * 1000, 10 * 1000);

	spin_lock_irqsave(&priv->lock, flags);

	if (HCD_HW_ACCESSIBLE(hcd) == 0) {
		dev_err(dev, "%s() resume work failed\n", __func__);

		spin_unlock_irqrestore(&priv->lock, flags);

		return -ESHUTDOWN;
	}

	/* disable irqs INTEN */
	hdc_disable_interrupt(base, HDC_HCD_MAX_EP);

	/* disable irqs HOSTINTEN */
	set_all_interrupt_factor(base, 0, true);

	/* clear status HOSTINTEN */
	clear_all_int_status_change(base,
				    !hcd->self.root_hub->do_remote_wakeup);

	if (hdc_read_bits(base, HDC_PORTSC, BIT_SUSPENDED_RHS, 1)) {
		/* disable wakeup if do_remote_wakeup is set*/
		if (hcd->self.root_hub->do_remote_wakeup)
			hdc_write_bits(base, HDC_PORTSC, BIT_WAKEUP_REQ, 1, 0);

		hdc_write_bits(base, HDC_PORTSC, BIT_RESUME_REQ, 1, 1);

		spin_unlock_irqrestore(&priv->lock, flags);

		usleep_range(20 * 1000, 30 * 1000);

		spin_lock_irqsave(&priv->lock, flags);
	}

	/* start F_USB20HDC controller */
	hdc_write_bits(base, HDC_OTGC, BIT_DM_PULL_DOWN, 1, 1);
	hdc_write_bits(base, HDC_OTGC, BIT_DP_PULL_DOWN, 1, 1);
	hdc_write_bits(base, HDC_MODE, BIT_HOST_EN, 1, 1);

	/* make the endpoint transfer link list */
	initialize_td_schedule_list(base);
	for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++)
		hdc_write_bits(base, HDC_EPCMD(ch), BIT_NLINKINVALID, 1, 0);

	priv->next_statechange = jiffies + msecs_to_jiffies(5);

	hcd->state = HC_STATE_RUNNING;

	/* now safely enable interrupt factor */
	hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 1);
	set_all_interrupt_factor(base, 1, false);

	hdc_set_host_run(base, 1);

	/*
	 * set wakup_req for hub_control().
	 * if HC is wakeup when only hub device is in connected tree,
	 * then connection change event to roothub is a MUST,
	 * which is the only event can make waked-up root hub to run
	 * configuration job, rather to fall into suspend again.
	*/
	priv->wakeup_req = 1;
	dev_dbg(dev, "%s() make a wakeup_req to hub_control()\n", __func__);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
#endif

static int debugfs_reset_connection_write(void *data, u64 val)
{
	struct f_usb20hdc_hcd *priv = data;
	void __iomem *base = priv->reg_base;

	if (!priv) {
		pr_warn("%s() f_usb20hdc struct not found\n", __func__);
		return -EINVAL;
	}

	if (val) {
		hdc_write_bits(base, HDC_PORTSC, BIT_ENABLE_REQ, 1, 0);
		while (hdc_read_bits(base, HDC_PORTSC, BIT_ENABLE_RHS, 1))
				;
#ifdef VIRTUAL_DEVICE_ADDRESS
		free_all_mapped_addr(priv->f_otg->hcd);
#endif
		hdc_write_bits(base, HDC_PORTSC, BIT_POWER_REQ, 1, 0);
		hdc_write_bits(base, HDC_PORTSC, BIT_POWER_REQ, 1, 1);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reset_connection_fops, NULL,
			debugfs_reset_connection_write, "%llu\n");

static int debugfs_line_state_read(void *data, u64 *val)
{
	struct f_usb20hdc_hcd *priv = data;
	void __iomem *base = priv->reg_base;

	if (!priv) {
		pr_warn("%s() f_usb20hdc struct not found\n", __func__);
		return -EINVAL;
	}

	dbg_print_connection(priv->dev, base);

	*val = hdc_get_linestate(base);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(line_state_fops, debugfs_line_state_read,
			NULL, "%llu\n");

int hdc_host_probe(struct f_usb20hdc *f_otg)
{
	static const struct hc_driver f_usb20hdc_hc_driver = {
		.description		= "f_usb20hdc_hcd",
		.product_desc		= "F_USB20HDC HCD",
		.hcd_priv_size		= sizeof(struct f_usb20hdc_hcd),
		.irq			= hdc_host_irq,
		.flags			= HCD_MEMORY | HCD_USB2,
		.reset			= hdc_host_reset,
		.start			= hdc_host_start,
		.stop			= hdc_host_stop,
		.shutdown		= NULL,
		.get_frame_number	= hdc_host_get_frame_number,
		.urb_enqueue		= hdc_host_urb_enqueue,
		.urb_dequeue		= hdc_host_urb_dequeue,
		.endpoint_disable	= NULL,
		.endpoint_reset		= NULL,
		.hub_status_data	= hdc_host_hub_status_data,
		.hub_control		= hdc_host_hub_control,
#if defined(CONFIG_PM)
		.bus_suspend		= hdc_host_bus_suspend,
		.bus_resume		= hdc_host_bus_resume,
#endif
		.clear_tt_buffer_complete	= NULL,
		.map_urb_for_dma	= hdc_host_map_urb_for_dma,
		.unmap_urb_for_dma = hdc_host_unmap_urb_for_dma,
	};
	u32 ch;
	struct f_usb20hdc_hcd *priv;
	struct f_usb20hdc_hcd_ep *ep;
	void __iomem *reg_base;
	struct usb_hcd *hcd = NULL;
	int result;
	struct platform_device *pdev = f_otg->pdev;
	struct device *dev = f_otg->dev;

	if (unlikely(!pdev))
		return -EINVAL;

	/* create hcd and save it in f_otg*/
	hcd = usb_create_hcd(&f_usb20hdc_hc_driver, &pdev->dev,
			     "f_usb20hdc_hcd");
	if (!hcd) {
		dev_err(dev, "%s() usb_create_hcd() failed\n", __func__);
		result = -ENOMEM;
		goto done;
	}
	dev_set_drvdata(&pdev->dev, f_otg);

	/* setup the F_USB20HDC HCD device driver structure parameter */
	priv = hcd_to_f_usb20hdc(hcd);
	priv->dev = &pdev->dev;
	priv->f_otg = f_otg;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	for (ch = HDC_DMA_CH1; ch < HDC_MAX_DMA_CH; ch++)
		priv->dma_data[ch] = f_otg->dma_data[ch];
#endif

	priv->bulk_in_buf = dma_alloc_coherent(priv->dev,
		HDC_HCD_MAX_DMA_TRANSFER_SIZE,
		&priv->bulk_in_dma, GFP_KERNEL);
	priv->bulk_out_buf = dma_alloc_coherent(priv->dev,
		HDC_HCD_MAX_DMA_TRANSFER_SIZE,
		&priv->bulk_out_dma, GFP_KERNEL);

	/* Create the debugfs */
	priv->reset_file = debugfs_create_file("reset_connection",
		S_IWUGO, f_otg->root, priv, &reset_connection_fops);
	if (!priv->reset_file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		return -ENOMEM;
	}

	/* Create the debugfs */
	priv->line_state_file = debugfs_create_file("line_state",
		S_IRUGO, f_otg->root, priv, &line_state_fops);
	if (!priv->line_state_file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		return -ENOMEM;
	}

	/* get a register base address for a F_USB20HDC device */
	reg_base = ioremap(f_otg->mem_start, f_otg->mem_size);
	if (!reg_base) {
		dev_err(dev, "%s() ioremap() failed\n", __func__);
		result = -ENODEV;
		goto err_res;
	}

	/* setup the hcd structure parameter */
	hcd->rsrc_start = f_otg->mem_start;
	hcd->rsrc_len = f_otg->mem_size;
	hcd->regs = reg_base;

	if (f_otg->variant == FHDC_VARIANT_LAP)
		writel(1, reg_base + USB_TOP_IDVBUSSEL);

	hcd->self.uses_pio_for_control = 1;

	/* initialize data except for those need to be set to zero */
	spin_lock_init(&priv->lock);
	priv->resource = f_otg->mem_res;
	priv->reg_base = reg_base;
	priv->irq = f_otg->irq;
	priv->eps_inuse_map = 0x0007;
	initialize_endpoint_configure(priv);
	for (ch = EP0; ch < HDC_HCD_MAX_EP; ch++) {
		ep = &priv->ep[ch];
		ep->priv = priv;
		INIT_LIST_HEAD(&ep->queue);
	}

	/* check endpoint buffer size */
	if (!is_endpoint_buffer_usable()) {
		dev_err(dev, "%s() ep ram buffer is insufficient\n", __func__);
		result = -ENODEV;
		goto err_irq;
	}

#if (HDC_HCD_USE_HIGH_SPEED_MODE == 1)
	priv->hs_support = 1;
#endif

	priv->ctrl_stage = CONTROL_TRANSFER_STAGE_SETUP;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	result = hdc_dma_attach(priv->dma_data,
				HDC_MAX_DMA_CH, HDC_HCD_MAX_DMA_TRANSFER_SIZE);
	if (result < 0) {
		dev_err(dev, "failed to attach DMA\n");
		goto err_dma;
	}
#endif

	if (f_otg->variant == FHDC_VARIANT_LAP) {
	        writel(0, reg_base + USB_TOP_ANPDCTL);
	        writel(3, reg_base + USB_TOP_CLKCTL);
	        mdelay(1);
	        writel(1, reg_base + USB_TOP_RSTCTL);
	        writel(1, reg_base + USB_TOP_IDVBUSSEL);
	}

	/* initialize F_USB20HDC controller */
	initialize_hcd_controller(priv);

	/* add hcd device */
	result = usb_add_hcd(hcd, priv->irq, IRQF_SHARED);
	if (result) {
		dev_err(dev, "%s() usb_add_hcd() failed\n", __func__);
		goto err_data;
	}
	f_otg->hcd = hcd;

	/* driver registering log output */
	dev_info(dev, "F_USB20HDC HCD driver (version %s) is registered\n",
		 HDC_HCD_CONFIG_DRIVER_VERSION);

	return 0;

err_data:
	platform_set_drvdata(pdev, NULL);
	f_otg->hcd = NULL;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
err_dma:
	hdc_dma_detach(priv->dma_data, HDC_MAX_DMA_CH,
		       HDC_HCD_MAX_DMA_TRANSFER_SIZE);
#endif
err_irq:
	iounmap(reg_base);
err_res:
	usb_put_hcd(hcd);
done:
	dev_err(dev, "%s() is ended with err\n", __func__);

	return result;
}

int hdc_host_remove(struct f_usb20hdc *f_otg)
{
	struct usb_hcd *hcd = f_otg->hcd;
	struct f_usb20hdc_hcd *priv = hcd_to_f_usb20hdc(hcd);
	struct platform_device *pdev = f_otg->pdev;

	if (unlikely(!pdev))
		return -EINVAL;

	debugfs_remove(priv->reset_file);
	debugfs_remove(priv->line_state_file);

	/* get hcd data */
	if (!hcd) {
		dev_err(&pdev->dev, "%s() hcd is NULL\n", __func__);
		return -EINVAL;
	}

	/* remove hcd device */
	usb_remove_hcd(hcd);

	/* disable F_USB20HDC controller interrupt */
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	hdc_dma_detach(priv->dma_data, HDC_MAX_DMA_CH,
		       HDC_HCD_MAX_DMA_TRANSFER_SIZE);
#endif
	if (priv->bulk_in_buf)
		dma_free_coherent(priv->dev, HDC_HCD_MAX_DMA_TRANSFER_SIZE,
				  priv->bulk_in_buf, priv->bulk_in_dma);
	if (priv->bulk_out_buf)
		dma_free_coherent(priv->dev, HDC_HCD_MAX_DMA_TRANSFER_SIZE,
				  priv->bulk_out_buf, priv->bulk_out_dma);

	/* release device resource */
	platform_set_drvdata(pdev, NULL);
	f_otg->hcd = NULL;
	iounmap(priv->reg_base);

	/* free F_USB20HDC HCD device driver structure data memory */
	usb_put_hcd(hcd);

	/* driver deregistering log output */
	dev_info(&pdev->dev, "F_USB20HDC HCD driver is deregistered\n");

	return 0;
}

#ifdef CONFIG_PM
int hdc_host_suspend(struct f_usb20hdc *f_otg)
{
	struct usb_hcd		*hcd = f_otg->hcd;
	struct f_usb20hdc_hcd	*priv;
	unsigned long flags = 0;
	struct device *dev = f_otg->dev;

	if (!hcd) {
		dev_err(dev, "%s() no hcd instance\n", __func__);
		return -EINVAL;
	}

	priv = hcd_to_f_usb20hdc(hcd);

	spin_lock_irqsave(&priv->lock, flags);

	/* disable the hcd interrupt factor and set hcd flag*/
	hdc_write_bits(priv->reg_base, HDC_INTEN, BIT_HOST_INTEN, 1, 0);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

int hdc_host_resume(struct f_usb20hdc *f_otg)
{
	struct usb_hcd		*hcd = f_otg->hcd;
	struct f_usb20hdc_hcd	*priv;
	unsigned long flags = 0;
	struct device *dev = f_otg->dev;

	if (!hcd) {
		dev_err(dev, "%s() no hcd instance\n", __func__);
		return -EINVAL;
	}

	priv = hcd_to_f_usb20hdc(hcd);

	/* supply power early for slow firmware be ready quickly */
	hdc_set_vbus(f_otg->reg_base, 1);

	spin_lock_irqsave(&priv->lock, flags);

	/* enable the hcd interrupt factor and set hcd flag*/
	hdc_write_bits(priv->reg_base, HDC_INTEN, BIT_HOST_INTEN, 1, 1);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef COLD_RESUME_SUPPORT
	priv->wakeup_from_poweroff = 1;
	hdc_host_reset(hcd);
	hdc_host_start(hcd);
	priv->wakeup_from_poweroff = 0;
#endif

	return 0;
}

int hdc_host_restore(struct f_usb20hdc *f_otg)
{
	usb_root_hub_lost_power(f_otg->hcd->self.root_hub);
	return 0;
}
#endif /* CONFIG_PM */
