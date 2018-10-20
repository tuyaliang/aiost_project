/*
 * linux/drivers/usb/host/f_usb20hdc-hcd.h - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) SOCIONEXT ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2012-2015 SOCIONEXT SEMICONDUCTOR LIMITED.
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

#ifndef _F_USB20HDC_HCD_H
#define _F_USB20HDC_HCD_H

/* workaroud for some hub can't be initialized */
#define HUB_RESET_WORKAROUND

/*
 * using mapped device address is due to the only 4 bits device
 * addressing capability in F_USB20HDC IP.
*/
#define VIRTUAL_DEVICE_ADDRESS

#include "f_usb20hdc_phy.h"

/* F_USB20HDC HCD driver transfer type enumeration */
#define TRANSFER_TYPE_CONTROL				0
#define TRANSFER_TYPE_BULK_IN				1
#define TRANSFER_TYPE_BULK_OUT				2
#define TRANSFER_TYPE_INTERRUPT_IN			3
#define TRANSFER_TYPE_INTERRUPT_OUT		4
#define TRANSFER_TYPE_ISOCHRONOUS_IN		5
#define TRANSFER_TYPE_ISOCHRONOUS_OUT		6

/* F_USB20HDC HCD driver control transfer stage enumeration */
enum f_usb20hdc_ctrl_stage {
	CONTROL_TRANSFER_STAGE_SETUP = 0,	/* SETUP stage */
	CONTROL_TRANSFER_STAGE_IN_DATA,		/* IN data stage */
	CONTROL_TRANSFER_STAGE_OUT_DATA,	/* OUT data stage */
	CONTROL_TRANSFER_STAGE_IN_STATUS,	/* IN status stage */
	CONTROL_TRANSFER_STAGE_OUT_STATUS,	/* OUT status stage */
};

/* F_USB20HDC driver version */
#define HDC_HCD_CONFIG_DRIVER_VERSION		"0.9.4"

/* F_USB20HDC HCFRMINIT register frminit bit value */
#define HDC_HCD_CONFIG_FRMINIT_BIT_VALUE	7499

/* endpoint channel count */
#define HDC_HCD_MAX_EP				8

/* maximium buffer size per endpoint x1[bytes] */
#define HDC_HCD_MAX_BUF_SIZE_PER_EP		512

/* endpoint buffer RAM size x1[bytes] */
#define HDC_HCD_EP_BUFFER_RAM_SIZE		32768

/* F_USB20HDC maximium DMA transfer size x1[bytes] */
#define HDC_HCD_MAX_DMA_TRANSFER_SIZE		4194304

/* F_USB20HDC speed mode, 1:high speed mode, 0:full speed mode */
#define HDC_HCD_USE_HIGH_SPEED_MODE		1

/* F_USB20HDC maximium ports of root hub */
#define HDC_HCD_ROOT_HUB_MAX_PORT		1

#define USB_TOP_BASE            (0x00010000)
#define USB_TOP_CLKCTL          (USB_TOP_BASE + 0x0000)
#define USB_TOP_RSTCTL          (USB_TOP_BASE + 0x0004)
#define USB_TOP_ANPDCTL         (USB_TOP_BASE + 0x0008)
#define USB_TOP_IDVBUSSEL       (USB_TOP_BASE + 0x1000)
#define USB_TOP_IDVBUSCTL       (USB_TOP_BASE + 0x1004)
#define USB_TOP_IDVBUSDET       (USB_TOP_BASE + 0x1008)
#define USB_TOP_HDMAC1          (USB_TOP_BASE + 0x2000)
#define USB_TOP_HDMAC2          (USB_TOP_BASE + 0x2004)
#define USB_TOP_DMAFSM1         (USB_TOP_BASE + 0x2008)
#define USB_TOP_DMAFSM2         (USB_TOP_BASE + 0x200C)

/* F_USB20HDC HCD device driver request structure */
struct f_usb20hdc_hcd_req {
	struct urb		*urb;	/* USB request block */
	struct list_head	queue;	/* request queue head */
	u8	request_execute;	/* request execute flag */
	u8	ep_ch;			/* ep that handling the request */
	u16	maxpacket;		/* max pkt size for ep req*/

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	u8	dmac_int_occurred;
	u8	usb_dma_int_occurred;
#endif
};

/* F_USB20HDC HCD split token structure */
struct f_usb20hdc_hcd_split_token {
	u8 hub_addr;	/* hub address */
	u8 port_addr;	/* port address */
	u8 startbit;		/* Start bit */
};

/* F_USB20HDC HCD device driver endpoint structure */
struct f_usb20hdc_hcd_ep {
	/* F_USB20HDC HCD driver structure */
	struct f_usb20hdc_hcd	*priv;
	u8	ep_ch;		/* ep channel */
	u8	transfer_type;/* ep trans type*/
	u16	max_packet_size;	/* max packet size*/
	u32	iso_packets;	/* trans pkt count */
	int	transfer_status;	/* transfer status */
	u16	buf_offset[EP_BUF_CNT]; /* ep buf address offset */
	u16	buffer_size;	/* endpoint buf size */
	u8	buffers;	/* endpoint buf count */

	struct f_usb20hdc_hcd_req	*req;	/* current request */
	struct list_head		queue;	/* endpoint list head */

	/*device number for dynamic endpoints in use */
	u8 inuse_devnum;
	/* endpoint number for dynamic endpoints in use */
	u8 inuse_epnum;
	struct f_usb20hdc_hcd_split_token split;	/* split token */
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	s8 dma_ch;		/* controller's DMA channel used by ep */
#endif
};

#define MAX_DEV_ADDR 15 /* 1~15 is available */
struct dev_addr_mapping {
	u8 logical_addr;
	u8 hub_addr;
	u8 port_num;
	u8 used;
};

/* F_USB20HDC HCD device driver structure */
struct f_usb20hdc_hcd {
	struct device			*dev;
	spinlock_t			lock;		/* mutex */
	struct resource			*resource;	/* dev resource */
	void __iomem			*reg_base;	/* reg base addr */
	int				irq;		/* IRQ number */

	/* HCD device driver endpoint structure array */
	struct f_usb20hdc_hcd_ep ep[HDC_HCD_MAX_EP];

	u16	eps_inuse_map;	/* map for endpoints in use */
	u8	hs_support;		/* high-speed support flag */
	u8	bulkin_remain;	/* endpoint remain TX data */

	/* control transfer stage */
	enum f_usb20hdc_ctrl_stage	ctrl_stage;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_dma_data	dma_data[HDC_MAX_DMA_CH];
#endif
	unsigned long next_statechange;
	u8 wakeup_req;

#ifdef HUB_RESET_WORKAROUND
	u8 reset_start;
	u8 reset_failed_cnt;
#endif
#ifdef VIRTUAL_DEVICE_ADDRESS
	struct dev_addr_mapping dev_addr_table[MAX_DEV_ADDR];
#endif
	struct f_usb20hdc	*f_otg;
	u8		wakeup_from_poweroff; /*0 :wake up, 1: suspended*/
	struct dentry	*reset_file;
	struct dentry	*line_state_file;

	void		*bulk_in_buf;
	dma_addr_t	bulk_in_dma;
	void		*bulk_out_buf;
	dma_addr_t	bulk_out_dma;
};

#define TOGGLE_SET	1
#define TOGGLE_CLR	0

#define hdc_set_host_run(base, run) hdc_write_bits(base,\
				HDC_HCCTRL, 0, 1, run)
#define hdc_set_startlink(base, ep_ch) hdc_write_bits(base,\
		HDC_HCSTLINK, 0, 3, ep_ch)
#define hdc_ep_set_nextlink(base, ep_ch, hcep) hdc_write_bits(base,\
		HDC_EPCMD(ep_ch), BIT_NEXTLINK, LEN_NEXTLINK, hcep)
#define hdc_frame_set_count(base, count)	hdc_write_bits((base),\
					HDC_HCFRMINIT, 0, 13, (count))
#define hdc_frame_set_index(base, frame, uframe)		\
do {							\
	hdc_write_bits((base), HDC_HCFRMIDX, 0, 3, (uframe));	\
	hdc_write_bits((base), HDC_HCFRMIDX, 3, 11, (frame));	\
} while (0)

#define hdc_sof_value(base)		hdc_read_bits((base),\
				HDC_HCFRMIDX, 16, 11)
/* endpoint cmd registers will have different function in different mode */
#define hdc_ep_set_stop(base, ep_ch)			\
do {							\
	hdc_write_bits(base, HDC_EPCMD(ep_ch), BIT_STOP, 1, 1);\
	hdc_read_epctrl_bits(base, HDC_HCEPCTRL1(ep_ch), 26, 2) == 2\
		? udelay(250) : mdelay(2);\
} while (0)

#define hdc_ep_status_stall(ep)	hdc_read_epctrl_bits((ep)->priv->reg_base,\
			HDC_HCEPCTRL1((ep)->ep_ch), BIT_STATUS_STALL, 1)
#define hdc_ep_status_halt(ep)	hdc_read_epctrl_bits((ep)->priv->reg_base,\
			HDC_HCEPCTRL1((ep)->ep_ch), BIT_STATUS_HALT, 1)
#define hdc_ep_toggle_bit(ep)	 hdc_read_epctrl_bits((ep)->priv->reg_base,\
			HDC_HCEPCTRL1((ep)->ep_ch), BIT_TOGGLE, 1)
#define hdc_ep_empty_bit(ep)	hdc_read_epctrl_bits((ep)->priv->reg_base,\
			HDC_HCEPCTRL1((ep)->ep_ch), BIT_EMPTY, 1)
#define hdc_ep_status_trans(ep)	hdc_read_epctrl_bits((ep)->priv->reg_base,\
			HDC_HCEPCTRL1((ep)->ep_ch), BIT_TRANS_EN, 1)
#define hdc_ep_complete_inten_set(ep, on)	hdc_write_bits(\
		(ep)->priv->reg_base, HDC_HOSTINTEN,\
		BIT_TRANS_DONE_INTEN((ep)->ep_ch), 1, on)

#define hdc_status_stall(epctrl1)	VALUE(epctrl1, BIT_STATUS_STALL, 1)
#define hdc_status_halt(epctrl1)	VALUE(epctrl1, BIT_STATUS_HALT, 1)
#define hdc_toggle_bit(epctrl1)	VALUE(epctrl1, BIT_TOGGLE, 1)
#define hdc_empty_bit(epctrl1)	VALUE(epctrl1, BIT_EMPTY, 1)
#define hdc_status_trans(epctrl1)	VALUE(epctrl1, BIT_TRANS_EN, 1)

#define hdc_hdmac_stop(ep)						\
do {								\
	spin_unlock(&ep->priv->lock);				\
	hdmac_stop_nowait(ep->priv->dma_data[ep->dma_ch].hdmac_ch);	\
	stop_usb_top_hdmac(ep); \
	spin_lock(&ep->priv->lock);					\
} while (0)

#define hcd_to_f_usb20hdc(hcd)	((struct f_usb20hdc_hcd *)((hcd)->hcd_priv))

#define f_usb20hdc_to_hcd(f_usb20hdc)	(container_of((void *)f_usb20hdc,\
				struct usb_hcd, hcd_priv))

#ifdef DEBUG
static void dbg_print_connection(struct device *pdev, void __iomem *base);
#else
#define dbg_print_connection(pdev, base) do { } while (0)
#endif

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
void hdc_dma_if_stop(struct f_usb20hdc_hcd_ep *ep);
static void end_dma_transfer(u32 channel, void *data, int state);
#endif

#endif
