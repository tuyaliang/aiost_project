/*
 * linux/drivers/usb/gadget/f_usb20hdc_udc.h - F_USB20HDC USB function
 * controller driver
 *
 * Copyright (C) SOCIONEXT ELECTRONICS INC. 2011-2012. All rights reserved.
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

#ifndef _F_USB20HDC_UDC_H
#define _F_USB20HDC_UDC_H

#include "f_usb20hdc_phy.h"

enum hdc_udc_ctrl_stage {
	F_USB20HDC_STAGE_SETUP = 0,	/* SETUP stage */
	F_USB20HDC_STAGE_IN_DATA,	/* IN data stage */
	F_USB20HDC_STAGE_OUT_DATA,	/* OUT data stage */
	F_USB20HDC_STAGE_IN_STATUS,	/* IN status stage */
	F_USB20HDC_STAGE_OUT_STATUS,	/* OUT status stage */
	F_USB20HDC_STAGE_MAX,		/* max value */
};

/* F_USB20HDC driver version */
#define HDC_UDC_DRIVER_VERSION			"1.0.6"

/* endpoint channel count */
#define HDC_UDC_MAX_EP				7

/* endpoint buffer RAM size */
#define HDC_UDC_EP_BUFFER_RAM_SIZE		8192	/* x1[bytes] */
#define HDC_UDC_EP_BUFFER_RAM_SIZE_LAP		7936	/* x1[bytes] */

/* hangup recovery mode, 1:recovery use , 0:recovery unuse */
#define HDC_UDC_USE_HANGUP_RECOVERY		1

/* stall error recovery mode, 1:recovery use, 0:recovery unuse */
#define HDC_UDC_USE_AUTO_STALL_RECOVERY		1

/* stall error recovery wait time, (1 to 60000)x1[ms] */
#define HDC_UDC_AUTO_STALL_RECOVERY_TIME	1

/* DMA controller transfer maximum byte */
#define HDC_UDC_DMA_TRANS_MAX_BYTES		4194304	/* x1[bytes] */

/* F_USB20HDC UDC device driver request structure */
struct f_usb20hdc_udc_req {
	struct usb_request	req;
	struct list_head	queue;
	u8			request_execute;
	u8			dma_transfer_buffer_map;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	/* HDMAC trans done irq flag */
	u8 dmac_int_occurred;

	/* USB controller's DMA transfer done irq occurred */
	u8 usb_dma_int_occurred;
#endif
};

/* F_USB20HDC UDC device driver endpoint structure */
struct f_usb20hdc_udc_ep {
	struct usb_ep		ep;	     /* endpoint structure	 */
	struct f_usb20hdc_udc	*priv; /* F_USB20HDC driver struct */
	struct list_head	queue;	     /* endpoint queue head */

	/* current request structure */
	struct f_usb20hdc_udc_req	*req;

	u8	ep_ch;			/* endpoint channel */
	u8	transfer_direction;	/* endpoint trans dir flag */
	u8	transfer_type;		/* endpoint transfer type */
	u16	buf_offset[EP_BUF_CNT];	/* endpoint buffer addr offset*/
	u16	buffer_size;		/* endpoint buffer size */
	u8	buffers;		/* endpoint buffer count */
	u8	pio_auto_change;	/* PIO trans auto change flag */

	/* IN transfer end notify timing to USB host flag */
	u8	in_trans_end_timing;

	s8	dma_ch;			/* USB controller DMA channel */
	u8	halt;			/* transfer halt flag */
	u8	force_halt;		/* transfer force halt flag */
	u8	null_packet;		/* NULL packet transfer fl	ag */
	u8	dma_transfer;		/* DMA transfer flag */
};

/* F_USB20HDC UDC device driver structure */
struct f_usb20hdc_udc {
	struct device		*dev;
	struct usb_gadget	gadget;		/* gadget structure */
	spinlock_t		lock;

	/* gadget driver structure */
	struct usb_gadget_driver	*gadget_driver;

#if (HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	/* timer structure for halt transfer error recovery */
	struct timer_list halt_transfer_error_recovery_timer;
#endif
	struct resource		*resource;
	void __iomem		*reg_base;
	int			irq;

	/* udc driver endpoint structure array */
	struct f_usb20hdc_udc_ep ep[HDC_UDC_MAX_EP];

	u8 device_add;			/* device driver register flag*/
	u8 bus_connect;			/* bus connect status flag */
	u8 selfpowered;			/* self-powered flag */
	u8 configure_value_last;	/* last configure value */

	enum usb_device_state	device_state;	   /* USB device state */
	enum usb_device_state	device_state_last; /* last USB device state */

	enum hdc_udc_ctrl_stage	ctrl_stage;	/* control transfer stage */

	/*control transfer priority-processing direction flag */
	u8 ctrl_pri_dir;
	/* control transfer status stage delay flag */
	u8 ctrl_status_delay;

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_dma_data	dma_data[HDC_MAX_DMA_CH];
#endif
	/*this is for otg resume and suspend function*/
#if (defined(CONFIG_USB_F_USB20HDC) ||\
	defined(CONFIG_USB_F_USB20HDC_MODULE))
	u8 gadget_connected;
	u8 otg_suspend_state;
#endif
	struct f_usb20hdc	*f_otg;
	u32			test_selector;
};

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static void on_end_dma_transfer(u32 channel, void *data, int state);
#endif

static void abort_in_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init);
static void abort_out_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init);

#endif
