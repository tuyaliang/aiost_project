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

#ifndef _F_USB20HDC_CORE_H
#define _F_USB20HDC_CORE_H

#include <linux/platform_data/dma-mb8ac0300-hdmac.h>
#include <linux/platform_data/f_usb20hdc.h>

/* make controller can resume from power off or hibernation */
#define COLD_RESUME_SUPPORT

#define HANDLE_PIO_URB	/* dma map exception handle when no dma flag is set */

/* platform data information structure of f_usb20hdc driver */
struct f_usb20hdc_pdata {
	u32	dma_dreq[HDC_MAX_DMA_CH]; /* HDMAC dreq number */
	u32	hdmac_ch[HDC_MAX_DMA_CH]; /* HDMAC channel number*/
};

/* F_USB20HDC device driver DMA data structure */
struct f_usb20hdc_dma_data {
	u32	hdmac_ch;	/* HDMAC transfer channel */
	u32	dreq;		/* HDMAC transfer dreq */
	s8	ep_ch;		/* endpoint channel using DMA */
	struct hdmac_req	hdmac_req;	/* HDMAC request structure */
	phys_addr_t		epbuf_daddr;	/* EP buffer pyhsical address */
};

/* F_USB20HDC HCD device driver structure */
struct f_usb20hdc {
	struct platform_device	*pdev;
	struct device		*dev;
	enum f_usb2otg_hcd_variants variant;
	struct clk		*clks[5];
	u8			clk_cnt;
	void __iomem		*reg_base;/* F_USB20HDC reg base addr */
	int			irq;	/* F_USB20HDC IRQ number */
	struct resource		*mem_res;
	resource_size_t		mem_start;
	resource_size_t		mem_size;
	u8			host_working;
	int			otg_id;
	struct delayed_work	switch_to_host;
	struct delayed_work	switch_to_gadget;

	/* lock for role switch handler */
	struct mutex		role_switch_lock;
	struct usb_hcd		*hcd;
	void			*f_usb20hdc_udc;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	struct f_usb20hdc_pdata		pdata;
	struct f_usb20hdc_dma_data	dma_data[HDC_MAX_DMA_CH];
#endif
	s8				mode;
	struct dentry			*root;
	struct dentry			*file;
	u8				handle_pio_urb_quirk;
};

#define HDC_MODE_HOST			0
#define HDC_MODE_DEVICE			1
#define HDC_MODE_DUAL_ROLE		2

#define MASK(bit_length)			((u32)-1 >> (32 - (bit_length)))
#define VALUE(reg, start_bit, bit_length)	(((reg) >> (start_bit)) &\
						  MASK(bit_length))

static inline u32 hdc_read_epctrl_bits(
	void __iomem *base_addr, u32 offset, u8 start_bit,
	u8 bit_length)
{
	return  (__raw_readl(base_addr + offset) >> start_bit) &
		MASK(bit_length);
}

static inline u32 hdc_read_epctrl(void __iomem *base_addr, u32 offset)
{
	return __raw_readl(base_addr + offset);
}

static inline void hdc_reg_cache_bits(void __iomem *base, u32 offset,
				      u32 *reg_cache)
{
	if (offset == HDC_INTS) {
		*reg_cache |=
		(1 << BIT_PHY_ERR_INT) | (1 << BIT_CMD_INT) |
		(1 << BIT_DMA1_INT) | (1 << BIT_DMA2_INT);
	} else if (offset == HDC_PORTSC) {
		*reg_cache &=
		~((1 << BIT_RESET_REQ) | (1 << BIT_SUSPEND_REQ) |
		(1 << BIT_RESUME_REQ));
	} else if (offset == HDC_PORTSTSC) {
		*reg_cache |= (1 << BIT_CONNECTION_C) |
		(1 << BIT_ENABLE_C) | (1 << BIT_SUSPEND_C) |
		(1 << BIT_OV_CURR_C) | (1 << BIT_RESET_C);
	} else if (offset == HDC_HOSTEVENTSRC) {
		*reg_cache |= (1 << BIT_SOFSTART) |
		(1 << BIT_FRAMEOV) | (0xff << BIT_TRANS_DONE(0));
	} else if (offset == HDC_DEVS) {
		*reg_cache |=
		(1 << BIT_SUSPENDE_INT) | (1 << BIT_SUSPENDB_INT) |
		(1 << BIT_SOF_INT) | (1 << BIT_SETUP_INT) |
		(1 << BIT_USBRSTE_INT) | (1 << BIT_USBRSTB_INT) |
		(1 << BIT_STATUS_OK_INT) | (1 << BIT_STATUS_NG_INT);
	} else if (offset == HDC_OTGSTSC) {
		*reg_cache |=
		(1 << BIT_TMROUT_C) | (1 << BIT_ID_C) |
		(1 << BIT_VBUS_VLD_C);
	}
}

static inline u32 hdc_read_bits(void __iomem *base, u32 offset, u8 start_bit,
				u8 bit_length)
{
	return (__raw_readl(base + offset) >> start_bit & MASK(bit_length));
}

static inline void hdc_epcmd_cache_bits_host_mode(void __iomem *base,
						  u32 offset, u32 *reg_cache)
{
	u32 reg;
	u8 ep_ch = (offset - HDC_EPCMD(0)) ?
	(offset - HDC_EPCMD(0)) / 4 :
	0;

	*reg_cache &=
		~((1 << BIT_START) | (1 << BIT_STOP) |
		(1 << BIT_INIT) | (1 << BIT_BUFWR) |
		(1 << BIT_BUFRD) | (1 << BIT_TOGGLE_SET) |
		(1 << BIT_TOGGLE_CLR) | (1 << BIT_ERRCNT_CLR) |
		(1 << BIT_STATUS_CLR));

	reg = hdc_read_epctrl(base, HDC_HCEPCTRL1(ep_ch));

	*reg_cache |=
		(1 << BIT_WRITE_EN) |
		(VALUE(reg, BIT_HCEPCTRL1_NEXTLINK, LEN_HCEPCTRL1_NEXTLINK) <<
		 BIT_NEXTLINK) |
		(VALUE(reg, BIT_HCEPCTRL1_NLINKINVALID, 1)
		 << BIT_NLINKINVALID) |
		(VALUE(reg, BIT_HCEPCTRL1_SENDPID, LEN_HCEPCTRL1_SENDPID) <<
		 BIT_SENDPID) |
		(VALUE(reg, BIT_EPCTRL_ET, LEN_EPCTRL_ET) << BIT_ET) |
		(VALUE(reg, BIT_HCEPCTRL1_SC, 1) << BIT_SC) |
		(VALUE(reg, BIT_EPCTRL_BNUM, LEN_EPCTRL_BNUM) << BIT_BNUM) |
		(VALUE(reg, BIT_HCEPCTRL1_SPEED, LEN_HCEPCTRL1_SPEED) <<
			BIT_SPEED);
}

static inline void hdc_epcmd_cache_bits_dev_mode(void __iomem *base, u32 offset,
						 u32 *register_cache)
{
	u32 reg;
	u8 ep_ch = (offset - HDC_EPCMD(0)) ?
	(offset - HDC_EPCMD(0)) / 4 :
	0;

	*register_cache &=
		~((1 << BIT_START) | (1 << BIT_STOP) |
		(1 << BIT_INIT) | (1 << BIT_BUFWR) |
		(1 << BIT_BUFRD) | (1 << BIT_STALL_SET) |
		(1 << BIT_STALL_CLR) | (1 << BIT_TOGGLE_SET) |
		(1 << BIT_TOGGLE_CLR) | (1 << BIT_RDYI_RDY_INT_CLR) |
		(1 << BIT_RDYO_EMPTY_INT_CLR) | (1 << BIT_PING_INT_CLR) |
		(1 << BIT_STALLED_INT_CLR) | (1 << BIT_NACK_INT_CLR));

	reg = hdc_read_epctrl(base,  HDC_EPCTRL(ep_ch));

	*register_cache |=
		(VALUE(reg, 16, 1) << BIT_NULLRESP) |
		(VALUE(reg, 17, 1) << BIT_NACKRESP) |
			(1 << BIT_WRITE_EN) |
		(VALUE(reg, BIT_READY_INTEN, 1) << BIT_RDYI_RDY_INTEN) |
		(VALUE(reg, BIT_EMPTY_INTEN, 1) << BIT_RDYO_EMPTY_INTEN) |
		(VALUE(reg, 20, 1) << BIT_PING_INTEN) |
		(VALUE(reg, BIT_STALL_INTEN, 1) << BIT_STALLED_INTEN) |
		(VALUE(reg, BIT_EPCTRL_NACK_INTEN, 1) << BIT_NACK_INTEN) |
		(VALUE(reg, BIT_EPCTRL_ET, LEN_EPCTRL_ET) << BIT_ET) |
		(VALUE(reg, 3, 1) << BIT_DIR) |
		(VALUE(reg, BIT_EPCTRL_BNUM, LEN_EPCTRL_BNUM) << BIT_BNUM) |
		(VALUE(reg, 14, 2) << BIT_HIBAND);
}

/*endpoint command register*/
static inline void hdc_cmd0_spin(void __iomem *base, u32 offset)
{
	u32 counter;
	u8 ep_ch = (offset - HDC_EPCMD(0)) ?
		(offset - HDC_EPCMD(0)) / 4 : 0;
	for (counter = 0xffff; (counter &&
				hdc_read_bits(base, HDC_EPCMD(ep_ch), 31, 1));
	     counter--)
		;
}

static inline u32 hdc_readl(void __iomem *base, u32 offset)
{
	return __raw_readl(base + offset);
}

static inline void hdc_writel(void __iomem *base, u32 offset, u32 value)
{
	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);

	__raw_writel(value, base + offset);

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);
}

void hdc_dma_if_dear(void __iomem *base, int dma_mode);
void hdc_set_vbus(void __iomem *base, u8 enable);
int hdc_dma_attach(struct f_usb20hdc_dma_data *dma_data, int ch_num, int size);
int hdc_dma_detach(struct f_usb20hdc_dma_data *dma_data, int ch_num, int size);
void hdc_disable_interrupt(void __iomem *base, int max_ep_num);
void hdc_core_soft_reset(struct f_usb20hdc *f_otg);

/* common macro function between host mode and gadget mode */
#define hdc_get_linestate(base)	hdc_read_bits(base, HDC_PORTSTSC,\
		BIT_LINE_STATE, LEN_LINE_STATE)

#define is_host_mode_usage(base)	hdc_read_bits(base, HDC_MODE,\
		BIT_HOST_EN, 1)

#define is_device_mode_usage(base)	hdc_read_bits(base, HDC_MODE,\
		BIT_DEV_EN, 1)

#define hdc_get_epbuf_addr_offset()	(HDC_EPBUF - F_USB20HDC_C_HSEL_ADDR)

#define hdc_get_epbuf_dma_addr(base, dma_ch)	(base + HDC_DMA_BUFFER(dma_ch))

#define hdc_write_epbuf(base, offset, data, bytes)			\
	memcpy_toio(base + F_USB20HDC_C_HSEL_ADDR + offset, data, bytes)

#define hdc_read_epbuf(base, offset, data, bytes)			\
	memcpy_fromio(data, base + F_USB20HDC_C_HSEL_ADDR + offset, bytes)

#if defined(CONFIG_USB_F_USB20HDC_HOST_ONLY) || \
	defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
int hdc_host_probe(struct f_usb20hdc *f_otg);
int hdc_host_remove(struct f_usb20hdc *f_otg);
int hdc_host_suspend(struct f_usb20hdc *f_otg);
int hdc_host_resume(struct f_usb20hdc *f_otg);
#else
static inline int hdc_host_probe(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_host_remove(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_host_suspend(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_host_resume(struct f_usb20hdc *f_otg)
{ return 0; }
#endif

#if defined(CONFIG_USB_F_USB20HDC_GADGET_ONLY) || \
	defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
int hdc_gadget_probe(struct f_usb20hdc *f_otg);
int hdc_gadget_remove(struct f_usb20hdc *f_otg);
int hdc_gadget_suspend(struct f_usb20hdc *f_otg);
int hdc_gadget_resume(struct f_usb20hdc *f_otg);
#else
static inline int hdc_gadget_probe(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_gadget_remove(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_gadget_suspend(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_gadget_resume(struct f_usb20hdc *f_otg)
{ return 0; }
#endif

#if defined(CONFIG_USB_F_USB20HDC_DUAL_ROLE)
int hdc_gadget_otg_suspend(struct f_usb20hdc *f_otg);
int hdc_gadget_otg_resume(struct f_usb20hdc *f_otg);
#else
static inline int hdc_gadget_otg_suspend(struct f_usb20hdc *f_otg)
{ return 0; }
static inline int hdc_gadget_otg_resume(struct f_usb20hdc *f_otg)
{ return 0; }
#endif

#endif
