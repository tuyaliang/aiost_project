/*
 * linux/drivers/usb/socionext_otg/f_usb20hdc_phy.c - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) SOCIONEXT ELECTRONICS INC. 2013. All rights reserved.
 * Copyright (C) 2012-2015 SOCIONEXT SEMICONDUCTOR LIMITED.
 * Author: Sneeker Yeh <Sneeker.Yeh@tw.socionext.com>
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
#include <asm/unaligned.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

#include <linux/platform_data/dma-mb8ac0300-hdmac.h>

#include "f_usb20hdc_phy.h"

static void hdc_write_bits(void __iomem *base, u32 offset, u8 start_bit,
			   u8 bit_length, u32 value)
{
	u32 reg;

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);

	reg = __raw_readl(base + offset);

	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15))) {
		if (hdc_read_bits(base, HDC_MODE, BIT_DEV_EN, 1))
			hdc_epcmd_cache_bits_dev_mode(base, offset, &reg);
		else
			hdc_epcmd_cache_bits_host_mode(base, offset, &reg);
	} else {
		hdc_reg_cache_bits(base, offset, &reg);
	}

	reg &= ~(MASK(bit_length) << start_bit);
	reg |= ((value & MASK(bit_length)) << start_bit);

	__raw_writel(reg, base + offset);
	if ((offset >= HDC_EPCMD(0)) && (offset <= HDC_EPCMD(15)))
		hdc_cmd0_spin(base, offset);
}

void hdc_dma_if_dear(void __iomem *base, int dma_mode)
{
	int i;

	for (i = HDC_DMA_CH1; i < HDC_MAX_DMA_CH; i++) {
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_ST, 1, 0);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_MODE, 1, dma_mode);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_SENDNULL, 1, 0);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_INT_EMPTY, 1, 0);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_SPR, 1, 0);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_EP, LEN_DMA_EP, 0);
		hdc_write_bits(base, HDC_DMAC(i), BIT_DMA_BLKSIZE,
			       LEN_DMA_BLKSIZE, 0);
		hdc_writel(base, HDC_DMATCI(i), 0);
	}
}

void hdc_set_vbus(void __iomem *base, u8 enable)
{
	u32 i;

	hdc_write_bits(base, HDC_PORTSC, BIT_POWER_REQ, 1, enable ? 1 : 0);
	if (enable)
		for (i = 0xffff;
		     (i && !hdc_read_bits(base, HDC_PORTSC, BIT_POWER_RHS, 1));
		     i--)
			;
	else
		for (i = 0xffff;
		     (i && hdc_read_bits(base, HDC_PORTSC, BIT_POWER_RHS, 1));
		     i--)
			;
}

void hdc_disable_interrupt(void __iomem *base, int max_ep_num)
{
	int i;

	/* disable irqs INTEN */
	hdc_write_bits(base, HDC_INTEN, BIT_HOST_INTEN, 1, 0);
	hdc_write_bits(base, HDC_INTEN, BIT_DEV_INTEN, 1, 0);
	hdc_write_bits(base, HDC_INTEN, BIT_PHY_ERR_INTEN, 1, 0);
	hdc_write_bits(base, HDC_INTEN, BIT_CMD_INTEN, 1, 0);
	for (i = HDC_DMA_CH1; i < HDC_MAX_DMA_CH; i++)
		hdc_write_bits(base, HDC_INTEN, BIT_DMA_INTEN(i), 1, 0);
	for (i = EP0; i < max_ep_num; i++)
		hdc_write_bits(base, HDC_INTEN, BIT_DEV_EP_INTEN(i), 1, 0);

	/* clear status INTEN */
	hdc_write_bits(base, HDC_INTS, BIT_PHY_ERR_INT, 1, 0);
	hdc_write_bits(base, HDC_INTS, BIT_CMD_INT, 1, 0);
	for (i = HDC_DMA_CH1; i < HDC_MAX_DMA_CH; i++)
		hdc_write_bits(base, HDC_INTS, BIT_DMA_INT(i), 1, 0);
}

static void _hdc_set_id_pullup(struct f_usb20hdc *f_otg, bool on)
{
	void __iomem *base = f_otg->reg_base;

	if (f_otg->variant == FHDC_VARIANT_LAP)
		hdc_write_bits(base, LAPREG_IDVBUSCTL,
			       LAP_IDVBUSCTL__ID_PULLUP__SHIFT,
			       1, on);
	else
		hdc_write_bits(base, HDC_OTGC, BIT_ID_PULL_UP, 1, on);
}

static void _hdc_set_vbus_provision(struct f_usb20hdc *f_otg, bool on)
{
	void __iomem *base = f_otg->reg_base;

	hdc_write_bits(base, HDC_PORTSC, BIT_POWER_CTL_REQ, 1, on);
}

static u32 _hdc_read_id_state(struct f_usb20hdc *f_otg)
{
	void __iomem *base = f_otg->reg_base;

	switch (f_otg->variant) {
        case FHDC_VARIANT_ORIG:
		return hdc_read_bits(base, HDC_OTGSTS_R_ONLY, BIT_ID, 1);
	case FHDC_VARIANT_LAP:
		return hdc_read_bits(base, LAPREG_IDVBUSDET,
				LAP_IDVBUSDET__ID_DET_STATE__SHIFT, 1);
	}
	BUG();
}

static int _hdc_soft_reset(struct f_usb20hdc *f_otg)
{
	int timeout = 1000;
	void __iomem *base = f_otg->reg_base;

	/* do soft reset */
	switch (f_otg->variant) {
	case FHDC_VARIANT_ORIG:
		hdc_write_bits(base, HDC_CONF, BIT_SOFT_RESET, 1, 1);
		while (--timeout &&
		     hdc_read_bits(base, HDC_CONF, BIT_SOFT_RESET, 1))
			udelay(10);
		break;
	case FHDC_VARIANT_LAP:
		hdc_write_bits(base, LAPREG_RSTCTL,
			       LAP_RSTCTL__SFTRST__SHIFT, 1, 0);
		udelay(10);
		hdc_write_bits(base, LAPREG_ANPDCTL,
			       LAP_ANPDCTL__ANPDCTL__SHIFT, 1, 1);
		udelay(10);
		hdc_write_bits(base, LAPREG_ANPDCTL,
			       LAP_ANPDCTL__ANPDCTL__SHIFT, 1, 0);
		udelay(10);

		hdc_write_bits(base, LAPREG_CLKCTRL,
			       LAP_CLKCTRL__PCKEN__SHIFT, 1, 1);
		hdc_write_bits(base, LAPREG_CLKCTRL,
			       LAP_CLKCTRL__HCKEN__SHIFT, 1, 1);

		hdc_write_bits(base, LAPREG_RSTCTL,
			       LAP_RSTCTL__SFTRST__SHIFT, 1, 1);
		break;
	}

	if (!timeout)
		return -ETIMEDOUT;

	return 0;
}


/*
 * when host and device mode run without otg driver, it has do reset
 * by itself. However when they run with otg driver code, reset job
 * must be handled by otg, in order to prevent otg interrupt setting
 * being clear accidentally by other code.
 */
void hdc_core_soft_reset(struct f_usb20hdc *f_otg)
{
	void __iomem *base = f_otg->reg_base;

	if (f_otg->variant == FHDC_VARIANT_LAP)
		/* let usb2 core control vbus */
		hdc_write_bits(base, LAPREG_IDVBUSSEL,
			       LAP_IDVBUSSEL__ID_VBUS_SEL__SHIFT, 1, 1);

	/* controller reset*/
	if (hdc_read_bits(base, HDC_PORTSC, 21, 1) &&
	    hdc_read_bits(base, HDC_PORTSC, 22, 1)) {
		/* release vbus control before soft reset */
		_hdc_set_vbus_provision(f_otg, false);

		/* do soft reset */
		_hdc_soft_reset(f_otg);

		hdc_set_vbus(base, 1);
		if (f_otg->variant == FHDC_VARIANT_LAP)
			hdc_write_bits(base, LAPREG_IDVBUSSEL,
				       LAP_IDVBUSSEL__ID_VBUS_SEL__SHIFT,
				       1, 1);
		_hdc_set_vbus_provision(f_otg, true);
	} else
		_hdc_soft_reset(f_otg);

	/* phy setting */
	_hdc_set_id_pullup(f_otg, true);

	/* let the pullup pull it up */
	mdelay(50);

	/* if host (ID forced low) we should provide power */
	if (!_hdc_read_id_state(f_otg)) {
		_hdc_set_vbus_provision(f_otg, true);
		hdc_set_vbus(base, 1);
	}

	if (f_otg->mode == HDC_MODE_DUAL_ROLE) {
		/* otg id interrupt setting */
		hdc_write_bits(base, HDC_OTGSTSRISE,
			       BIT_ID_REN, 1, 1);
		hdc_write_bits(base, HDC_OTGSTSFALL,
			       BIT_ID_FEN, 1, 1);
	}

	/*
	 * host mode don't need id and vbus interrupt
	 * and otg_inten should be 1 forever in other mode.
	 */
	hdc_write_bits(base, HDC_INTEN, BIT_OTG_INTEN, 1,
		       f_otg->mode != HDC_MODE_HOST);

	/*
	 * initialize F_USB20HDC system configuration register
	 * [notice]:set of soft_reset bit is prohibition
	 */
	if (f_otg->variant == FHDC_VARIANT_ORIG) {
		hdc_write_bits(base, HDC_CONF, BIT_BYTE_ORDER, 1, 0);
		hdc_write_bits(base, HDC_CONF, BIT_BURST_WAIT, 1, 1);
	}
}

/* attach external DMA controller and allocate a DMA coherent buffer */
int hdc_dma_attach(struct f_usb20hdc_dma_data *dma_data, int ch_num, int size)
{
	int count, ret;

	for (count = HDC_DMA_CH1; count < ch_num; count++) {
		/* allocate HDMAC channel for a F_USB20HDC DMAC device */
		ret = hdmac_get_channel(dma_data[count].hdmac_ch,
					HDMAC_AUTOSTART_DISABLE);
		if (ret) {
			pr_err("%s() ex-DMA ch %d attach failed at %d\n",
			       __func__, count, ret);
			return -ENOMEM;
		}
	}

	return 0;
}

int hdc_dma_detach(struct f_usb20hdc_dma_data *dma_data, int ch_num, int size)
{
	int count;

	for (count = HDC_DMA_CH1; count < ch_num; count++)
		hdmac_free(dma_data[count].hdmac_ch);

	return 0;
}

/*
 * This interrupt might happen with the connected interrupt.
 * so it better be as soon as possible, and let initialized host
 * to process connected interrupt.
 */
static void switch_to_host_event(struct work_struct *work)
{
	struct delayed_work *d_work = container_of(work,
		struct delayed_work, work);
	struct f_usb20hdc *f_otg = container_of(d_work,
		struct f_usb20hdc, switch_to_host);

	mutex_lock(&f_otg->role_switch_lock);
	if (f_otg->host_working == 1) {
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	/*suspend udc driver*/
	if (hdc_gadget_otg_suspend(f_otg)) {
		dev_err(f_otg->dev, "%s() %d.", __func__, __LINE__);
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	/*do reset for host mode initial*/
	hdc_core_soft_reset(f_otg);

	/*register Socionext hcd driver*/
	if (!hdc_host_probe(f_otg))
		f_otg->host_working = 1;
	else
		dev_err(f_otg->dev, "failed to enable host mode.");

	mutex_unlock(&f_otg->role_switch_lock);
}

/*
 * This interrupt might happen with the disconnected interrupt.
 * so it better be delayed, let host mode finish disconnect
 * procedure before unregisterring host mode driver.
 */
static void switch_to_gadget_event(struct work_struct *work)
{
	struct delayed_work *d_work = container_of(work,
		struct delayed_work, work);
	struct f_usb20hdc *f_otg = container_of(d_work,
		struct f_usb20hdc, switch_to_gadget);

	mutex_lock(&f_otg->role_switch_lock);

	if (!f_otg->host_working) {
		dev_err(f_otg->dev, "%s(): no need to switch role.", __func__);
		goto done;
	}

	hdc_host_remove(f_otg);
	f_otg->host_working = 0;

	hdc_core_soft_reset(f_otg);

	if (hdc_gadget_otg_resume(f_otg))
		dev_err(f_otg->dev, "%s() %d.", __func__, __LINE__);

done:
	mutex_unlock(&f_otg->role_switch_lock);
}

/*
 * Interrupt handler.  OTG/host/peripheral share the same int line.
 * OTG driver process only event of OTG id change event.
 */
static irqreturn_t hdc_core_interrupt(int irq, void *dev_id)
{
	u8 id = 0;
	struct f_usb20hdc *f_otg = (struct f_usb20hdc *)dev_id;
	struct device *dev = f_otg->dev;

	if (!hdc_read_bits(f_otg->reg_base, HDC_INTS, BIT_OTG_INT, 1) ||
	    !hdc_read_bits(f_otg->reg_base, HDC_OTGSTSC, BIT_ID_C, 1))
		/*don't process events are not "OTG ID change"*/
		return IRQ_NONE;

	/*only process the OTG ID change event*/
	id = hdc_read_bits(f_otg->reg_base, HDC_OTGSTS_R_ONLY, BIT_ID, 1);
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSC, BIT_ID_C, 1, 0);
	if (f_otg->mode == HDC_MODE_DUAL_ROLE) {
		switch (id) {
		case 0:
			/* host should be initialized as soon as possible*/
			cancel_delayed_work(&f_otg->switch_to_gadget);
			schedule_delayed_work(&f_otg->switch_to_host, 0);
			break;
		case 1:
			/* postpone gadget initial after host's disconnection*/
			cancel_delayed_work(&f_otg->switch_to_host);
			schedule_delayed_work(&f_otg->switch_to_gadget, 100);
			break;
		default:
			dev_err(dev, "%s() OTG ID is invalid : %d\n",
				__func__, id);
			break;
		}
	}
	/* f_otg->otg_mode is for debugging purpose */
	f_otg->otg_id = id;
	return IRQ_HANDLED;
}

static int debugfs_vbus_read(void *data, u64 *val)
{
	struct f_usb20hdc *priv = data;

	if (!priv) {
		pr_warn("%s: f_usb20hdc struct not found\n", __func__);
		return -EINVAL;
	}

	*val = hdc_read_bits(priv->reg_base,
			     HDC_OTGSTS_R_ONLY, BIT_VBUS_VLD, 1);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vbus_control, debugfs_vbus_read,
			NULL, "%llu\n");

static int hdc_core_clk_control(struct device *dev, bool on)
{
	struct f_usb20hdc *priv = dev_get_drvdata(dev);
	int ret, i = priv->clk_cnt;

	if (!on)
		goto clock_off;

	for (i = 0; i < priv->clk_cnt; i++) {
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret) {
			dev_err(dev, "failed to enable clock[%d]\n", i);
			on = ret;
			goto clock_off;
		}
	}

	return 0;

clock_off:
	for (; i > 0;)
		clk_disable_unprepare(priv->clks[--i]);

	return on;
}

static const struct of_device_id f_usb20hdc_dt_ids[] = {
	{
		.compatible = "socionext,f_usb20hdc_drc",
		.data = (void *)FHDC_VARIANT_ORIG
	}, {
		.compatible = "socionext,f_usb20hdc_drc_lap",
		.data = (void *)FHDC_VARIANT_LAP
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_usb20hdc_dt_ids);


#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
static u64 hdc_dma_mask = DMA_BIT_MASK(32);
#endif

static int hdc_core_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
		of_match_device(f_usb20hdc_dt_ids, &pdev->dev);
	int i, mode, result = 0;
	struct clk *clk;
	struct f_usb20hdc *f_otg;
	struct device *dev;
	struct resource *res;
	phys_addr_t sram;

	if (unlikely(!pdev))
		return -EINVAL;

	dev = &pdev->dev;
#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	dev->dma_mask = &dev->coherent_dma_mask;
	dev->coherent_dma_mask = hdc_dma_mask;
	dev_info(dev, "F_USB20HDC dma mask 0x%llx coherent mask 0x%llx\n",
		 (u64)*dev->dma_mask, (u64)dev->coherent_dma_mask);
#else
	/*
	 * prevent core would assump hcd want to use dma, then transfer
	 * buffer might be a sg without cpu virtual address that pio transfer
	 * need
	 */
	dev->dma_mask = NULL;
	dev->coherent_dma_mask = 0;
#endif
	dev_info(dev, "F_USB20HDC OTG driver probe start\n");

	/* allocate and save private driver data in global rather in dev*/
	f_otg = kzalloc(sizeof(*f_otg), GFP_KERNEL);
	if (!f_otg)
		return -EINVAL;

	/*driver private data initialization*/
	f_otg->dev = &pdev->dev;
	f_otg->pdev = pdev;
	f_otg->variant = (enum f_usb2otg_hcd_variants)of_id->data;
	dev_set_drvdata(&pdev->dev, f_otg);
#ifdef HANDLE_PIO_URB
	f_otg->handle_pio_urb_quirk = true;
#else
	f_otg->handle_pio_urb_quirk = false;
#endif
	mutex_init(&f_otg->role_switch_lock);
	INIT_DELAYED_WORK(&f_otg->switch_to_host, switch_to_host_event);
	INIT_DELAYED_WORK(&f_otg->switch_to_gadget, switch_to_gadget_event);

	f_otg->root = debugfs_create_dir(dev_name(&pdev->dev), NULL);
	if (!f_otg->root) {
		dev_err(&pdev->dev, "debugfs_create_dir fail\n");
		result = -ENOMEM;
		goto err_res;
	}

	f_otg->file = debugfs_create_file("vbus_control", S_IRUGO
		, f_otg->root, f_otg, &vbus_control);
	if (!f_otg->file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		result = -ENOMEM;
		goto err_debugfs1;
	}

	if (of_property_read_u32(pdev->dev.of_node, "mode", &mode))
		mode = -1;

	/* confiure mode based on choice on Kconfig or device tree blob */
	if (IS_ENABLED(CONFIG_USB_F_USB20HDC_HOST_ONLY)) {
		f_otg->mode = HDC_MODE_HOST;
		dev_dbg(dev, "%s() select Host Mode in dts\n", __func__);
	} else if (IS_ENABLED(CONFIG_USB_F_USB20HDC_GADGET_ONLY)) {
		f_otg->mode = HDC_MODE_DEVICE;
		dev_dbg(dev, "%s() select Device Mode in dts\n", __func__);
	} else if (IS_ENABLED(CONFIG_USB_F_USB20HDC_DUAL_ROLE)) {
		f_otg->mode = mode == -1 ?
			HDC_MODE_DUAL_ROLE :
			mode;
		dev_dbg(dev, "%s() select Dual Mode in dts\n", __func__);
	} else {
		f_otg->mode = -1;
		dev_err(dev, "%s() invalid mode select\n", __func__);
		goto err_debugfs2;
	}

	/* get a resource for a F_USB20HDC device */
	f_otg->mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!f_otg->mem_res) {
		dev_err(dev, "%s():platform_get_resource() failed\n",
			__func__);
		result = -ENODEV;
		goto err_res;
	}
	f_otg->mem_size = f_otg->mem_res->end - f_otg->mem_res->start + 1;
	f_otg->mem_start = f_otg->mem_res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		sram = res->start - 0x10000;
	else
		sram = f_otg->mem_start;

	/* get a register base address for a F_USB20HDC device */
	f_otg->reg_base = ioremap(f_otg->mem_start, f_otg->mem_size);
	if (!f_otg->reg_base) {
		dev_err(dev, "%s() ioremap() failed\n", __func__);
		result = -ENODEV;
		goto err_debugfs2;
	}

	/* get an IRQ for a F_USB20HDC device */
	f_otg->irq = platform_get_irq(pdev, 0);
	if (f_otg->irq < 0) {
		dev_err(dev, "%s() platform_get_irq() failed\n", __func__);
		result = -ENODEV;
		goto err_map;
	}

	/* set interrupt handler */
	result = request_irq(f_otg->irq, hdc_core_interrupt, IRQF_SHARED,
			     "f_usb20hdc", f_otg);
	if (result) {
		dev_err(dev, "%s() request_irq() failed\n",  __func__);
		goto err_map;
	}

#if defined(CONFIG_USB_F_USB20HDC_USED_DMA_TRANSFER)
	if (pdev->dev.platform_data) {
		f_otg->pdata = *(struct f_usb20hdc_pdata *)
						pdev->dev.platform_data;
	} else {
		if (!pdev->dev.of_node) {
			dev_err(dev, "%s() DMA data invalid\n", __func__);
			result = -EINVAL;
			goto err_clk;
		}

		if (of_property_read_u32_array(dev->of_node, "dma_dreq",
					       f_otg->pdata.dma_dreq, 2)) {
			dev_err(dev, "%s() dma_dreq not found\n", __func__);
			result = -EINVAL;
			goto err_clk;
		}

		if (of_property_read_u32_array(dev->of_node, "hdmac_channel",
					       f_otg->pdata.hdmac_ch, 2)) {
			dev_err(dev, "%s() hdmac_ch not found\n", __func__);
			result = -EINVAL;
			goto err_clk;
		}
	}

	/* prepare DMA data for hcd and udc */
	for (i = HDC_DMA_CH1; i < HDC_MAX_DMA_CH; i++) {
		/* initialize F_USB20HDC Host DMA device data */
		f_otg->dma_data[i].dreq = f_otg->pdata.dma_dreq[i];
		f_otg->dma_data[i].hdmac_ch = f_otg->pdata.hdmac_ch[i];
		f_otg->dma_data[i].epbuf_daddr = hdc_get_epbuf_dma_addr(
						sram, i);
	}
#endif

	/* get clock number */
	for (i = 0; i < ARRAY_SIZE(f_otg->clks); i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;
		f_otg->clks[i] = clk;
	}
	f_otg->clk_cnt = i;
	if (!i) {
		dev_err(dev, "clock not found\n");
		result = PTR_ERR(clk);
		goto err_clk;
	}
	hdc_core_clk_control(dev, true);

	/* enable power if has power domain */
	pm_runtime_enable(&pdev->dev);
	result = pm_runtime_get_sync(&pdev->dev);
	if (result < 0) {
		dev_err(dev, "get_sync failed with err %d\n", result);
		goto err_clk;
	}

	/* prevent role-switch from otg isr at the same time */
	mutex_lock(&f_otg->role_switch_lock);

	/* reset controller for hcd and udc driver */
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSC, BIT_ID_C, 1, 0);
	hdc_core_soft_reset(f_otg);

	/* initialize f_usb20hdc dual-role usb driver */
	switch (f_otg->mode) {
	case HDC_MODE_HOST:
		result = hdc_host_probe(f_otg);
		if (!result)
			f_otg->host_working = 1;
		break;
	case HDC_MODE_DEVICE:
	case HDC_MODE_DUAL_ROLE:
		result = hdc_gadget_probe(f_otg);
		break;
	default:
		result = -1;
		dev_err(dev, "invalid mode!\n");
		break;
	}
	if (result) {
		dev_err(dev, "Failed to register Socionext usb 2.0 dual-role module!\n");
		result = -ENODEV;
		goto err_power;
	}

	/* start role-switching first time */
	if (f_otg->mode == HDC_MODE_DUAL_ROLE) {
		switch (hdc_read_bits(f_otg->reg_base, HDC_OTGSTS_R_ONLY,
				      BIT_ID, 1)) {
		case 0: /* ID 0 means Host mode choosed by OTG cable */
			if (hdc_gadget_otg_suspend(f_otg) != 0) {
				result = -ENODEV;
				break;
			}
			if (!hdc_host_probe(f_otg)) {
				f_otg->host_working = 1;
				result = 0;
			} else {
				result = -ENODEV;
			}
			break;
		case 1:  /* ID 1 means Device mode choosed by OTG cable */
			result = 0;
			break;
		default:
			result = -ENODEV;
			break;
		}
	}
	if (result != 0) {
		dev_err(dev, "failed to switch role!\n");
		goto err_power;
	}

	/* now we could let otg isr to do whatever he wants  */
	mutex_unlock(&f_otg->role_switch_lock);
	dev_info(dev, "F_USB20HDC OTG driver probe end\n");
	return 0;

err_power:
	pm_runtime_put(&pdev->dev);
	mutex_unlock(&f_otg->role_switch_lock);
err_clk:
	free_irq(f_otg->irq, f_otg);
err_map:
	iounmap(f_otg->reg_base);
err_debugfs2:
	debugfs_remove(f_otg->file);
err_debugfs1:
	debugfs_remove(f_otg->root);
err_res:
	kfree(f_otg);
	dev_err(dev, "F_USB20HDC OTG driver is failed to be registered\n");
	return result;
}

#if defined(CONFIG_USB_F_USB20HDC_MODULE)
static int hdc_core_remove(struct platform_device *pdev)
{
	struct f_usb20hdc *f_otg = dev_get_drvdata(&pdev->dev);
	struct device *dev = &pdev->dev;

	if (unlikely(!pdev))
		return -EINVAL;

	disable_irq(f_otg->irq);
	free_irq(f_otg->irq, f_otg);

	/* get a device driver parameter */
	if (!f_otg) {
		dev_err(dev, "%s() %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* turn off irq first which might schedule new work*/
	cancel_delayed_work(&f_otg->switch_to_host);
	cancel_delayed_work(&f_otg->switch_to_gadget);

	/* deinit gadget and hcd driver before disabling clock*/
	if (f_otg->mode != HDC_MODE_HOST)
		hdc_gadget_remove(f_otg);
	if (f_otg->host_working)
		hdc_host_remove(f_otg);

	/*inform suspend handler don't do udc, hcd suspend*/
	f_otg->host_working = 0;

	/*disable power, clock, irq, and suspend hcd, udc */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	debugfs_remove(f_otg->file);
	debugfs_remove(f_otg->root);

	iounmap(f_otg->reg_base);
	kfree(f_otg);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int hdc_core_suspend(struct device *dev)
{
	struct f_usb20hdc *f_otg = dev_get_drvdata(dev);

	if (f_otg->host_working)
		hdc_host_suspend(f_otg);
	if (f_otg->mode != HDC_MODE_HOST)
		hdc_gadget_suspend(f_otg);

	/* disable OTG ID interrupt factor */
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSRISE, BIT_ID_REN, 1, 0);
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSFALL, BIT_ID_FEN, 1, 0);
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSC, BIT_ID_C, 1, 0);

	/* disable clock and interrupt */
	disable_irq(f_otg->irq);
	hdc_core_clk_control(dev, false);

	return 0;
}

static int hdc_core_resume(struct device *dev)
{
	struct f_usb20hdc *f_otg = dev_get_drvdata(dev);

	hdc_core_clk_control(dev, true);

#ifdef COLD_RESUME_SUPPORT
	/* recovery from power-off or hibernation */
	hdc_core_soft_reset(f_otg);
#else
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSRISE, BIT_ID_REN, 1, 1);
	hdc_write_bits(f_otg->reg_base, HDC_OTGSTSFALL, BIT_ID_FEN, 1, 1);
#endif

	if (f_otg->host_working)
		hdc_host_resume(f_otg);
	if (f_otg->mode != HDC_MODE_HOST)
		hdc_gadget_resume(f_otg);

	/*
	 * enable interrupt safely at last, after register
	 * been recovered from power-off or hibernation.
	 */
	enable_irq(f_otg->irq);

	/* runtime set active to reflect active state. */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}

static const struct dev_pm_ops hdc_core_pm_ops = {
	.suspend = hdc_core_suspend,
	.resume = hdc_core_resume,
};

#define HDC_CORE_PM_OPS (&hdc_core_pm_ops)
#else
#define HDC_CORE_PM_OPS NULL
#endif /* CONFIG_PM */

struct platform_driver f_usb20hdc_driver = {
	.probe = hdc_core_probe,
	.remove = __exit_p(hdc_core_remove),
	.driver = {
		.name = "f_usb20hdc_drc",
		.owner = THIS_MODULE,
		.pm = HDC_CORE_PM_OPS,
		.of_match_table = f_usb20hdc_dt_ids,
	},
};

module_platform_driver(f_usb20hdc_driver);

/* F_USB20HDC OTG device module definition */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Socionext Semiconductor Limited");
MODULE_DESCRIPTION("F_USB20HDC USB Dual-role controller driver");
MODULE_ALIAS("platform:f_usb20hdc_drc");

