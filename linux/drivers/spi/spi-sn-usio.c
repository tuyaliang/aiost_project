/*
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * based on -->
 *
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 * Author: Addy Ke <addy.ke@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/dmaengine.h>

#define DRIVER_NAME "sn-usio-spi"

enum {
	SN_USIO_REG_SMR			= 0,
		SN_USIO_SMR_SOE			= BIT(0),
		SN_USIO_SMR_SCKE		= BIT(1),
		SN_USIO_SMR_BDS			= BIT(2),
		SN_USIO_SMR_SCINV		= BIT(3),
		SN_USIO_SMR_MD_SHIFT		= 5,
		SN_USIO_SMR_MD_MASK		= 7,
	SN_USIO_REG_SCR			= 1,
		SN_USIO_SCR_TXE			= BIT(0),
		SN_USIO_SCR_RXE			= BIT(1),
		SN_USIO_SCR_TBIE		= BIT(2),
		SN_USIO_SCR_TIE			= BIT(3),
		SN_USIO_SCR_RIE			= BIT(4),
		SN_USIO_SCR_SPI			= BIT(5),
		SN_USIO_SCR_UPCL		= BIT(7),
	SN_USIO_REG_ESCR		= 2,
		SN_USIO_ESCR_L_8BIT		= 0,
		SN_USIO_ESCR_L_5BIT		= 1,
		SN_USIO_ESCR_L_6BIT		= 2,
		SN_USIO_ESCR_L_7BIT		= 3,
		SN_USIO_ESCR_P			= BIT(3),
		SN_USIO_ESCR_PEN		= BIT(4),

	SN_USIO_REG_SSR			= 3,
		SN_USIO_SSR_TBI			= BIT(0),
		SN_USIO_SSR_TDRE		= BIT(1),
		SN_USIO_SSR_RDRF		= BIT(2),
		SN_USIO_SSR_ORE			= BIT(3),
		SN_USIO_SSR_FRE			= BIT(4),
		SN_USIO_SSR_PE			= BIT(5),
		SN_USIO_SSR_REC			= BIT(7),
	SN_USIO_REG_DR			= 4,
	SN_USIO_REG_BGR			= 6,
	SN_USIO_REG_FCR			= 0xc,
		SN_USIO_FCR_FE1			= BIT(0),
		SN_USIO_FCR_FE2			= BIT(1),
		SN_USIO_FCR_FCL1		= BIT(2),
		SN_USIO_FCR_FCL2		= BIT(3),
		SN_USIO_FCR_FSET		= BIT(4),
		SN_USIO_FCR_FTIE		= BIT(9),
		SN_USIO_FCR_FDRQ		= BIT(10),
		SN_USIO_FCR_FRIIE		= BIT(11),
		
	SN_USIO_REG_FBYTE		= 0xe,
};

#define RXBUSY						(1 << 0)
#define TXBUSY						(1 << 1)


struct sn_usio_spi_dma_data {
	struct dma_chan *ch;
	enum dma_transfer_direction direction;
	dma_addr_t addr;
};

struct sn_usio_spi {
	struct device *dev;
	struct spi_master *master;

	struct clk *spiclk;

	void __iomem *regs;
	u32 fifo_len;
	u32 max_freq;
	int pin_cs[4];

	u16 mode;
	u8 tmode;
	u8 bpw;
	u8 n_bytes;
	u32 len;
	u32 speed;

	const void *tx;
	void *rx;

	u32 state;
	/* protect state */
	spinlock_t lock;

	u32 use_dma;
	struct sg_table tx_sg;
	struct sg_table rx_sg;
	struct sn_usio_spi_dma_data dma_rx;
	struct sn_usio_spi_dma_data dma_tx;
};


static inline void flush_fifo(struct sn_usio_spi *rs)
{
//	while (readl_relaxed(rs->regs + ROCKCHIP_SPI_RXFLR))
//		readl_relaxed(rs->regs + ROCKCHIP_SPI_RXDR);
}

static void wait_for_idle(struct sn_usio_spi *rs)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5);

	do {
		if (readb(rs->regs + SN_USIO_REG_SSR) &
		    SN_USIO_SSR_TBI)
			return;
	} while (!time_after(jiffies, timeout));

	dev_err(rs->dev, "spi controller stuck SSR=0x%x\n", readb(rs->regs + SN_USIO_REG_SSR));
}

static int sn_usio_spi_prepare_message(struct spi_master *master,
					struct spi_message *msg)
{
	struct sn_usio_spi *rs = spi_master_get_devdata(master);
	struct spi_device *spi = msg->spi;

	rs->mode = spi->mode;

	return 0;
}

static int sn_usio_spi_unprepare_message(struct spi_master *master,
					  struct spi_message *msg)
{
	unsigned long flags;
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	spin_lock_irqsave(&rs->lock, flags);

	if (rs->use_dma) {
		if (rs->state & RXBUSY) {
			dmaengine_terminate_all(rs->dma_rx.ch);
			flush_fifo(rs);
		}

		if (rs->state & TXBUSY)
			dmaengine_terminate_all(rs->dma_tx.ch);
	}

	spin_unlock_irqrestore(&rs->lock, flags);

	return 0;
}

static int sn_usio_spi_pio_transfer(struct sn_usio_spi *rs)
{
	int remain =rs->len;

	while (remain--) {
		if (rs->tx) {
			writew(*(u8*)rs->tx, rs->regs + SN_USIO_REG_DR);
			rs->tx++;
		} else {
			writew(0x55, rs->regs + SN_USIO_REG_DR);
		}
		wait_for_idle(rs);

		if (rs->rx) {
			*(u8 *)rs->rx = readw(rs->regs + SN_USIO_REG_DR);
			rs->rx++;
		}
		cpu_relax();
	}

	if (rs->tx)
		wait_for_idle(rs);

	return 0;
}

static void sn_usio_spi_dma_rxcb(void *data)
{
	unsigned long flags;
	struct sn_usio_spi *rs = data;

	spin_lock_irqsave(&rs->lock, flags);

	rs->state &= ~RXBUSY;
	if (!(rs->state & TXBUSY)) {
		spi_finalize_current_transfer(rs->master);
	}

	spin_unlock_irqrestore(&rs->lock, flags);
}

static void sn_usio_spi_dma_txcb(void *data)
{
	unsigned long flags;
	struct sn_usio_spi *rs = data;

	/* Wait until the FIFO data completely. */
	wait_for_idle(rs);

	spin_lock_irqsave(&rs->lock, flags);

	rs->state &= ~TXBUSY;
	if (!(rs->state & RXBUSY)) {
		spi_finalize_current_transfer(rs->master);
	}

	spin_unlock_irqrestore(&rs->lock, flags);
}

static void sn_usio_spi_prepare_dma(struct sn_usio_spi *rs)
{
	unsigned long flags;
	struct dma_slave_config rxconf, txconf;
	struct dma_async_tx_descriptor *rxdesc, *txdesc;

	spin_lock_irqsave(&rs->lock, flags);
	rs->state &= ~RXBUSY;
	rs->state &= ~TXBUSY;
	spin_unlock_irqrestore(&rs->lock, flags);

	rxdesc = NULL;
	if (rs->rx) {
		rxconf.direction = rs->dma_rx.direction;
		rxconf.src_addr = rs->dma_rx.addr;
		rxconf.src_addr_width = rs->n_bytes;
		rxconf.src_maxburst = rs->n_bytes;
		dmaengine_slave_config(rs->dma_rx.ch, &rxconf);

		rxdesc = dmaengine_prep_slave_sg(
				rs->dma_rx.ch,
				rs->rx_sg.sgl, rs->rx_sg.nents,
				rs->dma_rx.direction, DMA_PREP_INTERRUPT);

		rxdesc->callback = sn_usio_spi_dma_rxcb;
		rxdesc->callback_param = rs;
	}

	txdesc = NULL;
	if (rs->tx) {
		txconf.direction = rs->dma_tx.direction;
		txconf.dst_addr = rs->dma_tx.addr;
		txconf.dst_addr_width = rs->n_bytes;
		txconf.dst_maxburst = rs->n_bytes;
		dmaengine_slave_config(rs->dma_tx.ch, &txconf);

		txdesc = dmaengine_prep_slave_sg(
				rs->dma_tx.ch,
				rs->tx_sg.sgl, rs->tx_sg.nents,
				rs->dma_tx.direction, DMA_PREP_INTERRUPT);

		txdesc->callback = sn_usio_spi_dma_txcb;
		txdesc->callback_param = rs;
	}

	/* rx must be started before tx due to spi instinct */
	if (rxdesc) {
		spin_lock_irqsave(&rs->lock, flags);
		rs->state |= RXBUSY;
		spin_unlock_irqrestore(&rs->lock, flags);
		dmaengine_submit(rxdesc);
		dma_async_issue_pending(rs->dma_rx.ch);
	}

	if (txdesc) {
		spin_lock_irqsave(&rs->lock, flags);
		rs->state |= TXBUSY;
		spin_unlock_irqrestore(&rs->lock, flags);
		dmaengine_submit(txdesc);
		dma_async_issue_pending(rs->dma_tx.ch);
	}
}

static void sn_usio_spi_config(struct sn_usio_spi *rs)
{
	int div;

//	dev_err(rs->dev, "speed == %dHz\n", rs->speed);

//	clk_set_rate(rs->spiclk, rs->speed);
	rs->max_freq = clk_get_rate(rs->spiclk);

	div = max_t(u32, rs->max_freq / rs->speed, 1);
//	dev_err(rs->dev, "div == %d\n", div);

	div = 12;

	writew_relaxed(div, rs->regs + SN_USIO_REG_BGR);
	writeb(SN_USIO_SCR_UPCL, rs->regs + SN_USIO_REG_SCR);
	writeb(0, rs->regs + SN_USIO_REG_SCR);

	writeb(0, rs->regs + SN_USIO_REG_ESCR);

	writeb((2 << SN_USIO_SMR_MD_SHIFT) | SN_USIO_SMR_SCKE |
	       SN_USIO_SMR_SOE | SN_USIO_SMR_SCINV | SN_USIO_SMR_BDS,
	       rs->regs + SN_USIO_REG_SMR);
	writew(readw(rs->regs + SN_USIO_REG_FCR) | SN_USIO_FCR_FTIE,
	       rs->regs + SN_USIO_REG_FCR);
	writeb(SN_USIO_SCR_SPI | SN_USIO_SCR_TXE | SN_USIO_SCR_RXE |
		 SN_USIO_SCR_TIE | SN_USIO_SCR_RIE,
	       rs->regs + SN_USIO_REG_SCR);
}

static int sn_usio_spi_transfer_one(
		struct spi_master *master,
		struct spi_device *spi,
		struct spi_transfer *xfer)
{
	int ret = 1;
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	if (!xfer->tx_buf && !xfer->rx_buf) {
		dev_err(rs->dev, "No buffer for transfer\n");
		return -EINVAL;
	}

	rs->speed = xfer->speed_hz;
	rs->bpw = xfer->bits_per_word;
	rs->n_bytes = rs->bpw >> 3;

	rs->tx = xfer->tx_buf;
	rs->rx = xfer->rx_buf;
	rs->len = xfer->len;

	rs->tx_sg = xfer->tx_sg;
	rs->rx_sg = xfer->rx_sg;

//	dev_err(rs->dev, "xfer len = %d tx %p rx %p tx.nents=%d rx.nents=%d\n", xfer->len, rs->tx, rs->rx, rs->tx_sg.nents, rs->rx_sg.nents);

	/* we need prepare dma before spi was enabled */
	if (master->can_dma && master->can_dma(master, spi, xfer))
		rs->use_dma = 1;
	else
		rs->use_dma = 0;
	sn_usio_spi_config(rs);

	if (rs->use_dma) {
//		pr_err("using DMA\n");
		sn_usio_spi_prepare_dma(rs);
	} else {
//		pr_err("using PIO\n");
		ret = sn_usio_spi_pio_transfer(rs);
	}

	return ret;
}
#if 0
static bool sn_usio_spi_can_dma(struct spi_master *master,
				 struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	return (xfer->len > rs->fifo_len);
}
#endif

static void sn_usio_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct sn_usio_spi *rs = spi_master_get_devdata(spi->master);
	int n = spi->chip_select;

	gpio_set_value(rs->pin_cs[n], enable);
}

static int sn_usio_spi_probe(struct platform_device *pdev)
{
	int ret = 0, n = 0;
	struct sn_usio_spi *rs;
	struct spi_master *master;
	struct resource *mem;

	master = spi_alloc_master(&pdev->dev, sizeof(struct sn_usio_spi));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	rs = spi_master_get_devdata(master);
	memset(rs, 0, sizeof(struct sn_usio_spi));

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rs->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(rs->regs)) {
		ret =  PTR_ERR(rs->regs);
		goto err_ioremap_resource;
	}
	for (; n < of_gpio_named_count(pdev->dev.of_node,"cs-gpios"); n++) {
		rs->pin_cs[n] = of_get_named_gpio(pdev->dev.of_node,
						  "cs-gpios", n);
		if (rs->pin_cs[n] == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto err_ioremap_resource;
		}
		master->num_chipselect = n;
	}
	rs->spiclk = devm_clk_get(&pdev->dev, "spiclk");
	if (IS_ERR(rs->spiclk)) {
		dev_err(&pdev->dev, "Failed to get spi_pclk\n");
		ret = PTR_ERR(rs->spiclk);
		goto err_ioremap_resource;
	}

	ret = clk_prepare_enable(rs->spiclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable spi_clk\n");
		goto err_spiclk_enable;
	}


	rs->master = master;
	rs->dev = &pdev->dev;
	rs->max_freq = clk_get_rate(rs->spiclk);

	rs->fifo_len = 128;

	spin_lock_init(&rs->lock);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	sn_usio_spi_config(rs);

	master->auto_runtime_pm = true;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->dev.of_node = pdev->dev.of_node;
	master->bits_per_word_mask = SPI_BPW_MASK(8);

	master->set_cs = sn_usio_spi_set_cs;
	master->prepare_message = sn_usio_spi_prepare_message;
	master->unprepare_message = sn_usio_spi_unprepare_message;
	master->transfer_one = sn_usio_spi_transfer_one;
#if 0
	rs->dma_tx.ch = dma_request_slave_channel(rs->dev, "tx");
	if (!rs->dma_tx.ch)
		dev_warn(rs->dev, "Failed to request TX DMA channel\n");

	rs->dma_rx.ch = dma_request_slave_channel(rs->dev, "rx");
	if (!rs->dma_rx.ch) {
		if (rs->dma_tx.ch) {
			dma_release_channel(rs->dma_tx.ch);
			rs->dma_tx.ch = NULL;
		}
		dev_warn(rs->dev, "Failed to request RX DMA channel\n");
	}

	if (rs->dma_tx.ch && rs->dma_rx.ch) {
		rs->dma_tx.addr = (dma_addr_t)(mem->start + SN_USIO_REG_DR);
		rs->dma_rx.addr = (dma_addr_t)(mem->start + SN_USIO_REG_DR);
		rs->dma_tx.direction = DMA_MEM_TO_DEV;
		rs->dma_rx.direction = DMA_DEV_TO_MEM;

		master->can_dma = sn_usio_spi_can_dma;
		master->dma_tx = rs->dma_tx.ch;
		master->dma_rx = rs->dma_rx.ch;
	}
#endif
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register master\n");
		goto err_register_master;
	}

	return 0;

err_register_master:
	if (rs->dma_tx.ch)
		dma_release_channel(rs->dma_tx.ch);
	if (rs->dma_rx.ch)
		dma_release_channel(rs->dma_rx.ch);
	clk_disable_unprepare(rs->spiclk);
err_spiclk_enable:
err_ioremap_resource:
	while (master->num_chipselect--)
		gpio_free(rs->pin_cs[master->num_chipselect]);

	spi_master_put(master);

	return ret;
}

static int sn_usio_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(rs->spiclk);

	if (rs->dma_tx.ch)
		dma_release_channel(rs->dma_tx.ch);
	if (rs->dma_rx.ch)
		dma_release_channel(rs->dma_rx.ch);

	while (master->num_chipselect--)
		gpio_free(rs->pin_cs[master->num_chipselect]);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sn_usio_spi_suspend(struct device *dev)
{
	int ret = 0;
	struct spi_master *master = dev_get_drvdata(dev);
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	ret = spi_master_suspend(rs->master);
	if (ret)
		return ret;

	if (!pm_runtime_suspended(dev)) {
		clk_disable_unprepare(rs->spiclk);
	}

	return ret;
}

static int sn_usio_spi_resume(struct device *dev)
{
	int ret = 0;
	struct spi_master *master = dev_get_drvdata(dev);
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	if (!pm_runtime_suspended(dev)) {
		ret = clk_prepare_enable(rs->spiclk);
		if (ret < 0)
			return ret;
	}

	ret = spi_master_resume(rs->master);
	if (ret < 0) {
		clk_disable_unprepare(rs->spiclk);
	}

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int sn_usio_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	clk_disable_unprepare(rs->spiclk);

	return 0;
}

static int sn_usio_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sn_usio_spi *rs = spi_master_get_devdata(master);

	return clk_prepare_enable(rs->spiclk);
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops sn_usio_spi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sn_usio_spi_suspend, sn_usio_spi_resume)
	SET_RUNTIME_PM_OPS(sn_usio_spi_runtime_suspend,
			   sn_usio_spi_runtime_resume, NULL)
};

static const struct of_device_id sn_usio_spi_dt_match[] = {
	{ .compatible = "socionext,m8m-usio-spi", },
	{ },
};
MODULE_DEVICE_TABLE(of, sn_usio_spi_dt_match);

static struct platform_driver sn_usio_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.pm = &sn_usio_spi_pm,
		.of_match_table = of_match_ptr(sn_usio_spi_dt_match),
	},
	.probe = sn_usio_spi_probe,
	.remove = sn_usio_spi_remove,
};

module_platform_driver(sn_usio_spi_driver);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("Socionext USIO SPI Controller Driver");
MODULE_LICENSE("GPL v2");
