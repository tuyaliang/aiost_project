/*
 * Copyright 2016 Socionext, Inc.
 * Masayuki Sasaki <sasaki.masayuki_s@aa.socionext.com>
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
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>

#define DRIVER_NAME "sni-spi"

/* SPI register offset */
#define SPI_TXDATA_OFS				(0x0)
#define SPI_RXDATA_OFS				(0x4)
#define SPI_DIV_OFS				(0x8)
#define SPI_CTRL_OFS				(0xC)
#define SPI_AUX_CTRL_OFS			(0x10)
#define SPI_ST_OFS				(0x14)
#define SPI_SLV_SEL_OFS				(0x18)
#define SPI_SLV_POL_OFS				(0x1C)
#define SPI_INT_EN_OFS				(0x20)
#define SPI_INT_ST_OFS				(0x24)
#define SPI_INT_CLR_OFS				(0x28)
#define SPI_TXFIFO_OFS				(0x2C)
#define SPI_RXFIFO_OFS				(0x30)
#define SPI_DMA_TO_OFS				(0x34)
#define SPI_MS_DLY_OFS				(0x38)
#define SPI_EN_OFS				(0x3C)
#define SPI_FIFO_DPTH_OFS			(0x48)
#define SPI_FIFO_WMK_OFS			(0x4C)
#define SPI_TX_DWR_OFS				(0x50)

/* Bit fields in DIV */
#define SPI_DIV_DIVISOR				(0)

/* Bit fields in CTRL */
#define SPI_CTRL_CONTXFER			(0)
#define SPI_CTRL_DIVENABLE			(1)
#define SPI_CTRL_MSB1ST				(2)
#define SPI_CTRL_CPHA				(3)
#define SPI_CTRL_CPOL				(4)
#define SPI_CTRL_MASTER				(5)
#define SPI_CTRL_DMA				(10)
#define SPI_CTRL_MWAITEN			(11)

/* Bit fields in AUX_CTRL */
#define SPI_AUX_CTRL_SPIMODE			(0)
#define SPI_AUX_CTRL_INHIBITDIN			(3)
#define SPI_AUX_CTRL_XFERFORMAT			(4)
#define SPI_AUX_CTRL_CONTXFEREXTEND		(7)
#define SPI_AUX_CTRL_BITSIZE			(8)

/* Bit fields in ST */
#define SPI_ST_XFERIP				(0)
#define SPI_ST_TXEMPTY				(2)
#define SPI_ST_TXWMARK				(3)
#define SPI_ST_TXFULL				(4)
#define SPI_ST_RXEMPTY				(5)
#define SPI_ST_RXWMARK				(6)
#define SPI_ST_RXFULL				(7)
#define SPI_ST_RXOVERFLOW			(8)
#define SPI_ST_RXTIMEOUT			(9)

/* Bit fields in SLV_SEL */
#define SPI_SLV_SEL_SSOUT0			(0)
#define SPI_SLV_SEL_SSOUT1			(1)
#define SPI_SLV_SEL_SSOUT2			(2)
#define SPI_SLV_SEL_SSOUT3			(3)

/* Bit fields in SLV_POL */
#define SPI_SLV_POL_SSPOL0			(0)
#define SPI_SLV_POL_SSPOL1			(1)
#define SPI_SLV_POL_SSPOL2			(2)
#define SPI_SLV_POL_SSPOL3			(3)

/* Bit fields in INT_EN */
#define SPI_INT_EN_TXEMPTYPULSE			(0)
#define SPI_INT_EN_TXWMARKPULSE			(1)
#define SPI_INT_EN_RXWMARKPULSE			(2)
#define SPI_INT_EN_RXFULLPULSE			(3)
#define SPI_INT_EN_XFERDONEPULSE		(4)
#define SPI_INT_EN_RXFIFOOVERFLOW		(7)
#define SPI_INT_EN_RXTIMEOUT			(8)

/* Bit fields in INT_ST */
#define SPI_INT_ST_TXEMPTYPULSE			(0)
#define SPI_INT_ST_TXWMARKPULSE			(1)
#define SPI_INT_ST_RXWMARKPULSE			(2)
#define SPI_INT_ST_RXFULLPULSE			(3)
#define SPI_INT_ST_XFERDONEPULSE		(4)
#define SPI_INT_ST_RXFIFOOVERFLOW		(7)
#define SPI_INT_ST_RXTIMEOUT			(8)

/* Bit fields in INT_CLR */
#define SPI_INT_CLR_TXEMPTYPULSE		(0)
#define SPI_INT_CLR_TXWMARKPULSE		(1)
#define SPI_INT_CLR_RXWMARKPULSE		(2)
#define SPI_INT_CLR_RXFULLPULSE			(3)
#define SPI_INT_CLR_XFERDONEPULSE		(4)
#define SPI_INT_CLR_RXFIFOOVERFLOW		(7)
#define SPI_INT_CLR_RXTIMEOUT			(8)

/* Bit fields in TXFIFO */
#define SPI_TXFIFO_TX_FIFO_LEVEL		(0)

/* Bit fields in RXFIFO */
#define SPI_RXFIFO_RX_FIFO_LEVEL		(0)

/* Bit fileds in DMA_TO */
#define SPI_DMA_TO_TIMEOUT			(0)

/* Bit fields in DMA_MS_DLY */
#define SPI_DMA_MS_DLY_MWAIT			(0)

/* Bit fields in EN */
#define SPI_EN_ENABLEREQ			(0)
#define SPI_EN_EXTENSEL				(1)

/* Bit fields in FIFO_DPTH */
#define SPI_FIFO_DPTH_FIFODEPTH			(0)

/* Bit fields in FIFO_WMK */
#define SPI_FIFO_WMK_RXWMARKSET			(0)
#define SPI_FIFO_WMK_TXWMARKSET			(8)

/* Bit fields in TX_DWR */
#define SPI_TX_DWR_TXDUMMYWR			(0)

#define RXBUSY					(1 << 0)
#define TXBUSY					(1 << 1)

#define SPI_INT_ALL_BIT_SET			(0x0000001F)

struct sni_spi_dma_data {
	struct dma_chan *ch;
	enum dma_transfer_direction direction;
	dma_addr_t addr;
};

struct sni_spi {
	struct device *dev;
	struct spi_master *master;

	struct clk *spiclk;

	void __iomem *regs;
	u32 fifo_len;
	u32 max_freq;
	int pin_cs[4];

	u16 mode;
	u8 signal;
	u8 bit_dir;
	u8 bpw;
	u8 clk_div;
	u8 fifo_wmark_tx;
	u8 fifo_wmark_rx;
	u8 n_bytes;
	u32 len;
	u32 speed;
#define SPI_CONT_TRANS_INACT_BETWEEN	0
#define SPI_CONT_TRANS_INACT_FIFO_EMPTY	1
#define SPI_CONT_TRANS_ACT_FIFO_EMPTY	2

	const void *tx;
	void *rx;

	u8 irq;
	u32 state;
	/* protect state */
	spinlock_t lock;

	// DMA parameter
	u32 use_dma;
	struct sg_table tx_sg;
	struct sg_table rx_sg;
	struct sni_spi_dma_data dma_rx;
	struct sni_spi_dma_data dma_tx;
};

static irqreturn_t sni_spi_int_handler(int irq, void *dev_id)
{
	unsigned int value;

	struct spi_master *master = dev_id;
	struct sni_spi *rs = spi_master_get_devdata(master);

	value = readw_relaxed(rs->regs + SPI_INT_ST_OFS);

	// ssOut interrupt
	if (value & (1 << SPI_INT_ST_XFERDONEPULSE)) {
		writew_relaxed(1 << SPI_INT_CLR_XFERDONEPULSE, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Rx FIFO Overflow interrupt
	if (value & (1 << SPI_INT_ST_RXFIFOOVERFLOW)) {
		writew_relaxed(1 << SPI_INT_CLR_RXFIFOOVERFLOW, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Tx FIFO Empty interrupt
	if (value & (1 << SPI_INT_ST_TXEMPTYPULSE)) {
		writew_relaxed(1 << SPI_INT_CLR_TXEMPTYPULSE, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Tx FIFO Watermark level interrupt
	if (value & (1 << SPI_INT_ST_TXWMARKPULSE)) {
		writew_relaxed(1 << SPI_INT_CLR_TXWMARKPULSE, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Rx FIFO Watermark level interrupt
	if (value & (1 << SPI_INT_ST_RXWMARKPULSE)) {
		writew_relaxed(1 << SPI_INT_CLR_RXWMARKPULSE, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Rx FIFO full interrupt
	if (value & (1 << SPI_INT_ST_RXFULLPULSE)) {
		writew_relaxed(1 << SPI_INT_CLR_RXFULLPULSE, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	// Rx timeout interrupt
	if (value & (1 << SPI_INT_ST_RXTIMEOUT)) {
		writew_relaxed(1 << SPI_INT_CLR_RXTIMEOUT, rs->regs + SPI_INT_CLR_OFS);
		wmb();
	}

	return IRQ_HANDLED;
}

static int check_overflow(struct sni_spi *rs)
{
	unsigned int value;

	// Check for buffer overflow error
	value = readw_relaxed(rs->regs + SPI_ST_OFS);

	if (value & (1 << SPI_INT_ST_RXFIFOOVERFLOW)) {
		return 1;
	}
	return 0;
}

static inline void flush_fifo(struct sni_spi *rs)
{
//	while (readl_relaxed(rs->regs + ROCKCHIP_SPI_RXFLR))
//		readl_relaxed(rs->regs + ROCKCHIP_SPI_RXDR);
}

static int wait_for_receive(struct sni_spi *rs)
{
	while ((readw_relaxed(rs->regs + SPI_ST_OFS) & (1 << SPI_ST_RXEMPTY))) {
		// FIFO is empty, wait receive
	}

	return 0;
}

static int wait_for_send(struct sni_spi *rs)
{
	while ((readw_relaxed(rs->regs + SPI_ST_OFS) & (1 << SPI_ST_TXFULL))) {
		// FIFO is full, Wait fifo empty
	}

	return 0;
}

static int sni_spi_prepare_message(struct spi_master *master,
					struct spi_message *msg)
{
	struct sni_spi *rs = spi_master_get_devdata(master);
	struct spi_device *spi = msg->spi;

	rs->mode = spi->mode;

	return 0;
}

static int sni_spi_unprepare_message(struct spi_master *master,
					  struct spi_message *msg)
{
	unsigned long flags;
	struct sni_spi *rs = spi_master_get_devdata(master);

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

static int sni_spi_pio_readx(struct sni_spi *rs, void *buf)
{
	if (rs->bpw > 16) {
		*(u32*)buf = readl_relaxed(rs->regs + SPI_RXDATA_OFS);
		return sizeof(u32);
	}
	else if (rs->bpw > 8) {
		*(u16*)buf = readw_relaxed(rs->regs + SPI_RXDATA_OFS);
		return sizeof(u16);
	}
	else {
		*(u8*)buf = readb_relaxed(rs->regs + SPI_RXDATA_OFS);
		return sizeof(u8);
	}
}

static int sni_spi_pio_writex(struct sni_spi *rs, void *buf)
{
	if (rs->bpw > 16) {
		writel_relaxed(*(u32*)buf, rs->regs + SPI_TXDATA_OFS);
		return sizeof(u32);
	}
	else if (rs->bpw > 8) {
		writew_relaxed(*(u16*)buf, rs->regs + SPI_TXDATA_OFS);
		return sizeof(u16);
	}
	else {
		writeb_relaxed(*(u8*)buf, rs->regs + SPI_TXDATA_OFS);
		return sizeof(u8);
	}
}

static int sni_spi_pio_transfer(struct sni_spi *rs)
{
	unsigned int value;
	unsigned int num;
	int remain = rs->len;
	unsigned int fifo_depth;

	uint8_t *tx_addr;
	uint8_t *rx_addr;

	if (rs->tx)
		tx_addr = (u8*)rs->tx;
	if (rs->rx)
		rx_addr = (u8*)rs->rx;

	if (rs->bpw > 16)
		// 16bit over
		remain = remain / 4;
	else if (rs->bpw > 8)
		// 8bit over
		remain = remain / 2;

	// 11) Write the Enable Register to enable the SPI port
	writeb_relaxed(1, rs->regs + SPI_EN_OFS);

	wmb();

	// Send data
	if ((rs->tx) && (rs->rx)) {
		// full duplex
		// send data
		for (num = 0; num < remain; num++) {
			tx_addr += sni_spi_pio_writex(rs, tx_addr);
			wait_for_send(rs);
		}

		// recv data
		for (num = 0; num < remain; num++) {
			wait_for_receive(rs);

			if (check_overflow(rs)) {
				dev_err(rs->dev, "buffer overflow error!\n");
				return -1;
			}
			rx_addr += sni_spi_pio_readx(rs, rx_addr);
		}
		cpu_relax();
	}
	else if ((rs->tx) && (!(rs->rx))) {
		// master send
		// Disable RX_FIFO
		value = readw_relaxed(rs->regs + SPI_AUX_CTRL_OFS);
		value += (1 << SPI_AUX_CTRL_INHIBITDIN);
		writew_relaxed(value, rs->regs + SPI_AUX_CTRL_OFS);

		wmb();

		for (num = 0; num < remain; num++) {
			tx_addr += sni_spi_pio_writex(rs, tx_addr);
			wait_for_send(rs);
		}
		cpu_relax();
	}
	else if (!(rs->tx) && (rs->rx)) {
		// master recieve
		// set interrput(TXEMPTYPULSE, TXWMARKPULSE)
		value = readw_relaxed(rs->regs + SPI_INT_EN_OFS);
		value += 1 << (SPI_INT_EN_TXEMPTYPULSE | SPI_INT_EN_TXWMARKPULSE);
		writew_relaxed(value, rs->regs + SPI_INT_EN_OFS);

		wmb();

		// fifo_depth check
		fifo_depth = readw(rs->regs + SPI_FIFO_DPTH_OFS);

		// dummy data for master receiving
		if (fifo_depth < remain) {
			// dummy count exceed to fifo_depth
			while (fifo_depth < remain) {
				writeb_relaxed(fifo_depth, rs->regs + SPI_TX_DWR_OFS);

				wmb();

				for (num = 0; num < fifo_depth; num++) {
					wait_for_receive(rs);

					if (check_overflow(rs)) {
						dev_err(rs->dev, "buffer overflow error!\n");
						return -1;
					}
					rx_addr += sni_spi_pio_readx(rs, rx_addr);
				}
				cpu_relax();
				remain -= fifo_depth;
			}

			writeb_relaxed(fifo_depth, rs->regs + SPI_TX_DWR_OFS);

			wmb();

			for (num = 0; num < fifo_depth; num++) {
				wait_for_receive(rs);

				if (check_overflow(rs)) {
					dev_err(rs->dev, "buffer overflow error!\n");
					return -1;
				}
				rx_addr += sni_spi_pio_readx(rs, rx_addr);
			}
			cpu_relax();
		}
		else{
			// dummy count settle in fifo_depth
			writeb_relaxed(remain, rs->regs + SPI_TX_DWR_OFS);

			wmb();

			for (num = 0; num < remain; num++) {
				wait_for_receive(rs);

				if (check_overflow(rs)) {
					dev_err(rs->dev, "buffer overflow error!\n");
					return -1;
				}
				rx_addr += sni_spi_pio_readx(rs, rx_addr);
			}
			cpu_relax();
		}
	}
	return 0;
}

static void sni_spi_dma_rxcb(void *data)
{
	unsigned long flags;
	struct sni_spi *rs = data;

	spin_lock_irqsave(&rs->lock, flags);

	rs->state &= ~RXBUSY;
	if (!(rs->state & TXBUSY)) {
		spi_finalize_current_transfer(rs->master);
	}

	spin_unlock_irqrestore(&rs->lock, flags);
}

static void sni_spi_dma_txcb(void *data)
{
	unsigned long flags;
	struct sni_spi *rs = data;

	/* Wait until the FIFO data completely. */
	wait_for_send(rs);

	spin_lock_irqsave(&rs->lock, flags);

	rs->state &= ~TXBUSY;
	if (!(rs->state & RXBUSY)) {
		spi_finalize_current_transfer(rs->master);
	}

	spin_unlock_irqrestore(&rs->lock, flags);
}

static void sni_spi_prepare_dma(struct sni_spi *rs)
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

		rxdesc->callback = sni_spi_dma_rxcb;
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

		txdesc->callback = sni_spi_dma_txcb;
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

static void sni_spi_end_process(struct spi_device *spi)
{
	unsigned int wait;
	unsigned int ctrl_value;
	unsigned int aux_ctrl_value;
	unsigned int value;

	struct sni_spi *rs = spi_master_get_devdata(spi->master);

	ctrl_value = readw_relaxed(rs->regs + SPI_CTRL_OFS);
	aux_ctrl_value = readw_relaxed(rs->regs + SPI_AUX_CTRL_OFS);

	// if continuous transfer enable, not operate end process
	if ((ctrl_value && 1 << SPI_CTRL_CONTXFER) &&
	    (aux_ctrl_value && SPI_AUX_CTRL_CONTXFEREXTEND)) {
		return;
	}

	// Disable interrupt
	writew_relaxed(0, rs->regs + SPI_INT_EN_OFS);

	// 1) Write Enable Register to disable the SPI port
	value = readw_relaxed(rs->regs + SPI_EN_OFS);
	value = value & 0xFFFFFFFE;
	writew_relaxed(value, rs->regs + SPI_EN_OFS);

	wmb();

	// 2) Read the Enable Register - it should be 0 before continuing to the next step
	for (wait = 0; wait < 1000; wait++) {
		value = readb_relaxed(rs->regs + SPI_EN_OFS);
		value = value & 0x00000001;
		if (value == 0) {
			return;
		}
	}
}

static int sni_spi_config(struct spi_device *spi)
{
	unsigned int div;
	unsigned int value;
	unsigned int wait;
	unsigned int ctrl_value;
	unsigned int aux_ctrl_value;
	unsigned int ssout_cont;
	unsigned int ss;

	struct sni_spi *rs = spi_master_get_devdata(spi->master);

	// 1) Write Enable Register to disable the SPI port
	value = readw_relaxed(rs->regs + SPI_EN_OFS);
	value = value & 0xFFFFFFFE;
	writew_relaxed(value, rs->regs + SPI_EN_OFS);

	wmb();

	// 2) Read the Enable Register - it should be 0 before continuing to the next step
	for (wait = 0; wait < 1000; wait++) {
		value = readb_relaxed(rs->regs + SPI_EN_OFS);
		value = value & 0x00000001;
		if (value == 0) {
			break;
		}
	}

	if (value != 0) {
		dev_err(rs->dev, "spi status abnormal!\n");
		return -1;
	}

	// 3) Clear any interrupts by writing 'h0000_01FF to Interrupt Clear Register
	writew_relaxed(SPI_INT_ALL_BIT_SET, rs->regs + SPI_INT_CLR_OFS);

	wmb();

	// 4) If using interrupts, enable interrupts by writing the Interrupt Enable Register
	writew_relaxed(1 << SPI_INT_EN_XFERDONEPULSE, rs->regs + SPI_INT_EN_OFS);

	// 5) Set the polality of the select lines by writing the Slave Select Polarity Register
	writew_relaxed(0, rs->regs + SPI_SLV_POL_OFS);

	// 6) If a master, select which slave is to be selected by writing the Slave Select Register
	of_property_read_u32(spi->dev.of_node, "ss", &ss);

	writew_relaxed(1 << ss, rs->regs + SPI_SLV_SEL_OFS);

	// 7) If a master, and if a large inter-transfer delay is desired for use with low-performance slaves, write the Master Inter - Transfer Delay Register
	writew_relaxed(0, rs->regs + SPI_MS_DLY_OFS);

	// 8) If a master, and if DMA operation is desired, write to the DMA Timeout Register to set the DMA Timeout Value
	writew_relaxed(0, rs->regs + SPI_DMA_TO_OFS);

	// 9) Write Control Register
	// a. inter-transfer delay enable(master)
	ctrl_value = 0 << SPI_CTRL_MWAITEN;

	// b. dmaEnable
	ctrl_value += (0 << SPI_CTRL_DMA);

	// c. master or slave mode
	ctrl_value += (1 << SPI_CTRL_MASTER);

	// d. cpol
	if (spi->mode & SPI_CPOL) {
		ctrl_value += (1 << SPI_CTRL_CPOL);
	}
	else {
		ctrl_value += (0 << SPI_CTRL_CPOL);
	}

	// e. cpha
	if (spi->mode & SPI_CPHA) {
		ctrl_value += (1 << SPI_CTRL_CPHA);
	}
	else {
		ctrl_value += (0 << SPI_CTRL_CPHA);
	}

	// f. msb1st
	if (spi->mode & SPI_LSB_FIRST) {
		ctrl_value += 0 << SPI_CTRL_MSB1ST;
	}
	else {
		ctrl_value += 1 << SPI_CTRL_MSB1ST;
	}

	// g.divisorEnable(master)
	rs->max_freq = clk_get_rate(rs->spiclk);

	if (rs->speed != 0) {
		div = (rs->max_freq / rs->speed / 2) - 1;
		if (!((div >= 0) && (div <= 255))) {
			dev_err(rs->dev, "div value exceed error!\n");
			return -1;
		}

		writew_relaxed(div, rs->regs + SPI_DIV_OFS);
		ctrl_value += 1 << SPI_CTRL_DIVENABLE;
	}
	else {
		writew_relaxed(0, rs->regs + SPI_DIV_OFS);
		ctrl_value += 0 << SPI_CTRL_DIVENABLE;
	}

	// 10) Write Auxiliary Control Register
	// a. continuous transfer extend(master)
	of_property_read_u32(spi->dev.of_node, "ssout-cont", &ssout_cont);

	if (ssout_cont == SPI_CONT_TRANS_ACT_FIFO_EMPTY) {
		ctrl_value += 1 << SPI_CTRL_CONTXFER;
		aux_ctrl_value = 1 << SPI_AUX_CTRL_CONTXFEREXTEND;
	}
	else if (ssout_cont == SPI_CONT_TRANS_INACT_FIFO_EMPTY) {
		ctrl_value += 1 << SPI_CTRL_CONTXFER;
		aux_ctrl_value = 0 << SPI_AUX_CTRL_CONTXFEREXTEND;
	}
	else {
		ctrl_value += 0 << SPI_CTRL_CONTXFER;
		aux_ctrl_value = 0 << SPI_AUX_CTRL_CONTXFEREXTEND;
	}

	writew_relaxed(ctrl_value, rs->regs + SPI_CTRL_OFS);

	// b. bitsize (from 4 to 32 bits)
	aux_ctrl_value += (spi->bits_per_word - 1) << SPI_AUX_CTRL_BITSIZE;

	// c. inhibit writes to RX FIFO
	aux_ctrl_value += 0 << SPI_AUX_CTRL_INHIBITDIN;

	// d. Spi mode
	// Normal SPI Only
	aux_ctrl_value += 0 << SPI_AUX_CTRL_SPIMODE;

	// Motorola SPI only
	aux_ctrl_value += 0 << SPI_AUX_CTRL_XFERFORMAT;

	writew_relaxed(aux_ctrl_value, rs->regs + SPI_AUX_CTRL_OFS);

	return 0;
}

static int sni_spi_transfer_one(
		struct spi_master *master,
		struct spi_device *spi,
		struct spi_transfer *xfer)
{
	int ret = 1;
	struct sni_spi *rs = spi_master_get_devdata(master);

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
	sni_spi_config(spi);

	if (rs->use_dma) {
		sni_spi_prepare_dma(rs);
	} else {
		ret = sni_spi_pio_transfer(rs);
	}

	// end SPI communication
	sni_spi_end_process(spi);

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

static void sni_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct sni_spi *rs = spi_master_get_devdata(spi->master);
	int n = spi->chip_select;

	gpio_set_value(rs->pin_cs[n], enable);
}

static int sni_spi_probe(struct platform_device *pdev)
{
	int ret = 0, n = 0;
	struct sni_spi *rs;
	struct spi_master *master;
	resource_size_t irq;
	int irq_err;
	u32 cs_nums;

	master = spi_alloc_master(&pdev->dev, sizeof(struct sni_spi));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	rs = spi_master_get_devdata(master);
	memset(rs, 0, sizeof(struct sni_spi));

	/* Get basic io resource and map it */
	rs->regs = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(rs->regs)) {
		ret =  PTR_ERR(rs->regs);
		goto err_ioremap_resource;
	}

	// Get cs-nums and ssout-cont
	of_property_read_u32(pdev->dev.of_node, "cs-nums", &cs_nums);

	master->num_chipselect = cs_nums;

	for (; n < of_gpio_named_count(pdev->dev.of_node,"cs-gpios"); n++) {
		rs->pin_cs[n] = of_get_named_gpio(pdev->dev.of_node,
						  "cs-gpios", n);
		if (rs->pin_cs[n] == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto err_ioremap_resource;
		}
	}

	// set irq
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "could not get IRQ\n");
		goto err_ioremap_resource;
	}

	rs->irq = irq;

	irq_err= request_irq(rs->irq, sni_spi_int_handler, 0, "sni_spi", master);
	if (irq_err) {
		dev_err(&pdev->dev, "No IRQ found\n");
		goto err_ioremap_resource;
	}

	rs->spiclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(rs->spiclk)) {
		dev_err(&pdev->dev, "Failed to get clk\n");
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

	master->auto_runtime_pm = true;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP | SPI_LSB_FIRST;
	master->dev.of_node = pdev->dev.of_node;

	master->set_cs = sni_spi_set_cs;
	master->prepare_message = sni_spi_prepare_message;
	master->unprepare_message = sni_spi_unprepare_message;
	master->transfer_one = sni_spi_transfer_one;
	master->setup = sni_spi_config;

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

static int sni_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct sni_spi *rs = spi_master_get_devdata(master);

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
static int sni_spi_suspend(struct device *dev)
{
	int ret = 0;
	struct spi_master *master = dev_get_drvdata(dev);
	struct sni_spi *rs = spi_master_get_devdata(master);

	ret = spi_master_suspend(rs->master);
	if (ret)
		return ret;

	if (!pm_runtime_suspended(dev)) {
		clk_disable_unprepare(rs->spiclk);
	}

	return ret;
}

static int sni_spi_resume(struct device *dev)
{
	int ret = 0;
	struct spi_master *master = dev_get_drvdata(dev);
	struct sni_spi *rs = spi_master_get_devdata(master);

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
static int sni_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sni_spi *rs = spi_master_get_devdata(master);

	clk_disable_unprepare(rs->spiclk);

	return 0;
}

static int sni_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sni_spi *rs = spi_master_get_devdata(master);

	return clk_prepare_enable(rs->spiclk);
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops sni_spi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sni_spi_suspend, sni_spi_resume)
	SET_RUNTIME_PM_OPS(sni_spi_runtime_suspend,
			   sni_spi_runtime_resume, NULL)
};

static const struct of_device_id sni_spi_dt_match[] = {
	{ .compatible = "socionext,sni-spi", },
	{ },
};
MODULE_DEVICE_TABLE(of, sni_spi_dt_match);

static struct platform_driver sni_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.pm = &sni_spi_pm,
		.of_match_table = of_match_ptr(sni_spi_dt_match),
	},
	.probe = sni_spi_probe,
	.remove = sni_spi_remove,
};

module_platform_driver(sni_spi_driver);

MODULE_AUTHOR("Masayuki Sasaki <sasaki.masayuki_s@aa.socionext.com>");
MODULE_DESCRIPTION("Socionext SPI Controller Driver");
MODULE_LICENSE("GPL v2");
