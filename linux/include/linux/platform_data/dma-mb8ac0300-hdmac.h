/*
 *  linux/arch/arm/mach-mb8ac0300/include/mach/hdmac.h
 *
 * Copyright (C) 2011 SOCIONEXT SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __PLAT_HDMAC_H
#define __PLAT_HDMAC_H

#include <linux/device.h>

#define CHIP_INDEX(chan)	((chan) / HDMAC_MAX_CHIP_CHANNELS)
#define CHAN_INDEX(chan)	((chan) % HDMAC_MAX_CHIP_CHANNELS)
#define OUTER_CHAN(chip, chan)	(((chip) * HDMAC_MAX_CHIP_CHANNELS) + (chan))

#define HDMAC_CM3_CHIP		0
#define HDMAC_MSIO_CHIP		1

#define HDMAC_MAX_CHIPS		4
#define HDMAC_MAX_CHIP_CHANNELS	8
#define HDMAC_MAX_CHANNELS	(HDMAC_MAX_CHIPS * HDMAC_MAX_CHIP_CHANNELS)

#define HDMAC_AUTOSTART_ENABLE	1
#define HDMAC_AUTOSTART_DISABLE	0

/* HDMAC registers */
#define HDMAC_REG_DMACR		0x00	/* DMACR register offset */
#define HDMAC_REG_DMACA		0x00	/* DMACA registers offset */
#define HDMAC_REG_DMACB		0x04	/* DMACB registers offset */
#define HDMAC_REG_DMACSA	0x08	/* DMACSA registers offset */
#define HDMAC_REG_DMACDA	0x0C	/* DMACDA registers offset */

#define DMACR		HDMAC_REG_DMACR
#define DMACA(ch)	(HDMAC_REG_DMACA + (((ch) + 1) * 0x10))
#define DMACB(ch)	(HDMAC_REG_DMACB + (((ch) + 1) * 0x10))
#define DMACSA(ch)	(HDMAC_REG_DMACSA + (((ch) + 1) * 0x10))
#define DMACDA(ch)	(HDMAC_REG_DMACDA + (((ch) + 1) * 0x10))

/* HDMAC state */
#define HDMAC_PREPARE			0
#define HDMAC_IDLE			1
#define HDMAC_RUNNING			2
#define HDMAC_STOP_REQUEST		3
#define HDMAC_STOP_REQUEST_NOWAIT	4

/* HDMAC overall configuration register */
#define HDMACR_DE	(0x01 << 31)	/* HDMA all channels are enable */
#define HDMACR_DS	(0x01 << 30)	/*
					 * The DMA transfers of all chnnels
					 * are halted by disable/halt setting
					 */
#define HDMACR_PR	(0x01 << 28)	/* Priority is rotated */
#define HDMACR_HALT	(0x01 << 24)	/* All channels are halted */

/* DMAC Configuration A register */
#define HDMACA_EB	(0x01 << 31)	/* This channel is enabled */
#define HDMACA_PB	(0x01 << 30)	/* This channel is halted */
#define HDMACA_ST_MASK	(0x01 << 29)
#define HDMACA_ST	(0x01 << 29)	/* Software request */

/* Input Select */
#define HDMACA_IS_MASK	(0x1F << 24)
#define HDMACA_IS_SW	(0x00 << 24)	/* Software */

#define HDMACA_IS_DERQH	(0x0E << 24)	/* DREQ High level or Positive edge */
#define HDMACA_IS_DERQL	(0x0F << 24)	/* DREQ Low level or Negative edge */
					/* IDREQ High level or Positive edge */
#define HDMACA_IS_IDREQ0H	(0x10 << 24)
#define HDMACA_IS_IDREQ1H	(0x11 << 24)
#define HDMACA_IS_IDREQ2H	(0x12 << 24)
#define HDMACA_IS_IDREQ3H	(0x13 << 24)
#define HDMACA_IS_IDREQ4H	(0x14 << 24)
#define HDMACA_IS_IDREQ5H	(0x15 << 24)
#define HDMACA_IS_IDREQ6H	(0x16 << 24)
#define HDMACA_IS_IDREQ7H	(0x17 << 24)
#define HDMACA_IS_IDREQ8H	(0x18 << 24)
#define HDMACA_IS_IDREQ9H	(0x19 << 24)
#define HDMACA_IS_IDREQ10H	(0x1A << 24)
#define HDMACA_IS_IDREQ11H	(0x1B << 24)
#define HDMACA_IS_IDREQ12H	(0x1C << 24)
#define HDMACA_IS_IDREQ13H	(0x1D << 24)
#define HDMACA_IS_IDREQ14H	(0x1E << 24)
#define HDMACA_IS_IDREQ15H	(0x1F << 24)
/* Beat Type*/
#define HDMACA_BT_MASK		(0x0F << 20)
#define HDMACA_BT_NORMAL	(0x00 << 20)
#define HDMACA_BT_SINGLE	(0x08 << 20)	/* same as NORMAL*/
#define HDMACA_BT_INCR		(0x09 << 20)
#define HDMACA_BT_WRAP4		(0x0A << 20)
#define HDMACA_BT_INCR4		(0x0B << 20)
#define HDMACA_BT_WRAP8		(0x0C << 20)
#define HDMACA_BT_INCR8		(0x0D << 20)
#define HDMACA_BT_WRAP16	(0x0E << 20)
#define HDMACA_BT_INCR16	(0x0F << 20)

/* Block Count*/
#define HDMACA_BC_MASK		(0x0F << 16)

/* Transfer Count*/
#define HDMACA_TC_MASK		(0xFF << 0)

/* DMAC Configuration B register */
#define HDMACB_TT_2CYCLE	(0x00 << 30)	/* 2cycle transfer */

/* Mode Select */
#define HDMACB_MS_MASK		(0x03 << 28)
#define HDMACB_MS_BLOCK		(0x00 << 28)	/* Block transfer mode */
#define HDMACB_MS_BURST		(0x01 << 28)	/* Burst transfer mode */
#define HDMACB_MS_DEMAND	(0x02 << 28)	/* Demand transfer mode */

/* Transfer Width */
#define HDMACB_TW_BYTE		(0x00 << 26)	/* Byte */
#define HDMACB_TW_HALFWORD	(0x01 << 26)	/* Half-word */
#define HDMACB_TW_WORD		(0x02 << 26)	/* Word */

/* Fixed Source */
#define HDMACB_FS	(0x01 << 25)	/* Source address is fixed */
/* Fixed Destination */
#define HDMACB_FD	(0x01 << 24)	/* Destination address is fixed */
/* Reload Count */
#define HDMACB_RC	(0x01 << 23)	/* Transfer count is enabled */
/* Reload Source */
#define HDMACB_RS	(0x01 << 22)	/* Source address is enabled */
/* Reload Destination */
#define HDMACB_RD	(0x01 << 21)	/* Destination address is enabled */
/* Error Interrupt */
#define HDMACB_EI	(0x01 << 20)	/* Error irq issuance is enabled */
/* Completion Interrupt */
#define HDMACB_CI	(0x01 << 19)	/* Completion irq is enabled */

/* Stop Status */
#define HDMACB_SS_MASK				(0x07 << 16)
#define HDMACB_SS_NONE				(0x00 << 16)
#define HDMACB_SS_ADD_OVERFLOW			(0x01 << 16)
#define HDMACB_SS_TRANSFER_STOP_REQUEST		(0x02 << 16)
#define HDMACB_SS_SOURCE_ACCESS_ERROR		(0x03 << 16)
#define HDMACB_SS_DESTINATION_ACCESS_ERROR	(0x04 << 16)
#define HDMACB_SS_NORMAL_END			(0x05 << 16)
#define HDMACB_SS_DMA_PAUSE			(0x07 << 16)
#define HDMAC_REQ_DATA_FLUSHED			0xFF000000

#ifndef __PLAT_XDMAC_H
struct dma_req_data {
	u32 size;		/* request size in bytes */
	dma_addr_t src;		/* source of DMA data */
	dma_addr_t dst;		/* destination of DMA data */

	/* driver handles */
	void *irq_data;			/* data for the callback */
	void (*callback_fn)(u32 channel, void *irq_data, int state);
					/*
					 * DMA request done callback function
					 * channel : channel number
					 * irq_data: callback data
					 * state : DMA stopped state
					 */
};
#endif

struct hdmac_chip;

struct hdmac_chan {
	struct mb8ac0300_hdmac_chip *chip; /* owning chip */
	/* channel state flags and information */
	u32 number;		/* number of this hdmac channel */
	u8 state;		/* channel state */

	char name[10];

	/* channel's hardware position and configuration */
	u32 irq;	/* channel irq */
	u32 dmaca;	/* DMACA register value */
	u32 dmacb;	/* DMACB register value */

	/* channel configuration */
	u8 autostart_flg;	/* DMA auto start when DMA request loaded */

	/* hdmac request list and information */
	struct list_head list;

	spinlock_t lock;	/* protect req info */
	struct completion stop_completion;
};

struct mb8ac0300_hdmac_chip {
	int chip_index;
	int channels;
	void *base;
	struct hdmac_chan chans[HDMAC_MAX_CHIP_CHANNELS];
#ifdef CONFIG_PM
	u32 hdmac_dmacr;
#endif
	struct device *dev;
};

struct hdmac_req {
	struct dma_req_data req_data;	/* dma request data */
	struct list_head node;
	u32 dmaca;	/* DMACA */
	u32 dmacb;	/* DMACB */
};

/*
 * hdmac_get_channel
 *
 * request a dma channel exclusivley
 */
extern int hdmac_get_channel(u32 channel, u8 autostart_flg);

/*
 * hdmac_enqueue
 *
 * place the given DMA request onto the queue of operations for the channel.
 */
extern int hdmac_enqueue(u32 channel, struct hdmac_req *hdmac_req);

/*
 * hdmac_start
 *
 * start a dma channel going
 */
extern int hdmac_start(u32 channel);

/*
 * hdmac_getposition
 *
 * returns the current transfer points for the dma source and destination
 */
extern int hdmac_getposition(u32 channel, dma_addr_t *src, dma_addr_t *dst);

/*
 * hdmac_stop(synchronous)
 *
 * stop a dma channel
 */
extern int hdmac_stop(u32 channel);

/*
 * hdmac_stop(asynchronous)
 *
 * stop a dma channel
 */
extern int hdmac_stop_nowait(u32 channel);

/*
 * hdmac_flush
 *
 * remove all current and pending transfers
 */
extern int hdmac_flush(u32 channel);

/*
 * hdmac_free
 * free the dma channel (will also abort any outstanding operations)
 */
extern int hdmac_free(u32 channel);

/*
 * hdmac_init
 *
 * initialization hdmac
 */
extern void hdmac_init(u32 chip, void *base);

#endif /* __PLAT_HDMAC_H */
