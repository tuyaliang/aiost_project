/*
 *
 * Copyright (C) 2010-2015 SOCIONEXT
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

#ifndef __PLAT_XDMAC_H
#define __PLAT_XDMAC_H

#include <linux/device.h>

#define MAX_XDMAC_CHANNELS	8

#define XDMAC_REG_XDACS	0x0	/* XDACS registers offset */
#define XDMAC_REG_XDTBC	0x0	/* XDTBC registers offset */
#define XDMAC_REG_XDSSA	0x4	/* XDSSA registers offset */
#define XDMAC_REG_XDDSA	0x8	/* XDDSA registers offset */
#define XDMAC_REG_XDSAC	0xc	/* XDSAC registers offset */
#define XDMAC_REG_XDDAC	0x10	/* XDDAC registers offset */
#define XDMAC_REG_XDDCC	0x14	/* XDDCC registers offset */
#define XDMAC_REG_XDDES	0x18	/* XDDES registers offset */
#define XDMAC_REG_XDDPC	0x1c	/* XDDPC registers offset */
#define XDMAC_REG_XDDSD	0x20	/* XDDSD registers offset */

#define XDACS		XDMAC_REG_XDACS
#define XDTBC(ch)  (XDMAC_REG_XDTBC + ((ch * 0x30) + 0x10))
#define XDSSA(ch)  (XDMAC_REG_XDSSA + ((ch * 0x30) + 0x10))
#define XDDSA(ch)  (XDMAC_REG_XDDSA + ((ch * 0x30) + 0x10))
#define XDSAC(ch)  (XDMAC_REG_XDSAC + ((ch * 0x30) + 0x10))
#define XDDAC(ch)  (XDMAC_REG_XDDAC + ((ch * 0x30) + 0x10))
#define XDDCC(ch)  (XDMAC_REG_XDDCC + ((ch * 0x30) + 0x10))
#define XDDES(ch)  (XDMAC_REG_XDDES + ((ch * 0x30) + 0x10))
#define XDDPC(ch)  (XDMAC_REG_XDDPC + ((ch * 0x30) + 0x10))
#define XDDSD(ch)  (XDMAC_REG_XDDSD + ((ch * 0x30) + 0x10))

/* XDMAC state */
#define XDMAC_PREPARE	0
#define XDMAC_IDLE	1
#define XDMAC_RUNNING	2
#define XDMAC_STOP_REQUEST 3
#define XDMAC_STOP_REQUEST_NOWAIT 4
#define XDMAC_IRQ_HANDLED 0xF0000000


/* XDACS all channel setting */
#define XDACS_XE	(0x1 << 28)
#define XDACS_CP	(0x1 << 24)
#define XDACS_LP	(0x1 << 20)
#define XDACS_XS	(0x1 << 16)

/* XDSAC source access configuration */
#define XDSAC_SCT_BUFFERABLE	(0x1 << 20)
#define XDSAC_SCT_CACHEABLE	(0x1 << 21)
#define XDSAC_SCT_READALLOC	(0x1 << 22)
#define XDSAC_SCT_WRITEALLOC	(0x1 << 23)

#define XDSAC_SBS_BYTE		(0x0 << 16)
#define XDSAC_SBS_HALFWORD	(0x1 << 16)
#define XDSAC_SBS_WORD		(0x2 << 16)
#define XDSAC_SBS_DOUBLEWORD	(0x3 << 16)

#define XDSAC_SBL_1	(0x0 << 8)
#define XDSAC_SBL_2	(0x1 << 8)
#define XDSAC_SBL_3	(0x2 << 8)
#define XDSAC_SBL_4	(0x3 << 8)
#define XDSAC_SBL_5	(0x4 << 8)
#define XDSAC_SBL_6	(0x5 << 8)
#define XDSAC_SBL_7	(0x6 << 8)
#define XDSAC_SBL_8	(0x7 << 8)
#define XDSAC_SBL_9	(0x8 << 8)
#define XDSAC_SBL_10	(0x9 << 8)
#define XDSAC_SBL_11	(0xa << 8)
#define XDSAC_SBL_12	(0xb << 8)
#define XDSAC_SBL_13	(0xc << 8)
#define XDSAC_SBL_14	(0xd << 8)
#define XDSAC_SBL_15	(0xe << 8)
#define XDSAC_SBL_16	(0xf << 8)


#define XDSAC_SAF	(0x1 << 2)
#define XDSAC_SRL	(0x1 << 0)


/* XDDAC destination acces configuration */
#define XDDAC_DCT_BUFFERABLE	(0x1 << 20)
#define XDDAC_DCT_CACHEABLE	(0x1 << 21)
#define XDDAC_DCT_READALLOC	(0x1 << 22)
#define XDDAC_DCT_WRITEALLOC	(0x1 << 23)

#define XDDAC_DBS_BYTE	(0x0 << 16)
#define XDDAC_DBS_HALFWORD	(0x1 << 16)
#define XDDAC_DBS_WORD	(0x2 << 16)
#define XDDAC_DBS_DOUBLEWORD	(0x3 << 16)

#define XDDAC_DBL_1	(0x0 << 8)
#define XDDAC_DBL_2	(0x1 << 8)
#define XDDAC_DBL_3	(0x2 << 8)
#define XDDAC_DBL_4	(0x3 << 8)
#define XDDAC_DBL_5	(0x4 << 8)
#define XDDAC_DBL_6	(0x5 << 8)
#define XDDAC_DBL_7	(0x6 << 8)
#define XDDAC_DBL_8	(0x7 << 8)
#define XDDAC_DBL_9	(0x8 << 8)
#define XDDAC_DBL_10	(0x9 << 8)
#define XDDAC_DBL_11	(0xa << 8)
#define XDDAC_DBL_12	(0xb << 8)
#define XDDAC_DBL_13	(0xc << 8)
#define XDDAC_DBL_14	(0xd << 8)
#define XDDAC_DBL_15	(0xe << 8)
#define XDDAC_DBL_16	(0xf << 8)


#define XDDAC_DAF	(0x1 << 2)
#define XDDAC_DRL	(0x1 << 0)


/* XDDCC descriptor chain configuration */
#define XDDCC_DCN	(0x1 << 0)


/* XDDES DMA enable setting */
#define XDDES_CE	(0x1 << 28)
#define XDDES_SE	(0x1 << 24)
#define XDDES_SE_MASK	(0x1 << 24)

#define XDDES_TF_SOFTWEARE	(0x1 << 20)
#define XDDES_TF_DREQ0		(0x2 << 20)
#define XDDES_TF_DREQ1		(0x3 << 20)
#define XDDES_TF_DREQ2		(0x4 << 20)
#define XDDES_TF_DREQ3		(0x5 << 20)
#define XDDES_TF_DREQ4		(0x6 << 20)
#define XDDES_TF_DREQ5		(0x7 << 20)
#define XDDES_TF_DREQ6		(0x8 << 20)
#define XDDES_TF_DREQ7		(0x9 << 20)
#define XDDES_TF_DREQ8		(0xa << 20)
#define XDDES_TF_DREQ9		(0xb << 20)
#define XDDES_TF_DREQ10		(0xc << 20)
#define XDDES_TF_DREQ11		(0xd << 20)
#define XDDES_TF_DREQ12		(0xe << 20)
#define XDDES_TF_DREQ13		(0xf << 20)
#define XDDES_TF_MASK		(0xf << 20)


#define XDDES_SA	(0x1 << 15)
#define XDDES_BURST	(0x0 << 12)
#define XDDES_BLOCK	(0x1 << 12)
#define XDDES_BR	(0x1 << 8)
#define XDDES_AT_SRC	(0x0 << 4)
#define XDDES_AT_DES	(0x1 << 4)
#define XDDES_EI	(0x1 << 1)
#define XDDES_TI	(0x1 << 0)


/* XDDPC DMA protection control */
#define XDDPC_SP0_PRIVILEGED_ACCESS	(0x1 << 4)
#define XDDPC_SP1_SECURE_ACCESS	(0x1 << 5)
#define XDDPC_SP2_DATA_ACCESS	(0x0 << 6)

#define XDDPC_DP0_PRIVILEGED_ACCESS	(0x1 << 0)
#define XDDPC_DP1_SECURE_ACCESS	(0x1 << 1)
#define XDDPC_DP2_DATA_ACCESS	(0x0 << 2)


/* XDDSD  status display */
#define XDDSD_TS_RUNNING	(0x1 << 16)

#define XDDSD_IS_MASK					(0xf)
#define XDDSD_IS_NONE					(0x0)
#define XDDSD_IS_STOP_BY_DSTP				(0x1)
#define XDDSD_IS_DISABLE_CE_XE				(0x2)
#define XDDSD_IS_SOURCE_ACCESS_ERROR			(0x4)
#define XDDSD_IS_DESTINATION_ACCESS_ERROR		(0x5)
#define XDDSD_IS_DESCRIPTOR_CHAIN_MEM_ACCESS_ERROR	(0x6)
#define XDDSD_IS_NORMAL_END				(0x8)

#define XDMAC_REQ_DATA_FLUSHED				0xFF000000


#ifndef __PLAT_HDMAC_H
struct dma_req_data {
	u32	size;			/* request size in bytes */
	dma_addr_t	src;		/* source of DMA data */
	dma_addr_t	dst;		/* distnation of DMA data */
	void *irq_data;			/* data for the callback */
	/* driver handles */
	void (*callback_fn)(u32 channel, void *irq_data, int state);
				/* request done callback function
				 * channel : channel number
				 * irq_data: callback data
				 * state : DMA stopped state
				 */
};
#endif

/* XDMAC Descriptor chain */
struct xdmac_desc {
	unsigned long xdtbc;	/* XDTBC register value */
	unsigned long xdssa;	/* XDSSA register value */
	unsigned long xddsa;	/* XDDSA register value */
	unsigned long xdsac;	/* XDSAC register value */
	unsigned long xddac;	/* XDDAC register value */
	unsigned long xddcc;	/* XDDCC register value */
} __aligned(4);


struct xdmac_req {
	struct dma_req_data req_data; /* DMA request */
	u32 xdsac;	/* XDSAC register value */
	u32 xddac;	/* XDDAC register value */
	u32 xddcc;	/* XDDCC register value */
	u32 xddes;	/* XDDES register value */
	u32 xddpc;	/* XDDPC register value */
	struct list_head node;
};

struct xdmac_chan {
	/* channel state flags and information */
	u32 number;	/* number of this xdma channel */
	u32 irq;	/* channel irq */

	/* channel state */
	u8 state;	/* DMA state */

	/* channel's hardware position and configuration */
	u32 xdsac;	/* XDSAC register value */
	u32 xddac;	/* XDDAC register value */
	u32 xddcc;	/* XDDCC register value */
	u32 xddes;	/* XDDES register value */
	u32 xddpc;	/* XDDPC register value */

	/* channel configuration */
	u8 autostart_flg;	/* DMA auto start when request loaded */

	/* xdmac request list and information */
	struct list_head list;

	spinlock_t lock;	/* protect req info */
	struct completion stop_completion;
};


/* functions --------------------------------------------------------------- */

/* xdmac_get_channel
 *
 * request a dma channel exclusivley
 */
extern int xdmac_get_channel(u32 channel, u8 autostart_flg);

/* xdmac_enqueue
 *
 * place the given request onto the queue of operations for the channel.
 */
extern int xdmac_enqueue(u32 channel, struct xdmac_req *xdmac_req);

/* xdmac_start
 * start a dma channel going
 */
extern int xdmac_start(u32 channel);

/* xdmac_getposition
 *
 * returns the current transfer points for the dma source and destination
 */
extern int xdmac_getposition(u32 channel, u32 *src, u32 *dst);

/* xdmac_stop(synchronous)
 *
 * stop a dma channel
 */
extern int xdmac_stop(u32 channel);

/* xdmac_stop_nowait(asynchronous)
 *
 * stop a dma channel
 */
extern int xdmac_stop_nowait(u32 channel);

/* xdmac_flush
 *
 * remove all current and pending transfers
 */
extern int xdmac_flush(u32 channel);

/* xdmac_free
 * free the dma channel (will also abort any outstanding operations)
 */
extern int xdmac_free(u32 channel);
/* xdmac_init
 * initialization xdmac
 */
extern void xdmac_init(void *base);

#endif /* __PLAT_XDMAC_H */


