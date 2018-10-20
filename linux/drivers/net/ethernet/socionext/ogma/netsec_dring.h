/**
 * @file netsec_dring.h
 * @author K.MATSUI
 * @date 2014/Apr/25-2014/May/xx
 * @brief NETSEC Network Offload Macro device driver
 */
/* Copyright (C) 2015 SOCIONEXT INCORPORATED All Rights Reserved. */

#ifndef __NETSEC_DRING_H
#define __NETSEC_DRING_H

typedef struct {
	u32 desc[4];
} NETSEC_DESC;

typedef struct {
	u16 num_desc;
	u16 size_desc;
	u16 rd_idx;
	u16 wr_idx;

	u16 id;
	u8 full;
	u8 tx;

	u32 buff_size;

	spinlock_t lock;

	void *dring;
	dma_addr_t phys_dring;

	u32 *info;
} NETSEC_DRING;

int netsec_dring_tx_init(NETSEC_DRING * d, u16 num, u16 size, u16 id);
int netsec_dring_rx_init(NETSEC_DRING * d, u16 num, u16 size, u16 id,
			 u32 bufsz);
int netsec_dring_exit(NETSEC_DRING * d);
int netsec_dring_put_tx(NETSEC_DRING * d, void *desc, u32 info);
int netsec_dring_get_tx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 * pinfo);
int netsec_dring_clear_tx(NETSEC_DRING * d);
int netsec_dring_get_rx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 * pinfo);
int netsec_dring_reset_rx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 info);

int netsec_dring_get_tx_free(NETSEC_DRING * dring);
int netsec_dring_get_rx_free(NETSEC_DRING * dring);

#endif				/* __NETSEC_DRING_H */
