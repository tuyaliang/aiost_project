/**
 * @file netsec_drv.h
 * @author K.MATSUI
 * @date 2014/Apr/25-2014/May/xx
 * @brief NETSEC Network Offload Macro device driver
 */
/* Copyright (C) 2015 SOCIONEXT INCORPORATED All Rights Reserved. */

#ifndef __NETSEC_DRV_H
#define __NETSEC_DRV_H

#include <linux/reset.h>
#include "ogma_driver_global.h"

#define	NETSEC_DESC_OWN		(0x80000000)
#define	NETSEC_DESC_LD		(0x40000000)
#define	NETSEC_DESC_DRID_MASK	(0x0f000000)
#define	NETSEC_DESC_DRID_SHIFT	(24)
#define	NETSEC_DESC_MARK	(0x00200000)
#define	NETSEC_DESC_LPBE	(0x00008000)
#define	NETSEC_DESC_AST_INT	(0x00004000)
#define	NETSEC_DESC_CR		(0x00002000)
#define	NETSEC_DESC_FN		(0x00001000)
#define	NETSEC_DESC_SI		(0x00000800)
#define	NETSEC_DESC_LI		(0x00000400)
#define	NETSEC_DESC_FS		(0x00000200)
#define	NETSEC_DESC_LS		(0x00000100)
#define	NETSEC_DESC_ENC		(0x00000080)
#define	NETSEC_DESC_DTID_MASK	(0x00000070)
#define	NETSEC_DESC_DTID_SHIFT	(4)
#define	NETSEC_DESC_STID_MASK	(0x0000000f)
#define	NETSEC_DESC_STID_SHIFT	(0)

#define	NETSEC_TX_DONE_QUE_NUM	(512)

#define	NETSEC_NUM_DESCRIPTORS_MIN	(16)
#define	NETSEC_NUM_DESCRIPTORS_MAX	(1024)

#define	NETSEC_RX_BUF_SIZE_MIN		(1520)
#define	NETSEC_RX_BUF_SIZE_MAX		(2048)

#define	NETSEC_SIZE_CTX_MIN		(4096)
#define	NETSEC_SIZE_CTX_MAX		(8192)

typedef struct {
	NETSEC_DESC *dring;
	void **rxbuff;
	u16 num_desc;
	u16 rd_idx;
} NETSEC_RX_DRING;

typedef struct {
	u32 desc[12];
} NETSEC_CRYPT_DESC;

typedef struct {
	NETSEC_CRYPT_DESC *dring;
	void **rxbuff;
	u16 num_desc;
	u16 rd_idx;
} NETSEC_CRYPT_RX_DRING;

typedef struct {
	u16 num_desc;
	u16 rd_idx;
	u16 wr_idx;
	u16 flags;
	NETSEC_DESC *dring;
	u32 *uniq_id;
} NETSEC_TX_DRING;

typedef struct {
	u16 num_desc;
	u16 rd_idx;
	u16 wr_idx;
	u16 flags;
	NETSEC_CRYPT_DESC *dring;
	u32 *uniq_id;
} NETSEC_CRYPT_TX_DRING;

typedef struct {
	u32 uniq_id;
	u32 result;
} NETSEC_TX_DONE;

typedef struct {
	u16 num;
	u16 rd_idx;
	u16 wr_idx;
	u16 flags;
	NETSEC_TX_DONE *txque;
} NETSEC_TX_DONE_QUE;

typedef struct {
	struct device *dev;
	struct cdev cdev;
	dev_t dev_num;
	struct net_device *netdev_p;

	int irqa, irqb;
	void __iomem *base;
	struct clk *netauclk;
	struct clk *rclk;
	struct clk *osc_clk;
	struct reset_control *rst;

	u8 mac[ETH_ALEN];

	NETSEC_RX_DRING rx_dring[3];
	NETSEC_CRYPT_RX_DRING crypt_rx_dring;
	NETSEC_TX_DRING tx_dring[9];
	NETSEC_CRYPT_TX_DRING crypt_tx_dring;

	NETSEC_DRING dring[15];

	wait_queue_head_t txdoneq;
	struct semaphore txdones;

	spinlock_t tx_done_lock;
	NETSEC_TX_DONE_QUE tx_done_que;

	u8 *ctx_data_addr;
	phys_addr_t ctx_phys_addr;
	u32 ctx_size;

	u16 num_descriptors[4 + 10];
	u32 rx_buf_size[4];
	NETSEC_RX_QUE *lpbk_rx_que;
	NETSEC_RX_QUE *lpbk_rx_que_addr;
	struct task_struct *lpbk_rx_que_tsk;
	struct work_struct lpbk_rx_que_work;
	struct semaphore lpbk_rx_que_sem;

	spinlock_t sessdb_lock;

	int init_done;
	int reinit;

	spinlock_t init_spinlock;

	struct workqueue_struct *mac_st_tsk;
	struct work_struct mac_st_work;

	struct ogma_priv *priv;
} NETSEC_DRIVER_DATA;

#endif				/* __NETSEC_DRV_H */
