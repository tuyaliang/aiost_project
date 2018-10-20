/**
 * @file netsec_api.h
 * @author K.MATSUI
 * @date 2014/Apr/25-2014/May/xx
 * @brief NETSEC Network Offload Macro device driver
 */
/* Copyright (C) 2015 SOCIONEXT INCORPORATED All Rights Reserved. */

#ifndef __NETSEC_API_H
#define __NETSEC_API_H

#define NETSEC_DEV_NAME "netsec"

#define NETSEC_IOC_MAGIC	0x67

#define NETSEC_IOCINIT      _IOWR( NETSEC_IOC_MAGIC, 0, struct netsec_init_arg )
#define NETSEC_IOCRESET     _IO  ( NETSEC_IOC_MAGIC, 1 )
#define NETSEC_IOCSMODE     _IO  ( NETSEC_IOC_MAGIC, 2 )
#define NETSEC_IOCGMODE     _IOR ( NETSEC_IOC_MAGIC, 3, u32 * )
#define NETSEC_IOCSSESSDB   _IOW ( NETSEC_IOC_MAGIC, 4, struct netsec_sessdb_arg )
#define NETSEC_IOCSCTX      _IOWR( NETSEC_IOC_MAGIC, 5, struct netsec_ctx_arg )
#define NETSEC_IOCGDRINFO   _IOWR( NETSEC_IOC_MAGIC, 6, struct netsec_drinfo_arg )
#define NETSEC_IOCWAITCRYPT _IOW ( NETSEC_IOC_MAGIC, 7, struct netsec_wcrypt_arg )
#define NETSEC_IOCRDCRYPT   _IOW ( NETSEC_IOC_MAGIC, 8, struct netsec_rdcrypt_arg )
#define NETSEC_IOCGTCNT90K  _IOR ( NETSEC_IOC_MAGIC, 9, struct netsec_tcnt32_arg )
#define NETSEC_IOCGTCNT8K   _IOR ( NETSEC_IOC_MAGIC, 10, struct netsec_tcnt32_arg )
#define NETSEC_IOCGTCNT27M  _IOR ( NETSEC_IOC_MAGIC, 11, struct netsec_tcnt64_arg )

#define NETSEC_IOC_MAXNR 12

#define	NETSEC_MODE_ETS			(0x80000000)
#define	NETSEC_MODE_VJS			(0x40000000)
#define	NETSEC_MODE_EN_JUMBO		(0x08000000)
#define	NETSEC_MODE_EN_ID_REP		(0x04000000)
#define	NETSEC_MODE_LOG_CKSM_ER		(0x00000008)
#define	NETSEC_MODE_LOG_HD_INCOMPLETE	(0x00000004)
#define	NETSEC_MODE_LOG_HD_ER		(0x00000002)

#define	NETSEC_MODE_TTS			(0x20000000)
#define	NETSEC_MODE_RFM			(0x10000000)

#define	NETSEC_MODE_MASK_PKT_CTRL1	(0xc000000e)
#define	NETSEC_MODE_MASK_PKT_CTRL2	(0x000000c0)
#define	NETSEC_REG_MASK_PKT_CTRL2	(0x0c000000)
#define	NETSEC_MODE_SHIFT_PKT_CTRL2	(20)
#define	NETSEC_MODE_MASK_STRM_CTRL	(0x38000000)

#define	NETSEC_SESSDB_STRM_ID_MIN	(0)
#define	NETSEC_SESSDB_STRM_ID_MAX	(31)
#define	NETSEC_SESSDB_BASIC_ID_MIN	(32)
#define	NETSEC_SESSDB_BASIC_ID_MAX	(63)

#define	NETSEC_LPB_RX_BUFF_LEN	(1520)

typedef struct {
	u32 desc0;
	u32 length;
	u32 reserved[2];
	u8 rx_buff[NETSEC_LPB_RX_BUFF_LEN];
} NETSEC_RX_DATA;

typedef struct {
	u32 num;
	u32 wr_count;
	u32 reserved[2];

	NETSEC_RX_DATA rxque[1];
} NETSEC_RX_QUE;

struct netsec_init_arg {
	u32 *dmac_hm_me_code;
	u32 dmac_hm_me_size;
	u32 *dmac_mh_me_code;
	u32 dmac_mh_me_size;
	u32 *pktc_code;
	u32 pktc_size;
	u16 num_descriptors[4 + 10];
	u32 rx_buf_size[4];
	u32 size_ctx;
	NETSEC_RX_QUE *lpbk_rx_que;
	u8 mac[6];
};

#define	NETSEC_SESS_DB_NUM_ITEM		(16)

struct netsec_sessdb_arg {
	u32 sess_db_id;
	u32 sess_db[NETSEC_SESS_DB_NUM_ITEM];
};

struct netsec_ctx_arg {
	u8 *ctx_data;
	u32 ctx_size;
	u32 ctx_offset;
	u32 ctx_addr;
};

struct netsec_drinfo_arg {
	u16 desc_id;
	u16 free;
	u16 total;
};

enum netsec_dring_id {
	NETSEC_DESC_RING_ID_PKT_TX = 0x00,
	NETSEC_DESC_RING_ID_PKT_RX = 0x01,
	NETSEC_DESC_RING_ID_VDO_TX = 0x02,
	NETSEC_DESC_RING_ID_ETH_LPB_RX = 0x03,
	NETSEC_DESC_RING_ID_ADO_TX = 0x04,
	NETSEC_DESC_RING_ID_RTP_LPB_RX = 0x05,
	NETSEC_DESC_RING_ID_CDB_TX = 0x06,

	NETSEC_DESC_RING_ID_CRY_TX = 0x08,
	NETSEC_DESC_RING_ID_CRY_RX = 0x09,
	NETSEC_DESC_RING_ID_JPG1_TX = 0x0a,
	NETSEC_DESC_RING_ID_JPG2_TX = 0x0b,
	NETSEC_DESC_RING_ID_VSUB_TX = 0x0c,
	NETSEC_DESC_RING_ID_MDT_TX = 0x0d,
	NETSEC_DESC_RING_ID_TS_TX = 0x0e,
};

#define	NETSEC_WCRYPT_MAX_PKTS			(4)

struct netsec_wcrypt_arg {
	u32 timeout;
	u32 recv_pkts;
	u32 recv_sizes[NETSEC_WCRYPT_MAX_PKTS];
};

struct netsec_rdcrypt_arg {
	u8 *recv_buf;
	u32 recv_size;
	u32 recv_len;
};

struct netsec_tcnt32_arg {
	u32 tcount;
};

struct netsec_tcnt64_arg {
	u32 upper;
	u32 lower;
};

// write: Descriptor
struct netsec_desc {
	u32 uniq_id;
	u32 desc[4];
};

// read: completion
struct netsec_comp {
	u32 uniq_id;
	u32 result;
};

enum netsec_comp_result {
	NETSEC_COMP_RESULT_OKAY = 0x00,
	NETSEC_COMP_RESULT_EXOKAY = 0x01,
	NETSEC_COMP_RESULT_SLVERR = 0x02,
	NETSEC_COMP_RESULT_DECERR = 0x03,
	NETSEC_COMP_RESULT_RESET = 0xffffffff,
};

#endif				/* __NETSEC_API_H */
