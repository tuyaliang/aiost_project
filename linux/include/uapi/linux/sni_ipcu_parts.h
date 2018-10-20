/**
 * @file sni_ipcu_parts.h
 * @author
 * @date  
 * @brief SNI IPCU Parts
 */
/* Copyright 2015 Socionext Inc. */

#ifndef __SNI_IPCU_PARTS_H
#define __SNI_IPCU_PARTS_H


#include <linux/types.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>


#define SNI_IPCU_SHAREDMEM_TOP        0x73100000UL
#define SNI_IPCU_SHAREDMEM_SIZE       0x00100000UL

#define SNI_IPCU_MAGIC_CODE           0xBEEFCAFEUL
#define SNI_IPCU_MAGIC_SIZE           0x00000020

/* definition of interface constant value */
#define SNI_IPCU_DIR_INIT 0
#define SNI_IPCU_DIR_SEND 1
#define SNI_IPCU_DIR_RECV 2

#define SNI_IPCU_CHSTAT_OPEN 1  /* this channel is used by someone    */
#define SNI_IPCU_CHSTAT_IGN  2  /* this channel is ignored after user */

/* make a bitmap data from channel_id */
#define SNI_IPCU_BITMAP(id) (1 << (id))

/**
 * @brief Tentative definition
 */
#define IPCU_MAX_DEPTH 256
#define IPCU_MAX_LEN   2048
#define IPCU_MAX_UNIT  2
#define IPCU_MAX_CH    (8 * 2)

#define IPCU_MAX_DATA  9

/**
 * @brifef data body definition for static reserved.
 */
//struct sni_ipcu_data {
//    u32 array[IPCU_MAX_DEPTH][IPCU_MAX_LEN / sizeof( u32 )];
//};

/**
 * @note
 * memory map between CPUs. this memory area is configured to uncacheable.
 * for RTOS developer, memory map image is described as follows:
 *
 */
struct sni_ipcu_ch_config {
    u32 channel_id;     /* channel id                                     */
    u32 direction;      /* data direction 1:Linux->RTOS 2:RTOS->Linux     */
    u32 stat_tx;        /* status of TX channel                           */
    u32 stat_rx;        /* status of RX channel                           */
    u32 data_size;      /* size of data, in Bytes.                        */
    u32 data_depth;     /* number of data.                                */
    u32 offset2dat;     /* offset from top of shared memory between CPUs. */
    u32 rd_idx;
    u32 wr_idx;
    u32 data[IPCU_MAX_DATA];
//    struct sni_ipcu_data data;
};

struct sni_ipcu_device {
	struct miscdevice miscdev;
	char devname[24];
	struct mutex mlock;
	unsigned dest_unit;        /* destination unit i/f    */
	unsigned dest_channel;     /* destination channel i/f */
};

struct ipcu_driver_info {
	unsigned int dest_unit;
	void __iomem *ipcu_io_mem;
	void __iomem *ipcu_magic_mem;
	unsigned int dest_channel[IPCU_MAX_CH];
	unsigned int dst_int_ch[IPCU_MAX_CH];
	unsigned int src_int_ch[IPCU_MAX_CH];
	int ipcu_rec_irq[IPCU_MAX_CH];
	int ipcu_ack_irq[IPCU_MAX_CH];
	struct mutex ch_mutex[IPCU_MAX_CH];
	struct completion ack_notify[IPCU_MAX_CH];
};

extern struct ipcu_driver_info ipcu_drv_inf[IPCU_MAX_UNIT];

/**
 *@brief initialize IPCU.
 *
 */
int sni_ipcu_ch_init(u32 unit, u32 channel, u32 direction, void* ipcu_dev);

/**
 *@brief exit IPCU.
 *
 */
void sni_ipcu_ch_exit(u32 unit, u32 channel, void* ipcu_dev);

/**
 *@brief Open channel.
 *
 */
int sni_ipcu_opench(u32 unit, u32 channel, u32 direction);

/**
 *@brief Close channel.
 *
 */
int sni_ipcu_closech(u32 unit, u32 channel, u32 direction);

/**
 * @brief Send Message
 */
int sni_ipcu_send_msg(u32 unit, u32 channel, void* buf, u32 len, u32 flags);

/**
 * @brief Recive Message
 */
int sni_ipcu_recv_msg(u32 unit, u32 channel, void* buf, u32 len, u32 flags);

/**
 * @brief Send Message
 */
int sni_ipcu_send_msg_kernel(u32 unit, u32 channel, void* buf, u32 len, u32 flags);

/**
 * @brief Recive Message
 */
int sni_ipcu_recv_msg_kernel(u32 unit, u32 channel, void* buf, u32 len, u32 flags);

/**
 * @brief Send Ack
 */
int sni_ipcu_ack_send(u32 unit, u32 channel);

/**
 * @brief Recive Message flash
 */
int sni_ipcu_recv_flsh(u32 unit, u32 channel);

#endif  /* __SNI_IPCU_PARTS_H */
