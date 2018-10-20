/**
 * ogma_api.h
 *
 *  Copyright (c) 2015 SOCIONEXT INCORPORATED.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef OGMA_API_H
#define OGMA_API_H

#include "ogma_version.h"
#include "ogma_config.h"
#include "ogma_basic_type.h"
#include "pfdep.h"
/**
 * Check configuration macro settings.
 */

#ifdef OGMA_CONFIG_CLK_HZ
#if ( (OGMA_CONFIG_CLK_HZ < 0x200000) || (OGMA_CONFIG_CLK_HZ > 0x10000000) )
#error "OGMA_CONFIG_CLK_HZ is not appropriate."
#endif				/* ( (OGMA_CONFIG_CLK_HZ < 0x200000) || (OGMA_CONFIG_CLK_HZ > 0x10000000) ) */
#else				/* ! OGMA_CONFIG_CLK_HZ */
#error "OGMA_CONFIG_CLK_HZ is not given."
#endif				/* OGMA_CONFIG_CLK_HZ */

#ifndef OGMA_CONFIG_GMAC_CLK_HZ
#define OGMA_CONFIG_GMAC_CLK_HZ OGMA_CONFIG_CLK_HZ
#endif

/**
 * Number of Common Descriptor ring id
 */
#define OGMA_DESC_RING_ID_NRM_TX      0
#define OGMA_DESC_RING_ID_NRM_RX      1
#define OGMA_DESC_RING_ID_RESERVED_RX 2
#define OGMA_DESC_RING_ID_RESERVED_TX 3
#define OGMA_DESC_RING_ID_GMAC   15
#define OGMA_DESC_RING_ID_MAX     3

/**
 * Numbre of TCP Segmentation length limits
 */
#define OGMA_TCP_SEG_LEN_MAX 1460
#define OGMA_TCP_JUMBO_SEG_LEN_MAX 8960
#define OGMA_TCP_SEG_LEN_MIN  536

/**
 * Number of checksum calculation result for received packet
 */
#define OGMA_RX_CKSUM_RESULT_OK       0x1
#define OGMA_RX_CKSUM_RESULT_NG       0x2
#define OGMA_RX_CKSUM_RESULT_NOTAVAIL 0x0

/**
 * Number of top interrupt enable register bit field
 */
#define OGMA_TOP_IRQ_REG_CODE_LOAD_END   (1UL << 20)
#define OGMA_TOP_IRQ_REG_NRM_RX	         (1UL <<  1)
#define OGMA_TOP_IRQ_REG_NRM_TX	         (1UL <<  0)

/**
 *  Number of top channel enable register bit field
 */
#define OGMA_CH_IRQ_REG_EMPTY   (1UL << 17)
#define OGMA_CH_IRQ_REG_ERR     (1UL << 16)
#define OGMA_CH_IRQ_REG_PKT_CNT (1UL << 15)
#define OGMA_CH_IRQ_REG_TIMEUP  (1UL << 14)
#define OGMA_CH_IRQ_REG_RCV     (OGMA_CH_IRQ_REG_PKT_CNT | OGMA_CH_IRQ_REG_TIMEUP)

/**
 *  Number of top channel enable register bit field for F_TAIKI
 */
#define OGMA_CH_IRQ_REG_TX_DONE (1UL << 15)
#define OGMA_CH_IRQ_REG_SND     (OGMA_CH_IRQ_REG_TX_DONE | OGMA_CH_IRQ_REG_TIMEUP)

/**
 *  Number of RGSR register bit field
 */
#define OGMA_GMAC_RGSR_REG_LS  (1U << 3)
#define OGMA_GMAC_RGSR_REG_LSP (1U << 1)
#define OGMA_GMAC_RGSR_REG_LM  (1U << 0)

/**
 * Number of various limits
 */
#define OGMA_DESC_ENTRY_NUM_MIN        2
#define OGMA_DESC_ENTRY_NUM_MAX        2047
#define OGMA_INT_PKTCNT_MAX            2047

/**
 * Number of ogma phy interface setting
 */
#define OGMA_PHY_INTERFACE_GMII  0
#define OGMA_PHY_INTERFACE_RGMII 1
#define OGMA_PHY_INTERFACE_RMII  4

/**
 * Number of ogma link speed setting
 */
#define OGMA_PHY_LINK_SPEED_1G   0
#define OGMA_PHY_LINK_SPEED_100M 1
#define OGMA_PHY_LINK_SPEED_10M  2

/**
 * Number of flow control limits
 */
#define OGMA_FLOW_CTRL_START_THRESHOLD_MAX 383U
#define OGMA_FLOW_CTRL_STOP_THRESHOLD_MAX  383U
#define OGMA_FLOW_CTRL_PAUSE_TIME_MIN      5

enum ogma_err_e {
	OGMA_ERR_OK = 0,
	OGMA_ERR_PARAM,
	OGMA_ERR_ALLOC,
	OGMA_ERR_BUSY,
	OGMA_ERR_RANGE,
	OGMA_ERR_DATA,
	OGMA_ERR_NOTAVAIL,
	OGMA_ERR_INTERRUPT,
	OGMA_ERR_AGAIN,
	OGMA_ERR_INVALID
};

typedef void *ogma_handle_t;
typedef struct ogma_param_s ogma_param_t;
typedef struct ogma_pkt_ctrl_param_s ogma_pkt_ctrl_param_t;
typedef struct ogma_desc_ring_param_s ogma_desc_ring_param_t;
typedef enum ogma_err_e ogma_err_t;
typedef ogma_uint8 ogma_desc_ring_id_t;
typedef struct ogma_tx_pkt_ctrl_s ogma_tx_pkt_ctrl_t;
typedef struct ogma_rx_pkt_info_s ogma_rx_pkt_info_t;
typedef struct ogma_frag_info_s ogma_frag_info_t;
typedef struct ogma_gmac_config_s ogma_gmac_config_t;
typedef struct ogma_gmac_mode_s ogma_gmac_mode_t;

struct ogma_gmac_config_s {
	ogma_uint8 phy_interface;
};

struct ogma_pkt_ctrl_param_s {
	ogma_uint log_chksum_er_flag:1;
	ogma_uint log_hd_imcomplete_flag:1;
	ogma_uint log_hd_er_flag:1;
};

struct ogma_desc_ring_param_s {
	ogma_uint valid_flag:1;
	ogma_uint little_endian_flag:1;
	ogma_uint tmr_mode_flag:1;
	ogma_uint16 entry_num;
};

struct ogma_param_s {
	ogma_uint use_gmac_flag:1;
	ogma_uint use_jumbo_pkt_flag:1;
	ogma_pkt_ctrl_param_t pkt_ctrl_param;
	ogma_desc_ring_param_t desc_ring_param[OGMA_DESC_RING_ID_MAX + 1];
	ogma_gmac_config_t gmac_config;
};

struct ogma_tx_pkt_ctrl_s {
	ogma_uint cksum_offload_flag:1;
	ogma_uint tcp_seg_offload_flag:1;
	ogma_desc_ring_id_t target_desc_ring_id;
	ogma_uint16 tcp_seg_len;
};

struct ogma_rx_pkt_info_s {
	ogma_uint fragmented_flag:1;
	ogma_uint err_flag:1;
	ogma_uint rx_cksum_result:2;
	ogma_uint8 err_code;
};

struct ogma_frag_info_s {
	pfdep_phys_addr_t phys_addr;
	void *addr;
	ogma_uint32 len;
};

struct ogma_gmac_mode_s {
	ogma_uint half_duplex_flag:1;
	ogma_uint flow_ctrl_enable_flag:1;
	ogma_uint8 link_speed;
	ogma_uint16 flow_ctrl_start_threshold;
	ogma_uint16 flow_ctrl_stop_threshold;
	ogma_uint16 pause_time;
};

/**************************
***************************
***************************/

ogma_err_t ogma_init(void *base_addr,
		     pfdep_dev_handle_t dev_handle,
		     const ogma_param_t * param_p,
		     void *netsec_handle, ogma_handle_t * ogma_handle_p);

ogma_err_t ogma_terminate(ogma_handle_t ogma_handle);
void ogma_cnt_minus(void);

ogma_err_t ogma_start_gmac(ogma_handle_t ogma_handle,
			   ogma_bool rx_flag, ogma_bool tx_flag);

ogma_err_t ogma_stop_gmac(ogma_handle_t ogma_handle,
			  ogma_bool rx_flag, ogma_bool tx_flag);

ogma_err_t ogma_set_gmac_mode(ogma_handle_t ogma_handle,
			      const ogma_gmac_mode_t * gmac_mode_p);

void ogma_set_phy_reg(ogma_handle_t ogma_handle,
		      ogma_uint8 phy_addr,
		      ogma_uint8 reg_addr, ogma_uint16 value);

ogma_uint16 ogma_get_phy_reg(ogma_handle_t ogma_handle,
			     ogma_uint8 phy_addr, ogma_uint8 reg_addr);

ogma_uint32 ogma_get_top_irq_enable(ogma_handle_t ogma_handle);

ogma_uint32 ogma_get_top_irq_status_non_clear(ogma_handle_t ogma_handle,
					      ogma_bool mask_flag);

ogma_err_t ogma_clear_top_irq_status(ogma_handle_t ogma_handle,
				     ogma_uint32 value);

ogma_uint32 ogma_get_desc_ring_irq_enable(ogma_handle_t ogma_handle,
					  ogma_desc_ring_id_t ring_id);

ogma_uint32 ogma_get_desc_ring_irq_status_non_clear(ogma_handle_t ogma_handle,
						    ogma_desc_ring_id_t ring_id,
						    ogma_bool mask_flag);

ogma_err_t ogma_clear_desc_ring_irq_status(ogma_handle_t ogma_handle,
					   ogma_desc_ring_id_t ring_id,
					   ogma_uint32 value);

/* ogma_desc_ring_access.c */
ogma_err_t ogma_start_desc_ring(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id);

ogma_err_t ogma_stop_desc_ring(ogma_handle_t ogma_handle,
			       ogma_desc_ring_id_t ring_id);

ogma_uint16 ogma_get_rx_num(ogma_handle_t ogma_handle,
			    ogma_desc_ring_id_t ring_id);

ogma_uint16 ogma_get_tx_avail_num(ogma_handle_t ogma_handle,
				  ogma_desc_ring_id_t ring_id);

ogma_err_t ogma_clean_tx_desc_ring(ogma_handle_t ogma_handle,
				   ogma_desc_ring_id_t ring_id);

ogma_err_t ogma_clean_rx_desc_ring(ogma_handle_t ogma_handle,
				   ogma_desc_ring_id_t ring_id);

ogma_err_t ogma_set_tx_pkt_data(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id,
				const ogma_tx_pkt_ctrl_t * tx_pkt_ctrl_p,
				ogma_uint8 scat_num,
				const ogma_frag_info_t * scat_info_p,
				pfdep_pkt_handle_t pkt_handle);

ogma_err_t ogma_get_rx_pkt_data(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id,
				ogma_rx_pkt_info_t * rx_pkt_info_p,
				ogma_frag_info_t * frag_info_p,
				ogma_uint16 * len_p,
				pfdep_pkt_handle_t * pkt_handle_p);

ogma_err_t ogma_enable_top_irq(ogma_handle_t ogma_handle,
			       ogma_uint32 irq_factor);

ogma_err_t ogma_disable_top_irq(ogma_handle_t ogma_handle,
				ogma_uint32 irq_factor);

ogma_err_t ogma_enable_desc_ring_irq(ogma_handle_t ogma_handle,
				     ogma_desc_ring_id_t ring_id,
				     ogma_uint32 irq_factor);

ogma_err_t ogma_disable_desc_ring_irq(ogma_handle_t ogma_handle,
				      ogma_desc_ring_id_t ring_id,
				      ogma_uint32 irq_factor);

ogma_uint32 ogma_get_hw_ver(ogma_handle_t ogma_handle);

ogma_uint32 ogma_get_mcr_ver(ogma_handle_t ogma_handle);

/**
 * Set up IRQ coalesce parameters.
 *
 * [Note]
 *  - This is a tentative implementation.
 *    Not tested enough. Use with care.
 *
 *  - Call this function after every invocation of ogma_start_desc_ring()
 *    because ogma_start_desc_ring() resets IRQ coalesce settings.
 *
 */
ogma_err_t ogma_set_irq_coalesce_param(ogma_handle_t ogma_handle,
				       ogma_desc_ring_id_t ring_id,
				       ogma_uint16 int_pktcnt,
				       ogma_bool int_tmr_unit_ms_flag,
				       ogma_uint16 int_tmr_cnt);

ogma_err_t ogma_read_gmac_stat(ogma_handle_t ogma_handle,
			       ogma_uint32 * value_p, ogma_bool reset_flag);

ogma_err_t ogma_reset_gmac_stat(ogma_handle_t ogma_handle);

#endif				/* OGMA_API_H */
