/**
 * ogma_desc_ring_access_internal.h
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
 * 
 */
#ifndef OGMA_DESC_RING_ACCESS_INTERNAL_H
#define OGMA_DESC_RING_ACCESS_INTERNAL_H

#include "ogma_basic_type.h"
#include "ogma_internal.h"

#define OGMA_TX_PKT_DESC_RING_OWN_FIELD        (31)
#define OGMA_TX_PKT_DESC_RING_LD_FIELD         (30)
#define OGMA_TX_PKT_DESC_RING_DRID_FIELD       (24)
#define OGMA_TX_PKT_DESC_RING_PT_FIELD         (21)
#define OGMA_TX_PKT_DESC_RING_TDRID_FIELD      (16)
#define OGMA_TX_PKT_DESC_RING_CC_FIELD         (15)
#define OGMA_TX_PKT_DESC_RING_FS_FIELD         (9)
#define OGMA_TX_PKT_DESC_RING_LS_FIELD         (8)
#define OGMA_TX_PKT_DESC_RING_CO_FIELD         (7)
#define OGMA_TX_PKT_DESC_RING_SO_FIELD         (6)
#define OGMA_TX_PKT_DESC_RING_TRS_FIELD        (4)

#define OGMA_RX_PKT_DESC_RING_OWN_FIELD        (31)
#define OGMA_RX_PKT_DESC_RING_LD_FIELD         (30)
#define OGMA_RX_PKT_DESC_RING_SDRID_FIELD      (24)
#define OGMA_RX_PKT_DESC_RING_FR_FIELD         (23)
#define OGMA_RX_PKT_DESC_RING_ER_FIELD         (21)
#define OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD (16)
#define OGMA_RX_PKT_DESC_RING_TDRID_FIELD      (12)
#define OGMA_RX_PKT_DESC_RING_FS_FIELD         (9)
#define OGMA_RX_PKT_DESC_RING_LS_FIELD         (8)
#define OGMA_RX_PKT_DESC_RING_CO_FIELD         (6)

#define OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD_MASK (0x3)

#define OGMA_MAX_TX_PKT_LEN       1518U
#define OGMA_MAX_TX_JUMBO_PKT_LEN 9018U

typedef struct ogma_tx_desc_entry_s ogma_tx_desc_entry_t;
typedef struct ogma_rx_desc_entry_s ogma_rx_desc_entry_t;

struct ogma_tx_desc_entry_s {

	ogma_uint32 attr;

	ogma_uint32 data_buf_addr;

	ogma_uint32 buf_len_info;

	ogma_uint32 reserved;
};

struct ogma_rx_desc_entry_s {

	ogma_uint32 attr;

	ogma_uint32 data_buf_addr;

	ogma_uint32 buf_len_info;

	ogma_uint32 reserved;

};

static __inline ogma_bool ogma_is_pkt_desc_ring(const ogma_desc_ring_t *
						desc_ring_p)
{
	return (desc_ring_p->rx_desc_ring_flag ||
		desc_ring_p->tx_desc_ring_flag);
}

static __inline void ogma_check_desc_own_sanity(ogma_ctrl_t * ctrl_p,
						const ogma_desc_ring_t *
						desc_ring_p, ogma_uint16 idx,
						ogma_uint expected_own)
{
	ogma_uint32 tmp;
	(void)ctrl_p;		/* Suppress compiler warning */

	tmp = *(ogma_uint32 *)
	    ((pfdep_cpu_addr_t) desc_ring_p->desc_ring_cpu_addr +
	     desc_ring_p->desc_entry_len * idx);

	pfdep_assert((tmp >> 31) == expected_own);
}

#endif				/* OGMA_DESC_RING_ACCESS_INTERNAL_H */
