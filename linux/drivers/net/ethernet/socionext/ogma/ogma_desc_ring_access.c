/**
 * ogma_desc_ring_access.c
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
#include "ogma_internal.h"
#include "ogma_basic_access.h"
#include "ogma_desc_ring_access_internal.h"

#undef STATIC
#ifdef MODULE_TEST
#define STATIC
#else
#define STATIC static
#endif

const ogma_uint32 ogma_desc_start_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_DESC_START,
	OGMA_REG_ADDR_NRM_RX_DESC_START,
	OGMA_REG_ADDR_RESERVED_RX_DESC_START,
	OGMA_REG_ADDR_RESERVED_TX_DESC_START
};

const ogma_uint32 desc_ring_irq_inten_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_INTEN,
	OGMA_REG_ADDR_NRM_RX_INTEN,
	0,
	0
};

const ogma_uint32 desc_ring_irq_inten_set_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_INTEN_SET,
	OGMA_REG_ADDR_NRM_RX_INTEN_SET,
	0,
	0
};

const ogma_uint32 desc_ring_irq_inten_clr_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_INTEN_CLR,
	OGMA_REG_ADDR_NRM_RX_INTEN_CLR,
	0,
	0
};

static const ogma_uint32 int_tmr_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_TXINT_TMR,
	OGMA_REG_ADDR_NRM_RX_RXINT_TMR,
	0,
	0
};

static const ogma_uint32 rx_pkt_cnt_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	0,
	OGMA_REG_ADDR_NRM_RX_PKTCNT,
	0,
	0
};

static const ogma_uint32 tx_pkt_cnt_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_PKTCNT,
	0,
	0,
	0
};

static const ogma_uint32 int_pkt_cnt_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT,
	OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT,
	0,
	0
};

static const ogma_uint32 tx_done_pkt_cnt_reg_addr[OGMA_DESC_RING_ID_MAX + 1] = {
	OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT,
	0,
	0,
	0
};

STATIC void ogma_set_tx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   const ogma_tx_pkt_ctrl_t * tx_pkt_ctrl_p,
				   ogma_bool first_flag,
				   ogma_bool last_flag,
				   const ogma_frag_info_t * frag_info_p,
				   pfdep_pkt_handle_t pkt_handle);

STATIC void ogma_set_rx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   const ogma_frag_info_t * frag_info_p,
				   pfdep_pkt_handle_t pkt_handle);

STATIC void ogma_get_rx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   ogma_rx_pkt_info_t * rx_pkt_info_p,
				   ogma_frag_info_t * frag_info_p,
				   ogma_uint16 * len_p,
				   pfdep_pkt_handle_t * pkt_handle_p);

STATIC void ogma_inc_desc_head_idx(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 increment);

STATIC void ogma_inc_desc_tail_idx(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 increment);

STATIC ogma_uint16 ogma_get_tx_avail_num_sub(ogma_ctrl_t * ctrl_p,
					     const ogma_desc_ring_t *
					     desc_ring_p);

STATIC ogma_uint16 ogma_get_tx_done_num_sub(ogma_ctrl_t * ctrl_p,
					    ogma_desc_ring_t * desc_ring_p);

int netsec_get_pkt_tx_free(void *netsec_handle);
int netsec_put_pkt_tx(void *netsec_handle, u32 * desc, void *info);
int netsec_clean_pkt_tx(void *netsec_handle);
int netsec_get_pkt_rx(void *netsec_handle, u32 * desc, void **info);
int netsec_set_pkt_rx(void *netsec_handle, u32 * desc, void *info);

/**********************************************************************
 * Function definitions
 **********************************************************************/

ogma_err_t ogma_alloc_desc_ring(ogma_ctrl_t * ctrl_p,
				ogma_desc_ring_id_t ring_id)
{

	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_desc_ring_t *desc_ring_p = &ctrl_p->desc_ring[ring_id];
	ogma_desc_ring_param_t *desc_ring_param_p =
	    &ctrl_p->desc_ring[ring_id].param;
	pfdep_err_t pfdep_err;

	ogma_uint8 rx_desc_entry_len = 0;

	if ((ctrl_p->param.desc_ring_param[ring_id].valid_flag) &&
	    ((ctrl_p->param.desc_ring_param[ring_id].entry_num <
	      OGMA_DESC_ENTRY_NUM_MIN) ||
	     (ctrl_p->param.desc_ring_param[ring_id].entry_num >
	      OGMA_DESC_ENTRY_NUM_MAX))) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Please set entry_num between %d and %d.\n",
			    OGMA_DESC_ENTRY_NUM_MIN, OGMA_DESC_ENTRY_NUM_MAX);
		return OGMA_ERR_PARAM;
	}

	desc_ring_p->ring_id = ring_id;

	pfdep_memcpy(desc_ring_param_p,
		     &ctrl_p->param.desc_ring_param[ring_id],
		     sizeof(ogma_desc_ring_param_t));

	if (!desc_ring_param_p->valid_flag) {

		desc_ring_p->desc_ring_phys_addr =
		    ctrl_p->dummy_desc_entry_phys_addr;

		return OGMA_ERR_OK;
	}

	if ((ring_id == OGMA_DESC_RING_ID_NRM_RX) ||
	    (ring_id == OGMA_DESC_RING_ID_RESERVED_RX)) {
		rx_desc_entry_len = 16U;
	}

	switch (ring_id) {
	case OGMA_DESC_RING_ID_NRM_TX:
		desc_ring_p->tx_desc_ring_flag = OGMA_TRUE;
		desc_ring_p->desc_entry_len = sizeof(ogma_tx_desc_entry_t);
		break;

	case OGMA_DESC_RING_ID_NRM_RX:
		desc_ring_p->rx_desc_ring_flag = OGMA_TRUE;
		desc_ring_p->desc_entry_len = rx_desc_entry_len;
		break;

	case OGMA_DESC_RING_ID_RESERVED_RX:
		desc_ring_p->rx_desc_ring_flag = OGMA_TRUE;
		desc_ring_p->desc_entry_len = rx_desc_entry_len;
		break;

	case OGMA_DESC_RING_ID_RESERVED_TX:
		desc_ring_p->tx_desc_ring_flag = OGMA_TRUE;
		desc_ring_p->desc_entry_len = sizeof(ogma_tx_desc_entry_t);
		break;

	default:
		pfdep_assert(0);
	}

	if ((pfdep_err =
	     pfdep_init_hard_lock(&desc_ring_p->inten_reg_hard_lock))
	    != PFDEP_ERR_OK) {
		pfdep_memset(desc_ring_param_p,
			     0, sizeof(ogma_desc_ring_param_t));
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Failed to inten_reg_hard_lock's initialization.\n");
		return OGMA_ERR_ALLOC;
	}

	if ((pfdep_err = pfdep_init_soft_lock(&desc_ring_p->soft_lock))
	    != PFDEP_ERR_OK) {
		pfdep_uninit_hard_lock(&desc_ring_p->inten_reg_hard_lock);
		pfdep_memset(desc_ring_param_p,
			     0, sizeof(ogma_desc_ring_param_t));
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Failed to soft_lock's initialization.\n");
		return OGMA_ERR_ALLOC;
	}

	if ((pfdep_err =
	     pfdep_dma_malloc((pfdep_dev_handle_t) ctrl_p->dev_handle,
			      (pfdep_uint32) desc_ring_p->desc_entry_len *
			      desc_ring_param_p->entry_num,
			      (void **)&desc_ring_p->desc_ring_cpu_addr,
			      (pfdep_phys_addr_t *) &
			      desc_ring_p->desc_ring_phys_addr))
	    != PFDEP_ERR_OK) {
		ogma_err = OGMA_ERR_ALLOC;
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Failed to desc_ring entry memory allocation.\n");
		goto err;
	}

	pfdep_memset(desc_ring_p->desc_ring_cpu_addr,
		     0,
		     (ogma_uint32) desc_ring_p->desc_entry_len *
		     desc_ring_param_p->entry_num);

	if ((desc_ring_p->frag_info_p =
	     pfdep_malloc(sizeof(ogma_frag_info_t) *
			  desc_ring_param_p->entry_num))
	    == NULL) {
		ogma_err = OGMA_ERR_ALLOC;
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Failed to fragment infomation memory allocation.\n");
		goto err;
	}

	pfdep_memset(desc_ring_p->frag_info_p,
		     0,
		     sizeof(ogma_frag_info_t) * desc_ring_param_p->entry_num);

	if ((desc_ring_p->priv_data_p =
	     pfdep_malloc(sizeof(ogma_desc_entry_priv_t) *
			  desc_ring_param_p->entry_num))
	    == NULL) {
		ogma_err = OGMA_ERR_ALLOC;
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_alloc_desc_ring.\n"
			    "Failed to private data memory allocation.\n");
		goto err;
	}

	pfdep_memset(desc_ring_p->priv_data_p,
		     0,
		     sizeof(ogma_desc_entry_priv_t) *
		     desc_ring_param_p->entry_num);

	return OGMA_ERR_OK;

 err:
	ogma_free_desc_ring(ctrl_p, desc_ring_p);
	return ogma_err;
}

void ogma_free_desc_ring(ogma_ctrl_t * ctrl_p, ogma_desc_ring_t * desc_ring_p)
{

	if (!desc_ring_p->param.valid_flag) {
		return;
	}
	if (ogma_is_pkt_desc_ring(desc_ring_p)) {
		if ((desc_ring_p->desc_ring_cpu_addr != NULL) &&
		    (desc_ring_p->frag_info_p != NULL) &&
		    (desc_ring_p->priv_data_p != NULL)) {
			ogma_uninit_pkt_desc_ring(ctrl_p, desc_ring_p);
		}
	}
	if (desc_ring_p->desc_ring_cpu_addr != NULL) {
		pfdep_dma_free(ctrl_p->dev_handle,
			       (ogma_uint32) desc_ring_p->desc_entry_len *
			       desc_ring_p->param.entry_num,
			       desc_ring_p->desc_ring_cpu_addr,
			       desc_ring_p->desc_ring_phys_addr);
	}

	if (desc_ring_p->frag_info_p != NULL) {
		pfdep_free(desc_ring_p->frag_info_p);
	}

	if (desc_ring_p->priv_data_p != NULL) {
		pfdep_free(desc_ring_p->priv_data_p);
	}

	pfdep_uninit_hard_lock(&desc_ring_p->inten_reg_hard_lock);

	pfdep_uninit_soft_lock(&desc_ring_p->soft_lock);

	pfdep_memset(desc_ring_p, 0, sizeof(ogma_desc_ring_t));
}

ogma_err_t ogma_setup_rx_desc_ring(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p)
{
	ogma_uint16 idx;
	ogma_frag_info_t frag_info = { 0, 0, 0 };

	pfdep_err_t pfdep_err;
	pfdep_pkt_handle_t tmp_pkt_handle;

	frag_info.len = ctrl_p->rx_pkt_buf_len;

	for (idx = 0; idx < desc_ring_p->param.entry_num; idx++) {
		if ((pfdep_err = pfdep_alloc_pkt_buf((pfdep_dev_handle_t)
						     ctrl_p->dev_handle,
						     (ogma_uint16)
						     frag_info.len,
						     (void **)&frag_info.addr,
						     (pfdep_phys_addr_t *) &
						     frag_info.phys_addr,
						     (pfdep_pkt_handle_t *) &
						     tmp_pkt_handle))
		    != PFDEP_ERR_OK) {
			ogma_uninit_pkt_desc_ring(ctrl_p, desc_ring_p);
			pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
				    "An error occurred at ogma_setup_rx_desc_ring.\n"
				    "Failed to rx packet memory allocation.\n");
			return OGMA_ERR_ALLOC;
		}
		ogma_set_rx_desc_entry(ctrl_p,
				       desc_ring_p,
				       idx, &frag_info, tmp_pkt_handle);
	}
	return OGMA_ERR_OK;
}

void ogma_uninit_pkt_desc_ring(ogma_ctrl_t * ctrl_p,
			       ogma_desc_ring_t * desc_ring_p)
{
	ogma_uint16 idx;
	ogma_uint32 tmp;
	ogma_bool last_flag;

	for (idx = 0; idx < desc_ring_p->param.entry_num; idx++) {
		if (desc_ring_p->frag_info_p[idx].addr == NULL) {
			continue;
		}
		tmp = *((ogma_uint32 *)
			((pfdep_cpu_addr_t) desc_ring_p->desc_ring_cpu_addr +
			 (pfdep_cpu_addr_t) desc_ring_p->desc_entry_len * idx));

		last_flag = (((tmp >> 8) & 0x1) != 0);

		pfdep_free_pkt_buf(ctrl_p->dev_handle,
				   desc_ring_p->frag_info_p[idx].len,
				   desc_ring_p->frag_info_p[idx].addr,
				   desc_ring_p->frag_info_p[idx].phys_addr,
				   last_flag,
				   desc_ring_p->priv_data_p[idx].pkt_handle);
	}

	/* clear frag_info_p */
	pfdep_memset(desc_ring_p->frag_info_p,
		     0,
		     sizeof(ogma_frag_info_t) * desc_ring_p->param.entry_num);

	/* clear pkt_handle_p */
	pfdep_memset(desc_ring_p->priv_data_p,
		     0,
		     sizeof(ogma_desc_entry_priv_t) *
		     desc_ring_p->param.entry_num);

	/* clear desc ring entry */
	pfdep_memset(desc_ring_p->desc_ring_cpu_addr,
		     0,
		     (ogma_uint32) desc_ring_p->desc_entry_len *
		     desc_ring_p->param.entry_num);
}

STATIC void ogma_set_tx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   const ogma_tx_pkt_ctrl_t * tx_pkt_ctrl_p,
				   ogma_bool first_flag,
				   ogma_bool last_flag,
				   const ogma_frag_info_t * frag_info_p,
				   pfdep_pkt_handle_t pkt_handle)
{
	ogma_tx_desc_entry_t tx_desc_entry;
	ogma_uint32 attr, i, *debug_desc_entry_p;

	ogma_check_clk_supply(ctrl_p, 0);

	ogma_check_desc_own_sanity(ctrl_p, desc_ring_p, idx, 0);

	pfdep_memset(&tx_desc_entry, 0, sizeof(ogma_tx_desc_entry_t));

	attr = (1UL << OGMA_TX_PKT_DESC_RING_OWN_FIELD) |
	    ((idx == (desc_ring_p->param.entry_num - 1)) <<
	     OGMA_TX_PKT_DESC_RING_LD_FIELD) |
	    (desc_ring_p->ring_id << OGMA_TX_PKT_DESC_RING_DRID_FIELD) |
	    /* ( 1U << OGMA_TX_PKT_DESC_RING_PT_FIELD) | */
	    (tx_pkt_ctrl_p->target_desc_ring_id <<
	     OGMA_TX_PKT_DESC_RING_TDRID_FIELD) |
	    (first_flag << OGMA_TX_PKT_DESC_RING_FS_FIELD) |
	    (last_flag << OGMA_TX_PKT_DESC_RING_LS_FIELD) |
	    (tx_pkt_ctrl_p->cksum_offload_flag <<
	     OGMA_TX_PKT_DESC_RING_CO_FIELD) |
	    (tx_pkt_ctrl_p->tcp_seg_offload_flag <<
	     OGMA_TX_PKT_DESC_RING_SO_FIELD) |
	    (1U << OGMA_TX_PKT_DESC_RING_TRS_FIELD);

	tx_desc_entry.attr = attr;

	tx_desc_entry.data_buf_addr = frag_info_p->phys_addr;

	tx_desc_entry.buf_len_info =
	    (tx_pkt_ctrl_p->tcp_seg_len << 16) | frag_info_p->len;

	pfdep_memcpy(((void *)((pfdep_cpu_addr_t)
			       desc_ring_p->desc_ring_cpu_addr +
			       desc_ring_p->desc_entry_len * idx)),
		     (void *)&tx_desc_entry, desc_ring_p->desc_entry_len);
	netsec_put_pkt_tx(ctrl_p->netsec_handle, &tx_desc_entry.attr,
			  (void *)((u32) idx));

	debug_desc_entry_p = (ogma_uint32 *) & tx_desc_entry;

	for (i = 0; i < (sizeof(ogma_tx_desc_entry_t) >> 2); i++) {
		pfdep_print(PFDEP_DEBUG_LEVEL_DEBUG_MORE_DETAILED,
			    "%08x\n", debug_desc_entry_p[i]);
	}

	desc_ring_p->frag_info_p[idx].phys_addr = frag_info_p->phys_addr;
	desc_ring_p->frag_info_p[idx].addr = frag_info_p->addr;
	desc_ring_p->frag_info_p[idx].len = frag_info_p->len;
	desc_ring_p->priv_data_p[idx].pkt_handle = pkt_handle;

}

STATIC void ogma_set_rx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   const ogma_frag_info_t * frag_info_p,
				   pfdep_pkt_handle_t pkt_handle)
{
	ogma_rx_desc_entry_t rx_desc_entry;

	ogma_check_desc_own_sanity(ctrl_p, desc_ring_p, idx, 0);

	pfdep_memset(&rx_desc_entry, 0, sizeof(ogma_rx_desc_entry_t));

	rx_desc_entry.attr = (1UL << OGMA_RX_PKT_DESC_RING_OWN_FIELD) | (1UL << OGMA_RX_PKT_DESC_RING_FS_FIELD) | (1UL << OGMA_RX_PKT_DESC_RING_LS_FIELD);	/* OWN = FS = LS = 1 */

	rx_desc_entry.data_buf_addr = frag_info_p->phys_addr;

	rx_desc_entry.buf_len_info = frag_info_p->len;

	if (idx == (desc_ring_p->param.entry_num - 1)) {
		rx_desc_entry.attr |= (0x1U << OGMA_RX_PKT_DESC_RING_LD_FIELD);	/* LD = 1 */
	}

	pfdep_memcpy(((void *)((pfdep_cpu_addr_t)
			       desc_ring_p->desc_ring_cpu_addr +
			       desc_ring_p->desc_entry_len * idx + 4)),
		     (void *)((pfdep_cpu_addr_t) & rx_desc_entry + 4),
		     (ogma_uint32) (desc_ring_p->desc_entry_len - 4U));

	pfdep_write_mem_barrier();

	pfdep_memcpy(((void *)((pfdep_cpu_addr_t)
			       desc_ring_p->desc_ring_cpu_addr +
			       desc_ring_p->desc_entry_len * idx)),
		     (void *)&rx_desc_entry, 4);
	netsec_set_pkt_rx(ctrl_p->netsec_handle, &rx_desc_entry.attr,
			  (void *)((u32) idx));

	desc_ring_p->frag_info_p[idx].phys_addr = frag_info_p->phys_addr;
	desc_ring_p->frag_info_p[idx].addr = frag_info_p->addr;
	desc_ring_p->frag_info_p[idx].len = frag_info_p->len;
	desc_ring_p->priv_data_p[idx].pkt_handle = pkt_handle;
}

/* OGMA_CONFIG_USE_PKT_DESC_RING*/

STATIC void ogma_get_rx_desc_entry(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 idx,
				   ogma_rx_pkt_info_t * rx_pkt_info_p,
				   ogma_frag_info_t * frag_info_p,
				   ogma_uint16 * len_p,
				   pfdep_pkt_handle_t * pkt_handle_p)
{
	ogma_uint32 *debug_desc_entry_p;
	ogma_rx_desc_entry_t rx_desc_entry;
	ogma_rx_desc_entry_t *rx_desc_entry_p;
	void *info;
	int ret;

	ogma_check_clk_supply(ctrl_p, 0);

	pfdep_memset(&rx_desc_entry, 0, sizeof(ogma_rx_desc_entry_t));
	pfdep_memset(rx_pkt_info_p, 0, sizeof(ogma_rx_pkt_info_t));

	ret =
	    netsec_get_pkt_rx(ctrl_p->netsec_handle, &rx_desc_entry.attr,
			      &info);
	if (ret <= 0) {
		printk(KERN_ERR "%s netsec_get_pkt_rx %d\n", __FUNCTION__, ret);
	}
	rx_desc_entry_p =
	    (void *)((pfdep_cpu_addr_t) desc_ring_p->desc_ring_cpu_addr +
		     desc_ring_p->desc_entry_len * idx), rx_desc_entry_p->attr =
	    rx_desc_entry.attr;

	debug_desc_entry_p = (ogma_uint32 *) & rx_desc_entry;

	pfdep_print(PFDEP_DEBUG_LEVEL_DEBUG_MORE_DETAILED,
		    "%08x\n", *debug_desc_entry_p);

	*len_p = rx_desc_entry.buf_len_info >> 16;

	rx_pkt_info_p->fragmented_flag = (rx_desc_entry.attr >> OGMA_RX_PKT_DESC_RING_FR_FIELD) & 0x1;	/* FR */

	rx_pkt_info_p->err_flag = (rx_desc_entry.attr >> OGMA_RX_PKT_DESC_RING_ER_FIELD) & 0x1;	/* ER */

	rx_pkt_info_p->rx_cksum_result = (rx_desc_entry.attr >> OGMA_RX_PKT_DESC_RING_CO_FIELD) & 0x3;	/* CO */

	rx_pkt_info_p->err_code = (rx_desc_entry.attr >> OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD) & OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD_MASK;	/* Error Code */

	pfdep_memcpy(frag_info_p,
		     &desc_ring_p->frag_info_p[idx], sizeof(ogma_frag_info_t));

	*pkt_handle_p = desc_ring_p->priv_data_p[idx].pkt_handle;
}

/* OGMA_CONFIG_USE_PKT_DESC_RING*/

STATIC void ogma_inc_desc_head_idx(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 increment)
{
	ogma_uint32 sum;

	ogma_check_clk_supply(ctrl_p, 0);

	if ((desc_ring_p->tail_idx > desc_ring_p->head_idx) ||
	    desc_ring_p->full_flag) {
		pfdep_assert(increment <=
			     (desc_ring_p->tail_idx - desc_ring_p->head_idx));
	} else {
		pfdep_assert(increment <=
			     (desc_ring_p->param.entry_num +
			      desc_ring_p->tail_idx - desc_ring_p->head_idx));
	}

	sum = (ogma_uint32) desc_ring_p->head_idx + increment;

	if (sum >= desc_ring_p->param.entry_num) {
		sum -= desc_ring_p->param.entry_num;
	}

	desc_ring_p->head_idx = (ogma_uint16) sum;

	if (desc_ring_p->head_idx == desc_ring_p->tail_idx) {
		desc_ring_p->full_flag = OGMA_TRUE;
	}

}

STATIC void ogma_inc_desc_tail_idx(ogma_ctrl_t * ctrl_p,
				   ogma_desc_ring_t * desc_ring_p,
				   ogma_uint16 increment)
{
	ogma_uint32 sum;

	ogma_check_clk_supply(ctrl_p, 0);

	if ((desc_ring_p->head_idx >= desc_ring_p->tail_idx) &&
	    (!desc_ring_p->full_flag)) {
		pfdep_assert(increment <=
			     (desc_ring_p->head_idx - desc_ring_p->tail_idx));
	} else {
		pfdep_assert(increment <=
			     (desc_ring_p->param.entry_num +
			      desc_ring_p->head_idx - desc_ring_p->tail_idx));
	}

	sum = (ogma_uint32) desc_ring_p->tail_idx + increment;

	if (sum >= desc_ring_p->param.entry_num) {
		sum -= desc_ring_p->param.entry_num;
	}

	desc_ring_p->tail_idx = (ogma_uint16) sum;

	desc_ring_p->full_flag = OGMA_FALSE;

}

STATIC ogma_uint16 ogma_get_tx_avail_num_sub(ogma_ctrl_t * ctrl_p,
					     const ogma_desc_ring_t *
					     desc_ring_p)
{
	ogma_uint16 tx_avail_num;

	ogma_check_clk_supply(ctrl_p, 0);

	tx_avail_num = netsec_get_pkt_tx_free(ctrl_p->netsec_handle);

	return tx_avail_num;
}

STATIC ogma_uint16 ogma_get_tx_done_num_sub(ogma_ctrl_t * ctrl_p,
					    ogma_desc_ring_t * desc_ring_p)
{
	ogma_uint32 value;

	ogma_check_clk_supply(ctrl_p, OGMA_CLK_EN_REG_DOM_D);

	value = ogma_read_reg(ctrl_p,
			      tx_done_pkt_cnt_reg_addr[desc_ring_p->ring_id]);

	desc_ring_p->tx_done_num += value;

	return desc_ring_p->tx_done_num;
}

ogma_err_t ogma_start_desc_ring(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id)
{
	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_uint32 value, domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;

	pfdep_soft_lock_ctx_t soft_lock_ctx;
	pfdep_err_t pfdep_err;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	if (desc_ring_p->running_flag) {
		pfdep_release_soft_lock(&desc_ring_p->soft_lock,
					&soft_lock_ctx);
		return OGMA_ERR_BUSY;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	if (desc_ring_p->rx_desc_ring_flag) {

		ogma_write_reg(ctrl_p,
			       desc_ring_irq_inten_set_reg_addr[ring_id],
			       OGMA_CH_IRQ_REG_RCV);

		ogma_write_reg(ctrl_p, int_pkt_cnt_reg_addr[ring_id], 1);
	}

	if (desc_ring_p->tx_desc_ring_flag) {

		value = OGMA_CH_IRQ_REG_EMPTY;

		ogma_write_reg(ctrl_p,
			       desc_ring_irq_inten_set_reg_addr[ring_id],
			       value);

		ogma_write_reg(ctrl_p, int_pkt_cnt_reg_addr[ring_id], 1);

	}

	desc_ring_p->running_flag = OGMA_TRUE;

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);

	return ogma_err;
}

ogma_err_t ogma_stop_desc_ring(ogma_handle_t ogma_handle,
			       ogma_desc_ring_id_t ring_id)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;
	pfdep_err_t pfdep_err;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if ((ctrl_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (!ogma_is_pkt_desc_ring(&ctrl_p->desc_ring[ring_id])) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	if (!desc_ring_p->running_flag) {
		pfdep_release_soft_lock(&desc_ring_p->soft_lock,
					&soft_lock_ctx);
		return OGMA_ERR_INVALID;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	value = (OGMA_CH_IRQ_REG_RCV |
		 OGMA_CH_IRQ_REG_EMPTY | OGMA_CH_IRQ_REG_SND);

	ogma_write_reg(ctrl_p,
		       desc_ring_irq_inten_clr_reg_addr[ring_id], value);

	/* pop domain d clock */
	ogma_pop_clk_req(ctrl_p, domain);

	desc_ring_p->running_flag = OGMA_FALSE;

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);

	return OGMA_ERR_OK;
}

ogma_uint16 ogma_get_rx_num(ogma_handle_t ogma_handle,
			    ogma_desc_ring_id_t ring_id)
{

	ogma_uint32 result, domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p = NULL;
	ogma_desc_ring_id_t tmp_ring_id;

	pfdep_soft_lock_ctx_t soft_lock_ctx;
	pfdep_err_t pfdep_err;

	if ((ctrl_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_rx_num.\n"
			    "Please set valid argument.\n");
		return 0;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_rx_num.\n"
			    "Please set valid argument.\n");
		return 0;
	}

	tmp_ring_id = ring_id;

	if (!ctrl_p->desc_ring[tmp_ring_id].rx_desc_ring_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_rx_num.\n"
			    "Please select rx packet desc ring.\n");
		return 0;
	}

	desc_ring_p = &ctrl_p->desc_ring[tmp_ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {

		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_rx_num.\n"
			    "Failed to get soft lock.\n");
		return 0;
	}

	if (!desc_ring_p->running_flag) {
		pfdep_release_soft_lock(&desc_ring_p->soft_lock,
					&soft_lock_ctx);

		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_rx_num.\n"
			    "Please select running desc ring.\n");
		return 0;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	result = ogma_read_reg(ctrl_p, rx_pkt_cnt_reg_addr[tmp_ring_id]);

	desc_ring_p->rx_num += result;

	if (desc_ring_p->rx_desc_ring_flag && (result != 0)) {
		ogma_inc_desc_head_idx(ctrl_p, desc_ring_p,
				       (ogma_uint16) result);
	}

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);

	return desc_ring_p->rx_num;

}

ogma_uint16 ogma_get_tx_avail_num(ogma_handle_t ogma_handle,
				  ogma_desc_ring_id_t ring_id)
{

	ogma_uint16 result;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_id_t tmp_ring_id;
	ogma_desc_ring_t *desc_ring_p = NULL;

	pfdep_soft_lock_ctx_t soft_lock_ctx;
	pfdep_err_t pfdep_err;

	if ((ctrl_p == NULL) ||
	    (ring_id > OGMA_DESC_RING_ID_MAX) ||
	    (!ctrl_p->desc_ring[ring_id].param.valid_flag)) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_tx_avail_num.\n"
			    "Please select valid argument.\n");
		return 0;
	}

	tmp_ring_id = ring_id;

	if (!ctrl_p->desc_ring[tmp_ring_id].tx_desc_ring_flag) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_tx_avail_num.\n"
			    "Please select tx packet desc ring.\n");
		return 0;
	}

	desc_ring_p = &ctrl_p->desc_ring[tmp_ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_tx_avail_num.\n"
			    "Failed to get soft lock.\n");
		return 0;
	}

	if (!desc_ring_p->running_flag) {
		pfdep_release_soft_lock(&desc_ring_p->soft_lock,
					&soft_lock_ctx);
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_get_tx_avail_num.\n"
			    "Please select running desc ring.\n");
		return 0;
	}

	result = ogma_get_tx_avail_num_sub(ctrl_p, desc_ring_p);

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);
	return result;
}

ogma_err_t ogma_clean_tx_desc_ring(ogma_handle_t ogma_handle,
				   ogma_desc_ring_id_t ring_id)
{
	ogma_uint32 tmp;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;
	ogma_tx_desc_entry_t *tx_desc_entry_p;

	pfdep_err_t pfdep_err;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if ((ctrl_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ctrl_p->desc_ring[ring_id].tx_desc_ring_flag) {
		return OGMA_ERR_PARAM;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	ogma_get_tx_done_num_sub(ctrl_p, desc_ring_p);

	while (((desc_ring_p->tail_idx != desc_ring_p->head_idx) ||
		desc_ring_p->full_flag) && (desc_ring_p->tx_done_num != 0)) {
		tx_desc_entry_p = (ogma_tx_desc_entry_t *)
		    ((pfdep_cpu_addr_t) desc_ring_p->desc_ring_cpu_addr +
		     desc_ring_p->desc_entry_len * desc_ring_p->tail_idx);

		tmp = tx_desc_entry_p->attr;
		tx_desc_entry_p->attr = 0;	/* clear desc0 */

		netsec_clean_pkt_tx(ctrl_p->netsec_handle);

		pfdep_free_pkt_buf(ctrl_p->dev_handle,
				   desc_ring_p->
				   frag_info_p[desc_ring_p->tail_idx].len,
				   desc_ring_p->
				   frag_info_p[desc_ring_p->tail_idx].addr,
				   desc_ring_p->
				   frag_info_p[desc_ring_p->tail_idx].phys_addr,
				   (((tmp >> OGMA_TX_PKT_DESC_RING_LS_FIELD) &
				     0x1) != 0),
				   desc_ring_p->
				   priv_data_p
				   [desc_ring_p->tail_idx].pkt_handle);

		pfdep_memset(&desc_ring_p->frag_info_p[desc_ring_p->tail_idx],
			     0, sizeof(ogma_frag_info_t));

		ogma_inc_desc_tail_idx(ctrl_p, desc_ring_p, 1);

		if ((tmp & (1UL << OGMA_TX_PKT_DESC_RING_LS_FIELD)) != 0) {

			pfdep_assert(desc_ring_p->tx_done_num != 0);

			desc_ring_p->tx_done_num--;

		}

	}

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);
	return OGMA_ERR_OK;

}

ogma_err_t ogma_clean_rx_desc_ring(ogma_handle_t ogma_handle,
				   ogma_desc_ring_id_t ring_id)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;

	pfdep_err_t pfdep_err;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	if ((ctrl_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ctrl_p->desc_ring[ring_id].rx_desc_ring_flag) {
		return OGMA_ERR_PARAM;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	while (desc_ring_p->full_flag ||
	       (desc_ring_p->tail_idx != desc_ring_p->head_idx)) {

		ogma_set_rx_desc_entry(ctrl_p,
				       desc_ring_p,
				       desc_ring_p->tail_idx,
				       &desc_ring_p->
				       frag_info_p[desc_ring_p->tail_idx],
				       desc_ring_p->
				       priv_data_p
				       [desc_ring_p->tail_idx].pkt_handle);

		--desc_ring_p->rx_num;
		ogma_inc_desc_tail_idx(ctrl_p, desc_ring_p, 1);
	}

	pfdep_assert(desc_ring_p->rx_num == 0);	/* error check */

	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);
	return OGMA_ERR_OK;

}

ogma_err_t ogma_set_tx_pkt_data(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id,
				const ogma_tx_pkt_ctrl_t * tx_pkt_ctrl_p,
				ogma_uint8 scat_num,
				const ogma_frag_info_t * scat_info_p,
				pfdep_pkt_handle_t pkt_handle)
{
	ogma_uint i;
	ogma_uint16 tx_avail_num;
	ogma_uint32 sum_len = 0;
	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;

	pfdep_err_t pfdep_err;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;

	if ((ctrl_p == NULL) ||
	    (tx_pkt_ctrl_p == NULL) ||
	    (scat_info_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ctrl_p->desc_ring[ring_id].tx_desc_ring_flag) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->param.use_gmac_flag ||
	    (tx_pkt_ctrl_p->target_desc_ring_id != OGMA_DESC_RING_ID_GMAC)) {
		return OGMA_ERR_DATA;
	}

	if (tx_pkt_ctrl_p->tcp_seg_offload_flag &&
	    (!tx_pkt_ctrl_p->cksum_offload_flag)) {
		return OGMA_ERR_DATA;
	}

	if (tx_pkt_ctrl_p->tcp_seg_offload_flag) {

		if (tx_pkt_ctrl_p->tcp_seg_len < OGMA_TCP_SEG_LEN_MIN) {
			return OGMA_ERR_DATA;
		}

		if (ctrl_p->param.use_jumbo_pkt_flag) {
			if (tx_pkt_ctrl_p->tcp_seg_len >
			    OGMA_TCP_JUMBO_SEG_LEN_MAX) {
				return OGMA_ERR_DATA;
			}
		} else {
			if (tx_pkt_ctrl_p->tcp_seg_len > OGMA_TCP_SEG_LEN_MAX) {
				return OGMA_ERR_DATA;
			}
		}

	} else {
		if (tx_pkt_ctrl_p->tcp_seg_len != 0) {
			return OGMA_ERR_DATA;
		}
	}

	if (scat_num == 0) {
		return OGMA_ERR_RANGE;
	}

	for (i = 0; i < scat_num; i++) {
		if ((scat_info_p[i].len == 0) || (scat_info_p[i].len > 0xffffU)) {
			pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
				    "An error occurred at ogma_set_tx_pkt_data.\n"
				    "Pleas check scat_info_p[%u].len.\n", i);
			return OGMA_ERR_DATA;
		}
		sum_len += scat_info_p[i].len;
	}

	if (!tx_pkt_ctrl_p->tcp_seg_offload_flag) {

		if (ctrl_p->param.use_jumbo_pkt_flag) {
			if (sum_len > OGMA_MAX_TX_JUMBO_PKT_LEN) {
				return OGMA_ERR_DATA;
			}
		} else {
			if (sum_len > OGMA_MAX_TX_PKT_LEN) {
				return OGMA_ERR_DATA;
			}
		}

	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	if (!desc_ring_p->running_flag) {
		ogma_err = OGMA_ERR_NOTAVAIL;
		goto end;
	}

	tx_avail_num = ogma_get_tx_avail_num_sub(ctrl_p, desc_ring_p);

	if (scat_num > tx_avail_num) {
		ogma_err = OGMA_ERR_BUSY;
		goto end;
	}

	/*push clock */
	ogma_push_clk_req(ctrl_p, domain);

	for (i = 0; i < scat_num; i++) {
		ogma_set_tx_desc_entry(ctrl_p,
				       desc_ring_p,
				       desc_ring_p->head_idx,
				       tx_pkt_ctrl_p,
				       (i == 0),
				       (i == (scat_num - 1U)),
				       &scat_info_p[i], pkt_handle);
		ogma_inc_desc_head_idx(ctrl_p, desc_ring_p, 1);
	}

	pfdep_write_mem_barrier();

	ogma_write_reg(ctrl_p, tx_pkt_cnt_reg_addr[ring_id], (ogma_uint32) 1);

	/* pop domain d clock */
	ogma_pop_clk_req(ctrl_p, domain);

 end:
	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);

	return ogma_err;
}

ogma_err_t ogma_get_rx_pkt_data(ogma_handle_t ogma_handle,
				ogma_desc_ring_id_t ring_id,
				ogma_rx_pkt_info_t * rx_pkt_info_p,
				ogma_frag_info_t * frag_info_p,
				ogma_uint16 * len_p,
				pfdep_pkt_handle_t * pkt_handle_p)
{

	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_desc_ring_t *desc_ring_p;
	ogma_frag_info_t tmp_frag_info;

	pfdep_err_t pfdep_err;
	pfdep_pkt_handle_t tmp_pkt_handle;
	pfdep_soft_lock_ctx_t soft_lock_ctx;

	if ((ctrl_p == NULL) ||
	    (rx_pkt_info_p == NULL) ||
	    (frag_info_p == NULL) ||
	    (len_p == NULL) ||
	    (pkt_handle_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ctrl_p->desc_ring[ring_id].rx_desc_ring_flag) {
		return OGMA_ERR_PARAM;
	}

	desc_ring_p = &ctrl_p->desc_ring[ring_id];

	if ((pfdep_err = pfdep_acquire_soft_lock(&desc_ring_p->soft_lock,
						 &soft_lock_ctx)) !=
	    PFDEP_ERR_OK) {
		return OGMA_ERR_INTERRUPT;
	}

	if (!desc_ring_p->running_flag) {
		ogma_err = OGMA_ERR_NOTAVAIL;
		goto end;
	}

	if (desc_ring_p->rx_num == 0) {
		ogma_err = OGMA_ERR_INVALID;
		goto end;
	}

	tmp_frag_info.len = ctrl_p->rx_pkt_buf_len;

	pfdep_read_mem_barrier();

	if ((pfdep_err = pfdep_alloc_pkt_buf(ctrl_p->dev_handle,
					     tmp_frag_info.len,
					     &tmp_frag_info.addr,
					     &tmp_frag_info.phys_addr,
					     &tmp_pkt_handle)) !=
	    PFDEP_ERR_OK) {
		ogma_set_rx_desc_entry(ctrl_p, desc_ring_p,
				       desc_ring_p->tail_idx,
				       &desc_ring_p->
				       frag_info_p[desc_ring_p->tail_idx],
				       desc_ring_p->
				       priv_data_p
				       [desc_ring_p->tail_idx].pkt_handle);
		ogma_err = OGMA_ERR_ALLOC;

	} else {

		ogma_get_rx_desc_entry(ctrl_p,
				       desc_ring_p,
				       desc_ring_p->tail_idx,
				       rx_pkt_info_p,
				       frag_info_p, len_p, pkt_handle_p);

		ogma_set_rx_desc_entry(ctrl_p,
				       desc_ring_p,
				       desc_ring_p->tail_idx,
				       &tmp_frag_info, tmp_pkt_handle);
	}

	ogma_inc_desc_tail_idx(ctrl_p, desc_ring_p, 1);

	--desc_ring_p->rx_num;

 end:
	pfdep_release_soft_lock(&desc_ring_p->soft_lock, &soft_lock_ctx);

	return ogma_err;
}

ogma_err_t ogma_set_irq_coalesce_param(ogma_handle_t ogma_handle,
				       ogma_desc_ring_id_t ring_id,
				       ogma_uint16 int_pktcnt,
				       ogma_bool int_tmr_unit_ms_flag,
				       ogma_uint16 int_tmr_cnt)
{

	ogma_uint32 domain = OGMA_CLK_EN_REG_DOM_D;
	ogma_err_t ogma_err = OGMA_ERR_OK;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if ((ctrl_p == NULL) || (ring_id > OGMA_DESC_RING_ID_MAX)) {
		return OGMA_ERR_PARAM;
	}

	if (int_pktcnt > OGMA_INT_PKTCNT_MAX) {
		return OGMA_ERR_RANGE;
	}

	if (!ctrl_p->desc_ring[ring_id].param.valid_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ogma_is_pkt_desc_ring(&ctrl_p->desc_ring[ring_id])) {
		return OGMA_ERR_PARAM;
	}

	/* push clock */
	ogma_push_clk_req(ctrl_p, domain);

	ogma_write_reg(ctrl_p, int_pkt_cnt_reg_addr[ring_id], int_pktcnt);

	ogma_write_reg(ctrl_p,
		       int_tmr_reg_addr[ring_id],
		       ((((ogma_uint32) int_tmr_unit_ms_flag) << 31) |
			int_tmr_cnt));

	/* pop clock */
	ogma_pop_clk_req(ctrl_p, domain);

	return ogma_err;

}
