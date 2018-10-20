/**
 * pfdep_linux.c
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
 * Platform dependent API implementation for Linux.
 *
 */

#include "pfdep.h"

#include <linux/dma-mapping.h>

/**********************************************************************
 * Variable definitions
 **********************************************************************/
pfdep_debug_level_t pfdep_debug_level = PFDEP_DEBUG_LEVEL_NOTICE;

/**********************************************************************
 * Function definitions
 **********************************************************************/

pfdep_err_t pfdep_dma_malloc(pfdep_dev_handle_t dev_handle,
			     pfdep_uint32 len,
			     void **addr_p, pfdep_phys_addr_t * phys_addr_p)
{

	(void)dev_handle;	/* Suppress compiler warning */

	*addr_p = dma_alloc_coherent(NULL,
				     (size_t) len, phys_addr_p, GFP_KERNEL);

	if (*addr_p == NULL) {
		return PFDEP_ERR_ALLOC;
	}

	return PFDEP_ERR_OK;

}

void pfdep_dma_free(pfdep_dev_handle_t dev_handle,
		    pfdep_uint32 len, void *addr, pfdep_phys_addr_t phys_addr)
{

	(void)dev_handle;	/* Suppress compiler warning */

	dma_free_coherent(NULL, (size_t) len, addr, phys_addr);

}

pfdep_err_t pfdep_alloc_pkt_buf(pfdep_dev_handle_t dev_handle,
				pfdep_uint16 len,
				void **addr_p,
				pfdep_phys_addr_t * phys_addr_p,
				pfdep_pkt_handle_t * pkt_handle_p)
{

	struct sk_buff *skb_p;

	if ((skb_p = dev_alloc_skb((unsigned int)len + 2)) == NULL) {
		return PFDEP_ERR_ALLOC;
	}

	/* Reserve leading 2 bytes to make IP header word-aligned. */
	skb_reserve(skb_p, 2);

	/*
	 * Make packet buffer accessible to device DMA.
	 */
	if ((*phys_addr_p = dma_map_single(dev_handle,
					   skb_p->data,
					   (size_t) len, DMA_FROM_DEVICE))
	    == (dma_addr_t) 0) {
		dev_kfree_skb(skb_p);
		return PFDEP_ERR_ALLOC;
	}

	*addr_p = (void *)skb_p->data;
	*pkt_handle_p = skb_p;

	/* Mark that this is a receiving buffer. */
	pfdep_set_pkt_buf_type(skb_p, PFDEP_TRUE);

	return PFDEP_ERR_OK;

}

void pfdep_free_pkt_buf(pfdep_dev_handle_t dev_handle,
			pfdep_uint16 len,
			void *addr,
			pfdep_phys_addr_t phys_addr,
			pfdep_bool last_flag, pfdep_pkt_handle_t pkt_handle)
{

	/*
	 * Disarm platform-dependent memory trick.
	 */
	dma_unmap_single(dev_handle,
			 phys_addr,
			 (size_t) len,
			 (pfdep_is_recv_pkt_buf(pkt_handle) ?
			  DMA_FROM_DEVICE : DMA_TO_DEVICE));

	/*
	 * Free packet information if this is the last fragment.
	 */
	if (last_flag) {
		dev_kfree_skb(pkt_handle);
	}

}
