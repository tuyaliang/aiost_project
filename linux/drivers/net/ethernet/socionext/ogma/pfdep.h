/**
 * pfdep.h
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

#ifndef PFDEP_H
#define PFDEP_H

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#endif				/* __KERNEL__ */

/**********************************************************************
 * Constant definitions
 **********************************************************************/
#define PFDEP_INT64_AVAILABLE

/**********************************************************************
 * Elementary type definitions
 **********************************************************************/
typedef char pfdep_int8;
typedef unsigned char pfdep_uint8;
typedef short pfdep_int16;
typedef unsigned short pfdep_uint16;
typedef int pfdep_int32;
typedef unsigned int pfdep_uint32;
typedef long long pfdep_int64;
typedef unsigned long long pfdep_uint64;
typedef int pfdep_bool;
typedef char pfdep_char;

#define PFDEP_TRUE ((pfdep_bool)1)
#define PFDEP_FALSE ((pfdep_bool)0)

/**********************************************************************
 * Complex type definitions
 **********************************************************************/

typedef enum pfdep_err_e {
	PFDEP_ERR_OK = 0,
	PFDEP_ERR_PARAM,
	PFDEP_ERR_ALLOC,
	PFDEP_ERR_INTERRUPT
} pfdep_err_t;

#ifdef __KERNEL__

typedef struct device *pfdep_dev_handle_t;
typedef struct sk_buff *pfdep_pkt_handle_t;
typedef dma_addr_t pfdep_phys_addr_t;
typedef void (*pfdep_timer_func_t) (void *);
typedef struct timer_list *pfdep_timer_handle_t;

#else				/* __KERNEL__ */

typedef void *pfdep_dev_handle_t;
typedef void *pfdep_pkt_handle_t;
typedef void *pfdep_phys_addr_t;

#endif				/* __KERNEL__ */

#if __SIZEOF_POINTER__ == 4
typedef unsigned int pfdep_cpu_addr_t;
#elif __SIZEOF_POINTER__ == 8
typedef unsigned long long pfdep_cpu_addr_t;
#else				/*  __SIZEOF_POINTER__ == unknown */
#error "Unsupported __SIZEOF_POINTER__"
#endif				/* __SIZEOF_POINTER__ == 4 */

#ifdef __KERNEL__

typedef spinlock_t pfdep_hard_lock_t;
typedef spinlock_t pfdep_soft_lock_t;

typedef unsigned long pfdep_hard_lock_ctx_t;
typedef int pfdep_soft_lock_ctx_t;

typedef unsigned int pfdep_debug_level_t;

#define PFDEP_DEBUG_LEVEL_FATAL               ((pfdep_debug_level_t)1)
#define PFDEP_DEBUG_LEVEL_WARNING             ((pfdep_debug_level_t)2)
#define PFDEP_DEBUG_LEVEL_NOTICE              ((pfdep_debug_level_t)3)
#define PFDEP_DEBUG_LEVEL_DEBUG               ((pfdep_debug_level_t)4)
#define PFDEP_DEBUG_LEVEL_DEBUG_DETAILED      ((pfdep_debug_level_t)5)
#define PFDEP_DEBUG_LEVEL_DEBUG_MORE_DETAILED ((pfdep_debug_level_t)6)

#endif				/* __KERNEL__ */

/**********************************************************************
 * Variable declarations
 **********************************************************************/
#ifdef __KERNEL__
extern pfdep_debug_level_t pfdep_debug_level;	/* defined in pfdep_linux.c */
#endif				/* __KERNEL__ */

/**********************************************************************
 * Function declarations
 **********************************************************************/

#ifdef __KERNEL__

static __inline pfdep_uint32 pfdep_iomem_read(void *addr)
{
	return *((volatile pfdep_uint32 *)(addr));
}

static __inline void pfdep_iomem_write(void *addr, pfdep_uint32 val)
{
	*((volatile pfdep_uint32 *)(addr)) = val;

}

#define pfdep_read_mem_barrier() rmb()
#define pfdep_write_mem_barrier() wmb()
#define pfdep_mem_barrier() mb()

static __inline void *pfdep_malloc(pfdep_uint32 len)
{
	return kmalloc((size_t) len, GFP_NOWAIT);
}

static __inline void pfdep_free(void *addr)
{
	kfree(addr);
}

pfdep_err_t pfdep_dma_malloc(pfdep_dev_handle_t dev_handle,
			     pfdep_uint32 len,
			     void **addr_p, pfdep_phys_addr_t * phys_addr_p);

void pfdep_dma_free(pfdep_dev_handle_t dev_handle,
		    pfdep_uint32 len, void *addr, pfdep_phys_addr_t phys_addr);

pfdep_err_t pfdep_alloc_pkt_buf(pfdep_dev_handle_t dev_handle,
				pfdep_uint16 len,
				void **addr_p,
				pfdep_phys_addr_t * phys_addr_p,
				pfdep_pkt_handle_t * pkt_handle_p);

void pfdep_free_pkt_buf(pfdep_dev_handle_t dev_handle,
			pfdep_uint16 len,
			void *addr,
			pfdep_phys_addr_t phys_addr,
			pfdep_bool last_flag, pfdep_pkt_handle_t pkt_handle);

static __inline pfdep_err_t pfdep_init_hard_lock(pfdep_hard_lock_t *
						 hard_lock_p)
{
	spin_lock_init(hard_lock_p);

	return PFDEP_ERR_OK;
}

static __inline void pfdep_uninit_hard_lock(pfdep_hard_lock_t * hard_lock_p)
{
	return;
}

static __inline void pfdep_acquire_hard_lock(pfdep_hard_lock_t * hard_lock_p,
					     pfdep_hard_lock_ctx_t * ctx_p)
{
	spin_lock_irqsave(hard_lock_p, *ctx_p);
}

static __inline void pfdep_release_hard_lock(pfdep_hard_lock_t * hard_lock_p,
					     pfdep_hard_lock_ctx_t * ctx_p)
{
	spin_unlock_irqrestore(hard_lock_p, *ctx_p);
}

static __inline pfdep_err_t pfdep_init_soft_lock(pfdep_soft_lock_t *
						 soft_lock_p)
{
	spin_lock_init(soft_lock_p);

	return PFDEP_ERR_OK;
}

static __inline void pfdep_uninit_soft_lock(pfdep_soft_lock_t * soft_lock_p)
{
	return;
}

static __inline pfdep_err_t pfdep_acquire_soft_lock(pfdep_soft_lock_t *
						    soft_lock_p,
						    pfdep_soft_lock_ctx_t *
						    ctx_p)
{
	if (in_softirq()) {
		*ctx_p = 1;	/* Mark that bh is already disabled. */
		spin_lock(soft_lock_p);
	} else {
		*ctx_p = 0;
		spin_lock_bh(soft_lock_p);
	}

	return PFDEP_ERR_OK;
}

static __inline void pfdep_release_soft_lock(pfdep_soft_lock_t * soft_lock_p,
					     pfdep_soft_lock_ctx_t * ctx_p)
{

	if (*ctx_p == 1) {
		spin_unlock(soft_lock_p);
	} else {		/* *ctx_p == 0 */
		spin_unlock_bh(soft_lock_p);
	}

}

#define pfdep_l2h_32(val_le) le32_to_cpu(val_le)
#define pfdep_h2l_32(val)    cpu_to_le32(val)
#define pfdep_l2h_16(val_le) le16_to_cpu(val_le)
#define pfdep_h2l_16(val)    cpu_to_le16(val)
#define pfdep_b2h_32(val_be) be32_to_cpu(val_be)
#define pfdep_h2b_32(val)    cpu_to_be32(val)
#define pfdep_b2h_16(val_be) be16_to_cpu(val_be)
#define pfdep_h2b_16(val)    cpu_to_be16(val)

static __inline void pfdep_memcpy(void *dst_p, const void *src_p,
				  pfdep_uint32 len)
{
	memcpy(dst_p, src_p, (size_t) len);
}

static __inline void pfdep_memset(void *dst_p, pfdep_uint8 c, pfdep_uint32 len)
{
	memset(dst_p, c, (size_t) len);
}

static __inline pfdep_err_t pfdep_msleep(pfdep_uint32 wait_ms)
{

	msleep((unsigned int)wait_ms);

	return PFDEP_ERR_OK;
}

#define pfdep_print(level,...) \
do { \
    if (level <= pfdep_debug_level) { \
        printk(KERN_ERR "[F_TAIKI]" __VA_ARGS__); \
    } \
} while (0)

static __inline pfdep_debug_level_t pfdep_get_debug_level(void)
{
	return pfdep_debug_level;
}

static __inline void pfdep_set_debug_level(pfdep_debug_level_t level)
{
	pfdep_debug_level = level;
}

#define pfdep_assert(cond) BUG_ON(! (cond))

/*
 * Examine whether or not this packet handle is of a receiving buffer.
 *
 * This is a private API for Linux enviromnent.
 */
static __inline pfdep_bool pfdep_is_recv_pkt_buf(pfdep_pkt_handle_t pkt_handle)
{
	struct sk_buff *skb_p = (struct sk_buff *)pkt_handle;

	return (*((pfdep_bool *) skb_p->cb));
}

/*
 * Set packet buffer type (recv/send) to a private field
 * in the packet management information.
 *
 * This is a private API for Linux enviromnent.
 */
static __inline void pfdep_set_pkt_buf_type(pfdep_pkt_handle_t pkt_handle, pfdep_bool recv_buf_flag	/* TRUE:recv, FALSE:send */
    )
{
	struct sk_buff *skb_p = (struct sk_buff *)pkt_handle;

	(*((pfdep_bool *) skb_p->cb)) = recv_buf_flag;
}

#endif				/* __KERNEL__ */

#endif				/* PFDEP_H */
