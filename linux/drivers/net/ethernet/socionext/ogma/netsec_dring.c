/**
 * @file netsec_dring.c
 * @author K.MATSUI
 * @date 2014/Jun/xx-2014/Jun/xx
 * @brief NETSEC device driver
 */
/* Copyright (C) 2015 SOCIONEXT INCORPORATED All Rights Reserved. */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/netdevice.h>
#include <linux/dma-mapping.h>

#include "netsec_api.h"
#include "netsec_dring.h"
#include "netsec_drv.h"

int netsec_dring_tx_init(NETSEC_DRING * d, u16 num, u16 size, u16 id)
{
	NETSEC_DESC *dp;
	u8 *p;

	d->dring = dma_alloc_coherent(NULL,
				      (size_t) (num * size),
				      &d->phys_dring, GFP_KERNEL);
	if (d->dring == NULL) {
		return -ENOMEM;
	}

	d->info = kzalloc(num * sizeof(u32), GFP_NOWAIT);
	if (d->info == NULL) {
		dma_free_coherent(NULL, (size_t) (num * size), d->dring,
				  d->phys_dring);
		d->dring = NULL;
		return -ENOMEM;
	}

	memset(d->dring, 0, num * size);
	memset(d->info, 0, num * sizeof(u32));

	d->num_desc = num;
	d->size_desc = size;
	d->rd_idx = 0;
	d->wr_idx = 0;
	d->id = id;
	d->full = 0;
	d->tx = 1;
	d->buff_size = 0;

	spin_lock_init(&d->lock);

	/* last descriptor */
	p = (u8 *) d->dring;
	p += ((d->num_desc - 1) * d->size_desc);
	dp = (NETSEC_DESC *) p;
	dp->desc[0] |= NETSEC_DESC_LD;

	return 0;
}

int netsec_dring_rx_init(NETSEC_DRING * d, u16 num, u16 size, u16 id, u32 bufsz)
{
	NETSEC_DESC *dp;
	u8 *p;

	d->dring = dma_alloc_coherent(NULL,
				      (size_t) (num * size),
				      &d->phys_dring, GFP_KERNEL);
	if (d->dring == NULL) {
		return -ENOMEM;
	}

	d->info = kzalloc(num * sizeof(u32), GFP_NOWAIT);
	if (d->info == NULL) {
		dma_free_coherent(NULL, (size_t) (num * size), d->dring,
				  d->phys_dring);
		d->dring = NULL;
		return -ENOMEM;
	}

	memset(d->dring, 0, num * size);
	memset(d->info, 0, num * sizeof(u32));

	d->num_desc = num;
	d->size_desc = size;
	d->rd_idx = 0;
	d->wr_idx = 0;
	d->id = id;
	d->full = 0;
	d->tx = 0;
	d->buff_size = bufsz;

	spin_lock_init(&d->lock);

	/* last descriptor */
	p = (u8 *) d->dring;
	p += ((d->num_desc - 1) * d->size_desc);
	dp = (NETSEC_DESC *) p;
	dp->desc[0] |= NETSEC_DESC_LD;

	return 0;
}

int netsec_dring_exit(NETSEC_DRING * d)
{
	if (d->dring != NULL) {
		dma_free_coherent(NULL, (size_t) (d->num_desc * d->size_desc),
				  d->dring, d->phys_dring);
		d->dring = NULL;
	}
	if (d->info != NULL) {
		kfree(d->info);
		d->info = NULL;
	}
	return 0;
}

int netsec_dring_put_tx(NETSEC_DRING * d, void *desc, u32 info)
{
	u8 *p;
	NETSEC_DESC *dp;

	if (d->tx == 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	if (d->rd_idx == d->wr_idx) {
		if (d->full != 0) {
			spin_unlock(&d->lock);
			return -ENOMEM;
		}
	}

	dp = desc;
	dp->desc[0] |= NETSEC_DESC_OWN;
	if (d->wr_idx == (d->num_desc - 1)) {
		dp->desc[0] |= NETSEC_DESC_LD;
	}

	d->info[d->wr_idx] = info;

	p = (u8 *) d->dring;
	p += (d->size_desc * d->wr_idx);
	memcpy(p, desc, d->size_desc);

	d->wr_idx++;
	if (d->wr_idx >= d->num_desc) {
		d->wr_idx = 0;
	}
	if (d->wr_idx == d->rd_idx) {
		d->full = 1;
	}

	spin_unlock(&d->lock);

	return 1;
}

int netsec_dring_get_tx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 * pinfo)
{
	u8 *p;
	NETSEC_DESC *dp;

	if (d->tx == 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	if (d->rd_idx == d->wr_idx) {
		if (d->full == 0) {
			spin_unlock(&d->lock);
			return 0;	/* empty */
		}
	}

	p = (u8 *) d->dring;
	p += (d->size_desc * d->rd_idx);
	dp = (NETSEC_DESC *) p;

	if (dp->desc[0] & NETSEC_DESC_OWN) {
		spin_unlock(&d->lock);

		return 0;	/* sending */
	}

	memcpy(dptr, dp, d->size_desc);
	*pinfo = d->info[d->rd_idx];

	spin_unlock(&d->lock);

	return 1;
}

int netsec_dring_clear_tx(NETSEC_DRING * d)
{
	u8 *p;
	NETSEC_DESC *dp;

	if (d->tx == 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	if (d->rd_idx == d->wr_idx) {
		if (d->full == 0) {
			spin_unlock(&d->lock);
			return 0;	/* empty */
		}
	}

	p = (u8 *) d->dring;
	p += (d->size_desc * d->rd_idx);
	dp = (NETSEC_DESC *) p;

	if (dp->desc[0] & NETSEC_DESC_OWN) {
		spin_unlock(&d->lock);
		return 0;	/* sending */
	}

	memset(dp, 0, d->size_desc);
	d->info[d->rd_idx] = 0;

	d->rd_idx++;
	if (d->rd_idx >= d->num_desc) {
		d->rd_idx = 0;
	}
	d->full = 0;

	spin_unlock(&d->lock);

	return 1;
}

int netsec_dring_get_rx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 * pinfo)
{
	u8 *p;
	NETSEC_DESC *dp;

	if (d->tx != 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	p = (u8 *) d->dring;
	p += (d->size_desc * d->rd_idx);
	dp = (NETSEC_DESC *) p;

	if (dp->desc[0] & NETSEC_DESC_OWN) {
		spin_unlock(&d->lock);
		return 0;	/* not received */
	}

	memcpy(dptr, dp, d->size_desc);
	*pinfo = d->info[d->rd_idx];

	spin_unlock(&d->lock);

	return 1;
}

int netsec_dring_reset_rx(NETSEC_DRING * d, NETSEC_DESC * dptr, u32 info)
{
	u8 *p;
	NETSEC_DESC *dp;

	if (d->tx != 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	p = (u8 *) d->dring;
	p += (d->size_desc * d->rd_idx);
	dp = (NETSEC_DESC *) p;

	/* Clear OWN */
	dptr->desc[0] &= ~NETSEC_DESC_OWN;

	memcpy(dp, dptr, d->size_desc);
	if (d->rd_idx == (d->num_desc - 1)) {
		dp->desc[0] |= NETSEC_DESC_LD;
	}
	wmb();
	dp->desc[0] |= NETSEC_DESC_OWN;
	d->info[d->rd_idx] = info;

	d->rd_idx++;
	if (d->rd_idx >= d->num_desc) {
		d->rd_idx = 0;
	}

	spin_unlock(&d->lock);

	return 1;
}

int netsec_dring_get_tx_free(NETSEC_DRING * d)
{
	int ret;

	if (d->tx == 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	if (d->rd_idx == d->wr_idx) {
		if (d->full == 0) {
			ret = d->num_desc;	/* empty */
		} else {
			ret = 0;	/* full */
		}
	} else if (d->rd_idx < d->wr_idx) {
		ret = d->num_desc - (d->wr_idx - d->rd_idx);
	} else {
		ret = d->rd_idx - d->wr_idx;
	}

	spin_unlock(&d->lock);

	return ret;
}

int netsec_dring_get_rx_free(NETSEC_DRING * d)
{
	u16 ix;
	u8 *p;
	int count;
	NETSEC_DESC *dp;

	if (d->tx != 0) {
		return -EINVAL;
	}

	spin_lock(&d->lock);

	count = 0;
	ix = d->rd_idx;
	for (;;) {
		p = (u8 *) d->dring;
		p += (d->size_desc * ix);
		dp = (NETSEC_DESC *) p;

		if (dp->desc[0] & NETSEC_DESC_OWN) {
			/* not received */
			break;
		}
		count++;
		ix++;
		if (ix >= d->num_desc) {
			ix = 0;
		}
		if (ix == d->rd_idx) {
			break;
		}
	}

	spin_unlock(&d->lock);

	return d->num_desc - count;
}
