/*
 * Copyright (C) 2015 Socionext Semiconductor Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * @file   sni_ipcu_comm.c
 * @author
 * @date
 * @brief  SNI IPCU Communication
 */

#include "sni_ipcu_drv.h"
#include <uapi/linux/sni_ipcu_parts.h>
#include "sni_ipcu_comm.h"

#include <stdbool.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#define CREATE_TRACE_POINTS
#include <trace/events/ipcu.h>

static DEFINE_SPINLOCK(spinlock_sni_ipcu);

static int sni_ipcu_com_init_done[IPCU_MAX_UNIT][IPCU_MAX_CH];


int sni_ipcu_comm_init(u32 unit, u32 ch)
{
	spin_lock(&spinlock_sni_ipcu);

	sni_ipcu_com_init_done[unit][ch] = 1;
	mutex_init(&ipcu_drv_inf[unit].ch_mutex[ch]);

	spin_unlock(&spinlock_sni_ipcu);

	return 0;
}

int sni_ipcu_send_req(u32 unit, u32 ch, u32 *data)
{
	u32 src_num;
	u32 src_bit;
	u32 dst_num;
	u32 mbx_bit;
	u32 mbstat;
	u32 cid;
	int ix;
	struct io_sni_ipcu *io_sni_ipcu;

	if (sni_ipcu_com_init_done[unit][ch] != 1)
		return -EACCES;

	if (mutex_lock_interruptible(
		&ipcu_drv_inf[unit].ch_mutex[ch]) == -EINTR)
		return -EINTR;

	io_sni_ipcu = (struct io_sni_ipcu *)(ipcu_drv_inf[unit].ipcu_io_mem);

	src_num = ipcu_drv_inf[unit].src_int_ch[ch];

	dst_num = ipcu_drv_inf[unit].dst_int_ch[ch];
	cid     = ch;

	src_bit = (1 << src_num);

	mbstat = io_sni_ipcu->mbstat;
	mbx_bit = (1 << cid);

	if ((mbstat & mbx_bit) != 0) {
		pr_err("%s:%d [ERROR] mbstat:%X, mbx_bit:%X, cid:%X\n",
			__func__, __LINE__, mbstat, mbx_bit, cid);
		mutex_unlock(&ipcu_drv_inf[unit].ch_mutex[ch]);
		/* no free mailbox */
		return -EBUSY;
	}

	/* Set source */
	io_sni_ipcu->mailbox[cid].source = src_bit;
	if (io_sni_ipcu->mailbox[cid].source != src_bit) {
		mutex_unlock(&ipcu_drv_inf[unit].ch_mutex[ch]);
		return -EFAULT;
	}

	/* Set destination */
	io_sni_ipcu->mailbox[cid].dest_set = (1 << dst_num);

	/* Set Mode "Manual Mode 2" */
	io_sni_ipcu->mailbox[cid].mode = 1;

	wmb();
	/* Data */
	for (ix = 0; ix < IO_IPCU_MBX_DATA_MAX; ix++)
		io_sni_ipcu->mailbox[cid].data[ix] = data[ix];

	wmb();

	/* Start sending */
	io_sni_ipcu->mailbox[cid].send = 1;
	trace_ipcu_send_req(unit, cid, src_num, data);
	wait_for_completion(&ipcu_drv_inf[unit].ack_notify[cid]);

	mutex_unlock(&ipcu_drv_inf[unit].ch_mutex[ch]);

	return 0;
}

int sni_ipcu_send_ack(u32 unit, u32 ch)
{
	struct io_sni_ipcu *io_sni_ipcu;

	if (sni_ipcu_com_init_done[unit][ch] != 1)
		return -EACCES;

	io_sni_ipcu = (struct io_sni_ipcu *)(ipcu_drv_inf[unit].ipcu_io_mem);

	/* clear destination */
	io_sni_ipcu->mailbox[ch].dest_clr =
		(1 << ipcu_drv_inf[unit].dst_int_ch[ch]);
	io_sni_ipcu->mailbox[ch].ack_set  =
		(1 << ipcu_drv_inf[unit].dst_int_ch[ch]);

	trace_ipcu_send_ack(unit, ch);
	return 0;
}

void sni_ipcu_handle_rec(int irq, u32 unit, u32 ch, u32 *data)
{
	int ix;
	struct io_sni_ipcu *io_sni_ipcu;

	if (sni_ipcu_com_init_done[unit][ch] != 1) {
		pr_err("%s:%d [ERROR] No initialization mbox\n",
			__func__, __LINE__);
		return;
	}

	io_sni_ipcu = (struct io_sni_ipcu *)ipcu_drv_inf[unit].ipcu_io_mem;

	for (ix = 0; ix < IO_IPCU_MBX_DATA_MAX; ix++)
		data[ix] = io_sni_ipcu->mailbox[ch].data[ix];

	/* clear destination */
	io_sni_ipcu->mailbox[ch].dest_clr =
		(1 << ipcu_drv_inf[unit].dst_int_ch[ch]);

	trace_ipcu_handle_rec(unit, ch, irq, data);
}

void sni_ipcu_handle_ack(int irq, u32 unit, u32 ch)
{
	u32 ack_reg;
	struct io_sni_ipcu *io_sni_ipcu;

	if (sni_ipcu_com_init_done[unit][ch] != 1) {
		pr_err("%s:%d [ERROR] No initialization mbox\n",
			__func__, __LINE__);
		return;
	}

	io_sni_ipcu = (struct io_sni_ipcu *)ipcu_drv_inf[unit].ipcu_io_mem;

	/* Read acknowldge status */
	ack_reg = io_sni_ipcu->mailbox[ch].ack_stat;

	/* Clear acknowlegde */
	io_sni_ipcu->mailbox[ch].ack_clr = ack_reg;

	/* Release Mail Box */
	io_sni_ipcu->mailbox[ch].source = 0;

	trace_ipcu_handle_ack(unit, ch, irq);
	/* Notify that received ack */
	complete(&ipcu_drv_inf[unit].ack_notify[ch]);
}

