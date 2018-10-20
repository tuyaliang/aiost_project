/**
 * ogma_driver_global.h
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
 *  Global data/type definitions and function declarations.
 *
 */

#ifndef OGMA_DRIVER_GLOBAL_H
#define OGMA_DRIVER_GLOBAL_H

#include "ogma_api.h"
#include "pfdep.h"
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/mii.h>

/************************************************************
 * Constant definitions
 ************************************************************/

#define OGMA_NETDEV_TX_PKT_SCAT_NUM_MAX (19)

/**
 * Minimum interval duration of PHY status polling.
 */
#define OGMA_PHY_STATUS_POLL_INTERVAL_MS_MIN (100L)

/************************************************************
 * Data type definitions
 ************************************************************/

typedef struct ogma_netdev_s {
	ogma_handle_t ogma_handle;
	struct net_device *netdev_p;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	struct napi_struct napi;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

	struct net_device_stats stats;
	struct device *dev_p;
	spinlock_t tx_queue_lock;
	ogma_uint rx_cksum_offload_flag:1;

	unsigned short tx_desc_num;
	unsigned short rx_desc_num;

	/* Rx IRQ coalesce parameters */
	unsigned short rxint_tmr_cnt_us;
	unsigned short rxint_pktcnt;

	/*
	 * Suppress tx_empty IRQ while tx_avail_num is larger than
	 * this threshold.
	 */
	unsigned short tx_empty_irq_activation_threshold;

	/*
	 * Handle of PHY status poller kernel thread.
	 */
	struct task_struct *phy_handler_kthread_p;

	unsigned int prev_link_status_flag:1;

	unsigned int prev_auto_nego_complete_flag:1;

	struct mii_if_info ogma_mii_if_info;
} ogma_netdev_t;

struct ogma_priv {
	void __iomem *base;
	int irq;
	int id;
	u8 mac[ETH_ALEN];
	struct net_device *netdev_p;
	ogma_handle_t handle;
	struct clk *clk[3];
	int gpio_phy_enable;
	int gpio_phy_nrst;
};

/************************************************************
 * Variable declarations
 ************************************************************/

extern unsigned int ogma_phy_interface;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
extern bool flow_ctrl;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */
extern int flow_ctrl;
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */

extern unsigned int flow_ctrl_start_threshold;
extern unsigned int flow_ctrl_stop_threshold;
extern unsigned short pause_time;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
extern bool use_jumbo_pkt_flag;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */
extern int use_jumbo_pkt_flag;
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */

extern short phy_dev_addr;
extern long phy_status_poll_interval_ms;

/************************************************************
 * Function declarations
 ************************************************************/

int ogma_netdev_init(ogma_handle_t ogma_handle,
		     struct device *dev_p,
		     const ogma_uint8 * mac_addr_p,
		     const char *name_p,
		     int napi_weight,
		     unsigned short tx_desc_num,
		     unsigned short rx_desc_num, struct net_device **netdev_pp);

void ogma_netdev_uninit(struct net_device *netdev_p);

#define OGMA_MSG_PFX "f_taiki :"

#define OGMA_MSG_ERR(x,y...) \
pfdep_print ( PFDEP_DEBUG_LEVEL_FATAL, " :%s: " x "\n", __func__, ## y)

#define OGMA_MSG_INFO(x,y...) \
pfdep_print ( PFDEP_DEBUG_LEVEL_NOTICE, " :%s: " x "\n", __func__, ## y)

#define OGMA_MSG_DEBUG(x,y...) \
pfdep_print ( PFDEP_DEBUG_LEVEL_DEBUG, " :%s: " x "\n", __func__, ## y)

#endif				/* OGMA_DRIVER_GLOBAL_H */
