/**
 * ogma_netdev.c
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
 * OGMA network device implementation
 *
 */

#include "ogma_driver_global.h"
#include "ogma_basic_access.h"

#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <net/tcp.h>
#include <net/ip6_checksum.h>
#include <net/rtnetlink.h>
#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/mii.h>

/************************************************************
 * Constant definitions
 ************************************************************/
#define DRIVER_NAME "F_TAIKI Refernce Main Driver"
#define DRIVER_VERSION "1.0.0"

/*
 * Bit field definitions for PHY status register
 */
#define OGMA_PHY_SR_REG_AN_C                           (0x0020U)
#define OGMA_PHY_SR_REG_LINK                           (0x0004U)

/*
 * Bit field definitions for PHY 1000Base status register
 */
#define OGMA_PHY_1000BASE_REG_FULL                     (0x0800U)

/*
 * Bit field definitions for PHY ANLPA/ANA register
 */
#define OGMA_PHY_ANLPA_REG_TXF                         (0x0100U)
#define OGMA_PHY_ANLPA_REG_TXD                         (0x0080U)
#define OGMA_PHY_ANLPA_REG_TF                          (0x0040U)
#define OGMA_PHY_ANLPA_REG_TD                          (0x0020U)

/*
 * Bit field definitions for PHY ctrl register
 */
#define OGMA_PHY_CTRL_REG_RESET             (1U << 15)
#define OGMA_PHY_CTRL_REG_LOOPBACK          (1U << 14)
#define OGMA_PHY_CTRL_REG_SPSEL_LSB         (1U << 13)
#define OGMA_PHY_CTRL_REG_AUTO_NEGO_EN      (1U << 12)
#define OGMA_PHY_CTRL_REG_POWER_DOWN        (1U << 11)
#define OGMA_PHY_CTRL_REG_ISOLATE           (1U << 10)
#define OGMA_PHY_CTRL_REG_RESTART_AUTO_NEGO (1U << 9)
#define OGMA_PHY_CTRL_REG_DUPLEX_MODE       (1U << 8)
#define OGMA_PHY_CTRL_REG_COL_TEST          (1U << 7)
#define OGMA_PHY_CTRL_REG_SPSEL_MSB         (1U << 6)
#define OGMA_PHY_CTRL_REG_UNIDIR_EN         (1U << 5)

/*
 * Bit field definitions for PHY MASTER-SLAVE Ctrl register
 */
#define OGMA_PHY_MSC_REG_1000BASE_FULL (1U << 9)

/**
 * PHY register offset constants
 */
#define OGMA_PHY_REG_ADDR_CTRL        (0U)
#define OGMA_PHY_REG_ADDR_SR          (1U)
#define OGMA_PHY_REG_ADDR_ANA         (4U)
#define OGMA_PHY_REG_ADDR_ANLPA       (5U)
#define OGMA_PHY_REG_ADDR_MSC         (9U)
#define OGMA_PHY_REG_ADDR_1000BASE_SR (10U)

/************************************************************
 * Data type definitions
 ************************************************************/

/************************************************************
 * Variable definitions
 ************************************************************/
unsigned int ogma_phy_interface = OGMA_PHY_INTERFACE_RGMII;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
bool flow_ctrl = false;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */
int flow_ctrl = 0;
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */

unsigned int flow_ctrl_start_threshold = 36;
unsigned int flow_ctrl_stop_threshold = 48;
unsigned short pause_time = 256;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
bool use_jumbo_pkt_flag = true;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */
int use_jumbo_pkt_flag = 1;
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) */

short phy_dev_addr = -1;
long phy_status_poll_interval_ms = 500;

static const char ogma_gmac_mmc_reg_name[][ETH_GSTRING_LEN] = {
	"MMC_INTR_RX",
	"MMC_INTR_TX",
	"MMC_INTR_MASK_RX",
	"MMC_INTR_MASK_TX",
	"TXOCTETCOUNT_GB",
	"TXFRAMECOUNT_GB",
	"TXBROADCASTFRAMES_G",
	"TXMULTICASTFRAMES_G",
	"TX64OCTETS_GB",
	"TX65TO127OCTETS_GB",
	"TX128TO255OCTETS_GB",
	"TX256TO511OCTETS_GB",
	"TX512TO1023OCTETS_GB",
	"TX1024TOMAXOCTETS_GB",
	"TXUNICASTFRAMES_GB",
	"TXMULTICASTFRAMES_GB",
	"TXBROADCASTFRAMES_GB",
	"TXUNDERFLOWERROR",
	"TXSINGLECOL_G",
	"TXMULTICOL_G",
	"TXDEFERRED",
	"TXLATECOL",
	"TXEXESSCOL",
	"TXCARRIERERRROR",
	"TXOCTETCOUNT_G",
	"TXFRAMECOUNT_G",
	"TXEXECESSDEF",
	"TXPAUSEFRAMES",
	"TXVLANFRAMES_G",
	"RXFRAMECOUNT_GB",
	"RXOCTETCOUNT_GB",
	"RXOCTETCOUNT_G",
	"RXBROADCASTFRAMES_G",
	"RXMULTICASTFRAMES_G",
	"RXCRCERROR",
	"RXALLIGNMENTERROR",
	"RXRUNTERROR",
	"RXJABBERERROR",
	"RXUNDERSIZE_G",
	"RXOVERSIZE_G",
	"RX64OCTETS_GB",
	"RX65TO127OCTETS_GB",
	"RX128TO255OCTETS_GB",
	"RX256TO511OCTETS_GB",
	"RX512TO1023OCTETS_GB",
	"RX1024TOMAXOCTETS_GB",
	"RXUNICASTFRAMES_G",
	"RXLENGTHERROR",
	"RXOUTOFRANGETYPE",
	"RXPAUSEFRAMES",
	"RXFIFOOVERFLOW",
	"RXVLANFRAMES_GB",
	"RXWATCHDOGERROR",
	"MMC_IPC_INTR_MASK_RX",
	"MMC_IPC_INTR_RX",
	"RXIPV4_GD_FRMS",
	"RXIPV4_HDRERR_FRMS",
	"RXIPV4_NOPAY_FRMS",
	"RXIPV4_FRAG_FRMS",
	"RXIPV4_UDSBL_FRMS",
	"RXIPV6_GD_FRMS",
	"RXIPV6_HDRERR_FRMS",
	"RXIPV6_NOPAY_FRMS",
	"RXUDP_GD_FRMS",
	"RXUDP_ERR_FRMS",
	"RXTCP_GD_FRMS",
	"RXTCP_ERR_FRMS",
	"RXICMP_GD_FRMS",
	"RXICMP_ERR_FRMS",
	"RXIPV4_GD_OCTETS",
	"RXIPV4_HDRERR_OCTETS",
	"RXIPV4_NOPAY_OCTETS",
	"RXIPV4_FRAG_OCTETS",
	"RXIPV4_UDSBL_OCTETS",
	"RXIPV6_GD_OCTETS",
	"RXIPV6_HDRERR_OCTETS",
	"RXIPV6_NOPAY_OCTETS",
	"RXUDP_GD_OCTETS",
	"RXUDP_ERR_OCTETS",
	"RXTCP_GD_OCTETS",
	"RXTCP_ERR_OCTETS",
	"RXICMP_GD_OCTETS",
	"RXICMP_ERR_OCTETS"
};

static ogma_uint64 ogma_gmac_mmc_reg_stats[ARRAY_SIZE(ogma_gmac_mmc_reg_name)] =
    { 0 };
/************************************************************
 * Local function declarations
 ************************************************************/

static int ogma_netdev_open(struct net_device *netdev_p);

static int ogma_netdev_open_sub(ogma_netdev_t * ogma_netdev_p);

static int ogma_netdev_stop(struct net_device *netdev_p);

static void ogma_netdev_stop_sub(ogma_netdev_t * ogma_netdev_p,
				 ogma_bool gmac_stop_flag);

static netdev_tx_t ogma_netdev_start_xmit(struct sk_buff *skb_p,
					  struct net_device *netdev_p);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
static int ogma_netdev_napi_poll(struct napi_struct *napi_p, int budget);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
static int ogma_netdev_napi_poll(struct net_device *netdev_p, int *budget_p);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

static struct net_device_stats *ogma_netdev_get_stats(struct net_device
						      *netdev_p);

void ogma_netdev_set_rx_mode(struct net_device *netdev_p);

static void ogma_ethtool_get_drvinfo(struct net_device *netdev_p,
				     struct ethtool_drvinfo *drvinfo_p);

static u32 ogma_ethtool_get_msglevel(struct net_device *netdev_p);

static void ogma_ethtool_set_msglevel(struct net_device *netdev_p, u32 data);

static int ogma_netdev_change_mtu(struct net_device *netdev_p, int new_mtu);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
static int ogma_netdev_set_features(struct net_device *netdev_p,
				    netdev_features_t features);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) */
static u32 ogma_ethtool_get_rx_csum(struct net_device *netdev_p);

static int ogma_ethtool_set_rx_csum(struct net_device *netdev_p, u32 data);

static int ogma_ethtool_set_tso(struct net_device *netdev_p, u32 data);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) */

static void ogma_netdev_phy_poll(ogma_netdev_t * ogma_netdev_p);

static int ogma_netdev_set_mac_address(struct net_device *netdev_p, void *data);

static int ogma_ethtool_get_settings(struct net_device *netdev_p,
				     struct ethtool_cmd *cmd_p);

static int ogma_ethtool_set_settings(struct net_device *netdev_p,
				     struct ethtool_cmd *cmd_p);

static int ogma_get_sset_count(struct net_device *netdev_p, int stringset);

static void ogma_get_strings(struct net_device *netdev_p,
			     ogma_uint32 stringset, ogma_uint8 * data_p);

static void ogma_get_ethtool_stats(struct net_device *netdev_p,
				   struct ethtool_stats *ethtool_stats_p,
				   ogma_uint64 * stats_data_p);

static void ogma_netdev_mmc_stats_read(ogma_netdev_t * ogma_netdev_p,
				       ogma_uint64 * stats_data_p,
				       ogma_bool reset_flag);

static int ogma_netdev_nway_reset(struct net_device *netdev_p);

int ogma_netdev_mii_read(struct net_device *netdev_p,
			 int phy_addr, int reg_addr);

void ogma_netdev_mii_write(struct net_device *netdev_p,
			   int phy_addr, int reg_addr, int value);

void ogma_netdev_set_mii_if_info(struct net_device *netdev_p,
				 ogma_netdev_t * ogma_netdev_p);

/************************************************************
 * Variable definitions (cont.)
 ************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops ogma_netdev_ops = {
	.ndo_open = ogma_netdev_open,
	.ndo_stop = ogma_netdev_stop,
	.ndo_start_xmit = ogma_netdev_start_xmit,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	.ndo_set_features = ogma_netdev_set_features,
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) */
	.ndo_get_stats = ogma_netdev_get_stats,
	.ndo_set_rx_mode = ogma_netdev_set_rx_mode,
	.ndo_change_mtu = ogma_netdev_change_mtu,
	.ndo_set_mac_address = ogma_netdev_set_mac_address,
	.ndo_validate_addr = eth_validate_addr
};
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */

static const struct ethtool_ops ogma_ethtool_ops = {
	.get_drvinfo = ogma_ethtool_get_drvinfo,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
	.get_rx_csum = ogma_ethtool_get_rx_csum,
	.set_rx_csum = ogma_ethtool_set_rx_csum,
	.get_tx_csum = ethtool_op_get_tx_csum,
	.set_tx_csum = ethtool_op_set_tx_ipv6_csum,
	.get_sg = ethtool_op_get_sg,
	.set_sg = ethtool_op_set_sg,
	.get_tso = ethtool_op_get_tso,
	.set_tso = ogma_ethtool_set_tso,
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) */
	.get_coalesce = NULL,
	.set_coalesce = NULL,
	.get_msglevel = ogma_ethtool_get_msglevel,
	.set_msglevel = ogma_ethtool_set_msglevel,
	.get_link = ethtool_op_get_link,
	.get_settings = ogma_ethtool_get_settings,
	.set_settings = ogma_ethtool_set_settings,
	.get_sset_count = ogma_get_sset_count,
	.get_strings = ogma_get_strings,
	.get_ethtool_stats = ogma_get_ethtool_stats,
	.nway_reset = ogma_netdev_nway_reset
};

/************************************************************
 * Function definitions
 ************************************************************/

int ogma_netdev_init(ogma_handle_t ogma_handle,
		     struct device *dev_p,
		     const ogma_uint8 * mac_addr_p,
		     const char *name_p,
		     int napi_weight,
		     unsigned short tx_desc_num,
		     unsigned short rx_desc_num, struct net_device **netdev_pp)
{

	struct net_device *netdev_p;
	ogma_netdev_t *ogma_netdev_p;
	int err = 0;

	if ((netdev_p = alloc_netdev(sizeof(ogma_netdev_t),
				     name_p, NET_NAME_UNKNOWN, ether_setup))
	    == NULL) {
		return -ENOMEM;
	}

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	ogma_netdev_p->dev_p = dev_p;
	ogma_netdev_p->phy_handler_kthread_p = NULL;

	SET_NETDEV_DEV(netdev_p, dev_p);

	memcpy(netdev_p->dev_addr, mac_addr_p, 6);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	netif_napi_add(netdev_p,
		       &ogma_netdev_p->napi,
		       ogma_netdev_napi_poll, napi_weight);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	netdev_p->poll = ogma_netdev_napi_poll;
	netdev_p->weight = napi_weight;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
	netdev_p->netdev_ops = &ogma_netdev_ops;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29) */
	netdev_p->open = ogma_netdev_open;
	netdev_p->stop = ogma_netdev_stop;
	netdev_p->hard_start_xmit = ogma_netdev_start_xmit;
	netdev_p->get_stats = ogma_netdev_get_stats;
	netdev_p->change_mtu = ogma_netdev_change_mtu;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */

	netdev_p->ethtool_ops = &ogma_ethtool_ops;

	netdev_p->features = (NETIF_F_SG |
			      NETIF_F_IP_CSUM |
			      NETIF_F_IPV6_CSUM |
			      NETIF_F_TSO |
			      NETIF_F_TSO6 |
			      NETIF_F_GSO | NETIF_F_HIGHDMA | NETIF_F_RXCSUM);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	netdev_p->hw_features = netdev_p->features;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) */

	ogma_netdev_p->ogma_handle = ogma_handle;
	ogma_netdev_p->netdev_p = netdev_p;
	ogma_netdev_p->rx_cksum_offload_flag = OGMA_TRUE;
	spin_lock_init(&ogma_netdev_p->tx_queue_lock);

	ogma_netdev_p->tx_desc_num = tx_desc_num;
	ogma_netdev_p->rx_desc_num = rx_desc_num;

	/*
	 * Rx IRQ coalesce is disabled by default. 
	 */
	ogma_netdev_p->rxint_tmr_cnt_us = 0;
	ogma_netdev_p->rxint_pktcnt = 1;

	/*
	 * Tx_empty IRQ suppression is effective by default.
	 */
	ogma_netdev_p->tx_empty_irq_activation_threshold = tx_desc_num - 2;

	ogma_netdev_p->prev_link_status_flag = 0;
	ogma_netdev_p->prev_auto_nego_complete_flag = 0;

	if ((err = register_netdev(netdev_p)) != 0) {
		OGMA_MSG_ERR("register_netdev() failed");
		goto error;
	}

	ogma_netdev_set_mii_if_info(netdev_p, ogma_netdev_p);

	*netdev_pp = netdev_p;

	return 0;

 error:
	free_netdev(netdev_p);

	return err;

}

void ogma_netdev_uninit(struct net_device *netdev_p)
{

	ogma_netdev_t *ogma_netdev_p;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	if (ogma_netdev_p->phy_handler_kthread_p != NULL) {
		kthread_stop(ogma_netdev_p->phy_handler_kthread_p);
		ogma_netdev_p->phy_handler_kthread_p = NULL;
	}

	unregister_netdev(netdev_p);

	free_netdev(netdev_p);

}

static int ogma_netdev_open(struct net_device *netdev_p)
{
	ogma_netdev_t *ogma_netdev_p;
	ogma_err_t ogma_err;
	int err = 0;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	if ((ogma_err = ogma_clean_rx_desc_ring(ogma_netdev_p->ogma_handle,
						OGMA_DESC_RING_ID_NRM_RX))
	    != OGMA_ERR_OK) {
		OGMA_MSG_ERR
		    ("ogma_clean_rx_desc_ring() failed with error code %d.",
		     (int)ogma_err);
		err = -EINVAL;
		goto error;
	}

	if ((ogma_err = ogma_clean_tx_desc_ring(ogma_netdev_p->ogma_handle,
						OGMA_DESC_RING_ID_NRM_TX))
	    != OGMA_ERR_OK) {
		OGMA_MSG_ERR
		    ("ogma_clean_tx_desc_ring() failed with error code %d.",
		     (int)ogma_err);
		err = -EINVAL;
		goto error;
	}

	ogma_clear_desc_ring_irq_status(ogma_netdev_p->ogma_handle,
					OGMA_DESC_RING_ID_NRM_TX,
					OGMA_CH_IRQ_REG_EMPTY);

	if ((err = ogma_netdev_open_sub(ogma_netdev_p)) != 0) {
		OGMA_MSG_ERR("ogma_netdev_open_sub() failed\n");
	}

 error:
	return err;
}

static int ogma_netdev_open_sub(ogma_netdev_t * ogma_netdev_p)
{

	ogma_err_t ogma_err;
	int err = 0;

	struct net_device *netdev_p = ogma_netdev_p->netdev_p;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	napi_enable(&ogma_netdev_p->napi);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	netif_poll_enable(netdev_p);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

	if ((ogma_err = ogma_start_desc_ring(ogma_netdev_p->ogma_handle,
					     OGMA_DESC_RING_ID_NRM_RX))
	    != OGMA_ERR_OK) {
		OGMA_MSG_ERR
		    ("ogma_start_desc_ring(ring_id=%d) failed with error code %d.",
		     OGMA_DESC_RING_ID_NRM_RX, (int)ogma_err);
		err = -EINVAL;
		goto disable_napi;
	}

	if ((ogma_err = ogma_set_irq_coalesce_param(ogma_netdev_p->ogma_handle,
						    OGMA_DESC_RING_ID_NRM_RX,
						    ogma_netdev_p->rxint_pktcnt,
						    OGMA_FALSE,
						    ogma_netdev_p->rxint_tmr_cnt_us))
	    != OGMA_ERR_OK) {
		OGMA_MSG_ERR
		    ("ogma_set_irq_coalesce_param() failed with error code %d.",
		     (int)ogma_err);
		err = -EINVAL;
		goto stop_desc_ring_nrm_rx;
	}

	if ((ogma_err = ogma_start_desc_ring(ogma_netdev_p->ogma_handle,
					     OGMA_DESC_RING_ID_NRM_TX))
	    != OGMA_ERR_OK) {
		OGMA_MSG_ERR
		    ("ogma_start_desc_ring(ring_id=%d) failed with error code %d.",
		     OGMA_DESC_RING_ID_NRM_TX, (int)ogma_err);
		err = -EINVAL;
		goto stop_desc_ring_nrm_rx;
	}

	/*
	 * We adaptively control tx_empty IRQ.
	 */
	ogma_disable_desc_ring_irq(ogma_netdev_p->ogma_handle,
				   OGMA_DESC_RING_ID_NRM_TX,
				   OGMA_CH_IRQ_REG_EMPTY);

	netif_start_queue(netdev_p);

	ogma_netdev_phy_poll(ogma_netdev_p);

	ogma_netdev_set_rx_mode(netdev_p);

	return 0;

	netif_stop_queue(ogma_netdev_p->netdev_p);

	ogma_stop_desc_ring(ogma_netdev_p->ogma_handle,
			    OGMA_DESC_RING_ID_NRM_TX);

 stop_desc_ring_nrm_rx:
	ogma_stop_desc_ring(ogma_netdev_p->ogma_handle,
			    OGMA_DESC_RING_ID_NRM_RX);

 disable_napi:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	napi_disable(&ogma_netdev_p->napi);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	netif_poll_disable(netdev_p);
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */

	return err;

}

static int ogma_netdev_stop(struct net_device *netdev_p)
{

	ogma_netdev_t *ogma_netdev_p;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	ogma_netdev_stop_sub(ogma_netdev_p, OGMA_TRUE);

	return 0;

}

static void ogma_netdev_stop_sub(ogma_netdev_t * ogma_netdev_p,
				 ogma_bool gmac_stop_flag)
{

	struct net_device *netdev_p = ogma_netdev_p->netdev_p;

	if (ogma_netdev_p->phy_handler_kthread_p != NULL) {
		kthread_stop(ogma_netdev_p->phy_handler_kthread_p);
		ogma_netdev_p->phy_handler_kthread_p = NULL;
	}

	ogma_netdev_p->prev_link_status_flag = 0;
	ogma_netdev_p->prev_auto_nego_complete_flag = 0;

	netif_stop_queue(netdev_p);

	if (gmac_stop_flag) {
		ogma_stop_gmac(ogma_netdev_p->ogma_handle,
			       OGMA_TRUE, OGMA_TRUE);
	}

	ogma_stop_desc_ring(ogma_netdev_p->ogma_handle,
			    OGMA_DESC_RING_ID_NRM_RX);
	ogma_stop_desc_ring(ogma_netdev_p->ogma_handle,
			    OGMA_DESC_RING_ID_NRM_TX);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	napi_disable(&ogma_netdev_p->napi);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	netif_poll_disable(netdev_p);
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */

}

static netdev_tx_t ogma_netdev_start_xmit(struct sk_buff *skb_p,
					  struct net_device *netdev_p)
{
	ogma_netdev_t *ogma_netdev_p;
	ogma_tx_pkt_ctrl_t tx_pkt_ctrl;
	ogma_frag_info_t *scat_info_p;
	ogma_uint8 scat_num;
	skb_frag_t *skb_frag_p;
	ogma_err_t ogma_err;
	ogma_uint16 tx_avail_num;
	ogma_uint16 tso_seg_len;	/* 0 means TSO is not required. */
	int i;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	memset(&tx_pkt_ctrl, 0, sizeof(ogma_tx_pkt_ctrl_t));

	ogma_clear_desc_ring_irq_status(ogma_netdev_p->ogma_handle,
					OGMA_DESC_RING_ID_NRM_TX,
					OGMA_CH_IRQ_REG_EMPTY);

	ogma_clean_tx_desc_ring(ogma_netdev_p->ogma_handle,
				OGMA_DESC_RING_ID_NRM_TX);

	BUG_ON(skb_shinfo(skb_p)->nr_frags >= OGMA_NETDEV_TX_PKT_SCAT_NUM_MAX);

	scat_num = skb_shinfo(skb_p)->nr_frags + 1;

	scat_info_p = kzalloc(scat_num * sizeof(ogma_frag_info_t), GFP_NOWAIT);
	if (scat_info_p == NULL) {
		return NETDEV_TX_BUSY;
	}

	if (skb_p->ip_summed == CHECKSUM_PARTIAL) {
		if (skb_p->protocol == htons(ETH_P_IP)) {
			ip_hdr(skb_p)->check = 0;
		}
		tx_pkt_ctrl.cksum_offload_flag = OGMA_TRUE;
	}

	tso_seg_len = 0;

	if (skb_is_gso(skb_p)) {
		tso_seg_len = skb_shinfo(skb_p)->gso_size;

		BUG_ON(skb_p->ip_summed != CHECKSUM_PARTIAL);
		BUG_ON(tso_seg_len == 0);
		BUG_ON(tso_seg_len >
		       (use_jumbo_pkt_flag ? OGMA_TCP_JUMBO_SEG_LEN_MAX :
			OGMA_TCP_SEG_LEN_MAX));

		/*
		 * If peer MSS is smaller than the minimum processible value,
		 * use our minimum instead.
		 */
		if (tso_seg_len < OGMA_TCP_SEG_LEN_MIN) {
			tso_seg_len = OGMA_TCP_SEG_LEN_MIN;

			if (skb_p->data_len < OGMA_TCP_SEG_LEN_MIN) {
				/*
				 * TSO is no longer neccesary under the modified MSS.
				 */
				tso_seg_len = 0;
			}
		}
	}

	if (tso_seg_len > 0) {

		if (skb_p->protocol == htons(ETH_P_IP)) {

			BUG_ON((skb_shinfo(skb_p)->gso_type & SKB_GSO_TCPV4) ==
			       0);

			ip_hdr(skb_p)->tot_len = 0;

			tcp_hdr(skb_p)->check =
			    ~tcp_v4_check(0,
					  ip_hdr(skb_p)->saddr,
					  ip_hdr(skb_p)->daddr, 0);
		} else {

			BUG_ON((skb_shinfo(skb_p)->gso_type & SKB_GSO_TCPV6) ==
			       0);

			ipv6_hdr(skb_p)->payload_len = 0;

			tcp_hdr(skb_p)->check =
			    ~csum_ipv6_magic(&ipv6_hdr(skb_p)->saddr,
					     &ipv6_hdr(skb_p)->daddr,
					     0, IPPROTO_TCP, 0);
		}

		tx_pkt_ctrl.tcp_seg_offload_flag = OGMA_TRUE;
		tx_pkt_ctrl.tcp_seg_len = tso_seg_len;

	}

	scat_info_p[0].phys_addr =
	    dma_map_single(ogma_netdev_p->dev_p,
			   skb_p->data, skb_headlen(skb_p), DMA_TO_DEVICE);

	scat_info_p[0].addr = skb_p->data;
	scat_info_p[0].len = skb_headlen(skb_p);

	for (i = 0; i < skb_shinfo(skb_p)->nr_frags; i++) {

		skb_frag_p = &skb_shinfo(skb_p)->frags[i];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
		scat_info_p[i + 1].phys_addr =
		    skb_frag_dma_map(ogma_netdev_p->dev_p,
				     skb_frag_p,
				     0,
				     skb_frag_size(skb_frag_p), DMA_TO_DEVICE);
		scat_info_p[i + 1].addr = skb_frag_address(skb_frag_p);
#else				/* LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0) */
		scat_info_p[i + 1].phys_addr =
		    dma_map_single(ogma_netdev_p->dev_p,
				   page_address(skb_frag_p->page)
				   + skb_frag_p->page_offset,
				   skb_frag_p->size, DMA_TO_DEVICE);
		scat_info_p[i + 1].addr = page_address(skb_frag_p->page)
		    + skb_frag_p->page_offset;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0) */
		scat_info_p[i + 1].len = skb_frag_p->size;
	}

	tx_pkt_ctrl.target_desc_ring_id = OGMA_DESC_RING_ID_GMAC;

	pfdep_set_pkt_buf_type(skb_p, PFDEP_FALSE);

	ogma_err = ogma_set_tx_pkt_data(ogma_netdev_p->ogma_handle,
					OGMA_DESC_RING_ID_NRM_TX,
					&tx_pkt_ctrl,
					scat_num, scat_info_p, skb_p);

	if (ogma_err != OGMA_ERR_OK) {
		OGMA_MSG_ERR("ogma_set_tx_pkt_data failed with error code %d.",
			     (int)ogma_err);
		for (i = 0; i < scat_num; i++) {

			dma_unmap_single(ogma_netdev_p->dev_p,
					 scat_info_p[i].phys_addr,
					 scat_info_p[i].len, DMA_TO_DEVICE);
		}

		kfree(scat_info_p);

		++ogma_netdev_p->stats.tx_dropped;
		ogma_netdev_p->netdev_p->stats.tx_dropped++;

		return NETDEV_TX_BUSY;
	}

	kfree(scat_info_p);

	++ogma_netdev_p->stats.tx_packets;
	ogma_netdev_p->stats.tx_bytes += skb_p->len;
	netdev_p->stats.tx_packets++;
	netdev_p->stats.tx_bytes += skb_p->len;

	spin_lock(&ogma_netdev_p->tx_queue_lock);

	tx_avail_num = ogma_get_tx_avail_num(ogma_netdev_p->ogma_handle,
					     OGMA_DESC_RING_ID_NRM_TX);

	/*
	 * Adaptively switch on/off tx_empty IRQ.
	 */

	if (tx_avail_num < OGMA_NETDEV_TX_PKT_SCAT_NUM_MAX) {
		ogma_enable_desc_ring_irq(ogma_netdev_p->ogma_handle,
					  OGMA_DESC_RING_ID_NRM_TX,
					  OGMA_CH_IRQ_REG_EMPTY);
		netif_stop_queue(netdev_p);
	} else if (tx_avail_num <=
		   ogma_netdev_p->tx_empty_irq_activation_threshold) {
		ogma_enable_desc_ring_irq(ogma_netdev_p->ogma_handle,
					  OGMA_DESC_RING_ID_NRM_TX,
					  OGMA_CH_IRQ_REG_EMPTY);
	} else {
		ogma_disable_desc_ring_irq(ogma_netdev_p->ogma_handle,
					   OGMA_DESC_RING_ID_NRM_TX,
					   OGMA_CH_IRQ_REG_EMPTY);

	}

	spin_unlock(&ogma_netdev_p->tx_queue_lock);

	return NETDEV_TX_OK;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
static int ogma_netdev_napi_poll(struct napi_struct *napi_p, int budget)
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
static int ogma_netdev_napi_poll(struct net_device *netdev_p, int *budget_p)
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */
{
	ogma_netdev_t *ogma_netdev_p;
	ogma_rx_pkt_info_t rx_pkt_info;
	ogma_frag_info_t frag_info;
	ogma_uint16 len;
	struct sk_buff *skb_p;
	ogma_err_t ogma_err;
	int i = 0;
	int tx_queue_status;
	int rx_num;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	int budget;
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	ogma_netdev_p = (ogma_netdev_t *) container_of(napi_p,
						       ogma_netdev_t, napi);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	ogma_netdev_p = netdev_priv(netdev_p);
	budget = min(*budget_p, netdev_p->quota);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

	for (i = 0, rx_num = 0; i < budget; i++, rx_num--) {

		if ((rx_num == 0) &&
		    ((rx_num = ogma_get_rx_num(ogma_netdev_p->ogma_handle,
					       OGMA_DESC_RING_ID_NRM_RX))
		     == 0)) {

			break;
		}

		if ((ogma_err = ogma_get_rx_pkt_data(ogma_netdev_p->ogma_handle,
						     OGMA_DESC_RING_ID_NRM_RX,
						     &rx_pkt_info,
						     &frag_info, &len, &skb_p))
		    == OGMA_ERR_ALLOC) {
			OGMA_MSG_ERR
			    ("ogma_get_rx_pkt_data failed with error code %d.",
			     (int)ogma_err);

			++ogma_netdev_p->stats.rx_dropped;
			ogma_netdev_p->netdev_p->stats.rx_dropped++;
			continue;
		}

		dma_unmap_single(ogma_netdev_p->dev_p,
				 frag_info.phys_addr,
				 frag_info.len, DMA_FROM_DEVICE);

		skb_put(skb_p, len);

		skb_p->protocol =
		    eth_type_trans(skb_p, ogma_netdev_p->netdev_p);

		skb_p->dev = ogma_netdev_p->netdev_p;

		if ((ogma_netdev_p->rx_cksum_offload_flag == OGMA_TRUE) &&
		    (rx_pkt_info.rx_cksum_result == OGMA_RX_CKSUM_RESULT_OK)) {
			skb_p->ip_summed = CHECKSUM_UNNECESSARY;
		} else {
			skb_p->ip_summed = CHECKSUM_NONE;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
		napi_gro_receive(napi_p, skb_p);
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29) */
		netif_receive_skb(skb_p);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */

		++ogma_netdev_p->stats.rx_packets;
		ogma_netdev_p->stats.rx_bytes += len;
		ogma_netdev_p->netdev_p->stats.rx_packets++;
		ogma_netdev_p->netdev_p->stats.rx_bytes += len;
	}

	ogma_clear_desc_ring_irq_status(ogma_netdev_p->ogma_handle,
					OGMA_DESC_RING_ID_NRM_TX,
					OGMA_CH_IRQ_REG_EMPTY);

	ogma_clean_tx_desc_ring(ogma_netdev_p->ogma_handle,
				OGMA_DESC_RING_ID_NRM_TX);

	spin_lock(&ogma_netdev_p->tx_queue_lock);

	tx_queue_status = netif_queue_stopped(ogma_netdev_p->netdev_p);

	spin_unlock(&ogma_netdev_p->tx_queue_lock);

	if (tx_queue_status != 0) {

		if (ogma_get_tx_avail_num(ogma_netdev_p->ogma_handle,
					  OGMA_DESC_RING_ID_NRM_TX)
		    >= OGMA_NETDEV_TX_PKT_SCAT_NUM_MAX) {
			/* 4-6 */
			netif_wake_queue(ogma_netdev_p->netdev_p);
		}
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (i == budget) {
		return budget;
	}
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	netif_rx_complete(netdev_p);
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

	napi_complete(napi_p);

	ogma_enable_top_irq(ogma_netdev_p->ogma_handle,
			    (OGMA_TOP_IRQ_REG_NRM_TX |
			     OGMA_TOP_IRQ_REG_NRM_RX));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	return i;
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
	/* 9 */
	*budget_p -= i;
	metdev_p->quota -= i;
	return 0;
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */

}

static struct net_device_stats *ogma_netdev_get_stats(struct net_device
						      *netdev_p)
{
	return &netdev_p->stats;
}

void ogma_netdev_set_rx_mode(struct net_device *netdev_p)
{
	ogma_netdev_t *ogma_netdev_p;
	ogma_ctrl_t *ctrl_p;
	struct netdev_hw_addr *ha;
	int ix;
	ogma_uint32 hi, lo;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);
	ctrl_p = (ogma_ctrl_t *) ogma_netdev_p->ogma_handle;

	// my mac address
	hi = netdev_p->dev_addr[5];
	hi <<= 8;
	hi |= netdev_p->dev_addr[4];
	hi |= 0x80000000;

	lo = netdev_p->dev_addr[3];
	lo <<= 8;
	lo |= netdev_p->dev_addr[2];
	lo <<= 8;
	lo |= netdev_p->dev_addr[1];
	lo <<= 8;
	lo |= netdev_p->dev_addr[0];

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MAR0H, hi);
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MAR0L, lo);

	if (netdev_p->flags & IFF_PROMISC) {
//              printk(KERN_DEBUG "%s: IFF_PROMISC\n", __FUNCTION__);
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MFFR, 0x80000001);
	} else if ((netdev_p->flags & IFF_ALLMULTI) ||
		   (netdev_mc_count(netdev_p) > 31)) {
//              printk(KERN_DEBUG "%s: IFF_ALLMULTI or mc_count(%d) > 31\n",
//                      __FUNCTION__, netdev_mc_count(netdev_p));
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MFFR, 0x00000010);
	} else if (netdev_mc_empty(netdev_p)) {
//              printk(KERN_DEBUG "%s: mc_empty\n", __FUNCTION__);
		for (ix = 1; ix < 16; ix++) {
			ogma_set_mac_reg(ctrl_p,
					 OGMA_GMAC_REG_ADDR_MAR0H + (ix * 8),
					 0x00000000);
			ogma_set_mac_reg(ctrl_p,
					 OGMA_GMAC_REG_ADDR_MAR0L + (ix * 8),
					 0x00000000);
		}
		for (; ix < 32; ix++) {
			ogma_set_mac_reg(ctrl_p,
					 OGMA_GMAC_REG_ADDR_MAR16H + (ix * 8),
					 0x00000000);
			ogma_set_mac_reg(ctrl_p,
					 OGMA_GMAC_REG_ADDR_MAR16L + (ix * 8),
					 0x00000000);
		}
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MFFR, 0x00000000);
	} else {
//              printk(KERN_DEBUG "%s: mc_count=%d\n", __FUNCTION__,
//                              netdev_mc_count(netdev_p));
		ix = 1;
		netdev_for_each_mc_addr(ha, netdev_p) {
//                      printk(KERN_DEBUG "%s: addr=%x:%x:%x:%x:%x:%x\n",
//                              __FUNCTION__,
//                              ha->addr[0],ha->addr[1],ha->addr[2],
//                              ha->addr[3],ha->addr[4],ha->addr[5]);
			hi = ha->addr[5];
			hi <<= 8;
			hi |= ha->addr[4];
			hi |= 0x80000000;

			lo = ha->addr[3];
			lo <<= 8;
			lo |= ha->addr[2];
			lo <<= 8;
			lo |= ha->addr[1];
			lo <<= 8;
			lo |= ha->addr[0];

			if (ix < 16) {
				ogma_set_mac_reg(ctrl_p,
						 OGMA_GMAC_REG_ADDR_MAR0H +
						 (ix * 8), hi);
				ogma_set_mac_reg(ctrl_p,
						 OGMA_GMAC_REG_ADDR_MAR0L +
						 (ix * 8), lo);
			} else {
				ogma_set_mac_reg(ctrl_p,
						 OGMA_GMAC_REG_ADDR_MAR16H +
						 (ix * 8), hi);
				ogma_set_mac_reg(ctrl_p,
						 OGMA_GMAC_REG_ADDR_MAR16L +
						 (ix * 8), lo);
			}
			ix++;
		}
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MFFR, 0x00000000);
	}
}

static void ogma_ethtool_get_drvinfo(struct net_device *netdev_p,
				     struct ethtool_drvinfo *drvinfo)
{
	char buf[32] = { 0 };
	u32 value = 0;

	ogma_netdev_t *ogma_netdev_p;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	strlcpy(drvinfo->driver, DRIVER_NAME, sizeof(drvinfo->driver));

	strlcpy(drvinfo->version, DRIVER_VERSION, sizeof(drvinfo->version));

	value = ogma_get_mcr_ver(ogma_netdev_p->ogma_handle);

	sprintf(buf, "MCR Version %08x", value);

	strlcpy(drvinfo->fw_version, buf, sizeof(drvinfo->fw_version));
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
static u32 ogma_ethtool_get_rx_csum(struct net_device *netdev_p)
{
	ogma_netdev_t *ogma_netdev_p;
	/* 1 */
	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);
	/* 2 */
	return (int)ogma_netdev_p->rx_cksum_offload_flag;
}

static int ogma_ethtool_set_rx_csum(struct net_device *netdev_p, u32 data)
{
	ogma_netdev_t *ogma_netdev_p;
	/* 1 */
	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);
	/* 2 */
	ogma_netdev_p->rx_cksum_offload_flag = (data != 0);
	return 0;
}
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) */

static u32 ogma_ethtool_get_msglevel(struct net_device *netdev_p)
{
	return (u32) pfdep_get_debug_level();
}

static void ogma_ethtool_set_msglevel(struct net_device *netdev_p, u32 data)
{
	pfdep_set_debug_level((pfdep_debug_level_t) data);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
static int ogma_ethtool_set_tso(struct net_device *netdev_p, u32 data)
{
	if (data) {

		netdev_p->features |= (NETIF_F_TSO | NETIF_F_TSO6);

	} else {

		netdev_p->features &= ~(NETIF_F_TSO | NETIF_F_TSO6);
	}

	return 0;
}
#endif				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) */

static int ogma_netdev_change_mtu(struct net_device *netdev_p, int new_mtu)
{

	if (use_jumbo_pkt_flag == 0) {
		return eth_change_mtu(netdev_p, new_mtu);
	}

	if ((new_mtu < 68) || (new_mtu > 9000)) {
		return -EINVAL;
	}

	netdev_p->mtu = new_mtu;

	return 0;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
static int ogma_netdev_set_features(struct net_device *netdev_p,
				    netdev_features_t features)
{
	ogma_netdev_t *ogma_netdev_p;

	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	ogma_netdev_p->rx_cksum_offload_flag = ! !(features & NETIF_F_RXCSUM);

	return 0;
}
#endif				/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) */

static void ogma_netdev_get_phy_link_status(ogma_netdev_t * ogma_netdev_p,
					    unsigned int *link_status_flag_p,
					    unsigned int
					    *auto_nego_complete_flag_p, unsigned int
					    *latched_link_down_flag_p,
					    unsigned int *link_speed_p,
					    unsigned int *half_duplex_flag_p)
{

	unsigned short reg_val_msc;

	*link_status_flag_p = 1;
	*auto_nego_complete_flag_p = 1;
	*latched_link_down_flag_p = 0;
	*link_speed_p = OGMA_PHY_LINK_SPEED_100M;
	*half_duplex_flag_p = 0;

	reg_val_msc = 0x0020;
	ogma_set_phy_reg(ogma_netdev_p->ogma_handle,
			 phy_dev_addr, 0x18, reg_val_msc);
	reg_val_msc =
	    ogma_get_phy_reg(ogma_netdev_p->ogma_handle, phy_dev_addr, 0x18);
}

static ogma_err_t ogma_netdev_configure_mac(ogma_netdev_t * ogma_netdev_p,
					    int link_speed,
					    int half_duplex_flag)
{

	ogma_err_t ogma_err;
	ogma_gmac_mode_t ogma_gmac_mode;

	memset(&ogma_gmac_mode, 0, sizeof(ogma_gmac_mode_t));

	ogma_gmac_mode.link_speed = link_speed;
	ogma_gmac_mode.half_duplex_flag = (ogma_bool) half_duplex_flag;
	ogma_gmac_mode.flow_ctrl_enable_flag = (ogma_bool) flow_ctrl;
	ogma_gmac_mode.flow_ctrl_start_threshold =
	    (ogma_uint16) flow_ctrl_start_threshold;
	ogma_gmac_mode.flow_ctrl_stop_threshold =
	    (ogma_uint16) flow_ctrl_stop_threshold;

	ogma_err = OGMA_ERR_OK;

	return ogma_err;

}

/**
 * Check PHY link status and configure F_GMAC4MT if necessary.
 */
static void ogma_netdev_phy_poll(ogma_netdev_t * ogma_netdev_p)
{

	unsigned int link_status_flag;
	unsigned int auto_nego_complete_flag;
	unsigned int latched_link_down_flag;
	unsigned int link_speed;
	unsigned half_duplex_flag;

	ogma_err_t ogma_err;

	if ((ogma_netdev_p->netdev_p->flags & IFF_UP) == 0) {
		/*
		 * I/F is down. Quit immediately.
		 */
		return;
	}

	ogma_netdev_get_phy_link_status(ogma_netdev_p,
					&link_status_flag,
					&auto_nego_complete_flag,
					&latched_link_down_flag,
					&link_speed, &half_duplex_flag);

	if ((latched_link_down_flag == 0) &&
	    (link_status_flag == ogma_netdev_p->prev_link_status_flag) &&
	    (auto_nego_complete_flag ==
	     ogma_netdev_p->prev_auto_nego_complete_flag)) {
		/*
		 * No status change is detected since last call.
		 */
		return;
	}

	/*
	 * Configure GMAC if link is up and auto negotiation is complete.
	 */
	if ((link_status_flag != 0) && (auto_nego_complete_flag != 0)) {

		if ((ogma_err = ogma_netdev_configure_mac(ogma_netdev_p,
							  link_speed,
							  half_duplex_flag))
		    != OGMA_ERR_OK) {
			OGMA_MSG_ERR("ogma_netdev_configure_mac() failed");
			link_status_flag = auto_nego_complete_flag = 0;
		}
	}

	/*
	 * Notify Linux kernel of link status change.
	 */
	if (ogma_netdev_p->prev_link_status_flag != link_status_flag) {
		if (link_status_flag) {
			netif_carrier_on(ogma_netdev_p->netdev_p);
		} else {
			netif_carrier_off(ogma_netdev_p->netdev_p);
		}
	}

	/* Update saved PHY status */
	ogma_netdev_p->prev_link_status_flag = link_status_flag;
	ogma_netdev_p->prev_auto_nego_complete_flag = auto_nego_complete_flag;

}

static int ogma_netdev_set_mac_address(struct net_device *netdev_p, void *data)
{
	struct sockaddr *sockaddr_p = data;

	if (netif_running(netdev_p)) {
		return -EBUSY;
	}

	if (!is_valid_ether_addr(sockaddr_p->sa_data)) {
		return -EADDRNOTAVAIL;
	}

	memcpy(netdev_p->dev_addr, sockaddr_p->sa_data, netdev_p->addr_len);

	return 0;
}

static int ogma_ethtool_get_settings(struct net_device *netdev_p,
				     struct ethtool_cmd *cmd_p)
{
	int err = 0;
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	err = mii_ethtool_gset(&ogma_netdev_p->ogma_mii_if_info, cmd_p);

	return err;
}

static int ogma_ethtool_set_settings(struct net_device *netdev_p,
				     struct ethtool_cmd *cmd_p)
{
	int err = 0;
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	if (cmd_p->autoneg != AUTONEG_ENABLE) {

		OGMA_MSG_ERR("Autonegotiation Disable not support.");

		err = -EOPNOTSUPP;

		return err;
	}

	err = mii_ethtool_sset(&ogma_netdev_p->ogma_mii_if_info, cmd_p);

	return err;
}

static int ogma_get_sset_count(struct net_device *netdev_p, int stringset)
{

	if (stringset == ETH_SS_STATS) {

		return ARRAY_SIZE(ogma_gmac_mmc_reg_name);

	} else {

		return -EOPNOTSUPP;
	}
}

static void ogma_get_strings(struct net_device *netdev_p,
			     ogma_uint32 stringset, ogma_uint8 * data_p)
{

	if (stringset != ETH_SS_STATS) {

		return;
	}

	memcpy(data_p, ogma_gmac_mmc_reg_name, sizeof(ogma_gmac_mmc_reg_name));
	return;
}

static void ogma_get_ethtool_stats(struct net_device *netdev_p,
				   struct ethtool_stats *ethtool_stats_p,
				   ogma_uint64 * stats_data_p)
{
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	ogma_netdev_mmc_stats_read(ogma_netdev_p, stats_data_p, OGMA_FALSE);
	return;
}

static void ogma_netdev_mmc_stats_read(ogma_netdev_t * ogma_netdev_p,
				       ogma_uint64 * stats_data_p,
				       ogma_bool reset_flag)
{
	ogma_uint i;
	ogma_uint32 mmc_reg_num[ARRAY_SIZE(ogma_gmac_mmc_reg_name)] = { 0 };

	(void)ogma_read_gmac_stat(ogma_netdev_p->ogma_handle,
				  mmc_reg_num, reset_flag);
	if (reset_flag) {

		for (i = 0; i <= 3; i++) {
			ogma_gmac_mmc_reg_stats[i] = mmc_reg_num[i];
		}

		for (; i < ARRAY_SIZE(ogma_gmac_mmc_reg_name); i++) {
			ogma_gmac_mmc_reg_stats[i] += mmc_reg_num[i];
		}

		if (stats_data_p != NULL) {
			memcpy(stats_data_p,
			       ogma_gmac_mmc_reg_stats,
			       sizeof(ogma_gmac_mmc_reg_stats));
		}

	} else {

		BUG_ON(stats_data_p == NULL);

		for (i = 0; i <= 3; i++) {
			stats_data_p[i] = mmc_reg_num[i];
		}

		for (; i < ARRAY_SIZE(ogma_gmac_mmc_reg_name); i++) {
			stats_data_p[i] =
			    ogma_gmac_mmc_reg_stats[i] + mmc_reg_num[i];
		}
	}

}

static int ogma_netdev_nway_reset(struct net_device *netdev_p)
{
	int err = 0;
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	err = mii_nway_restart(&ogma_netdev_p->ogma_mii_if_info);

	return err;
}

int ogma_netdev_mii_read(struct net_device *netdev_p,
			 int phy_addr, int reg_addr)
{
	int value;
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	value = (int)ogma_get_phy_reg(ogma_netdev_p->ogma_handle,
				      (ogma_uint8) phy_addr,
				      (ogma_uint8) reg_addr);

	if (reg_addr == OGMA_PHY_REG_ADDR_SR) {

		if (((value & OGMA_PHY_SR_REG_LINK) == 0) &&
		    (ogma_netdev_p->prev_link_status_flag)) {

			OGMA_MSG_ERR("Link Down.");

			ogma_netdev_p->prev_link_status_flag = OGMA_FALSE;

			netif_stop_queue(ogma_netdev_p->netdev_p);

			netif_carrier_off(ogma_netdev_p->netdev_p);
		}
	}

	return value;

}

void ogma_netdev_mii_write(struct net_device *netdev_p,
			   int phy_addr, int reg_addr, int value)
{
	ogma_netdev_t *ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	if (value > 0xffff) {

		return;
	}

	ogma_set_phy_reg(ogma_netdev_p->ogma_handle,
			 (ogma_uint8) phy_addr,
			 (ogma_uint8) reg_addr, (ogma_uint16) value);
	return;
}

void ogma_netdev_set_mii_if_info(struct net_device *netdev_p,
				 ogma_netdev_t * ogma_netdev_p)
{
	ogma_netdev_p->ogma_mii_if_info.dev = netdev_p;

	ogma_netdev_p->ogma_mii_if_info.phy_id = phy_dev_addr;

	ogma_netdev_p->ogma_mii_if_info.phy_id_mask = 0x1f;

	ogma_netdev_p->ogma_mii_if_info.reg_num_mask = 0x1f;

	ogma_netdev_p->ogma_mii_if_info.mdio_read = ogma_netdev_mii_read;

	ogma_netdev_p->ogma_mii_if_info.mdio_write = ogma_netdev_mii_write;

	ogma_netdev_p->ogma_mii_if_info.supports_gmii = OGMA_TRUE;

	ogma_netdev_p->ogma_mii_if_info.advertising =
	    (ADVERTISE_10HALF | ADVERTISE_10FULL |
	     ADVERTISE_100HALF | ADVERTISE_100FULL | ADVERTISE_1000FULL);

}
