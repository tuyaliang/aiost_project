/**
 * ogma_gmac_access.c
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
#include "ogma_config.h"

#include "ogma_internal.h"
#include "ogma_basic_access.h"

/**********************************************************************
 * Constant definitions
 **********************************************************************/

/**
 * Clock range index for F_GMAC4MT::GAR::CR field.
 */
#if (OGMA_CONFIG_GMAC_CLK_HZ < 35 * OGMA_CLK_MHZ)
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_25_35_MHZ
#elif (OGMA_CONFIG_GMAC_CLK_HZ < 60 * OGMA_CLK_MHZ)
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_35_60_MHZ
#elif (OGMA_CONFIG_GMAC_CLK_HZ < 100 * OGMA_CLK_MHZ)
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_60_100_MHZ
#elif (OGMA_CONFIG_GMAC_CLK_HZ < 150 * OGMA_CLK_MHZ)
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_100_150_MHZ
#elif (OGMA_CONFIG_GMAC_CLK_HZ < 250 * OGMA_CLK_MHZ)
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_150_250_MHZ
#else
#define OGMA_CLOCK_RANGE_IDX OGMA_GMAC_GAR_REG_CR_250_300_MHZ
#endif

/**********************************************************************
 * Local function declarations
 **********************************************************************/

#ifdef OGMA_CONFIG_USE_DUMP_GMAC_STAT
static void ogma_dump_gmac_stat(ogma_ctrl_t * ctrl_p);
#endif				/*  OGMA_CONFIG_USE_DUMP_GMAC_STAT */

/**********************************************************************
 * Function definitions
 **********************************************************************/

ogma_err_t ogma_start_gmac(ogma_handle_t ogma_handle,
			   ogma_bool rx_flag, ogma_bool tx_flag)
{
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->param.use_gmac_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if (!ctrl_p->gmac_mode_valid_flag) {
		return OGMA_ERR_INVALID;
	}

	if ((!rx_flag) && (!tx_flag)) {
		return OGMA_ERR_OK;
	}

	if (ctrl_p->gmac_rx_running_flag && ctrl_p->gmac_tx_running_flag) {
		return OGMA_ERR_OK;
	}

	if ((rx_flag && ctrl_p->gmac_rx_running_flag) && !tx_flag) {
		return OGMA_ERR_OK;
	}

	if ((tx_flag && ctrl_p->gmac_tx_running_flag) && !rx_flag) {
		return OGMA_ERR_OK;
	}

	/*push domain c clock */
	ogma_push_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

	/* pop domain c clock */
	ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

	return OGMA_ERR_OK;

}

ogma_err_t ogma_stop_gmac(ogma_handle_t ogma_handle,
			  ogma_bool rx_flag, ogma_bool tx_flag)
{
	ogma_uint32 value;
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->param.use_gmac_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if ((!rx_flag) && (!tx_flag)) {
		return OGMA_ERR_OK;
	}

	if ((rx_flag && ctrl_p->gmac_rx_running_flag) ||
	    (tx_flag && ctrl_p->gmac_tx_running_flag)) {
		/* Read F_GMAC4MT OMR */
		value = ogma_get_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_OMR);

		if (rx_flag && ctrl_p->gmac_rx_running_flag) {
			value &= (~OGMA_GMAC_OMR_REG_SR);
			ctrl_p->gmac_rx_running_flag = OGMA_FALSE;
		}

		if (tx_flag && ctrl_p->gmac_tx_running_flag) {
			value &= (~OGMA_GMAC_OMR_REG_ST);
			ctrl_p->gmac_tx_running_flag = OGMA_FALSE;
		}

		/* set F_GMAC4MT OMR */
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_OMR, value);
	}

	return OGMA_ERR_OK;

}

ogma_err_t ogma_set_gmac_mode(ogma_handle_t ogma_handle,
			      const ogma_gmac_mode_t * gmac_mode_p)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if ((ctrl_p == NULL) || (gmac_mode_p == NULL)) {
		return OGMA_ERR_PARAM;
	}

	if (!ctrl_p->param.use_gmac_flag) {
		return OGMA_ERR_NOTAVAIL;
	}

	if ((ctrl_p->gmac_rx_running_flag) || (ctrl_p->gmac_tx_running_flag)) {
		return OGMA_ERR_BUSY;
	}

	if ((gmac_mode_p->link_speed != OGMA_PHY_LINK_SPEED_1G) &&
	    (gmac_mode_p->link_speed != OGMA_PHY_LINK_SPEED_100M) &&
	    (gmac_mode_p->link_speed != OGMA_PHY_LINK_SPEED_10M)) {
		return OGMA_ERR_DATA;
	}

	if ((gmac_mode_p->link_speed == OGMA_PHY_LINK_SPEED_1G) &&
	    (gmac_mode_p->half_duplex_flag)) {
		return OGMA_ERR_DATA;
	}

	if (gmac_mode_p->half_duplex_flag && gmac_mode_p->flow_ctrl_enable_flag) {
		return OGMA_ERR_DATA;
	}

	if (gmac_mode_p->flow_ctrl_enable_flag) {

		if ((gmac_mode_p->flow_ctrl_start_threshold == 0) ||
		    (gmac_mode_p->flow_ctrl_start_threshold >
		     OGMA_FLOW_CTRL_START_THRESHOLD_MAX)) {
			return OGMA_ERR_DATA;
		}

		if ((gmac_mode_p->flow_ctrl_stop_threshold <
		     gmac_mode_p->flow_ctrl_start_threshold) ||
		    (gmac_mode_p->flow_ctrl_stop_threshold >
		     OGMA_FLOW_CTRL_STOP_THRESHOLD_MAX)) {
			return OGMA_ERR_DATA;
		}

		if (gmac_mode_p->pause_time < OGMA_FLOW_CTRL_PAUSE_TIME_MIN) {
			return OGMA_ERR_DATA;
		}
	}

	pfdep_memcpy((void *)&ctrl_p->gmac_mode,
		     (void *)gmac_mode_p, sizeof(ogma_gmac_mode_t));

	ctrl_p->gmac_mode_valid_flag = OGMA_TRUE;

	return OGMA_ERR_OK;
}

void ogma_set_phy_reg(ogma_handle_t ogma_handle,
		      ogma_uint8 phy_addr,
		      ogma_uint8 reg_addr, ogma_uint16 value)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 cmd;

	if ((ctrl_p == NULL)
	    || (!ctrl_p->param.use_gmac_flag)
	    || (phy_addr >= 32)
	    || (reg_addr >= 32)) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at ogma_set_phy_reg.\nPlease set valid argument.\n");
		return;
	}

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GDR, value);

	cmd = ((phy_addr << OGMA_GMAC_GAR_REG_SHIFT_PA)
	       | (reg_addr << OGMA_GMAC_GAR_REG_SHIFT_GR)
	       | (OGMA_CLOCK_RANGE_IDX << OGMA_GMAC_GAR_REG_SHIFT_CR)
	       | OGMA_GMAC_GAR_REG_GW | OGMA_GMAC_GAR_REG_GB);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GAR, cmd);

	while ((ogma_get_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GAR) &
		OGMA_GMAC_GAR_REG_GB)
	       != 0) {
		;
	}

}

ogma_uint16 ogma_get_phy_reg(ogma_handle_t ogma_handle,
			     ogma_uint8 phy_addr, ogma_uint8 reg_addr)
{

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint32 cmd;

	if ((ctrl_p == NULL)
	    || (!ctrl_p->param.use_gmac_flag)
	    || (phy_addr >= 32)
	    || (reg_addr >= 32)) {
		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "ogma_get_phy_reg: ctrl_p=%p use_gmac_flag=%d, phy_addr=%d, reg_addr=%d\n",
			    ctrl_p, ctrl_p->param.use_gmac_flag, phy_addr,
			    reg_addr);
		return 0;
	}

	cmd = ((phy_addr << OGMA_GMAC_GAR_REG_SHIFT_PA)
	       | (reg_addr << OGMA_GMAC_GAR_REG_SHIFT_GR)
	       | (OGMA_CLOCK_RANGE_IDX << OGMA_GMAC_GAR_REG_SHIFT_CR)
	       | OGMA_GMAC_GAR_REG_GB);

	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GAR, cmd);

	while ((ogma_get_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GAR) &
		OGMA_GMAC_GAR_REG_GB)
	       != 0) {
		;
	}

	return (ogma_uint16) ogma_get_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_GDR);

}

static const struct {
	ogma_uint32 addr;
	ogma_char *name_p;
} ogma_gmac_mmc_reg_info[] = {
	{
	OGMA_GMAC_REG_ADDR_MMC_INTR_RX, "MMC_INTR_RX"}, {
	OGMA_GMAC_REG_ADDR_MMC_INTR_TX, "MMC_INTR_TX"}, {
	OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_RX, "MMC_INTR_MASK_RX"}, {
	OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_TX, "MMC_INTR_MASK_TX"}, {
	OGMA_GMAC_REG_ADDR_TXOCTETCOUNT_GB, "TXOCTETCOUNT_GB"}, {
	OGMA_GMAC_REG_ADDR_TXFRAMECOUNT_GB, "TXFRAMECOUNT_GB"}, {
	OGMA_GMAC_REG_ADDR_TXBROADCASTFRAMES_G, "TXBROADCASTFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_TXMULTICASTFRAMES_G, "TXMULTICASTFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_TX64OCTETS_GB, "TX64OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TX65TO127OCTETS_GB, "TX65TO127OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TX128TO255OCTETS_GB, "TX128TO255OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TX256TO511OCTETS_GB, "TX256TO511OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TX512TO1023OCTETS_GB, "TX512TO1023OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TX1024TOMAXOCTETS_GB, "TX1024TOMAXOCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_TXUNICASTFRAMES_GB, "TXUNICASTFRAMES_GB"}, {
	OGMA_GMAC_REG_ADDR_TXMULTICASTFRAMES_GB, "TXMULTICASTFRAMES_GB"}, {
	OGMA_GMAC_REG_ADDR_TXBROADCASTFRAMES_GB, "TXBROADCASTFRAMES_GB"}, {
	OGMA_GMAC_REG_ADDR_TXUNDERFLOWERROR, "TXUNDERFLOWERROR"}, {
	OGMA_GMAC_REG_ADDR_TXSINGLECOL_G, "TXSINGLECOL_G"}, {
	OGMA_GMAC_REG_ADDR_TXMULTICOL_G, "TXMULTICOL_G"}, {
	OGMA_GMAC_REG_ADDR_TXDEFERRED, "TXDEFERRED"}, {
	OGMA_GMAC_REG_ADDR_TXLATECOL, "TXLATECOL"}, {
	OGMA_GMAC_REG_ADDR_TXEXESSCOL, "TXEXESSCOL"}, {
	OGMA_GMAC_REG_ADDR_TXCARRIERERRROR, "TXCARRIERERRROR"}, {
	OGMA_GMAC_REG_ADDR_TXOCTETCOUNT_G, "TXOCTETCOUNT_G"}, {
	OGMA_GMAC_REG_ADDR_TXFRAMECOUNT_G, "TXFRAMECOUNT_G"}, {
	OGMA_GMAC_REG_ADDR_TXEXECESSDEF, "TXEXECESSDEF"}, {
	OGMA_GMAC_REG_ADDR_TXPAUSEFRAMES, "TXPAUSEFRAMES"}, {
	OGMA_GMAC_REG_ADDR_TXVLANFRAMES_G, "TXVLANFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_RXFRAMECOUNT_GB, "RXFRAMECOUNT_GB"}, {
	OGMA_GMAC_REG_ADDR_RXOCTETCOUNT_GB, "RXOCTETCOUNT_GB"}, {
	OGMA_GMAC_REG_ADDR_RXOCTETCOUNT_G, "RXOCTETCOUNT_G"}, {
	OGMA_GMAC_REG_ADDR_RXBROADCASTFRAMES_G, "RXBROADCASTFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_RXMULTICASTFRAMES_G, "RXMULTICASTFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_RXCRCERROR, "RXCRCERROR"}, {
	OGMA_GMAC_REG_ADDR_RXALLIGNMENTERROR, "RXALLIGNMENTERROR"}, {
	OGMA_GMAC_REG_ADDR_RXRUNTERROR, "RXRUNTERROR"}, {
	OGMA_GMAC_REG_ADDR_RXJABBERERROR, "RXJABBERERROR"}, {
	OGMA_GMAC_REG_ADDR_RXUNDERSIZE_G, "RXUNDERSIZE_G"}, {
	OGMA_GMAC_REG_ADDR_RXOVERSIZE_G, "RXOVERSIZE_G"}, {
	OGMA_GMAC_REG_ADDR_RX64OCTETS_GB, "RX64OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RX65TO127OCTETS_GB, "RX65TO127OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RX128TO255OCTETS_GB, "RX128TO255OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RX256TO511OCTETS_GB, "RX256TO511OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RX512TO1023OCTETS_GB, "RX512TO1023OCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RX1024TOMAXOCTETS_GB, "RX1024TOMAXOCTETS_GB"}, {
	OGMA_GMAC_REG_ADDR_RXUNICASTFRAMES_G, "RXUNICASTFRAMES_G"}, {
	OGMA_GMAC_REG_ADDR_RXLENGTHERROR, "RXLENGTHERROR"}, {
	OGMA_GMAC_REG_ADDR_RXOUTOFRANGETYPE, "RXOUTOFRANGETYPE"}, {
	OGMA_GMAC_REG_ADDR_RXPAUSEFRAMES, "RXPAUSEFRAMES"}, {
	OGMA_GMAC_REG_ADDR_RXFIFOOVERFLOW, "RXFIFOOVERFLOW"}, {
	OGMA_GMAC_REG_ADDR_RXVLANFRAMES_GB, "RXVLANFRAMES_GB"}, {
	OGMA_GMAC_REG_ADDR_RXWATCHDOGERROR, "RXWATCHDOGERROR"}, {
	OGMA_GMAC_REG_ADDR_MMC_IPC_INTR_MASK_RX, "MMC_IPC_INTR_MASK_RX"}, {
	OGMA_GMAC_REG_ADDR_MMC_IPC_INTR_RX, "MMC_IPC_INTR_RX"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_GD_FRMS, "RXIPV4_GD_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_HDRERR_FRMS, "RXIPV4_HDRERR_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_NOPAY_FRMS, "RXIPV4_NOPAY_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_FRAG_FRMS, "RXIPV4_FRAG_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_UDSBL_FRMS, "RXIPV4_UDSBL_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_GD_FRMS, "RXIPV6_GD_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_HDRERR_FRMS, "RXIPV6_HDRERR_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_NOPAY_FRMS, "RXIPV6_NOPAY_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXUDP_GD_FRMS, "RXUDP_GD_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXUDP_ERR_FRMS, "RXUDP_ERR_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXTCP_GD_FRMS, "RXTCP_GD_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXTCP_ERR_FRMS, "RXTCP_ERR_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXICMP_GD_FRMS, "RXICMP_GD_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXICMP_ERR_FRMS, "RXICMP_ERR_FRMS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_GD_OCTETS, "RXIPV4_GD_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_HDRERR_OCTETS, "RXIPV4_HDRERR_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_NOPAY_OCTETS, "RXIPV4_NOPAY_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_FRAG_OCTETS, "RXIPV4_FRAG_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV4_UDSBL_OCTETS, "RXIPV4_UDSBL_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_GD_OCTETS, "RXIPV6_GD_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_HDRERR_OCTETS, "RXIPV6_HDRERR_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXIPV6_NOPAY_OCTETS, "RXIPV6_NOPAY_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXUDP_GD_OCTETS, "RXUDP_GD_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXUDP_ERR_OCTETS, "RXUDP_ERR_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXTCP_GD_OCTETS, "RXTCP_GD_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXTCP_ERR_OCTETS, "RXTCP_ERR_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXICMP_GD_OCTETS, "RXICMP_GD_OCTETS"}, {
	OGMA_GMAC_REG_ADDR_RXICMP_ERR_OCTETS, "RXICMP_ERR_OCTETS"}
};

void ogma_dump_gmac_stat(ogma_handle_t ogma_handle)
{
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;
	ogma_uint i;

	pfdep_print(PFDEP_DEBUG_LEVEL_NOTICE,
		    "Dumping GMAC statistics registers(MMC registers):\n");

	for (i = 0;
	     i <
	     sizeof(ogma_gmac_mmc_reg_info) / sizeof(ogma_gmac_mmc_reg_info[0]);
	     i++) {
		pfdep_print(PFDEP_DEBUG_LEVEL_NOTICE, "  %-21s => 0x%08lx\n",
			    ogma_gmac_mmc_reg_info[i].name_p,
			    (unsigned long)ogma_get_mac_reg(ctrl_p,
							    ogma_gmac_mmc_reg_info
							    [i].addr));
	}

	/* Reset all counters. */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_CNTL, 1);

}

ogma_err_t ogma_read_gmac_stat(ogma_handle_t ogma_handle,
			       ogma_uint32 * value_p, ogma_bool reset_flag)
{

	ogma_uint i;

	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if ((ctrl_p == NULL) || (value_p == NULL)) {
		return OGMA_ERR_PARAM;
	}
#ifdef OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK

	/* push domain c clock */
	ogma_push_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

	if (ogma_gmac_hw_ctrl_grant_status_check(ctrl_p) != OGMA_ERR_OK) {

		/* pop domain c clock */
		ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at %s.\nhw ctrl grant error.\n",
			    __func__);
		return OGMA_ERR_NO_AUTH;
	}
#endif				/* OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK */

	for (i = 0;
	     i <
	     sizeof(ogma_gmac_mmc_reg_info) / sizeof(ogma_gmac_mmc_reg_info[0]);
	     i++) {

		value_p[i] = ogma_get_mac_reg(ctrl_p,
					      ogma_gmac_mmc_reg_info[i].addr);
	}

	if (reset_flag) {
		/* Reset all counters. */
		ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_CNTL, 1);
	}
#ifdef OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK
	/* pop domain c clock */
	ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);
#endif

	return OGMA_ERR_OK;
}

ogma_err_t ogma_reset_gmac_stat(ogma_handle_t ogma_handle)
{
	ogma_ctrl_t *ctrl_p = (ogma_ctrl_t *) ogma_handle;

	if (ctrl_p == NULL) {
		return OGMA_ERR_PARAM;
	}
#ifdef OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK

	/* push domain c clock */
	ogma_push_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

	if (ogma_gmac_hw_ctrl_grant_status_check(ctrl_p) != OGMA_ERR_OK) {

		/* pop domain c clock */
		ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);

		pfdep_print(PFDEP_DEBUG_LEVEL_FATAL,
			    "An error occurred at %s.\nhw ctrl grant error.\n",
			    __func__);
		return OGMA_ERR_NO_AUTH;
	}
#endif				/* OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK */

	/* Reset all counters. */
	ogma_set_mac_reg(ctrl_p, OGMA_GMAC_REG_ADDR_MMC_CNTL, 1);

#ifdef OGMA_CONFIG_USE_HW_CTRL_GRANT_STATUS_CHECK
	/* pop domain c clock */
	ogma_pop_clk_req(ctrl_p, OGMA_CLK_EN_REG_DOM_C);
#endif

	return OGMA_ERR_OK;
}
