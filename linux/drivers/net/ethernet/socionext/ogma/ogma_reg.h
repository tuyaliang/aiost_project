/**
 * ogma_reg.h
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
#ifndef OGMA_REG_H
#define OGMA_REG_H

#include "ogma_reg_f_taiki.h"

#include "ogma_reg_f_gmac_4mt.h"

/*bit fields for PKT_CTRL*/
#define OGMA_PKT_CTRL_REG_MODE_NRM          (1UL << 28)
#define OGMA_PKT_CTRL_REG_EN_JUMBO          (1UL << 27)
#define OGMA_PKT_CTRL_REG_LOG_CHKSUM_ER     (1UL << 3 )
#define OGMA_PKT_CTRL_REG_LOG_HD_INCOMPLETE (1UL << 2 )
#define OGMA_PKT_CTRL_REG_LOG_HD_ER         (1UL << 1 )

#define OGMA_CLK_EN_REG_DOM_G (1UL << 5)
#define OGMA_CLK_EN_REG_DOM_C (1UL << 1)
#define OGMA_CLK_EN_REG_DOM_D (1UL << 0)

/************************************************************
 * Bit fields
 ************************************************************/
/* bit fields for com_init */
#define OGMA_COM_INIT_REG_PKT  (1UL << 1)
#define OGMA_COM_INIT_REG_CORE (1UL << 0)
#define OGMA_COM_INIT_REG_ALL  ( OGMA_COM_INIT_REG_CORE | OGMA_COM_INIT_REG_PKT)

/* bit fields for soft_rst */
#define OGMA_SOFT_RST_REG_RESET (0)
#define OGMA_SOFT_RST_REG_RUN   (1UL << 31)

/* bit fields for dma_hm_ctrl */
#define OGMA_DMA_CTRL_REG_STOP 1UL

/* bit fields for gmac_cmd */
#define OGMA_GMAC_CMD_ST_READ  (0)
#define OGMA_GMAC_CMD_ST_WRITE (1UL << 28)
#define OGMA_GMAC_CMD_ST_BUSY  (1UL << 31)

/* bit fields for F_GMAC4MT BMR*/
#define OGMA_GMAC_BMR_REG_COMMON (0x00412080)
#define OGMA_GMAC_BMR_REG_RESET  (0x00020181)
#define OGMA_GMAC_BMR_REG_SWR    (0x00000001)

/* bit fields for F_GMAC4MT OMR*/
#define OGMA_GMAC_OMR_REG_ST (1UL << 13)
#define OGMA_GMAC_OMR_REG_SR (1UL << 1)

/* bit fields for F_GMAC4MT MCR*/
#define OGMA_GMAC_MCR_REG_CST                (1UL << 25)
#define OGMA_GMAC_MCR_REG_JE                 (1UL << 20)
#define OGMA_GMAC_MCR_REG_PS                 (1UL << 15)
#define OGMA_GMAC_MCR_REG_FES                (1UL << 14)
#define OGMA_GMAC_MCR_REG_FULL_DUPLEX_COMMON (0x0000280c)
#define OGMA_GMAC_MCR_REG_HALF_DUPLEX_COMMON (0x0001a00c)

/*bit fields for F_GMAC4MT FCR*/
#define OGMA_GMAC_FCR_REG_RFE (1UL << 2)
#define OGMA_GMAC_FCR_REG_TFE (1UL << 1)

/* bit fields for F_GMAC4MT GAR */
#define OGMA_GMAC_GAR_REG_GW (1UL << 1)
#define OGMA_GMAC_GAR_REG_GB (1UL << 0)

/* bit fields for F_GMAC4MT RDLAR*/
#define OGMA_GMAC_RDLAR_REG_COMMON (0x00008000UL)

/* bit fields for F_GMAC4MT TDLAR*/
#define OGMA_GMAC_TDLAR_REG_COMMON (0x0000c000UL)

#define OGMA_GMAC_GAR_REG_SHIFT_PA (11)
#define OGMA_GMAC_GAR_REG_SHIFT_GR (6)
#define OGMA_GMAC_GAR_REG_SHIFT_CR (2)

#define OGMA_GMAC_GAR_REG_CR_25_35_MHZ   (2)
#define OGMA_GMAC_GAR_REG_CR_35_60_MHZ   (3)
#define OGMA_GMAC_GAR_REG_CR_60_100_MHZ  (0)
#define OGMA_GMAC_GAR_REG_CR_100_150_MHZ (1)
#define OGMA_GMAC_GAR_REG_CR_150_250_MHZ (4)
#define OGMA_GMAC_GAR_REG_CR_250_300_MHZ (5)

#define OGMA_REG_ADDR_OGMA_VER_F_TAIKI    0x00060001UL

/* bit fields for DESC RING CONFIG */
#define OGMA_REG_DESC_RING_CONFIG_CFG_UP     (31)
#define OGMA_REG_DESC_RING_CONFIG_CH_RST     (30)
#define OGMA_REG_DESC_RING_CONFIG_TMR_MODE   (4)
#define OGMA_REG_DESC_RING_CONFIG_DAT_ENDIAN (0)

/* bit fields for mac_desc_soft_rst */
#define OGMA_MAC_DESC_SOFT_RST_SOFT_RST 1UL

/* bit fields for mac_desc_init */
#define OGMA_MAC_DESC_INIT_REG_INIT 1UL

#endif				/*OGMA_REG_H */
