/**
 * ogma_reg_f_taiki.h
 *
 *  Copyright (c) 2012 - 2015 SOCIONEXT INCORPORATED.
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
 */
#ifndef OGMA_REG_F_TAIKI_H
#define OGMA_REG_F_TAIKI_H

#define OGMA_REG_ADDR_SOFT_RST                   (0x41)
#define OGMA_REG_ADDR_COM_INIT                   (0x48)

#define OGMA_REG_ADDR_DMAC_HM_CMD_BUF            (0x84)
#define OGMA_REG_ADDR_DMAC_MC_ADDR_MH            (0x89)
#define OGMA_REG_ADDR_DMAC_MC_SIZE_MH            (0x8a)
#define OGMA_REG_ADDR_PKTC_MC_ADDR               (0x5c)
#define OGMA_REG_ADDR_PKTC_MC_SIZE               (0x5d)
#define OGMA_REG_ADDR_DIS_CORE                   (0x86)

#define OGMA_REG_ADDR_DMA_HM_CTRL                (0x85)
#define OGMA_REG_ADDR_DMA_MH_CTRL                (0x88)

#define OGMA_REG_ADDR_CLK_EN_0                   (0x40)
#define OGMA_REG_ADDR_CLK_EN_1                   (0x64)
#define OGMA_REG_ADDR_PKT_CTRL                   (0x50)
#define OGMA_REG_ADDR_STRM_CTRL                  (0x42)

// for f_taiki driver
#define OGMA_REG_ADDR_CLK_EN                     (OGMA_REG_ADDR_CLK_EN_0)

#define OGMA_REG_ADDR_PKT_TX_DESC_START          (0x102)
#define OGMA_REG_ADDR_PKT_RX_DESC_START          (0x112)
#define OGMA_REG_ADDR_VDO_TX_DESC_START          (0x122)
#define OGMA_REG_ADDR_ETH_LPB_RX_DESC_START      (0x132)
#define OGMA_REG_ADDR_ADO_TX_DESC_START          (0x142)
#define OGMA_REG_ADDR_RTP_LPB_RX_DESC_START      (0x152)
#define OGMA_REG_ADDR_CDB_TX_DESC_START          (0x162)
#define OGMA_REG_ADDR_CRY_TX_DESC_START          (0x182)
#define OGMA_REG_ADDR_CRY_RX_DESC_START          (0x192)
#define OGMA_REG_ADDR_JPG1_TX_DESC_START         (0x1a2)
#define OGMA_REG_ADDR_JPG2_TX_DESC_START         (0x1b2)
#define OGMA_REG_ADDR_VSUB_TX_DESC_START         (0x1c2)
#define OGMA_REG_ADDR_MDT_TX_DESC_START          (0x1d2)
#define OGMA_REG_ADDR_TS_TX_DESC_START           (0x1e2)

// for f_taiki driver
#define OGMA_REG_ADDR_NRM_TX_DESC_START          (OGMA_REG_ADDR_PKT_TX_DESC_START)
#define OGMA_REG_ADDR_NRM_RX_DESC_START          (OGMA_REG_ADDR_PKT_RX_DESC_START)

#define OGMA_REG_ADDR_PKT_TX_CONFIG              (0x10c)
#define OGMA_REG_ADDR_PKT_RX_CONFIG              (0x11c)
#define OGMA_REG_ADDR_VDO_TX_CONFIG              (0x12c)
#define OGMA_REG_ADDR_ETH_LPB_RX_CONFIG          (0x13c)
#define OGMA_REG_ADDR_ADO_TX_CONFIG              (0x14c)
#define OGMA_REG_ADDR_RTP_LPB_RX_CONFIG          (0x15c)
#define OGMA_REG_ADDR_CDB_TX_CONFIG              (0x16c)
#define OGMA_REG_ADDR_CRY_TX_CONFIG              (0x18c)
#define OGMA_REG_ADDR_CRY_RX_CONFIG              (0x19c)
#define OGMA_REG_ADDR_JPG1_TX_CONFIG             (0x1ac)
#define OGMA_REG_ADDR_JPG2_TX_CONFIG             (0x1bc)
#define OGMA_REG_ADDR_VSUB_TX_CONFIG             (0x1cc)
#define OGMA_REG_ADDR_MDT_TX_CONFIG              (0x1dc)
#define OGMA_REG_ADDR_TS_TX_CONFIG               (0x1ec)

// for f_taiki driver
#define OGMA_REG_ADDR_NRM_TX_CONFIG              (OGMA_REG_ADDR_PKT_TX_CONFIG)
#define OGMA_REG_ADDR_NRM_RX_CONFIG              (OGMA_REG_ADDR_PKT_RX_CONFIG)

#define OGMA_REG_ADDR_DMA_TMR_CTRL               (0x83)

#define OGMA_REG_ADDR_TOP_STATUS                 (0x80)
#define OGMA_REG_ADDR_TOP_INTEN_A                (0x81)
#define OGMA_REG_ADDR_TOP_INTEN_A_SET            (0x8d)
#define OGMA_REG_ADDR_TOP_INTEN_A_CLR            (0x8e)
#define OGMA_REG_ADDR_TOP_INTEN_B                (0x8f)
#define OGMA_REG_ADDR_TOP_INTEN_B_SET            (0x90)
#define OGMA_REG_ADDR_TOP_INTEN_B_CLR            (0x91)

// for f_taiki driver
#define OGMA_REG_ADDR_TOP_INTEN                  (OGMA_REG_ADDR_TOP_INTEN_B)
#define OGMA_REG_ADDR_TOP_INTEN_SET              (OGMA_REG_ADDR_TOP_INTEN_B_SET)
#define OGMA_REG_ADDR_TOP_INTEN_CLR              (OGMA_REG_ADDR_TOP_INTEN_B_CLR)

#define OGMA_REG_ADDR_PKT_STATUS                 (0x03)
#define OGMA_REG_ADDR_PKT_INTEN                  (0x04)
#define OGMA_REG_ADDR_PKT_INTEN_SET              (0x07)
#define OGMA_REG_ADDR_PKT_INTEN_CLR              (0x08)

#define OGMA_REG_ADDR_STRM_STATUS                (0x05)
#define OGMA_REG_ADDR_STRM_INTEN                 (0x06)
#define OGMA_REG_ADDR_STRM_INTEN_SET             (0x09)
#define OGMA_REG_ADDR_STRM_INTEN_CLR             (0x0a)

#define OGMA_REG_ADDR_SUB_STATUS                 (0x0b)
#define OGMA_REG_ADDR_SUB_INTEN                  (0x0c)
#define OGMA_REG_ADDR_SUB_INTEN_SET              (0x0d)
#define OGMA_REG_ADDR_SUB_INTEN_CLR              (0x0e)

#define OGMA_REG_ADDR_MAC_STATUS                 (0x409)
#define OGMA_REG_ADDR_MAC_INTEN                  (0x40a)
#define OGMA_REG_ADDR_MAC_TX_RX_INFO_STATUS      (0x486)
#define OGMA_REG_ADDR_MAC_TX_RX_INFO_INTEN       (0x487)

#define OGMA_REG_ADDR_PKT_TX_STATUS              (0x100)
#define OGMA_REG_ADDR_PKT_TX_INTEN               (0x101)
#define OGMA_REG_ADDR_PKT_TX_INTEN_SET           (0x10a)
#define OGMA_REG_ADDR_PKT_TX_INTEN_CLR           (0x10b)
#define OGMA_REG_ADDR_PKT_RX_STATUS              (0x110)
#define OGMA_REG_ADDR_PKT_RX_INTEN               (0x111)
#define OGMA_REG_ADDR_PKT_RX_INTEN_SET           (0x11a)
#define OGMA_REG_ADDR_PKT_RX_INTEN_CLR           (0x11b)
#define OGMA_REG_ADDR_VDO_TX_STATUS              (0x120)
#define OGMA_REG_ADDR_VDO_TX_INTEN               (0x121)
#define OGMA_REG_ADDR_VDO_TX_INTEN_SET           (0x12a)
#define OGMA_REG_ADDR_VDO_TX_INTEN_CLR           (0x12b)
#define OGMA_REG_ADDR_ETH_LPB_RX_STATUS          (0x130)
#define OGMA_REG_ADDR_ETH_LPB_RX_INTEN           (0x131)
#define OGMA_REG_ADDR_ETH_LPB_RX_INTEN_SET       (0x13a)
#define OGMA_REG_ADDR_ETH_LPB_RX_INTEN_CLR       (0x13b)
#define OGMA_REG_ADDR_ADO_TX_STATUS              (0x140)
#define OGMA_REG_ADDR_ADO_TX_INTEN               (0x141)
#define OGMA_REG_ADDR_ADO_TX_INTEN_SET           (0x14a)
#define OGMA_REG_ADDR_ADO_TX_INTEN_CLR           (0x14b)
#define OGMA_REG_ADDR_RTP_LPB_RX_STATUS          (0x150)
#define OGMA_REG_ADDR_RTP_LPB_RX_INTEN           (0x151)
#define OGMA_REG_ADDR_RTP_LPB_RX_INTEN_SET       (0x15a)
#define OGMA_REG_ADDR_RTP_LPB_RX_INTEN_CLR       (0x15b)
#define OGMA_REG_ADDR_CDB_TX_STATUS              (0x160)
#define OGMA_REG_ADDR_CDB_TX_INTEN               (0x161)
#define OGMA_REG_ADDR_CDB_TX_INTEN_SET           (0x16a)
#define OGMA_REG_ADDR_CDB_TX_INTEN_CLR           (0x16b)
#define OGMA_REG_ADDR_CRY_TX_STATUS              (0x180)
#define OGMA_REG_ADDR_CRY_TX_INTEN               (0x181)
#define OGMA_REG_ADDR_CRY_TX_INTEN_SET           (0x18a)
#define OGMA_REG_ADDR_CRY_TX_INTEN_CLR           (0x18b)
#define OGMA_REG_ADDR_CRY_RX_STATUS              (0x190)
#define OGMA_REG_ADDR_CRY_RX_INTEN               (0x191)
#define OGMA_REG_ADDR_CRY_RX_INTEN_SET           (0x19a)
#define OGMA_REG_ADDR_CRY_RX_INTEN_CLR           (0x19b)
#define OGMA_REG_ADDR_JPG1_TX_STATUS             (0x1a0)
#define OGMA_REG_ADDR_JPG1_TX_INTEN              (0x1a1)
#define OGMA_REG_ADDR_JPG1_TX_INTEN_SET          (0x1aa)
#define OGMA_REG_ADDR_JPG1_TX_INTEN_CLR          (0x1ab)
#define OGMA_REG_ADDR_JPG2_TX_STATUS             (0x1b0)
#define OGMA_REG_ADDR_JPG2_TX_INTEN              (0x1b1)
#define OGMA_REG_ADDR_JPG2_TX_INTEN_SET          (0x1ba)
#define OGMA_REG_ADDR_JPG2_TX_INTEN_CLR          (0x1bb)
#define OGMA_REG_ADDR_VSUB_TX_STATUS             (0x1c0)
#define OGMA_REG_ADDR_VSUB_TX_INTEN              (0x1c1)
#define OGMA_REG_ADDR_VSUB_TX_INTEN_SET          (0x1ca)
#define OGMA_REG_ADDR_VSUB_TX_INTEN_CLR          (0x1cb)
#define OGMA_REG_ADDR_MDT_TX_STATUS              (0x1d0)
#define OGMA_REG_ADDR_MDT_TX_INTEN               (0x1d1)
#define OGMA_REG_ADDR_MDT_TX_INTEN_SET           (0x1da)
#define OGMA_REG_ADDR_MDT_TX_INTEN_CLR           (0x1db)
#define OGMA_REG_ADDR_TS_TX_STATUS               (0x1e0)
#define OGMA_REG_ADDR_TS_TX_INTEN                (0x1e1)
#define OGMA_REG_ADDR_TS_TX_INTEN_SET            (0x1ea)
#define OGMA_REG_ADDR_TS_TX_INTEN_CLR            (0x1eb)

// for f_taiki driver
#define OGMA_REG_ADDR_NRM_TX_STATUS              (OGMA_REG_ADDR_PKT_TX_STATUS)
#define OGMA_REG_ADDR_NRM_TX_INTEN               (OGMA_REG_ADDR_PKT_TX_INTEN)
#define OGMA_REG_ADDR_NRM_TX_INTEN_SET           (OGMA_REG_ADDR_PKT_TX_INTEN_SET)
#define OGMA_REG_ADDR_NRM_TX_INTEN_CLR           (OGMA_REG_ADDR_PKT_TX_INTEN_CLR)
#define OGMA_REG_ADDR_NRM_RX_STATUS              (OGMA_REG_ADDR_PKT_RX_STATUS)
#define OGMA_REG_ADDR_NRM_RX_INTEN               (OGMA_REG_ADDR_PKT_RX_INTEN)
#define OGMA_REG_ADDR_NRM_RX_INTEN_SET           (OGMA_REG_ADDR_PKT_RX_INTEN_SET)
#define OGMA_REG_ADDR_NRM_RX_INTEN_CLR           (OGMA_REG_ADDR_PKT_RX_INTEN_CLR)

#define OGMA_REG_ADDR_PKT_TX_CNT                 (0x104)
#define OGMA_REG_ADDR_VDO_TX_CNT                 (0x124)
#define OGMA_REG_ADDR_ADO_TX_CNT                 (0x144)
#define OGMA_REG_ADDR_CDB_TX_CNT                 (0x164)
#define OGMA_REG_ADDR_CRY_TX_CNT                 (0x184)
#define OGMA_REG_ADDR_JPG1_TX_CNT                (0x1a4)
#define OGMA_REG_ADDR_JPG2_TX_CNT                (0x1b4)
#define OGMA_REG_ADDR_VSUB_TX_CNT                (0x1c4)
#define OGMA_REG_ADDR_MDT_TX_CNT                 (0x1d4)
#define OGMA_REG_ADDR_TS_TX_CNT                  (0x1e4)

#define OGMA_REG_ADDR_PKT_TX_DONE_PKTCNT         (0x105)
#define OGMA_REG_ADDR_VDO_TX_DONE_CNT            (0x125)
#define OGMA_REG_ADDR_ADO_TX_DONE_CNT            (0x145)
#define OGMA_REG_ADDR_CDB_TX_DONE_CNT            (0x165)
#define OGMA_REG_ADDR_CRY_TX_DONE_CNT            (0x185)
#define OGMA_REG_ADDR_JPG1_TX_DONE_CNT           (0x1a5)
#define OGMA_REG_ADDR_JPG2_TX_DONE_CNT           (0x1b5)
#define OGMA_REG_ADDR_VSUB_TX_DONE_CNT           (0x1c5)
#define OGMA_REG_ADDR_MDT_TX_DONE_CNT            (0x1d5)
#define OGMA_REG_ADDR_TS_TX_DONE_CNT             (0x1e5)

#define OGMA_REG_ADDR_PKT_TX_DONE_TXINT_PKTCNT   (0x106)
#define OGMA_REG_ADDR_VDO_TX_DONE_TXINT_CNT      (0x126)
#define OGMA_REG_ADDR_ADO_TX_DONE_TXINT_CNT      (0x146)
#define OGMA_REG_ADDR_CDB_TX_DONE_TXINT_CNT      (0x166)
#define OGMA_REG_ADDR_CRY_TX_DONE_TXINT_CNT      (0x186)
#define OGMA_REG_ADDR_JPG1_TX_DONE_TXINT_CNT     (0x1a6)
#define OGMA_REG_ADDR_JPG2_TX_DONE_TXINT_CNT     (0x1b6)
#define OGMA_REG_ADDR_VSUB_TX_DONE_TXINT_CNT     (0x1c6)
#define OGMA_REG_ADDR_MDT_TX_DONE_TXINT_CNT      (0x1d6)
#define OGMA_REG_ADDR_TS_TX_DONE_TXINT_CNT       (0x1e6)

#define OGMA_REG_ADDR_PKT_TX_TMR                 (0x107)

#define OGMA_REG_ADDR_PKT_TX_TXINT_TMR           (0x108)

#define OGMA_REG_ADDR_PKT_RX_PKTCNT              (0x115)
#define OGMA_REG_ADDR_ETH_LPB_RX_PKTCNT          (0x135)
#define OGMA_REG_ADDR_RTP_LPB_RX_CNT             (0x155)
#define OGMA_REG_ADDR_CRY_RX_CNT                 (0x195)

#define OGMA_REG_ADDR_PKT_RX_RXINT_PKTCNT        (0x116)
#define OGMA_REG_ADDR_ETH_LPB_RX_RXINT_CNT       (0x136)
#define OGMA_REG_ADDR_RTP_LPB_RX_RXINT_CNT       (0x156)
#define OGMA_REG_ADDR_CRY_RX_RXINT_CNT           (0x196)

#define OGMA_REG_ADDR_PKT_RX_TMR                 (0x117)

#define OGMA_REG_ADDR_PKT_RX_RXINT_TMR           (0x118)

// for f_taiki driver
#define OGMA_REG_ADDR_NRM_TX_PKTCNT              (OGMA_REG_ADDR_PKT_TX_CNT)
#define OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT   (OGMA_REG_ADDR_PKT_TX_DONE_TXINT_PKTCNT)
#define OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT        (OGMA_REG_ADDR_PKT_RX_RXINT_PKTCNT)
#define OGMA_REG_ADDR_NRM_TX_TXINT_TMR           (OGMA_REG_ADDR_PKT_TX_TXINT_TMR)
#define OGMA_REG_ADDR_NRM_RX_RXINT_TMR           (OGMA_REG_ADDR_PKT_RX_RXINT_TMR)
#define OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT         (OGMA_REG_ADDR_PKT_TX_DONE_PKTCNT)
#define OGMA_REG_ADDR_NRM_RX_PKTCNT              (OGMA_REG_ADDR_PKT_RX_PKTCNT)
#define OGMA_REG_ADDR_NRM_TX_TMR                 (OGMA_REG_ADDR_PKT_TX_TMR)
#define OGMA_REG_ADDR_NRM_RX_TMR                 (OGMA_REG_ADDR_PKT_RX_TMR)

#define OGMA_REG_ADDR_SDB_CMD_ST                 (0x74)
#define OGMA_REG_ADDR_SDB_DATA                   (0x75)

#define OGMA_REG_ADDR_MAC_CMD                    (0x471)
#define OGMA_REG_ADDR_MAC_DATA                   (0x470)
#define OGMA_REG_ADDR_MAC_INTF_SEL               (0x475)
#define OGMA_REG_ADDR_MAC_DESC_INIT              (0x47f)
#define OGMA_REG_ADDR_MAC_DESC_SOFT_RST          (0x481)
#define OGMA_REG_ADDR_MAC_FLOW_TH                (0x473)

#define OGMA_REG_ADDR_IV_INIT_VAL                (0x45)

#define OGMA_REG_ADDR_CNT90K_32                  (0x483)
#define OGMA_REG_ADDR_CNT_ADO_32                 (0x488)
#define OGMA_REG_ADDR_CNT27M_U32                 (0x489)
#define OGMA_REG_ADDR_CNT27M_L32                 (0x48a)
#define OGMA_REG_ADDR_CNT_EN                     (0x484)
#define OGMA_REG_ADDR_CNT_ADO_DIV                (0x485)

#define OGMA_REG_ADDR_MUTEX_ST                   (0x65)
#define OGMA_REG_ADDR_MUTEX_REQ_0                (0x66)
#define OGMA_REG_ADDR_MUTEX_REQ_1                (0x67)

#define OGMA_REG_ADDR_F_NETSEC_TS_MC_VER         (0x8b)
#define OGMA_REG_ADDR_F_NETSEC_TS_VER            (0x8c)

#define OGMA_REG_ADDR_PKTC_CMD_BUF               (0x34)
#define OGMA_REG_ADDR_DMAC_MH_CMD_BUF            (0x87)

#define OGMA_REG_ADDR_F_TAIKI_MC_VER             (OGMA_REG_ADDR_F_NETSEC_TS_MC_VER)
#define OGMA_REG_ADDR_F_TAIKI_VER                (OGMA_REG_ADDR_F_NETSEC_TS_VER)

#define OGMA_REG_ADDR_RESERVED_RX_DESC_START     (0x122)
#define OGMA_REG_ADDR_RESERVED_TX_DESC_START     (0x132)

#endif				/* OGMA_REG_F_TAIKI_H */
