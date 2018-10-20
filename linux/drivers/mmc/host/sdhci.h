/*
 *  linux/drivers/mmc/host/sdhci.h - Secure Digital Host Controller Interface driver
 *
 * Header file for Host Controller registers and I/O accessors.
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *  Copyright (C) 2014 Intel Corp, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef __SDHCI_HW_H
#define __SDHCI_HW_H

#include <linux/scatterlist.h>
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/io.h>

#include <linux/mmc/host.h>

/*
 * Controller registers
 */

#define SDHCI_DMA_ADDRESS	0x00	/* Host version less than 4.0 */
#define SDHCI_ARGUMENT2		SDHCI_DMA_ADDRESS
/*
 * 32bit Block Count register is only selected when Host Version Enable
 * is set to 4 and 16bit Block Count register is set to zero.
 */
#define SDHCI_32B_BLOCK_COUNT	SDHCI_DMA_ADDRESS /* Host version is 4.0 */

#define SDHCI_BLOCK_SIZE	0x04
#define  SDHCI_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

#define SDHCI_BLOCK_COUNT	0x06	/* 16bit Block Count */

#define SDHCI_ARGUMENT		0x08

#define SDHCI_TRANSFER_MODE	0x0C
#define  SDHCI_TRNS_DMA		0x01
#define  SDHCI_TRNS_BLK_CNT_EN	0x02
#define  SDHCI_TRNS_AUTO_CMD12	0x04
#define  SDHCI_TRNS_AUTO_CMD23	0x08
/*
 * Since Version 4.10, use of Auto CMD Auto Select is recommended rather
 * than use of Auto CMD12 Enable or Auto CMD23 Enable.
 * Selection of Auto CMD depends on setting of CMD23 Enable in the Host
 * Control 2 register.
 */
#define  SDHCI_TRNS_AUTO_CMD_AUTO_SEL	0x0C
#define  SDHCI_TRNS_READ	0x10
#define  SDHCI_TRNS_MULTI	0x20
/*
 * Defined in Host Version 4.10.
 * 1 - R5 (SDIO)
 * 0 - R1 (Memory)
 */
#define  SDHCI_TRNS_RES_TYPE		0x40
#define  SDHCI_TRNS_RES_ERR_CHECK	0x80
#define  SDHCI_TRNS_RES_INT_DIS		0x0100

#define SDHCI_COMMAND		0x0E
#define  SDHCI_CMD_RESP_MASK	0x03
/*
 * Host Version 4.10 adds this bit to distinguish a main command or
 * sub command.
 * CMD53(SDIO) - main command
 * CMD52(SDIO) - sub command which doesn't have data block or doesn't
 * indicate busy.
 */
#define  SDHCI_CMD_SUB_CMD	0x04
#define  SDHCI_CMD_CRC		0x08
#define  SDHCI_CMD_INDEX	0x10
#define  SDHCI_CMD_DATA		0x20
#define  SDHCI_CMD_ABORTCMD	0xC0

#define  SDHCI_CMD_RESP_NONE	0x00
#define  SDHCI_CMD_RESP_LONG	0x01
#define  SDHCI_CMD_RESP_SHORT	0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))
#define SDHCI_GET_CMD(c) ((c>>8) & 0x3f)

#define SDHCI_RESPONSE		0x10
#define  SDHCI_RESPONSE_CM_TRAN_ABORT_OFFSET	0x10
#define  SDHCI_RESPONSE_CM_TRAN_ABORT_SIZE	4
#define  SDHCI_RESPONSE_SD_TRAN_ABORT_OFFSET	0x18
#define  SDHCI_RESPONSE_SD_TRAN_ABORT_SIZE	8

#define SDHCI_BUFFER		0x20

#define SDHCI_PRESENT_STATE	0x24
#define  SDHCI_CMD_INHIBIT	0x00000001
#define  SDHCI_DATA_INHIBIT	0x00000002
#define  SDHCI_DATA_HIGH_LVL_MASK	0x000000F0
#define  SDHCI_DOING_WRITE	0x00000100
#define  SDHCI_DOING_READ	0x00000200
#define  SDHCI_SPACE_AVAILABLE	0x00000400
#define  SDHCI_DATA_AVAILABLE	0x00000800
#define  SDHCI_CARD_PRESENT	0x00010000
#define  SDHCI_WRITE_PROTECT	0x00080000
#define  SDHCI_DATA_LVL_MASK	0x00F00000
#define   SDHCI_DATA_LVL_SHIFT	20
#define   SDHCI_DATA_0_LVL_MASK	0x00100000
#define  SDHCI_HOST_REGULATOR_STABLE	0x02000000
#define  SDHCI_CMD_NOT_ISSUE_ERR	0x08000000
#define  SDHCI_SUB_CMD_STATUS		0x10000000
#define  SDHCI_UHS2_IN_DORMANT_STATE	0x20000000
#define  SDHCI_UHS2_LANE_SYNC		0x40000000
#define  SDHCI_UHS2_IF_DETECT		0x80000000

#define SDHCI_HOST_CONTROL	0x28
#define  SDHCI_CTRL_LED		0x01
#define  SDHCI_CTRL_4BITBUS	0x02
#define  SDHCI_CTRL_HISPD	0x04
#define  SDHCI_CTRL_DMA_MASK	0x18
#define   SDHCI_CTRL_SDMA	0x00
#define   SDHCI_CTRL_ADMA1	0x08
#define   SDHCI_CTRL_ADMA32	0x10
#define   SDHCI_CTRL_ADMA64	0x18
#define   SDHCI_CTRL_8BITBUS	0x20

#define SDHCI_POWER_CONTROL	0x29
#define  SDHCI_POWER_ON		0x01
#define  SDHCI_POWER_180	0x0A
#define  SDHCI_POWER_300	0x0C
#define  SDHCI_POWER_330	0x0E
/* VDD2 - UHS2 */
#define  SDHCI_VDD2_POWER_ON		0x10
#define  SDHCI_VDD2_POWER_180		0xA0
#define  SDHCI_VDD2_POWER_120		0x80

#define SDHCI_BLOCK_GAP_CONTROL	0x2A

#define SDHCI_WAKE_UP_CONTROL	0x2B
#define  SDHCI_WAKE_ON_INT	0x01
#define  SDHCI_WAKE_ON_INSERT	0x02
#define  SDHCI_WAKE_ON_REMOVE	0x04

#define SDHCI_CLOCK_CONTROL	0x2C
#define  SDHCI_DIVIDER_SHIFT	8
#define  SDHCI_DIVIDER_HI_SHIFT	6
#define  SDHCI_DIV_MASK	0xFF
#define  SDHCI_DIV_MASK_LEN	8
#define  SDHCI_DIV_HI_MASK	0x300
#define  SDHCI_PROG_CLOCK_MODE	0x0020
#define  SDHCI_PLL_EN	0x0008
#define  SDHCI_CLOCK_CARD_EN	0x0004
#define  SDHCI_CLOCK_INT_STABLE	0x0002
#define  SDHCI_CLOCK_INT_EN	0x0001

#define SDHCI_TIMEOUT_CONTROL	0x2E

#define SDHCI_SOFTWARE_RESET	0x2F
#define  SDHCI_RESET_ALL	0x01
#define  SDHCI_RESET_CMD	0x02
#define  SDHCI_RESET_DATA	0x04

#define SDHCI_INT_STATUS	0x30
#define SDHCI_INT_ENABLE	0x34
#define SDHCI_SIGNAL_ENABLE	0x38
#define  SDHCI_INT_RESPONSE	0x00000001
#define  SDHCI_INT_DATA_END	0x00000002
#define  SDHCI_INT_BLK_GAP	0x00000004
#define  SDHCI_INT_DMA_END	0x00000008
#define  SDHCI_INT_SPACE_AVAIL	0x00000010
#define  SDHCI_INT_DATA_AVAIL	0x00000020
#define  SDHCI_INT_CARD_INSERT	0x00000040
#define  SDHCI_INT_CARD_REMOVE	0x00000080
#define  SDHCI_INT_CARD_INT	0x00000100
/* Host Version 4.10 */
#define  SDHCI_INT_FX_EVENT	0x00002000
#define  SDHCI_INT_ERROR	0x00008000
#define  SDHCI_INT_TIMEOUT	0x00010000
#define  SDHCI_INT_CRC		0x00020000
#define  SDHCI_INT_END_BIT	0x00040000
#define  SDHCI_INT_INDEX	0x00080000
#define  SDHCI_INT_DATA_TIMEOUT	0x00100000
#define  SDHCI_INT_DATA_CRC	0x00200000
#define  SDHCI_INT_DATA_END_BIT	0x00400000
#define  SDHCI_INT_BUS_POWER	0x00800000
#define  SDHCI_INT_ACMD12ERR	0x01000000
#define  SDHCI_INT_ADMA_ERROR	0x02000000
/* Host Version 4.0 */
#define  SDHCI_INT_RESPONSE_ERROR	0x08000000

#define  SDHCI_INT_NORMAL_MASK	0x00007FFF
#define  SDHCI_INT_ERROR_MASK	0xFFFF8000

#define  SDHCI_INT_CMD_MASK	(SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT | \
		SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX)
#define  SDHCI_INT_DATA_MASK	(SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
		SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
		SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
		SDHCI_INT_DATA_END_BIT | SDHCI_INT_ADMA_ERROR | \
		SDHCI_INT_BLK_GAP)
#define SDHCI_INT_ALL_MASK	((unsigned int)-1)

#define SDHCI_ACMD12_ERR	0x3C
/* Host Version 4.10 */
#define  SDHCI_ACMD_RESPONSE_ERROR	0x0020

#define SDHCI_HOST_CONTROL2		0x3E
#define  SDHCI_CTRL_UHS_MASK		0x0007
#define   SDHCI_CTRL_UHS_SDR12		0x0000
#define   SDHCI_CTRL_UHS_SDR25		0x0001
#define   SDHCI_CTRL_UHS_SDR50		0x0002
#define   SDHCI_CTRL_UHS_SDR104		0x0003
#define   SDHCI_CTRL_UHS_DDR50		0x0004
#define   SDHCI_CTRL_HS400		0x0005 /* Non-standard */
#define  SDHCI_CTRL_VDD_180		0x0008
#define  SDHCI_CTRL_DRV_TYPE_MASK	0x0030
/* UHS2 */
#define   SDHCI_CTRL_UHS_2		0x0007
#define  SDHCI_CTRL_VDD_180		0x0008	/* UHS-I only */
#define  SDHCI_CTRL_DRV_TYPE_MASK	0x0030	/* UHS-I only */
#define   SDHCI_CTRL_DRV_TYPE_B		0x0000
#define   SDHCI_CTRL_DRV_TYPE_A		0x0010
#define   SDHCI_CTRL_DRV_TYPE_C		0x0020
#define   SDHCI_CTRL_DRV_TYPE_D		0x0030
#define  SDHCI_CTRL_EXEC_TUNING		0x0040	/* UHS-I only */
#define  SDHCI_CTRL_TUNED_CLK		0x0080	/* UHS-I only */
/* UHS2 */
#define  SDHCI_CTRL_UHS2_INTERFACE_EN	0x0100
#define  SDHCI_CTRL_ADMA2_LEN_MODE	0x0400
#define  SDHCI_CTRL_CMD23_EN		0x0800
#define  SDHCI_CTRL_HOST_4_EN		0x1000
#define  SDHCI_CTRL_ADDRESS_64_BIT	0x2000
#define  SDHCI_CTRL_ASYNC_INT_EN	0x4000
#define  SDHCI_CTRL_PRESET_VAL_ENABLE	0x8000

#define SDHCI_CAPABILITIES	0x40
#define  SDHCI_TIMEOUT_CLK_MASK	0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT	0x00000080
#define  SDHCI_CLOCK_BASE_MASK	0x00003F00
#define  SDHCI_CLOCK_V3_BASE_MASK	0x0000FF00
#define  SDHCI_CLOCK_BASE_SHIFT	8
#define  SDHCI_MAX_BLOCK_MASK	0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_8BIT	0x00040000
#define  SDHCI_CAN_DO_ADMA2	0x00080000
#define  SDHCI_CAN_DO_ADMA1	0x00100000
#define  SDHCI_CAN_DO_HISPD	0x00200000
#define  SDHCI_CAN_DO_SDMA	0x00400000
#define  SDHCI_CAN_VDD_330	0x01000000
#define  SDHCI_CAN_VDD_300	0x02000000
#define  SDHCI_CAN_VDD_180	0x04000000
/* Host Version 4.10 */
#define  SDHCI_CAN_64BIT_V4	0x08000000
#define  SDHCI_CAN_64BIT	0x10000000
#define  SDHCI_CAN_ASYNC_INT	0x20000000

#define  SDHCI_SUPPORT_SDR50	0x00000001
#define  SDHCI_SUPPORT_SDR104	0x00000002
#define  SDHCI_SUPPORT_DDR50	0x00000004
#define  SDHCI_DRIVER_TYPE_A	0x00000010
#define  SDHCI_DRIVER_TYPE_C	0x00000020
#define  SDHCI_DRIVER_TYPE_D	0x00000040
#define  SDHCI_RETUNING_TIMER_COUNT_MASK	0x00000F00
#define  SDHCI_RETUNING_TIMER_COUNT_SHIFT	8
#define  SDHCI_USE_SDR50_TUNING			0x00002000
#define  SDHCI_RETUNING_MODE_MASK		0x0000C000
#define  SDHCI_RETUNING_MODE_SHIFT		14
#define  SDHCI_CLOCK_MUL_MASK	0x00FF0000
#define  SDHCI_CLOCK_MUL_SHIFT	16
#define  SDHCI_SUPPORT_HS400	0x80000000 /* Non-standard */
/* UHS2 */
#define  SDHCI_SUPPORT_VDD2_180	0x10000000

#define SDHCI_CAPABILITIES_1	0x44
/* UHS2 */
#define  SDHCI_SUPPORT_UHS2	0x00000008
#define  SDHCI_CAN_DO_ADMA3	0x08000000

#define SDHCI_MAX_CURRENT		0x48
#define SDHCI_MAX_CURRENT_1		0x4C
#define  SDHCI_MAX_CURRENT_LIMIT	0xFF
#define  SDHCI_MAX_CURRENT_330_MASK	0x0000FF
#define  SDHCI_MAX_CURRENT_330_SHIFT	0
#define  SDHCI_MAX_CURRENT_300_MASK	0x00FF00
#define  SDHCI_MAX_CURRENT_300_SHIFT	8
#define  SDHCI_MAX_CURRENT_180_MASK	0xFF0000
#define  SDHCI_MAX_CURRENT_180_SHIFT	16
/* UHS2 */
#define  SDHCI_MAX_CURRENT_VDD2_180_MASK	0x0000000FF
#define   SDHCI_MAX_CURRENT_MULTIPLIER	4

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR	0x50
/* Host Version 4.10 */
#define SDHCI_SET_ACMD_RESPONSE_ERROR	0x20
#define SDHCI_SET_INT_ERROR	0x52
/* Host Version 4.10 */
#define SDHCI_SET_INT_TUNING_ERROR	0x0400
#define SDHCI_SET_INT_RESPONSE_ERROR	0x0800

#define SDHCI_ADMA_ERROR	0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS	0x58
#define SDHCI_ADMA_ADDRESS_HI	0x5C

/* 60-FB reserved */

#define SDHCI_PRESET_FOR_SDR12 0x66
#define SDHCI_PRESET_FOR_SDR25 0x68
#define SDHCI_PRESET_FOR_SDR50 0x6A
#define SDHCI_PRESET_FOR_SDR104        0x6C
#define SDHCI_PRESET_FOR_DDR50 0x6E
#define SDHCI_PRESET_FOR_HS400 0x74 /* Non-standard */
/* UHS2 */
#define SDHCI_PRESET_FOR_UHS2  0x74
#define SDHCI_PRESET_DRV_MASK  0xC000
#define SDHCI_PRESET_DRV_SHIFT  14
#define SDHCI_PRESET_CLKGEN_SEL_MASK   0x400
#define SDHCI_PRESET_CLKGEN_SEL_SHIFT	10
#define SDHCI_PRESET_SDCLK_FREQ_MASK   0x3FF
#define SDHCI_PRESET_SDCLK_FREQ_SHIFT	0

#define SDHCI_ADMA3_ADDRESS	0x78

/* UHS-II */
#define SDHCI_UHS2_BLOCK_SIZE	0x80
#define  SDHCI_UHS2_MAKE_BLKSZ(dma, blksz) \
	(((dma & 0x7) << 12) | (blksz & 0xFFF))

#define SDHCI_UHS2_BLOCK_COUNT	0x84

#define SDHCI_UHS2_CMD_PACKET	0x88
#define  SDHCI_UHS2_CMD_PACK_MAX_LEN	20

#define SDHCI_UHS2_TRANS_MODE	0x9C
#define  SDHCI_UHS2_TRNS_DMA		0x0001
#define  SDHCI_UHS2_TRNS_BLK_CNT_EN	0x0002
#define  SDHCI_UHS2_TRNS_DATA_TRNS_WRT	0x0010
#define  SDHCI_UHS2_TRNS_BLK_BYTE_MODE	0x0020
#define  SDHCI_UHS2_TRNS_RES_R5		0x0040
#define  SDHCI_UHS2_TRNS_RES_ERR_CHECK_EN	0x0080
#define  SDHCI_UHS2_TRNS_RES_INT_DIS	0x0100
#define  SDHCI_UHS2_TRNS_WAIT_EBSY	0x4000
#define  SDHCI_UHS2_TRNS_2L_HD		0x8000

#define SDHCI_UHS2_COMMAND	0x9E
#define  SDHCI_UHS2_COMMAND_SUB_CMD	0x0004
#define  SDHCI_UHS2_COMMAND_DATA	0x0020
#define  SDHCI_UHS2_COMMAND_TRNS_ABORT	0x0040
#define  SDHCI_UHS2_COMMAND_CMD12	0x0080
#define  SDHCI_UHS2_COMMAND_DORMANT	0x00C0
#define  SDHCI_UHS2_COMMAND_PACK_LEN_MASK	0x1F00
#define  SDHCI_UHS2_COMMAND_PACK_LEN_SHIFT	8

#define SDHCI_UHS2_RESPONSE	0xA0
#define  SDHCI_UHS2_RESPONSE_MAX_LEN	20

#define SDHCI_UHS2_MSG_SELECT	0xB4
#define SDHCI_UHS2_MSG_SELECT_CURR	0x0
#define SDHCI_UHS2_MSG_SELECT_ONE	0x1
#define SDHCI_UHS2_MSG_SELECT_TWO	0x2
#define SDHCI_UHS2_MSG_SELECT_THREE	0x3

#define SDHCI_UHS2_MSG		0xB8

#define SDHCI_UHS2_DEV_INT_STATUS	0xBC

#define SDHCI_UHS2_DEV_SELECT	0xBE
#define SDHCI_UHS2_DEV_SELECT_DEV_SEL_MASK	0x0F
#define SDHCI_UHS2_DEV_SELECT_INT_MSG_EN	0x80

#define SDHCI_UHS2_DEV_INT_CODE	0xBF

#define SDHCI_UHS2_SW_RESET	0xC0
#define SDHCI_UHS2_SW_RESET_FULL	0x0001
#define SDHCI_UHS2_SW_RESET_SD		0x0002

#define SDHCI_UHS2_TIMER_CTRL	0xC2
#define SDHCI_UHS2_TIMER_CTRL_DEADLOCK_SHIFT	4

#define SDHCI_UHS2_ERR_INT_STATUS	0xC4
#define SDHCI_UHS2_ERR_INT_STATUS_EN	0xC8
#define SDHCI_UHS2_ERR_INT_SIG_EN	0xCC
#define SDHCI_UHS2_ERR_INT_STATUS_HEADER	0x00000001
#define SDHCI_UHS2_ERR_INT_STATUS_RES		0x00000002
#define SDHCI_UHS2_ERR_INT_STATUS_RETRY_EXP	0x00000004
#define SDHCI_UHS2_ERR_INT_STATUS_CRC		0x00000008
#define SDHCI_UHS2_ERR_INT_STATUS_FRAME		0x00000010
#define SDHCI_UHS2_ERR_INT_STATUS_TID		0x00000020
#define SDHCI_UHS2_ERR_INT_STATUS_UNRECOVER	0x00000080
#define SDHCI_UHS2_ERR_INT_STATUS_EBUSY		0x00000100
#define SDHCI_UHS2_ERR_INT_STATUS_ADMA		0x00008000
#define SDHCI_UHS2_ERR_INT_STATUS_RES_TIMEOUT	0x00010000
#define SDHCI_UHS2_ERR_INT_STATUS_DEADLOCK_TIMEOUT	0x00020000
#define SDHCI_UHS2_ERR_INT_STATUS_VENDOR	0x08000000
#define SDHCI_UHS2_ERR_INT_STATUS_MASK	\
		(SDHCI_UHS2_ERR_INT_STATUS_HEADER |	\
		SDHCI_UHS2_ERR_INT_STATUS_RES |		\
		SDHCI_UHS2_ERR_INT_STATUS_RETRY_EXP |	\
		SDHCI_UHS2_ERR_INT_STATUS_CRC |		\
		SDHCI_UHS2_ERR_INT_STATUS_FRAME |	\
		SDHCI_UHS2_ERR_INT_STATUS_TID |		\
		SDHCI_UHS2_ERR_INT_STATUS_UNRECOVER |	\
		SDHCI_UHS2_ERR_INT_STATUS_EBUSY |	\
		SDHCI_UHS2_ERR_INT_STATUS_ADMA |	\
		SDHCI_UHS2_ERR_INT_STATUS_RES_TIMEOUT |	\
		SDHCI_UHS2_ERR_INT_STATUS_DEADLOCK_TIMEOUT)
#define SDHCI_UHS2_ERR_INT_STATUS_CMD_MASK	\
		(SDHCI_UHS2_ERR_INT_STATUS_HEADER |	\
		SDHCI_UHS2_ERR_INT_STATUS_RES |		\
		SDHCI_UHS2_ERR_INT_STATUS_FRAME |	\
		SDHCI_UHS2_ERR_INT_STATUS_TID |		\
		SDHCI_UHS2_ERR_INT_STATUS_RES_TIMEOUT)
/* TODO: CRC is for data or cmd? */
#define SDHCI_UHS2_ERR_INT_STATUS_DATA_MASK	\
		(SDHCI_UHS2_ERR_INT_STATUS_RETRY_EXP |	\
		SDHCI_UHS2_ERR_INT_STATUS_CRC |		\
		SDHCI_UHS2_ERR_INT_STATUS_UNRECOVER |	\
		SDHCI_UHS2_ERR_INT_STATUS_EBUSY |	\
		SDHCI_UHS2_ERR_INT_STATUS_ADMA |	\
		SDHCI_UHS2_ERR_INT_STATUS_DEADLOCK_TIMEOUT)

#define SDHCI_UHS2_SET_PTR	0xE0
#define  SDHCI_UHS2_GEN_SET	0
#define   SDHCI_UHS2_GEN_SET_POWER_LOW	0x0001
#define   SDHCI_UHS2_GEN_SET_N_LANES_POS	8
#define   SDHCI_UHS2_GEN_SET_2L_FD_HD	0x0
#define   SDHCI_UHS2_GEN_SET_2D1U_FD	0x2
#define   SDHCI_UHS2_GEN_SET_1D2U_FD	0x3
#define   SDHCI_UHS2_GEN_SET_2D2U_FD	0x4

#define  SDHCI_UHS2_PHY_SET	4
#define   SDHCI_UHS2_PHY_SET_SPEED_POS	6
#define   SDHCI_UHS2_PHY_SET_HIBER_POS	15
#define   SDHCI_UHS2_PHY_SET_HIBER_EN	0x00008000
#define   SDHCI_UHS2_PHY_SET_N_LSS_SYN_MASK	0x000F0000
#define   SDHCI_UHS2_PHY_SET_N_LSS_SYN_POS	16
#define   SDHCI_UHS2_PHY_SET_N_LSS_DIR_MASK	0x00F00000
#define   SDHCI_UHS2_PHY_SET_N_LSS_DIR_POS	20

#define  SDHCI_UHS2_TRAN_SET	8
#define   SDHCI_UHS2_TRAN_SET_N_FCU_MASK	0x0000FF00
#define   SDHCI_UHS2_TRAN_SET_N_FCU_POS	8
#define   SDHCI_UHS2_TRAN_SET_RETRY_CNT_MASK	0x00030000
#define   SDHCI_UHS2_TRAN_SET_RETRY_CNT_POS	16

#define  SDHCI_UHS2_TRAN_SET_1	12
#define   SDHCI_UHS2_TRAN_SET_1_N_DAT_GAP_MASK	0x000000FF

#define SDHCI_UHS2_HOST_CAPS_PTR	0xE2
#define  SDHCI_UHS2_HOST_CAPS_GEN_OFFSET	0
#define   SDHCI_UHS2_HOST_CAPS_GEN_DAP_MASK	0x0000000F
#define   SDHCI_UHS2_HOST_CAPS_GEN_GAP_MASK	0x000000F0
#define   SDHCI_UHS2_HOST_CAPS_GEN_GAP_SHIFT	4
#define   SDHCI_UHS2_HOST_CAPS_GEN_GAP(gap)	(gap * 360)
#define   SDHCI_UHS2_HOST_CAPS_GEN_LANE_MASK	0x00003F00
#define   SDHCI_UHS2_HOST_CAPS_GEN_LANE_SHIFT	8
#define    SDHCI_UHS2_HOST_CAPS_GEN_2L_HD_FD	1
#define    SDHCI_UHS2_HOST_CAPS_GEN_2D1U_FD	2
#define    SDHCI_UHS2_HOST_CAPS_GEN_1D2U_FD	4
#define    SDHCI_UHS2_HOST_CAPS_GEN_2D2U_FD	8
#define   SDHCI_UHS2_HOST_CAPS_GEN_ADDR_64	0x00004000
#define   SDHCI_UHS2_HOST_CAPS_GEN_BOOT		0x00008000
#define   SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_MASK	0x00030000
#define   SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_SHIFT	16
#define    SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_RMV	0
#define    SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_EMB	1
#define    SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_EMB_RMV	2
#define   SDHCI_UHS2_HOST_CAPS_GEN_NUM_DEV_MASK		0x003C0000
#define   SDHCI_UHS2_HOST_CAPS_GEN_NUM_DEV_SHIFT	18
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_MASK	0x00C00000
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_SHIFT	22
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_P2P		0
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_RING	1
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_HUB		2
#define   SDHCI_UHS2_HOST_CAPS_GEN_BUS_TOPO_HUB_RING	3

#define  SDHCI_UHS2_HOST_CAPS_PHY_OFFSET	4
#define   SDHCI_UHS2_HOST_CAPS_PHY_REV_MASK	0x0000003F
#define   SDHCI_UHS2_HOST_CAPS_PHY_RANGE_MASK	0x000000C0
#define   SDHCI_UHS2_HOST_CAPS_PHY_RANGE_SHIFT	6
#define   SDHCI_UHS2_HOST_CAPS_PHY_RANGE_A	0
#define   SDHCI_UHS2_HOST_CAPS_PHY_RANGE_B	1
#define   SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_SYN_MASK	0x000F0000
#define   SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_SYN_SHIFT	16
#define   SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_DIR_MASK	0x00F00000
#define   SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_DIR_SHIFT	20
#define  SDHCI_UHS2_HOST_CAPS_TRAN_OFFSET	8
#define   SDHCI_UHS2_HOST_CAPS_TRAN_LINK_REV_MASK	0x0000003F
#define   SDHCI_UHS2_HOST_CAPS_TRAN_N_FCU_MASK	0x0000FF00
#define   SDHCI_UHS2_HOST_CAPS_TRAN_N_FCU_SHIFT	8
#define   SDHCI_UHS2_HOST_CAPS_TRAN_HOST_TYPE_MASK	0x00070000
#define   SDHCI_UHS2_HOST_CAPS_TRAN_HOST_TYPE_SHIFT	16
#define   SDHCI_UHS2_HOST_CAPS_TRAN_BLK_LEN_MASK	0xFFF00000
#define   SDHCI_UHS2_HOST_CAPS_TRAN_BLK_LEN_SHIFT	20

#define  SDHCI_UHS2_HOST_CAPS_TRAN_1_OFFSET	12
#define  SDHCI_UHS2_HOST_CAPS_TRAN_1_N_DATA_GAP_MASK	0x000000FF

#define SDHCI_UHS2_TEST_PTR	0xE4
#define  SDHCI_UHS2_TEST_ERR_HEADER	0x00000001
#define  SDHCI_UHS2_TEST_ERR_RES	0x00000002
#define  SDHCI_UHS2_TEST_ERR_RETRY_EXP	0x00000004
#define  SDHCI_UHS2_TEST_ERR_CRC	0x00000008
#define  SDHCI_UHS2_TEST_ERR_FRAME	0x00000010
#define  SDHCI_UHS2_TEST_ERR_TID	0x00000020
#define  SDHCI_UHS2_TEST_ERR_UNRECOVER	0x00000080
#define  SDHCI_UHS2_TEST_ERR_EBUSY	0x00000100
#define  SDHCI_UHS2_TEST_ERR_ADMA	0x00008000
#define  SDHCI_UHS2_TEST_ERR_RES_TIMEOUT	0x00010000
#define  SDHCI_UHS2_TEST_ERR_DEADLOCK_TIMEOUT	0x00020000
#define  SDHCI_UHS2_TEST_ERR_VENDOR	0x08000000

#define SDHCI_UHS2_EMBED_CTRL	0xE6
#define SDHCI_UHS2_VENDOR	0xE8

#define SDHCI_SLOT_INT_STATUS	0xFC

#define SDHCI_HOST_VERSION	0xFE
#define  SDHCI_VENDOR_VER_MASK	0xFF00
#define  SDHCI_VENDOR_VER_SHIFT	8
#define  SDHCI_SPEC_VER_MASK	0x00FF
#define  SDHCI_SPEC_VER_SHIFT	0
#define   SDHCI_SPEC_100	0
#define   SDHCI_SPEC_200	1
#define   SDHCI_SPEC_300	2
#define   SDHCI_SPEC_400	3
#define   SDHCI_SPEC_410	4

/*
 * End of controller registers.
 */

#define SDHCI_MAX_DIV_SPEC_200	256
#define SDHCI_MAX_DIV_SPEC_300	2046

/*
 * Host SDMA buffer boundary. Valid values from 4K to 512K in powers of 2.
 */
#define SDHCI_DEFAULT_BOUNDARY_SIZE  (512 * 1024)
#define SDHCI_DEFAULT_BOUNDARY_ARG   (ilog2(SDHCI_DEFAULT_BOUNDARY_SIZE) - 12)

/* ADMA2 32-bit DMA descriptor size */
#define SDHCI_ADMA2_32_DESC_SZ	8

/* ADMA2 32-bit DMA alignment */
#define SDHCI_ADMA2_32_ALIGN	4

/* ADMA2 32-bit descriptor */
struct sdhci_adma2_32_desc {
	__le16	cmd;
	__le16	len;
	__le32	addr;
}  __packed __aligned(SDHCI_ADMA2_32_ALIGN);

/* ADMA2 64-bit DMA descriptor size */
#define SDHCI_ADMA2_64_DESC_SZ	12

/* ADMA2 64-bit DMA alignment */
#define SDHCI_ADMA2_64_ALIGN	8

/*
 * ADMA2 64-bit descriptor. Note 12-byte descriptor can't always be 8-byte
 * aligned.
 */
struct sdhci_adma2_64_desc {
	__le16	cmd;
	__le16	len;
	__le32	addr_lo;
	__le32	addr_hi;
}  __packed __aligned(4);

#define ADMA2_TRAN_VALID	0x21
#define ADMA2_NOP_END_VALID	0x3
#define ADMA2_END		0x2

/*
 * Maximum segments assuming a 512KiB maximum requisition size and a minimum
 * 4KiB page size.
 */
#define SDHCI_MAX_SEGS		128

enum sdhci_cookie {
	COOKIE_UNMAPPED,
	COOKIE_MAPPED,
	COOKIE_GIVEN,
};

struct sdhci_host {
	/* Data set by hardware interface driver */
	const char *hw_name;	/* Hardware bus name */

	unsigned int quirks;	/* Deviations from spec. */

/* Controller doesn't honor resets unless we touch the clock register */
#define SDHCI_QUIRK_CLOCK_BEFORE_RESET			(1<<0)
/* Controller has bad caps bits, but really supports DMA */
#define SDHCI_QUIRK_FORCE_DMA				(1<<1)
/* Controller doesn't like to be reset when there is no card inserted. */
#define SDHCI_QUIRK_NO_CARD_NO_RESET			(1<<2)
/* Controller doesn't like clearing the power reg before a change */
#define SDHCI_QUIRK_SINGLE_POWER_WRITE			(1<<3)
/* Controller has flaky internal state so reset it on each ios change */
#define SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS		(1<<4)
/* Controller has an unusable DMA engine */
#define SDHCI_QUIRK_BROKEN_DMA				(1<<5)
/* Controller has an unusable ADMA engine */
#define SDHCI_QUIRK_BROKEN_ADMA				(1<<6)
/* Controller can only DMA from 32-bit aligned addresses */
#define SDHCI_QUIRK_32BIT_DMA_ADDR			(1<<7)
/* Controller can only DMA chunk sizes that are a multiple of 32 bits */
#define SDHCI_QUIRK_32BIT_DMA_SIZE			(1<<8)
/* Controller can only ADMA chunks that are a multiple of 32 bits */
#define SDHCI_QUIRK_32BIT_ADMA_SIZE			(1<<9)
/* Controller needs to be reset after each request to stay stable */
#define SDHCI_QUIRK_RESET_AFTER_REQUEST			(1<<10)
/* Controller needs voltage and power writes to happen separately */
#define SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER		(1<<11)
/* Controller provides an incorrect timeout value for transfers */
#define SDHCI_QUIRK_BROKEN_TIMEOUT_VAL			(1<<12)
/* Controller has an issue with buffer bits for small transfers */
#define SDHCI_QUIRK_BROKEN_SMALL_PIO			(1<<13)
/* Controller does not provide transfer-complete interrupt when not busy */
#define SDHCI_QUIRK_NO_BUSY_IRQ				(1<<14)
/* Controller has unreliable card detection */
#define SDHCI_QUIRK_BROKEN_CARD_DETECTION		(1<<15)
/* Controller reports inverted write-protect state */
#define SDHCI_QUIRK_INVERTED_WRITE_PROTECT		(1<<16)
/* Controller does not like fast PIO transfers */
#define SDHCI_QUIRK_PIO_NEEDS_DELAY			(1<<18)
/* Controller has to be forced to use block size of 2048 bytes */
#define SDHCI_QUIRK_FORCE_BLK_SZ_2048			(1<<20)
/* Controller cannot do multi-block transfers */
#define SDHCI_QUIRK_NO_MULTIBLOCK			(1<<21)
/* Controller can only handle 1-bit data transfers */
#define SDHCI_QUIRK_FORCE_1_BIT_DATA			(1<<22)
/* Controller needs 10ms delay between applying power and clock */
#define SDHCI_QUIRK_DELAY_AFTER_POWER			(1<<23)
/* Controller uses SDCLK instead of TMCLK for data timeouts */
#define SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK		(1<<24)
/* Controller reports wrong base clock capability */
#define SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN		(1<<25)
/* Controller cannot support End Attribute in NOP ADMA descriptor */
#define SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC		(1<<26)
/* Controller is missing device caps. Use caps provided by host */
#define SDHCI_QUIRK_MISSING_CAPS			(1<<27)
/* Controller uses Auto CMD12 command to stop the transfer */
#define SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12		(1<<28)
/* Controller doesn't have HISPD bit field in HI-SPEED SD card */
#define SDHCI_QUIRK_NO_HISPD_BIT			(1<<29)
/* Controller treats ADMA descriptors with length 0000h incorrectly */
#define SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC		(1<<30)
/* The read-only detection via SDHCI_PRESENT_STATE register is unstable */
#define SDHCI_QUIRK_UNSTABLE_RO_DETECT			(1<<31)

	unsigned int quirks2;	/* More deviations from spec. */

#define SDHCI_QUIRK2_HOST_OFF_CARD_ON			(1<<0)
#define SDHCI_QUIRK2_HOST_NO_CMD23			(1<<1)
/* The system physically doesn't support 1.8v, even if the host does */
#define SDHCI_QUIRK2_NO_1_8_V				(1<<2)
#define SDHCI_QUIRK2_PRESET_VALUE_BROKEN		(1<<3)
#define SDHCI_QUIRK2_CARD_ON_NEEDS_BUS_ON		(1<<4)
/* Controller has a non-standard host control register */
#define SDHCI_QUIRK2_BROKEN_HOST_CONTROL		(1<<5)
/* Controller does not support HS200 */
#define SDHCI_QUIRK2_BROKEN_HS200			(1<<6)
/* Controller does not support DDR50 */
#define SDHCI_QUIRK2_BROKEN_DDR50			(1<<7)
/* Stop command (CMD12) can set Transfer Complete when not using MMC_RSP_BUSY */
#define SDHCI_QUIRK2_STOP_WITH_TC			(1<<8)
/* Controller does not support 64-bit DMA */
#define SDHCI_QUIRK2_BROKEN_64_BIT_DMA			(1<<9)
/* need clear transfer mode register before send cmd */
#define SDHCI_QUIRK2_CLEAR_TRANSFERMODE_REG_BEFORE_CMD	(1<<10)
/* Capability register bit-63 indicates HS400 support */
#define SDHCI_QUIRK2_CAPS_BIT63_FOR_HS400		(1<<11)
/* forced tuned clock */
#define SDHCI_QUIRK2_TUNING_WORK_AROUND			(1<<12)
/* disable the block count for single block transactions */
#define SDHCI_QUIRK2_SUPPORT_SINGLE			(1<<13)
/* Controller broken with using ACMD23 */
#define SDHCI_QUIRK2_ACMD23_BROKEN			(1<<14)
/* Broken Clock divider zero in controller */
#define SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN		(1<<15)
/*
 * When internal clock is disabled, a delay is needed before modifying the
 * SD clock frequency or enabling back the internal clock.
 */
#define SDHCI_QUIRK2_NEED_DELAY_AFTER_INT_CLK_RST	(1<<16)
/* unwisely, the LED output controls card power and must stay up */
#define SDHCI_QUIRK2_LED_ABUSED_AS_CARD_POWER		(1<<17)
/* Disable the tuning for SDR50 */
#define SDHCI_QUIRK2_BROKEN_SDR50_TUNING		(1<<18)
/* SD power control */
#define SDHCI_QUIRK2_OPS_SD_POWER_CONTROL		(1<<19)

	int irq;		/* Device IRQ */
	void __iomem *ioaddr;	/* Mapped address */

	const struct sdhci_ops *ops;	/* Low level hw interface */

	/* Internal data */
	struct mmc_host *mmc;	/* MMC structure */
	struct mmc_host_ops mmc_host_ops;	/* MMC host ops */
	u64 dma_mask;		/* custom DMA mask */

#if defined(CONFIG_LEDS_CLASS) || defined(CONFIG_LEDS_CLASS_MODULE)
	struct led_classdev led;	/* LED control */
	char led_name[32];
#endif

	spinlock_t lock;	/* Mutex */

	int flags;		/* Host attributes */
#define SDHCI_USE_SDMA		(1<<0)	/* Host is SDMA capable */
#define SDHCI_USE_ADMA		(1<<1)	/* Host is ADMA capable */
#define SDHCI_REQ_USE_DMA	(1<<2)	/* Use DMA for this req. */
#define SDHCI_DEVICE_DEAD	(1<<3)	/* Device unresponsive */
#define SDHCI_SDR50_NEEDS_TUNING (1<<4)	/* SDR50 needs tuning */
#define SDHCI_AUTO_CMD12	(1<<6)	/* Auto CMD12 support */
#define SDHCI_AUTO_CMD23	(1<<7)	/* Auto CMD23 support */
#define SDHCI_PV_ENABLED	(1<<8)	/* Preset value enabled */
#define SDHCI_SDIO_IRQ_ENABLED	(1<<9)	/* SDIO irq enabled */
#define SDHCI_SDR104_NEEDS_TUNING (1<<10)	/* SDR104/HS200 needs tuning */
#define SDHCI_USE_64_BIT_DMA	(1<<12)	/* Use 64-bit DMA */
#define SDHCI_HS400_TUNING	(1<<13)	/* Tuning for HS400 */
#define SDHCI_USE_UHS2         (1<<14) /* Support UHS2 */

	unsigned int version;	/* SDHCI spec. version */

	unsigned int max_clk;	/* Max possible freq (MHz) */
	unsigned int timeout_clk;	/* Timeout freq (KHz) */
	unsigned int clk_mul;	/* Clock Muliplier value */

	unsigned int clock;	/* Current clock (MHz) */
	u8 pwr;			/* Current voltage */

	bool runtime_suspended;	/* Host is runtime suspended */
	bool bus_on;		/* Bus power prevents runtime suspend */
	bool preset_enabled;	/* Preset is enabled */

	struct mmc_request *mrq;	/* Current request */
	struct mmc_command *cmd;	/* Current command */
	struct mmc_data *data;	/* Current data request */
	unsigned int data_early:1;	/* Data finished before cmd */
	unsigned int busy_handle:1;	/* Handling the order of Busy-end */

	struct sg_mapping_iter sg_miter;	/* SG state for PIO */
	unsigned int blocks;	/* remaining PIO blocks */

	int sg_count;		/* Mapped sg entries */

	void *adma_table;	/* ADMA descriptor table */
	void *align_buffer;	/* Bounce buffer */

	size_t adma_table_sz;	/* ADMA descriptor table size */
	size_t align_buffer_sz;	/* Bounce buffer size */

	dma_addr_t adma_addr;	/* Mapped ADMA descr. table */
	dma_addr_t align_addr;	/* Mapped bounce buffer */

	unsigned int desc_sz;	/* ADMA descriptor size */
	unsigned int align_sz;	/* ADMA alignment */
	unsigned int align_mask;	/* ADMA alignment mask */

	struct tasklet_struct finish_tasklet;	/* Tasklet structures */

	struct timer_list timer;	/* Timer for timeouts */

	u32 caps;		/* Alternative CAPABILITY_0 */
	u32 caps1;		/* Alternative CAPABILITY_1 */

	unsigned int            ocr_avail_sdio;	/* OCR bit masks */
	unsigned int            ocr_avail_sd;
	unsigned int            ocr_avail_mmc;
	u32 ocr_mask;		/* available voltages */

	unsigned		timing;		/* Current timing */

	u32			thread_isr;

	/* cached registers */
	u32			ier;

	wait_queue_head_t	buf_ready_int;	/* Waitqueue for Buffer Read Ready interrupt */
	unsigned int		tuning_done;	/* Condition flag set when CMD19 succeeds */

	unsigned int		tuning_count;	/* Timer count for re-tuning */
	unsigned int		tuning_mode;	/* Re-tuning mode supported by host */
#define SDHCI_TUNING_MODE_1	0

	unsigned long private[0] ____cacheline_aligned;
};

struct sdhci_ops {
#ifdef CONFIG_MMC_SDHCI_IO_ACCESSORS
	u32		(*read_l)(struct sdhci_host *host, int reg);
	u16		(*read_w)(struct sdhci_host *host, int reg);
	u8		(*read_b)(struct sdhci_host *host, int reg);
	void		(*write_l)(struct sdhci_host *host, u32 val, int reg);
	void		(*write_w)(struct sdhci_host *host, u16 val, int reg);
	void		(*write_b)(struct sdhci_host *host, u8 val, int reg);
#endif

	void	(*set_clock)(struct sdhci_host *host, unsigned int clock);

	int		(*enable_dma)(struct sdhci_host *host);
	unsigned int	(*get_max_clock)(struct sdhci_host *host);
	unsigned int	(*get_min_clock)(struct sdhci_host *host);
	unsigned int	(*get_timeout_clock)(struct sdhci_host *host);
	unsigned int	(*get_max_timeout_count)(struct sdhci_host *host);
	void		(*set_timeout)(struct sdhci_host *host,
				       struct mmc_command *cmd);
	void		(*set_bus_width)(struct sdhci_host *host, int width);
	void (*platform_send_init_74_clocks)(struct sdhci_host *host,
					     u8 power_mode);
	unsigned int    (*get_ro)(struct sdhci_host *host);
	void		(*reset)(struct sdhci_host *host, u8 mask);
	int	(*platform_execute_tuning)(struct sdhci_host *host, u32 opcode);
	void	(*set_uhs_signaling)(struct sdhci_host *host, unsigned int uhs);
	void	(*hw_reset)(struct sdhci_host *host);
	void    (*adma_workaround)(struct sdhci_host *host, u32 intmask);
	void	(*platform_init)(struct sdhci_host *host);
	void    (*card_event)(struct sdhci_host *host);
	void	(*voltage_switch)(struct sdhci_host *host);
	int	(*select_drive_strength)(struct sdhci_host *host,
					 struct mmc_card *card,
					 unsigned int max_dtr, int host_drv,
					 int card_drv, int *drv_type);
	int	(*clock_change_quirk)(struct sdhci_host *host);
	int	(*clock_change_quirk2)(struct sdhci_host *host);
	int	(*set_version_quirk)(struct sdhci_host *host);
	int	(*uhs2_to_uhs1)(struct sdhci_host *host,
				irq_handler_t fn0, irq_handler_t fn1);
	int	(*uhs1_to_uhs2)(struct sdhci_host *host,
				irq_handler_t fn0, irq_handler_t fn1);
	void	(*sd_power_on)(struct sdhci_host *host);
	void	(*sd_power_off)(struct sdhci_host *host);
};

#ifdef CONFIG_MMC_SDHCI_IO_ACCESSORS

static inline void sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	if (unlikely(host->ops->write_l))
		host->ops->write_l(host, val, reg);
	else
		writel(val, host->ioaddr + reg);
}

static inline void sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	if (unlikely(host->ops->write_w))
		host->ops->write_w(host, val, reg);
	else
		writew(val, host->ioaddr + reg);
}

static inline void sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	if (unlikely(host->ops->write_b))
		host->ops->write_b(host, val, reg);
	else
		writeb(val, host->ioaddr + reg);
}

static inline u32 sdhci_readl(struct sdhci_host *host, int reg)
{
	if (unlikely(host->ops->read_l))
		return host->ops->read_l(host, reg);
	else
		return readl(host->ioaddr + reg);
}

static inline u16 sdhci_readw(struct sdhci_host *host, int reg)
{
	if (unlikely(host->ops->read_w))
		return host->ops->read_w(host, reg);
	else
		return readw(host->ioaddr + reg);
}

static inline u8 sdhci_readb(struct sdhci_host *host, int reg)
{
	if (unlikely(host->ops->read_b))
		return host->ops->read_b(host, reg);
	else
		return readb(host->ioaddr + reg);
}

#else

static inline void sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	writel(val, host->ioaddr + reg);
}

static inline void sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	writew(val, host->ioaddr + reg);
}

static inline void sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	writeb(val, host->ioaddr + reg);
}

static inline u32 sdhci_readl(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + reg);
}

static inline u16 sdhci_readw(struct sdhci_host *host, int reg)
{
	return readw(host->ioaddr + reg);
}

static inline u8 sdhci_readb(struct sdhci_host *host, int reg)
{
	return readb(host->ioaddr + reg);
}

#endif /* CONFIG_MMC_SDHCI_IO_ACCESSORS */

extern struct sdhci_host *sdhci_alloc_host(struct device *dev,
	size_t priv_size);
extern void sdhci_free_host(struct sdhci_host *host);

static inline void *sdhci_priv(struct sdhci_host *host)
{
	return (void *)host->private;
}

extern void sdhci_card_detect(struct sdhci_host *host);
extern int sdhci_add_host(struct sdhci_host *host);
extern void sdhci_remove_host(struct sdhci_host *host, int dead);
extern void sdhci_send_command(struct sdhci_host *host,
				struct mmc_command *cmd);

static inline bool sdhci_sdio_irq_enabled(struct sdhci_host *host)
{
	return !!(host->flags & SDHCI_SDIO_IRQ_ENABLED);
}

void sdhci_set_clock(struct sdhci_host *host, unsigned int clock);
void sdhci_set_bus_width(struct sdhci_host *host, int width);
void sdhci_reset(struct sdhci_host *host, u8 mask);
void sdhci_set_uhs_signaling(struct sdhci_host *host, unsigned timing);

#ifdef CONFIG_PM
extern int sdhci_suspend_host(struct sdhci_host *host);
extern int sdhci_resume_host(struct sdhci_host *host);
extern void sdhci_enable_irq_wakeups(struct sdhci_host *host);
extern int sdhci_runtime_suspend_host(struct sdhci_host *host);
extern int sdhci_runtime_resume_host(struct sdhci_host *host);
#endif

/* sdhci_uhs2.c needed */
extern void sdhci_enable_preset_value(struct sdhci_host *host, bool enable);
extern void sdhci_dumpregs(struct sdhci_host *host);
extern void sdhci_finish_data(struct sdhci_host *);
extern void sdhci_set_power(struct sdhci_host *host, unsigned char mode,
			unsigned short vdd, unsigned short vdd2);
extern void sdhci_runtime_pm_bus_off(struct sdhci_host *host);
#endif /* __SDHCI_HW_H */
