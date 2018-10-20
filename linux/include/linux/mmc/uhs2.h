/*
 *  linux/drivers/mmc/host/uhs2.h - UHS-II driver
 *
 * Header file for UHS-II packets, Host Controller registers and I/O
 * accessors.
 *
 *  Copyright (C) 2014 Intel Corp, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef LINUX_MMC_UHS2_H
#define LINUX_MMC_UHS2_H

struct mmc_request;

/* LINK Layer definition */
/* UHS2 Header */
#define UHS2_NATIVE_PACKET_POS	7
#define UHS2_NATIVE_PACKET	(1 << UHS2_NATIVE_PACKET_POS)

#define UHS2_PACKET_TYPE_POS	4
#define UHS2_PACKET_TYPE_CCMD	(0 << UHS2_PACKET_TYPE_POS)
#define UHS2_PACKET_TYPE_DCMD	(1 << UHS2_PACKET_TYPE_POS)
#define UHS2_PACKET_TYPE_RES	(2 << UHS2_PACKET_TYPE_POS)
#define UHS2_PACKET_TYPE_DATA	(3 << UHS2_PACKET_TYPE_POS)
#define UHS2_PACKET_TYPE_MSG	(7 << UHS2_PACKET_TYPE_POS)

#define UHS2_DEST_ID_MASK	0x0F
#define UHS2_DEST_ID		0x1

#define UHS2_SRC_ID_POS		12
#define UHS2_SRC_ID_MASK	0xF000

#define UHS2_TRANS_ID_POS	8
#define UHS2_TRANS_ID_MASK	0x0700

/* UHS2 MSG */
#define UHS2_MSG_CTG_POS	5
#define UHS2_MSG_CTG_LMSG	0x00
#define UHS2_MSG_CTG_INT	0x60
#define UHS2_MSG_CTG_AMSG	0x80

#define UHS2_MSG_CTG_FCREQ	0x00
#define UHS2_MSG_CTG_FCRDY	0x01
#define UHS2_MSG_CTG_STAT	0x02

#define UHS2_MSG_CODE_POS			8
#define UHS2_MSG_CODE_FC_UNRECOVER_ERR		0x8
#define UHS2_MSG_CODE_STAT_UNRECOVER_ERR	0x8
#define UHS2_MSG_CODE_STAT_RECOVER_ERR		0x1

/* TRANS Layer definition */

/* Native packets*/
#define UHS2_NATIVE_CMD_RW_POS	7
#define UHS2_NATIVE_CMD_WRITE	(1 << UHS2_NATIVE_CMD_RW_POS)
#define UHS2_NATIVE_CMD_READ	(0 << UHS2_NATIVE_CMD_RW_POS)

#define UHS2_NATIVE_CMD_PLEN_POS	4
#define UHS2_NATIVE_CMD_PLEN_4B		(1 << UHS2_NATIVE_CMD_PLEN_POS)
#define UHS2_NATIVE_CMD_PLEN_8B		(2 << UHS2_NATIVE_CMD_PLEN_POS)
#define UHS2_NATIVE_CMD_PLEN_16B	(3 << UHS2_NATIVE_CMD_PLEN_POS)

#define UHS2_NATIVE_CCMD_GET_MIOADR_MASK	0xF00
#define UHS2_NATIVE_CCMD_MIOADR_MASK		0x0F

#define UHS2_NATIVE_CCMD_LIOADR_POS		8
#define UHS2_NATIVE_CCMD_GET_LIOADR_MASK	0x0FF

#define UHS2_DCMD_DM_POS	6
#define UHS2_DCMD_2L_HD_MODE	(1 << UHS2_DCMD_DM_POS)
#define UHS2_DCMD_LM_POS	5
#define UHS2_DCMD_LM_TLEN_EXIST	(1 << UHS2_DCMD_LM_POS)
#define UHS2_DCMD_TLUM_POS	4
#define UHS2_DCMD_TLUM_BYTE_MODE	(1 << UHS2_DCMD_TLUM_POS)
#define UHS2_NATIVE_DCMD_DAM_POS	3
#define UHS2_NATIVE_DCMD_DAM_IO		(1 << UHS2_NATIVE_DCMD_DAM_POS)
/*
 * Per UHS2 spec, DCMD payload should be MSB first. There may be
 * two types of data be assembled to MSB:
 * 1. TLEN: Input block size for signle read/write and number of blocks
 * for multiple read/write to calculate TLEN as MSB first per spec.
 * 2. SD command argument.
 */
static inline u32 uhs2_dcmd_convert_msb(u32 input)
{
	u32 ret = 0;

	ret = ((input & 0xFF) << 24) |
		(((input >> 8) & 0xFF) << 16) |
		(((input >> 16) & 0xFF) << 8) |
		((input >> 24) & 0xFF);
	return ret;
}

#define UHS2_RES_NACK_POS	7
#define UHS2_RES_NACK_MASK	(0x1 << UHS2_RES_NACK_POS)

#define UHS2_RES_ECODE_POS	4
#define UHS2_RES_ECODE_MASK	0x7
#define UHS2_RES_ECODE_COND	1
#define UHS2_RES_ECODE_ARG	2
#define UHS2_RES_ECODE_GEN	3

/* IOADR of device registers */
#define UHS2_IOADR_GENERIC_CAPS		0x00
#define UHS2_IOADR_PHY_CAPS		0x02
#define UHS2_IOADR_LINK_CAPS		0x04
#define UHS2_IOADR_RSV_CAPS		0x06
#define UHS2_IOADR_GENERIC_SETTINGS	0x08
#define UHS2_IOADR_PHY_SETTINGS		0x0A
#define UHS2_IOADR_LINK_SETTINGS	0x0C
#define UHS2_IOADR_PRESET		0x40

/* SD application packets */
#define UHS2_SD_CMD_INDEX_POS		8

#define UHS2_SD_CMD_APP_POS		14
#define UHS2_SD_CMD_APP			(1 << UHS2_SD_CMD_APP_POS)

struct uhs2_command {
	u16	header;
	u16	arg;
	u32	*payload;
	u32	payload_len;
	u32	packet_len;
	u8	*resp;
	u8	resp_len;
};

struct uhs2_host_caps {
	/* General */
	u8	card_type;
	u8	addr64;
	u8	n_lanes;
	u8	gap;
	u8	dap;
	/* PHY */
	u8	n_lss_dir;
	u8	n_lss_sync;
	u8	speed_range;
	u8	phy_rev;
	/* LINK TRAN */
	u8	n_data_gap;
	u32	maxblk_len;
	u8	host_type;
	u32	n_fcu;
	u8	link_rev;
};

struct uhs2_card_prop {
	u32	node_id;
	/* General */
	u8	app_type;
	u8	dadr_len;
	u8	n_lanes;
	/* PHY */
	u8	n_lss_dir;
	u8	n_lss_sync;
	u8	can_hibernate;
	u8	phy_major_rev;
	u8	phy_minor_rev;
	/* LINK TRAN */
	u8	n_data_gap;
	u32	maxblk_len;
	u8	dev_type;
	u32	n_fcu;
	u8	link_major_rev;
	u8	link_minor_rev;
};

struct uhs2_set_config {
	/* General */
	u8	cfg_complete;      /* Device only */
	u8	n_lanes_set;
	u8	pwrctrl_mode_set;
	/* PHY */
	u8	n_lss_dir_set;
	u8	n_lss_sync_set;
	u8	hibernate_set;     /* Host only   */
	u8	speed_range_set;
	u8	phy_major_rev_set; /* Device only */
	/* LINK TRAN */
	u8	n_data_gap_set;
	u32	maxblk_len_set;    /* Device only */
	u8	max_retry_set;
	u32	n_fcu_set;
};

enum uhs2_act {
	SET_CONFIG,
	ENABLE_INT,
	DISABLE_INT,
	SET_SPEED_B,
	CHECK_DORMANT,
};

/* UHS-II Device Registers */
#define UHS2_DEV_CONFIG_REG	0x000

/* General Caps and Settings registers */
#define  UHS2_DEV_CONFIG_GEN_CAPS	(UHS2_DEV_CONFIG_REG + 0x000)
#define   UHS2_DEV_CONFIG_N_LANES_POS	8
#define   UHS2_DEV_CONFIG_N_LANES_MASK	0x3F
#define   UHS2_DEV_CONFIG_2L_HD_FD	0x1
#define   UHS2_DEV_CONFIG_2D1U_FD	0x2
#define   UHS2_DEV_CONFIG_1D2U_FD	0x4
#define   UHS2_DEV_CONFIG_2D2U_FD	0x8
#define   UHS2_DEV_CONFIG_DADR_POS	14
#define   UHS2_DEV_CONFIG_DADR_MASK	0x1
#define   UHS2_DEV_CONFIG_APP_POS	16
#define   UHS2_DEV_CONFIG_APP_MASK	0xFF
#define   UHS2_DEV_CONFIG_APP_SD_MEM	0x1

#define  UHS2_DEV_CONFIG_GEN_SET	(UHS2_DEV_CONFIG_REG + 0x008)
#define   UHS2_DEV_CONFIG_GEN_SET_2L_FD_HD	0x0
#define   UHS2_DEV_CONFIG_GEN_SET_CFG_COMPLETE	(0x1 << 31)

/* PHY Caps and Settings registers */
#define  UHS2_DEV_CONFIG_PHY_CAPS	(UHS2_DEV_CONFIG_REG + 0x002)
#define   UHS2_DEV_CONFIG_PHY_MINOR_MASK	0xF
#define   UHS2_DEV_CONFIG_PHY_MAJOR_POS		4
#define   UHS2_DEV_CONFIG_PHY_MAJOR_MASK	0x3
#define   UHS2_DEV_CONFIG_CAN_HIBER_POS		15
#define   UHS2_DEV_CONFIG_CAN_HIBER_MASK	0x1
#define  UHS2_DEV_CONFIG_PHY_CAPS1	(UHS2_DEV_CONFIG_REG + 0x003)
#define   UHS2_DEV_CONFIG_N_LSS_SYN_MASK	0xF
#define   UHS2_DEV_CONFIG_N_LSS_DIR_POS		4
#define   UHS2_DEV_CONFIG_N_LSS_DIR_MASK	0xF

#define  UHS2_DEV_CONFIG_PHY_SET	(UHS2_DEV_CONFIG_REG + 0x00A)
#define   UHS2_DEV_CONFIG_PHY_SET_SPEED_POS	6
#define   UHS2_DEV_CONFIG_PHY_SET_SPEED_A	0x0
#define   UHS2_DEV_CONFIG_PHY_SET_SPEED_B	0x1

/* LINK-TRAN Caps and Settins registers */
#define  UHS2_DEV_CONFIG_LINK_TRAN_CAPS	(UHS2_DEV_CONFIG_REG + 0x004)
#define   UHS2_DEV_CONFIG_LT_MINOR_MASK		0xF
#define   UHS2_DEV_CONFIG_LT_MAJOR_POS		4
#define   UHS2_DEV_CONFIG_LT_MAJOR_MASK		0x3
#define   UHS2_DEV_CONFIG_N_FCU_POS		8
#define   UHS2_DEV_CONFIG_N_FCU_MASK		0xFF
#define   UHS2_DEV_CONFIG_DEV_TYPE_POS		16
#define   UHS2_DEV_CONFIG_DEV_TYPE_MASK		0x7
#define   UHS2_DEV_CONFIG_MAX_BLK_LEN_POS	20
#define   UHS2_DEV_CONFIG_MAX_BLK_LEN_MASK	0xFFF
#define  UHS2_DEV_CONFIG_LINK_TRAN_CAPS1	(UHS2_DEV_CONFIG_REG + 0x005)
#define   UHS2_DEV_CONFIG_N_DATA_GAP_MASK	0xFF

#define  UHS2_DEV_CONFIG_LINK_TRAN_SET	(UHS2_DEV_CONFIG_REG + 0x00C)
#define   UHS2_DEV_CONFIG_LT_SET_MAX_BLK_LEN	0x200
#define   UHS2_DEV_CONFIG_LT_SET_MAX_RETRY_POS	16

/* Preset register */
#define  UHS2_DEV_CONFIG_PRESET		(UHS2_DEV_CONFIG_REG + 0x040)

#define UHS2_DEV_INT_REG	0x100

#define UHS2_DEV_STATUS_REG	0x180

#define UHS2_DEV_CMD_REG	0x200
#define  UHS2_DEV_CMD_FULL_RESET	(UHS2_DEV_CMD_REG + 0x000)
#define  UHS2_DEV_CMD_GO_DORMANT_STATE	(UHS2_DEV_CMD_REG + 0x001)
#define   UHS2_DEV_CMD_DORMANT_HIBER	(0x1 << 7)
#define  UHS2_DEV_CMD_DEVICE_INIT	(UHS2_DEV_CMD_REG + 0x002)
#define  UHS2_DEV_CMD_ENUMERATE		(UHS2_DEV_CMD_REG + 0x003)
#define  UHS2_DEV_CMD_TRANS_ABORT	(UHS2_DEV_CMD_REG + 0x004)

#define UHS2_RCLK_MAX	52000000
#define UHS2_RCLK_MIN	26000000

#endif /* LINUX_MMC_UHS2_H */
