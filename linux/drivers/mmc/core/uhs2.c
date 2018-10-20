/*
 *  linux/drivers/mmc/core/uhs2.c
 *
 *  Copyright (C) 2014 Intel Corp, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mmc/uhs2.h>

#include "uhs2.h"
#include "mmc_ops.h"
#include "sd_ops.h"
#include "core.h"

#define BYTESWAP_32(x) ((((x)&0xFF)<<24) \
         |(((x)>>24)&0xFF) \
         |(((x)&0x0000FF00)<<8)    \
         |(((x)&0x00FF0000)>>8)    )

#define UHS2_ERROR_CHECK(x) \
	if (err) { \
		pr_err("%s: %s: %s = 0x%x!\n", \
			mmc_hostname(host), __func__, x,err); \
		return -EIO; \
	}

#ifdef CONFIG_MMC_DEBUG
	#define DBG(f, x...) \
		pr_warn("[%s()]: " f, __func__, ## x)

	#define UHS2_CHECK_RESP(x) \
	{ \
		int i; \
		pr_warn("%s: %s is: ", mmc_hostname(host), x); \
		for (i = 0; i < resp_len; i++) \
			pr_warn("  0x%02x ", resp[i]); \
		pr_warn("\n"); \
	}
#else
	#define DBG(f, x...)
	#define UHS2_CHECK_RESP(x)
#endif

/*
 * TODO: payload, uhs2_cmd are all allocated which should be freed when
 * response is got.
 * resp is inputted outside which should be a variable created by caller
 * so caller should handle it. For SD command, there is no uhs2_resp and
 * response should be stored in resp of mmc_command.
 */
int uhs2_cmd_assemble(struct mmc_command *cmd, u16 header, u16 arg,
			u32 *payload, u8 plen, u8 *resp, u8 resp_len)
{
	struct uhs2_command *uhs2_cmd;

	if (cmd == NULL)
		return -EIO;

	uhs2_cmd = kzalloc(sizeof(struct uhs2_command),
						GFP_KERNEL);
	if (!uhs2_cmd) {
		if (payload)
			kfree(payload);
		return -ENOMEM;
	}

	uhs2_cmd->header = header;
	uhs2_cmd->arg = arg;
	uhs2_cmd->payload = payload;
	uhs2_cmd->payload_len = plen * sizeof(u32);
	uhs2_cmd->packet_len = uhs2_cmd->payload_len + 4;

	cmd->uhs2_cmd = uhs2_cmd;
	cmd->uhs2_resp = resp;
	cmd->uhs2_resp_len = resp_len;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: uhs2_cmd->header = 0x%x, uhs2_cmd->arg = 0x%x,"
		" uhs2_cmd->payload_len = %d, uhs2_cmd->packet_len = %d,"
		" resp_len = %d.\n",
		__func__, uhs2_cmd->header,
		uhs2_cmd->arg, uhs2_cmd->payload_len, uhs2_cmd->packet_len,
		cmd->uhs2_resp_len);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(uhs2_cmd_assemble);

int uhs2_prepare_sd_cmd(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	u16 header = 0, arg = 0;
	u32 *payload;
	u8 plen = 0;
	int err = 0;

	cmd = mrq->cmd;
	header = host->uhs2_dev_prop.node_id;
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC)
		header |= UHS2_PACKET_TYPE_DCMD;
	else
		header |= UHS2_PACKET_TYPE_CCMD;
	DBG("header = 0x%x.\n", header);

	arg = cmd->opcode << UHS2_SD_CMD_INDEX_POS;
	if (host->flags & MMC_UHS2_APP_CMD) {
		arg |= UHS2_SD_CMD_APP;
		host->flags &= ~MMC_UHS2_APP_CMD;
	}


	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		/* TODO: do not understand. It may relates with ADMA3. */
		if (cmd->data->blocks > 1) {
			payload = kzalloc(2*sizeof(u32), GFP_KERNEL);
			if (!payload)
				return -ENOMEM;

			plen =  8 / sizeof(u32);

			if (host->flags & MMC_UHS2_2L_HD)
				arg |= UHS2_DCMD_2L_HD_MODE;

			arg |= UHS2_DCMD_LM_TLEN_EXIST;
			payload[0] = 0;
			payload[1] = uhs2_dcmd_convert_msb(cmd->data->blocks);

		} else {
			payload = kzalloc(1*sizeof(u32), GFP_KERNEL);
			if (!payload)
				return -ENOMEM;
			plen =  4 / sizeof(u32);
			payload[0] = 0;
		}

		if (cmd->opcode == SD_IO_RW_EXTENDED) {
			arg &= ~(UHS2_DCMD_LM_TLEN_EXIST |
				UHS2_DCMD_TLUM_BYTE_MODE |
				UHS2_NATIVE_DCMD_DAM_IO);
			payload[1] = 0;
			plen = 4 / sizeof(u32);
		}
	} else {
		payload = kzalloc(1 * sizeof(u32), GFP_KERNEL);
		if (!payload)
			return -ENOMEM;
		plen = 4 / sizeof(u32);
	}

	payload[0] = uhs2_dcmd_convert_msb(cmd->arg);
#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: sd_cmd->arg = 0x%x, payload[0]= 0x%x.\n",
		 mmc_hostname(host), __func__, cmd->arg, payload[0]);
#endif

	err = uhs2_cmd_assemble(cmd, header, arg, payload, plen,
				NULL, 0);

	return err;
}
EXPORT_SYMBOL_GPL(uhs2_prepare_sd_cmd);

/*
 * Apply power to the UHS2 stack.  This is a two-stage process.
 * First, we enable power to the card without the clock running.
 * We then wait a bit for the power to stabilise.  Finally,
 * enable the bus drivers and clock to the card.
 *
 * We must _NOT_ enable the clock prior to power stablising.
 *
 * If a host does all the power sequencing itself, ignore the
 * initial MMC_POWER_UP stage.
 */
void uhs2_power_up(struct mmc_host *host)
{
	if (host->ios.power_mode == MMC_POWER_ON)
		return;

	DBG("Enter!\n");

	host->ios.vdd = fls(host->ocr_avail) - 1;
	host->ios.vdd2 = fls(host->ocr_avail_uhs2) - 1;
	if (mmc_host_is_spi(host))
		host->ios.chip_select = MMC_CS_HIGH;
	else
		host->ios.chip_select = MMC_CS_DONTCARE;
	host->ios.clock = host->f_init;
	host->ios.timing = MMC_TIMING_UHS2;
	host->ios.power_mode = MMC_POWER_UP; /* for LED Power ON */
	mmc_set_ios(host);

	host->ios.power_mode = MMC_POWER_ON;
	/*
	 * This delay should be sufficient to allow the power supply
	 * to reach the minimum voltage.
	 */
	mmc_delay(10);
}
EXPORT_SYMBOL_GPL(uhs2_power_up);

static int uhs2_np_resp_check(struct mmc_host *host, u16 arg, u8 *resp, u32 did)
{
	int ecode;

	/* Nack = 1 is only for P2P */
	if (did && (resp[2] & 0x80)) {
		ecode = (resp[2] & 0x70) >> 4;
		if (ecode == 1)
			pr_err("%s: %s: response is COND_ERR !\n",
			mmc_hostname(host), __func__);
		else if (ecode == 2)
			pr_err("%s: %s: response is ARG_ERR !\n",
			mmc_hostname(host), __func__);
		else if (ecode == 3)
			pr_err("%s: %s: response is GEN_ERR !\n",
			mmc_hostname(host), __func__);
		else
			pr_err("%s: %s: response is ERR(ecode = 0x%X) !\n",
			mmc_hostname(host), __func__, ecode);
		return -EIO;
	}

	if ((resp[2] & 0x7F) != (arg & 0x7F) ||
	     resp[3] != ((arg >> 8) & 0xFF)) {
		pr_err("%s: %s: response is CMD_ECHO_BACK ERR!\n",
			mmc_hostname(host), __func__);
		return -EIO;
	}

	return 0;
}


static int uhs2_dev_init(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u32 cnt;
	u32 dap, gap, gap1;
	u16 header = 0, arg = 0;
	u32 *payload;
	u8 plen = 1;
	u8 gd = 0, cf = 1;
	u8 resp[8] = {0};
	u8 resp_len = 8;
	int err;
	u8 did = 0;

	dap = host->uhs2_caps.dap;
	gap = host->uhs2_caps.gap;
	DBG("dap = 0x%x, gap = 0x%x.\n", dap, gap);

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CMD_DEVICE_INIT & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE |
		UHS2_NATIVE_CMD_PLEN_4B |
		(UHS2_DEV_CMD_DEVICE_INIT >> 8);

	for (cnt = 0; cnt < 30; cnt++) {
		payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);
		if (!payload)
			return -ENOMEM;
		payload[0] = ((dap & 0xF) << 12) |
					 (cf << 11) |
					 ((gd  & 0xF) << 4) |
					 (gap & 0xF);

		err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen,
				resp, resp_len);
		UHS2_ERROR_CHECK("UHS2 CMD assembling err");

		DBG("Begin DEVICE_INIT, header=0x%x, arg=0x%x, payload=0x%x.\n",
			header, arg, payload[0]);
		DBG("Sending DEVICE_INIT. Count = %d\n", cnt);

		err = mmc_wait_for_cmd(host, &cmd, 0);
		UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

		UHS2_CHECK_RESP("DEVICE_INIT response");
		err = uhs2_np_resp_check(host, arg, resp, did);
		UHS2_ERROR_CHECK("DEVICE_INIT response is wrong!");

		if (resp[5] & 0x8) {
			DBG("CF is set, device is initialized!\n");
			host->group_desc = gd;
			break;
		} else {
			gap1 = resp[4] & 0x0F;
			if (gap == gap1)
				gd++;
		}
	}
	if (30 == cnt) {
		pr_err("%s: DEVICE_INIT fail, already 30 times!\n",
			mmc_hostname(host));
		return -EIO;
	}

	return 0;
}

static int uhs2_enum(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 *payload;
	u8 plen = 1;
	u8 id_f = 0xF, id_l = 0x0;
	u8 resp[8] = {0};
	u8 resp_len = 8;
	u8 did = 0;
	int err;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CMD_ENUMERATE & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE |
		UHS2_NATIVE_CMD_PLEN_4B |
		(UHS2_DEV_CMD_ENUMERATE >> 8);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	payload[0] = (id_f << 4) | id_l;

	DBG("Begin ENUMERATE, header=0x%x, arg=0x%x, payload=0x%x.\n",
		header, arg, payload[0]);
	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("ENUMERATE response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("ENUMERATE response is wrong!");

	/* TODO: shall I keep id_f or id_l as device node id?
	 * For P2P connection, I think id_f is ok.
	 *  => payload:id_l=0 then resp:id_f and resp:id_l are same
	 */
	id_f = (resp[4] >> 4) & 0xF;
	id_l = resp[4] & 0xF;
	DBG("id_f = %d, id_l = %d.\n", id_f, id_l);
	DBG("Enumerate Cmd Completed. No. of Devices connected = %d\n",
		id_l - id_f + 1);
	host->uhs2_dev_prop.node_id = id_f;

	return 0;
}


static int uhs2_phy_inquiry_config(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 cap;
	int err;
	u32 *payload;
	u8 plen = 2;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = 0;
	u32 n_lss_sync;
	u32 n_lss_dir;
	u32 hibernate;
	u32 phy_major_rev;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CONFIG_PHY_CAPS & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ                   |
		UHS2_NATIVE_CMD_PLEN_8B                |
		(UHS2_DEV_CONFIG_PHY_CAPS >> 8);

	DBG("Begin PHY_INQUIRY_CFG, header=0x%x, arg=0x%x.\n", header, arg);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);

	n_lss_sync    = host->uhs2_caps.n_lss_sync;
	n_lss_dir     = host->uhs2_caps.n_lss_dir;
	hibernate     = 1;
	phy_major_rev = host->uhs2_caps.phy_rev & 0x18;
	/* payload format Byte4[31:24] Byte5[23:16] Byte6[15:8] Byte7[7:0] */
	payload[0] = (hibernate << UHS2_DEV_CONFIG_CAN_HIBER_POS) | phy_major_rev;
	payload[0] = BYTESWAP_32(payload[0]);
	payload[1] = (n_lss_dir << UHS2_DEV_CONFIG_N_LSS_DIR_POS) | n_lss_sync;
	payload[1] = BYTESWAP_32(payload[1]);

	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen, resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("PHY_INQUIRY_CFG response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("PHY_INQUIRY_CFG response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device PHY_INQUIRY_CFG (0-31) is: 0x%x.\n", cap);

	hibernate = (cap >> UHS2_DEV_CONFIG_CAN_HIBER_POS) &
				UHS2_DEV_CONFIG_CAN_HIBER_MASK;

	host->uhs2_set_data.hibernate_set = hibernate;
	host->uhs2_set_data.phy_major_rev_set =
		(cap >> UHS2_DEV_CONFIG_PHY_MAJOR_POS) &
			UHS2_DEV_CONFIG_PHY_MAJOR_MASK;

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];

	DBG("Device PHY_INQUIRY_CFG (32-63) is: 0x%x.\n", cap);
	n_lss_sync = cap & UHS2_DEV_CONFIG_N_LSS_SYN_MASK;
	n_lss_dir = (cap >> UHS2_DEV_CONFIG_N_LSS_DIR_POS) &
				UHS2_DEV_CONFIG_N_LSS_DIR_MASK;

	host->uhs2_set_data.n_lss_sync_set = n_lss_sync;
	host->uhs2_set_data.n_lss_dir_set  = n_lss_dir;
	host->uhs2_set_data.speed_range_set = host->uhs2_caps.speed_range;

	return 0;
}

static int uhs2_link_tran_inquiry_config(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 cap;
	int err;
	u32 *payload;
	u8 plen = 2;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = 0;
	u32 n_data_gap;
	u32 maxblk_len;
	u32 n_fcu;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CONFIG_LINK_TRAN_CAPS & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ                         |
		UHS2_NATIVE_CMD_PLEN_8B                      |
		(UHS2_DEV_CONFIG_LINK_TRAN_CAPS >> 8);

	DBG("Begin LINK_TRAN_INQUIRY_CFG, header=0x%x, arg=0x%x.\n", header, arg);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);

	n_data_gap = host->uhs2_caps.n_data_gap;
	maxblk_len = host->uhs2_caps.maxblk_len;
	n_fcu      = host->uhs2_caps.n_fcu;
	/* payload format Byte4[31:24] Byte5[23:16] Byte6[15:8] Byte7[7:0] */
	payload[0] = (maxblk_len << UHS2_DEV_CONFIG_MAX_BLK_LEN_POS) |
		     (n_fcu << UHS2_DEV_CONFIG_N_FCU_POS);
	payload[0] = BYTESWAP_32(payload[0]);
	payload[1] = n_data_gap;
	payload[1] = BYTESWAP_32(payload[1]);

	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen, resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("LINK_TRAN_INQUIRY_CFG response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("LINK_TRAN_INQUIRY_CFG response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device LINK_TRAN_INQUIRY_CFG (0-31) is: 0x%x.\n", cap);

	n_fcu = (cap >> UHS2_DEV_CONFIG_N_FCU_POS) &
			UHS2_DEV_CONFIG_N_FCU_MASK;
	host->uhs2_set_data.n_fcu_set = n_fcu;

	if (host->uhs2_dev_prop.app_type == UHS2_DEV_CONFIG_APP_SD_MEM)
		maxblk_len = UHS2_DEV_CONFIG_LT_SET_MAX_BLK_LEN;
	else
		maxblk_len = (cap >> UHS2_DEV_CONFIG_MAX_BLK_LEN_POS) &
				UHS2_DEV_CONFIG_MAX_BLK_LEN_MASK;

	host->uhs2_set_data.maxblk_len_set = maxblk_len;

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("Device LINK_TRAN_INQUIRY_CFG (32-63) is: 0x%x.\n", cap);
	n_data_gap = cap & UHS2_DEV_CONFIG_N_DATA_GAP_MASK;
	host->uhs2_set_data.n_data_gap_set = n_data_gap;

	host->uhs2_set_data.max_retry_set  = 3;

	return 0;
}

static int uhs2_inquiry_config(struct mmc_host *host)
{
	int err;
	err = uhs2_phy_inquiry_config(host);
	if (err)
		return err;

	err = uhs2_link_tran_inquiry_config(host);
	if (err)
		return err;

	return 0;
}


static int uhs2_phy_set_config(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 cap;
	int err;
	u32 *payload;
	u8  plen = 2;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = 0;
	u32 n_lss_sync;
	u32 n_lss_dir;
	u32 phy_major_rev;
	u32 speed_range;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CONFIG_PHY_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE                 |
		UHS2_NATIVE_CMD_PLEN_8B               |
		(UHS2_DEV_CONFIG_PHY_SET >> 8);

	DBG("Begin PHY_SET_CFG, header=0x%x, arg=0x%x.\n", header, arg);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);

	n_lss_sync    = host->uhs2_set_data.n_lss_sync_set;
	n_lss_dir     = host->uhs2_set_data.n_lss_dir_set;
	speed_range   = host->uhs2_set_data.speed_range_set;
	phy_major_rev = host->uhs2_set_data.phy_major_rev_set;
	/* payload format Byte4[31:24] Byte5[23:16] Byte6[15:8] Byte7[7:0] */
	payload[0] = (speed_range   << UHS2_DEV_CONFIG_PHY_SET_SPEED_POS) |
		     (phy_major_rev << UHS2_DEV_CONFIG_PHY_MAJOR_POS);
	payload[0] = BYTESWAP_32(payload[0]);
	payload[1] = (n_lss_dir << UHS2_DEV_CONFIG_N_LSS_DIR_POS) |
		      n_lss_sync;
	payload[1] = BYTESWAP_32(payload[1]);

	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen, resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("PHY_SET_CONFIG response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("PHY_SET_CONFIG response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device PHY SET CONFIG (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("Device PHY SET CONFIG (32-63) is: 0x%x.\n", cap);

	return 0;
}

static int uhs2_link_tran_set_config(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 cap;
	int err;
	u32 *payload;
	u8  plen = 2;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = 0;
	u32 n_data_gap;
	u32 maxblk_len;
	u32 n_fcu;
	u32 max_retry_num;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CONFIG_LINK_TRAN_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE                       |
		UHS2_NATIVE_CMD_PLEN_8B                     |
		(UHS2_DEV_CONFIG_LINK_TRAN_SET >> 8);

	DBG("Begin LINK_TRAN_SET_CFG, header=0x%x, arg=0x%x.\n", header, arg);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);

	n_data_gap    = host->uhs2_set_data.n_data_gap_set;
	maxblk_len    = host->uhs2_set_data.maxblk_len_set;
	n_fcu         = host->uhs2_set_data.n_fcu_set;
	max_retry_num = host->uhs2_set_data.max_retry_set;
	/* payload format Byte4[31:24] Byte5[23:16] Byte6[15:8] Byte7[7:0] */
	payload[0] = (maxblk_len    << UHS2_DEV_CONFIG_MAX_BLK_LEN_POS) |
		     (max_retry_num << UHS2_DEV_CONFIG_LT_SET_MAX_RETRY_POS) |
		     (n_fcu         << UHS2_DEV_CONFIG_N_FCU_POS);
	payload[0] = BYTESWAP_32(payload[0]);
	payload[1] = n_data_gap;
	payload[1] = BYTESWAP_32(payload[1]);

	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen, resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("LINK_TRAN_SET_CFG response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("LINK_TRAN_SET_CFG response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device LINK_TRAN_SET_CFG (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("Device LINK_TRAN_SET_CFG (32-63) is: 0x%x.\n", cap);

	return 0;
}

static int uhs2_general_set_config(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 cap;
	int err;
	u32 *payload;
	u8  plen = 2;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = 0;
	u32 power_mode = 0; /* Fast mode */
	u32 number_of_lanes = UHS2_DEV_CONFIG_GEN_SET_2L_FD_HD;

	host->uhs2_set_data.n_lanes_set      = number_of_lanes;
	host->uhs2_set_data.pwrctrl_mode_set = power_mode;
	host->uhs2_set_data.cfg_complete     = 1;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CONFIG_GEN_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE                 |
		UHS2_NATIVE_CMD_PLEN_8B               |
	       (UHS2_DEV_CONFIG_GEN_SET >> 8);

	DBG("Begin GENERAL_SET_CFG, header=0x%x, arg=0x%x.\n", header, arg);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);

	/* payload format Byte4[31:24] Byte5[23:16] Byte6[15:8] Byte7[7:0] */
	payload[0] = (number_of_lanes << UHS2_DEV_CONFIG_N_LANES_POS) |
		      power_mode;
	payload[0] = BYTESWAP_32(payload[0]);
	payload[1] = UHS2_DEV_CONFIG_GEN_SET_CFG_COMPLETE;
	payload[1] = BYTESWAP_32(payload[1]);

	err = uhs2_cmd_assemble(&cmd, header, arg, payload,
					plen, resp, resp_len);

	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("GENERAL_SET_CFG response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("GENERAL_SET_CFG response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device GENERAL_SET_CFG (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("Device GENERAL_SET_CFG (32-63) is: 0x%x.\n", cap);

	return 0;
}

#ifdef CONFIG_MMC_DEBUG
static int uhs2_setting_read(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = host->uhs2_dev_prop.node_id;
	u32 cap;
	int err;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;

	arg = ((UHS2_DEV_CONFIG_GEN_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ                   |
		UHS2_NATIVE_CMD_PLEN_8B                |
	       (UHS2_DEV_CONFIG_GEN_SET >> 8);

	DBG("Begin READ_SETTINGS GENERIC: header=0x%x, arg=0x%x.\n",
		header, arg);
	/* There is no payload because per spec, there should be
	 * no payload field for read CCMD.
	 * Plen is set in arg. Per spec, plen for read CCMD
	 * represents the len of read data which is assigned in payload
	 * of following RES (p136).
	 */
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_SETTINGS GENERIC response");

	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_SETTINGS GENERIC response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("DEVICE GENERIC Set (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("DEVICE GENERIC Set (32-63) is: 0x%x.\n", cap);

	arg = ((UHS2_DEV_CONFIG_PHY_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ |
		UHS2_NATIVE_CMD_PLEN_8B |
		(UHS2_DEV_CONFIG_PHY_SET >> 8);
	DBG("Begin READ_SETTINGS PHY: header=0x%x, arg=0x%x.\n",
		header, arg);
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_SETTINGS PHY response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_SETTINGS PHY response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("DEVICE PHY SETTINGS (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("DEVICE PHY SETTINGS (32-63) is: 0x%x.\n", cap);

	arg = ((UHS2_DEV_CONFIG_LINK_TRAN_SET & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ |
		UHS2_NATIVE_CMD_PLEN_8B |
		(UHS2_DEV_CONFIG_LINK_TRAN_SET >> 8);

	DBG("Begin READ_SETTINGS LINK-TRAN header=0x%x, arg=0x%x.\n",
		header, arg);
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_SETTINGS LINK-TRAN response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_SETTINGS LINK-TRAN response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("DEVICE LINK-TRAN SETTINGS (0-31) is: 0x%x.\n", cap);

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("DEVICE LINK-TRAN SETTINGS (32-63) is: 0x%x.\n", cap);

	return 0;
}
#endif

static int uhs2_set_config(struct mmc_host *host)
{
	int err;
	err = uhs2_phy_set_config(host);
	if (err)
		return err;

	err = uhs2_link_tran_set_config(host);
	if (err)
		return err;

	err = uhs2_general_set_config(host);
	if (err)
		return err;

#ifdef CONFIG_MMC_DEBUG
	/* Check Settings */
	uhs2_setting_read(host);
#endif

	/* Set host Config Setting registers */
	if (host->uhs2_dev_prop.n_lanes == UHS2_DEV_CONFIG_2L_HD_FD &&
	    host->uhs2_caps.n_lanes == UHS2_DEV_CONFIG_2L_HD_FD) {
		DBG("Both Host and device support 2L-HD.\n");
		host->flags |= MMC_UHS2_2L_HD;
	} else {
		/* Only support 2L-FD so far */
		DBG("Support 2L-FD.\n");
		host->flags &= ~MMC_UHS2_2L_HD;
	}

	if (host->uhs2_caps.speed_range == UHS2_DEV_CONFIG_PHY_SET_SPEED_B)
		host->flags |= MMC_UHS2_SPEED_B;
	else
		host->flags &= ~MMC_UHS2_SPEED_B;

	if (host->ops->uhs2_set_reg(host, SET_CONFIG)) {
		pr_err("%s: %s: UHS2 SET_CONFIG fail!\n",
			mmc_hostname(host), __func__);
		return -EIO;
	}

	return 0;
}

static int uhs2_config_read(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u8 resp[12] = {0};
	u8 resp_len = 12;
	u8 did = host->uhs2_dev_prop.node_id;
	u32 cap;
	int err;

	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;

	arg = ((UHS2_DEV_CONFIG_GEN_CAPS & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ                   |
		UHS2_NATIVE_CMD_PLEN_4B                |
	       (UHS2_DEV_CONFIG_GEN_CAPS >> 8);

	DBG("Begin READ_CAPABILITIES GENERIC: header=0x%x, arg=0x%x.\n",
		header, arg);
	/* There is no payload because per spec, there should be
	 * no payload field for read CCMD.
	 * Plen is set in arg. Per spec, plen for read CCMD
	 * represents the len of read data which is assigned in payload
	 * of following RES (p136).
	 */
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_CAPABILITIES GENERIC response");

	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_CAPABILIYIES GENERIC response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("DEVICE GENERIC CAPABILITIES (0-31) is: 0x%x.\n", cap);
	host->uhs2_dev_prop.n_lanes = (cap >> UHS2_DEV_CONFIG_N_LANES_POS) &
					UHS2_DEV_CONFIG_N_LANES_MASK;
	host->uhs2_dev_prop.dadr_len = (cap >> UHS2_DEV_CONFIG_DADR_POS) &
					UHS2_DEV_CONFIG_DADR_MASK;
	host->uhs2_dev_prop.app_type = (cap >> UHS2_DEV_CONFIG_APP_POS) &
					UHS2_DEV_CONFIG_APP_MASK;

	arg = ((UHS2_DEV_CONFIG_PHY_CAPS & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ |
		UHS2_NATIVE_CMD_PLEN_8B |
		(UHS2_DEV_CONFIG_PHY_CAPS >> 8);
	DBG("Begin READ_CAPABILITIES PHY: header=0x%x, arg=0x%x.\n",
		header, arg);
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_CAPABILITIES PHY response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_CAPABILITIES PHY response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("Device PHY CAPABILITIES (0-31) is: 0x%x.\n", cap);
	host->uhs2_dev_prop.phy_minor_rev = cap &
					UHS2_DEV_CONFIG_PHY_MINOR_MASK;
	host->uhs2_dev_prop.phy_major_rev = (cap >>
					UHS2_DEV_CONFIG_PHY_MAJOR_POS) &
					UHS2_DEV_CONFIG_PHY_MAJOR_MASK;
	host->uhs2_dev_prop.can_hibernate = (cap >>
					UHS2_DEV_CONFIG_CAN_HIBER_POS) &
					UHS2_DEV_CONFIG_CAN_HIBER_MASK;

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("DEVICE PHY CAPABILITIES (32-63) is: 0x%x.\n", cap);
	host->uhs2_dev_prop.n_lss_sync = cap & UHS2_DEV_CONFIG_N_LSS_SYN_MASK;
	host->uhs2_dev_prop.n_lss_dir  = (cap >>
					  UHS2_DEV_CONFIG_N_LSS_DIR_POS) &
					  UHS2_DEV_CONFIG_N_LSS_DIR_MASK;

	arg = ((UHS2_DEV_CONFIG_LINK_TRAN_CAPS & 0xFF) << 8) |
		UHS2_NATIVE_CMD_READ |
		UHS2_NATIVE_CMD_PLEN_8B |
		(UHS2_DEV_CONFIG_LINK_TRAN_CAPS >> 8);

	DBG("Begin READ_CAPABILITIES LINK-TRAN: header=0x%x, arg=0x%x.\n",
		header, arg);
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, 0,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("READ_CAPABILITIES Link-Tran response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("READ_CAPABILITIES Link-Tran response is wrong!");

	cap = resp[4] << 24 | resp[5] << 16 | resp[6] << 8 | resp[7];
	DBG("DEVICE LINK-TRAN CAPABILITIES (0-31) is: 0x%x.\n", cap);
	host->uhs2_dev_prop.link_minor_rev = cap &
					UHS2_DEV_CONFIG_LT_MINOR_MASK;
	host->uhs2_dev_prop.link_major_rev = (cap >>
					UHS2_DEV_CONFIG_LT_MAJOR_POS) &
					UHS2_DEV_CONFIG_LT_MAJOR_MASK;
	host->uhs2_dev_prop.n_fcu = (cap >> UHS2_DEV_CONFIG_N_FCU_POS) &
					UHS2_DEV_CONFIG_N_FCU_MASK;
	host->uhs2_dev_prop.dev_type = (cap >> UHS2_DEV_CONFIG_DEV_TYPE_POS) &
					UHS2_DEV_CONFIG_DEV_TYPE_MASK;
	host->uhs2_dev_prop.maxblk_len = (cap >>
					UHS2_DEV_CONFIG_MAX_BLK_LEN_POS) &
					UHS2_DEV_CONFIG_MAX_BLK_LEN_MASK;

	cap = resp[8] << 24 | resp[9] << 16 | resp[10] << 8 | resp[11];
	DBG("Device LINK-TRAN CAPABILITIES (32-63) is: 0x%x.\n", cap);
	host->uhs2_dev_prop.n_data_gap = cap & UHS2_DEV_CONFIG_N_DATA_GAP_MASK;

	return 0;
}


static int uhs2_go_dormant(struct mmc_host *host, bool hibernate)
{
	struct mmc_command cmd = {0};
	u16 header = 0, arg = 0;
	u32 *payload;
	u8 plen = 1;
	u8 resp[8] = {0};
	u8 resp_len = 8;
	u8 did = host->uhs2_dev_prop.node_id;
	int err;

	BUG_ON(!host->ops->uhs2_set_reg);

	/* Disable Normal INT */
	err = host->ops->uhs2_set_reg(host, DISABLE_INT);
	UHS2_ERROR_CHECK("UHS2 DISABLE_INT fail!");

	/* TODO: shall I use host->uhs2_dev_prop.node_id here? */
	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;

	arg = ((UHS2_DEV_CMD_GO_DORMANT_STATE & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE |
		UHS2_NATIVE_CMD_PLEN_4B |
		(UHS2_DEV_CMD_GO_DORMANT_STATE >> 8);

	payload = kcalloc(plen, sizeof(u32), GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	if (hibernate)
		payload[0] = UHS2_DEV_CMD_DORMANT_HIBER;

	DBG("Begin GO_DORMANT_STATE, header=0x%x, arg=0x%x, payload=0x%x.\n",
		header, arg, payload[0]);
	err = uhs2_cmd_assemble(&cmd, header, arg, payload, plen,
				resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");

	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("GO_DORMANT_STATE response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("GO_DORMANT_STATE response is wrong!");

	/* Check Dormant State in Present */
	err = host->ops->uhs2_set_reg(host, CHECK_DORMANT);
	UHS2_ERROR_CHECK("UHS2 GO_DORMANT_STATE fail!");

	host->ios.clock = 0;
	/* TODO: Not use hibernate=1, if use,
		 we must care host->ios.vdd = -1 at sdhci_set_power() */
	if (hibernate)
		host->ios.vdd = -1;
	mmc_set_ios(host);

	return 0;
}

int uhs2_trans_abort(struct mmc_host *host)
{
	struct mmc_command cmd = { 0 };
	u16 header = 0, arg = 0;
	u8 plen = 0;
	u8 resp[8] = { 0 };
	u8 resp_len = 8;
	u8 did = host->uhs2_dev_prop.node_id;
	int err;

	BUG_ON(!host->ops->uhs2_set_reg);

	/* Disable Normal INT */
	err = host->ops->uhs2_set_reg(host, DISABLE_INT);
	UHS2_ERROR_CHECK("UHS2 DISABLE_INT fail!");

	/* TODO: shall I use host->uhs2_dev_prop.node_id here? */
	header = UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did;
	arg = ((UHS2_DEV_CMD_TRANS_ABORT & 0xFF) << 8) |
		UHS2_NATIVE_CMD_WRITE |
		(UHS2_DEV_CMD_TRANS_ABORT >> 8);

	DBG("Begin TRANS_ABORT, header=0x%x, arg=0x%x.\n",
		header, arg);
	err = uhs2_cmd_assemble(&cmd, header, arg, NULL, plen,
		resp, resp_len);
	UHS2_ERROR_CHECK("UHS2 CMD assembling err");
	err = mmc_wait_for_cmd(host, &cmd, 0);
	UHS2_ERROR_CHECK("UHS2 CMD send fail, err");

	UHS2_CHECK_RESP("TRANS_ABORT response");
	err = uhs2_np_resp_check(host, arg, resp, did);
	UHS2_ERROR_CHECK("TRANS_ABORT response is wrong!");

	return err;
}

EXPORT_SYMBOL(uhs2_trans_abort);

static int uhs2_change_speed(struct mmc_host *host)
{
	int err;

	BUG_ON(!host->ops->uhs2_detect_init);
	BUG_ON(!host->ops->uhs2_set_reg);

	err = uhs2_go_dormant(host, false);
	UHS2_ERROR_CHECK("UHS2 GO_DORMANT_STATE fail, err");

	/* Enable Normal INT */
	if (host->ops->uhs2_set_reg(host, ENABLE_INT)) {
		pr_err("%s: %s: UHS2 ENABLE_INT fail!\n",
			mmc_hostname(host), __func__);
		return -EIO;
	}

	/* TODO: if I set clock in sdhci_uhs2_interface_detect(), I should
	 * remove below codes.
	 */
	host->ios.clock = host->f_max;
	mmc_set_ios(host);

	udelay(600);

	if (host->ops->uhs2_detect_init(host)) {
		pr_err("%s: %s: uhs2_detect_init() fail!\n",
			mmc_hostname(host), __func__);
		return -EIO;
	}

	DBG("Change to Speed Range B succeeds.\n");

	return 0;
}

int mmc_uhs2_try_frequency(struct mmc_host *host, unsigned freq)
{
	int err = -EIO;

	BUG_ON(!host->ops->uhs2_detect_init);
	BUG_ON(!host->ops->uhs2_set_reg);

	host->flags |= MMC_UHS2_SUPPORT;
	host->f_init = freq;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: trying to init card at %u Hz\n",
		 mmc_hostname(host), __func__, host->f_init);
#endif

	uhs2_power_up(host);
	if (host->ops->uhs2_detect_init(host)) {
		/* SD30 card comes here */
		pr_info("%s: fail to detect UHS2!\n", mmc_hostname(host));
		err = UHS2_PHY_INIT_ERR;
		goto init_fail;
	}

	if (uhs2_dev_init(host)) {
		pr_err("%s: UHS2 DEVICE_INIT fail!\n", mmc_hostname(host));
		goto init_fail;
	}

	if (uhs2_enum(host)) {
		pr_err("%s: UHS2 ENUMERATE fail!\n", mmc_hostname(host));
		goto init_fail;
	}

	if (uhs2_config_read(host)) {
		pr_err("%s: UHS2 CONFIG_READ fail!\n", mmc_hostname(host));
		goto init_fail;
	}

	if (uhs2_inquiry_config(host)) {
		pr_err("%s: UHS2 INQUIRY_CONFIG fail!\n", mmc_hostname(host));
		goto init_fail;
	}

	if (uhs2_set_config(host)) {
		pr_err("%s: UHS2 SET_CONFIG fail!\n", mmc_hostname(host));
		goto init_fail;
	}

	mmc_delay(10);

	/* Change to Speed Range B if it is supported */
	if (host->flags & MMC_UHS2_SPEED_B)
		if (uhs2_change_speed(host)) {
			pr_err("%s: UHS2 uhs2_change_speed() fail!\n",
				mmc_hostname(host));
			goto init_fail;
		}

	host->flags |= MMC_UHS2_INITIALIZED;

	mmc_go_idle(host);

	mmc_send_if_cond(host, host->ocr_avail);

	/* On market, only can some SD cards support UHS-II so only call SD
	 * attach process here.
	 */
	if (!mmc_attach_sd(host))
		return 0;

init_fail:
	mmc_power_off(host);
	if (host->flags & MMC_UHS2_INITIALIZED)
		host->flags &= ~MMC_UHS2_INITIALIZED;
	host->flags &= ~MMC_UHS2_SUPPORT;

	return err;
}
EXPORT_SYMBOL_GPL(mmc_uhs2_try_frequency);

int  mmc_uhs2_power_limit(struct mmc_card *card)
{
	u8 *status;
	int err = 0;
	struct device_node *np;
	u32 power_limit;
	u32 d_power_limit;

	np = card->host->parent->of_node;

	if (of_property_read_u32(np, "uhs2-power-limit", &power_limit))
		return 0;

	if (power_limit == 2 || power_limit == 3 || power_limit > 4) {
		pr_err("%s: UHS-II Power Limit(%d) is out-of-spec\n"
				, mmc_hostname(card->host), power_limit);
		return 0;
	}

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		pr_err("%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	/* read card Power Limit info */
	err = mmc_sd_switch(card, 0, 0, 0, status);
	if (err)
		goto out;

	d_power_limit = status[7];

	switch (power_limit) {
	case 4:
		if (d_power_limit & 0x10)
			break; /* power_limit = 4 */
		power_limit = 1;
	case 1:
		if (d_power_limit & 0x02)
			break; /* power_limit = 1 */
		power_limit = 0;
	case 0:
		if (d_power_limit & 0x01)
			break; /* power_limit = 0 */
	default:
		return 0;
	}

	/* set Power Limit */
	err = mmc_sd_switch(card, 1, 3, power_limit, status);

	if (((status[15] >> 4) & 0x0F) != power_limit)
		pr_warn("%s: Problem setting power limit!\n",
		mmc_hostname(card->host));

out:
	kfree(status);

	return err;
}
EXPORT_SYMBOL_GPL(mmc_uhs2_power_limit);
