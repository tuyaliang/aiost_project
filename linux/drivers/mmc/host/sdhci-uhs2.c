/*
 *  linux/drivers/mmc/host/sdhci_uhs2.c - Secure Digital Host Controller
 *  Interface driver
 *
 *  Copyright (C) 2014 Intel Corp, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <linux/mmc/mmc.h>

#include "sdhci.h"
#include "sdhci-uhs2.h"

#define DRIVER_NAME "sdhci_uhs2"
#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__, ## x)

//#define SPEED_A_ONLY   /* for debug */
//#define SET_DIR3_SYNC6 /* for debug */
//#define FD_ONLY        /* for debug */

static void sdhci_clear_set_irqs(struct sdhci_host *host, u32 clear, u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	ier &= ~clear;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}

static void sdhci_uhs2_clear_set_irqs(struct sdhci_host *host, u32 clear,
					u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_UHS2_ERR_INT_STATUS_EN);
	ier &= ~clear;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_UHS2_ERR_INT_STATUS_EN);
	sdhci_writel(host, ier, SDHCI_UHS2_ERR_INT_SIG_EN);
}

void sdhci_uhs2_reset(struct sdhci_host *host, u16 mask)
{
	unsigned long timeout;
	u32 uninitialized_var(ier);
	u32 uninitialized_var(ier2);
	int d[4];

	if (!(host->mmc->caps & MMC_CAP_UHS2))
		return;

	/* Keep enable */
	if (mask == SDHCI_UHS2_SW_RESET_SD) {
		d[0] = sdhci_readl(host, SDHCI_INT_ENABLE);
		d[1] = sdhci_readl(host, SDHCI_SIGNAL_ENABLE);
		d[2] = sdhci_readl(host, SDHCI_UHS2_ERR_INT_STATUS_EN);
		d[3] = sdhci_readl(host, SDHCI_UHS2_ERR_INT_SIG_EN);
	}

	sdhci_writew(host, mask, SDHCI_UHS2_SW_RESET); // #define SDHCI_UHS2_SW_RESET	0xC0

	if (mask & SDHCI_UHS2_SW_RESET_FULL) {
		host->clock = 0;
		/* Reset-all turns off SD Bus Power */
		if (host->quirks2 & SDHCI_QUIRK2_CARD_ON_NEEDS_BUS_ON)
			sdhci_runtime_pm_bus_off(host);
	}

	/* Wait max 100 ms */
	timeout = 10000;

	/* hw clears the bit when it's done */
	while (sdhci_readw(host, SDHCI_UHS2_SW_RESET) & mask) {
		if (timeout == 0) {
			pr_err("%s: Reset 0x%x never completed.\n",
				mmc_hostname(host->mmc), (int)mask);
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		udelay(10);
	}


	if (mask == SDHCI_UHS2_SW_RESET_SD) {
		sdhci_writel(host, d[0], SDHCI_INT_ENABLE);
		sdhci_writel(host, d[1], SDHCI_SIGNAL_ENABLE);
		sdhci_writel(host, d[2], SDHCI_UHS2_ERR_INT_STATUS_EN);
		sdhci_writel(host, d[3], SDHCI_UHS2_ERR_INT_SIG_EN);
	}
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_reset);

static u8 sdhci_calc_timeout_uhs2(struct sdhci_host *host,
				u8 *cmd_res,
				u8 *dead_lock)
{
	u8 count;
	unsigned cmd_res_timeout, dead_lock_timeout, current_timeout;

	/*
	 * If the host controller provides us with an incorrect timeout
	 * value, just skip the check and use 0xE.  The hardware may take
	 * longer to time out, but that's much better than having a too-short
	 * timeout value.
	 */
	if (host->quirks & SDHCI_QUIRK_BROKEN_TIMEOUT_VAL) {
		*cmd_res = 0xE;
		*dead_lock = 0xE;
		return 0xE;
	}

	/* timeout in us */
	cmd_res_timeout = 5 * 1000;
	dead_lock_timeout = 1 * 1000 * 1000;

	/*
	 * Figure out needed cycles.
	 * We do this in steps in order to fit inside a 32 bit int.
	 * The first step is the minimum timeout, which will have a
	 * minimum resolution of 6 bits:
	 * (1) 2^13*1000 > 2^22,
	 * (2) host->timeout_clk < 2^16
	 *     =>
	 *     (1) / (2) > 2^6
	 */
	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < cmd_res_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF) {
		DBG("%s: Too large timeout 0x%x requested for CMD_RES!\n",
			mmc_hostname(host->mmc), count);
		count = 0xE;
	}
	*cmd_res = count;

	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < dead_lock_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF) {
		DBG("%s: Too large timeout 0x%x requested for DEADLOCK!\n",
			mmc_hostname(host->mmc), count);
		count = 0xE;
	}
	*dead_lock = count;

	return count;
}

void sdhci_uhs2_set_transfer_mode(struct sdhci_host *host,
	struct mmc_command *cmd)
{
	u16 mode;
	struct mmc_data *data = cmd->data;

	if (data == NULL) {
		/* clear Auto CMD settings for no data CMDs */
		mode = sdhci_readw(host, SDHCI_UHS2_TRANS_MODE);
		if (cmd->opcode == MMC_STOP_TRANSMISSION)
			mode |= SDHCI_UHS2_TRNS_WAIT_EBSY;
#ifdef CONFIG_MMC_DEBUG
		DBG("UHS2 no data trans mode is 0x%x.\n", mode);
#endif
		sdhci_writew(host, mode, SDHCI_UHS2_TRANS_MODE);
		return;
	}

	WARN_ON(!host->data);

	mode = SDHCI_UHS2_TRNS_BLK_CNT_EN | SDHCI_UHS2_TRNS_WAIT_EBSY;
	if (data->flags & MMC_DATA_WRITE)
		mode |= SDHCI_UHS2_TRNS_DATA_TRNS_WRT;

	if (host->flags & SDHCI_REQ_USE_DMA)
		mode |= SDHCI_UHS2_TRNS_DMA;

	if (host->mmc->flags & MMC_UHS2_2L_HD &&
	    (cmd->opcode == MMC_READ_MULTIPLE_BLOCK ||
	     cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK))
		mode |= SDHCI_UHS2_TRNS_2L_HD;

	sdhci_writew(host, mode, SDHCI_UHS2_TRANS_MODE);

#ifdef CONFIG_MMC_DEBUG
	DBG("UHS2 trans mode is 0x%x.\n", mode);
#endif
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_set_transfer_mode);

void sdhci_uhs2_send_command(struct sdhci_host *host, struct mmc_command *cmd)
{
	int i, j;
	int cmd_reg;

	if (host->mmc->flags & MMC_UHS2_INITIALIZED) {
		if (cmd->uhs2_cmd == NULL) {
			pr_err("%s: fatal error, no uhs2_cmd!\n",
				mmc_hostname(host->mmc));
			BUG();
			return;
		}
	}

	for (i = 0; i < SDHCI_UHS2_CMD_PACK_MAX_LEN; i++)
		sdhci_writeb(host, 0, SDHCI_UHS2_CMD_PACKET + i);

	i = 0;
	sdhci_writeb(host, cmd->uhs2_cmd->header,
			SDHCI_UHS2_CMD_PACKET + i++);
	sdhci_writeb(host, (cmd->uhs2_cmd->header >> 8) & 0xFF,
			SDHCI_UHS2_CMD_PACKET + i++);
	sdhci_writeb(host, cmd->uhs2_cmd->arg,
			SDHCI_UHS2_CMD_PACKET + i++);
	sdhci_writeb(host, (cmd->uhs2_cmd->arg >> 8) & 0xFF,
			SDHCI_UHS2_CMD_PACKET + i++);

	/* TODO: Per spec, playload (config) should be MSB before sending out.
	 * But we don't need convert here because I will set payload as
	 * MSB when preparing config read/write commands.
	 */
	for (j = 0; j < cmd->uhs2_cmd->payload_len/sizeof(u32); j++) {
		sdhci_writeb(host,
			(*(cmd->uhs2_cmd->payload + j) & 0xFF),
			SDHCI_UHS2_CMD_PACKET + i++);
		sdhci_writeb(host,
			((*(cmd->uhs2_cmd->payload + j)) >> 8) & 0xFF,
			SDHCI_UHS2_CMD_PACKET + i++);
		sdhci_writeb(host,
			((*(cmd->uhs2_cmd->payload + j)) >> 16) & 0xFF,
			SDHCI_UHS2_CMD_PACKET + i++);
		sdhci_writeb(host,
			((*(cmd->uhs2_cmd->payload + j)) >> 24) & 0xFF,
			SDHCI_UHS2_CMD_PACKET + i++);
	}

#ifdef CONFIG_MMC_DEBUG
	DBG("UHS2 CMD packet_len = %d.\n", cmd->uhs2_cmd->packet_len);
	for (i = 0; i < cmd->uhs2_cmd->packet_len; i++)
		DBG("UHS2 CMD_PACKET[%d] = 0x%x.\n", i,
			sdhci_readb(host, SDHCI_UHS2_CMD_PACKET + i));
#endif

	cmd_reg = cmd->uhs2_cmd->packet_len <<
				SDHCI_UHS2_COMMAND_PACK_LEN_SHIFT;

	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC)
		cmd_reg |= SDHCI_UHS2_COMMAND_DATA;
	if (cmd->opcode == MMC_STOP_TRANSMISSION)
		cmd_reg |= SDHCI_UHS2_COMMAND_CMD12;

	if (cmd->uhs2_cmd->header & UHS2_NATIVE_PACKET) {
		int cmd_tmp;
		cmd_tmp = ((cmd->uhs2_cmd->arg & 0x0F) << 8) |
			((cmd->uhs2_cmd->arg >> 8) & 0xFF);
		/* UHS2 Native ABORT */
		if (cmd_tmp == UHS2_DEV_CMD_TRANS_ABORT)
			cmd_reg |= SDHCI_UHS2_COMMAND_TRNS_ABORT;
		 /* UHS2 Native DORMANT */
		else if (cmd_tmp == UHS2_DEV_CMD_GO_DORMANT_STATE)
			cmd_reg |= SDHCI_UHS2_COMMAND_DORMANT;
	}

	DBG("0x%x is set to UHS2 CMD register.\n", cmd_reg);
	sdhci_writew(host, cmd_reg, SDHCI_UHS2_COMMAND);
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_send_command);

void sdhci_uhs2_finish_command(struct sdhci_host *host)
{
	int i;
	u8 resp;

	if (host->mmc->flags & MMC_UHS2_INITIALIZED) {
		resp = sdhci_readb(host, SDHCI_UHS2_RESPONSE + 2);
		if (resp & UHS2_RES_NACK_MASK)
			pr_err("%s: NACK is got, ECODE=0x%x.\n",
				mmc_hostname(host->mmc),
				resp & UHS2_RES_ECODE_MASK);
	}

	if (host->cmd->uhs2_resp &&
	    host->cmd->uhs2_resp_len &&
	    host->cmd->uhs2_resp_len <= 20)
		/* Get whole response of some native CCMD, like
		 * DEVICE_INIT, ENUMERATE.
		 */
		for (i = 0; i < host->cmd->uhs2_resp_len; i++)
			host->cmd->uhs2_resp[i] = sdhci_readb(host,
				SDHCI_UHS2_RESPONSE + i);
	else
		/* Get SD CMD response and Payload for some read
		 * CCMD, like INQUIRY_CFG.
		 */
		/* Per spec (p136), payload field is divided into
		 * a unit of DWORD and transmission order within
		 * a DWORD is big endian.
		 */
		for (i = 4; i < 20; i += 4)
			host->cmd->resp[i/4-1] = (sdhci_readb(host,
				SDHCI_UHS2_RESPONSE + i) << 24) |
					(sdhci_readb(host,
				SDHCI_UHS2_RESPONSE + i + 1) << 16) |
					(sdhci_readb(host,
				SDHCI_UHS2_RESPONSE + i + 2) << 8) |
					sdhci_readb(host,
				SDHCI_UHS2_RESPONSE + i + 3);
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_finish_command);

void sdhci_uhs2_do_set_ios(struct sdhci_host *host, struct mmc_ios *ios)
{
	u8 cmd_res, dead_lock;
	u16 ctrl_2;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	/* Preset register: no need to handle */
	/* Asynchronous interrupt enable */
	/* TODO: as async interrupt is same as SDHC 3.0
	 * I think we don't need handle it here.
	 */

	/* UHS2 Timeout Control */
	sdhci_calc_timeout_uhs2(host, &cmd_res, &dead_lock);
	cmd_res |= dead_lock << SDHCI_UHS2_TIMER_CTRL_DEADLOCK_SHIFT;
	sdhci_writeb(host, cmd_res, SDHCI_UHS2_TIMER_CTRL);

	/* UHS2 timing */
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	if (ios->timing == MMC_TIMING_UHS2)
		ctrl_2 |= SDHCI_CTRL_UHS_2 | SDHCI_CTRL_UHS2_INTERFACE_EN;
	else
		ctrl_2 &= ~(SDHCI_CTRL_UHS_2 | SDHCI_CTRL_UHS2_INTERFACE_EN);
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (!(host->quirks2 & SDHCI_QUIRK2_PRESET_VALUE_BROKEN))
		sdhci_enable_preset_value(host, true);

	/* Set VDD2 */
	if (ios->power_mode != MMC_POWER_UNDEFINED)
		sdhci_set_power(host, ios->power_mode, ios->vdd, ios->vdd2);

	spin_unlock_irqrestore(&host->lock, flags);
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_do_set_ios);

static int sdhci_uhs2_interface_detect(struct sdhci_host *host)
{
//	int timeout = 20;
	int timeout = 1000;

	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
		SDHCI_UHS2_IF_DETECT)) {
		if (timeout == 0) {
			pr_warn("%s: not detect UHS2 interface in 200us.\n",
				mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return -EIO;
		}
		timeout--;
		udelay(10);
	}

	timeout = 150;
	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
		SDHCI_UHS2_LANE_SYNC)) {
		if (timeout == 0) {
			pr_warn("%s: UHS2 Lane sync fail in 150ms.\n",
				mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return -EIO;
		}
		timeout--;
		mdelay(1);
	}

	DBG("%s: UHS2 Lane synchronized in UHS2 mode, PHY is initializaed.\n",
		mmc_hostname(host->mmc));
	return 0;
}

static int sdhci_uhs2_init(struct sdhci_host *host)
{
	u16 caps_ptr = 0;
	u32 caps_gen = 0;
	u32 caps_phy = 0;
	u32 caps_tran[2] = {0, 0};
	struct mmc_host *mmc = host->mmc;

	/* TODO: may add corresponding members in sdhci_host to
	 * keep these caps.
	 */
	caps_ptr = sdhci_readw(host, SDHCI_UHS2_HOST_CAPS_PTR);

	if (caps_ptr < 0x100 || caps_ptr > 0x1FF) {
		pr_err("%s: SDHCI_UHS2_HOST_CAPS_PTR(%d) is wrong.\n",
			mmc_hostname(mmc), caps_ptr);
		return -ENODEV;
	}
	caps_gen = sdhci_readl(host,
		caps_ptr + SDHCI_UHS2_HOST_CAPS_GEN_OFFSET);
	caps_phy = sdhci_readl(host,
		caps_ptr + SDHCI_UHS2_HOST_CAPS_PHY_OFFSET);
	caps_tran[0] = sdhci_readl(host,
		caps_ptr + SDHCI_UHS2_HOST_CAPS_TRAN_OFFSET);
	caps_tran[1] = sdhci_readl(host,
		caps_ptr + SDHCI_UHS2_HOST_CAPS_TRAN_1_OFFSET);

	/* Geneneral Caps */
	mmc->uhs2_caps.dap = caps_gen & SDHCI_UHS2_HOST_CAPS_GEN_DAP_MASK;
	mmc->uhs2_caps.gap = (caps_gen & SDHCI_UHS2_HOST_CAPS_GEN_GAP_MASK)
				>> SDHCI_UHS2_HOST_CAPS_GEN_GAP_SHIFT;
#ifdef FD_ONLY
	mmc->uhs2_caps.n_lanes = 0;
#else
	mmc->uhs2_caps.n_lanes = (caps_gen & SDHCI_UHS2_HOST_CAPS_GEN_LANE_MASK)
			>> SDHCI_UHS2_HOST_CAPS_GEN_LANE_SHIFT;
#endif
	mmc->uhs2_caps.addr64 =
		(caps_gen & SDHCI_UHS2_HOST_CAPS_GEN_ADDR_64) ? 1 : 0;
	mmc->uhs2_caps.card_type =
		(caps_gen & SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_MASK) >>
		SDHCI_UHS2_HOST_CAPS_GEN_DEV_TYPE_SHIFT;

	/* PHY Caps */
	mmc->uhs2_caps.phy_rev = caps_phy & SDHCI_UHS2_HOST_CAPS_PHY_REV_MASK;

#ifndef SPEED_A_ONLY
	mmc->uhs2_caps.speed_range =
		(caps_phy & SDHCI_UHS2_HOST_CAPS_PHY_RANGE_MASK)
		>> SDHCI_UHS2_HOST_CAPS_PHY_RANGE_SHIFT;
#endif

#ifdef SET_DIR3_SYNC6
	mmc->uhs2_caps.n_lss_sync = 6;
	mmc->uhs2_caps.n_lss_dir = 3;
#else
	mmc->uhs2_caps.n_lss_sync =
		(caps_phy & SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_SYN_MASK)
		>> SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_SYN_SHIFT;
	mmc->uhs2_caps.n_lss_dir =
		(caps_phy & SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_DIR_MASK)
		>> SDHCI_UHS2_HOST_CAPS_PHY_N_LSS_DIR_SHIFT;
#endif

	/* LINK/TRAN Caps */
	mmc->uhs2_caps.link_rev =
		caps_tran[0] & SDHCI_UHS2_HOST_CAPS_TRAN_LINK_REV_MASK;
	mmc->uhs2_caps.n_fcu =
		(caps_tran[0] & SDHCI_UHS2_HOST_CAPS_TRAN_N_FCU_MASK)
		>> SDHCI_UHS2_HOST_CAPS_TRAN_N_FCU_SHIFT;
	mmc->uhs2_caps.host_type =
		(caps_tran[0] & SDHCI_UHS2_HOST_CAPS_TRAN_HOST_TYPE_MASK)
		>> SDHCI_UHS2_HOST_CAPS_TRAN_HOST_TYPE_SHIFT;
	mmc->uhs2_caps.maxblk_len =
		(caps_tran[0] & SDHCI_UHS2_HOST_CAPS_TRAN_BLK_LEN_MASK)
		>> SDHCI_UHS2_HOST_CAPS_TRAN_BLK_LEN_SHIFT;
	mmc->uhs2_caps.n_data_gap =
		caps_tran[1] & SDHCI_UHS2_HOST_CAPS_TRAN_1_N_DATA_GAP_MASK;

	return 0;
}

int sdhci_uhs2_do_detect_init(struct sdhci_host *host)
{
	unsigned long flags;
	int ret = -EIO;

	DBG("%s: begin UHS2 init.\n", __func__);
	spin_lock_irqsave(&host->lock, flags);

	if (sdhci_uhs2_interface_detect(host)) { // UHS2 Lane sync
		pr_debug("%s: cannot detect UHS2 interface.\n",
				mmc_hostname(host->mmc));
		goto out;
	}

	if (sdhci_uhs2_init(host)) {
		pr_warn("%s: UHS2 init fail.\n",
				mmc_hostname(host->mmc));
		goto out;
	}

	/* TODO: is this necessary? */
	/* Init complete, do soft reset and enable UHS2 error irqs. */
	sdhci_uhs2_clear_set_irqs(host, SDHCI_INT_ALL_MASK,
			SDHCI_UHS2_ERR_INT_STATUS_MASK); /* No reset No need */

	ret = 0;
out:
	/* TODO: do some clear work here? */
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_do_detect_init);

static void sdhci_uhs2_set_config(struct sdhci_host *host)
{
	u32 value;
	u32 ptr = sdhci_readw(host, SDHCI_UHS2_SET_PTR);

	/* Set Gen Settings */
	value = (host->mmc->uhs2_set_data.n_lanes_set <<
		SDHCI_UHS2_GEN_SET_N_LANES_POS) |
		host->mmc->uhs2_set_data.pwrctrl_mode_set;
	sdhci_writel(host, value, ptr + SDHCI_UHS2_GEN_SET);

	/* Set PHY Settings */
	value = (host->mmc->uhs2_set_data.n_lss_dir_set <<
			SDHCI_UHS2_PHY_SET_N_LSS_DIR_POS) |
		(host->mmc->uhs2_set_data.n_lss_sync_set <<
			SDHCI_UHS2_PHY_SET_N_LSS_SYN_POS) |
		(host->mmc->uhs2_set_data.hibernate_set <<
			SDHCI_UHS2_PHY_SET_HIBER_POS) |
		(host->mmc->uhs2_set_data.speed_range_set <<
			SDHCI_UHS2_PHY_SET_SPEED_POS);
	sdhci_writel(host, value, ptr + SDHCI_UHS2_PHY_SET);

	/* Set LINK-TRAN Settings */
	value = (host->mmc->uhs2_set_data.max_retry_set <<
			SDHCI_UHS2_TRAN_SET_RETRY_CNT_POS) |
		(host->mmc->uhs2_set_data.n_fcu_set <<
			SDHCI_UHS2_TRAN_SET_N_FCU_POS);
	sdhci_writel(host, value, ptr + SDHCI_UHS2_TRAN_SET);

	value = host->mmc->uhs2_set_data.n_data_gap_set;
	sdhci_writel(host, value, ptr + SDHCI_UHS2_TRAN_SET_1);
}

static int sdhci_uhs2_check_dormant(struct sdhci_host *host)
{
	int timeout = 100;

	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
		SDHCI_UHS2_IN_DORMANT_STATE)) {
		if (timeout == 0) {
			pr_warn("%s: UHS2 IN_DORMANT fail in 100ms.\n",
				mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return -EIO;
		}
		timeout--;
		mdelay(1);
	}
	return 0;
}

int sdhci_uhs2_do_set_reg(struct sdhci_host *host, enum uhs2_act act)
{
	unsigned long flags;
	int err = 0;

	DBG("Begin sdhci_uhs2_set_reg, act %d.\n", act);
	spin_lock_irqsave(&host->lock, flags);

	switch (act) {
	case SET_CONFIG:
		sdhci_uhs2_set_config(host);
		break;
	case ENABLE_INT:
		sdhci_clear_set_irqs(host, 0, SDHCI_INT_CARD_INT);
		break;
	case DISABLE_INT:
		sdhci_clear_set_irqs(host, SDHCI_INT_CARD_INT, 0);
		break;
	case CHECK_DORMANT:
		err = sdhci_uhs2_check_dormant(host);
		break;
	default:
		pr_err("%s: input action %d is wrong!\n",
			mmc_hostname(host->mmc), act);
		err = -EIO;
		break;
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	return err;
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_do_set_reg);

void sdhci_uhs2_irq(struct sdhci_host *host)
{
	u32 uhs2mask;

	uhs2mask = sdhci_readl(host, SDHCI_UHS2_ERR_INT_STATUS);
	DBG("*** %s got UHS2 interrupt: 0x%08x\n",
		mmc_hostname(host->mmc), uhs2mask);

	sdhci_writel(host, uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_MASK,
		SDHCI_UHS2_ERR_INT_STATUS);

	if (!(uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_MASK))
		return;

	if (uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_CMD_MASK) {
		host->cmd->error = -EILSEQ;
		if (uhs2mask &
		    SDHCI_UHS2_ERR_INT_STATUS_RES_TIMEOUT)
			host->cmd->error = -ETIMEDOUT;
	} else if (uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_DATA_MASK) {
		if (!host->data) {
			pr_err("%s: Got data interrupt 0x%08x even "
				"though no data operation was in progress.\n",
				mmc_hostname(host->mmc), (unsigned)uhs2mask);
			sdhci_dumpregs(host);
			return;
		}

		if (uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_DEADLOCK_TIMEOUT)
			host->data->error = -ETIMEDOUT;
		else if (uhs2mask & SDHCI_UHS2_ERR_INT_STATUS_ADMA) {
			pr_err("%s: ADMA error = 0x %x\n",
				mmc_hostname(host->mmc),
				sdhci_readb(host, SDHCI_ADMA_ERROR));
			host->data->error = -EIO;
		} else
			host->data->error = -EILSEQ;
	} else
		/* TODO: not sure if this is required. */
		host->cmd->error = -EILSEQ;

	/* TODO: per spec, below codes should be executed when data error
	 * found. But the codes will be executed in sdhci_tasklet_finish()
	 * finally. So remove them first.
	 */
	if (host->data != NULL)
		if (host->data->error)
			sdhci_finish_data(host);
		else
			tasklet_schedule(&host->finish_tasklet);
	else
		tasklet_schedule(&host->finish_tasklet);
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_irq);

int sdhci_uhs2_add_host(struct sdhci_host *host, u32 caps1)
{
	struct mmc_host *mmc;
	u32 max_current_caps2;

	if (host->version < SDHCI_SPEC_400)
		return 0;

	mmc = host->mmc;

	/* Support UHS2 */
	if (caps1 & SDHCI_SUPPORT_UHS2) {
		mmc->caps |= MMC_CAP_UHS2;
		mmc->flags |= MMC_UHS2_SUPPORT;
	}

	max_current_caps2 = sdhci_readl(host, SDHCI_MAX_CURRENT_1);
	if (!max_current_caps2 && !IS_ERR(mmc->supply.vmmc2)) {
		/* TODO: UHS2 - VDD2 */
		int curr = regulator_get_current_limit(mmc->supply.vmmc2);

		if (curr > 0) {
			/* convert to SDHCI_MAX_CURRENT format */
			curr = curr/1000;  /* convert to mA */
			curr = curr/SDHCI_MAX_CURRENT_MULTIPLIER;
			curr = min_t(u32, curr,
				SDHCI_MAX_CURRENT_LIMIT);
			max_current_caps2 = curr;
		}
	}

	if (caps1 & SDHCI_SUPPORT_VDD2_180) {
		mmc->ocr_avail_uhs2 |= MMC_VDD2_165_195;
		/* need vdd2 and timing for sdhci_runtime_resume_host() */
		mmc->ios.vdd2 = fls(mmc->ocr_avail_uhs2) - 1;
		mmc->ios.timing = MMC_TIMING_UHS2;
		/* UHS2 doesn't require this. Only UHS-I bus needs to set
		 * max current.
		 */
		mmc->max_current_180_vdd2 = (max_current_caps2 &
					SDHCI_MAX_CURRENT_VDD2_180_MASK) *
					SDHCI_MAX_CURRENT_MULTIPLIER;
	} else { /* TODO: shall I? */
		mmc->caps &= ~MMC_CAP_UHS2;
		mmc->flags &= ~MMC_UHS2_SUPPORT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_add_host);

void sdhci_uhs2_remove_host(struct sdhci_host *host, int dead)
{
	if (!(host->mmc) || !(host->mmc->flags & MMC_UHS2_SUPPORT))
		return;

	if (!dead) {
		sdhci_uhs2_reset(host, SDHCI_UHS2_SW_RESET_FULL);
	}

	sdhci_writel(host, 0, SDHCI_UHS2_ERR_INT_STATUS_EN);
	sdhci_writel(host, 0, SDHCI_UHS2_ERR_INT_SIG_EN);
	host->mmc->flags &= ~MMC_UHS2_SUPPORT;
	host->mmc->flags &= ~MMC_UHS2_INITIALIZED;
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_remove_host);

void sdhci_uhs2_trans_abort(struct sdhci_host *host)
{
	int reg_data;
	u8 did = host->mmc->uhs2_dev_prop.node_id;
	int to;

	reg_data = ((UHS2_DEV_CMD_TRANS_ABORT & 0xFF)                      << 24) |
		((UHS2_NATIVE_CMD_WRITE | (UHS2_DEV_CMD_TRANS_ABORT >> 8)) << 16) |
		(0                                                         <<  8) |
		(UHS2_NATIVE_PACKET | UHS2_PACKET_TYPE_CCMD | did);

	sdhci_writel(host, reg_data, SDHCI_UHS2_CMD_PACKET);

	sdhci_writew(host, 0, SDHCI_UHS2_TRANS_MODE);

	reg_data = (4 << SDHCI_UHS2_COMMAND_PACK_LEN_SHIFT)
			| SDHCI_UHS2_COMMAND_TRNS_ABORT;
	sdhci_writew(host, reg_data, SDHCI_UHS2_COMMAND);

	to = 100;
	while (--to && !(sdhci_readl(host, SDHCI_INT_STATUS) & 0x1)) /* 4 */
		udelay(10);

	sdhci_writel(host, sdhci_readl(host, SDHCI_INT_STATUS), SDHCI_INT_STATUS);

	if (!to) {
		pr_err("%s: UHS2 TRANS_ABORT COMMAND Timeout Error\n",
		       mmc_hostname(host->mmc));
	}
}
EXPORT_SYMBOL_GPL(sdhci_uhs2_trans_abort);

