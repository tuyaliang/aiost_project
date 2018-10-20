/*
 *  linux/drivers/mmc/host/sdhci-uhs2.h - Secure Digital Host Controller
 *  Interface driver
 *
 * Header file for Host Controller UHS2 related registers and I/O accessors.
 *
 *  Copyright (C) 2014 Intel Corp, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef __SDHCI_UHS2_H
#define __SDHCI_UHS2_H

#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include "sdhci.h"
#include <linux/mmc/uhs2.h>

extern void sdhci_uhs2_reset(struct sdhci_host *host, u16 mask);
extern void sdhci_uhs2_set_transfer_mode(struct sdhci_host *host,
					struct mmc_command *cmd);
extern void sdhci_uhs2_send_command(struct sdhci_host *host,
					struct mmc_command *cmd);
extern void sdhci_uhs2_finish_command(struct sdhci_host *host);
extern void sdhci_uhs2_do_set_ios(struct sdhci_host *host,
					struct mmc_ios *ios);
extern int sdhci_uhs2_do_detect_init(struct sdhci_host *host);
extern int sdhci_uhs2_do_set_reg(struct sdhci_host *host, enum uhs2_act act);
extern void sdhci_uhs2_irq(struct sdhci_host *host);
extern int sdhci_uhs2_add_host(struct sdhci_host *host, u32 caps1);
extern void sdhci_uhs2_remove_host(struct sdhci_host *host, int dead);
extern void sdhci_uhs2_trans_abort(struct sdhci_host *host);
extern int sdhci_uhs2_to_uhs1(struct mmc_host *host);
extern int sdhci_uhs1_to_uhs2(struct mmc_host *host);

#endif /* __SDHCI_UHS2_H */
