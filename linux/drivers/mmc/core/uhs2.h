/*
 *  driver/mmc/core/uhs2.h - UHS-II driver
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
#ifndef MMC_UHS2_H
#define MMC_UHS2_H

#include <linux/mmc/core.h>
#include <linux/mmc/host.h>

#define UHS2_PHY_INIT_ERR	1

extern int uhs2_prepare_sd_cmd(struct mmc_host *host, struct mmc_request *mrq);
extern void uhs2_power_up(struct mmc_host *host);
extern int mmc_uhs2_try_frequency(struct mmc_host *host, unsigned freq);
extern int  mmc_uhs2_power_limit(struct mmc_card *card);

#endif /* MMC_UHS2_H */
