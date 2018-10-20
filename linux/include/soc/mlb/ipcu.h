/*
 * include/soc/mb86s7x/ipcu.h
 *
 * Created by: Jassi Brar <jassisinghbrar@gmail.com>
 * Copyright:	(C) 2013-2015 Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MB86S7X_IPCU_H
#define __MB86S7X_IPCU_H

#define BUF_LEN		9

struct ipcu_mssg {
	u32 data[BUF_LEN];
	u16 mask;
};

#endif /* __MB86S7X_IPCU_H */
