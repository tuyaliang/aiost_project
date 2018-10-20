/*
 * uac.h  --  USB Audio Class Gadget driver
 *
 * Copyright (C) 2017 Infinitegra, Inc.
 *
 * Based on uvc.h
 * Copyright (C) 2009-2010 Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _UAC_GADGET_H_
#define _UAC_GADGET_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>

#define UAC_EVENT_CONNECT		0
#define UAC_EVENT_DISCONNECT		1
#define UAC_EVENT_STREAMON		2
#define UAC_EVENT_STREAMOFF		3
#define UAC_EVENT_SRATE			4

struct uac_event {
	__u32		type;
	union {
		int	srate;
		__u8	data[64];
	} u;
	__u32		pending;
	__u32		sequence;
	struct timespec	timestamp;
};

#define UACIOC_DQEVENT			_IOR('A', 0, int)
#define UACIOC_SRATE			_IOR('A', 1, int)

#endif /* _UAC_GADGET_H_ */

