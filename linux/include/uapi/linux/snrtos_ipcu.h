
#ifndef _UAPI_LINUX_SNRTOS_H
#define _UAPI_LINUX_SNRTOS_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define SNRTOS_CMD_LEN	9 /* max buffer words for each command */

#define SNRTOS_IPCU_READ	_IOR('S', 0, int)
#define SNRTOS_IPCU_WRITE	_IOW('S', 0, int)
#define SNRTOS_IPCU_WRITE_READ	_IOWR('S', 0, int)

#endif /* _UAPI_LINUX_SNRTOS_H */
