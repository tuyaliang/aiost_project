/**
 * Copyright (C) 2015 Socionext Semiconductor Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * @file   sni_ipcu_drv.c
 * @author
 * @date
 * @brief  SNI IPCU device driver
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/ipcu_userland.h>

#include "sni_ipcu_drv.h"
#include <uapi/linux/sni_ipcu_parts.h>
#include "sni_ipcu_comm.h"

#include <uapi/linux/shared_mem.h>

#define DEV_COUNT 1

#define MAGIC_CODE_OFFSET_UNIT0 0x00
#define MAGIC_CODE_OFFSET_UNIT1 0x20
static const u8 mc_offset[] = { MAGIC_CODE_OFFSET_UNIT0, MAGIC_CODE_OFFSET_UNIT1 };

struct ipcu_driver_info ipcu_drv_inf[IPCU_MAX_UNIT];


/* ioctl */
static long sni_ipcu_drv_ioctl(struct file *filp,
			       unsigned int cmd,
			       unsigned long arg)
{
	int err = 0;
	int rc = 0;
	struct ipcu_open_close_ch_argv ipcu_open_close_channel;
	struct ipcu_send_recv_msg_argv ipcu_send_recv_message;
	struct miscdevice *mdev = filp->private_data;
	struct sni_ipcu_device *sni_ipcu_dev =
			container_of(mdev, struct sni_ipcu_device, miscdev);


	if (_IOC_TYPE(cmd) != IPCU_IOCTL_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > IPCU_IOCTL_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err)
		return -EFAULT;

	switch (cmd) {
	case IPCU_IOCTL_OPENCH:
		if (copy_from_user(&ipcu_open_close_channel,
				   (void *)arg,
				   sizeof(ipcu_open_close_channel))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		rc = sni_ipcu_ch_init(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				ipcu_open_close_channel.direction,
				sni_ipcu_dev);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] cannot init ipcu channel.\n",
							__func__, __LINE__);
			return rc;
		}

		rc = sni_ipcu_opench(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				ipcu_open_close_channel.direction);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] sni_ipcu_opench(): %d\n",
						__func__, __LINE__, rc);
			return rc;
		}
		break;

	case IPCU_IOCTL_CLOSECH:
		if (copy_from_user(&ipcu_open_close_channel,
				   (void *)arg,
				   sizeof(ipcu_open_close_channel))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		sni_ipcu_ch_exit(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				sni_ipcu_dev);

		rc = sni_ipcu_closech(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				ipcu_open_close_channel.direction);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] sni_ipcu_closech(): %d\n",
						__func__, __LINE__, rc);
			return rc;
		}
		break;

	case IPCU_IOCTL_SENDMSG:
		if (copy_from_user(&ipcu_send_recv_message,
				   (void *)arg,
				   sizeof(ipcu_send_recv_message))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		rc = sni_ipcu_send_msg(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				ipcu_send_recv_message.buf,
				ipcu_send_recv_message.len,
				ipcu_send_recv_message.flags);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] sni_ipcu_send_msg(): %d\n",
						__func__, __LINE__, rc);
			return rc;
		}
		break;

	case IPCU_IOCTL_RECVMSG:
		if (copy_from_user(&ipcu_send_recv_message,
				   (void *)arg,
				   sizeof(ipcu_send_recv_message))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		rc = sni_ipcu_recv_msg(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel,
				ipcu_send_recv_message.buf,
				ipcu_send_recv_message.len,
				ipcu_send_recv_message.flags);
		if (rc < 0) {
			if (rc != -ERESTARTSYS) {
				pr_err("%s:%d [ERROR]sni_ipcu_recv_msg():%d\n",
					__func__, __LINE__, rc);
			}
			return rc;
		}

		if (copy_to_user((void *)arg,
				 &ipcu_send_recv_message,
				 sizeof(ipcu_send_recv_message))) {
			pr_err("%s:%d copy_to_user failed\n",
					__func__, __LINE__);
			return -EFAULT;
		}
		break;

	case IPCU_IOCTL_RECV_FLASH:
		if (copy_from_user(&ipcu_send_recv_message,
				   (void *)arg,
				   sizeof(ipcu_send_recv_message))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		rc = sni_ipcu_recv_flsh(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] sni_ipcu_send_msg(): %d\n",
						__func__, __LINE__, rc);
			return rc;
		}

		break;

	case IPCU_IOCTL_ACKSEND:
		if (copy_from_user(&ipcu_send_recv_message,
				   (void *)arg,
				   sizeof(ipcu_send_recv_message))) {
			pr_err("%s:%d copy_from_user failed\n",
						__func__, __LINE__);
			return -EFAULT;
		}

		rc = sni_ipcu_ack_send(
				sni_ipcu_dev->dest_unit,
				sni_ipcu_dev->dest_channel);
		if (rc < 0) {
			pr_err("%s:%d [ERROR] sni_ipcu_send_msg(): %d\n",
						__func__, __LINE__, rc);
			return rc;
		}

		break;

	default:
		pr_err("%s:%d [ERROR] unknown command: %d\n",
					__func__, __LINE__, cmd);
		return -EINVAL;
	}

	return rc;
}

/* .open */
static int sni_ipcu_drv_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct sni_ipcu_device *sni_ipcu_dev =
			container_of(mdev, struct sni_ipcu_device, miscdev);
	int ret;


	mutex_lock(&sni_ipcu_dev->mlock);
	ret = nonseekable_open(inode, file);
	mutex_unlock(&sni_ipcu_dev->mlock);
	return ret;
}

/* .release */
static int sni_ipcu_drv_release(struct inode *inode, struct file *file)
{
#if 0
	struct miscdevice *mdev = file->private_data;
	struct sni_ipcu_device *sni_ipcu_dev =
			container_of(mdev, struct sni_ipcu_device, miscdev);
#endif
	return 0;
}

/* The file operations for the rena_share_mem */
struct file_operations sni_ipcu_drv_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = sni_ipcu_drv_ioctl,
	.open		= sni_ipcu_drv_open,
	.release	= sni_ipcu_drv_release,
	.llseek		= no_llseek,
};

static int sni_ipcu_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sni_ipcu_device *sni_ipcu_dev;
	struct resource	*res_mem;
	struct resource	*res_irq;
	void __iomem *io_mem;
	void __iomem *io_get_mem;
	u32 dest_unit = 0;
	u32 dest_channel = 0;
	u32 src_int_ch = 0, dst_int_ch = 0;
	u32 unit_flg = 0;


	sni_ipcu_dev = devm_kzalloc(dev, sizeof(*sni_ipcu_dev), GFP_KERNEL);
	if (!sni_ipcu_dev)
		return -ENOMEM;

	if (!of_property_read_u32(dev->of_node, "dst-unit", &dest_unit)) {
		if (dest_unit >= 2) {
			dev_err(dev, "Invalid destination unit i/f %d\n",
								dest_unit);
			return -EINVAL;
		}
		unit_flg = 1;
	}
	sni_ipcu_dev->dest_unit = dest_unit;

	if (of_property_read_u32(dev->of_node, "dst-channel",
							&dest_channel)) {
		dev_err(dev, "%s:%d No RTOS channel i/f provided\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	if (dest_channel >= 16) {
		dev_err(dev, "Invalid destination channel i/f %d\n",
							dest_channel);
		return -EINVAL;
	}
	sni_ipcu_dev->dest_channel = dest_channel;

	if (of_property_read_u32(dev->of_node, "src-int-ch",
							&src_int_ch)) {
		if (dest_unit == 0)
			src_int_ch = dest_channel + IO_IPCU_INT_ACK_UNIT0;
		else
			src_int_ch = dest_channel + IO_IPCU_INT_ACK_UNIT1;
	}
	if (src_int_ch >= 16) {
		dev_err(dev, "Invalid destination channel i/f %d\n",
							src_int_ch);
		return -EINVAL;
	}
	if (of_property_read_u32(dev->of_node, "dst-int-ch",
							&dst_int_ch)) {
		dst_int_ch = dest_channel;
	}
	if (dst_int_ch >= 16) {
		dev_err(dev, "Invalid destination channel i/f %d\n",
							dst_int_ch);
		return -EINVAL;
	}

	if ((dest_channel == 0 && dest_unit == 0) ||
	    (dest_channel == 0 && dest_unit == 1)) {
		res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res_mem) {
			dev_err(dev, "No IOMEM found\n");
			return -ENXIO;
		}

		io_mem = devm_ioremap(dev,
				res_mem->start,
				resource_size(res_mem));
		if (!io_mem) {
			dev_err(dev, "ioremap failed.\n");
			return -ENXIO;
		}
		ipcu_drv_inf[dest_unit].ipcu_io_mem = io_mem;

		io_mem = shared_mem_get_mem(E_SHARED_MEM_SYNC);
		if (!io_mem) {
			dev_err(dev, "get shared_mem failed.\n");
			return -ENXIO;
		}

		io_get_mem = devm_ioremap(dev,
				ioread32(io_mem) + mc_offset[dest_unit],
				sizeof(io_mem));
		if (!io_get_mem) {
			dev_err(dev, "ioremap failed.\n");
			return -ENXIO;
		}
		ipcu_drv_inf[dest_unit].ipcu_magic_mem = io_get_mem;
	}

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq) {
		dev_err(dev, "No IRQ found\n");
		return -ENXIO;
	}

	if (of_property_match_string(dev->of_node,
					"direction",
					"recv") == 0) {
		ipcu_drv_inf[dest_unit].ipcu_rec_irq[dest_channel] =
							res_irq->start;
		ipcu_drv_inf[dest_unit].ipcu_ack_irq[dest_channel] = -1;
	} else if (of_property_match_string(dev->of_node,
					"direction",
					"send") == 0) {
		ipcu_drv_inf[dest_unit].ipcu_rec_irq[dest_channel] = -1;
		ipcu_drv_inf[dest_unit].ipcu_ack_irq[dest_channel] =
							res_irq->start;
	} else {
		dev_err(dev, "%s:%d Direction not found\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	ipcu_drv_inf[dest_unit].src_int_ch[dest_channel] = src_int_ch;
	ipcu_drv_inf[dest_unit].dst_int_ch[dest_channel] = dst_int_ch;

	mutex_init(&sni_ipcu_dev->mlock);

	if (unit_flg == 1) {
		snprintf(sni_ipcu_dev->devname, 24, "snrtos%d_%d",
					sni_ipcu_dev->dest_unit,
					sni_ipcu_dev->dest_channel);
	} else {
		snprintf(sni_ipcu_dev->devname, 24, "snrtos%d",
					sni_ipcu_dev->dest_channel);
	}
	sni_ipcu_dev->miscdev.name = sni_ipcu_dev->devname;
	sni_ipcu_dev->miscdev.minor = MISC_DYNAMIC_MINOR,
	sni_ipcu_dev->miscdev.fops = &sni_ipcu_drv_fops,

	platform_set_drvdata(pdev, sni_ipcu_dev);

	init_completion(&ipcu_drv_inf[dest_unit].ack_notify[dest_channel]);

	return misc_register(&sni_ipcu_dev->miscdev);
}

static int sni_ipcu_drv_remove(struct platform_device *pdev)
{
	struct sni_ipcu_device *sni_ipcu_dev = platform_get_drvdata(pdev);

	if ((sni_ipcu_dev->dest_channel == 0 &&
		sni_ipcu_dev->dest_unit == 0) ||
	    (sni_ipcu_dev->dest_channel == 0 &&
		sni_ipcu_dev->dest_unit == 1)) {
		if (ipcu_drv_inf[sni_ipcu_dev->dest_unit].ipcu_io_mem
								!= NULL) {
			devm_iounmap(
			&pdev->dev,
			ipcu_drv_inf[sni_ipcu_dev->dest_unit].ipcu_io_mem);
		}
		if (ipcu_drv_inf[sni_ipcu_dev->dest_unit].ipcu_magic_mem
								!= NULL) {
			devm_iounmap(
			&pdev->dev,
			ipcu_drv_inf[sni_ipcu_dev->dest_unit].ipcu_magic_mem);
		}
	}

	misc_deregister(&sni_ipcu_dev->miscdev);
	return 0;
}

static const struct of_device_id sni_ipcu_id[] = {
	{ .compatible = "socionext,ipcu-device" },
	{},
};
MODULE_DEVICE_TABLE(of, sni_ipcu_id);

static struct platform_driver sni_ipcu_driver = {
	.probe = sni_ipcu_drv_probe,
	.remove = sni_ipcu_drv_remove,
	.driver = {
		.name = "sni-ipcu-driver",
		.of_match_table = sni_ipcu_id,
	},
};
module_platform_driver(sni_ipcu_driver);

MODULE_LICENSE("GPL");
