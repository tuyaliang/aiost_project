/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/snrtos_ipcu.h>
#include <soc/mb86s7x/ipcu.h>

struct sn_rtos_client {
	struct mbox_client cl;
	struct miscdevice miscdev;
	char devname[10];
	struct mbox_chan *chan;
	struct mutex rlock;
	struct mutex wlock;
	spinlock_t rx_lock;
	struct completion rx_done;
	unsigned dest_cpu; /* destination cpu i/f */
	struct ipcu_mssg tx_mssg;
	struct ipcu_mssg rx_mssg;
};

static void snrtos_rx_callback(struct mbox_client *c, void *mssg)
{
	struct sn_rtos_client *sncl = container_of(c,
						struct sn_rtos_client, cl);
	struct ipcu_mssg *r = mssg;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&sncl->rx_lock, flags);

	sncl->rx_mssg.mask = r->mask;
	for (i = 0; i < SNRTOS_CMD_LEN; i++)
		sncl->rx_mssg.data[i] = r->data[i];

	if (!completion_done(&sncl->rx_done))
		complete(&sncl->rx_done);

	spin_unlock_irqrestore(&sncl->rx_lock, flags);
}

static long sn_rtos_ioctl(struct file *file,
			  unsigned int cmd, unsigned long ptr)
{
	struct miscdevice *mdev = file->private_data;
	struct sn_rtos_client *sncl = container_of(mdev,
					struct sn_rtos_client, miscdev);
	int i, ret = -EINVAL;
	unsigned long flags;

	switch (cmd) {
	/* Only these 3 commands are supported */
	case SNRTOS_IPCU_READ:
	case SNRTOS_IPCU_WRITE:
	case SNRTOS_IPCU_WRITE_READ:
		break;
	default:
		return ret;
	}

	cmd = _IOC_DIR(cmd);

	if (cmd & _IOC_WRITE) {
		mutex_lock(&sncl->wlock);
		sncl->tx_mssg.mask = BIT(sncl->dest_cpu);
		ret = copy_from_user(sncl->tx_mssg.data, (__u8 __user *)ptr,
					SNRTOS_CMD_LEN * 4) ? -EFAULT : 0;
		if (ret) {
			pr_err("%s:%d copy_from_user failed\n",
				__func__, __LINE__);
			goto wrexit;
		}

		ret = mbox_send_message(sncl->chan, &sncl->tx_mssg);
		if (ret < 0) {
			pr_err("%s:%d send failed\n", __func__, __LINE__);
			goto wrexit;
		}
wrexit:
		mutex_unlock(&sncl->wlock);
	}

	if (cmd & _IOC_READ) {
		struct ipcu_mssg r;

		mutex_lock(&sncl->rlock);

		ret = wait_for_completion_interruptible(&sncl->rx_done);
		if (ret) {
			pr_err("%s:%d got no data\n", __func__, __LINE__);
			goto rdexit;
		}

		spin_lock_irqsave(&sncl->rx_lock, flags);

		for (i = 0; i < SNRTOS_CMD_LEN; i++)
			r.data[i] = sncl->rx_mssg.data[i];
		reinit_completion(&sncl->rx_done);

		spin_unlock_irqrestore(&sncl->rx_lock, flags);

		ret = copy_to_user((__u8 __user *)ptr, r.data,
				SNRTOS_CMD_LEN * 4) ? -EFAULT : 0;
		if (ret)
			pr_err("%s:%d copy_to_user failed\n",
				__func__, __LINE__);
rdexit:
		mutex_unlock(&sncl->rlock);
	}

	return ret;
}

static int sn_rtos_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct sn_rtos_client *sncl = container_of(mdev,
					struct sn_rtos_client, miscdev);
	int ret;

	mutex_lock(&sncl->rlock);
	mutex_lock(&sncl->wlock);

	if (!IS_ERR_OR_NULL(sncl->chan)) {
		ret = -EBUSY;
		goto exit;
	}

	sncl->chan = mbox_request_channel(&sncl->cl, 0);
	if (IS_ERR_OR_NULL(sncl->chan)) {
		ret = -EAGAIN;
		goto exit;
	}

	ret = nonseekable_open(inode, file);
exit:
	mutex_unlock(&sncl->wlock);
	mutex_unlock(&sncl->rlock);
	return ret;
}

static int sn_rtos_release(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct sn_rtos_client *sncl = container_of(mdev,
					struct sn_rtos_client, miscdev);
	mutex_lock(&sncl->rlock);
	mutex_lock(&sncl->wlock);
	mbox_free_channel(sncl->chan);
	sncl->chan = NULL;
	mutex_unlock(&sncl->wlock);
	mutex_unlock(&sncl->rlock);

	return 0;
}

static const struct file_operations sn_rtos_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= sn_rtos_ioctl,
	.open		= sn_rtos_open,
	.release	= sn_rtos_release,
	.llseek		= no_llseek,
};

static int sn_rtos_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sn_rtos_client *sncl;
	u32 dest_cpu;

	sncl = devm_kzalloc(dev, sizeof(*sncl), GFP_KERNEL);
	if (!sncl)
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "dst-intf", &dest_cpu)) {
		dev_err(dev, "%s:%d No RTOS cpu i/f provided\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (dest_cpu >= 16) {
		dev_err(dev, "Invalid destination cpu i/f %d\n", dest_cpu);
		return -EINVAL;
	}

	sncl->dest_cpu = dest_cpu;
	mutex_init(&sncl->rlock);
	mutex_init(&sncl->wlock);
	spin_lock_init(&sncl->rx_lock);
	init_completion(&sncl->rx_done);

	sncl->cl.dev = dev;
	sncl->cl.rx_callback = snrtos_rx_callback;
	sncl->cl.tx_block = true;
	sncl->cl.tx_tout = 500;
	sncl->cl.knows_txdone = false;
	snprintf(sncl->devname, 10, "snrtos%d", sncl->dest_cpu);
	sncl->miscdev.name = sncl->devname;
	sncl->miscdev.minor = MISC_DYNAMIC_MINOR,
	sncl->miscdev.fops = &sn_rtos_fops,

	platform_set_drvdata(pdev, sncl);

	return misc_register(&sncl->miscdev);
}

static int sn_rtos_remove(struct platform_device *pdev)
{
	struct sn_rtos_client *sncl = platform_get_drvdata(pdev);

	misc_deregister(&sncl->miscdev);
	return 0;
}

static const struct of_device_id sn_rtos_id[] = {
	{ .compatible = "socionext,rtos-client" },
	{},
};
MODULE_DEVICE_TABLE(of, sn_rtos_id);

static struct platform_driver sn_rtos_driver = {
	.probe = sn_rtos_probe,
	.remove = sn_rtos_remove,
	.driver = {
		.name = "rtos-client",
		.of_match_table = sn_rtos_id,
	},
};
module_platform_driver(sn_rtos_driver);

MODULE_LICENSE("GPL");
