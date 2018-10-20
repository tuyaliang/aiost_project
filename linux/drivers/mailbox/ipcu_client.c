/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <soc/mb86s7x/ipcu.h>

struct ipcu_client {
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct list_head node;
};

static LIST_HEAD(ipcu_clients);

static void ipc_recv(struct mbox_client *cl, void *data)
{
	struct ipcu_client *ipc = (struct ipcu_client *)cl;
	struct ipcu_mssg *mssg = data;

	pr_err("%p: From %lu  {%x %x %x %x %x %x %x %x %x}\n", ipc->chan, __ffs(mssg->mask),
		mssg->data[0], mssg->data[1], mssg->data[2], mssg->data[3],
		mssg->data[4], mssg->data[5], mssg->data[6], mssg->data[7],
		mssg->data[8]);
}

void ipc_send(int sender)
{
	struct ipcu_client *ipc;
	struct ipcu_mssg mssg;
	int i, ret;

	i = 0;
	list_for_each_entry(ipc, &ipcu_clients, node)
		if (i++ == sender)
			break;

	mssg.mask = 0xff;
	for (i = 0; i < 9; i++)
		mssg.data[i] = 0xabcdef01 + i;

	ret = mbox_send_message(ipc->chan, &mssg);
	if (ret < 0)
		pr_err("Chan-%p send failed\n", ipc->chan);
	else
		pr_err("Chan-%p sent OK\n", ipc->chan);
}

static int f_ipcu_probe(struct platform_device *pdev)
{
	struct ipcu_client *ipc;
	struct mbox_client *cl;

	ipc = kzalloc(sizeof(*ipc), GFP_KERNEL);
	cl = &ipc->cl;
	cl->tx_block = true;
	cl->tx_tout = 2000;
	cl->knows_txdone = false;
	cl->rx_callback = ipc_recv;
	cl->dev = &pdev->dev;
	list_add_tail(&ipc->node, &ipcu_clients);
	platform_set_drvdata(pdev, ipc);
	ipc->chan = mbox_request_channel(cl, 0);

	return 0;
}

static int f_ipcu_remove(struct platform_device *pdev)
{
	struct ipcu_client *ipc = platform_get_drvdata(pdev);

	mbox_free_channel(ipc->chan);
	return 0;
}

static const struct of_device_id ipcu_dt_ids[] = {
	{ .compatible = "socionext,ipcu_client" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ipcu_dt_ids);

static struct platform_driver f_ipcu_client = {
	.driver		= {
		.name	= "ipcu_client",
		.of_match_table = ipcu_dt_ids,
	},
	.probe = f_ipcu_probe,
	.remove	= f_ipcu_remove,
};
module_platform_driver(f_ipcu_client);
