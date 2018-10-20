/*
 * Copyright (C) 2015 Socionext Semiconductor Ltd.
 * Copyright (C) 2015 Linaro Ltd.
 * Author: Jassi Brar <jaswinder.singh@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>
#include <soc/mb86s7x/ipcu.h>

#define MBOX_START	0x100
#define MBOX_REGS	0x80
#define MAR(i)		(0x80 + (i) * 4)
#define MBOX_OFF(m)	(MBOX_START + MBOX_REGS * (m))
#define SRC_REG(m)	(MBOX_OFF(m) + 0x0)
#define MODE_REG(m)	(MBOX_OFF(m) + 0x4)
#define SEND_REG(m)	(MBOX_OFF(m) + 0x8)
	#define TRIGGER		0
#define DST_SET(m)	(MBOX_OFF(m) + 0x10)
#define DST_CLR(m)	(MBOX_OFF(m) + 0x14)
#define DST_STAT(m)	(MBOX_OFF(m) + 0x18)
#define ACK_SET(m)	(MBOX_OFF(m) + 0x30)
#define ACK_CLR(m)	(MBOX_OFF(m) + 0x34)
#define ACK_STAT(m)	(MBOX_OFF(m) + 0x38)
	#define REQ	12
	#define ACK	13
#define DAT_REG(m)	(MBOX_OFF(m) + 0x40)

#define MAX_CPUIF	16
#define MAX_MBOX	8

struct ipcu_link {
	unsigned cpuif; /* IPCUINT# */
	unsigned mbox; /* physical mbox */
	int irq; /* read from client's dt node */
	struct ipcu *ipcu; /* parent controller */
};

struct ipcu {
	struct mbox_controller mbox; /* keep at top */
	struct mbox_chan chan[MAX_CPUIF];
	struct ipcu_link link[MAX_CPUIF];
	void __iomem *base;
	struct clk *clk;
};

static irqreturn_t ipcu_interrupt(int irq, void *data)
{
	struct mbox_chan *chan = data;
	struct ipcu_link *link = chan->con_priv;
	struct ipcu *ipcu = link->ipcu;
	irqreturn_t ret = IRQ_NONE;
	u32 mar, ack = 0;
	int i, src;

	do {
		mar = readl(ipcu->base + MAR(link->cpuif));
		src = ((mar & 0xfff) - MBOX_START) / MBOX_REGS;

		if (mar & BIT(ACK)) {
			ack = readl(ipcu->base + ACK_STAT(src));
			writel(ack, ipcu->base + ACK_CLR(src));
			writel(0, ipcu->base + SEND_REG(src));
			mbox_chan_txdone(chan, 0);
			ret = IRQ_HANDLED;
		}

		if (mar & BIT(REQ)) {
			struct ipcu_mssg mssg;

			/* Read the data */
			for (i = 0; i < BUF_LEN; i++)
				mssg.data[i] = readl(ipcu->base +
						DAT_REG(src) + i * 4);
			mssg.mask = readl(ipcu->base + SRC_REG(src));
			mb();

			/* Handover the message to client */
			mbox_chan_received_data(chan, (void *)&mssg);

			writel(1 << link->cpuif, ipcu->base + DST_CLR(src));
			writel(1 << link->cpuif, ipcu->base + ACK_SET(src));
			ret = IRQ_HANDLED;
		}
	} while (mar & BIT(ACK) || mar & BIT(REQ));

	return ret;
}

static int ipcu_send_data(struct mbox_chan *chan, void *data)
{
	struct ipcu_link *link = chan->con_priv;
	struct ipcu *ipcu = link->ipcu;
	struct ipcu_mssg *mssg = data;
	int i;

	/* clear our own destination */
	mssg->mask &= ~(1 << link->cpuif);

	i = 0;
	do {
		if (mssg->mask & (1 << i))
			writel(1 << i, ipcu->base + DST_SET(link->mbox));
	} while (++i < MAX_CPUIF);

	/* Fill the data */
	for (i = 0; i < BUF_LEN; i++)
		writel(mssg->data[i],
			ipcu->base + DAT_REG(link->mbox) + i * 4);
	/* Trigger */
	writel(BIT(TRIGGER), ipcu->base + SEND_REG(link->mbox));

	return 0;
}

static bool ipcu_last_tx_done(struct mbox_chan *chan)
{
	struct ipcu_link *link = chan->con_priv;
	struct ipcu *ipcu = link->ipcu;

	return !readl(ipcu->base + SEND_REG(link->mbox));
}

static int ipcu_startup(struct mbox_chan *chan)
{
	struct ipcu_link *link = chan->con_priv;
	struct ipcu *ipcu = link->ipcu;
	int i, ret = 0;
	u32 val;

	/* Try to own a physical mailbox */
	for (i = 0; i < MAX_MBOX; i++) {
		val = readl(ipcu->base + SRC_REG(i));
		if (val) /* skip already taken */
			continue;

		writel(1 << link->cpuif, ipcu->base + SRC_REG(i));
		mb();
		val = readl(ipcu->base + SRC_REG(i));
		if (val == (1 << link->cpuif)) {
			/* Set Auto-Ack-Mode1 */
			writel(2, ipcu->base + MODE_REG(i));
			break;
		}
	}
	if (i == MAX_MBOX)
		return -EAGAIN;
	link->mbox = i;

	if (link->irq >= 0) {
		ret = request_irq(link->irq, ipcu_interrupt,
				IRQF_SHARED, "ipcu", chan);
		if (ret)
			pr_err("Unable to aquire IRQ\n");
	}

	return ret;
}

static void ipcu_shutdown(struct mbox_chan *chan)
{
	struct ipcu_link *link = chan->con_priv;
	struct ipcu *ipcu = link->ipcu;

	if (link->irq >= 0)
		free_irq(link->irq, chan);

	/* release the mailbox */
	writel(0, ipcu->base + SRC_REG(link->mbox));
}

static struct mbox_chan_ops ipcu_ops = {
	.send_data = ipcu_send_data,
	.startup = ipcu_startup,
	.shutdown = ipcu_shutdown,
	.last_tx_done = ipcu_last_tx_done,
};

static struct mbox_chan *
ipcu_of_xlate(struct mbox_controller *mbox, const struct of_phandle_args *sp)
{
	struct platform_device *pdev = to_platform_device(mbox->dev);
	struct ipcu *ipcu = (struct ipcu *)mbox;
	unsigned cpuif = sp->args[0];
	char irqname[10];

	if (cpuif >= MAX_CPUIF)
		return NULL;

	snprintf(irqname, 10, "cpuif-%d", cpuif);
	ipcu->link[cpuif].irq = platform_get_irq_byname(pdev, irqname);

	return &ipcu->chan[cpuif];
}

static int f_ipcu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipcu_link *link;
	struct mbox_chan *chan;
	struct resource	*res;
	struct ipcu *ipcu;
	int i, err;

	ipcu = devm_kzalloc(dev, sizeof(*ipcu), GFP_KERNEL);
	if (!ipcu)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No IOMEM found\n");
		return -ENXIO;
	}
	ipcu->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!ipcu->base) {
		dev_err(dev, "ioremap failed.\n");
		return -ENXIO;
	}

	for (i = 0; i < MAX_CPUIF; i++) {
		link = &ipcu->link[i];
		chan = &ipcu->chan[i];
		chan->con_priv = link;
		link->ipcu = ipcu;
		link->cpuif = i;
	}
	ipcu->mbox.dev = dev;
	ipcu->mbox.chans = &ipcu->chan[0];
	ipcu->mbox.num_chans = MAX_CPUIF;
	ipcu->mbox.ops = &ipcu_ops;
	ipcu->mbox.txdone_irq = true;
	ipcu->mbox.of_xlate = ipcu_of_xlate;

	platform_set_drvdata(pdev, ipcu);

	err = mbox_controller_register(&ipcu->mbox);
	if (err) {
		dev_err(dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

	ipcu->clk = devm_clk_get(dev, "ipcu_clk");
	if (IS_ERR(ipcu->clk))
		dev_info(dev, "unable to init clock\n");
	else
		clk_prepare_enable(ipcu->clk);

	dev_info(dev, "IPCU Mailbox registered\n");
	return 0;
}

static int f_ipcu_remove(struct platform_device *pdev)
{
	struct ipcu *ipcu = platform_get_drvdata(pdev);

	clk_disable_unprepare(ipcu->clk);
	mbox_controller_unregister(&ipcu->mbox);

	return 0;
}

static const struct of_device_id f_ipcu_dt_ids[] = {
	{ .compatible = "socionext,ipcu" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_ipcu_dt_ids);

static struct platform_driver f_ipcu_driver = {
	.driver		= {
		.name	= "f_ipcu",
		.owner = THIS_MODULE,
		.of_match_table = f_ipcu_dt_ids,
	},
	.probe		= f_ipcu_probe,
	.remove		= f_ipcu_remove,
};
module_platform_driver(f_ipcu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Socionext IPCU Driver");
MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
