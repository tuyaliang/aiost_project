/**
 * ogma_driver.c 
 *
 *  Copyright (c) 2015 SOCIONEXT INCORPORATED.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Main source file of OGMA Linux sample driver.
 *
 */

#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/random.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/io.h>

#include "ogma_driver_global.h"
#include "ogma_internal.h"

static unsigned short tx_desc_num = 128;
static unsigned short rx_desc_num = 128;
static int napi_weight = 64;
extern unsigned int flow_ctrl_start_threshold;
extern unsigned int flow_ctrl_stop_threshold;
extern unsigned short pause_time;

static irqreturn_t ogma_irq_handler(int irq, void *dev_id)
{
	ogma_uint32 status;
	struct net_device *netdev_p;
	ogma_netdev_t *ogma_netdev_p;

	netdev_p = (struct net_device *)dev_id;
	ogma_netdev_p = (ogma_netdev_t *) netdev_priv(netdev_p);

	dev_dbg(&netdev_p->dev, "ogma_irq_handler\n");

	status =
	    ogma_get_top_irq_status_non_clear(ogma_netdev_p->ogma_handle,
					      OGMA_TRUE);

	if (!status)
		return IRQ_NONE;

	if ((status & (OGMA_TOP_IRQ_REG_NRM_TX | OGMA_TOP_IRQ_REG_NRM_RX))) {
		ogma_disable_top_irq(ogma_netdev_p->ogma_handle,
				     OGMA_TOP_IRQ_REG_NRM_TX |
				     OGMA_TOP_IRQ_REG_NRM_RX);
		napi_schedule(&ogma_netdev_p->napi);
	}

	return IRQ_HANDLED;
}

int ogma_probe(struct device *dev_p, void *netsec_handle,
			   void __iomem * base, int irq, u8 * mac, int tx, int rx,
			   struct net_device **netdev_pp, struct ogma_priv **curpriv)
{
	ogma_param_t param;
	int err, i;
	ogma_err_t ogma_err;
	struct ogma_priv *priv;
	const u32 *p;
	const char *cp;
	ogma_ctrl_t *ctrl_p;

	if (*curpriv == NULL) {
		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;
		*curpriv = priv;
	} else {
		priv = *curpriv;
	}
	
#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "id", NULL);
	if (p)
		priv->id = be32_to_cpu(*p);
#else
	priv->id = 0;
#endif

	memset(&param, 0, sizeof(param));

	param.use_gmac_flag = OGMA_TRUE;
	for (i = 0; i < ETH_ALEN; i++)
		priv->mac[i] = mac[i];

	tx_desc_num = tx;
	rx_desc_num = rx;

	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].little_endian_flag =
	    OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].valid_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].entry_num = tx_desc_num;

	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].little_endian_flag =
	    OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].valid_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].entry_num = rx_desc_num;

	param.gmac_config.phy_interface = OGMA_PHY_INTERFACE_RGMII;

	priv->base = base;

	dev_p->coherent_dma_mask = DMA_BIT_MASK(32);
	dev_p->dma_mask = &dev_p->coherent_dma_mask;

	priv->clk[0] = of_clk_get(dev_p->of_node, 0);

	if (IS_ERR(priv->clk[0])) {
		dev_err(dev_p, "Failed to get clock\n");
		goto bail2;
	}
	clk_prepare_enable(priv->clk[0]);

	phy_dev_addr = 0;	// @ MEL-eva board

#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "phy-status-poll-interval-ms",
			    NULL);
	if (p)
		phy_status_poll_interval_ms = be32_to_cpu(*p);
#else
	phy_status_poll_interval_ms = 10000;
#endif

	param.use_jumbo_pkt_flag = 0;
#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "use-jumbo", NULL);
	if (p)
		param.use_jumbo_pkt_flag = ! !be32_to_cpu(*p);
#endif

	ogma_err =
	    ogma_init(priv->base, dev_p, &param, netsec_handle, &priv->handle);
	if (ogma_err != OGMA_ERR_OK) {
		dev_err(dev_p, "ogma_init() failed: %d\n", ogma_err);
		switch (ogma_err) {
		case OGMA_ERR_ALLOC:
			err = -ENOMEM;
			break;
		case OGMA_ERR_NOTAVAIL:
			err = -ENODEV;
			break;
		case OGMA_ERR_BUSY:
			err = -EBUSY;
			break;
		default:
			err = -EINVAL;
		}
		goto bail3a;
	}

	ctrl_p = (ogma_ctrl_t *) priv->handle;

	ctrl_p->gmac_mode.flow_ctrl_start_threshold = flow_ctrl_start_threshold;
	ctrl_p->gmac_mode.flow_ctrl_stop_threshold = flow_ctrl_stop_threshold;
	ctrl_p->gmac_mode.pause_time = pause_time;

	ctrl_p->gmac_mode.half_duplex_flag = 0;
#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "half-duplex", NULL);
	if (p)
		ctrl_p->gmac_mode.half_duplex_flag = ! !be32_to_cpu(*p);
#endif

	ctrl_p->gmac_mode.flow_ctrl_enable_flag = 0;
#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "flow-control", NULL);
	if (p)
		ctrl_p->gmac_mode.flow_ctrl_enable_flag = ! !be32_to_cpu(*p);
#endif

	param.use_jumbo_pkt_flag = 0;
#if	defined(CONFIG_USE_OF)
	p = of_get_property(dev_p->of_node, "use-jumbo", NULL);
	if (p)
		param.use_jumbo_pkt_flag = ! !be32_to_cpu(*p);
#endif

	ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_100M;
#if	defined(CONFIG_USE_OF)
	cp = of_get_property(dev_p->of_node, "link-speed", NULL);
	if (cp) {
		if (!strcmp(cp, "1G"))
			ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_1G;
		else if (!strcmp(cp, "100M"))
			ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_100M;
		else if (!strcmp(cp, "10M"))
			ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_10M;
		else {
			dev_err(dev_p,
				"link-speed should be 1G, 100M or 10M\n");
			goto bail1;
		}
	}
#endif

	err = ogma_netdev_init(priv->handle, dev_p,
			       (const ogma_uint8 *)priv->mac,
			       "eth%d", napi_weight,
			       tx_desc_num, rx_desc_num, &priv->netdev_p);
	if (err) {
		dev_err(dev_p, "ogma_netdev_init() failed\n");
		ogma_terminate(priv->handle);
		goto bail4;
	}
	*netdev_pp = priv->netdev_p;

	priv->irq = irq;
	priv->netdev_p->irq = priv->irq;
	err = request_irq(priv->irq, ogma_irq_handler, IRQF_SHARED,
			  priv->netdev_p->name, priv->netdev_p);
	if (err) {
		dev_err(dev_p, "request_irq() failed\n");
		goto bail5;
	}

	ogma_enable_top_irq(priv->handle, OGMA_TOP_IRQ_REG_NRM_TX |
			    OGMA_TOP_IRQ_REG_NRM_RX);

/* [YNK] add ether phy configuration for BCM54210*/
	{
		unsigned long phyid = 0;
		int misc_reg;
		ogma_netdev_t *ogma_netdev_p =
		    (ogma_netdev_t *) netdev_priv(priv->netdev_p);
		int (*mii_read) (struct net_device *, int, int) =
		    ogma_netdev_p->ogma_mii_if_info.mdio_read;
		void (*mii_write) (struct net_device *, int, int, int) =
		    ogma_netdev_p->ogma_mii_if_info.mdio_write;

		phyid = (mii_read(priv->netdev_p, phy_dev_addr, 2) << 16)
		    | mii_read(priv->netdev_p, phy_dev_addr, 3);
		if ((phyid & 0xFFFFFFF0) == 0x600d84a0) {
			dev_info(dev_p, "%s detected BCM54210E(phy ID=%lX)\n",
				 priv->netdev_p->name, phyid);
			/* 1000Base-T Control Register 09h:
			   Advertize no 1000Base-T */
			mii_write(priv->netdev_p, phy_dev_addr, 0x09, 0x0000);

			mii_write(priv->netdev_p, phy_dev_addr, 0x18, 0x7007);
			misc_reg = mii_read(priv->netdev_p, phy_dev_addr, 0x18);
			/* bit 5: "RGMII out-of-band status" should  clear */
			misc_reg = (misc_reg & 0xFFDF) | 0x8000;
			mii_write(priv->netdev_p, phy_dev_addr, 0x18, misc_reg);

			/* Errata work-around ---- */
			mii_write(priv->netdev_p, phy_dev_addr, 0x18, 0x0C00);
			/* Expansion Register Select */
			mii_write(priv->netdev_p, phy_dev_addr, 0x17, 0x0FF0);
			mii_write(priv->netdev_p, phy_dev_addr, 0x15, 0x2000);

			mii_write(priv->netdev_p, phy_dev_addr, 0x18, 0x0400);
			/* Errata work-around ---- */

			/* BMCR: Restart AN */
			mii_write(priv->netdev_p, phy_dev_addr, 0x00, 0x3300);

			/* Wait for AN Restart */
			while (1) {
				if ((mii_read
				     (priv->netdev_p, phy_dev_addr,
				      0x00) & 0x0200) == 0)
					break;
			}
			/* Read BMSR */
			mii_read(priv->netdev_p, phy_dev_addr, 0x01);
		}
	}
	dev_info(dev_p, "%s initialized\n", priv->netdev_p->name);

	return 0;

 bail5:
	ogma_netdev_uninit(priv->netdev_p);
 bail4:
	ogma_terminate(priv->handle);
 bail3a:
	if (priv->gpio_phy_enable) {
		err = gpio_direction_output(priv->gpio_phy_enable, 0);
		gpio_free(priv->gpio_phy_enable);
	}
	if (priv->gpio_phy_nrst) {
		err = gpio_direction_output(priv->gpio_phy_nrst, 0);
		gpio_free(priv->gpio_phy_nrst);
	}
 bail2:
	iounmap(priv->base);
 bail1:
	kfree(priv);

	return -EINVAL;
}

int ogma_remove(struct platform_device *pdev)
{
	struct ogma_priv *priv = platform_get_drvdata(pdev);
	int n;

	ogma_disable_top_irq(priv->handle, OGMA_TOP_IRQ_REG_NRM_TX |
			     OGMA_TOP_IRQ_REG_NRM_RX);
	free_irq(priv->irq, priv->netdev_p);
	ogma_netdev_uninit(priv->netdev_p);
	ogma_terminate(priv->handle);

	if (priv->gpio_phy_enable) {
		gpio_direction_output(priv->gpio_phy_enable, 0);
		gpio_free(priv->gpio_phy_enable);
	}
	if (priv->gpio_phy_nrst) {
		gpio_direction_output(priv->gpio_phy_nrst, 0);
		gpio_free(priv->gpio_phy_nrst);
	}
	for (n = 0; n < ARRAY_SIZE(priv->clk); n++) {
		clk_disable_unprepare(priv->clk[n]);
		clk_put(priv->clk[n]);
	}
	kfree(priv);

	return 0;
}
