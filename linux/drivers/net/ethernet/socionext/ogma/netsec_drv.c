/**
 * @file netsec.c
 * @author K.MATSUI
 * @date 2014/Jun/xx-2014/Jun/xx
 * @brief NETSEC device driver
 */
/* Copyright (C) 2015 SOCIONEXT INCORPORATED All Rights Reserved. */

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
#include <linux/cdev.h>
#include <linux/pm_runtime.h>

#include <linux/poll.h>

#include <asm/io.h>
#include <asm/pgalloc.h>
#include <asm/uaccess.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <asm/pgtable.h>

#include <linux/mm.h>
#include <linux/pagemap.h>

#include "ogma_reg_f_taiki.h"
#include "ogma_reg_f_gmac_4mt.h"
#include "ogma_reg.h"
#include "ogma_api.h"

#include "netsec_api.h"
#include "netsec_dring.h"
#include "netsec_drv.h"

#define	FORCE_10MHALF_TO_10MFULL

#define	NETSEC_COM_INIT_TIMEOUT		(10000)
#define	NETSEC_CODE_LOAD_END_TIMEOUT	(10000)
#define	NETSEC_MAC_DESC_INIT_TIMEOUT	(10000)
#define	NETSEC_DMA_SWRST_TIMEOUT	(10000)
#define	NETSEC_GMAC_SWRST_TIMEOUT	(10000)

#define DEV_COUNT 1

static struct class *netsec_class;

static unsigned long netsec_strm_status_le = 0;

int ogma_probe(struct device *dev, void *netsec_handle, void __iomem * base,
	       int irq, u8 * mac, int tx, int rx,
			   struct net_device **netdev_pp, struct ogma_priv **curpriv);
int ogma_remove(struct platform_device *pdev);
void ogma_netdev_set_rx_mode(struct net_device *netdev_p);

u32 netsec_reg_read(NETSEC_DRIVER_DATA * drv, u32 reg_addr)
{
	return *(volatile u32 *)((u32) drv->base + (reg_addr << 2));
}

void netsec_reg_write(NETSEC_DRIVER_DATA * drv, u32 reg_addr, u32 val)
{
	*(volatile u32 *)((u32) drv->base + (reg_addr << 2)) = val;
}

#define	MAC_CMD_ST			(0x80000000)
#define	MAC_CMD_READ		(0)
#define	MAC_CMD_WRITE		(0x10000000)
#define	MAC_CMD_REG_MASK	(0x0000ffff)

u32 netsec_mac_reg_read(NETSEC_DRIVER_DATA * drv, u32 reg_addr)
{
	u32 dat;

	dat = reg_addr | MAC_CMD_READ;

	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_CMD, dat);

	for (;;) {
		dat = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_CMD);
		if ((dat & MAC_CMD_ST) == 0) {
			break;
		}
	}

	dat = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_DATA);

	return dat;
}

void netsec_mac_reg_write(NETSEC_DRIVER_DATA * drv, u32 reg_addr, u32 val)
{
	u32 dat;

	dat = reg_addr | MAC_CMD_WRITE;

	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_DATA, val);
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_CMD, dat);

	for (;;) {
		dat = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_CMD);
		if ((dat & MAC_CMD_ST) == 0) {
			break;
		}
	}

	return;
}

#define	PKT_INTEN_TX_DONE_TS			(0x10000000)
#define	PKT_INTEN_TX_DONE_META			(0x08000000)
#define	PKT_INTEN_TX_DONE_VSUB			(0x04000000)
#define	PKT_INTEN_TX_DONE_J2			(0x02000000)
#define	PKT_INTEN_TX_DONE_J1			(0x01000000)
#define	PKT_INTEN_TX_DONE_A				(0x00800000)
#define	PKT_INTEN_TX_DONE_5				(0x00200000)
#define	PKT_INTEN_TX_DONE_4				(0x00100000)
#define	PKT_INTEN_TX_DONE_3				(0x00080000)
#define	PKT_INTEN_TX_DONE_2				(0x00040000)
#define	PKT_INTEN_TX_DONE_1				(0x00020000)
#define	PKT_INTEN_TX_DONE_0				(0x00010000)
#define	PKT_INTEN_TLS_DEC_FAIL			(0x00000400)
#define	PKT_INTEN_TLS_AUTH_FAIL			(0x00000200)
#define	PKT_INTEN_TLS_BAD_ALIGN			(0x00000100)
#define	PKT_INTEN_MAC_ER				(0x00000020)
#define	PKT_INTEN_JUMBO_ER				(0x00000010)
#define	PKT_INTEN_CHKSUM_ER				(0x00000008)
#define	PKT_INTEN_HD_INCOMPLETE			(0x00000004)
#define	PKT_INTEN_HD_ER					(0x00000002)

#define	STRM_INTEN_LE					(0x00000001)

#define	MAC_INTEN_INT_PMT				(0x80000000)
#define	MAC_INTEN_INT_SBD				(0x40000000)
#define	MAC_INTEN_INT_LPI				(0x20000000)
#define	MAC_INTEN_MAC_TX_RX_INFO_INT	(0x08000000)

#define	PKT_TX_INTEN_TX_NOT_OWNER		(0x00020000)
#define	PKT_TX_INTEN_TR_ERR				(0x00010000)
#define	PKT_TX_INTEN_TX_DONE			(0x00008000)
#define	PKT_TX_INTEN_TMR_EXP			(0x00004000)

#define	PKT_RX_INTEN_TR_ERR				(0x00010000)
#define	PKT_RX_INTEN_PKT_CNT			(0x00008000)
#define	PKT_RX_INTEN_TMR_EXP			(0x00004000)

#define	TX_CHAN_INTEN_TX_NOT_OWNER		(0x00020000)
#define	TX_CHAN_INTEN_TR_ERR			(0x00010000)
#define	TX_CHAN_INTEN_TX_DONE			(0x00008000)

#define	RX_CHAN_INTEN_TR_ERR			(0x00010000)
#define	RX_CHAN_INTEN_PKT_CNT			(0x00008000)

#define	TOP_INTEN_INT_SUB_ST			(0x00200000)
#define	TOP_INTEN_INT_ME_START			(0x00100000)
#define	TOP_INTEN_INT_MAC_ST			(0x00080000)
#define	TOP_INTEN_INT_PKT_ST			(0x00040000)
#define	TOP_INTEN_INT_STRM_ST			(0x00020000)
#define	TOP_INTEN_INT_REDC_ST			(0x00010000)
#define	TOP_INTEN_INT_TS_TX_ST			(0x00004000)
#define	TOP_INTEN_INT_MDT_TX_ST			(0x00002000)
#define	TOP_INTEN_INT_VSUB_TX_ST		(0x00001000)
#define	TOP_INTEN_INT_JPG2_TX_ST		(0x00000800)
#define	TOP_INTEN_INT_JPG1_TX_ST		(0x00000400)
#define	TOP_INTEN_INT_CRY_RX_ST			(0x00000200)
#define	TOP_INTEN_INT_CRY_TX_ST			(0x00000100)
#define	TOP_INTEN_INT_CDB_TX_ST			(0x00000040)
#define	TOP_INTEN_INT_RTP_LPB_RX_ST		(0x00000020)
#define	TOP_INTEN_INT_ADO_TX_ST			(0x00000010)
#define	TOP_INTEN_INT_ETH_LPB_RX_ST		(0x00000008)
#define	TOP_INTEN_INT_VDO_TX_ST			(0x00000004)
#define	TOP_INTEN_INT_PKT_RX_ST			(0x00000002)
#define	TOP_INTEN_INT_PKT_TX_ST			(0x00000001)

static int netsec_enable_irq(NETSEC_DRIVER_DATA * drv)
{
	u32 val;
	u32 tx, rx;

	val = MAC_INTEN_INT_SBD;
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTEN, val);

	/* Disable MAC LPI Interrupt */
	val = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_IMR);
	val |= 0x00000400;
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_IMR, val);

	/* Packet Interrupt Enable */
	val = PKT_INTEN_TLS_DEC_FAIL |
	    PKT_INTEN_TLS_AUTH_FAIL |
	    PKT_INTEN_TLS_BAD_ALIGN |
	    PKT_INTEN_MAC_ER |
	    PKT_INTEN_JUMBO_ER |
	    PKT_INTEN_CHKSUM_ER | PKT_INTEN_HD_INCOMPLETE | PKT_INTEN_HD_ER;

	/* MAC Status */
	val = MAC_INTEN_INT_SBD;
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTEN, val);

	/* PKT TX Channel Interrupt Enable */
	val = PKT_TX_INTEN_TX_NOT_OWNER |
	    PKT_TX_INTEN_TR_ERR | PKT_TX_INTEN_TX_DONE | PKT_TX_INTEN_TMR_EXP;
	netsec_reg_write(drv, OGMA_REG_ADDR_PKT_TX_INTEN_SET, val);

	/* PKT RX Channel Interrupt Enable */
	val = PKT_RX_INTEN_TR_ERR | PKT_RX_INTEN_PKT_CNT | PKT_RX_INTEN_TMR_EXP;
	netsec_reg_write(drv, OGMA_REG_ADDR_PKT_RX_INTEN_SET, val);

	/* Video TX Channel Interrupt Enable */
	tx = TX_CHAN_INTEN_TX_NOT_OWNER |
	    TX_CHAN_INTEN_TR_ERR | TX_CHAN_INTEN_TX_DONE;
	netsec_reg_write(drv, OGMA_REG_ADDR_VDO_TX_INTEN_SET, tx);

	/* Ether Loopback RX Interrupt Count Register */
	netsec_reg_write(drv, OGMA_REG_ADDR_ETH_LPB_RX_RXINT_CNT, 1);

	/* Ether Loopback RX Channel Interrupt Enable */
	rx = RX_CHAN_INTEN_TR_ERR | RX_CHAN_INTEN_PKT_CNT;
	netsec_reg_write(drv, OGMA_REG_ADDR_ETH_LPB_RX_INTEN_SET, rx);

	/* Audio TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_ADO_TX_INTEN_SET, tx);

	/* RTP Loopback RX Interrupt Count Register */
	netsec_reg_write(drv, OGMA_REG_ADDR_RTP_LPB_RX_RXINT_CNT, 1);

	/* RTP Loopback RX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_RTP_LPB_RX_INTEN_SET, rx);

	/* Context DB TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_CDB_TX_INTEN_SET, tx);

	/* Crypt TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_CRY_TX_INTEN_SET, tx);

	/* Crypt RX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_CRY_RX_INTEN_SET, rx);

	/* JPEG1 TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_JPG1_TX_INTEN_SET, tx);

	/* JPEG2 TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_JPG2_TX_INTEN_SET, tx);

	/* Video Sub TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_VSUB_TX_INTEN_SET, tx);

	/* Metadata TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_MDT_TX_INTEN_SET, tx);

	/* TS TX Channel Interrupt Enable */
	netsec_reg_write(drv, OGMA_REG_ADDR_TS_TX_INTEN_SET, tx);

	/* Top Interrupt Enable A */
	val = TOP_INTEN_INT_MAC_ST |
	    TOP_INTEN_INT_PKT_ST |
	    TOP_INTEN_INT_TS_TX_ST |
	    TOP_INTEN_INT_MDT_TX_ST |
	    TOP_INTEN_INT_VSUB_TX_ST |
	    TOP_INTEN_INT_JPG2_TX_ST |
	    TOP_INTEN_INT_JPG1_TX_ST |
	    TOP_INTEN_INT_CRY_RX_ST |
	    TOP_INTEN_INT_CRY_TX_ST |
	    TOP_INTEN_INT_CDB_TX_ST |
	    TOP_INTEN_INT_RTP_LPB_RX_ST |
	    TOP_INTEN_INT_ADO_TX_ST |
	    TOP_INTEN_INT_ETH_LPB_RX_ST | TOP_INTEN_INT_VDO_TX_ST;
	netsec_reg_write(drv, OGMA_REG_ADDR_TOP_INTEN_A_SET, val);

	/* Top Interrupt Enable B */
	val = TOP_INTEN_INT_PKT_RX_ST | TOP_INTEN_INT_PKT_TX_ST;
	netsec_reg_write(drv, OGMA_REG_ADDR_TOP_INTEN_B_SET, val);

	return 0;
}

static int netsec_disable_irq(NETSEC_DRIVER_DATA * drv)
{
	u32 val;

	val = 0x003f7f7f;	/* all clear */

	/* Top Interrupt Enable A */
	netsec_reg_write(drv, OGMA_REG_ADDR_TOP_INTEN_A_CLR, val);

	/* Top Interrupt Enable B */
	netsec_reg_write(drv, OGMA_REG_ADDR_TOP_INTEN_B_CLR, val);

	return 0;
}

#define	NETSEC_SDB_CMD_ST		(0x80000000)
#define	NETSEC_SDB_CMD_ID_MASK		(0x0003f000)
#define	NETSEC_SDB_CMD_ID_SHIFT		(12)
#define	NETSEC_SDB_CMD_ITEM_MASK	(0x00000f00)
#define	NETSEC_SDB_CMD_ITEM_SHIFT	(8)
#define	NETSEC_SDB_CMD_RDCLR		(0x00000002)
#define	NETSEC_SDB_CMD_WRITE		(0x00000001)

int netsec_sdb_write(NETSEC_DRIVER_DATA * drv, u32 sdb_id, u32 item_no, u32 val)
{
	u32 cmd, st;
	int timeout = 0x00800000;

	cmd = ((sdb_id << NETSEC_SDB_CMD_ID_SHIFT) & NETSEC_SDB_CMD_ID_MASK) |
	    ((item_no << NETSEC_SDB_CMD_ITEM_SHIFT) & NETSEC_SDB_CMD_ITEM_MASK)
	    | NETSEC_SDB_CMD_WRITE;

	netsec_reg_write(drv, OGMA_REG_ADDR_SDB_DATA, val);
	netsec_reg_write(drv, OGMA_REG_ADDR_SDB_CMD_ST, cmd);

	while (timeout-- > 0) {
		st = netsec_reg_read(drv, OGMA_REG_ADDR_SDB_CMD_ST);
		if ((st & NETSEC_SDB_CMD_ST) == 0) {
			break;
		}
	}

	if (timeout <= 0) {
		printk(KERN_ERR "%s: busy timeout!\n", __FUNCTION__);
		return -EBUSY;
	}

	return 0;
}

int netsec_sdb_read(NETSEC_DRIVER_DATA * drv, u32 sdb_id, u32 item_no,
		    u32 * val)
{
	u32 cmd, st;
	int timeout = 0x00800000;

	cmd = ((sdb_id << NETSEC_SDB_CMD_ID_SHIFT) & NETSEC_SDB_CMD_ID_MASK) |
	    ((item_no << NETSEC_SDB_CMD_ITEM_SHIFT) & NETSEC_SDB_CMD_ITEM_MASK);

	netsec_reg_write(drv, OGMA_REG_ADDR_SDB_CMD_ST, cmd);

	while (timeout-- > 0) {
		st = netsec_reg_read(drv, OGMA_REG_ADDR_SDB_CMD_ST);
		if ((st & NETSEC_SDB_CMD_ST) == 0) {
			break;
		}
	}

	*val = netsec_reg_read(drv, OGMA_REG_ADDR_SDB_DATA);

	if (timeout <= 0) {
		printk(KERN_ERR "%s: busy timeout!\n", __FUNCTION__);
		return -EBUSY;
	}

	return 0;
}

static u16 netsec_txdone_count(NETSEC_TX_DONE_QUE * q)
{

	if (q->wr_idx == q->rd_idx) {
		if (q->flags != 0) {
			// full
			return q->num;
		} else {
			return 0;
		}
	} else {
		if (q->wr_idx > q->rd_idx) {
			return (q->wr_idx - q->rd_idx);
		} else {
			return (q->num - (q->rd_idx - q->wr_idx));
		}
	}
}

static int netsec_txdone_add(NETSEC_TX_DONE_QUE * q, u32 uid, u32 rslt)
{
	NETSEC_TX_DONE *txd;
	u16 count;

	count = netsec_txdone_count(q);
	if (count < q->num) {
		txd = &q->txque[q->wr_idx];
		txd->uniq_id = uid;
		txd->result = rslt;

		q->wr_idx++;
		if (q->wr_idx >= q->num) {
			q->wr_idx = 0;
		}
		if (q->wr_idx == q->rd_idx) {
			q->flags = 1;
		}
		return 1;
	}
	return 0;
}

static int netsec_txdone_get(NETSEC_TX_DONE_QUE * q, NETSEC_TX_DONE * pdone)
{
	NETSEC_TX_DONE *txd;
	u16 count;

	count = netsec_txdone_count(q);
	if (count > 0) {
		txd = &q->txque[q->rd_idx];
		pdone->uniq_id = txd->uniq_id;
		pdone->result = txd->result;

		q->rd_idx++;
		if (q->rd_idx >= q->num) {
			q->rd_idx = 0;
		}
		q->flags = 0;
		return 1;
	}
	return 0;
}

int netsec_get_pkt_tx_free(void *netsec_handle)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) netsec_handle;
	int nfree;

	nfree =
	    netsec_dring_get_tx_free(&drv->dring[NETSEC_DESC_RING_ID_PKT_TX]);

	return nfree;
}

int netsec_put_pkt_tx(void *netsec_handle, u32 * desc, void *info)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) netsec_handle;
	int ret;

	ret =
	    netsec_dring_put_tx(&drv->dring[NETSEC_DESC_RING_ID_PKT_TX], desc,
				(u32) info);

	return ret;
}

int netsec_clean_pkt_tx(void *netsec_handle)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) netsec_handle;
	int ret;

	ret = netsec_dring_clear_tx(&drv->dring[NETSEC_DESC_RING_ID_PKT_TX]);

	return ret;
}

int netsec_get_pkt_rx(void *netsec_handle, u32 * desc, void **info)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) netsec_handle;
	int ret;

	ret =
	    netsec_dring_get_rx(&drv->dring[NETSEC_DESC_RING_ID_PKT_RX],
				(NETSEC_DESC *) desc, (u32 *) info);

	return ret;
}

int netsec_set_pkt_rx(void *netsec_handle, u32 * desc, void *info)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) netsec_handle;
	int ret;

	ret =
	    netsec_dring_reset_rx(&drv->dring[NETSEC_DESC_RING_ID_PKT_RX],
				  (NETSEC_DESC *) desc, (u32) info);

	return ret;
}

/* .read */
static ssize_t netsec_driver_read(struct file *filp, char __user * buf,
				  size_t count, loff_t * f_pos)
{
	NETSEC_DRIVER_DATA *drv = filp->private_data;
	unsigned long flags;
	ssize_t retval = 0;
	int ret;
	NETSEC_TX_DONE txd;

	if (buf == NULL) {
		return -EFAULT;
	}

	if (down_trylock(&drv->txdones)) {
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		if (down_interruptible(&drv->txdones)) {
			return -ERESTARTSYS;
		}
	}

	spin_lock_irqsave(&drv->tx_done_lock, flags);

	while (retval < count) {
		if ((count - retval) < sizeof(NETSEC_TX_DONE)) {
			break;
		}

		if (!access_ok(VERIFY_WRITE, (void __user *)buf, sizeof(txd))) {
			printk(KERN_ERR
			       "%s: cause mem fault to write txdone to user\n",
			       __FUNCTION__);
			retval = -EFAULT;
			break;
		}
		ret = netsec_txdone_get(&drv->tx_done_que, &txd);
		if (ret == 0) {
			break;
		}

		if (copy_to_user((void *)buf, &txd, sizeof(txd))) {
			printk(KERN_ERR
			       "%s: mem fault to write txdone to user\n",
			       __FUNCTION__);
			retval = -EFAULT;
			break;
		}

		buf += sizeof(NETSEC_TX_DONE);
		retval += sizeof(NETSEC_TX_DONE);
	}

	spin_unlock_irqrestore(&drv->tx_done_lock, flags);

	up(&drv->txdones);

	return retval;
}

static u32 tx_cnt_reg_addr[] = {
	OGMA_REG_ADDR_PKT_TX_CNT,
	0,
	OGMA_REG_ADDR_VDO_TX_CNT,
	0,
	OGMA_REG_ADDR_ADO_TX_CNT,
	0,
	OGMA_REG_ADDR_CDB_TX_CNT,
	0,
	OGMA_REG_ADDR_CRY_TX_CNT,
	0,
	OGMA_REG_ADDR_JPG1_TX_CNT,
	OGMA_REG_ADDR_JPG2_TX_CNT,
	OGMA_REG_ADDR_VSUB_TX_CNT,
	OGMA_REG_ADDR_MDT_TX_CNT,
	OGMA_REG_ADDR_TS_TX_CNT,
};

/* .write */
static ssize_t netsec_driver_write(struct file *filp, const char __user * buf,
				   size_t count, loff_t * f_pos)
{
	NETSEC_DRIVER_DATA *drv = filp->private_data;
	ssize_t n = 0;
	u32 txnum = 0;
	NETSEC_DRING *d;
	u32 tx_cnt_reg;
	int drid;
	int dtid;
	int last_drid = -1;
	int ret;
	unsigned long flags;
	char *bufp, *bufp0;
	char *bufp_ptr;
	size_t cnt;
	int desc_num, nspace;
	struct netsec_desc *dptr;
	struct netsec_desc bp_buf[32];

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	if (count < sizeof(NETSEC_DESC)) {
		return 0;
	}

	/* read descriptors from user space */
	if (count > sizeof(bp_buf)) {
		bufp_ptr = kzalloc(count, GFP_NOWAIT);
		if (bufp_ptr == NULL) {
			printk(KERN_ERR "%s: mem alloc error for desc %d\n",
			       __FUNCTION__, count);
			return -ENOMEM;
		}
		bufp0 = bufp_ptr;
	} else {
		bufp_ptr = NULL;
		bufp0 = (char *)bp_buf;
	}

	if (copy_from_user(bufp0, (void *)buf, count)) {
		printk(KERN_ERR "%s: mem fault to read desc from user\n",
		       __FUNCTION__);
		if (bufp_ptr != NULL) {
			kfree(bufp_ptr);
		}
		return -EFAULT;
	}

	/* check and count descriptors */
	bufp = bufp0;

	cnt = count;
	desc_num = 0;
	for (;;) {
		if (cnt < sizeof(struct netsec_desc)) {
			break;
		}
		dptr = (struct netsec_desc *)bufp;
		drid =
		    (dptr->desc[0] & NETSEC_DESC_DRID_MASK) >>
		    NETSEC_DESC_DRID_SHIFT;
		if (drid != last_drid) {
			if (last_drid == -1) {
				last_drid = drid;
			} else {
				if (bufp_ptr != NULL) {
					kfree(bufp_ptr);
				}
				return 0;
			}
		}

		if (drid == NETSEC_DESC_RING_ID_CRY_TX) {
			bufp += sizeof(u32) * 8;
			cnt -= (sizeof(u32) * 8);
		}

		bufp += sizeof(struct netsec_desc);
		cnt -= sizeof(struct netsec_desc);
		desc_num++;
	}
	d = &drv->dring[drid];

	spin_lock_irqsave(&drv->init_spinlock, flags);

	/* check descriptor write space */
	nspace = netsec_dring_get_tx_free(d);
	if (nspace < 0) {
		spin_unlock_irqrestore(&drv->init_spinlock, flags);
		if (bufp_ptr != NULL) {
			kfree(bufp_ptr);
		}
		printk(KERN_ERR "%s: write invalid descriptor ring\n",
		       __FUNCTION__);
		return 0;
	}
	if (nspace < desc_num) {
		spin_unlock_irqrestore(&drv->init_spinlock, flags);
		if (bufp_ptr != NULL) {
			kfree(bufp_ptr);
		}
		return 0;
	}

	/* write descriptors */
	last_drid = -1;
	bufp = bufp0;

	for (;;) {

		if ((count - n) < sizeof(struct netsec_desc)) {
			break;
		}

		/* DRID */
		dptr = (struct netsec_desc *)bufp;
		drid =
		    (dptr->desc[0] & NETSEC_DESC_DRID_MASK) >>
		    NETSEC_DESC_DRID_SHIFT;

		if (drid != last_drid) {
			if (last_drid != -1) {
				if (txnum > 0) {
					netsec_reg_write(drv, tx_cnt_reg,
							 txnum);
					txnum = 0;
				}
			}
			last_drid = drid;
		}

		switch (drid) {
		case NETSEC_DESC_RING_ID_VDO_TX:
		case NETSEC_DESC_RING_ID_ADO_TX:
		case NETSEC_DESC_RING_ID_CDB_TX:
		case NETSEC_DESC_RING_ID_CRY_TX:
		case NETSEC_DESC_RING_ID_JPG1_TX:
		case NETSEC_DESC_RING_ID_JPG2_TX:
		case NETSEC_DESC_RING_ID_VSUB_TX:
		case NETSEC_DESC_RING_ID_MDT_TX:
		case NETSEC_DESC_RING_ID_TS_TX:
			d = &drv->dring[drid];
			tx_cnt_reg = tx_cnt_reg_addr[drid];
			break;

		default:
			printk(KERN_ERR
			       "%s: invalid descriptor ring id 0x%x \n",
			       __FUNCTION__, drid);
			goto end0;
		}

		dptr->desc[0] &=
		    ~(NETSEC_DESC_OWN | NETSEC_DESC_LD);
		dtid =
		    ((dptr->desc[0] & NETSEC_DESC_DTID_MASK) >>
		     NETSEC_DESC_DTID_SHIFT);

		if (dptr->desc[0] & NETSEC_DESC_LS) {
			txnum++;
		}

		ret = netsec_dring_put_tx(d, &dptr->desc[0], dptr->uniq_id);
		if (ret != 1) {
			printk(KERN_ERR
			       "%s: write error descriptor ring id 0x%x \n",
			       __FUNCTION__, drid);
		}

		if (drid == NETSEC_DESC_RING_ID_CRY_TX) {
			bufp += sizeof(u32) * 8;
			n += (sizeof(u32) * 8);
		}
		n += sizeof(struct netsec_desc);
		bufp += sizeof(struct netsec_desc);
	}

 end0:

	if (txnum > 0) {
		netsec_reg_write(drv, tx_cnt_reg, txnum);
	}

	spin_unlock_irqrestore(&drv->init_spinlock, flags);

	if (bufp_ptr != NULL) {
		kfree(bufp_ptr);
	}

	return n;
}

/* .poll */
static unsigned int netsec_driver_poll(struct file *filp, poll_table * wait)
{
	NETSEC_DRIVER_DATA *drv = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &drv->txdoneq, wait);

	spin_lock_irqsave(&drv->tx_done_lock, flags);
	if (netsec_txdone_count(&drv->tx_done_que) > 0) {
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&drv->tx_done_lock, flags);

	return mask;
}

static void netsec_irq_mac_st_work ( struct work_struct *work )
{
	NETSEC_DRIVER_DATA *drv = container_of(work, NETSEC_DRIVER_DATA, mac_st_work);
	u32 val;
	u32 rgsr = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_RSR);

	// Link-up
	val = 0x00000000;	// Disable MAC Interrupt
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTEN, val);

	/* MAC Soft-Reset */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_BMR, 0x00020181);

	for (;;) {
		val = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_BMR);
		if ((val & 0x00000001) == 0) {
			break;
		}
	}

	/* MAC Descriptor Soft-Reset */
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_DESC_SOFT_RST, 0x1);

	for (;;) {
		val = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_DESC_SOFT_RST);
		if ((val & 0x00000001) == 0) {
			break;
		}
	}

	/* MAC Descriptor Initialize */
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_DESC_INIT, 0x1);

	for (;;) {
		val = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_DESC_INIT);
		if ((val & 0x00000001) == 0) {
			break;
		}
	}

	/* MMC Receive Interrupt All Masked */
	netsec_mac_reg_write( drv, OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_RX, 0x00FFFFFF);
	/* MMC Transmit Interrupt All Masked */
	netsec_mac_reg_write( drv, OGMA_GMAC_REG_ADDR_MMC_INTR_MASK_TX, 0x01FFFFFF);
	/* MMC Receive Checksum Offload Interrupt All Masked */
	netsec_mac_reg_write( drv, OGMA_GMAC_REG_ADDR_MMC_IPC_INTR_MASK_RX, 0x3FFF3FFF);

	/* Disable MAC LPI/Time Stamp/PMT Interrupt */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_IMR, 0x00000608);

	/* GMAC BMR */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_BMR, OGMA_GMAC_BMR_REG_COMMON);
	/* GMAC RDLAR */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_RDLAR, OGMA_GMAC_RDLAR_REG_COMMON);
	/* GMAC TDLAR */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_TDLAR, OGMA_GMAC_TDLAR_REG_COMMON);
	if (drv->netdev_p != NULL) {
		/* GMAC MFFR */
		ogma_netdev_set_rx_mode(drv->netdev_p);
	} else {
		/* GMAC MFFR */
		netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_MFFR, 0x80000001UL);
	}

	/* GMAC OMR */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_OMR, (OGMA_GMAC_OMR_REG_ST | OGMA_GMAC_OMR_REG_SR));

	if (rgsr & 0x00000001) {
		val = 0x0200080c;	// full duplex
	} else {
		val = 0x0201000c;	// half duplex
	}

	switch(rgsr & 0x00000006) {
	case 0x0:	/* 2.5MHz (10Mbps) */
#ifdef	FORCE_10MHALF_TO_10MFULL
		val = 0x0200a80c; // force 10Mbps, full duplex
#else	/* FORCE_10MHALF_TO_10MFULL */
		val |= 0x0000a000;
#endif	/* FORCE_10MHALF_TO_10MFULL */
		break;

	case 0x4:	/* 125MHz (1Gbps) */
		val |= 0x00002000;
		break;

	case 0x2:	/* 25MHz (100Mbps) */
	default:
		val |= 0x0000e000;
		break;

	}

	/* GMAC MCR */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_MCR, val);

	/* Wait for re-link-up */
	val = 0;
	do {
		udelay(100);
		rgsr = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_SR);
		val ++;
		if (val > 50000) {
			printk(KERN_ERR "%s: timeout for link-up status! (assume linked-up)\n", __FUNCTION__);
			break;
		}
	} while ((rgsr & 0x04000000) == 0);

	// clear staus again!
	rgsr = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_RSR);
	val = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_SR);
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_SR, val);
	val = MAC_INTEN_INT_SBD;
	netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTEN, val);

	if (drv->netdev_p != NULL) {
		netif_carrier_on(drv->netdev_p);
	}

}

static int netsec_ioctl_init_rxbuf(NETSEC_DRIVER_DATA * drv, NETSEC_DRING * d)
{
	NETSEC_DESC *dp;
	u8 *p;
	int ix;
	void *phys;
	struct sk_buff *skb;

	p = d->dring;
	for (ix = 0; ix < d->num_desc; ix++) {
		skb = dev_alloc_skb(d->buff_size + 2);
		if (skb == NULL) {
			printk(KERN_ERR "%s: failed to dev_alloc_skb(sz=%d)\n",
			       __FUNCTION__, d->buff_size + 2);
			break;
		}

		phys = (void *)dma_map_single(drv->dev,
					      skb->data,
					      (size_t) d->buff_size,
					      DMA_FROM_DEVICE);
		if (phys == NULL) {
			printk(KERN_ERR "%s: failed to dma_map_single(sz=%d)\n",
			       __FUNCTION__, d->buff_size + 2);
			dev_kfree_skb(skb);
			break;
		}

		dp = (NETSEC_DESC *) p;
		dp->desc[0] = 0x80000300;
		if (ix == (d->num_desc - 1)) {
			dp->desc[0] |= 0x40000000;
		}
		dp->desc[1] = (u32) phys;
		dp->desc[2] = d->buff_size;
		dp->desc[3] = 0;

		d->info[ix] = (u32) skb;

		p += d->size_desc;
	}

	if (ix != d->num_desc) {
		int x;

		p = d->dring;
		for (x = 0; x < ix; x++) {
			dp = (NETSEC_DESC *) p;
			dma_unmap_single(drv->dev,
					 dp->desc[1],
					 d->buff_size, DMA_FROM_DEVICE);
			dev_kfree_skb((struct sk_buff *)d->info[x]);
			p += d->size_desc;
		}

		return -ENOMEM;
	}

	return 0;
}

static void netsec_ioctl_exit_rxbuf(NETSEC_DRIVER_DATA * drv, NETSEC_DRING * d)
{
	NETSEC_DESC *dp;
	u8 *p;
	int ix;

	p = d->dring;
	for (ix = 0; ix < d->num_desc; ix++) {
		dp = (NETSEC_DESC *) p;
		dma_unmap_single(drv->dev,
				 dp->desc[1], d->buff_size, DMA_FROM_DEVICE);
		dev_kfree_skb((struct sk_buff *)d->info[ix]);
		p += d->size_desc;
	}
}

static int netsec_ioctl_init_mem(NETSEC_DRIVER_DATA * drv,
				 struct netsec_init_arg *param)
{
	int ret;

	ret = netsec_dring_rx_init(&drv->dring[NETSEC_DESC_RING_ID_PKT_RX],
				   param->num_descriptors[0],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_PKT_RX,
				   param->rx_buf_size[0]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d,sz=%d) PKT_RX\n",
		       __FUNCTION__, param->num_descriptors[0],
		       param->rx_buf_size[0]);
		goto err0;
	}

	ret = netsec_dring_rx_init(&drv->dring[NETSEC_DESC_RING_ID_ETH_LPB_RX],
				   param->num_descriptors[1],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_ETH_LPB_RX,
				   param->rx_buf_size[1]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d,sz=%d) ETH_LPB_RX\n",
		       __FUNCTION__, param->num_descriptors[1],
		       param->rx_buf_size[1]);
		goto err1;
	}

	ret = netsec_dring_rx_init(&drv->dring[NETSEC_DESC_RING_ID_RTP_LPB_RX],
				   param->num_descriptors[2],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_RTP_LPB_RX,
				   param->rx_buf_size[2]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d,sz=%d) RTP_LPB_RX\n",
		       __FUNCTION__, param->num_descriptors[2],
		       param->rx_buf_size[2]);
		goto err2;
	}

	ret = netsec_dring_rx_init(&drv->dring[NETSEC_DESC_RING_ID_CRY_RX],
				   param->num_descriptors[3],
				   sizeof(NETSEC_CRYPT_DESC),
				   NETSEC_DESC_RING_ID_CRY_RX,
				   param->rx_buf_size[3]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d,sz=%d) CRY_RX\n",
		       __FUNCTION__, param->num_descriptors[3],
		       param->rx_buf_size[3]);
		goto err3;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_PKT_TX],
				   param->num_descriptors[4],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_PKT_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) PKT_TX\n",
		       __FUNCTION__, param->num_descriptors[4]);
		goto err4;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_VDO_TX],
				   param->num_descriptors[5],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_VDO_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) VDO_TX\n",
		       __FUNCTION__, param->num_descriptors[5]);
		goto err5;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_ADO_TX],
				   param->num_descriptors[6],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_ADO_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) ADP_TX\n",
		       __FUNCTION__, param->num_descriptors[6]);
		goto err6;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_CDB_TX],
				   param->num_descriptors[7],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_CDB_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) CDB_TX\n",
		       __FUNCTION__, param->num_descriptors[7]);
		goto err7;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_CRY_TX],
				   param->num_descriptors[8],
				   sizeof(NETSEC_CRYPT_DESC),
				   NETSEC_DESC_RING_ID_CRY_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) CRY_TX\n",
		       __FUNCTION__, param->num_descriptors[8]);
		goto err8;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_JPG1_TX],
				   param->num_descriptors[9],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_JPG1_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) JPG1_TX\n",
		       __FUNCTION__, param->num_descriptors[9]);
		goto err9;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_JPG2_TX],
				   param->num_descriptors[10],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_JPG2_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) JPG2_TX\n",
		       __FUNCTION__, param->num_descriptors[10]);
		goto err10;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_VSUB_TX],
				   param->num_descriptors[11],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_VSUB_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) VSUB_TX\n",
		       __FUNCTION__, param->num_descriptors[11]);
		goto err11;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_MDT_TX],
				   param->num_descriptors[12],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_MDT_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) MDT_TX\n",
		       __FUNCTION__, param->num_descriptors[12]);
		goto err12;
	}

	ret = netsec_dring_tx_init(&drv->dring[NETSEC_DESC_RING_ID_TS_TX],
				   param->num_descriptors[13],
				   sizeof(NETSEC_DESC),
				   NETSEC_DESC_RING_ID_TS_TX);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_dring_rx_init(num=%d) TS_TX\n",
		       __FUNCTION__, param->num_descriptors[13]);
		goto err13;
	}

	ret =
	    netsec_ioctl_init_rxbuf(drv,
				    &drv->dring
				    [NETSEC_DESC_RING_ID_ETH_LPB_RX]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_ioctl_init_rxbuf ETH_LPB_RX\n",
		       __FUNCTION__);
		goto err15;
	}

	ret =
	    netsec_ioctl_init_rxbuf(drv,
				    &drv->dring
				    [NETSEC_DESC_RING_ID_RTP_LPB_RX]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_ioctl_init_rxbuf RTP_LPB_RX\n",
		       __FUNCTION__);
		goto err16;
	}

	ret =
	    netsec_ioctl_init_rxbuf(drv,
				    &drv->dring[NETSEC_DESC_RING_ID_CRY_RX]);
	if (ret != 0) {
		printk(KERN_ERR
		       "%s: failed to netsec_ioctl_init_rxbuf CRY_RX\n",
		       __FUNCTION__);
		goto err17;
	}

	/* Context DB */
	drv->ctx_data_addr = (u8 *) dma_alloc_coherent(NULL,
						       (size_t) param->size_ctx,
						       &drv->ctx_phys_addr,
						       GFP_KERNEL);
	if (drv->ctx_data_addr == NULL) {
		printk(KERN_ERR
		       "%s: failed to dma_alloc_coherent CTX (sz=%d)\n",
		       __FUNCTION__, param->size_ctx);
		goto err18;
	}
	drv->ctx_size = param->size_ctx;

	if (drv->reinit == 0) {
		drv->tx_done_que.txque =
		    (NETSEC_TX_DONE *) kzalloc(sizeof(NETSEC_TX_DONE) *
					       NETSEC_TX_DONE_QUE_NUM,
					       GFP_KERNEL);
		if (drv->tx_done_que.txque == NULL) {
			printk(KERN_ERR
			       "%s: failed to alloc tx_done_que (num=%d)\n",
			       __FUNCTION__, NETSEC_TX_DONE_QUE_NUM);
			goto err19;
		}
	}
	memset(drv->tx_done_que.txque, 0,
	       sizeof(NETSEC_TX_DONE) * NETSEC_TX_DONE_QUE_NUM);
	drv->tx_done_que.num = NETSEC_TX_DONE_QUE_NUM;
	drv->tx_done_que.rd_idx = 0;
	drv->tx_done_que.wr_idx = 0;
	drv->tx_done_que.flags = 0;

	return 0;

 err19:
	dma_free_coherent(NULL, (size_t) param->size_ctx, drv->ctx_data_addr,
			  drv->ctx_phys_addr);

 err18:
	netsec_ioctl_exit_rxbuf(drv, &drv->dring[NETSEC_DESC_RING_ID_CRY_RX]);
 err17:
	netsec_ioctl_exit_rxbuf(drv,
				&drv->dring[NETSEC_DESC_RING_ID_RTP_LPB_RX]);
 err16:
	netsec_ioctl_exit_rxbuf(drv,
				&drv->dring[NETSEC_DESC_RING_ID_ETH_LPB_RX]);
 err15:
//      netsec_ioctl_exit_rxbuf(drv, &drv->dring[NETSEC_DESC_RING_ID_PKT_RX]);
//err14:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_TS_TX]);
 err13:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_MDT_TX]);
 err12:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_VSUB_TX]);
 err11:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_JPG2_TX]);
 err10:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_JPG1_TX]);
 err9:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_CRY_TX]);
 err8:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_CDB_TX]);
 err7:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_ADO_TX]);
 err6:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_VDO_TX]);
 err5:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_PKT_TX]);
 err4:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_CRY_RX]);
 err3:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_RTP_LPB_RX]);
 err2:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_ETH_LPB_RX]);
 err1:
	netsec_dring_exit(&drv->dring[NETSEC_DESC_RING_ID_PKT_RX]);
 err0:

	return -ENOMEM;
}

static int netsec_ioctl_init_macro(NETSEC_DRIVER_DATA * drv,
				   struct netsec_init_arg *param)
{
	u32 val, num;
	u32 *hm_me;
	u32 *mh_me;
	u32 *pktc;
	phys_addr_t phys_mh_me;
	phys_addr_t phys_pktc;

	/* Initialize Register */
	netsec_reg_write(drv, OGMA_REG_ADDR_COM_INIT, 0x00000004);

	for (num = 0; num < NETSEC_COM_INIT_TIMEOUT; num++) {
		val = netsec_reg_read(drv, OGMA_REG_ADDR_COM_INIT);
		if ((val & 0x00000004) == 0) {
			break;
		}
	}
	if (num >= NETSEC_COM_INIT_TIMEOUT) {
		printk(KERN_ERR "%s: COM_INIT timeout!\n", __FUNCTION__);
		return -EINVAL;
	}
	/* not enable jumbo */

	/* Count Audio Divider Register */
	val = netsec_reg_read(drv, OGMA_REG_ADDR_CNT_ADO_DIV);
	netsec_reg_write(drv, OGMA_REG_ADDR_CNT_ADO_DIV, val | 0x80000000);

	/* Descriptor Ring Start Address Register */
	netsec_reg_write(drv,
			 OGMA_REG_ADDR_PKT_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_PKT_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_PKT_RX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_PKT_RX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_VDO_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_VDO_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_ETH_LPB_RX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_ETH_LPB_RX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_ADO_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_ADO_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_RTP_LPB_RX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_RTP_LPB_RX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_CDB_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_CDB_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_CRY_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_CRY_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_CRY_RX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_CRY_RX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_JPG1_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_JPG1_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_JPG2_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_JPG2_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_VSUB_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_VSUB_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_MDT_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_MDT_TX].phys_dring);

	netsec_reg_write(drv,
			 OGMA_REG_ADDR_TS_TX_DESC_START,
			 drv->dring[NETSEC_DESC_RING_ID_TS_TX].phys_dring);

	/* Channel Configuration Register */
	netsec_reg_write(drv, OGMA_REG_ADDR_PKT_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_PKT_RX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_VDO_TX_CONFIG, 0x00000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_ETH_LPB_RX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_ADO_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_RTP_LPB_RX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_CDB_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_CRY_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_CRY_RX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_JPG1_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_JPG2_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_VSUB_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_MDT_TX_CONFIG, 0xc0000001);
	netsec_reg_write(drv, OGMA_REG_ADDR_TS_TX_CONFIG, 0xc0000001);

	/* DMA Timer Control Register */

	if (drv->reinit == 0) {
		/* Alloc buffer */
		hm_me =
		    (u32 *) kzalloc(param->dmac_hm_me_size * sizeof(u32),
				    GFP_KERNEL);
		if (hm_me == NULL) {
			printk(KERN_ERR "%s: failed to alloc HM ME (sz=%d)\n",
			       __FUNCTION__, param->dmac_hm_me_size);
			return -ENOMEM;
		}

		mh_me = (u32 *) dma_alloc_coherent(NULL,
						   (size_t)
						   (param->dmac_mh_me_size *
						    sizeof(u32)), &phys_mh_me,
						   GFP_KERNEL);
		if (mh_me == NULL) {
			printk(KERN_ERR "%s: failed to alloc MH ME (sz=%d)\n",
			       __FUNCTION__, param->dmac_mh_me_size);
			kfree(hm_me);
			return -ENOMEM;
		}

		pktc = (u32 *) dma_alloc_coherent(NULL,
						  (size_t) (param->pktc_size *
							    sizeof(u32)),
						  &phys_pktc, GFP_KERNEL);
		if (pktc == NULL) {
			printk(KERN_ERR "%s: failed to alloc PKTC (sz=%d)\n",
			       __FUNCTION__, param->pktc_size);
			kfree(hm_me);
			dma_free_coherent(NULL,
					  (size_t) (param->dmac_mh_me_size *
						    sizeof(u32)), mh_me,
					  phys_mh_me);
			return -ENOMEM;
		}

		/* copy micro-code from user-area */
		if (copy_from_user
		    (hm_me, (void *)param->dmac_hm_me_code,
		     (param->dmac_hm_me_size * sizeof(u32)))
		    || copy_from_user(mh_me, (void *)param->dmac_mh_me_code,
				      (param->dmac_mh_me_size * sizeof(u32)))
		    || copy_from_user(pktc, (void *)param->pktc_code,
				      (param->pktc_size * sizeof(u32)))) {
			printk(KERN_ERR
			       "%s: mem fault to copy microcode from user\n",
			       __FUNCTION__);
			kfree(hm_me);
			dma_free_coherent(NULL,
					  (size_t) (param->dmac_mh_me_size *
						    sizeof(u32)), mh_me,
					  phys_mh_me);
			dma_free_coherent(NULL,
					  (size_t) (param->pktc_size *
						    sizeof(u32)), pktc,
					  phys_pktc);
			return -EFAULT;
		}
		wmb();

		/* DMAC HM ME Micro-code */
		for (num = 0; num < param->dmac_hm_me_size; num++) {
			netsec_reg_write(drv, OGMA_REG_ADDR_DMAC_HM_CMD_BUF,
					 hm_me[num]);
		}

		/* DMAC MH Micro-code */
		netsec_reg_write(drv, OGMA_REG_ADDR_DMAC_MC_ADDR_MH,
				 phys_mh_me);
		netsec_reg_write(drv, OGMA_REG_ADDR_DMAC_MC_SIZE_MH,
				 param->dmac_mh_me_size);

		/* PKTC Micro-code */
		netsec_reg_write(drv, OGMA_REG_ADDR_PKTC_MC_ADDR, phys_pktc);
		netsec_reg_write(drv, OGMA_REG_ADDR_PKTC_MC_SIZE,
				 param->pktc_size);

		/* Disable Core Register */
		netsec_reg_write(drv, OGMA_REG_ADDR_DIS_CORE, 0x00000000);

		/* wait int_me_st */
		msleep(1);
		for (num = 0; num < NETSEC_CODE_LOAD_END_TIMEOUT; num++) {
			val = netsec_reg_read(drv, OGMA_REG_ADDR_TOP_STATUS);
			if ((val & OGMA_TOP_IRQ_REG_CODE_LOAD_END) != 0) {
				break;
			}
			udelay(1);
		}
		if (num >= NETSEC_CODE_LOAD_END_TIMEOUT) {
			printk(KERN_ERR "%s: CODE_LOAD_END timeout!\n",
			       __FUNCTION__);
			return -EINVAL;
		}
		netsec_reg_write(drv, OGMA_REG_ADDR_TOP_STATUS,
				 OGMA_TOP_IRQ_REG_CODE_LOAD_END);

		/* Clock Enable Register 0/1 */
		netsec_reg_write(drv, OGMA_REG_ADDR_CLK_EN_0, 0x00000020);
		netsec_reg_write(drv, OGMA_REG_ADDR_CLK_EN_1, 0x00000020);

		/* MAC I/F Select Register */
		netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTF_SEL, 0x00000001);

		/* free memory */
		kfree(hm_me);
		dma_free_coherent(NULL,
				  (size_t) (param->dmac_mh_me_size *
					    sizeof(u32)), mh_me, phys_mh_me);
		dma_free_coherent(NULL,
				  (size_t) (param->pktc_size * sizeof(u32)),
				  pktc, phys_pktc);

	} else {

		/* Disable Core Register */
		netsec_reg_write(drv, OGMA_REG_ADDR_DIS_CORE, 0x00000000);

		/* MAC Descriptor Initialize Register */
		netsec_reg_write(drv, OGMA_REG_ADDR_MAC_DESC_INIT, 0x00000001);

		for (num = 0; num < NETSEC_MAC_DESC_INIT_TIMEOUT; num++) {
			val = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_DESC_INIT);
			if ((val & 0x00000001) == 0) {
				break;
			}
		}
		if (num >= NETSEC_MAC_DESC_INIT_TIMEOUT) {
			printk(KERN_ERR "%s: MAC_DESC_INIT timeout!\n",
			       __FUNCTION__);
			return -EINVAL;
		}

		/* MAC I/F Select Register */
		netsec_reg_write(drv, OGMA_REG_ADDR_MAC_INTF_SEL, 0x00000001);

	}

	/* Disable Counter */
	netsec_reg_write(drv, OGMA_REG_ADDR_CNT_EN, 0x00000000);

	/* setup audio divider */
	val = clk_get_rate(drv->netauclk) / 8000;
	if (val > 0) {
		val -= 1;
	} else {
		val = 0x09ff;
		printk(KERN_ERR "%s: invalid NETAUCLK? (rate=%lu)\n",
		       __FUNCTION__, clk_get_rate(drv->netauclk));
	}
	netsec_reg_write(drv, OGMA_REG_ADDR_CNT_ADO_DIV, (0x80000000 | val));

	/* Enable Counter */
	netsec_reg_write(drv, OGMA_REG_ADDR_CNT_EN, 0x00000007);

	return 0;
}

static int netsec_ioctl_init(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	int ret;
	int ix;
	struct netsec_init_arg param;
	struct netsec_rx_que_head {
		u32 num;
		u32 wr_count;
	} rq;

	if (copy_from_user(&param, (void *)arg, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy init-param from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	if (param.lpbk_rx_que == NULL) {
		printk(KERN_ERR "%s: lpbk_rx_que null\n", __FUNCTION__);
		return -EINVAL;
	}

	if (copy_from_user
	    (&rq, (void *)param.lpbk_rx_que,
	     sizeof(struct netsec_rx_que_head))) {
		printk(KERN_ERR
		       "%s: mem fault to copy NETSEC_RX_QUE from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	drv->lpbk_rx_que = param.lpbk_rx_que;
	drv->lpbk_rx_que_tsk = current;

	if (drv->init_done != 0) {
		return -EACCES;
	}

	if (param.dmac_hm_me_code == NULL) {
		printk(KERN_ERR "%s: dmac_hm_me_code NULL\n", __FUNCTION__);
		return -EINVAL;
	}
	if (param.dmac_mh_me_code == NULL) {
		printk(KERN_ERR "%s: dmac_mh_me_code NULL\n", __FUNCTION__);
		return -EINVAL;
	}
	if (param.pktc_code == NULL) {
		printk(KERN_ERR "%s: pktc_code NULL\n", __FUNCTION__);
		return -EINVAL;
	}

	for (ix = 0; ix < 14; ix++) {
		if ((param.num_descriptors[ix] < NETSEC_NUM_DESCRIPTORS_MIN) ||
		    (param.num_descriptors[ix] > NETSEC_NUM_DESCRIPTORS_MAX)) {
			break;
		}
		drv->num_descriptors[ix] = param.num_descriptors[ix];
	}
	if (ix < 14) {
		printk(KERN_ERR "%s: num descriptor out of range (%d)\n",
		       __FUNCTION__, ix);
		return -EINVAL;
	}

	for (ix = 0; ix < 4; ix++) {
		if ((param.rx_buf_size[ix] < NETSEC_RX_BUF_SIZE_MIN) ||
		    (param.rx_buf_size[ix] > NETSEC_RX_BUF_SIZE_MAX)) {
			break;
		}
		drv->rx_buf_size[ix] = param.rx_buf_size[ix];
	}
	if (ix < 4) {
		printk(KERN_ERR "%s: rx_buf_size out of range (%d)\n",
		       __FUNCTION__, ix);
		return -EINVAL;
	}

	if ((param.size_ctx < NETSEC_SIZE_CTX_MIN) ||
	    (param.size_ctx > NETSEC_SIZE_CTX_MAX)) {
		printk(KERN_ERR "%s: size_ctx out of range\n", __FUNCTION__);
		return -EINVAL;
	}
	drv->ctx_size = param.size_ctx;

	ret = netsec_ioctl_init_mem(drv, &param);
	if (ret != 0) {
		return ret;
	}

	ret = netsec_ioctl_init_macro(drv, &param);
	if (ret != 0) {
		spin_unlock(&drv->init_spinlock);
		return ret;
	}

	if (drv->reinit == 0) {

		for (ix = 0; ix < ETH_ALEN; ix++) {
			drv->mac[ix] = param.mac[ix];
		}

		drv->priv = NULL;
		ogma_probe(drv->dev, drv, drv->base, drv->irqb,
				   drv->mac,
				   param.num_descriptors[4], param.num_descriptors[0],
				   &drv->netdev_p, &drv->priv);

		if (drv->netdev_p != NULL) {
			netif_carrier_off(drv->netdev_p);
		}
	}

	drv->mac_st_tsk = create_singlethread_workqueue("mac_st_tsk");
	INIT_WORK(&drv->mac_st_work, netsec_irq_mac_st_work);

	netsec_enable_irq(drv);

	drv->init_done = 1;

	return 0;
}

static void netsec_reset_tx_dring(NETSEC_DRIVER_DATA * drv, NETSEC_DRING * d)
{
	int ret;
	u32 info;
	NETSEC_DESC dscr;

	for (;;) {
		ret = netsec_dring_get_tx(d, &dscr, &info);
		if (ret != 1) {
			break;
		}

		netsec_dring_clear_tx(d);

		netsec_txdone_add(&drv->tx_done_que, info,
				  NETSEC_COMP_RESULT_RESET);
	}

}

static int netsec_macro_reset(NETSEC_DRIVER_DATA * drv)
{
	u32 val;
	u32 num;

	netsec_disable_irq(drv);

	/* DMA HM ME/MH ME Control Register */
	val = netsec_reg_read(drv, OGMA_REG_ADDR_DMAC_MC_SIZE_MH);
	if (val != 0) {
		netsec_reg_write(drv, OGMA_REG_ADDR_DMA_HM_CTRL, 0x00000001);
		netsec_reg_write(drv, OGMA_REG_ADDR_DMA_MH_CTRL, 0x00000001);

		for (num = 0; num < NETSEC_DMA_SWRST_TIMEOUT; num++) {
			val = netsec_reg_read(drv, OGMA_REG_ADDR_DMA_HM_CTRL);
			if ((val & 0x00000001) == 0) {
				break;
			}
		}

		for (; num < NETSEC_DMA_SWRST_TIMEOUT; num++) {
			val = netsec_reg_read(drv, OGMA_REG_ADDR_DMA_MH_CTRL);
			if ((val & 0x00000001) == 0) {
				break;
			}
		}
		if (num >= NETSEC_DMA_SWRST_TIMEOUT) {
			printk(KERN_ERR "%s: DMA_SWRST timeout!\n",
			       __FUNCTION__);
			return -EINVAL;
		}
	}

	/* MAC Soft-Reset */
	netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_BMR, 0x00020181);

	for (num = 0; num < NETSEC_GMAC_SWRST_TIMEOUT; num++) {
		val = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_BMR);
		if ((val & 0x00000001) == 0) {
			break;
		}
	}
	if (num >= NETSEC_GMAC_SWRST_TIMEOUT) {
		printk(KERN_ERR "%s: GMAC_SWRST timeout!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* Soft Reset Register */
	netsec_reg_write(drv, OGMA_REG_ADDR_SOFT_RST, 0x00000000);
	netsec_reg_write(drv, OGMA_REG_ADDR_SOFT_RST, 0x80000000);

	return 0;
}

static int netsec_ioctl_reset(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	int ret;
	unsigned long flags;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	spin_lock_irqsave(&drv->init_spinlock, flags);

	ret = netsec_macro_reset(drv);
	if (ret != 0) {
		spin_unlock_irqrestore(&drv->init_spinlock, flags);
		return ret;
	}

	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_VDO_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_ADO_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_CDB_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_CRY_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_JPG1_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_JPG2_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_VSUB_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_MDT_TX]);
	netsec_reset_tx_dring(drv, &drv->dring[NETSEC_DESC_RING_ID_TS_TX]);

	drv->init_done = 0;
	drv->reinit = 1;

	spin_unlock_irqrestore(&drv->init_spinlock, flags);

	return 0;
}

static int netsec_ioctl_smode(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	u32 pkt_ctrl;
	u32 strm_ctrl;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	pkt_ctrl = (arg & NETSEC_MODE_MASK_PKT_CTRL1);
	pkt_ctrl |=
	    ((arg & NETSEC_MODE_MASK_PKT_CTRL2) << NETSEC_MODE_SHIFT_PKT_CTRL2);
	strm_ctrl = (arg & NETSEC_MODE_MASK_STRM_CTRL);
	netsec_reg_write(drv, OGMA_REG_ADDR_PKT_CTRL, pkt_ctrl);
	netsec_reg_write(drv, OGMA_REG_ADDR_STRM_CTRL, strm_ctrl);

	return 0;
}

static int netsec_ioctl_gmode(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	u32 pkt_ctrl;
	u32 strm_ctrl;
	u32 param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	pkt_ctrl = netsec_reg_read(drv, OGMA_REG_ADDR_PKT_CTRL);
	strm_ctrl = netsec_reg_read(drv, OGMA_REG_ADDR_STRM_CTRL);
	param = (pkt_ctrl & NETSEC_REG_MASK_PKT_CTRL2);
	param >>= NETSEC_MODE_SHIFT_PKT_CTRL2;
	param |= (pkt_ctrl & NETSEC_MODE_MASK_PKT_CTRL1);
	param |= (strm_ctrl & NETSEC_MODE_MASK_STRM_CTRL);

	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static int netsec_ioctl_ssessdb(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	int item, ret;
	int num_item = 0;
	u32 val;
	unsigned long flags;
	struct netsec_sessdb_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	if (copy_from_user(&param, (void *)arg, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy param from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	if ((param.sess_db_id >= NETSEC_SESSDB_STRM_ID_MIN) &&
	    (param.sess_db_id <= NETSEC_SESSDB_STRM_ID_MAX)) {
		num_item = 16;
	} else if ((param.sess_db_id >= NETSEC_SESSDB_BASIC_ID_MIN) &&
		   (param.sess_db_id <= NETSEC_SESSDB_BASIC_ID_MAX)) {
		/* replace source MAC address field */
		val = drv->mac[0];
		val <<= 8;
		val |= drv->mac[1];
		val <<= 8;
		val |= drv->mac[2];
		val <<= 8;
		val |= drv->mac[3];
		param.sess_db[3] = val;

		val = drv->mac[4];
		val <<= 8;
		val |= drv->mac[5];
		val <<= 16;
		val |= (param.sess_db[4] & 0x0000ffff);
		param.sess_db[4] = val;
		num_item = 15;
	} else {
		printk(KERN_ERR "%s: sess_db_id out of range (%d)\n",
		       __FUNCTION__, param.sess_db_id);
		return -EINVAL;
	}

	spin_lock_irqsave(&drv->sessdb_lock, flags);
	for (item = 0; item < num_item; item++) {
		ret =
		    netsec_sdb_write(drv, param.sess_db_id, item,
				     param.sess_db[item]);
		if (ret != 0) {
			spin_unlock_irqrestore(&drv->sessdb_lock, flags);
			printk(KERN_ERR "%s: failed to sdb_write\n",
			       __FUNCTION__);
			return ret;
		}
	}
	spin_unlock_irqrestore(&drv->sessdb_lock, flags);

	return 0;
}

static int netsec_ioctl_sctx(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	struct netsec_ctx_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	if (copy_from_user(&param, (void *)arg, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy param from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	if ((param.ctx_offset + param.ctx_size) > drv->ctx_size) {
		printk(KERN_ERR "%s: offset/size invalid (%x:%x)\n",
		       __FUNCTION__, param.ctx_offset, param.ctx_size);
		return -EINVAL;
	}

	if (param.ctx_data == NULL) {
		printk(KERN_ERR "%s: ctx_data null\n", __FUNCTION__);
		return -EINVAL;
	}

	if (copy_from_user(drv->ctx_data_addr + param.ctx_offset,
			   (void *)param.ctx_data, param.ctx_size)) {
		printk(KERN_ERR "%s: mem fault to copy ctx_data from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	param.ctx_addr = (u32) (drv->ctx_phys_addr + param.ctx_offset);
	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static int netsec_ioctl_gdrinfo(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	NETSEC_DRING *dring;
	struct netsec_drinfo_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	if (copy_from_user(&param, (void *)arg, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy param from user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	if ((param.desc_id == 0x07) || (param.desc_id > 0x0e)) {
		printk(KERN_ERR "%s: desc_id invalid (%d)\n", __FUNCTION__,
		       param.desc_id);
		return -EINVAL;
	}

	dring = &drv->dring[param.desc_id];

	param.total = dring->num_desc;
	if (dring->tx) {
		param.free = netsec_dring_get_tx_free(dring);
	} else {
		param.free = netsec_dring_get_rx_free(dring);
	}

	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static int netsec_ioctl_waitcrypt(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	return 0;
}

static int netsec_ioctl_rdcrypt(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	return 0;
}

static int netsec_ioctl_gtcnt90k(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	struct netsec_tcnt32_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	param.tcount = netsec_reg_read(drv, OGMA_REG_ADDR_CNT90K_32);

	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static int netsec_ioctl_gtcnt8k(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	struct netsec_tcnt32_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	param.tcount = netsec_reg_read(drv, OGMA_REG_ADDR_CNT_ADO_32);

	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static int netsec_ioctl_gtcnt27m(NETSEC_DRIVER_DATA * drv, unsigned long arg)
{
	struct netsec_tcnt64_arg param;

	if (drv->init_done == 0) {
		printk(KERN_ERR "%s: not initialized macro\n", __FUNCTION__);
		return -EACCES;
	}

	param.upper = netsec_reg_read(drv, OGMA_REG_ADDR_CNT27M_U32);
	param.lower = netsec_reg_read(drv, OGMA_REG_ADDR_CNT27M_L32);
	param.upper = netsec_reg_read(drv, OGMA_REG_ADDR_CNT27M_U32);

	if (copy_to_user((void *)arg, &param, sizeof(param))) {
		printk(KERN_ERR "%s: mem fault to copy to user\n",
		       __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static long netsec_driver_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	int err = 0;
	int rc = 0;
	NETSEC_DRIVER_DATA *drv = filp->private_data;

	if (_IOC_TYPE(cmd) != NETSEC_IOC_MAGIC) {
		printk(KERN_ERR "%s: invalid ioctl magic\n", __FUNCTION__);
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > NETSEC_IOC_MAXNR) {
		printk(KERN_ERR "%s: invalid ioctl number\n", __FUNCTION__);
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		printk(KERN_ERR "%s: access fault to arg\n", __FUNCTION__);
		return -EFAULT;
	}

	switch (cmd) {
	case NETSEC_IOCINIT:
		rc = netsec_ioctl_init(drv, arg);
		break;

	case NETSEC_IOCRESET:
		rc = netsec_ioctl_reset(drv, arg);
		break;

	case NETSEC_IOCSMODE:
		rc = netsec_ioctl_smode(drv, arg);
		break;

	case NETSEC_IOCGMODE:
		rc = netsec_ioctl_gmode(drv, arg);
		break;

	case NETSEC_IOCSSESSDB:
		rc = netsec_ioctl_ssessdb(drv, arg);
		break;

	case NETSEC_IOCSCTX:
		rc = netsec_ioctl_sctx(drv, arg);
		break;

	case NETSEC_IOCGDRINFO:
		rc = netsec_ioctl_gdrinfo(drv, arg);
		break;

	case NETSEC_IOCWAITCRYPT:
		rc = netsec_ioctl_waitcrypt(drv, arg);
		break;

	case NETSEC_IOCRDCRYPT:
		rc = netsec_ioctl_rdcrypt(drv, arg);
		break;

	case NETSEC_IOCGTCNT90K:
		rc = netsec_ioctl_gtcnt90k(drv, arg);
		break;

	case NETSEC_IOCGTCNT8K:
		rc = netsec_ioctl_gtcnt8k(drv, arg);
		break;

	case NETSEC_IOCGTCNT27M:
		rc = netsec_ioctl_gtcnt27m(drv, arg);
		break;

	default:
		printk(KERN_ERR "[ERROR] unknown command: %d\n", cmd);
		return -EINVAL;
	}

	return rc;
}

/* .open */
static int netsec_driver_open(struct inode *inode, struct file *filp)
{
	int rc;
	NETSEC_DRIVER_DATA *drv;

	rc = nonseekable_open(inode, filp);

	drv = container_of(inode->i_cdev, NETSEC_DRIVER_DATA, cdev);
	filp->private_data = drv;

	return rc;
}

/* .release */
static int netsec_driver_close(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static void netsec_irq_mac_st(NETSEC_DRIVER_DATA * drv)
{
	u32 mac_st = netsec_reg_read(drv, OGMA_REG_ADDR_MAC_STATUS);
	u32 val;

	/* INT_SBD */
	if (mac_st & MAC_INTEN_INT_SBD) {
		u32 rgsr;
		val = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_SR);

		rgsr = netsec_mac_reg_read(drv, OGMA_GMAC_REG_ADDR_RSR);

		netsec_mac_reg_write(drv, OGMA_GMAC_REG_ADDR_SR, val);

		/* GMAC Line Interface Interrupt */
		if (val & 0x04000000) {
		    if (rgsr & 0x00000008) {
				schedule_work(&drv->mac_st_work);
		    } else {
				// Link-down
				if (drv->netdev_p != NULL) {
					netif_carrier_off(drv->netdev_p);
				}
			}
		}
	}

}

#define	TX_STATUS_TX_NOT_OWNER		(0x00020000)
#define	TX_STATUS_TX_TR_ERR		(0x00010000)
#define	TX_STATUS_TX_DONE		(0x00008000)
#define	TX_STATUS_TX_ERR_ST_MASK	(0x000000c0)
#define	TX_STATUS_TX_ERR_ST_SHIFT	(6)

static void netsec_irq_tx_clear(NETSEC_DRIVER_DATA * drv, NETSEC_DRING * d,
				u32 tx_status_reg)
{
	int ret, count;
	unsigned long flags;
	u32 info;
	u32 status;
	u32 result;
	NETSEC_DESC dscr;

	count = 0;

	spin_lock_irqsave(&drv->tx_done_lock, flags);

	for (;;) {

		ret = netsec_dring_get_tx(d, &dscr, &info);
		if (ret != 1) {
			break;
		}

		netsec_dring_clear_tx(d);

		status = netsec_reg_read(drv, tx_status_reg);

		result =
		    ((status & TX_STATUS_TX_ERR_ST_MASK) >>
		     TX_STATUS_TX_ERR_ST_SHIFT);

		if (info == 0xffffffff) {
			continue;
		}

		ret = netsec_txdone_add(&drv->tx_done_que, info, result);
		if (ret > 0) {
			count++;
		}

	}

	if (count > 0) {
		wake_up(&drv->txdoneq);
	}

	spin_unlock_irqrestore(&drv->tx_done_lock, flags);

}

#define	PKT_STATUS_TX_DONE_TS			(0x10000000)
#define	PKT_STATUS_TX_DONE_META			(0x08000000)
#define	PKT_STATUS_TX_DONE_VSUB			(0x04000000)
#define	PKT_STATUS_TX_DONE_J2			(0x02000000)
#define	PKT_STATUS_TX_DONE_J1			(0x01000000)
#define	PKT_STATUS_TX_DONE_A			(0x00800000)
#define	PKT_STATUS_TX_DONE_5			(0x00200000)
#define	PKT_STATUS_TX_DONE_4			(0x00100000)
#define	PKT_STATUS_TX_DONE_3			(0x00080000)
#define	PKT_STATUS_TX_DONE_2			(0x00040000)
#define	PKT_STATUS_TX_DONE_1			(0x00020000)
#define	PKT_STATUS_TX_DONE_0			(0x00010000)
#define	PKT_STATUS_TLS_DEC_FAIL			(0x00000400)
#define	PKT_STATUS_TLS_AUTH_FAIL		(0x00000200)
#define	PKT_STATUS_TLS_BAD_ALIGN		(0x00000100)
#define	PKT_STATUS_MAC_ER				(0x00000020)
#define	PKT_STATUS_JUMBO_ER				(0x00000010)
#define	PKT_STATUS_CHKSUM_ER			(0x00000008)
#define	PKT_STATUS_HD_INCOMPLETE		(0x00000004)
#define	PKT_STATUS_HD_ER				(0x00000002)

#define	STRM_STATUS_LE			(0x00000001)

static void netsec_irq_strm_st(NETSEC_DRIVER_DATA * drv)
{
	u32 strm_st;

	strm_st = netsec_reg_read(drv, OGMA_REG_ADDR_STRM_STATUS);

	/* clear interrupt */
	netsec_reg_write(drv, OGMA_REG_ADDR_STRM_STATUS, strm_st);

	if (strm_st & STRM_STATUS_LE) {
		netsec_strm_status_le++;
	}
}

static void netsec_irq_tx_st(NETSEC_DRIVER_DATA * drv, u32 tx_status,
			     u32 tx_done_cnt)
{
	u32 tx_st;
	u32 tx_count;

	tx_st = netsec_reg_read(drv, tx_status);

	/* clear interrupt */
	netsec_reg_write(drv, tx_status, tx_st);

	if (tx_st & TX_STATUS_TX_DONE) {
		/* clear tx done counter */
		tx_count = netsec_reg_read(drv, tx_done_cnt);
	}
}

#define	RX_STATUS_TR_ERR			(0x00010000)
#define	RX_STATUS_PKT_CNT			(0x00008000)
#define	RX_STATUS_RX_ERR_ST_MASK	(0x0000000c)
#define	RX_STATUS_RX_ERR_ST_SHIFT	(2)

static void netsec_irq_crypt_rx(NETSEC_DRIVER_DATA * drv)
{
	u32 rx_st;
	u32 rx_count;
	int ret;
	NETSEC_CRYPT_DESC dscr;
	u32 info;

	rx_st = netsec_reg_read(drv, OGMA_REG_ADDR_CRY_RX_STATUS);

	/* clear interrupt */
	if (rx_st & RX_STATUS_TR_ERR) {
		netsec_reg_write(drv, OGMA_REG_ADDR_CRY_RX_STATUS,
				 rx_st & RX_STATUS_TR_ERR);
	}

	if (rx_st & RX_STATUS_PKT_CNT) {
		/* clear rx counter */
		rx_count = netsec_reg_read(drv, OGMA_REG_ADDR_CRY_RX_CNT);
	}

	for (;;) {
		ret =
		    netsec_dring_get_rx(&drv->dring[NETSEC_DESC_RING_ID_CRY_RX],
					(NETSEC_DESC *) & dscr, &info);
		if (ret == 0) {
			break;
		}

		dscr.desc[0] &= NETSEC_DESC_LD;
		dscr.desc[0] |= (NETSEC_DESC_FS | NETSEC_DESC_LS);
		dscr.desc[2] = drv->rx_buf_size[2];
		netsec_dring_reset_rx(&drv->dring[NETSEC_DESC_RING_ID_CRY_RX],
				      (NETSEC_DESC *) & dscr, info);
	}
}

#ifdef	CONFIG_F_NETSEC_TS_MODULE
/*
 * Access another process' address space as given in mm.  If non-NULL, use the
 * given task for page fault accounting.
 */
static int netsec_access_remote_vm(struct task_struct *tsk,
				   struct mm_struct *mm, unsigned long addr,
				   void *buf, int len, int write)
{
	struct vm_area_struct *vma;
	void *old_buf = buf;

	down_read(&mm->mmap_sem);
	/* ignore errors, just check how much was successfully transferred */
	while (len) {
		int bytes, ret, offset;
		void *maddr;
		struct page *page = NULL;

		ret = get_user_pages(tsk, mm, addr, 1, write, 1, &page, &vma);
		if (ret <= 0) {
			/*
			 * Check if this is a VM_IO | VM_PFNMAP VMA, which
			 * we can access using slightly different code.
			 */
#ifdef CONFIG_HAVE_IOREMAP_PROT
			vma = find_vma(mm, addr);
			if (!vma || vma->vm_start > addr)
				break;
			if (vma->vm_ops && vma->vm_ops->access)
				ret = vma->vm_ops->access(vma, addr, buf,
							  len, write);
			if (ret <= 0)
#endif
				break;
			bytes = ret;
		} else {
			bytes = len;
			offset = addr & (PAGE_SIZE - 1);
			if (bytes > PAGE_SIZE - offset)
				bytes = PAGE_SIZE - offset;

			maddr = kmap(page);
			if (write) {
				memcpy(maddr + offset, buf, bytes);
				set_page_dirty_lock(page);
			} else {
				memcpy(buf, maddr + offset, bytes);
			}
			kunmap(page);
			page_cache_release(page);
		}
		len -= bytes;
		buf += bytes;
		addr += bytes;
	}
	up_read(&mm->mmap_sem);

	return buf - old_buf;
}

/*
 * Access another process' address space.
 * Source/target buffer must be kernel space,
 * Do not walk the page table directly, use get_user_pages
 */
static int netsec_access_process_vm(struct task_struct *tsk, unsigned long addr,
				    void *buf, int len, int write)
{
	struct mm_struct *mm;
	int ret;

	mm = get_task_mm(tsk);
	if (!mm)
		return 0;

	ret = netsec_access_remote_vm(tsk, mm, addr, buf, len, write);
	mmput(mm);

	return ret;
}
#endif				/* CONFIG_F_NETSEC_TS_MODULE */

static int get_long(NETSEC_DRIVER_DATA * drv, u8 * addr, u32 * pval)
{
	int ret;
	u32 val = 0;

#ifdef	CONFIG_F_NETSEC_TS_MODULE
	ret =
	    netsec_access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr,
				     &val, sizeof val, 0);
#else
	ret =
	    access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr, &val,
			      sizeof val, 0);
#endif
	if (ret != sizeof val) {
		printk(KERN_DEBUG "%s: could not read access addr=%p, ret=%d\n",
		       __FUNCTION__, addr, ret);
		return -1;
	}
	*pval = val;

	return 0;
}

static int set_long(NETSEC_DRIVER_DATA * drv, u8 * addr, u32 val)
{
	int ret;

#ifdef	CONFIG_F_NETSEC_TS_MODULE
	ret =
	    netsec_access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr,
				     &val, sizeof val, 1);
#else
	ret =
	    access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr, &val,
			      sizeof val, 1);
#endif
	if (ret != sizeof val) {
		return -1;
	}
	return 0;
}

static int copy_data(NETSEC_DRIVER_DATA * drv, u8 * addr, u8 * data, u32 len)
{
	int ret;

#ifdef	CONFIG_F_NETSEC_TS_MODULE
	ret =
	    netsec_access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr,
				     data, len, 1);
#else
	ret =
	    access_process_vm(drv->lpbk_rx_que_tsk, (unsigned long)addr, data,
			      len, 1);
#endif
	if (ret != len) {
		return -1;
	}
	return 0;
}

static void netsec_rtp_lpb_rx_store(NETSEC_DRIVER_DATA * drv, u32 desc0,
				    u32 length, u8 * data)
{
	NETSEC_RX_QUE *rxq = drv->lpbk_rx_que;
	NETSEC_RX_DATA *rxd;
	u32 num;
	u32 wr_count;
	u32 len;
	u32 wix;

	if (get_long(drv, (u8 *) & rxq->num, &num) != 0) {
		return;
	}
	if (get_long(drv, (u8 *) & rxq->wr_count, &wr_count) != 0) {
		return;
	}
	wix = wr_count % num;
	rxd = &rxq->rxque[wix];
	if (get_long(drv, (u8 *) & rxd->length, &len) != 0) {
		return;
	}
	if (len == 0) {
		copy_data(drv, rxd->rx_buff, data, length);
		set_long(drv, (u8 *) & rxd->desc0, desc0);
		set_long(drv, (u8 *) & rxd->length, length);
		wr_count++;
		set_long(drv, (u8 *) & rxq->wr_count, wr_count);
	} else {
	}
}

static void netsec_irq_rtp_lpb_rx_work(struct work_struct *work)
{
	NETSEC_DRIVER_DATA *drv =
	    container_of(work, NETSEC_DRIVER_DATA, lpbk_rx_que_work);
	int ret;
	NETSEC_DESC dscr;
	u32 info;
	struct sk_buff *skb;

	if (down_interruptible(&drv->lpbk_rx_que_sem)) {
		return;
	}

	for (;;) {
		ret =
		    netsec_dring_get_rx(&drv->dring
					[NETSEC_DESC_RING_ID_RTP_LPB_RX],
					(NETSEC_DESC *) & dscr, &info);
		if (ret == 0) {
			break;
		}

		dma_unmap_single(drv->dev,
				 dscr.desc[1], dscr.desc[2], DMA_FROM_DEVICE);

		skb = (struct sk_buff *)info;
		netsec_rtp_lpb_rx_store(drv, dscr.desc[0], dscr.desc[2],
					skb->data);

		dscr.desc[0] &= NETSEC_DESC_LD;
		dscr.desc[0] |= (NETSEC_DESC_FS | NETSEC_DESC_LS);
		dscr.desc[2] = drv->rx_buf_size[2];
		netsec_dring_reset_rx(&drv->dring
				      [NETSEC_DESC_RING_ID_RTP_LPB_RX],
				      (NETSEC_DESC *) & dscr, info);
	}

	up(&drv->lpbk_rx_que_sem);
}

static void netsec_irq_rtp_lpb_rx(NETSEC_DRIVER_DATA * drv)
{
	u32 rx_st;
	u32 rx_count;

	rx_st = netsec_reg_read(drv, OGMA_REG_ADDR_RTP_LPB_RX_STATUS);

	/* clear interrupt */
	if (rx_st & RX_STATUS_TR_ERR) {
		netsec_reg_write(drv, OGMA_REG_ADDR_RTP_LPB_RX_STATUS,
				 rx_st & RX_STATUS_TR_ERR);
	}

	if (rx_st & RX_STATUS_PKT_CNT) {
		/* clear rx counter */
		rx_count = netsec_reg_read(drv, OGMA_REG_ADDR_RTP_LPB_RX_CNT);
	}

	schedule_work(&drv->lpbk_rx_que_work);
}

static void netsec_irq_eth_lpb_rx(NETSEC_DRIVER_DATA * drv)
{
	u32 rx_st;
	u32 rx_count;
	int ret;
	NETSEC_DESC dscr;
	u32 info;

	rx_st = netsec_reg_read(drv, OGMA_REG_ADDR_ETH_LPB_RX_STATUS);

	/* clear interrupt */
	if (rx_st & RX_STATUS_TR_ERR) {
		netsec_reg_write(drv, OGMA_REG_ADDR_ETH_LPB_RX_STATUS,
				 rx_st & RX_STATUS_TR_ERR);
	}

	if (rx_st & RX_STATUS_PKT_CNT) {
		/* clear rx counter */
		rx_count =
		    netsec_reg_read(drv, OGMA_REG_ADDR_ETH_LPB_RX_PKTCNT);
	}

	for (;;) {
		ret =
		    netsec_dring_get_rx(&drv->dring
					[NETSEC_DESC_RING_ID_ETH_LPB_RX],
					(NETSEC_DESC *) & dscr, &info);
		if (ret == 0) {
			break;
		}

		dscr.desc[0] &= NETSEC_DESC_LD;
		dscr.desc[0] |= (NETSEC_DESC_FS | NETSEC_DESC_LS);
		dscr.desc[2] = drv->rx_buf_size[1];
		netsec_dring_reset_rx(&drv->dring
				      [NETSEC_DESC_RING_ID_ETH_LPB_RX],
				      (NETSEC_DESC *) & dscr, info);
	}
}

#define	TOP_STATUS_INT_SUB_ST			(0x00200000)
#define	TOP_STATUS_INT_ME_START			(0x00100000)
#define	TOP_STATUS_INT_MAC_ST			(0x00080000)
#define	TOP_STATUS_INT_PKT_ST			(0x00040000)
#define	TOP_STATUS_INT_STRM_ST			(0x00020000)
#define	TOP_STATUS_INT_REDC_ST			(0x00010000)
#define	TOP_STATUS_INT_TS_TX_ST			(0x00004000)
#define	TOP_STATUS_INT_MDT_TX_ST		(0x00002000)
#define	TOP_STATUS_INT_VSUB_TX_ST		(0x00001000)
#define	TOP_STATUS_INT_JPG2_TX_ST		(0x00000800)
#define	TOP_STATUS_INT_JPG1_TX_ST		(0x00000400)
#define	TOP_STATUS_INT_CRY_RX_ST		(0x00000200)
#define	TOP_STATUS_INT_CRY_TX_ST		(0x00000100)
#define	TOP_STATUS_INT_CDB_TX_ST		(0x00000040)
#define	TOP_STATUS_INT_RTP_LPB_RX_ST		(0x00000020)
#define	TOP_STATUS_INT_ADO_TX_ST		(0x00000010)
#define	TOP_STATUS_INT_ETH_LPB_RX_ST		(0x00000008)
#define	TOP_STATUS_INT_VDO_TX_ST		(0x00000004)
#define	TOP_STATUS_INT_PKT_RX_ST		(0x00000002)
#define	TOP_STATUS_INT_PKT_TX_ST		(0x00000001)

static irqreturn_t netsec_irq_handler(int irq, void *dev_id)
{
	NETSEC_DRIVER_DATA *drv = (NETSEC_DRIVER_DATA *) dev_id;
	u32 top_status;

	top_status = netsec_reg_read(drv, OGMA_REG_ADDR_TOP_STATUS);

	if (top_status & TOP_STATUS_INT_MAC_ST) {
		netsec_irq_mac_st(drv);
	}

	if (top_status & TOP_STATUS_INT_STRM_ST) {
		netsec_irq_strm_st(drv);
	}

	if (top_status & TOP_STATUS_INT_TS_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_TS_TX_STATUS,
				 OGMA_REG_ADDR_TS_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_TS_TX],
							OGMA_REG_ADDR_TS_TX_STATUS);		
		
	}
	if (top_status & TOP_STATUS_INT_MDT_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_MDT_TX_STATUS,
				 OGMA_REG_ADDR_MDT_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_MDT_TX],
							OGMA_REG_ADDR_MDT_TX_STATUS);		
	}
	if (top_status & TOP_STATUS_INT_VSUB_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_VSUB_TX_STATUS,
				 OGMA_REG_ADDR_VSUB_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_VSUB_TX],
							OGMA_REG_ADDR_VSUB_TX_STATUS);				
	}
	if (top_status & TOP_STATUS_INT_JPG2_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_JPG2_TX_STATUS,
				 OGMA_REG_ADDR_JPG2_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_JPG2_TX],
							OGMA_REG_ADDR_JPG2_TX_STATUS);				
	}
	if (top_status & TOP_STATUS_INT_JPG1_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_JPG1_TX_STATUS,
				 OGMA_REG_ADDR_JPG1_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_JPG1_TX],
							OGMA_REG_ADDR_JPG1_TX_STATUS);				
	}
	if (top_status & TOP_STATUS_INT_CRY_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_CRY_TX_STATUS,
				 OGMA_REG_ADDR_CRY_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_CRY_TX],
							OGMA_REG_ADDR_CRY_TX_STATUS);				
	}
	if (top_status & TOP_STATUS_INT_CDB_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_CDB_TX_STATUS,
				 OGMA_REG_ADDR_CDB_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_CDB_TX],
							OGMA_REG_ADDR_CDB_TX_STATUS);						
	}
	if (top_status & TOP_STATUS_INT_ADO_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_ADO_TX_STATUS,
				 OGMA_REG_ADDR_ADO_TX_DONE_CNT);
		netsec_irq_tx_clear(drv,
							&drv->dring[NETSEC_DESC_RING_ID_ADO_TX],
							OGMA_REG_ADDR_ADO_TX_STATUS);		
	}
	if (top_status & TOP_STATUS_INT_VDO_TX_ST) {
		netsec_irq_tx_st(drv, OGMA_REG_ADDR_VDO_TX_STATUS,
				 OGMA_REG_ADDR_VDO_TX_DONE_CNT);

		netsec_irq_tx_clear(drv,
				    &drv->dring[NETSEC_DESC_RING_ID_VDO_TX],
				    OGMA_REG_ADDR_VDO_TX_STATUS);		
	}

	if (top_status & TOP_STATUS_INT_CRY_RX_ST) {
		netsec_irq_crypt_rx(drv);
	}

	if (top_status & TOP_STATUS_INT_RTP_LPB_RX_ST) {
		netsec_irq_rtp_lpb_rx(drv);
	}

	if (top_status & TOP_STATUS_INT_ETH_LPB_RX_ST) {
		netsec_irq_eth_lpb_rx(drv);
	}

	return IRQ_HANDLED;
}

/*
* The file operations for the rena_share_mem
*/
struct file_operations netsec_drv_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = netsec_driver_read,
	.write = netsec_driver_write,
	.poll = netsec_driver_poll,
	.unlocked_ioctl = netsec_driver_ioctl,
	.open = netsec_driver_open,
	.release = netsec_driver_close,
	.fasync = NULL,
};

typedef struct {
#ifdef CONFIG_PM_WARP
	NETSEC_DRIVER_DATA *drv;
	struct resource *drv_res;
#endif
} netsec_platform_data;

static int netsec_driver_probe(struct platform_device *pdev)
{
	int ret;
	NETSEC_DRIVER_DATA *drv;
	struct resource *res;
#ifdef CONFIG_PM_WARP
	netsec_platform_data *pdata;
#endif				/* CONFIG_PM_WARP */

	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv) {
		printk(KERN_ALERT "%s: cannot alloc driver data.\n",
		       __FUNCTION__);
		return -ENOMEM;
	}
#ifdef CONFIG_PM_WARP
	pdata = devm_kzalloc(&pdev->dev, sizeof(netsec_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdev->dev.platform_data = pdata;
	pdata->drv = drv;
#endif				/* CONFIG_PM_WARP */

	drv->dev_num = MKDEV(301, 0);
	ret = register_chrdev_region(drv->dev_num, DEV_COUNT, NETSEC_DEV_NAME);
	if (ret < 0) {
		printk(KERN_EMERG "%s: cannot register device number.\n",
		       __FUNCTION__);
		goto err1;
	}

	netsec_class = class_create(THIS_MODULE, "netsec");
	if (IS_ERR(netsec_class)) {
		ret = PTR_ERR(netsec_class);
		goto err2;
	}

	device_create(netsec_class, NULL, MKDEV(301, 0), NULL, "netsec0");

	cdev_init(&drv->cdev, &netsec_drv_fops);
	drv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&drv->cdev, drv->dev_num, DEV_COUNT);
	if (ret < 0) {
		printk(KERN_ALERT "%s: cannot register character device.\n",
		       __FUNCTION__);
		goto err3;
	}

	drv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing base resource\n");
		ret = -EINVAL;
		goto err4;
	}
#ifdef CONFIG_PM_WARP
	pdata->drv_res = res;
#endif				/* CONFIG_PM_WARP */

	drv->base = ioremap(res->start, res->end - res->start + 1);
	if (!drv->base) {
		dev_err(&pdev->dev, "ioremap_nocache() failed\n");
		ret = -EINVAL;
		goto err4;
	}
	printk(KERN_INFO "f_netsec_ts: io 0x%x-0x%x\n", res->start, res->end);

	pdev->dev.init_name = "ogma",
	    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	drv->rst = devm_reset_control_get_optional(&pdev->dev, NULL);
	if (IS_ERR(drv->rst)) {
		ret = PTR_ERR(drv->rst);
		if (ret == -EPROBE_DEFER)
			goto err5;
		drv->rst = NULL;
	} else {
		ret = reset_control_deassert(drv->rst);
		if (ret)
			goto err5;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		goto err5;
	}
	drv->irqa = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		goto err5;
	}

	drv->irqb = res->start;

	ret = request_irq(drv->irqa, netsec_irq_handler, IRQF_SHARED,
			  NETSEC_DEV_NAME, drv);

	if (ret) {
		dev_err(&pdev->dev, "request_irq() failed\n");
		goto err6;
	}

	printk(KERN_INFO "f_netsec_ts: irq %d, %d\n", drv->irqa, drv->irqb);

	drv->netauclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(drv->netauclk)) {
		dev_err(&pdev->dev, "clk_get(NETAUCLK) failed\n");
		goto err6;
	}
	ret = clk_prepare_enable(drv->netauclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prepare_enable() failed\n");
		goto err6;
	}

	drv->rclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(drv->rclk)) {
		drv->rclk = 0;
	}

	drv->osc_clk = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(drv->osc_clk)) {
		drv->osc_clk = 0;
	}

	ret = netsec_macro_reset(drv);
	if (ret != 0)
		goto err6;

	sema_init(&drv->txdones, 1);
	init_waitqueue_head(&drv->txdoneq);

	spin_lock_init(&drv->tx_done_lock);
	INIT_WORK(&drv->lpbk_rx_que_work, netsec_irq_rtp_lpb_rx_work);
	sema_init(&drv->lpbk_rx_que_sem, 1);

	spin_lock_init(&drv->sessdb_lock);
	spin_lock_init(&drv->init_spinlock);

	platform_set_drvdata(pdev, drv);

	printk(KERN_INFO "f_netsec_ts: initialized\n");

	return 0;

 err6:
	free_irq(drv->irqa, drv);
 err5:
	iounmap(drv->base);
 err4:
	cdev_del(&drv->cdev);
 err3:
	device_destroy(netsec_class, MKDEV(301, 0));
	class_destroy(netsec_class);

 err2:
	unregister_chrdev_region(drv->dev_num, DEV_COUNT);
 err1:
	kfree(drv);

	return ret;
}

static int netsec_driver_remove(struct platform_device *pdev)
{
	NETSEC_DRIVER_DATA *drv = platform_get_drvdata(pdev);

/* KMATSUI DEBUG */
	{
		int ix = 0x7FFFFFFF;
		while (ix-- > 0) {
			udelay(1000);
		}
	}
/* KMATSUI DEBUG */
	if ((drv->init_done != 0) || (drv->reinit != 0)) {
		netsec_disable_irq(drv);

		ogma_remove(pdev);
	}

	free_irq(drv->irqa, drv);
	iounmap(drv->base);

	cdev_del(&drv->cdev);
	device_destroy(netsec_class, MKDEV(301, 0));
	class_destroy(netsec_class);
	unregister_chrdev_region(drv->dev_num, DEV_COUNT);

	kfree(drv);

	return 0;
}
#ifdef CONFIG_PM_SLEEP
static int ogma_suspend(struct device *dev)
{
#ifdef CONFIG_PM_WARP
	netsec_platform_data *pdata = dev->platform_data;
	NETSEC_DRIVER_DATA *drv = pdata->drv;
	if (pm_device_down) {

		disable_irq(drv->irqa);
		if (drv->init_done == 1) {
			free_irq(drv->irqb, drv->netdev_p);
			drv->init_done = 0;			
			ogma_netdev_uninit(drv->netdev_p);
			ogma_cnt_minus();
		}
		clk_disable_unprepare(drv->netauclk);

		iounmap(drv->base);
	}
#endif				/* CONFIG_PM_WARP */

	return 0;
}
static int ogma_resume(struct device *dev)
{
#ifdef CONFIG_PM_WARP
	int ret;		
	netsec_platform_data *pdata = dev->platform_data;		
	NETSEC_DRIVER_DATA *drv = pdata->drv;
	struct resource *drv_res = pdata->drv_res;

	if (pm_device_down) {

		drv->base =
		    ioremap(drv_res->start, drv_res->end - drv_res->start + 1);
		if (!drv->base) {
			dev_err(dev, "ioremap_nocache() failed\n");
			return -EINVAL;
		}
		printk(KERN_INFO "f_netsec_ts: io 0x%x-0x%x\n", drv_res->start,
		       drv_res->end);

		if (drv->rst)
			reset_control_deassert(drv->rst);

		ret = clk_prepare_enable(drv->netauclk);
		if (ret < 0) {
			dev_err(dev, "clk_prepare_enable() failed\n");
		}

		sema_init(&drv->txdones, 1);
		init_waitqueue_head(&drv->txdoneq);

		spin_lock_init(&drv->tx_done_lock);
		INIT_WORK(&drv->lpbk_rx_que_work, netsec_irq_rtp_lpb_rx_work);
		sema_init(&drv->lpbk_rx_que_sem, 1);

		spin_lock_init(&drv->sessdb_lock);
		spin_lock_init(&drv->init_spinlock);		
	
		dev_set_drvdata(dev, drv);

		enable_irq(drv->irqa);
		
	}
#endif				/* CONFIG_PM_WARP */

	return 0;
}
static const struct dev_pm_ops ogma_pm_ops = {
	.suspend = ogma_suspend,
	.resume = ogma_resume,
	.freeze = ogma_suspend,
	.thaw = ogma_resume,
	.restore = ogma_resume,
};

#define OGMA_PM_OPS (&ogma_pm_ops)
#endif				/* CONFIG_PM_SLEEP */

static const struct of_device_id ogma_dt_ids[] = {
	{.compatible = "socionext,ogma"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ogma_dt_ids);

static struct platform_driver ogma_driver = {
	.probe = netsec_driver_probe,
	.remove = netsec_driver_remove,
	.driver = {
		.name = "ogma",
		.of_match_table = ogma_dt_ids,
#ifdef CONFIG_PM_SLEEP
		.pm = OGMA_PM_OPS,
#endif
   },
};

module_platform_driver(ogma_driver);

MODULE_AUTHOR("SOCIONEXT INCORPORATED");
MODULE_DESCRIPTION("NETSEC device driver");
MODULE_LICENSE("GPL");

MODULE_ALIAS("platform:ogma");
