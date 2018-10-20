/*
 * linux/drivers/i2c/busses/i2c-sni2c.c
 *
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED
 * Copyright (C) 2013-2015 Linaro, Ltd
 *
 * Andy Green <andy.green@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>


/*
 * There are two Socionext IP variants for I2C.
 * The first type appeared in mb8ac0300 and mb86s7x.
 * The second type appeared in mb86s27.
 * The register layouts are different but many features are the same,
 * so the driver is combined.
 */
enum sni2c_variants {
	F_I2C_V1,
	F_I2C_V2,
};
/*
 * Variant "F_I2C_V1" layout
 */
#define F_I2C_REG_BSR		0x00	/* Bus Status Regster */
#define F_I2C_REG_BCR		0x04	/* Bus Control Register */
#define F_I2C_REG_CCR		0x08	/* Clock Control Register */
#define F_I2C_REG_ADR		0x0c	/* Address Register */
#define F_I2C_REG_DAR		0x10	/* Data Register */
#define F_I2C_REG_CSR		0x14	/* Expansion CS Register */
#define F_I2C_REG_FSR		0x18	/* Bus Clock Frequency Register */
#define F_I2C_REG_BC2R		0x1c	/* Bus Control 2 Register */

/* Variant "F_I2C_V2" layout */

#define F_I2C2_REG_IBCR		0
#define F_I2C2_REG_IBSR		4
#define F_I2C2_REG_ITBA		8
#define F_I2C2_REG_ITMK		0xc
#define F_I2C2_REG_ISMK		0x10
#define F_I2C2_REG_ISBA		0x14
#define F_I2C2_REG_IDAR		0x18
#define F_I2C2_REG_ICCR		0x1c

#define F_I2C_BSR_FBT		BIT(0)  /* First Byte Transfer */
#define F_I2C_BSR_LRB		BIT(4)  /* Last Received Bit */
#define F_I2C_BSR_AL		BIT(5)  /* Arbitration Lost */
#define F_I2C_BSR_BB		BIT(7)  /* Bus Busy */

#define F_I2C_BCR_INTE		BIT(1)  /* Interrupt Enable */
#define F_I2C_BCR_ACK		BIT(3)  /* Acknowledge */
#define F_I2C_BCR_MSS		BIT(4)  /* Master Slave Select */
#define F_I2C_BCR_SCC		BIT(5)  /* Start Condition Continue */
#define F_I2C_BCR_BEIE		BIT(6)  /* Bus Error Interrupt Enable */
#define F_I2C_BCR_BER		BIT(7)  /* Bus Error */

#define F_I2C_CCR_CS_MASK	(0x1f)  /* CCR Clock Period Select */
#define F_I2C_CCR_EN		BIT(5)  /* Enable */
#define F_I2C_CCR_FM		BIT(6)  /* Speed Mode Select */

#define F_I2C_CSR_CS_MASK	(0x3f)  /* CSR Clock Period Select */

#define F_I2C_BC2R_SCLL		BIT(0)  /* SCL Low Drive */
#define F_I2C_BC2R_SDAL		BIT(1)  /* SDA Low Drive */
#define F_I2C_BC2R_SCLS		BIT(4)  /* SCL Status */
#define F_I2C_BC2R_SDAS		BIT(5)  /* SDA Status */

enum sni2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE
};

struct sni2c {
	struct completion completion;

	struct i2c_msg *msg;
	unsigned int count_messages;
	unsigned int curr_msg_idx;
	unsigned int pos_inside_curr_msg;

	struct device *dev;
	void __iomem *base;
	unsigned int irq;
	struct clk *clk[4];
	int clocks;
	unsigned long clkrate;
	unsigned int speed;
	unsigned long timeout;
	enum sni2c_state state;
	struct i2c_adapter adapter;

	enum sni2c_variants variant;
	bool is_suspended;
};

/* The vairiants have some common registers at different offsets */

static const u8 dar_v[] = { F_I2C_REG_DAR, F_I2C2_REG_IDAR };
static const u8 bsr_v[] = { F_I2C_REG_BSR, F_I2C2_REG_IBSR };
static const u8 bcr_v[] = { F_I2C_REG_BCR, F_I2C2_REG_IBCR };
static const u8 ccr_v[] = { F_I2C_REG_CCR, F_I2C2_REG_ICCR };

static void delay_i2c_clocks(struct sni2c *i2c, int count)
{
        u32 delay = (((1000000000 + i2c->clkrate - 1) /
                i2c->clkrate + count - 1) / count) + 10;

        ndelay(delay);
}

static inline unsigned long calc_timeout_ms(struct sni2c *i2c,
					struct i2c_msg *msgs, int num)
{
	unsigned long bit_count = 0;
	int i;

	for (i = 0; i < num; i++, msgs++)
		bit_count += msgs->len;

	return DIV_ROUND_UP(((bit_count * 9) + (10 * num)) * 3, 200) + 10;
}

static void sni2c_stop(struct sni2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "STOP\n");

	/*
	 * clear IRQ (INT=0, BER=0)
	 * set Stop Condition (MSS=0)
	 * Interrupt Disable
	 */
	writeb(0, i2c->base + bcr_v[i2c->variant]);

	i2c->state = STATE_IDLE;

	i2c->pos_inside_curr_msg = 0;
	i2c->msg = NULL;
	i2c->curr_msg_idx++;
	i2c->count_messages = 0;
	if (ret)
		i2c->curr_msg_idx = ret;

	complete(&i2c->completion);
}

static int sni2c_init_hardware(struct sni2c *i2c)
{
        u32 ccr_cs = 0, div, base = 65;
        writeb(0, i2c->base + F_I2C_REG_ADR);
        writeb((readb(i2c->base + bcr_v[i2c->variant]) & ~F_I2C_BCR_BER) |
               F_I2C_BCR_MSS,
               i2c->base + bcr_v[i2c->variant]);

        if (i2c->variant != F_I2C_V1) {
                /* v2 hw has nonlinear divisor setting in 5 bits */
                div = (((i2c->clkrate / i2c->speed) - 4) / 12) - 1;
                if (div > F_I2C_CCR_CS_MASK)
                        return -EINVAL;

                /* clearing F_I2C_CCR_EN clears bus busy detection */
                writeb(div, i2c->base + F_I2C2_REG_ICCR);
                writeb(div | F_I2C_CCR_EN, i2c->base + F_I2C2_REG_ICCR);
                /* clear IRQ (INT=0, BER=0), Interrupt Disable */
		writeb(0, i2c->base + bcr_v[i2c->variant]);

                return 0;
        }

        /* v1 hw needs to know input clock rate in 20MHz steps */
        writeb((i2c->clkrate / 20000000) + 1,
               i2c->base + F_I2C_REG_FSR);

        /* and set a magic "fast mode" bit... */
        if (i2c->speed == 400000) {
                ccr_cs |= F_I2C_CCR_FM;
                /* ...that changes the divider characteristics */
                base = 1;
        }
        /*
	 * v1 hw has linear divisor split across CCR/CSR
	 * but specifies CSR part (b10..b5) to always be zero..
	 * and divisor start ratio differs according to "fast mode"
	 */
        div = (i2c->clkrate / i2c->speed);
        if (div < base || div > base + 31)
                return -EINVAL;

        /* clearing F_I2C_CCR_EN clears bus busy detection */
        writeb(ccr_cs | (div - base), i2c->base + F_I2C_REG_CCR);
        writeb(F_I2C_CCR_EN | ccr_cs | (div - base), i2c->base + F_I2C_REG_CCR);

        writeb(0, i2c->base + F_I2C_REG_CSR);

        /* make sure no bitbanging */
        writeb(0, i2c->base + F_I2C_REG_BC2R);

        return 0;
}

static int sni2c_master_start(struct sni2c *i2c, struct i2c_msg *pmsg)
{
	unsigned char bsr, bcr;
	int dar = F_I2C_REG_DAR;

	if (i2c->variant != F_I2C_V1)
		dar = F_I2C2_REG_IDAR;

	if (pmsg->flags & I2C_M_RD)
		writeb((pmsg->addr << 1) | 1, i2c->base + dar);
	else
		writeb(pmsg->addr << 1, i2c->base + dar);

	dev_dbg(i2c->dev, "%s slave:0x%02x\n", __func__, pmsg->addr);

	/* Generate Start Condition */
	bcr = readb(i2c->base + bcr_v[i2c->variant]);
	bsr = readb(i2c->base + bsr_v[i2c->variant]);
	dev_dbg(i2c->dev, "%s bsr:0x%02x, bcr:0x%02x\n", __func__, bsr, bcr);

	if ((bsr & F_I2C_BSR_BB) && !(bcr & F_I2C_BCR_MSS)) {
		dev_dbg(i2c->dev, "%s bus is busy", __func__);
		return -EBUSY;
	}

	if (bsr & F_I2C_BSR_BB) { /* Bus is busy */
		dev_dbg(i2c->dev, "%s Continuous Start", __func__);
		 writeb(bcr | F_I2C_BCR_SCC, i2c->base + bcr_v[i2c->variant]);
	} else {
		if (bcr & F_I2C_BCR_MSS) {
			dev_dbg(i2c->dev, "%s is not in master mode", __func__);
			return -EAGAIN;
		}
		dev_dbg(i2c->dev, "%s Start Condition", __func__);
		/* Start Condition + Enable Interrupts */
		writeb(bcr | F_I2C_BCR_MSS | F_I2C_BCR_INTE |
			F_I2C_BCR_BEIE, i2c->base + bcr_v[i2c->variant]);
	}
	delay_i2c_clocks(i2c, 10);

	bcr = readb(i2c->base + bcr_v[i2c->variant]);
	bsr = readb(i2c->base + bsr_v[i2c->variant]);
	dev_dbg(i2c->dev, "%s bsr:0x%02x, bcr:0x%02x\n", __func__, bsr, bcr);

	if ((bsr & F_I2C_BSR_AL) || !(bcr & F_I2C_BCR_MSS)) {
		dev_dbg(i2c->dev, "%s arbitration lost\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

static int sni2c_master_recover(struct sni2c *i2c)
{
	unsigned int count = 0;
	unsigned char bc2r;

	if (i2c->variant != F_I2C_V1) {
		/* disable the unit to drive (let go of the clock) */
		writel(readl(i2c->base + F_I2C2_REG_ICCR) & ~F_I2C_CCR_EN,
		       i2c->base + F_I2C2_REG_ICCR);
		/* wait... */
		delay_i2c_clocks(i2c, 10);
		/* reenable... */
		writel(readl(i2c->base + F_I2C2_REG_ICCR) | F_I2C_CCR_EN,
		       i2c->base + F_I2C2_REG_ICCR);
		/* wait... */
		delay_i2c_clocks(i2c, 30);
		/* return to restart... */
		return -EAGAIN;
	}

	writeb(0, i2c->base + bcr_v[i2c->variant]);
	/* monitor SDA, SCL */
	bc2r = readb(i2c->base + F_I2C_REG_BC2R);
	dev_dbg(i2c->dev, "%s bc2r:0x%02x\n", __func__, (unsigned)bc2r);

	while (!(bc2r & F_I2C_BC2R_SDAS) && (bc2r & F_I2C_BC2R_SCLS)) {
		delay_i2c_clocks(i2c, 10);
		bc2r = readb(i2c->base + F_I2C_REG_BC2R);

		/* another master is running */
		if (++count >= 100) {
			dev_dbg(i2c->dev, "%s: another master is running?\n",
							__func__);
			return -EAGAIN;
		}
	}

	/* Force to make one clock pulse */
	count = 0;
	for (;;) {
		/* SCL = L->H */
		writeb(F_I2C_BC2R_SCLL, i2c->base + F_I2C_REG_BC2R);
		delay_i2c_clocks(i2c, 10);
		writeb(0, i2c->base + F_I2C_REG_BC2R);
		delay_i2c_clocks(i2c, 5);

		bc2r = readb(i2c->base + F_I2C_REG_BC2R);
		if (bc2r & F_I2C_BC2R_SDAS)
			break;
		if (++count > 9) {
			dev_err(i2c->dev, "%s: count: %i, bc2r: 0x%x\n",
						__func__, count, bc2r);
			return -EIO;
		}
	}

	/* force to make bus-error phase */
	writeb(F_I2C_BC2R_SDAL, i2c->base + F_I2C_REG_BC2R);
	delay_i2c_clocks(i2c, 1);
	writeb(0, i2c->base + F_I2C_REG_BC2R);

	/* Both SDA & SDL should be H */
	bc2r = readb(i2c->base + F_I2C_REG_BC2R);
	if (!(bc2r & F_I2C_BC2R_SDAS) || !(bc2r & F_I2C_BC2R_SCLS)) {
		dev_err(i2c->dev, "%s: bc2r: 0x%x\n", __func__, bc2r);
		return -EIO;
	}

	return 0;
}

static int sni2c_doxfer(struct sni2c *i2c,
				struct i2c_msg *msgs, int num)
{
	unsigned char bsr;
	unsigned long timeout, bb_timout;
	int ret = 0;

	if (i2c->is_suspended)
		return -EBUSY;

	sni2c_init_hardware(i2c);
	bsr = readb(i2c->base + bsr_v[i2c->variant]);
	if (bsr & F_I2C_BSR_BB) {
		dev_err(i2c->dev, "cannot get bus (bus busy)\n");
		return -EBUSY;
	}

	init_completion(&i2c->completion);

	i2c->msg = msgs;
	i2c->count_messages = num;
	i2c->pos_inside_curr_msg = 0;
	i2c->curr_msg_idx = 0;
	i2c->state = STATE_START;

	ret = sni2c_master_start(i2c, i2c->msg);
	if (ret < 0) {
		dev_dbg(i2c->dev, "Address failed: (0x%08x)\n", ret);
		goto out;
	}

	timeout = wait_for_completion_timeout(&i2c->completion,
					      msecs_to_jiffies(i2c->timeout));
	if (timeout <= 0) {
		dev_dbg(i2c->dev, "timeout\n");
		ret = -EAGAIN;
		goto out;
	}

	ret = i2c->curr_msg_idx;
	if (ret != num) {
		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}

	/* ensure the stop has been through the bus */
	bb_timout = jiffies + HZ / 1000;
	do {
		bsr = readb(i2c->base + bsr_v[i2c->variant]);
	} while ((bsr & F_I2C_BSR_BB) && time_before(jiffies, bb_timout));
out:
	return ret;
}

static irqreturn_t sni2c_isr(int irq, void *dev_id)
{
	struct sni2c *i2c = dev_id;
	unsigned char byte;
	unsigned char bsr, bcr;
	int ret = 0;

	bcr = readb(i2c->base + bcr_v[i2c->variant]);
	bsr = readb(i2c->base + bsr_v[i2c->variant]);

	if (bcr & F_I2C_BCR_BER) {
		dev_err(i2c->dev, "%s: bus error\n", __func__);
		sni2c_stop(i2c, -EAGAIN);
		goto out;
	}
	if (bsr & F_I2C_BSR_AL || !(bcr & F_I2C_BCR_MSS)) {
		dev_err(i2c->dev, "%s arbitration lost\n", __func__);
		sni2c_stop(i2c, -EAGAIN);
		goto out;
	}

	switch (i2c->state) {
	case STATE_START:
		if (bsr & F_I2C_BSR_LRB) {
			dev_dbg(i2c->dev, "ack was not received\n");
			sni2c_stop(i2c, -EAGAIN);
			goto out;
		}

		if (i2c->msg->flags & I2C_M_RD)
			i2c->state = STATE_READ;
		else
			i2c->state = STATE_WRITE;

		if (i2c->curr_msg_idx >= (i2c->count_messages - 1) &&
		    i2c->msg->len == 0) {
			sni2c_stop(i2c, 0);
			goto out;
		}

		if (i2c->state == STATE_READ)
			goto prepare_read;

		/* fallthru */

	case STATE_WRITE:
		if (bsr & F_I2C_BSR_LRB) {
			dev_dbg(i2c->dev, "WRITE: No Ack\n");
			sni2c_stop(i2c, -EAGAIN);
			goto out;
		}

		if (i2c->pos_inside_curr_msg < i2c->msg->len) {
			writeb(i2c->msg->buf[i2c->pos_inside_curr_msg++],
			       i2c->base + dar_v[i2c->variant]);

			/* clear IRQ, and continue */
			writeb(F_I2C_BCR_BEIE | F_I2C_BCR_MSS | F_I2C_BCR_INTE,
			       i2c->base + bcr_v[i2c->variant]);
			break;
		}
		if (i2c->curr_msg_idx >= (i2c->count_messages - 1)) {
			sni2c_stop(i2c, 0);
			break;
		}
		dev_dbg(i2c->dev, "WRITE: Next Message\n");

		i2c->pos_inside_curr_msg = 0;
		i2c->curr_msg_idx++;
		i2c->msg++;

		/* send the new start */
		ret = sni2c_master_start(i2c, i2c->msg);
		if (ret < 0) {
			dev_dbg(i2c->dev, "restart err:0x%08x\n", ret);
			sni2c_stop(i2c, -EAGAIN);
			break;
		}
		i2c->state = STATE_START;
		break;

	case STATE_READ:
		if (!(bsr & F_I2C_BSR_FBT)) { /* data */
			byte = readb(i2c->base + dar_v[i2c->variant]);
			i2c->msg->buf[i2c->pos_inside_curr_msg++] = byte;
		}

prepare_read:
		if (i2c->pos_inside_curr_msg == (i2c->msg->len - 1)) {
			writeb(F_I2C_BCR_MSS | F_I2C_BCR_BEIE |
				F_I2C_BCR_INTE, i2c->base + bcr_v[i2c->variant]);
			break;
		}
		if (i2c->pos_inside_curr_msg < i2c->msg->len) {
			writeb(F_I2C_BCR_MSS | F_I2C_BCR_BEIE |
				F_I2C_BCR_INTE | F_I2C_BCR_ACK,
				i2c->base + bcr_v[i2c->variant]);
			break;
		}
		if (i2c->curr_msg_idx >= (i2c->count_messages - 1)) {
			/* last message, send stop and complete */
			dev_dbg(i2c->dev, "READ: Send Stop\n");
			sni2c_stop(i2c, 0);
			break;
		}
		dev_dbg(i2c->dev, "READ: Next Transfer\n");

		i2c->pos_inside_curr_msg = 0;
		i2c->curr_msg_idx++;
		i2c->msg++;

		ret = sni2c_master_start(i2c, i2c->msg);
		if (ret < 0) {
			dev_dbg(i2c->dev, "restart err: 0x%08x\n", ret);
			sni2c_stop(i2c, -EAGAIN);
		} else
			i2c->state = STATE_START;
		break;
	default:
		dev_err(i2c->dev, "%s: called in err STATE (%d)\n",
			 __func__, i2c->state);
		break;
	}

out:
	delay_i2c_clocks(i2c, 10);
	return IRQ_HANDLED;
}

static int sni2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int retry = adap->retries;
	struct sni2c *i2c;
	int bits = 0;
	int n = num;
	int ret;

	if (!msgs)
		return -EINVAL;
	if (num <= 0)
		return -EINVAL;

	i2c = i2c_get_adapdata(adap);

	while (n--)
		bits += msgs[n].len;

	i2c->timeout = DIV_ROUND_UP(((num * 10) + (bits * 9)) * 3, 200) + 100;

	do {
		ret = sni2c_doxfer(i2c, msgs, num);
		//if (ret != -EAGAIN)
		if (ret)
			return ret;

		dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

		sni2c_master_recover(i2c);
		switch (i2c->variant) {
		case F_I2C_V1:
			/* Disable clock */
			writeb(0, i2c->base + F_I2C_REG_CCR);
			writeb(0, i2c->base + F_I2C_REG_CSR);
			break;
		case F_I2C_V2:
			writel(0, i2c->base + F_I2C2_REG_ICCR);
			break;
		}
		delay_i2c_clocks(i2c, 100);
		sni2c_init_hardware(i2c);

	} while (--retry);

	dev_err(i2c->dev, "addr 0x%x: transmission err\n", msgs->addr);

	return -EIO;
}

static u32 sni2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sni2c_algo = {
	.master_xfer   = sni2c_xfer,
	.functionality = sni2c_functionality,
};

static struct i2c_adapter sni2c_ops = {
	.owner		= THIS_MODULE,
	.name		= "sni2c-adapter",
	.algo		= &sni2c_algo,
	.retries	= 5,
};

static const struct of_device_id sni2c_dt_ids[] = {
	{ .compatible = "socionext,sni2c",	.data = (void *)F_I2C_V1 },
	{ .compatible = "socionext,sni2c-v2",	.data = (void *)F_I2C_V2 },
	{ /* sentinel */ }
};

static int sni2c_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(sni2c_dt_ids, &pdev->dev);
	struct resource *r;
	struct sni2c *i2c;
	int ret = 0;

	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->variant = (enum sni2c_variants)of_id->data;
	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				 &i2c->speed))
		i2c->speed = 100000;

	do {
		i2c->clk[i2c->clocks] =
				of_clk_get(pdev->dev.of_node, i2c->clocks);
		if (IS_ERR(i2c->clk[i2c->clocks]))
			break;
		clk_prepare_enable(i2c->clk[i2c->clocks++]);
	} while (i2c->clocks < ARRAY_SIZE(i2c->clk));
	if (!i2c->clocks) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(i2c->clk);
		goto err_noclk;
	}

	i2c->clkrate = clk_get_rate(i2c->clk[0]);
	dev_info(&pdev->dev, "clock rate %ld\n", i2c->clkrate);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->base = ioremap(r->start, r->end - r->start + 1);
	if (!i2c->base) {
		ret = -ENOMEM;
		goto err_clk;
	}

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	ret = request_irq(i2c->irq, sni2c_isr, 0, "sni2c", i2c);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}

	i2c->dev = &pdev->dev;

	ret = sni2c_init_hardware(i2c);
	if (ret)
		goto err_irq;

	i2c->adapter = sni2c_ops;
	i2c_set_adapdata(&i2c->adapter, i2c);
	i2c->adapter.dev.parent = &pdev->dev;
	i2c->adapter.dev.of_node = of_node_get(pdev->dev.of_node);
	i2c->adapter.nr = pdev->id;

	ret = i2c_add_numbered_adapter(&i2c->adapter);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_irq;
	}

	platform_set_drvdata(pdev, i2c);
	dev_info(&pdev->dev, "%s: sni2c adapter v%d\n",
			     dev_name(&i2c->adapter.dev), i2c->variant + 1);

	return 0;

err_irq:
	of_node_put(pdev->dev.of_node);
	free_irq(i2c->irq, i2c);

err_iomap:
	iounmap(i2c->base);

err_clk:
	while (i2c->clocks) {
		clk_disable_unprepare(i2c->clk[i2c->clocks]);	
		clk_put(i2c->clk[i2c->clocks--]);
	}
err_noclk:
	kfree(i2c);

	return ret;
}

static int sni2c_remove(struct platform_device *pdev)
{
	struct sni2c *i2c = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c->adapter);
	while (i2c->clocks) {
		clk_disable_unprepare(i2c->clk[i2c->clocks]);
		clk_put(i2c->clk[i2c->clocks--]);
	}
	free_irq(i2c->irq, i2c);
	iounmap(i2c->base);
	kfree(i2c);
	of_node_put(pdev->dev.of_node);

	return 0;
};


#ifdef CONFIG_PM_SLEEP
static int sni2c_suspend(struct device *dev)
{
	struct sni2c *i2c = dev_get_drvdata(dev);
//	int n = i2c->clocks;

	i2c_lock_adapter(&i2c->adapter);
	i2c->is_suspended = true;
	disable_irq(i2c->irq);
	i2c_unlock_adapter(&i2c->adapter);
#if 0
	while (n--)
		clk_disable_unprepare(i2c->clk[n]);
#endif

	return 0;
}

static int sni2c_resume(struct device *dev)
{
	struct sni2c *i2c = dev_get_drvdata(dev);
	int ret = 0;
//	int n = 0;
	int bad = 0;

	i2c_lock_adapter(&i2c->adapter);
#if 0
	while (n < i2c->clocks) {
		ret = clk_prepare_enable(i2c->clk[n++]);
		if (ret) {
			dev_err(dev, "clock %d failed enable\n", n - 1);
			bad = ret;
		}
	}
#endif
#if 0
	{ void __iomem *p = ioremap(0x18001000, 0x200);
		pr_err("%s: %x %x\n", __func__, readl(p + 0x28), readl(p + 0x20));
		iounmap(p);
		mdelay(100);
	}
#endif
	if (!bad) {
		sni2c_init_hardware(i2c);
		enable_irq(i2c->irq);
		i2c->is_suspended = false;
	} else
		ret = bad;

	i2c_unlock_adapter(&i2c->adapter);

	return ret;
}

struct dev_pm_ops sni2c_pm = {
	.suspend_late = sni2c_suspend,
	.resume_early = sni2c_resume,
};
#define F_I2C_PM	(&sni2c_pm)
#else
#define F_I2C_PM	NULL
#endif

MODULE_DEVICE_TABLE(of, sni2c_dt_ids);

static struct platform_driver sni2c_driver = {
	.probe   = sni2c_probe,
	.remove  = sni2c_remove,
	.driver  = {
		.owner = THIS_MODULE,
		.name = "sni2c",
		.of_match_table = sni2c_dt_ids,
		.pm = F_I2C_PM,
	},
};

static int __init sni2c_init(void)
{
	return platform_driver_register(&sni2c_driver);
}

static void __exit sni2c_exit(void)
{
	platform_driver_unregister(&sni2c_driver);
}

module_init(sni2c_init);
module_exit(sni2c_exit);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("Socionext I2C IP Driver");
MODULE_LICENSE("GPL");

