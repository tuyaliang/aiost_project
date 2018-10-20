/*
 * linux/drivers/i2c/busses/i2c-snisc2000a.c
 *
 * Copyright (C) 2016 SOCIONEXT INC
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


/* Address map */
#define F_I2C_REG_SDAT		0x00	/* Serial Data register       */
#define F_I2C_REG_ST		0x04	/* Status register            */
#define F_I2C_REG_CST		0x08	/* Control Status register    */
#define F_I2C_REG_CTL1		0x0c	/* Control 1 Register         */
#define F_I2C_REG_ADDR		0x10	/* Own Address Register       */
#define F_I2C_REG_CTL2		0x14	/* Control 2 Register         */
#define F_I2C_REG_TOPR		0x18	/* TimeOut Prescaler Register */
#define F_I2C_REG_CTL3		0x1c	/* Control 3 Register         */

/* I2C#_SDAT (Serial Data register)       */
#define F_I2C_SDAT_DATA		0
#define F_I2C_SDAT_DATA_MASK	0x7f
/* I2C#_ST (Status register)              */
#define F_I2C_ST_MODE		0
#define F_I2C_ST_MODE_MASK	0x1f
#define F_I2C_ST_MODE_BIT	BIT(0)
#define F_I2C_ST_INT		7
#define F_I2C_ST_INT_MASK	0x01
#define F_I2C_ST_INT_BIT	BIT(7)
/* I2C#_CST (Control Status register)     */
#define F_I2C_CST_BB		0
#define F_I2C_CST_BB_MASK	0x01
#define F_I2C_CST_BB_BIT	BIT(0)
#define F_I2C_CST_TOCDIV	1
#define F_I2C_CST_TOCDIV_MASK	0x03
#define F_I2C_CST_TOCDIV_BIT	BIT(1)
#define F_I2C_CST_TERR		3
#define F_I2C_CST_TERR_MASK	0x01
#define F_I2C_CST_TERR_BIT	BIT(3)
#define F_I2C_CST_TSDA		4
#define F_I2C_CST_TSDA_MASK	0x01
#define F_I2C_CST_TSDA_BIT	BIT(4)
#define F_I2C_CST_TGSCL		5
#define F_I2C_CST_TGSCL_MASK	0x01
#define F_I2C_CST_TGSCL_BIT	BIT(5)
#define F_I2C_CST_PECNEXT	6
#define F_I2C_CST_PECNEXT_MASK	0x01
#define F_I2C_CST_PECNEXT_BIT	BIT(6)
#define F_I2C_CST_PECFAULT	7
#define F_I2C_CST_PECFAULT_MASK	0x01
#define F_I2C_CST_PECFAULT_BIT	BIT(7)
/* I2C#_CTL1 (Control 1 register)         */
#define F_I2C_CTL1_START	0
#define F_I2C_CTL1_START_MASK	0x01
#define F_I2C_CTL1_START_BIT	BIT(0)
#define F_I2C_CTL1_STOP		1
#define F_I2C_CTL1_STOP_MASK	0x01
#define F_I2C_CTL1_STOP_BIT	BIT(1)
#define F_I2C_CTL1_INTEN	2
#define F_I2C_CTL1_INTEN_MASK	0x01
#define F_I2C_CTL1_INTEN_BIT	BIT(2)
#define F_I2C_CTL1_ACK		4
#define F_I2C_CTL1_ACK_MASK	0x01
#define F_I2C_CTL1_ACK_BIT	BIT(4)
#define F_I2C_CTL1_GCMEN	5
#define F_I2C_CTL1_GCMEN_MASK	0x01
#define F_I2C_CTL1_GCMEN_BIT	BIT(5)
#define F_I2C_CTL1_SMBARE	6
#define F_I2C_CTL1_SMBARE_MASK	0x01
#define F_I2C_CTL1_SMBARE_BIT	BIT(6)
#define F_I2C_CTL1_CLRST	7
#define F_I2C_CTL1_CLRST_MASK	0x01
#define F_I2C_CTL1_CLRST_BIT	BIT(7)
/* I2C#_ADDR (Own Address register)       */
#define F_I2C_ADDR_ADDR		0
#define F_I2C_ADDR_ADDR_MASK	0x7f
#define F_I2C_ADDR_SAEN		7
#define F_I2C_ADDR_SAEN_MASK	0x01
#define F_I2C_ADDR_SAEN_BIT	BIT(7)
/* I2C#_CTL2 (Control 2 register)         */
#define F_I2C_CTL2_ENABLE	0
#define F_I2C_CTL2_ENABLE_MASK	0x01
#define F_I2C_CTL2_ENABLE_BIT	BIT(0)
#define F_I2C_CTL2_SCLFRQ	1
#define F_I2C_CTL2_SCLFRQ_MASK	0x7f
/* I2C#_TOPR (TimeOut Prescaler register) */
#define F_I2C_TOPR_TOPR		0
#define F_I2C_TOPR_TOPR_MASK	0xff
/* I2C#_CTL3 (Control 3 register)         */
#define F_I2C_CTL3_S10ADR	0
#define F_I2C_CTL3_S10ADR_MASK	0x07
#define F_I2C_CTL3_S10ADR_BIT	BIT(0)
#define F_I2C_CTL3_S10EN	3
#define F_I2C_CTL3_S10EN_MASK	0x01
#define F_I2C_CTL3_S10EN_BIT	BIT(3)
#define F_I2C_CTL3_HSCDIV	7
#define F_I2C_CTL3_HSCDIV_MASK	0x0f
#define F_I2C_CTL3_HSCDIV_BIT	BIT(7)

#define F_I2C_ADRS_MASK		(0x7f)

#define F_I2C_SCLFRQ_MAX	(127)	/* SCLFRQ MAX         */
#define F_I2C_SCLFRQ_100K_50MHZ	(125)	/* 100Kbps PCLK=50MHz */
#define F_I2C_SCLFRQ_100K_25MHZ	(63)	/* 100Kbps PCLK=25MHz */
#define F_I2C_SCLFRQ_400K_50MHZ	(32)	/* 400Kbps PCLK=50MHz */
#define F_I2C_SCLFRQ_400K_25MHZ	(16)	/* 400Kbps PCLK=25MHz */
#define F_I2C_SCLFRQ_MIN	(4)	/* SCLFRQ MIN         */
#define F_I2C_PCLK50MHZ		(50000000)
#define F_I2C_PCLK25MHZ		(25000000)
#define F_I2C_BPS100K		(100000)
#define F_I2C_BPS400K		(400000)

enum sc2a_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_RESTART,
	STATE_READ,
	STATE_WRITE
};

/* _ST MODE */
enum sc2a_i2c_st_mode {
/* No mode Information Available */
	ST_MODE_IDLE	= 0x00,
/* Start condition generated */
	ST_MODE_STDONE  = 0x01,
/* Repeated start condition generated */
	ST_MODE_RSDONE	= 0x02,
/* Arbitrationlost, unaddressed slave mode entered */
	ST_MODE_IDLARL	= 0x03,
/* Slave address sent, positive ACK */
	ST_MODE_MTADPA	= 0x04,
/* Slave address sent, negative ACK */
	ST_MODE_MTADNA	= 0x05,
/* Data byte sent, positive ACK */
	ST_MODE_MTDAPA	= 0x06,
/* Data byte sent, negative ACK */
	ST_MODE_MTDANA	= 0x07,
/* Slave address sent, positive ACK */
	ST_MODE_MRADPA	= 0x08,
/* Slave address sent, negative ACK */
	ST_MODE_MRADNA	= 0x09,
/* Data byte received, positive ACK */
	ST_MODE_MRDAPA	= 0x0A,
/* Data byte received, negative ACK */
	ST_MODE_MRDANA	= 0x0B,
/* Master code transmitted error detected */
	ST_MODE_MTMCER	= 0x0C,

/* Slave address received, positive ACK */
	ST_MODE_SRADPA	= 0x10,
/* Slave address received after arbitration loss, positive ACK */
	ST_MODE_SRAAPA	= 0x11,
/* Data byte received, positive ACK */
	ST_MODE_SRDAPA	= 0x12,
/* Data byte received, negative ACK */
	ST_MODE_SRDANA	= 0x13,
/* Slave address received, positive ACK */
	ST_MODE_STADPA	= 0x14,
/* Slave address received after arbitration loss, positive ACK */
	ST_MODE_STAAPA	= 0x15,
/* Data byte received, positive ACK */
	ST_MODE_STDAPA	= 0x16,
/* Data byte received, negative ACK */
	ST_MODE_STDANA	= 0x17,
/* Alart response address received, positive ACK */
	ST_MODE_SATADP	= 0x18,
/* Alart response address received after arbitration loss, positive ACK */
	ST_MODE_SATAAP	= 0x19,
/* Addressed With Alart response address, data byte send, positive ACK */
	ST_MODE_SATDAP	= 0x1A,
/* Addressed With Alart response address, data byte send, negative ACK */
	ST_MODE_SATDAN	= 0x1B,
/* Slave mode stop condition detected */
	ST_MODE_SSTOP	= 0x1C,
/* Slave address received after arbitration loss, positive ACK */
	ST_MODE_SGADPA	= 0x1D,
/* Slave address received after arbitration loss, positive ACK */
	ST_MODE_SGAAPA	= 0x1E,
/* Invalid start or stop condition detected */
	ST_MODE_BERROR	= 0x1F,

/* Master code transmitted OK – switched to Hs mode */
	ST_MODE_MTMCOK	= 0x21,
/* Repeated start condition generated */
	ST_MODE_HRSDONE	= 0x22,
/* Arbitration lost, high-speed unaddressed slave mode enterd */
	ST_MODE_HIDLARL	= 0x23,
/* Slave address sent, positive ACK */
	ST_MODE_HMTADPA	= 0x24,
/* Slave address sent, negative ACK */
	ST_MODE_HMTADNA	= 0x25,
/* Data byte sent, positive ACK */
	ST_MODE_HMTDAPA	= 0x26,
/* Data byte sent, negative ACK */
	ST_MODE_HMTDANA	= 0x27,
/* Slave address sent, positive ACK */
	ST_MODE_HMRADPA	= 0x28,
/* Slave address sent, negative ACK */
	ST_MODE_HMRADNA	= 0x29,
/* Data byte received, positive ACK */
	ST_MODE_HMRDAPA	= 0x2A,
/* Data byte received, negative ACK */
	ST_MODE_HMRDANA	= 0x2B,
/* Slave address received, positive ACK */
	ST_MODE_HSRADPA	= 0x30,
/* Data byte received, positive ACK */
	ST_MODE_HSRDAPA	= 0x32,
/* Data byte received, negative ACK */
	ST_MODE_HSRDANA	= 0x33,
/* Slave address received, positive ACK */
	ST_MODE_HSTADTA	= 0x34,
/* Data byte send, positive ACK */
	ST_MODE_HSTDAPA	= 0x36,
/* Data byte send, negative ACK */
	ST_MODE_HSTDANA	= 0x37,
};

struct sni_i2c {
	struct completion completion;

	struct i2c_msg *msg;
	unsigned int count_messages;
	unsigned int curr_msg_idx;
	unsigned int pos_inside_curr_msg;
	unsigned char rwflg;
	unsigned char slv_address;

	struct device *dev;
	void __iomem *base;
	unsigned int irq;
	struct clk *clk[4];
	int clocks;
	unsigned long clkrate;
	unsigned int speed;
	unsigned long timeout;
	enum sc2a_i2c_state state;
	struct i2c_adapter adapter;

	unsigned int readcnt;

	bool is_suspended;
};


static void delay_i2c_clocks(struct sni_i2c *i2c, int count)
{
	u32 delay = (((1000000000 + i2c->clkrate - 1) /
			i2c->clkrate + count - 1) / count) + 10;
	ndelay(delay);
}

static inline unsigned long calc_timeout_ms(struct sni_i2c *i2c,
					struct i2c_msg *msgs, int num)
{
	unsigned long bit_count = 0;
	int i;

	for (i = 0; i < num; i++, msgs++)
		bit_count += msgs->len;

	return DIV_ROUND_UP(((bit_count * 9) + (10 * num)) * 3, 200) + 10;
}

static void set_i2c_register(struct sni_i2c *i2c,
				int reg, int bit, int mask, int val)
{
	unsigned char setval;

	setval = (val & mask) << bit;
	writeb(readb(i2c->base + reg) | setval, i2c->base + reg);
}

static void clear_i2c_register(struct sni_i2c *i2c,
				int reg, int bit, int mask)
{
	unsigned char setval;

	setval = (0 & mask) << bit;
	writeb(readb(i2c->base + reg) | setval, i2c->base + reg);
}

static bool set_sdat_data(struct sni_i2c *i2c, unsigned char data)
{
	unsigned char st;

	st = readb(i2c->base + F_I2C_REG_ST);
	if (st & F_I2C_ST_INT_BIT) {
		clear_i2c_register(i2c, F_I2C_REG_SDAT,
					F_I2C_SDAT_DATA,
					F_I2C_SDAT_DATA_MASK);
		writeb(data, i2c->base + F_I2C_REG_SDAT);
	} else {
		dev_dbg(i2c->dev, "%s ST.INT not set\n", __func__);
		return false;
	}

	return true;
}

static bool set_slave_address(struct sni_i2c *i2c)
{
	unsigned char byte;

	/* set slave address */
	switch (i2c->state) {
	case STATE_READ:
		byte = (((i2c->slv_address << 1) | 1) &
					F_I2C_ADRS_MASK);
		break;

	case STATE_WRITE:
		byte = ((i2c->slv_address << 1) &
					F_I2C_ADRS_MASK);
		break;

	default:
		return false;
	}

	if (!set_sdat_data(i2c, byte))
		return false;

	return true;
}

static unsigned char get_st_mode(struct sni_i2c *i2c)
{
	unsigned char st, mode;

	st = readb(i2c->base + F_I2C_REG_ST);   /* Status register */
	mode = st & F_I2C_ST_MODE_MASK;
	return mode;
}

static void register_dump(struct sni_i2c *i2c, const char *func)
{
	dev_dbg(i2c->dev, "%s ------\n", func);
	dev_dbg(i2c->dev, "SDAT:0x%02x\n", readb(i2c->base + F_I2C_REG_SDAT));
	dev_dbg(i2c->dev, "ST  :0x%02x\n", readb(i2c->base + F_I2C_REG_ST));
	dev_dbg(i2c->dev, "CST :0x%02x\n", readb(i2c->base + F_I2C_REG_CST));
	dev_dbg(i2c->dev, "CTL1:0x%02x\n", readb(i2c->base + F_I2C_REG_CTL1));
	dev_dbg(i2c->dev, "ADDR:0x%02x\n", readb(i2c->base + F_I2C_REG_ADDR));
	dev_dbg(i2c->dev, "CTL2:0x%02x\n", readb(i2c->base + F_I2C_REG_CTL2));
	dev_dbg(i2c->dev, "TOPR:0x%02x\n", readb(i2c->base + F_I2C_REG_TOPR));
	dev_dbg(i2c->dev, "CTL3:0x%02x\n", readb(i2c->base + F_I2C_REG_CTL3));
}

static u32 select_scl_frequency(struct sni_i2c *i2c)
{
	u32 sclfrq;

	if (i2c->clkrate == F_I2C_PCLK50MHZ) {
		if (i2c->speed == F_I2C_BPS100K)
			sclfrq = F_I2C_SCLFRQ_100K_50MHZ;
		else if (i2c->speed == F_I2C_BPS400K)
			sclfrq = F_I2C_SCLFRQ_400K_50MHZ;
		else
			sclfrq = F_I2C_SCLFRQ_MAX;
	} else if (i2c->clkrate == F_I2C_PCLK25MHZ) {
		if (i2c->speed == F_I2C_BPS100K)
			sclfrq = F_I2C_SCLFRQ_100K_25MHZ;
		else if (i2c->speed == F_I2C_BPS400K)
			sclfrq = F_I2C_SCLFRQ_400K_25MHZ;
		else
			sclfrq = F_I2C_SCLFRQ_MAX;
	} else {
		sclfrq = F_I2C_SCLFRQ_MAX;
	}

	return sclfrq;
}

static int sc2a_i2c_init_hardware(struct sni_i2c *i2c)
{

	/* Clear ST.INT */
	set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);

	/* Enable the I2C */
	writeb(0, i2c->base + F_I2C_REG_CTL2);
	set_i2c_register(i2c, F_I2C_REG_CTL2, F_I2C_CTL2_ENABLE,
						F_I2C_CTL2_ENABLE_MASK, 1);
	/* SCLFRQ divisor setting */
	set_i2c_register(i2c, F_I2C_REG_CTL2, F_I2C_CTL2_SCLFRQ,
			F_I2C_CTL2_SCLFRQ_MASK, select_scl_frequency(i2c));

	/* Slave Address Enable */
	clear_i2c_register(i2c, F_I2C_REG_ADDR, F_I2C_ADDR_SAEN,
						F_I2C_ADDR_SAEN_MASK);

	/* Global Call Match Enable */
	clear_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_GCMEN,
						F_I2C_CTL1_GCMEN_MASK);
	/* Alert Response Match Enable */
	clear_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_SMBARE,
						F_I2C_CTL1_SMBARE_MASK);

	/* PEC Next */
	clear_i2c_register(i2c, F_I2C_REG_CST, F_I2C_CST_PECNEXT,
						F_I2C_CST_PECNEXT_MASK);
	/* Timeout Error */
	clear_i2c_register(i2c, F_I2C_REG_CST, F_I2C_CST_TERR,
						F_I2C_CST_TERR_MASK);
	/* Timeout Divider */
	clear_i2c_register(i2c, F_I2C_REG_CST, F_I2C_CST_TOCDIV,
						F_I2C_ADDR_SAEN_MASK);

	/*  SCL timeout prescaler */
	clear_i2c_register(i2c, F_I2C_REG_TOPR, F_I2C_TOPR_TOPR,
						F_I2C_TOPR_TOPR_MASK);

	i2c->state = STATE_IDLE;
	i2c->readcnt = 0;

	return 0;
}

static void sc2a_i2c_stop(struct sni_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "%s STOP\n", __func__);

	/* status bit INT can be cleared */
	set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
	/* interrupts are disabled */
	clear_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_INTEN,
						F_I2C_CTL1_INTEN_MASK);
	/* generates a stop condition */
	set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_STOP,
						F_I2C_CTL1_STOP_MASK, 1);

	i2c->state = STATE_IDLE;
	i2c->pos_inside_curr_msg = 0;
	i2c->msg = NULL;
	i2c->curr_msg_idx++;
	i2c->count_messages = 0;
	if (ret)
		i2c->curr_msg_idx = ret;

	complete(&i2c->completion);
}

static int sc2a_i2c_master_start(struct sni_i2c *i2c, struct i2c_msg *pmsg)
{
	unsigned char cst;

	register_dump(i2c, __func__); /* debug print */

	/* Generate Start Condition */
	cst = readb(i2c->base + F_I2C_REG_CST); /* Control Status register */
	if ((cst & F_I2C_CST_BB_BIT) && (get_st_mode(i2c) == ST_MODE_IDLE)) {
		dev_info(i2c->dev, "%s bus is busy", __func__);
		return -EBUSY;
	}

	i2c->rwflg = pmsg->flags & I2C_M_RD;
	i2c->slv_address = pmsg->addr;

	/* Start Condition + Enable Interrupts */
	dev_dbg(i2c->dev, "%s Generate Start Condition", __func__);
	set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_START,
						F_I2C_CTL1_START_MASK, 1);
	wmb();

	if (i2c->state == STATE_RESTART) {
		/* Clear ST.INT */
		set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
		wmb();
	}

	delay_i2c_clocks(i2c, 10);
	set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_INTEN,
						F_I2C_CTL1_INTEN_MASK, 1);

	return 0;
}

static int sc2a_i2c_doxfer(struct sni_i2c *i2c,
				struct i2c_msg *msgs, int num)
{
	unsigned char cst;
	unsigned long timeout, bb_timout;
	int ret = 0;

	if (i2c->is_suspended)
		return -EBUSY;

	sc2a_i2c_init_hardware(i2c);

	/* Bus Busy check */
	cst = readb(i2c->base + F_I2C_REG_CST);
	if (cst & F_I2C_CST_BB_BIT) {
		dev_err(i2c->dev, "%s cannot get bus (bus busy)\n",
							__func__);
		return -EBUSY;
	}

	init_completion(&i2c->completion);

	i2c->msg = msgs;
	i2c->count_messages = num;
	i2c->pos_inside_curr_msg = 0;
	i2c->curr_msg_idx = 0;
	i2c->state = STATE_START;

	ret = sc2a_i2c_master_start(i2c, i2c->msg);
	if (ret < 0) {
		dev_info(i2c->dev, "%s Address failed: (0x%08x)\n",
						__func__, ret);
		goto out;
	}

	timeout = wait_for_completion_timeout(&i2c->completion,
					msecs_to_jiffies(i2c->timeout));
	if (timeout <= 0) {
		dev_info(i2c->dev, "%s timeout\n", __func__);
		ret = -EAGAIN;
		goto out;
	}

	ret = i2c->curr_msg_idx;
	if (ret != num) {
		dev_info(i2c->dev, "%s incomplete xfer (%d)\n", __func__, ret);
		goto out;
	}

	/* ensure the stop has been through the bus */
	bb_timout = jiffies + HZ / 1000;
	do {
		cst = readb(i2c->base + F_I2C_REG_CST);
	} while ((cst & F_I2C_CST_BB_BIT) && time_before(jiffies, bb_timout));
out:
	return ret;
}

static irqreturn_t sc2a_i2c_isr(int irq, void *dev_id)
{
	struct sni_i2c *i2c = dev_id;
	unsigned char byte;
	unsigned char cst, ctl1;
	int ret = 0;

	/* Time out check */
	cst = readb(i2c->base + F_I2C_REG_CST);
	if (cst & F_I2C_CST_TERR_BIT) {
		dev_err(i2c->dev, "%s time out error\n", __func__);
		sc2a_i2c_stop(i2c, -ETIMEDOUT);
		return -ETIMEDOUT;
	}

	register_dump(i2c, __func__); /* debug print */

	ctl1 = readb(i2c->base + F_I2C_REG_CTL1);
	switch (get_st_mode(i2c)) {
	/* master */
	case ST_MODE_STDONE:
	case ST_MODE_RSDONE:
		dev_dbg(i2c->dev, "%s Start condition generated(%d)\n",
						__func__, get_st_mode(i2c));
		if (i2c->rwflg)		/* read mode */
			i2c->state = STATE_READ;
		else			/* write mode*/
			i2c->state = STATE_WRITE;

		/* Slave address set */
		if (!set_slave_address(i2c))
			sc2a_i2c_stop(i2c, -EACCES);

		/* Clear ST.INT */
		set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
		wmb();
		break;

	case ST_MODE_IDLARL:
		dev_dbg(i2c->dev, "%s Arbitration lost\n", __func__);
		sc2a_i2c_stop(i2c, -ECOMM);
		break;

	/*  master transmit */
	case ST_MODE_MTADPA:
		dev_dbg(i2c->dev, "%s address ACK received(WRITE)\n", __func__);
		byte = i2c->msg->buf[i2c->pos_inside_curr_msg++];
		if (set_sdat_data(i2c, byte)) {
			/* Clear ST.INT */
			set_i2c_register(i2c, F_I2C_REG_CTL1,
						F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
		} else {
			sc2a_i2c_stop(i2c, -EACCES);
		}

		break;

	case ST_MODE_MTADNA:
		dev_dbg(i2c->dev, "%s address NACK received(WRITE)\n",
							__func__);
		sc2a_i2c_stop(i2c, -ECOMM);
		break;

	case ST_MODE_MTDAPA:
		dev_dbg(i2c->dev, "%s data ACK received(WRITE)\n", __func__);
		if (i2c->pos_inside_curr_msg < i2c->msg->len) {
			byte = i2c->msg->buf[i2c->pos_inside_curr_msg++];
			if (set_sdat_data(i2c, byte)) {
				/* Clear ST.INT */
				set_i2c_register(i2c, F_I2C_REG_CTL1,
						F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
				wmb();
			} else {
				sc2a_i2c_stop(i2c, -EACCES);
			}
			break;
		}
		if (i2c->curr_msg_idx >= (i2c->count_messages - 1)) {
			sc2a_i2c_stop(i2c, 0);
			break;
		}

		/* clear ST.INTEN */
		clear_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_INTEN,
						F_I2C_CTL1_INTEN_MASK);
		wmb();

		/* next message new start */
		dev_dbg(i2c->dev, "%s Next start condition generated\n",
							__func__);
		i2c->pos_inside_curr_msg = 0;
		i2c->curr_msg_idx++;
		i2c->msg++;
		i2c->state = STATE_RESTART;
		ret = sc2a_i2c_master_start(i2c, i2c->msg);
		if (ret < 0) {
			dev_info(i2c->dev, "%s Address failed: (0x%08x)\n",
							__func__, ret);
			sc2a_i2c_stop(i2c, -EACCES);
		}

		break;

	case ST_MODE_MTDANA:
		dev_dbg(i2c->dev, "%s data NACK received(WRITE)\n", __func__);
		sc2a_i2c_stop(i2c, -ECOMM);
		break;

	/*  master receive */
	case ST_MODE_MRADPA:
		dev_dbg(i2c->dev, "%s address ACK received(READ)\n", __func__);
		/* Slave address set */
		if (!set_slave_address(i2c)) {
			sc2a_i2c_stop(i2c, -EACCES);
			break;
		}
		if (i2c->msg->len == 1)
			/* next is NACK */
			set_i2c_register(i2c, F_I2C_REG_CTL1,
						F_I2C_CTL1_ACK,
						F_I2C_CTL1_ACK_MASK, 1);

		/* Clear ST.INT */
		set_i2c_register(i2c, F_I2C_REG_CTL1, F_I2C_CTL1_CLRST,
						F_I2C_CTL1_CLRST_MASK, 1);
		wmb();
		break;

	case ST_MODE_MRADNA:
		dev_dbg(i2c->dev, "%s address NACK received(READ)\n", __func__);
		sc2a_i2c_stop(i2c, -ECOMM);
		break;

	case ST_MODE_MRDAPA:
		dev_dbg(i2c->dev, "%s data ACK received(READ)\n", __func__);
		byte = readb(i2c->base + F_I2C_REG_SDAT);
		i2c->msg->buf[i2c->pos_inside_curr_msg++] = byte;
		if ((i2c->pos_inside_curr_msg + 1) == i2c->msg->len)
			/* next is NACK */
			set_i2c_register(i2c, F_I2C_REG_CTL1,
						F_I2C_CTL1_ACK,
						F_I2C_CTL1_ACK_MASK, 1);

		else if (i2c->pos_inside_curr_msg < i2c->msg->len)
			/* next is ACK */
			clear_i2c_register(i2c, F_I2C_REG_CTL1,
						F_I2C_CTL1_ACK,
						F_I2C_CTL1_ACK_MASK);

		/* Clear ST.INT */
		set_i2c_register(i2c, F_I2C_REG_CTL1,
					F_I2C_CTL1_CLRST,
					F_I2C_CTL1_CLRST_MASK, 1);
		wmb();
		break;

	case ST_MODE_MRDANA:
		dev_dbg(i2c->dev, "%s data NACK received(READ)\n", __func__);
		byte = readb(i2c->base + F_I2C_REG_SDAT);
		i2c->msg->buf[i2c->pos_inside_curr_msg++] = byte;

		/* ACK */
		clear_i2c_register(i2c, F_I2C_REG_CTL1,
					F_I2C_CTL1_ACK,
					F_I2C_CTL1_ACK_MASK);
		/* Clear ST.INT */
		set_i2c_register(i2c, F_I2C_REG_CTL1,
					F_I2C_CTL1_CLRST,
					F_I2C_CTL1_CLRST_MASK, 1);

		wmb();
		sc2a_i2c_stop(i2c, i2c->pos_inside_curr_msg);
		i2c->pos_inside_curr_msg = 0;
		i2c->state = STATE_IDLE;
		break;

	default:
		dev_err(i2c->dev, "%s: I2C Operating Modes(0x%02x)\n",
					__func__, get_st_mode(i2c));
		sc2a_i2c_stop(i2c, -ENOSYS);
		break;
	}

	register_dump(i2c, __func__); /* debug print */

	delay_i2c_clocks(i2c, 10);

	return IRQ_HANDLED;
}

static int sc2a_i2c_xfer(struct i2c_adapter *adap,
				struct i2c_msg *msgs, int num)
{
	int retry = adap->retries;
	struct sni_i2c *i2c;
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
		ret = sc2a_i2c_doxfer(i2c, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		dev_dbg(i2c->dev, "%s Retrying transmission (%d) ret=%d\n",
							__func__, retry, ret);
		delay_i2c_clocks(i2c, 100);
	} while (--retry);

	dev_info(i2c->dev, "%s addr 0x%x: transmission err\n",
					__func__, msgs->addr);
	return -EIO;
}

static u32 sc2a_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}


static const struct i2c_algorithm sc2a_i2c_algo = {
	.master_xfer   = sc2a_i2c_xfer,
	.functionality = sc2a_i2c_functionality,
};

static struct i2c_adapter sc2a_i2c_ops = {
	.owner		= THIS_MODULE,
	.name		= "sni-i2c-adapter",
	.algo		= &sc2a_i2c_algo,
	.retries	= 1,
};

static const struct of_device_id sc2a_i2c_dt_ids[] = {
	{ .compatible = "socionext,sni-sc2000a-i2c"},
	{ /* sentinel */ }
};

static int sc2a_i2c_probe(struct platform_device *pdev)
{
	struct resource *mem;
	struct sni_i2c *i2c;
	int ret = 0;

	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (of_property_read_u32(pdev->dev.of_node,
				"clock-frequency",
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
	dev_dbg(&pdev->dev, "clock rate %ld\n", i2c->clkrate);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->base = devm_ioremap_resource(&pdev->dev, mem);
	if (!i2c->base) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "%s cannot remap\n", __func__);
		goto err_clk;
	}

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "%s cannot find IRQ\n", __func__);
		goto err_iomap;
	}

	ret = devm_request_irq(&pdev->dev, i2c->irq,
				sc2a_i2c_isr, IRQF_NO_SUSPEND,
				"sni-i2c", i2c);
	if (ret) {
		dev_err(&pdev->dev, "%s cannot claim IRQ %d\n",
						__func__, i2c->irq);
		goto err_iomap;
	}

	i2c->dev = &pdev->dev;
	ret = sc2a_i2c_init_hardware(i2c);
	if (ret)
		goto err_irq;

	i2c->adapter = sc2a_i2c_ops;
	i2c_set_adapdata(&i2c->adapter, i2c);
	i2c->adapter.dev.parent = &pdev->dev;
	i2c->adapter.dev.of_node = of_node_get(pdev->dev.of_node);
	i2c->adapter.nr = pdev->id;

	ret = i2c_add_numbered_adapter(&i2c->adapter);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s failed to add bus to i2c core\n",
								__func__);
		goto err_irq;
	}

	platform_set_drvdata(pdev, i2c);
	dev_dbg(&pdev->dev, "%s: sni-i2c adapter\n",
				dev_name(&i2c->adapter.dev));

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

static int sc2a_i2c_remove(struct platform_device *pdev)
{
	struct sni_i2c *i2c = platform_get_drvdata(pdev);

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
static int sc2a_i2c_suspend(struct device *dev)
{
	struct sni_i2c *i2c = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c->adapter);
	i2c->is_suspended = true;
	disable_irq(i2c->irq);
	i2c_unlock_adapter(&i2c->adapter);
	return 0;
}

static int sc2a_i2c_resume(struct device *dev)
{
	struct sni_i2c *i2c = dev_get_drvdata(dev);
	int ret = 0;

	i2c_lock_adapter(&i2c->adapter);

	sc2a_i2c_init_hardware(i2c);
	enable_irq(i2c->irq);
	i2c->is_suspended = false;

	i2c_unlock_adapter(&i2c->adapter);

	return ret;
}

struct dev_pm_ops sc2a_i2c_pm = {
	.suspend_late = sc2a_i2c_suspend,
	.resume_early = sc2a_i2c_resume,
};
#define F_I2C_PM	(&sc2a_i2c_pm)
#else
#define F_I2C_PM	NULL
#endif

MODULE_DEVICE_TABLE(of, sc2a_i2c_dt_ids);

static struct platform_driver sc2a_i2c_driver = {
	.probe   = sc2a_i2c_probe,
	.remove  = sc2a_i2c_remove,
	.driver  = {
		.owner = THIS_MODULE,
		.name = "sni-i2c",
		.of_match_table = sc2a_i2c_dt_ids,
		.pm = F_I2C_PM,
	},
};

static int __init sc2a_i2c_init(void)
{
	return platform_driver_register(&sc2a_i2c_driver);
}

static void __exit sc2a_i2c_exit(void)
{
	platform_driver_unregister(&sc2a_i2c_driver);
}

module_init(sc2a_i2c_init);
module_exit(sc2a_i2c_exit);

MODULE_AUTHOR("Socionext Inc.");
MODULE_DESCRIPTION("Socionext I2C IP Driver");
MODULE_LICENSE("GPL");

