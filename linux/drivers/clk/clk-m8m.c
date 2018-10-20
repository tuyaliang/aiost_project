/*
 * Copyright (C) 2015 Linaro Ltd.
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

#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#define CLKSEL_A	0x0
#define CLKSEL_B	0x4
#define CLKSEL_C	0x8
#define CLKSEL_D	0xc
#define CLKSEL_E	0x10
#define CLKSEL_F	0x14
#define CLKSEL_G	0x18
#define CLKSEL_H	0x1c

#define PLLCNT1		0x20
#define PLLCNT2		0x24

#define CLKSTOP_A	0x28
#define CLKSTOP_B	0x2c
#define CLKSTOP_C	0x30
#define CLKSTOP_D	0x34
#define CLKSTOP_E	0x38
#define CLKSTOP_F	0x3c

#define CRSWR		0x40
#define CRRRS		0x44
#define CRRSM		0x48

#define CLKSTOPS_A	0x128
#define CLKSTOPS_B	0x12c
#define CLKSTOPS_C	0x130
#define CLKSTOPS_D	0x134
#define CLKSTOPS_E	0x138
#define CLKSTOPS_F	0x13c

#define CLKSTOPC_A	0x228
#define CLKSTOPC_B	0x22c
#define CLKSTOPC_C	0x230
#define CLKSTOPC_D	0x234
#define CLKSTOPC_E	0x238
#define CLKSTOPC_F	0x23c

#define to_m8m_mux(_hw) 	container_of(_hw, struct m8m_mux, hw)
#define to_m8m_gate(_hw)	container_of(_hw, struct m8m_gate, hw)
#define to_m8m_gap(_hw)		container_of(_hw, struct m8m_gap, hw)
#define to_m8m_div(_hw) 	container_of(_hw, struct m8m_div, hw)
#define to_m8m_pll(_hw) 	container_of(_hw, struct m8m_pll, hw)

static void __iomem *clk_base;
static struct device_node *np_top;
static u32 blacklist_clksel[17], blacklist_clkstops[6], blacklist_clkstopc[6];

static void m8m_clk_writel(u32 val, void __iomem *p)
{
	u32 change = readl(p) ^ val;
	u32 ofs = p - clk_base;

	if (ofs < sizeof(blacklist_clksel))
		if (change & blacklist_clksel[ofs >> 2]) {
			printk("change=%08x,blacklist Index=%d\n",change,(ofs >> 2));
			WARN_ON(1);
		}
	if (ofs > 0x128 && ofs < 0x128 + sizeof(blacklist_clkstops))
		if (change & blacklist_clkstops[(ofs - 0x128) >> 2]) {
			printk("change=%08x,blacklist Index=%d\n",change,((ofs - 0x128) >> 2));
			WARN_ON(1);
		}
	if (ofs > 0x228 && ofs < 0x228 + sizeof(blacklist_clkstopc))
		if (change & blacklist_clkstopc[(ofs - 0x228) >> 2]) {
			printk("change=%08x,blacklist Index=%d\n",change,((ofs - 0x228) >> 2));
			WARN_ON(1);
		}

	writel(val, p);
}

void m8m_crrrs_set(unsigned offset)
{
	u32 val = readl(clk_base + CRRRS);

	if (offset > 31)
		return;

	m8m_clk_writel(val | BIT(offset), clk_base + CRRRS);
}

void m8m_crrrs_clr(unsigned offset)
{
	u32 val = readl(clk_base + CRRRS);

	if (offset > 31)
		return;

	m8m_clk_writel(val & ~BIT(offset), clk_base + CRRRS);
}

static __init void __iomem *m8m_clk_iomap(void)
{
	if (clk_base)
		return clk_base;

	np_top = of_find_compatible_node(NULL, NULL, "socionext,m8m-clk-regs");
	if (!np_top) {
		pr_err("%s: CLK iomap failed!\n", __func__);
		return NULL;
	}

	/*
	 * if these are absent or incomplete, unspecified ones default to all
	 * access allowed
	 */
	of_property_read_u32_array(np_top, "blacklist-clksel",
			blacklist_clksel, ARRAY_SIZE(blacklist_clksel));
	of_property_read_u32_array(np_top, "blacklist-clkstops",
			blacklist_clkstops, ARRAY_SIZE(blacklist_clkstops));
	of_property_read_u32_array(np_top, "blacklist-clkstopc",
			blacklist_clkstopc, ARRAY_SIZE(blacklist_clkstopc));

	clk_base = of_iomap(np_top, 0);
	of_node_put(np_top);

	return clk_base;
}

struct m8m_mux {
	struct clk_hw hw;
	const char *cname;
	u32 offset;
	u32 mask;
	u32 *table;
	u32 tlen;
	u32 parent;
};

static u8 m8m_mux_get_parent(struct clk_hw *hw)
{
	struct m8m_mux *mcm = to_m8m_mux(hw);
	struct clk_hw *parent;
	u32 *table = mcm->table;
	u32 off, shift, val;
	int i;

	if (!mcm->tlen) {
		i = clk_hw_get_num_parents(hw);
		while (i--) {
			parent = clk_hw_get_parent_by_index(hw, i);
			if (clk_hw_get_rate(parent))
				break;
		}

		if (i < 0) {
			pr_info("%s:%s no parent?!\n",
				__func__, mcm->cname);
			i = 0;
		}

		return i;
	}

	off = mcm->offset / 32 * 4;
	shift = mcm->offset % 32;

	val = readl(clk_base + CLKSEL_A + off);
	val = (val >> shift) & mcm->mask;

	for (i = 0; i < mcm->tlen && val != table[i]; i++)
		if (val == (table[i] & mcm->mask))
			break;

	if (i == mcm->tlen || i != mcm->parent) {
		pr_info("%s:%s parent i=%d h/w(%d) != s/w(%d)\n",
				__func__, mcm->cname, i, val, mcm->parent);
		if (i < mcm->tlen)
			mcm->parent = i; /* fix parent */
	}

	return i;
}

static int m8m_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct m8m_mux *mcm = to_m8m_mux(hw);
	u32 *table = mcm->table;
	u32 off, shift, val;

	if (!mcm->tlen)
		goto out;

	off = mcm->offset / 32 * 4;
	shift = mcm->offset % 32;

	val = readl(clk_base + CLKSEL_A + off);
	val &= ~(mcm->mask << shift);
	val |= (table[index] << shift);
	m8m_clk_writel(val, clk_base + CLKSEL_A + off);

out:
	mcm->parent = index;
	return 0;
}

static struct clk_ops m8m_mux_ops = {
	.get_parent = m8m_mux_get_parent,
	.set_parent = m8m_mux_set_parent,
	.determine_rate = __clk_mux_determine_rate_closest,
};

void __init m8m_clk_mux_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char **parent_names;
	struct m8m_mux *mcm;
	struct clk *clk;
	u32 offset;
	u32 mask, *mux_table = NULL;
	int i, ret, mux, parents;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parents = of_clk_get_parent_count(node);
	if (parents < 2) {
		pr_err("%s: not a mux\n", clk_name);
		return;
	}

	mux = of_property_count_u32_elems(node, "mux");
	if (mux > 0 && mux != parents) {
		pr_err("%s: invalid 'mux' property\n", clk_name);
		return;
	}
	if (mux < 0) /* readonly mux */
		mux = 0;

	if (mux) {
		ret = of_property_read_u32(node, "offset", &offset);
		if (ret) {
			pr_err("%s: missing 'offset' property\n", clk_name);
			return;
		}

		ret = of_property_read_u32(node, "mask", &mask);
		if (ret) {
			pr_err("%s: missing 'mask' property\n", clk_name);
			return;
		}

		mux_table = kzalloc(sizeof(*mux_table) * mux, GFP_KERNEL);
		if (!mux_table)
			return;
		of_property_read_u32_array(node, "mux", mux_table, mux);
	}

	parent_names = kzalloc((sizeof(char *) * parents), GFP_KERNEL);
	if (!parent_names)
		goto err_prnt;

	for (i = 0; i < parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	mcm = kzalloc(sizeof(*mcm), GFP_KERNEL);
	if (!mcm)
		goto err_mcm;

	init.name = clk_name;
	init.ops = &m8m_mux_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.num_parents = parents;
	init.parent_names = parent_names;

	mcm->cname = clk_name;
	mcm->offset = offset;
	mcm->mask = mask;
	mcm->parent = 0;
	mcm->tlen = mux;
	mcm->table = mux_table;
	mcm->hw.init = &init;

	clk = clk_register(NULL, &mcm->hw);
	if (IS_ERR(clk))
		goto err_clk;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	return;

err_clk:
	kfree(mcm);
err_mcm:
	kfree(parent_names);
err_prnt:
	kfree(mux_table);
}
CLK_OF_DECLARE(m8m_clk_mux, "socionext,m8m-clk-mux", m8m_clk_mux_setup);


struct m8m_pll {
	struct clk_hw hw;
	const char *cname;
	u32 offset;
	u32 rate_ctrl;
};

static void _mpg_enable(struct clk_hw *hw, bool enable)
{
	struct m8m_pll *mpg = to_m8m_pll(hw);
	u32 off, shift, val;

	off = (mpg->offset / 32) * 4;
	shift = (mpg->offset % 32) + 1;

	val = readl(clk_base + PLLCNT1 + off);
	if (enable)
		val &= ~(1 << shift);
	else
		val |= (1 << shift);
	m8m_clk_writel(val, clk_base + PLLCNT1 + off);
}

static int mpg_enable(struct clk_hw *hw)
{
	_mpg_enable(hw, true);
	return 0;
}

static void mpg_disable(struct clk_hw *hw)
{
	_mpg_enable(hw, false);
}

static int mpg_is_enabled(struct clk_hw *hw)
{
	struct m8m_pll *mpg = to_m8m_pll(hw);
	u32 off, shift, val;

	off = (mpg->offset / 32) * 4;
	shift = (mpg->offset % 32);

	val = readl(clk_base + PLLCNT1 + off);
	val &= (0x3 << shift);

	return !val;
}

static void _mpg_prepare(struct clk_hw *hw, bool prepare)
{
	struct m8m_pll *mpg = to_m8m_pll(hw);
	u32 off, shift, val;
	unsigned wait;

	off = (mpg->offset / 32) * 4;
	shift = (mpg->offset % 32);

	val = readl(clk_base + PLLCNT1 + off);
	if (prepare) {
		val &= ~(1 << shift);
		wait = 200;
	} else {
		val |= (1 << shift);
		wait = 5;
	}
	m8m_clk_writel(val, clk_base + PLLCNT1 + off);

	udelay(wait);
}

static int mpg_prepare(struct clk_hw *hw)
{
	_mpg_prepare(hw, true);
	return 0;
}

static void mpg_unprepare(struct clk_hw *hw)
{
	_mpg_prepare(hw, false);
}

#define M8M_PLL_MULT	21

static u32 pll_mult[M8M_PLL_MULT] = {0x27, 0x29, 0x2b, 0x2d, 0x2f,
					0x31, 0x59, 0x5a, 0x5b, 0x5c,
					0x5d, 0x5e, 0x5f, 0x60, 0x61,
					0x62, 0x63, 0x64, 0x65, 0x66, 0x67};

static long mpg_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent)
{
	unsigned long prate;
	int i;

	for (i = M8M_PLL_MULT - 1; i >= 0; i--) {
		prate = *parent * (20 + i);
		if (rate >= prate)
			break;
	}

	return prate;
}

static int mpg_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	struct m8m_pll *mpg = to_m8m_pll(hw);
	u32 off, shift, val;
	int i;

	off = mpg->rate_ctrl / 32 * 4;
	shift = mpg->rate_ctrl % 32;

	rate = mpg_round_rate(hw, rate, &prate);
	do_div(rate, prate);
	i = rate - 20;

	val = readl(clk_base + PLLCNT1 + off);
	val &= ~(0x7f << shift);
	val |= (pll_mult[i] << shift);
	m8m_clk_writel(val, clk_base + PLLCNT1 + off);

	return 0;
}

static unsigned long mpg_recalc_rate(struct clk_hw *hw,
		unsigned long prate)
{
	struct m8m_pll *mpg = to_m8m_pll(hw);
	u32 off, shift, val;
	int i;

	off = mpg->rate_ctrl / 32 * 4;
	shift = mpg->rate_ctrl % 32;

	val = readl(clk_base + PLLCNT1 + off);
	val = (val >> shift) & 0x7f;

	for (i = 0; i < M8M_PLL_MULT && pll_mult[i] != val; i++)
		;

	return prate * (20 + i);
}

static struct clk_ops m8m_pll_ops = {
	.prepare = mpg_prepare,
	.enable = mpg_enable,
	.is_enabled = mpg_is_enabled,
	.disable = mpg_disable,
	.unprepare = mpg_unprepare,
	.round_rate = mpg_round_rate,
	.set_rate = mpg_set_rate,
	.recalc_rate = mpg_recalc_rate,
};

void __init m8m_pll_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct m8m_pll *mpg;
	struct clk *clk;
	u32 offset, rate_ctrl = 0;
	int ret;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	if (of_property_read_u32(node, "rate-control", &rate_ctrl)) {
		m8m_pll_ops.set_rate = NULL;
		m8m_pll_ops.round_rate = NULL;
		m8m_pll_ops.recalc_rate = NULL;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	mpg = kzalloc(sizeof(*mpg), GFP_KERNEL);
	if (!mpg)
		return;

	init.name = clk_name;
	init.ops = &m8m_pll_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mpg->cname = clk_name;
	mpg->offset = offset;
	mpg->rate_ctrl = rate_ctrl;
	mpg->hw.init = &init;

	clk = clk_register(NULL, &mpg->hw);
	if (IS_ERR(clk))
		kfree(mpg);
	else
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(m8m_clk_pll_gate, "socionext,m8m-pll", m8m_pll_setup);

struct m8m_div {
	struct clk_hw hw;
	const char *cname;
	bool waitdchreq;
	u32 offset;
	u32 mask;
	u32 *table;
	u32 tlen;
};

static void mdc_set_div(struct m8m_div *mdc, u32 div)
{
	u32 off, shift, val;

	off = mdc->offset / 32 * 4;
	shift = mdc->offset % 32;

	val = readl(clk_base + CLKSEL_A + off);
	val &= ~(mdc->mask << shift);
	val |= (div << shift);
	m8m_clk_writel(val, clk_base + CLKSEL_A + off);

	if (mdc->waitdchreq) {
		unsigned count = 250;

		m8m_clk_writel(1, clk_base + CLKSEL_G);

		do {
			udelay(1);
		} while (--count && readl(clk_base + CLKSEL_G) & 1);

		if (!count)
			pr_err("%s:%s CLK(%d) couldn't stabilize\n",
				__func__, mdc->cname, mdc->offset);
	}
}

static u32 mdc_get_div(struct m8m_div *mdc)
{
	u32 off, shift, div;

	off = mdc->offset / 32 * 4;
	shift = mdc->offset % 32;

	div = readl(clk_base + CLKSEL_A + off);
	div >>= shift;
	div &= mdc->mask;

	return div;
}

static long mdc_div_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent)
{
	struct m8m_div *mdc = to_m8m_div(hw);
	unsigned long prate;
	int i;

	/* divisors are already in descending order in DT */
	for (i = mdc->tlen - 2; i >= 0; i -= 2) {
		prate = *parent;
		do_div(prate, mdc->table[i]);

		if (rate >= prate)
			break;
	}

	return prate;
}

static int mdc_div_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	struct m8m_div *mdc = to_m8m_div(hw);
	unsigned long pr;
	int i;

	/* divisors are already in descending order in DT */
	for (i = mdc->tlen - 2; i >= 0; i -= 2) {
		pr = prate;
		do_div(pr, mdc->table[i]);

		if (rate >= pr)
			break;
	}
	if (i < 0)
		i = 0;

	mdc_set_div(mdc, mdc->table[i + 1]);

	return 0;
}

static unsigned long mdc_div_recalc_rate(struct clk_hw *hw,
		unsigned long prate)
{
	struct m8m_div *mdc = to_m8m_div(hw);
	u32 div;
	int i;

	div = mdc_get_div(mdc);

	for (i = 1; i < mdc->tlen && div != mdc->table[i]; i += 2)
		if (div == (mdc->table[i] & mdc->mask))
			break; /* the MSB is already read back as 0 */

	if (i > mdc->tlen) /* some other is enabled in the mux */
		prate = 0;
	else
		do_div(prate, mdc->table[i - 1]);

	return prate;
}

/* Do nothing */
static int mdc_div_enable(struct clk_hw *hw)
{
	return 0;
}

static void mdc_div_disable(struct clk_hw *hw)
{
	struct m8m_div *mdc = to_m8m_div(hw);

	/* Park in least output rate */
	mdc_set_div(mdc, mdc->table[1]);
}

static struct clk_ops m8m_div_ops = {
	.enable = mdc_div_enable,
	.disable = mdc_div_disable,
	.round_rate = mdc_div_round_rate,
	.set_rate = mdc_div_set_rate,
	.recalc_rate = mdc_div_recalc_rate,
};

void __init m8m_clk_div_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct m8m_div *mdc;
	struct clk *clk;
	u32 *table, mask;
	u32 offset;
	int count, ret;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);
	if (!parent_name) {
		pr_err("%s: no parent specified\n", clk_name);
		return;
	}

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	ret = of_property_read_u32(node, "mask", &mask);
	if (ret) {
		pr_err("%s: missing 'mask' property\n", clk_name);
		return;
	}

	count = of_property_count_u32_elems(node, "ratios");
	if (count < 2 || count%2) {
		pr_err("%s: invalid 'ratios' property\n", clk_name);
		return;
	}

	table = kzalloc(sizeof(*table) * count, GFP_KERNEL);
	if (!table)
		return;

	/*
	 * The 'ratios' must be in descending order, we park at
	 * first ratio (biggest divider) when disabled.
	 */
	ret = of_property_read_u32_array(node, "ratios", table, count);
	if (ret) {
		pr_err("%s: 'ratios' property read fail\n", clk_name);
		goto err_mdc;
	}

	mdc = kzalloc(sizeof(*mdc), GFP_KERNEL);
	if (!mdc)
		goto err_mdc;

	if (of_get_property(node, "wait-on-dchreq", NULL))
		mdc->waitdchreq = true;

	init.name = clk_name;
	init.ops = &m8m_div_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mdc->cname = clk_name;
	mdc->offset = offset;
	mdc->mask = mask;
	mdc->table = table;
	mdc->tlen = count;
	mdc->hw.init = &init;

	clk = clk_register(NULL, &mdc->hw);
	if (IS_ERR(clk))
		goto err_clk;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	return;

err_clk:
	kfree(mdc);
err_mdc:
	kfree(table);
	return;
}
CLK_OF_DECLARE(m8m_clk_div, "socionext,m8m-clk-div", m8m_clk_div_setup);

void __init m8m_clk_fdiv_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	u32 *table, mask, delta = 0;
	struct clk_init_data init;
	const char *parent_name;
	u32 offset, min, max;
	struct m8m_div *mdc;
	int i, count, ret;
	struct clk *clk;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);
	if (!parent_name) {
		pr_err("%s: no parent specified\n", clk_name);
		return;
	}

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	ret = of_property_read_u32(node, "mask", &mask);
	if (ret) {
		pr_err("%s: missing 'mask' property\n", clk_name);
		return;
	}

	of_property_read_u32(node, "delta", &delta);

	count = of_property_count_u32_elems(node, "ratios");
	if (count != 2) {
		pr_err("%s: invalid 'ratios' property\n", clk_name);
		return;
	}
	of_property_read_u32_index(node, "ratios", 0, &max);
	of_property_read_u32_index(node, "ratios", 1, &min);
	if (max < min) { /* swap if 'ratios' is in ascedning order */
		count = max;
		max = min;
		min = count;
	}

	count = (max - min + 1) * 2;

	table = kzalloc(sizeof(*table) * count, GFP_KERNEL);
	if (!table)
		return;

	for (i = 0; i < count; i += 2) {
		table[i] = max - i/2;
		table[i+1] = table[i] + delta - 1;
	}

	mdc = kzalloc(sizeof(*mdc), GFP_KERNEL);
	if (!mdc)
		goto err_mdc;

	init.name = clk_name;
	init.ops = &m8m_div_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mdc->cname = clk_name;
	mdc->mask = mask;
	mdc->offset = offset;
	mdc->table = table;
	mdc->tlen = count;
	mdc->hw.init = &init;

	clk = clk_register(NULL, &mdc->hw);
	if (IS_ERR(clk))
		goto err_clk;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	return;
err_clk:
	kfree(mdc);
err_mdc:
	kfree(table);
}
CLK_OF_DECLARE(m8m_clk_fdiv, "socionext,m8m-clk-fine-div", m8m_clk_fdiv_setup);

struct m8m_gap {
	struct clk_hw hw;
	const char *cname;
	u32 offset;
	u32 mask;
};

static long mgc_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	unsigned long pr = *prate;
	int i;

	pr >>= 3; 

	for (i = 8; i > 0; i--)
		if (rate >= pr * i)
			return pr * i;
	return pr;
}

static int mgc_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	struct m8m_gap *mgc = to_m8m_gap(hw);
	u32 off, r, mask;
	int i;

	prate >>= 3; 

	for (i = 8; i > 0; i--)
		if (rate >= prate * i)
			break;
	if (i == 0)
		i = 1;

	off = (mgc->offset / 32) * 4;
	mask = mgc->mask == 1 ? 0xf : 0x1f;
	r = readl(clk_base + CLKSEL_A + off);
	r &= ~(mask << (mgc->offset % 32));
	r |= (((mgc->mask << 3) | i) << (mgc->offset % 32));
	m8m_clk_writel(r, clk_base + CLKSEL_A + off);

	return 0;
}

static unsigned long mgc_recalc_rate(struct clk_hw *hw,
		unsigned long prate)
{
	struct m8m_gap *mgc = to_m8m_gap(hw);
	u32 off, r;

	off = (mgc->offset / 32) * 4;
	r = readl(clk_base + CLKSEL_A + off);
	r = (r >> (mgc->offset % 32)) & 0x7;
	if (!r)
		return prate;

	do_div(prate, 8);
	prate *= r;

	return prate;
}

static struct clk_ops m8m_gap_ops = {
	.round_rate = mgc_round_rate,
	.set_rate = mgc_set_rate,
	.recalc_rate = mgc_recalc_rate,
};

void __init m8m_clk_gap_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct m8m_gap *mgc;
	struct clk *clk;
	u32 offset = 190;
	u32 mask = 1;
	int ret;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	of_property_read_u32(node, "or-mask", &mask);
	if (mask != 1 && mask != 2) {
		pr_err("%s: Invalid 'mask' property\n", clk_name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);
	if (!parent_name) {
		pr_err("%s: no parent specified\n", clk_name);
		return;
	}

	mgc = kzalloc(sizeof(*mgc), GFP_KERNEL);
	if (!mgc)
		return;

	init.name = clk_name;
	init.ops = &m8m_gap_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mgc->cname = clk_name;
	mgc->offset = offset;
	mgc->mask = mask;
	mgc->hw.init = &init;

	clk = clk_register(NULL, &mgc->hw);
	if (IS_ERR(clk))
		kfree(mgc);
	else
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(m8m_clk_gap, "socionext,m8m-clk-gap", m8m_clk_gap_setup);


struct m8m_gate {
	struct clk_hw hw;
	const char *cname;
	u32 offset;
};

static void _mgc_enable(struct clk_hw *hw, bool enable)
{
	struct m8m_gate *mgc = to_m8m_gate(hw);
	u32 off, mask;

	off = (mgc->offset / 32) * 4;
	mask = 1 << (mgc->offset % 32);

	if (enable)
		off += CLKSTOPC_A;
	else
		off += CLKSTOPS_A;

	m8m_clk_writel(mask, clk_base + off);
}

static int mgc_enable(struct clk_hw *hw)
{
	_mgc_enable(hw, true);
	return 0;
}

static void mgc_disable(struct clk_hw *hw)
{
	_mgc_enable(hw, false);
}

static int mgc_is_enabled(struct clk_hw *hw)
{
	struct m8m_gate *mgc = to_m8m_gate(hw);
	u32 off, mask, val;

	off = (mgc->offset / 32) * 4;
	off += CLKSTOP_A;
	mask = 1 << (mgc->offset % 32);

	val = readl(clk_base + off);

	return !(val & mask);
}

static struct clk_ops m8m_gate_ops = {
	.enable = mgc_enable,
	.disable = mgc_disable,
	.is_enabled = mgc_is_enabled,
};

void __init m8m_clk_gate_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct m8m_gate *mgc;
	struct clk *clk;
	u32 offset;
	int ret;

	if (!m8m_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	mgc = kzalloc(sizeof(*mgc), GFP_KERNEL);
	if (!mgc)
		return;

	init.name = clk_name;
	init.ops = &m8m_gate_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mgc->cname = clk_name;
	mgc->offset = offset;
	mgc->hw.init = &init;

	clk = clk_register(NULL, &mgc->hw);
	if (IS_ERR(clk))
		kfree(mgc);
	else
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(m8m_clk_gate, "socionext,m8m-clk-gate", m8m_clk_gate_setup);
