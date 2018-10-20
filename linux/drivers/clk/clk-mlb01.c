/*
 * Copyright (C) 2016 Linaro Ltd.
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

#define CLKSEL1		0x0
#define CLKSEL(n)	(((n) - 1) * 4 + CLKSEL1)

#define PLLCNT1		0x30
#define PLLCNT(n)	(((n) - 1) * 4 + PLLCNT1)

#define CLKSTOP1	0x54
#define CLKSTOP(n)	(((n) - 1) * 4 + CLKSTOP1)

#define CRSWR		0x8c
#define CRRRS		0x90
#define CRRSM		0x94

#define to_mlb01_mux(_hw) 	container_of(_hw, struct mlb01_mux, hw)
#define to_mlb01_gate(_hw)	container_of(_hw, struct mlb01_gate, hw)
#define to_mlb01_div(_hw) 	container_of(_hw, struct mlb01_div, hw)
#define to_mlb01_pll(_hw) 	container_of(_hw, struct mlb01_pll, hw)

static void __iomem *clk_base;
static struct device_node *np_top;
static DEFINE_SPINLOCK(crglock);

void mlb01_crrrs_set(unsigned offset)
{
	u32 val = readl(clk_base + CRRRS);

	if (offset > 31)
		return;

	writel(val | BIT(offset), clk_base + CRRRS);
}

void mlb01_crrrs_clr(unsigned offset)
{
	u32 val = readl(clk_base + CRRRS);

	if (offset > 31)
		return;

	writel(val & ~BIT(offset), clk_base + CRRRS);
}

static __init void __iomem *mlb01_clk_iomap(void)
{
	if (clk_base)
		return clk_base;

	np_top = of_find_compatible_node(NULL, NULL, "socionext,mlb01-clk-regs");
	if (!np_top) {
		pr_err("%s: CLK iomap failed!\n", __func__);
		return NULL;
	}

	clk_base = of_iomap(np_top, 0);
	of_node_put(np_top);

	return clk_base;
}

struct mlb01_mux {
	struct clk_hw hw;
	const char *cname;
	u32 parent;
};

static u8 mlb01_mux_get_parent(struct clk_hw *hw)
{
	struct mlb01_mux *mcm = to_mlb01_mux(hw);
	struct clk_hw *parent;
	int i;

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

static int mlb01_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct mlb01_mux *mcm = to_mlb01_mux(hw);

	mcm->parent = index;
	return 0;
}

static struct clk_ops mlb01_mux_ops = {
	.get_parent = mlb01_mux_get_parent,
	.set_parent = mlb01_mux_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

void __init mlb01_clk_mux_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char **parent_names;
	struct mlb01_mux *mcm;
	struct clk *clk;
	int i, parents;

	if (!mlb01_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parents = of_clk_get_parent_count(node);
	if (parents < 2) {
		pr_err("%s: not a mux\n", clk_name);
		return;
	}

	parent_names = kzalloc((sizeof(char *) * parents), GFP_KERNEL);
	if (!parent_names)
		return;

	for (i = 0; i < parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	mcm = kzalloc(sizeof(*mcm), GFP_KERNEL);
	if (!mcm)
		goto err_mcm;

	init.name = clk_name;
	init.ops = &mlb01_mux_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.num_parents = parents;
	init.parent_names = parent_names;

	mcm->cname = clk_name;
	mcm->parent = 0;
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
}
CLK_OF_DECLARE(mlb01_clk_mux, "socionext,mlb01-clk-mux", mlb01_clk_mux_setup);

struct mlb01_pll {
	struct clk_hw hw;
	const char *cname;
	struct clk_ops ops;
	u32 offset;
	u32 div, mult;
	bool ro;
};

#define ST	1
#define SEL	2

static void _mpg_enable(struct clk_hw *hw, unsigned enable)
{
	struct mlb01_pll *mpg = to_mlb01_pll(hw);
	unsigned long flags;
	u32 val;

	if (mpg->ro) {
		pr_debug("%s:%d %s: read-only\n",
			 __func__, __LINE__, mpg->cname);
		return;
	}

	spin_lock_irqsave(&crglock, flags);

	val = readl(clk_base + PLLCNT(SEL));
	if (enable)
		val |= (1 << mpg->offset);
	else
		val &= ~(1 << mpg->offset);
	writel(val, clk_base + PLLCNT(SEL));

	spin_unlock_irqrestore(&crglock, flags);
}

static int mpg_enable(struct clk_hw *hw)
{
	_mpg_enable(hw, 1);
	return 0;
}

static void mpg_disable(struct clk_hw *hw)
{
	_mpg_enable(hw, 0);
}

static int mpg_is_enabled(struct clk_hw *hw)
{
	struct mlb01_pll *mpg = to_mlb01_pll(hw);

	return readl(clk_base + PLLCNT(SEL)) & (1 << mpg->offset);
}

static void _mpg_prepare(struct clk_hw *hw, unsigned on)
{
	struct mlb01_pll *mpg = to_mlb01_pll(hw);
	unsigned long flags;
	u32 val;

	if (mpg->ro) {
		pr_debug("%s:%d %s: read-only\n",
			 __func__, __LINE__, mpg->cname);
		return;
	}

	val = readl(clk_base + PLLCNT(ST));
	if (!(on ^ ((val >> mpg->offset) & 1)))
		return;

	/* disable */
	mpg_disable(hw);

	spin_lock_irqsave(&crglock, flags);

	val = readl(clk_base + PLLCNT(ST));
	if (on)
		val |= (1 << mpg->offset);
	else
		val &= ~(1 << mpg->offset);
	writel(val, clk_base + PLLCNT(ST));

	spin_unlock_irqrestore(&crglock, flags);

	udelay(on ? 200 : 10);
}

static int mpg_prepare(struct clk_hw *hw)
{
	_mpg_prepare(hw, 1);
	return 0;
}

static void mpg_unprepare(struct clk_hw *hw)
{
	_mpg_prepare(hw, 0);
}

static int mpg_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	return 0;
}

static unsigned long mpg_recalc_rate(struct clk_hw *hw,
		unsigned long prate)
{
	struct mlb01_pll *mpg = to_mlb01_pll(hw);
	unsigned long long int rate = prate;

	if (mpg_is_enabled(hw)) {
		rate = (unsigned long long int)prate * mpg->mult;
		do_div(rate, mpg->div);
	}

	return (unsigned long)rate;
}

static long mpg_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct mlb01_pll *mpg = to_mlb01_pll(hw);

	if (mpg->ro)
		return mpg_recalc_rate(hw, *prate);

	return (*prate / mpg->div) * mpg->mult;
}

static struct clk_ops mlb01_pll_ops = {
	.prepare = mpg_prepare,
	.enable = mpg_enable,
	.is_enabled = mpg_is_enabled,
	.disable = mpg_disable,
	.unprepare = mpg_unprepare,
	.round_rate = mpg_round_rate,
	.set_rate = mpg_set_rate,
	.recalc_rate = mpg_recalc_rate,
};

void __init mlb01_pll_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	u32 offset, div, mult;
	struct mlb01_pll *mpg;
	struct clk *clk;
	int ret;

	if (!mlb01_clk_iomap())
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

	ret = of_property_read_u32(node, "offset", &offset);
	if (ret) {
		pr_err("%s: missing 'offset' property\n", clk_name);
		return;
	}

	div = mult = 1;
	of_property_read_u32(node, "clock-div", &div);
	of_property_read_u32(node, "clock-mult", &mult);

	parent_name = of_clk_get_parent_name(node, 0);

	mpg = kzalloc(sizeof(*mpg), GFP_KERNEL);
	if (!mpg)
		return;

	init.name = clk_name;
	init.ops = &mlb01_pll_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mpg->cname = clk_name;
	mpg->offset = offset;
	mpg->div = div;
	mpg->mult = mult;
	mpg->hw.init = &init;
	if (of_get_property(node, "read-only", NULL))
		mpg->ro = true;

	clk = clk_register(NULL, &mpg->hw);
	if (IS_ERR(clk))
		kfree(mpg);
	else
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(mlb01_clk_pll_gate, "socionext,mlb01-pll-fixed-factor", mlb01_pll_setup);

struct mlb01_div {
	struct clk_hw hw;
	const char *cname;
	bool waitdchreq;
	u32 offset;
	u32 mask;
	u32 *table;
	u32 tlen;
	bool ro;
};

static void mdc_set_div(struct mlb01_div *mdc, u32 div)
{
	u32 off, shift, val;

	off = mdc->offset / 32 * 4;
	shift = mdc->offset % 32;

	val = readl(clk_base + CLKSEL1 + off);
	val &= ~(mdc->mask << shift);
	val |= (div << shift);
	writel(val, clk_base + CLKSEL1 + off);

	if (mdc->waitdchreq) {
		unsigned count = 250;

		writel(1, clk_base + CLKSEL(11));

		do {
			udelay(1);
		} while (--count && readl(clk_base + CLKSEL(11)) & 1);

		if (!count)
			pr_err("%s:%s CLK(%d) couldn't stabilize\n",
				__func__, mdc->cname, mdc->offset);
	}
}

static u32 mdc_get_div(struct mlb01_div *mdc)
{
	u32 off, shift, div;

	off = mdc->offset / 32 * 4;
	shift = mdc->offset % 32;

	div = readl(clk_base + CLKSEL1 + off);
	div >>= shift;
	div &= mdc->mask;

	return div;
}

static int mdc_div_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	struct mlb01_div *mdc = to_mlb01_div(hw);
	unsigned long pr;
	int i;

	if (mdc->ro) {
		pr_debug("%s:%d %s: read-only\n",
			 __func__, __LINE__, mdc->cname);
		return 0;
	}

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
	struct mlb01_div *mdc = to_mlb01_div(hw);
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

static long mdc_div_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent)
{
	struct mlb01_div *mdc = to_mlb01_div(hw);
	unsigned long prate;
	int i;

	if (mdc->ro)
		return mdc_div_recalc_rate(hw, *parent);

	/* divisors are already in descending order in DT */
	for (i = mdc->tlen - 2; i >= 0; i -= 2) {
		prate = *parent;
		do_div(prate, mdc->table[i]);

		if (rate >= prate)
			break;
	}

	return prate;
}

static struct clk_ops mlb01_div_ops = {
	.round_rate = mdc_div_round_rate,
	.set_rate = mdc_div_set_rate,
	.recalc_rate = mdc_div_recalc_rate,
};

void __init mlb01_clk_div_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct mlb01_div *mdc;
	struct clk *clk;
	u32 *table, mask;
	u32 offset;
	int count, ret;

	if (!mlb01_clk_iomap())
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
	init.ops = &mlb01_div_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mdc->cname = clk_name;
	mdc->offset = offset;
	mdc->mask = mask;
	mdc->table = table;
	mdc->tlen = count;
	mdc->hw.init = &init;
	if (of_get_property(node, "read-only", NULL))
		mdc->ro = true;

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
CLK_OF_DECLARE(mlb01_clk_div, "socionext,mlb01-clk-div", mlb01_clk_div_setup);

struct mlb01_gate {
	struct clk_hw hw;
	const char *cname;
	u32 offset;
	bool ro;
};

static void _mgc_enable(struct clk_hw *hw, bool en)
{
	struct mlb01_gate *mgc = to_mlb01_gate(hw);
	u32 off, mask;

	if (mgc->ro) {
		pr_debug("%s:%d %s: read-only\n",
			 __func__, __LINE__, mgc->cname);
		return;
	}

	off = CLKSTOP1 + (mgc->offset / 32) * 4;

	mask = (en ? 2 : 3) << (mgc->offset % 32);

	writel(mask, clk_base + off);
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
	struct mlb01_gate *mgc = to_mlb01_gate(hw);
	u32 off, val, mask = 1 << (mgc->offset % 32);

	off = CLKSTOP1 + (mgc->offset / 32) * 4;
	val = readl(clk_base + off);

	return !(val & mask);
}

static struct clk_ops mlb01_gate_ops = {
	.enable = mgc_enable,
	.disable = mgc_disable,
	.is_enabled = mgc_is_enabled,
};

void __init mlb01_clk_gate_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk_init_data init;
	const char *parent_name;
	struct mlb01_gate *mgc;
	struct clk *clk;
	u32 offset;
	int ret;

	if (!mlb01_clk_iomap())
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
	init.ops = &mlb01_gate_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	mgc->cname = clk_name;
	mgc->offset = offset;
	mgc->hw.init = &init;
	if (of_get_property(node, "read-only", NULL))
		mgc->ro = true;

	clk = clk_register(NULL, &mgc->hw);
	if (IS_ERR(clk))
		kfree(mgc);
	else
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(mlb01_clk_gate, "socionext,mlb01-clk-gate", mlb01_clk_gate_setup);
