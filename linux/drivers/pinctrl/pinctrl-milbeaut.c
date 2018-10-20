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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include "pinctrl-utils.h"

#define EIMASK		0x0
#define EISRCSEL	0x4
#define EIREQSTA	0x8
#define EIRAWREQSTA	0xc
#define EIREQCLR	0x10
#define EILVL		0x14
#define EIEDG		0x18
#define EISIR		0x1c

#define PDR		0x0
#define PDR_S		0x50
#define PDR_C		0xa0
#define DDR		0x100
#define EPCR		0x200
#define PUDER		0x300
#define PUDCR		0x400

#define M8M_BANKS	18
#define MLB01_BANKS	26
#define PINS_PER_BANK	8
#define M8M_TOTAL_PINS	(M8M_BANKS * PINS_PER_BANK)
#define MLB01_TOTAL_PINS	(MLB01_BANKS * PINS_PER_BANK)
#define M8M_EXIU_PINS	32
#define MLB01_EXIU_PINS	16

#define FPINT_INVALID	-1

struct pin_irq_map {
	int pin; /* offset of pin in the managed range */
	int irq; /* virq of the pin as fpint */
	int type;
	char irqname[8];
};

enum milbeaut_variant {
	VAR_INVAL = 0,
	VAR_M8M,
	VAR_MLB01,
};

struct milbeaut_pinctrl {
	void __iomem *base;
	void __iomem *exiu;
	struct gpio_chip gc;
	struct pinctrl_desc pd;
	char pin_names[4 * MLB01_TOTAL_PINS];
	struct pinctrl_pin_desc pins[MLB01_TOTAL_PINS];
	unsigned gpins[MLB01_TOTAL_PINS][1]; /* 1 pin-per-group */
	struct irq_domain *irqdom;
	spinlock_t irq_lock, lock;
	int extints;
	u32 blacklist[MLB01_BANKS];
	enum milbeaut_variant variant;
	struct pin_irq_map fpint[]; /* keep at end */
};

struct milbeaut_function {
	const char		*name;
	const char * const	*groups;
	unsigned int		ngroups;
};

static const char m8m_bank_name[] = {'3', '4', '5', '6', '7', '8', '9', 'A', 'C',
				 'D', 'E', 'F', 'G', 'P', 'R', 'S', 'T', 'U'};
static const char * const au0_m8m_grps[] = {"P30", "P31", "P32", "P33"};
static const char * const au1_m8m_grps[] = {"P34", "P35", "P36", "P37"};
static const char * const au2_m8m_grps[] = {"PU0", "PU1", "PU2", "PU3", "P41"};
static const char * const i2c0_m8m_grps[] = {"P50", "P51"};
static const char * const i2c1_m8m_grps[] = {"P42", "P43"};
static const char * const i2c2_m8m_grps[] = {"P44", "P45"};
static const char * const usio0_m8m_grps[] = {"P52", "P53", "P54"};
static const char * const usio1_m8m_grps[] = {"P55", "P56", "P57"};
static const char * const usio2_m8m_grps[] = {"PS1", "PS2", "PS3"};
static const char * const usio3_m8m_grps[] = {"P83", "P84", "P85"};
static const char * const usio4_m8m_grps[] = {"PA1", "PA2", "PA3"};
static const char * const usio5_m8m_grps[] = {"PD5", "PD4", "PD3"};
static const char * const sd0_m8m_grps[] = {"PC2", "PC3", "PC4",
					  "PC5", "PC6", "PC7", "PA5"};
static const char * const srlot_m8m_grps[] = {"PS4", "PS5", "PS6"};
static const char * gpio_m8m_grps[M8M_TOTAL_PINS];

static const struct milbeaut_function m8m_functions[] = {
#define FUNC_M8M(fname)					\
	{						\
		.name = #fname,				\
		.groups = fname##_m8m_grps,		\
		.ngroups = ARRAY_SIZE(fname##_m8m_grps),	\
	}
	FUNC_M8M(gpio), /* GPIO always at index 0 */
	FUNC_M8M(au0),
	FUNC_M8M(au1),
	FUNC_M8M(au2),
	FUNC_M8M(i2c0),
	FUNC_M8M(i2c1),
	FUNC_M8M(i2c2),
	FUNC_M8M(usio0),
	FUNC_M8M(usio1),
	FUNC_M8M(usio2),
	FUNC_M8M(usio3),
	FUNC_M8M(usio4),
	FUNC_M8M(usio5),
	FUNC_M8M(sd0),
	FUNC_M8M(srlot),
};

static const int m8m_exiu_pin_list[] = {32, 33, 34, 35, 0, 1, 2, 3,
				24, 25, 26, 27, 49, 50, 51, 127,
				60, 120, 78, 79, 112, 113, 119, 129,
				130, 124, 125, 63, 4, 5, 7, 6};

static const char mlb01_bank_name[] = {'0', '1', '2', '3', '4', '5', '6', '7',
				 '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
				 'G', 'H', 'W', 'J', 'K', 'L', 'M', 'N',
				 'Y', 'P'};
static const char * const usio0_mlb01_grps[] = {"PE2", "PE3", "PF0"};
static const char * const usio1_mlb01_grps[] = {"PE4", "PE5", "PF1"};
static const char * const usio2_mlb01_grps[] = {"PE0", "PE1"};
static const char * const usio3_mlb01_grps[] = {"PY0", "PY1", "PY2"};
static const char * const usio4_mlb01_grps[] = {"PP0", "PP1", "PP2"};
static const char * const usio5_mlb01_grps[] = {"PM0", "PM1", "PM3"};
static const char * const usio6_mlb01_grps[] = {"PN0", "PN1", "PN3"};
static const char * const usio7_mlb01_grps[] = {"PY3", "PY5", "PY6"};
static const char * gpio_mlb01_grps[MLB01_TOTAL_PINS];

static const struct milbeaut_function mlb01_functions[] = {
#define FUNC_MLB01(fname)					\
	{						\
		.name = #fname,				\
		.groups = fname##_mlb01_grps,		\
		.ngroups = ARRAY_SIZE(fname##_mlb01_grps),	\
	}
	FUNC_MLB01(gpio), /* GPIO always at index 0 */
	FUNC_MLB01(usio0),
	FUNC_MLB01(usio1),
	FUNC_MLB01(usio2),
	FUNC_MLB01(usio3),
	FUNC_MLB01(usio4),
	FUNC_MLB01(usio5),
	FUNC_MLB01(usio6),
	FUNC_MLB01(usio7),
};

static const int mlb01_exiu_pin_list[] = {48, 49, 50, 51, 52, 53, 54, 55,
				56, 57, 58, 59, 60, 61, 62, 63};

static const char *bank_name;
static const struct milbeaut_function *milbeaut_functions;

static inline unsigned pins_per_reg(struct milbeaut_pinctrl *pctl)
{
	return pctl->variant == VAR_MLB01 ? 16 : 8;
}

static int milbeaut_pconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned group,
				 unsigned long *configs,
				 unsigned num_configs)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 pin, val, reg, offset;
	unsigned long flags;
	int i;

	pin = pctl->gpins[group][0];
	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	if (pctl->blacklist[reg >> 1] & (1 << offset)) {
		printk("change=%08x,blacklist Index=%d\n",(1 << offset),(reg >> 1));
		WARN_ON(1);
	}
	spin_lock_irqsave(&pctl->lock, flags);

	for (i = 0; i < num_configs; i++) {
		switch (pinconf_to_config_param(configs[i])) {
		case PIN_CONFIG_BIAS_PULL_UP:
			/* enable Pull-Up/Down resistance */
			val = readl_relaxed(pctl->base + PUDER + reg);
			val |= BIT(offset);
			writel_relaxed(val, pctl->base + PUDER + reg);
			/* enable Pull-Up */
			val = readl_relaxed(pctl->base + PUDCR + reg);
			val |= BIT(offset);
			writel_relaxed(val, pctl->base + PUDCR + reg);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			/* enable Pull-Up/Down resistance */
			val = readl_relaxed(pctl->base + PUDER + reg);
			val |= BIT(offset);
			writel_relaxed(val, pctl->base + PUDER + reg);
			/* enable Pull-Down */
			val = readl_relaxed(pctl->base + PUDCR + reg);
			val &= ~BIT(offset);
			writel_relaxed(val, pctl->base + PUDCR + reg);
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			val = readl_relaxed(pctl->base + PUDER + reg);
			val &= ~BIT(offset);
			writel_relaxed(val, pctl->base + PUDER + reg);
			break;
		default:
			break;
		}
	}

	spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static const struct pinconf_ops milbeaut_pconf_ops = {
	.pin_config_group_set	= milbeaut_pconf_group_set,
};

static int milbeaut_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->variant == VAR_MLB01 ? MLB01_TOTAL_PINS : M8M_TOTAL_PINS;
}

static const char *milbeaut_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned pin)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return &pctl->pin_names[4 * pin];
}

static int milbeaut_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned group,
				      const unsigned **pins,
				      unsigned *num_pins)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctl->gpins[group];
	*num_pins = 1;
	return 0;
}

static const struct pinctrl_ops milbeaut_pctrl_ops = {
	.get_groups_count	= milbeaut_pctrl_get_groups_count,
	.get_group_name		= milbeaut_pctrl_get_group_name,
	.get_group_pins		= milbeaut_pctrl_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int milbeaut_pmx_get_funcs_cnt(struct pinctrl_dev *pctldev)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (pctl->variant == VAR_MLB01)
		return ARRAY_SIZE(mlb01_functions);

	return ARRAY_SIZE(m8m_functions);
}

static const char *milbeaut_pmx_get_func_name(struct pinctrl_dev *pctldev,
					   unsigned function)
{
	return milbeaut_functions[function].name;
}

static int milbeaut_pmx_get_func_groups(struct pinctrl_dev *pctldev,
				     unsigned function,
				     const char * const **groups,
				     unsigned * const num_groups)
{
	*groups = milbeaut_functions[function].groups;
	*num_groups = milbeaut_functions[function].ngroups;
	return 0;
}

static void _set_mux(struct milbeaut_pinctrl *pctl, unsigned pin, bool gpio)
{
	u32 val, reg, offset;
	unsigned long flags;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	if (pctl->blacklist[reg >> 1] & (1 << offset)) {
		printk("change=%08x,blacklist Index=%d\n",(1 << offset),(reg >> 1));
		WARN_ON(1);
	}

	reg += EPCR;

	spin_lock_irqsave(&pctl->lock, flags);

	val = readl_relaxed(pctl->base + reg);

	if (gpio)
		val &= ~BIT(offset);
	else
		val |= BIT(offset);

	writel_relaxed(val, pctl->base + reg);

	spin_unlock_irqrestore(&pctl->lock, flags);
}

static int milbeaut_pmx_set_mux(struct pinctrl_dev *pctldev,
			     unsigned function,
			     unsigned group)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 pin = pctl->gpins[group][0]; /* each group has exactly 1 pin */

	_set_mux(pctl, pin, !function);

	return 0;
}

static int _set_direction(struct milbeaut_pinctrl *pctl,
			unsigned pin, bool input)
{
	u32 val, reg, offset;
	unsigned long flags;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	if (pctl->blacklist[reg >> 1] & (1 << offset)) {
		printk("change=%08x,blacklist Index=%d\n",(1 << offset),(reg >> 1));
		WARN_ON(1);
	}

	reg += DDR;

	spin_lock_irqsave(&pctl->lock, flags);

	val = readl_relaxed(pctl->base + reg);

	if (input)
		val &= ~BIT(offset);
	else
		val |= BIT(offset);

	writel_relaxed(val, pctl->base + reg);

	spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static int
milbeaut_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range,
			unsigned pin, bool input)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return _set_direction(pctl, pin, input);
}

static int
milbeaut_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
			    struct pinctrl_gpio_range *range,
			    unsigned pin)
{
	struct milbeaut_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	_set_mux(pctl, pin, true);
	return 0;
}

static const struct pinmux_ops milbeaut_pmx_ops = {
	.get_functions_count	= milbeaut_pmx_get_funcs_cnt,
	.get_function_name	= milbeaut_pmx_get_func_name,
	.get_function_groups	= milbeaut_pmx_get_func_groups,
	.set_mux		= milbeaut_pmx_set_mux,
	.gpio_set_direction	= milbeaut_pmx_gpio_set_direction,
	.gpio_request_enable	= milbeaut_pmx_gpio_request_enable,
};

static int milbeaut_gpio_get(struct gpio_chip *gc, unsigned group)
{
	struct milbeaut_pinctrl *pctl = container_of(gc, struct milbeaut_pinctrl, gc);
	u32 pin, val, reg, offset;

	pin = pctl->gpins[group][0];
	reg = PDR + pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);
	val = readl_relaxed(pctl->base + reg);

	return !!(val & BIT(offset));
}

static void m8m_gpio_set(struct gpio_chip *gc, unsigned group, int set)
{
	struct milbeaut_pinctrl *pctl = container_of(gc, struct milbeaut_pinctrl, gc);
	u32 pin, reg, offset;

	pin = pctl->gpins[group][0];
	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	if (pctl->blacklist[reg >> 1] & (1 << offset)) {
		printk("change=%08x,blacklist Index=%d\n",(1 << offset),(reg >> 1));
		WARN_ON(1);
	}

	if (set)
		reg += PDR_S;
	else
		reg += PDR_C;

	writel_relaxed(BIT(offset), pctl->base + reg);
}

static void mlb01_gpio_set(struct gpio_chip *gc, unsigned group, int set)
{
	struct milbeaut_pinctrl *pctl = container_of(gc, struct milbeaut_pinctrl, gc);
	u32 pin, reg, offset, val;

	pin = pctl->gpins[group][0];
	reg = PDR + pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	val = BIT(offset + 16);
	if (set)
		val |= BIT(offset);

	writel_relaxed(val, pctl->base + reg);
}

static void (*gpio_set)(struct gpio_chip *, unsigned, int);

static int milbeaut_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int milbeaut_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int value)
{
	int ret;

	ret = pinctrl_gpio_direction_output(gc->base + offset);
	if (!ret)
		gpio_set(gc, offset, value);

	return ret;
}

static int milbeaut_gpio_request(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_request_gpio(gc->base + offset);
}

static void milbeaut_gpio_free(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_free_gpio(gc->base + offset);
}

static int milbeaut_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct milbeaut_pinctrl *pctl = container_of(gc, struct milbeaut_pinctrl, gc);

	return irq_linear_revmap(pctl->irqdom, offset);
}

static struct lock_class_key gpio_lock_class;

static int pin_to_extint(struct milbeaut_pinctrl *pctl, int pin)
{
	int extint;

        if (pctl->blacklist[pin / pins_per_reg(pctl)]
			& (1 << (pin % pins_per_reg(pctl))))
		return -1;

	for (extint = 0; extint < pctl->extints; extint++)
		if (pctl->fpint[extint].pin == pin)
			break;

	if (extint == pctl->extints)
		return -1;

	return extint;
}

static void update_trigger(struct milbeaut_pinctrl *pctl, int extint)
{
	int type = pctl->fpint[extint].type;
	int pin = pctl->fpint[extint].pin;
	u32 masked, val, eilvl, eiedg;
	int lvl;
	u32 reg, offset;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	if (pctl->blacklist[reg >> 1] & (1 << offset) || !pctl->exiu)
		return;

	/* sense gpio */
	val = readl_relaxed(pctl->base + PDR + reg);
	lvl = (val >> offset) & 1;

	eilvl = readl_relaxed(pctl->exiu + EILVL);
	eiedg = readl_relaxed(pctl->exiu + EIEDG);

	if (type == IRQ_TYPE_LEVEL_LOW ||
			(lvl && (type & IRQ_TYPE_LEVEL_LOW))) {
		eilvl &= ~(1 << extint);
		eiedg &= ~(1 << extint);
	}

	if (type == IRQ_TYPE_EDGE_FALLING ||
			(lvl && (type & IRQ_TYPE_EDGE_FALLING))) {
		eilvl &= ~(1 << extint);
		eiedg |= 1 << extint;
	}

	if (type == IRQ_TYPE_LEVEL_HIGH ||
			(!lvl && (type & IRQ_TYPE_LEVEL_HIGH))) {
		eilvl |= 1 << extint;
		eiedg &= ~(1 << extint);
	}

	if (type == IRQ_TYPE_EDGE_RISING ||
			(!lvl && (type & IRQ_TYPE_EDGE_RISING))) {
		eilvl |= 1 << extint;
		eiedg |= 1 << extint;
	}

	/* Mask the interrupt */
	val = readl_relaxed(pctl->exiu + EIMASK);
	masked = val & (1 << extint); /* save status */
	val |= 1 << extint;
	writel_relaxed(val, pctl->exiu + EIMASK);

	/* Program trigger */
	writel_relaxed(eilvl, pctl->exiu + EILVL);
	writel_relaxed(eiedg, pctl->exiu + EIEDG);

	if (masked)
		return;

	/* UnMask the interrupt */
	val = readl_relaxed(pctl->exiu + EIMASK);
	val &= ~(1 << extint);
	writel_relaxed(val, pctl->exiu + EIMASK);
}

static irqreturn_t milbeaut_gpio_irq_handler(int irq, void *data)
{
	struct milbeaut_pinctrl *pctl = data;
	int i, pin;
	u32 val;

	for (i = 0; i < pctl->extints; i++)
		if (pctl->fpint[i].irq == irq)
			break;
	if (i == pctl->extints) {
		pr_err("%s:%d IRQ(%d)!\n", __func__, __LINE__, irq);
		return IRQ_NONE;
	}

	if (!pctl->exiu)
		return IRQ_NONE;

	val = readl_relaxed(pctl->exiu + EIREQSTA);
	if (!(val & (1 << i))) {
		pr_err("%s:%d i=%d EIREQSTA=0x%x IRQ(%d)!\n",
				__func__, __LINE__, i, val, irq);
		return IRQ_NONE;
	}

	pin = pctl->fpint[i].pin;
	generic_handle_irq(irq_linear_revmap(pctl->irqdom, pin));

	return IRQ_HANDLED;
}

static void milbeaut_gpio_irq_enable(struct irq_data *data)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));
	unsigned long flags;
	u32 val;

	if (extint < 0 || !pctl->exiu)
		return;

	_set_mux(pctl, irqd_to_hwirq(data), true);
	_set_direction(pctl, irqd_to_hwirq(data), true);

	spin_lock_irqsave(&pctl->irq_lock, flags);

	/* Clear before enabling */
	writel_relaxed(1 << extint, pctl->exiu + EIREQCLR);

	/* UnMask the interrupt */
	val = readl_relaxed(pctl->exiu + EIMASK);
	val &= ~(1 << extint);
	writel_relaxed(val, pctl->exiu + EIMASK);

	spin_unlock_irqrestore(&pctl->irq_lock, flags);
}

static void milbeaut_gpio_irq_disable(struct irq_data *data)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));
	unsigned long flags;
	u32 val;

	if (extint < 0 || !pctl->exiu)
		return;

	spin_lock_irqsave(&pctl->irq_lock, flags);

	/* Mask the interrupt */
	val = readl_relaxed(pctl->exiu + EIMASK);
	val |= 1 << extint;
	writel_relaxed(val, pctl->exiu + EIMASK);

	spin_unlock_irqrestore(&pctl->irq_lock, flags);
}

static int milbeaut_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));
	unsigned long flags;

	if (extint < 0 || !pctl->exiu)
		return -EINVAL;

	spin_lock_irqsave(&pctl->irq_lock, flags);

	pctl->fpint[extint].type = type;
	update_trigger(pctl, extint);

	spin_unlock_irqrestore(&pctl->irq_lock, flags);

	return 0;
}

void milbeaut_gpio_irq_ack(struct irq_data *data)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));

	if (extint < 0 || !pctl->exiu)
		return;

	writel_relaxed(1 << extint, pctl->exiu + EIREQCLR);
}

void milbeaut_gpio_irq_mask(struct irq_data *data)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));
	unsigned long flags;
	u32 val;

	if (extint < 0 || !pctl->exiu)
		return;

	spin_lock_irqsave(&pctl->irq_lock, flags);

	val = readl_relaxed(pctl->exiu + EIMASK);
	val |= 1 << extint;
	writel_relaxed(val, pctl->exiu + EIMASK);

	spin_unlock_irqrestore(&pctl->irq_lock, flags);
}

void milbeaut_gpio_irq_unmask(struct irq_data *data)
{
	struct milbeaut_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));
	unsigned long flags;
	u32 val;

	if (extint < 0 || !pctl->exiu)
		return;

	spin_lock_irqsave(&pctl->irq_lock, flags);

	update_trigger(pctl, extint);

	val = readl_relaxed(pctl->exiu + EIMASK);
	val &= ~(1 << extint);
	writel_relaxed(val, pctl->exiu + EIMASK);

	spin_unlock_irqrestore(&pctl->irq_lock, flags);
}

static struct irq_chip milbeaut_gpio_irq_chip = {
	.name = "milbeaut-pin-irq",
	.irq_enable = milbeaut_gpio_irq_enable,
	.irq_disable = milbeaut_gpio_irq_disable,
	.irq_set_type = milbeaut_gpio_irq_set_type,
	.irq_mask = milbeaut_gpio_irq_mask,
	.irq_unmask = milbeaut_gpio_irq_unmask,
	.irq_ack = milbeaut_gpio_irq_ack,
};

static struct of_device_id milbeaut_pmatch[] = {
	{ .compatible = "socionext,m8m-pinctrl", .data = (void *)VAR_M8M },
	{ .compatible = "socionext,mlb01-pinctrl", .data = (void *)VAR_MLB01 },
	{}
};
MODULE_DEVICE_TABLE(of, milbeaut_pmatch);

static int milbeaut_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_dev *pctl_dev;
	struct pin_irq_map fpint[32];
	struct milbeaut_pinctrl *pctl;
	struct pinctrl_desc *pd;
	struct gpio_chip *gc;
	struct resource *res;
	int i, ret, extints, tpins;
	const int *exiu_pin_list;
	enum milbeaut_variant variant;

	variant = (enum milbeaut_variant)of_match_device(milbeaut_pmatch,
							&pdev->dev)->data;
	
	if (variant == VAR_MLB01)
		extints = MLB01_EXIU_PINS;
	else
		extints = M8M_EXIU_PINS;

	pctl = devm_kzalloc(&pdev->dev,	sizeof(*pctl) +
				sizeof(struct pin_irq_map) * extints,
				GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->variant = variant;

	if (pctl->variant == VAR_MLB01) {
		gpio_set = mlb01_gpio_set;
		bank_name = mlb01_bank_name;
		milbeaut_functions = mlb01_functions;
		tpins = MLB01_TOTAL_PINS;
		exiu_pin_list = mlb01_exiu_pin_list;
	} else {
		gpio_set = m8m_gpio_set;
		bank_name = m8m_bank_name;
		milbeaut_functions = m8m_functions;
		tpins = M8M_TOTAL_PINS;
		exiu_pin_list = m8m_exiu_pin_list;
	}

	pd = &pctl->pd;
	gc = &pctl->gc;
	spin_lock_init(&pctl->lock);
	spin_lock_init(&pctl->irq_lock);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pinctrl");
	pctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (!pctl->base)
		return -EINVAL;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "exiu");
	if (res)
		pctl->exiu = devm_ioremap_resource(&pdev->dev, res);
	if (res && !IS_ERR(pctl->exiu)) {
		writel_relaxed(~0, pctl->exiu + EIMASK); /* mask all */
		writel_relaxed(~0, pctl->exiu + EIREQCLR); /* eoi all */
		writel_relaxed(0, pctl->exiu + EISRCSEL); /* all fpint */
		writel_relaxed(~0, pctl->exiu + EILVL); /* rising edge type*/
		writel_relaxed(~0, pctl->exiu + EIEDG);
	} else {
		dev_info(&pdev->dev, "continuing without EXIU support\n");
		pctl->exiu = NULL;
	}

	for (i = 0; i < tpins; i++) {
		pctl->pins[i].number = i;
		pctl->pins[i].name = &pctl->pin_names[4 * i];
		snprintf(&pctl->pin_names[4 * i], 4, "P%c%d",
			bank_name[i / PINS_PER_BANK], i % PINS_PER_BANK);
		if (pctl->variant == VAR_MLB01)
			gpio_mlb01_grps[i] = &pctl->pin_names[4 * i];
		else
			gpio_m8m_grps[i] = &pctl->pin_names[4 * i];
		pctl->gpins[i][0] = i;
	}
	/* absent or incomplete entries allow all access */
	of_property_read_u32_array(np, "blacklist", &pctl->blacklist[0],
				   ARRAY_SIZE(pctl->blacklist));
	pd->name = dev_name(&pdev->dev);
	pd->pins = pctl->pins;
	pd->npins = tpins;
	pd->pctlops = &milbeaut_pctrl_ops;
	pd->pmxops = &milbeaut_pmx_ops;
	pd->confops = &milbeaut_pconf_ops;
	pd->owner = THIS_MODULE;
	pctl_dev = pinctrl_register(pd, &pdev->dev, pctl);
	if (!pctl_dev) {
		dev_err(&pdev->dev, "couldn't register pinctrl driver\n");
		return -EINVAL;
	}

	pctl->extints = extints;

	pctl->irqdom = irq_domain_add_linear(np, tpins,
						&irq_domain_simple_ops, pctl);
	for (i = 0; i < pctl->extints ; i++) {
		int irq;

		snprintf(fpint[i].irqname, 8, "pin-%d", exiu_pin_list[i]);
		irq = platform_get_irq_byname(pdev, fpint[i].irqname);
		if (irq < 0) {
			fpint[i].irq = FPINT_INVALID;
			fpint[i].pin = FPINT_INVALID;
			continue;
		}
		fpint[i].irq = irq;
		fpint[i].pin = exiu_pin_list[i];
	}

	for (i = 0; i < pctl->extints; i++) {
		int j = 0, irq = platform_get_irq(pdev, i);
		if(irq < 0)
			continue;
		while (fpint[j].irq != irq)
			j++;

		snprintf(pctl->fpint[j].irqname, 8, "pin-%d", fpint[j].pin);
		pctl->fpint[j].irq = fpint[j].irq;
		pctl->fpint[j].pin = fpint[j].pin;
	}
	for(i = 0; i < pctl->extints; i++) {
		if(pctl->fpint[i].irq == 0) {
			pctl->fpint[i].irq = FPINT_INVALID;
			pctl->fpint[i].pin = FPINT_INVALID;
			pctl->fpint[i].irqname[0] = 0;
		}
	}


	for (i = 0; i < pctl->extints; i++) {
		int irq, err;
		if(pctl->fpint[i].irq == FPINT_INVALID)
			continue;
		err = devm_request_irq(&pdev->dev, pctl->fpint[i].irq,
					milbeaut_gpio_irq_handler, IRQF_SHARED,
					pctl->fpint[i].irqname, pctl);
		if (err)
			continue;
		irq = irq_create_mapping(pctl->irqdom, pctl->fpint[i].pin);
		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_and_handler(irq, &milbeaut_gpio_irq_chip,
					 handle_level_irq);
		irq_set_chip_data(irq, pctl);
	}

	gc->base = -1;
	gc->ngpio = tpins;
	gc->label = dev_name(&pdev->dev);
	gc->dev = &pdev->dev;
	gc->owner = THIS_MODULE;
	gc->of_node = np;
	gc->direction_input = milbeaut_gpio_direction_input;
	gc->direction_output = milbeaut_gpio_direction_output;
	gc->get = milbeaut_gpio_get;
	gc->set = gpio_set;
	gc->to_irq = milbeaut_gpio_to_irq;
	gc->request = milbeaut_gpio_request;
	gc->free = milbeaut_gpio_free;
	ret = gpiochip_add(gc);
	if (ret) {
		dev_err(&pdev->dev, "Failed register gpiochip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(gc, dev_name(&pdev->dev),
					0, 0, tpins);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add pin range\n");
		gpiochip_remove(gc);
		return ret;
	}

	return 0;
}

static struct platform_driver milbeaut_pinctrl_driver = {
	.probe	= milbeaut_pinctrl_probe,
	.driver	= {
		.name		= "milbeaut-pinctrl",
		.of_match_table	= milbeaut_pmatch,
	},
};
module_platform_driver(milbeaut_pinctrl_driver);

MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
MODULE_DESCRIPTION("Socionext Milbeaut pinctrl driver");
MODULE_LICENSE("GPL");
