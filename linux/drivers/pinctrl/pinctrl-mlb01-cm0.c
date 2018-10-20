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

#define CM0_BANKS	6
#define PINS_PER_BANK	8
#define CM0_TOTAL_PINS	(CM0_BANKS * PINS_PER_BANK)

struct pin_irq_map {
	int pin; /* offset of pin in the managed range */
	int irq; /* virq of the pin as fpint */
	int type;
	char irqname[8];
};

struct cm0_pinctrl {
	void __iomem *base;
	void __iomem *exiu;
	struct gpio_chip gc;
	struct pinctrl_desc pd;
	char pin_names[4 * CM0_TOTAL_PINS];
	struct pinctrl_pin_desc pins[CM0_TOTAL_PINS];
	unsigned gpins[CM0_TOTAL_PINS][1]; /* 1 pin-per-group */
	struct irq_domain *irqdom;
	spinlock_t irq_lock, lock;
	int extints;
	struct pin_irq_map fpint[]; /* keep at end */
};

struct cm0_function {
	const char		*name;
	const char * const	*groups;
	unsigned int		ngroups;
};

static const char cm0_bank_name[] = {'0', '1', '2', '3', '4', '5'};
static const char * gpio_cm0_grps[CM0_TOTAL_PINS];

static const struct cm0_function cm0_functions[] = {
#define FUNC_CM0(fname)					\
	{						\
		.name = #fname,				\
		.groups = fname##_cm0_grps,		\
		.ngroups = ARRAY_SIZE(fname##_cm0_grps),	\
	}
	FUNC_CM0(gpio), /* GPIO always at index 0 */
};

static inline unsigned pins_per_reg(struct cm0_pinctrl *pctl)
{
	return 16;
}

static int cm0_pconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned group,
				 unsigned long *configs,
				 unsigned num_configs)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 pin, val, reg, offset;
	unsigned long flags;
	int i;

	pin = pctl->gpins[group][0];
	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

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

static const struct pinconf_ops cm0_pconf_ops = {
	.pin_config_group_set	= cm0_pconf_group_set,
};

static int cm0_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return CM0_TOTAL_PINS;
}

static const char *cm0_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned pin)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return &pctl->pin_names[4 * pin];
}

static int cm0_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned group,
				      const unsigned **pins,
				      unsigned *num_pins)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctl->gpins[group];
	*num_pins = 1;
	return 0;
}

static const struct pinctrl_ops cm0_pctrl_ops = {
	.get_groups_count	= cm0_pctrl_get_groups_count,
	.get_group_name		= cm0_pctrl_get_group_name,
	.get_group_pins		= cm0_pctrl_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int cm0_pmx_get_funcs_cnt(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(cm0_functions);
}

static const char *cm0_pmx_get_func_name(struct pinctrl_dev *pctldev,
					   unsigned function)
{
	return cm0_functions[function].name;
}

static int cm0_pmx_get_func_groups(struct pinctrl_dev *pctldev,
				     unsigned function,
				     const char * const **groups,
				     unsigned * const num_groups)
{
	*groups = cm0_functions[function].groups;
	*num_groups = cm0_functions[function].ngroups;
	return 0;
}

static void _set_mux(struct cm0_pinctrl *pctl, unsigned pin, bool gpio)
{
	u32 val, reg, offset;
	unsigned long flags;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

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

static int cm0_pmx_set_mux(struct pinctrl_dev *pctldev,
			     unsigned function,
			     unsigned group)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 pin = pctl->gpins[group][0]; /* each group has exactly 1 pin */

	_set_mux(pctl, pin, !function);

	return 0;
}

static int _set_direction(struct cm0_pinctrl *pctl,
			unsigned pin, bool input)
{
	u32 val, reg, offset;
	unsigned long flags;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

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
cm0_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range,
			unsigned pin, bool input)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return _set_direction(pctl, pin, input);
}

static int
cm0_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
			    struct pinctrl_gpio_range *range,
			    unsigned pin)
{
	struct cm0_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	_set_mux(pctl, pin, true);
	return 0;
}

static const struct pinmux_ops cm0_pmx_ops = {
	.get_functions_count	= cm0_pmx_get_funcs_cnt,
	.get_function_name	= cm0_pmx_get_func_name,
	.get_function_groups	= cm0_pmx_get_func_groups,
	.set_mux		= cm0_pmx_set_mux,
	.gpio_set_direction	= cm0_pmx_gpio_set_direction,
	.gpio_request_enable	= cm0_pmx_gpio_request_enable,
};

static int cm0_gpio_get(struct gpio_chip *gc, unsigned group)
{
	struct cm0_pinctrl *pctl = container_of(gc, struct cm0_pinctrl, gc);
	u32 pin, val, reg, offset;

	pin = pctl->gpins[group][0];
	reg = PDR + pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);
	val = readl_relaxed(pctl->base + reg);

	return !!(val & BIT(offset));
}

static void cm0_gpio_set(struct gpio_chip *gc, unsigned group, int set)
{
	struct cm0_pinctrl *pctl = container_of(gc, struct cm0_pinctrl, gc);
	u32 pin, reg, offset, val;

	pin = pctl->gpins[group][0];
	reg = PDR + pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

	val = BIT(offset + 16);
	if (set)
		val |= BIT(offset);

	writel_relaxed(val, pctl->base + reg);
}

static int cm0_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int cm0_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int value)
{
	int ret;

	ret = pinctrl_gpio_direction_output(gc->base + offset);
	if (!ret)
		cm0_gpio_set(gc, offset, value);

	return ret;
}

static int cm0_gpio_request(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_request_gpio(gc->base + offset);
}

static void cm0_gpio_free(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_free_gpio(gc->base + offset);
}

static int cm0_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct cm0_pinctrl *pctl = container_of(gc, struct cm0_pinctrl, gc);

	return irq_linear_revmap(pctl->irqdom, offset);
}

static struct lock_class_key gpio_lock_class;

static int pin_to_extint(struct cm0_pinctrl *pctl, int pin)
{
	int extint;

	for (extint = 0; extint < pctl->extints; extint++)
		if (pctl->fpint[extint].pin == pin)
			break;

	if (extint == pctl->extints)
		return -1;

	return extint;
}

static void update_trigger(struct cm0_pinctrl *pctl, int extint)
{
	int type = pctl->fpint[extint].type;
	int pin = pctl->fpint[extint].pin;
	u32 masked, val, eilvl, eiedg;
	int lvl;
	u32 reg, offset;

	reg = pin / pins_per_reg(pctl) * 4;
	offset = pin % pins_per_reg(pctl);

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

static irqreturn_t cm0_gpio_irq_handler(int irq, void *data)
{
	struct cm0_pinctrl *pctl = data;
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

static void cm0_gpio_irq_enable(struct irq_data *data)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
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

static void cm0_gpio_irq_disable(struct irq_data *data)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
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

static int cm0_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
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

void cm0_gpio_irq_ack(struct irq_data *data)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
	int extint = pin_to_extint(pctl, irqd_to_hwirq(data));

	if (extint < 0 || !pctl->exiu)
		return;

	writel_relaxed(1 << extint, pctl->exiu + EIREQCLR);
}

void cm0_gpio_irq_mask(struct irq_data *data)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
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

void cm0_gpio_irq_unmask(struct irq_data *data)
{
	struct cm0_pinctrl *pctl = irq_data_get_irq_chip_data(data);
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

static struct irq_chip cm0_gpio_irq_chip = {
	.name = "mlb01-cm0-pin-irq",
	.irq_enable = cm0_gpio_irq_enable,
	.irq_disable = cm0_gpio_irq_disable,
	.irq_set_type = cm0_gpio_irq_set_type,
	.irq_mask = cm0_gpio_irq_mask,
	.irq_unmask = cm0_gpio_irq_unmask,
	.irq_ack = cm0_gpio_irq_ack,
};

static struct of_device_id cm0_pmatch[] = {
	{ .compatible = "socionext,mlb01-cm0-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, cm0_pmatch);

static int cm0_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_dev *pctl_dev;
	struct pin_irq_map fpint[32];
	struct cm0_pinctrl *pctl;
	struct pinctrl_desc *pd;
	struct gpio_chip *gc;
	struct resource *res;
	int idx, i, ret, extints;

	extints = of_irq_count(np);

	pctl = devm_kzalloc(&pdev->dev,	sizeof(*pctl) +
				sizeof(struct pin_irq_map) * extints,
				GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

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

	for (i = 0; i < CM0_TOTAL_PINS; i++) {
		pctl->pins[i].number = i;
		pctl->pins[i].name = &pctl->pin_names[4 * i];
		snprintf(&pctl->pin_names[4 * i], 4, "P%c%d",
			cm0_bank_name[i / PINS_PER_BANK], i % PINS_PER_BANK);
		gpio_cm0_grps[i] = &pctl->pin_names[4 * i];
		pctl->gpins[i][0] = i;
	}
	pd->name = dev_name(&pdev->dev);
	pd->pins = pctl->pins;
	pd->npins = CM0_TOTAL_PINS;
	pd->pctlops = &cm0_pctrl_ops;
	pd->pmxops = &cm0_pmx_ops;
	pd->confops = &cm0_pconf_ops;
	pd->owner = THIS_MODULE;
	pctl_dev = pinctrl_register(pd, &pdev->dev, pctl);
	if (!pctl_dev) {
		dev_err(&pdev->dev, "couldn't register pinctrl driver\n");
		return -EINVAL;
	}

	pctl->extints = extints;

	pctl->irqdom = irq_domain_add_linear(np, CM0_TOTAL_PINS,
						&irq_domain_simple_ops, pctl);
	idx = 0;
	for (i = 0; i < CM0_TOTAL_PINS && idx < pctl->extints; i++) {
		int irq;

		snprintf(fpint[idx].irqname, 8, "pin-%d", i);
		irq = platform_get_irq_byname(pdev, fpint[idx].irqname);
		if (irq < 0)
			continue;
		fpint[idx].irq = irq;
		fpint[idx].pin = i;
		idx++;
	}

	for (idx = 0, i = 0; i < pctl->extints; i++) {
		int j = 0, irq = platform_get_irq(pdev, i);

		while (fpint[j].irq != irq)
			j++;

		snprintf(pctl->fpint[idx].irqname, 8, "pin-%d", fpint[j].pin);
		pctl->fpint[idx].irq = fpint[j].irq;
		pctl->fpint[idx].pin = fpint[j].pin;
		idx++;
	}

	for (i = 0; i < pctl->extints; i++) {
		int irq, err = devm_request_irq(&pdev->dev, pctl->fpint[i].irq,
					cm0_gpio_irq_handler, IRQF_SHARED,
					pctl->fpint[i].irqname, pctl);
		if (err)
			continue;

		irq = irq_create_mapping(pctl->irqdom, pctl->fpint[i].pin);
		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_and_handler(irq, &cm0_gpio_irq_chip,
					 handle_level_irq);
		irq_set_chip_data(irq, pctl);
	}

	gc->base = -1;
	gc->ngpio = CM0_TOTAL_PINS;
	gc->label = dev_name(&pdev->dev);
	gc->dev = &pdev->dev;
	gc->owner = THIS_MODULE;
	gc->of_node = np;
	gc->direction_input = cm0_gpio_direction_input;
	gc->direction_output = cm0_gpio_direction_output;
	gc->get = cm0_gpio_get;
	gc->set = cm0_gpio_set;
	gc->to_irq = cm0_gpio_to_irq;
	gc->request = cm0_gpio_request;
	gc->free = cm0_gpio_free;
	ret = gpiochip_add(gc);
	if (ret) {
		dev_err(&pdev->dev, "Failed register gpiochip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(gc, dev_name(&pdev->dev),
					0, 0, CM0_TOTAL_PINS);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add pin range\n");
		gpiochip_remove(gc);
		return ret;
	}

	return 0;
}

static struct platform_driver cm0_pinctrl_driver = {
	.probe	= cm0_pinctrl_probe,
	.driver	= {
		.name		= "mlb01-cm0-pinctrl",
		.of_match_table	= cm0_pmatch,
	},
};
module_platform_driver(cm0_pinctrl_driver);

MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
MODULE_DESCRIPTION("Socionext Milbeaut CM0 pinctrl driver");
MODULE_LICENSE("GPL");
