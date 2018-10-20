/*
 * Regulator driver for Spansion S6AP412
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

enum s6ap412_regs {
	S6AP412_REG_OUTDD1	= 0x00,
	S6AP412_REG_OUTDD2	= 0x01,
	S6AP412_REG_OUTDD3	= 0x02,

	S6AP412_REG_SOFTDD1	= 0x10,
	S6AP412_REG_SOFTDD2	= 0x11,
	S6AP412_REG_SOFTDD3	= 0x12,

	S6AP412_REG_OPMODE	= 0x20,
	S6AP412_REG_ONOFF	= 0x30,
	S6AP412_REG_ERROR	= 0x40,
	S6AP412_REG_PG		= 0x50
};

struct s6ap412_regulator_info {
	struct device		*dev;
	struct regmap		*regmap;
};

static int s6ap412_enable_time(struct regulator_dev *rdev)
{
	struct s6ap412_regulator_info *info = rdev_get_drvdata(rdev);
	int voltage, rate, ret;
	unsigned int val;

	ret = regmap_read(info->regmap, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;
	voltage = regulator_list_voltage_linear(rdev, (unsigned char)val);

	ret = regmap_read(info->regmap, S6AP412_REG_SOFTDD1 +
			  rdev->desc->vsel_reg - S6AP412_REG_OUTDD1, &val);
	if (ret)
		return ret;
	rate = (32 * 1000) / (val + 1);	/* uV/uS */

	return DIV_ROUND_UP(voltage, rate);
}

static int s6ap412_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct s6ap412_regulator_info *info = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		regmap_update_bits(info->regmap, S6AP412_REG_OPMODE,
				   rdev->desc->enable_mask,
				   rdev->desc->enable_mask);
		break;
	case REGULATOR_MODE_NORMAL:
		regmap_update_bits(info->regmap, S6AP412_REG_OPMODE,
				   rdev->desc->enable_mask, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int s6ap412_get_mode(struct regulator_dev *rdev)
{
	struct s6ap412_regulator_info *info = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(info->regmap, S6AP412_REG_OPMODE, &val);
	if (ret)
		return ret;
	if (val & rdev->desc->enable_mask)
		return REGULATOR_MODE_FAST;

	return REGULATOR_MODE_NORMAL;
}

static const struct regulator_ops s6ap412_dcdc_ops = {
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.list_voltage	= regulator_list_voltage_linear,
	.map_voltage	= regulator_map_voltage_linear,
	.enable		= regulator_enable_regmap,
	.disable	= regulator_disable_regmap,
	.is_enabled	= regulator_is_enabled_regmap,
	.enable_time	= s6ap412_enable_time,
	.set_mode	= s6ap412_set_mode,
	.get_mode	= s6ap412_get_mode,
};

static struct regulator_desc dcdc_desc[] = {
	{
		.name		= "dd1",
		.ops		= &s6ap412_dcdc_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= 1 << 5,
		.owner		= THIS_MODULE,
		.vsel_reg	= S6AP412_REG_OUTDD1,
		.vsel_mask	= 0x1f,
		.min_uV		= 700000,
		.uV_step	= 20000,
		.enable_reg	= S6AP412_REG_ONOFF,
		.enable_mask	= 1,
	}, {
		.name		= "dd2",
		.ops		= &s6ap412_dcdc_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= 1 << 4,
		.owner		= THIS_MODULE,
		.vsel_reg	= S6AP412_REG_OUTDD2,
		.vsel_mask	= 0xf,
		.min_uV		= 1200000,
		.uV_step	= 50000,
		.enable_reg	= S6AP412_REG_ONOFF,
		.enable_mask	= 2,
	}, {
		.name		= "dd3",
		.ops		= &s6ap412_dcdc_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= 1 << 3,
		.owner		= THIS_MODULE,
		.vsel_reg	= S6AP412_REG_OUTDD3,
		.vsel_mask	= 7,
		.min_uV		= 2800000,
		.uV_step	= 100000,
		.enable_reg	= S6AP412_REG_ONOFF,
		.enable_mask	= 4,
	}
};

static const struct regmap_config s6ap412_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int s6ap412_regulator_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct s6ap412_regulator_info *info;
	struct regulator_dev *regulator;
	struct regulator_config config = { };
	struct device_node *np;
	int n;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->regmap = devm_regmap_init_i2c(client, &s6ap412_regmap_config);
	if (IS_ERR(info->regmap)) {
		n = PTR_ERR(info->regmap);
		dev_err(&client->dev, "Regmap init failed: %d\n", n);
		return n;
	}

	info->dev = &client->dev;
	i2c_set_clientdata(client, info);

	config.dev = &client->dev;
	config.driver_data = info;
	config.regmap = info->regmap;

	np = of_get_child_by_name(config.dev->of_node, "regulators");

	for (n = 0; n < ARRAY_SIZE(dcdc_desc); n++) {
		config.of_node = of_get_child_by_name(np, dcdc_desc[n].name);
		config.init_data = of_get_regulator_init_data(config.dev,
							      config.of_node,
							      &dcdc_desc[n]);
		regulator = devm_regulator_register(&client->dev,
						    &dcdc_desc[n], &config);
		if (IS_ERR(regulator)) {
			dev_err(info->dev, "failed to register regulator %s\n",
				dcdc_desc[n].name);
			return PTR_ERR(regulator);
		}
	}

	return 0;
}

static const struct i2c_device_id s6ap412_id[] = {
	{ "s6ap412", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s6ap412_id);

static struct i2c_driver s6ap412_driver = {
	.probe		= s6ap412_regulator_probe,
	.driver		= {
		.name	= "s6ap412",
	},
	.id_table	= s6ap412_id,
};

static int __init s6ap412_init(void)
{
	return i2c_add_driver(&s6ap412_driver);
}
subsys_initcall(s6ap412_init);

static void __exit s6ap412_exit(void)
{
	i2c_del_driver(&s6ap412_driver);
}
module_exit(s6ap412_exit);

/* Module information */
MODULE_DESCRIPTION("Spansion S6AP412");
MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_LICENSE("GPL");
