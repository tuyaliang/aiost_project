/*
 * Driver for Casio COM3OT38391
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * based on ->>
 * Driver for the Himax HX-8357 LCD Controller
 *
 * Copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

struct com3ot38391_data {
	unsigned		reset;
	struct spi_device	*spi;
	int			state;
	struct backlight_device *bd;
	u8			txbuf[2];
};

static u8 reverse(u8 n)
{
	return ((n &    1) << 7) |
	       ((n &    2) << 5) |
	       ((n &    4) << 3) |
	       ((n &    8) << 1) |
	       ((n & 0x10) >> 1) |
	       ((n & 0x20) >> 3) |
	       ((n & 0x40) >> 5) |
	       ((n & 0x80) >> 7);
}

static int com3ot38391_spi_write(struct lcd_device *lcdev, u8 reg, u8 val)
{
	struct com3ot38391_data *lcd = lcd_get_data(lcdev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	int ret = 0;

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	lcd->txbuf[0] = reverse(reg);
	lcd->txbuf[1] = reverse(val);

	xfer[0].len = 2;
	xfer[0].bits_per_word = 8;
	xfer[0].tx_buf = &lcd->txbuf[0];
	spi_message_add_tail(&xfer[0], &msg);

	ret = spi_sync(lcd->spi, &msg);
	if (ret < 0)
		dev_err(&lcdev->dev, "Couldn't send SPI data\n");

	return ret;
}

static int com3ot38391_enter_standby(struct lcd_device *lcdev)
{
	com3ot38391_spi_write(lcdev, 7, BIT(2) | BIT(1));

	return 0;
}

static int com3ot38391_exit_standby(struct lcd_device *lcdev)
{

	com3ot38391_spi_write(lcdev, 7, BIT(2) | BIT(1));
	com3ot38391_spi_write(lcdev, 0xa, 8);
	com3ot38391_spi_write(lcdev, 0xb, 0xf);
	com3ot38391_spi_write(lcdev, 0x10, 3);

	com3ot38391_spi_write(lcdev, 7, BIT(4) | BIT(2) | BIT(1));

	return 0;
}

static void com3ot38391_lcd_reset(struct lcd_device *lcdev)
{
	struct com3ot38391_data *lcd = lcd_get_data(lcdev);

	if (!gpio_is_valid(lcd->reset))
		return;

	/* Reset the screen */
	gpio_set_value(lcd->reset, 1);
	usleep_range(10000, 12000);
	gpio_set_value(lcd->reset, 0);
	usleep_range(10000, 12000);
	gpio_set_value(lcd->reset, 1);
}

static int com3ot38391_lcd_init(struct lcd_device *lcdev)
{
	com3ot38391_exit_standby(lcdev);

	return 0;
}
#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

static int com3ot38391_set_power(struct lcd_device *lcdev, int power)
{
	struct com3ot38391_data *lcd = lcd_get_data(lcdev);
	int ret = 0;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->state))
		ret = com3ot38391_exit_standby(lcdev);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->state))
		ret = com3ot38391_enter_standby(lcdev);

	if (ret == 0)
		lcd->state = power;
	else
		dev_warn(&lcdev->dev, "failed to set power mode %d\n", power);

	return ret;
}

static int com3ot38391_get_power(struct lcd_device *lcdev)
{
	struct com3ot38391_data *lcd = lcd_get_data(lcdev);

	return lcd->state;
}

static struct lcd_ops com3ot38391_ops = {
	.set_power	= com3ot38391_set_power,
	.get_power	= com3ot38391_get_power,
};

static const struct of_device_id com3ot38391_dt_ids[] = {
	{
		.compatible = "casio,com3ot38391",
		.data = com3ot38391_lcd_init,
	},
	{},
};
MODULE_DEVICE_TABLE(of, com3ot38391_dt_ids);

static int com3ot_bl_update_status(struct backlight_device *bd)
{
	struct lcd_device *lcdev = bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

//	com3ot38391_spi_write(lcdev, 0, 0xff - (brightness << 2));
	com3ot38391_spi_write(lcdev, 7, (brightness != 0) << 4 |
					BIT(2) | BIT(1));
	com3ot38391_spi_write(lcdev, 0x23, 0);
	com3ot38391_spi_write(lcdev, 0x29, 0);
	if (brightness)
		com3ot38391_spi_write(lcdev, 0x27, brightness | BIT(7));
	else
		com3ot38391_spi_write(lcdev, 0x27, 0);

	return 0;
}

static const struct backlight_ops com3ot_bl_ops = {
	.update_status	= com3ot_bl_update_status,
};

static int com3ot38391_probe(struct spi_device *spi)
{
	struct lcd_device *lcdev;
	struct com3ot38391_data *lcd;
	const struct of_device_id *match;
	struct backlight_properties props;
	int ret;

	lcd = devm_kzalloc(&spi->dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI setup failed.\n");
		return ret;
	}

	lcd->spi = spi;

	match = of_match_device(com3ot38391_dt_ids, &spi->dev);
	if (!match || !match->data)
		return -EINVAL;

	lcd->reset = of_get_named_gpio(spi->dev.of_node, "gpios-reset", 0);
	if (gpio_is_valid(lcd->reset)) {
		ret = devm_gpio_request_one(&spi->dev, lcd->reset,
				    GPIOF_OUT_INIT_HIGH,
				    "com3ot38391-reset");
		if (ret) {
			dev_err(&spi->dev,
				"failed to request gpio %d: %d\n",
				lcd->reset, ret);
			return -EINVAL;
		}
	}

	lcdev = devm_lcd_device_register(&spi->dev, "com3ot38391",
					 &spi->dev, lcd, &com3ot38391_ops);
	if (IS_ERR(lcdev)) {
		ret = PTR_ERR(lcdev);
		return ret;
	}
	spi_set_drvdata(spi, lcdev);

	com3ot38391_lcd_reset(lcdev);

	ret = ((int (*)(struct lcd_device *))match->data)(lcdev);
	if (ret) {
		dev_err(&spi->dev, "Couldn't initialize panel\n");
		return ret;
	}

	if (of_property_read_bool(spi->dev.of_node, "backlight")) {
		memset(&props, 0, sizeof(props));
		props.type = BACKLIGHT_RAW;
		props.max_brightness = 0x3f;
		props.brightness = 0x3f;
		lcd->bd = devm_backlight_device_register(&spi->dev, "bl",
					spi->dev.parent, lcdev, &com3ot_bl_ops,
					&props);
		if (IS_ERR(lcd->bd)) {
			dev_err(&spi->dev, "failed to register backlight\n");
			return PTR_ERR(lcd->bd);
		}
	}
	dev_info(&spi->dev, "Panel probed\n");

	return 0;
}

static struct spi_driver com3ot38391_driver = {
	.probe  = com3ot38391_probe,
	.driver = {
		.name = "com3ot38391",
		.of_match_table = com3ot38391_dt_ids,
	},
};

module_spi_driver(com3ot38391_driver);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("Casio COM3OT38391 Panel backlight driver");
MODULE_LICENSE("GPL");
