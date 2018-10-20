/*
 * ALSA SoC codec driver for HDMI audio on MB86S70.
 * Copyright (C) 2013-2015 Socionext
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <sound/soc.h>

#define DRV_NAME "f_hdmi_codec"

static struct snd_soc_codec_driver socionext_hdmi_codec;

static struct snd_soc_dai_driver socionext_hdmi_codec_dai = {
	.name = "socionext-hdmi-hifi",
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
};

static int socionext_hdmi_codec_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_codec(&pdev->dev, &socionext_hdmi_codec,
			&socionext_hdmi_codec_dai, 1);

	dev_info(&pdev->dev, "Registered %d\n", ret);

	return ret;
}

static int socionext_hdmi_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id hdmi_dt_ids[] = {
	{ .compatible = "socionext,f_hdmi_codec" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hdmi_dt_ids);

static struct platform_driver socionext_hdmi_codec_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = hdmi_dt_ids,
	},

	.probe		= socionext_hdmi_codec_probe,
	.remove		= socionext_hdmi_codec_remove,
};

module_platform_driver(socionext_hdmi_codec_driver);

MODULE_AUTHOR("Socionext Semiconductor Limited");
MODULE_DESCRIPTION("ASoC HDMI codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*f_hdmi_codec*");
