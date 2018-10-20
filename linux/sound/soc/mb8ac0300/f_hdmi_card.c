/*
 * linux/sound/soc/mb8ac0300/f_hdmi_card.c
 *
 * Copyright (C) 2013-2015 Socionext
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */


#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>

#define DRV_NAME "socionext-hdmi-audio"

/**
 * Returns 0 if no error, -EINVAL or other negative errno on failure
 */
static int hdmi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	//unsigned int rate = 0;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret)
		return ret;

	/* set i2s system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0,
					24576000, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	return 0;
}

/* operations of sound device */
static struct snd_soc_ops hdmi_ops = {
	.hw_params = hdmi_hw_params,
};

static struct snd_soc_dai_link socionext_hdmi_dai = {
	.name = "f_hdmi_codec",  /* "codec name" */
	.stream_name = "hdmi", /* stream name */

	.cpu_dai_name = "0.f_hdmi_audio_dai",
	.platform_name = "mb8ac0300_pcm@1",
	.codec_name = "0.f_hdmi_codec",
	.be_id = 0,
	.ops = &hdmi_ops,
	.codec_dai_name = "socionext-hdmi-hifi",
};

static struct snd_soc_card snd_soc_socionext_hdmi = {
	.name = "sn-hdmi",
	.owner = THIS_MODULE,
	.dai_link = &socionext_hdmi_dai,
	.num_links = 1,
};

static int socionext_hdmi_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_socionext_hdmi;
	int ret;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_info(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		card->dev = NULL;
		return ret;
	}
	return 0;
}

static int socionext_hdmi_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	card->dev = NULL;
	return 0;
}

static const struct of_device_id mb86s70_hdmi_dt_ids[] = {
	{ .compatible = "socionext,hdmitx14_audio_card" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_hdmi_dt_ids);

static struct platform_driver socionext_hdmi_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_hdmi_dt_ids,
	},
	.probe = socionext_hdmi_probe,
	.remove = socionext_hdmi_remove,
};

module_platform_driver(socionext_hdmi_driver);

MODULE_AUTHOR("Socionext");
MODULE_DESCRIPTION("Socionext HDMI machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
