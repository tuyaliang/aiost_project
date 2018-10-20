/*
 * linux/sound/soc/socionext/mb86s27_ak4642.c
 *
 * Copyright (C) 2015 Linaro, Ltd
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

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

//#include "mb8ac0300_pcm.h"

#define DRIVER_NAME "mb86s27_ak4642"

static int mb86s27_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, 1,
				     24576000, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     12288000, SND_SOC_CLOCK_OUT);
	return ret;
}

static struct snd_soc_dai_link mb86s27_ak4642_dai_link[] = {
	[0] = {
		.name = "ak4642-playback",
		.stream_name = "Playback",
		.codec_name = "ak4642-codec.1-0013",
		.codec_dai_name = "ak4642-hifi",
		.be_id = 0,
		.cpu_dai_name = "2c080100.i2s",
		.platform_name = "mb8ac0300_pcm@0",
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.init = mb86s27_dai_init,
	},
	[1] = {
		.name = "ak4642-capture",
		.stream_name = "Capture",
		.codec_name = "ak4642-codec.1-0013",
		.codec_dai_name = "ak4642-hifi",
		.be_id = 0,
		.cpu_dai_name = "2c080200.i2s",
		.platform_name = "mb8ac0300_pcm@0",
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.init = mb86s27_dai_init,
	},
};

/* card information of sound device */
static struct snd_soc_card snd_soc_mb86s27_ak4642 = {
	.name = "mb86s27_snd",
	.driver_name = DRIVER_NAME,
	.dai_link = mb86s27_ak4642_dai_link,
	.num_links = ARRAY_SIZE(mb86s27_ak4642_dai_link),
};

/**
 * mb86s27_ak4642_probe - mb86s27 sound card probe function
 * @pdev:	pointer of platform device
 *
 * Returns 0 if no error, -ENOMEM -EINVAL or other negative errno on failure
 */
static int mb86s27_ak4642_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_mb86s27_ak4642;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret)
		pr_info(DRIVER_NAME": Failed to register card!\n");

	return ret;
}

/**
 * mb86s27_ak4642_remove - mb86s27 sound card remove function
 * @pdev:	pointer of platform device
 *
 * Returns 0
 */
static int mb86s27_ak4642_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mb86s27_ak4642_dt_ids[] = {
	{ .compatible = "socionext,mb86s27_ak4642" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s27_ak4642_dt_ids);

/* machine driver information of platform device */
static struct platform_driver mb86s27_ak4642_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s27_ak4642_dt_ids,
	},
	.probe  = mb86s27_ak4642_probe,
	.remove = mb86s27_ak4642_remove,
};
module_platform_driver(mb86s27_ak4642_driver);

MODULE_AUTHOR("FMPI");
MODULE_DESCRIPTION("MB86S27 + AK4642 audio driver");
MODULE_LICENSE("GPL");
