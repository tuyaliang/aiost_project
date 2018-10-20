/*
 * ALSA SoC DAI driver for HDMI audio on MB86S70 processors.
 * Copyright (C) 2013-2015 Socionext
 * Authors:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/asound.h>
#include <sound/asoundef.h>
#include <sound/dmaengine_pcm.h>
#include <video/fdb.h>
#include <linux/clk.h>
#include "mb8ac0300_pcm.h"

#define DRV_NAME "hdmi-audio-dai"

struct snd_soc_dai *snd_soc_find_dai(
        const struct snd_soc_dai_link_component *dlc);

struct hdmi_priv {
	struct snd_dmaengine_dai_dma_data dma_data;
	unsigned int dma_req;
	struct f_fdb_audio fdb_audio;
	struct snd_aes_iec958 iec;
	struct snd_cea_861_aud_if cea;
	struct f_fdb_child	*fdbdev;
	struct snd_soc_dai *i2s_dai;
	const char *transport_dai_name;
};

static int socionext_hdmi_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_dai_link_component cpu_dai_component;
	int err;

	if (!priv->i2s_dai) {
		/*
		 * Try to hook ourselves up to the transport DAI (I2S DAI)
		 */
		cpu_dai_component.name = priv->transport_dai_name;
		cpu_dai_component.of_node = NULL;
		cpu_dai_component.dai_name = priv->transport_dai_name;
		priv->i2s_dai = snd_soc_find_dai(&cpu_dai_component);
		if (!priv->i2s_dai) {
			dev_err(dai->dev, "ASoC: CPU DAI %s not registered\n",
				cpu_dai_component.dai_name);
			return -EPROBE_DEFER;
		}
	}

	/*
	 * Make sure that the period bytes are multiple of the DMA packet size.
	 * Largest packet size we use is 32 32-bit words = 128 bytes
	 */
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
				 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 128);
	if (err < 0) {
		dev_err(dai->dev, "could not apply constraint\n");
		return err;
	}
	snd_soc_dai_set_dma_data(dai, substream, &priv->dma_data);

	if (priv->i2s_dai->driver->ops->startup)
		priv->i2s_dai->driver->ops->startup(substream, priv->i2s_dai);

	return 0;
}

static int socionext_hdmi_dai_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);

	if (priv->i2s_dai->driver->ops->prepare)
		priv->i2s_dai->driver->ops->prepare(substream, priv->i2s_dai);

	return priv->fdbdev->ops->audio_enable(priv->fdbdev);
}

static int socionext_hdmi_dai_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	if (priv->i2s_dai->driver->ops->set_fmt)
		priv->i2s_dai->driver->ops->set_fmt(priv->i2s_dai, fmt);

	return 0;
}

static int fujistu_hdmi_dai_set_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	if (priv->i2s_dai->driver->ops->set_sysclk)
		priv->i2s_dai->driver->ops->set_sysclk(priv->i2s_dai, clk_id,
							freq, dir);

	return 0;
}

static int socionext_hdmi_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_cea_861_aud_if *cea = &priv->cea;
	struct snd_aes_iec958 *iec = &priv->iec;
	unsigned int mclk_rate;
	int err = 0;

	if (priv->i2s_dai->driver->ops->hw_params)
		priv->i2s_dai->driver->ops->hw_params(substream, params,
							priv->i2s_dai);
	/*
	 * fill the IEC-60958 channel status word
	 */
	/* initialize the word bytes */
	memset(iec->status, 0, sizeof(iec->status));

	iec->status[0] &= ~(IEC958_AES0_PROFESSIONAL | IEC958_AES0_NONAUDIO);
	iec->status[0] |= IEC958_AES0_CON_NOT_COPYRIGHT |
			  IEC958_AES0_CON_EMPHASIS_NONE |
			  IEC958_AES1_PRO_MODE_NOTID;
	iec->status[1] = IEC958_AES1_CON_GENERAL;
	iec->status[2] |= IEC958_AES2_CON_SOURCE_UNSPEC |
			  IEC958_AES2_CON_CHANNEL_UNSPEC;

	switch (params_rate(params)) {
	case 32000:
		iec->status[3] |= IEC958_AES3_CON_FS_32000;
		break;
	case 44100:
		iec->status[3] |= IEC958_AES3_CON_FS_44100;
		break;
	case 48000:
		iec->status[3] |= IEC958_AES3_CON_FS_48000;
		break;
	case 88200:
		iec->status[3] |= IEC958_AES3_CON_FS_88200;
		break;
	case 96000:
		iec->status[3] |= IEC958_AES3_CON_FS_96000;
		break;
	case 176400:
		iec->status[3] |= IEC958_AES3_CON_FS_176400;
		break;
	case 192000:
		iec->status[3] |= IEC958_AES3_CON_FS_192000;
		break;
	default:
		dev_err(dai->dev, "rate not supported!\n");
		return -EINVAL;
	}

	iec->status[3] |= IEC958_AES3_CON_CLOCK_1000PPM;
	mclk_rate = params_rate(params) * 128;

	/*
	 * specify the word length. The same word length value can mean
	 * two different lengths. Hence, we need to specify the maximum
	 * word length as well.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iec->status[4] |= IEC958_AES4_CON_WORDLEN_20_16;
		iec->status[4] &= ~IEC958_AES4_CON_MAX_WORDLEN_24;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iec->status[4] |= IEC958_AES4_CON_WORDLEN_24_20;
		iec->status[4] |= IEC958_AES4_CON_MAX_WORDLEN_24;
		break;
	default:
		dev_err(dai->dev, "format not supported!\n");
		return -EINVAL;
	}

	/*
	 * Fill the CEA-861 audio infoframe (see spec for details)
	 */

	cea->db1_ct_cc = (params_channels(params) - 1) &
			 CEA861_AUDIO_INFOFRAME_DB1CC;
	cea->db1_ct_cc |= CEA861_AUDIO_INFOFRAME_DB1CT_FROM_STREAM;
	cea->db2_sf_ss = CEA861_AUDIO_INFOFRAME_DB2SF_FROM_STREAM;
	cea->db2_sf_ss |= CEA861_AUDIO_INFOFRAME_DB2SS_FROM_STREAM;
	cea->db3 = 0; /* not used, all zeros */

	/*
	 * The HDMI IP requires to use the 8-channel channel code when
	 * transmitting more than two channels.
	 */
	if (params_channels(params) == 2)
		cea->db4_ca = 0x0;
	else
		cea->db4_ca = 0x13;

	cea->db5_dminh_lsv = CEA861_AUDIO_INFOFRAME_DB5_DM_INH_PROHIBITED;
	/* the expression is trivial but makes clear what we are doing */
	cea->db5_dminh_lsv |= (0 & CEA861_AUDIO_INFOFRAME_DB5_LSV);

	priv->fdb_audio.iec = iec;
	priv->fdb_audio.cea = cea;

	err = priv->fdbdev->ops->audio_config(priv->fdbdev,
						 &priv->fdb_audio);

	return err;
}

static int socionext_hdmi_dai_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	int err = 0;

	if (priv->i2s_dai->driver->ops->trigger)
		priv->i2s_dai->driver->ops->trigger(substream, cmd,
							priv->i2s_dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		err = priv->fdbdev->ops->audio_start(priv->fdbdev);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		priv->fdbdev->ops->audio_stop(priv->fdbdev);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static void socionext_hdmi_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);

	if (priv->i2s_dai->driver->ops->shutdown)
		priv->i2s_dai->driver->ops->shutdown(substream, priv->i2s_dai);

	priv->fdbdev->ops->audio_disable(priv->fdbdev);
}

static const struct snd_soc_dai_ops socionext_hdmi_dai_ops = {
	.startup	= socionext_hdmi_dai_startup,
	.hw_params	= socionext_hdmi_dai_hw_params,
	.prepare	= socionext_hdmi_dai_prepare,
	.trigger	= socionext_hdmi_dai_trigger,
	.shutdown	= socionext_hdmi_dai_shutdown,
	.set_fmt        = socionext_hdmi_dai_set_fmt,
	.set_sysclk     = fujistu_hdmi_dai_set_sysclk,
};

static struct snd_soc_dai_driver socionext_hdmi_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &socionext_hdmi_dai_ops,
};

static const struct snd_soc_component_driver socionext_hdmi_component = {
	.name		= DRV_NAME,
};


static int socionext_hdmi_probe(struct platform_device *pdev)
{
	struct hdmi_priv *hdmi_data;
	struct f_fdb *bus = NULL;
	int ret, n;

	hdmi_data = devm_kzalloc(&pdev->dev, sizeof(*hdmi_data), GFP_KERNEL);
	if (!hdmi_data) {
		dev_err(&pdev->dev, "No memory\n");
		return -ENOMEM;
	}

	bus = f_fdb_from_dev(&pdev->dev);

	for (n = 0; n < bus->count_fdb_children; n++) {
		if (!bus->child[n]->ops)
			continue;
		if (!bus->child[n]->ops->get_connector_type)
			continue;
		if (bus->child[n]->ops->get_connector_type(bus->child[n]) ==
		    DRM_MODE_CONNECTOR_HDMIA) {
			hdmi_data->fdbdev = bus->child[n];
			break;
		}
	}

	if (n == bus->count_fdb_children) {
		dev_err(&pdev->dev, "no driver for HDMI display found\n");
		return -ENODEV;
	}

	of_property_read_string(pdev->dev.of_node, "transport-dai",
				&hdmi_data->transport_dai_name);

	dev_set_drvdata(&pdev->dev, hdmi_data);
	ret = snd_soc_register_component(&pdev->dev, &socionext_hdmi_component,
					 &socionext_hdmi_dai, 1);

	return ret;
}

static int socionext_hdmi_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id mb86s70_hdmi_dai_dt_ids[] = {
	        { .compatible = "socionext,hdmitx14_dai" },
		{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_hdmi_dai_dt_ids);

static struct platform_driver hdmi_dai_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_hdmi_dai_dt_ids,
	},
	.probe = socionext_hdmi_probe,
	.remove = socionext_hdmi_remove,
};

module_platform_driver(hdmi_dai_driver);

MODULE_AUTHOR("Socionext");
MODULE_DESCRIPTION("SoC Digital Audio Interface for HDMI audio");
MODULE_LICENSE("GPL");
