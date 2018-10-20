/*
 * linux/sound/soc/m8m/jmilau8m.c - I2S IP driver
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/reset-controller.h>
#include <linux/clk.h>

#include "../mb8ac0300/mb8ac0300_pcm.h"
#include "jmilau8m.h"

struct jmilau8m {
	struct device *dev;
	struct reset_control *rc;
	struct clk *clk[8];
	int clocks;
	struct snd_soc_dai_driver dai;
	void __iomem *base;
	void __iomem *base_WHILE_NO_PINMUX;
	phys_addr_t base_phys;
	int clk_rate;
	spinlock_t lock;
	int rate;
	int format;
	u8 bits;
	u8 channels;
	u8 id;
	u8 channel_length;
	u8 use;
	u32 master:1;
	u32 status:1;
};

static void au8m_bits(struct jmilau8m *i2s, u32 ofs, u32 reset, u32 set)
{
	u32 val = readl(i2s->base + ofs) & ~reset;

	writel(val | set, i2s->base + ofs);
}

static int _jmilau8m_set_fmt(struct jmilau8m *i2s,
			     struct snd_pcm_substream *substream)
{
	/* use the 16-deep FIFO mode */
	au8m_bits(i2s, JMILAU_CHANREG_AUMD, 0, JMILAU_AUMD__OSTG16 |
					       JMILAU_AUMD__ISTG16);

	switch (i2s->format & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s->master = false;
		au8m_bits(i2s, JMILAU_CHANREG_AUCC, JMILAU_AUCC__AUCKOE, 0);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->master = true;
		au8m_bits(i2s, JMILAU_CHANREG_AUCC, 0, JMILAU_AUCC__AUCKOE);
		break;
	default:
		return -EINVAL;
	}

	switch (i2s->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_TYPE_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_I2S << JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_TYPE_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_I2S << JMILAU_AUCR__AUIF_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_TYPE_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_LEFT_JUSTIFIED <<
					JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_TYPE_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_LEFT_JUSTIFIED <<
					JMILAU_AUCR__AUIF_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_TYPE_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_RIGHT_JUSTIFIED <<
					JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_TYPE_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_RIGHT_JUSTIFIED <<
					JMILAU_AUCR__AUIF_SHIFT);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
int jmilau8m_startup(struct snd_pcm_substream *substream,
		     struct snd_soc_dai *cpu_dai)
{
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);
	int ret, n;

	if (i2s->clocks >= 3)
		clk_set_parent(i2s->clk[1], i2s->clk[2]);
	if (i2s->clocks >= 5)
		clk_set_parent(i2s->clk[3], i2s->clk[4]);

	for (n = 0; n < i2s->clocks; n++) {
		ret = clk_prepare_enable(i2s->clk[n]);
		if (ret)
			return ret;
	}
	{
		u32 __iomem *p = ioremap_nocache(0x18001004, 4);

		if (p && !IS_ERR(p)) {
			writel((readl(p) & ~(7 << 25)) | (6 << 25), p);
			iounmap(p);
		}
	}

	reset_control_deassert(i2s->rc);
	if (i2s->base_WHILE_NO_PINMUX)
		writel(readl(i2s->base_WHILE_NO_PINMUX) &
			      ~(3 << 29), i2s->base_WHILE_NO_PINMUX);

	return 0;
}
void jmilau8m_shutdown(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *cpu_dai)
{
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);
	int n;

	reset_control_assert(i2s->rc);
	if (i2s->base_WHILE_NO_PINMUX)
		writel(readl(i2s->base_WHILE_NO_PINMUX) |
		       (3 << 29), i2s->base_WHILE_NO_PINMUX);

	for (n = 0; n < i2s->clocks; n++)
		clk_disable_unprepare(i2s->clk[n]);
}

static void jmilau8m_txctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);

	spin_lock(&i2s->lock);

	if (on) {
		au8m_bits(i2s, JMILAU_CHANREG_AUST,
			(JMILAU_AUST__ESTG_MASK << JMILAU_AUST__ESTG_SHIFT) |
			JMILAU_AUST__EINTOE,
				JMILAU_AUST__EDMOE |
				JMILAU_AUST__EINTOE |
				JMILAU_AUST__UFIE |
				(0 << JMILAU_AUST__ESTG_SHIFT));
		au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			JMILAU_AUCR__AUODSEL |
			JMILAU_AUCR__AUOE, /* we set it right at the end */
			    JMILAU_AUCR__AUODSEL | /* use DMA fifo refs */
			    JMILAU_AUCR__INTOE);
		au8m_bits(i2s, JMILAU_CHANREG_AUDMA2CTL,
			  JMILAU_AUDMA2CTL__DMO2CHEN, 0);
		au8m_bits(i2s, JMILAU_CHANREG_AUCR, 0, JMILAU_AUCR__AUOE);
	} else {
		au8m_bits(i2s, JMILAU_CHANREG_AUST,
			(JMILAU_AUST__ESTG_MASK << JMILAU_AUST__ESTG_SHIFT) |
			JMILAU_AUST__EINTOE |
			JMILAU_AUST__EDMOE, 0);
		au8m_bits(i2s, JMILAU_CHANREG_AUCR, JMILAU_AUCR__AUODSEL |
						    JMILAU_AUCR__AUOE |
						    JMILAU_AUCR__INTOE |
						    JMILAU_AUCR__DMOE, 0);
		au8m_bits(i2s, JMILAU_CHANREG_AUDMA2CTL,
			  JMILAU_AUDMA2CTL__DMO2CHEN, 0);
	}

	spin_unlock(&i2s->lock);
}

static void jmilau8m_rxctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);

	spin_lock(&i2s->lock);

	if (on) {
		au8m_bits(i2s, JMILAU_CHANREG_AUST,
			(JMILAU_AUST__ESTG_MASK << JMILAU_AUST__ESTG_SHIFT) |
			JMILAU_AUST__EDMIE |
			JMILAU_AUST__EINTIE,
				JMILAU_AUST__EDMIE |
				JMILAU_AUST__EINTIE |
				JMILAU_AUST__OFIE |
				(0 << JMILAU_AUST__ESTG_SHIFT));
		au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			JMILAU_AUCR__AUIDSEL |
			JMILAU_AUCR__DMIE |
			JMILAU_AUCR__AUIE, /* we set it right at the end */
			    JMILAU_AUCR__AUIDSEL | /* use DMA fifo refs */
			    JMILAU_AUCR__INTIE);
		au8m_bits(i2s, JMILAU_CHANREG_AUDMA2CTL,
				JMILAU_AUDMA2CTL__DMI2CHEN, 0);
		/* enable grabbing the incoming serial data into the fifo */
		au8m_bits(i2s, JMILAU_CHANREG_AUCR, 0, JMILAU_AUCR__AUIE);
	} else {
		au8m_bits(i2s, JMILAU_CHANREG_AUCR, JMILAU_AUCR__AUIE, 0);
	}

	spin_unlock(&i2s->lock);
}

static int jmilau8m_set_sysclk(struct snd_soc_dai *cpu_dai,
			     int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int jmilau8m_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);

	i2s->format = fmt;
	i2s->master = (i2s->format & SND_SOC_DAIFMT_MASTER_MASK) ==
		      SND_SOC_DAIFMT_CBS_CFS;

	return 0;
}

static int jmilau8m_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *cpu_dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8_pcm_runtime *prtd = runtime->private_data;
	struct jmilau8m *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned int div, cpw;

	_jmilau8m_set_fmt(i2s, substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prtd->dma_addr = i2s->base_phys +
					JMILAU_CHANREG_AUODDMAPT;
	else
		prtd->dma_addr = i2s->base_phys +
					JMILAU_CHANREG_AUIDDMAPT;

	i2s->channels = params_channels(params);
	if (i2s->channels == 1)
		au8m_bits(i2s, JMILAU_CHANREG_AUMD, JMILAU_AUMD__STEREO, 0);
	else
		au8m_bits(i2s, JMILAU_CHANREG_AUMD, 0, JMILAU_AUMD__STEREO);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U8:
	case SNDRV_PCM_FORMAT_S8:
		i2s->bits = 8;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_BITS_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_BITS_8 << JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_BITS_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_BITS_8 << JMILAU_AUCR__AUIF_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		i2s->bits = 16;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_BITS_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_BITS_16 << JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_BITS_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_BITS_16 << JMILAU_AUCR__AUIF_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		i2s->bits = 32;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_OFORMAT_BITS_MASK << JMILAU_AUCR__AUOF_SHIFT,
			  JMILAU_OFORMAT_BITS_24 << JMILAU_AUCR__AUOF_SHIFT);
		else
			au8m_bits(i2s, JMILAU_CHANREG_AUCR,
			  JMILAU_IFORMAT_BITS_MASK << JMILAU_AUCR__AUIF_SHIFT,
			  JMILAU_IFORMAT_BITS_24 << JMILAU_AUCR__AUIF_SHIFT);
		break;
	default:
		dev_err(cpu_dai->dev, "Bad format\n");
		return -EINVAL;
	}

	i2s->channel_length = i2s->channels * i2s->bits;
	i2s->rate = params_rate(params);

	if (!i2s->rate || !i2s->channel_length) {
		dev_err(cpu_dai->dev, "channels/rate/bits on i2s bad\n");
		return -EINVAL;
	}

	if (!i2s->master)
		return 0;

	div = clk_get_rate(i2s->clk[0]) / (2 * i2s->channel_length * i2s->rate);

	/* how many AUCLKs per word (0 =16, 1 = 32, 2 = 48, 3 = 64 */
	cpw = (i2s->channel_length / 16) - 1;

	au8m_bits(i2s, JMILAU_CHANREG_AUCC,
		  (JMILAU_AUCC__DIVMCK_MASK << JMILAU_AUCC__DIVMCK_SHIFT) |
		  (JMILAU_AUCC__DIVCK_MASK << JMILAU_AUCC__DIVCK_SHIFT) |
		  (JMILAU_AUCC__DIVLR_MASK << JMILAU_AUCC__DIVLR_SHIFT),
			  JMILAU_AUCC__AUCKOE |
			  JMILAU_AUCC__DIVE |
			  (0 << JMILAU_AUCC__DIVMCK_SHIFT) | /* no division */
			  (((div / 4) - 1) << JMILAU_AUCC__DIVCK_SHIFT) |
			  (cpw << JMILAU_AUCC__DIVLR_SHIFT)
	);

	return 0;
}

static int jmilau8m_trigger(struct snd_pcm_substream *substream, int cmd,
			  struct snd_soc_dai *cpu_dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jmilau8m_rxctrl(cpu_dai, 1);
		else
			jmilau8m_txctrl(cpu_dai, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jmilau8m_rxctrl(cpu_dai, 0);
		else
			jmilau8m_txctrl(cpu_dai, 0);
		break;
	default:
		dev_err(cpu_dai->dev, "uknown cmd\n");
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops jmilau8m_dai_ops = {
	.trigger	= jmilau8m_trigger,
	.hw_params	= jmilau8m_hw_params,
	.set_fmt	= jmilau8m_set_fmt,
	.set_sysclk	= jmilau8m_set_sysclk,
	.startup	= jmilau8m_startup,
	.shutdown	= jmilau8m_shutdown,
};

struct snd_soc_dai_driver jmilau8m_dai_init = {
	.name = "jmilau8m",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			   SNDRV_PCM_FMTBIT_U8 |
			   SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_U16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_U24_LE,
		.rates = SNDRV_PCM_RATE_8000_192000,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			   SNDRV_PCM_FMTBIT_U8 |
			   SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_U16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_U24_LE,
		.rates = SNDRV_PCM_RATE_8000_192000,
	},
	.ops = &jmilau8m_dai_ops,
};

static const struct snd_soc_component_driver jmilau8m_i2s_comp = {
	.name = "jmilau8m-i2s",
};

static int jmilau8m_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jmilau8m *i2s;
	struct resource *res;
	int ret;

	i2s = kzalloc(sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = dev;
	spin_lock_init(&i2s->lock);
	i2s->rc = reset_control_get(dev, "reset");
	if (IS_ERR_OR_NULL(i2s->rc)) {
		ret = PTR_ERR(i2s->rc);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err2;
	}
	i2s->base_phys = (phys_addr_t)res->start;

	i2s->dai = jmilau8m_dai_init;
	dev_set_drvdata(&pdev->dev, i2s);

	i2s->base = devm_ioremap_resource(dev, res);
	if (i2s->base == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	i2s->base_WHILE_NO_PINMUX = of_iomap(dev->of_node, 1);

	do {
		i2s->clk[i2s->clocks] = of_clk_get(pdev->dev.of_node,
						   i2s->clocks);
		if (IS_ERR_OR_NULL(i2s->clk[i2s->clocks]))
			break;
		i2s->clocks++;
	} while (i2s->clocks < ARRAY_SIZE(i2s->clk));
	if (!i2s->clocks) {
		ret = PTR_ERR(i2s->clk);
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto err2;
	}

	ret = snd_soc_register_component(&pdev->dev, &jmilau8m_i2s_comp,
					 &i2s->dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register dai\n");
		goto err3;
	}
	dev_info(&pdev->dev, "Registered as %s\n", i2s->dai.name);

	return 0;

err3:
	while (--i2s->clocks)
		clk_put(i2s->clk[i2s->clocks]);
err2:
	if (i2s->base_WHILE_NO_PINMUX)
		iounmap(i2s->base_WHILE_NO_PINMUX);

	reset_control_put(i2s->rc);
err:
	kfree(i2s);

	return ret;
}

static int jmilau8m_remove(struct platform_device *pdev)
{
	struct jmilau8m *i2s = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);
	iounmap(i2s->base);
	reset_control_put(i2s->rc);
	while (--i2s->clocks)
		clk_put(i2s->clk[i2s->clocks]);

	kfree(i2s);

	return 0;
}

static const struct of_device_id jmilau8m_dt_ids[] = {
	{ .compatible = "socionext,jmilau8m" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, jmilau8m_dt_ids);

static struct platform_driver jmilau8m_driver = {
	.probe = jmilau8m_probe,
	.remove = jmilau8m_remove,
	.driver = {
		.name = "jmilau8m",
		.owner = THIS_MODULE,
		.of_match_table = jmilau8m_dt_ids,
	},
};

module_platform_driver(jmilau8m_driver);


MODULE_DESCRIPTION("Socionext JMILAU8M I2S driver");
MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_LICENSE("GPL");

