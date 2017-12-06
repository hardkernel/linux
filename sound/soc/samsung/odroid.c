/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "i2s.h"
#include "i2s-regs.h"

struct odroid_priv {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;
};

static int set_audio_clock_heirachy(struct device *card_dev);

/*
 * The initial rate that EPLL will be set to.  This is the smallest multiple (4)
 * of the desired master clock frequency 256 * FS for FS = 44.1khz that can
 * be generated on both the 5250 and 5420 SoCs.
 */
static int set_audss_pll_rate(struct device *card_dev, unsigned long epll,
		unsigned long sclk, unsigned long rclk)
{
	int ret;
	struct clk *fout_epll;
	struct clk *dout_srp, *dout_aud_bus;
	unsigned long clk_margin=500;

	ret = set_audio_clock_heirachy(card_dev);
	if (ret) {
		dev_err(card_dev, "failed to set up clock hierarchy (%d)\n",
			ret);
	}
	fout_epll = devm_clk_get(card_dev, "fout_epll");
	if (IS_ERR(fout_epll)) {
		dev_err(card_dev, "%s: failed to get fout_epll\n", __func__);
		return PTR_ERR(fout_epll);
	}
	dout_srp = devm_clk_get(card_dev, "dout_srp");
	if (IS_ERR(dout_srp)) {
		dev_err(card_dev, "%s: failed to get dout_srp\n", __func__);
		goto out1;
	}
	dout_aud_bus = devm_clk_get(card_dev, "dout_aud_bus");
	if (IS_ERR(dout_aud_bus)) {
		dev_err(card_dev, "%s: failed to get dout_aud_bus\n", __func__);
		goto out2;
	}

	ret = clk_set_rate(fout_epll, epll);
	if (ret < 0) {
		dev_err(card_dev, "failed to clk_set_rate of fout_epll for audio\n");
		goto out3;
	}
	ret = clk_set_rate(dout_srp, sclk+clk_margin);
	if (ret < 0) {
		dev_err(card_dev, "failed to clk_set_rate of dout_srp for audio\n");
		goto out3;
	}
	ret = clk_set_rate(dout_aud_bus, rclk+clk_margin);
	if (ret < 0) {
		dev_err(card_dev, "failed to clk_set_rate of dout_aud_bus for audio\n");
		goto out3;
	}
	dev_dbg(card_dev,"%s[%d] : epll=%ld\n",__func__,__LINE__,clk_get_rate(fout_epll));
	dev_dbg(card_dev,"%s[%d] : dout_srp=%ld\n",__func__,__LINE__,clk_get_rate(dout_srp));
	dev_dbg(card_dev,"%s[%d] : dout_bus=%ld\n",__func__,__LINE__,clk_get_rate(dout_aud_bus));
out3:
	clk_put(dout_aud_bus);
out2:
	clk_put(dout_srp);
out1:
	clk_put(fout_epll);

	return 0;
}

/* Audio clock settings are belonged to board specific part. Every
 * board can set audio source clock setting which is matched with H/W
 * like this function-'set_audio_clock_heirachy'.
 */
static int set_audio_clock_heirachy(struct device *card_dev)
{
	struct clk *fout_epll, *mout_sclk_epll;
	struct clk *mout_mau_epll, *mout_mau_epll_user;
	struct clk *mout_audss, *mout_i2s;
	struct clk *mau_epll;
	int ret = 0;

	fout_epll = devm_clk_get(card_dev, "fout_epll");
	if (IS_ERR(fout_epll)) {
		dev_err(card_dev, "%s: Cannot find fout_epll.\n",
				__func__);
		return -EINVAL;
	}
	clk_prepare_enable(fout_epll);

	mout_sclk_epll = devm_clk_get(card_dev, "mout_sclk_epll");
	if (IS_ERR(mout_sclk_epll)) {
		dev_err(card_dev, "%s: Cannot find mout_sclk_epll.\n", __func__);
		ret = -EINVAL;
		goto out1;
	}
	clk_prepare_enable(mout_sclk_epll);

	mout_mau_epll = devm_clk_get(card_dev, "mout_mau_epll");
	if (IS_ERR(mout_mau_epll)) {
		dev_err(card_dev,
			"%s: Cannot find mout_mau_epll clocks.\n", __func__);
		ret = -EINVAL;
		goto out2;
	}
	clk_prepare_enable(mout_mau_epll);

	mout_mau_epll_user = devm_clk_get(card_dev, "mout_mau_epll_user");
	if (IS_ERR(mout_mau_epll_user)) {
		dev_err(card_dev,
			"%s: Cannot find mout_mau_epll_user clocks.\n", __func__);
		ret = -EINVAL;
		goto out3;
	}
	clk_prepare_enable(mout_mau_epll_user);

	mout_audss = devm_clk_get(card_dev, "mout_audss");
	if (IS_ERR(mout_audss)) {
		dev_err(card_dev,
			"%s: Cannot find mout_audss clocks.\n", __func__);
		ret = -EINVAL;
		goto out4;
	}
	clk_prepare_enable(mout_audss);

	mout_i2s = devm_clk_get(card_dev, "mout_i2s");
	if (IS_ERR(mout_i2s)) {
		dev_err(card_dev,
			"%s: Cannot find mout_i2s clocks.\n", __func__);
		ret = -EINVAL;
		goto out5;
	}
	clk_prepare_enable(mout_i2s);

	mau_epll = devm_clk_get(card_dev, "mau_epll_clk");
	if (IS_ERR(mau_epll)) {
		dev_err(card_dev,
			"%s: Cannot find mau_epll clocks.\n", __func__);
		ret = -EINVAL;
		goto out6;
	}
	clk_prepare_enable(mau_epll);

	/* Set audio clock hierarchy for S/PDIF */
	ret = clk_set_parent(mout_sclk_epll, fout_epll);
	if (ret < 0) {
		dev_err(card_dev, "Failed to set parent of epll.\n");
		goto out7;
	}
	ret = clk_set_parent(mout_mau_epll, mout_sclk_epll);
	if (ret < 0) {
		dev_err(card_dev, "Failed to set parent of mout_mau_epll.\n");
		goto out7;
	}
	ret = clk_set_parent(mout_mau_epll_user, mout_mau_epll);
	if (ret < 0) {
		dev_err(card_dev, "Failed to set parent of epll.\n");
		goto out7;
	}
	ret = clk_set_parent(mout_audss, mout_mau_epll_user);
	if (ret < 0) {
		dev_err(card_dev, "Failed to set parent of audss.\n");
		goto out7;
	}
	ret = clk_set_parent(mout_i2s, mout_audss);
	if (ret < 0) {
		dev_err(card_dev, "Failed to set parent of mout i2s.\n");
		goto out7;
	}
out7:
	clk_put(mau_epll);
out6:
	clk_put(mout_i2s);
out5:
	clk_put(mout_audss);
out4:
	clk_put(mout_mau_epll_user);
out3:
	clk_put(mout_mau_epll);
out2:
	clk_put(mout_sclk_epll);
out1:
	clk_put(fout_epll);

	return ret;
}

static int odroid_card_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int bfs, psr, rfs, ret, div=4;
	unsigned long rclk, sclk;
	unsigned long pll;
	struct device *card_dev = substream->pcm->card->dev;

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_U24_LE:
		case SNDRV_PCM_FORMAT_S24_LE:
			bfs = 48;
			break;
		case SNDRV_PCM_FORMAT_U16_LE:
		case SNDRV_PCM_FORMAT_S16_LE:
			bfs = 32;
			break;
		default:
			return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
		break;
	default:
		return -EINVAL;
	}

	rclk = params_rate(params) * rfs;

	switch (rclk) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;
	case 67737600:
	case 73728000:
		psr = 1;
		break;
	default:
		dev_err(card_dev, "rclk = %lu is not yet supported!\n", rclk);
		return -EINVAL;
	}

	dev_dbg(card_dev,"%s[%d]: rate=%d, bfs=%d, rfs=%d, psr=%d \n",
			__func__,__LINE__, params_rate(params), bfs, rfs, psr);

	sclk = rclk * psr;
	pll = sclk * div;
	dev_dbg(card_dev,"%s[%d]: rclk=%ld, sclk=%ld, pll=%ld\n",
			__func__,__LINE__, rclk, sclk, pll);

	ret = set_audss_pll_rate(card_dev, pll, sclk, rclk);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, rclk, SND_SOC_CLOCK_IN);

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
					0, MOD_OPCLK_PCLK);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_RCLKSRC_0,
					rclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
					rfs, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct snd_soc_ops odroid_card_ops = {
	.hw_params = odroid_card_hw_params,
};

static void odroid_put_codec_of_nodes(struct snd_soc_dai_link *link)
{
	struct snd_soc_dai_link_component *component = link->codecs;
	int i;

	for (i = 0; i < link->num_codecs; i++, component++) {
		if (!component->of_node)
			break;
		of_node_put(component->of_node);
	}
}

static int odroid_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *cpu, *codec;
	struct odroid_priv *priv;
	struct snd_soc_dai_link *link;
	struct snd_soc_card *card;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card = &priv->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->fully_routed = true;

	snd_soc_card_set_drvdata(card, priv);

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0)
		return ret;

	if (of_property_read_bool(dev->of_node, "samsung,audio-widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(card,
						"samsung,audio-widgets");
		if (ret < 0)
			return ret;
	}

	if (of_property_read_bool(dev->of_node, "samsung,audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card,
						"samsung,audio-routing");
		if (ret < 0)
			return ret;
	}

	link = &priv->dai_link;

	link->ops = &odroid_card_ops;
	link->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	card->dai_link = &priv->dai_link;
	card->num_links = 1;

	cpu = of_get_child_by_name(dev->of_node, "cpu");
	codec = of_get_child_by_name(dev->of_node, "codec");

	link->cpu_of_node = of_parse_phandle(cpu, "sound-dai", 0);
	if (!link->cpu_of_node) {
		dev_err(dev, "Failed parsing cpu/sound-dai property\n");
		return -EINVAL;
	}

	ret = snd_soc_of_get_dai_link_codecs(dev, codec, link);
	if (ret < 0)
		goto err_put_codec_n;

	link->platform_of_node = link->cpu_of_node;

	link->name = "Primary";
	link->stream_name = link->name;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0) {
		dev_err(dev, "snd_soc_register_card() failed: %d\n", ret);
		goto err_put_clk_i2s;
	}

	ret = set_audio_clock_heirachy(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to set up clock hierarchy (%d)\n",
			ret);
		snd_soc_unregister_card(card);
	}

	return 0;

err_put_clk_i2s:
	of_node_put(link->cpu_of_node);
err_put_codec_n:
	odroid_put_codec_of_nodes(link);
	return ret;
}

static int odroid_audio_remove(struct platform_device *pdev)
{
	struct odroid_priv *priv = platform_get_drvdata(pdev);

	of_node_put(priv->dai_link.cpu_of_node);
	odroid_put_codec_of_nodes(&priv->dai_link);

	return 0;
}

static const struct of_device_id odroid_audio_of_match[] = {
	{ .compatible	= "samsung,odroid-xu3-audio" },
	{ .compatible	= "samsung,odroid-xu4-audio"},
	{ },
};
MODULE_DEVICE_TABLE(of, odroid_audio_of_match);

static struct platform_driver odroid_audio_driver = {
	.driver = {
		.name		= "odroid-audio",
		.of_match_table	= odroid_audio_of_match,
		.pm		= &snd_soc_pm_ops,
	},
	.probe	= odroid_audio_probe,
	.remove	= odroid_audio_remove,
};
module_platform_driver(odroid_audio_driver);

MODULE_AUTHOR("Sylwester Nawrocki <s.nawrocki@samsung.com>");
MODULE_DESCRIPTION("Odroid XU3/XU4 audio support");
MODULE_LICENSE("GPL v2");
