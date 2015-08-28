/*
 *  odroid_max98090.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#include <mach/regs-pmu.h>

#include "i2s.h"
#include "i2s-regs.h"

#define ODROID_AUD_PLL_FREQ	196608009

static struct snd_soc_card odroid;
static bool is_dummy_codec = false;

static int set_aud_pll_rate(unsigned long rate)
{
	struct clk *fout_epll;

	fout_epll = __clk_lookup("fout_epll");
	if (IS_ERR(fout_epll)) {
		printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return PTR_ERR(fout_epll);
	}

	if (rate == clk_get_rate(fout_epll))
		goto out;

	clk_set_rate(fout_epll, rate);
	printk("%s[%d] : aud_pll set_rate=%ld, get_rate = %ld\n",
		__func__,__LINE__,rate,clk_get_rate(fout_epll));
out:
	clk_put(fout_epll);

	return 0;
}

/*
 * ODROID MAX98090 I2S DAI operations. (AP master)
 */
static int odroid_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int pll, div, sclk, bfs, psr, rfs, ret;
	unsigned long rclk;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
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
		printk("Not yet supported!\n");
		return -EINVAL;
	}

	/* Set AUD_PLL frequency */
	sclk = rclk * psr;
	for (div = 2; div <= 16; div++) {
		if (sclk * div > ODROID_AUD_PLL_FREQ)
			break;
	}
	pll = sclk * (div - 1);

	set_aud_pll_rate(pll);

	/* Set CPU DAI configuration */
	if(!is_dummy_codec) {
    	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
    			| SND_SOC_DAIFMT_NB_NF
    			| SND_SOC_DAIFMT_CBS_CFS);
    	if (ret < 0)
    		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
					0, MOD_OPCLK_PCLK);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_RCLKSRC_1,
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

	if(!is_dummy_codec) {
    	ret = snd_soc_dai_set_sysclk(codec_dai, 0, rclk, SND_SOC_CLOCK_IN);
    	if (ret < 0)
    		return ret;
    }

	return 0;
}

static struct snd_soc_ops odroid_ops = {
	.hw_params = odroid_hw_params,
};

static struct snd_soc_dai_link odroid_dai[] = {
	{ /* Primary DAI i/f */
		.name = "MAX98090 AIF1",
		.stream_name = "i2s0-sec",
		.cpu_dai_name = "samsung-i2s-sec",
		.platform_name = "samsung-i2s-sec",
		.codec_dai_name = "HiFi",
		.ops = &odroid_ops,

	}, { /* Secondary DAI i/f */
		.name = "MAX98090 AIF2",
		.stream_name = "i2s0-pri",
		.codec_dai_name = "HiFi",
		.ops = &odroid_ops,
	}
};

static struct snd_soc_card odroid = {
	.name = "odroid-audio",
	.owner = THIS_MODULE,
	.dai_link = odroid_dai,
	.num_links = ARRAY_SIZE(odroid_dai),
};

static int odroid_audio_probe(struct platform_device *pdev)
{
	int n, ret;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &odroid;
	card->dev = &pdev->dev;

	for (n = 0; np && n < ARRAY_SIZE(odroid_dai); n++) {
		if (!odroid_dai[n].cpu_dai_name) {
			odroid_dai[n].cpu_of_node = of_parse_phandle(np,
					"samsung,audio-cpu", n);

			if (!odroid_dai[n].cpu_of_node) {
				dev_err(&pdev->dev, "Property "
				"'samsung,audio-cpu' missing or invalid\n");
				ret = -EINVAL;
			}
		}

		if (!odroid_dai[n].platform_name)
			odroid_dai[n].platform_of_node = odroid_dai[n].cpu_of_node;

		odroid_dai[n].codec_name = NULL;
		odroid_dai[n].codec_of_node = of_parse_phandle(np,
				"samsung,audio-codec", n);
		if (!odroid_dai[0].codec_of_node) {
			dev_err(&pdev->dev,
			"Property 'samsung,audio-codec' missing or invalid\n");
			ret = -EINVAL;
		}
	}

	ret = snd_soc_register_card(card);
	if (ret) { // MAX98090 register failed.
        dev_err(&pdev->dev, "snd_soc_register_card() failed(max98090): %d\n", ret);

        odroid_dai[0].name="DUMMY-SEC";
        odroid_dai[0].stream_name = "i2s0-sec";
        odroid_dai[0].codec_dai_name="dummy-aif1";

        odroid_dai[1].name="DUMMY-PRI";
        odroid_dai[1].stream_name = "i2s0-pri";
        odroid_dai[1].codec_dai_name="dummy-aif2";

    	for (n = 0; np && n < ARRAY_SIZE(odroid_dai); n++) {
    		odroid_dai[n].codec_name = NULL;
    		odroid_dai[n].codec_of_node = of_parse_phandle(np,
    				"samsung,audio-dummy", n);
    		if (!odroid_dai[n].codec_of_node) {
    			dev_err(&pdev->dev,
    			"Property 'samsung,audio-dummy' missing or invalid\n");
    			ret = -EINVAL;
    		}
    	}
    	ret = snd_soc_register_card(card);
    	if (ret) {
            dev_err(&pdev->dev, "snd_soc_register_card() failed(dummy_codec): %d\n", ret);
            is_dummy_codec=false;
		}
		else is_dummy_codec=true;
	}

	return ret;
}

static int odroid_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id odroid_max98090_of_match[] = {
	{ .compatible = "hardkernel,odroid-max98090", },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_max98090_of_match);
#endif /* CONFIG_OF */

static struct platform_driver odroid_audio_driver = {
	.driver		= {
		.name	= "odroid-audio",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(odroid_max98090_of_match),
#endif
	},
	.probe		= odroid_audio_probe,
	.remove		= odroid_audio_remove,
};

module_platform_driver(odroid_audio_driver);

MODULE_DESCRIPTION("ALSA SoC ODROID MAX98090");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:odroid-audio");
