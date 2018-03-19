/*
 * ASoC driver for AIT MCRV2 platform
 *
 * Author:      Vincent Chen , <vincent_chen@a-i-t.com>
 * Copyright:   (C) 2015 AIT
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#define MCLK_RATE 12000000

#define CODEC_AUDIO_FORMAT_CAPTURE (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM)

#define CODEC_AUDIO_FORMAT_PLAY (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)

#define MCRV2_SOC_I2S_DAI_FMT_CAPTURE (SND_SOC_DAIFMT_I2S  | SND_SOC_DAIFMT_NB_NF |SND_SOC_DAIFMT_CBS_CFS)

#define PLL_OUT_DEVIDER 0
#define SOC_I2S_MCLK_DEVIDER 1

static int mcrv2_i2s_wm8973_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)                         
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *codec_dai = rtd->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        int ret = 0;
        unsigned sysclk;

	sysclk = 12000000;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

	        /* set codec DAI configuration */
	       ret = snd_soc_dai_set_fmt(codec_dai, CODEC_AUDIO_FORMAT_PLAY);
	        if (ret < 0)
	                return ret;



	} else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE){


	        /* set codec DAI configuration */
	       ret = snd_soc_dai_set_fmt(codec_dai, CODEC_AUDIO_FORMAT_CAPTURE);
	        if (ret < 0)
	                return ret;

		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, MCRV2_SOC_I2S_DAI_FMT_CAPTURE);
		if (ret < 0)
			return ret;

	}

        /* set the codec system clock */
        ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 96000000, SND_SOC_CLOCK_OUT);	//configure DAI system or master clock.
        if (ret < 0)
                return ret;

        ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, 1);	//configure DAI clock dividers.
        if (ret < 0)
                return ret;

        ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, 96000000/sysclk);	//configure DAI clock dividers.
        if (ret < 0)
                return ret;
		
        /* set the codec system clock */
        ret = snd_soc_dai_set_sysclk(codec_dai, 1, sysclk, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        return 0;
}

static struct snd_soc_ops mcrv2_snd_i2s_wm8973_ops = {
	.hw_params = mcrv2_i2s_wm8973_hw_params,
};

static const struct snd_soc_dapm_widget mcrv2_i2s_dapm_widgets[] = {
        SND_SOC_DAPM_HP("Headphone Jack", NULL),
        SND_SOC_DAPM_LINE("Line Out", NULL),
        SND_SOC_DAPM_MIC("Mic Jack", NULL),
        SND_SOC_DAPM_LINE("Line In", NULL),
};

/* mcrv2 machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
        /* Headphone connected to HPLOUT, HPROUT */
        {"Headphone Jack", NULL, "HPLOUT"},
        {"Headphone Jack", NULL, "HPROUT"},

        /* Line Out connected to LLOUT, RLOUT */
        {"Line Out", NULL, "LLOUT"},
        {"Line Out", NULL, "RLOUT"},

        /* Mic connected to (MIC3L | MIC3R) */
        {"MIC3L", NULL, "Mic Bias 2V"},
        {"MIC3R", NULL, "Mic Bias 2V"},
        {"Mic Bias 2V", NULL, "Mic Jack"},

        /* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
        {"LINE1L", NULL, "Line In"},
        {"LINE2L", NULL, "Line In"},
        {"LINE1R", NULL, "Line In"},
        {"LINE2R", NULL, "Line In"},
};

static int mcrv2_soc_snd_i2s_init(struct snd_soc_pcm_runtime *rtd)
{
        struct snd_soc_codec *codec = rtd->codec;
        struct snd_soc_dapm_context *dapm = &codec->dapm;

        /* Add davinci-evm specific widgets */
        snd_soc_dapm_new_controls(dapm, mcrv2_i2s_dapm_widgets,
                                  ARRAY_SIZE(mcrv2_i2s_dapm_widgets));

        /* Set up davinci-evm specific audio path audio_map */
        snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

        /* not connected */
        snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
        snd_soc_dapm_disable_pin(dapm, "HPLCOM");
        snd_soc_dapm_disable_pin(dapm, "HPRCOM");

        /* always connected */
        snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
        snd_soc_dapm_enable_pin(dapm, "Line Out");
        snd_soc_dapm_enable_pin(dapm, "Mic Jack");
        snd_soc_dapm_enable_pin(dapm, "Line In");

        return 0;
}

static const struct snd_soc_dapm_widget wm8973_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route wm8973_intercon[] = {

	/* speaker connected to LHPOUT */
	{"Ext Spk", NULL, "LHPOUT"},

	/* mic is connected to Mic Jack, with WM8731 Mic Bias */
	{"MICIN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
};


static int ait6366evb_i2s_wm8973_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 1,//WM8731_SYSCLK_MCLK,
		MCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set WM8973 SYSCLK: %d\n", ret);
		return ret;
	}

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "MIC");
	snd_soc_dapm_enable_pin(dapm, "LOUT1");
	snd_soc_dapm_enable_pin(dapm, "ROUT1");
	snd_soc_dapm_enable_pin(dapm, "LOUT2");
	snd_soc_dapm_enable_pin(dapm, "ROUT2");
	snd_soc_dapm_enable_pin(dapm, "MONO");
	snd_soc_dapm_enable_pin(dapm, "LINPUT1");
	snd_soc_dapm_enable_pin(dapm, "RINPUT1");


	
	return 0;
}


static int ait6366evb_i2s_wm8737_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 1,//WM8731_SYSCLK_MCLK,
		MCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set WM8737 SYSCLK: %d\n", ret);
		return ret;
	}


	/* not connected */


	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "LINPUT1");
	snd_soc_dapm_enable_pin(dapm, "LINPUT2");
	snd_soc_dapm_enable_pin(dapm, "LINPUT3");

	snd_soc_dapm_enable_pin(dapm, "RINPUT1");
	snd_soc_dapm_enable_pin(dapm, "RINPUT2");
	snd_soc_dapm_enable_pin(dapm, "RINPUT3");

	snd_soc_dapm_enable_pin(dapm, "LACIN");
	snd_soc_dapm_enable_pin(dapm, "RACIN");
	return 0;
}


static struct snd_soc_dai_link mcrv2_snd_soc_i2s_dai[] = {
#if 0	
	{
		.name = "MCRV2-I2S DUMMY",
		.stream_name = "I2S CODEC DUMMY",
		.cpu_dai_name = "mcrv2-soc-i2s",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",
		.codec_dai_name = "i2s-codec-dummy-hifi",

		.init = mcrv2_soc_snd_i2s_init,
		.codec_name = "i2s-codec-dummy",
		.ops = &ait8455evb_snd_i2s_ops,
	},
#endif	
	{
		.name = "MCRV2-I2S WM8973",
		.stream_name = "WM8973 PCM",
		.cpu_dai_name = "mcrv2-soc-i2s",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",
		.codec_dai_name = "wm8973-hifi",

		.init = ait6366evb_i2s_wm8973_init,
		.codec_name = "wm8973-codec.0-001a",	// means: [codec name-i2c-slave id]
		.ops = &mcrv2_snd_i2s_wm8973_ops,
	}			
   //     .platform_name = "mcrv2-soc-i2s",


//	.codec_name = "mcrv2-i2s-codec",
//	.codec_dai_name = "mcrv2-i2s-codec-dai",		
};

static struct snd_soc_dai_link ait6366evb_i2s_wm8973_dai_link = {
	.name = "MCRV2-I2S WM8973",
	.stream_name = "WM8973 PCM",
	.cpu_dai_name = "mcrv2-soc-i2s",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",
	.codec_name = "wm8973-codec.1-001a",	// means: [codec name-i2c-slave id]
	.codec_dai_name = "wm8973-hifi",
	.init = ait6366evb_i2s_wm8973_init,
 
	.platform_name  = "mcrv2-pcm-audio",
	.ops = &mcrv2_snd_i2s_wm8973_ops,
};

static struct snd_soc_dai_link ait6366evb_i2s_wm8737_dai_link = {
	.name = "MCRV2-I2S WM8737",
	.stream_name = "WM8737 PCM",
	.cpu_dai_name = "mcrv2-soc-i2s",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",
	.codec_name = "wm8737.1-001a",	// means: [codec name-i2c-slave id]
	.codec_dai_name = "wm8737",
	.init = ait6366evb_i2s_wm8737_init,

	.platform_name  = "mcrv2-pcm-audio",
	.ops = &mcrv2_snd_i2s_wm8973_ops,
	//.ops = &mcrv2_snd_i2s_wm8737_ops,
};

#if 1
static struct snd_soc_dai_link ait6366evb_i2s1_dai_link = {
	.name = "MCRV2-I2S 1",
	.stream_name = "HDMI Bridge PCM",
	.cpu_dai_name = "mcrv2-soc-i2s.1",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",
	.codec_name = "i2s-codec-dummy",
	.codec_dai_name = "i2s-codec-dummy-hifi",
	.init = ait6366evb_i2s_wm8973_init,

	.platform_name  = "mcrv2-pcm-audio.1",
	.ops = &mcrv2_snd_i2s_wm8973_ops,
};
#else
static struct snd_soc_dai_link ait6366evb_i2s1_dai_link = {
        .name = "MCRV2-I2S 1",
	.stream_name = "HDMI Bridge PCM",
        .cpu_dai_name = "mcrv2-soc-i2s.1",//"mcrv2-i2s-codec-dai",//"mcrv2-audio-i2s-dai",

	.codec_name = "wm8973-codec.1-001a",	// means: [codec name-i2c-slave id]
	.codec_dai_name = "wm8973-hifi",
	.init = ait6366evb_i2s_wm8973_init,

	.platform_name  = "mcrv2-pcm-audio.1",
        .ops = &mcrv2_snd_i2s_wm8973_ops,
};
#endif

static struct snd_soc_card mcrv2_snd_soc_card0 = {
        .name = "AIT-MCRV2-I2S",
        .owner = THIS_MODULE,
        //.dai_link = &ait6366evb_i2s_wm8737_dai_link,//&mcrv2_snd_soc_i2s_dai,
        .dai_link = &ait6366evb_i2s_wm8973_dai_link,
        .num_links = 1//ARRAY_SIZE(mcrv2_snd_soc_i2s_dai),
};

static struct snd_soc_card mcrv2_snd_soc_card1 = {
        .name = "AIT-MCRV2-I2S",
        .owner = THIS_MODULE,
        //.dai_link = &ait6366evb_i2s_wm8973_dai_link,
        .dai_link = &ait6366evb_i2s1_dai_link,		//From I2S1
        .num_links = 1
};

static struct platform_device *mcrv2_snd_device;

static int __init mcrv2_soc_audio_init(void)
{
        struct snd_soc_card *snd_card_data;
        int index;
        int ret;

        snd_card_data = &mcrv2_snd_soc_card1;
        index = 0;


        mcrv2_snd_device = platform_device_alloc("soc-audio", index);
        if (!mcrv2_snd_device)
                return -ENOMEM;

        platform_set_drvdata(mcrv2_snd_device, snd_card_data);
        ret = platform_device_add(mcrv2_snd_device);
        if (ret)
                platform_device_put(mcrv2_snd_device);

        return ret;
}

static void __exit mcrv2_soc_audio_exit(void)
{
        platform_device_unregister(mcrv2_snd_device);
}

module_init(mcrv2_soc_audio_init);
module_exit(mcrv2_soc_audio_exit);

MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("AIT MCRV2 ASoC driver");
MODULE_LICENSE("GPL");
