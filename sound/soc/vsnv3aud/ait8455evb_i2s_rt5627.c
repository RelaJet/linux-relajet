#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <linux/atmel-ssc.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "../codecs/rt5627.h"
#include "ait-pcm.h"
#include "ait_ssc_dai.h"

#define MCLK_RATE 12000000

/*
 * As shipped the board does not have inputs.  However, it is relatively
 * straightforward to modify the board to hook them up so support is left
 * in the driver.
 */
#undef ENABLE_MIC_INPUT

static struct clk *mclk;

static int ait8455evb_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);//SND_SOC_DAIFMT_CBS_CFS);//);

	if (ret < 0)
	{
		printk(KERN_ERR "snd_soc_dai_set_fmt(%s):%d\n",codec_dai->name,ret);
		return ret;
	}
	else
	{
		printk(KERN_DEBUG "%s:snd_soc_dai_set_fmt(%s) OK\n",__FUNCTION__,codec_dai->name);
	}
	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_LEFT_J  |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFS);//);
	if (ret < 0)
	{
			printk(KERN_ERR "%s: snd_soc_dai_set_fmt(%s):%d\n",__FUNCTION__,cpu_dai->name,ret);
		return ret;
	}
	else
		printk(KERN_DEBUG "%s: snd_soc_dai_set_fmt(%s) OK\n",__FUNCTION__,cpu_dai->name);
	return 0;
}

//static 
	struct snd_soc_ops ait8455evb_snd_i2s_rt5627_ops = {
	.hw_params = ait8455evb_hw_params,
};

static int ait8455evb_i2s_set_bias_level(struct snd_soc_card *card,
					struct snd_soc_dapm_context *dapm,
					enum snd_soc_bias_level level)
{
	static int mclk_on;
	int ret = 0;
	pr_debug("level = %d\n",level);
#if 0
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		if (!mclk_on)
			ret = clk_enable(mclk);
		if (ret == 0)
			mclk_on = 1;
		break;

	case SND_SOC_BIAS_OFF:
	case SND_SOC_BIAS_STANDBY:
		if (mclk_on)
			clk_disable(mclk);
		mclk_on = 0;
		break;
	}
#endif
//Vin: add
	mclk_on = 1;
	return ret;
}

static const struct snd_soc_dapm_route intercon[] = {

	/* speaker connected to LHPOUT */
	{"Ext Spk", NULL, "LHPOUT"},

	/* mic is connected to Mic Jack, with WM8731 Mic Bias */
	{"MICIN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
};

/*
 * Logic for a rt5627 as connected on a at91sam9g20ek board.
 */
static int at91sam9g20ek_rt5627_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	printk(KERN_WARNING
			"at91sam9g20ek_rt5627 "
			": at91sam9g20ek_rt5627_init() called\n");


	ret = snd_soc_dai_set_pll(codec_dai, RT5627_PLL_FR_MCLK, 0, MCLK_RATE, 16384000);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set ALC5627 PLL: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_link ait8455evb_i2s_dai_link= {
	.name = "RT5627",

	.stream_name = "RT5627 PCM",

	.cpu_dai_name = "atmel-ssc-dai.0",

    .init = at91sam9g20ek_rt5627_init,

	.platform_name = "atmel-pcm-audio",
	
	.codec_name = "rt5627-codec.1-0019",

	.codec_dai_name = "rt5627_hifi",
	
	.ops = &ait8455evb_snd_i2s_rt5627_ops,
};

static struct snd_soc_card snd_card_ait8455evb_i2s = {
	.name = "AIT8455-I2S",
	.dai_link = &ait8455evb_i2s_dai_link,
	.num_links = 1,
	.set_bias_level = ait8455evb_i2s_set_bias_level,
};

static struct platform_device *ait8455_i2s_snd_device;

static int __init ait8455evb_i2s_rt5627_init(void)
{
	struct clk *pllb;
	int ret;
	pr_debug( "%s\n",__FUNCTION__);

	ret = atmel_ssc_set_audio(0);
	if (ret != 0) {
		pr_err("Failed to set SSC 0 for audio: %d\n", ret);
		return ret;
	}
	ait8455_i2s_snd_device = platform_device_alloc("soc-audio", 0);
	if (!ait8455_i2s_snd_device) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		ret = -ENOMEM;
		goto err_mclk;
	}

	platform_set_drvdata(ait8455_i2s_snd_device,
			&snd_card_ait8455evb_i2s);

	ret = platform_device_add(ait8455_i2s_snd_device);
	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		goto err_device_add;
	}

	return ret;

err_device_add:
	platform_device_put(ait8455_i2s_snd_device);
err_mclk:
	clk_put(mclk);
	mclk = NULL;
err:
	return ret;
}

static void __exit ait8455evb_i2s_rt5627_exit(void)
{
	platform_device_unregister(ait8455_i2s_snd_device);
	ait8455_i2s_snd_device = NULL;
	clk_put(mclk);
	mclk = NULL;
}

module_init(ait8455evb_i2s_rt5627_init);
module_exit(ait8455evb_i2s_rt5627_exit);

/* Module information */
MODULE_AUTHOR("JimmyHung");
MODULE_DESCRIPTION("ALSA SoC AIT8455evb_RT5627");
//MODULE_LICENSE("GPL");
