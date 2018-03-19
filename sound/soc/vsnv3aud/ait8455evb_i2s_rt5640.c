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

#include "../codecs/rt5640.h"
#include "ait-pcm.h"
#include "ait_ssc_dai.h"

/*
#include <mach/mmp_register.h>
#include <mach/mmp_reg_vif.h>
#include <mach/mmp_reg_audio.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_sd.h>
#include <mach/mmp_reg_spi.h>
#include <mach/mmp_reg_i2cm.h>
#include <mach/mmpf_i2cm.h>
#include <mach/mmpf_vif.h>
#include <mach/mmpf_i2s_ctl.h>
#include <mach/mmpf_pll.h>
#include <mach/mmpf_sd.h>
*/

#if 1 // for ALL

#define MCLK_RATE 12000000
//#define MCLK_RATE 16000000

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
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_LEFT_J|
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
	struct snd_soc_ops ait8455evb_snd_i2s_rt5640_ops = {
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

#if 0
static const struct snd_soc_dapm_route intercon[] = {

	/* speaker connected to LHPOUT */
	{"RT5640 Ext Spk", NULL, "LHPOUT"},

	/* mic is connected to Mic Jack, with WM8731 Mic Bias */
	{"RT5640 MICIN", NULL, "Mic Bias"},
	{"RT5640 Mic Bias", NULL, "Int Mic"},
};
#endif

/*
 * Logic for a rt5627 as connected on a at91sam9g20ek board.
 */
//static 

int at91sam9g20ek_rt5640_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	int N=80,M=8,K=2,SYS_CLK;  // MCLK 12Mhz
	//unsigned int N=78,M=11,K=2,SYS_CLK;  // MCLK 16Mhz

	SYS_CLK = (MCLK_RATE * (N+2)) / ((M+2)*(K+2));
	//SYS_CLK = 24576000;

	printk(KERN_WARNING
			"at91sam9g20ek_rt5640 "
			": at91sam9g20ek_rt5640_init() called\n");

	printk("ERROR: Need implement RT5640 pll control ? \r\n");
#if 1
	//MCLK -> PLL -> SYS
	ret = snd_soc_dai_set_pll(	codec_dai,
								0,						//PLL ID
								RT5640_PLL1_S_MCLK, 	//SOURCE
								MCLK_RATE,				//IN
								SYS_CLK					//OUT
							 );
	ret = snd_soc_dai_set_sysclk(	codec_dai,
									RT5640_SCLK_S_PLL1,
									//RT5640_SCLK_S_MCLK,
									//24576000/2,
									SYS_CLK/2,
									0
							 	 );
	if (ret < 0) {
		printk(KERN_ERR "Failed to set RT5640 PLL: %d\n", ret);
		return ret;
	}
#endif
	return 0;
}

static struct snd_soc_dai_link ait8455evb_i2s_dai_link[]= {
  {
	.name = "RT5640-HIFI-1",
	.stream_name = "RT5640 PCM1",
	.cpu_dai_name = "atmel-ssc-dai.0",
    .init = at91sam9g20ek_rt5640_init,
	.platform_name = "atmel-pcm-audio",
	//.codec_name = "rt5640.1-0038",
	.codec_name = "rt5640.1-001c",
	.codec_dai_name = "rt5640-aif1",
	.ops = &ait8455evb_snd_i2s_rt5640_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
  },
  {
	.name = "RT5640-HIFI-2",

	.stream_name = "RT5640 PCM2",

	.cpu_dai_name = "atmel-ssc-dai.0",

    .init = at91sam9g20ek_rt5640_init,

	.platform_name = "atmel-pcm-audio",

	//.codec_name = "rt5640.1-0038",
	.codec_name = "rt5640.1-001c",

	.codec_dai_name = "rt5640-aif2",

	.ops = &ait8455evb_snd_i2s_rt5640_ops,

	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
  },
};

static struct snd_soc_card snd_card_ait8455evb_i2s = {
	.name = "AIT8455-I2S",
	.dai_link = &ait8455evb_i2s_dai_link,
	.num_links = ARRAY_SIZE(ait8455evb_i2s_dai_link),
	.set_bias_level = ait8455evb_i2s_set_bias_level,
};

static struct platform_device *ait8455_i2s_snd_device;

static int __init ait8455evb_i2s_rt5640_init(void)
{
	int ret;
	printk("%s\n",__FUNCTION__);

	//for test , force PESN enable
	//printk("set PESN Enable\r\n");
	//MMPF_VIF_SetPIOOutput(VIF_SIF_SEN,MMP_TRUE);

#if 1
	ait8455_i2s_snd_device = platform_device_alloc("soc-audio", -1);
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
#endif
	return ret;

err_device_add:
	//platform_device_put(ait8455_i2s_snd_device);
err_mclk:
	clk_put(mclk);
	mclk = NULL;
err:
	return ret;
}

static void __exit ait8455evb_i2s_rt5640_exit(void)
{
	platform_device_unregister(ait8455_i2s_snd_device);
	ait8455_i2s_snd_device = NULL;
	clk_put(mclk);
	mclk = NULL;
}

module_init(ait8455evb_i2s_rt5640_init);
module_exit(ait8455evb_i2s_rt5640_exit);

/* Module information */
MODULE_AUTHOR("JimmyHung");
MODULE_DESCRIPTION("ALSA SoC AIT8455evb_RT5640");
//MODULE_LICENSE("GPL");

#endif //for ALL
