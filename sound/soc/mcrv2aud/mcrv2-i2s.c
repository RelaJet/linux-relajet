/*
 * mcrv2_i2s.c  --  ALSA SoC AIT I2S Audio Layer Platform driver
 *
 * Copyright (C) 2015 AIT
 *
 * Author: Vincent Chen<vincent_chen@a-i-t.com.tw>
 *         AIT CORP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
//#include <linux/atmel_pdc.h>

//#include <linux/atmel-ssc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/hardware.h>
#include <mach/mmp_reg_audio.h>
#include <mach/mmpf_pll.h>

#include "ait-pcm.h"
#include "ait_aud_dai.h"


static irqreturn_t mcrv2_i2s_interrupt(int irq, void *dev_id)
{
        struct mcrv2_soc_i2s_dev *dev = (struct mcrv2_soc_i2s_dev *) dev_id;
	struct mcrv2_pcm_dma_params *dma_param;
	int i;
	u32 i2s_sr,i2s_fifo_sr,i2s_mask;
	AITPS_I2S pI2S = dev->regbase; 

	{
	
		i2s_fifo_sr = 	mcrv2_i2s_reg_readb(dev, I2S_REG_OFFSET(I2S_FIFO_SR.MP.RX_SR));	
		i2s_sr = 	(AITC_BASE_I2S0)->I2S_FIFO_INT_SR; //mcrv2_i2s_reg_readb(dev, I2S_REG_OFFSET(I2S_FIFO_INT_SR));	
		
		/*-DEFINE-----------------------------------------------------*/
		if(i2s_sr&I2S_FIFO_SRC_CH0)
		{
			dma_param = &dev->dma_params[SNDRV_PCM_STREAM_CAPTURE];
			//pr_info("dma_param= %x\n",dma_param);
			//pr_info("dma_param->substream= %x\n",dma_param->substream);
			dma_param->dma_intr_handler(i2s_fifo_sr,dma_param->substream);

			AITC_BASE_I2S0->I2S_FIFO_INT_SR = I2S_FIFO_SRC_CH0;
			return IRQ_HANDLED;
		}else if(i2s_sr&I2S_FIFO_SRC_CH1)
		{

			dma_param = &dev->dma_params[SNDRV_PCM_STREAM_CAPTURE];
			//pr_info("dma_param= %x\n",dma_param);
			//pr_info("dma_param->substream= %x\n",dma_param->substream);
			dma_param->dma_intr_handler(i2s_fifo_sr,dma_param->substream);

			AITC_BASE_I2S0->I2S_FIFO_INT_SR = I2S_FIFO_SRC_CH1;
			return IRQ_HANDLED;
		}else if(i2s_sr&I2S_FIFO_SRC_CH2)
		{
			dma_param = &dev->dma_params[SNDRV_PCM_STREAM_CAPTURE];
			dma_param->dma_intr_handler(i2s_fifo_sr,dma_param->substream);

			AITC_BASE_I2S0->I2S_FIFO_INT_SR = I2S_FIFO_SRC_CH2;
			return IRQ_HANDLED;
		}
	}
	return IRQ_NONE;
}



static int mcrv2_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,  unsigned int fmt)
{
	
        struct mcrv2_soc_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
        u8 modectrl, clkctrl, bit_align=0;
	
        switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	        case SND_SOC_DAIFMT_CBM_CFM:
	              modectrl = I2S_MASTER;
			clkctrl = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN );
				  
	              break;
	        case SND_SOC_DAIFMT_CBS_CFM:
			modectrl = I2S_MASTER;
			clkctrl = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN );			
			break;
	        case SND_SOC_DAIFMT_CBM_CFS:
			modectrl = I2S_SLAVE;
			clkctrl = I2S_SDO_OUT_EN  ;			
					
			break;
	        case SND_SOC_DAIFMT_CBS_CFS:
	                modectrl = I2S_SLAVE;
	                break;
	        default:
	                return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		bit_align = 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		bit_align = 1;
		
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		bit_align = 1;

		break;
	case SND_SOC_DAIFMT_DSP_A:

		break;
	case SND_SOC_DAIFMT_DSP_B:

		break;
	default:
		return -EINVAL;
	}

	mcrv2_i2s_reg_writeb(dev, I2S_REG_OFFSET(I2S_BIT_ALIGN_IN), bit_align);
	
	mcrv2_i2s_reg_writeb(dev, I2S_REG_OFFSET(I2S_CTL), clkctrl|I2S_HCK_CLK_EN);
	mcrv2_i2s_reg_writeb(dev, I2S_REG_OFFSET(I2S_MODE_CTL), modectrl|I2S_MCLK_OUT_EN);	
	mcrv2_i2s_reg_writeb(dev, I2S_REG_OFFSET(I2S_MUX_MODE_CTL), AUD_MUX_AUTO);

        return 0;

}

static int mcrv2_i2s_dai_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	int dir_mask;
	
	struct mcrv2_soc_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
pr_info("mcrv2_i2s_dai_startup dev=%x\n",dev);
pr_info("mcrv2_i2s_dai_startup &dev->dma_params[substream->stream ]=%x\n",&dev->dma_params[substream->stream ]);



	
	snd_soc_dai_set_dma_data(dai, substream, &dev->dma_params[substream->stream ]);


	
#if 0
	ait_afe_info *ssc_p = &ssc_info[dai->id];
	int dir_mask;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{		
		dir_mask = SSC_DIR_MASK_PLAYBACK;
	}
	else
	{
		pr_debug("Start capture\n");
		dir_mask = SSC_DIR_MASK_CAPTURE;
	}

	spin_lock_irq(&ssc_p->lock);
	if (ssc_p->dir_mask & dir_mask) {
		spin_unlock_irq(&ssc_p->lock);
		return -EBUSY;
	}
	ssc_p->dir_mask |= dir_mask;
	spin_unlock_irq(&ssc_p->lock);
#endif
	return 0;
}

/*
 * Shutdown.  Clear DMA parameters and shutdown the SSC if there
 * are no other substreams open.
 */
static void mcrv2_i2s_dai_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
#if 0
	ait_afe_info *ssc_p = &ssc_info[dai->id];
	struct ait_pcm_dma_params *dma_params;
	int dir, dir_mask;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	dma_params = ssc_p->dma_params[dir];

	if (dma_params != NULL) {
		dma_params->ssc = NULL;
		dma_params->substream = NULL;
		ssc_p->dma_params[dir] = NULL;
	}

	dir_mask = 1 << dir;

	spin_lock_irq(&ssc_p->lock);

	//if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	//	MMPF_Audio_PowerDownDAC(MMP_TRUE);
	//else
	//	MMPF_Audio_PowerDownADC();
	
	ssc_p->dir_mask &= ~dir_mask;
	if (!ssc_p->dir_mask) {
		//if (ssc_p->initialized) {
		if (--ssc_p->initialized <= 0) {
			/* Shutdown the SSC clock. */
			pr_debug("ait_ssc_dai: Stopping clock\n");
			clk_disable(ssc_p->ssc->clk);

			free_irq(ssc_p->ssc->irq, ssc_p);
			ssc_p->initialized = 0;
		}

		/* Reset the SSC */
		//ssc_writel(ssc_p->ssc->regs, CR, SSC_BIT(CR_SWRST));
		/* Clear the SSC dividers */
		ssc_p->cmr_div = ssc_p->tcmr_period = ssc_p->rcmr_period = 0;
	}
	spin_unlock_irq(&ssc_p->lock);
#endif	
}

static int mcrv2_i2s_set_dai_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
#define PLL_OUT_DEVIDER 0
#define SOC_I2S_MCLK_DEVIDER 1
	struct mcrv2_soc_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);


	AITPS_I2S pAUD = dev->regbase;
	AITPS_GBL pGBL = AITC_BASE_GBL;


	switch(div_id)
	{

		case PLL_OUT_DEVIDER:
			pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(div);

			break;

		case SOC_I2S_MCLK_DEVIDER:
			pAUD->I2S_CLK_DIV = div;

			break;


	}
	return 0;
}

static int mcrv2_i2s_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{

	int id = dai->id;
	int  channels, bits;
	int ret;
	int clkdiv;
	struct mcrv2_soc_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	AITPS_I2S pI2S = dev->regbase; //AITC_BASE_I2S0;

	//snd_soc_dai_set_clkdiv(dai,0,528/132);
	MMPF_PLL_SetAudioPLL_I2S(freq);


	pI2S->I2S_MODE_CTL = pI2S->I2S_MODE_CTL|I2S_MCLK_OUT_EN;

	pI2S->I2S_CLK_CTL |= I2S_MCLK_FIX;

	//clkdiv = 96000000/freq;
	//pAUD->I2S_CLK_DIV = clkdiv;//clkDiv;		//	48M
	//pAUD->I2S_RATIO_N_M =  ratioM << 8 | ratioN;
	//pAUD->I2S_MCLK_CTL = I2S_256_FS;

	//pAUD->I2S_LRCK_POL = I2S_LRCK_L_CH_HIGH;
	pI2S->I2S_MCLK_CTL = I2S_256_FS;
	
	//MMPF_I2S_PADSET(0, 0);

	pr_debug("atmel_ssc_dai,hw_params: SSC initialized\n");

	return 0;
}

static int ait_i2s_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	//struct ait_i2s_info *ssc_p = &ssc_info[dai->id];
	struct ait_pcm_dma_params *dma_params;
	int dir;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	//dma_params = ssc_p->dma_params[dir];

	return 0;
}

static int ait_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{

	struct mcrv2_soc_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	AITPS_I2S pI2S = dev->regbase;
	AIT_REG_B param_bit_width,i2s_bit_width;

	if (params_channels(params) == 2)
		{}

	param_bit_width = snd_pcm_format_width(params_format(params));

		
	switch(param_bit_width)
	{
		case 16:
			i2s_bit_width = I2S_OUT_16BITS;                                            // 0x34
			break;
		case 24:
			i2s_bit_width = I2S_OUT_24BITS;
			break;
		case 32:
			i2s_bit_width = I2S_OUT_32BITS;
			break;
		default:

			return -EINVAL;

			break;
	}
	pI2S->I2S_BIT_CLT = i2s_bit_width;	

	return 0;
}

static struct snd_soc_dai_ops mcrv2_i2s_dai_ops = {
	.startup	= mcrv2_i2s_dai_startup,
	.shutdown	= mcrv2_i2s_dai_shutdown,
	
	.set_fmt	= mcrv2_i2s_set_dai_fmt,	
	.set_clkdiv	= mcrv2_i2s_set_dai_clkdiv,
	.set_sysclk	= mcrv2_i2s_set_dai_sysclk,	

	.prepare	= ait_i2s_prepare,
	.hw_params	= ait_i2s_hw_params,
		
};


static int mcrv2_i2s_dai_probe(struct snd_soc_dai *dai)
{
	int ret =0;
	return ret;
}

static int mcrv2_i2s_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static int mcrv2_i2s_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int mcrv2_i2s_resume(struct snd_soc_dai *dai)
{
	return 0;
}
#define AIT_I2S_RATES SNDRV_PCM_RATE_8000_48000
#define AIT_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE )



static struct snd_soc_dai_driver mcrv2_i2s_dai= {

		.name = "mcrv2-i2s-soc-dai",
		.probe = mcrv2_i2s_dai_probe,
		.remove = mcrv2_i2s_remove,
		.suspend = mcrv2_i2s_suspend,
		.resume = mcrv2_i2s_resume,
		
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIT_I2S_RATES,
			.formats = AIT_I2S_FORMATS},
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIT_I2S_RATES,
			.formats = AIT_I2S_FORMATS},
		.ops = &mcrv2_i2s_dai_ops,


};

static struct platform_device *mcrv2_i2s_pcm_device;


static int mcrv2_soc_i2s_probe(struct platform_device *pdev)
{
        struct mcrv2_soc_i2s_dev *dev;
        struct resource *mem, *region, *res;
        u8 __iomem *mmio;
        int ret;

        mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!mem) {
                dev_err(&pdev->dev, "no mem resource?\n");
                ret = -ENODEV;
                goto err_release_none;
        }

        region = request_mem_region(mem->start, resource_size(mem),pdev->name);
        if (!region) {
                dev_err(&pdev->dev, "I2S region already claimed\n");
                ret = -EBUSY;
                goto err_release_none;
        }

         mmio = ioremap(mem->start, resource_size(mem));
         if (!mmio) {
                dev_err(&pdev->dev, "can't ioremap mem region\n");
                ret = -ENOMEM;
                goto err_release_region;
        }


        dev = kzalloc(sizeof(struct mcrv2_soc_i2s_dev), GFP_KERNEL);
        if (!dev) {
                ret = -ENOMEM;
                goto err_release_map;
        }
        dev_set_drvdata(&pdev->dev, dev);

	dev->regbase = mmio;
#if 1
	dev->clk = clk_get(&pdev->dev, "clk_audio_i2s");
	if (IS_ERR(dev->clk)) {
		dev_dbg(&pdev->dev, "no clk_audio_i2s clock defined\n");
		ret = -ENXIO;
		goto err_release_map;
	}
#endif
	clk_enable(dev->clk);


	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "failed to acquire irq resource\n");
		ret = -ENOENT;
		goto err_release_clk;
	}
	dev->irq = res->start;

	ret = request_irq(dev->irq, mcrv2_i2s_interrupt, 0,"mcrv2 I2S irq", dev);
	if (ret < 0) {
		printk(KERN_WARNING "request irq %d failure(%d)\n",dev->irq,ret);
			pr_debug("Stoping clock\n");
		//	clk_disable(dev->clk);
		ret = -ENOENT;
		goto err_release_clk;
	}

	dev->ch = pdev->id;
	
	MMPF_I2S_PADSET(dev->ch , 0); //dev->ch = 1 means I2S1_0
	
        ret = snd_soc_register_dai(&pdev->dev, &mcrv2_i2s_dai);
        if (ret)
                goto err_release_dev;

        mcrv2_i2s_pcm_device = platform_device_alloc("mcrv2-pcm-audio", dev->ch);
        if (!mcrv2_i2s_pcm_device)
        {
                ret =  -ENOMEM;
                goto err_release_dai;
        }
pr_info("i2s probe dev=%x\n",dev);
        platform_set_drvdata(mcrv2_i2s_pcm_device, dev);
        ret = platform_device_add(mcrv2_i2s_pcm_device);
        if (ret)
        {
                ret =  -ENOMEM;
		goto err_release_pcm_dev;
        }
        return 0;

err_release_pcm_dev:
	platform_device_put(mcrv2_i2s_pcm_device);

err_release_dai:
	snd_soc_unregister_dai(&pdev->dev);
err_release_dev:
        kfree(dev);
err_release_irq:
	free_irq(dev->irq, dev);	
err_release_clk:
	clk_disable(dev->clk);	
err_release_map:
        iounmap(mmio);
err_release_region:
        release_mem_region(mem->start, resource_size(mem));
err_release_none:
        return ret;
}

static int __devexit mcrv2_soc_i2s_remove(struct platform_device *pdev)
{
        struct mcrv2_soc_i2s_dev *dev = dev_get_drvdata(&pdev->dev);
        struct resource *region;
        void __iomem *mmio = dev->regbase;
	snd_soc_unregister_dai(&pdev->dev);
	kfree(dev);

        region = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        release_mem_region(region->start, resource_size(region));

        iounmap(mmio);
	return 0;
}

static struct platform_driver mcrv2_soc_i2s_driver = {
         .probe          = mcrv2_soc_i2s_probe,
         .remove         = mcrv2_soc_i2s_remove,
         .driver         = {
                 .name   = "mcrv2-soc-i2s",
                 .owner  = THIS_MODULE,
         },
};


module_platform_driver(mcrv2_soc_i2s_driver);

MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("AIT MCRV2 I2S SoC Interface");
MODULE_LICENSE("GPL");
