/*
 * atmel_ssc_dai.c  --  ALSA SoC ATMEL SSC Audio Layer Platform driver
 *
 * Copyright (C) 2005 SAN People
 * Copyright (C) 2008 Atmel
 *
 * Author: Sedji Gaouaou <sedji.gaouaou@atmel.com>
 *         ATMEL CORP.
 *
 * Based on at91-ssc.c by
 * Frank Mandarino <fmandarino@endrelia.com>
 * Based on pxa2xx Platform drivers by
 * Liam Girdwood <lrg@slimlogic.co.uk>
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
 //Vin
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>

#include <linux/atmel-ssc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/hardware.h>

#include "ait-pcm.h"
#include "ait_ssc_dai.h"

#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_audio.h>

#include <mach/mmpf_mcrv2_audio_ctl.h>
//#include <mach/mmpf_pll.h>
#include <mach/mmpf_pio.h>


#if defined(CONFIG_ARCH_MCRV2)
#define NUM_SSC_DEVICES		2
//#define NUM_SSC_DEVICES		1
#else
#define NUM_SSC_DEVICES		3
#endif

/*
 * SSC PDC registers required by the PCM DMA engine.
 */
static struct atmel_pdc_regs pdc_tx_reg = {
	.xpr		= ATMEL_PDC_TPR,
	.xcr		= ATMEL_PDC_TCR,
	.xnpr		= ATMEL_PDC_TNPR,
	.xncr		= ATMEL_PDC_TNCR,
};

static struct atmel_pdc_regs pdc_rx_reg = {
	.xpr		= ATMEL_PDC_RPR,
	.xcr		= ATMEL_PDC_RCR,
	.xnpr		= ATMEL_PDC_RNPR,
	.xncr		= ATMEL_PDC_RNCR,
};

/*
 * SSC & PDC status bits for transmit and receive.
 */
static struct atmel_ssc_mask ssc_tx_mask = {
	.ssc_enable	= SSC_BIT(CR_TXEN),		//TX Enable
	.ssc_disable	= SSC_BIT(CR_TXDIS),
	.ssc_endx	= SSC_BIT(SR_ENDTX),
	.ssc_endbuf	= SSC_BIT(SR_TXBUFE),
	.pdc_enable	= ATMEL_PDC_TXTEN,
	.pdc_disable	= ATMEL_PDC_TXTDIS,
};

static struct atmel_ssc_mask ssc_rx_mask = {
	.ssc_enable	= SSC_BIT(CR_RXEN),
	.ssc_disable	= SSC_BIT(CR_RXDIS),
	.ssc_endx	= SSC_BIT(SR_ENDRX),
	.ssc_endbuf	= SSC_BIT(SR_RXBUFF),
	.pdc_enable	= ATMEL_PDC_RXTEN,
	.pdc_disable	= ATMEL_PDC_RXTDIS,
};


/*
 * DMA parameters.
 */
static struct atmel_pcm_dma_params ssc_dma_params[NUM_SSC_DEVICES][2] = {
	{{
	.name		= "SSC0 PCM out",	//I2S FIFO out
	.pdc		= &pdc_tx_reg,
	.mask		= &ssc_tx_mask,
	},
	{
	.name		= "SSC0 PCM in",//AFE FIFO in
	.pdc		= &pdc_rx_reg,
	.mask		= &ssc_rx_mask,
	} },
#if NUM_SSC_DEVICES == 3||NUM_SSC_DEVICES == 2
	{{
	.name		= "Audio AFE out(Unsuuport)",
	.pdc		= &pdc_tx_reg,
	.mask		= &ssc_tx_mask,
	},
	{
	.name		= "SSC1 AFE in",
	.pdc		= &pdc_rx_reg,
	.mask		= &ssc_rx_mask,
	} },
#endif	

};


static struct atmel_ssc_info ssc_info[NUM_SSC_DEVICES] = {
	{
	.name		= "ssc0",
	.lock		= __SPIN_LOCK_UNLOCKED(ssc_info[0].lock),
	.dir_mask	= 0,//SSC_DIR_MASK_UNUSED,
	.initialized	= 0,
	},
#if NUM_SSC_DEVICES == 3||NUM_SSC_DEVICES == 2
	{
	.name		= "ssc1",
	.lock		= __SPIN_LOCK_UNLOCKED(ssc_info[1].lock),
	.dir_mask	= 0,//SSC_DIR_MASK_UNUSED,
	.initialized	= 0,
	},
#endif

};


/*
 * SSC interrupt handler.  Passes PDC interrupts to the DMA
 * interrupt handler in the PCM driver.
 */
static irqreturn_t ait_audio_i2s_interrupt(int irq, void *dev_id)
{
	struct ait_ssc_info *ssc_p = dev_id;
	struct ait_pcm_dma_params *dma_params;
	u32 ssc_sr;
	u32 ssc_substream_mask;
	int i;

	ssc_sr =(unsigned long)((AITPS_I2S)AITC_BASE_I2S0)->I2S_FIFO_SR&(unsigned long)((AITPS_I2S)AITC_BASE_I2S0)->I2S_FIFO_CPU_INT_EN;
		

		//(unsigned long)ssc_readl(ssc_p->ssc->regs, SR)
			//& (unsigned long)ssc_readl(ssc_p->ssc->regs, IMR);
	/*
	 * Loop through the substreams attached to this SSC.  If
	 * a DMA-related interrupt occurred on that substream, call
	 * the DMA interrupt handler function, if one has been
	 * registered in the dma_params structure by the PCM driver.
	 */
	for (i = 0; i < ARRAY_SIZE(ssc_p->dma_params); i++) {
		dma_params = ssc_p->dma_params[i];
		if ((dma_params != NULL) &&
			(dma_params->dma_intr_handler != NULL)) {
			ssc_substream_mask = AUD_INT_FIFO_EMPTY|
								AUD_INT_FIFO_REACH_UNWR_TH ;
			if (ssc_sr & ssc_substream_mask) {
					dma_params->dma_intr_handler(ssc_sr,
							dma_params->substream);
			}
		}
	}



	return IRQ_HANDLED;
}


/*-------------------------------------------------------------------------*\
 * DAI functions
\*-------------------------------------------------------------------------*/
/*
 * Startup.  Only that one substream allowed in each direction.
 */
static int atmel_ssc_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct atmel_ssc_info *ssc_p = &ssc_info[dai->id];
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

	return 0;
}

/*
 * Shutdown.  Clear DMA parameters and shutdown the SSC if there
 * are no other substreams open.
 */
static void atmel_ssc_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct atmel_ssc_info *ssc_p = &ssc_info[dai->id];
	struct atmel_pcm_dma_params *dma_params;
	int dir, dir_mask;
	AITPS_I2S pAUD = AITC_BASE_I2S0;

	
	pr_debug("%s\r\n",__FUNCTION__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	dma_params = ssc_p->dma_params[dir];

	if (dma_params != NULL) {
		//ssc_writel(ssc_p->ssc->regs, CR, dma_params->mask->ssc_disable);
//		pr_debug("atmel_ssc_shutdown: %s disabled SSC_SR=0x%08x\n",
	//		(dir ? "receive" : "transmit"),
		//	0,0));//ssc_readl(ssc_p->ssc->regs, SR));

		dma_params->ssc = NULL;
		dma_params->substream = NULL;
		ssc_p->dma_params[dir] = NULL;
	}

	dir_mask = 1 << dir;

	spin_lock_irq(&ssc_p->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		//MMPF_Audio_PowerDownDAC(MMP_TRUE);
		//TODO: I2S OUT POWER UP
	}
	else
	{
		//MMPF_Audio_PowerDownADC();
		//TODO: I2S IN POWER UP
	}
	
	ssc_p->dir_mask &= ~dir_mask;
	if (!ssc_p->dir_mask) {
		if (--ssc_p->initialized <= 0) {
			/* Shutdown the SSC clock. */
			pr_debug("atmel_ssc_dau: Stopping clock\n");
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
}


/*
 * Record the DAI format for use in hw_params().
 */
static int atmel_ssc_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct atmel_ssc_info *ssc_p = &ssc_info[cpu_dai->id];

	ssc_p->daifmt = fmt;
	return 0;
}

/*
 * Record SSC clock dividers for use in hw_params().
 */
static int atmel_ssc_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
	return 0;
}

/*
 * Configure the SSC.
 */
static int atmel_ssc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	int id = dai->id;
	struct atmel_ssc_info *ssc_p = &ssc_info[id];
	struct atmel_pcm_dma_params *dma_params;
	int dir, channels, bits;
	//u32 tfmr, rfmr, tcmr, rcmr;
	//int start_event;
	int ret;

	//AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_I2S pAUD = AITC_BASE_I2S0;

	pr_debug("%s: para =0x%x \r\n",__func__,params_format(params));
	
	/*
	 * Currently, there is only one set of dma params for
	 * each direction.  If more are added, this code will
	 * have to be changed to select the proper set.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	dma_params = &ssc_dma_params[id][dir];
	dma_params->ssc = ssc_p->ssc;
	dma_params->substream = substream;

	ssc_p->dma_params[dir] = dma_params;

	/*
	 * The snd_soc_pcm_stream->dma_data field is only used to communicate
	 * the appropriate DMA parameters to the pcm driver hw_params()
	 * function.  It should not be used for other purposes
	 * as it is common to all substreams.
	 */
	snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, dma_params);

	channels = params_channels(params);

	/*
	 * Determine sample size in bits and the PDC increment.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		bits = 8;
		dma_params->pdc_xfer_size = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		bits = 16;
		dma_params->pdc_xfer_size = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bits = 24;
		dma_params->pdc_xfer_size = 4;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bits = 32;
		dma_params->pdc_xfer_size = 4;
		break;
	default:
		printk(KERN_WARNING "atmel_ssc_dai: unsupported PCM format");
		return -EINVAL;
	}
	//pAUD->I2S_BIT_CLT = I2S_OUT_32BITS;
	//pAUD->I2S_BIT_ALIGN_OUT = 0;//(pAUD->I2S_BIT_ALIGN_OUT &0xFF00) ;	
	/*
	 * The SSC only supports up to 16-bit samples in I2S format, due
	 * to the size of the Frame Mode Register FSLEN field.
	 */
	if ((ssc_p->daifmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S
		&& bits > 16) {
		printk(KERN_WARNING
				"atmel_ssc_dai: sample size %d "
				"is too large for I2S\n", bits);
		return -EINVAL;
	}

#if 0
	switch (ssc_p->daifmt	& SND_SOC_DAIFMT_FORMAT_MASK)
	{
		case SND_SOC_DAIFMT_I2S:
			pAUD->I2S_OUT_MODE_CTL = I2S_I2S_MODE;//This setting is Bug.
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
		case SND_SOC_DAIFMT_LEFT_J:			
			pAUD->I2S_OUT_MODE_CTL = I2S_STD_MODE;//pAUD->I2S_OUT_MODE_CTL = i2s_mode;			
			break;
		default:
			printk(KERN_WARNING "atmel_ssc_dai: unsupported DAI format 0x%x\n",
				ssc_p->daifmt);
			return -EINVAL;			

	}
//			pAUD->I2S_OUT_MODE_CTL = I2S_STD_MODE;//pAUD->I2S_OUT_MODE_CTL = i2s_mode;
	pAUD->I2S_OUT_MODE_CTL = I2S_STD_MODE;//pAUD->I2S_OUT_MODE_CTL = i2s_mode;			
#endif	
	/*
	 * Compute SSC register settings.
	 */
	switch (ssc_p->daifmt
		& (SND_SOC_DAIFMT_MASTER_MASK)) {

	case SND_SOC_DAIFMT_CBM_CFS:

		pAUD->I2S_MODE_CTL = pAUD->I2S_MODE_CTL|I2S_SLAVE;
		pAUD->I2S_CTL = I2S_SDO_OUT_EN  | I2S_HCK_CLK_EN;			
		/*
		 * I2S format, SSC provides BCLK and LRC clocks.
		 *
		 * The SSC transmit and receive clocks are generated
		 * from the MCK divider, and the BCLK signal
		 * is output on the SSC TK line.
		 */
	
		break;

	case SND_SOC_DAIFMT_CBM_CFM:

		pAUD->I2S_MODE_CTL = pAUD->I2S_MODE_CTL|I2S_MASTER;//MMPF_Audio_SetI2SOutFormat(I2S_MASTER_MODE);
		
		pAUD->I2S_CTL = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN | I2S_HCK_CLK_EN);
		
		/*
		 * I2S format, CODEC supplies BCLK and LRC clocks.
		 *
		 * The SSC transmit clock is obtained from the BCLK signal on
		 * on the TK line, and the SSC receive clock is
		 * generated from the transmit clock.
		 *
		 *  For single channel data, one sample is transferred
		 * on the falling edge of the LRC clock.
		 * For two channel data, one sample is
		 * transferred on both edges of the LRC clock.
		 */
	
		break;

	case SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFM:
	default:
		printk(KERN_WARNING "atmel_ssc_dai: unsupported DAI format 0x%x\n",
			ssc_p->daifmt);
		return -EINVAL;
	}

	pAUD->I2S_DATA_IN_SEL = I2S_SDI_IN;

	pAUD->I2S_DATA_OUT_EN = 0x1;	// Enable serial data output

	pr_debug("atmel_ssc_hw_params: "
				"sample rate = %d\r\n",params_rate(params));

	//snd_soc_dai_set_clkdiv(dai,0,528/132);
	MMPF_PLL_SetAudioPLL(params_rate(params),0,0);

	pAUD->I2S_MODE_CTL = pAUD->I2S_MODE_CTL|I2S_MCLK_OUT_EN;
	
	pAUD->I2S_LRCK_POL = LEFT_CHANNEL_HIGH;

#if 0			
	if (!ssc_p->initialized) {

		/* Enable PMC peripheral clock for this SSC */
		pr_debug("atmel_ssc_dai: Starting clock\n");
		clk_enable(ssc_p->ssc->clk);
		
		ret = request_irq(ssc_p->ssc->irq, ait_audio_i2s_interrupt, 0,
				ssc_p->name, ssc_p);
		if (ret < 0) {
			printk(KERN_WARNING
					"atmel_ssc_dai: request_irq failure\n");
			pr_debug("Atmel_ssc_dai: Stoping clock\n");
			clk_disable(ssc_p->ssc->clk);
			return ret;
		}

		ssc_p->initialized = 1;
	}
#else
	//if (!ssc_p->initialized) {
	  if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	  {
		/* Enable PMC peripheral clock for this SSC */
#define  afe_wb(reg,val) afereg_writeb(ssc_p->ssc->regs,reg,val)
#define  afe_rb(reg) afereg_readb(ssc_p->ssc->regs,reg)

		// 256* 44100	= 11289600
		// 512*44100 	= 22579200
		// 132000000/11289600 = 12
		// 132000000/22579200 = 6
		clk_enable(ssc_p->ssc->clk);
		//MMPF_Audio_SetPLL(0,params_rate(params));
		MMPF_Audio_EnableAFEClock(MMP_TRUE, params_rate(params),AFE_PATH_ADC); 
		MMPF_Audio_SetVoiceInPath(AUDIO_IN_AFE_DIFF);
		//MMPF_Audio_SetADCFreq(params_rate(params));

		MMPF_Audio_InitializeEncodeFIFO( ADC_TO_AFE_FIFO, params_rate(params)*2/1000 ); //// 1ms data 
		//MMPF_Audio_InitializeEncodeFIFO( AFE_IN, params_rate(params)*2/1000 ); //// 1ms data 
		MMPF_Audio_PowerOnADC(params_rate(params));
		MMPF_Audio_SetADCAnalogGain(0x1F,MMP_TRUE);
		//MMPF_Audio_SetADCDigitalGain(0xA0); //move to codec control 
#undef afe_wb
#undef afe_rb
	}
	else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{

#define  afe_wb(reg,val) afereg_writeb(ssc_p->ssc->regs,reg,val)
#define  afe_rb(reg) afereg_readb(ssc_p->ssc->regs,reg)

		// 256* 44100	= 11289600
		// 512*44100 	= 22579200
		// 132000000/11289600 = 12
		// 132000000/22579200 = 6
		clk_enable(ssc_p->ssc->clk);
		//MMPF_Audio_SetPLL(0,params_rate(params));
		//MMPF_Audio_SetVoiceOutPath(AUDIO_OUT_AFE_LINE);
		MMPF_Audio_EnableAFEClock(MMP_TRUE, params_rate(params),AFE_PATH_DAC); 
		MMPF_Audio_SetVoiceOutPath(AUDIO_OUT_AFE_LINE|AUDIO_OUT_AFE_HP);
		//MMPF_Audio_SetDACFreq(params_rate(params));

		//afereg_writeb(ssc_p->ssc->regs,AFE_ADDA_BIT_MOD, 0x01); //Set DAC fifo 20bit mode
		//sean@DEL
		//afereg_writeb(ssc_p->ssc->regs,AFE_DATA_BIT_TRANS, 0x4);
		//PF_Audio_InitializeEncodeFIFO( ADC_TO_AFE_FIFO, 0x3FF ); //// 1ms data 		
		MMPF_Audio_InitializePlayFIFO(AFE_FIFO_TO_DAC, params_rate(params)*2/200 ); //set 5 ms buffer 
		//MMPF_Audio_InitializePlayFIFO(AFE_OUT, params_rate(params)*2/200 ); //set 5 ms buffer 

		MMPF_Audio_PowerOnDAC(params_rate(params));

              // Power ON AMP
              MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO97, MMP_TRUE,0);
              MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO97, MMP_TRUE, MMP_FALSE);
              MMPF_PIO_SetData(MMPF_PIO_REG_GPIO97, MMP_TRUE, MMP_FALSE);
				
		MMPF_Audio_SetDACAnalogGain(0x00);
		//MMPF_Audio_SetDACDigitalGain(67); //move to codec control 
#undef afe_wb
#undef afe_rb
	
	}

	if(!ssc_p->initialized)
	{
		pr_info("ssc irq = 0x%x\r\n",ssc_p->ssc->irq);
		ret = request_irq(ssc_p->ssc->irq, ait_afe_interrupt, 0,ssc_p->name, ssc_p);
		if (ret < 0) {
			printk(KERN_WARNING
				"atmel_ssc_dai: request_irq failure\n");
				pr_debug("Atmel_ssc_dai: Stoping clock\n");
				clk_disable(ssc_p->ssc->clk);
			return ret;
		}
		//ssc_p->initialized = 1;
		ssc_p->initialized ++;
	}

#endif
	pr_debug("atmel_ssc_dai,hw_params: SSC initialized\n");
	return 0;
}


static int atmel_ssc_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct atmel_ssc_info *ssc_p = &ssc_info[dai->id];
	struct atmel_pcm_dma_params *dma_params;
	int dir;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	dma_params = ssc_p->dma_params[dir];

	return 0;
}


#ifdef CONFIG_PM
static int atmel_ssc_suspend(struct snd_soc_dai *cpu_dai)
{
	struct atmel_ssc_info *ssc_p;

	if (!cpu_dai->active)
		return 0;

	ssc_p = &ssc_info[cpu_dai->id];

	return 0;
}



static int atmel_ssc_resume(struct snd_soc_dai *cpu_dai)
{
	struct atmel_ssc_info *ssc_p;

	if (!cpu_dai->active)
		return 0;

	ssc_p = &ssc_info[cpu_dai->id];

	return 0;
}
#else /* CONFIG_PM */
#  define atmel_ssc_suspend	NULL
#  define atmel_ssc_resume	NULL
#endif /* CONFIG_PM */

static int atmel_ssc_probe(struct snd_soc_dai *dai)
{
	struct atmel_ssc_info *ssc_p = &ssc_info[dai->id];
	int ret = 0;

	snd_soc_dai_set_drvdata(dai, ssc_p);

	/*
	 * Request SSC device
	 */
	ssc_p->ssc = ssc_request(dai->id);
	if (IS_ERR(ssc_p->ssc)) {
		printk(KERN_ERR "ASoC: Failed to request SSC %d\n", dai->id);
		ret = PTR_ERR(ssc_p->ssc);
	}

	return ret;
}

static int atmel_ssc_remove(struct snd_soc_dai *dai)
{
	struct atmel_ssc_info *ssc_p = snd_soc_dai_get_drvdata(dai);

	ssc_free(ssc_p->ssc);
	return 0;
}
#if 0
#define ATMEL_SSC_RATES (SNDRV_PCM_RATE_8000_96000)


#define ATMEL_SSC_FORMATS (SNDRV_PCM_FMTBIT_S8     | SNDRV_PCM_FMTBIT_S16_LE |\
			  SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
#endif
#define AIT_AUDIO_I2S_RATES SNDRV_PCM_RATE_8000_48000//(SNDRV_PCM_RATE_16000)
#define AIT_AUDIO_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE )


static struct snd_soc_dai_ops atmel_ssc_dai_ops = {
	.startup	= atmel_ssc_startup,
	.shutdown	= atmel_ssc_shutdown,
	.prepare	= atmel_ssc_prepare,
	.hw_params	= atmel_ssc_hw_params,
	.set_fmt	= atmel_ssc_set_dai_fmt,
	.set_clkdiv	= atmel_ssc_set_dai_clkdiv,
};

static struct snd_soc_dai_driver atmel_ssc_dai[NUM_SSC_DEVICES] = {
	{
		.name = "atmel-ssc-dai.0",
		.probe = atmel_ssc_probe,
		.remove = atmel_ssc_remove,
		.suspend = atmel_ssc_suspend,
		.resume = atmel_ssc_resume,
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = AIT_AUDIO_I2S_RATES,
			.formats = AIT_AUDIO_I2S_FORMATS,},
#if 0
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = ATMEL_SSC_RATES,
			.formats = ATMEL_SSC_FORMATS,},
#endif					
		.ops = &atmel_ssc_dai_ops,

	},
#if NUM_SSC_DEVICES == 2 || NUM_SSC_DEVICES == 3
	{
		.name = "atmel-ssc-dai.1",
		.probe = atmel_ssc_probe,
		.remove = atmel_ssc_remove,
		.suspend = atmel_ssc_suspend,
		.resume = atmel_ssc_resume,
#if 0		
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = ATMEL_SSC_RATES,
			.formats = ATMEL_SSC_FORMATS,},
#endif			
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_11025|SNDRV_PCM_RATE_16000,//ATMEL_SSC_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE},
		.ops = &atmel_ssc_dai_ops,
	},
#endif

};

static __devinit int asoc_ssc_probe(struct platform_device *pdev)
{

	BUG_ON(pdev->id < 0);
	BUG_ON(pdev->id >= ARRAY_SIZE(atmel_ssc_dai));
	return snd_soc_register_dai(&pdev->dev, &atmel_ssc_dai[pdev->id]);
}

static int __devexit asoc_ssc_remove(struct platform_device *pdev)
{

	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver asoc_ssc_driver = {
	.driver = {
			.name = "atmel-ssc-dai",
			.owner = THIS_MODULE,
	},

	.probe = asoc_ssc_probe,
	.remove = __devexit_p(asoc_ssc_remove),
};

/**
 * atmel_ssc_set_audio - Allocate the specified SSC for audio use.
 */
int atmel_ssc_set_audio(int ssc_id)
{
	struct ssc_device *ssc;
	static struct platform_device *dma_pdev,*vsnv3_dma_pdev;
	struct platform_device *ssc_pdev;
	int ret;
	pr_debug("%s\r\n",__FUNCTION__);

	if (ssc_id < 0 || ssc_id >= ARRAY_SIZE(atmel_ssc_dai))
		return -EINVAL;

	/* Allocate a dummy device for DMA if we don't have one already */
	if (!dma_pdev) {
		dma_pdev = platform_device_alloc("atmel-pcm-audio", -1);
		if (!dma_pdev)
			return -ENOMEM;

		ret = platform_device_add(dma_pdev);
		if (ret < 0) {
			platform_device_put(dma_pdev);
			dma_pdev = NULL;
			return ret;
		}
	}

	if(ssc_id==1 && !vsnv3_dma_pdev)
	{
		vsnv3_dma_pdev = platform_device_alloc("vsnv3afe-pcm-audio", -1);
		if (!vsnv3_dma_pdev)
			return -ENOMEM;

		ret = platform_device_add(vsnv3_dma_pdev);
		if (ret < 0) {
			platform_device_put(vsnv3_dma_pdev);
			vsnv3_dma_pdev = NULL;
			return ret;
		}
	}
	
	ssc_pdev = platform_device_alloc("atmel-ssc-dai", ssc_id);
	if (!ssc_pdev)
		return -ENOMEM;

	/* If we can grab the SSC briefly to parent the DAI device off it */
	ssc = ssc_request(ssc_id);
	if (IS_ERR(ssc))
		pr_warn("Unable to parent ASoC SSC DAI on SSC: %ld\n",
			PTR_ERR(ssc));
	else {
		ssc_pdev->dev.parent = &(ssc->pdev->dev);
		ssc_free(ssc);
	}

	ret = platform_device_add(ssc_pdev);
	if (ret < 0)
	{
		pr_err( "%s: %d ret = %d\n",__FUNCTION__,__LINE__,ret);
		platform_device_put(ssc_pdev);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(atmel_ssc_set_audio);

static int __init snd_atmel_ssc_init(void)
{
	return platform_driver_register(&asoc_ssc_driver);
}
module_init(snd_atmel_ssc_init);

static void __exit snd_atmel_ssc_exit(void)
{
	platform_driver_unregister(&asoc_ssc_driver);
}
module_exit(snd_atmel_ssc_exit);

/* Module information */
MODULE_AUTHOR("Sedji Gaouaou, sedji.gaouaou@atmel.com, www.atmel.com");
MODULE_DESCRIPTION("ATMEL SSC ASoC Interface");
MODULE_LICENSE("GPL");
