/*
 * ait_i2s_dai.c  --  ALSA SoC AIT SSC Audio Layer Platform driver
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
#include "ait_aud_dai.h"
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_audio.h>

#include <mach/mmpf_mcrv2_audio_ctl.h>
#include <mach/mmpf_mcrv2_i2s_ctl.h>
#include <mach/mmpf_i2s_ctl.h>
#include <mach/mmpf_pio.h>


#if defined(CONFIG_ARCH_MCRV2)
//#define NUM_SSC_DEVICES		2
#define NUM_SSC_DEVICES		1
#else
#define NUM_SSC_DEVICES		3
#endif


/*
 * SSC PDC registers required by the PCM DMA engine.
 */
static struct atmel_pdc_regs pdc_tx_reg = {
	.xpr			= 0,//ATMEL_PDC_TPR,
	.xcr			= 0,//ATMEL_PDC_TCR,
	.xnpr		= 0,//ATMEL_PDC_TNPR,
	.xncr		= 0,//ATMEL_PDC_TNCR,
};

static struct atmel_pdc_regs pdc_rx_reg = {
	.xpr			= 0,//ATMEL_PDC_RPR,
	.xcr			= 0,//ATMEL_PDC_RCR,
	.xnpr		= 0,//ATMEL_PDC_RNPR,
	.xncr		= 0,//ATMEL_PDC_RNCR,
};


#if 0
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
#endif 

/*
 * DMA parameters.
 */
static struct ait_pcm_dma_params ssc_dma_params[NUM_SSC_DEVICES][2] = {
	{{
	.name		= "SSC0 PCM out",	//AFE FIFO out
	.pdc		= &pdc_tx_reg,
	//.mask		= &ssc_tx_mask,
	},
	{
	.name		= "SSC0 PCM in",//AFE FIFO in
	.pdc		= &pdc_rx_reg,
	//.mask		= &ssc_rx_mask,
	} },
#if NUM_SSC_DEVICES == 3||NUM_SSC_DEVICES == 2
	{{
	.name		= "Audio AFE out(Unsuuport)",
	.pdc		= &pdc_tx_reg,
	//.mask		= &ssc_tx_mask,
	},
	{
	.name		= "SSC1 AFE in",
	.pdc		= &pdc_rx_reg,
	//.mask		= &ssc_rx_mask,
	} },
#endif	

};


static ait_afe_info ssc_info[NUM_SSC_DEVICES] = {
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
static irqreturn_t ait_i2s_interrupt(int irq, void *dev_id)
{
	ait_afe_info *ssc_p = dev_id;
	struct ait_pcm_dma_params *dma_params;
	int i;
	u32 i2s_sr,i2s_mask;
	
	//if(ssc_p->ssc->regs==AITC_BASE_AFE)
	if(ssc_p->ssc->regs)
	{
        //(unsigned long)ssc_readl(ssc_p->ssc->regs, SR)
            //& (unsigned long)ssc_readl(ssc_p->ssc->regs, IMR);
		/*
		 * Loop through the substreams attached to this SSC.  If
		 * a DMA-related interrupt occurred on that substream, call
		 * the DMA interrupt handler function, if one has been
		 * registered in the dma_params structure by the PCM driver.
		 */
		for (i = 0; i < ARRAY_SIZE(ssc_p->dma_params); i++)
		{
			dma_params = ssc_p->dma_params[i];
			if ((dma_params != NULL) &&(dma_params->dma_intr_handler != NULL))
			{
				int dir = dma_params->substream->stream;
				if(dir==SNDRV_PCM_STREAM_CAPTURE)
				{
					i2s_sr = i2s_reg(ssc_p->ssc->regs,I2S_FIFO_CPU_INT_EN) & 0x0F;
					i2s_mask = i2s_fifo_status(ssc_p->ssc->regs,RX_SR) & 0x0F;
				}else{
					i2s_sr = i2s_reg(ssc_p->ssc->regs,I2S_FIFO_CPU_INT_EN) & 0x0F;
					i2s_mask = i2s_fifo_status(ssc_p->ssc->regs,TX_SR) & 0x0F;
				}
				//i2s_sr = i2s_sr&i2s_mask;
				
				if (i2s_sr & i2s_mask )
				{
					u32 sr = 0;
					if(i2s_sr & AUD_INT_FIFO_REACH_UNWR_TH)
						sr |= FIFO_REACH_UNWR_TH;

					if(i2s_sr & AUD_INT_FIFO_REACH_UNRD_TH)
						sr |= FIFO_REACH_UNRD_TH;

					if(i2s_sr & AUD_INT_FIFO_FULL)
						sr |= FIFO_FULL;
					
					if(i2s_sr & AUD_INT_FIFO_EMPTY)
						sr |= FIFO_EMPTY;
					
					dma_params->dma_intr_handler(i2s_sr,dma_params->substream);
				}				
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
static int ait_i2s_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
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

	return 0;
}

/*
 * Shutdown.  Clear DMA parameters and shutdown the SSC if there
 * are no other substreams open.
 */
static void ait_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
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
}


/*
 * Record the DAI format for use in hw_params().
 */
static int ait_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	ait_afe_info *ssc_p = &ssc_info[cpu_dai->id];
	ssc_p->daifmt = fmt;
	return 0;
}

/*
 * Record SSC clock dividers for use in hw_params().
 */
static int ait_i2s_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
//	struct atmel_ssc_info *ssc_p = &ssc_info[cpu_dai->id];

	return 0;
}

/*
 * Configure the SSC.
 */
static int ait_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	int id = dai->id;
	ait_afe_info *ssc_p = &ssc_info[id];
	struct ait_pcm_dma_params *dma_params;
	int dir, channels, bits;
	int start_event;
	int ret;
    //AITPS_GBL pGBL = AITC_BASE_GBL;

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

	/*
	 * Compute SSC register settings.
	 */
	switch (ssc_p->daifmt
		& (SND_SOC_DAIFMT_FORMAT_MASK | SND_SOC_DAIFMT_MASTER_MASK)) {

	case SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS:
		/*
		 * I2S format, SSC provides BCLK and LRC clocks.
		 *
		 * The SSC transmit and receive clocks are generated
		 * from the MCK divider, and the BCLK signal
		 * is output on the SSC TK line.
		 */
		break;

	case SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM:
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
		start_event = ((channels == 1)
				? SSC_START_FALLING_RF
				: SSC_START_EDGE_RF);
		
		break;

	case SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBS_CFS:
		/*
		 * DSP/PCM Mode A format, SSC provides BCLK and LRC clocks.
		 *
		 * The SSC transmit and receive clocks are generated from the
		 * MCK divider, and the BCLK signal is output
		 * on the SSC TK line.
		 */
	
		break;

	case SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFM:
	default:
		printk(KERN_WARNING "atmel_ssc_dai: unsupported DAI format 0x%x\n",
			ssc_p->daifmt);
		return -EINVAL;
	}

	pr_debug("atmel_ssc_hw_params: "
				"sample rate = %d\r\n",params_rate(params));

	//if (!ssc_p->initialized) {
	  if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	  {
		/* Enable PMC peripheral clock for this SSC */
//#define  afe_wb(reg,val) afereg_writeb(ssc_p->ssc->regs,reg,val)
//#define  afe_rb(reg) afereg_readb(ssc_p->ssc->regs,reg)

		// 256* 44100	= 11289600
		// 512*44100 	= 22579200
		// 132000000/11289600 = 12
		// 132000000/22579200 = 6
		clk_enable(ssc_p->ssc->clk);
		MMPF_Audio_SetPLL(0,params_rate(params));
		//MMPF_Audio_SetVoiceInPath(AUDIO_INOUT_I2S0);
		//MMPF_Audio_EnableAFEClock(MMP_TRUE, params_rate(params)); 
		//MMPF_Audio_SetADCFreq(params_rate(params));

		MMPF_Audio_InitializeEncodeFIFO( SDI_TO_I2S0_FIFO, params_rate(params)*2/1000 ); //// 1ms data 
		//MMPF_Audio_InitializeEncodeFIFO( AFE_IN, params_rate(params)*2/1000 ); //// 1ms data 
		//MMPF_Audio_PowerOnADC(params_rate(params));
		//MMPF_Audio_SetADCAnalogGain(0x1F,MMP_TRUE);
		//MMPF_Audio_SetADCDigitalGain(0xA0); //move to codec control
		MMPF_I2S_PADSET(0,0);
//#undef afe_wb
//#undef afe_rb
	}
	else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{

//#define  afe_wb(reg,val) afereg_writeb(ssc_p->ssc->regs,reg,val)
//#define  afe_rb(reg) afereg_readb(ssc_p->ssc->regs,reg)

		// 256* 44100	= 11289600
		// 512*44100 	= 22579200
		// 132000000/11289600 = 12
		// 132000000/22579200 = 6
		clk_enable(ssc_p->ssc->clk);
		MMPF_Audio_SetPLL(0,params_rate(params));
		//MMPF_Audio_SetVoiceOutPath(AUDIO_OUT_AFE_LINE|AUDIO_OUT_AFE_HP);
		//MMPF_Audio_EnableAFEClock(MMP_TRUE, params_rate(params)); 
		
		//afereg_writeb(ssc_p->ssc->regs,AFE_DATA_BIT_TRANS, 0x4);	
		MMPF_Audio_InitializePlayFIFO(I2S0_FIFO_TO_SDO, params_rate(params)*2/200 ); //set 5 ms buffer 

		//MMPF_Audio_PowerOnDAC(params_rate(params));

              // Power ON AMP
              //MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO97, MMP_TRUE,0);
              ////MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO97, MMP_TRUE, MMP_FALSE);
              //MMPF_PIO_SetData(MMPF_PIO_REG_GPIO97, MMP_TRUE, MMP_FALSE);
				
		//MMPF_Audio_SetDACAnalogGain(0x00);
//#undef afe_wb
//#undef afe_rb
	
	}

	if(!ssc_p->initialized)
	{
		pr_info("ssc irq = 0x%x\r\n",ssc_p->ssc->irq);
		ret = request_irq(ssc_p->ssc->irq, ait_i2s_interrupt, 0,ssc_p->name, ssc_p);
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

	pr_debug("atmel_ssc_dai,hw_params: SSC initialized\n");
	return 0;
}


static int ait_i2s_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct ait_i2s_info *ssc_p = &ssc_info[dai->id];
	struct ait_pcm_dma_params *dma_params;
	int dir;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = 0;
	else
		dir = 1;

	//dma_params = ssc_p->dma_params[dir];

	return 0;
}


#ifdef CONFIG_PM
static int ait_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	ait_afe_info *ssc_p;

	if (!cpu_dai->active)
		return 0;

	//TODO: turn off codec 
#if 0
	ssc_p = &ssc_info[cpu_dai->id];

	/* Save the status register before disabling transmit and receive */
	ssc_p->ssc_state.ssc_sr = ssc_readl(ssc_p->ssc->regs, SR);
	ssc_writel(ssc_p->ssc->regs, CR, SSC_BIT(CR_TXDIS) | SSC_BIT(CR_RXDIS));

	/* Save the current interrupt mask, then disable unmasked interrupts */
	ssc_p->ssc_state.ssc_imr = ssc_readl(ssc_p->ssc->regs, IMR);
	ssc_writel(ssc_p->ssc->regs, IDR, ssc_p->ssc_state.ssc_imr);

	ssc_p->ssc_state.ssc_cmr = ssc_readl(ssc_p->ssc->regs, CMR);
	ssc_p->ssc_state.ssc_rcmr = ssc_readl(ssc_p->ssc->regs, RCMR);
	ssc_p->ssc_state.ssc_rfmr = ssc_readl(ssc_p->ssc->regs, RFMR);
	ssc_p->ssc_state.ssc_tcmr = ssc_readl(ssc_p->ssc->regs, TCMR);
	ssc_p->ssc_state.ssc_tfmr = ssc_readl(ssc_p->ssc->regs, TFMR);
#endif 
	return 0;
}



static int ait_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	ait_afe_info *ssc_p;
	u32 cr;

	if (!cpu_dai->active)
		return 0;
#if 0
	ssc_p = &ssc_info[cpu_dai->id];

	/* restore SSC register settings */
	ssc_writel(ssc_p->ssc->regs, TFMR, ssc_p->ssc_state.ssc_tfmr);
	ssc_writel(ssc_p->ssc->regs, TCMR, ssc_p->ssc_state.ssc_tcmr);
	ssc_writel(ssc_p->ssc->regs, RFMR, ssc_p->ssc_state.ssc_rfmr);
	ssc_writel(ssc_p->ssc->regs, RCMR, ssc_p->ssc_state.ssc_rcmr);
	ssc_writel(ssc_p->ssc->regs, CMR, ssc_p->ssc_state.ssc_cmr);

	/* re-enable interrupts */
	ssc_writel(ssc_p->ssc->regs, IER, ssc_p->ssc_state.ssc_imr);

	/* Re-enable receive and transmit as appropriate */
	cr = 0;
	cr |=
	    (ssc_p->ssc_state.ssc_sr & SSC_BIT(SR_RXEN)) ? SSC_BIT(CR_RXEN) : 0;
	cr |=
	    (ssc_p->ssc_state.ssc_sr & SSC_BIT(SR_TXEN)) ? SSC_BIT(CR_TXEN) : 0;
	ssc_writel(ssc_p->ssc->regs, CR, cr);
#endif 
	return 0;
}
#else /* CONFIG_PM */
#define atmel_ssc_suspend	NULL
#define atmel_ssc_resume	NULL
#endif /* CONFIG_PM */

static int ait_i2s_probe(struct snd_soc_dai *dai)
{
	ait_afe_info *ssc_p = &ssc_info[dai->id];
	int ret = 0;
	
	snd_soc_dai_set_drvdata(dai, ssc_p);

	/*
	 * Request SSC device
	 */
	ssc_p->ssc = ssc_request(0);
	if (IS_ERR(ssc_p->ssc)) {
		printk(KERN_ERR "ASoC: Failed to request SSC %d\n", dai->id);
		ret = PTR_ERR(ssc_p->ssc);
	}
	return ret;
}

static int ait_i2s_remove(struct snd_soc_dai *dai)
{
	ait_afe_info *ssc_p = snd_soc_dai_get_drvdata(dai);

	ssc_free(ssc_p->ssc);
	return 0;
}

#define AIT_AFE_RATES SNDRV_PCM_RATE_8000_48000
#define AIT_AFE_FORMATS (SNDRV_PCM_FMTBIT_S16_LE )


static struct snd_soc_dai_ops ait_i2s_dai_ops = {
	.startup	= ait_i2s_startup,
	.shutdown	= ait_i2s_shutdown,
	.prepare	= ait_i2s_prepare,
	.hw_params	= ait_i2s_hw_params,
	.set_fmt	= ait_i2s_set_dai_fmt,
	.set_clkdiv	= ait_i2s_set_dai_clkdiv,
};

static struct snd_soc_dai_driver ait_i2s_dai= {

		.name = "ait-i2s-dai.0",
		.probe = ait_i2s_probe,
		.remove = ait_i2s_remove,
		.suspend = ait_i2s_suspend,
		.resume = ait_i2s_resume,
#if 0		
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = ATMEL_SSC_RATES,
			.formats = ATMEL_SSC_FORMATS,},
#endif			
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIT_AFE_RATES,
			.formats = AIT_AFE_FORMATS},
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIT_AFE_RATES,
			.formats = AIT_AFE_FORMATS},
		.ops = &ait_i2s_dai_ops,


};

static __devinit int asoc_i2s_probe(struct platform_device *pdev)
{
#if 1
	return snd_soc_register_dai(&pdev->dev, &ait_i2s_dai);
#else
	BUG_ON(pdev->id < 0);
	BUG_ON(pdev->id >= ARRAY_SIZE(atmel_ssc_dai));
	return snd_soc_register_dai(&pdev->dev, &atmel_ssc_dai[pdev->id]);
#endif	
}

static int __devexit asoc_i2s_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver ait_i2s_driver = {
	.driver = {
			.name = "ait-i2s-dai",
			.owner = THIS_MODULE,
	},

	.probe = asoc_i2s_probe,
	.remove = __devexit_p(asoc_i2s_remove),
};

/**
 * ait_i2s_set_audio - Allocate the specified SSC for audio use.
 */
int ait_i2s_set_audio(int /*ssc_id*/reserve)
{
	struct ssc_device *ssc;
	static struct platform_device *vsnv3_dma_pdev;
	struct platform_device *ssc_pdev;
	int ret;

	//pr_debug("%s\r\n",__FUNCTION__);
	//if (ssc_id < 0 || ssc_id >= ARRAY_SIZE(atmel_ssc_dai))
	//    return -EINVAL;

	if(!vsnv3_dma_pdev)
	{
		vsnv3_dma_pdev = platform_device_alloc("mcrv2-i2s-pcm-audio", -1);
		if (!vsnv3_dma_pdev)
			return -ENOMEM;

		ret = platform_device_add(vsnv3_dma_pdev);
		if (ret < 0) {
			platform_device_put(vsnv3_dma_pdev);
			vsnv3_dma_pdev = NULL;
			return ret;
		}
	}
	
	ssc_pdev = platform_device_alloc(ait_i2s_driver.driver.name, -1);
	if (!ssc_pdev)
		return -ENOMEM;

	/* If we can grab the SSC briefly to parent the DAI device off it */
	ssc = ssc_request(0);
	if (IS_ERR(ssc))
		pr_warn("AIT I2S unable to parent ASoC SSC DAI on SSC: %ld\n",
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
EXPORT_SYMBOL_GPL(ait_i2s_set_audio);

module_platform_driver(ait_i2s_driver)
	
/* Module information */
MODULE_AUTHOR("Sedji Gaouaou, sedji.gaouaou@atmel.com, www.atmel.com");
MODULE_DESCRIPTION("ATMEL SSC ASoC Interface");
MODULE_LICENSE("GPL");
