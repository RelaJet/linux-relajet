/*
 * atmel-pcm.c  --  ALSA PCM interface for the Atmel atmel SoC.
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *
 * Authors: Sedji Gaouaou <sedji.gaouaou@atmel.com>
 *
 * Based on at91-pcm. by:
 * Frank Mandarino <fmandarino@endrelia.com>
 * Copyright 2006 Endrelia Technologies Inc.
 *
 * Based on pxa2xx-pcm.c by:
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/atmel_pdc.h>
#include <linux/atmel-ssc.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "ait-pcm.h"

#include <mach/mmpf_typedef.h>
//#include <mach/mmp_err.h>
#include <mach/mmp_reg_audio.h>
#include <mach/mmp_reg_gbl.h>
//#include <mach/mmpf_audio_ctl.h>

/*--------------------------------------------------------------------------*\
 * Hardware definition
\*--------------------------------------------------------------------------*/
/* TODO: These values were taken from the AT91 platform driver, check
 *	 them against real values for AT32
 */
static const struct snd_pcm_hardware atmel_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= 192,	
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 1024,
	.buffer_bytes_max	= 20* 512,//32 * 1024,
};


/*--------------------------------------------------------------------------*\
 * Data types
\*--------------------------------------------------------------------------*/
struct atmel_runtime_data {
	struct atmel_pcm_dma_params *params;
	dma_addr_t dma_buffer;		/* physical address of dma buffer */
	dma_addr_t dma_buffer_end;	/* first address beyond DMA buffer */
	size_t period_size;

	dma_addr_t period_ptr;		/* physical address of next period */

	/* PDC register save */
	u32 pdc_xpr_save;
	u32 pdc_xcr_save;
	u32 pdc_xnpr_save;
	u32 pdc_xncr_save;
};


/*--------------------------------------------------------------------------*\
 * Helper functions
\*--------------------------------------------------------------------------*/
static int atmel_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = atmel_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
					  &buf->addr, GFP_KERNEL);
	pr_debug("atmel-pcm:"
		"preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
		(void *) buf->area,
		(void *) buf->addr,
		size);

	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}
/*--------------------------------------------------------------------------*\
 * ISR
\*--------------------------------------------------------------------------*/
static int I2SSentCntInPeriod;

int k = 0;
u32 TotalTxCnt = 0;

static void atmel_pcm_dma_irq(u32 ssc_sr,
	struct snd_pcm_substream *substream)
{

    AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	static int count;
    int unWriteCnt;
	struct atmel_runtime_data *prtd = substream->runtime->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;


struct snd_dma_buffer *buf = &substream->dma_buffer;


	count++;

	if (ssc_sr & AUD_INT_FIFO_REACH_UNWR_TH){
		int i;
		unsigned short* pcmPtr;
#if 0//def DEBUG	
		pr_warning("atmel-pcm: buffer %s on %s"
				" (SSC_SR=%#x, count=%d)\n",
				substream->stream == SNDRV_PCM_STREAM_PLAYBACK
				? "underrun" : "overrun",
				params->name, ssc_sr, count);
#endif
		/* re-start the PDC */
//		ssc_writex(params->ssc->regs, ATMEL_PDC_PTCR,
//			   params->mask->pdc_disable);


		pAUD->I2S_FIFO_CPU_INT_EN &= ~AUD_INT_FIFO_REACH_UNWR_TH;
		unWriteCnt =(int)pAUD->I2S_FIFO_WR_TH;///params->pdc_xfer_size;	// Bytes per channel



		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+I2SSentCntInPeriod;

		if((I2SSentCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{

			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-I2SSentCntInPeriod);++i)
			{
				pAUD->I2S_FIFO_DATA =pcmPtr[i];
			}
			
			unWriteCnt -= (prtd->period_size/ params->pdc_xfer_size)-I2SSentCntInPeriod;

			I2SSentCntInPeriod = 0;


			prtd->period_ptr += prtd->period_size;			

			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->pdc_xfer_size);
			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = prtd->period_size / params->pdc_xfer_size;
#if 0//def DEBUG	

			pr_err("params->pdc->xpr : 0x%x\r\n",params->pdc->xpr  );
			pr_err("params->pdc->xcr : 0x%x\r\n",params->pdc->xcr );
			pr_err("prtd->period_ptr : 0x%x\r\n",prtd->period_ptr );			
#endif
			snd_pcm_period_elapsed(substream);			
		}



		for(i=0;i<unWriteCnt;++i)
		{
			pAUD->I2S_FIFO_DATA =pcmPtr[i];
		}


		pAUD->I2S_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;


		I2SSentCntInPeriod+=unWriteCnt;
		
		params->pdc->xpr = prtd->period_ptr+I2SSentCntInPeriod*params->pdc_xfer_size;

	}
}

/*--------------------------------------------------------------------------*\
 * PCM operations
\*--------------------------------------------------------------------------*/
static int vsnv3_i2s_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct atmel_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	/* this may get called several times by oss emulation
	 * with different params */

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	prtd->params->dma_intr_handler = atmel_pcm_dma_irq;

	prtd->dma_buffer = runtime->dma_addr;
	prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);

	pr_debug("atmel-pcm: "
		"hw_params: DMA for %s initialized "
		"(dma_bytes=%u, period_size=%u)\n",
		prtd->params->name,
		runtime->dma_bytes,
		prtd->period_size);
	return 0;
}

static int vsnv3_i2s_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct atmel_runtime_data *prtd = substream->runtime->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;

	if (params != NULL) {
		prtd->params->dma_intr_handler = NULL;
	}

	return 0;
}

static int vsnv3_i2s_pcm_prepare(struct snd_pcm_substream *substream)
{
	AITPS_I2S   pAUD    = AITC_BASE_I2S0;	


	pAUD->I2S_FIFO_CPU_INT_EN &= ~AUD_INT_FIFO_REACH_UNWR_TH;
	return 0;
}


static int start =0;
static int vsnv3_i2s_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct atmel_runtime_data *prtd = rtd->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;
	int ret = 0;
	AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	AITPS_AUD   pAFE    = AITC_BASE_AUD;


	pr_debug("atmel_pcm_trigger: cmd = %d\r\n",cmd);
	pr_debug("atmel-pcm:buffer_size = %ld,"
		"dma_area = %p, dma_bytes = %u\n",
		rtd->buffer_size, rtd->dma_area, rtd->dma_bytes);
	


	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			if(start ==0)
			{
					int i;
					prtd->period_ptr = prtd->dma_buffer;

					params->pdc->xpr = prtd->period_ptr;
					params->pdc->xcr = 0;
					params->pdc->xnpr = prtd->period_ptr;
					params->pdc->xncr = 0;
					I2SSentCntInPeriod = 0;
#if 0
					ssc_writex(params->ssc->regs, params->pdc->xpr,
						   prtd->period_ptr);
					ssc_writex(params->ssc->regs, params->pdc->xcr,
						   prtd->period_size / params->pdc_xfer_size);

					prtd->period_ptr += prtd->period_size;

					ssc_writex(params->ssc->regs, params->pdc->xnpr,
						   prtd->period_ptr);
					ssc_writex(params->ssc->regs, params->pdc->xncr,
						   prtd->period_size / params->pdc_xfer_size);
#endif

					pr_debug("atmel-pcm: trigger: "
						"period_ptr=%lx, I2SFIFO=%u, "
						"I2SCPU=%u, \n",
							(unsigned long)prtd->period_ptr,
							pAUD->I2S_FIFO_CPU_INT_EN,
							pAUD->I2S_CPU_INT_EN
						);

					pAUD->I2S_FIFO_RST = AUD_FIFO_RST_EN;
					pAUD->I2S_FIFO_RST = 0;
					pAUD->I2S_FIFO_WR_TH = 128;

					for (i = 0; i< 256; i++)
						pAUD->I2S_FIFO_DATA = 0;

					pAUD->I2S_CLK_CTL |= I2S_MCLK_FIX;
					pAUD->I2S_CLK_DIV = 4;	//for audio clock = 48 MHz
			
					pAUD->I2S_FIFO_CPU_INT_EN = AUD_INT_FIFO_REACH_UNWR_TH;
					pAUD->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
					pAUD->I2S_CPU_INT_EN = AUD_INT_EN;

					pAFE->AFE_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;	// 16 bit auto mode


					start = 1;
			}

			pAUD->I2S_FIFO_SR = pAUD->I2S_FIFO_SR;
			pAUD->I2S_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
			
#if 0
			pr_debug("atmel-pcm: trigger: "
				"period_ptr=%lx, xpr=%u, "
				"xcr=%u, xnpr=%u, xncr=%u\n",
				(unsigned long)prtd->period_ptr,
				ssc_readx(params->ssc->regs, params->pdc->xpr),
				ssc_readx(params->ssc->regs, params->pdc->xcr),
				ssc_readx(params->ssc->regs, params->pdc->xnpr),
				ssc_readx(params->ssc->regs, params->pdc->xncr));

			ssc_writex(params->ssc->regs, SSC_IER,
				   params->mask->ssc_endx | params->mask->ssc_endbuf);
			ssc_writex(params->ssc->regs, SSC_PDC_PTCR,
				   params->mask->pdc_enable);

			pr_debug("sr=%u imr=%u\n",
				ssc_readx(params->ssc->regs, SSC_SR),
				ssc_readx(params->ssc->regs, SSC_IER));
#endif		
			break;		/* SNDRV_PCM_TRIGGER_START */

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

			pAUD->I2S_FIFO_CPU_INT_EN &= ~AUD_INT_FIFO_REACH_UNWR_TH;
			pAUD->I2S_FIFO_SR = pAUD->I2S_FIFO_SR;
			pAUD->I2S_FIFO_RST = AUD_FIFO_RST_EN;
			pAUD->I2S_FIFO_RST = 0;
			{
				int i;
				for (i = 0; i< 512; i++)
					pAUD->I2S_FIFO_DATA = 0;
			}
			start =0;
			break;

		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			pAUD->I2S_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
			break;

		default:
			ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t vsnv3_i2s_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct atmel_runtime_data *prtd = runtime->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	ptr = params->pdc->xpr;
	x = bytes_to_frames(runtime, ptr - prtd->dma_buffer);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int vsnv3_i2s_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct atmel_runtime_data *prtd;
	int ret = 0;

	snd_soc_set_runtime_hwparams(substream, &atmel_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
	
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct atmel_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {	
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;

 out:
	return ret;
}

static int vsnv3_i2s_pcm_close(struct snd_pcm_substream *substream)
{
	struct atmel_runtime_data *prtd = substream->runtime->private_data;

	AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	pAUD->I2S_FIFO_CPU_INT_EN &= ~AUD_INT_FIFO_REACH_UNWR_TH;

	kfree(prtd);
	return 0;
}

static int vsnv3_i2s_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{

	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_ops atmel_pcm_ops = {
	.open		= vsnv3_i2s_pcm_open,
	.close		= vsnv3_i2s_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= vsnv3_i2s_pcm_hw_params,
	.hw_free	=vsnv3_i2s_pcm_hw_free,
	.prepare	= vsnv3_i2s_pcm_prepare,
	.trigger	= vsnv3_i2s_pcm_trigger,
	.pointer	= vsnv3_i2s_pcm_pointer,
	.mmap		= vsnv3_i2s_pcm_mmap,
};


/*--------------------------------------------------------------------------*\
 * ASoC platform driver
\*--------------------------------------------------------------------------*/
//static u64 atmel_pcm_dmamask = 0xffffffff;


static int vsnv3_i2s_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;	//created by soc_new_pcm->snd_pcm_new
	int ret = 0;
	
	/*if (!card->dev->dma_mask)
		card->dev->dma_mask = 0xffffffff;//&atmel_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;*/

	if (dai->driver->playback.channels_min) {
		ret = atmel_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		pr_debug("atmel-pcm:"
				"Allocating PCM capture DMA buffer\n");
		ret = atmel_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

static void vsnv3_i2s_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_coherent(pcm->card->dev, buf->bytes,
				  buf->area, buf->addr);
		buf->area = NULL;
	}
}

#ifdef CONFIG_PM
static int vsnv3_i2s_pcm_suspend(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct atmel_runtime_data *prtd;
	struct atmel_pcm_dma_params *params;


	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;

	/* disable the PDC and save the PDC registers */

	ssc_writel(params->ssc->regs, PDC_PTCR, params->mask->pdc_disable);

	prtd->pdc_xpr_save = ssc_readx(params->ssc->regs, params->pdc->xpr);
	prtd->pdc_xcr_save = ssc_readx(params->ssc->regs, params->pdc->xcr);
	prtd->pdc_xnpr_save = ssc_readx(params->ssc->regs, params->pdc->xnpr);
	prtd->pdc_xncr_save = ssc_readx(params->ssc->regs, params->pdc->xncr);

	return 0;
}

static int vsnv3_i2s_pcm_resume(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct atmel_runtime_data *prtd;
	struct atmel_pcm_dma_params *params;


	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;

	/* restore the PDC registers and enable the PDC */
	ssc_writex(params->ssc->regs, params->pdc->xpr, prtd->pdc_xpr_save);
	ssc_writex(params->ssc->regs, params->pdc->xcr, prtd->pdc_xcr_save);
	ssc_writex(params->ssc->regs, params->pdc->xnpr, prtd->pdc_xnpr_save);
	ssc_writex(params->ssc->regs, params->pdc->xncr, prtd->pdc_xncr_save);

	ssc_writel(params->ssc->regs, PDC_PTCR, params->mask->pdc_enable);
	return 0;
}
#else
#define vsnv3_i2s_pcm_suspend	NULL
#define vsnv3_i2s_pcm_resume	NULL
#endif

static struct snd_soc_platform_driver vsnv3_i2s_soc_platform = {
	.ops		= &atmel_pcm_ops,
	.pcm_new	= vsnv3_i2s_pcm_new,
	.pcm_free	= vsnv3_i2s_pcm_free_dma_buffers,
	.suspend	= vsnv3_i2s_pcm_suspend,
	.resume		= vsnv3_i2s_pcm_resume,
};

static int __devinit vsnv3_i2s_soc_platform_probe(struct platform_device *pdev)
{

	return snd_soc_register_platform(&pdev->dev, &vsnv3_i2s_soc_platform);
}

static int __devexit vsnv3_i2s_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver vsnv3_i2s_pcm_driver = {
	.driver = {
			.name = "atmel-pcm-audio",
			.owner = THIS_MODULE,
	},

	.probe = vsnv3_i2s_soc_platform_probe,
	.remove = __devexit_p(vsnv3_i2s_soc_platform_remove),
};

static int __init snd_atmel_pcm_init(void)
{
	return platform_driver_register(&vsnv3_i2s_pcm_driver);
}
module_init(snd_atmel_pcm_init);

static void __exit snd_atmel_pcm_exit(void)
{
	platform_driver_unregister(&vsnv3_i2s_pcm_driver);
}
module_exit(snd_atmel_pcm_exit);

MODULE_AUTHOR("Sedji Gaouaou <sedji.gaouaou@atmel.com>");
MODULE_DESCRIPTION("AIT8455 PCM module");
MODULE_LICENSE("GPL");
