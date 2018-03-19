/*
 * mcrv2-i2s-pcm.c  --  ALSA PCM interface for the AIT MCRV2 SoC.
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
#include <sound/asound.h>

#include "ait-pcm.h"

#include <mach/mmp_reg_audio.h>
#include <mach/mmpf_mcrv2_audio_ctl.h>

/*--------------------------------------------------------------------------*\
 * Hardware definition
\*--------------------------------------------------------------------------*/
#define HW_FIFO_SIZE ( 512*16 >> 3) // Channels    
static const struct snd_pcm_hardware mcrv2_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= 256,//160,
	.period_bytes_max	= 8192,
	.periods_min		= 100,
	.periods_max		= 1000,//24,
	.buffer_bytes_max	= 65536 * 50, //64 * 1024,
	.fifo_size			= HW_FIFO_SIZE,
};


/*--------------------------------------------------------------------------*\
 * Data types
\*--------------------------------------------------------------------------*/
struct mcrv2_runtime_data {
	struct mcrv2_pcm_dma_params *params;
	dma_addr_t dma_buffer;		/* physical address of dma buffer */
	dma_addr_t dma_buffer_end;	/* first address beyond DMA buffer */
	size_t period_size;

	dma_addr_t period_ptr;		/* physical address of next period */

	u8   channels;
	u16 fifo_threshold;
	u32 CntInPeriod;
};

static struct mcrv2_soc_device {
	struct mcrv2_soc_i2s_dev *soc_i2s_data;
		
} mcrv2_soc_device;
/*--------------------------------------------------------------------------*\
 * Helper functions
\*--------------------------------------------------------------------------*/
static int mcrv2_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mcrv2_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
					  &buf->addr, GFP_KERNEL);
	pr_debug("mcrv2-pcm:"
		"preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
		(void *) buf->area,
		(void *) buf->addr,
		size);

	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void mcrv2_pcm_deallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
        struct snd_pcm_substream *substream;
        struct snd_dma_buffer *buf;

        substream = pcm->streams[stream].substream;
        if (!substream)
                return;

        buf = &substream->dma_buffer;
        if (!buf->area)
                return;

        dma_free_writecombine(pcm->card->dev, buf->bytes,
                                buf->area, buf->addr);
        buf->area = NULL;
}

/*--------------------------------------------------------------------------*\
 * ISR
\*--------------------------------------------------------------------------*/
//u32 totalbytecnt = 0;
//static int AFESentCntInPeriod;

static void mcrv2_pcm_out_dma_irq(u32 i2s_sr,
	struct snd_pcm_substream *substream)
{
#if 0
	AITPS_AUD_I2SFIFO   pAUD    = AITC_BASE_I2S_FIFO;
	static int count;	
	int unWriteCnt;	
	struct atmel_runtime_data *prtd = substream->runtime->private_data;
	struct mcrv2_pcm_dma_params *params = prtd->params;


	struct snd_dma_buffer *buf = &substream->dma_buffer;

	count++;

	if (i2s_sr & AUD_INT_FIFO_REACH_UNWR_TH){
		int i;
		unsigned short* pcmPtr;

		unWriteCnt =(int)pAUD->I2S_FIFO_WR_TH;

		if(prtd->channels==1)
			unWriteCnt >>=1;
		
		BUG_ON(!prtd->period_ptr );
		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+I2SSentCntInPeriod;

		if((I2SSentCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{

			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-I2SSentCntInPeriod);++i)
			{
				pAUD->I2S_FIFO_DATA =pcmPtr[i];			
				if(prtd->channels==1)
					pAUD->I2S_FIFO_DATA = pcmPtr[i];
			}
			
			unWriteCnt -= (prtd->period_size/ params->pdc_xfer_size)-I2SSentCntInPeriod;

			I2SSentCntInPeriod = 0;

			prtd->period_ptr += prtd->period_size;			

			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;
			BUG_ON(!prtd->period_ptr );
			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->pdc_xfer_size);
			params->xfer_ptr = prtd->period_ptr;

			snd_pcm_period_elapsed(substream);			
		}

		for(i=0;i<unWriteCnt;++i)
		{
			pAUD->I2S_FIFO_DATA =pcmPtr[i];
			if(prtd->channels==1)
				pAUD->I2S_FIFO_DATA = pcmPtr[i];
		}

		I2SSentCntInPeriod+=unWriteCnt;
		
		params->xfer_ptr = prtd->period_ptr+I2SSentCntInPeriod*params->pdc_xfer_size;		

	}
	else if(i2s_sr &AUD_INT_FIFO_EMPTY)
	{
		int i;
		for (i = 0; i< 512; i++)
			pAUD->I2S_FIFO_DATA = 0;

		pr_warn("I2S FIFO EMPTY\n");

	}
#endif
	
}

/*
buf->area(Virt) = 0xffdf0000													dma_buf_end(PHY)														

        |-----------*--------*-------*--size = period_size----*--------*---------*-------|

		^
		 | ------------------>
	pcmPtr




* prtd->period_ptr

*/
unsigned long t ; 
unsigned long long samples =0 ; 
int numIRQ=0;	
static void mcrv2_pcm_in_dma_irq(u32 i2s_sr,struct snd_pcm_substream *substream)
{
#define PERIOD_SAMPLE  (prtd->period_size / params->xfer_size)
#define PERIOD_PTR_OFFSET (prtd->period_ptr-prtd->dma_buffer)
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mcrv2_runtime_data *prtd = runtime->private_data;

	struct mcrv2_pcm_dma_params *params = prtd->params;

	int unReadCntInPeriod;
	struct snd_dma_buffer *buf;// = &substream->dma_buffer;
	snd_pcm_uframes_t frams;

	AITPS_I2S   pI2S  = mcrv2_soc_device.soc_i2s_data->regbase;//AITC_BASE_PHY_I2S0;
	int readSampleCount;

	if (i2s_sr & AUD_INT_FIFO_REACH_UNRD_TH){
		int i;
		unsigned short* pcmPtr;
		unsigned short pcmPtrDummy;

		buf = &substream->dma_buffer;
		unReadCntInPeriod = prtd->fifo_threshold;

		samples+= unReadCntInPeriod/2;
		numIRQ++;
		if (time_after(jiffies, t +30* HZ)) {
			pr_info("IRQs = %d   samples = %d\n",numIRQ,samples);
			//pr_info("Period Samples = %d",PERIOD_SAMPLE);

			numIRQ = 0;
			samples = 0;
			t = jiffies;
			//pr_info("buf->area = 0x%08x   prtd->period_ptr = 0x%08x end= 0x%08x\n",buf->area,prtd->period_ptr,prtd->dma_buffer_end);
			
		}

		pcmPtr = ((unsigned short*)buf->area+PERIOD_PTR_OFFSET/params->xfer_size+prtd->CntInPeriod);
		//pr_info("pcmPtr 0x%08x\n",pcmPtr );
		//BUG_ON(pcmPtr>=((unsigned short*)buf->area+runtime->dma_bytes));
		if (pcmPtr>=((unsigned short*)buf->area+runtime->dma_bytes))
		{
			pr_debug("mcrv2_pcm_in_dma_irq: pcmPtr= 0x%08x, runtime->dma_bytes= %d \n", pcmPtr, runtime->dma_bytes);
		}

		if((prtd->CntInPeriod+unReadCntInPeriod)>= (prtd->period_size / params->xfer_size))
		{
			readSampleCount = ((prtd->period_size/ params->xfer_size)-prtd->CntInPeriod);
			//for(i=0;i<((prtd->period_size/ params->xfer_size)-prtd->CntInPeriod);++i)
			for(i=0;i<readSampleCount;++i)
			{
				pcmPtr[i] =  pI2S->I2S_FIFO.MP.DATA_RX;
				if(prtd->channels==1)
					pcmPtrDummy = pI2S->I2S_FIFO.MP.DATA_RX;
			}
//pr_info("#Read %d B\n", params->xfer_size*((prtd->period_size/ params->xfer_size)-prtd->CntInPeriod));

			unReadCntInPeriod -= readSampleCount;
//pr_info("#unReadCntInPeriod %d B\n", params->xfer_size*unReadCntInPeriod);
			
			prtd->CntInPeriod = 0;

			prtd->period_ptr += prtd->period_size;			
//pr_info("#prtd->period_ptr= %x\n",prtd->period_ptr);


			if (prtd->period_ptr >= prtd->dma_buffer_end)
			{
				prtd->period_ptr = prtd->dma_buffer;
			}
			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->xfer_size);
//pr_info("@prtd->period_ptr= %x\n",prtd->period_ptr);
			params->xfer_ptr= prtd->period_ptr; 
//pr_info("# pcmPtr= %x\n",pcmPtr);

			
			snd_pcm_period_elapsed(substream);
		}

//		if(unReadCntInPeriod)
		{
			for(i=0;i<unReadCntInPeriod;++i)
			{
				pcmPtr[i] = pI2S->I2S_FIFO.MP.DATA_RX;//pI2S->I2S_FIFO_DATA;
				if(prtd->channels==1)
					pcmPtrDummy = pI2S->I2S_FIFO.MP.DATA_RX;// pI2S->I2S_FIFO_DATA;
				
			}
	//pr_info("!Read %d B\n", unReadCntInPeriod);

			prtd->CntInPeriod+=unReadCntInPeriod;
			
			params->xfer_ptr= prtd->period_ptr+prtd->CntInPeriod*params->xfer_size;

		}
//#endif		
	}else if(i2s_sr &AUD_INT_FIFO_FULL){

		pr_warn("I2S FIFO FULL\n");

	}
	
	
}

#if 0
static void vsnv3_pcm_dma_irq(u32 fifo_sr,
	struct snd_pcm_substream *substream)
{
	static int I2SSentCntInPeriod  (prtd->CurProcCntInPeriod)
	#define AFEDacCntInPeriod (prtd->CurProcCntInPeriod)
	struct mcrv2_runtime_data *prtd = substream->runtime->private_data;
	struct mcrv2_pcm_dma_params *params = prtd->params;
	static int count;

	int unReadCntInPeriod;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	//AITPS_AFE   pAFE    = AITC_BASE_AFE;
	AITPS_I2S   pI2S    = AITC_BASE_PHY_I2S0;
	
	count++;
	
//#if 1
	if (fifo_sr & FIFO_REACH_UNRD_TH && prtd->period_ptr ){
		int i;
		unsigned short* pcmPtr,pcmPtrDummy;

		//unReadCntInPeriod = pAFE->ADC_FIFO.MP.RD_TH/params->pdc_xfer_size;	//For 2 bye per sample		
		unReadCntInPeriod = i2s_fifo_reg(pI2S,RD_TH_RX)/params->xfer_size;	//For 2 bye per sample
		pr_debug("ADC data len = %d \r\n",unReadCntInPeriod);

		pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->xfer_size+AFESentCntInPeriod);

		if ((AFESentCntInPeriod+unReadCntInPeriod)>= (prtd->period_size / params->xfer_size))
		{
			for(i=0;i<((prtd->period_size/ params->xfer_size)-AFESentCntInPeriod);++i)
			{
				if(prtd->channels==1)				
					pcmPtrDummy = i2s_fifo_reg(pI2S,DATA_RX);				
				pcmPtr[i] = i2s_fifo_reg(pI2S,DATA_RX);
			}
		
			unReadCntInPeriod -= ((prtd->period_size/ params->xfer_size)-AFESentCntInPeriod);
			AFESentCntInPeriod = 0;

			
			prtd->period_ptr += prtd->period_size;			

			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->xfer_size);
			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = prtd->period_size / params->xfer_size;

			
			snd_pcm_period_elapsed(substream);
		}

		for(i=0;i<unReadCntInPeriod;++i)
		{
			if(prtd->channels==1)				
				pcmPtrDummy = i2s_fifo_reg(pI2S,DATA_RX);				
			pcmPtr[i] = i2s_fifo_reg(pI2S,DATA_RX);	
		}

		AFESentCntInPeriod+=unReadCntInPeriod;
		
		params->pdc->xpr = prtd->period_ptr+AFESentCntInPeriod*params->pdc_xfer_size;

		//totalbytecnt+= unReadCntInPeriod<<1;

	}
	else if (fifo_sr & FIFO_REACH_UNWR_TH && prtd->period_ptr )
	{
		int i;
		unsigned short* pcmPtr;

		u32 unWriteCnt = (u32) i2s_fifo_reg(pI2S,UNWR_CNT_TX);	
		if(prtd->channels==1)
			unWriteCnt/=2; //because AFE HW always 2 channel


		pr_debug("DAC data len = %d \r\n",unWriteCnt);


		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+AFEDacCntInPeriod;

		if((AFEDacCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{

			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod);++i)
			{
				 i2s_fifo_reg(AITC_BASE_AFE,DATA_TX) = pcmPtr[i];
				if(prtd->channels==1)
					i2s_fifo_reg(AITC_BASE_AFE,DATA_TX) = pcmPtr[i];					
			}
			
			unWriteCnt -= (prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod;

			AFEDacCntInPeriod = 0;


			prtd->period_ptr += prtd->period_size;			

			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->pdc_xfer_size);
			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = prtd->period_size / params->pdc_xfer_size;
			
			snd_pcm_period_elapsed(substream);			
		}
		else
		{
			for(i=0;i<unWriteCnt;++i)
			{
				i2s_fifo_reg(AITC_BASE_AFE,DATA_TX) = pcmPtr[i];
				if(prtd->channels==1)
					i2s_fifo_reg(AITC_BASE_AFE,DATA_TX) = pcmPtr[i];			
			}
			AFEDacCntInPeriod+=unWriteCnt;
		}
		params->pdc->xpr = prtd->period_ptr+AFEDacCntInPeriod*params->pdc_xfer_size;
	}
	else if(fifo_sr &FIFO_FULL )
	{
		pr_warn("AFE FIFO FULL\n");

	}
//#endif 

#undef AFESentCntInPeriod
#undef AFEDacCntInPeriod
}
#endif

/*--------------------------------------------------------------------------*\
 * PCM operations
\*--------------------------------------------------------------------------*/
static int mcrv2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mcrv2_runtime_data *prtd;
	int ret = 0;
//pr_info("mcrv2_pcm_open substream= %x\n",substream);

	snd_soc_set_runtime_hwparams(substream, &mcrv2_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
	
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct mcrv2_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		pr_err("%s: prtd == NULL\r\n",__FUNCTION__);		
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;
 out:
	return ret;
}

static int mcrv2_pcm_close(struct snd_pcm_substream *substream)
{
	struct mcrv2_runtime_data *prtd = substream->runtime->private_data;

	kfree(prtd);
	return 0;
}

static int mcrv2_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mcrv2_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int bits;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);
	//pr_info("mcrv2_pcm_hw_params prtd= %x\n",prtd);

	prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	//pr_info("mcrv2_pcm_hw_params prtd->params= %x\n",prtd->params);

	prtd->params->substream  = substream;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prtd->params->dma_intr_handler = mcrv2_pcm_out_dma_irq;
	else
		prtd->params->dma_intr_handler = mcrv2_pcm_in_dma_irq;
		
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		bits = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		bits = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bits = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bits = 32;
		break;
	default:
		printk(KERN_WARNING "mcrv2_pcm_hw_params: unsupported PCM format");
		return -EINVAL;
	}

	prtd->params->xfer_size = bits>>3;
		
	prtd->dma_buffer = runtime->dma_addr;

	prtd->period_size = params_period_bytes(params);

	prtd->dma_buffer_end = runtime->dma_addr + (runtime->dma_bytes/prtd->period_size)*prtd->period_size;
	
	prtd->channels = params_channels(params);

	pr_info("mcrv2-pcm: "
		"hw_params: params_period_size = %d frames\n"
		"phy width = %d bits\n",
		params_period_size(params) ,
		snd_pcm_format_physical_width(params_format(params)));
	
		
	pr_info(
		"hw_params: DMA for %s initialized \n"
		"(dma_bytes=%u, period_size=%u bytes)\n"
		"fifo_size=%u bytes xfer_size=%u\n",
		prtd->params->name,
		runtime->dma_bytes,
		prtd->period_size,
		runtime->hw.fifo_size,
		prtd->params->xfer_size);

	prtd->fifo_threshold =  (runtime->hw.fifo_size/prtd->params->xfer_size)>>1;	// totoal fifo fames /2
//	prtd->fifo_threshold =  runtime->hw.fifo_size>>1;		// totoal fifo fames /2
	pr_info("mcrv2-pcm: fifo threshold = %d\n ",prtd->fifo_threshold );
	
	return 0;
}

static int mcrv2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mcrv2_runtime_data *prtd = substream->runtime->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;

	if (params != NULL) {
	}

	return 0;
}

static int mcrv2_pcm_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}



static int mcrv2_pcm_trigger(struct snd_pcm_substream *substream,int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct mcrv2_runtime_data *prtd = rtd->private_data;
	struct mcrv2_pcm_dma_params *params = prtd->params;
	int ret = 0;
	AITPS_I2S   pI2S    = mcrv2_soc_device.soc_i2s_data->regbase;

	int dir = substream->stream; 
	
	//pr_info("mcrv2_pcm_trigger: cmd = %d\r\n",cmd);	
	pr_info("mcrv2-pcm:buffer_size = %ld frames,"
		"dma_area = %p, dma_bytes = %u\n",
		rtd->buffer_size, rtd->dma_area, rtd->dma_bytes);
	

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			
                		prtd->period_ptr = prtd->dma_buffer;
#if 0
						
                		params->pdc->xpr = prtd->period_ptr;
                		params->pdc->xcr = 0;
                		params->pdc->xnpr = prtd->period_ptr;
                		params->pdc->xncr = 0;
				prtd->CurProcCntInPeriod = 0;
				#if 0
				pr_debug(	"ait-pcm: trigger: "
							"period_ptr=%lx,  AFEFIFO=%u, AFECPU=%u\n",
							(unsigned long)prtd->period_ptr,
							//pAFE->AFE_FIFO_CPU_INT_EN,//AFE_INT_FIFO_REACH_UNRD_TH;
							dir == SNDRV_PCM_STREAM_CAPTURE ? afe_i2s_fifo_reg(pI2S, CPU_INT_EN) : afe_dac_fifo_reg(pI2S, CPU_INT_EN),
							pAFE->AFE_CPU_INT_EN
						);
				#endif
			}

			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				MMPF_Audio_SetMux(SDI_TO_I2S0_RX_FIFO,MMP_TRUE);
				//afereg_writew(pAFE, AFE_REG_OFFSET(AFE_ADC_DIGITAL_GAIN), vsnv3_afe_data->mic_digital_gain<<8 | vsnv3_afe_data->mic_digital_gain);
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				MMPF_Audio_SetMux(I2S0_FIFO_TO_SDO,MMP_TRUE);
				//afereg_writew(pAFE, AFE_REG_OFFSET(AFE_DAC_DIGITAL_GAIN), vsnv3_afe_data->spk_digital_gain<<8 | vsnv3_afe_data->spk_digital_gain);
			}
#endif			
			#if 0
			pr_debug("atmel-pcm: trigger: "
						"period_ptr=%lx,  AFEFIFO=%u, AFECPU=%u\n",
						(unsigned long)prtd->period_ptr,
						dir == SNDRV_PCM_STREAM_CAPTURE ? afe_adc_fifo_reg(pAFE, CPU_INT_EN) : afe_dac_fifo_reg(pAFE, CPU_INT_EN),
						pAFE->AFE_CPU_INT_EN
					);
			#endif

			pI2S->I2S_FIFO.MP.RD_TH_RX = prtd->fifo_threshold;// (rtd->hw.fifo_size/params->xfer_size)/2;
			




			if (dir == SNDRV_PCM_STREAM_PLAYBACK)
			{		

				mcrv2_i2s_reg_writeb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO_CPU_INT_EN),AUD_INT_FIFO_EMPTY|AUD_INT_FIFO_REACH_UNWR_TH);	

			}
			else
			{
				//pr_debug("Start capture\n");
				mcrv2_i2s_reg_writeb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO_CPU_INT_EN), AUD_INT_FIFO_FULL|AUD_INT_FIFO_REACH_UNRD_TH);
			}
		//pr_info("I2S_FIFO_CPU_INT_EN = %x\n",mcrv2_i2s_reg_readb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO_CPU_INT_EN)));
		//pr_info("pI2S->I2S_FIFO.MP.RD_TH_RX = %x\n",pI2S->I2S_FIFO.MP.RD_TH_RX );//mcrv2_i2s_reg_readb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO.MP.RD_TH_RX)));
		pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
		samples =0 ; 
		numIRQ=0;			
		t = jiffies;
		break;		/* SNDRV_PCM_TRIGGER_START */

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				//MMPF_Audio_SetMux(SDI_TO_I2S0_RX_FIFO,MMP_FALSE);
				//MMPF_Audio_EnableAFEClock(MMP_FALSE,0);
				u8 int_en = mcrv2_i2s_reg_readb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO_CPU_INT_EN));				
				mcrv2_i2s_reg_writeb(mcrv2_soc_device.soc_i2s_data, I2S_REG_OFFSET(I2S_FIFO_CPU_INT_EN), int_en&(~(AUD_INT_FIFO_FULL|AUD_INT_FIFO_REACH_UNRD_TH)));
				
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				//MMPF_Audio_SetMux(I2S0_FIFO_TO_SDO,MMP_FALSE);
				//MMPF_Audio_EnableAFEClock(MMP_FALSE,0);
			}
			//start =0;

			pI2S->I2S_CPU_INT_EN = AUD_INT_DIS;
			
			break;

		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
#if 0			
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				//TODO: Enable I2S input interrupt
				//afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AFE_INT_FIFO_REACH_UNRD_TH;
				//afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AUD_ADC_INT_EN;
				break;
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				//TODO: Disable I2S output interrupt 
				//afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AFE_INT_FIFO_REACH_UNWR_TH;
				//afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AUD_DAC_INT_EN;		
			}
#endif			
		break;
		
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static snd_pcm_uframes_t mcrv2_pcm_pointer(struct snd_pcm_substream *substream)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mcrv2_runtime_data *prtd = runtime->private_data;
	struct mcrv2_pcm_dma_params *params = prtd->params;
	dma_addr_t ptr;
	snd_pcm_uframes_t frams;

	ptr = params->xfer_ptr;	
	frams = bytes_to_frames(runtime, ptr - prtd->dma_buffer);
	if (frams == runtime->buffer_size)
		frams = 0;

	return frams;

}

static int mcrv2_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}


static struct snd_pcm_ops mcrv2_i2s_pcm_ops = {
	.open		= mcrv2_pcm_open,
	.close		= mcrv2_pcm_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params	= mcrv2_pcm_hw_params,
	.hw_free		= mcrv2_pcm_hw_free,
	.prepare		= mcrv2_pcm_prepare,
	.trigger		= mcrv2_pcm_trigger,
	.pointer		= mcrv2_pcm_pointer,
	.mmap		= mcrv2_pcm_mmap,
};


/*--------------------------------------------------------------------------*\
 * ASoC platform driver
\*--------------------------------------------------------------------------*/
static int mcrv2_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;	//created by soc_new_pcm->snd_pcm_new
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = (u64*)0xffffffff;//&atmel_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = (u64)0xffffffff;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		pr_debug("mcrv2-pcm: Allocating PCM playback DMA buffer\n");
		
		ret = mcrv2_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		pr_debug("mcrv2-pcm: Allocating PCM capture DMA buffer\n");
		ret = mcrv2_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

static void mcrv2_pcm_free(struct snd_pcm *pcm)
{
	mcrv2_pcm_deallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
	mcrv2_pcm_deallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);

}

#ifdef CONFIG_PM
static int mcrv2_pcm_suspend(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct mcrv2_runtime_data *prtd;
	struct atmel_pcm_dma_params *params;

	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;


	return 0;
}

static int mcrv2_pcm_resume(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct mcrv2_runtime_data *prtd;
	struct mcrv2_pcm_dma_params *params;

	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;
	return 0;
}
#else
#define mcrv2_pcm_suspend	NULL
#define mcrv2_pcm_resume	NULL
#endif

static int mcrv2_pcm_probe(struct snd_soc_platform *platform)
{
	snd_soc_platform_set_drvdata(platform, &mcrv2_soc_device);
	return 0;
}

static int mcrv2_pcm_remove(struct snd_soc_platform *platform)
{
	return 0;
}

static struct snd_soc_platform_driver mcrv2_soc_platform = {
	.probe 	= mcrv2_pcm_probe,
	.remove  =mcrv2_pcm_remove,
	.ops		= &mcrv2_i2s_pcm_ops,
		
	.pcm_new	= mcrv2_pcm_new,
	.pcm_free	= mcrv2_pcm_free,
	.suspend		= mcrv2_pcm_suspend,
	.resume		= mcrv2_pcm_resume,
	
	.read =0,
	.write= 0,
	
};

static int __devinit mcrv2_soc_platform_probe(struct platform_device *pdev)
{
	struct mcrv2_soc_i2s_dev *devdata = platform_get_drvdata(pdev);
	mcrv2_soc_device.soc_i2s_data = devdata;
	return snd_soc_register_platform(&pdev->dev, &mcrv2_soc_platform);
}

static int __devexit mcrv2_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver mcrv2_pcm_driver = {
	.driver = {
			.name = "mcrv2-pcm-audio",
			.owner = THIS_MODULE,
	},

	.probe = mcrv2_soc_platform_probe,
	.remove = __devexit_p(mcrv2_soc_platform_remove),
};

module_platform_driver(mcrv2_pcm_driver)

MODULE_AUTHOR("Vincent Chen <vincent_chen@a-i-t.com.tw>");
MODULE_DESCRIPTION("AIT PCM module");
MODULE_LICENSE("GPL");
