/*
 * ait-afe-pcm-ex.c --  ALSA PCM interface enhance for the AIT MCRV2 SoC.
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
#include <linux/spinlock_types.h>
#include <linux/kthread.h>
//#include <linux/semaphore.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "ait-pcm.h"
#include "ait_aud_dai.h"

#include <mach/mmp_reg_audio.h>
#include <mach/mmpf_mcrv2_audio_ctl.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm_api.h>
#include <mach/mcrv2_afe.h>
#include <mach/gpio.h>

#include "aec-work-queue.h"
#include "audio-ring-buf.h"

#define CONFIG_MIC_REVERSE
//#define CONFIG_AEC_8K
/*--------------------------------------------------------------------------*\
 * Hardware definition
\*--------------------------------------------------------------------------*/
/* TODO: These values were taken from the AT91 platform driver, check
 *	 them against real values for AT32
 */
static const struct snd_pcm_hardware vsnv3_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= 512,
	.period_bytes_max	= 8192,
	.periods_min		= 100,
	.periods_max		= 1024,
	#if defined(CONFIG_AEC_8K)
	.buffer_bytes_max	= 128 * 1024,  //512 * 1024,
	#else
	.buffer_bytes_max       = 512 * 1024,
	#endif
};


/*--------------------------------------------------------------------------*\
 * Data types
\*--------------------------------------------------------------------------*/
struct vsnv3_runtime_data {
	struct ait_pcm_dma_params *params;
	dma_addr_t dma_buffer;		/* physical address of dma buffer */
	dma_addr_t dma_buffer_end;	/* first address beyond DMA buffer */
	size_t period_size;

	dma_addr_t period_ptr;		/* physical address of next period */

	/* PDC register save */
	u32 pdc_xpr_save;
	u32 pdc_xcr_save;
	u32 pdc_xnpr_save;
	u32 pdc_xncr_save;
	u8   channels;
	u32 CurProcCntInPeriod;
};

/*--------------------------------------------------------------------------*\
 * Helper functions
\*--------------------------------------------------------------------------*/
//DEFINE_SPINLOCK(g_aec_stop_lock);
//unsigned long g_aec_lock_flag;
DEFINE_SEMAPHORE(g_aec_stop_lock);
volatile aec_work_handle *g_aec_work_handle = 0;
int MIC_CHANNELS = 0;
void aec_data_done(audio_ring_buffer *aec_output,void* user_data)
{
	//spin_lock_irqsave(&g_aec_stop_lock,g_aec_lock_flag);
	down(&g_aec_stop_lock);
	if(g_aec_work_handle)
	{
		struct snd_pcm_substream *substream = (struct snd_pcm_substream*) user_data;
		struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
		
		while(audio_ring_get_count(aec_output) > (prtd->period_size/sizeof(short)) )
		{
			struct ait_pcm_dma_params *params = prtd->params;	
			struct snd_dma_buffer *buf = &substream->dma_buffer;
			
			short* dest_ptr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size);

			audio_ring_buf_read(aec_output, dest_ptr, prtd->period_size/sizeof(short) );
			
			prtd->period_ptr += prtd->period_size;			
			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = prtd->period_size / params->pdc_xfer_size;
				
			snd_pcm_period_elapsed(substream);
		}
	}
	//spin_unlock_irqrestore(&g_aec_stop_lock,g_aec_lock_flag);	
	up(&g_aec_stop_lock);
}

static int vsnv3_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = vsnv3_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
					  &buf->addr, GFP_KERNEL);
	pr_debug("vsnv3-pcm:"
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
extern AUDIO_ENH_MODE g_audio_mode; 

short ADCTempBuf[AEC_PERIOD_SAMPLE];
short DACTempBuf[AEC_PERIOD_SAMPLE];
static int g_adc_status=SNDRV_PCM_TRIGGER_STOP;
static int g_dac_status=SNDRV_PCM_TRIGGER_STOP;
static void vsnv3_pcm_dma_irq_aec_enabled(u32 afe_sr, struct snd_pcm_substream *substream)
{
	#define AFESentCntInPeriod  (prtd->CurProcCntInPeriod)
	#define AFEDacCntInPeriod (prtd->CurProcCntInPeriod)
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
	struct ait_pcm_dma_params *params = prtd->params;
	//static int count;

	int unReadCntInPeriod;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	AITPS_AFE   pAFE    = AITC_BASE_AFE;
	//count++;

	if(params->status == SNDRV_PCM_TRIGGER_STOP)
		return;
	
	if (afe_sr & AFE_INT_FIFO_REACH_UNRD_TH && prtd->period_ptr ){
		int i;
		short* pcmPtr = ADCTempBuf;
		short pcmPtrDummy;

		//unReadCntInPeriod = pAFE->ADC_FIFO.MP.RD_TH/params->pdc_xfer_size;	//For 2 bye per sample		
		unReadCntInPeriod = afe_adc_fifo_reg(pAFE,RD_TH)/params->pdc_xfer_size;	//For 2 bye per sample
		pr_debug("ADC data len = %d \r\n",unReadCntInPeriod);

		for(i=0;i<unReadCntInPeriod;i+=prtd->channels)
		{
			#if defined(CONFIG_MIC_REVERSE)
				if(prtd->channels==1)
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
					ADCTempBuf[i] = pcmPtr[i];
					//ADCTempBuf[i+1] = pcmPtrDummy;
				}
				else 
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
					ADCTempBuf[i] = pcmPtr[i];
					ADCTempBuf[i+1] = pcmPtr[i+1];				
				}
			#else
				if(prtd->channels==1)
				{
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					ADCTempBuf[i] = pcmPtr[i] ;//pcmPtrDummy;
					//ADCTempBuf[i+1] = pcmPtr[i];
				}
				else 
				{
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					ADCTempBuf[i] = pcmPtr[i];
					ADCTempBuf[i+1] = pcmPtr[i+1];						
				}
			#endif
		}

#if 0
		audio_ring_buf_write( &g_adc_ring, ADCTempBuf, unReadCntInPeriod );
		while( audio_ring_get_count(&g_adc_ring)>=AEC_PERIOD_SAMPLE 
				&& audio_ring_get_count(&g_dac_ring)>=AEC_PERIOD_SAMPLE)
		{
			queue_aec_a2b_work(g_aec_work_handle,&g_adc_ring,&g_dac_ring,(void*)substream);
		}
#endif 
		aec_wq_put_adc_data( 	g_aec_work_handle,		//awc wq handle
								ADCTempBuf,			//source data buffer
								unReadCntInPeriod,		//source data length
								aec_data_done,			//aec done callback
								(void*)substream			//user data 
							);
	
	}
	else if (afe_sr & AFE_INT_FIFO_REACH_UNWR_TH && prtd->period_ptr )
	{
		int i;
		unsigned short* pcmPtr;

		u32 unWriteCnt = (u32) afe_dac_fifo_reg(pAFE,UNWR_CNT);	
		if(prtd->channels==1)
			unWriteCnt/=2; //because AFE HW always 2 channel


		pr_debug("DAC data len = %d \r\n",unWriteCnt);


		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+AFEDacCntInPeriod;

		if((AFEDacCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{

			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod);++i)
			{
				 afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
				 DACTempBuf[i] = pcmPtr[i];
				if(prtd->channels==1)
				{
					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
					DACTempBuf[i] = pcmPtr[i];
				}
			}
			
			////////////// AEC ////////////////
			//audio_ring_buf_write( &g_dac_ring,DACTempBuf,(prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod);	
			aec_wq_put_dac_data( 	g_aec_work_handle,		//awc wq handle
									DACTempBuf,			//source data buffer
									(prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod		//source data length
								);
			
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
				afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
				DACTempBuf[i] = pcmPtr[i];
				if(prtd->channels==1)
				{
					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
					DACTempBuf[i] = pcmPtr[i];
				}
			}

			////////////// AEC ////////////////
			//audio_ring_buf_write( &g_dac_ring,DACTempBuf,unWriteCnt);	
 			aec_wq_put_dac_data( 	g_aec_work_handle,		//awc wq handle
									DACTempBuf,			//source data buffer
									unWriteCnt				//source data length
								);
			
			AFEDacCntInPeriod+=unWriteCnt;
		}
		params->pdc->xpr = prtd->period_ptr+AFEDacCntInPeriod*params->pdc_xfer_size;
	}
	else if(afe_sr &AFE_INT_FIFO_FULL )
	{

		pr_warn("AFE FIFO FULL\n");

	}
//#endif 

#undef AFESentCntInPeriod
#undef AFEDacCntInPeriod
}

static int AFESentCntInPeriod;
static volatile int g_AEC_Start_Flag=0;
#define ADC_FIFO_READY 0x01
#define DAC_FIFO_READY 0x02
static void vsnv3_pcm_dma_irq(u32 afe_sr,
	struct snd_pcm_substream *substream)
{
	#define AFESentCntInPeriod  (prtd->CurProcCntInPeriod)
	#define AFEDacCntInPeriod (prtd->CurProcCntInPeriod)
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
	struct ait_pcm_dma_params *params = prtd->params;

	int unReadCntInPeriod;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	AITPS_AFE   pAFE    = AITC_BASE_AFE;

	if(params->status == SNDRV_PCM_TRIGGER_STOP)
		return;
	
	if( 	g_aec_work_handle 
		&&(g_AEC_Start_Flag&ADC_FIFO_READY)
		&& (g_AEC_Start_Flag&DAC_FIFO_READY)
		&& (g_dac_status == SNDRV_PCM_TRIGGER_START) // if aec enabled 
		&& (g_adc_status == SNDRV_PCM_TRIGGER_START)
	  )
	{
		vsnv3_pcm_dma_irq_aec_enabled(afe_sr,substream);
		return;
	}
	
	if (afe_sr & AFE_INT_FIFO_REACH_UNRD_TH && prtd->period_ptr ){
		int i;
		unsigned short* pcmPtr,pcmPtrDummy;	

		g_AEC_Start_Flag |=  ADC_FIFO_READY;
		
		unReadCntInPeriod = afe_adc_fifo_reg(pAFE,RD_TH)/params->pdc_xfer_size;	//For 2 bye per sample
		pr_debug("ADC data len = %d \r\n",unReadCntInPeriod);

		pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+AFESentCntInPeriod);

		if ((AFESentCntInPeriod+unReadCntInPeriod)>= (prtd->period_size / params->pdc_xfer_size))
		{
			//for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFESentCntInPeriod);++i)
			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFESentCntInPeriod);i+=prtd->channels)
			{
#if defined(CONFIG_MIC_REVERSE)
				if(prtd->channels==1)
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
				}
				else 
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
				}
#else
				if(prtd->channels==1)
				{
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
				}
				else 
				{
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
				}
#endif
			}
		
			unReadCntInPeriod -= ((prtd->period_size/ params->pdc_xfer_size)-AFESentCntInPeriod);
			AFESentCntInPeriod = 0;

			
			prtd->period_ptr += prtd->period_size;			

			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			pcmPtr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/ params->pdc_xfer_size);
			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = prtd->period_size / params->pdc_xfer_size;

			
			snd_pcm_period_elapsed(substream);
		}

		//for(i=0;i<unReadCntInPeriod;++i)
		for(i=0;i<unReadCntInPeriod;i+=prtd->channels)
		{
#if defined(CONFIG_MIC_REVERSE)
				if(prtd->channels==1)
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
				}
				else 
				{
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
				}
#else
				if(prtd->channels==1)
				{
					pcmPtrDummy = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
				}
				else 
				{
					pcmPtr[i+1] = afe_adc_fifo_reg(pAFE,DATA);
					pcmPtr[i] = afe_adc_fifo_reg(pAFE,DATA);
				}
#endif
		}

		AFESentCntInPeriod+=unReadCntInPeriod;	
		params->pdc->xpr = prtd->period_ptr+AFESentCntInPeriod*params->pdc_xfer_size;

	}
	else if (afe_sr & AFE_INT_FIFO_REACH_UNWR_TH && prtd->period_ptr )
	{
		int i;
		unsigned short* pcmPtr;
		u32 unWriteCnt = (u32) afe_dac_fifo_reg(pAFE,UNWR_CNT);	

		g_AEC_Start_Flag |=  DAC_FIFO_READY;
		
		if(prtd->channels==1)
			unWriteCnt/=2; //because AFE HW always 2 channel


		pr_debug("DAC data len = %d \r\n",unWriteCnt);


		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+AFEDacCntInPeriod;

		if((AFEDacCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{

			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod);++i)
			{
				 afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
				if(prtd->channels==1)
					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];					
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
				afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
				if(prtd->channels==1)
					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];			
			}
			AFEDacCntInPeriod+=unWriteCnt;
		}
		params->pdc->xpr = prtd->period_ptr+AFEDacCntInPeriod*params->pdc_xfer_size;
	}
	else if(afe_sr &AFE_INT_FIFO_FULL )
	{

		pr_warn("AFE FIFO FULL\n");

	}
#undef AFESentCntInPeriod
#undef AFEDacCntInPeriod
}


/*--------------------------------------------------------------------------*\
 * PCM operations
\*--------------------------------------------------------------------------*/
static int vsnv3_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct vsnv3_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	AITPS_AUD   pAFE    = AITC_BASE_AUD;

	/* this may get called several times by oss emulation
	 * with different params */

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	prtd->params->dma_intr_handler = vsnv3_pcm_dma_irq;

	prtd->dma_buffer = runtime->dma_addr;
	//prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);
	prtd->dma_buffer_end = runtime->dma_addr + (runtime->dma_bytes/prtd->period_size)*prtd->period_size;
	prtd->channels = params_channels(params);
	MIC_CHANNELS = prtd->channels;
	pr_info("vsnv3-afes-pcm: "
		"hw_params: DMA for %s initialized "
		"(dma_bytes=%u, period_size=%u , pdc_xfer_size = %u)\n",
		prtd->params->name,
		runtime->dma_bytes,
		prtd->period_size,
		prtd->params->pdc_xfer_size);
	return 0;
}

static int vsnv3_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
	struct atmel_pcm_dma_params *params = prtd->params;

	if (params != NULL) {
	}

	return 0;
}

static int vsnv3_pcm_prepare(struct snd_pcm_substream *substream)
{
	AITPS_AFE   pAFE    = AITC_BASE_AFE;
	return 0;
}

//DEFINE_SEMAPHORE(g_aec_trigger_mutex); // this mutex is used to protect aec and aec work queue when it start/stop
static void aec_ctl_func(struct work_struct* ws)
{
	aec_ctl_work *ctl_ws = container_of(ws,aec_ctl_work,ws);


	switch(ctl_ws->cmd_id)
	{
	case AEC_CTL_START:
		{
			aec_work_handle ** aec_handle = (aec_work_handle **)ctl_ws->param;	
			pr_info("AEC start. +\r\n");				
			*aec_handle = aec_wq_start();
			pr_info("AEC start. -\r\n");
		}
		break;
	case AEC_CTL_STOP:
		{
			aec_work_handle *aec_handle = (aec_work_handle *)ctl_ws->param;			
			pr_info("AEC wq stop+. \r\n");
			aec_wq_stop(aec_handle);
			pr_info("AEC wq stop-. \r\n");
		}
		break;
	}
	return 0;
}

aec_ctl_work g_aec_ctl_work[2];
static int vsnv3_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct vsnv3_runtime_data *prtd = rtd->private_data;
	struct ait_pcm_dma_params *params = prtd->params;
	
	int ret = 0;
	AITPS_AFE   pAFE    = AITC_BASE_AFE;
	//static int start =0;
	int dir = substream->stream; 
		
	//printk("%s\r\n",__func__);
	
	pr_debug("vsnv3_pcm_trigger: cmd = %d\r\n",cmd);	
	pr_debug("atmel-pcm:buffer_size = %ld,"
		"dma_area = %p, dma_bytes = %u\n",
		rtd->buffer_size, rtd->dma_area, rtd->dma_bytes);
	//pr_info("ait-pcm trigger: ");


	switch (cmd) {
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		case SNDRV_PCM_TRIGGER_START:
			//if (start ==0)
			{
                		prtd->period_ptr = prtd->dma_buffer;
                		params->pdc->xpr = prtd->period_ptr;
                		params->pdc->xcr = 0;
                		params->pdc->xnpr = prtd->period_ptr;
                		params->pdc->xncr = 0;
				prtd->CurProcCntInPeriod = 0;
				
				pr_debug(	"ait-pcm: trigger: "
							"period_ptr=%lx,  AFEFIFO=%u, AFECPU=%u\n",
							(unsigned long)prtd->period_ptr,
							//pAFE->AFE_FIFO_CPU_INT_EN,//AFE_INT_FIFO_REACH_UNRD_TH;
							dir == SNDRV_PCM_STREAM_CAPTURE ? afe_adc_fifo_reg(pAFE, CPU_INT_EN) : afe_dac_fifo_reg(AITC_BASE_AFE, CPU_INT_EN),
							pAFE->AFE_CPU_INT_EN
						);
		                //start = 1;
			}

			//spin_lock_irq(&ssc_p->lock);
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				MMPF_Audio_SetMux(ADC_TO_AFE_FIFO,MMP_TRUE);
				if(params->status == SNDRV_PCM_TRIGGER_STOP)
					MMPF_Audio_EnableAFEClock(MMP_TRUE,params->sampling_rate,AFE_PATH_ADC);

				if(g_dac_status==SNDRV_PCM_TRIGGER_START && g_audio_mode==AUDIO_ENH_AEC &&   !g_aec_work_handle)
				{
					pr_debug("AEC start1. +\r\n");									
					aec_ctl_work* ctl_work = &g_aec_ctl_work[0];
					g_AEC_Start_Flag = 0;
					INIT_WORK((struct work_struct*)ctl_work,aec_ctl_func);
					ctl_work->cmd_id = AEC_CTL_START;
					ctl_work->param = (void*)&g_aec_work_handle;
					schedule_work((struct work_struct*)ctl_work);					
				}
				g_adc_status = SNDRV_PCM_TRIGGER_START;
				
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				MMPF_Audio_SetMux(AFE_FIFO_TO_DAC,MMP_TRUE);
				//MMPF_Audio_EnableAFEClock(MMP_TRUE,params_rate(params),AFE_PATH_DAC);
				if(params->status == SNDRV_PCM_TRIGGER_STOP)
					MMPF_Audio_EnableAFEClock(MMP_TRUE,params->sampling_rate,AFE_PATH_DAC);
				
				if(g_adc_status==SNDRV_PCM_TRIGGER_START &&  g_audio_mode==AUDIO_ENH_AEC && !g_aec_work_handle)
				{
					pr_debug("AEC start2. +\r\n");					
					aec_ctl_work* ctl_work = &g_aec_ctl_work[0];
					g_AEC_Start_Flag = 0;
					INIT_WORK((struct work_struct*)ctl_work,aec_ctl_func);
					ctl_work->cmd_id = AEC_CTL_START;
					ctl_work->param = (void*)&g_aec_work_handle;
					schedule_work((struct work_struct*)ctl_work);
				}
				g_dac_status = SNDRV_PCM_TRIGGER_START;
				//control amp for intermittent data
				ait_afe_power_on_amp(1);
			}
			//spin_unlock_irq(&ssc_p->lock);
					
			params->status = SNDRV_PCM_TRIGGER_START;
			
			pr_debug("atmel-pcm: trigger: "
						"period_ptr=%lx,  AFEFIFO=%u, AFECPU=%u\n",
						(unsigned long)prtd->period_ptr,
						dir == SNDRV_PCM_STREAM_CAPTURE ? afe_adc_fifo_reg(pAFE, CPU_INT_EN) : afe_dac_fifo_reg(pAFE, CPU_INT_EN),
						pAFE->AFE_CPU_INT_EN
					);
		break;		/* SNDRV_PCM_TRIGGER_START */

		case SNDRV_PCM_TRIGGER_STOP:
		{
			struct task_struct	*aec_stop_task;
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				MMPF_Audio_SetMux(ADC_TO_AFE_FIFO,MMP_FALSE);
				if(params->status == SNDRV_PCM_TRIGGER_START)
					MMPF_Audio_EnableAFEClock(MMP_FALSE,0,AFE_PATH_ADC);

				g_adc_status = SNDRV_PCM_TRIGGER_STOP;
				
				if(g_aec_work_handle)
				{
					aec_ctl_work* ctl_work = &g_aec_ctl_work[1];
					aec_work_handle *tmp = g_aec_work_handle;
					
					//spin_lock_irqsave(&g_aec_stop_lock,g_aec_lock_flag);
					down(&g_aec_stop_lock); //it is ok to use semaphore here, because capture stop never trigger from interrupt context  
					g_aec_work_handle = 0; //set g_aec_work_handle to block new incoming work
					//spin_unlock_irqrestore(&g_aec_stop_lock,g_aec_lock_flag);
					up(&g_aec_stop_lock);
					
					g_AEC_Start_Flag = 0;
					INIT_WORK((struct work_struct*)ctl_work,aec_ctl_func);
					ctl_work->cmd_id = AEC_CTL_STOP;
					ctl_work->param = (void*)tmp;
					schedule_work((struct work_struct*)ctl_work);
				}
				
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				//must clean FIFO when stop. Our audio engine will never stop after enabling ADC.
				//we close the amp so we don't need to clean fifo, 2017/03/01
				//int i = 0;
				//for(i=0;i<512;i++)
				//{
				//	afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = 0;
				//}


				MMPF_Audio_SetMux(AFE_FIFO_TO_DAC,MMP_FALSE);
				if(params->status == SNDRV_PCM_TRIGGER_START)
					MMPF_Audio_EnableAFEClock(MMP_FALSE,0,AFE_PATH_DAC);

				g_dac_status = SNDRV_PCM_TRIGGER_STOP;	
				
				if(g_aec_work_handle)
				{
					g_AEC_Start_Flag &= ~DAC_FIFO_READY;
				}

				//control amp for intermittent data
				ait_afe_power_on_amp(0);
			}
			params->status = SNDRV_PCM_TRIGGER_STOP;
		}
		break;
			

		case SNDRV_PCM_TRIGGER_RESUME:
			BUG_ON(0);
			break;
			
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
				afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AFE_INT_FIFO_REACH_UNRD_TH;
				afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AUD_ADC_INT_EN;
				break;
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
				afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AFE_INT_FIFO_REACH_UNWR_TH;
				afe_adc_fifo_reg(pAFE,CPU_INT_EN) |= AUD_DAC_INT_EN;		
			}
		break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t vsnv3_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct vsnv3_runtime_data *prtd = runtime->private_data;
	struct ait_pcm_dma_params *params = prtd->params;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	ptr = params->pdc->xpr;
	x = bytes_to_frames(runtime, ptr - prtd->dma_buffer);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int vsnv3_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct vsnv3_runtime_data *prtd;
	int ret = 0;

	snd_soc_set_runtime_hwparams(substream, &vsnv3_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
	
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct vsnv3_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		pr_err("%s: prtd == NULL\r\n",__FUNCTION__);		
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;

 out:
	return ret;
}

static int vsnv3_pcm_close(struct snd_pcm_substream *substream)
{
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;

	kfree(prtd);
	return 0;
}

static int vsnv3_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_ops ait_afe_pcm_ops = {
	.open		= vsnv3_pcm_open,
	.close		= vsnv3_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= vsnv3_pcm_hw_params,
	.hw_free	= vsnv3_pcm_hw_free,
	.prepare	= vsnv3_pcm_prepare,
	.trigger	= vsnv3_pcm_trigger,
	.pointer	= vsnv3_pcm_pointer,
	.mmap		= vsnv3_pcm_mmap,
};


/*--------------------------------------------------------------------------*\
 * ASoC platform driver
\*--------------------------------------------------------------------------*/
static int vsnv3_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;	//created by soc_new_pcm->snd_pcm_new
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = (u64*)0xffffffff;//&atmel_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = (u64)0xffffffff;

	if (dai->driver->playback.channels_min) {
		ret = vsnv3_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		pr_debug("vsnv3-pcm:"
				"Allocating PCM capture DMA buffer\n");
		ret = vsnv3_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

static void vsnv3_pcm_free_dma_buffers(struct snd_pcm *pcm)
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
static int vsnv3_pcm_suspend(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct vsnv3_runtime_data *prtd;
	struct atmel_pcm_dma_params *params;

	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;

	/* disable the PDC and save the PDC registers */
	/*
	ssc_writel(params->ssc->regs, PDC_PTCR, params->mask->pdc_disable);
	prtd->pdc_xpr_save = 0;
	prtd->pdc_xcr_save = 0;
	prtd->pdc_xnpr_save = 0;
	prtd->pdc_xncr_save = 0;
	*/
	return 0;
}

static int vsnv3_pcm_resume(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct vsnv3_runtime_data *prtd;
	struct ait_pcm_dma_params *params;

	if (!runtime)
		return 0;

	prtd = runtime->private_data;
	params = prtd->params;

	/* restore the PDC registers and enable the PDC */
	/*
	ssc_writex(params->ssc->regs, params->pdc->xpr, prtd->pdc_xpr_save);
	ssc_writex(params->ssc->regs, params->pdc->xcr, prtd->pdc_xcr_save);
	ssc_writex(params->ssc->regs, params->pdc->xnpr, prtd->pdc_xnpr_save);
	ssc_writex(params->ssc->regs, params->pdc->xncr, prtd->pdc_xncr_save);
	ssc_writel(params->ssc->regs, PDC_PTCR, params->mask->pdc_enable);
	*/
	return 0;
}
#else
#define vsnv3_pcm_suspend	NULL
#define vsnv3_pcm_resume	NULL
#endif

static struct snd_soc_platform_driver ait_soc_platform = {
	.ops		= &ait_afe_pcm_ops,
	.pcm_new	= vsnv3_pcm_new,
	.pcm_free	= vsnv3_pcm_free_dma_buffers,
	.suspend	= vsnv3_pcm_suspend,
	.resume		= vsnv3_pcm_resume,
};

static int __devinit vsnv3_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &ait_soc_platform);
}

static int __devexit vsnv3_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver vsnv3_pcm_driver = {
	.driver = {
			.name = "vsnv3afe-pcm-audio",
			.owner = THIS_MODULE,
	},

	.probe = vsnv3_soc_platform_probe,
	.remove = __devexit_p(vsnv3_soc_platform_remove),
};

module_platform_driver(vsnv3_pcm_driver)

MODULE_AUTHOR("Vincent Chen <vincent_chen@a-i-t.com.tw>,Andy Hsieh <andy_hsieh@a-i-t.com.tw>,");
MODULE_DESCRIPTION("AIT PCM module");
MODULE_LICENSE("GPL");
