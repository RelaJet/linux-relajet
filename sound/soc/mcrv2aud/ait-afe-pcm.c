/*
 * vsnv3-pcm.c  --  ALSA PCM interface for the AIT VSNV3 SoC.
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
#include "ait_aud_dai.h"

#include <mach/mmp_reg_audio.h>
#include <mach/mmpf_mcrv2_audio_ctl.h>
#include <mach/mcrv2_afe.h>
#if defined(CONFIG_AIT_FAST_BOOT)
#include <mach/ait_alsa_ipc.h>
#endif
extern AUDIO_IPC_MODE audio_ipc_mode ;


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
	.period_bytes_min	= 8*100*2,//512, //160,
	.period_bytes_max	= 48*1000*2*2, //8192*2,
	.periods_min		= 100,
	.periods_max		= 1000,//1024,
	#if defined(CONFIG_AIT_FAST_BOOT)
	.buffer_bytes_max	= 1*192000,////256 * 1024,
	#else
	.buffer_bytes_max	= 2*192000,////256 * 1024,
	
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
	/* no used
	u32 pdc_xpr_save;
	u32 pdc_xcr_save;
	u32 pdc_xnpr_save;
	u32 pdc_xncr_save;
	*/
	u8   channels;
	u32 CurProcCntInPeriod;
	#if defined(CONFIG_AIT_FAST_BOOT)
	snd_pcm_sframes_t spk_wr_ptr ;
	#endif
};

static u32 audio_ipc_proc_samples[2] = {0,0} ;
void set_pcm_proc_samples(int path ,int samples)
{
  audio_ipc_proc_samples[path] = samples ;  
}
EXPORT_SYMBOL(set_pcm_proc_samples);

/*--------------------------------------------------------------------------*\
 * Helper functions
\*--------------------------------------------------------------------------*/
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
					&buf->addr, GFP_KERNEL  );
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
u32 totalbytecnt = 0;
//static int AFESentCntInPeriod;

unsigned long t ; 
unsigned long long samples =0 ; 
int numIRQ=0;	

static void vsnv3_pcm_dma_irq(u32 afe_sr,
	struct snd_pcm_substream *substream)
{
  // ADC
	#define AFESentCntInPeriod  (prtd->CurProcCntInPeriod)
	#define AFESentBytesInPeriod  (prtd->CurProcCntInPeriod * params->pdc_xfer_size )
	#define PeriodCnt             (prtd->period_size/ params->pdc_xfer_size)
	#define Bytes2Cnt(bytes)      ( (bytes) / params->pdc_xfer_size )
  // DAC
	#define AFEDacCntInPeriod (prtd->CurProcCntInPeriod)
	#define Cnt2Bytes(cnt)        ( (cnt) * params->pdc_xfer_size )
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
	struct ait_pcm_dma_params *params = prtd->params;

	int unReadCntInPeriod=0;
	int unWriteCnt = 0 ;
	
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	AITPS_AFE   pAFE    = AITC_BASE_AFE;
	

	
	if(params->status == SNDRV_PCM_TRIGGER_STOP)
		return;
	
	if (afe_sr & AFE_INT_FIFO_REACH_UNRD_TH && prtd->period_ptr ){
		int i;
		unsigned short* pcmPtr,pcmPtrDummy;

    if(audio_ipc_mode==AUDIO_IPC_NONE) {
		  unReadCntInPeriod = afe_adc_fifo_reg(pAFE,RD_TH) ; // per channel
	  }
	  else {
#if defined(CONFIG_AIT_FAST_BOOT) 	    
		  unReadCntInPeriod = audio_ipc_proc_samples[ALSA_MIC]  ;	// per channel
#endif		  
	  }
	  // single channel
    if(prtd->channels==1) unReadCntInPeriod = unReadCntInPeriod >> 1;
    
		pcmPtr = ((unsigned short*)buf->area + Bytes2Cnt(prtd->period_ptr-prtd->dma_buffer) + AFESentCntInPeriod);

		//pr_info("dot per channel = %d,curptr : 0x%08x\r\n",unReadCntInPeriod,pcmPtr);

		if ((AFESentCntInPeriod+unReadCntInPeriod)>= PeriodCnt )
		{
			
			if(audio_ipc_mode==AUDIO_IPC_NONE) {
  			for(i=0;i<(PeriodCnt - AFESentCntInPeriod);i+=prtd->channels)
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
		  }
		  else{
		  //  alsa_ipc_rd_data(  Bytes2Cnt(PeriodCnt - AFESentCntInPeriod) ) ;  
		  }
			unReadCntInPeriod -= (PeriodCnt-AFESentCntInPeriod);
			AFESentCntInPeriod = 0;
			prtd->period_ptr += prtd->period_size;			
			if (prtd->period_ptr >= prtd->dma_buffer_end)
				prtd->period_ptr = prtd->dma_buffer;

			pcmPtr = (unsigned short*)buf->area + Bytes2Cnt(prtd->period_ptr-prtd->dma_buffer) ;

			params->pdc->xpr = prtd->period_ptr;
			params->pdc->xcr = PeriodCnt ;
			snd_pcm_period_elapsed(substream);
		}

		//for(i=0;i<unReadCntInPeriod;++i)
		if(audio_ipc_mode==AUDIO_IPC_NONE) {
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
		} 
		else {
    //  alsa_ipc_rd_data( Bytes2Cnt(unReadCntInPeriod) );
		}
		AFESentCntInPeriod+=unReadCntInPeriod;
		params->pdc->xpr = prtd->period_ptr+ AFESentBytesInPeriod ;
#if defined(CONFIG_AIT_FAST_BOOT)
		// only need to update wr pointer which sync to cpub
		if(audio_ipc_mode==AUDIO_IPC_EN) {
      alsa_ipc_rd_mic_data(  Cnt2Bytes( audio_ipc_proc_samples[ALSA_MIC]) >> (prtd->channels==1) ) ;  
    }
#endif
	}
	else if (afe_sr & AFE_INT_FIFO_REACH_UNWR_TH && prtd->period_ptr )
	{
  	struct snd_pcm_runtime *runtime = substream->runtime;
	  
		int i;
		unsigned short* pcmPtr;
    if(audio_ipc_mode==AUDIO_IPC_NONE) {
		  unWriteCnt = afe_dac_fifo_reg(pAFE,UNWR_CNT);	
	  }
	  else {
#if defined(CONFIG_AIT_FAST_BOOT)
	    unWriteCnt = audio_ipc_proc_samples[ALSA_SPK] ;
#endif	    
	  }
		if(prtd->channels==1)
			unWriteCnt/=2; //because AFE HW always 2 channel


		pr_debug("DAC data len = %d \r\n",unWriteCnt);


		pcmPtr = (unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size+AFEDacCntInPeriod;

		if((AFEDacCntInPeriod+unWriteCnt)>= (prtd->period_size / params->pdc_xfer_size))
		{
      if(audio_ipc_mode==AUDIO_IPC_NONE) {
  			for(i=0;i<((prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod);++i)
  			{
  				afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
  				if(prtd->channels==1)
  					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];					
  			}
		  }
		  else {
		    int wr_cnt = (prtd->period_size/ params->pdc_xfer_size)-AFEDacCntInPeriod;
#if defined(CONFIG_AIT_FAST_BOOT) 		    
//		    alsa_ipc_wr_spk_data( Cnt2Bytes(wr_cnt) ) ;
#endif		    
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
		  if(audio_ipc_mode==AUDIO_IPC_NONE) {
  			for(i=0;i<unWriteCnt;++i)
  			{
  				afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];
  				if(prtd->channels==1)
  					afe_dac_fifo_reg(AITC_BASE_AFE,DATA) = pcmPtr[i];			
  			}
		  }
		  else {
#if defined(CONFIG_AIT_FAST_BOOT) 		    
//		    alsa_ipc_wr_spk_data( Cnt2Bytes(unWriteCnt) ) ;  
#endif		    
		  }
			AFEDacCntInPeriod+=unWriteCnt;
		}
		
		//pr_info( "[%d]",i);
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
	struct ait_pcm_dma_params *ait_params ;//= prtd->params;


	AITPS_AUD   pAFE    = AITC_BASE_AUD;

	/* this may get called several times by oss emulation
	 * with different params */

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	prtd->params->dma_intr_handler = vsnv3_pcm_dma_irq;
	//prtd->params->dma_intr_handler = mcrv2_afe_pcm_in_irq;

	prtd->dma_buffer = runtime->dma_addr;
	//prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);
	prtd->dma_buffer_end = runtime->dma_addr + (runtime->dma_bytes/prtd->period_size)*prtd->period_size;
	prtd->channels = params_channels(params);
#if defined(CONFIG_AIT_FAST_BOOT)
  if(audio_ipc_mode==AUDIO_IPC_EN) {
    /*
    SEAN:
    */
    if(substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
      alsa_ipc_resize_ringbuf(ALSA_MIC,prtd->dma_buffer_end-prtd->dma_buffer);
    }
    else {
      alsa_ipc_resize_ringbuf(ALSA_SPK,prtd->dma_buffer_end-prtd->dma_buffer);
		  alsa_ipc_reset_spk_wr_data( frames_to_bytes(rtd,runtime->control->appl_ptr ) );
		  prtd->spk_wr_ptr = runtime->control->appl_ptr ;
      
    }
  }
  //prtd->spk_wr_ptr = 0 ;
#endif
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
	//int dir = substream->stream; 
	//struct snd_pcm_runtime *rtd = substream->runtime;
	
	return 0;
}

#if defined(CONFIG_AIT_FAST_BOOT)
static void update_rtos_spk_wr_ptr(struct snd_pcm_runtime *runtime)
{
	struct vsnv3_runtime_data *prtd = runtime->private_data;
  signed long wr_bytes ;
//  pr_info("spk.wr : %d , new.ptr : %d\n",prtd->spk_wr_ptr ,runtime->control->appl_ptr);

  if( prtd->spk_wr_ptr <= runtime->control->appl_ptr ) {
    //alsa_ipc_wr_spk_data( ) ;    
    wr_bytes = frames_to_bytes( runtime,runtime->control->appl_ptr - prtd->spk_wr_ptr) ;
  }
  else {
    //alsa_ipc_wr_spk_data(  ) ;
    wr_bytes = frames_to_bytes(runtime ,runtime->control->appl_ptr) ;
  }
  if(wr_bytes) {
    alsa_ipc_wr_spk_data(wr_bytes);
  }
  prtd->spk_wr_ptr = runtime->control->appl_ptr ;
  
}
#endif

static int vsnv3_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct vsnv3_runtime_data *prtd = rtd->private_data;
	struct ait_pcm_dma_params *params = prtd->params;
  snd_pcm_sframes_t avail ;
	
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
#if defined(CONFIG_AIT_FAST_BOOT)  			
			if(dir == SNDRV_PCM_STREAM_PLAYBACK) {
			  // here to re-sync spk data rw pointer
			  //alsa_ipc_reset_spk_wr_data( frames_to_bytes(rtd,rtd->control->appl_ptr ) );
			  //prtd->spk_wr_ptr = rtd->control->appl_ptr ;
			  update_rtos_spk_wr_ptr(rtd);
			  alsa_ipc_reset_spk_rd_data();
			}
#endif			
        prtd->period_ptr = prtd->dma_buffer;
        params->pdc->xpr = prtd->period_ptr;
        params->pdc->xcr = 0;
        params->pdc->xnpr = prtd->period_ptr;
        params->pdc->xncr = 0;
        prtd->CurProcCntInPeriod = 0;				
      
			//spin_lock_irq(&ssc_p->lock);
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
			  if(audio_ipc_mode==AUDIO_IPC_NONE) {
  				MMPF_Audio_SetMux(ADC_TO_AFE_FIFO,MMP_TRUE);
  				if(params->status == SNDRV_PCM_TRIGGER_STOP)
  					MMPF_Audio_EnableAFEClock(MMP_TRUE,params->sampling_rate,AFE_PATH_ADC);
        }
        else {
#if defined(CONFIG_AIT_FAST_BOOT)           
          pr_info("mic.t\n");
          alsa_ipc_trigger_start(ALSA_MIC,prtd->period_size / params->pdc_xfer_size);
#endif          
        }
				t =0 ; 
				samples =0 ; 
				numIRQ=0;	
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
			  if(audio_ipc_mode==AUDIO_IPC_NONE) {
  				MMPF_Audio_SetMux(AFE_FIFO_TO_DAC,MMP_TRUE);
  				//MMPF_Audio_EnableAFEClock(MMP_TRUE,params_rate(params),AFE_PATH_DAC);
  				if(params->status == SNDRV_PCM_TRIGGER_STOP)
  					MMPF_Audio_EnableAFEClock(MMP_TRUE,params->sampling_rate,AFE_PATH_DAC);
				}
				else {
#if defined(CONFIG_AIT_FAST_BOOT) 				  
				  pr_info("->spk.t\n");
				  alsa_ipc_trigger_start(ALSA_SPK,prtd->period_size / params->pdc_xfer_size);
#endif				  
				}
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
			//spin_lock_irq(&ssc_p->lock);
			if(dir == SNDRV_PCM_STREAM_CAPTURE)
			{
			  if(audio_ipc_mode==AUDIO_IPC_NONE) {
  				MMPF_Audio_SetMux(ADC_TO_AFE_FIFO,MMP_FALSE);
  				if(params->status == SNDRV_PCM_TRIGGER_START)
  					MMPF_Audio_EnableAFEClock(MMP_FALSE,0,AFE_PATH_ADC);
				}
				else {
#if defined(CONFIG_AIT_FAST_BOOT) 				  
				  //pr_info("tbd:vsnv3_pcm_trigger.mic.stop\n");
				  alsa_ipc_trigger_stop_noack(ALSA_MIC);
#endif				  
				}
			}
			else if(dir == SNDRV_PCM_STREAM_PLAYBACK)
			{
			  if(audio_ipc_mode==AUDIO_IPC_NONE) {
  				MMPF_Audio_SetMux(AFE_FIFO_TO_DAC,MMP_FALSE);
  				if(params->status == SNDRV_PCM_TRIGGER_START)
  					MMPF_Audio_EnableAFEClock(MMP_FALSE,0,AFE_PATH_DAC);
				}
				else {
#if defined(CONFIG_AIT_FAST_BOOT) 		
          // can't call here , may in interrupt!		  
				  alsa_ipc_trigger_stop_noack(ALSA_SPK);
#endif				  
				  
				}
			}
			params->status = SNDRV_PCM_TRIGGER_STOP;
			//spin_unlock_irq(&ssc_p->lock);
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

  if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK ) {
    //pr_info("spk.pointer:%d frames\n",x);  
#if defined(CONFIG_AIT_FAST_BOOT)
    update_rtos_spk_wr_ptr(runtime);
#endif
  }
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
