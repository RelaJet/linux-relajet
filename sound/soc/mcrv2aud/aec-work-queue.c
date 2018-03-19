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
#include "aec-work-queue.h"

#define AEC_OBJ_SZ    sizeof(struct cpu_comm_transfer_data )
#define AEC_SEND(id,data)          do {                \
                                CpuComm_DataSend( id, (MMP_UBYTE*)data, AEC_OBJ_SZ,0 );         \
                                CpuComm_DataReceive( id, (MMP_UBYTE*)data,  AEC_OBJ_SZ,0,0);   \
                            } while(0)


extern int g_aec_dbg;
extern int MIC_CHANNELS;
//---------- AEC APIs ------------
int cpucomm_aec_register(cpub_aec_handle* aec)
{
	CPU_COMM_ERR ret;
	aec->id = CPU_COMM_ID_AEC;
	ret = CpuComm_RegisterEntry(aec->id, CPU_COMM_TYPE_DATA);
	if(ret!=CPU_COMM_ERR_NONE)
	{
		pr_err("Failed to register CPUB AEC.\r\n");
	}
	return ret;	
}

int cpucomm_aec_unregister(cpub_aec_handle* aec)
{
	CpuComm_UnregisterEntry(aec->id);
	return 0;
}

//__attribute__((optimize("O0")))
void cpub_aec_init(cpub_aec_handle* aec)
{
	struct cpu_comm_transfer_data data;

	pr_debug("%s +",__func__);
	data.command = AEC_INIT;
	data.phy_addr	 = 0;
	data.size		= 0;
	data.seq		= MIC_CHANNELS; //0;
	data.flag		= CPUCOMM_FLAG_CMDSND;
	
	//MMPF_MMU_FlushDCacheMVA( (unsigned long)&data, sizeof(data) );
#if 1
  AEC_SEND(aec->id,&data);
#else	
	
	// Send the data to CPU_A and wait the result
	CpuComm_DataSend( aec->id, (MMP_UBYTE*)&data, sizeof(data),0 );

	pr_debug("Wait for buffer info Rcv\n");
	
	CpuComm_ReceiveDataStart( aec->id, (MMP_UBYTE*)&data,  sizeof(data) );
	//MMPF_MMU_FlushDCacheMVA( (MMP_ULONG)&data,  sizeof(data) );

	CpuComm_ReceiveDataEnd( aec->id, data.command);
#endif	
	pr_debug("%s -",__func__);

	return 0;
}

void cpub_aec_run(cpub_aec_handle* aec,AEC_run_t* aec_sample,unsigned long aec_sample_phys,int dbg)
{
	struct cpu_comm_transfer_data data;

	//pr_debug("%s +",__func__);

	//aec_sample->AEC_Process_Samples = AEC_PERIOD_SAMPLE;
	if(!dbg) {
		data.command = AEC_RUN;
	}
	else {
		data.command = AEC_RUN | AEC_DBG;
		
	}
	data.phy_addr	 = aec_sample_phys;
	data.size		= sizeof(AEC_run_t);
	data.seq		= 0;
	data.flag		= CPUCOMM_FLAG_CMDSND;
	
	//MMPF_MMU_FlushDCacheMVA( (unsigned long)&data, sizeof(data) );
#if 1
  AEC_SEND(aec->id,&data);
#else	
	
	// Send the data to CPU_A and wait the result
	CpuComm_DataSend( aec->id, (MMP_UBYTE*)&data, sizeof(data),0 );

	CpuComm_ReceiveDataStart( aec->id, 0, 0);
	//MMPF_MMU_FlushDCacheMVA( (MMP_ULONG)&data,  sizeof(data) );


	CpuComm_ReceiveDataEnd( aec->id, data.command);
	//pr_debug("%s -",__func__);
#endif
	return 0;
}

//static void aec_data_transfer_routine(struct work_struct* ws);

/////////////// aec work queue api //////////////
typedef struct 
{
	void* addr_virt;
	dma_addr_t addr_phys;
	size_t max_size;
}cpu_shared_memory;
cpu_shared_memory g_cpu_shared_mem;

static int __init init_cpu_share_memory(void)
{
	g_cpu_shared_mem.max_size = MAX_AEC_A2B_WORKS*(AEC_PERIOD_BUF_SIZE*2+sizeof(AEC_run_t));
	g_cpu_shared_mem.addr_virt = dma_alloc_coherent( 0,
												g_cpu_shared_mem.max_size,
												&g_cpu_shared_mem.addr_phys,
												GFP_KERNEL);
	pr_info("audio aec reserved memory, virt addr=0x%X, pyhs addr=0x%X, size=0x%X bytes", 
			(size_t) g_cpu_shared_mem.addr_virt,
			g_cpu_shared_mem.addr_phys,
			g_cpu_shared_mem.max_size
		    );
	
	return 0;
}

device_initcall(init_cpu_share_memory);

static void aec_data_run(struct work_struct *ws);
static int aec_wq_put_work(aec_work_handle* handle,audio_ring_buffer* adc_ring,audio_ring_buffer* dac_ring,aec_done_callback fp_cb,void* user_data);

#if 0
aec_work_handle* aec_wq_start()
{
	int n;
	unsigned long aec_reserved_virt = AIT_CPUB_AUDIO_VIRT_BASE;
	unsigned long aec_reserved_phys = AIT_CPUB_AUDIO_PHYS_BASE;	

	aec_work_handle* handle = kmalloc(sizeof(aec_work_handle),GFP_KERNEL);

	spin_lock_init(&handle->data_lock);
	
	//init aec
	cpucomm_aec_register(&handle->aec);
	cpub_aec_init(&handle->aec);

	//init ring buffer
	audio_ring_buf_init(&handle->dac_ring,SZ_16K);
	audio_ring_buf_init(&handle->adc_ring,SZ_16K);
	audio_ring_buf_init(&handle->output_ring,SZ_16K);
	
	//init work queue
	aec_a2b_work *aec_a2b_works = handle->works;
	
	for(n=0;n<MAX_AEC_A2B_WORKS;++n)
	{
		dma_addr_t phys_addr;
		INIT_WORK( (struct work_struct*)&aec_a2b_works[n], aec_data_run);
		
		aec_reserved_virt += (SZ_4K*3)*n;
		aec_reserved_phys += (SZ_4K*3)*n;

		aec_a2b_works[n].aec_data.buf_adc = aec_reserved_virt; //AIT_CPUB_AUDIO_VIRT_BASE + SZ_4K * n;
		aec_a2b_works[n].cpu_share_data->MIC_in_buffer  = aec_reserved_phys; 			//__virt_to_phys((unsigned long)aec_a2b_works[n].aec_data.buf_adc);

		aec_a2b_works[n].aec_data.buf_dac = aec_reserved_virt + SZ_4K; //aec_a2b_works[n].aec_data.buf_adc + SZ_4K;
		aec_a2b_works[n].cpu_share_data->SPK_out_buffer= aec_reserved_phys + SZ_4K;		//__virt_to_phys((unsigned long)aec_a2b_works[n].aec_data.buf_dac);
		
		aec_a2b_works[n].cpu_share_data = aec_reserved_virt + SZ_4K*2;
		aec_a2b_works[n].cpu_share_data_phys = aec_reserved_phys + SZ_4K*2;

		atomic_set(&aec_a2b_works[n].aec_data.users,0);
		aec_a2b_works[n].user_data = 0;
		aec_a2b_works[n].fp_cb = 0;
		aec_a2b_works[n].aec_api_handle = &handle->aec;
		aec_a2b_works[n].output_ring = &handle->output_ring;	
		

		aec_a2b_works[n].cpu_share_data->samples = AEC_PERIOD_SAMPLE;
		aec_a2b_works[n].cpu_share_data->AEC_Process_Samples = 0;

	}
	handle->wq = create_singlethread_workqueue("aec_a2b_wq");
	return handle;
}
#else
aec_work_handle* aec_wq_start()
{
	int n;
	//unsigned long aec_reserved_virt = AIT_CPUB_AUDIO_VIRT_BASE;
	//unsigned long aec_reserved_phys = AIT_CPUB_AUDIO_PHYS_BASE;	
	unsigned long aec_reserved_virt = (unsigned long) g_cpu_shared_mem.addr_virt;
	unsigned long aec_reserved_phys = (unsigned long) g_cpu_shared_mem.addr_phys;
	
	aec_work_handle* handle = kmalloc(sizeof(aec_work_handle),GFP_KERNEL);

	spin_lock_init(&handle->data_lock);
	
	//init aec
	cpucomm_aec_register(&handle->aec);
	cpub_aec_init(&handle->aec);

	//init ring buffer
	audio_ring_buf_init(&handle->dac_ring,SZ_16K);
	audio_ring_buf_init(&handle->adc_ring,SZ_16K);
	audio_ring_buf_init(&handle->output_ring,SZ_16K);
	
	//init work queue
	aec_a2b_work *aec_a2b_works = handle->works;
	
	for(n=0;n<MAX_AEC_A2B_WORKS;++n)
	{
		dma_addr_t phys_addr;
		INIT_WORK( (struct work_struct*)&aec_a2b_works[n], aec_data_run);

		//aec_reserved_virt += (SZ_4K*4)*n;
		//aec_reserved_phys += (SZ_4K*4)*n;
		//aec_a2b_works[n].cpu_share_data_phys = 0;
		//aec_a2b_works[n].cpu_share_data_virt = dma_alloc_coherent( 0, SZ_16K, &aec_a2b_works[n].cpu_share_data_phys, GFP_KERNEL);
		
		//pr_info("DMA buffer, pyhs=0x%X, virt=0x%X\r\n",aec_a2b_works[n].cpu_share_data_phys,(uint32_t)aec_a2b_works[n].cpu_share_data_virt);
		//aec_reserved_virt = (unsigned long) aec_a2b_works[n].cpu_share_data_virt ;
		//aec_reserved_phys = aec_a2b_works[n].cpu_share_data_phys;

		//aec_a2b_works[n].cpu_share_data = aec_reserved_virt + SZ_4K*2;
		//aec_a2b_works[n].cpu_share_data_phys = aec_reserved_phys + SZ_4K*2;
		
		aec_a2b_works[n].cpu_share_data = aec_reserved_virt;
		aec_a2b_works[n].cpu_share_data_phys = aec_reserved_phys;	
		aec_reserved_virt 	+= sizeof(AEC_run_t);
		aec_reserved_phys +=  sizeof(AEC_run_t);
		
		aec_a2b_works[n].aec_data.buf_adc = aec_reserved_virt;
		aec_a2b_works[n].cpu_share_data->MIC_in_buffer  = aec_reserved_phys;
		aec_reserved_virt 	+= AEC_PERIOD_BUF_SIZE;
		aec_reserved_phys +=  AEC_PERIOD_BUF_SIZE;		
		
		aec_a2b_works[n].aec_data.buf_dac = aec_reserved_virt;
		aec_a2b_works[n].cpu_share_data->SPK_out_buffer= aec_reserved_phys;
		aec_reserved_virt 	+= AEC_PERIOD_BUF_SIZE;
		aec_reserved_phys +=  AEC_PERIOD_BUF_SIZE;
		
		//pr_info("AEC buffer, pyhs=0x%X, virt=0x%X\r\n",aec_reserved_phys,aec_reserved_virt);
		
		atomic_set(&aec_a2b_works[n].aec_data.users,0);
		aec_a2b_works[n].user_data = 0;
		aec_a2b_works[n].fp_cb = 0;
		aec_a2b_works[n].aec_api_handle = &handle->aec;
		aec_a2b_works[n].output_ring = &handle->output_ring;	
		

		aec_a2b_works[n].cpu_share_data->samples = AEC_PERIOD_SAMPLE;
		aec_a2b_works[n].cpu_share_data->AEC_Process_Samples = 0;

	}
	handle->wq = create_singlethread_workqueue("aec_a2b_wq");
	return handle;
}
EXPORT_SYMBOL(aec_wq_start);
#endif

void aec_wq_stop(aec_work_handle* handle)
{
	int n;
	aec_a2b_work *aec_a2b_works = handle->works;
	
	flush_workqueue(handle->wq);
	destroy_workqueue(handle->wq);
	
	cpucomm_aec_unregister(&handle->aec);
	
	kfree(handle);
}
EXPORT_SYMBOL(aec_wq_stop);

int aec_wq_put_dac_data(aec_work_handle* handle, short *data, int n_samples)
{
	return audio_ring_buf_write( &handle->dac_ring, data, n_samples);
}
EXPORT_SYMBOL(aec_wq_put_dac_data);

//__attribute__((optimize("O0")))
int aec_wq_put_adc_data(aec_work_handle* handle, short *data, int n_samples,aec_done_callback fp_cb,void* user_dat)
{
	int n_sent = 0;
	audio_ring_buf_write( &handle->adc_ring, data, n_samples );	
	while( audio_ring_get_count(&handle->adc_ring)>=AEC_PERIOD_SAMPLE 
			&& audio_ring_get_count(&handle->dac_ring)>=AEC_PERIOD_SAMPLE)
	{
		int ret;
		ret = aec_wq_put_work(handle,&handle->adc_ring,&handle->dac_ring,fp_cb,(void*)user_dat);
	 	if(ret < 0)
		{
			n_sent += 0;
			break;
		}
		else
			n_sent += AEC_PERIOD_SAMPLE;
	}
	return n_sent;
}
EXPORT_SYMBOL(aec_wq_put_adc_data);
//#define  AEC_DEBUG_MODE 

#ifndef AEC_DEBUG_MODE
static void aec_data_run(struct work_struct *ws)
{
	aec_a2b_work *aec_work = container_of(ws,aec_a2b_work,ws);
	aec_work_data *data = &aec_work->aec_data;
#if 0	
	if(count++%62 == 0)
		pr_info("aec_data_transfer_routine, buf_adc=0x%X, buf_dac=0x%X, count=%d\r\n",
				(uint32_t) aec_work->cpu_share_data->MIC_in_buffer,
				(uint32_t) aec_work->cpu_share_data->SPK_out_buffer,
				count );
#endif 

	//pr_info("aec run.\r\n");
	
	cpub_aec_run(aec_work->aec_api_handle,aec_work->cpu_share_data,aec_work->cpu_share_data_phys,g_aec_dbg);
	audio_ring_buf_write(aec_work->output_ring,data->buf_adc,AEC_PERIOD_SAMPLE);
	if(aec_work->fp_cb)
	{
		aec_work->fp_cb(aec_work->output_ring,aec_work->user_data);
	}
	//--(data->users);
	atomic_dec(&data->users);
	
	return;
}
#else
static short aec_dbg_buf[AEC_PERIOD_SAMPLE];
static void aec_data_run(struct work_struct *ws)
{
	int n=0;
	aec_a2b_work *aec_work = container_of(ws,aec_a2b_work,ws);
	aec_work_data *data = &aec_work->aec_data;

	for(n=0;n<AEC_PERIOD_SAMPLE/2;++n)
	{
		aec_dbg_buf[n] = data->buf_adc[n*2+1];
	}
	
	cpub_aec_run(aec_work->aec_api_handle,aec_work->cpu_share_data,aec_work->cpu_share_data_phys);
	
	for(n=0;n<AEC_PERIOD_SAMPLE/2;++n)
	{
		data->buf_adc[n*2+1] = aec_dbg_buf[n] ;
	}
	
	audio_ring_buf_write(aec_work->output_ring,data->buf_adc,AEC_PERIOD_SAMPLE);
	if(aec_work->fp_cb)
	{
		aec_work->fp_cb(aec_work->output_ring,aec_work->user_data);
	}

	atomic_dec(&data->users);
	
	return;
}
#endif 

//put new work (audio data) to user context , then it will be sent to cpub later  
//this function can only use in interrupt context
//int queue_aec_a2b_work(aec_work_handle* handle,audio_ring_buffer* adc_ring,audio_ring_buffer* dac_ring,aec_done_callback fp_cb,void* user_data)
static int aec_wq_put_work(aec_work_handle* handle,audio_ring_buffer* adc_ring,audio_ring_buffer* dac_ring,aec_done_callback fp_cb,void* user_data)
{
	int i;
	aec_a2b_work *ws=0;	
	//spin_lock_irqsave(&handle->data_lock, handle->lock_flag);
	for(i=0;i<MAX_AEC_A2B_WORKS;++i)
	{
		//if(handle->works[i].aec_data.users==0)
		if( atomic_read( &handle->works[i].aec_data.users) ==0 )
		{
			ws = &handle->works[i];
			//++ws->aec_data.users;
			atomic_inc(&ws->aec_data.users);
			break;
		}	
	}
	//spin_unlock_irqrestore(&handle->data_lock, handle->lock_flag);
	
	if(ws)
	{
		audio_ring_buf_read(adc_ring,ws->aec_data.buf_adc,AEC_PERIOD_SAMPLE);
		audio_ring_buf_read(dac_ring,ws->aec_data.buf_dac,AEC_PERIOD_SAMPLE);
		ws->cpu_share_data->samples 				= AEC_PERIOD_SAMPLE;
		ws->cpu_share_data->AEC_Process_Samples 	= AEC_PERIOD_SAMPLE;
		ws->fp_cb = fp_cb;
		ws->user_data = user_data;
		//pr_info( "aec, adc=%d, dac=%d.\r\n", audio_ring_get_count(adc_ring), audio_ring_get_count(dac_ring));
		queue_work( handle->wq, (struct work_struct*)ws );
		return 0;
	}else{
		pr_err("audio aec, no work queue space.\r\n");
		//BUG();
		return -1;
	}
}

#if 0
static void aec_data_transfer_routine(struct work_struct *ws)
{ 
	static int count=0;
	//aec_work_data *data = &(container_of(ws,aec_a2b_work,ws)->aec_data);
	//struct snd_pcm_substream *substream = (struct snd_pcm_substream*) (container_of(ws,aec_a2b_work,ws)->user_data);
	aec_a2b_work *aec_work = container_of(ws,aec_a2b_work,ws);
	aec_work_data *data = &aec_work->aec_data;

#if 0	
	if(count++%62 == 0)
		pr_info("aec_data_transfer_routine, buf_adc=0x%X, buf_dac=0x%X, count=%d\r\n",
				(uint32_t) aec_work->cpu_share_data->MIC_in_buffer,
				(uint32_t) aec_work->cpu_share_data->SPK_out_buffer,
				count );
#endif 
	//pr_info("aec run.\r\n");
	cpub_aec_run(aec_work->aec_api_handle,aec_work->cpu_share_data,aec_work->cpu_share_data_phys);

	//pr_info("skip aec run.\r\n");

	audio_ring_buf_write(&g_aec_done_ring,data->buf_adc,AEC_PERIOD_SAMPLE);	
	//audio_ring_buf_write(&g_aec_done_ring,data->buf_dac,AEC_PERIOD_SAMPLE);
	
	//spin_lock_irqsave(&aec_work->data_lock, aec_work->data_lock_flag);
	if(!g_aec_work_handle)
	{
		//spin_unlock_irqrestore(&aec_work->data_lock, aec_work->data_lock_flag);
		return;
	}
	struct snd_pcm_substream *substream = (struct snd_pcm_substream*) aec_work->user_data;
	struct vsnv3_runtime_data *prtd = substream->runtime->private_data;
	
	while(audio_ring_get_count(&g_aec_done_ring) > (prtd->period_size/sizeof(short)) )
	{
		struct ait_pcm_dma_params *params = prtd->params;	
		struct snd_dma_buffer *buf = &substream->dma_buffer;
		
		short* dest_ptr = ((unsigned short*)buf->area+(prtd->period_ptr-prtd->dma_buffer)/params->pdc_xfer_size);

		audio_ring_buf_read( &g_aec_done_ring, dest_ptr, prtd->period_size/sizeof(short) );
		
		prtd->period_ptr += prtd->period_size;			
		if (prtd->period_ptr >= prtd->dma_buffer_end)
			prtd->period_ptr = prtd->dma_buffer;

		params->pdc->xpr = prtd->period_ptr;
		params->pdc->xcr = prtd->period_size / params->pdc_xfer_size;
			
		//pr_info("period elapsed.\r\n");
		snd_pcm_period_elapsed(substream);
		
	}
	--(data->users);
	//spin_unlock_irqrestore(&aec_work->data_lock, aec_work->data_lock_flag);
}
#endif 
