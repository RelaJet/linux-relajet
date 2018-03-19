#ifndef AEC_WORK_QUEUE_H
#define AEC_WORK_QUEUE_H

#include "audio-ring-buf.h"

#define AEC_PERIOD_SAMPLE (16*16*2) //16ms sample @ 16K hz sampling rate 2 ch   //SZ_512
#define AEC_PERIOD_BUF_SIZE (AEC_PERIOD_SAMPLE*sizeof(short)) // 1K bytes 
#define MAX_AEC_A2B_WORKS 8

//--------------------- AEC APIs ---------------
typedef enum
{
    AEC_INIT,//0
    AEC_GET_VERSION,	
    AEC_RUN,
    AEC_NOT_SUPPORT = -1,
    AEC_DBG = 0x80000000
} AEC_COMMAND;

typedef struct AEC_run_s
{
	short* MIC_in_buffer;
	short* SPK_out_buffer;
	short  samples;
	short  AEC_Process_Samples;
} AEC_run_t;

typedef struct 
{
	int id;
}cpub_aec_handle; // ,*aec_handle;

typedef enum {
	AEC_CTL_START,
	AEC_CTL_STOP
}AEC_CTL_ID;

typedef struct 
{
	struct work_struct ws;
	AEC_CTL_ID cmd_id;
	void *param;				//user data
}aec_ctl_work;
	
//------------------- AEC WORK QUEUE ----------
typedef struct aec_work_data_
{
	short* buf_adc;
	short* buf_dac;
	//int users;
	atomic_t users;
}aec_work_data;

typedef void (*aec_done_callback)(audio_ring_buffer*, void*);
typedef struct aec_a2b_work_
{
	struct work_struct ws;
	AEC_run_t *cpu_share_data; 			//shared data struct between cpua and cpub
	//unsigned long cpu_share_data_phys;	//shared data physical address 
	dma_addr_t cpu_share_data_phys;
	void* cpu_share_data_virt;
	
	aec_work_data aec_data;
	cpub_aec_handle *aec_api_handle;			//handle to 2 cpu api
	
	//aec done callback
	audio_ring_buffer *output_ring;		//pointer to aec processed data buffer, work function put cpub aec output to this buffer 
	void *user_data;			
	aec_done_callback fp_cb;	//user callback	
}aec_a2b_work;

typedef struct aec_work_handle_
{
	aec_a2b_work works[MAX_AEC_A2B_WORKS];
	struct workqueue_struct *wq;				//aec data work queue
	cpub_aec_handle aec;						//cpub aec handle 
	int users;  //reference count 
	
	spinlock_t data_lock;
	unsigned long data_lock_flag;

	audio_ring_buffer adc_ring;
	audio_ring_buffer dac_ring;
	audio_ring_buffer output_ring; 
}aec_work_handle;

aec_work_handle* aec_wq_start();
void aec_wq_stop(aec_work_handle* handle);
int aec_wq_put_dac_data(aec_work_handle* handle, short *data, int n_samples);
int aec_wq_put_adc_data(aec_work_handle* handle, short *data, int n_samples,aec_done_callback fp_cb,void* user_dat);

#endif 
