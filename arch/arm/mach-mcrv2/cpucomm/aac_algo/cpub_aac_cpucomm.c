/*
 * Driver for aac alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-vsnv3/cpucomm/cpub_aac.c which has following copyrights:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/dmapool.h>

#include <asm/uaccess.h>
#include <mach/os_wrap.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm_id.h>
#include <mach/cpucomm/cpucomm_api.h>
#include "cpub_aac_cpucomm.h"

#if 1 
#define MS_FUNC(who,statement)  statement
#else
#define MS_FUNC(who,statement)				\
do {										              \
	MMP_ULONG64 t1,t2 ;                   \
	MMPF_OS_GetTimestamp(&t1,MMPF_OS_TIME_UNIT_JIFFIES);    \
	statement ;								                            \
	MMPF_OS_GetTimestamp(&t2,MMPF_OS_TIME_UNIT_JIFFIES);		\
	pr_info("%s time : %d\r\n",who,t2 - t1 );\
} while (0)
#endif
#define COM_OBJ_SZ    sizeof(struct cpu_comm_transfer_data )
// cpub_malloc(COM_OBJ_SZ,GFP_KERNEL,&h_dma);  
#define COM_OBJ_ALLOC(x)    do {                    \
                                data = &__data;     \
                                if(!data) {         \
                                    return -ENOMEM ; \
                                }                   \
                            } while(0)
#define COM_OBJ_FREE(x)     //cpub_free(data,h_dma )
                            
#define COM_DMA(addr)             (u32)addr                                
                                
#define AAC_IPC_TIMEOUT		(1000)

#define COM_SEND()          do {                \
                                MS_FUNC("s:",err = CpuComm_DataSend( cpucmm_id, (MMP_UBYTE*)data, COM_OBJ_SZ,AAC_IPC_TIMEOUT ) );         \
                                if(! err ) {	\
                                	MS_FUNC("r:",err = CpuComm_DataReceive( cpucmm_id, (MMP_UBYTE*)data,  COM_OBJ_SZ,AAC_IPC_TIMEOUT,0) );   \
                                } \
                            } while(0)

#define COM_SEND_NOACK()    do {                \
                                MS_FUNC("s:",err = CpuComm_DataSend( cpucmm_id, (MMP_UBYTE*)data, COM_OBJ_SZ,AAC_IPC_TIMEOUT ) ); \
                            } while(0)


extern void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
extern void cpub_free(int size,void *va,dma_addr_t h_dma) ;


static int debug = 0;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "AIT AAC cpucomm driver debug level (0-1)");


u32 aac_seq_cnt =0;
static CPU_COMM_ID cpucmm_id = -1;
extern void MMPF_MMU_FlushDCacheMVA(unsigned long ulRegion, unsigned long ulSize);


int cpub_aac_run(int sid,int ack_off)
{
  //dma_addr_t h_dma ;
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;
	data->command = (sid << 8) | AAC_ENCODE;
	// use phy_addr to pass parameters
	data->phy_addr		= (ack_off?1:0); //COM_DMA(frame);
	data->size		= sizeof(int);
	data->seq		= aac_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;

	
	if(!ack_off) {
	  COM_SEND();
	}
	else {
	  COM_SEND_NOACK();
	}  
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
//		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(frame));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
  COM_OBJ_FREE(0);
  // TBD : want to check error !
	return 0;
}
EXPORT_SYMBOL(cpub_aac_run);


int cpub_aac_init(int sid,struct AAC_init_s* aac_init)
{
  //dma_addr_t h_dma ;
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);
  
	data->command 	= (sid << 8) | AAC_INIT;
	data->phy_addr	= COM_DMA(aac_init);
	data->size			= sizeof(struct AAC_init_s);
	data->seq			= aac_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	//BUG_ON(data->command !=AAC_INIT);

	if(debug)
	{
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(aac_init));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			  = %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	//CpuComm_ReceiveDataEnd( cpucmm_id, data->command);
	COM_OBJ_FREE(0);
	return 0;	
}
EXPORT_SYMBOL(cpub_aac_init);

int cpub_aac_register(int id)
{
	CPU_COMM_ERR ret;
	ret = CpuComm_RegisterEntry(id, CPU_COMM_TYPE_DATA);
	if(ret==CPU_COMM_ERR_NONE)
		cpucmm_id = id;
	return ret;
}
EXPORT_SYMBOL(cpub_aac_register);

int cpub_aac_unregister(void)
{
	CpuComm_UnregisterEntry(cpucmm_id);
	return 0;
}
EXPORT_SYMBOL(cpub_aac_unregister);

