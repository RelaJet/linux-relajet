/*
 * Driver for aes alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-vsnv3/cpucomm/cpub_aes.c which has following copyrights:
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
#include "cpub_aes_cpucomm.h"

#if 1
#define MS_FUNC(who,statement)  statement
#else
#define MS_FUNC(who,statement)				\
do {										              \
	MMP_ULONG64 t1,t2 ;                 \              
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
#define COM_SEND()          do {                \
                                MS_FUNC("s:",err = CpuComm_DataSend( cpucmm_id, (MMP_UBYTE*)data, COM_OBJ_SZ,0 ) );         \
                                if(!err) {  \
                                  MS_FUNC("r:",CpuComm_DataReceive( cpucmm_id, (MMP_UBYTE*)data,  COM_OBJ_SZ,0,0) );   \
                                } \
                            } while(0)
#define COM_RECV()          //CpuComm_ReceiveDataEnd( cpucmm_id,data->command)

extern void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
extern void cpub_free(int size,void *va,dma_addr_t h_dma) ;


static int debug = 0;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "AIT AES cpucomm driver debug level (0-1)");


u32 aes_seq_cnt =0;
static CPU_COMM_ID cpucmm_id = -1;
extern void MMPF_MMU_FlushDCacheMVA(unsigned long ulRegion, unsigned long ulSize);


int cpub_aes_run(struct AES_proc_info_s *aes_proc)
{
  //dma_addr_t h_dma ;
  //char *out_frame = 0 ;
  //int i;
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;
	

	data->command = AES_ENCODE;
	data->phy_addr		= COM_DMA(aes_proc);
	data->size		= sizeof(struct AES_proc_info_s);
	data->seq		= aes_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	
	
	COM_SEND();
	
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(aes_proc));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	//pr_info("data->phy : 0x%08x,result : %d\n",data->phy_addr,data->result );
	if(!err) {
	  COM_RECV();
  }  
  COM_OBJ_FREE(0);
  
  return err ? -EIO : 0 ;
}
EXPORT_SYMBOL(cpub_aes_run);


int cpub_aes_register(int id)
{
	CPU_COMM_ERR ret;
	ret = CpuComm_RegisterEntry(id, CPU_COMM_TYPE_DATA);
	if(ret==CPU_COMM_ERR_NONE)
		cpucmm_id = id;
	return ret;
}
EXPORT_SYMBOL(cpub_aes_register);

int cpub_aes_unregister(void)
{
	CpuComm_UnregisterEntry(cpucmm_id);
	return 0;
}
EXPORT_SYMBOL(cpub_aes_unregister);

