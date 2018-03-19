/*
 * Driver for MD alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-vsnv3/cpucomm/cpub_md.c which has following copyrights:
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
#include <asm/uaccess.h>
#include <mach/os_wrap.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm_id.h>
#include <mach/cpucomm/cpucomm_api.h>
#include <mach/ait_cpu_sharemem.h>
#include "cpub_md_cpucomm.h"
#include "md.h"

#define MD_IPC_TIMEOUT	(1000)

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
#define COM_OBJ_ALLOC(x)    do {                    \
                                data = &__data;     \
                                if(!data) {         \
                                    return -ENOMEM ; \
                                }                   \
                            } while(0)
#define COM_OBJ_FREE(x)     //cpub_free(data,h_dma )
                            
#define COM_DMA(addr)             (u32)addr                                
                                
#define COM_SEND()          do {                \
                                MS_FUNC("s:",err = CpuComm_DataSend( cpucmm_id, (MMP_UBYTE*)data, COM_OBJ_SZ,MD_IPC_TIMEOUT ) );         \
                                if(! err ) {	\
                                	MS_FUNC("r:",err = CpuComm_DataReceive( cpucmm_id, (MMP_UBYTE*)data,  COM_OBJ_SZ,MD_IPC_TIMEOUT,0) );   \
                                } \
                            } while(0)
#define COM_RECV()          //CpuComm_ReceiveDataEnd( cpucmm_id,data->command)


int debug = 0;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "AIT MD cpucomm driver debug level (0-1)");


u32 md_seq_cnt = 1;
static CPU_COMM_ID cpucmm_id = -1;
static int enable_handler = 0 ;
extern void MMPF_MMU_InvalidateDCacheMVA(MMP_ULONG ulRegion, MMP_ULONG ulSize);
extern void MMPF_MMU_FlushDCacheMVA(unsigned long ulRegion, unsigned long ulSize);

int cpub_md_set_window(struct MD_detect_window_s* pWin)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0) ;
  
	data->command 	= MD_SET_WINDOW;
	data->phy_addr		= COM_DMA(pWin) ;//virt_to_phys(pWin);
	data->size			= sizeof(struct MD_detect_window_s);
	data->seq			= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(pWin));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV() ;
  COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_md_set_window);

int cpub_md_set_para(struct MD_window_parameter_in_s* para)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0) ;
  
	data->command = MD_SET_WINDOW_PARA_IN;
	data->phy_addr	= COM_DMA(para) ; //virt_to_phys(para);
	data->size		= sizeof(MD_window_parameter_in_t);
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;

	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(para));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
  COM_RECV();
  COM_OBJ_FREE(0);
  
	return 0;


}
EXPORT_SYMBOL(cpub_md_set_para);

int cpub_md_get_windowsize(struct MD_detect_window_s* win)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;
	data->command = MD_GET_WINDOW_SIZE;
	data->phy_addr= 0x89abcdef;
	data->size	 = 0xdeadbeff;			
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)data->phy_addr);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV();
  COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_md_get_windowsize);


int cpub_md_get_window_para(struct MD_window_parameter_in_s* para)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);

	data->command = MD_GET_WINDOW_PARA_IN;
	data->phy_addr= 0x89abcdef;
	data->size	 = 0xdeadbeff;			
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,data->phy_addr);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV();
  COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_md_get_window_para);


int cpub_md_get_buffer_info(struct MD_buffer_info_s* info)
{
  CPU_COMM_ERR err ;
	int bufSize;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
	data->command 	= MD_GET_BUFFER_INFO;
	data->phy_addr		= COM_DMA( info );// virt_to_phys(info);
	data->size			= sizeof(struct MD_buffer_info_s);		
	data->seq			= md_seq_cnt++;
	data->flag			=CPUCOMM_FLAG_CMDSND|CPUCOMM_FLAG_WAIT_FOR_RESP;

  COM_SEND();
  
	BUG_ON(data->command !=MD_GET_BUFFER_INFO);

	if(debug)
	{
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data, COM_DMA( info ));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	bufSize = data->phy_addr ;
	COM_RECV();

	if(debug)
		pr_info("bufSize 			= %d\n",(int)bufSize);
  COM_OBJ_FREE(0);
	return bufSize;

}
EXPORT_SYMBOL(cpub_md_get_buffer_info);


int cpub_md_get_version(unsigned int * version)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);
  
	data->command = MD_GET_VERSION;
	data->phy_addr= 0x89abcdef;
	data->size	 = 0xdeadbeff;		
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,data->phy_addr);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	*version = data->result ;
	COM_RECV();
  COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_md_get_version);



int cpub_md_get_result(struct MD_window_parameter_out_s* para)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);

	data->command 	= MD_GET_WINDOW_PARA_OUT;
	data->phy_addr		= COM_DMA(para);
	data->size			= sizeof(struct MD_window_parameter_out_s);		
	data->seq			= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;

	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(para));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV();
  COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_md_get_result);


int cpub_md_suspend(struct MD_suspend_s* suspndinfo)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);

	data->command = MD_SUSPEND;
	data->phy_addr		= COM_DMA(suspndinfo);
	data->size		= sizeof(struct MD_suspend_s);	
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;

	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(suspndinfo));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	
	COM_RECV();
  COM_OBJ_FREE(0);
	return 0;	
}
EXPORT_SYMBOL(cpub_md_suspend);


int cpub_md_run(void *frame,struct MD_motion_info_s *md_info)
{
	CPU_COMM_ERR err ;
  //int i;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;

	data->command = MD_RUN;
	data->phy_addr		= COM_DMA(frame);
	data->size		= sizeof(int);
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(frame));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
		pr_info("phy_addr   = 0x%x\n",(u32)data->phy_addr );
	}
  
	if(data->result && !err )
	{
		if(frame) {
			//we must inavlidate Data cache. It should change to use cpua memory!!
			MMPF_MMU_InvalidateDCacheMVA(phys_to_virt(data->phy_addr), sizeof(struct MD_motion_info_s));
			*md_info =  *(struct MD_motion_info_s *)phys_to_virt(data->phy_addr) ;   
    		}
    		else
		{
			*md_info =  *(struct MD_motion_info_s *)AIT_RAM_P2V(data->phy_addr) ;
		}
	}
	else {
	    	md_info->obj_cnt = 0 ;    
	}
	

  if(debug) {
	  pr_info("obj_cnt : %d\n",md_info->obj_cnt );
  }
	COM_RECV();
  COM_OBJ_FREE(0);
	if(err) {
		return -EINVAL ;	
	}
	return 0;
}
EXPORT_SYMBOL(cpub_md_run);


int cpub_md_init(struct MD_init_s* pInit)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  COM_OBJ_ALLOC(0);
  
	data->command 	= MD_INIT;
	data->phy_addr	= COM_DMA(pInit);
	data->size			= sizeof(struct MD_init_s);
	data->seq			= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	        
	COM_SEND();
	BUG_ON(data->command !=MD_INIT);
	if(debug)
	{
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(pInit));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			  = %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV();
	COM_OBJ_FREE(0);
	/*
	cpu-b md must return 0 for sub-windows ver, 1 for ROI version
	*/
	
	return (int)data->result;	
}
EXPORT_SYMBOL(cpub_md_init);


int cpub_md_histgram(struct MD_motion_info_s *md_info)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;

	data->command = MD_HISTGRAM;
	data->phy_addr		= 0;
	data->size		= sizeof(int);
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)0);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
		pr_info("phy_addr   = 0x%x\n",(u32)data->phy_addr );
	}
  
	if(data->result && !err ) {
	  #ifndef CONFIG_AIT_FAST_BOOT
      *md_info =  *(struct MD_motion_info_s *)phys_to_virt(data->phy_addr) ;   
    #else      
      *md_info =  *(struct MD_motion_info_s *)AIT_RAM_P2V(data->phy_addr) ;  
    #endif  
	}
	else {
	    md_info->obj_cnt = 0 ;    
	}
	

  if(debug) {
	  pr_info("obj_cnt : 0x%04x\n",md_info->obj_cnt );
  }
	COM_RECV();
  COM_OBJ_FREE(0);
	if(err) {
		return -EINVAL ;	
	}
	return 0;
}
EXPORT_SYMBOL(cpub_md_histgram);

//====== TD =========
int cpub_td_init(struct TD_init_s* pInit)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
  
	data->command 	= TD_INIT;
	data->phy_addr	= COM_DMA(pInit);
	data->size	= sizeof(struct TD_init_s);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND; 
	        
	COM_SEND();
	BUG_ON(data->command !=TD_INIT);
	if(debug)
	{
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(pInit));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV();
	COM_OBJ_FREE(0);
	/*

	cpu-b td must return 0
	*/
	
	return (int)data->result;	
}
EXPORT_SYMBOL(cpub_td_init);

int cpub_td_get_version(unsigned int* version)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
  
	data->command 	= TD_GET_VERSION;
	data->phy_addr	= 0x89abcdef;
	data->size	= 0xdeadbeff;		
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,data->phy_addr);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	*version = data->result ;
	COM_RECV();
	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_get_version);

int cpub_td_run(void *frame,int* td_result )
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;

	data->command 	= TD_RUN;
	data->phy_addr	= COM_DMA(frame);
	data->size	= sizeof(int);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(frame));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
		pr_info("phy_addr   = 0x%x\n",(u32)data->phy_addr );
	}
	*td_result = data->result ;
	COM_RECV();
  	COM_OBJ_FREE(0);

	return 0;
}
EXPORT_SYMBOL(cpub_td_run);

int cpub_td_set_window(struct TD_detect_window_s* para)
{
  	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  	COM_OBJ_ALLOC(0) ;
  
	data->command	= TD_SET_WINDOW;
	data->phy_addr	= COM_DMA(para) ;//virt_to_phys(pWin);
	data->size	= sizeof(struct TD_detect_window_s);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(para));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	COM_RECV() ;
  	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_set_window);

int cpub_td_get_window_size(struct TD_detect_window_size_s* size_para)
{
 	CPU_COMM_ERR err ;
	int window_size = 0;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;
	data->command 	= TD_GET_WINDOW_SIZE;
	data->phy_addr	= COM_DMA(size_para);
	data->size	= sizeof(struct TD_detect_window_size_s);;			
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(size_para) );
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	window_size = data->phy_addr;
	COM_RECV();
  	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_get_window_size);

int cpub_td_set_para_in(struct Tamper_params_in_s* para_in)
{
  	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
  	COM_OBJ_ALLOC(0) ;
	data->command 	= TD_SET_PARA_IN;
	data->phy_addr	= COM_DMA(para_in) ; //virt_to_phys(para);
	data->size	= sizeof(struct Tamper_params_in_s);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(para_in));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}

  	COM_RECV();
  	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_set_para_in);

int cpub_td_get_window_result(struct TD_window_result_s *td_win_result)
{
	CPU_COMM_ERR err ;

	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;

	data->command 	= TD_GET_WINDOW_RESULT;
	data->phy_addr	= COM_DMA(td_win_result);
	data->size	= sizeof(struct TD_window_result_s);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,(u32)COM_DMA(td_win_result));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
		pr_info("phy_addr   = 0x%x\n",(u32)data->phy_addr );
	}
	//pr_info("phy_addr   = 0x%x  virt = 0x%x \n",(u32)data->phy_addr,(u32)phys_to_virt(data->phy_addr));
	MMPF_MMU_InvalidateDCacheMVA(phys_to_virt(data->phy_addr), sizeof(struct TD_window_result_s));
	*(struct TD_window_result_s *)phys_to_virt(td_win_result) = *(struct TD_window_result_s *)phys_to_virt(data->phy_addr);

	COM_RECV();
  	COM_OBJ_FREE(0);

	return 0;
}
EXPORT_SYMBOL(cpub_td_get_window_result);

int cpub_td_get_buffer_size(struct TD_buffer_size_s* buf_size)
{
  	CPU_COMM_ERR err ;
	int bufSize;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
	data->command 	= TD_GET_BUFFER_SIZE;
	data->phy_addr	= COM_DMA( buf_size );
	data->size	= sizeof(struct TD_buffer_size_s);		
	data->seq	= md_seq_cnt++;
	data->flag	=CPUCOMM_FLAG_CMDSND|CPUCOMM_FLAG_WAIT_FOR_RESP;

 	COM_SEND();
	BUG_ON(data->command !=TD_GET_BUFFER_SIZE);
	if(debug)
	{
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data, COM_DMA( buf_size ));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	bufSize = data->phy_addr ;
	COM_RECV();

	if(debug)
		pr_info("bufSize 			= %d\n",(int)bufSize);
  	COM_OBJ_FREE(0);
	return bufSize;
}
EXPORT_SYMBOL(cpub_td_get_buffer_size);

int cpub_td_set_windows_en(struct TD_set_window_enable_s* win_en)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
  
	data->command 	= TD_SET_WINDOW_EN;
	data->phy_addr	= COM_DMA(win_en) ;
	data->size	= sizeof(struct TD_set_window_enable_s);		
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(win_en));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}

	COM_RECV();
	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_set_windows_en);

int cpub_td_get_windows_en(struct TD_get_window_enable_s* win_en)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
  
	data->command 	= TD_GET_WINDOW_EN;
	data->phy_addr	= COM_DMA(win_en) ;
	data->size	= sizeof(struct TD_get_window_enable_s);
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,COM_DMA(win_en));
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}
	*(win_en->en) = data->result ;
	COM_RECV();
	COM_OBJ_FREE(0);
	return win_en->en;
}
EXPORT_SYMBOL(cpub_td_get_windows_en);

int cpub_td_release(void)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0);
  
	data->command 	= TD_RELEASE;
	data->phy_addr	= 0x89abcdef;
	data->size	= 0xdeadbeff;		
	data->seq	= md_seq_cnt++;
	data->flag	= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
	if(debug)
	{
		pr_info("%s Rcv command 	= 0x%x\n",__func__,(int)data->command );
		pr_info("(com,payload)=(0x%08x,0x%08x)\n", (u32)data,data->phy_addr);
		pr_info("size 			= %d\n",(int)data->size );
		pr_info("seq 			= %d\n",(int)data->seq );
		pr_info("result 		= 0x%x\n",(int)data->result );	
		pr_info("flag 			= 0x%x\n",(int)data->flag );
	}

	COM_RECV();
	COM_OBJ_FREE(0);
	return 0;
}
EXPORT_SYMBOL(cpub_td_release);

int cpub_md_enable_event(int enable)
{
	CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data *data,__data;
	COM_OBJ_ALLOC(0) ;

	data->command = MD_EVENT_CHANGES;
	data->phy_addr		= enable;
	data->size		= 0;
	data->seq		= md_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND;
	
	COM_SEND();
  
	if(data->result && !err ) {
	  err = CPU_COMM_ERR_UNSUPPORT ;
	}
	
	
	COM_RECV();
  COM_OBJ_FREE(0);
	if(err) {
		return -EINVAL ;	
	}
	return 0;
}

unsigned int cpub_md_handler(void *slot) 
{
extern void ait_md_wakeup(unsigned int event_id,int val)  ;
  struct cpu_share_mem_slot *r_slot = (struct cpu_share_mem_slot *)slot ;
  if( !enable_handler ) {
    return 0 ;
  }
  
  if(! r_slot ) {
    pr_info("#unknow slot isr\n");
    return 0;  
  }
  if(r_slot->dev_id==(unsigned int)cpucmm_id) {
    switch (r_slot->command) 
    {
      case MD_EVENT_CHANGES:
      ait_md_wakeup(MD_EVENT_CHANGES,r_slot->send_parm[0]); 
      break ;
    }
  }
  return 0;
      
}

void enable_md_event(int en)
{
  enable_handler = en ;  
}

int cpub_md_register(int id)
{
	CPU_COMM_ERR ret;
	ret = CpuComm_RegisterEntry(id, CPU_COMM_TYPE_DATA);
	if(ret==CPU_COMM_ERR_NONE) {
		cpucmm_id = (CPU_COMM_ID)id;
	  CpuComm_RegisterISRService( id ,cpub_md_handler );
	  /*
	  Enable MD event notify service
	  Please make sure cpu-b md driver support this feature or not
	  */
	  /* it is too early to enable event here
	  if( cpub_md_enable_event(1) ) {
	    enable_handler = 0 ;
	  }
	  else {
	    enable_handler = 1;
	  }
	  */
	  //printk("Enable md event : %d\n",enable_handler);
  }	
	return ret;
}
EXPORT_SYMBOL(cpub_md_register);

int cpub_md_unregister(void)
{
	CpuComm_UnregisterEntry(cpucmm_id);
	return 0;
}
EXPORT_SYMBOL(cpub_md_unregister);

