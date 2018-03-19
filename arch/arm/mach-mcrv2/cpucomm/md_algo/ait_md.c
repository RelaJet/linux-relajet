/*
 * Driver for MD alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-mcrv2/cpucomm/cpub_md.c which has following copyrights:
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
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <mach/cpucomm/cpucomm_id.h>
#include "ait_md.h"
#include "cpub_md_cpucomm.h"
extern void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
extern void cpub_free(int size,void *va,dma_addr_t h_dma) ;
extern int debug ;
#define COM_PAYLOAD_ALLOC(size) do {    \
                                    payload = cpub_malloc(size,GFP_KERNEL,&h_dma);    \
                                    if(!payload) {          \
                                        return -ENOMEM ;    \
                                    }                       \
                                } while (0)
                                
                                     
#define COM_PAYLOAD_FREE(size) cpub_free(size,payload,h_dma )

typedef struct md_event_data_s
{
  rwlock_t lock;
  int md_event ; // event is comming
  int md_cnt   ; // how many ROI have motion
} md_event_data_t ;

//static DECLARE_WAIT_QUEUE_HEAD(ait_md_waitq);
static wait_queue_head_t ait_md_waitq ;
static int open_cnt = 0 ;
static md_event_data_t md_event_data; 

#define DRIVER_NAME "ait_md"


static long ait_md_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	//int err = 0;
	//unsigned char val;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	int ret = -ENOTTY;

	//int size;
	void *payload ;
	dma_addr_t h_dma ;
	
	switch (cmd) {
	case MDIOCS_INIT:
		{
		  struct MD_init_s   *pInitInfo ;
			COM_PAYLOAD_ALLOC( sizeof(struct MD_init_s) ) ;
			pInitInfo = (struct MD_init_s  *)payload ;
			ret = copy_from_user(payload, argp, sizeof(struct MD_init_s));
			
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct MD_init_s)) ;
				ret = -EFAULT;
				goto done;
			}
      pInitInfo = (struct MD_init_s   *)payload;
      if(debug) {
  			pr_info("===MDIOCS_INIT===(0x%08x,0x%08x)\n",(u32)pInitInfo,(u32)h_dma);
  			pr_info("working_buf_ptr = 0x%x\n",pInitInfo->working_buf_ptr);
  			pr_info("working_buf_len = %d\n",pInitInfo->working_buf_len);
  			pr_info("width = %d\n",pInitInfo->width);
  			pr_info("height = %d\n",pInitInfo->height);
  			pr_info("color = %d\n",pInitInfo->color);
      }
			ret = cpub_md_init(h_dma);

      COM_PAYLOAD_FREE(sizeof(struct MD_init_s) ) ;
			//kfree(pInitInfo);			
		}
		break;

	case MDIOCG_VERSION:
		{
			int ver;
			pr_info("===MDIOCG_VERSION===\n");
			
			ret = cpub_md_get_version(&ver);
			if(ret)
			{
				pr_err("cpub_md_get_version error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}
			pr_info("MD_Ver : 0x%08x\n",ver);
			ret = put_user(ver, p);
		}
		break;

	case MDIOCSG_RUN:
		{
      struct MD_proc_info_s *md_proc ;
			COM_PAYLOAD_ALLOC( sizeof(struct MD_proc_info_s ) ) ;
			md_proc = (struct MD_proc_info_s *)payload ;
			
			ret = copy_from_user(payload, argp, sizeof(struct MD_proc_info_s));
			if(ret)
			{
			  COM_PAYLOAD_FREE(sizeof(struct MD_proc_info_s )) ;  
				ret = -EFAULT;
				goto done;
			}
			//pr_info("===MDIOCS_RUN===(0x%08x,0x%08x)\n",(u32)md_proc,(u32)h_dma);

			ret =cpub_md_run((void *)md_proc->frame,&md_proc->result);
			if(ret) {
				goto exit ;
			}
			ret = copy_to_user(argp, payload, sizeof(struct MD_proc_info_s)) ;
			
			COM_PAYLOAD_FREE(sizeof(struct MD_proc_info_s )) ;
exit:			
			if(ret)
			{
				pr_err("cpub_md_run error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}		
		}			
		break;
	case MDIOCG_HISTGRAM:
		{
      struct MD_motion_info_s md_info ;
			ret = cpub_md_histgram(&md_info);
			if(ret) {
				pr_err("cpub_md_histgram error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			}		
			else {
			  ret = copy_to_user(argp, &md_info, sizeof(struct MD_motion_info_s)) ;
			}
		}			
		break;
	case MDIOCS_WINDOW:
		{
			struct MD_detect_window_s  * pWin ;
			COM_PAYLOAD_ALLOC( sizeof(struct MD_detect_window_s) ) ;
			pWin = (struct MD_detect_window_s  *)payload ;
			
			
			ret = copy_from_user(pWin, argp, sizeof(struct MD_detect_window_s));
			if (ret) {
				//kfree(pWin);
				COM_PAYLOAD_FREE( sizeof(struct MD_detect_window_s) );
				goto done;
			}
      if(debug) {
  			pr_info("===MDIOCS_WINDOW===\n");
  			pr_info("lt_x = %d\n",pWin->lt_x);
  			pr_info("lt_y = %d\n",pWin->lt_y);
  			pr_info("rb_x = %d\n",pWin->rb_x);
  			pr_info("rb_y = %d\n",pWin->rb_y);
  			pr_info("w_div = %d\n",pWin->w_div);
  			pr_info("h_div = %d\n",pWin->h_div);
		  }
			ret = cpub_md_set_window(h_dma);
			//kfree(pWin);	
			COM_PAYLOAD_FREE( sizeof(struct MD_detect_window_s));
			if(ret)
			{
				pr_err("cpub_md_set_window error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}				
		
		}
		break;
		
	case MDIOCG_WINDOW_SIZE:
		{
			struct MD_detect_window_s* pWinSize ;//= kmalloc(sizeof(struct MD_detect_window_s),GFP_KERNEL);
			COM_PAYLOAD_ALLOC( sizeof(struct MD_detect_window_s) ) ;
			pWinSize = (struct MD_detect_window_s* )payload ;
			

			pr_info("===MDIOCG_WINDOW_SIZE===\n");

			ret = cpub_md_get_windowsize(h_dma);
			if(ret)
			{
				//kfree(pWinSize);			
        COM_PAYLOAD_FREE(sizeof(struct MD_detect_window_s));
				pr_err("cpub_md_get_windowsize error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}			

			ret = copy_to_user(argp, pWinSize, sizeof(struct MD_detect_window_s)) ;
			//kfree(pWinSize);
			COM_PAYLOAD_FREE(sizeof(struct MD_detect_window_s));
			if (ret) {
				goto done;
			}						
		}				
		break;


	case MDIOCS_WINDOW_PARA_IN:
		{
//static int my_inc = 0 ;

			struct MD_window_parameter_in_s *pParaIn ;
      COM_PAYLOAD_ALLOC( sizeof(struct MD_window_parameter_in_s) ) ;  			
			pParaIn = (struct MD_window_parameter_in_s *)payload ;
			ret = copy_from_user(pParaIn, argp, sizeof(struct MD_window_parameter_in_s));
			if (ret) {
				//kfree(pParaIn);
				COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_in_s) );
				goto done;
			}
			
			#if 0
			pParaIn->w_num += my_inc ;
			pParaIn->h_num += my_inc ;
			pParaIn->param.enable += my_inc  ;
			pParaIn->param.size_perct_thd_min += my_inc  ;
			pParaIn->param.size_perct_thd_max += my_inc  ;
			pParaIn->param.sensitivity += my_inc  ;
			pParaIn->param.learn_rate += my_inc  ;
			my_inc += 10 ;
			#endif
			if(debug) {
        pr_info("===MDIOCS_WINDOW_PARA_IN===(0x%08x,0x%08x)\n",pParaIn,h_dma);
  		
  			pr_info("w_num = %d\n",pParaIn->w_num);
  			pr_info("h_num = %d\n",pParaIn->h_num);
  			pr_info("enable = %d\n",pParaIn->param.enable);
  			pr_info("size_perct_thd_min = %d\n",pParaIn->param.size_perct_thd_min);
  			pr_info("size_perct_thd_max = %d\n",pParaIn->param.size_perct_thd_max);
  			pr_info("sensitivity = %d\n",pParaIn->param.sensitivity);
  			pr_info("learn_rate = %d\n",pParaIn->param.learn_rate);
      }
			ret = cpub_md_set_para(h_dma);
			//kfree(pParaIn);
			COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_in_s) );
			if(ret)
			{			
				pr_err("cpub_md_set_para error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			}			
		}		
		break;
		
	case MDIOCG_WINDOW_PARA_IN:
		{
			struct MD_window_parameter_in_s* pParaIn;//= kmalloc(sizeof(struct MD_window_parameter_in_s),GFP_KERNEL);
      COM_PAYLOAD_ALLOC( sizeof(struct MD_window_parameter_in_s) ) ;  	
			pParaIn = (struct MD_window_parameter_in_s *)payload ;
      


			pr_info("===MDIOCG_WINDOW_PARA_IN===\n");

			ret = cpub_md_get_window_para(h_dma);
			if(ret)
			{
				COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_in_s) );
				pr_err("cpub_md_get_window_para error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}			

			ret = copy_to_user(argp, pParaIn, sizeof(struct MD_window_parameter_in_s)) ;
			COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_in_s) );		
			if (ret) {
				goto done;
			}			
		}		
		break;
	case MDIOCSG_WINDOW_PARA_OUT:
		if(1){
			struct MD_window_parameter_out_s* pParaOut ;//= kmalloc(sizeof(struct MD_window_parameter_out_s),GFP_KERNEL);
      COM_PAYLOAD_ALLOC( sizeof(struct MD_window_parameter_out_s) ) ;  
      
      pParaOut = (struct MD_window_parameter_out_s *)payload;
			ret = copy_from_user(pParaOut, argp, sizeof(struct MD_window_parameter_out_s));
			if (ret) {
				//kfree(pParaOut);	
				COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_out_s));
				goto done;
			}
			if(debug) {
			  pr_info("===MDIOCSG_WINDOW_PARA_OUT===\n");
			  pr_info("w_num = %d\n",(int)pParaOut->w_num);
			  pr_info("h_num = %d\n",(int)pParaOut->h_num);
		  }
#if 1
			ret = cpub_md_get_result(h_dma);
			if(ret)
			{
				pr_err("cpub_md_get_result error = %d!\n",ret);
				COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_out_s));
				ret = -EFAULT;
				goto done;
			
			}			
      if(debug) {
			  pr_info("md_result = %d\n",pParaOut->param.md_result);
			  pr_info("obj_cnt = %d\n",pParaOut->param.obj_cnt);
		  }
			ret = copy_to_user(argp, pParaOut, sizeof(struct MD_window_parameter_out_s)) ;
			COM_PAYLOAD_FREE(sizeof(struct MD_window_parameter_out_s));	
			
			if (ret) {
				goto done;
			}	
#endif				
		}			
		break;
	case MDIOCSG_BUFFER_INFO:
		{
			struct MD_buffer_info_s* pBufinfo ;//= kmalloc(sizeof(struct MD_buffer_info_s),GFP_KERNEL);
      COM_PAYLOAD_ALLOC( sizeof(struct MD_buffer_info_s) ) ;  
      pBufinfo = (struct MD_buffer_info_s *)payload ;

			ret = copy_from_user(pBufinfo, argp, sizeof(struct MD_buffer_info_s));
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct MD_buffer_info_s));
				goto done;
			}
			if(debug) {
			  pr_info("===MDIOCG_BUFFER_INFO===(0x%08x,0x%08x)\n",(u32)pBufinfo,h_dma );
			  pr_info("width = %d\n",pBufinfo->width);
			  pr_info("height = %d\n",pBufinfo->height);
			  pr_info("color = %d\n",pBufinfo->color);
			  pr_info("w_div = %d\n",pBufinfo->w_div);
			  pr_info("h_div = %d\n",pBufinfo->h_div);
			  pr_info("return size = %d\n",pBufinfo->return_size );
      }
			ret= cpub_md_get_buffer_info(h_dma);
			if(ret<=0)
			{
				COM_PAYLOAD_FREE(sizeof(struct MD_buffer_info_s));
				pr_err("cpub_md_get_result error = %d!\n",ret);
				ret = -EFAULT;
				goto done;			
			}			
			pBufinfo->return_size = ret;	//Temp For test
			ret = copy_to_user(argp, pBufinfo, sizeof(struct MD_buffer_info_s)) ;

			pr_info("Size = %d\n",((MD_buffer_info_t*)argp)->return_size);
			
			COM_PAYLOAD_FREE(sizeof(struct MD_buffer_info_s));
			
			if (ret) {
				goto done;
			}
		}			
		
		break;

	case MDIOCS_MD_SUSPEND:
		{
			struct MD_suspend_s* suspndinfo ;//= kmalloc(sizeof(struct MD_suspend_s),GFP_KERNEL);
			COM_PAYLOAD_ALLOC( sizeof(struct MD_suspend_s ) ); 
			
      suspndinfo = (struct MD_suspend_s *)payload ;
      		
			pr_info("===MDIOCS_MD_SUSPEND===\n");
			pr_info("md_suspend_enable = %d\n",suspndinfo->md_suspend_enable);
			pr_info("md_suspend_duration = %d\n",suspndinfo->md_suspend_duration);
			pr_info("md_suspend_threshold = %d\n",suspndinfo->md_suspend_threshold);

			ret = copy_from_user(suspndinfo, argp, sizeof(struct MD_suspend_s));
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct MD_suspend_s ) );
				goto done;
			}

			ret = cpub_md_suspend( h_dma);
			COM_PAYLOAD_FREE(sizeof(struct MD_suspend_s ) );
			
			if(ret)
			{
				pr_err("cpub_md_suspend error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			}			
		}
				
		break;
	case TDIOCS_INIT:
		{
			struct TD_init_s   *pInitInfo ;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_init_s) ) ;
			pInitInfo = (struct TD_init_s  *)payload ;
			ret = copy_from_user(payload, argp, sizeof(struct TD_init_s));
			
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct TD_init_s)) ;
				ret = -EFAULT;
				goto done;
			}

			if(debug) {
	  			pr_info("===TDIOCS_INIT===(0x%08x,0x%08x)\n",(u32)pInitInfo,(u32)h_dma);
	  			pr_info("working_buf_ptr = 0x%x\n",pInitInfo->working_buf_ptr);
	  			pr_info("working_buf_len = %d\n",pInitInfo->working_buf_len);
	  			pr_info("width = %d\n",pInitInfo->width);
	  			pr_info("height = %d\n",pInitInfo->height);
	  			pr_info("color = %d\n",pInitInfo->color);
			}
			ret = cpub_td_init(h_dma);
			COM_PAYLOAD_FREE(sizeof(struct TD_init_s) ) ;
			//kfree(pInitInfo);			
		}
		break;
	case TDIOCG_VERSION:
		{
			int ver;
			pr_info("===TDIOCG_VERSION===\n");
			
			ret = cpub_td_get_version(&ver);
			if(ret)
			{
				pr_err("cpub_td_get_version error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}
			pr_info("TD_Ver : 0x%08x\n",ver);
			ret = put_user(ver, p);
		}
		break;
	case TDIOCSG_RUN:
		{
      			struct TD_run_s *td_run ;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_run_s ) ) ;
			td_run = (struct TD_run_s *)payload ;
			
			ret = copy_from_user(payload, argp, sizeof(struct TD_run_s));
			if(ret)
			{
			  COM_PAYLOAD_FREE(sizeof(struct TD_run_s )) ;  
				ret = -EFAULT;
				goto done;
			}
			//pr_info("===TDIOCSG_RUN===(0x%08x,0x%08x)\n",(u32)td_run,(u32)h_dma);

			ret =cpub_td_run((void *)td_run->frame,&td_run->td_result);
			//pr_info("===cpub_td_run end %d===\n",ret);
			if(ret) {
				goto tdrun_exit ;
			}
			ret = copy_to_user(argp, payload, sizeof(struct TD_run_s)) ;
			
			COM_PAYLOAD_FREE(sizeof(struct TD_run_s )) ;
		tdrun_exit:			
			if(ret)
			{
				pr_err("cpub_td_run error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}		
		}			
		break;
	case TDIOCS_WINDOW :
		{
			struct TD_detect_window_s  * pWin ;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_detect_window_s) ) ;
			pWin = (struct TD_detect_window_s  *)payload ;
						
			ret = copy_from_user(pWin, argp, sizeof(struct TD_detect_window_s));
			if (ret) {
				//kfree(pWin);
				COM_PAYLOAD_FREE( sizeof(struct TD_detect_window_s) );
				goto done;
			}
      			if(debug) {
	  			pr_info("===TDIOCS_WINDOW===\n");
	  			pr_info("lt_x = %d\n",pWin->lt_x);
	  			pr_info("lt_y = %d\n",pWin->lt_y);
	  			pr_info("rb_x = %d\n",pWin->rb_x);
	  			pr_info("rb_y = %d\n",pWin->rb_y);
	  			pr_info("w_div = %d\n",pWin->w_div);
	  			pr_info("h_div = %d\n",pWin->h_div);
		  	}
			ret = cpub_td_set_window(h_dma);
			//kfree(pWin);	
			COM_PAYLOAD_FREE( sizeof(struct TD_detect_window_s));
			if(ret)
			{
				pr_err("cpub_td_set_window error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}				
		
		}
		break;
	case TDIOCG_WINDOW_SIZE:
		{
			struct TD_detect_window_size_s* pWinSize ;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_detect_window_size_s) ) ;
			pWinSize = (struct TD_detect_window_size_s* )payload ;
			

			pr_info("===TDIOCG_WINDOW_SIZE===\n");

			ret = cpub_md_get_windowsize(h_dma);
			if(ret)
			{
				//kfree(pWinSize);			
        			COM_PAYLOAD_FREE(sizeof(struct TD_detect_window_size_s));
				pr_err("cpub_md_get_windowsize error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}			

			ret = copy_to_user(argp, pWinSize, sizeof(struct TD_detect_window_size_s)) ;
			//kfree(pWinSize);
			COM_PAYLOAD_FREE(sizeof(struct TD_detect_window_size_s));
			if (ret) {
				goto done;
			}						
		}
		break;
	case TDIOCS_PARA_IN:
		{
			struct Tamper_params_in_s* para_in;
      			COM_PAYLOAD_ALLOC( sizeof(struct Tamper_params_in_s) ) ;  	
			para_in = (struct Tamper_params_in_s *)payload ;
			pr_info("===TDIOCS_PARA_IN===\n");

			ret = copy_from_user(para_in, argp, sizeof(struct Tamper_params_in_s));
			if (ret) {
				//kfree(pWin);
				COM_PAYLOAD_FREE( sizeof(struct Tamper_params_in_s) );
				goto done;
			}

			ret = cpub_td_set_para_in(h_dma);
			if(ret)
			{
				COM_PAYLOAD_FREE(sizeof(struct Tamper_params_in_s) );
				pr_err("cpub_td_set_para_in error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}			
			COM_PAYLOAD_FREE(sizeof(struct Tamper_params_in_s) );		
		
		}		
		break;
	case TDIOCSG_WINDOW_RESULT:
		{
			struct TD_window_result_s* td_win_result ;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_window_result_s) ) ;
			td_win_result = (struct td_win_result* )payload ;
			
			//pr_info("===TDIOCSG_WINDOW_RESULT %d %d ===\n",td_win_result->w_num,td_win_result->h_num);
			ret = copy_from_user(td_win_result, argp, sizeof(struct TD_window_result_s));
			if (ret) {

				COM_PAYLOAD_FREE( sizeof(struct TD_window_result_s) );
				goto done;
			}
			ret = cpub_td_get_window_result(h_dma);
			//pr_info("===TDIOCSG_WINDOW_RESULT end===\n");
			if(ret)
			{
			
        			COM_PAYLOAD_FREE(sizeof(struct TD_window_result_s));
				pr_err("cpub_md_get_windowsize error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			
			}			

			ret = copy_to_user(argp, td_win_result, sizeof(struct TD_window_result_s)) ;
			COM_PAYLOAD_FREE(sizeof(struct TD_window_result_s));
			if (ret) {
				goto done;
			}						
		}
		break;
	case TDIOCSG_BUFFER_SIZE:
		{
			struct TD_buffer_size_s* pBufsize ;
      			COM_PAYLOAD_ALLOC( sizeof(struct TD_buffer_size_s) ) ;  
      			pBufsize = (struct TD_buffer_size_s *)payload ;

			ret = copy_from_user(pBufsize, argp, sizeof(struct TD_buffer_size_s));
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct TD_buffer_size_s));
				goto done;
			}
			if(debug) {
			  	pr_info("===TDIOCSG_BUFFER_SIZE===(0x%08x,0x%08x)\n",(u32)pBufsize,h_dma );
			  	pr_info("width = %d\n",pBufsize->width);
			  	pr_info("height = %d\n",pBufsize->height);
			  	pr_info("color = %d\n",pBufsize->color);
			  	pr_info("w_div = %d\n",pBufsize->w_div);
			  	pr_info("h_div = %d\n",pBufsize->h_div);
			  	pr_info("return size = %d\n",pBufsize->return_size );
      			}
			ret= cpub_td_get_buffer_size(h_dma);
			if(ret<=0)
			{
				COM_PAYLOAD_FREE(sizeof(struct TD_buffer_size_s));
				pr_err("cpub_md_get_result error = %d!\n",ret);
				ret = -EFAULT;
				goto done;			
			}			
			pBufsize->return_size = ret;	//Temp For test
			ret = copy_to_user(argp, pBufsize, sizeof(struct TD_buffer_size_s)) ;

			pr_info("Size = %d\n",((TD_buffer_size_t*)argp)->return_size);
			
			COM_PAYLOAD_FREE(sizeof(struct TD_buffer_size_s));
			
			if (ret) {
				goto done;
			}

		}			
		break;
	case TDIOCS_WINDOW_EN :
		{
			struct TD_set_window_enable_s* win_en;
			COM_PAYLOAD_ALLOC( sizeof(struct TD_set_window_enable_s) ) ;
			win_en = (struct TD_set_window_enable_s *)payload ;
			ret = copy_from_user(payload, argp, sizeof(struct TD_set_window_enable_s));
			
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct TD_set_window_enable_s)) ;
				ret = -EFAULT;
				goto done;
			}
			
			if(debug) {
	  			pr_info("===TDIOCS_WINDOW_EN===(0x%08x,0x%08x)\n",(u32)win_en,(u32)h_dma);
			}
			ret = cpub_td_set_windows_en(h_dma);
			COM_PAYLOAD_FREE(sizeof(struct TD_set_window_enable_s) ) ;
		}
		break;
	case TDIOCG_WINDOW_EN:
		{
			struct TD_get_window_enable_s* win_en ;
      			COM_PAYLOAD_ALLOC( sizeof(struct TD_get_window_enable_s) ) ;  
      			win_en = (struct TD_get_window_enable_s *)payload ;

			ret = copy_from_user(win_en, argp, sizeof(struct TD_get_window_enable_s));
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct TD_get_window_enable_s));
				goto done;
			}
			if(debug) {
			  	pr_info("===TDIOCG_WINDOW_EN===(0x%08x,0x%08x)\n",(u32)win_en,h_dma );

      			}
			ret= cpub_td_get_windows_en(h_dma);
			if(ret < 0)
			{
				COM_PAYLOAD_FREE(sizeof(struct TD_get_window_enable_s));
				pr_err("cpub_md_get_result error = %d!\n",ret);
				ret = -EFAULT;
				goto done;			
			}			
			win_en->en = ret;	//Temp For test
			ret = copy_to_user(argp, win_en, sizeof(struct TD_buffer_size_s)) ;
		
			COM_PAYLOAD_FREE(sizeof(struct TD_buffer_size_s));
			
			if (ret) {
				goto done;

			}
		}
		break;  
	case TDIOCS_RELEASE:
		{
			if(debug) {
	  			pr_info("===TTDIOCS_RELEASE===\n");
			}
			ret = cpub_td_release();		
		}
		break; 
	 	default:
		ret = -ENOTTY;
	}

done:
	return ret;
}

ssize_t ait_md_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	md_event_data_t *md_data = (md_event_data_t *)filp->private_data;
	int cnt ,len = (count < sizeof(int) ) ? count : sizeof(int) ;
	int i,r;
	read_lock( &md_data->lock );
	cnt = md_data->md_cnt ;
	read_unlock( &md_data->lock);
  copy_to_user(buf, &md_data->md_cnt , len );
  if( count > len ) {
    r = count - len ;
    for(i=0;i<r;i++) {
      copy_to_user( &buf[i],0,1);
    }  
  }
  
  return count ;
}


static ssize_t ait_md_write(struct file *file, const char *data, size_t len,  loff_t *ppos)
{
	return len;
}

void ait_md_wakeup(unsigned int event_id,int val)
{
  write_lock( &md_event_data.lock);
  md_event_data.md_event= 1 ;
  md_event_data.md_cnt = val ;
  write_unlock( &md_event_data.lock);
  wake_up(&ait_md_waitq);
}

static unsigned int ait_md_poll(struct file *file, struct poll_table_struct *pt)
{
	md_event_data_t *md_data = (md_event_data_t *)file->private_data;
  
  unsigned int mask = 0 ;
  poll_wait(file, &ait_md_waitq, pt);
  if(md_data->md_event) {
    md_data->md_event = 0;
    mask = POLLIN | POLLRDNORM ;    
  }
  return mask; 
}


static int ait_md_close(struct inode *inode, struct file *filp)
{
  if(debug) {
	  printk(KERN_ALERT "%s call.\n", __func__);
  }
	if (filp->private_data) {
		//kfree(filp->private_data);
		filp->private_data = NULL;
	}
  open_cnt = 0;
	return 0;
}

static int ait_md_open(struct inode *inode, struct file *filp)
{
  md_event_data_t *md_data ;  
	if(debug) {
	  printk(KERN_ALERT "%s call.\n", __func__);
  }
  if(open_cnt) {
    printk("MD device busy\n");
    return -EBUSY ;
  }
  init_waitqueue_head(&ait_md_waitq);
  md_data = &md_event_data ;
  rwlock_init(&md_data->lock);
  md_data->md_cnt   = 0 ;
  md_data->md_event = 0 ;
  filp->private_data = md_data;
  open_cnt = 1 ;
  /*
  try to enable event handler here
  */
  if( cpub_md_enable_event(1) ) {
    enable_md_event(0) ;
  }
  else {
    enable_md_event(1) ;
  }
  
  
	return 0;
}

static const struct file_operations ait_cpub_md_fops = {
	.owner = THIS_MODULE,

	.open = ait_md_open,
	.release = ait_md_close,
	.read = ait_md_read,
	.write = ait_md_write,
	.poll = ait_md_poll,//unsigned int (*poll) (struct file *, struct poll_table_struct *);
	
	.unlocked_ioctl = ait_md_ioctl,
	.llseek = no_llseek,

	
};

static struct miscdevice ait_cpub_md_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ait_md",
	.fops = &ait_cpub_md_fops,
};

static int __devinit ait_md_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	cpub_md_register(CPU_COMM_ID_MD);
	ret = misc_register(&ait_cpub_md_miscdev);
	if (ret < 0) {
		dev_err(dev, "cannot register ait_md misc device\n");
	} 
	init_waitqueue_head(&ait_md_waitq);
	open_cnt = 0 ;
	return ret;
}

static int __devexit ait_md_remove(struct platform_device *pdev)
{
	cpub_md_unregister();
  misc_deregister(&ait_cpub_md_miscdev);
  open_cnt= 0;
	return 0;
}


static struct platform_driver ait_md_driver = {
	.driver = {
		.name = "ait_md",
		.owner	= THIS_MODULE,
	},
	.probe = ait_md_probe,
	.remove = __devexit_p(ait_md_remove),
};

static int __init ait_md_init(void)
{
	return platform_driver_register(&ait_md_driver);
}

static void __exit ait_md_exit(void)
{
	platform_driver_unregister(&ait_md_driver);
}


module_init(ait_md_init);
module_exit(ait_md_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("This is ioctl for MD algo. module.");


//module_param(heartbeat, int, 0);
//MODULE_PARM_DESC(heartbeat,
//		 "Watchdog heartbeat period in seconds from 1 to "
//		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
//		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:cpub_md");
