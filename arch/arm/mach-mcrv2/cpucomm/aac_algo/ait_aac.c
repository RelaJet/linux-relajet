/*
 * Driver for AAC alog. on AIT CPUB
 *
 * Copyright (C) 2015 Sean @ AIT
 *
 * arch/arm/mach-mcrv2/cpucomm/aac_algo/ait_aac.c which has following copyrights:
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
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <mach/cpucomm/cpucomm_id.h>
#include "../cpub_ringbuf.h"
#include "ait_aac.h"
#include "cpub_aac_cpucomm.h"
#define AAC_VER "1.0.5"
// 1.0.5 : support 2 aac handles

extern void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
extern void cpub_free(int size,void *va,dma_addr_t h_dma) ;

#define MAX_SID                 2
#define MAX_CHANNELS            2
#define AACENC_INBUF_FRAME_NUM  8
#define AACENC_INBUF_SIZE       (1024*MAX_CHANNELS*AACENC_INBUF_FRAME_NUM * sizeof(short))

#define AACENC_OUTBUF_SIZE      (16*1024)

#define COM_PAYLOAD_ALLOC(size) do {    \
                                    payload = cpub_malloc(size,GFP_KERNEL,&h_dma);    \
                                    if(!payload) {          \
                                        return -ENOMEM ;    \
                                    }                       \
                                } while (0)
                                
                                     
#define COM_PAYLOAD_FREE(size) cpub_free(size,payload,h_dma )

static DECLARE_WAIT_QUEUE_HEAD(ait_aac_waitq);

#define DRIVER_NAME "ait_aac"
//static unsigned int cpub_aac_ioctl_major = CPUCOMM_MAJOR;
//static unsigned int num_of_dev = 1;
//static struct cdev cpub_md_ioctl_cdev;
//static int ioctl_num = 0;
static int aac_stream_id = 0 ;
static int aac_stream_id_flag[] = { 0,0 } ;

static RINGBUF_ID _BUFID_IN[]  = {BUFID_IN ,BUFID1_IN };
static RINGBUF_ID _BUFID_OUT[] = {BUFID_OUT,BUFID1_OUT};

struct aac_ioctl_data {
    rwlock_t lock;
    int          sid;
    AAC_init_t   aac_enc_info ;
    ring_bufctl_t *bufctl ;
    dma_addr_t   bufctl_dma_addr ;
    aac_options_t aac_options ;
};

static int ait_aac_get_sid(void)
{
  int i;
  for(i=0;i<MAX_SID;i++) {
    if(!aac_stream_id_flag[i]) {
      aac_stream_id_flag[i] = 1 ;
      return i ;
    }
  }
  return -1 ;
}

static int ait_aac_free_sid(int sid)
{
    aac_stream_id_flag[sid] = 0 ;
}
static unsigned short ait_aac_get_len(int id)
{
  unsigned short samples ;
  unsigned char out_frame[2] = {0,0} ;
  MMPF_Audio_ReadBuf(_BUFID_OUT[id],(char *)out_frame, 2) ;
  samples = (unsigned short)(out_frame[0] | out_frame[1] << 8 );
  return samples ;
}

static long ait_aac_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	//int err = 0;
	//unsigned char val;
	void __user *argp = (void __user *)arg;
	//int __user *p = argp;

	//struct test_ioctl_data *ioctl_data = file->private_data;
	int ret = -ENOTTY;

  struct aac_ioctl_data *pdata = (struct aac_ioctl_data *)file->private_data;
	//ring_bufctl_t *bufctl = pdata->bufctl ;
  
	//int size;
	void *payload ;
	dma_addr_t h_dma ;

	
	switch (cmd) {
	
	case AACIOCS_INIT:
		{
		  struct AAC_init_s   *pInitInfo ;
			COM_PAYLOAD_ALLOC( sizeof(struct AAC_init_s) ) ;
			pInitInfo = (struct AAC_init_s  *)payload ;
			
			
			ret = copy_from_user(payload, argp, sizeof(struct AAC_init_s));
			
			if (ret) {
				COM_PAYLOAD_FREE(sizeof(struct AAC_init_s)) ;
				ret = -EFAULT;
				goto done;
			}
			copy_from_user( &pdata->aac_options ,pInitInfo->priv,sizeof(aac_options_t) ) ;
			
      pdata->aac_enc_info      =  *pInitInfo ;
      pdata->aac_enc_info.priv =  (void *)pdata->bufctl_dma_addr ;
      pInitInfo->priv          =  pdata->aac_enc_info.priv ;
      
			pr_info("===AACIOCS_INIT(%d)===(0x%08x,0x%08x)\n",pdata->sid,(u32)pInitInfo,(u32)h_dma);
			pr_info("opts : 0x%08x\n",pdata->aac_options.options );
			pr_info("channels = %d\n",pdata->aac_enc_info.channel);
			pr_info("samplerate = %d\n",pdata->aac_enc_info.samplerate);
			pr_info("bitrate = %d\n",pdata->aac_enc_info.bitrate);
			pr_info("priv = 0x%08x\n",pdata->aac_enc_info.priv);

			cpub_aac_init( pdata->sid,(struct AAC_init_s*)h_dma);

      COM_PAYLOAD_FREE(sizeof(struct AAC_init_s)) ;
			//kfree(pInitInfo);			
		}
		break;

  /*
  input samples should be 2048 bundary, no handle for ring.
  */
	case AACIOCSG_RUN:
		{
		  //void *kpcm, *kpcm_dma ;
		  //char *out_frame ;
      struct AAC_proc_info_s *aac_proc ;
			COM_PAYLOAD_ALLOC( sizeof(struct AAC_proc_info_s ) ) ;
			aac_proc = (struct AAC_proc_info_s *)payload ;
			
			ret = copy_from_user(payload, argp, sizeof(struct AAC_proc_info_s));
			
			
			if(ret)
			{
			  COM_PAYLOAD_FREE(sizeof(struct AAC_proc_info_s )) ;  
				ret = -EFAULT;
				goto done;
			}

			if(aac_proc->samples) {
			  MMPF_Audio_WriteBufFromUser(_BUFID_IN[pdata->sid],aac_proc->frame,  aac_proc->samples *sizeof(short) ) ;
			}
			
			//pr_info("===AACIOCS_RUN===(%d)\n",aac_proc->samples);
			// process to check ring buffer, no need to pass frame address
			ret = cpub_aac_run(pdata->sid, pdata->aac_options.options & (1 << AAC_ACK_OFF) ) ;
	  
			// overwrite the input frames
			//if(aac_proc->samples) {
			if( MMPF_Audio_BufDataAvailable(_BUFID_OUT[pdata->sid]) > 0 ) {
			  
			  aac_proc->samples =  ait_aac_get_len(pdata->sid);
        //pr_info("[aac.ioctl][%d] : size:%d,copy to : 0x%08x\n",pdata->sid,aac_proc->samples,aac_proc->out_frame);
        
        if( !(pdata->aac_options.options & (1 << ADTS_HEADER_ON) ) ) {
          // move ADTS 7 bytes header
          MMPF_Audio_AdvanceBufReadPtr(_BUFID_OUT[pdata->sid],7) ;  
          aac_proc->samples -= 7 ;
        }
        
			  MMPF_Audio_ReadBufToUser(_BUFID_OUT[pdata->sid],(char *)aac_proc->out_frame,aac_proc->samples);
		  }
		  else {
		    pr_info("no aac data\n");
		    aac_proc->samples = 0;  
		  }
			copy_to_user(argp, payload, sizeof(struct AAC_proc_info_s)) ;
			
			
			COM_PAYLOAD_FREE(sizeof(struct AAC_proc_info_s )) ;
			if(ret)
			{
				pr_err("aac_run error = %d!\n",ret);
				ret = -EFAULT;
				goto done;
			}		
		}			
		break;
		default:
		ret = -ENOTTY;
	}

done:
	return ret;
}

ssize_t ait_aac_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct aac_ioctl_data *ioctl_data = filp->private_data;
//	unsigned char val;
	int retval = 0 ;
	//int i = 0;

	read_lock(&ioctl_data->lock);
	//val = ioctl_data->val;
	read_unlock(&ioctl_data->lock);


	retval = count;

//out:
	return retval;
}


static ssize_t
ait_aac_write(struct file *file, const char *data, size_t len,  loff_t *ppos)
{

	return len;
}


static unsigned int ait_aac_poll(struct file *file, struct poll_table_struct *pt)
{
    unsigned int mask = POLLIN | POLLRDNORM;  
    poll_wait(file, &ait_aac_waitq, pt);  
    return mask; 

}


static int ait_aac_close(struct inode *inode, struct file *filp)
{
	printk(KERN_ALERT "%s call.\n", __func__);
	if (filp->private_data) {
	  struct aac_ioctl_data *pdata = (struct aac_ioctl_data *)filp->private_data;
	  ring_bufctl_t *bufctl = pdata->bufctl ;
	  if(bufctl) {
  	  if(bufctl->dma_inbuf_virt_addr) {
          cpub_free(bufctl->dma_inbuf_buffer_size ,bufctl->dma_inbuf_virt_addr,bufctl->dma_inbuf_phy_addr);	   
          bufctl->dma_inbuf_virt_addr = 0 ; 
      }
      if(bufctl->dma_outbuf_virt_addr) {
           cpub_free(bufctl->dma_outbuf_buffer_size ,bufctl->dma_outbuf_virt_addr,bufctl->dma_outbuf_phy_addr);	 
           bufctl->dma_outbuf_virt_addr = 0 ;   
      }
      // fixed : i remeber to release.
      cpub_free( sizeof(ring_bufctl_t),  bufctl , pdata->bufctl_dma_addr ) ;
    }
    ait_aac_free_sid(pdata->sid) ;
    kfree(filp->private_data);
    filp->private_data = NULL;
    aac_stream_id--;
	}

	return 0;
}

static int ait_aac_open(struct inode *inode, struct file *filp)
{
	ring_bufctl_t *bufctl ; //= pdata->bufctl ;
	struct aac_ioctl_data *paac;
	printk(KERN_ALERT "%s call.\n", __func__);

  if(aac_stream_id >= MAX_SID) {
    pr_info("aac open cnt:%d( > 1 )\n",aac_stream_id);
    return -ENOMEM;
  }
	paac = kmalloc(sizeof(struct aac_ioctl_data), GFP_KERNEL);
	if (paac == NULL)
		return -ENOMEM;
  
  paac->bufctl = cpub_malloc( sizeof(ring_bufctl_t),GFP_KERNEL ,&paac->bufctl_dma_addr ) ;
  if(!paac->bufctl) {
    kfree(paac);
    filp->private_data = 0 ;
    return -ENOMEM ;
  }
  bufctl = paac->bufctl ;
	rwlock_init(&paac->lock);
	
	bufctl->dma_inbuf_buffer_size  = AACENC_INBUF_SIZE;
	bufctl->dma_inbuf_virt_addr = (void *)cpub_malloc(bufctl->dma_inbuf_buffer_size , GFP_KERNEL,&bufctl->dma_inbuf_phy_addr );
	if(!bufctl->dma_inbuf_virt_addr) {
	    return -ENOMEM ;
	}	
	
	bufctl->dma_outbuf_buffer_size  = AACENC_OUTBUF_SIZE;
	bufctl->dma_outbuf_virt_addr = (void *)cpub_malloc(bufctl->dma_outbuf_buffer_size , GFP_KERNEL,&bufctl->dma_outbuf_phy_addr );
	if(!bufctl->dma_outbuf_virt_addr) {
	    cpub_free(bufctl->dma_inbuf_buffer_size,bufctl->dma_inbuf_virt_addr,bufctl->dma_inbuf_phy_addr);
	    bufctl->dma_inbuf_virt_addr = 0 ;
	    return -ENOMEM ;
	}	

	paac->sid = ait_aac_get_sid() ;
	
	MMPF_Audio_InitBufHandle(_BUFID_IN[paac->sid ],&bufctl->in_buf_h, (char *)bufctl->dma_inbuf_virt_addr,  (char *)bufctl->dma_inbuf_phy_addr ,bufctl->dma_inbuf_buffer_size );
	MMPF_Audio_InitBufHandle(_BUFID_OUT[paac->sid ],&bufctl->out_buf_h,(char *)bufctl->dma_outbuf_virt_addr, (char *)bufctl->dma_outbuf_phy_addr ,bufctl->dma_outbuf_buffer_size );
	

	//AUTL_RingBuf_Init(&bufctl->in_buf_h ,bufctl->dma_inbuf_virt_addr, bufctl->dma_inbuf_phy_addr ,bufctl->dma_inbuf_buffer_size );
	//AUTL_RingBuf_Init(&bufctl->out_buf_h,bufctl->dma_outbuf_virt_addr,bufctl->dma_outbuf_phy_addr ,bufctl->dma_outbuf_buffer_size );
	filp->private_data = paac;	
	pr_info("aac bufctl[%d] : (0x%08x,0x%08x),size:%d\n", paac->sid,paac->bufctl,paac->bufctl_dma_addr ,sizeof(ring_bufctl_t));
	pr_info("aac dma in.buffer  : (0x%08x,0x%08x),size:%d\n", (u32)bufctl->dma_inbuf_virt_addr,bufctl->dma_inbuf_phy_addr ,bufctl->dma_inbuf_buffer_size);
	pr_info("aac dma out.buffer : (0x%08x,0x%08x),size:%d\n", (u32)bufctl->dma_outbuf_virt_addr,bufctl->dma_outbuf_phy_addr ,bufctl->dma_outbuf_buffer_size);
	aac_stream_id++ ;
	return 0;
}

static const struct file_operations ait_cpub_aac_fops = {
	.owner = THIS_MODULE,

	.open = ait_aac_open,
	.release = ait_aac_close,
	.read = ait_aac_read,
	.write = ait_aac_write,
	.poll = ait_aac_poll,
	.unlocked_ioctl = ait_aac_ioctl,
	.llseek = no_llseek,
};

static struct miscdevice ait_cpub_aac_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ait_aac",
	.fops = &ait_cpub_aac_fops,
};


static int __devinit ait_aac_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	cpub_aac_register(CPU_COMM_ID_AAC);
	ret = misc_register(&ait_cpub_aac_miscdev);
	if (ret < 0) {
		dev_err(dev, "cannot register ait_aac misc device\n");
	} 
	aac_stream_id = 0 ;
	{
	  int i;
	  for(i=0;i<MAX_SID;i++) {
	    ait_aac_free_sid(i);
	  }  
	}
	pr_info("AIT cpub aac driver ver :%s\n",AAC_VER);
	return ret;
}

static int __devexit ait_aac_remove(struct platform_device *pdev)
{
	cpub_aac_unregister();
  misc_deregister(&ait_cpub_aac_miscdev);
	return 0;
}


static struct platform_driver ait_aac_driver = {
	.driver = {
		.name = "ait_aac",
		.owner	= THIS_MODULE,
	},
	.probe = ait_aac_probe,
	.remove = __devexit_p(ait_aac_remove),
};

static int __init ait_aac_init(void)
{
	return platform_driver_register(&ait_aac_driver);
}

static void __exit ait_aac_exit(void)
{
	platform_driver_unregister(&ait_aac_driver);
}


module_init(ait_aac_init);
module_exit(ait_aac_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean");
MODULE_DESCRIPTION("This is ioctl for AAC algo. module.");


//module_param(heartbeat, int, 0);
//MODULE_PARM_DESC(heartbeat,
//		 "Watchdog heartbeat period in seconds from 1 to "
//		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
//		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_ALIAS_MISCDEV(MISC_DYNAMIC_MINOR);
MODULE_ALIAS("platform:cpub_aac");
