/*
 * Driver for AES alog. on AIT CPUB
 *
 * Copyright (C) 2015 Sean @ AIT
 *
 * arch/arm/mach-mcrv2/cpucomm/aes_algo/ait_aes.c which has following copyrights:
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
//#include "../cpub_ringbuf.h"
#include "ait_aes.h"
#include "cpub_aes_cpucomm.h"


extern void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
extern void cpub_free(int size,void *va,dma_addr_t h_dma) ;
#define AES_KEY_LEN   (244)
#define AES_IV_LEN    (16)

#define AESENC_MAX_BLK_SIZE       (8*1024)

#define COM_PAYLOAD_ALLOC(size) do {    \
                                    payload = cpub_malloc(size,GFP_KERNEL,&h_dma);    \
                                    if(!payload) {          \
                                        return -ENOMEM ;    \
                                    }                       \
                                } while (0)
                                
                                     
#define COM_PAYLOAD_FREE(size) cpub_free(size,payload,h_dma )

static DECLARE_WAIT_QUEUE_HEAD(ait_aes_waitq);

#define DRIVER_NAME "ait_aes"
//static unsigned int cpub_aes_ioctl_major = CPUCOMM_MAJOR;
//static unsigned int num_of_dev = 1;
//static struct cdev cpub_md_ioctl_cdev;
//static int ioctl_num = 0;

struct aes_ioctl_data {
    rwlock_t lock;
    struct mutex  ctl_mutex;
    dma_addr_t  in_dma_addr,out_dma_addr ;
    dma_addr_t  key_dma_addr,iv_dma_addr ;
    
    void *in,*out;
    void *iv,*key;
    
};

static long ait_aes_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0 ;
  struct aes_ioctl_data *paes = (struct aes_ioctl_data *)file->private_data;
	void *payload ;
	dma_addr_t h_dma ;
	switch (cmd) {
  
	case AESIOCS_RUN:
		{
		  dma_addr_t dma_addr_key,dma_addr_iv ;
		  
		  char *out_frame ;
      struct AES_proc_info_s *aes_proc_to_cpub,aes_proc ,aes_user ;
      //pr_info("--run.lock--\n");
      mutex_lock(&paes->ctl_mutex);
      
			COM_PAYLOAD_ALLOC( sizeof(struct AES_proc_info_s ) ) ;
			aes_proc_to_cpub = (struct AES_proc_info_s *)payload ;
			
			ret = copy_from_user(payload, argp, sizeof(struct AES_proc_info_s));
			aes_user = *aes_proc_to_cpub ;
      #if 0
      pr_info("aes_user.len:%d\n",aes_user.len) ;
      pr_info("aes_user.in :0x%08x\n",aes_user.in ) ;
      pr_info("aes_user.out :0x%08x\n",aes_user.out ) ;
      pr_info("aes_user.key :0x%08x\n",aes_user.key ) ;
      pr_info("aes_user.iv :0x%08x\n",aes_user.iv ) ;
      #endif
      
			if(ret) {
			  pr_info("payload failed:%d\n",ret);
				goto exit_run;
			}
			
      if(aes_user.len ) {
        ret = copy_from_user( paes->in,aes_user.in, aes_user.len  );
        if(ret) {
          pr_info("paes->in failed:%d\n",ret);
          goto exit_run ;
        }
        ret = copy_from_user(paes->key, aes_user.key, AES_KEY_LEN);
        if( ret ) {
          pr_info("paes->key failed:%d\n",ret);
          goto exit_run ;  
        } 
        ret = copy_from_user(paes->iv  ,aes_user.iv   ,AES_IV_LEN);
        if(ret) {
          pr_info("paes->iv failed:%d\n",ret);
          goto exit_run ;
        }
			  aes_proc_to_cpub->iv  = (unsigned char *)paes->iv_dma_addr;
        aes_proc_to_cpub->key = (void *)paes->key_dma_addr ;
 			  aes_proc_to_cpub->in  = (unsigned char *)paes->in_dma_addr ;
			  aes_proc_to_cpub->out = (unsigned char *)paes->out_dma_addr;
			  //pr_info("enc.s(%d)\n",aes_user.len);			  
			  ret = cpub_aes_run( /*aes_proc_to_cpub*/ (struct AES_proc_info_s *)h_dma );			  
			  if(ret) {
			    pr_info("cpub_aes_run failed:%d\n",ret);  
			    goto exit_run ;
			  }
			  //pr_info("enc.e\n",aes_user.len);			  
			  ret = copy_to_user(aes_user.out,paes->out,aes_user.len );
			  if(ret) {
			    pr_info("aes_user.out failed:%d\n",ret);
			  }
		  }
		  else {
		    pr_info("Bad encrypt size....\n");
		    ret = -EINVAL ; 
		  }
exit_run:
      mutex_unlock(&paes->ctl_mutex);
      //pr_info("--run.unlock(%d)--\n",ret );
      COM_PAYLOAD_FREE(sizeof(struct AES_proc_info_s )) ;
		}		
		break;
		default:
		  ret = -EINVAL;
	}
done:
	return ret;
}

ssize_t ait_aes_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct aes_ioctl_data *ioctl_data = filp->private_data;
	unsigned char val;
	int retval = 0 ;
	int i = 0;

	read_lock(&ioctl_data->lock);
	//val = ioctl_data->val;
	read_unlock(&ioctl_data->lock);


	retval = count;

	return retval;
}


static ssize_t
ait_aes_write(struct file *file, const char *data, size_t len,  loff_t *ppos)
{

	return len;
}


static unsigned int ait_aes_poll(struct file *file, struct poll_table_struct *pt)
{
    unsigned int mask = POLLIN | POLLRDNORM;  
    poll_wait(file, &ait_aes_waitq, pt);  
    return mask; 

}


static int ait_aes_close(struct inode *inode, struct file *filp)
{
	printk(KERN_ALERT "%s call.\n", __func__);
	if (filp->private_data) {
	  struct aes_ioctl_data *paes = (struct aes_ioctl_data *)filp->private_data;
	  
	  if(paes->in) {
        cpub_free(AESENC_MAX_BLK_SIZE,paes->in,paes->in_dma_addr);	   
        paes->in = 0 ; 
    }
    if(paes->out) {
        cpub_free(AESENC_MAX_BLK_SIZE,paes->out,paes->out_dma_addr);	 
        paes->out=0 ;   
    }
    if(paes->key) {
        cpub_free(AES_KEY_LEN,paes->key,paes->key_dma_addr);	 
        paes->key=0 ;   
    }
    if(paes->iv) {
        cpub_free(AES_IV_LEN,paes->iv,paes->iv_dma_addr);	 
        paes->iv=0 ;   
    }
    
    kfree(filp->private_data);
    filp->private_data = NULL;
	}

	return 0;
}

static int ait_aes_open(struct inode *inode, struct file *filp)
{
	//ring_bufctl_t *bufctl ; //= pdata->bufctl ;
	struct aes_ioctl_data *paes;
	printk(KERN_ALERT "%s call.\n", __func__);
	pr_info( "%s call.\n", __func__);

	paes = kmalloc(sizeof(struct aes_ioctl_data), GFP_KERNEL);
	if (paes == NULL)
		return -ENOMEM;
  
	rwlock_init(&paes->lock);
	mutex_init(&paes->ctl_mutex);
	paes->in = (void *)cpub_malloc(AESENC_MAX_BLK_SIZE , GFP_KERNEL,&paes->in_dma_addr);
	if(!paes->in) {
	    return -ENOMEM ;
	}	
	
	paes->out = (void *)cpub_malloc(AESENC_MAX_BLK_SIZE , GFP_KERNEL,&paes->out_dma_addr);
	if(!paes->out) {
	    cpub_free(AESENC_MAX_BLK_SIZE,paes->in,paes->in_dma_addr);
	    return -ENOMEM ;
	}	
	
	paes->key = cpub_malloc(AES_KEY_LEN,GFP_KERNEL, &paes->key_dma_addr );
	if(!paes->key) {
	    cpub_free(AESENC_MAX_BLK_SIZE,paes->in ,paes->in_dma_addr );
	    cpub_free(AESENC_MAX_BLK_SIZE,paes->out,paes->out_dma_addr);
	    return -ENOMEM ;	    
	}
	
	paes->iv = cpub_malloc(AES_IV_LEN,GFP_KERNEL, &paes->iv_dma_addr );
	if(!paes->iv) {
	    cpub_free(AESENC_MAX_BLK_SIZE,paes->in ,paes->in_dma_addr );
	    cpub_free(AESENC_MAX_BLK_SIZE,paes->out,paes->out_dma_addr);
	    cpub_free(AES_KEY_LEN,paes->key,paes->key_dma_addr);
	    
	    return -ENOMEM ;	    
	  
	}
	
	filp->private_data = paes;	
	pr_info("aes open :%d\n",filp->f_count);
	pr_info("aes dma in.buffer  : (0x%08x,0x%08x),size:%d\n", (u32)paes->in ,paes->in_dma_addr  ,AESENC_MAX_BLK_SIZE);
	pr_info("aes dma out.buffer : (0x%08x,0x%08x),size:%d\n", (u32)paes->out,paes->out_dma_addr ,AESENC_MAX_BLK_SIZE);
	pr_info("aes dma key.buffer  : (0x%08x,0x%08x),size:%d\n", (u32)paes->in ,paes->in_dma_addr  ,AES_KEY_LEN);
	pr_info("aes dma iv.buffer : (0x%08x,0x%08x),size:%d\n", (u32)paes->out,paes->out_dma_addr ,AES_IV_LEN);
	return 0;
}

static const struct file_operations ait_cpub_aes_fops = {
	.owner = THIS_MODULE,

	.open = ait_aes_open,
	.release = ait_aes_close,
	.read = ait_aes_read,
	.write = ait_aes_write,
	.poll = ait_aes_poll,
	.unlocked_ioctl = ait_aes_ioctl,
	.llseek = no_llseek,
};

static struct miscdevice ait_cpub_aes_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ait_aes",
	.fops = &ait_cpub_aes_fops,
};


static int __devinit ait_aes_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	cpub_aes_register(CPU_COMM_ID_AES);
	ret = misc_register(&ait_cpub_aes_miscdev);
	if (ret < 0) {
		dev_err(dev, "cannot register ait_aes misc device\n");
	} 
	return ret;
}

static int __devexit ait_aes_remove(struct platform_device *pdev)
{
	cpub_aes_unregister();
  misc_deregister(&ait_cpub_aes_miscdev);
	return 0;
}


static struct platform_driver ait_aes_driver = {
	.driver = {
		.name = "ait_aes",
		.owner	= THIS_MODULE,
	},
	.probe = ait_aes_probe,
	.remove = __devexit_p(ait_aes_remove),
};

static int __init ait_aes_init(void)
{
	return platform_driver_register(&ait_aes_driver);
}

static void __exit ait_aes_exit(void)
{
	platform_driver_unregister(&ait_aes_driver);
}


module_init(ait_aes_init);
module_exit(ait_aes_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean");
MODULE_DESCRIPTION("This is ioctl for AES algo. module.");


//module_param(heartbeat, int, 0);
//MODULE_PARM_DESC(heartbeat,
//		 "Watchdog heartbeat period in seconds from 1 to "
//		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
//		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_ALIAS_MISCDEV(MISC_DYNAMIC_MINOR);
MODULE_ALIAS("platform:cpub_aes");
