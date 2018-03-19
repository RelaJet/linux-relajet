/*
 *  linux/arch/arm/mach-mcrv2/cpucomm/cpub_mgr.c - AIT CPU Communication Driver
 *
 *  Copyright (C) 2015 Alpha Image Techenology , All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <mach/cpucomm/cpucomm_id.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm_cmd.h>
#include <mach/ait_cpu_sharemem.h>

#define DEF_DMA_POOL_BLK_SIZE (64)
#define MAX_RTOS_FW_SIZE  (512*1024)

// ITCM Address
#define CPUA_ITCM_ADDR          0x00800000
#define CPUB_ITCM_ADDR          AIT_CPUB_ITCM_VIRT_BASE
#define CPUB_ITCM_SIZE          AIT_CPUB_ITCM_SIZE      // 8K

// SRAM Address
#define CPUB_SRAM_ADDR          MMU_TRANSLATION_TABLE_ADDR
#define CPUB_SRAM_SIZE          0x00004000      // 16K

// DRAM Address for uC/OS2
#define CPUB_DRAM_ADDR_UCOS2    AIT_CPUB_DRAM_VIRT_BASE
#define CPUB_DRAM_SIZE_UCOS2    AIT_CPUB_DRAM_SIZE      // 128K

// DRAM Address for Linux
#if 0
#define CPUB_DRAM_ADDR_LINUX    0x03000000      // DRAM strart address for Linux
#define CPUB_DRAM_SIZE_LINUX    0x02000000      // DRAM size for Linux
#define CPUB_ZIMG_ADDR_LINUX    0x04000000      // zImage entry address
#define CPUB_RDSK_ADDR_LINUX    0x04800000      // initrd start address
#define CPUB_RDSK_SIZE_LINUX    0x00800000      // initrd size
#endif

//#define CPUB_ITCM_FILE_UCOS2    "/etc/firmware/ITCM_EXE_UCOS2"
//#define CPUB_DRAM_FILE_UCOS2    "/etc/firmware/ALL_DRAM_UCOS2"
#define CPUB_ITCM_FILE_UCOS2    CONFIG_AIT_CPUB_BOOTCODE_PATH
#define CPUB_DRAM_FILE_UCOS2    CONFIG_AIT_CPUB_FW_PATH


#define CPUB_UBOOT_ADDR			CPUB_ZIMG_ADDR_LINUX
#define CPUB_DRAM_FILE_UBOOT	"SD:\\u-boot.bin"

MMP_ERR MMPF_SYS_StartStopCpuB(int halt);

int pool_blk_size = 0;
module_param(pool_blk_size, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pool_blk_size, "DMA Pool Blk Size");

int max_cpub_fw_load_size = 0;
module_param(max_cpub_fw_load_size, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_cpub_fw_load_size, "MAX CPUB FW SIZE");

typedef struct _RegionData
{
    const char  *pszFilePath;
    MMP_UBYTE   *pbyTargetAddr;
    MMP_ULONG   ulTargetSize;
    MMP_UBYTE   *pybTempAddr;
} RegionData;

static RegionData sUcos2Regions[] =
{
	{ CPUB_ITCM_FILE_UCOS2, (MMP_UBYTE *)CPUB_ITCM_ADDR,  CPUB_ITCM_SIZE, (MMP_UBYTE *)CPUB_DRAM_ADDR_UCOS2 },
	{ CPUB_DRAM_FILE_UCOS2, (MMP_UBYTE *)CPUB_DRAM_ADDR_UCOS2, CPUB_DRAM_SIZE_UCOS2,   (MMP_UBYTE *)NULL }
};

static struct dma_pool *cpub_dma_pool ;
static struct proc_dir_entry *proc_dual_cpu_cmd_info;
static struct task_struct *cpu_cmd_tsk;
static struct proc_dir_entry *haltcpub_proc;

static char haltcpub ; 
static int haltcpub_dump(struct seq_file *m, void *v)
{
  AITPS_CORE      pCORE = AITC_BASE_CORE;
  if(haltcpub) {
    haltcpub = 1 ;
  }
  seq_printf(m, "haltcpub:%d\n", haltcpub);
  seq_printf(m, "wficpua:%d\n", (pCORE->CORE_A_CFG & CPU_SLEEP_MODE_EN ) ? 1 : 0 );
  seq_printf(m, "wficpub:%d\n", (pCORE->CORE_B_CFG & CPU_SLEEP_MODE_EN ) ? 1 : 0 );
  
  return 0;
}

static int haltcpub_open(struct inode *inode, struct  file *file)
{
	return single_open(file, haltcpub_dump, NULL);
}

static ssize_t haltcpub_write(struct file *file, const char *buffer, size_t len, loff_t * off)
{
  #if 1
  int ret = 0,run;
  char tmp[128],halt = 0 ;
  char who[64] ;
  
  if(len > 0) {
    AITPS_CORE      pCORE = AITC_BASE_CORE;
    ret = copy_from_user( (void *)&tmp , buffer, len ) ;
    sscanf(tmp,"%s %d",who,&run) ;
    //printk("%s -> %d\n",who,run );
    if ( !strcmp(who,"haltcpub") ) {
      halt = run ;
      if(haltcpub != halt) {
        haltcpub = halt ;
        MMPF_SYS_StartStopCpuB(haltcpub);
      }
    }
    else if ( !strcmp(who,"wficpua") ) {
      if(run) {
        pCORE->CORE_A_CFG |= CPU_SLEEP_MODE_EN ;  
      }
      else {
        pCORE->CORE_A_CFG &= ~CPU_SLEEP_MODE_EN ;
      }
    }
    else if ( !strcmp(who,"wficpub") ) {
      if(run) {
        pCORE->CORE_B_CFG |= CPU_SLEEP_MODE_EN ;  
      }
      else {
        pCORE->CORE_B_CFG &= ~CPU_SLEEP_MODE_EN ;
      }
      
    }
    return len;
  }
  #endif
  
  return 0 ;
}

static int cpub_create_pool(void)
{
    if(!pool_blk_size) {
      pool_blk_size =  DEF_DMA_POOL_BLK_SIZE  ;
    }      
      
    cpub_dma_pool = dma_pool_create("cpub_pool", 0,pool_blk_size, 16, 0);
    if(!cpub_dma_pool) {
        return -ENOMEM ;
    }
    return 0 ;
}

static int cpub_delete_pool(void)
{
    if(cpub_dma_pool) {
        dma_pool_destroy(cpub_dma_pool);        
    }    
    return 0 ;
}

void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma)
{
    //dma_addr_t h_dma ;
    if(!cpub_dma_pool) {
        return (void *) 0 ;    
    }
    if(size > pool_blk_size) {
      return dma_alloc_coherent(0,size,h_dma, flags) ;
   
    } else {
      return dma_pool_alloc(cpub_dma_pool,flags,h_dma);
    }
}
EXPORT_SYMBOL(cpub_malloc) ; 

void cpub_free(int size,void *va,dma_addr_t h_dma)
{
    if(size > pool_blk_size) {
      dma_free_coherent(0,  size, va,h_dma);
      
    } else {
      dma_pool_free(cpub_dma_pool,va,h_dma);
    }
}
EXPORT_SYMBOL(cpub_free);

struct file* file_open(const char* path, int flags, int rights) {
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}

void file_close(struct file* file) {
	filp_close(file, NULL);
}

int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}

static MMP_ULONG MMPF_SYS_ReadCodeRegion( RegionData* psRegionData )
{
	int /*ulFileId,*/ ulReadSize;
	//MMP_ULONG64 ulFileSize;
	//MMP_ERR   ulRet;
	void *cpu_addr;//[4096];// = NULL;
	dma_addr_t dma_addr ;
	//int i;


	struct file *fp = file_open( psRegionData->pszFilePath, O_RDONLY, 0);

	pr_debug("read itcm file %s\r\n",psRegionData->pszFilePath);
	
	// Open file
	if(fp ==NULL)
	{
		pr_err( "can't open file\r\n" );
		return 0;
	}

	//memblock = kmalloc(MAX_RTOS_FW_SIZE,GFP_KERNEL );
	//cpu_addr = (void *)dma_alloc_coherent(0, MAX_RTOS_FW_SIZE, &dma_addr , GFP_KERNEL);
	if( !max_cpub_fw_load_size ) {
	  max_cpub_fw_load_size = MAX_RTOS_FW_SIZE ;
	}
	cpu_addr = (void *)cpub_malloc(max_cpub_fw_load_size,GFP_KERNEL, &dma_addr);
	
	if(!cpu_addr) {
	    pr_err("allocate %d bytes buffer failed\r\n",max_cpub_fw_load_size); 
	    file_close(fp);
	    return 0 ;   
	}
	ulReadSize = file_read(fp, 0, cpu_addr,  max_cpub_fw_load_size);

	if( ulReadSize <0 )
	{
		pr_err( "can't read file\r\n" );
		ulReadSize = 0;
		goto RETURN;
	}
	pr_err( "Load %s into 0x%08x ,size = %d,bufaddr = 0x%08x\r\n",psRegionData->pszFilePath,(u32)psRegionData->pbyTargetAddr,ulReadSize ,(u32)cpu_addr);
	
	memcpy( (void*)psRegionData->pbyTargetAddr, (void*)cpu_addr, ulReadSize );
	// Copy data from temproal buffer to target address
	// We need this setp because some HW can't access some memory, such ash ITCM
	
	//pr_info( "load %s to %08lx\r\n", psRegionData->pszFilePath, psRegionData->pbyTargetAddr );

	RETURN:

  //dma_free_coherent(0, MAX_RTOS_FW_SIZE, cpu_addr,dma_addr);
  if(max_cpub_fw_load_size > 0) {
    cpub_free(max_cpub_fw_load_size,cpu_addr,dma_addr);
  }
	// Close file
	file_close(fp);
	//kfree(memblock);
	return ulReadSize;
}

MMP_ERR MMPF_SYS_StartCpuB( RegionData* psRegionData, MMP_ULONG ulRegionData )
{

	AITPS_CORE  pCORE = AITC_BASE_CORE;
	MMP_ULONG   ulFileSize, i;
	int ret = MMP_ERR_NONE;
	if( max_cpub_fw_load_size < 0 ) {
		pr_info("cpu/ipc : wait remote site...\n");
		goto skip_load;  
	}
  //pGBL->GBL_CLK_EN[0] |= (GBL_CLK_CPU_B | GBL_CLK_CPU_B_PHL ) ;
	//Enable CPUB clock
	pCORE->CORE_B_WD_CFG |= HOST_RST_CPU_EN;
	//Enable CPUA to CPUB download mode
	pCORE->CORE_MISC_CFG &=~ (CPU_A2B_DNLD_EN | CPU_B2A_DNLD_EN);
	pCORE->CORE_MISC_CFG |= CPU_A2B_DNLD_EN;
	//Hang CPUB
	pCORE->CORE_B_CFG |= CPU_RST_EN;

	for( i=0; i<ulRegionData; i++ )
	{
	    ulFileSize = MMPF_SYS_ReadCodeRegion( psRegionData+i );
	    if( ulFileSize == 0 )
	    {
	        pr_err( "can't load %s\r\n", psRegionData[i].pszFilePath );
	        haltcpub = 1;
	        return MMP_SYSTEM_ERR_CPUBOOT;
	    }
	}

	//Release CPUB
	pCORE->CORE_B_WD_CFG &= ~HOST_RST_CPU_EN;
	pCORE->CORE_B_CFG &=~ CPU_RST_EN;
  haltcpub = 0;
	pr_info( "CPUB is startting....\r\n" );
skip_load:
	// Wait CPU B init done
	//DualCpu_SysFlag_WaitCpuBInitDone();
#if !defined(CONFIG_CPU_SHARE_MEM)
	ret = CpuComm_SemWait( CPU_COMM_ID_SYSFLAG, 0 );
	if( ret != CPU_COMM_ERR_NONE )
	{
	    pr_err( "WaitCpuB failed %X\r\n", ret );
		return ret;
	}
#else

#if !defined(CONFIG_AIT_FAST_BOOT)
	wait_cpub_ready(MAX_CPU_QUEUE);
	pr_info("cpu/ipc : ready\n");
#else
	pr_info("cpu/ipc : ready...no wait\n");
#endif	

#endif	
	//pr_info("cpucomm: CPUB Init Done exit MMPF_SYS_StartCpuB\n");

	return ret;//MMP_ERR_NONE;
}

struct task_struct	*task = 0;
struct task_struct	*mdtask = 0;

MMP_ERR MMPF_SYS_StartStopCpuB(int halt)
{
	AITPS_CORE  pCORE = AITC_BASE_CORE;
  AITPS_GBL   pGBL  = AITC_BASE_GBL;
  if( halt ) {
    pGBL->GBL_CLK_DIS[0] |= (GBL_CLK_CPU_B | GBL_CLK_CPU_B_PHL);
  }
  else {
    pGBL->GBL_CLK_EN[0] |= (GBL_CLK_CPU_B | GBL_CLK_CPU_B_PHL);
  }
  printk("cpub : %s\n",(halt)?"halt":"run");

	return MMP_ERR_NONE;
}

struct file_operations haltcpub_ops =
{
	.open	  = haltcpub_open,
	.read	  = seq_read,
	.llseek	= seq_lseek,
	.write	= haltcpub_write,
	.owner	= THIS_MODULE,
};

static int cpub_mgr_probe(struct platform_device *pdev)
{
	pr_info("%s\n",__func__);

	if(cpub_create_pool() < 0) {
		pr_err("cpub_mgr : create cpub dma pool failed\n");
		return -ENOMEM;  
	}
#if defined(CONFIG_CPU_SHARE_MEM)
	if(init_cpu_share_mem(MAX_CPU_QUEUE) != 0)
	{
		printk("Initial sharemem Fail\n");
		return -EIO;
	}
#endif
#if defined(CONFIG_AIT_CPUB_CMD_THD_MODULE)
	//start cpu command thread
	start_cpu_cmd_thd();
#endif

	if( MMPF_SYS_StartCpuB(sUcos2Regions, sizeof(sUcos2Regions)/sizeof(RegionData)) != MMP_ERR_NONE ) {
	    exit_cpu_share_mem(MAX_CPU_QUEUE) ;
	    return -EIO ;
	}

  // create an interface to start/stop cpu-b
  // note : cpu-b must has code first.
  haltcpub_proc = proc_create("cpu_sharemem/haltcpu", 0666, NULL, &haltcpub_ops);
  if(haltcpub_proc == NULL)
  {
  	printk("ERROR: Can't create haltcpub\n");
  }

	return 0;
}

static int cpub_mgr_remove(struct platform_device *pdev)
{
	pr_info("%s\n",__func__);
#if defined(CONFIG_CPU_SHARE_MEM)
	exit_cpu_share_mem(MAX_CPU_QUEUE);
#endif
	cpub_delete_pool();
#if defined(CONFIG_AIT_CPUB_CMD_THD_MODULE)
	stop_cpu_cmd_thd();
#endif
	close_send_dev_id(0, CPU_COMM_ID_CMD);
	close_recv_dev_id(0, CPU_COMM_ID_CMD);

	return 0;
}

static void cpub_mgr_shutdown(struct platform_device *pdev)
{
	pr_info("%s\n",__func__);
}

static int cpub_mgr_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s\n",__func__);
	return 0;
}
static int cpub_mgr_resume(struct platform_device *pdev)
{
	pr_info("%s\n",__func__);
	return 0;
}

static struct platform_driver ait_cpub_mgr_driver = {

	.probe =  cpub_mgr_probe,//int (*probe)(struct platform_device *);
	.remove =  __devexit_p(cpub_mgr_remove),//int (*remove)(struct platform_device *);
	.shutdown = cpub_mgr_shutdown,//void (*shutdown)(struct platform_device *);
	.suspend = cpub_mgr_suspend,//int (*suspend)(struct platform_device *, pm_message_t state);
	.resume = cpub_mgr_resume,//int (*resume)(struct platform_device *);
	.driver = {//struct device_driver driver;
		.name	= "cpub_mgr",
		.owner	= THIS_MODULE,
	},

};

static int __init cpub_mgr_init(void)
{
	//int i;
	int err = -EINVAL;
	pr_info("%s\n",__func__);
	err = platform_driver_register(&ait_cpub_mgr_driver);
	if(err)
	{
		printk(KERN_ERR"cpub_mgr: platform_driver_probe fail!\n");
	}
	
	return err;
}

static void __exit cpub_mgr_exit(void)
{
	platform_driver_unregister(&ait_cpub_mgr_driver );
}

module_init(cpub_mgr_init);
module_exit(cpub_mgr_exit);

MODULE_DESCRIPTION("AIT CPUB Management Driver ");
MODULE_AUTHOR("Vincent");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:AIT CPUB Management Driver");

