/* 
 *   Copyright (C) 2014
 *
 *	This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <mach/ait_cpu_sharemem.h>
#include <mach/mmp_cpucomm.h>
#include <mach/cpucomm/cpucomm_api.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/mmp_register.h>
//#define CPU_SHARE_DEBUG
#ifdef CPU_SHARE_DEBUG
#define cs_dbg(format, arg...) printk( format, ##arg)
#else
#define cs_dbg(format, arg...)
#endif

//------------------------------------------------------------------------------------REGISTER_COMM
#define SUPPORT_REGISTER_COMM
#ifdef SUPPORT_REGISTER_COMM
#define CPUCOMM_CHN     0 // with ack
#define CPUCOMM_CHN_ISR 1 // without ack
struct semaphore send_semaphore[MAX_DEV_ID];
unsigned int cpucomm_send_ack(void *slot)
{
	struct cpu_share_mem_slot *S_slot = (struct cpu_share_mem_slot *)slot;
	struct cpu_comm_transfer_data *data = (struct cpu_comm_transfer_data *)S_slot->send_parm[3];
	data->result = S_slot->recv_parm[0];	
	up(&(send_semaphore[S_slot->dev_id]));
	return 0;
}

CPU_COMM_ERR CpuComm_RegisterEntry(CPU_COMM_ID ulCommId, CPU_COMM_TYPE ulCommType)
{
	if(init_send_dev_id(CPUCOMM_CHN, ulCommId) != 0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
	
	if(init_recv_dev_id(CPUCOMM_CHN, ulCommId) != 0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
  #if MAX_CPU_QUEUE > 1
	if(init_send_dev_id(CPUCOMM_CHN_ISR, ulCommId) != 0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
	
	if(init_recv_dev_id(CPUCOMM_CHN_ISR, ulCommId) != 0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
  #endif
	
	sema_init(&(send_semaphore[ulCommId]), 0);

	cs_dbg("%s ulCommId = 0x%x\n", __func__, ulCommId);
	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_RegisterEntry);

CPU_COMM_ERR CpuComm_UnregisterEntry(CPU_COMM_ID ulCommId)
{
	if(close_send_dev_id(CPUCOMM_CHN, ulCommId) !=0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}

	if(close_recv_dev_id(CPUCOMM_CHN, ulCommId) !=0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
  #if MAX_CPU_QUEUE > 1
	if(close_send_dev_id(CPUCOMM_CHN_ISR, ulCommId) !=0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}

	if(close_recv_dev_id(CPUCOMM_CHN_ISR, ulCommId) !=0)
	{
		return CPU_COMM_ERR_INIT_FAIL;
	}
  #endif
	cs_dbg("%s ulCommId = 0x%x\n", __func__, ulCommId);
	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_UnregisterEntry);

CPU_COMM_ERR CpuComm_RegisterISRService(CPU_COMM_ID ulCommId , unsigned int (*recv_cb)(void *slot))
{
    #if MAX_CPU_QUEUE > 1 
  	if(enable_send_dev_id_noack(CPUCOMM_CHN_ISR, ulCommId) != 0)
  	{
  		return CPU_COMM_ERR_INIT_FAIL;
  	}
  	if(enable_recv_dev_id_noack(CPUCOMM_CHN_ISR, ulCommId, recv_cb) != 0)
  	{
  		return CPU_COMM_ERR_INIT_FAIL;
  	}
  	#endif
    return CPU_COMM_ERR_NONE ;
}
EXPORT_SYMBOL(CpuComm_RegisterISRService);


CPU_COMM_ERR CpuComm_SemPost(CPU_COMM_ID ulCommId, MMP_ULONG ulTimeout)
{
	cs_dbg("%s !!!\n", __func__);
	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_SemPost);

CPU_COMM_ERR CpuComm_SemWait(CPU_COMM_ID ulCommId, MMP_ULONG ulTimeout)
{
	cs_dbg("%s !!!\n", __func__);
	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_SemWait);

CPU_COMM_ERR CpuComm_DataSend(CPU_COMM_ID ulCommId, MMP_UBYTE *pbyData, MMP_ULONG ulDataSize, MMP_ULONG ulTimeout)
{
	struct cpu_share_mem_slot *S_slot;
	struct cpu_comm_transfer_data *data = (struct cpu_comm_transfer_data *)pbyData;

	while(1)
	{
		if(get_slot(CPUCOMM_CHN, &S_slot) == 0)
		{
			S_slot->dev_id = ulCommId;
			S_slot->command = data->command;
			S_slot->data_phy_addr = data->phy_addr;
			S_slot->size = data->size;
			S_slot->send_parm[0] = data->seq;
			S_slot->send_parm[1] = data->flag;
			S_slot->send_parm[2] = data->result;
			S_slot->send_parm[3] = (unsigned int)pbyData;
			S_slot->ack_func = cpucomm_send_ack;
			send_slot(CPUCOMM_CHN, S_slot);
			down(&(send_semaphore[S_slot->dev_id]));
			break;
		}else{
			cs_dbg("%s can't get slot\n", __func__);
		}
	}

	if(data->result != CPUCOMM_FLAG_RESULT_OK)
	{
		printk("%s result = 0x%x\n", __func__, (unsigned int)data->result);
		return data->result;
	}

	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_DataSend);

CPU_COMM_ERR CpuComm_DataReceive(CPU_COMM_ID ulCommId, MMP_UBYTE *pbyData, MMP_ULONG ulDataSize, MMP_ULONG ulTimeout, CPUCOMM_RCV_PREPROC psPreproc)
{
	struct cpu_share_mem_slot *R_slot;
	struct cpu_comm_transfer_data *data = (struct cpu_comm_transfer_data *)pbyData;

	while(1)
	{
		if(recv_slot(CPUCOMM_CHN, ulCommId, &R_slot, 0) == 0)
		{
			data->command = R_slot->command;
			data->phy_addr = R_slot->data_phy_addr;
			data->size = R_slot->size;
			data->seq = R_slot->send_parm[0];
			data->flag = R_slot->send_parm[1];
			data->result = R_slot->send_parm[2];
			R_slot->recv_parm[0] = CPUCOMM_FLAG_RESULT_OK;
			ack_slot(CPUCOMM_CHN, R_slot);
			break;
		}else{
			cs_dbg("%s queue is empty after release\n", __func__);
		}
	}

	return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_DataReceive);

#endif
//------------------------------------------------------------------------------------REGISTER_COMM

static struct _cpu_share_mem_info *csm_info = NULL;
static struct proc_dir_entry *cpu_sharemem, *proc_queue_info, *proc_key_info;
static struct proc_dir_entry *fb_dump ;


static int queue_info_dump(struct seq_file *m, void *v)
{
	int i, q;
	struct cpu_share_mem_slot *temp_slot;

	if(csm_info == NULL)
	{
		seq_printf(m, "No queue information for Cpu sharemem!!!\n");
	}else{
		for(q=0;q<MAX_CPU_QUEUE;q++)
		{
			if(csm_info->send_queue_info[q].init_done == 1)
			{
				seq_printf(m, "Send Queue%dinfo:\n", q);
				seq_printf(m, "Send Queue%d v_addr = 0x%x\n", q,csm_info->send_queue_info[q].slot_v_st_addr);
				seq_printf(m, "Send Queue%d p_addr = 0x%x\n", q,csm_info->send_queue_info[q].slot_p_st_addr);
				seq_printf(m, "Send Queue%d total_slot = 0x%x\n", q,csm_info->send_queue_info[q].total_slot);
				seq_printf(m, "Send Queue%d free_slot = 0x%x\n", q,csm_info->send_queue_info[q].free_slot);
				seq_printf(m, "Send Queue%d used_slot = 0x%x\n", q,csm_info->send_queue_info[q].used_slot);
				seq_printf(m, "Send Queue%d wait_ack_slot = 0x%x\n", q, csm_info->send_queue_info[q].wait_ack_slot);
				seq_printf(m, "Send Queue%d dev_id_map = 0x%llx\n",q, csm_info->send_queue_info[q].dev_id_map);
				seq_printf(m, "Send Queue%d dev_id_no_ack = 0x%llx\n",q, csm_info->send_queue_info[q].dev_id_no_ack);
				seq_printf(m, "Send Queue%d seq = 0x%x\n", q, csm_info->send_queue_info[q].seq);
				for(i=0;i<csm_info->send_queue_info[q].total_slot;i++)
				{
					temp_slot = (struct cpu_share_mem_slot *)(csm_info->send_queue_info[q].slot_v_st_addr+(i * sizeof(struct cpu_share_mem_slot)));
					seq_printf(m, "Send Queue%d Slot %d status = 0x%x seq = 0x%x dev_id = 0x%x command = 0x%x\n", q, i,\
							temp_slot->slot_status, temp_slot->seq, temp_slot->dev_id, temp_slot->command);
				}
				seq_printf(m, "\n");
			}else{
				seq_printf(m, "No Send_queue information for Cpu sharemem!!!\n");
			}
		
			if(csm_info->recv_queue_info[q].init_done == 1)
			{
				seq_printf(m, "Recv Queue%d info:\n", q);
				seq_printf(m, "Recv Queue%d v_addr = 0x%x\n", q, csm_info->recv_queue_info[q].slot_v_st_addr);
				seq_printf(m, "Recv Queue%d p_addr = 0x%x\n", q, csm_info->recv_queue_info[q].slot_p_st_addr);
				seq_printf(m, "Recv Queue%d dev_id_map = 0x%llx\n", q, csm_info->recv_queue_info[q].dev_id_map);
				seq_printf(m, "Recv Queue%d dev_id_no_ack = 0x%llx\n", q, csm_info->recv_queue_info[q].dev_id_no_ack);
				for(i=0;i<csm_info->recv_queue_info[q].total_slot;i++)
				{
					temp_slot = (struct cpu_share_mem_slot *)(csm_info->recv_queue_info[q].slot_v_st_addr+(i * sizeof(struct cpu_share_mem_slot)));
					seq_printf(m, "Recv Queue%d Slot %d status = 0x%x seq = 0x%x dev_id = 0x%x command = 0x%x\n", q, i,\
							temp_slot->slot_status, temp_slot->seq, temp_slot->dev_id, temp_slot->command);
				}
				seq_printf(m, "\n");
			}else{
				seq_printf(m, "No Recv_queue information for Cpu sharemem!!!\n");
			}
		}
	}

	return 0;
}

static int queue_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, queue_info_dump, NULL);
}

static int key_info_dump(struct seq_file *m, void *v)
{
  volatile unsigned char *ptr = (volatile unsigned char *)( &AITC_BASE_CPU_IPC->CPU_IPC[4] ) ;
  if(csm_info == NULL) {
  }
  else {
    seq_printf(m, "%d\n", ptr[0]);  
  }
  return 0;
}

static int key_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, key_info_dump, NULL);
}


static int fb_dump_open(struct inode *inode, struct  file *file)
{
	return 0;//try_module_get(THIS_MODULE);
}

static int fb_dump_close(struct inode *inode, struct  file *file)
{
	//module_put(THIS_MODULE);
	return 0;
}

static u32 fb_cur_addr ;
static u32 fb_cur_size ;
static u32 fb_cur_pos  ;
static ssize_t fb_dump_read(struct file *filp,char *buffer,	size_t len,loff_t * off)
{
  u32 read_len = 0 ;
  
  char *virt_addr ;
  
  if(len > 0) {
    read_len = ( fb_cur_size > len )?len:fb_cur_size;
    virt_addr = (char *)AIT_RAM_P2V(fb_cur_addr + fb_cur_pos);
    copy_to_user( buffer, virt_addr,read_len );
    fb_cur_pos  += read_len ;
    fb_cur_size -= read_len ;
    //pr_info("fb.read[0x%08x,0x%08x]:%d\n",virt_addr,fb_cur_addr + fb_cur_pos ,len);  
  }
  return read_len ;
}

static ssize_t fb_dump_write(struct file *file, const char *buffer, size_t len, loff_t * off)
{
  if(len > 0) {
    char *data =  kmalloc( len , GFP_KERNEL);
    if(data) {
      copy_from_user( data, buffer, len ) ;
      sscanf(data,"%d:%d",&fb_cur_addr,&fb_cur_size) ;
      //pr_info("FB.dump,phy addr:0x%08x,size : %d bytes\n",fb_cur_addr,fb_cur_size);
      fb_cur_pos = 0 ;
      kfree(data);
    }
  }
  return len ;
}

struct file_operations proc_q_ops =
{
	.open	= queue_info_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.owner	= THIS_MODULE,
};

struct file_operations proc_key_ops =
{
	.open	= key_info_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.owner	= THIS_MODULE,
};

struct file_operations proc_fb_ops =
{
	.open	  = fb_dump_open,
	.read	  = fb_dump_read,
	.write	= fb_dump_write,
	.release= fb_dump_close,
	.owner	= THIS_MODULE,
};

static void triger_cpux_int(void)
{
	AITPS_HINT pHINT = csm_info->baseaddr_hint;

#if defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA)
	cs_dbg("%s Triger_CPUB\n", __func__);
	pHINT->HINT_SET_CPU_INT = HINT_CPU2CPUB_INT;
#elif defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUB)
	cs_dbg("%s Triger_CPUA\n", __func__);
	pHINT->HINT_SET_CPU_INT = HINT_CPU2CPUA_INT;
#else
#error Not define DUAL_OS_ON_XXXX
#endif
}

static void clear_cpux_int(void)
{
	AITPS_HINT pHINT = csm_info->baseaddr_hint;

#if defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA)
	cs_dbg("%s Clear_CPUA\n", __func__);
	pHINT->HINT_CLR_CPU_INT = HINT_CPU2CPUA_INT;
#elif defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUB)
	cs_dbg("%s Clear_CPUB\n", __func__);
	pHINT->HINT_CLR_CPU_INT = HINT_CPU2CPUB_INT;
#else
#error Not define DUAL_OS_ON_XXXX
#endif
}

static void set_slot_free(struct _queue_info *qi, unsigned int slot_num)
{
	qi->free_slot = qi->free_slot | (1 << slot_num);
	qi->used_slot = qi->used_slot & ~(1 << slot_num);
	qi->wait_ack_slot = qi->wait_ack_slot & ~(1 << slot_num);
}

static void set_slot_used(struct _queue_info *qi, unsigned int slot_num)
{
	qi->free_slot = qi->free_slot & ~(1 << slot_num);
	qi->used_slot = qi->used_slot | (1 << slot_num);
}

static void set_slot_wait(struct _queue_info *qi, unsigned int slot_num)
{
	qi->wait_ack_slot = qi->wait_ack_slot | (1 << slot_num);
}

unsigned int get_share_register(void)
{
	return (unsigned int)csm_info->baseaddr_share; 
}
EXPORT_SYMBOL_GPL(get_share_register);

int init_send_queue(unsigned int channel, unsigned int slot_v_st_addr, unsigned int slot_p_st_addr, unsigned int total_slot)
{
	struct cpu_share_mem_slot *temp_slot;
	int i;

	//check channel and slot number
	if((channel >= MAX_SEND_QUEUE) || (total_slot > MAX_SLOT))
	{
		printk("ERROR: channel or total_slot are invalid. channel = 0x%x total_slot = 0x%x\n", channel, total_slot);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->send_queue_info[channel].init_done == 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}

	//initial queue info
	csm_info->send_queue_info[channel].init_done = 1;
	csm_info->send_queue_info[channel].seq = 0;
	csm_info->send_queue_info[channel].slot_v_st_addr = slot_v_st_addr;
	csm_info->send_queue_info[channel].slot_p_st_addr = slot_p_st_addr;
	csm_info->send_queue_info[channel].total_slot = total_slot;
	csm_info->send_queue_info[channel].free_slot = (unsigned int)((1 << total_slot) - 1);
	csm_info->send_queue_info[channel].used_slot = 0;
	csm_info->send_queue_info[channel].wait_ack_slot = 0;
	csm_info->send_queue_info[channel].dev_id_map = 0;
	csm_info->send_queue_info[channel].dev_id_no_ack = 0;
	
	spin_lock_init(&(csm_info->send_queue_info[channel].queue_info_lock));

	//initial slot info
	for(i=0;i<total_slot;i++)
	{
		temp_slot = (struct cpu_share_mem_slot *)(slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
		memset(temp_slot, 0, sizeof(struct cpu_share_mem_slot));
		temp_slot->hdr_phy_addr = (unsigned int)(slot_p_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
		temp_slot->slot_direct = L_TO_R_SLOT;
		temp_slot->slot_num = i;
		temp_slot->slot_status = SLOT_FREE;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(init_send_queue);

int exit_send_queue(unsigned int channel)
{
  csm_info->send_queue_info[channel].init_done = 0 ;
  return 0 ;
}
EXPORT_SYMBOL_GPL(exit_send_queue);

int init_send_dev_id(unsigned int channel, unsigned int dev_id)
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *sqi;
	cs_dbg("%s = 0x%x\n", __func__, dev_id);

	//check channel
	if((channel >= MAX_SEND_QUEUE) || (dev_id > MAX_DEV_ID))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->send_queue_info[channel].init_done != 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}else{
		sqi = &(csm_info->send_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(sqi->queue_info_lock), flags);

	//check active bitmap
	if((sqi->dev_id_map & (1 << dev_id)) == (1 << dev_id))
	{
		spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);
		printk("ERROR: dev_id have enabled\n");
		return -EBUSY;
	}else{
		sqi->dev_id_map = sqi->dev_id_map | (1 << dev_id);
	}

	//unlock queue info
	spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(init_send_dev_id);

int enable_send_dev_id_noack(unsigned int channel, unsigned int dev_id)
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *sqi;

	//check channel
	if((channel >= MAX_SEND_QUEUE) || (dev_id > MAX_DEV_ID))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->send_queue_info[channel].init_done != 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}else{
		sqi = &(csm_info->send_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(sqi->queue_info_lock), flags);

	//check active bitmap
	if((sqi->dev_id_no_ack & (1 << dev_id)) == (1 << dev_id))
	{
		spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);
		printk("ERROR: No ack options have enabled\n");
		return -EBUSY;
	}else{
		sqi->dev_id_no_ack = sqi->dev_id_no_ack | (1 << dev_id);
	}

	//unlock queue info
	spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(enable_send_dev_id_noack);

int close_send_dev_id(unsigned int channel, unsigned int dev_id)
{
	unsigned long flags;
	int ret=0, i;
	struct _queue_info *sqi;
	struct cpu_share_mem_slot *temp_slot;

	//check channel
	if((channel >= MAX_SEND_QUEUE) || (dev_id > MAX_DEV_ID))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->send_queue_info[channel].init_done != 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}else{
		sqi = &(csm_info->send_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(sqi->queue_info_lock), flags);

	//check active bitmap
	if((sqi->dev_id_map & (1 << dev_id)) == (1 << dev_id))
	{
		sqi->dev_id_map = sqi->dev_id_map & ~(1 << dev_id);
	}else{
		spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);
		printk("ERROR: dev_id is not enabled\n");
		return -EINVAL;
	}

	// check active no ack bitmap
	if((sqi->dev_id_no_ack & (1 << dev_id)) == (1 << dev_id)) {
	    sqi->dev_id_no_ack = sqi->dev_id_no_ack & ~(1 << dev_id);
	}


	for(i=0;i<sqi->total_slot;i++)
	{
		if(((sqi->wait_ack_slot & (1 << i)) == (1 << i)) || ((sqi->used_slot & (1 << i)) == (1 << i)))
		{
			temp_slot = (struct cpu_share_mem_slot *)(sqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
			temp_slot->slot_status = SLOT_FREE;
			set_slot_free(sqi, i);
		}
	}

	//unlock queue info
	spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(close_send_dev_id);

int get_slot(unsigned int channel, struct cpu_share_mem_slot **slot)
{
	unsigned long flags;
	int ret=0, i;
	struct _queue_info *sqi;
	cs_dbg("%s+++\n", __func__);

	//check channel
	if(channel >= MAX_SEND_QUEUE)
	{
		printk("ERROR: channel is invalid. channel = 0x%x", channel);
		return -EINVAL;
	}

	//check queue info
	if(csm_info->send_queue_info[channel].init_done != 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}else{
		sqi = &(csm_info->send_queue_info[channel]);
	}

	//check if slots are full
	if(sqi->free_slot == 0)
	{
		printk("ERROR: Slot is full\n");
		return -EBUSY;
	}

	//lock queue info
	spin_lock_irqsave(&(sqi->queue_info_lock), flags);

	//find free slot
	for(i=0;i<sqi->total_slot;i++)
	{
		if((sqi->free_slot & (1 << i)) == (1 << i))
		{
			//update slot status
			*slot = (struct cpu_share_mem_slot *)(sqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
			if((*slot)->slot_status != SLOT_FREE)  //No ack slot
			{
				cs_dbg("%s find no ack slot and continue\n", __func__);
				continue;
			}else{  //Normal slot
				(*slot)->slot_status = SLOT_GET;
				(*slot)->slot_direct = L_TO_R_SLOT;
				(*slot)->seq = sqi->seq;
				if((*slot)->slot_num != i)
				{
					printk("ERROR: slot num is not match bitmap!!!!\n");
				}

				//update queue status
				sqi->seq = (sqi->seq + 1) % 0xFFFF;
				set_slot_used(sqi, i);

				cs_dbg("%s find free slot = %d slot_vaddr = 0x%x slot_status = 0x%x\n", __func__, i, (unsigned int)*(slot), (*slot)->slot_status);
				break;
			}
		}
	}

	//unlock queue info
	spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);

	//check if all slot are used
	if(i == sqi->total_slot)
	{
		printk("ERROR: All slot are used\n");
		return -EBUSY;
	}

	cs_dbg("%s---\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(get_slot);

int send_slot(unsigned int channel, struct cpu_share_mem_slot *slot)
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *sqi;
	cs_dbg("%s channel = %d+++\n", __func__, channel);

	//check channel
	if(channel >= MAX_SEND_QUEUE)
	{
		printk("ERROR: channel is invalid. channel = 0x%x\n", channel);
		return -EINVAL;
	}

	//check queue info
	if(csm_info->send_queue_info[channel].init_done != 1)
	{
		printk("ERROR: send queue is not initial\n");
		return -EINVAL;
	}else{
		sqi = &(csm_info->send_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(sqi->queue_info_lock), flags);

	//check slot status and slot bitmap
	if(((sqi->used_slot & (1 << slot->slot_num)) != (1 << slot->slot_num)) || (slot->slot_status != SLOT_GET))
	{
		printk("ERROR: this slot(%d) is not in the list of used.(bitmap = 0x%x) status = %d.\n", slot->slot_num, sqi->used_slot, slot->slot_status);
		spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);
		return -EINVAL;
	}

	//change queue and slot status
	if((sqi->dev_id_no_ack & (1 << slot->dev_id)) != (1 << slot->dev_id))
	{
		set_slot_wait(sqi, slot->slot_num); //set slot to be wait.
	}else{
		set_slot_free(sqi, slot->slot_num); //For no ack slot we should free the slot but should not change the status.
	}
	slot->slot_status = SLOT_SEND;

	//triger interrupt
	triger_cpux_int();

	//unlock queue info
	spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);

	cs_dbg("%s---\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(send_slot);

int init_recv_queue(unsigned int channel, unsigned int slot_v_st_addr, unsigned int slot_p_st_addr, unsigned int total_slot)
{
	int i;

	//check channel and slot
	if((channel >= MAX_RECV_QUEUE) || (total_slot > MAX_SLOT))
	{
		printk("ERROR: channel or total_slot are invalid. channel = 0x%x total_slot = 0x%x\n", channel, total_slot);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done == 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EINVAL;
	}

	//initial queue info
	csm_info->recv_queue_info[channel].init_done = 1;
	csm_info->recv_queue_info[channel].seq = 0;
	csm_info->recv_queue_info[channel].slot_v_st_addr = slot_v_st_addr;
	csm_info->recv_queue_info[channel].slot_p_st_addr = slot_p_st_addr;
	csm_info->recv_queue_info[channel].total_slot = total_slot;
	csm_info->recv_queue_info[channel].free_slot = (unsigned int)((1 << total_slot) - 1);
	csm_info->recv_queue_info[channel].used_slot = 0;
	csm_info->recv_queue_info[channel].wait_ack_slot = 0;
	csm_info->send_queue_info[channel].dev_id_map = 0;
	csm_info->send_queue_info[channel].dev_id_no_ack = 0;
	
	
	spin_lock_init(&(csm_info->recv_queue_info[channel].queue_info_lock));
	for(i=0;i<MAX_DEV_ID;i++)
	{
		csm_info->recv_queue_info[channel].no_ack_func[i] = NULL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(init_recv_queue);

int exit_recv_queue(unsigned int channel)
{
  csm_info->recv_queue_info[channel].init_done = 0 ;
  return 0;
}
EXPORT_SYMBOL_GPL(exit_recv_queue);


int init_recv_dev_id(unsigned int channel, unsigned int dev_id)
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *rqi;
	cs_dbg("%s = 0x%x channel = %d\n", __func__, dev_id, channel);

	//check channel
	if((channel > MAX_RECV_QUEUE) || (dev_id > MAX_DEV_ID))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done != 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EBUSY;
	}else{
		rqi = &(csm_info->recv_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(rqi->queue_info_lock), flags);

	//check active bitmap
	if((rqi->dev_id_map & (1 << dev_id)) == (1 << dev_id))
	{
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
		printk("ERROR: dev_id have enabled\n");
		return -EBUSY;
	}else{
		rqi->dev_id_map = rqi->dev_id_map | (1 << dev_id);
	}

	//init semaphore
	sema_init(&(rqi->dev_sem[dev_id]), 0);

	//set bitmap to be zero
	rqi->recv_q_slot[dev_id] = 0;

	//unlock queue info
	spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(init_recv_dev_id);

int enable_recv_dev_id_noack(unsigned int channel, unsigned int dev_id, unsigned int (*no_ack_func)(void *slot))
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *rqi;

	//check channel
	if((channel > MAX_RECV_QUEUE) || (dev_id > MAX_DEV_ID) || (no_ack_func == NULL))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done != 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EBUSY;
	}else{
		rqi = &(csm_info->recv_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(rqi->queue_info_lock), flags);

	//check active bitmap
	if((rqi->dev_id_no_ack & (1 << dev_id)) == (1 << dev_id))
	{
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
		printk("ERROR: No ack options have enabled\n");
		return -EBUSY;
	}else{
		rqi->dev_id_no_ack = rqi->dev_id_no_ack | (1 << dev_id);
		rqi->no_ack_func[dev_id] = no_ack_func;
	}

	//unlock queue info
	spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(enable_recv_dev_id_noack);

int close_recv_dev_id(unsigned int channel, unsigned int dev_id)
{
	unsigned long flags;
	int ret=0, i;
	struct _queue_info *rqi;
	struct cpu_share_mem_slot *temp_slot;

	//check channel
	if((channel > MAX_RECV_QUEUE) || (dev_id > MAX_DEV_ID))
	{
		printk("ERROR: channel or dev_id are invalid channel = 0x%x dev_id = 0x%x\n", channel, dev_id);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done != 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EBUSY;
	}else{
		rqi = &(csm_info->recv_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(rqi->queue_info_lock), flags);

	//check active bitmap
	if((rqi->dev_id_map & (1 << dev_id)) == (1 << dev_id))
	{
		rqi->dev_id_map = rqi->dev_id_map & ~(1 << dev_id);
	}else{
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
		printk("ERROR: dev_id is not enabled\n");
		return -EINVAL;
	}

	// check active no ack bitmap
	if((rqi->dev_id_no_ack & (1 << dev_id)) == (1 << dev_id)) {
	    rqi->dev_id_no_ack = rqi->dev_id_no_ack & ~(1 << dev_id);
	}

	//scan slot and ack error
	for(i=0;i<rqi->total_slot;i++)
	{
		//check queue status 
		if((rqi->recv_q_slot[dev_id] & (1 << i)) == (1 << i))
		{
			temp_slot = (struct cpu_share_mem_slot *)(rqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
			temp_slot->recv_parm[0] = ACK_ID_DISABLE;
			temp_slot->slot_status = SLOT_ACK;
			triger_cpux_int();
		}
	}

	//unlock queue info
	spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(close_recv_dev_id);

int recv_slot(unsigned int channel, unsigned int dev_id, struct cpu_share_mem_slot **slot, unsigned int timeout)
{
	unsigned long flags;
	int ret=0, i;
	struct _queue_info *rqi;
	cs_dbg("%s channel = %d dev_id = 0x%x\n", __func__, channel, dev_id);

	//check channel and slot
	if(channel >= MAX_RECV_QUEUE)
	{
		printk("ERROR: channel is invalid. channel = 0x%x\n", channel);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done != 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EBUSY;
	}else{
		rqi = &(csm_info->recv_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(rqi->queue_info_lock), flags);

	//check if queue is empty or all slot are in process
	while((rqi->recv_q_slot[dev_id] == 0) || (rqi->recv_q_slot[dev_id] == rqi->recv_q_proc[dev_id]))
	{
		//unlock queue info
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

		//lock 
		cs_dbg("queue is empty and wait until slot is coming\n");
		down(&(rqi->dev_sem[dev_id]));

		//lock queue info
		spin_lock_irqsave(&(rqi->queue_info_lock), flags);

		if(rqi->recv_q_slot[dev_id] == 0)
		{
			//Somethings are wrong. we can still wait until queue is not empty
			cs_dbg("WARNING:Queue is still empty after release\n");
		}
	}

	//scan slot
	for(i=0;i<rqi->total_slot;i++)
	{
		//check queue status
		if((rqi->recv_q_slot[dev_id] & (1 << i)) == (1 << i))
		{
			*slot = (struct cpu_share_mem_slot *)(rqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
			if((*slot)->slot_status == SLOT_QUEUE)
			{
				//change the status. we will clean the slot status when we ack the slot. It can avoid two many up(release semaphore).
				//rqi->recv_q_slot[dev_id] = rqi->recv_q_slot[dev_id] & ~(1 << i);
				rqi->recv_q_proc[dev_id] = rqi->recv_q_proc[dev_id] | (1 << i);
				(*slot)->slot_status = SLOT_PROC;
				if((*slot)->slot_num != i)
				{
					printk("ERROR: slot num is not match bitmap!!!!\n");
				}
				cs_dbg("Recv one slot. dev_id = 0x%x. slot_num = 0x%x\n", (*slot)->dev_id, (*slot)->slot_num);
				break;
			}else if((*slot)->slot_status == SLOT_PROC){
				cs_dbg("This slot is already in proccess status\n");
			}else{
				//unlock queue info
				spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
				printk("ERROR: Slot status is not SLOT_QUEUE or SLOT_PROC when recv slot\n");
				return -EIO;
			}
		}
	}

	//unlock queue info
	spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(recv_slot);

int ack_slot(unsigned channel, struct cpu_share_mem_slot* slot)
{
	unsigned long flags;
	int ret=0;
	struct _queue_info *rqi;

	//check channel and slot
	if(channel >= MAX_RECV_QUEUE)
	{
		printk("ERROR: channel is invalid. channel = 0x%x\n", channel);
		return -EINVAL;
	}

	//check queue status
	if(csm_info->recv_queue_info[channel].init_done != 1)
	{
		printk("ERROR: recv queue is not initial\n");
		return -EBUSY;
	}else{
		rqi = &(csm_info->recv_queue_info[channel]);
	}

	//lock queue info
	spin_lock_irqsave(&(rqi->queue_info_lock), flags);

	//check slot status and change the status to be SLOT_ACK. We need to dequeuq the slot before ack
	if((slot->slot_status != SLOT_PROC))
	{
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
		return -EINVAL;
	}else{
		rqi->recv_q_slot[slot->dev_id] = rqi->recv_q_slot[slot->dev_id] & ~(1 << slot->slot_num);
		rqi->recv_q_proc[slot->dev_id] = rqi->recv_q_proc[slot->dev_id] & ~(1 << slot->slot_num);
		slot->slot_status = SLOT_ACK;
		triger_cpux_int();
	}

	//unlock queue info
	spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ack_slot);

void wait_cpub_ready(int max_chn)
{
	int i;
	volatile int *share_reg = (volatile int *)get_share_register();

	for(i=0;i<max_chn;i++)
	{
		while((*(share_reg+(i*2)) != 0) && (*(share_reg+(i*2)+1) != 0))
		{
        //cs_dbg("cpu_sharemem: channel %d share register -> 0x%x 0x%x\n", i, *(share_reg+(i*2)), *(share_reg+(i*2)+1));
        msleep(10);
		}
		printk("cpu_sharemem: channel %d OK\n", i);
	}
}
EXPORT_SYMBOL_GPL(wait_cpub_ready);

struct _cpu_rtos_mem_info *get_cpub_rtos_mem(void)
{
static int rtos_mem_got = 0 ;
static struct _cpu_rtos_mem_info cpu_rtos_mem ;  
  struct _cpu_rtos_mem_info *mem ;
  if(!rtos_mem_got) {
    mem = (struct _cpu_rtos_mem_info *)&AITC_BASE_CPU_SAHRE->CPU_SAHRE_REG[48] ;
    cpu_rtos_mem = *mem ;
    rtos_mem_got = 1 ;
  }  
  else {
    mem = &cpu_rtos_mem ;
  }
  /*
  printk("mem->reset_base : 0x%08x\n",mem->reset_base );
  printk("mem->v4l2_base : 0x%08x\n",mem->v4l2_base );
  printk("mem->working_size_mb : 0x%08x\n",mem->working_size_mb );
  printk("mem->v4l2_size_mb : 0x%08x\n",mem->v4l2_size_mb );
  */
  return mem ;
}
EXPORT_SYMBOL_GPL(get_cpub_rtos_mem);

static unsigned int sendq_v_addr[MAX_SEND_QUEUE],sendq_p_addr[MAX_SEND_QUEUE] ;
static unsigned int recvq_v_addr[MAX_RECV_QUEUE],recvq_p_addr[MAX_RECV_QUEUE] ;


int init_cpu_share_mem(int max_chn)
{
	unsigned int v_addr, p_addr, i;
	volatile int *share_reg = (volatile int *)get_share_register();
  
	for(i=0;i<max_chn;i++)
	{
	  sendq_v_addr[i] = sendq_p_addr[i] = 0 ;
	  recvq_v_addr[i] = recvq_p_addr[i] = 0 ;
	  
		v_addr = (unsigned int)dma_alloc_coherent(0, SZ_4K, &p_addr, GFP_KERNEL);
		if(v_addr) {
		  sendq_v_addr[i] = v_addr ;
		  sendq_p_addr[i] = p_addr ;
		}
		v_addr = ALIGN(v_addr, 0x40);
		p_addr = ALIGN(p_addr, 0x40);
		cs_dbg("Channel = %d Send v_addr = 0x%x p_addr = 0x%x\n", i, v_addr, p_addr);
		if(init_send_queue(i, v_addr, p_addr, MAX_SLOT) == 0)
		{
			*(share_reg+(i*2)) = p_addr;
		}else{
			printk("ERROR: CPU_SHARE_MEM_SEND_QUEUE FAIL\n");
			return -EIO;
		}

		v_addr = (unsigned int)dma_alloc_coherent(0, SZ_4K, &p_addr, GFP_KERNEL);
		if(v_addr) {
		  recvq_v_addr[i] = v_addr ;
		  recvq_p_addr[i] = p_addr ;
		}
		
		v_addr = ALIGN(v_addr, 0x40);
		p_addr = ALIGN(p_addr, 0x40);
		cs_dbg("Channel = %d Recv v_addr = 0x%x p_addr = 0x%x\n", i, v_addr, p_addr);
		if(init_recv_queue(i, v_addr, p_addr, MAX_SLOT) == 0)
		{
			*(share_reg+(i*2)+1) = p_addr;
		}else{
			printk("ERROR: CPU_SHARE_MEM_RECV_QUEUE FAIL\n");
			return -EIO;
		}
		cs_dbg("channel = %d share_reg = 0x%x 0x%x\n", i, *(share_reg+(i*2)), *(share_reg+(i*2)+1));
	}

	return 0;
}
EXPORT_SYMBOL_GPL(init_cpu_share_mem);

int exit_cpu_share_mem(int max_chn)
{
	unsigned int i;
	volatile int *share_reg = (volatile int *)get_share_register();

	for(i=0;i<max_chn;i++)
	{
	  
	  exit_send_queue(i) ;
	  exit_recv_queue(i) ;
	  
	  if(sendq_v_addr[i]) {
	    //pr_info("free v,p addr : 0x%08x,0x%08x\n",sendq_v_addr[i],sendq_p_addr[i]);
	    dma_free_coherent(0,  SZ_4K, (void *)sendq_v_addr[i],(dma_addr_t)sendq_p_addr[i]);
	  }
	  if(recvq_v_addr[i]) {
	    // pr_info("free v,p addr : 0x%08x,0x%08x\n",recvq_v_addr[i],recvq_p_addr[i]);
	    dma_free_coherent(0,  SZ_4K, (void *)recvq_v_addr[i],(dma_addr_t)recvq_p_addr[i]);
	  }
		
		cs_dbg("exit channel = %d \n",i);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exit_cpu_share_mem);

static irqreturn_t cpu_share_mem_irq(int irq, void *dev_id)
{
	unsigned long flags;
        struct _cpu_share_mem_info *csm_info = (struct _cpu_share_mem_info *)dev_id;
	int channel, i;
	struct _queue_info *rqi, *sqi;
	volatile struct cpu_share_mem_slot *temp_slot;

	//clear interrupt first to prevent next IRQ is missing
	clear_cpux_int();

	cs_dbg("%s +++\n", __func__);

	//check send queue
	for(channel=0;channel<MAX_SEND_QUEUE;channel++)
	{
		if(csm_info->send_queue_info[channel].init_done != 1)
		{
			continue;
		}else{
			sqi = &(csm_info->send_queue_info[channel]);
			cs_dbg("Check send channel %d\n", channel);
		}

		//lock queue info
		spin_lock_irqsave(&(sqi->queue_info_lock), flags);

		//check wait ack slot
		for(i=0;(i<sqi->total_slot)&&(sqi->wait_ack_slot!=0);i++)
		{
			//check wait ack slot
			if((sqi->wait_ack_slot & (1 << i)) == (1 << i))
			{
				temp_slot = (struct cpu_share_mem_slot *)(sqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
				cs_dbg("slot_status = 0x%x, slot_num = 0x%x\n", temp_slot->slot_status, temp_slot->slot_num);
				if(temp_slot->slot_status == SLOT_ACK)
				{
					//check if need to do callback function
					if((temp_slot->ack_func != NULL) && (sqi->dev_id_map & (1 << temp_slot->dev_id)) == (1 << temp_slot->dev_id))
					{
						cs_dbg("Callback function\n");
						temp_slot->ack_func((void *)temp_slot);
					}

					//update queue statsu
					temp_slot->slot_status = SLOT_FREE;
					set_slot_free(sqi, i);
					cs_dbg("Finish slot %d send\n", i);
				}
			}
		}

		//unlock queue info
		spin_unlock_irqrestore(&(sqi->queue_info_lock), flags);
	}

	//check recv queue
	for(channel=0;channel<MAX_RECV_QUEUE;channel++)
	{
		if(csm_info->recv_queue_info[channel].init_done != 1)
		{
			continue;
		}else{
			rqi = &(csm_info->recv_queue_info[channel]);
			cs_dbg("Check recv channel %d\n", channel);
		}

		//lock queue info
		spin_lock_irqsave(&(rqi->queue_info_lock), flags);

		for(i=0;i<rqi->total_slot;i++)
		{
			temp_slot = (struct cpu_share_mem_slot *)(rqi->slot_v_st_addr + (i * sizeof(struct cpu_share_mem_slot)));
			if(temp_slot->slot_status == SLOT_SEND)
			{
				if(rqi->dev_id_map & (1 << temp_slot->dev_id) & rqi->dev_id_no_ack) //device id don't need ack
				{
					if(rqi->no_ack_func[temp_slot->dev_id] != NULL)
					{
						cs_dbg("No Ack callback function device id = 0x%x\n", temp_slot->dev_id);
						rqi->no_ack_func[temp_slot->dev_id]((void *)temp_slot);
					}
					temp_slot->slot_status = SLOT_FREE; //just free
				}
				else if(rqi->dev_id_map & (1 << temp_slot->dev_id)) //device id need ack
				{
					if(rqi->recv_q_slot[temp_slot->dev_id] == 0)
					{
						//queue is empty and need to do up
						temp_slot->slot_status = SLOT_QUEUE;
						rqi->recv_q_slot[temp_slot->dev_id] = rqi->recv_q_slot[temp_slot->dev_id] | (1 << i);
						up(&(rqi->dev_sem[temp_slot->dev_id]));
						cs_dbg("Add slot(0x%x) into 0x%x queue and up\n", temp_slot->slot_num, temp_slot->dev_id);
					}else{
						//queue is not empty
						temp_slot->slot_status = SLOT_QUEUE;
						rqi->recv_q_slot[temp_slot->dev_id] = rqi->recv_q_slot[temp_slot->dev_id] | (1 << i);
						cs_dbg("Add slot(0x%x) into 0x%x queue\n", temp_slot->slot_num, temp_slot->dev_id);
					}
				}else{ //Error
					cs_dbg("Recv one slot and dev_id (0x%x) is not enable. Ack with error\n", temp_slot->dev_id);
					temp_slot->recv_parm[0] = ACK_ID_DISABLE;
					temp_slot->slot_status = SLOT_ACK;
					triger_cpux_int();
				}
			}
		}

		//unlock queue info
		spin_unlock_irqrestore(&(rqi->queue_info_lock), flags);
	}

	//clear interrupt
	//clear_cpux_int();

        return IRQ_HANDLED;
}

static int cpu_share_mem_probe(struct platform_device *pdev)
{
	struct resource *r;
	int irq, ret;

	cs_dbg("%s +++\n", __func__);

	csm_info = kzalloc(sizeof(struct _cpu_share_mem_info), GFP_KERNEL);
	if(csm_info == NULL)
	{
		printk("ERROR: Can't get csm_info memory\n");
		return -ENOMEM;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL)
	{
		printk("ERROR: Can't get cpucomm_hint IO mem resource\n");
		return -ENODEV;
	}

	r = request_mem_region(r->start, resource_size(r), "cpucomm_hint");
	if(r == NULL)
	{
		printk("ERROR: Can't request CPU mem resource(cpucomm_hint)\n");
		return -EBUSY;
	}

	csm_info->baseaddr_hint = ioremap(r->start, resource_size(r));
	if(csm_info->baseaddr_hint == NULL)
	{
		printk("ERROR: Can't remap io resource(cpucomm_hint)\n");
		return -ENOMEM;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if(r == NULL)
	{
		printk("ERROR: Can't get cpucomm_share_reg IO mem resource\n");
		return -ENODEV;
	}

	r = request_mem_region(r->start, resource_size(r), "cpucomm_share_reg");
	if(r == NULL)
	{
		printk("ERROR: Can't request CPU mem resource(cpucomm_share_reg)\n");
		return -EBUSY;
	}

	csm_info->baseaddr_share = ioremap(r->start, resource_size(r));
	if(csm_info->baseaddr_share == NULL)
	{
		printk("ERROR: Can't remap io resource(cpucomm_share_reg)\n");
		return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if(irq < 0)
	{
		printk("ERROR: Can'get cpucomm irq\n");
		return -ENODEV;
	}

	ret = request_irq(irq, cpu_share_mem_irq, 0, DEVICE_NAME_CPUCOMM, csm_info);
	if(ret != 0)
	{
		printk("ERROR: Can't request irq = %d\n", irq);
		return -EIO;
	}

	cpu_sharemem = proc_mkdir("cpu_sharemem", NULL);
	if(cpu_sharemem == NULL)
	{
		printk("ERROR: Can't create /proc/cpu_sharemem\n");
	}else{
		proc_queue_info = proc_create("queue_info", 0666, cpu_sharemem, &proc_q_ops);
		if(proc_queue_info == NULL)
		{
			printk("ERROR: Can't create send_queue\n");
		}
		proc_key_info = proc_create("key_info", 0666, cpu_sharemem, &proc_key_ops);
		if(proc_key_info == NULL)
		{
			printk("ERROR: Can't create proc_key_info\n");
		}
		fb_dump = proc_create("fb_dump", 0666, cpu_sharemem, &proc_fb_ops);
		if(fb_dump == NULL)
		{
			printk("ERROR: Can't create proc_fb_dump\n");
		}
		
		
	}

	return 0;
}

static int cpu_share_mem_remove(struct platform_device *dev)
{
	
	return 0;
}

static struct platform_driver cpu_share_mem_driver = {
	.probe		= cpu_share_mem_probe,
	.remove		= cpu_share_mem_remove,
//	.suspend	= cpu_share_mem_suspend,
//	.resume		= cpu_share_mem_resume,
	.driver		= {
		.name	= DEVICE_NAME_CPUCOMM,
		.owner	= THIS_MODULE,
	},
};

static int __init cpu_share_mem_init(void)
{
	return platform_driver_register(&cpu_share_mem_driver);
}

static void __exit cpu_share_mem_exit(void)
{
	platform_driver_unregister(&cpu_share_mem_driver);
}

/* We must initialize early, because some subsystems register cpucomm drivers
 * in subsys_initcall() code, but are linked (and initialized) before cpucomm.
 */
postcore_initcall(cpu_share_mem_init);
module_exit(cpu_share_mem_exit);
