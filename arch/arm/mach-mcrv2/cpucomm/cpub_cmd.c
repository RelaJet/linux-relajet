#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <mach/mmpf_uart.h>
#include <mach/cpucomm/cpucomm_id.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm_cmd.h>
#include <mach/cpucomm/cpucomm_file.h>

static struct proc_dir_entry *proc_dual_cpu_cmd_info;
static struct task_struct *cpu_cmd_tsk;
struct semaphore usr_cmd_sema;
static LIST_HEAD(usr_cmd_list);		//command queue in kerenl and wait user to receive
static LIST_HEAD(usr_ack_list); 	//command queue in kernel and wait user to finish command 
static wait_queue_head_t cpu_cmd_event_wait;
static char *rd_v_addr, *wr_v_addr; 
static int rd_p_addr, wr_p_addr;

ssize_t dual_cpu_cmd_r(struct file *filp, char __user *data, size_t count, loff_t *off)
{
	return count;
}

ssize_t dual_cpu_cmd_w(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	return count;
}

unsigned int dual_cpu_cmd_p(struct file *filp, struct poll_table_struct *wait)
{
	int ret = 0;

	down(&usr_cmd_sema);
	ccmd_dbg("%s \n", __func__);

	if(list_empty(&usr_cmd_list) == 1)
	{
		up(&usr_cmd_sema);
		poll_wait(filp, &cpu_cmd_event_wait, wait);
		down(&usr_cmd_sema);
		if(list_empty(&usr_cmd_list) != 1)
		{
			ret |= POLLIN | POLLRDNORM;
		}
	}
	else
	{
		ret |= POLLIN | POLLRDNORM;
	}
	
	up(&usr_cmd_sema);

	return ret;
}

long dual_cpu_cmd_ctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct usr_cmd_f *u;
	struct cpu_share_mem_slot wr_slot;
	struct list_head *p, *n;
	int rw_size;

	ccmd_dbg("cmd = 0x%x\n", cmd);
	switch(cmd)
	{
	case CPU_CMD_RD:
		down(&usr_cmd_sema);
		if(list_empty(&usr_cmd_list) != 1)
		{
			u = list_first_entry(&usr_cmd_list, struct usr_cmd_f, list);
			if (copy_to_user((void __user *)arg, u->cmd_slot, sizeof(struct cpu_share_mem_slot)))
			{
				ret = -EFAULT;
			}
			else
			{
				list_move_tail(&u->list, &usr_ack_list);
			}
		}
		else
		{
			ret = -ENODATA;
		}
		up(&usr_cmd_sema);
		break;
	case CPU_CMD_WR:
		down(&usr_cmd_sema);
		if(copy_from_user(&wr_slot, (void __user *)arg, sizeof(struct cpu_share_mem_slot)))
		{
			ret = -EFAULT;
		}
		else
		{
			ret = -EFAULT;
			list_for_each_safe(p, n, &usr_ack_list)
			{
				u = list_entry(p, struct usr_cmd_f, list);
				if((u->cmd_slot->seq == wr_slot.seq) && (u->cmd_slot->slot_num == wr_slot.slot_num))
				{
					list_del(&u->list);
					memcpy(u->cmd_slot->send_parm, wr_slot.send_parm, sizeof(int)*4); //send_parm[4]
					memcpy(u->cmd_slot->recv_parm, wr_slot.recv_parm, sizeof(int)*4); //recv_parm[4]
					ack_slot(CPU_CMD_CHN, u->cmd_slot);
					vfree(u);
					ret = 0;
				}
			}
		}
		up(&usr_cmd_sema);
		break;
	case CPU_CMD_F_RD:
		if(copy_from_user(&rw_size, (void __user *)&(((struct _usr_cmd_f_rw *)arg)->size), sizeof(int)))
		{
			ret = -EFAULT;
		}
		else
		{
			ccmd_dbg("read size = %d\n", rw_size);
			if(copy_from_user(rd_v_addr, (void __user *)(((struct _usr_cmd_f_rw *)arg)->buf), rw_size))
			{
				ret = -EFAULT;
			}
		}
		break;
	case CPU_CMD_F_WR:
		if(copy_from_user(&rw_size, (void __user *)&(((struct _usr_cmd_f_rw *)arg)->size), sizeof(int)))
		{
			ret = -EFAULT;
		}
		else
		{
			ccmd_dbg("write size = %d\n", rw_size);
			if(copy_to_user((void __user *)(((struct _usr_cmd_f_rw *)arg)->buf), wr_v_addr, rw_size))
			{
				ret = -EFAULT;
			}
		}
		break;
	default:
		printk("Unsupport ioctl\n");
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int do_user_cmd(struct cpu_share_mem_slot *cmd_slot)
{
	int cmd  = cmd_slot->command;
	struct usr_cmd_f *new_cmd;
	ccmd_dbg("%s cmd = 0x%x\n", __func__, cmd);

	//new_cmd = kmalloc(sizeof(struct usr_cmd_f), GFP_KERNEL);
	new_cmd = vmalloc(sizeof(struct usr_cmd_f));
	if(new_cmd == NULL)
	{
		printk("Can't alloc memory for cpu user command!!!\n");
		ack_slot(CPU_CMD_CHN, cmd_slot);
	}

	new_cmd->cmd_slot = cmd_slot;

	down(&usr_cmd_sema);

	if(list_empty(&usr_cmd_list) == 1)
	{
		wake_up_interruptible(&cpu_cmd_event_wait);
	}

	list_add_tail(&new_cmd->list, &usr_cmd_list);

	up(&usr_cmd_sema);

	return 0;
}

static int do_kernel_cmd(struct cpu_share_mem_slot *cmd_slot)
{
	int i, cmd = cmd_slot->command, uart_num;
	ccmd_dbg("%s cmd = 0x%x\n", __func__, cmd);

	if(cmd >= CPU_KNL_CMD_END)
	{
		return -1;
	}

	switch(cmd)
	{
	case CPU_KNL_CMD_OK:
		break;
	case CPU_KNL_CMD_ERROR:
		break;
	case CPU_KNL_CMD_ECHO:
		break;
	case CPU_KNL_CMD_EN_UART_RX:
		uart_num = cmd_slot->send_parm[0];
		for(i=0;i<MMPF_UART_MAX_COUNT;i++)
		{
			if((uart_num >> i) & 0x01)
			{
				MMPF_Uart_EnableRx(i);
			}
		}
		enable_irq(AIC_SRC_UART);
		break;
	case CPU_KNL_CMD_DIS_UART_RX:
		uart_num = cmd_slot->send_parm[0];
		for(i=0;i<MMPF_UART_MAX_COUNT;i++)
		{
			if((uart_num >> i) & 0x01)
			{
				MMPF_Uart_DisableRx(i);
			}
		}
		disable_irq(AIC_SRC_UART);
		break;
	case CPU_KNL_CMD_SETUP_FILE:
		if((rd_v_addr == NULL) && (wr_v_addr == NULL))
		{
			rd_v_addr = dma_alloc_coherent(0, SZ_4K, &rd_p_addr, GFP_KERNEL);
			if(rd_v_addr == NULL)
			{
				printk("Can't alloc memory for CPU_KNL_CMD_SETUP_FILE!!\n");
				cmd_slot->recv_parm[0] = -1;
				cmd_slot->recv_parm[1] = -1;
				cmd_slot->recv_parm[3] = 0xdeadbeef;
			}
			else
			{
				wr_v_addr = rd_v_addr + SZ_2K;
				wr_p_addr = rd_p_addr + SZ_2K;
				cmd_slot->recv_parm[0] = rd_p_addr; 
				cmd_slot->recv_parm[1] = wr_p_addr;
				cmd_slot->recv_parm[3] = 0;
			}
		}
		else
		{
			cmd_slot->recv_parm[0] = rd_p_addr; 
			cmd_slot->recv_parm[1] = wr_p_addr;
			cmd_slot->recv_parm[3] = 0;
		}
		printk("CPUB FILE rd_p_addr = 0x%x, wr_p_addr = 0x%x\n", rd_p_addr, wr_p_addr);
		break;
	case CPU_KNL_CMD_FREE_FILE:
		break;
	default:
		break;
	}

	return 0;
}

int cpu_cmd_thd(void *arg)
{
	int ret = 0;
	struct cpu_share_mem_slot *cmd_slot;

	ret = init_send_dev_id(CPU_CMD_CHN, CPU_COMM_ID_CMD);
	if(ret != 0)
	{
		printk("Can't initiate send ID (%d)\n", CPU_COMM_ID_CMD);
	}

	ret = init_recv_dev_id(CPU_CMD_CHN, CPU_COMM_ID_CMD);
	if(ret != 0)
	{
		printk("Can't initiate recv ID (%d)\n", CPU_COMM_ID_CMD);
	}

	sema_init(&usr_cmd_sema, 1);
	init_waitqueue_head(&cpu_cmd_event_wait);

	while(1)
	{
		if(recv_slot(CPU_CMD_CHN, CPU_COMM_ID_CMD, &cmd_slot, 0) == 0)
		{
			if((cmd_slot->command & KNL_CMD) == KNL_CMD)
			{
				if(do_kernel_cmd(cmd_slot) != 0)
				{
					printk("Do kernel cmd fail!!\n");
				}
				ack_slot(CPU_CMD_CHN, cmd_slot);
			}
			else
			{
				if(do_user_cmd(cmd_slot) != 0)
				{
					printk("Do user cmd fail!!\n");
				}
			}
		}
		else
		{
			printk("CPU Command recv_slot error\n");
		}
	}
}

struct file_operations dual_cpu_cmd_ops =
{
	.read			= dual_cpu_cmd_r,
	.write			= dual_cpu_cmd_w,
	.poll			= dual_cpu_cmd_p,
	.unlocked_ioctl = dual_cpu_cmd_ctl,
    .owner			= THIS_MODULE,
};

int start_cpu_cmd_thd(void)
{

	cpu_cmd_tsk = kthread_create(cpu_cmd_thd, NULL, "CPU_CMD_TSK"); 
	if (IS_ERR(cpu_cmd_tsk))
	{
		cpu_cmd_tsk = NULL;
		return PTR_ERR(cpu_cmd_tsk);
	}

	wake_up_process(cpu_cmd_tsk);

	proc_dual_cpu_cmd_info = proc_create("cpu_sharemem/dual_cpu_cmd", 0666, NULL, &dual_cpu_cmd_ops);
	if(proc_dual_cpu_cmd_info == NULL)
	{
		printk("ERROR: Can't create dual_cpu_cmd\n");
	}
	return 0;
}
EXPORT_SYMBOL(start_cpu_cmd_thd);

void stop_cpu_cmd_thd(void)
{
	kthread_stop(cpu_cmd_tsk);
}
EXPORT_SYMBOL(stop_cpu_cmd_thd);
