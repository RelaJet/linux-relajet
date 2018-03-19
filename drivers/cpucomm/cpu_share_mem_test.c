/*
 *   Copyright (C) 2014
 *
 *      This program is free software; you can redistribute it and/or modify
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
#include <linux/platform_device.h>
#include <linux/kthread.h>

#include <mach/ait_cpu_sharemem.h>
#include <mach/mmp_cpucomm.h>
#include <mach/cpucomm/cpucomm_api.h>
#include <mach/cpucomm/cpucomm_if.h>

//#define TEST_NO_ACK
#define SEND_CHN	0
#define SEND1_CHN	0
#define SEND_DEV_ID	1
#define SEND1_DEV_ID	10
#define RECV_DEV_ID	2
#define RECV1_DEV_ID	11

static struct task_struct *cm_test_R_tsk, *cm_test_S_tsk, *cm_test_S1_tsk, *cm_test_R1_tsk;
struct semaphore send_semaphore;
struct semaphore send1_semaphore;
struct semaphore recv_semaphore;
struct semaphore recv1_semaphore;

unsigned int send_ack_up(void *slot)
{
	up(&send_semaphore);
	return 0;
}

unsigned int send1_ack_up(void *slot)
{
	up(&send1_semaphore);
	return 0;
}

unsigned int recv_ack_up(void *slot)
{
	up(&recv_semaphore);
	return 0;
}

unsigned int recv1_ack_up(void *slot)
{
	up(&recv1_semaphore);
	return 0;
}

static int cs_test_S(void *parm)
{
	struct cpu_share_mem_slot *S_slot;

	init_send_dev_id(SEND_CHN, SEND_DEV_ID);
	sema_init(&send_semaphore, 0);

	printk("start_send_test\n");
	while(1)
	{
		if(kthread_should_stop())
		{
			break;
		}

		if(get_slot(SEND_CHN, &S_slot) == 0)
		{
			S_slot->dev_id = SEND_DEV_ID;
			S_slot->command = 24;
			S_slot->ack_func = send_ack_up;
			send_slot(SEND_CHN, S_slot);
			down(&send_semaphore);
		}else{
			printk("%s get slot fail\n", __func__);
		}
	}

	close_send_dev_id(SEND_CHN, SEND_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}

#ifdef TEST_NO_ACK
static int cs_test_S1(void *parm)
{
	struct cpu_share_mem_slot *S_slot;

	init_send_dev_id(SEND1_CHN, SEND1_DEV_ID);
	enable_send_dev_id_noack(SEND1_CHN, SEND1_DEV_ID);

	printk("start_send1_test\n");
	while(1)
	{
		if(kthread_should_stop())
		{
			break;
		}

		if(get_slot(SEND1_CHN, &S_slot) == 0)
		{
			S_slot->dev_id = SEND1_DEV_ID;
			S_slot->command = 24;
			send_slot(SEND1_CHN, S_slot);
			msleep(30);
		}else{
			printk("%s get slot fail\n", __func__);
		}
	}

	close_send_dev_id(SEND1_CHN, SEND1_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}
#else
static int cs_test_S1(void *parm)
{
	struct cpu_share_mem_slot *S_slot;

	init_send_dev_id(SEND1_CHN, SEND1_DEV_ID);
	sema_init(&send1_semaphore, 0);

	printk("start_send1_test\n");
	while(1)
	{
		if(kthread_should_stop())
		{
			break;
		}

		if(get_slot(SEND1_CHN, &S_slot) == 0)
		{
			S_slot->dev_id = SEND1_DEV_ID;
			S_slot->command = 24;
			S_slot->ack_func = send1_ack_up;
			send_slot(SEND1_CHN, S_slot);
			down(&send1_semaphore);
		}else{
			printk("%s get slot fail\n", __func__);
		}
	}

	close_send_dev_id(SEND1_CHN, SEND1_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}
#endif

static int cs_test_R(void *parm)
{
	int i=0,j=0;
	struct cpu_share_mem_slot *S_slot, *R_slot;

        init_send_dev_id(SEND_CHN, RECV_DEV_ID);
	init_recv_dev_id(SEND_CHN, RECV_DEV_ID);
	sema_init(&recv_semaphore, 0);

	while(1)
	{
		if(kthread_should_stop())
		{
			break;
		}

		if(j==0)
		{
			if(get_slot(SEND_CHN, &S_slot) == 0)
			{
				S_slot->dev_id = RECV_DEV_ID;
				S_slot->command = 24;
				S_slot->ack_func = recv_ack_up;
				send_slot(SEND_CHN, S_slot);
				down(&recv_semaphore);
			}else{
				return 0;
			}
			printk("start_recv_test\n");
		}

		printk("start recv j=%d\n", j);

		while(i<4096)
		{
			if(recv_slot(SEND_CHN, RECV_DEV_ID, &R_slot, 0) == 0)
			{
				R_slot->recv_parm[0] = ACK_CMD_OK;
				ack_slot(SEND_CHN, R_slot);
				i++;
			}else{
				printk("recv_slot error, i = %d\n", i);
			}
        	}
		printk("R_Finish\n");

		printk("wait ...\n");
		j++;
		msleep(1000);
		i=0;
	}

        close_send_dev_id(SEND_CHN, RECV_DEV_ID);
	close_recv_dev_id(SEND_CHN, RECV_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}

#ifdef TEST_NO_ACK
unsigned int test_no_ack(void *slot)
{
	struct cpu_share_mem_slot *R_slot = (struct cpu_share_mem_slot *)slot;
	printk("slot->slot_num = %d slot->dev_id = 0x%x slot->slot_status = 0x%x\r\n", R_slot->slot_num, R_slot->dev_id, R_slot->slot_status);
	return 0;
}
static int cs_test_R1(void *parm)
{
	struct cpu_share_mem_slot *S_slot;
        init_send_dev_id(SEND1_CHN, RECV1_DEV_ID);
	init_recv_dev_id(SEND1_CHN, RECV1_DEV_ID);
	enable_recv_dev_id_noack(SEND1_CHN, RECV1_DEV_ID, test_no_ack);
	sema_init(&recv1_semaphore, 0);

	if(get_slot(SEND1_CHN, &S_slot) == 0)
	{
		S_slot->dev_id = RECV1_DEV_ID;
		S_slot->command = 24;
		S_slot->ack_func = recv1_ack_up;
		send_slot(SEND1_CHN, S_slot);
		down(&recv1_semaphore);
	}else{
		return 0;
	}
	printk("start_recv_test\n");

	while(1)
	{
		msleep(5000);
	}

        close_send_dev_id(SEND1_CHN, RECV1_DEV_ID);
	close_recv_dev_id(SEND1_CHN, RECV1_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}
#else
static int cs_test_R1(void *parm)
{
	int i=0,j=0;
	struct cpu_share_mem_slot *S_slot, *R_slot;

        init_send_dev_id(SEND1_CHN, RECV1_DEV_ID);
	init_recv_dev_id(SEND1_CHN, RECV1_DEV_ID);
	sema_init(&recv1_semaphore, 0);

	while(1)
	{
		if(kthread_should_stop())
		{
			break;
		}

		if(j==0)
		{
			if(get_slot(SEND1_CHN, &S_slot) == 0)
			{
				S_slot->dev_id = RECV1_DEV_ID;
				S_slot->command = 24;
				S_slot->ack_func = recv1_ack_up;
				send_slot(SEND1_CHN, S_slot);
				down(&recv1_semaphore);
			}else{
				return 0;
			}
			printk("start_recv_test\n");
		}

		printk("start recv j=%d\n", j);

		while(i<4096)
		{
			if(recv_slot(SEND1_CHN, RECV1_DEV_ID, &R_slot, 0) == 0)
			{
				R_slot->recv_parm[0] = ACK_CMD_OK;
				ack_slot(SEND1_CHN, R_slot);
				i++;
			}else{
				printk("recv_slot error, i = %d\n", i);
			}
        	}
		printk("R1_Finish\n");

		printk("wait ...\n");
		j++;
		msleep(1000);
		i=0;
	}

        close_send_dev_id(SEND1_CHN, RECV1_DEV_ID);
	close_recv_dev_id(SEND1_CHN, RECV1_DEV_ID);

	printk("%s exit\n", __func__);
	return 0;
}
#endif

static int __init cpu_share_mem_test_init(void)
{
	int ret = 0;

	cm_test_S_tsk = kthread_create(cs_test_S, NULL, "CS_TEST_S");
	if(IS_ERR(cm_test_S_tsk))
	{
		ret = PTR_ERR(cm_test_S_tsk);
		cm_test_S_tsk = NULL;
		return ret;
	} 

	wake_up_process(cm_test_S_tsk);

	cm_test_S1_tsk = kthread_create(cs_test_S1, NULL, "CS_TEST_S1");
	if(IS_ERR(cm_test_S1_tsk))
	{
		ret = PTR_ERR(cm_test_S1_tsk);
		cm_test_S1_tsk = NULL;
		return ret;
	}

	wake_up_process(cm_test_S1_tsk);

	cm_test_R_tsk = kthread_create(cs_test_R, NULL, "CS_TEST_R");
	if(IS_ERR(cm_test_R_tsk))
	{
		ret = PTR_ERR(cm_test_R_tsk);
		cm_test_R_tsk = NULL;
		return ret;
	} 

	wake_up_process(cm_test_R_tsk);

	cm_test_R1_tsk = kthread_create(cs_test_R1, NULL, "CS_TEST_R1");
	if(IS_ERR(cm_test_R1_tsk))
	{
		ret = PTR_ERR(cm_test_R1_tsk);
		cm_test_R1_tsk = NULL;
		return ret;
	} 

	wake_up_process(cm_test_R1_tsk);

	return 0;
}

static void __exit cpu_share_mem_test_exit(void)
{
	kthread_stop(cm_test_R_tsk);
	kthread_stop(cm_test_S_tsk);
	kthread_stop(cm_test_S1_tsk);
	kthread_stop(cm_test_R1_tsk);
}

module_init(cpu_share_mem_test_init);
module_exit(cpu_share_mem_test_exit);

MODULE_LICENSE("GPL");
