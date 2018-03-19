/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef __AIT_CPU_SHAREMEM_H__
#define __AIT_CPU_SHAREMEM_H__

#include <linux/semaphore.h>
#include <asm/sizes.h>
//#include <mach/mmp_reg_int.h>
//#include <mach/mmp_cpucomm.h>

#define	DEVICE_NAME_CPUCOMM	"cpucomm"
#define MAX_CPU_QUEUE		2
#define	MAX_SEND_QUEUE		MAX_CPU_QUEUE
#define	MAX_RECV_QUEUE		MAX_CPU_QUEUE
#define MAX_SLOT		32
#define MAX_DEV_ID		64

enum Slot_block
{
	SEND_SLOT_BLOCK_MODE	= 0,
	SEND_SLOT_NONBLOCK_MODE	= 1,
};

enum Slot_direct
{
	L_TO_R_SLOT	= (1 << 0),	//Linux to RTOS
	R_TO_L_SLOT	= (1 << 1),	//RTOS to Linux
	A_TO_B_SLOT	= (1 << 2),	//CPUA tO CPUB 
	B_TO_A_SLOT	= (1 << 3),	//CPUB to CPUA
};

enum Slot_status
{
	SLOT_FREE	= 0,
	SLOT_GET	= 1,
	SLOT_SEND	= 2,
	SLOT_QUEUE	= 3,
	SLOT_PROC	= 4,
	SLOT_ACK	= 5,

	SLOT_ERROR	= 0xFF,
};

enum Slot_ack_status
{
	ACK_CMD_OK	= 0,
	ACK_CMD_ERROR	= 1,
	ACK_ID_DISABLE	= 2,
	ACK_ID_CLOSE	= 3,
};

struct _queue_info {
	unsigned int slot_v_st_addr;
	unsigned int slot_p_st_addr;
	unsigned int total_slot;
	unsigned int free_slot;			//free slot bitmap 1:free 0:used 
	unsigned int used_slot;			//used slot bitmap 0:free 1:used.
	unsigned int wait_ack_slot;		//wait ack slot bitmap 0:not wait 1:wait
	unsigned int recv_q_slot[MAX_DEV_ID];	//the bitmap indicate the slot need to process for each device ID.
	unsigned int recv_q_proc[MAX_DEV_ID];	//the bitmap indicate the slot need to process for each device ID.
	unsigned long long dev_id_map;		//active device id bitmap,
	unsigned long long dev_id_no_ack;	//enable slot to work like UDP
	spinlock_t queue_info_lock;
	struct semaphore dev_sem[MAX_DEV_ID];
	unsigned int (*no_ack_func[MAX_DEV_ID])(void *slot);
	unsigned short init_done;
	unsigned short seq;
};

struct cpu_share_mem_slot {
	unsigned int dev_id;
	unsigned int command;
	unsigned int data_phy_addr;
	unsigned int size;
	unsigned int hdr_phy_addr;
	unsigned int (*ack_func)(void *slot);
	unsigned short seq;
	unsigned short slot_direct;
	unsigned short slot_num;
	unsigned short slot_status;
//32bytes
	unsigned int send_parm[4];
	unsigned int recv_parm[4];
//32bytes
};

struct _cpu_share_mem_info {
	struct _queue_info send_queue_info[MAX_SEND_QUEUE];
	struct _queue_info recv_queue_info[MAX_RECV_QUEUE];
	void __iomem *baseaddr_hint;
	void __iomem *baseaddr_share;
};

// provide to linux to construct / reserve rtos-working space
struct _cpu_rtos_mem_info {
    unsigned int   reset_base   ;
    unsigned int   v4l2_base    ;
    unsigned char  v4l2_size_mb ;   // v4l2 size    
    unsigned char  working_size_mb; // rtos working size, not include v4l2    
    unsigned char  noused[2] ;
    unsigned int   osd_base2rtos;
} ;

unsigned int get_share_register(void);
//send function
int init_send_queue(unsigned int channel, unsigned int slot_v_st_addr, unsigned int slot_p_st_addr, unsigned int total_slot);
int exit_send_queue(unsigned int channel);
int init_send_dev_id(unsigned int channel, unsigned int dev_id);
int enable_send_dev_id_noack(unsigned int channel, unsigned int dev_id);
int close_send_dev_id(unsigned int channel, unsigned int dev_id);
int get_slot(unsigned int channel, struct cpu_share_mem_slot **slot);
int send_slot(unsigned int channel, struct cpu_share_mem_slot *slot); 

//recv function
int init_recv_queue(unsigned int channel, unsigned int slot_v_st_addr, unsigned int slot_p_st_addr, unsigned int total_slot);
int exit_recv_queue(unsigned int channel);
int init_recv_dev_id(unsigned int channel, unsigned int dev_id);
int enable_recv_dev_id_noack(unsigned int channel, unsigned int dev_id, unsigned int (*no_ack_func)(void *slot));
int close_recv_dev_id(unsigned int channel, unsigned int dev_id);
int recv_slot(unsigned int channel, unsigned int dev_id, struct cpu_share_mem_slot **slot, unsigned int timeout);
int ack_slot(unsigned channel, struct cpu_share_mem_slot *slot);

//global init function
int init_cpu_share_mem(int max_chn);
int exit_cpu_share_mem(int max_chn);
void wait_cpub_ready(int max_chn);

// to know rtos fw work & v4l2 size
struct _cpu_rtos_mem_info *get_cpub_rtos_mem(void);

#endif
