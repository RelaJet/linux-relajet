#ifndef __CPU_COMM_CMD_H__
#define __CPU_COMM_CMD_H__

#include <linux/poll.h>
#include <linux/list.h>
#include <mach/ait_cpu_sharemem.h>

#define CPU_CMD_DEBUG
#ifdef CPU_CMD_DEBUG
#define ccmd_dbg(format, arg...) printk( format, ##arg)
#else
#define ccmd_dbg(format, arg...)
#endif

#define CPU_CMD_CHN		0
#define USR_CMD			0x00000000
#define KNL_CMD			0x80000000

#define CPU_CMD_IOCTL_BASE 'C'
#define CPU_CMD_RD _IOR(CPU_CMD_IOCTL_BASE, 0, struct cpu_share_mem_slot)
#define CPU_CMD_WR _IOW(CPU_CMD_IOCTL_BASE, 1, struct cpu_share_mem_slot)
#define CPU_CMD_F_RD _IOWR(CPU_CMD_IOCTL_BASE, 2, struct _usr_cmd_f_rw)
#define CPU_CMD_F_WR _IOWR(CPU_CMD_IOCTL_BASE, 3, struct _usr_cmd_f_rw)

struct usr_cmd_f
{
	struct list_head list;
	struct cpu_share_mem_slot *cmd_slot;
};

typedef enum _CPU_KNL_CMD
{
	CPU_KNL_CMD_START = KNL_CMD,
	CPU_KNL_CMD_OK,
	CPU_KNL_CMD_ERROR,
	CPU_KNL_CMD_ECHO,
	CPU_KNL_CMD_EN_UART_RX,
	CPU_KNL_CMD_DIS_UART_RX,
	CPU_KNL_CMD_SETUP_FILE,
	CPU_KNL_CMD_FREE_FILE,

	CPU_KNL_CMD_END,
}CPU_KNL_CMD;

int start_cpu_cmd_thd(void);
void stop_cpu_cmd_thd(void);
#endif
