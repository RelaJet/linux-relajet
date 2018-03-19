#ifndef __CPU_COMM_FILE_H__
#define __CPU_COMM_FILE_H__

#include <linux/poll.h>
#include <linux/list.h>

struct CPUB_FILE
{
	struct list_head list;	
    unsigned int rd_v_addr;
    unsigned int wr_v_addr;
    int fd;
};

struct _usr_cmd_f_rw
{
    int size;
    char buf[SZ_2K];
};

#endif
