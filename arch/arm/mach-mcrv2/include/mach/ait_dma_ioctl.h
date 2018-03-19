#ifndef _DEVONE_IOCTL_H
#define _DEVONE_IOCTL_H

#include <linux/ioctl.h>
struct ioctl_arg {
	unsigned int reg;
	unsigned int offset;
	unsigned int val;

	void* src_addr;
	void* dest_addr;
	unsigned int size;

};

struct aitdma_mem{
	unsigned long kvirt_addr;
	unsigned long kphy_addr;
	unsigned long size;
};

struct aitdma_memcpy{
	unsigned long kphy_src;
	unsigned long kphy_dest;	
	unsigned long size;
	int timeout;
};

#define IOC_MAGIC 0xEE

#define IOCTL_AITDMA_SET _IOW(IOC_MAGIC, 1, struct ioctl_arg)
#define IOCTL_AITDMA_GET _IOR(IOC_MAGIC, 2, struct ioctl_arg)
#define IOCTL_VALGET_NUM  _IOR(IOC_MAGIC, 3, int)
#define IOCTL_AITDMA_SETSRC _IOW(IOC_MAGIC, 4, int)
#define IOCTL_AITDMA_SETDEST _IOW(IOC_MAGIC, 5, int)
#define IOCTL_AITDMA_START _IOW(IOC_MAGIC, 6, struct aitdma_memcpy)
#define IOCTL_AITDMA_MALLOC _IOW(IOC_MAGIC, 7, struct aitdma_mem)
#define IOCTL_AITDMA_FREEMEM _IOW(IOC_MAGIC, 8, struct aitdma_mem)
#define IOCTL_VALSET_NUM  _IOW(IOC_MAGIC, 9, int)

#define IOCTL_VAL_MAXNR 7

#endif
