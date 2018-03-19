#ifndef __I2CS_H__
#define __I2CS_H__

struct _i2cs_event
{
	unsigned short addr;
	unsigned short size;
	unsigned char  reserved[12];
};

struct _i2cs_data
{
	wait_queue_head_t event_wait;
	struct _i2cs_event i2cs_event;
	struct semaphore sem;
	int event;
};

#define I2CS_REG_SIZE 0x10
#define I2CS_IOCTL_BASE 'I'

#define I2CS_GETMEMSIZE _IOR(I2CS_IOCTL_BASE, 0, unsigned long)
#define I2CS_GETEVENT _IOR(I2CS_IOCTL_BASE, 1, struct _i2cs_event)


#endif
