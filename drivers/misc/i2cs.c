/*
 * Dallas Semiconductor DS1682 Elapsed Time Recorder device driver
 *
 * Written by: Grant Likely <grant.likely@secretlab.ca>
 *
 * Copyright (C) 2007 Secret Lab Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * The DS1682 elapsed timer recorder is a simple device that implements
 * one elapsed time counter, one event counter, an alarm signal and 10
 * bytes of general purpose EEPROM.
 *
 * This driver provides access to the DS1682 counters and user data via
 * the sysfs.  The following attributes are added to the device node:
 *     elapsed_time (u32): Total elapsed event time in ms resolution
 *     alarm_time (u32): When elapsed time exceeds the value in alarm_time,
 *                       then the alarm pin is asserted.
 *     event_count (u16): number of times the event pin has gone low.
 *     eeprom (u8[10]): general purpose EEPROM
 *
 * Counter registers and user data are both read/write unless the device
 * has been write protected.  This driver does not support turning off write
 * protection.  Once write protection is turned on, it is impossible to
 * turn it off again, so I have left the feature out of this driver to avoid
 * accidental enabling, but it is trivial to add write protect support.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
//#include <asm/poll.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_pad.h>
#include <mach/mmp_reg_i2cs.h>
#include <mach/mmpf_system.h>
#include <mach/hardware.h>
#include "i2cs.h"

struct _i2cs_data *i2cs_data;

static irqreturn_t mcrv2_i2cs_irq(int irq, void *data)
{
	AITPS_I2CS pI2CS = AITC_BASE_I2CS;
	int *pt = AIT_I2CS_DRAM_VIRT_BASE;

	if(*pt != 0)
	{
		//printk("event = 0x%x\n", *pt);
		i2cs_data->i2cs_event.addr = (*pt) & 0xFFFF;
		i2cs_data->i2cs_event.size = ((*pt) >> 16) & 0xFFFF;
		//clear event
		*pt = 0;
		i2cs_data->event = 1;
		wake_up_interruptible(&i2cs_data->event_wait);
	}

	pI2CS->I2CS_INT_CPU_SR = I2CS_DET_STOP_BIT;

	return IRQ_HANDLED;
}

static long i2cs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int i2csize = AIT_I2CS_DRAM_SIZE, ret = 0;
	switch (cmd)
	{
	case I2CS_GETMEMSIZE:
		if (copy_to_user((unsigned long __user *)arg, &i2csize, sizeof(unsigned long)))
		{
			ret = -EFAULT;
		}
		break;
	case I2CS_GETEVENT:
		if(i2cs_data->event == 1)
		{
			if (copy_to_user((unsigned long __user *)arg, &i2cs_data->i2cs_event, sizeof(struct _i2cs_event)))
			{
				ret = -EFAULT;
			}
			i2cs_data->event = 0;
		}else{
			ret = -ENODATA;
		}
		break;
	default:
		printk("Unsupport ioctl\n");
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static int i2cs_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long i2cs_pa;

	if (vma->vm_end - vma->vm_start != AIT_I2CS_DRAM_SIZE)
	{
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	i2cs_pa = AIT_I2CS_DRAM_PHYS_BASE;
	i2cs_pa &= 0xfffffffffffffffUL;
	i2cs_pa = i2cs_pa >> PAGE_SHIFT;

	if (remap_pfn_range(vma, vma->vm_start, i2cs_pa, AIT_I2CS_DRAM_SIZE, vma->vm_page_prot))
	{
		printk("remap_pfn_range failed");
		return -EAGAIN;
	}

	return 0;
}

static unsigned int i2cs_poll(struct file *filp, struct poll_table_struct *wait)
{
	int ret = 0;

	if(i2cs_data == NULL)
	{
		return -EBADFD;
	}

	poll_wait(filp, &i2cs_data->event_wait, wait);

	if(i2cs_data->event == 1)
	{
		ret |= POLLIN | POLLRDNORM;
	}

	return ret;
}

static const struct file_operations i2cs_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= i2cs_ioctl,
	.mmap			= i2cs_mmap,
	.poll			= i2cs_poll,
};

static struct miscdevice i2cs_misc = {
	.fops	= &i2cs_fops,
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mcrv2_i2cs",
};

static __init int i2cs_init(void)
{
	int ret =0, mem_bank=0x20, *pt = AIT_I2CS_DRAM_VIRT_BASE, i;
    AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_I2CS pI2CS = AITC_BASE_I2CS;

	//register misc driver
	ret = misc_register(&i2cs_misc);
	if(ret != 0)
	{
		printk("ERROR: I2CS register misc driver fail. ret = 0x%x\n", ret);
		return ret;
	}

	//clear status register
	for(i=0;i<(I2CS_REG_SIZE/sizeof(int));i++)
	{
		*(pt+i) = 0;
	}

	//cal mem_bank and initial i2cs memory block
	mem_bank = AIT_I2CS_DRAM_PHYS_BASE / SZ_512K;
    pGBL->_x69F2[6] = (mem_bank & 0x0f) << 4; //0x800069f8
    pGBL->_x69F2[7] = (mem_bank >> 4) & 0x1F; //0x800069f9, 0x20 = 32 -> memory map to 0x1000000
	printk("Mem_bank = 0x%x, reg0 = 0x%x, reg1 = 0x%x\n", mem_bank, pGBL->_x69F2[6], pGBL->_x69F2[7]);

	//register interrupt
	ret = request_irq(AIC_SRC_I2S, mcrv2_i2cs_irq, 0, "I2CS_DEV", NULL);
	if(ret != 0)
	{
		printk("ERROR: I2CS error ret = 0x%x\n", ret);
	}else{
		//enable stop bit interrupt
		pI2CS->I2CS_INT_CPU_EN = I2CS_DET_STOP_BIT;
	}

	i2cs_data = kzalloc(sizeof(struct _i2cs_data), GFP_KERNEL);

	//initialize data
	init_waitqueue_head(&i2cs_data->event_wait);

	return 0;
}

static __exit void i2cs_exit(void)
{

}

MODULE_LICENSE("GPL");
late_initcall(i2cs_init);
module_exit(i2cs_exit);
