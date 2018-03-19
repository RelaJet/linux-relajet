/*
 * Driver for the AIT DMA operation
 *
 * Copyright (C) 2015 Vincent Chen
 *
 * arch/arm/mach-vsnv3/ait_dma_ioctl.c which has following copyrights:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <asm/uaccess.h>
#include <mach/ait_dma_ioctl.h>

#define DRIVER_NAME "test_ioctl"
static unsigned int test_ioctl_major = 0;
static unsigned int num_of_dev = 1;
static struct cdev test_ioctl_cdev;
static int ioctl_num = 0;

struct test_ioctl_data {
unsigned char val;
rwlock_t lock;

	void* src_addr;
	void* dest_addr;
	unsigned int size;
};
//char* src;
char* dest;

static struct dma_chan *dma_channel;

static bool filter(struct dma_chan *chan, void *param)
{
	return true;
}

static void dmatest_callback(void *completion)
{
	complete(completion);
}

static int dma_init(void)
{
	dma_cap_mask_t mask;
	int err = 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	dma_channel = dma_request_channel(mask, filter, NULL);
	
	if (!dma_channel) {
				pr_err("Dma pipe allocation error.\n");

				return -EBUSY;
	}
	return err;
}
static int dma_release(void)
{
	dma_release_channel(dma_channel);
}

enum dma_status dma_start(dma_addr_t dest, dma_addr_t src, size_t len)
{
	unsigned long start, tmo, end = 0 /* compiler... */;
	struct completion cmp;
	dma_cookie_t		cookie;
	enum dma_status		status;

	struct dma_device *dev = dma_channel->device;
	struct dma_async_tx_descriptor *tx = NULL;

	tx = dev->device_prep_dma_memcpy(dma_channel,		//ep93xx_dma_prep_dma_memcpy
					 dest,
					 src, 
					 len,
					 DMA_MEMCPY|DMA_CTRL_ACK);

	init_completion(&cmp);
		
	tx->callback = dmatest_callback;
	tx->callback_param = &cmp;
	cookie = tx->tx_submit(tx);		//ep93xx_dma_tx_submit

	if (dma_submit_error(cookie)) {
		pr_warning("DMA submit error %d with src=0x%x "
				"dst=0x%x len=0x%x\n",
				cookie,
				src, dest, len);

	}

	start = jiffies;
	
	end = start + msecs_to_jiffies(1000);
	tmo = wait_for_completion_interruptible_timeout(&cmp, end - start);

	if (tmo == 0) {
		pr_warning("DMA timed out\n");

	} else if (status != DMA_SUCCESS) {
		pr_warning("DMA got completion callback,"
			   " but status is \'%s\'\n",
			   status == DMA_ERROR ? "error" : "in progress");
	}
	return status;

}

static long test_ioctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct test_ioctl_data *ioctl_data = filp->private_data;
	int retval;
	unsigned char val;
	struct ioctl_arg data;
	int size;
	memset(&data, 0, sizeof(data));
	switch (cmd) {

	case IOCTL_AITDMA_MALLOC:
		{
			void *packet_data_temp_buf = NULL;
			struct aitdma_mem mem;

			if (copy_from_user(&mem, (int __user *)arg, sizeof(mem))) {
				retval = -EFAULT;
				goto done;
			}
			
			mem.kvirt_addr = (unsigned char *)__get_free_pages(GFP_KERNEL, get_order(mem.size));
			mem.kphy_addr = (unsigned long)virt_to_phys(mem.kvirt_addr);

			if (copy_to_user((int __user *)arg, &mem, sizeof(mem)) ) {
				retval = -EFAULT;
			       goto done;
			}
		}
		break;

	case IOCTL_AITDMA_FREEMEM:
		{
			void *packet_data_temp_buf = NULL;
			struct aitdma_mem mem;

			if (copy_from_user(&mem, (int __user *)arg, sizeof(mem))) {
				retval = -EFAULT;
				goto done;
			}
			
			mem.kvirt_addr = (unsigned char *)__get_free_pages(GFP_KERNEL, get_order(mem.size));
			free_page ((u32)(mem.kvirt_addr ));

			
		}
		break;
		

	case IOCTL_AITDMA_START:
		{
			struct aitdma_memcpy dmacpy;
			
			if (copy_from_user(&dmacpy, (int __user *)arg, sizeof(dmacpy))) {
				retval = -EFAULT;
				goto done;
			}

			dma_start(dmacpy.kphy_dest, dmacpy.kphy_src, dmacpy.size);
			break;
		}

	case IOCTL_VALGET_NUM:
		retval = __put_user(ioctl_num, (int __user *)arg);
		break;
	case IOCTL_VALSET_NUM:
		/*
		if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
		*/
		ioctl_num = arg;

		break;
		default:
		retval = -ENOTTY;
	}

done:
	return retval;
}

ssize_t test_ioctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct test_ioctl_data *ioctl_data = filp->private_data;
	unsigned char val;
	int retval;
	int i = 0;

	read_lock(&ioctl_data->lock);
	val = ioctl_data->val;
	read_unlock(&ioctl_data->lock);

	for (;i < count ;i++) {
		if (copy_to_user(&buf[i], &val, 1)) {
			retval = -EFAULT;
			goto out;
		}
	}

	retval = count;

out:
	return retval;
}

static int test_ioctl_close(struct inode *inode, struct file *filp)
{
	printk(KERN_ALERT "%s call.\n", __func__);
	if (filp->private_data) {
		kfree(filp->private_data);
		filp->private_data = NULL;

		dma_release();
	}

	return 0;
}

static int test_ioctl_open(struct inode *inode, struct file *filp)
{
	struct test_ioctl_data *ioctl_data;
	printk(KERN_ALERT "%s call.\n", __func__);

	ioctl_data = kmalloc(sizeof(struct test_ioctl_data), GFP_KERNEL);
	if (ioctl_data == NULL)
		return -ENOMEM;

	rwlock_init(&ioctl_data->lock);
	ioctl_data->val = 0xFF;
	ioctl_data->src_addr = 0x11;
	ioctl_data->dest_addr = 0x22;
	ioctl_data->size = 0x33;
		
	filp->private_data = ioctl_data;

	dma_init();
	
	return 0;
}

struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = test_ioctl_open,
	.release = test_ioctl_close,
	.read = test_ioctl_read,
	.unlocked_ioctl = test_ioctl_ioctl,
};

static int test_ioctl_init(void)
{
	dev_t dev = MKDEV(test_ioctl_major, 0);
	int alloc_ret = 0;
	int cdev_ret = 0;

	alloc_ret = alloc_chrdev_region(&dev, 0, num_of_dev, DRIVER_NAME);
	if (alloc_ret)
		goto error;

	test_ioctl_major = MAJOR(dev);

	cdev_init(&test_ioctl_cdev, &fops);
	cdev_ret = cdev_add(&test_ioctl_cdev, dev, num_of_dev);
	if (cdev_ret)
		goto error;

	printk(KERN_ALERT "%s driver(major: %d) installed.\n", DRIVER_NAME, test_ioctl_major);
	return 0;
error:
	if (cdev_ret == 0)
		cdev_del(&test_ioctl_cdev);
	if (alloc_ret == 0)
		unregister_chrdev_region(dev, num_of_dev);

	return -1;
}

static void test_ioctl_exit(void)
{
	dev_t dev = MKDEV(test_ioctl_major, 0);

	cdev_del(&test_ioctl_cdev);
	unregister_chrdev_region(dev, num_of_dev);
	printk(KERN_ALERT "%s driver removed.\n", DRIVER_NAME);
}

module_init(test_ioctl_init);
module_exit(test_ioctl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("This is ioctl for DMA module.");
