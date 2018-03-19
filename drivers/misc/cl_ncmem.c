/*
 * Copyright (C) 2013 Corelogic, Inc. All rights reserved.
 *
 * Author:
 *   Choi Sung Dae <csd79@bokwang.com>, Aug 2013
 *
  * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "ncmem.h"

#define NCMEM_MINOR     254 /* Major 10, Minor 254, /dev/cl_ncmem */

static struct cl_ncmem_info cl_ncmem;

struct cl_ncmem_info *get_cl_ncmem_instance(void)
{
    return &cl_ncmem;
}

static int cl_ncmem_proc_read(char *resp_buf,char **start, off_t offset, int count, int *eof, void *data)
{
    struct list_head *node;
    struct list_mem_entry *entry;
    int ret, i;

    i = 0;
    ret = sprintf(resp_buf, "- current non-cache memory list info.\n");
    list_for_each(node, &cl_ncmem.list_mem) {
        entry = list_entry(node, struct list_mem_entry, list);
        ret += sprintf(resp_buf+ret, "  [entry %02d] phys:0x%lx, virt:0x%p, user:0x%lx\n", i++, entry->phys_addr, entry->virt_addr, entry->user_addr);
    }

    return ret;
}

/* internal interface functions */
struct list_mem_entry *cl_ncmem_detect_matched_entry(struct cl_ncmem_info *ncmem, unsigned long phys_addr)
{
    struct list_head *node;
    struct list_mem_entry *entry;

    list_for_each(node, &ncmem->list_mem) {
        entry = list_entry(node, struct list_mem_entry, list);
        if (phys_addr == entry->phys_addr) {
                return entry;
        }
    }

    return NULL;
}
EXPORT_SYMBOL(cl_ncmem_detect_matched_entry);

int cl_ncmem_set_user_address(unsigned long phys_addr, unsigned long user_addr)
{
    struct list_mem_entry *entry;

    mutex_lock(&cl_ncmem.mutex);

    entry = cl_ncmem_detect_matched_entry(&cl_ncmem, phys_addr);
    if (!entry) {
        mutex_unlock(&cl_ncmem.mutex);
        return -EINVAL;
    }
    entry->user_addr = user_addr;

    mutex_unlock(&cl_ncmem.mutex);

    return 0;
}
EXPORT_SYMBOL(cl_ncmem_set_user_address);

int cl_ncmem_get_phys_address_info(unsigned int user_addr)
{
    struct list_head *node;
    struct list_mem_entry *entry;
    unsigned int phys_addr;

    mutex_lock(&cl_ncmem.mutex);

    phys_addr = user_to_phys(user_addr);

    list_for_each(node, &cl_ncmem.list_mem) {
        entry = list_entry(node, struct list_mem_entry, list);
        if (phys_addr >= entry->phys_addr && phys_addr < (entry->phys_addr + entry->size)) {
            if (!entry->user_addr) { break; }
            phys_addr = entry->phys_addr + (user_addr - entry->user_addr);
            mutex_unlock(&cl_ncmem.mutex);
            return phys_addr;
        }
    }

    mutex_unlock(&cl_ncmem.mutex);

    return phys_addr;
}
EXPORT_SYMBOL(cl_ncmem_get_phys_address_info);

int cl_ncmem_is_non_cache_memory(unsigned int user_addr)
{
    struct list_head *node;
    struct list_mem_entry *entry;
    unsigned int phys_addr;
    int ret = 0;

    mutex_lock(&cl_ncmem.mutex);

    phys_addr = user_to_phys(user_addr);
    if (phys_addr) {
        list_for_each(node, &cl_ncmem.list_mem) {
            entry = list_entry(node, struct list_mem_entry, list);
            if (phys_addr >= entry->phys_addr && phys_addr < (entry->phys_addr + entry->size)) {
                ret = 1;
                break;
            }
        }
    }

    mutex_unlock(&cl_ncmem.mutex);

    return ret;
}
EXPORT_SYMBOL(cl_ncmem_is_non_cache_memory);

static int ncmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned int size = vma->vm_end - vma->vm_start;
    struct list_mem_entry *entry;

    mutex_lock(&cl_ncmem.mutex);

    entry = cl_ncmem_detect_matched_entry(&cl_ncmem, vma->vm_pgoff << 12);
    if (entry == NULL) {
        dev_err(cl_ncmem.dev, "not detected matched entry(phys_addr=0x%x)\n", (unsigned int)vma->vm_pgoff);
        mutex_unlock(&cl_ncmem.mutex);
        return -EINVAL;
    }

	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_pgoff, size, vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot)) {
		dev_err(cl_ncmem.dev, "fail to dma_mmap_writecombine operation\n"); 
        mutex_unlock(&cl_ncmem.mutex);
        return -EAGAIN;
	}

    mutex_unlock(&cl_ncmem.mutex);

	return 0;
}

static int ncmem_open(struct inode *inode, struct file *file)
{
    struct cl_ncmem_file_info *priv;

    priv = kmalloc(sizeof(struct cl_ncmem_file_info), GFP_KERNEL);
    if (priv == NULL) {
        dev_err(cl_ncmem.dev, "fail to allocate kernel memory for ncmem file indicator\n");
        return -ENOMEM;
    }
    INIT_LIST_HEAD(&priv->list_file_mem);

    file->private_data = priv;

    return 0;
}

/* DMA memory allcoate type */
extern long ncmem_dma_alloc_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);
extern int ncmem_dma_alloc_release(struct inode *inode, struct file *file);
static struct file_operations ncmem_dma_alloc_fop =
{
    .owner		    = THIS_MODULE,
    .read		    = NULL,
    .write		    = NULL,
    .unlocked_ioctl	= ncmem_dma_alloc_ioctl,
    .mmap		    = ncmem_mmap,
    .open		    = ncmem_open,
    .release	    = ncmem_dma_alloc_release,
};

static struct miscdevice ncmem_dma_alloc_misc_device =
{
    NCMEM_MINOR,
    "cl_ncmem",
    &ncmem_dma_alloc_fop,
};

/* DMA memory pool type */
extern long ncmem_dma_pool_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);
extern int ncmem_dma_pool_release(struct inode *inode, struct file *file);
static struct file_operations ncmem_dma_pool_fop =
{
    .owner		    = THIS_MODULE,
    .read		    = NULL,
    .write		    = NULL,
    .unlocked_ioctl	= ncmem_dma_pool_ioctl,
    .mmap		    = ncmem_mmap,
    .open		    = ncmem_open,
    .release	    = ncmem_dma_pool_release,
};

static struct miscdevice ncmem_dma_pool_misc_device =
{
    NCMEM_MINOR,
    "cl_ncmem",
    &ncmem_dma_pool_fop,
};

extern int ncmem_create_dma_pool(struct cl_ncmem_info *cl_ncmem, unsigned int pool_size);
static struct miscdevice *misc;
static int cl_ncmem_probe(struct platform_device *pdev)
{
    struct proc_dir_entry *entry;
    int ret;

    cl_ncmem.dev = &pdev->dev;

    mutex_init(&cl_ncmem.mutex);
    INIT_LIST_HEAD(&cl_ncmem.list_mem);

    /* for debug memory state */
    entry = create_proc_entry("cl_ncmem", S_IRUGO | S_IWUGO , 0);
    if (entry < 0) {
        dev_err(cl_ncmem.dev, "fail to create proc entry\n");
    	return -ENOMEM;
    }   
    entry->read_proc = cl_ncmem_proc_read;

    /* ncmem: DMA memory alloc type */
    misc = &ncmem_dma_alloc_misc_device;

    if (misc_register(misc))
    {
        dev_err(cl_ncmem.dev, "fail to register misc device\n");
        return -EBUSY;
    }

    return 0;
}

extern void ncmem_destroy_dma_pool(struct cl_ncmem_info *cl_ncmem);
static int cl_ncmem_remove(struct platform_device *pdev)
{
    misc_deregister(misc);

    remove_proc_entry("cl_ncmem", NULL);
    mutex_destroy(&cl_ncmem.mutex);

    platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver cl_ncmem_driver = {
	.probe		= cl_ncmem_probe,
	.remove		= cl_ncmem_remove,
	.driver		= {
        .name	= "cl_ncmem",
        .owner	= THIS_MODULE,
	},
};

static __init int cl_ncmem_dma_init(void)
{
    int ret = platform_driver_register(&cl_ncmem_driver);

    return ret;
}

static __exit void cl_ncmem_dma_exit(void)
{
	platform_driver_unregister(&cl_ncmem_driver);
}

module_init(cl_ncmem_dma_init);
module_exit(cl_ncmem_dma_exit);
