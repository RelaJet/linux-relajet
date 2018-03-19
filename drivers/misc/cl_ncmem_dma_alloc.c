
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

#include <linux/cl_ncmem.h>
#include "ncmem.h"

static __inline int _alloc_dma_memory(struct file *fp, unsigned int size)
{
    struct cl_ncmem_info *ncmem = get_cl_ncmem_instance();
    struct cl_ncmem_file_info *priv = (struct cl_ncmem_file_info *)fp->private_data;
    struct list_mem_entry *entry;

    mutex_lock(&ncmem->mutex);

    entry = kmalloc(sizeof(struct list_mem_entry), GFP_KERNEL);
    if (!entry) {
        dev_err(ncmem->dev, "fail to kernel memory allocate for memory list entry\n");
        mutex_unlock(&ncmem->mutex);
        return 0;
    }
    memset(entry, 0x0, sizeof(struct list_mem_entry));

    entry->size = PAGE_ALIGN(size);
    entry->virt_addr = dma_alloc_writecombine(ncmem->dev, entry->size, (dma_addr_t *)&entry->phys_addr, GFP_KERNEL);
    if (!entry->virt_addr) {
        dev_err(ncmem->dev, "fail to operation dma_alloc_writecombine\n");
        mutex_unlock(&ncmem->mutex);
        return 0;
    }
    list_add_tail(&entry->list, &ncmem->list_mem);

    /* add list entry for file exception handling */
    entry->file_entry.phys_addr = entry->phys_addr;
    list_add_tail(&entry->file_entry.list, &priv->list_file_mem);    

    mutex_unlock(&ncmem->mutex);

    return (int)entry->phys_addr;
}

static __inline int _free_dma_memory(unsigned long phys_addr)
{
    struct cl_ncmem_info *ncmem = get_cl_ncmem_instance();
    struct list_mem_entry *entry;

    mutex_lock(&ncmem->mutex);

    entry = cl_ncmem_detect_matched_entry(ncmem, phys_addr);
    if (!entry) {
        mutex_unlock(&ncmem->mutex);
        return -EINVAL;
    }

    dma_free_writecombine(ncmem->dev, entry->size, entry->virt_addr, entry->phys_addr);

    list_del(&entry->file_entry.list);
    list_del(&entry->list);

    kfree(entry);

    mutex_unlock(&ncmem->mutex);
     
    return 0;
}

long ncmem_dma_alloc_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    struct user_addr_info info;
    struct cl_ncmem_info *ncmem = get_cl_ncmem_instance();
    int ret;

    ret = checkIoctlData(CL_NCMEM_IOC_MAGIC, cmd, arg);
    if (ret < 0) {
        dev_err(ncmem->dev, "fail to data check for non-cache memory allocator ioctl cmd (%d)\n", cmd);
        return ret;
    }

    switch (cmd) {
    case CL_NCMEM_IOC_ALLOCATE_NON_CACHE_MEMORTY:
        ret = _alloc_dma_memory(fp, arg);
        break;
    case CL_NCMEM_IOC_FREE_NON_CACHE_MEMORTY:
        ret = _free_dma_memory(arg);
        break;
    case CL_NCMEM_IOC_SET_USER_ADDRESS_INFO:
        copy_from_user((void *)&info, (const void *)arg, sizeof(struct user_addr_info));
        ret = cl_ncmem_set_user_address(info.phys_addr, info.user_addr);
        break;
    case CL_NCMEM_IOC_GET_PHYS_ADDRESS_INFO:
        ret = cl_ncmem_get_phys_address_info(arg);
        break;
    case CL_NCMEM_IOC_CHECK_MEMORY_OWNER:
        ret = cl_ncmem_is_non_cache_memory(arg);
        break;
    default:
        dev_err(ncmem->dev, "invalid IOCTL command type for non-cache memory allocator (%d)\n", cmd);
        return -EINVAL;
    }

    return ret;
}
EXPORT_SYMBOL(ncmem_dma_alloc_ioctl);

int ncmem_dma_alloc_release(struct inode *inode, struct file *file)
{
    struct cl_ncmem_info *ncmem = get_cl_ncmem_instance();
    struct cl_ncmem_file_info *priv = (struct cl_ncmem_file_info *)file->private_data;
    struct list_head *node;
    struct list_file_entry *file_entry;
    struct list_mem_entry *entry;

    mutex_lock(&ncmem->mutex);

    list_for_each(node, &priv->list_file_mem) {
        file_entry = list_entry(node, struct list_file_entry, list);
        entry = cl_ncmem_detect_matched_entry(ncmem, file_entry->phys_addr);
        if (!entry) { continue; }

        dma_free_writecombine(ncmem->dev, entry->size, entry->virt_addr, entry->phys_addr);
        list_del(&entry->file_entry.list);
        list_del(&entry->list);
        kfree(entry);

        if (list_empty(&priv->list_file_mem)) { break; }
    }

    mutex_unlock(&ncmem->mutex);

    kfree(priv);
    file->private_data = NULL;

    return 0;
}
EXPORT_SYMBOL(ncmem_dma_alloc_release);
