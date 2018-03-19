#ifndef __CL_NCMEM_PRIVATE_H__
#define __CL_NCMEM_PRIVATE_H__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/backing-dev.h>
#include <linux/shm.h>
#include <linux/mman.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/syscalls.h>
#include <linux/capability.h>
#include <linux/file.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/hugetlb.h>
#include <linux/profile.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/mempolicy.h>
#include <linux/rmap.h>
#include <linux/mmu_notifier.h>
#include <linux/perf_event.h>

#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/dev-tool.h>

/* for device file exception handling */
struct list_file_entry {
    unsigned long phys_addr;
    struct list_head list;
};

struct cl_ncmem_file_info {
    struct list_head list_file_mem;
};

/* for ncmem control */

struct list_mem_entry {
    unsigned long phys_addr;
    void __iomem *virt_addr;
    unsigned long user_addr;

    int start;
    int blk_num;
    unsigned int size;

    struct list_file_entry file_entry;

    struct list_head list;
};

struct mem_block_info {
    bool used;
};

struct cl_ncmem_info {
	struct device *dev;

    struct {
        struct {
            unsigned long phys_addr;
            void __iomem *virt_addr;
        }base;
        unsigned int num_of_blk;
        struct mem_block_info *mem_blocks;
    }pool;
    unsigned long base_addr;

    struct list_head list_mem;
    struct mutex mutex;
};

struct cl_ncmem_info *get_cl_ncmem_instance(void);
struct list_mem_entry *cl_ncmem_detect_matched_entry(struct cl_ncmem_info *ncmem, unsigned long phys_addr);
int cl_ncmem_set_user_address(unsigned long phys_addr, unsigned long user_addr);
int cl_ncmem_get_phys_address_info(unsigned int user_addr);
int cl_ncmem_is_non_cache_memory(unsigned int user_addr);
void __iomem *cl_ncmem_get_non_cache_memory_virt_address(unsigned long phys_addr);

#endif
