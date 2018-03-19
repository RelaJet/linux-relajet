
/*
 * Copyright (C) 2013 Corelogic, Inc. All rights reserved.
 *
 * Author:
 *   Choi Sung Dae <csd79@bokwang.com>, Jan 2013
 * 
 * Irene SoC - driver support functions 
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm-generic/ioctl.h>

static inline unsigned long cpu_to_phys(void * cpu)
{
    unsigned long address = (unsigned long)cpu;
    pmd_t *pmdp;
    pte_t *ptep;
    pmdp = pmd_offset(pud_offset(pgd_offset_k(address), address), address);
    
    preempt_disable();
    
    ptep = pte_offset_kernel(pmdp, address);
    if (pte_present(*ptep)) {
        address = (unsigned long) page_address(pte_page(*ptep));
        address += address & ~PAGE_MASK;
        address = virt_to_phys((void *)address);
    }
    
    preempt_enable();
    
    return address;
}

static inline unsigned long user_to_phys(unsigned long user)
{
    unsigned long address = user;
    pmd_t *pmdp;
    pte_t *ptep;
    pmdp = pmd_offset(pud_offset(pgd_offset(current->mm, address), address), address);
    
    preempt_disable();
    
    ptep = pte_offset_map(pmdp, address);
    if (pte_present(*ptep)) {
        address = (unsigned long) page_address(pte_page(*ptep));
        address += address & ~PAGE_MASK;
        address = virt_to_phys((void *)address);
    }
    
    pte_unmap(ptep);
    
    preempt_enable();
    
    return address;
}

static inline int checkIoctlData(unsigned char magic, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

	if (_IOC_TYPE(cmd) != magic)
            return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
            ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
            ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (ret)
            return -EFAULT;

    return 0;
}

static inline void write_bits_per_reg(void __iomem *addr, int bit_mask, unsigned int value, int offset)
{
	unsigned long reg;

	reg = readl(addr);
	reg &= ~(bit_mask << offset);
	reg |= ((value & bit_mask) << offset);
	writel(reg, addr);
}

static inline unsigned long read_bits_per_reg(void __iomem *addr, int bit_mask, int offset)
{
    return ((readl(addr) >> offset) & bit_mask);
}
