/* arch/arm/mach-sv886x/reserved_fb.c
 *
 *
 * Copyright (C) 2009 Skyviiav, Inc.
 * Author: Wen-Shien Chen <wschen@skymedi.com.tw>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <generated/autoconf.h>
#include <linux/ptrace.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/bootmem.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <mach/reserved_mem.h>

unsigned long reserved_mem_size = 0;
unsigned long reserved_mem_phys_addr = 0;
caddr_t reserved_mem_start_addr = 0;

static int __init reserved_mem_init(char *str)
{
	unsigned long memsize;

	memsize = virt_to_phys(high_memory);
	reserved_mem_size = CONFIG_RESERVED_MEM_SIZE << 20;

	if(reserved_mem_size) {
		reserved_mem_phys_addr = memsize - reserved_mem_size;
		reserved_mem_start_addr = __alloc_bootmem(reserved_mem_size, SZ_4K, reserved_mem_phys_addr);
	}

	return 1;
}
__setup("reserved_mem=", reserved_mem_init);

