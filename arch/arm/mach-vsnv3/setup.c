/*
 * Copyright (C) 2007 Atmel Corporation.
 * Copyright (C) 2011 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * Under GPLv2
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/cpu.h>
#include <mach/os_wrap.h>

#include <mach/mmpf_sd.h>
#include <mach/mmpf_i2cm.h>

#include <asm/setup.h>

#include "soc.h"
#include "generic.h"

#include <mach/mmp_reg_gbl.h>

struct ait_init_soc __initdata ait_boot_soc;



void __init ait_soc_init_irq_default(void)
{
    /* Initialize the AIC interrupt controller */
    ait_aic_init(ait_boot_soc.default_irq_priority);

    ait_gpio_irq_setup();
}

static struct map_desc resv_dram_desc __initdata;

void __init ait_soc_resv_dram(int bank, unsigned long base, unsigned int length)
{
	struct map_desc *desc = &resv_dram_desc;

	desc->virtual   = AIT_RAM_P2V(base);
	desc->pfn       = __phys_to_pfn(base),
	desc->length    = length;
	desc->type      = MT_DEVICE;

	pr_info("AIT: resv dram at 0x%lx of 0x%x mapped at 0x%lx\n",
		base, length, desc->virtual);

	iotable_init(desc, 1);
}

static struct map_desc ait_soc_io_desc[] __initdata = {
    {
        .virtual = AIT_OPR_VIRT_BASE,
        .pfn	= __phys_to_pfn(0x80000000),
        .length	= SZ_64K,
        .type	= MT_DEVICE,
    },
    {
        .virtual = AIT_SRAM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_SRAM_PHYS_BASE),
        .length = AIT_SRAM_SIZE,
        .type   = MT_DEVICE,
    },

    {
        .virtual = AIT_CPUPHL_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_CPUPHL_PHYS_BASE),
        .length = AIT_CPUPHL_SIZE/*SZ_128K*/,//SZ_16K,
        .type   = MT_DEVICE,
    },
#if defined CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA
    {
        .virtual = AIT_CPUB_ITCM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_CPUB_ITCM_PHYS_BASE),
        .length = AIT_CPUB_ITCM_SIZE,
        .type   = MT_DEVICE,
    },
    {
        .virtual = AIT_CPUB_DRAM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_CPUB_DRAM_PHYS_BASE),
        .length = SZ_4M,
        .type   = MT_DEVICE,
    },
#endif    
};

#if 0
void __iomem *ait_soc_ioremap(unsigned long p, size_t size, unsigned int type)
{
	pr_info("ait_soc_ioremap: 0x%08x  size: %d\n", (unsigned int)p,size);

	if (p >= AIT_CPUPHL_PHYS_BASE && p <= (AIT_CPUPHL_PHYS_BASE + SZ_16K - 1))
		return (void __iomem *)AIT_CPUPHL_P2V(p);

	if (p >= AIT_OPR_PHYS_BASE && p <= (AIT_OPR_PHYS_BASE + AIT_OPR_PHYS_SIZE - 1))
		return (void __iomem *)AIT_OPR_P2V(p);
	
	return __arm_ioremap_caller(p, size, type, __builtin_return_address(0));
}
EXPORT_SYMBOL(ait_soc_ioremap);

void ait_soc_iounmap(volatile void __iomem *addr)
{
	unsigned long virt = (unsigned long)addr;

	if (virt >= VMALLOC_START && virt < VMALLOC_END)
		__iounmap(addr);
}
EXPORT_SYMBOL(ait_soc_iounmap);
#endif


void __init ait_soc_map_io(void)
{
     unsigned long TotalDramSize;
     unsigned long ResvDramSize = CONFIG_AIT_VIDEO_RESERVED_SIZE;

	/* Map peripherals */
	pr_info("physical mem size 0x%x\n", (unsigned int)meminfo.bank[0].size);

	TotalDramSize = (((meminfo.bank[0].size >> 26) + 1) << 26);  // estimate Total Dram Size(64MB alignment)
	ResvDramSize = (TotalDramSize - meminfo.bank[0].size);  // calculate memory size for video driver
	ait_soc_resv_dram(0, (meminfo.bank[0].size)+CONFIG_PHYS_OFFSET, ResvDramSize);

	iotable_init(ait_soc_io_desc, ARRAY_SIZE(ait_soc_io_desc));

	ait_boot_soc = ait8x_soc;

	if (!ait_soc_is_enabled())
		panic("AIT: Soc not enabled");

	if (ait_boot_soc.map_io)
		ait_boot_soc.map_io();
}

void __init ait_soc_initialize(unsigned long main_clock)
{
	/* MMPF driver initialization */
	MMPF_OS_Initialize();

	MMPF_PIO_Initialize();

	MMPF_SD_InitHandler();

	MMPF_I2cm_InitializeDriver();

	/* Init clock subsystem */
	ait_clock_init(main_clock);

	/* Register the processor-specific clocks */
	ait_boot_soc.register_clocks();

	ait_boot_soc.init();
}
