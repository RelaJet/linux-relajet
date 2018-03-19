/*
 * Copyright (C) 2007 Atmel Corporation.
 * Copyright (C) 2011 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * Under GPLv2
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/memblock.h>

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
#if defined (CONFIG_AIT_FAST_BOOT)
#include <mach/ait_cpu_sharemem.h>
#endif

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

	pr_info("[AIT]: resv dram at 0x%lx of 0x%x mapped at 0x%lx\n",
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
#if defined CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA && AIT_CPUB_DRAM_SIZE

    {
        .virtual = AIT_CPUB_ITCM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_CPUB_ITCM_PHYS_BASE),
        .length = AIT_CPUB_ITCM_SIZE,
        .type   = MT_DEVICE,
    },
    {
        .virtual = AIT_CPUB_DRAM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_CPUB_DRAM_PHYS_BASE),
        .length = AIT_CPUB_DRAM_SIZE,
        .type   = MT_DEVICE,
    },
#endif
#if defined (CONFIG_MCRV2_I2CS)
	{
		.virtual = AIT_I2CS_DRAM_VIRT_BASE,
        .pfn    = __phys_to_pfn(AIT_I2CS_DRAM_PHYS_BASE),
        .length = AIT_I2CS_DRAM_SIZE,
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


void __init ait_dram_reserve(phys_addr_t paddr, phys_addr_t size)
{

	if (paddr) {
		if (paddr & ~PAGE_MASK) {
			pr_err("ait_dram_reserve:DRAM start address 0x%08x not page aligned\n",
			paddr);
			return;
		}

		if (!memblock_is_region_memory(paddr, size)) {
			pr_err("ait_dram_reserve:Illegal DRAM region 0x%08x..0x%08x for DRAM\n",
			paddr, paddr + size - 1);
			return;
		}

		if (memblock_is_region_reserved(paddr, size)) {
			pr_err("ait_dram_reserve:: failed to reserve DVRAM - busy\n");
			return;
		}

		if (memblock_reserve(paddr, size) < 0) {
			pr_err("ait_dram_reserve:: failed to reserve DRAM - no memory\n");
			return;
		}
	}
}

#if 0//defined (CONFIG_AIT_FAST_BOOT)
unsigned long resv_cpub_addr ,resv_cpub_size ;
static int __init resv_cpub_size_init(char *p)
{
	//reserved_mem_size = CONFIG_RESERVED_MEM_SIZE << 20;
	unsigned long size=0,start=0;
	char *endp;
	size  = memparse(p, &endp);
	if (*endp == '@')
		start = memparse(endp + 1, NULL);

	resv_cpub_size = size  ;
	resv_cpub_addr = (start ) + CONFIG_PHYS_OFFSET ;
	
  pr_info("#CPUB: Resv size from cmd line : %s,0x%08x@0x%08x\n",p,resv_cpub_size,resv_cpub_addr);
	return 1;
}
__setup("resv_cpub_size=", resv_cpub_size_init);
#endif


void __init ait_dram_reserve_cpub(void)
{
#if defined CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA && AIT_CPUB_DRAM_SIZE
  	ait_dram_reserve(AIT_CPUB_DRAM_PHYS_BASE, AIT_CPUB_DRAM_SIZE);
  	pr_info("cpub reserved at 0x%08x,size : 0x%08x\n",AIT_CPUB_DRAM_PHYS_BASE,AIT_CPUB_DRAM_SIZE);
#endif

}

void __init ait_dram_reserve_i2cs(void)
{
  	ait_dram_reserve(AIT_I2CS_DRAM_PHYS_BASE, AIT_I2CS_DRAM_SIZE);
  	pr_info("i2cs reserved at 0x%08x,size : 0x%08x\n",AIT_I2CS_DRAM_PHYS_BASE, AIT_I2CS_DRAM_SIZE);
}

extern void __init init_consistent_dma_size(unsigned long size);

void __init ait_soc_map_io(void)
{
     unsigned long TotalDramSize;
     unsigned long ResvDramSize = CONFIG_AIT_VIDEO_RESERVED_MBYTE*1024*1024;
     unsigned long ResvStartAddr = meminfo.bank[0].size ;
#if defined (CONFIG_AIT_FAST_BOOT)
  struct _cpu_rtos_mem_info *rtos_mem ;
	init_consistent_dma_size(SZ_2M);
	// init io-remap...
	iotable_init(ait_soc_io_desc, ARRAY_SIZE(ait_soc_io_desc));
	// so that we can get register's here
  rtos_mem = get_cpub_rtos_mem() ;
#else
	init_consistent_dma_size(SZ_8M);
#endif
	/* Map peripherals */

#ifdef CONFIG_MACH_MCRV2_AIT6366G_EVB // TBD¡@¡G dead define
	TotalDramSize = 128*1024*1024;
	ResvDramSize = (TotalDramSize - meminfo.bank[0].size);  // calculate memory size for video driver
#else

  #if defined (CONFIG_AIT_FAST_BOOT)
  	//TotalDramSize = ((( (meminfo.bank[0].size+ ResvDramSize) >> 26) + 1) << 26);  // estimate Total Dram Size(64MB alignment)
  	/*
  	fixed addr for 64/128/256MB dram compatible.
  	*/
  	ResvStartAddr = 20 * 1024 * 1024 ;
  	if(rtos_mem->reset_base) {
  	  ResvStartAddr = (rtos_mem->reset_base - CONFIG_PHYS_OFFSET );
  	  ResvDramSize  = ((unsigned int)(rtos_mem->v4l2_size_mb + rtos_mem->working_size_mb )) * 1024*1024 ;
  	}
  #else
	if(meminfo.bank[0].size < SZ_32M)
	{
		TotalDramSize = SZ_32M;
	}else if(meminfo.bank[0].size < SZ_64M){
		TotalDramSize = SZ_64M;
	}else if(meminfo.bank[0].size < SZ_128M){
		TotalDramSize = SZ_128M;
	}else if(meminfo.bank[0].size < SZ_256M){
		TotalDramSize = SZ_256M - SZ_16M;
	}else{
		BUG();
	}
	ResvDramSize = (TotalDramSize - meminfo.bank[0].size);  // calculate memory size for video driver
  #endif

#endif
	ait_soc_resv_dram(0, (ResvStartAddr)+CONFIG_PHYS_OFFSET, ResvDramSize);
#if !defined (CONFIG_AIT_FAST_BOOT)
	iotable_init(ait_soc_io_desc, ARRAY_SIZE(ait_soc_io_desc));
#else
// end of phy dram has occupied by Linux (  ? Kb )
// don't reserved addr 62~64Mb when MDDR size is 64MB
{
  int i ;
  int n_of_2M = ( ResvDramSize + SZ_2M - 1  ) / SZ_2M ;
  unsigned long resv_addr ;
  //pr_info("Reserved CPUB from 0x%0x,size:%d,# of 2M : %d\n",ResvStartAddr,ResvDramSize,n_of_2M);
  
  for(i=0;i<n_of_2M;i++) {
    resv_addr = (CONFIG_PHYS_OFFSET + ResvStartAddr + i*SZ_2M );
    ait_dram_reserve( resv_addr , SZ_2M );
    //pr_info("CPUB : resv addr : 0x%08x\n",resv_addr );
  }
}
#endif

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
