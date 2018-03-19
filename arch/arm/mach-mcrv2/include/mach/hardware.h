/*
 * arch/arm/mach-at91/include/mach/hardware.h
 *
 *  Copyright (C) 2003 SAN People
 *  Copyright (C) 2003 ATMEL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

#define CHIP_INVAL  (0)
#define P_V2        (1)
#define VSN_V2      (2)
#define VSN_V3      (3)
#define MERCURY     (4)
#define VSN_V5      (5)
#define MCR_V2      (6)

#define CHIP_CORE_ID_MCR_V2_SHT     (0x82)
#define CHIP_CORE_ID_MCR_V2_MP       (0x83)
	


#define CHIP        (MCR_V2)
#define SYSTEM_CORE_ID	CHIP_CORE_ID_MCR_V2_MP	


#include <mach/ait_arch_def.h>


/*
 * Peripheral identifiers/interrupts.
 */
#define AIT_ID_FIQ      0	/* Advanced Interrupt Controller (FIQ) */

#ifdef CONFIG_MMU
/*
 * Remap the peripherals from address 0xFFFE0000 .. 0xFFFFFFFF
 * to 0xFF000000-AIT_CPUPHL_SIZE .. 0xFF000000.  (544Kb)
 */

#define AIT_CPUPHL_PHYS_BASE    (0xFFFE0000)
#define AIT_CPUPHL_VIRT_BASE    (0xFF000000 - AIT_CPUPHL_SIZE)
#define AIT_OPR_VIRT_BASE       (0xF0000000)
#define AIT_OPR_VIRT_SIZE       AIT_OPR_PHYS_SIZE
#else
/*
 * Identity mapping for the non MMU case.
 */
#define AIT_CPUPHL_PHYS_BASE	(0xFFFE0000)
#define AIT_CPUPHL_VIRT_BASE	AIT_CPUPHL_PHYS_BASE
#endif

#define AIT_CPUPHL_SIZE     (0xFFFFFFFF - AIT_CPUPHL_PHYS_BASE + 1)

 /* Convert a physical IO address to virtual IO address */
#define AIT_CPUPHL_P2V(x)   ((x) - AIT_CPUPHL_PHYS_BASE + AIT_CPUPHL_VIRT_BASE)

/*
 * Virtual to Physical Address mapping for IO devices.
 */

	
#if defined(CONFIG_AIT_VIDEO_RESERVED_MBYTE)
	#define AIT_DRAM_SIZE       (1024*1024*CONFIG_AIT_VIDEO_RESERVED_MBYTE)
#else
	#define AIT_DRAM_SIZE       (1024*1024*12)
#endif
//#define DDR_2G_OPTIONS
#if defined(DDR_2G_OPTIONS)
#define AIT_RAM_P2V(x)      ((x - (SZ_16M + SZ_8M)) | 0xF0000000)
#define AIT_RAM_V2P(x)      (((x) < (0xF0000000+AIT_OPR_PHYS_SIZE))? \
                                ((x)&~0x70000000): ((x + (SZ_16M + SZ_8M))&~0xF0000000))
#else
#define AIT_RAM_P2V(x)      ((x) | 0xF0000000)
#define AIT_RAM_V2P(x)      (((x) < (0xF0000000+AIT_OPR_PHYS_SIZE))? \
                                ((x)&~0x70000000): ((x)&~0xF0000000))
#endif

#define AIT_OPR_PHYS_BASE   (0x80000000)
#define AIT_OPR_PHYS_SIZE   (SZ_64K)
#define AIT_SRAM_PHYS_BASE  (0x100000)
#define AIT_SRAM_SIZE       (0x26000)
#define AIT_CPUB_ITCM_PHYS_BASE          0x00900000
#define AIT_CPUB_ITCM_SIZE          		SZ_8K      // 4K
#define AIT_CPUB_DRAM_PHYS_BASE          0x03400000
#if defined(CONFIG_AIT_FAST_BOOT)
#define AIT_CPUB_DRAM_SIZE          		(0) //SZ_4M      // 4M
#else
#define AIT_CPUB_DRAM_SIZE          		SZ_4M      // 4M
#endif
#define AIT_SRAM_VIRT_BASE  (AIT_RAM_P2V(AIT_SRAM_PHYS_BASE))
//#define AIT_DRAM_VIRT_BASE  (AIT_RAM_P2V(AIT_DRAM_PHYS_BASE))
#if defined(DDR_2G_OPTIONS)
#define AIT_CPUB_ITCM_VIRT_BASE  (AIT_CPUB_ITCM_PHYS_BASE | 0xF0000000)
#define AIT_CPUB_DRAM_VIRT_BASE  (AIT_CPUB_DRAM_PHYS_BASE | 0xF0000000)
#else
#define AIT_CPUB_ITCM_VIRT_BASE  (AIT_RAM_P2V(AIT_CPUB_ITCM_PHYS_BASE))
#define AIT_CPUB_DRAM_VIRT_BASE  (AIT_RAM_P2V(AIT_CPUB_DRAM_PHYS_BASE))
#endif

 /* Internal SRAM is mapped below the IO devices */
#define AT91_SRAM_MAX		SZ_128K//SZ_1M
#define AT91_VIRT_BASE		(AIT_OPR_VIRT_BASE - AT91_SRAM_MAX)

#define AIT_OPR_P2V(x)      (((x)- AIT_OPR_PHYS_BASE+AIT_OPR_VIRT_BASE))
#define AIT_OPR_V2P(x)      (((x)- AIT_OPR_VIRT_BASE+AIT_OPR_PHYS_BASE))

/* Serial ports */
#define AIT_MAX_UART		(4)

/* I2CS start address*/
#define AIT_I2CS_DRAM_PHYS_BASE 0x03300000
#define AIT_I2CS_DRAM_SIZE	SZ_4K
#define AIT_I2CS_DRAM_VIRT_BASE  (AIT_RAM_P2V(AIT_I2CS_DRAM_PHYS_BASE))

#endif
