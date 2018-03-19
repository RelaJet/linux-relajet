/*
 * arch/arm/mach-vsnv3/ait_arch.c
 *
 *  Copyright (C)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/cpu.h>

#include <mach/board.h>
#include <mach/ait_arch_def.h>

#include "soc.h"
#include "generic.h"
#include "clock.h"

#include "mmpf_system.h"

/*
 * The peripheral clocks.
 */
static struct clk usart0_clk = {
	.name		= "usart0_clk",
	.type		= CLK_TYPE_PERIPHERAL,
};

static struct clk usart1_clk = {
	.name		= "usart1_clk",
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk usb_clk;
static struct clk udc_clk = {
	.name		= "udc_clk",
	.parent		= &usb_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk sd0_clk;
static struct clk sd0_ctl_clk = {
	.name		= "sd0_ctl_clk ",
	.parent		= &sd0_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk sd1_clk;
static struct clk sd1_ctl_clk = {
	.name		= "sd1_ctl_clk ",
	.parent		= &sd1_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk sd2_clk;
static struct clk sd2_ctl_clk = {
	.name		= "sd2_ctl_clk ",
	.parent		= &sd2_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk i2c_clk;
static struct clk i2cm_ctl_clk = {
	.name		="i2cm_ctl_clk ",
	.parent 	= &i2c_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk pspi_clk;
static struct clk pspi_ctl_clk = {
	.name		= "pspi_ctl_clk",
	.parent 	= &pspi_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk bootspi_clk;
static struct clk sif_ctl_clk = {
	.name		= "sif_ctl_clk",
	.parent 	= &bootspi_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};
#if 1
extern struct clk dac_clk;
static struct clk ssc_clk = {
	.name		= "ssc_clk",
	.parent 		= &dac_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk adc_clk;
static struct clk afe_clk = {
	.name		= "afe_clk",
	//.parent 	= &adc_clk,
	.parent 		= &dac_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk aud_clk;
static struct clk aud_i2s_clk = {
	.name		= "aud_i2s_clk",
	//.parent 	= &adc_clk,
	.parent 		= &aud_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};
#else
extern struct clk audio_clk;
static struct clk ssc_clk = {
	.name		= "ssc_clk",
	.parent 	= &audio_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};

extern struct clk audio2_clk;
static struct clk afe_clk = {
	.name		= "afe_clk",
	.parent 	= &audio2_clk,
	.type		= CLK_TYPE_PERIPHERAL,
};
#endif
//extern struct clk cpu_peri_clk;
//static struct clk tc0_clk = {
//	.name		= "tc0_clk",
//	.parent 	= &cpu_peri_clk,
//	.type		= CLK_TYPE_PERIPHERAL,
//};
//
//static struct clk tc1_clk = {
//	.name		= "tc1_clk",
//	.parent 	= &cpu_peri_clk,
//	.type		= CLK_TYPE_PERIPHERAL,
//};
//
//static struct clk tc2_clk = {
//	.name		= "tc2_clk",
//	.parent 	= &cpu_peri_clk,
//	.type		= CLK_TYPE_PERIPHERAL,
//};
//
//static struct clk wdt_clk = {
//	.name		= "wdt_clk",
//	.parent 	= &cpu_peri_clk,
//	.type		= CLK_TYPE_PERIPHERAL,
//};

/* Exported peripherals for module drivers */
static struct clk *periph_clocks[] __initdata = {
	&usart0_clk,
    &usart1_clk,

	&udc_clk,

	&sd0_ctl_clk,
	&sd1_ctl_clk,
	&sd2_ctl_clk,	
	
	&i2cm_ctl_clk,

	&pspi_ctl_clk,
	&sif_ctl_clk,

	//&tc0_clk,
	//&tc1_clk,
	//&tc2_clk,
	//&wdt_clk,

	&afe_clk,
	&aud_i2s_clk
};

static struct clk_lookup periph_clocks_lookups[] = {
	CLKDEV_CON_DEV_ID("sd0_ctl_clk", DEVICE_NAME_SD0, &sd0_ctl_clk),	
	CLKDEV_CON_DEV_ID("sd1_ctl_clk", DEVICE_NAME_SD1, &sd1_ctl_clk),			
	CLKDEV_CON_DEV_ID("sd2_ctl_clk", DEVICE_NAME_SD2, &sd2_ctl_clk),			
	CLKDEV_CON_DEV_ID("pspi_ctl_clk", "ait_pspi.0", &pspi_ctl_clk),
	CLKDEV_CON_DEV_ID("pspi_ctl_clk", "ait_pspi.1", &pspi_ctl_clk),
	CLKDEV_CON_DEV_ID("pspi_ctl_clk", "ait_pspi.2", &pspi_ctl_clk),	
	CLKDEV_CON_DEV_ID("pspi_ctl_clk", "ait_pspi.3", &pspi_ctl_clk),		
	CLKDEV_CON_DEV_ID("sif_ctl_clk", "vsnv3_sif.0", &sif_ctl_clk),

	//CLKDEV_CON_DEV_ID("tc1_clk", "vsnv3_tc1", &tc1_clk),
	//CLKDEV_CON_DEV_ID("tc2_clk", "vsnv3_tc2", &tc2_clk),
	//CLKDEV_CON_DEV_ID("wdt_ctl_clk", "vsnv3_wdt", &wdt_clk),
	CLKDEV_CON_DEV_ID("pclk", "vsnv3aud.0", &ssc_clk),
	CLKDEV_CON_DEV_ID("pclk", "vsnv3aud.1", &afe_clk),
	CLKDEV_CON_DEV_ID("udc_clk", "vsnv3udc", &udc_clk),	

	CLKDEV_CON_DEV_ID("afe_clk", "vsnv3-afe-codec", &afe_clk),	
	CLKDEV_CON_DEV_ID("clk_audio_i2s", "mcrv2-soc-i2s", &aud_i2s_clk),	
	CLKDEV_CON_DEV_ID("clk_audio_i2s", "mcrv2-soc-i2s.0", &aud_i2s_clk),
	CLKDEV_CON_DEV_ID("clk_audio_i2s", "mcrv2-soc-i2s.1", &aud_i2s_clk),
	
};

static struct clk_lookup usart_clocks_lookups[] = {
    CLKDEV_CON_DEV_ID("usart", "ait_usart0", &usart0_clk),
    CLKDEV_CON_DEV_ID("usart", "ait_usart1", &usart1_clk),
    CLKDEV_CON_DEV_ID("usart", "ait_usart2", &usart1_clk),
    CLKDEV_CON_DEV_ID("usart", "ait_usart3", &usart1_clk)   
};

static void __init ait8x_register_clocks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(periph_clocks); i++)
		clk_register(periph_clocks[i]);

	clkdev_add_table(periph_clocks_lookups,
			 ARRAY_SIZE(periph_clocks_lookups));
	clkdev_add_table(usart_clocks_lookups,
			 ARRAY_SIZE(usart_clocks_lookups));
}

static struct clk_lookup console_clock_lookup[AIT_MAX_UART];

void __init ait_arch_set_console_clock(int id)
{
	if (id >= ARRAY_SIZE(usart_clocks_lookups))
		return;

	console_clock_lookup[id].con_id = "usart";
	console_clock_lookup[id].clk = usart_clocks_lookups[id].clk;
	clkdev_add(&console_clock_lookup);
}

/* --------------------------------------------------------------------
 *  GPIO
 * -------------------------------------------------------------------- */
extern struct clk gpio_clk;
extern struct clk pwm_clk ;

static struct ait_gpio_bank ait_arch_gpio[] = {
	{
		.id		= 0,
		.name = "Bank0",
		.clock		= &gpio_clk,
	},
	{
		.id		= 1,
		.name = "Bank1",
		.clock		= &gpio_clk,
	},
	{
		.id		= 2,
		.name = "Bank2",	
		.clock		= &gpio_clk,
	},
	{
		.id		= 3,
		.name = "Bank3",	
		.clock		= &gpio_clk,
	},
#if 1 // add for 8428 saradc
	{
		.id = 4 ,
		.name = "Bank4",
		.clock = &pspi_clk ,
	},
#endif	
#if 1 // pwm
  {
     .id = 5,
     .name = "Bank5",
     .clock = &pwm_clk,
  },
#endif  
#if 1 // watchdog
  {
   .id= 6,
   .name ="hwdog",  
   .clock=0, // no clock
  }
#endif  
};


/* --------------------------------------------------------------------
 *  Processor initialization
 * -------------------------------------------------------------------- */

static void __init ait8x_map_io(void)
{
}

static void ait8x_reset(void)
{
    pr_alert("AIT RESET.\r\n");
    MMPF_SYS_ResetSystem(0);
}

static void __init ait8x_initialize(void)
{
    ait_arch_reset = ait8x_reset;
	
#if !defined(CONFIG_AIT_FAST_BOOT) // LaCam

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SCALE, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_JPG, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_ISP, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_PWM, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_PSPI, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_DMA, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_H264, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_ICON, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_GRA, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_IBC, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_GNR, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SM, MMP_FALSE);

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_CCIR, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_HDMI, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_TV, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_BAYER, MMP_FALSE);

#if !defined(CONFIG_SND_MCRV2_SOC_I2S)
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_RAW_F, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_RAW_S0, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_RAW_S1, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_RAW_S2, MMP_FALSE);
#endif	
#ifdef CONFIG_AIT_MCRV2_DUAL_OS_DISABLE
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_CPU_B, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_CPU_B_PHL, MMP_FALSE);
     
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_COLOR_MCI, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_CCIR, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_HDMI, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_TV, MMP_FALSE);
	
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD0, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD1, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD2, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD3, MMP_FALSE);

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_GRP0_NUM, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_I2CM, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_BS_SPI, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_GPIO, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_AUD, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_ADC, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_DAC, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_IRDA, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_LDC, MMP_FALSE);	
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_BAYER, MMP_FALSE);
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_MDL_NUM, MMP_FALSE);
#endif	

#endif
    //ait_gpio_init(ait_arch_gpio, 1);
	ait_gpio_init(ait_arch_gpio, ARRAY_SIZE(ait_arch_gpio));
}

__inline void AT91F_ARM_CleanDCacheIDX(unsigned int index)
{
    /* ARM926EJ-S */
    // 16K cache
	asm volatile("mcr p15, 0, %0, c7, c10, 2" : : "r" ((index & 0xC0000FE0)));
}

void AT91F_ARM_DrainWriteBuffer()
{
	register unsigned int sbz = 0;
	asm volatile("mcr p15, 0, %0, c7, c10, 4" : : "r" (sbz));	
}

EXPORT_SYMBOL(AT91F_ARM_DrainWriteBuffer);

__inline void AT91F_ARM_CleanInvalidateDCacheMVA(unsigned int mva)
{
 	asm volatile("MCR p15, 0, %0, c7, c14, 1" : : "r" (mva & 0xFFFFFFE0));
}


void MMPF_MMU_FlushDCacheMVA(MMP_ULONG ulRegion, MMP_ULONG ulSize)
{
    MMP_ULONG	i;
    MMP_ULONG	start_addr, end_addr;
    start_addr = FLOOR32(ulRegion);
    end_addr = ALIGN32(ulRegion + ulSize);

    for (i = start_addr; i < end_addr; i += 32) {
        AT91F_ARM_CleanInvalidateDCacheMVA(i);
    }
    // Flush write buffers
    AT91F_ARM_DrainWriteBuffer();
}
EXPORT_SYMBOL(MMPF_MMU_FlushDCacheMVA);


__inline void AT91F_ARM_InvalidateDCacheMVA(unsigned int mva)
{
   	asm volatile("MCR p15, 0, %0, c7, c6, 1": : "r" ((mva & 0xFFFFFFE0)));
}

void AT91F_CleanDCache()
{
    register int seg, index;
    int setsize = 128;

    /* ARM926EJ-S */
    /// For ARM926EJ-S, seg is the index of "WAY", index is the index of "set".
    /// ("WAY is ""LINE", 4-WAY means 4 lines in each "set")

    /// ARM926EJ-S addressing is WAY at the first, then SET.
    for (seg = 0; seg < 4; ++seg) {
        for (index = 0; index < setsize; ++index) {
            AT91F_ARM_CleanDCacheIDX((seg << 30) | (index << 5));
        }
    }
}
EXPORT_SYMBOL(AT91F_CleanDCache);


void MMPF_MMU_InvalidateDCacheMVA(MMP_ULONG ulRegion, MMP_ULONG ulSize)
{
    MMP_ULONG	i;
    MMP_ULONG	start_addr, end_addr;
    start_addr = ulRegion;
    end_addr = ulRegion + ulSize;

    for (i = start_addr; i < end_addr; i += 32) {
        AT91F_ARM_InvalidateDCacheMVA(i);
    }
}
EXPORT_SYMBOL(MMPF_MMU_InvalidateDCacheMVA);

/* --------------------------------------------------------------------
 *  Interrupt initialization
 * -------------------------------------------------------------------- */

/*
 * The default interrupt priority levels (1 = lowest, 7 = highest).
 */
static unsigned int ait8x_default_irq_priority[NR_AIC_IRQS] __initdata = {
    [AIC_SRC_USBDMA] = 7,
    [AIC_SRC_USB] = 6,		
    [AIC_SRC_SPI] = 7,
    [AIC_SRC_TC0] = 5,
    [AIC_SRC_WD] = 1,
    // else 0, will treat as 3 in ait_init_aic
};

struct ait_init_soc __initdata ait8x_soc = {
	.map_io                 = ait8x_map_io,
	.default_irq_priority   = ait8x_default_irq_priority,
	.register_clocks        = ait8x_register_clocks,
	.init                   = ait8x_initialize,
};
