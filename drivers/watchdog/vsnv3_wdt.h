/*
 * drivers/watchdog/at91sam9_wdt.h
 *
 * Copyright (C) 2007 Andrew Victor
 * Copyright (C) 2007 Atmel Corporation.
 *
 * Watchdog Timer (WDT) - System peripherals regsters.
 * Based on AT91SAM9261 datasheet revision D.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VSNV3_WDT_H
#define VSNV3_WDT_H
#if 0
typedef struct _AITS_WD {//0xFFFF 8000~800F
    AIT_REG_D   WD_MODE_CTL0;		//0x00
        #define WD_CTL_ACCESS_KEY        0x2340
        #define WD_INT_EN      0x04 
        #define WD_RT_EN          0x02 
        #define WD_EN                0x01      
    AIT_REG_D   WD_MODE_CTL1;		//0x04
        #define WD_CLK_CTL_ACCESS_KEY    0x3700       
    AIT_REG_D   WD_RE_ST;            		//0x08
        #define WD_RESTART    0xC071
    AIT_REG_D   WD_SR;        			//0x0C
        #define WD_RESET_SR        0x02
        #define WD_OVERFLOW_SR     0x01
} AITS_WD, *AITPS_WD;

typedef enum _WD_CLK_DIVIDER
{
	WD_CLK_MCK_D8    = 0x00, 
	WD_CLK_MCK_D32,
	WD_CLK_MCK_D128,
	WD_CLK_MCK_D1024
}WD_CLK_DIVIDER;

#define WD_REG_OFFSET(reg) offsetof(AITS_WD,reg)

#define ait_wd_readl(reg) \
	__raw_readl(AITC_BASE_WD + WD_REG_OFFSET(reg))
#define ait_wd_writel(reg,value) \
	__raw_writel((value), AITC_BASE_WD + WD_REG_OFFSET(reg))

#define WD_RESET ait_wd_writel(WD_RE_ST,WD_RESTART)
#endif
#endif	
#if 0	
#define AT91_WDT_CR		(AT91_WDT + 0x00)	/* Watchdog Control Register */
#define		AT91_WDT_WDRSTT		(1    << 0)		/* Restart */
#define		AT91_WDT_KEY		(0xa5 << 24)		/* KEY Password */

#define AT91_WDT_MR		(AT91_WDT + 0x04)	/* Watchdog Mode Register */
#define		AT91_WDT_WDV		(0xfff << 0)		/* Counter Value */
#define		AT91_WDT_WDFIEN		(1     << 12)		/* Fault Interrupt Enable */
#define		AT91_WDT_WDRSTEN	(1     << 13)		/* Reset Processor */
#define		AT91_WDT_WDRPROC	(1     << 14)		/* Timer Restart */
#define		AT91_WDT_WDDIS		(1     << 15)		/* Watchdog Disable */
#define		AT91_WDT_WDD		(0xfff << 16)		/* Delta Value */
#define		AT91_WDT_WDDBGHLT	(1     << 28)		/* Debug Halt */
#define		AT91_WDT_WDIDLEHLT	(1     << 29)		/* Idle Halt */

#define AT91_WDT_SR		(AT91_WDT + 0x08)	/* Watchdog Status Register */
#define		AT91_WDT_WDUNF		(1 << 0)		/* Watchdog Underflow */
#define		AT91_WDT_WDERR		(1 << 1)		/* Watchdog Error */

#endif
