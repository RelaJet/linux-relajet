/*
 * linux/arch/arm/mach-vsnv3/clock.c
 * Copyright (C) 2014 Alpha Image Tech.
 *
 *  Modify from linux/arch/arm/mach-at91/clock.c
 *
 * Copyright (C) 2005 David Brownell
 * Copyright (C) 2005 Ivan Kokshaysky
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/includes_fw.h>
#include <mach/mmpf_pll.h>
#include <mach/mmpf_system.h>

#include <mach/cpu.h>

#include "clock.h"
#include "generic.h"
//#include <mach/mmp_registr.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_sd.h>


/*
 * There's a lot more which can be done with clocks, including cpufreq
 * integration, slow clock mode support (for system suspend), letting
 * PLLB be used at other rates (on boards that don't need USB), etc.
 */
#define CLK_DEBUG

#define clk_is_primary(x)	    ((x)->type & CLK_TYPE_PRIMARY)
#define clk_is_programmable(x)	((x)->type & CLK_TYPE_PROGRAMMABLE)
#define clk_is_peripheral(x)	((x)->type & CLK_TYPE_PERIPHERAL)
#define clk_is_sys(x)		    ((x)->type & CLK_TYPE_CLK_SYS)

static LIST_HEAD(clocks);
static DEFINE_SPINLOCK(clk_lock);

static void pmc_sys_mode(struct clk *clk, int is_on);
#if 0
static struct clk clk32k = {
	.name		= "clk32k",
	.rate_hz	= AT91_SLOW_CLOCK,
	.users		= 1,		/* always on */
	.id		= 0,
	.type		= CLK_TYPE_PRIMARY,
};
#endif
static struct clk main_clk = {
    .name       = "main",
    .rate_hz    = 12000000,
    .pmc_mask   = 0,
    .id         = 1,
    .type       = CLK_TYPE_PRIMARY,
};
#if 0
static struct clk plla = {
	.name		= "plla",
	.parent		= &main_clk,
	.pmc_mask	= 0,
	.id		= 2,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
#endif

static struct clk pll0 = {
	.name		= "pll0",
	.rate_hz		= 600000000,		
	.parent		= &main_clk,
	.pmc_mask	= 0,//AT91_PMC_LOCKA,
	.id		= 3,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static struct clk pll1 = {
	.name		= "pll1",
	.rate_hz		= 180000000,
	.parent		= &main_clk,
	.pmc_mask	= 0,//AT91_PMC_LOCKA,
	.id		= 4,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static struct clk pll2 = {
	.name		= "pll2",
	//.rate_hz		= 264000000,						
	.parent		= &main_clk,
	.pmc_mask	= 0,//AT91_PMC_LOCKA,
	.id		= 5,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};


static void pllb_mode(struct clk *clk, int is_on)
{
	u32	value;

	if (is_on) {
//		is_on = AT91_PMC_LOCKB;
//		value = at91_pllb_usb_init;
	} else
		value = 0;

	// REVISIT: Add work-around for AT91RM9200 Errata #26 ?
//	at91_sys_write(AT91_CKGR_PLLBR, value);

//	do {
//		cpu_relax();
//	} while ((at91_sys_read(AT91_PMC_SR) & AT91_PMC_LOCKB) != is_on);
}
#if 0
static struct clk pllb = {
	.name		= "pllb",
	.parent		= &main_clk,
	.pmc_mask	= 0,//AT91_PMC_LOCKB,	/* in PMC_SR */
	.mode		= pllb_mode,
	.id		= 3,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
#endif


static void pmc_sys_mode(struct clk *clk, int is_on)
{

	AITPS_GBL pGBL = AITC_BASE_GBL;

	BUG_ON(clk->type&CLK_TYPE_CLK_SYS == 0);

	pr_info("Sys clock %s %s\n",clk->name,is_on?"on":"off");


	if (is_on)
	{
		if(clk->type==CLK_TYPE_CLK_CTL1)
		{
			pGBL->GBL_CLK_EN[0] |=clk->pmc_mask;
		}
		if(clk->type==CLK_TYPE_CLK_CTL2)
		{
			pGBL->GBL_CLK_EN[1] |=clk->pmc_mask;
		}

	}	
	else
	{
		if(clk->type==CLK_TYPE_CLK_CTL1)
		{
			pGBL->GBL_CLK_EN[0] &=~clk->pmc_mask;
		}
		if(clk->type==CLK_TYPE_CLK_CTL2)
		{
			pGBL->GBL_CLK_EN[1] &=~clk->pmc_mask;
		}

	}
		
}

static void pmc_periph_mode(struct clk *clk, int is_on)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
	
	BUG_ON(clk->type != CLK_TYPE_PERIPHERAL);


	pr_info("Periph clock %s %s\n",clk->name,is_on?"on":"off");

	if (is_on)
	{
//		pr_info("turn on.\r\n");

		if(strcmp(clk->name,"sd0_ctl_clk")==0)	//if(clk->pmc_mask& (1<<AITVSNV3_ID_SD))
		{
			AITPS_SD pSD = AITC_BASE_SD0;
			pSD->SD_CTL_1 |= CLK_EN;
		}
		else if(strcmp(clk->name,"sd1_ctl_clk")==0)//if(clk->pmc_mask&( 1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD1;		
			pSD->SD_CTL_1 |= CLK_EN;
		}
		else if(strcmp(clk->name,"sd2_ctl_clk")==0)//if(clk->pmc_mask&( 1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD2;		
			pSD->SD_CTL_1 |= CLK_EN;
		}	
#if 0
		if(clk->pmc_mask& (1<<AITVSNV3_ID_SD))
		{
			AITPS_SD pSD = AITC_BASE_SD0;
			pSD->SD_CTL_1 |= CLK_EN;
		}
		else if(clk->pmc_mask&( 1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD1;		
			pSD->SD_CTL_1 |= CLK_EN;
		}
		
		else if(clk->pmc_mask& (1<<AITVSNV3_ID_AFE_FIFO))
		{
			pGBL->GBL_CLK_DIS2 &=~GBL_CLK_AUD_CODEC_DIS;
		}

		else if(clk->pmc_mask&( 1<<AITVSNV3_ID_I2S_FIFO))
		{
			//pGBL->GBL_CLK_DIS2 &=~GBL_CLK_AUD_CODEC_DIS;
			pGBL->GBL_CLK_DIS0 &=~GBL_CLK_AUD_DIS;
		}else if(clk->pmc_mask&(1<<AITVSNV3_ID_PSPI))
		{
			pGBL->GBL_CLK_DIS1 &= (~GBL_CLK_PSPI_DIS);
		}else if(clk->pmc_mask&(1<<AITVSNV3_ID_USB))
		{
			pGBL->GBL_CLK_DIS1 &= (~GBL_CLK_USB_DIS);
		}
#endif

	}
	else
	{
		if(strcmp(clk->name,"sd0_ctl_clk")==0)	//if(clk->pmc_mask& (1<<AITVSNV3_ID_SD))
		{
			AITPS_SD pSD = AITC_BASE_SD0;
			pSD->SD_CTL_1 &= ~CLK_EN;
		}
		else if(strcmp(clk->name,"sd1_ctl_clk")==0)//if(clk->pmc_mask&( 1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD1;		
			pSD->SD_CTL_1 &= ~CLK_EN;
		}
		else if(strcmp(clk->name,"sd2_ctl_clk")==0)//if(clk->pmc_mask&( 1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD2;		
			pSD->SD_CTL_1 &= ~CLK_EN;
		}	

//		pr_info("turn off.\r\n");
#if 0
		if(clk->pmc_mask& (1<<AITVSNV3_ID_SD))
		{
			AITPS_SD pSD = AITC_BASE_SD0;
			pSD->SD_CTL_1 &= ~CLK_EN;
		}
		if(clk->pmc_mask& (1<<AITVSNV3_ID_SD1))
		{
			AITPS_SD pSD = AITC_BASE_SD1;	
			pSD->SD_CTL_1 &= ~CLK_EN;
		}

		if(clk->pmc_mask&( 1<<AITVSNV3_ID_AFE_FIFO))
		{
			.//pGBL->GBL_CLK_DIS2 |=GBL_CLK_AUD_CODEC_DIS;
		}
		if(clk->pmc_mask& (1<<AITVSNV3_ID_I2S_FIFO))
		{
//			pGBL->GBL_CLK_DIS2 |=GBL_CLK_AUD_CODEC_DIS;
			pGBL->GBL_CLK_DIS0 |=GBL_CLK_AUD_DIS;
		}else if(clk->pmc_mask&(1<<AITVSNV3_ID_USB))
		{
			pGBL->GBL_CLK_DIS1 |= GBL_CLK_USB_DIS;
		}
#endif		
	}
}

static void pmc_uckr_mode(struct clk *clk, int is_on)
{
#if 0
	unsigned int uckr = at91_sys_read(AT91_CKGR_UCKR);

	if (cpu_is_at91sam9g45()) {
		if (is_on)
			uckr |= AT91_PMC_BIASEN;
		else
			uckr &= ~AT91_PMC_BIASEN;
	}

	if (is_on) {
		is_on = AT91_PMC_LOCKU;
		at91_sys_write(AT91_CKGR_UCKR, uckr | clk->pmc_mask);
	} else
		at91_sys_write(AT91_CKGR_UCKR, uckr & ~(clk->pmc_mask));

	do {
		cpu_relax();
	} while ((at91_sys_read(AT91_PMC_SR) & AT91_PMC_LOCKU) != is_on);
#endif	
}


static struct clk cpu_clk = {
	.name		= "cpu_clk",
	.rate_hz		= 528000000,						
	.parent		= &main_clk,
	.pmc_mask	= 0,
	.id			= 6,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
static struct clk g0_global_clk = {		//For Global
	.name		= "g0_global_clk",
	.parent		= 0,
	.pmc_mask	= 0,
	.id		    	= 7,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static struct clk g0_global_clk_half = {		//For Global
	.name		= "g0_global_clk_half",
	.parent		= &g0_global_clk,
	.pmc_mask	= 0,
	.id			= 7,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static struct clk g1_dram_clk = {		//For DRAM
	.name		= "g1_dram_clk",
	.rate_hz		= 180000000,	
	.parent		= &pll1,
	.pmc_mask	= 0,
	.id		= 8,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
#if 0
static struct clk g2_clk = {		//For USBPHY
	.name		= "g0_clk",
	.rate_hz		= 264000000,
	.parent		= &main_clk,
	.pmc_mask	= AT91_PMC_LOCKA,
	.id		= 9,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
static struct clk g3_clk = {		//For RX_BIST
	.name		= "g0_clk",
	.rate_hz		= 264000000,	
	.parent		= &pll2,
	.pmc_mask	= AT91_PMC_LOCKA,
	.id		= 10,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
#endif
static struct clk g4_sensor_clk = {		//For Sensor
	.name		= "g4_sensor_clk",

	.parent		= &g0_global_clk,
	.pmc_mask	= 0,
	.id		= 11,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
static struct clk g5_audio_clk = {		//For Audio
	.name		= "g5_audio_clk",
						
	.parent		= &g0_global_clk,
	.pmc_mask	= 0,
	.id		= 12,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
static struct clk g6_isp_clk = {		//For ISP
	.name		= "g6_isp_clk",
					
	.parent		= &g0_global_clk,
	.pmc_mask	= 0,
	.id		= 13,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};
/* Global Register: System Clock Control 1 0x6903 */


static struct clk vif_clk = {
	.name		= "vif_clk",
	.pmc_mask	= GBL_CLK_VIF,
	.type		= CLK_TYPE_CLK_CTL1,
};

static struct clk raw_f_clk = {
	.name		= "raw_f_clk",
	.pmc_mask	= GBL_CLK_RAW_F,
	.type		= CLK_TYPE_CLK_CTL1,
};

static struct clk raw_s0_clk = {
	.name		= "raw_s0_clk",
	.pmc_mask	= GBL_CLK_RAW_S0,
	.type		= CLK_TYPE_CLK_CTL1,
};

static struct clk raw_s1_clk = {
	.name		= "raw_s1_clk",
	.pmc_mask	= GBL_CLK_RAW_S1,
	.type		= CLK_TYPE_CLK_CTL1,
};

static struct clk raw_s2_clk = {
	.name		= "raw_s2_clk",
	.pmc_mask	= GBL_CLK_RAW_S2,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk isp_clk = {
	.name		= "isp_clk",
	.pmc_mask	= GBL_CLK_ISP,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk mci_clk = {
	.name		= "color_mci_clk",
	.pmc_mask	= GBL_CLK_COLOR_MCI,
	.type		= CLK_TYPE_CLK_CTL1,
};


static  struct clk gnr_clk = {
	.name		= "gnr_clk",
	.pmc_mask	= GBL_CLK_GNR,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk scaling_clk = {
	.name		= "scale_clk",
	.pmc_mask	= GBL_CLK_SCALE,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk icon_clk = {
	.name		= "icon_clk",
	.pmc_mask	= GBL_CLK_ICON,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk ibc_clk = {
	.name		= "ibc_clk",
	.pmc_mask	= GBL_CLK_IBC,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk ccir_clk = {
	.name		= "ccir_clk",
	.pmc_mask	= GBL_CLK_CCIR,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk dspy_clk = {
	.name		= "dspy_clk",
	.pmc_mask	= GBL_CLK_DSPY,
	.type		= CLK_TYPE_CLK_CTL1,
};


static  struct clk hdmi_clk = {
	.name		= "hdmi_clk",
	.pmc_mask	= GBL_CLK_HDMI,
	.type		= CLK_TYPE_CLK_CTL1,
};


static  struct clk tv_clk = {
	.name		= "tv_clk",
	.pmc_mask	= GBL_CLK_TV,
	.type		= CLK_TYPE_CLK_CTL1,
};


static  struct clk jpeg_clk = {
	.name		= "jpeg_clk",
	.pmc_mask	= GBL_CLK_JPG,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk h264_clk = {
	.name		= "h264_clk",
	.pmc_mask	= GBL_CLK_H264,
	.type		= CLK_TYPE_CLK_CTL1,
};

static  struct clk graphic_clk = {
	.name		= "gra_clk",
	.pmc_mask	= GBL_CLK_GRA,
	.type		= CLK_TYPE_CLK_CTL1,
};

static struct clk dma_clk = {
	.name		= "dma_clk",
	.pmc_mask	= GBL_CLK_DMA,
	.type		= CLK_TYPE_CLK_CTL1,
};

/*static*/  struct clk pwm_clk = {
	.name		= "pwm_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_PWM,
	.type		= CLK_TYPE_CLK_CTL1,
};

struct clk pspi_clk = {
	.name		= "pspi_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_PSPI,
	.type		= CLK_TYPE_CLK_CTL1,
};
	
struct clk sm_clk = {
	.name		= "sm_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_SM,
	.type		= CLK_TYPE_CLK_CTL1,
};


struct clk sd0_clk = {
	.name		= "sd0_clk",
	.parent		= &g0_global_clk,		
	.pmc_mask	= GBL_CLK_SD0 ,
	.type		= CLK_TYPE_CLK_CTL1,
};

struct clk sd1_clk = {
	.name		= "sd1_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= GBL_CLK_SD1 ,
	.type		= CLK_TYPE_CLK_CTL1,
};


struct clk sd2_clk = {
	.name		= "sd2_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= GBL_CLK_SD2,
    	.mode       	= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_CTL1,
};


struct clk sd3_clk = {
	.name		= "sd3_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= GBL_CLK_SD3,
    	.mode       	= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_CTL1,
};

struct clk audio_clk = {
	.name		= "audio_clk",
	.pmc_mask	= MMPF_SYS_CLK_AUD,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk audio2_clk = {	//For codec, adc df, adc hbfs
	.name		= "audio2_clk",
	.parent		= &audio_clk,
	.pmc_mask	= MMPF_SYS_CLK_ADC,
	.mode		= pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_SYS,
};



/* Global Register: System Clock Control 2 0x6904 */
const struct clk usb_clk = {
	.name		= "usb_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_USB,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk i2c_clk = {
	.name		= "i2c_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_I2C,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_CTL2,
};


const struct clk bootspi_clk = {
	.name		= "bs_spi_clk",
	.parent		= &g0_global_clk_half,	
	.pmc_mask	= GBL_CLK_BS_SPI,
	.mode		= pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_CTL2,
};


//static 
const struct clk gpio_clk = {
	.name		= "gpio_clk",
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_GPIO,
	.type		= CLK_TYPE_CLK_CTL2,
};


const struct clk aud_clk = {
	.name		= "aud_clk",
	//.parent		=&g0_global_clk,
	.parent		= &g0_global_clk_half,
	.pmc_mask	= GBL_CLK_AUD,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk adc_clk = {
	.name		= "adc_clk",
	//.parent		= &g0_global_clk_half,
	.parent		= &aud_clk,
	.pmc_mask	= GBL_CLK_ADC,
	.mode		= pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk dac_clk = {
	.name		= "dac_clk",
	//.parent		= &g0_global_clk_half,
	.parent		= &adc_clk,
	.pmc_mask	= GBL_CLK_DAC ,
	.mode		= pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_CTL2,
};


const struct clk irda_clk = {
	.name		= "irda_clk",
	.pmc_mask	= GBL_CLK_IRDA ,
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk ldc_clk = {
	.name		= "ldc_clk",
	.pmc_mask	= GBL_CLK_LDC ,
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk bayer_clk = {
	.name		= "bayer_clk",
	.pmc_mask	= GBL_CLK_BAYER ,
	.type		= CLK_TYPE_CLK_CTL2,
};

const struct clk mipi_rx_bist_clk = {
	.name		= "mipi_rx_bist_clk",
	.pmc_mask	= GBL_CLK_MIPI_RX_BIST ,
	.type		= CLK_TYPE_CLK_CTL2,
};






static void __clk_enable(struct clk *clk)
{
	pr_debug("%s On,  type:0x%x ",clk->name,clk->type);	

	if (clk->parent)
		__clk_enable(clk->parent);

	if (clk->users++ == 0 && clk->mode)
	{
		clk->mode(clk, 1);
	}
	pr_debug("%s: User = %d\n",	clk->name,clk->users);	


}

int clk_enable(struct clk *clk)
{
	unsigned long	flags;

	spin_lock_irqsave(&clk_lock, flags);
	__clk_enable(clk);
	spin_unlock_irqrestore(&clk_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

static void __clk_disable(struct clk *clk)
{
	BUG_ON(clk->users == 0);
	pr_debug("%s Off,  type:0x%x ",clk->name,clk->type);		

	if (--clk->users == 0 && clk->mode)
		clk->mode(clk, 0);
	if (clk->parent)
		__clk_disable(clk->parent);
	pr_debug("%s: User = %d\n",	clk->name,clk->users);	
}

void clk_disable(struct clk *clk)
{
	unsigned long	flags;

	spin_lock_irqsave(&clk_lock, flags);

	__clk_disable(clk);
	spin_unlock_irqrestore(&clk_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long	flags;
	unsigned long	rate;

	spin_lock_irqsave(&clk_lock, flags);
	for (;;) {
		rate = clk->rate_hz;
		if (rate || !clk->parent)
			break;
		clk = clk->parent;
	}
	spin_unlock_irqrestore(&clk_lock, flags);
	return rate;
}
EXPORT_SYMBOL(clk_get_rate);


#ifdef CONFIG_DEBUG_FS

static int ait_clk_show(struct seq_file *s, void *unused)
{
	struct clk	*clk;

	list_for_each_entry(clk, &clocks, node) {
		char	*state;

		state = "NA";

		seq_printf(s, "%-10s users=%2d %-3s %9ld Hz %s\n",
			clk->name, clk->users, state, clk_get_rate(clk),
			clk->parent ? clk->parent->name : "");
	}
	return 0;
}

static int ait_clk_open(struct inode *inode, struct file *file)
{
	return single_open(file, ait_clk_show, NULL);
}

static const struct file_operations ait_clk_operations = {
	.open		= ait_clk_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init ait_clk_debugfs_init(void)
{
	(void) debugfs_create_file("ait_clk", S_IFREG | S_IRUGO, NULL, NULL, &ait_clk_operations);

	return 0;
}
postcore_initcall(ait_clk_debugfs_init);

#endif

/*------------------------------------------------------------------------*/

/* Register a new clock */
static void __init ait_clk_add(struct clk *clk)
{
	list_add_tail(&clk->node, &clocks);

	clk->cl.con_id = clk->name;
	clk->cl.clk = clk;
	clkdev_add(&clk->cl);
}

int __init clk_register(struct clk *clk)
{
	pr_debug("reg clk: %s\r\n",clk->name);
	if (clk_is_peripheral(clk)) {
		if (!clk->parent)
			clk->parent = &g0_global_clk;
		clk->mode = pmc_periph_mode;
	}
	else if (clk_is_sys(clk)) {
		if (!clk->parent)
			clk->parent = &main_clk;
		clk->mode = pmc_sys_mode;
	}

	ait_clk_add(clk);

	return 0;
}

/*------------------------------------------------------------------------*/

static struct clk *const standard_pmc_clocks[] __initdata = {
	/* four primary clocks */
//	&clk32k,
	&main_clk,
//	&plla,

	/* MCK */
//	&mck

};
static struct clk *const vsnv3_sys_clocks[] __initdata = {
	&g0_global_clk_half,
	&gpio_clk,
	&isp_clk,
	&vif_clk,
 	&adc_clk,
	&dac_clk,
	&jpeg_clk,
	&scaling_clk,
#if 0
	&cpu_clk,
#endif
	&mci_clk,
	&graphic_clk,
	&icon_clk,
	&h264_clk,
	&i2c_clk,
	&dma_clk,
	&raw_f_clk,
	&bootspi_clk,
	//&dram_clk,
	//&cpu_peri_clk ,
	//&mipi_clk ,
	&pspi_clk ,
	&sd0_clk ,
	&sd1_clk ,
//#ifdef CONFIG_AIT_CHIP_MCR_V2_MP
#if CHIP == MCR_V2
	&sd2_clk ,
#endif	
	&pwm_clk,
	&ibc_clk,
	&usb_clk ,
};

MMP_UBYTE gbPostDiv[8] = {1, 2, 3, 6, 2, 4, 6, 12};


MMP_ERR MMPF_PLL_GetPLLFreq(MMPF_GROUP_SRC pllNo, MMP_ULONG *ulPllFreq)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_UBYTE bM, bN, PostDiv, bP;

    switch(pllNo)
    {
        case MMPF_PLL_ID_0:  // PLL0
            bM = gbPostDiv[(pGBL->GBL_DPLL0_M & 0x07)];
            bN = pGBL->GBL_DPLL0_N;
            PostDiv = gbPostDiv[(pGBL->GBL_DPPL0_PARAM[0x00] & 0x07)];
            *ulPllFreq = (MMP_ULONG)(24000 * bN) / (PostDiv * bM); 
            break;
        case MMPF_PLL_ID_1:  // PLL1
            bM = gbPostDiv[(pGBL->GBL_DPLL1_M & 0x07)];
            bN = pGBL->GBL_DPLL1_N;
            PostDiv = gbPostDiv[(pGBL->GBL_DPPL1_PARAM[0x00] & 0x07)];
            *ulPllFreq = (MMP_ULONG)(24000 * bN) / (PostDiv * bM); 
            break;
        case MMPF_PLL_ID_2:  // PLL2
            bM = gbPostDiv[(pGBL->GBL_DPLL2_M & 0x07)];
            bN = pGBL->GBL_DPLL2_N;
            PostDiv = gbPostDiv[(pGBL->GBL_DPPL2_PARAM[0x00] & 0x07)];
            *ulPllFreq = (MMP_ULONG)(24000 * bN) / (PostDiv * bM); 
            break;
        case MMPF_PLL_ID_PMCLK:  // PMCLK
            *ulPllFreq = 24000;
            break;
        case MMPF_PLL_ID_3:  // PLL3
            bM = pGBL->GBL_DPLL3_M + 1;
            bN = pGBL->GBL_DPLL3_N + 1;
            *ulPllFreq = (MMP_ULONG)(24576 * bN) / bM;
            break;
        case MMPF_PLL_ID_4:  // PLL4
            bM = pGBL->GBL_DPLL4_M + 1;
            bN = pGBL->GBL_DPLL4_N + 1;
            bP = gbPostDiv[(pGBL->GBL_DPPL4_PARAM[0x01] & 0x07)];
            *ulPllFreq = (MMP_ULONG)((24000 * bN) * 4) / (bM * bP);
            break;
        case MMPF_PLL_ID_5:  // PLL5
            //bM = gbPostDiv[((pGBL->GBL_DPPL5_PARAM[0x02] >> 2) & 0x07)];
            //bN = pGBL->GBL_DPLL5_N * 2;
            //bP = gbPostDiv[(pGBL->GBL_DPLL5_P & 0x07)];
            //*ulPllFreq = (MMP_ULONG)((24000 * bN)) / (bM * bP);
            *ulPllFreq = 400000;
            break;
        default:
            bN = pGBL->GBL_DPLL0_N;
            PostDiv = gbPostDiv[(pGBL->GBL_DPPL0_PARAM[0x00] & 0x07)];
            *ulPllFreq = (MMP_ULONG)(12000 * bN) / PostDiv; 
            break;
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PLL_GetGroup0Freq
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_PLL_MCRV2_GetGroup0Freq(MMP_ULONG *ulGroupFreq)
{
    //MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, ulGroupFreq);
	AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_UBYTE m_pll_src;
    MMP_UBYTE ubPostDiv = 0x0;
    MMP_ULONG pll_out_clk;

	if(pGBL->GBL_CLK_CTL & GRP_CLK_SEL_SRC1) { // source selection
	    m_pll_src = ((pGBL->GBL_CLK_SRC >> 4) & 0x07);
	}
	else {
	    m_pll_src = (pGBL->GBL_CLK_SRC & 0x07);
	}

    MMPF_PLL_GetPLLFreq(m_pll_src, &pll_out_clk);

    ubPostDiv = ((pGBL->GBL_CLK_DIV >> 1) & 0x1f) + 1;
   
    *ulGroupFreq = pll_out_clk/ubPostDiv;

    return MMP_ERR_NONE;
}


int __init ait_clock_init(unsigned long main_clock)
{
	unsigned freq;
	int i;
	unsigned long g0clk_khz;

	main_clk.rate_hz = main_clock;
	MMPF_PLL_MCRV2_GetGroup0Freq(&g0clk_khz);
	g0_global_clk.rate_hz = g0clk_khz*1000;
	g0_global_clk_half.rate_hz = g0_global_clk.rate_hz >>1;
	printk(KERN_INFO"g0clk = %u\n",g0clk_khz);
	/*
	 * MCK and CPU derive from one of those primary clocks.
	 * For now, assume this parentage won't change.
	 */
	freq = clk_get_rate(&cpu_clk);
	
	/* Register the PMC's standard clocks */
	for (i = 0; i < ARRAY_SIZE(standard_pmc_clocks); i++)
		clk_register(standard_pmc_clocks[i]);

	for (i = 0; i < ARRAY_SIZE(vsnv3_sys_clocks); i++)
		clk_register(vsnv3_sys_clocks[i]);

	printk(KERN_INFO"Clocks: CPU %u MHz, master %u MHz, main %u.%03u MHz\n",
		freq / 1000000, (unsigned) clk_get_rate(& g0_global_clk)/ 1000000,
		(unsigned) main_clock / 1000000,
		((unsigned) main_clock % 1000000) / 1000);

	return 0;
}
/*
 * Several unused clocks may be active.  Turn them off.
 */
static int __init ait_clock_reset(void)
{
	unsigned long pcdr = 0;
	unsigned long scdr = 0;
	struct clk *clk;

	list_for_each_entry(clk, &clocks, node) {
		if (clk->users > 0)
			continue;

		if (clk->mode == pmc_periph_mode)
			pcdr |= clk->pmc_mask;

		if (clk->mode == pmc_sys_mode)
			scdr |= clk->pmc_mask;

		pr_debug("Clocks: disable unused %s\n", clk->name);
	}

	return 0;
}
late_initcall(ait_clock_reset);

