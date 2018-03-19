/*
 * linux/arch/arm/mach-ait/clock.c
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


static struct clk main_clk = {
    .name       = "main",
    .rate_hz    = 12000000,
    .pmc_mask   = 0,
    .id         = 1,
    .type       = CLK_TYPE_PRIMARY,
};

static struct clk g0_global_clk = {
	.name		= "g0_global_clk",
	.parent		= 0,
	.pmc_mask	= 0,
	.id		    = 7,
	.type		= CLK_TYPE_PRIMARY,
};

static struct clk gpio_clk = {
	.name		= "gpio_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_GPIO,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk audio_clk = {
	.name		= "audio_clk",
	.pmc_mask	= MMPF_SYS_CLK_AUD,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk i2c_clk = {
	.name		= "i2c_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_I2CM,
	.mode		= pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};
struct clk bootspi_clk = {
	.name		= "bootspi_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_BS_SPI,
    .mode       = pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};
//struct clk cpu_peri_clk = {
//	.name		= "cpu_peri_clk",
//	.parent		= &g0_global_clk,
//	.pmc_mask	= ,
//    .mode       = pmc_sys_mode,
//	.type		= CLK_TYPE_CLK_SYS,
//};

struct clk pspi_clk = {
	.name		= "pspi_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_PSPI ,
    .mode       = pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk sd0_clk = {
	.name		= "sd0_clk",
	.parent		= &g0_global_clk,		
	.pmc_mask	= MMPF_SYS_CLK_SD0,
    .mode       = pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk sd1_clk = {
	.name		= "sd1_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_SD1,
    .mode       = pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk sd2_clk = {
	.name		= "sd2_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_SD2,
    .mode       = pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

static struct clk pwm_clk = {
	.name		= "pwm_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_PWM ,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk usb_clk = {
	.name		= "usb_clk",
	.parent		= &g0_global_clk,
	.pmc_mask	= MMPF_SYS_CLK_USB ,
   	.mode       = pmc_sys_mode,
	.type		= CLK_TYPE_CLK_SYS,
};

static struct clk sm_clk = {
	.name		= "sm_clk",
	.pmc_mask	= MMPF_SYS_CLK_SM,
	.type		= CLK_TYPE_CLK_SYS,
};

struct clk audio2_clk = {	//For codec, adc df, adc hbfs
	.name		= "audio2_clk",
	.parent		= &audio_clk,
	.pmc_mask	= MMPF_SYS_CLK_ADC,
	.mode		= pmc_sys_mode,	
	.type		= CLK_TYPE_CLK_SYS,
};


static void pmc_sys_mode(struct clk *clk, int is_on)
{
    #ifdef CLK_DEBUG
    pr_info("Sys clock %s %s mod %d\r\n", clk->name, is_on?"on":"off",
            clk->pmc_mask);
    #endif

    if (clk->type != CLK_TYPE_CLK_SYS) {
        return;
    }

    MMPF_SYS_EnableClock(clk->pmc_mask, is_on? MMP_TRUE: MMP_FALSE);
}

static void __clk_enable(struct clk *clk)
{
    #ifdef CLK_DEBUG
	pr_info("Clock %s Enable.\r\n",clk->name);
    #endif

	if (clk->parent)
		__clk_enable(clk->parent);

	if (clk->users++ == 0 && clk->mode)
	{
		clk->mode(clk, 1);
	}
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
    #ifdef CLK_DEBUG
	pr_info("Clock %s Disable.\r\n",clk->name);
    #endif

	if (--clk->users == 0 && clk->mode)
		clk->mode(clk, 0);

	if (clk->parent)
		__clk_disable(clk->parent);
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
    #ifdef CLK_DEBUG
	pr_info("reg clk: %s\r\n",clk->name);
    #endif

	if (clk_is_peripheral(clk)) {
		if (!clk->parent)
			clk->parent = &g0_global_clk;
		clk->mode = NULL;
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

static struct clk *const sys_clocks[] __initdata = {
    &main_clk,

    &gpio_clk,
    &audio_clk,
    &i2c_clk,
    &bootspi_clk,
    //&cpu_peri_clk ,
    &pspi_clk ,
    &sd0_clk ,
    &sd1_clk ,
    &sd2_clk ,
    &pwm_clk,
    &usb_clk ,
    &sm_clk,
    &audio2_clk,
};


int __init ait_clock_init(unsigned long main_clock)
{
	int i;
	MMP_ULONG freq_khz = 0;

	main_clk.rate_hz = main_clock;

	MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &freq_khz);
	g0_global_clk.rate_hz = freq_khz * 1000;

	for (i = 0; i < ARRAY_SIZE(sys_clocks); i++)
	    ait_clk_add(sys_clocks[i]);

	pr_info("Clocks: master %u MHz, main %u.%03u MHz\n",
		(unsigned) clk_get_rate(&g0_global_clk) / 1000000,
		(unsigned) main_clock / 1000000,
		((unsigned) main_clock % 1000000) / 1000);

	return 0;
}

/*
 * Several unused clocks may be active.  Turn them off.
 */
static int __init ait_clock_reset(void)
{
//	unsigned long pcdr = 0;
//	unsigned long scdr = 0;
//	struct clk *clk;
//
//	list_for_each_entry(clk, &clocks, node) {
//		if (clk->users > 0)
//			continue;
//
//		if (clk->mode == pmc_periph_mode)
//			pcdr |= clk->pmc_mask;
//
//		if (clk->mode == pmc_sys_mode)
//			scdr |= clk->pmc_mask;
//
//		pr_debug("Clocks: disable unused %s\n", clk->name);
//	}

	return 0;
}
late_initcall(ait_clock_reset);
