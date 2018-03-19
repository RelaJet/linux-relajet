/*
 * linux/arch/arm/mach-ait/generic.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clkdev.h>
/* DRAM Reserve */
extern void __init ait_dram_reserve_cpub(void);
extern void __init ait_dram_reserve_i2cs(void);
extern void __init ait_dram_reserve(phys_addr_t paddr, phys_addr_t size);

 /* Map io */
extern void __init ait_soc_map_io(void);

 /* Processors */
extern void __init ait_soc_initialize(unsigned long main_clock);

 /* Interrupts */
extern void __init ait_soc_init_irq_default(void);
extern void __init ait_aic_init(unsigned int priority[]);

 /* Timer */
struct sys_timer;
extern struct sys_timer ait_pit_timer;

/*
 * function to specify the clock of the default console. As we do not
 * use the device/driver bus, the dev_name is not intialize. So we need
 * to link the clock to a specific con_id only "usart"
 */
extern void __init ait_arch_set_console_clock(int id);

extern int __init ait_clock_init(unsigned long main_clock);

 /* Power Management */
extern void ait_irq_suspend(void);
extern void ait_irq_resume(void);
struct ait_gpio_bank {
	unsigned short id;      /* peripheral ID */
	unsigned char* name;
	struct clk *clock;      /* associated clock */
};
extern void __init ait_gpio_init(struct ait_gpio_bank *, int nr_banks);
void __init ait_gpio_irq_setup(void);

extern void (*ait_arch_reset)(void);
