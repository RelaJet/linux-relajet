/*
 * arch/arm/mach-at91/include/mach/gpio.h
 *
 *  Copyright (C) 2005 HP Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_AT91RM9200_GPIO_H
#define __ASM_ARCH_AT91RM9200_GPIO_H

#include <linux/kernel.h>
#include <asm/irq.h>


#define PIN_BASE		NR_AIC_IRQS

#define MAX_GPIO_BANKS		(5+1+1)
#define NR_BUILTIN_GPIO		(PIN_BASE + (MAX_GPIO_BANKS * 32))

/* these pin numbers double as IRQ numbers, like AT91xxx_ID_* values */

#ifndef __ASSEMBLY__
/* setup setup routines, called from board init or driver probe() */
extern int __init_or_module vsnv3__set_GPIO_periph(unsigned pin, int use_pullup);
extern int __init_or_module vsnv3__set_A_periph(unsigned pin, int use_pullup);
extern int __init_or_module vsnv3__set_B_periph(unsigned pin, int use_pullup);
extern int __init_or_module vsnv3__set_gpio_input(unsigned pin, int use_pullup);
extern int __init_or_module vsnv3_set_gpio_output(unsigned pin, int value);
extern int __init_or_module vsnv3__set_deglitch(unsigned pin, int is_on);
extern int __init_or_module vsnv3__set_multi_drive(unsigned pin, int is_on);

/* callable at any time */
extern int vsnv3__set_gpio_value(unsigned pin, int value);
extern int vsnv3__get_gpio_value(unsigned pin);

/* callable only from core power-management code */
extern void vsnv3_gpio_suspend(void);
extern void vsnv3_gpio_resume(void);

void ait_disable_gpio(unsigned pin);

/*-------------------------------------------------------------------------*/

/* wrappers for "new style" GPIO calls. the old AT91-specific ones should
 * eventually be removed (along with this errno.h inclusion), and the
 * gpio request/free calls should probably be implemented.
 */

#include <asm/errno.h>

//#define gpio_to_irq(gpio) xxxxxxxxxxx (AITVSNV3_ID_GPIO )
//#define irq_to_gpio(irq)  (irq>32?(irq-NR_AIC_IRQS):-1)

#endif	/* __ASSEMBLY__ */

#endif
