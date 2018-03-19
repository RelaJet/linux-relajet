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

#ifndef __ASM_ARCH_AIT_GPIO_H
#define __ASM_ARCH_AIT_GPIO_H

#include <linux/kernel.h>
#include <asm/irq.h>

#define PIN_BASE		    NR_AIC_IRQS

#if (CHIP == VSN_V3) // mcrv2 in mcrv2_gpio.h
#define MAX_GPIO_BANKS		(3)
#endif

#define NR_BUILTIN_GPIO		(PIN_BASE + (MAX_GPIO_BANKS * 32))



#ifndef __ASSEMBLY__

extern int __init_or_module ait_set_gpio_input(unsigned pin, int use_pullup);
extern int __init_or_module ait_set_gpio_output(unsigned pin, int value);

/* callable at any time */
extern int ait_set_gpio_value(unsigned pin, int value);
extern int ait_get_gpio_value(unsigned pin);

/*-------------------------------------------------------------------------*/

/* wrappers for "new style" GPIO calls. the old AT91-specific ones should
 * eventually be removed (along with this errno.h inclusion), and the
 * gpio request/free calls should probably be implemented.
 */

#include <asm/errno.h>

#if defined(CONFIG_AIT_CHIP_MCR_V2) 
#define ARCH_NR_GPIOS (127)
#elif defined( CONFIG_AIT_CHIP_MCR_V2_MP)
#define ARCH_NR_GPIOS (128 + 8 + 6 + 1) // 8 for sadc // 6 for pwm0~2. // 1 for hw watchdog
#else
#define ARCH_NR_GPIOS (66)
#endif

#define gpio_to_irq(gpio) (NR_AIC_IRQS+gpio)
#define irq_to_gpio(irq)  (irq>32?(irq-NR_AIC_IRQS):-1)

#endif	/* __ASSEMBLY__ */

#endif
