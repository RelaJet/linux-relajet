/*
 * 
 * Modify from linux/arch/arm/mach-at91/gpio.c
 *
 * Copyright (C) 2005 HP Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/io.h>

#include <mach/hardware.h>

#include "generic.h"

#include "includes_fw.h"
#include "reg_retina.h"
#include "lib_retina.h"

#include "mmpf_pio.h"


struct ait_gpio_chip {
	struct gpio_chip	    chip;
	struct ait_gpio_chip	*next;		/* Bank sharing same clock */
	struct ait_gpio_bank	*bank;		/* Bank definition */
	void __iomem		    *regbase;	/* Base of register bank */
};

#define to_ait_gpio_chip(c) container_of(c, struct ait_gpio_chip, chip)

static void ait_gpiolib_dbg_show(struct seq_file *s, struct gpio_chip *chip);
static void ait_gpiolib_set(struct gpio_chip *chip, unsigned offset, int val);
static int ait_gpiolib_get(struct gpio_chip *chip, unsigned offset);
static int ait_gpiolib_direction_output(struct gpio_chip *chip,
					 unsigned offset, int val);
static int ait_gpiolib_direction_input(struct gpio_chip *chip,
					unsigned offset);

#define AIT_GPIO_CHIP(name, base_gpio, nr_gpio)			\
	{								\
		.chip = {						\
			.label		  = name,			\
			.direction_input  = ait_gpiolib_direction_input, \
			.direction_output = ait_gpiolib_direction_output, \
			.get		  = ait_gpiolib_get,		\
			.set		  = ait_gpiolib_set,		\
			.dbg_show	  = ait_gpiolib_dbg_show,	\
			.base		  = base_gpio,			\
			.ngpio		  = nr_gpio,			\
		},							\
	}

static struct ait_gpio_chip gpio_chip[] = {
    AIT_GPIO_CHIP("GPIOA", 0+PIN_BASE, 32),
    AIT_GPIO_CHIP("GPIOB", 0x20+PIN_BASE, 32),
    AIT_GPIO_CHIP("GPIOC", 0x40+PIN_BASE, 2),
};

static int gpio_banks;

static inline unsigned pin_to_mask(unsigned pin)
{
	pin -= PIN_BASE;
	return 1 << (pin & 0x1F);
}


/*
 * mux the pin to the gpio controller (instead of "A" or "B" peripheral), and
 * configure it for an input.
 */
int __init_or_module ait_set_gpio_input(unsigned pin, int use_pullup)
{
	MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode((MMPF_PIO_REG) pin, 0, MMPF_OS_LOCK_CTX_ISR);

	return 0;
}
EXPORT_SYMBOL(ait_set_gpio_input);


/*
 * mux the pin to the gpio controller (instead of "A" or "B" peripheral),
 * and configure it for an output.
 */
int __init_or_module ait_set_gpio_output(unsigned pin, int value)
{
	MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode((MMPF_PIO_REG) pin, 1, MMPF_OS_LOCK_CTX_ISR);

	return 0;
}
EXPORT_SYMBOL(ait_set_gpio_output);

/*
 * assuming the pin is muxed as a gpio output, set its value.
 */
int ait_set_gpio_value(unsigned pin, int value)
{
	return MMPF_PIO_SetData((MMPF_PIO_REG) pin, (MMP_BOOL)value, MMPF_OS_LOCK_CTX_ISR);
}
EXPORT_SYMBOL(ait_set_gpio_value);


/*
 * read the pin's value (works even if it's not muxed as a gpio).
 */
int ait_get_gpio_value(unsigned pin)
{
	MMP_UBYTE value;

	MMPF_PIO_GetData((MMPF_PIO_REG) pin, &value);

	return value;
}
EXPORT_SYMBOL(ait_get_gpio_value);

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_PM

static u32 wakeups[MAX_GPIO_BANKS];

static int gpio_irq_set_wake(struct irq_data *d, unsigned state)
{
	unsigned	mask = pin_to_mask(d->irq);
	unsigned	bank = ((d->irq - PIN_BASE) >> 5);

	if (unlikely(bank >= MAX_GPIO_BANKS))
		return -EINVAL;

	if (state)
		wakeups[bank] |= mask;
	else
		wakeups[bank] &= ~mask;

	irq_set_irq_wake(gpio_chip[bank].bank->id, state);

	return 0;
}

#else
#define gpio_irq_set_wake	NULL
#endif


/* Several AIC controller irqs are dispatched through this GPIO handler.
 * To use any AT91_PIN_* as an externally triggered IRQ, first call
 * ait_set_gpio_input() then maybe enable its glitch filter.
 * Then just request_irq() with the pin ID; it works like any ARM IRQ
 * handler, though it always triggers on rising and falling edges.
 *
 * Alternatively, certain pins may be used directly as IRQ0..IRQ6 after
 * configuring them with at91_set_a_periph() or at91_set_b_periph().
 * IRQ0..IRQ6 should be configurable, e.g. level vs edge triggering.
 */

static void ait_gpio_fw_irq_callback(MMP_ULONG pin, ...);
static void gpio_irq_mask(struct irq_data *d)
{
    MMPF_PIO_EnableInterrupt((MMPF_PIO_REG)(d->irq - PIN_BASE), MMP_FALSE, 0,
            0, MMPF_OS_LOCK_CTX_ISR);
    //at91_sys_write(AT91_AIC_IDCR, 1 << 16);
}

static void gpio_irq_unmask(struct irq_data *d)
{
    MMPF_PIO_EnableInterrupt((MMPF_PIO_REG)(d->irq - PIN_BASE), MMP_TRUE, 0,
            ait_gpio_fw_irq_callback, MMPF_OS_LOCK_CTX_ISR);

	//MMPF_PIO_EnableInterrupt(3, MMP_TRUE, 0, 0, MMPF_OS_LOCK_CTX_ISR);

	//at91_sys_write(AT91_AIC_IECR, 1 << 16);
}

static int gpio_irq_type(struct irq_data *d, unsigned type)
{
	return 0;
}

static struct irq_chip gpio_irqchip = {
	.name		    = "GPIO",
	.irq_disable	= gpio_irq_mask,
	.irq_mask	    = gpio_irq_mask,
	.irq_unmask	    = gpio_irq_unmask,
	.irq_set_type	= gpio_irq_type,
	.irq_set_wake	= gpio_irq_set_wake,
};

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_DEBUG_FS

static int ait_gpio_show(struct seq_file *s, void *unused)
{
	int bank, j;

	/* print heading */
	seq_printf(s, "Pin\t");
	for (bank = 0; bank < gpio_banks; bank++) {
		seq_printf(s, "PIO%c\t", 'A' + bank);
	};
	seq_printf(s, "\n\n");

	/* print pin status */
	for (j = 0; j < 32; j++) {
		seq_printf(s, "%i:\t", j);

		for (bank = 0; bank < gpio_banks; bank++) {
			seq_printf(s, "\t");
		}

		seq_printf(s, "\n");
	}

	return 0;
}

static int ait_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, ait_gpio_show, NULL);
}

static const struct file_operations ait_gpio_operations = {
	.open		= ait_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init ait_gpio_debugfs_init(void)
{
	/* /sys/kernel/debug/at91_gpio */
	(void) debugfs_create_file("ait_gpio", S_IFREG | S_IRUGO, NULL, NULL, &ait_gpio_operations);

	return 0;
}
postcore_initcall(ait_gpio_debugfs_init);

#endif

/*--------------------------------------------------------------------------*/

static int gpio_cur_pin;
static void ait_gpio_fw_irq_callback(MMP_ULONG pin, ...)
{
    gpio_cur_pin = pin;
}

#if 0 //temp fixme
struct gpio_button_data {
    struct gpio_keys_button *button;
    struct input_dev *input;
    struct timer_list timer;
    struct work_struct work;
    int timer_debounce; /* in msecs */
    bool disabled;
};
#endif
static void ait_gpio_irq_flow_handler(unsigned int irq, struct irq_desc *desc)
{
    //struct gpio_button_data *bdata = NULL;

    MMPF_PIO_ISR();

    #if 0 //temp fixme
    if (desc->action)
        bdata = (struct gpio_button_data *)(desc->action->dev_id);

    if (bdata && bdata->button)
        bdata->button->value = gpio_cur_pin;
    #endif

    handle_simple_irq(irq, desc);
}

/*
 * This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

/*
 * Called from the processor-specific init to enable GPIO interrupt support.
 */
void __init ait_gpio_irq_setup(void)
{
    irq_set_lockdep_class(AIC_SRC_GPIO, &gpio_lock_class);

    irq_set_chip_and_handler(AIC_SRC_GPIO, &gpio_irqchip, ait_gpio_irq_flow_handler);

    RTNA_AIC_IRQ_En(AITC_BASE_AIC, AIC_SRC_GPIO);
}

/* gpiolib support */
static int ait_gpiolib_direction_input(struct gpio_chip *chip,
					unsigned offset)
{
	MMPF_PIO_EnableGpioMode(chip->base+offset, 1, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode(chip->base+offset, 0, MMPF_OS_LOCK_CTX_ISR);
	return 0;
}

static int ait_gpiolib_direction_output(struct gpio_chip *chip,
					 unsigned offset, int val)
{
    MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) offset, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode(offset, 1, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_SetData(offset, val, MMPF_OS_LOCK_CTX_ISR);

	return 0;
}

static int ait_gpiolib_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned char returnValue;

	MMPF_PIO_GetData(offset, &returnValue);

	return returnValue;
}

static void ait_gpiolib_set(struct gpio_chip *chip, unsigned offset, int val)
{
	MMPF_PIO_SetData(offset,val, MMPF_OS_LOCK_CTX_ISR);
}

static void ait_gpiolib_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	int i;

	for (i = 0; i < chip->ngpio; i++) {
		const char *gpio_label;

		gpio_label = gpiochip_is_requested(chip, i);
		if (gpio_label) {
			seq_printf(s, "[%s] GPIO%s%d: ", gpio_label, chip->label, i);
		}
	}
}

/*
 * Called from the processor-specific init to enable GPIO pin support.
 */
void __init ait_gpio_init(struct ait_gpio_bank *data, int nr_banks)
{
	unsigned		i;
	struct ait_gpio_chip *ait_gpio;

	BUG_ON(nr_banks > MAX_GPIO_BANKS);

    //For I2S
//	MMPF_PIO_EnableGpioMode(AIT_GPIO_I2S_0, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
//	MMPF_PIO_EnableGpioMode(AIT_GPIO_I2S_1, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
//	MMPF_PIO_EnableGpioMode(AIT_GPIO_I2S_2, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
//	MMPF_PIO_EnableGpioMode(AIT_GPIO_I2S_3, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);

	gpio_banks = nr_banks;

	for (i = 0; i < nr_banks; i++) {
	    ait_gpio = &gpio_chip[i];

		gpiochip_add(&ait_gpio->chip);
	}

}
