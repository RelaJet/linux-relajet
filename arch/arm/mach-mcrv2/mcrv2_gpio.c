/*
 * 
 * Modify from linux/arch/arm/mach-vsnv3/mcrv2_gpio.c
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
#include <mach/mcrv2_gpio.h>

#include "generic.h"

#include <mach/mmpf_pio.h>
#include <mach/mmpf_saradc.h>
#include <mach/mmpf_pwm.h>
#include <mach/mmpf_wd.h>

struct ait_gpio_chip {
	struct gpio_chip	chip;
	struct ait_gpio_chip	*next;		/* Bank sharing same clock */
	struct ait_gpio_bank	*bank;		/* Bank definition */
	void __iomem		*regbase;	/* Base of register bank */
//	int pin;
};

struct ait_pwm_attr {
  MMPF_PIO_REG gpio;  
  MMPF_PWMPIN  pwm ;  
  int freq ;
  int duty ;
} ;

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
#define BNAK_SIZE 32
#define SADC_BASE  0x80
#define SADC_CHANS 8
#define PWM_BASE (SADC_BASE + SADC_CHANS)
#define PWM_CHANS 3
#define DOG_BASE ( PWM_BASE + PWM_CHANS * 2)
#define DOG_CHANS 1

static struct ait_gpio_chip gpio_chip[] = {
   // AIT_GPIO_CHIP("GPIO0", 0, 127)
    AIT_GPIO_CHIP("GPIO0_31", 0, BNAK_SIZE),
    AIT_GPIO_CHIP("GPIO32_63", 0x20, BNAK_SIZE),
    AIT_GPIO_CHIP("GPIO64_95", 0x40, BNAK_SIZE),
    AIT_GPIO_CHIP("GPIO96_127", 0x60, BNAK_SIZE),
    AIT_GPIO_CHIP("SADC1_SADC8", SADC_BASE, SADC_CHANS),
    /*
    off = 0 -> pwm0 freq
    off = 1 -> pwm1 freq
    off = 2 -> pwm2 freq
    off = 3 -> pwm0 duty
    off = 4 -> pwm1 duty
    off = 5 -> pwm2 duty
    */
    AIT_GPIO_CHIP("PWM0_PWM2", PWM_BASE, PWM_CHANS*2), // for Freq & Duty pair
    /*
    set watdog window by gpio
    */
    AIT_GPIO_CHIP("HWDOG", DOG_BASE, 1),
};

static int gpio_banks;

static struct ait_pwm_attr ait_pwm[] = {
  [0] = {MMPF_PIO_REG_GPIO3,MMPF_PWM0_PIN_AGPIO1, 10000,0},
  [1] = {MMPF_PIO_REG_GPIO4,MMPF_PWM1_PIN_AGPIO2, 10000,0},
  [2] = {MMPF_PIO_REG_GPIO5,MMPF_PWM2_PIN_AGPIO3, 10000,0}
} ;

static inline void __iomem *pin_to_controller(unsigned pin)
{
	pin -= PIN_BASE;
	pin /= BNAK_SIZE;
	if (likely(pin < gpio_banks))
		return gpio_chip[pin].regbase;
	return NULL;
}

static inline unsigned pin_to_mask(unsigned pin)
{
	//pin -= PIN_BASE;
	//return 1 << (pin % 66);
	return 1 << (pin & 0x1F);
}


/*--------------------------------------------------------------------------*/

/* Not all hardware capabilities are exposed through these calls; they
 * only encapsulate the most common features and modes.  (So if you
 * want to change signals in groups, do it directly.)
 *
 * Bootloaders will usually handle some of the pin multiplexing setup.
 * The intent is certainly that by the time Linux is fully booted, all
 * pins should have been fully initialized.  These setup calls should
 * only be used by board setup routines, or possibly in driver probe().
 *
 * For bootloaders doing all that setup, these calls could be inlined
 * as NOPs so Linux won't duplicate any setup code
 */

/*
 *  disable gpio pin for other peripheral usage
 */
void ait_disable_gpio(unsigned pin)
{
	MMPF_PIO_EnableGpioMode(pin,0,MMPF_OS_LOCK_CTX_BYPASS); //TX
}


/*
 * mux the pin to the "GPIO" peripheral role.
 */
int __init_or_module vsnv3_set_gpio_periph(unsigned pin, int use_pullup)
{

	void __iomem	*pio = pin_to_controller(pin);
//	unsigned	mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;
	
	return 0;
}
EXPORT_SYMBOL(vsnv3_set_gpio_periph);


/*
 * mux the pin to the gpio controller (instead of "A" or "B" peripheral), and
 * configure it for an input.
 */
int __init_or_module ait_set_gpio_input(unsigned pin, int use_pullup)
{

	void __iomem	*pio = pin_to_controller(pin);
//	unsigned	mask = pin_to_mask(pin);

	pr_debug("%s: pin = %d use_pullup = %d\n",__func__,pin,use_pullup);


	if (!pio)
		return -EINVAL;

	MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode((MMPF_PIO_REG) pin, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
	
	
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
	// DEL : dummy repeat
	//MMPF_PIO_EnableOutputMode((MMPF_PIO_REG) pin, 1, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_SetData(pin, value, MMPF_OS_LOCK_CTX_ISR);

	return 0;
}
EXPORT_SYMBOL(ait_set_gpio_output);


/*
 * enable/disable the glitch filter; mostly used with IRQ handling.
 */
int __init_or_module ait_set_deglitch(unsigned pin, int is_on)
{

	void __iomem	*pio = pin_to_controller(pin);
//	unsigned	mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(ait_set_deglitch);

/*
 * enable/disable the multi-driver; This is only valid for output and
 * allows the output pin to run as an open collector output.
 */
int __init_or_module ait_set_multi_drive(unsigned pin, int is_on)
{

	void __iomem	*pio = pin_to_controller(pin);
//	unsigned	mask = pin_to_mask(pin);

	if (!pio)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(ait_set_multi_drive);

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
static u32 backups[MAX_GPIO_BANKS];
static void gpio_irq_unmask(struct irq_data *d);

static void MMPF_PIO_WakeUpEnable(MMP_BOOL bEnable)
{
    AITPS_GPIO_CNT  pGPIOCTL = AITC_BASE_GPIOCTL;

    if (bEnable)
        pGPIOCTL->GPIO_WAKEUP_INT_EN = GPIO_WAKEUP_MODE_EN;
    else
        pGPIOCTL->GPIO_WAKEUP_INT_EN &= ~(GPIO_WAKEUP_MODE_EN);
}

static int gpio_irq_set_wake(struct irq_data *d, unsigned state)
{
	unsigned pin = irq_to_gpio(d->irq);
	unsigned	mask = pin_to_mask(pin);
	unsigned	bank = pin / BNAK_SIZE;

	BUG_ON(pin==-1);

	if (unlikely(bank >= MAX_GPIO_BANKS))
		return -EINVAL;

	if (state)
		wakeups[bank] |= mask;
	else
		wakeups[bank] &= ~mask;

	irq_set_irq_wake(d->irq, state);

	enable_irq_wake(AIC_SRC_GPIO);

	return 0;
}

void vsnv3_gpio_suspend(void)
{
	unsigned	mask;
	int i;
	int pin;	
	AITPS_GPIO  pGPIO = AITC_BASE_GPIO;
	
	MMPF_PIO_WakeUpEnable(1);

	for (i = 0; i < gpio_banks; i++) {
		int pin =i*BNAK_SIZE;
		mask = wakeups[i];

		if (!wakeups[i] && gpio_chip[i].bank->clock)
			clk_disable(gpio_chip[i].bank->clock);

		
		while(mask)
		{
			if(mask&1)
			{
				printk(KERN_INFO"Bank %d GPIO%d wake interrupt enable\n", i, pin);

				//MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(pin ),MMPF_PIO_TRIGMODE_EDGE_L2H, MMP_TRUE,0);
				MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(pin ),MMPF_PIO_TRIGMODE_EDGE_H2L, MMP_TRUE,0);
			}
			mask>>=1;
			pin++;
		}

		backups[i] = pGPIO->GPIO_INT_CPU_EN[i];		

	}
}

void vsnv3_gpio_resume(void)
{

	unsigned	wake_mask,bakeup_mask;
	MMPF_PIO_WakeUpEnable(0);

	int i;
	for (i = 0; i < gpio_banks; i++) {
		wake_mask = wakeups[i];
		bakeup_mask = backups[i];
		int pin = i*BNAK_SIZE;

		if (!wakeups[i] && gpio_chip[i].bank->clock)
			clk_enable(gpio_chip[i].bank->clock);
		
		while(wake_mask)
		{
			if(wake_mask&1)
			{
				printk(KERN_INFO"Bank %d GPIO%d wake interrupt enable\n", i, pin);

				MMPF_PIO_EnableTrigMode(pin, MMPF_PIO_TRIGMODE_EDGE_L2H, MMP_FALSE, MMP_FALSE);
			}
			wake_mask>>=1;
			pin++;
		}
		pin = i*BNAK_SIZE;
		while(bakeup_mask)
		{
			if(bakeup_mask&1)
			{
				printk(KERN_INFO"Bank %d GPIO%d wake interrupt enable\n", i, pin);

				MMPF_PIO_EnableInterrupt((MMPF_PIO_REG)pin, MMP_TRUE, 0, NULL, MMPF_OS_LOCK_CTX_ISR);
			}
			bakeup_mask>>=1;
			pin++;
		}
	}
}

#else
#define gpio_irq_set_wake	NULL
#endif


/* Several AIC controller irqs are dispatched through this GPIO handler.
 * To use any AT91_PIN_* as an externally triggered IRQ, first call
 * at91_set_gpio_input() then maybe enable its glitch filter.
 * Then just request_irq() with the pin ID; it works like any ARM IRQ
 * handler, though it always triggers on rising and falling edges.
 *
 * Alternatively, certain pins may be used directly as IRQ0..IRQ6 after
 * configuring them with at91_set_a_periph() or at91_set_b_periph().
 * IRQ0..IRQ6 should be configurable, e.g. level vs edge triggering.
 */

static void gpio_irq_mask(struct irq_data *d)
{
    MMPF_PIO_EnableInterrupt((MMPF_PIO_REG)(d->irq - PIN_BASE), MMP_FALSE, 0,
            0, MMPF_OS_LOCK_CTX_ISR);
    //at91_sys_write(AT91_AIC_IDCR, 1 << 16);
}

static void gpio_irq_unmask(struct irq_data *d)
{
    MMPF_PIO_EnableInterrupt((MMPF_PIO_REG)(d->irq - PIN_BASE), MMP_TRUE, 0,
            NULL, MMPF_OS_LOCK_CTX_ISR);
//	unsigned	mask = pin_to_mask(d->irq);

	//if (pio)
	//	__raw_writel(mask, pio + PIO_IER);
	//MMPF_PIO_EnableInterrupt(3, MMP_TRUE, 0, 0, MMPF_OS_LOCK_CTX_ISR);
	/* Enable interrupt on AIC */
	//at91_sys_write(AT91_AIC_IECR, 1 << 16);
}

static int gpio_irq_type(struct irq_data *d, unsigned type)
{

	switch(type)
	{
		case IRQ_TYPE_EDGE_FALLING:
			MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(d->irq - PIN_BASE),MMPF_PIO_TRIGMODE_EDGE_H2L, MMP_TRUE,0);

			break;
		case IRQ_TYPE_EDGE_RISING:

			MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(d->irq - PIN_BASE),MMPF_PIO_TRIGMODE_EDGE_L2H, MMP_TRUE,0);
	
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(d->irq - PIN_BASE),MMPF_PIO_TRIGMODE_LEVEL_H, MMP_TRUE,0);
			break;
		case IRQ_TYPE_EDGE_BOTH:

			MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(d->irq - PIN_BASE),MMPF_PIO_TRIGMODE_EDGE_H2L, MMP_TRUE,0);
			MMPF_PIO_EnableTrigMode((MMPF_PIO_REG)(d->irq - PIN_BASE),MMPF_PIO_TRIGMODE_EDGE_L2H, MMP_TRUE,0);

			break;
			
		default:
			printk(KERN_ERR"gpio_irq_type: trigger type %d undefined!!",type);
			BUG_ON(1);
			break;
	}

	return 0;
}

void	ait_aic_irq_enable(struct irq_data *data)
{
	MMPF_PIO_REG pin = (MMPF_PIO_REG)(data->irq - PIN_BASE);
	MMPF_PIO_EnableGpioMode( pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	MMPF_PIO_EnableOutputMode( pin, 0, MMPF_OS_LOCK_CTX_ISR);

	MMPF_PIO_EnableInterrupt(pin, MMP_TRUE, 0,            NULL, MMPF_OS_LOCK_CTX_ISR);
}

static struct irq_chip gpio_irqchip = {
	.name		= "AIT_GPIO_IRQ",
	.irq_disable	= gpio_irq_mask,
	.irq_mask	= gpio_irq_mask,
	.irq_unmask	= gpio_irq_unmask,
	.irq_set_type	= gpio_irq_type,
	.irq_set_wake	= gpio_irq_set_wake,
	.irq_enable	= ait_aic_irq_enable,
	
};


static void gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{

	MMP_ULONG i = 0;//, j = 0;
	MMP_ULONG  intsrc_H = 0, intsrc_L = 0, intsrc_H2L = 0, intsrc_L2H = 0;
	MMPF_PIO_REG piopin = MMPF_PIO_REG_UNKNOWN;
	AITPS_GPIO pGPIO = AITC_BASE_GPIO;
	struct irq_desc * gpio_irq_desc;
	struct irqaction *action;// = desc->action;
	struct gpio_button_data *bdata;// = (struct gpio_button_data *)(action->dev_id) ;

	//To find out the GPIO number and clean the interrupt
	for (i = 0; i < 4; i++) {
		
		intsrc_H = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_H_SR[i];
		intsrc_L = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_L_SR[i];
		intsrc_H2L = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_H2L_SR[i];
		intsrc_L2H = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_L2H_SR[i];

		if(intsrc_H2L != 0x0){
			pGPIO->GPIO_INT_H2L_SR[i] = intsrc_H2L&(1<<(ffs(intsrc_H2L)-1));
			piopin = i*0x20 + ffs(intsrc_H2L)-1;
			//break;
		}
		
		if(intsrc_L2H != 0x0&&(piopin == MMPF_PIO_REG_UNKNOWN)){
			pGPIO->GPIO_INT_L2H_SR[i] = intsrc_L2H&(1<<(ffs(intsrc_L2H)-1));				
			piopin = i*0x20 + ffs(intsrc_L2H)-1;
		}
		
		if(intsrc_H != 0x0 &&(piopin == MMPF_PIO_REG_UNKNOWN)){
			pGPIO->GPIO_INT_H_SR[i] = intsrc_H&(1<<(ffs(intsrc_H)-1));
			piopin = i*0x20 + ffs(intsrc_H)-1;
		}
		if(intsrc_L != 0x0&&(piopin == MMPF_PIO_REG_UNKNOWN)){
			pGPIO->GPIO_INT_L_SR[i] = intsrc_L&(1<<(ffs(intsrc_L)-1));
			piopin = i*0x20 + ffs(intsrc_L)-1;
		}
		



		if(piopin != MMPF_PIO_REG_UNKNOWN)
		{
			gpio_irq_desc = irq_to_desc(NR_AIC_IRQS+piopin);

//			action = gpio_irq_desc->action;
//			bdata = (struct gpio_button_data *)(action->dev_id) ;

//			pr_debug("%s:piopin = %d\n",__func__,piopin);	

			handle_simple_irq(NR_AIC_IRQS+piopin, gpio_irq_desc);

			piopin = MMPF_PIO_REG_UNKNOWN;
		}
	}

	return;

}

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
//			unsigned	pin  = PIN_BASE + (32 * bank) + j;
	//		void __iomem	*pio = pin_to_controller(pin);
//			unsigned	mask = pin_to_mask(pin);

			//if (__raw_readl(pio + PIO_PSR) & mask)
				//seq_printf(s, "GPIO:%s", __raw_readl(pio + PIO_PDSR) & mask ? "1" : "0");
			//else
				//seq_printf(s, "%s", __raw_readl(pio + PIO_ABSR) & mask ? "B" : "A");

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

static int __init vsnv3_gpio_debugfs_init(void)
{
	/* /sys/kernel/debug/at91_gpio */
	(void) debugfs_create_file("mcrv2_gpio", S_IFREG | S_IRUGO, NULL, NULL, &ait_gpio_operations);

	return 0;
}
postcore_initcall(vsnv3_gpio_debugfs_init);

#endif

/*--------------------------------------------------------------------------*/

/*
 * This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;
extern void MMPF_PIO_ISR(unsigned int irq,struct irq_desc *desc);
/*
 * Called from the processor-specific init to enable GPIO interrupt support.
 */
void __init ait_gpio_irq_setup(void)
{

						 
//	extern void MMPF_PIO_ISR(unsigned int irq,struct irq_desc *desc);
	//unsigned		pin;
	struct ait_gpio_chip	*this, *prev;

//	pr_info("VSNV3: %d gpio irqs in %d banks\n", pin - PIN_BASE, gpio_banks);
		
	unsigned		pioc, pin;

	//irq_set_chip_and_handler(AITVSNV3_ID_GPIO, &gpio_irqchip, MMPF_PIO_ISR);
	

	for (pioc = 0, pin = PIN_BASE, this = gpio_chip, prev = NULL;
			pioc++ < gpio_banks;
			prev = this, this++) {
	//	unsigned	id = this->bank->id;
		unsigned	i;

		for (i = 0, pin = (this->chip.base+PIN_BASE); i <  this->chip.ngpio; i++, pin++) {
			irq_set_lockdep_class(pin, &gpio_lock_class);

			/*
			 * Can use the "simple" and not "edge" handler since it's
			 * shorter, and the AIC handles interrupts sanely.
			 */
			irq_set_chip_and_handler(pin, &gpio_irqchip,
						 handle_simple_irq);
			set_irq_flags(pin, IRQF_VALID);
		
		}

		/* The toplevel handler handles one bank of GPIOs, except
		 * AT91SAM9263_ID_PIOCDE handles three... PIOC is first in
		 * the list, so we only set up that handler.
		 */
		//if (prev && prev->next == this)
		//	continue;

		//irq_set_chip_data(id, this);
		//irq_set_chained_handler(id, gpio_irq_handler);
			irq_set_chip_data(AIC_SRC_GPIO, this);

	}
	irq_set_chained_handler(AIC_SRC_GPIO, MMPF_PIO_ISR);			

	pr_info("VSNV3: %d gpio irqs in %d banks\n", pin - PIN_BASE, gpio_banks);

}

/* gpiolib support */
static int ait_gpiolib_direction_input(struct gpio_chip *chip,
					unsigned offset)
{
  unsigned pin = chip->base + offset ;
      
  if(pin < MMPF_PIO_REG_MAX) {
    MMPF_PIO_EnableGpioMode(pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	  MMPF_PIO_EnableOutputMode(pin, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
  }
  else if(chip->base==SADC_BASE){
    //dbg_printf(0,"[sean] : Set SADC Channel : %d\r\n",offset+1);
    #if 0//seantest
    {
      MMPF_SARADC_ATTRIBUTE sar_att;
      sar_att.TPWait=1;
      MMPF_SARADC_Init(&sar_att);
    }  
    #endif

    MMPF_SARADC_SetChannel(offset+1);  
  }
  else if(chip->base==PWM_BASE){
    int pwm_n = 0 ;
    int set_freq = 0 ;
    
    pwm_n = (offset < PWM_CHANS)?offset:offset-PWM_CHANS;
    set_freq = (offset < PWM_CHANS)?1:0 ;
    MMPF_PWM_ControlSet(PWM_GET_ID(ait_pwm[pwm_n].pwm), 0);
		MMPF_PWM_EnableOutputPin(ait_pwm[pwm_n].pwm, MMP_FALSE);
		// switch to gpio mode
    MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) ait_pwm[pwm_n].gpio, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR); 
    MMPF_PIO_SetData((MMPF_PIO_REG) ait_pwm[pwm_n].gpio, MMP_FALSE,MMPF_OS_LOCK_CTX_ISR) ;
  }
  else if (chip->base==DOG_BASE) {
            
  }
	return 0;
}
//EXPORT_SYMBOL(ait_set_gpio_input);


static int ait_gpiolib_direction_output(struct gpio_chip *chip,
					 unsigned offset, int val)
{
  unsigned pin = chip->base + offset ;
  if(pin < MMPF_PIO_REG_MAX) {    
    MMPF_PIO_EnableGpioMode(pin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
	  MMPF_PIO_EnableOutputMode(pin, 1, MMPF_OS_LOCK_CTX_ISR);
	  MMPF_PIO_SetData(pin, val, MMPF_OS_LOCK_CTX_ISR);
  }
  else if (chip->base==PWM_BASE) {
    int pwm_n = 0 ;
    pwm_n    = (offset<PWM_CHANS)?offset:offset-PWM_CHANS;
		// switch to non-gpio mode
    MMPF_PIO_EnableGpioMode((MMPF_PIO_REG) ait_pwm[pwm_n].gpio, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR); 
        
    pr_debug("Output PWM %d,Value:%d\r\n",pwm_n,val);
  }
  else if(chip->base==DOG_BASE ) {
        
  }
	return 0;
}
//EXPORT_SYMBOL(ait_set_gpio_input);

static int ait_gpiolib_get(struct gpio_chip *chip, unsigned offset)
{
	int returnValue=0;
  	unsigned pin = chip->base + offset ;
  	if(pin < MMPF_PIO_REG_MAX) { 
		MMPF_PIO_GetData(pin, (unsigned char *)&returnValue);
  	}
  	else if(chip->base==SADC_BASE){
		// set channel here 
		MMPF_SARADC_SetChannel(offset+1);  
		MMPF_SARADC_GetData( (MMP_USHORT *)&returnValue);
  	}
  	else if(chip->base==PWM_BASE) {
    		int pwm_n = 0 ;
		int set_freq = 0 ;
		pwm_n    = (offset < PWM_CHANS)?offset:offset-PWM_CHANS;
		set_freq = (offset < PWM_CHANS)?1:0 ;
		returnValue = (set_freq)?ait_pwm[pwm_n].freq : ait_pwm[pwm_n].duty ;
		dbg_printf(0,"[sean] : Get PWM%d Value:%d\r\n",  pwm_n , returnValue); 
 	}
	else if (chip->base==DOG_BASE) {
		returnValue =  MMPF_WD_GetTimeOut();  
		dbg_printf(0,"[sean] : Get watchdog windows:%d\r\n", returnValue); 
	}
	pr_debug("%s:+ returnValue = %d\n",__func__,returnValue);
	
	return returnValue;
}

static void ait_gpiolib_set(struct gpio_chip *chip, unsigned offset, int val)
{
//	struct vsnv3_gpio_chip *at91_gpio = to_at91_gpio_chip(chip);
//	void __iomem *pio = at91_gpio->regbase;
//	unsigned mask = 1 << offset;
  unsigned pin = chip->base + offset ;
  if(pin < MMPF_PIO_REG_MAX) { 
	  MMPF_PIO_SetData(pin,val, MMPF_OS_LOCK_CTX_ISR);
  }
  else if(chip->base==PWM_BASE) {
    int pwm_n = 0 ;
    int set_freq = 0 ;
    
    pwm_n    = (offset<PWM_CHANS)?offset:offset-PWM_CHANS;
    set_freq = (offset < PWM_CHANS)?1:0 ;
    if(!set_freq) {
        if(val > 100) val = 100 ;
    }
    if(set_freq) {
      ait_pwm[pwm_n].freq = val ;
    }
    else {
      ait_pwm[pwm_n].duty = val ;
    }
    pr_debug("Set PWM %d,Set : %s,Value:%d\r\n",pwm_n,set_freq?"Freq":"Duty",val);
    pr_debug("Freq :%d,Duty : %d\r\n",ait_pwm[pwm_n].freq,ait_pwm[pwm_n].duty );
    MMPF_PWM_SetFreqDuty(ait_pwm[pwm_n].pwm,ait_pwm[pwm_n].freq,ait_pwm[pwm_n].duty );      
  }
  else if (chip->base==DOG_BASE) {
    if(val==0) {
      MMPF_WD_EnableWD(0,0,0,0,0);
    } else if (val > 0) {
      MMPF_WD_SetTimeOut(val,1024) ; 
      MMPF_WD_EnableWD(MMP_TRUE,MMP_TRUE,MMP_FALSE,0,MMP_TRUE); 
    }
    dbg_printf(0,"[sean] : Set watchdog window: %d\r\n",val);
  }
	//__raw_writel(mask, pio + (val ? PIO_SODR : PIO_CODR));
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

	MMPF_PIO_Initialize();

  #if 1// support saradc
  {
    MMPF_SARADC_ATTRIBUTE sar_att;
    sar_att.TPWait=1;
    MMPF_SARADC_Init(&sar_att);
  }
  // support pwm  
  {
     MMPF_PWM_Initialize();
  }
  #endif
    
	//gpio_banks = nr_banks;
	gpio_banks =min( nr_banks,ARRAY_SIZE(gpio_chip));
	
	for (i = 0; i < nr_banks; i++) {
		ait_gpio = &gpio_chip[i];

		ait_gpio->bank = &data[i]; //NEW , for test
		//at91_gpio->chip.base = PIN_BASE + i * 32;
		//at91_gpio->regbase = at91_gpio->bank->offset +
		//	(void __iomem *)AT91_VA_BASE_SYS;
//			(void __iomem *)AIT8453_REG_VIRT_BASE;//AT91_VA_BASE_SYS;
//printk("at91_gpio->regbase = 0x%08X\r\n",at91_gpio->regbase);



		/* enable PIO controller's clock */
//		clk_enable(at91_gpio->bank->clock);
    if(ait_gpio->bank->clock) {
		  clk_enable(ait_gpio->bank->clock);
    }
		/* AT91SAM9263_ID_PIOCDE groups PIOC, PIOD, PIOE */
		//if (last && last->bank->id == at91_gpio->bank->id)
		//	last->next = at91_gpio;
		//last = at91_gpio;

		gpiochip_add(&ait_gpio->chip);
	}
	

}
