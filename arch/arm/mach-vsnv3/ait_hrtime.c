/*
 * at91sam926x_time.c - Periodic Interval Timer (PIT) for at91sam926x
 *
 * Copyright (C) 2005-2006 M. Amine SAYA, ATMEL Rousset, France
 * Revision	 2005 M. Nicolas Diremdjian, ATMEL Rousset, France
 * Converted to ClockSource/ClockEvents by David Brownell.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clockchips.h>

#include <asm/mach/time.h>

#include "includes_fw.h"
#include "mmpf_timer.h"


#define HRTIMER_EVT             (MMPF_TIMER_0)//(MMPF_TIMER_2)
#define HRTIMER_EVT_IRQ         (AIC_SRC_TC0)
#define HRTIMER_EVT_MCLK_DIV    (MMPF_TIMER_MCLK_D1)
#define HRTIMER_SRC             (MMPF_TIMER_1)

static struct clock_event_device vsnv3_clockevent;


static cycle_t read_cycle(struct clocksource *cs)
{
    return MMPF_Timer_ReadCount(HRTIMER_SRC);
}


void __init vsnv3_clocksource_init(const char *name)
{
    MMPF_TIMER_ATTRIBUTE tmr_clksrc;

    tmr_clksrc.Callback   = NULL;
    tmr_clksrc.EventMode  = MMPF_TIMER_EVT_NONE;
    tmr_clksrc.MClkDiv    = MMPF_TIMER_MCLK_D128;
    tmr_clksrc.TimeUnit   = MMPF_TIMER_PRCN_TICK;
    tmr_clksrc.ulDelayCnt = (MMP_ULONG)-1;
    tmr_clksrc.bEnableTick = MMP_TRUE;
    MMPF_Timer_Open(HRTIMER_SRC, &tmr_clksrc);

    clocksource_mmio_init(NULL, "hrtimer", MMPF_Timer_CalculateTickFreq(tmr_clksrc.MClkDiv),
            200, 32, read_cycle);
}

/*
 * IRQ handler for the timer
*/
static irqreturn_t vsnv3_timer_interrupt(int irq, void *dev_id)
{
    #ifdef CONFIG_LEDS_TIMER
	extern void do_leds(void);
    #endif
	/*
	 * irqs should be disabled here, but as the irq is shared they are only
	 * guaranteed to be off if the timer irq is registered first.
	 */

	WARN_ON_ONCE(!irqs_disabled());

	MMPF_TIMER_ISR(HRTIMER_EVT); // clear sr

    #ifdef CONFIG_LEDS_TIMER
	do_leds();
    #endif

	if (vsnv3_clockevent.event_handler)
        vsnv3_clockevent.event_handler(&vsnv3_clockevent);

	return IRQ_HANDLED;
}

static void vsnv3_timer_set_mode(enum clock_event_mode mode,
        struct clock_event_device *evt)
{
    switch (mode) {
    case /*2*/CLOCK_EVT_MODE_PERIODIC:
        {
            MMPF_TIMER_ATTRIBUTE attr;

            attr.Callback   = NULL;
            attr.EventMode  = MMPF_TIMER_EVT_PERIODIC;
            attr.MClkDiv    = HRTIMER_EVT_MCLK_DIV;
            attr.TimeUnit   = MMPF_TIMER_PRCN_MSEC;
            attr.ulDelayCnt = DIV_ROUND_CLOSEST(MSEC_PER_SEC, HZ);
            WARN_ON(attr.ulDelayCnt*HZ != MSEC_PER_SEC);
            attr.bEnableTick = MMP_TRUE;

            MMPF_Timer_Open(HRTIMER_EVT, &attr);
            MMPF_Timer_EnableInterrupt(HRTIMER_EVT, MMP_TRUE);
        }
        break;
    case /*3*/CLOCK_EVT_MODE_ONESHOT:
        {
            MMPF_TIMER_ATTRIBUTE attr;

            attr.Callback   = NULL;
            attr.EventMode  = MMPF_TIMER_EVT_ONESHOT;
            attr.MClkDiv    = HRTIMER_EVT_MCLK_DIV;
            attr.TimeUnit   = MMPF_TIMER_PRCN_TICK;
            attr.ulDelayCnt = 0x55667788;
            attr.bEnableTick = MMP_FALSE;

            MMPF_Timer_Open(HRTIMER_EVT, &attr);
            MMPF_Timer_EnableInterrupt(HRTIMER_EVT, MMP_TRUE);
        }
        break;
    case /*0*/CLOCK_EVT_MODE_UNUSED:
    case /*1*/CLOCK_EVT_MODE_SHUTDOWN:
    default:
        MMPF_Timer_EnableInterrupt(HRTIMER_EVT, MMP_FALSE);
        MMPF_Timer_Close(HRTIMER_EVT);
        break;
    }
}

static int vsnv3_set_next_event(unsigned long next,
        struct clock_event_device *evt)
{
    MMPF_Timer_RestartTick(HRTIMER_EVT, MMP_TRUE, next);

	return 0;
}
 
static struct clock_event_device vsnv3_clockevent = {
    .shift          = 32,
    .features       = CLOCK_EVT_FEAT_PERIODIC|CLOCK_EVT_FEAT_ONESHOT,
    .set_mode       = vsnv3_timer_set_mode,
    .set_next_event = vsnv3_set_next_event,
    .rating         = 400,
    .cpumask        = cpu_all_mask,
};
 
static struct irqaction vsnv3_timer_irq = {
    .name           = "hrtimer_irq",
    .flags          = IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .handler        = vsnv3_timer_interrupt,
    .dev_id         = &vsnv3_clockevent,
};
 
void __init vsnv3_clockevents_init(unsigned int irq, const char *name)
{
    struct clock_event_device *evt = &vsnv3_clockevent;
    long rate;

    MMPF_Timer_EnableInterrupt(HRTIMER_EVT, MMP_FALSE);
    MMPF_Timer_Close(HRTIMER_EVT);

    rate = MMPF_Timer_CalculateTickFreq(HRTIMER_EVT_MCLK_DIV);

    evt->name = name;
    evt->irq = irq;
    evt->mult = div_sc(rate, NSEC_PER_SEC, evt->shift);
    evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
    evt->min_delta_ns = clockevent_delta2ns(0xf, evt);

    setup_irq(irq, &vsnv3_timer_irq);
    clockevents_register_device(evt);
}

static int __init init_ait_clocksource(void)
{
	vsnv3_clockevents_init(HRTIMER_EVT_IRQ, "tc0_clk_hr");
	vsnv3_clocksource_init("tc1_clk");
	return 0;
}

module_init(init_ait_clocksource);

