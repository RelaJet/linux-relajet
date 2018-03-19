/*
 * ait_time.c - Periodic Interval Timer (PIT) for
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <asm/mach/time.h>

#include "mmpf_timer.h"

#define SYSTEM_TIMER        (MMPF_TIMER_0)
#define SYSTEM_TIMER_IRQ    (AIC_SRC_TC0)

struct ait_pit_info {
    u32 cycle;
    u32 count;
    union {
        u32 per_ms;
        u32 per_us;
    };
};


static struct ait_pit_info pitctl;

//static u32 pit_cycle;   /* write-once */
//static u32 pit_cnt;		/* access only w/system irq blocked */
//static u32 pit_time_ms  = 0;
//static bool bIrqRegistered = false;

static struct irqaction ait_pit_irqaction;

/*
 * Clocksource:  just a monotonic counter of MCK/16 cycles.
 * We don't care whether or not PIT irqs are enabled.
 */
static cycle_t read_pit_clk(struct clocksource *cs)
{
    unsigned long flags;
    u32 elapsed, ct;

    raw_local_irq_save(flags);
    elapsed = pitctl.count;
    ct = MMPF_Timer_ReadCount(SYSTEM_TIMER);
    if (MMPF_Timer_IsOnTime(SYSTEM_TIMER)) {
        ct = pitctl.cycle + MMPF_Timer_ReadCount(SYSTEM_TIMER);
    }
    raw_local_irq_restore(flags);

    return elapsed + ct;
}

static struct clocksource pit_clk = {
	.name		= "pit",
	.rating		= 175,
	.read		= read_pit_clk,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Clockevent device:  interrupts every 1/HZ (== cycles * MCK/16)
 */
static void
pit_clkevt_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
	    {
	        MMPF_TIMER_ATTRIBUTE pit_attr;
	        //if (!bIrqRegistered) {
	        //    setup_irq(SYSTEM_TIMER_IRQ, &ait_pit_irqaction);
	        //    bIrqRegistered = true;
	        //}

	        pitctl.count += MMPF_Timer_ReadCount(SYSTEM_TIMER); //save boot time till now

    		pit_attr.Callback   = NULL;
    		pit_attr.EventMode  = MMPF_TIMER_EVT_PERIODIC;
    		pit_attr.MClkDiv    = MMPF_TIMER_MCLK_D1;
    		pit_attr.TimeUnit   = MMPF_TIMER_PRCN_MSEC;
    		pit_attr.ulDelayCnt = pitctl.per_ms;
    		pit_attr.bEnableTick = MMP_TRUE;
            MMPF_Timer_Open(SYSTEM_TIMER, &pit_attr);
            MMPF_Timer_EnableInterrupt(SYSTEM_TIMER, MMP_TRUE);
        }
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		BUG(); /* FALLTHROUGH */
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
        MMPF_Timer_EnableInterrupt(SYSTEM_TIMER, MMP_FALSE);
        MMPF_Timer_Close(SYSTEM_TIMER);
        //remove_irq(SYSTEM_TIMER_IRQ, &ait_pit_irqaction);
        //bIrqRegistered = false;
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device pit_clkevt = {
	.name		= "pit",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.shift		= 32,
	.rating		= 100,
	.set_mode	= pit_clkevt_mode,
};

/*
 * IRQ handler for the timer.
 */
static irqreturn_t ait_pit_isr(int irq, void *dev_id)
{
	/*
	 * irqs should be disabled here, but as the irq is shared they are only
	 * guaranteed to be off if the timer irq is registered first.
	 */
	WARN_ON_ONCE(!irqs_disabled());

	/* The PIT interrupt may be disabled, and is shared */
	if (pit_clkevt.mode == CLOCK_EVT_MODE_PERIODIC) {
        #ifdef CONFIG_LEDS_TIMER
        extern void do_leds(void);
        #endif

	    MMPF_TIMER_ISR(SYSTEM_TIMER);

        #ifdef CONFIG_LEDS_TIMER
	    do_leds();
        #endif

	    pitctl.count += pitctl.cycle;

	    if (pit_clkevt.event_handler)
	        pit_clkevt.event_handler(&pit_clkevt);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static struct irqaction ait_pit_irqaction = {
	.name		= "tc0_pit",
	.flags		= IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .dev_id     = &pit_clkevt,
	.handler	= ait_pit_isr
};

/*
 * Set up both clocksource and clockevent support.
 */
static void __init ait_pit_init(void)
{
	unsigned long pit_rate;
	MMPF_TIMER_ATTRIBUTE pit_attr;

	//clock         : 264000000
	//pit_rate      : 128906
	//pitctl.cycle  : 1289
	//HZ            : 100

	pitctl.per_ms = DIV_ROUND_CLOSEST(MSEC_PER_SEC, HZ);
	WARN_ON(pitctl.per_ms*HZ != MSEC_PER_SEC);

	/* Initialize and enable the timer */
    MMPF_Timer_EnableInterrupt(SYSTEM_TIMER, MMP_FALSE);
    MMPF_Timer_Close(SYSTEM_TIMER);

	pit_attr.Callback   = NULL;
	pit_attr.EventMode  = MMPF_TIMER_EVT_PERIODIC;
	pit_attr.MClkDiv    = MMPF_TIMER_MCLK_D1;
	pit_attr.TimeUnit   = MMPF_TIMER_PRCN_MSEC;
	pit_attr.ulDelayCnt = pitctl.per_ms;
	pit_attr.bEnableTick = MMP_TRUE;
    MMPF_Timer_Open(SYSTEM_TIMER, &pit_attr);

    pit_rate = MMPF_Timer_CalculateTickFreq(pit_attr.MClkDiv);
    pitctl.cycle = pit_rate / HZ;
    pitctl.count = 0;

	/*
	 * Register clocksource.  The high order bits of PIV are unused,
	 * so this isn't a 32-bit counter unless we get clockevent irqs.
	 */
	//bits = 12 /* PICNT */ + ilog2(pitctl.cycle) /* PIV */;
	pit_clk.mask = 0xffffffff;
	clocksource_register_hz(&pit_clk, pit_rate);

	/* Set up irq handler */
	setup_irq(SYSTEM_TIMER_IRQ, &ait_pit_irqaction);
	//bIrqRegistered = true;

	/* Set up and register clockevents */
	pit_clkevt.mult = div_sc(pit_rate, NSEC_PER_SEC, pit_clkevt.shift);
	pit_clkevt.cpumask = cpumask_of(0);
	clockevents_register_device(&pit_clkevt);
}

static void ait_pit_suspend(void)
{
    MMPF_Timer_EnableTick(SYSTEM_TIMER, MMP_FALSE);
}

static void ait_pit_resume(void)
{
    MMPF_Timer_EnableTick(SYSTEM_TIMER, MMP_TRUE);
}

struct sys_timer ait_pit_timer = {
	.init		= ait_pit_init,
	.suspend	= ait_pit_suspend,
	.resume		= ait_pit_resume,
};

