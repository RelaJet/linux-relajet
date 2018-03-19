/*
 * linux/arch/arm/mach-vsnv3/irq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>


static void ait_aic_mask_irq(struct irq_data *d)
{
    RTNA_AIC_IRQ_Dis(AITC_BASE_AIC, d->irq);
}

static void ait_aic_unmask_irq(struct irq_data *d)
{
    RTNA_AIC_IRQ_En(AITC_BASE_AIC, d->irq);
}

static int ait_aic_set_type(struct irq_data *d, unsigned int type)
{
	unsigned int srctype;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		srctype = AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE;
		break;
	case IRQ_TYPE_EDGE_RISING:
		srctype = AIC_SRCTYPE_POSITIVE_EDGE_TRIGGERED;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		srctype = AIC_SRCTYPE_LOW_LEVEL_SENSITIVE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	    srctype = AIC_SRCTYPE_NEGATIVE_EDGE_TRIGGERED;
		break;
	default:
		return -EINVAL;
	}

    RTNA_AIC_SetMode(AITC_BASE_AIC, d->irq, AIC_SRCTYPE_MASK, srctype);

	return 0;
}

#ifdef CONFIG_PM

#define IRQ_BANK_NUM        ((NR_AIC_IRQS + 31) >> 5)
#define irq_to_bank(_n)     ((_n) >> 5)
#define irq_to_offset(_n)   ((_n) & 0x1F)

#if 1
static u32 wakeups[IRQ_BANK_NUM];
static u32 backups[IRQ_BANK_NUM];
#endif

static int ait_aic_set_wake(struct irq_data *d, unsigned value)
{
    #if 1
	if (unlikely(d->irq >= NR_AIC_IRQS))
		return -EINVAL;

	if (value)
		wakeups[irq_to_bank(d->irq)] |= (1 << irq_to_offset(d->irq));
	else
        wakeups[irq_to_bank(d->irq)] &= ~(1 << irq_to_offset(d->irq));
    #endif

	return 0;
}

void ait_irq_suspend(void)
{
    #if 0
    backups = at91_sys_read(AT91_AIC_IMR);
    at91_sys_write(AT91_AIC_IDCR, backups);
    at91_sys_write(AT91_AIC_IECR, wakeups);
    #endif

    int bank = 0, irq = 0;

    for (irq = 0; irq < NR_AIC_IRQS; irq++) {
        if (!(irq & 0x1F)) {
            bank = irq_to_bank(irq);
            backups[bank] = 0;
        }

        if (RTNA_AIC_GetState(AITC_BASE_AIC, irq)) {
            backups[bank] |= (1 << irq_to_offset(irq));

            RTNA_AIC_IRQ_Dis(AITC_BASE_AIC, irq);
        }
        if (wakeups[bank] & (1 << irq_to_offset(irq))) {
            RTNA_AIC_IRQ_En(AITC_BASE_AIC, irq);
        }
    }

    return;
}

void ait_irq_resume(void)
{
    #if 0
	at91_sys_write(AT91_AIC_IDCR, wakeups);
	at91_sys_write(AT91_AIC_IECR, backups);
    #endif

    int bank = 0, irq = 0;

    for (irq = 0; irq < NR_AIC_IRQS; irq++) {
        if (!(irq & 0x1F)) {
            bank = irq_to_bank(irq);
        }

        if (wakeups[bank] & (1 << irq_to_offset(irq))) {
            RTNA_AIC_IRQ_Dis(AITC_BASE_AIC, irq);
        }

        if (backups[bank] & (1 << irq_to_offset(irq))) {
            RTNA_AIC_IRQ_En(AITC_BASE_AIC, irq);
        }
    }

	return;
}

#else // CONFIG_PM

#define ait_aic_set_wake	NULL

#endif // CONFIG_PM


static struct irq_chip ait_aic_chip = {
	.name		= "AIC",
	//.irq_ack	= ait_aic_mask_irq,
	.irq_mask	= ait_aic_mask_irq,
	.irq_unmask	= ait_aic_unmask_irq,
	.irq_set_type	= ait_aic_set_type,
	.irq_set_wake	= ait_aic_set_wake,
};

/*
 * Initialize the AIC interrupt controller.
 */
void __init ait_aic_init(unsigned int priority[NR_AIC_IRQS])
{
	unsigned int i;

	/*
	 * The IVR is used by macro get_irqnr_and_base to read and verify.
	 * The irq number is NR_AIC_IRQS when a spurious interrupt has occurred.
	 */
	for (i = 0; i < NR_AIC_IRQS; i++) {
		/* Put irq number in Source Vector Register: */
	    RTNA_AIC_SetVector(AITC_BASE_AIC, i, i);

        RTNA_AIC_SetMode(AITC_BASE_AIC, i, AIC_PRIOR_MASK | AIC_SRCTYPE_MASK | AIC_INT_MASK,
                ((priority[i]? priority[i]: 3)  & AIC_PRIOR_MASK)
                    | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | AIC_INT_TO_IRQ);

		irq_set_chip_and_handler(i, &ait_aic_chip, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}

	/* No debugging in AIC: Debug (Protect) Control Register */
	RTNA_AIC_SetProtectMode(AITC_BASE_AIC, 0);

	/* Disable and clear all interrupts initially */
	RTNA_AIC_IRQ_DisAll(AITC_BASE_AIC);
	RTNA_AIC_IRQ_ClearAll(AITC_BASE_AIC);
}


