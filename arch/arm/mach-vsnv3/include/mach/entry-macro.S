/*
 * arch/arm/mach-at91/include/mach/entry-macro.S
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 * Low-level IRQ helper macros for AT91RM9200 platforms
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/hardware.h>
#include <mach/ait_aic.h>

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =(AIT_CPUPHL_P2V(AIT_AIC_PHYS_BASE))	@ base virtual address of AIC peripheral
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr     \irqstat, [\base, #(AIT_AIC_CISR_OFST)] @ read interrupt source number
	teq     \irqstat, #0                            @ ISR is 0 when no current interrupt
	ldrne   \irqnr, [\base, #(AIT_AIC_IVR_OFST)]    @ read IRQ vector register: de-asserts nIRQ to processor (and clears interrupt)
	.endm

