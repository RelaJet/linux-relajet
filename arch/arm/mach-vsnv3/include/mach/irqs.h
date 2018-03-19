/*
 * arch/arm/mach-at91/include/mach/irqs.h
 *
 *  Copyright (C) 2004 SAN People
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

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#include <linux/io.h>
#include <mach/includes_fw.h>
#include <mach/lib_retina.h>

#if (CHIP == VSN_V3)
#define NR_AIC_IRQS         (32)
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define NR_AIC_IRQS         (64)
#endif

/*
 * Acknowledge interrupt with AIC after interrupt has been handled.
 *   (by kernel/irq.c)
 */
#define irq_finish(irq) do { RTNA_AIC_IRQ_EOICR(AITC_BASE_AIC); } while (0)

/*
 * IRQ interrupt symbols are the AT91xxx_ID_* symbols
 * for IRQs handled directly through the AIC, or else the AT91_PIN_*
 * symbols in gpio.h for ones handled indirectly as GPIOs.
 * We make provision for ? banks of GPIO.
 */
#if (CHIP == VSN_V3)
#define NR_IRQS     (NR_AIC_IRQS + 66)
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define NR_IRQS     (NR_AIC_IRQS + 127+8+3*2+1)	// 8 SADC      3*2 PWM 1 HWDOG
#endif

/* FIQ is AIC source 0. */
#define FIQ_START   AIT_ID_FIQ

#endif
