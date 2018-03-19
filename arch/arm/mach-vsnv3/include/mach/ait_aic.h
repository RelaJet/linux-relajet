/*
 * arch/arm/mach-ait/include/mach/ait_aic.h
 *
 * Advanced Interrupt Controller (AIC) - System peripherals registers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef AIT_AIC_H
#define AIT_AIC_H

#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)

#define AIT_AIC_IVR_OFST        (0x100)	    /* Interrupt Vector Register */

#define AIT_AIC_CISR_OFST       (0x114)	    /* Core Interrupt Status Register */

#endif // (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)

#endif // AIT_AIC_H
