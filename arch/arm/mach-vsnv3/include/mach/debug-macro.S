/*
 * arch/arm/mach-at91/include/mach/debug-macro.S
 *
 *  Copyright (C) 2003-2005 SAN People
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <mach/hardware.h>

#if 1
	.macro	addruart, rp, rv, tmp
	ldr	\rp, =(AIT_UART0_PHYS_BASE)	@ System peripherals (phys address)
	ldr	\rv, =AIT_OPR_P2V(AIT_UART0_PHYS_BASE)
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #0x08]		@ Write to Transmitter Holding Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #0x24]		@ Read Status Register
	tst	\rd, #0x0000F000
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
@1001:	ldr	\rd, [\rx, #(0x24)]		@ Read Status Register
@	tst	\rd, #0xff00					@ DBGU_TXEMPTY = 1 when transmission complete
@	beq	1001b	
	.endm

#else
	.macro	addruart, rp, rv, tmp
	ldr	\rp, =(AIT_UART0_PHYS_BASE)	@ System peripherals (phys address)
	ldr	\rv, =AIT_OPR_P2V(AIT_UART0_PHYS_BASE)	@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #(AT91_DBGU_THR)]		@ Write to Transmitter Holding Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(AT91_DBGU_SR)]		@ Read Status Register
	tst	\rd, #AT91_DBGU_TXRDY			@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(AT91_DBGU_SR)]		@ Read Status Register
	tst	\rd, #AT91_DBGU_TXEMPTY			@ DBGU_TXEMPTY = 1 when transmission complete
	beq	1001b
	.endm
#endif
