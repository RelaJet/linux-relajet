/*
 * arch/arm/mach-at91/include/mach/uncompress.h
 *
 *  Copyright (C) 2003 SAN People
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

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <linux/io.h>

#include <mach/mmp_register.h>
#include <mach/mmp_reg_uart.h>

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader.  If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 *
 * This does not append a newline
 */
static void putc(int c)
{

#if 0//(CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    AITPS_UART pUART = (AITPS_UART) (AITC_BASE_PHY_UART_BOOT); /* physical address */
    int txcnt, ulLength = 1;
    int timeout = 0;

    do {
        txcnt = pUART->US0.US_TX_FIFO_DATA_CNT;
    } while (txcnt == 0 && timeout++ < 100);

    if (txcnt) {
        if (txcnt > ulLength) {
            txcnt = ulLength;
        }
        pUART->US0.US_TXPR = c;
    }

    timeout = 0;
    do {
        txcnt = pUART->US0.US_TX_FIFO_DATA_CNT;
        timeout++;
    } while (txcnt != 128 && timeout++ < 100);
#endif

#if 0
    void __iomem *sys = (void __iomem *) UART_OFFSET; /* physical address */

    while (!(__raw_readl(sys + ATMEL_US_CSR) & ATMEL_US_TXRDY))
        barrier();
    __raw_writel(c, sys + ATMEL_US_THR);
#endif
}

static inline void flush(void)
{
#if 0
    void __iomem *sys = (void __iomem *) UART_OFFSET; /* physical address */

    while (!(__raw_readl(sys + ATMEL_US_CSR) & ATMEL_US_TXEMPTY))
        barrier();
#endif
}

#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif
