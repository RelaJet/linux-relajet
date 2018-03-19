/*
 * cpucomm-uart.c virtual uart driver on cpucomm bus
 *
 * Based on drivers/mmc/card/sdio_uart.c by Nicolas Pitre
 *
 * Based on drivers/serial/8250.c and drivers/serial/serial_core.c by Russell King.
 *
 * Author:	Chiket
 * Created:	Dec 23, 2014
 * Copyright:	A.I.T Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>


#if 0
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif

#include "cpucomm.h"

#define UART_NR		8	/* Number of UARTs this driver can handle */


#define FIFO_SIZE	PAGE_SIZE
#define WAKEUP_CHARS	256

struct uart_icount {
	__u32	cts;
	__u32	dsr;
	__u32	rng;
	__u32	dcd;
	__u32	rx;
	__u32	tx;
	__u32	frame;
	__u32	overrun;
	__u32	parity;
	__u32	brk;
};

struct cpucomm_uart_port {
	struct tty_port		port;
	struct kref		kref;
	struct tty_struct	*tty;
	unsigned int		index;
	struct cpucomm_client	*client;
	struct mutex		func_lock;
	struct task_struct	*in_cpucomm_uart_irq;
	unsigned int		regs_offset;
	struct kfifo		xmit_fifo;
	spinlock_t		write_lock;
	struct uart_icount	icount;
	unsigned int		uartclk;
	unsigned int		mctrl;
	unsigned int		rx_mctrl;
	unsigned int		read_status_mask;
	unsigned int		ignore_status_mask;
	unsigned char		x_char;
	unsigned char           ier;
	unsigned char           lcr;

    unsigned char       *phy_work_buffer;
    unsigned char       *work_buffer[2];
    unsigned int        work_index;
    CPU_COMM_ID         entry_id;


};

static struct cpucomm_uart_port *cpucomm_uart_table[UART_NR]; // Need review the array further because we have one UART only
static DEFINE_SPINLOCK(cpucomm_uart_table_lock);

static int cpucomm_uart_add_port(struct cpucomm_uart_port *port)
{
	int index, ret = -EBUSY;

	kref_init(&port->kref);
	mutex_init(&port->func_lock);
	spin_lock_init(&port->write_lock);
	if (kfifo_alloc(&port->xmit_fifo, FIFO_SIZE, GFP_KERNEL))
		return -ENOMEM;

	spin_lock(&cpucomm_uart_table_lock);
	for (index = 0; index < UART_NR; index++) {
		if (!cpucomm_uart_table[index]) {
			port->index = index;
			cpucomm_uart_table[index] = port;
			ret = 0;
			break;
		}
	}
	spin_unlock(&cpucomm_uart_table_lock);

	return ret;
}

static struct cpucomm_uart_port *cpucomm_uart_port_get(unsigned index)
{
	struct cpucomm_uart_port *port;

	if (index >= UART_NR)
		return NULL;

	spin_lock(&cpucomm_uart_table_lock);
	port = cpucomm_uart_table[index];
	if (port)
		kref_get(&port->kref);
	spin_unlock(&cpucomm_uart_table_lock);

	return port;
}

static void cpucomm_uart_port_destroy(struct kref *kref)
{
	struct cpucomm_uart_port *port =
		container_of(kref, struct cpucomm_uart_port, kref);
	kfifo_free(&port->xmit_fifo);
	kfree(port);
}

static void cpucomm_uart_port_put(struct cpucomm_uart_port *port)
{
	kref_put(&port->kref, cpucomm_uart_port_destroy);
}

static void cpucomm_uart_port_remove(struct cpucomm_uart_port *port)
{
	struct cpucomm_client	*client;
	struct tty_struct       *tty;

	BUG_ON(cpucomm_uart_table[port->index] != port);

	spin_lock(&cpucomm_uart_table_lock);
	cpucomm_uart_table[port->index] = NULL;
	spin_unlock(&cpucomm_uart_table_lock);

	/*
	 * We're killing a port that potentially still is in use by
	 * the tty layer. Be careful to prevent any further access
	 * to the SDIO function and arrange for the tty layer to
	 * give up on that port ASAP.
	 * Beware: the lock ordering is critical.
	 */
	mutex_lock(&port->port.mutex);
	mutex_lock(&port->func_lock);
	client = port->client;
	port->client = NULL;
	mutex_unlock(&port->func_lock);
	tty = tty_port_tty_get(&port->port);
	/* tty_hangup is async so is this safe as is ?? */
	if (tty) {
		tty_hangup(tty);
		tty_kref_put(tty);
	}
	mutex_unlock(&port->port.mutex);

    // Unregister entries
    // CpuComm_UnregisterEntry() ??

	cpucomm_uart_port_put(port);
}

#if 0
static int cpucomm_uart_claim_func(struct cpucomm_uart_port *port)
{
	mutex_lock(&port->func_lock);
	if (unlikely(!port->client)) {
		mutex_unlock(&port->func_lock);
		return -ENODEV;
	}
	if (likely(port->in_cpucomm_uart_irq != current))
		cpucomm_claim_host(port->client);
	mutex_unlock(&port->func_lock);
	return 0;
}

static inline void cpucomm_uart_release_func(struct cpucomm_uart_port *port)
{
	if (likely(port->in_cpucomm_uart_irq != current))
		cpucomm_release_host(port->client);
}
#endif

#if 0
static inline unsigned int cpucomm_in(struct cpucomm_uart_port *port, int offset)
{
	unsigned char c;
	c = cpucomm_readb(port->func, port->regs_offset + offset, NULL);
	return c;
}

static inline void cpucomm_out(struct cpucomm_uart_port *port, int offset, int value)
{
	cpucomm_writeb(port->func, value, port->regs_offset + offset, NULL);
}

static unsigned int cpucomm_uart_get_mctrl(struct cpucomm_uart_port *port)
{
	unsigned char status;
	unsigned int ret;

	/* FIXME: What stops this losing the delta bits and breaking
	   cpucomm_uart_check_modem_status ? */
	status = cpucomm_in(port, UART_MSR);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void cpucomm_uart_write_mctrl(struct cpucomm_uart_port *port,
				  unsigned int mctrl)
{
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	cpucomm_out(port, UART_MCR, mcr);
}

static inline void cpucomm_uart_update_mctrl(struct cpucomm_uart_port *port,
					  unsigned int set, unsigned int clear)
{
	unsigned int old;

	old = port->mctrl;
	port->mctrl = (old & ~clear) | set;
	if (old != port->mctrl)
		cpucomm_uart_write_mctrl(port, port->mctrl);
}

#define cpucomm_uart_set_mctrl(port, x)	cpucomm_uart_update_mctrl(port, x, 0)
#define cpucomm_uart_clear_mctrl(port, x)	cpucomm_uart_update_mctrl(port, 0, x)

static void cpucomm_uart_change_speed(struct cpucomm_uart_port *port,
				   struct ktermios *termios,
				   struct ktermios *old)
{
	unsigned char cval, fcr = 0;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	for (;;) {
		baud = tty_termios_baud_rate(termios);
		if (baud == 0)
			baud = 9600;  /* Special case: B0 rate. */
		if (baud <= port->uartclk)
			break;
		/*
		 * Oops, the quotient was zero.  Try again with the old
		 * baud rate if possible, otherwise default to 9600.
		 */
		termios->c_cflag &= ~CBAUD;
		if (old) {
			termios->c_cflag |= old->c_cflag & CBAUD;
			old = NULL;
		} else
			termios->c_cflag |= B9600;
	}
	quot = (2 * port->uartclk + baud) / (2 * baud);

	if (baud < 2400)
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
	else
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10;

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	port->ier &= ~UART_IER_MSI;
	if ((termios->c_cflag & CRTSCTS) || !(termios->c_cflag & CLOCAL))
		port->ier |= UART_IER_MSI;

	port->lcr = cval;

	cpucomm_out(port, UART_IER, port->ier);
	cpucomm_out(port, UART_LCR, cval | UART_LCR_DLAB);
	cpucomm_out(port, UART_DLL, quot & 0xff);
	cpucomm_out(port, UART_DLM, quot >> 8);
	cpucomm_out(port, UART_LCR, cval);
	cpucomm_out(port, UART_FCR, fcr);

	cpucomm_uart_write_mctrl(port, port->mctrl);
}
#endif

#if 0
static void cpucomm_uart_start_tx(struct cpucomm_uart_port *port)
{
	if (!(port->ier & UART_IER_THRI)) {
		port->ier |= UART_IER_THRI;
		cpucomm_out(port, UART_IER, port->ier);
	}
}
#endif

#if 0
static void cpucomm_uart_stop_tx(struct cpucomm_uart_port *port)
{
	if (port->ier & UART_IER_THRI) {
		port->ier &= ~UART_IER_THRI;
		cpucomm_out(port, UART_IER, port->ier);
	}
}
#endif

#if 0
static void cpucomm_uart_stop_rx(struct cpucomm_uart_port *port)
{
	port->ier &= ~UART_IER_RLSI;
	port->read_status_mask &= ~UART_LSR_DR;
	cpucomm_out(port, UART_IER, port->ier);
}
#endif

#if 0 // need to rewrite
static void cpucomm_uart_receive_chars(struct cpucomm_uart_port *port,
				    unsigned int *status)
{
	struct tty_struct *tty = tty_port_tty_get(&port->port);
	unsigned int ch, flag;
	int max_count = 256;

	do {
		ch = cpucomm_in(port, UART_RX);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
					UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
			} else if (*status & UART_LSR_PE)
				port->icount.parity++;
			else if (*status & UART_LSR_FE)
				port->icount.frame++;
			if (*status & UART_LSR_OE)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			*status &= port->read_status_mask;
			if (*status & UART_LSR_BI)
				flag = TTY_BREAK;
			else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if ((*status & port->ignore_status_mask & ~UART_LSR_OE) == 0)
			if (tty)
				tty_insert_flip_char(tty, ch, flag);

		/*
		 * Overrun is special.  Since it's reported immediately,
		 * it doesn't affect the current character.
		 */
		if (*status & ~port->ignore_status_mask & UART_LSR_OE)
			if (tty)
				tty_insert_flip_char(tty, 0, TTY_OVERRUN);

		*status = cpucomm_in(port, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	if (tty) {
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
	}
}
#endif

#if 0 // need to rewrite
static void cpucomm_uart_transmit_chars(struct cpucomm_uart_port *port)
{
	struct kfifo *xmit = &port->xmit_fifo;
	int count;
	struct tty_struct *tty;
	u8 iobuf[16];
	int len;

	if (port->x_char) {
		cpucomm_out(port, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	tty = tty_port_tty_get(&port->port);

	if (tty == NULL || !kfifo_len(xmit) ||
				tty->stopped || tty->hw_stopped) {
		cpucomm_uart_stop_tx(port);
		tty_kref_put(tty);
		return;
	}

	len = kfifo_out_locked(xmit, iobuf, 16, &port->write_lock);
	for (count = 0; count < len; count++) {
		cpucomm_out(port, UART_TX, iobuf[count]);
		port->icount.tx++;
	}

	len = kfifo_len(xmit);
	if (len < WAKEUP_CHARS) {
		tty_wakeup(tty);
		if (len == 0)
			cpucomm_uart_stop_tx(port);
	}
	tty_kref_put(tty);
}
#endif

#if 0 // need to rewrite
static void cpucomm_uart_check_modem_status(struct cpucomm_uart_port *port)
{
	int status;
	struct tty_struct *tty;

	status = cpucomm_in(port, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		port->icount.rng++;
	if (status & UART_MSR_DDSR)
		port->icount.dsr++;
	if (status & UART_MSR_DDCD) {
		port->icount.dcd++;
		/* DCD raise - wake for open */
		if (status & UART_MSR_DCD)
			wake_up_interruptible(&port->port.open_wait);
		else {
			/* DCD drop - hang up if tty attached */
			tty = tty_port_tty_get(&port->port);
			if (tty) {
				tty_hangup(tty);
				tty_kref_put(tty);
			}
		}
	}
	if (status & UART_MSR_DCTS) {
		port->icount.cts++;
		tty = tty_port_tty_get(&port->port);
		if (tty && (tty->termios->c_cflag & CRTSCTS)) {
			int cts = (status & UART_MSR_CTS);
			if (tty->hw_stopped) {
				if (cts) {
					tty->hw_stopped = 0;
					cpucomm_uart_start_tx(port);
					tty_wakeup(tty);
				}
			} else {
				if (!cts) {
					tty->hw_stopped = 1;
					cpucomm_uart_stop_tx(port);
				}
			}
		}
		tty_kref_put(tty);
	}
}
#endif

/*
 * This handles the interrupt from one port.
 */
#if 0  // need to replace by tasklet or working queue
static void cpucomm_uart_irq(struct cpucomm_client *client)
{
	struct cpucomm_uart_port *port = cpucomm_get_drvdata(func);
	unsigned int iir, lsr;

	/*
	 * In a few places cpucomm_uart_irq() is called directly instead of
	 * waiting for the actual interrupt to be raised and the SDIO IRQ
	 * thread scheduled in order to reduce latency.  However, some
	 * interaction with the tty core may end up calling us back
	 * (serial echo, flow control, etc.) through those same places
	 * causing undesirable effects.  Let's stop the recursion here.
	 */
	if (unlikely(port->in_cpucomm_uart_irq == current))
		return;

	iir = cpucomm_in(port, UART_IIR);
	if (iir & UART_IIR_NO_INT)
		return;

	port->in_cpucomm_uart_irq = current;
	lsr = cpucomm_in(port, UART_LSR);
	if (lsr & UART_LSR_DR)
		cpucomm_uart_receive_chars(port, &lsr);
	cpucomm_uart_check_modem_status(port);
	if (lsr & UART_LSR_THRE)
		cpucomm_uart_transmit_chars(port);
	port->in_cpucomm_uart_irq = NULL;
}
#endif

static int cpucomm_uart_carrier_raised(struct tty_port *tport)
{
	return 0;
}

/**
 *	cpucomm_uart_dtr_rts	-	 port helper to set uart signals
 *	@tport: tty port to be updated
 *	@onoff: set to turn on DTR/RTS
 *
 *	Called by the tty port helpers when the modem signals need to be
 *	adjusted during an open, close and hangup.
 */

static void cpucomm_uart_dtr_rts(struct tty_port *tport, int onoff)
{
}

/**
 *	cpucomm_uart_activate	-	start up hardware
 *	@tport: tty port to activate
 *	@tty: tty bound to this port
 *
 *	Activate a tty port. The port locking guarantees us this will be
 *	run exactly once per set of opens, and if successful will see the
 *	shutdown method run exactly once to match. Start up and shutdown are
 *	protected from each other by the internal locking and will not run
 *	at the same time even during a hangup event.
 *
 *	If we successfully start up the port we take an extra kref as we
 *	will keep it around until shutdown when the kref is dropped.
 */

static int cpucomm_uart_activate(struct tty_port *tport, struct tty_struct *tty)
{
	return 0;
}

/**
 *	cpucomm_uart_shutdown	-	stop hardware
 *	@tport: tty port to shut down
 *
 *	Deactivate a tty port. The port locking guarantees us this will be
 *	run only if a successful matching activate already ran. The two are
 *	protected from each other by the internal locking and will not run
 *	at the same time even during a hangup event.
 */

static void cpucomm_uart_shutdown(struct tty_port *tport)
{
}

/**
 *	cpucomm_uart_install	-	install method
 *	@driver: the driver in use (cpucomm_uart in our case)
 *	@tty: the tty being bound
 *
 *	Look up and bind the tty and the driver together. Initialize
 *	any needed private data (in our case the termios)
 */
static int cpucomm_uart_install(struct tty_driver *driver, struct tty_struct *tty)
{
	int idx = tty->index;
	struct cpucomm_uart_port *port = cpucomm_uart_port_get(idx);
	int ret = tty_init_termios(tty);

	if (ret == 0) {
		tty_driver_kref_get(driver);
		tty->count++;
		/* This is the ref cpucomm_uart_port get provided */
		tty->driver_data = port;
		driver->ttys[idx] = tty;
	} else
		cpucomm_uart_port_put(port);
	return ret;
}

/**
 *	cpucomm_uart_cleanup	-	called on the last tty kref drop
 *	@tty: the tty being destroyed
 *
 *	Called asynchronously when the last reference to the tty is dropped.
 *	We cannot destroy the tty->driver_data port kref until this point
 */
static void cpucomm_uart_cleanup(struct tty_struct *tty)
{
	struct cpucomm_uart_port *port = tty->driver_data;
	tty->driver_data = NULL;	/* Bug trap */
	cpucomm_uart_port_put(port);
}

/*
 *	Open/close/hangup is now entirely boilerplate
 */

static int cpucomm_uart_open(struct tty_struct *tty, struct file *filp)
{
	struct cpucomm_uart_port *port = tty->driver_data;

	return tty_port_open(&port->port, tty, filp);
}

static void cpucomm_uart_close(struct tty_struct *tty, struct file * filp)
{
	struct cpucomm_uart_port *port = tty->driver_data;
	tty_port_close(&port->port, tty, filp);
}

static void cpucomm_uart_hangup(struct tty_struct *tty)
{
	struct cpucomm_uart_port *port = tty->driver_data;
	tty_port_hangup(&port->port);
}

void __UartWrite( const char *pWrite_Str );

static int cpucomm_uart_write(struct tty_struct *tty, const unsigned char *buf,
			   int count)
{
#if 0 // need rewrite
	struct cpucomm_uart_port *port = tty->driver_data;
	int ret = 0;

	if (!port->func)
		return -ENODEV;

	ret = kfifo_in_locked(&port->xmit_fifo, buf, count, &port->write_lock);
	if (!(port->ier & UART_IER_THRI)) {
		int err = cpucomm_uart_claim_func(port);
		if (!err) {
			cpucomm_uart_start_tx(port);
			cpucomm_uart_irq(port->func);
			cpucomm_uart_release_func(port);
		} else
			ret = err;
	}
	return ret;
#else
	struct cpucomm_uart_port *port = tty->driver_data;
    int size = count < 255 ? count : 255;
    unsigned int phy_addr;

    // copy string data to working buffer
    memcpy( port->work_buffer[port->work_index], buf, size );
    port->work_buffer[port->work_index][size] = '\0';

    // flush data cache of CPU
    flush_kernel_vmap_range(port->work_buffer[port->work_index], 256 );

    // Calculate the phy addr of current working buffer
    if( port->work_index )
        phy_addr = (unsigned int)(port->phy_work_buffer) + 256;
    else
        phy_addr = (unsigned int)(port->phy_work_buffer);

    // send data pointer to another CPU
    cpucomm_data_send( port->client, port->entry_id, &phy_addr, sizeof(buf), 1000 );

    // ping-pong work_index
    port->work_index = (port->work_index + 1)&0x1;

    return size;
#endif
}

static int cpucomm_uart_write_room(struct tty_struct *tty)
{
#if 0
	struct cpucomm_uart_port *port = tty->driver_data;
	return FIFO_SIZE - kfifo_len(&port->xmit_fifo);
#else
    return 256; // it should be a availible size.
#endif
}

static int cpucomm_uart_chars_in_buffer(struct tty_struct *tty)
{
#if 0
	struct cpucomm_uart_port *port = tty->driver_data;
	return kfifo_len(&port->xmit_fifo);
#else
    return 0; // it hsould be a unsent size
#endif
}

// Send a high priority character to the tty even if stopped
static void cpucomm_uart_send_xchar(struct tty_struct *tty, char ch)
{
#if 0
	struct cpucomm_uart_port *port = tty->driver_data;

	port->x_char = ch;
	if (ch && !(port->ier & UART_IER_THRI)) {
		if (cpucomm_uart_claim_func(port) != 0)
			return;
		cpucomm_uart_start_tx(port);
		cpucomm_uart_irq(port->func);
		cpucomm_uart_release_func(port);
	}
#endif
}

// Indicate that a tty should stop transmitting data down the stack.
static void cpucomm_uart_throttle(struct tty_struct *tty)
{
#if 0
	struct cpucomm_uart_port *port = tty->driver_data;

	if (!I_IXOFF(tty) && !(tty->termios->c_cflag & CRTSCTS))
		return;

	if (cpucomm_uart_claim_func(port) != 0)
		return;

	if (I_IXOFF(tty)) {
		port->x_char = STOP_CHAR(tty);
		cpucomm_uart_start_tx(port);
	}

	if (tty->termios->c_cflag & CRTSCTS)
		cpucomm_uart_clear_mctrl(port, TIOCM_RTS);

	cpucomm_uart_irq(port->func);
	cpucomm_uart_release_func(port);
#endif
}

// Indicate that a tty may continue transmitting data down the stack.
static void cpucomm_uart_unthrottle(struct tty_struct *tty)
{
#if 0
	struct cpucomm_uart_port *port = tty->driver_data;

	if (!I_IXOFF(tty) && !(tty->termios->c_cflag & CRTSCTS))
		return;

	if (cpucomm_uart_claim_func(port) != 0)
		return;

	if (I_IXOFF(tty)) {
		if (port->x_char) {
			port->x_char = 0;
		} else {
			port->x_char = START_CHAR(tty);
			cpucomm_uart_start_tx(port);
		}
	}

	if (tty->termios->c_cflag & CRTSCTS)
		cpucomm_uart_set_mctrl(port, TIOCM_RTS);

	cpucomm_uart_irq(port->func);
	cpucomm_uart_release_func(port);
#endif
}

static void cpucomm_uart_set_termios(struct tty_struct *tty,
						struct ktermios *old_termios)
{
#if 0 // maybe we could remove this function complete becase nothing to do with termios
	struct cpucomm_uart_port *port = tty->driver_data;
	unsigned int cflag = tty->termios->c_cflag;

	if (cpucomm_uart_claim_func(port) != 0)
		return;

	cpucomm_uart_change_speed(port, tty->termios, old_termios);

	/* Handle transition to B0 status */
	if ((old_termios->c_cflag & CBAUD) && !(cflag & CBAUD))
		cpucomm_uart_clear_mctrl(port, TIOCM_RTS | TIOCM_DTR);

	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) && (cflag & CBAUD)) {
		unsigned int mask = TIOCM_DTR;
		if (!(cflag & CRTSCTS) || !test_bit(TTY_THROTTLED, &tty->flags))
			mask |= TIOCM_RTS;
		cpucomm_uart_set_mctrl(port, mask);
	}

	/* Handle turning off CRTSCTS */
	if ((old_termios->c_cflag & CRTSCTS) && !(cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		cpucomm_uart_start_tx(port);
	}

	/* Handle turning on CRTSCTS */
	if (!(old_termios->c_cflag & CRTSCTS) && (cflag & CRTSCTS)) {
		if (!(cpucomm_uart_get_mctrl(port) & TIOCM_CTS)) {
			tty->hw_stopped = 1;
			cpucomm_uart_stop_tx(port);
		}
	}

	cpucomm_uart_release_func(port);
#endif
}

static int cpucomm_uart_break_ctl(struct tty_struct *tty, int break_state)
{
#if 0 // maybe we can implement console swap (uC/Lx) here
	struct cpucomm_uart_port *port = tty->driver_data;
	int result;

	result = cpucomm_uart_claim_func(port);
	if (result != 0)
		return result;

	if (break_state == -1)
		port->lcr |= UART_LCR_SBC;
	else
		port->lcr &= ~UART_LCR_SBC;
	cpucomm_out(port, UART_LCR, port->lcr);

	cpucomm_uart_release_func(port);
#endif
	return 0;
}

static int cpucomm_uart_tiocmget(struct tty_struct *tty)
{
#if 0 // maybe we could remove this function complete becase nothing to do with modem
	struct cpucomm_uart_port *port = tty->driver_data;
	int result;

	result = cpucomm_uart_claim_func(port);
	if (!result) {
		result = port->mctrl | cpucomm_uart_get_mctrl(port);
		cpucomm_uart_release_func(port);
	}

	return result;
#else
    return 0;
#endif
}

static int cpucomm_uart_tiocmset(struct tty_struct *tty,
			      unsigned int set, unsigned int clear)
{
#if 0 // maybe we could remove this function complete becase nothing to do with modem
	struct cpucomm_uart_port *port = tty->driver_data;
	int result;

	result = cpucomm_uart_claim_func(port);
	if (!result) {
		cpucomm_uart_update_mctrl(port, set, clear);
		cpucomm_uart_release_func(port);
	}

	return result;
#else
    return 0;
#endif
}

static int cpucomm_uart_proc_show(struct seq_file *m, void *v)
{
    // Show nothing currently
	return 0;
}

static int cpucomm_uart_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpucomm_uart_proc_show, NULL);
}

static const struct file_operations cpucomm_uart_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= cpucomm_uart_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct tty_port_operations cpucomm_uart_port_ops = {
	.carrier_raised = cpucomm_uart_carrier_raised,
	.dtr_rts = cpucomm_uart_dtr_rts,
	.shutdown = cpucomm_uart_shutdown,
	.activate = cpucomm_uart_activate,
};

static const struct tty_operations cpucomm_uart_ops = {
	.open			= cpucomm_uart_open,
	.close			= cpucomm_uart_close,
	.write			= cpucomm_uart_write,
	.write_room		= cpucomm_uart_write_room,
	.chars_in_buffer	= cpucomm_uart_chars_in_buffer,
	.send_xchar		= cpucomm_uart_send_xchar,
	.throttle		= cpucomm_uart_throttle,
	.unthrottle		= cpucomm_uart_unthrottle,
	.set_termios		= cpucomm_uart_set_termios,
	.hangup			= cpucomm_uart_hangup,
	.break_ctl		= cpucomm_uart_break_ctl,
	.tiocmget		= cpucomm_uart_tiocmget,
	.tiocmset		= cpucomm_uart_tiocmset,
	.install		= cpucomm_uart_install,
	.cleanup		= cpucomm_uart_cleanup,
	.proc_fops		= &cpucomm_uart_proc_fops,
};

static struct tty_driver *cpucomm_uart_tty_driver;

static int cpucomm_uart_probe(struct cpucomm_client *client/*, const struct cpucomm_device_id *id*/)
{
	struct cpucomm_uart_port *port;
    struct cpucomm_uart_data *uart_data = client->dev.platform_data;
    struct device *dev;
	int ret = -ENOMEM;

    // Allocate buffer
	port = kzalloc(sizeof(struct cpucomm_uart_port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

    // Register CpuComm Entry (hard code)
    cpucomm_register_entry( client, uart_data->id, CPU_COMM_TYPE_DATA);

    // Resuest SRAM as working buffer (hard code)
	if (!request_mem_region(uart_data->work_buf, 512, "cpucomm_uart"))
		goto err;

    // ioremap
    port->phy_work_buffer = (unsigned char *)(uart_data->work_buf);
    port->work_buffer[0] = ioremap(uart_data->work_buf, 512);
    if( port->work_buffer[0] == NULL && port->work_buffer[1] == NULL )
        goto err;

    // init variables
    port->work_buffer[1] = port->work_buffer[0]+256;
    port->work_index = 0;
    port->entry_id = uart_data->id;

    // Device driver related stuff
	port->client = client;
	cpucomm_set_clientdata(client, port);
	tty_port_init(&port->port);
	port->port.ops = &cpucomm_uart_port_ops;

    // Add uart data to cpucomm_uart_table for the usage of tty operations.
	ret = cpucomm_uart_add_port(port);
	if (ret) {
		kfree(port);
        goto err_iounmap;
	}

    // Register tty device
    dev = tty_register_device(cpucomm_uart_tty_driver,
						port->index, &client->dev);
	if (IS_ERR(dev)) {
		cpucomm_uart_port_remove(port);
		ret = PTR_ERR(dev);
        goto err_iounmap;
	}

	return ret;

err_iounmap:
    iounmap(port->work_buffer[0]);

err:
    CpuComm_UnregisterEntry(uart_data->id);
	return ret;
}

static int cpucomm_uart_remove(struct cpucomm_client *client)
{
	struct cpucomm_uart_port *port = cpucomm_get_clientdata(client);

	tty_unregister_device(cpucomm_uart_tty_driver, port->index);
	cpucomm_uart_port_remove(port);

    iounmap(port->work_buffer[0]);
    CpuComm_UnregisterEntry(port->entry_id);

    return 0;
}

#if 0
static const struct cpucomm_device_id cpucomm_uart_ids[] = {
	{ SDIO_DEVICE_CLASS(SDIO_CLASS_UART)		},
	{ SDIO_DEVICE_CLASS(SDIO_CLASS_GPS)		},
	{ /* end: all zeroes */				},
};

MODULE_DEVICE_TABLE(cpucomm, cpucomm_uart_ids);
#endif

static struct cpucomm_driver cpucomm_uart_driver = {
	.driver = {
		.name	= "cpucomm_uart",
		.owner	= THIS_MODULE,
	},
	.probe		= cpucomm_uart_probe,
	.remove		= __devexit_p(cpucomm_uart_remove),
	//.name		= "cpucomm_uart",
	//.id_table	= cpucomm_uart_ids,
};

static int __init cpucomm_uart_init(void)
{
	int ret;
	struct tty_driver *tty_drv;

	cpucomm_uart_tty_driver = tty_drv = alloc_tty_driver(UART_NR);
	if (!tty_drv)
		return -ENOMEM;

	tty_drv->owner = THIS_MODULE;
	tty_drv->driver_name = "cpucomm_uart";
	tty_drv->name =   "ttyCPU";
	tty_drv->major = 0;  /* dynamically allocated */
	tty_drv->minor_start = 0;
	tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype = SERIAL_TYPE_NORMAL;
	tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_drv->init_termios = tty_std_termios;
	tty_drv->init_termios.c_cflag = B4800 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_drv->init_termios.c_ispeed = 4800;
	tty_drv->init_termios.c_ospeed = 4800;
	tty_set_operations(tty_drv, &cpucomm_uart_ops);

	ret = tty_register_driver(tty_drv);
	if (ret)
		goto err1;

	ret = cpucomm_add_driver(&cpucomm_uart_driver);
	if (ret)
		goto err2;

	return 0;

err2:
	tty_unregister_driver(tty_drv);
err1:
	put_tty_driver(tty_drv);
	return ret;
}

static void __exit cpucomm_uart_exit(void)
{
	cpucomm_unregister_driver(&cpucomm_uart_driver);
	tty_unregister_driver(cpucomm_uart_tty_driver);
	put_tty_driver(cpucomm_uart_tty_driver);
}

module_init(cpucomm_uart_init);
module_exit(cpucomm_uart_exit);

MODULE_AUTHOR("Chiket Lin");
// MODULE_LICENSE("GPL");
