/*
 *
 *  Based on drivers/tty/serial/atmel_serial.c
 *  Based on drivers/char/serial_sa1100.c, by Deep Blue Solutions Ltd.
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
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
 *
 */
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>

#include <asm/io.h>
#include <asm/ioctls.h>

#include <mach/board.h>

#ifdef CONFIG_ARM
#include <mach/cpu.h>
#include <asm/gpio.h>
#endif

#if defined(CONFIG_SERIAL_AIT_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

/*Vin@ add for debugging*/
//#include <mach/includes_fw.h>
#include <mach/mmpf_uart.h>

static void ait_stop_rx(struct uart_port *port);

/* Use device name ttyS, major 4, minor 64-68.  This is the usual serial port
 * name, but it is legally reserved for the 8250 driver. */
#define SERIAL_AIT_MAJOR	TTY_MAJOR
#define MINOR_START		64
#define AIT_CONSOLE_DEVICE  "ttyS"


#define UART_GET_TXCNT(port) ((((( (struct ait_uart_data *) (port)->dev->platform_data ))->pUS->US_FSR )&US_TX_FIFO_UNWR_MASK)>>8)

static int (*ait_open_hook)(struct uart_port *);
static void (*ait_close_hook)(struct uart_port *);

struct ait_uart_chdesc {
	u16		status;
	u16		ch;
};

#define AIT_RX_RINGSIZE 1024

/*
 * We wrap our port structure around the generic uart_port.
 */
struct ait_uart_port {
	struct uart_port    uart;		/* uart */
	struct clk          *clk;		/* uart clock */
	int			may_wakeup;	/* cached value of device_may_wakeup for times we need to disable it */
	u32			backup_imr;	/* IMR saved during suspend */
	int			break_active;	/* break being received */

	struct tasklet_struct	tasklet;

	struct circ_buf		rx_ring;
};

static struct ait_uart_port ait_usports[AIT_MAX_UART];
static unsigned long ait_usports_in_use;

#if defined(CONFIG_OF)
static const struct of_device_id ait_serial_dt_ids[] = {
	{ .compatible = "atmel,at91rm9200-usart" },
	{ .compatible = "atmel,at91sam9260-usart" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ait_serial_dt_ids);
#endif

static inline struct ait_uart_port *
to_ait_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct ait_uart_port, uart);
}

static inline struct ait_uart_data *
to_ait_uart_data(struct uart_port *port)
{
	return (struct ait_uart_data *)port->dev->platform_data;
}
static bool ait_use_dma_tx(struct uart_port *port)
{
	return false;
}

/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static u_int ait_tx_empty(struct uart_port *port)
{
    return MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_TXEMPTY)?
            TIOCSER_TEMT: 0;
}

/*
 * Set state of the modem control output lines
 */
static void ait_set_mctrl(struct uart_port *port, u_int mctrl)
{
}

/*
 * Get state of the modem control input lines
 */
static u_int ait_get_mctrl(struct uart_port *port)
{
	return 0;
}

/*
 * Stop transmitting.
 */
static void ait_stop_tx(struct uart_port *port)
{
	if (ait_use_dma_tx(port)) {
	}
}

/*
 * Start transmitting.
 */
static void ait_start_tx(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	tasklet_schedule(&ait_port->tasklet);
}

/*
 * Stop receiving - port is in process of being closed.
 */
static void ait_stop_rx(struct uart_port *port)
{
}

/*
 * Enable modem status interrupts
 */
static void ait_enable_ms(struct uart_port *port)
{}

/*
 * Control the transmission of a break signal
 */
static void ait_break_ctl(struct uart_port *port, int break_state)
{
	if (break_state != 0)
		;/* start break */
	else
		;/* stop break */
}

/*
 * Characters received (called from interrupt handler)
 */
static void ait_rx_chars(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
    struct circ_buf *ring = &ait_port->rx_ring;
    struct ait_uart_chdesc *c;
	unsigned int nr;
	unsigned char ch;

    while ((nr = MMPF_Uart_TryRead(port->line, &ch, 1)) != 0) {
        if (!CIRC_SPACE(ring->head, ring->tail, AIT_RX_RINGSIZE)) {
            /* Buffer overflow, ignore char */
            break;
        }

        c = &((struct ait_uart_chdesc *)ring->buf)[ring->head];
        c->status   = 0;//reserved ATMEL_US_RXRDY;
        c->ch       = ch;

        /* Make sure the character is stored before we update head. */
        smp_wmb();

        ring->head = (ring->head + 1) & (AIT_RX_RINGSIZE - 1);
        //atmel_buffer_rx_char(port, ATMEL_US_RXRDY, ch);
    }

	tasklet_schedule(&ait_port->tasklet);
}


/*
 * Transmit characters (called from tasklet with TXRDY interrupt
 * disabled)
 */
 static void ait_tx_chars(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;

    if (unlikely(port->x_char)) {
        MMPF_Uart_Write(port->line, (char*)&(port->x_char), 1);

        port->icount.tx++;
        port->x_char = 0;
    }

    if (uart_circ_empty(xmit) || uart_tx_stopped(port))
        return;

    while (!uart_circ_empty(xmit)) {
        MMPF_Uart_Write(port->line, (char*)(xmit->buf+xmit->tail), 1);

        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

/*
 * Interrupt handler
 */
static irqreturn_t ait_uart_int_handler(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;

    if (MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_RXOVERTH)) {

        ait_rx_chars(port);
    }

    return IRQ_HANDLED;
}

static void ait_rx_from_ring(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	struct circ_buf *ring = &ait_port->rx_ring;
	unsigned int flg;
	unsigned int status;

	while (ring->head != ring->tail) {
		struct ait_uart_chdesc c;

		/* Make sure c is loaded after head. */
		smp_rmb();

		c = ((struct ait_uart_chdesc *)ring->buf)[ring->tail];

		ring->tail = (ring->tail + 1) & (AIT_RX_RINGSIZE - 1);

		port->icount.rx++;
		status = c.status;
		flg = TTY_NORMAL;

		if (uart_handle_sysrq_char(port, c.ch))
			continue;

		uart_insert_char(port, status, 0, c.ch, flg);
	}

	/*
	 * Drop the lock here since it might end up calling
	 * uart_start(), which takes the lock.
	 */
	spin_unlock(&port->lock);
	tty_flip_buffer_push(port->state->port.tty);
	spin_lock(&port->lock);
}

/*
 * tasklet handling tty stuff outside the interrupt handler.
 */
static void ait_uart_tasklet_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;

	/* The interrupt handler does not take the lock */
	spin_lock(&port->lock);

	ait_tx_chars(port);

	wake_up_interruptible(&port->state->port.delta_msr_wait);

    ait_rx_from_ring(port);

	spin_unlock(&port->lock);
}

/*
 * Perform initialization and enable port for reception
 */
static int ait_startup(struct uart_port *port)
{
	extern MMP_ULONG MMPF_PLL_GetGroup0Freq(MMP_ULONG MClk);
	struct tty_struct *tty = port->state->port.tty;
	int retval;
	MMPF_UART_ATTRIBUTE attr;
	struct ait_uart_data *hw_cfg = to_ait_uart_data(port);
	
	MMPF_Uart_SetInterruptEnable(port->line, MMPF_UART_EVENT_RXFIFO_OVERTH,
	                                MMP_TRUE);
										

	
	//MMPF_Uart_SetInterruptEnable(DEBUG_UART_NUM, MMPF_UART_EVENT_RXFIFO_OVERTH,
	                               // MMP_TRUE);
	MMPF_Uart_Reset(port->line, MMPF_UART_DIRECTION_RX);
	{
	 	attr.bParityEn         = MMP_FALSE;
		attr.parity            = MMPF_UART_PARITY_ODD;
		attr.bFlowCtlEn        = MMP_FALSE;
		attr.ubFlowCtlSelect   = 1;
	    attr.padset            = hw_cfg->pad;
    attr.ulMasterclk       = MMPF_PLL_GetGroup0Freq(0);
	
		attr.ulBaudrate = hw_cfg->baudrate;

		//MMPF_Uart_Open(hw_cfg->uart_hw_id,&attr);
		hw_cfg->hw_status = UART_READY;
	}
	
	MMPF_Uart_Open( port->line, &attr );
	MMPF_Uart_ConfigRx( port->line, 1 );
		
	/*
	 * Ensure that no interrupts are enabled otherwise when
	 * request_irq() is called we could get stuck trying to
	 * handle an unexpected interrupt
	 */

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, ait_uart_int_handler, IRQF_SHARED,
			tty ? tty->name : "ait_serial", port);
	if (retval) {
		printk("ait_serial: ait_startup - Can't get irq\n");
		return retval;
	}

	/*
	 * If there is a specific "open" function (to register
	 * control line interrupts)
	 */
	if (ait_open_hook) {
		retval = ait_open_hook(port);
		if (retval) {
			free_irq(port->irq, port);
			return retval;
		}
	}

	/* Save current CSR for comparison in ait_uart_tasklet_func() */

	/*
	 * Finally, enable the serial port
	 */

	return 0;
}

/*
 * Disable the port
 */
static void ait_shutdown(struct uart_port *port)
{	
	struct ait_uart_port *atmel_port = to_ait_uart_port(port);
	struct ait_uart_data *hw_cfg = to_ait_uart_data(port);
	int retry = 0;
	/*
	 * Ensure everything is stopped.
	 */
#if 0
	down(&atmel_port->tasklet_sem);
	atmel_port->tasklet_exit = 1;
	up(&atmel_port->tasklet_sem);

	while( UART_GET_TXCNT(port) < hw_cfg->fifo_size && retry++<10 )
	{
		msleep(1);
	}
#endif
	while (!MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_TXEMPTY)) {
		cpu_relax();
	}

	ait_stop_rx(port);
	ait_stop_tx(port);

	/*
	 * Free the interrupt
	 */
	free_irq(port->irq, port);

	/*
	 * If there is a specific "close" function (to unregister
	 * control line interrupts)
	 */
	if (ait_close_hook)
	    ait_close_hook(port);
}

/*
 * Flush any TX data submitted for DMA. Called when the TX circular
 * buffer is reset.
 */
static void ait_flush_buffer(struct uart_port *port)
{
}

/*
 * Power / Clock management.
 */
static void ait_serial_pm(struct uart_port *port, unsigned int state,
			    unsigned int oldstate)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	switch (state) {
	case 0:
		/*
		 * Enable the peripheral clock for this serial port.
		 * This is called on uart_open() or a resume event.
		 */
		clk_enable(ait_port->clk);

		/* re-enable interrupts if we disabled some on suspend */
		break;
	case 3:
		/* Back up the interrupt mask and disable all interrupts */

		/*
		 * Disable the peripheral clock for this serial port.
		 * This is called on uart_close() or a suspend event.
		 */
		clk_disable(ait_port->clk);
		break;
	default:
		printk(KERN_ERR "ait_serial: unknown pm %d\n", state);
	}
}

/*
 * Change the port parameters
 */
static void ait_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	//unsigned long flags;
	//unsigned int quot, baud;

	//baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	//quot = uart_get_divisor(port, baud);

	//spin_lock_irqsave(&port->lock, flags);

	//spin_unlock_irqrestore(&port->lock, flags);
}

static void ait_set_ldisc(struct uart_port *port, int new)
{
	int line = port->line;

	if (line >= port->state->port.tty->driver->num)
		return;

	if (port->state->port.tty->ldisc->ops->num == N_PPS) {
		port->flags |= UPF_HARDPPS_CD;
		ait_enable_ms(port);
	} else {
		port->flags &= ~UPF_HARDPPS_CD;
	}
}

/*
 * Return string describing the specified port
 */
static const char *ait_type(struct uart_port *port)
{
	return (port->type == PORT_ATMEL) ? "ATMEL_SERIAL" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void ait_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	release_mem_region(port->mapbase, size);

	if (port->flags & UPF_IOREMAP) {
		iounmap(port->membase);
		port->membase = NULL;
	}
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int ait_request_port(struct uart_port *port)
{
    #if (IOREMAP_USE == 1)
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(port->mapbase, size, "ait_serial"))
		return -EBUSY;

	if (port->flags & UPF_IOREMAP) {
		port->membase = ioremap(port->mapbase, size);
		if (port->membase == NULL) {
			release_mem_region(port->mapbase, size);
			return -ENOMEM;
		}
	}
    #endif

	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void ait_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_ATMEL;
		ait_request_port(port);
	}
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int ait_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_ATMEL)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
    #if (IOREMAP_USE == 1)
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
    #endif
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
static int ait_poll_get_char(struct uart_port *port)
{
    MMP_UBYTE ch = 0;

    while (MMPF_Uart_TryRead(port->line, &ch, 1) == 0) {
		cpu_relax();
    }
	return ch;
}

static void ait_poll_put_char(struct uart_port *port, unsigned char ch)
{
    while (!MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_TXUNDERTH))
	    cpu_relax();

    MMPF_Uart_Write(port->line, &ch, 1);
}
#endif

static struct uart_ops ait_pops = {
	.tx_empty	= ait_tx_empty,
	.set_mctrl	= ait_set_mctrl,
	.get_mctrl	= ait_get_mctrl,
	.stop_tx	= ait_stop_tx,
	.start_tx	= ait_start_tx,//used
	.stop_rx	= ait_stop_rx,
	.enable_ms	= ait_enable_ms,
	.break_ctl	= ait_break_ctl,
	.startup	= ait_startup,//used
	.shutdown	= ait_shutdown,//used
	.flush_buffer	= ait_flush_buffer,
	.set_termios	= ait_set_termios,//used
	.set_ldisc	= ait_set_ldisc,
	.type		= ait_type,
	.release_port	= ait_release_port,
	.request_port	= ait_request_port,
	.config_port	= ait_config_port,
	.verify_port	= ait_verify_port,
	.pm		    = ait_serial_pm,
	//.ioctl		= ait_ioctl,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= ait_poll_get_char,
	.poll_put_char	= ait_poll_put_char,
#endif
};


/*
 * Configure the port from the platform device resource info.
 */
static void __devinit ait_init_port(struct ait_uart_port *ait_port,
				      struct platform_device *pdev)
{
	struct uart_port *port = &ait_port->uart;
	struct ait_uart_data *pdata = pdev->dev.platform_data;

	port->iotype    = UPIO_MEM;
	port->flags		= UPF_BOOT_AUTOCONF;
	port->ops		= &ait_pops;
	port->fifosize  = 1;
	port->dev		= &pdev->dev;
	port->mapbase	= pdev->resource[0].start;
	port->irq	    = pdev->resource[1].start;

	tasklet_init(&ait_port->tasklet, ait_uart_tasklet_func,
			(unsigned long)port);

	memset(&ait_port->rx_ring, 0, sizeof(ait_port->rx_ring));

	if (pdata && pdata->regs) {
		/* Already mapped by setup code */
		port->membase = pdata->regs;
	} else {
		port->flags	|= UPF_IOREMAP;
		port->membase	= NULL;
	}


	/* for console, the clock could already be configured */

	if (!ait_port->clk) {
	    ait_port->clk = clk_get(&pdev->dev, "usart");
		clk_enable(ait_port->clk);
		port->uartclk = clk_get_rate(ait_port->clk);
		clk_disable(ait_port->clk);
		/* only enable clock when USART is in use */
	}
}

#ifdef CONFIG_SERIAL_AIT_CONSOLE

extern void putstr(const char *ptr);

static void ait_console_putchar(struct uart_port *port, int ch)
{
    MMP_UBYTE c = ch;

    while (!MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_TXEMPTY)) {
        cpu_relax();
    }

    MMPF_Uart_Write(port->line, &c, 1);
}


/*
 * Interrupts are disabled on entering
 */
static void ait_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *port = &ait_usports[co->index].uart;

	uart_console_write(port, s, count, ait_console_putchar);
}

static int __init ait_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &ait_usports[co->index].uart;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (port->membase == NULL) {
		/* Port not initialized yet - delay setup */
		return -ENODEV;
	}

	clk_enable(ait_usports[co->index].clk);
	
	if (options) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	} else {
        /*
         * If the port was already initialised (eg, by a boot loader),
         * we may determine from the current HW setup. TBD.
         */
	}

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver ait_uart;

static struct console ait_console = {
	.name		= AIT_CONSOLE_DEVICE,
	.write		= ait_console_write,
	.device		= uart_console_device,
	.setup		= ait_console_setup,
	.flags		= CON_PRINTBUFFER|CON_CONSDEV|CON_ENABLED/*|CON_BOOT*/|CON_ANYTIME|CON_BRL,
	.index		= -1,
	.data		= &ait_uart,
};

/*
 * Early console initialization (before VM subsystem initialized).
 */
static int __init ait_console_init(void)
{
	if (ait_default_console_device) {
		struct ait_uart_data *pdata = ait_default_console_device->dev.platform_data;
		int id = pdata->num;
		struct ait_uart_port *port = &ait_usports[id];

		port->backup_imr = 0;
		port->uart.line = id;
		ait_console.index = id;

		extern MMP_ULONG MMPF_PLL_GetGroup0Freq(MMP_ULONG MClk);
		//	struct tty_struct *tty = port->uart->state->port.tty;
		int retval;
		MMPF_UART_ATTRIBUTE attr;
		struct ait_uart_data *hw_cfg = to_ait_uart_data(&port->uart);

		MMPF_Uart_SetInterruptEnable(port->uart.line, MMPF_UART_EVENT_RXFIFO_OVERTH, MMP_TRUE);



		//MMPF_Uart_SetInterruptEnable(DEBUG_UART_NUM, MMPF_UART_EVENT_RXFIFO_OVERTH,
		// MMP_TRUE);
		//MMPF_Uart_Reset(port->line, MMPF_UART_DIRECTION_RX);
		{
		attr.bParityEn         = MMP_FALSE;
		attr.parity            = MMPF_UART_PARITY_ODD;
		attr.bFlowCtlEn        = MMP_FALSE;
		attr.ubFlowCtlSelect   = 1;
		attr.padset            = pdata->pad;
		attr.ulMasterclk       = MMPF_PLL_GetGroup0Freq(0);

		attr.ulBaudrate = pdata->baudrate;

		//MMPF_Uart_Open(hw_cfg->uart_hw_id,&attr);
		pdata->hw_status = UART_READY;
		}

		MMPF_Uart_Open( port->uart.line, &attr );

	
		add_preferred_console(AIT_CONSOLE_DEVICE, id, NULL);
		ait_init_port(port, ait_default_console_device);
		register_console(&ait_console);
	}

	return 0;
}

console_initcall(ait_console_init);

/*
 * Late console initialization.
 */
static int __init ait_late_console_init(void)
{
	if (ait_default_console_device
	    && !(ait_console.flags & CON_ENABLED))
		register_console(&ait_console);

	return 0;
}

core_initcall(ait_late_console_init);

static inline bool ait_is_console_port(struct uart_port *port)
{
	return port->cons && port->cons->index == port->line;
}

#else
static inline bool ait_is_console_port(struct uart_port *port)
{
	return false;
}
#endif

static struct uart_driver ait_uart = {
	.owner      = THIS_MODULE,
	.driver_name = "ait_serial",
	.dev_name   = AIT_CONSOLE_DEVICE,
	.major      = SERIAL_AIT_MAJOR,
	.minor      = MINOR_START,
	.nr         = AIT_MAX_UART,
	.cons       = (&ait_console),
};

#ifdef CONFIG_PM
extern int ait_suspend_entering_slow_clock(void);
static bool ait_serial_clk_will_stop(void)
{

	return ait_suspend_entering_slow_clock();
}

static int ait_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	if (ait_is_console_port(port) && console_suspend_enabled) {
		/* Drain the TX shifter */
		while (!MMPF_Uart_CheckState(port->line, MMPF_UART_STAT_TXEMPTY))
			cpu_relax();
	}

	/* we can not wake up if we're running on slow clock */
	ait_port->may_wakeup = device_may_wakeup(&pdev->dev);
	if (ait_serial_clk_will_stop())
		device_set_wakeup_enable(&pdev->dev, 0);

	uart_suspend_port(&ait_uart, port);

	return 0;
}

static int ait_serial_resume(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	uart_resume_port(&ait_uart, port);
	device_set_wakeup_enable(&pdev->dev, ait_port->may_wakeup);

	return 0;
}
#else
#define ait_serial_suspend NULL
#define ait_serial_resume NULL
#endif

static int __devinit ait_serial_probe(struct platform_device *pdev)
{
	struct ait_uart_port *port;
	struct device_node *np = pdev->dev.of_node;
	struct ait_uart_data *pdata = pdev->dev.platform_data;
	void *data;
	int ret = -ENODEV;

	BUILD_BUG_ON(AIT_RX_RINGSIZE & (AIT_RX_RINGSIZE - 1));

	if (np)
		ret = of_alias_get_id(np, "serial");
	else
		if (pdata)
			ret = pdata->num;

	if (ret < 0)
		/* port id not found in platform data nor device-tree aliases:
		 * auto-enumerate it */
		ret = find_first_zero_bit(&ait_usports_in_use,
				sizeof(ait_usports_in_use));

	if (ret > AIT_MAX_UART) {
		ret = -ENODEV;
		goto err;
	}

	if (test_and_set_bit(ret, &ait_usports_in_use)) {
		/* port already in use */
		ret = -EBUSY;
		goto err;
	}

	port = &ait_usports[ret];
	port->backup_imr = 0;
	port->uart.line = ret;

	ait_init_port(port, pdev);

    ret = -ENOMEM;
    data = kmalloc(sizeof(struct ait_uart_chdesc)
            * AIT_RX_RINGSIZE, GFP_KERNEL);
    if (!data)
        goto err_alloc_ring;
    port->rx_ring.buf = data;

	ret = uart_add_one_port(&ait_uart, &port->uart);
	if (ret)
		goto err_add_port;

    #ifdef CONFIG_SERIAL_AIT_CONSOLE
	if (ait_is_console_port(&port->uart)
			&& ait_console.flags & CON_ENABLED) {
		/*
		 * The serial core enabled the clock for us, so undo
		 * the clk_enable() in ait_console_setup()
		 */
		clk_disable(port->clk);
	}
    #endif

	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);

	return 0;

err_add_port:
	kfree(port->rx_ring.buf);
	port->rx_ring.buf = NULL;
err_alloc_ring:
	if (!ait_is_console_port(&port->uart)) {
		clk_put(port->clk);
		port->clk = NULL;
	}
err:
	return ret;
}

static int __devexit ait_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	int ret = 0;

	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);

	ret = uart_remove_one_port(&ait_uart, port);

	tasklet_kill(&ait_port->tasklet);
	kfree(ait_port->rx_ring.buf);

	/* "port" is allocated statically, so we shouldn't free it */

	clear_bit(port->line, &ait_usports_in_use);

	clk_put(ait_port->clk);

	return ret;
}

static struct platform_driver ait_serial_driver = {
	.probe		= ait_serial_probe,
	.remove		= __devexit_p(ait_serial_remove),
	.suspend	= ait_serial_suspend,
	.resume		= ait_serial_resume,
	.driver		= {
		.name	= "ait_usart",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ait_serial_dt_ids),
	},
};

static int __init ait_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&ait_uart);
	if (ret)
		return ret;

	ret = platform_driver_register(&ait_serial_driver);
	if (ret)
		uart_unregister_driver(&ait_uart);

	return ret;
}

static void __exit ait_serial_exit(void)
{
	platform_driver_unregister(&ait_serial_driver);
	uart_unregister_driver(&ait_uart);
}

module_init(ait_serial_init);
module_exit(ait_serial_exit);

MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("AIT serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ait_usart");
