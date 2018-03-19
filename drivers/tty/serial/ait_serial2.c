/*
 *  Driver for Atmel AT91 / AT32 Serial ports
 *  Copyright (C) 2003 Rick Bronson
 *
 *  Based on drivers/char/serial_sa1100.c, by Deep Blue Solutions Ltd.
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  DMA support added by Chip Coldwell.
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
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/ioctls.h>

#include <asm/mach/serial_at91.h>
#include <mach/board.h>

#define USE_SEM_INSTEAD_SPINLOCK 0	//use 

#ifdef CONFIG_ARM
#include <mach/cpu.h>
#include <asm/gpio.h>
#endif

#define PDC_BUFFER_SIZE		512
/* Revisit: We should calculate this based on the actual port settings */
#define PDC_RX_TIMEOUT		(3 * 10)		/* 3 bytes */

//#if defined(CONFIG_SERIAL_ATMEL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#if defined(CONFIG_SERIAL_AIT_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

/*Vin@ add for debugging*/
#include <mach/mmp_reg_uart.h>
#include <mach/mmpf_uart.h>


static void ait_start_rx(struct uart_port *port);
static void ait_stop_rx(struct uart_port *port);

#ifdef CONFIG_SERIAL_AIT_CONSOLE 
static inline bool ait_is_console_port(struct uart_port *port);
#endif 

#if 0
//#ifdef CONFIG_SERIAL_AIT_TTYAT
/* Use device name ttyAT, major 204 and minor 154-169.  This is necessary if we
 * should coexist with the 8250 driver, such as if we have an external 16C550
 * UART. */
#define SERIAL_AIT_MAJOR	204
#define MINOR_START		154
#define AITL_DEVICENAME	"ttyAT"

#endif

/* Use device name ttyS, major 4, minor 64-68.  This is the usual serial port
 * name, but it is legally reserved for the 8250 driver. */
#define SERIAL_AIT_MAJOR	TTY_MAJOR
#define MINOR_START		64
#define AIT_CONSOLE_DEVICE	"ttyS"

//#endif



extern void printch(int);

#define AIT_US_RXRDY 0x01
#define AIT_US_TXRDY 0x02
#define AIT_US_RXBRK 0x04
#define AIT_ISR_PASS_LIMIT	0 //256

/* UART registers. CR is write-only, hence no GET macro */
//#define UART_PUT_CR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_CR)
//#define UART_GET_MR(port)	//__raw_readl((port)->membase + ATMEL_US_MR)
//#define UART_PUT_MR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_MR)
//#define UART_PUT_IER(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_IER)
//#define UART_PUT_IDR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_IDR)
#define UART_GET_IMR(port)	//__raw_readl((port)->membase + ATMEL_US_IMR)
//#define UART_GET_CSR(port)	(AITC_BASE_UARTB->US[0].US_ISR)//__raw_readl((port)->membase + ATMEL_US_CSR)
#define UART_GET_CSR(port)	(((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_ISR)

#if (CHIP == VSN_V3)
#define UART_GET_TXCNT(port) ((( (struct ait_uart_data *) (port)->dev->platform_data ))->pUS->US_TX_FIFO_DATA_CNT )
#define UART_GET_RXCNT(port) ((( (struct ait_uart_data *) (port)->dev->platform_data ))->pUS->US_RX_FIFO_DATA_CNT )
#define UART_GET_CHAR(port)	 	(((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_RXPR)
#define UART_PUT_CHAR(port,v)	do{ while((((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_TX_FIFO_DATA_CNT==0));\
															           ((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_TXPR = v;\
																		}while(0)
#endif 


#if (CHIP == MCR_V2)
#define UART_GET_TXCNT(port) (((( (struct ait_uart_data *) (port)->dev->platform_data ))->pUS->US_FSR & US_TX_FIFO_UNWR_MASK) >> 8 )
#define UART_GET_RXCNT(port) ((( (struct ait_uart_data *) (port)->dev->platform_data ))->pUS->US_FSR & US_RX_FIFO_UNRD_MASK )
#define UART_GET_CHAR(port)	 	(((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_RXPR)
#define UART_PUT_CHAR(port,v)	do{ while(UART_GET_TXCNT(port)==0);\
								((struct ait_uart_data *) (port)->dev->platform_data )->pUS->US_TXPR = v;\
								}while(0)
#endif 


#define UART_GET_BRGR(port)	//__raw_readl((port)->membase + ATMEL_US_BRGR)
#define UART_PUT_BRGR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_BRGR)
#define UART_PUT_RTOR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_US_RTOR)
#define UART_PUT_TTGR(port, v)	//__raw_writel(v, (port)->membase + ATMEL_US_TTGR)

 /* PDC registers */
#define UART_PUT_PTCR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_PTCR)
#define UART_GET_PTSR(port)	//__raw_readl((port)->membase + ATMEL_PDC_PTSR)

#define UART_PUT_RPR(port,v)//	__raw_writel(v, (port)->membase + ATMEL_PDC_RPR)
#define UART_GET_RPR(port)	//__raw_readl((port)->membase + ATMEL_PDC_RPR)
#define UART_PUT_RCR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_RCR)
#define UART_PUT_RNPR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_RNPR)
#define UART_PUT_RNCR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_RNCR)

#define UART_PUT_TPR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_TPR)
#define UART_PUT_TCR(port,v)	//__raw_writel(v, (port)->membase + ATMEL_PDC_TCR)
#define UART_GET_TCR(port)	//__raw_readl((port)->membase + ATMEL_PDC_TCR)
//#define UART_GET_TXCNT(port) (AITC_BASE_UARTB->US[0].US_TX_FIFO_DATA_CNT)

static int (*ait_open_hook)(struct uart_port *);
static void (*ait_close_hook)(struct uart_port *);
#if 0
struct ait_dma_buffer {
	unsigned char	*buf;
	dma_addr_t	dma_addr;
	unsigned int	dma_size;
	unsigned int	ofs;
};
#endif 
struct ait_uart_char {
	u16		status;
	u16		ch;
};

#define AIT_SERIAL_RINGSIZE 1024

/*
 * We wrap our port structure around the generic uart_port.
 */
struct ait_uart_port {
	struct uart_port	uart;		/* uart */
	struct clk		*clk;		/* uart clock */
	int			may_wakeup;	/* cached value of device_may_wakeup for times we need to disable it */
	u32			backup_imr;	/* IMR saved during suspend */
	int			break_active;	/* break being received */

	struct tasklet_struct	tasklet;
	unsigned int		irq_status;
	unsigned int		irq_status_prev;

#if USE_SEM_INSTEAD_SPINLOCK
	struct semaphore	tasklet_sem;
	u8 tasklet_sem_inited;
#endif
	u8 tasklet_exit;
	struct circ_buf		rx_ring;

	struct serial_rs485	rs485;		/* rs485 settings */
	unsigned int		tx_done_mask;
};


//static struct ait_uart_port ait_ports[AIT_MAX_UART] = {
static struct ait_uart_port ait_ports[] = {

		{
#if USE_SEM_INSTEAD_SPINLOCK
			.tasklet_sem_inited = 0,
			.tasklet_exit = 0,
#endif
		},
//#if defined(CONFIG_SERIAL_AIT_PORT_1)
		{
		#if USE_SEM_INSTEAD_SPINLOCK
			.tasklet_sem_inited = 0,
			.tasklet_exit = 0,
		#endif
		},
//#endif

//#if defined(CONFIG_SERIAL_AIT_PORT_2)
		{
		#if USE_SEM_INSTEAD_SPINLOCK
			.tasklet_sem_inited = 0,
			.tasklet_exit = 0,
		#endif
		},
//#endif

//#if defined(CONFIG_SERIAL_AIT_PORT_2)
		{
		#if USE_SEM_INSTEAD_SPINLOCK
			.tasklet_sem_inited = 0,
			.tasklet_exit = 0,
		#endif
		},
//#endif

};


static unsigned long ait_ports_in_use;

#ifdef SUPPORT_SYSRQ
static struct console ait_console;
#endif

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

static bool ait_use_dma_rx(struct uart_port *port)
{
	return false;
}

static bool ait_use_dma_tx(struct uart_port *port)
{
	return false;
}



/* Enable or disable the rs485 support */
void ait_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
#if 0
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Disable interrupts */
	UART_PUT_IDR(port, ait_port->tx_done_mask);
	
	ait_port->rs485 = *rs485conf;

	if (rs485conf->flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		ait_port->tx_done_mask = ATMEL_US_TXEMPTY;
		if ((rs485conf->delay_rts_after_send) > 0)
			UART_PUT_TTGR(port, rs485conf->delay_rts_after_send);
	} else {
		dev_dbg(port->dev, "Setting UART to RS232\n");
		if (ait_use_dma_tx(port))
			ait_port->tx_done_mask = ATMEL_US_ENDTX |
				ATMEL_US_TXBUFE;
		else
			ait_port->tx_done_mask = ATMEL_US_TXRDY;
	}

	/* Enable interrupts */
	UART_PUT_IER(port, ait_port->tx_done_mask);

	spin_unlock_irqrestore(&port->lock, flags);
#endif 
}


/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static u_int ait_tx_empty(struct uart_port *port)
{
	struct ait_uart_data *pdata = to_ait_uart_data(port); //(struct ait_uart_data *)port->dev->platform_data;
	unsigned int c = UART_GET_TXCNT(port);
	return c==pdata->fifo_size ? TIOCSER_TEMT:0;
}

/*
 * Set state of the modem control output lines
 */
static void ait_set_mctrl(struct uart_port *port, u_int mctrl)
{
#if 0
	unsigned int control = 0;
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	if (mctrl & TIOCM_RTS)
		control |= ATMEL_US_RTSEN;
	else
		control |= ATMEL_US_RTSDIS;

	if (mctrl & TIOCM_DTR)
		control |= ATMEL_US_DTREN;
	else
		control |= ATMEL_US_DTRDIS;


	/* Local loopback mode? */

	if (ait_port->rs485.flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		if ((ait_port->rs485.delay_rts_after_send) > 0)
			UART_PUT_TTGR(port,
					ait_port->rs485.delay_rts_after_send);
	} else {
		dev_dbg(port->dev, "Setting UART to RS232\n");
	}
#endif 
}

/*
 * Get state of the modem control input lines
 */
static u_int ait_get_mctrl(struct uart_port *port)
{
#if 0
	unsigned int status, ret = 0;

	/*
	 * The control signals are active low.
	 */

	if (!(status & ATMEL_US_DCD))
		ret |= TIOCM_CD;
	if (!(status & ATMEL_US_CTS))
		ret |= TIOCM_CTS;
	if (!(status & ATMEL_US_DSR))
		ret |= TIOCM_DSR;
	if (!(status & ATMEL_US_RI))
		ret |= TIOCM_RI;
	return ret;
#else
	unsigned int status, ret = 0;
	struct ait_uart_data *pdata = to_ait_uart_data(port);
	status = pdata->pUS->US_ISR;

	if (!(status & US_TX_FIFO_EMPTY & US_TX_FIFO_UNDER_THRES))
		ret |= TIOCM_CTS;

	return ret;
#endif 
	return 0;
}

/*
 * Stop transmitting.
 */
static void ait_stop_tx(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	
	/* Disable interrupts */
	//UART_PUT_IDR(port, ait_port->tx_done_mask);

	if ((ait_port->rs485.flags & SER_RS485_ENABLED) &&
	    !(ait_port->rs485.flags & SER_RS485_RX_DURING_TX))
		ait_start_rx(port);
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
 * start receiving - port is in process of being opened.
 */
static void ait_start_rx(struct uart_port *port)
{
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
{
}

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
 * Stores the incoming character in the ring buffer
 */
static void ait_buffer_rx_char(struct uart_port *port, unsigned int status,
		     unsigned int ch)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	struct circ_buf *ring = &ait_port->rx_ring;
	struct ait_uart_char *c;

	if (!CIRC_SPACE(ring->head, ring->tail, AIT_SERIAL_RINGSIZE))
		/* Buffer overflow, ignore char */
		return;

	c = &((struct ait_uart_char *)ring->buf)[ring->head];
	c->status	= status;
	c->ch		= ch;

	/* Make sure the character is stored before we update head. */
	smp_wmb();

	ring->head = (ring->head + 1) & (AIT_SERIAL_RINGSIZE - 1);
}

/*
 * Characters received (called from interrupt handler)
 */
static void ait_rx_chars(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	unsigned int status, ch, nbyte_in_fifo,n;
	//struct ait_uart_data *pdata = to_ait_uart_data(port); //(struct ait_uart_data *)port->dev->platform_data;
	nbyte_in_fifo = UART_GET_RXCNT(port);
	for(n=0;n<nbyte_in_fifo;++n)
	{
		ch = UART_GET_CHAR(port);
		ait_buffer_rx_char(port, status, ch);
#if 0 //enable if you want to dump UART rx data to kernel message
		if(pdata->uart_hw_id == 1)
			printk("Rx:0x%.2X\r\n",ch);
#endif
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
	struct ait_uart_data *pdata = to_ait_uart_data(port); //(struct ait_uart_data *)port->dev->platform_data;

	int txcnt;//,ulLength =1;
	//int timeout = 0;
	int sent =0;

	txcnt = UART_GET_TXCNT(port);

	if (likely(port->x_char&&txcnt) ) {			//	if (port->x_char && UART_GET_CSR(port) & ait_port->tx_done_mask) {

		UART_PUT_CHAR(port, port->x_char);

		port->icount.tx++;
		port->x_char = 0;
		txcnt--;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
	{
		return;
	}

	do{
		for(sent=0;sent<txcnt;sent++)		//UART_GET_CSR(port) & ait_port->tx_done_mask)
		{
			UART_PUT_CHAR(port, xmit->buf[xmit->tail]);
#if 0	//enable if you want to dump uart tx data to kernel message
			if(pdata->uart_hw_id != 0 )
				printk("Tx:0x%.2X\r\n",xmit->buf[xmit->tail]);
#endif
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			if (uart_circ_empty(xmit))
				break;
		}
		txcnt = UART_GET_TXCNT(port);
	}
	while( !uart_circ_empty(xmit) && txcnt);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if( !uart_circ_empty(xmit) )
	{
		struct ait_uart_port *ait_port = to_ait_uart_port(port);
		tasklet_schedule(&ait_port->tasklet);
	}
}

/*
 * receive interrupt handler.
 */
static void
ait_handle_receive(struct uart_port *port, unsigned int pending)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	/* Interrupt receive */
	if (pending & AIT_US_RXRDY)
		ait_rx_chars(port);
	else if (pending & AIT_US_RXBRK) {
		/*
		 * End of break detected. If it came along with a
		 * character, ait_rx_chars will handle it.
		 */
		ait_port->break_active = 0;
	}
}

/*
 * transmit interrupt handler. (Transmit is IRQF_NODELAY safe)
 */
static void
ait_handle_transmit(struct uart_port *port, unsigned int pending)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);

	//if (pending & ait_port->tx_done_mask) 
	{
		/* Either PDC or interrupt transmission */
		tasklet_schedule(&ait_port->tasklet);
	}
}

/*
 * status flags interrupt handler.
 */
static void
ait_handle_status(struct uart_port *port, unsigned int pending,
		    unsigned int status)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	ait_port->irq_status = status;
		tasklet_schedule(&ait_port->tasklet);
#if 0		
	if (pending & (ATMEL_US_RIIC | ATMEL_US_DSRIC | ATMEL_US_DCDIC
				| ATMEL_US_CTSIC)) {
		ait_port->irq_status = status;
		tasklet_schedule(&ait_port->tasklet);
	}
#endif	
}

/*
 * Interrupt handler
 */
static irqreturn_t ait_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status, pending, pass_counter = 0;
	struct ait_uart_data *pdata = to_ait_uart_data(port);//(struct ait_uart_data *)port->dev->platform_data;
	unsigned int intsrc = pdata->pUS->US_ISR;
	intsrc &= pdata->pUS->US_IER;
#if 0
	if(intsrc & US_RX_FIFO_OVER_THRES)
	{
		do {
			pending = AIT_US_RXRDY;
			ait_handle_receive(port, pending);
			ait_handle_status(port, pending, status);
		} while (pass_counter++ < AIT_ISR_PASS_LIMIT);
		//Todo: Even Interrupt not enable interrupt still assert	printk("US_IER = 0x%x\r\n",pUART->US[0].US_IER );
		pdata->pUS->US_ISR = intsrc; //clear ISR
		return pass_counter ? IRQ_HANDLED : IRQ_NONE;
	}else
		return 0;
#else
	if(intsrc & US_RX_FIFO_OVER_THRES)
	{
		pending = AIT_US_RXRDY;
		ait_handle_receive(port, pending);
		ait_handle_status(port, pending, status);
		return IRQ_HANDLED;
	}else
		return IRQ_NONE;
#endif 
}

static void ait_rx_from_ring(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	struct circ_buf *ring = &ait_port->rx_ring;
	unsigned int flg;
	unsigned int status;
	while (ring->head != ring->tail) {
		struct ait_uart_char c;

		/* Make sure c is loaded after head. */
		smp_rmb();

		c = ((struct ait_uart_char *)ring->buf)[ring->tail];

		ring->tail = (ring->tail + 1) & (AIT_SERIAL_RINGSIZE - 1);

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
static void ait_tasklet_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	unsigned int status;
	unsigned int status_change;
#if USE_SEM_INSTEAD_SPINLOCK
	down(&ait_port->tasklet_sem);
	if(ait_port->tasklet_exit)
	{
		up(&ait_port->tasklet_sem);
		return;
	}
#else
	/* The interrupt handler does not take the lock */
	spin_lock(&port->lock);
	if(ait_port->tasklet_exit)
	{
		spin_unlock(&port->lock);
		return;
	}
#endif

	ait_tx_chars(port);

	status = ait_port->irq_status;
	status_change = status ^ ait_port->irq_status_prev;

	wake_up_interruptible(&port->state->port.delta_msr_wait);
	ait_port->irq_status_prev = status;

	ait_rx_from_ring(port);
#if USE_SEM_INSTEAD_SPINLOCK
	up(&ait_port->tasklet_sem);
#else
	spin_unlock(&port->lock);
#endif
}

/*
 * Perform initialization and enable port for reception
 */
unsigned long MMPF_PLL_GetGroup0Freq(void);//unsigned long *ulGroupFreq)
static int ait_startup(struct uart_port *port)
{
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	struct tty_struct *tty = port->state->port.tty;
	struct ait_uart_data *pdata = to_ait_uart_data(port); //(struct ait_uart_data *)port->dev->platform_data;
	int retval;

	MMPF_UART_ATTRIBUTE attr;
	clk_enable(ait_port->clk);
	//if(pdata->uart_hw_id > 0) //since UART0 initialized in u-boot and only for console, we don't need init again
	{
		memset(&attr,0,sizeof(attr));
		//attr.bParityEn = MMP_TRUE;
		attr.padset = pdata->pad;
		if(pdata->baudrate>0)
		{
			attr.ulBaudrate = pdata->baudrate;
			attr.ulMasterclk = MMPF_PLL_GetGroup0Freq() / 2;
			dev_dbg(port->dev,"UART%d, mclk=%d\r\n",pdata->uart_hw_id,attr.ulMasterclk);	
			MMPF_uart_open(pdata->uart_hw_id,&attr);
			pdata->hw_status = UART_READY;
		}
	}
	ait_port->tasklet_exit = 0;

	/*
	 * Ensure that no interrupts are enabled otherwise when
	 * request_irq() is called we could get stuck trying to
	 * handle an unexpected interrupt
	 */

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, ait_interrupt, IRQF_SHARED,
			tty ? tty->name : "ait_serial", port);
	if (retval) {
		printk("ait_serial: ait_startup - Can't get irq\n");
		return retval;
	}

#if 0
	/*
	 * Initialize DMA (if necessary)
	 */
	if (ait_use_dma_rx(port)) {
		int i;

		for (i = 0; i < 2; i++) {
			struct ait_dma_buffer *pdc = &ait_port->pdc_rx[i];

			pdc->buf = kmalloc(PDC_BUFFER_SIZE, GFP_KERNEL);
			if (pdc->buf == NULL) {
				if (i != 0) {
					dma_unmap_single(port->dev,
						ait_port->pdc_rx[0].dma_addr,
						PDC_BUFFER_SIZE,
						DMA_FROM_DEVICE);
					kfree(ait_port->pdc_rx[0].buf);
				}
				free_irq(port->irq, port);
				return -ENOMEM;
			}
			pdc->dma_addr = dma_map_single(port->dev,
						       pdc->buf,
						       PDC_BUFFER_SIZE,
						       DMA_FROM_DEVICE);
			pdc->dma_size = PDC_BUFFER_SIZE;
			pdc->ofs = 0;
		}

		ait_port->pdc_rx_idx = 0;

		UART_PUT_RPR(port, ait_port->pdc_rx[0].dma_addr);
		UART_PUT_RCR(port, PDC_BUFFER_SIZE);

		UART_PUT_RNPR(port, ait_port->pdc_rx[1].dma_addr);
		UART_PUT_RNCR(port, PDC_BUFFER_SIZE);
	}

	if (ait_use_dma_tx(port)) {
		struct ait_dma_buffer *pdc = &ait_port->pdc_tx;
		struct circ_buf *xmit = &port->state->xmit;

		pdc->buf = xmit->buf;
		pdc->dma_addr = dma_map_single(port->dev,
					       pdc->buf,
					       UART_XMIT_SIZE,
					       DMA_TO_DEVICE);
		pdc->dma_size = UART_XMIT_SIZE;
		pdc->ofs = 0;
	}
#endif 

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

	/* Save current CSR for comparison in ait_tasklet_func() */
//	ait_port->irq_status_prev = UART_GET_CSR(port);
	ait_port->irq_status = ait_port->irq_status_prev;

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
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	struct ait_uart_data *pdata = to_ait_uart_data(port);
	int retry = 0;
	/*
	 * Ensure everything is stopped.
	 */

#if USE_SEM_INSTEAD_SPINLOCK
	down(&ait_port->tasklet_sem);
	ait_port->tasklet_exit = 1;
	up(&ait_port->tasklet_sem);
#else
	spin_lock(&port->lock);
	ait_port->tasklet_exit = 1;
	spin_unlock(&port->lock);
#endif

	while( UART_GET_TXCNT(port) < pdata->fifo_size && retry++<10 )
	{
		msleep(1);
	}

	MMPF_Uart_Close(pdata->uart_hw_id);
	pdata->hw_status = UART_UNINIT;

	/*
	 * Disable all interrupts, port and break condition.
	 */

	/*
	 * Free the interrupt
	 */
	free_irq(port->irq, port);

	//disable clock 
	clk_disable(ait_port->clk);
	
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
	//struct ait_uart_port *ait_port = to_ait_uart_port(port);
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
	unsigned long flags;
	unsigned int quot, baud;/*, imr,mode, */
	struct ait_uart_data *pdata = to_ait_uart_data(port);
	//struct ait_uart_port *ait_port = to_ait_uart_port(port);

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);
	
	pdata->baudrate = baud;
	//pdata->baudrate = 115200;
	
	dev_dbg(port->dev,"Change Baudrate = %d\r\n",baud);
	//if( pdata->hw_status == UART_READY)
	if(!ait_is_console_port(port))
	{
		MMPF_UART_ATTRIBUTE attr;
		memset(&attr,0,sizeof(attr));
	#if USE_SEM_INSTEAD_SPINLOCK
		down(&ait_port->tasklet_sem);
	#else
		spin_lock(&port->lock);
	#endif
		//attr.bParityEn = MMP_TRUE;
		attr.padset = pdata->pad;
		attr.ulBaudrate = pdata->baudrate;
		attr.ulMasterclk = MMPF_PLL_GetGroup0Freq() / 2;

	
		attr.bFlowCtlEn = 0;
		if (termios->c_cflag & PARENB)
			attr.bParityEn = MMPF_UART_PARITY_EVEN;
		else if (termios->c_cflag & PARODD)
			attr.bParityEn = MMPF_UART_PARITY_ODD;

		if (termios->c_cflag & CRTSCTS)
		{
			attr.bFlowCtlEn = 1;
			attr.ubFlowCtlSelect = MMP_FALSE;
		}
		dev_dbg(port->dev,"UART%d, mclk=%d\r\n",pdata->uart_hw_id,attr.ulMasterclk);

		MMPF_Uart_Close(pdata->uart_hw_id);
		MMPF_uart_open(pdata->uart_hw_id,&attr);
	#if USE_SEM_INSTEAD_SPINLOCK
		up(&ait_port->tasklet_sem);
	#else
		spin_unlock(&port->lock);
	#endif
	}

	spin_lock_irqsave(&port->lock, flags);

	/* CTS flow-control and modem-status interrupts */
	if (UART_ENABLE_MS(port, termios->c_cflag))
		port->ops->enable_ms(port);

	spin_unlock_irqrestore(&port->lock, flags);
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
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
x
static int ait_poll_get_char(struct uart_port *port)
{
	while (!(UART_GET_CSR(port) & ATMEL_US_RXRDY))
		cpu_relax();

	return UART_GET_CHAR(port);
}

static void ait_poll_put_char(struct uart_port *port, unsigned char ch)
{
//	while (!(UART_GET_CSR(port) & ATMEL_US_TXRDY))
//		cpu_relax();

	UART_PUT_CHAR(port, ch);
}
#endif

static int
ait_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct serial_rs485 rs485conf;

	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

		ait_config_rs485(port, &rs485conf);
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_ait_uart_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}



static struct uart_ops ait_pops = {
	.tx_empty	= ait_tx_empty,
	.set_mctrl	= ait_set_mctrl,
	.get_mctrl	= ait_get_mctrl,
	.stop_tx	= ait_stop_tx,
	.start_tx	= ait_start_tx,
	.stop_rx	= ait_stop_rx,
	.enable_ms	= ait_enable_ms,
	.break_ctl	= ait_break_ctl,
	.startup	= ait_startup,
	.shutdown	= ait_shutdown,
	.flush_buffer	= ait_flush_buffer,
	.set_termios	= ait_set_termios,
	.set_ldisc	= ait_set_ldisc,
	.type		= ait_type,
	.release_port	= ait_release_port,
	.request_port	= ait_request_port,
	.config_port	= ait_config_port,
	.verify_port	= ait_verify_port,
	.pm		= ait_serial_pm,
	.ioctl		= ait_ioctl,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= ait_poll_get_char,
	.poll_put_char	= ait_poll_put_char,
#endif
};

static void __devinit ait_of_init_port(struct ait_uart_port *ait_port,
					 struct device_node *np)
{
#if 0
	u32 rs485_delay[2];

	/* DMA/PDC usage specification */
	if (of_get_property(np, "ait,use-dma-rx", NULL))
		ait_port->use_dma_rx	= 1;
	else
		ait_port->use_dma_rx	= 0;
	if (of_get_property(np, "ait,use-dma-tx", NULL))
		ait_port->use_dma_tx	= 1;
	else
		ait_port->use_dma_tx	= 0;

	/* rs485 properties */
	if (of_property_read_u32_array(np, "rs485-rts-delay",
					    rs485_delay, 2) == 0) {
		struct serial_rs485 *rs485conf = &ait_port->rs485;

		rs485conf->delay_rts_before_send = rs485_delay[0];
		rs485conf->delay_rts_after_send = rs485_delay[1];
		rs485conf->flags = 0;

		if (of_get_property(np, "rs485-rx-during-tx", NULL))
			rs485conf->flags |= SER_RS485_RX_DURING_TX;

		if (of_get_property(np, "linux,rs485-enabled-at-boot-time", NULL))
			rs485conf->flags |= SER_RS485_ENABLED;
	}
#endif

}

/*
 * Configure the port from the platform device resource info.
 */
static void __devinit ait_init_port(struct ait_uart_port *ait_port,
				      struct platform_device *pdev)
{
	struct uart_port *port = &ait_port->uart;
	struct ait_uart_data *pdata = pdev->dev.platform_data;
#if USE_SEM_INSTEAD_SPINLOCK
	if(!ait_port->tasklet_sem_inited)
	{
		sema_init(&ait_port->tasklet_sem,1);
		ait_port->tasklet_sem_inited = 1;
	}
#endif
	ait_port->tasklet_exit = 0;

	port->iotype		= UPIO_MEM;
	port->flags		= UPF_BOOT_AUTOCONF;
	port->ops		= &ait_pops;
	port->fifosize		= 1;
	port->dev		= &pdev->dev;
	port->mapbase	= pdev->resource[0].start;
	port->irq	= pdev->resource[1].start;

	tasklet_init(&ait_port->tasklet, ait_tasklet_func,
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
		//ait_port->clk = clk_get(&pdev->dev, "usart");
		ait_port->clk = clk_get(&pdev->dev, "usart0_clk");
		clk_enable(ait_port->clk);
		port->uartclk = clk_get_rate(ait_port->clk);
		clk_disable(ait_port->clk);
		/* only enable clock when USART is in use */
	}

}

/*
 * Register board-specific modem-control line handlers.
 */
 /*
void __init ait_register_uart_fns(struct ait_port_fns *fns)
{

	if (fns->enable_ms)
		ait_pops.enable_ms = fns->enable_ms;
	if (fns->get_mctrl)
		ait_pops.get_mctrl = fns->get_mctrl;
	if (fns->set_mctrl)
		ait_pops.set_mctrl = fns->set_mctrl;		
	ait_open_hook		= fns->open;
	ait_close_hook	= fns->close;
	ait_pops.pm		= fns->pm;
	ait_pops.set_wake	= fns->set_wake;

}
*/

//#ifdef CONFIG_SERIAL_ATMEL_CONSOLE
#ifdef CONFIG_SERIAL_AIT_CONSOLE

extern void putstr(const char *ptr);

static void ait_console_putchar(struct uart_port *port, int ch)
{
	//struct ait_uart_data *hw_cfg = (struct ait_uart_data *)port->dev->platform_data;
#if 0
	while(UART_GET_TXCNT(port)==0)
		cpu_relax();
	UART_PUT_CHAR(port,ch);
#endif
	unsigned int c = UART_GET_TXCNT(port);
	while(c==0)
	{
		cpu_relax();
		c = UART_GET_TXCNT(port);
	}
	UART_PUT_CHAR(port,ch);
}


/*
 * Interrupts are disabled on entering
 */
static void ait_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *port = &ait_ports[co->index].uart;
	/*
	 * First, save IMR and then disable interrupts
	 */

	uart_console_write(port, s, count, ait_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore IMR
	 */
}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init ait_console_get_options(struct uart_port *port, int *baud,
					     int *parity, int *bits)
{
}

static int __init ait_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &ait_ports[co->index].uart;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (port->membase == NULL) {
		/* Port not initialized yet - delay setup */
		return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		ait_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver ait_uart;

static struct console ait_console = {
	.name		= AIT_CONSOLE_DEVICE,//ATMEL_DEVICENAME,
	.write		= ait_console_write,
	.device		= uart_console_device,
	.setup		= ait_console_setup,
	.flags		= CON_PRINTBUFFER|CON_PRINTBUFFER|CON_CONSDEV|CON_ENABLED/*|CON_BOOT*/|CON_ANYTIME|CON_BRL,
	.index		= -1,
	.data		= &ait_uart,
};

#define AIT_CONSOLE_DEVICE	(&ait_console)

/*
 * Early console initialization (before VM subsystem initialized).
 */
static int __init ait_console_init(void)
{
  
//  putstr("ait_console_init+\r\n");	
	if (ait_default_console_device) {
		struct ait_uart_data *pdata =
			ait_default_console_device->dev.platform_data;
		int id = pdata->num;
		struct ait_uart_port *port = &ait_ports[id];

		port->backup_imr = 0;
		port->uart.line = id;

		add_preferred_console(AIT_CONSOLE_DEVICE, id, NULL);
		ait_init_port(port, ait_default_console_device);	
		 
		//enable uart HW 
		clk_enable(port->clk);
		
		if(pdata->uart_hw_id != MMPF_UART_ID_0) //since UART0 initialized in u-boot and only for console, we don't need init again
		{
			MMPF_UART_ATTRIBUTE attr;
			memset(&attr,0,sizeof(attr));
			attr.padset = pdata->pad;
			attr.ulBaudrate = pdata->baudrate;
			attr.ulMasterclk = MMPF_PLL_GetGroup0Freq() / 2;
			//dev_dbg(port->dev,"UART%d, mclk=%d\r\n",pdata->uart_hw_id,attr.ulMasterclk);		
			MMPF_uart_open(pdata->uart_hw_id,&attr);
		}		
		pdata->hw_status = UART_CONSOLE;//UART_READY;// 
		
		register_console(&ait_console);
	}
//putstr("ait_console_init-\r\n");	
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
x
#define AIT_CONSOLE_DEVICE	NULL

static inline bool ait_is_console_port(struct uart_port *port)
{
	return false;
}
#endif

static struct uart_driver ait_uart = {
	.owner		= THIS_MODULE,
	.driver_name	= "ait_serial",
	.dev_name	=  AIT_CONSOLE_DEVICE,
	.major		= SERIAL_AIT_MAJOR,
	.minor		= MINOR_START,
	.nr			= AIT_MAX_UART,
	.cons		= AIT_CONSOLE_DEVICE,
};

#ifdef CONFIG_PM
static bool ait_serial_clk_will_stop(void)
{
#ifdef CONFIG_ARCH_AT91
	return at91_suspend_entering_slow_clock();
#else
	return false;
#endif
}

static int ait_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct ait_uart_port *ait_port = to_ait_uart_port(port);
	printk("%s\r\n",__FUNCTION__);
#if 0
	if (ait_is_console_port(port) && console_suspend_enabled) {
		/* Drain the TX shifter */
		//while (!(UART_GET_CSR(port) & ATMEL_US_TXEMPTY))
		while (!MMPF_Uart_CheckState(port->uart_hw_id, MMPF_UART_STAT_TXEMPTY))
			cpu_relax();
	}
#endif
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
	printk("%s\r\n",__FUNCTION__);
#if 0	
	uart_resume_port(&ait_uart, port);
#endif
	if (ait_serial_clk_will_stop())
		device_set_wakeup_enable(&pdev->dev, ait_port->may_wakeup);
	
	uart_resume_port(&ait_uart, port);
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

	BUILD_BUG_ON(AIT_SERIAL_RINGSIZE & (AIT_SERIAL_RINGSIZE - 1));

	pdata->pUS = GetpUS(pdata->uart_hw_id); //get controller register address

	if (np)
		ret = of_alias_get_id(np, "serial");
	else
		if (pdata)
			ret = pdata->num;

	if (ret < 0)
		/* port id not found in platform data nor device-tree aliases:
		 * auto-enumerate it */
		ret = find_first_zero_bit(&ait_ports_in_use,
				sizeof(ait_ports_in_use)*BITS_PER_BYTE);

	if (ret > AIT_MAX_UART) {
		ret = -ENODEV;
		goto err;
	}

	if (test_and_set_bit(ret, &ait_ports_in_use)) {
		/* port already in use */
		ret = -EBUSY;
		goto err;
	}

	port = &ait_ports[ret];
	port->backup_imr = 0;
	port->uart.line = ret;

	ait_init_port(port, pdev);

	//if (!ait_use_dma_rx(&port->uart)) {
		ret = -ENOMEM;
		data = kmalloc(sizeof(struct ait_uart_char)
				* AIT_SERIAL_RINGSIZE, GFP_KERNEL);
		if (!data)
			goto err_alloc_ring;
		port->rx_ring.buf = data;
	//}

	ret = uart_add_one_port(&ait_uart, &port->uart);
	if (ret)
		goto err_add_port;

//#ifdef CONFIG_SERIAL_ATMEL_CONSOLE
#ifdef CONFIG_SERIAL_AIT_CONSOLE
	if (ait_is_console_port(&port->uart)
			&& AIT_CONSOLE_DEVICE->flags & CON_ENABLED) {
		/*
		 * The serial core enabled the clock for us, so undo
		 * the clk_enable() in ait_console_setup()
		 */
		clk_disable(port->clk);
	}
#endif

	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);

	//if (port->rs485.flags & SER_RS485_ENABLED) {
	//	UART_PUT_MR(&port->uart, ATMEL_US_USMODE_NORMAL);
	//	UART_PUT_CR(&port->uart, ATMEL_US_RTSEN);
	//}

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


	/* "port" is allocated statically, so we shouldn't free it */

	clear_bit(port->line, &ait_ports_in_use);

	clk_put(ait_port->clk);

	return ret;
}

static struct platform_driver ait_serial_driver = {
	.probe		= ait_serial_probe,
	.remove		= __devexit_p(ait_serial_remove),
	.suspend		= ait_serial_suspend,
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
MODULE_DESCRIPTION("AIT Vision V3 serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ait_usart");
