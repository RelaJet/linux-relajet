/*
 * Driver for MCRV2 PSPI Controllers
 *
 * Copyright (C) 2006 Atmel Corporation
 * Copyright (C) 2016 Alpha Imageing Technology Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <mach/board.h>
#include <asm/gpio.h>
#include <mach/cpu.h>

#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_spi.h>
#include <mach/mmpf_pio.h>
#include <mach/mmp_reg_pad.h>

#include <mach/mmpf_system.h>

#define CS_GPIO_PIN            113//102//17//113  //PLCD11
int is_irq = 0;
int tx_done = 0;
int rx_done = 0;
int int_count = 10000;
struct spi_device *spi_backup;
//int clk_mode_backup;
//struct spi_message	*msg_backup;

/*
 * The core SPI transfer engine just talks to a register bank to set up
 * DMA transfers; transfer queue progress is driven by IRQs.  The clock
 * framework provides the base clock, subdivided for each spi_device.
 */
struct ait_spi
{
    spinlock_t		lock;

    void __iomem		*regs;
    int			irq;
    struct clk		*clk;
    struct platform_device	*pdev;
    struct spi_device	*stay;

    u8			stopping;
    struct list_head	queue;
    struct spi_transfer	*current_transfer;
    unsigned long		current_remaining_bytes;
    struct spi_transfer	*next_transfer;
    unsigned long		next_remaining_bytes;

    void			*buffer;
    dma_addr_t		buffer_dma;

    bool is_master;

    /* Timer for timeouts */
    struct timer_list timer;
    u8 cmd;
};

/* Controller-specific per-slave state */
struct ait_spi_device
{
    unsigned int		npcs_pin;
    u32			csr;
};

#define BUFFER_SIZE		PAGE_SIZE
#define INVALID_DMA_ADDRESS	0xffffffff
#define SPI_RX_BUF_SRAM_PHY_ADDR (0x120000)
//#define DMA_BY_SRAM	1

static void
ait_spi_msg_done(struct spi_master *master, struct ait_spi *as,
                 struct spi_message *msg, int status, int stay);
static void ait_spi_timeout(unsigned long data);


// debug for memory with hw dma
#if 0
static void showMemoryValueAtAddress(int input,int len)
{
    int i;
    for(i=0; i<len; i++)
    {
        u8* address;
        address = (int*) (input+i);
        printk(KERN_ERR"address: %p \t", address);
        printk(KERN_ERR"value: %.2x \n", *address);
        //if(value)
    }
}
#endif
//static unsigned long startxfer,waittxspc;
static inline int ait_spi_xfer_is_last(struct spi_message *msg,
                                       struct spi_transfer *xfer)
{
    pr_debug("%s,%d\r\n",__func__,__LINE__);
    return msg->transfers.prev == &xfer->transfer_list;
}


static void ait_spi_next_xfer_data(struct spi_master *master,
                                   struct spi_transfer *xfer,
                                   dma_addr_t *tx_dma,
                                   dma_addr_t *rx_dma,
                                   u32 *plen)
{
    struct ait_spi	*as = spi_master_get_devdata(master);
    u32			len = *plen;

    pr_debug("%s,%d\r\n",__func__,__LINE__);



    /* use scratch buffer only when rx or tx data is unspecified */
    if (xfer->rx_buf)
    {
        *rx_dma = xfer->rx_dma + xfer->len - *plen;
    }
    else
    {
        *rx_dma = as->buffer_dma;
        if (len+xfer->len > BUFFER_SIZE)
        {
            len = BUFFER_SIZE-xfer->len;
        }
    }
    if (xfer->tx_buf)
    {
        *tx_dma = xfer->tx_dma + xfer->len - *plen;
    }
    else
    {
        *tx_dma = as->buffer_dma;
        if (len+xfer->len > BUFFER_SIZE)
        {
            len = BUFFER_SIZE-xfer->len;
        }
        memset(as->buffer, 0, len);
#ifndef DMA_BY_SRAM
        dma_sync_single_for_device(&as->pdev->dev,
                                   as->buffer_dma, len, DMA_TO_DEVICE);
#endif
    }

    *plen = len;
}


#if 0
int SPI_GET_CLK_DIV[2];

int SPI_GET_CFG[2];

int SPI_GET_WORD_LEN[2];
int SPI_GET_DLY_CYCLE[2];

int SPI_GET_WAIT_CYCLE[2];

int SPI_GET_TXFIFO_SPC[2];
int SPI_GET_RXFIFO_SPC[2];

int SPI_GET_TXFIFO_THD[2];

int SPI_GET_RXFIFO_THD[2];

int SPI_GET_INT_SR[2];


int SPI_GET_INT_EN[2];
int SPI_GET_INT_HOST_SR[2];
int SPI_GET_INT_HOST_EN[2];
int SPI_GET_TXDMA_ADDR[2];
int SPI_GET_RXDMA_ADDR[2];
int SPI_GET_TXDMA_SIZE[2];

int  SPI_GET_RXDMA_SIZE[2];

#endif

/*
 * Submit next transfer for DMA.
 * lock is held, spi irq is blocked
 */
static void ait_spi_next_xfer(struct spi_master *master,
                              struct spi_message *msg)
{
    struct ait_spi	*as = spi_master_get_devdata(master);
    struct spi_transfer	*xfer;
    u32			len, remaining=0;
    u32			int_en=0,spi_cfg,spi_ctl=0;
    dma_addr_t		tx_dma, rx_dma;


//JFYTIMER

    //MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PSPI, MMP_TRUE);

    setup_timer(&as->timer, ait_spi_timeout, (unsigned long)master);

    if (!as->current_transfer)
    {
        xfer = list_entry(msg->transfers.next,
                          struct spi_transfer, transfer_list);
    }
    else if (!as->next_transfer)
    {
        xfer = list_entry(as->current_transfer->transfer_list.next,
                          struct spi_transfer, transfer_list);
        as->next_transfer = xfer;
    }
    else
    {
        xfer = NULL;
    }

    if(xfer==NULL)
    {
        ait_spi_msg_done(master, as, msg,0, 0);
        spin_unlock(&as->lock);
        return;
    }
    else if(xfer->len==0)
    {
        ait_spi_msg_done(master, as, msg, 0, 0);
        spin_unlock(&as->lock);
        return;
    }

    //SPI_SET_CTL(as, SPI_RX_DMA_STOP | SPI_TX_DMA_STOP);
    //SPI_SET_INT_EN(as, 0);
    int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
    int_en = SPI_GET_INT_EN(as) & (~ int_en);
    SPI_SET_INT_EN(as, int_en);
    //spi_cfg = SPI_GET_CFG(as)&(~(TX_NON_XCH_MODE));
    //SPI_SET_CFG(as, spi_cfg);

    //SPI_SET_CTL(as,SPI_TXFIFO_CLR | SPI_RXFIFO_CLR);

    SPI_SET_INT_SR(as, SPI_TXDMA_DONE | SPI_RXDMA_DONE);

    if (xfer)
    {

        len= xfer->len;
        ait_spi_next_xfer_data(master, xfer, &tx_dma, &rx_dma, &len);
        remaining = xfer->len - len;

        SPI_SET_TXDMA_ADDR(as, tx_dma);
        SPI_SET_RXDMA_ADDR(as, rx_dma);
        SPI_SET_TXDMA_SIZE(as, xfer->len);
        SPI_SET_RXDMA_SIZE(as, xfer->len);
        SPI_SET_CTL(as, SPI_RX_DMA_START|SPI_TX_DMA_START);

        //spi_ctl |= SPI_TX_DMA_START;
        //spi_ctl |= SPI_RX_DMA_START;

        //spi_cfg |= SPI_TX_EN;//FIFO clear
        //spi_cfg |= SPI_RX_EN;//FIFO clear
        //spi_cfg |= RX_PHA_ADJ;
        //spi_cfg |= RX_POL_ADJ;
        //spi_cfg |= RX_CLK_ADJ_EN;
        //spi_cfg |= MASTER_RX_USE_PAD_CLK;
        //spi_cfg &= ~RX_CLK_ADJ_EN;

        dev_dbg(&msg->spi->dev,
                "tx_buf rx_buf-->  start xfer %p: len %u tx %p/%08x rx %p/%08x\n",
                xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
                as->buffer, as->buffer_dma);
    }
    else
    {
        xfer = as->next_transfer;
        remaining = as->next_remaining_bytes;
    }

    as->current_transfer = xfer;
    as->current_remaining_bytes = remaining;

    if (remaining > 0)
        len = remaining;
    else if (!ait_spi_xfer_is_last(msg, xfer))
    {

        xfer = list_entry(xfer->transfer_list.next,
                          struct spi_transfer, transfer_list);
        len = xfer->len;
    }
    else
    {
        xfer = NULL;
    }

    as->next_transfer = xfer;

    if (xfer)
    {
        printk("%s:%d should be not here\n",__FILE__, __LINE__);
        len= xfer->len;
        ait_spi_next_xfer_data(master, xfer, &tx_dma, &rx_dma, &len);
        remaining = xfer->len - len;

        SPI_SET_TXDMA_ADDR(as, tx_dma);
        SPI_SET_RXDMA_ADDR(as, rx_dma);
        SPI_SET_TXDMA_SIZE(as, xfer->len);
        SPI_SET_RXDMA_SIZE(as, xfer->len);
        SPI_SET_CTL(as, SPI_RX_DMA_START|SPI_TX_DMA_START);

        //spi_ctl |= SPI_TX_DMA_START;
        //spi_ctl |= SPI_RX_DMA_START;

        //spi_cfg |= SPI_TX_EN;//FIFO clear
        //spi_cfg |= SPI_RX_EN;//FIFO clear
        //spi_cfg |= RX_PHA_ADJ;
        //spi_cfg |= RX_POL_ADJ;
        //spi_cfg |= RX_CLK_ADJ_EN;
        //spi_cfg |= MASTER_RX_USE_PAD_CLK;
        //spi_cfg &= ~RX_CLK_ADJ_EN;

        dev_dbg(&msg->spi->dev,
                "tx_buf rx_buf-->  start xfer %p: len %u tx %p/%08x rx %p/%08x\n",
                xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
                as->buffer, as->buffer_dma);
    }
    is_irq = 0;
    tx_done = 0;
    rx_done = 0;
    //SPI_SET_CFG(as, spi_cfg|TX_XCH_MODE);

    ait_set_gpio_value(CS_GPIO_PIN, 0);//Hai
    mod_timer(&as->timer, jiffies + msecs_to_jiffies(1000));
    int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
    SPI_SET_INT_EN(as, int_en);

    SPI_SET_XCH_CTL(as, XCH_START);

}


static void ait_spi_next_message(struct spi_master *master)
{
    struct ait_spi	*as = spi_master_get_devdata(master);
    struct spi_message	*msg;
    struct spi_device	*spi;

    BUG_ON(as->current_transfer);

    msg = list_entry(as->queue.next, struct spi_message, queue);
    spi = msg->spi;

    dev_dbg(master->dev.parent, "start message %p for %s\n",
            msg, dev_name(&spi->dev));
    ait_spi_next_xfer(master, msg);
}


/*
 * For DMA, tx_buf/tx_dma have the same relationship as rx_buf/rx_dma:
 *  - The buffer is either valid for CPU access, else NULL
 *  - If the buffer is valid, so is its DMA address
 *
 * This driver manages the dma address unless message->is_dma_mapped.
 */
static int
ait_spi_dma_map_xfer(struct ait_spi *as, struct spi_transfer *xfer)
{
    struct device	*dev = &as->pdev->dev;

    xfer->tx_dma = xfer->rx_dma = INVALID_DMA_ADDRESS;
//printk(KERN_ALERT"%s: \n",__func__);
    if (xfer->tx_buf)
    {
        /* tx_buf is a const void* where we need a void * for the dma
         * mapping */
        void *nonconst_tx = (void *)xfer->tx_buf;

        xfer->tx_dma = dma_map_single(dev,
                                      nonconst_tx, xfer->len,
                                      DMA_TO_DEVICE);
        if (dma_mapping_error(dev, xfer->tx_dma))
            return -ENOMEM;

    }
    if (xfer->rx_buf)
    {

        xfer->rx_dma = dma_map_single(dev,
                                      xfer->rx_buf, xfer->len,
                                      DMA_FROM_DEVICE);
        if (dma_mapping_error(dev, xfer->rx_dma))
        {
            if (xfer->tx_buf)
                dma_unmap_single(dev,
                                 xfer->tx_dma, xfer->len,
                                 DMA_TO_DEVICE);

            return -ENOMEM;
        }
    }
    return 0;
}


static void ait_spi_dma_unmap_xfer(struct spi_master *master,
                                   struct spi_transfer *xfer)
{
//printk(KERN_ALERT"%s: \n",__func__);
    if (xfer->tx_dma != INVALID_DMA_ADDRESS)
        dma_unmap_single(master->dev.parent, xfer->tx_dma,
                         xfer->len, DMA_TO_DEVICE);
    if (xfer->rx_dma != INVALID_DMA_ADDRESS)
        dma_unmap_single(master->dev.parent, xfer->rx_dma,
                         xfer->len, DMA_FROM_DEVICE);
}

static void
ait_spi_msg_done(struct spi_master *master, struct ait_spi *as,
                 struct spi_message *msg, int status, int stay)
{

    u32 int_en = 0;
    ait_set_gpio_value(CS_GPIO_PIN, 1);//Hai
    //printk(KERN_ALERT"%s: \n",__func__);
    list_del(&msg->queue);
    msg->status = status;

    dev_dbg(master->dev.parent,
            "xfer complete: %u bytes transferred\n",
            msg->actual_length);
    spin_unlock(&as->lock);
    msg->complete(msg->context);
    spin_lock(&as->lock);
    del_timer(&as->timer);
    as->current_transfer = NULL;
    as->next_transfer = NULL;
    del_timer_sync(&as->timer);
    SPI_SET_CTL(as, SPI_RX_DMA_STOP | SPI_TX_DMA_STOP);
    /* continue if needed */
    if (list_empty(&as->queue) || as->stopping)
    {
        int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
        int_en = SPI_GET_INT_EN(as) & (~ int_en);
        SPI_SET_INT_EN(as, int_en);
    }//spi_writel(as, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));
    else
    {
        printk("%s:%d not here\n", __FILE__, __LINE__);
        ait_spi_next_message(master);
    }
}


static void ait_spi_timeout(unsigned long data)
{
    struct spi_message	*msg;

    struct spi_master *master = (struct spi_master *)data;

    struct ait_spi *as = spi_master_get_devdata(master);
    struct spi_transfer	*xfer;
    struct spi_device	*spi;
    AITPS_PAD   pPAD  = AITC_BASE_PAD;
    int clk_mode;
//printk(KERN_ALERT"%s: \n",__func__);
    pr_warn("ait_spi_timeout\n");
    pr_warn("is_irq = %d, tx_done = %d, rx_done = %d\n", is_irq, tx_done, rx_done);
#ifdef DEBUG
    if(as)
    {
        int i;
        xfer = as->current_transfer;

        if(xfer)
        {
            pr_warn("xfer->len = %x\n",(unsigned int)xfer->len);

            for(i=0; i<min(xfer->len,5); ++i)
            {
                pr_warn(" 0x%x ",(unsigned int)((char*)xfer->tx_buf)[i]);
            }
            pr_warn(" \n");
        }

        if(xfer->rx_buf)
        {
            pr_warn("rx_buf = %x\n",(unsigned int)xfer->rx_buf);
            for(i=0; i<min(xfer->len,5); ++i)
            {
                pr_warn(" 0x%x ",(unsigned int)((char*)xfer->rx_buf)[i]);
            }
            pr_warn(" \n");
        }
    }
#endif
    pr_warn("SPI_GET_CLK_DIV = 0x%04x\r\n",SPI_GET_CLK_DIV(as));

    pr_warn("SPI_GET_CFG = 0x%08x\r\n",SPI_GET_CFG(as));

    pr_warn("SPI_GET_WORD_LEN = 0x%04x\r\n",SPI_GET_WORD_LEN(as));
    pr_warn("SPI_GET_DLY_CYCLE = 0x%04x\r\n",SPI_GET_DLY_CYCLE(as));

    pr_warn("SPI_GET_WAIT_CYCLE = 0x%04x\r\n",SPI_GET_WAIT_CYCLE(as));

    pr_warn("SPI_GET_TXFIFO_SPC = 0x%04x\r\n",SPI_GET_TXFIFO_SPC(as));
    pr_warn("SPI_GET_RXFIFO_SPC = 0x%04x\r\n",SPI_GET_RXFIFO_SPC(as));

    pr_warn("SPI_GET_TXFIFO_THD = 0x%04x\r\n",SPI_GET_TXFIFO_THD(as));

    pr_warn("SPI_GET_RXFIFO_THD = 0x%04x\r\n",SPI_GET_RXFIFO_THD(as));

    pr_warn("SPI_GET_INT_SR = 0x%04x\r\n",SPI_GET_INT_SR(as));


    pr_warn("SPI_GET_INT_EN = 0x%04x\r\n",SPI_GET_INT_EN(as));
    pr_warn("SPI_GET_INT_HOST_SR = 0x%04x\r\n",SPI_GET_INT_HOST_SR(as));
    pr_warn("SPI_GET_INT_HOST_EN= 0x%04x\r\n",SPI_GET_INT_HOST_EN(as));
    pr_warn("SPI_GET_TXDMA_ADDR = 0x%04x\r\n",SPI_GET_TXDMA_ADDR(as));
    pr_warn("SPI_GET_RXDMA_ADDR = 0x%04x\r\n",SPI_GET_RXDMA_ADDR(as));
    pr_warn("SPI_GET_TXDMA_SIZE= 0x%08x\r\n",SPI_GET_TXDMA_SIZE(as));

    pr_warn("SPI_GET_RXDMA_SIZE = 0x%04x\r\n",SPI_GET_RXDMA_SIZE(as));

#if 0
    pr_warn("Before====\r\n");

    pr_warn("SPI_GET_CLK_DIV = 0x%04x\r\n",SPI_GET_CLK_DIV[0]);

    pr_warn("SPI_GET_CFG = 0x%04x\r\n",SPI_GET_CFG[0]);

    pr_warn("SPI_GET_WORD_LEN = 0x%04x\r\n",SPI_GET_WORD_LEN[0]);
    pr_warn("SPI_GET_DLY_CYCLE = 0x%04x\r\n",SPI_GET_DLY_CYCLE[0]);

    pr_warn("SPI_GET_WAIT_CYCLE = 0x%04x\r\n",SPI_GET_WAIT_CYCLE[0]);

    pr_warn("SPI_GET_TXFIFO_SPC = 0x%04x\r\n",SPI_GET_TXFIFO_SPC[0]);
    pr_warn("SPI_GET_RXFIFO_SPC = 0x%04x\r\n",SPI_GET_RXFIFO_SPC[0]);

    pr_warn("SPI_GET_TXFIFO_THD = 0x%04x\r\n",SPI_GET_TXFIFO_THD[0]);

    pr_warn("SPI_GET_RXFIFO_THD = 0x%04x\r\n",SPI_GET_RXFIFO_THD[0]);

    pr_warn("SPI_GET_INT_SR = 0x%04x\r\n",SPI_GET_INT_SR[0]);


    pr_warn("SPI_GET_INT_EN = 0x%04x\r\n",SPI_GET_INT_EN[0]);
    pr_warn("SPI_GET_INT_HOST_SR = 0x%04x\r\n",SPI_GET_INT_HOST_SR[0]);
    pr_warn("SPI_GET_INT_HOST_EN= 0x%04x\r\n",SPI_GET_INT_HOST_EN[0]);
    pr_warn("SPI_GET_TXDMA_ADDR = 0x%04x\r\n",SPI_GET_TXDMA_ADDR[0]);
    pr_warn("SPI_GET_RXDMA_ADDR = 0x%04x\r\n",SPI_GET_RXDMA_ADDR[0]);
    pr_warn("SPI_GET_TXDMA_SIZE= 0x%08x\r\n",SPI_GET_TXDMA_SIZE[0]);

    pr_warn("SPI_GET_RXDMA_SIZE = 0x%04x\r\n",SPI_GET_RXDMA_SIZE[0]);

    pr_warn("After====\r\n");

    pr_warn("SPI_GET_CLK_DIV = 0x%04x\r\n",SPI_GET_CLK_DIV[1]);
    pr_warn("SPI_GET_CFG = 0x%04x\r\n",SPI_GET_CFG[1]);

    pr_warn("SPI_GET_WORD_LEN = 0x%04x\r\n",SPI_GET_WORD_LEN[1]);
    pr_warn("SPI_GET_DLY_CYCLE = 0x%04x\r\n",SPI_GET_DLY_CYCLE[1]);

    pr_warn("SPI_GET_WAIT_CYCLE = 0x%04x\r\n",SPI_GET_WAIT_CYCLE[1]);

    pr_warn("SPI_GET_TXFIFO_SPC = 0x%04x\r\n",SPI_GET_TXFIFO_SPC[1]);
    pr_warn("SPI_GET_RXFIFO_SPC = 0x%04x\r\n",SPI_GET_RXFIFO_SPC[1]);

    pr_warn("SPI_GET_TXFIFO_THD = 0x%04x\r\n",SPI_GET_TXFIFO_THD[1]);

    pr_warn("SPI_GET_RXFIFO_THD = 0x%04x\r\n",SPI_GET_RXFIFO_THD[1]);

    pr_warn("SPI_GET_INT_SR = 0x%04x\r\n",SPI_GET_INT_SR[1]);


    pr_warn("SPI_GET_INT_EN = 0x%04x\r\n",SPI_GET_INT_EN[1]);
    pr_warn("SPI_GET_INT_HOST_SR = 0x%04x\r\n",SPI_GET_INT_HOST_SR[1]);
    pr_warn("SPI_GET_INT_HOST_EN= 0x%04x\r\n",SPI_GET_INT_HOST_EN[1]);
    pr_warn("SPI_GET_TXDMA_ADDR = 0x%04x\r\n",SPI_GET_TXDMA_ADDR[1]);
    pr_warn("SPI_GET_RXDMA_ADDR = 0x%04x\r\n",SPI_GET_RXDMA_ADDR[1]);
    pr_warn("SPI_GET_TXDMA_SIZE= 0x%08x\r\n",SPI_GET_TXDMA_SIZE[1]);

    pr_warn("SPI_GET_RXDMA_SIZE = 0x%04x\r\n",SPI_GET_RXDMA_SIZE[1]);
#endif

    //SPI_SET_CTL(as,SPI_RXFIFO_CLR|SPI_TXFIFO_CLR);
    SPI_SET_INT_SR(as,0xffffff);
#if 1
    u32 int_en = 0;
    int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
    int_en = SPI_GET_INT_EN(as) & (~ int_en);
    SPI_SET_INT_EN(as, int_en);
#endif

    msg = list_entry(as->queue.next, struct spi_message, queue);

    spi = msg->spi;


    if (!msg->is_dma_mapped)
    {
        xfer = list_entry(msg->transfers.next,
                          struct spi_transfer, transfer_list);
        ait_spi_dma_unmap_xfer(master, xfer);

        xfer = list_entry(xfer->transfer_list.next,
                          struct spi_transfer, transfer_list);
        ait_spi_dma_unmap_xfer(master, xfer);
    }

    ait_spi_msg_done(master, as, msg, -ETIMEDOUT, 0);
#if 1
    printk(KERN_ALERT"********RESET SPI HARDWARE start**********!!!");
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PSPI, MMP_FALSE);
    SPI_SET_CFG(as,MASTER_RX_USE_PAD_CLK | \
                SPI_MASTER_MODE | \
                SS_POLAR_LOW | RX_CLK_ADJ_EN | \
                RX_PHA_ADJ | RX_POL_ADJ | \
                SPI_TX_EN | SPI_RX_EN);
    //SPI_SET_CFG(as, MASTER_RX_USE_PAD_CLK|SPI_MASTER_MODE|SS_POLAR_LOW|SPI_TX_EN|SPI_RX_EN);

    SPI_SET_CTL(as,SPI_TXFIFO_CLR | SPI_RXFIFO_CLR);
    //SPI_SET_CTL(as, SPI_RX_DMA_STOP|SPI_TX_DMA_STOP);

    //SPI_SET_CTL(as, SPI_RX_DMA_START|SPI_TX_DMA_START);

    SPI_SET_CTL(as, SPI_RX_DMA_STOP|SPI_TX_DMA_STOP);

    SPI_SET_INT_EN(as, 0);

    SPI_SET_WORD_LEN(as,8);
#if 1
    as = spi_master_get_devdata(spi->master);

    clk_mode = SPI_GET_CFG(as)&~(SCLK_POL_HIGH|SCLK_PHA_HIGH);
//	printk(KERN_ALERT"%s, spi->mode:%02X \n",__func__,spi->mode);

    if (spi->mode == 0x0)
    {
        SPI_SET_CFG(as,clk_mode);
    }
    if (spi->mode == 0x1)
    {
        SPI_SET_CFG(as,clk_mode|SCLK_PHA_HIGH);
    }
    if (spi->mode == 0x2)
    {
        SPI_SET_CFG(as,clk_mode|SCLK_POL_HIGH);
    }

    if (spi->mode == 0x3)
    {
        SPI_SET_CFG(as,clk_mode|SCLK_PHA_HIGH|SCLK_POL_HIGH);
    }

    //int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
    //SPI_SET_INT_EN(as, int_en);
#endif
    //int_en = SPI_RXDMA_DONE| SPI_TXDMA_DONE;
    //int_en = SPI_GET_INT_EN(as) & (~ int_en);
    //SPI_SET_INT_EN(as, int_en);

    //SPI_SET_INT_SR(as, 0xffffffff);
    SPI_SET_DLY_CYCLE(as, 0);
    SPI_SET_WAIT_CYCLE(as, 0);

    printk(KERN_ALERT"********RESET SPI HARDWARE done***********!!!");
#endif
    spin_unlock(&as->lock);

    return;
}



static irqreturn_t
ait_spi_interrupt(int irq, void *dev_id)
{
    struct spi_master	*master = dev_id;
    struct ait_spi	*as = spi_master_get_devdata(master);
    struct spi_message	*msg;
    struct spi_transfer	*xfer;
    u32			status, pending, imr;
    int			ret = IRQ_NONE;
    BUG_ON(!as);

    spin_lock(&as->lock);
    xfer = as->current_transfer;
    is_irq ++;
    msg = list_entry(as->queue.next, struct spi_message, queue);

    //#pragma pack(4)
    imr = SPI_GET_INT_EN(as);//0x6D24 PSPI CPU Interrupt Enable 0
    // interrupt enable:
    // bit     7     6      5   4           3     2      1   0
    // TXFIFO  Full, empty, GE, LE;; RXFIFO Full, empty, GE, LE
    //#pragma pack(4)
    status = SPI_GET_INT_SR(as);//0x6D20 PSPI CPU Status Register 0
    // bit    7     6      5   4            3     2      1   0
    // TXFIFO Full, empty, GE, LE ;; TXFIFO Full, empty, GE, LE

    pending = status & imr;

    if(xfer==0)
    {
        pr_err("mcrv2_pspi_interrupt: as->current_transfer is NULL pointer. as = %x\r\n",(unsigned int)as);
        pr_warn("SPI_GET_INT_SR = 0x%04x\r\n",SPI_GET_INT_SR(as));
        pr_warn("SPI_GET_INT_EN = 0x%02x\r\n",SPI_GET_INT_EN(as));
        pr_warn("SPI_GET_TXDMA_SIZE = 0x%02x\r\n",SPI_GET_TXDMA_SIZE(as));
        pr_warn("SPI_GET_RXDMA_SIZE = 0x%02x\r\n",SPI_GET_RXDMA_SIZE(as));
        pr_warn("SPI_GET_CFG = 0x%02x\r\n",SPI_GET_CFG(as));

        imr = SPI_GET_INT_EN(as);
        status = SPI_GET_INT_SR(as)&imr;
        pending = status & imr;
        pr_warn("EN: 0x%08x\r\n",imr);
        pr_warn("SR 0x%08x\r\n",status);
        pr_warn("pending 0x%08x\r\n",pending);

        SPI_SET_INT_SR(as, pending);
        //ait_spi_msg_done(master, as, msg, -EIO,0);
        spin_unlock(&as->lock);

        return IRQ_HANDLED;
    }

    if(pending & SPI_TXFIFO_FULL)
    {
        pr_warn("SPI_TXFIFO_FULL %d\n", xfer->len);
        SPI_SET_INT_SR(as,SPI_TXFIFO_FULL);
        spin_unlock(&as->lock);
        return IRQ_HANDLED;
    }

    if(pending & SPI_TXFIFO_GE)
    {
        pr_warn("SPI_TXFIFO_GE\n");
        SPI_SET_INT_SR(as,SPI_TXFIFO_GE);
        spin_unlock(&as->lock);
        return IRQ_HANDLED;
    }


    if(pending &(SPI_RXFIFO_FULL|SPI_RXFIFO_OF))
    {

        if(pending & SPI_RXFIFO_FULL)
        {
            pr_warn("SPI_RXFIFO_FULL\n");
            SPI_SET_INT_SR(as,SPI_RXFIFO_FULL);
        }
        else//(pending & SPI_RXFIFO_OF)
        {
            pr_warn("SPI_RXFIFO_OF\n");
        }

        SPI_SET_CTL(as,SPI_RXFIFO_CLR|SPI_TXFIFO_CLR);
        SPI_SET_INT_SR(as,0xffffff);

        ait_spi_msg_done(master, as, msg, -EIO,0);

        spin_unlock(&as->lock);
        return IRQ_HANDLED;
    }

    if(!(pending &(SPI_TXDMA_DONE|SPI_RXDMA_DONE)))
    {
        printk("WRONG status=%08X\n", status);
        spin_unlock(&as->lock);
        return IRQ_HANDLED;
    }

    if(pending & SPI_TXDMA_DONE)
    {
        tx_done = 1;
        SPI_SET_INT_SR(as,SPI_TXDMA_DONE);
        if( (!(pending & SPI_RXDMA_DONE)) &&
                xfer->rx_buf)
        {

            //printk(KERN_ALERT"%s:Wait RX\n",__func__);
            spin_unlock(&as->lock);
            //printk("Wait RX\n");
            return IRQ_HANDLED;
        }
    }

    if (pending & (SPI_RXDMA_DONE))
    {
        SPI_SET_INT_SR(as,SPI_RXDMA_DONE);
        rx_done = 1;
    }

    ret = IRQ_HANDLED;

    if (as->current_remaining_bytes == 0)
    {

        msg->actual_length += xfer->len;

        if (!msg->is_dma_mapped)
            ait_spi_dma_unmap_xfer(master, xfer);

        /* REVISIT: udelay in irq is unfriendly */
        if (xfer->delay_usecs)
            udelay(xfer->delay_usecs);

        if (ait_spi_xfer_is_last(msg, xfer))
        {
            /* report completed message */
            //SPI_SET_INT_SR(as,0xffffff);
            //SPI_SET_INT_HOST_SR(as,0xffffff);
            ait_spi_msg_done(master, as, msg, 0,
                             xfer->cs_change);
            spin_unlock(&as->lock);
        }
        else
        {

            printk("no last\n");
            /*
             * Not done yet. Submit the next transfer.
             *
             * FIXME handle protocol options for xfer
             */
            ait_spi_next_xfer(master, msg);
        }
    }
    else
    {
        printk("remain=0\n");
        /*
         * Keep going, we still have data to send in
         * the current transfer.
         */
        ait_spi_next_xfer(master, msg);
    }
    spin_unlock(&as->lock);
    return ret;
}


static int ait_spi_setup(struct spi_device *spi)
{
    struct ait_spi	*as;
    unsigned int		bits = spi->bits_per_word;
    int			clk_mode;
    AITPS_PAD   pPAD  = AITC_BASE_PAD;
    as = spi_master_get_devdata(spi->master);
    spi_backup=spi;
    if (as->stopping)
        return -ESHUTDOWN;

    if (spi->chip_select > spi->master->num_chipselect)
    {
        dev_dbg(&spi->dev,"setup: invalid chipselect %u (%u defined)\n",spi->chip_select, spi->master->num_chipselect);

        return -EINVAL;
    }


    if (bits < 8 || bits > 16)
    {
        dev_dbg(&spi->dev,
                "setup: invalid bits_per_word %u (8 to 16)\n",
                bits);
        return -EINVAL;
    }
    SPI_SET_WORD_LEN(as,8);

    /*
    	if (spi->max_speed_hz) {
    		u8  divisor=1;
    		u32	bus_hz = clk_get_rate(as->clk);

    		if(spi->max_speed_hz<=(bus_hz/(2*(divisor+1))))
    		{
    			for(divisor=1;divisor<=0x80;divisor++)
    			{
    					pr_info("bus_hz = %d    target clock=%d divisor = %d \r\n",bus_hz,(bus_hz/(2*(divisor+1))),divisor);
    					if((bus_hz/(2*(divisor+1)))<=spi->max_speed_hz)
    					break;
    			}
    		}

    		spi->max_speed_hz = bus_hz/(2*(divisor+1));

    		SPI_SET_CLK_DIV(as,divisor);

    		pr_info("Wanted speed: %d Hz , provided %d Hz\n",spi->max_speed_hz,bus_hz/(2*(divisor+1)));
    	}else
    		SPI_SET_CLK_DIV(as,1);

    */
    clk_mode = SPI_GET_CFG(as)&~(SCLK_POL_HIGH|SCLK_PHA_HIGH);

    if (spi->mode == 0x0)
    {
        SPI_SET_CFG(as,clk_mode);
        pPAD->PAD_IO_CFG_PLCD[0] = 0x63;//for PULL low
    }
    if (spi->mode == 0x1)
    {
        pPAD->PAD_IO_CFG_PLCD[0] = 0x63;//for PULL low
        SPI_SET_CFG(as,clk_mode|SCLK_PHA_HIGH);
    }
    if (spi->mode == 0x2)
    {
        pPAD->PAD_IO_CFG_PLCD[0] = 0x65;//for PULL High
        SPI_SET_CFG(as,clk_mode|SCLK_POL_HIGH);
    }
    if (spi->mode == 0x3)
    {
        pPAD->PAD_IO_CFG_PLCD[0] = 0x65;//for PULL High
        SPI_SET_CFG(as,clk_mode|SCLK_PHA_HIGH|SCLK_POL_HIGH);
    }
    pr_info("%s:clk_mode = %d\r\n",dev_name(&spi->dev),spi->mode&(SPI_CPOL|SPI_CPHA));

    return 0;
}


static int ait_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct ait_spi	*as;
    struct spi_transfer	*xfer;
    unsigned long		flags;

    struct spi_master	*master;

    as = spi_master_get_devdata(spi->master);

    if (unlikely(list_empty(&msg->transfers)))
        return -EINVAL;

    if (as->stopping)
        return -ESHUTDOWN;

    list_for_each_entry(xfer, &msg->transfers, transfer_list)
    {
        if (!(xfer->tx_buf || xfer->rx_buf) && xfer->len)
        {
            dev_err(&spi->dev, "missing rx or tx buf\n");
            return -EINVAL;
        }

        if(!xfer->bits_per_word)
            xfer->bits_per_word = spi->bits_per_word;


        if (xfer->tx_buf&&xfer->bits_per_word)
        {
            SPI_SET_WORD_LEN(as,xfer->bits_per_word);
        }

        if (xfer->speed_hz)
        {
            u32 target_speed;
            u8  divisor;
            u32	bus_hz = clk_get_rate(as->clk);

            if(xfer->speed_hz>spi->max_speed_hz)
            {
                divisor = 10;
                target_speed = spi->max_speed_hz;
            }
            else
            {
                for(divisor=1; divisor<=0xFF; divisor++)
                    if((bus_hz/(2*(divisor+1)))<=xfer->speed_hz)
                        break;
            }

            SPI_SET_CLK_DIV(as,divisor*2);

            //pr_info( "%s:Wanted speed: %d Hz , provided %d Hz   divisor*2 = %d\n",dev_name(&spi->dev),xfer->speed_hz,bus_hz/(2*(divisor+1)),divisor*2);
            //return -ENOPROTOOPT;
        }

        /*
         * DMA map early, for performance (empties dcache ASAP) and
         * better fault reporting.  This is a DMA-only driver.
         *
         * NOTE that if dma_unmap_single() ever starts to do work on
         * platforms supported by this driver, we would need to clean
         * up mappings for previously-mapped transfers.
         */

        if (!msg->is_dma_mapped)
        {
            if (ait_spi_dma_map_xfer(as, xfer) < 0)
                return -ENOMEM;
        }
    }

#if 0//def VERBOSE
    list_for_each_entry(xfer, &msg->transfers, transfer_list)
    {
        pr_info(	"  xfer %p: len %u tx %p/%08x rx %p/%08x\n",
                    xfer, xfer->len,
                    xfer->tx_buf, xfer->tx_dma,
                    xfer->rx_buf, xfer->rx_dma);
    }
#endif

    msg->status = -EINPROGRESS;
    msg->actual_length = 0;


    spin_lock_irqsave(&as->lock, flags);
    list_add_tail(&msg->queue, &as->queue);
    if (!as->current_transfer)
    {

        ait_spi_next_message(spi->master);
    }

    spin_unlock_irqrestore(&as->lock, flags);

    return 0;
}


static void ait_spi_cleanup(struct spi_device *spi)
{
#if 0
    struct ait_spi	*as = spi_master_get_devdata(spi->master);
    struct ait_spi_device	*asd = spi->controller_state;
    unsigned		gpio = (unsigned) spi->controller_data;
    unsigned long		flags;

    if (!asd)
        return;

    spin_lock_irqsave(&as->lock, flags);
    if (as->stay == spi)
    {
        as->stay = NULL;
        cs_deactivate(as, spi);
    }
    spin_unlock_irqrestore(&as->lock, flags);

    spi->controller_state = NULL;
    gpio_free(gpio);
    kfree(asd);
#endif
}


static int __devinit ait_spi_probe(struct platform_device *pdev)
{
    struct resource		*regs;
    int			irq;
    struct clk		*clk;
    int			ret;
    struct spi_master	*master;
    struct ait_spi	*as;

    printk(KERN_INFO "ait_spi_probe.\n");

    ait_set_gpio_output(CS_GPIO_PIN, 1);
    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs)
        return -ENXIO;

    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
        return irq;

    clk = clk_get(&pdev->dev, "pspi_ctl_clk");
    if (IS_ERR(clk))
        return PTR_ERR(clk);



    ret = -ENOMEM;
    master = spi_alloc_master(&pdev->dev, sizeof(*as));
    if (!master)
        goto alloc_master_fail;
    else
    {
    }
    /* the spi->mode bits understood by this driver: */
    master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

    master->bus_num = pdev->id;
    master->num_chipselect = 1;
    master->setup = ait_spi_setup;
    master->transfer = ait_spi_transfer;
    master->cleanup = ait_spi_cleanup;
    platform_set_drvdata(pdev, master);

    as = spi_master_get_devdata(master);

    /*
     * Scratch buffer is used for throwaway rx and tx data.
     * It's coherent to minimize dcache pollution.
     */
#ifdef DMA_BY_SRAM
    sram_res = request_mem_region(SPI_RX_BUF_SRAM_PHY_ADDR, BUFFER_SIZE, "SRAM Region:PSPI RX");

    if (sram_res==NULL)
    {
        printk(KERN_ALERT"Request SRAM mem region failed!\n");
        goto alloc_master_fail;
    }
    else
    {
        printk(KERN_ALERT"Request SRAM mem region success!\n");
        as->buffer_dma = SPI_RX_BUF_SRAM_PHY_ADDR;

        as->buffer = (void*)AIT_RAM_P2V(as->buffer_dma );
        // printk(KERN_ALERT"as->buffer virtual address : %x \n",as->buffer);
    }
#else
    as->buffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
                                    &as->buffer_dma, GFP_KERNEL);
#endif
    if (!as->buffer)
        goto alloc_master_fail;

    spin_lock_init(&as->lock);
    INIT_LIST_HEAD(&as->queue);
    as->pdev = pdev;
    as->regs = ioremap(regs->start, resource_size(regs));
    if (!as->regs)
        goto ioremap_fail;
    as->irq = irq;



    as->clk = clk;

    ret = request_irq(irq, ait_spi_interrupt, IRQF_SHARED | IRQF_DISABLED,dev_name(&pdev->dev), master);
    if (ret)
        goto req_irq_fail;

    /* Initialize the hardware */
    clk_enable(clk);

    //SPI_SET_CFG(as,MASTER_RX_USE_INCLK|/*MASTER_RX_PAD_CLK|*/SPI_MASTER_MODE|SS_POLAR_LOW|RX_CLK_ADJ_EN|RX_PHA_ADJ|RX_POL_ADJ);
    SPI_SET_CFG(as,MASTER_RX_USE_PAD_CLK | \
                SPI_MASTER_MODE | \
                SS_POLAR_LOW | RX_CLK_ADJ_EN | \
                RX_PHA_ADJ | RX_POL_ADJ | \
                SPI_TX_EN | SPI_RX_EN);
    //SPI_SET_CFG(as, MASTER_RX_USE_PAD_CLK|SPI_MASTER_MODE|SS_POLAR_LOW|SPI_TX_EN|SPI_RX_EN);

    SPI_SET_CTL(as,SPI_TXFIFO_CLR | SPI_RXFIFO_CLR);
    SPI_SET_CTL(as, SPI_RX_DMA_STOP|SPI_TX_DMA_STOP);

    SPI_SET_INT_EN(as, 0);
    SPI_SET_INT_SR(as, 0xffffffff);
    SPI_SET_DLY_CYCLE(as, 0);
    SPI_SET_WAIT_CYCLE(as, 0);

    dev_info(&pdev->dev, "AIT SPI Controller at 0x%08lx (irq %d)\n",
             (unsigned long)regs->start, irq);

    //setup_timer(&as->timer, ait_spi_timeout, (unsigned long)master);


    ret = spi_register_master(master);
    if (ret)
        goto reg_spi_master_fail;

    return 0;

reg_spi_master_fail:
    clk_disable(clk);
    free_irq(irq, master);
req_irq_fail:
    iounmap(as->regs);
ioremap_fail:
#ifndef DMA_BY_SRAM

    dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
                      as->buffer_dma);
#endif
alloc_master_fail:
    clk_put(clk);
    spi_master_put(master);
    return ret;
}


static int __devexit ait_spi_remove(struct platform_device *pdev)
{
    struct spi_master	*master = platform_get_drvdata(pdev);
    struct ait_spi	*as = spi_master_get_devdata(master);
    struct spi_message	*msg;

    /* reset the hardware and block queue progress */
    spin_lock_irq(&as->lock);
    as->stopping = 1;

    SPI_SET_INT_EN(as, 0);

    /* Timer for timeouts */
    del_timer_sync(&as->timer);

    spin_unlock_irq(&as->lock);

    /* Terminate remaining queued transfers */
    list_for_each_entry(msg, &as->queue, queue)
    {
        /* REVISIT unmapping the dma is a NOP on ARM and AVR32
         * but we shouldn't depend on that...
         */
        msg->status = -ESHUTDOWN;
        msg->complete(msg->context);
    }


#ifndef DMA_BY_SRAM
    dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
                      as->buffer_dma);
#else
    release_mem_region(SPI_RX_BUF_SRAM_PHY_ADDR, BUFFER_SIZE);
#endif
    clk_disable(as->clk);
    clk_put(as->clk);
    free_irq(as->irq, master);
    iounmap(as->regs);

    spi_unregister_master(master);

    return 0;
}


#ifdef	CONFIG_PM

static int ait_spi_suspend(struct platform_device *pdev, pm_message_t mesg)
{

    struct spi_master	*master = platform_get_drvdata(pdev);
    struct ait_spi	*as = spi_master_get_devdata(master);
    SPI_SET_INT_EN(as, 0);

    clk_disable(as->clk);
    return 0;
}

static int ait_spi_resume(struct platform_device *pdev)
{

    struct spi_master	*master = platform_get_drvdata(pdev);
    struct ait_spi	*as = spi_master_get_devdata(master);

    clk_enable(as->clk);
    return 0;
}

#else
#define	ait_spi_suspend	NULL
#define	ait_spi_resume	NULL
#endif


static struct platform_driver ait_spi_driver =
{
    .driver		= {
        .name	= "ait_pspi",
        .owner	= THIS_MODULE,
    },
    .probe		= ait_spi_probe,
    .remove		= __exit_p(ait_spi_remove),
    .shutdown	= NULL,
    .suspend	= ait_spi_suspend,
    .resume		= ait_spi_resume,
};
module_platform_driver(ait_spi_driver);

MODULE_DESCRIPTION("AIT MCRV2 PSPI Controller driver");
MODULE_AUTHOR("AIT");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ait_mcrv2_pspi");
