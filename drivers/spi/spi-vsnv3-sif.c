/*
 * Driver for Vision V3 SIF Controllers
 *
 * Copyright (C) 2006 Atmel Corporation
 * Copyright (C) 2013 Alpha Imageing Technology Corporation
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
#include <mach/mmp_reg_sf.h>
#include <mach/mmp_reg_mci.h>

#define VMEM_FIX_EN             (1)

#define SIF_CMD_READ_STATUS				0x05
#define SIF_CMD_WRITE_STATUS			0x01
#define SIF_CMD_WRITE_ENABLE			0x06
#define SIF_CMD_WRITE_DISABLE			0x04

#define SIF_CMD_PAGE_PROGRAM			0x02
#define SIF_CMD_PAGE_QUAD_PROGRAM		0x32
#define SIF_CMD_READ_DATA				0x03
#define SIF_CMD_FAST_READ_DUAL_IO		0xBB
#define SIF_CMD_FAST_READ_DUAL_OUTPUT	0x3B
#define SIF_CMD_FAST_READ_QUAD_OUTPUT	0x6B
#define SIF_CMD_FAST_READ_QUAD_IO		0xEB
#define SIF_CMD_READ_QUAD_IO				0xE7
#define SIF_CMD_WRITE_EAR				0xC5
#define SIF_CMD_READ_EAR				0xC8

#define SIF_CMD_EXIT_OTP_MODE			0x04


#define SIF_CMD_FAST_READ_DATA			0x0B

#define SIF_CMD_ENTER_OTP_MODE			0x3A

#define SIF_CMD_SECTOR_ERASE			0x20
#define SIF_CMD_BLOCK_ERASE				0x52
#define SIF_CMD_CHIP_ERASE				0x60


#define SIF_CMD_EBSY					0x70
#define SIF_CMD_DBSY					0x80
#define SIF_CMD_READ_DEVICE_ID			0x9F
#define SIF_CMD_RELEASE_DEEP_POWER_DOWN 0xAB
#define SIF_CMD_ADDR_AUTO_INC_WRITE		0xAD
#define SIF_CMD_DEEP_POWER_DOWN			0xB9

#define REMOVE_DUMMY_RX_DATA
/*
 * The core SPI transfer engine just talks to a register bank to set up
 * DMA transfers; transfer queue progress is driven by IRQs.  The clock
 * framework provides the base clock, subdivided for each spi_device.
 */
struct vsnv3_spi {
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
#if VMEM_FIX_EN
  void        *vmem_buffer ;
#endif
	bool is_master;

		/* Timer for timeouts */
	struct timer_list timer;
	u8 cmd;
	u8 cmd_len;	
#ifdef REMOVE_DUMMY_RX_DATA	
	u8 dumy_rx_data;	
#endif		
};

static void vsnv3_sif_timeout(unsigned long data);
static void vsnv3_sif_msg_done(struct spi_master *master, struct vsnv3_spi *as,struct spi_message *msg, int status, int stay);
static void vsnv3_sif_dma_unmap_xfer(struct spi_master *master, struct spi_transfer *xfer);

#define BUFFER_SIZE		PAGE_SIZE*16 //temp. It need to get by spiflash driver.
#define INVALID_DMA_ADDRESS	0xffffffff

static void vsnv3_sif_cmd_abort(unsigned long data)
{
	vsnv3_sif_timeout(data);
}

static inline int vsnv3_sif_xfer_is_last(struct spi_message *msg,
					struct spi_transfer *xfer)
{
	pr_debug("%s,%d\r\n",__func__,__LINE__);
	return msg->transfers.prev == &xfer->transfer_list;
}

static inline int vsnv3_sif_xfer_can_be_chained(struct spi_transfer *xfer)
{
	pr_debug("%s,%d\r\n",__func__,__LINE__);
	return xfer->delay_usecs == 0 && !xfer->cs_change;
}

static void vsnv3_sif_next_xfer_data(struct spi_master *master,
				struct spi_transfer *xfer,
				dma_addr_t *tx_dma,
				dma_addr_t *rx_dma,
				u32 *plen)
{
	struct vsnv3_spi *as = spi_master_get_devdata(master);
	u32			len = *plen;

	/* use scratch buffer only when rx or tx data is unspecified */
	if (xfer->rx_buf)
		*rx_dma = xfer->rx_dma + xfer->len - *plen;
	else {
		*rx_dma = as->buffer_dma;
		if (len > BUFFER_SIZE)
			len = BUFFER_SIZE;
	}
	if (xfer->tx_buf)
		*tx_dma = xfer->tx_dma + xfer->len - *plen;
	else {
		*tx_dma = as->buffer_dma;
		if (len > BUFFER_SIZE)
			len = BUFFER_SIZE;
		memset(as->buffer, 0, len);
		dma_sync_single_for_device(&as->pdev->dev,
				as->buffer_dma, len, DMA_TO_DEVICE);
	}

	*plen = len;
}


/*
 * Submit next transfer for DMA.
 * lock is held, spi irq is blocked
 */
static void vsnv3_sif_next_xfer(struct spi_master *master,
				struct spi_message *msg)
{
	struct vsnv3_spi *as = spi_master_get_devdata(master);
	struct spi_transfer	*xfer;
	u32			len, remaining;
	u32			int_en,spi_cfg;
	dma_addr_t		tx_dma, rx_dma;
	u8 			spi_cmd=0;	
	u8 			spi_ctl=0;
	u8 			spi_ctl_data_width=0;
	u32 			spi_tx_addr ;	
	int i;
	
	pr_debug("%s,%d\r\n",__func__,__LINE__);
	pr_debug("as->current_transfer:0x%x\r\n",as->current_transfer);
	pr_debug("as->next_transfer:0x%x\r\n",as->next_transfer);
	u8 isCmdSupported;
	u8 unsupported_cmd[] = {SIF_CMD_FAST_READ_DUAL_IO,SIF_CMD_FAST_READ_QUAD_IO,SIF_CMD_READ_QUAD_IO,0xe3,0x77,0x92,0x94};
	
	if (!as->current_transfer)
		xfer = list_entry(msg->transfers.next,
				struct spi_transfer, transfer_list);
	else if (!as->next_transfer)
	{
		xfer = list_entry(as->current_transfer->transfer_list.next,
				struct spi_transfer, transfer_list);
		as->next_transfer = xfer;
	}
	else
		xfer = NULL;

	SIF_SET_DMA_CNT(as,0);

	spi_ctl = SIF_START|SIF_DMA_EN;
		
	if (xfer&&xfer->tx_buf) {

		pr_debug("xfer = %x\r\n",xfer);
		pr_debug("len = %d\r\n",xfer->len);

		len = as->cmd_len = xfer->len;
		
		vsnv3_sif_next_xfer_data(master, xfer, &tx_dma, &rx_dma, &len);
		remaining = xfer->len - len;
		pr_debug("remaining = %d\r\n",remaining);

		spi_cmd = *(AIT_REG_B*)xfer->tx_buf;
		SIF_SET_INT_SR(as,SIF_CLR_CMD_STATUS);
		SIF_SET_CMD(as,spi_cmd);	

		as->cmd = spi_cmd;
			
		for(i=0;i<sizeof unsupported_cmd;++i)
		{
			if(spi_cmd==unsupported_cmd[i])
			{
				pr_err("%s:SPI Command 0x%02x unsupported!",as->pdev->name,spi_cmd);
				vsnv3_sif_cmd_abort((unsigned long)master);
				return;
			}
		}
					
		spi_ctl_data_width = SIF_GET_CTL2(as)&(~( SIF_QUAD_IO_EN|SIF_DUAL_IO_EN));

		//TODO: route sif commands here 
		if(len<=2) //if xfer length < 2 , there is no chance to use DMA on TX, but if next transfer is RX , we will enable  SIF_DMA_EN again.
		{
#ifdef CONFIG_MTD_NAND_AIT_SPINAND
			u32 addr;

			spi_ctl |= SIF_ADDR_UNDETERMINED;
			spi_tx_addr =*(AIT_REG_D*)(xfer->tx_buf);
			spi_tx_addr>>=8;

			if(len == 2)	//for nand get status cmd(1) addr(1) data(1)
			{
				spi_ctl |= SIF_ADDR_EN;
				addr = (spi_tx_addr & 0xFF);

				SIF_SET_ADDR(as,addr);
			}
#else
			spi_ctl |= SIF_ADDR_UNDETERMINED;
#endif
		}
		else if(len<=4)
		{
			if(len ==3 && spi_cmd==SIF_CMD_WRITE_STATUS)	//TODO: special command , 1 byte command id , 2 bytes data
			{
				spi_ctl |= SIF_ADDR_UNDETERMINED;	//NO ADDRESS
				spi_ctl &= ~SIF_DMA_EN;				//NO DMA
				spi_ctl_data_width |= SIF_ADDR_2_BYTE;	//set data 2 bytes
			}
			else //sif command , 1 byte cmd with 2 or 3 bytes address  
			{ 
				u32 addr;
				spi_ctl |= (SIF_ADDR_EN|(len-2)&0x03);
				
				spi_tx_addr =*(AIT_REG_D*)(xfer->tx_buf);
				spi_tx_addr>>=8;	// remove cmdcd /mn

				if(len==4)	// 1 cmd 3 addr
					addr = (spi_tx_addr&0x0000ff)<<16 |(spi_tx_addr&0x00ff00)|(spi_tx_addr&0xff0000)>>16;
				else if (len==3)	// 1 cmd 2 addr
					addr = (spi_tx_addr&0x0000ff)<<8 |(spi_tx_addr&0x00ff00)>>8;
						
				//pr_info("address = 0x%.8X\r\n",addr);
				SIF_SET_ADDR(as,addr );
			}

		}
		else 
		{
			if((len==5)&&(spi_cmd == SIF_CMD_FAST_READ_DATA||spi_cmd ==SIF_CMD_FAST_READ_DUAL_IO))
				spi_ctl |= SIF_FAST_READ;
			else
				vsnv3_sif_cmd_abort((unsigned long)master);
		}			
	
		if(spi_cmd ==SIF_CMD_PAGE_QUAD_PROGRAM||spi_cmd ==SIF_CMD_FAST_READ_QUAD_OUTPUT)
		{

			spi_ctl_data_width|=SIF_QUAD_IO_EN;
		}else if(spi_cmd == SIF_CMD_FAST_READ_DUAL_OUTPUT )
		{
			spi_ctl_data_width|=SIF_DUAL_IO_EN;
		}

		if(spi_cmd==SIF_CMD_FAST_READ_DATA || spi_cmd == SIF_CMD_FAST_READ_DUAL_OUTPUT||spi_cmd ==SIF_CMD_FAST_READ_QUAD_OUTPUT )
			spi_ctl |= SIF_FAST_READ;
			
		pr_debug("CMD = 0x%02x\r\n",spi_cmd);
		pr_debug("TX ADDR = 0x%02x\r\n",spi_tx_addr);
		
	}
	else {
		xfer = as->next_transfer;
		remaining = as->next_remaining_bytes;
	}

	as->current_transfer = xfer;
	as->current_remaining_bytes = remaining;

	if (remaining > 0)
		len = remaining;
	else if (!vsnv3_sif_xfer_is_last(msg, xfer)
			&& vsnv3_sif_xfer_can_be_chained(xfer)) {
		xfer = list_entry(xfer->transfer_list.next,
				struct spi_transfer, transfer_list);
		len = xfer->len;
	}
	else //For no dma no second transfer and first transfer is TX case,
	{ 
#ifdef CONFIG_MTD_NAND_AIT_SPINAND
		spi_ctl |= SIF_R ;
		spi_ctl &= ~SIF_DMA_EN;
#else
		u8* data = ((u8*)as->current_transfer->tx_buf)+1;
		if(as->current_transfer->len == 1 )
		{
			spi_ctl |= SIF_W ;
			spi_ctl &= ~SIF_DMA_EN; 
			//SIF_SET_DATA_WR(as,(u16)*data);			
		}
		else if(as->current_transfer->len == 2 )
		{
			spi_ctl |= SIF_DATA_EN |SIF_W ;
			spi_ctl &= ~SIF_DMA_EN; 
			SIF_SET_DATA_WR(as,(u16)data[0]);
		}
		else if(as->current_transfer->len == 3 )
		{
			spi_ctl |= SIF_DATA_EN |SIF_W ;
			spi_ctl &= ~SIF_DMA_EN; 
			spi_ctl_data_width &= SIF_WRITE_2_BYTES;
			SIF_SET_DATA_WR(as,*(u16*)data);
		}
		else
		{
			spi_ctl |= SIF_R;
		}
#endif
		xfer = NULL;
	}

	as->next_transfer = xfer;

	if (xfer&&xfer->tx_buf) {
      
		pr_debug("next tx xfer = %x\r\n",xfer);
		
		u32	total;
		as->current_transfer = xfer;

		total = xfer->len;
		vsnv3_sif_next_xfer_data(master, xfer, &tx_dma, &rx_dma, &len);
		as->next_remaining_bytes = total - len;			
		dev_dbg(&msg->spi->dev,
			"  next xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
			xfer->rx_buf, xfer->rx_dma);

		if(likely(xfer->len!=0) && (spi_ctl&SIF_DMA_EN) ) 
		{
			SIF_SET_DMA_ADDR(as,xfer->tx_dma);

				SIF_SET_DMA_CNT(as,total);
			
			spi_ctl |= SIF_DATA_EN |SIF_W ;
			
			pr_debug("len = %x\r\n",total);			
		}
		else if( xfer->len!=0 && !(spi_ctl&SIF_DMA_EN) ) //a sif command has probability split to two transfers
		{
			//For handling 2 transfers.
			u8* data = ((u8*)xfer->tx_buf)+1;
			if(xfer->len == 2 )
			{
				spi_ctl |= SIF_DATA_EN |SIF_W ;
				spi_ctl &= ~SIF_DMA_EN; 
				SIF_SET_DATA_WR(as,(u16)data[0]);
			}
			else if(xfer->len == 3 )
			{
				spi_ctl |= SIF_DATA_EN |SIF_W ;
				spi_ctl &= ~SIF_DMA_EN; 
				SIF_SET_DATA_WR(as,*(u16*)data);
			}
		}
		else
		{
			SIF_SET_DMA_CNT(as,0);
			spi_ctl |= SIF_W;
		}
	}else if (xfer&&xfer->rx_buf) {
		pr_debug("next rx xfer = %x\r\n",xfer);
		
		u32	total;
		as->current_transfer = xfer;

			total = xfer->len;
			vsnv3_sif_next_xfer_data(master, xfer, &tx_dma, &rx_dma, &len);
			as->next_remaining_bytes = total - len;			
			dev_dbg(&msg->spi->dev,
				"  next xfer %p: len %u tx %p/%08x rx %p/%08x\n",
				xfer, xfer->len, xfer->tx_buf, xfer->tx_dma,
				xfer->rx_buf, xfer->rx_dma);

			if(likely(xfer->len!=0))
			{			
				SIF_SET_DMA_ADDR(as,as->buffer_dma);
#ifdef REMOVE_DUMMY_RX_DATA
				/* Dummy bytes will be read, it will be remove in isr */
				if(spi_cmd == SIF_CMD_FAST_READ_DUAL_OUTPUT)
					as->dumy_rx_data = 2;
				else if(spi_cmd == SIF_CMD_FAST_READ_QUAD_OUTPUT)
					as->dumy_rx_data = 0;					
				else
					as->dumy_rx_data = 0;
				SIF_SET_DMA_CNT(as,total+as->dumy_rx_data );

				// 0x05, 0xC8 for spi-nor. 0x0f for spi-nand
				if((spi_cmd == 0x05)|(spi_cmd == 0xC8)|(spi_cmd == 0x0f))	//reduce dma count for mci
				{
					SIF_SET_INT_EN(as, 0x00); // close interrupt
					spi_ctl |= SIF_DATA_EN | SIF_R;
					spi_ctl &= ~SIF_DMA_EN;
					SIF_SET_CTL(as, spi_ctl);

					while((SIF_GET_INT_SR(as)& SIF_CMD_DONE) == 0)
					{
					}

					if (!msg->is_dma_mapped)
					{
						vsnv3_sif_dma_unmap_xfer(master, xfer);
					}

					*((char *)(xfer->rx_buf)) = SIF_GET_DATA_RD(as);

					if(msg)
					{
						msg->actual_length += xfer->len+as->cmd_len;
					}

					vsnv3_sif_msg_done(master, as, msg, 0,0);
					return;
				}
#else
				SIF_SET_DMA_CNT(as,total);
#endif

				spi_ctl |= SIF_DATA_EN | SIF_R;
				pr_debug("rx_dma = %x\r\n",as->buffer_dma);				
				pr_debug("len = %x\r\n",total);			
			}
			else
			{
				SIF_SET_DMA_CNT(as,1);
		
				spi_ctl |= SIF_R  ;
			}

	}

	pr_debug("SIF ctl=0x%X, ctl2=0x%X, cmd=0x%X \r\n",spi_ctl,spi_ctl_data_width,spi_cmd);
	mod_timer(&as->timer, jiffies +  msecs_to_jiffies(1000));
#ifndef CONFIG_MTD_NAND_AIT_SPINAND
	SIF_SET_CTL2(as,spi_ctl_data_width);
#endif
	SIF_SET_CTL(as,spi_ctl );

	if((SIF_GET_INT_EN(as) & 0x01) == 0x00)
	{
		SIF_SET_INT_EN(as,SIF_CMD_DONE);
	}

	dev_dbg(&msg->spi->dev,"spi_ctl = 0x%x\r\n",spi_ctl);

	return;

}

static void vsnv3_sif_next_message(struct spi_master *master)
{
	struct vsnv3_spi *as = spi_master_get_devdata(master);
	struct spi_message	*msg;
	struct spi_device	*spi;

	pr_debug("%s,%d\r\n",__func__,__LINE__);
	BUG_ON(as->current_transfer);

	msg = list_entry(as->queue.next, struct spi_message, queue);
	spi = msg->spi;

	dev_dbg(master->dev.parent, "start message %p for %s\n",
			msg, dev_name(&spi->dev));
#if 0
	/* select chip if it's not still active */
	if (as->stay) {
		if (as->stay != spi) {
			cs_deactivate(as, as->stay);
			cs_activate(as, spi);
		}
		as->stay = NULL;
	} else
		cs_activate(as, spi);
#endif
	vsnv3_sif_next_xfer(master, msg);
}

/*
 * For DMA, tx_buf/tx_dma have the same relationship as rx_buf/rx_dma:
 *  - The buffer is either valid for CPU access, else NULL
 *  - If the buffer is valid, so is its DMA address
 *
 * This driver manages the dma address unless message->is_dma_mapped.
 */
static int
vsnv3_sif_dma_map_xfer(struct vsnv3_spi *as, struct spi_transfer *xfer)
{
	struct device	*dev = &as->pdev->dev;
	
	xfer->tx_dma = xfer->rx_dma = INVALID_DMA_ADDRESS;
		
	
	if (xfer->tx_buf) {
		/* tx_buf is a const void* where we need a void * for the dma
		 * mapping */
		//void *nonconst_tx = (void *)xfer->tx_buf;
		void *tx_ptr = (void *)xfer->tx_buf ;
		
		#if VMEM_FIX_EN==1
		if(! virt_addr_valid(xfer->tx_buf) ) {
		  tx_ptr = as->vmem_buffer;
		  memcpy( tx_ptr, xfer->tx_buf, xfer->len );
		}
		#endif
		
		xfer->tx_dma = dma_map_single(dev,
				tx_ptr, xfer->len,
				DMA_TO_DEVICE);
		
		if (dma_mapping_error(dev, xfer->tx_dma))
			return -ENOMEM;
	}
	if (xfer->rx_buf) {
    void *rx_ptr = (void *)xfer->rx_buf ;
    #if VMEM_FIX_EN==1
		if(! virt_addr_valid(xfer->rx_buf) ) {
		  rx_ptr = as->vmem_buffer;
		  memcpy( rx_ptr, xfer->rx_buf, xfer->len );
		}
    #endif
		xfer->rx_dma = dma_map_single(dev,
				rx_ptr/*xfer->rx_buf*/, xfer->len,
				DMA_FROM_DEVICE);
		
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			if (xfer->tx_buf)
				dma_unmap_single(dev,
						xfer->tx_dma, xfer->len,
						DMA_TO_DEVICE);
			return -ENOMEM;
		}
	}
	return 0;
}

static void vsnv3_sif_dma_unmap_xfer(struct spi_master *master,
				     struct spi_transfer *xfer)
{
	pr_debug("%s,%d\r\n",__func__,__LINE__);
	if (xfer->tx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(master->dev.parent, xfer->tx_dma,
				 xfer->len, DMA_TO_DEVICE);
	if (xfer->rx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(master->dev.parent, xfer->rx_dma,
				 xfer->len, DMA_FROM_DEVICE);
}

static void
vsnv3_sif_msg_done(struct spi_master *master, struct vsnv3_spi *as,
		struct spi_message *msg, int status, int stay)
{
	pr_debug("%s,%d\r\n",__func__,__LINE__);
#if 0
	if (!stay || status < 0)
		{
//		cs_deactivate(as, msg->spi);
		}
	else
		as->stay = msg->spi;
#endif
	list_del(&msg->queue);
	msg->status = status;

	dev_dbg(master->dev.parent,
		"xfer complete: %u bytes transferred\n",
		msg->actual_length);

	spin_unlock(&as->lock);
	msg->complete(msg->context);
	del_timer(&as->timer);
	spin_lock(&as->lock);

	as->current_transfer = NULL;
	as->next_transfer = NULL;

	/* continue if needed */
	if (list_empty(&as->queue) || as->stopping)
	{}//		spi_writel(as, PTCR, SPI_BIT(RXTDIS) | SPI_BIT(TXTDIS));
	else{
		vsnv3_sif_next_message(master);
	}
}


static void vsnv3_sif_timeout(unsigned long data)
{
	struct spi_message	*msg;

	struct spi_master *master = (struct spi_master *)data;

	struct vsnv3_spi *as = spi_master_get_devdata(master);


	pr_info("%s,%d\r\n",__func__,__LINE__);

	//xfer = as->current_transfer;
	msg = list_entry(as->queue.next, struct spi_message, queue);
	
	pr_info("SIF_GET_INT_EN = %x\r\n",SIF_GET_INT_EN(as));
	pr_info("SIF_GET_INT_SR = %x\r\n",SIF_GET_INT_SR(as));
	pr_info("SIF_GET_CTL = %x\r\n",SIF_GET_CTL(as));
	pr_info("SIF_GET_CTL2 = %x\r\n",SIF_GET_CTL2(as));
	pr_info("SIF_GET_CLK_DIV = %x\r\n",SIF_GET_CLK_DIV(as));
	pr_info("SIF_GET_CMD = %x\r\n",SIF_GET_CMD(as));
	pr_info("SIF_GET_DMA_ADDR = %x\r\n",SIF_GET_DMA_ADDR(as));
	pr_info("SIF_GET_DMA_CNT = %x\r\n",SIF_GET_DMA_CNT(as));
	pr_info("SIF_GET_CMD_DELAY = %x\r\n",SIF_GET_CMD_DELAY(as));
	pr_info("SIF_GET_ADDR = %x\r\n",SIF_GET_ADDR(as));

	SIF_SET_INT_SR(as,SIF_CMD_DONE);
	SIF_SET_DMA_CNT(as,0);



	if (!msg->is_dma_mapped)
	{
		vsnv3_sif_dma_unmap_xfer(master, as->current_transfer);
		if(as->next_transfer != NULL)
		{
			vsnv3_sif_dma_unmap_xfer(master, as->next_transfer);
		}
	}
	vsnv3_sif_msg_done(master, as, msg, -EIO, 0);
		
	return;
}


static irqreturn_t
vsnv3_sif_interrupt(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct vsnv3_spi *as = spi_master_get_devdata(master);
	struct spi_message	*msg;
	struct spi_transfer	*xfer;
	u32			status , int_en,int_mask;
	int			ret = IRQ_NONE;

	pr_debug("%s,%d\r\n",__func__);

	spin_lock(&as->lock);

	if(as)
		xfer = as->current_transfer;
	else
	{
		struct spi_message msg_tmp;
		msg = & msg_tmp;
		
		pr_err("vsnv3_sif_interrupt: as is NULL pointer\r\n");

		vsnv3_sif_msg_done(master, as, msg, -EIO,0);		
		return IRQ_HANDLED;
	}

	msg = list_entry(as->queue.next, struct spi_message, queue);

	int_mask = SIF_GET_INT_EN(as);
	status = SIF_GET_INT_SR(as);

	pr_debug("EN: 0x%08x\r\n",int_mask);
	pr_debug("SR 0x%08x\r\n",status);

	int_en = status & int_mask;

	if(likely(int_en&SIF_CMD_DONE))
	{
		SIF_SET_INT_SR(as,SIF_CMD_DONE);	

		if(xfer->rx_buf&&xfer->len)
		{
				memcpy(xfer->rx_buf,as->buffer+as->dumy_rx_data, xfer->len);
		}
		if(msg)
			msg->actual_length += xfer->len+as->cmd_len;
		else
		{
			pr_err("vsnv3_sif_interrupt: msg is NULL pointer\r\n");
			vsnv3_sif_msg_done(master, as, msg, -EIO,0);		
			return IRQ_HANDLED;
		}

		if (!msg->is_dma_mapped)
			vsnv3_sif_dma_unmap_xfer(master, xfer);
		
		if(xfer->rx_buf)
			pr_debug("rx data = %x\r\n",*(int*)xfer->rx_buf);
		/* REVISIT: udelay in irq is unfriendly */
//			if (xfer->delay_usecs)
//				udelay(xfer->delay_usecs);

		vsnv3_sif_msg_done(master, as, msg, 0,0);

		ret = IRQ_HANDLED;
	}
	else
	{
		ret = IRQ_NONE;
	}

	spin_unlock(&as->lock);

	return ret;
}

static int vsnv3_sif_setup(struct spi_device *spi)
{
	struct vsnv3_spi *as;
	struct vsnv3_spi_device	*asd;
	unsigned int		bits = spi->bits_per_word;
	unsigned long		bus_hz;
	int			ret;

	pr_debug("%s,%d\r\n",__func__,__LINE__);
	as = spi_master_get_devdata(spi->master);

	if (as->stopping)
		return -ESHUTDOWN;

	if (spi->chip_select > spi->master->num_chipselect) {
		dev_dbg(&spi->dev,
				"setup: invalid chipselect %u (%u defined)\n",
				spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (bits < 8 || bits > 16) {
		dev_dbg(&spi->dev,
				"setup: invalid bits_per_word %u (8 to 16)\n",
				bits);
		return -EINVAL;
	}


	/* v1 chips start out at half the peripheral bus speed. */
	bus_hz = clk_get_rate(as->clk);
	pr_debug("bus_hz = %d\r\n",bus_hz);


	if (spi->max_speed_hz) {
		u8  divisor=0;
		u32	bus_hz = clk_get_rate(as->clk);
		pr_info("bus_hz = %d divisor = %d , max spi : %d hz\r\n",bus_hz,divisor,spi->max_speed_hz);
		if(spi->max_speed_hz<(bus_hz/(2*(divisor+1))))
		{
			for(divisor=1;divisor<0x80;divisor++)
				if((bus_hz/(2*(divisor+1)))<=spi->max_speed_hz)
					break;
		}

		spi->max_speed_hz = bus_hz/(2*(divisor+1));

    

		SIF_SET_CLK_DIV(as,divisor);
		
		if(spi->max_speed_hz==33000000)
			SIF_SET_DATA_IN_LATCH(as,1);		//For 66MHz
		if(spi->max_speed_hz==66000000)
			SIF_SET_DATA_IN_LATCH(as,1);		//For 66MHz
			
		pr_info("%s:max_speed_hz = %d divisor = %d\r\n",dev_name(&spi->dev),spi->max_speed_hz,divisor);	
//		dev_WARN(&spi->dev, "Wanted speed: %d Hz , provided %d Hz\n",spi->max_speed_hz,bus_hz/(2*(divisor+1)));
		//return -ENOPROTOOPT;
	}

	if (spi->mode!=SPI_MODE_0)
		return -EINVAL;

	return 0;
}

static int vsnv3_sif_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct vsnv3_spi *as;
	struct spi_transfer	*xfer;
	unsigned long		flags;
	struct device		*controller = spi->master->dev.parent;
	u8			bits;

	AITPS_SIF pSIF = (AITPS_SIF) AITC_BASE_SIF;

	if (unlikely(msg==0))
		return -EINVAL;


	as = spi_master_get_devdata(spi->master);
	dev_dbg(controller, "new message %p submitted for %s\n",
			msg, dev_name(&spi->dev));

	if (unlikely(list_empty(&msg->transfers)))
		return -EINVAL;
	if (as->stopping)
		return -ESHUTDOWN;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!(xfer->tx_buf || xfer->rx_buf) && xfer->len) {
			dev_dbg(&spi->dev, "missing rx or tx buf\n");
			return -EINVAL;
		}
		if (xfer->bits_per_word) {
			if (bits != xfer->bits_per_word - 8) {
				dev_dbg(&spi->dev, "you can't yet change "
					 "bits_per_word in transfers\n");
				//return -ENOPROTOOPT;
			}
		}
		xfer->speed_hz=spi->max_speed_hz;
#if 1		
		if (xfer->speed_hz) {
			u32 target_speed;
			u8  divisor;
			u32	bus_hz = clk_get_rate(as->clk);
			//pr_info("bus_hz = %d\r\n",bus_hz);
			if(xfer->speed_hz>spi->max_speed_hz)
			{
				divisor = 10;
				target_speed = spi->max_speed_hz; 
			}
			else
			{
				for(divisor=0;divisor<0x80;divisor++)
					if((bus_hz/(2*(divisor+1)))<=xfer->speed_hz)
						break;
			}

      
			SIF_SET_CLK_DIV(as,divisor);
			if(xfer->speed_hz==66000000)
				SIF_SET_DATA_IN_LATCH(as,1);		//For 66MHz
			
			//pr_warn( "%s:Wanted speed: %d Hz , provided %d Hz\n",dev_name(&spi->dev),xfer->speed_hz,bus_hz/(2*(divisor+1)));
			///return -ENOPROTOOPT;
		}
#endif		
		/*
		 * DMA map early, for performance (empties dcache ASAP) and
		 * better fault reporting.  This is a DMA-only driver.
		 *
		 * NOTE that if dma_unmap_single() ever starts to do work on
		 * platforms supported by this driver, we would need to clean
		 * up mappings for previously-mapped transfers.
		 */
		if (!msg->is_dma_mapped) {
			if (vsnv3_sif_dma_map_xfer(as, xfer) < 0)
				return -ENOMEM;
		}
	}
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		dev_dbg(controller,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			xfer, xfer->len,
			xfer->tx_buf, xfer->tx_dma,
			xfer->rx_buf, xfer->rx_dma);
	}
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	spin_lock_irqsave(&as->lock, flags);
	list_add_tail(&msg->queue, &as->queue);
	if (!as->current_transfer)
		vsnv3_sif_next_message(spi->master);
	spin_unlock_irqrestore(&as->lock, flags);
	return 0;
}

static void vsnv3_sif_cleanup(struct spi_device *spi)
{
	spi->controller_state = NULL;
}

/*-------------------------------------------------------------------------*/

static int __devinit vsnv3_sif_probe(struct platform_device *pdev)
{
	struct resource		*regs;
	int			irq;
	struct clk		*clk;
	int			ret;
	struct spi_master	*master;
	struct vsnv3_spi	*as;
	AITPS_SIF pSIF = AITC_BASE_SIF;
	AITPS_MCI pMCI = AITC_BASE_MCI;

        pMCI->MCI_INIT_DEC_VAL[MCI_SRC_DMAR1_ICON_SIF] = MCI_INIT_WT_MAX;
        pMCI->MCI_NA_INIT_DEC_VAL[MCI_SRC_DMAR1_ICON_SIF] = MCI_INIT_WT_MAX;
        pMCI->MCI_ROW_INIT_DEC_VAL[MCI_SRC_DMAR1_ICON_SIF] = MCI_INIT_WT_MAX;
        pMCI->MCI_RW_INIT_DEC_VAL[MCI_SRC_DMAR1_ICON_SIF] = MCI_INIT_WT_MAX;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = clk_get(&pdev->dev, "sif_ctl_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	/* setup spi core then atmel-specific driver state */
	ret = -ENOMEM;
	master = spi_alloc_master(&pdev->dev, sizeof *as);
	if (!master)
		goto alloc_master_fail;

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	master->bus_num = pdev->id;
	master->num_chipselect = 1;
	master->setup = vsnv3_sif_setup;
	master->transfer = vsnv3_sif_transfer;
	master->cleanup = vsnv3_sif_cleanup;
	platform_set_drvdata(pdev, master);

	as = spi_master_get_devdata(master);

	/*
	 * Scratch buffer is used for throwaway rx and tx data.
	 * It's coherent to minimize dcache pollution.
	 */
	as->buffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					&as->buffer_dma, GFP_KERNEL);
	if (!as->buffer)
		goto alloc_master_fail;
#if VMEM_FIX_EN
	as->vmem_buffer = (void *)kmalloc(BUFFER_SIZE, GFP_KERNEL);
	if (!as->vmem_buffer) {
	  dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
			as->buffer_dma);
		goto alloc_master_fail;
  }
#endif

	spin_lock_init(&as->lock);
	INIT_LIST_HEAD(&as->queue);
	as->pdev = pdev;
	as->regs = ioremap(regs->start, resource_size(regs));
	if (!as->regs)
		goto ioremap_fail;
	as->irq = irq;
	as->clk = clk;



	/* Initialize the hardware */
	clk_enable(clk);

//clear int status.
	SIF_SET_INT_SR(as,0xff);	

	ret = request_irq(irq, vsnv3_sif_interrupt, 0,
			dev_name(&pdev->dev), master);
	if (ret)
		goto req_irq_fail;

#ifdef DEBUG
	SIF_SET_CLK_DIV(as,1);

	SIF_SET_DATA_IN_LATCH(as,1);		//For 66MHz

	SIF_SET_INT_SR(as,SIF_CLR_CMD_STATUS);
	SIF_SET_CMD(as,0x9f);	
	SIF_SET_DMA_ADDR(as,(AIT_REG_D)as->buffer_dma);
	SIF_SET_DMA_CNT(as,3);
	SIF_SET_CTL(as,SIF_START | SIF_DATA_EN | SIF_R | SIF_DMA_EN);
    int timeout = 0;

    while((SIF_GET_INT_SR(as)& SIF_CMD_DONE) == 0);

	pr_info("SIF ID: 0x%x\r\n",*(int*)as->buffer );
#endif
	dev_info(&pdev->dev, "AIT SPI Controller at 0x%08lx (irq %d)\n",
			(unsigned long)regs->start, irq);

	setup_timer(&as->timer, vsnv3_sif_timeout, (unsigned long)master);
	
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
	dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
			as->buffer_dma);
#if VMEM_FIX_EN
	kfree(as->vmem_buffer);
#endif			
alloc_master_fail:
	clk_put(clk);
	spi_master_put(master);
	return ret;
}

static int __devexit vsnv3_sif_remove(struct platform_device *pdev)
{
	struct spi_master	*master = platform_get_drvdata(pdev);
	struct vsnv3_spi	*as = spi_master_get_devdata(master);
	struct spi_message	*msg;

	/* reset the hardware and block queue progress */
	spin_lock_irq(&as->lock);
	as->stopping = 1;

	spin_unlock_irq(&as->lock);

	/* Terminate remaining queued transfers */
	list_for_each_entry(msg, &as->queue, queue) {
		/* REVISIT unmapping the dma is a NOP on ARM and AVR32
		 * but we shouldn't depend on that...
		 */
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}

	dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
			as->buffer_dma);

	clk_disable(as->clk);
	clk_put(as->clk);
	free_irq(as->irq, master);
	iounmap(as->regs);

	/* Timer for timeouts */
	del_timer_sync(&as->timer);

	spi_unregister_master(master);

	return 0;
}

#ifdef	CONFIG_PM

static int vsnv3_sif_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct spi_master	*master = platform_get_drvdata(pdev);
	struct vsnv3_spi	*as = spi_master_get_devdata(master);

	clk_disable(as->clk);
	return 0;
}

static int vsnv3_sif_resume(struct platform_device *pdev)
{
	struct spi_master	*master = platform_get_drvdata(pdev);
	struct vsnv3_spi	*as = spi_master_get_devdata(master);

	clk_enable(as->clk);
	return 0;
}

#else
#define	vsnv3_sif_suspend	NULL
#define	vsnv3_sif_resume	NULL
#endif


static void vsnv3_sif_shutdown(struct platform_device *pdev)
{
	struct spi_master	*master = platform_get_drvdata(pdev);
	struct vsnv3_spi	*as = spi_master_get_devdata(master);
	struct spi_message	*msg;

	/* reset the hardware and block queue progress */
	spin_lock_irq(&as->lock);
	as->stopping = 1;

	spin_unlock_irq(&as->lock);

	/* Terminate remaining queued transfers */
	list_for_each_entry(msg, &as->queue, queue) {
		/* REVISIT unmapping the dma is a NOP on ARM and AVR32
		 * but we shouldn't depend on that...
		 */
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}

	dma_free_coherent(&pdev->dev, BUFFER_SIZE, as->buffer,
			as->buffer_dma);

	clk_disable(as->clk);
	clk_put(as->clk);

	SIF_SET_INT_EN(as,0);
		
	free_irq(as->irq, master);
	iounmap(as->regs);

	/* Timer for timeouts */
	del_timer_sync(&as->timer);

	spi_unregister_master(master);

}

static struct platform_driver vsnv3_sif_driver = {
	.driver		= {
		.name	= "vsnv3_sif",
		.owner	= THIS_MODULE,
	},
	.probe		= vsnv3_sif_probe,
	.remove		= __exit_p(vsnv3_sif_remove),
	.shutdown	= vsnv3_sif_shutdown,
	.suspend	= vsnv3_sif_suspend,
	.resume		= vsnv3_sif_resume,
};
module_platform_driver(vsnv3_sif_driver);

MODULE_DESCRIPTION("AIT Vsion V3 Controller driver");
MODULE_AUTHOR("Vincent Chen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ait_vsnv3_spi");
