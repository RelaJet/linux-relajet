/*
 *  linux/drivers/mmc/host/vsnv3_mmc.c - AIT MCI Driver
 *  Modify from linux/drivers/mmc/host/at91_mci.c
 *
 *  Copyright (C) 2005 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 *  Copyright (C) 2006 Malcolm Noyes
 *
 *  Copyright (C) 2013 Alpha Image Techenology , All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>
#include <linux/highmem.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/version.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <mach/board.h>
#include <mach/cpu.h>

#include <mach/mmp_reg_sd.h>
#include <linux/irq.h>


#define GO_IDLE_STATE       0
#define SEND_OP_COND        1   // reserved for SD
#define ALL_SEND_CID        2
#define SET_RELATIVE_ADDR   3
#define SEND_RELATIVE_ADDR  3
#define SET_DSR             4
#define IO_SEND_OP_COND     5
#define SWITCH_FUNC         6
#define SELECT_CARD         7
#define DESELECT_CARD       7
#define SEND_IF_COND        8
#define SEND_EXT_CSD        8
#define SEND_CSD            9
#define SEND_CID            10
#define READ_DAT_UTIL_STOP  11  // reserved for SD
#define STOP_TRANSMISSION   12
#define SEND_STATUS         13
#define GO_INACTIVE_STATE   15

// Block oriented read commands (class 2)
#define SET_BLOCKLEN        16
#define READ_SINGLE_BLOCK   17
#define READ_MULTIPLE_BLOCK 18

// Block oriented write commands (class 4)
#define WRITE_BLOCK         24
#define WRITE_MULTIPLE_BLOCK    25
#define PROGRAM_CSD         27

// Erase commands
#define ERASE_WR_BLK_START  32
#define ERASE_WR_BLK_END    33
#define ERASE               38

// Application specific commands (class 8)
#define APP_CMD             55
#define GEN_CMD             56

// SD Application command Index
#define SET_BUS_WIDTH           6
#define SD_STATUS               13
#define SEND_NUM_WR_BLOCKS      22
#define SET_WR_BLK_ERASE_COUNT  23
#define SD_APP_OP_COND          41
#define SET_CLR_CARD_DETECT     42
#define SEND_SCR                51
#define IO_SET_BUS_WIDTH        52
#define IO_SET_CLR_CARD_DETECT  52
#define IO_RW_DIRECT            52
#define IO_RW_EXTENDED          53

// SD Vender command
#define VENDER_COMMAND_62       62

#define SD_DEFAULT_TIMEOUT           0xFFFF             		// 24M 
#define SD_NORMAL_READ_TIMEOUT       0xFFFF             // 24M => (100*24000/256)*1.2   (20% tolerance)
#define SD_NORMAL_WRITE_TIMEOUT     0xFFFF             // 24M => (250*24000/256)*1.2   (20% tolerance)
#define SD_HIGHSPEED_READ_TIMEOUT    0x1FFFF			// 48M => (100*48000/256)*1.2   (20% tolerance)
#define SD_HIGHSPEED_WRITE_TIMEOUT   0x1FFFF			// 48M => (250*48000/256)*1.2   (20% tolerance)


#define FL_SENT_COMMAND	(1 << 0)
#define FL_SENT_STOP	(1 << 1)

#define VSNV3_SD_ERRORS (DATA_CRC_ERR|DATA_TOUT |DATA_ST_BIT_ERR\
        |CMD_RESP_CRC_ERR | CMD_RESP_TOUT | BUSY_TOUT )

#define MCI_BLKSIZE 		512
#define MCI_MAXBLKSIZE 	4096
//#define DMA_BY_SRAM 
#ifdef DMA_BY_SRAM
#define MCI_BLKATONCE 	128
#else
#define MCI_BLKATONCE 	256
#endif
#define MCI_BUFSIZE 		(MCI_BLKSIZE * MCI_BLKATONCE)

extern ait_set_gpio_input(unsigned pin, int use_pullup);

/*
 * Low level type for this driver
 */
// #define RX_TASKLET
 
struct vsnv3sd_host
{
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_request *request;

	void __iomem *baseaddr;
	int irq;

	struct ait_sd_data *board;
	int present;

	struct clk *mci_clk;

	/*
	 * Flag indicating when the command has been sent. This is used to
	 * work out whether or not to send the stop
	 */
	unsigned int flags;
	/* flag for current bus settings */
	u32 bus_mode;

	/* DMA buffer used for transmitting */
	unsigned int* buffer;
	dma_addr_t physical_address;
	unsigned int total_length;
	unsigned int /*dma_data_direction*/ dma_dir;
	unsigned int *sgbuffer;
	/* Latest in the scatterlist that has been enabled for transfer, but not freed */
	int in_use_index;

	/* Latest in the scatterlist that has been enabled for transfer */
	int transfer_index;

	/* Timer for timeouts */
	struct timer_list timer;
	unsigned int status;

#ifdef RX_TASKLET
	struct tasklet_struct	rx_tasklet;
#endif
};

/*
 * Reset the controller and restore most of the state
 */
static void vsnv3_sd_reset_host(struct vsnv3sd_host *host)
{
	unsigned long flags;
	AIT_REG_W imr;
	AIT_REG_B imr2;
	AIT_REG_B imr3;

	AITPS_SD pSD = 	host->baseaddr;
	
	pr_debug("vsnv3_sd_reset_host: %s\n",__FUNCTION__);
	local_irq_save(flags);
#if 0	
	imr = at91_mci_read(host, AT91_MCI_IMR);



	at91_mci_write(host, AT91_MCI_IDR, 0xffffffff);

	/* save current state */
	mr = at91_mci_read(host, AT91_MCI_MR) & 0x7fff;
	sdcr = at91_mci_read(host, AT91_MCI_SDCR);
	dtor = at91_mci_read(host, AT91_MCI_DTOR);

	/* reset the controller */
	at91_mci_write(host, AT91_MCI_CR, AT91_MCI_MCIDIS | AT91_MCI_SWRST);

	/* restore state */
	at91_mci_write(host, AT91_MCI_CR, AT91_MCI_MCIEN);
	at91_mci_write(host, AT91_MCI_MR, mr);
	at91_mci_write(host, AT91_MCI_SDCR, sdcr);
	at91_mci_write(host, AT91_MCI_DTOR, dtor);
	at91_mci_write(host, AT91_MCI_IER, imr);

	/* make sure sdio interrupts will fire */
	at91_mci_read(host, AT91_MCI_SR);
#endif
	imr =pSD->SD_CPU_INT_EN;
	imr2 =pSD->SD_CPU_INT_EN_2;
	imr3 =pSD->SD_CPU_INT_EN_3;
	pSD->SD_CPU_INT_EN = 0;
	pSD->SD_CPU_INT_EN_2 = 0;
	pSD->SD_CPU_INT_EN_3 = 0;

	pSD->SD_CPU_INT_EN = imr;
	pSD->SD_CPU_INT_EN_2 = imr2;
	pSD->SD_CPU_INT_EN_3 = imr3;	
	
	local_irq_restore(flags);
}

static void vsnv3_sd_timeout_timer(unsigned long data)
{
	struct vsnv3sd_host *host;
	AITPS_SD pSD;
	
	host = (struct vsnv3sd_host *)data;

	pSD =  (AITPS_SD )host->baseaddr;
	
	pSD->SD_CPU_INT_SR = pSD->SD_CPU_INT_SR;

	if (host->request) {
		dev_err(host->mmc->parent, "Timeout waiting end of packet\n");
		dev_err(host->mmc->parent, "CMD %d\n",host->cmd->opcode);		
		dev_err(host->mmc->parent, "arg 0x%x\n",host->cmd->arg);	

		if (host->cmd && host->cmd->data) {
			host->cmd->data->error = -ETIMEDOUT;
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->request->cmd->error = -ETIMEDOUT;
		}

		vsnv3_sd_reset_host(host);
		mmc_request_done(host->mmc, host->request);
	}
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void vsnv3_sd_sg_to_dma(struct vsnv3sd_host *host, struct mmc_data *data)
{
	unsigned int len, i, size;
	unsigned *dmabuf = host->buffer;

	size = data->blksz * data->blocks;
	len = data->sg_len;
#if 0
	/* MCI1 rev2xx Data Write Operation and number of bytes erratum */
	if (at91mci_is_mci1rev2xx())
		if (host->total_length == 12)
			memset(dmabuf, 0, 12);
#endif
	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */
	for (i = 0; i < len; i++) {
		struct scatterlist *sg;
		int amount;
		unsigned int *sgbuffer;

		sg = &data->sg[i];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,8)
		sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
#else		
		sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;		
#endif		
		amount = min(size, sg->length);
		size -= amount;
		{
			char *tmpv = (char *)dmabuf;
			memcpy(tmpv, sgbuffer, amount);
			tmpv += amount;
			dmabuf = (unsigned *)tmpv;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,8)
		kunmap_atomic(sgbuffer);
#else
		kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
#endif		
		if (size == 0)
			break;
	}

	/*
	 * Check that we didn't get a request to transfer
	 * more data than can fit into the SG list.
	 */
	BUG_ON(size != 0);
}

/*
 * Handle after a dma read
 */
static void vsnv3_sd_post_dma_read(struct vsnv3sd_host *host)
{
	struct mmc_command *cmd;
	struct mmc_data *data;
	unsigned int len, i, size;
	unsigned *dmabuf = host->buffer;

	pr_debug("post dma read\n");

	cmd = host->cmd;
	if (!cmd) {
		pr_debug("no command\n");
		return;
	}

	data = cmd->data;
	if (!data) {
		pr_debug("no data\n");
		return;
	}

	size = data->blksz * data->blocks;
	len = data->sg_len;

	for (i = 0; i < len; i++) {
		struct scatterlist *sg;
		int amount;
		unsigned int *sgbuffer;

		sg = &data->sg[i];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,8)
		sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
#else
		sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
#endif		
		amount = min(size, sg->length);
		size -= amount;

		{
			char *tmpv = (char *)dmabuf;

			memcpy(sgbuffer, tmpv, amount);
			tmpv += amount;
			dmabuf = (unsigned *)tmpv;
		}

		flush_kernel_dcache_page(sg_page(sg));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,8)
		kunmap_atomic(sgbuffer);	
#else			
		kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
#endif		
		data->bytes_xfered += amount;
		if (size == 0)
			break;
	}

	pr_debug("post dma read done\n");
}

/*
 * Update bytes tranfered count during a write operation
 */
static void vsnv3_sd_update_bytes_xfered(struct vsnv3sd_host *host)
{
	struct mmc_data *data;
	pr_debug( 	"VSNV3 MMC: %s\n",__FUNCTION__);
	/* always deal with the effective request (and not the current cmd) */

	if (host->request->cmd && host->request->cmd->error != 0)
		return;

	if (host->request->data) {
		data = host->request->data;
		if (data->flags & MMC_DATA_WRITE) {
			/* card is in IDLE mode now */
			pr_debug("-> bytes_xfered %d, total_length = %d\n",
				data->bytes_xfered, host->total_length);
			data->bytes_xfered = data->blksz * data->blocks;
			pr_debug("-> bytes_xfered %d\r\n",data->bytes_xfered  );
		}
	}
}

/*
 * Enable the controller
 */
static void vsnv3_sd_enable(struct vsnv3sd_host *host)
{
	AITPS_SD pSD = host->baseaddr;
	pr_debug( 	"VSNV3 MCI: %s\n",__FUNCTION__);

	pSD->SD_CPU_INT_SR = (CMD_RESP_CRC_ERR|/*CMD_RESP_TOUT|*/BUSY_TOUT|CMD_SEND_DONE|SDIO_INT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE);
	
	pSD->SD_CTL_0 = ( SD_DMA_EN | CLK_IN_SYS_DIV );

	pSD->SD_CTL_1 |= CLK_EN | WAIT_LAST_BUSY_EN/*Vin | AUTO_CLK_EN*/ | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;

}

/*
 * Disable the controller
 */
static void vsnv3_sd_disable(struct vsnv3sd_host *host)
{

	AITPS_SD pSD = host->baseaddr;
	
	pSD->SD_CPU_INT_EN = 0;
	pSD->SD_CPU_INT_EN_2 = 0;
	pSD->SD_CPU_INT_EN_3 =0;

	pSD->SD_CPU_INT_SR = (CMD_RESP_CRC_ERR|/*CMD_RESP_TOUT|*/BUSY_TOUT|CMD_SEND_DONE|SDIO_INT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE);
	
//	at91_mci_write(host, AT91_MCI_CR, AT91_MCI_MCIDIS | AT91_MCI_SWRST);
}

/*
 * Send a command
 */

static void vsnv3_sd_send_command(struct vsnv3sd_host *host, struct mmc_command *cmd)
{
	unsigned int block_length;
	struct mmc_data *data = cmd->data;

	unsigned int blocks;

	AITPS_SD pSD = host->baseaddr;

	host->cmd = cmd;

	/* Needed for leaving busy state before CMD1 */


	if ((pSD->SD_CPU_INT_SR & CMD_RESP_TOUT) && (cmd->opcode == 1)) {
		pr_debug("Clearing timeout\n");
		pSD->SD_CPU_INT_SR |= CMD_RESP_TOUT; 

		{
			/* spin */
			pr_debug("Clearing: SR = %08X\n", pSD->SD_CPU_INT_SR);
		}
	}
	pSD->SD_CPU_INT_SR = 0xffff;

	if (data) {

		if (data->blksz<1 || data->blksz >MCI_MAXBLKSIZE) {
			pr_debug("Unsupported block size\n");
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->request);
			return;
		}

		
		if (data->blocks<1 || data->blocks >MCI_BLKATONCE) {	//If AIT8455 can reach , Not verify yet.
			pr_debug("Unsupported %d blocks \n",data->blocks );
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->request);
			return;
		}
			
		if (data->flags & MMC_DATA_STREAM) {
			pr_debug("Stream commands not supported\n");
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->request);
			return;
		}


		block_length = data->blksz;
		blocks = data->blocks;

		if(((cmd->opcode==WRITE_BLOCK||cmd->opcode==READ_SINGLE_BLOCK)&& blocks!=1)||((cmd->opcode==READ_MULTIPLE_BLOCK||cmd->opcode==WRITE_MULTIPLE_BLOCK)&&blocks==1))
		{
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->request);
			return;
		}

		if(cmd->opcode==SEND_STATUS&&(block_length!=64 ||blocks!=1))
		{
			pr_info("SD CMD 13 and none zero block require(blk len = %d, blks = %d\n",block_length,blocks);
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->request);
			return;
		}
	}
	else {
		block_length = 0;
		blocks = 0;
	}

	pSD->SD_CMD_REG_0 = cmd->opcode;

	switch (mmc_resp_type(cmd)) {

		case MMC_RSP_NONE:
//			pSD->SD_CMD_REG_0 |= NO_RESP;
			break;

		case MMC_RSP_R1B:
			pSD->SD_CMD_REG_0 |= R1B_RESP;
			break;
			
		case MMC_RSP_R2:   
			pSD->SD_CMD_REG_0 |= R2_RESP;
			break;
			
		default: /* others: R1,R3,R4,R5,R6 */
				pSD->SD_CMD_REG_0 |= OTHER_RESP;

			break;

	}



	pSD->SD_CMD_ARG = cmd->arg; /* SD command's argument */
	
    /* Clear interrupt status */
	pSD->SD_CPU_INT_SR = (CMD_RESP_CRC_ERR|/*CMD_RESP_TOUT|*/BUSY_TOUT|CMD_SEND_DONE|SDIO_INT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE);
	pSD->SD_CPU_INT_EN |= /*CMD_SEND_DONE |*/ CMD_RESP_TOUT | BUSY_TOUT;

	if(cmd->flags&MMC_RSP_CRC)
	{
		pSD->SD_CPU_INT_EN |= CMD_RESP_CRC_ERR;
	}

	/*
	 * Set the arguments and send the command
	 */
	pr_debug("Sending command %d, arg = %08X, blocks = %d, length = %d (MR = %08X)\n",
		cmd->opcode, cmd->arg, blocks, block_length, pSD->SD_CTL_0);

//	spin_lock(&host->mmc->lock);


    /* Clear interrupt status */

	if ((cmd->opcode == SET_BLOCKLEN))
	{
		pSD->SD_BLK_NUM = cmd->arg;
	}

	if (!data) {
		pSD->SD_CPU_INT_EN |= CMD_SEND_DONE;
		pSD->SD_CMD_REG_1 |= SEND_CMD;		
	} else {
		struct scatterlist *sg;
		pSD->SD_BLK_LEN = block_length;
		pSD->SD_BLK_NUM = blocks;


		sg = data->sg;

		data->bytes_xfered = 0;
		host->transfer_index = 0;
		host->in_use_index = 0;
		
		if(cmd->opcode==SEND_STATUS)
		{
			pSD->SD_DMA_ST_ADDR = (AIT_REG_D)host->physical_address;

			pr_debug("SEND_STATUS  \n");

			pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);
		
		}				
		else if(data->flags & MMC_DATA_READ){
			
			/*
			 * Handle a read
			 */
			host->total_length = 0;
#if 0	//For reference
			if(data->sg_len==1)
			{
				host->dma_dir = DMA_FROM_DEVICE;
				if(dma_map_sg(mmc_dev(host->mmc), sg,1,host->dma_dir)!=1){
					cmd->error = -EINVAL;
					return;
				}

				pSD->SD_DMA_ST_ADDR = (AIT_REG_D)sg_dma_address(sg);	

			}					
			else
#endif
				pSD->SD_DMA_ST_ADDR = (AIT_REG_D)host->physical_address;
	

			pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);
			
		} else if (data->flags & MMC_DATA_WRITE) {
			/*
			 * Handle a write
			 */
			host->total_length = block_length * blocks;



#if 0
			if(data->sg_len==1)
			{
				host->dma_dir = DMA_TO_DEVICE;
				if(dma_map_sg(mmc_dev(host->mmc), sg,1,host->dma_dir)!=1){
					return -EINVAL;
				}

				pSD->SD_DMA_ST_ADDR = (AIT_REG_D)sg_dma_address(sg);					
			}					
			else
#endif					
			{
				vsnv3_sd_sg_to_dma(host, data);
				
				pSD->SD_DMA_ST_ADDR = (AIT_REG_D)host->physical_address;
			}				
	
			pSD->SD_CMD_REG_1 |= (ADTC_WRITE|SEND_CMD);
			
		}
	}

//	spin_unlock(&host->mmc->lock);

}

/*
 * Process the next step in the request
 */
static void vsnv3_sd_process_next(struct vsnv3sd_host *host)
{

	if (!(host->flags & FL_SENT_COMMAND)) {
		host->flags |= FL_SENT_COMMAND;
		vsnv3_sd_send_command(host, host->request->cmd);
	}
	else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
		host->flags |= FL_SENT_STOP;
		vsnv3_sd_send_command(host, host->request->stop);
	} else {
		del_timer(&host->timer);
		mmc_request_done(host->mmc, host->request);
	}
}

/*
 * Handle a command that has been completed
 */
static void vsnv3_sd_completed_command(struct vsnv3sd_host *host, unsigned int status)
{
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = cmd->data;
	AITPS_SD pSD = host->baseaddr;

	switch (mmc_resp_type(cmd)) {
	default:

	case MMC_RSP_NONE:
	case MMC_RSP_R1:	//   	eq to MMC_RSP_R5   	MMC_RSP_R6 MMC_RSP_R7:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		cmd->resp[0] = pSD->SD_RESP.D[3];/* OCR Status*/
		break;
	case MMC_RSP_R2:   
		cmd->resp[0] = pSD->SD_RESP.D[3];/* CID or SSD */
		cmd->resp[1] = pSD->SD_RESP.D[2];
		cmd->resp[2] = pSD->SD_RESP.D[1];
		cmd->resp[3] = pSD->SD_RESP.D[0];
		break;

	}

	pr_debug("Status = %08X/%08x [%08X %08X %08X %08X]\n",
		 status, pSD->SD_CPU_INT_SR,
		 cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

	if ((status&0xff00) == DATA_SEND_DONE) {
		cmd->error = 0;

	}
	else if (status & VSNV3_SD_ERRORS) {
		if ((status & CMD_RESP_CRC_ERR) && (mmc_resp_type(cmd) & MMC_RSP_CRC))
		{
			cmd->error = -EILSEQ;//cmd->error = 0;
		}
		else
		{		
			if ((status & CMD_RESP_CRC_ERR) && !(mmc_resp_type(cmd) & MMC_RSP_CRC)) {
				cmd->error = 0;
			}
			else
			{
			if (status & (DATA_TOUT | DATA_CRC_ERR)) {
				if (data) {
					if (status & DATA_TOUT)
						data->error = -ETIMEDOUT;
					else if (status & DATA_CRC_ERR)
					{
						if(cmd->opcode==WRITE_BLOCK && cmd->data &&(cmd->data->blocks>1))
							data->error = -ETIMEDOUT;
						else
							data->error = -EILSEQ;
					}
				}
			} else {
				if (status & CMD_RESP_TOUT)//if (status & AT91_MCI_RTOE)
					cmd->error = -ETIMEDOUT;
				else if (status & CMD_RESP_CRC_ERR)//else if (status & AT91_MCI_RCRCE)
					cmd->error = -EILSEQ;
				else
					cmd->error = -EIO;
			}

			pr_debug("Error detected and set to %d/%d (cmd = %d, retries = %d)\n",
				cmd->error, data ? data->error : 0,
				 cmd->opcode, cmd->retries);
			}
		}
	}
	else
		cmd->error = 0;

	vsnv3_sd_process_next(host);
}

/*
 * Handle an MMC request
 */
static void vsnv3_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct vsnv3sd_host *host = mmc_priv(mmc);
	host->request = mrq;
	host->flags = 0;
	pr_debug("VSNV3 MMC: %s Cmd = %d\n",__FUNCTION__,mrq->cmd->opcode);
	/* more than 1s timeout needed with slow SD cards */

	mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));

	vsnv3_sd_process_next(host);
}

/*
 * Set the IOS
 */
static void vsnv3_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int clkdiv;
	
	struct vsnv3sd_host *host = mmc_priv(mmc);
	int sdid = host->board->controller_id;
	unsigned int vsnv3_master_clock = clk_get_rate(host->mci_clk);
	AITPS_SD pSD;// = host->baseaddr;
	static int curDiv = 0;
	int time_out = 0;

	pSD = host->baseaddr;

	pr_debug("VSNV3 MCI(%d): vsnv3_master_clock = %d  ios->clock = %d\n",sdid,vsnv3_master_clock,ios->clock);
	//printk("VSNV3 MCI(%d): vsnv3_master_clock = %d  ios->clock = %d\n",sdid,vsnv3_master_clock,ios->clock);

	host->bus_mode = ios->bus_mode;

	if (ios->clock == 0) {
		/* Disable the MCI controller */
//		at91_mci_write(host, AT91_MCI_CR, AT91_MCI_MCIDIS);
		clkdiv = 0;
	}
	else {
		/* Enable the MCI controller */

		if ((vsnv3_master_clock % (ios->clock * 2)) == 0)
		{
			clkdiv= (vsnv3_master_clock /(2*ios->clock ))-1;
		}
		else
			clkdiv= (vsnv3_master_clock /(2*ios->clock ));


			if(curDiv!=clkdiv)
			{

				// Wait for SD clk can be switched
				while(!(pSD->SD_CPU_FIFO_SR & CLK_SWITCH_INDICATE)) {
					time_out++;
					if (time_out >= 0x100000) {

						break;
					}
				}

				pSD->SD_CTL_0 &= 0xF3;

				// Assign new SD clock
				if(clkdiv == SD_CLOCK_DIV_1) {

					pSD->SD_CTL_0 |= CLK_IN_NORM_SYS;
				}
				else if (clkdiv == SD_CLOCK_DIV_3) {
					pSD->SD_CTL_0 |= CLK_IN_SYS_DIV_3;
				}
				else {
					pSD->SD_CLK_CTL = clkdiv;
				}

				curDiv = clkdiv;
			}

		//pr_info("pSD->SD_CLK_CTL = 0x%X\r\n",pSD->SD_CLK_CTL );
		pr_debug("clkdiv = %d. mmc clock = %ld\n", clkdiv, vsnv3_master_clock / (2 * (clkdiv + 1)));
	}

	pSD->SD_CTL_0 &= ~(BUS_WIDTH_8|BUS_WIDTH_4|BUS_WIDTH_1);

	pr_debug("MMC: Setting controller bus width to %d\n",ios->bus_width == MMC_BUS_WIDTH_4?4:((ios->bus_width == MMC_BUS_WIDTH_8)?8:1));

	if (ios->bus_width == MMC_BUS_WIDTH_4 && host->board->wire4) {
		pSD->SD_CTL_0|= BUS_WIDTH_4;
	}
	else if (ios->bus_width == MMC_BUS_WIDTH_8 && host->board->wire4 &&host->board->bus_width==8) {
		pSD->SD_CTL_0|= BUS_WIDTH_8;
	}
	else {
		pSD->SD_CTL_0|= BUS_WIDTH_1;		
	}

	pSD->SDIO_ATA_CTL = SDIO_EN;


	/* Set the clock divider */

	/* maybe switch power to the card */
	if (host->board->vcc_pin) {
		switch (ios->power_mode) {
			case MMC_POWER_OFF:
				//gpio_set_value(host->board->vcc_pin, 0);
				break;
			case MMC_POWER_UP:
				//gpio_set_value(host->board->vcc_pin, 1);
				break;
			case MMC_POWER_ON:
				break;
			default:
				WARN_ON(1);
		}
	}

	pSD->SD_CMD_RESP_TOUT_MAX = 0xFF;
#if defined(CONFIG_ARCH_MCRV2)
	pSD->SD_DATA_TOUT = SD_DEFAULT_TIMEOUT;
#else
	pSD->SD_DATA_TOUT[0] = 0xff;
	pSD->SD_DATA_TOUT[1] = 0xff;
	pSD->SD_DATA_TOUT[2] = 0xff;

#endif
	
}
#ifdef RX_TASKLET

static void vsnv3_sd_rx_tasklet(unsigned long data)
{
	struct vsnv3sd_host *host = (struct vsnv3sd_host *)data;


	vsnv3_sd_post_dma_read(host);
	vsnv3_sd_completed_command(host, host->status);


}
#endif

/*
 * Handle an interrupt
 */
static irqreturn_t vsnv3_sd_irq(int irq, void *devid)
{
	struct vsnv3sd_host *host = devid;
	volatile int completed = 0;
	volatile unsigned int int_status, int_mask;

	BUG_ON(!host);
	BUG_ON(!host->board);
	
	AITPS_SD pSD = host->baseaddr;
	int sdid = host->board->controller_id;
	int sdcmd;	
	int_status =pSD->SD_CPU_INT_SR;
	int_mask = pSD->SD_CPU_INT_EN;
	
	pr_debug("MCI irq(%d): status = %08X, %08X, %08X\n", sdid,int_status, int_mask,
		int_status & int_mask);

	int_status &= int_mask;

	if(unlikely(!int_status))
		return IRQ_NONE;

	BUG_ON(!host->cmd);
	
	sdcmd = host->cmd->opcode;			
	host->status = int_status;
	
	if (int_status & VSNV3_SD_ERRORS) {
		completed = 1;

		if (int_status & BUSY_TOUT)
			printk(KERN_DEBUG "MMC(%d): CMD %d Busy timeout\n",sdid,sdcmd);
		if (int_status & DATA_TOUT)
			printk(KERN_DEBUG "MMC(%d): CMD %d Data timeout\n",sdid,sdcmd);
		if (int_status & DATA_CRC_ERR)
			printk(KERN_DEBUG "MMC(%d): CMD %d CRC error in data\n",sdid,sdcmd);
		if (int_status & CMD_RESP_TOUT)
			printk(KERN_DEBUG "MMC(%d): CMD %d Response timeout\n",sdid,sdcmd);
		if (int_status & DATA_ST_BIT_ERR)
			printk(KERN_DEBUG "MMC(%d): CMD %d Response start bit error\n",sdid,sdcmd);
		if (int_status & CMD_RESP_CRC_ERR)
			printk(KERN_DEBUG "MMC(%d): CMD %d Response CRC error\n",sdid,sdcmd);

		pSD->SD_CPU_INT_SR = VSNV3_SD_ERRORS;
		if(int_status&CMD_SEND_DONE)
		{
			pSD->SD_CPU_INT_SR = CMD_SEND_DONE;
		}
	} 
	else {
		/* Only continue processing if no errors */

		if (int_status & DATA_SEND_DONE) {
			pr_debug("Data send done\n");
		
			{
			
				if(host->cmd->data->flags & MMC_DATA_READ) {
#ifdef RX_TASKLET

					tasklet_schedule(&host->rx_tasklet);
					pSD->SD_CPU_INT_SR = DATA_SEND_DONE;
					return IRQ_HANDLED;

#else

#if 0	//For reference
					if(host->cmd->data->sg_len==1)
					{		
						host->cmd->data->bytes_xfered = min(host->cmd->data->blksz * host->cmd->data->blocks, host->cmd->data->sg->length);
						dma_unmap_sg(mmc_dev(host->mmc), host->cmd->data->sg,1,host->dma_dir);
//						__dma_unmap_page(mmc_dev(host->mmc), sg_dma_address(host->cmd->data->sg), sg_dma_len(host->cmd->data->sg), host->dma_dir);

					}					
					else
#endif						
					vsnv3_sd_post_dma_read(host);
#endif
				}
				else{
					if(host->cmd->data->sg_len==1)
					{
						host->cmd->data->bytes_xfered = host->cmd->data->blksz * host->cmd->data->blocks;
						dma_unmap_sg(mmc_dev(host->mmc), host->cmd->data->sg,1,host->dma_dir);						
					}
					else
						vsnv3_sd_update_bytes_xfered(host);
				}
			}
				
			completed = 1;

			pSD->SD_CPU_INT_SR = DATA_SEND_DONE;
		}
		else if (int_status & SDIO_INT) {
			pr_debug("SDIO_INT has ended %d, status:%08x, mask:%08x\n", completed, int_status, int_mask);
			
			mmc_signal_sdio_irq(host->mmc);

			pSD->SD_CPU_INT_SR = SDIO_INT;
		}
		else if (int_status & CMD_SEND_DONE) {
			pr_debug("Command send done\n");

			if (likely(host->cmd&&host->cmd->data)) {
				pSD->SD_CPU_INT_EN |= DATA_CRC_ERR | DATA_ST_BIT_ERR | DATA_SEND_DONE | DATA_TOUT;
			}
			else {
				pSD->SD_CPU_INT_EN &= ~(DATA_CRC_ERR | DATA_ST_BIT_ERR | DATA_SEND_DONE | DATA_TOUT);
			}

			if (unlikely(!host->cmd->data )) {
				completed = 1;
			}

			pSD->SD_CPU_INT_SR = CMD_SEND_DONE;
		}
		else if (int_status & CE_ATA_CMD_COMPLETE) {
			pr_debug("CE_ATA_CMD_COMPLETE\n");
		}

	}

	if (completed) {
		pr_debug("Completed command\n");
		vsnv3_sd_completed_command(host, int_status);
	} 

	return IRQ_HANDLED;
}

static irqreturn_t vsnv3_sd_det_irq(int irq, void *_host)
{
	struct vsnv3sd_host *host = _host;
	int present;
	AITPS_SD pSD = host->baseaddr;
	pr_debug("AIT MCI: %s\n",__FUNCTION__);
	/* entering this ISR means that we have configured det_pin:
	 * we can use its value in board structure */
	//pr_info("!!! vsnv3_sd_det_irq  \n");
	if (host->board->det_pin)
		present = gpio_get_value(host->board->det_pin)^host->board->active_low ;
	else
		present = 0;
	/*
	 * we expect this irq on both insert and remove,
	 * and use a short delay to debounce.
	 */
	if (present != host->present) {
		host->present = present;
		pr_info("%s: card %s\n", mmc_hostname(host->mmc),
			present ? "insert" : "remove");
		if (!present) {
			pr_debug("****** Resetting SD-card bus width ******\n");
			 pSD->SD_CTL_0 &= ~(BUS_WIDTH_8 | BUS_WIDTH_4 | BUS_WIDTH_1);
		}
		/* 0.5s needed because of early card detect switch firing */
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
	}
	return IRQ_HANDLED;
}

static int vsnv3_sd_get_ro(struct mmc_host *mmc)
{
	struct vsnv3sd_host *host = mmc_priv(mmc);
	pr_debug("VSNV3 MMC: %s\n",__FUNCTION__);
	if (host->board->wp_pin)
		return !!gpio_get_value(host->board->wp_pin);
	/*
	 * Board doesn't support read only detection; let the mmc core
	 * decide what to do.
	 */
	return -ENOSYS;
}

static int vsnv3_sd_get_cd(struct mmc_host *mmc)
{
	int			present = -ENOSYS;
	struct  vsnv3sd_host *host = mmc_priv(mmc);
	if (host->board->det_pin) 
		if (gpio_is_valid(host->board->det_pin)) {
			present = (gpio_get_value(host->board->det_pin))^host->board->active_low;
			//present = !(gpio_get_value(host->board->det_pin));// ^
	//			    host->detect_is_active_high);
			dev_dbg(&mmc->class_dev, "card is %spresent\n",
					present ? "" : "not ");
		}

	return present;
}

static void vsnv3_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct vsnv3sd_host *host = mmc_priv(mmc);
	AITPS_SD pSD = host->baseaddr;
	
	pr_debug("%s: sdio_irq %c : %s\n", mmc_hostname(host->mmc),
		host->board->controller_id ? '1':'0', enable ? "enable" : "disable");

	if(enable) {
		pSD->SD_CPU_INT_EN |= SDIO_INT;
	}
	else {
		pSD->SD_CPU_INT_EN &= ~(SDIO_INT);
	}

}

void	vsnv3_hw_reset(struct mmc_host *host)
{
	struct vsnv3sd_host *vsv3_host = mmc_priv(host);
	AITPS_SD pSD = vsv3_host->baseaddr;
	
	vsnv3_sd_reset_host(vsv3_host );

	pSD->SD_CTL_0|= BUS_WIDTH_1;
	
}

static const struct mmc_host_ops vsnv3_mci_ops = {
	.request	= vsnv3_sd_request,
	.set_ios	= vsnv3_sd_set_ios,
	.get_ro		= vsnv3_sd_get_ro,
	.get_cd    = vsnv3_sd_get_cd,
	.enable_sdio_irq = vsnv3_sd_enable_sdio_irq,
	.hw_reset = vsnv3_hw_reset

};

/*
 * Probe for the device
 */
static int __init vsnv3_sd_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct vsnv3sd_host *host;
	struct resource *res;
	int ret;
	int irq;

	pr_debug("vsnv3_sd_probe: %s\n",pdev->name);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	mmc = mmc_alloc_host(sizeof(struct vsnv3sd_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
		goto fail6;
	}

	mmc->ops = &vsnv3_mci_ops;
	mmc->ocr_avail = 
#if 0	
	mmc->ocr_avail = MMC_VDD_28_29 |/* VDD voltage 2.8 ~ 2.9 */
					MMC_VDD_29_30 |/* VDD voltage 2.9 ~ 3.0 */
					MMC_VDD_30_31 |/* VDD voltage 3.0 ~ 3.1 */
					MMC_VDD_31_32 |/* VDD voltage 3.1 ~ 3.2 */
#endif					
					MMC_VDD_32_33 |/* VDD voltage 3.2 ~ 3.3 */
					MMC_VDD_33_34; /* VDD voltage 3.3 ~ 3.4 */
	mmc->caps = MMC_CAP_MMC_HIGHSPEED;

	mmc->max_blk_size  = MCI_MAXBLKSIZE;
	mmc->max_blk_count = MCI_BLKATONCE;
	mmc->max_req_size  = MCI_BUFSIZE;
	mmc->max_segs      = MCI_BLKATONCE;
	mmc->max_seg_size  = MCI_BUFSIZE;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->bus_mode = 0;
	host->board =(struct ait_sd_data *) pdev->dev.platform_data;

	if (host->board->wire4) {
		dev_info(&pdev->dev, "Support 4 wire bus mode ");
		mmc->caps |= MMC_CAP_4_BIT_DATA;
		
		if(!host->board->disable_sdio)
			mmc->caps |= MMC_CAP_SDIO_IRQ;	// For WIFI

		if(host->board->bus_width==8)
		{
			dev_info(&pdev->dev, "Support  8 wire bus mode ");
			mmc->caps |= MMC_CAP_8_BIT_DATA;
		}
	}

	
#ifdef DMA_BY_SRAM

	host->physical_address = (void*)0x115A00;

	host->buffer = AIT_RAM_P2V(host->physical_address );
#else
	host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE,
					&host->physical_address, GFP_KERNEL);

	if (!host->buffer) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
		goto fail5;
	}
#endif	

#ifdef RX_TASKLET
	tasklet_init(&host->rx_tasklet, vsnv3_sd_rx_tasklet, (unsigned long) host);
#endif
	/*
	 * Reserve GPIOs ... board init code makes sure these pins are set
	 * up as GPIOs with the right direction (input, except for vcc)
	 */

	if (host->board->det_pin) {
		ret = gpio_request(host->board->det_pin, "mmc_detect");
		if (ret < 0) {
			dev_dbg(&pdev->dev, "couldn't claim card detect pin\n");
			goto fail4b;
		}
	}

	if (host->board->wp_pin) {
		ret = gpio_request(host->board->wp_pin, "mmc_wp");
		if (ret < 0) {
			dev_dbg(&pdev->dev, "couldn't claim wp sense pin\n");
			goto fail4;
		}
	}

	if (host->board->vcc_pin) {
		ret = gpio_request(host->board->vcc_pin, "mmc_vcc");
		if (ret < 0) {
			dev_dbg(&pdev->dev, "couldn't claim vcc switch pin\n");
			goto fail3;
		}
	}

	/*
	 * Get Clock
	 */
	 if(host->board->controller_id<3)
	{
			static char tmpstr[sizeof("sd1_ctl_clk")];

			sprintf(tmpstr, "sd%d_ctl_clk",host->board->controller_id);
			
			host->mci_clk = clk_get(&pdev->dev, tmpstr);				
	}
	else
		return -ENODEV;

	if (IS_ERR(host->mci_clk)) {
		ret = -ENODEV;
		dev_warn(&pdev->dev, "no mmc_clk?\n");
		goto fail2;
	}

	mmc->f_min = clk_get_rate(host->mci_clk)/(2*(1023+1)-1);
	
	if(host->board->max_clk_rate)
		mmc->f_max = 	host->board->max_clk_rate;
	else
		mmc->f_max = 25000000;

	if (!request_mem_region(res->start, resource_size(res), pdev->name))
		return -EBUSY;
	
	/*
	 * Map I/O region
	 */
	host->baseaddr = ioremap(res->start, resource_size(res));
	if (!host->baseaddr) {
		ret = -ENOMEM;
		goto fail1;
	}


	/*
	 * Reset hardware
	 */
	clk_enable(host->mci_clk);		/* Enable the peripheral clock */
	vsnv3_sd_disable(host);
	vsnv3_sd_enable(host);

	/*
	 * Allocate the MCI interrupt
	 */
	host->irq = platform_get_irq(pdev, 0);
	ret = request_irq(host->irq, vsnv3_sd_irq, IRQF_SHARED,
			mmc_hostname(mmc), host);
	if (ret) {
		dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
		goto fail0;
	}

	setup_timer(&host->timer, vsnv3_sd_timeout_timer, (unsigned long)host);

	platform_set_drvdata(pdev, mmc);

	/*
	 * Add host to MMC layer
	 */
	if (host->board->det_pin) {
		//host->present = !gpio_get_value(host->board->det_pin);
		host->present = gpio_get_value(host->board->det_pin)^host->board->active_low;
	}
	else
		host->present = -1;


	host->mmc->pm_caps =MMC_PM_KEEP_POWER ;

	mmc_add_host(mmc);

	/*
	 * monitor card insertion/removal if we can
	 */
	if (host->board->det_pin) {
		ait_set_gpio_input(host->board->det_pin,1);
		irq=gpio_to_irq(host->board->det_pin);
	        irq_set_irq_type(irq, /*IRQ_TYPE_EDGE_FALLING|*/IRQ_TYPE_EDGE_RISING|IRQ_TYPE_EDGE_FALLING);
		ret = request_irq(irq, vsnv3_sd_det_irq, 0, mmc_hostname(mmc), host);
		if (ret)
			dev_warn(&pdev->dev, "request MMC detect irq failed\n");
		else
			device_init_wakeup(&pdev->dev, 1);
	}

	pr_debug("Added SD driver: %s\n",pdev->name);

	return 0;

fail0:
	clk_disable(host->mci_clk);
	iounmap(host->baseaddr);
fail1:
	clk_put(host->mci_clk);
fail2:
	if (host->board->vcc_pin)
		gpio_free(host->board->vcc_pin);
fail3:
	if (host->board->wp_pin)
		gpio_free(host->board->wp_pin);
fail4:
	if (host->board->det_pin)
		gpio_free(host->board->det_pin);
fail4b:
#ifndef DMA_BY_SRAM	
	if (host->buffer)
		dma_free_coherent(&pdev->dev, MCI_BUFSIZE,
				host->buffer, host->physical_address);
#endif	
fail5:
	mmc_free_host(mmc);
fail6:
	release_mem_region(res->start, resource_size(res));
	dev_err(&pdev->dev, "probe failed, err %d\n", ret);
	return ret;
}

/*
 * Remove a device
 */
static int __exit vsnv3_sd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct vsnv3sd_host *host;
	struct resource *res;

	printk(KERN_INFO 	"MMC: %s\n",__FUNCTION__);

	if (!mmc)
		return -1;

	host = mmc_priv(mmc);
#ifndef DMA_BY_SRAM
	if (host->buffer)
		dma_free_coherent(&pdev->dev, MCI_BUFSIZE,
				host->buffer, host->physical_address);
#endif
	if (host->board->det_pin) {
		if (device_can_wakeup(&pdev->dev))
			free_irq(gpio_to_irq(host->board->det_pin), host);
		device_init_wakeup(&pdev->dev, 0);
		gpio_free(host->board->det_pin);
	}

	vsnv3_sd_disable(host);
	del_timer_sync(&host->timer);
	mmc_remove_host(mmc);
	free_irq(host->irq, host);

	clk_disable(host->mci_clk);			/* Disable the peripheral clock */
	clk_put(host->mci_clk);

	if (host->board->vcc_pin)
		gpio_free(host->board->vcc_pin);
	if (host->board->wp_pin)
		gpio_free(host->board->wp_pin);

	iounmap(host->baseaddr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);
	pr_debug("MCI Removed\n");

	return 0;
}

#ifdef CONFIG_PM
static int vsnv3_sd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct vsnv3sd_host *host = mmc_priv(mmc);
	int ret = 0;


	if (host->board->det_pin && device_may_wakeup(&pdev->dev))
		enable_irq_wake(host->board->det_pin);
	clk_disable(host->mci_clk);
	if (mmc)
		ret = mmc_suspend_host(mmc);

	return ret;
}

static int vsnv3_sd_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct vsnv3sd_host *host = mmc_priv(mmc);
	int ret = 0;
	clk_enable(host->mci_clk);

	if (host->board->det_pin && device_may_wakeup(&pdev->dev))
		disable_irq_wake(host->board->det_pin);

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define vsnv3_sd_suspend	NULL
#define vsnv3_sd_resume		NULL
#endif

static struct platform_driver vsnv3_sd_driver[] = {
#ifdef CONFIG_MMC_AIT_IF0
{
	.remove		= __exit_p(vsnv3_sd_remove),
	.suspend	= vsnv3_sd_suspend,
	.resume		= vsnv3_sd_resume,
	.driver		= {
		.name	= DEVICE_NAME_SD0,
		.owner	= THIS_MODULE,
	},
},
#endif
#ifdef CONFIG_MMC_AIT_IF1
{
	.remove		= __exit_p(vsnv3_sd_remove),
	.suspend	= vsnv3_sd_suspend,
	.resume		= vsnv3_sd_resume,
	.driver		= {
		.name	= DEVICE_NAME_SD1,
		.owner	= THIS_MODULE,
	},
},
#endif
#ifdef CONFIG_MMC_AIT_IF2
{
	.remove		= __exit_p(vsnv3_sd_remove),
	.suspend	= vsnv3_sd_suspend,
	.resume		= vsnv3_sd_resume,
	.driver		= {
		.name	= DEVICE_NAME_SD2,
		.owner	= THIS_MODULE,
	},
}
#endif
};


static int __init vsnv3_sd_init(void)
{
	int i;
	int err = -EINVAL;
	
	for(i=0;i<ARRAY_SIZE(vsnv3_sd_driver);++i)
	{
		err = platform_driver_probe(&vsnv3_sd_driver[i] , vsnv3_sd_probe);
		if(err)
		{
			printk(KERN_ERR"platform_driver_probe fail!\n");
			//return err;
		}
	}
	
	return err;
}

static void __exit vsnv3_sd_exit(void)
{
	int i;
	for(i=0;i<ARRAY_SIZE(vsnv3_sd_driver);++i)
	{
		platform_driver_unregister(&vsnv3_sd_driver[i] );
	}
}

module_init(vsnv3_sd_init);
module_exit(vsnv3_sd_exit);

MODULE_DESCRIPTION("AIT Multimedia Card Interface driver");
MODULE_AUTHOR("Vincent");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:AIT MMC");
