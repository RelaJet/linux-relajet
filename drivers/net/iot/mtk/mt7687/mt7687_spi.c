
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
#include <linux/skbuff.h>

#include <asm/io.h>
#include <mach/board.h>
#include <asm/gpio.h>
#include <mach/cpu.h>


#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/dma-mapping.h>
#include "asm/io.h"
#include "mt7687_spi.h"
#include "mach/mmp_reg_spi.h"
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_spi.h>
#include <linux/mm.h>

#include "iot.h"


static u8 read_cmd_buf[4] = {MT7687_SPI_READ, 0xFF, 0xFF, 0xFF};
static u8 write_cmd_buf[4] = {MT7687_SPI_WRITE, 0xFF, 0xFF, 0xFF};

 int ait_spi_xfer_fifo(struct spi_device *spidev, struct spi_transfer* xfer);

static inline int
ait_spi_write(struct spi_device *spi, const void *txbuf, size_t txlen){
	struct spi_transfer	t[2] = {
			{
				.tx_buf		= txbuf,
				.len		= txlen,
			},
			{
				.rx_buf		= 0,
				.len		= 0,
			}
		};
	return ait_spi_xfer_fifo(spi, t);
}

 u16 inline mt7687_read_reg (struct mt7687_data *mt_spi, u8 reg, u16* val)
{
	u8* tx_buf = read_cmd_buf;
	int ret;

	tx_buf[1] = reg;	// register address
	tx_buf[2] = 0x00;	// dumy cycle
	tx_buf[3] = 0x00;	// dumy cycle

	ret = spi_write_then_read (mt_spi->spi, tx_buf, 2, (u8 *)val, 2);
	
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	else
		__swab16s(val);//le16_to_cpus (val);
	
	return ret;
}

 u16 inline mt7687_write_reg (struct mt7687_data *mt_spi, u16 reg, u16 value)
{
	u8* tx_buf = write_cmd_buf;
	int ret;

	tx_buf[1] = reg;			// register address
	tx_buf[2] = value >> 8;
	tx_buf[3] = value;
	
	ret = ait_spi_write(mt_spi->spi, tx_buf, 4);
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	return ret;
}

 u16 inline mt7687_bus_access_read (struct mt7687_data *mt_spi)
{
	u8* tx_buf = write_cmd_buf;
	int ret;

	tx_buf[1] = SPIS_REG_BUS_ACCESS;			// register address
	tx_buf[2] = 0;
	tx_buf[3] = SPIS_BUS_ACCESS_WORD |SPIS_BUS_ACCESS_READ;

	ret = ait_spi_write(mt_spi->spi, tx_buf, 4);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	
	return ret;
}

 u16 inline mt7687_bus_access_write (struct mt7687_data *mt_spi)
{
	u8* tx_buf = write_cmd_buf;
	int ret;

	tx_buf[1] = SPIS_REG_BUS_ACCESS;			// register address
	tx_buf[2] = 0;
	tx_buf[3] = SPIS_BUS_ACCESS_WORD |SPIS_BUS_ACCESS_WRITE;

	ret = ait_spi_write(mt_spi->spi, tx_buf, 4);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	
	return ret;
}

int inline  mt7687_write_address (struct mt7687_data *mt_spi, u32 addr )
{
	int ret;
	ret = mt7687_write_reg(mt_spi, SPIS_REG_ADDR, addr );	// put bus address [15:0] into SPI slave reg02[15:0]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	ret = mt7687_write_reg(mt_spi, SPIS_REG_ADDR+2, addr>>16 );	// put bus address [31:16] into SPI slave reg02[31:16]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	return ret;
}

int inline  mt7687_read_address (struct mt7687_data *mt_spi, u32* data )
{
	int ret;
	u16 tmp;
	ret = mt7687_read_reg(mt_spi, SPIS_REG_ADDR, &tmp );	// put bus address [15:0] into SPI slave reg02[15:0]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	*data = tmp&0xffff;
	
	ret = mt7687_read_reg(mt_spi, SPIS_REG_ADDR+2, &tmp );	// put bus address [31:16] into SPI slave reg02[31:16]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	*data |= tmp<<16;	

	return ret;


}


 int inline mt7687_write_fifo (struct mt7687_data *mt_spi, u32 data )
{
	int ret;
	
	ret = mt7687_write_reg(mt_spi, SPIS_REG_DATA_WR, data );	// put bus address [15:0] into SPI slave reg02[15:0]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	
	ret = mt7687_write_reg(mt_spi, SPIS_REG_DATA_WR+2, data>>16 );	// put bus address [31:16] into SPI slave reg02[31:16]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	
	return ret;
}
#define mt7687_write_cmd mt7687_write_fifo

int inline mt7687_read_fifo(struct mt7687_data *mt_spi, u32* data )
{
	int ret;
	u16 tmp;
	ret = mt7687_read_reg(mt_spi, SPIS_REG_DATA_RD, &tmp );	// put bus address [15:0] into SPI slave reg02[15:0]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	*data = tmp&0xffff;
	
	ret = mt7687_read_reg(mt_spi, SPIS_REG_DATA_RD+2, &tmp );	// put bus address [31:16] into SPI slave reg02[31:16]
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	*data |= tmp<<16;	

	return ret;
}


u16  inline mt7687_set_d2h_flag(struct mt7687_data *mt_spi, u16 flag)
{
	return mt7687_write_reg(mt_spi, SPIS_REG_MAILBOX_D2H, flag& 0x3f);
}

u16  inline mt7687_clear_d2h_flag(struct mt7687_data *mt_spi, u16 flag)
{
	return mt7687_write_reg(mt_spi, SPIS_REG_MAILBOX_D2H, flag& 0x3f);
}

u16 inline mt7687_get_d2h_flag(struct mt7687_data *mt_spi, u16* flag)
{

	return mt7687_read_reg(mt_spi, SPIS_REG_MAILBOX_D2H, flag );	// put bus address [15:0] into SPI slave reg02[15:0]
}

u16  inline mt7687_set_h2d_flag(struct mt7687_data *mt_spi, u16 flag)
{
	return mt7687_write_reg(mt_spi, SPIS_REG_MAILBOX_H2D, flag& 0x3f);
}

u16  inline mt7687_clear_h2d_flag(struct mt7687_data *mt_spi, u16 flag)
{
	return mt7687_write_reg(mt_spi, SPIS_REG_MAILBOX_H2D, flag& 0x3f);
}

u16  inline mt7687_get_h2d_flag(struct mt7687_data *mt_spi, u16* flag)
{
	return mt7687_read_reg(mt_spi, SPIS_REG_MAILBOX_H2D, flag);
}
	
u16  inline mt7687_activate_swirq (struct mt7687_data *mt_spi)
{
	u8* tx_buf = write_cmd_buf;
	int ret;
		
	tx_buf[1] = SPIS_REG_SWIRQ;			// register address
	tx_buf[2] = 0;
	tx_buf[3] = SPIS_SWIRQ_ACTIVE;

	ret = ait_spi_write(mt_spi->spi, tx_buf, 4);
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
	
	return ret;
}


 u16  mt7687_read_status (struct mt7687_data *mt_spi, u16* status)
{
	int ret;

	ret = mt7687_read_reg (mt_spi, SPIS_REG_STATUS, status);
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);


	return ret;
}



void mt7687_dump_regs (struct mt7687_data *mt_spi)
{
	u8 i;
	u16 val;
	printk(KERN_INFO "MT7687 SPI Slave Register:\n");
	for (i = 0; i < 0x10; ++i) {
		mt7687_read_reg (mt_spi, i * 2 ,&val);
			
		printk (KERN_INFO"Addr[%02x]: 0x%04x  ",i*2,val);
	}
	printk (KERN_INFO"\n");

}

int mt7687_send_data (struct mt7687_data *mt_spi, int id, void *data, int len)
{
	int ret,i;
	u32* p = data;
	u16 words = (len+3)>>2;
#ifdef IOT_SPI_DEBUG

	u32 readback_addr;
#endif

	mt7687_set_h2d_flag (mt_spi,SPIS_H2D_TYPE_DATA);

	ret = mt7687_write_address(mt_spi, mt_spi->wr_addr[id]);
	if(ret<0)
	{
		BUG_ON(1);
		return ret;
	}
#ifdef IOT_SPI_DEBUG
	mt7687_read_address(mt_spi, &readback_addr );
	if(readback_addr!=mt_spi->wr_addr[id])
	{
		pr_err("%s: Target Address Setting error. %x => %x\n",mt_spi->wr_addr[id], readback_addr);

	}
#endif		
	for(i=0; i<words; i++)
	{
		ret = mt7687_write_fifo(mt_spi, *(p+i) );
		if(ret<0)
		{
			BUG_ON(1);
			return ret;
		}

		mt7687_bus_access_write(mt_spi);
	}
#ifdef IOT_SPI_DEBUG
	mt7687_read_address(mt_spi, &readback_addr );
	if(readback_addr!=(mt_spi->wr_addr[id]+words*4))
	{
		pr_err("%s: Target Address Setting error after werite. %x => %x\n",mt_spi->wr_addr[id]+words*4, readback_addr);

	}
	if((words*4)>1528)
	{

		pr_info("mt_spi->wr_addr[%d]= 0x%08x\n",id, mt_spi->wr_addr[id]);	
		pr_info("words %d addr 0x%08x\n",words*4, readback_addr);
	}
#endif	
	return ret;
	
}

int mt7687_send_info (struct mt7687_data *mt_spi, int subcmd, void *data, int len)
{
	int ret,i;
	u32* p = data;
	u16 words = (len+3)>>2;
	int timeout;
	u16 flag_h2d;
	unsigned long t, t2;

	mt7687_set_h2d_flag(mt_spi,SPIS_H2D_TYPE_EVENT);
	
	ret = mt7687_write_address(mt_spi, mt_spi->info_addr);
	if(ret<0)
	{
		BUG_ON(1);
		return ret;
	}

	for(i=0; i<words; i++)
	{
		ret = mt7687_write_fifo(mt_spi, *(p+i) );
		if(ret<0)
		{
			BUG_ON(1);
			return ret;
		}

		mt7687_bus_access_write(mt_spi);
	}

	ret = mt7687_write_cmd(mt_spi,subcmd);
	if(ret<0)
	{
		pr_info("cmd:%d mt7687_send_event error = 0x%x\n",SPIS_H2D_TYPE_EVENT, ret);
		return ret;
	}
	
	mt7687_activate_swirq (mt_spi);

	timeout = 0;

	t=jiffies;
	
	while(((ret = mt7687_get_h2d_flag (mt_spi,&flag_h2d))==0) )
	{
		if(!(flag_h2d & SPIS_H2D_TYPE_EVENT))
			break;

		schedule_timeout_interruptible(HZ/100);
		timeout++;		
		if (time_after(jiffies, t + 1 * HZ)) {
			printk(KERN_ERR "%s: mt7687_get_h2d_flag timeout.\n", __func__);
			timeout = 1;
			break ; 
		}
		
	}

	t2=jiffies;
	
	if((t2-t)>100)
	{
		pr_err("mt7687_send_info ack timeout too long. Scheduling = %d jiffies = %d ret = %d\n", timeout,(int)(t2-t), ret);
	
		return -ETIME;
	}

	return ret;
	
}


int mt7687_read_info(struct mt7687_data *mt_spi, void *data, int len)
{

	int ret,i;
	u16 status;
	u32* p = data;
	u16 words = (len+3)>>2;	// word = 4 bytes
	int timeout;
	u16 flag_d2h;

	timeout = 100;
	while((mt7687_get_d2h_flag (mt_spi,&flag_d2h)==0) && timeout--)
	{
		if(flag_d2h & SPIS_D2H_TYPE_DATA_READY)
			break;
		msleep(1);		
	}

	mt7687_read_status(mt_spi,&status);
	if(status)
	{
		printk (KERN_ERR "mt7687_read_info: mt7687_read_status error\n");
		return -EBUSY;
	}

	ret = mt7687_write_address(mt_spi, mt_spi->info_addr);
	
	if(ret<0)
	{
		BUG_ON(1);
		return ret;
	}
	
	for(i=0; i<words; i++)
	{
		mt7687_bus_access_read(mt_spi);

		mt7687_read_status(mt_spi,&status);
		if(status)
			BUG_ON(1);

		ret = mt7687_read_fifo(mt_spi, (p+i) );
		if(ret<0)
		{
			BUG_ON(1);
			return ret;
		}
	}

	{
		int timeout;
		u16 flag_h2d;
		mt7687_clear_d2h_flag(mt_spi, SPIS_D2H_TYPE_DATA_READY);
		mt7687_set_h2d_flag(mt_spi,SPIS_H2D_TYPE_RD_DATA_DONE);
		mt7687_activate_swirq (mt_spi);
		timeout = 100;
		while((mt7687_get_h2d_flag (mt_spi,&flag_h2d)==0) && --timeout)
		{
			if(!(flag_h2d &SPIS_H2D_TYPE_RD_DATA_DONE))
				break;		
			msleep(1);
		}
		if(!timeout)
		{
			printk (KERN_ERR "mt7687_read_info: H2D SPIS_H2D_TYPE_RD_DATA_DONE timeout.\n");
			return -ETIME;
		}
	}	

	return ret;
	
}



int mt7687_send_event(struct mt7687_data *mt_spi, u32 cmd)
{
	int ret;
	u16 status;
	int timeout;
	u16 flag_h2d;
	

	mt7687_read_status(mt_spi,&status);
	if(status)
		pr_info("mt7687_send_event: cmd%d , mt7687 status error = 0x%x\n",cmd, status);
		
	mt7687_set_h2d_flag(mt_spi,SPIS_H2D_TYPE_EVENT);
	
	ret = mt7687_write_cmd(mt_spi,cmd);
	if(ret<0)
	{
		pr_info("cmd:%d mt7687_send_event error = 0x%x\n",cmd, ret);
		return ret;
	}

#if 0
	for(i=0; i<words; i++)
	{
		ret = mt7687_write_fifo(mt_spi, *(p+i) );
		if(ret<0)
		{
			BUG_ON(1);
			return ret;
		}

		mt7687_bus_access_write(mt_spi);
	}
#endif
	mt7687_activate_swirq (mt_spi);

	timeout = 100;
	while((mt7687_get_h2d_flag (mt_spi,&flag_h2d)==0) && --timeout)
	{
		if(!(flag_h2d & SPIS_H2D_TYPE_EVENT))
			break;
		msleep(1);
	}
	if(timeout==0)
	{
		pr_err("cmd:%d mt7687_send_event ack timeout = 0x%x\n",cmd, ret);
	
		return -ETIME;
	}
	
	return ret;
	
}


int mt7687_read_data (struct mt7687_data *mt_spi, u16 type, void *data, int len, int finished)
{

	int ret,i;
	u16 status;
	u32* p = data;
	u16 words = (len+3)>>2;	// word = 4 bytes
	int timeout;
	u16 flag_d2h;

	mt7687_read_status(mt_spi,&status);
	if(status)
	{
		printk (KERN_ERR "mt7687_read_data: mt7687_read_status error\n");
		return -EBUSY;
	}
	
	timeout = 100;
	while((mt7687_get_d2h_flag (mt_spi,&flag_d2h)==0) && --timeout)
	{
		if(flag_d2h & SPIS_D2H_TYPE_DATA_READY)
			break;
		mdelay(1);
	}
	if(timeout==0)
	{
		pr_err("mt7687_read_data: mt7687_get_d2h_flag timeout.\n");
		return -ETIME;
	}

	if(type == DATA_TYPE_RD_ADDR || type==DATA_TYPE_WR_ADDR || type==DATA_TYPE_MAC)
 		ret = mt7687_write_address(mt_spi, mt_spi->info_addr);
	else
		ret = mt7687_write_address(mt_spi, mt_spi->rd_addr);
	
	if(ret<0)
	{
		BUG_ON(1);
		return ret;
	}
	for(i=0; i<words; i++)
	{
		mt7687_bus_access_read(mt_spi);

		mt7687_read_status(mt_spi,&status);
		if(status)
			BUG_ON(1);

		ret = mt7687_read_fifo(mt_spi, (p+i) );
		if(ret<0)
		{
			BUG_ON(1);
			return ret;
		}
	}
	if(finished)
	{
		int timeout;
		u16 flag_h2d;
		mt7687_clear_d2h_flag(mt_spi, SPIS_D2H_TYPE_DATA_READY);
		mt7687_set_h2d_flag(mt_spi,SPIS_H2D_TYPE_RD_DATA_DONE);
		mt7687_activate_swirq (mt_spi);
		timeout = 100;
		while((mt7687_get_h2d_flag (mt_spi,&flag_h2d)==0) && --timeout)
		{
			if(!(flag_h2d &SPIS_H2D_TYPE_RD_DATA_DONE))
				break;
			msleep(1);
		}
		if(!timeout)
		{
			printk (KERN_ERR "mt7687_handle_d2h_isr: H2D SPIS_H2D_TYPE_RD_DATA_DONE timeout.\n");
			return -ETIME;
		}
		
	}	

	return ret;
	
}

int mt7687_read_spis_info_addr(struct mt7687_data *mt_spi, u32 *addr )
{
	int i=0;
	u16 val;
	u16 flag;
	int timeout;
	struct device* dev = &mt_spi->spi->dev;

	*addr = 0;
	
	mt7687_get_d2h_flag(mt_spi, &flag);
	
	if(flag&SPIS_D2H_STATUS_INFO_READY)
	{
		dev_err(dev,"Read information buffer address fail. D2H flag = 0x%02x",flag);
		return -EBUSY;
	}
	else
	{
		for(i=0;i<8;++i)
		{
			if(mt7687_send_event(mt_spi, SPIS_CMD_GET_INFO_BUF_ADDR0+i)!=0)
			{
				pr_err("mt7687_send_event error %d\n",SPIS_CMD_GET_INFO_BUF_ADDR0+i);
				mt7687_clear_d2h_flag(mt_spi, SPIS_D2H_STATUS_INFO_READY|0xf);
				return -1;
			}

			timeout = 1000;
			while(mt7687_get_d2h_flag(mt_spi, &val)==0 && --timeout!=0)
			{
				if((val&SPIS_D2H_STATUS_INFO_READY))
					break;
				pr_err("mt7687_read_spis_info_addr:  mt7687_get_d2h_flag val =0x%x \n",val);
				mdelay(1);
			}
			if(timeout==0)
			{
				pr_err("mt7687_read_spis_info_addr:  mt7687_get_d2h_flag timeout.\n");
				return -ETIME;				
			}else if(timeout<99)
			{
				pr_err("mt7687_read_spis_info_addr:  mt7687_get_d2h_flag okay. (%d)\n",timeout);
			}
			pr_debug("info addr %d: 0x%x", i, (val&0xf)<<(i*4));
			mt7687_clear_d2h_flag(mt_spi, SPIS_D2H_STATUS_INFO_READY|0xf);
			*addr |= (val&0xf)<<(i*4);	
		}
	}

	return 0;
}

int mt7687_read_spis_rd_addr(struct mt7687_data *mt_spi, u32 *addr )
{
	char spisbuf[4+SPIS_INFO_LEN];
	hal_spi_slave_data_info_t* pInfo =(hal_spi_slave_data_info_t*) spisbuf;

	mt7687_send_event(mt_spi, SPIS_CMD_GET_RD_BUF_ADDR);	

	mt7687_read_data(mt_spi,DATA_TYPE_RD_ADDR, (void*)pInfo,4+SPIS_INFO_LEN, 1);

	if(pInfo->type!=DATA_TYPE_RD_ADDR ||!(pInfo->status&1) || pInfo->len!=4)
	{
		dev_err(&mt_spi->spi->dev, " Read spis rd buffer addr error.\n");
		pr_info("Ready: %x\n",pInfo->type);
		pr_info("Status: %x\n",pInfo->status);	
		pr_info("Len    : %d\n",pInfo->len);		
		
		return -EHOSTDOWN;
	}
	else
		memcpy(addr, spisbuf+SPIS_INFO_LEN,4);
	return 0;
}

int mt7687_read_spis_wr_addr(struct mt7687_data *mt_spi, u32 *addr, int num_queue )
{
	void* spisbuf;
	hal_spi_slave_data_info_t* pInfo;
	int ret = 0;

	spisbuf = kmalloc(num_queue*sizeof(u32)+SPIS_INFO_LEN, GFP_KERNEL);
	pInfo =(hal_spi_slave_data_info_t*) spisbuf;
	mt7687_send_event(mt_spi, SPIS_CMD_GET_WR_BUF_ADDR);	

	ret = mt7687_read_data(mt_spi, DATA_TYPE_WR_ADDR, (void*)pInfo,num_queue*sizeof(u32)+SPIS_INFO_LEN, 1);
	if(ret<0)
		return ret;
	
	if(pInfo->type!=DATA_TYPE_WR_ADDR ||!(pInfo->status&1)  || pInfo->len!=num_queue*sizeof(u32) )
	{
		dev_err(&mt_spi->spi->dev, " Read spis wr buffer addr error.\n");
		pr_info("Type: %x\n",pInfo->type);
		pr_info("Status: %x\n",pInfo->status);	
		pr_info("Len    : %d\n",pInfo->len);		
		kfree(spisbuf);

		return -EHOSTDOWN;
	}
	else
		memcpy(addr, spisbuf+SPIS_INFO_LEN,num_queue*sizeof(u32));

	kfree(spisbuf);

	return 0;
}
	


int mt7687_read_spis_mac(struct mt7687_data *mt_spi, char *addr )
{
	char spisbuf[6+SPIS_INFO_LEN];
	hal_spi_slave_data_info_t* pInfo = (hal_spi_slave_data_info_t* )spisbuf;
	
	mt7687_send_event(mt_spi, SPIS_CMD_GET_MAC);	

	mt7687_read_data(mt_spi, DATA_TYPE_MAC, (void*)pInfo,6+SPIS_INFO_LEN, 1);

	if(pInfo->type!=SPIS_CMD_GET_MAC ||!(pInfo->status&1) || pInfo->len!=6)
	{

		dev_err(&mt_spi->spi->dev, " Read spis mac addr error.\n");
		pr_info("Ready: %x\n",pInfo->type);
		pr_info("Status: %x\n",pInfo->status);	
		pr_info("Len    : %d\n",pInfo->len);		
		
		return -EHOSTDOWN;
	}
	else
		memcpy(addr, spisbuf+SPIS_INFO_LEN,6);
	return 0;
}

int mt7687_read_linkstatus(struct mt7687_data *mt_spi, u32 *link )
{
	char spisbuf[ALIGN4((4+SPIS_INFO_LEN))];
	hal_spi_slave_data_info_t* pInfo = (hal_spi_slave_data_info_t* )spisbuf;
	
	mt7687_send_event(mt_spi, SPIS_CMD_GET_LINK);	

	mt7687_read_info(mt_spi, (void*)pInfo,4+SPIS_INFO_LEN);
	
	if(pInfo->type!=SPIS_CMD_GET_LINK ||!(pInfo->status&1) || pInfo->len!=4)
	{

		dev_err(&mt_spi->spi->dev, " Read spis link status error.\n");
		pr_info("Ready: %x\n",pInfo->type);
		pr_info("Status: %x\n",pInfo->status);	
		pr_info("Len    : %d\n",pInfo->len);		
		*link=0;
		return -EHOSTDOWN;
	}
	else
		memcpy(link, spisbuf+SPIS_INFO_LEN,4);
	return 0;
}

int mt7687_iot_get_info(struct mt7687_data *mt_spi, mt7687_dev_info_t *info )
{
	char spisbuf[ALIGN4((sizeof(mt7687_dev_info_t)+SPIS_INFO_LEN))];
	hal_spi_slave_data_info_t* pInfo = (hal_spi_slave_data_info_t* )spisbuf;

	mt7687_send_event(mt_spi, SPIS_CMD_GET_INFO);	
	mt7687_read_info(mt_spi, (void*)pInfo,sizeof(mt7687_dev_info_t)+SPIS_INFO_LEN);
	
	if(pInfo->type!=SPIS_CMD_GET_INFO ||!(pInfo->status&1) || pInfo->len!=sizeof(mt7687_dev_info_t))
	{

		dev_err(&mt_spi->spi->dev, " Read iot info fail.\n");
		pr_info("Ready: %x\n",pInfo->type);
		pr_info("Status: %x\n",pInfo->status);	
		pr_info("Len    : %d. lenth must be %d.\n",pInfo->len , sizeof(mt7687_dev_info_t));		
		return -EHOSTDOWN;
	}
	else
		memcpy(info, spisbuf+SPIS_INFO_LEN,sizeof(mt7687_dev_info_t));
	return 0;
}


int mt7687_set_wifi(struct mt7687_data *mt_spi, iot_wifi_setting_t *wifisetting)
{
	int ret;

	printk(KERN_INFO "mt7687_set_wifi\n");

	if((ret = mt7687_send_info(mt_spi, SPIS_CMD_SET_WIFI, wifisetting,sizeof(iot_wifi_setting_t)))<0)
		pr_err("mt7687_set_wifi error %d\n", ret);


	return 0;
}

int mt7687_iot_set_power(struct mt7687_data *mt_spi, u32 mode )
{
	int ret = 0;

	printk(KERN_INFO "mt7687_iot_set_power %d.\n",mode);

	if((ret = mt7687_send_info(mt_spi, SPIS_CMD_SET_POWERMODE, &mode,sizeof(u32)))<0)
		pr_err("mt7687_set_wifi error %d\n", ret);

	return ret;
}

int iot_set_rx_mode(struct mt7687_data *mt_spi, u16 mode)
{
	int ret;

	if((ret = mt7687_send_info(mt_spi, SPIS_CMD_SET_RX_MODE, &mode ,sizeof(u16)))<0)
		pr_err("iot_set_rx_mode %d\n", ret);

	return ret;
}

int mt7687_rx_done(struct mt7687_data *mt_spi, u16 seq)
{
	int ret;

	if((ret = mt7687_send_info(mt_spi, SPIS_CMD_SET_READ_DONE, &seq ,sizeof(u16)))<0)
		pr_err("mt7687_rx_done %d\n", ret);

	return ret;
}

int mt7687_iot_buf_send(struct mt7687_data *mt_spi, int ready_buf_mask, int* sent_time )
{
	int timeout ;
	u16 flag_d2h;
	unsigned long t1, t2;
	int ret;
	
	mt7687_set_h2d_flag (mt_spi,SPIS_H2D_TYPE_DATA);

	mt7687_write_cmd(mt_spi,ready_buf_mask);
	
	mt7687_activate_swirq (mt_spi);
	flag_d2h = 0;
	timeout = 0;

	t1=jiffies;

	while(((ret = mt7687_get_h2d_flag (mt_spi,&flag_d2h))==0) )
	{
		if(!(flag_d2h &SPIS_H2D_TYPE_DATA))
			break;		
		msleep_interruptible(10);//schedule_timeout(HZ/10);
		
		if (time_after(jiffies, t1 + 1 * HZ)) {
			printk(KERN_ERR "%s: mt7687_get_h2d_flag timeout.\n", __func__);
			timeout = 1;
			break ; 
		}
	}
	t2=jiffies;

	if(timeout==1){
		pr_warn("mt7687_send_data: device no response. avaliable_buf_mask = 0x%08x\n",ready_buf_mask);
	}		
	else
	{
		if((t2-t1)>10)
		{
			pr_info("mt7687_send_data: okay(%d,0x%08x)\n",(int)(t2-t1),ready_buf_mask);		
			*sent_time = (t2-t1);
		}
	}
	return 0;
}

int mt7687_iot_buf_prepare(struct mt7687_data *mt_spi, u32 dest_buf_id, struct sk_buff *tx_skb )
{
	int ret;
	while((ret = mt7687_send_data(mt_spi, dest_buf_id, tx_skb->data, tx_skb->len)))
	{

		if(ret==0)
			break;
		
		if(ret==-ETIME)
			continue;
		else
		{
			pr_err("mt7687_hard_xmit error %d\n",ret);
			
			break;
		}
	}
	return ret;
}
