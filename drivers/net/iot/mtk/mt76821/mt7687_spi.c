
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

#include "iot_api.h"
#include "iot.h"

//static struct spi_device *spi_host;
struct mt7687_data *pMT7682_data ;
	
static u8 read_cmd_buf[4] = {MT7687_SPI_READ, 0xFF, 0xFF, 0xFF};
//static u8 write_cmd_buf[4] = {MT7687_SPI_WRITE, 0xFF, 0xFF, 0xFF};
 static u8 cmd_buf[9] = {0};

 int ait_spi_xfer_fifo(struct spi_device *spidev, struct spi_transfer* xfer);
//#pragma GCC optimize("O0")
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

static inline int mt7682_cmd_write_read(struct spi_device *spi, const void *txbuf, unsigned n_tx, void *rxbuf, unsigned n_rx)
{
	int status;
	struct spi_message message;
	struct spi_transfer x[2];

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if(n_tx)
	{
		x[0].len = n_tx;
		x[0].tx_buf = txbuf;
		spi_message_add_tail(&x[0], &message);
	}
	if(n_rx)
	{
		x[1].len = n_rx;
		x[1].rx_buf = rxbuf;
		spi_message_add_tail(&x[1], &message);
	}

	//do spi transfer
	status = spi_sync(spi, &message);
	if(status != 0)
	{
		printk("ERROR: %s status = 0x%x\n", __func__, status);
		return -EIO;
	}

	return 0;
}

static u16 mt7682_cmd_config_type(struct mt7687_data *mt_spi)
{
	u8 tx_buf[2];

	struct spi_message m;

	struct spi_transfer t = {
		.tx_buf	 = tx_buf,
		.len		 = 2,
	};

	tx_buf[0] = 0x10;
	tx_buf[1] = 0x04;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(mt_spi->spi, &m);

}

static u16 mt7682_cmd_power_on(void)
{
	u8* tx_buf = read_cmd_buf;

	struct spi_message m;

	struct spi_transfer t = {
		.tx_buf	 = tx_buf,
		.len		 = 1,
	 };
 
	tx_buf[0] = SPIS_PWON_CMD;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(pMT7682_data->spi, &m);

}

static int mt7682_cmd_power_off(void)
{
	u8* tx_buf = cmd_buf;

	struct spi_message m;

	struct spi_transfer t = {
		.tx_buf	  = tx_buf,
		.len		  = 1,
	};

	tx_buf[0] = SPIS_PWOFF_CMD;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(pMT7682_data->spi, &m);
 
}
 

static inline u16 mt7682_cmd_read_status(struct mt7687_data *mt_spi, u8* val)
{

	u8* tx_buf = cmd_buf;
	int ret;
	tx_buf[0] = SPIS_RS_CMD;
	
	ret = mt7682_cmd_write_read(mt_spi->spi, tx_buf, 1, val, 1);
	printk ("======= spi ret = %d =======\n ", ret);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	return ret;
}

static u16 mt7682_cmd_clear_status(struct mt7687_data *mt_spi)
{
	int ret;
	static u8 tx_buf[2] = {0x08, 0xD8};

	ret = mt7682_cmd_write_read(mt_spi->spi, tx_buf, 2, NULL, 0);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	return ret;
}


static inline u16 mt7682_cmd_config_write(struct spi_device *spi, u32 data_type, u32 length)
{

	 u32 data_len = length-1;
	 struct spi_message m;

	 u8 tx_buf[9] = {SPIS_CFG_WR_CMD,
	 				(data_type & 0xff),
					(data_type >> 8) & 0xff,
					(data_type >> 16) & 0xff,
					(data_type >> 24) & 0xff,
					(data_len&0xff),
					(data_len >> 8) & 0xff,
					(data_len >> 16) & 0xff,
					(data_len >> 24) & 0xff};	 
	 struct spi_transfer t = {
		 .tx_buf	 = tx_buf,
		 .len		 = 9,
	 };	 

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(spi, &m);

}
 
static inline u16 mt7682_spi_cmd_config_read(struct mt7687_data *mt_spi, u32 data_type, u32 length)
{
// #define SPIS_ADDRESS_ID         0x55aa0000
// #define SPI_TEST_DATA_SIZE      1024
// #define SPIS_CFG_LENGTH         (SPI_TEST_DATA_SIZE - 1)

	int ret;
	int data_len = length-1;

	u8 tx_buf[9] = {SPIS_CFG_RD_CMD,
		(data_type & 0xff),
		(data_type >> 8) & 0xff,
		(data_type >> 16) & 0xff,
		(data_type >> 24) & 0xff,
		(data_len&0xff),
		(data_len >> 8) & 0xff,
		(data_len >> 16) & 0xff,
		(data_len >> 24) & 0xff};


	ret = spi_write(mt_spi->spi, tx_buf, 9);
	if (ret < 0) {
		dev_err(&mt_spi->spi->dev, "SPI write word error\n");
		return ret;
	}

	return ret;

}
 

static inline u16 mt7682_cmd_write_data(struct mt7687_data *mt_spi, void* buf, u32 length)
{
 
	int ret;
	u8* tx_buf = (u8*)buf;
	static u8 tempbuf[256] = {0x0e};

	tempbuf[0] = SPIS_WR_CMD;

	memcpy(tempbuf+1, buf, length);

	if(tempbuf[0]!=SPIS_WR_CMD)
	{
		printk("write data command ID error.\r\n");
		tx_buf[0] = SPIS_WR_CMD;		
	}
	
	ret = mt7682_cmd_write_read(mt_spi->spi, tempbuf, length+1, NULL, 0);
 
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
				  
	 return ret;
}

static inline u16 mt7682_cmd_write_eth_pkt(struct mt7687_data *mt_spi, void* buf, u32 length)
{
 
	int ret;
	u8* tx_buf = (u8*)buf;

	if(tx_buf[0]!=SPIS_WR_CMD)
	{
	
		printk("write data command ID error.\r\n");
		BUG_ON(1);
		tx_buf[0] = SPIS_WR_CMD;		
	}
	
	ret = mt7682_cmd_write_read(mt_spi->spi, tx_buf, length+1, NULL, 0);
 
	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);
				  
	 return ret;
}

static u16 mt7682_spi_cmd_read_data(struct mt7687_data *mt_spi, void* buf, u32 length)
{

	int ret;

	u8* tx_buf = cmd_buf;
	tx_buf[0] = SPIS_RD_CMD;

	ret = mt7682_cmd_write_read(mt_spi->spi, tx_buf, 1, buf, length);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	return ret;
}

static inline int mt7682_read_data (u32 data_type, void *data, int len)
 {
	 u8 val;
 	unsigned long t2, t;
	static int rd_seq=0;

	rd_seq++;

	mt7682_spi_cmd_config_read(pMT7682_data,data_type, len);

	t = jiffies;
		
	while(1)
	{
		udelay(500);	
		mt7682_cmd_read_status(pMT7682_data,&val);
	 
		if((val&0x07)==7)
			break;
		
		if (time_after(jiffies, t + HZ)) {
			pr_info("CR spi read status timeout.\n");
			return -1;
		}

	}

	t2 = jiffies;
	if((t2-t)>1)
		pr_info("CR spend %d jiffies\r\n",(int)(t2-t));
	
#if 0
	if(likely(data_type==SPIS_CMD_GET_ETH_PKT))
	{
	//	for(i=0;i < 2;++i)
		{
		 	
			mt7682_spi_cmd_read_data(mt_spi, data, len);//7682_spi_cmd_read_data(mt_spi, &info,  sizeof(hal_spi_slave_data_info_t));	

			udelay(200);				

			t = jiffies;
			while(1)
			{
				mt7682_cmd_read_status(mt_spi,&val);
				if(val&0x18)
				{
					mt7682_cmd_clear_status(mt_spi);
					printk (KERN_ERR "RD error: %x\r\n",val);		 

					return -1;
				}

				if(val&0x20)
				{
				//	printk (KERN_ERR "RD Okay: %x\r\n",val);			 
					break;// 0;
				}

				if (time_after(jiffies, t + HZ)) {
					pr_info("spi read status timeout.\n");
					return -1;
				}
				udelay(100);
			}

			pr_info("rlen = %d\n",info.len);
			
		}

		rlen = min(len, info.len);
	 	rlen+= sizeof(hal_spi_slave_data_info_t);	
	}
#endif	
	{
 
		mt7682_spi_cmd_read_data(pMT7682_data, data, len);

		udelay(50);				

		t = jiffies;
		while(1)
		{
			mt7682_cmd_read_status(pMT7682_data,&val);
			if(val&0x18)
			{
				mt7682_cmd_clear_status(pMT7682_data);
				printk (KERN_ERR "RD error: %x\r\n",val);		 

				return -1;
			}

			if(val&0x20)
			{
			//	printk (KERN_ERR "RD Okay: %x\r\n",val);		
				break;
			}

			if (time_after(jiffies, t + HZ)) {
				pr_info("RD spi read status timeout.\n");
				return -1;
			}
			//udelay(50);
		}		

		t2 = jiffies;
		if((t2-t)>1)
			pr_info("RD spend %d jiffies\r\n",(int)(t2-t));		
	}

	return 0;
	
}

static inline int mt7682_write_data ( u32 data_type, void *data, int len)
{
	unsigned long t,t2;
	u8 val;
	int ret;
	struct mt7687_data *mt_spi = pMT7682_data;
	struct spi_device *spi = (struct spi_device *)mt_spi->spi;
	static int seq = 0;
	
 //Write Data Test		 
	 mt7682_cmd_config_write(spi,((seq++)<<8)|data_type, len);

	t = jiffies;

	while(1)
	{
		mt7682_cmd_read_status(mt_spi,&val);

		if((val&0x07)==7)
			break;
		//printk (KERN_ERR "CW status: %x\r\n",val); 
		if (time_after(jiffies, t + HZ)) {
			printk (KERN_ERR"CW spi read status timeout. %d\n",val);
			return -1;
		}
	}
	t2 = jiffies;
	if((t2-t)>1)
		pr_info("CW spend %d jiffies\r\n",(int)(t2-t));

	if(likely(data_type==SPIS_CMD_ETH_PKT_TX))
		ret = mt7682_cmd_write_eth_pkt(mt_spi,data, len);
	else
		ret = mt7682_cmd_write_data(mt_spi,data, len);


	udelay(100);
	if(ret==-1)
		return -1;
	t = jiffies;

	while(1)
	{
		mt7682_cmd_read_status(mt_spi,&val);

		if(val&0x80)
		{
			mt7682_cmd_clear_status(mt_spi);
			printk (KERN_ERR "WR CMD error: %x\r\n",val); 

			return -1;
		}

		if(val&0x18)
		{
			mt7682_cmd_clear_status(mt_spi);
			printk (KERN_ERR "WR error: %x\r\n",val);		 

			return -1;
		}

		if(val&0x20)
		{
		//	printk (KERN_ERR "WR Okay: %x\r\n",val);			 
			return 0;
		}
		printk (KERN_ERR "WR Status: %x\r\n",val);			 
		 
		if (time_after(jiffies, t + HZ)) {
			pr_info("WR spi read status timeout.\n");
			return -1;
		}
	}
	
	t2 = jiffies;
	if((t2-t)>1)
		pr_info("CW spend %d ms\r\n",(int)(t2-t));

	return 0;
}

static int mt7682_set_wifi_config(iot_wifi_setting_t *wifisetting)
{
	int ret;
	
	printk(KERN_INFO "mt7687_set_wifi\n");

	if((ret = mt7682_write_data( SPIS_CMD_SET_WIFI, wifisetting,sizeof(iot_wifi_setting_t)))<0)
		pr_err("mt7687_set_wifi error %d\n", ret);


	return 0;
}

static int mt7682_get_mac(char* mac)
{
	return mt7682_read_data( SPIS_CMD_GET_MAC,  mac, 6);
}

static int mt7682_init(void* priv_data)
{
//	static uint8_t g_transfer_tx_big_buffer[1024 + 1];
//	static uint8_t g_transfer_rx_big_buffer[1024];
	int i;
	struct mt7687_data *mt_spi = (struct mt7687_data *)priv_data;

	pMT7682_data = mt_spi;

		
	for (i = 0; i < (1024 + 1); i++) {
//		g_transfer_tx_big_buffer[i] = 0x5A;
	}

	{
		unsigned char val;
		
		while(1)
		{
				
			mt7682_cmd_power_on();
	
			mt7682_cmd_read_status(mt_spi,&val);
			printk (KERN_ERR "Power on: %x\r\n", val);
			if((val&&1))
				break;
				//msleep(1000);
		}


		mt7682_cmd_config_type(mt_spi);
#if 0		
		mt7682_write_data( 0x5a/*SPIS_ADDRESS_ID*/, g_transfer_tx_big_buffer, SPI_TEST_DATA_SIZE);

//Read Data Test	

		//while(1)
		{
			int ret;		
			ret = mt7682_read_data(  0x12345678, g_transfer_rx_big_buffer, 1024);
			
			if(ret)
			{
				//continue;
			}
			else
			{
				for (i = 0; i < 1024; i++) {
					if (g_transfer_rx_big_buffer[i] != 0xA5) {
						printk(KERN_ERR"Data check failed: address: %d, value: %x\r\n", i, g_transfer_rx_big_buffer[i]);
						break;//continue;
					}
				}			
				printk(KERN_ERR" Read Test okay\r\n");

			}
		}


		//while(1)
		{
				mt7682_write_data(0x5a, g_transfer_tx_big_buffer, SPI_TEST_DATA_SIZE);
						// printk(KERN_ERR" Write Test okay\r\n");
		//		msleep(100);
		}
#endif
	}
	return 0;
}
struct iot_dev_ops mt7682_iot_ops ={

	.init = mt7682_init,
	.get_mac = mt7682_get_mac,
	.read = mt7682_read_data,
	.write = mt7682_write_data,
	.eth_tx = mt7682_write_data,
	.set_wifi_config = mt7682_set_wifi_config,
	.sleep = mt7682_cmd_power_off,
};



static __init int mt7682_spi_drv_init(void)
{

	register_iot_dev(&mt7682_iot_ops);
	return 0;
}

__initcall(mt7682_spi_drv_init);

