/* MT7687 SPI IOT Linux driver */

/* 
 * Copyright (c) 2017 MSTAR Corporation
 * Authur: Vincent Chen
*/
 
#include <linux/platform_device.h>

#include "../../iot_api.h"
#include "../../iot_ether.h"
#include "../../iot_ctrl.h"

#include "mt7682_spi.h"

#define MAX_SPI_CMD_LEN SPIS_CFG_RD_CMD_LEN
struct iot_data *pMT7682_data ;
	
static u8 read_cmd_buf[MAX_SPI_CMD_LEN] = {0};

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

	status = spi_sync(spi, &message);
	if(status != 0)
	{
		printk("ERROR: %s status = 0x%x\n", __func__, status);
		return -EIO;
	}

	return 0;
}

static u16 mt7682_cmd_config_type(struct iot_data *mt_spi)
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

	return (u16)spi_sync(mt_spi->spi, &m);

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
	u8* tx_buf = read_cmd_buf;

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
 

static inline u16 mt7682_cmd_read_status(struct iot_data *mt_spi, u8* val)
{

	u8* tx_buf = read_cmd_buf;
	int ret;
	tx_buf[0] = SPIS_RS_CMD;
	
	ret = mt7682_cmd_write_read(mt_spi->spi, tx_buf, SPIS_RS_CMD_LEN, val, 1);

	if (ret)
		printk (": %s() failed: ret = %d\n", __func__, ret);

	return ret;
}

static u16 mt7682_cmd_clear_status(struct iot_data *mt_spi)
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

	 u8 tx_buf[SPIS_CFG_WR_CMD_LEN] = {SPIS_CFG_WR_CMD,
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
		 .len		 = SPIS_CFG_WR_CMD_LEN,
	 };	 

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(spi, &m);

}
 
static inline u16 mt7682_spi_cmd_config_read(struct iot_data *mt_spi, u32 data_type, u32 length)
{

	int ret;
	int data_len = length-1;

	u8 tx_buf[SPIS_CFG_RD_CMD_LEN] = {SPIS_CFG_RD_CMD,
		(data_type & 0xff),
		(data_type >> 8) & 0xff,
		(data_type >> 16) & 0xff,
		(data_type >> 24) & 0xff,
		(data_len&0xff),
		(data_len >> 8) & 0xff,
		(data_len >> 16) & 0xff,
		(data_len >> 24) & 0xff};


	ret = spi_write(mt_spi->spi, tx_buf, SPIS_CFG_RD_CMD_LEN);
	if (ret < 0) {
		dev_err(&mt_spi->spi->dev, "SPI write word error\n");
		return ret;
	}

	return ret;

}
 


static inline u16 mt7682_cmd_write_data(struct iot_data *mt_spi, void* buf, u32 length)
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

static inline u16 mt7682_cmd_write_eth_pkt(struct iot_data *mt_spi, void* buf, u32 length)
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

static u16 mt7682_spi_cmd_read_data(struct iot_data *mt_spi, void* buf, u32 length)
{

	int ret;

	u8* tx_buf = read_cmd_buf;
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
			break;
		}

		if (time_after(jiffies, t + HZ)) {
			pr_info("RD spi read status timeout.\n");
			return -1;
		}
	}		

	t2 = jiffies;
	if((t2-t)>1)
		pr_info("RD spend %d jiffies\r\n",(int)(t2-t));		

	return 0;
	
}

static inline int mt7682_write_data ( u32 data_type, void *data, int len)
{
	unsigned long t,t2;
	u8 val;
	int ret;
	struct iot_data *mt_spi = pMT7682_data;
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
	
	printk(KERN_INFO "mtk_iot_set_wifi\n");

	if((ret = mt7682_write_data( SPIS_CMD_SET_WIFI, wifisetting,sizeof(iot_wifi_setting_t)))<0)
		pr_err("mtk_iot_set_wifi error %d\n", ret);


	return 0;
}

static int mt7682_get_mac(int eth_if, char* mac)
{

	if(eth_if)
	{
		mt7682_read_data( SPIS_CMD_GET_MAC_ETH1,  mac, 6);
	}
	else
		mt7682_read_data( SPIS_CMD_GET_MAC,  mac, 6);
	
	return 0;
}

static int mt7682_init(void* priv_data)
{
	unsigned char val;

	struct iot_data *mt_spi = (struct iot_data *)priv_data;

	pMT7682_data = mt_spi;
		
	while(1)
	{
				
		mt7682_cmd_power_on();
	
		mt7682_cmd_read_status(mt_spi,&val);
		printk (KERN_ERR "MT7682 SPI Slave Init Status: %x\r\n",val);
		if((val&1))
			break;
	}


	mt7682_cmd_config_type(mt_spi);

	return 0;
}




static struct sk_buff *
mt7682_fill_skb_header(struct net_device *ndev, struct sk_buff_head *q)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	struct sk_buff *skb, *tx_skb;
	struct iot_pkt_head_t *info;
	int headroom;
	int tailroom;
	u8 need_pages;
	u16 tol_len, pkt_len;
	u8 padlen;
	static u16 seq_num;
	 
	static unsigned long pre_sent_size = 0;
	static unsigned long  sent_data_size = 0;
	static unsigned long sent_num_pkts = 0;
	static unsigned long t=0;


	if (skb_queue_empty (q)) {
		return NULL;
	}

	skb = skb_peek (q);
	pkt_len = skb->len;

	sent_data_size += pkt_len;
	sent_num_pkts++;
	if(t==0 ||time_after(jiffies, t+20*HZ))
	{
		t = jiffies;
		pr_info("Sent %d packets, %d Bytes %d bps\r\n",(int)sent_num_pkts, (int)sent_data_size,(int)(((sent_data_size-pre_sent_size)*8)/30));
		pre_sent_size = sent_data_size;
	}
	
	need_pages = (pkt_len + TX_OVERHEAD + 127) >> 7;

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);
	
	padlen = ((pkt_len + 3 ) & 0x7FC) - pkt_len;
	tol_len = ((pkt_len + 3 ) & 0x7FC) + 
			TX_HEAD_SIZE + SPIM_HEADER_SIZE;
	seq_num++;//= ++host->seq_num;// & 0x1F;

	info = (struct iot_pkt_head_t *) skb->cb;
	info->cmd = SPIS_WR_CMD;
	info->pkt_len = pkt_len;

	if ((!skb_cloned(skb)) &&
	    (headroom >= (TX_HEAD_SIZE)) &&
	    (tailroom >= (padlen))) {

		info->seq_num = seq_num;
		memcpy (skb_push (skb, TX_HEAD_SIZE), (void*)info, TX_HEAD_SIZE);

		/* Make 32-bit aligment */
		skb_put (skb, padlen);
		tx_skb = skb;
		skb_unlink(skb, q);

	} else {

		tx_skb = alloc_skb (tol_len, GFP_KERNEL);
		if (!tx_skb)
			return NULL;

		info->seq_num = seq_num;

		memcpy (skb_push (skb, TX_HEAD_SIZE), (void*)info, TX_HEAD_SIZE);

		memcpy (skb_put (tx_skb, tol_len),
				skb->data, tol_len);

		skb_unlink (skb, q);
		dev_kfree_skb (skb);
	}

	return tx_skb;
}


struct iot_dev_ops mt7682_iot_ops ={

	.init = mt7682_init,
	.get_mac = mt7682_get_mac,
	.read = mt7682_read_data,
	.write = mt7682_write_data,
	.eth_tx = mt7682_write_data,
	.set_wifi_config = mt7682_set_wifi_config,
	.sleep = mt7682_cmd_power_off,
	.fill_skb_header = mt7682_fill_skb_header,
};



static __init int mt7682_spi_drv_init(void)
{

	register_iot_dev(&mt7682_iot_ops);
	return 0;
}

__initcall(mt7682_spi_drv_init);

