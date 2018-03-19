
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

#include "mt7687_main.h"
#include "iot_api.h"

static struct iot_dev_ops *pIoT_dev_ops;

static int iot_write_eth_data (struct mt7687_data *mt_spi, void *data, int len)
{
	int ret;//,i;
//	u32* p = data;
//pr_err("tx: 0x%02x 0x%02x  0x%02x 0x%02x  len = %d\r\n",p[0],p[1],p[2],p[3], len);

//do{
//	ret = iot_write_data(mt_spi,SPIS_CMD_ETH_PKT_TX, data, len);
	ret = pIoT_dev_ops->eth_tx(SPIS_CMD_ETH_PKT_TX, data, len);
//}while(ret==-1);
	//udelay(20);	

	return ret;
	
}
#if 0
int mt7687_send_info (struct mt7687_data *mt_spi, int subcmd, void *data, int len)
{
	return pIoT_dev_ops->write(subcmd, data, len);
}


int mt7687_read_info(struct mt7687_data *mt_spi, void *data, int len)
{
	return 0;	
}
#endif


int mt7687_send_event(struct mt7687_data *mt_spi, u32 cmd)
{
	return 0;
#if 0
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
#endif	
}


int iot_read_data (struct mt7687_data *mt_spi, u16 type, void *data, int len)
{
	return pIoT_dev_ops->read( type,  data, len);
}
#if 0
int mt7687_read_spis_info_addr(struct mt7687_data *mt_spi, u32 *addr )
{
//	int i=0;
//	u16 val;
//	u16 flag;
//	int timeout;
//	struct device* dev = &mt_spi->spi->dev;
	return 0;
#if 0
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
#endif	
}
#endif

int iot_get_mac(struct mt7687_data *mt_spi, char *addr )
{
	return pIoT_dev_ops->get_mac(addr);
}

int mt7687_read_linkstatus(struct mt7687_data *mt_spi, u32 *link )
{
//	char spisbuf[ALIGN4((4+SPIS_INFO_LEN))];
//	hal_spi_slave_data_info_t* pInfo = (hal_spi_slave_data_info_t* )spisbuf;
	return 0;
}

int mt7687_iot_get_info(struct mt7687_data *mt_spi, mt7687_dev_info_t *info )
{
	char spisbuf[ALIGN4((sizeof(mt7687_dev_info_t)+SPIS_INFO_LEN))];
	hal_spi_slave_data_info_t* pInfo = (hal_spi_slave_data_info_t* )spisbuf;
	return 0;

	mt7687_send_event(mt_spi, SPIS_CMD_GET_INFO);	
//	mt7687_read_info(mt_spi, (void*)pInfo,sizeof(mt7687_dev_info_t)+SPIS_INFO_LEN);
	
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


int iot_set_wifi(struct mt7687_data *mt_spi, iot_wifi_setting_t *wifisetting)
{

	return pIoT_dev_ops->set_wifi_config(wifisetting);
}

int iot_set_power(struct mt7687_data *mt_spi, u32 mode )
{
	printk(KERN_INFO "iot_set_power %d.\n",mode);	
	return  pIoT_dev_ops->sleep();
}


int iot_packet_send(struct mt7687_data *mt_spi, struct sk_buff *tx_skb )
{
	int ret;

	while((ret = iot_write_eth_data(mt_spi, tx_skb->data, tx_skb->len)))
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


void iot_init(void* priv_data)
{

	if(pIoT_dev_ops)
		pIoT_dev_ops->init(priv_data);
	else
		pr_err("IoT device driver ops not register.");
	return;
}

int register_iot_dev(struct iot_dev_ops* ops)
{
	pIoT_dev_ops = ops;
	return 0;
}

