/* SPI IoT Ethernet Linux driver */

 /* 
 * Copyright (c) 2016 MSTAR Corporation
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
 
#include <linux/kernel.h>
#include "iot_api.h"

#include "iot_bus.h"

static struct iot_dev_ops *pIoT_dev_ops;

static int iot_write_eth_data (void *data, int len)
{
	return pIoT_dev_ops->eth_tx(SPIS_CMD_ETH_PKT_TX, data, len);
}


static int iot_read_data (u16 type, void *data, int len)
{
	return pIoT_dev_ops->read( type,  data, len);
}


int iot_get_mac(int eth_if, char *addr )
{
	return pIoT_dev_ops->get_mac(eth_if, addr);
}


int iot_get_linkstatus(u32 *link )
{
	return iot_read_data(SPIS_CMD_GET_LINK, link, (u32)sizeof(u32));
}


int iot_get_info(int eth_if, iot_dev_info_t *info )
{
	if(eth_if)
		return iot_read_data( SPIS_CMD_GET_INFO_ETH1, info, (u32)sizeof(iot_dev_info_t));
	else
		return iot_read_data( SPIS_CMD_GET_INFO, info, (u32)sizeof(iot_dev_info_t));
}


int iot_set_wifi( iot_wifi_setting_t *wifisetting)
{
	return pIoT_dev_ops->set_wifi_config(wifisetting);
}

int iot_set_power(u32 mode )
{
	printk(KERN_INFO "iot_set_power %d.\n",mode);	
	return  pIoT_dev_ops->sleep();
}


struct sk_buff *
iot_fill_skb_header(struct net_device *ndev, struct sk_buff_head *q)
{
	return pIoT_dev_ops->fill_skb_header(ndev, q);
}


int iot_packet_send( struct sk_buff *tx_skb )
{
	int ret;

	while((ret = iot_write_eth_data( tx_skb->data, tx_skb->len)))
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


int iot_read_eth_packet( void *data, int len)
{
	return iot_read_data(SPIS_CMD_GET_ETH_PKT, data, len);
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

