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
#include <linux/ip.h>
#include <linux/spi/spi.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <mach/board.h>

#include "iot_ether.h"
#include "iot_ctrl.h"

#include "iot_api.h"


#define MAX_TX_COUNT_PER_WORKLOOP 32
#define PKT_SPI_RX_SIZE 1600
static const u8 broadcast_addr[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static char version[] __devinitdata =
KERN_INFO ADP_NAME ".c:v" DRV_VERSION " " __TIME__ " " __DATE__ "\n";

static int msg_enable = NETIF_MSG_PROBE |
			NETIF_MSG_LINK |
			NETIF_MSG_TIMER |
			NETIF_MSG_RX_ERR |
			NETIF_MSG_TX_ERR |
			NETIF_MSG_TX_DONE |
			NETIF_MSG_PKTDATA |			
			NETIF_MSG_WOL;			

static struct spi_device *m_iot_spi_dev;
static void iot_eth_watchdog (PIOT_PRIVATE host)
{
	unsigned long time_to_chk = MT7687_WATCHDOG_PERIOD;

	if (time_to_chk)
		mod_timer (&host->watchdog, jiffies + time_to_chk);
}

static void iot_eth_watchdog_timer (unsigned long arg)
{
	struct net_device *ndev = (struct net_device *)(arg);
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	set_bit (EVENT_WATCHDOG, &host->flags);
		
	queue_work (host->iot_eth_work_queue, &host->iot_eth_work);
}

static void iot_eth_set_mac_addr (struct net_device *ndev)
{
	int ret;

	char* mac = ndev->dev_addr;


	if(strcmp(ndev->name, "eth1")==0)
	{
		ret = iot_get_mac(1, mac );
	}
	else
	{
		ret = iot_get_mac(0, mac );
	}
	pr_info("mtk_iot_set_mac_addr: ret=%d %02x:%02x:%02x:%02x:%02x:%02x\r\n",ret,mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );

	return;
}


static int iot_eth_set_mac_address (struct net_device *ndev, void *p)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	struct sockaddr *addr = p;
	pr_info("%s: %s",__func__, ndev->name);

	if(!strcmp(ndev->name, "eth1"))
	{
		memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);
		return 0;
	}

	
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	down (&host->spi_lock);

	iot_eth_set_mac_addr (ndev);

	up (&host->spi_lock);

	return 0;
}


static int iot_eth_hard_xmit (PIOT_PRIVATE host, u32 isTcp)
{
	struct sk_buff *tx_skb;
	int ret;
	
	if(isTcp)
		tx_skb = iot_fill_skb_header(host->ndev, &host->tx_tcp_wait_q);	
	else
		tx_skb = iot_fill_skb_header(host->ndev, &host->tx_wait_q);	

	if (!tx_skb) {

		return 0;
	}
	
	if(tx_skb->len>(1526+1))	
		pr_err("len = %d\n",tx_skb->len);

	ret = iot_packet_send(tx_skb);

	if(ret==0)
	{
		host->ndev->stats.tx_bytes += tx_skb->len;

		host->ndev->stats.tx_packets++;	
	}
	
	dev_kfree_skb (tx_skb);
	return 1;
}

static int iot_eth1_hard_xmit (PIOT_ETH1_PRIVATE eth1_priv_data, u32 isTcp)
{
	struct sk_buff *tx_skb;
	int ret;
	
	if(isTcp)
		tx_skb = iot_fill_skb_header(eth1_priv_data->ndev, &eth1_priv_data->tx_tcp_wait_q);	
	else
		tx_skb = iot_fill_skb_header(eth1_priv_data->ndev, &eth1_priv_data->tx_wait_q);	

	if (!tx_skb) {

		return 0;
	}
	
	if(tx_skb->len>(1526+1))	
		pr_err("len = %d\n",tx_skb->len);

	ret = iot_packet_send(tx_skb);

	if(ret==0)
	{
		eth1_priv_data->ndev->stats.tx_bytes += tx_skb->len;

		eth1_priv_data->ndev->stats.tx_packets++;	
	}
	
	dev_kfree_skb (tx_skb);
	return 1;
}

static int
iot_eth_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	int n;

	if(strcmp(ndev->name,"eth1")==0)
	{
		PIOT_ETH1_PRIVATE eth_priv_data = (PIOT_ETH1_PRIVATE )netdev_priv(ndev);

		host = eth_priv_data->pHost;
	}
	else
		host = (PIOT_PRIVATE)netdev_priv(ndev);
	
	if(ip_hdr(skb)->protocol == IPPROTO_TCP)
	{
		skb_queue_tail(&host->tx_tcp_wait_q, skb);

		if ((n=skb_queue_len (&host->tx_tcp_wait_q)) > (TX_QUEUE_HIGH_THRESHOLD/2)) {
			if (netif_msg_tx_queued (host))
				printk ("%s: Too much TX(TCP) packets in queue %d\n"
					, __FUNCTION__
					, skb_queue_len (&host->tx_tcp_wait_q));
			netif_stop_queue (ndev);
		}	
	}
	else
		skb_queue_tail (&host->tx_wait_q, skb);

	if ((n=skb_queue_len (&host->tx_wait_q)) > TX_QUEUE_HIGH_THRESHOLD) {
		if (netif_msg_tx_queued (host))
			printk ("%s: Too much TX(UDP) packets in queue %d\n"
				, __FUNCTION__
				, skb_queue_len (&host->tx_wait_q));
		netif_stop_queue (ndev);
	}
		
	set_bit (EVENT_TX, &host->flags);
	queue_work (host->iot_eth_work_queue, &host->iot_eth_work);
	
	return NETDEV_TX_OK;

}

static int
iot_eth1_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	int n;


	PIOT_ETH1_PRIVATE  eth_priv_data = (PIOT_ETH1_PRIVATE )netdev_priv(ndev);

	host = eth_priv_data->pHost;


	
	if(ip_hdr(skb)->protocol == IPPROTO_TCP)
	{
		skb_queue_tail(&eth_priv_data->tx_tcp_wait_q, skb);

		if ((n=skb_queue_len (&eth_priv_data->tx_tcp_wait_q)) > (TX_QUEUE_HIGH_THRESHOLD/2)) {
			if (netif_msg_tx_queued (host))
				printk ("%s: Too much TX(TCP) packets in queue %d\n"
					, __FUNCTION__
					, skb_queue_len (&eth_priv_data->tx_tcp_wait_q));
			netif_stop_queue (ndev);
		}	
	}
	else
	{
		skb_queue_tail (&eth_priv_data->tx_wait_q, skb);

		if ((n=skb_queue_len (&eth_priv_data->tx_wait_q)) > TX_QUEUE_HIGH_THRESHOLD) {
			if (netif_msg_tx_queued (host))
				printk ("%s: Too much TX(UDP) packets in queue %d\n"
					, __FUNCTION__
					, skb_queue_len (&eth_priv_data->tx_wait_q));
			netif_stop_queue (ndev);
		}
	}	
	set_bit (EVENT_TX, &host->flags);
	queue_work (host->iot_eth_work_queue, &host->iot_eth_work);
	
	return NETDEV_TX_OK;

}




static int
iot_eth_rx(struct net_device *dev)
{
#define OFFSET_DEST_MAC 12

	struct sk_buff *skb;
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(dev);
	struct iot_rxinfo* pRxinfo;
	int rxlen = PKT_SPI_RX_SIZE;
	struct net_device *ndev_eth1 = host->ndev_eth1;

	if (!netif_running(dev))
		goto drop;	

	do {

		if((skb = dev_alloc_skb(rxlen+IOT_RX_INFO_SIZE+SPIM_HEADER_SIZE)) != NULL) {
			u8 *pRead;
			int i;			
			pRead = (u8 *) skb_put(skb, rxlen);

			iot_read_eth_packet((void*)pRead,PKT_SPI_RX_SIZE);

			pRxinfo =(struct iot_rxinfo* ) pRead;
			
			if(pRxinfo->RxPktReady !=1)
				pr_info("RxPktReady: %d\r\n", pRxinfo->RxPktReady );		
			if(pRxinfo->status !=0x55)
				pr_info("status: %d\r\n", pRxinfo->status );		

			if(pRxinfo->type == RX_PKT_TYPE_LINK_STATE)
			{
				char* p = pRead;
				int i;
				for(i=0;i<32;++i)
					pr_info("%d: %x",i,p[i]);
				pr_info(" Link State: %s",p+sizeof(struct iot_rxinfo)+SPIM_HEADER_SIZE);

				if(strncmp(p+IOT_RX_INFO_SIZE+SPIM_HEADER_SIZE, "wifi disconnected", sizeof("wifi disconnected"))==0)
				{
				pr_info("netif_carrier_off");
					netif_carrier_off(dev);				
				}					
				if(strncmp(p+IOT_RX_INFO_SIZE+SPIM_HEADER_SIZE, "wifi connected", sizeof("wifi connected"))==0)
				{
					pr_info("netif_carrier_on");
					netif_carrier_on(dev);
				}
				dev_kfree_skb(skb);
				return 0;
			}
			
			rxlen = le16_to_cpu(pRxinfo->len);
#ifdef IOT_ETH_DEBUG
			if (netif_msg_rx_status(host)) {
  
				dev_info(host->dev, "RX: status %02x, length %04x\n",
					pRxinfo->status, rxlen);

			}
#endif		

			/* Packet Status check */
			if ((rxlen < 0x2a )||(rxlen > 1536)) {			
				if(rxlen!=0)
				{
					if (netif_msg_rx_err(host))
						dev_info(host->dev, "RX: Bad Packet (runt). len = %d\n",rxlen);

					dev->stats.rx_length_errors++;
					dev->stats.rx_errors++;
					dev->stats.rx_dropped++;
				}
				dev_kfree_skb(skb);
				return 0;
			}
			
			if(rxlen<=(0x2a+12))
				for(i=0;i<rxlen;++i)
				{
					pr_info(" %d %x ",i, pRead[i]);

				}
				
			dev->stats.rx_bytes += rxlen;

			skb_reserve(skb, IOT_RX_INFO_SIZE+SPIM_HEADER_SIZE);
			/* Pass to upper layer */

			if (netif_running(ndev_eth1))
			{
				if(!memcmp(&pRead[OFFSET_DEST_MAC], dev->dev_addr ,ETH_ALEN))
				{
					skb->protocol = eth_type_trans(skb, dev);	
					netif_rx(skb);				
				}
				else if(!memcmp(&pRead[OFFSET_DEST_MAC], ndev_eth1->dev_addr, ETH_ALEN))
				{
					skb->protocol = eth_type_trans(skb, ndev_eth1);	
					netif_rx(skb);			
				}
				else if(!memcmp(&pRead[OFFSET_DEST_MAC], broadcast_addr,  ETH_ALEN))
				{
					struct sk_buff * ap_skb = skb_clone(skb, GFP_ATOMIC);	
					ap_skb->protocol = eth_type_trans(ap_skb, ndev_eth1);
					netif_rx(ap_skb);

					skb->protocol = eth_type_trans(skb, dev);	
					netif_rx(skb);			
					
				}
			}
			else
			{
				skb->protocol = eth_type_trans(skb, dev);	
				netif_rx(skb);			
			}

			dev->stats.rx_packets++;

		}
	}while(0);
drop:	
	return 0;
}


static irqreturn_t iot_eth_interrupt (int irq, void *dev_instance)
{
	struct net_device *ndev = dev_instance;
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	if(gpio_get_value(MMPF_PIO_REG_GPIO117)==0)
		return IRQ_HANDLED;
	
	if (ndev == NULL) {
		printk (KERN_ERR "irq %d for unknown device.\n", irq);
		return IRQ_RETVAL (0);
	}
	if (!netif_running(ndev))
		return -EINVAL;

	disable_irq_nosync (irq);

	if (netif_msg_intr (host))
		printk ("%s: Interrupt occurred\n", ndev->name);

	set_bit (EVENT_D2H_INTR, &host->flags);

	queue_work (host->iot_eth_work_queue, &host->iot_eth_work);

	return IRQ_HANDLED;
}


static void iot_eth_work (struct work_struct *work)
{
#define OTHER_PKT 0
#define TCP_PKT 1

	int total_queue_len, total_eth1_queue_len;
	PIOT_PRIVATE host = 
			container_of(work, IOT_PRIVATE, iot_eth_work);

	down (&host->spi_lock);

	if (test_bit (EVENT_WATCHDOG, &host->flags)) {
		iot_eth_watchdog (host);

		clear_bit (EVENT_WATCHDOG, &host->flags);
	}

	if (test_bit (EVENT_D2H_INTR, &host->flags)) {

		iot_eth_rx(host->ndev);
		
		clear_bit (EVENT_D2H_INTR, &host->flags);

		enable_irq (host->ndev->irq);

	}
	
	if (test_bit (EVENT_TX, &host->flags)) {
		int n;
		int ret;
		int tx_count;
		PIOT_ETH1_PRIVATE eth_priv_data = (PIOT_ETH1_PRIVATE)netdev_priv(host->ndev_eth1);
	
		tx_count = 0;		
		while ((n = skb_queue_len(&host->tx_tcp_wait_q))) 
		{
		
			ret = iot_eth_hard_xmit (host, TCP_PKT); 
			
			if (!ret)
			{
				break;	
			}
			tx_count++;
			if(tx_count==MAX_TX_COUNT_PER_WORKLOOP)
				break;
			
		}

		tx_count = 0;
		while ((n = skb_queue_len(&host->tx_wait_q))) {
			
			ret = iot_eth_hard_xmit (host, OTHER_PKT ); 
			if (!ret)
			{
				break;	
			}
			tx_count++;
			if(tx_count==MAX_TX_COUNT_PER_WORKLOOP)
				break;
		}

		tx_count = 0;
		while ((n = skb_queue_len(&eth_priv_data->tx_tcp_wait_q))) 
		{
		
			ret = iot_eth1_hard_xmit (eth_priv_data, TCP_PKT); 
			
			if (!ret)
			{
				break;	
			}
			tx_count++;
			if(tx_count==MAX_TX_COUNT_PER_WORKLOOP)
				break;
			
		}

		tx_count = 0;
		while ((n = skb_queue_len(&eth_priv_data->tx_wait_q))) {
			
			ret = iot_eth1_hard_xmit (eth_priv_data, OTHER_PKT ); 
			if (!ret)
			{
				break;	
			}
			tx_count++;
			if(tx_count==MAX_TX_COUNT_PER_WORKLOOP)
				break;
		}

		clear_bit (EVENT_TX, &host->flags);

		total_queue_len = (skb_queue_len(&host->tx_wait_q)+skb_queue_len(&host->tx_tcp_wait_q)) ;
		if (netif_queue_stopped (host->ndev) && 
		    ((total_queue_len) < TX_QUEUE_LOW_THRESHOLD))
		{
			netif_wake_queue (host->ndev);
		}

		total_eth1_queue_len = (skb_queue_len(&eth_priv_data->tx_wait_q)+skb_queue_len(&host->tx_tcp_wait_q)) ;
		if (netif_queue_stopped (eth_priv_data->ndev) && 
		    ((total_eth1_queue_len) < TX_QUEUE_LOW_THRESHOLD))
		{
			netif_wake_queue (eth_priv_data->ndev);
		}

		if(total_queue_len+total_eth1_queue_len)
		{
			set_bit (EVENT_TX, &host->flags);
			queue_work (host->iot_eth_work_queue, &host->iot_eth_work);
		}		
	}

	if (test_bit (EVENT_SET_WIFI, &host->flags)) {
		printk(KERN_INFO "EVENT_SET_WIFI\n");

		if(host->pWifiSetting){
			iot_set_wifi(host->pWifiSetting);		
			host->pWifiSetting = 0;
		}

		clear_bit (EVENT_SET_WIFI, &host->flags);

	}

	up (&host->spi_lock);	
}


static struct net_device_stats *iot_eth_get_stats(struct net_device *ndev)
{
	return &ndev->stats;
}


static int
iot_eth_open(struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	int ret;
	unsigned long irq_flag = IRQF_SHARED;
	struct iot_spi_platform_data *spi_data = m_iot_spi_dev->dev.platform_data;

	netif_carrier_off (ndev);

	down (&host->spi_lock);
	
	// if(spi_data && spi_data->wakeup_pin)
		gpio_set_value(MMPF_PIO_REG_GPIO30, 1);

	iot_init(&host->mt_spi );
	iot_eth_set_mac_addr (ndev);

	irq_flag|=IRQF_TRIGGER_HIGH;

	ret = request_irq (ndev->irq, iot_eth_interrupt,
			irq_flag, ndev->name, ndev);
	if (ret) {
		printk (KERN_ERR "%s: unable to get IRQ %d (errno=%d).\n",
				ndev->name, ndev->irq, ret);
		goto err;
	}

	host->seq_num = 0x1f;

	netif_start_queue (ndev);

	spi_message_init(&host->mt_spi.rx_msg);

	up (&host->spi_lock);

	init_timer (&host->watchdog);
	host->watchdog.function = &iot_eth_watchdog_timer;
	host->watchdog.expires = jiffies + MT7687_WATCHDOG_PERIOD;
	host->watchdog.data = (unsigned long) ndev;

	add_timer (&host->watchdog);

	netif_carrier_on (ndev);

	return 0;
	
err:
      up (&host->spi_lock);
      return ret;
}


static int
iot_eth1_open(struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	netif_carrier_off (ndev);

	iot_eth_set_mac_addr (ndev);

	host->seq_num = 0x1f;

	netif_start_queue (ndev);

	spi_message_init(&host->mt_spi.rx_msg);
	netif_carrier_on (ndev);

	return 0;
	
}

static void iot_eth_free_skb_queue (struct sk_buff_head *q)
{
	struct sk_buff *skb;

	while (q->qlen) {
		skb = skb_dequeue (q);
		kfree_skb (skb);
	}
}


static int
iot_eth_close(struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	struct iot_spi_platform_data *spi_data = m_iot_spi_dev->dev.platform_data;

	netif_stop_queue(ndev);

	del_timer_sync (&host->watchdog);

	free_irq (ndev->irq, ndev);

	down (&host->spi_lock);
	iot_eth_free_skb_queue (&host->tx_tcp_wait_q);
	iot_eth_free_skb_queue (&host->tx_wait_q);
	up (&host->spi_lock);


	// if(spi_data && spi_data->wakeup_pin)
		gpio_set_value(MMPF_PIO_REG_GPIO30, 0);


	return 0;
}

static int
iot_eth1_close(struct net_device *ndev)
{

	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	PIOT_ETH1_PRIVATE  eth_priv_data = (PIOT_ETH1_PRIVATE )netdev_priv(ndev);

	host = eth_priv_data->pHost;

//	struct iot_spi_platform_data *spi_data = m_iot_spi_dev->dev.platform_data;

	netif_stop_queue(ndev);

	down (&host->spi_lock);
	iot_eth_free_skb_queue (&eth_priv_data->tx_tcp_wait_q);
	iot_eth_free_skb_queue (&eth_priv_data->tx_wait_q);
	up (&host->spi_lock);

	return 0;
}

static int iot_eth_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	return iot_priv_ioctl(ndev, cmd, req->ifr_data);
}


static const struct net_device_ops iot_eth_netdev_ops = {
	.ndo_open		= iot_eth_open,
	.ndo_stop		= iot_eth_close,
	.ndo_start_xmit		= iot_eth_start_xmit,
	.ndo_get_stats		= iot_eth_get_stats,
	.ndo_do_ioctl		= iot_eth_ioctl,
	.ndo_set_mac_address	= iot_eth_set_mac_address,
};

static const struct net_device_ops iot_eth1_netdev_ops = {
	.ndo_open		= iot_eth1_open,
	.ndo_stop		= iot_eth1_close,
	.ndo_start_xmit		= iot_eth1_start_xmit,
	.ndo_get_stats		= iot_eth_get_stats,
	.ndo_do_ioctl		= iot_eth_ioctl,
	.ndo_set_mac_address	= iot_eth_set_mac_address,
};

static int __devinit iot_eth_probe (struct spi_device *spi)
{
	struct net_device *ndev, *ndev_eth1;
	PIOT_PRIVATE host;
	PIOT_ETH1_PRIVATE  eth_priv_data;
	int ret;
		
 	ndev = alloc_etherdev (sizeof (*host));
	if (!ndev) {
		printk(KERN_ERR
		       "iot_eth_probe: Could not allocate ethernet device\n");
		return -ENOMEM;
	}

	ndev_eth1 = alloc_etherdev (sizeof (*eth_priv_data));
	if (!ndev) {
		printk(KERN_ERR
		       "iot_eth_probe: Could not allocate ethernet device\n");
		return -ENOMEM;
	}


	m_iot_spi_dev = spi;

	if(m_iot_spi_dev->dev.platform_data)
	{
		struct iot_spi_platform_data *spi_data = m_iot_spi_dev->dev.platform_data;
		// if(spi_data->wakeup_pin)
			gpio_request_one(MMPF_PIO_REG_GPIO30, GPIOF_DIR_OUT|GPIOF_INIT_LOW,"WAKEUP_IoT");
	}		


	spi->bits_per_word = 8;

	host = (PIOT_PRIVATE)netdev_priv(ndev);
	memset (host, 0, sizeof (*host));

	dev_set_drvdata(&spi->dev, host);
	host->dev = &spi->dev;

	host->spi = spi;
	host->mt_spi.spi = spi;	
	ndev->irq = spi->irq;

	printk(version);

	host->msg_enable =  msg_enable;

	ndev->netdev_ops	= &iot_eth_netdev_ops;
	ndev->ethtool_ops	= &iot_ethtool_ops;

	host->ndev = ndev;

	if (netif_msg_probe (host))
		printk(KERN_INFO "addr 0x%lx, irq %d, ",
		       ndev->base_addr, ndev->irq);


	INIT_WORK(&host->iot_eth_work, iot_eth_work);
	
	host->iot_eth_work_queue = 
			create_singlethread_workqueue ("iot_eth_work");

	sema_init (&host->spi_lock,1);
	
	skb_queue_head_init(&host->tx_tcp_wait_q);
	skb_queue_head_init(&host->tx_wait_q);
	
	ndev->features &= ~NETIF_F_HW_CSUM;
	ndev->hard_header_len += (TX_OVERHEAD+4);

	eth_priv_data = (PIOT_ETH1_PRIVATE)netdev_priv(ndev_eth1);
	
	ndev_eth1->netdev_ops	= &iot_eth1_netdev_ops;
	ndev_eth1->ethtool_ops	= &iot_ethtool_ops;
	ndev_eth1->features &= ~NETIF_F_HW_CSUM;
	ndev_eth1->hard_header_len += (TX_OVERHEAD+4);
	
	eth_priv_data->pHost = host;
	eth_priv_data->ndev = ndev_eth1;

	skb_queue_head_init(&eth_priv_data->tx_tcp_wait_q);
	skb_queue_head_init(&eth_priv_data->tx_wait_q);

	
	host->ndev_eth1 = ndev_eth1;


	ret = register_netdev(ndev);
	if (ret)
	{	
		goto exit;
	}

	ret = register_netdev(ndev_eth1);
	if (ret)
	{	
		goto exit;
	}
	
	return ret;
exit:
	destroy_workqueue(host->iot_eth_work_queue);

	free_netdev(ndev);

	return ret;
}


static int
iot_eth_suspend (struct spi_device *spi, pm_message_t mesg)
{
	PIOT_PRIVATE host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "iot_eth_suspend.\n");
	if (!ndev || !netif_running (ndev))
		return 0;

	netif_device_detach (ndev);

	netif_stop_queue (ndev);
	return 0;
}


static int
iot_eth_resume (struct spi_device *spi)
{

	PIOT_PRIVATE host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "iot_eth_resume.\n");

	down (&host->spi_lock);

	netif_device_attach(ndev);

	netif_start_queue (ndev);

	up (&host->spi_lock);

	return 0;
}


static int __devexit iot_eth_remove (struct spi_device *spi)
{
	PIOT_PRIVATE host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;
		
	destroy_workqueue (host->iot_eth_work_queue);

	unregister_netdev(ndev);

	free_netdev(ndev);

	printk(KERN_INFO "iot_eth_remove: released and freed device\n");

	return 0;
}


static struct spi_driver iot_eth_spi_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = iot_eth_probe,
	.remove = __devexit_p(iot_eth_remove),
	.suspend = iot_eth_suspend,
	.resume = iot_eth_resume,
};


static __init int iot_eth_spi_init(void)
{

	printk(KERN_INFO "Register IoT SPI Net Driver.\n");
	spi_register_driver(&iot_eth_spi_driver);


	return 0;
}


static __exit void iot_eth_spi_exit(void)
{
	spi_unregister_driver(&iot_eth_spi_driver);
}

module_init(iot_eth_spi_init);
module_exit(iot_eth_spi_exit);


MODULE_AUTHOR("Vincent@Mstar");
MODULE_DESCRIPTION("SPI IOT ethernet driver");
MODULE_LICENSE("GPL");
