/* MT7687 SPI IOT Linux driver */

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

#include "mt7687_main.h"
#include "mt7687_ioctl.h"
#include "mt7687_spi.h"

#include "iot_api.h"


#define AIT_GPIO_ETHERNET_RESET MMPF_PIO_REG_GPIO115

#define MT7687_DRV_NAME	"MT7687-SPI"
#define MAX_TX_COUNT_PER_WORKLOOP 32
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
#if 0
			NETIF_MSG_TIMER |
			NETIF_MSG_RX_ERR |
			NETIF_MSG_TX_ERR |
			NETIF_MSG_TX_QUEUED |
			NETIF_MSG_INTR |
			NETIF_MSG_TX_DONE |
			NETIF_MSG_RX_STATUS |
			NETIF_MSG_PKTDATA |
			NETIF_MSG_HW |
			NETIF_MSG_WOL;
#endif

module_param (msg_enable, int, 0);
MODULE_PARM_DESC (msg_enable, "Message level");


static int reset_pin = AIT_GPIO_ETHERNET_RESET;

module_param (reset_pin, int, 0);
MODULE_PARM_DESC (reset_pin, "Reset GPIO PIN #");

static void mt7687_watchdog (struct mt7687_spi_host* host)
{
	unsigned long time_to_chk = MT7687_WATCHDOG_PERIOD;

	if (time_to_chk)
		mod_timer (&host->watchdog, jiffies + time_to_chk);
}

static void mt7687_watchdog_timer (unsigned long arg)
{
	struct net_device *ndev = (struct net_device *)(arg);
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	set_bit (EVENT_WATCHDOG, &host->flags);
		
	queue_work (host->mt7687_work_queue, &host->mt7687_work);
}
#if 0
static void mt7687_set_multicast (struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	set_bit (EVENT_SET_MULTI, &host->flags);
	queue_work (host->mt7687_work_queue, &host->mt7687_work);
}
#endif

static void mt7687_set_mac_addr (struct net_device *ndev)
{
	int ret;
	struct mt7687_spi_host*  host = (struct mt7687_spi_host* )netdev_priv(ndev);

	char* mac = ndev->dev_addr;

	ret = iot_get_mac(&host->mt_spi, mac );
	
	pr_info("mt7687_set_mac_addr: %d %x %x %x %x %x %x\r\n",ret,mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );


	return;
}


static int mt7687_set_mac_address (struct net_device *ndev, void *p)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	down (&host->spi_lock);

	mt7687_set_mac_addr (ndev);

	up (&host->spi_lock);

	return 0;
}


unsigned long pre_sent_size = 0;
unsigned long  sent_data_size = 0;
unsigned long sent_num_pkts = 0;
unsigned long t=0;
static struct sk_buff *
mt_iot_fill_header(struct net_device *ndev, struct sk_buff_head *q)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	struct sk_buff *skb, *tx_skb;
	struct iot_pkt_head_t *info;
	struct skb_data *entry;
	int headroom;
	int tailroom;
	u8 need_pages;
	u16 tol_len, pkt_len;
	u8 padlen;
	u16 seq_num;


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
			TX_HEAD_SIZE + TX_EOP_SIZE;
	seq_num = ++host->seq_num;// & 0x1F;

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

	entry = (struct skb_data *) tx_skb->cb;
	memset (entry, 0, sizeof (*entry));
	entry->len = pkt_len;

	return tx_skb;
}

#if 0
static int mt7687_hard_xmit_tcp (struct mt7687_spi_host* host, u32 bufid)
{
	struct sk_buff *tx_skb;
	struct skb_data *entry;

	tx_skb = mt_iot_fill_header(host->ndev, &host->tx_tcp_wait_q);	
	if (!tx_skb) {

		return 0;
	}

	entry = (struct skb_data *) tx_skb->cb;
	
	if(tx_skb->len>1526+1)	
		pr_err("len = %d\n",tx_skb->len);

	iot_packet_send(&host->mt_spi, bufid, tx_skb);
	
	entry->state = tx_done;
	dev_kfree_skb (tx_skb);
	return 1;
}

#endif
static int mt7687_hard_xmit (struct mt7687_spi_host* host, u32 isTcp)
{
	struct sk_buff *tx_skb;
	struct skb_data *entry;

	if(isTcp)
		tx_skb = mt_iot_fill_header(host->ndev, &host->tx_tcp_wait_q);	
	else
		tx_skb = mt_iot_fill_header(host->ndev, &host->tx_wait_q);	

	if (!tx_skb) {

		return 0;
	}

	entry = (struct skb_data *) tx_skb->cb;
	
	if(tx_skb->len>(1526+1))	
		pr_err("len = %d\n",tx_skb->len);

	iot_packet_send(&host->mt_spi, tx_skb);
	
	entry->state = tx_done;
	dev_kfree_skb (tx_skb);
	return 1;
}

static int
mt7687_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	int n;
	if(ip_hdr(skb)->protocol == IPPROTO_TCP)
	{
		skb_queue_tail(&host->tx_tcp_wait_q, skb);
		#ifdef DEBUG 
		pr_info("add tcp skb to q\n");
		#endif


		if ((n=skb_queue_len (&host->tx_tcp_wait_q)) > TX_QUEUE_HIGH_THRESHOLD) {
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
	queue_work (host->mt7687_work_queue, &host->mt7687_work);
	
	return NETDEV_TX_OK;

}

#if 0
static int
mt7687_rx(struct net_device *dev)
{
	struct sk_buff *skb;
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(dev);
	struct mt7687_rxinfo rxinfo;
	int rxlen = 1600;
//	char tempbuf[1600];
	
	iot_read_data(&host->mt_spi, SPIS_CMD_GET_ETH_PKT, (void*)tempbuf,1600);

	if (!netif_running(dev))
		goto drop;	
#ifdef DEBUG  	
	{
		pr_info("rx");
		{
			int i;
			for(i=0;i<64;++i)
			{
				if(i%16==0)
					pr_info("\r\n%d: ",i);
				pr_info("%x ",tempbuf[i]);
	
			}
	
		}				
	}
#endif
	do {

//		memcpy((void*)&rxinfo, tempbuf, sizeof(rxinfo));

//pr_info("RxPktReady: %d\r\n", rxinfo.RxPktReady );		
//		pr_info("status: %d\r\n", rxinfo.status );		
//		pr_info("len: %d\r\n", rxinfo.len);		

//		rxlen = le16_to_cpu(rxinfo.len);
		pr_info("rxlen : %d\r\n", rxlen ); 	

		if (netif_msg_rx_status(host)) {
		#ifdef DEBUG  
			dev_info(host->dev, "RX: status %02x, length %04x\n",
				rxinfo.status, rxlen);
		#endif		
		}
		/* Packet Status check */
		if (rxlen < 0x2a) {
			if (netif_msg_rx_err(host))
				dev_dbg(host->dev, "RX: Bad Packet (runt)\n");
		}

		if (rxlen > 1536) {
			dev_dbg(host->dev, "RST: RX Len:%x\n", rxlen);
		}

		if((skb = dev_alloc_skb(rxlen+8+4)) != NULL) {
			u8 *pRead;
			
			pRead = (u8 *) skb_put(skb, rxlen);

			//memcpy((void*)pRead, tempbuf, rxlen+8+4);
			iot_read_data(&host->mt_spi, SPIS_CMD_GET_ETH_PKT, (void*)tempbuf,1600);

			dev->stats.rx_bytes += rxlen;

			skb_reserve(skb, 8+4);
			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb, dev);

			netif_rx(skb);
			dev->stats.rx_packets++;

		}
	}while(0);
drop:	
	return 0;
}
#else
static int
mt7687_rx(struct net_device *dev)
{
	struct sk_buff *skb;
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(dev);
	struct mt7687_rxinfo* pRxinfo;
	int rxlen = 1600;
	

	if (!netif_running(dev))
		goto drop;	
#ifdef DEBUG  	
	{
		pr_info("rx");
		{
			int i;
			for(i=0;i<64;++i)
			{
				if(i%16==0)
					pr_info("\r\n%d: ",i);
				pr_info("%x ",tempbuf[i]);
	
			}
	
		}				
	}
#endif
	do {

		if((skb = dev_alloc_skb(rxlen+8+4)) != NULL) {
			u8 *pRead;
			
			pRead = (u8 *) skb_put(skb, rxlen);

			iot_read_data(&host->mt_spi, SPIS_CMD_GET_ETH_PKT, (void*)pRead,1600);

			pRxinfo =(struct mt7687_rxinfo* ) pRead;
			
			if(pRxinfo->RxPktReady !=1)
				pr_info("RxPktReady: %d\r\n", pRxinfo->RxPktReady );		
			if(pRxinfo->status !=0x55)
				pr_info("status: %d\r\n", pRxinfo->status );		

			rxlen = le16_to_cpu(pRxinfo->len);

			if (netif_msg_rx_status(host)) {
#ifdef DEBUG  
				dev_info(host->dev, "RX: status %02x, length %04x\n",
					rxinfo.status, rxlen);
#endif		
			}
			/* Packet Status check */
			if (rxlen < 0x2a) {
				
			}

			if ((rxlen < 0x2a )||(rxlen > 1536)) {
				if (netif_msg_rx_err(host))
					dev_dbg(host->dev, "RX: Bad Packet (runt)\n");
	
				pr_info("len: %d\r\n", pRxinfo->len);		
						
			}

			dev->stats.rx_bytes += rxlen;

			skb_reserve(skb, 8+4);
			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb, dev);

			netif_rx(skb);
			dev->stats.rx_packets++;

		}
	}while(0);
drop:	
	return 0;
}
#endif

static irqreturn_t mt7687_interrupt (int irq, void *dev_instance)
{
	struct net_device *ndev = dev_instance;
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
 
	if (ndev == NULL) {
		printk (KERN_ERR "irq %d for unknown device.\n", irq);
		return IRQ_RETVAL (0);
	}
	if (!netif_running(ndev))
		return -EINVAL;

	disable_irq_nosync (irq);

	if (netif_msg_intr (host))
		printk ("%s: Interrupt occurred\n", ndev->name);

	set_bit (EVENT_H2D_INTR, &host->flags);
	queue_work (host->mt7687_work_queue, &host->mt7687_work);

	return IRQ_HANDLED;
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_work
 * Purpose:
 * ----------------------------------------------------------------------------
 */

static void mt7687_work (struct work_struct *work)
{
	int total_queue_len;
//	int sent_time = 0,total_tx_time=0;
	struct mt7687_spi_host* host = 
			container_of(work, struct mt7687_spi_host, mt7687_work);

	down (&host->spi_lock);

	if (test_bit (EVENT_WATCHDOG, &host->flags)) {
		mt7687_watchdog (host);

		clear_bit (EVENT_WATCHDOG, &host->flags);
	}

	if (test_bit (EVENT_SET_MULTI, &host->flags)) {
#ifdef DEBUG
	printk(KERN_INFO "EVENT_SET_MULTI\n");
#endif
		clear_bit (EVENT_SET_MULTI, &host->flags);
	}

	if (test_bit (EVENT_H2D_INTR, &host->flags)) {

		while (1) {
			
			if (!	mt7687_rx(host->ndev))
				break;
		}

		clear_bit (EVENT_H2D_INTR, &host->flags);

		enable_irq (host->ndev->irq);
	
	}
	
	if (test_bit (EVENT_TX, &host->flags)) {
		int n;
		int ret;
		int tx_count;
		while ((n = skb_queue_len(&host->tx_tcp_wait_q))) 
		{
		
			ret = mt7687_hard_xmit (host, 1); 
			
			if (!ret)
			{
				break;	
			}
		}
		//printk(KERN_INFO "EVENT_TX\n");
		tx_count = 0;
		while ((n = skb_queue_len(&host->tx_wait_q))) {
			
			ret = mt7687_hard_xmit (host, 0); 
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

		if(total_queue_len)
		{
			set_bit (EVENT_TX, &host->flags);
			queue_work (host->mt7687_work_queue, &host->mt7687_work);
		}
	}

	if (test_bit (EVENT_SET_WIFI, &host->flags)) {
		printk(KERN_INFO "EVENT_SET_WIFI\n");

		if(host->pWifiSetting){
			iot_set_wifi(&host->mt_spi, host->pWifiSetting);		
			host->pWifiSetting = 0;
		}

		clear_bit (EVENT_SET_WIFI, &host->flags);

	}

	up (&host->spi_lock);	
}

static struct net_device_stats *mt7687_get_stats(struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	return &host->stats;
}

static int
mt7687_open(struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	int ret;
	unsigned long irq_flag = IRQF_SHARED;

	netif_carrier_off (host->ndev);

	down (&host->spi_lock);

	iot_init(&host->mt_spi );
	mt7687_set_mac_addr (ndev);


	irq_flag|=IRQF_TRIGGER_RISING;//IRQF_TRIGGER_HIGH;

	ret = request_irq (ndev->irq, mt7687_interrupt,
			irq_flag, ndev->name, ndev);
	if (ret) {
		printk (KERN_ERR "%s: unable to get IRQ %d (errno=%d).\n",
				ndev->name, ndev->irq, ret);
		goto err;
	}

	host->seq_num = 0x1f;

	netif_start_queue (ndev);

	if (netif_msg_hw (host)) {
#ifdef DEBUG	  
		printk ("\nDump all MAC registers after initialization:\n");
		mt7687_dump_regs (&host->mt_spi);
#endif		
	}

	spi_message_init(&host->mt_spi.rx_msg);

	up (&host->spi_lock);

	init_timer (&host->watchdog);
	host->watchdog.function = &mt7687_watchdog_timer;
	host->watchdog.expires = jiffies + MT7687_WATCHDOG_PERIOD;
	host->watchdog.data = (unsigned long) ndev;
	host->w_state = chk_cable;
	host->w_ticks = 0;

	add_timer (&host->watchdog);

	netif_carrier_on (host->ndev);

	return 0;
	
err:
      up (&host->spi_lock);
      return ret;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_free_skb_queue
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void mt7687_free_skb_queue (struct sk_buff_head *q)
{
	struct sk_buff *skb;

	while (q->qlen) {
		skb = skb_dequeue (q);
		kfree_skb (skb);
	}
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_close
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static int
mt7687_close(struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);

	netif_stop_queue(ndev);

	del_timer_sync (&host->watchdog);

	free_irq (ndev->irq, ndev);

	down (&host->spi_lock);
	mt7687_free_skb_queue (&host->tx_tcp_wait_q);
	mt7687_free_skb_queue (&host->tx_wait_q);
	up (&host->spi_lock);

	return 0;
}

static int mt7687_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	return mt7687_priv_ioctl(ndev, cmd, req->ifr_data);
}


static const struct net_device_ops mt7687_netdev_ops = {
	.ndo_open		= mt7687_open,
	.ndo_stop		= mt7687_close,
	.ndo_start_xmit		= mt7687_start_xmit,
	.ndo_get_stats		= mt7687_get_stats,
//	.ndo_set_rx_mode = mt7687_set_multicast,
	.ndo_do_ioctl		= mt7687_ioctl,
	.ndo_set_mac_address	= mt7687_set_mac_address,
};


static int __devinit mt7687_probe (struct spi_device *spi)
{
	struct net_device *ndev;
	struct mt7687_spi_host* host;
	int ret;

 	ndev = alloc_etherdev (sizeof (*host));
	if (!ndev) {
		printk(KERN_ERR
		       "MT7687 SPI: Could not allocate ethernet device\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 8;

	host = (struct mt7687_spi_host*)netdev_priv(ndev);
	memset (host, 0, sizeof (*host));

	dev_set_drvdata(&spi->dev, host);
	host->dev = &spi->dev;

	host->spi = spi;
	host->mt_spi.spi = spi;	
	ndev->irq = spi->irq;

	printk(version);

	host->msg_enable =  msg_enable;

	ndev->netdev_ops	= &mt7687_netdev_ops;
	ndev->ethtool_ops	= &mt7687_ethtool_ops;

	host->ndev = ndev;

	if (netif_msg_probe (host))
		printk(KERN_INFO PFX "addr 0x%lx, irq %d, ",
		       ndev->base_addr, ndev->irq);


	INIT_WORK(&host->mt7687_work, mt7687_work);
	
	host->mt7687_work_queue = 
			create_singlethread_workqueue ("mt7687_work");

	sema_init (&host->spi_lock,1);
	
	skb_queue_head_init(&host->tx_tcp_wait_q);
	skb_queue_head_init(&host->tx_wait_q);
	
	ndev->features &= ~NETIF_F_HW_CSUM;
	ndev->hard_header_len += (TX_OVERHEAD+4);

	ret = register_netdev(ndev);
	if (!ret)
	{	
		return ret;
	}
	
	destroy_workqueue(host->mt7687_work_queue);

	free_netdev(ndev);

	return ret;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_suspend
 * Purpose: Device suspend handling function
 * ----------------------------------------------------------------------------
 */
static int
mt7687_suspend (struct spi_device *spi, pm_message_t mesg)
{
	struct mt7687_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "mt7687_suspend.\n");
	if (!ndev || !netif_running (ndev))
		return 0;

	netif_device_detach (ndev);

	netif_stop_queue (ndev);
	return 0;
}


static int
mt7687_resume (struct spi_device *spi)
{

	struct mt7687_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "mt7687_resume.\n");

	down (&host->spi_lock);

	netif_device_attach(ndev);

	netif_start_queue (ndev);

	up (&host->spi_lock);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_remove
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static int __devexit mt7687_remove (struct spi_device *spi)
{
	struct mt7687_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;
		
	destroy_workqueue (host->mt7687_work_queue);

	unregister_netdev(ndev);

	free_netdev(ndev);

	printk(KERN_INFO "mt7687-spi: released and freed device\n");

	return 0;
}

static struct spi_driver mt7687_spi_driver = {
	.driver = {
		.name = "spidev-mt7687",
		.owner = THIS_MODULE,
	},
	.probe = mt7687_probe,
	.remove = __devexit_p(mt7687_remove),
	.suspend = mt7687_suspend,
	.resume = mt7687_resume,
};

/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_spi_init
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static __init int mt7687_spi_init(void)
{

	printk(KERN_INFO "Register MT7687 SPI Net Driver.\n");
	spi_register_driver(&mt7687_spi_driver);
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: MT7687_spi_exit
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static __exit void mt7687_spi_exit(void)
{
	spi_unregister_driver(&mt7687_spi_driver);
}

module_init(mt7687_spi_init);
module_exit(mt7687_spi_exit);


MODULE_AUTHOR("Vincent@Mstar");
MODULE_DESCRIPTION("MT7687 SPI IOT driver");
MODULE_LICENSE("GPL");
