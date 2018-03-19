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

#include "mt7687_main.h"
#include "mt7687_ioctl.h"
#include "mt7687_spi.h"

#include <linux/gpio.h>
#include <mach/board.h>

#define MT7687_DRV_NAME	"MT7687-SPI"

//#define USE_RX_WAIT_Q

static char version[] __devinitdata =
KERN_INFO ADP_NAME ".c:v" DRV_VERSION " " __TIME__ " " __DATE__ "\n";

static int comp = 1;

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

module_param (comp, int, 0);
MODULE_PARM_DESC (comp, 
	"0=Non-Compression Mode, 1=Compression Mode");

module_param (msg_enable, int, 0);
MODULE_PARM_DESC (msg_enable, "Message level");

#define AIT_GPIO_ETHERNET_RESET MMPF_PIO_REG_GPIO115

static int reset_pin = AIT_GPIO_ETHERNET_RESET;

module_param (reset_pin, int, 0);
MODULE_PARM_DESC (reset_pin, "Reset GPIO PIN #");

static struct spi_device *m_rx_spi;

#if CONFIG_SUPPORT_2WAY_SPI
struct task_struct	*mt7687_spim_handler_task = 0;

struct mt7687_spi_rx_ctx {
	struct net_device *ndev;
	struct sk_buff *skb;
	u8 *rx_data;
	struct completion *x;	
	struct mt7687_spi_host* spihost;	
};
static volatile u16 pre_seq = 0;
static volatile u16 rx_mode = 0;

static void spi_rx_complete(void *arg)
{

	struct mt7687_spi_rx_ctx* ctx = arg;
	struct iot_spi_platform_data *spi_data = m_rx_spi->dev.platform_data;
	u8* pRead = ctx->rx_data;
	struct sk_buff *skb = ctx->skb;
	volatile u16 seq;
	int rxlen;

	seq = (pRead[5]<<8)|pRead[4];

	if((seq-pre_seq)!=1)
		pr_warn("pre_seq = %d   seq: %d\n",pre_seq, seq);

	pre_seq = seq;
	
	rxlen = ((pRead[9]<<8)|pRead[8])-14;	
	skb_reserve(skb, 12);
	skb_trim(skb, rxlen);

	skb->protocol = eth_type_trans(skb, ctx->ndev);
							
	/* Pass to upper layer */
	netif_rx(skb);

	ctx->ndev->stats.rx_bytes += rxlen;
	ctx->ndev->stats.rx_packets++;

	complete(ctx->x);

	gpio_direction_output(spi_data->s2m_irq_gpio, 0);	
}

static int mt7687_spi_rx_handler_task(void* ndev)//struct net_device *ndev)
{

	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv((struct net_device *)ndev);
	struct iot_spi_platform_data *spi_data = m_rx_spi->dev.platform_data;
	struct sk_buff *skb;
	struct sched_param param = {.sched_priority = 99};
	struct mt7687_spi_rx_ctx spi_rx_ctx;
	struct spi_message	message;
	struct spi_transfer	x[2];
	static char tx_buf[20];
	int ret;
	unsigned long t1,t2;

	
	DECLARE_COMPLETION_ONSTACK(done);
	pr_info("mt7687_spim_rx thread start\n");

	sched_setscheduler(current, SCHED_FIFO, &param);

	tx_buf[0] = 0x11;
	tx_buf[1] = 0x22;
	tx_buf[2] = 0x33;
	tx_buf[3] = 0x44;
	tx_buf[4] = 0x55;
	tx_buf[5] = 0x66;
	tx_buf[6] = 0x77;
	tx_buf[7] = 0x88;
	tx_buf[8] = 0x99;
	tx_buf[9] = 0xaa;
	tx_buf[10] = 0xbb;
	tx_buf[11] = 0xcc;
	tx_buf[12] = 0xdd;
	tx_buf[13] = 0xee;
	tx_buf[14] = 0xff;
	tx_buf[15] = 0x00;
	tx_buf[16] = 0x16;
	tx_buf[17] = 0x17;
	tx_buf[18] = 0x18;
	tx_buf[19] = 0x19;

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	x[0].len = 8;
	x[0].tx_buf = tx_buf;				
	spi_message_add_tail(&x[0], &message);

	x[1].len =  1514+12;
	spi_message_add_tail(&x[1], &message);

	message.complete = spi_rx_complete;
	message.context = &spi_rx_ctx;

				
	spi_rx_ctx.ndev = ndev;
	spi_rx_ctx.x = &done;	
	spi_rx_ctx.spihost = host;
				
	while (!kthread_should_stop()) {
		t1 = jiffies;
		if((skb = dev_alloc_skb(1536+8)) != NULL) {
			u8 *pRead;
						
			pRead = (u8 *) skb_put(skb, 1536);

			x[1].rx_buf = pRead;

			spi_rx_ctx.skb = skb;
			spi_rx_ctx.rx_data = pRead;

			spi_async(m_rx_spi, &message);
			gpio_set_value(spi_data->s2m_irq_gpio, 1);

			t2 = jiffies;			
			if((t2-t1)>1)
				pr_info("spi setup time too long: %d\n",t2-t1);
		
			wait_for_completion_interruptible(&done);


		}
	}
	pr_info("mt7687_spim_rx thread exit.\n");
	return 0;
}



#endif

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

static void mt7687_set_multicast (struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	set_bit (EVENT_SET_MULTI, &host->flags);
	queue_work (host->mt7687_work_queue, &host->mt7687_work);
}

static void mt7687_set_mac_addr (struct net_device *ndev)
{
	struct mt7687_spi_host*  host = (struct mt7687_spi_host* )netdev_priv(ndev);

	mt7687_read_spis_mac(&host->mt_spi, ndev->dev_addr);

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


static struct sk_buff *
mt7687_tx_fixup (struct net_device *ndev, struct sk_buff_head *q)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	struct sk_buff *skb, *tx_skb;
	struct tx_pkt_info *info;
	struct skb_data *entry;
	int headroom;
	int tailroom;
	u8 need_pages;
	u16 tol_len, pkt_len;
	u8 padlen, seq_num;
	u8 spi_len = host->mt_spi.comp ? 1 : 4;

	if (skb_queue_empty (q)) {
		return NULL;
	}

	skb = skb_peek (q);
	pkt_len = skb->len;
	need_pages = (pkt_len + TX_OVERHEAD + 127) >> 7;

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);
	padlen = ((pkt_len + 3 ) & 0x7FC) - pkt_len;
	tol_len = ((pkt_len + 3 ) & 0x7FC) + 
			TX_OVERHEAD + TX_EOP_SIZE + spi_len;
	seq_num = ++host->seq_num & 0x1F;

	info = (struct tx_pkt_info *) skb->cb;
	info->pkt_len = pkt_len;

	if ((!skb_cloned(skb)) &&
	    (headroom >= (TX_OVERHEAD + spi_len)) &&
	    (tailroom >= (padlen + TX_EOP_SIZE))) {

		info->seq_num = seq_num;

		/* SOP and SEG header */
		memcpy (skb_push (skb, TX_OVERHEAD), &info->sop, TX_OVERHEAD);

		/* Make 32-bit aligment */
		skb_put (skb, padlen);

		/* EOP header */
		memcpy (skb_put (skb, TX_EOP_SIZE), &info->eop, TX_EOP_SIZE);

		tx_skb = skb;
		skb_unlink(skb, q);

	} else {

		tx_skb = alloc_skb (tol_len, GFP_KERNEL);
		if (!tx_skb)
			return NULL;

		info->seq_num = seq_num;

		/* SOP and SEG header */
		memcpy (skb_put (tx_skb, TX_OVERHEAD),
				&info->sop, TX_OVERHEAD);

		/* Packet */
		memcpy (skb_put (tx_skb, ((pkt_len + 3) & 0xFFFC)),
				skb->data, pkt_len);

		/* EOP header */
		memcpy (skb_put (tx_skb, TX_EOP_SIZE),
				&info->eop, TX_EOP_SIZE);

		skb_unlink (skb, q);
		dev_kfree_skb (skb);
	}

	entry = (struct skb_data *) tx_skb->cb;
	memset (entry, 0, sizeof (*entry));
	entry->len = pkt_len;

	return tx_skb;
}

#define SUPPORT_TCP_WAIT_QUEUE 1

#ifdef SUPPORT_TCP_WAIT_QUEUE
static int mt7687_hard_xmit_tcp (struct mt7687_spi_host* host, u32 bufid)
{
	struct sk_buff *tx_skb;
	struct skb_data *entry;

	tx_skb = mt7687_tx_fixup (host->ndev, &host->tx_tcp_wait_q);

	if (!tx_skb) {

		return 0;
	}

	entry = (struct skb_data *) tx_skb->cb;
	
	if(tx_skb->len>1526)	
		pr_err("len = %d\n",tx_skb->len);

	mt7687_iot_buf_prepare(&host->mt_spi, bufid, tx_skb);
	
	entry->state = tx_done;
	dev_kfree_skb (tx_skb);
	return 1;
}
#endif

static int mt7687_hard_xmit (struct mt7687_spi_host* host, u32 bufid)
{
	struct sk_buff *tx_skb;
	struct skb_data *entry;

	tx_skb = mt7687_tx_fixup (host->ndev, &host->tx_wait_q);

	if (!tx_skb) {

		return 0;
	}

	entry = (struct skb_data *) tx_skb->cb;
	
	if(tx_skb->len>1526)	
		pr_err("len = %d\n",tx_skb->len);

	mt7687_iot_buf_prepare(&host->mt_spi, bufid, tx_skb);
	
	entry->state = tx_done;
	dev_kfree_skb (tx_skb);
	return 1;
}

static int
mt7687_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(ndev);
	int n;
#ifdef SUPPORT_TCP_WAIT_QUEUE	
if(ip_hdr(skb)->protocol == IPPROTO_TCP)
{
	skb_queue_tail(&host->tx_tcp_wait_q, skb);
	#ifdef DEBUG 
	pr_info("add tcp skb to q\n");
	#endif
}
else
#endif	
	skb_queue_tail (&host->tx_wait_q, skb);

	if ((n=skb_queue_len (&host->tx_tcp_wait_q)) > TX_QUEUE_HIGH_THRESHOLD) {
		if (netif_msg_tx_queued (host))
			printk ("%s: Too much TX(TCP) packets in queue %d\n"
				, __FUNCTION__
				, skb_queue_len (&host->tx_tcp_wait_q));
		netif_stop_queue (ndev);
	}


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


static void
mt7687_rx(struct net_device *dev)
{
	struct sk_buff *skb;
	struct mt7687_spi_host* host = (struct mt7687_spi_host*)netdev_priv(dev);
	struct mt7687_rxinfo rxinfo;

	bool GoodPacket;
	int rxlen;

	do {

		mt7687_read_data(&host->mt_spi, 0, (void*)&rxinfo,sizeof(rxinfo), 0);

		rxlen = le16_to_cpu(rxinfo.len);

		if (netif_msg_rx_status(host)) {
		#ifdef DEBUG  
			dev_info(host->dev, "RX: status %02x, length %04x\n",
				rxinfo.status, rxlen);
		#endif		
		}
		/* Packet Status check */
		if (rxlen < 0x2a) {
			GoodPacket = false;
			if (netif_msg_rx_err(host))
				dev_dbg(host->dev, "RX: Bad Packet (runt)\n");
		}

		if (rxlen > 1536) {
			dev_dbg(host->dev, "RST: RX Len:%x\n", rxlen);
		}

		if((skb = dev_alloc_skb(rxlen+8+4)) != NULL) {
			u8 *pRead;
			
			pRead = (u8 *) skb_put(skb, rxlen);

			mt7687_read_data(&host->mt_spi, 0, (void*)pRead,rxlen+8+4, 1);

			dev->stats.rx_bytes += rxlen;

			skb_reserve(skb, 8+4);
			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb, dev);

			netif_rx(skb);
			dev->stats.rx_packets++;

		}
	}while(0);

}


static int mt7687_handle_d2h_isr (struct mt7687_spi_host* host)
{
	u16 int_status;
	u8 done = 0;
	u16 flag_h2d;
	int timeout;

	 mt7687_get_d2h_flag(&host->mt_spi, &int_status);
	 if((int_status&0xffc0))
	 {
		printk(KERN_INFO "%s: IoT is in invalid status maybe it is down. %d\n",__func__,(int)int_status);
		netif_carrier_off (host->ndev);
	 	return -1;
	 }


	if(int_status & SPIS_D2H_ISR_EVENT)
	{
		u32 link = 0;
		int ret = 0;
	
		ret = mt7687_read_linkstatus(&host->mt_spi, &link);
		mt7687_clear_d2h_flag(&host->mt_spi, SPIS_D2H_ISR_EVENT);	

		if((ret<0 || !(link&4))){
			printk(KERN_INFO "IoT Disconnected. ret = %x  link = %d\n",ret, link);
			netif_carrier_off (host->ndev);	
		}
		else if(link==4)
			printk(KERN_INFO "IoT Connected link = %x\n",link);
		else if(link==0x84)
		{
			printk(KERN_INFO "IoT IP is available.link = %x\n",link);
			netif_carrier_on(host->ndev);
		}		
		else
			printk(KERN_INFO "IoT Unknow link status Link = %x\n",link);
	}

	if(int_status & SPIS_D2H_ISR_PKT_RECV)
	{
		mt7687_rx(host->ndev);
		mt7687_clear_d2h_flag(&host->mt_spi, SPIS_D2H_ISR_PKT_RECV);	
	}

	mt7687_set_h2d_flag(&host->mt_spi, SPIS_H2D_TYPE_ISR_DONE);
	mt7687_activate_swirq (&host->mt_spi);

	timeout = 1000;
	while((mt7687_get_h2d_flag (&host->mt_spi,&flag_h2d)==0) && timeout--)
	{
		if(!(flag_h2d &SPIS_H2D_TYPE_ISR_DONE))
			break;
		msleep(1);
			
	}
	if(!timeout)
	{
		netif_carrier_off (host->ndev);
		printk (KERN_ERR "mt7687_handle_d2h_isr: H2D SPIS_H2D_TYPE_ISR_DONE timeout.\n");
	}
	return done;
}

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

//	if (netif_msg_intr (host))
//		printk ("%s: Interrupt occurred\n", ndev->name);

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
	int sent_time = 0,total_tx_time=0;
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
			if (!mt7687_handle_d2h_isr (host))
				break;
		}

		clear_bit (EVENT_H2D_INTR, &host->flags);

		enable_irq (host->ndev->irq);
	
	}
	
	if (test_bit (EVENT_TX, &host->flags)) {
		int n;
		int ret;
		u32 ready_buf_mask=0;
		u32 dest_buf_id = 0;
#ifdef SUPPORT_TCP_WAIT_QUEUE	
		while ((n = skb_queue_len(&host->tx_tcp_wait_q))) 
		{
		
			ret = mt7687_hard_xmit_tcp (host, dest_buf_id); 
			
			if (!ret)
			{
				break;	
			}
			
			ready_buf_mask |=(1<<dest_buf_id);

			dest_buf_id++;
			
			if(ready_buf_mask==0x00ff)
			{
				mt7687_iot_buf_send(&host->mt_spi, ready_buf_mask, &sent_time );
				total_tx_time =sent_time;
				ready_buf_mask = 0;
				dest_buf_id = 0;
#ifdef DEBUG 
				pr_info("add tcp skb sent\n");
#endif				
			//	if(n<8)
					break;
			}					

			
		}
	
		if(ready_buf_mask)
		{
		
			mt7687_iot_buf_send(&host->mt_spi, ready_buf_mask, &sent_time );
			total_tx_time+=sent_time;			
			dest_buf_id = 0;		
			ready_buf_mask = 0;
#ifdef DEBUG 			
				pr_info("add tcp skb sent 2\n");			
#endif				
		}		
#endif
		if(total_tx_time>10)
		{
			netif_queue_stopped (host->ndev);

		}
		
		while ((n = skb_queue_len(&host->tx_wait_q))) {
			ret = mt7687_hard_xmit (host, dest_buf_id); 
			if (!ret)
			{
				break;	
			}
			
			ready_buf_mask |=(1<<dest_buf_id);

			dest_buf_id++;
			
			if(ready_buf_mask==0x00ff)
			{
			
				mt7687_iot_buf_send(&host->mt_spi, ready_buf_mask, &sent_time  );
				total_tx_time =sent_time;			

				ready_buf_mask = 0;
				break;
			}			


			
		}

		if(ready_buf_mask)
		{
		
			mt7687_iot_buf_send(&host->mt_spi, ready_buf_mask, &sent_time  );
			total_tx_time +=sent_time;
		}		
		if(total_tx_time>10)
		{
			netif_queue_stopped (host->ndev);

		}		
		clear_bit (EVENT_TX, &host->flags);

		total_queue_len = (skb_queue_len(&host->tx_wait_q) +skb_queue_len(&host->tx_tcp_wait_q)) ;
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
			mt7687_set_wifi(&host->mt_spi, host->pWifiSetting);		
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
	int i;
	unsigned long irq_flag = IRQF_SHARED;


	netif_carrier_off (host->ndev);

	down (&host->spi_lock);

	mt7687_clear_d2h_flag(&host->mt_spi, 0x3f);
	mt7687_clear_h2d_flag(&host->mt_spi, 0);

	if((ret = mt7687_read_spis_info_addr(&host->mt_spi, &host->mt_spi.info_addr ))<0)
	{
		ret = -ENXIO;
		goto err;
	}

	if((host->mt_spi.info_addr&0xfff00000)!=0x20000000)
		printk(KERN_INFO "mt7687_open: INFO Addr = 0x%08x abnormal\n",host->mt_spi.info_addr);
	else {
#ifdef DEBUG	  
		printk(KERN_INFO "mt7687_open: INFO Addr = 0x%08x \n",host->mt_spi.info_addr);
#endif		
	}
	if((ret = mt7687_read_spis_rd_addr(&host->mt_spi, &host->mt_spi.rd_addr))<0)
	{
		ret = -ENXIO;
		goto err;
	}
	if((host->mt_spi.rd_addr&0xfff00000)!=0x20000000)
		printk(KERN_INFO "mt7687_open: RD Addr = 0x%08x abnormal\n",host->mt_spi.rd_addr);

	if((ret = mt7687_read_spis_wr_addr(&host->mt_spi, host->mt_spi.wr_addr, NUM_OF_TX_Q))<0)
	{
		ret = -ENXIO;
		goto err;
	}

	for(i=0;i<8/*NUM_OF_TX_Q*/;++i)
	{
		if((host->mt_spi.wr_addr[i]&0xfff00000)!=0x20000000)
			printk(KERN_INFO "mt7687_open: WR Addr = 0x%08x abnormal\n",host->mt_spi.wr_addr[i]);	
	}
#ifdef CONFIG_SUPPORT_2WAY_SPI
	rx_mode = 1;
	iot_set_rx_mode(&host->mt_spi, rx_mode);
#endif
	mt7687_set_mac_addr (ndev);

	irq_flag|=IRQF_TRIGGER_HIGH;

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
#if 0	// for testing
	while(0)
	{
		u8 tx_buf[1600] ;
		u8 rx_buf[1600] ;

		extern int spi_rw_dma(struct spi_device *spi,
				const void *txbuf, unsigned n_tx,
				void *rxbuf, unsigned n_rx);
		int i ;
		tx_buf[0] = 0x11;
		tx_buf[1] = 0x22;
		tx_buf[2] = 0x33;
		tx_buf[3] = 0x44;
		tx_buf[4] = 0x55;
		tx_buf[5] = 0x66;
		tx_buf[6] = 0x77;
		tx_buf[7] = 0x88;
		tx_buf[8] = 0x99;
		tx_buf[9] = 0xaa;
		tx_buf[10] = 0xbb;
		tx_buf[11] = 0xcc;
		tx_buf[12] = 0xdd;
		tx_buf[13] = 0xee;
		tx_buf[14] = 0xff;
		tx_buf[15] = 0x00;
		tx_buf[16] = 0x16;
		tx_buf[17] = 0x17;
		tx_buf[18] = 0x18;
		tx_buf[19] = 0x19;

		ret = spi_rw_dma (m_rx_spi, tx_buf, 8, (u8 *)rx_buf, 1526-16);	//TX 12 RX
		for(i=0;i<20;++i)
			pr_info("%d: %x ",i, rx_buf[i]);
		pr_info("spi tx rx\n");
	}
#endif
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

#if 0//def CONFIG_SUPPORT_2WAY_SPI
	if(mt7687_spim_handler_task)
	{
	//	kthread_stop(mt7687_spim_handler_task);
		wait_for_completion(mt7687_spim_handler_task);
	}	
#endif

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
	.ndo_set_rx_mode = mt7687_set_multicast,
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
	
//	INIT_WORK(&host->mt7687_rx_work, mt7687_rx_work);

	host->mt7687_work_queue = 
			create_singlethread_workqueue ("mt7687_work");

//	host->mt7687_rx_work_queue = 
//			create_singlethread_workqueue ("mt7687_rx_work");

	sema_init (&host->spi_lock,1);
	
	skb_queue_head_init(&host->tx_tcp_wait_q);
	skb_queue_head_init(&host->tx_wait_q);
//	skb_queue_head_init(&host->rx_wait_q);
	
	ndev->features &= ~NETIF_F_HW_CSUM;
	ndev->hard_header_len += (TX_OVERHEAD + 4);

	ret = register_netdev(ndev);
	if (!ret)
	{


#ifdef CONFIG_SUPPORT_2WAY_SPI
	if(mt7687_spim_handler_task==0 )
	{
		mt7687_spim_handler_task = kthread_run(mt7687_spi_rx_handler_task, ndev, "%s",	"mt7687_spim_rx_handler_task");
		if (IS_ERR(mt7687_spim_handler_task)) {
			pr_warning("MT7687: Failed to run thread %s\n",
					"mt7687_spim_rx_handler_task");

			ret = PTR_ERR(mt7687_spim_handler_task);
			mt7687_spim_handler_task = NULL;
			return ret;

	//		goto err;
		}
		wake_up_process(mt7687_spim_handler_task);
	}	
#endif

	
		return ret;
	}
	
	destroy_workqueue(host->mt7687_work_queue);

	free_netdev(ndev);

	return ret;
}
#if CONFIG_SUPPORT_2WAY_SPI
extern uint AIT_GPIO_PSPI_S2M_IRQ;
static int __devinit mt7687_rx_spi_probe (struct spi_device *spi)
{
	int ret;
	pr_info("mt7687_rx_spi_probe: %s\n",spi->modalias);
	if(strcmp(spi->modalias, "spidev-mt7687-slave")==0)
	{
		struct iot_spi_platform_data *spi_data = spi->dev.platform_data;
		m_rx_spi = spi;

		pr_info("mt7687_rx_spi_probe: s2m_irq_gpio = %d\n",spi_data->s2m_irq_gpio);		
		ret = gpio_request_one(spi_data->s2m_irq_gpio,GPIOF_DIR_OUT|GPIOF_INIT_LOW,"SPIS_READY");
		if (ret) {
			dev_err(&spi->dev, "ret = %d. Failed to request GPIO %d\n",ret, spi_data->s2m_irq_gpio);
			return -1;
		}

		return 0;
	}	
		return -1;	
}
#endif
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
//	destroy_workqueue (host->mt7687_rx_work_queue);	

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
#if CONFIG_SUPPORT_2WAY_SPI
static struct spi_driver mt7687_spi_slave_driver = {
	.driver = {
		.name = "spidev-mt7687-slave",
		.owner = THIS_MODULE,
	},
	.probe = mt7687_rx_spi_probe,

};
#endif


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
#if CONFIG_SUPPORT_2WAY_SPI
	spi_register_driver(&mt7687_spi_slave_driver);
#endif
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
