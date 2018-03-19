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

#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/tcp.h>
#include <asm/checksum.h>

#include "cc3200_main.h"
#include "cc3200_ioctl.h"
#include "cc3200_spi.h"

//#define CC3200_DEBUG 1

struct spi_rx_pkt_t{
	u16 buf[SPI_TX_COUNT];
	int len;
} __packed;



struct spi_stat_report_t{
	struct iot_pkt_head_t hdr;
	u8 tx_buf_cnt;
} __packed;

typedef enum iot_pkt_type{
	IOT_PKT_TYPE_ETHERNET,
	IOT_PKT_TYPE_COMMAND,
	IOT_PKT_TYPE_STATISTIC,
	IOT_PKT_TYPE_ERROR = -1,	
}iot_pkt_type_t;


#define CC3200_DRV_NAME	"CC3200-IoT"
#define AIT_GPIO_ETHERNET_RESET MMPF_PIO_REG_GPIO115

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

struct tasklet_struct rx_handler_tasklet;
struct list_head	spi_rx_pkt_list;

extern struct completion	spi_cmd_wait_event;

static int cmd_seq = 0;
static unsigned long cmd_resp_timeout = 0;
struct iot_cmd_t g_iot_command;
struct iot_cmd_resp_t g_iot_command_resp;

int is_if_close = 0;

static void cc3200_watchdog (struct cc3200_spi_host* host)
{
	unsigned long time_to_chk = CC3200_WATCHDOG_PERIOD;

	if (time_to_chk)
		mod_timer (&host->watchdog, jiffies + time_to_chk);
}

static void cc3200_watchdog_timer (unsigned long arg)
{
	struct net_device *ndev = (struct net_device *)(arg);
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	set_bit (EVENT_WATCHDOG, &host->flags);
		
	queue_work (host->work_queue, &host->work);
}

static void cc3200_set_multicast (struct net_device *ndev)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	set_bit (EVENT_SET_MULTI, &host->flags);
	queue_work (host->work_queue, &host->work);
}

static void __attribute__ ((unused)) cc3200_set_mac_addr (struct net_device *ndev)
{

	struct cc3200_spi_host*  host = (struct cc3200_spi_host* )netdev_priv(ndev);	
	struct iot_cmd_t* spi_cmd = &g_iot_command;//&kmalloc(sizeof(struct iot_cmd_t), GFP_KERNEL);

	spi_cmd->cmd = SPIS_CMD_GET_MAC;
	spi_cmd->len = ETH_ALEN*2;

	set_bit (EVENT_SEND_CMD, &host->flags);
	queue_work (host->work_queue, &host->work);

	wait_for_completion_timeout(&spi_cmd_wait_event, 2*HZ);

	if(g_iot_command_resp.cmd){//	if(!list_empty(&spi_cmd_resp_list)) {	
		struct iot_cmd_resp_t* spi_cmd_resp = &g_iot_command_resp;// list_first_entry(&spi_cmd_resp_list,  struct iot_cmd_resp_t, list);


		pr_info("spi_cmd_resp->cmd = %x",spi_cmd_resp->cmd);
		pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
		memcpy(ndev->dev_addr, spi_cmd_resp->data, ETH_ALEN*2);
		{
			char* p;
			p =ndev->dev_addr;
			pr_info("CC3200 MAC: %02x:%02x:%02x:%02x:%02x:%02x:" ,p[0], p[1], p[2], p[3], p[4], p[5]);
			pr_info("CC3200 MAC Checking: %02x:%02x:%02x:%02x:%02x:%02x:" ,p[6], p[7], p[8], p[9], p[10], p[11]);	
		}

		g_iot_command_resp.cmd = 0;
	}
	kfree(spi_cmd);


	return;
}


static int cc3200_set_mac_address (struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	return 0;
}

unsigned long pre_sent_size = 0;
unsigned long  sent_data_size = 0;
unsigned long sent_num_pkts = 0;
unsigned long t=0;
static struct sk_buff *
cc3200_fill_header(struct net_device *ndev, struct sk_buff_head *q)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
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
	if(t==0 ||time_after(jiffies, t+30*HZ))
	{
		t = jiffies;
		pr_info("Sent %d packets, %d Bytes %d bps\r\n",sent_num_pkts, sent_data_size,((sent_data_size-pre_sent_size)*8)/30);
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
	info->pkt_len = pkt_len;

	if ((!skb_cloned(skb)) &&
	    (headroom >= (TX_HEAD_SIZE)) &&
	    (tailroom >= (padlen))) {

		info->seq_num = seq_num;
		memcpy(info->pkt_type,"ETH",4);
		memcpy (skb_push (skb, TX_HEAD_SIZE), (void*)info, TX_HEAD_SIZE);

		/* Make 32-bit aligment */
		skb_put (skb, padlen);

		/* EOP header */
		//memcpy (skb_put (skb, TX_EOP_SIZE), &end_mark, TX_EOP_SIZE);

		tx_skb = skb;
		skb_unlink(skb, q);

	} else {

		tx_skb = alloc_skb (tol_len, GFP_KERNEL);
		if (!tx_skb)
			return NULL;

		info->seq_num = seq_num;
		memcpy(info->pkt_type,"ETH",4);

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

static void cc3200_rx(struct cc3200_spi_host* host, __u8* pRxPktBuf)
{

	struct sk_buff *skb;
	struct net_device *ndev = host->ndev;
	struct iot_pkt_head_t* pktHdr =(struct iot_pkt_head_t*) pRxPktBuf;
	int rxlen;
	int seq;

	seq = pktHdr->seq_num;
	rxlen = pktHdr->pkt_len ;
	
	if (netif_msg_rx_status(host)) {
		dev_info(host->dev, "RX: length %04x\n",rxlen);
	}
	/* Packet Status check */
	if (rxlen < 0x2a) {
		if (netif_msg_rx_err(host))
			dev_dbg(host->dev, "RX: Bad Packet (runt)\n");
		return;
	}

	if (rxlen > 1536) {
		dev_dbg(host->dev, "RST: RX Len:%x\n", rxlen);
		return;		
	}

	if((skb = dev_alloc_skb(rxlen)) != NULL) {
		u8 *pRead, *pEthHdr;

		pRead = (u8 *) skb_put(skb, rxlen);

		pEthHdr = (u8*)pktHdr;
		pEthHdr+=sizeof(struct iot_pkt_head_t);
		memcpy(pRead,pEthHdr , rxlen);

		ndev->stats.rx_bytes += rxlen;

		skb->protocol = eth_type_trans(skb, ndev);

		netif_rx(skb);
		ndev->stats.rx_packets++;
	}
}


static int
cc3200_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	
	int n;


	skb_set_network_header(skb, ETH_HLEN);	

	if(ip_hdr(skb)->protocol == IPPROTO_TCP)
	{
		skb_queue_tail(&host->tx_tcp_wait_q, skb);
			
		#ifdef DEBUG 
		pr_info("add tcp skb to q\n");
		#endif
	}
	else
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
	queue_work (host->work_queue, &host->work);
	
	return NETDEV_TX_OK;

}


static irqreturn_t cc3200_interrupt  (int irq, void *dev_instance)
{
	struct net_device *ndev = dev_instance;
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);

	if (ndev == NULL) {
		printk (KERN_ERR "irq %d for unknown device.\n", irq);
		return IRQ_RETVAL (0);
	}
	if (!netif_running(ndev))
		return -EINVAL;

	disable_irq_nosync (irq);
	
	set_bit (EVENT_H2D_INTR, &host->flags);
	queue_work (host->work_queue, &host->work);

	return IRQ_HANDLED;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: work
 * Purpose:
 * ----------------------------------------------------------------------------
 */

static struct spi_rx_pkt_t *spi_rx_ptk;

static void cc3200_work (struct work_struct *work)
{
	int total_queue_len;
	static int spi_tx_buf_busy = 0;
			
	struct cc3200_spi_host* host = 
			container_of(work, struct cc3200_spi_host, work);
	struct net_device *ndev = host->ndev;
			
	down (&host->spi_lock);

	if (test_bit (EVENT_WATCHDOG, &host->flags)) {
		cc3200_watchdog (host);

		clear_bit (EVENT_WATCHDOG, &host->flags);
	}

	if (test_bit (EVENT_SET_MULTI, &host->flags)) {

	printk(KERN_INFO "EVENT_SET_MULTI\n");

		clear_bit (EVENT_SET_MULTI, &host->flags);
	}


	if (test_bit (EVENT_SEND_CMD, &host->flags)) {

		struct iot_cmd_t* cmd;
		{
			void* tx_ptr;
			char* pRxBuf;
			
			cmd = &g_iot_command;//list_first_entry(&spi_cmd_list,  struct iot_cmd_t, list);

			memcpy(cmd->PktHdr.pkt_type,"CMD", 4);
			cmd->PktHdr.seq_num = cmd_seq++;
			cmd->PktHdr.pkt_len= cmd->len+4;
				
			tx_ptr = &cmd->PktHdr;
			
			pRxBuf = (char*)spi_rx_ptk->buf;

			spi_rw_dma(&host->spi_priv_data,tx_ptr, SPI_DMA_BYTES, pRxBuf, SPI_DMA_BYTES);

			if(!strncmp(pRxBuf, "ETH", 4))
			{
				struct iot_pkt_head_t* pktHdr =(struct iot_pkt_head_t*) pRxBuf;			
				if(pktHdr->seq_num != 0xffff)
				{
					cc3200_rx(host,(__u8*) pktHdr);
				}
	
			}	

			set_bit (EVENT_WAIT_CMD_RESP, &host->flags);			
		}
		clear_bit (EVENT_SEND_CMD, &host->flags);	

		cmd_resp_timeout=jiffies;
	}

	if (test_bit (EVENT_TX, &host->flags)) {
		int total_queue_len;
		int i = 40;

		while((i--)&&((skb_queue_len(&host->tx_wait_q))||(skb_queue_len(&host->tx_tcp_wait_q))))
		{
			struct sk_buff *tx_skb;

			char* pRxBuf;
			pRxBuf = (char*)spi_rx_ptk->buf;
			
			if(!spi_tx_buf_busy)
			{
				if(skb_queue_len(&host->tx_tcp_wait_q))
					tx_skb = cc3200_fill_header (host->ndev, &host->tx_tcp_wait_q);
				else
					tx_skb = cc3200_fill_header (host->ndev, &host->tx_wait_q);
			
				spi_rw_dma(&host->spi_priv_data,tx_skb->data, SPI_DMA_BYTES, pRxBuf, SPI_DMA_BYTES);
				dev_kfree_skb (tx_skb);			
			}
			else
			{
				u16 txbuf[6] = {0};
				spi_rw_dma(&host->spi_priv_data, txbuf, SPI_DMA_BYTES, pRxBuf, SPI_DMA_BYTES);
				set_bit (EVENT_TX, &host->flags);	
				queue_work (host->work_queue, &host->work);	
			}
				
			if(!strncmp(pRxBuf, "ETH", 4))
			{
				struct iot_pkt_head_t* pktHdr = (struct iot_pkt_head_t* )pRxBuf;		
				
				if(pktHdr->seq_num == 0xffff)
				{
					spi_pkt_iot_info_t* pIoT_StatusPkt = (spi_pkt_iot_info_t* )pRxBuf;
					
					if(pIoT_StatusPkt->status.available_spi2wifi_buf<2)
					{
						netif_stop_queue (ndev);										
						spi_tx_buf_busy = 1;
					}
					else
						spi_tx_buf_busy = 0;
					
					break;					
				}
				else
				{
					cc3200_rx(host, (__u8*)pktHdr);
				}
			}
			else if(!strncmp(pRxBuf, "CMD", strlen("CMD")))
			{
				struct iot_cmd_resp_t* cmd_resp  = &g_iot_command_resp;

				memcpy(&cmd_resp->PktHdr, pRxBuf ,sizeof(struct iot_cmd_resp_t));
				complete(&spi_cmd_wait_event);
				clear_bit (EVENT_WAIT_CMD_RESP, &host->flags);					
				break;
			}	
			
			if(!spi_tx_buf_busy)
			{
				total_queue_len = skb_queue_len(&host->tx_wait_q)+skb_queue_len(&host->tx_tcp_wait_q);
			
				if (netif_queue_stopped (host->ndev) && 
				    ((total_queue_len) < TX_QUEUE_LOW_THRESHOLD))
				{
					netif_wake_queue (host->ndev);
				}
			}	

		}
		
	}/* EVENT_TX */

	if(test_bit (EVENT_H2D_INTR, &host->flags))
	{
		int max_rx_cnt = 16;
		while(max_rx_cnt--)
		{
			char* pRxBuf;
			u16 txbuf[6] = {0};

			pRxBuf = (char*)spi_rx_ptk->buf;

			if(-ETIME==spi_rw_dma(&host->spi_priv_data,txbuf, SPI_DMA_BYTES, pRxBuf, SPI_DMA_BYTES))
			{	
				break;
			}

			if(!strncmp(pRxBuf, "ETH", 4))
			{

				struct iot_pkt_head_t* pktHdr = (struct iot_pkt_head_t* )pRxBuf;		
				
				if(pktHdr->seq_num == 0xffff)
				{
					spi_pkt_iot_info_t* pIoT_StatusPkt = (spi_pkt_iot_info_t* )pRxBuf;
					

					if(pIoT_StatusPkt->status.available_spi2wifi_buf<2){
						netif_stop_queue (ndev);										
						spi_tx_buf_busy = 1;
					}
					else
					{
						spi_tx_buf_busy = 0;
					}
					break;					
				}
				else
				{
					cc3200_rx(host, (__u8*)pktHdr);
				}
			}
			else if(!strncmp(pRxBuf, "CMD", strlen("CMD")))
			{
				struct iot_cmd_resp_t* cmd_resp  = &g_iot_command_resp;//kmalloc(sizeof(struct iot_cmd_resp_t), GFP_KERNEL);
		
				memcpy(&cmd_resp->PktHdr, pRxBuf,sizeof(struct iot_cmd_resp_t));
				complete(&spi_cmd_wait_event);
				clear_bit (EVENT_WAIT_CMD_RESP, &host->flags);					
			}
		}

		clear_bit (EVENT_H2D_INTR, &host->flags);
		if(!is_if_close)
			enable_irq (host->ndev->irq);

	}/* EVENT_H2D_INTR */

	if(test_bit (EVENT_WAIT_CMD_RESP, &host->flags))
	{
			
		while(1)
		{
			char* pRxBuf;
			u16 txbuf[6] = {0};

			pRxBuf = (char*)spi_rx_ptk->buf;

			if (time_after(jiffies, cmd_resp_timeout + 2 * HZ)) {
				printk(KERN_ERR "%s: CMD response timeout.\n", __func__);

				clear_bit (EVENT_WAIT_CMD_RESP, &host->flags);						
				cmd_resp_timeout = 0;			
				break ; 
			}
			
			if(-ETIME==spi_rw_dma(&host->spi_priv_data,txbuf, SPI_DMA_BYTES, pRxBuf, SPI_DMA_BYTES))
			{	
				break;
			}

			if(!strncmp(pRxBuf, "ETH", 4))
			{
				struct iot_pkt_head_t* pktHdr = (struct iot_pkt_head_t* )pRxBuf;				
				if(pktHdr->seq_num == 0xffff)
				{
					spi_pkt_iot_info_t* pIoT_StatusPkt = (spi_pkt_iot_info_t* )pRxBuf;
					if(pIoT_StatusPkt->status.available_spi2wifi_buf<2)
						spi_tx_buf_busy = 1;
					else
						spi_tx_buf_busy = 0;
		
					break; 
				}
				else
				{
					cc3200_rx(host, (__u8*)pktHdr);
				}
			}
			else if(!strncmp(pRxBuf, "CMD", strlen("CMD")))
			{
				struct iot_cmd_resp_t* cmd_resp  = &g_iot_command_resp;//kmalloc(sizeof(struct iot_cmd_resp_t), GFP_KERNEL);

				memcpy(&cmd_resp->PktHdr, pRxBuf ,sizeof(struct iot_cmd_resp_t));
				complete(&spi_cmd_wait_event);
				clear_bit (EVENT_WAIT_CMD_RESP, &host->flags);					
				break;
			}		
		}

		if(test_bit (EVENT_WAIT_CMD_RESP, &host->flags))
			queue_work (host->work_queue, &host->work);
	}/* EVENT_H2D_INTR */

	total_queue_len = skb_queue_len(&host->tx_wait_q)+skb_queue_len(&host->tx_tcp_wait_q);
	if(total_queue_len)
	{
		set_bit (EVENT_TX, &host->flags);
		queue_work (host->work_queue, &host->work);

		if (netif_queue_stopped (host->ndev) && 
		    ((total_queue_len) < TX_QUEUE_LOW_THRESHOLD))
		{
			netif_wake_queue (host->ndev);
		}
		
	}
	else
	{
		netif_wake_queue (host->ndev);	
		clear_bit (EVENT_TX, &host->flags);
	}

	up (&host->spi_lock);	

}


static struct net_device_stats *cc3200_get_stats(struct net_device *ndev)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	return &host->stats;
}


static int
cc3200_open(struct net_device *ndev)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	int ret;
	unsigned long irq_flag = IRQF_SHARED;
	extern int ait_spi_xfer_fifo(struct spi_device *spidev, struct spi_transfer* xfer);

	
	netif_carrier_off (host->ndev);

	down (&host->spi_lock);

	irq_flag|=IRQF_TRIGGER_HIGH;

	ret = request_irq (ndev->irq, cc3200_interrupt ,
			irq_flag, ndev->name, ndev);
	if (ret) {
		printk (KERN_ERR "%s: unable to get IRQ %d (errno=%d).\n",
				ndev->name, ndev->irq, ret);
		goto err;
	}

	is_if_close = 0;

	host->seq_num = 0x1f;

	netif_start_queue (ndev);

	if (netif_msg_hw (host)) {
#ifdef DEBUG	  
		printk ("\nDump all MAC registers after initialization:\n");
		cc3200_dump_regs (&host->spi_priv_data);
#endif		
	}

	spi_message_init(&host->spi_priv_data.rx_msg);

	up (&host->spi_lock);

	init_timer (&host->watchdog);
	host->watchdog.function = &cc3200_watchdog_timer;
	host->watchdog.expires = jiffies + CC3200_WATCHDOG_PERIOD;
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
static void cc3200_free_skb_queue (struct sk_buff_head *q)
{
	struct sk_buff *skb;
	printk(KERN_INFO "%s.\n",__func__);
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
cc3200_close(struct net_device *ndev)
{
	struct cc3200_spi_host* host = (struct cc3200_spi_host*)netdev_priv(ndev);
	is_if_close = 1;

	if (netif_msg_ifdown(host))
		dev_dbg(host->dev, "shutting down %s\n", ndev->name);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	del_timer_sync (&host->watchdog);
	free_irq (ndev->irq, ndev);

	down (&host->spi_lock);
	cc3200_free_skb_queue (&host->tx_wait_q);
	cc3200_free_skb_queue (&host->tx_tcp_wait_q);	
	
	up (&host->spi_lock);

	return 0;
}

static int cc3200_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	return iot_priv_ioctl(ndev, cmd, req->ifr_data);
}


static const struct net_device_ops cc3200_netdev_ops = {
	.ndo_open		= cc3200_open,
	.ndo_stop		= cc3200_close,
	.ndo_start_xmit		= cc3200_start_xmit,
	.ndo_get_stats		= cc3200_get_stats,
	.ndo_set_rx_mode = cc3200_set_multicast,
	.ndo_do_ioctl		= cc3200_ioctl,
	.ndo_set_mac_address	= cc3200_set_mac_address,
};


static int __devinit cc3200_probe (struct spi_device *spi)
{
	struct net_device *ndev;
	struct cc3200_spi_host* host;
	int ret;
	struct iot_spi_platform_data* p_iotdata = spi->dev.platform_data;

	char macaddr[ETH_ALEN] = {0x98, 0x7b, 0xf3, 0x13, 0x5f, 0x6e};
	u16 txbuf[6] = {0};

 	ndev = alloc_etherdev (sizeof (*host));
	if (!ndev) {
		printk(KERN_ERR
		       "CC3200 SPI: Could not allocate ethernet device\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 16;

	host = (struct cc3200_spi_host*)netdev_priv(ndev);
	memset (host, 0, sizeof (*host));

	dev_set_drvdata(&spi->dev, host);
	host->dev = &spi->dev;

	host->spi = spi;
	host->spi_priv_data.spi = spi;	
	ndev->irq = spi->irq;
	
	printk(version);

	host->msg_enable =  msg_enable;

	ndev->netdev_ops	= &cc3200_netdev_ops;
	ndev->ethtool_ops	= &iot_ethtool_ops;


	memcpy(ndev->dev_addr, macaddr, ETH_ALEN);
	
	host->ndev = ndev;

	ret = gpio_request_one(p_iotdata->s2m_spi_busy_gpio, GPIOF_DIR_IN, "SPIS_READY");
	if (ret) {
		dev_err(&spi->dev, "ret = %d. Failed to request GPIO %d\n",ret, p_iotdata->s2m_spi_busy_gpio);
		return -1;
	}


	host->gpio_spi_busy = p_iotdata->s2m_spi_busy_gpio;
	
	if (netif_msg_probe (host))
		printk(KERN_INFO PFX "addr 0x%lx, irq %d, ",
		       ndev->base_addr, ndev->irq);


	INIT_WORK(&host->work, cc3200_work);
	host->work_queue = 
			create_singlethread_workqueue ("work");

	sema_init (&host->spi_lock,1);

	INIT_LIST_HEAD(&spi_rx_pkt_list);
	init_completion(&spi_cmd_wait_event);
	
	skb_queue_head_init(&host->tx_wait_q);
	skb_queue_head_init(&host->tx_tcp_wait_q);

	ndev->features &= ~NETIF_F_HW_CSUM;
	ndev->hard_header_len += (TX_HEAD_SIZE + 4);

	spi_rx_ptk = kmalloc(sizeof(struct spi_rx_pkt_t), GFP_KERNEL);
//	memset(spi_rx_ptk, 0 , 6);
//Send a dummy spi packet.
//	spi_rw_dma(&host->spi_priv_data,txbuf, SPI_DMA_BYTES, 0, SPI_DMA_BYTES);		//DMA_COUNT can be reduce to MTU
	
	ret = register_netdev(ndev);
	if (!ret)
	{
		return ret;
	}
	
	destroy_workqueue(host->work_queue);

	free_netdev(ndev);

	printk(KERN_INFO "%s.ret = %d\n",__func__,ret);

	return ret;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: cc3200_suspend
 * Purpose: Device suspend handling function
 * ----------------------------------------------------------------------------
 */
static int
cc3200_suspend (struct spi_device *spi, pm_message_t mesg)
{
	struct cc3200_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "cc3200_suspend.\n");
	if (!ndev || !netif_running (ndev))
		return 0;

	netif_device_detach (ndev);

	netif_stop_queue (ndev);
	return 0;
}


static int
cc3200_resume (struct spi_device *spi)
{

	struct cc3200_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;

	printk(KERN_INFO "cc3200_resume.\n");

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
static int __devexit cc3200_remove (struct spi_device *spi)
{
	struct cc3200_spi_host* host = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = host->ndev;
		
	destroy_workqueue (host->work_queue);

	unregister_netdev(ndev);

	free_netdev(ndev);
	kfree(spi_rx_ptk);
	
	printk(KERN_INFO "cc3200-spi: released and freed device\n");

	return 0;
}

static struct spi_driver cc3200_spi_driver = {
	.driver = {
		.name = "spidev-iot-cc3200",
		.owner = THIS_MODULE,
	},
	.probe = cc3200_probe,
	.remove = __devexit_p(cc3200_remove),
	.suspend = cc3200_suspend,
	.resume = cc3200_resume,
};


/*
 * ----------------------------------------------------------------------------
 * Function Name: cc3200_spi_init
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static __init int cc3200_spi_init(void)
{

	printk(KERN_INFO "Register CC3200 SPI Net Driver.\n");
	spi_register_driver(&cc3200_spi_driver);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: cc3200_spi_exit
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static __exit void cc3200_spi_exit(void)
{
	spi_unregister_driver(&cc3200_spi_driver);
}

module_init(cc3200_spi_init);
module_exit(cc3200_spi_exit);


MODULE_AUTHOR("Vincent@Mstar");
MODULE_DESCRIPTION("CC3200 SPI IoT driver");
MODULE_LICENSE("GPL");
