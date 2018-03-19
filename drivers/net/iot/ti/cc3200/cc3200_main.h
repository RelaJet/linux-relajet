/*  MT7687 SPI IoT Linux driver */

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

#ifndef _MT7687_MAIN_H
#define _MT7687_MAIN_H

/* INCLUDE FILE DECLARATIONS */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <asm/io.h>
#include <asm/dma.h>


#include "iot.h"

#include "cc3200_spi.h"

#define DRV_NAME	"CC3200-SPI"
#define ADP_NAME	"CC3200 SPI Ethernet Adapter"
#define DRV_VERSION	"1.0.0"
#define PFX DRV_NAME	": "

#define TX_QUEUE_HIGH_THRESHOLD		100		/* Tx queue high water mark */
#define TX_QUEUE_LOW_THRESHOLD		40		/* Tx queue low water mark */

#define RX_QUEUE_HIGH_THRESHOLD		100		/* Tx queue high water mark */

#define CC3200_WATCHDOG_PERIOD	( 5*HZ)

#define MT7687_WATCHDOG_RESTART	7

#define TX_OVERHEAD			6
#define TX_EOP_SIZE			4

	
enum watchdog_state {
	chk_link = 0,
	chk_cable,
	ax_nop,
};

typedef struct cc3200_spi_host{

	struct resource		*addr_res;   /* resources found */
	struct resource		*addr_req;   /* resources requested */
	struct resource		*irq_res;

	struct spi_device	*spi;
	struct device	*dev;

	struct net_device	*ndev;

	struct net_device_stats	stats;

	struct timer_list	watchdog;
	enum watchdog_state	w_state;
	size_t			w_ticks;

	struct work_struct	work;
	struct workqueue_struct *work_queue;

	//struct work_struct	rx_work;
	//struct workqueue_struct *cc3200_rx_work_queue;
	
	struct tasklet_struct	bh;

	struct semaphore	spi_lock;

	struct sk_buff_head	tx_tcp_wait_q;
	struct sk_buff_head	tx_wait_q;
	struct sk_buff_head	rx_wait_q;	

	struct cc3200_data	spi_priv_data;

	int			msg_enable;

	u16			seq_num;

	u16			wol;

	u8			checksum;

	u8			plat_endian;
		#define PLAT_LITTLE_ENDIAN	0
		#define PLAT_BIG_ENDIAN		1

	unsigned long		flags;
#		define EVENT_H2D_INTR		1
#		define EVENT_TX				2
#		define EVENT_RX				3
#		define EVENT_SET_MULTI		4
#		define EVENT_WATCHDOG		8
#		define EVENT_SET_WIFI		0x10
#		define EVENT_SEND_CMD		5
#		define EVENT_WAIT_CMD_RESP 6


	iot_wifi_setting_t* pWifiSetting;
	int gpio_spi_busy;
} IOT_PRIVATE, *PIOT_PRIVATE;

struct cc3200_rxinfo {
	u16	RxPktReady;
	u16	status;
	__le16	len;
	u16 reserved;
} __packed;

enum skb_state {
	illegal = 0,
	tx_done,
	rx_done,
	rx_err,
};

struct skb_data;

struct skb_data {
	enum skb_state state;
	struct net_device *ndev;
	struct sk_buff *skb;
	size_t len;
	dma_addr_t phy_addr;
};



/* Tx headers structure */
struct tx_sop_header {
	/* bit 15-11: flags, bit 10-0: packet length */
	u16 flags_len;
	/* bit 15-11: sequence number, bit 11-0: packet length bar */
	u16 seq_lenbar; 
} __attribute__((packed));

struct tx_segment_header {
	/* bit 15-14: flags, bit 13-11: segment number, bit 10-0: segment length */
	u16 flags_seqnum_seglen;
	/* bit 15-14: end offset, bit 13-11: start offset */
	/* bit 10-0: segment length bar */
	u16 eo_so_seglenbar;
} __attribute__((packed));

struct tx_eop_header {
	/* bit 15-11: sequence number, bit 10-0: packet length */
	u16 seq_len;
	/* bit 15-11: sequence number bar, bit 10-0: packet length bar */
	u16 seqbar_lenbar;
} __attribute__((packed));

struct tx_pkt_info {
	struct tx_sop_header sop;
	struct tx_segment_header seg;
	struct tx_eop_header eop;
	u16 pkt_len;
	u16 seq_num;
} __attribute__((packed));



#define TX_HEAD_SIZE			sizeof(struct iot_pkt_head_t)
#endif


