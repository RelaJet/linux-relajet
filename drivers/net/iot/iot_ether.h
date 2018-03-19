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

#include "iot_ctrl.h"

/* NAMING CONSTANT AND TYPE DECLARATIONS */
// These identify the driver base version and may not be removed.
#define DRV_NAME	"spidev-iot-eth"
#define ADP_NAME	"IOT SPI Ethernet Adapter"
#define DRV_VERSION	"1.0.0"

#define TX_QUEUE_HIGH_THRESHOLD		100		/* Tx queue high water mark */
#define TX_QUEUE_LOW_THRESHOLD		20		/* Tx queue low water mark */

#define RX_QUEUE_HIGH_THRESHOLD		100		/* Tx queue high water mark */

#define MT7687_WATCHDOG_PERIOD	(1 * HZ)

#define MT7687_WATCHDOG_RESTART	7

#define TX_OVERHEAD			6
#define SPIM_HEADER_SIZE			4
#define IOT_RX_INFO_SIZE (sizeof(struct iot_rxinfo))

struct iot_data {
	struct spi_device	*spi;
	struct spi_message	rx_msg;
	struct spi_transfer	spi_rx_xfer[2];
	u8			cmd_buf[6];
	u8			comp;
#define NUM_OF_TX_Q 16
};


typedef struct iot_spi_host{

//	struct resource		*addr_res;   /* resources found */
//	struct resource		*addr_req;   /* resources requested */
//	struct resource		*irq_res;

	struct spi_device	*spi;
	struct device	*dev;

	struct net_device	*ndev;
	struct net_device	*ndev_eth1;	

	struct net_device_stats	stats;

	struct timer_list	watchdog;

	struct work_struct	iot_eth_work;
	struct workqueue_struct *iot_eth_work_queue;

	struct semaphore	spi_lock;

	struct sk_buff_head	tx_tcp_wait_q;
	struct sk_buff_head	tx_wait_q;

	struct iot_data	mt_spi;

	int			msg_enable;

	u16			seq_num;

	volatile unsigned long		flags;
#		define EVENT_D2H_INTR		1
#		define EVENT_TX			2
#		define EVENT_WATCHDOG		8
#		define EVENT_SET_WIFI		0x10
	iot_wifi_setting_t* pWifiSetting;
} IOT_PRIVATE, *PIOT_PRIVATE;

typedef struct iot_eth1_data{
	PIOT_PRIVATE pHost;
	struct net_device	*ndev;	
	struct sk_buff_head	tx_tcp_wait_q;
	struct sk_buff_head	tx_wait_q;	
} IOT_ETH1_PRIVATE, *PIOT_ETH1_PRIVATE;

struct iot_rxinfo {
	u16	RxPktReady;
	u8	status;
	u8	type;	
#define RX_PKT_TYPE_ETH 1
#define RX_PKT_TYPE_LINK_STATE 2
	__le16	len;
	u16 reserved;
} __packed;


struct skb_data;

struct skb_data {
//	enum skb_state state;
	struct net_device *ndev;
	struct sk_buff *skb;
	size_t len;
	dma_addr_t phy_addr;
};


#endif


