/* MT7687 SPI IOT Linux driver */

/* 
 * Copyright (c) 2016 Mstar Semi.
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
 
#include <linux/spinlock.h>
#include <linux/poll.h>
#include "mt7687_main.h"
#include "mt7687_spi.h"
#include "mt7687_ioctl.h"
#include "iot.h"

static iot_wifi_setting_t WifiSetting;
static int mt7687_ioctl_g_info(struct net_device *ndev,  iot_ioctl_req_t __user *pInfoReq)
{
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);
	iot_ioctl_req_t* req = pInfoReq;

	mt7687_dev_info_t devinfo;
	int ret =0;
	__be32 ipaddr;

	down (&host->spi_lock);	
	ret = mt7687_iot_get_info(&host->mt_spi, &devinfo);
	up (&host->spi_lock);

	ipaddr = devinfo.ipaddr;
	pr_info("IoT Device %s Info: \n", devinfo.name);
	pr_info("Status: 0x%02x\n",	(int)devinfo.status);
	pr_info("IP Address: %d.%d.%d.%d\n",ipaddr&0xff, (ipaddr>>8)&0xff,(ipaddr>>16)&0xff, ipaddr>>24 );

	if (!ret && copy_to_user((void*)req->data, &devinfo, sizeof (mt7687_dev_info_t)))
		ret = -EFAULT;

	return ret;
}

static int mt7687_ioctl_s_wifi(struct net_device *ndev,  iot_ioctl_req_t __user *pWifiReq)
{
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);

	iot_ioctl_req_t req;
	int ret =0;
	
	if (!ret && copy_from_user(&req, pWifiReq, sizeof (iot_ioctl_req_t)))
		ret = -EFAULT;
	pr_info("WIFI subcmd: %d\n",req.subcmd);
	pr_info("WIFI data addr: 0x%08x\n",req.data);


	switch(req.subcmd)
	{

		case IOT_SET_WIFI_SETTING:
				
			if (!ret && copy_from_user(&WifiSetting,(void*) req.data, sizeof (iot_wifi_setting_t)))
				ret = -EFAULT;
				
			
			pr_info("Set Wifi SSID: %s\n",WifiSetting.ssid);
			pr_info("Set Wifi Password: %s\n",WifiSetting.psk);
			pr_info("Set Wifi auth: %d\n",WifiSetting.auth);
			pr_info("Set Wifi encrypt: %d\n",WifiSetting.encrypt);			
			pr_info("Set Wifi Reconnect: %d\n",WifiSetting.reconnect);
			ret =0;

			host->pWifiSetting = &WifiSetting;
			
			set_bit (EVENT_SET_WIFI, &host->flags);
			queue_work (host->mt7687_work_queue, &host->mt7687_work);
			
			break;

		case IOT_GET_WIFI_SETTING:

			if (!ret && copy_to_user((void*) req.data, &WifiSetting, sizeof (iot_wifi_setting_t)))
				ret = -EFAULT;
			
			pr_info("Return Wifi SSID: %s\n",WifiSetting.ssid);
			pr_info("Return Wifi Password: %s\n",WifiSetting.psk);
			pr_info("Return Wifi auth: %d\n",WifiSetting.auth);
			pr_info("Return Wifi encrypt: %d\n",WifiSetting.encrypt);
			pr_info("Return Wifi Reconnect: %d\n",WifiSetting.reconnect);			
			ret =0;

			
			break;
				
	}

	return ret;
}

static int mt7687_ioctl_s_powermode(struct net_device *ndev,  iot_ioctl_req_t __user *pReq)
{
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);

	iot_ioctl_req_t req;
	int ret =0;
	
	if (!ret && copy_from_user(&req, pReq, sizeof (iot_ioctl_req_t)))
		ret = -EFAULT;
	pr_info("WIFI subcmd: %d\n",req.subcmd);
	pr_info("WIFI data addr: 0x%08x\n",req.data);


	switch(req.subcmd)
	{

		case IOT_SET_IOT_STANDBY:
		{
			__u32 mode;
			
			if (!ret && copy_from_user(&mode,(void*) req.data, sizeof (__u32)))
				ret = -EFAULT;
				
			down (&host->spi_lock);	
			ret = mt7687_iot_set_power(&host->mt_spi, 0);
			up (&host->spi_lock);

			pr_info("Set Power mode: %d\n",(int)mode);
			ret =0;

		}
			break;
				
	}

	return ret;
}

long mt7687_priv_ioctl(struct net_device *ndev, unsigned int cmd, void __user *arg)
{
	int retval = 0;
	struct ioctl_arg data;

	memset(&data, 0, sizeof(data));
	
	switch (cmd) {

	case SIOCG_DEVINFO:
		retval = mt7687_ioctl_g_info(ndev, arg);
		break;

	case SIOC_IOT_WIFI:
		retval = mt7687_ioctl_s_wifi(ndev, arg);
		break;

	case SIOCS_POWERMODE:
		retval = mt7687_ioctl_s_powermode(ndev, arg);
		break;
	
	}

	return retval;
}

static void mt7687_get_drvinfo (struct net_device *ndev,
				 struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	strncpy (info->driver, DRV_NAME, sizeof info->driver);
	strncpy (info->version, DRV_VERSION, sizeof info->version);
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_get_link
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static u32 mt7687_get_link(struct net_device *ndev)
{
	u32 link = 0;
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);

	down (&host->spi_lock);

	mt7687_read_linkstatus(&host->mt_spi, &link);
	up (&host->spi_lock);

	return link;

	
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_get_msglevel
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static u32 mt7687_get_msglevel (struct net_device *ndev)
{
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);
	return host->msg_enable;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: mt7687_set_msglevel
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static void mt7687_set_msglevel (struct net_device *ndev, u32 level)
{
	PMT_PRIVATE host = (PMT_PRIVATE)netdev_priv(ndev);
	host->msg_enable = level;
}

const struct ethtool_ops mt7687_ethtool_ops = {
	.get_drvinfo		= mt7687_get_drvinfo,
	.get_link		= mt7687_get_link,
	.get_msglevel		= mt7687_get_msglevel,
	.set_msglevel		= mt7687_set_msglevel,

};


