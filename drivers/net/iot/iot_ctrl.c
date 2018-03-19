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
 
#include "iot_ether.h"
#include "iot_api.h"
#include "iot_ctrl.h"

static iot_wifi_setting_t WifiSetting;
static int iot_ioctl_g_info(struct net_device *ndev,  iot_ioctl_req_t __user *pInfoReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	PIOT_ETH1_PRIVATE ap_data;

	iot_ioctl_req_t* req = pInfoReq;

	iot_dev_info_t devinfo;
	int ret =0;
	__be32 ipaddr;
	__be32 ap_ipaddr;
	int ethif = 0;
		
	if(strcmp(ndev->name, "eth1")==0)	{
		ap_data = (PIOT_PRIVATE)netdev_priv(ndev);
		host = ap_data->pHost;
		ethif = 1;
	}

	down (&host->spi_lock);
	ret = iot_get_info( ethif, &devinfo);
	up (&host->spi_lock);

	ipaddr = devinfo.ipaddr;
	pr_info("IoT Device %s Info: \n", devinfo.name);
	pr_info("Status: 0x%02x\n",	(int)devinfo.status);
	pr_info("MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x \n",	devinfo.mac_addr[0],
															devinfo.mac_addr[1],
															devinfo.mac_addr[2],
															devinfo.mac_addr[3],
															devinfo.mac_addr[4],
															devinfo.mac_addr[5]);

	pr_info("IP Address: %d.%d.%d.%d\n",ipaddr&0xff, (ipaddr>>8)&0xff,(ipaddr>>16)&0xff, ipaddr>>24 );

	pr_info("opmode: 0x%02x\n",	(int)devinfo.wifisetting.opmode);
	pr_info("ssid: %s\n",	devinfo.wifisetting.ssid);
	pr_info("psk: %s\n",	devinfo.wifisetting.psk);
	pr_info("auth: 0x%02x\n",	(int)devinfo.wifisetting.auth);
	pr_info("encrypt: 0x%02x\n",	(int)devinfo.wifisetting.encrypt);
	pr_info("reconnect: 0x%02x\n",	(int)devinfo.wifisetting.reconnect);

	if(devinfo.status==0x84)
	{
		pr_info("netif_carrier_on");
		netif_carrier_on(ndev);
	}
	if (!ret && copy_to_user((void*)req->data, &devinfo, sizeof (iot_dev_info_t)))
		ret = -EFAULT;

	return ret;
}

static int iot_ioctl_s_wifi(struct net_device *ndev,  iot_ioctl_req_t __user *pWifiReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	iot_ioctl_req_t req;
	int ret =0;

	if(strcmp(ndev->name, "eth1")==0)	{

		return 0;
	}
	
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
			queue_work (host->iot_eth_work_queue, &host->iot_eth_work);
			
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

static int iot_ioctl_s_powermode(struct net_device *ndev,  iot_ioctl_req_t __user *pReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	iot_ioctl_req_t req;
	int ret =0;

	if(strcmp(ndev->name, "eth1")==0)	{

		return 0;
	}

	
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
			ret = iot_set_power( 0);
			up (&host->spi_lock);

			pr_info("Set Power mode: %d\n",(int)mode);
			ret =0;

		}
		break;
				
	}

	return ret;
}

long iot_priv_ioctl(struct net_device *ndev, unsigned int cmd, void __user *arg)
{
	int retval = 0;
	
	switch (cmd) {

	case SIOCG_DEVINFO:
		retval = iot_ioctl_g_info(ndev, arg);
		break;

	case SIOC_IOT_WIFI:
		retval = iot_ioctl_s_wifi(ndev, arg);
		break;

	case SIOCS_POWERMODE:
		retval = iot_ioctl_s_powermode(ndev, arg);
		break;
	
	}

	return retval;
}

static void iot_get_drvinfo (struct net_device *ndev,
				 struct ethtool_drvinfo *info)
{
	if(strcmp(ndev->name, "eth1")==0)	{

		return 0;
	}

	/* Inherit standard device info */
	
	strncpy (info->driver, DRV_NAME, sizeof info->driver);
	strncpy (info->version, DRV_VERSION, sizeof info->version);
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: iot_get_link
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static u32 iot_get_link(struct net_device *ndev)
{
	u32 link = 0;
	int ret;
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	if(strcmp(ndev->name, "eth1")==0)	{

		return 0;
	}

	down (&host->spi_lock);
	ret = iot_get_linkstatus( &link);
	up (&host->spi_lock);

	return link;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: mtk_iot_get_msglevel
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static u32 iot_get_msglevel (struct net_device *ndev)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	return host->msg_enable;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: iot_set_msglevel
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static void iot_set_msglevel (struct net_device *ndev, u32 level)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	host->msg_enable = level;
}

const struct ethtool_ops iot_ethtool_ops = {
	.get_drvinfo		= iot_get_drvinfo,
	.get_link			= iot_get_link,
	.get_msglevel		= iot_get_msglevel,
	.set_msglevel		= iot_set_msglevel,

};
