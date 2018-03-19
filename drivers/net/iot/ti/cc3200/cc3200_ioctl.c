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
#include "iot.h"

#include "cc3200_main.h"
#include "cc3200_spi.h"
#include "cc3200_ioctl.h"

struct completion	spi_cmd_wait_event;
extern struct iot_cmd_t g_iot_command;
extern struct iot_cmd_resp_t g_iot_command_resp;
static iot_wifi_setting_t WifiSetting;
static int iot_ioctl_g_info(struct net_device *ndev,  iot_ioctl_req_t __user *pInfoReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	iot_ioctl_req_t* req = pInfoReq;

	iot_dev_info_t* pDevinfo;
	int ret =0;
	__be32 ipaddr;

	struct iot_cmd_t* spi_cmd = &g_iot_command;


	spi_cmd->cmd = SPIS_CMD_GET_INFO;
	spi_cmd->len = sizeof(iot_dev_info_t);

	set_bit (EVENT_SEND_CMD, &host->flags);
	queue_work (host->work_queue, &host->work);


	pr_info("iot send command: SPIS_CMD_GET_INFO(%d)",spi_cmd->cmd);
	
	wait_for_completion_timeout(&spi_cmd_wait_event, 2*HZ);
	if(g_iot_command_resp.cmd){
		struct iot_cmd_resp_t* spi_cmd_resp = &g_iot_command_resp;

		if(spi_cmd_resp->cmd!=SPIS_CMD_GET_INFO)
		{
			pr_err("Recv wrong responsed packet! The cmmand id not match.");
			pr_err("spi_cmd_resp->cmd = %d",spi_cmd_resp->cmd);		
			ret = -EFAULT;
		}

		if(spi_cmd_resp->len!=sizeof(iot_dev_info_t))
		{
			pr_err("Response data lenth not same as expected.");
			pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
			ret = -EFAULT;		
		}
		
		pDevinfo =(iot_dev_info_t* ) spi_cmd_resp->data;

		ipaddr = pDevinfo->ipaddr;
		pr_info("IoT Device %s Info: \n", pDevinfo->name);
		pr_info("Status: 0x%02x\n",	(int)pDevinfo->status);
		pr_info("IP Address: %d.%d.%d.%d\n",ipaddr&0xff, (ipaddr>>8)&0xff,(ipaddr>>16)&0xff, ipaddr>>24 );

		if (!ret && copy_to_user((void*)req->data, pDevinfo, sizeof (iot_dev_info_t)))
			ret = -EFAULT;
		g_iot_command_resp.cmd = 0;
	}
	else
	{
		ret = -EFAULT;
		pr_info("iot send command: SPIS_CMD_GET_INFO(%d) timeout!!",spi_cmd->cmd);
		
	}

	return ret;
}

static int iot_ioctl_g_statistics(struct net_device *ndev,  iot_ioctl_req_t __user *pInfoReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);
	iot_ioctl_req_t* req = pInfoReq;
	iot_ioctl_statistics_t drv_statistics;
	int ret = 0;
	
	drv_statistics.drv_buf_tcptx_count = skb_queue_len (&host->tx_tcp_wait_q);
	drv_statistics.drv_buf_udptx_count = skb_queue_len (&host->tx_wait_q);
pr_info("tcp buf count %d\r\n",drv_statistics.drv_buf_tcptx_count );
pr_info("udp buf count %d\r\n",drv_statistics.drv_buf_udptx_count);


	if (copy_to_user((void*)req->data, &drv_statistics, sizeof (iot_ioctl_statistics_t)))
		ret = -EFAULT;

	return ret;
}


static int iot_ioctl_s_wifi(struct net_device *ndev,  iot_ioctl_req_t __user *pWifiReq)
{
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	iot_ioctl_req_t req;
	int ret =0;
	
	if (!ret && copy_from_user(&req, pWifiReq, sizeof (iot_ioctl_req_t)))
		ret = -EFAULT;
	pr_info("WIFI subcmd: %d\n",req.subcmd);
	pr_info("WIFI data addr: 0x%08x\n",req.data);


	switch(req.subcmd)
	{

		case IOT_SET_WIFI_SETTING:
		{

			struct iot_cmd_t* spi_cmd = &g_iot_command;
			char* p;
			
			if (!ret && copy_from_user(&WifiSetting,(void*) req.data, sizeof (iot_wifi_setting_t)))
				ret = -EFAULT;
							
			pr_info("Set Wifi SSID: %s\n",WifiSetting.ssid);
			pr_info("Set Wifi Password: %s\n",WifiSetting.psk);
			pr_info("Set Wifi auth: %d\n",WifiSetting.auth);

			ret =0;

			host->pWifiSetting = &WifiSetting;

			spi_cmd->cmd = SPIS_CMD_SET_WIFI;
			spi_cmd->len = sizeof(iot_wifi_setting_t);

			WifiSetting.encrypt = 0;


			memcpy(spi_cmd->data, &WifiSetting,  sizeof(iot_wifi_setting_t));
			spi_cmd->len = sizeof(iot_dev_info_t);
			set_bit (EVENT_SEND_CMD, &host->flags);
			queue_work (host->work_queue, &host->work);


			pr_info("iot send command: SPIS_CMD_GET_INFO(%d)",spi_cmd->cmd);
			
			wait_for_completion_timeout(&spi_cmd_wait_event, 2*HZ);

			if(g_iot_command_resp.cmd){	
				struct iot_cmd_resp_t* spi_cmd_resp = &g_iot_command_resp;


				pr_info("spi_cmd_resp->cmd = %x",spi_cmd_resp->cmd);
				pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
				p = (char*)spi_cmd_resp->data;		
	
				pr_info("spi_cmd_resp->data: %s",p);
				
				if (!ret && copy_to_user((void*)req.data, "OK", 4))
					ret = -EFAULT;

				g_iot_command_resp.cmd = 0;
			}
			
			if (!ret && copy_to_user((void*)req.data, "NG", 4))
				ret = -EFAULT;
		}
			
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
	
	if (!ret && copy_from_user(&req, pReq, sizeof (iot_ioctl_req_t)))
		ret = -EFAULT;
	pr_info("Power mode subcmd: %d\n",req.subcmd);
	pr_info("Power mode  data addr: 0x%08x\n",req.data);


	switch(req.subcmd)
	{

		case IOT_SET_IOT_STANDBY:
		{
			__u32 mode;
			struct iot_cmd_t* spi_cmd = &g_iot_command;
			char* p;

			
			if (!ret && copy_from_user(&mode,(void*) req.data, sizeof (__u32)))
				ret = -EFAULT;
			

#define POWER_OFF "POWER LPDS"



			spi_cmd->cmd = SPIS_CMD_SET_POWERMODE;
			spi_cmd->len = sizeof(POWER_OFF);
			memcpy(spi_cmd->data, POWER_OFF,  sizeof(POWER_OFF));

			set_bit (EVENT_SEND_CMD, &host->flags);
			queue_work (host->work_queue, &host->work);
			pr_info("iot send command: SPIS_CMD_SET_POWERMODE(%d)",spi_cmd->cmd);
			wait_for_completion_timeout(&spi_cmd_wait_event, 2*HZ);


			if(g_iot_command_resp.cmd){
				struct iot_cmd_resp_t* spi_cmd_resp = &g_iot_command_resp;


					if(spi_cmd_resp->cmd!=SPIS_CMD_SET_POWERMODE)
					{
						pr_err("Recv wrong responsed packet! The cmmand id not match.");
						pr_err("spi_cmd_resp->cmd = %d",spi_cmd_resp->cmd);		
					}

					if(spi_cmd_resp->len!=sizeof(POWER_OFF)+sizeof(" OK"))
					{
						pr_err("Response data lenth not same as expected.");
						pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
					}



					pr_info("spi_cmd_resp->cmd = %x",spi_cmd_resp->cmd);
					pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
					p = (char*)spi_cmd_resp->data;
					p[sizeof(POWER_OFF)-1] = ' ';
					p[sizeof(POWER_OFF)+2] = 0;

					pr_info("spi_cmd_resp->data: %s",p);

					if (!ret && copy_to_user((void*)req.data, "OK", 4))
						ret = -EFAULT;

					pr_info("Set Power mode: %d\n",(int)mode);
					ret =0;

					host->pWifiSetting = &WifiSetting;
					g_iot_command_resp.cmd = 0;					
			}
			else{
				pr_info("iot send command: SPIS_CMD_SET_POWERMODE(%d) timeout!!",spi_cmd->cmd);

				ret = -EFAULT;
			}				
		}
		break;
				
	}

	return ret;
}

long iot_priv_ioctl(struct net_device *ndev, unsigned int cmd, void __user *arg)
{
	int retval = 0;
	struct ioctl_arg data;

	memset(&data, 0, sizeof(data));
	
	switch (cmd) {

	case SIOCG_DEVINFO:
		
		return iot_ioctl_g_info(ndev, arg);
		
		break;

	case SIOC_IOT_WIFI://SIOC_S_WIFI:
			 return iot_ioctl_s_wifi(ndev, arg);
		break;

	case SIOCS_POWERMODE:
			 return iot_ioctl_s_powermode(ndev, arg);
		break;
	case SIOCG_STATISTICS:
		return iot_ioctl_g_statistics(ndev, arg);
		break;
		
	case SIOCS_DATA_SEND:
		{
			int i;
			const struct iovec *iov = (struct iovec*)arg;
			char* buf = iov->iov_base;
			pr_info("Data len from user = %d\r\n",iov->iov_len);
			for(i=0;i<iov->iov_len;++i)
				pr_info("%x  = %d\r\n",buf[i]);
		
		}
		break;
	
	}

	return retval;
}

static void iot_get_drvinfo (struct net_device *ndev,
				 struct ethtool_drvinfo *info)
{
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

	char* p;
	u32 link = 0;
	PIOT_PRIVATE host = (PIOT_PRIVATE)netdev_priv(ndev);

	struct iot_cmd_t* spi_cmd = &g_iot_command;//kmalloc(sizeof(struct iot_cmd_t), GFP_KERNEL);

	spi_cmd->cmd = SPIS_CMD_GET_LINK;
	spi_cmd->len = 1;

	pr_info("iot send command: SPIS_CMD_GET_LINK(%d)",spi_cmd->cmd);

	set_bit (EVENT_SEND_CMD, &host->flags);
	queue_work (host->work_queue, &host->work);

	wait_for_completion_timeout(&spi_cmd_wait_event, 2*HZ);
	if(g_iot_command_resp.cmd){
		struct iot_cmd_resp_t* spi_cmd_resp = &g_iot_command_resp;

		if(spi_cmd_resp->cmd!=SPIS_CMD_GET_LINK)
		{
			pr_err("Recv wrong responsed packet! The cmmand id not match.");
			pr_err("spi_cmd_resp->cmd = %d",spi_cmd_resp->cmd);		
		}

		if(spi_cmd_resp->len!=1)
		{
			pr_err("Response data lenth not same as expected.");
			pr_info("spi_cmd_resp->len = %x",spi_cmd_resp->len);
		}
		
		p = (char*)spi_cmd_resp->data;
		pr_info("Link Status = %d" ,p[0]);

		link = IS_CONNECTED(p[0])?1:0;
		pr_info("link = %x",link);

		g_iot_command_resp.cmd = 0;
	}
	else
		pr_info("iot send command: SPIS_CMD_GET_LINK(%d) timeout!!",spi_cmd->cmd);
	return link;

	
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: iot_get_msglevel
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
	.get_link		= iot_get_link,
	.get_msglevel		= iot_get_msglevel,
	.set_msglevel		= iot_set_msglevel,

};

