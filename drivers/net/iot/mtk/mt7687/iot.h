/* MT7687 IOT I/F */

/* 
 * Copyright (c) 2016 Mstart
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

#ifndef _MT7687_IOT_H
#define _MT7687_IOT_H

typedef struct iot_ioctl_req{
__u32 subcmd;
__u32 data;
}iot_ioctl_req_t;

typedef struct iot_wifi_setting{
	__u8 opmode;	
	char ssid[32];
	char psk[65];

	__u8 auth;
	__u8 encrypt;
	__u8  reconnect;
}iot_wifi_setting_t;

typedef struct mt7687_dev_info{
	char	name[16];
	long	status;
	__be32 ipaddr;
	iot_wifi_setting_t wifisetting;
} mt7687_dev_info_t;

#define SIOCG_DEVINFO			(SIOCDEVPRIVATE+0)
#define SIOC_IOT_WIFI			(SIOCDEVPRIVATE+1)
	#define IOT_SET_WIFI_SETTING	0
	#define IOT_GET_WIFI_SETTING	1	
#define SIOCS_POWERMODE				(SIOCDEVPRIVATE+2)
	#define IOT_SET_IOT_STANDBY	0

#endif
