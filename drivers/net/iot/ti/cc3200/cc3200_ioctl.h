/* MT7687 SPI IOT Linux driver */

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

#ifndef _MT7687_IOCTL_H
#define _MT7687_IOCTL_H

struct ioctl_arg {
	unsigned int status;
	u32 ipaddr;
	
	unsigned int val;
	void* src_addr;
	void* dest_addr;
	unsigned int size;

};

struct mt7687_ioctl_data {
unsigned char val;
rwlock_t lock;

	void* src_addr;
	void* dest_addr;
	unsigned int size;
};



extern const struct file_operations iot_fops;
extern const struct ethtool_ops iot_ethtool_ops;

long iot_priv_ioctl(struct net_device *dev, unsigned int cmd, void* arg);

#endif

