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


#define MT7687_IOC_MAGIC 0xED

#define IOCTL_MT7687_SET _IOW(MT7687_IOC_MAGIC, 1, struct ioctl_arg)
#define IOCTL_MT7687_GET _IOR(MT7687_IOC_MAGIC, 2, struct ioctl_arg)
#define IOCTL_MT7687_NUM  _IOR(MT7687_IOC_MAGIC, 3, int)
#define IOCTL_MT7687_SETSRC _IOW(MT7687_IOC_MAGIC, 4, int)
#define IOCTL_MT7687_SETDEST _IOW(MT7687_IOC_MAGIC, 5, int)
#define IOCTL_MT7687_START _IOW(MT7687_IOC_MAGIC, 6, struct aitdma_memcpy)
#define IOCTL_MT7687_MALLOC _IOW(MT7687_IOC_MAGIC, 7, struct aitdma_mem)
#define IOCTL_MT7687_FREEMEM _IOW(MT7687_IOC_MAGIC, 8, struct aitdma_mem)
#define IOCTL_VALSET_NUM  _IOW(MT7687_IOC_MAGIC, 9, int)

#define IOCTL_VAL_MAXNR 7



extern const struct file_operations mt7687_fops;
extern const struct ethtool_ops mt7687_ethtool_ops;

//int mt7687_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
long mt7687_priv_ioctl(struct net_device *dev, unsigned int cmd, void* arg);

#endif

