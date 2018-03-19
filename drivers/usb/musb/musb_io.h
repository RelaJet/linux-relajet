/*
 * MUSB OTG driver register I/O
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MUSB_LINUX_PLATFORM_ARCH_H__
#define __MUSB_LINUX_PLATFORM_ARCH_H__

#include <linux/io.h>

//#define ENABLE_REG_INFO

#if !defined(CONFIG_ARM) && !defined(CONFIG_SUPERH) \
	&& !defined(CONFIG_AVR32) && !defined(CONFIG_PPC32) \
	&& !defined(CONFIG_PPC64) && !defined(CONFIG_BLACKFIN)
static inline void readsl(const void __iomem *addr, void *buf, int len)
	{ insl((unsigned long)addr, buf, len); }
static inline void readsw(const void __iomem *addr, void *buf, int len)
	{ insw((unsigned long)addr, buf, len); }
static inline void readsb(const void __iomem *addr, void *buf, int len)
	{ insb((unsigned long)addr, buf, len); }

static inline void writesl(const void __iomem *addr, const void *buf, int len)
	{ outsl((unsigned long)addr, buf, len); }
static inline void writesw(const void __iomem *addr, const void *buf, int len)
	{ outsw((unsigned long)addr, buf, len); }
static inline void writesb(const void __iomem *addr, const void *buf, int len)
	{ outsb((unsigned long)addr, buf, len); }

#endif

#ifndef CONFIG_BLACKFIN
#ifdef ENABLE_REG_INFO
static char* strRegName[0x400]=
{
// 0
"Faddr","Power","IntTx0","IntTx1","IntRx0","IntRx1","IntTx0E","IntTx1E","IntRx0E","IntRx1E","IntUSB","IntUSBE","Frm0","Frm1","Index","",
// 0x10
"","","","","","","","","","","","","","","","CfgD",
// 0x20
"","","","","","","","","","","","","","","","",
// 0x30
"","","","","","","","","","","","","","","","",
// 0x40
"","","","","","","","","","","","","","","","",
// 0x50
"","","","","","","","","","","","","","","","",
// 0x60
"DevCtl","","","","","","","","","","","","HWVer","","","",
// 0x70
"","","","","","","","","","","","","","","","",
// 0x80
"","","","","","","","","","","","","","","",""
// 0x90
"","","","","","","","","","","","","","","",""
// 0xa0
"","","","","","","","","","","","","","","",""
// 0xb0
"","","","","","","","","","","","","","","",""
// 0xc0
"","","","","","","","","","","","","","","",""
// 0xd0
"","","","","","","","","","","","","","","",""
// 0xe0
"","","","","","","","","","","","","","","",""
// 0xf0
"","","","","","","","","","","","","","","",""
// 0x100
"","","","","","","","","","","","","","","",""
// 0x110
"","","","","","","","","","","","","","","",""
// 0x120
"","","","","","","","","","","","","","","",""
// 0x130
"","","","","","","","","","","","","","","",""
// 0x140
"","","","","","","","","","","","","","","",""
// 0x150
"","","","","","","","","","","","","","","",""

};

/* NOTE:  these offsets are all in bytes */

static inline u16 musb_readw(const void __iomem *addr, unsigned offset)
{ 
	u16 v;	
	v = __raw_readw(addr + offset); 
	printk(KERN_INFO"R: 0x%03x(%s) => 0x%04x\n",(int)(addr + offset)&0xfff,strRegName[(int)(addr + offset)&0xfff],v);

	return v;
}

static inline u32 musb_readl(const void __iomem *addr, unsigned offset)
{	
	u32 v;	
	v = __raw_readl(addr + offset); 
	printk(KERN_INFO"R: 0x%03x(%s)  => 0x%04x\n",(int)(addr + offset)&0xfff,strRegName[(int)(addr + offset)&0xfff],v);	

	return v;
}

static inline void musb_writew(void __iomem *addr, unsigned offset, u16 data)
{ 
	printk(KERN_INFO"W: 0x%03x(%s)  <= 0x%04x\n ",(int)(addr + offset)&0xfff,strRegName[(int)(addr + offset)&0xfff],data);
	__raw_writew(data, addr + offset); 
}

static inline void musb_writel(void __iomem *addr, unsigned offset, u32 data)
{

	printk(KERN_INFO"W: 0x%03x(%s)  <= 0x%08x\n ",(int)(addr + offset)&0xfff,data);
	__raw_writel(data, addr + offset);
}

#else
static inline u16 musb_readw(const void __iomem *addr, unsigned offset)
	{ return __raw_readw(addr + offset); }

static inline u32 musb_readl(const void __iomem *addr, unsigned offset)
	{ return __raw_readl(addr + offset); }


static inline void musb_writew(void __iomem *addr, unsigned offset, u16 data)
	{ __raw_writew(data, addr + offset); }

static inline void musb_writel(void __iomem *addr, unsigned offset, u32 data)
	{ __raw_writel(data, addr + offset); }

#endif

#ifdef CONFIG_USB_MUSB_TUSB6010

/*
 * TUSB6010 doesn't allow 8-bit access; 16-bit access is the minimum.
 */
static inline u8 musb_readb(const void __iomem *addr, unsigned offset)
{
	u16 tmp;
	u8 val;

	tmp = __raw_readw(addr + (offset & ~1));
	if (offset & 1)
		val = (tmp >> 8);
	else
		val = tmp & 0xff;

	return val;
}

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	u16 tmp;

	tmp = __raw_readw(addr + (offset & ~1));
	if (offset & 1)
		tmp = (data << 8) | (tmp & 0xff);
	else
		tmp = (tmp & 0xff00) | data;

	__raw_writew(tmp, addr + (offset & ~1));
}
#else
static inline u8 musb_readb(const void __iomem *addr, unsigned offset)

#ifdef ENABLE_REG_INFO
	{ 
		u8 v;
		v = readb(addr + offset); 
		printk(KERN_INFO"R: 0x%03x(%s)  => 0x%02x\n",(int)(addr + offset)&0xfff,strRegName[(int)(addr + offset)&0xfff],v);	
		return v;
	}
#else

	{ return __raw_readb(addr + offset); }
#endif

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)

{ 
#ifdef ENABLE_REG_INFO
	printk(KERN_INFO"W: 0x%03x(%s)  <= 0x%02x\n ",(int)(addr + offset)&0xfff,strRegName[(int)(addr + offset)&0xfff],data);
#endif
	 __raw_writeb(data, addr + offset);


}

#endif	/* CONFIG_USB_MUSB_TUSB6010 */

#else

static inline u8 musb_readb(const void __iomem *addr, unsigned offset)
	{ return (u8) (bfin_read16(addr + offset)); }

static inline u16 musb_readw(const void __iomem *addr, unsigned offset)
	{ return bfin_read16(addr + offset); }

static inline u32 musb_readl(const void __iomem *addr, unsigned offset)
	{ return (u32) (bfin_read16(addr + offset)); }

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)
	{ bfin_write16(addr + offset, (u16) data); }

static inline void musb_writew(void __iomem *addr, unsigned offset, u16 data)
	{ bfin_write16(addr + offset, data); }

static inline void musb_writel(void __iomem *addr, unsigned offset, u32 data)
	{ bfin_write16(addr + offset, (u16) data); }

#endif /* CONFIG_BLACKFIN */

#endif
