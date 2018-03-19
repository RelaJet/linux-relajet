/*
 * at91-pcm.h - ALSA PCM interface for the Atmel AT91 SoC.
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *
 * Authors: Sedji Gaouaou <sedji.gaouaou@atmel.com>
 *
 * Based on at91-pcm. by:
 * Frank Mandarino <fmandarino@endrelia.com>
 * Copyright 2006 Endrelia Technologies Inc.
 *
 * Based on pxa2xx-pcm.c by:
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AIT_PCM_H
#define _AIT_PCM_H

#include <linux/atmel-ssc.h>

#define FIFO_REACH_UNRD_TH 	0x01
#define FIFO_REACH_UNWR_TH 	0x02
#define FIFO_FULL 				0x04
#define FIFO_EMPTY				0x08

/*
 * Registers and status bits that are required by the PCM driver.
 */
struct atmel_pdc_regs {
	unsigned int	xpr;		/* PDC recv/trans pointer */
	unsigned int	xcr;		/* PDC recv/trans counter */
	unsigned int	xnpr;		/* PDC next recv/trans pointer */
	unsigned int	xncr;		/* PDC next recv/trans counter */
	unsigned int	ptcr;		/* PDC transfer control */
};

#if 0
struct atmel_ssc_mask {
	u32	ssc_enable;		/* SSC recv/trans enable */
	u32	ssc_disable;		/* SSC recv/trans disable */
	u32	ssc_endx;		/* SSC ENDTX or ENDRX */
	u32	ssc_endbuf;		/* SSC TXBUFE or RXBUFF */
	u32	pdc_enable;		/* PDC recv/trans enable */
	u32	pdc_disable;		/* PDC recv/trans disable */
};
#endif 

/*
 * This structure, shared between the PCM driver and the interface,
 * contains all information required by the PCM driver to perform the
 * PDC DMA operation.  All fields except dma_intr_handler() are initialized
 * by the interface.  The dma_intr_handler() pointer is set by the PCM
 * driver and called by the interface SSC interrupt handler if it is
 * non-NULL.
 */
struct ait_pcm_dma_params {
	char *name;			/* stream identifier */
	int pdc_xfer_size;		/* PDC counter increment in bytes */
	struct ssc_device *ssc;		/* SSC device for stream */
	struct atmel_pdc_regs *pdc;	/* PDC receive or transmit registers */
	//struct atmel_ssc_mask *mask;	/* SSC & PDC status bits */
	struct snd_pcm_substream *substream;
	void (*dma_intr_handler)(u32, struct snd_pcm_substream *);
	int status;
	unsigned int sampling_rate;
};

struct mcrv2_pcm_dma_params {
	char *name;			/* stream identifier */
	int xfer_size;		
	dma_addr_t xfer_ptr;

	struct snd_pcm_substream *substream;
	void (*dma_intr_handler)(u32, struct snd_pcm_substream *);
	int status;
	unsigned int sampling_rate;
};

struct mcrv2_soc_i2s_dev {
	u8 __iomem *regbase;
	struct mcrv2_pcm_dma_params dma_params[2];
	struct clk		*clk;
	int irq;

	int ch;
	int pad;
	
	dma_addr_t sifbase;

	unsigned int wide;
	unsigned int channel_in;
	unsigned int channel_out;
	unsigned int lines_in;
	unsigned int lines_out;
		
};
/*
 * SSC register access (since ssc_writel() / ssc_readl() require literal name)
 */
#define ssc_readx(base, reg)            (__raw_readl((base) + (reg)))
#define ssc_writex(base, reg, value)    __raw_writel((value), (base) + (reg))

static inline unsigned int mcrv2_i2s_reg_readb(struct mcrv2_soc_i2s_dev *dev, unsigned int reg)
{
	return readb(dev->regbase + reg);
}

static inline unsigned int mcrv2_i2s_reg_readw(struct mcrv2_soc_i2s_dev *dev, unsigned int reg)
{
	return readw(dev->regbase + reg);
}

static inline unsigned int mcrv2_i2s_reg_readl(struct mcrv2_soc_i2s_dev *dev, unsigned int reg)
{
	return readl(dev->regbase + reg);
}

static inline void mcrv2_i2s_reg_writeb(struct mcrv2_soc_i2s_dev *dev, int reg, u8 val)
{
	writeb(val, dev->regbase + reg);
}

static inline void mcrv2_i2s_reg_writew(struct mcrv2_soc_i2s_dev *dev, int reg, u16 val)
{
	writew(val, dev->regbase + reg);
}

static inline void mcrv2_i2s_reg_writel(struct mcrv2_soc_i2s_dev *dev, int reg, u32 val)
{
	writel(val, dev->regbase + reg);
}


#endif /* _ATMEL_PCM_H */
