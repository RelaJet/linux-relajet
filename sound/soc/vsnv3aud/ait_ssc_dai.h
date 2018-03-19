/*
 * atmel_ssc_dai.h - ALSA SSC interface for the Atmel  SoC
 *
 * Copyright (C) 2005 SAN People
 * Copyright (C) 2008 Atmel
 *
 * Author: Sedji Gaouaou <sedji.gaouaou@atmel.com>
 *         ATMEL CORP.
 *
 * Based on at91-ssc.c by
 * Frank Mandarino <fmandarino@endrelia.com>
 * Based on pxa2xx Platform drivers by
 * Liam Girdwood <lrg@slimlogic.co.uk>
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

#ifndef _ATMEL_SSC_DAI_H
#define _ATMEL_SSC_DAI_H

#include <linux/types.h>
#include <linux/atmel-ssc.h>

#include "ait-pcm.h"
#if 0
/* SSC system clock ids */
#define ATMEL_SYSCLK_MCK	0 /* SSC uses AT91 MCK as system clock */

/* SSC divider ids */
#define ATMEL_SSC_CMR_DIV	0 /* MCK divider for BCLK */
#define ATMEL_SSC_TCMR_PERIOD	1 /* BCLK divider for transmit FS */
#define ATMEL_SSC_RCMR_PERIOD	2 /* BCLK divider for receive FS */
/*
 * SSC direction masks
 */
#define SSC_DIR_MASK_UNUSED	0
#define SSC_DIR_MASK_PLAYBACK	1
#define SSC_DIR_MASK_CAPTURE	2

/*
 * SSC register values that Atmel left out of <linux/atmel-ssc.h>.  These
 * are expected to be used with SSC_BF
 */
/* START bit field values */
#define SSC_START_CONTINUOUS	0
#define SSC_START_TX_RX		1
#define SSC_START_LOW_RF	2
#define SSC_START_HIGH_RF	3
#define SSC_START_FALLING_RF	4
#define SSC_START_RISING_RF	5
#define SSC_START_LEVEL_RF	6
#define SSC_START_EDGE_RF	7
#define SSS_START_COMPARE_0	8

/* CKI bit field values */
#define SSC_CKI_FALLING		0
#define SSC_CKI_RISING		1

/* CKO bit field values */
#define SSC_CKO_NONE		0
#define SSC_CKO_CONTINUOUS	1
#define SSC_CKO_TRANSFER	2

/* CKS bit field values */
#define SSC_CKS_DIV		0
#define SSC_CKS_CLOCK		1
#define SSC_CKS_PIN		2

/* FSEDGE bit field values */
#define SSC_FSEDGE_POSITIVE	0
#define SSC_FSEDGE_NEGATIVE	1

/* FSOS bit field values */
#define SSC_FSOS_NONE		0
#define SSC_FSOS_NEGATIVE	1
#define SSC_FSOS_POSITIVE	2
#define SSC_FSOS_LOW		3
#define SSC_FSOS_HIGH		4
#define SSC_FSOS_TOGGLE		5

#define START_DELAY		1
#endif
/* START bit field values */
#define SSC_START_CONTINUOUS	0
#define SSC_START_TX_RX		1
#define SSC_START_LOW_RF	2
#define SSC_START_HIGH_RF	3
#define SSC_START_FALLING_RF	4
#define SSC_START_RISING_RF	5
#define SSC_START_LEVEL_RF	6
#define SSC_START_EDGE_RF	7
#define SSS_START_COMPARE_0	8
int atmel_ssc_set_audio(int ssc);


/*
 * SSC direction masks
 */
#define SSC_DIR_MASK_UNUSED	0
#define SSC_DIR_MASK_PLAYBACK	1
#define SSC_DIR_MASK_CAPTURE	2
struct atmel_ssc_state {
	u32 ssc_cmr;
	u32 ssc_rcmr;
	u32 ssc_rfmr;
	u32 ssc_tcmr;
	u32 ssc_tfmr;
	u32 ssc_sr;
	u32 ssc_imr;
};

struct atmel_ssc_info {
	char *name;
	struct ssc_device *ssc;
	spinlock_t lock;	/* lock for dir_mask */
	unsigned short dir_mask;	/* 0=unused, 1=playback, 2=capture */
	unsigned short initialized;	/* true if SSC has been initialized */
	unsigned short daifmt;
	unsigned short cmr_div;
	unsigned short tcmr_period;
	unsigned short rcmr_period;
	struct atmel_pcm_dma_params *dma_params[2];
	struct atmel_ssc_state ssc_state;
};


//======================================//
// I2S_MASTER_SLAVE_MODE_SELECT_L : 0x48//
//======================================//
#define	I2S_SLAVE_MODE		0x00
#define	I2S_MASTER_MODE		0x01

//======================================//
// I2S_HCK_MODE_SELECT_L : 0x4C         //
//======================================//
#define	I2S_256_FS		0x04

//======================================//
// I2S_OUTPUT_BITS_SIZE_SETTING_L : 0x34//
//======================================//
//#define	I2S_OUTPUT_16_BITS		0x01
//#define	I2S_OUTPUT_24_BITS		0x02
//#define	I2S_OUTPUT_32_BITS		0x04

//============================//
// I2S_LRCK_POLARITY_L : 0x1C //
//============================//
#define	LEFT_CHANNEL_HIGH		0x01
#define	LEFT_CHANNEL_LOW		0x00

//=====================================//
// I2S_OUTPUT_DATA_DELAY_MODE_L : 0x60 //
//=====================================//
//#define	I2S_STD_MODE			0x01
//#define	I2S_I2S_MODE			0x00

#endif /* _AT91_SSC_DAI_H */
