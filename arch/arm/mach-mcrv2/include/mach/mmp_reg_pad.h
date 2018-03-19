//==============================================================================
//
//  File        : mmph_reg_pad.h
//  Description : INCLUDE File for the Pad register map.
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPH_REG_PAD_H_
#define _MMPH_REG_PAD_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

#if (CHIP == VSN_V3)
//-----------------------------
// PAD Structure (0x8000 9C00)
//-----------------------------
typedef struct _AITS_PAD {
    AIT_REG_B                           _x00[64];
    AIT_REG_B   PAD_PS_RST;
    AIT_REG_B   PAD_PHSYNC;
    AIT_REG_B   PAD_PVSYNC;
    AIT_REG_B   PAD_PSEN;
    AIT_REG_B   PAD_PSDA;
    AIT_REG_B   PAD_PSCK;
    AIT_REG_B   PAD_PPXL_CLK;
    AIT_REG_B   PAD_PDCLK;

    AIT_REG_B                           _x48[8];
    AIT_REG_B   PAD_PD0;
    AIT_REG_B   PAD_PD1;
    AIT_REG_B   PAD_PD4_11;
    AIT_REG_B   PAD_PSGPIO;
    AIT_REG_B   PAD_PD2;
    AIT_REG_B   PAD_PD3;
    AIT_REG_B                           _x56[170];
    AIT_REG_B   PAD_GPIO[64];
        /*-DEFINE-----------------------------------------------------*/
        #define     PAD_SCHMITT_TRIG        0x01
        #define     PAD_PULL_LOW            0x02
        #define     PAD_PULL_HIGH           0x04
        #define     PAD_SLEW_RATE_SLOW      0x07
        #define     PAD_E4_CURRENT          0x20
        #define     PAD_E8_CURRENT          0x40
        /*------------------------------------------------------------*/
} AITS_PAD, *AITPS_PAD;
#endif

#if (CHIP == MERCURY)
TODO
#endif

#if (CHIP == MCR_V2)
//-----------------------------
// PAD Structure (0x8000 5100)
//-----------------------------
typedef struct _AITS_PAD
{
    AIT_REG_B   PAD_IO_CFG_PSTM_B[0x20];                    // 0x5100
        /*-DEFINE-----------------------------------------------------*/
        #define PAD_NORMAL_TRIG             0x00
        #define PAD_SCHMITT_TRIG            0x01
        #define PAD_PULL_DOWN               0x02
        #define PAD_PULL_UP                 0x04
        #define PAD_FAST_SLEW               0x00
        #define PAD_SLOW_SLEW               0x08
        #define PAD_IDDQ_TEST_EN            0x10
        #define PAD_OUT_DRIVING(_a)         (((_a)&0x03)<<5)
        /*------------------------------------------------------------*/
    AIT_REG_B   PAD_IO_CFG_PSTM_T[0x36];                    // 0x5120
    AIT_REG_B   _x5156[0x0A];
    AIT_REG_B   PAD_IO_CFG_PBGPIO[0x10];                    // 0x5160
    AIT_REG_B   PAD_IO_CFG_PCGPIO[0x20];                    // 0x5170
    AIT_REG_B   PAD_IO_CFG_PAGPIO[0x08];                    // 0x5190
        #define PAD_IO_PULL_UP (1<<2)
        #define PAD_IO_PULL_DOWN (1<<1)		
        #define PAD_IO_PULL_DRIVING (3<<5)
    AIT_REG_B   PAD_IO_CFG_PHI2C[0x02];                     // 0x5198
    AIT_REG_B   PAD_IO_CFG_PI2S[0x05];                      // 0x519A
    AIT_REG_B   _x519F;
    AIT_REG_B   PAD_IO_CFG_PSNR[0x0D];                      // 0x51A0
    AIT_REG_B   _x51AD[0x03];
    AIT_REG_B   PAD_IO_CFG_PDGPIO[0x06];                    // 0x51B0
    AIT_REG_B   _x51B6[0x02];
    AIT_REG_B   PAD_IO_CFG_PMCLK;                           // 0x51B8
    AIT_REG_B   _x51B9;
    AIT_REG_B   PAD_IO_CFG_PVER_ID[0x05];                   // 0x51BA
    AIT_REG_B   _x51BF;
    AIT_REG_B   PAD_IO_CFG_PBGPIO2[0x06];                   // 0x51C0
    AIT_REG_B   _x51C6[0x0A];
    AIT_REG_B   PAD_IO_CFG_PLCD[0x08];                      // 0x51D0
    AIT_REG_B   PAD_IO_CFG_HDMI[0x02];                      // 0x51D8
} AITS_PAD, *AITPS_PAD;
#endif // (CHIP == MCR_V2)

/// @}	

#endif // _MMPH_REG_PAD_H_

