//------------------------------------------------------------------------------
//
//  File        : usb_cfg.h
//  Description : Header file of  
//  Author      : 
//  Revision    : 0.0
//
//------------------------------------------------------------------------------
#ifndef _AUDIO_EQ_CFG_H_
#define _AUDIO_EQ_CFG_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

typedef enum 
{
    EQ_CURVE_12dB = 0,      /*   12dB*/
    EQ_CURVE_10D5dB,        /* 10.5dB*/
    EQ_CURVE_9dB,           /*    9dB*/      
    EQ_CURVE_7D5dB,         /*  7.5dB*/     
    EQ_CURVE_6dB,           /*    6dB*/      
    EQ_CURVE_4D5dB,         /*  4.5dB*/     
    EQ_CURVE_3dB,           /*    3dB*/     
    EQ_CURVE_1D5dB,         /*  1.5dB*/     
    EQ_CURVE_0dB,           /*    0dB*/     
    EQ_CURVE_N1D5dB,        /* -1.5dB*/     
    EQ_CURVE_N3dB,          /*   -3dB*/     
    EQ_CURVE_N4D5dB,        /* -4.5dB*/     
    EQ_CURVE_N6dB,          /*   -6dB*/     
    EQ_CURVE_N7D5dB,        /* -7.5dB*/     
    EQ_CURVE_N9dB,          /*   -9dB*/     
    EQ_CURVE_N10D5dB,       /*-10.5dB*/
    EQ_BYPASS            
} MMP_EQ_GAIN;

typedef enum _MMP_EQ_BAND
{
    EQ_LSF=0,               /*Low shelf filter  central freq = 50Hz*/
    EQ_PK1,                 /*Peaking filter 1  central freq = 150Hz*/
    EQ_PK2,                 /*Peaking filter 2  central freq = 500Hz*/
    EQ_PK3,                 /*Peaking filter 3  central freq = 2500Hz*/
    EQ_HSF                  /*High shelf filter central freq = 5000Hz*/
} MMP_EQ_BAND;


MMP_ERR MMPF_Audio_ConfigHWEQGain(MMP_EQ_BAND band, MMP_EQ_GAIN gainstep);

/*
By project
*/
//#include "krypto_eq_cfg.h"
#define DEFAULT_EQ_HSF_GAIN EQ_BYPASS
#define DEFAULT_EQ_PK3_GAIN EQ_BYPASS
#define DEFAULT_EQ_PK2_GAIN EQ_BYPASS
#define DEFAULT_EQ_PK1_GAIN EQ_BYPASS
#define DEFAULT_EQ_LSF_GAIN EQ_BYPASS

#endif // _USB_CFG_H_

