//==============================================================================
//
//  File        : mmpf_vif.h
//  Description : INCLUDE File for the Firmware VIF Control driver function
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_VIF_H_
#define _MMPF_VIF_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#if (CHIP == VSN_V3)
#define MAX_MIPI_DATA_LANE_NUM  (2)
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define MAX_MIPI_DATA_LANE_NUM  (4)
#endif

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================

typedef enum _MMPF_VIF_IN_TYPE{
    MMPF_VIF_PARALLEL,
    MMPF_VIF_MIPI
} MMPF_VIF_IN_TYPE;

typedef enum _MMPF_VIF_MDL_ID{
    MMPF_VIF_MDL_ID0,    
    MMPF_VIF_MDL_ID1,
    #if (CHIP == MERCURY)    
    MMPF_VIF_MDL_ID2,
    #endif
    MMPF_VIF_MDL_NUM
} MMPF_VIF_MDL_ID;

typedef enum _MMPF_VIF_IF{
	MMPF_VIF_IF_PARALLEL        = 0,
	MMPF_VIF_IF_MIPI_SINGLE_0   = 1,
	MMPF_VIF_IF_MIPI_SINGLE_1   = 2,
	MMPF_VIF_IF_MIPI_SINGLE_2   = 3,
	MMPF_VIF_IF_MIPI_SINGLE_3   = 4,
	MMPF_VIF_IF_MIPI_DUAL_01    = 5,
	MMPF_VIF_IF_MIPI_DUAL_02    = 6,
	MMPF_VIF_IF_MIPI_DUAL_03    = 7,
	MMPF_VIF_IF_MIPI_DUAL_12    = 8,
	MMPF_VIF_IF_MIPI_DUAL_13    = 9,
	MMPF_VIF_IF_MIPI_DUAL_23    = 10,
	MMPF_VIF_IF_MIPI_QUAD       = 11
} MMPF_VIF_IF;

typedef enum _MMPF_VIF_PATH {
    MMPF_VIF_PATH1_BAYER_TO_ISP = 0,
    MMPF_VIF_PATH2_YCbCr422_2_YCbCr444_BYPASS_ISP,
    MMPF_VIF_PATH3_YCbCr422_2_BAYER,
    MMPF_VIF_PATH4_YCbCr422_2_YUV444,
    MMPF_VIF_PATH5_BAYER_TO_RAWPROC,
    MMPF_VIF_PATH6_JPG2RAWPROC,        
    MMPF_VIF_PATH7_YCbCr422_2_YCbCr420_TO_RAWPROC,
    MMPF_VIF_PATH8_VC_DATA
} MMPF_VIF_PATH;

typedef enum _MMPF_VIF_COLOR_ID{
    MMPF_VIF_COLORID_00 = 0,
    MMPF_VIF_COLORID_01,
    MMPF_VIF_COLORID_10,
    MMPF_VIF_COLORID_11        
} MMPF_VIF_COLOR_ID;

typedef enum _MMPF_VIF_SNR_PHASE_DELAY{
    MMPF_VIF_SNR_PHASE_DELAY_NONE = 0,
    MMPF_VIF_SNR_PHASE_DELAY_0_5F,
    MMPF_VIF_SNR_PHASE_DELAY_1_0F,
    MMPF_VIF_SNR_PHASE_DELAY_1_5F        
} MMPF_VIF_SNR_PHASE_DELAY;

typedef enum _MMPF_VIF_SNR_CLK_POLARITY{
    MMPF_VIF_SNR_CLK_POLARITY_POS = 0,
    MMPF_VIF_SNR_CLK_POLARITY_NEG       
} MMPF_VIF_SNR_CLK_POLARITY;

typedef enum _MMPF_VIF_SNR_LATCH_TIMING{
    MMPF_VIF_SNR_LATCH_POS_EDGE = 0,
    MMPF_VIF_SNR_LATCH_NEG_EDGE      
} MMPF_VIF_SNR_LATCH_TIMING;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef struct _MMPF_VIF_GRAB_INFO {
    MMP_USHORT                  usStartX;
    MMP_USHORT                  usStartY;
    MMP_USHORT                  usGrabWidth;
    MMP_USHORT                  usGrabHeight;
} MMPF_VIF_GRAB_INFO;

typedef struct _MMPF_VIF_GRAB_POS {
    MMP_USHORT                  usStartX;
    MMP_USHORT                  usStartY;
    MMP_USHORT                  usEndX;
    MMP_USHORT                  usEndY;
} MMPF_VIF_GRAB_POS;

typedef struct _MMPF_VIF_DATA_OFFSET {
    MMP_UBYTE                   ubCompId;
    MMP_BOOL                    bPostive;
    MMP_UBYTE                   ubOffVal;
} MMPF_VIF_DATA_OFFSET;

typedef struct _MMPF_VIF_MCLK_ATTR {
    MMP_BOOL                    bClkOutEn;
    MMP_UBYTE                   ubClkFreqDiv;       // If ubClkFreqDiv = 0, Use ulMClkFreq and ulDesiredFreq to calculate division.
    MMP_ULONG                   ulMClkFreq;         // Input clock  : kHz
    MMP_ULONG                   ulDesiredFreq;      // Output clock : kHz
    MMPF_VIF_SNR_PHASE_DELAY    ubClkPhase;
    MMPF_VIF_SNR_CLK_POLARITY   ubClkPolarity;
} MMPF_VIF_MCLK_ATTR;

typedef struct _MMPF_VIF_PARAL_ATTR {    
    MMPF_VIF_SNR_LATCH_TIMING   ubLatchTiming;
    MMPF_VIF_SNR_CLK_POLARITY   ubHsyncPolarity;
    MMPF_VIF_SNR_CLK_POLARITY   ubVsyncPolarity;
} MMPF_VIF_PARAL_ATTR;

typedef struct _MMPF_MIPI_RX_ATTR {
    MMP_BOOL                    bClkDelayEn;
    MMP_BOOL                    bClkLaneSwapEn;
    MMP_USHORT                  usClkDelay;
    MMPF_VIF_SNR_LATCH_TIMING   ubbClkLatchTiming;
    MMP_BOOL                    bDataLaneEn[MAX_MIPI_DATA_LANE_NUM];
    MMP_BOOL                    bDataDelayEn[MAX_MIPI_DATA_LANE_NUM]; 
    MMP_BOOL                    bDataLaneSwapEn[MAX_MIPI_DATA_LANE_NUM];
    MMP_UBYTE                   ubDataLaneSrc[MAX_MIPI_DATA_LANE_NUM];
    MMP_USHORT                  usDataDelay[MAX_MIPI_DATA_LANE_NUM];
    MMP_UBYTE                  	ubDataSotCnt[MAX_MIPI_DATA_LANE_NUM];
} MMPF_MIPI_RX_ATTR;

typedef struct _MMPF_VIF_IGBT_ATTR {
    MMP_BOOL                    bEnable;
    MMP_USHORT                  usStartLine;
    MMP_USHORT                  usStartOfst;
    MMP_USHORT                  usEndCycle;    
} MMPF_VIF_IGBT_ATTR;

// in mmpf_reg_vif.h
//#define VIF_SIF_SEN (0x1 << 0)		//sensor vif power enable 
//#define VIF_SIF_RST (0x1 << 3 )		//sensor vif power enable 

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR	    MMPF_VIF_SetPIODir(MMP_UBYTE ubVid, MMP_UBYTE mask, MMP_BOOL bOutput);
MMP_ERR	    MMPF_VIF_SetPIOOutput(MMP_UBYTE ubVid, MMP_UBYTE mask, MMP_BOOL bSetHigh);
MMP_BOOL    MMPF_VIF_GetPIOOutput(MMP_UBYTE ubVid, MMP_UBYTE mask);

MMP_ERR 	MMPF_VIF_EnableInputInterface(MMP_UBYTE ubVid, MMP_BOOL enable);
MMP_ERR		MMPF_VIF_RegisterInputInterface(MMP_UBYTE ubVid, MMPF_VIF_IN_TYPE type);
MMP_ERR 	MMPF_VIF_EnableOutput(MMP_UBYTE ubVid, MMP_BOOL bEnable);
MMP_ERR 	MMPF_VIF_IsInterfaceEnable(MMP_UBYTE ubVid, MMP_BOOL *bEnable);
MMP_ERR 	MMPF_VIF_SetGrabPosition(MMP_UBYTE ubVid, MMPF_VIF_GRAB_INFO *pGrab);
MMP_ERR 	MMPF_VIF_GetGrabResolution(MMP_UBYTE ubVid, MMP_ULONG *ulWidth, MMP_ULONG *ulHeight);
MMP_ERR     MMPF_VIF_IsOutputEnable(MMP_UBYTE ubVid, MMP_BOOL *bEnable);

MMP_ERR     MMPF_VIF_GetGrabPosition(MMP_UBYTE ubVid, MMP_USHORT *usPixelStart, MMP_USHORT *usPixelEnd,
								    MMP_USHORT *usLineStart, MMP_USHORT *usLineEnd);
MMP_ERR     MMPF_VIF_SetInterrupt(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_BOOL bEnable);

MMP_ERR     MMPF_VIF_WaitFrameSig(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_USHORT usFrameCount);
MMP_ERR     MMPF_VIF_CheckFrameSig(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_USHORT usFrameCount);
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
MMP_ERR     MMPF_VIF_SetFormatAndPath(MMP_UBYTE ubVid, MMPF_VIF_PATH path);
#endif
MMP_ERR     MMPF_VIF_SetFixedDataOut(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMP_UBYTE ubData);
MMP_ERR     MMPF_VIF_SetColorID(MMP_UBYTE ubVid, MMPF_VIF_COLOR_ID clrId);
MMP_ERR     MMPF_VIF_SetFrameSkip(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMP_UBYTE ubSkipIdx);
MMP_ERR     MMPF_VIF_SetDownSample(MMP_UBYTE ubVid, MMP_BOOL bEnable, 
                               		MMP_UBYTE ubHratio, MMP_UBYTE ubVratio, MMP_BOOL bHsmooth);
MMP_ERR     MMPF_VIF_AdjustInputPixel(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMPF_VIF_DATA_OFFSET* pOffset);
MMP_ERR     MMPF_VIF_SetSensorMClockAttr(MMP_UBYTE ubVid, MMPF_VIF_MCLK_ATTR* pAttr);
MMP_ERR     MMPF_VIF_SetParallelTimingAttr(MMP_UBYTE ubVid, MMPF_VIF_PARAL_ATTR* pAttr);
MMP_ERR     MMPF_VIF_SetSensorMipiAttr(MMP_UBYTE ubVid, MMPF_MIPI_RX_ATTR* pAttr);
MMP_ERR     MMPF_VIF_SetIGBT(MMP_UBYTE ubVid, MMPF_VIF_IGBT_ATTR* pAttr);

#endif // _MMPF_VIF_H_
