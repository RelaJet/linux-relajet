//==============================================================================
//
//  File        : mmpf_pll.h
//  Description : INCLUDE File for the Firmware PLL Driver.
//  Author      : Rogers Chen
//  Revision    : 1.0
//
//==============================================================================



#ifndef _MMPF_PLL_H_
#define _MMPF_PLL_H_

#include    "includes_fw.h"
#include    "mmp_err.h"


//==============================================================================
//
//                              COMPILER OPTION
//
//==============================================================================


//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================
#define EXT_PMCLK_CKL       12000
#define MMPF_PLL_GROUP0     0x01
#define MMPF_PLL_GROUP1     0x02
#define MMPF_PLL_GROUPNULL  0x00

#if (CHIP == VSN_V2)
#define MAX_GROUP_NUM 	    5 	//Group 0 ==> GBL_CLK
							    //Group 1 ==> DRAM_CLK
							    //Group 2 ==> USBPHY_CLK
							    //Group 3 ==> RX_BIST_CLK
							    //Group 4 ==> SENSOR_CLK
#endif
#if (CHIP == VSN_V3)
#define MAX_GROUP_NUM 	    7 	//Group 0 ==> GBL_CLK
							    //Group 1 ==> DRAM_CLK
							    //Group 2 ==> USBPHY_CLK
							    //Group 3 ==> RX_BIST_CLK
							    //Group 4 ==> SENSOR_CLK
							    //Group 5 ==> AUDIO_CLK
							    //Group 6 ==> COLOR_CLK(ISP)
#endif
#if (CHIP == MCR_V2) ||  (CHIP == MERCURY)
#define MAX_GROUP_NUM       9   //Group 0 ==> GBL_CLK
                                //Group 1 ==> DRAM_CLK
                                //Group 2 ==> USBPHY_CLK
                                //Group 3 ==> RX_BIST_CLK
                                //Group 4 ==> SENSOR_CLK
                                //Group 5 ==> AUDIO_CLK
                                //Group 6 ==> COLOR_CLK(ISP)
                                //Group 7 ==> BAYER¡BRAW FETCH
                                //Group 8 ==> MIPI TX
#endif

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef enum _MMPF_CLK_GRP
{
    MMPF_CLK_GRP_GBL = 0,
    MMPF_CLK_GRP_DRAM,
    MMPF_CLK_GRP_USB,
    MMPF_CLK_GRP_RXBIST,
    MMPF_CLK_GRP_SENSOR,

    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    MMPF_CLK_GRP_AUDIO,
    MMPF_CLK_GRP_COLOR,
    #endif

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_CLK_GRP_BAYER_RAW,
    MMPF_CLK_GRP_MIPITX,
    #endif
    MMPF_CLK_MAX_GRP_NUM,
    MMPF_PLL_GRP_MAX = MMPF_CLK_MAX_GRP_NUM

} MMPF_CLK_GRP;

typedef enum _MMPF_AUDSRC {
	MMPF_AUDSRC_MCLK = 0,
	MMPF_AUDSRC_I2S
	
} MMPF_AUDSRC;

typedef enum _MMPF_PLL_MODE {
    #if (CHIP == VSN_V3) || (CHIP == VSN_V2)
    //Group 0 ==> GBL_CLK
    //Group 1 ==> DRAM_CLK
    //Group 2 ==> USBPHY_CLK
    //Group 3 ==> RX_BIST_CLK
    //Group 4 ==> SENSOR_CLK
	MMPF_PLL_24CPU_24G0134_X = 0x0,
	MMPF_PLL_48CPU_48G0134_X,
	MMPF_PLL_96CPU_96G0134_X,
	MMPF_PLL_120CPU_120G0134_X,
	MMPF_PLL_132CPU_132G0134_X,
	MMPF_PLL_133CPU_133G0134_X,
	MMPF_PLL_144CPU_144G0134_X,
	MMPF_PLL_166CPU_166G0134_X,
	MMPF_PLL_266CPU_144G0134_X,
	MMPF_PLL_300CPU_144G0134_X,
	MMPF_PLL_332CPU_192G034_166G1_X,
	MMPF_PLL_332CPU_200G034_166G1_X,
	MMPF_PLL_168CPU_133G0134_X,
	MMPF_PLL_192CPU_133G0134_X,
	MMPF_PLL_400CPU_144G0134_X,
	MMPF_PLL_400CPU_166G0134_X,
	MMPF_PLL_400CPU_192G034_200G1_X,
	MMPF_PLL_400CPU_240G034_192G1_X,
	MMPF_PLL_498CPU_166G0134_X,
	MMPF_PLL_498CPU_192G034_200G1_X,
	MMPF_PLL_498CPU_216G034_192G1_X,
	MMPF_PLL_498CPU_264G034_192G1_X,
	MMPF_PLL_600CPU_166G0134_X,
	MMPF_PLL_600CPU_192G0134_X,
	MMPF_PLL_600CPU_216G0_192G134_X,
	MMPF_PLL_600CPU_240G0_192G134_X,
	MMPF_PLL_600CPU_264G0_192G134_X,
	MMPF_PLL_600CPU_192G034_166G1_X,
	MMPF_PLL_600CPU_192G034_200G1_X,
	MMPF_PLL_AUDIO_48K,
	MMPF_PLL_AUDIO_44d1K,
	MMPF_PLL_AUDIO_32K,
	MMPF_PLL_AUDIO_24K,
	MMPF_PLL_AUDIO_22d05K,
	MMPF_PLL_AUDIO_16K,
	MMPF_PLL_AUDIO_12K,
	MMPF_PLL_AUDIO_11d025K,
	MMPF_PLL_AUDIO_8K,
    #endif

    #if (CHIP == MERCURY)
    MMPF_PLL_AUDIO_48K,
    MMPF_PLL_AUDIO_44d1K,
    MMPF_PLL_AUDIO_32K,
    MMPF_PLL_AUDIO_24K,
    MMPF_PLL_AUDIO_22d05K,
    MMPF_PLL_AUDIO_16K,
    MMPF_PLL_AUDIO_12K,
    MMPF_PLL_AUDIO_11d025K,
    MMPF_PLL_AUDIO_8K,
    #endif

    #if (CHIP == MCR_V2)
    //for PLL3 to Audio(G5)
    MMPF_PLL_AUDIO_192K,
    MMPF_PLL_AUDIO_128K,
    MMPF_PLL_AUDIO_96K,
    MMPF_PLL_AUDIO_64K,
    MMPF_PLL_AUDIO_48K,
    MMPF_PLL_AUDIO_44d1K,
    MMPF_PLL_AUDIO_32K,
    MMPF_PLL_AUDIO_24K,
    MMPF_PLL_AUDIO_22d05K,
    MMPF_PLL_AUDIO_16K,
    MMPF_PLL_AUDIO_12K,
    MMPF_PLL_AUDIO_11d025K,
    MMPF_PLL_AUDIO_8K,

    MMPF_PLL_ExtClkOutput,
    #endif

    MMPF_PLL_MODE_NUMBER
} MMPF_PLL_MODE;

typedef enum _MMPF_PLL_FREQ {
    #if (CHIP == VSN_V3) || (CHIP == VSN_V3)
    MMPF_PLL_FREQ_600MHz = 0,
    MMPF_PLL_FREQ_498MHz,
    MMPF_PLL_FREQ_400MHz,
    MMPF_PLL_FREQ_333MHz,
    MMPF_PLL_FREQ_332MHz,
    MMPF_PLL_FREQ_300MHz,
	MMPF_PLL_FREQ_266MHz,
	MMPF_PLL_FREQ_264MHz_PLL1,
	MMPF_PLL_FREQ_240MHz_PLL0,
	MMPF_PLL_FREQ_240MHz_PLL1,
	MMPF_PLL_FREQ_216MHz_PLL0,
	MMPF_PLL_FREQ_216MHz_PLL1,
	MMPF_PLL_FREQ_200MHz_PLL0,
	MMPF_PLL_FREQ_200MHz_PLL1,
	MMPF_PLL_FREQ_192MHz_PLL0,
	MMPF_PLL_FREQ_192MHz_PLL1,
	MMPF_PLL_FREQ_168MHz,
    MMPF_PLL_FREQ_166MHz_PLL0,
    MMPF_PLL_FREQ_166MHz_PLL1,
    MMPF_PLL_FREQ_162MHz,
    MMPF_PLL_FREQ_156MHz,
    MMPF_PLL_FREQ_144MHz,
    MMPF_PLL_FREQ_133MHz,
    MMPF_PLL_FREQ_132MHz,
    MMPF_PLL_FREQ_120MHz,
	MMPF_PLL_FREQ_96MHz,
    MMPF_PLL_FREQ_60MHz,
    MMPF_PLL_FREQ_54MHz,
    MMPF_PLL_FREQ_48MHz,
    MMPF_PLL_FREQ_39MHz,
    MMPF_PLL_FREQ_24MHz, 
    MMPF_PLL_FREQ_EXT_CLK,
    #endif

    #if (CHIP == MCR_V2)
    MMPF_PLL_FREQ13_400MHz = 0,
    MMPF_PLL_FREQ13_333MHz,
    MMPF_PLL_FREQ13_300MHz,
    MMPF_PLL_FREQ13_266MHz,
    MMPF_PLL_FREQ13_240MHz,
    MMPF_PLL_FREQ13_216MHz,
    MMPF_PLL_FREQ13_192MHz,
    MMPF_PLL_FREQ13_166MHz,
    MMPF_PLL_FREQ13_162MHz,
    MMPF_PLL_FREQ13_156MHz,
    MMPF_PLL_FREQ13_144MHz,
    MMPF_PLL_FREQ13_133MHz,
    MMPF_PLL_FREQ13_132MHz,
    MMPF_PLL_FREQ13_120MHz,
    MMPF_PLL_FREQ13_96MHz,
    MMPF_PLL_FREQ13_60MHz,
    MMPF_PLL_FREQ13_54MHz,
    MMPF_PLL_FREQ13_48MHz,
    MMPF_PLL_FREQ13_39MHz,
    MMPF_PLL_FREQ13_24MHz,

    //for PLL3 to Audio(G5)
    MMPF_PLL_FREQ13_98d3040MHz,
    MMPF_PLL_FREQ13_65d5360MHz,
    MMPF_PLL_FREQ13_24d5760MHz,
    MMPF_PLL_FREQ13_22d5792MHz,
    MMPF_PLL_FREQ13_20d4800MHz,
    MMPF_PLL_FREQ13_12d2880MHz,
    MMPF_PLL_FREQ13_11d2896MHz,
    MMPF_PLL_FREQ13_8d1920MHz,
    MMPF_PLL_FREQ13_6d1440MHz,
    MMPF_PLL_FREQ13_5d6448MHz,
    MMPF_PLL_FREQ13_5d1200MHz,

    MMPF_PLL_FREQ13_ExtClkOutput,
    #endif

    MMPF_PLL_FREQ_TOTAL_NO
} MMPF_PLL_FREQ;

#if (CHIP == MERCURY)
typedef enum _MMPF_CPU_CLK {
    MMPF_CPU_A,
    MMPF_CPU_B
} MMPF_CPU_CLK;
#endif

typedef enum _MMPF_PLL_ID {
    MMPF_PLL_ID_0 = 0,
    MMPF_PLL_ID_1,
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    MMPF_PLL_ID_2,
    #endif
    MMPF_PLL_ID_PMCLK,
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_PLL_ID_3,
    MMPF_PLL_ID_4,
    #endif
    #if (CHIP == MCR_V2)
    MMPF_PLL_ID_5,
    #endif
    MMPF_PLL_ID_MAX
} MMPF_PLL_ID;

#if (CHIP == MCR_V2) || (CHIP == MERCURY)
typedef struct _MMPF_PLL_SETTINGS {
    MMP_UBYTE M;
    MMP_UBYTE N;
    MMP_UBYTE K;
    MMP_ULONG frac;
    MMP_UBYTE VCO;
    MMP_ULONG freq;
} MMPF_PLL_SETTINGS;

typedef struct _MMPF_SYS_CLK_SRC {
    MMPF_PLL_ID cpua;
    MMP_UBYTE   cpua_div;
    MMPF_PLL_ID cpub;
    MMP_UBYTE   cpub_div;
    MMPF_PLL_ID global;
    MMP_UBYTE   global_div;
    MMPF_PLL_ID dram;
    MMP_UBYTE   dram_div;
    MMPF_PLL_ID usb_phy;
    MMP_UBYTE   usb_phy_div;
    MMPF_PLL_ID rx_bist;
    MMP_UBYTE   rx_bist_div;
    MMPF_PLL_ID sensor;
    MMP_UBYTE   sensor_div;
    MMPF_PLL_ID audio;
    MMP_UBYTE   audio_div;
    MMPF_PLL_ID isp;
    MMP_UBYTE   isp_div;
    MMPF_PLL_ID bayer;
    MMP_UBYTE   bayer_div;
    MMPF_PLL_ID mipitx;
    MMP_UBYTE   mipitx_div;
    #if (CHIP == MCR_V2)
    MMPF_PLL_ID hdmi;
    MMP_UBYTE   hdmi_div;
    #endif
} MMPF_SYS_CLK_SRC;
#endif //(CHIP == MCR_V2)

typedef enum _MMPF_PLL_MIPI_FREQ
{
    #if (CHIP == MCR_V2)
    MMPF_PLL_MIPI_FREQ13_192MHz,
    MMPF_PLL_MIPI_FREQ13_162MHz,
    MMPF_PLL_MIPI_FREQ13_156MHz,
    MMPF_PLL_MIPI_FREQ13_144MHz,
    MMPF_PLL_MIPI_FREQ13_132MHz,
    MMPF_PLL_MIPI_FREQ13_96MHz,
    MMPF_PLL_MIPI_FREQ13_54MHz,
    MMPF_PLL_MIPI_FREQ13_48MHz,
    MMPF_PLL_MIPI_FREQ13_ExtClkOutput,
    #endif
    MMPF_PLL_MIPI_FREQ_TOTAL_NO
} MMPF_PLL_MIPI_FREQ;

typedef enum _MMPF_PLL_NO
{
    MMPF_PLL_0 = 0,
    MMPF_PLL_1,
    MMPF_PLL_2,
#if (CHIP == MCR_V2)
    MMPF_PLL_3,
    MMPF_PLL_4,
    MMPF_PLL_5,
#endif
    MMPF_PLL_NULL
} MMPF_PLL_NO;

typedef enum _MMPF_GROUP_SRC {
    MMPF_GUP_SRC_PLL_0  = MMPF_PLL_ID_0,
    MMPF_GUP_SRC_PLL_1  = MMPF_PLL_ID_1,
    MMPF_GUP_SRC_PLL_2  = MMPF_PLL_ID_2,
    MMPF_GUP_SRC_PMCLK1 = MMPF_PLL_ID_PMCLK,
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_GUP_SRC_PLL_3  = MMPF_PLL_ID_3,
    MMPF_GUP_SRC_PLL_4  = MMPF_PLL_ID_4,
    MMPF_GUP_SRC_PLL_5  = MMPF_PLL_ID_5,
    #endif
    MMPF_GUP_SRC_NULL
} MMPF_GROUP_SRC;

#if (CHIP == VSN_V3) || (CHIP == VSN_V2)
typedef enum _MMPF_PLL_SRC {
	MMPF_PLL_SRC_PMCLK = 0x0,
    MMPF_PLL_SRC_DPLL0,
    MMPF_PLL_SRC_DPLL1,
    #if (CHIP == VSN_V3)
    MMPF_PLL_SRC_DPLL2
    #endif
} MMPF_PLL_SRC;
#endif

#if (CHIP == MCR_V2)
typedef enum _MMPF_PLL_HDMI_MODE
{
    MMPF_PLL_HDMI_27MHZ = 0,
    MMPF_PLL_HDMI_74_25MHZ,
    MMPF_PLL_HDMI_SYNC_DISPLAY
} MMPF_PLL_HDMI_MODE;
#endif


//==============================================================================
//
//                              VARIABLES
//
//==============================================================================


//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================
MMP_ERR MMPF_PLL_Setting(MMPF_PLL_MODE PLLMode, MMP_BOOL KeepG0);
void MMPF_PLL_WaitCount(MMP_ULONG count);
MMP_ERR MMPF_PLL_GetCPUFreq(MMP_ULONG *ulCPUFreq);
MMP_ERR MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP ubGroupNum, MMP_ULONG *ulGroupFreq);
//MMP_ERR MMPF_PLL_SetAudioPLL(MMPF_PLL_MODE ulSampleRate);
//MMP_ULONG MMPF_PLL_GetGroup0Freq();

typedef enum _LINUX_SOC_PATH
{
	AFE_PATH_ADC=0,
	AFE_PATH_DAC,
	AFE_PATH_I2S_IN,
	AFE_PATH_I2S_OUT
}LINUX_SOC_PATH;

MMP_ERR MMPF_PLL_SetAudioPLL(MMPF_PLL_MODE ulSampleRate,LINUX_SOC_PATH path,MMP_BYTE reset_pll);

MMP_ERR MMPF_PLL_SetAudioPLL_I2S( int pllclockrate);
extern	void MMPF_I2S_PADSET(MMP_UBYTE ch, MMP_BOOL pad);


//==============================================================================
//
//                              MACRO FUNCTIONS
//
//==============================================================================


#endif // _INCLUDES_H_
