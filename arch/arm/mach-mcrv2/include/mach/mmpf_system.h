/**
 @file mmpf_system.h
 @brief Header File for the mmpf system.
 @author Truman Yang
 @version 1.0
*/

/** @addtogroup MMPF_SYS
@{
*/

#ifndef _MMPF_SYSTEM_H_
#define _MMPF_SYSTEM_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//============================================================================== 

#include "includes_fw.h"
#include "mmpf_pll.h"
#include "mmpf_pio.h"

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define COMPILER_DATE_LEN   (12) /// "mmm dd yyyy"
#define COMPILER_TIME_LEN   (10) /// "hh:mm:ss" 9 bytes but word alignment

#define CALL_BY_FILE_READ   (0x0001)
#define CALL_BY_TIMER       (0x0002)
#define CALL_BY_ISR         (0x0004)
#define CALL_BY_EOF         (0x0008)
#define CALL_BY_CMD         (CALL_BY_TIMER | CALL_BY_ISR | CALL_BY_EOF)

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_SYSTEM_POWER_LEVEL
{
    MMPF_POWER_LEVEL_0 = 0,             // power level 0 (lowest power)
    MMPF_POWER_LEVEL_1,                 // power level 1
    MMPF_POWER_LEVEL_2,                 // power level 2
    MMPF_POWER_LEVEL_3,                 // power level 3
    MMPF_POWER_LEVEL_4,                 // power level 4
    MMPF_POWER_LEVEL_5,                 // power level 5 (highest power)
    MMPF_POWER_LEVEL_MAX
} MMPF_SYSTEM_POWER_LEVEL;



typedef enum _MMPF_SYS_MDL
{
#if (CHIP == VSN_V3)
    MMPF_SYS_MDL_CPU_PHL    = 0,
    MMPF_SYS_MDL_VIF        = 1,
    MMPF_SYS_MDL_ISP        = 2,
    MMPF_SYS_MDL_JPG        = 3,
    MMPF_SYS_MDL_SCAL0      = 4,
    MMPF_SYS_MDL_GPIO       = 5,
    MMPF_SYS_MDL_AUD        = 6,
    MMPF_SYS_MDL_DRAM       = 7,
    MMPF_SYS_MDL_MCI        = 8,
    MMPF_SYS_MDL_RAW        = 9,
    MMPF_SYS_MDL_DMA        = 10,
    MMPF_SYS_MDL_I2CS       = 11,
    MMPF_SYS_MDL_USB        = 12,
    MMPF_SYS_MDL_H264       = 13,
    MMPF_SYS_MDL_IBC        = 14,
    MMPF_SYS_MDL_GRA        = 15,

    MMPF_SYS_MDL_CPU_SRAM   = 16,
    MMPF_SYS_MDL_SD0        = 17,
    MMPF_SYS_MDL_PWM        = 18,
    MMPF_SYS_MDL_PSPI       = 19,
    MMPF_SYS_MDL_USB_PHY    = 20,

    MMPF_SYS_MDL_SM         = 21,
    MMPF_SYS_MDL_SCAL1      = 22,
    MMPF_SYS_MDL_SCAL2      = 23,
    MMPF_SYS_MDL_SD1        = 24,
    MMPF_SYS_MDL_NUM        = 25
#endif

#if (CHIP == MERCURY)
    TODO
#endif

#if (CHIP == MCR_V2)
    MMPF_SYS_MDL_VIF0       = 0,
    MMPF_SYS_MDL_VIF1       = 1,
    MMPF_SYS_MDL_VIF2       = 2,
    MMPF_SYS_MDL_RAW_S0     = 3,
    MMPF_SYS_MDL_RAW_S1     = 4,
    MMPF_SYS_MDL_RAW_S2     = 5,
    MMPF_SYS_MDL_RAW_F      = 6,
    MMPF_SYS_MDL_ISP        = 7,

    MMPF_SYS_MDL_SCAL0      = 8,
    MMPF_SYS_MDL_SCAL1      = 9,
    MMPF_SYS_MDL_SCAL2      = 10,
    MMPF_SYS_MDL_SCAL3      = 11,
    MMPF_SYS_MDL_ICON0      = 12,
    MMPF_SYS_MDL_ICON1      = 13,
    MMPF_SYS_MDL_ICON2      = 14,
    MMPF_SYS_MDL_ICON3      = 15,

    MMPF_SYS_MDL_IBC0       = 16,
    MMPF_SYS_MDL_IBC1       = 17,
    MMPF_SYS_MDL_IBC2       = 18,
    MMPF_SYS_MDL_IBC3       = 19,
    MMPF_SYS_MDL_CCIR       = 20,
    MMPF_SYS_MDL_HDMI       = 21,
    MMPF_SYS_MDL_DSPY       = 22,
    MMPF_SYS_MDL_TV         = 23,

    MMPF_SYS_MDL_MCI        = 24,
    MMPF_SYS_MDL_DRAM       = 25,
    MMPF_SYS_MDL_H264       = 26,
    MMPF_SYS_MDL_JPG        = 27,
    MMPF_SYS_MDL_SD0        = 28,
    MMPF_SYS_MDL_SD1        = 29,
    MMPF_SYS_MDL_SD2        = 30,
    MMPF_SYS_MDL_SD3        = 31,

    MMPF_SYS_MDL_CPU_PHL    = 32,
    MMPF_SYS_MDL_PHL        = 33,
    MMPF_SYS_MDL_AUD        = 34,
    MMPF_SYS_MDL_GRA        = 35,
    MMPF_SYS_MDL_DMA_M0     = 36,
    MMPF_SYS_MDL_DMA_M1     = 37,
    MMPF_SYS_MDL_DMA_R0     = 38,
    MMPF_SYS_MDL_DMA_R1     = 39,

    MMPF_SYS_MDL_GPIO       = 40,
    MMPF_SYS_MDL_PWM        = 41,
    MMPF_SYS_MDL_PSPI       = 42,
    MMPF_SYS_MDL_DBIST      = 43,
    MMPF_SYS_MDL_IRDA       = 44,
    MMPF_SYS_MDL_RTC        = 45,
    MMPF_SYS_MDL_USB        = 46,
    MMPF_SYS_MDL_USBPHY     = 47,

    MMPF_SYS_MDL_I2CS       = 48,
    MMPF_SYS_MDL_SM         = 49,
    MMPF_SYS_MDL_LDC        = 50,
    MMPF_SYS_MDL_NUM        = 51
#endif
} MMPF_SYS_MDL;

typedef enum _MMPF_SYS_CLK
{
    #if (CHIP == VSN_V3)
    MMPF_SYS_CLK_MCI  = 0,
    MMPF_SYS_CLK_CPU  = 1,
    MMPF_SYS_CLK_SCALE = 2,
    MMPF_SYS_CLK_JPG  = 3,
    MMPF_SYS_CLK_AUD  = 4,
    MMPF_SYS_CLK_VIF  = 5,
    MMPF_SYS_CLK_ISP  = 6,
    MMPF_SYS_CLK_GPIO = 7,

    MMPF_SYS_CLK_DRAM = 8,
    MMPF_SYS_CLK_BS_SPI = 9,
    MMPF_SYS_CLK_RAWPROC = 10,
    MMPF_SYS_CLK_DMA  = 11,
    MMPF_SYS_CLK_I2CM = 12,
    MMPF_SYS_CLK_H264 = 13,
    MMPF_SYS_CLK_ICON = 14,
    MMPF_SYS_CLK_GRA  = 15,

    MMPF_SYS_CLK_USB  = 16,
    MMPF_SYS_CLK_IBC  = 17,
    MMPF_SYS_CLK_PWM  = 18,
    MMPF_SYS_CLK_SD1  = 19,
    MMPF_SYS_CLK_SD0  = 20,
    MMPF_SYS_CLK_PSPI = 21,
    MMPF_SYS_CLK_MIPI = 22,
    MMPF_SYS_CLK_CPU_PHL  = 23,

    MMPF_SYS_CLK_ADC = 24,
    MMPF_SYS_CLK_CIDC = 25,
    MMPF_SYS_CLK_GNR = 26,
    MMPF_SYS_CLK_SM   = 27,
    MMPF_SYS_CLK_COLR  = 28,

    MMPF_SYS_CLK_MDL_NUM = 29,
    #endif //(CHIP == VSN_V3)

    #if (CHIP == MERCURY)
    TODO
    #endif

    #if (CHIP == MCR_V2)
    MMPF_SYS_CLK_CPU_A          = 0,
    MMPF_SYS_CLK_CPU_A_PHL      = 1,
    MMPF_SYS_CLK_CPU_B          = 2,
    MMPF_SYS_CLK_CPU_B_PHL      = 3,
    MMPF_SYS_CLK_MCI            = 4,
    MMPF_SYS_CLK_DRAM           = 5,
    MMPF_SYS_CLK_VIF            = 6,
    MMPF_SYS_CLK_RAW_F          = 7,

    MMPF_SYS_CLK_RAW_S0         = 8,
    MMPF_SYS_CLK_RAW_S1         = 9,
    MMPF_SYS_CLK_RAW_S2         = 10,
    MMPF_SYS_CLK_ISP            = 11,
    MMPF_SYS_CLK_COLOR_MCI      = 12,
    MMPF_SYS_CLK_GNR            = 13,
    MMPF_SYS_CLK_SCALE          = 14,
    MMPF_SYS_CLK_ICON           = 15,

    MMPF_SYS_CLK_IBC            = 16,
    MMPF_SYS_CLK_CCIR           = 17,
    MMPF_SYS_CLK_DSPY           = 18,
    MMPF_SYS_CLK_HDMI           = 19,
    MMPF_SYS_CLK_TV             = 20,
    MMPF_SYS_CLK_JPG            = 21,
    MMPF_SYS_CLK_H264           = 22,
    MMPF_SYS_CLK_GRA            = 23,

    MMPF_SYS_CLK_DMA            = 24,
    MMPF_SYS_CLK_PWM            = 25,
    MMPF_SYS_CLK_PSPI           = 26,
    MMPF_SYS_CLK_SM             = 27,
    MMPF_SYS_CLK_SD0            = 28,
    MMPF_SYS_CLK_SD1            = 29,
    MMPF_SYS_CLK_SD2            = 30,
    MMPF_SYS_CLK_SD3            = 31,
    MMPF_SYS_CLK_GRP0_NUM       = 32,
    
    MMPF_SYS_CLK_USB            = 32,
    MMPF_SYS_CLK_I2CM           = 33,
    MMPF_SYS_CLK_BS_SPI         = 34,
    MMPF_SYS_CLK_GPIO           = 35,
    MMPF_SYS_CLK_AUD            = 36,
    MMPF_SYS_CLK_ADC            = 37,
    MMPF_SYS_CLK_DAC            = 38,
    MMPF_SYS_CLK_IRDA           = 39,

    MMPF_SYS_CLK_LDC            = 40,
    MMPF_SYS_CLK_BAYER          = 41,
    MMPF_SYS_CLK_MDL_NUM        = 42
    #endif //(CHIP == MCR_V2)

} MMPF_SYS_CLK;

typedef enum _MMPF_SYSTEM_SNR_IF_CAP {
    MMPF_SYSTEM_SNR_IF_CAP_3M = 0,
    MMPF_SYSTEM_SNR_IF_CAP_5M,
    MMPF_SYSTEM_SNR_IF_CAP_8M,
    MMPF_SYSTEM_SNR_IF_CAP_MAX
} MMPF_SYSTEM_SNR_IF_CAP;

typedef enum _MMPF_WAKEUP_EVNET
{
	MMPF_WAKEUP_NONE = 0x0,
	MMPF_WAKEUP_GPIO,
	MMPF_WAKEUP_TVPLUG_IN,
	MMPF_WAKEUP_USB_RESUME,
	MMPF_WAKEUP_PPMIC_INT,
	MMPF_WAKEUP_HDMI_HPD,
	MMPF_WAKEUP_MAX
} MMPF_WAKEUP_EVNET;
//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef struct MMP_CUSTOMER {
    MMP_UBYTE   customer;
    MMP_UBYTE   project;
    MMP_UBYTE   hardware;
} MMP_CUSTOMER;

typedef struct MMP_RELEASE_VERSION {
    MMP_UBYTE   major;
    MMP_UBYTE   minor;
    MMP_USHORT  build;
} MMP_RELEASE_VERSION;

typedef struct MMP_RELEASE_DATE {
    MMP_UBYTE   year;
    MMP_UBYTE   month;
    MMP_UBYTE   day;
} MMP_RELEASE_DATE;

typedef struct MMP_SYSTEM_BUILD_VERSION {
    MMP_UBYTE szDate[COMPILER_DATE_LEN]; /// "mmm dd yyyy"
    MMP_UBYTE szTime[COMPILER_TIME_LEN]; /// "hh:mm:ss" 9 bytes
} MMP_SYSTEM_BUILD_VERSION;

#if (SUPPORT_CPU_CLOCK_SW == 1)||(SUPPORT_G0_CLOCK_SW == 1)
typedef struct _MMP_SYSTEM_POWER_FREQ {
    MMPF_PLL_FREQ CPUFreq;              ///< CPU clock frequency
    MMPF_PLL_FREQ SYSFreq;              ///< Group 0 clock frequency
    MMPF_PLL_FREQ DRAMFreq;             ///< DRAM clock frequency
} MMP_SYSTEM_POWER_FREQ;
#endif

    
//==============================================================================
//
//                              VARIABLES
//
//==============================================================================


//==============================================================================
//
//                              MACRO FUNCTIONS
//
//==============================================================================

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================
MMP_ERR MMPF_SYS_EnableClock(MMPF_SYS_CLK clocktype, MMP_BOOL bEnableclock);
MMP_ERR MMPF_SYS_ResetHModule(MMPF_SYS_MDL moduletype, MMP_BOOL bResetRegister);
MMP_ERR MMPF_SYS_ResetSystem(MMP_UBYTE ubSpecialCase);
MMP_ERR MMPF_SYS_TuneMCIPriority(MMP_UBYTE ubMode);
MMP_ERR MMPF_SYS_SetSensorInputCapability(MMPF_SYSTEM_SNR_IF_CAP Capability);
MMP_UBYTE MMPF_SYS_ReadCoreID(void);

#if (OS_TYPE == OS_UCOSII)
MMP_ERR MMPF_SYS_EnterPSMode(MMP_BOOL bEnterPSMode);
MMP_ERR MMPF_SYS_EnterBypassMode(MMP_BOOL bEnterBypassMode);
MMP_ERR MMPF_SYS_InitializeHIF(void);
MMP_ERR MMPF_SYS_ProcessCmd(void);
MMP_ERR MMPF_SYS_GetFWEndAddr(MMP_ULONG *ulEndAddr);
void MMPF_SYS_InitFB(void) ;
void *MMPF_SYS_AllocFB(char *tag,MMP_ULONG size, MMP_USHORT align);
void MMPF_SYS_SetCurFBAddr(char *ptr);
void *MMPF_SYS_GetCurFBAddr(void) ;
MMP_ERR MMPF_SYS_EnterSelfSleepMode(void);
MMP_ERR MMPF_Self_Rom_Boot(void);
#endif

#endif	//_MMPF_SYSTEM_H_

/** @}*/ //end of MMPF_SYS
