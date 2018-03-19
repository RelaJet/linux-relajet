#ifndef MMPF_SD_H
#define MMPF_SD_H

#include "reg_retina.h"
#include "includes_fw.h"
#include "mmp_reg_sd.h"
/** @addtogroup MMPF_SD
@{
*/
#define ENABLE_SD_SEQUENTIAL_OP         (0)
#define ENABLE_DETECT_SDCLASS           (1)
#define SD_BUS_TYPE                     (4) //< Valid input: 1 or 4, dafault: 4
#define ENABLE_SDIO_FEATURE             (0) //< 0: disable SDIO feature; 1: enable SDIO feature.
#define ENABLE_SDHC_SWITCH_HIGH_SPEED   (1) //< 0: disable HSM for SDHC; 1: enable HSM for SDHC
#define SD_BUS_REENTRY_PROTECT          (1) //< 0: disable SD BUS reentry protect; 1: enable SD BUS reentry protect
#define ENABLE_SD_CLKGATING             (0) //< 0: disable SD clock gating; 1: enable SD clock gating
#define DEFINE_MMC_BUS_WIDTH            (8) //< 4: support 4bit bus width; 8: support 8bit bus width; else is 1 bit
#if (CHIP == MCR_V2)
#if defined(ALL_FW)
#define ENABLE_SD_READTOSRAM            (0)//dbg (1) //< 1: Need to read data to sram first then copy to Dram
#endif
#if defined(MBOOT_FW)||defined(UPDATER_FW)
#define ENABLE_SD_READTOSRAM            (0) //< 0: No need to read data to sram first then copy to Dram
#endif
#endif
#if (CHIP == VSN_V3)
#define ENABLE_SD_READTOSRAM            (0) //< 0: No need to read data to sram first then copy to Dram
#endif
#if (CHIP == P_V2)
#define ENABLE_SD_READTOSRAM            (0) //< 0: No need to read data to sram first then copy to Dram
#endif

#define ENABLE_READ_RETRY               (1)
#define ENABLE_WRITE_RETRY              (1)

#if (ENABLE_SDIO_FEATURE == 1)
#define SDIO_AUTO_CLK_EN                (0) // user define
#define SDIO_SW_CLK_GATING              (1) // user define
#endif

#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
/* This pad selection of SD1 may changed for different platform,
 * please check schematic to look for which SD pad set is used for SD1.
 */
#define SD1_PAD_NUM                     (MMPF_SD1_PAD1)
#define SD_TEMP_SIZE                    (0x10000)
#endif

#if defined(SDIO_AUTO_CLK_EN) && defined(SDIO_SW_CLK_GATING)
#define SDIO_SW_CLK_CTL     ((!SDIO_AUTO_CLK_EN) && (SDIO_SW_CLK_GATING))
#endif

typedef enum _MMPF_SD_ID
{
    MMPF_SD_0 = 0,
    MMPF_SD_1,
#if (CHIP == MCR_V2)
    MMPF_SD_2,
#endif
    MMPF_SD_DEV_NUM
} MMPF_SD_ID;

typedef enum _MMPF_SD_PAD
{
    #if (CHIP == P_V2)
    MMPF_SD_PAD0 = 0,
    MMPF_SD_PAD1,
    MMPF_SD_PAD2,
    MMPF_SD_PAD3,
    #endif
    #if (CHIP == VSN_V3)
    MMPF_SD_PAD0 = 0,
    MMPF_SD_PAD1,
    #endif
    #if (CHIP == MCR_V2)
    MMPF_SD0_PAD0 = 0,  /* 1/4-bit   PCGPIO0~5   */
    MMPF_SD1_PAD0,      /* 1/4-bit   PCGPIO6~11  */
    MMPF_SD1_PAD1,      /* 1/4/8-bit PBGPIO12~21 */
    MMPF_SD2_PAD0,      /* 1/4-bit   PCGPIO12~17 */
    #endif
    MMPF_SD_PAD_MAX
} MMPF_SD_PAD;

typedef enum
{
    MMPF_SD_SDTYPE,
    MMPF_SD_MMCTYPE    
}MMPF_SD_CARDTYPE;

typedef enum {
    MMC_CLK_26M = 0,
    MMC_CLK_52M
} MMC4xClkMode;

typedef struct _stSDMMCHandler{
    MMPF_SD_ID          id;
    AITPS_SD            pSD;
    MMP_BOOL            bIsSD;
    MMP_BOOL            bIsHC;
    MMP_BOOL            ubSDHCHsm;
    MMP_BOOL            bIs2GMMC;
    MMP_BOOL            bIsMMC4X;
    #if (ENABLE_DETECT_SDCLASS == 1)
    MMP_UBYTE           ubSDClass;
    #endif
    MMC4xClkMode        ubMMC4XClkMode;

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
    //MMP_BOOL          bEnableSequentialOp;
    MMP_BOOL            bSDNeedStopTransmit;
    MMP_ULONG           ulSDLastWriteSect;
    #endif

    MMP_UBYTE           ubSdPadMapping;
    MMP_UBYTE           ubSdBusWidth;

    MMP_ULONG           ulCardAddr;
    MMP_USHORT          usCurClkDiv;
    MMP_BOOL            bPwrOnFirst;
    MMP_USHORT          usBlockLen;
    MMP_BOOL            bAccessFail;

    MMP_ULONG           ulCardSize;
    MMP_ULONG           ulReadTimeout;
    MMP_ULONG           ulWriteTimeout;
    MMP_ULONG           ulMMCBootSize;
    MMP_ULONG           ulMMCCurrentPartition;
    MMPF_OS_SEMID       IntTriggerSemID;
    #if (SD_BUS_REENTRY_PROTECT==1)
    MMPF_OS_SEMID       BusySemID;
    #endif


    #if (EN_CARD_DETECT == 1)
    MMP_UBYTE           ubSDCardDetPinNum;
    MMP_UBYTE           ubSDCardDetPolarity;
    #endif

    #if (EN_CARD_WRITEPROTECT == 1)
    MMP_UBYTE           ubSDCardWPPinNum;
    MMP_UBYTE           ubSDCardWPPolarity;
    #endif

    #if (EN_CARD_PWRCTL == 1)
    MMP_UBYTE           ubSDCardPwrCtlPinNum;
    MMP_UBYTE           ubSDCardPwrCtlPolarity;
    #endif

    #if (ENABLE_SDIO_FEATURE == 1)
    MMPF_OS_SEMID       SDIOBusySemID;
    void (*MMPF_SDIO_CallBackISR)(struct _stSDMMCHandler* SDMMCArg);
    #if (SDIO_SW_CLK_CTL == 1)
    MMP_ULONG           ulSDIOClkCnt;
    #endif

    #endif
    #if (ENABLE_SD_READTOSRAM == 1)
    MMPF_OS_SEMID       DMASemID;
    MMP_ULONG           ulTempSDAddr;
    #endif
} stSDMMCHandler;

//------------------------------------------------------------------------------
// Macro
//------------------------------------------------------------------------------
#define SD_SUPPORT_VOLTAGE  0x403F0000 // 3.3~2.8v
//#define SD_SUPPORT_VOLTAGE    0x00FFC0000 // 3.6~2.7v

//------------------------------------------------------------------------------
// MMC/SD Command Index
//------------------------------------------------------------------------------
// Basic command (class 0)
#define GO_IDLE_STATE       0
#define SEND_OP_COND        1   // reserved for SD
#define ALL_SEND_CID        2
#define SET_RELATIVE_ADDR   3
#define SEND_RELATIVE_ADDR  3
#define SET_DSR             4
#define IO_SEND_OP_COND     5
#define SWITCH_FUNC         6
#define SELECT_CARD         7
#define DESELECT_CARD       7
#define SEND_IF_COND        8
#define SEND_EXT_CSD        8
#define SEND_CSD            9
#define SEND_CID            10
#define READ_DAT_UTIL_STOP  11  // reserved for SD
#define STOP_TRANSMISSION   12
#define SEND_STATUS         13
#define GO_INACTIVE_STATE   15

// Block oriented read commands (class 2)
#define SET_BLOCKLEN        16
#define READ_SINGLE_BLOCK   17
#define READ_MULTIPLE_BLOCK 18

// Block oriented write commands (class 4)
#define WRITE_BLOCK         24
#define WRITE_MULTIPLE_BLOCK    25
#define PROGRAM_CSD         27

// Erase commands
#define ERASE_WR_BLK_START  32
#define ERASE_WR_BLK_END    33
#define ERASE               38

// Application specific commands (class 8)
#define APP_CMD             55
#define GEN_CMD             56

// SD Application command Index
#define SET_BUS_WIDTH           6
#define SD_STATUS               141  // 13+0x80
#define SEND_NUM_WR_BLOCKS      22
#define SET_WR_BLK_ERASE_COUNT  23
#define SD_APP_OP_COND          41
#define SET_CLR_CARD_DETECT     42
#define SEND_SCR                51
#define IO_SET_BUS_WIDTH        52
#define IO_SET_CLR_CARD_DETECT  52
#define IO_RW_DIRECT            52
#define IO_RW_EXTENDED          53

// SD Vender command
#define VENDER_COMMAND_62       62


// SD timeout setting
#define SD_DEFAULT_TIMEOUT           0xFFFF                     // 24M
#define SD_NORMAL_READ_TIMEOUT       0xFFFF//0x2BF2             // 24M => (100*24000/256)*1.2   (20% tolerance)
#define SD_NORMAL_WRITE_TIMEOUT      0xFFFF//0x6DDD             // 24M => (250*24000/256)*1.2   (20% tolerance)
#define SD_HIGHSPEED_READ_TIMEOUT    0x1FFFF//0x57E4            // 48M => (100*48000/256)*1.2   (20% tolerance)
#define SD_HIGHSPEED_WRITE_TIMEOUT   0x1FFFF//0xDBBA            // 48M => (250*48000/256)*1.2   (20% tolerance)

//add@101118: for automatic clock dividing.
//base on SD spec2.0
#define DEFINE_MMC_FULLSPEED (26000)
#define DEFINE_MMC_HIGHSPEED (52000)
#define DEFINE_SD_FULLSPEED (25000)
#define DEFINE_SD_HIGHSPEED (50000)

#define DEFINE_SDIO_LOWSPEED  (400)
#define DEFINE_SDIO_FULLSPEED (25000)
#define DEFINE_SDIO_HIGHSPEED (50000)

#define SD_MMC_CLK_DBG_MSG (0)      //enable/disable dbg msg about sd/mmc clock setting

typedef enum{
    MMPF_SD_NORMAL_MODE =0, 
    MMPF_SD_HIGHSPEED_MODE 
}MMPF_SD_SPEED_MODE;

typedef enum{
    MMPF_SD_DEFAULT_TIMEOUT =0, 
    MMPF_SD_READ_TIMEOUT , 
    MMPF_SD_WRITE_TIMEOUT
}MMPF_SD_TIMEOUT;

typedef enum{
    MMPF_MMC_USER_AREA =0, 
    MMPF_MMC_BOOT1_AREA , 
    MMPF_MMC_BOOT2_AREA
}MMPF_MMC_BOOTPARTITION;
//------------------------------------------------------------------------------
// Card Detect Return Value
//------------------------------------------------------------------------------
#define SD_IN           1
#define SD_OUT          0

#endif /* MMPF_SD_H */

//------------------------------------------------------------------------------
//  Function Prototype
//------------------------------------------------------------------------------
///@brief Get SD handler
stSDMMCHandler *MMPF_SD_GetHandler(MMPF_SD_ID SDId);
///@brief Initial SD handler for each controller
void     MMPF_SD_InitHandler(void);
///@brief Initial SD handler for each controller
MMP_ERR MMPF_SD_ConfigPadMapping(stSDMMCHandler *SDMMCArg, MMPF_SD_PAD padid, MMP_BOOL b8bitBus);
///@brief Initial SD Controller Interafce
MMP_ERR  MMPF_SD_InitialInterface(stSDMMCHandler *SDMMCArg);
///@brief Reset MMC device
MMP_ERR  MMPF_MMC_Reset(stSDMMCHandler *SDMMCArg);
///@brief Reset SD device
MMP_ERR  MMPF_SD_Reset(stSDMMCHandler *SDMMCArg);
/**
@brief Get size of SD card
@param pSize it returns sector count of SD card
*/
MMP_ERR  MMPF_SD_GetSize(stSDMMCHandler *SDMMCArg, MMPF_MMC_BOOTPARTITION part, MMP_ULONG *pSize);
/**
@brief Read data from SD Sector
@param dmastartaddr Start address of buffer. Total read size = sectcount * sector size (512Byte).
@param startsect Start sector number.
@param sectcount Count of sector.
*/
MMP_ERR  MMPF_SD_ReadSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount);
/**
@brief Write data into SD Sector
@param dmastartaddr Start address of buffer. Total write size = sectcount * sector size (512Byte).
@param startsect Start sector number.
@param sectcount Count of sector.
*/
MMP_ERR  MMPF_SD_WriteSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount);
/**
@brief Send SD command by SD controller
@param id Index of SD host controller.
@param command Command Code.
@param argument Command parameter.
*/
MMP_ERR  MMPF_SD_SendCommand(stSDMMCHandler *SDMMCArg, MMP_UBYTE command, MMP_ULONG argument);
///@brief Get card information
MMP_ERR  MMPF_SD_GetCardInfo(stSDMMCHandler *SDMMCArg, MMPF_SD_CARDTYPE btype);
///@brief Get card status
//@retval SD_NO_ERROR No error
//@retval MMP_SD_ERR_COMMAND_FAILED If any error
MMP_ERR  MMPF_SD_CheckCardStatus(stSDMMCHandler *SDMMCArg);

/**
@brief Wait for specific count. When CPU working freq is 96.2MHz, count 0x800000 is about 1 second.
@param count Wait count.
*/
void MMPF_SD_WaitCount(MMP_ULONG count);
/**
@brief Enable the clock of storage controller
@param id id of specific SD device clock setting.
@param usClkDiv Target clock divider. (MUST refer to Register Guide for available setting)
*/
void MMPF_SD_SwitchClock(stSDMMCHandler *SDMMCArg, MMP_USHORT usClkDiv);
/**
@brief Detect for SD Card Write Protect
@retval 0 Writable (Can write data into storage)
@retval 1 Write Protect (Cannot write data into storages)
*/
MMP_USHORT  MMPF_SD_CheckCardWP(stSDMMCHandler *SDMMCArg);
/**
@brief Detect for SD Card In
@retval SD_IN Card plug
@retval SD_OUT Card unplug
*/
MMP_USHORT MMPF_SD_CheckCardIn(stSDMMCHandler *SDMMCArg);

/**
@brief Enable Card power on
*/
void MMPF_SD_EnableCardPWR(stSDMMCHandler *SDMMCArg, MMP_BOOL enable);


#if (ENABLE_DETECT_SDCLASS == 1)
MMP_ERR MMPF_SD_GetCardClass(stSDMMCHandler *SDMMCArg, MMP_UBYTE *ubClass);
#endif

/** @brief Set temp buffer address for SD driver

@param[in] ulStartAddr Start address for temp buffer address.
@param[in] ulSize Size for temp buffer. Currently buffer size is only 64 Byte for SDHC message handshake
@return NONE
*/
void MMPF_SD_SetTmpAddr(MMP_ULONG ulStartAddr, MMP_ULONG ulSize);
/** @brief SD switch timeout mode

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] Read or write timeout. 
@retval MMP_ERR_NONE Success
*/
MMP_ERR  MMPF_SD_SwitchTimeout(stSDMMCHandler *SDMMCArg, MMPF_SD_TIMEOUT mode);
/** @brief SD set timeout mode

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] SD high speed mode or not. 
@retval MMP_ERR_NONE Success
*/
MMP_ERR  MMPF_SD_SetTimeout(stSDMMCHandler *SDMMCArg, MMPF_SD_SPEED_MODE mode);

/** @brief SD erase sectors in assigned range.

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] startsect Start sector number.
@param[in] sectcount Count of sector.
@retval MMP_ERR_NONE Success
*/
MMP_ERR MMPF_SD_EraseSector(stSDMMCHandler *SDMMCArg, MMP_ULONG startsect, MMP_ULONG sectcount);
/// @}

MMP_ERR MMPF_MMC_ConfigBootPartition(stSDMMCHandler *SDMMCArg, MMP_ULONG ulBootSectorNum);
MMP_ERR MMPF_MMC_SwitchBootPartition(stSDMMCHandler *SDMMCArg, MMPF_MMC_BOOTPARTITION part);
void    MMPF_SD_GetTmpAddr(MMP_ULONG *ulStartAddr);

/** @addtogroup MMPF_SDIO
@{
*/
#if (ENABLE_SDIO_FEATURE == 1)
MMP_ERR MMPF_SDIO_Reset(stSDMMCHandler *SDMMCArg);
MMP_ERR MMPF_SDIO_WriteReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE  fun_num, MMP_ULONG reg_addr, MMP_UBYTE src);
MMP_ERR MMPF_SDIO_ReadReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE fun_num, MMP_ULONG reg_addr, MMP_UBYTE *p_dst);
MMP_ERR MMPF_SDIO_WriteMultiReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE fun_num, MMP_UBYTE blk_mode, MMP_UBYTE op_code, MMP_ULONG reg_addr, MMP_ULONG count, MMP_ULONG blk_size, MMP_UBYTE *p_src);
MMP_ERR MMPF_SDIO_ReadMultiReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE fun_num, MMP_UBYTE blk_mode, MMP_UBYTE op_code, MMP_ULONG reg_addr, MMP_ULONG count, MMP_ULONG blk_size, MMP_UBYTE *p_dst);
//MMP_ERR MMPF_SDIO_ReadMultiRegDirect(stSDMMCHandler *SDMMCArg, MMP_UBYTE fun_num, MMP_UBYTE blk_mode, MMP_UBYTE op_code, MMP_ULONG reg_addr, MMP_ULONG count, MMP_ULONG blk_size, MMP_UBYTE *p_dst);
stSDMMCHandler *MMPF_SDIO_GetHandler(MMP_ULONG id);
void MMPF_SDIO_EnaISR(stSDMMCHandler *SDMMCArg, MMP_USHORT enable);
void MMPF_SDIO_RegSDIOCBFunc(stSDMMCHandler *SDMMCArg, void *Func);
MMP_ERR MMPF_SDIO_EnableOutputClk(stSDMMCHandler *SDMMCArg, MMP_USHORT enable);
#endif
/// @}
