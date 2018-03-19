/** @file mmpf_sd.c
@brief Driver functions of SD/MMC/SDIO
@author Philip Lin
@author Sunny
@author Hans Liu
@author Penguin Torng
@version 2.0
*/

#include "includes_fw.h"
#include "config_fw.h"
#include "lib_retina.h"
#include "mmp_reg_pad.h"
#include "mmp_reg_gbl.h"
#include "mmpf_system.h"
#include "mmpf_pll.h"
#include "mmpf_sd.h"
#if (EN_CARD_DETECT) || (EN_CARD_WRITEPROTECT) || (EN_CARD_PWRCTL)
#include "mmp_reg_gpio.h"
#endif
#if (ENABLE_SD_READTOSRAM)
#include "mmpf_dma.h"
#endif

stSDMMCHandler m_SDMMCArg[MMPF_SD_DEV_NUM];
MMP_ULONG m_ulSDG0SrcClk;   //wilson@110509: to make sure source clock won't exceed simulation result;
MMP_UBYTE gStorageSerialNumber[16];


MMP_UBYTE mGblSdInited = MMP_FALSE;
/// This vairable is ONLY temp buffer for SDHC switch function (ACMD51 + CMD6)
MMP_ULONG m_ulSDDmaAddr;

#if (ENABLE_SD_READTOSRAM)
#if (OS_TYPE == OS_UCOSII)
#if defined(ALL_FW)&&(CHIP == MCR_V2)
#pragma arm section zidata = "SD_WORKAROUND"
static MMP_UBYTE    m_ubSDMMCWorkaroundBuf[MMPF_SD_DEV_NUM][SD_TEMP_SIZE];
#pragma arm section zidata
#endif
#endif
#if (OS_TYPE == OS_LINUX)
static MMP_UBYTE    m_ubSDMMCWorkaroundBuf[MMPF_SD_DEV_NUM][SD_TEMP_SIZE];
#endif
#endif // (ENABLE_SD_READTOSRAM)

#if (!USING_SD_IF) /* Dummy API when no SD IF */
#if 0
void __sDEFINE_DUMMY_API__(){return;} //dummy
#endif
MMP_ERR MMPF_SD_ReadSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount){return 7;}
MMP_ERR MMPF_SD_WriteSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount){return 7;}
MMP_ERR MMPF_SD_GetSize(stSDMMCHandler *SDMMCArg, MMP_ULONG *pSize){*pSize = 0;return 1;}
MMP_ERR MMPF_SD_InitialInterface(stSDMMCHandler *SDMMCArg){/* RTNA_DBG_Str(1, "dummy InitSDIF\r\n"); */return 1;}
MMP_ERR MMPF_SD_Reset(stSDMMCHandler *SDMMCArg){/* RTNA_DBG_Str(1, "dummy ResetSD\r\n"); */return 1;}
MMP_ERR MMPF_MMC_Reset(stSDMMCHandler *SDMMCArg){return 1;}
MMP_ERR MMPF_SD_CheckCardStatus(stSDMMCHandler *SDMMCArg){return 1;}
MMP_ERR MMPF_SD_CheckCardIn(stSDMMCHandler *SDMMCArg){return 0;}
MMP_ERR MMPF_SD_CheckCardWP(stSDMMCHandler *SDMMCArg){return 1;}
void MMPF_SD_EnableCardPWR(stSDMMCHandler *SDMMCArg, MMP_BOOL enable){}
void MMPF_SD_ISR(void){}
#if 0
void __eDEFINE_DUMMY_API__(){return;} //dummy
#endif
#endif /* (!USING_SD_IF) */

/** @addtogroup MMPF_SD
@{
*/
#if (OS_TYPE == OS_UCOSII)
#if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
extern void MMPF_MMU_FlushDCacheMVA(MMP_ULONG ulRegion, MMP_ULONG ulSize);
#endif
#endif // (OS_TYPE == OS_UCOSII)

#if (USING_SD_IF)
    #define SD_TIMEOUT_COUNT     0x100000
    #define SD_CPU_NONBLOCKING   (1)
    MMP_ERR MMPF_SD_WaitIsrDone(stSDMMCHandler *SDMMCArg, MMP_ULONG waitcount);

#if (ENABLE_SDIO_FEATURE == 1)
typedef void SDIOCallBackFunc(stSDMMCHandler *SDMMCArg);
//SDIOCallBackFunc *MMPF_SDIO_CallBackISR = NULL;
#endif

/** @addtogroup MMPF_SDIO
@{
*/
#if (ENABLE_SDIO_FEATURE == 1)
    ///SDIO command argument (in CMD)
    #define RW_FLAG(x)      ((MMP_ULONG)x<<31)
    #define FUN_NUM(x)      ((MMP_ULONG)x<<28)
    #define RAW_FLAG(x)     ((MMP_ULONG)x<<27)
    #define REG_ADD(x)      ((MMP_ULONG)x<<9)
    #define WTITE_DATA(x)   ((MMP_ULONG)x)
    #define BLK_MODE(x)     ((MMP_ULONG)x<<27)
    #define OP_CODE(x)      ((MMP_ULONG)x<<26)

    ///SDIO return status (in DATA)
    #define COM_CRC_ERROR       0x8000
    #define ILLEGAL_COMMAND     0x4000
    #define UNKNOW_ERROR        0x0800
    #define INV_FUN_NUM         0x0200
    #define OUT_OF_RANGE        0x0100

    //MMPF_OS_SEMID mSDIOBusySemID[MMPF_SD_DEV_NUM];
    extern MMP_USHORT MMPF_SDIO_BusRelease(stSDMMCHandler *SDMMCArg);
    extern MMP_ERR MMPF_SDIO_BusAcquire(stSDMMCHandler *SDMMCArg);

    /// Add for SDIO NONBLOCKING CPU mode (pending for Block Transfer Done interrupt)
    #define SDIO_CPU_NONBLOCKING (1)

    extern MMPF_OS_FLAGID SYS_Flag_Hif;
#endif /* (ENABLE_SDIO_FEATURE==1) */
/** @} */ // MMPF_SDIO

extern MMPF_OS_FLAGID SYS_Flag_Hif;
extern MMP_ERR MMPF_SD_BusAcquire(stSDMMCHandler *SDMMCArg, MMP_ULONG ulTimeout);
extern MMP_ERR MMPF_SD_BusRelease(stSDMMCHandler *SDMMCArg);

#if (ENABLE_SD_CLKGATING == 1)
extern void MMPF_SD_EnableModuleClock(stSDMMCHandler *SDMMCArg, MMP_BOOL bEnable);
#endif

#if (ENABLE_READ_RETRY == 1)
static  MMP_BYTE    m_bReadRetryCnt[MMPF_SD_DEV_NUM];
#endif
#if (ENABLE_WRITE_RETRY == 1)
static  MMP_BYTE    m_bWriteRetryCnt[MMPF_SD_DEV_NUM];
#endif

////----------------------------------------------------------------------------
//  Function    : Return the AITPS_SD of the given id
//  Description : Helper function to get correct pSD from ID
//  Note        : Because we have SD hardware more than one.
//  Return      :
//------------------------------------------------------------------------------
static AITPS_SD GetpSD(MMPF_SD_ID id)
{
    #if (CHIP == P_V2)
    return (AITC_BASE_SD + id);
    #endif
    #if (CHIP == MCR_V2)||(CHIP == MERCURY)||(CHIP == VSN_V3)
    switch(id) {
    #if (CHIP == MCR_V2)
    case MMPF_SD_2:
        return AITC_BASE_SD2;
    #endif
    case MMPF_SD_1:
        return AITC_BASE_SD1;
    case MMPF_SD_0:
    default:
        return AITC_BASE_SD0;
    }
    #endif
}

stSDMMCHandler *MMPF_SD_GetHandler(MMPF_SD_ID SDId)
{
    return m_SDMMCArg + ((SDId < MMPF_SD_DEV_NUM)? SDId: 0);
}

void MMPF_SD_InitHandler(void)
{
    MMP_UBYTE i;
    static MMP_BOOL initialized = MMP_FALSE;

    // Only initial once after system power on
    if (initialized)
        return;

    for (i = 0; i < MMPF_SD_DEV_NUM; i++) {
        #if (ENABLE_READ_RETRY == 1)
        m_bReadRetryCnt[i] = 2;
        #endif
        #if (ENABLE_WRITE_RETRY == 1)
        m_bWriteRetryCnt[i] = 2;
        #endif
        m_SDMMCArg[i].id = i;
        m_SDMMCArg[i].pSD = GetpSD(i);
        m_SDMMCArg[i].bIsSD = MMP_FALSE;
        m_SDMMCArg[i].bIsHC = MMP_FALSE;
        m_SDMMCArg[i].ubSDHCHsm = MMP_FALSE;
        m_SDMMCArg[i].bIs2GMMC = MMP_FALSE;
        m_SDMMCArg[i].bIsMMC4X = MMP_FALSE;
        #if (ENABLE_DETECT_SDCLASS == 1)
        m_SDMMCArg[i].ubSDClass = 0xFF;  // Reserved Value
        #endif
        m_SDMMCArg[i].ubMMC4XClkMode = MMC_CLK_26M;

        #if (ENABLE_SD_SEQUENTIAL_OP == 1)
        //m_SDMMCArg[i].bEnableSequentialOp = MMP_FALSE;
        m_SDMMCArg[i].bSDNeedStopTransmit = MMP_FALSE;
        m_SDMMCArg[i].ulSDLastWriteSect = 0x00;
        #endif

        #if (CHIP == P_V2)
        m_SDMMCArg[i].ubSdPadMapping = i;
        #endif
        #if (CHIP == MERCURY)||(CHIP == VSN_V3)
        m_SDMMCArg[i].ubSdPadMapping = 1;
        #endif
        #if (CHIP == MCR_V2)
        m_SDMMCArg[i].ubSdPadMapping = 0; // set later by compiling option
        #endif
        m_SDMMCArg[i].ubSdBusWidth = 0x04;

        m_SDMMCArg[i].ulCardAddr = 0x00100000 + 0x10000*i;
        m_SDMMCArg[i].usCurClkDiv = SD_CLOCK_DIV_256;
        m_SDMMCArg[i].bPwrOnFirst = MMP_TRUE;
        m_SDMMCArg[i].usBlockLen = 0x200;
        m_SDMMCArg[i].bAccessFail = MMP_FALSE;

        m_SDMMCArg[i].ulCardSize = 0x00;
        m_SDMMCArg[i].ulReadTimeout = 0x00;
        m_SDMMCArg[i].ulWriteTimeout = 0x00;
        m_SDMMCArg[i].ulMMCBootSize = 0x00;
        m_SDMMCArg[i].ulMMCCurrentPartition = 0x00;
        m_SDMMCArg[i].IntTriggerSemID = 0xFF;
        #if (SD_BUS_REENTRY_PROTECT==1)
        m_SDMMCArg[i].BusySemID = 0xFF;
        #endif

        #if (EN_CARD_DETECT == 1)
        m_SDMMCArg[i].ubSDCardDetPinNum = 0xFF;
        m_SDMMCArg[i].ubSDCardDetPolarity = 0x01;
        #endif

        #if (EN_CARD_WRITEPROTECT == 1)
        m_SDMMCArg[i].ubSDCardWPPinNum = 0xFF;
        m_SDMMCArg[i].ubSDCardWPPolarity = 0x01;
        #endif

        #if (EN_CARD_PWRCTL == 1)
        m_SDMMCArg[i].ubSDCardPwrCtlPinNum = 0xFF;
        m_SDMMCArg[i].ubSDCardPwrCtlPolarity = 0x00;
        #endif

        #if (ENABLE_SDIO_FEATURE == 1)
        m_SDMMCArg[i].SDIOBusySemID = 0xFF;

        m_SDMMCArg[i].MMPF_SDIO_CallBackISR = NULL;
        #if (SDIO_SW_CLK_CTL == 1)
        m_SDMMCArg[i].ulSDIOClkCnt = 0;
        #endif
        #endif
        #if (ENABLE_SD_READTOSRAM)
        #if defined(ALL_FW)&&(CHIP == MCR_V2)
        m_SDMMCArg[i].ulTempSDAddr = (MMP_ULONG)m_ubSDMMCWorkaroundBuf[i];
        #endif
        #endif
    }

    initialized = MMP_TRUE;
    //you could add customize setting here, ex: pad mapping, bus width...etc;

}

#if (OS_TYPE == OS_UCOSII)
//------------------------------------------------------------------------------
//  Function    : MMPF_SD_ISR
//------------------------------------------------------------------------------
void MMPF_SD_ISR(void)
{
    AITPS_SD    pSD;
    MMP_UBYTE   tIdx = 0;
    MMP_USHORT  intSR;
    MMP_USHORT  intEN;
    #if (SD_CPU_NONBLOCKING == 1)
    MMP_UBYTE   ret;
    #endif

    do {
        pSD = m_SDMMCArg[tIdx].pSD;
        intSR = pSD->SD_CPU_INT_SR;
        intEN = pSD->SD_CPU_INT_EN;
        // Only check status for current enabled INT
        intSR &= intEN;

        #if 0
        MMPF_DBG_Int(tIdx, -2);
        RTNA_DBG_Str3(":");
        RTNA_DBG_Short3(intSR);
        RTNA_DBG_Str3(":");
        RTNA_DBG_Short3(intEN);
        RTNA_DBG_Str3(" = tIdx:intSR:intEN of SD_INT.\r\n");
        #endif

        if (intSR & (CMD_SEND_DONE) ) {
            #if (SD_CPU_NONBLOCKING == 1)
            ret = MMPF_OS_ReleaseSem(m_SDMMCArg[tIdx].IntTriggerSemID);
            if ( ret!= OS_NO_ERR) {
                RTNA_DBG_Str1(":SD OSSemPost 1 fail\r\n");
                #if 0
                RTNA_DBG_Short3(tIdx);
                RTNA_DBG_Str3(":");
                RTNA_DBG_Long3(ret);
                RTNA_DBG_Str3("= id: retVal.\r\n");
                #endif
            }
            #endif /* (SD_CPU_NONBLOCKING == 1) */

            pSD->SD_CPU_INT_EN &= ~(CMD_SEND_DONE);
        }

        if (intSR & DATA_SEND_DONE) {

            #if (SD_CPU_NONBLOCKING == 1)
            ret = MMPF_OS_ReleaseSem(m_SDMMCArg[tIdx].IntTriggerSemID);
            if ( ret!= OS_NO_ERR) {
                RTNA_DBG_Str1(":SD OSSemPost 2 fail\r\n");
                #if 0
                RTNA_DBG_Short3(tIdx);
                RTNA_DBG_Str3(":");
                RTNA_DBG_Long3(ret);
                RTNA_DBG_Str3("= id: retVal.\r\n");
                #endif
            }
            #endif /* (SD_CPU_NONBLOCKING == 1) */

            pSD->SD_CPU_INT_EN &= (~DATA_SEND_DONE);
        }

        #if (ENABLE_SDIO_FEATURE == 1)
        if(intSR & SDIO_INT) {
            //RTNA_DBG_Str3("\r\nSDIO_RX\r\n");
            #if defined(MDTV_FW)
            MMPF_OS_SetFlags(SYS_Flag_Hif, SYS_FLAG_SDIO_RX, MMPF_OS_FLAG_SET);
            #endif
            if (m_SDMMCArg[tIdx].MMPF_SDIO_CallBackISR)
                m_SDMMCArg[tIdx].MMPF_SDIO_CallBackISR( &m_SDMMCArg[tIdx]);

            pSD->SD_CPU_INT_SR = SDIO_INT;
            //pSD->SD_CPU_INT_EN &= (~SDIO_INT);
        }
        #endif

        #if 0
        RTNA_DBG_Short3(pSD->SD_CPU_INT_SR);
        RTNA_DBG_Str3(":");
        RTNA_DBG_Short3(pSD->SD_CPU_INT_EN);
        RTNA_DBG_Str3(" = SD_CPU_INT_SR:SD_CPU_INT_EN of SD_INT.\r\n");
        #endif
        tIdx++;

    } while (tIdx < MMPF_SD_DEV_NUM);

} /* MMPF_SD_ISR */

MMP_ERR  MMPF_SD_SoftwareResetDevice(stSDMMCHandler *SDMMCArg)
{
    AITPS_GBL    pGBL = AITC_BASE_GBL;

    #if (CHIP == P_V2)||(CHIP == VSN_V3)
    switch(SDMMCArg->id) {
    case 0:
        #if (CHIP == P_V2)
        pGBL->GBL_RST_SW_EN = GBL_MODULE_SD_CTL0;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_SW_DIS = GBL_MODULE_SD_CTL0;
        #endif
        #if (CHIP == VSN_V3)
        pGBL->GBL_RST_CTL2 |= GBL_SD0_RST;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_CTL2 &= ~(GBL_SD0_RST);
        #endif
        break;

    case 1:
        #if (CHIP == P_V2)
        pGBL->GBL_RST_SW_EN_2 = GBL_MODULE_SD_CTL1;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_SW_DIS_2 = GBL_MODULE_SD_CTL1;
        #endif
        #if (CHIP == VSN_V3)
        pGBL->GBL_RST_CTL3 |= GBL_SD1_RST;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_CTL3 &= ~(GBL_SD1_RST);
        #endif
        break;
    #if (CHIP == P_V2)
    case 2:
        pGBL->GBL_RST_SW_EN_2 = GBL_MODULE_SD_CTL2;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_SW_DIS_2 = GBL_MODULE_SD_CTL2;
        break;
     case 3:
        pGBL->GBL_RST_SW_EN_2 = GBL_MODULE_SD_CTL3;
        RTNA_WAIT_MS(1);
        pGBL->GBL_RST_SW_DIS_2 = GBL_MODULE_SD_CTL3;
        break;
    #endif
    }
    #endif //(CHIP == P_V2)||(CHIP == VSN_V3)
	
    #if (CHIP == MERCURY)
	TODO
	#endif

    #if (CHIP == MCR_V2)
    pGBL->GBL_SW_RST_EN[0] = (GBL_RST_SD0 << SDMMCArg->id);
    RTNA_WAIT_MS(1);
    pGBL->GBL_SW_RST_DIS[0] = (GBL_RST_SD0 << SDMMCArg->id);
    #endif

    return MMP_ERR_NONE;
}

/** @brief Write sector into card

This function send command sequence to write data into card at specific sector.
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] dmastartaddr Destination address for SD DMA address for outgoing data
@param[in] startsect Start sector number for writing card
@param[in] sectcount Sector count for writing card
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_BUSY SD bus was occupied
@retval MMP_SD_ERR_COMMAND_FAILED command error
@retval MMP_SD_ERR_DATA command OK, but data error
@retval MMP_SD_ERR_CARD_REMOVED card removed
*/
MMP_ERR  MMPF_SD_WriteSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  usSR;
    #if (ENABLE_WRITE_RETRY == 1)
    MMP_ERR     err;
    #endif

#if 0
    RTNA_DBG_Str0("SyncWRITE V1: startsect : count = ");
    RTNA_DBG_Long0(dmastartaddr);
    RTNA_DBG_Str0(":");
    RTNA_DBG_Long0(startsect);
    RTNA_DBG_Str0(":");
    RTNA_DBG_Long0(sectcount);
    RTNA_DBG_Str0("\r\n");
#endif
#if 0
    DBG_S3("id:glsdCardAddr[id] = ");
    DBG_B3(SDMMCArg->id);
    DBG_S3(":");
    DBG_L3(SDMMCArg->ulCardAddr);
    DBG_S3("\r\n");
#endif

    if (SDMMCArg->bAccessFail == MMP_TRUE) {
        RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
        RTNA_DBG_Str0(" SD access fail\r\n");
        return MMP_SD_ERR_CARD_REMOVED;
    }

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMP_ERR_NONE != MMPF_SD_BusAcquire(SDMMCArg, 0)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
        if ((SDMMCArg->bSDNeedStopTransmit == MMP_TRUE) &&
            ((SDMMCArg->ulSDLastWriteSect!= startsect)))
        {
            if (MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0))
            {
                #if (ENABLE_SD_CLKGATING==1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                    if (MMPF_SD_BusRelease(SDMMCArg))
                    {
                        return MMP_SD_ERR_BUSY;
                    }
                #endif

                return MMP_SD_ERR_COMMAND_FAILED;
            }
            SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
        }
    #endif

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
    #else
        if (SDMMCArg->bIsSD && sectcount != 1) {
            // ACMD 23
            if (MMPF_SD_SendCommand(SDMMCArg, APP_CMD, SDMMCArg->ulCardAddr)) {
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                if (MMPF_SD_BusRelease(SDMMCArg)) {
                    return MMP_SD_ERR_BUSY;
                }
                #endif
                RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
                RTNA_DBG_Str0(" CMD55 fail\r\n");
                return MMP_SD_ERR_COMMAND_FAILED;
            }

            if (MMPF_SD_SendCommand(SDMMCArg, SET_WR_BLK_ERASE_COUNT, (MMP_ULONG)sectcount)) {
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                if (MMPF_SD_BusRelease(SDMMCArg)) {
                    return MMP_SD_ERR_BUSY;
                }
                #endif
                RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
                RTNA_DBG_Str0("ACMD23 fail\r\n");
                return MMP_SD_ERR_COMMAND_FAILED;
            }
        }
    #endif

    pSD->SD_BLK_NUM = sectcount;
    pSD->SD_DMA_ST_ADDR = dmastartaddr;

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
        if (SDMMCArg->bSDNeedStopTransmit == MMP_TRUE) {
            pSD->SD_CMD_ARG = SDMMCArg->ulCardAddr;
        }
        else {
    #endif
            if((SDMMCArg->bIsHC ==1) || (SDMMCArg->bIs2GMMC == 1)) {
                pSD->SD_CMD_ARG = startsect;
            }
            else {
                pSD->SD_CMD_ARG = startsect << 9;
            }
    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
        }
    #endif

    MMPF_SD_SwitchTimeout(SDMMCArg, MMPF_SD_WRITE_TIMEOUT);
    // Clear all previous status
    pSD->SD_CPU_INT_SR = (DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE|CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|CMD_SEND_DONE);

    #if (ENABLE_SD_SEQUENTIAL_OP == 0)
    if (sectcount == 1)
    {
        pSD->SD_CMD_REG_0 = WRITE_BLOCK | OTHER_RESP;
        pSD->SD_CMD_REG_1 = ADTC_WRITE  | SEND_CMD;
    }
    else
    #endif
    {
        #if (ENABLE_SD_SEQUENTIAL_OP == 1)
            if (SDMMCArg->bSDNeedStopTransmit == MMP_TRUE)
            {
                pSD->SD_CMD_REG_0 = SEND_STATUS | OTHER_RESP;
                pSD->SD_CMD_REG_1 = ADTC_WRITE | SEND_CMD;

                SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
            }
            else
            {
        #endif
                pSD->SD_CMD_REG_0 = WRITE_MULTIPLE_BLOCK | OTHER_RESP;
                pSD->SD_CMD_REG_1 = ADTC_WRITE  | SEND_CMD;
        #if (ENABLE_SD_SEQUENTIAL_OP == 1)
            }
        #endif
    }
    while(!(pSD->SD_CPU_INT_SR & CMD_SEND_DONE));

    /* modified by rftorng 20090113. when command have errors, hardware will not issue dma transfer, so
    driver should return from here.*/
    if (pSD->SD_CPU_INT_SR & (CMD_RESP_CRC_ERR | CMD_RESP_TOUT | BUSY_TOUT)) {
        SDMMCArg->bAccessFail = MMP_TRUE;

        MMPF_SD_SoftwareResetDevice(SDMMCArg);

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT==1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        RTNA_DBG_Short0(pSD->SD_CPU_INT_SR);
        RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
        RTNA_DBG_Str0(" CMD24 or CMD25 fail\r\n");
        return  MMP_SD_ERR_CARD_REMOVED;
    }


    #if (SD_CPU_NONBLOCKING == 1)
    pSD->SD_CPU_INT_EN = DATA_SEND_DONE;
    #endif

    #if (EN_CARD_DETECT == 1)
    if (!MMPF_SD_CheckCardIn(SDMMCArg)) {
        MMPF_HIF_SetCmdStatus(SD_CARD_NOT_EXIST);

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT==1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return  MMP_SD_ERR_CARD_REMOVED;
    }
    #endif

    #if (SD_CPU_NONBLOCKING == 1)
    if (MMPF_SD_WaitIsrDone(SDMMCArg, 0)) {
        RTNA_DBG_Str1("write acquire fail\r\n");
        if(MMPF_SD_CheckCardStatus(SDMMCArg)) {
            MMPF_HIF_SetCmdStatus(SD_CARD_NOT_EXIST);

            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            #if (SD_BUS_REENTRY_PROTECT == 1)
            if (MMPF_SD_BusRelease(SDMMCArg)) {
                return MMP_SD_ERR_BUSY;
            }
            #endif
            return MMP_SD_ERR_DATA;
        }
    }
    #else
    // Confirm DATA sent, otherwise skip when DATA error.
    while( !(pSD->SD_CPU_INT_SR & (DATA_SEND_DONE|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR)) );
    #endif

    ///After data done (DATA_SEND_DONE)
    usSR = pSD->SD_CPU_INT_SR;
    pSD->SD_CPU_INT_SR = usSR;

    ///Check the last CMD/DATA status
    if(usSR & (CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR)) {
        RTNA_DBG_Short0(usSR);
        RTNA_DBG_Str0(":write sector error\r\n");
        if(usSR & (DATA_CRC_ERR|DATA_TOUT)) {
            #if (ENABLE_WRITE_RETRY == 1)
            #if (ENABLE_SD_SEQUENTIAL_OP == 0)
            if (sectcount != 1)
            #endif
            {
                if (MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0)) {
                    #if (ENABLE_SD_CLKGATING == 1)
                    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                    #endif

                    #if (SD_BUS_REENTRY_PROTECT == 1)
                    if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                        RTNA_DBG_Str0("sectcnt != 1 bus release\r\n");
                        return MMP_SD_ERR_BUSY;
                    }
                    #endif
                    return MMP_SD_ERR_COMMAND_FAILED;
                }
            }
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            #if (SD_BUS_REENTRY_PROTECT == 1)
            if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                RTNA_DBG_Str0("bus release err\r\n");
                return MMP_SD_ERR_BUSY;
            }
            #endif
            if (m_bWriteRetryCnt[SDMMCArg->id] > 0) {
                m_bWriteRetryCnt[SDMMCArg->id]--;
                err = MMPF_SD_WriteSector(SDMMCArg, dmastartaddr, startsect, sectcount);
                RTNA_DBG_Short0(sectcount);
                RTNA_DBG_Long0(err);
                RTNA_DBG_Str0(" sd wr again\r\n");
                m_bWriteRetryCnt[SDMMCArg->id] = 2;
                return err;
            }
            else {
                #if (SD_BUS_REENTRY_PROTECT == 1)
                if (MMP_ERR_NONE != MMPF_SD_BusAcquire(SDMMCArg, 0)) {
                    return MMP_SD_ERR_BUSY;
                }
                #endif
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
                #endif
                err = MMPF_SD_SendCommand(SDMMCArg, SEND_STATUS, SDMMCArg->ulCardAddr);
                if (err) {
                    RTNA_DBG_Long0(err);
                    RTNA_DBG_Str0(" Get card status failed\r\n");
                }
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif
                #if (SD_BUS_REENTRY_PROTECT == 1)
                if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                    RTNA_DBG_Str0("bus release err\r\n");
                    return MMP_SD_ERR_BUSY;
                }
                #endif
                return MMP_SD_ERR_DATA;
            }
            #else
            MMPF_SD_SoftwareResetDevice(SDMMCArg);
            #endif
        }
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        RTNA_DBG_Short0(usSR);
        RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
        RTNA_DBG_Str0(" data error\r\n");
        return MMP_SD_ERR_DATA;
    }
    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
    #else
    if (sectcount != 1)
    #endif
    {
        #if (ENABLE_SD_SEQUENTIAL_OP == 1)
            SDMMCArg->ulSDLastWriteSect = startsect + sectcount;
            SDMMCArg->bSDNeedStopTransmit = MMP_TRUE;
        #else
            if(MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0)) {
                #if (ENABLE_SD_CLKGATING==1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                if (MMPF_SD_BusRelease(SDMMCArg)) {
                    return MMP_SD_ERR_BUSY;
                }
                #endif
                RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
                RTNA_DBG_Str0(" stop error\r\n");
                return MMP_SD_ERR_COMMAND_FAILED;
            }
            usSR = pSD->SD_CPU_INT_SR;
            if (usSR & (CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR)) {
                RTNA_DBG_Short0(usSR);
                RTNA_DBG_Long0(pSD->SD_RESP.D[3]);
                RTNA_DBG_Str0(" stop error\r\n");
                return MMP_SD_ERR_DATA;
            }
        #endif
    }

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
    #endif

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMPF_SD_BusRelease(SDMMCArg)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif
    return MMP_ERR_NONE;
}

/** @brief Read sector from card

This function send command sequence to read data from card at specific sector.
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] dmastartaddr Destination address for SD DMA address for incoming data
@param[in] startsect Start sector number for reading card
@param[in] sectcount Sector count for reading card
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_BUSY SD bus was occupied
@retval MMP_SD_ERR_COMMAND_FAILED command error
@retval MMP_SD_ERR_DATA command OK, but data error
@retval MMP_SD_ERR_CARD_REMOVED card removed
*/
MMP_ERR  MMPF_SD_ReadPhysicalSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  usSR;
#if 0
    RTNA_DBG_Str3("SyncREAD V1: startsect : count = ");
    RTNA_DBG_Long3(dmastartaddr);
    RTNA_DBG_Str3(":");
    RTNA_DBG_Long3(startsect);
    RTNA_DBG_Str3(":");
    RTNA_DBG_Long3(sectcount);
    RTNA_DBG_Str3("\r\n");
#endif
#if 0
    RTNA_DBG_Str0("id:glsdCardAddr[id] = ");
    RTNA_DBG_Byte0(SDMMCArg->id);
    RTNA_DBG_Str0(":");
    RTNA_DBG_Long0(SDMMCArg->ulCardAddr);
    RTNA_DBG_Str0("\r\n");
#endif

    if (SDMMCArg->bAccessFail == MMP_TRUE) {
        return  MMP_SD_ERR_CARD_REMOVED;
    }

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMP_ERR_NONE != MMPF_SD_BusAcquire(SDMMCArg, 0)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
        if (SDMMCArg->bSDNeedStopTransmit == MMP_TRUE)
        {
            if (MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0))
            {
                #if (ENABLE_SD_CLKGATING==1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                    if (MMPF_SD_BusRelease(SDMMCArg))
                    {
                        return MMP_SD_ERR_BUSY;
                    }
                #endif

                return MMP_SD_ERR_COMMAND_FAILED;
            }
            SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
        }
    #endif

    MMPF_SD_SwitchTimeout(SDMMCArg, MMPF_SD_READ_TIMEOUT);

    pSD->SD_BLK_NUM = sectcount;

    pSD->SD_DMA_ST_ADDR = dmastartaddr;
    if((SDMMCArg->bIsHC ==1) || (SDMMCArg->bIs2GMMC == 1)) {
        pSD->SD_CMD_ARG = startsect;
    }
    else {
        pSD->SD_CMD_ARG = startsect << 9;
    }

    pSD->SD_CPU_INT_SR = (DATA_CRC_ERR | DATA_TOUT | DATA_ST_BIT_ERR | DATA_SEND_DONE |
                            CMD_RESP_CRC_ERR | CMD_RESP_TOUT | BUSY_TOUT | CMD_SEND_DONE);

    if (sectcount == 1) {
        pSD->SD_CMD_REG_0 = READ_SINGLE_BLOCK | OTHER_RESP;
        pSD->SD_CMD_REG_1 = ADTC_READ |SEND_CMD;
    }
    else {
        pSD->SD_CMD_REG_0 = READ_MULTIPLE_BLOCK | OTHER_RESP;
        pSD->SD_CMD_REG_1 = ADTC_READ |SEND_CMD;
    }

    while(!(pSD->SD_CPU_INT_SR & CMD_SEND_DONE));
    /* modified by rftorng 20090113. when command have errors, hardware will not issue dma transfer, so
    driver should return from here.*/
    if (pSD->SD_CPU_INT_SR & (CMD_RESP_CRC_ERR | CMD_RESP_TOUT)) {
        SDMMCArg->bAccessFail = MMP_TRUE;

        MMPF_SD_SoftwareResetDevice(SDMMCArg);

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return  MMP_SD_ERR_CARD_REMOVED;
    }
    else {
        #if (SD_CPU_NONBLOCKING == 1)
        pSD->SD_CPU_INT_EN = DATA_SEND_DONE;
        #endif
    }
    #if (EN_CARD_DETECT == 1)
    if (!MMPF_SD_CheckCardIn(SDMMCArg)) {
        MMPF_HIF_SetCmdStatus(SD_CARD_NOT_EXIST);

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT==1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return  MMP_SD_ERR_CARD_REMOVED;
    }
    #endif

    #if (SD_CPU_NONBLOCKING == 1)
    if  (MMPF_SD_WaitIsrDone(SDMMCArg, 0)) {
        RTNA_DBG_Str1("read acquire fail\r\n");
        if(MMPF_SD_CheckCardStatus(SDMMCArg)) {
            MMPF_HIF_SetCmdStatus(SD_CARD_NOT_EXIST);

            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            #if (SD_BUS_REENTRY_PROTECT == 1)
            if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                return MMP_SD_ERR_BUSY;
            }
            #endif
            return MMP_SD_ERR_DATA;
        }
    }

    #else
    // Confirm DATA sent, otherwise skip when DATA error.
    while( !(pSD->SD_CPU_INT_SR & (DATA_SEND_DONE|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR)) );
    #endif

    ///After data done (DATA_SEND_DONE)
    usSR = pSD->SD_CPU_INT_SR;
    pSD->SD_CPU_INT_SR = usSR;

    ///Check the last CMD/DATA status
    if(usSR & (CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR)) {
        DBG_W2(usSR);
        DBG_S2(":read sector error\r\n");
        if(usSR & (DATA_CRC_ERR|DATA_TOUT)) {
            #if (ENABLE_READ_RETRY == 1)
            if (sectcount != 1) {
                if (MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0)) {
                    #if (ENABLE_SD_CLKGATING == 1)
                    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                    #endif

                    #if (SD_BUS_REENTRY_PROTECT == 1)
                    if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                        RTNA_DBG_Str0("sectcnt != 1 bus release\r\n");
                        return MMP_SD_ERR_BUSY;
                    }
                    #endif
                    return MMP_SD_ERR_COMMAND_FAILED;
                }
            }
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            #if (SD_BUS_REENTRY_PROTECT == 1)
            if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                RTNA_DBG_Str0("sectcnt != 1 bus release\r\n");
                return MMP_SD_ERR_BUSY;
            }
            #endif
            return MMP_SD_ERR_DATA;
            #else
            MMPF_SD_SoftwareResetDevice(SDMMCArg);
            #endif
        }
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif

        return MMP_SD_ERR_DATA;
    }

    if (sectcount != 1) {
        if(MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0)) {

            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            #if (SD_BUS_REENTRY_PROTECT == 1)
            if (MMP_ERR_NONE != MMPF_SD_BusRelease(SDMMCArg)) {
                return MMP_SD_ERR_BUSY;
            }
            #endif
            return MMP_SD_ERR_COMMAND_FAILED;
        }
    }

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
    #endif
    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMPF_SD_BusRelease(SDMMCArg)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif

    return MMP_ERR_NONE;
}

#if (ENABLE_SD_READTOSRAM)
/** @brief Callback when DMA move done

For SD read bandwidth issue, the destination buffer of SD read is set to
SRAM always. Then copy sectors data from SRAM to our target address by
Move DMA engine.
Whenever DMA move done, this callback function will be executed.
*/
void  MMPF_SD_DMACopyCallBack(void* argument)
{
    stSDMMCHandler *SDMMCArg = (stSDMMCHandler *)argument;

    if (MMPF_OS_ReleaseSem(SDMMCArg->DMASemID) != OS_NO_ERR)
        RTNA_DBG_Str(0, "Release failed SD DMA semaphore failed");
}
#endif

/** @brief Read sector from card

This function send command sequence to read data from card at specific sector.
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] dmastartaddr Destination address for SD DMA address for incoming data
@param[in] startsect Start sector number for reading card
@param[in] sectcount Sector count for reading card
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_BUSY SD bus was occupied
@retval MMP_SD_ERR_COMMAND_FAILED command error
@retval MMP_SD_ERR_DATA command OK, but data error
@retval MMP_SD_ERR_CARD_REMOVED card removed
*/
MMP_ERR  MMPF_SD_ReadSector(stSDMMCHandler *SDMMCArg, MMP_ULONG dmastartaddr, MMP_ULONG startsect, MMP_USHORT sectcount)
{
    MMP_ERR err = MMP_ERR_NONE;
    MMP_ULONG tempDMAAddr = 0;
    MMP_USHORT usRemainSector = sectcount;
    MMP_USHORT usCurrentSector = 0;
    MMP_USHORT usSectorOffset = 0;
    MMP_UBYTE i = 0;

    #if (ENABLE_SD_READTOSRAM)
    tempDMAAddr = SDMMCArg->ulTempSDAddr;
    if (usRemainSector > (SD_TEMP_SIZE >> 9)) {
        usCurrentSector = (SD_TEMP_SIZE >> 9);
    }
    else {
        usCurrentSector = sectcount;
    }
    #else
    tempDMAAddr = dmastartaddr;
    usCurrentSector = sectcount;
    #endif

    while(1) {
        #if (ENABLE_READ_RETRY == 1)
        for (i = 0; i < m_bReadRetryCnt[SDMMCArg->id]; i++) {
            err = MMPF_SD_ReadPhysicalSector(SDMMCArg, tempDMAAddr, (startsect + usSectorOffset), usCurrentSector);
            if (err == MMP_ERR_NONE)
                break;
        }
        if (err != MMP_ERR_NONE) {
            return err;
        }

        #if (ENABLE_SD_READTOSRAM)
        // Start DMA copy
        MMPF_DMA_MoveData(tempDMAAddr, (dmastartaddr + (usSectorOffset << 9)), (usCurrentSector<< 9), MMPF_SD_DMACopyCallBack, SDMMCArg, 0, 0);
        MMPF_OS_AcquireSem(SDMMCArg->DMASemID, 0);
        usSectorOffset += usCurrentSector;
        #endif

        usRemainSector -= usCurrentSector;
        if (usRemainSector == 0) {
            break;
        }

        #if (ENABLE_SD_READTOSRAM)
        if (usRemainSector <= (SD_TEMP_SIZE >> 9)) {
            usCurrentSector = usRemainSector;
        }
        else {
            usCurrentSector = (SD_TEMP_SIZE >> 9);
        }
        #endif
        #endif
    }

    return err;
} /* MMPF_SD_ReadSector */

/** @brief Control working clock for SD module

This function control working clock for SD module
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] usClkDiv prefer working clock
@return NONE
*/
void MMPF_SD_SwitchClock(stSDMMCHandler *SDMMCArg, MMP_USHORT usClkDiv)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_ULONG   time_out = 0;

    // Wait for SD clk can be switched
    while(!(pSD->SD_CPU_FIFO_SR & CLK_SWITCH_INDICATE)) {
        time_out++;
        if (time_out >= SD_TIMEOUT_COUNT) {
            #if 0
            RTNA_DBG_Byte2(pSD->SD_CPU_FIFO_SR);
            RTNA_DBG_Str2("(");
            RTNA_DBG_Long2((unsigned long) &(pSD->SD_CPU_FIFO_SR));
            RTNA_DBG_Str2("):clk switch fail\r\n");
            #endif
            RTNA_DBG_Str(0, "ERROR: SD switch clock timeout\r\n");
            return;
        }
    }

    // Assign new SD clock
    if(usClkDiv == SD_CLOCK_DIV_1) {
        pSD->SD_CTL_0 &= 0xF3;      //Wilson@100120
        pSD->SD_CTL_0 |= CLK_IN_NORM_SYS;
    }
    else if (usClkDiv == SD_CLOCK_DIV_3) {
        pSD->SD_CTL_0 |= CLK_IN_SYS_DIV_3;
    }
    else {
        pSD->SD_CTL_0 &= 0xF3;      //Wilson@100120
        pSD->SD_CLK_CTL = usClkDiv;
    }

    SDMMCArg->usCurClkDiv = usClkDiv;

    return;
} /* MMPF_SD_SwitchClock */

/** @brief Control working clock for SD module by G0 clock

This function control working clock for SD module according to G0 clock
@param[in] ulG0Clock the clock frequency of group 0
@return NONE
*/
void MMPF_SD_ConfigClock(stSDMMCHandler *SDMMCArg, MMP_ULONG ulG0Clock, MMP_ULONG ulOutputClock)
{
    #if (CHIP == P_V2)
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ULONG targetClk, divValue;
    #endif
    MMP_ULONG ulStorageClk;
    MMP_ULONG uldivider = 1;

    #if (CHIP == P_V2)
    //wilson@110509: setting clk divide value before enable SD module clock
    targetClk = 100*1000;   //100Mhz
    divValue = (ulG0Clock - (ulG0Clock % targetClk))/targetClk;
    if (ulG0Clock % targetClk != 0)
        divValue++;
    ulG0Clock /= divValue;
    pGBL->GBL_CLK_0_LCD_SD = ((pGBL->GBL_CLK_0_LCD_SD)&0x0F) | (GBL_CLK_SD_DIV(divValue));
    #endif
    m_ulSDG0SrcClk = ulG0Clock;

    #if (SD_MMC_CLK_DBG_MSG == 1)
    RTNA_DBG_Str0("SD/MMC clock src:");
    RTNA_DBG_Long0(ulG0Clock);
    RTNA_DBG_Str0("\r\n");
    #endif
    if(ulOutputClock != 0) {
        ulStorageClk = ulOutputClock;
    }
    else
    {
        if (SDMMCArg->bIsSD == MMP_FALSE) { //SD
            if (SDMMCArg->ubSDHCHsm == MMP_TRUE) {
                ulStorageClk = DEFINE_SD_HIGHSPEED;
            }
            else {
                ulStorageClk = DEFINE_SD_FULLSPEED;
            }
        }
        else if (SDMMCArg->bIsMMC4X == 1) { //MMC 4x
            if (SDMMCArg->ubMMC4XClkMode > MMC_CLK_52M) {
                ulStorageClk = DEFINE_MMC_HIGHSPEED;
            }
            else {
                ulStorageClk = DEFINE_MMC_FULLSPEED;
            }
        }
        else {
            ulStorageClk = DEFINE_MMC_FULLSPEED;
        }
    }

    uldivider = (ulG0Clock + ulStorageClk - 1)/ulStorageClk;
    if (uldivider == 1) {
        MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
    }
    else if (uldivider == 3) {
        MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
    }
    else {
        if (uldivider % 2)
            uldivider++;
        MMPF_SD_SwitchClock(SDMMCArg, (uldivider >> 1) - 1);
    }

    #if (SD_MMC_CLK_DBG_MSG == 1)
    RTNA_DBG_Str0("Current SD/MMC clock:");
    RTNA_DBG_Long0(ulG0Clock/uldivider);
    RTNA_DBG_Str0("\r\n");
    #endif

    return;
} /* MMPF_SD_SwitchClock */
#endif // (OS_TYPE == OS_UCOSII)

#if (ENABLE_SD_CLKGATING == 1)
/** @brief Control SD module clock

This function control SD module clock
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] bEnable 0 to turn-off, 1 to turn-on SD module clock
@return It reports the status of the operation. Now it always return success.
*/
void MMPF_SD_EnableModuleClock(stSDMMCHandler *SDMMCArg, MMP_BOOL bEnable)
{
    #if (CHIP == P_V2)
    MMPF_SYS_CLK    clk_id = MMPF_SYS_CLK_SD;
    #endif
    #if (CHIP == VSN_V3)
    MMPF_SYS_CLK    clk_id = (SDMMCArg->id)? MMPF_SYS_CLK_SD1: MMPF_SYS_CLK_SD0;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_SYS_CLK    clk_id = MMPF_SYS_CLK_SD0 + SDMMCArg->id;
    #endif

    MMPF_SYS_EnableClock(clk_id, bEnable);
} /* MMPF_SD_EnableModuleClock */
#endif

/** @brief Config PAD setting for SD host controller

This function config PAD setting for SD host controller and MUST be called before file system initialize.
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] padid ID of SD PAD. For DIAMOND, valid input is 0, 1 and 2.
@return It reports the status of the operation. Now it always return success.
*/
MMP_ERR MMPF_SD_ConfigPadMapping(stSDMMCHandler *SDMMCArg, MMPF_SD_PAD padid, MMP_BOOL b8bitBus)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    #if (CHIP == MCR_V2)
    AITPS_PAD   pPAD = AITC_BASE_PAD;
    MMP_BOOL    bPadCfgCorrect = MMP_TRUE;
    #endif

    #if (CHIP == VSN_V3)
    if ((SDMMCArg->id == MMPF_SD_0) && (padid == MMPF_SD_PAD1)) {
        pGBL->GBL_IO_CFG_PCGPIO[0] = 0<<5|1<<2; //PCGPIO10 ==> SD1_CLK , 9.6mA , pull-up
        pGBL->GBL_IO_CFG_PCGPIO[1] = 0<<5|1<<2; //PCGPIO11 ==> SD1_CMD, 9.6mA ,pull-up
        pGBL->GBL_IO_CFG_PCGPIO[2] = 0<<5|1<<2; //PCGPIO12 ==> SD1_DATA0, 8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO[3] = 0<<5|1<<2; //PCGPIO13 ==> SD1_DATA1,  8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO[4] = 0<<5|1<<2; //PCGPIO14 ==> SD1_DATA2,  8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO[5] = 0<<5|1<<2; //PCGPIO15 ==> SD1_DATA3,  8.4mA, pull-up

        pGBL->GBL_IO_CTL3 |= GBL_SD0_IO_PAD1_EN;
    }
    if ((SDMMCArg->id == MMPF_SD_1) && (padid == MMPF_SD_PAD1)) {
        pGBL->GBL_IO_CFG_PCGPIO6[4] = 0<<5|1<<2;    //PCGPIO10 ==> SD1_CLK , 9.6mA , pull-up
        pGBL->GBL_IO_CFG_PCGPIO6[5] = 0<<5|1<<2;    //PCGPIO11 ==> SD1_CMD, 9.6mA ,pull-up
        pGBL->GBL_IO_CFG_PCGPIO6[6] = 0<<5|1<<2;    //PCGPIO12 ==> SD1_DATA0, 8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO6[7] = 0<<5|1<<2;    //PCGPIO13 ==> SD1_DATA1,  8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO6[8] = 0<<5|1<<2;    //PCGPIO14 ==> SD1_DATA2,  8.4mA, pull-up
        pGBL->GBL_IO_CFG_PCGPIO6[9] = 0<<5|1<<2;    //PCGPIO15 ==> SD1_DATA3,  8.4mA, pull-up

        pGBL->GBL_IO_CTL4 |= GBL_SD1_IO_PAD1_EN;
    }
    #endif
	#if (CHIP == MERCURY)
	TODO
	#endif

    #if (CHIP == MCR_V2)
    bPadCfgCorrect = MMP_TRUE;

    if ((SDMMCArg->id == MMPF_SD_0) && (padid == MMPF_SD0_PAD0)) {
        pGBL->GBL_SD_PAD_CFG |= GBL_SD0_PAD_EN;
        if (b8bitBus == MMP_TRUE)
            bPadCfgCorrect = MMP_FALSE;
        pPAD->PAD_IO_CFG_PCGPIO[0] = 0x20; //SD_CLK 3.3V 14.4mA
        pPAD->PAD_IO_CFG_PCGPIO[1] = 0x24; //SD_CMD 3.3V 14.4mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[2] = 0x04; //SD_DAT0 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[3] = 0x04; //SD_DAT1 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[4] = 0x04; //SD_DAT2 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[5] = 0x04; //SD_DAT3 3.3V 12mA, pull-up
    }
    else if ((SDMMCArg->id == MMPF_SD_2) && (padid == MMPF_SD2_PAD0)) {
        pGBL->GBL_SD_PAD_CFG |= GBL_SD2_PAD_EN;
        if (b8bitBus == MMP_TRUE)
            bPadCfgCorrect = MMP_FALSE;
        pPAD->PAD_IO_CFG_PCGPIO[12] = 0x00; //SD_CLK 3.3V 14.4mA, ori=0x20
        pPAD->PAD_IO_CFG_PCGPIO[13] = 0x04; //SD_CMD 3.3V 14.4mA, pull-up, ori=0x24
        pPAD->PAD_IO_CFG_PCGPIO[14] = 0x04; //SD_DAT0 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[15] = 0x04; //SD_DAT1 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[16] = 0x04; //SD_DAT2 3.3V 12mA, pull-up
        pPAD->PAD_IO_CFG_PCGPIO[17] = 0x04; //SD_DAT3 3.3V 12mA, pull-up
    }
    else if (SDMMCArg->id == MMPF_SD_1) {
	 BUG_ON((padid - MMPF_SD1_PAD0)>1);
        pGBL->GBL_SD_PAD_CFG &= ~(GBL_SD1_PAD_MASK);
        pGBL->GBL_SD_PAD_CFG |= GBL_SD1_PAD(padid - MMPF_SD1_PAD0);
        if (padid == MMPF_SD1_PAD0) {
            pPAD->PAD_IO_CFG_PCGPIO[6] = 0x20; //SD_CLK 3.3V 14.4mA
            pPAD->PAD_IO_CFG_PCGPIO[7] = 0x24; //SD_CMD 3.3V 14.4mA, pull-up
            pPAD->PAD_IO_CFG_PCGPIO[8] = 0x04; //SD_DAT0 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PCGPIO[9] = 0x04; //SD_DAT1 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PCGPIO[10] = 0x04; //SD_DAT2 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PCGPIO[11] = 0x04; //SD_DAT3 3.3V 12mA, pull-up
        }
        else if (padid == MMPF_SD1_PAD1) {
            pPAD->PAD_IO_CFG_PBGPIO2[0] = 0x60; //SD_CLK 3.3V 14.4mA
            pPAD->PAD_IO_CFG_PBGPIO2[1] = 0x44; //SD_CMD 3.3V 14.4mA, pull-up
            pPAD->PAD_IO_CFG_PBGPIO2[2] = 0x44; //SD_DAT0 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PBGPIO2[3] = 0x44; //SD_DAT1 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PBGPIO2[4] = 0x44; //SD_DAT2 3.3V 12mA, pull-up
            pPAD->PAD_IO_CFG_PBGPIO2[5] = 0x44; //SD_DAT3 3.3V 12mA, pull-up
            if (b8bitBus) {
                pPAD->PAD_IO_CFG_PBGPIO[12] = 0x44; //SD_DAT4 3.3V 12mA, pull-up
                pPAD->PAD_IO_CFG_PBGPIO[13] = 0x44; //SD_DAT5 3.3V 12mA, pull-up
                pPAD->PAD_IO_CFG_PBGPIO[14] = 0x44; //SD_DAT6 3.3V 12mA, pull-up
                pPAD->PAD_IO_CFG_PBGPIO[15] = 0x44; //SD_DAT7 3.3V 12mA, pull-up
            }		
        }
        if ((b8bitBus == MMP_TRUE) && (padid != MMPF_SD1_PAD1))
            bPadCfgCorrect = MMP_FALSE;
    }
    else {
        bPadCfgCorrect = MMP_FALSE;
    }
    if (!bPadCfgCorrect) {
        RTNA_DBG_Str(0, "Invalid SD pad config!\r\n");
    }
    #endif //(CHIP == MCR_V2)

    return MMP_ERR_NONE;
}

/** @brief Initial SD module and interafce

This function initial SD module and interafce
@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@retval MMP_ERR_NONE Now it always return success.
*/
MMP_ERR  MMPF_SD_InitialInterface(stSDMMCHandler *SDMMCArg)
{
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC   pAIC = AITC_BASE_AIC;
    #endif // (OS_TYPE == OS_UCOSII)
    AITPS_SD    pSD = SDMMCArg->pSD;
    #if (CHIP == P_V2)
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    MMP_ULONG   divValue, targetClk;
    MMPF_SYS_CLK    clk_id = MMPF_SYS_CLK_SD;
    #endif
    #if (CHIP == VSN_V3)
    MMPF_SYS_CLK    clk_id = (SDMMCArg->id)? MMPF_SYS_CLK_SD1: MMPF_SYS_CLK_SD0;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_SYS_CLK    clk_id = MMPF_SYS_CLK_SD0 + SDMMCArg->id;
    #endif

    #if (EN_CARD_DETECT == 1)||(EN_CARD_WRITEPROTECT == 1)||(EN_CARD_PWRCTL == 1)
    AITPS_GPIO pGPIO = AITC_BASE_GPIO;
    #endif

    MMPF_SD_ConfigPadMapping(SDMMCArg, SDMMCArg->ubSdPadMapping,
                            (SDMMCArg->ubSdBusWidth == 8));

    if (mGblSdInited == MMP_FALSE) {
        #if (OS_TYPE == OS_UCOSII)
        RTNA_AIC_Open(pAIC, AIC_SRC_SD, sd_isr_a, AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
        RTNA_AIC_IRQ_En(pAIC, AIC_SRC_SD);
        #endif // (OS_TYPE == OS_UCOSII)

        mGblSdInited = MMP_TRUE;
    }

    if (SDMMCArg->bPwrOnFirst){
        #if (OS_TYPE == OS_UCOSII) //not be added back
        MMPF_SD_SoftwareResetDevice(SDMMCArg);

        #if (EN_CARD_PWRCTL == 1) //GPIO output
        if (SDMMCArg->ubSDCardPwrCtlPinNum != 0xFF) {
            pGPIO->GPIO_OUT_EN[(SDMMCArg->ubSDCardPwrCtlPinNum / 32)] |= (1 << (SDMMCArg->ubSDCardPwrCtlPinNum % 32));
        }
        MMPF_SD_EnableCardPWR(SDMMCArg, MMP_TRUE);
        #endif

        #if (EN_CARD_DETECT == 1) //GPIO input
        if (SDMMCArg->ubSDCardDetPinNum != 0xFF) {
            pGPIO->GPIO_OUT_EN[(SDMMCArg->ubSDCardDetPinNum / 32)] &= ~(1 << (SDMMCArg->ubSDCardDetPinNum % 32));
        }
        #endif

        #if (EN_CARD_WRITEPROTECT == 1) //GPIO input
        if (SDMMCArg->ubSDCardWPPinNum != 0xFF) {
            pGPIO->GPIO_OUT_EN[(SDMMCArg->ubSDCardWPPinNum / 32)] &= ~(1 << (SDMMCArg->ubSDCardWPPinNum % 32));
        }
        #endif

        /// Default state: no trigger
        #if (SD_CPU_NONBLOCKING == 1)
        SDMMCArg->IntTriggerSemID = MMPF_OS_CreateSem(0);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        /// Default state: SD BUS available
        SDMMCArg->BusySemID = MMPF_OS_CreateSem(1);
        #endif
        #if (ENABLE_SD_READTOSRAM == 1)
        SDMMCArg->DMASemID = MMPF_OS_CreateSem(0);
        #endif
        #endif // (OS_TYPE == OS_UCOSII)
        SDMMCArg->bPwrOnFirst = MMP_FALSE;

    }

    #if (OS_TYPE == OS_LINUX)
    MMPF_SYS_EnableClock(clk_id, MMP_TRUE);
    pSD->SD_CTL_1 |= CLK_EN; //temp workaround, fixme
    MMPF_SYS_EnableClock(clk_id, MMP_FALSE);
    #endif //OS_TYPE == OS_LINUX)

    #if (OS_TYPE == OS_UCOSII) //not be added back
    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &m_ulSDG0SrcClk);
    #if (CHIP == P_V2)
    //wilson@110509: setting clk divide value before enable SD module clock
    targetClk = 100*1000;   //100Mhz
    divValue = (m_ulSDG0SrcClk - (m_ulSDG0SrcClk % targetClk))/targetClk;
    if (m_ulSDG0SrcClk % targetClk != 0)
        divValue++;
    m_ulSDG0SrcClk /= divValue;
    pGBL->GBL_CLK_0_LCD_SD = ((pGBL->GBL_CLK_0_LCD_SD)&0x0F) | (GBL_CLK_SD_DIV(divValue));
    #endif

    // Enable SD module clock before access SD module's register
    MMPF_SYS_EnableClock(clk_id, MMP_TRUE);

    pSD->SD_CMD_RESP_TOUT_MAX = 0xFF;
    MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_NORMAL_MODE);
    MMPF_SD_SwitchTimeout(SDMMCArg, MMPF_SD_DEFAULT_TIMEOUT);
    #endif // (OS_TYPE == OS_UCOSII)
    return  MMP_ERR_NONE;
}



/** @brief SD set timeout mode

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] SD high speed mode or not.
@retval MMP_ERR_NONE Success
*/
MMP_ERR  MMPF_SD_SetTimeout(stSDMMCHandler *SDMMCArg, MMPF_SD_SPEED_MODE mode)
{
    if (mode == MMPF_SD_HIGHSPEED_MODE) {          //48M
        SDMMCArg->ulReadTimeout = SD_HIGHSPEED_READ_TIMEOUT;
        SDMMCArg->ulWriteTimeout = SD_HIGHSPEED_WRITE_TIMEOUT;
    }
    else {                   //24M
        SDMMCArg->ulReadTimeout = SD_NORMAL_READ_TIMEOUT;
        SDMMCArg->ulWriteTimeout = SD_NORMAL_WRITE_TIMEOUT;
    }

    return  MMP_ERR_NONE;
}

/** @brief SD switch timeout mode

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@param[in] Read or write timeout.
@retval MMP_ERR_NONE Success
*/
MMP_ERR  MMPF_SD_SwitchTimeout(stSDMMCHandler *SDMMCArg, MMPF_SD_TIMEOUT mode)
{
    AITPS_SD    pSD = SDMMCArg->pSD;

    switch(mode){
    case MMPF_SD_READ_TIMEOUT:
        //RTNA_DBG_PrintLong(0, SDMMCArg->ulReadTimeout);
        pSD->SD_DATA_TOUT = SDMMCArg->ulReadTimeout;
        break;

    case MMPF_SD_WRITE_TIMEOUT:
        //RTNA_DBG_PrintLong(0, SDMMCArg->ulWriteTimeout);
        pSD->SD_DATA_TOUT = SDMMCArg->ulWriteTimeout;
        break;

    case MMPF_SD_DEFAULT_TIMEOUT:
    default:
        pSD->SD_DATA_TOUT = SD_DEFAULT_TIMEOUT;
        break;
    }

    return  MMP_ERR_NONE;

}


#if (OS_TYPE == OS_UCOSII)

/** @brief Send command sequence for MMC reset

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_RESET Initial command fail, maybe no card.
@retval MMP_SD_ERR_COMMAND_FAILED CMD0 ok, but follow command fail, maybe not MMC card.
*/
MMP_ERR  MMPF_MMC_Reset(stSDMMCHandler *SDMMCArg)
{
    MMP_USHORT  i;
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_ULONG   ulMMCclk, divValue, j;
    MMP_BYTE    b26MPowerClass, b52MPowerClass;

    SDMMCArg->bIsMMC4X = MMP_FALSE;
    SDMMCArg->bAccessFail = MMP_FALSE;

    #if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
    SDMMCArg->ubSDHCHsm = 0;
    #endif

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
    SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
    #endif

    //wilson@110509: move g0 src clk divide setting to init interface function
    //pGBL->GBL_CLK_0_LCD_SD = ((pGBL->GBL_CLK_0_LCD_SD)&0x0F) | (GBL_CLK_SD_DIV(1));

    #if (ENABLE_SD_CLKGATING==1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_NORMAL_MODE);

    #if 1// Before 1.05.02.Philip3: No using slow clock
    pSD->SD_CTL_0 = (RISING_EDGE | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_1);
    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1024);
    #else // After 1.05.02.Philip3: Using slow clock
    pSD->SD_CTL_0 = (RISING_EDGE | SLOW_CLK_EN | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_1);
    pSD->SD_SLOW_CLK_DIV = SD_CLOCK_DIV_2048;
    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_256);
    #endif

    pSD->SD_CTL_1 = WAIT_LAST_BUSY_EN| AUTO_CLK_EN | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;

    // CMD  0
    MMPF_SD_SendCommand(SDMMCArg, GO_IDLE_STATE, 0);
    MMPF_SD_WaitCount(10000);
    i = 0;
    do {
        MMPF_SD_WaitCount(40000);
        // CMD  1
        if((!MMPF_SD_SendCommand(SDMMCArg, SEND_OP_COND, SD_SUPPORT_VOLTAGE))) {

        }
        else {
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif

            return  MMP_SD_ERR_RESET;
        }
        i++;
    } while (!(pSD->SD_RESP.D[3] & 0x80000000) && (i < 1000));

    if (i == 1000) {
        RTNA_DBG_Long1(pSD->SD_RESP.D[3]);
        RTNA_DBG_Str0("Get OCR failed\r\n");

        #if (ENABLE_SD_CLKGATING==1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    RTNA_DBG_Str1("Get OCR success. ");
    RTNA_DBG_Short1(i);
    RTNA_DBG_Str1(" times\r\n");
    // CMD  2
    if (MMPF_SD_SendCommand(SDMMCArg, ALL_SEND_CID, 0)) {

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return  MMP_SD_ERR_RESET;
    }

    //storage serial number
    j = 7;
    for(i = 0; i < 8; i++)
    {
        gStorageSerialNumber[2*i] = (unsigned char)(pSD->SD_RESP.W[j]>>8);
        gStorageSerialNumber[2*i+1] = (unsigned char)(pSD->SD_RESP.W[j--]);
    }

    SDMMCArg->ulCardAddr += 0x10000;

    // CMD  3
    if (MMPF_SD_SendCommand(SDMMCArg, SET_RELATIVE_ADDR, SDMMCArg->ulCardAddr)) {

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    MMPF_SD_WaitCount(5000);

    // CMD  9
    if (MMPF_SD_GetCardInfo(SDMMCArg, MMPF_SD_MMCTYPE)) {

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return  MMP_SD_ERR_RESET;
    }

    // Select SD card
    if (MMPF_SD_SendCommand(SDMMCArg, SELECT_CARD, SDMMCArg->ulCardAddr)) {

        #if (ENABLE_SD_CLKGATING==1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }

    RTNA_DBG_Str0("switch clock\r\n");

    //MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulMMCclk);
    ulMMCclk = m_ulSDG0SrcClk;      //wilson@110509

    RTNA_DBG_Str0("MMC clock src ");
    RTNA_DBG_Long0(ulMMCclk);
    RTNA_DBG_Str0("\r\n");

    if (ulMMCclk % DEFINE_MMC_FULLSPEED == 0) {
        divValue = ulMMCclk/DEFINE_MMC_FULLSPEED;
    }
    else {
        divValue = (ulMMCclk - (ulMMCclk % DEFINE_MMC_FULLSPEED)) / DEFINE_MMC_FULLSPEED + 1;
    }

    if ((divValue == 1)||(divValue == 3)){
        if (divValue == 1)
            MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
        else MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
    }
    else {
        if (divValue%2 != 0)
            divValue++;
        MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
    }
    #if (SD_MMC_CLK_DBG_MSG == 1)
    RTNA_DBG_Str(0, "Current MMC clock = ");
    RTNA_DBG_Short(0, ulMMCclk/(1000*divValue));
    RTNA_DBG_Str(0, "\r\n");
    #endif

    if(SDMMCArg->bIsMMC4X == MMP_TRUE)
    {
        #if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
        MMPF_MMU_FlushDCacheMVA(m_ulSDDmaAddr, 512);
        #endif
        /*Read EXT OSD for more information*/
        if (!MMPF_SD_SendCommand(SDMMCArg, SEND_EXT_CSD, 0)){
            if(SDMMCArg->bIs2GMMC == MMP_TRUE){
                SDMMCArg->ulCardSize = *(MMP_ULONG *)(m_ulSDDmaAddr+212);
                RTNA_DBG_Long3(SDMMCArg->ulCardSize);
                RTNA_DBG_Str3(" MMC large Size\r\n");
            }
            SDMMCArg->ubMMC4XClkMode = *(MMP_UBYTE *)(m_ulSDDmaAddr+196);

            //ref to JESD84-A44 spec.
            b26MPowerClass = *(MMP_UBYTE *)(m_ulSDDmaAddr+203);
            b52MPowerClass = *(MMP_UBYTE *)(m_ulSDDmaAddr+202);
            SDMMCArg->ulMMCBootSize = *(MMP_UBYTE *)(m_ulSDDmaAddr+226);
            SDMMCArg->ulMMCBootSize = SDMMCArg->ulMMCBootSize*256;
        }

        /*Siwtch to 4 bit or 8bit width*/
        #if (DEFINE_MMC_BUS_WIDTH == 8)
            //#if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
            {
                MMP_ULONG ulTmp;
                if(SDMMCArg->ubMMC4XClkMode > MMC_CLK_52M)
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b52MPowerClass&0xF0)<<4);
                else
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b26MPowerClass&0xF0)<<4);

                if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, ulTmp)){
                    if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                        #if (ENABLE_SD_CLKGATING == 1)
                        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                        #endif
                        return  MMP_SD_ERR_RESET;
                    }
                }
            }
            //#endif
        if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B70200)){
            if(!MMPF_SD_CheckCardStatus(SDMMCArg)){
                MMPF_SD_ConfigPadMapping(SDMMCArg, SDMMCArg->ubSdPadMapping, MMP_TRUE);
                //pSD->SD_CTL_0 = (RISING_EDGE | SLOW_CLK_EN | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_8);
                pSD->SD_CTL_0 = (RISING_EDGE | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_8);
                RTNA_DBG_Str1("MMC switch 8 bit \r\n");
            }
        }

        #elif (DEFINE_MMC_BUS_WIDTH == 4)
            //#if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
            {
                MMP_ULONG ulTmp;
                if(SDMMCArg->ubMMC4XClkMode > MMC_CLK_52M)
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b52MPowerClass&0x0F)<<8);
                else
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b26MPowerClass&0x0F)<<8);
                if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, ulTmp)){
                    if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                        #if (ENABLE_SD_CLKGATING == 1)
                        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                        #endif
                        return  MMP_SD_ERR_RESET;
                    }
                }
            }
            //#endif
        if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B70100)){
            if(!MMPF_SD_CheckCardStatus(SDMMCArg)){
                //pSD->SD_CTL_0 = (RISING_EDGE | SLOW_CLK_EN | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
                pSD->SD_CTL_0 = (RISING_EDGE | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
                RTNA_DBG_Str1("MMC switch 4 bit \r\n");
            }
        }

        #else
            {
                MMP_ULONG ulTmp;
                if(SDMMCArg->ubMMC4XClkMode > MMC_CLK_52M)
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b52MPowerClass&0x0F)<<8);
                else
                    ulTmp = 0x03BB0000 | ((MMP_ULONG)(b26MPowerClass&0x0F)<<8);


                if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, ulTmp)){
                    if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                        #if (ENABLE_SD_CLKGATING == 1)
                        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                        #endif
                        return  MMP_SD_ERR_RESET;
                    }
                }
            }
        if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B70000)){
            if(!MMPF_SD_CheckCardStatus(SDMMCArg)){
                //pSD->SD_CTL_0 = (RISING_EDGE | SLOW_CLK_EN | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
                pSD->SD_CTL_0 = (RISING_EDGE | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_1);
                RTNA_DBG_Str1("MMC switch 1 bit \r\n");
            }
        }
        #endif
        //Scan all eMMC for testing
        /*
        gbIsSD[id] = 0;
        pSD->SD_BLK_LEN = 0x200;
        for(ul=0;ul<0x774000;ul++){
            MMPF_SD_WaitCount(5000);
            MMPF_SD_ReadSector(id, m_ulSDDmaAddr, ul, 1);
        }
        for(ul=0;ul<0x774000;ul++){
            MMPF_SD_WaitCount(5000);
            MMPF_SD_WriteSector(id, m_ulSDDmaAddr, ul, 1);
        }
        */
        #if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
        /*Siwtch to high speed mode*/
        {
            // CMD 6: SWITCH_FUNC
            if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B90100)) {
                if (!MMPF_SD_CheckCardStatus(SDMMCArg)) {
                    RTNA_DBG_Str1("MMC high speed mode \r\n");
                    if (SDMMCArg->ubMMC4XClkMode > MMC_CLK_52M){  //52M
                    //if(0){
                        if (ulMMCclk % DEFINE_MMC_HIGHSPEED==0) {
                            divValue = ulMMCclk / DEFINE_MMC_HIGHSPEED;
                        }
                        else {
                            divValue = (ulMMCclk - (ulMMCclk%DEFINE_MMC_HIGHSPEED))/DEFINE_MMC_HIGHSPEED + 1;
                        }

                        if ((divValue == 1)||(divValue == 3)){
                            if (divValue == 1)
                                MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
                            else MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
                        }
                        else {
                            if (divValue%2 != 0)
                                divValue++;
                            MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
                        }
                        #if (SD_MMC_CLK_DBG_MSG == 1)
                        RTNA_DBG_Str(0, "Current MMC clock = ");
                        RTNA_DBG_Short(0, ulMMCclk/(1000*divValue));
                        RTNA_DBG_Str(0, "\r\n");
                        #endif

                        MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_HIGHSPEED_MODE);
                        RTNA_DBG_Long1(ulMMCclk);
                        RTNA_DBG_Str(3,"High Speed 52 Switch done.\r\n");
                    }
                    else{
                        if (ulMMCclk%DEFINE_MMC_FULLSPEED==0) {
                            divValue = ulMMCclk/DEFINE_MMC_FULLSPEED;
                        }
                        else {
                            divValue = (ulMMCclk - (ulMMCclk%DEFINE_MMC_FULLSPEED))/DEFINE_MMC_FULLSPEED + 1;
                        }

                        if ((divValue == 1)||(divValue == 3)){
                            if (divValue == 1)
                                MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
                            else MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
                        }
                        else {
                            if (divValue%2 != 0)
                                divValue++;
                            MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
                        }
                        #if (SD_MMC_CLK_DBG_MSG == 1)
                        RTNA_DBG_Str(0, "Current SD clock = ");
                        RTNA_DBG_Short(0, ulMMCclk/(1000*divValue));
                        RTNA_DBG_Str(0, "\r\n");
                        #endif

                        RTNA_DBG_Long1(ulMMCclk);
                        RTNA_DBG_Str(3,"High Speed 26 Switch done.\r\n");
                    }

                }
                else {
                    RTNA_DBG_Str(0,"Not Switch MMC High Speed Mode.\r\n");
                    SDMMCArg->ubSDHCHsm = MMP_FALSE;
                }

            }
        }
        #endif
    }

    SDMMCArg->bIsSD = 0;
    pSD->SD_BLK_LEN = 0x200;

    if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B30000)){
        if(MMPF_SD_CheckCardStatus(SDMMCArg)){
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif
            return  MMP_SD_ERR_RESET;
        }
    }

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
    #endif

    return  MMP_ERR_NONE;
}

MMP_ERR MMPF_MMC_SwitchBootPartition(stSDMMCHandler *SDMMCArg, MMPF_MMC_BOOTPARTITION part)
{
    if((SDMMCArg->ulMMCCurrentPartition == part)|| (!SDMMCArg->bIsMMC4X))
        return  MMP_ERR_NONE;

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    switch(part){
        case MMPF_MMC_BOOT1_AREA:
            if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B30100)){
                if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                    #if (ENABLE_SD_CLKGATING==1)
                    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                    #endif
                    return  MMP_SD_ERR_RESET;
                }
            }
            break;

        case MMPF_MMC_BOOT2_AREA:
            if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B30200)){
                if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                    #if (ENABLE_SD_CLKGATING==1)
                    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                    #endif
                    return  MMP_SD_ERR_RESET;
                }
            }
            break;

        case MMPF_MMC_USER_AREA:
        default:
            if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x03B30000)){
                if(MMPF_SD_CheckCardStatus(SDMMCArg)){
                    #if (ENABLE_SD_CLKGATING==1)
                    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                    #endif
                    return  MMP_SD_ERR_RESET;
                }
            }
            break;

    }

    SDMMCArg->ulMMCCurrentPartition = part;
    #if (ENABLE_SD_CLKGATING==1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
    #endif
    return  MMP_ERR_NONE;

}

#if (EN_MOVINAND_CONFIG_BOOTSIZE)
MMP_ERR  MMPF_MMC_ConfigBootPartition(stSDMMCHandler *SDMMCArg, MMP_ULONG ulBootSectorNum)
{
    MMP_ULONG ulSuperBlockSize, ulSuperblockNum;

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    /*Read Super block size*/
    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0xEFAC62EC))
        return MMP_SD_ERR_COMMAND_FAILED;
    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0x0000CCEE))
        return MMP_SD_ERR_COMMAND_FAILED;
    if(MMPF_SD_ReadSector(SDMMCArg, (MMP_ULONG)m_ulSDDmaAddr, 0, 1))
        return MMP_SD_ERR_COMMAND_FAILED;

    /*RTNA_DBG_Long3(*(MMP_ULONG *)&m_ulSDDmaAddr[0]);
    RTNA_DBG_Long3(*(MMP_ULONG *)&m_ulSDDmaAddr[4]);
    RTNA_DBG_Long3(*(MMP_ULONG *)&m_ulSDDmaAddr[12]);
    RTNA_DBG_Long3(*(MMP_ULONG *)&m_ulSDDmaAddr[20]);*/
    ulSuperBlockSize = *(MMP_ULONG *)(m_ulSDDmaAddr+4);

    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0xEFAC62EC))
        return MMP_SD_ERR_COMMAND_FAILED;
    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0x00DECCEE))
        return MMP_SD_ERR_COMMAND_FAILED;



    /*Configure boot partition size*/
    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0xEFAC62EC))
        return MMP_SD_ERR_COMMAND_FAILED;
    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, 0x00CBAEA7))
        return MMP_SD_ERR_COMMAND_FAILED;

    ulSuperblockNum = ((ulBootSectorNum << 9) + ulSuperBlockSize -1 )/ ulSuperBlockSize;

    if(MMPF_SD_SendCommand(SDMMCArg, VENDER_COMMAND_62, (ulSuperblockNum + 1) >> 1))
        return MMP_SD_ERR_COMMAND_FAILED;

    RTNA_DBG_Str3("Start polling card status\r\n");
    while (MMPF_SD_CheckCardStatus(SDMMCArg));
    RTNA_DBG_Str3("End polling card status\r\n");


    return  MMP_ERR_NONE;
}
#endif


/** @brief Send command sequence for SD reset

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_RESET Initial command fail, maybe no card.
@retval MMP_SD_ERR_COMMAND_FAILED CMD0 ok, but follow command fail, maybe not SD card.
*/
MMP_ERR  MMPF_SD_Reset(stSDMMCHandler *SDMMCArg)
{
    MMP_USHORT  i;
    MMP_ULONG   ulSDclk, divValue;
    int         sd2 = 0;
    AITPS_SD    pSD = SDMMCArg->pSD;

    SDMMCArg->bAccessFail = MMP_FALSE;
    SDMMCArg->bIsMMC4X = MMP_FALSE;
    SDMMCArg->ulMMCCurrentPartition = 0;

    #if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
    SDMMCArg->ubSDHCHsm = MMP_FALSE;
    #endif

    #if (ENABLE_SD_SEQUENTIAL_OP == 1)
    SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
    #endif

    #if (ENABLE_DETECT_SDCLASS == 1)
    SDMMCArg->ubSDClass = 0xFF;  // Reserved Value
    #endif

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif
    MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_NORMAL_MODE);
    MMPF_SD_SwitchTimeout(SDMMCArg, MMPF_SD_DEFAULT_TIMEOUT);
    //wilson@110509: move g0 src clk divide setting to init interface function
    //pGBL->GBL_CLK_0_LCD_SD = ((pGBL->GBL_CLK_0_LCD_SD)&0x0F) | (GBL_CLK_SD_DIV(1));

    #if 1 // Before 1.05.02.Philip2: No using slow clock
    pSD->SD_CTL_0 = (SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1024);
    #else // After 1.05.02.Philip2: Using slow clock
    pSD->SD_CTL_0 = (SLOW_CLK_EN | SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
    pSD->SD_SLOW_CLK_DIV = SD_CLOCK_DIV_2048;
    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_256);
    #endif

    pSD->SD_CTL_1 = WAIT_LAST_BUSY_EN| AUTO_CLK_EN | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;

    // CMD  0
    MMPF_SD_SendCommand(SDMMCArg, GO_IDLE_STATE, 0);
    i = 0;

    // CMD  8
    do {
        if(MMPF_SD_SendCommand(SDMMCArg, SEND_IF_COND, (0x1 << 8)| 0xAA)) {
            RTNA_DBG_Str3("non-sd2.0\r\n");
            sd2 = 0;
        }
        else{
            RTNA_DBG_Str3("sd2.0\r\n");
            sd2 = 1;
        }
        i++;
    } while(i < 5 && (sd2 == 0));


    MMPF_SD_WaitCount(10000);
    i = 0;
    do {
        MMPF_SD_WaitCount(40000);
        // AMCD41
        if(!MMPF_SD_SendCommand(SDMMCArg, APP_CMD, 0)) {
            if(!MMPF_SD_SendCommand(SDMMCArg, SD_APP_OP_COND, SD_SUPPORT_VOLTAGE| (1<<30))) {
                i++;
            }
            else {
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif
                return  MMP_SD_ERR_RESET;
            }
        }
        else {
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif
            return  MMP_SD_ERR_RESET;
        }
    } while( !(pSD->SD_RESP.D[3] & 0x80000000) && (i < 1000));
    if(pSD->SD_RESP.D[3] & 0x40000000) {
        SDMMCArg->bIsHC = MMP_TRUE;
        RTNA_DBG_Str0("SDHC\r\n");
    }
    else {
        SDMMCArg->bIsHC = MMP_FALSE;
    }
    if (i == 1000) {
        RTNA_DBG_Str0("Get OCR failed\r\n");

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    RTNA_DBG_Str0("Get OCR success. ");
    RTNA_DBG_Short0(i);
    RTNA_DBG_Str0(" times\r\n");
    // CMD  2
    if (MMPF_SD_SendCommand(SDMMCArg, ALL_SEND_CID, 0)) {

        #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return  MMP_SD_ERR_RESET;
    }
    // CMD  3
    if(!MMPF_SD_SendCommand(SDMMCArg, SEND_RELATIVE_ADDR, 0)) {
        SDMMCArg->ulCardAddr = (MMP_ULONG)pSD->SD_RESP.D[3];
        #if 0
        RTNA_DBG_Long0(SDMMCArg->ulCardAddr);
        RTNA_DBG_Str0(": SD Card Addr.\r\n");
        #endif
    }
    else {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    // CMD  9
    if (MMPF_SD_GetCardInfo(SDMMCArg, MMPF_SD_SDTYPE)) {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return  MMP_SD_ERR_RESET;
    }

    // Select SD card
    if (MMPF_SD_SendCommand(SDMMCArg, SELECT_CARD, SDMMCArg->ulCardAddr)) {

        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }

    // Transfer State (tran)
    // ACMD42
    if (MMPF_SD_SendCommand(SDMMCArg, APP_CMD, SDMMCArg->ulCardAddr)) {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    if (MMPF_SD_SendCommand(SDMMCArg, SET_CLR_CARD_DETECT, (SDMMCArg->ulCardAddr|0))) {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }

    // ACMD6
    if (MMPF_SD_SendCommand(SDMMCArg, APP_CMD, SDMMCArg->ulCardAddr)) {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    if (MMPF_SD_SendCommand(SDMMCArg, SET_BUS_WIDTH, 0x2)) {
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_RESET;
    }
    MMPF_SD_WaitCount(10000);

    //#if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
    //if (!mIsHC[id]) {
    //#endif
        {
            //MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulSDclk);
            ulSDclk = m_ulSDG0SrcClk;   //wilson@110509

            #if (SD_MMC_CLK_DBG_MSG == 1)
            RTNA_DBG_Str0("SD clock src ");
            RTNA_DBG_Long0(ulSDclk);
            RTNA_DBG_Str0("\r\n");
            #endif

            if (ulSDclk % DEFINE_SD_FULLSPEED==0) {
                divValue = ulSDclk / DEFINE_SD_FULLSPEED;
            }
            else {
                divValue = (ulSDclk - (ulSDclk % DEFINE_SD_FULLSPEED))/DEFINE_SD_FULLSPEED + 1;
            }

            if ((divValue == 1)||(divValue == 3)){
                if (divValue == 1)
                    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
                else
                    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
            }
            else {
                if (divValue%2 != 0)
                    divValue++;
                MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
            }
            #if (SD_MMC_CLK_DBG_MSG == 1)
            RTNA_DBG_Str(0, "Current SD clock = ");
            RTNA_DBG_Short(0, ulSDclk/(1000*divValue));
            RTNA_DBG_Str(0, "\r\n");
            #endif
        }
#if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
    //}
    //else {
        //ACMD51
        if (MMPF_SD_SendCommand(SDMMCArg, APP_CMD, SDMMCArg->ulCardAddr)) {
            #if (ENABLE_SD_CLKGATING == 1)
            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
            #endif
            return MMP_SD_ERR_RESET;
        }
        #if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
        MMPF_MMU_FlushDCacheMVA(m_ulSDDmaAddr, 512);
        #endif
        if (!MMPF_SD_SendCommand(SDMMCArg, SEND_SCR, 0)) {
            MMP_UBYTE *pTmp;
            pTmp = (MMP_UBYTE *) m_ulSDDmaAddr;

            // Check "SD_SPEC" is 2.0
            if ((*pTmp)>0) {
                SDMMCArg->ubSDHCHsm = 1;
                #if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
                MMPF_MMU_FlushDCacheMVA(m_ulSDDmaAddr, 512);
                #endif
                // CMD 6: SWITCH_FUNC
                if (!MMPF_SD_SendCommand(SDMMCArg, SWITCH_FUNC, 0x80FFFF01)) {
                    pTmp = (MMP_UBYTE *) m_ulSDDmaAddr;
                    if (pTmp[13] & 0x03) {
                        MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_HIGHSPEED_MODE);
                        RTNA_DBG_Str(0,"High Speed Mode Switch done.\r\n");

                        if (ulSDclk%DEFINE_SD_HIGHSPEED==0) {
                            divValue = ulSDclk/DEFINE_SD_HIGHSPEED;
                        }
                        else {
                            divValue = (ulSDclk - (ulSDclk%DEFINE_SD_HIGHSPEED))/DEFINE_SD_HIGHSPEED + 1;
                        }

                        if ((divValue == 1)||(divValue == 3)) {
                            if (divValue == 1)
                                MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
                            else MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
                        }
                        else {
                            if (divValue%2 != 0)
                                divValue++;
                            MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
                        }

                        #if (SD_MMC_CLK_DBG_MSG == 1)
                        RTNA_DBG_Str(0, "Current SD clock = ");
                        RTNA_DBG_Short(0, ulSDclk/(1000*divValue));
                        RTNA_DBG_Str(0, "\r\n");
                        #endif

                        pSD->SD_CTL_0 |= RISING_EDGE;
                    }
                    else {
                        RTNA_DBG_Str(3,"Not Switch SD High Speed Mode.\r\n");
                        SDMMCArg->ubSDHCHsm = MMP_FALSE;
                    }

                    #if (ENABLE_DETECT_SDCLASS)
                    if(SDMMCArg->ubSDHCHsm) {
                        #if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
                        MMPF_MMU_FlushDCacheMVA(m_ulSDDmaAddr, 512);
                        #endif
                        if (MMPF_SD_SendCommand(SDMMCArg, APP_CMD, SDMMCArg->ulCardAddr)) {
                            #if (ENABLE_SD_CLKGATING == 1)
                            MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                            #endif
                            return MMP_SD_ERR_RESET;
                        }

                        if (!MMPF_SD_SendCommand(SDMMCArg, SD_STATUS, 0)) {
                            SDMMCArg->ubSDClass = (*(MMP_UBYTE *)(m_ulSDDmaAddr + 8) & 0xFF) * 2;
                        } else {
                            SDMMCArg->ubSDClass = 0;
                        }

                    }
                    #endif

                }
            } /* check "SD_SPEC" is 2.0 */
        } /* if SEND_SCR ok */
    //} /* if (mIsHC[id]) */

#endif /* (ENABLE_SDHC_SWITCH_HIGH_SPEED==1) */

    SDMMCArg->usBlockLen = 0x200;
    pSD->SD_BLK_LEN = SDMMCArg->usBlockLen;

    SDMMCArg->bIsSD = MMP_TRUE;
    pSD->SD_CTL_0 &= ~(SLOW_CLK_EN);

    return  MMP_ERR_NONE;
}

#if (ENABLE_DETECT_SDCLASS == 1)
MMP_ERR MMPF_SD_GetCardClass(stSDMMCHandler *SDMMCArg, MMP_UBYTE *ubClass)
{
    *ubClass = SDMMCArg->ubSDClass;
    return MMP_ERR_NONE;
}
#endif

/** @brief Send SEND_CSD (CMD 9) to get card info after card reset

@param[in] devid ID of SD controller. For DIAMOND, valid input is 0, 1 and 2.
@retval MMP_ERR_NONE Success
@retval MMP_SD_ERR_RESET Command fail, maybe no card.
*/
MMP_ERR  MMPF_SD_GetCardInfo(stSDMMCHandler *SDMMCArg, MMPF_SD_CARDTYPE btype)
{
    MMP_USHORT  c_size;
    MMP_UBYTE   csizemult;
    MMP_UBYTE   block_size;
    AITPS_SD    pSD = SDMMCArg->pSD;

    if (!MMPF_SD_SendCommand(SDMMCArg, SEND_CSD, SDMMCArg->ulCardAddr)) {
        // The maximum size of SD2.0 is 32 GB, so the (pSD->SD_RESP.W[4] & 0x3F) should be zero.
        if ((SDMMCArg->bIsHC)&&(btype == MMPF_SD_SDTYPE)) {
            c_size = (pSD->SD_RESP.W[3] | ((pSD->SD_RESP.W[4] & 0x3F)<<16) );
            SDMMCArg->ulCardSize = (c_size +1)*1024;        //the unit is sector
        }
        else {
            block_size = pSD->SD_RESP.W[5] & 0x000f;
            c_size = (pSD->SD_RESP.W[4] & 0x03ff) << 2;
            c_size += (pSD->SD_RESP.W[3] & 0xC000) >> 14;
            csizemult = (pSD->SD_RESP.W[3] & 0x0003) << 1;
            csizemult += (pSD->SD_RESP.B[5] & 0x80) >> 7;
            SDMMCArg->ulCardSize = (((MMP_ULONG)(c_size + 1)) * (1 << (csizemult + 2))) << (block_size - 9);
        }

        if((btype == MMPF_SD_MMCTYPE)&&((pSD->SD_RESP.B[15]&0x3C) == 0x10)){
            SDMMCArg->bIsMMC4X = MMP_TRUE;
            if(c_size == 0xFFF){
                SDMMCArg->bIs2GMMC = MMP_TRUE;
            }
            else
                SDMMCArg->bIs2GMMC = MMP_FALSE;
        }
        else{
            SDMMCArg->bIsMMC4X = MMP_FALSE;
        }


        #if 0
        RTNA_DBG_Long2( (SDMMCArg->ulCardSize)*512 );
        RTNA_DBG_Str2(": Card Size.\r\n");
        #endif

        return  MMP_ERR_NONE;
    }
    else {
        return  MMP_SD_ERR_RESET;
    }
}
//------------------------------------------------------------------------------
//  Function    : MMPF_GetSDSize
//------------------------------------------------------------------------------
MMP_ERR  MMPF_SD_GetSize(stSDMMCHandler *SDMMCArg, MMPF_MMC_BOOTPARTITION part, MMP_ULONG *pSize)
{
    switch(part){
        case MMPF_MMC_BOOT1_AREA:
        case MMPF_MMC_BOOT2_AREA:
            *pSize = SDMMCArg->ulMMCBootSize;
            break;
        case MMPF_MMC_USER_AREA:
            *pSize = SDMMCArg->ulCardSize;
            break;
    }
    return  MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SD_SendCommand
//------------------------------------------------------------------------------
MMP_ERR  MMPF_SD_SendCommand(stSDMMCHandler *SDMMCArg, MMP_UBYTE command, MMP_ULONG argument)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  tmp;

    switch(command) {
    case GO_IDLE_STATE: /* no resp */
        pSD->SD_CMD_REG_0 = command | NO_RESP;
        break;
    case ALL_SEND_CID: /* FALL THROUGH */
    case SEND_CSD: /* r2 */
        pSD->SD_CMD_REG_0 = command | R2_RESP;
        break;
    case STOP_TRANSMISSION: /* FALL THROUGH */
    case SELECT_CARD: /* r1b*/
    case VENDER_COMMAND_62:
    case ERASE:
        pSD->SD_CMD_REG_0 = command | R1B_RESP;
        break;
    case SWITCH_FUNC:
        if( SDMMCArg->bIsMMC4X == MMP_TRUE){
            pSD->SD_CMD_REG_0 = command | R1B_RESP;
            break;
        }
    case SD_STATUS:
        pSD->SD_CMD_REG_0 = (command & 0x3F) | OTHER_RESP;
        break;
    default: /* others */
        pSD->SD_CMD_REG_0 = command | OTHER_RESP;
        break;
    }

    pSD->SD_CMD_ARG = argument; /* SD command's argument */

    /* Clear interrupt status */
    pSD->SD_CPU_INT_SR = (CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|CMD_SEND_DONE|SDIO_INT|DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE);

    #if (EN_CARD_DETECT == 1)
    if (!MMPF_SD_CheckCardIn(SDMMCArg)) {
        MMPF_HIF_SetCmdStatus(SD_CARD_NOT_EXIST);

        return  MMP_SD_ERR_CARD_REMOVED;
    }
    #endif

    #if (SD_CPU_NONBLOCKING == 1)
    if (command == STOP_TRANSMISSION) {
        pSD->SD_CPU_INT_EN = (CMD_SEND_DONE);
    }
    #endif

    /* SD_TRANSFER_START */
    //#if (ENABLE_SDHC_SWITCH_HIGH_SPEED == 1)
    if (command == SEND_SCR) {
        // Refer to Section 5.6, SCR register size is 64 bit (8 Byte)
        pSD->SD_BLK_LEN = 8;
        //RTNA_DBG_Str3("Issue SEND_SCR\r\n");
        pSD->SD_BLK_NUM = 1;
        pSD->SD_DMA_ST_ADDR = m_ulSDDmaAddr;
        pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);

        #if (SD_CPU_NONBLOCKING == 1)
        pSD->SD_CPU_INT_EN |= (DATA_SEND_DONE);
        #endif
    }
    else if ((command == SWITCH_FUNC) && (SDMMCArg->ubSDHCHsm == MMP_TRUE)) {
        // Refer to Section 4.3.10, SWITCH response size is 512 bit (64 Byte)
        pSD->SD_BLK_LEN = 64;
        pSD->SD_BLK_NUM = 1;
        pSD->SD_DMA_ST_ADDR = m_ulSDDmaAddr;
        pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);

        #if (SD_CPU_NONBLOCKING == 1)
        pSD->SD_CPU_INT_EN |= (DATA_SEND_DONE);
        #endif
    }
    else if ((command == SD_STATUS) && (SDMMCArg->ubSDHCHsm == MMP_TRUE)) {
        pSD->SD_BLK_LEN = 64;
        pSD->SD_BLK_NUM = 1;
        pSD->SD_DMA_ST_ADDR = m_ulSDDmaAddr;
        pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);

        #if (SD_CPU_NONBLOCKING == 1)
        pSD->SD_CPU_INT_EN |= (DATA_SEND_DONE);
        #endif
    }
    else if ((command == SEND_EXT_CSD) && (SDMMCArg->bIsMMC4X == MMP_TRUE)) {
        RTNA_DBG_Str3("SEND_EXT_CSD cmd \r\n");
        // 512 byte extend CSD
        pSD->SD_BLK_LEN = 512;
        //RTNA_DBG_Str3("Issue SWITCH_FUN\r\n");
        pSD->SD_BLK_NUM = 1;
        pSD->SD_DMA_ST_ADDR = m_ulSDDmaAddr;
        pSD->SD_CMD_REG_1 |= (ADTC_READ|SEND_CMD);

        #if (SD_CPU_NONBLOCKING == 1)
        pSD->SD_CPU_INT_EN |= (DATA_SEND_DONE);
        #endif
    }
    else
    //#endif /* (ENABLE_SDHC_SWITCH_HIGH_SPEED==1) */
    {
        pSD->SD_CMD_REG_1 |= SEND_CMD; // Send CMD here
    }

    /* **************** */
    /* Check CMD status */
    /* **************** */
    if (command == GO_IDLE_STATE) {
        while (!(pSD->SD_CPU_INT_SR & CMD_SEND_DONE));
    }
    #if (SD_CPU_NONBLOCKING == 1)
    else if (command == STOP_TRANSMISSION) {
        MMPF_SD_WaitIsrDone(SDMMCArg, 0);
    }
    #endif /* (SD_CPU_NONBLOCKING == 1) */
    else {
        while(1) {
            tmp = pSD->SD_CPU_INT_SR;
            //RTNA_DBG_Short3(tmp);RTNA_DBG_Str(3, ":pSD->SD_CPU_INT_SR.\r\n");//PhilipTrace
            if ((tmp & CMD_SEND_DONE) && (!(tmp & CMD_RESP_TOUT))) {
                break;
            }

            if(tmp & CMD_RESP_TOUT) {

                #if 0
                RTNA_DBG_Byte0(SDMMCArg->id);
                RTNA_DBG_Str0(" : ");
                RTNA_DBG_Long0(command);
                RTNA_DBG_Str0(" : id, AITPS_SD for CMD_RESP_TOUT.\r\n");
                #endif

                #if (ENABLE_SDIO_FEATURE == 1)
                if (command == IO_SEND_OP_COND) {
                    break;
                }
                #endif

                // Designer recommand that wait for a while
                MMPF_SD_WaitCount(500);

                if (command != SEND_IF_COND) {
                    #if 0
                    RTNA_DBG_Str0("SD cmd");
                    MMPF_DBG_Int(command, -2);
                    RTNA_DBG_Str0(" fails. Status = ");
                    RTNA_DBG_Byte0(pSD->SD_CPU_INT_SR);
                    RTNA_DBG_Str0(".\r\n");
                    #endif
                }

                return MMP_SD_ERR_COMMAND_FAILED;
            }
        }
    }


    /* Check CMD response status */
    if ((command == STOP_TRANSMISSION)||(command == SELECT_CARD)||(command == VENDER_COMMAND_62)) { /* ac, R1b */
        //while(!(pSD->SD_DATA_BUS_SR & DAT0_SR));
        if( pSD->SD_CPU_INT_SR & CMD_RESP_CRC_ERR){
            RTNA_DBG_Str0("crc7 error\r\n");
        }
    }
    #if (ENABLE_SDIO_FEATURE == 1)
    else if ((command == IO_RW_DIRECT) || (command == IO_RW_EXTENDED)) { /* adtc, R4 */
        if (pSD->SD_RESP.D[3] & (COM_CRC_ERROR | ILLEGAL_COMMAND
                                | UNKNOW_ERROR | INV_FUN_NUM
                                | OUT_OF_RANGE) ) {
            RTNA_DBG_Str2("SDIO Command ");
            MMPF_DBG_Int(command, -2);
            RTNA_DBG_Str2(" fails. Status = ");
            RTNA_DBG_Short2(pSD->SD_CPU_INT_SR);
            RTNA_DBG_Str2("\r\n");
        }
    }
    #endif
    else if ((command != ALL_SEND_CID) && (command != SET_RELATIVE_ADDR) &&
             (command != SEND_OP_COND) &&
            #if (ENABLE_SDIO_FEATURE == 1)
             (command != IO_SEND_OP_COND) &&
            #endif
             (command != SD_APP_OP_COND)
            ) {

        tmp = pSD->SD_CPU_INT_SR;

        if (tmp & CMD_RESP_CRC_ERR) {
            RTNA_DBG_Str0("SD cmd");
            MMPF_DBG_Int(command, -2);
            RTNA_DBG_Str0(":crc7 error.\r\n");
            return MMP_SD_ERR_DATA;
        }
    }

    //#if (ENABLE_SDHC_SWITCH_HIGH_SPEED==1)
    if ( (command==SEND_SCR)
        || ((command==SWITCH_FUNC) && (SDMMCArg->ubSDHCHsm == MMP_TRUE))
        || ((command == SD_STATUS) && (SDMMCArg->ubSDHCHsm == MMP_TRUE))
        || ((command == SEND_EXT_CSD) && (SDMMCArg->bIsMMC4X == MMP_TRUE))) {

        #if (SD_CPU_NONBLOCKING == 1)
        MMPF_SD_WaitIsrDone(SDMMCArg, 0);
        #else
        while(!(pSD->SD_CPU_INT_SR & DATA_SEND_DONE));
        #endif

    }
    //#endif

    return  MMP_ERR_NONE;
} /* MMPF_SD_SendCommand */

//------------------------------------------------------------------------------
//  Function    : MMPF_SD_CheckCardStatus
//------------------------------------------------------------------------------
MMP_ERR  MMPF_SD_CheckCardStatus(stSDMMCHandler *SDMMCArg)
{
    MMP_ERR err;

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMPF_SD_BusAcquire(SDMMCArg, 0)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    err = MMPF_SD_SendCommand(SDMMCArg, SEND_STATUS, SDMMCArg->ulCardAddr);
    #if (ENABLE_SD_CLKGATING==1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
    #endif

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMPF_SD_BusRelease(SDMMCArg)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif

    if (err)
        return MMP_SD_ERR_COMMAND_FAILED;
    else
        return MMP_ERR_NONE;
}

#if (SD_BUS_REENTRY_PROTECT == 1)
//------------------------------------------------------------------------------
//  Function    : MMPF_SD_BusAcquire
//------------------------------------------------------------------------------
MMP_ERR MMPF_SD_BusAcquire(stSDMMCHandler *SDMMCArg, MMP_ULONG ulTimeout)
{
    MMP_UBYTE ret;

    ret = MMPF_OS_AcquireSem(SDMMCArg->BusySemID, ulTimeout);

    if (ret == OS_NO_ERR) {
        return MMP_ERR_NONE;
    }
    else {
        DBG_S1("SD_BusAcquire OSSemPend fail : ");
        DBG_L1(ret);
        DBG_S1("\r\n");
        return MMP_SD_ERR_BUSY;
    }
}
//------------------------------------------------------------------------------
//  Function    : MMPF_SD_BusRelease
//------------------------------------------------------------------------------
MMP_ERR MMPF_SD_BusRelease(stSDMMCHandler *SDMMCArg)
{
    MMP_UBYTE ret;

    ret = MMPF_OS_ReleaseSem(SDMMCArg->BusySemID);

    if ( ret== OS_NO_ERR) {
        return MMP_ERR_NONE;
    }
    else {
        DBG_L1(ret);
        DBG_S1(":SD_BusRelease OSSemPost fail\r\n");
        return MMP_SD_ERR_BUSY;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SD_EraseSector
//  flow: start(CMD32) -> end(CMD33) -> erase(CMD38)
//------------------------------------------------------------------------------
MMP_ERR MMPF_SD_EraseSector(stSDMMCHandler *SDMMCArg, MMP_ULONG startsect, MMP_ULONG sectcount)
{
    if (MMPF_SD_SendCommand(SDMMCArg, ERASE_WR_BLK_START, (MMP_ULONG)startsect)){
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }
    if (MMPF_SD_SendCommand(SDMMCArg, ERASE_WR_BLK_END, (MMP_ULONG)(startsect+sectcount))){
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }
    if (MMPF_SD_SendCommand(SDMMCArg, ERASE, startsect)){
        #if (ENABLE_SD_CLKGATING == 1)
        MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
        #endif

        #if (SD_BUS_REENTRY_PROTECT == 1)
        if (MMPF_SD_BusRelease(SDMMCArg)) {
            return MMP_SD_ERR_BUSY;
        }
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }
    return MMP_ERR_NONE;
}

#if (ENABLE_SD_SEQUENTIAL_OP == 1)
//------------------------------------------------------------------------------
//  Function    : MMPF_SD_LeaveBurstMode
//  usage: when enable sequential write operation, force to send STOP_TRANSMIT command
//         to flush data when non-continuous sector write or write operation is done.
//------------------------------------------------------------------------------
MMP_ERR MMPF_SD_LeaveBurstMode(stSDMMCHandler *SDMMCArg)
{
    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMP_ERR_NONE != MMPF_SD_BusAcquire(SDMMCArg, 0)) {
        return MMP_SD_ERR_BUSY;
    }
    else
    #endif
    {
        if (SDMMCArg->bSDNeedStopTransmit == MMP_TRUE)
        {
            if (MMPF_SD_SendCommand(SDMMCArg, STOP_TRANSMISSION, 0))
            {
                #if (ENABLE_SD_CLKGATING == 1)
                MMPF_SD_EnableModuleClock(SDMMCArg, MMP_FALSE);
                #endif

                #if (SD_BUS_REENTRY_PROTECT == 1)
                    if (MMPF_SD_BusRelease(SDMMCArg))
                    {
                        return MMP_SD_ERR_BUSY;
                    }
                #endif

                return MMP_SD_ERR_COMMAND_FAILED;
            }
        }
        SDMMCArg->bSDNeedStopTransmit = MMP_FALSE;
    }

    #if (SD_BUS_REENTRY_PROTECT == 1)
    if (MMPF_SD_BusRelease(SDMMCArg)) {
        return MMP_SD_ERR_BUSY;
    }
    #endif
    return MMP_ERR_NONE;
}
#endif

#endif /* (SD_BUS_REENTRY_PROTECT==1) */


/** @addtogroup MMPF_SDIO
@{
*/
#if (ENABLE_SDIO_FEATURE==1)
//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_Reset
//------------------------------------------------------------------------------
MMP_ERR  MMPF_SDIO_Reset(stSDMMCHandler *SDMMCArg)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_ULONG   ulSDclk = 0, divValue = 0;
    MMP_ULONG   SDIOSpeed = 0;
    MMP_UBYTE   ret, LowSpeed = 0, SetHSCnt = 10;

    SDMMCArg->bAccessFail = MMP_FALSE;

    #if (ENABLE_SD_CLKGATING == 1)
    MMPF_SD_EnableModuleClock(SDMMCArg, MMP_TRUE);
    #endif

    MMPF_SD_SetTimeout(SDMMCArg, MMPF_SD_NORMAL_MODE);
    MMPF_SD_SwitchTimeout(SDMMCArg, MMPF_SD_DEFAULT_TIMEOUT);

    pSD->SD_CTL_0 = (SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_1);
    MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_256);

    pSD->SDIO_ATA_CTL = SDIO_EN;

    SDMMCArg->SDIOBusySemID = MMPF_OS_CreateSem(1);

    SDMMCArg->MMPF_SDIO_CallBackISR = NULL;

    #if (SDIO_SW_CLK_CTL == 1)
    SDMMCArg->ulSDIOClkCnt = 0;
    #endif

    #if (SDIO_AUTO_CLK_EN)
    pSD->SD_CTL_1 = WAIT_LAST_BUSY_EN | AUTO_CLK_EN | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;
    #else
    pSD->SD_CTL_1 = WAIT_LAST_BUSY_EN | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;
    #if (SDIO_SW_CLK_CTL)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_TRUE);
    #else
    pSD->SD_CTL_1 |= CLK_EN;
    while(pSD->SD_CTL_1 & CLK_EN);
    #endif
    #endif
#if 1

    MMPF_SD_SendCommand(SDMMCArg, GO_IDLE_STATE, 0);

    MMPF_SD_SendCommand(SDMMCArg, IO_SEND_OP_COND, 0);

    MMPF_SD_SendCommand(SDMMCArg, IO_SEND_OP_COND, 0x300000);

    MMPF_SD_SendCommand(SDMMCArg, SET_RELATIVE_ADDR, 0);

    SDMMCArg->ulCardAddr = (MMP_ULONG)pSD->SD_RESP.D[3];

    MMPF_SD_SendCommand(SDMMCArg, SELECT_CARD, SDMMCArg->ulCardAddr);

    // END of Initial Dialog sequence with SDIO card
    // Start of initializing normal operation parameters

    MMPF_SDIO_ReadReg( SDMMCArg, 0, 0x08, &ret);

    if(ret & 0x40) {
        LowSpeed = 1;
    }

    if((ret & 0xC0) == 0x40) {
        // only support 1 bit mode
    } else {
        // changing DATA bus width on both Host & Device sides
        MMPF_SDIO_ReadReg( SDMMCArg, 0, 0x07, &ret);
        MMPF_SDIO_WriteReg(SDMMCArg, 0, 0x07, (ret & 0xfc) | 0x02);  // select 4-bit bus
        pSD->SD_CTL_0 = (SD_DMA_EN | CLK_IN_SYS_DIV | BUS_WIDTH_4);
    }

    if(LowSpeed) {
        SDIOSpeed = DEFINE_SDIO_LOWSPEED;
    } else {
        // changing DATA bus speed on both Host & Device sides
        MMPF_SDIO_ReadReg( SDMMCArg, 0, 0x13, &ret);
        if ((ret & 0x01) == 0) {
            SDIOSpeed = DEFINE_SDIO_FULLSPEED;
        }
        else
        {
            SDIOSpeed = DEFINE_SDIO_FULLSPEED;
            do {
                MMPF_SDIO_WriteReg(SDMMCArg, 0, 0x13, (ret & 0xFD) | 0x02);  // select HIGH SPEED
                MMPF_SDIO_ReadReg( SDMMCArg, 0, 0x13, &ret);
                if((ret& 0x03) == 0x03) {
                    SDIOSpeed = DEFINE_SDIO_HIGHSPEED;
                    break;
                }
                SetHSCnt--;
                MMPF_OS_Sleep(1);
            } while(SetHSCnt);
        }
    }
    //MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulSDclk);
    ulSDclk = m_ulSDG0SrcClk;   //wilson@110509

    if (ulSDclk % SDIOSpeed==0) {
        divValue = ulSDclk / SDIOSpeed;
    }
    else {
        divValue = (ulSDclk - (ulSDclk % SDIOSpeed))/SDIOSpeed + 1;
    }

    if ((divValue == 1)||(divValue == 3)){
        if (divValue == 1)
            MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_1);
        else
            MMPF_SD_SwitchClock(SDMMCArg, SD_CLOCK_DIV_3);
    }
    else {
        if (divValue%2 != 0)
            divValue++;
        MMPF_SD_SwitchClock(SDMMCArg, (divValue>>1)-1);
    }
    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
    #endif
#else
    MMPF_SD_WaitCount(10000);
    i = 0;
    do {
        MMPF_SD_WaitCount(40000);
        if (!MMPF_SD_SendCommand(SDMMCArg, IO_SEND_OP_COND, 0x000000)) {
            i++;
        }
        else {
            RTNA_DBG_Str0("SDIO_RESET_ERROR\r\n");
            return  MMP_SD_ERR_RESET;
        }
    } while( !(pSD->SD_RESP.D[3] & 0x80000000) && (i < 10));
    if (i != 10) {
        RTNA_DBG_Str1("Get 0 OCR success. ");
        RTNA_DBG_Short1(i);
        RTNA_DBG_Str1(" times\r\n");
    }
    else {
        // After OCR request response, then set OCR
        i = 0;
        do {
            MMPF_SD_WaitCount(4000);
            if (!MMPF_SD_SendCommand(SDMMCArg, IO_SEND_OP_COND, 0x180000)) {
                i++;
            }
            else {
                RTNA_DBG_Str0("SDIO_RESET_ERROR\r\n");
                return  MMP_SD_ERR_RESET;
            }
        }
        while (!(pSD->SD_RESP.D[3] & 0x80000000) && (i < 10));
        if (i != 10) {
            RTNA_DBG_Str1("Get 1 OCR success. ");
            RTNA_DBG_Short1(i);
            RTNA_DBG_Str1(" times\r\n");
        }
        else {
            RTNA_DBG_Str0("Get OCR fail.");
            return MMP_SD_ERR_RESET;
        }
    }

    // CMD  3
    if(!MMPF_SD_SendCommand(SDMMCArg, SEND_RELATIVE_ADDR, 0)) {
        ulSDIOCardAddr = (MMP_ULONG)pSD->SD_RESP.D[3];
        #if 0
        RTNA_DBG_Long2(ulSDIOCardAddr);
        RTNA_DBG_Str2("=ulSDIOCardAddr\r\n");
        #endif
    }
    else {
        return MMP_SD_ERR_RESET;
    }
    SDMMCArg->usBlockLen = 0x200;
    pSD->SD_BLK_LEN = SDMMCArg->usBlockLen;
    // Select SD card
    if (MMPF_SD_SendCommand(SDMMCArg, SELECT_CARD, ulSDIOCardAddr))
    {
        return MMP_SD_ERR_RESET;
    }

    SDMMCArg->usCurClkDiv = SD_CLOCK_DIV_2; //For Diamond, SD defalut clock src is PLL_G0/3

    SDMMCArg->bIsSD = MMP_TRUE;
    //RTNA_DBG_Str(0,"RESET SDIO SUCCESS\r\n");

    MMPF_SD_SwitchClock(SDMMCArg, SDMMCArg->usCurClkDiv);
    RTNA_DBG_Str(0,"RESET SDIO SUCCESS2\r\n");
    //pSD->SD_CTL_1 = WAIT_LAST_BUSY_EN| AUTO_CLK_EN | RD_TOUT_EN | WR_TOUT_EN | R1B_TOUT_EN;
#endif

    return  MMP_ERR_NONE;
} /* MMPF_SDIO_Reset */


//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_WriteReg
//------------------------------------------------------------------------------
MMP_ERR MMPF_SDIO_WriteReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE  fun_num, MMP_ULONG reg_addr, MMP_UBYTE src)
{
    MMP_USHORT ret;

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_TRUE);
    #endif

    if(MMP_ERR_NONE != MMPF_SDIO_BusAcquire(SDMMCArg)) {
        DBG_S(0, "MMPF_SDIO_EnableOutputClk aquire sem failed\r\n");
        return MMP_SD_ERR_BUSY;
    }
#if 1
    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_DIRECT, (RW_FLAG(1)| FUN_NUM(fun_num) | RAW_FLAG(1)
                                            | REG_ADD(reg_addr) | src)) ;
#else
    param = (RW_FLAG(1)| FUN_NUM(fun_num) | RAW_FLAG(1) | REG_ADD(reg_addr) | src);
    RTNA_DBG_Long(3, (param << 8) );
    RTNA_DBG_Str(3, " ");
    RTNA_DBG_Long(3, (param) );
    RTNA_DBG_Str(3, ":param for SD controller while CMD52 write.\r\n");
    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_DIRECT, param);
#endif

    MMPF_SDIO_BusRelease(SDMMCArg);

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
    #endif

    if (ret) {
        RTNA_DBG_Str3("write reg fail\r\n");
        return MMP_SD_ERR_COMMAND_FAILED;
    }
    else {
        return MMP_ERR_NONE;
    }
} /* MMPF_SDIO_WriteReg */

//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_ReadReg
//------------------------------------------------------------------------------
MMP_ERR MMPF_SDIO_ReadReg(stSDMMCHandler *SDMMCArg, MMP_UBYTE fun_num, MMP_ULONG reg_addr, MMP_UBYTE *p_dst)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  ret;

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_TRUE);
    #endif

    if(MMP_ERR_NONE != MMPF_SDIO_BusAcquire(SDMMCArg)) {
        DBG_S(0, "MMPF_SDIO_EnableOutputClk aquire sem failed\r\n");
        return MMP_SD_ERR_BUSY;
    }
#if 1
    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_DIRECT, (RW_FLAG(0)| FUN_NUM(fun_num) | RAW_FLAG(0)
                                            | REG_ADD(reg_addr) | 0));
#else
    param = (RW_FLAG(0)| FUN_NUM(fun_num) | RAW_FLAG(0) | REG_ADD(reg_addr) | 0);
    RTNA_DBG_Long(3, (param << 8) );
    RTNA_DBG_Str(3, " ");
    RTNA_DBG_Long(3, (param) );
    RTNA_DBG_Str(3, ":param for SD controller while CMD52 read.\r\n");
    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_DIRECT, param);
#endif

    if(p_dst != NULL) {
        *p_dst = pSD->SD_RESP.D[3] & 0xFF;
    }

    MMPF_SDIO_BusRelease(SDMMCArg);

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, FALSE);
    #endif

    if (ret) {
        RTNA_DBG_Str3("read reg fail\r\n");
        return MMP_SD_ERR_COMMAND_FAILED;
    }
    else {
        #if 0
        RTNA_DBG_Str3("MMPF_SDIO_ReadReg:");
        RTNA_DBG_Long3( pSD->SD_RESP.D[3] );
        RTNA_DBG_Str3("\r\n");
        #endif
        return MMP_ERR_NONE;
    }
} /* MMPF_SDIO_ReadReg */
//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_WriteMultiReg
//------------------------------------------------------------------------------
MMP_ERR MMPF_SDIO_WriteMultiReg(
            stSDMMCHandler *SDMMCArg,
            MMP_UBYTE           fun_num,
            MMP_UBYTE           blk_mode,
            MMP_UBYTE           op_code,
            MMP_ULONG           reg_addr,
            MMP_ULONG           count,
            MMP_ULONG           blk_size,
            MMP_UBYTE           *p_src)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  ret;

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_TRUE);
    #endif

    if(MMP_ERR_NONE != MMPF_SDIO_BusAcquire(SDMMCArg)) {
        DBG_S(0, "MMPF_SDIO_EnableOutputClk aquire sem failed\r\n");
        return MMP_SD_ERR_BUSY;
    }

    pSD->SD_DMA_ST_ADDR = (MMP_ULONG)p_src;
    #if 0
    RTNA_DBG_Long(1,pSD->SD_DMA_START_ADDR_L);
    RTNA_DBG_Str(1,"==pSD->SD_DMA_START_ADDR_L\r\n");
    #endif

    if (blk_mode) {
        pSD->SD_CMD_REG_1 = ADTC_WRITE;
        pSD->SD_BLK_NUM = count;
        pSD->SD_BLK_LEN = blk_size;
    }
    else {
        pSD->SD_CMD_REG_1 = ADTC_WRITE;
        pSD->SD_BLK_NUM = 1;
        pSD->SD_BLK_LEN = count;
    }

    // Clear all previous status
    pSD->SD_CPU_INT_SR = (DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE|CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|CMD_SEND_DONE);

    pSD->SD_CPU_INT_EN |= DATA_SEND_DONE;
    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_EXTENDED, (RW_FLAG(1)| FUN_NUM(fun_num)
                            | BLK_MODE(blk_mode) | OP_CODE(op_code)| REG_ADD(reg_addr) | count)) ;

    if(ret) {
        MMPF_SDIO_BusRelease(SDMMCArg);
        RTNA_DBG_Str0("write cmd fail.\r\n");
        #if (SDIO_SW_CLK_CTL == 1)
        MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }
#if (SDIO_CPU_NONBLOCKING==1)
    MMPF_SD_WaitIsrDone(SDMMCArg, 0x10000);
#endif

    MMPF_SDIO_BusRelease(SDMMCArg);

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
    #endif

    return MMP_ERR_NONE;
} /* MMPF_SDIO_WriteMultiReg */

//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_ReadMultiReg
//------------------------------------------------------------------------------
MMP_ERR   MMPF_SDIO_ReadMultiReg(
            stSDMMCHandler *SDMMCArg,
            MMP_UBYTE           fun_num,
            MMP_UBYTE           blk_mode,
            MMP_UBYTE           op_code,
            MMP_ULONG           reg_addr,
            MMP_ULONG           count,
            MMP_ULONG           blk_size,
            MMP_UBYTE          *p_dst)
{
    AITPS_SD    pSD = SDMMCArg->pSD;
    MMP_USHORT  ret;
    #if (SDIO_CPU_NONBLOCKING==0)
    MMP_ULONG   time_out;
    #endif

    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_TRUE);
    #endif

    if(MMP_ERR_NONE != MMPF_SDIO_BusAcquire(SDMMCArg)) {
        DBG_S(0, "MMPF_SDIO_EnableOutputClk aquire sem failed\r\n");
        return MMP_SD_ERR_BUSY;
    }


    pSD->SD_DMA_ST_ADDR = (MMP_ULONG)p_dst;

    if (blk_mode) {
        pSD->SD_CMD_REG_1 = ADTC_READ;
        pSD->SD_BLK_NUM = count;
        pSD->SD_BLK_LEN = blk_size;
    }
    else {
        pSD->SD_CMD_REG_1 = ADTC_READ;
        pSD->SD_BLK_NUM = 1;
        pSD->SD_BLK_LEN = count;
    }

    // Clear all previous status
    pSD->SD_CPU_INT_SR = (DATA_CRC_ERR|DATA_TOUT|DATA_ST_BIT_ERR|DATA_SEND_DONE|CMD_RESP_CRC_ERR|CMD_RESP_TOUT|BUSY_TOUT|CMD_SEND_DONE);

#if (SDIO_CPU_NONBLOCKING==1) //++Philip@070531: Add for ASYNC SDIO READ
    pSD->SD_CPU_INT_EN |= DATA_SEND_DONE; // Enable interrupt when "DATA SEND DONE"
#endif /* (SDIO_CPU_NONBLOCKING==1) */

    ret = MMPF_SD_SendCommand(SDMMCArg, IO_RW_EXTENDED, (RW_FLAG(0)| FUN_NUM(fun_num)
                            | BLK_MODE(blk_mode) | OP_CODE(op_code) | REG_ADD(reg_addr) | count));

    if(ret) {
        MMPF_SDIO_BusRelease(SDMMCArg);
        RTNA_DBG_Str3("read cmd fail\r\n");
        #if (SDIO_SW_CLK_CTL == 1)
        MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
        #endif
        return MMP_SD_ERR_COMMAND_FAILED;
    }

#if (SDIO_CPU_NONBLOCKING==1) //++Philip@070531: Add for ASYNC SDIO READ
    MMPF_SD_WaitIsrDone(SDMMCArg, 0x10000);
#elif (SDIO_CPU_NONBLOCKING==0)
    if (time_out == SD_TIMEOUT_COUNT) {

        RTNA_DBG_Str0("==read time-out");
        #if 0
        RTNA_DBG_Str0("blk, num:");
        RTNA_DBG_Long3(blk_size);
        RTNA_DBG_Str0(":");
        RTNA_DBG_Long3(count);
        RTNA_DBG_Str0("\r\n");
        #endif //0
        MMPF_SDIO_BusRelease(SDMMCArg);

        #if (SDIO_SW_CLK_CTL == 1)
        MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
        #endif

        return MMP_SD_ERR_DATA;
    }
#endif /* (SDIO_CPU_NONBLOCKING==0) */

    #if 0
    if (blk_mode) {
//        RTNA_DBG_Str3("read blk multi data:\r\n");
        for (i = 0; i < 0x8000; i++) ; /* dummy wait */

        for (i = 0; i < (blk_size*count); i++) {
            p_dst[i] = *(MMP_UBYTE *) (m_ulSDDmaAddr + i);
            #if 0
            RTNA_DBG_Byte3(p_dst[i]);
            RTNA_DBG_Str3(" ");
            #endif
        }
    }
    else {
//        RTNA_DBG_Str3("read byte multi data:\r\n");
        for (i = 0; i < (count); i++) {
            p_dst[i] = *(MMP_UBYTE *) (m_ulSDDmaAddr + i);
            #if 0
            RTNA_DBG_Byte3(p_dst[i]);
            RTNA_DBG_Str3(" ");
            #endif
        }
        #if 0 //PhilipDebug
        RTNA_DBG_Long(3, (int)&p_dst[0]);
        RTNA_DBG_Str(3, ": &p_dst[0]\r\n");
        RTNA_DBG_Long(3, m_ulSDDmaAddr);
        RTNA_DBG_Str(3, ": m_ulSDDmaAddr\r\n");
        for (i = 0; i < 4; i++) {

            RTNA_DBG_Byte3(p_dst[i]);
            RTNA_DBG_Str3(" ");
        }
        RTNA_DBG_Str(3, ": the first 4 bytes of p_dst[]\r\n");
        #endif
    }
    #endif
    MMPF_SDIO_BusRelease(SDMMCArg);
    #if (SDIO_SW_CLK_CTL == 1)
    MMPF_SDIO_EnableOutputClk(SDMMCArg, MMP_FALSE);
    #endif
    return MMP_ERR_NONE;
} /* MMPF_SDIO_ReadMultiReg */


//------------------------------------------------------------------------------
//  Function    : MMPF_SDIOBusyAcquireSema
//------------------------------------------------------------------------------
MMP_ERR MMPF_SDIO_BusAcquire(stSDMMCHandler *SDMMCArg)
{
    MMP_UBYTE ret;

    ret = MMPF_OS_AcquireSem(SDMMCArg->SDIOBusySemID, 0);
    if (ret == OS_NO_ERR) {
        return MMP_ERR_NONE;
    }
    else {
        RTNA_DBG_Long1(ret);
        RTNA_DBG_Str1(":SDIO OSSemPend fail\r\n");
        return MMP_SD_ERR_BUSY;
    }
}
//------------------------------------------------------------------------------
//  Function    : MMPF_SDIO_BusRelease
//------------------------------------------------------------------------------
MMP_USHORT MMPF_SDIO_BusRelease(stSDMMCHandler *SDMMCArg)
{
    MMP_UBYTE ret;

    ret = MMPF_OS_ReleaseSem(SDMMCArg->SDIOBusySemID);
    if ( ret== OS_NO_ERR) {
    }
    else {
        RTNA_DBG_Long1(ret);
        RTNA_DBG_Str1(":SDIO OSSemPost fail\r\n");
    }
    return ret;
}

//------------------------------------------------------------------------------
//  FUNCTION    : MMPF_SDIO_GetHandler
//------------------------------------------------------------------------------
stSDMMCHandler *MMPF_SDIO_GetHandler(MMP_ULONG id)
{
    return &m_SDMMCArg[id];
}

//------------------------------------------------------------------------------
//  FUNCTION    : MMPF_SDIO_EnaISR
//------------------------------------------------------------------------------
void MMPF_SDIO_EnaISR(stSDMMCHandler *SDMMCArg, MMP_USHORT enable)
{
    AITPS_SD    pSD = SDMMCArg->pSD;

    pSD->SD_CPU_INT_SR = SDIO_INT;
    if (enable)
        pSD->SD_CPU_INT_EN |= SDIO_INT;
    else
        pSD->SD_CPU_INT_EN &= (~SDIO_INT);
}

//------------------------------------------------------------------------------
//  FUNCTION    : MMPF_SDIO_EnableOutputClk
//------------------------------------------------------------------------------
MMP_ERR MMPF_SDIO_EnableOutputClk(stSDMMCHandler *SDMMCArg, MMP_USHORT enable)
{
#if (SDIO_SW_CLK_CTL == 1)
    AITPS_SD    pSD = SDMMCArg->pSD;

    if(MMP_ERR_NONE != MMPF_SDIO_BusAcquire(SDMMCArg)) {
        DBG_S(0, "MMPF_SDIO_EnableOutputClk aquire sem failed\r\n");
        return MMP_SD_ERR_BUSY;
    }
    if(enable) {
        SDMMCArg->ulSDIOClkCnt++;
    } else {
        if(SDMMCArg->ulSDIOClkCnt > 0) {
            SDMMCArg->ulSDIOClkCnt--;
        } else {
            DBG_S(0, "Fatal Error!!!! SDIO Output Clock is already disable.\r\n");
        }
    }

    if(SDMMCArg->ulSDIOClkCnt) {
        pSD->SD_CTL_1 |= CLK_EN;
        while(pSD->SD_CTL_1 & CLK_EN);
    } else {
        pSD->SD_CTL_1 |= CLK_DIS;
        while(pSD->SD_CTL_1 & CLK_DIS);
    }

    MMPF_SDIO_BusRelease(SDMMCArg);
#endif
    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  FUNCTION    : MMPF_SDIO_RegSDIOCBFunc
//------------------------------------------------------------------------------
void MMPF_SDIO_RegSDIOCBFunc(stSDMMCHandler *SDMMCArg, void *Func)
{
    SDMMCArg->MMPF_SDIO_CallBackISR = (SDIOCallBackFunc *)Func;
}
#endif /* (ENABLE_SDIO_FEATURE==1) */
/** @} */ // MMPF_SDIO

//------------------------------------------------------------------------------
// FUNCTION : MMPF_SD_CheckCardIn
//------------------------------------------------------------------------------
MMP_USHORT  MMPF_SD_CheckCardIn(stSDMMCHandler *SDMMCArg)
{
    #if (EN_CARD_DETECT == 1)
    AITPS_GPIO pGPIO = AITC_BASE_GPIO;

    if (gbSDCardDetectionPinNum[id]==0xFF) {
        return  SD_IN;
    }
    if (gbSDCardDetectionPolarity[id]) {
        if(pGPIO->GPIO_DATA[(gbSDCardDetectionPinNum[id]/32)] & (1 << (gbSDCardDetectionPinNum[id]%32)) ) {
            return  SD_IN;
        }
        else {
            MMPF_HIF_SetCpu2HostInt(MMPF_HIF_INT_SDCARD_REMOVE);
            return  SD_OUT;
        }
    }
    else {
        if(pGPIO->GPIO_DATA[(gbSDCardDetectionPinNum[id]/32)] & (1 << (gbSDCardDetectionPinNum[id]%32))) {
            MMPF_HIF_SetCpu2HostInt(MMPF_HIF_INT_SDCARD_REMOVE);
            return  SD_OUT;
        }
        else {
            return  SD_IN;
        }
    }
    #else
    return SD_IN;
    #endif
}

//------------------------------------------------------------------------------
// FUNCTION: MMPF_SD_CheckCardWP
//------------------------------------------------------------------------------
MMP_USHORT MMPF_SD_CheckCardWP(stSDMMCHandler *SDMMCArg)
{
#if (EN_CARD_WRITEPROTECT)
    AITPS_GPIO pGPIO = AITC_BASE_GPIO;

    if (gbSDCardWProtectPinNum[id]==0xFF) {
        return  0;
    }

    if (gbSDCardWProtectPolarity[id]) {
        if(pGPIO->GPIO_DATA[(gbSDCardWProtectPinNum[id]/32)] & (1 << (gbSDCardWProtectPinNum[id]%32)))
            return  1;
        else
            return  0;
    }
    else {
        if(pGPIO->GPIO_DATA[(gbSDCardWProtectPinNum[id]/32)] & (1 << (gbSDCardWProtectPinNum[id]%32))){
            return  0;
        }
        else
            return  1;
    }
#else
    return 0;
#endif
}

//------------------------------------------------------------------------------
// FUNCTION: MMPF_SD_EnableCardPWR
//------------------------------------------------------------------------------
void MMPF_SD_EnableCardPWR(stSDMMCHandler *SDMMCArg, MMP_BOOL enable)
{
#if (EN_CARD_PWRCTL)
    AITPS_GPIO pGPIO = AITC_BASE_GPIO;

    if (gbSDCardPwrCtlPinNum[id]==0xFF) {
        return;
    }

    if (gbSDCardPwrCtlPolarity[id]) {
        if(enable)
            pGPIO->GPIO_DATA[(gbSDCardPwrCtlPinNum[id]/32)] |= (1 << (gbSDCardPwrCtlPinNum[id]%32));
        else
            pGPIO->GPIO_DATA[(gbSDCardPwrCtlPinNum[id]/32)] &= (~(1 << (gbSDCardPwrCtlPinNum[id]%32)));
    }
    else {
        if(enable)
            pGPIO->GPIO_DATA[(gbSDCardPwrCtlPinNum[id]/32)] &= (~(1 << (gbSDCardPwrCtlPinNum[id]%32)));
        else
            pGPIO->GPIO_DATA[(gbSDCardPwrCtlPinNum[id]/32)] |= (1 << (gbSDCardPwrCtlPinNum[id]%32));
    }
#endif
}


#if (SD_CPU_NONBLOCKING == 1)
//------------------------------------------------------------------------------
//  Function    : MMPF_SD_WaitIsrDone
//------------------------------------------------------------------------------
MMP_ERR MMPF_SD_WaitIsrDone(stSDMMCHandler *SDMMCArg, MMP_ULONG waitcount)
{
    MMP_UBYTE ret;

    ret = MMPF_OS_AcquireSem(SDMMCArg->IntTriggerSemID, waitcount);
    if (ret == OS_NO_ERR) {
        return MMP_ERR_NONE;
    }
    else {
        RTNA_DBG_Long0(ret);
        RTNA_DBG_Str0(":SD OSSemPend fail\r\n");
        return MMP_SD_ERR_BUSY;
    }
}
#endif /* (SD_CPU_NONBLOCKING == 1) */

//------------------------------------------------------------------------------
//  Function    : MMPF_SD_WaitCount
//------------------------------------------------------------------------------
void    MMPF_SD_WaitCount(MMP_ULONG count)
{
    MMP_ULONG   i;
    for (i = 0; i < count; i++);
}

/** @brief Set temp buffer address for SD driver

@param[in] ulStartAddr Start address for temp buffer address.
@param[in] ulSize Size for temp buffer. Currently buffer size is only 64 Byte for SDHC message handshake
@return NONE
*/
void    MMPF_SD_SetTmpAddr(MMP_ULONG ulStartAddr, MMP_ULONG ulSize)
{
//#if (ENABLE_SDHC_SWITCH_HIGH_SPEED==1)
    m_ulSDDmaAddr = ulStartAddr;
    ulSize = ulSize; /* dummy for prevent compile warning */
    RTNA_DBG_Str(0, "sd dma addr: ");
    RTNA_DBG_Long(0, ulStartAddr);
    RTNA_DBG_Str(0, " \r\n");
//#endif
}

/** @brief Get temp buffer address for SD driver

@param[out] ulStartAddr Start address for temp buffer address.
@return NONE
*/
void    MMPF_SD_GetTmpAddr(MMP_ULONG *ulStartAddr)
{
    #if (PMP_USE_MUON == 1)||defined(MBOOT_FW)
    MMPF_MMU_FlushDCacheMVA(m_ulSDDmaAddr, 512);
    #endif
    *ulStartAddr = m_ulSDDmaAddr;
}
#endif // (OS_TYPE == OS_USOSII)


/** @} */ // MMPF_SD

#endif  /* (USING_SD_IF) */
