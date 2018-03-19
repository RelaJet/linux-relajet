//==============================================================================
//
//  File        : mmpf_usb_ctl.c
//  Description : Firmware USB Controller Driver
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================

#include <mach/includes_fw.h>

#include <mach/mmp_reg_usb.h>
#if 0//(USB_EN)

#include "mmps_usb.h"
#include "lib_retina.h"
#include "reg_retina.h"
#include "ait_utility.h"
#include "mmpf_hif.h"
#include "mmpf_usbextrn.h"
#include "mmpf_usbpccam.h"
#include "mmpf_usbvend.h"
#include "mmpf_usbphy.h"
#include "mmpf_usbmtp.h"
#include "mmpf_usbdps.h"
#include "mmpf_usbmsdc.h"
#include "mmpf_timer.h"
#include "mmpf_vif.h"
#include "mmpf_storage_api.h"
#include "mmp_reg_usb.h"
#if (SUPPORT_UVC_FUNC == 1)
#include "mmps_3gprecd.h"
#include "pCam_api.h"
#include "mmp_reg_ibc.h"
#include "mmp_reg_jpeg.h"
#include "mmp_reg_vif.h"
#include "mmpf_usbuvc.h"
#include "pCam_usb.h"
#endif
#include "mmpf_system.h"
#include "mmpf_display.h"

#if (SUPPORT_UVC_FUNC)
#include "ex_usb_otg.h"
#endif

/** @addtogroup MMPF_USB
@{
*/

//==============================================================================
//
//                              LOCAL VARIABLES
//
//==============================================================================
#if (SUPPORT_UVC_FUNC)
#if (VIDRECD_MULTI_TRACK == 0)
static MMP_ULONG                m_ulRecNameIdx  = 0;
static MMP_USHORT               m_usResolIdx    = 4;//FHD
#endif
#endif

//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================
MMP_BOOL            usIsAdapter = MMP_TRUE;
MMPF_USB_OP_MODE    glUsbApplicationMode = USB_MAX_MODE;
MMPF_OS_SEMID       m_usbReadSem;

#if (MSDC_WR_FLOW_TEST_EN == 1)
MMP_ULONG           m_ulMSDCtimer = 0;
MMPF_OS_SEMID       m_usbWriteSem;
_st_usb_write_buff_queue m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH + 1];

MMP_ULONG           m_ulMSDC_WbufQ_Addr;
MMP_ULONG           m_ulMSDC_CurWrite_Point;
MMP_ULONG           m_ulMSDC_CurRead_Point;
MMP_ULONG           m_semCount = 0;
#endif

#if ( USE_SEC_MTP_DEV_FW == 0 )
MMP_UBYTE           m_ubPtpProcess = go_init;
#endif

#if (SUPPORT_UVC_FUNC == 1)
MMP_UBYTE           gbStopMJPEGEvent = 1, gbStopYUV422Event = 1;
MMP_UBYTE           gbOrgVideoResolution;  //for still capture
MMP_ULONG           glSystemEvent = 0;
MMPS_3GPRECD_VIDEO_FORMAT   gbOrgVideoFormat;
#endif
MMPF_OS_SEMID    m_UVCHBulkInSemID;
//MMPF_OS_SEMID    m_UVCHRefreshSemID;
MMP_UBYTE   h264_nalu[64];
MMP_UBYTE   Profile_IDC;
MMP_UBYTE   Level_IDC;
MMP_BOOL    bUVCHRecdEnabled = MMP_FALSE;

//==============================================================================
//
//                              EXTERN VARIABLES
//
//==============================================================================
#if (MSDC_WR_FLOW_TEST_EN == 1)
extern MMP_BOOL             m_bIsHostCancelMSDC;
extern MMP_BOOL             m_bMSDC_StartFlush;
extern MMP_ULONG            glUsbMSDCState;
extern MMP_BOOL             m_bMSDCSmallSizeSect;
extern MMP_BOOL             m_bMSDCWriteSuspend;
extern MMP_ULONG            glMSDCLun;
extern MMP_USHORT           gsCardExist[MSDC_MAX_UNITS];
extern STORAGE_UNIT         gpMSDC_Device[MSDC_MAX_UNITS];
#endif

extern MMP_ULONG            glMSDCTxBufAddr;
extern MMP_ULONG            glMSDCRxBufAddr;
extern MMPF_OS_FLAGID       USB_OP_Flag;
extern MMPF_USB_OP_MODE     glUsbApplicationMode;
extern MMP_ULONG            glMSDCInterfaceBufSize;
extern MMP_ULONG            glMSDCBufStarAddr;
#if (SUPPORT_PCSYNC)
extern MMP_ULONG            glPCSyncHandshackBufAddr;
extern MMP_ULONG            glPCSyncHostOutBufAddr, glPCSyncHostInBufAddr;
#endif
extern MMP_ULONG            glUsbCtlBufStarAddr;
extern MMP_ULONG            glMSDCCmdBufAddr;
extern MMPS_USB_StatusMode  gsUsbCurStatus;

extern MMP_UBYTE            m_MSDC_ReadFlag;
extern MMP_UBYTE            m_MSDC_WriteFlag;

#if 1//(SUPPORT_PCCAM_FUNC == 1)
extern MMP_ULONG            glPCCamCompressBufAddr;
extern MMP_ULONG            glPCCamCompressBufSize;
extern MMP_ULONG            glPCCamLineBufAddr;
extern MMP_ULONG            glPCcamCaptureEnable;
#endif /* (SUPPORT_PCCAM_FUNC==1) */

#if (SUPPORT_MTP_FUNC == 1)||(SUPPORT_DPS_FUNC == 1)
extern MTP_DMA_HANDLE       MTPBufHandle;
extern MMP_UBYTE            MtpInitInMainON;
#endif

#if (USE_SEC_MTP_DEV_FW == 0)
extern MMP_ULONG            mtpRXbufAddr;
extern MMP_UBYTE            MTPCurPhase;
extern ContainerStruct      *ConStrPt;
extern ObjectInfoStruct     CurObjInfo;
extern OP_RP_dataset        curOPinfo;

extern MMPF_OS_FLAGID       MTP_Flag;
extern MMP_BOOL             gbIsStalled;
extern MMP_USHORT           gsUsbRxEpStatus[MAX_RX_ENDPOINT_NUM];
extern MMP_USHORT           gsUsbTxEpStatus[MAX_TX_ENDPOINT_NUM];
#endif

#if (USB_SUSPEND_TEST == 1)
extern volatile MMP_UBYTE   gbUSBSuspendEvent;
extern volatile MMP_UBYTE   gbUSBSuspendFlag;
#endif

#if (SUPPORT_UVC_FUNC == 1)
extern MMP_UBYTE            gbDevicePowerSavingStatus;
extern MMP_BOOL             m_bIsVCStart;
extern MMP_UBYTE            gbdrawflag;
extern MMP_BOOL             m_bVidRecdPreviewStatus;
extern STREAM_CFG           gsYUY2Stream_Cfg;
extern MMP_ULONG            glPccamResolution;
extern STILL_PROBE          sc;
extern MMP_UBYTE            gbCaptureBayerRawFlag;
extern MMP_ULONG            glPCCamJpegSize;
extern MMP_ULONG            glPCCAM_VIDEO_BUF_ADDR;
extern MMP_UBYTE            gbausentflag;
extern MMP_UBYTE            gbUVCDSCCommand;
extern MMPS_3GPRECD_VIDEO_FORMAT    m_VideoFmt;
extern MMPF_USBUVC_CAPTURE_EVENT    gbStillCaptureEvent;
#endif
#if (SUPPORT_UVC_FUNC)
extern UVCH_XFER_CTL        uvch_xfer_ctl;
extern SD_BUF_CTL           sd_buf_ctl;
extern LCD_BUF_CTL          lcd_buf_ctl;
extern MMP_UBYTE            video_format;
extern UVCH_Rx_Param        m_UVCHRxArg;
extern MMP_BYTE             gbRecordFileName[20];
extern MMP_BYTE             gbUVCRecdFileName[];
#endif

//==============================================================================
//
//                              EXTERN FUNCTIONS
//
//==============================================================================
#if (CHIP == P_V2)
extern MMP_ERR  MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP ubGroupNum, 
                                      MMP_ULONG *ulGroupFreq);
#endif

#if (MSDC_WR_FLOW_TEST_EN == 1)
extern void     MMPF_MSDC_HandleWriteSuspend(void);
extern void     MMPF_MSDC_SetCswHeader(MMP_UBYTE);
extern void     MMPF_MSDC_TriggerUsbTX(MMP_ULONG, MMP_ULONG);
#endif

#if (USE_SEC_MTP_DEV_FW == 0)
extern void     SetProptectStatusByHandle(MMP_ULONG handle, MMP_USHORT protFlag);
extern void     GetParentHandle(MMP_ULONG handle, MMP_USHORT *ptHandle);
extern MMP_UBYTE DelObjByObjHdl(MMP_ULONG handle,MMP_ULONG *status);
#endif

//wilson@110704: for uvc porting
#if (SUPPORT_UVC_FUNC == 1)
extern void     MMPF_USB_ResetDMA(void);
extern MMP_ERR  MMPF_UVCWRAP_AutoFocus(void);
extern void     uac_process_audio_data(void);
extern PCAM_USB_INFO *pcam_get_info(void);
#endif

extern MMP_UBYTE USB_UVCH_Chk_Payload_Header(MMP_UBYTE ubShow);
extern MMP_UBYTE USB_UVCH_Check_AIT_Header(MMP_UBYTE ubShow, MMP_UBYTE chk);
extern MMP_UBYTE USB_UVCH_Check_H264_Header(MMP_UBYTE ubShow);
extern void USB_UVCH_ReqOnePkt(void);
extern void USB_UVCH_TriggerDmaRx(void);
extern void USBCore_Host_SET_CMD(MMP_UBYTE bReqType, MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen);

extern MMP_ERR MMPF_Display_SetWinAddr(MMPF_DISPLAY_WINID winID, MMP_ULONG ulBaseAddr, MMP_ULONG ulBaseUAddr, MMP_ULONG ulBaseVAddr);
extern MMP_ERR MMPF_Display_SetWinActive(MMPF_DISPLAY_WINID winID, MMP_BOOL bEnable);
extern void CreateRecordFileName(MMP_USHORT usResolIdx, MMP_ULONG usIndex , MMP_UBYTE ubFileType, MMP_UBYTE ubEmerg);
extern void UVCRecordCbFileFull(void);

//==============================================================================
//
//                              FUNCTION PROTOTYPE
//
//==============================================================================
void USB_UVCH_Record_Setting(void);

//==============================================================================
//  Function    : MMPF_USB_ConnectDevice
//==============================================================================
MMP_ERR MMPF_USB_ConnectDevice(void)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_POWER |= USB_DEV_SOFT_CONN;
    return MMP_ERR_NONE;
}
#endif
//==============================================================================
//  Function    : MMPS_USB_StopDevice
//==============================================================================
/**
    @brief Disable USB device.
    @return MMP_ERR_NONE.
*/
MMP_ERR MMPF_USB_StopDevice(void)
{
	extern void MMPF_USBPHY_RollbackHS(void);

    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_POWER &= ~(USB_DEV_SOFT_CONN);

	MMPF_USBPHY_RollbackHS();

    return MMP_ERR_NONE;
}



#if 0
//==============================================================================
//  Function    : MMPF_USB_DetectMode
//==============================================================================
/**
    @brief
    @return
*/
void MMPF_USB_DetectMode(void *ptmr, void *parg)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    if (usIsAdapter == MMP_TRUE)
        gsUsbCurStatus = MMPS_USB_ISADAPTER;

    RTNA_DBG_Long(0, usIsAdapter);
    RTNA_DBG_Long(0, gsUsbCurStatus);
    RTNA_DBG_Str(0," timer stop\r\n");

    if (glUsbApplicationMode == USB_DETECT_MODE) {
        //disable suspend mode
        pUSB_CTL->USB_POWER &= ~(USB_SUSPENDM_EN | USB_RESUME | USB_RESET);
        #ifdef USB_DMA_MODE
        //disable EP0 interrupt
        pUSB_CTL->USB_TX_INT_EN &= ~(USB_INT_EP0);
        //Remove? disable suspend and reset interrupt
        pUSB_CTL->USB_INT_EVENT_EN &= ~(USB_INT_RESET | USB_INT_SUSPEND);
        #else
        pUSB_CTL->USB_TX_INT_EN &= ~(USB_INT_EP0 | USB_INT_TX_EP(1));
        #endif
        pUSB_CTL->USB_POWER &= ~(USB_DEV_SOFT_CONN);
    }
}

void MMPF_USB_ActiveModeDetect(void)
{
    MMPF_OS_StartTimer(300, MMPF_OS_TMR_OPT_ONE_SHOT, MMPF_USB_DetectMode, NULL);
}

void MMPF_SetUSBChangeMode(MMPF_USB_OP_MODE mode)
{
    AITPS_AIC       pAIC = AITC_BASE_AIC;
    AITPS_USB_DMA   pUSB_DMA = AITC_BASE_USBDMA;
    #if (CHIP == P_V2)
    AITPS_GBL       pGBL = AITC_BASE_GBL;
    #endif

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB, MMP_TRUE);

    #if (CHIP == P_V2)
    pGBL->GBL_CLK_USB_DIV = 0x01;
    pGBL->GBL_CLK_PATH_CTL3 |= GBL_USBPHY_CLK_SRC_PMCLK;
    pUSB_DMA->USB_UTMI_PHY_CTL1 &= ~(UTMI_XTL_30MHZ);
    #endif
    #if (CHIP == MCR_V2)
    /* We set PLL M by control register, but not by XTL_SELECT,
     * so we don't need to care the setting above.
     * And, we don't change USB PHY clock source here,
     * it should be well set during PLL configure in boot loader.
     */
    #endif

    // 0: Still mode, 1: MSDC mode  2: PCSync mode;
    RTNA_DBG_PrintByte(0, mode);

    if (mode == USB_DETECT_MODE) {
        switch(glUsbApplicationMode) {
        #if (SUPPORT_DPS_FUNC)
        case USB_DPS_MODE:
        #endif
        #if (SUPPORT_MTP_FUNC)
        case USB_MTP_MODE:
        #endif
        case USB_MSDC_MODE:
#if (SUPPORT_MSDC_SCSI_AIT_CMD_MODE == 1)
        case USB_MSDC_AIT_DEBUG_MODE:
#endif
            usIsAdapter = MMP_TRUE;
            RTNA_DBG_Str(0,"keep original mode detect\r\n");
            MMPF_USB_ActiveModeDetect();
            return;

        #if (SUPPORT_PCCAM_FUNC)||(SUPPORT_UVC_FUNC)
        case USB_PCCAM_MODE:
        #endif
        #if (SUPPORT_PCSYNC_FUNC)
        case USB_PCSYNC_MODE:
        #endif
        default:
            usIsAdapter = MMP_TRUE;
            glUsbApplicationMode = USB_DETECT_MODE;
            RTNA_DBG_Str(0,"start active mode detect\r\n");
            MMPF_USB_ActiveModeDetect();
            break;
        }
    }
    else {
        glUsbApplicationMode = mode;
    }

    if (glUsbApplicationMode == USB_ADAPTER_MODE) {
        return; //USB adapter or USB charger connected, just do nothing.
    }

    /* ============================
     * ===== USB PHY Power Up =====
     * ============================*/
    MMPF_USBPHY_Initalize(MMP_FALSE);

    pUSB_DMA->USB_UTMI_PHY_CTL0 &= ~(CLKOSC_OFF_IN_SUSPEND);
    pUSB_DMA->USB_UTMI_PHY_CTL0 |= UTMI_PLL_ON_IN_SUSPEND;
    // remove DP/DM Pulldown
    pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_DPPULLDOWN | UTMI_DMPULLDOWN);
    //pUSB_DMA->USB_UTMI_CTL1 |= UTMI_USBPHY_NOSUSPEND;
    // disable USB testmode
    pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_USBPHY_TESTMODE);

    UsbRestIntHandler();

    RTNA_AIC_Open(pAIC, AIC_SRC_USB, usb_isr_a,
                  AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_USB);

    RTNA_DBG_Str(0,"===USB Init OK\r\n");
    RTNA_DBG_Str(0,"===USB D+D- OK\r\n");
}

#if (MSDC_WR_FLOW_TEST_EN == 1)
/*
initial msdc write buffer setting, include buffer allocation and point initialization.
*/
void MMPF_SetMSDCWriteBuffQueue(MMP_ULONG buf_addr, MMP_ULONG bufBlockSize)//, MMP_ULONG bufDepth)
{
    MMP_UBYTE i;

    m_ulMSDC_WbufQ_Addr = buf_addr;

    RTNA_DBG_PrintLong(0, m_ulMSDC_WbufQ_Addr);
    for ( i=0; i<=MMPF_USB_W_DATA_BUFF_DEPTH; i++){
        m_stUSB_MSDC_WBUFF[i].dataBuf= (MMP_UBYTE *)(m_ulMSDC_WbufQ_Addr+bufBlockSize*i);
        RTNA_DBG_Long(0, (MMP_ULONG)m_stUSB_MSDC_WBUFF[i].dataBuf);
        RTNA_DBG_Str(0, "\r\n");
        m_stUSB_MSDC_WBUFF[i].isValid = 0;
    }
    RTNA_DBG_Str(0, "\r\n");

    m_ulMSDC_CurWrite_Point = 0;
    m_ulMSDC_CurRead_Point = 0;
}
#endif

void MMPF_SetMSDCBuf(unsigned int buf_addr, unsigned int sector_buf_size)
{
    glMSDCBufStarAddr = buf_addr;
    glMSDCTxBufAddr = buf_addr;
    glMSDCRxBufAddr = buf_addr;
    glMSDCInterfaceBufSize = sector_buf_size;

    #if 0 //Debug only
    RTNA_DBG_PrintLong(0, glMSDCTxBufAddr);
    RTNA_DBG_PrintLong(0, glMSDCRxBufAddr);
    RTNA_DBG_PrintLong(0, glMSDCInterfaceBufSize);
    #endif
}

void MMPF_SetUsbCtlBuf(unsigned int buf_addr)
{
    glUsbCtlBufStarAddr = buf_addr;
    glMSDCCmdBufAddr = buf_addr;
}

#if (SUPPORT_PCSYNC)
void MMPF_SetPCSYNCInBuf(unsigned int buf_addr)
{
    glPCSyncHostInBufAddr = buf_addr;
}
void MMPF_SetPCSYNCOutBuf(unsigned int buf_addr)
{
    glPCSyncHostOutBufAddr = buf_addr;
}
void MMPF_SetPCSYNCHandShakeBuf(unsigned int buf_addr)
{
    glPCSyncHandshackBufAddr = buf_addr;

    *( MMP_UBYTE * ) (glPCSyncHandshackBufAddr+PCSYNC_INFLAG_OFFSET_B)=0;
    *( MMP_UBYTE * ) (glPCSyncHandshackBufAddr+PCSYNC_OUTFLAG_OFFSET_B)=0;
    *( MMP_USHORT * ) ( glPCSyncHandshackBufAddr+PCSYNC_OUTSIZE_OFFSET_W)=0;
    *( MMP_USHORT * ) ( glPCSyncHandshackBufAddr+PCSYNC_INSIZE_OFFSET_W)=0;
}
#endif

#if (SUPPORT_UVC_FUNC == 1)
extern MMP_ULONG    glPCCamCompressBufAddr;
extern MMP_ULONG    glPCCamCompressBufSize;
extern MMP_ULONG    glPCCamLineBufAddr;
extern MMP_ULONG    glPCcamCaptureEnable;
void MMPF_SetPCCAMCompressBuf(unsigned int buf_addr,unsigned int buf_size)
{
    glPCCamCompressBufAddr = buf_addr;
    glPCCamCompressBufSize = buf_size;
}
void MMPF_SetPCCAMLineBuf(unsigned int buf_addr)
{
    glPCCamLineBufAddr = buf_addr;
}
void MMPF_EnablePCCamCapture(void)
{
    glPCcamCaptureEnable = 1;
}
#endif /* (SUPPORT_PCCAM_FUNC==1) */

void MMPF_SetMTPEPBuf(MMP_ULONG rxbuf_addr,MMP_ULONG txbuf_addr,MMP_ULONG size,MMP_ULONG mbuf_addr,MMP_ULONG mbuf_size) //Andy++
{
#if ( USE_SEC_MTP_DEV_FW == 0 )
    #if (SUPPORT_MTP_FUNC==1)||(SUPPORT_DPS_FUNC==1)
    extern MMP_ULONG RxOutBlockSiz;  //Andy
    extern MMP_ULONG TxOutBlockSiz;  //Andy

    MTPBufHandle.RxBufAddr=rxbuf_addr;
    MTPBufHandle.TxBufAddr=txbuf_addr;
    MTPBufHandle.RxTxBufSize=size;
    TxOutBlockSiz=size;  //Andy
    RxOutBlockSiz=size;  //Andy
    MTPBufHandle.MBufAddr=mbuf_addr; //Andy
    MTPBufHandle.MBufSize=mbuf_size; //Andy
    #endif
    #endif
}

#if (SUPPORT_DPS_FUNC==1)
void MMPF_StartDPSPrintJob(void){
    dps_StartDPSPrintJob();
}
void MMPF_AbortDPSPrintJob(void){
    dps_AbortDPSPrintJob();
}
void MMPF_ContinueDPSPrintJob(void){
    dps_ContinueDPSPrintJob();
}

MMP_USHORT MMPF_GetPrinterStatus(void){
    return GetPrinterStatus();
}
MMP_ULONG MMPF_DPSGetBufAddr(MMP_ULONG buftype){
    return DPSGetbufAddr(buftype);
}
#endif
#if ( USE_SEC_MTP_DEV_FW == 0 )
#if (SUPPORT_DPS_FUNC == 1)||(SUPPORT_MTP_FUNC == 1)
void MMPF_USB_SendPtpEvent(MMP_USHORT eventCode)
{
    switch(eventCode){
        case PTP_EC_CANCEL_TRANSACTION:
        case PTP_EC_DEV_INFO_CHANGED:
        case PTP_EC_DEV_RESET:
        case PTP_EC_UNREPORTED_STATUS:
            DPSEventTx(eventCode, 0, 0, 0, 0);
        break;

        case PTP_EC_OBJECT_ADDED:
        case PTP_EC_OBJECT_REMOVED:
        case PTP_EC_OBJECT_INFO_CHANGED:
        case PTP_EC_REQUEST_OBJECT_TRANSFER:
            DPSEventTx(eventCode, 1, CurObjInfo.ObjectHandle, 0, 0);
        break;

        case PTP_EC_StoreAdded:
        case PTP_EC_StoreRemoved:
        case PTP_EC_STORE_FULL:
        case PTP_EC_STORAGE_INFO_CHANGED:
            DPSEventTx(eventCode, 1, CurObjInfo.StorageID, 0, 0);
        break;
        case PTP_EC_DEV_PRPTP_OC_CHANGED:
            DPSEventTx(eventCode, 1, curOPinfo.Para[0], 0, 0);
        break;

        case PTP_EC_CAPTURE_COMPLETE:
            DPSEventTx(eventCode, 1, curOPinfo.TransactionID, 0, 0);
        break;
        default:
                RTNA_DBG_Str(0, "ERROR: unsupport event code\r\n");
        break;
    }
}
void MMPF_USB_SendPtpResp(MMP_USHORT opCode)
{
    MtpResponsePhase();
}
//==============================================================================
//  Function    : MMPF_USB_TriggerPtpTxTransfer
//==============================================================================
/**
    @brief
    @return
    @retval
*/
MMP_ERR MMPF_USB_TriggerPtpTxTransfer(MMP_ULONG bufAddr, MMP_ULONG dataLen)
{
    MMPF_OS_FLAGS   flags;

    MTPBufHandle.RxBufAddr = bufAddr;
    m_ubPtpProcess = mtpbulkingoing;
    mtp_bulk_in(dataLen);

    if (MMPF_OS_WaitFlags(MTP_Flag, MTP_FLAG_BULKOUT_DONE|MTP_FLAG_CANCELREQUEST, MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME, 5000, &flags)) {
        m_ubPtpProcess = mtpbulkindone;
        return MMP_ERR_NONE;
    }
    return -1;
}

//==============================================================================
//  Function    : MMPF_USB_TriggerPtpRxTransfer
//==============================================================================
/**
    @brief
    @return
    @retval
*/
MMP_ERR MMPF_USB_TriggerPtpRxTransfer(MMP_ULONG bufAddr, MMP_ULONG dataLen)
{
    MMP_UBYTE ret;
    MMPF_OS_FLAGS   flags;

    if(m_ubPtpProcess == mtpcancelrequest)         //Only Bulk Out need to check
        ret = 1;
    m_ubPtpProcess = mtpbulkoutgoing;
    mtp_bulk_out(dataLen);

    if (MMPF_OS_WaitFlags(MTP_Flag, MTP_FLAG_BULKOUT_DONE|MTP_FLAG_CANCELREQUEST, MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME, 5000, &flags)) {
        m_ubPtpProcess = mtpbulkoutdone;
        return MMP_ERR_NONE;
    }
    return ret;
}

//==============================================================================
//  Function    : MMPF_USB_TriggerPtpEventTransfer
//==============================================================================
/**
    @brief
    @return
    @retval
*/
MMP_ERR MMPF_USB_TriggerPtpEventTransfer(MMP_ULONG bufAddr, MMP_ULONG dataLen)
{
    MMP_ULONG i;
    MMPF_OS_FLAGS   flags;
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    m_ubPtpProcess = mtpbulkingoing;

    for ( i=0; i<dataLen; i++ )
        pUSB_CTL->USB_FIFO_EP[0x3].FIFO_B = *(MMP_UBYTE *)bufAddr + i;

    mtp_interrupt_in(16);

    if (MMPF_OS_WaitFlags(MTP_Flag, MTP_FLAG_EVENT_DONE, MMPF_OS_FLAG_WAIT_SET_ANY|MMPF_OS_FLAG_CONSUME, 5000, &flags)){
        m_ubPtpProcess = mtpbulkindone;
        return MMP_ERR_NONE;
    }

    return -1;
}

void    MMPF_USB_StallPtpRx(void)
{
    MMP_USHORT csr;

    gbIsStalled = MMP_TRUE;
    csr = UsbReadRxEpCSR(MTP_RX_EP_ADDR);
    csr = csr & PHL_EP_RX_CSR_MASK;
    UsbWriteRxEpCSR(MTP_RX_EP_ADDR, csr|PHL_EP_RX_SET_STALL);
    gsUsbRxEpStatus[MTP_RX_EP_ADDR] |= SET_HALT_FEATURE;

}


void    MMPF_USB_StallPtpTx(void)
{
    MMP_USHORT csr;

    gbIsStalled = MMP_TRUE;
    csr = UsbReadTxEpCSR(MTP_TX_EP_ADDR);
    csr = csr & PHL_EP_TX_CSR_MASK;
    UsbWriteTxEpCSR(MTP_TX_EP_ADDR, csr|PHL_EP_TX_SET_STALL);
    gsUsbTxEpStatus[MTP_TX_EP_ADDR] |= SET_HALT_FEATURE;

}

void MMPF_USB_DeletePtpObjByHandle(MMP_ULONG ulHandler)
{
    MMP_ULONG status;
    DelObjByObjHdl(ulHandler, &status);
    if (status)
        RTNA_DBG_Str(0, "MTP ERROR:delete fail\r\n");
}

MMP_USHORT MMPF_USB_GetPtpProtStatus(MMP_ULONG ulHandler)
{
    return GetProptectStatusByHandle(ulHandler);
}

void MMPF_USB_SetPtpProtStatus(MMP_ULONG ulHandler, MMP_USHORT protFlag)
{
    SetProptectStatusByHandle(ulHandler, protFlag);
}

MMP_USHORT MMPF_USB_GetPtpParentHandle(MMP_ULONG ulHandle)
{
    MMP_USHORT *ptHandle;

    GetParentHandle(ulHandle, ptHandle);
    return *ptHandle;
}

#endif
#endif
#if defined(UPDATER_FW)
void MMPF_USB_StopUSB(void)
{
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
}
#endif

#if USB_SUSPEND_TEST&&(SUPPORT_UVC_FUNC == 1)
void USB_SuspendProcess(void)
{
    MMP_BOOL    ispreview ;
    #if (CHIP == P_V2)
    MMP_SHORT   timeout = 500;
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    #endif

    MMPS_3GPRECD_GetPreviewStatus(&ispreview) ;
    if(ispreview==1){
        MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_STOP_PREVIEW, MMPF_OS_FLAG_SET);
        MMPF_OS_Sleep(2);
        #if (CHIP == P_V2)
        do {
            if (pGBL->GBL_I2C_SEL_CTL == I2C_BY_I2CM) {
                MMPF_OS_Sleep(1);
            } else {
                break ;
            }
            timeout--;
        } while ( timeout > 0);
        #endif
    }
}
#endif

MMP_ERR MMPF_USB_ProcessCmd(void)
{
    MMP_ULONG   ulParameter[MAX_HIF_ARRAY_SIZE], i;
    MMP_USHORT  usCommand;

    usCommand = m_ulHifCmd[GRP_IDX_USB];    

	if ((usCommand & GRP_MASK) != GRP_USB)
       return MMP_ERR_NONE;

	for (i = 0; i < MAX_HIF_ARRAY_SIZE; i ++)
        ulParameter[i] = m_ulHifParam[GRP_IDX_USB][i];
    
	m_ulHifCmd[GRP_IDX_USB] = 0;

    switch (usCommand & (GRP_MASK|FUNC_MASK)) {
    #if (USE_SEC_MTP_DEV_FW == 0)&&(SUPPORT_MTP_FUNC == 1)
    case HIF_CMD_USB_PTP_CMD:
        switch (usCommand & SUB_MASK) {
        case HIF_PTP_TRIGGER_TX:
            MMPF_USB_TriggerPtpTxTransfer(ulParameter[0], ulParameter[1]);
            break;
        case HIF_PTP_TRIGGER_RX:
            MMPF_USB_TriggerPtpRxTransfer(ulParameter[0], ulParameter[1]);
            break;
        case HIF_PTP_TRIGGER_EVENT:
            MMPF_USB_TriggerPtpEventTransfer(ulParameter[0], ulParameter[1]);
            break;
        case HIF_PTP_STALL:
            MMPF_USB_StallPtpRx();
            MMPF_USB_StallPtpTx();
            break;
        case HIF_PTP_DEL_BY_HANDLE:
            MMPF_USB_DeletePtpObjByHandle(ulParameter[0]);
            break;
        case HIF_PTP_GET_PROTSTATUS:
            MMPF_HIF_FeedbackParamW(GRP_IDX_USB, 0, MMPF_USB_GetPtpProtStatus(ulParameter[0]));
            break;
        case HIF_PTP_SET_PROTSTATUS:
            MMPF_USB_SetPtpProtStatus(ulParameter[0], ulParameter[1]);
            break;
        case HIF_PTP_GET_PARENTHANDLE:
            MMPF_HIF_FeedbackParamW(GRP_IDX_USB, 0, MMPF_USB_GetPtpParentHandle(ulParameter[0]));
            break;
        }
        break;
    #endif
    case HIF_USB_CMD_GET_RW_FLAG:
        switch (usCommand & SUB_MASK){
        case HIF_USB_GET_W_FLAG:
            MMPF_HIF_FeedbackParamB(GRP_IDX_USB, 0, m_MSDC_WriteFlag);
            break;
        case HIF_USB_GET_R_FLAG:
            MMPF_HIF_FeedbackParamB(GRP_IDX_USB, 0, m_MSDC_ReadFlag);
            break;
        }
        break;
    case HIF_USB_CMD_GET_MODE:
        MMPF_HIF_FeedbackParamB(GRP_IDX_USB, 0, glUsbApplicationMode);
        break;
    case HIF_USB_CMD_STOPDEVICE:
        MMPF_USB_StopDevice();
        break;
    case HIF_USB_CMD_CONNECTDEVICE:
        MMPF_USB_ConnectDevice();
        break;
    case HIF_USB_CMD_SET_MODE:
        MMPF_SetUSBChangeMode(usCommand >> 8);
        break;
    case HIF_USB_CMD_SET_MSDCBUF:
        #if (MSDC_WR_FLOW_TEST_EN == 1)
        if ((usCommand & SUB_MASK) == SET_MSDC_W_BUFF_QUEUE)
            MMPF_SetMSDCWriteBuffQueue(ulParameter[0], ulParameter[1]);
        else
        #endif
		#if  (SUPPORT_MSDC_SCSI_AIT_CMD_MODE == 1)  
        if ((usCommand & SUB_MASK) == SET_DBG_BUF) {
        	extern void MMPF_AIT_DBG_SetBuffer(MMP_ULONG ulbuf_addr, MMP_ULONG ulbuf_size);
            MMPF_AIT_DBG_SetBuffer(ulParameter[0], ulParameter[1]);
        }
        else
		#endif
            MMPF_SetMSDCBuf(ulParameter[0], ulParameter[1]);
        break;
    case HIF_USB_CMD_SET_MEMDEV_BUF:
        #if (USING_MEMORY_IF == 1)
        MMPF_MemDev_Open(ulParameter[0], ulParameter[1]);
        #endif
        break;
    case HIF_USB_CMD_SET_CONTROLBUF:
        MMPF_SetUsbCtlBuf(ulParameter[0]);
        break;
    case HIF_USB_CMD_SET_PCSYNCBUF:
        #if (SUPPORT_PCSYNC)
        switch (usCommand & SUB_MASK) {
        case SET_PCSYNC_IN_BUF:
            MMPF_SetPCSYNCInBuf(ulParameter[0]);
            break;
        case SET_PCSYNC_OUT_BUF:
            MMPF_SetPCSYNCOutBuf(ulParameter[0]);
            break;
        case SET_PCSYNC_HANDSHAKE_BUF:
            MMPF_SetPCSYNCHandShakeBuf(ulParameter[0]);
            break;
        }
        #endif
        break;
    case HIF_USB_CMD_SET_PCCAMBUF:
        #if (SUPPORT_PCCAM_FUNC==1)
        switch (usCommand & SUB_MASK) {
        case SET_PCCAM_COMPRESS_BUF:
            MMPF_SetPCCAMCompressBuf(ulParameter[0], ulParameter[1]);
            break;
        case SET_PCCAM_LINE_BUF:
            MMPF_SetPCCAMLineBuf(ulParameter[0]);
            break;
        }
        #endif
        break;
    case HIF_USB_CMD_SET_MTPBUF:
        switch (usCommand & SUB_MASK) {
        case SET_MTP_EP_BUF:
            #if (SUPPORT_MTP_FUNC==1)||(SUPPORT_DPS_FUNC==1)
            RTNA_DBG_Str3("SET_MTP_EP_BUF:");
            MMPF_SetMTPEPBuf(ulParameter[0], ulParameter[1], ulParameter[2],
                             ulParameter[3], ulParameter[4]);
            MtpInitInMainON=1;
            #endif
            break;
        }
        break;
    #if (SUPPORT_DPS_FUNC==1)
    case HIF_USB_CMD_DPS: //0x78
        switch (usCommand & SUB_MASK) {
        case DPS_START_JOB:
            MMPF_StartDPSPrintJob();
            break;
        case DPS_ABORT_JOB:
            MMPF_AbortDPSPrintJob();
            break;
        case DPS_CONTINUE_JOB:
            MMPF_ContinueDPSPrintJob();
            break;
        case DPS_PRINTER_STATUS:
            MMPF_HIF_FeedbackParamW(GRP_IDX_USB, 0, MMPF_GetPrinterStatus());
            break;
        case DPS_GETBUFADDR:
            MMPF_HIF_FeedbackParamL(GRP_IDX_USB, 0, MMPF_DPSGetBufAddr(ulParameter[0]));
            break;
        }
        break;
    #endif
    case HIF_USB_CMD_PCCAM_CAPTURE:
        #if (SUPPORT_PCCAM_FUNC==1)
        MMPF_EnablePCCamCapture();
        #endif
        break;
    case HIF_USB_CMD_ADJUST_SIGNAL:
        MMPF_USBPHY_SetSignalTune(ulParameter[0], ulParameter[1]);
        break;
    case HIF_USB_CMD_SET_MSDC_UNIT:
        MMPF_MSDC_SetMaxUnits((MMP_UBYTE)ulParameter[0]);
        break;
    }

    return 0;
}

#ifdef BUILD_CE
#include "mmph_hif.h"
extern MMP_BOOL bWaitForUsbCommandDone;
extern MMPF_OS_FLAGID   DSC_UI_Flag;
#endif


#define DUMP_H264_BY_FS_IF   (0)
MMP_ULONG glCount;
MMP_BYTE gbPreCtlBusy = 0;
MMP_UBYTE gbDbgLogLcdEn = 0; //Bruce add dbglog
MMP_UBYTE gbDbgLogSdEn = 0; //Bruce add dbglog
MMP_USHORT gbWrCount = 0;

#if (DUMP_H264_BY_FS_IF)
#define DUMP_H264_FILE          "SD:\\0829_DumpH264_1920x1080_12.h264"
MMP_ULONG fpUvc;
MMP_ULONG resultUvc;
#endif

#if (MSDC_WR_FLOW_TEST_EN == 1)
void USB_Write_Task()
{
    MMP_UBYTE retStatus;
    MMP_USHORT fRet;
    MMP_ULONG i, j, loopSize;
    MMP_ULONG Qstart, Qend;
    MMP_BOOL isWrite;

    m_usbWriteSem = MMPF_OS_CreateSem(0);
    
    while(1) {
        retStatus = MMPF_OS_AcquireSem(m_usbWriteSem, 0);
        m_semCount--;

        if (retStatus == OS_NO_ERR) {
            if (m_bIsHostCancelMSDC == MMP_TRUE){
                for ( i=0; i<=MMPF_USB_W_DATA_BUFF_DEPTH; i++ ){
                    m_stUSB_MSDC_WBUFF[i].isValid = 0;
                }
                m_ulMSDC_CurRead_Point = 0;
                m_ulMSDC_CurWrite_Point = 0;
                        m_bMSDCWriteSuspend = MMP_FALSE;
                        m_bMSDCSmallSizeSect = MMP_FALSE;
            }
            else {
                if (( m_bMSDCSmallSizeSect == MMP_TRUE )&&(m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].isValid == 1)) {
                    m_bMSDCSmallSizeSect = MMP_FALSE;
                    fRet = gpMSDC_Device[m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].ulMsdcLun].device->pfWriteBurst(gpMSDC_Device[m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].ulMsdcLun].unit,
                                (MMP_ULONG)m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].startSec,
                                m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].secSize,
                                (void *)m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].dataBuf);
                    if (fRet) {
                        DBG_S0("Storage write error\r\n");
                        gsCardExist[glMSDCLun] &= ~CARD_EXIST;
                        MMPF_MSDC_SetSenseData(CUR_ERR, NOT_READY, 0, ASC_MEDIA_NOT_PRESENT, 0, 0, 0);
                        MMPF_MSDC_StallRx();
                        MMPF_MSDC_SetCswHeader(1);
                        MMPF_MSDC_TriggerUsbTX(glMSDCCmdBufAddr, 0x0d);
                        return;
                    }
                    else {
                        //return CSW
                        m_stUSB_MSDC_WBUFF[MMPF_USB_W_DATA_BUFF_DEPTH].isValid = 0;
                        m_MSDC_WriteFlag = 0;
                        glUsbMSDCState = MSDC_CBW_STATE;
                        MMPF_MSDC_SetCswHeader(0);
                        MMPF_MSDC_TriggerUsbTX(glMSDCCmdBufAddr, 0x0d);
                    }
                }
                else if (m_stUSB_MSDC_WBUFF[m_ulMSDC_CurRead_Point].isValid == 1) {

                    //get read/write point;
                    isWrite = MMP_FALSE;
                    Qstart = m_ulMSDC_CurRead_Point;
                    Qend = m_ulMSDC_CurWrite_Point;
                    if ( Qend == 0) {
                        Qend = MMPF_USB_W_DATA_BUFF_DEPTH-1;
                    }
                    else Qend--;

                    //RTNA_DBG_PrintLong(0, Qstart);
                    //RTNA_DBG_PrintLong(0, Qend);
                    //calculate read/write point difference
                    if ( Qend < Qstart )
                        loopSize = Qend + MMPF_USB_W_DATA_BUFF_DEPTH - Qstart + 1;
                    else loopSize = Qend - Qstart + 1;

                    if (loopSize > USB_MSDC_Q_THRESHOLD)
                        loopSize = USB_MSDC_Q_THRESHOLD;

                    //RTNA_DBG_PrintLong(0, loopSize);
                    Qend = Qstart;

                    if (loopSize == 1) {
                        fRet = gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].device->pfWriteBurst(gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].unit,
                                (MMP_ULONG)m_stUSB_MSDC_WBUFF[Qstart].startSec,
                                0x80, (void *)m_stUSB_MSDC_WBUFF[Qstart].dataBuf);
                        if (!fRet) {
                            isWrite = MMP_TRUE;
                            m_stUSB_MSDC_WBUFF[Qstart].isValid = 0;
                            m_ulMSDC_CurRead_Point = Qend+1;
                            if (m_ulMSDC_CurRead_Point >= MMPF_USB_W_DATA_BUFF_DEPTH)
                                m_ulMSDC_CurRead_Point = 0;
                        }
                        else {
                            RTNA_DBG_Str(0, "ERROR: write sector error_0\r\n");
                            return;
                        }
                    }
                    else {
                        //start write sectors
                        for ( i=1; i<loopSize; i++ ) {
                            if ( (Qend+1) == MMPF_USB_W_DATA_BUFF_DEPTH ) {
                                fRet = gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].device->pfWriteBurst(gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].unit,
                                        (MMP_ULONG)m_stUSB_MSDC_WBUFF[Qstart].startSec,
                                        (Qend-Qstart+1)*0x80, (void *)m_stUSB_MSDC_WBUFF[Qstart].dataBuf);
                                if (!fRet) {
                                    isWrite = MMP_TRUE;

                                    if (Qend == Qstart){
                                        m_stUSB_MSDC_WBUFF[Qstart].isValid = 0;
                                    }
                                    else {
                                        for ( j=Qstart; j<Qend; j++ ) {

                                            MMPF_OS_AcquireSem(m_usbWriteSem, 0);
                                            m_semCount--;
                                            m_stUSB_MSDC_WBUFF[j].isValid = 0;
                                        }
                                        m_stUSB_MSDC_WBUFF[j].isValid = 0;
                                    }
                                    i+= loopSize; //exit for loop
                                    m_ulMSDC_CurRead_Point = 0; //Qend+1;
                                }
                                else {
                                    RTNA_DBG_Str(0, "ERROR: write sector error_1\r\n");
                                    return;
                                }
                            }
                            //else if ( (m_stUSB_MSDC_WBUFF[Qstart].startSec + m_stUSB_MSDC_WBUFF[Qstart].secSize) == m_stUSB_MSDC_WBUFF[Qstart+1].startSec ){
                            else if (((m_stUSB_MSDC_WBUFF[Qend].startSec + m_stUSB_MSDC_WBUFF[Qend].secSize) == m_stUSB_MSDC_WBUFF[Qend+1].startSec )&&
                            (m_stUSB_MSDC_WBUFF[Qend].ulMsdcLun == m_stUSB_MSDC_WBUFF[Qend+1].ulMsdcLun)){
                                Qend++;
                            }
                            else {
                                fRet = gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].device->pfWriteBurst(gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].unit,
                                        (MMP_ULONG)m_stUSB_MSDC_WBUFF[Qstart].startSec,
                                        (Qend-Qstart+1)*0x80, (void *)m_stUSB_MSDC_WBUFF[Qstart].dataBuf);
                                if (!fRet) {
                                    isWrite = MMP_TRUE;

                                    if (Qend == Qstart){
                                        m_stUSB_MSDC_WBUFF[Qstart].isValid = 0;
                                    }
                                    else {
                                        for ( j=Qstart; j<Qend; j++ ) {

                                            MMPF_OS_AcquireSem(m_usbWriteSem, 0);
                                            m_semCount--;
                                            m_stUSB_MSDC_WBUFF[j].isValid = 0;
                                        }
                                        m_stUSB_MSDC_WBUFF[j].isValid = 0;
                                    }
                                    i+= loopSize; //exit for loop
                                    m_ulMSDC_CurRead_Point = Qend+1;
                                    if (m_ulMSDC_CurRead_Point >= MMPF_USB_W_DATA_BUFF_DEPTH)
                                        m_ulMSDC_CurRead_Point = 0;
                                }
                                else {
                                    RTNA_DBG_Str(0, "ERROR: write sector error_2\r\n");
                                    return;
                                }
                            }
                        }
                    }
                    if (!isWrite) {
                        fRet = gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].device->pfWriteBurst(gpMSDC_Device[m_stUSB_MSDC_WBUFF[Qstart].ulMsdcLun].unit,
                                (MMP_ULONG)m_stUSB_MSDC_WBUFF[Qstart].startSec,
                                (Qend-Qstart+1)*0x80, (void *)m_stUSB_MSDC_WBUFF[Qstart].dataBuf);
                        if (!fRet) {

                            for ( j=Qstart; j<Qend; j++ ) {

                                MMPF_OS_AcquireSem(m_usbWriteSem, 0);
                                m_semCount--;
                                m_stUSB_MSDC_WBUFF[j].isValid = 0;
                            }
                            m_stUSB_MSDC_WBUFF[j].isValid = 0;

                            m_ulMSDC_CurRead_Point = Qend+1;
                            if (m_ulMSDC_CurRead_Point >= MMPF_USB_W_DATA_BUFF_DEPTH)
                                m_ulMSDC_CurRead_Point = 0;
                        }
                        else {
                            DBG_S0("Storage write error\r\n");

                            gsCardExist[glMSDCLun] &= ~CARD_EXIST;
                            MMPF_MSDC_SetSenseData(CUR_ERR, NOT_READY, 0, ASC_MEDIA_NOT_PRESENT, 0, 0, 0);
                            MMPF_MSDC_StallRx();
                            MMPF_MSDC_SetCswHeader(1);
                            MMPF_MSDC_TriggerUsbTX(glMSDCCmdBufAddr, 0x0d);
                            return;
                        }

                    }
                    if ( m_bMSDCWriteSuspend == MMP_TRUE ) {

                        if (m_bMSDCSmallSizeSect)
                            RTNA_DBG_Str(0, "small and suspend\r\n");
                        m_bMSDCWriteSuspend = MMP_FALSE;
                        MMPF_MSDC_HandleWriteSuspend();
                    }
                }
            } //end of acquireSem no error
        }// m_bIsHostCancelMSDC == MMP_FALSE;
        else {//acquire sem error
            RTNA_DBG_Str(0, "ERROR:SEM\r\n");
            return; //add error handler here
        }
    }
}
#endif

void USB_Task(void)
{
    MMPF_OS_FLAGS flags;
    #if (CHIP == P_V2)||(SUPPORT_UVC_FUNC == 1)
    #if (CHIP == P_V2)
    AITPS_GBL   pGBL  = AITC_BASE_GBL;
    #endif
    #endif
    #if (CHIP == P_V2)
    MMP_ULONG   ulUsbG0Clk;
    #endif
    #if (SUPPORT_UVC_FUNC == 1)
    MMP_BOOL    IsPreviewEnable;
    AITPS_IBC   pIBC = AITC_BASE_IBC;
    AITPS_AIC   pAIC = AITC_BASE_AIC;
    AITPS_JPG   pJPG = AITC_BASE_JPG;
    AITPS_IBCP  pIBCP0 = &pIBC->IBCP_0;
    AITPS_IBCP  pIBCP1 = &pIBC->IBCP_1;

    extern MMP_UBYTE gbStopPreviewEvent;
    extern MMP_UBYTE glAudioEnable;
    #endif

    #if (CHIP == P_V2)
    //wilson@110505: MCI clk : Controller clk = 2:1 or 1:1
    //and controller clk need > 30MHz; need further modify
    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulUsbG0Clk);
    if (ulUsbG0Clk > 0x64) {
        pGBL->GBL_CLK_0_JPG_DIV &= 0x07; //mask JPEG setting and clear default controller divide value
        pGBL->GBL_CLK_0_JPG_DIV |= GBL_CLK_USB_CTL_DIV(0x02);
    }
    #endif

    RTNA_DBG_Str(0, "USB_Task()\r\n");

    //creat sem here; wilson@110422
    m_usbReadSem = MMPF_OS_CreateSem(0);

    while(1){
        #if (SUPPORT_DPS_FUNC==1)
        MMPF_OS_WaitFlags(USB_OP_Flag, USB_FLAG_GENOP | USB_FLAG_DSC, 
                MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME , 0, &flags);
        #elif (SUPPORT_UVC_FUNC == 1)
        MMPF_OS_WaitFlags(USB_OP_Flag,
                USB_FLAG_CLSOP | USB_FLAG_GENOP | USB_FLAG_UVC |
                USB_FLAG_UVC_START_PREVIEW | USB_FLAG_UVC_STOP_PREVIEW,
                MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME , 0, &flags);
        #else
        MMPF_OS_WaitFlags(USB_OP_Flag, USB_FLAG_GENOP | USB_FLAG_CLSOP,
                MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME , 0, &flags);
        #endif

        if (flags & USB_FLAG_GENOP) {
            MMPF_USB_ProcessCmd();
        }

        if (flags & USB_FLAG_CLSOP) {
            #if (SUPPORT_PCCAM_FUNC)
            if (glUsbApplicationMode == USB_PCCAM_MODE) {
                MMPF_PollingPCCam();
                MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_CLSOP, MMPF_OS_FLAG_SET);
            }
            #endif

            if ((glUsbApplicationMode == USB_MSDC_MODE) 
#if (SUPPORT_MSDC_SCSI_AIT_CMD_MODE == 1)
			|| (glUsbApplicationMode == USB_MSDC_AIT_DEBUG_MODE)
#endif			
			) {
                MMPF_MSDC_Polling();
            }

            #if (SUPPORT_MTP_FUNC)||(SUPPORT_DPS_FUNC)
            #if (USE_SEC_MTP_DEV_FW == 0)
            if (glUsbApplicationMode == USB_MTP_MODE) {
                /*
                    MTP Task
                */
                if(MtpInitInMainON==1){
                   MTPInit();
                   MtpInitInMainON=0;
                }
                if (MTPprocess == mtpbulkoutdone) {
                    MTPprocess = none_prs;
                    mtpbulkoutdone_process();
                }
                else if (MTPprocess == mtpbulkindone) {
                    MTPprocess = none_prs;
                    mtpbulkindone_process();
                }
                else if (MTPprocess == mtpINTindone) {
                    MTPprocess = none_prs;
                    mtpinterruptindone_process();
                }
                else if (MTPprocess == mtpcancelrequest) {
                    MTPprocess = none_prs;
                    mtpcancelrequest_process();
                }
            }
            #endif
            #endif
        }

        #if (SUPPORT_DPS_FUNC == 1)
        if (flags & USB_FLAG_DSC) {
            MMPF_DSC_ProcessCmd();
        }
        #endif

        #ifdef BUILD_CE
        if (bWaitForUsbCommandDone) {
            MMPF_OS_SetFlags(DSC_UI_Flag, SYS_FLAG_USB_CMD_DONE, MMPF_OS_FLAG_SET);
        }
        #endif

        #if (SUPPORT_UVC_FUNC == 1)
        if (flags & USB_FLAG_UVC_START_PREVIEW)
        {
            RTNA_DBG_Str(0, "[DBG] Start Preview Flag get ...\r\n");
            MMPS_3GPRECD_GetPreviewStatus(&IsPreviewEnable);
            if ((IsPreviewEnable == MMP_FALSE) &&(gbStopPreviewEvent==0 ))
            {
                MMP_BOOL mjpeg_stream ;

                //MMPF_USB_ResetExtDMA(UVC_TX_EP_ADDR);
                mjpeg_stream = usb_uvc_config_stream();

                USB_VideoPreviewStart();
                uvc_init();
                MMPF_USB_ResetDMA();
                usb_uvc_init_stream(mjpeg_stream);
                usb_vc_send_image(0);

                gbDevicePowerSavingStatus = 0;
                #if (CHIP == P_V2)
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_VID, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SPI, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD, MMP_FALSE);
                #endif
                #if (CHIP == MCR_V2)
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD0, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD1, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD2, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_SD3, MMP_FALSE);
                MMPF_SYS_EnableClock(MMPF_SYS_CLK_BS_SPI, MMP_FALSE);
                
                #endif
                if (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_H264)
                {
                   //wilson: uvc; todo
                    #if SUPPORT_AUTO_FOCUS
                    //MMPF_UVCWRAP_AutoFocus();
                    #endif
                }
            }
            RTNA_DBG_Str(0, "[DBG] Start Preview End *****************\r\n");
        }

        if (flags & USB_FLAG_UVC_STOP_PREVIEW)
        {
            RTNA_DBG_Str(0, "SYS_FLAG_USB_STOP_PREVIEW\r\n");

            m_bIsVCStart = MMP_FALSE;
            m_VideoFmt = pcam_get_info()->pCamVideoFormat;

            SetH264Mode(0);//disable H264 mode

            if (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_MJPEG) {
                gbStopMJPEGEvent = 1;
                gbdrawflag = 0;
                m_bVidRecdPreviewStatus = 0;
            }
            else if ((m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_YUV422) ||
                (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_YUV420)) {
                RTNA_DBG_Str(3,"<Fmt : Stop.YUV>\r\n");

                if (gsYUY2Stream_Cfg.pipe_en & PIPE0_EN) {
                    pIBC->IBC_P0_INT_CPU_EN &= ~(IBC_INT_FRM_RDY);
                    while (!(pIBC->IBC_P0_INT_CPU_SR & IBC_INT_FRM_RDY));
                    pIBCP0->IBC_BUF_CFG &= ~(IBC_STORE_EN);
                }
                if (gsYUY2Stream_Cfg.pipe_en & PIPE1_EN) {
                    pIBC->IBC_P1_INT_CPU_EN &= ~(IBC_INT_FRM_RDY);
                    while(!(pIBC->IBC_P1_INT_CPU_SR & IBC_INT_FRM_RDY));
                    pIBCP1->IBC_BUF_CFG &= ~(IBC_STORE_EN);
                }

                gbStopYUV422Event = 1;
                gbdrawflag = 0;
                m_bVidRecdPreviewStatus = 0;
            }
            else {
                RTNA_DBG_Str(3,"<Fmt : Stop.H264>\r\n");
                MMPF_VIF_CheckFrameSig(MMPF_VIF_MDL_ID0, MMPF_VIF_INT_EVENT_FRM_ST, 2);  // wait 2 frame for H264 encode end

                USB_VideoPreviewStop();
                gbdrawflag = 0;
            }
            USB_VideoPowerDown();

            if (gbStillCaptureEvent == STILL_IMAGE_PREVIEW_STOP) {
                MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC, MMPF_OS_FLAG_SET);
            }
            MMPF_USB_ResetExtDMA(UVC_TX_EP_ADDR);
        }
        /////////////////////////////////////////////////////////////

        if (flags & USB_FLAG_UVC) {
            if ((gbdrawflag == 1) && (m_bIsVCStart == 1) &&
                (gbStillCaptureEvent==STILL_IMAGE_WAITING)) {
                gbdrawflag = 0;
                if (usb_vc_send_image(0)== UVC_SEND_IMG_RET_END_FRAME) {
                    //RTNA_DBG_Str3("END.FRAME\r\n");
                    MMPF_Video_UpdateRdPtrByPayloadLength();
                }
            }
            else if (gbStillCaptureEvent == STILL_IMAGE_TRIGGER) {
                if (glAudioEnable) {
                    RTNA_AIC_IRQ_Dis(pAIC, AIC_SRC_AFE_FIFO);
                }
                if (usb_vc_send_image(0) == UVC_SEND_IMG_RET_END_FRAME) {
                    while(gbdrawflag == 0);  // wait USB DMA done for last frame
                    RTNA_WAIT_US(500);

                    if (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_H264) {
                        MMPS_3GPRECD_StopRecord();
                        RTNA_DBG_Str0("MMPS_3GPRECD_StopRecord\r\n");
                    }

                    SetH264Mode(0);//disable H264 mode
                    gbStillCaptureEvent = STILL_IMAGE_PREVIEW_STOP;
                    RTNA_DBG_Str(0, "Still capture: STILL_IMAGE_TRIGGER\r\n");
                }
            }
            else if (gbStillCaptureEvent == STILL_IMAGE_PREVIEW_STOP) {
                gbStillCaptureEvent = STILL_IMAGE_SENDING;

                // store original format/resolution for video capture
                gbOrgVideoFormat = m_VideoFmt;
                gbOrgVideoResolution = glPccamResolution;
                RTNA_DBG_Str(0, " ** gbOrgVideoFormat = ");
                RTNA_DBG_Byte(0, gbOrgVideoFormat);
                RTNA_DBG_Str(0, " **\r\n");
                RTNA_DBG_Str(0, " ** gbOrgVideoResolution = ");
                RTNA_DBG_Byte(0, gbOrgVideoResolution);
                RTNA_DBG_Str(0, " **\r\n");

                // set new format/resolution for still capture
                usb_vc_set_still_resolution(sc.bFormatIndex, sc.bFrameIndex);

                if (gbOrgVideoFormat == MMPS_3GPRECD_VIDEO_FORMAT_H264) {
                    // re-initialize pipe engine for JPEG or YUV still capture
                    // if video format is H.264
                    USB_VideoPreviewStart();
                }

                #if CAPTURE_BAYER_RAW_ENABLE
                gbCaptureBayerRawFlag = 1;
                #endif

                uvc_init();
                RTNA_DBG_Str0("  ** Capture still image begin\r\n");

                if (gbCaptureBayerRawFlag) {
                    #if (SUPPORT_PCCAM_FUNC == MMP_TRUE)
                    glPCCamJpegSize = usb_vc_take_raw_picture(0, glPCCamCompressBufAddr, glPCCAM_VIDEO_BUF_ADDR);
                    #endif
                    usb_vc_send_image(1);
                }
                else {
                    usb_vc_send_image(1);
                }

                RTNA_DBG_Str3("  ** Capture still image  end\r\n");

                #if (CAPTURE_BAYER_RAW_ENABLE)
                gbCaptureBayerRawFlag = 0;
                #else
                #if (ENABLE_JPEG_ISR)
                if (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_MJPEG) {
                    while(pJPG->JPG_CTL & JPG_ENC_EN);
                }
                #endif
                #if (ENABLE_YUV_ISR)
                if ((m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_YUV422) ||
                    (m_VideoFmt == MMPS_3GPRECD_VIDEO_FORMAT_YUV420)) {
                    if (gsYUY2Stream_Cfg.pipe_en & PIPE0_EN) {
                        while(!(pIBC->IBC_P0_INT_CPU_SR & IBC_INT_FRM_RDY));
                    }
                    if (gsYUY2Stream_Cfg.pipe_en & PIPE1_EN) {
                        while(!(pIBC->IBC_P1_INT_CPU_SR & IBC_INT_FRM_RDY));
                    }
                }
                #endif
                #endif

                RTNA_DBG_Str(0, "Still capture: STILL_IMAGE_PREVIEW_STOP\r\n");
            }
            else if (gbStillCaptureEvent == STILL_IMAGE_SENDING) {

                if (usb_vc_send_image(1) == UVC_SEND_IMG_RET_END_FRAME) {
                    RTNA_DBG_Str(0, "  @@ STILL_IMAGE_SENDING finished @@\r\n");
                    gbStillCaptureEvent = STILL_IMAGE_FINISH;
                    gbStopPreviewEvent = 1;

                    #if (ENABLE_JPEG_ISR) || (ENABLE_YUV_ISR)
                    MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC, MMPF_OS_FLAG_SET);
                    #endif
                }
            }
            else if (gbStillCaptureEvent == STILL_IMAGE_FINISH) {
                gbStillCaptureEvent = STILL_IMAGE_WAITING;

                // restore original format/resolution for video capture
                m_VideoFmt = gbOrgVideoFormat;
                glPccamResolution = gbOrgVideoResolution;
                usb_vc_capture_trigger(0);

                if (glAudioEnable) {
                    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_AFE_FIFO);
                }
                RTNA_DBG_Str(0, "Still capture: STILL_IMAGE_FINISH\r\n");
            }

            if (gbUVCDSCCommand) {
                switch(gbUVCDSCCommand) {
                default:
                    break;
                }
                gbUVCDSCCommand = 0;
            }

            #if USB_SUSPEND_TEST&&(SUPPORT_UVC_FUNC == 1)
            if (gbUSBSuspendEvent) {
                gbUSBSuspendEvent = 0;
                if (gbUSBSuspendFlag == 0) {
                    RTNA_DBG_Str(0, "---- gbUSBSuspendEvent ----\r\n");
                    gbUSBSuspendFlag = 1;
                    USB_SuspendProcess();
                }
            }
            #endif
        }
        #endif
    }
}

#if (SUPPORT_UVC_FUNC)
#if (UVC_HOST_TEST)
void USB_UVCH_Preview_Ctl(void *argu)
{
    MMP_ULONG ulCount = *(MMP_ULONG *)argu;
    LCD_BUF_CTL    *lcd_bctl = &lcd_buf_ctl;
//    AITPS_DSPY pDSPY 		= AITC_BASE_DSPY;

    //Bruce add dbglog
    if(gbDbgLogLcdEn)
    {
        RTNA_DBG_Str(0, "4_"); 
        RTNA_DBG_Long(0, ulCount); 
        //RTNA_DBG_Str(0, "_"); 
        //RTNA_DBG_Long(0, (MMP_ULONG)argu); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Byte(0, lcd_bctl->ubInBufIdx); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Long(0, lcd_bctl->ulInputPtr); 
        RTNA_DBG_Str(0, ".\r\n");
        gbDbgLogLcdEn = 0;
    }
    
    lcd_bctl->ulInputPtr += ulCount;
    lcd_bctl->ulDataSize[lcd_bctl->ubInBufIdx] = lcd_bctl->ulInputPtr;

    if( lcd_bctl->ulDataSize[lcd_bctl->ubInBufIdx] >= (DEVICE_TX_YUY2_FRAME_SIZE) )
//    if( lcd_bctl->ulDataSize[lcd_bctl->ubInBufIdx] >= (640*480*2) )
    {
        
        MMPF_Display_SetWinAddr(MMPF_DISPLAY_WIN_OVERLAY,
                                            lcd_bctl->ulLcdBufAddr[lcd_bctl->ubInBufIdx],
                                            lcd_bctl->ulLcdBufAddr[lcd_bctl->ubInBufIdx],
                                            lcd_bctl->ulLcdBufAddr[lcd_bctl->ubInBufIdx]);
        
        MMPF_Display_SetWinActive(MMPF_DISPLAY_WIN_OVERLAY, MMP_TRUE);
        //MMPS_Display_SetDisplayRefresh(MMPS_DISPLAY_PRM_CTL); //no need, host will refresh!

/*
        RTNA_DBG_Str(0, "T:");
        RTNA_DBG_Short(0, OSTimeGet());		            
        RTNA_DBG_Str(0, "\r\n");
*/
        lcd_bctl->ulInputPtr = 0;
    
        lcd_bctl->ubInBufIdx++;
        if (lcd_bctl->ubInBufIdx == (LCD_BUF_CNT-1))
            lcd_bctl->ubInBufIdx = 0;

        lcd_bctl->ulDataSize[lcd_bctl->ubInBufIdx] = lcd_bctl->ulInputPtr;

    }
/*
    if (MMPF_OS_ReleaseSem(m_UVCHRefreshSemID) != OS_NO_ERR) {
        RTNA_DBG_Str(0, "m_UVCHRefreshSemID OSSemPost: Fail \r\n");
    }
*/
    gbPreCtlBusy = 0;
}

void USB_UVCH_Ep2Lcd_BufCtl(void)
{
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
    LCD_BUF_CTL    *lcd_bctl = &lcd_buf_ctl;
    MMP_ULONG    ulSrcaddr, ulDstaddr, ulCount;
    MMP_ULONG    ubOffset;

    if(gbPreCtlBusy) {
        RTNA_DBG_Str(0, "PreCtlBusy!\r\n");
    }
    gbPreCtlBusy = 1;

    ulSrcaddr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]; // source address

    ubOffset = 0x00;

    if ( USB_UVCH_Chk_Payload_Header(0) ) {
        ubOffset += 0x0C; // header length
//        RTNA_DBG_Str(0, "==> Payload Header OK\r\n");

        if ( USB_UVCH_Check_AIT_Header(0, 0) ) {
            ubOffset += 0x20; // header length
//            RTNA_DBG_Str(0, "==> AIT Header OK\r\n");
        }

    }
    else{
        RTNA_DBG_Str(0, "Ep2Lcd ==> Payload Header Error\r\n");
    }

    ulSrcaddr += ubOffset; // start address
    ulCount = ep_bctl->ulContentSize[ep_bctl->ulWriteBufIdx] - ubOffset; // data-length to dma 
    ulDstaddr = (lcd_bctl->ulLcdBufAddr[lcd_bctl->ubInBufIdx]) + (lcd_bctl->ulInputPtr); // destination address

    glCount = ulCount;
    
    //Bruce add dbglog
    if(gbDbgLogLcdEn)
    {
        RTNA_DBG_Str(0, "1_"); 
        RTNA_DBG_Long(0, glCount); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Long(0, ulSrcaddr); 
        RTNA_DBG_Str(0, " =>"); 
        RTNA_DBG_Long(0, ulDstaddr); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Byte(0, ep_bctl->ulWriteBufIdx); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Byte(0, lcd_bctl->ubInBufIdx); 
        RTNA_DBG_Str(0, ".\r\n");
    }

    if(MMPF_DMA_MoveData(ulSrcaddr, ulDstaddr, ulCount, USB_UVCH_Preview_Ctl, &glCount, MMP_FALSE, NULL)) {
        RTNA_DBG_Str(0, "MMPF_UVCH_MoveData Fail\r\n"); // 
    }
/*
    if (MMPF_OS_AcquireSem(m_UVCHRefreshSemID, 10)) {
        RTNA_DBG_Str(0, "m_UVCHRefreshSemID OSSemPend: Fail\r\n");
    }
*/
}

void USB_UVCH_Record_Ctl(void *argu)
{
    UVCH_Rx_Param *UVCHRxArg = (UVCH_Rx_Param *)argu;
    SD_BUF_CTL    *sd_bctl = &sd_buf_ctl;

    sd_bctl->ulInputPtr += UVCHRxArg->ulDmaByte;
    //sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] = sd_bctl->ulInputPtr;
    sd_bctl->ulDataSize[sd_bctl->ubInBufCurWrIdx] += UVCHRxArg->ulDmaByte;

    if(gbDbgLogSdEn)
    {
        RTNA_DBG_Str(0, "[R"); 
        //RTNA_DBG_Long(0, UVCHRxArg->ulDmaByte); 
        //RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Byte(0, sd_bctl->ubInBufIdx); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Long(0, sd_bctl->ulInputPtr); 
        RTNA_DBG_Str(0, "_"); 
        RTNA_DBG_Long(0, sd_bctl->ulDataSize[sd_bctl->ubInBufIdx]); 
        RTNA_DBG_Str(0, ".\r\n");
    }

    if( sd_bctl->ulDataSize[sd_bctl->ubInBufCurWrIdx] >= (sd_bctl->ulSdBufSize) )
    {
        if(gbDbgLogSdEn)
        {
            RTNA_DBG_Str(0, "[SD_S_F!\r\n");
        }
        
        sd_bctl->ulDataSize[sd_bctl->ubInBufCurWrIdx + 1] = sd_bctl->ulDataSize[sd_bctl->ubInBufCurWrIdx] - sd_bctl->ulSdBufSize;
        sd_bctl->ulDataSize[sd_bctl->ubInBufCurWrIdx] = sd_bctl->ulSdBufSize;
        sd_bctl->ubInBufCurWrIdx++;
        
        if (sd_bctl->ubInBufCurWrIdx >= SD_BUF_CNT) {
            RTNA_DBG_Str(0, " ![OUT OF SD_BUF!\r\n");
        }
    }
    
    #if 0
    while( sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] > (sd_bctl->ulSdBufSize) ) { // SD buffer slot full, 
        RTNA_DBG_Str(0, "[SD_S_F!\r\n");
        sd_bctl->ulDataSize[sd_bctl->ubInBufIdx + 1] = sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] - sd_bctl->ulSdBufSize;
        sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] = sd_bctl->ulSdBufSize;
        sd_bctl->ubInBufIdx++;
        
        if (sd_bctl->ubInBufIdx == SD_BUF_CNT) {
            RTNA_DBG_Str(0, " ![OUT OF SD_BUF!\r\n");
        }
    }
    #endif
    
//    if( sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] >= (640*480*2) )
//    if( sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] >= (UVCHRxArg->ulFrameSize) )
    if( sd_bctl->ulInputPtr >= (UVCHRxArg->ulFrameSize) )
    {
        if(0)//(gbDbgLogSdEn)
        {
            RTNA_DBG_Str(0, "R"); 
            RTNA_DBG_Long(0, sd_bctl->ulInputPtr); 
            RTNA_DBG_Str(0, ".\r\n");
        }

        /* keep for MMPS_3GPRECD_UVCRecdInputFrame */
        if (sd_bctl->ubFrameCnt >= SD_BUF_FRM_SLOT_CNT) {
            RTNA_DBG_Str(0, " ![OUT OF SD_OUT_BUF!\r\n");
        }
        sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotWr] = sd_bctl->ubInBufIdx;
        sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotWr] = sd_bctl->ulInputPtr;
        sd_bctl->ulFrameTimeStamp[sd_bctl->ubCurFrameSlotWr] = UVCHRxArg->ulTimeStamp;
        sd_bctl->ubFrameType[sd_bctl->ubCurFrameSlotWr] = UVCHRxArg->usFrameType;
        #if (0)
        {
            MMP_UBYTE nIdx;
            RTNA_DBG_Str(0, "[D");
            //RTNA_DBG_Byte(0, sd_bctl->ubCurFrameSlotWr);
            RTNA_DBG_Long(0, UVCHRxArg->ulFrameSize);
            RTNA_DBG_Str(0, ",");
            RTNA_DBG_Long(0, sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotWr]);
            //RTNA_DBG_Long(0, UVCHRxArg->ulTimeStamp);
            //RTNA_DBG_Str(0, ",");
            //RTNA_DBG_Byte(0, UVCHRxArg->usFrameType);
            //RTNA_DBG_Byte(0, sd_bctl->ubFrameCnt);
            //RTNA_DBG_Byte(0, sd_bctl->ubCurFrameSlotWr);
            //RTNA_DBG_Byte(0, sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotWr]);
            //RTNA_DBG_Long(0, (sd_bctl->ulSdBufAddr[sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotWr]]+0));
            //RTNA_DBG_Long(0, (sd_bctl->ulSdBufAddr[sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotWr]]+1));
            //RTNA_DBG_Str(0, ",");
            //for(nIdx=0; nIdx<12; nIdx++) {
            //    RTNA_DBG_Byte(0, (*(MMP_UBYTE *) (sd_bctl->ulSdBufAddr[sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotWr]]+nIdx)));
            //}
            RTNA_DBG_Str(0, "\r\n");
        }
        #endif

        sd_bctl->ubCurFrameSlotWr++;
        if (sd_bctl->ubCurFrameSlotWr >= SD_BUF_FRM_SLOT_CNT)
            sd_bctl->ubCurFrameSlotWr = 0;
        
        sd_bctl->ubFrameCnt++;
//        MMPS_3GPRECD_UVCRecdInputFrame(sd_bctl->ulSdBufAddr[sd_bctl->ubInBufIdx], UVCHRxArg->ulFrameSize, UVCHRxArg->ulTimeStamp, UVCHRxArg->usFrameType, 0);
//        MMPS_3GPRECD_UVCRecdInputFrame(sd_bctl->ulSdBufAddr[sd_bctl->ubInBufIdx], sd_bctl->ulDataSize[sd_bctl->ubInBufIdx], UVCHRxArg->ulTimeStamp, UVCHRxArg->usFrameType, 0);
        MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_INPUT_FRAME_REQ, MMPF_OS_FLAG_SET);      

/*
        RTNA_DBG_Str(0, "T:");
        RTNA_DBG_Short(0, OSTimeGet());		            
        RTNA_DBG_Str(0, "\r\n");
*/
        sd_bctl->ulInputPtr = 0;
        
        #if 1
        //sd_bctl->ubInBufIdx = ((sd_bctl->ubInBufCurWrIdx+5)/5)*5;
        sd_bctl->ubInBufIdx += SD_BUF_NUM_PER_FRAME;
        if (sd_bctl->ubInBufIdx >= SD_BUF_CNT)
            sd_bctl->ubInBufIdx = 0;
        sd_bctl->ubInBufCurWrIdx = sd_bctl->ubInBufIdx;
        #else
        sd_bctl->ubInBufIdx++;
        if (sd_bctl->ubInBufIdx == (SD_BUF_CNT-1))
            sd_bctl->ubInBufIdx = 0;
        #endif

        sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] = sd_bctl->ulInputPtr;

    }

}

void USB_UVCH_Ep2Sd_BufCtl(void)
{
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
    SD_BUF_CTL    *sd_bctl = &sd_buf_ctl;
    MMP_ULONG    ulSrcaddr, ulDstaddr, ulCount;
    MMP_ULONG    ubOffset;
    MMP_UBYTE    nRemoveSize;
    MMP_ULONG    nFillSize;
    
    ulSrcaddr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]; // source address

    ubOffset = 0x00;

    if ( USB_UVCH_Chk_Payload_Header(0) ) {
        ubOffset += 0x0C; // header length
//        RTNA_DBG_Str(0, "==> Payload Header OK\r\n");

        nRemoveSize = USB_UVCH_Check_H264_Header(0);
//        if ( USB_UVCH_Check_AIT_Header(0, 0) ) {
//        if ( USB_UVCH_Check_H264_Header(0) ) {
        if ( nRemoveSize ) {
            ubOffset += 0x20; // header length
//            RTNA_DBG_Str(0, "==> AIT Header OK\r\n");
            if(0)
            {
                RTNA_DBG_Str(0, "FS:");
                RTNA_DBG_Long(0, m_UVCHRxArg.ulFrameSize);
                RTNA_DBG_Str(0, ", FC:");
                RTNA_DBG_Short(0, uvch_xfer_ctl.ulFrameCnt);
                RTNA_DBG_Str(0, "\r\n");
            }
            
            #if (DUMP_H264_BY_FS_IF)
            //nop
            #else
            if(nRemoveSize > 0x04) {
                nFillSize = m_UVCHRxArg.ulFrameSize - nRemoveSize;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+1) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+2) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+3) = 0x0B;//SPS

                *(MMP_BYTE *) (ulSrcaddr+ubOffset+15) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+16) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+17) = 0x00;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+18) = 0x04;//PPS

                *(MMP_BYTE *) (ulSrcaddr+ubOffset+23) = (MMP_BYTE)((nFillSize>>24)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+24) = (MMP_BYTE)((nFillSize>>16)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+25) = (MMP_BYTE)((nFillSize>>8)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+26) = (MMP_BYTE)(nFillSize&0xFF);
            }
            else
            {
                nFillSize = m_UVCHRxArg.ulFrameSize - nRemoveSize;
                *(MMP_BYTE *) (ulSrcaddr+ubOffset) = (MMP_BYTE)((nFillSize>>24)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+1) = (MMP_BYTE)((nFillSize>>16)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+2) = (MMP_BYTE)((nFillSize>>8)&0xFF);
                *(MMP_BYTE *) (ulSrcaddr+ubOffset+3) = (MMP_BYTE)(nFillSize&0xFF);
            }
            #endif
        }

    }
    else {
        RTNA_DBG_Str(0, "Ep2Sd ==> Payload Header Error\r\n");
    }

    ulSrcaddr += ubOffset; // start address
    ulCount = ep_bctl->ulContentSize[ep_bctl->ulWriteBufIdx] - ubOffset; // data-length to dma 
    m_UVCHRxArg.ulDmaByte = ulCount; // data-length to dma 

#if 1
    ulDstaddr = (sd_bctl->ulSdBufAddr[sd_bctl->ubInBufIdx]) + (sd_bctl->ulInputPtr); // destination address

#else
    if ( ((sd_bctl->ulInputPtr)+ulCount) > (sd_bctl->ulSdBufSize) ) { // SD buffer slot full, 
        RTNA_DBG_Str(0, "sd_bctl->ulInputPtr > 0x10000!\r\n");
        
        sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] = sd_bctl->ulInputPtr;

        sd_bctl->ubInBufIdx++;
        if (sd_bctl->ubInBufIdx == (SD_BUF_CNT-1))
            sd_bctl->ubInBufIdx = 0;
/*
        RTNA_DBG_Str(0, "InBufIdx: ");
        RTNA_DBG_Byte(0, sd_bctl->ubInBufIdx);
        RTNA_DBG_Str(0, "\r\n");
*/
//        sd_bctl->ubFrameCnt++;
        sd_bctl->ulInputPtr = 0;

        ulDstaddr = sd_bctl->ulSdBufAddr[sd_bctl->ubInBufIdx];
    }
    else {
        ulDstaddr = (sd_bctl->ulSdBufAddr[sd_bctl->ubInBufIdx]) + (sd_bctl->ulInputPtr); // destination address
    }
#endif

    if(gbDbgLogSdEn)
    {
        RTNA_DBG_Str(0, "[M"); 
        RTNA_DBG_Long(0, ulCount); 
        RTNA_DBG_Str(0, ":"); 
        //RTNA_DBG_Long(0, ulSrcaddr); 
        //RTNA_DBG_Str(0, " =>"); 
        //RTNA_DBG_Long(0, ulDstaddr); 
        //RTNA_DBG_Str(0, "_"); 
        //RTNA_DBG_Byte(0, ep_bctl->ulWriteBufIdx); 
        //RTNA_DBG_Str(0, "_"); 
        //RTNA_DBG_Byte(0, sd_bctl->ubInBufIdx); 
        //RTNA_DBG_Str(0, "_"); 
        //RTNA_DBG_Long(0, sd_bctl->ulInputPtr); 
        //RTNA_DBG_Str(0, ".\r\n");
    }
    
    if(MMPF_DMA_MoveData(ulSrcaddr, ulDstaddr, ulCount, USB_UVCH_Record_Ctl, &m_UVCHRxArg, MMP_FALSE, NULL)) {
        RTNA_DBG_Str(0, "MMPF_UVCH_MoveData Fail\r\n"); // 
    }

//    memcpy((void *)ulDstaddr, (void *)ulSrcaddr, ulCount);
//    if ( memcmp((MMP_UBYTE *)(ulSrcaddr), (MMP_UBYTE *)(ulDstaddr), ulCount) )
//            RTNA_DBG_Str(0, "Error\r\n");

    //sd_bctl->ulInputPtr += ulCount; //Bruce remove
    //sd_bctl->ulDataSize[sd_bctl->ubInBufIdx] = sd_bctl->ulInputPtr; //Bruce remove

}

void USB_UVCH_BulkIn_Task(void)
{
    MMPF_OS_FLAGS flags;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
    SD_BUF_CTL    *sd_bctl = &sd_buf_ctl;
    MMP_ULONG   ulHeaderCnt;
#if 0
    MMPS_DISPLAY_DISPATTRIBUTE  dispAttr;
    MMPS_DISPLAY_WINATTRIBUTE   winAttr;
    MMPD_SCALER_FIT_RANGE           fitrange;
    MMPD_SCALER_GRABCONTROL         DispGrabctl;
#endif
    #if (OS_CRITICAL_METHOD == 3)
    OS_CPU_SR   cpu_sr = 0;
    #endif
            

    RTNA_DBG_Str(0, "USB_UVCH_BulkIn_Task()\r\n");

    //creat sem here;
    m_UVCHBulkInSemID = MMPF_OS_CreateSem(0);
    if (m_UVCHBulkInSemID >= MMPF_OS_SEMID_MAX) {
        RTNA_DBG_Str(0, "Create m_UVCHBulkInSemID sem failed\r\n");
        MMPF_OS_DeleteSem(m_UVCHBulkInSemID);
        return;
    }
//    m_UVCHRefreshSemID = MMPF_OS_CreateSem(0);

    while(1){

        MMPF_OS_WaitFlags(USB_OP_Flag,
                USB_FLAG_UVC_HOST_REQ | USB_FLAG_UVC_PKT_GOT | USB_FLAG_UVC_HOST_STP | USB_FLAG_UVC_INPUT_FRAME_REQ,
                MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME , 0, &flags);
        

        if (flags & USB_FLAG_UVC_HOST_REQ)
        {
            //gbDbgLogLcdEn = 1;     //Bruce add dbglog
            
            RTNA_DBG_Str(0, "UVC Req\r\n ");
            MEMSET0(&h264_nalu);
            bUVCHRecdEnabled = MMP_FALSE;
            /* Received data and write to file */
            uvch_xfer_ctl.ubUVCRxState = RX_UNKNOWN_PKT;
            uvch_xfer_ctl.ulXferSize = 0;
            uvch_xfer_ctl.ulFrameCnt = 0;
            ep_bctl->ulWritePtr = 0;
            ep_bctl->ulWriteBufIdx = 0;

#if 1
            //Bruce test TV out
            #if 1
            *((MMP_USHORT*)(0x80002d84)) = 1;//0x03;
            *((MMP_USHORT*)(0x80002d86)) = 1;//0x04;
            *((MMP_USHORT*)(0x80002d88)) = 1;//0x03;
            *((MMP_USHORT*)(0x80002d8a)) = 1;//0x04;

            *((MMP_USHORT*)(0x80002d98)) = 0;
            *((MMP_USHORT*)(0x80002d9A)) = 0;

            *((MMP_USHORT*)(0x80002d10)) = 2;
            *((MMP_USHORT*)(0x80002d14)) = 320*2;

            *((MMP_USHORT*)(0x80002d50)) = 320;//0xf0;
            *((MMP_USHORT*)(0x80002d54)) = 240/2;//0xb4;
            *((MMP_USHORT*)(0x80002d58)) = 340;
            *((MMP_USHORT*)(0x80002d5C)) = 100;
			
            *((MMP_USHORT*)(0x80002da0)) = 320;//0x140;
            *((MMP_USHORT*)(0x80002da4)) = 240/2;//0xf0;
            *((MMP_USHORT*)(0x80002db0)) = 1;
            *((MMP_USHORT*)(0x80002db4)) = 320;//0xf0;
            *((MMP_USHORT*)(0x80002db8)) = 1;
            *((MMP_USHORT*)(0x80002dbc)) = 240/2;//0xb4;
			
            *((MMP_ULONG*)(0x80002d68)) = 320*240;
            *((MMP_ULONG*)(0x80002d6C)) = 320*240/2;//0xa8c0;
            *((MMP_UBYTE*)(0x8000284E)) = 0xc6;

            *((MMP_UBYTE*)(0x80002d82)) = 0x02;
            *((MMP_UBYTE*)(0x80002d64)) = 0x20;
            
            #else
            winAttr.ulBaseAddr     = 0x01500000;
            winAttr.ulBaseUAddr    = 0x01500000;
            winAttr.ulBaseVAddr    = 0x01500000;
            winAttr.usWidth        = 320;
            winAttr.usHeight       = 240;
            winAttr.usLineOffset   = 320*2;
            winAttr.colordepth     = MMPS_DISPLAY_WINCOLORDEPTH_YUV422;

            dispAttr.usStartX          = 0;
            dispAttr.usStartY          = 0;
            dispAttr.usDisplayWidth    = 320;
            dispAttr.usDisplayHeight   = 240;
            dispAttr.usDisplayOffsetX  = 340;
            dispAttr.usDisplayOffsetY  = 200;
            dispAttr.rotatetype        = MMPD_DISPLAY_ROTATE_NO_ROTATE;
            dispAttr.bMirror           = MMP_FALSE;

            //MMPS_Display_SetWinActive(MMPD_DISPLAY_WIN_OVERLAY, MMP_FALSE); //active after frame ready
            MMPS_TV_SetInterlaceMode(MMP_TRUE);
            MMPS_Display_SetWindowAttr(MMPD_DISPLAY_WIN_OVERLAY,winAttr,dispAttr);
            //MMPS_Display_ClearWinBuffer(MMPD_DISPLAY_WIN_OVERLAY,0x0,320,240,0,0);
            MMPS_Display_SetWinPriority(MMPD_DISPLAY_WIN_OVERLAY,MMPD_DISPLAY_WIN_PIP,MMPD_DISPLAY_WIN_MAIN,MMPD_DISPLAY_WIN_OSD);
            MMPS_Display_SetWinActive(MMPD_DISPLAY_WIN_OVERLAY, MMP_TRUE);
            #endif
#endif
#if 0
            // window attributes
            winAttr.usWidth        = 640; // 240;
            winAttr.usHeight       = 480; // 160;
            winAttr.usLineOffset   = (winAttr.usWidth *2); // 480;//width * 2;
            winAttr.colordepth     = MMPD_DISPLAY_WINCOLORDEPTH_YUV422;//MMPD_DISPLAY_WINCOLORDEPTH_YUV422;
            winAttr.ulBaseAddr     = lcd_bctl->ulLcdBufAddr[0];
            winAttr.ulBaseUAddr    = lcd_bctl->ulLcdBufAddr[0];
            winAttr.ulBaseVAddr    = lcd_bctl->ulLcdBufAddr[0];
            MMPD_Display_SetWinAttributes(MMPF_DISPLAY_WIN_OVERLAY, &winAttr);

            // Display Setting
            dispAttr.usStartX              = 0;
            dispAttr.usStartY              = 0;    
            dispAttr.usDisplayWidth        = 240; // 320;//winAttr.usWidth;
            dispAttr.usDisplayHeight       = 160; // 160; // 320; // 240;//winAttr.usHeight;
            dispAttr.usDisplayOffsetX      = 0;
            dispAttr.usDisplayOffsetY      = 160; // (320 /2);    
            dispAttr.bMirror               = 0;
            dispAttr.rotatetype            = MMPD_DISPLAY_ROTATE_NO_ROTATE;
            MMPD_Display_SetWinToDisplay(MMPF_DISPLAY_WIN_OVERLAY, &dispAttr);

            fitrange.fitmode        = MMPD_SCALER_FITMODE_OUT;
            fitrange.usFitResol     = 64; // 64;
            fitrange.usInWidth      = 240; // winAttr.usWidth; // 320;
            fitrange.usInHeight     = 160; // winAttr.usHeight; // 240;        
            fitrange.usOutWidth     = winAttr.usWidth; // 240;
            fitrange.usOutHeight    = winAttr.usHeight; // 160; // 160; // 180;
//            MMPD_Display_GetWindowScale(&fitrange, &DispGrabctl);
            MMPD_Scaler_GetBestFitScale(&fitrange, &DispGrabctl);
            MMPD_Display_SetWinScaling(MMPF_DISPLAY_WIN_OVERLAY, MMP_TRUE, MMP_FALSE, &fitrange, &DispGrabctl);	    
#endif
             USB_UVCH_ReqOnePkt();
         }

        if (flags & USB_FLAG_UVC_PKT_GOT)
        {
//            RTNA_DBG_Str(0, "Pkt Got\r\n ");

            switch(uvch_xfer_ctl.ubUVCRxState) {

                case RX_UNKNOWN_PKT :
                    uvch_xfer_ctl.ulFrameSize = 0;
                    uvch_xfer_ctl.ulXferSize = 0;
                    ulHeaderCnt = 0;

                    if(uvch_xfer_ctl.ulFTarget) {

                        if(uvch_xfer_ctl.ulFTarget <= uvch_xfer_ctl.ulFrameCnt) {

                            uvch_xfer_ctl.ulFrameCnt = 0;

                            #if (0)
                            //stop anything operation to avoid mistake pointer
                            if(ST_H264 == video_format) { // H264
                                
                                #if 1
                                sd_bctl->ubInBufIdx = 0;
                                #else
                                sd_bctl->ubInBufIdx++;
                                if (sd_bctl->ubInBufIdx == SD_BUF_CNT)
                                    sd_bctl->ubInBufIdx = 0;
                                #endif
  
//                                sd_bctl->ubFrameCnt++;
                                sd_bctl->ulInputPtr = 0;
                            }
                            #endif


                            //bUVCHRecdEnabled = MMP_FALSE; //Bruce test, do not release for next record!

                            //uvch_xfer_ctl.ubUVCRxState = RX_INACTIVE;
                            //MMPF_OS_ReleaseSem(m_UVCHBulkInSemID);
                        }
                    } 

                    if ( USB_UVCH_Chk_Payload_Header(0) && USB_UVCH_Check_AIT_Header(0, 1) ) {
//                        RTNA_DBG_Str(0, "Fend Pkt\r\n");
                        uvch_xfer_ctl.ubUVCRxState = RX_1ST_DATA_PKT;
                        
                    }
                    else {
//                        RTNA_DBG_Str(0, "Fend Pkt\r\n");
                        USB_UVCH_ReqOnePkt();
                        break;
                    }

                case RX_1ST_DATA_PKT :

#if 0
                    //YUY2 Single format
                    uvch_xfer_ctl.ulFrameSize = 0x25800;
                    //uvch_xfer_ctl.ulFrameSize = (MMP_ULONG)640*480*2;
#else
                    m_UVCHRxArg.ulFrameSize = (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x2B))<<24 |
    					(*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x2A))<<16 |
    					(*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x29))<<8 |
    					(*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x28));

                    uvch_xfer_ctl.ulFrameSize = m_UVCHRxArg.ulFrameSize + 0x20;
#endif
                    ulHeaderCnt = uvch_xfer_ctl.ulFrameSize/0x1FF4;
                    if ((uvch_xfer_ctl.ulFrameSize % 0x1FF4) != 0) 
                        ulHeaderCnt++;

                    uvch_xfer_ctl.ulFrameSize = uvch_xfer_ctl.ulFrameSize + (ulHeaderCnt * 0x0C);

                    if(ST_H264 == video_format) // H264
                    {
                        if(gbDbgLogSdEn)
                        {
                            struct _UVC_H264_PH *header = (struct _UVC_H264_PH *)((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x0C) ;
                            //RTNA_DBG_Str(0, "[TP:");
                            //RTNA_DBG_Byte(0, video_format);
                            RTNA_DBG_Str(0, ",SNO:");
                            RTNA_DBG_Short(0, header->dwFrameSeq);
                            //RTNA_DBG_Str(0, ",FS:");
                            //RTNA_DBG_Long(0, uvch_xfer_ctl.ulFrameSize);
                            //RTNA_DBG_Long(0, header->dwPayloadSize);
                            //RTNA_DBG_Str(0, ", ulFrameCnt: ");
                            //RTNA_DBG_Str(0, ", FC:");
                            //RTNA_DBG_Short(0, uvch_xfer_ctl.ulFrameCnt);
                            RTNA_DBG_Str(0, ",");
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x0)));
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x1)));
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x2)));//PTS
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x3)));//PTS
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x4)));//PTS
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0x5)));//PTS
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0xA)));//SCR
                            //RTNA_DBG_Byte(0, (*(MMP_UBYTE *) ((ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx]) + 0xB)));//SCR
                            RTNA_DBG_Str(0, "\r\n");
                        }
                    }

                    //case: frame < 512 Byte
                    if(uvch_xfer_ctl.ulFrameSize < uvch_xfer_ctl.ulUsbRxByte) {
                        uvch_xfer_ctl.ulUsbRxByte = uvch_xfer_ctl.ulFrameSize;
                    }

                case RX_RES_DATA_PKT :
                    /* Update total received data size */
                    uvch_xfer_ctl.ulXferSize += uvch_xfer_ctl.ulUsbRxByte;
                    ep_bctl->ulWritePtr += uvch_xfer_ctl.ulUsbRxByte;
/*
                    RTNA_DBG_Str(0, "ulXferSize: ");
                    RTNA_DBG_Long(0, uvch_xfer_ctl.ulXferSize);
                    RTNA_DBG_Str(0, "\r\n");
*/
                    /* Update buf addr */
                    if ( (ep_bctl->ulWritePtr+512) > (ep_bctl->ulBufSize) ) {

                        /* One slot full(EP_RX_BUF_SIZE), trigger task to deal and move data */
                        ep_bctl->ulContentSize[ep_bctl->ulWriteBufIdx] = ep_bctl->ulWritePtr;
/*
                        RTNA_DBG_Str(0, "UVCH Get : ");
                        RTNA_DBG_Byte(0, ep_bctl->ulWriteBufIdx);
                        RTNA_DBG_Long(0, ep_bctl->ulContentSize[ep_bctl->ulWriteBufIdx]);
                        RTNA_DBG_Str(0, "\r\n");
*/
                        if(ST_H264 == video_format) { // H264
//                            RTNA_DBG_Str(0, "2_");
                            //stop move frame once reach ulFTarget
                            if(gbWrCount < (uvch_xfer_ctl.ulFTarget+1)) {
                                USB_UVCH_Ep2Sd_BufCtl();
                            }
                        }
                        else if(ST_YUY2 == video_format) { // YUY2
                            
                            USB_UVCH_Ep2Lcd_BufCtl();
                        }

                        ep_bctl->ulWritePtr = 0;
                        ep_bctl->ulWriteBufIdx++;
                        if (ep_bctl->ulWriteBufIdx == EP_RX_BUF_CNT) {
                            ep_bctl->ulWriteBufIdx = 0;
                        }
                    }

                    if (uvch_xfer_ctl.ulXferSize < uvch_xfer_ctl.ulFrameSize) {
                        uvch_xfer_ctl.ubUVCRxState = RX_RES_DATA_PKT;
                        USB_UVCH_TriggerDmaRx();
                    }
                    else { //  Data payload completed, wait for Frame End Packet
                        uvch_xfer_ctl.ulXferSize = 0;
                        
/*
                        RTNA_DBG_Str(0, "ulFrameCnt: ");
                        RTNA_DBG_Long(0, uvch_xfer_ctl.ulFrameCnt);
                        RTNA_DBG_Str(0, "\r\n");
*/
                        if(ep_bctl->ulWritePtr) {
                            ep_bctl->ulContentSize[ep_bctl->ulWriteBufIdx] = ep_bctl->ulWritePtr;

                            if(ST_H264 == video_format) { // H264
//                                RTNA_DBG_Str(0, "3_");
                                //stop move frame once reach ulFTarget
                                if(gbWrCount < (uvch_xfer_ctl.ulFTarget+1)) {
                                    USB_UVCH_Ep2Sd_BufCtl();
                                }
                            }
                            else if(ST_YUY2 == video_format) { // YUY2
                                
                                USB_UVCH_Ep2Lcd_BufCtl();
                            }

                            ep_bctl->ulWritePtr = 0;
                            ep_bctl->ulWriteBufIdx++;
                            if (ep_bctl->ulWriteBufIdx == EP_RX_BUF_CNT) {
                                ep_bctl->ulWriteBufIdx = 0;
                            }
                        }
                        
                        if(ST_H264 == video_format) { // H264
                            uvch_xfer_ctl.ulFrameCnt++;
                            //next frame toggle bit for check payload header
                            uvch_xfer_ctl.ubFrameToggle++ ;
                            uvch_xfer_ctl.ubFrameToggle = uvch_xfer_ctl.ubFrameToggle & 1; 
                        }
						
                        video_format = ST_UNDEF;

                        if(gbWrCount < (uvch_xfer_ctl.ulFTarget+1)) {
                            //stop get frame once reach ulFTarget
                            USB_UVCH_ReqOnePkt();
                        } else {
                            uvch_xfer_ctl.ubUVCRxState = RX_INACTIVE;
                            MMPF_OS_ReleaseSem(m_UVCHBulkInSemID);
                            RTNA_DBG_Str(0, "[END] un-lock task\r\n");
                            RTNA_DBG_Str(0, "CHECK:");
                            RTNA_DBG_Str(0, "\r\n");
                        }

                     }

                    break;

                default :
                    break;
            }

        }

        if (flags & USB_FLAG_UVC_HOST_STP)
        {
            /*
             * Clear_Feature
             */
            RTNA_DBG_Str(0, "= Clear_Feature =\r\n");
            USBCore_Host_SET_CMD(0x02, CLEAR_FEATURE, 0, 0x81, 0);
            uvch_xfer_ctl.ubUVCRxState = RX_INACTIVE;
        }
        
        if (flags & USB_FLAG_UVC_INPUT_FRAME_REQ)
        {
            #if (0) //debug log
            RTNA_DBG_Str(0, "[");
            //RTNA_DBG_Str(0, "[S");
            //RTNA_DBG_Byte(0, sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotRd]);
            //RTNA_DBG_Long(0, sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotRd]);
            //RTNA_DBG_Str(0, ",");
            //RTNA_DBG_Long(0, sd_bctl->ulFrameTimeStamp[sd_bctl->ubCurFrameSlotRd]);
            //RTNA_DBG_Byte(0, sd_bctl->ubFrameType[sd_bctl->ubCurFrameSlotRd]);
            RTNA_DBG_Byte(0, gbWrCount);
            RTNA_DBG_Str(0, "\r\n");
            #endif

            if(gbWrCount < uvch_xfer_ctl.ulFTarget) {
                #if (DUMP_H264_BY_FS_IF)
                //MMPF_FS_FWrite, MMPS_FS_FileWrite
                MMPF_FS_FWrite(fpUvc, 
                                  (MMP_UBYTE *)sd_bctl->ulSdBufAddr[sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotRd]], 
                                  sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotRd], 
                                  &resultUvc);
                
                if(resultUvc!=sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotRd]) {
                    RTNA_DBG_Str(0, " ![#"); RTNA_DBG_Long(0, (__LINE__));
                    RTNA_DBG_Str(0, ",");
                    RTNA_DBG_Long(0, resultUvc);
                    RTNA_DBG_Str(0, "\r\n");
                }
                #else
                MMPS_3GPRECD_UVCRecdInputFrame(sd_bctl->ulSdBufAddr[sd_bctl->ubFrameIdx[sd_bctl->ubCurFrameSlotRd]], 
                                               sd_bctl->ulFrameSize[sd_bctl->ubCurFrameSlotRd], 
                                               sd_bctl->ulFrameTimeStamp[sd_bctl->ubCurFrameSlotRd], 
                                               sd_bctl->ubFrameType[sd_bctl->ubCurFrameSlotRd], 0);
                #endif

                gbWrCount++;
                sd_bctl->ubCurFrameSlotRd++;
                if (sd_bctl->ubCurFrameSlotRd >= SD_BUF_FRM_SLOT_CNT)
                    sd_bctl->ubCurFrameSlotRd = 0;
                
                OS_ENTER_CRITICAL();
                sd_bctl->ubFrameCnt--;
                OS_EXIT_CRITICAL();
                
                if(sd_bctl->ubFrameCnt) {
                    MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_INPUT_FRAME_REQ, MMPF_OS_FLAG_SET);      
                }
            } else {
                
                if(gbWrCount < (uvch_xfer_ctl.ulFTarget+1)) {
                    #if (DUMP_H264_BY_FS_IF)
                    MMPS_FS_FileClose(fpUvc);
                    
                    RTNA_DBG_Str(0,"Save file complete\r\n");   
                    #else
                    MMPS_3GPRECD_StopUVCRecd();
                    
                    #if 0
                    MMPS_3GPRECD_StartSeamless(MMP_FALSE);
                    #if (VMD_SUPPORT)
                    if (MMPS_Sensor_IsVMDEnable())
                        MMPS_Sensor_EnableVMD(MMP_FALSE);
                    #endif
                    MMPS_3GPRECD_StopRecord();
                    #if (VIDREC_SUPPORT_SEAMLESS)
                    m_ulFirstRecNameIdx = 0xFFFFFFFF;
                    #endif
                    RTNA_DBG_Str(0, "stop\r\n");
                    #if (VIDREC_STICKER_ENABLE)
                    MMPF_OS_StopTimer(tmrID, OS_TMR_OPT_NONE);
                    #endif
                    #endif
                    
                    RTNA_DBG_Str(0,"Save file complete:");   
                    RTNA_DBG_Short(0,gbWrCount);   
                    RTNA_DBG_Str(0," frames\r\n");   
                    #endif
                    
                    gbWrCount++;
                }
            }
        }

    }
}

void USB_UVCH_Record_Setting(void)
{
    #if (VIDRECD_MULTI_TRACK == 0)
    MMP_ULONG   ulFileNameLen;
    #endif
    MMPS_3GPRECD_CONTAINER_INFO pInfo;

    //if (0) {
    if (bUVCHRecdEnabled == MMP_FALSE) {
        
        #if (DUMP_H264_BY_FS_IF)
        //ulFileNameLen = STRLEN(gbUVCRecdFileName);
        //MMPS_FS_FileOpen(gbUVCRecdFileName, ulFileNameLen, "w+b", 4, &fpUvc);
        ulFileNameLen = STRLEN(DUMP_H264_FILE);
        MMPS_FS_FileOpen(DUMP_H264_FILE, ulFileNameLen, "w+b", 4, &fpUvc);
        
    	if(fpUvc == NULL) 
        {
            RTNA_DBG_Str(0,"![Cannot open: ");
            RTNA_DBG_Str(0, gbUVCRecdFileName);
            RTNA_DBG_Str(0,"\r\n");
            return;
        }
        
        #else
        // need to set UVC recording information
        pInfo.VideoEncodeFormat = MMPS_3GPRECD_VIDEO_FORMAT_H264;
        pInfo.ulFrameWidth = (h264_nalu[0x07])<<8 |(h264_nalu[0x06]);;  // 1280;
        pInfo.ulFrameHeight = (h264_nalu[0x09])<<8 |(h264_nalu[0x08]);; // 720;
        pInfo.ulTimeIncrement  = 1000;
        pInfo.ulTimeResolution = 30000;
        pInfo.usPFrameCount = 30;
        pInfo.usBFrameCount = 0;
        pInfo.ulSPSAddr = (MMP_ULONG)(&h264_nalu[0x24]);
        pInfo.ulSPSSize = 11;
        pInfo.ulPPSAddr = (MMP_ULONG)(&h264_nalu[0x33]);
        pInfo.ulPPSSize = 4;
        pInfo.ulProfileIDC = (MMP_USHORT)Profile_IDC; //BASELINE_PROFILE, 0x42 from AQUILIA
        pInfo.ulLevelIDC = (MMP_USHORT)Level_IDC; //0x28 from AQUILIA
        MMPS_3GPRECD_SetUVCRecdInfo(&pInfo); 
        
        MMPS_3GPRECD_SetStoragePath(MMPS_3GPRECD_SRCMODE_CARD);
        #if (VIDRECD_MULTI_TRACK == 0)
        CreateRecordFileName(m_usResolIdx, m_ulRecNameIdx, 0, 0);
        ulFileNameLen = STRLEN(gbUVCRecdFileName);
        MMPS_3GPRECD_SetFileName(/*gbRecordFileName*/gbUVCRecdFileName, ulFileNameLen, MMPS_3GPRECD_FILETYPE_UVCRECD);
        #endif
        MMPS_3GPRECD_RegisterCallback(MMPS_3GPRECD_EVENT_UVCFILE_FULL, (void *)UVCRecordCbFileFull);
    
        #if (VIDRECD_MULTI_TRACK == 0)
        MMPS_3GPRECD_StartUVCRecd(MMPS_UVCRECD_MULFILE); 
        #else
        MMPS_3GPRECD_StartUVCRecd(MMPS_UVCRECD_MULTRACK); 
        #endif
        
        #endif
        
        #if (VIDRECD_MULTI_TRACK == 0)
	    RTNA_DBG_Str(0, "USB_UVCH_Record_Setting \r\n");
        m_ulRecNameIdx++;
        #endif
        bUVCHRecdEnabled = MMP_TRUE;
    }

}
#endif
#endif

#endif // End of #if (USB_EN)
/// @}
