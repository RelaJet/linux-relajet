//==============================================================================
//
//  File        : mmpf_usbh_ctl.c
//  Description : USB HOST controller related function
//  Author      : Bruce_Chien
//  Revision    : 1.0
//
//==============================================================================

#if 1 //(USB_EN)
#if (SUPPORT_USB_HOST_FUNC)
#include "mmpf_typedef.h"
#include "lib_retina.h"
#include "mmp_reg_gbl.h"
#include "mmp_reg_usb.h"
#include "mmpf_system.h"
#include "mmpf_usbuac.h"
#include "mmpf_usbmsdc.h"
#include "mmpf_usbphy.h"
#include "mmpf_usbh_cfg.h"
#include "mmpf_usbh_ctl.h"
#include "mmpf_usbh_kernal.h"


//==============================================================================
//
//                              CONSTANT
//
//==============================================================================
#define LIST_DMA_CH_ENABLE      (0)
#define TRY_DMA_INT_2           (0)// 0x8000_14D0
#define POLLING_STATUS          (0)     /* check status by busy polling */
#define EN_DBGLOG_TRIGUSBDMA    (0)

#define USBH_OTG_DEV_BASE_ADDR      (0x05)  /* just an example, never mind to modify it */
#define UVC_GET_FRAME_TARGET_CNT    (300)   /* lock CPU to get target frame # continuously */


//==============================================================================
//
//                              VARIABLES
//
//==============================================================================
static CMD_BLOCK_WRAPPER    cbw;
#if (DMA3_RX_LAST_PKT_PATCH)
static CMD_STATUS_WRAPPER   csw;
#endif
static USB_OTG_HANDLE       otg_handle;
static volatile MMP_BOOL    dev_connected = MMP_FALSE;
//static MMP_ULONG   glUSBHDramEndAddr = 0;

static MMP_ULONG   glDmaTxBufAddr;
#if (DMA3_RX_LAST_PKT_PATCH)
static MMP_ULONG   glDmaRxBufAddr;
#endif

UVCH_XFER_CTL        uvch_xfer_ctl;
SD_BUF_CTL   sd_buf_ctl;
LCD_BUF_CTL   lcd_buf_ctl;
MMP_UBYTE   video_format;
UVCH_Rx_Param m_UVCHRxArg;

//==============================================================================
//
//                              EXTERNAL
//
//==============================================================================
extern MMPF_OS_FLAGID USB_OP_Flag;
extern MMPF_OS_SEMID  m_UVCHBulkInSemID;


//==============================================================================
//
//                              LOCAL FUNCTIONS
//
//==============================================================================
MMP_ERR MMPF_USBH_WaitEp0TxDone(void)
{
    MMP_ERR   err = MMP_ERR_NONE;
    MMP_USHORT csr;

    while(1) {
        csr = GetEp0Csr();
        if (!(csr & EP0_TX_PKTRDY))
            break;
    }

    if (csr & EP0_STALLED) {
        SetEp0Csr(EP0_CLR_STALL);
        RTNA_DBG_Str(0, "Device stalled\r\n");
        err = MMP_USB_ERR_EP_RX_STALL;
    }
    if (csr & HOST_EP0_ERR_SR) {
        SetEp0Csr(HOST_EP0_ERR_SR);
        RTNA_DBG_Str(0, "Device Tx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP0_NAK_TO) {
        SetEp0Csr(HOST_EP0_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }

    return err;
}

MMP_ERR MMPF_USBH_WaitEp0RxDone(void)
{
    MMP_ERR   err = MMP_ERR_NONE;
    MMP_USHORT csr;
    MMP_USHORT wait_csr;

    wait_csr = EP0_STALLED | HOST_EP0_ERR_SR |
                HOST_EP0_NAK_TO | EP0_RX_PKTRDY;
    while(1) {
        csr = GetEp0Csr();
        if (csr & wait_csr)
            break;
    }

    if (csr & EP0_STALLED) {
        SetEp0Csr(EP0_CLR_STALL);
        RTNA_DBG_Str(0, "Device stalled\r\n");
        err = MMP_USB_ERR_EP_RX_STALL;
    }
    if (csr & HOST_EP0_ERR_SR) {
        SetEp0Csr(HOST_EP0_ERR_SR);
        RTNA_DBG_Str(0, "Device Rx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP0_NAK_TO) {
        SetEp0Csr(HOST_EP0_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }
    /*
    if (csr & EP0_RX_PKTRDY) {
        RTNA_DBG_Str(0, "Rx PktRdy\r\n");
    }
    */
    return err;
}

MMP_ERR MMPF_USBH_WaitEpTxDone(MMP_UBYTE ep)
{
    MMP_ERR   err = MMP_ERR_NONE;
    MMP_USHORT csr;

    if (ep != 1 && ep != 2)
        return MMP_USB_ERR_EP_UNKNOWN;
        
    while(1) {
        csr = GetEpTxCsr(ep);
        if (!(csr & EP_TX_PKTRDY))
            break;
    }

    if (csr & EP_TX_STALLED) {
        SetEpTxCsr(ep, EP_TX_CLR_STALL);
        RTNA_DBG_Str(0, "Tx stalled\r\n");
        err = MMP_USB_ERR_EP_TX_STALL;
    }
    if (csr & HOST_EP_TX_ERR_SR) {
        SetEpTxCsr(ep, HOST_EP_TX_ERR_SR);
        RTNA_DBG_Str(0, "Tx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP_TX_BULK_NAK_TO) {
        SetEpTxCsr(ep, HOST_EP_TX_BULK_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }

    return err;
}

MMP_ERR MMPF_USBH_WaitEpRxDone(MMP_UBYTE ep)
{
    MMP_ERR    err = MMP_ERR_NONE;
    MMP_USHORT csr;
    MMP_USHORT wait_csr;

    if (ep != 1 && ep != 2)
        return MMP_USB_ERR_EP_UNKNOWN;
        
    wait_csr = EP_RX_STALLED | HOST_EP_RX_ERR_SR |
                HOST_EP_RX_BULK_NAK_TO | EP_RX_PKTRDY;
    while(1) {
        csr = GetEpRxCsr(ep);
        if (csr & wait_csr)
            break;
    }

    if (csr & EP_RX_STALLED) {
        SetEpRxCsr(ep, EP_RX_CLR_STALL);
        RTNA_DBG_Str(0, "Rx stalled\r\n");
        err = MMP_USB_ERR_EP_RX_STALL;
    }
    if (csr & HOST_EP_RX_ERR_SR) {
        SetEpRxCsr(ep, HOST_EP_RX_ERR_SR);
        RTNA_DBG_Str(0, "Rx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP_RX_BULK_NAK_TO) {
        SetEpRxCsr(ep, HOST_EP_RX_BULK_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }

    return err;
}

MMP_ERR MMPF_USBH_WriteEpFifo(MMP_UBYTE ep, MMP_UBYTE *data, MMP_ULONG size)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    if ((ep > 7) || !size || !data)
        return MMP_USB_ERR_PARAMETER;

    while(size--)
        pUSB_CTL->USB_FIFO_EP[ep].FIFO_B = *data++;
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_ReadEpFifo(MMP_UBYTE ep, MMP_UBYTE *buf, MMP_USHORT *psByteinFifo)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT  bytes_infifo, unread;

    if ((ep > 7) || !buf) {
        *psByteinFifo = 0;
        return MMP_USB_ERR_PARAMETER;
    }

    bytes_infifo = GetEpRxCount(ep);
    unread = bytes_infifo;
    while(unread--)
        *buf++ = pUSB_CTL->USB_FIFO_EP[ep].FIFO_B;

    *psByteinFifo = bytes_infifo;
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_SetDeviceAddr(MMP_UBYTE addr)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_FADDR = addr;
    pUSB_CTL->USB_MP_CTL[0].EP_TX_FUNC_ADDR = addr;
    pUSB_CTL->USB_MP_CTL[0].EP_RX_FUNC_ADDR = addr;
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_ConfigEp0(MMP_UBYTE speed)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_EP[0].USB_EP_TYPE = speed & EP_XFER_SPEED_MASK;
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_ConfigEp(MMP_UBYTE ep, USB_EP_ATR *attribute)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT csr;

    if ((ep > 7) || !attribute)
        return MMP_USB_ERR_PARAMETER;

    if (attribute->ep_dir == DIR_IN) {
        pUSB_CTL->USB_EP[ep].USB_EP_RX_MAXP = attribute->ep_maxP;
        pUSB_CTL->USB_EP[ep].USB_EP_RX_TYPE = attribute->ep_num | 
                        attribute->ep_xfer_type | attribute->ep_speed;
        pUSB_CTL->USB_EP[ep].USB_EP_RX_INTVL = attribute->ep_interval;
        SetEpRxCsr(ep, EP_RX_CLR_DATATOG);
        csr = GetEpRxCsr(ep);
        if (csr & EP_RX_PKTRDY) {
            /* Flush FIFO twice if double buffering is enabled */
            SetEpRxCsr(ep, csr | EP_RX_FLUSH_FIFO);
            SetEpRxCsr(ep, csr | EP_RX_FLUSH_FIFO);
        }
    }
    else if (attribute->ep_dir == DIR_OUT) {
        pUSB_CTL->USB_EP[ep].USB_EP_TX_MAXP = attribute->ep_maxP;
        pUSB_CTL->USB_EP[ep].USB_EP_TYPE = attribute->ep_num | 
                        attribute->ep_xfer_type | attribute->ep_speed;
        pUSB_CTL->USB_EP[ep].USB_EP_TX_INTVL = attribute->ep_interval;
        SetEpTxCsr(ep, EP_TX_MODE | EP_TX_CLR_DATATOG);
        csr = GetEpTxCsr(ep);
        if (csr & EP_TX_FIFO_NOTEMPTY) {
            /* Flush FIFO twice if double buffering is enabled */
            SetEpTxCsr(ep, csr | EP_TX_FLUSH_FIFO);
            SetEpTxCsr(ep, csr | EP_TX_FLUSH_FIFO);
        }
    }
    pUSB_CTL->USB_MP_CTL[ep].EP_TX_FUNC_ADDR = attribute->ep_addr;
    pUSB_CTL->USB_MP_CTL[ep].EP_RX_FUNC_ADDR = attribute->ep_addr;
    
    return MMP_ERR_NONE;
}



MMP_ERR MMPF_USBH_SendEp0Setup(EP0_REQUEST *setup_pkt)
{
    MMPF_USBH_WriteEpFifo(0, (MMP_UBYTE *)setup_pkt, 8);

    SetEp0Csr(HOST_EP0_SETUP | EP0_TX_PKTRDY);
    MMPF_USBH_WaitEp0TxDone();
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_ReceiveEp0Data(MMP_UBYTE *data_buf, MMP_ULONG *plDataBytes)
{
    MMP_USHORT  read_bytes;
    MMP_ULONG   data_bytes = 0;
    MMP_ERR     err = MMP_ERR_NONE;

    while(1) {
        SetEp0Csr(HOST_EP0_REQ_PKT);
        err = MMPF_USBH_WaitEp0RxDone();
        if (err) {
            *plDataBytes = data_bytes;
            return err;
        }

        MMPF_USBH_ReadEpFifo(0, data_buf, &read_bytes);
        data_buf += read_bytes;
        data_bytes += read_bytes;
        SetEp0Csr(EP0_RX_PKTRDY);

        if (read_bytes < EP0_MAX_PACKET_SIZE) {
            *plDataBytes = data_bytes;
            return err;
        }
    }
    return err;
}

MMP_ERR MMPF_USBH_SendEp0Data(MMP_UBYTE *data_buf, MMP_ULONG size)
{
    MMPF_USBH_WriteEpFifo(0, data_buf, size);

    SetEp0Csr(EP_TX_MODE | EP0_TX_PKTRDY);
    MMPF_USBH_WaitEp0TxDone();
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_HandleEp0Status(MMP_UBYTE dir)
{
    /* Status Stage @ USB spec. 8.5.3 Control Transfers */
    if (dir == DIR_OUT) {
        SetEp0Csr(HOST_EP0_STATUS | EP0_TX_PKTRDY);
        MMPF_USBH_WaitEp0TxDone();
    }
    else {
        SetEp0Csr(HOST_EP0_STATUS | HOST_EP0_REQ_PKT);
        MMPF_USBH_WaitEp0RxDone();
        /* Must flush FIFO here, otherwise, old data sent
         * in the following Tx. No idea about that, it is
         * try-out result. */
        SetEp0Csr(EP0_RX_PKTRDY | EP0_FLUSH_FIFO);
    }
    
    return MMP_ERR_NONE;
}


#if (SEPARATE_HEADER_RX)
MMP_ERR MMPF_USBH_TriggerDmaRx64Byte(void)
{
    MMP_ULONG       buf_addr;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
    MMP_ERR         err;

    buf_addr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx] + ep_bctl->ulWritePtr;
    memset((MMP_UBYTE *)buf_addr, 0, BULK_MAX_PACKET_SIZE);

    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT){
        //TBD if necessary to IN_TOKEN on SHT?
        SetEpRxCsr(1, HOST_EP_RX_REQ_PKT);
        err = MMPF_USBH_WaitEpRxDone(1);
    }
    
    #if (EN_DBGLOG_TRIGUSBDMA)
    //can not identify format @ this moment
    //if(video_format==ST_H264) {
    //  RTNA_DBG_Str(0, "H\r\n");
    //}
    #endif
    
    uvch_xfer_ctl.ubUVCRxState = RX_UNKNOWN_PKT;
    uvch_xfer_ctl.ulUsbRxByte = PKT_64BYTE_LEN;//(UVC_PH_LEN + FRAME_PAYLOAD_HEADER_SZ);
    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbRxByte, 1);
    //RTNA_DBG_Str(0, "~R64 ");
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_TriggerDmaRx448Byte(void)
{
    MMP_ULONG       buf_addr;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;

    buf_addr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx] + ep_bctl->ulWritePtr;
    memset((MMP_UBYTE *)buf_addr, 0, BULK_MAX_PACKET_SIZE);
    
    #if (EN_DBGLOG_TRIGUSBDMA)
    //can not identify format @ this moment
    //if(video_format==ST_H264) {
    //  RTNA_DBG_Str(0, "H\r\n");
    //}
    #endif
    
    uvch_xfer_ctl.ubUVCRxState = RX_VALID_PKT;
    uvch_xfer_ctl.ulUsbRxByte = BULK_MAX_PACKET_SIZE - PKT_64BYTE_LEN;//FRAME_PAYLOAD_HEADER_SZ;
    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbRxByte, 0);
    //RTNA_DBG_Str(0, "~R448 ");
    
    return MMP_ERR_NONE;
}
#endif

MMP_ERR MMPF_USBH_TriggerDmaRxOnePkt(void)
{
    MMP_ULONG       buf_addr;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
    MMP_ERR    err;

    buf_addr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx] + ep_bctl->ulWritePtr;
    memset((MMP_UBYTE *)buf_addr, 0, BULK_MAX_PACKET_SIZE);

    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT){
        SetEpRxCsr(1, HOST_EP_RX_REQ_PKT);
        err = MMPF_USBH_WaitEpRxDone(1);
    }
    
    #if (EN_DBGLOG_TRIGUSBDMA)
    //can not identify format @ this moment
    //if(video_format==ST_H264) {
    //  RTNA_DBG_Str(0, "H\r\n");
    //}
    #endif
    
    uvch_xfer_ctl.ubUVCRxState = RX_UNKNOWN_PKT;
    uvch_xfer_ctl.ulUsbRxByte = BULK_MAX_PACKET_SIZE;
    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbRxByte, 0);
    //RTNA_DBG_Str(0, "~ ");
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_TriggerDmaRx(void)
{
    MMP_ULONG       buf_addr; // , UsbDmaSize;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;

    buf_addr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx] + ep_bctl->ulWritePtr;

    uvch_xfer_ctl.ulUsbRxByte = uvch_xfer_ctl.ulFrameSize - uvch_xfer_ctl.ulXferSize;
    if (uvch_xfer_ctl.ulUsbRxByte > EP_RX_BUF_SIZE) {
        if(uvch_xfer_ctl.ulXferSize & 0x00001FFF)
            uvch_xfer_ctl.ulUsbRxByte = EP_RX_BUF_SIZE - (uvch_xfer_ctl.ulXferSize & 0x00001FFF);
        else	
            uvch_xfer_ctl.ulUsbRxByte = EP_RX_BUF_SIZE;
    }
    
    #if (EN_DBGLOG_TRIGUSBDMA)
    if(video_format==ST_H264) {
        RTNA_DBG_Str(0, "K");
        RTNA_DBG_Long(0, uvch_xfer_ctl.ulUsbRxByte);
        RTNA_DBG_Str(0, "\r\n");
    }
    #endif
    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbRxByte, 1);
    
    return MMP_ERR_NONE;
}

void USB_OTG_ISR(void)
{
    AITPS_USB_CTL   pUSB_CTL = AITC_BASE_USBCTL;
    AITPS_USB_DMA   pUSB_DMA = AITC_BASE_USBDMA;
    MMP_USHORT      dma_int;
    MMP_USHORT  tx_int, rx_int;
    MMP_UBYTE   usb_int;
#if (DMA3_RX_LAST_PKT_PATCH)
    MMP_ULONG       buf_addr;
    SLOT_BUF_CTL    *ep_bctl = &uvch_xfer_ctl.buf_ctl;
#endif

    tx_int = pUSB_CTL->USB_TX_INT_SR & pUSB_CTL->USB_TX_INT_EN;
    rx_int = pUSB_CTL->USB_RX_INT_SR & pUSB_CTL->USB_RX_INT_EN;
    usb_int = pUSB_CTL->USB_INT_EVENT_SR & pUSB_CTL->USB_INT_EVENT_EN;

    #if (CHIP == P_V2)
    dma_int = (pUSB_DMA->USB_DMA.INT_SR)&(pUSB_DMA->USB_DMA.INT_EN);
    #endif
    #if (CHIP == MCR_V2)
    dma_int = (pUSB_DMA->USB_DMA.INT_SR)&(pUSB_DMA->USB_DMA.CTL.INT_EN);
    #endif

    if (usb_int) {
        if (usb_int & USB_INT_CONN) {
            RTNA_DBG_Str(0, "DevConnected\r\n");
            dev_connected = MMP_TRUE;
        }
        if (usb_int & USB_INT_DISCON) {
            RTNA_DBG_Str(0, "DevDisconnected\r\n");
            dev_connected = MMP_FALSE;
        }
    }

    if (dma_int) {
        pUSB_DMA->USB_DMA.INT_SR = 0;
        #if (CHIP == P_V2)
        if ((pUSB_DMA->USB_DMA.CTL1) & USB_DMA_RX)
        #endif
        #if (CHIP == MCR_V2)
        if ((pUSB_DMA->USB_DMA3.CTL1) & USB_DMA_RX)
        #endif
        {
#if (DMA3_RX_LAST_PKT_PATCH)
            if(uvch_xfer_ctl.ulUsbPktCnt > 1) {
                if(uvch_xfer_ctl.ubUVCStreamEn)
                {
                    #if (EN_DBGLOG_TRIGUSBDMA)
                    if(video_format==ST_H264) {
                        RTNA_DBG_Str(0, "I");
                        RTNA_DBG_Long(0, uvch_xfer_ctl.ulUsbLastPktByte);
                        RTNA_DBG_Str(0, "\r\n");
                    }
                    #endif
                    buf_addr = ep_bctl->ulBufAddr[ep_bctl->ulWriteBufIdx] + ep_bctl->ulWritePtr + (uvch_xfer_ctl.ulUsbPktCnt * 0x200) - 0x200;
                    #if (SEPARATE_HEADER_RX)
                    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbLastPktByte, 1);
                    #else
                    MMPF_USBH_SetBulkDma3Rx(1, buf_addr, uvch_xfer_ctl.ulUsbLastPktByte, 0);
                    #endif
                    uvch_xfer_ctl.ulUsbPktCnt = 0;
                }
                else
                {
                    /* stop trigger if UVC device stream off */
                    uvch_xfer_ctl.ulUsbPktCnt = 0;
                    MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_PKT_GOT, MMPF_OS_FLAG_SET);      
                }
            }
            else {
                #if (EN_DBGLOG_TRIGUSBDMA)
                if(video_format==ST_H264) {
                    RTNA_DBG_Str(0, "J\r\n");
                }
                #endif
                MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_PKT_GOT, MMPF_OS_FLAG_SET);      
            }
#else
            MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_PKT_GOT, MMPF_OS_FLAG_SET);      
#endif
        }
    }
}

#if (POLLING_STATUS)
MMP_ERR MMPF_USBH_WaitDmaDone(MMP_UBYTE ch)
{
    MMP_USHORT  dma_int, int_wait;

    RTNA_DBG_Str(0, "Polling : Dma ch");
    RTNA_DBG_Byte(0, ch);
    RTNA_DBG_Str(0, " ....");

    if(ch ==3)
        int_wait = USB_INT_DMA3_DONE;
    else if(ch ==2)
        int_wait = USB_INT_DMA2_DONE;
    else if(ch ==1)
        int_wait = USB_INT_DMA1_DONE;
#if (TRY_DMA_INT_2)
    while(1) {
        dma_int = GetDmaInt2SR();
        if (dma_int & int_wait)
            break;
    }

    SetDmaIntSR(0x00);
    SetDmaInt2SR(0x00);
#else
    while(1) {
        dma_int = GetDmaIntSR();
        if (dma_int & int_wait)
            break;
    }

    SetDmaIntSR(0x00);
    SetDmaInt2SR(0x00);
#endif
    RTNA_DBG_Str(0, " Done\r\n");
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_WaitDmaRxDone(MMP_UBYTE ep, MMP_UBYTE ch)
{
    MMP_ERR   err = MMP_ERR_NONE;
    MMP_USHORT csr;

    MMPF_USBH_WaitDmaDone(ch);

    csr = GetEpRxCsr(ep);

    if (csr & EP_RX_STALLED) {
        SetEpRxCsr(ep, EP_RX_CLR_STALL);
        RTNA_DBG_Str(0, "Rx stalled\r\n");
        err = MMP_USB_ERR_EP_RX_STALL;
    }
    if (csr & HOST_EP_RX_ERR_SR) {
        SetEpRxCsr(ep, HOST_EP_RX_ERR_SR);
        RTNA_DBG_Str(0, "Rx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP_RX_BULK_NAK_TO) {
        SetEpRxCsr(ep, HOST_EP_RX_BULK_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }

    return err;
}

MMP_ERR MMPF_USBH_WaitDmaTxDone(MMP_UBYTE ep, MMP_UBYTE ch)
{
    MMP_ERR   err = MMP_ERR_NONE;
    MMP_USHORT csr;

    MMPF_USBH_WaitDmaDone(ch);

    csr = GetEpTxCsr(ep);

    if (csr & EP_TX_STALLED) {
        SetEpTxCsr(ep, EP_TX_CLR_STALL);
        RTNA_DBG_Str(0, "Tx stalled\r\n");
        err = MMP_USB_ERR_EP_TX_STALL;
    }
    if (csr & HOST_EP_TX_ERR_SR) {
        SetEpTxCsr(ep, HOST_EP_TX_ERR_SR);
        RTNA_DBG_Str(0, "Tx error\r\n");
        err = MMP_USB_ERR_EP_ERROR;
    }
    if (csr & HOST_EP_TX_BULK_NAK_TO) {
        SetEpTxCsr(ep, HOST_EP_TX_BULK_NAK_TO);
        RTNA_DBG_Str(0, "NAK timeout\r\n");
        err = MMP_USB_ERR_EP_NAK_TIMEOUT;
    }

    return err;
}

#endif


MMP_ERR MMPF_USBH_ConfigDma3Tx(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT pkt_byte,MMP_USHORT last_pkt_byte,MMP_ULONG pkt_sum,MMP_ULONG zero_end)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
//    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

//    pUSB_CTL->USB_MP_CTL[endpoint].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[endpoint].EP_RX_FUNC_ADDR = otg_handle.dev_addr;

#if 0
	RTNA_DBG_Str(0, "\r\n========USB DMA3 TX CONFIG========\r\n");
	RTNA_DBG_PrintLong(0, endpoint);
	RTNA_DBG_PrintLong(0, fb_addr);
	RTNA_DBG_PrintLong(0, pkt_byte);
	RTNA_DBG_PrintLong(0, last_pkt_byte);
	RTNA_DBG_PrintLong(0, pkt_sum);
	RTNA_DBG_Str(0, "========USB DMA3 TX CONFIG========\r\n");
#endif
//	pUSB_DMA->USB_DMA3_CTL1 = 0x02;
	pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR = fb_addr;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE = pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE_LAST = last_pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_SUM = pkt_sum -1;

	pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xFF10FFFF;
	// ooxx842 add 0x20A70000
	pUSB_DMA->USB_DMA3.TAR_OR_VAL = 0x20A70000;
	*(volatile MMP_ULONG*)(0x800014D4) = 0xFFFFFFFF;
	*(volatile MMP_ULONG*)(0x800014D8) = 0x20A70000;

	pUSB_DMA->USB_DMA.INT_SR = 0x00;
	pUSB_DMA->USB_DMA_INT2_SR = 0x00;

#if (TRY_DMA_INT_2)
	pUSB_DMA->USB_DMA_INT2_EN = 0x40;
#else
	pUSB_DMA->USB_DMA.CTL.INT_EN = 0x40;
#endif

	pUSB_DMA->USB_DMA3.MODE.NORM.FIFO_ADDR = 0x1024 + ((endpoint - 1)<<2);
	pUSB_DMA->USB_DMA3.MODE.NORM.CMD_ADDR = 0x1100 + (endpoint<<4); 

	if(zero_end)
		pUSB_DMA->USB_DMA3.CTL1 =(0x01 | ((0x10)<<endpoint)) + 0x10;
	else
		pUSB_DMA->USB_DMA3.CTL1 =0x01 | ((0x10)<<endpoint);
    
    return MMP_ERR_NONE;
}

#if (DMA3_RX_LAST_PKT_PATCH)
MMP_ERR MMPF_USBH_ConfigDma3RxLP(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT pkt_byte,MMP_USHORT last_pkt_byte,MMP_ULONG pkt_sum, MMP_BOOL bPktReq)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

#if 0
	RTNA_DBG_Str(0, "\r\n========DMA Last Pkt RX========\r\n");
	RTNA_DBG_PrintLong(0, endpoint);
	RTNA_DBG_PrintLong(0, fb_addr);
	RTNA_DBG_PrintLong(0, pkt_byte);
	RTNA_DBG_PrintLong(0, last_pkt_byte);
	RTNA_DBG_PrintLong(0, pkt_sum);
	RTNA_DBG_Str(0, "========DMA Last Pkt RX========\r\n");
#endif
	pUSB_DMA->USB_DMA3.CTL1 = 0x02;
	pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR = fb_addr;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE = pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE_LAST = last_pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_SUM = pkt_sum -1;

	//CHIP_CORE_ID_MCR_V2_SHT: do (AND OR OP) after DMA done
	//CHIP_CORE_ID_MCR_V2_MP: do (AND OR OP) before DMA done
    #if (SEPARATE_HEADER_RX==0)
	if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP){
		bPktReq = 1;
	}
    #endif
    #if (SEPARATE_HEADER_RX)
    if(bPktReq)
	    pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xF820FFFF;
    else
	    pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xF821FFFF; //0xF801FFFF
    #else
	pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xF820FFFF;
    #endif
	pUSB_DMA->USB_DMA3.TAR_OR_VAL = 0x00000000 | ((MMP_ULONG)bPktReq<<21); // Set PktReq bit 
	if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP){
		//do LAST (AND  OR OP) after DMA trigger
		pUSB_DMA->LAST_TAR_AND_VAL = 0xFFFFFFFF;
		pUSB_DMA->LAST_TAR_OR_VAL = 0x00000000;
	}
	
	if(0)
	{
		RTNA_DBG_Str(0, "[");
		RTNA_DBG_Short(0, pkt_byte);
		RTNA_DBG_Str(0, " x");
		RTNA_DBG_Long(0, pkt_sum);
		RTNA_DBG_Str(0, " +");
		RTNA_DBG_Short(0, last_pkt_byte);
		RTNA_DBG_Str(0, ",REQ:");
		RTNA_DBG_Byte(0, bPktReq);
		RTNA_DBG_Str(0, ",");
        RTNA_DBG_Long(0, uvch_xfer_ctl.ulXferSize);
		RTNA_DBG_Str(0, ",");
		RTNA_DBG_Str(0, "\r\n");
	}
	
#if (TRY_DMA_INT_2)
	pUSB_DMA->USB_DMA_INT2_EN = 0x40;
#else
	pUSB_DMA->USB_DMA.CTL.INT_EN = USB_INT_DMA3_DONE; // Enable Int for dma3 done 
#endif

	pUSB_DMA->USB_DMA3.MODE.NORM.FIFO_ADDR = 0x1024 + ((endpoint - 1)<<2);
	pUSB_DMA->USB_DMA3.MODE.NORM.CMD_ADDR = 0x1104 + (endpoint <<4);
	if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT){
		if (bPktReq)
			pUSB_CTL->USB_EP[endpoint].USB_EP_RX_CSR = 0x20; // Set PktReq bit to issue IN token
	}

	pUSB_DMA->USB_DMA3.CTL1= 0x05 | ((endpoint&7)<<5); // Start USB EP DMA
    
    return MMP_ERR_NONE;
}
#else // #if (DMA3_RX_LAST_PKT_PATCH)
MMP_ERR MMPF_USBH_ConfigDma3Rx(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT pkt_byte,MMP_USHORT last_pkt_byte,MMP_ULONG pkt_sum)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
	//AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

//    pUSB_CTL->USB_MP_CTL[endpoint].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[endpoint].EP_RX_FUNC_ADDR = otg_handle.dev_addr;
#if 0
	RTNA_DBG_Str(0, "\r\n========USB DMA3 RX CONFIG========\r\n");
	RTNA_DBG_PrintLong(0, endpoint);
	RTNA_DBG_PrintLong(0, fb_addr);
	RTNA_DBG_PrintLong(0, pkt_byte);
	RTNA_DBG_PrintLong(0, last_pkt_byte);
	RTNA_DBG_PrintLong(0, pkt_sum);
	RTNA_DBG_Str(0, "========USB DMA3 RX CONFIG========\r\n");
#endif
    
	pUSB_DMA->USB_DMA3.CTL1 = 0x02;
	pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR = fb_addr;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE = pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE_LAST = last_pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_SUM = pkt_sum -1;

	pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xF820FFFF;
	pUSB_DMA->USB_DMA3.TAR_OR_VAL = 0x00200000;
	if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP){
    	//do LAST (AND  OR OP) after DMA trigger
    	pUSB_DMA->LAST_TAR_AND_VAL = 0xFFFFFFFF;
    	//*(volatile MMP_ULONG*)(0x800014D8) = 0x00200200;
    	pUSB_DMA->LAST_TAR_OR_VAL = 0x00000000;
    }

	if(0)
	{
		RTNA_DBG_Str(0, "[");
		RTNA_DBG_Short(0, pkt_byte);
		RTNA_DBG_Str(0, " x");
		RTNA_DBG_Long(0, pkt_sum);
		RTNA_DBG_Str(0, " +");
		RTNA_DBG_Short(0, last_pkt_byte);
		//RTNA_DBG_Str(0, ",REQ:");
		//RTNA_DBG_Byte(0, bPktReq);
		RTNA_DBG_Str(0, ",");
        RTNA_DBG_Long(0, uvch_xfer_ctl.ulXferSize);
		RTNA_DBG_Str(0, ",");
		RTNA_DBG_Str(0, "\r\n");
	}

//	pUSB_DMA->USB_DMA.INT_SR = 0x00;
//	pUSB_DMA->USB_DMA_INT2_SR = 0x00;

#if (TRY_DMA_INT_2)
	pUSB_DMA->USB_DMA_INT2_EN = 0x40;
#else
	pUSB_DMA->USB_DMA.CTL.INT_EN = USB_INT_DMA3_DONE;
#endif

	pUSB_DMA->USB_DMA3.MODE.NORM.FIFO_ADDR = 0x1024 + ((endpoint - 1)<<2);
	pUSB_DMA->USB_DMA3.MODE.NORM.CMD_ADDR = 0x1104 + (endpoint<<4);

//	pUSB_CTL->USB_EP[endpoint].USB_EP_RX_CSR = 0x20; // Set PktReq bit to issue IN token
	pUSB_DMA->USB_DMA3.CTL1= 0x05 | ((endpoint&7)<<5);
    
    return MMP_ERR_NONE;
}
#endif // #if (DMA3_RX_LAST_PKT_PATCH)

MMP_ERR MMPF_USBH_SetBulkDma3Tx(MMP_UBYTE ep, MMP_ULONG fb_addr,MMP_ULONG size)
{
    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

  #if (USB_XFER_SPEED == USB_FULL_SPEED)
//    pkt_byte = (size<0x40) ? 0 : 0x40;
    pkt_byte = (size<0x40) ? (size) : 0x40;
    last_pkt_byte = (size<0x40) ? (size) : (size & 0x3F);
    pkt_cnt = ((size+0x3F)>>6);
  #else
//    pkt_byte = (size<0x200) ? 0 : 0x200;
    pkt_byte = (size<0x200) ? size : 0x200;
    last_pkt_byte = (size<0x200) ? (size) : (size & 0x1FF);
    pkt_cnt = ((size+0x1FF)>>9);
  #endif
 
    MMPF_USBH_ConfigDma3Tx(ep,fb_addr,pkt_byte,last_pkt_byte,pkt_cnt, 0x00);

#if (POLLING_STATUS)
    MMPF_USBH_WaitDmaTxDone(ep, 3);
//    SetEpTxCsr(ep, EP_TX_MODE | EP_TX_PKTRDY);
#endif
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_SetBulkDma3Rx(MMP_UBYTE ep, MMP_ULONG fb_addr,MMP_ULONG rx_byte, MMP_BOOL bPktReq)
{
//    MMP_ERR    err, data_bytes = 0;
#if (DMA3_RX_LAST_PKT_PATCH)

    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

  #if (USB_XFER_SPEED == USB_FULL_SPEED)
    pkt_byte = (rx_byte<0x40) ? rx_byte : 0x40;
    last_pkt_byte = (rx_byte<0x40) ? (rx_byte) : 0x40;
    pkt_cnt = ((rx_byte+0x3F)>>6);
  #else
    if (1){//(gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT){
        pkt_byte = (rx_byte<0x200) ? rx_byte : 0x200;
        last_pkt_byte = (rx_byte<0x200) ? (rx_byte) : 0x200;
    } else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP){
        pkt_byte = 0x200;
        last_pkt_byte = (rx_byte>0x200) ? 0x200: (rx_byte);
    }
    pkt_cnt = ((rx_byte+0x1FF)>>9);
  #endif
    uvch_xfer_ctl.ulUsbPktCnt = pkt_cnt;
    uvch_xfer_ctl.ulUsbLastPktByte = rx_byte - pkt_byte*(pkt_cnt-1);
    //if(video_format==ST_H264) {
    //    RTNA_DBG_Str(0, "_P");
    //    RTNA_DBG_Long(0, pkt_cnt);
    //    RTNA_DBG_Str(0, ",");
    //}

    if(pkt_cnt > 1)
        MMPF_USBH_ConfigDma3RxLP(ep, fb_addr, pkt_byte, pkt_byte, (pkt_cnt-1), bPktReq);
    else
        MMPF_USBH_ConfigDma3RxLP(ep, fb_addr, pkt_byte, last_pkt_byte, pkt_cnt, bPktReq);

#else   // #if (DMA3_RX_LAST_PKT_PATCH)

    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

  #if (USB_XFER_SPEED == USB_FULL_SPEED)
//    pkt_byte = (rx_byte<0x40) ? 0 : 0x40;
    pkt_byte = (rx_byte<0x40) ? rx_byte : 0x40;
    last_pkt_byte = (rx_byte<0x40) ? (rx_byte) : (rx_byte & 0x3F);
    pkt_cnt = ((rx_byte+0x3F)>>6);
  #else
//    pkt_byte = (rx_byte<0x200) ? 0 : 0x200;
    pkt_byte = (rx_byte<0x200) ? (rx_byte) : 0x200;
    last_pkt_byte = (rx_byte<0x200) ? (rx_byte) : (rx_byte & 0x1FF);
    pkt_cnt = ((rx_byte+0x1FF)>>9);
    if(pkt_cnt==1) {
        last_pkt_byte = pkt_byte;
    }
  #endif

    MMPF_USBH_ConfigDma3Rx(ep,fb_addr,pkt_byte,last_pkt_byte,pkt_cnt);

#endif  // #if (DMA3_RX_LAST_PKT_PATCH)

#if (POLLING_STATUS)
//    MMPF_USBH_WaitDmaRxDone(ep, 3);
#endif
    
    return MMP_ERR_NONE;
}

#if (LIST_DMA_CH_ENABLE)
void USBCore_dma_setlist(MMP_USHORT usSize, MMP_ULONG ulAddr, MMP_ULONG ulFbAddr, MMP_USHORT usPara)
{
    pUsbDmaDesc=(void*)ulAddr;

    if(usPara&LIST_LAST)    
       pUsbDmaDesc->dwNextDescAddr=0;
    else
	pUsbDmaDesc->dwNextDescAddr= (MMP_ULONG)(pUsbDmaDesc+1);
//	pUsbDmaDesc->dwNextDescAddr=ulAddr + USBDMA_LIST_LEN;    
    
    pUsbDmaDesc->dwPLDAddr=ulFbAddr;
    pUsbDmaDesc->wPara=usPara;
    pUsbDmaDesc->wPLDSize=usSize-1; 
    
    #if 0
    RTNA_DBG_PrintLong(0, ulAddr);
    RTNA_DBG_PrintLong(0, pUsbDmaDesc->dwNextDescAddr);
    RTNA_DBG_PrintLong(0, pUsbDmaDesc->dwPLDAddr);
    RTNA_DBG_PrintShort(0, pUsbDmaDesc->wPara);
    RTNA_DBG_PrintShort(0, pUsbDmaDesc->wPLDSize);
    #endif

    pUsbDmaDesc++;
}

void USBCore_Dma1_Enable(MMP_ULONG ulAddr,MMP_UBYTE ubEnableInt, MMP_UBYTE ubEndpoint)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
//	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	pUSB_DMA->USB_DMA.INT_SR = 0x00;
	pUSB_DMA->USB_DMA.CTL.INT_EN = 0x00; // ubEnableInt;
	pUSB_DMA->USB_DMA3.CTL1 = 0x02;
	RTNA_DBG_Str(0, "\r\n== DMA1 Enable ==\r\n");
	pUSB_DMA->USB_DMA.MODE.NORM.FB_ST_ADDR = ulAddr; // Descriptor List buffer address
	pUSB_DMA->USB_DMA.TAR_AND_VAL = 0xFF10FFFF;
	pUSB_DMA->USB_DMA.TAR_OR_VAL = 0x00A70000;
	pUSB_DMA->USB_DMA.MODE.NORM.CMD_ADDR = 0x1100|(ubEndpoint<<4);

#if (TRY_DMA_INT_2)
//	pUSB_DMA->USB_DMA_INT_SR &= ~(USB_INT_DMA1_DONE_EN|USB_INT_DMA1_SET_TXPKTRDY);
	pUSB_DMA->USB_DMA_INT2_SR = 0x00;
	pUSB_DMA->USB_DMA_INT2_EN |= 0x01; // ubEnableInt;
#else
//	pUSB_DMA->USB_DMA_INT_SR &= ~(USB_INT_DMA1_DONE_EN|USB_INT_DMA1_SET_TXPKTRDY);
	pUSB_DMA->USB_DMA.INT_SR = 0x00;
	pUSB_DMA->USB_DMA.CTL.INT_EN |= 0x01; // ubEnableInt;
#endif

	//DMA1 enable
	pUSB_DMA->USB_DMA.CTL1 = 0x01;
/*
    RTNA_DBG_Str(0, "EP_TX_FUNC_ADD");
    RTNA_DBG_Short(0, ubEndpoint);
    RTNA_DBG_Byte(0, pUSB_CTL->USB_MP_CTL[ubEndpoint].EP_TX_FUNC_ADDR);
    RTNA_DBG_Str(0, "\r\n");
*/
}

void USBCore_ExtDmaDscrList(MMP_USHORT ep, MMP_ULONG fb_addr, MMP_USHORT pkt_byte, MMP_USHORT last_pkt_byte,
                    MMP_ULONG pkt_sum, MMP_ULONG zero_end)
{
    MMP_UBYTE i = 0x0;

//    RTNA_DBG_Str(0, "\r\n== Prepare EXTDMA Descriptor List ==\r\n");
    
    for(i = 0x0; i < (pkt_sum-1); i++) {
        USBCore_dma_setlist(pkt_byte, otg_handle.tx_list_buf + i*(0xC), fb_addr + i*(pkt_byte),
//                           LIST_TXPKTRDY|LIST_INDEX|ep);
                           LIST_TXPKTRDY|ep);
    }
     
    USBCore_dma_setlist(last_pkt_byte, otg_handle.tx_list_buf + i*(0xC), fb_addr + i*(pkt_byte),
//                             LIST_TXPKTRDY|LIST_LAST|LIST_INDEX|ep);
                             LIST_TXPKTRDY|LIST_LAST|ep);

}

void USBCore_Dma1TxBuf_Config(MMP_USHORT ep, MMP_ULONG fb_addr,MMP_ULONG tx_byte)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

//    pUSB_CTL->USB_MP_CTL[ep].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[ep].EP_RX_FUNC_ADDR = otg_handle.dev_addr;

#if (USB_XFER_SPEED == USB_FULL_SPEED)
    pkt_byte = (tx_byte<0x40) ? 0 : 0x40;
//    last_pkt_byte = (tx_byte<0x40) ? (tx_byte) : 0x40;
    last_pkt_byte = (tx_byte&0x3F);
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x40 ;
    pkt_cnt = ((tx_byte+0x3F)>>6);
#else
    pkt_byte = (tx_byte<0x200) ? 0 : 0x200;
//    last_pkt_byte = (tx_byte<0x200) ? (tx_byte) : 0x200;
    last_pkt_byte = (tx_byte&0x1FF) ;
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x200 ;
    pkt_cnt = ((tx_byte+0x1FF)>>9);
#endif

    USBCore_ExtDmaDscrList(ep,fb_addr,pkt_byte,last_pkt_byte,pkt_cnt,0x0);

    USBCore_Dma1_Enable(otg_handle.tx_list_buf, USB_DMA_DONE, ep);

#if (POLLING_STATUS)
    MMPF_USBH_WaitDmaTxDone(ep, 1);
#endif
}

void USBCore_Dma2_Enable(MMP_ULONG ulAddr,MMP_UBYTE ubEnableInt, MMP_UBYTE ubEndpoint)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;

	RTNA_DBG_Str(0, "\r\n== DMA2 Enable ==\r\n");
	pUSB_DMA->USB_DMA2.MODE.NORM.FB_ST_ADDR = ulAddr; // Descriptor List buffer address
	pUSB_DMA->USB_DMA2.TAR_AND_VAL = 0xFF10FFFF;
	pUSB_DMA->USB_DMA2.TAR_OR_VAL = 0x00A70000;
	pUSB_DMA->USB_DMA2.MODE.NORM.CMD_ADDR = 0x1100|(ubEndpoint<<4);

	pUSB_DMA->USB_DMA.INT_SR &= ~(USB_INT_DMA2_DONE|USB_INT_DMA2_TXPKTRDY);
	pUSB_DMA->USB_DMA.CTL.INT_EN |= 0x08;

	//DMA2 enable
	pUSB_DMA->USB_DMA2.CTL1 |= 0x01;
}

void USBCore_Dma2TxBuf_Config(MMP_USHORT ep, MMP_ULONG fb_addr,MMP_ULONG tx_byte)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

//    pUSB_CTL->USB_MP_CTL[ep].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[ep].EP_RX_FUNC_ADDR = otg_handle.dev_addr;

#if (USB_XFER_SPEED == USB_FULL_SPEED)
    pkt_byte = (tx_byte<0x40) ? 0 : 0x40;
//    last_pkt_byte = (tx_byte<0x40) ? (tx_byte) : 0x40;
    last_pkt_byte = (tx_byte&0x3F);
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x40 ;
    pkt_cnt = ((tx_byte+0x3F)>>6);
#else
    pkt_byte = (tx_byte<0x200) ? 0 : 0x200;
//    last_pkt_byte = (tx_byte<0x200) ? (tx_byte) : 0x200;
    last_pkt_byte = (tx_byte&0x1FF) ;
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x200 ;
    pkt_cnt = ((tx_byte+0x1FF)>>9);
#endif

    USBCore_ExtDmaDscrList(ep,fb_addr,pkt_byte,last_pkt_byte,pkt_cnt,0x0);

    USBCore_Dma2_Enable(otg_handle.tx_list_buf, USB_DMA_DONE, ep);

#if (POLLING_STATUS)
    MMPF_USBH_WaitDmaTxDone(ep, 2);
#endif
}
#endif

MMP_ERR MMPF_USBH_initializeFB(void)
{
    MMP_ULONG idx;
    static MMP_BOOL sem_initialized = MMP_FALSE;

    /* Reset to default */
    //otg_handle.ep0_state = EP0_IDLE;

    /* Initialize semaphores */
    if (!sem_initialized) {
#if 0
        otg_handle.rx_sem = MMPF_OS_CreateSem(0);
        if (otg_handle.rx_sem >= MMPF_OS_SEMID_MAX) {
            RTNA_DBG_Str(0, "Create Rx sem failed\r\n");
            MMPF_OS_DeleteSem(otg_handle.rx_sem);
            return 0;
        }
        otg_handle.ep1_sem = MMPF_OS_CreateSem(0);
        if (otg_handle.ep1_sem >= MMPF_OS_SEMID_MAX) {
            RTNA_DBG_Str(0, "Create EP1 sem failed\r\n");
            MMPF_OS_DeleteSem(otg_handle.ep1_sem);
            return 0;
        }
        otg_handle.ep2_sem = MMPF_OS_CreateSem(0);
        if (otg_handle.ep2_sem >= MMPF_OS_SEMID_MAX) {
            RTNA_DBG_Str(0, "Create EP2 sem failed\r\n");
            MMPF_OS_DeleteSem(otg_handle.ep1_sem);
            MMPF_OS_DeleteSem(otg_handle.ep2_sem);
            return 0;
        }
        otg_handle.dma_sem = MMPF_OS_CreateSem(0);
        if (otg_handle.dma_sem >= MMPF_OS_SEMID_MAX) {
            RTNA_DBG_Str(0, "Create DMA sem failed\r\n");
            MMPF_OS_DeleteSem(otg_handle.dma_sem);
            return 0;
        }
#endif
        sem_initialized = MMP_TRUE;
    }
    
    /* Initialize file transfer control */
    //RTNA_DBG_Str(0, "EP BufAddr\r\n");
    MEMSET0(&uvch_xfer_ctl);
    for(idx = 0; idx < EP_RX_BUF_CNT; idx++) {
        uvch_xfer_ctl.buf_ctl.ulBufAddr[idx] = otg_handle.ep_data_buf[idx];
        //RTNA_DBG_Long(0, uvch_xfer_ctl.buf_ctl.ulBufAddr[idx]);
        //RTNA_DBG_Str(0, "\r\n");
    }
    uvch_xfer_ctl.buf_ctl.ulBufSize = EP_RX_BUF_SIZE;

    //RTNA_DBG_Str(0, "ulSdBufAddr\r\n");
    MEMSET0(&sd_buf_ctl);
    for(idx = 0; idx < SD_BUF_CNT; idx++) {
        sd_buf_ctl.ulSdBufAddr[idx] = otg_handle.sd_data_buf[idx];
        //RTNA_DBG_Long(0, sd_buf_ctl.ulSdBufAddr[idx]);
        //RTNA_DBG_Str(0, "\r\n");
    }
    sd_buf_ctl.ulSdBufSize = SD_BUF_SIZE;

    //RTNA_DBG_Str(0, "ulLcdBufAddr\r\n");
    MEMSET0(&lcd_buf_ctl);
    for(idx = 0; idx < LCD_BUF_CNT; idx++) {
        lcd_buf_ctl.ulLcdBufAddr[idx] = otg_handle.lcd_data_buf[idx];
        //RTNA_DBG_Long(0, lcd_buf_ctl.ulLcdBufAddr[idx]);
        //RTNA_DBG_Str(0, "\r\n");
    }
    lcd_buf_ctl.ulLcdBufSize = LCD_BUF_SIZE;
    
    MEMSET0(&m_UVCHRxArg);
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_SetFBMemory(MMP_ULONG plStartAddr, MMP_ULONG *plAllocSize)
{
    //MMP_ULONG size;
    MMP_ULONG idx;
    MMP_ULONG cur_addr;

    cur_addr = ALIGN32(plStartAddr);

    /* EP0 setup packet buffer */
    otg_handle.ep0_setup_buf = cur_addr;
    cur_addr = ALIGN32(cur_addr + EP0_SETUP_PACKET_SIZE);

    /* EP0 data packets buffer */    
    otg_handle.ep0_data_buf = cur_addr;
    cur_addr = ALIGN32(cur_addr + EP0_DATA_PACKET_SIZE);

    /* Rx EP data buffer */
    for(idx = 0; idx < EP_RX_BUF_CNT; idx++) {
        otg_handle.ep_data_buf[idx] = cur_addr;
        cur_addr = ALIGN32(cur_addr + EP_RX_BUF_SIZE);
        #if 0
        //Bruce @
        for(size = 0; size < EP_RX_BUF_SIZE; size++) {
            *(MMP_BYTE *)(otg_handle.ep_data_buf[idx] + size) = (rand()&0xFF);
        }
        #endif
    }

    cur_addr = ALIGN32(cur_addr + SD_BUF_SIZE);

    /* SD data buffer */
    for(idx = 0; idx < SD_BUF_CNT; idx++) {
        otg_handle.sd_data_buf[idx] = cur_addr;
        cur_addr = ALIGN256(cur_addr + SD_BUF_SIZE);
    }

    /* LCD data buffer */
    for(idx = 0; idx < LCD_BUF_CNT; idx++) {
        otg_handle.lcd_data_buf[idx] = cur_addr;
        cur_addr = ALIGN256(cur_addr + LCD_BUF_SIZE);
        #if 0
        //Bruce @
        for(size = 0; size < LCD_BUF_SIZE; size++) {
            *(MMP_BYTE *)(otg_handle.lcd_data_buf[idx] + size) = 0xAA;
        }
        #endif
    }

    cur_addr = ALIGN256(cur_addr + 512);

#if 0
    /* Rx EP data buffer */
    otg_handle.ep_rx_buf = cur_addr;
    otg_handle.ep_rx_buf_sz = EP_RX_BUF_SIZE;
    cur_addr = ALIGN32(cur_addr + otg_handle.ep_rx_buf_sz);

    /* Tx EP data buffer */
    otg_handle.ep_tx_buf = cur_addr;
    otg_handle.ep_tx_buf_sz = EP_TX_BUF_SIZE;
    cur_addr = ALIGN256(cur_addr + otg_handle.ep_tx_buf_sz);

    /* Tx EP data buffer */
    otg_handle.tx_list_buf = cur_addr;
    otg_handle.tx_list_buf_sz = 256;
    cur_addr = ALIGN32(cur_addr + otg_handle.tx_list_buf_sz);
#endif
    
    //glUSBHDramEndAddr = cur_addr;
    *plAllocSize = cur_addr - plStartAddr;
    
    RTNA_DBG_Str(0, "USBH FB memory from:");
    RTNA_DBG_Long(0, plStartAddr);
    RTNA_DBG_Str(0, ", End:");
    RTNA_DBG_Long(0, cur_addr);
    RTNA_DBG_Str(0, "\r\n");
    
    return MMP_ERR_NONE;
}


static MMP_ERR MMPF_USBH_DebugData(MMP_UBYTE *data, MMP_ULONG size)
{
    MMP_ULONG i = 0;

    for(i = 0; i < size; i++) {
        if ((i & 0x7) == 0) {
            if (i && (i != (size - 1)))
                RTNA_DBG_Str(0, "\r\n");
            RTNA_DBG_Str(0, "    ");
        }
        RTNA_DBG_Byte(0, data[i]);
    }
    RTNA_DBG_Str(0, "\r\n");
    return MMP_ERR_NONE;
}
#endif
#if 1
//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR MMPF_USBH_InitializeOTG(void)
{
#define EN_FORCE_HOST_MODE (1)
    #if (CHIP == P_V2)
    AITPS_GBL       pGBL = AITC_BASE_GBL;
    #endif
    AITPS_AIC       pAIC = AITC_BASE_AIC;
    AITPS_USB_CTL   pUSB_CTL = AITC_BASE_USBCTL;
    AITPS_USB_DMA   pUSB_DMA = AITC_BASE_USBDMA;
    //MMP_UBYTE       dev_ctl;

    /* Reset PHY & controller first */
    MMPF_USBPHY_PowerDown();
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
    #if (EN_FORCE_HOST_MODE)
    MMPF_USBPHY_Initialize(MMP_FALSE);//disable OTG comparators
    #else
    MMPF_USBPHY_Initialize(MMP_TRUE);
    #endif
    
    /* Enable USB OTG, signal from PHY */
    #if (EN_FORCE_HOST_MODE)
    pUSB_DMA->USB_OTG_CTL = (OTG_AVALID_TRUE | OTG_VBUSVALID_TRUE);//disable OTG_EN, force enable OTG_AVALID, OTG_VBUSVALID
    #else
    pUSB_DMA->USB_OTG_CTL = OTG_EN;
    #endif

    /* Connect DMPULLDOWN to PHY, must set */
    pUSB_DMA->USB_UTMI_PHY_CTL1 |= DMPULLDOWN_TO_PHY_EN;
    pUSB_CTL->USB_POWER = USB_SUSPENDM_EN | USB_HS_EN;
    pUSB_CTL->USB_ADT_DEV_CTL |= USB_SESSION;

    /* Wait device connected */
    pUSB_CTL->USB_INT_EVENT_EN = USB_INT_CONN | USB_INT_DISCON;
    RTNA_AIC_Open(pAIC, AIC_SRC_USB, usbotg_isr_a,
                  AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_USB);

    while(!dev_connected);
    RTNA_DBG_PrintByte(0, pUSB_CTL->USB_ADT_DEV_CTL); // 0x5D is expected

    pUSB_CTL->USB_POWER = USB_SUSPENDM_EN | USB_HS_EN;
    /*
     * The CPU should keep the Reset bit set for at least 20 ms 
     * to ensure correct resetting of the target device.
     */
    pUSB_CTL->USB_POWER |= USB_RESET;
    MMPF_OS_Sleep(30);
    pUSB_CTL->USB_POWER &= ~(USB_RESET);
/*
    dev_ctl = pUSB_CTL->USB_ADT_DEV_CTL;
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL should be 0x5D\r\n");
    RTNA_DBG_Str(0, "------------------------------------------------\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[7]: 0 => A device, 1 => B device\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[6]: 1 => FS or HS\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[5]: 1 => LS\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[4:3]: VBus[1:0]\r\n");
    RTNA_DBG_Str(0, "         00 => Below SessionEnd\r\n");
    RTNA_DBG_Str(0, "         01 => Above SessionEnd, below AValid\r\n");
    RTNA_DBG_Str(0, "         10 => Above AValid, below VBus Valid\r\n");
    RTNA_DBG_Str(0, "         11 => Above VBus Valid\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[2]: 1 => Host mode\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[1]: 1 => Host req\r\n");
    RTNA_DBG_Str(0, "USB_ADT_DEV_CTL[0]: 1 => Session Start\r\n");
    RTNA_DBG_Str(0, "------------------------------------------------\r\n");
    if (dev_ctl & 0x80)
        RTNA_DBG_Str(0, "- B device\r\n");
    else
        RTNA_DBG_Str(0, "- A device\r\n");
    if (dev_ctl & 0x40)
        RTNA_DBG_Str(0, "- FS or HS\r\n");
    else if (dev_ctl & 0x20)
        RTNA_DBG_Str(0, "- LS\r\n");
    switch ((dev_ctl & 0x18) >> 3) {
    case 0:
        RTNA_DBG_Str(0, "- VBus level: Below SessionEnd\r\n");
        break;
    case 1:
        RTNA_DBG_Str(0, "- VBus level: Above SessionEnd, below AValid\r\n");
        break;
    case 2:
        RTNA_DBG_Str(0, "- VBus level: Above AValid, below VBus Valid\r\n");
        break;
    case 3:
        RTNA_DBG_Str(0, "- VBus level: Above VBus Valid\r\n");
        break;
    }
    if (dev_ctl & 0x04)
        RTNA_DBG_Str(0, "- Host mode\r\n");
    else
        RTNA_DBG_Str(0, "- Device mode\r\n");
*/
    /* Diable all interrupts*/
    pUSB_CTL->USB_TX_INT_EN = 0;
    pUSB_CTL->USB_RX_INT_EN = 0;
    
    return MMP_ERR_NONE;
}
#endif

#if 0

MMP_ERR MMPF_USBH_OpenOTGDevice(MMP_ULONG work_buf, MMP_ULONG *buf_size)
{
    EP0_REQUEST *request;
    MMP_ULONG   data_length;
    USB_EP_ATR  ep_attr;

    /* Assign USB working buffer */
    MMPF_USBH_SetFBMemory(work_buf, buf_size);
    MMPF_USBH_initializeFB();

    #if (USB_XFER_SPEED == USB_FULL_SPEED)
    MMPF_USBH_ConfigEp0(EP_XFER_SPEED_FULL);
    #elif (USB_XFER_SPEED == USB_HIGH_SPEED)
    MMPF_USBH_ConfigEp0(EP_XFER_SPEED_HIGH);
    #endif

    /*
     * Get_Descriptor(Device)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Device) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, DEVICE_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x40);
    MMPF_USBH_SendEp0Setup(request);

    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Set_Address(USBH_OTG_DEV_BASE_ADDR)
     */
    RTNA_DBG_Str(0, "= Set_Address(USBH_OTG_DEV_BASE_ADDR) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, SET_ADDRESS);
    USETW(request->wValue, USBH_OTG_DEV_BASE_ADDR);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);
    MMPF_USBH_SetDeviceAddr(USBH_OTG_DEV_BASE_ADDR);

    /*
     * Get_Descriptor(Device)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Device) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, DEVICE_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x40);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Config)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, CONFIG_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x20);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Set_Configuration(1)
     */
    RTNA_DBG_Str(0, "= Set_Configuration(1) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, SET_CONFIGURATION);
    USETW(request->wValue, 1);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);

    /*
     * GetMaxLUN
     */
    RTNA_DBG_Str(0, "= GetMaxLUN =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_CLASS | RCPT_IF);
    USETB(request->bRequest, MSDC_GET_MAX_LUN);
    USETW(request->wValue, 0);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 1);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Config Bulk IN/OUT EPs
     */
    ep_attr.ep_dir = DIR_OUT;
    #if (USB_XFER_SPEED == USB_FULL_SPEED)
    ep_attr.ep_speed = EP_XFER_SPEED_FULL;
    #elif (USB_XFER_SPEED == USB_HIGH_SPEED)
    ep_attr.ep_speed = EP_XFER_SPEED_HIGH;
    #endif
    ep_attr.ep_xfer_type = EP_XFER_TYPE_BULK;
    ep_attr.ep_num = 2;
    ep_attr.ep_addr = USBH_OTG_DEV_BASE_ADDR;
    ep_attr.ep_maxP = BULK_MAX_PACKET_SIZE;
    ep_attr.ep_interval = 0;
    MMPF_USBH_ConfigEp(2, &ep_attr);

    ep_attr.ep_dir = DIR_IN;
    ep_attr.ep_num = 1;
    MMPF_USBH_ConfigEp(1, &ep_attr);
    /*
     * INQUIRY
     */
    RTNA_DBG_Str(0, "\r\n= INQUIRY =\r\n");
    memset((void *)&cbw, 0, CBW_PACKET_SIZE);
    cbw.dCBWSignature = CBW_SIGANTURE;
    cbw.dCBWTag = 0x07A64820;
    cbw.dCBWDataTxLength = INQUIRY_DATA_SIZE;
    cbw.bmCBWFlags = DATA_DIR_IN;
    cbw.bCBWLUN = 0;
    cbw.bCBWCBLength = 6;
    cbw.bCBWCB[0] = CMD_INQUIRY;
    cbw.bCBWCB[3] = 0x24;
    cbw.bCBWCB[4] = 0x00;

//    USBCore_EpBulkTx(2, (MMP_UBYTE *)&cbw, CBW_PACKET_SIZE);

    glDmaTxBufAddr = (MMP_ULONG)&cbw;
    MMPF_USBH_SetBulkDma3Tx(2, glDmaTxBufAddr, CBW_PACKET_SIZE);

  #if (DMA3_RX_LAST_PKT_PATCH)
    glDmaRxBufAddr = otg_handle.ep_data_buf[0];
    MMPF_USBH_SetBulkDma3Rx(1, glDmaRxBufAddr, INQUIRY_DATA_SIZE, 1);
    RTNA_DBG_Str(0, "= Data Packet =\r\n");
    MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep_data_buf[0], INQUIRY_DATA_SIZE+0x10);

    glDmaRxBufAddr = (MMP_ULONG)&csw;
    MMPF_USBH_SetBulkDma3Rx(1, glDmaRxBufAddr, CSW_PACKET_SIZE, 0);
    RTNA_DBG_Str(0, "= CSW =\r\n");
    MMPF_USBH_DebugData((MMP_UBYTE *)&csw, CSW_PACKET_SIZE+0x10);

  #else
#if 0
    glDmaRxBufAddr = otg_handle.ep_rx_buf;
    MMPF_USBH_SetBulkDma3Rx(1, glDmaRxBufAddr, INQUIRY_DATA_SIZE, 1);
    RTNA_DBG_Str(0, "= Data Packet =\r\n");
    MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep_rx_buf, INQUIRY_DATA_SIZE);

    glDmaRxBufAddr = (MMP_ULONG)&csw;
    MMPF_USBH_SetBulkDma3Rx(1, glDmaRxBufAddr, CSW_PACKET_SIZE, 1);
    RTNA_DBG_Str(0, "= CSW =\r\n");
    MMPF_USBH_DebugData((MMP_UBYTE *)&csw, CSW_PACKET_SIZE);
#endif
  #endif // #if (DMA3_RX_LAST_PKT_PATCH)


//    MMPF_USBH_TestBulkOut(0x4, 1); // dma channel 1
//    MMPF_USBH_TestBulkOut(0x4, 2); // dma channel 2
#if 0
    data_length = 1; // sector number
    do {
        RTNA_DBG_Str(0, "\r\n# Sector Count:");
        RTNA_DBG_Long(0, data_length);
        RTNA_DBG_Str(0, "\r\n");

        if (data_length & 0x00000001)
            MMPF_USBH_TestBulkOut(data_length, 1); // dma channel 1
        else
            MMPF_USBH_TestBulkOut(data_length, 2); // dma channel 2

        if(data_length > 7)
            data_length = 1; // sector number
        else
            data_length++; // sector number

    } while(1);
#endif
    
    return MMP_ERR_NONE;
}

#if 0
MMP_ERR MMPF_USBH_SetTxBuf(MMP_ULONG fb_addr,MMP_USHORT size)
{
    MMP_USHORT index;

    for(index=0; index<size; index++) {
        if(index & 0x0100)
            *(MMP_UBYTE *)(fb_addr+index) = 0xFF - (index & 0x00FF);
        else
            *(MMP_UBYTE *)(fb_addr+index) = (index & 0x00FF);
    }
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_TestBulkOut(MMP_ULONG sec_num, MMP_UBYTE dma_ch)
{
    MMP_ULONG   data_length;
    /*
     * CMD_AIT_SCSI_OPCODE (MSDC_AIT_SCSI_WRITEBOOTMEMORY)
     */
    RTNA_DBG_Str(0, "\r\n= AIT WRITE =\r\n");
    memset((void *)&cbw, 0, CBW_PACKET_SIZE);
    cbw.dCBWSignature = CBW_SIGANTURE;
    cbw.dCBWTag = 0x07A64820;
    cbw.dCBWDataTxLength = (sec_num<<9);
    cbw.bmCBWFlags = DATA_DIR_OUT;
    cbw.bCBWLUN = 0;
    cbw.bCBWCBLength = 6;
    cbw.bCBWCB[0] = CMD_AIT_SCSI_OPCODE;
    cbw.bCBWCB[3] = 0x02; // MSDC_AIT_SCSI_WRITEBOOTMEMORY
    cbw.bCBWCB[4] = (MMP_UBYTE)((sec_num & 0x0000FF00) >> 8);
    cbw.bCBWCB[5] = (MMP_UBYTE)(sec_num & 0x000000FF);
#if 1 // FIFO
    USBCore_EpBulkTx(2, (MMP_UBYTE *)&cbw, CBW_PACKET_SIZE);
#else
    glDmaTxBufAddr = (MMP_ULONG)&cbw;
    MMPF_USBH_SetBulkDma3Tx(2, glDmaTxBufAddr, CBW_PACKET_SIZE);
#endif
    // AIT_Write Data Payload
    RTNA_DBG_Str(0, "= Data Payload =\r\n");
//    glDmaTxBufAddr = otg_handle.ep_tx_buf;
    MMPF_USBH_SetTxBuf(otg_handle.ep_tx_buf, (sec_num<<9));
    if(1 == dma_ch)
        USBCore_Dma1TxBuf_Config(2, otg_handle.ep_tx_buf, (sec_num<<9));
    else
        USBCore_Dma2TxBuf_Config(2, otg_handle.ep_tx_buf, (sec_num<<9));
//    RTNA_DBG_Str(0, "Sent\r\n");

    // CSW
#if 1 // FIFO
    data_length = USBCore_EpBulkRx(1, (MMP_UBYTE *)&csw);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)&csw, data_length);
    }
#else
    glDmaRxBufAddr = (MMP_ULONG)&csw;
    MMPF_USBH_SetBulkDma3Rx(1, glDmaRxBufAddr, CSW_PACKET_SIZE, 1);

    RTNA_DBG_Str(0, "= CSW =\r\n");
    MMPF_USBH_DebugData((MMP_UBYTE *)&csw, CSW_PACKET_SIZE);
#endif
    
    return MMP_ERR_NONE;
}
#endif


/*
  * Get_Class_Request
  */
MMP_ERR MMPF_USBH_GetClassIfCmd(MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen, MMP_ULONG *UVCDataLength, MMP_UBYTE *UVCDataBuf)
{
    EP0_REQUEST *request;
//    MMP_ULONG   data_length;

    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_CLASS | RCPT_IF); // 0xA1
    USETB(request->bRequest, bReq);
    USETW(request->wValue, wVal);
    USETW(request->wIndex, wInd);
    USETW(request->wLength, wLen);
//    RTNA_DBG_Str(0, "= UVC_Get :");
//    MMPF_USBH_DebugData((MMP_UBYTE *)request, 8);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)UVCDataBuf, UVCDataLength);
//    if (*UVCDataLength) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)UVCDataBuf, *UVCDataLength);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);
    return MMP_ERR_NONE;
}

/*
  * Set_Class_IF_Request
  */
MMP_ERR MMPF_USBH_SetClassIfCmd(MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen, MMP_UBYTE *UVCDataBuf)
{
    EP0_REQUEST *request;

    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_CLASS | RCPT_IF); // 0x21
    USETB(request->bRequest, bReq);
    USETW(request->wValue, wVal);
    USETW(request->wIndex, wInd);
    USETW(request->wLength, wLen);
//    RTNA_DBG_Str(0, "= UVC_Set :");
//    MMPF_USBH_DebugData((MMP_UBYTE *)request, 8);
    MMPF_USBH_SendEp0Setup(request);

    MMPF_USBH_SendEp0Data(UVCDataBuf, wLen);
    MMPF_USBH_HandleEp0Status(DIR_IN);
    return MMP_ERR_NONE;
}

/*
  * Set_Std_EP_Request
  */
MMP_ERR MMPF_USBH_SetStdEpCmd(MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen, MMP_UBYTE *UVCDataBuf)
{
    EP0_REQUEST *request;

    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_EP); // 0x02
    USETB(request->bRequest, bReq);
    USETW(request->wValue, wVal);
    USETW(request->wIndex, wInd);
    USETW(request->wLength, wLen);
//    RTNA_DBG_Str(0, "= UVC_Set :");
//    MMPF_USBH_DebugData((MMP_UBYTE *)request, 8);
    MMPF_USBH_SendEp0Setup(request);

    if(wLen) {
        MMPF_USBH_SendEp0Data(UVCDataBuf, wLen);
    }
    MMPF_USBH_HandleEp0Status(DIR_IN);
    return MMP_ERR_NONE;
}

/*
  * Standard_Get_Request
  */
MMP_ERR MMPF_USBH_GetStdDevCmd(MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen)
{
    EP0_REQUEST *request;
    MMP_ULONG   data_length;

    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, bReq);
    USETW(request->wValue, wVal);
    USETW(request->wIndex, wInd);
    USETW(request->wLength, wLen);
//    RTNA_DBG_Str(0, "= Get_RTL :");
//    MMPF_USBH_DebugData((MMP_UBYTE *)request, 8);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
    if (data_length) {
        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);
    return MMP_ERR_NONE;
}

/*
  * Standard_Set_Request
  */
MMP_ERR MMPF_USBH_SetCmd(MMP_UBYTE bReqType, MMP_UBYTE bReq, MMP_USHORT wVal, MMP_USHORT wInd, MMP_USHORT wLen)
{
    EP0_REQUEST *request;

    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
//    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bmRequestType, bReqType);
    USETB(request->bRequest, bReq);
    USETW(request->wValue, wVal);
    USETW(request->wIndex, wInd);
    USETW(request->wLength, wLen);
//    RTNA_DBG_Str(0, "= Set_RTL :");
//    MMPF_USBH_DebugData((MMP_UBYTE *)request, 8);
    MMPF_USBH_SendEp0Setup(request);
    if(wLen)
        MMPF_USBH_SendEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, wLen);
    MMPF_USBH_HandleEp0Status(DIR_IN);
    return MMP_ERR_NONE;
}




MMP_ERR MMPF_USBH_GetDeviceConnectedSR(MMP_BOOL *pbEnable)
{
    *pbEnable = dev_connected;
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_SetDeviceConnectedSR(MMP_BOOL pbEnable)
{
    dev_connected = pbEnable;
    return MMP_ERR_NONE;
}


MMP_ERR MMPF_USBH_OpenUVCDevice(void)
{
    EP0_REQUEST *request;
    MMP_ULONG   data_length;
    USB_EP_ATR  ep_attr;
    EP0_DATA *DataBuf;

    #if (USB_XFER_SPEED == USB_FULL_SPEED)
    MMPF_USBH_ConfigEp0(EP_XFER_SPEED_FULL);
    #elif (USB_XFER_SPEED == USB_HIGH_SPEED)
    MMPF_USBH_ConfigEp0(EP_XFER_SPEED_HIGH);
    #endif
    DataBuf = (EP0_DATA *)otg_handle.ep0_data_buf;

    /*
     * Config Bulk IN/OUT EPs
     */
    ep_attr.ep_dir = DIR_OUT;
    #if (USB_XFER_SPEED == USB_FULL_SPEED)
    ep_attr.ep_speed = EP_XFER_SPEED_FULL;
    #elif (USB_XFER_SPEED == USB_HIGH_SPEED)
    ep_attr.ep_speed = EP_XFER_SPEED_HIGH;
    #endif
    ep_attr.ep_xfer_type = EP_XFER_TYPE_BULK;
    ep_attr.ep_num = 2;
    ep_attr.ep_addr = USBH_UVC_DEV_BASE_ADDR;
    ep_attr.ep_maxP = BULK_MAX_PACKET_SIZE;
    ep_attr.ep_interval = 0;
    MMPF_USBH_ConfigEp(2, &ep_attr);

    ep_attr.ep_dir = DIR_IN;
    ep_attr.ep_num = 1;
    MMPF_USBH_ConfigEp(1, &ep_attr);

    /*
     * Get_Descriptor(Device)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Device) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, DEVICE_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x40);
    MMPF_USBH_SendEp0Setup(request);

    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Set_Address(USBH_UVC_DEV_BASE_ADDR)
     */
    RTNA_DBG_Str(0, "= Set_Address(USBH_UVC_DEV_BASE_ADDR) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, SET_ADDRESS);
    USETW(request->wValue, USBH_UVC_DEV_BASE_ADDR);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);
    MMPF_USBH_SetDeviceAddr(USBH_UVC_DEV_BASE_ADDR);

    /*
     * Get_Descriptor(Device)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Device) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, DEVICE_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x12);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Config)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, CONFIG_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0xFF);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Config)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, CONFIG_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x44D);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, STRING_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0xFF);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0xFF);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Device)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Device) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, DEVICE_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x12);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Config)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, CONFIG_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x09);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(Config)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, CONFIG_DESC << 8);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0x44D);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Set_Configuration(1)
     */
    RTNA_DBG_Str(0, "= Set_Configuration(1) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, SET_CONFIGURATION);
    USETW(request->wValue, 1);
    USETW(request->wIndex, 0);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);

    /*
     * Set_Interface(1)
     */
    RTNA_DBG_Str(0, "= Set_Interface(3) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_IF);
    USETB(request->bRequest, SET_INTERFACE);
    USETW(request->wValue, 0);
    USETW(request->wIndex, 3);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8)|0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x04);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x2A);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8)|0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x04);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x2A);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8)|0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x04);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x2A);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8)|0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x04);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x2A);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

#if 1
//EU2
//XU_ENCODER_VIDEO_FORMAT_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_ENCODER_VIDEO_FORMAT_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_ENCODER_VIDEO_FORMAT_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_ENCODER_VIDEO_FORMAT_CONTROL, 10, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// VC_REQUEST_ERROR_CODE_CONTROL
    MMPF_USBH_GetVCCmd(GET_CUR_CMD, VC_REQUEST_ERROR_CODE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//XU_ENCODER_CONFIGURATION_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_ENCODER_CONFIGURATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_ENCODER_CONFIGURATION_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_ENCODER_CONFIGURATION_CONTROL, 25, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_CUR_CMD, XU_ENCODER_CONFIGURATION_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//XU_RATE_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_RATE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_RATE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_RATE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MAX_CMD, XU_RATE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_RES_CMD, XU_RATE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// VC_REQUEST_ERROR_CODE_CONTROL
    MMPF_USBH_GetVCCmd(GET_CUR_CMD, VC_REQUEST_ERROR_CODE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//XU_FRAME_TYPE_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_FRAME_TYPE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_FRAME_TYPE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_FRAME_TYPE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// VC_REQUEST_ERROR_CODE_CONTROL
    MMPF_USBH_GetVCCmd(GET_CUR_CMD, VC_REQUEST_ERROR_CODE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//XU_CAMERA_DELAY_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_CAMERA_DELAY_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_CAMERA_DELAY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_CAMERA_DELAY_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// VC_REQUEST_ERROR_CODE_CONTROL
    MMPF_USBH_GetVCCmd(GET_CUR_CMD, VC_REQUEST_ERROR_CODE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//XU_FILTER_CONTROL
    MMPF_USBH_GetEU2Cmd(GET_LEN_CMD, XU_FILTER_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_INFO_CMD, XU_FILTER_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU2Cmd(GET_MIN_CMD, XU_FILTER_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// VC_REQUEST_ERROR_CODE_CONTROL
    MMPF_USBH_GetVCCmd(GET_CUR_CMD, VC_REQUEST_ERROR_CODE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);

//EU1_SET_ISP
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_ISP, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_ISP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_ISP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_ISP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_ISP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_ISP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_ISP_RESULT
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_ISP_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_ISP_RESULT, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_ISP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_ISP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_ISP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_ISP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_SET_FW_DATA
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_FW_DATA, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_FW_DATA, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_FW_DATA, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_FW_DATA, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_FW_DATA, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_FW_DATA, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_SET_MMP
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_MMP, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_MMP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_MMP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_MMP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_MMP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_MMP, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_MMP_RESULT
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_MMP_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_MMP_RESULT, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_MMP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_MMP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_MMP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_MMP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
//EU1_SET_ISP_EX
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_ISP_EX, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_ISP_EX, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_ISP_EX, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_ISP_EX, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_ISP_EX, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_ISP_EX, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_ISP_EX_RESULT
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_ISP_EX_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_ISP_EX_RESULT, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_ISP_EX_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_ISP_EX_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_ISP_EX_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_ISP_EX_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_READ_MMP_MEM
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_READ_MMP_MEM, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_READ_MMP_MEM, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_READ_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_READ_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_READ_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_READ_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_WRITE_MMP_MEM
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_WRITE_MMP_MEM, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_WRITE_MMP_MEM, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_WRITE_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_WRITE_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_WRITE_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_WRITE_MMP_MEM, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_CHIP_INFO
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_CHIP_INFO, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_CHIP_INFO, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_CHIP_INFO, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_CHIP_INFO, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_CHIP_INFO, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_CHIP_INFO, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_DATA_32
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_DATA_32, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_DATA_32, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_SET_DATA_32
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_DATA_32, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_DATA_32, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_DATA_32, 32, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_SET_MMP_CMD16
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_MMP_CMD16, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_SET_MMP_CMD16, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_SET_MMP_CMD16, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_SET_MMP_CMD16, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_SET_MMP_CMD16, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_SET_MMP_CMD16, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU1_GET_MMP_CMD16_RESULT
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_MMP_CMD16_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_INFO_CMD, EU1_GET_MMP_CMD16_RESULT, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MIN_CMD, EU1_GET_MMP_CMD16_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_MAX_CMD, EU1_GET_MMP_CMD16_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_RES_CMD, EU1_GET_MMP_CMD16_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_DEF_CMD, EU1_GET_MMP_CMD16_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD1_SKYPE_ENCRES
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD1_SKYPE_ENCRES, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD1_SKYPE_ENCRES, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD1_SKYPE_ENCRES, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD1_SKYPE_ENCRES, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD1_SKYPE_ENCRES, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD1_SKYPE_ENCRES, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD2_SKYPE_FRAMERATE
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD2_SKYPE_FRAMERATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD2_SKYPE_FRAMERATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD2_SKYPE_FRAMERATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD2_SKYPE_FRAMERATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD2_SKYPE_FRAMERATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD2_SKYPE_FRAMERATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD3_SKYPE_BITRATE
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD3_SKYPE_BITRATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD3_SKYPE_BITRATE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD3_SKYPE_BITRATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD3_SKYPE_BITRATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD3_SKYPE_BITRATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD3_SKYPE_BITRATE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD4_SKYPE_ENFORCEKEY
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD4_SKYPE_ENFORCEKEY, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD5_SKYPE_MINQP
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD5_SKYPE_MINQP, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD5_SKYPE_MINQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD5_SKYPE_MINQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD5_SKYPE_MINQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD5_SKYPE_MINQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD5_SKYPE_MINQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD6_SKYPE_MAXQP
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD6_SKYPE_MAXQP, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD6_SKYPE_MAXQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD6_SKYPE_MAXQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD6_SKYPE_MAXQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD6_SKYPE_MAXQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD6_SKYPE_MAXQP, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMD7_SKYPE_CABAC
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMD7_SKYPE_CABAC, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMD7_SKYPE_CABAC, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMD7_SKYPE_CABAC, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMD7_SKYPE_CABAC, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMD7_SKYPE_CABAC, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMD7_SKYPE_CABAC, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDa_SKYPE_FWDAYS
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDa_SKYPE_FWDAYS, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDa_SKYPE_FWDAYS, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDa_SKYPE_FWDAYS, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDa_SKYPE_FWDAYS, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDa_SKYPE_FWDAYS, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDa_SKYPE_FWDAYS, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDb_SKYPE_PROFILE
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDb_SKYPE_PROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDb_SKYPE_PROFILE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDb_SKYPE_PROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDb_SKYPE_PROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDb_SKYPE_PROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDb_SKYPE_PROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDc_SKYPE_LEVEL
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDc_SKYPE_LEVEL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDc_SKYPE_LEVEL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDc_SKYPE_LEVEL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDc_SKYPE_LEVEL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDc_SKYPE_LEVEL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDc_SKYPE_LEVEL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDd_SKYPE_SUPPROFILE
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDd_SKYPE_SUPPROFILE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDe_SKYPE_UVCVER
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDe_SKYPE_UVCVER, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDe_SKYPE_UVCVER, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDe_SKYPE_UVCVER, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDe_SKYPE_UVCVER, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDe_SKYPE_UVCVER, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDe_SKYPE_UVCVER, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// EU0_CMDf_SKYPE_MODE
    MMPF_USBH_GetEU0Cmd(GET_LEN_CMD, EU0_CMDf_SKYPE_MODE, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_INFO_CMD, EU0_CMDf_SKYPE_MODE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MIN_CMD, EU0_CMDf_SKYPE_MODE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_MAX_CMD, EU0_CMDf_SKYPE_MODE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_RES_CMD, EU0_CMDf_SKYPE_MODE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU0Cmd(GET_DEF_CMD, EU0_CMDf_SKYPE_MODE, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// CT_ZOOM_ABSOLUTE_CONTROL
    MMPF_USBH_GetCTCmd(GET_INFO_CMD, CT_ZOOM_ABSOLUTE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_MIN_CMD, CT_ZOOM_ABSOLUTE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_MAX_CMD, CT_ZOOM_ABSOLUTE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_RES_CMD, CT_ZOOM_ABSOLUTE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_DEF_CMD, CT_ZOOM_ABSOLUTE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// CT_PANTILT_ABSOLUTE_CONTROL
    MMPF_USBH_GetCTCmd(GET_INFO_CMD, CT_PANTILT_ABSOLUTE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_MIN_CMD, CT_PANTILT_ABSOLUTE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_MAX_CMD, CT_PANTILT_ABSOLUTE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_RES_CMD, CT_PANTILT_ABSOLUTE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetCTCmd(GET_DEF_CMD, CT_PANTILT_ABSOLUTE_CONTROL, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_BRIGHTNESS_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_BRIGHTNESS_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_BRIGHTNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_BRIGHTNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_BRIGHTNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_BRIGHTNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_CONTRAST_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_CONTRAST_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_CONTRAST_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_CONTRAST_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_CONTRAST_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_CONTRAST_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_HUE_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_HUE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_HUE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_HUE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_HUE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_HUE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_SATURATION_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_SATURATION_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_SATURATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_SATURATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_SATURATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_SATURATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_SHARPNESS_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_SHARPNESS_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_SHARPNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_SHARPNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_SHARPNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_SHARPNESS_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_GAMMA_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_GAMMA_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_GAMMA_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_GAMMA_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_GAMMA_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_GAMMA_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_WHITE_BALANCE_TEMPERATURE_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD,  PU_WHITE_BALANCE_TEMPERATURE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD,  PU_WHITE_BALANCE_TEMPERATURE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD,  PU_WHITE_BALANCE_TEMPERATURE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD,  PU_WHITE_BALANCE_TEMPERATURE_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_BACKLIGHT_COMPENSATION_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_BACKLIGHT_COMPENSATION_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_BACKLIGHT_COMPENSATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_BACKLIGHT_COMPENSATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_BACKLIGHT_COMPENSATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_BACKLIGHT_COMPENSATION_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// PU_POWER_LINE_FREQUENCY_CONTROL	
    MMPF_USBH_GetPUCmd(GET_INFO_CMD, PU_POWER_LINE_FREQUENCY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MIN_CMD, PU_POWER_LINE_FREQUENCY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_MAX_CMD, PU_POWER_LINE_FREQUENCY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_RES_CMD, PU_POWER_LINE_FREQUENCY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetPUCmd(GET_DEF_CMD, PU_POWER_LINE_FREQUENCY_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
#endif	

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(String) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8)|0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x04);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);

    /*
     * Get_Descriptor(String)
     */
    RTNA_DBG_Str(0, "= Get_Descriptor(Config) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_IN | TYPE_STANDARD | RCPT_DEV);
    USETB(request->bRequest, GET_DESCRIPTOR);
    USETW(request->wValue, (STRING_DESC << 8) | 0x02);
    USETW(request->wIndex, 0x0409);
    USETW(request->wLength, 0x2A);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_ReceiveEp0Data((MMP_UBYTE *)otg_handle.ep0_data_buf, &data_length);
//    if (data_length) {
//        MMPF_USBH_DebugData((MMP_UBYTE *)otg_handle.ep0_data_buf, data_length);
//    }
    MMPF_USBH_HandleEp0Status(DIR_OUT);
#if 1
// FU_MUTE_CONTROL
    MMPF_USBH_GetFUCmd(GET_CUR_CMD, FU_MUTE_CONTROL, 1, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
// FU_VOLUME_CONTROL
    MMPF_USBH_GetFUCmd(GET_CUR_CMD, FU_VOLUME_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetFUCmd(GET_MIN_CMD, FU_VOLUME_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetFUCmd(GET_MAX_CMD, FU_VOLUME_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetFUCmd(GET_RES_CMD, FU_VOLUME_CONTROL, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
#endif

    /*
     * Set_Interface(3)
     */
    RTNA_DBG_Str(0, "= Set_Interface(3) =\r\n");
    request = (EP0_REQUEST *)otg_handle.ep0_setup_buf;
    USETB(request->bmRequestType, DIR_OUT | TYPE_STANDARD | RCPT_IF);
    USETB(request->bRequest, SET_INTERFACE);
    USETW(request->wValue, 0);
    USETW(request->wIndex, 3);
    USETW(request->wLength, 0);
    MMPF_USBH_SendEp0Setup(request);
    MMPF_USBH_HandleEp0Status(DIR_IN);

/////////////////////////////////////////

/////////////////////////////////////////
    #if 0
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_MMP_CMD16, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_SetEU1Cmd(SET_CUR_CMD, EU1_SET_MMP_CMD16, 16, STREAMVIEW_T01); // SEC TV options

    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_MMP_CMD16_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetEU1Cmd(GET_CUR_CMD, EU1_GET_MMP_CMD16_RESULT, 16, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);

    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_MMP_CMD16, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_SetEU1Cmd(SET_CUR_CMD, EU1_SET_MMP_CMD16, 16, STREAMVIEW_T05); // SEC TV options
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_MMP_CMD16, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_SetEU1Cmd(SET_CUR_CMD, EU1_SET_MMP_CMD16, 16, STREAMVIEW_T07); //set signal type
	
    MMPF_USBH_GetVSCmd(GET_CUR_CMD, VS_PROBE_CONTROL, 26, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_SetVSCmd(SET_CUR_CMD, VS_PROBE_CONTROL, 26, STREAMVIEW_T09);
    MMPF_USBH_GetVSCmd(GET_CUR_CMD, VS_PROBE_CONTROL, 26, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    MMPF_USBH_GetVSCmd(GET_MAX_CMD, VS_PROBE_CONTROL, 26, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf); //VS_PROBE_CONTROL => UVC_DMA_SIZE => transaction
    MMPF_USBH_GetVSCmd(GET_MIN_CMD, VS_PROBE_CONTROL, 26, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    
    MMPF_USBH_SetVSCmd(SET_CUR_CMD, VS_COMMIT_CONTROL, 26, STREAMVIEW_T13); //VS_COMMIT_CONTROL

    //MMPF_USBH_GetStreamReq(0);

    RTNA_DBG_Str(0, "= GET_LEN_CMD =\r\n");
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_SET_ISP, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    //RTNA_DBG_Str(0, "= SET_CUR_CMD =\r\n");
    MMPF_USBH_SetEU1Cmd(SET_CUR_CMD, EU1_SET_ISP, 8, STREAMVIEW_T18);
    //RTNA_DBG_Str(0, "= GET_LEN_CMD =\r\n");
    MMPF_USBH_GetEU1Cmd(GET_LEN_CMD, EU1_GET_ISP_RESULT, 2, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    //RTNA_DBG_Str(0, "= GET_CUR_CMD =\r\n");
    MMPF_USBH_GetEU1Cmd(GET_CUR_CMD, EU1_GET_ISP_RESULT, 8, &data_length, (MMP_UBYTE *)otg_handle.ep0_data_buf);
    
    //MMPF_USBH_SetEU1Cmd(SET_CUR_CMD, EU1_SET_ISP, 8, SET_IDR_FRAME_04);

    MMPF_USBH_GetStreamReq(1);
#if 0	
    MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_HOST_STP, MMPF_OS_FLAG_SET);      
#endif
    #endif
    
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_GetStreamReq(MMP_USHORT wMin)
{
    if(wMin > 10) {
        uvch_xfer_ctl.ulFTarget = 0xFFFFFFFF ; // It will stop by manual control or command
    }
    else if(wMin > 0) {
        //uvch_xfer_ctl.ulFTarget = (MMP_ULONG)1800 * wMin ; // minute to frame number
        uvch_xfer_ctl.ulFTarget = (MMP_ULONG)UVC_GET_FRAME_TARGET_CNT * wMin ; // minute to frame number
        //uvch_xfer_ctl.ulFTarget = 200 ; // minute to frame number
    }
    else {
        uvch_xfer_ctl.ulFTarget = 1; // At least 1 frame
    }
	
    //RTNA_DBG_Str(0, "[BEGIN] lock task to get frame count:");
    //RTNA_DBG_Long(0, uvch_xfer_ctl.ulFTarget);
    //RTNA_DBG_Str(0, "\r\n");
    MMPF_OS_SetFlags(USB_OP_Flag, USB_FLAG_UVC_HOST_REQ, MMPF_OS_FLAG_SET);      
    //MMPF_OS_AcquireSem(m_UVCHBulkInSemID, 0); //avoid UART debug not overlap menu tree
    
    return MMP_ERR_NONE;
}

MMP_UBYTE USB_UVCH_GetRemainFrameCount(void)
{
    SD_BUF_CTL    *sd_bctl = &sd_buf_ctl;
    return sd_bctl->ubFrameCnt;
}



#else
void USB_OTG_ISR(void)  { }

#endif //(SUPPORT_USB_HOST_FUNC)
#endif //(USB_EN)

