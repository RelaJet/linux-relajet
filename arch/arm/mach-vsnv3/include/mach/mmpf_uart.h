//==============================================================================
//
//  File        : mmpf_uart.h
//  Description : INCLUDE File for the Firmware UART Control Driver
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_UART_H_
#define _MMPF_UART_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define UART_RXINT_MODE_EN	    (1)
#define UART_RXINT_SEM_TIMEOUT  (0x10000)
#define UART_DMA_MODE_EN	    (1)

#if (OS_TYPE == OS_LINUX)
//#undef UART_RXINT_MODE_EN
//    #define UART_RXINT_MODE_EN  (0)
#undef UART_DMA_MODE_EN
    #define UART_DMA_MODE_EN    (0)
#endif

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_UART_ID
{
    MMPF_UART_ID_0 = 0,
    MMPF_UART_ID_1,
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_UART_ID_2,
    MMPF_UART_ID_3,
    #endif
    MMPF_UART_MAX_COUNT
} MMPF_UART_ID;

typedef enum _MMPF_UART_PADSET
{
    MMPF_UART_PADSET_0 = 0,
    MMPF_UART_PADSET_1,
    MMPF_UART_PADSET_2,
    MMPF_UART_PADSET_3,
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_UART_PADSET_4,
    #endif
    MMPF_UART_PADSED_MAX
} MMPF_UART_PADSET;

#if (OS_TYPE == OS_LINUX)
typedef enum _MMPF_UART_STAT {
    MMPF_UART_STAT_TXEMPTY = 0,
    MMPF_UART_STAT_RXEMPTY,
    MMPF_UART_STAT_TXUNDERTH,
    MMPF_UART_STAT_RXOVERTH
} MMPF_UART_STAT;

typedef enum _MMPF_UART_DIRECTION {
    MMPF_UART_DIRECTION_RX = 0x01,
    MMPF_UART_DIRECTION_TX = 0x02
} MMPF_UART_DIRECTION;

typedef enum _MMPF_UART_EVENT {
    MMPF_UART_EVENT_RXFIFO_OVERTH = 0,
    MMPF_UART_EVENT_TXFIFO_UNDERTH,
    MMPF_UART_EVENT_MAX
} MMPF_UART_EVENT;
#endif // (OS_TYPE == OS_LINUX)

typedef enum _MMPF_UART_DMAMODE
{
    MMPF_UART_RXDMA_RING = 0,
    MMPF_UART_TXDMA,
    MMPF_UART_RXDMA,
    MMPF_UART_DMA_MAX
} MMPF_UART_DMAMODE;

typedef enum _MMPF_UART_DMA_INT_MODE
{
    MMPF_UART_TXDMA_END_EN = 0,
    MMPF_UART_RXDMA_TH_EN,
    MMPF_UART_RXDMA_WRMEM_EN,
    MMPF_UART_RXDMA_DROPDATA_EN,
    MMPF_UART_DMA_INT_MAX
} MMPF_UART_DMA_INT_MODE;

typedef enum _MMPF_UART_PARITY
{
	MMPF_UART_PARITY_NONE = 0,
	MMPF_UART_PARITY_EVEN,
	MMPF_UART_PARITY_ODD,
	MMPF_UART_PARITY_FORCE0,
	MMPF_UART_PARITY_FORCE1,
	MMPF_UART_PARITY_MAX
}MMPF_UART_PARITY;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef void UartCallBackFunc(MMP_UBYTE num, ...);

#define AIT_UART_RTSCTS 0x01
typedef struct _MMPF_UART_ATTRIBUTE
{
    MMPF_UART_PADSET    padset;
    MMP_ULONG           ulMasterclk;
    MMP_ULONG           ulBaudrate;
    MMP_BOOL			bParityEn;
    MMP_BOOL			bFlowCtlEn;
    MMP_UBYTE			ubFlowCtlSelect; //0: low active , 1: high active
    MMPF_UART_PARITY    parity;
    MMP_ULONG 			mode;
} MMPF_UART_ATTRIBUTE;

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR MMPF_Uart_Init(void);
MMP_ERR	MMPF_Uart_Open(MMPF_UART_ID uart_id, MMPF_UART_ATTRIBUTE *uartattribute);
MMP_ERR MMPF_Uart_PadSet( MMPF_UART_ID uart_id, MMPF_UART_PADSET padset );
MMP_ERR	MMPF_Uart_Write(MMPF_UART_ID uart_id, char *pWrite_Str, MMP_ULONG ulLength);
MMP_ERR	MMPF_Uart_Close(MMPF_UART_ID uart_id);
//The following UART DMA are opened for upper layer using.
MMP_ERR MMPF_UART_UseTxDMA(MMPF_UART_ID uart_id, MMP_ULONG ulTxStartAddr, MMP_USHORT usTxTotalByte,  MMPF_UART_DMA_INT_MODE uart_int_mode, MMP_BOOL bEnableInt, UartCallBackFunc* callBackFunc);
MMP_ERR MMPF_UART_UseRxDMA(MMPF_UART_ID uart_id, MMP_BOOL bRingModeEn, MMP_ULONG ulRxStartAddr, MMP_ULONG ulRxEndAddr, MMP_ULONG ulRxLowBoundAddr,  MMPF_UART_DMA_INT_MODE uart_int_mode, 
							MMP_BOOL bEnableInt, UartCallBackFunc* callBackFunc, MMP_USHORT usRxThreshold, MMP_UBYTE ubRxTimeOut);
//The following UART DMA functions are used inside the driver, upper layer usually does not need to access them.
MMP_ERR MMPF_Uart_SwitchToDmaMode(MMPF_UART_ID uart_id, MMP_BOOL bEnable, MMP_UBYTE *ubUartCtl);
MMP_ERR MMPF_Uart_SetTxDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_ULONG ulTxStartAddr, MMP_USHORT usTxTotalByte);
MMP_ERR MMPF_Uart_SetRxDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_ULONG ulRxStartAddr, MMP_ULONG ulRxEndAddr, MMP_ULONG ulRxLowBoundAddr);
MMP_ERR MMPF_Uart_SetDmaInterruptMode (MMPF_UART_ID uart_id, MMPF_UART_DMA_INT_MODE uart_int_mode, MMP_BOOL bEnable, UartCallBackFunc* callBackFunc, MMP_USHORT usRxThreshold, MMP_UBYTE ubRxTimeOut);
MMP_ERR MMPF_Uart_EnableDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_BOOL bEnable);
// Functions for UART RX interrupt mode
#if (UART_RXINT_MODE_EN == 1)
MMP_BOOL MMPF_Uart_IsRxEnable(MMPF_UART_ID uart_id);
void MMPF_Uart_ConfigRx(MMPF_UART_ID uart_id, MMP_ULONG threshold);
MMP_ERR MMPF_Uart_EnableRx(MMPF_UART_ID uart_id, MMP_ULONG threshold, UartCallBackFunc *callBackFunc);
MMP_ERR MMPF_Uart_DisableRx(MMPF_UART_ID uart_id);
#endif

MMP_ULONG MMPF_Uart_TryRead(MMPF_UART_ID uart_id, MMP_UBYTE *buf, MMP_ULONG ulMaxReadLen);
MMP_BOOL MMPF_Uart_CheckState(MMPF_UART_ID uart_id, MMPF_UART_STAT State);
MMP_ERR MMPF_Uart_Reset(MMPF_UART_ID uart_id, MMPF_UART_DIRECTION direction);
MMP_ERR MMPF_Uart_SetInterruptEnable(MMPF_UART_ID uart_id, MMPF_UART_EVENT event, MMP_BOOL bEnable);

MMP_ERR MMPF_uart_open(MMPF_UART_ID uart_id, MMPF_UART_ATTRIBUTE *uartattribute);
#endif // _INCLUDES_H_
