//==============================================================================
//
//  File        : mmpf_uart.c
//  Description : Firmware UART Control Function
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include <linux/module.h>
#include "includes_fw.h"
#include "mmpf_uart.h"
#include "mmp_reg_pad.h"
#include "mmp_reg_gbl.h"
#include "mmp_reg_uart.h"
#include "lib_retina.h"
#include "mmpf_pio.h"

//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================

#if (OS_TYPE == OS_UCOSII)
#pragma arm section code = "MP3LowPower", rodata = "MP3LowPower", rwdata = "MP3LowPower",  zidata = "MP3LowPower"
#endif

static AITPS_US             m_pUS[MMPF_UART_MAX_COUNT] = {
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    &(AITC_BASE_UARTB->UART_0),
    &(AITC_BASE_UARTB->UART_1),
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    &(AITC_BASE_UARTB->UART_2),
    &(AITC_BASE_UARTB->UART_3),
    #endif
    #endif
};
static MMPF_OS_SEMID        m_UartSemID[MMPF_UART_MAX_COUNT];
static MMP_BOOL	            m_UartInDmaMode[MMPF_UART_MAX_COUNT];

#if (UART_RXINT_MODE_EN == 1)
static MMP_BOOL	            m_UartRxEnable[MMPF_UART_MAX_COUNT];
static UartCallBackFunc     *m_UartRx_CallBack[MMPF_UART_MAX_COUNT];
#define RX_ENTER_SIGNAL  	(13)
#define RX_SENSITIVE  		(100)
#define RX_QUEUE_SIZE		(128)
#endif

#if (UART_DMA_MODE_EN == 1)
static UartCallBackFunc     *m_UartDma_CallBack[MMPF_UART_MAX_COUNT];
#endif

#if (OS_TYPE == OS_UCOSII)
#pragma arm section code, rodata, rwdata, zidata
#endif

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : Return the AITPS_US of given id
//  Description : Helper function to get correct pTC from ID
//  Note        : Because timer hardware is split into 2 group. The ID
//                to pTC is handled here 
//  Return      : 
//------------------------------------------------------------------------------
//static __inline AITPS_US GetpUS(MMPF_UART_ID id)
AITPS_US GetpUS(const int mmpf_uart_id)
{
	//AITPS_UART  pUART = AITC_BASE_UART;
	AITPS_UART	pUART  = AITC_BASE_UART_BASE;

    switch(mmpf_uart_id) {
    case MMPF_UART_ID_0:
        return &(pUART->US0);
    case MMPF_UART_ID_1:
        return &(pUART->US1);
    #if (CHIP == MERCURY ||CHIP == MCR_V2 )
    case MMPF_UART_ID_2:
        return &(pUART->US2);
    case MMPF_UART_ID_3:
        return &(pUART->US3);
    #endif
    default:
        RTNA_DBG_Str(0, "Invalid uart ID !!\r\n");
        return 0;
    }
}

#if 0
void ____Uart_DMA_Function____(){ruturn;} //dummy
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_SwitchToDmaMode
//  Description :
//  Note        : This is the 1st step of UART using DMA mode
//------------------------------------------------------------------------------
/** @brief This function set the UART device from normal mode to DMA mode.

This function set the UART device from normal mode to DMA mode.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] bEnable stands for enable switch to DMA mode or back from DMA mode.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Uart_SwitchToDmaMode(MMPF_UART_ID uart_id, MMP_BOOL bEnable, MMP_UBYTE* ubUartCtl)
{
	#if (UART_DMA_MODE_EN == 1)
    AITPS_US    pUS = m_pUS[uart_id];

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    if (uart_id != MMPF_UART_ID_0) {
        RTNA_DBG_Str(0, "No DMA mode on Uart1,2,3\r\n");
        return MMP_ERR_NONE;    
    }
    #endif
     
	if (bEnable == MMP_TRUE) 
	{
		// To use DMA interrupt mode normanlly, we should turn off the interrupt we used in UART normal mode
		#if (UART_RXINT_MODE_EN == 1)
		pUS->US_IER &= (~US_RX_FIFO_OVER_THRES);
		#endif
		
		*ubUartCtl = (MMP_UBYTE)(pUS->US_CR & (~US_CLEAN_CTL0));

		pUS->US_CR |= US_TXEN;      // Clean FIFO first. //EROY CHECK
		MMPF_OS_Sleep(10);
		pUS->US_CR &= US_CLEAN_CTL0;// Switch to dma mode,
		m_UartInDmaMode[uart_id] = MMP_TRUE;
	}
	else {

    	pUS->US_CR &= US_CLEAN_CTL0; //Switch to Normal mode, //EROY CHECK
    	pUS->US_CR |= *ubUartCtl;
    	#if (UART_RXINT_MODE_EN == 1)
		pUS->US_IER = US_RX_FIFO_OVER_THRES;
		#endif
    	m_UartInDmaMode[uart_id] = MMP_FALSE;
	}
	#else
	RTNA_DBG_Str(0, "No DMA mode: UART_DMA_MODE_EN = 0 !!\r\n");
	#endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_SetTxDmaMode
//  Description :
//  Note        : This is the 2nd step of UART using DMA TX mode
//------------------------------------------------------------------------------
/** @brief This function set the parameters using by UART DMA TX mode.

This function set the parameters using by UART DMA Tx mode.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] uart_dma_mode indicate which DMA mode to be used, please refer the data structure MMPF_UART_DMAMODE.
@param[in] ulTxStartAddr indicate the Tx DMA start address.
@param[in] usTxTotalByte indicate number of bytes would be sent (start from start address).
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Uart_SetTxDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_ULONG ulTxStartAddr, MMP_USHORT usTxTotalByte)
{
	#if (UART_DMA_MODE_EN == 1)
    AITPS_US    pUS = m_pUS[uart_id];

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    if (uart_id != MMPF_UART_ID_0) {
        RTNA_DBG_Str(0, "No DMA mode on Uart1,2,3\r\n");
        return MMP_ERR_NONE;
    }
    #endif
	
	if (uart_dma_mode == MMPF_UART_TXDMA) {
		pUS->US_TXDMA_START_ADDR = ulTxStartAddr;
		pUS->US_TXDMA_TOTAL_BYTE = usTxTotalByte;
	}
	else {
		return MMP_UART_ERR_PARAMETER;
	}
	#else
	RTNA_DBG_Str(0, "No DMA mode: UART_DMA_MODE_EN = 0 !!\r\n");
	#endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_SetRxDmaMode
//  Description :
//  Note        : This is the 2nd step of UART using DMA RX mode
//------------------------------------------------------------------------------
/** @brief This function set the parameters using by UART DMA RX mode.

This function set the parameters using by UART DMA Tx mode.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] uart_dma_mode indicate which DMA mode to be used, please refer the data structure MMPF_UART_DMAMODE.
@param[in] ulRxStartAddr indicate the RX DMA start address.
@param[in] ulRxEndAddr indicate the RX DMA End address.
@param[in] ulRxLowBoundAddr indicate the RX lower bound address (Using by RX DMA Ring Mode). 
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Uart_SetRxDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_ULONG ulRxStartAddr, MMP_ULONG ulRxEndAddr, MMP_ULONG ulRxLowBoundAddr)
{
	#if (UART_DMA_MODE_EN == 1)
    AITPS_US    pUS = m_pUS[uart_id];

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    if (uart_id != MMPF_UART_ID_0) {
        RTNA_DBG_Str(0, "No DMA mode on Uart1,2,3\r\n");
        return MMP_ERR_NONE;
    }
    #endif
	
	if ((uart_dma_mode == MMPF_UART_RXDMA) || (uart_dma_mode == MMPF_UART_RXDMA_RING)) {

		pUS->US_RXDMA_START_ADDR = ulRxStartAddr;
		
		if(uart_dma_mode == MMPF_UART_RXDMA_RING) {
			// Note: for ring mode, RX_DMA_END_ADDR must be 8 byte alignment
			pUS->US_RXDMA_END_ADDR = ulRxEndAddr;
			pUS->US_RXDMA_LB_ADDR  = ulRxLowBoundAddr;
		}	
	}
	else {
		return MMP_UART_ERR_PARAMETER;
	}
	#else
	RTNA_DBG_Str(0, "No DMA mode: UART_DMA_MODE_EN = 0 !!\r\n");
	#endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_SetDmaInterruptMode
//  Description :
//  Note        : This is the step of UART using DMA interrupt mode settings. (This step can be done betwee step2 and step3)
//------------------------------------------------------------------------------
/** @brief This function sets the UART DMA interrupt mode.

This function sets the UART DMA interrupt mode.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] uart_int_mode indicate which DMA interrupt mode to be used, please refer the data structure MMPF_UART_DMA_INT_MODE.
@param[in] bEnable stands for "enable the related interrupt mode or not".
@param[in] callBackFunc is used as interrupt handler.
@param[in] usRxThreshold is used by RX DMA mode, when dma count reaches the Threshold and the related interrupt occurs. 
@param[in] ubRxTimeOut is used in RX interrupt mode (MMPF_UART_RXDMA_WRMEM_EN & MMPF_UART_RXDMA_DROPDATA_EN)
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Uart_SetDmaInterruptMode(MMPF_UART_ID uart_id, MMPF_UART_DMA_INT_MODE uart_int_mode, MMP_BOOL bEnable, UartCallBackFunc* callBackFunc, MMP_USHORT usRxThreshold, MMP_UBYTE ubRxTimeOut)
{
	#if (UART_DMA_MODE_EN == 1)
    AITPS_US    pUS = m_pUS[uart_id];

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    if (uart_id != MMPF_UART_ID_0) {
        RTNA_DBG_Str(0, "No DMA mode on Uart1,2,3\r\n");
        return MMP_ERR_NONE;
    }
    #endif

	if (bEnable == MMP_TRUE) 
	{
		switch (uart_int_mode) {
			case MMPF_UART_TXDMA_END_EN:
				pUS->US_IER |= US_TXDMA_END_EN;
				break;
			case MMPF_UART_RXDMA_TH_EN:
				pUS->US_RXDMA_TOTAL_THR = usRxThreshold;
				pUS->US_IER |= US_RXDMA_TH_EN;
				break;
			case MMPF_UART_RXDMA_WRMEM_EN:
				pUS->US_IER |= US_RXDMA_WRMEM_EN;
				pUS->US_BRGR |= (ubRxTimeOut << US_RX_TIMEOUT_SHIFTER);
				pUS->US_CR |= US_RX_TIMEOUT_EN;
				break;
			case MMPF_UART_RXDMA_DROPDATA_EN:
				pUS->US_IER |= US_RXDMA_DROPDATA_EN;
				pUS->US_BRGR |= (ubRxTimeOut << US_RX_TIMEOUT_SHIFTER);
				pUS->US_CR |= US_RX_TIMEOUT_EN;
				break;
			default:
				return MMP_UART_ERR_PARAMETER;
				break;
		}
		
		if (callBackFunc != NULL) {	
			m_UartDma_CallBack[uart_id] = callBackFunc;
		}
	}
	else 
	{
		switch (uart_int_mode) {
			case MMPF_UART_TXDMA_END_EN:
				pUS->US_IER &= (~US_TXDMA_END_EN);
				break;
			case MMPF_UART_RXDMA_TH_EN:
				pUS->US_RXDMA_TOTAL_THR = 0;
				pUS->US_IER &= (~US_RXDMA_TH_EN);
				break;
			case MMPF_UART_RXDMA_WRMEM_EN:
				pUS->US_IER &= (~US_RXDMA_WRMEM_EN);
				pUS->US_CR &= (~US_RX_TIMEOUT_EN);
				pUS->US_BRGR &= (~US_RX_TIMEOUT_MASK);
				break;
			case MMPF_UART_RXDMA_DROPDATA_EN:
				pUS->US_IER &= (~US_RXDMA_DROPDATA_EN);
				pUS->US_CR &= (~US_RX_TIMEOUT_EN);
				pUS->US_BRGR &= (~US_RX_TIMEOUT_MASK);
				break;
			default:
				return MMP_UART_ERR_PARAMETER;
				break;
		}	
		m_UartDma_CallBack[uart_id] = NULL;
	}
	
	#else 
	RTNA_DBG_Str(0, "No DMA mode: UART_DMA_MODE_EN = 0 !!\r\n");
	#endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_EnableDmaMode
//  Description :
//  Note        : This function is the 3rd step of UART DMA settings.
//------------------------------------------------------------------------------
/** @brief This function is used to enable or disable UART DMA mode.

This function is used to enable or disable UART DMA mode.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] uart_dma_mode indicate which DMA mode to be used, please refer the data structure MMPF_UART_DMAMODE.
@param[in] bEnable stands for "enable the related mode or not".
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Uart_EnableDmaMode(MMPF_UART_ID uart_id, MMPF_UART_DMAMODE uart_dma_mode, MMP_BOOL bEnable)
{
	#if (UART_DMA_MODE_EN == 1)
    AITPS_US    pUS = m_pUS[uart_id];

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    if (uart_id != MMPF_UART_ID_0) {
        RTNA_DBG_Str(0, "No DMA mode on Uart1,2,3\r\n");
        return MMP_ERR_NONE;
    }
    #endif

	if (bEnable == MMP_TRUE) 
	{
		switch (uart_dma_mode) {
			case MMPF_UART_RXDMA_RING:				
				pUS->US_CR |= (US_RXDMA_RING_EN | US_RXDMA_EN) ;
				pUS->US_CR |= US_RXDMA_START_FLAG | (US_RXDMA_RING_EN | US_RXDMA_EN);
				pUS->US_CR |= US_RSTRX | (US_RXDMA_RING_EN | US_RXDMA_EN);  //Add RXDMA_EN again, this is because of 0x6403 can not read issue
				pUS->US_CR |= US_RXEN | (US_RXDMA_RING_EN | US_RXDMA_EN);	//Add RXDMA_EN again, this is because of 0x6403 can not read issue
				break;
    		case MMPF_UART_TXDMA:
    			pUS->US_CR |= US_TXDMA_EN;
    			pUS->US_CR |= US_RSTTX | US_TXDMA_EN; //Reset TX, for the TXDMA can not read bug, we add TX_DMA_EN again
    			pUS->US_CR |= US_TXEN | US_TXDMA_EN;  //Enable TX, for the TXDMA can not read bug, we add TX_DMA_EN again
    			break;
    		case MMPF_UART_RXDMA:
    			pUS->US_CR |= US_RXDMA_EN;
    			pUS->US_CR |= US_RXDMA_START_FLAG | US_RXDMA_EN;//Add RXDMA_EN again, this is because of 0x6403 can not read issue
    			pUS->US_CR |= US_RSTRX | US_RXDMA_EN;           //Add RXDMA_EN again, this is because of 0x6403 can not read issue
				pUS->US_CR |= US_RXEN | US_RXDMA_EN;	        //Add RXDMA_EN again, this is because of 0x6403 can not read issue
    			break;
    		default:
    			return MMP_UART_ERR_PARAMETER;
    			break;
		}
	}
	else {
		switch (uart_dma_mode) {
			case MMPF_UART_RXDMA_RING:
				pUS->US_CR &= (~(US_RXDMA_RING_EN | US_RXDMA_EN | US_RXDMA_START_FLAG));
				break;
    		case MMPF_UART_TXDMA:
    			pUS->US_CR &= (~US_TXDMA_EN);
    			break;
    		case MMPF_UART_RXDMA:
    			pUS->US_CR &= (~(US_RXDMA_EN | US_RXDMA_START_FLAG));
    			break;
    		default:
    			return MMP_UART_ERR_PARAMETER;
    			break;
		}
	}
	
	#else 
	RTNA_DBG_Str(0, "No DMA mode: UART_DMA_MODE_EN = 0 !!\r\n");
	#endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_UART_UseTxDMA
//  Description :
//  Note        : UART tx_dma mode API, the only one API for upper layer.
//------------------------------------------------------------------------------
/** @brief UART tx_dma mode API, the only one API for upper layer.

UART tx_dma mode API, the only one API for upper layer.
@param[in] uart_id indicate which UART device, please refer the data structure of MMPF_UART_ID
@param[in] ulTxStartAddr indicate the Tx DMA start address.
@param[in] usTxTotalByte indicate number of bytes would be sent (start from start address).
@param[in] uart_int_mode indicate which DMA interrupt mode to be used, please refer the data structure MMPF_UART_DMA_INT_MODE.
@param[in] bEnableInt indicate enable interrup mode or not
@param[in] callBackFunc is the callback function when interrupt occur.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_UART_UseTxDMA(MMPF_UART_ID uart_id, MMP_ULONG ulTxStartAddr, MMP_USHORT usTxTotalByte,  
                          MMPF_UART_DMA_INT_MODE uart_int_mode, MMP_BOOL bEnableInt, UartCallBackFunc* callBackFunc)
{
	MMP_ERR     status = MMP_ERR_NONE;
	MMP_UBYTE   ubUartCtl;
	MMP_ERR     SemStatus = 0xFF;
	
	SemStatus = MMPF_OS_AcquireSem(m_UartSemID[uart_id], UART_RXINT_SEM_TIMEOUT);
	
	if (SemStatus == 0) {
	
		status |= MMPF_Uart_SwitchToDmaMode(uart_id, MMP_TRUE, &ubUartCtl);
		status |= MMPF_Uart_SetTxDmaMode(uart_id, MMPF_UART_TXDMA, ulTxStartAddr, usTxTotalByte);
		status |= MMPF_Uart_SetDmaInterruptMode(uart_id, uart_int_mode, bEnableInt, callBackFunc, 0, 0); 
	    status |= MMPF_Uart_EnableDmaMode(uart_id, MMPF_UART_TXDMA, MMP_TRUE);
	    
	    if(bEnableInt == MMP_FALSE) {
	    	status |= MMPF_Uart_SwitchToDmaMode(uart_id, MMP_FALSE, &ubUartCtl);
	    }
	    
	    MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
    }
    else {
    	RTNA_DBG_PrintLong(0, (MMP_ULONG)SemStatus);
    	return MMP_UART_SYSTEM_ERR;
    }
    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_UART_UseRxDMA
//  Description :
//  Note        : UART rx_dma mode API, the only one RX API for upper layer.
//------------------------------------------------------------------------------
/** @brief UART rx_dma mode API, the only one RX API for upper layer.

UART rx_dma mode API, the only one RX API for upper layer.
@param[in] uart_id indicates which UART device, please refer the data structure of MMPF_UART_ID
@param[in] bRingModeEn indicates to use ring mode or not.
@param[in] ulRxStartAddr indicate the RX DMA start address.
@param[in] ulRxEndAddr indicate the RX DMA End address.
@param[in] ulRxLowBoundAddr indicate the RX lower bound address (Using by RX DMA Ring Mode). 
@param[in] uart_int_mode indicate which DMA interrupt mode to be used, please refer the data structure MMPF_UART_DMA_INT_MODE.
@param[in] bEnableInt indicate enable interrup mode or not
@param[in] callBackFunc is the callback function when interrupt occur.
@param[in] usRxThreshold which is used in ring mode.
@param[in] ubRxTimeOut is used in RX interrupt mode (MMPF_UART_RXDMA_WRMEM_EN & MMPF_UART_RXDMA_DROPDATA_EN)
@return It reports the status of the operation.
*/
MMP_ERR MMPF_UART_UseRxDMA(MMPF_UART_ID uart_id, MMP_BOOL bRingModeEn, MMP_ULONG ulRxStartAddr, MMP_ULONG ulRxEndAddr, MMP_ULONG ulRxLowBoundAddr,  MMPF_UART_DMA_INT_MODE uart_int_mode, 
							MMP_BOOL bEnableInt, UartCallBackFunc* callBackFunc, MMP_USHORT usRxThreshold, MMP_UBYTE ubRxTimeOut)
{
	MMP_ERR     status = MMP_ERR_NONE;
	MMPF_UART_DMAMODE rx_dma_mode = MMPF_UART_RXDMA;
	MMP_UBYTE   ubUartCtl;
	MMP_ERR     SemStatus = 0x0;
	
	if(bRingModeEn) {
		rx_dma_mode = MMPF_UART_RXDMA_RING;
	}
	
	SemStatus = MMPF_OS_AcquireSem(m_UartSemID[uart_id], UART_RXINT_SEM_TIMEOUT);
	
	if(SemStatus == 0) {
	
		status |= MMPF_Uart_SwitchToDmaMode(uart_id, MMP_TRUE, &ubUartCtl);
		status |= MMPF_Uart_SetRxDmaMode(uart_id, rx_dma_mode, ulRxStartAddr, ulRxEndAddr, ulRxLowBoundAddr);
		status |= MMPF_Uart_SetDmaInterruptMode(uart_id, uart_int_mode, bEnableInt, callBackFunc, usRxThreshold, ubRxTimeOut); 
		status |= MMPF_Uart_EnableDmaMode(uart_id, rx_dma_mode, MMP_TRUE);
		
		if(bEnableInt == MMP_FALSE) {
	    	status |= MMPF_Uart_SwitchToDmaMode(uart_id, MMP_FALSE, &ubUartCtl);
	    }
	}
	else {
    	RTNA_DBG_PrintLong(0, (MMP_ULONG)SemStatus);
    	return MMP_UART_SYSTEM_ERR;
    }
	
	return status;
}

#if 0
void ____Uart_General_Function____(){ruturn;} //dummy
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_UART_ISR
//  Description :
//  Note        : for uart debug or uart dma interrupt mode, the uart resource protect is done by
//				  MMPF_UART_UseTxDMA and MMPF_UART_UseRxDMA
//------------------------------------------------------------------------------
void MMPF_UART_ISR(void)
{
    MMP_ULONG   intsrc, i;
    #if (UART_RXINT_MODE_EN == 1)
    MMP_ULONG   len;
    #endif
    AITPS_US    pUS;

    for (i = 0; i < MMPF_UART_MAX_COUNT; i++) 
    {
        pUS = m_pUS[i];
        intsrc = pUS->US_ISR & pUS->US_IER;
        pUS->US_ISR = intsrc;

        #if (UART_RXINT_MODE_EN == 1)
        if ((m_UartInDmaMode[i] == MMP_FALSE) && (intsrc & US_RX_FIFO_OVER_THRES)) {

            len = pUS->US_FSR & US_RX_FIFO_UNRD_MASK;
                       
            if ((m_UartRx_CallBack[i] != NULL) && len)
                (*m_UartRx_CallBack[i])((MMP_UBYTE)len, &(pUS->US_RXPR));
            break;
        }
        else if (intsrc & US_RX_PARITY_ERR) {
            RTNA_DBG_Str(0, "Error: US_RX_PARITY_ERR\r\n");
            break;
        }
        #endif

        #if (UART_DMA_MODE_EN == 1)
        if (intsrc & US_TXDMA_END_EN) {
            if ((m_UartDma_CallBack[i] != NULL) && (m_UartInDmaMode[i] == MMP_TRUE)) {
                (*m_UartDma_CallBack[i]) (MMPF_UART_TXDMA_END_EN);
            }	
        }
        else if (intsrc & US_RXDMA_TH_EN) {
            if ((m_UartDma_CallBack[i] != NULL) && (m_UartInDmaMode[i] == MMP_TRUE)) {
                (*m_UartDma_CallBack[i]) (MMPF_UART_RXDMA_TH_EN);	
            }
        }
        else if (intsrc & US_RXDMA_WRMEM_EN) {
            if ((m_UartDma_CallBack[i] != NULL) && (m_UartInDmaMode[i] == MMP_TRUE)){
                (*m_UartDma_CallBack[i]) (MMPF_UART_RXDMA_WRMEM_EN);
            }
        }
        else if (intsrc & US_RXDMA_DROPDATA_EN) {
            if ((m_UartDma_CallBack[i] != NULL) && (m_UartInDmaMode[i] == MMP_TRUE)){
                (*m_UartDma_CallBack[i]) (MMPF_UART_RXDMA_DROPDATA_EN);
            }
        }
        #endif
    }
}

#if (OS_TYPE == OS_UCOSII)
#pragma arm section code = "MP3LowPower", rodata = "MP3LowPower", rwdata = "MP3LowPower",  zidata = "MP3LowPower"
#endif
//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Init
//  Description : Initial the semaphore and call-back functions.
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_Init(void)
{
	static MMP_BOOL bUartInitFlag = MMP_FALSE;
    #if (OS_TYPE == OS_UCOSII)
	AITPS_AIC   pAIC = AITC_BASE_AIC;
    #endif
	MMP_USHORT 	i = 0;
	
	if (!bUartInitFlag) 
	{
        #if (OS_TYPE == OS_UCOSII)
		RTNA_AIC_Open(pAIC, AIC_SRC_UART,   uart_isr_a,
	                    AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
	    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_UART);
        #endif
	
		for (i = 0; i < MMPF_UART_MAX_COUNT; i++) 
		{
			m_UartSemID[i]          = MMPF_OS_CreateSem(1);
			m_UartInDmaMode[i]      = MMP_FALSE;
			#if (UART_RXINT_MODE_EN == 1)
			m_UartRxEnable[i]       = MMP_FALSE;
			m_UartRx_CallBack[i]    = NULL;
			#endif
			#if (UART_DMA_MODE_EN == 1)
			m_UartDma_CallBack[i]   = NULL;
            #endif
		}

		bUartInitFlag = MMP_TRUE;
	}
	
	return	MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_PadSet
//  Description :
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_PadSet( MMPF_UART_ID uart_id, MMPF_UART_PADSET padset )
{
	AITPS_GBL   pGBL = AITC_BASE_GBL;

#if (CHIP == VSN_V3)
        if (uart_id == MMPF_UART_ID_0) {
            // Clean PAD config first.
            pGBL->GBL_IO_CTL1 &= ~(GBL_UART_TX_PAD0 | GBL_UART_TX_PAD1);
            pGBL->GBL_IO_CTL3 &= ~(GBL_UART_RX_PAD0 | GBL_UART_RX_PAD1 |
                                    GBL_UART_RX_PAD2 | GBL_UART_TX_PAD2);

        switch(padset) {
            case MMPF_UART_PADSET_0:    // use AGPIO0 as uart TX
                pGBL->GBL_IO_CTL1 |= GBL_UART_TX_PAD0;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD0;
                break;
            case MMPF_UART_PADSET_1:    // use PSNR_D8 as uart TX
                pGBL->GBL_IO_CTL1 |= GBL_UART_TX_PAD1;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD1;
                break;
            case MMPF_UART_PADSET_2:    // use BGPIO14 as uart tx
                pGBL->GBL_IO_CTL3 |= GBL_UART_TX_PAD2;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD2;
                break;
            default:
            return MMP_UART_ERR_PARAMETER;
            }
        } else {
        return MMP_UART_ERR_PARAMETER;
        }
#endif
#if (CHIP == MCR_V2)
        switch(uart_id) {
        case 0:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART0_PAD_MASK);
        pGBL->GBL_UART_PAD_CFG |= GBL_UART0_PAD(padset);
            break;
        case 1:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART1_PAD_MASK);
        pGBL->GBL_UART_PAD_CFG |= GBL_UART1_PAD(padset);
            break;
        case 2:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART2_PAD_MASK);
        pGBL->GBL_UART_PAD_CFG |= GBL_UART2_PAD(padset);
            break;
        case 3:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART3_PAD_MASK);
        pGBL->GBL_UART_PAD_CFG |= GBL_UART3_PAD(padset);
            break;
        default:
        return MMP_UART_ERR_PARAMETER;;
    }
#endif
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Open
//  Description :
//  Note        :
//------------------------------------------------------------------------------
asmlinkage int printk(const char *fmt, ...);
MMP_ERR MMPF_Uart_Open(MMPF_UART_ID uart_id, MMPF_UART_ATTRIBUTE *uartattribute)
{
	MMP_ERR     status = MMP_ERR_NONE;
    AITPS_US    pUS;

	// status |= MMPF_Uart_Init();
	pUS = m_pUS[uart_id];

 //   printk( KERN_INFO"uartattribute->ulMasterclk = %d (%X)\n", uartattribute->ulMasterclk, ((uartattribute->ulMasterclk / uartattribute->ulBaudrate) + 1) >> 1 );
//    printk( KERN_INFO"pUS->US_BRGR  = %d (%X)\n", pUS->US_BRGR,((uartattribute->ulMasterclk
//										/ uartattribute->ulBaudrate) + 1) >> 1  );

    // if (MMPF_OS_AcquireSem(m_UartSemID[uart_id], UART_RXINT_SEM_TIMEOUT) == 0)
    if( 1 )
    {
        // Set output pad set
        if( MMPF_Uart_PadSet( uart_id, uartattribute->padset ) != MMP_ERR_NONE )
            goto L_INVAL_UART;

	    // Disable interrupts
	    pUS->US_IER = 0;
	    
	    // Reset receiver and transmitter
	    pUS->US_CR = US_RSTRX | US_RSTTX | US_RXDIS | US_TXDIS;
 //   printk( KERN_INFO"MMPF_Uart_Open 2\n");	    
	    // Define the baud rate divisor register
		pUS->US_BRGR = ((uartattribute->ulMasterclk
										/ uartattribute->ulBaudrate) + 1) >> 1;								
	    
	    // Define the UART mode 8-N-1
	    pUS->US_CR = US_ASYNC_MODE | US_TXEN | US_RXEN;
		
		#if (UART_RXINT_MODE_EN == 1)
	    pUS->US_FTHR &= ~US_RX_FIFO_THRES_MASK;
	    pUS->US_FTHR |= US_RX_FIFO_THRES(1);
	    pUS->US_IER = US_RX_FIFO_OVER_THRES;
		#endif
		
		if(uartattribute->bParityEn == MMP_TRUE) 
		{
			pUS->US_CR &= (~US_PAR_DIS);
			pUS->US_CR &= (~US_PAR_MASK);
			
			switch(uartattribute->parity) {
				case MMPF_UART_PARITY_EVEN:
					pUS->US_CR |= US_PAR_EVEN;
					break;
				case MMPF_UART_PARITY_ODD:
					pUS->US_CR |= US_PAR_ODD;
					break;
				case MMPF_UART_PARITY_FORCE0:
					pUS->US_CR |= US_PAR_0;
					break;
				case MMPF_UART_PARITY_FORCE1:
					pUS->US_CR |= US_PAR_1;
					break;
				default:
					break;
			} 
		}
		else {
			pUS->US_CR |= US_PAR_DIS;
		}
		
        if (uartattribute->bFlowCtlEn == MMP_TRUE) {
		    if (uartattribute->ubFlowCtlSelect) {
		    	pUS->US_CR |= US_RTSCTS_ACTIVE_H;
		    }
		    else {
		    	pUS->US_CR &= (~US_RTSCTS_ACTIVE_H); 
		    }

			pUS->US_CR |= US_RTSCTS_EN;
		}
		else {
			pUS->US_CR &= (~US_RTSCTS_EN);
		}

		MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
	}
	else {
		return MMP_UART_SYSTEM_ERR;
	}

	return	status;

L_INVAL_UART:
    MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
    RTNA_DBG_Str(0, "Un-supported uart setting !!\r\n");
    return MMP_UART_ERR_PARAMETER;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Open
//  Description :
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_uart_open(MMPF_UART_ID uart_id, MMPF_UART_ATTRIBUTE *uartattribute)
{
	AITPS_GBL   pGBL = AITC_BASE_GBL;
	MMP_ERR     status = MMP_ERR_NONE;
    AITPS_US    pUS;

	status |= MMPF_Uart_Init();
	pUS = m_pUS[uart_id];

    if (MMPF_OS_AcquireSem(m_UartSemID[uart_id], UART_RXINT_SEM_TIMEOUT) == 0) 
    {
	    #if (CHIP == VSN_V3)
        if (uart_id == MMPF_UART_ID_0) {
            // Clean PAD config first.
            pGBL->GBL_IO_CTL1 &= ~(GBL_UART_TX_PAD0 | GBL_UART_TX_PAD1);
            pGBL->GBL_IO_CTL3 &= ~(GBL_UART_RX_PAD0 | GBL_UART_RX_PAD1 |
                                    GBL_UART_RX_PAD2 | GBL_UART_TX_PAD2);

            switch(uartattribute->padset) {
            case MMPF_UART_PADSET_0:    // use AGPIO0 as uart TX
                pGBL->GBL_IO_CTL1 |= GBL_UART_TX_PAD0;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD0;
                break;
            case MMPF_UART_PADSET_1:    // use PSNR_D8 as uart TX
                pGBL->GBL_IO_CTL1 |= GBL_UART_TX_PAD1;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD1;
                break;
            case MMPF_UART_PADSET_2:    // use BGPIO14 as uart tx
                pGBL->GBL_IO_CTL3 |= GBL_UART_TX_PAD2;
                pGBL->GBL_IO_CTL3 |= GBL_UART_RX_PAD2;
                break;
            default:
                goto L_INVAL_UART;
            }
        } else {
            goto L_INVAL_UART;
        }
	    #endif
	    #if (CHIP == MERCURY)
	    switch(uart_id) {
	    case MMPF_UART_ID_0:
	        // Clean PAD config first
	        pGBL->GBL_IO_CTL3 &= ~(GBL_UART0_TX_PAD_MASK);
	        pGBL->GBL_IO_CTL3 |= GBL_UART0_TX_PAD(padset);
	        pGBL->GBL_LCD_BYPASS_CTL1 &= ~(GBL_UART0_RX_PAD_MASK);
	        pGBL->GBL_LCD_BYPASS_CTL1 |= GBL_UART0_RX_PAD(padset);
	        break;
	    case MMPF_UART_ID_1:
	        pGBL->GBL_UART1_PAD_CTL &= ~(GBL_UART1_TX_PAD_MASK|GBL_UART1_RX_PAD_MASK);
	        pGBL->GBL_UART1_PAD_CTL |= GBL_UART1_TX_PAD(padset);
	        pGBL->GBL_UART1_PAD_CTL |= GBL_UART1_RX_PAD(padset);
	        break;
	    case MMPF_UART_ID_2:
	        pGBL->GBL_UART2_PAD_CTL &= ~(GBL_UART2_TX_PAD_MASK|GBL_UART2_RX_PAD_MASK);
	        pGBL->GBL_UART2_PAD_CTL |= GBL_UART2_TX_PAD(padset);
	        pGBL->GBL_UART2_PAD_CTL |= GBL_UART2_RX_PAD(padset);
	        break;
	    case MMPF_UART_ID_3:
	        pGBL->GBL_UART3_PAD_CTL &= ~(GBL_UART3_TX_PAD_MASK|GBL_UART3_RX_PAD_MASK);
	        pGBL->GBL_UART3_PAD_CTL |= GBL_UART3_TX_PAD(padset);
	        pGBL->GBL_UART3_PAD_CTL |= GBL_UART3_RX_PAD(padset);
	        break;
	    default:
            goto L_INVAL_UART;
	    }
        #endif
        #if (CHIP == MCR_V2)
        switch(uart_id) {
        case 0:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART0_PAD_MASK);
            pGBL->GBL_UART_PAD_CFG |= GBL_UART0_PAD(uartattribute->padset);
            break;
        case 1:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART1_PAD_MASK);
            pGBL->GBL_UART_PAD_CFG |= GBL_UART1_PAD(uartattribute->padset);
            break;
        case 2:
            pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART2_PAD_MASK);
            pGBL->GBL_UART_PAD_CFG |= GBL_UART2_PAD(uartattribute->padset);
            break;
        case 3:
			if ((pGBL->GBL_UART_PAD_CFG & GBL_UART3_PAD_MASK) != GBL_UART3_PAD(uartattribute->padset))
			{ //if condition is for avoid glitches when ait configure the io pads to uart pads
				pGBL->GBL_UART_PAD_CFG &= ~(GBL_UART3_PAD_MASK);
				pGBL->GBL_UART_PAD_CFG |= GBL_UART3_PAD(uartattribute->padset);
			}
			if(uartattribute->padset==0)
			{
				MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO60,0,MMPF_OS_LOCK_CTX_BYPASS); //TX
				MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO61,0,MMPF_OS_LOCK_CTX_BYPASS); //RX
				//if(uartattribute->mode & AIT_UART_RTSCTS)
				if(uartattribute->bFlowCtlEn == MMP_TRUE)
				{
					/*MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO58,1,MMPF_OS_LOCK_CTX_BYPASS); //CTS
					MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO59,1,MMPF_OS_LOCK_CTX_BYPASS); //RTS
					MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO58,1,MMPF_OS_LOCK_CTX_BYPASS); //CTS
					MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO59,1,MMPF_OS_LOCK_CTX_BYPASS); //RTS
					MMPF_PIO_SetData(MMPF_PIO_REG_GPIO58,0,MMPF_OS_LOCK_CTX_BYPASS); //CTS
					MMPF_PIO_SetData(MMPF_PIO_REG_GPIO59,0,MMPF_OS_LOCK_CTX_BYPASS); //RTS
					udelay(1000);*/
					MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO58,0,MMPF_OS_LOCK_CTX_BYPASS); //CTS
					MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO59,0,MMPF_OS_LOCK_CTX_BYPASS); //RTS
				}
			}
            break;
        default:
            goto L_INVAL_UART;
        }
        #endif

	    // Disable interrupts
	    pUS->US_IER = 0;
	    
	    // Reset receiver and transmitter
	    pUS->US_CR = US_RSTRX | US_RSTTX | US_RXDIS | US_TXDIS;
	    
	    // Define the baud rate divisor register
		pUS->US_BRGR = (((uartattribute->ulMasterclk << 1) 
										/ uartattribute->ulBaudrate) + 1) >> 1;								
	    
	    // Define the UART mode 8-N-1
	    pUS->US_CR = US_ASYNC_MODE | US_TXEN | US_RXEN;

		#if (UART_RXINT_MODE_EN == 1)
	    pUS->US_FTHR &= ~US_RX_FIFO_THRES_MASK;
	    pUS->US_FTHR |= US_RX_FIFO_THRES(1);
	    pUS->US_IER = US_RX_FIFO_OVER_THRES;
		pUS->US_ISR &= pUS->US_IER;
		#endif
		
		if(uartattribute->bParityEn == MMP_TRUE) 
		{
			pUS->US_CR &= (~US_PAR_DIS);
			pUS->US_CR &= (~US_PAR_MASK);
			
			switch(uartattribute->parity) {
				case MMPF_UART_PARITY_EVEN:
					pUS->US_CR |= US_PAR_EVEN;
					break;
				case MMPF_UART_PARITY_ODD:
					pUS->US_CR |= US_PAR_ODD;
					break;
				case MMPF_UART_PARITY_FORCE0:
					pUS->US_CR |= US_PAR_0;
					break;
				case MMPF_UART_PARITY_FORCE1:
					pUS->US_CR |= US_PAR_1;
					break;
				default:
					break;
			} 
		}
		else {
			pUS->US_CR |= US_PAR_DIS;
		}
		
        if (uartattribute->bFlowCtlEn == MMP_TRUE) {
		    if (uartattribute->ubFlowCtlSelect) {
		    	pUS->US_CR |= US_RTSCTS_ACTIVE_H;
		    }
		    else {
		    	pUS->US_CR &= (~US_RTSCTS_ACTIVE_H); 
		    }
			
#if 0
		    //pUS->US_CR |= US_RTSCTS_EN;
		    pUS->US_CR |= US_RTSCTS_EN | US_ASYNC_MODE | US_RTSCTS_MODE_1 |US_RTSCTS_HOST_MODE | 0x30;
		    //pUS->US_CR |= US_RTSCTS_EN | US_ASYNC_MODE | US_RTSCTS_MODE_1 | US_RTSCTS_HOST_MODE | 0x30;
		    pUS->US_HOST_IER = US_RX_FIFO_FULL | US_RX_FIFO_OVER_THRES;
		    pUS->US_IER = US_RX_FIFO_FULL | US_RX_FIFO_OVER_THRES;
#else		
    		pUS->US_CR = US_ASYNC_MODE | US_TXEN | US_RXEN |
		 	 	 	 US_RTSCTS_EN | US_RTSCTS_MODE_1 | US_RTSCTS_HOST_MODE | 0x30;
		    //pUS->US_FTHR &= ~(US_RX_FIFO_THRES_MASK);
		    //pUS->US_FTHR |= US_RX_FIFO_THRES(1);
		    //pUS->US_IER = US_RX_FIFO_OVER_THRES | US_RX_FIFO_OVER_THRES;		///Enable RX_FIFO_OVER_THRES interrupt
		    //pUS->US_ISR &= pUS->US_IER;
#endif
		}
		else {
			pUS->US_CR &= (~US_RTSCTS_EN);
		}

		MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
	}
	else {
		return MMP_UART_SYSTEM_ERR;
	}
	return	status;

L_INVAL_UART:
    MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
    RTNA_DBG_Str(0, "Un-supported uart setting !!\r\n");
    return MMP_UART_ERR_PARAMETER;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Write
//  Description : Debug output a string
//  Note : uart are often used in CPU ISR mode (ex: for debug), so we comment out the resource protect code.
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_Write(MMPF_UART_ID uart_id, char *pWrite_Str, MMP_ULONG ulLength)
{
    MMP_ULONG   i, txcnt;
	MMP_ERR	    status = MMP_ERR_NONE;
    AITPS_US    pUS = m_pUS[uart_id];

    while (ulLength) {
        txcnt = (pUS->US_FSR & US_TX_FIFO_UNWR_MASK) >> 8;

        if (txcnt) {
            if (txcnt > ulLength) {
                txcnt = ulLength;
            }
            for (i = 0; i < txcnt; i++) {

                pUS->US_TXPR = *pWrite_Str++;
            }
            ulLength -= txcnt;
        }
    }

    return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Close
//  Description :
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_Close(MMPF_UART_ID uart_id)
{
    AITPS_US    pUS = m_pUS[uart_id];

	if (MMPF_OS_AcquireSem(m_UartSemID[uart_id], UART_RXINT_SEM_TIMEOUT) == 0) {
	    
	    // Disable interrupts
	    pUS->US_IER = 0;

	    // Reset receiver and transmitter
	    pUS->US_CR = US_RSTRX | US_RSTTX | US_RXDIS | US_TXDIS;
	    
	    MMPF_OS_ReleaseSem(m_UartSemID[uart_id]);
    }
    else {
    	return MMP_UART_SYSTEM_ERR;
    }

    return	MMP_ERR_NONE;
}
#if (OS_TYPE == OS_UCOSII)
#pragma arm section code, rodata, rwdata, zidata
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Close
//  Description : Return if RX is enabled with the specified UART ID
//  Note        :
//------------------------------------------------------------------------------
MMP_BOOL MMPF_Uart_IsRxEnable(MMPF_UART_ID uart_id)
{
    if (uart_id < MMPF_UART_MAX_COUNT)
        return m_UartRxEnable[uart_id];
    else
        return MMP_FALSE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_ConfigRx
//  Description :
//  Note        :
//------------------------------------------------------------------------------
void MMPF_Uart_ConfigRx(MMPF_UART_ID uart_id, MMP_ULONG threshold)
{
    AITPS_US    pUS = m_pUS[uart_id];

    // Disable FIFO interrupt first before reset FIFO threshold
    pUS->US_IER &= ~(US_RX_FIFO_OVER_THRES);

    pUS->US_FTHR &= ~(US_RX_FIFO_THRES_MASK);
    pUS->US_FTHR |= US_RX_FIFO_THRES(threshold);

    // Enable receiver and Rx FIFO interrupt
    pUS->US_CR |= US_RXEN;
    pUS->US_IER |= US_RX_FIFO_OVER_THRES;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_EnableRx
//  Description : Enable RX with the specified UART ID, set interrupt threshold
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_EnableRx(MMPF_UART_ID uart_id)
{ 
	AITPS_US    pUS = m_pUS[uart_id];

    if (uart_id >= MMPF_UART_MAX_COUNT)
	{
		return MMP_UART_ERR_PARAMETER;
	}

	pUS->US_CR |= US_RXEN;

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Uart_EnableRx);

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_DisableRx
//  Description : Disable RX with the specified UART ID
//  Note        :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_DisableRx(MMPF_UART_ID uart_id)
{
    AITPS_US    pUS = m_pUS[uart_id];

    if (uart_id >= MMPF_UART_MAX_COUNT)
	{
        return MMP_UART_ERR_PARAMETER;
	}

	pUS->US_CR &= ~(US_RXEN);

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Uart_DisableRx);

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_TryRead
//  Description : Non-blocking read, return the actual read chars
//------------------------------------------------------------------------------
MMP_ULONG MMPF_Uart_TryRead(MMPF_UART_ID uart_id, MMP_UBYTE *buf, MMP_ULONG ulMaxReadLen)
{
    AITPS_US    pUS = m_pUS[uart_id];
    MMP_ULONG   nr = 0;

    while ((pUS->US_FSR & US_RX_FIFO_UNRD_MASK) && (nr < ulMaxReadLen)) {
        buf[nr++] = pUS->US_RXPR;
    }

    return nr;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_CheckState
//  Description :
//------------------------------------------------------------------------------
MMP_BOOL MMPF_Uart_CheckState(MMPF_UART_ID uart_id, MMPF_UART_STAT State)
{
    AITPS_US    pUS = m_pUS[uart_id];

    switch (State) {
    case MMPF_UART_STAT_TXEMPTY:
        return (pUS->US_ISR & US_TX_FIFO_EMPTY)? MMP_TRUE: MMP_FALSE;
    case MMPF_UART_STAT_RXEMPTY:
        return (pUS->US_ISR & US_RX_FIFO_EMPTY)? MMP_TRUE: MMP_FALSE;
    case MMPF_UART_STAT_TXUNDERTH:
        return (pUS->US_ISR & US_TX_FIFO_UNDER_THRES)? MMP_TRUE: MMP_FALSE;
    case MMPF_UART_STAT_RXOVERTH:
        return (pUS->US_ISR & US_RX_FIFO_OVER_THRES)? MMP_TRUE: MMP_FALSE;
    default:
        return MMP_FALSE;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_Reset
//  Description : Reset RX, TX or both
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_Reset(MMPF_UART_ID uart_id, MMPF_UART_DIRECTION direction)
{
    AITPS_US    pUS = m_pUS[uart_id];
    MMP_ULONG   flags = 0;

    if (direction & MMPF_UART_DIRECTION_RX) {
        flags |= US_RSTRX;
    }
    if (direction & MMPF_UART_DIRECTION_TX) {
        flags |= US_RSTTX;
    }
    pUS->US_CR |= flags;

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Uart_SetInterruptEnable
//  Description : UART interrupt enable/disable
//------------------------------------------------------------------------------
MMP_ERR MMPF_Uart_SetInterruptEnable(MMPF_UART_ID uart_id, MMPF_UART_EVENT event, MMP_BOOL bEnable)
{
    static const MMP_ULONG flag[MMPF_UART_EVENT_MAX] = {
        US_RX_FIFO_OVER_THRES, US_TX_FIFO_UNDER_THRES
    };
    AITPS_US    pUS = m_pUS[uart_id];

    if (bEnable) {
        if (flag[event] & US_SR_W1C_MASK) {
            pUS->US_ISR = flag[event];
        }
        pUS->US_IER |= flag[event];
    } else {
        pUS->US_IER &= ~(flag[event]);
    }

    return MMP_ERR_NONE;
}
