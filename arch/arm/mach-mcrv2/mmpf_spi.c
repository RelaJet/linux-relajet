
//==============================================================================
//
//  File        : MMPF_spi.c
//  Description : Programmable Serial Peripheral Interface
//                Module Control driver function
//  Author      : Sunny Sun
//  Revision    : 1.0
//
//==============================================================================

/**
*  @file mmpf_spi.c
*  @brief The PSPI Module Control functions
*  @author Sunny Sun
*  @version 1.0
*/

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
//#include "includes_fw.h"
//#include "config_fw.h"
//#include "lib_retina.h"
#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_spi.h>
#include <mach/mmpf_spi.h>
#include <mach/mmpf_system.h>

/** @addtogroup MMPF_Spi
 *  @{
 */

//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================

static SPICallBackFunc *SPICallBack[MMPF_SPI_ID_MAX];

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_ISR
//  Description : The function is the SPI interrupt service routine
//------------------------------------------------------------------------------
void MMPF_SPI_ISR(void)
{
    AITPS_SPI   pSPI;
    MMP_ULONG   int_src = 0;
    MMP_USHORT  i;

    for (i = 0; i < MMPF_SPI_ID_MAX; i++) 
    {
        pSPI = &AITC_BASE_SPIB->SPI[i];
        
        int_src = pSPI->SPI_INT_CPU_SR & pSPI->SPI_INT_CPU_EN;
        
        pSPI->SPI_INT_CPU_EN &= (~int_src);
        pSPI->SPI_INT_CPU_SR = int_src;
        
        if (int_src & SPI_FIFO_TX_DONE) {
            if (SPICallBack[i]) {
                (*SPICallBack[i])();
            }
        }
        if (int_src & SPI_TXDMA_DONE) {
            if (SPICallBack[i]) {
                (*SPICallBack[i])();
            }
        }
        if (int_src & SPI_RXDMA_DONE) {
            if (SPICallBack[i]) {
                (*SPICallBack[i])();
            }
        }
        pSPI->SPI_INT_CPU_EN |= (int_src);
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_Initialize
//  Description : The function initialize SPI module.
//------------------------------------------------------------------------------
MMP_ERR MMPF_SPI_Initialize(void)
{
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC pAIC = AITC_BASE_AIC;
    #endif
    
    #if (CHIP == P_V2)
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_SPI, MMP_TRUE);

    RTNA_AIC_Open(pAIC, AIC_SRC_SIF_SPI_0_1_2, MMPF_SPI_ISR, AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 2);
    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_SIF_SPI_0_1_2);
    #endif
    #if (CHIP == MCR_V2)
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_PSPI, MMP_TRUE);
    #if (OS_TYPE == OS_UCOSII)
    RTNA_AIC_Open(pAIC, AIC_SRC_SPI, MMPF_SPI_ISR, AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 2);
    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_SPI);  
    #endif  
    #endif
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_SetAttributes
//  Description : The function sets the attributes to the specified spi channel.
//------------------------------------------------------------------------------
/** @brief The function sets the attributes to the specified spi channel with its spi ID

The function sets the attributes to the specified spi channel with its icon ID. These attributes include
spi master/slave mode, signal type, INT enable, and related clock. It is implemented by programming PSPI
Controller registers to set those attributes.

  @param[in] usSpiID the SPI ID
  @param[in] spiattribute the SPI attribute
  @return It reports the status of the operation.
*/
MMP_ERR MMPF_SPI_SetAttributes(MMPF_SPI_ID usSpiID, MMPF_SPI_ATTRIBUTE *spiattribute)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    AITPS_SPI   pSPI = &AITC_BASE_SPIB->SPI[usSpiID];

    if (spiattribute->padCtl < MMPF_SPI_PAD_MAX) {
        #if (CHIP == P_V2)
        pGBL->GBL_MIO_SPI_CTL |= (GBL_SPI_PADMAP(spiattribute->padCtl, usSpiID)|
                                  GBL_SPI_PADSET_EN(spiattribute->padCtl));
        #endif
        #if (CHIP == MCR_V2)
        if (usSpiID == MMPF_SPI_ID_0) {
            pGBL->GBL_SPI_PAD_CFG &= ~(GBL_PSPI0_PAD_MASK);
            pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI0_PAD(spiattribute->padCtl);
        }
        else if (usSpiID == MMPF_SPI_ID_1) {
            pGBL->GBL_SPI_PAD_CFG &= ~(GBL_PSPI1_PAD_MASK);
            pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI1_PAD(spiattribute->padCtl);
        }
        else {
            pGBL->GBL_SPI_PAD_CFG &= ~(GBL_PSPI2_PAD_MASK);
            pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI2_PAD_EN;
        }
        #endif
    }

    pSPI->SPI_CTL |= (SPI_TXFIFO_CLR | SPI_RXFIFO_CLR);
    
    if (spiattribute->spiMode == MMPF_SPI_MASTER_MODE) {
        
        pSPI->SPI_CFG = (MASTER_RX_USE_PAD_CLK | SPI_MASTER_MODE | spiattribute->usSignalCtl);

        pSPI->SPI_WORD_LEN      = SPI_WORD_LENGTH(spiattribute->ubWordLength);
        pSPI->SPI_CLK_DIV       = spiattribute->ubSclkDiv;
        pSPI->SPI_DLY_CYCLE     = spiattribute->ubPspiDelay;
        pSPI->SPI_WAIT_CYCLE    = spiattribute->ubPspiWait;
        pSPI->SPI_TXFIFO_THD    = spiattribute->ubTxFIFOThres;
        pSPI->SPI_RXFIFO_THD    = spiattribute->ubRxFIFOThres;
        pSPI->SPI_INT_CPU_SR    = spiattribute->ulIntEnable;
        pSPI->SPI_INT_CPU_EN    = spiattribute->ulIntEnable;
    }
    else {
        pSPI->SPI_CFG = (MASTER_RX_USE_PAD_CLK | spiattribute->usSignalCtl);
        
        pSPI->SPI_WORD_LEN      = SPI_WORD_LENGTH(spiattribute->ubWordLength);
        pSPI->SPI_TXFIFO_THD    = spiattribute->ubTxFIFOThres;
        pSPI->SPI_RXFIFO_THD    = spiattribute->ubRxFIFOThres;
        pSPI->SPI_INT_CPU_SR    = spiattribute->ulIntEnable;
        pSPI->SPI_INT_CPU_EN    = spiattribute->ulIntEnable;
    }
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_SetAttributes
//  Description : The function excute fifo/dma read/write with its SPI ID.
//------------------------------------------------------------------------------
/** @brief The function excute fifo/dma read/write with its spi ID

  @param[in] usSpiID the SPI ID
  @param[in] spiop operation setting
  @param[in] fpSPICallBack call back function pointer
  @return It reports the status of the operation.
*/
MMP_ERR MMPF_SPI_Operation(MMPF_SPI_ID usSpiID, MMPF_SPI_OPERATION *spiop, SPICallBackFunc *fpSPICallBack)
{
    AITPS_SPI   pSPI = &AITC_BASE_SPIB->SPI[usSpiID];
    MMP_BOOL    tx_en = MMP_FALSE, rx_en = MMP_FALSE;
    MMP_BOOL    tx_dma_mode = MMP_FALSE, rx_dma_mode = MMP_FALSE;
    MMP_USHORT  outbyte, transferbyte, i;
    MMP_UBYTE   *ptr, *ptr2;
    MMP_USHORT  byte_count;

    SPICallBack[usSpiID] = fpSPICallBack;
    
    if (spiop->dir & MMPF_SPI_TX) {
        tx_en = MMP_TRUE;
        if (spiop->ulTxDmaAddr != 0) {
            tx_dma_mode = MMP_TRUE;
        }
    }
    if (spiop->dir & MMPF_SPI_RX) {
        rx_en = MMP_TRUE;
        if (spiop->ulRxDmaAddr != 0) {
            rx_dma_mode = MMP_TRUE;
        }
    }
    
    if( tx_en == MMP_TRUE && rx_en == MMP_TRUE )
    {
        if (tx_dma_mode == MMP_TRUE && rx_dma_mode == MMP_TRUE) 
        {
            pSPI->SPI_INT_CPU_SR = SPI_TXDMA_DONE | SPI_RXDMA_DONE;
            
            pSPI->SPI_TXDMA_ADDR = spiop->ulTxDmaAddr;
            pSPI->SPI_RXDMA_ADDR = spiop->ulRxDmaAddr;
            pSPI->SPI_TXDMA_SIZE = SPI_DMA_SIZE(spiop->usTransferSize);
            pSPI->SPI_RXDMA_SIZE = SPI_DMA_SIZE(spiop->usTransferSize);
            
            pSPI->SPI_CFG |= (SPI_TX_EN | SPI_RX_EN | TX_XCH_MODE);
            pSPI->SPI_CTL = (SPI_RX_DMA_START | SPI_TX_DMA_START);
            
            //EROY CHECK : The start condition
            if (!((pSPI->SPI_INT_CPU_EN & (SPI_RXDMA_DONE|SPI_TXDMA_DONE) ) == (SPI_RXDMA_DONE|SPI_TXDMA_DONE) )) 
            {
                pSPI->SPI_XCH_CTL = SPI_XCH_START;
             
                while( !(pSPI->SPI_INT_CPU_SR & SPI_TXDMA_DONE) );
                while( !(pSPI->SPI_INT_CPU_SR & SPI_FIFO_TX_DONE));
                while( !(pSPI->SPI_INT_CPU_SR & SPI_RXDMA_DONE) );
            }
        }
        else 
        {                
            pSPI->SPI_CFG |= (SPI_RX_EN | SPI_TX_EN | TX_NON_XCH_MODE);
            
            outbyte = spiop->usTransferSize;

            ptr  = spiop->ubTxFifoPtr;
            ptr2 = spiop->ubRxFifoPtr;
                            
            while (outbyte) 
            {
                transferbyte = pSPI->SPI_TXFIFO_SPC;
                transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
            
                if (transferbyte) {
                    for (i = 0; i < transferbyte; i++) {
                        pSPI->SPI_TXFIFO_DATA.B[0] = *ptr++;
                        if (pSPI->SPI_RXFIFO_SPC) {
                            *ptr2++ = pSPI->SPI_RXFIFO_DATA.B[0];
                        }
                    }
                }

                outbyte -= transferbyte;
            }
            
            while(! (pSPI->SPI_INT_CPU_SR & SPI_FIFO_TX_DONE));
            
            while(pSPI->SPI_RXFIFO_SPC) {
                *ptr2++ = pSPI->SPI_RXFIFO_DATA.B[0];
            }
        }    
    }
    else if ( tx_en == MMP_TRUE && rx_en == MMP_FALSE )
    {
        if (tx_dma_mode == MMP_TRUE) {

            pSPI->SPI_INT_CPU_SR = SPI_TXDMA_DONE;

            pSPI->SPI_TXDMA_ADDR = spiop->ulTxDmaAddr;
            pSPI->SPI_TXDMA_SIZE = SPI_DMA_SIZE(spiop->usTransferSize);
            
            pSPI->SPI_CFG &= ~(SPI_RX_EN);
            pSPI->SPI_CFG |= (SPI_TX_EN | TX_XCH_MODE);

            pSPI->SPI_CTL = SPI_TX_DMA_START;
            pSPI->SPI_XCH_CTL = SPI_XCH_START;

            if (! (pSPI->SPI_INT_CPU_EN & SPI_TXDMA_DONE) ) {
                while( !(pSPI->SPI_INT_CPU_SR & SPI_TXDMA_DONE));
                while( !(pSPI->SPI_INT_CPU_SR & SPI_FIFO_TX_DONE));
            }
        }
        else {
        
            pSPI->SPI_CFG &= ~(SPI_RX_EN);
            pSPI->SPI_CFG |= (SPI_TX_EN | TX_NON_XCH_MODE);

            outbyte = spiop->usTransferSize;
            
            ptr = spiop->ubTxFifoPtr;
            
            while (outbyte) {
                
                transferbyte = pSPI->SPI_TXFIFO_SPC;
                transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
                
                if (transferbyte) {
                    for (i = 0; i < transferbyte; i++) {
                        pSPI->SPI_TXFIFO_DATA.B[0] = *ptr++;
                    }
                }

                outbyte -= transferbyte;
            }
            
            while(!(pSPI->SPI_INT_CPU_SR & SPI_FIFO_TX_DONE));
        }
    }
    else if ( tx_en == MMP_FALSE && rx_en == MMP_TRUE )
    {
        if (rx_dma_mode == MMP_TRUE) {

            pSPI->SPI_INT_CPU_SR = SPI_RXDMA_DONE;
            
            //EROY CHECK : Need Set Tx? 
            pSPI->SPI_TXDMA_SIZE = SPI_DMA_SIZE(spiop->usTransferSize);

            pSPI->SPI_RXDMA_ADDR = spiop->ulRxDmaAddr;
            pSPI->SPI_RXDMA_SIZE = SPI_DMA_SIZE(spiop->usTransferSize);

            pSPI->SPI_CFG &= ~(SPI_TX_EN);
            pSPI->SPI_CFG |= (SPI_RX_EN | TX_XCH_MODE);

            pSPI->SPI_CTL = SPI_RX_DMA_START;
         
            pSPI->SPI_XCH_CTL = SPI_XCH_START;

            if ( !(pSPI->SPI_INT_CPU_EN & SPI_RXDMA_DONE) ) {
                while( !(pSPI->SPI_INT_CPU_SR & SPI_RXDMA_DONE) );
            }
        }
        else {

            // For master mode, enable tx fifo, too
            byte_count = (pSPI->SPI_WORD_LEN + 7) >> 3;

            pSPI->SPI_CFG |= (SPI_TX_EN | SPI_RX_EN | TX_NON_XCH_MODE);

            outbyte = spiop->usTransferSize;
            
            ptr = spiop->ubRxFifoPtr;
            
            if (byte_count == 3) {
            
                while (outbyte) {

                    transferbyte = pSPI->SPI_TXFIFO_SPC;
                    transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
                    
                    for (i = 0; i < transferbyte; i+=3) {
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                    }

                    while(pSPI->SPI_RXFIFO_SPC != transferbyte);
                    
                    for (i = 0; i < transferbyte; i+=3) {
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                    }
                    outbyte -= transferbyte;
                }
            }
            else if (byte_count == 2) {
             
                while (outbyte) {

                    transferbyte = pSPI->SPI_TXFIFO_SPC;
                    transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
                    
                    for (i = 0; i < transferbyte; i+=2) {
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                    }

                    while( pSPI->SPI_RXFIFO_SPC != transferbyte);
                    
                    for (i = 0; i < transferbyte; i+=2) {
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                    }
                    outbyte -= transferbyte;
                }
            }
            else if (byte_count == 1) {
            
                while (outbyte) {
            
                    transferbyte = pSPI->SPI_TXFIFO_SPC;
                    transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
            
                    for (i = 0; i < transferbyte; i++) {
                        pSPI->SPI_TXFIFO_DATA.B[0] = 0xFF;
                    }
            
                    while( pSPI->SPI_RXFIFO_SPC != transferbyte);
            
                    for (i = 0; i < transferbyte; i++) {
                        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
                    }
                    outbyte -= transferbyte;
                }
            }

        }
    }

    return MMP_ERR_NONE;
}


MMP_ERR MMPF_SPI_WriteSub(MMPF_SPI_ID usSpiID, MMP_ULONG ulTxAddr, MMP_ULONG ulTxSize)
{
    AITPS_SPI   pSPI = &AITC_BASE_SPIB->SPI[usSpiID];
    MMP_USHORT  transferbyte, i, outbyte;
    MMP_UBYTE   *ptr = (MMP_UBYTE *)ulTxAddr;
    
    outbyte = ulTxSize;
    while (outbyte) 
    {
        transferbyte = 32-pSPI->SPI_TXFIFO_SPC;
        transferbyte = (transferbyte > outbyte) ?  outbyte : transferbyte;
        
        if (transferbyte) {
            for (i = 0; i < transferbyte; i++) {
                pSPI->SPI_TXFIFO_DATA.B[0] = *ptr++; 
            }
        }
        outbyte -= transferbyte;
    }
    
    while(! (pSPI->SPI_INT_CPU_SR & SPI_FIFO_TX_DONE));
    return MMP_ERR_NONE;

}
//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_WriteData
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SPI_Write(MMPF_SPI_ID usSpiID, MMP_ULONG ulTxAddr, MMP_ULONG ulTxSize)
{
    AITPS_SPI   pSPI = &AITC_BASE_SPIB->SPI[usSpiID];
      
    pSPI->SPI_CFG &= ~(SPI_RX_EN);
    pSPI->SPI_CFG |= (SPI_TX_EN | TX_NON_XCH_MODE);
       
    MMPF_SPI_WriteSub(usSpiID,ulTxAddr,ulTxSize);
    
    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_SPI_ReadData
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SPI_Read(MMPF_SPI_ID usSpiID, MMP_ULONG ulRxAddr, MMP_ULONG ulRxSize,MMP_ULONG ulTxAddr, MMP_ULONG ulTxSize)
{
    AITPS_SPI   pSPI = &AITC_BASE_SPIB->SPI[usSpiID];
    MMP_USHORT  inputbyte, i;
    MMP_UBYTE   *ptr;
    
    pSPI->SPI_CFG |= ( SPI_RX_EN |SPI_TX_EN| TX_NON_XCH_MODE);
    
    
    MMPF_SPI_WriteSub(usSpiID,ulTxAddr,ulTxSize);
    inputbyte = ulRxSize;           
   
    ptr = (MMP_UBYTE*) ulRxAddr;
                 
    while(pSPI->SPI_RXFIFO_SPC < inputbyte);
    
    for (i = 0; i < inputbyte; i++) {
        *ptr++ = pSPI->SPI_RXFIFO_DATA.B[0];
        //RTNA_DBG_Byte(0, pSPI->SPI_RXFIFO_DATA.B[0]);
    }    
    
    return MMP_ERR_NONE;}

/// @}


