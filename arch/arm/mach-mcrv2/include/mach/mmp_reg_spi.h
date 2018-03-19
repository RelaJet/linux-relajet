//==============================================================================
//
//  File        : mmp_reg_spi.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_SPI_H_
#define _MMP_REG_SPI_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

//--------------------------------------
// SPI Structure (0x8000 CC00) for P_V2
// SPI Structure (0x8000 6D00) for MCR_V2
//--------------------------------------
typedef struct _AITS_SPI {
    AIT_REG_W   SPI_CTL;                                                // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_TXFIFO_CLR              0x20
        #define SPI_RXFIFO_CLR	            0x10
        #define SPI_TX_DMA_STOP				0x08
        #define SPI_RX_DMA_STOP				0x04
        #define SPI_TX_DMA_START		    0x02
        #define SPI_RX_DMA_START		    0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   SPI_CLK_DIV;                                            // 0x02
    AIT_REG_B                           _x03;
    AIT_REG_W   SPI_CFG;                                                // 0x04
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_RX_CLK_ADJUST_EN        0x1000
        #define RX_CLK_ADJ_EN				SPI_RX_CLK_ADJUST_EN
        #define RX_PHA_ADJ						0x0800
        #define RX_POL_ADJ						0x0400
        #define MASTER_RX_USE_INCLK         0x0000
        #define MASTER_RX_USE_PAD_CLK       0x0200
        #define TX_NON_XCH_MODE			    0x0100
        #define TX_XCH_MODE 		        0x0000
        #define SPI_CHECK_RX_FIFO           0x0080
        #define SPI_MASTER_MODE             0x0040
        #define SPI_SLAVE_MODE              0x0000
        #define SPI_TX_EN   	    	    0x0020
        #define SPI_RX_EN	    	        0x0010
		#define	SS_POLAR_LOW				0x0000
		#define	SS_POLAR_HIGH				0x0008
		#define SS_BURST_MODE 			 	0x0000
		#define SS_NON_BURST_MODE		 	0x0004
		#define SCLK_PHASE0_OP 			 	0x0000
		#define SCLK_PHASE1_OP 			 	0x0002
		#define SCLK_IDLE_LOW 			 	0x0000
		#define SCLK_IDLE_HIGH			 	0x0001
		#define SCLK_PHA_LOW			 	 0x0000
		#define SCLK_PHA_HIGH 			 	 SCLK_PHASE1_OP
		#define SCLK_POL_LOW 			 	 SCLK_IDLE_LOW
		#define SCLK_POL_HIGH			 	 SCLK_IDLE_HIGH	
        /*------------------------------------------------------------*/
    AIT_REG_B   SPI_WORD_LEN;                                           // 0x06
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_WORD_LEN_MASK		    0x1F
        #define SPI_WORD_LENGTH(_a)		    ((_a - 1) & SPI_WORD_LEN_MASK)
   		#define SPI_BITCOUNT_MASK			SPI_WORD_LEN_MASK		
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x07;
    AIT_REG_W   SPI_DLY_CYCLE;                                          // 0x08
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_DLY_CYCLE_MASK			0xFFFF
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0A[0x2];
    AIT_REG_W   SPI_WAIT_CYCLE;                                         // 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_WAIT_CYCLE_MASK			0xFFFF
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0E[0x2];
    
    AIT_REG_B   SPI_XCH_CTL;											// 0x10
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_XCH_START               0x01
        #define XCH_START                   0x01		
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x11[0x3];
    AIT_REG_B   SPI_TXFIFO_SPC;											// 0x14
    AIT_REG_B   SPI_RXFIFO_SPC;											// 0x15
    AIT_REG_B   SPI_TXFIFO_THD;											// 0x16
    AIT_REG_B   SPI_RXFIFO_THD;											// 0x17
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_FIFO_DEPTH              0x20
        /*------------------------------------------------------------*/        
    
    union _SPI_TXFIFO_DATA {                                            // 0x18
    AIT_REG_B   B[4];
    AIT_REG_W   W[2];
    AIT_REG_D   D[1];
    } SPI_TXFIFO_DATA;
    
    union _SPI_RXFIFO_DATA {                                            // 0x1C
    AIT_REG_B   B[4];
    AIT_REG_W   W[2];
    AIT_REG_D   D[1];
    } SPI_RXFIFO_DATA;

	AIT_REG_D   SPI_INT_CPU_SR;                                         // 0x20
	AIT_REG_D   SPI_INT_CPU_EN;                                         // 0x24
	AIT_REG_D   SPI_INT_HOST_SR;                                        // 0x28
	AIT_REG_D   SPI_INT_HOST_EN;                                        // 0x2C
		/*-DEFINE-----------------------------------------------------*/
        #define SPI_RXFIFO_LE               0x00000001
        #define SPI_RXFIFO_GE               0x00000002
		#define SPI_RXFIFO_EMPTY            0x00000004
		#define SPI_RXFIFO_FULL             0x00000008
        #define SPI_TXFIFO_LE               0x00000010
        #define SPI_TXFIFO_GE               0x00000020
		#define SPI_TXFIFO_EMPTY            0x00000040
		#define SPI_TXFIFO_FULL             0x00000080
		#define SPI_RXDMA_DONE	            0x00000100
		#define SPI_TXDMA_DONE	            0x00000200
		#define SPI_FIFO_TX_DONE            0x00001000
		#define SPI_RXFIFO_OVF	            0x00010000
		#define SPI_TXFIFO_UDF	            0x00020000
		#define SPI_RX_BIT_OVF	            0x00040000
        #define SPI_SLAVE_ERR               0x00070000
		#define SPI_RXFIFO_OF	            0x00010000
		#define SPI_TXFIFO_UF	            0x00020000
		#define SPI_RX_BIT_OF	            0x00040000		
		/*------------------------------------------------------------*/
		
    AIT_REG_D   SPI_TXDMA_ADDR;											// 0x30
    AIT_REG_D   SPI_RXDMA_ADDR;											// 0x34
    AIT_REG_W   SPI_TXDMA_SIZE;											// 0x38
    AIT_REG_B                           _x3A[0x2];
    AIT_REG_W   SPI_RXDMA_SIZE;											// 0x3C
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_DMA_SIZE_MASK		    0xFFFF
        #define SPI_DMA_SIZE(_a)		    ((_a - 1) & SPI_DMA_SIZE_MASK)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x3E[0x2];
    
    #if (CHIP == P_V2)
    AIT_REG_W                           _x40[96];
    #endif
} AITS_SPI, *AITPS_SPI;

typedef struct _AITS_SPIB {
    AITS_SPI    SPI[3];
} AITS_SPIB, *AITPS_SPIB;


/* Register access macros */
#define SPI_REG_OFFSET(reg) offsetof(AITS_SPI,reg)//(((AITPS_AFE)0)->reg-(AITPS_AFE)0)

#define ait_spi_readb(port,reg) \
	__raw_readb((port)->regs + SPI_REG_OFFSET(reg))
#define ait_spi_writeb(port,reg,value) \
	__raw_writeb((value), (port)->regs + SPI_REG_OFFSET(reg))

#define ait_spi_readw(port,reg) \
	__raw_readw((port)->regs + SPI_REG_OFFSET(reg))
#define ait_spi_writew(port,reg,value) \
	__raw_writew((value), (port)->regs + SPI_REG_OFFSET(reg))

#define ait_spi_readl(port,reg) \
	__raw_readl((port)->regs + SPI_REG_OFFSET(reg))
#define ait_spi_writel(port,reg,value) \
	__raw_writel((value), (port)->regs + SPI_REG_OFFSET(reg))



//#define SPI_GET_CTL(port) ait_spi_readw(port,SPI_CTL)
#define SPI_SET_CTL(port,v) ait_spi_writew(port,SPI_CTL,v)


#define SPI_GET_CLK_DIV(port) ait_spi_readb(port,SPI_CLK_DIV)
#define SPI_SET_CLK_DIV(port,v) ait_spi_writeb(port,SPI_CLK_DIV,v)


#define SPI_GET_CFG(port) ait_spi_readw(port,SPI_CFG)
#define SPI_SET_CFG(port,v) ait_spi_writew(port,SPI_CFG,v)

#define SPI_GET_WORD_LEN(port) ait_spi_readb(port,SPI_WORD_LEN)
#define SPI_SET_WORD_LEN(port,v) ait_spi_writeb(port,SPI_WORD_LEN,(v-1)&SPI_BITCOUNT_MASK)


#define SPI_GET_DLY_CYCLE(port) ait_spi_readw(port,SPI_DLY_CYCLE)
#define SPI_SET_DLY_CYCLE(port,v) ait_spi_writew(port,SPI_DLY_CYCLE,v)


#define SPI_GET_WAIT_CYCLE(port) ait_spi_readw(port,SPI_WAIT_CYCLE)
#define SPI_SET_WAIT_CYCLE(port,v) ait_spi_writew(port,SPI_WAIT_CYCLE,v)

#define SPI_SET_XCH_CTL(port,v) ait_spi_writeb(port,SPI_XCH_CTL,v)


#define SPI_GET_TXFIFO_SPC(port) ait_spi_readb(port,SPI_TXFIFO_SPC)
#define SPI_SET_TXFIFO_SPC(port,v) ait_spi_writeb(port,SPI_TXFIFO_SPC,v)

#define SPI_GET_RXFIFO_SPC(port) ait_spi_readb(port,SPI_RXFIFO_SPC)
#define SPI_SET_RXFIFO_SPC(port,v) ait_spi_writeb(port,SPI_RXFIFO_SPC,v)

#define SPI_GET_TXFIFO_THD(port) ait_spi_readb(port,SPI_TXFIFO_THD)
#define SPI_SET_TXFIFO_THD(port,v) ait_spi_writeb(port,SPI_TXFIFO_THD,v)

#define SPI_GET_RXFIFO_THD(port) ait_spi_readb(port,SPI_RXFIFO_THD)
#define SPI_SET_RXFIFO_THD(port,v) ait_spi_writeb(port,SPI_RXFIFO_THD,v)

#define SPI_GET_TXFIFO_B(port) ait_spi_readb(port,SPI_TXFIFO_DATA)
#define SPI_SET_TXFIFO_B(port,v) ait_spi_writeb(port,SPI_TXFIFO_DATA,v)
#define SPI_GET_TXFIFO_W(port) ait_spi_readw(port,SPI_TXFIFO_DATA)
#define SPI_SET_TXFIFO_W(port,v) ait_spi_writew(port,SPI_TXFIFO_DATA,v)
#define SPI_GET_TXFIFO_L(port) ait_spi_readl(port,SPI_TXFIFO_DATA)
#define SPI_SET_TXFIFO_L(port,v) ait_spi_writel(port,SPI_TXFIFO_DATA,v)

#define SPI_GET_RXFIFO_B(port) ait_spi_readb(port,SPI_RXFIFO_DATA)
#define SPI_SET_RXFIFO_B(port,v) ait_spi_writeb(port,SPI_RXFIFO_DATA,v)
#define SPI_GET_RXFIFO_W(port) ait_spi_readw(port,SPI_RXFIFO_DATA)
#define SPI_SET_RXFIFO_W(port,v) ait_spi_writew(port,SPI_RXFIFO_DATA,v)
#define SPI_GET_RXFIFO_L(port) ait_spi_readl(port,SPI_RXFIFO_DATA)
#define SPI_SET_RXFIFO_L(port,v) ait_spi_writel(port,SPI_RXFIFO_DATA,v)


#define SPI_GET_INT_SR(port) ait_spi_readl(port,SPI_INT_CPU_SR)
#define SPI_SET_INT_SR(port,v) ait_spi_writel(port,SPI_INT_CPU_SR,v)

#define SPI_GET_INT_EN(port) ait_spi_readl(port,SPI_INT_CPU_EN)
#define SPI_SET_INT_EN(port,v) ait_spi_writel(port,SPI_INT_CPU_EN,v)

#define SPI_GET_INT_HOST_SR(port) ait_spi_readl(port,SPI_INT_HOST_SR)
#define SPI_SET_INT_HOST_SR(port,v) ait_spi_writel(port,SPI_INT_HOST_SR,v)

#define SPI_GET_INT_HOST_EN(port) ait_spi_readl(port,SPI_INT_HOST_EN)
#define SPI_SET_INT_HOST_EN(port,v) ait_spi_writel(port,SPI_INT_HOST_EN,v)

#define SPI_GET_TXDMA_ADDR(port) ait_spi_readl(port,SPI_TXDMA_ADDR)
#define SPI_SET_TXDMA_ADDR(port,v) ait_spi_writel(port,SPI_TXDMA_ADDR,v)

#define SPI_GET_RXDMA_ADDR(port) ait_spi_readl(port,SPI_RXDMA_ADDR)
#define SPI_SET_RXDMA_ADDR(port,v) ait_spi_writel(port,SPI_RXDMA_ADDR,v)

#define SPI_GET_TXDMA_SIZE(port) ait_spi_readw(port,SPI_TXDMA_SIZE)
#define SPI_SET_TXDMA_SIZE(port,v) ait_spi_writew(port,SPI_TXDMA_SIZE,v-1)

#define SPI_GET_RXDMA_SIZE(port) ait_spi_readw(port,SPI_RXDMA_SIZE)
#define SPI_SET_RXDMA_SIZE(port,v) ait_spi_writew(port,SPI_RXDMA_SIZE,v-1)
#endif // _MMPH_REG_SPI_H_