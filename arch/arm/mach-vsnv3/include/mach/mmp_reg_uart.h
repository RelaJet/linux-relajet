//==============================================================================
//
//  File        : mmp_reg_uart.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_UART_H_
#define _MMP_REG_UART_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/
//-------------------------------
// US structure (0x80006400)
//-------------------------------
typedef struct _AITS_US {
    AIT_REG_D   US_CR;              // Control Register                 // 0x00
		/*-DEFINE-----------------------------------*/
		#define US_RSTRX            		0x00000001	        // Reset Receiver
		#define US_RSTTX		            0x00000002	        // Reset Transmitter
		#define US_RXEN                     0x00000004		    // Receiver Enable
		#define US_RXDIS                    0x00000000		    // Receiver Disable
		#define US_TXEN                     0x00000008		    // Transmitter Enable
		#define US_TXDIS                    0x00000000		    // Transmitter Disable
		#define US_PAR_DIS                  0x00000100		    // Parity Check/Generate Disable
		#define US_PAR_EN                   0x00000000		    // Parity Check/Generate Enable
		#define US_PAR_EVEN                 0x00000000		    // Even Parity
		#define US_PAR_ODD                  0x00000200		    // Odd Parity
		#define US_PAR_0                    0x00000400		    // Parity Force to "0"
		#define US_PAR_1                    0x00000600		    // Parity Force to "1"
		#define US_PAR_MASK                 0x00000600          // Parity Mask
		#define US_RXERR_WRITE_EN           0x00000800		    // Parity Error Write to RX Fifo
		#define US_RXERR_WRITE_DIS          0x00000000		    // Parity Error Not Write to RX Fifo
		#define US_RX_TIMEOUT_EN            0x00001000          // Enable uart Rx Timeout
		#define US_RTSCTS_EN				0x00010000		    // Enable uart HW Flow Control
		#define US_RTSCTS_MODE_0            0x00000000          // RTS/CTS Mode Selection
		#define US_RTSCTS_MODE_1            0x00020000          // RTS/CTS Mode Selection
		#define US_RTSCTS_ACTIVE_H			0x00040000          // RTS/CTS Active Select high				
		#define US_RTSCTS_PERI_MODE         0x00000000          // RTS/CTS Host/peripheral Mode Selection
		#define US_RTSCTS_HOST_MODE         0x00080000          // RTS/CTS Host/peripheral Mode Selection
		#define US_CTS_DETC_WAIT_CNT_MASK   0x00700000          // CTS Detect Wait Count
		#define US_RXDMA_START_FLAG			0x01000000
		#define US_RXDMA_RING_EN			0x02000000
		#define US_TXDMA_EN					0x04000000
		#define US_RXDMA_EN					0x08000000
		#define US_TXDMA_SW_RST             0x10000000
		#define US_RXDMA_SW_RST             0x20000000
		#define US_CLEAN_CTL0				0xFFFFFF00
		#define US_CLEAN_CTL1				0xFFFF00FF
        #define US_ASYNC_MODE               (US_PAR_DIS + US_RXERR_WRITE_DIS)
		/*------------------------------------------*/
    AIT_REG_D   US_BRGR;            // Baud Rate Generator Register     // 0x04
        /*-DEFINE-----------------------------------*/
    	#define US_BR_DIV_MASK              0x00FFFFFF
    	#define US_RX_TIMEOUT_SHIFTER		0x18
    	#define US_RX_TIMEOUT_MASK			0xFF000000
    	/*------------------------------------------*/
    AIT_REG_B   US_TXPR;            // Tx Data Port                     // 0x08
    AIT_REG_B                           _x09[0x3];
    AIT_REG_B   US_RXPR;            // Rx Data Port                     // 0x0C
    AIT_REG_B   US_RXSR;            // Rx Data Port Status              // 0x0D
    AIT_REG_B                           _x0E[0x2];

    AIT_REG_D   US_IER;             // Interrupt Enable Register        // 0x10
        /*-DEFINE-----------------------------------*/
		#define US_RX_PARITY_ERR            0x00000001          // Receiver Parity Check Error
		#define US_RX_FRAME_ERR             0x00000002          // Receiver Frame Error
		#define US_RX_FIFO_FULL             0x00000004          // Receiver FIFO Full
		#define US_RX_FIFO_EMPTY            0x00000008          // Receiver FIFO Empty
		#define US_RX_FIFO_OVER_THRES       0x00000010          // Receiver FIFO Over(equal) Threshold
		#define US_RX_FIFO_OVERFLOW         0x00000020          // Receiver FIFO Overflow
		#define US_RX_TIMEOUT               0x00000040          // Receiver Time Out
		#define US_TX_FIFO_FULL             0x00000100          // Transmitter FIFO Full
		#define US_TX_FIFO_EMPTY            0x00000200          // Transmitter FIFO Empty
		#define US_TX_FIFO_UNDER_THRES      0x00000400          // Transmitter FIFO Under(equal) Threshold
		#define US_TXDMA_END_EN				0x00010000
		#define US_RXDMA_TH_EN				0x00020000
		#define US_RXDMA_WRMEM_EN			0x00040000
		#define US_RXDMA_DROPDATA_EN		0x00080000
		#define US_RX_ERROR                 (US_RX_PARITY_ERR | US_RX_FRAME_ERR)
        #define US_SR_W1C_MASK              0x000F0063          // Write one clear bits
        /*------------------------------------------*/
    AIT_REG_D   US_ISR;             // Interrupt Status Register        // 0x14
    AIT_REG_B                           _x18[0x8];
    
    AIT_REG_W   US_FTHR;            // FIFO Threshold Register          // 0x20
        /*-DEFINE-----------------------------------*/
		#define US_TX_FIFO_THRES_MASK       0x00FF            // Transmitter FIFO Threshold Mask
		#define US_RX_FIFO_THRES_MASK       0x3F00            // Receiver FIFO Threshold Mask
		#define US123_TX_FIFO_THRES_MASK    0x003F            // Transmitter FIFO Threshold Mask		
		#define US_TX_FIFO_THRES(_a)    	(_a)              // Transmitter FIFO Threshold Shift
		#define US_RX_FIFO_THRES(_a)		(_a<<8)			  // Receiver FIFO Threshold Shift
        /*------------------------------------------*/
    AIT_REG_B                           _x22[0x2];
    AIT_REG_D   US_FSR;             // FIFO Status Register             // 0x24
        /*-DEFINE-----------------------------------*/
		#define US_RX_FIFO_UNRD_MASK        0x003F            // Transmitter FIFO Un-Write Mask
		#define US_TX_FIFO_UNWR_MASK        0xFF00            // Receiver FIFO Un-Read Mask
		#define US123_TX_FIFO_UNWR_MASK     0x3F00            // Receiver FIFO Un-Read Mask
        /*------------------------------------------*/
    AIT_REG_B                           _x28[0x8];

    AIT_REG_D   US_HOST_IER;        // Interrupt Enable Register        // 0x30
    AIT_REG_D   US_HOST_ISR;        // Interrupt Status Register        // 0x34
    AIT_REG_B                           _x38[0x8];
    
    /* Note : Below DMA section belong to Uart0 only, Uart1/Uart2/Uart3 are reserved */ 
    AIT_REG_D	US_TXDMA_START_ADDR;									// 0x40
    AIT_REG_W	US_TXDMA_TOTAL_BYTE;									// 0x44
    AIT_REG_B   						_x46[0x2];
    AIT_REG_W	US_TXDMA_UNRD_COUNT;									// 0x48
    AIT_REG_W	US_RXDMA_UNWR_COUNT;									// 0x4A
    AIT_REG_B   						_x4C[0x4];
    
    AIT_REG_D	US_RXDMA_START_ADDR;									// 0x50
    AIT_REG_D	US_RXDMA_END_ADDR;										// 0x54
    AIT_REG_D	US_RXDMA_LB_ADDR;										// 0x58
    AIT_REG_W	US_RXDMA_TOTAL_THR;										// 0x5C
    AIT_REG_B   						_x5E[0x2];
    
    AIT_REG_D   US_RXDBR;           // Rx Debug Register                // 0x60
    AIT_REG_D   US_TXDBR;           // Tx Debug Register                // 0x64
    AIT_REG_D   US_DBLBR;           // Debug Loop Back Register         // 0x68
    AIT_REG_B                           _x6C[0x4];
    
    AIT_REG_D	US_RXDMA_ADDR;											// 0x70
    AIT_REG_B	US_RXDMA_STATUS;										// 0x74
    AIT_REG_B							_x75[0xB];
    
    AIT_REG_D	US_TXDMA_ADDR;											// 0x80
    AIT_REG_B	US_TXDMA_STATUS;										// 0x84
    AIT_REG_B                           _x85[0x7B];						// 0x85 - 0xFF
} AITS_US, *AITPS_US;

#pragma pack(push,1)
typedef struct _AITS_UART {
#if (CHIP == VSN_V3)
    AITS_US     US1;                                        // 0x5C00~0x5CFF
    AIT_REG_D                           _x5D00[0x340];      // 0x5D00~0x69FF
    AITS_US     US0;                                        // 0x6A00~0x6AFF
#endif

#if (CHIP == MCR_V2)
    AITS_US     US3;                                                 // 0x5300
    AITS_US     US2;                                                 // 0x5400
    AIT_REG_B   _x5500[0x700];
    AITS_US     US1;                                                 // 0x5C00
    AIT_REG_B   _x5D00[0xD00];
    AITS_US     US0;                                                 // 0x6A00
#endif
} AITS_UART, *AITPS_UART;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct _AITS_UARTB {
    #if (CHIP == VSN_V3)
    AITS_US     UART_1;                                     // 0x5C00~0x5CFF
    AIT_REG_D                           _x5D00[0x340];      // 0x5D00~0x69FF
    AITS_US     UART_0;                                     // 0x6A00~0x6AFF
    #endif
    #if (CHIP == MERCURY)
    AITS_US     UART_3;                                     // 0x5300~0x53FF
    AITS_US     UART_2;                                     // 0x5400~0x54FF
    AIT_REG_D                           _x5500[0x1C0];      // 0x5500~0x5BFF
    AITS_US     UART_1;                                        // 0x5C00~0x5CFF
    AIT_REG_D                           _x5D00[0x340];      // 0x5D00~0x69FF
    AITS_US     UART_0;                                     // 0x6A00~0x6AFF
    #endif
	#if (CHIP == MCR_V2)
    AITS_US     UART_3;                                                 // 0x5300
    AITS_US     UART_2;                                                 // 0x5400
    AIT_REG_B   _x5500[0x700];
    AITS_US     UART_1;                                                 // 0x5C00
    AIT_REG_B   _x5D00[0xD00];
    AITS_US     UART_0;                                                 // 0x6A00
	#endif
} AITS_UARTB, *AITPS_UARTB;
#pragma pack(pop)
/// @}

AITPS_US GetpUS(const int mmpf_uart_id);

#endif	// _REG_UART_H_
