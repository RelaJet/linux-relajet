//==============================================================================
//
//  File        : mmp_register_usb.h
//  Description : INCLUDE File for the USB register map.
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================



#ifndef _MMP_REG_USB_H_
#define _MMP_REG_USB_H_

#include <mach/includes_fw.h>
#if (CHIP == P_V2)||(CHIP == VSN_V2)||(CHIP == VSN_V3)
//#include <mach/mmp_register.h>
/*
#define     USB_DMA_BASE_ADDR		(0x80001400)  //USB DMA registers base address
#define 	USB_CTL_BASE_ADDR		(0x80001000)  //USB control registers base address
*/
/** @addtogroup MMPH_reg
@{
*/

// *****************************************************************************
//   USB base control register structure (0x8000 6000)
// *****************************************************************************
typedef struct _AITS_USB_DMA {
    AIT_REG_B   USB_CTL_SEL;                                          	// 0x00
   	AIT_REG_B   USB_SIDDQ;                                          	// 0x01
   	AIT_REG_B   USB_LOP_CTL;                                          	// 0x02
   	AIT_REG_B   USB_UTMI_WRA_CTL;                                       // 0x03
   	AIT_REG_B   USB_PLL_CTL;                                          	// 0x04
   	AIT_REG_B   USB_XO_CTL;                                          	// 0x05
   	AIT_REG_B   USB_PHY_PROBE_SEL;                                      // 0x06
   	AIT_REG_B   USB_TRCV_TM_CTL;                                        // 0x07
   	AIT_REG_B   USB_POR_BYPASS;                                 		// 0x08
   	AIT_REG_B   USB_MODE_CTL;                                      		// 0x09
   	AIT_REG_B   USB_BIST_EN;                                          	// 0x0A
   	AIT_REG_B   USB_BIST_SR;                                          	// 0x0B
   	AIT_REG_B                   _0x0C[0x4];								//0x0C~0x0F reserved
   	
   	AIT_REG_B   USB_PHY_SPI_CTL0;                                       // 0x10
   	AIT_REG_B                   _0x11[0x1];								// 0x11
   	AIT_REG_B   USB_PHY_SPI_CTL1;                                       // 0x12
   	AIT_REG_B   USB_PHY_SPI_CTL2;                                       // 0x13
   	AIT_REG_B                   _0x14[0x28];							//0x14~0x3B reserved
   	AIT_REG_B   USB_OTG_TM;                                          	// 0x3C
   	AIT_REG_B   USB_OTG_CTL;                                          	// 0x3D
   	AIT_REG_B   USB_OTG_SRAM_CTL;                                       // 0x3E
   	AIT_REG_B                   _0x3F[0x1];								// 0x3F
   	
   	
   	AIT_REG_B   USB_UTMI_CTL0;                                         	// 0x40
   	AIT_REG_B   USB_UTMI_CTL1;                                          // 0x41
   	AIT_REG_B   USB_UTMI_CTL2;                                          // 0x42
   	AIT_REG_B   USB_UTMI_CTL3;                                          // 0x43
   	AIT_REG_B   USB_UTMI_CTL4;                                         	// 0x44
   	AIT_REG_B                   _0x45[0xB];
   	
   	
   	AIT_REG_B   USB_FARADAY_PHY_CTL0;                                   // 0x50
   	AIT_REG_B   USB_FARADAY_PHY_CTL1;                                   // 0x51
   	AIT_REG_B   USB_FARADAY_PHY_CTL2;                                   // 0x52
   	AIT_REG_B                   _0x53[0x11];							// 0x53~0x63 reserved
   	AIT_REG_D	USB_SOF_CNT;
   	#if (CHIP == VSN_V2)
   	AIT_REG_B                   _0x68[0x18];							// 0x68~0x7F reserved
   	AIT_REG_B	USB_DMA_CTL;											// 0x80
   	AIT_REG_B	USB_DMA_DESC_CNT;										// 0x81
   	#endif
   	#if (CHIP == VSN_V3)
   	AIT_REG_B	USB_CLK_GATED_SEL;										// 0x68
   	AIT_REG_B                   _0x69[0x17];							// 0x69~0x7F reserved
   	AIT_REG_B	USB_DMA1_CTL1;											// 0x80
   	AIT_REG_B	USB_DMA1_CTL2;											// 0x81
   	#endif
   	
   	AIT_REG_B	USB_DMA_INT_EN;											// 0x82
   		/*-DEFINE-----------------------------------------------------*/
   		#define USB_INT_DMA1_DONE_EN		0x01
   		#define USB_INT_DMA1_SET_TXPKTRDY	0x02
   		#if (CHIP == VSN_V3)
   		#define USB_INT_DMA1_DESC_CNT_EN	0x04
   		#define USB_INT_DMA2_DONE_EN		0x08
   		#define USB_INT_DMA2_SET_TXPKTRDY	0x10
   		#define USB_INT_DMA2_DESC_CNT_EN	0x20
   		#define USB_INT_DMA3_DONE_EN		0x40
   		#endif
   		/*------------------------------------------------------------*/
   	AIT_REG_B	USB_DMA_INT_SR;											// 0x83
   	
   	#if (CHIP == VSN_V2)
   	AIT_REG_D	USB_DMA_FB_ST_ADDR;										// 0x84
   	AIT_REG_B	USB_DMA_FIXED_DATA;										// 0x88
   	AIT_REG_B					_0x89[0x1];								// 0x89 reserved
   	AIT_REG_W	USB_DMA_CMD_ADDR;										// 0x8A
   	AIT_REG_B                   _0x8C[0x8];								// 0x8C~0x93 reserved
   	AIT_REG_D	USB_DMA_TAR_AND_VAL;									// 0x94
   	AIT_REG_D	USB_DMA_TAR_OR_VAL;										// 0x98
   	AIT_REG_B                   _0x9C[0x14];							// 0x9C~0xAF reserved
   	AIT_REG_B	USB_CLK_GATED_SEL;										// 0xB0
   	AIT_REG_B                   _0xB1[0x0F];							// 0xB1~0xBF reserved
   	#endif
   	#if (CHIP == VSN_V3)
   	AIT_REG_D	USB_DMA1_FB_ST_ADDR;									// 0x84
   	AIT_REG_B	USB_DMA1_FIXED_DATA;									// 0x88
   	AIT_REG_B					_0x89[0x1];								// 0x89 reserved
   	AIT_REG_W	USB_DMA1_CMD_ADDR;										// 0x8A
   	AIT_REG_W	USB_DMA1_DESC_CNT;										// 0x8C
   	AIT_REG_W	USB_DMA1_DESC_CNT_NUM;									// 0x8E
   	AIT_REG_W	USB_DMA1_PKT_BYTE_CNT;									// 0x90
   	AIT_REG_B                   _0x92[0x2];								// 0x92~0x93 reserved
   	AIT_REG_D	USB_DMA1_TAR_AND_VAL;									// 0x94
   	AIT_REG_D	USB_DMA1_TAR_OR_VAL;									// 0x98
   	AIT_REG_B                   _0x9C[0x4];								// 0x9C~0x9F reserved
   	AIT_REG_B	USB_DMA2_CTL1;											// 0xA0
   	AIT_REG_B	USB_DMA2_CTL2;											// 0xA1
   	AIT_REG_B                   _0xA2[0x2];								// 0xA2~0xA3 reserved
   	AIT_REG_D	USB_DMA2_FB_ST_ADDR;									// 0xA4
   	AIT_REG_B	USB_DMA2_FIXED_DATA;									// 0xA8
   	AIT_REG_B					_0xA9[0x1];								// 0xA9 reserved
   	AIT_REG_W	USB_DMA2_CMD_ADDR;										// 0xAA
   	AIT_REG_W	USB_DMA2_DESC_CNT;										// 0xAC
   	AIT_REG_W	USB_DMA2_DESC_CNT_NUM;									// 0xAE
   	AIT_REG_W	USB_DMA2_PKT_BYTE_CNT;									// 0xB0
   	AIT_REG_B                   _0xB2[0x2];								// 0xB2~0xB3 reserved
   	AIT_REG_D	USB_DMA2_TAR_AND_VAL;									// 0xB4
   	AIT_REG_D	USB_DMA2_TAR_OR_VAL;									// 0xB8
   	AIT_REG_B                   _0xBC[0x4];								// 0xBC~0xBF reserved
   	#endif
   	
   	AIT_REG_B	USB_INCOMPTX_CTL;										// 0xC0
   	AIT_REG_B					_0xC1[0x1];								// 0xC1 reserved
   	AIT_REG_B	USB_EP_DISABLE;											// 0xC2
   	AIT_REG_B                   _0xC3[0xD];								// 0xC3~0xCF reserved
   	AIT_REG_B	USB_DMA_INT2_EN;
   	AIT_REG_B	USB_DMA_INT2_SR;
   	#if (CHIP == VSN_V2)
   	AIT_REG_B					_0xD2[0x2];								// 0xD2~0xD3 reserved
   	AIT_REG_W	USB_PKT_BYTE_CNT;
   	#endif
   	#if (CHIP == VSN_V3)
   	AIT_REG_B					_0xD2[0xE];								// 0xD2~0xDF reserved
   	AIT_REG_B	USB_DMA3_CTL1;											// 0xE0
   	AIT_REG_B	USB_DMA3_POST_WAIT;										// 0xE1
   	AIT_REG_B	USB_DMA3_CTL2;											// 0xE2
   	AIT_REG_B					_0xE3[0x1];								// 0xE3 reserved
   	AIT_REG_D	USB_DMA3_FB_ST_ADDR;									// 0xE4
   	AIT_REG_W	USB_DMA3_FIFO_ADDR;										// 0xE8
   	AIT_REG_W	USB_DMA3_CMD_ADDR;										// 0xEA
   	AIT_REG_W	USB_DMA3_PKT_BYTE;										// 0xEC
   	AIT_REG_W	USB_DMA3_PKT_BYTE_LAST;									// 0xEE
   	AIT_REG_D	USB_DMA3_PKT_SUM;										// 0xF0
   	AIT_REG_D	USB_DMA3_TAR_AND_VAL;									// 0xF4
   	AIT_REG_D	USB_DMA3_TAR_OR_VAL;									// 0xF8
   	#endif
} AITS_USB_DMA, *AITPS_USB_DMA;

union USB_FIFO_RW{
	AIT_REG_B FIFO_B;
	AIT_REG_W FIFO_W;
	AIT_REG_D FIFO_D;
};

typedef struct _AITS_USB_EP {
   	AIT_REG_W   USB_EP_TX_MAXP;
   	AIT_REG_W   USB_EP_TX_CSR; 
  	AIT_REG_W   USB_EP_RX_MAXP; 
   	AIT_REG_W   USB_EP_RX_CSR; 
   	AIT_REG_W   USB_EP_COUNT;
   	AIT_REG_B                   _0x9[0x5]; 
   	AIT_REG_B   USB_CFG_DATA; 
} USB_EP_CTL, *USBS_EP_CTL;

// *****************************************************************************
//   USB DMA register structure (0x8000 A800)
// *****************************************************************************
typedef struct _AITS_USB_CTL {
   	AIT_REG_B   USB_FADDR;                                          		// 0x00
   	AIT_REG_B   USB_POWER;                                          		// 0x01
   	AIT_REG_W   USB_TX_INT_SR;                                          	// 0x02
   	AIT_REG_W   USB_RX_INT_SR;                                          	// 0x04
   	AIT_REG_W   USB_TX_INT_EN;                                          	// 0x06
   	AIT_REG_W   USB_RX_INT_EN;                                          	// 0x08
   	AIT_REG_B   USB_INT_EVENT_SR;                                          // 0x0A
   	AIT_REG_B   USB_INT_EVENT_EN;                                          	// 0x0B
   	AIT_REG_W   USB_FRAME_NUM;                                          	// 0x0C
   	AIT_REG_B   USB_INDEX_EP_SEL;                                          	// 0x0E
   	AIT_REG_B   USB_TESTMODE;                                          		// 0x0F
   	USB_EP_CTL	USB_INDEX_EP[0x1];  	   									// 0x10
   	//#if (CHIP == VSN_V2)
   	//union USB_FIFO_RW   USB_FIFO_EP[0x6];                                   // 0x20
   	//AIT_REG_B                   _0x38[0x34]; 								// 0x38~0x6B
   	//#endif
   //	#if (CHIP == VSN_V3)
   	union USB_FIFO_RW   USB_FIFO_EP[0x8];                                   // 0x20
   	AIT_REG_B                   _0x40[0x2C]; 								// 0x40~0x6B
   //	#endif
   	AIT_REG_W   USB_ADT_HW_VER;                                     		// 0x6C
   	AIT_REG_B                   _0x6E[0xA]; 
   	AIT_REG_B   USB_ADT_EP_INFO;                                          	// 0x78
   	AIT_REG_B   USB_ADT_RAM_INFO;                                          	// 0x79
   	AIT_REG_B   USB_ADT_LINK_INFO;                                          // 0x7A
   	AIT_REG_B   USB_ADT_VP_LEN;                                          	// 0x7B
   	AIT_REG_B   USB_ADT_HS_EOF;                                          	// 0x7C
   	AIT_REG_B   USB_ADT_FS_EOF;                                          	// 0x7D
   	AIT_REG_B   USB_ADT_LS_EOF;                                          	// 0x7E
   	AIT_REG_B   USB_ADT_SOFT_RST;                                          	// 0x7F
   	AIT_REG_B                   _0x80[0x80];
   	//#if (CHIP == VSN_V2)
   	//USB_EP_CTL	USB_EP[6];													// 0x100
   	//AIT_REG_B                   _0x140[0x1E0]; 
   	//#endif
   	//#if (CHIP == VSN_V3)
   	USB_EP_CTL	USB_EP[8];													// 0x100
   	AIT_REG_B                   _0x160[0x1C0];
   	//#endif
   	AIT_REG_W   USB_RX_DPKBUFDIS;                                          	// 0x340
   	AIT_REG_W   USB_TX_DPKBUFDIS;                                          	// 0x342
   	AIT_REG_W   USB_UCH;                                          			// 0x344
   	AIT_REG_W   USB_HSRTN;                                          		// 0x346
  
} AITS_USB_CTL, *AITPS_USB_CTL;
#endif

#if (CHIP == MCR_V2)

// *****************************************************************************
//   USB DMA control register structure
//   P_V2:      (0x8000 6000)
//   MCR_V2:    (0x8000 1400)
// *****************************************************************************
/*
 *   DMA control settings
 *
 *   P_V2:   DMA (0x6080~0x609F)
 *   MCR_V2: DMA1 (0x1480~0x149F); DMA2 (0x14A0~0x14BF); DMA3 (0x14E0~0x14FF)
 */

typedef struct _USB_DMA_CTL {
    AIT_REG_B   CTL1;                                           // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define USB_DMA_EN              0x01
        #define USB_DMA_STOP            0x02
        #define USB_DMA_RX              0x04
        #if (CHIP == MCR_V2)
        // ++MCR_V2_MP only
        #define USB_DMA3_DBG_EN         0x08
        // --MCR_V2_MP only
        #endif
        #define USB_DMA_TX_ZLP          0x10
        #if (CHIP == P_V2)
        #define USB_DMA_EP1_3(ep)       (0x1 << (ep + 4))
        #endif
        #if (CHIP == MCR_V2)
        #define USB_DMA_EP(ep)          ((ep & 0x7) << 5)
        #endif
        /*------------------------------------------------------------*/
    AIT_REG_B   CTL2;                                           // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define USB_DMA_DELAY_WR        0x01
        #if (CHIP == MCR_V2)
        #define USB_NO_WAIT_IN_FOR_TX   0x01
        #endif
        /*------------------------------------------------------------*/
    #if (CHIP == P_V2)
    AIT_REG_B   INT_EN;                                         // 0x02
    #endif
    #if (CHIP == MCR_V2)
    __packed union {
        AIT_REG_B   INT_EN;
        AIT_REG_B   DMA3_CTL2;
    } CTL;
    #endif
        /*-DEFINE-----------------------------------------------------*/
        #if (CHIP == P_V2)
        #define USB_INT_DMA_DIS         0x00
        #define USB_INT_DMA_EN          0x01
        #endif
        #if (CHIP == MCR_V2)
        // ++MCR_V2_MP only
        #define USB_DMA3_CMD_ERR        0x02
        // --MCR_V2_MP only
        #endif
        /*------------------------------------------------------------*/
    AIT_REG_B   INT_SR;                                         // 0x03
        /*-DEFINE-----------------------------------------------------*/
        #if (CHIP == P_V2)
        #define USB_INT_DMA_DONE        0x01
        #endif
        #if (CHIP == MCR_V2)
        #define USB_INT_DMA1_DONE       0x01
        #define USB_INT_DMA1_TXPKTRDY   0x02
        #define USB_INT_DMA1_DESC_CNT   0x04
        #define USB_INT_DMA2_DONE       0x08
        #define USB_INT_DMA2_TXPKTRDY   0x10
        #define USB_INT_DMA2_DESC_CNT   0x20
        #define USB_INT_DMA3_DONE       0x40
        #define USB_INT_LPM             0x80
        #endif
        /*------------------------------------------------------------*/
    __packed union {
        /* DMA normal mode, for PythonV2 DMA & MercuryV2 DMA3 */
        __packed struct {
			AIT_REG_D   FB_ST_ADDR;                             // 0x04
			AIT_REG_W   FIFO_ADDR;                              // 0x08
			AIT_REG_W   CMD_ADDR;                               // 0x0A
            AIT_REG_W   PKT_BYTE;                               // 0x0C
            AIT_REG_W   PKT_BYTE_LAST;                          // 0x0E
            AIT_REG_D   PKT_SUM;                                // 0x10
        } NORM;

        #if (CHIP == MCR_V2)
        /* DMA description mode, for DMA1 & DMA2 of MercuryV2 */
        __packed struct {
            AIT_REG_D   DESC_ST_ADDR;                           // 0x04
			AIT_REG_B   FIXED_DATA;                             // 0x08
            AIT_REG_B   IBC_OVERFLOW_SR;                        // 0x09
			AIT_REG_W   CMD_ADDR;                               // 0x0A
            AIT_REG_W   DESC_DONE_CNT;                          // 0x0C
            AIT_REG_W   DESC_INT_CNT;                           // 0x0E
            AIT_REG_W   PKT_BYTE_CNT;                           // 0x10
            AIT_REG_B   _0x92[0x02];
        } DESC;
        #endif
    } MODE;
    AIT_REG_D   TAR_AND_VAL;                                    // 0x14
    AIT_REG_D   TAR_OR_VAL;                                     // 0x18
    #if (CHIP == P_V2)
    AIT_REG_D   PROBE;                                          // 0x1C
    #endif
    #if (CHIP == MCR_V2)
    __packed union {
        AIT_REG_D       _0x1C;                                  // 0x1C
        // ++MCR_V2_MP only
        AIT_REG_D       ERR_PKT_SUM;                            // 0x1C
        // --MCR_V2_MP only
    } DMA3_STAT;
    #endif
} USB_DMA_CTL, *USBS_DMA_CTL;

typedef struct _AITS_USB_DMA {
    AIT_REG_B   USB_CTL_SEL;                                    // 0x00
    AIT_REG_B   USB_SIDDQ;                                      // 0x01
    AIT_REG_B   USB_LOP_CTL;                                    // 0x02
    AIT_REG_B   USB_UTMI_WRA_CTL;                               // 0x03
    AIT_REG_B   USB_PLL_CTL;                                    // 0x04
    AIT_REG_B   USB_XO_CTL;                                     // 0x05
    AIT_REG_B   USB_PHY_PROBE_SEL;                              // 0x06
    AIT_REG_B   USB_TRCV_TM_CTL;                                // 0x07
    AIT_REG_B   USB_POR_BYPASS;                                 // 0x08
    AIT_REG_B   USB_MODE_CTL;                                   // 0x09
    AIT_REG_B   USB_BIST_EN;                                    // 0x0A
    AIT_REG_B   USB_BIST_SR;                                    // 0x0B
    AIT_REG_B                   _0x0C[0x4];                     // 0x0C~0x0F

    AIT_REG_B   USB_PHY_SPI_CTL0;                               // 0x10
    AIT_REG_B                   _0x11[0x1];                     // 0x11
    AIT_REG_B   USB_PHY_SPI_CTL1;                               // 0x12
        /*-DEFINE-----------------------------------------------------*/
        #define OPR_SS_LOW              0x00
        #define OPR_SS_HIGH             0x01
        #define OPR_SCLK_LOW            0x00
        #define OPR_SCLK_HIGH           0x02
        #define OPR_MOSI_LOW            0x00
        #define OPR_MOSI_HIGH           0x04
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_PHY_SPI_CTL2;                               // 0x13
        /*-DEFINE-----------------------------------------------------*/
        #define OPR_MISO_LOW            0x00
        #define OPR_MISO_HIGH           0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0x14[0x28];                    // 0x14~0x3B
    AIT_REG_B   USB_TEST_MODE;                                  // 0x3C
    AIT_REG_B   USB_OTG_CTL;                                    // 0x3D
        /*-DEFINE-----------------------------------------------------*/
        #define OTG_EN                  0x01
        #define AIT_DMA_MODE            0x00
        #define OTG_AVALID_TRUE         0x02
        #define OTG_VBUSVALID_TRUE      0x04
        #define MENTER_DMA_MODE         0x08
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_OTG_SRAM_CTL;                               // 0x3E
    AIT_REG_B                   _0x3F[0x1];                     // 0x3F

    AIT_REG_B   USB_UTMI_CTL0;                                  // 0x40
    AIT_REG_B   USB_UTMI_CTL1;                                  // 0x41
        /*-DEFINE-----------------------------------------------------*/
        #define UTMI_IDPULLUP           0x01
        #define UTMI_DPPULLDOWN         0x02
        #define UTMI_DMPULLDOWN         0x04
        #define UTMI_DRVVBUS            0x08
        #define UTMI_CHRGVBUS           0x10
        #define UTMI_DISCHRGVBUS        0x20
        #define UTMI_USBPHY_NOSUSPEND   0x40
        #define UTMI_USBPHY_TESTMODE    0x80
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_UTMI_CTL2;                                  // 0x42
    AIT_REG_B   USB_UTMI_CTL3;                                  // 0x43
    #if (CHIP == P_V2)
    AIT_REG_B                   _0x44[0xC];                     // 0x44~0x4F
    #endif
    #if (CHIP == MCR_V2)
    AIT_REG_B   USB_UTMI_CTL4;                                  // 0x44
        /*-DEFINE-----------------------------------------------------*/
        #define RXERROR_FROM_PHY        0x00
        #define RXERROR_TIE_LOW         0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0x45[0xB];                     // 0x45~0x4F
    #endif

    AIT_REG_B   USB_UTMI_PHY_CTL0;                              // 0x50
        /*-DEFINE-----------------------------------------------------*/
        #define CLKOSC_OFF_IN_SUSPEND   0x40
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_UTMI_PHY_CTL1;                              // 0x51
        /*-DEFINE-----------------------------------------------------*/
        #define UTMI_XTL_12MHZ          0x00
        #define UTMI_XTL_30MHZ          0x04
        #define UTMI_PLL_ON_IN_SUSPEND  0x08
        #define UTMI_OUTCLK_XSCI        0x00
        #define UTMI_OUTCLK_PLL         0x10
        #define UTMI_DATA_BUS_16BIT     0x00
        #define UTMI_DATA_BUS_8BIT      0x20
        #define DMPULLDOWN_TO_PHY_DIS   0x00
        #define DMPULLDOWN_TO_PHY_EN    0x40
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_UTMI_PHY_CTL2;                              // 0x52
    #if (CHIP == P_V2)
    AIT_REG_B                   _0x53[0x25];                    // 0x53~0x77
    AIT_REG_B   USB_INTER_CTL;                                  // 0x78
        /*-DEFINE-----------------------------------------------------*/
        #define RXERROR_FROM_PHY        0x00
        #define RXERROR_TIE_LOW         0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0x79[0x7];                     // 0x79~0x7F
    #endif
    #if (CHIP == MCR_V2)
    AIT_REG_B                   _0x53[0xD];                     // 0x53~0x5F
    AIT_REG_B   USB_SOF_CNT_CTL;                                // 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define SOF_CNT_CLR             0x01
        #define SOF_CLK_EN              0x02
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0x61[0x03];                    // 0x61~0x63
    AIT_REG_D   USB_SOF_CNT;                                    // 0x64
    AIT_REG_B   USB_CLK_GATED_SEL;                              // 0x68
    AIT_REG_B                   _0x69[0x7];                     // 0x69~0x6F

    AIT_REG_D   USB_DMA1_IBC_RING_SIZE;                         // 0x70
    AIT_REG_D   USB_DMA2_IBC_RING_SIZE;                         // 0x74
    AIT_REG_B                   _0x78[0x8];                     // 0x78~0x7F
    #endif

    USB_DMA_CTL USB_DMA;                                        // 0x80~0x9F
    #if (CHIP == P_V2)
    AIT_REG_B   USB_DMA_EP_CTL2;                                // 0xA0
        /*-DEFINE-----------------------------------------------------*/
        #define USB_DMA_EP4_6(ep)       (0x1 << (ep - 4))
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0xA1[0x5F];                    // 0xA1~0xFF
    #endif
    #if (CHIP == MCR_V2)
    USB_DMA_CTL USB_DMA2;                                       // 0xA0~0xBF
    AIT_REG_B   USB_INCOMPTX_CTL;                               // 0xC0
    AIT_REG_B                   _0xC1[0x1];                     // 0xC1
    AIT_REG_B   USB_EP_DIS;                                     // 0xC2
        /*-DEFINE-----------------------------------------------------*/
        #define USB_EP_DIS(ep)          (1 << ep)
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0xC3[0xD];                     // 0xC3~0xCF
    AIT_REG_B   USB_DMA_INT2_EN;                                // 0xD0
    AIT_REG_B   USB_DMA_INT2_SR;                                // 0xD1
    AIT_REG_B                   _0xD2[0x2];                     // 0xD2~0xD3
    // ++MCR_V2_MP only
    AIT_REG_D   LAST_TAR_AND_VAL;                               // 0xD4
    AIT_REG_D   LAST_TAR_OR_VAL;                                // 0xD8
    // --MCR_V2_MP only
    AIT_REG_B                   _0xDC[0x4];                     // 0xDC~0xDF
    USB_DMA_CTL USB_DMA3;                                       // 0xE0~0xFF
    #endif
} AITS_USB_DMA, *AITPS_USB_DMA;

typedef union _USB_FIFO_RW {
    AIT_REG_B FIFO_B;
    AIT_REG_W FIFO_W;
    AIT_REG_D FIFO_D;
} USB_FIFO_RW;

typedef struct _USB_MULTIPOINT {
    AIT_REG_B   EP_TX_FUNC_ADDR;                                // 0x00
    AIT_REG_B                   _0x01;
    AIT_REG_B   EP_TX_HUB_ADDR;                                 // 0x02
    AIT_REG_B   EP_TX_HUB_PORT;                                 // 0x03
    AIT_REG_B   EP_RX_FUNC_ADDR;                                // 0x04
    AIT_REG_B                   _0x05;
    AIT_REG_B   EP_RX_HUB_ADDR;                                 // 0x06
    AIT_REG_B   EP_RX_HUB_PORT;                                 // 0x07
} USB_MULTIPOINT;

typedef struct _AITS_USB_EP {
    AIT_REG_W   USB_EP_TX_MAXP;                                 // 0x00
    AIT_REG_W   USB_EP_TX_CSR;                                  // 0x02
        /*-DEFINE-----------------------------------------------------*/
        /* For EP0 only */
        #define EP0_RX_PKTRDY           0x0001
        #define EP0_TX_PKTRDY           0x0002
        #define EP0_STALLED             0x0004
        #define EP0_CLR_STALL           EP0_STALLED
        #define EP0_FLUSH_FIFO          0x0100
        /* EP0 in peripheral mode */
        #define PHL_EP0_DATA_END        0x0008
        #define PHL_EP0_SETUP_END       0x0010
        #define PHL_EP0_STALL           0x0020
        #define PHL_EP0_SERVICE_RX      0x0040
        #define PHL_EP0_SERVICE_SETUP   0x0080
        #define PHL_EP0_SETUP_SERVICED  (PHL_EP0_SERVICE_SETUP|EP0_STALLED)
        #define PHL_EP0_RX_SERVICED     (PHL_EP0_SERVICE_RX|EP0_STALLED)
        #define PHL_EP0_SET_TX_PKYRDY   (EP0_TX_PKTRDY|EP0_STALLED)
        #define PHL_EP0_SET_STALL       (PHL_EP0_STALL|EP0_STALLED)
        /* EP0 in host mode */
        #define HOST_EP0_SETUP          0x0008
        #define HOST_EP0_ERR_SR         0x0010
        #define HOST_EP0_REQ_PKT        0x0020
        #define HOST_EP0_STATUS         0x0040
        #define HOST_EP0_NAK_TO         0x0080
        #define HOST_EP0_DATATOG        0x0200
        #define HOST_EP0_DATATOG_WR     0x0400
        #define HOST_EP0_PING_DIS       0x0800
        /* For all EPs except EP0 */
        #define EP_TX_PKTRDY            0x0001
        #define EP_TX_FIFO_NOTEMPTY     0x0002
        #define EP_TX_FLUSH_FIFO        0x0008
        #define EP_TX_STALLED           0x0020
        #define EP_TX_CLR_STALL         EP_TX_STALLED
        #define EP_TX_CLR_DATATOG       0x0040
        #define EP_TX_DMAREQ_MODE       0x0400
        #define EP_TX_FORCE_DATATOG     0x0800
        #define EP_TX_DMAREQ_EN         0x1000
        #define EP_TX_MODE              0x2000
        #define EP_TX_AUTO_SET          0x8000
        /* EPs in peripheral mode */
        #define PHL_EP_TX_UNDERRUN      0x0004
        #define PHL_EP_TX_STALL         0x0010
        #define PHL_EP_TX_ISO_INCOMP    0x0080
        #define PHL_EP_TX_ISO           0x4000
        #define PHL_EP_TX_STATUS        (PHL_EP_TX_ISO_INCOMP|EP_TX_STALLED|\
                                         PHL_EP_TX_UNDERRUN|EP_TX_FIFO_NOTEMPTY)
        #define PHL_EP_TX_CLR_DATATOG   (PHL_EP_TX_STATUS|EP_TX_CLR_DATATOG)
        #define PHL_EP_TX_SET_PKYRDY    (PHL_EP_TX_STATUS|EP_TX_PKTRDY)
        #define PHL_EP_TX_CLR_STALLED   (PHL_EP_TX_ISO_INCOMP|\
                                         PHL_EP_TX_UNDERRUN|EP_TX_FIFO_NOTEMPTY)
        #define PHL_EP_TX_FLUSH_FIFO    (PHL_EP_TX_STATUS|EP_TX_FLUSH_FIFO)
        #define PHL_EP_TX_SET_STALL     (PHL_EP_TX_ISO_INCOMP|EP_TX_STALLED|\
                                         PHL_EP_TX_STALL|EP_TX_PKTRDY)
        #define PHL_EP_TX_CSR_MASK     ~(PHL_EP_TX_STATUS|EP_TX_CLR_DATATOG|\
                                         EP_TX_FLUSH_FIFO|EP_TX_PKTRDY)
        /* EP0 in host mode */
        #define HOST_EP_TX_ERR_SR       0x0004
        #define HOST_EP_TX_SETUP        0x0010
        #define HOST_EP_TX_BULK_NAK_TO  0x0080
        #define HOST_EP_TX_ISO_INCOMP   HOST_EP_TX_BULK_NAK_TO
        #define HOST_EP_TX_DATATOG      0x0100
        #define HOST_EP_TX_DATATOG_WR   0x0200
        /*------------------------------------------------------------*/
    AIT_REG_W   USB_EP_RX_MAXP;                                 // 0x04
    AIT_REG_W   USB_EP_RX_CSR;                                  // 0x06
        /*-DEFINE-----------------------------------------------------*/
        /* For all EPs except EP0 */
        #define EP_RX_PKTRDY            0x0001
        #define EP_RX_FIFO_FULL         0x0002
        #define EP_RX_ISO_DATA_ERR      0x0008
        #define EP_RX_FLUSH_FIFO        0x0010
        #define EP_RX_STALLED           0x0040
        #define EP_RX_CLR_STALL         EP_RX_STALLED
        #define EP_RX_CLR_DATATOG       0x0080
        #define EP_RX_ISO_INCOMP        0x0100
        #define EP_RX_DMAREQ_MODE       0x0800
        #define EP_RX_PID_ERR           0x1000
        #define EP_RX_DMAREQ_EN         0x2000
        #define EP_RX_AUTO_CLR          0x8000
        /* EPs in peripheral mode */
        #define PHL_EP_RX_OVERRUN       0x0004
        #define PHL_EP_RX_STALL         0x0020
        #define PHL_EP_RX_NYET_DIS      EP_RX_PID_ERR
        #define PHL_EP_RX_ISO           0x4000
        #define PHL_EP_RX_STATUS        (EP_RX_ISO_INCOMP|EP_RX_STALLED|\
                                         PHL_EP_RX_OVERRUN|EP_RX_PKTRDY)
        #define PHL_EP_RX_CLR_DATATOG   (PHL_EP_RX_STATUS|EP_RX_CLR_DATATOG)
        #define PHL_EP_RX_SET_STALL     (PHL_EP_RX_STATUS|PHL_EP_RX_STALL)
        #define PHL_EP_RX_CLR_STALLED   (EP_RX_ISO_INCOMP|\
                                         PHL_EP_RX_OVERRUN|EP_RX_PKTRDY)
        #define PHL_EP_RX_CLR_PKYRDY    (EP_RX_ISO_INCOMP|EP_RX_STALLED|\
                                         PHL_EP_RX_OVERRUN)
        #define PHL_EP_RX_CSR_MASK     ~(PHL_EP_RX_CLR_DATATOG|EP_RX_FLUSH_FIFO|\
                                         EP_RX_ISO_DATA_ERR|EP_RX_FIFO_FULL)
        /* EP0 in host mode */
        #define HOST_EP_RX_ERR_SR       0x0004
        #define HOST_EP_RX_BULK_NAK_TO  EP_RX_ISO_DATA_ERR
        #define HOST_EP_RX_REQ_PKT      0x0020
        #define HOST_EP_RX_DATATOG      0x0200
        #define HOST_EP_RX_DATATOG_WR   0x0400
        #define HOST_EP_RX_AUTO_REQ     0x4000
        /*------------------------------------------------------------*/
    AIT_REG_W   USB_EP_COUNT;                                   // 0x08
    AIT_REG_B   USB_EP_TYPE;                                    // 0x0A
        /*-DEFINE-----------------------------------------------------*/
        /* Host mode only */
        #define EP_TARGET_NUM_MASK      0x0F
        #define EP_XFER_TYPE_CTL        0x00
        #define EP_XFER_TYPE_ISO        0x10
        #define EP_XFER_TYPE_BULK       0x20
        #define EP_XFER_TYPE_INT        0x30
        #define EP_XFER_TYPE_MASK       0x30
        #define EP_XFER_SPEED_HIGH      0x40
        #define EP_XFER_SPEED_FULL      0x80
        #define EP_XFER_SPEED_LOW       0xC0
        #define EP_XFER_SPEED_MASK      0xC0
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_EP_TX_INTVL;                                // 0x0B
    AIT_REG_B   USB_EP_RX_TYPE;                                 // 0x0C
    AIT_REG_B   USB_EP_RX_INTVL;                                // 0x0D
    AIT_REG_B                   _0xE;
    AIT_REG_B   USB_CFG_DATA;                                   // 0x0F
} USB_EP_CTL, *USBS_EP_CTL;

// *****************************************************************************
//   USB base control register structure
//   P_V2:      (0x8000 A800)
//   MCR_V2:    (0x8000 1000)
// *****************************************************************************
typedef struct _AITS_USB_CTL {
    AIT_REG_B   USB_FADDR;                                      // 0x00
    AIT_REG_B   USB_POWER;                                      // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define USB_SUSPENDM_EN     0x01
        #define USB_SUSPEND_MODE    0x02
        #define USB_RESUME          0x04
        #define USB_RESET           0x08
        #define USB_HS_MODE         0x10
        #define USB_HS_EN           0x20
        /* For peripheral mode */
        #define USB_DEV_SOFT_CONN   0x40
        #define USB_DEV_ISO_UPD     0x80
        /*------------------------------------------------------------*/
    AIT_REG_W   USB_TX_INT_SR;                                  // 0x02
    AIT_REG_W   USB_RX_INT_SR;                                  // 0x04
    AIT_REG_W   USB_TX_INT_EN;                                  // 0x06
    AIT_REG_W   USB_RX_INT_EN;                                  // 0x08
        /*-DEFINE-----------------------------------------------------*/
        #define USB_INT_EP0         0x01
        #define USB_INT_TX_EP(ep)   (1 << (ep & 0x7))
        #define USB_INT_RX_EP(ep)   (1 << (ep & 0x7))
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_INT_EVENT_SR;                               // 0x0A
    AIT_REG_B   USB_INT_EVENT_EN;                               // 0x0B
        /*-DEFINE-----------------------------------------------------*/
        #define USB_INT_SUSPEND     0x01
        #define USB_INT_RESUME      0x02
        #define USB_INT_RESET       0x04
        #define USB_INT_BABBLE      0x04
        #define USB_INT_SOF         0x08
        #define USB_INT_CONN        0x10
        #define USB_INT_DISCON      0x20
        #define USB_INT_SESS_REQ    0x40
        #define USB_INT_VBUS_ERR    0x80
        #define USB_INT_STATUS_MASK    0xff
        /*------------------------------------------------------------*/
    AIT_REG_W   USB_FRAME_NUM;                                  // 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define USB_FRAME_NUM_MASK  0x07FF
        /*------------------------------------------------------------*/
    AIT_REG_B   USB_INDEX_EP_SEL;                               // 0x0E
    AIT_REG_B   USB_TESTMODE;                                   // 0x0F
        /*-DEFINE-----------------------------------------------------*/
        #define USB_TEST_SE0_NAK    0x01
        #define USB_TEST_J          0x02
        #define USB_TEST_K          0x04
        #define USB_TEST_PACKET     0x08
        #define USB_FORCE_HS        0x10
        #define USB_FORCE_FS        0x20
        #define USB_FIFO_ACCESS     0x40
        #define USB_FORCE_HOST      0x80
        /*------------------------------------------------------------*/

    USB_EP_CTL  USB_INDEX_EP;                                   // 0x10
    #if (CHIP == P_V2)
    USB_FIFO_RW USB_FIFO_EP[0x7];                               // 0x20
    AIT_REG_B                   _0x3C[0x24];                    // 0x3C~0x5F
    #endif
    #if (CHIP == MCR_V2)
    USB_FIFO_RW USB_FIFO_EP[0x8];                               // 0x20
    AIT_REG_B                   _0x40[0x20];                    // 0x40~0x5F
    #endif

    AIT_REG_B   USB_ADT_DEV_CTL;                                // 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define USB_SESSION         0x01
        #define USB_HOST_REQ        0x02
        #define USB_HOST_MODE       0x04
        #define USB_LSDEV           0x20
        #define USB_FSDEV           0x40
        #define USB_B_DEVICE        0x80
        /*------------------------------------------------------------*/
    AIT_REG_B                   _0x61[0x0B];                    // 0x61~0x6B
    AIT_REG_W   USB_ADT_HW_VER;                                 // 0x6C
    AIT_REG_B                   _0x6E[0xA];                     // 0x6E~0x77
    AIT_REG_B   USB_ADT_EP_INFO;                                // 0x78
    AIT_REG_B   USB_ADT_RAM_INFO;                               // 0x79
    AIT_REG_B   USB_ADT_LINK_INFO;                              // 0x7A
    AIT_REG_B   USB_ADT_VP_LEN;                                 // 0x7B
    AIT_REG_B   USB_ADT_HS_EOF;                                 // 0x7C
    AIT_REG_B   USB_ADT_FS_EOF;                                 // 0x7D
    AIT_REG_B   USB_ADT_LS_EOF;                                 // 0x7E
    AIT_REG_B   USB_ADT_SOFT_RST;                               // 0x7F
    #if (CHIP == P_V2)
    USB_MULTIPOINT  USB_MP_CTL[0x7];                            // 0x80~0xB7
    AIT_REG_B                   _0xB8[0x48];                    // 0xB8~0xFF
    #endif
    #if (CHIP == MCR_V2)
    USB_MULTIPOINT  USB_MP_CTL[0x8];                            // 0x80~0xBF
    AIT_REG_B                   _0xC0[0x40];                    // 0xC0~0xFF
    #endif

    #if (CHIP == P_V2)
    USB_EP_CTL  USB_EP[0x7];                                    // 0x100
    AIT_REG_B                   _0x170[0x190];                  // 0x170~0x2FF
    AIT_REG_W   USB_EP_RQ_PKT_CNT[0x7];                         // 0x300~0x30D
    AIT_REG_B                   _0x30E[0x32];                   // 0x30E~0x33F
    #endif
    #if (CHIP == MCR_V2)
    USB_EP_CTL  USB_EP[0x8];                                    // 0x100
    AIT_REG_B                  _0x180[0x180];
    AIT_REG_W   USB_EP_RQ_PKT_CNT[0x8];                         // 0x300~0x30F
    AIT_REG_B                   _0x30E[0x30];                   // 0x310~0x33F
    #endif
    AIT_REG_W   USB_RX_DPKTBUFDIS;                              // 0x340
    AIT_REG_W   USB_TX_DPKTBUFDIS;                              // 0x342
    AIT_REG_W   USB_C_T_UCH;                                    // 0x344
    AIT_REG_W   USB_C_T_HSRTN;                                  // 0x346
    AIT_REG_B                   _0x348[0xB8];                   // 0x348~0x3FF
} AITS_USB_CTL, *AITPS_USB_CTL;
#endif
////////////////////////////////////
// Register definition
//

#if !defined(BUILD_FW)
// DMA OPR

#if (CHIP == P_V2)||(CHIP == VSN_V2)||(CHIP == VSN_V3)

#endif

#endif
/// @}

#endif // _MMPH_REG_USB_H_
