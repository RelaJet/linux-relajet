//==============================================================================
//
//  File        : mmp_reg_display.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_DISPLAY_H_
#define _MMP_REG_DISPLAY_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

#if (CHIP == P_V2)
// *******************************
//   Display structure (0x8000 7000)
// *******************************
typedef struct _AITS_DSPY {
    AIT_REG_W   DSPY_CTL_0;                                             // 0x0000
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_FRAME_TX                0x0002
        #define LCD_IDX_RDY                 0x0004
        #define LCD_CMD_RDY                 0x0008
        #define LCD_FRAME_TX_SETADDR_EN     0x0010
        #define LCD_PANEL_READ_EN          	0x0040
        #define LCD_BUSY_STATUS             0x0080
        #define SCD_DSPY_REG_READY          0x0100
        #define SW_UPDATE_PRM_OPR_NOW		0x8000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_CTL_2;                                             // 0x0002
        /*-DEFINE-----------------------------------------------------*/
        #define PRM_DSPY_REG_READY          0x0001
        #define LCD_A0_DIS                  0x0002
        #define LCD_RD_DIS                  0x0004
        #define LCD_WR_DIS                  0x0008
        #define LCD_CS2_DIS                 0x0010
        #define LCD_CS1_DIS                 0x0020
        #define TV_FIEND_SYNC_EN            0x0040
        #define TV_FRAME_SYNC_EN			0x0000
        #define TV_SYNC_OPR_DIS          	0x0080
 
		#define DSPY_TYPE_PL_LCD            0x00
        #define DSPY_TYPE_SPI_LCD           0x01
        #define DSPY_TYPE_RGB_LCD           0x02
        #define DSPY_TYPE_TV                0x03
        #define DSPY_TYPE_PRM_DSI           0x40
        #define DSPY_TYPE_SCD_DSI           0x20
        #define DSPY_PRM_SEL_SHIFT          8
        #define DSPY_PRM_SEL(_a)            (MMP_USHORT)(_a << DSPY_PRM_SEL_SHIFT)
        #define DSPY_PRM_SEL_MASK           0x4300
        #define DSPY_PRM_EN                 0x1000
        #define DSPY_SCD_SEL_SHIFT          10
        #define DSPY_SCD_SEL(_a)            (MMP_USHORT)(_a << DSPY_SCD_SEL_SHIFT)
        #define DSPY_SCD_SEL_MASK           0x8C00
        #define DSPY_SCD_EN                 0x2000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_INT_HOST_EN;                                       // 0x0004
    AIT_REG_W   DSPY_INT_HOST_SR;										// 0x0006
    AIT_REG_W   DSPY_INT_CPU_EN;										// 0x0008
    AIT_REG_W   DSPY_INT_CPU_SR;										// 0x000A
        /*-DEFINE-----------------------------------------------------*/
        #define PRM_FRME_TX_END             0x0001
        #define PRM_IDX_TX_END              0x0002
        #define PRM_CMD_TX_END              0x0004
        #define SCD_FRME_TX_END             0x0008
        #define SCD_IDX_TX_END              0x0010
        #define SCD_CMD_TX_END              0x0020
        #define RGB_VSYNC_ACTIVE            0x0040
        #define RGB_FRME_CNT_HIT            0x0080
        #define TV_LINE_INT1                0x0100
        #define TV_LINE_INT2                0x0200
        #define TV_READ_DSPY_BUF_UDF    	0x0400
        #define TV_ODD_FIELD_START          0x0800
        #define TV_EVEN_FIELD_START         0x1000
        #define TV_COMP_OUT_L2H             0x2000
        #define TV_COMP_OUT_H2L             0x4000
        #define LCD_FIFO_FINISHED			0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_LCD_FLM_CTL;                                       // 0x000C
    	/*-DEFINE-----------------------------------------------------*/
    	#define LCD_CHK_FLM_EN				0x01
    	#define FLM_SIG_ACT_LOW				0x02
    	#define FLM_SIG_ACT_HIGH			0x00
    	/*------------------------------------------------------------*/
    AIT_REG_B   DSPY_LCD_VSYNC_CTL;                                     // 0x000D
		/*-DEFINE-----------------------------------------------------*/
		#define LCD_OUT_VSYNC_EN			0x01
    	#define VSYNC_SIG_ACT_LOW			0x02
    	#define VSYNC_SIG_ACT_HIGH			0x00
		/*------------------------------------------------------------*/
    AIT_REG_W   DSPY_FLM_VSYNC_CNT;                                     // 0x000E

    AIT_REG_D   DSPY_LCD_TX_0;                                          // 0x0010
    AIT_REG_D   DSPY_LCD_TX_1;											// 0x0014
    AIT_REG_D   DSPY_LCD_TX_2;											// 0x0018
    AIT_REG_D   DSPY_LCD_TX_3;											// 0x001C

    AIT_REG_D   DSPY_LCD_TX_4;                                          // 0x0020
    AIT_REG_D   DSPY_LCD_TX_5;											// 0x0024
    AIT_REG_D   DSPY_LCD_TX_6;											// 0x0028
    AIT_REG_W   DSPY_LCD_AUTO_CFG;										// 0x002C
        /*-DEFINE-----------------------------------------------------*/
        #define AUTO_TX_TYPE_IDX            0x0000
        #define AUTO_TX_TYPE_CMD            0x0001
        #define LCD_TX_TYPE_IDX(_n)         (AUTO_TX_TYPE_IDX << _n)
        #define LCD_TX_TYPE_CMD(_n)         (AUTO_TX_TYPE_CMD << _n)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x002E[0x2];

    AIT_REG_W   DSPY_PLCD_CTL;                                          // 0x0030
        /*-DEFINE-----------------------------------------------------*/
        #define PLCD_BUS_8BPP               0x0000
        #define PLCD_BUS_16BPP              0x0001
        #define PLCD_BUS_18BPP              0x0002
        #define PLCD_BUS_12BPP              0x0003
        #define PLCD_BUS_MASK               0x0003
        #define PLCD_68SYS_RC_HIGH        	0x0004
        #define PLCD_CMD_BURST              0x0008
        #define PLCD_CMD_NONBURST			0x0000
        #define PLCD_PHA_0                  0x0000
        #define PLCD_PHA_1                  0x0010
        #define PLCD_POR_0                  0x0000
        #define PLCD_POR_1                  0x0020
        #define PLCD_TYPE_80                0x0040
        #define PLCD_TYPE_68                0x0000
        #define LCD_SPI1_PL2				0x0000
        #define LCD_PL1_SPI2                0x0100
        #define LCD_PL1_PL2                 0x0200
        #define LCD_SPI1_SPI2               0x0300
        #define PLCD_RS_LEAD_CS_EN        	0x0400
        #define A0_SIG_LOW_FOR_IDX			0x0000
        #define A0_SIG_HIGH_FOR_IDX			0x0800
        #define RGB565_TX_GB_FIRST			0x1000
        #define RGB565_TX_RG_FIRST			0x0000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PLCD_FMT;											// 0x0032
        /*-DEFINE-----------------------------------------------------*/
        #define PLCD_RGB565_BUS16           0x0000
        #define PLCD_RGB444_BUS16           0x0001
        #define PLCD_RGB666_BUS16           0x0002
        #define PLCD_RGB888_BUS8            0x0003
        #define PLCD_RGB332_BUS8            0x0004
        #define PLCD_RGB444_BUS8            0x0005
        #define PLCD_RGB666_BUS8            0x0006
        #define PLCD_RGB565_BUS8            0x0007
        #define PLCD_RGB666_BUS18           0x0008
        #define PLCD_RGB666_BUS18_9_9       0x0009
        #define PLCD_RGB666_BUS18_2_16      0x000A
        #define PLCD_RGB666_BUS18_16_2      0x000B
        #define PLCD_RGB24_BUS18_16_8       0x000C
        #define PLCD_RGB24_BUS18_8_16       0x000D
        #define PLCD_RGB666_BUS8_2_7        0x0010
        #define PLCD_RGB444_B12_EXT_B16     0x0020
        #define PLCD_RGB444_B15_14     		0x0040
        #define PLCD_RGBB9_9_17             0x0080

        #define PLCD_RGB666_B9              0x0100
        #define LCD_RGB666_B2_16            0x0200
        #define LCD_RGB666_B16_2            0x0300
        #define LCD_B8_RGB666_12_17         0x0400
        #define LCD_B16_1_8_10_17           0x0800
        #define LCD_B8_10_17                0x1000
        #define LCD_B8_1_8                  0x2000
        #define LCD_B8_RGB565_G_LSB_FIRST   0x4000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_LCD_SR;											// 0x0034
        /*-DEFINE-----------------------------------------------------*/
        #define PL_LCD_BUSY                 0x0001
        #define SPI_LCD_BUSY                0x0002
        #define SPI_RGB_LCD_BUSY			0x0004
        #define TV_PLUG_OUT_STATUS     		0x0008
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_TV_LINE;                                         	// 0x0036
    AIT_REG_W   DSPY_PLCD_RS_LEAD_CS_CYC;								// 0x0038
    AIT_REG_W   DSPY_PLCD_CS_LEAD_RW_CYC;								// 0x003A
    AIT_REG_W   DSPY_PLCD_RW_CYC;										// 0x003C
    AIT_REG_W   DSPY_PLCD_IDX_CMD_NUM;									// 0x003E
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_PLCD_IDX_CMD_MAX_NUM	0x0007
        /*------------------------------------------------------------*/

    AIT_REG_W   DSPY_W;                                                 // 0x0040
    AIT_REG_B                           _x0042[0x2];
    AIT_REG_W   DSPY_H;													// 0x0044
    AIT_REG_B                           _x0046[0x2];
    AIT_REG_D   DSPY_PIXL_CNT;											// 0x0048
    AIT_REG_W   DSPY_CTL_4;												// 0x004C
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_OUT_RGB                 0x0000
        #define LCD_OUT_BGR                 0x0002
        #define LCD_OUT_SEL_NONE            0x0000
        #define LCD_OUT_SEL_LCD1            0x0004
        #define LCD_OUT_SEL_LCD2            0x0008
        #define LCD_OUT_SEL_MASK            0x000C
        #define LCD_BG_COLR_565             0x0000
        #define LCD_BG_COLR_888             0x0020
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_WIN_PRIO;											// 0x004E
        /*-DEFINE-----------------------------------------------------*/
        #define MAIN_WIN                    0x0
        #define PIP_WIN                     0x1
        #define OVLY_WIN                    0x2
        #define ICON_WIN                    0x3
        #define WIN_1_SHFT                  0
        #define WIN_2_SHFT                  2
        #define WIN_3_SHFT                  4
        #define WIN_4_SHFT                  6
        #define WIN_PRIO(p0,p1,p2,p3)		((p3 << 6) | (p2 << 4) | (p1 << 2) | p0)
        /*------------------------------------------------------------*/
        
    AIT_REG_W   DSPY_ICON_W;                                            // 0x0050
    AIT_REG_B                           _x0052[0x2];
    AIT_REG_W   DSPY_ICON_H;											// 0x0054
    AIT_REG_B                           _x0056[0x2];
    AIT_REG_W   DSPY_ICON_X;											// 0x0058
    AIT_REG_B                           _x005A[0x2];
    AIT_REG_W   DSPY_ICON_Y;											// 0x005C
    AIT_REG_B                           _x005E[0x2];

    AIT_REG_W   DSPY_ICON_CTL;                                 			// 0x0060
    	/*-DEFINE-----------------------------------------------------*/
    	#define ICON_WIN_ENABLE				0x0001
    	/*------------------------------------------------------------*/
    AIT_REG_B                           _x0062[0x2];
    AIT_REG_D   DSPY_BG_COLOR;											// 0x0064
    AIT_REG_D   DSPY_PLCD_READ_PORT;									// 0x0068
    AIT_REG_W   DSPY_FIFO_CLR;											// 0x006C
        /*-DEFINE-----------------------------------------------------*/
        #define MAIN_FIFO_CLR               0x0001
        #define PIP_FIFO_CLR                0x0002
        #define OVLY_FIFO_CLR               0x0004
        #define DSPY_BUF_CLR                0x0008
        #define SCAL_BUF_CLR                0x0010
        #define OVLY_RGB_OUT_CLR            0x0020
        #define SCD_DSPY_BUF_CLR            0x0040
      	#define MAIN_RGB_OUT_CLR            0x0080
		#define PIP_DROP_LINE_FOR_TV        0x0000
        #define PIP_DO_NOT_DROP_LINE        0x0800
        #define FETCH_DATA_BY_PROG          0x0000
        #define FETCH_DATA_BY_INTER         0x0200
        /*------------------------------------------------------------*/
    AIT_REG_W   TV_DAC_TEST_MODE_DATA;									// 0x006E      

    AIT_REG_W   TVENC[8*9];                                             // 0x0070-0x00FF

    AIT_REG_D   DSPY_MAIN_ADDR_ST;                                    	// 0x0100
    AIT_REG_B                           _x0104[0x4];
    AIT_REG_D   DSPY_MAIN_OFST_ST;										// 0x0108
    AIT_REG_B                           _x010C[0x4];

    AIT_REG_W   DSPY_MAIN_OFST_PIXL;                                    // 0x0110
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_OFST_NEG                0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0112[0x2];
    AIT_REG_W   DSPY_MAIN_OFST_ROW;										// 0x0114
    AIT_REG_B                           _x0116[0xA];
    
    AIT_REG_D   DSPY_MAIN_U_ADDR_ST;                                  	// 0x0120
    AIT_REG_B                           _x0124[0x4];
    AIT_REG_D   DSPY_MAIN_OFST_UV_ST;									// 0x0128
    AIT_REG_B                           _x012C[0x4];

    AIT_REG_W   DSPY_MAIN_OFST_UV_PIXL;                                 // 0x0130
    AIT_REG_B                           _x0132[0x2];
    AIT_REG_W   DSPY_MAIN_OFST_UV_ROW;									// 0x0134
    AIT_REG_B                           _x0136[0xA];

    AIT_REG_D   DSPY_MAIN_V_ADDR_ST;                                  	// 0x0140
    AIT_REG_B                           _x0144[0xC];

    AIT_REG_W   DSPY_MAIN_W;                                            // 0x0150
    AIT_REG_B                           _x0152[0x2];
    AIT_REG_W   DSPY_MAIN_H;											// 0x0154
    AIT_REG_B                           _x0156[0x1];
    AIT_REG_W   DSPY_MAIN_X;											// 0x0158
    AIT_REG_B                           _x015A[0x2];
    AIT_REG_W   DSPY_MAIN_Y;											// 0x015C
    AIT_REG_B                           _x015E[0x2];

    AIT_REG_W   DSPY_MAIN_CTL;                                      	// 0x0160
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_EN                      0x0001
        #define WIN_ROTATE_EN               0x0002
        #define WIN_SRC_GRAB_EN             0x0004
        #define WIN_YUV_SCALUP_EN           0x0008	//[PIP Only]
        #define WIN_IN_RGB                  0x0000
        #define WIN_IN_BGR                  0x0040
        #define WIN_OUT_565_EXT_888       	0x0080
        
        #define WIN_YUV420_DITHER_18BIT   	0x0100
        #define WIN_YUV420_RING_BUF_EN      0x0200
        #define WIN_YUV_LPF_DIS        		0x0400
        #define WIN_YUV420_DITHER_16BIT   	0x0800
        #define WIN_YUV420_DITHER_12BIT   	0x1000
        #define WIN_YUV420_INTERLEAVE_MASK  0xC000
        #define WIN_YUV420_INTERLEAVE_UV    0x8000
        #define WIN_YUV420_INTERLEAVE_VU    0xC000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_CTL_2;                                    	// 0x0162
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_V_1X                    0x0001
        #define WIN_V_2X                    0x0002
        #define WIN_V_4X                    0x0004
        #define WIN_H_1X                    0x0008
        #define WIN_H_2X                    0x0010
        #define WIN_H_4X                    0x0020
        #define WIN_V_DUP_MASK				0x0007
        #define WIN_H_DUP_MASK				0x0038
        #define WIN_DUP_MASK				0x003F
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_FMT;											// 0x0164
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_4BPP                    0x0001  // MAIN,      OVLY
        #define WIN_8BPP                    0x0002	// MAIN,      OVLY
        #define WIN_16BPP                   0x0004	// MAIN, PIP, OVLY
        #define WIN_24BPP                   0x0008	//       PIP,
        #define WIN_YUV420                  0x0010	//       PIP, OVLY
        #define WIN_YUV422                  0x0020	//       PIP, OVLY
        #define WIN_32BPP                   0x0040	//       PIP, OVLY
        
        #define WIN_16BIT_ARGB3454     		0x0200	//       PIP, OVLY
        #define WIN_16BIT_ARGB4444			0x0300
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0166[0x2];
    AIT_REG_D   DSPY_MAIN_PIXL_CNT;										// 0x0168	
    AIT_REG_B                           _x016C[0x4];

    AIT_REG_W   DSPY_MAIN_TP_CTL;                            			// 0x0170
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_TP_EN                   0x0001
        #define WIN_ALPHA_BLEND_EN          0x0002
        #define WIN_ALPHA_BLEND_DIS         0x0000
        #define WIN_ALPHA_BLEND_GLOBAL     	0x0000
        #define WIN_ALPHA_BLEND_LOCAL      	0x0004
        #define WIN_ALPHA_BLEND_OP_AND      0x0008
        #define WIN_ALPHA_BLEND_OP_OR       0x000C
        #define WIN_ALPHA_BLEND_OP_INV_UP   0x0010
        #define WIN_ALPHA_BLEND_OP_INV_DN   0x0014
        #define WIN_ALPHA_BLEND_MASK    	0x001C
        #define WIN_ALPHA_LSB               0x0020
        #define WIN_ALPHA_MSB               0x0000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_GLOBAL_ALPHA_WT;								// 0x0172
    AIT_REG_D   DSPY_MAIN_TP_COLOR;										// 0x0174
    AIT_REG_B                           _x0178[0x8];

	AIT_REG_B                           _x0180[0x10];
	
    AIT_REG_B   DSPY_MAIN_UV_GAIN_11;                                   // 0x0190
    AIT_REG_B   DSPY_MAIN_UV_GAIN_12;									// 0x0191
    AIT_REG_B   DSPY_MAIN_UV_GAIN_21;									// 0x0192
    AIT_REG_B   DSPY_MAIN_UV_GAIN_22;									// 0x0193
    AIT_REG_W   DSPY_MAIN_RGB_GAIN;										// 0x0194
    AIT_REG_W   DSPY_MAIN_RGB_OFST;										// 0x0196
    AIT_REG_B                           _x0198[0x8]; 

    AIT_REG_W   DSPY_SCD_CTL;						                    // 0x01A0
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_CTL_0
        #define	SCD_SRC_RGB888				0x0100
        #define	SCD_SRC_RGB565				0x0200
        #define	SCD_SRC_FMT_MASK			0x0300
        #define	SCD_565_2_888_STUFF_0		0x0000
        #define	SCD_565_2_888_STUFF_MSB		0x0400
        #define SW_UPDATE_SCD_OPR_NOW		0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x01A2[0x2];
    AIT_REG_B   DSPY_SCD_FLM_CTL;                                       // 0x01A4
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_LCD_FLM_CTL
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_SCD_VSYNC_CTL;                                     // 0x01A5	
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_LCD_VSYNC_CTL
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_SCD_FLM_VSYNC_CNT;                                 // 0x01A6
    AIT_REG_D   DSPY_SCD_BG_COLOR;                                 		// 0x01A8
    AIT_REG_D   DSPY_SCD_PLCD_READ_PORT;								// 0x01AC

    AIT_REG_D   DSPY_SCD_LCD_TX_0;                                      // 0x01B0
    AIT_REG_D   DSPY_SCD_LCD_TX_1;										// 0x01B4
    AIT_REG_D   DSPY_SCD_LCD_TX_2;										// 0x01B8
    AIT_REG_W   DSPY_SCD_LCD_AUTO_CFG;									// 0x01BC
    AIT_REG_B                           _x01BE[0x2];
    
    AIT_REG_W   DSPY_SCD_W;                                             // 0x01C0
    AIT_REG_B                           _x01C2[0x2];
    AIT_REG_W   DSPY_SCD_H;												// 0x01C4
    AIT_REG_B                           _x01C6[0x2];
    AIT_REG_D   DSPY_SCD_PIXL_CNT;										// 0x01C8
    AIT_REG_D   DSPY_SCD_WIN_PIXL_CNT;									// 0x01CC
    
    AIT_REG_D   DSPY_SCD_WIN_ADDR_ST;                                 	// 0x01D0
    AIT_REG_D   DSPY_SCD_WIN_OFST_ST;									// 0x01D4
    AIT_REG_W   DSPY_SCD_WIN_W;                                         // 0x01D8
    AIT_REG_W   DSPY_SCD_WIN_H;                                         // 0x01DA
    AIT_REG_W   DSPY_SCD_WIN_X;                                         // 0x01DC
    AIT_REG_W   DSPY_SCD_WIN_Y;                                         // 0x01DE
    
    AIT_REG_D   DSPY_MAIN_TV_EVEN_FIELD_ST;                         	// 0x01E0
    AIT_REG_B                           _x01E4[0x4];
	AIT_REG_D	DSPY_WIN_ALPHA_WT_1;
	AIT_REG_D	DSPY_WIN_ALPHA_WT_2;
        /*-DEFINE-----------------------------------------------------*/
		#define ALPHA_W_1(l, w)             ((w | ((8-w) << 4)) << (l * 8))    
		#define ALPHA_W_2(l, w)             ((w | ((8-w) << 4)) << ((l - 4)* 8))    
        /*------------------------------------------------------------*/

    AIT_REG_B                           _x01F0[0x10];

    AIT_REG_D   DSPY_PIP_ADDR_ST;                                    	// 0x0200
    AIT_REG_B                 			_x0204[0x4];
    AIT_REG_D   DSPY_PIP_OFST_ST;										// 0x0208
    AIT_REG_B                 			_x020C[0x4];

    AIT_REG_W   DSPY_PIP_OFST_PIXL;                                     // 0x0210
    AIT_REG_B                           _x0212[0x2];
    AIT_REG_W   DSPY_PIP_OFST_ROW;										// 0x0214
    AIT_REG_B                           _x0216[0xA];

    AIT_REG_D   DSPY_PIP_U_ADDR_ST;                                   	// 0x0220
    AIT_REG_B                 			_x0224[0x4];
    AIT_REG_D   DSPY_PIP_OFST_UV_ST;									// 0x0228
    AIT_REG_B                 			_x022C[0x4];
    
    AIT_REG_W   DSPY_PIP_OFST_UV_PIXL;                                  // 0x0230
    AIT_REG_B                           _x0232[0x2];
    AIT_REG_W   DSPY_PIP_OFST_UV_ROW;									// 0x0234
    AIT_REG_B                           _x0236[0xA];
 
    AIT_REG_D   DSPY_PIP_V_ADDR_ST;                          			// 0x0240
    AIT_REG_B                           _x0244[0xC];

    AIT_REG_W   DSPY_PIP_W;                                             // 0x0250
    AIT_REG_B                           _x0252[0x2];
    AIT_REG_W   DSPY_PIP_H;												// 0x0254
    AIT_REG_B                           _x0256[0x2];
    AIT_REG_W   DSPY_PIP_X;												// 0x0258
    AIT_REG_B                           _x025A[0x2];
    AIT_REG_W   DSPY_PIP_Y;												// 0x025C
    AIT_REG_B                           _x025E[0x2];

    AIT_REG_W   DSPY_PIP_CTL;                                           // 0x0260
    AIT_REG_W   DSPY_PIP_CTL_2;                                         // 0x0262
    AIT_REG_W   DSPY_PIP_FMT;											// 0x0264
    AIT_REG_B                           _x0266[0x2];
    AIT_REG_D   DSPY_PIP_PIXL_CNT;										// 0x0268
    AIT_REG_D   DSPY_SOUT_GRAB_PIXL_CNT;								// 0x026C

    AIT_REG_W   DSPY_PIP_TP_CTL;                                        // 0x0270
    AIT_REG_W   DSPY_PIP_GLOBAL_ALPHA_WT;								// 0x0272
    AIT_REG_D   DSPY_PIP_TP_COLOR;										// 0x0274
    AIT_REG_B                           _x0278[0x8];

    AIT_REG_W   DSPY_PIP_SCAL_CTL;                                		// 0x0280
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_NM                0x0000
        #define DSPY_SCAL_BYPASS            0x0001      // bypass N/M and LPF
        #define DSPY_SCAL_WT_DYN         	0x0000
        #define DSPY_SCAL_WT_FIX         	0x0002
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SOUT_CTL;										// 0x0282
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SOUT_YUV_EN            0x0001
        #define DSPY_SOUT_RGB_EN            0x0002
        #define DSPY_SOUT_YUV_444           0x0000
        #define DSPY_SOUT_YUV_422           0x0008
        #define DSPY_SOUT_RGB_888           0x0000
        #define DSPY_SOUT_RGB_565           0x0010
        #define DSPY_SOUT_RGB_666           0x0020
        #define DSPY_SOUT_DITHER_EN   	    0x0040
        #define DSPY_SOUT_RGB            	0x0000
        #define DSPY_SOUT_BGR           	0x0080
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SCAL_H_N;										// 0x0284
    AIT_REG_W   DSPY_PIP_SCAL_H_M;										// 0x0286
    AIT_REG_W   DSPY_PIP_SCAL_V_N;										// 0x0288
    AIT_REG_W   DSPY_PIP_SCAL_V_M;										// 0x028A
    AIT_REG_W   DSPY_PIP_SEDGE_CTL;										// 0x028C
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SEDGE_BYPASS            0x0001
        #define DSPY_SEDGE_YUV_AVG           0x0002
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_PIP_SEDGE_GAIN_VAL;								// 0x028E
    AIT_REG_B   DSPY_PIP_SEDGE_CORE_VAL;								// 0x028F

    AIT_REG_B   DSPY_PIP_UV_GAIN_11;                          			// 0x0290
    AIT_REG_B   DSPY_PIP_UV_GAIN_12;									// 0x0291
    AIT_REG_B   DSPY_PIP_UV_GAIN_21;									// 0x0292
    AIT_REG_B   DSPY_PIP_UV_GAIN_22;									// 0x0293
    AIT_REG_W   DSPY_PIP_RGB_GAIN;										// 0x0294
    AIT_REG_W   DSPY_PIP_RGB_OFST;										// 0x0296
    AIT_REG_W   DSPY_PIP_SCAL_H_WT;										// 0x0298
    AIT_REG_W   DSPY_PIP_SCAL_V_WT;										// 0x029A
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_WT_AVG 	    	0x0001
        #define DSPY_SCAL_WT_MASK       	0xFFFE
        #define DSPY_SCAL_WT(_w)			((_w << 1) & DSPY_SCAL_WT_MASK)
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SCAL_WT_FORCE1_EN;								// 0x029C
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_WT_FORCE1_EN	    0x0001
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x029E[0x2];

    AIT_REG_W   DSPY_PIP_SCA_IN_W;                                      // 0x02A0
    AIT_REG_B                           _x02A2[0x2];
    AIT_REG_W   DSPY_PIP_SCA_IN_H;										// 0x02A4
    AIT_REG_B                           _x02A6[0x2];
    AIT_REG_W   DSPY_PIP_SCA_OUT_W;                                		// 0x02A8
    AIT_REG_B                           _x02AA[0x2];
    AIT_REG_W   DSPY_PIP_SCA_OUT_H;										// 0x02AC
    AIT_REG_B                           _x02AE[0x2];

    AIT_REG_W   DSPY_PIP_SOUT_GRAB_H_ST;                                // 0x02B0
    AIT_REG_B                           _x02B2[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_H_ED;								// 0x02B4
    AIT_REG_B                           _x02B6[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_V_ST;								// 0x02B8
    AIT_REG_B                           _x02BA[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_V_ED;								// 0x02BC
    AIT_REG_B                           _x02BE[0x2];

    AIT_REG_D   DSPY_Y_ADDR_LOW_BOUND;									// 0x02C0
    AIT_REG_D   DSPY_Y_ADDR_HIGH_BOUND;                             	// 0x02C4
    AIT_REG_D   DSPY_U_ADDR_LOW_BOUND;									// 0x02C8
    AIT_REG_D   DSPY_U_ADDR_HIGH_BOUND;                             	// 0x02CC

	AIT_REG_W   DSPY_MPEG4_DEC_ROW_CNT;									// 0x02D0
	AIT_REG_B                           _x02D2[0x6];
    AIT_REG_D   DSPY_V_ADDR_LOW_BOUND;									// 0x02D8
    AIT_REG_D   DSPY_V_ADDR_HIGH_BOUND;                             	// 0x02DC

    AIT_REG_D   DSPY_PIP_TV_EVEN_FIELD_ST;                        		// 0x02E0
    AIT_REG_B                           _x02E4[0xC];
    
	AIT_REG_B                           _x02F0[0x2];
    AIT_REG_W   DSPY_PIP_BUF_FULL_THD;                                 	// 0x02F2                    
    AIT_REG_W   DSPY_PIP_BUF_STOP_THD;                                 	// 0x02F4        
    AIT_REG_W   DSPY_FIFO_THD;                                          // 0x02F6
    AIT_REG_B                           _x02F8;
    AIT_REG_B   DSPY_RGB_SYNC_MODE;                                  	// 0x02F9
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_RGB_SYNC_MODE_EN 	    0x01
        #define DSPY_RGB_SYNC_MODE_DIS      0x00
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x02FA[0x6];

    AIT_REG_D   DSPY_OVLY_ADDR_ST;                                    	// 0x0300
    AIT_REG_B                           _x0304[0x4]; 
    AIT_REG_D   DSPY_OVLY_OFST_ST;										// 0x0308
    AIT_REG_B                           _x030C[0x4]; 

    AIT_REG_W   DSPY_OVLY_OFST_PIXL;                                    // 0x0310
    AIT_REG_B                           _x0312[0x2];
    AIT_REG_W   DSPY_OVLY_OFST_ROW;										// 0x0314
    AIT_REG_B                           _x0316[0xA];

    AIT_REG_D   DSPY_OVLY_U_ADDR_ST;                                  	// 0x0320
    AIT_REG_B                           _x0324[0x4]; 
    AIT_REG_D   DSPY_OVLY_OFST_UV_ST;									// 0x0328
    AIT_REG_D   						_x032C[0x4];

    AIT_REG_W   DSPY_OVLY_OFST_UV_PIXL;                                 // 0x0330
    AIT_REG_B                           _x0332[0x2];
    AIT_REG_W   DSPY_OVLY_OFST_UV_ROW;									// 0x0334
    AIT_REG_B                           _x0336[0xA];

    AIT_REG_D   DSPY_OVLY_V_ADDR_ST;                                  	// 0x0340                                  
    AIT_REG_B                           _x0344[0xC];                                   

    AIT_REG_W   DSPY_OVLY_W;                                            // 0x0350
    AIT_REG_B                           _x0352[0x2];
    AIT_REG_W   DSPY_OVLY_H;											// 0x0354
    AIT_REG_B                           _x0356[0x2];
    AIT_REG_W   DSPY_OVLY_X;											// 0x0358
    AIT_REG_B                           _x035A[0x2];
    AIT_REG_W   DSPY_OVLY_Y;											// 0x035C
    AIT_REG_B                           _x035E[0x2];

    AIT_REG_W   DSPY_OVLY_CTL;                                          // 0x0360
    AIT_REG_W   DSPY_OVLY_CTL_2;                                        // 0x0362
    AIT_REG_W   DSPY_OVLY_FMT;											// 0x0364
    AIT_REG_B                           _x0366[0x2];
    AIT_REG_D   DSPY_OVLY_PIXL_CNT;										// 0x0368
    AIT_REG_B                           _x036C[0x4];

    AIT_REG_W   DSPY_OVLY_TP_CTL;                                       // 0x0370
    AIT_REG_W   DSPY_OVLY_SEMITP_WT;									// 0x0372
    AIT_REG_D   DSPY_OVLY_TP_COLR;										// 0x0374
    AIT_REG_B                           _x0378[0x8]; 
    
    AIT_REG_W   DSPY_RGB_LINE_CPU_INT_1;                               	// 0x0380
    AIT_REG_W   DSPY_RGB_LINE_CPU_INT_2;                                // 0x0382
    AIT_REG_W   DSPY_RGB_LINE_HOST_INT_1;                               // 0x0384
    AIT_REG_W   DSPY_RGB_LINE_HOST_INT_2;                               // 0x0386
    AIT_REG_B   DSPY_INT_HOST_EN_2;                                     // 0x0388
    AIT_REG_B   DSPY_INT_CPU_EN_2;                                      // 0x0389
    AIT_REG_B   DSPY_INT_HOST_SR_2;                                     // 0x038A
    AIT_REG_B   DSPY_INT_CPU_SR_2;                                      // 0x038B
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_FLM_INT					0x10
        #define LCD_WRITE_BACK_DONE         0x08
        #define RGB_LINE_INT2               0x04
        #define RGB_LINE_INT1               0x02
        #define HSYNC_ACTIVE	            0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x038C[0x4];
    
    AIT_REG_B   DSPY_OVLY_UV_GAIN_11;                                   // 0x0390
    AIT_REG_B   DSPY_OVLY_UV_GAIN_12;                                   // 0x0391
    AIT_REG_B   DSPY_OVLY_UV_GAIN_21;                                   // 0x0392
    AIT_REG_B   DSPY_OVLY_UV_GAIN_22;                                   // 0x0393
    AIT_REG_W   DSPY_OVLY_RGB_GAIN;                                     // 0x0394
    AIT_REG_W   DSPY_OVLY_RGB_OFST;                                     // 0x0396
    AIT_REG_B                           _x0398[0x8];		

	AIT_REG_B	DSPY_RGB_CTL;											// 0x03A0
        /*-DEFINE-----------------------------------------------------*/
        #define DDRCLK_POLAR_NORMAL         0x00
        #define DDRCLK_POLAR_INVERT         0x80
        #define DOTCLK_NORMAL_MODE          0x00
        #define DOTCLK_DDR_MODE             0x40
        #define HSYNC_POLAR_LOW	       	    0x00
        #define HSYNC_POLAR_HIGH            0x20
        #define VSYNC_POLAR_LOW             0x00
        #define VSYNC_POLAR_HIGH            0x10
        #define DOT_POLAR_PST               0x00
        #define DOT_POLAR_NEG               0x08
		#define	DEN_DATA_MODE			    0x00
		#define	PRT_DATA_MODE			    0x04
        #define PARTIAL_MODE_EN			    0x02
        #define RGB_IF_EN		            0x01
        /*------------------------------------------------------------*/
	AIT_REG_B	DSPY_RGB_SPI_CTL;										// 0x03A1	
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_START_ID                0x01
        /*------------------------------------------------------------*/
	AIT_REG_B	DSPY_RGB_FMT;											// 0x03A2	
        /*-DEFINE-----------------------------------------------------*/
        #define YUV422_D8BIT_Y1VY0U         0x27
        #define YUV422_D8BIT_Y1UY0V         0x26
        #define YUV422_D8BIT_Y0VY1U         0x25
        #define YUV422_D8BIT_Y0UY1V         0x24
        #define YUV422_D8BIT_VY1UY0         0x23
        #define YUV422_D8BIT_UY1VY0         0x22
        #define YUV422_D8BIT_VY0UY1         0x21
        #define YUV422_D8BIT_UY0VY1         0x20
        
        #define RGB_D24BIT_BGR332			0x0D
        #define RGB_D24BIT_BGR333			0x0C
        #define RGB_D24BIT_BGR444			0x0B
        #define RGB_D24BIT_BGR565			0x0A
        #define RGB_D24BIT_BGR666			0x09
        #define RGB_D24BIT_BGR888			0x08
        #define RGB_D24BIT_RGB332			0x05
        #define RGB_D24BIT_RGB333			0x04
        #define RGB_D24BIT_RGB444			0x03
        #define RGB_D24BIT_RGB565			0x02
        #define RGB_D24BIT_RGB666			0x01
        #define RGB_D24BIT_RGB888			0x00
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_RGB_SHARE_P_LCD_BUS;                       		// 0x03A3
        /*-DEFINE-----------------------------------------------------*/
        #define P_LCD_ONLY                  0x00
        #define RGB_18BIT_SHARE_WITH_P_LCD  0x01
        #define RGB_24BIT_SHARE_WITH_P_LCD  0x02
        #define RGB_LCD_ONLY                0x03
       	#define DISP_SHARE_BUS_MASK			0x03
       	#define HDMI_VSYNC_DELAY_MASK		0x07
        #define HDMI_VSYNC_DELAY(_t)		((_t & HDMI_VSYNC_DELAY_MASK) << 4)
        /*------------------------------------------------------------*/
    AIT_REG_B	DSPY_RGB_DOT_CLK_RATIO;									// 0x03A4	
    AIT_REG_B   DSPY_RGB_PORCH_HIGH_BIT_EXT;                       		// 0x03A5
    AIT_REG_B	DSPY_RGB_V_BPORCH;										// 0x03A6	
    AIT_REG_B	DSPY_RGB_V_BLANK;										// 0x03A7	
    AIT_REG_B	DSPY_RGB_H_BPORCH;										// 0x03A8	
    AIT_REG_B	DSPY_RGB_H_BLANK;										// 0x03A9	
    AIT_REG_B	DSPY_RGB_HSYNC_W;										// 0x03AA	
    AIT_REG_B	DSPY_RGB_VSYNC_W;										// 0x03AB	
    AIT_REG_B	DSPY_RGB_V_2_H_DOT;										// 0x03AC	
    AIT_REG_B                           _x03AD;
    AIT_REG_B	DSPY_RGB_PRT_2_H_DOT;									// 0x03AE	
    AIT_REG_B                           _x03AF;
    
    AIT_REG_W	DSPY_RGB_PART_ST_Y;  									// 0x03B0	
    AIT_REG_W	DSPY_RGB_PART_ED_Y;	    							    // 0x03B2	
    AIT_REG_W	DSPY_RGB_PART_ST_X;  									// 0x03B4	
    AIT_REG_W	DSPY_RGB_PART_ED_X;									    // 0x03B6
    AIT_REG_B	DSPY_RGB_RATIO_SPI_MCI;                                 // 0x03B8
    AIT_REG_B	                        _x03B9;
    AIT_REG_B	DSPY_RGB_SPI_CS_SETUP_CYCLE;                            // 0x03BA
    AIT_REG_B	DSPY_RGB_SPI_CS_HOLD_CYCLE;                             // 0x03BB
    AIT_REG_B	DSPY_RGB_SPI_CS_HIGH_WIDTH;                             // 0x03BC
    AIT_REG_B	                        _x03BD;
    AIT_REG_B	DSPY_RGB_SPI_CTRL_REGISTER1;                         	// 0x03BE
        /*-DEFINE-----------------------------------------------------*/
        #define RGB_SPI_DATA_ONLY_MODE      0x01
        /*------------------------------------------------------------*/        
    AIT_REG_B	DSPY_RGB_DELTA_MODE;                                    // 0x03BF
        /*-DEFINE-----------------------------------------------------*/
        #define RGB_DELTA_MODE_ENABLE       0x01
        #define RGB_DUMMY_MODE_ENABLE       0x02
        
        #define SPI_ODD_LINE_RGB            0x00
        #define SPI_ODD_LINE_RBG            0x04
        #define SPI_ODD_LINE_GRB            0x08
        #define SPI_ODD_LINE_GBR            0x0C
        #define SPI_ODD_LINE_BRG            0x10
        #define SPI_ODD_LINE_BGR            0x12
        
        #define SPI_EVEN_LINE_RGB           0x00
        #define SPI_EVEN_LINE_RBG           0x20
        #define SPI_EVEN_LINE_GRB           0x40
        #define SPI_EVEN_LINE_GBR           0x60
        #define SPI_EVEN_LINE_BRG           0x80
        #define SPI_EVEN_LINE_BGR           0xA0
        /*------------------------------------------------------------*/   
         
    AIT_REG_B	DSPY_SPI_CONTROL_REGISTER1;                             // 0x03C0
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_POLARITY_POSITIVE_EDGE  0x00
        #define SPI_POLARITY_NEGATIVE_EDGE  0x02
        #define SPI_RW_HIGH_READ           	0x00
        #define SPI_RW_HIGH_WRITE         	0x04
        #define SPI_RS_HIGH_COMMAND         0x00
        #define SPI_RS_HIGH_INDEX           0x08
        #define SPI_CS_ACTIVE_LOW           0x00
        #define SPI_CS_ACTIVE_HIGH          0x10
        #define SPI_PANEL_8BITS             0x00
        #define SPI_PANEL_9BITS             0x20
        #define SPI_PANEL_12BITS            0x40
        #define SPI_PANEL_16BITS            0x60
        #define SPI_PANEL_18BITS            0x80
        #define SPI_PANEL_24BITS            0xA0
        /*------------------------------------------------------------*/        
    AIT_REG_B	                        _x03C1;
    AIT_REG_B	DSPY_RATIO_MCI_SPI;									    // 0x03C2
    AIT_REG_B	                        _x03C3;                         // 0x03C3
    AIT_REG_B	DSPY_SPI_CS_SETUP_CYCLE;                                // 0x03C4
    AIT_REG_B	DSPY_SPI_CS_HOLD_CYCLE;                                 // 0x03C5
    AIT_REG_B	DSPY_SPI_CS_HIGH_WIDTH;                                 // 0x03C6
    AIT_REG_B	                        _x03C7;
    AIT_REG_B   DSPY_RGB_FRAM_CNT_CPU_INT;                              // 0x03C8
    AIT_REG_B                           _x03C9;
    AIT_REG_B   DSPY_RGB_FRAM_CNT_HOST_INT;                             // 0x03CA
    AIT_REG_B                           _x03CB[0x5];
    
    AIT_REG_B   DSPY_WBACK_CTL;                             			// 0x03D0
        /*-DEFINE-----------------------------------------------------*/
        #define WBACK_ONLY_MODE				0x10
        #define WBACK_FMT_BGR				0x08
        #define WBACK_FMT_RGB				0x00
        #define WBACK_FMT_888				0x04
        #define WBACK_FMT_565				0x02
        #define WBACK_EN					0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x03D1[0x3];
    AIT_REG_D   DSPY_WBACK_ADDR;                             			// 0x03D4
    AIT_REG_W   DSPY_WBACK_W;                             				// 0x03D8
    AIT_REG_W   DSPY_WBACK_H;                             				// 0x03DA
    AIT_REG_W   DSPY_WBACK_X;                             				// 0x03DC
    AIT_REG_W   DSPY_WBACK_Y;                             				// 0x03DE
    
    AIT_REG_D   DSPY_OVLY_TV_EVFIELD_ST;                                // 0x03E0
    AIT_REG_B                           _x03E4[0xC];
    
    AIT_REG_B                           _x03F0[0x10];

} AITS_DSPY, *AITPS_DSPY;

//-----------------------------
// TV structure (0x8000 7070)
//-----------------------------
typedef struct _AITS_TV {
    AIT_REG_D TVIF_TEST_1ST_Y2CrY1Cb;                                   // 0x0070
    AIT_REG_D TVIF_TEST_2ND_Y2CrY1Cb;                                   // 0x0074
    AIT_REG_B TVIF_DAC_IF_1ST_CTL;                                      // 0x0078
        /*-DEFINE-----------------------------------*/
        #define TV_DAC_POWER_DOWN_EN        0x08	
        #define TV_BGREF_POWER_DOWN_EN      0x04	
        #define TV_IQUARTER                 0x02	
        #define TV_OTYPE                    0x01	  
        /*------------------------------------------*/
    AIT_REG_B TVIF_DAC_IF_2ND_CTL;                                      // 0x0079
        /*-DEFINE-----------------------------------*/
        #define TV_VPLUGREF                 0x10	
        #define TV_COMP_LEVEL               0x08	
        #define TV_HYS_ON                   0x04	
        #define TV_TEST_COMP                0x02	
        #define TV_PLUG_DECT                0x01	        
        /*------------------------------------------*/
    AIT_REG_B TVIF_DAC_IF_3RD_CTL;                                      // 0x007A
        /*-DEFINE-----------------------------------*/
        #define TV_DAC_CLOCK_DATA_EXT       0x04
        /*------------------------------------------*/
    AIT_REG_B                               _x007B;
    AIT_REG_B TVIF_BACKGROUND_Y_COLOR;                                  // 0x007C
    AIT_REG_B TVIF_BACKGROUND_U_COLOR;                                  // 0x007D
    AIT_REG_B TVIF_BACKGROUND_V_COLOR;                                  // 0x007E    
    AIT_REG_B TVIF_CLK_DELAY_V1;                                        // 0x007F
        /*-DEFINE-----------------------------------*/
        #define NO_DELAY                    0x00
        #define DELAY_1T                    0x01
        #define DELAY_2T                    0x02
        #define DELAY_3T                    0x03
        /*------------------------------------------*/
    AIT_REG_B TVIF_IF_EN;                                               // 0x0080
        /*-DEFINE-----------------------------------*/
        #define TV_ENC_TEST_MODE_EN         0x80
        #define TV_IF_DAC_CTL               0x40
        #define TV_TYPE_NTSC                0x00
        #define TV_TYPE_PAL                 0x20
        #define TV_UV_SEL_HALF_SUM          0x00
        #define TV_UV_SEL_U1V1              0x08
        #define TV_UV_SEL_U2V2              0x10
        #define TV_8MHZ_FPGA_TEST           0x04
        #define TV_DISPLAY_SPECIFIED_IMAGE  0x02
        #define TV_ENC_IF_EN                0x01
        /*------------------------------------------*/
    AIT_REG_B TVIF_ENDLINE_OFFSET_CTL;                                  // 0x0081
    AIT_REG_B TVIF_EARLY_PIXL;		                                    // 0x0082
    AIT_REG_B TVIF_1ST_PXL_RQST_TIMING;                                 // 0x0083
    AIT_REG_W TVIF_NTSC_ODFIELD_LINE;     	                            // 0x0084
    AIT_REG_W TVIF_NTSC_EVFIELD_LINE;                                   // 0x0086
    AIT_REG_W TVIF_PAL_1ST_FIELD_LINE;                                  // 0x0088
    AIT_REG_W TVIF_PAL_2ND_FIELD_LINE;                                  // 0x008A
    AIT_REG_W TVIF_NTSC_EVLINE_SUB1;                                    // 0x008C
    AIT_REG_W TVIF_PAL_EVLINE_SUB1;		                                // 0x008E
    AIT_REG_W TVIF_INT1_CPU;                                            // 0x0090
    AIT_REG_W TVIF_INT2_CPU;                                            // 0x0092
    AIT_REG_W TVIF_INT1_HOST;                                           // 0x0094
    AIT_REG_W TVIF_INT2_HOST;                                           // 0x0096
    AIT_REG_W TVIF_IMAGE_WIDTH;                                         // 0x0098
    AIT_REG_W TVIF_IMAGE_HEIGHT;                                        // 0x009A
    AIT_REG_W TVIF_IMAGE_START_X;                                       // 0x009C
    AIT_REG_W TVIF_IMAGE_START_Y;                                       // 0x009E
    AIT_REG_D TVENC_SYNC_CTL;                                           // 0x00A0
        /*-DEFINE-----------------------------------*/
        #define TV_ENC_SYNC_SW_RST          0xC0000000
        #define TV_UV_SWAPPING_EN           0x00001000
        /*------------------------------------------*/
    AIT_REG_D TVENC_MODE_CTL;                                           // 0x00A4
        /*-DEFINE-----------------------------------*/
        #define TV_714MV_286MV_MODE         0x80000000
        #define TV_BLACKER_LEVEL_EN         0x40000000
        #define TV_COLOR_BAR_TYPE           0x10000000
        #define TV_FULL_WIDTH_OUTPUT_EN     0x08000000
        #define TV_SLEW_RATE_CTL_DIS        0x02000000
        #define TV_MIX_SUB_VIDEO_EN         0x01000000
        #define TV_SVIDEO_CVBS_EN           0x00040000
        #define TV_OUTPUT_CVBS_MODE         0x00020000
        #define TV_OUTPUT_SVIDEO_MODE       0x00010000
        #define TV_CHROMA_UPSAMPLE_EN       0x00004000
        #define TV_DELAY_INPUT_Y_HALF_PIX_1 0x00000000
        #define TV_DELAY_INPUT_Y_HALF_PIX_2 0x00000400
        #define TV_DELAY_INPUT_Y_HALF_PIX_3 0x00000800
        #define TV_DELAY_INPUT_Y_HALF_PIX_4 0x00000C00
        #define TV_DELAY_INPUT_Y_ONE_PIX_1  0x00000000
        #define TV_DELAY_INPUT_Y_ONE_PIX_2  0x00000100
        #define TV_DELAY_INPUT_Y_ONE_PIX_3  0x00000200
        #define TV_DELAY_INPUT_Y_ONE_PIX_4  0x00000300
        #define TV_LUMA_LPF_EN              0x00000080
        #define TV_UV_SWAPPING_SUB_VIDEO    0x00000040
        #define TV_CHROMA_LPF_EN            0x00000020
        #define TV_SETUP_751RE_EN           0x00000004
        #define TV_COLOR_BAR_EN             0x00000002
        #define TV_ENCODER_EN               0x00000001
        /*------------------------------------------*/
    AIT_REG_D TVENC_CLOSED_CAPTION;                                     // 0x00A8
        /*-DEFINE-----------------------------------*/
        #define TV_CLOSED_CAP_LINE_21_22    0x00010000
        #define TV_CLOSED_CAP_LINE_284_335  0x00020000
        /*------------------------------------------*/
    AIT_REG_D TVENC_Y_SCALE_CTL;                                        // 0x00AC
        /*-DEFINE-----------------------------------*/
        #define TV_SUB_VIDEO_DELAY_SEL_1T   0xC0000000
        #define TV_SUB_VIDEO_DELAY_SEL_2T   0x80000000
        #define TV_SUB_VIDEO_DELAY_SEL_3T   0x40000000
        #define TV_SUB_VIDEO_DELAY_SEL_4T   0x00000000
        #define TV_SUB_PIC_DELAY_SEL_1T     0x30000000
        #define TV_SUB_PIC_DELAY_SEL_2T     0x20000000
        #define TV_SUB_PIC_DELAY_SEL_3T     0x10000000
        #define TV_SUB_PIC_DELAY_SEL_4T     0x00000000
        #define TV_OSD_DELAY_SEL_1T         0x0C000000
        #define TV_OSD_DELAY_SEL_2T         0x08000000
        #define TV_OSD_DELAY_SEL_3T         0x04000000
        #define TV_OSD_DELAY_SEL_4T         0x00000000
        /*------------------------------------------*/
    AIT_REG_D TVENC_U_SCALE_CTL;                                        // 0x00B0
    AIT_REG_D TVENC_V_SCALE_CTL;                                        // 0x00B4
    AIT_REG_D TVENC_GAMMA_COEF_0;                                       // 0x00B8
        /*-DEFINE-----------------------------------*/
        #define TV_ACTIVE_VBI_EN            0x00400000
        /*------------------------------------------*/
    AIT_REG_D TVENC_GAMMA_COEF_1_2;                                     // 0x00BC
    AIT_REG_D TVENC_GAMMA_COEF_3_4;                                     // 0x00C0
    AIT_REG_D TVENC_GAMMA_COEF_5_6;                                     // 0x00C4
    AIT_REG_D TVENC_GAMMA_COEF_7_8;                                     // 0x00C8
    AIT_REG_D TVENC_DAC_CONFIG;                                         // 0x00CC
        /*-DEFINE-----------------------------------*/
        #define TV_VREF_OUTPUT_DIS          0x00008000
        #define TV_CLOCK_DAC_NEGATIVE_EDGE  0x00004000
        #define TV_TRIM_MODE_EN             0x00002000
        #define TV_PLUG_DETECT_EN           0x00001000
        #define TV_DAS_Y_OUTPUT_OFF         0x00000000
        #define TV_DAS_Y_AUTO_DETECT        0x00000040
        #define TV_DAS_Y_OUTPUT_ON          0x00000080
        #define TV_DAX_C_OUTPUT_OFF         0x00000000
        #define TV_DAX_C_AUTO_DETECT        0x00000010
        #define TV_DAX_C_OUTPUT_ON          0x00000020
        /*------------------------------------------*/
    AIT_REG_D TVENC_COLOR_BURST_CONFIG;                                 // 0x00D0
        /*-DEFINE-----------------------------------*/
        #define TV_PAL_MODE_BURST_SEL_STR   0x04000000
        #define TV_UV_EXTRA_GAIN_EN         0x02000000
        #define TV_PAL_MODE_BURST_SEL_END   0x01000000
        #define TV_FORCE_PAL60_NTSC443      0x00008000
        /*------------------------------------------*/
    AIT_REG_D                               _x00D4;
    AIT_REG_D TVENC_WSS_IF_MODE;                                        // 0x00D8
        /*-DEFINE-----------------------------------*/
        #define TV_WSS_IF_MODE_EN           0x00100000
        /*------------------------------------------*/
    AIT_REG_D TVENC_UV_SCALE_GAIN_4_5;                                  // 0x00DC
    AIT_REG_D TVENC_Y_LPF_COEF_00_03;                                   // 0x00E0
    AIT_REG_D TVENC_Y_LPF_COEF_04_07;                                   // 0x00E4
    AIT_REG_D TVENC_Y_LPF_COEF_08_0B;                                   // 0x00E8
    AIT_REG_D TVENC_Y_LPF_COEF_0C_0F;                                   // 0x00EC
    AIT_REG_D TVENC_Y_LPF_COEF_10_13;                                   // 0x00F0
    AIT_REG_D TVENC_C1_LPF_COEF_00_03;                                  // 0x00F4
    AIT_REG_D TVENC_C1_LPF_COEF_04;                                     // 0x00F8
    AIT_REG_D                               _x00FC;
} AITS_TV, *AITPS_TV;

#if !defined(BUILD_FW)
// LCD  OPR
#define DSPY_CTL_0                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_0              )))
#define DSPY_CTL_2	                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_2              )))
#define DSPY_LCD_TX_6               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_LCD_TX_6           )))

#define DSPY_W                      (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_W                  )))
#define DSPY_H                      (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_H                  )))
#define DSPY_PIXL_CNT               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIXL_CNT           )))
#define DSPY_CTL_4                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_4              )))
#define DSPY_BG_COLOR        		(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_BG_COLOR           )))

#define DSPY_PIP_OFST_ROW           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_OFST_ROW       )))
#define DSPY_PIP_SOUT_CTL           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_SOUT_CTL       )))
#define DSPY_PIP_TV_EVEN_FIELD_ST 	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_TV_EVEN_FIELD_ST)))

// SCD OPR
#define DSPY_SCD_CTL                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_CTL            )))
#define DSPY_SCD_BG_COLOR       	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_BG_COLOR       )))
#define DSPY_SCD_W                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_W              )))
#define DSPY_SCD_H                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_H              )))
#define DSPY_SCD_PIXL_CNT           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_PIXL_CNT       )))

// RGB OPR
#define DSPY_RGB_SYNC_MODE     		(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_SYNC_MODE      )))
#define DSPY_RGB_CTL                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_CTL            )))
#define DSPY_RGB_FMT                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_FMT            )))
#define DSPY_RGB_SHARE_P_LCD_BUS    (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_SHARE_P_LCD_BUS)))
#define DSPY_RGB_DOT_CLK_RATIO   	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_DOT_CLK_RATIO   )))
#define DSPY_RGB_PORCH_HIGH_BIT_EXT (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PORCH_HIGH_BIT_EXT)))
#define DSPY_RGB_V_BPORCH           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_BPORCH       )))
#define DSPY_RGB_V_BLANK            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_BLANK        )))
#define DSPY_RGB_H_BPORCH           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_H_BPORCH       )))
#define DSPY_RGB_H_BLANK            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_H_BLANK        )))
#define DSPY_RGB_HSYNC_W            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_HSYNC_W        )))
#define DSPY_RGB_VSYNC_W            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_VSYNC_W        )))
#define DSPY_RGB_V_2_H_DOT          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_2_H_DOT      )))
#define DSPY_RGB_PRT_2_H_DOT        (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PRT_2_H_DOT    )))
#define DSPY_RGB_PART_ST_X          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ST_X      )))
#define DSPY_RGB_PART_ST_Y          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ST_Y      )))
#define DSPY_RGB_PART_ED_X          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ED_X      )))
#define DSPY_RGB_PART_ED_Y          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ED_Y      )))
#define DSPY_RGB_LINE_CPU_INT_1		(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_LINE_CPU_INT_1 )))
#define DSPY_RGB_LINE_CPU_INT_2		(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_LINE_CPU_INT_2 )))
#define DSPY_RGB_LINE_HOST_INT_1	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_LINE_HOST_INT_1)))
#define DSPY_RGB_LINE_HOST_INT_2	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_LINE_HOST_INT_2)))
#define DSPY_INT_CPU_EN_2           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_INT_CPU_EN_2       )))
#define DSPY_INT_CPU_SR_2           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_INT_CPU_SR_2       )))
#define DSPY_RGB_DELTA_MODE         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_DELTA_MODE             )))

// PALETTE OPR
#define MAIN_PALETTE_R0             0x8000E400
#define MAIN_PALETTE_G0             0x8000E200
#define MAIN_PALETTE_B0             0x8000E000

#define OVERLAY_PALETTE_R0          0x8000F400
#define OVERLAY_PALETTE_G0          0x8000F200
#define OVERLAY_PALETTE_B0          0x8000F000

#define ICON_PALETTE_R0             0x8000DC00
#define ICON_PALETTE_G0             0x8000DA00
#define ICON_PALETTE_B0             0x8000D800

// TV OPR
#define TVIF_DAC_IF_1ST_CTL         (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_DAC_IF_1ST_CTL       )))
#define TVIF_BACKGROUND_Y_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_Y_COLOR   )))
#define TVIF_BACKGROUND_U_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_U_COLOR   ))) 
#define TVIF_BACKGROUND_V_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_V_COLOR   )))
#define TVIF_CLK_DELAY_V1           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_CLK_DELAY_V1         ))) 
#define TVIF_IF_EN                  (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IF_EN                )))
#define TVIF_ENDLINE_OFFSET_CTL     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_ENDLINE_OFFSET_CTL   )))
#define TVIF_EARLY_PIXL		        (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_EARLY_PIXL           )))
#define TVIF_1ST_PXL_RQST_TIMING    (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_1ST_PXL_RQST_TIMING  ))) 
#define TVIF_NTSC_ODFIELD_LINE      (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_ODFIELD_LINE    )))
#define TVIF_NTSC_EVFIELD_LINE      (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_EVFIELD_LINE    )))
#define TVIF_PAL_ODFIELD_LINE       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_ODFIELD_LINE     )))
#define TVIF_PAL_EVFIELD_LINE       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_EVFIELD_LINE     ))) 
#define TVIF_NTSC_EVLINE_SUB1       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_EVLINE_SUB1     )))
#define TVIF_PAL_EVLINE_SUB1        (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_EVLINE_SUB1      )))
#define TVIF_IMAGE_WIDTH            (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_WIDTH          )))
#define TVIF_IMAGE_HEIGHT           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_HEIGHT         )))
#define TVIF_IMAGE_START_X          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_START_X        )))
#define TVIF_IMAGE_START_Y          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_START_Y        )))
#define TVENC_Y_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_Y_SCALE_CTL         )))
#define TVENC_U_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_U_SCALE_CTL         )))
#define TVENC_V_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_V_SCALE_CTL         ))) 
#define TVENC_GAMMA_COEF_0          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_GAMMA_COEF_0        )))
#define TVENC_UV_SCALE_GAIN_4_5     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_UV_SCALE_GAIN_4_5   )))
#endif
#endif //(CHIP == P_V2)

#if (CHIP == MCR_V2)
__packed typedef union {
	__packed struct {
		AIT_REG_D   TP_COLOR;	    		// 0x00
	    AIT_REG_B   _reserved[0x2];			// 0x04
	} SHT;

	__packed struct {
		AIT_REG_B   B_COLOR;				// 0x00
		AIT_REG_B   B_MASK;					// 0x01
		AIT_REG_B   G_COLOR;				// 0x02
		AIT_REG_B   G_MASK;					// 0x03
		AIT_REG_B   R_COLOR;				// 0x04
		AIT_REG_B   R_MASK;					// 0x05
	} MP;
} AIT_DSPY_TP;

// *******************************
//   Display structure (0x8000 2800)
// *******************************
typedef struct _AITS_DSPY {
    AIT_REG_W   DSPY_CTL_0;                                             // 0x0000
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_FRAME_TX                0x0002
        #define LCD_IDX_RDY                 0x0004
        #define LCD_CMD_RDY                 0x0008
        #define LCD_FRAME_TX_SETADDR_EN     0x0010
        #define LCD_PANEL_READ_EN         	0x0040
        #define LCD_BUSY_STATUS             0x0080
        #define SCD_DSPY_REG_READY          0x0100
        #define SW_UPDATE_PRM_OPR_NOW		0x8000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_CTL_2;                                             // 0x0002
        /*-DEFINE-----------------------------------------------------*/
        #define PRM_DSPY_REG_READY          0x0001
        #define LCD_A0_DIS                  0x0002
        #define LCD_RD_DIS                  0x0004
        #define LCD_WR_DIS                  0x0008
        #define LCD_CS2_DIS                 0x0010
        #define LCD_CS1_DIS                 0x0020
        #define TV_FIEND_SYNC_EN            0x0040
        #define TV_FRAME_SYNC_EN			0x0000
        #define TV_SYNC_OPR_DIS	          	0x0080

        #define DSPY_TYPE_PL_LCD            0x00
        #define DSPY_TYPE_SPI_LCD           0x01
        #define DSPY_TYPE_RGB_LCD           0x02
        #define DSPY_TYPE_TV                0x03
        #define DSPY_TYPE_PRM_DSI           0x40
        #define DSPY_TYPE_PRM_RGB2			0x41
        #define DSPY_TYPE_SCD_DSI           0x20
        #define DSPY_TYPE_SCD_RGB2			0x21
        #define DSPY_PRM_SEL_SHIFT          8
        #define DSPY_PRM_SEL(_a)            (MMP_USHORT)(_a << DSPY_PRM_SEL_SHIFT)
        #define DSPY_PRM_SEL_MASK           0x4300
        #define DSPY_PRM_EN                 0x1000
        #define DSPY_SCD_SEL_SHIFT          10
        #define DSPY_SCD_SEL(_a)            (MMP_USHORT)(_a << DSPY_SCD_SEL_SHIFT)
        #define DSPY_SCD_SEL_MASK           0x8C00
        #define DSPY_SCD_EN                 0x2000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_INT_HOST_EN;                                       // 0x0004
    AIT_REG_W   DSPY_INT_HOST_SR;										// 0x0006
    AIT_REG_W   DSPY_INT_CPU_EN;										// 0x0008
    AIT_REG_W   DSPY_INT_CPU_SR;										// 0x000A
        /*-DEFINE-----------------------------------------------------*/
        #define PRM_FRME_TX_END             0x0001
        #define PRM_IDX_TX_END              0x0002
        #define PRM_CMD_TX_END              0x0004
        #define SCD_FRME_TX_END             0x0008
        #define SCD_IDX_TX_END              0x0010
        #define SCD_CMD_TX_END              0x0020
        #define RGB_VSYNC_ACTIVE        	0x0040
        #define RGB_FRME_CNT_HIT            0x0080
        #define TV_LINE_INT1                0x0100
        #define TV_LINE_INT2                0x0200
        #define TV_READ_DSPY_BUF_UDF     	0x0400
        #define TV_ODD_FIELD_START          0x0800
        #define TV_EVEN_FIELD_START         0x1000
        #define TV_COMP_OUT_L2H             0x2000
        #define TV_COMP_OUT_H2L             0x4000
        #define LCD_FIFO_FINISHED			0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_LCD_FLM_CTL;                                       // 0x000C
    	/*-DEFINE-----------------------------------------------------*/
    	#define LCD_CHK_FLM_EN				0x01
    	#define FLM_SIG_ACT_LOW				0x02
    	#define FLM_SIG_ACT_HIGH			0x00
    	#define FLM_INT_MODE_CNT_DONE		0x04
    	#define THIRD_BYTE_OF_FLM_CNT_MASK	0xF0
    	/*------------------------------------------------------------*/
    AIT_REG_B   DSPY_LCD_VSYNC_CTL;                                  	// 0x000D
		/*-DEFINE-----------------------------------------------------*/
		#define LCD_OUT_VSYNC_EN			0x01
    	#define VSYNC_SIG_ACT_LOW			0x02
    	#define VSYNC_SIG_ACT_HIGH			0x00
		/*------------------------------------------------------------*/
    AIT_REG_W   DSPY_FLM_VSYNC_CNT;                                     // 0x000E

    AIT_REG_D   DSPY_LCD_TX_0;                                          // 0x0010
    AIT_REG_D   DSPY_LCD_TX_1;											// 0x0014
    AIT_REG_D   DSPY_LCD_TX_2;											// 0x0018
    AIT_REG_D   DSPY_LCD_TX_3;											// 0x001C

    AIT_REG_D   DSPY_LCD_TX_4;                                          // 0x0020
    AIT_REG_D   DSPY_LCD_TX_5;											// 0x0024
    AIT_REG_D   DSPY_LCD_TX_6;											// 0x0028
    AIT_REG_W   DSPY_LCD_AUTO_CFG;										// 0x002C
        /*-DEFINE-----------------------------------------------------*/
        #define AUTO_TX_TYPE_IDX            0x0000
        #define AUTO_TX_TYPE_CMD            0x0001
        #define LCD_TX_TYPE_IDX(_n)         (AUTO_TX_TYPE_IDX << _n)
        #define LCD_TX_TYPE_CMD(_n)         (AUTO_TX_TYPE_CMD << _n)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x002E[0x2];

    AIT_REG_W   DSPY_PLCD_CTL;                                          // 0x0030
        /*-DEFINE-----------------------------------------------------*/
        #define PLCD_BUS_8BPP               0x0000
        #define PLCD_BUS_16BPP              0x0001
        #define PLCD_BUS_18BPP              0x0002
        #define PLCD_BUS_12BPP              0x0003
        #define PLCD_BUS_MASK               0x0003
        #define PLCD_68SYS_RC_HIGH       	0x0004
        #define PLCD_CMD_BURST              0x0008
        #define PLCD_CMD_NONBURST			0x0000
        #define PLCD_PHA_0                  0x0000
        #define PLCD_PHA_1                  0x0010
        #define PLCD_POR_0                  0x0000
        #define PLCD_POR_1                  0x0020
        #define PLCD_TYPE_80                0x0040
        #define PLCD_TYPE_68                0x0000
        #define LCD_SPI1_PL2				0x0000
        #define LCD_PL1_SPI2                0x0100
        #define LCD_PL1_PL2                 0x0200
        #define LCD_SPI1_SPI2               0x0300
        #define PLCD_RS_LEAD_CS_EN      	0x0400
        #define A0_SIG_LOW_FOR_IDX			0x0000
        #define A0_SIG_HIGH_FOR_IDX			0x0800
        #define RGB565_TX_GB_FIRST			0x1000
        #define RGB565_TX_RG_FIRST			0x0000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PLCD_FMT;											// 0x0032
        /*-DEFINE-----------------------------------------------------*/
        #define PLCD_RGB565_BUS16           0x0000
        #define PLCD_RGB444_BUS16           0x0001
        #define PLCD_RGB666_BUS16           0x0002
        #define PLCD_RGB888_BUS8            0x0003
        #define PLCD_RGB332_BUS8            0x0004
        #define PLCD_RGB444_BUS8            0x0005
        #define PLCD_RGB666_BUS8            0x0006
        #define PLCD_RGB565_BUS8            0x0007
        #define PLCD_RGB666_BUS18           0x0008
        #define PLCD_RGB666_BUS18_9_9     	0x0009
        #define PLCD_RGB666_BUS18_2_16      0x000A
        #define PLCD_RGB666_BUS18_16_2      0x000B
        #define PLCD_RGB24_BUS18_16_8     	0x000C
        #define PLCD_RGB24_BUS18_8_16  		0x000D
        #define PLCD_RGB18_BUS8_2_8_8		0x000E
        #define PLCD_RGB666_BUS8_2_7       	0x0010
        #define PLCD_RGB444_B12_EXT_B16   	0x0020
        #define PLCD_RGB444_B15_14     		0x0040
        #define PLCD_RGBB9_9_17             0x0080

        #define PLCD_RGB666_B9              0x0100
        #define LCD_RGB666_B2_16            0x0200
        #define LCD_RGB666_B16_2            0x0300
        #define LCD_B8_RGB666_12_17         0x0400
        #define LCD_B16_1_8_10_17           0x0800
        #define LCD_B8_10_17                0x1000
        #define LCD_B8_1_8                  0x2000
        #define LCD_B8_RGB565_G_LSB_FIRST   0x4000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_LCD_SR;											// 0x0034
        /*-DEFINE-----------------------------------------------------*/
        #define PL_LCD_BUSY                 0x0001
        #define SPI_LCD_BUSY                0x0002
        #define SPI_RGB_LCD_BUSY			0x0004
        #define TV_PLUG_OUT_STATUS      	0x0008
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_TV_LINE;                                         	// 0x0036
    AIT_REG_W   DSPY_PLCD_RS_LEAD_CS_CYC;								// 0x0038
    AIT_REG_W   DSPY_PLCD_CS_LEAD_RW_CYC;								// 0x003A
    AIT_REG_W   DSPY_PLCD_RW_CYC;										// 0x003C
    AIT_REG_W   DSPY_PLCD_IDX_CMD_NUM;									// 0x003E
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_PLCD_IDX_CMD_MAX_NUM	0x0007
        /*------------------------------------------------------------*/

    AIT_REG_W   DSPY_W;                                                 // 0x0040
    AIT_REG_B                           _x0042[0x2];
    AIT_REG_W   DSPY_H;													// 0x0044
    AIT_REG_B                           _x0046[0x2];
    AIT_REG_D   DSPY_PIXL_CNT;											// 0x0048
    AIT_REG_B   DSPY_CTL_4;												// 0x004C
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_OUT_RGB                 0x0000
        #define LCD_OUT_BGR                 0x0002
        #define LCD_OUT_SEL_NONE            0x0000
        #define LCD_OUT_SEL_LCD1            0x0004
        #define LCD_OUT_SEL_LCD2            0x0008
        #define LCD_OUT_SEL_MASK            0x000C
        #define LCD_BG_COLR_565             0x0000 // [Reserved]
        #define LCD_BG_COLR_888             0x0020 // [Reserved]
    AIT_REG_B   DSPY_OSD_MCI_PORT_SEL_MP;    							// 0x004D
        /*------------------------------------------------------------*/
        #define MCI_PORT_3_SHARE            0x00
        #define MCI_PORT_1_SHARE            0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_WIN_PRIO;											// 0x004E
        /*-DEFINE-----------------------------------------------------*/
        #define MAIN_WIN                    0x0
        #define PIP_WIN                     0x1
        #define OVLY_WIN                    0x2
        #define ICON_WIN                    0x3
        #define WIN_1_SHFT                  0
        #define WIN_2_SHFT                  2
        #define WIN_3_SHFT                  4
        #define WIN_4_SHFT                  6
        #define WIN_PRIO(p0,p1,p2,p3)		((p3 << 6) | (p2 << 4) | (p1 << 2) | p0)
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_WIN_BIND_SEL_MP;    								// 0x004F
        /*-DEFINE-----------------------------------------------------*/
        #define MAIN_WIN_SHIFT              0x06
        #define PIP_WIN_SHIFT              	0x04
        #define OVLY_WIN_SHIFT              0x02
        #define OSD_WIN_SHIFT             	0x00
        #define BIND_SEL_MASK				0x03
        #define PRM_SEL						0x02
        #define SCD_SEL						0x01
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_ICON_W;                                            // 0x0050
    AIT_REG_B                           _x0052[0x2];
    AIT_REG_W   DSPY_ICON_H;											// 0x0054
    AIT_REG_B                           _x0056[0x2];
    AIT_REG_W   DSPY_ICON_X;											// 0x0058
    AIT_REG_B                           _x005A[0x2];
    AIT_REG_W   DSPY_ICON_Y;											// 0x005C
    AIT_REG_B                           _x005E[0x2];

    AIT_REG_W   DSPY_ICON_CTL;                                          // 0x0060
    	/*-DEFINE-----------------------------------------------------*/
    	#define ICON_WIN_ENABLE				0x0001
    	/*------------------------------------------------------------*/
    AIT_REG_B                           _x0062[0x2];
    AIT_REG_D   DSPY_BG_COLOR;											// 0x0064
    AIT_REG_D   DSPY_PLCD_READ_PORT;									// 0x0068
    AIT_REG_W   DSPY_FIFO_CLR;											// 0x006C
        /*-DEFINE-----------------------------------------------------*/
        #define MAIN_FIFO_CLR               0x0001
        #define PIP_FIFO_CLR                0x0002
        #define OVLY_FIFO_CLR               0x0004
        #define OSD_FIFO_CLR                0x0008
        #define PIP_SCAL_BUF_CLR            0x0010
        #define OVLY_RGB_OUT_CLR            0x0020
        #define MAIN_RGB_OUT_CLR            0x0040
        #define OSD_RGB_OUT_CLR            	0x0080
        
        #define PRM_DSPY_BUF_CLR			0x0100
        #define SCD_DSPY_BUF_CLR			0x0200
        #define PRM_ALPHA_BUF_CLR			0x0400
        #define SCD_ALPHA_BUF_CLR			0x0800
        // ++MCR_V2_MP Only
        #define PRM_OVLP_BUF_CLR_MP			0x0400
        #define SCD_OVLP_BUF_CLR_MP			0x0800
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_W   TV_DAC_TEST_MODE_DATA;									// 0x006E
    
    AIT_REG_W   TVENC[8*9];                                             // 0x0070-0x00FF

    AIT_REG_D   DSPY_MAIN_ADDR_ST;                                    	// 0x0100
    AIT_REG_B                           _x0104[0x4];
    AIT_REG_D   DSPY_MAIN_OFST_ST;										// 0x0108
    AIT_REG_B                           _x010C[0x4];

    AIT_REG_W   DSPY_MAIN_OFST_PIXL;                                    // 0x0110
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_OFST_NEG                0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0112[0x2];
    AIT_REG_W   DSPY_MAIN_OFST_ROW;										// 0x0114
    AIT_REG_B                           _x0116[0xA];
    
    AIT_REG_D   DSPY_MAIN_U_ADDR_ST;                                  	// 0x0120
    AIT_REG_B                           _x0124[0x4];
    AIT_REG_D   DSPY_MAIN_OFST_UV_ST;									// 0x0128
    AIT_REG_B                           _x012C[0x4];

    AIT_REG_W   DSPY_MAIN_OFST_UV_PIXL;                                 // 0x0130
    AIT_REG_B                           _x0132[0x2];
    AIT_REG_W   DSPY_MAIN_OFST_UV_ROW;									// 0x0134
    AIT_REG_B                           _x0136[0xA];

    AIT_REG_D   DSPY_MAIN_V_ADDR_ST;                                  	// 0x0140
    AIT_REG_B                           _x0144[0x4];
    AIT_REG_B	DSPY_MAIN_DBG_OBS_SEL_MP;								// 0x0148
    AIT_REG_B	DSPY_MAIN_FIFO_MAX_CTL_MP;								// 0x0149
    AIT_REG_B                           _x014A[0x2];
    AIT_REG_B   DSPY_MAIN_DBG_SEL_BYTE_MP[4];							// 0x014C

    AIT_REG_W   DSPY_MAIN_W;                                            // 0x0150
    AIT_REG_B                           _x0152[0x2];
    AIT_REG_W   DSPY_MAIN_H;											// 0x0154
    AIT_REG_B                           _x0156[0x1];
    AIT_REG_W   DSPY_MAIN_X;											// 0x0158
    AIT_REG_B                           _x015A[0x2];
    AIT_REG_W   DSPY_MAIN_Y;											// 0x015C
    AIT_REG_B                           _x015E[0x2];

    AIT_REG_W   DSPY_MAIN_CTL;                                  		// 0x0160
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_EN                      0x0001
        #define WIN_ROTATE_EN               0x0002
        #define WIN_SRC_GRAB_EN             0x0004
        #define WIN_YUV_SCALUP_EN           0x0008	// [Main:X, PIP:X, OSD:X, OVL:X]
        #define WIN_IN_RGB                  0x0000
        #define WIN_IN_BGR                  0x0040
        #define WIN_OUT_565_EXT_888      	0x0080
        
        #define WIN_YUV420_DITHER_18BIT 	0x0100	// [Main:O, PIP:X, OSD:O, OVL:X]
        #define WIN_YUV420_RING_BUF_EN    	0x0200
        #define WIN_YUV_LPF_DIS          	0x0400
        #define WIN_YUV420_DITHER_16BIT  	0x0800	// [Main:O, PIP:X, OSD:O, OVL:X]
        #define WIN_YUV420_DITHER_12BIT  	0x1000	// [Main:O, PIP:X, OSD:O, OVL:X]
        #define WIN_YUV420_INTERLEAVE_MASK  0xC000
        #define WIN_YUV420_INTERLEAVE_UV    0x8000
        #define WIN_YUV420_INTERLEAVE_VU    0xC000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_CTL_2;                              			// 0x0162
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_V_1X                    0x0001
        #define WIN_V_2X                    0x0002
        #define WIN_V_4X                    0x0004
        #define WIN_H_1X                    0x0008
        #define WIN_H_2X                    0x0010
        #define WIN_H_4X                    0x0020
        #define WIN_V_DUP_MASK				0x0007
        #define WIN_H_DUP_MASK				0x0038
        #define WIN_DUP_MASK				0x003F
			
		#define WIN_FIFO_INTERLACE_LATCH	0x0100
		#define WIN_FIFO_PROGRESS_LATCH		0x0000
        // ++MCR_V2_MP Only
        #define WIN_V_1X_MP                 0x0001
        #define WIN_V_2X_MP                 0x0002
        #define WIN_V_3X_MP                 0x0003
		#define WIN_V_4X_MP                 0x0004
		#define WIN_V_5X_MP                 0x0005
		#define WIN_V_6X_MP                 0x0006
		#define WIN_H_1X_MP                 0x0008
		#define WIN_H_2X_MP                 0x0010
		#define WIN_H_3X_MP                 0x0018
		#define WIN_H_4X_MP                 0x0020
		#define WIN_H_5X_MP                 0x0028
		#define WIN_H_6X_MP                 0x0030
		#define WIN_V_DIV_2_MP              0x0040
		#define WIN_H_DIV_2_MP              0x0080
		// --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_FMT;											// 0x0164
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_4BPP                    0x0001	// MAIN,      OSD, 
        #define WIN_8BPP                    0x0002	// MAIN,      OSD, 
        #define WIN_16BPP                   0x0004	// MAIN, PIP, OSD, OVLY
        #define WIN_24BPP                   0x0008	// MAIN, PIP, OSD,
        #define WIN_YUV420                  0x0010	// MAIN, PIP, OSD, OVLY
        #define WIN_YUV422                  0x0020	// MAIN, PIP, OSD, OVLY
        #define WIN_32BPP                   0x0040	// MAIN, PIP, OSD, OVLY
        
        #define WIN_16BIT_RGB565        	0x0000	// MAIN, PIP, OSD, OVLY
        #define WIN_16BIT_ARGB3454        	0x0200	// MAIN, PIP, OSD, OVLY
        #define WIN_16BIT_ARGB4444			0x0300	// MAIN,	, OSD, OVLY
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0166[0x2];
    AIT_REG_D   DSPY_MAIN_PIXL_CNT;										// 0x0168	
    AIT_REG_B                           _x016C[0x4];

    AIT_REG_W   DSPY_MAIN_TP_CTL;                               		// 0x0170
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_TP_EN                   0x0001
        #define WIN_ALPHA_BLEND_EN          0x0002
        #define WIN_ALPHA_BLEND_DIS         0x0000
        #define WIN_ALPHA_BLEND_GLOBAL     	0x0000
        #define WIN_ALPHA_BLEND_LOCAL      	0x0004
        #define WIN_ALPHA_BLEND_OP_AND      0x0008
        #define WIN_ALPHA_BLEND_OP_OR       0x000C
        #define WIN_ALPHA_BLEND_OP_INV_UP   0x0010
        #define WIN_ALPHA_BLEND_OP_INV_DN   0x0014
        #define WIN_ALPHA_BLEND_MASK    	0x001C
        #define WIN_ALPHA_LSB               0x0020
        #define WIN_ALPHA_MSB               0x0000
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_MAIN_GLOBAL_ALPHA_WT;								// 0x0172
    AIT_DSPY_TP DSPY_MAIN_TP;											// 0x0174
	AIT_REG_W   DSPY_MAIN_WIDTH_DUP_DIV_MP;								// 0x017A
	AIT_REG_D   DSPY_MAIN_PIXL_CNT_DUP_DIV_MP;							// 0x017C

	AIT_REG_B                           _x0180[0x10];

    AIT_REG_B   DSPY_MAIN_UV_GAIN_11;                                   // 0x0190
    AIT_REG_B   DSPY_MAIN_UV_GAIN_12;									// 0x0191
    AIT_REG_B   DSPY_MAIN_UV_GAIN_21;									// 0x0192
    AIT_REG_B   DSPY_MAIN_UV_GAIN_22;									// 0x0193
    AIT_REG_W   DSPY_MAIN_RGB_GAIN;										// 0x0194
    AIT_REG_W   DSPY_MAIN_RGB_OFST;										// 0x0196
    AIT_REG_B                           _x0198[0x8];
    
    AIT_REG_W   DSPY_SCD_CTL;						                    // 0x01A0
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_CTL_0
        #define	SCD_SRC_RGB888				0x0100
        #define	SCD_SRC_RGB565				0x0200
        #define	SCD_SRC_FMT_MASK			0x0300
        #define	SCD_565_2_888_STUFF_0		0x0000
        #define	SCD_565_2_888_STUFF_MSB		0x0400
        #define SW_UPDATE_SCD_OPR_NOW		0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x01A2[0x2];
    AIT_REG_B   DSPY_SCD_FLM_CTL;                                       // 0x01A4
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_LCD_FLM_CTL
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_SCD_VSYNC_CTL;                                     // 0x01A5	
        /*-DEFINE-----------------------------------------------------*/
        // Ref: DSPY_LCD_VSYNC_CTL
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_SCD_FLM_VSYNC_CNT;                                 // 0x01A6
    AIT_REG_D   DSPY_SCD_BG_COLOR;                                 		// 0x01A8
    AIT_REG_D   DSPY_SCD_PLCD_READ_PORT;								// 0x01AC

    AIT_REG_D   DSPY_SCD_LCD_TX_0;                                      // 0x01B0
    AIT_REG_D   DSPY_SCD_LCD_TX_1;										// 0x01B4
    AIT_REG_D   DSPY_SCD_LCD_TX_2;										// 0x01B8
    AIT_REG_W   DSPY_SCD_LCD_AUTO_CFG;									// 0x01BC
    AIT_REG_B                           _x01BE[0x2];
    
    AIT_REG_W   DSPY_SCD_W;                                             // 0x01C0
    AIT_REG_B                           _x01C2[0x2];
    AIT_REG_W   DSPY_SCD_H;												// 0x01C4
    AIT_REG_B                           _x01C6[0x2];
    AIT_REG_D   DSPY_SCD_PIXL_CNT;										// 0x01C8
    AIT_REG_D   DSPY_SCD_WIN_PIXL_CNT;									// 0x01CC
    
    AIT_REG_D   DSPY_SCD_WIN_ADDR_ST;                                 	// 0x01D0
    AIT_REG_D   DSPY_SCD_WIN_OFST_ST;									// 0x01D4
    AIT_REG_W   DSPY_SCD_WIN_W;                                         // 0x01D8
    AIT_REG_W   DSPY_SCD_WIN_H;                                         // 0x01DA
    AIT_REG_W   DSPY_SCD_WIN_X;                                         // 0x01DC
    AIT_REG_W   DSPY_SCD_WIN_Y;                                         // 0x01DE
    
    AIT_REG_D   DSPY_MAIN_TV_EVEN_FIELD_ST;                      		// 0x01E0
    AIT_REG_B                           _x01E4[0x4];
	AIT_REG_D	DSPY_WIN_ALPHA_WT_1;									// 0x01E8
	AIT_REG_D	DSPY_WIN_ALPHA_WT_2;									// 0x01EC
        /*-DEFINE-----------------------------------------------------*/
		#define ALPHA_W_1(l, w)         	((w | ((8-w) << 4)) << (l * 8))    
		#define ALPHA_W_2(l, w)         	((w | ((8-w) << 4)) << ((l - 4)* 8))    
        /*------------------------------------------------------------*/

    AIT_REG_B							_x01F0[0x8];
    AIT_REG_W   DSPY_MAIN_Y_FIFO_CTL;                                   // 0x01F8
    AIT_REG_W   DSPY_MAIN_U_FIFO_CTL;                                   // 0x01FA
    AIT_REG_W   DSPY_MAIN_V_FIFO_CTL;                                   // 0x01FC 
    AIT_REG_B                 			_x01FE[0x2];

    AIT_REG_D   DSPY_PIP_ADDR_ST;                                    	// 0x0200
    AIT_REG_B                 			_x0204[0x4];
    AIT_REG_D   DSPY_PIP_OFST_ST;										// 0x0208
    AIT_REG_B                 			_x020C[0x4];

    AIT_REG_W   DSPY_PIP_OFST_PIXL;                                     // 0x0210
    AIT_REG_B                           _x0212[0x2];
    AIT_REG_W   DSPY_PIP_OFST_ROW;										// 0x0214
    AIT_REG_B                           _x0216[0xA];

    AIT_REG_D   DSPY_PIP_U_ADDR_ST;                                   	// 0x0220
    AIT_REG_B                 			_x0224[0x4];
    AIT_REG_D   DSPY_PIP_OFST_UV_ST;									// 0x0228
    AIT_REG_B                 			_x022C[0x4];
    
    AIT_REG_W   DSPY_PIP_OFST_UV_PIXL;                                  // 0x0230
    AIT_REG_B                           _x0232[0x2];
    AIT_REG_W   DSPY_PIP_OFST_UV_ROW;									// 0x0234
    AIT_REG_B                           _x0236[0xA];
 
    AIT_REG_D   DSPY_PIP_V_ADDR_ST;                                   	// 0x0240
    AIT_REG_B                           _x0244[0x4];
    AIT_REG_B	DSPY_PIP_DBG_OBS_SEL_MP;								// 0x0248
    AIT_REG_B	DSPY_PIP_FIFO_MAX_CTL_MP;								// 0x0249
    AIT_REG_B                           _x024A[0x2];
    AIT_REG_B   DSPY_PIP_DBG_SEL_BYTE_MP[4];							// 0x024C

    AIT_REG_W   DSPY_PIP_W;                                             // 0x0250
    AIT_REG_B                           _x0252[0x2];
    AIT_REG_W   DSPY_PIP_H;												// 0x0254
    AIT_REG_B                           _x0256[0x2];
    AIT_REG_W   DSPY_PIP_X;												// 0x0258
    AIT_REG_B                           _x025A[0x2];
    AIT_REG_W   DSPY_PIP_Y;												// 0x025C
    AIT_REG_B                           _x025E[0x2];

    AIT_REG_W   DSPY_PIP_CTL;                                           // 0x0260
    AIT_REG_W   DSPY_PIP_CTL_2;                                         // 0x0262
        /*-DEFINE-----------------------------------------------------*/
        #define WIN_SCAL_OUT_DROP_LINE		0x0200
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_FMT;											// 0x0264
    AIT_REG_B                           _x0266[0x2];
    AIT_REG_D   DSPY_PIP_PIXL_CNT;										// 0x0268
    AIT_REG_D   DSPY_SOUT_GRAB_PIXL_CNT;								// 0x026C

    AIT_REG_W   DSPY_PIP_TP_CTL;                                        // 0x0270
    AIT_REG_W   DSPY_PIP_GLOBAL_ALPHA_WT;								// 0x0272
	AIT_DSPY_TP DSPY_PIP_TP;											// 0x0274
	AIT_REG_W   DSPY_PIP_WIDTH_DUP_DIV_MP;								// 0x027A
	AIT_REG_D   DSPY_PIP_PIXL_CNT_DUP_DIV_MP;							// 0x027C

    AIT_REG_W   DSPY_PIP_SCAL_CTL;                                   	// 0x0280
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_NM            	0x0000
        #define DSPY_SCAL_BYPASS        	0x0001      // bypass N/M and LPF
        #define DSPY_SCAL_WT_DYN     		0x0000
        #define DSPY_SCAL_WT_FIX          	0x0002
        #define DSPY_SCAL_BICUBIC_IDX		0x000C
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SOUT_CTL;										// 0x0282
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SOUT_YUV_EN        	0x0001
        #define DSPY_SOUT_RGB_EN        	0x0002
        #define DSPY_SOUT_YUV_444       	0x0000
        #define DSPY_SOUT_YUV_422       	0x0008
        #define DSPY_SOUT_RGB_888      	 	0x0000
        #define DSPY_SOUT_RGB_565       	0x0010
        #define DSPY_SOUT_RGB_666       	0x0020
        #define DSPY_SOUT_DITHER_EN     	0x0040
        #define DSPY_SOUT_RGB           	0x0000
        #define DSPY_SOUT_BGR           	0x0080
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SCAL_H_N;										// 0x0284
    AIT_REG_W   DSPY_PIP_SCAL_H_M;										// 0x0286
    AIT_REG_W   DSPY_PIP_SCAL_V_N;										// 0x0288
    AIT_REG_W   DSPY_PIP_SCAL_V_M;										// 0x028A
    AIT_REG_W   DSPY_PIP_SEDGE_CTL;										// 0x028C
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SEDGE_BYPASS       	0x0001
        #define DSPY_SEDGE_YUV_AVG      	0x0002
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_PIP_SEDGE_GAIN_VAL;								// 0x028E
    AIT_REG_B   DSPY_PIP_SEDGE_CORE_VAL;								// 0x028F

    AIT_REG_B   DSPY_PIP_UV_GAIN_11;                          			// 0x0290
    AIT_REG_B   DSPY_PIP_UV_GAIN_12;									// 0x0291
    AIT_REG_B   DSPY_PIP_UV_GAIN_21;									// 0x0292
    AIT_REG_B   DSPY_PIP_UV_GAIN_22;									// 0x0293
    AIT_REG_W   DSPY_PIP_RGB_GAIN;										// 0x0294
    AIT_REG_W   DSPY_PIP_RGB_OFST;										// 0x0296
    AIT_REG_W   DSPY_PIP_SCAL_H_WT;										// 0x0298
    AIT_REG_W   DSPY_PIP_SCAL_V_WT;										// 0x029A
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_WT_AVG 	    	0x0001
        #define DSPY_SCAL_WT_MASK       	0xFFFE
        #define DSPY_SCAL_WT(_w)			((_w << 1) & DSPY_SCAL_WT_MASK)
        /*------------------------------------------------------------*/
    AIT_REG_W   DSPY_PIP_SCAL_WT_FORCE1_EN;								// 0x029C
        /*-DEFINE-----------------------------------------------------*/
        #define DSPY_SCAL_WT_FORCE1_EN	    0x0001
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x029E[0x2];

    AIT_REG_W   DSPY_PIP_SCA_IN_W;                                      // 0x02A0
    AIT_REG_B                           _x02A2[0x2];
    AIT_REG_W   DSPY_PIP_SCA_IN_H;										// 0x02A4
    AIT_REG_B                           _x02A6[0xA];

    AIT_REG_W   DSPY_PIP_SOUT_GRAB_H_ST;                                // 0x02B0
    AIT_REG_B                           _x02B2[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_H_ED;								// 0x02B4
    AIT_REG_B                           _x02B6[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_V_ST;								// 0x02B8
    AIT_REG_B                           _x02BA[0x2];
    AIT_REG_W   DSPY_PIP_SOUT_GRAB_V_ED;								// 0x02BC
    AIT_REG_B                           _x02BE[0x2];

    AIT_REG_D   DSPY_Y_ADDR_LOW_BOUND;									// 0x02C0
    AIT_REG_D   DSPY_Y_ADDR_HIGH_BOUND;                             	// 0x02C4
    AIT_REG_D   DSPY_U_ADDR_LOW_BOUND;									// 0x02C8
    AIT_REG_D   DSPY_U_ADDR_HIGH_BOUND;                             	// 0x02CC
	
	AIT_REG_W   DSPY_MPEG4_DEC_ROW_CNT;									// 0x02D0
	AIT_REG_B                           _x02D2[0x6];
    AIT_REG_D   DSPY_V_ADDR_LOW_BOUND;									// 0x02D8
    AIT_REG_D   DSPY_V_ADDR_HIGH_BOUND;                             	// 0x02DC
	
    AIT_REG_D   DSPY_PIP_TV_EVEN_FIELD_ST;                          	// 0x02E0
    AIT_REG_B                           _x02E4[0xC];
    
    AIT_REG_B   DSPY_PIP_BUF_FULL_THD;                                  // 0x02F0
    AIT_REG_B   DSPY_PIP_BUF_STOP_THD;                                  // 0x02F1
    AIT_REG_B                           _x02F2[0x2];
    AIT_REG_W   DSPY_PRM_BUF_FULL_THD;                                  // 0x02F4                    
    AIT_REG_W   DSPY_SCD_BUF_FULL_THD;                                  // 0x02F6        
    AIT_REG_W   DSPY_PIP_Y_FIFO_CTL;                                    // 0x02F8
    AIT_REG_W   DSPY_PIP_U_FIFO_CTL;                                    // 0x02FA
    AIT_REG_W   DSPY_PIP_V_FIFO_CTL;                                    // 0x02FC 
    AIT_REG_B                           _x02FE[0x2];                        
    
    AIT_REG_B                           _x0300[0x80];              		// 0x0300
    
    AIT_REG_W   DSPY_RGB1_LINE_CPU_INT_1;                               // 0x0380
    AIT_REG_W   DSPY_RGB1_LINE_CPU_INT_2;                               // 0x0382
    AIT_REG_W   DSPY_RGB1_LINE_HOST_INT_1;                              // 0x0384
    AIT_REG_W   DSPY_RGB1_LINE_HOST_INT_2;                              // 0x0386
    AIT_REG_B   DSPY_INT_HOST_EN_2;                                     // 0x0388
    AIT_REG_B   DSPY_INT_CPU_EN_2;                                      // 0x0389
    AIT_REG_B   DSPY_INT_HOST_SR_2;                                     // 0x038A
    AIT_REG_B   DSPY_INT_CPU_SR_2;                                      // 0x038B
        /*-DEFINE-----------------------------------------------------*/
        #define LCD_FLM_INT					0x10
        #define LCD_WRITE_BACK_DONE			0x08
        #define RGB_LINE_INT2    			0x04
        #define RGB_LINE_INT1      			0x02
        #define HSYNC_ACTIVE	 			0x01
        /*------------------------------------------------------------*/ 
    AIT_REG_B                           _x038C[0x4];
                       
    AIT_REG_B                           _x0390[0x10];

	AIT_REG_B	DSPY_RGB_CTL;											// 0x03A0
        /*-DEFINE-----------------------------------------------------*/
        #define DDRCLK_POLAR_NORMAL   		0x00
        #define DDRCLK_POLAR_INVERT			0x80
        #define DOTCLK_NORMAL_MODE      	0x00
        #define DOTCLK_DDR_MODE         	0x40
        #define HSYNC_POLAR_LOW	       		0x00
        #define HSYNC_POLAR_HIGH        	0x20
        #define VSYNC_POLAR_LOW         	0x00
        #define VSYNC_POLAR_HIGH        	0x10
        #define DOT_POLAR_PST           	0x00
        #define DOT_POLAR_NEG           	0x08
		#define	DEN_DATA_MODE				0x00
		#define	PRT_DATA_MODE				0x04
        #define PARTIAL_MODE_EN				0x02
        #define RGB_IF_EN		        	0x01
        /*------------------------------------------------------------*/
	AIT_REG_B	DSPY_RGB_SPI_CTL;										// 0x03A1	
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_START_ID            	0x01
        /*------------------------------------------------------------*/
	AIT_REG_B	DSPY_RGB_FMT;											// 0x03A2	
        /*-DEFINE-----------------------------------------------------*/
        #define YUV422_D8BIT_Y1VY0U         0x27
        #define YUV422_D8BIT_Y1UY0V         0x26
        #define YUV422_D8BIT_Y0VY1U         0x25
        #define YUV422_D8BIT_Y0UY1V         0x24
        #define YUV422_D8BIT_VY1UY0         0x23
        #define YUV422_D8BIT_UY1VY0         0x22
        #define YUV422_D8BIT_VY0UY1         0x21
        #define YUV422_D8BIT_UY0VY1         0x20
        
        #define RGB_D24BIT_BGR332			0x0D
        #define RGB_D24BIT_BGR333			0x0C
        #define RGB_D24BIT_BGR444			0x0B
        #define RGB_D24BIT_BGR565			0x0A
        #define RGB_D24BIT_BGR666			0x09
        #define RGB_D24BIT_BGR888			0x08
        #define RGB_D24BIT_RGB332			0x05
        #define RGB_D24BIT_RGB333			0x04
        #define RGB_D24BIT_RGB444			0x03
        #define RGB_D24BIT_RGB565			0x02
        #define RGB_D24BIT_RGB666			0x01
        #define RGB_D24BIT_RGB888			0x00

        #define DSPY_RGB_SYNC_MODE_EN 	    0x00
        #define DSPY_RGB_SYNC_MODE_DIS      0x80
        #define DSPY_RGB_SYNC_MODE_MASK		0x80
        /*------------------------------------------------------------*/
    AIT_REG_B   DSPY_RGB_SHARE_P_LCD_BUS;                 				// 0x03A3
        /*-DEFINE-----------------------------------------------------*/
        #define P_LCD_ONLY                  0x00
        #define RGB_18BIT_SHARE_WITH_P_LCD  0x01
        #define RGB_24BIT_SHARE_WITH_P_LCD  0x02
        #define RGB_LCD_ONLY                0x03
        #define DISP_SHARE_BUS_MASK			0x03
        #define RGBLCD_SRC_SEL_RGB       	0x00
        #define RGBLCD_SRC_SEL_RGB2        	0x04
        #define RGBLCD_SRC_SEL_MASK			0x04
        #define HDMI_SRC_SEL_RGB          	0x00
        #define HDMI_SRC_SEL_RGB2          	0x08
        #define HDMI_SRC_SEL_MASK          	0x08
        #define RGB_UDF_FLAG				0x80
        /*------------------------------------------------------------*/
    AIT_REG_B	DSPY_RGB_DOT_CLK_RATIO;									// 0x03A4	
    AIT_REG_B   DSPY_RGB_PORCH_HIGH_BIT_EXT;                      		// 0x03A5
    AIT_REG_B	DSPY_RGB_V_BPORCH;										// 0x03A6	
    AIT_REG_B	DSPY_RGB_V_BLANK;										// 0x03A7	
    AIT_REG_B	DSPY_RGB_H_BPORCH;										// 0x03A8	
    AIT_REG_B	DSPY_RGB_H_BLANK;										// 0x03A9	
    AIT_REG_B	DSPY_RGB_HSYNC_W;										// 0x03AA	
    AIT_REG_B	DSPY_RGB_VSYNC_W;										// 0x03AB	
    AIT_REG_B	DSPY_RGB_V_2_H_DOT;	//[Reserved]						// 0x03AC	
    AIT_REG_B                           _x03AD;
    AIT_REG_B	DSPY_RGB_PRT_2_H_DOT;									// 0x03AE	
    AIT_REG_B   DSPY_RGB_INTERLACE_EN;                            	 	// 0x03AF
        /*-DEFINE-----------------------------------------------------*/
        #define RGB_INTERLACE_DIS      		0x00
        #define RGB_INTERLACE_EN       		0x01
        #define RGB_TOTAL_ODD               0x00
        #define RGB_TOTAL_EVEN              0x02
        /*------------------------------------------------------------*/
        
    AIT_REG_W	DSPY_RGB_PART_ST_Y;  									// 0x03B0	
    AIT_REG_W	DSPY_RGB_PART_ED_Y;	    							    // 0x03B2	
    AIT_REG_W	DSPY_RGB_PART_ST_X;  									// 0x03B4	
    AIT_REG_W	DSPY_RGB_PART_ED_X;									    // 0x03B6
    AIT_REG_B	DSPY_RGB_RATIO_SPI_MCI;                                 // 0x03B8
    AIT_REG_B	                        _x03B9;
    AIT_REG_B	DSPY_RGB_SPI_CS_SETUP_CYCLE;                            // 0x03BA
    AIT_REG_B	DSPY_RGB_SPI_CS_HOLD_CYCLE;                             // 0x03BB
    AIT_REG_B	DSPY_RGB_SPI_CS_HIGH_WIDTH;                             // 0x03BC
    AIT_REG_B	                        _x03BD;
    AIT_REG_B	DSPY_RGB_SPI_CTRL_REGISTER1;                         	// 0x03BE
        /*-DEFINE-----------------------------------------------------*/
        #define RGB_SPI_DATA_ONLY_MODE  	0x01
        /*------------------------------------------------------------*/        
    AIT_REG_B	DSPY_RGB_DELTA_MODE;                                    // 0x03BF
        /*-DEFINE-----------------------------------------------------*/
        #define RGB_DELTA_MODE_ENABLE   	0x01
        #define RGB_DUMMY_MODE_ENABLE   	0x02
        
        #define SPI_ODD_LINE_RGB        	0x00
        #define SPI_ODD_LINE_RBG        	0x04
        #define SPI_ODD_LINE_GRB        	0x08
        #define SPI_ODD_LINE_GBR        	0x0C
        #define SPI_ODD_LINE_BRG        	0x10
        #define SPI_ODD_LINE_BGR        	0x14
        
        #define SPI_EVEN_LINE_RGB       	0x00
        #define SPI_EVEN_LINE_RBG       	0x20
        #define SPI_EVEN_LINE_GRB       	0x40
        #define SPI_EVEN_LINE_GBR       	0x60
        #define SPI_EVEN_LINE_BRG       	0x80
        #define SPI_EVEN_LINE_BGR       	0xA0
        /*------------------------------------------------------------*/
                
    AIT_REG_B	DSPY_SPI_CONTROL_REGISTER1;                             // 0x03C0
        /*-DEFINE-----------------------------------------------------*/
        #define SPI_POLARITY_POSITIVE_EDGE  0x00
        #define SPI_POLARITY_NEGATIVE_EDGE  0x02
        #define SPI_RW_HIGH_READ        	0x00
        #define SPI_RW_HIGH_WRITE       	0x04
        #define SPI_RS_HIGH_COMMAND     	0x00
        #define SPI_RS_HIGH_INDEX       	0x08
        #define SPI_CS_ACTIVE_LOW       	0x00
        #define SPI_CS_ACTIVE_HIGH      	0x10
        #define SPI_PANEL_8BITS         	0x00
        #define SPI_PANEL_9BITS         	0x20
        #define SPI_PANEL_12BITS        	0x40
        #define SPI_PANEL_16BITS        	0x60
        #define SPI_PANEL_18BITS        	0x80
        #define SPI_PANEL_24BITS        	0xA0
        /*------------------------------------------------------------*/        
    AIT_REG_B	                        _x03C1;
    AIT_REG_B	DSPY_RATIO_MCI_SPI;									    // 0x03C2
    AIT_REG_B	                        _x03C3;
    AIT_REG_B	DSPY_SPI_CS_SETUP_CYCLE;                                // 0x03C4
    AIT_REG_B	DSPY_SPI_CS_HOLD_CYCLE;                                 // 0x03C5
    AIT_REG_B	DSPY_SPI_CS_HIGH_WIDTH;                                 // 0x03C6
    AIT_REG_B	                        _x03C7; 
    AIT_REG_B   DSPY_RGB_FRAM_CNT_CPU_INT;                              // 0x03C8
    AIT_REG_B                           _x03C9;
    AIT_REG_B   DSPY_RGB_FRAM_CNT_HOST_INT;                             // 0x03CA
    AIT_REG_B                           _x03CB[0x5];
    
    AIT_REG_B   DSPY_WBACK_CTL;                             			// 0x03D0
        /*-DEFINE-----------------------------------------------------*/
        #define WBACK_ONLY_MODE	        	0x10
        #define WBACK_FMT_BGR		    	0x08
        #define WBACK_FMT_RGB		    	0x00
        #define WBACK_FMT_888		    	0x04
        #define WBACK_FMT_565		    	0x02
        #define WBACK_EN			    	0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x03D1[0x3];
    AIT_REG_D   DSPY_WBACK_ADDR;                             			// 0x03D4
    AIT_REG_W   DSPY_WBACK_W;                             				// 0x03D8
    AIT_REG_W   DSPY_WBACK_H;                             				// 0x03DA
    AIT_REG_W   DSPY_WBACK_X;                             				// 0x03DC
    AIT_REG_W   DSPY_WBACK_Y;											// 0x03DE
    
    AIT_REG_B                           _x03E0[0x20];
    
    AIT_REG_D   DSPY_OSD_ADDR_ST;                               		// 0x0400
    AIT_REG_B                           _x0404[0x4];
    AIT_REG_D   DSPY_OSD_OFST_ST;                                 		// 0x0408
    AIT_REG_B                           _x040C[0x4];

    AIT_REG_W   DSPY_OSD_OFST_PIXL;                                		// 0x0410
    AIT_REG_B                           _x0412[0x2];
    AIT_REG_W   DSPY_OSD_OFST_ROW;										// 0x0414
    AIT_REG_B                           _x0416[0xA];

    AIT_REG_D   DSPY_OSD_U_ADDR_ST;                                    	// 0x0420
    AIT_REG_B                           _x0424[0x4];
    AIT_REG_D   DSPY_OSD_OFST_UV_ST;									// 0x0428
    AIT_REG_B                           _x042C[0x4];
    
    AIT_REG_W   DSPY_OSD_OFST_UV_PIXL;                             		// 0x0430
    AIT_REG_B                           _x0432[0x2];
    AIT_REG_W   DSPY_OSD_OFST_UV_ROW;									// 0x0434
    AIT_REG_B                           _x0436[0xA];
 
    AIT_REG_D   DSPY_OSD_V_ADDR_ST;                                    	// 0x0440
    AIT_REG_B                           _x0444[0x4];
	AIT_REG_B	DSPY_OSD_DBG_OBS_SEL_MP;								// 0x0448			    
    AIT_REG_B	DSPY_OSD_FIFO_MAX_CTL_MP;								// 0x0449			
    AIT_REG_B                           _x044A[0x2];    
    AIT_REG_B   DSPY_OSD_DBG_SEL_BYTE_MP[4];							// 0x044C

    AIT_REG_W   DSPY_OSD_W;                                          	// 0x0450
    AIT_REG_B                           _x0452[0x2];
    AIT_REG_W   DSPY_OSD_H;												// 0x0454
    AIT_REG_B                           _x0456[0x2];
    AIT_REG_W   DSPY_OSD_X;												// 0x0458
    AIT_REG_B                           _x045A[0x2];
    AIT_REG_W   DSPY_OSD_Y;												// 0x045C
    AIT_REG_B                           _x045E[0x2];

    AIT_REG_W   DSPY_OSD_CTL;                                      		// 0x0460
    AIT_REG_W   DSPY_OSD_CTL_2;                                     	// 0x0462
    AIT_REG_W   DSPY_OSD_FMT;											// 0x0464
    AIT_REG_B                           _x0466[0x2];
    AIT_REG_D   DSPY_OSD_PIXL_CNT;										// 0x0468
    AIT_REG_B                           _x046C[0x4];

    AIT_REG_W   DSPY_OSD_TP_CTL;                                   		// 0x0470
    AIT_REG_W   DSPY_OSD_SEMITP_WT;										// 0x0472
	AIT_DSPY_TP DSPY_OSD_TP;											// 0x0474
	AIT_REG_W   DSPY_OSD_WIDTH_DUP_DIV_MP;								// 0x047A
	AIT_REG_D   DSPY_OSD_PIXL_CNT_DUP_DIV_MP;							// 0x047C

    AIT_REG_W   DSPY_RGB2_LINE_CPU_INT_1;                             	// 0x0480
    AIT_REG_W   DSPY_RGB2_LINE_CPU_INT_2;                               // 0x0482
    AIT_REG_W   DSPY_RGB2_LINE_HOST_INT_1;                              // 0x0484
    AIT_REG_W   DSPY_RGB2_LINE_HOST_INT_2;                              // 0x0486
    AIT_REG_B   DSPY_INT_HOST_EN_4;                                     // 0x0488
    AIT_REG_B   DSPY_INT_CPU_EN_4;                                      // 0x0489
    AIT_REG_B   DSPY_INT_HOST_SR_4;                                     // 0x048A
    AIT_REG_B   DSPY_INT_CPU_SR_4;                                      // 0x048B
        /*-DEFINE-----------------------------------------------------*/
        #define RGB2_VSYNC_ACTIVE       	0x08
        #define RGB2_LINE_INT2        		0x04
        #define RGB2_LINE_INT1        		0x02
        #define RGB2_HSYNC_ACTIVE	 		0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x048C[0x4];
    
    AIT_REG_B   DSPY_OSD_UV_GAIN_11;                                	// 0x0490
    AIT_REG_B   DSPY_OSD_UV_GAIN_12;									// 0x0491
    AIT_REG_B   DSPY_OSD_UV_GAIN_21;									// 0x0492
    AIT_REG_B   DSPY_OSD_UV_GAIN_22;									// 0x0493
    AIT_REG_W   DSPY_OSD_RGB_GAIN;										// 0x0494
    AIT_REG_W   DSPY_OSD_RGB_OFST;										// 0x0496
    AIT_REG_B                           _x0498[0x8];
    
    AIT_REG_B	DSPY_RGB2_CTL;											// 0x04A0
	AIT_REG_B                           _x04A1;	
	AIT_REG_B	DSPY_RGB2_FMT;											// 0x04A2	
    AIT_REG_B   DSPY_RGB2_SHARE_P_LCD_BUS;                              // 0x04A3
	AIT_REG_B	DSPY_RGB2_DOT_CLK_RATIO;								// 0x04A4	
    AIT_REG_B   DSPY_RGB2_PORCH_HIGH_BIT_EXT;                           // 0x04A5
	AIT_REG_B	DSPY_RGB2_V_BPORCH;										// 0x04A6	
	AIT_REG_B	DSPY_RGB2_V_BLANK;										// 0x04A7	
	AIT_REG_B	DSPY_RGB2_H_BPORCH;										// 0x04A8	
	AIT_REG_B	DSPY_RGB2_H_BLANK;										// 0x04A9	
	AIT_REG_B	DSPY_RGB2_HSYNC_W;										// 0x04AA	
	AIT_REG_B	DSPY_RGB2_VSYNC_W;										// 0x04AB	
    AIT_REG_B                           _x04AC[0x2];
	AIT_REG_B	DSPY_RGB2_PRT_2_H_DOT;									// 0x04AE	
    AIT_REG_B   DSPY_RGB2_INTERLACE_EN;									// 0x04AF
    
	AIT_REG_W	DSPY_RGB2_PART_ST_Y;  									// 0x04B0	
	AIT_REG_W	DSPY_RGB2_PART_ED_Y;	    							// 0x04B2	
	AIT_REG_W	DSPY_RGB2_PART_ST_X;  									// 0x04B4	
	AIT_REG_W	DSPY_RGB2_PART_ED_X;									// 0x04B6
    AIT_REG_B	                        _x04B8[0x7];
    AIT_REG_B	DSPY_RGB2_DELTA_MODE;                                   // 0x04BF
    
    AIT_REG_B                           _x04C0[0x8];
    AIT_REG_B   DSPY_RGB2_FRAM_CNT_CPU_INT;                             // 0x04C8
    AIT_REG_B                           _x04C9;
    AIT_REG_B   DSPY_RGB2_FRAM_CNT_HOST_INT;                            // 0x04CA
    AIT_REG_B                           _x04CB[0x5];
    
    AIT_REG_B                           _x04D0[0x10];    
      
    AIT_REG_D   DSPY_OSD_TV_EVFIELD_ST;                                	// 0x04E0
    AIT_REG_B                           _x03E4[0xC];
    
    AIT_REG_B                           _x03F0[0x8];
    
    AIT_REG_W   DSPY_OSD_Y_FIFO_CTL;                                    // 0x04F8
    AIT_REG_W   DSPY_OSD_U_FIFO_CTL;                                    // 0x04FA
    AIT_REG_W   DSPY_OSD_V_FIFO_CTL;                                    // 0x04FC 
    AIT_REG_B                           _x04FE[0x2]; 
    
    AIT_REG_D   DSPY_OVLY_ADDR_ST;                                    	// 0x0500
    AIT_REG_B                           _x0504[0x4]; 
    AIT_REG_D   DSPY_OVLY_OFST_ST;										// 0x0508
    AIT_REG_B                           _x050C[0x4]; 

    AIT_REG_W   DSPY_OVLY_OFST_PIXL;                                    // 0x0510
    AIT_REG_B                           _x0512[0x2];
    AIT_REG_W   DSPY_OVLY_OFST_ROW;										// 0x0514
    AIT_REG_B                           _x0516[0xA];

    AIT_REG_D   DSPY_OVLY_U_ADDR_ST;                                  	// 0x0520
    AIT_REG_B                           _x0524[0x4]; 
    AIT_REG_D   DSPY_OVLY_OFST_UV_ST;									// 0x0528
    AIT_REG_B   						_x052C[0x4];

    AIT_REG_W   DSPY_OVLY_OFST_UV_PIXL;                                 // 0x0530
    AIT_REG_B                           _x0532[0x2];
    AIT_REG_W   DSPY_OVLY_OFST_UV_ROW;									// 0x0534
    AIT_REG_B                           _x0536[0xA];

    AIT_REG_D   DSPY_OVLY_V_ADDR_ST;                                  	// 0x0540                                  
    AIT_REG_B                           _x0544[0x4];                                 
	AIT_REG_B	DSPY_OVLY_DBG_OBS_SEL_MP;								// 0x0548
    AIT_REG_B	DSPY_OVLY_FIFO_MAX_CTL_MP;								// 0x0549
    AIT_REG_B                           _x054A[0x2];    
    AIT_REG_B   DSPY_OVLY_DBG_SEL_BYTE_MP[4];							// 0x054C

    AIT_REG_W   DSPY_OVLY_W;                                            // 0x0550
    AIT_REG_B                           _x0552[0x2];
    AIT_REG_W   DSPY_OVLY_H;											// 0x0554
    AIT_REG_B                           _x0556[0x2];
    AIT_REG_W   DSPY_OVLY_X;											// 0x0558
    AIT_REG_B                           _x055A[0x2];
    AIT_REG_W   DSPY_OVLY_Y;											// 0x055C
    AIT_REG_B                           _x055E[0x2];

    AIT_REG_W   DSPY_OVLY_CTL;                                          // 0x0560
    AIT_REG_W   DSPY_OVLY_CTL_2;                                        // 0x0562
    AIT_REG_W   DSPY_OVLY_FMT;                                          // 0x0564
    AIT_REG_B                           _x0566[0x2];
    AIT_REG_D   DSPY_OVLY_PIXL_CNT;                                     // 0x0568
    AIT_REG_D   DSPY_OVLY_SCA_OUT_PIXL_CNT;                             // 0x056C
    
    AIT_REG_W   DSPY_OVLY_TP_CTL;                                       // 0x0570
    AIT_REG_W   DSPY_OVLY_SEMITP_WT;									// 0x0572
	AIT_DSPY_TP DSPY_OVLY_TP;											// 0x0574
	AIT_REG_W   DSPY_OVLY_WIDTH_DUP_DIV_MP;								// 0x057A
	AIT_REG_D   DSPY_OVLY_PIXL_CNT_DUP_DIV_MP;							// 0x057C

    AIT_REG_W   DSPY_OVLY_SCAL_CTL;                                     // 0x0580
    AIT_REG_W   DSPY_OVLY_SOUT_CTL;                                     // 0x0582
    AIT_REG_W   DSPY_OVLY_SCAL_H_N;                                     // 0x0584 
    AIT_REG_W   DSPY_OVLY_SCAL_H_M;                                     // 0x0586
    AIT_REG_W   DSPY_OVLY_SCAL_V_N;                                     // 0x0588
    AIT_REG_W   DSPY_OVLY_SCAL_V_M;                                     // 0x058A
    AIT_REG_W   DSPY_OVLY_SEDGE_CTL;                                    // 0x058C
    AIT_REG_B   DSPY_OVLY_SEDGE_GAIN_VAL;                               // 0x058E
    AIT_REG_B   DSPY_OVLY_SEDGE_CORE_VAL;                               // 0x058F

    AIT_REG_B   DSPY_OVLY_UV_GAIN_11;                                   // 0x0590
    AIT_REG_B   DSPY_OVLY_UV_GAIN_12;                                   // 0x0591
    AIT_REG_B   DSPY_OVLY_UV_GAIN_21;                                   // 0x0592
    AIT_REG_B   DSPY_OVLY_UV_GAIN_22;                                   // 0x0593
    AIT_REG_W   DSPY_OVLY_RGB_GAIN;                                     // 0x0594
    AIT_REG_W   DSPY_OVLY_RGB_OFST;                                     // 0x0596
    AIT_REG_W   DSPY_OVLY_SCAL_H_WT;                                    // 0x0598
    AIT_REG_W   DSPY_OVLY_SCAL_V_WT;                                    // 0x059A
    AIT_REG_W   DSPY_OVLY_SCAL_WT_FORCE1_EN;                     		// 0x059C
    AIT_REG_B                           _x059E[0x2];

    AIT_REG_W   DSPY_OVLY_SCA_IN_W;                                     // 0x05A0
    AIT_REG_B                           _x05A2[0x2];
    AIT_REG_W   DSPY_OVLY_SCA_IN_H;										// 0x05A4
    AIT_REG_B                           _x05A6[0xA];

    AIT_REG_W   DSPY_OVLY_SOUT_GRAB_H_ST;                               // 0x05B0
    AIT_REG_B                           _x05B2[0x2];
    AIT_REG_W   DSPY_OVLY_SOUT_GRAB_H_ED;								// 0x05B4
    AIT_REG_B                           _x05B6[0x2];
    AIT_REG_W   DSPY_OVLY_SOUT_GRAB_V_ST;								// 0x05B8
    AIT_REG_B                           _x05BA[0x2];
    AIT_REG_W   DSPY_OVLY_SOUT_GRAB_V_ED;								// 0x05BC
    AIT_REG_B                           _x05BE[0x2];
    
    AIT_REG_B                           _x05C0[0x20];
    
    AIT_REG_D   DSPY_OVLY_TV_EVEN_FIELD_ST;                         	// 0x05E0
    AIT_REG_B                           _x05E4[0xC];
    
    AIT_REG_B   DSPY_OVLY_BUF_FULL_THD;                                 // 0x05F0
    AIT_REG_B   DSPY_OVLY_BUF_STOP_THD;                                 // 0x05F1
    AIT_REG_B                           _x05F2[0x2]; 
    AIT_REG_W   DSPY_OVLY_PRM_BUF_FULL_THD;                       		// 0x05F4                    
    AIT_REG_W   DSPY_OVLY_SCD_BUF_FULL_THD;                   			// 0x05F6     
    AIT_REG_W   DSPY_OVLY_Y_FIFO_CTL;                                   // 0x05F8
    AIT_REG_W   DSPY_OVLY_U_FIFO_CTL;                                   // 0x05FA
    AIT_REG_W   DSPY_OVLY_V_FIFO_CTL;                                   // 0x05FC 
    AIT_REG_B                           _x05FE[0x2];
} AITS_DSPY, *AITPS_DSPY;

//-----------------------------
// TV structure (0x8000 2870)
//-----------------------------
typedef struct _AITS_TV {
    AIT_REG_D TVIF_TEST_1ST_Y2CrY1Cb;                                   // 0x0070
    AIT_REG_D TVIF_TEST_2ND_Y2CrY1Cb;                                   // 0x0074
    AIT_REG_B TVIF_DAC_IF_1ST_CTL;                                      // 0x0078
        /*-DEFINE-----------------------------------*/
        #define TV_DAC_POWER_DOWN_EN        0x08	
        #define TV_BGREF_POWER_DOWN_EN      0x04	
        #define TV_IQUARTER                 0x02	
        #define TV_OTYPE                    0x01	  
        /*------------------------------------------*/
    AIT_REG_B TVIF_DAC_IF_2ND_CTL;                                      // 0x0079
        /*-DEFINE-----------------------------------*/
        #define TV_VPLUGREF                 0x10	
        #define TV_COMP_LEVEL               0x08	
        #define TV_HYS_ON                   0x04	
        #define TV_TEST_COMP                0x02	
        #define TV_PLUG_DECT                0x01	        
        /*------------------------------------------*/
    AIT_REG_B TVIF_DAC_IF_3RD_CTL;                                      // 0x007A
        /*-DEFINE-----------------------------------*/
        #define TV_DAC_CLOCK_DATA_EXT       0x04
        /*------------------------------------------*/
    AIT_REG_B                               _x007B;
    AIT_REG_B TVIF_BACKGROUND_Y_COLOR;                                  // 0x007C
    AIT_REG_B TVIF_BACKGROUND_U_COLOR;                                  // 0x007D
    AIT_REG_B TVIF_BACKGROUND_V_COLOR;                                  // 0x007E    
    AIT_REG_B TVIF_CLK_DELAY_V1;                                        // 0x007F
        /*-DEFINE-----------------------------------*/
        #define NO_DELAY                    0x00
        #define DELAY_1T                    0x01
        #define DELAY_2T                    0x02
        #define DELAY_3T                    0x03
        /*------------------------------------------*/
    AIT_REG_B TVIF_IF_EN;                                               // 0x0080
        /*-DEFINE-----------------------------------*/
        #define TV_ENC_TEST_MODE_EN         0x80
        #define TV_IF_DAC_CTL               0x40
        #define TV_TYPE_NTSC                0x00
        #define TV_TYPE_PAL                 0x20
        #define TV_UV_SEL_HALF_SUM          0x00
        #define TV_UV_SEL_U1V1              0x08
        #define TV_UV_SEL_U2V2              0x10
        #define TV_8MHZ_FPGA_TEST           0x04
        #define TV_DISPLAY_SPECIFIED_IMAGE  0x02
        #define TV_ENC_IF_EN                0x01
        /*------------------------------------------*/
    AIT_REG_B TVIF_ENDLINE_OFFSET_CTL;                                  // 0x0081
    AIT_REG_B TVIF_EARLY_PIXL;		                                    // 0x0082
    AIT_REG_B TVIF_1ST_PXL_RQST_TIMING;                                 // 0x0083
    AIT_REG_W TVIF_NTSC_ODFIELD_LINE;     	                            // 0x0084
    AIT_REG_W TVIF_NTSC_EVFIELD_LINE;                                   // 0x0086
    AIT_REG_W TVIF_PAL_1ST_FIELD_LINE;                                  // 0x0088
    AIT_REG_W TVIF_PAL_2ND_FIELD_LINE;                                  // 0x008A
    AIT_REG_W TVIF_NTSC_EVLINE_SUB1;                                    // 0x008C
    AIT_REG_W TVIF_PAL_EVLINE_SUB1;		                                // 0x008E
    AIT_REG_W TVIF_INT1_CPU;                                            // 0x0090
    AIT_REG_W TVIF_INT2_CPU;                                            // 0x0092
    AIT_REG_W TVIF_INT1_HOST;                                           // 0x0094
    AIT_REG_W TVIF_INT2_HOST;                                           // 0x0096
    AIT_REG_W TVIF_IMAGE_WIDTH;                                         // 0x0098
    AIT_REG_W TVIF_IMAGE_HEIGHT;                                        // 0x009A
    AIT_REG_W TVIF_IMAGE_START_X;                                       // 0x009C
    AIT_REG_W TVIF_IMAGE_START_Y;                                       // 0x009E
    AIT_REG_D TVENC_SYNC_CTL;                                           // 0x00A0
        /*-DEFINE-----------------------------------*/
        #define TV_ENC_SYNC_SW_RST          0xC0000000
        #define TV_UV_SWAPPING_EN           0x00001000
        /*------------------------------------------*/
    AIT_REG_D TVENC_MODE_CTL;                                           // 0x00A4
        /*-DEFINE-----------------------------------*/
        #define TV_714MV_286MV_MODE         0x80000000
        #define TV_BLACKER_LEVEL_EN         0x40000000
        #define TV_COLOR_BAR_TYPE           0x10000000
        #define TV_FULL_WIDTH_OUTPUT_EN     0x08000000
        #define TV_SLEW_RATE_CTL_DIS        0x02000000
        #define TV_MIX_SUB_VIDEO_EN         0x01000000
        #define TV_SVIDEO_CVBS_EN           0x00040000
        #define TV_OUTPUT_CVBS_MODE         0x00020000
        #define TV_OUTPUT_SVIDEO_MODE       0x00010000
        #define TV_CHROMA_UPSAMPLE_EN       0x00004000
        #define TV_DELAY_INPUT_Y_HALF_PIX_1 0x00000000
        #define TV_DELAY_INPUT_Y_HALF_PIX_2 0x00000400
        #define TV_DELAY_INPUT_Y_HALF_PIX_3 0x00000800
        #define TV_DELAY_INPUT_Y_HALF_PIX_4 0x00000C00
        #define TV_DELAY_INPUT_Y_ONE_PIX_1  0x00000000
        #define TV_DELAY_INPUT_Y_ONE_PIX_2  0x00000100
        #define TV_DELAY_INPUT_Y_ONE_PIX_3  0x00000200
        #define TV_DELAY_INPUT_Y_ONE_PIX_4  0x00000300
        #define TV_LUMA_LPF_EN              0x00000080
        #define TV_UV_SWAPPING_SUB_VIDEO    0x00000040
        #define TV_CHROMA_LPF_EN            0x00000020
        #define TV_SETUP_751RE_EN           0x00000004
        #define TV_COLOR_BAR_EN             0x00000002
        #define TV_ENCODER_EN               0x00000001
        /*------------------------------------------*/
    AIT_REG_D TVENC_CLOSED_CAPTION;                                     // 0x00A8
        /*-DEFINE-----------------------------------*/
        #define TV_CLOSED_CAP_LINE_21_22    0x00010000
        #define TV_CLOSED_CAP_LINE_284_335  0x00020000
        /*------------------------------------------*/
    AIT_REG_D TVENC_Y_SCALE_CTL;                                        // 0x00AC
        /*-DEFINE-----------------------------------*/
        #define TV_SUB_VIDEO_DELAY_SEL_1T   0xC0000000
        #define TV_SUB_VIDEO_DELAY_SEL_2T   0x80000000
        #define TV_SUB_VIDEO_DELAY_SEL_3T   0x40000000
        #define TV_SUB_VIDEO_DELAY_SEL_4T   0x00000000
        #define TV_SUB_PIC_DELAY_SEL_1T     0x30000000
        #define TV_SUB_PIC_DELAY_SEL_2T     0x20000000
        #define TV_SUB_PIC_DELAY_SEL_3T     0x10000000
        #define TV_SUB_PIC_DELAY_SEL_4T     0x00000000
        #define TV_OSD_DELAY_SEL_1T         0x0C000000
        #define TV_OSD_DELAY_SEL_2T         0x08000000
        #define TV_OSD_DELAY_SEL_3T         0x04000000
        #define TV_OSD_DELAY_SEL_4T         0x00000000
        /*------------------------------------------*/
    AIT_REG_D TVENC_U_SCALE_CTL;                                        // 0x00B0
    AIT_REG_D TVENC_V_SCALE_CTL;                                        // 0x00B4
    AIT_REG_D TVENC_GAMMA_COEF_0;                                       // 0x00B8
        /*-DEFINE-----------------------------------*/
        #define TV_ACTIVE_VBI_EN            0x00400000
        /*------------------------------------------*/
    AIT_REG_D TVENC_GAMMA_COEF_1_2;                                     // 0x00BC
    AIT_REG_D TVENC_GAMMA_COEF_3_4;                                     // 0x00C0
    AIT_REG_D TVENC_GAMMA_COEF_5_6;                                     // 0x00C4
    AIT_REG_D TVENC_GAMMA_COEF_7_8;                                     // 0x00C8
    AIT_REG_D TVENC_DAC_CONFIG;                                         // 0x00CC
        /*-DEFINE-----------------------------------*/
        #define TV_VREF_OUTPUT_DIS          0x00008000
        #define TV_CLOCK_DAC_NEGATIVE_EDGE  0x00004000
        #define TV_TRIM_MODE_EN             0x00002000
        #define TV_PLUG_DETECT_EN           0x00001000
        #define TV_DAS_Y_OUTPUT_OFF         0x00000000
        #define TV_DAS_Y_AUTO_DETECT        0x00000040
        #define TV_DAS_Y_OUTPUT_ON          0x00000080
        #define TV_DAX_C_OUTPUT_OFF         0x00000000
        #define TV_DAX_C_AUTO_DETECT        0x00000010
        #define TV_DAX_C_OUTPUT_ON          0x00000020
        /*------------------------------------------*/
    AIT_REG_D TVENC_COLOR_BURST_CONFIG;                                 // 0x00D0
        /*-DEFINE-----------------------------------*/
        #define TV_PAL_MODE_BURST_SEL_STR   0x04000000
        #define TV_UV_EXTRA_GAIN_EN         0x02000000
        #define TV_PAL_MODE_BURST_SEL_END   0x01000000
        #define TV_FORCE_PAL60_NTSC443      0x00008000
        /*------------------------------------------*/
    AIT_REG_D                               _x00D4;
    AIT_REG_D TVENC_WSS_IF_MODE;                                        // 0x00D8
        /*-DEFINE-----------------------------------*/
        #define TV_WSS_IF_MODE_EN           0x00100000
        /*------------------------------------------*/
    AIT_REG_D TVENC_UV_SCALE_GAIN_4_5;                                  // 0x00DC
    AIT_REG_D TVENC_Y_LPF_COEF_00_03;                                   // 0x00E0
    AIT_REG_D TVENC_Y_LPF_COEF_04_07;                                   // 0x00E4
    AIT_REG_D TVENC_Y_LPF_COEF_08_0B;                                   // 0x00E8
    AIT_REG_D TVENC_Y_LPF_COEF_0C_0F;                                   // 0x00EC
    AIT_REG_D TVENC_Y_LPF_COEF_10_13;                                   // 0x00F0
    AIT_REG_D TVENC_C1_LPF_COEF_00_03;                                  // 0x00F4
    AIT_REG_D TVENC_C1_LPF_COEF_04;                                     // 0x00F8
    AIT_REG_D                               _x00FC;
} AITS_TV, *AITPS_TV;

#if !defined(BUILD_FW)

// LCD  OPR
#define DSPY_CTL_0                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_0              )))
#define DSPY_CTL_2	                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_2              )))
#define DSPY_LCD_TX_6               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_LCD_TX_6           )))

#define DSPY_W                      (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_W                  )))
#define DSPY_H                      (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_H                  )))
#define DSPY_PIXL_CNT               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIXL_CNT           )))
#define DSPY_CTL_4                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_CTL_4              )))
#define DSPY_BG_COLOR               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_BG_COLOR           )))

#define DSPY_PIP_OFST_ROW           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_OFST_ROW       )))
#define DSPY_PIP_SOUT_CTL           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_SOUT_CTL       )))
#define DSPY_PIP_TV_EVEN_FIELD_ST   (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_PIP_TV_EVEN_FIELD_ST)))
#define DSPY_WIN_BIND_SEL_MP		(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_WIN_BIND_SEL_MP)))

// SCD OPR
#define DSPY_SCD_CTL                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_CTL            )))
#define DSPY_SCD_BG_COLOR           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_BG_COLOR       )))
#define DSPY_SCD_W                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_W              )))
#define DSPY_SCD_H                  (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_H              )))
#define DSPY_SCD_PIXL_CNT           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_SCD_PIXL_CNT       )))

// RGB OPR
#define DSPY_RGB_CTL                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_CTL            )))
#define DSPY_RGB_FMT                (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_FMT            )))
#define DSPY_RGB_SHARE_P_LCD_BUS    (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_SHARE_P_LCD_BUS)))
#define DSPY_RGB_DOT_CLK_RATIO   	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_DOT_CLK_RATIO  )))
#define DSPY_RGB_PORCH_HIGH_BIT_EXT (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PORCH_HIGH_BIT_EXT)))
#define DSPY_RGB_V_BPORCH           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_BPORCH       )))
#define DSPY_RGB_V_BLANK            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_BLANK        )))
#define DSPY_RGB_H_BPORCH           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_H_BPORCH       )))
#define DSPY_RGB_H_BLANK            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_H_BLANK        )))
#define DSPY_RGB_HSYNC_W            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_HSYNC_W        )))
#define DSPY_RGB_VSYNC_W            (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_VSYNC_W        )))
#define DSPY_RGB_V_2_H_DOT          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_V_2_H_DOT      )))
#define DSPY_RGB_PRT_2_H_DOT        (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PRT_2_H_DOT    )))
#define DSPY_RGB_PART_ST_X          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ST_X      )))
#define DSPY_RGB_PART_ST_Y          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ST_Y      )))
#define DSPY_RGB_PART_ED_X          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ED_X      )))
#define DSPY_RGB_PART_ED_Y          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_PART_ED_Y      )))
#define DSPY_RGB1_LINE_CPU_INT_1	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB1_LINE_CPU_INT_1)))
#define DSPY_RGB1_LINE_CPU_INT_2	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB1_LINE_CPU_INT_2)))
#define DSPY_INT_CPU_EN_2           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_INT_CPU_EN_2       )))
#define DSPY_INT_CPU_SR_2           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_INT_CPU_SR_2       )))
#define DSPY_RGB_DELTA_MODE         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB_DELTA_MODE     )))

// RGB2 OPR
#define DSPY_RGB2_LINE_CPU_INT_1    (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_LINE_CPU_INT_1)))
#define DSPY_RGB2_LINE_CPU_INT_2    (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_LINE_CPU_INT_2)))
#define DSPY_RGB2_CTL               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_CTL           )))
#define DSPY_RGB2_FMT               (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_FMT           )))
#define DSPY_RGB2_DOT_CLK_RATIO  	(DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_DOT_CLK_RATIO  )))
#define DSPY_RGB2_PORCH_HIGH_BIT_EXT (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PORCH_HIGH_BIT_EXT)))
#define DSPY_RGB2_V_BPORCH          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_V_BPORCH      )))
#define DSPY_RGB2_V_BLANK           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_V_BLANK       )))
#define DSPY_RGB2_H_BPORCH          (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_H_BPORCH      )))
#define DSPY_RGB2_H_BLANK           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_H_BLANK       )))
#define DSPY_RGB2_HSYNC_W           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_HSYNC_W       )))
#define DSPY_RGB2_VSYNC_W           (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_VSYNC_W       )))
#define DSPY_RGB2_PRT_2_H_DOT       (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PRT_2_H_DOT   )))
#define DSPY_RGB2_PART_ST_Y         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PART_ST_Y     )))
#define DSPY_RGB2_PART_ED_Y         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PART_ED_Y     )))
#define DSPY_RGB2_PART_ST_X         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PART_ST_X     )))
#define DSPY_RGB2_PART_ED_X         (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_PART_ED_X     )))
#define DSPY_RGB2_DELTA_MODE        (DSPY_BASE +(MMP_ULONG)(&(((AITPS_DSPY )0)->DSPY_RGB2_DELTA_MODE    )))

// PALETTE OPR
#define MAIN_PALETTE_R0             0x80010400
#define MAIN_PALETTE_G0             0x80010200
#define MAIN_PALETTE_B0             0x80010000

#define OVERLAY_PALETTE_R0          0x80010C00
#define OVERLAY_PALETTE_G0          0x80010A00
#define OVERLAY_PALETTE_B0          0x80010800

#define OSD_PALETTE_R0              0x80011400
#define OSD_PALETTE_G0              0x80011200
#define OSD_PALETTE_B0              0x80011000

// TV OPR
#define TVIF_DAC_IF_1ST_CTL         (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_DAC_IF_1ST_CTL       ))) 
#define TVIF_BACKGROUND_Y_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_Y_COLOR   ))) 
#define TVIF_BACKGROUND_U_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_U_COLOR   ))) 
#define TVIF_BACKGROUND_V_COLOR     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_BACKGROUND_V_COLOR   ))) 
#define TVIF_CLK_DELAY_V1           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_CLK_DELAY_V1         )))
#define TVIF_IF_EN                  (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IF_EN                )))
#define TVIF_ENDLINE_OFFSET_CTL     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_ENDLINE_OFFSET_CTL   )))
#define TVIF_EARLY_PIXL		        (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_EARLY_PIXL           )))
#define TVIF_1ST_PXL_RQST_TIMING    (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_1ST_PXL_RQST_TIMING  )))
#define TVIF_NTSC_ODFIELD_LINE      (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_ODFIELD_LINE    )))
#define TVIF_NTSC_EVFIELD_LINE      (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_EVFIELD_LINE    )))
#define TVIF_PAL_ODFIELD_LINE       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_ODFIELD_LINE     ))) 
#define TVIF_PAL_EVFIELD_LINE       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_EVFIELD_LINE     ))) 
#define TVIF_NTSC_EVLINE_SUB1       (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_NTSC_EVLINE_SUB1     ))) 
#define TVIF_PAL_EVLINE_SUB1        (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_PAL_EVLINE_SUB1      )))
#define TVIF_IMAGE_WIDTH            (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_WIDTH          )))
#define TVIF_IMAGE_HEIGHT           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_HEIGHT         )))
#define TVIF_IMAGE_START_X          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_START_X        )))
#define TVIF_IMAGE_START_Y          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVIF_IMAGE_START_Y        ))) 
#define TVENC_Y_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_Y_SCALE_CTL         )))
#define TVENC_U_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_U_SCALE_CTL         )))
#define TVENC_V_SCALE_CTL           (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_V_SCALE_CTL         )))
#define TVENC_GAMMA_COEF_0          (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_GAMMA_COEF_0        )))
#define TVENC_UV_SCALE_GAIN_4_5     (TV_BASE  +(MMP_ULONG)(&(((AITPS_TV  )0)->TVENC_UV_SCALE_GAIN_4_5   )))
#endif
#endif //(CHIP == MCR_V2)

#endif // _MMPH_REG_DISPLAY_H_

///@end_ait_only

