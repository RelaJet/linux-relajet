//==============================================================================
//
//  File        : mmp_reg_vif.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_VIF_H_
#define _MMP_REG_VIF_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/
typedef struct _AITS_VIF_GRAB {
    AIT_REG_W   PIXL_ST;
    AIT_REG_W   PIXL_ED;
    AIT_REG_W   LINE_ST;
    AIT_REG_W   LINE_ED;
} AITS_VIF_GRAB, *AITPS_VIF_GRAB;

#define     MIPI_DATA_LANE_NUM  (4)

#define     VIF_NUM         (2)

//------------------------------
// VIF  Structure (0x8000 6000)
//------------------------------
typedef struct _AITS_VIF 
{
    AIT_REG_B   VIF_INT_HOST_EN [VIF_NUM];                              // 0x00
    AIT_REG_B                           _x02[0x2];
    AIT_REG_B   VIF_INT_HOST_SR [VIF_NUM];                              // 0x04    
    AIT_REG_B                           _x06[0x2];
    AIT_REG_B   VIF_INT_CPU_EN [VIF_NUM];                               // 0x08
    AIT_REG_B                           _x0A[0x2];
    AIT_REG_B   VIF_INT_CPU_SR [VIF_NUM];                               // 0x0C	
    AIT_REG_B                           _x0E[0x2];
        /*-DEFINE-----------------------------------------------------*/
		#define VIF_INT_GRAB_END        	0x01
        #define VIF_INT_FRM_END         	0x02
        #define VIF_INT_FRM_ST	        	0x04
        #define VIF_INT_LINE_0          	0x08
        #define VIF_INT_BUF_FULL        	0x10
        #define VIF_INT_LINE_1          	0x20
        #define VIF_INT_LINE_2          	0x40
        #define VIF_INT_SNR_FRM_END     	0x80
        #define VIF_INT_ALL             	0xFF
        /*------------------------------------------------------------*/
        
    AIT_REG_W   VIF_INT_LINE_NUM_0 [VIF_NUM];                           // 0x10
    AIT_REG_W   VIF_INT_LINE_NUM_1 [VIF_NUM];                           // 0x14
    AIT_REG_W   VIF_INT_LINE_NUM_2 [VIF_NUM];                           // 0x18
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_INT_LINE_NUM_MASK       0x01FFF
        /*------------------------------------------------------------*/
    AIT_REG_B							_x1C[0x2];
    AIT_REG_B   VIF_INT_MODE [VIF_NUM];                                 // 0x1E
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_INT_EVERY_FRM       	0x01
        #define VIF_INT_IF_OUT_ENABLE       0x00
        /*------------------------------------------------------------*/

    AIT_REG_B   VIF_IN_EN [VIF_NUM];                                    // 0x20
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_IN_ENABLE               0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_OUT_EN [VIF_NUM];                                   // 0x22
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_OUT_ENABLE              0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x24[0x2];
    AIT_REG_B   VIF_FRME_SKIP_EN [VIF_NUM];                             // 0x26
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_FRME_SKIP_ENABLE        0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_FRME_SKIP_NO [VIF_NUM];                             // 0x28
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_FRME_SKIP_NO_MASK       0x0F
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_IMG_BUF_EN [VIF_NUM];                             	// 0x2A
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_TO_ISP_IMG_BUF_EN		0x01
        #define VIF_TO_ISP_CLR_IMG_BUF_DIS	0x02
        #define VIF_TO_ISP_FS_CLR_IMG_BUF	0x04
        #define VIF_TO_ISP_LS_CLR_IMG_BUF	0x08
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x2C[0x4];

	AIT_REG_B	VIF_BLC_EN [VIF_NUM];									// 0x30
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_BLC_ENABLE              0x01
        /*------------------------------------------------------------*/
	AIT_REG_W	VIF_BLC_PIXL_UP_BOUND [VIF_NUM];						// 0x32
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_BLC_PIXL_UP_BOUND_MASK  0x02FF
        /*------------------------------------------------------------*/
	AIT_REG_B	VIF_BLC_OFFSET [VIF_NUM];								// 0x36
	AIT_REG_B	VIF_BLC_PIXL_H_ST [VIF_NUM];							// 0x38
	AIT_REG_B	VIF_BLC_LINE_V_ST [VIF_NUM];							// 0x3A
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_BLC_REGION_MASK         0x0F
        /*------------------------------------------------------------*/
	AIT_REG_B							_x3C[0x4];

	AIT_REG_B	VIF_COMP_ID00_OFFSET [VIF_NUM];							// 0x40
	AIT_REG_B	VIF_COMP_ID01_OFFSET [VIF_NUM];							// 0x42
	AIT_REG_B	VIF_COMP_ID10_OFFSET [VIF_NUM];							// 0x44
	AIT_REG_B	VIF_COMP_ID11_OFFSET [VIF_NUM];							// 0x46			
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_COMP_OFFSET_SIGN_MASK   0x80
        #define VIF_COMP_OFFSET_SIGN_POS    0x00
        #define VIF_COMP_OFFSET_SIGN_NEG    0x80
        #define VIF_COMP_OFFSET_MASK        0x7F                
        /*------------------------------------------------------------*/
	AIT_REG_B	VIF_DITHER_COMP_EN [VIF_NUM];							// 0x48
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_DITHER_COMP_ENABLE      0x01       
        /*------------------------------------------------------------*/
	AIT_REG_B	VIF_DITHER_COMP_MODE [VIF_NUM];							// 0x4A
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_DITHER_MODE_0       	0x00
        #define VIF_DITHER_MODE_1       	0x01
        #define VIF_DITHER_MODE_2       	0x10
        #define VIF_DITHER_MODE_3       	0x11
        #define VIF_DITHER_R_SHIFT			0
        #define VIF_DITHER_GR_SHIFT			2
        #define VIF_DITHER_GB_SHIFT			4
        #define VIF_DITHER_B_SHIFT			6                                          
        /*------------------------------------------------------------*/
	AIT_REG_B							_x4C[0x4];

	AIT_REG_W	VIF_BLC_ID00_ACC [VIF_NUM];								// 0x50
	AIT_REG_W	VIF_BLC_ID01_ACC [VIF_NUM];								// 0x54
	AIT_REG_W	VIF_BLC_ID11_ACC [VIF_NUM];								// 0x58
	AIT_REG_W	VIF_BLC_ID10_ACC [VIF_NUM];								// 0x5C		
	
	AIT_REG_B   VIF_BL_FILTER_EN [VIF_NUM];                             // 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_BL_FILTER_ENABLE        0x01       
        /*------------------------------------------------------------*/
	AIT_REG_B   VIF_BL_FILTER_CC1 [VIF_NUM];                            // 0x62
	AIT_REG_B   VIF_BL_FILTER_CC2 [VIF_NUM];                            // 0x64
	AIT_REG_B   VIF_BL_FILTER_CC3 [VIF_NUM];                            // 0x66
	AIT_REG_W   VIF_BL_FILTER_TH [VIF_NUM];                             // 0x68
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_BL_FILTER_TH_MASK       0x0FFF       
        /*------------------------------------------------------------*/
    AIT_REG_W   VIF_IGBT_EXT_COUNT [VIF_NUM];                           // 0x6C	

    AIT_REG_B   VIF_IGBT_EN [VIF_NUM];                                  // 0x70
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_IGBT_OUT_EN       		0x01
        #define VIF_IGBT_COUNT_IN_VBLANK    0x02                                          
        /*------------------------------------------------------------*/    
    AIT_REG_W   VIF_IGBT_LINE_ST [VIF_NUM];                             // 0x72
    AIT_REG_W   VIF_IGBT_OFST_ST [VIF_NUM];                             // 0x76
    AIT_REG_W   VIF_IGBT_LINE_CYC [VIF_NUM];                            // 0x7A
    AIT_REG_B                           _x7E[0x2];

    AIT_REG_B   VIF_SENSR_MCLK_CTL [VIF_NUM];                           // 0x80
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_SENSR_MCLK_EN           0x01
        #define VIF_SENSR_MCLK_POLAR_NEG    0x02
        #define VIF_SENSR_MCLK_POLAR_PST    0x00
        #define VIF_SENSR_PHASE_DELAY_NONE  0x00
        #define VIF_SENSR_PHASE_DELAY_0_5F  0x10
        #define VIF_SENSR_PHASE_DELAY_1_0F  0x20
        #define VIF_SENSR_PHASE_DELAY_1_5F  0x30
        #define VIF_SENSR_PHASE_DELAY_MASK  0x30
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_SENSR_CLK_FREQ [VIF_NUM];                           // 0x82
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_SENSR_CLK_PLL_DIV(_a)   (_a - 1)
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_SENSR_CTL [VIF_NUM];                                // 0x84
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_PCLK_LATCH_PST     	    0x00
        #define VIF_PCLK_LATCH_NEG     	    0x80
        #define VIF_PCLK_LATCH_MASK			0x80
        #define VIF_FIXED_OUT_EN        	0x40
        #define VIF_14BPP_OUT_EN        	0x10
        #define VIF_12BPP_OUT_EN        	0x00
        #define VIF_LINE_ID_POLAR       	0x08
        #define VIF_PIXL_ID_POLAR       	0x04
        #define VIF_COLORID_FORMAT_MASK 	0x0C
        #define VIF_COLORID_11              0x0C 
        #define VIF_COLORID_10              0x08
        #define VIF_COLORID_01              0x04
        #define VIF_COLORID_00              0x00      
        #define VIF_HSYNC_POLAR_NEG     	0x02
        #define VIF_HSYNC_POLAR_PST        	0x00
        #define VIF_VSYNC_POLAR_NEG     	0x01
        #define VIF_VSYNC_POLAR_PST    	    0x00
        #define VIF_SYNCSIG_POLAR_MASK		0x03
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_FIXED_OUT_DATA [VIF_NUM];                           // 0x86
    AIT_REG_B   VIF_IN_PIXL_CLK_DLY [VIF_NUM];                          // 0x88
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_IN_PIXL_CLK_NO_DLY      0x00
        #define VIF_IN_PIXL_CLK_DLY_1CELL 	0x01
        #define VIF_IN_PIXL_CLK_DLY_2CELL   0x02
        #define VIF_IN_PIXL_CLK_DLY_3CELL   0x03
        /*------------------------------------------------------------*/  
    AIT_REG_B   VIF_PARALLEL_SNR_BUS_CFG;								// 0x8A
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_PARA_DATA_MAP_MASK      0x0F
        #define VIF_DATA_BUS_MAP_16BIT		0x01
        #define VIF_DATA_BUS_MAP_14BIT_L0	0x02
        #define VIF_DATA_BUS_MAP_14BIT_LDUP	0x03
        #define VIF_DATA_BUS_MAP_12BIT_L0	0x04
        #define VIF_DATA_BUS_MAP_12BIT_LDUP	0x05
        #define VIF_DATA_BUS_MAP_10BIT_L0	0x06
        #define VIF_DATA_BUS_MAP_10BIT_LDUP	0x07
        #define VIF_DATA_BUS_MAP_8BIT_L0	0x08
        #define VIF_DATA_BUS_MAP_8BIT_LDUP	0x09
        
        #define VIF_SNR_INPUT_DIS			0x00
        #define VIF_SNR_INPUT_BUS_10BIT_EN	0x10
        #define VIF_SNR_INPUT_BUS_16BIT_EN	0x30                                       
        
        /* For parallel pin order workaround */
        #define VIF_PARA_10BIT_INVERT_PATCH	0x40
        #define VIF_PARA_6BIT_INVERT_PATCH	0x80
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x8B[0x5];
    
    AITS_VIF_GRAB   VIF_GRAB[VIF_NUM];									// 0x90
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_GRAB_RANGE_MASK       	0x01FF
        /*------------------------------------------------------------*/

    AIT_REG_B   VIF_OPR_UPD [VIF_NUM];                                  // 0xA0
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_OPR_UPD_FRAME_SYNC      0x01
        #define VIF_OPR_UPD_EN          	0x02
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xA2[0xE];
    
    AIT_REG_B   VIF_RAW_OUT_EN [VIF_NUM];                               // 0xB0
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_2_RAW_EN            	0x01
        #define VIF_2_ISP_EN            	0x02
        #define VIF_OUT_MASK                0x03
        #define VIF_JPG_2_RAW_EN        	0x04
        #define VIF_1_TO_ISP            	0x08
        #define VIF_PIPE_FRAME_ST_EN		0x10
        #define	VIF_PIPE_RRAME_ST_MODE		0x20
        #define VIF_1_YUV_TO_ISP			0x40
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_YUV_CTL [VIF_NUM];                                  // 0xB2
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_YUV_EN              	0x01
        #define VIF_Y2B_EN              	0x02
        #define VIF_Y2B_COLR_ID_MASK    	0x0C
        #define VIF_Y2B_COLR_ID(_pixl, _line)   (((_pixl << 2) | (_line << 3)) & VIF_Y2B_COLR_ID_MASK)
        #define VIF_YUV422_FMT_YUYV      	0x00
        #define VIF_YUV422_FMT_YVYU      	0x10
        #define VIF_YUV422_FMT_UYVY      	0x20
        #define VIF_YUV422_FMT_VYUY      	0x30
		#define VIF_YUV422_FMT_MASK     	0x30
		#define VIF_YUV_LPF_DIS         	0x40
		#define VIF_YUV_PATH_OUT_YCBCR444   0x00
		#define VIF_YUV_PATH_OUT_YUV       	0x80
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_YUV420_CTL [VIF_NUM];                              	// 0xB4
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_MIPI_YUV420_EN      	0x01
        #define VIF_MIPI_YUV420_U_EVEN_LN 	0x00
        #define VIF_MIPI_YUV420_Y_EVEN_LN 	0x02
        #define VIF_MIPI_YUV420_U_FIRST 	0x00
        #define VIF_MIPI_YUV420_V_FIRST 	0x04
        #define VIF_PARA_YUV422T0420_EN		0x10
        #define VIF_PARA_YUV422_FMT_MASK	0x60
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_TW4CH_CTL0;                              	        // 0xB6
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_TW_MODE_EN		    	0x01
        #define VIF_TW_CH_MASK				0x06
		#define VIF_TW_SRC_FROM_SNR0		0x00        
		#define	VIF_TW_SRC_FROM_SNR1		0x20
        /*------------------------------------------------------------*/
    AIT_REG_B   VIF_TW4CH_CTL1;                              	        // 0xB7 
    AIT_REG_B                           _xB8[0x8];
    
    AIT_REG_B   VIF_0_SENSR_SIF_EN;                                     // 0xC0
    AIT_REG_B   VIF_0_SENSR_SIF_DATA;                                   // 0xC1
    AIT_REG_B   VIF_1_SENSR_SIF_EN;                                     // 0xC2
    AIT_REG_B   VIF_1_SENSR_SIF_DATA;                                   // 0xC3
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_SIF_SEN_SHFT			0
        #define VIF_SIF_SCL_SHFT            1
        #define VIF_SIF_SDA_SHFT            2
        #define VIF_SIF_RST_SHFT            3
        #define VIF_SIF_SEN                 (0x1 << VIF_SIF_SEN_SHFT)
        #define VIF_SIF_SCL                 (0x1 << VIF_SIF_SCL_SHFT)
        #define VIF_SIF_SDA                 (0x1 << VIF_SIF_SDA_SHFT)
        #define VIF_SIF_RST                 (0x1 << VIF_SIF_RST_SHFT)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xC4[0x4];
    AIT_REG_W   VIF_SNR_CUR_LINE_CNT [VIF_NUM];                         // 0xC8
    AIT_REG_B                           _xCC[0x4];

    AIT_REG_B   VIF_DNSPL_RATIO_CTL [VIF_NUM];                          // 0xD0
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_DNSPL_H_RATIO_MASK		0x03
        #define VIF_DNSPL_V_RATIO_MASK      0x0C
        #define VIF_DNSPL_H_AVG_EN			0x10 
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xD2[0x1E];
    
    AIT_REG_W   VIF_SRAM_DELAY_CTL [VIF_NUM];     					    // 0xF0
    AIT_REG_B                           _xF4[0xC];

    AIT_REG_B   VIF_0_STS_MISSING_DAT;                                  // 0x100
        /*-DEFINE-----------------------------------------------------*/
        #define VIF_0_STS_MISSING_PIXL      0x01
        #define VIF_0_STS_MISSING_LINE		0x02
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x101;
    AIT_REG_W   VIF_0_ERR_PIXL_CNT;                                     // 0x102
    AIT_REG_W   VIF_0_ERR_LINE_CNT;                                     // 0x104
    AIT_REG_W   VIF_0_CRC_ERR_LINE_CNT;                                 // 0x106
    AIT_REG_B   VIF_1_STS_MISSING_DAT;                                  // 0x108
    AIT_REG_B                           _x109;
    AIT_REG_W   VIF_1_ERR_PIXL_CNT;                                     // 0x10A
    AIT_REG_W   VIF_1_ERR_LINE_CNT;                                     // 0x10C
    AIT_REG_W   VIF_1_CRC_ERR_LINE_CNT;                                 // 0x10E

} AITS_VIF, *AITPS_VIF;

//------------------------------
// MIPI Data Lane Structure (0x8000 6120)
//------------------------------
typedef struct _AITS_MIPI_DATA_LANE
{
    AIT_REG_B   MIPI_DATA_CFG [VIF_NUM];                               // 0x20
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_DAT_LANE_EN        	0x01
		#define MIPI_DAT_DLY_EN         	0x02
		#define MIPI_DAT_SRC_PHY_0      	0x00
		#define MIPI_DAT_SRC_PHY_1      	0x04
		#define MIPI_DAT_SRC_PHY_2      	0x08
		#define MIPI_DAT_SRC_PHY_3      	0x0C
		#define MIPI_DAT_SRC_SEL_MASK   	0x0C
		#define MIPI_DAT_LANE_SWAP_EN       0x10
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x22[0x2];
    AIT_REG_W   MIPI_DATA_DELAY [VIF_NUM];                              // 0x24
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_DATA_DLY_MASK       	0x0F
		#define MIPI_DATA_DLY(_a)        	(_a & MIPI_CLK_DLY_MASK)
		#define MIPI_DATA_SYNC_FIFO_SEL     0x10
		#define MIPI_DATA_RECOVERY          0x20
		
		#define MIPI_IGNORE_DATA_NUM_MASK   0x3F00
		#define MIPI_DATA_SOT_CNT(_a)		((_a << 8) & MIPI_IGNORE_DATA_NUM_MASK) 
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x28[0x6];
    AIT_REG_B   MIPI_DATA_ERR_SR [VIF_NUM];                            // 0x2E
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_SOT_SEQ_ERR        	0x01
		#define MIPI_SOT_SYNC_ERR       	0x02
		#define MIPI_CTL_ERR         		0x04
        /*------------------------------------------------------------*/
} AITS_MIPI_DATA_LANE, *AITPS_MIPI_DATA_LANE;

//------------------------------
// MIPI RX BIST Structure (0x8000 6180)
//------------------------------
typedef struct _AITS_MIPI_RX_BIST
{
    AIT_REG_B	MIPI_RX_BIST_CTL;										// 0x80
    AIT_REG_B							_x81;
    AIT_REG_D	MIPI_RX_BIST_BYTECNT;									// 0x82
    AIT_REG_B	MIPI_RX_BIST_TX_OPT;									// 0x86		
    AIT_REG_B							_x87;
    AIT_REG_B	MIPI_RX_BIST_TX_HS;									    // 0x88	
    AIT_REG_B	MIPI_RX_BIST_TEST_MODE;								    // 0x89
    AIT_REG_W	MIPI_RX_BIST_ERR_CNT;									// 0x8A
    AIT_REG_B	MIPI_RX_BIST_1ST_ERR_BYTE;							    // 0x8C        
    AIT_REG_B	MIPI_RX_BIST_1ST_GOLD_BYTE;							    // 0x8D 
    AIT_REG_B	MIPI_RX_BIST_2ND_ERR_BYTE;							    // 0x8E        
    AIT_REG_B	MIPI_RX_BIST_2ND_GOLD_BYTE;							    // 0x8F
} AITS_MIPI_RX_BIST, *AITPS_MIPI_RX_BIST;

//------------------------------
// MIPI  Structure (0x8000 6110)
//------------------------------
typedef struct _AITS_MIPI 
{
    AIT_REG_W   MIPI_CLK_CFG [VIF_NUM];                                 // 0x10
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_CSI2_EN           		0x0001
		#define MIPI_CLK_DLY_EN         	0x0002
		#define MIPI_BCLK_LATCH_EDGE_MASK   0x0004
		#define MIPI_BCLK_LATCH_EDGE_POS    0x0004
		#define MIPI_BCLK_LATCH_EDGE_NEG    0x0000
		#define MIPI_BCLK_SRC_LANE1         0x0000
		#define MIPI_BCLK_SRC_LANE2      	0x0008
		#define MIPI_CLK_LANE_SWAP_EN       0x0010
		#define MIPI_INPUT_MODE				0x0020
		#define MIPI_INPUT_EN				0x0040
		#define MIPI_CLK_DLY_MASK       	0x0F00
		#define MIPI_CLK_DLY(_a)        	((_a << 8) & MIPI_CLK_DLY_MASK)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x14[0x4];
    AIT_REG_B	MIPI_VC_CTL [VIF_NUM];									// 0x18
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_VC_EN        			0x01
		#define MIPI_VC2ISP_CH_SEL_MASK     0x06
		#define MIPI_VC2ISP_CH_SEL(ch)      ((ch << 1) & MIPI_VC2ISP_CH_SEL_MASK)
		#define MIPI_VC2ISP_ALL_CH_EN		0x08
		#define MIPI_VC2RAW_CH_EN_MASK     	0xF0
		#define MIPI_VC2RAW_CH_EN(ch)      	((1 << (4 + ch)) & MIPI_VC2RAW_CH_EN_MASK)
        /*------------------------------------------------------------*/ 
    AIT_REG_B	MIPI_DPCM_EN [VIF_NUM];									// 0x1A    
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_DPCM_MODE_EN        	0x01
        /*------------------------------------------------------------*/
    AIT_REG_B	MIPI_VC2ISP_CTL [VIF_NUM];								// 0x1C
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_VC2ISP_CH_EN_MASK     	0x0F
		#define MIPI_VC2ISP_CH_EN(ch)      	((1 << (0 + ch)) & MIPI_VC2ISP_CH_EN_MASK)
		#define MIPI_VC_SLOW_FS_FOR_STAGGER_MODE	0x10
        /*------------------------------------------------------------*/ 
    AIT_REG_B   MIPI_CLK_ERR_SR[VIF_NUM];                               // 0x1E
    	/*-DEFINE-----------------------------------------------------*/
    	// ref to MIPI_DATA0_ERR_SR
        /*------------------------------------------------------------*/
    
    AITS_MIPI_DATA_LANE DATA_LANE[MIPI_DATA_LANE_NUM];                  // 0x20
    
    AIT_REG_B							_x60[0x20];						/* D-PHY Testing Mode */
    
    AITS_MIPI_RX_BIST   MIPI_BIST_CTL[VIF_NUM];                         // 0x80
        
    AIT_REG_B	MIPI_CSI2_ERR_SR [VIF_NUM];								// 0xA0
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_CSI2_1ECC_ERR        	0x01
		#define MIPI_CSI2_2ECC_ERR        	0x02
		#define MIPI_CSI2_CRC_ERR        	0x04
        /*------------------------------------------------------------*/
    AIT_REG_B							_xA2[0x6];
    AIT_REG_B	MIPI_CSI2_DATA_ID [VIF_NUM];							// 0xA8
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_PH_DATA_FMT_MASK       0x3F
		#define MIPI_PH_VC_MASK        	    0xC0
        /*------------------------------------------------------------*/
    AIT_REG_W	MIPI_CSI2_WORD_CNT [VIF_NUM];							// 0xAA            
    	/*-DEFINE-----------------------------------------------------*/
		#define MIPI_PH_WORD_CNT_MASK       0xFFFF
        /*------------------------------------------------------------*/
    AIT_REG_B							_xAE[0x2];
    
    AIT_REG_B	MIPI_0_CSI2_SW_CTL;										// 0xB0
    AIT_REG_W	MIPI_0_CSI2_SW_WORD_CNT;								// 0xB1    
    AIT_REG_B							_xB3;
    AIT_REG_B	MIPI_1_CSI2_SW_CTL;										// 0xB4
    AIT_REG_W	MIPI_1_CSI2_SW_WORD_CNT;								// 0xB5    
    AIT_REG_B							_xB7[0x9];
    
} AITS_MIPI, *AITPS_MIPI;

#if (CHIP == MERCURY)
//------------------------------
// VIF Sensor2 Structure (0x8000 61C0)
//------------------------------
typedef struct _AITS_VIF_SNR2 
{
    AIT_REG_W   VIF_2_INT_LINE_NUM_0;									// 0xC0    				
    AIT_REG_W   VIF_2_INT_LINE_NUM_1;									// 0xC2  
    AIT_REG_W   VIF_2_INT_LINE_NUM_2;									// 0xC4      
 	AIT_REG_B   						_xC6;							
 	AIT_REG_B   VIF_2_INT_MODE;											// 0xC7
	AIT_REG_B   VIF_2_IN_EN;											// 0xC8
	AIT_REG_B   VIF_2_OUT_EN;											// 0xC9
	AIT_REG_B   VIF_2_FRME_SKIP_EN;										// 0xCA
	AIT_REG_B   VIF_2_FRME_SKIP_NO;										// 0xCB
	AIT_REG_B   VIF_2_SENSR_CLK_CTL;									// 0xCC
	AIT_REG_B   VIF_2_SENSR_CLK_FREQ;									// 0xCD
 	AIT_REG_B   						_xCE[0x2];
	
	AIT_REG_B   VIF_2_SENSR_CTL;										// 0xD0
	AIT_REG_B   VIF_2_FIXED_OUT_DATA;									// 0xD1 
	AIT_REG_B   VIF_2_IN_PIXL_CLK_DLY;									// 0xD2
	AIT_REG_B   VIF_2_RAW_OUT_EN;										// 0xD3
    	/*-DEFINE-----------------------------------------------------*/
		#define VIF_2_PIPE_FRAME_ST_EN		0x01
		#define VIF_2_PIPE_RRAME_ST_MODE    0x02
		#define VIF_2_JPG_2_RAW_EN        	0x04
        /*------------------------------------------------------------*/
 	AIT_REG_B   						_xD4[0x4];
	AITS_VIF_GRAB   VIF_2_GRAB;											// 0xD8

	AIT_REG_B   VIF_2_OPR_UPD;											// 0xE0
 	AIT_REG_B   						_xE1;
    AIT_REG_B   VIF_2_SENSR_SIF_EN;                                     // 0xE2
    AIT_REG_B   VIF_2_SENSR_SIF_DATA;                                   // 0xE3
        /*-DEFINE-----------------------------------------------------*/
		// Ref : VIF_0_SENSR_SIF_EN
        /*------------------------------------------------------------*/
	AIT_REG_W   VIF_2_IGBT_EXT_COUNT;									// 0xE4
	AIT_REG_B   VIF_2_IGBT_COUNT_IN_VB_EN;								// 0xE6
 	AIT_REG_B   						_xE7[0x9];
 	
 	AIT_REG_B	VIF_2_STS_MISSING_DAT;									// 0xF0
 	AIT_REG_B   						_xF1; 	
  	AIT_REG_W   VIF_2_ERR_PIXL_CNT;                                     // 0xF2
    AIT_REG_W   VIF_2_ERR_LINE_CNT;  									// 0xF4
 	AIT_REG_B   						_xF6[0x0A]; 
	
} AITS_VIF_SNR2, *AITPS_VIF_SNR2;
#endif
/// @}
#endif // _MMPH_REG_VIF_H_
