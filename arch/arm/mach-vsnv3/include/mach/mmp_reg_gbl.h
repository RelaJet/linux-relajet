//==============================================================================
//
//  File        : mmph_reg_gbl.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================



#ifndef _MMP_REG_GBL_H_
#define _MMP_REG_GBL_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/
#if (CHIP == MCR_V2)
// **********************************
//   GLOBAL structure (0x8000 6900)
// **********************************
typedef struct _AITS_GBL {
    AIT_REG_W   GBL_DPLL0_CFG;                              // 0x5D00
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL_CHANGE_CLK_WAIT_EN		0x0001
        #define DPLL_CHANGE_CLK_WAIT_CNT	0x0002
        #define DPLL_BYPASS                 0x0004
        #define DPLL_UPDATE_PARAM           0x0008
        #define DPLL_SRC_MCLK               0x0000
        #define DPLL0_SRC_DPLL5             0x0010
        #define DPLL0_SRC_DPLL1             0x0020
        #define DPLL0_SRC_DPLL2             0x0030
        #define DPLL0_SRC_DPLL3             0x0040
        #define DPLL0_SRC_DPLL4             0x0050
        #define DPLL_SRC_PI2S_MCLK          0x0060
        #define DPLL_SRC_PBGPIO13           0x0070
        #define DPLL_SRC_MASK               0x00F0
        #define DPLL_DELAY_MASK             0xFF00
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL0_PWR;                              // 0x5D02
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL_PWR_UP                 0x00
        #define DPLL_PWR_DOWN               0x01
        // ++MCR_V2_MP Only
        #define DPLL_BIST_EN                0x02
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL0_M;                                // 0x5D03
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL0_M_DIV_1               0x00
        #define DPLL0_M_DIV_2               0x01
        #define DPLL0_M_DIV_3               0x05
        #define DPLL0_M_DIV_4               0x02
        #define DPLL0_M_DIV_6               0x06
        // ++MCR_V2_MP Only
        #define DPLL0_M_DIV_1_MP            0x00
        #define DPLL0_M_DIV_2_MP            0x01
        #define DPLL0_M_DIV_3_MP            0x02
        #define DPLL0_M_DIV_4_MP            0x05
        #define DPLL0_M_DIV_6_MP            0x03
        #define DPLL0_M_DIV_12_MP           0x07
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL0_N;                                // 0x5D04
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL0_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL0_PARAM[0x0B];                      // 0x5D05
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_DPPL0_PARAM[0x00] 0x5D05 */
        // ++MCR_V2_MP Only
        #define DPLL0_P_DIV_1_MP            0x00
        #define DPLL0_P_DIV_2_MP            0x01
        #define DPLL0_P_DIV_3_MP            0x02
        #define DPLL0_P_DIV_4_MP            0x05
        #define DPLL0_P_DIV_6_MP            0x03
        #define DPLL0_P_DIV_12_MP           0x07
        // --MCR_V2_MP Only
        /* GBL_DPPL0_PARAM[0x01] 0x5D06 */
        // ++MCR_V2_MP Only
        #define DPLL0_DIV2_FB_LOOP          0x01
        #define DPLL0_VCO_CTL_1d5G          0x02
        #define DPLL0_KVCO_OFFSET_0uA       0x00
        #define DPLL0_KVCO_OFFSET_5uA       0x04
        #define DPLL0_KVCO_OFFSET_10uA      0x08
        #define DPLL0_KVCO_OFFSET_15uA      0x0C
        #define DPLL0_KVCO_OFFSET_MASK      0x0C
        #define DPLL0_ICP_5uA      			0x00
        #define DPLL0_ICP_10uA      		0x10
        #define DPLL0_ICP_15uA      		0x20
        #define DPLL0_ICP_20uA      		0x30
        #define DPLL0_ICP_MASK      		0x30
        #define DPLL0_VCP_AUTO      		0x40
        #define DPLL0_VCP_DIODE      		0x80
        #define DPLL0_VCP_OP  	    		0xC0
        #define DPLL0_VCP_MASK      		0xC0
        // --MCR_V2_MP Only
        /* GBL_DPPL0_PARAM[0x02][1:0] 0x5D07 */
        /* For Fractional PLL only */
        /* GBL_DPPL0_PARAM[0x02][7:2] 0x5D07 */
        #define DPLL_FVCO_DIV_1             0x0C
        #define DPLL_FVCO_DIV_2             0x08
        #define DPLL_FVCO_DIV_4             0x04
        #define DPLL_FVCO_DIV_8             0x00
        #define DPLL_FVCO_DIV_MASK          0x1C
        #define DPLL_TEST_MODE_EN			0x20
        #define DPLL_INV_CLK_EN				0x80
        
        /* GBL_DPPL0_PARAM[0x03] 0x5D08 */
        #define DPLL_336_792MHZ             0x00
        #define DPLL_384_1008MHZ            0x01
        #define DPLL_V2I_LOW_GAIN           0x00
        #define DPLL_V2I_HIGH_GAIN          0x02
        
        #define DPLL_LFS_MASK               0x1C
        #define DPLL_LFS_9d3k               0x00
        #define DPLL_LFS_8d3k               0x04
		#define DPLL_LFS_8d0k               0x08
        #define DPLL_LFS_80                 0x08
        #define DPLL_LFS_7d6k               0x0C
        #define DPLL_LFS_7d3k               0x10
        #define DPLL_LFS_6d6k               0x14
        #define DPLL_LFS_6d0k               0x18
        #define DPLL_LFS_5d6k               0x1C
        #define DPLL_BIAS_OPTION_1000       0x60
		#define DPLL_BIAS_OPTION_100d0    	0x60
        /* GBL_DPPL0_PARAM[0x04] 0x5D09 */
        #define DPLL_LOCK_DETECT_RANGE_0d7 	0x00
        #define DPLL_LOCK_DETECT_RANGE_2d0 	0x01
        #define DPLL_LOCK_DETECTOR_RANGE_20 0x01		
        #define DPLL_LOCK_DETECT_RANGE_3d0 	0x02
        #define DPLL_LOCK_DETECT_RANGE_3d5 	0x03
        #define DPLL_V2I_GAIN_ADJUST_5uA    0x00
        #define DPLL_V2I_GAIN_ADJUST_DIS    0x04
        #define DPLL_BIAS_CTL0_MASK         0xF0
        /* GBL_DPPL0_PARAM[0x05] 0x5D0A */
        #define DPLL_BIAS_CTL_VCO_5uA       0x00
        #define DPLL_BIAS_CTL_VCO_10uA      0x02
        #define DPLL_BIAS_CTL_VCO_15uA      0x04
        #define DPLL_BIAS_CTL2_MASK         0xFF
        /* GBL_DPPL0_PARAM[0x06] 0x5D0B */
        #define DPLL_BIAS_CTL_ICP_5uA       0x00
        #define DPLL_BIAS_CTL_ICP_7d5uA     0x01
        #define DPLL_BIAS_CTL_ICP_10uA      0x02
        #define DPLL_BIAS_CTL_ICP_12d5uA    0x03
        #define DPLL_BIAS_CTL3_MASK         0x0F
        /*------------------------------------------------------------*/

    AIT_REG_W   GBL_DPLL1_CFG;                              // 0x5D10
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL1_SRC_DPLL0             0x0010
        #define DPLL1_SRC_DPLL5             0x0020
        #define DPLL1_SRC_DPLL2             0x0030
        #define DPLL1_SRC_DPLL3             0x0040
        #define DPLL1_SRC_DPLL4             0x0050
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL1_PWR;                              // 0x5D12
    AIT_REG_B   GBL_DPLL1_M;                                // 0x5D13
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL1_M_DIV_1               0x00
        #define DPLL1_M_DIV_2               0x01
        #define DPLL1_M_DIV_3               0x05
        #define DPLL1_M_DIV_4               0x02
        #define DPLL1_M_DIV_6               0x06
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL1_N;                                // 0x5D14
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL1_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL1_PARAM[0x0B];                      // 0x5D15

    AIT_REG_W   GBL_DPLL2_CFG;                              // 0x5D20
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL2_SRC_DPLL0             0x0010
        #define DPLL2_SRC_DPLL1             0x0020
        #define DPLL2_SRC_DPLL5             0x0030
        #define DPLL2_SRC_DPLL3             0x0040
        #define DPLL2_SRC_DPLL4             0x0050
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL2_PWR;                              // 0x5D22
    AIT_REG_B   GBL_DPLL2_M;                                // 0x5D23
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL2_M_DIV_1               0x00
        #define DPLL2_M_DIV_2               0x01
        #define DPLL2_M_DIV_3               0x05
        #define DPLL2_M_DIV_4               0x02
        #define DPLL2_M_DIV_6               0x06
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL2_N;                                // 0x5D24
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL2_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL2_PARAM[0x0B];                      // 0x5D25

    AIT_REG_W   GBL_DPLL3_CFG;                              // 0x5D30
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL3_SRC_DPLL0             0x0010
        #define DPLL3_SRC_DPLL1             0x0020
        #define DPLL3_SRC_DPLL2             0x0030
        #define DPLL3_SRC_DPLL5             0x0040
        #define DPLL3_SRC_DPLL4             0x0050
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL3_PWR;                              // 0x5D32
    AIT_REG_B   GBL_DPLL3_M;                                // 0x5D33
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL3_M(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL3_N;                                // 0x5D34
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL3_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL3_PARAM[0x0B];                      // 0x5D35
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_DPPL3_PARAM[0x00] 0x5D35 */
        #define DPLL3_K0_MASK                0xFF
        /* GBL_DPPL3_PARAM[0x01] 0x5D36 */
        #define DPLL3_K1_MASK                0xFF
        /* GBL_DPPL3_PARAM[0x02] 0x5D37 */
        #define DPLL3_K2_MASK                0x03
        /* GBL_DPPL3_PARAM[0x03] 0x5D38 */
        #define DPLL3_CURR_BOOST_RATIO_4     0x00
        #define DPLL3_CURR_BOOST_RATIO_2     0x04
        #define DPLL3_DITHER_EN				0x20
        #define DPLL3_FRAC_DIS               0x40
        #define DPLL3_INV_CLK              	0x80

        /* GBL_DPPL3_PARAM[0x04] 0x5D39 */
        #define DPLL3_TCKO_OUT_EN			0x01
        #define DPLL3_TEST_MODE_EN			0x02
        #define DPLL3_BOOST_CP_EN           0x08	//[Reserved]
        #define DPLL3_MAIN_CP_VP_OP         0x00	//[Reserved]
        #define DPLL3_MAIN_CP_VP_DIODE      0x10	//[Reserved]
        /*------------------------------------------------------------*/

    AIT_REG_W   GBL_DPLL4_CFG;                              // 0x5D40
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL4_SRC_DPLL0             0x0010
        #define DPLL4_SRC_DPLL1             0x0020
        #define DPLL4_SRC_DPLL2             0x0030
        #define DPLL4_SRC_DPLL3             0x0040
        #define DPLL4_SRC_DPLL5             0x0050
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL4_PWR;                              // 0x5D42
    AIT_REG_B   GBL_DPLL4_M;                                // 0x5D43
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL4_M(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL4_N;                                // 0x5D44
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL4_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL4_PARAM[0x0B];                      // 0x5D45
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_DPPL4_PARAM[0x01] 0x5D46 */
        /*#define DPLL4_R(r0, r1, r2)         (((r0==1)?0x0:0x1)|\
                                            ((r1==1)?0x0:0x2)|\
                                            ((r2==1)?0x0:0x4))*/
        #define DPLL4_P_DIV_1               0x00
        #define DPLL4_P_DIV_2               0x01
        #define DPLL4_P_DIV_3               0x02
        #define DPLL4_P_DIV_6               0x03
        #define DPLL4_P_DIV_4               0x05
        #define DPLL4_P_DIV_12              0x07                                      
        #define DPLL4_R_MASK                0x07
        #define DPLL4_SS_EN                 0x10
        /* GBL_DPPL4_PARAM[0x02] 0x5D47 */
        #define DPLL4_SS_SRCSEL_0d31uA      0x00
        #define DPLL4_SS_SRCSEL_0d62uA      0x01
        #define DPLL4_SS_SRCSEL_1d25uA      0x02
        #define DPLL4_SS_SRCSEL_2d5uA       0x03
        /* GBL_DPPL4_PARAM[0x04] 0x5D49 */
        #define DPLL4_CP_2d5uA              0x00
        #define DPLL4_CP_5uA                0x04
        #define DPLL4_CP_7d5uA              0x08
        #define DPLL4_CP_10uA               0x0C
        #define DPLL4_400MHZ                0x00
        #define DPLL4_800MHZ                0x70
        #define DPLL4_CP_DIODE              0x00
        #define DPLL3_CP_OPMODE             0x80
        /*------------------------------------------------------------*/

    AIT_REG_W   GBL_DPLL5_CFG;                              // 0x5D50
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL5_SRC_DPLL0             0x0010
        #define DPLL5_SRC_DPLL1             0x0020
        #define DPLL5_SRC_DPLL2             0x0030
        #define DPLL5_SRC_DPLL3             0x0040
        #define DPLL5_SRC_DPLL4             0x0050
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL5_PWR;                              // 0x5D52
    AIT_REG_B   GBL_DPLL5_P;                                // 0x5D53
        /*-DEFINE-----------------------------------------------------*/
        /*#define DPLL5_P(p0, p1, p2)         (((p0==1)?0x0:0x1)|\
                                            ((p1==1)?0x0:0x2)|\
                                            ((p2==1)?0x0:0x4))*/
        #define DPLL5_P_DIV_1               0x00
        #define DPLL5_P_DIV_2               0x01
        #define DPLL5_P_DIV_3               0x02
        #define DPLL5_P_DIV_6               0x03
        #define DPLL5_P_DIV_4               0x05
        #define DPLL5_P_DIV_12              0x07                                    
        #define DPLL5_P_MASK                0x07
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPLL5_N;                                // 0x5D54
        /*-DEFINE-----------------------------------------------------*/
        #define DPLL5_N(_a)                 (_a&0xFF)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DPPL5_PARAM[0x0B];                      // 0x5D55
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_DPPL5_PARAM[0x02] 0x5D57 */
        /*#define DPLL5_O(o0, o1, o2)         ((((o0==1)?0x0:0x1)|\
                                            ((o1==1)?0x0:0x2)|\
                                            ((o2==1)?0x0:0x4))<<2)*/   
        #define DPLL5_OUTPUT_DIV_1          0x00
        #define DPLL5_OUTPUT_DIV_2          0x04
        #define DPLL5_OUTPUT_DIV_3          0x08   
        #define DPLL5_OUTPUT_DIV_6          0x0C
        #define DPLL5_OUTPUT_DIV_4          0x14
        #define DPLL5_OUTPUT_DIV_12         0x1C                                                                     
        #define DPLL5_R_MASK                0x1C
        
        /* GBL_DPPL5_PARAM[0x03] 0x5D58 */
        #define DPLL5_CHARGE_PUMP_2d5uA     0x00   
        #define DPLL5_CHARGE_PUMP_5uA       0x02
        #define DPLL5_CHARGE_PUMP_7d5uA     0x04
        #define DPLL5_CHARGE_PUMP_10uA      0x06
        #define DPLL5_800MHZ                0x00
        #define DPLL5_1200MHZ               0x08
        #define DPLL5_VCO_FREQ_MASK         0x08
        #define DPLL5_KVCO_OFFSET_0         0x00
        #define DPLL5_KVCO_OFFSET_5uA       0x10
        #define DPLL5_KVCO_OFFSET_10uA      0x20
        #define DPLL5_KVCO_OFFSET_15uA      0x30
        #define DPLL5_FRAC_DIS              0x40
        
        /* GBL_DPPL5_PARAM[0x04] 0x5D59 */
        #define DPLL5_SS_EN                 0x08
        #define DPLL5_SS_DOWN               0x10
        #define DPLL5_DIV2_LOOPDIV_EN       0x40
        /*------------------------------------------------------------*/
        
    AIT_REG_B   _x5D60[0x20];

    AIT_REG_B   GBL_CLK_SRC;                                // 0x5D80
        /*-DEFINE-----------------------------------------------------*/
        #define GRP_CLK_SRC0_PLL0           0x00
        #define GRP_CLK_SRC0_PLL1           0x01
        #define GRP_CLK_SRC0_PLL2           0x02
        #define GRP_CLK_SRC0_PMCLK          0x03
        #define GRP_CLK_SRC0_PLL3           0x04
        #define GRP_CLK_SRC0_PLL4           0x05
        #define GRP_CLK_SRC0_PLL5           0x06
        #define GRP_CLK_SRC0_MASK           0x07
        #define GRP_CLK_SRC1_PLL0           0x00
        #define GRP_CLK_SRC1_PLL1           0x10
        #define GRP_CLK_SRC1_PLL2           0x20
        #define GRP_CLK_SRC1_PMCLK          0x30
        #define GRP_CLK_SRC1_PLL3           0x40
        #define GRP_CLK_SRC1_PLL4           0x50
        #define GRP_CLK_SRC1_PLL5           0x60
        #define GRP_CLK_SRC1_MASK           0x70
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_CLK_DIV;                                // 0x5D81
        /*-DEFINE-----------------------------------------------------*/
        #define GRP_CLK_DIV_EN              0x01
        #define GRP_CLK_DIV(_a)             (((_a & 0x1F)-1) << 1)
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_CLK_CTL;                                // 0x5D82
        /*-DEFINE-----------------------------------------------------*/
        #define GRP_CLK_SEL_SRC0            0x00
        #define GRP_CLK_SEL_SRC1            0x01
        #define GRP_CLK_DIV2_DIS            0x00
        #define GRP_CLK_DIV2_EN             0x02
        /*------------------------------------------------------------*/
    AIT_REG_B   _x5D83;
    AIT_REG_B   GBL_CPU_A_CLK_SRC;                          // 0x5D84
    AIT_REG_B   GBL_CPU_A_CLK_DIV;                          // 0x5D85
    AIT_REG_B   GBL_CPU_A_CLK_CTL;                          // 0x5D86
    AIT_REG_B   _x5D87;
    AIT_REG_D   GBL_CPU_A_SMOOTH_DIV;                       // 0x5D88
    	/*-DEFINE-----------------------------------------------*/
    	#define PLL_SMOOTH_DIV_MASK           0x07
    	/*------------------------------------------------------------*/
    AIT_REG_B   GBL_TV_CLK_SRC;                             // 0x5D8C
    AIT_REG_B   GBL_TV_CLK_DIV;                             // 0x5D8D
    AIT_REG_B   GBL_USB_CLK_SRC;                            // 0x5D8E
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_USB_ROOT_CLK_SEL_USB    0x00
        #define GBL_USB_ROOT_CLK_SEL_PLL0   0x10
        #define GBL_USB_ROOT_CLK_SEL_PLL1   0x20
        #define GBL_USB_ROOT_CLK_SEL_MCLK   0x30
        #define GBL_USB_ROOT_CLK_SRC_MASK   0x30
        #define USBPHY_CLK_SRC_PMCLK_DIS    0x40
        #define USBPHY_CLK_SRC_PMCLK_EN     0x00
        // --MCR_V2
        // ++MCR_V2_MP 
        #define USBPHY_CLK_SRC_PMCLK_DIV2   0x00
        #define USBPHY_CLK_SRC_PMCLK        0x40
        #define USBPHY_CLK_SRC_ROOT_CLK     0x80
        #define USBPHY_CLK_SRC_MASK         0xC0
        // --MCR_V2_MP
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_USB_CLK_DIV;                            // 0x5D8F
    AIT_REG_B   GBL_DRAM_CLK_SRC;                           // 0x5D90
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_CLK_SYNC               0x00
        #define DRAM_CLK_ASYNC              0x10
        #define DRAM_CLK_DIV2_DIS           0x00
        #define DRMA_CLK_DIV2_EN            0x20
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_DRAM_CLK_DIV;                           // 0x5D91
    AIT_REG_B   GBL_VIF_CLK_SRC;                            // 0x5D92
    AIT_REG_B   GBL_VIF_CLK_DIV;                            // 0x5D93
    AIT_REG_B   GBL_BAYER_CLK_SRC;                          // 0x5D94
    AIT_REG_B   GBL_BAYER_CLK_DIV;                          // 0x5D95
    AIT_REG_B   GBL_ISP_CLK_SRC;                            // 0x5D96
    AIT_REG_B   GBL_ISP_CLK_DIV;                            // 0x5D97
    AIT_REG_B   GBL_MIPI_RX_BIST_CLK_SRC;                   // 0x5D98
    AIT_REG_B   _x5D99;
    AIT_REG_B   GBL_HDMI_CLK_SRC;                           // 0x5D9A
        /*-DEFINE-----------------------------------------------------*/
        #define HDMI_CLK_SRC_PIXL           0x07
        #define HDMI_CLK_DIV_MASK			0xF0
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_HDMI_CLK_DIV;                           // 0x5D9B
    AIT_REG_B   GBL_SENSR_CLK_SRC;                          // 0x5D9C
        /*-DEFINE-----------------------------------------------------*/
        #define SENSR0_CLK_SRC_PMCLK        0x00
        #define SENSR0_CLK_SRC_VI           0x01
        #define SENSR1_CLK_SRC_PMCLK        0x00
        #define SENSR1_CLK_SRC_VI           0x02
        #define SENSR2_CLK_SRC_PMCLK        0x00
        #define SENSR2_CLK_SRC_VI           0x04
        /*------------------------------------------------------------*/
    AIT_REG_B   _x5D9D[0x03];
    AIT_REG_B   GBL_AUD_CLK_SRC;                            // 0x5DA0
    AIT_REG_B   GBL_AUD_CLK_DIV[5];                         // 0x5DA1
    	#define AUD_CLK_DIV_ID_ADCM 	0x01 
    	#define AUD_CLK_DIV_ID_ADC 	0x02
    	#define AUD_CLK_DIV_ID_DACM 	0x03
	#define AUD_CLK_DIV_ID_DAC 	0x04
    	/*-DEFINE-----------------------------------------------------*/
    	// GBL_AUD_CLK_DIV[0x01~0x04] 0x5DA2~0x5DA5
    	#define AUD_CLK_DIV(_a)             ((_a & 0x1F)-1)
    	/*------------------------------------------------------------*/
    AIT_REG_B   _x5DA6[0x02];
    AIT_REG_B   GBL_CPU_B_CLK_SRC;                          // 0x5DA8
    AIT_REG_B   GBL_CPU_B_CLK_DIV;                          // 0x5DA9
    AIT_REG_B   GBL_CPU_B_CLK_CTL;                          // 0x5DAA
    AIT_REG_B   _x5DAB;
    AIT_REG_D   GBL_CPU_B_SMOOTH_DIV;                       // 0x5DAC
    AIT_REG_B   _x5DB0[0xB50];

    AIT_REG_D   GBL_CLK_DIS[2];                             // 0x6900
    AIT_REG_D   GBL_CLK_EN[2];                              // 0x6908
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_CLK_DIS[0x00] 0x6900 */
        #define GBL_CLK_CPU_A               0x00000001
		#define GBL_CLK_CPU_A_PHL           0x00000002
        #define GBL_CLK_CPU_B               0x00000004
        #define GBL_CLK_CPU_B_PHL           0x00000008
		#define GBL_CLK_MCI                 0x00000010
		#define GBL_CLK_DRAM                0x00000020
		#define GBL_CLK_VIF                 0x00000040
		#define GBL_CLK_RAW_F               0x00000080

        #define GBL_CLK_RAW_S0              0x00000100
        #define GBL_CLK_RAW_S1              0x00000200
        #define GBL_CLK_RAW_S2              0x00000400
        #define GBL_CLK_ISP                 0x00000800
        #define GBL_CLK_COLOR_MCI           0x00001000
        #define GBL_CLK_GNR                 0x00002000
        #define GBL_CLK_SCALE               0x00004000
        #define GBL_CLK_ICON                0x00008000

        #define GBL_CLK_IBC                 0x00010000
        #define GBL_CLK_CCIR                0x00020000
        #define GBL_CLK_DSPY                0x00040000
        #define GBL_CLK_HDMI                0x00080000
        #define GBL_CLK_TV                  0x00100000
        #define GBL_CLK_JPG                 0x00200000
        #define GBL_CLK_H264                0x00400000
        #define GBL_CLK_GRA                 0x00800000

        #define GBL_CLK_DMA                 0x01000000
        #define GBL_CLK_PWM                 0x02000000
        #define GBL_CLK_PSPI                0x04000000
        #define GBL_CLK_SM                  0x08000000
        #define GBL_CLK_SD0                 0x10000000
        #define GBL_CLK_SD1                 0x20000000
        #define GBL_CLK_SD2                 0x40000000
        #define GBL_CLK_SD3                 0x80000000
        #define GBL_CLK_ALL0                0xFFFFFFFF
        /* GBL_CLK_DIS[0x01] 0x6904 */
        #define GBL_CLK_USB                 0x00000001
		#define GBL_CLK_I2C                 0x00000002
        #define GBL_CLK_BS_SPI              0x00000004
        #define GBL_CLK_GPIO                0x00000008
		#define GBL_CLK_AUD                 0x00000010
		#define GBL_CLK_ADC                 0x00000020
		#define GBL_CLK_DAC                 0x00000040
		#define GBL_CLK_IRDA                0x00000080

        #define GBL_CLK_LDC                 0x00000100
        #define GBL_CLK_BAYER               0x00000200
        #define GBL_CLK_MIPI_RX_BIST        0x00000400
        #define GBL_CLK_ALL1                0x000007FF
        /*------------------------------------------------------------*/
    AIT_REG_D   GBL_SW_RST_EN[2];                           // 0x6910
    AIT_REG_D   GBL_SW_RST_DIS[2];                          // 0x6918
    AIT_REG_D   GBL_REG_RST_EN[2];                          // 0x6920
        /*-DEFINE-----------------------------------------------------*/
        /* GBL_SW_RST_EN[0x00] 0x6910 */
        #define GBL_RST_VIF0                0x00000001
		#define GBL_RST_VIF1                0x00000002
        #define GBL_RST_VIF2                0x00000004
        #define GBL_RST_RAW_S0              0x00000008
		#define GBL_RST_RAW_S1              0x00000010
		#define GBL_RST_RAW_S2              0x00000020
		#define GBL_RST_RAW_F               0x00000040
		#define GBL_RST_ISP                 0x00000080

        #define GBL_RST_SCAL0               0x00000100
        #define GBL_RST_SCAL1               0x00000200
        #define GBL_RST_SCAL2               0x00000400
        #define GBL_RST_SCAL3               0x00000800
        #define GBL_RST_ICON0               0x00001000
        #define GBL_RST_ICON1               0x00002000
        #define GBL_RST_ICON2               0x00004000
        #define GBL_RST_ICON3               0x00008000

        #define GBL_RST_IBC0                0x00010000
        #define GBL_RST_IBC1                0x00020000
        #define GBL_RST_IBC2                0x00040000
        #define GBL_RST_IBC3                0x00080000
        #define GBL_RST_CCIR                0x00100000
        #define GBL_RST_HDMI                0x00200000
        #define GBL_RST_DSPY                0x00400000
        #define GBL_RST_TV                  0x00800000

        #define GBL_RST_MCI                 0x01000000
        #define GBL_RST_DRAM                0x02000000
        #define GBL_RST_H264                0x04000000
        #define GBL_RST_JPG                 0x08000000
        #define GBL_RST_SD0                 0x10000000
        #define GBL_RST_SD1                 0x20000000
        #define GBL_RST_SD2                 0x40000000
        #define GBL_RST_SD3                 0x80000000
        /* GBL_SW_RST_EN[0x01] 0x6914 */
        #define GBL_RST_CPU_PHL             0x00000001
		#define GBL_RST_PHL                 0x00000002
        #define GBL_RST_AUD                 0x00000004
        #define GBL_RST_GRA                 0x00000008
		#define GBL_RST_DMA_M0              0x00000010
		#define GBL_RST_DMA_M1              0x00000020
		#define GBL_RST_DMA_R0              0x00000040
		#define GBL_RST_DMA_R1              0x00000080

        #define GBL_RST_GPIO                0x00000100
        #define GBL_RST_PWM                 0x00000200
        #define GBL_RST_PSPI                0x00000400
        #define GBL_RST_DBIST               0x00000800
        #define GBL_RST_IRDA                0x00001000
		#define GBL_RST_RTC                 0x00002000
		#define GBL_RST_USB                 0x00004000
		#define GBL_RST_USBPHY              0x00008000

        #define GBL_RST_I2CS                0x00010000
        #define GBL_RST_SM                  0x00020000
        #define GBL_RST_LDC                 0x00040000
        // ++MCR_V2_MP Only
        #define GBL_RST_PLL_JITTER          0x00080000
        #define GBL_RST_SCAL4 	            0x00100000
        #define GBL_RST_ICON4 	            0x00200000
        #define GBL_RST_IBC4 	            0x00400000
        #define GBL_RST_SCAL_SHU            0x00800000
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   _x6928[0x18];

    AIT_REG_D   GBL_GPIO_CFG[4];                            // 0x6940
    
    AIT_REG_D   GBL_PWM_IO_CFG;                             // 0x6950
    AIT_REG_B   					_x6954;
    AIT_REG_B   GBL_SPI_PAD_CFG;                            // 0x6955
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_PSPI0_PAD(_a)           (_a+1)
        #define GBL_PSPI0_PAD_MASK          0x03
        /* ---------------------------------------------------------- */
        /* | PSPI0  | CLK       | CS        | DI        | DO        | */
        /* ---------------------------------------------------------- */
        /* | PAD0   | PI2S_SCK  | PI2S_WS   | PI2S_SDI  | PI2S_SDO  | */
        /* | PAD1   | PBGPIO6   | PBGPIO7   | PBGPIO8   | PBGPIO9   | */
        /* | PAD2   | PLCD8     | PLCD9     | PLCD10    | PLCD12    | */
        /* ---------------------------------------------------------- */
        #define GBL_PSPI1_PAD(_a)           (((_a)+1)<<2)
        #define GBL_PSPI1_PAD_MASK          0x0C
        /* ---------------------------------------------------------- */
        /* | PSPI1  | CLK       | CS        | DI        | DO        | */
        /* ---------------------------------------------------------- */
        /* | PAD0   | PBGPIO18  | PBGPIO19  | PBGPIO20  | PBGPIO21  | */
        /* | PAD1   | PCGPIO24  | PCGPIO25  | PCGPIO26  | PCGPIO27  | */
        /* | PAD2   | PCGPIO6   | PCGPIO7   | PCGPIO8   | PCGPIO9   | */
        /* ---------------------------------------------------------- */
        #define GBL_PSPI2_PAD_EN            0x10
        #define GBL_PSPI2_PAD_MASK          0x30
        /* ---------------------------------------------------------- */
        /* | PSPI2  | CLK       | CS        | DI        | DO        | */
        /* ---------------------------------------------------------- */
        /* | PAD0   | PCGPIO14  | PCGPIO15  | PCGPIO16  | PCGPIO17  | */
        /* ---------------------------------------------------------- */
        #define GBL_SPIS_PAD(_a)            (((_a)+1)<<6)
        #define GBL_SPIS_PAD_MASK           0xC0
        /* ---------------------------------------------------------- */
        /* | SPIS   | CLK       | CS        | DI        | DO        | */
        /* ---------------------------------------------------------- */
        /* | PAD0   | PCGPIO18  | PCGPIO19  | PCGPIO20  | PCGPIO21  | */
        /* | PAD1   | PI2S_SCK  | PI2S_WS   | PI2S_SDI  | PI2S_SDO  | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
    AIT_REG_W   GBL_UART_PAD_CFG;                           // 0x6956
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_UART0_PAD(_a)           (_a+1)
        #define GBL_UART0_PAD_MASK          0x0007
        /* ---------------------------------------------------------- */
        /* | UART0 | TX         | RX          | CTS      | RTS      | */
        /* | PAD0  | PAGPIO0    | PAGPIO6     | Reserved | Reserved | */
        /* | PAD1  | PCGPIO30   | PCGPIO31    | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        #define GBL_UART1_PAD(_a)           (((_a)+1)<<4)
        #define GBL_UART1_PAD_MASK          0x0070
        /* ---------------------------------------------------------- */
        /* | UART1 | TX         | RX          | CTS      | RTS      | */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PCGPIO22   | PCGPIO23    | Reserved | Reserved | */
        /* | PAD1  | PBGPIO4    | PBGPIO5     | Reserved | Reserved | */
        /* | PAD2  | PBGPIO12   | PBGPIO13    | Reserved | Reserved | */
        /* | PAD3  | PCGPIO10   | PCGPIO11    | Reserved | Reserved | */
        /* | PAD4  | PLCD13     | PLCD14      | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        #define GBL_UART2_PAD(_a)           (((_a)+1)<<8)
        #define GBL_UART2_PAD_MASK          0x0700
        /* ---------------------------------------------------------- */
        /* | UART2 | TX         | RX          | CTS      | RTS      | */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PCGPIO26   | PCGPIO27    | Reserved | Reserved | */
        /* | PAD1  | PI2S_WS    | PI2S_SDO    | Reserved | Reserved | */
        /* | PAD2  | PBGPIO10   | PBGPIO11    | Reserved | Reserved | */
        /* | PAD3  | PLCD11     | PLCD15      | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        #define GBL_UART3_PAD(_a)           (((_a)+1)<<12)
        #define GBL_UART3_PAD_MASK          0x7000
        /* ---------------------------------------------------------- */
        /* | UART3 | TX         | RX          | CTS      | RTS      | */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PCGPIO28   | PCGPIO29    | PCGPIO26 | PCGPIO27 | */
        /* | PAD1  | PSNR_SEN_1 | PSNR_MCLK_1 | Reserved | Reserved | */
        /* | PAD2  | PLCD6      | PLCD7       | Reserved | Reserved | */
        /* | PAD3  | PLCD8      | PLCD9       | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_VIF_IGBT_CFG;                           // 0x6958
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_VIF0_IGBT_PAD(_a)		(_a + 1)
        #define GBL_VIF0_IGBT_MASK			0x07
        #define GBL_VIF1_IGBT_PAD(_a)		(_a + 1)<<4
        #define GBL_VIF1_IGBT_MASK			0x70
        /* ---------------------------------------------------------- */
        /* | IGBT  | IGBT0      | IGBT1	      |						| */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PAGPIO4    | PAGPIO5     | Reserved | Reserved | */
        /* | PAD1  | PSNR_VSYNC | PSNR_HSYNC  | Reserved | Reserved | */
        /* | PAD2  | PSNR_SEN_1 | PSNR_MCLK_1 | Reserved | Reserved | */
        /* | PAD3  | PBGPIO16   | PBGPIO17    | Reserved | Reserved | */
        /* | PAD4  | PCGPIO20   | PCGPIO21    | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_OTG_IRDA_CFG;                           // 0x6959
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_IRDA_PAD(_a)			(_a + 1)<<4
        #define GBL_IRDA_MASK				0x70
        /* ---------------------------------------------------------- */
        /* | IRDA  | IRDA       |             |						| */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PI2S_MCLK  | Reserved    | Reserved | Reserved | */
        /* | PAD1  | PSNR_PCLK_0| Reserved    | Reserved | Reserved | */
        /* | PAD2  | PCGPIO11   | Reserved    | Reserved | Reserved | */
        /* | PAD3  | PBGPIO4    | Reserved    | Reserved | Reserved | */
        /* | PAD4  | PCGPIO30   | Reserved    | Reserved | Reserved | */
        /* | PAD5  | PDGPIO5    | Reserved    | Reserved | Reserved | */
        /* | PAD6  | RX2_DA0P   | Reserved    | Reserved | Reserved | */
        /* ---------------------------------------------------------- */
        #define GBL_USB_OTG_PAD(_a)			(_a + 1)
        #define GBL_USB_OTG_MASK			0x07
        /* ---------------------------------------------------------- */
        /* | OTG   | IDPULLUP   | CHRG_VBUG   | DRV_VBUS | DIS_VBUS | */
        /* ---------------------------------------------------------- */
        /* | PAD0  | PCGPIO18   | PCGPIO19    | PCGPIO20 | PCGPIO21 | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_I2S_DMIC_CFG;                           // 0x695A
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_I2S0_PAD_EN             0x01
        #define GBL_I2S1_PAD(_a)            ((_a+1)<<2)
        #define GBL_I2S1_PAD_MASK           0x0C
        /* ---------------------------------------------------------- */
        /* | I2S1 | BCK     | LRCK    | SDI     | HCK     | SDO     | */
        /* ---------------------------------------------------------- */
        /* | PAD0 | PLCD8   | PLCD9   | PLCD10  | PLCD11  | PLCD12  | */
        /* | PAD1 | PCGPIO6 | PCGPIO7 | PCGPIO8 | PCGPIO9 | PCGPIO10| */
        /* ---------------------------------------------------------- */
        #define GBL_DMIC_PAD_DISABLE   (0<<4)		
        #define GBL_DMIC_PAD(_a)            ((_a+1)<<4)
        #define GBL_DMIC_PAD_MASK        (3<<4)
        /* ---------------------------------------------------------- */
        /* |  DMIC      |  DMIC_CLK           |  DMIC_DATA          | */
        /* ---------------------------------------------------------- */
        /* |  PAD0      |  PI2S_SCK           |  PI2S_SDI           | */
        /* |  PAD1      |  PLCD8              |  PLCD10             | */
        /* |  PAD2      |  PCGPIO6            |  PCGPIO8            | */
        /* ---------------------------------------------------------- */
        // ++MCR_V2_MP Only
        #define GBL_I2S2_PAD(_a)            (_a + 1)<<6
        #define GBL_I2S2_PAD_MASK           0xC0
        /* --------------------------------------------------------------- */
        /* | I2S1 | BCK      | LRCK     | SDI      | HCK      | SDO      | */
        /* --------------------------------------------------------------- */
        /* | PAD0 | PBGPIO11 | PBGPIO12 | PBGPIO13 | PBGPIO14 | PBGPIO15 | */
        /* | PAD1 | PCGPIO13 | PCGPIO14 | PCGPIO15 | PCGPIO16 | PCGPIO17 | */
        /* | PAD2 | PLCD_WE_ | PLCD_A0  | PLCD_RD_ | PLCD1_CS | PLCD_GPIO| */
        /* --------------------------------------------------------------- */
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_SD_PAD_CFG;                             // 0x695B
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_SD0_PAD_EN              0x01
        #define GBL_SD1_PAD(_a)             ((_a+1)<<2)
        #define GBL_SD1_PAD_MASK            0x0C
        /* ---------------------------------------------------------- */
        /* | SD1  | SD_CLK   | SD_CMD   | SD_DATA0~3  | SD_DATA4~7  | */
        /* ---------------------------------------------------------- */
        /* | PAD0 | PCGPIO6  | PCGPIO7  | PCGPIO8~11  | Reserved    | */
        /* | PAD1 | PBGPIO16 | PBGPIO17 | PBGPIO18~21 | PBGPIO12~15 | */
        /* ---------------------------------------------------------- */
        #define GBL_SD2_PAD_EN              0x10
        /*------------------------------------------------------------*/
    AIT_REG_W   GBL_I2CM_PAD_CFG;                           // 0x695C
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_I2CM0_PAD(_a)           (_a+1)
        #define GBL_I2CM0_PAD_MASK          0x0007
        /* ---------------------------------------------------------- */
        /* |  I2CM0     |  I2CM_SCK           |  I2CM_SDA           | */
        /* ---------------------------------------------------------- */
        /* |  PAD0      |  PSNR_SCK           |  PSNR_SDA           | */
        /* |  PAD1      |  PSNR_VSYNC         |  PSNR_HSYNC         | */
        /* |  PAD2      |  PHI2C_SCL          |  PHI2C_SDA          | */
        /* ---------------------------------------------------------- */
        #define GBL_I2CM1_PAD(_a)           ((_a+1)<<4)
        #define GBL_I2CM1_PAD_MASK          0x0070
        /* ---------------------------------------------------------- */
        /* |  I2CM1     |  I2CM_SCK           |  I2CM_SDA           | */
        /* ---------------------------------------------------------- */
        /* |  PAD0      |  PCGPIO7            |  PCGPIO9            | */
        /* |  PAD1      |  PHI2C_SCL          |  PHI2C_SDA          | */
        /* |  PAD2      |  PCGPIO18           |  PCGPIO19           | */
        /* |  PAD3      |  PBGPIO14           |  PBGPIO15           | */
        /* ---------------------------------------------------------- */
        #define GBL_I2CM2_PAD(_a)           ((_a+1)<<8)
        #define GBL_I2CM2_PAD_MASK          0x0700
        /* ---------------------------------------------------------- */
        /* |  I2CM2     |  I2CM_SCK           |  I2CM_SDA           | */
        /* ---------------------------------------------------------- */
        /* |  PAD0      |  PCGPIO24           |  PCGPIO25           | */
        /* |  PAD1      |  PLCD13             |  PLCD14             | */
        /* |  PAD2      |  PI2S_SDI           |  PI2S_MCLK          | */
        /* ---------------------------------------------------------- */
        #define GBL_I2CM3_PAD(_a)           ((_a+1)<<12)
        #define GBL_I2CM3_PAD_MASK          0x7000
        /* ---------------------------------------------------------- */
        /* |  I2CM3     |  I2CM_SCK           |  I2CM_SDA           | */
        /* ---------------------------------------------------------- */
        /* |  PAD0      |  PHDMI_SCL          |  PHDMI_SDA          | */
        /* |  PAD1      |  PLCD2_CS_          |  PLCD_FLM           | */
        /* |  PAD2      |  PSNR_VSYNC         |  PSNR_HSYNC         | */
        /* |  PAD3      |  PBGPIO6            |  PBGPIO7            | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_MISC_IO_CFG;                            // 0x695E
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_CCIR_PAD_EN             0x01
        #define GBL_SIF_PAD_EN              0x02
        #define GBL_NAND_PAD_EN             0x04
        #define GBL_VIF0_GPIO_EN            0x08
        #define GBL_VIF1_GPIO_EN            0x10
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_LCD_PAD_CFG;                            // 0x695F
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_LCD_PAD_EN              0x01
        #define GBL_LCD_RGB_SPI_DIS         0x00
        #define GBL_LCD_RGB_SPI_PAD0        0x02
        #define GBL_LCD_RGB_SPI_PAD1        0x04
        #define GBL_LCD_RGB_SPI_MASK        0x06
        #define GBL_LCD_SPI_EN				0x08
        /* ---------------------------------------------------------- */
        /* | RGB SPI  |   LCD_SCL     | LCD_SDA       | LCD_CS      | */
        /* ---------------------------------------------------------- */
        /* | PAD0     |   PLCD2_CS_   | LCD_FLM       | LCD_GPIO    | */
        /* | PAD1     |   PCGPIO28    | PCGPIO29      | PCGPIO30    | */
        /* ---------------------------------------------------------- */
        /*------------------------------------------------------------*/
        
    AIT_REG_B   GBL_PROBE_IO_CFG;                           // 0x6960
    AIT_REG_B   GBL_HOST_INT_PAD_CFG;                       // 0x6961
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_HOST_INT_PAD(_a)        (1<<(_a))
        #define GBL_HOST_INT_PAD_MASK       0x1F
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_JTAG_CFG;                               // 0x6962
    AIT_REG_B   GBL_PLL_BIST_CFG;                           // 0x6963
    	/*-DEFINE-----------------------------------------------------*/
    	// ++MCR_V2_MP Only
    	#define GBL_PLL_BIST(_a)            (_a + 1)
        #define GBL_PLL_BIST_MASK           0x03
        /* ------------------------------------------------------------------------- */
        /* | PLL  | PBGPIO11       | PBGPIO12  | PBGPIO13~17    | PBGPIO18~21      | */
        /* ------------------------------------------------------------------------- */
        /* | PLL0 | PLL0_BIST_FAIL | PLL0_CKD5 | PLL0_BITS[4:0] | PLL0_STATE[3:0]  | */
        /* | PLL1 | PLL1_BIST_FAIL | PLL1_CKD5 | PLL1_BITS[4:0] | PLL1_STATE[3:0]  | */
        /* | PLL2 | PLL2_BIST_FAIL | PLL2_CKD5 | PLL2_BITS[4:0] | PLL2_STATE[3:0]  | */
        /* ------------------------------------------------------------------------- */
    	#define GBL_PLL_JITTER_CAL_EN       0x04
    	// --MCR_V2_MP Only
    	/*------------------------------------------------------------*/
		#define GBL_JTAG_DBG_SRST_MODE 		0x08
		#define GBL_JTAG_DBG_EN				0x04
		#define GBL_JTAG_CHAIN_MODE 		0x02
		#define GBL_JTAG_MULTI_FUNC_SEL 	0x01
    AIT_REG_B   					_x6964[0x1C];

    AIT_REG_D   GBL_DBIST_ST_ADDR;                          // 0x6980
    AIT_REG_D   GBL_DBIST_END_ADDR;                         // 0x6984
    AIT_REG_D   GBL_BIST_MUX[2];                            // 0x6988
    AIT_REG_D   GBL_BIST_SEL[4];                            // 0x6990
    AIT_REG_W   GBL_BIST_CTL;                               // 0x69A0
    AIT_REG_B   GBL_DBIST_OFST_CFG;                         // 0x69A2
    AIT_REG_B   _x69A3[0x1D];

    AIT_REG_B   GBL_WAKEUP_INT_HOST_EN;                     // 0x69C0
    AIT_REG_B   GBL_WAKEUP_INT_HOST_SR;                     // 0x69C1
    AIT_REG_B   GBL_WAKEUP_INT_CPU_EN;                      // 0x69C2
    AIT_REG_B   GBL_WAKEUP_INT_CPU_SR;                      // 0x69C3
        /*-DEFINE-----------------------------------------------------*/
        #define GPIO_WAKE_INT               0x01
        #define USB_RESUME_WAKE_INT         0x02
        #define HDMI_PLUG_WAKE_INT          0x04
        #define TV_DETECT_WAKE_INT          0x08
        // ++MCR_V2_MP Only
        #define RTC2DIG_CNT_VALID_INT       0x10
        #define RTC2DIG_VALID_INT           0x20
        #define RTC2DIG_INT                 0x40
        #define RTC2DIG_ALARM_EN_INT        0x80
        #define RTC2DIG_INT_MASK            0xF0
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_WAKEUP_CFG;                             // 0x69C4
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_WAKE_BY_USB             0x02
        #define GBL_WAKE_BY_HDMI            0x04
        #define GBL_WAKE_BY_TV              0x08
        // ++MCR_V2_MP Only
        #define GBL_WAKE_BY_RTC_CNT_VALID   0x10
        #define GBL_WAKE_BY_RTC_VALID       0x20
        #define GBL_WAKE_BY_RTC_INT         0x40
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   _x69C5;
    AIT_REG_W   GBL_LCD_BYPASS_CTL;                         // 0x69C6
        /*-DEFINE-----------------------------------------------------*/
        #define BYPASS_GATE_PMCLK           0x0000
        #define BYPASS_KEEP_PMCLK           0x0001
        #define BYPASS_PWR_OFF_PLL          0x0002
        #define BYPASS_AUTO_PWR_OFF_PLL     0x0002		
        #define BYPASS_TO_SELF_SLEEP        0x0004
        #define BYPASS_PWR_OFF_XTAL         0x0008
        #define XTAL_START_UP_TIME_MASK		0x00F0
        #define PLL_SWITCH_TIME_MASK		0x0700
        /*------------------------------------------------------------*/
    AIT_REG_B	GBL_LCD_BYPASS_FSM_STATUS;					// 0x69C8
    AIT_REG_B   					_x69C9[0x17];

    AIT_REG_B   GBL_MISC_CTL;                               // 0x69E0
        /*-DEFINE-----------------------------------------------------*/
        #define CPU_AUTO_BOOT_DIS			0x01
        #define	THREE_STAGE_SYNC            0x00
        #define	TWO_STAGE_SYNC              0x02
        #define ROM_ACCESS_NORMAL_MODE		0x04
        #define OPR_CLK_KEEP_ON             0x08
        #define HOST_DOWNLOAD_EN			0x20
        #define DRAM_CKE_PWR_DOWN_LOW       0x40
        #define JTAG_DOWNLOAD_4B_TO_TCM		0x80
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_MISC_CTL1;                             // 0x69E1 
    	/*-DEFINE-----------------------------------------------------*/                                                  
        // ++MCR_V2_MP Only
        #define DRAM3_STACK_SLIDE           0x00
        #define DRAM3_STACK_OPT			    0x01 
        #define CPUA_SYNC_HS_EN             0x10
        #define CPUB_SYNC_HS_EN             0x20
        // --MCR_V2_MP Only
        /*------------------------------------------------------------*/
    AIT_REG_B   					_x69E2[0x03];
    AIT_REG_B   GBL_SW_RESOL_SEL;                           // 0x69E5
        /*-DEFINE-----------------------------------------------------*/
        #define	GBL_SENSOR_3M               0xF3
        #define	GBL_SENSOR_5M               0xF5
        #define	GBL_SENSOR_8M               0xF8
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_SIF_REBOOT_CTL;                         // 0x69E6
        /*-DEFINE-----------------------------------------------------*/
        #define SIF_REBOOT_EN               0x18
        /*------------------------------------------------------------*/
    AIT_REG_B   _x69E7[0x09];

    AIT_REG_W   GBL_BOOT_STRAP_CTL;                         // 0x69F0
        /*-DEFINE-----------------------------------------------------*/
        #define I2C_SLAVE_ADDR_03			0x0000	//[Ref:PAGPIO1]
        #define I2C_SLAVE_ADDR_04			0x0002	//[Ref:PAGPIO1]
        #define BOOT_UP_BY_CPUA				0x0000	//[Ref:PAGPIO2]
        #define BOOT_UP_BY_CPUB				0x0004	//[Ref:PAGPIO2]
        #define AUTO_SWITCH_PLL_EN			0x0000	//[Ref:PAGPIO3]
        #define AUTO_SWITCH_PLL_DIS			0x0008	//[Ref:PAGPIO3]
        #define JTAG_CHAIN_CPUA_SET0        0x0000
        #define JTAG_CHAIN_CPUB_SET0        0x0010
        #define JTAG_CHAIN_MODE_EN          0x0000
        #define JTAG_CHAIN_MODE_DIS         0x0020
        #define ROM_BOOT_MODE               0x0040
        #define	ARM_JTAG_MODE_EN            0x0080
        #define MOD_BOOT_STRAP_EN           0x8000
        #define MODIFY_BOOT_STRAP_EN        0x8000		
        /*------------------------------------------------------------*/
    AIT_REG_B   _x69F2[0x08];
    AIT_REG_B   GBL_CHIP_VER;                               // 0x69FA
    	/*-DEFINE-----------------------------------------------------*/
        #define GBL_CHIP_VER_MASK           0x07
        /*------------------------------------------------------------*/
    AIT_REG_B   GBL_ID_RD_CTL;                              // 0x69FB
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_PROJECT_ID_RD_EN        0x80
        /*------------------------------------------------------------*/
    AIT_REG_W   GBL_SECO_VER;                               // 0x69FC
    AIT_REG_W   GBL_PROJECT_ID;                             // 0x69FE
} AITS_GBL, *AITPS_GBL;

// **********************************
//   CORE structure (0x8000 3A00)
// **********************************
typedef struct _AITS_CORE {
    AIT_REG_B   CORE_A_CFG;                                 // 0x3A00
        /*-DEFINE-----------------------------------------------------*/
        #define CPU_CLK_SYNC                0x00
        #define CPU_CLK_ASYNC               0x01
        #define CPU_RST_EN                  0x02
        #define CPU_CLK_FAST_THAN_GBL       0x00
        #define CPU_CLK_SLOW_THAN_GBL       0x04
        #define CPU_SLEEP_MODE_EN			0x08
        #define CPU_WFI_GATE_CLK            0x40
        /*------------------------------------------------------------*/
    AIT_REG_B   CORE_A_TCM_SIZE;                            // 0x3A01
        /*-DEFINE-----------------------------------------------------*/
        #define CPU_ITCM_SIZE_MASK         	0x0F
        #define CPU_DTCM_SIZE_MASK      	0xF0
        /*------------------------------------------------------------*/
    AIT_REG_B   CORE_A_WD_CFG;                              // 0x3A02
        /*-DEFINE-----------------------------------------------------*/
        #define WD_RST_CPU                  0x00
        #define WD_RST_ALL_MDL              0x01
        #define WD_RST_ALL_CPU              0x02
        #define WD_RST_ALL_MDL_OPR          0x03
        #define WD_RST_ALL_CPU_MDL          0x05
        #define WD_RST_ALL_CPU_MDL_OPR      0x07
        #define HOST_RST_CPU_EN             0x08
        #define WD_RST_EXCL_SIF             0x10
        #define WD_RST_EXCL_SM              0x20
        /*------------------------------------------------------------*/
    AIT_REG_B   _x3A03;
    AIT_REG_D   CORE_A_SRAM_CFG;                            // 0x3A04
    AIT_REG_B   CORE_B_CFG;                                 // 0x3A08
    AIT_REG_B   CORE_B_TCM_SIZE;                            // 0x3A09
    AIT_REG_B   CORE_B_WD_CFG;                              // 0x3A0A
    AIT_REG_B   _x3A0B;
    AIT_REG_D   CORE_B_SRAM_CFG;                            // 0x3A0C

    AIT_REG_D   CORE_I_PREFETCH_ST_ADDR0;                   // 0x3A10
    AIT_REG_D   CORE_I_PREFETCH_END_ADDR0;                  // 0x3A14
    AIT_REG_D   CORE_I_PREFETCH_ST_ADDR1;                   // 0x3A18
    AIT_REG_D   CORE_I_PREFETCH_END_ADDR1;                  // 0x3A1C
    AIT_REG_D   CORE_I_PREFETCH_ST_ADDR2;                   // 0x3A20
    AIT_REG_D   CORE_I_PREFETCH_END_ADDR2;                  // 0x3A24
    AIT_REG_D   CORE_D_PREFETCH_ST_ADDR0;                   // 0x3A28
    AIT_REG_D   CORE_D_PREFETCH_END_ADDR0;                  // 0x3A2C
    AIT_REG_D   CORE_D_PREFETCH_ST_ADDR1;                   // 0x3A30
    AIT_REG_D   CORE_D_PREFETCH_END_ADDR1;                  // 0x3A34
    AIT_REG_D   CORE_D_PREFETCH_ST_ADDR2;                   // 0x3A38
    AIT_REG_D   CORE_D_PREFETCH_END_ADDR2;                  // 0x3A3C

    AIT_REG_D   CORE_MISC_CFG;                              // 0x3A40
        /*-DEFINE-----------------------------------------------------*/
        #define CPU_CORE_OFF_EN             0x0100
        #define CPU_A2B_DNLD_EN             0x0200
        #define CPU_B2A_DNLD_EN             0x0400
        /*------------------------------------------------------------*/
} AITS_CORE, *AITPS_CORE;

//-----------------------------
// Core structure (0x80003A60)
//-----------------------------
typedef struct _AITS_PLL_JITTER_CAL {
	// ++MCR_V2_MP Only
    AIT_REG_B   PLL_JITCAL_EN;                              // 0x3A60
        /*-DEFINE-----------------------------------------------------*/
        #define PLL_JITCAL_DIS              0x00
        #define PLL_JITCAL_PLL0             0x01
        #define PLL_JITCAL_PLL1             0x02
        #define PLL_JITCAL_PLL2             0x03
        /*------------------------------------------------------------*/
    AIT_REG_B   PLL_JITCAL_MODE_SEL;                        // 0x3A61  
    	/*-DEFINE-----------------------------------------------------*/
        #define PLL_JITCAL_MEAN_MODE        0x01
        #define PLL_JITCAL_P2P_MODE         0x02
        #define PLL_JITCAL_STD_MODE         0x04
        #define PLL_JITCAL_STD_DM_EN        0x08
        /*------------------------------------------------------------*/ 
    AIT_REG_W   PLL_JITCAL_STATE_VAL;                       // 0x3A62  
    AIT_REG_B   PLL_JITCAL_BITS_VAL;                        // 0x3A64      
    AIT_REG_B   PLL_JITCAL_SMP_CNT_OFS;                     // 0x3A65      
    AIT_REG_W   PLL_JITCAL_FAIL_CNT_TH;                     // 0x3A66   
    AIT_REG_W   PLL_JITCAL_P2P_TH;                          // 0x3A68 
    AIT_REG_W   PLL_JITCAL_STD_FIX_MEAN;                    // 0x3A6A   
    AIT_REG_D   PLL_JITCAL_STD_TH;                          // 0x3A6C
    	/*-DEFINE-----------------------------------------------------*/
        #define PLL_JITCAL_STD_TH_MASK      0x0FFF
        /*------------------------------------------------------------*/   
    AIT_REG_B   PLL_JITCAL_STATUS;                          // 0x3A70
    AIT_REG_B   PLL_JITCAL_SSTD_VAL_LOW0;                   // 0x3A71
	AIT_REG_B   PLL_JITCAL_SSTD_VAL_LOW1;                   // 0x3A72
    AIT_REG_B   PLL_JITCAL_SSTD_VAL_HIGH;                   // 0x3A73
    AIT_REG_W   PLL_JITCAL_MEAN_VAL;                        // 0x3A74
    AIT_REG_W   PLL_JITCAL_P2P_VAL;                         // 0x3A76
    AIT_REG_W   PLL_JITCAL_FAIL_CNT;                        // 0x3A78         
    // --MCR_V2_MP Only    
} AITS_PLL_JITTER_CAL, *AITPS_PLL_JITTER_CAL;

//--------------------------------
// RTC structure (0x80003A90)
//--------------------------------
typedef struct _AITS_RTC {
    AIT_REG_W   DIG2RTC_CTL;                                // 0x3A90
        /*-DEFINE-----------------------------------------------------*/
        #define RTC_RST                     0x0000
        #define RTC_EN                      0x0001
        #define RTC_BASE_WR                 0x0002
        #define RTC_BASE_RD                 0x0004
        #define RTC_CNT_RST                 0x0008
        #define RTC_ALARM_WR                0x0010
        #define RTC_CNT_RD                  0x0100
        #define RTC_ALARM_RD                0x0200
        #define RTC_GAIN_CTL_EN             0x0400// for analog debug
        #define RTC_ALARM_INT_EN            0x0800
        #define RTC_ALARM_INT_CLR           0x1000
        #define RTC_BYPASS                  0x2000// for analog debug
        #define RTC_CLK_OUT_EN              0x4000// for analog debug
        #define RTC_TMD                     0x8000// for analog debug
         /*------------------------------------------------------------*/
    AIT_REG_W   RTC_SEQ;                                    // 0x3A92
        /*-DEFINE-----------------------------------------------------*/
        // 1->2->3->4->5->2->1
        #define RTC_EN_SEQ_1                0x0000
        #define RTC_EN_SEQ_2                0x0100
        #define RTC_EN_SEQ_3                0x0300
        #define RTC_EN_SEQ_4                0x0700
        #define RTC_EN_SEQ_5                0x0500
        /*------------------------------------------------------------*/
    AIT_REG_D   RTC_WR_DATA;                                // 0x3A94
    AIT_REG_D   RTC2DIG_CTL;                                // 0x3A98
        /*-DEFINE-----------------------------------------------------*/
        #define RTC_ALARM_EN_ST             0x01
        #define RTC_ALARM_INT_ST            0x02
        #define RTC_CNT_VALID_ST            0x04
        #define RTC_VALID_ST                0x08
        /*------------------------------------------------------------*/
    AIT_REG_D   RTC_RD_DATA;                                // 0x3A9C
        /*-DEFINE-----------------------------------------------------*/
        #define RTC_CNT_BUSY                0x80000000
        /*------------------------------------------------------------*/
} AITS_RTC, *AITPS_RTC;

// **********************************
//  HDMI PHY structure (0x8000 3AA0)
// **********************************
typedef struct _AITS_HDMI_PHY {
    AIT_REG_B   HDMI_PHY_TMDS_CTL[7];                       // 0x3AA0
        /*-DEFINE-----------------------------------------------------*/
        /* HDMI_PHY_TMDS_CTL[0] 0x3AA0 */
        #define TDMS_CURRENT_SRC_EN         0x10
        #define TDMS_CURRENT_SRC_DIS        0x00
        #define HDMI_PHY_DATA_OUTPUT_EN     0x20
        #define HDMI_PHY_DATA_OUTPUT_DIS    0x00
        #define TDMS_BUF_POWER_DOWN         0x80
        #define TDMS_BUF_POWER_UP           0x00
        /* HDMI_PHY_TMDS_CTL[1] 0x3AA1 */
        #define CK_PN_SWAP_EN               0x01
        #define CK_PN_SWAP_DIS              0x00
        #define CH0_PN_SWAP_EN              0x02
        #define CH0_PN_SWAP_DIS             0x00
        #define CH1_PN_SWAP_EN              0x04
        #define CH1_PN_SWAP_DIS             0x00
        #define CH2_PN_SWAP_EN              0x08
        #define CH2_PN_SWAP_DIS             0x00
        #define HDMI_PHY_RISING_TRIGGER     0x10
        #define HDMI_PHY_FALLING_TRIGGER    0x00
        #define TEST_PARALLEL_DATA_EN       0x20
        #define TEST_PARALLEL_DATA_DIS      0x00
        /* HDMI_PHY_TMDS_CTL[3] 0x3AA3 */
        #define TDMS_TEST_PATTERN_DIS       0x00
        #define TDMS_TEST_PATTERN_EN        0x10
		/*------------------------------------------------------------*/
    AIT_REG_B   HDMI_PHY_LDO_CTL;                           // 0x3AA7
    AIT_REG_D   HDMI_PHY_BIST_CTL;                          // 0x3AA8
        /*-DEFINE-----------------------------------------------------*/
        #define HDMI_LOOPBACK_TEST_DIS      0x00000000
        #define HDMI_LOOPBACK_TEST_EN       0x00000004
        /*------------------------------------------------------------*/
    AIT_REG_W   HDMI_PHY_BANDGAP_CTL;                       // 0x3AAC
        /*-DEFINE-----------------------------------------------------*/
        #define BS_PWR_DOWN                 0x0040
        #define BS_PWR_UP                   0x0000
        #define BG_PWR_DOWN                 0x0080
        #define BG_PWR_UP                   0x0000
        #define BG_OUTPUT                   0x0000
        #define R_DIVIDER_OUTPUT            0x8000
        /*------------------------------------------------------------*/
    AIT_REG_B   HDMI_PHY_PLL_CTL[5];                        // 0x3AAE
        /*-DEFINE-----------------------------------------------------*/
        /* HDMI_PHY_PLL_CTL[0] 0x3AAE */
        #define LOCK_DETECTION              0x00
        #define REPORT_LOCK_STATE           0x01
        #define PERFECT_CP_CURRENT_EN       0x02
        #define PERFECT_CP_CURRENT_DIS      0x00
        #define VCO_TEST_EN                 0x10
        #define VCO_TEST_DIS                0x00
        #define FALLING_TRIGGER_TEST        0x00
        #define RISING_TRIGGER_TEST         0x20
        #define HDMI_PHY_RST_PLL            0x40
        #define HDMI_PHY_PWR_DOWN_PLL       0x80
        /* HDMI_PHY_PLL_CTL[1] 0x3AAF */
        #define HDMI_PHY_PHASE0             0x00
        #define HDMI_PHY_PHASE1             0x01
        #define HDMI_PHY_PHASE2             0x02
		/*------------------------------------------------------------*/
    AIT_REG_B   _x3AB3;
    AIT_REG_D   HDMI_PHY_DATA_LANE_SAMPLE;                  // 0x3AB4
    AIT_REG_D   HDMI_PHY_SR;                                // 0x3AB8
    AIT_REG_B   HDMI_PHY_INT_CPU_EN;                        // 0x3ABC
    AIT_REG_B   HDMI_PHY_INT_CPU_SR;                        // 0x3ABD
    AIT_REG_B   HDMI_PHY_INT_HOST_EN;                       // 0x3ABE
    AIT_REG_B   HDMI_PHY_INT_HOST_SR;                       // 0x3AEF
        /*-DEFINE-----------------------------------------------------*/
        #define HDMI_PLUGIN_INT             0x01
        /*------------------------------------------------------------*/
} AITS_HDMI_PHY, *AITPS_HDMI_PHY;

// **********************************
//   CPU share register (0x8000 3AC0)
// **********************************
typedef struct _AITS_CPU_SHARE {
    AIT_REG_B   CPU_SAHRE_REG[64];                          // 0x3AC0 ~ 0x3AFF
} AITS_CPU_SHARE, *AITPS_CPU_SHARE;

#endif // (CHIP == MCR_V2)

// **********************************
//   GLOBAL structure (0x8000 6900)
// **********************************
#if (CHIP == VSN_V3) || (CHIP == VSN_V2)
typedef struct _AITS_GBL {
	#if (CHIP == VSN_V3)
	AIT_REG_D	GBL_CPU_SMOOTH_DIV_CTL;						//Register Space 2, 0x00
	AIT_REG_B			_0x04[0xC];							//Register Space 2, 0x04~0x0F reserved
	AIT_REG_B	GBL_DPLL2_M;								//Register Space 2, 0x10
	AIT_REG_B	GBL_DPLL2_N;								//Register Space 2, 0x11
	AIT_REG_B	GBL_DPLL2_PWR;								//Register Space 2, 0x12
	AIT_REG_B	GBL_DPLL2_CFG2;								//Register Space 2, 0x13
	AIT_REG_B	GBL_DPLL2_CFG3;								//Register Space 2, 0x14
	AIT_REG_B	GBL_DPLL2_CFG4;								//Register Space 2, 0x15
	AIT_REG_B	GBL_DPLL2_SRC_SEL;							//Register Space 2, 0x16
	AIT_REG_B	GBL_DPLL2_CFG5;								//Register Space 2, 0x17
	AIT_REG_B	GBL_IO_CTL14;								//Register Space 2, 0x18
	AIT_REG_B			_0x19[0x27];						//Register Space 2, 0x19~0x3F reserved
	AIT_REG_B	GBL_IO_CFG_PSTM_T32[0xB];					//Register Space 2, 0x40
	AIT_REG_B			_0x4B[0x85];						//Register Space 2, 0x4B~0xCF reserved
	AIT_REG_B	GBL_IO_CFG_PCGPIO6[0xC];					//Register Space 2, 0xD0
	AIT_REG_B			_0xDC[0x4];							//Register Space 2, 0xDC~0xDF reserved
	AIT_REG_B	GBL_IO_CFG_PDGPIO[0x5];						//Register Space 2, 0xE0~0xE4
	
	AIT_REG_B			_0x5E00[0xB1B];						//Register Space offset
	#endif
	AIT_REG_B   GBL_CHIP_CFG;								//0x00
		/*-DEFINE-----------------------------------------------------*/
		#if (CHIP == VSN_V2)
		#define	ARM_JTAG_MODE_EN		   	0x3F
		#endif
		#if (CHIP == VSN_V3)
		#define	ARM_JTAG_MODE_EN		   	0x80
		#endif
		#define ROM_BOOT_MODE				0x40
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_CHIP_CTL;								//0x01
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_WD_RST_ALL_MODULE		0x01
		#define	MOD_BOOT_STRAP_EN		   	0x80
		
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_SYS_CLK_CTL;							//0x02
		/*-DEFINE-----------------------------------------------------*/
		#define	USB_CLK_SEL_EXT0		0x80
		#define DRAM_ASYNC_EN			0x40
        /*------------------------------------------------------------*/
	AIT_REG_B   GBL_CLK_DIS0; 								//0x03
		/*-DEFINE-----------------------------------------------------*/
		#define	CLK_EN_ALL		   				0x0000
		#define GBL_CLK_MCI_DIS					0x01
		#define GBL_CLK_CPU_DIS                 0x02
		#define GBL_CLK_SCAL_DIS                0x04
		#define GBL_CLK_JPG_DIS                 0x08
		#define GBL_CLK_AUD_DIS                 0x10
		#define GBL_CLK_VI_DIS                  0x20
		#define GBL_CLK_ISP_DIS                 0x40
		#define GBL_CLK_GPIO_DIS                0x80
        /*------------------------------------------------------------*/		
	AIT_REG_B   GBL_CLK_PATH_CTL;							//0x04
		/*-DEFINE-----------------------------------------------------*/
		#define	BYPASS_DPLL0		   	0x04
		#define	BYPASS_DPLL1		   	0x08
		#if (CHIP == VSN_V3)
		#define	BYPASS_DPLL2		   	0x10
		#endif
		#define USB_PHY_PLL_INPUT_EN	0x20
		#define	USB_CLK_SEL_EXT1		0x40
        /*------------------------------------------------------------*/
	AIT_REG_B	GBL_CLK_DIV;								//0x05
		/*-DEFINE-----------------------------------------------------*/
		#define PLL_SEL_PLL_OFFSET			0x06
		#define PLL_POST_DIV_EN				0x20
		#define PLL_CLK_MUX					0xC0
		#if (CHIP == VSN_V2)
		#define PLL_CLK_DIV_MCLK			0x80
		#endif
		#if (CHIP == VSN_V3)
		#define PLL_CLK_DIV_MCLK			0xC0
		#endif
		#define PLL_DIVIDE_MASK				0x1f
		#define	GROUP_CLK_DIV(_a)			(_a)
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_USBPHY_CLK_DIV;							//0x06	
	AIT_REG_B   GBL_PLL_WAIT_CNT;							//0x07
	AIT_REG_W	GBL_RST_REG_EN;								//0x08
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_REG_CPU_PEREF_RST       0x0001
		#define GBL_REG_VI_RST            	0x0002
		#define GBL_REG_ISP_RST            	0x0004
		#define GBL_REG_JPG_RST             0x0008
		#define GBL_REG_SCAL_RST          	0x0010
		#define GBL_REG_GPIO_RST            0x0020
		#define GBL_REG_AUDIO_RST           0x0040
		#define GBL_REG_DRAM_RST            0x0080
		
		#define GBL_REG_MCI_RST            	0x0100
		#define GBL_REG_RAW_RST            	0x0200
		#define GBL_REG_DMA_RST            	0x0400
		#define GBL_REG_USB_RST            	0x1000
		#define GBL_REG_H264_RST            0x2000
		#define GBL_REG_ICON_IBC_RST        0x4000
		#define GBL_REG_GRA_RST            	0x8000
		/*------------------------------------------------------------*/
	AIT_REG_W	GBL_RST_CTL01;								//0x0A
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_CPU_PEREF_RST           0x0001
		#define GBL_VI_RST           		0x0002
		#define GBL_ISP_RST           		0x0004
		#define GBL_JPG_RST           		0x0008
		#if (CHIP == VSN_V2)
		#define GBL_SCAL_RST           		0x0010
		#endif
		#if (CHIP == VSN_V3)
		#define GBL_SCAL0_RST           	0x0010
		#endif
		#define GBL_GPIO_RST           		0x0020
		#define GBL_AUD_PEREF_RST           0x0040
		#define GBL_DRAM_RST           		0x0080
		
		#define GBL_MCI_RST           		0x0100
		#define GBL_RAW_RST           		0x0200
		#define GBL_DMA_RST           		0x0400
		#define GBL_I2CS_RST           		0x0800
		#define GBL_USB_RST           		0x1000
		#define GBL_H264_RST           		0x2000
		#define GBL_ICON_IBC_RST           	0x4000
		#define GBL_GRA_RST           		0x8000
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_RST_CTL2;								//0x0C
		/*-DEFINE-----------------------------------------------------*/
		#define	GBL_CPU_SRAM_RST			0x01
		#define	GBL_SD0_RST					0x02
		#define	GBL_PWM_RST					0x04
		#define	GBL_PSPI_RST				0x08
		#define GBL_USB_PHY_RST             0x10
		#define	GBL_REG_SD0_RST_RST         0x20
		#define	GBL_REG_PWM_RST				0x40
		#define	GBL_REG_PSPI_RST			0x80
		/*------------------------------------------------------------*/
	AIT_REG_B   GBL_DRAM_CLK_DIV;							//0x0D
	AIT_REG_W   GBL_CLK_DIS1; 								//0x0E
		/*-DEFINE-----------------------------------------------------*/
        #define GBL_CLK_DRAM_DIS            0x0001
        #define GBL_CLK_BS_SPI_DIS			0x0002
        #define GBL_CLK_RAW_DIS             0x0004
        #define GBL_CLK_DMA_DIS             0x0008
        #define GBL_CLK_I2C_DIS             0x0010
        #define GBL_CLK_H264_DIS            0x0020
        #define GBL_CLK_ICON_DIS            0x0040
        #define GBL_CLK_GRA_DIS             0x0080
        #define GBL_CLK_USB_DIS             0x0100
        #define GBL_CLK_IBC_DIS             0x0200
        #define GBL_CLK_PWM_DIS             0x0400
        
        #if 1
        #define GBL_CLK_SD0_DIS             0x0800
        #define GBL_CLK_SD1_DIS             0x1000
        #else
        #define GBL_CLK_SD1_DIS		        0x0800
        #define GBL_CLK_SD0_DIS		        0x1000
        #endif

        #define GBL_CLK_PSPI_DIS            0x2000
        #define GBL_CLK_MIPI_DIS            0x4000
        #define GBL_CLK_CPU_PHL_DIS         0x8000
        
		
		/*------------------------------------------------------------*/
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x10[0x1];							//0x10 reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_DPLL0_M;								//0x10
	#endif
	
	AIT_REG_B	GBL_DPLL0_N;								//0x11
	#if (CHIP == VSN_V2)
	AIT_REG_B	GBL_DPLL_PWR;								//0x12
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_DPLL01_PWR;								//0x12
	#endif
		/*-DEFINE-----------------------------------------------*/
		#define DPLL_PWR_DOWN_EN		0x01
		/*-------------------------------------------------------*/
	
	AIT_REG_B	GBL_DPLL_CFG3;								//0x13
		/*-DEFINE-----------------------------------------------*/
		#define DPLL0_UPDATE_EN         0x01
		#define DPLL1_UPDATE_EN         0x02
		#if (CHIP == VSN_V3)
		#define DPLL2_UPDATE_EN         0x04
		#endif
		/*-------------------------------------------------------*/
	AIT_REG_B	GBL_DPLL0_CFG4;								//0x14
		/*-DEFINE-----------------------------------------------*/
		#define PLL_500_1500_1000MHZ	0x0 
		/*-------------------------------------------------------*/
	AIT_REG_B   		_0x15[0x1];							//0x15 reserved
	AIT_REG_B	GBL_DPLL0_K;								//0x16
		/*-DEFINE-----------------------------------------------------*/
		#define	PLL0_CLK_DIV(_a)		(_a - 1)
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_DPLL_SOURCE_SEL;						//0x17
	AIT_REG_B	GBL_DPLL1_M;								//0x18
	AIT_REG_B	GBL_DPLL1_N;								//0x19
		/*-DEFINE-----------------------------------------------------*/
		#define	PLL1_CLK_N(_a)			(_a - 2)
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_CPU_CLK_DIV;							//0x1A
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x1B[0x1];							//0x1B reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_COLOR_CLK_DIV;							//0x1B
	#endif
	AIT_REG_B	GBL_HOST_BANK_SEL0;							//0x1C
	AIT_REG_B	GBL_HOST_BANK_SEL1;							//0x1D
	AIT_REG_B	GBL_HOST_BANK_SEL2;							//0x1E
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x1F[0x1];							//0x1F reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B 	GBL_CLK_DIS2;								//0x1F
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_CLK_AUD_CODEC_DIS			0x01
		#define GBL_CLK_CIDC_DIS                0x02
		#define GBL_CLK_GNR_DIS                	0x04
		#define GBL_CLK_SM_DIS                 	0x08
		#define GBL_CLK_COLOR_DIS               0x10
        /*------------------------------------------------------------*/		
	#endif
	AIT_REG_B	GBL_SW_RESOL_SEL;							//0x20
        /*-DEFINE-----------------------------------------------------*/
        #define GBL_SENSOR_3M                   0xF3
        #define GBL_SENSOR_5M                   0xF5
        #define GBL_SENSOR_8M                   0xF8
        #define GBL_SENSOR_DEF                  0x00
        /*------------------------------------------------------------*/
	AIT_REG_B	GBL_CPU_AUTO_BOOT_CTL;						//0x21
	AIT_REG_B	GBL_LCD_BYPASS_CTL0;						//0x22
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_LCD_BYPASS_CLK_ACTIVE		0x01
		#define GBL_LCD_BYPASS_PWN_DPLL			0x02
		#define GBL_ENTER_SELF_SLEEP_MODE		0x04
		#define GBL_XTAL_OFF_BYPASS				0x20
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_LCD_BYPASS_CTL1;						//0x23
		/*-DEFINE-----------------------------------------------------*/
		#if (CHIP == VSN_V3)
		#define GBL_UART1_TX_PAD0			0x10 // PSNR_D4 as Tx
		#define GBL_UART1_RX_PAD0			0x20 // PSNR_D5 as Rx
		#define GBL_UART1_TX_PAD1			0x40 // PCGPIO2 as Tx
		#define GBL_UART1_RX_PAD1			0x80 // PCGPIO3 as Rx
		#endif
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_VI_CLK_DIV;								//0x24
	AIT_REG_B	GBL_IO_CTL0;								//0x25
		/*-DEFINE-----------------------------------------------------*/
		#if (CHIP == VSN_V3)
		#define GBL_I2C_1_PIN_SEL			0x40
		#define GBL_SD1_IO_PAD0_EN			0x80
		#endif
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_IO_CTL1;								//0x26
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_UART_TX_PAD0			0x01 // PAGPIO0 as Tx
		#define GBL_UART_TX_PAD1			0x02 // PSNR_D8 as Tx
        #define GBL_SIF_PAD_EN              0x04 // PB0~PB3
        #define GBL_I2C_0_PAD_EN            0x08 // For I2CM0 download function used only !!
        #define GBL_I2C_1_PAD_EN            0x10 // 
        #define GBL_I2C_0_PAD_SEL_SNR       0x20 // sensor pad
        #if (CHIP == VSN_V3)
        #define GBL_PSPI_IO_PAD1_EN			0x40
        #endif
        #define GBL_I2C_0_PAD_SEL_HOS       0x00 // host pad
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_IO_CTL2;								//0x27
	AIT_REG_B	GBL_IO_CTL3;								//0x28
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_UART_RX_PAD0			0x01 // PAGPIO6  as Rx
		#define GBL_UART_RX_PAD1			0x02 // PSNR_D9  as Rx
		#define GBL_UART_RX_PAD2			0x08 // PBGPIO12 as Rx
		#define GBL_UART_TX_PAD2			0x04 // PBGPIO14 as Tx
		#define GBL_PSPI_IO_PAD0_EN		(1<<4)
		#define GBL_SD0_IO_PAD0_EN			0x20
		#define GBL_SD0_IO_PAD1_EN			0x80		
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_IO_CTL4;								//0x29
		/*-DEFINE-----------------------------------------------------*/
		#if (CHIP == VSN_V3)
		#define GBL_SD1_IO_PAD1_EN			0x80
		#endif
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_IO_CTL5;								//0x2A
	AIT_REG_B	GBL_IO_CTL6;								//0x2B
	AIT_REG_B	GBL_IO_CTL7;								//0x2C
	AIT_REG_B	GBL_AUDIO_CLK_DIV;							//0x2D
		#define GBL_AUDIO_CLK_SRC_DPLL2						2<<6
		#define GBL_AUDIO_CLK_ON								1<<5
		#define GBL_AUDIO_CLK_OFF								0<<5		
		#define GBL_AUDIO_CLK_DIV_MASK						0x1F
		
	AIT_REG_B	GBL_TCM_SIZE_SEL;							//0x2E
	AIT_REG_B	GBL_PROBE_CTL;								//0x2F
		/*-DEFINE-----------------------------------------------------*/
		#if (CHIP == VSN_V3)
		#define GBL_JTAG_PDGPIO_SEL			0x10
		#endif
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_BIST_CTL;								//0x30
	AIT_REG_B	GBL_BIST_DBG_CTL;							//0x31
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x32[0x1];							//0x32 reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_BIST_DBG_CTL1;
	#endif
	AIT_REG_B	GBL_CPU_CFG;								//0x33
		/*-DEFINE-----------------------------------------------------*/
		#define	GBL_CPU_ASYNC_EN		0x01
		/*------------------------------------------------------------*/
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x34[0x2];							//0x34~0x35 reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_W	GBL_ADC_CLK_DIV;							//0x34	audio clock div [3:0] = 512fs	audio clock div [11:8] = audio clock div [7:4] = 256fs
	#endif
	AIT_REG_B	GBL_MACRO_TEST_MODE_CTL;					//0x36
	AIT_REG_B	GBL_ROM_MARGIN_CTL;							//0x37
	AIT_REG_D	GBL_BIST_RESULT_0;							//0x38
	AIT_REG_D	GBL_BIST_RESULT_1;							//0x3C
	
	AIT_REG_B	GBL_IO_CFG_PSTM_T[0x20];					//0x40~0x5F
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_IO_SCHMITT_TRIG_EN			0x01
		#define	GBL_IO_PULL_DOWN_RESIS_EN		0x02
		#define	GBL_IO_PULL_UP_RESIS_EN			0x04
		#define GBL_IO_SLOW_SLEW_EN				0x08
		#define GBL_IO_OUT_DRIVING(_a)			(_a << 0x5)
		/*------------------------------------------------------------*/
	
	AIT_REG_D	GBL_DRAM_BIST_ST_ADDR;						//0x60
	AIT_REG_D	GBL_DRAM_BIST_END_ADDR;						//0x64
	AIT_REG_B	GBL_GPIO_WAKE_EDGE_EN;						//0x68
	AIT_REG_B	GBL_GPIO_WAKE_LEVEL_EN;						//0x69
	AIT_REG_B	GBL_WAKE_CTL;								//0x6A
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_WAKE_GPIO0								0x01
		#define GBL_WAKE_GPIO6								0x02
		#define GBL_WAKE_USB_RESUME							0x04
		#if (CHIP == VSN_V3)
		#define GBL_WAKE_GPIO23								0x08
		#define GBL_WAKE_GPIO50								0x10
		#define GBL_WAKE_GPIO63								0x20
		#endif
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_GPIO_WAKE_INT_POLARITY;					//0x6B
	AIT_REG_B	GBL_GPIO_WAKE_HOST_SR;						//0x6C
	AIT_REG_B	GBL_GPIO_WAKE_HOST_EN;						//0x6D
	AIT_REG_B	GBL_GPIO_WAKE_CPU_SR;						//0x6E
	AIT_REG_B	GBL_GPIO_WAKE_CPU_EN;						//0x6F
	
	AIT_REG_B	GBL_IO_CTL8;								//0x70
	AIT_REG_B	GBL_IO_CTL9;								//0x71
	AIT_REG_B	GBL_IO_CTL10;								//0x72
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x73[0x1];							//0x73 reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_IO_CTL11;								//0x73
	#endif
	AIT_REG_B	GBL_SRAM_MARGIN_CTL0;						//0x74
	AIT_REG_B	GBL_SRAM_MARGIN_CTL1;						//0x75
	AIT_REG_B	GBL_SRAM_MARGIN_CTL2;						//0x76
	AIT_REG_B	GBL_MIPI_RX_CLK_SEL;						//0x77
		/*-DEFINE-----------------------------------------------------*/
		#define	PLL_MIPI_MCLK_EN		0x02
		/*------------------------------------------------------------*/
	AIT_REG_B	GBL_I2CS_HOST_INT_EN;						//0x78
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x79[0x1];							//0x79 reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_RST_CTL3;								//0x79
		/*-DEFINE-----------------------------------------------------*/
		#define	GBL_REG_SD1_RST         0x80
		#define	GBL_REG_SCAL2_RST	    0x40
		#define	GBL_REG_SCAL1_RST	    0x20
		#define	GBL_REG_SM_RST		    0x10
		#define	GBL_SD1_RST			    0x08
		#define	GBL_SCAL2_RST		    0x04
		#define	GBL_SCAL1_RST		    0x02
		#define	GBL_SM_RST			    0x01
		/*------------------------------------------------------------*/
	#endif
	AIT_REG_B	GBL_I2CS_HOST_INT_SR;						//0x7A
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x7B[0x1];							//0x7B reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_IO_CTL12;								//0x7B
	#endif
	AIT_REG_B	GBL_I2CS_CPU_INT_EN;						//0x7C
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0x7D[0x1];							//0x7D reserved
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_IO_CTL13;								//0x7D
	#endif
	AIT_REG_B	GBL_I2CS_CPU_INT_SR;						//0x7E
	AIT_REG_B   		_0x7F[0x1];							//0x7F reserved
	
	AIT_REG_B	GBL_BIST_SEL[0x10];							//0x80~0x8F
	
	AIT_REG_B	GBL_I2CS_CTL;								//0x90
	AIT_REG_B	GBL_I2CS_PROBE_SEL;							//0x91
	AIT_REG_B	GBL_I2CS_HOLD_TIME_CNT;						//0x92
	AIT_REG_B	GBL_I2CS_ST_CHECK_CNT;						//0x93
	AIT_REG_B	GBL_I2CS_ERR_TIMER;							//0x94
	AIT_REG_B   		_0x95[0x1];							//0x95 reserved
	AIT_REG_B	GBL_I2CS_FSM_STATE;							//0x96
	AIT_REG_B	GBL_I2CS_SR;								//0x97
	AIT_REG_B	GBL_IO_CFG_PSTM_B24[0x8];					//0x98~0x9F
	
	AIT_REG_B	GBL_XTAL_IO_CFG;							//0xA0
	AIT_REG_B	GBL_PI2C_AGPIO_CFG[0xE];					//0xA1
	AIT_REG_B   		_0xAF[0x1];							//0xAF reserved
	
	
	AIT_REG_B	GBL_IO_CFG_PBGPIO[0x10];					//0xB0~0xBF
	
	AIT_REG_B	GBL_IO_CFG_PSNR[0x6];						//0xC0~0xC5
	AIT_REG_B   		_0xC6[0x4];							//0xC6~0xC9 reserved
	AIT_REG_B	GBL_IO_CFG_PSNR2[0x3];						//0xC0~0xC5
	AIT_REG_B   		_0xCD[0x4];							//0xCD~0xD0 reserved
	AIT_REG_B	GBL_IO_CFG_PCGPIO[0x6];						//0xD1~0xD6
	AIT_REG_B   		_0xD7[0x1];							//0xD7 reserved
	AIT_REG_B	GBL_IO_CFG_PSTM_B[0x18];					//0xD8~0xEF
	
	AIT_REG_B   		_0xF0[0x4];							//0xF0~0xF3 reserved
	#if (CHIP == VSN_V2)
	AIT_REG_D   GBL_DPLL1_CFG;								//0xF4
		/*-DEFINE-----------------------------------------------*/
		#define PLL_DIV_BIT_MASK			0x7
		/*-------------------------------------------------------*/
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_W   GBL_DPLL1_CFG01;							//0xF4
		/*-DEFINE-----------------------------------------------*/
		#define PLL_DIV_BIT_MASK			0x7
		/*-------------------------------------------------------*/
	AIT_REG_B   GBL_DPLL1_CFG2;								//0xF6
	AIT_REG_B	GBL_CLK_SEL;								//0xF7
	#endif
		/*-DEFINE-----------------------------------------------*/
		#define PLL_200_500_350MHZ			0x00 
		#define PLL_300_700_500MHZ			0x40
		#define PLL1_CLK_DIV_8           	0x00
		#define PLL1_CLK_DIV_4           	0x01
		#define PLL1_CLK_DIV_2           	0x02
		#define PLL1_CLK_DIV_1           	0x03
		#if (CHIP == VSN_V2)
		#define PLL1_V2I_HIGH_GAIN_EN		0x80
		#define PLL1_V2I_GAIN_ADJUST_DIS	0x10000
		#endif
		#if (CHIP == VSN_V3)
		#define PLL_V2I_HIGH_GAIN_EN		0x80
		#define PLL_V2I_GAIN_ADJUST_DIS	0x01
		#define GBL_CLK_SEL_MUX0			0x01
		#define PLL_SEL_PLL_OFFSET1		0x01
		#endif
		/*-------------------------------------------------------*/
		
	AIT_REG_B	GBL_STACK_MEM_TYPE;							//0xF8
		/*-DEFINE-----------------------------------------------*/
		#if (CHIP == VSN_V2)
		#define MEM_ERTON_16Mb_SDR			0x00 
		#define MEM_WINBO_16Mb_SDR			0x00
		#define MEM_ISSI_16Mb_SDR			0x00
		
		#define MEM_WINBO_32Mb_SDR			0x01 
		#define MEM_WINBO_64Mb_SDR			0x01
		#define MEM_WINBO_32Mb_DDR			0x01 
		#define MEM_WINBO_64Mb_DDR			0x01
		#define MEM_ERTON_64Mb_DDR			0x01
		
		#define MEM_WINBO_128Mb_DDR			0x02
		#define MEM_WINBO_256Mb_DDR			0x03
		#define MEM_PIECE_512Mb_DDR			0x04
		#endif
		#if (CHIP == VSN_V3)
		#define MEM_WINBO_256Mb_DDR			0x02
		#endif
		#define GBL_PROJECT_ID_RD_EN		0x80
		/*-------------------------------------------------------*/
	#if (CHIP == VSN_V2)
	AIT_REG_B   		_0xF9[0x1];							//0xF9
	#endif
	#if (CHIP == VSN_V3)
	AIT_REG_B	GBL_CPU_CLK_SEL;							//0xF9
		/*-DEFINE-----------------------------------------------*/
		#define CPU_CLK_SEL_MUX0			0x01
		/*-------------------------------------------------------*/
	#endif
	AIT_REG_B   GBL_CHIP_VER; 								//0xFA
	AIT_REG_B   GBL_SIF_REBOOT_EN; 							//0xFB
	AIT_REG_W   GBL_SECO_VER; 								//0xFC
	AIT_REG_W   GBL_PROJETCT_ID; 							//0xFE
} AITS_GBL, *AITPS_GBL;
#endif // (CHIP == VSN_V3) || (CHIP == VSN_V2)

#endif // _MMPH_REG_GBL_H_
