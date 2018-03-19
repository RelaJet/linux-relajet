//==============================================================================
//
//  File        : mmp_reg_mci.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Eroy Yang
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_MCI_H_
#define _MMP_REG_MCI_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

#if (CHIP == MCR_V2)
#define	MCI_SRC_NUM         					(23)

#define MCI_SRC_CPUA_I                  (0)
#define MCI_SRC_CPUA_D                  (1)
#define MCI_SRC_CPUB_I                  (2)
#define MCI_SRC_CPUB_D                  (3)
#define MCI_SRC_LCD1_IBC3            	(4)
#define MCI_SRC_LCD2_CCIR1_IBC4         (5)
#define MCI_SRC_LCD3_CCIR2              (6)
#define MCI_SRC_IBC0                    (7)
#define MCI_SRC_IBC1                    (8)
#define MCI_SRC_IBC2                    (9)
#define MCI_SRC_RAWS1                   (10)
#define MCI_SRC_RAWS2_JPGLB             (11)
#define MCI_SRC_USB1_UART_SM_COLO       (12)
#define MCI_SRC_USB2_ROM_PSPI           (13)
#define MCI_SRC_H264_1                  (14)
#define MCI_SRC_H264_2                  (15)
#define MCI_SRC_RAW_F                   (16)
#define MCI_SRC_GRA                     (17)
#define MCI_SRC_DMAM0_SD0_JPGDEC        (18)
#define MCI_SRC_DMAM1_SD1_JPGCB         (19)
#define MCI_SRC_DMAR0_SD2               (20)
#define MCI_SRC_DMAR1_ICON_SIF          (21)
#define MCI_SRC_LDC_I2CM_HOST_DRAMBIST  (22)
#define MCI_SRC_MASK                    (0x7FFFFF)

//------------------------------
// MCI  Structure (0x8000 7700)
//------------------------------
typedef struct _AITS_MCI 
{
    AIT_REG_B   MCI_FB_CTL;                                 // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_FB_EN_ALWAYS_HIGH       0x02
        /*------------------------------------------------------------*/
    AIT_REG_B   MCI_SRAM_DRIVE_SEL[10];						// 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_READ_MARGIN_EN			0x10
        #define MCI_READ_MARGIN_IN_MASK		0x0F
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x0B;    
    AIT_REG_B   MCI_SRAM_LIGHT_SLEEP_CTL[3];				// 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_LIGHT_SLEEP_DIS			0x00
        #define MCI_LIGHT_SLEEP_EN			0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   MCI_Q_SRAM_DRIVE_SEL;						// 0x0F   
	
	AIT_REG_B   MCI_EXTM_TYPE;								// 0x10
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_EXTM_NONE               0x00
        #define MCI_EXTM_SRAM               0x01
        #define MCI_EXTM_DRAM               0x02
        /*------------------------------------------------------------*/
	AIT_REG_B   MCI_FB_DRAM_CTL;							// 0x11
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_RD_AFTER_WR_EN          0x04
        #define MCI_WR_Q_NUM_4              0x20
        #define MCI_WR_Q_NUM_8              0x00
        #define MCI_RD_Q_NUM_4              0x40
        #define MCI_RD_Q_NUM_8              0x00
        #define MCI_FB_REQ_WITH_LOCK        0x80
        /*------------------------------------------------------------*/
	AIT_REG_B   MCI_DRAM_COMP_VAL;							// 0x12

	AIT_REG_B   MCI_FB_INIT_DEC_VAL;						// 0x13
    AIT_REG_D   MCI_SRAM_DEEP_SLEEP_CTL;					// 0x14    
    AIT_REG_D   MCI_SRAM_SHUT_DOWN_CTL;						// 0x18
    AIT_REG_D  	MCI_SOURCR_REQ_INFO;						// 0x1C  

	AIT_REG_B   MCI_INIT_DEC_VAL[MCI_SRC_NUM];				// 0x20
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_INIT_DEC_DEFT_VAL		0x50
        #define MCI_INIT_WT_MASK            0x1F
        #define MCI_INIT_WT_MAX				0x1F
        #define MCI_INIT_WT_MIN				0x00
        #define MCI_DEC_VAL(d)              ((d & 0x03) << 6)
        #define MCI_DEC_VAL_MASK            0xC0
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x37[9];  	

	AIT_REG_B   MCI_DEC_THD[MCI_SRC_NUM];					// 0x40
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_WT_DEC_DEFT_VAL			0x40
        #define MCI_WT_DEC_THD_MASK         0x1F
        #define MCI_WT_SP_DEC_THD_MASK      0xE0
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x57[9];  	

	AIT_REG_B   MCI_NA_INIT_DEC_VAL[MCI_SRC_NUM];			// 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_NA_INIT_DEFT_VAl		0x11
        #define MCI_NA_INIT_WT_MASK         0x07
        #define MCI_NA_INIT_WT_MAX			0x07
        #define MCI_NA_DEC_VAL_MASK         0x30
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x77[9];  

	AIT_REG_B   MCI_ROW_INIT_DEC_VAL[MCI_SRC_NUM];			// 0x80
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_ROW_INIT_DEFT_VAl		0x11
        #define MCI_ROW_INIT_WT_MASK        0x07
        #define MCI_ROW_INIT_WT_MAX			0x07
        #define MCI_ROW_DEC_VAL_MASK        0x30
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x97[9];  

	AIT_REG_B   MCI_RW_INIT_DEC_VAL[MCI_SRC_NUM];			// 0xA0
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_RW_INIT_DEFT_VAl		0x11
        #define MCI_RW_INIT_WT_MASK        	0x07
        #define MCI_RW_INIT_WT_MAX			0x07
        #define MCI_RW_DEC_VAL_MASK        	0x30
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xB7[9];  

	AIT_REG_D   MCI_URGENT_EN;								// 0xC0
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_URGENT_ENABLE_MASK      0xFFFFFFFF
        #define MCI_URGENT_ENABLE(_a)       ((1 << _a) & MCI_URGENT_ENABLE_MASK)
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xC4[12]; 

	AIT_REG_D   MCI_REQ_MONITOR0_SRC_EN;					// 0xD0
	AIT_REG_D   MCI_REQ_MONITOR1_SRC_EN;					// 0xD4
	AIT_REG_B   MCI_MONITOR_CPU_INT_EN;						// 0xD8
	AIT_REG_B   MCI_MONITOR_HOST_INT_EN;					// 0xD9
	AIT_REG_B   MCI_MONITOR_CPU_INT_STATUS;					// 0xDA
	AIT_REG_B   MCI_MONITOR_HOST_INT_STATUS;				// 0xDB		
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_MONITOR_INT_EN			0x01
        #define MCI_MONITOR_SR_OVF        	0x01
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xDC[4]; 

    AIT_REG_D   MCI_REQ_COUNTER_0;                          // 0xE0 [RO]
    AIT_REG_D   MCI_URGENT_COUNTER_0;                       // 0xE4 [RO]
    AIT_REG_D   MCI_REQ_COUNTER_1;                          // 0xE8 [RO]
    AIT_REG_D   MCI_URGENT_COUNTER_1;                       // 0xEC [RO]

    AIT_REG_B                           _xF0[16]; 

} AITS_MCI, *AITPS_MCI;

#endif

/// @}
#endif // _MMP_REG_MCI_H_