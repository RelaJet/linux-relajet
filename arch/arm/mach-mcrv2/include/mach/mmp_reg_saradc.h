//==============================================================================
//
//  File        : mmp_reg_saradc.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Eroy Yang
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_SARADC_H_
#define _MMP_REG_SARADC_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/
#if (CHIP == MCR_V2)
// **********************************
//   SARADC_POR structure (0x8000 3A80)
// **********************************
typedef struct _AITS_SARADC_POR {
	AIT_REG_B   SARADC_CTL;                                  // 0x3A80
	    /*-DEFINE-----------------------------------------------------*/
   	    #define SARADC_SPI_EN               0x01
	    #define PSPI2_MISO_FROM_SARADC      0x00
	    #define SARADC_USB_SPI_SEL			0x08
	    #define SARADC_POLL					0x80
        /*------------------------------------------------------------*/    
	AIT_REG_B   POR_CTL;									 // 0x3A81
} AITS_SARADC_POR, *AITPS_SARADC_POR;

#define SARADC_SW_RST0          (0x00)
#define SARADC_SW_RST1          (0x01)
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_SW_RESET                 (0xFF)
    /*------------------------------------------------------------*/

#define SARADC_TP_PANEL_PARAM   (0x20)    
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_PULLUP_RESISTOR_MASK     (0x3F)
    #define SARADC_CURRENT_400uA            (0x40)
    #define SARADC_CURRENT_200uA            (0x00)
    #define SARADC_TOUCH_PANEL_5WIRE        (0x80)
    #define SARADC_TOUCH_PANEL_4WIRE        (0x00)
    /*------------------------------------------------------------*/    
#define SARADC_TP_ADC_SRC_SEL   (0x21)     
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_TP_COD_MODE_EN           (0x01)
    #define SARADC_TP_Y_COORDINATE_EN       (0x02)
    #define SARADC_TP_X_COORDINATE_EN       (0x04)
    #define SARADC_TP_PRESSURE_EN           (0x08)
    #define SARADC_TP_AUX1_EN               (0x10)
    #define SARADC_TP_AUX2_EN               (0x20)
    #define SARADC_TP_AUX3_EN               (0x40)
    #define SARADC_TP_CONTI_MODE            (0x80)
    #define SARADC_TP_POLLING_MODE          (0x00)
    /*------------------------------------------------------------*/
#define SARADC_TP_CLK_PD_CFG    (0x22)
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_TP_EXT_CLK               (0x08)
    #define SARADC_TP_INT_CLK               (0x00)
    #define SARADC_IREF_PILI_CURRENT_MASK   (0x70)
    #define SARADC_ENABLE_PEN_ADC_POWER     (0x80)
    /*------------------------------------------------------------*/
#define SARADC_TP_INT_CFG       (0x23)
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_TP_INT_SRC_SEL_MASK      (0x03)
    #define SARADC_TP_INT_SRC_PEN_DOWN      (0x00)
    #define SARADC_TP_INT_POL_HIGH          (0x40)
    #define SARADC_TP_INT_POL_LOW           (0x00)
    /*------------------------------------------------------------*/    
#define SARADC_TP_MEASURE_MODE  (0x24)
    /*-DEFINE-----------------------------------------------------*/
    #define SARADC_TP_WAIT_EN               (0x02)
    #define SARADC_CONTI_MODE_RATE_MASK     (0x0C)
    #define SARADC_ADC_SET_TIME_MASK        (0xF0)
    /*------------------------------------------------------------*/    
#define SARADC_TP_PEN_DOWN      (0x25)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_PEN_STATUS_PEN_UP            (0x00)
    #define TP_PEN_STATUS_PEN_DOWN          (0x02)
    #define TP_PEN_ADC_AUX_ADX_EN_MASK      (0x0C)
    #define TP_POR_INV_EN                   (0x10)
    #define TP_PEN_DOWN_EN                  (0x20)
    #define TP_FAST_READ_EN                 (0x00)
    #define TP_FAST_READ_DIS                (0x40)
    #define TP_PEN_DOWN_FLAG_EN             (0x80)
    /*------------------------------------------------------------*/
#define SARADC_TP_POLLING_STATUS (0x26)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_POLL_GET_RESULT              (0x01)
    #define TP_MEASURE_RESULT_STATUS        (0x02)
    /*------------------------------------------------------------*/ 

#define SARADC_TP_RD_REQ_STATUS (0x27)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_READ_REQ                     (0x01)
    #define TP_HS_FOR_READ_REQ              (0x02)
    /*------------------------------------------------------------*/ 
#define SARADC_TP_RD_DATA0      (0x28)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_ADC_DATA_LSB                 (0xFF)
    /*------------------------------------------------------------*/ 
#define SARADC_TP_RD_DATA1      (0x29)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_ADC_DATA_MSB                 (0x03)
    #define TP_PEN_DOWN_FLAG                (0x08)
    #define TP_ADC_SRC_ID_MASK              (0x70)
    #define TP_ADC_SRC_ID_NONE              (0x00)
    #define TP_ADC_SRC_ID_X                 (0x10)
    #define TP_ADC_SRC_ID_Y                 (0x20)
    #define TP_ADC_SRC_ID_PRESURE           (0x30)
    #define TP_ADC_SRC_ID_AUX1              (0x40)
    #define TP_ADC_SRC_ID_AUX2              (0x50)
    #define TP_ADC_SRC_ID_AUX3              (0x60)
    #define TP_ADC_DATA_VALID               (0x80)
    /*------------------------------------------------------------*/        
#define SARADC_BATT_DET         (0x2A)
    /*-DEFINE-----------------------------------------------------*/
    #define LOW_BATT_DELAY_MASK             (0x07)
    #define BATT_ALARM_EN                   (0x08)
    /*------------------------------------------------------------*/
#define SARADC_TP_AUX_SUB_SEL   (0x2B)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_AUX1_SUB_SEL_MASK            (0x03)
    #define TP_AUX2_SUB_SEL_MASK            (0x0C)
    #define TP_AUX3_SUB_SEL_MASK            (0x30)
    #define TP_AUX10_SUB_SEL_MASK           (0x40)
    /*------------------------------------------------------------*/

#define SARADC_TP_MODE_SEL      (0x70)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_MODE_NORMAL                  (0x01)
    #define TP_MODE_ANALOG_TEST             (0x01)
    #define TP_MODE_DIGITAL_TEST            (0x02)
    #define TP_ADC_INPUT_BYPASS             (0x04)
    /*------------------------------------------------------------*/
#define SARADC_ANA_TEST_CTL     (0x71)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_ANA_DIR_SELB_X               (0x01)
    #define TP_ANA_DIR_SELB_Y               (0x02)
    #define TP_ANA_DIR_SELB_PRY             (0x04)
    #define TP_ANA_DIR_SELB_PRX             (0x08)
    #define TP_ANA_DIR_SELB_AUX1            (0x10)
    #define TP_ANA_DIR_SELB_AUX2            (0x20)
    #define TP_ANA_DIR_SELB_AUX3            (0x40)
    #define TP_ANA_DIR_SELB_CENB            (0x80)
    /*------------------------------------------------------------*/
#define SARADC_DIGI_TEST_DATA0  (0x72)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_TEST_ADC_DATA_LSB            (0xFF)
    /*------------------------------------------------------------*/
#define SARADC_DIGI_TEST_DATA1  (0x73)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_TEST_ADC_DATA_MSB            (0x03)
    #define TP_TEST_PEN_DOWN_FLAG           (0x20)
    /*------------------------------------------------------------*/
#define SARADC_ANA_DIR_DATA0    (0x74)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_DIR_ADC_DATA_LSB             (0xFF)
    /*------------------------------------------------------------*/  
#define SARADC_ANA_DIR_DATA1    (0x75)
    /*-DEFINE-----------------------------------------------------*/
    #define TP_DIR_ADC_DATA_MSB             (0x03)
    #define TP_DIR_EOC_FLAG                 (0x10)
    #define TP_DIR_PEN_DOWN_FLAG            (0x20)
    /*------------------------------------------------------------*/
#endif

/// @}

#endif // _MMP_REG_SARADC_H_