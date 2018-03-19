//==============================================================================
//
//  File        : mmpf_usbphy.h
//  Description : INCLUDE File for the Firmware USB PHY Driver
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_USBPHY_H_
#define _MMPF_USBPHY_H_
#include <mach/hardware.h>

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================
/* - PHY register index -
 * For more detail information, please refer to
 * "AIT_USB2PHYA_U055L2_function_spec_v1.1.doc" for P_V2
 * "AIT_USB2PHYA_U040L7_function_spec_v1.0.doc" for MCR_V2
 */
#define BG_OPT1_TRCV_OPT        (0x00)
    /* Default value of TRCV Option reg */
    #define TRCV_OPT_DEFAULT        (0x0000)
    /* Default value of BG Option1 reg */
    #define BG_OPT1_DEFAULT         (0x0300)
    /* Band-Gap 1.2V selection */
    #define RESISTOR_DIV_OUTPUT     (0x0800)
    /* Embedded LDO Output Cotrol */
    #define LDO_OUT_1d20V           (0x0000)
    #define LDO_OUT_1d10V           (0x1000)
    #define LDO_OUT_1d30V           (0x2000)
    #define LDO_OUT_1d25V           (0x3000)
#define PLL_CTL1_BG_OPT2        (0x02)
    /* Default value of BG Option2 reg */
    #define BG_OPT2_DEFAULT         (0x005F)
    #if (CHIP == P_V2)
    /* Default value of PLL Control 1 reg */
    #define PLL_CTL1_DEFAULT        (0x1900)
    #endif
    #if (CHIP == MCR_V2)
    #define PLL_CTL1_DEFAULT        (0x9400)
    #endif
    /* Charge Pump current control */
    #define CHARGE_PUMP_7d5uA       (0x0800)
    #define CHARGE_PUMP_5d0uA       (0x0400)
#define PLL_CTL3_CTL2           (0x04)
    /* PLL M selection from register or XTL_SELECT */
    #define PLL_M_REG_CTL           (0x0080)
    #define PLL_M_12MHZ_DIV         (0x0028)
    #define PLL_M_24MHZ_DIV         (0x0014)
    #define PLL_M_30MHZ_DIV         (0x0010)
    #if (CHIP == P_V2)
    /* Default value of PLL Control 3 reg */
    #define PLL_CTL3_DEFAULT        (0x9000)
    #endif
    #if (CHIP == MCR_V2)
    #define PLL_CTL3_DEFAULT        (0x8200)
    #endif
    /* PLL lock judge value with REG_CLK count */
    #define TT_DELAY_1d4nS          (0x8000)
    #define N_SEL_IN_DIV256_EN      (0x0200)
#define FS_TX_OPT_PLL_CTL4      (0x06)
    #define PLL_CTL4_DEFAULT        (0x0010)
    /* For Full-Speed current tune */
    #define FXTX_RESL_42OHM         (0x0000)
    #define FXTX_RESL_45OHM         (0x0100)
    #define FXTX_RESL_49OHM         (0x0300)
    /* HSTX data sampler */
    #define HSTX_DATA_SAMPLER_DIS   (0x0400)
#define SQ_OPT_HS_TX_OPT        (0x08)
    /* Default value of HS_TX Option reg */
    #define HS_TX_DEFAULT           (0x0044)
    #define LOWER_AFF_POWER_EN      (0x0040)
    #define HS_CUR_MASK             (0x07)
    #define HS_CUR_200mV            (0x0000)
    #define HS_CUR_240mV            (0x0001)
    #define HS_CUR_280mV            (0x0002)
    #define HS_CUR_320mV            (0x0003)
    #define HS_CUR_360mV            (0x0004)
    #define HS_CUR_400mV            (0x0005)
    #define HS_CUR_440mV            (0x0006)
    #define HS_CUR_480mV            (0x0007)
    /* Enable Digital Filiter on Squelch Detection */
    #define SQ_FILTER_EN            (0x0800)
    /* Squelch level tuning option */
    #define SQ_LEVEL_SHIFT          (0x08)
    #define SQ_LEVEL_MASK           (0x07)
    #define SQ_LEVEL_75mV           (0x0000)
    #define SQ_LEVEL_87d5mV         (0x0100)
    #define SQ_LEVEL_100mV          (0x0200)
    #define SQ_LEVEL_112d5mV        (0x0300)
#define PLL_TEST_OTG_CTL        (0x0A)
    /* Control OTG reference voltage */
    #define OTG_VREF_PWR_DOWN       (0x0020)
    /* Enable OTG comparators */
    #define OTG_COMP_ENABLE         (0x000F)
    /* Analog test mode */
    #define ANA_TEST_MODE           (0x0100)
    /* Control REFCLK_CORE in Analog test mode */
    #define REFCLK_CORE_EN          (0x0000)
    #define REFCLK_CORE_OFF         (0x0200)
    /* Control XO power down in Analog test mode */
    #define XO_BLOCK_EN             (0x0000)
    #define XO_BLOCK_OFF            (0x0400)
    /* Enable Single-Pair PLL output */
    #define PLL_NORMAL_OUT          (0x0000)
    #define PLL_SPCLK_OUT           (0x2000)
    /* Control PLL power */
    #define PLL_PWR_UP              (0x0000)
    #define PLL_PWR_DOWN            (0x8000)
#define PWR_CTL_PUPD_TEST       (0x0C)
    /* RPU/RPD control */
    #define DM_PULL_DOWN            (0x0004)
    #define DP_PULL_DOWN            (0x0008)
    #define DM_PULL_UP1             (0x0010)
    #define DM_PULL_UP2             (0x0020)
    #define DP_PULL_UP1             (0x0040)
    #define DP_PULL_UP2             (0x0080)
    /* Power Down control */
    #define HSTX_PWR_DOWN           (0x0100)
    #define HSRX_PWR_DOWN           (0x0200)
    #define FSLS_TX_PWR_DOWN        (0x0400)
    #define FSLS_RX_PWR_DOWN        (0x0800)
    #define DISCON_DTC_PWR_DOWN     (0x1000)
    #define SQ_DTC_PWR_DOWN         (0x2000)
    #define USB_BS_PWR_DOWN         (0x4000)
    #define USB_BG_PWR_DOWN         (0x8000)
#define UTMI_DBG_CHIP_TEST      (0x10)
#define HS_RX_DBG_RX_ERR        (0x12)

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================
//MMP_USHORT MMPF_USBPHY_Read(MMP_UBYTE addr);
//void MMPF_USBPHY_Write(MMP_UBYTE addr, MMP_USHORT data);
void MMPF_USBPHY_Initialize(u8 bOtgEn);
void MMPF_USBPHY_PowerUp(void);
void MMPF_USBPHY_PowerDown(void);
//void MMPF_USBPHY_SetSignalTune(MMP_ULONG ulTxCur, MMP_ULONG ulSQ);

#endif // _MMPF_USBPHY_H_
