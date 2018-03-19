
//==============================================================================
//
//  File        : mmpf_usbphy.c
//  Description : Firmware USB PHY Driver
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================
//#include "includes_fw.h"
//#include "lib_retina.h"

#include <linux/module.h>

#include <mach/hardware.h>
#include <mach/mmpf_typedef.h>

#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_usb.h>
#include <mach/mmpf_usbphy.h>

#define CHIP_CORE_ID_MCR_V2_SHT (0x82)
#define CHIP_CORE_ID_MCR_V2_MP (0x83)
extern MMP_UBYTE gbSystemCoreID;// = CHIP_CORE_ID_MCR_V2_MP;
static MMP_UBYTE gbEcoVer = 0;
/** @addtogroup MMPF_USB
@{
*/

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================
#define PHY_DEV_ADDR_BITS       (8)
#define PHY_WORD_ADDR_BITS      (8)
#define PHY_WORD_DATA_BITS      (16)

#define PHY_WR_DEV_ADDR         (0x20)
#define PHY_RD_DEV_ADDR         (0x21)

//==============================================================================
//
//                              LOCAL VARIABLES
//
//==============================================================================
static const MMP_UBYTE mbSpiStartOpr[1] = {
    OPR_SS_HIGH,    //chip select inactive
};

static const MMP_UBYTE mbSpiStopOpr[3] = {
    OPR_SS_LOW | OPR_SCLK_LOW,
    OPR_SS_LOW | OPR_SCLK_LOW,
    OPR_SS_HIGH,    //chip select inactive
};

static MMP_UBYTE    mbSpiDevAddrOpr[PHY_DEV_ADDR_BITS << 1];
static MMP_UBYTE    mbSpiWordAddrOpr[PHY_WORD_ADDR_BITS << 1];
static MMP_UBYTE    mbSpiWordDataOpr[PHY_WORD_DATA_BITS << 1];

static MMP_BOOL     mbUsbPhyInitialized = MMP_FALSE;
static MMP_BOOL     mbUsbOtgEn = MMP_FALSE;
#if (CHIP == MCR_V2)
/* For MercuryV2 MP only, refine PHY settings in HS mode */
static MMP_BOOL     mbUsbPhyHsRefined = MMP_FALSE;
#endif

static MMP_ULONG    mVref = BG_VREF_400mV;
static MMP_ULONG    mBiasCurrent = BG_BIAS_CURRENT_25uA;
static MMP_ULONG    mTxCur = HS_CUR_400mV;
static MMP_ULONG    mTxCurPlus = HS_CUR_PLUS_00mV;
static MMP_ULONG    mSqLevel = SQ_LEVEL_62d5mV;

//==============================================================================
//
//                              EXTERN VARIABLES
//
//==============================================================================

//==============================================================================
//
//                              EXTERN FUNCTIONS
//
//==============================================================================

//==============================================================================
//
//                              FUNCTION PROTOTYPE
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_SetSpiWaveformOpr
//  Description : Set waveform OPR according to the specified value and bit num
//------------------------------------------------------------------------------
void MMPF_USBPHY_SetSpiWaveformOpr(MMP_UBYTE *opr, MMP_USHORT value, MMP_LONG bits)
{
    MMP_LONG i;

    // MSB bit rx/tx first
    for(i = (bits << 1) - 1; i > 0; i -= 2) {
        if (value & 0x01) {
            opr[i]   = OPR_MOSI_HIGH | OPR_SS_LOW | OPR_SCLK_HIGH;
            opr[i-1] = OPR_MOSI_HIGH | OPR_SS_LOW;
        }
        else {
            opr[i]   = OPR_MOSI_LOW | OPR_SS_LOW | OPR_SCLK_HIGH;
            opr[i-1] = OPR_MOSI_LOW | OPR_SS_LOW;
        }
        // check bit by bit
        value >>= 1;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_Read
//  Description : Read USB PHY controller register
//------------------------------------------------------------------------------
MMP_USHORT MMPF_USBPHY_Read(MMP_UBYTE addr)
{
    MMP_ULONG i, ofst = PHY_WORD_DATA_BITS - 1;
    MMP_USHORT data = 0;
    AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;

    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiDevAddrOpr, PHY_RD_DEV_ADDR, PHY_DEV_ADDR_BITS);
    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiWordAddrOpr, addr, PHY_WORD_ADDR_BITS);
    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiWordDataOpr, 0, PHY_WORD_DATA_BITS);

    for(i = 0; i < sizeof(mbSpiStartOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiStartOpr[i];
    for(i = 0; i < sizeof(mbSpiDevAddrOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiDevAddrOpr[i];
    for(i = 0; i < sizeof(mbSpiWordAddrOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiWordAddrOpr[i];
    for(i = 0; i < sizeof(mbSpiWordDataOpr); i++) {
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiWordDataOpr[i];
        if (mbSpiWordDataOpr[i] & OPR_SCLK_HIGH) {
            data |= (pUSB_DMA->USB_PHY_SPI_CTL2 & OPR_MISO_HIGH) << ofst;
            ofst--;
        }
    }
    for(i = 0; i < sizeof(mbSpiStopOpr); i++)
         pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiStopOpr[i];

    #if 0
    RTNA_DBG_Str(0, "USB_PHY Get [");
    RTNA_DBG_Byte(0, addr + 1);
    RTNA_DBG_Str(0, ",");
    RTNA_DBG_Byte(0, addr);
    RTNA_DBG_Str(0, "]:");
    RTNA_DBG_Short(0, data);
    RTNA_DBG_Str(0, "\r\n");
    #endif

    return data;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_Read
//  Description : Write USB PHY controller register
//------------------------------------------------------------------------------
void MMPF_USBPHY_Write(MMP_UBYTE addr, MMP_USHORT data)
{
    MMP_ULONG i;
    AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;

    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiDevAddrOpr, PHY_WR_DEV_ADDR, PHY_DEV_ADDR_BITS);
    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiWordAddrOpr, addr, PHY_WORD_ADDR_BITS);
    MMPF_USBPHY_SetSpiWaveformOpr(mbSpiWordDataOpr, data, PHY_WORD_DATA_BITS);

    for(i = 0; i < sizeof(mbSpiStartOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiStartOpr[i];
    for(i = 0; i < sizeof(mbSpiDevAddrOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiDevAddrOpr[i];
    for(i = 0; i < sizeof(mbSpiWordAddrOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiWordAddrOpr[i];
    for(i = 0; i < sizeof(mbSpiWordDataOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiWordDataOpr[i];
    for(i = 0; i < sizeof(mbSpiStopOpr); i++)
        pUSB_DMA->USB_PHY_SPI_CTL1 = mbSpiStopOpr[i];

    #if 0
    RTNA_DBG_Str(0, "USB_PHY Set [");
    RTNA_DBG_Byte(0, addr + 1);
    RTNA_DBG_Str(0, ",");
    RTNA_DBG_Byte(0, addr);
    RTNA_DBG_Str(0, "]:");
    RTNA_DBG_Short(0, data);
    RTNA_DBG_Str(0, "\r\n");
    #endif
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_SetVrefTune
//  Description : Overwrite the default BG Vref tune
//------------------------------------------------------------------------------
void MMPF_USBPHY_SetVrefTune(MMP_ULONG ulVref)
{
    mVref = (ulVref & BG_VREF_MASK) << BG_VREF_SHIFT;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_SetBiasCurrentTune
//  Description : Overwrite the default BG bias current tune
//------------------------------------------------------------------------------
void MMPF_USBPHY_SetBiasCurrentTune(MMP_ULONG ulBiasCurrent)
{
    mBiasCurrent = ulBiasCurrent & BG_BIAS_CURRENT_MASK;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_SetSignalTune
//  Description : Overwrite the default signal tune (Squelch level & HS current)
//------------------------------------------------------------------------------
void MMPF_USBPHY_SetSignalTune(MMP_ULONG ulTxCur, MMP_ULONG ulSQ)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    gbEcoVer = pGBL->_x69F2[7] - 0x30;

    mTxCur = ulTxCur & HS_CUR_MASK;
    //mTxCurPlus = (ulTxCurPlus & HS_CUR_PLUS_MASK) << HS_CUR_PLUS_SHIFT;
    mSqLevel = (ulSQ & SQ_LEVEL_MASK) << SQ_LEVEL_SHIFT;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_AdjustSquelch
//  Description : Adjust USB PHY signals (Squelch level & HS current)
//------------------------------------------------------------------------------
void MMPF_USBPHY_AdjustSignal(void)
{
    #if (CHIP == MCR_V2)
    MMP_USHORT value;

    value = LOWER_AFF_POWER_EN | mTxCur | mTxCurPlus | SQ_FILTER_EN | mSqLevel;
    MMPF_USBPHY_Write(SQ_OPT_HS_TX_OPT, value);
    #endif
}

#if (CHIP == MCR_V2)
//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_RefineHS
//  Description : Refine PHY setting for high speed mode
//------------------------------------------------------------------------------
void MMPF_USBPHY_RefineHS(void)
{
    /* For MercuryV2 MP only */
    if (!mbUsbPhyHsRefined) {
        mbUsbPhyHsRefined = MMP_TRUE;
        if (gbEcoVer == 0) {
            /* Change embedded LDO output 1.1V for better jitter performance */		
            MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT |
                                            BG_OPT1_DEFAULT | LDO_OUT_1d10V);
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2, (mVref | mBiasCurrent) |
                              CHARGE_PUMP_1d5uA | PLL_IMATCH_EN);
        }
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_RollbackHS
//  Description : Rollback PHY setting for high speed mode
//------------------------------------------------------------------------------
void MMPF_USBPHY_RollbackHS(void)
{
    /* For MercuryV2 MP only */
    if (mbUsbPhyHsRefined) {
        mbUsbPhyHsRefined = MMP_FALSE;
        if (gbEcoVer == 0) {	
            /* Rollback embedded LDO output 1.3V due to squelch level bug */	
        	MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT |
                                            BG_OPT1_DEFAULT | LDO_OUT_1d30V);
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2, (BG_VREF_412mV | mBiasCurrent) |
                              CHARGE_PUMP_1d5uA | PLL_IMATCH_EN);
        }
    }
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_Initialize
//  Description : Initalize USB PHY settings
//------------------------------------------------------------------------------
void MMPF_USBPHY_Initialize(MMP_BOOL bOtgEn)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    gbEcoVer = pGBL->_x69F2[7] - 0x30;

    if (!mbUsbPhyInitialized) {

        #if (CHIP == MCR_V2)
        if (gbEcoVer == 0) {
            MMPF_USBPHY_Write(FS_TX_OPT_PLL_CTL4, PLL_CTL4_DEFAULT | FXTX_RESL_42OHM);
        }
        else if (gbEcoVer == 1) {
            MMPF_USBPHY_Write(FS_TX_OPT_PLL_CTL4, PLL_CTL4_DEFAULT |
                                                  FXTX_RESL_42OHM  |
                                                  HSTX_DATA_SAMPLER_DIS);
        }
        /*
         * The default PLL clock freq. in USB PHY is incorrect.
         * To fix this problem, we should modify PLL M value.
         */
        // Power down PLL
        MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, PLL_PWR_DOWN | ANA_TEST_MODE);

            /* For better jitter performance */
        if (gbEcoVer == 0) {
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2, (BG_VREF_412mV | mBiasCurrent) |
                              CHARGE_PUMP_1d5uA | PLL_IMATCH_EN);
        }
        else if (gbEcoVer == 1) {
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2, (mVref | mBiasCurrent) |
                              CHARGE_PUMP_1d5uA | PLL_IMATCH_EN | PLL_FB_CLK);
        }

            MMPF_USBPHY_Write(PLL_CTL3_CTL2, TT_DELAY_1d4nS | PLL_M_24MHZ_DIV_MP);

        mbUsbOtgEn = bOtgEn;

        // Power up PLL again
        if (bOtgEn) // should enable OTG comparators
            MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, 0x0F);
        else
            MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, 0x0000);
        // Wait PLL stable
        MMPF_OS_Sleep(2);
        #endif

        MMPF_USBPHY_PowerUp();
        MMPF_USBPHY_AdjustSignal();

        // Enable USB_INT_DISCON, 21H[7:6] = 2'b11
        MMPF_USBPHY_Write(OTG_DISCONN_SIG_CTL, 0xC000);

        mbUsbPhyInitialized = MMP_TRUE;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_PowerUp
//  Description : Power up USB PHY
//------------------------------------------------------------------------------
void MMPF_USBPHY_PowerUp(void)
{
    MMPF_USBPHY_Write(PWR_CTL_PUPD_TEST, 0x0000);
    if (mbUsbOtgEn) // should enable OTG comparators
        MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, 0x000F);
    else
        MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, 0x0000);

        /* Set Embedded LDO output 1.3V (default 1.2V) due to some samples
         * can't run at high-speed on some PCs. The reason is not clear,
         * Under check by analog designer.
         */
    if (gbEcoVer == 0) {
        MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT |
                                        BG_OPT1_DEFAULT | LDO_OUT_1d30V);
    }
    else if (gbEcoVer == 1) {
        // ECO version always keep in default voltage
        MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT |
                                        BG_OPT1_DEFAULT | LDO_OUT_1d20V);
    }

    mbUsbPhyHsRefined = MMP_FALSE;
    mbUsbPhyInitialized = MMP_TRUE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_PowerDown
//  Description : Power down USB PHY
//------------------------------------------------------------------------------
void MMPF_USBPHY_PowerDown(void)
{
    #if (CHIP == P_V2)
    MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT,
                      TRCV_OPT_DEFAULT | BG_OPT1_DEFAULT | RESISTOR_DIV_OUTPUT);
    #endif
    MMPF_USBPHY_Write(PWR_CTL_PUPD_TEST,
                      DM_PULL_DOWN | DP_PULL_UP1 | HSTX_PWR_DOWN |
                      HSRX_PWR_DOWN | FSLS_TX_PWR_DOWN | FSLS_RX_PWR_DOWN |
                      DISCON_DTC_PWR_DOWN | SQ_DTC_PWR_DOWN |
                      USB_BS_PWR_DOWN | USB_BG_PWR_DOWN);

    MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, PLL_PWR_DOWN | XO_BLOCK_OFF |
                      REFCLK_CORE_OFF | ANA_TEST_MODE | OTG_VREF_PWR_DOWN);

    mbUsbPhyHsRefined = MMP_FALSE;
    /* 
     * Suspend mode will power-off USB phy for leakage issue.
     * We must resume USB phy before using it.
     */
    mbUsbPhyInitialized = MMP_FALSE;
}

EXPORT_SYMBOL(MMPF_USBPHY_PowerDown);

/// @}
