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
#include <mach/hardware.h>
#include <mach/mmpf_typedef.h>
#if 1// (USB_EN)
#include <mach/mmp_reg_usb.h>
#include "mmpf_usbphy.h"


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

static MMP_ULONG    mTxCur = HS_CUR_400mV;
static MMP_ULONG    mSqLevel = SQ_LEVEL_75mV;

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
//  Function    : MMPF_USBPHY_SetSignalTune
//  Description : Overwrite the default signal tune (Squelch level & HS current)
//------------------------------------------------------------------------------
void MMPF_USBPHY_SetSignalTune(MMP_ULONG ulTxCur, MMP_ULONG ulSQ)
{
    mTxCur = ulTxCur & HS_CUR_MASK;
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

    value = LOWER_AFF_POWER_EN | mTxCur | SQ_FILTER_EN | mSqLevel;
    MMPF_USBPHY_Write(SQ_OPT_HS_TX_OPT, value);
    #endif
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_Initalize
//  Description : Initalize USB PHY settings
//------------------------------------------------------------------------------
void MMPF_USBPHY_Initialize(MMP_BOOL bOtgEn)
{
    if (!mbUsbPhyInitialized) {
        #if (CHIP == P_V2)
        if (gbSystemCoreID == CHIP_CORE_ID_PV2) {
            MMPF_USBPHY_Write(FS_TX_OPT_PLL_CTL4,
                              FXTX_RESL_45OHM | HSTX_DATA_SAMPLER_DIS);
        }
        #endif
        #if (CHIP == MCR_V2)
        MMPF_USBPHY_Write(FS_TX_OPT_PLL_CTL4, PLL_CTL4_DEFAULT |
                          FXTX_RESL_45OHM | HSTX_DATA_SAMPLER_DIS);
        /*
         * The default PLL clock freq. in USB PHY is incorrect.
         * To fix this problem, we should modify PLL M value.
         */
        // Power down PLL
        MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, PLL_PWR_DOWN | ANA_TEST_MODE);
#if (OS_TYPE == OS_UCOSII)
        if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
            // Modify PLL_CTL1[7, 3:0]
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2,
                              BG_OPT2_DEFAULT | PLL_CTL1_DEFAULT);
            // PLL_M from reg, modify PLL_CTL2[5:0], PLL_CTL3[5:0]
            MMPF_USBPHY_Write(PLL_CTL3_CTL2, PLL_M_REG_CTL |
                              PLL_M_12MHZ_DIV | PLL_CTL3_DEFAULT);
        }
        else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
            /* The default PLL setting should be correct.
             * We don't need to do anything but just power down
             * and power up PLL again because the initial clock
             * is 12MHz instead of 24MHz.
             */
        }
#else
            MMPF_USBPHY_Write(PLL_CTL1_BG_OPT2,
                              BG_OPT2_DEFAULT | PLL_CTL1_DEFAULT);
            // PLL_M from reg, modify PLL_CTL2[5:0], PLL_CTL3[5:0]
            MMPF_USBPHY_Write(PLL_CTL3_CTL2, PLL_M_REG_CTL |
                              PLL_M_12MHZ_DIV | PLL_CTL3_DEFAULT);
#endif

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
    MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, 0x0000);
#if (OS_TYPE == OS_UCOSII)	
    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
        MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT | BG_OPT1_DEFAULT);
    }
    else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
        /* Set Embedded LDO output 1.3V (default 1.2V) due to some samples
         * can't run at high-speed on some PCs. The reason is not clear,
         * Under check by analog designer.
         */
        MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT |
                                            BG_OPT1_DEFAULT | LDO_OUT_1d30V);
    }
#else
        MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT, TRCV_OPT_DEFAULT | BG_OPT1_DEFAULT);

#endif
    mbUsbPhyInitialized = MMP_TRUE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_USBPHY_PowerDown
//  Description : Power down USB PHY
//------------------------------------------------------------------------------
void MMPF_USBPHY_PowerDown(void)
{
    MMPF_USBPHY_Write(BG_OPT1_TRCV_OPT,
                      TRCV_OPT_DEFAULT | BG_OPT1_DEFAULT | RESISTOR_DIV_OUTPUT);
    MMPF_USBPHY_Write(PWR_CTL_PUPD_TEST,
                      DM_PULL_DOWN | DP_PULL_UP1 | HSTX_PWR_DOWN |
                      HSRX_PWR_DOWN | FSLS_TX_PWR_DOWN | FSLS_RX_PWR_DOWN |
                      DISCON_DTC_PWR_DOWN | SQ_DTC_PWR_DOWN |
                      USB_BS_PWR_DOWN | USB_BG_PWR_DOWN);
    #if (CHIP == P_V2)
    MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, ANA_TEST_MODE | OTG_VREF_PWR_DOWN);
    #endif
    #if (CHIP == MCR_V2)
    MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, PLL_PWR_DOWN | XO_BLOCK_OFF |
                      REFCLK_CORE_OFF | ANA_TEST_MODE | OTG_VREF_PWR_DOWN);
    #endif

    /* 
     * Suspend mode will power-off USB phy for leakage issue.
     * We must resume USB phy before using it.
     */
    mbUsbPhyInitialized = MMP_FALSE;
}
EXPORT_SYMBOL(MMPF_USBPHY_PowerDown);
#endif  //(USB_EN)

/// @}
