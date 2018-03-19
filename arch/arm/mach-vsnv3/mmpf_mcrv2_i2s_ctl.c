#include "includes_fw.h"
#include "lib_retina.h"	
#include "mmp_reg_gbl.h"
#include "mmp_reg_audio.h"
#include "mmpf_audio_ctl.h"
#include "mmpf_mcrv2_i2s_ctl.h"
#include "mmpf_pll.h"

/** @addtogroup MMPF_I2S
@{
*/
#define I2S_CH_NUM 3
static MMPF_AUDIO_I2S_MCLK_MODE m_mclkMode[I2S_CH_NUM] = {MMPF_AUDIO_I2S_MCLK_MODE_NONE,MMPF_AUDIO_I2S_MCLK_MODE_NONE,MMPF_AUDIO_I2S_MCLK_MODE_NONE };
static MMP_ULONG m_ulMclkFixedFreq[I2S_CH_NUM] = {0,0,0}; //in unit of KHz

//------------------------------------------------------------------------------
//  Function    : MMPF_I2S_SetMclkFreq
//  Parameter   :
//          freq--sampling rate in unit of Hz
//  Return Value : None
//  Description : Set i2s mclk for the specified sample rate
//------------------------------------------------------------------------------
void MMPF_I2S_SetMclkFreq(MMP_UBYTE ch, MMP_ULONG fs)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    #if (CHIP == P_V2)
    AITPS_AUD   pAUD = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2)
    AITPS_I2S   pAUD;
    #endif
    MMP_ULONG   audClk = 0;
    MMP_UBYTE   clkDiv;
    #if (CHIP == P_V2)
    MMP_UBYTE ratioM, ratioN;
    #endif

    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_AUDIO, &audClk); //in unit of KHz
    #if (CHIP == P_V2)
    if (pGBL->GBL_CLK_AUD_DIV)
        audClk = audClk/pGBL->GBL_CLK_AUD_DIV;
    #endif
    #if (CHIP == MCR_V2)
    if (pGBL->GBL_AUD_CLK_DIV[0] & 0x01) {
        audClk = audClk/(((pGBL->GBL_AUD_CLK_DIV[0] & 0x3E) >> 1) + 1);
    }
    pAUD = (ch == I2S_CH0) ? AITC_BASE_I2S0 : AITC_BASE_I2S1;
    #endif

    if (m_mclkMode[ch] == MMPF_AUDIO_I2S_USB_MODE) {
        if (audClk % m_ulMclkFixedFreq[ch]) {
            //RTNA_DBG_Str0("!!! Audio clock [");
            //RTNA_DBG_Long0(audClk);
            //RTNA_DBG_Str0("] is not a mulitple of fixed Mclk [");
            //RTNA_DBG_Long0(m_ulMclkFixedFreq[ch]);
            //RTNA_DBG_Str0("]\r\n");
            printk("!! Audio does not a mulitple of fixed MCLK %d.\r\n",m_ulMclkFixedFreq[ch]);
        }
        pAUD->I2S_CLK_DIV = audClk/m_ulMclkFixedFreq[ch];
        pAUD->I2S_RATIO_N_M = 0;
        pAUD->I2S_CLK_CTL = I2S_MCLK_FIX;
    }
    else if (m_mclkMode[ch] == MMPF_AUDIO_I2S_FIX_256FS_MODE) {
        clkDiv = (MMP_UBYTE)((audClk * 1000)/(256 * fs));
        pAUD->I2S_CLK_DIV = clkDiv;
        pAUD->I2S_RATIO_N_M = 0;
        pAUD->I2S_CLK_CTL = I2S_MCLK_FIX;
    }
    #if (CHIP == P_V2)   
    else if (m_mclkMode[ch] == MMPF_AUDIO_I2S_256FS_MODE) {
        pAUD->I2S_CLK_CTL &= ~(I2S_MCLK_FIX);

        #if (CHIP == P_V2)
        clkDiv = (MMP_UBYTE)((audClk * 1000)/(256 * fs));
        ratioM = (MMP_UBYTE)((audClk * 1000)/fs - clkDiv * 256);
        ratioN = 256 - ratioM;

        pAUD->I2S_CLK_DIV = clkDiv;
        pAUD->I2S_RATIO_N_M = ratioM << 8 | ratioN;
        #endif
    }
    #endif
    return;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_I2S_SetMclkMode
//  Parameter   :
//          mclk_mode--
//  Return Value : None
//  Description : Set i2s MCLK mode
//------------------------------------------------------------------------------
void MMPF_I2S_SetMclkMode(MMP_UBYTE ch, MMP_ULONG mclk_mode)
{
    if (ch < I2S_CH_NUM)
        m_mclkMode[ch] = mclk_mode;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_I2S_GetMclkMode
//  Parameter   :
//          mclk_mode--current MCLK mode
//  Return Value : None
//  Description : Get i2s MCLK mode
//------------------------------------------------------------------------------
void MMPF_I2S_GetMclkMode(MMP_UBYTE ch, MMPF_AUDIO_I2S_MCLK_MODE *mclk_mode)
{
    if (mclk_mode && (ch < I2S_CH_NUM))
        *mclk_mode = m_mclkMode[ch];
}
//------------------------------------------------------------------------------
//  Function    : MMPF_I2S_SetUsbModeFreq
//  Parameter   :
//          mclk_freq--
//  Return Value : None
//  Description : Set i2s MCLK fixed frequency
//------------------------------------------------------------------------------
void MMPF_I2S_SetUsbModeFreq(MMP_UBYTE ch, MMP_ULONG mclk_freq)
{
    if (ch < I2S_CH_NUM)
        m_ulMclkFixedFreq[ch] = mclk_freq;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_I2S_EnableDataOutput
//  Parameter   :
//          bEnable--specify i2s output enable or disable
//  Return Value : None
//  Description : i2s output control
//------------------------------------------------------------------------------
void MMPF_I2S_EnableDataOutput(MMP_UBYTE ch, MMP_BOOL bEnable)
{
    #if (CHIP == P_V2)
    AITPS_AUD   pAUD = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2)
    AITPS_I2S   pAUD = (ch == I2S_CH0) ? AITC_BASE_I2S0 : ((ch == I2S_CH1) ? AITC_BASE_I2S1: AITC_BASE_I2S2_MP) ;
    #endif

    if (bEnable) {
    	pAUD->I2S_CTL |= I2S_SDO_OUT_EN;
        pAUD->I2S_DATA_OUT_EN |= 0x01;
    }
    else {
        pAUD->I2S_DATA_OUT_EN &= ~(0x01);
        pAUD->I2S_CTL &= ~(I2S_SDO_OUT_EN);
    }
}
/** @} */ // end of MMPF_I2S

void MMPF_I2S_PADSET(MMP_UBYTE ch, MMP_BOOL pad)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    if (ch==I2S_CH0) {
        pGBL->GBL_I2S_DMIC_CFG &=~  GBL_I2S0_PAD_EN;
        pGBL->GBL_I2S_DMIC_CFG |=   GBL_I2S0_PAD_EN;
    }
    else if (ch==I2S_CH1) {
        pGBL->GBL_I2S_DMIC_CFG &=~  GBL_I2S1_PAD_MASK;
        pGBL->GBL_I2S_DMIC_CFG |=   GBL_I2S1_PAD(pad);    
    }
    else if(ch==I2S_CH2) {
        pGBL->GBL_I2S_DMIC_CFG &=~  GBL_I2S2_PAD_MASK;
        pGBL->GBL_I2S_DMIC_CFG |=   GBL_I2S2_PAD(pad);
    }
}