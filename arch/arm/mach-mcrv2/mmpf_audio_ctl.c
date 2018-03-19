#include "config_fw.h"
#include "lib_retina.h"
#include "mmpf_audio.h"
#include "mmpf_i2s_ctl.h"
#include "config_fw.h"
#include "mmpf_pll.h"
#include "mmp_reg_audio.h"

#include "mmpf_audio_ctl.h"

/** @addtogroup MMPF_AUDIO
@{
*/
//++ Patch for 1.8V audio DAC/ADC
MMP_ULONG   glAudPlaySampleRate;

MMP_ULONG 	glRecordHeadMuteTime = 0;
MMP_ULONG 	glRecordTailCutTime = 0;
MMP_BOOL	gbRecordHeadMuteEnable = MMP_FALSE;
MMP_BOOL	gbRecordTailCutEnable = MMP_FALSE;

static MMP_ULONG glAudioSamplerate = 0; //must default initialize
static MMP_ULONG glTrigCnt;
#if (CHIP == MCR_V2)
static MMP_BOOL gbInitTriggerDAC;
#endif
void MMPF_SendInitWave(void);
//-- Patch for 1.8V audio DAC/ADC

MMP_UBYTE    gbUseAitADC;
MMP_UBYTE    gbUseAitDAC;

MMP_UBYTE    gbDACDigitalGain = DEFAULT_DAC_DIGITAL_GAIN;
MMP_UBYTE    gbDACAnalogGain  = DEFAULT_DAC_ANALOG_GAIN;
MMP_UBYTE    gbADCDigitalGain = DEFAULT_ADC_DIGITAL_GAIN;
MMP_UBYTE    gbADCAnalogGain  = DEFAULT_ADC_ANALOG_GAIN;
MMP_UBYTE    gbADCBoostp = MMP_FALSE;
MMP_UBYTE    gbDACLineOutGain = DEFAULT_DAC_LINEOUT_GAIN;

static MMP_BOOL     m_bDACPowerOnState = MMP_FALSE;
static MMP_BOOL     m_bADCPowerOnState = MMP_FALSE;


//#if (OS_TYPE == OS_UCOSII)
#if 1
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetMux
//  Parameter   :
//          path--select path
//          bEnable--1:enable;0:disable
//  Return Value : MMP_ERR_NONE
//  Description : Set audio mux
//------------------------------------------------------------------------------
MMP_ERR  MMPF_Audio_SetMux(MMPF_AUDIO_DATA_FLOW path, MMP_BOOL bEnable)
{
    AITPS_AUD   pAUD    = AITC_BASE_AUD;
    #if (CHIP == P_V2)
    AITPS_AUD   pI2S    = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    AITPS_I2S   pI2S    = AITC_BASE_I2S0;
    #endif
    MMP_ULONG   i       = 0x10;

    switch (path) {
        case AFE_FIFO_TO_DAC:
            #if (CHIP == P_V2)
            if (gbSystemCoreID == CHIP_CORE_ID_PV2P)
                break;
            #endif
            if (bEnable) {
                pAUD->AFE_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pAUD->AFE_FIFO_RST = 0;

                pAUD->AFE_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
                pAUD->AFE_CPU_INT_EN |= AUD_DAC_INT_EN;
                pAUD->AFE_L_CHNL_DATA = 0;
                pAUD->AFE_R_CHNL_DATA = 0;
                pAUD->AFE_MUX_MODE_CTL = AUD_MUX_AUTO;
            }
            else {
                MMPF_Audio_PowerDownDAC(MMP_FALSE);
                pAUD->AFE_L_CHNL_DATA = 0;
                pAUD->AFE_R_CHNL_DATA = 0;
                if (!m_bADCPowerOnState)
                    pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                pAUD->AFE_FIFO_CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
                pAUD->AFE_CPU_INT_EN &= ~(AUD_DAC_INT_EN);
            }
            break;
        case ADC_TO_AFE_FIFO:
        case SDI0_TO_AFE_FIFO:
        #if (CHIP == MCR_V2)
        case SDI1_TO_AFE_FIFO:
        #endif
            #if (CHIP == P_V2)
            if (gbSystemCoreID == CHIP_CORE_ID_PV2P)
                break;
            #endif
            if (bEnable) {
                #if (ADC_ESP_SUPPORT)
                m_bADCInitESP = MMP_TRUE;
                #endif
                pAUD->AFE_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pAUD->AFE_FIFO_RST = 0;

                pAUD->AFE_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNRD_TH;
                if (path == ADC_TO_AFE_FIFO)
                    pAUD->AFE_MUX_MODE_CTL = AUD_MUX_AUTO;
                else if (path == SDI0_TO_AFE_FIFO)
                    pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                #if (CHIP == MCR_V2)
                else if (path == SDI1_TO_AFE_FIFO) {
                    pI2S = AITC_BASE_I2S1;
                    pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                }
                #endif
                pAUD->AFE_CPU_INT_EN |= AUD_ADC_INT_EN;
            }
            else {
                pAUD->AFE_FIFO_CPU_INT_EN = 0;
                if (path == ADC_TO_AFE_FIFO) {
                    if (!m_bDACPowerOnState)
                        pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                }
                else if (path == SDI0_TO_AFE_FIFO) {
                    pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                }
                pAUD->AFE_CPU_INT_EN &= ~(AUD_ADC_INT_EN);
                if (path == ADC_TO_AFE_FIFO)
                    MMPF_Audio_PowerDownADC();
            }
            break;
        #if (CHIP == MCR_V2)
        case I2S1_FIFO_TO_SDO:
            pI2S = AITC_BASE_I2S1;
        #endif
        case I2S0_FIFO_TO_SDO:
            if (bEnable) {
                pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pI2S->I2S_FIFO_RST = 0;

                pI2S->I2S_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
                pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                pI2S->I2S_CPU_INT_EN = AUD_INT_EN;
            }
            else {
                pI2S->I2S_FIFO_CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
                pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                pI2S->I2S_CPU_INT_EN = AUD_INT_DIS;
            }
            break;
        #if (CHIP == MCR_V2)
        case SDI_TO_I2S1_FIFO:
        case ADC_TO_I2S1_FIFO:
            pI2S = AITC_BASE_I2S1;
        #endif
        case SDI_TO_I2S0_FIFO:
        case ADC_TO_I2S0_FIFO:
            if (bEnable) {
                pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pI2S->I2S_FIFO_RST = 0;

                pI2S->I2S_FIFO_CPU_INT_EN |= AUD_INT_FIFO_REACH_UNRD_TH;
                if (path == SDI_TO_I2S0_FIFO)
                    pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                else if (path == ADC_TO_I2S0_FIFO)
                    pAUD->AFE_MUX_MODE_CTL = AUD_MUX_AUTO;
                #if (CHIP == MCR_V2)
                else if (path == SDI_TO_I2S1_FIFO)
                    pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                else if (path == ADC_TO_I2S1_FIFO)
                    pAUD->AFE_MUX_MODE_CTL = AUD_MUX_AUTO;
                #endif
                pI2S->I2S_CPU_INT_EN = AUD_INT_EN;
            }
            else {
                pI2S->I2S_CPU_INT_EN = AUD_INT_DIS;
                pI2S->I2S_FIFO_CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNRD_TH);
                if (path == SDI_TO_I2S0_FIFO) {
                    pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                }
                else if (path == ADC_TO_I2S0_FIFO) {
                    if (!m_bDACPowerOnState)
                        pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                    MMPF_Audio_PowerDownADC();
                }
                #if (CHIP == MCR_V2)
                else if (path == SDI_TO_I2S1_FIFO) {
                    pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                }
                else if (path == ADC_TO_I2S1_FIFO) {
                    if (!m_bDACPowerOnState)
                        pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                    MMPF_Audio_PowerDownADC();
                }
                #endif
            }
            break;
    }
    return  MMP_ERR_NONE;
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InitializePlayFIFO
//  Parameter   :
//          path-- select path
//          threshold -- fifo int threshold
//  Return Value : None
//  Description : Init Audio Output Fifo
//------------------------------------------------------------------------------
MMP_ERR    MMPF_Audio_InitializePlayFIFO(MMP_USHORT usPath, MMP_USHORT usThreshold)
{
    AITPS_AUD   pAUD    = AITC_BASE_AUD;
    #if (CHIP == P_V2)
    AITPS_AUD   pI2S    = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    AITPS_I2S   pI2S    = AITC_BASE_I2S0;
    #if (CHIP == MCR_V2)
    AITPS_I2S   pI2S1    = AITC_BASE_I2S1;
    #endif
    #endif
    MMP_ULONG   i;

    switch (usPath) {
    #if (CHIP == MCR_V2)
    case I2S1_FIFO_TO_SDO:
        pI2S = AITC_BASE_I2S1;
    #endif
    case I2S0_FIFO_TO_SDO:
        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
        pI2S->I2S_FIFO_RST = 0;

        pI2S->I2S_FIFO_WR_TH = usThreshold;
        #if (CHIP == MCR_V2)
        pI2S1->I2S_FIFO_RD_TH = usThreshold;
        #endif
        for (i = 0; i < 256; i += 4) {
            pI2S->I2S_FIFO_DATA = 0;
            pI2S->I2S_FIFO_DATA = 0;
            pI2S->I2S_FIFO_DATA = 0;
            pI2S->I2S_FIFO_DATA = 0;
        }
        break;
    case AFE_FIFO_TO_DAC:
        pAUD->AFE_FIFO_RST = AUD_FIFO_RST_EN;
        pAUD->AFE_FIFO_RST = 0;

        pAUD->AFE_FIFO_WR_TH = usThreshold;
        for (i = 0; i < 256; i += 4) {
            pAUD->AFE_FIFO_DATA = 0;
            pAUD->AFE_FIFO_DATA = 0;
            pAUD->AFE_FIFO_DATA = 0;
            pAUD->AFE_FIFO_DATA = 0;
        }
        break;
    }

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_InitializePlayFIFO);

#if (OS_TYPE == OS_UCOSII)
#pragma arm section code, rodata, rwdata, zidata
#endif
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InitializeEncodeFIFO
//  Parameter   :
//          path-- select path
//          threshold -- fifo int threshold
//  Return Value : None
//  Description : Init audio input fifo
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_InitializeEncodeFIFO(MMP_USHORT usPath, MMP_USHORT usThreshold)
{
    AITPS_AUD   pAUD    = AITC_BASE_AUD;
    #if (CHIP == P_V2)
    AITPS_AUD   pI2S    = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    AITPS_I2S   pI2S    = AITC_BASE_I2S0;
    #endif

    switch (usPath) {
    #if (CHIP == MCR_V2)
    case SDI_TO_I2S1_FIFO:
    case ADC_TO_I2S1_FIFO:
        pI2S = AITC_BASE_I2S1;
    #endif
    case SDI_TO_I2S0_FIFO:
    case ADC_TO_I2S0_FIFO:
        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
        pI2S->I2S_FIFO_RST = 0;

        pI2S->I2S_FIFO_RD_TH = usThreshold;
        break;
    case ADC_TO_AFE_FIFO:
    case SDI0_TO_AFE_FIFO:
    #if (CHIP == MCR_V2)
    case SDI1_TO_AFE_FIFO:
    #endif
        pAUD->AFE_FIFO_RST = AUD_FIFO_RST_EN;
        pAUD->AFE_FIFO_RST = 0;

        pAUD->AFE_FIFO_RD_TH = usThreshold;
        break;
    }
    return MMP_ERR_NONE;

}
EXPORT_SYMBOL_GPL(MMPF_Audio_InitializeEncodeFIFO);

//------------------------------------------------------------------------------
//  Function    : MMPF_TWave
//  Parameter   : the poind index of wave
//  Return Value : trigger signal
//  Description : Get a trigger signal
//------------------------------------------------------------------------------
MMP_SHORT MMPF_TWave(MMP_ULONG idx)
{
    MMP_SHORT tmp;
    MMP_ULONG TWaveSignalFreq = 20;
    MMP_ULONG TWaveSampPerCyc = glAudPlaySampleRate/TWaveSignalFreq;
    MMP_ULONG TWaveSegment = TWaveSampPerCyc/4;
    MMP_ULONG TWaveDelay = 32767/TWaveSegment;

    idx = idx % TWaveSampPerCyc;
    if(idx < TWaveSegment)
        tmp = idx;
    else if(idx < 2*TWaveSegment)
        tmp = ((TWaveSegment<<1) - idx);
    else if(idx < 3*TWaveSegment)
        tmp = -(idx-(TWaveSegment << 1));
    else
        tmp = -(TWaveSampPerCyc - idx);

    tmp = tmp * TWaveDelay;

    return tmp;
}

#if 0 // for IDE function name list by section
void _______AUDIO_HARDWARE______(){}
#endif

//#if (OS_TYPE == OS_UCOSII)
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDuplexPath
//  Parameter   : None
//  Return Value : None
//  Description : Enable or disable codec/I2S full duplex path
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDuplexPath(MMPF_AUDIO_DATA_FLOW path, MMP_BOOL bEnable)
{
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    AITPS_I2S pI2S = AITC_BASE_I2S0;
    #endif

    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    if (bEnable) {
        if (path == ADC_TO_I2S0_FIFO)
            pI2S->DUPLEX_PATH_SEL |= AFE_FULL_DUPLEX_I2S0_EN;
        else if (path == SDI0_TO_AFE_FIFO)
            pI2S->DUPLEX_PATH_SEL |= I2S0_FULL_DUPLEX_EN;
        else if (path == ADC_TO_I2S1_FIFO)
            pI2S->DUPLEX_PATH_SEL |= AFE_FULL_DUPLEX_I2S1_EN;
        else if (path == SDI1_TO_AFE_FIFO)
            pI2S->DUPLEX_PATH_SEL |= I2S1_FULL_DUPLEX_EN;
    }
    else {
        if (path == ADC_TO_I2S0_FIFO)
            pI2S->DUPLEX_PATH_SEL &= ~(AFE_FULL_DUPLEX_I2S0_EN);
        else if (path == SDI0_TO_AFE_FIFO)
            pI2S->DUPLEX_PATH_SEL &= ~(I2S0_FULL_DUPLEX_EN);
        else if (path == ADC_TO_I2S1_FIFO)
            pI2S->DUPLEX_PATH_SEL &= ~(AFE_FULL_DUPLEX_I2S1_EN);
        else if (path == SDI1_TO_AFE_FIFO)
            pI2S->DUPLEX_PATH_SEL &= ~(I2S1_FULL_DUPLEX_EN);
    }
    #endif

    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetVoiceOutPath
//  Parameter   : None
//  Return Value : None
//  Description : Set Voice Out Path
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetVoiceOutPath(MMP_UBYTE path)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;

    gbUseAitDAC = path;

	if (!m_bDACPowerOnState)
		return MMP_ERR_NONE;

    //MMPF_OS_Sleep(100);
 
    if (gbUseAitDAC & AUDIO_OUT_AFE_LINE) {
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL |= LINEOUT_POWER_UP;
        #endif
    }
   
    return MMP_ERR_NONE;
}
//#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetVoiceInPath
//  Parameter   : None
//  Return Value : None
//  Description : Set Voice In Path
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetVoiceInPath(MMP_UBYTE path)
{
    #if (CHIP == MCR_V2)
	AITPS_AUD pAUD = AITC_BASE_AUD;
    #endif

	gbUseAitADC = path;

    #if (CHIP == MCR_V2)
    if (gbUseAitADC == AUDIO_IN_AFE_SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL=LPGA_SRC_IN_AUXL;
        pAUD->AFE_ADC_ANA_RPGA_CTL=RPGA_SRC_IN_AUXR;
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF){
        pAUD->AFE_ADC_ANA_LPGA_CTL  =   LPGA_SRC_IN_MICLIP_LIN;
        pAUD->AFE_ADC_ANA_RPGA_CTL  =   RPGA_SRC_IN_MICRIP_RIN;
        pAUD->AFE_MICBIAS_DUAL_CHAN =   ADC_MIC_BIAS_R_POWER_UP | ADC_MIC_BIAS_R_VOLT_0D65 | ADC_MIC_BIAS_L_POWER_UP | ADC_MIC_BIAS_L_VOLT_0D65;
        pAUD->AFE_ANA_GAIN_SETTING_CTL  = ANA_GAIN_DIRECT_METHOD | TIME_OUT_PULSE_ENA;
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL=LPGA_SRC_IN_MICLIP;
        pAUD->AFE_ADC_ANA_RPGA_CTL=RPGA_SRC_IN_MICRIP;
        pAUD->AFE_MICBIAS_DUAL_CHAN =   ADC_MIC_BIAS_R_POWER_UP | ADC_MIC_BIAS_R_VOLT_0D65 | ADC_MIC_BIAS_L_POWER_UP | ADC_MIC_BIAS_L_VOLT_0D65;
        pAUD->AFE_ANA_GAIN_SETTING_CTL  = ANA_GAIN_DIRECT_METHOD | TIME_OUT_PULSE_ENA;
    }
    #endif

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetVoiceInPath);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACDigitalGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDACDigitalGain(MMP_UBYTE gain)
{
    #if (CHIP == MCR_V2)
	AITPS_AUD   pAUD = AITC_BASE_AUD;
	MMP_USHORT  usGain = gain;
    #endif
    
    #if (CHIP == MCR_V2)
    gbDACDigitalGain = usGain & DAC_DIG_GAIN_MASK;
    pAUD->AFE_DAC_DIG_GAIN = (gbDACDigitalGain << 8) | gbDACDigitalGain;
    #endif

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACAnalogGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDACAnalogGain(MMP_UBYTE gain)
{
    #if (CHIP == MCR_V2)
	AITPS_AUD pAUD = AITC_BASE_AUD;
    #endif
	
	#if (CHIP == MCR_V2)
	if (gbUseAitDAC & AUDIO_OUT_AFE_LINE) {
	   gbDACLineOutGain = gain;
       pAUD->AFE_DAC_LOUT_VOL = gbDACLineOutGain<<1;  
	}
	#endif
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetADCDigitalGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetADCDigitalGain(MMP_UBYTE gain, MMP_BOOL bRecordSetting)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;
	
	if (bRecordSetting) {
    	gbADCDigitalGain = gain;
    }
    
    #if (CHIP == P_V2)
    if ((gbSystemCoreID == CHIP_CORE_ID_PV2) || (gbSystemCoreID == CHIP_CORE_ID_PV2_ECO))
        pAUD->AFE_ADC_GAIN_LR_VOL = (gain << 8)|gain;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == VSN_V3)

    pAUD->AFE_ADC_DIG_GAIN = (gain << 8)|gain;
    #endif
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetADCAnalogGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetADCAnalogGain(MMP_UBYTE gain, MMP_BOOL boostup, MMP_BOOL bRecordSetting)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;

    if (bRecordSetting) {
    	gbADCAnalogGain = gain;
    	gbADCBoostp = boostup;
    }

    #if (CHIP == MCR_V2) || (CHIP == VSN_V3)
    pAUD->AFE_ADC_ANA_LPGA_GAIN = gain;
    pAUD->AFE_ADC_ANA_RPGA_GAIN = gain;
    #endif

    #if (CHIP == VSN_V3)
    if (boostup) {
        if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING)) {
            //#if (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_20DB)
            //pAUD->AFE_ADC_BOOST_CTL = (MIC_LCH_BOOST(MIC_BOOST_20DB) | MIC_RCH_BOOST(MIC_BOOST_20DB));
            //#elif (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_30DB)
            pAUD->AFE_ADC_BOOST_CTL = (MIC_LCH_BOOST(MIC_BOOST_30DB) | MIC_RCH_BOOST(MIC_BOOST_30DB));
            //#elif (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_40DB)
            //pAUD->AFE_ADC_BOOST_CTL = (MIC_LCH_BOOST(MIC_BOOST_40DB) | MIC_RCH_BOOST(MIC_BOOST_40DB));
            //#endif
        }
        //if (gbUseAitADC == AUDIO_IN_AFE_SING) {
        //    not supported;
    }
    else {
        if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING)) {
            pAUD->AFE_ADC_BOOST_CTL = MIC_NO_BOOST;
        }
        //if (gbUseAitADC == AUDIO_IN_AFE_SING)
        //    not supported;
    }
    #endif

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_TriggerDAC
//  Parameter   : None
//  Return Value : None
//  Description : Trigger DAC
//------------------------------------------------------------------------------
void MMPF_TriggerDAC(void)
{
    #if (CHIP == MCR_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;

    glTrigCnt = 0;
    gbInitTriggerDAC = MMP_TRUE;

    MMPF_Audio_InitializePlayFIFO(AFE_FIFO_TO_DAC, MP3_I2S_FIFO_WRITE_THRESHOLD);

    //MMPF_Audio_SetMux(AFE_FIFO_TO_DAC, MMP_TRUE);
    //MMPF_OS_Sleep(310); //Wait 310ms = 300ms(Write FIFO) + 10ms(ADC ready)

    if (!m_bADCPowerOnState)
        pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
    pAUD->AFE_FIFO_CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
    pAUD->AFE_CPU_INT_EN &= ~(AUD_DAC_INT_EN);

    gbInitTriggerDAC = MMP_FALSE;
    #endif
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SendInitWave
//  Parameter   : None
//  Return Value : None
//  Description : Send some wave data to AFE FIFO when init DAC
//------------------------------------------------------------------------------
void MMPF_SendInitWave(void)
{
	MMP_SHORT  tmp;
	MMP_USHORT WriteCnt;
	MMP_ULONG  i;
	AITPS_AUD  pAUD = AITC_BASE_AUD;
	MMP_ULONG  TWaveTotalLen = (glAudPlaySampleRate*6)/20; //300ms

	if(pAUD->AFE_FIFO_WR_TH < (TWaveTotalLen-glTrigCnt))	
		WriteCnt = pAUD->AFE_FIFO_WR_TH;
	else
		WriteCnt = TWaveTotalLen-glTrigCnt;

    if(glTrigCnt < TWaveTotalLen) {
		for(i = 0; i < WriteCnt; i+=2) {
			tmp = MMPF_TWave(glTrigCnt++);		
			pAUD->AFE_FIFO_DATA = tmp;
		    pAUD->AFE_FIFO_DATA = tmp;
		}	
    }
	else {
		for(i = 0; i < pAUD->AFE_FIFO_WR_TH; i+=2) {
			pAUD->AFE_FIFO_DATA = 0x00;
		    pAUD->AFE_FIFO_DATA = 0x00;
		}	
	}	
}    

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACAnalogGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_ResetAfeFifo(void)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    MMP_ULONG i = 0x10;

    pAUD->AFE_FIFO_RST = AUD_FIFO_RST_EN;
    while(i--); // wait for one cycle of audio moudle clock
    pAUD->AFE_FIFO_RST = 0;

    return MMP_ERR_NONE;
}

//#if (OS_TYPE == OS_UCOSII)
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerOnDAC
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerOnDAC(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;

    glAudPlaySampleRate = samplerate;
	MMPF_Audio_SetDACFreq(glAudPlaySampleRate);

	if (m_bDACPowerOnState) {
		return MMP_ERR_NONE;
    }
    m_bDACPowerOnState = MMP_TRUE;
    #if (CHIP == MCR_V2)
    pAUD->AFE_OVF_BUGFIX |= DAC_DIG_VOL_OVF_FIX | DAC_SDM_OVF_FIX;
    #endif

    
    
    #if (CHIP == P_V2)
    pAUD->AFE_CLK_CTL |= DAC_CLK_USB_MODE;
    pAUD->AFE_CLK_CTL |= (CODEC_RESET_DIS_V1 | DAC_CLK_INV_EN);
    #endif
    #if (CHIP == MCR_V2)   
    //pAUD->AFE_CLK_CTL |= (DAC_CLK_INV_EN);
    pAUD->AFE_CLK_CTL |= DAC_CLK_256FS_MODE;
    #endif

	if (gbUseAitDAC & AUDIO_OUT_AFE_HP) {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS;
	}
	else {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS | DAC_DITHER_EN | DAC_DITHER_SCALE_2_1;
	}

    #if (CHIP == P_V2)
	pAUD->AFE_DAC_L_DIG_GAIN = DAC_SOFT_MUTE;
	pAUD->AFE_DAC_R_DIG_GAIN = DAC_SOFT_MUTE;
    #endif
    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN_CTL = DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE;
    #endif
    
    #if (CHIP == P_V2)
    pAUD->AFE_DAC_HP_VOL = 0x00;
    pAUD->AFE_DAC_LOUT_VOL = 0x00;
    if (!m_bADCPowerOnState) {
        pAUD->AFE_ANA_BIAS = ANA_GLOBAL_BIAS(0);
    }
    #endif
    
    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_LOUT_VOL |= 0x00<<1;
    #endif
    
    #if (CHIP == P_V2)
    pAUD->AFE_ANA_CTL &= ~(ANA_HP_REF_OP_EN);
    if (gbUseAitDAC & AUDIO_OUT_AFE_LINE)
        pAUD->AFE_ANA_CTL_5 = DAC_OP_LINE(1);
    #endif

    pAUD->AFE_POWER_CTL |= ANALOG_POWER_EN;
	
    MMPF_OS_Sleep(50); // origianl 300
    //MMPF_OS_Sleep(300);
    pAUD->AFE_POWER_CTL |= VREF_POWER_EN;
    
    //if(gbAudioDACFastCharge)
        MMPF_OS_Sleep(150); // origianl 100
    //else
    //    MMPF_OS_Sleep(2000); // origianl 100

    #if (CHIP == P_V2)
    pAUD->AFE_POWER_CTL |= DAC_ANALOG_LPF_POWER_EN;
    #endif
    
    MMPF_OS_Sleep(1); // not be checked    
    
    pAUD->AFE_POWER_CTL |= DAC_DF_POWER_EN;
    
    MMPF_OS_Sleep(1); // original 10
    
    #if (CHIP == P_V2)
	pAUD->AFE_POWER_CTL |= ANALOG_DAC_POWER_EN;
    #endif
    
    #if (CHIP == MCR_V2)
    pAUD->AFE_ANA_DAC_POWER_CTL |=DAC_POWER_UP;
    #endif
     MMPF_OS_Sleep(1); // original 10
    //MMPF_OS_Sleep(100);
    
    MMPF_TriggerDAC();
     
    MMPF_OS_Sleep(10);	 // origianl 100						
    //MMPF_OS_Sleep(100);

    if (gbUseAitDAC & AUDIO_OUT_AFE_LINE) {
        pAUD->AFE_DAC_LOUT_VOL = gbDACLineOutGain;
        #if (CHIP == P_V2)
        pAUD->AFE_POWER_CTL |= DAC_LINE_OUT_CTL_EN;
        #endif
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL |= LINEOUT_POWER_UP;
        #endif
    }
    else {
        #if (CHIP == P_V2)
        pAUD->AFE_POWER_CTL &= ~(DAC_LINE_OUT_CTL_EN);
        #endif
        
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL &= ~ LINEOUT_POWER_UP;
        #endif
    }
    
    #if (CHIP == P_V2)
    if (gbUseAitDAC & AUDIO_OUT_AFE_HP) {
        pAUD->AFE_ANA_CTL |= ANA_HP_REF_OP_EN;
        pAUD->AFE_DAC_HP_VOL = gbDACAnalogGain;
        pAUD->AFE_POWER_CTL |= PGA_DAC_POWER_EN;
    }
    else {
        pAUD->AFE_POWER_CTL &= ~(PGA_DAC_POWER_EN);
        pAUD->AFE_ANA_CTL &= ~(ANA_HP_REF_OP_EN);
    }
    #endif
    MMPF_OS_Sleep(100);

    #if (CHIP == P_V2)
	pAUD->AFE_DAC_L_DIG_GAIN = gbDACDigitalGain;
	pAUD->AFE_DAC_R_DIG_GAIN = gbDACDigitalGain;
    #endif
    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN = (gbDACDigitalGain << 8) | gbDACDigitalGain;
    pAUD->AFE_DAC_DIG_GAIN_CTL &= ~(DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE);
    #endif

    return MMP_ERR_NONE;
}


//#endif


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerOnADC
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerOnADC(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;

    #if (AUDIO_CODEC_DUPLEX_EN == 1)
    #else
    // if DAC is still power-on, power off it first
    if (m_bDACPowerOnState) {
        MMPF_Audio_PowerDownDAC(MMP_TRUE);
    }
    #endif
    m_bADCPowerOnState = MMP_TRUE;

    pAUD->AFE_ADC_LOOP_CTL = 0;

    #if (CHIP == P_V2)
    pAUD->AFE_ADC_GAIN_LR_VOL   = (gbADCDigitalGain << 8) | gbADCDigitalGain;
    pAUD->AFE_ADC_GAIN_PGA_GAIN = gbADCAnalogGain;
    #endif

    #if (CHIP == MCR_V2)
    pAUD->AFE_ADC_DIG_GAIN = (gbADCDigitalGain << 8) | gbADCDigitalGain;
    pAUD->AFE_ADC_ANA_LPGA_GAIN = gbADCAnalogGain;
    pAUD->AFE_ADC_ANA_RPGA_GAIN = gbADCAnalogGain;
    #endif

    #if (CHIP == P_V2)
    pAUD->AFE_HPF_CTL = ADC_HPF_EN;

    pAUD->AFE_ANA_BIAS &= ~(ANA_GLOBAL_BIAS_MASK|ANA_ADC_CONT_OP_MASK|ANA_ADC_DISC_OP_MASK);
    pAUD->AFE_ANA_BIAS |= ANA_GLOBAL_BIAS(4)|ANA_ADC_CONT_OP(1)|ANA_ADC_DISC_OP(1);
    pAUD->AFE_ANA_CTL &= ~(ANA_ADC_CHOPPER_EN);
    #endif 

    if ((pAUD->AFE_POWER_CTL & ANALOG_POWER_EN) == 0) {
        pAUD->AFE_POWER_CTL |= ANALOG_POWER_EN;
        msleep(10);
    }
    if ((pAUD->AFE_POWER_CTL & VREF_POWER_EN) == 0) {
        pAUD->AFE_POWER_CTL |= VREF_POWER_EN;
        msleep(10);
    }
    #if (CHIP == P_V2)
    if ((pAUD->AFE_POWER_CTL & ADC_PGA_POWER_EN) == 0) {
        pAUD->AFE_POWER_CTL |= ADC_PGA_POWER_EN;
        msleep(20);
    }
    if ((pAUD->AFE_POWER_CTL & ANALOG_ADC_POWER_EN) == 0) {
        pAUD->AFE_POWER_CTL |= ANALOG_ADC_POWER_EN;
        msleep(10);
    }
    #endif

    #if (CHIP == MCR_V2)
    if ((pAUD->AFE_ANA_ADC_POWER_CTL & (ADC_PGA_L_POWER_EN|ADC_PGA_R_POWER_EN)) == 0) {
        pAUD->AFE_ANA_ADC_POWER_CTL |= (ADC_PGA_L_POWER_EN|ADC_PGA_R_POWER_EN);
        msleep(20);
        pAUD->AFE_ANA_ADC_POWER_CTL |= (ADC_R_POWER_UP|ADC_L_POWER_UP);
    }
    #endif
    
    if ((pAUD->AFE_POWER_CTL & ADC_DF_POWER_EN) == 0) {
        pAUD->AFE_POWER_CTL |= ADC_DF_POWER_EN;
        msleep(10);
    }

    //Control ADC mic L/R boost up, mic bias
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING)) {
        #if (CHIP == P_V2)
        #if (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_NONE)
        pAUD->AFE_ADC_BOOST_CTL &= MIC_LR_BOOST_MASK;
        #elif (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_20DB)
        pAUD->AFE_ADC_BOOST_CTL = (pAUD->AFE_ADC_BOOST_CTL & MIC_LR_BOOST_MASK) | MIC_R_BOOST_20DB | MIC_L_BOOST_20DB;
        #elif (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_30DB)
        pAUD->AFE_ADC_BOOST_CTL = (pAUD->AFE_ADC_BOOST_CTL & MIC_LR_BOOST_MASK) | MIC_R_BOOST_30DB | MIC_L_BOOST_30DB;
        #elif (DEFAULT_ADC_BOOSTUP == ADC_BOOSTUP_40DB)
        pAUD->AFE_ADC_BOOST_CTL = (pAUD->AFE_ADC_BOOST_CTL & MIC_LR_BOOST_MASK) | MIC_R_BOOST_40DB | MIC_L_BOOST_40DB;
        #endif

        #if (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_NONE)
        pAUD->AFE_ANA_CTL_4 = 0; // power down mic bias
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d65AVDD)
        pAUD->AFE_ANA_CTL_4 = (pAUD->AFE_ANA_CTL_4 & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_VOLT_d65 | ADC_MIC_BIAS_EN;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d75AVDD)
        pAUD->AFE_ANA_CTL_4 = (pAUD->AFE_ANA_CTL_4 & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_VOLT_d75 | ADC_MIC_BIAS_EN;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d95AVDD)
        pAUD->AFE_ANA_CTL_4 = (pAUD->AFE_ANA_CTL_4 & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_VOLT_d95 | ADC_MIC_BIAS_EN;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_1d15AVDD)
        pAUD->AFE_ANA_CTL_4 = (pAUD->AFE_ANA_CTL_4 & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_VOLT_1d15 | ADC_MIC_BIAS_EN;
        #endif
        #endif //(CHIP == P_V2)
     
        #if (CHIP == MCR_V2)
        #if (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_NONE)
           pAUD->AFE_MICBIAS_DUAL_CHAN = 0; // power down mic bias
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d6AVDD)
           pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_R_VOLT_0D6| ADC_MIC_BIAS_L_VOLT_0D6 ;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d65AVDD)
           pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_R_VOLT_0D65| ADC_MIC_BIAS_L_VOLT_0D65 ;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d75AVDD)
            pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
                                ADC_MIC_BIAS_R_VOLT_0D75| ADC_MIC_BIAS_L_VOLT_0D75 ;
        #elif (DEFAULT_ADC_MIC_BIAS == ADC_MIC_BIAS_0d85AVDD)
            pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
                                 ADC_MIC_BIAS_R_VOLT_0D85| ADC_MIC_BIAS_L_VOLT_0D85 ;
        #endif
        #endif //(CHIP == MCR_V2)

    }

    //Control ADC in path
    #if (CHIP == P_V2)
    if (gbUseAitADC == AUDIO_IN_AFE_SING)
        pAUD->AFE_ADC_INPUT_SEL = (pAUD->AFE_ADC_INPUT_SEL & ADC_CTL_MASK_V1) | ADC_AUX_IN_V1;
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF)
        pAUD->AFE_ADC_INPUT_SEL = (pAUD->AFE_ADC_INPUT_SEL & ADC_CTL_MASK_V1) | ADC_MIC_IN_V1 | ADC_MIC_DIFF_V1;
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING)
        pAUD->AFE_ADC_INPUT_SEL = (pAUD->AFE_ADC_INPUT_SEL & ADC_CTL_MASK_V1) | ADC_MIC_IN_V1 | ADC_MIC_DIFF2SINGLE_V1;
    #endif

    #if (CHIP == MCR_V2)
    if (gbUseAitADC == AUDIO_IN_AFE_SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL=LPGA_SRC_IN_AUXL;
        pAUD->AFE_ADC_ANA_RPGA_CTL=RPGA_SRC_IN_AUXR;
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF){
        pAUD->AFE_ADC_ANA_LPGA_CTL=LPGA_SRC_IN_MICLIP_LIN;
        pAUD->AFE_ADC_ANA_RPGA_CTL=RPGA_SRC_IN_MICRIP_RIN;
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL=LPGA_SRC_IN_MICLIP;
        pAUD->AFE_ADC_ANA_RPGA_CTL=RPGA_SRC_IN_MICRIP;
    }
    #endif

    #if (CHIP == P_V2)
    pAUD->AFE_CLK_CTL |= ADC_CLK_USB_MODE;
    pAUD->AFE_CLK_CTL |= (ADC_CLK_INV_EN | CODEC_RESET_DIS_V1);
    #endif

    #if (CHIP == MCR_V2)
    //pAUD->AFE_CLK_CTL |= ADC_CLK_INV_EN;
    pAUD->AFE_DIG_GAIN_SETTING_CTL = 0x30;
    #endif
    MMPF_Audio_SetADCFreq(samplerate);
    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetPLL
//  Parameter   : 
//      ulSamplerate -- sampling rate
//  Return Value : None
//  Description : Dynamic change PLL for audio DAC.
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetPLL(MMP_UBYTE ubPath, MMP_ULONG ulSamplerate)
{
    MMP_ERR         err;
    MMPF_PLL_MODE   pll_mode;
    MMPF_GROUP_SRC  pll_src;
    
    #if (HDMI_OUTPUT_EN == 0) || (AUDIO_CODEC_DUPLEX_EN == 0)
    if (glAudioSamplerate != ulSamplerate ) {
    #endif
        switch(ulSamplerate) {
        case 192000:
            pll_mode = MMPF_PLL_AUDIO_192K;
            break;
        case 128000:
            pll_mode = MMPF_PLL_AUDIO_128K;
            break;
        case 96000:
            pll_mode = MMPF_PLL_AUDIO_96K;
            break;
        case 64000:
            pll_mode = MMPF_PLL_AUDIO_64K;
            break;
        case 48000:
            pll_mode = MMPF_PLL_AUDIO_48K;
            break;
        case 44100:
            pll_mode = MMPF_PLL_AUDIO_44d1K;
            break;
        case 32000:
            pll_mode = MMPF_PLL_AUDIO_32K;
            break;
        case 24000:
            pll_mode = MMPF_PLL_AUDIO_24K;
            break;
        case 22050:
            pll_mode = MMPF_PLL_AUDIO_22d05K;
            break;
        case 16000:
            pll_mode = MMPF_PLL_AUDIO_16K;
            break;
        case 12000:
            pll_mode = MMPF_PLL_AUDIO_12K;
            break;
        case 11025:
            pll_mode = MMPF_PLL_AUDIO_11d025K;
            break;
        case 8000:
            pll_mode = MMPF_PLL_AUDIO_8K;
            break;
        default:
            RTNA_DBG_Str0("Unsupported audio sample rate!\r\n");
            return MMP_AUDIO_ERR_PARAMETER;
            break;
        }

        err = MMPF_PLL_GetGroupSrc(MMPF_CLK_GRP_AUDIO, &pll_src);
        if (err != MMP_ERR_NONE) {
            RTNA_DBG_Str0("Get Audio group source failed!\r\n");
            return err;
        }
        err = MMPF_PLL_SetAudioPLL(MMPF_AUDSRC_MCLK, pll_mode, ubPath);           

        if (err != MMP_ERR_NONE) {
            RTNA_DBG_Str0("Set Audio PLL frequency failed!\r\n");
            return err;
        }
        #if (AUDIO_CODEC_DUPLEX_EN == 1)
        if((ubPath & AUDIO_AFE_MASK)< 0x04 )
        	glAudRecSampleRate = ulSamplerate;
        if((ubPath & AUDIO_AFE_MASK)>= 0x04 )
        	glAudPlaySampleRate = ulSamplerate;                
        #if (VR_AVSYNC_METHOD == VR_AVSYNC_REF_VID)
        MMPF_VIDMGR_SetAVSyncAudSampleRate(glAudRecSampleRate);
        #endif
        #else
        glAudioSamplerate = ulSamplerate;
        #if (VR_AVSYNC_METHOD == VR_AVSYNC_REF_VID)
        MMPF_VIDMGR_SetAVSyncAudSampleRate(glAudioSamplerate);
        #endif
		#endif
    #if (HDMI_OUTPUT_EN == 0)  || (AUDIO_CODEC_DUPLEX_EN == 0) 
    }
    #endif
  
    return MMP_ERR_NONE;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACFreq
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDACFreq(MMP_ULONG samplerate)
{
    #if (CHIP == MCR_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;
    AITPS_I2S pI2S = AITC_BASE_I2S0;

    if (samplerate <= 48000) {
        #if (HIGH_SRATE_MODE == DOWN_SAMPLE_TIMES)
        pI2S->AFE_DOWN_SAMPLE_SEL = DOWN_SAMPLE_OFF;
        #elif (HIGH_SRATE_MODE == BYPASS_FILTER_STAGE)
        pAUD->AFE_SAMPLE_RATE_SEL = SRATE_48000Hz_UNDER;
        #endif
    }

    // NOTE: if both AD & DA are working, their sample rate shoule be identical
    // due to the HW limitation.
    switch(samplerate) {
	case 48000:
        pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_48000HZ)|ADC_SRATE(SRATE_48000HZ);
		break;	
	case 44100:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_44100HZ)|ADC_SRATE(SRATE_44100HZ);
		break;	
	case 32000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_32000HZ)|ADC_SRATE(SRATE_32000HZ);
		break;	
	case 24000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_24000HZ)|ADC_SRATE(SRATE_24000HZ);
		break;	
	case 22050:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_22050HZ)|ADC_SRATE(SRATE_22050HZ);
		break;	
	case 16000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_16000HZ)|ADC_SRATE(SRATE_16000HZ);
		break;	
	case 12000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_12000HZ)|ADC_SRATE(SRATE_12000HZ);
		break;	
	case 11025:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_11025HZ)|ADC_SRATE(SRATE_11025HZ);
		break;
	case 8000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_8000HZ)|ADC_SRATE(SRATE_8000HZ);
		break;	
	}
    #endif

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetADCFreq
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetADCFreq(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;

    #if (AUDIO_CODEC_DUPLEX_EN == 1)
    glAudRecSampleRate = samplerate;
    #endif

    // NOTE: if both AD & DA are working, their sample rate shoule be identical
    // due to the HW limitation.
    switch(samplerate) {
	case 48000:
        pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_48000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_48000HZ)
            #endif
                ;
		break;	
	case 44100:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_44100HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_44100HZ)
            #endif
                    ;
		break;	
	case 32000:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_32000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_32000HZ)
            #endif
                    ;
		break;	
	case 24000:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_24000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_24000HZ)
            #endif
                    ;
		break;	
	case 22050:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_22050HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_22050HZ)
            #endif
                    ;
		break;	
	case 16000:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_16000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_16000HZ)
            #endif
                    ;
		break;	
	case 12000:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_12000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_12000HZ)
            #endif
                    ;
		break;	
	case 11025:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_11025HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_11025HZ)
            #endif
                    ;
		break;
	case 8000:
		pAUD->AFE_SAMPLE_RATE_CTL = ADC_SRATE(SRATE_8000HZ)
            #if (CHIP == MCR_V2)
                | DAC_SRATE(SRATE_8000HZ)
            #endif
                    ;
		break;	
	}

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerDownDAC
//  Parameter   : bPowerDownNow -- Truly power down DAC
//  Return Value : None
//  Description : Power down DAC only when bPowerDownNow is set to true
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerDownDAC(MMP_BOOL bPowerDownNow)
{
    #if (CHIP == MCR_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;
    #endif

    if (!bPowerDownNow) {
        // not really power down DAC, 
        // keep DAC in power on state to avoid pop-noise again
        return MMP_ERR_NONE;
    }
     
    #if (CHIP == MCR_V2)
    pAUD->AFE_POWER_CTL &= ~(DAC_DF_POWER_EN);
    pAUD->AFE_ANA_DAC_POWER_CTL &= ~(DAC_POWER_UP | LINEOUT_POWER_UP);
    #endif
    
    m_bDACPowerOnState = MMP_FALSE;

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerDownADC
//  Parameter   : None
//  Return Value : None
//  Description : Power down ADC
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerDownADC(void)
{
    #if (CHIP == MCR_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;

    pAUD->AFE_POWER_CTL &= ~(ADC_DF_POWER_EN);
    pAUD->AFE_ANA_ADC_POWER_CTL &= ~( ADC_PGA_L_POWER_EN | ADC_PGA_R_POWER_EN);
    #endif

    m_bADCPowerOnState = MMP_FALSE;

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerDownCodec
//  Parameter   : None
//  Return Value : None
//  Description : Power down ADC & DAC
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerDownCodec(void)
{
    #if (CHIP == MCR_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;
    #endif

    if (!m_bDACPowerOnState && !m_bADCPowerOnState)
        return MMP_ERR_NONE;

    #if (CHIP == P_V2)
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING))
        pAUD->AFE_ANA_CTL_4 &= ~(ADC_MIC_BIAS_EN);
    #endif
    #if (CHIP == MCR_V2)
    pAUD->AFE_POWER_CTL = AFE_POWER_OFF;
    #endif
  
    m_bDACPowerOnState = MMP_FALSE;
    m_bADCPowerOnState = MMP_FALSE;

    return MMP_ERR_NONE;
}

MMP_ERR MMPF_Audio_EnableAFEClock(MMP_BOOL bEnableClk, MMP_ULONG SampleRate)
{
    #if (CHIP == VSN_V3)
    AITPS_GBL pGBL = AITC_BASE_GBL;

    if (bEnableClk == MMP_TRUE) {
        pGBL->GBL_CLK_DIS2 &= (~ GBL_CLK_AUD_CODEC_DIS);    // (bit0) enable codec clock

        //pGBL->GBL_AUDIO_CLK_DIV = 0xAA;
        pGBL->GBL_AUDIO_CLK_DIV = GBL_AUDIO_CLK_SRC_DPLL2|GBL_AUDIO_CLK_ON|(GBL_AUDIO_CLK_DIV_MASK&10);
        //under USB mode, g0 = 96M
        if (SampleRate > 24000)
            pGBL->GBL_ADC_CLK_DIV = 0x0331;
        else if (SampleRate > 12000)
        {
            pGBL->GBL_ADC_CLK_DIV = 0x0773;
        }
        else
            RTNA_DBG_Str(0, "WARNING: Need to lower audio source clock");
            //pGBL->GBL_ADC_CLK_DIV = 0x0FF7;
        //pGBL->GBL_ADC_CLK_DIV = 0x0333;                       // audio codec clock divide by 4, ADC DF clock divide by 4, both get 96/4 = 24Mhz
                                                            // audio ADC HBF clock divide by 4, and get 96/4 = 24Mhz
    }
    else {
        pGBL->GBL_CLK_DIS2 |= GBL_CLK_AUD_CODEC_DIS;    // (bit0) enable codec clock
    }
    #endif

    return MMP_ERR_NONE;
}

MODULE_LICENSE("GPL");
/** @} */ // end of MMPF_AUDIO
