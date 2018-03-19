#include <linux/module.h>
#include <mach/mmpf_pll.h>

#include "includes_fw.h"
#include "config_fw.h"
#include "lib_retina.h"
#include "mmpf_audio.h"
#include "mmpf_i2s_ctl.h"
#include "config_fw.h"
#include "mmpf_pll.h"
#include "mmp_register.h"
#include "mmp_reg_audio.h"
#include "mmpf_mcrv2_audio_ctl.h"
#include "audio_eq_cfg.h"

/** @addtogroup MMPF_AUDIO
@{
*/
//++ Patch for 1.8V audio DAC/ADC
MMP_ULONG   glAudPlaySampleRate;

MMP_ULONG 	glRecordHeadMuteTime = 0;
MMP_ULONG 	glRecordTailCutTime = 0;
MMP_BOOL	gbRecordHeadMuteEnable = MMP_FALSE;
MMP_BOOL	gbRecordTailCutEnable = MMP_FALSE;

//static MMP_ULONG glAudioSamplerate = 0; //must default initialize
static MMP_ULONG glAudPllRefCount = 0; //must default initialize

static MMP_ULONG glTrigCnt;
#if (CHIP == MCR_V2)
static MMP_BOOL gbInitTriggerDAC;
#endif
void MMPF_SendInitWave(void);
//-- Patch for 1.8V audio DAC/ADC

MMP_UBYTE    gbUseAitADC;
static void SetADC_Usage(MMP_UBYTE status)
{
	gbUseAitADC = status;
	dbg_printf(0,"SET ADC usage 0x%X \r\n",gbUseAitADC);
}
static MMP_UBYTE GetADC_Usage ()
{
	return gbUseAitADC;
}

MMP_UBYTE    gbUseAitDAC;
static void SetDAC_Usage(MMP_UBYTE status)
{
	gbUseAitDAC = status;
	dbg_printf(0,"SET DAC usage 0x%X \r\n",gbUseAitDAC);
}
static MMP_UBYTE GetDAC_Usage ()
{
	return gbUseAitDAC;
}

MMP_UBYTE    gbDACDigitalGain = DEFAULT_DAC_DIGITAL_GAIN;
//MMP_UBYTE    gbDACAnalogGain  = DEFAULT_DAC_ANALOG_GAIN;
MMP_UBYTE    gbADCDigitalGain = DEFAULT_ADC_DIGITAL_GAIN;
MMP_UBYTE    gbADCAnalogGain  = DEFAULT_ADC_ANALOG_GAIN;
MMP_UBYTE    gbADCBoostp = MMP_FALSE;
MMP_UBYTE    gbDACLineOutGain = DEFAULT_DAC_LINEOUT_GAIN;
MMP_UBYTE    gbADCBoost = 40 ; // Change 1 or 0 to 20dB,30dB,40dB for Python_v2

/*static*/ MMP_BOOL     m_bDACPowerOnState = MMP_FALSE;
/*static*/ MMP_BOOL     m_bADCPowerOnState = MMP_FALSE;

MMP_BOOL    gbAudioDACFastCharge = MMP_TRUE;


#if (CHIP == MCR_V2)
#define DIG_STEP_TIME       3
#define MACRO_STEP_TIME     2
#define MACRO_TIMEOUT       2
#endif


AITPS_AUD   g_pAUD = AITC_BASE_AUD;
//int gbSystemCoreID = CHIP_CORE_ID_MCR_V2_MP;
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
    #if (CHIP == MCR_V2)
    AITPS_I2S   pI2S     = AITC_BASE_I2S0;
    #endif
    MMP_ULONG   i       = 0x10;

    #if (CHIP == MCR_V2)
    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
    #endif
	    switch (path) {
	        case AFE_FIFO_TO_DAC:

	            if (bEnable) {
	                pAUD->AFE_FIFO.SHT.RST = AUD_FIFO_RST_EN;
	                while(i--); // wait for one cycle of audio moudle clock
	                pAUD->AFE_FIFO.SHT.RST = 0;

	                pAUD->AFE_FIFO.SHT.CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
	                pAUD->AFE_CPU_INT_EN |= AUD_DAC_INT_EN;
	                pAUD->AFE_L_CHNL_DATA = 0;
	                pAUD->AFE_R_CHNL_DATA = 0;
	                #if (CHIP == MCR_V2)
					pAUD->AFE_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;
	            	#endif
	            }
	            else {
	                MMPF_Audio_PowerDownDAC(MMP_FALSE);
					pAUD->AFE_L_CHNL_DATA = 0;
	                pAUD->AFE_R_CHNL_DATA = 0;
	                #if (CHIP == MCR_V2)
	                if (!m_bADCPowerOnState)
	                    pAUD->AFE_MUX_MODE_CTL &= ~(AFE_MUX_AUTO_MODE);
	                #endif
	                pAUD->AFE_FIFO.SHT.CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
	                pAUD->AFE_CPU_INT_EN &= ~(AUD_DAC_INT_EN);
	            }
	            break;
	        case ADC_TO_AFE_FIFO:
	        case SDI0_TO_AFE_FIFO:
	        #if (CHIP == MCR_V2)
	        case SDI1_TO_AFE_FIFO:
	        #endif

	            if (bEnable) {
	                #if (ADC_ESP_SUPPORT)
	                m_bADCInitESP = MMP_TRUE;
	                #endif                                                            
	                pAUD->AFE_FIFO.SHT.CPU_INT_EN |= AUD_INT_FIFO_REACH_UNRD_TH;
	                #if (CHIP == MCR_V2)                          
	                if (path == ADC_TO_AFE_FIFO)                    
	                    pAUD->AFE_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;                             
	                else if (path == SDI0_TO_AFE_FIFO)
	                    pI2S->I2S_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;
	                else if (path == SDI1_TO_AFE_FIFO) {
	                    pI2S = AITC_BASE_I2S1;
	                    pI2S->I2S_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;
	                }
	                #endif
	                                             
	                pAUD->AFE_FIFO.SHT.RST = AUD_FIFO_RST_EN;
	                while(i--); // wait for one cycle of audio moudle clock
	                pAUD->AFE_FIFO.SHT.RST = 0;             
	                pAUD->AFE_CPU_INT_EN |= AUD_ADC_INT_EN;                                              
	            }
	            else {
	                pAUD->AFE_FIFO.SHT.CPU_INT_EN = 0;
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
	}
	#if (CHIP == MCR_V2)
	else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
		switch (path) {
        case AFE_FIFO_TO_DAC: //AFE_TX_FIFO_TO_DAC:
        case AFE_TX_FIFO_TO_DAC:
            if (bEnable) {
                //pAUD->AFE_FIFO.DAC_MP.RST = AUD_FIFO_RST_EN;
                //while(i--); // wait for one cycle of audio moudle clock
                //pAUD->AFE_FIFO.DAC_MP.RST = 0;

                pAUD->AFE_FIFO.DAC_MP.CPU_INT_EN |= AUD_INT_FIFO_REACH_UNWR_TH;
                pAUD->AFE_CPU_INT_EN |= AUD_DAC_INT_EN;
                pAUD->AFE_L_CHNL_DATA = 0;
                pAUD->AFE_R_CHNL_DATA = 0;	               
		  pAUD->AFE_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;
            }
            else {
                MMPF_Audio_PowerDownDAC(MMP_FALSE);
		  pAUD->AFE_L_CHNL_DATA = 0;
                pAUD->AFE_R_CHNL_DATA = 0;
                if (!m_bADCPowerOnState)
                    pAUD->AFE_MUX_MODE_CTL &= ~(AFE_MUX_AUTO_MODE);
                pAUD->AFE_FIFO.DAC_MP.CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
                pAUD->AFE_CPU_INT_EN &= ~(AUD_DAC_INT_EN);
            }
            break;
        case ADC_TO_AFE_FIFO: //ADC_TO_AFE_RX_FIFO:
        case ADC_TO_AFE_RX_FIFO:
            if (bEnable) {
                #if (ADC_ESP_SUPPORT)
                m_bADCInitESP = MMP_TRUE;
                #endif                                                            
                pAUD->ADC_FIFO.MP.CPU_INT_EN |= AUD_INT_FIFO_REACH_UNRD_TH;
                pAUD->AFE_MUX_MODE_CTL = AFE_MUX_AUTO_MODE;                             

                //pAUD->ADC_FIFO.MP.RST = AUD_FIFO_RST_EN;
                //while(i--); // wait for one cycle of audio moudle clock
                //pAUD->ADC_FIFO.MP.RST = 0;
                
                pAUD->AFE_CPU_INT_EN |= AUD_ADC_INT_EN;                                              
            }
            else {
                pAUD->ADC_FIFO.MP.CPU_INT_EN = 0;
                if (!m_bDACPowerOnState)
                    pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                pAUD->AFE_CPU_INT_EN &= ~(AUD_ADC_INT_EN);
                MMPF_Audio_PowerDownADC();
            }
            break;
        case I2S2_TX_FIFO_TO_SDO:
        	 pI2S = AITC_BASE_I2S2_MP;
        	 goto I2S_SDO_PATH;
        case I2S1_TX_FIFO_TO_SDO:
        	pI2S = AITC_BASE_I2S1;
        	goto I2S_SDO_PATH;
        case I2S0_TX_FIFO_TO_SDO:
        	pI2S = AITC_BASE_I2S0;
        	I2S_SDO_PATH:
            if (bEnable) {
                pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pI2S->I2S_FIFO_RST = 0;

                pI2S->I2S_FIFO_CPU_INT_EN |= TX_INT_EN_MP(AUD_INT_FIFO_REACH_UNWR_TH);
                pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                pI2S->I2S_CPU_INT_EN = AUD_INT_EN;
            }
            else {
                pI2S->I2S_FIFO_CPU_INT_EN &= ~TX_INT_EN_MP(AUD_INT_FIFO_REACH_UNWR_TH);
                pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
                pI2S->I2S_CPU_INT_EN = AUD_INT_DIS;
            }
            break;
        case SDI_TO_I2S0_RX_FIFO:
            pI2S = AITC_BASE_I2S0;
            goto I2S_SDI_PATH;
        case SDI_TO_I2S1_RX_FIFO:
            pI2S = AITC_BASE_I2S1;
            goto I2S_SDI_PATH;
        case SDI_TO_I2S2_RX_FIFO:
            pI2S = AITC_BASE_I2S2_MP;
            I2S_SDI_PATH:
            if (bEnable) {
                pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
                while(i--); // wait for one cycle of audio moudle clock
                pI2S->I2S_FIFO_RST = 0;

                pI2S->I2S_FIFO_CPU_INT_EN |= RX_INT_EN_MP(AUD_INT_FIFO_REACH_UNRD_TH);
                pI2S->I2S_MUX_MODE_CTL = AUD_MUX_AUTO;
                pI2S->I2S_CPU_INT_EN = AUD_INT_EN;
            }
            else {
                pI2S->I2S_CPU_INT_EN = AUD_INT_DIS;
                pI2S->I2S_FIFO_CPU_INT_EN &= ~RX_INT_EN_MP(AUD_INT_FIFO_REACH_UNRD_TH);
                pI2S->I2S_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);
            }
            break;
    	}
	}
	#endif
    return  MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetMux);
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
    #if (CHIP == MCR_V2)
    //known issuse(only for shuttle): I2S0 R/W fifo threshold map to I2S1 W/R fifo threshold 
    //                                (as the same as I2S1) 
    //work around: remapping fifo threshold OPR 
    AITPS_I2S   pI2S    = AITC_BASE_I2S0;
    AIT_REG_D   *AFE_FIFO_DATA,*I2S_FIFO_DATA;
    #endif
    
    MMP_ULONG   i=16;

	#if (CHIP == MCR_V2)
    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
    	I2S_FIFO_DATA=(AIT_REG_D *)&pI2S->I2S_FIFO.SHT.DATA;
    	AFE_FIFO_DATA=(AIT_REG_D *)&pAUD->AFE_FIFO.SHT.DATA;
    }
    else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
    	I2S_FIFO_DATA=(AIT_REG_D *)&pI2S->I2S_FIFO.MP.DATA_TX;
    	AFE_FIFO_DATA=(AIT_REG_D *)&pAUD->AFE_FIFO.DAC_MP.DATA;
	}
    #endif
    
    #if (CHIP == MCR_V2)
    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
    #endif
	    switch (usPath) {
		#if (CHIP == MCR_V2)
	    case I2S1_FIFO_TO_SDO:
	        pI2S = AITC_BASE_I2S1;
	        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	        pI2S->I2S_FIFO_RST = 0;
	        pI2S = AITC_BASE_I2S0;       
	        pI2S->I2S_FIFO.SHT.RD_TH = usThreshold;
	        break;
		#endif
	    case I2S0_FIFO_TO_SDO:
	        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	        pI2S->I2S_FIFO_RST = 0;       
		#if (CHIP == P_V2)
	        pI2S->I2S_FIFO_WR_TH = usThreshold;
		#endif
		#if (CHIP == MCR_V2)
	        pI2S    = AITC_BASE_I2S1;
	        pI2S->I2S_FIFO.SHT.RD_TH = usThreshold;
		#endif
	        for (i = 0; i < 256; i += 4) {
	            *I2S_FIFO_DATA = 0;
	            *I2S_FIFO_DATA = 0;
	            *I2S_FIFO_DATA = 0;
	            *I2S_FIFO_DATA = 0;
	        }
	        break;
	    case AFE_FIFO_TO_DAC:
	        pAUD->AFE_FIFO.SHT.RST = AUD_FIFO_RST_EN;
	        pAUD->AFE_FIFO.SHT.RST = 0;
	    
	        pAUD->AFE_FIFO.DAC_MP.WR_TH = usThreshold;
	        for (i = 0; i < 512; i += 4) {
	            *AFE_FIFO_DATA = 0;
	            *AFE_FIFO_DATA = 0;
	            *AFE_FIFO_DATA = 0;
	            *AFE_FIFO_DATA = 0;
	        }
	        break;
	}
	#if (CHIP == MCR_V2)
	}else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
		switch (usPath) {
		    case I2S2_TX_FIFO_TO_SDO:
		        pI2S = AITC_BASE_I2S2_MP;
		        break;
			case I2S1_TX_FIFO_TO_SDO:
		        pI2S = AITC_BASE_I2S1;
		        break;
	        case I2S0_TX_FIFO_TO_SDO:
	            pI2S = AITC_BASE_I2S0;
		        break;
		case AFE_FIFO_TO_DAC:
		case AFE_TX_FIFO_TO_DAC:
		        pAUD->AFE_FIFO.DAC_MP.RST = AUD_FIFO_RST_EN;
			 while(i--); // wait for one cycle of audio moudle clock
		        pAUD->AFE_FIFO.DAC_MP.RST = 0;
		    
		        pAUD->AFE_FIFO.DAC_MP.WR_TH = usThreshold;
		        for (i = 0; i < 512; i += 4) {
		            *AFE_FIFO_DATA = 0;
		            *AFE_FIFO_DATA = 0;
		            *AFE_FIFO_DATA = 0;
		            *AFE_FIFO_DATA = 0;
		        }
		        break;
		}
		
		switch (usPath) {
		    case I2S2_TX_FIFO_TO_SDO:
			case I2S1_TX_FIFO_TO_SDO:
	        case I2S0_TX_FIFO_TO_SDO:
	            pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	            pI2S->I2S_FIFO_RST = 0;
	            pI2S->I2S_FIFO_WR_TH_TX_MP = usThreshold;
	            for (i = 0; i < 512; i += 4) {
	                *I2S_FIFO_DATA = 0;
	                *I2S_FIFO_DATA = 0;
	                *I2S_FIFO_DATA = 0;
	                *I2S_FIFO_DATA = 0;
	            }   
	            break;
		
		}
	}
	#endif
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_InitializePlayFIFO);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InitializeEncodeFIFO
//  Parameter   :
//          path-- select path
//          threshold -- fifo int threshold
//  Return Value : None
//  Description : Init audio input fifo
//------------------------------------------------------------------------------
MMP_ERR	MMPF_Audio_InitializeEncodeFIFO(MMP_USHORT usPath, MMP_USHORT usThreshold)
{
    AITPS_AUD   pAUD    = AITC_BASE_AUD;
    #if (CHIP == P_V2)
    AITPS_AUD   pI2S    = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2)
    //known issuse: fifo threshold OPR address mapping is wrong (only for shuttle)
    //work around: I2S0 R/W fifo threshold  map to I2S1 W/R fifo threshold (as the same as I2S1)
    AITPS_I2S   pI2S     = AITC_BASE_I2S0;
    int i=15;
    #endif
	if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT || CHIP == P_V2) {
	    switch (usPath) {
	    #if (CHIP == MCR_V2)
	    case SDI_TO_I2S1_FIFO:
	    case ADC_TO_I2S1_FIFO:
	        pI2S = AITC_BASE_I2S1;
	        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	        pI2S->I2S_FIFO_RST = 0;
	        pI2S = AITC_BASE_I2S0;
	        pI2S->I2S_FIFO.SHT.WR_TH   =   usThreshold;
	        break;
	    #endif
	    case SDI_TO_I2S0_FIFO:
	    case ADC_TO_I2S0_FIFO:
	        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	        pI2S->I2S_FIFO_RST = 0;
	        #if (CHIP == P_V2)
	        pI2S->I2S_FIFO_RD_TH    	=   usThreshold;
	        #endif
	        #if (CHIP == MCR_V2)
	        pI2S     = AITC_BASE_I2S1;
	        pI2S->I2S_FIFO.SHT.WR_TH   	=   usThreshold;
	        #endif
	        break;
	    case ADC_TO_AFE_FIFO:
	    case SDI0_TO_AFE_FIFO:
	    #if (CHIP == MCR_V2)
	    case SDI1_TO_AFE_FIFO:
	    #endif
	        pAUD->AFE_FIFO.SHT.RST = AUD_FIFO_RST_EN;
		 while(i--); // wait for one cycle of audio moudle clock
	        pAUD->AFE_FIFO.SHT.RST = 0;
	        pAUD->AFE_FIFO.SHT.RD_TH = usThreshold;
	        break;
	    }
 	} else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
 		switch (usPath) {
	    case SDI_TO_I2S2_RX_FIFO:
	    	pI2S = AITC_BASE_I2S2_MP;
	    	goto I2S_INI_ENC_FIFO;
	    case SDI_TO_I2S1_RX_FIFO:
	    	pI2S = AITC_BASE_I2S1;
	    	goto I2S_INI_ENC_FIFO;
        case SDI_TO_I2S0_RX_FIFO:
        	pI2S = AITC_BASE_I2S0;
	    	I2S_INI_ENC_FIFO:
	        pI2S->I2S_FIFO_RST = AUD_FIFO_RST_EN;
	        pI2S->I2S_FIFO_RST = 0;
	        pI2S->I2S_FIFO.MP.RD_TH_RX   =   usThreshold;
	        break;
	   	case ADC_TO_AFE_RX_FIFO:
	    case AFE_FULL_DUPLEX:
	        pAUD->ADC_FIFO.MP.RST = AUD_FIFO_RST_EN;
	        pAUD->ADC_FIFO.MP.RST = 0;
	        pAUD->ADC_FIFO.MP.RD_TH = usThreshold;
	        break;
	    case ADC_TO_AFE_FIFO:
	        pAUD->ADC_FIFO.MP.RST = AUD_FIFO_RST_EN;
		 while(i--);
	        pAUD->ADC_FIFO.MP.RST = 0;
	        pAUD->ADC_FIFO.MP.RD_TH = usThreshold;
	        break;
	    }
 	}
    
    return MMP_ERR_NONE;

}
EXPORT_SYMBOL(MMPF_Audio_InitializeEncodeFIFO);

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


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDuplexPath
//  Parameter   : None
//  Return Value : None
//  Description : Enable or disable codec/I2S full duplex path
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDuplexPath(MMPF_AUDIO_DATA_FLOW path, MMP_BOOL bEnable)
{
    #if (CHIP == P_V2)
    AITPS_AUD pAUD = AITC_BASE_AUD;
    #endif
    #if (CHIP == MCR_V2)
    AITPS_I2S pI2S = AITC_BASE_I2S0;
    #endif

    #if (CHIP == P_V2)
    if (bEnable) {
        if (path == ADC_TO_I2S0_FIFO)
            pAUD->DUPLEX_PATH_SEL |= CODEC_FULL_DUPLEX_EN;
        else if (path == SDI0_TO_AFE_FIFO)
            pAUD->DUPLEX_PATH_SEL |= I2S_FULL_DUPLEX_EN;
    }
    else {
        if (path == ADC_TO_I2S0_FIFO)
            pAUD->DUPLEX_PATH_SEL &= ~(CODEC_FULL_DUPLEX_EN);
        else if (path == SDI0_TO_AFE_FIFO)
            pAUD->DUPLEX_PATH_SEL &= ~(I2S_FULL_DUPLEX_EN);
    }
    #endif
    #if (CHIP == MCR_V2)
    if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_SHT) {
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
	}
	else if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
	    if (bEnable) {
	        if (path == I2S2_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL |= I2S2_FULL_DUPLEX_EN_MP;
	        else if (path == I2S1_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL |= I2S1_FULL_DUPLEX_EN_MP;
	        else if (path == I2S0_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL |= I2S0_FULL_DUPLEX_EN_MP;
	    }
	    else {
	        if (path == I2S2_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL &= ~(I2S2_FULL_DUPLEX_EN_MP);
	        else if (path == I2S1_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL &= ~(I2S1_FULL_DUPLEX_EN_MP);
	        else if (path == I2S0_FULL_DUPLEX)
	            pI2S->DUPLEX_PATH_SEL &= ~(I2S0_FULL_DUPLEX_EN_MP);
	    }
		
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

    //gbUseAitDAC = path;
    SetDAC_Usage(path);

	if (!m_bDACPowerOnState)
		return MMP_ERR_NONE;

    #if (CHIP == P_V2)
	if (GetDAC_Status() & AUDIO_OUT_AFE_HP_INVT) {
        pAUD->AFE_ANA_DAC_CONFIG |= HP_OUT_INV;
    }
    else {
        pAUD->AFE_ANA_DAC_CONFIG &= ~HP_OUT_INV;
    }
	if (GetDAC_Status() & AUDIO_OUT_AFE_LINE_INVT) {
        pAUD->AFE_ANA_DAC_CONFIG |= LINE_OUT_INV;
    }
    else {
        pAUD->AFE_ANA_DAC_CONFIG &= ~LINE_OUT_INV;
    }
    #endif
	
    //MMPF_OS_Sleep(100);
 
    if (GetDAC_Usage() & AUDIO_OUT_AFE_LINE) {
        #if (CHIP == P_V2)
        pAUD->AFE_POWER_CTL |= DAC_LINE_OUT_CTL_EN;
        #endif
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL |= LINEOUT_POWER_UP;
        #endif
    }
    else {
        #if (CHIP == P_V2)
        pAUD->AFE_POWER_CTL &= ~DAC_LINE_OUT_CTL_EN;
        #endif
    }
    
    #if (CHIP == P_V2)  
    if (GetDAC_Status() & AUDIO_OUT_AFE_HP) {
        #if (CHIP == P_V2)
        pAUD->AFE_ANA_CTL |= ANA_HP_REF_OP_EN;
        pAUD->AFE_DAC_HP_VOL = gbDACAnalogGain;
		pAUD->AFE_POWER_CTL |= PGA_DAC_POWER_EN;
        #endif
    }else {
        #if (CHIP == P_V2)
		pAUD->AFE_POWER_CTL &= ~(PGA_DAC_POWER_EN);
        #endif
	    #if (CHIP == P_V2)
        pAUD->AFE_ANA_CTL &= ~(ANA_HP_REF_OP_EN);
        #endif
    }
   #endif    
   
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetVoiceOutPath);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetVoiceInPath
//  Parameter   : None
//  Return Value : None
//  Description : Set Voice In Path
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetVoiceInPath(MMP_UBYTE path)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;

	gbUseAitADC = path;
    
    pAUD->AFE_ADC_ANA_LPGA_CTL &= ~(LPGA_SRC_IN_MASK);
    pAUD->AFE_ADC_ANA_RPGA_CTL &= ~(RPGA_SRC_IN_MASK);

    if (gbUseAitADC == AUDIO_IN_AFE_SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL |= LPGA_SRC_IN_AUXL;
        pAUD->AFE_ADC_ANA_RPGA_CTL |= RPGA_SRC_IN_AUXR;
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF){
        pAUD->AFE_ADC_ANA_LPGA_CTL |= LPGA_SRC_IN_MICLIP_LIN;
        pAUD->AFE_ADC_ANA_RPGA_CTL |= RPGA_SRC_IN_MICRIP_RIN;            
    }
    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING){
        pAUD->AFE_ADC_ANA_LPGA_CTL |= LPGA_SRC_IN_MICLIP;
        pAUD->AFE_ADC_ANA_RPGA_CTL |= RPGA_SRC_IN_MICRIP;       
    }

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetVoiceInPath);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetADCDigitalGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetADCDigitalGain(MMP_UBYTE gain)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;
	
	//if (bRecordSetting) {
    	gbADCDigitalGain = gain;
    	//}
    
    pAUD->AFE_ADC_DIG_GAIN = (gain << 8)|gain;
    
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetADCDigitalGain);
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACDigitalGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDACDigitalGain(MMP_UBYTE gain)
{
    AITPS_AUD   pAUD = AITC_BASE_AUD;
    MMP_USHORT  usGain = gain;

    gbDACDigitalGain = usGain & DAC_DIG_GAIN_MASK;
    pAUD->AFE_DAC_DIG_GAIN = (gbDACDigitalGain << 8) | gbDACDigitalGain;

    //dbg_printf(0,"[sean] : DAG digital gain : 0x%0x\r\n",pAUD->AFE_DAC_DIG_GAIN );
    
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetDACDigitalGain);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetDACAnalogGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetDACAnalogGain(MMP_UBYTE gain)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    //sean@2015_02_17, alyways set for test
    gbDACLineOutGain = gain ;
    if (1/*GetDAC_Usage() & AUDIO_OUT_AFE_LINE*/) {	   
       pAUD->AFE_DAC_LOUT_VOL = (pAUD->AFE_DAC_LOUT_VOL & LOUT_ZC_GAIN_CTL_ACT ) | gbDACLineOutGain<<1;  
    }
    //dbg_printf(0,"[sean] : DAG analog gain : 0x%0x\r\n",pAUD->AFE_DAC_LOUT_VOL);
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetDACAnalogGain);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetADCAnalogGain
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetADCAnalogGain(MMP_UBYTE gain, MMP_BOOL boostup)
{
	AITPS_AUD pAUD = AITC_BASE_AUD;

    //if (bRecordSetting) {
  gbADCAnalogGain = gain;
    //	gbADCBoostp = boostup;
    //}
  pAUD->AFE_ADC_ANA_LPGA_GAIN = gbADCAnalogGain;
  pAUD->AFE_ADC_ANA_RPGA_GAIN = gbADCAnalogGain;
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetADCAnalogGain);
//------------------------------------------------------------------------------
//  Function    : MMPF_TriggerDAC
//  Parameter   : None
//  Return Value : None
//  Description : Trigger DAC
//------------------------------------------------------------------------------
void MMPF_TriggerDAC(void)
{   	
    AITPS_AUD pAUD = AITC_BASE_AUD;
    
    glTrigCnt = 0;
    gbInitTriggerDAC = MMP_TRUE;

    MMPF_Audio_InitializePlayFIFO(AFE_FIFO_TO_DAC, MP3_I2S_FIFO_WRITE_THRESHOLD);

	if (!m_bADCPowerOnState)
        pAUD->AFE_MUX_MODE_CTL &= ~(AUD_MUX_AUTO);

    pAUD->AFE_FIFO.DAC_MP.CPU_INT_EN &= ~(AUD_INT_FIFO_REACH_UNWR_TH);
    pAUD->AFE_CPU_INT_EN &= ~(AUD_DAC_INT_EN);

    gbInitTriggerDAC = MMP_FALSE;
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
    AIT_REG_D  *AFE_FIFO_DATA = (AIT_REG_D *)&pAUD->AFE_FIFO.DAC_MP.DATA;

	if(pAUD->AFE_FIFO.DAC_MP.WR_TH < (TWaveTotalLen-glTrigCnt))	
		WriteCnt = pAUD->AFE_FIFO.DAC_MP.WR_TH;
	else
		WriteCnt = TWaveTotalLen-glTrigCnt;

    if(glTrigCnt < TWaveTotalLen) {
		for(i = 0; i < WriteCnt; i+=2) {
			tmp = MMPF_TWave(glTrigCnt++);		
			*AFE_FIFO_DATA = tmp;
		    *AFE_FIFO_DATA = tmp;
		}	
    }
	else {
		for(i = 0; i < pAUD->AFE_FIFO.DAC_MP.WR_TH; i+=2) {
			*AFE_FIFO_DATA = 0x00;
		    *AFE_FIFO_DATA = 0x00;
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

    pAUD->AFE_FIFO.DAC_MP.RST = AUD_FIFO_RST_EN;
    while(i--); // wait for one cycle of audio moudle clock
    pAUD->AFE_FIFO.DAC_MP.RST = 0;

    return MMP_ERR_NONE;
}


static int dump_audio_reg(volatile u8* buffer,int size)
{
	volatile u8* ptr = (volatile u8*)buffer;
	int i = 0;

	dbg_printf(0,"DAC Contents (hex):\n");

  for(i=0;i<size;i++) {
   dbg_printf(0,"sregb 0x%04x 0x%02x ;\n",(u32)ptr+i,ptr[i]);  
  }  
	return 0;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerOnDAC
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerOnDAC(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    volatile u8 *reg_aud = (volatile u8 *)pAUD ;
    
    glAudPlaySampleRate = samplerate;
    MMPF_Audio_SetDACFreq(glAudPlaySampleRate);

    if (m_bDACPowerOnState) {
		    return MMP_ERR_NONE;
    }
    m_bDACPowerOnState = MMP_TRUE;
    #if (CHIP == MCR_V2)
    pAUD->AFE_OVF_BUGFIX |= DAC_DIG_VOL_OVF_FIX | DAC_SDM_OVF_FIX;
    #endif

    #if (CHIP == MCR_V2)   
    pAUD->AFE_CLK_CTL |= (DAC_CLK_INV_EN);
    pAUD->AFE_CLK_CTL |= DAC_CLK_256FS_MODE;
    #endif

	if (GetDAC_Usage() & AUDIO_OUT_AFE_HP) {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS;
	}
	else {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS | DAC_DITHER_EN | DAC_DITHER_SCALE_2_1;
	}

    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN_CTL = DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE;
    #endif
    
    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_LOUT_VOL &=   ~LOUT_ANA_GAIN_MASK;
    pAUD->AFE_DAC_LOUT_VOL |=   LOUT_ZC_GAIN_CTL_ACT ; //enable zero crossing.
    #endif
     pAUD->AFE_POWER_CTL |= VREF_POWER_EN;
    
    if(gbAudioDACFastCharge)
        MMPF_OS_Sleep(150); // origianl 100
    else
        MMPF_OS_Sleep(2000); // origianl 100


    pAUD->AFE_POWER_CTL |= ANALOG_POWER_EN;
	
    MMPF_OS_Sleep(50); // origianl 300
    //MMPF_OS_Sleep(300);
    
    //MMPF_OS_Sleep(1); // not be checked    
    
    pAUD->AFE_POWER_CTL |= DAC_DF_POWER_EN;
    
    MMPF_OS_Sleep(1); // original 10
      
    #if (CHIP == MCR_V2)
    pAUD->AFE_ANA_DAC_POWER_CTL |=DAC_POWER_UP;
    #endif
     MMPF_OS_Sleep(1); // original 10
    //MMPF_OS_Sleep(100);
    
    //MMPF_TriggerDAC();
     
    MMPF_OS_Sleep(10);	 // origianl 100						
    //MMPF_OS_Sleep(100);

    if (GetDAC_Usage() & AUDIO_OUT_AFE_LINE) {
        pAUD->AFE_DAC_LOUT_VOL |= (gbDACLineOutGain << 1);
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL |= LINEOUT_POWER_UP;
        #endif
    }
    else {
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL &= ~ LINEOUT_POWER_UP;
        #endif
    }
   
    MMPF_OS_Sleep(100);

    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN = (gbDACDigitalGain << 8) | gbDACDigitalGain;
    pAUD->AFE_DAC_DIG_GAIN_CTL &= ~(DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE);
    #endif
    dbg_printf(0,"[AIT]:PwrOnDAC:0x%0x,0x%0x\r\n",pAUD->AFE_DAC_LOUT_VOL,pAUD->AFE_DAC_DIG_GAIN );
    #if 0 // SEANTEST
    dump_audio_reg(reg_aud,256);
    #endif
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerOnDAC);

MMP_ERR MMPF_Audio_PowerOnDAC_Fast(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    volatile u8 *reg_aud = (volatile u8 *)pAUD ;
    
    glAudPlaySampleRate = samplerate;
    MMPF_Audio_SetDACFreq(glAudPlaySampleRate);

    if (m_bDACPowerOnState) {
		    return MMP_ERR_NONE;
    }
    m_bDACPowerOnState = MMP_TRUE;
    #if (CHIP == MCR_V2)
    pAUD->AFE_OVF_BUGFIX |= DAC_DIG_VOL_OVF_FIX | DAC_SDM_OVF_FIX;
    #endif

    #if (CHIP == MCR_V2)   
    pAUD->AFE_CLK_CTL |= (DAC_CLK_INV_EN);
    pAUD->AFE_CLK_CTL |= DAC_CLK_256FS_MODE;
    #endif

	if (GetDAC_Usage() & AUDIO_OUT_AFE_HP) {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS;
	}
	else {
	    pAUD->AFE_DAC_FILTER_CTL = DAC_128_FS | DAC_DITHER_EN | DAC_DITHER_SCALE_2_1;
	}

    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN_CTL = DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE;
    #endif
    
    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_LOUT_VOL &=   ~LOUT_ANA_GAIN_MASK;
    pAUD->AFE_DAC_LOUT_VOL |=   LOUT_ZC_GAIN_CTL_ACT ; //enable zero crossing.
    #endif
     pAUD->AFE_POWER_CTL |= VREF_POWER_EN;
    
   // if(gbAudioDACFastCharge)
   //     MMPF_OS_Sleep(150); // origianl 100
   // else
   //     MMPF_OS_Sleep(2000); // origianl 100


    pAUD->AFE_POWER_CTL |= ANALOG_POWER_EN;
	
    //MMPF_OS_Sleep(50); // origianl 300
    
    pAUD->AFE_POWER_CTL |= DAC_DF_POWER_EN;
    
    //MMPF_OS_Sleep(1); // original 10
      
    #if (CHIP == MCR_V2)
    pAUD->AFE_ANA_DAC_POWER_CTL |=DAC_POWER_UP;
    #endif

    //MMPF_OS_Sleep(10);	 // origianl 100						

    if (GetDAC_Usage() & AUDIO_OUT_AFE_LINE) {
        pAUD->AFE_DAC_LOUT_VOL |= (gbDACLineOutGain << 1);
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL |= LINEOUT_POWER_UP;
        #endif
    }
    else {
        #if (CHIP == MCR_V2)
        pAUD->AFE_ANA_DAC_POWER_CTL &= ~ LINEOUT_POWER_UP;
        #endif
    }
   
    //MMPF_OS_Sleep(100);

    #if (CHIP == MCR_V2)
    pAUD->AFE_DAC_DIG_GAIN = (gbDACDigitalGain << 8) | gbDACDigitalGain;
    pAUD->AFE_DAC_DIG_GAIN_CTL &= ~(DAC_R_SOFT_MUTE | DAC_L_SOFT_MUTE);
    #endif
    dbg_printf(0,"[AIT]:PwrOnDAC:0x%0x,0x%0x\r\n",pAUD->AFE_DAC_LOUT_VOL,pAUD->AFE_DAC_DIG_GAIN );
    #if 0 // SEANTEST
    dump_audio_reg(reg_aud,256);
    #endif
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerOnDAC_Fast);


MMP_ERR MMPF_Audio_SetADCMute(MMP_BOOL enable)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    if(enable) {
        pAUD->AFE_ADC_ANA_LPGA_CTL |=LPGA_MUTE_CTL;
        pAUD->AFE_ADC_ANA_RPGA_CTL |=RPGA_MUTE_CTL;    
        pAUD->AFE_DIG_GAIN_SETTING_CTL |= AFE_LR_MUTE_ENA;
    }
    else { 
        pAUD->AFE_ADC_ANA_LPGA_CTL &= ~LPGA_MUTE_CTL;
        pAUD->AFE_ADC_ANA_RPGA_CTL &= ~RPGA_MUTE_CTL;    
        pAUD->AFE_DIG_GAIN_SETTING_CTL &= ~AFE_LR_MUTE_ENA;    
    }
    return MMP_ERR_NONE;
}  
EXPORT_SYMBOL(MMPF_Audio_SetADCMute);

MMP_ERR MMPF_Audio_SetDACMute(void)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    gbDACLineOutGain = LOUT_ANA_GAIN_MUTE >> 1 ;
    gbDACDigitalGain = DAC_L_DIG_MUTE ;
    pAUD->AFE_DAC_LOUT_VOL |= LOUT_ANA_GAIN_MUTE;
    //pAUD->AFE_DAC_DIG_GAIN  = DAC_L_DIG_MUTE;
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_SetDACMute);

void MMPF_Audio_SoftMuteStart(void)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;

    #if (CHIP == MCR_V2)

    pAUD->AFE_CPU_INT_EN                    =   AUD_L_TARGET_GAIN_REACH_INT_EN |AUD_R_TARGET_GAIN_REACH_INT_EN ;
    
    pAUD->AFE_ADC_DIG_GAIN                  = (gbADCDigitalGain << 8) | gbADCDigitalGain;
    
    pAUD->AFE_LADC_DIG_STEP_TIME[0]         =   ((256*32*DIG_STEP_TIME) & 0xff);
    pAUD->AFE_LADC_DIG_STEP_TIME[1]         =   ((256*32*DIG_STEP_TIME)>>8 & 0xff);
    pAUD->AFE_LADC_DIG_STEP_TIME[2]         =   ((256*32*DIG_STEP_TIME)>>16 & 0xff);
    
    pAUD->AFE_RADC_DIG_STEP_TIME[0]         =   ((256*32*DIG_STEP_TIME) & 0xff);
    pAUD->AFE_RADC_DIG_STEP_TIME[1]         =   ((256*32*DIG_STEP_TIME)>>8 & 0xff);
    pAUD->AFE_RADC_DIG_STEP_TIME[2]         =   ((256*32*DIG_STEP_TIME)>>16 & 0xff);
        
    pAUD->AFE_ANA_L_STEP_TIME[0]            =   ((128*32*MACRO_STEP_TIME) & 0xff);
    pAUD->AFE_ANA_L_STEP_TIME[1]            =   ((128*32*MACRO_STEP_TIME)>>8 & 0xff);
    pAUD->AFE_ANA_L_STEP_TIME[2]            =   ((128*32*MACRO_STEP_TIME)>>16 & 0xff);
    
    pAUD->AFE_ANA_R_STEP_TIME[0]            =   ((128*32*MACRO_STEP_TIME) & 0xff);
    pAUD->AFE_ANA_R_STEP_TIME[1]            =   ((128*32*MACRO_STEP_TIME)>>8 & 0xff);
    pAUD->AFE_ANA_R_STEP_TIME[2]            =   ((128*32*MACRO_STEP_TIME)>>16 & 0xff);
    
    
    pAUD->AFE_ANA_L_TIME_OUT[0]            =   ((128*32*MACRO_TIMEOUT) & 0xff);
    pAUD->AFE_ANA_L_TIME_OUT[1]            =   ((128*32*MACRO_TIMEOUT)>>8 & 0xff);
    pAUD->AFE_ANA_L_TIME_OUT[2]            =   ((128*32*MACRO_TIMEOUT)>>16 & 0xff);
    
    pAUD->AFE_ANA_R_TIME_OUT[0]            =   ((128*32*MACRO_TIMEOUT) & 0xff);
    pAUD->AFE_ANA_R_TIME_OUT[1]            =   ((128*32*MACRO_TIMEOUT)>>8 & 0xff);
    pAUD->AFE_ANA_R_TIME_OUT[2]            =   ((128*32*MACRO_TIMEOUT)>>16 & 0xff);
    /* wait for digital gain ramp up */
    pAUD->AFE_DIG_SOFT_MUTE_SR      = pAUD->AFE_DIG_SOFT_MUTE_SR; //clear status
    pAUD->AFE_DIG_GAIN_SETTING_CTL &= ~(AFE_LR_MUTE_ENA);
    while(!(pAUD->AFE_DIG_SOFT_MUTE_SR & 0x01) || !(pAUD->AFE_DIG_SOFT_MUTE_SR & 0x02));
    pAUD->AFE_DIG_SOFT_MUTE_SR              =   pAUD->AFE_DIG_SOFT_MUTE_SR; //write 1 clear    

    /* start analog gain steps smoothly  */
    pAUD->AFE_DIG_GAIN_MUTE_STEP            =   ADC_PGA_SMOOTH_METHOD;
    pAUD->AFE_ADC_ANA_LPGA_GAIN             =   gbADCAnalogGain;
    pAUD->AFE_ADC_ANA_RPGA_GAIN             =   gbADCAnalogGain;
    
    #endif
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerOnADC
//  Parameter   : None
//  Return Value : None
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerOnADC(MMP_ULONG samplerate)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    AITPS_DADC_EXT  pDADC_EXT =AITC_BASE_DADC_EXT;
    
    #if (AUDIO_CODEC_DUPLEX_EN == 1)
    //if (gbSystemCoreID == CHIP_CORE_ID_MCR_V2_MP) {
   	//    if (!m_bDACPowerOnState) {
  	//        MMPF_Audio_PowerOnDAC(MMP_TRUE);
	//    }
    //}
    #else
    // if DAC is still power-on, power off it first
    if (m_bDACPowerOnState) {
        MMPF_Audio_PowerDownDAC(MMP_TRUE);
    }
    #endif
    if(!m_bADCPowerOnState){
	    m_bADCPowerOnState = MMP_TRUE;

	    pAUD->AFE_ADC_LOOP_CTL = 0;
	    pAUD->AFE_DIG_GAIN_SETTING_CTL          =  AFE_DIG_GAIN_SMOOTH_METHOD | AFE_LR_MUTE_ENA;
	    pAUD->AFE_ANA_GAIN_SETTING_CTL          =  AFE_ANA_GAIN_SMOOTH_METHOD | TIME_OUT_PULSE_ENA;

	    pDADC_EXT->DADC_HPF_MODE_SEL            =   ADC_HPF_VOC;
	    pDADC_EXT->DADC_HPF_VOC_MODE_COEF       =   HPF_VOC_FC_300HZ;
	    
	    if ((pAUD->AFE_POWER_CTL & VREF_POWER_EN) == 0) {
	        pAUD->AFE_POWER_CTL |= VREF_POWER_EN;
	        MMPF_OS_Sleep(20);
	    }
	    if ((pAUD->AFE_POWER_CTL & ANALOG_POWER_EN) == 0) {
	        pAUD->AFE_POWER_CTL |= ANALOG_POWER_EN;
	        MMPF_OS_Sleep(10);
	    }
	    
	    if ((pAUD->AFE_ANA_ADC_POWER_CTL & (ADC_PGA_L_POWER_EN|ADC_PGA_R_POWER_EN)) == 0) {
	        pAUD->AFE_ANA_ADC_POWER_CTL |= (ADC_PGA_L_POWER_EN|ADC_PGA_R_POWER_EN);
	        MMPF_OS_Sleep(20);
	        pAUD->AFE_ANA_ADC_POWER_CTL |= (ADC_R_POWER_UP|ADC_L_POWER_UP);
	    }
	    
	    if ((pAUD->AFE_POWER_CTL & ADC_DF_POWER_EN) == 0) {
	        pAUD->AFE_POWER_CTL |= ADC_DF_POWER_EN;
	        MMPF_OS_Sleep(10);
	    }

	    //Control ADC mic L/R boost up, mic bias
	    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING)) {
			#if (DEFAULT_ADC_MIC_BIAS_MP == ADC_MIC_BIAS_NONE)
		    pAUD->AFE_MICBIAS_DUAL_CHAN = 0; // power down mic bias
			#elif (DEFAULT_ADC_MIC_BIAS_MP == ADC_MIC_BIAS_0d65AVDD_MP)
		    pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
		    		                        ADC_MIC_BIAS_R_VOLT_0D65_MP| ADC_MIC_BIAS_L_VOLT_0D65_MP ;
			#elif (DEFAULT_ADC_MIC_BIAS_MP == ADC_MIC_BIAS_0d75AVDD_MP)
			pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
		    		                        ADC_MIC_BIAS_R_VOLT_0D75_MP| ADC_MIC_BIAS_L_VOLT_0D75_MP ;
			#elif (DEFAULT_ADC_MIC_BIAS_MP == ADC_MIC_BIAS_0d85AVDD_MP)
		    pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
		    		                        ADC_MIC_BIAS_R_VOLT_0D85_MP| ADC_MIC_BIAS_L_VOLT_0D85_MP ;
			#elif (DEFAULT_ADC_MIC_BIAS_MP == ADC_MIC_BIAS_0d95AVDD_MP)
		    pAUD->AFE_MICBIAS_DUAL_CHAN = (pAUD->AFE_MICBIAS_DUAL_CHAN & ADC_MIC_BIAS_VOLT_MASK) |
		                             		ADC_MIC_BIAS_R_VOLT_0D95_MP| ADC_MIC_BIAS_L_VOLT_0D85_MP ;
			#endif
	        pAUD->AFE_MICBIAS_DUAL_CHAN |=  ADC_MIC_BIAS_R_POWER_UP | ADC_MIC_BIAS_L_POWER_UP;
	    }

	    //Control ADC in path
	    if (gbUseAitADC == AUDIO_IN_AFE_SING){
	        pAUD->AFE_ADC_ANA_LPGA_CTL  =   LPGA_SRC_IN_AUXL | IN_LPGA_ZC_EN;
	        pAUD->AFE_ADC_ANA_RPGA_CTL  =   RPGA_SRC_IN_AUXR | IN_RPGA_ZC_EN;
	    }
	    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF){
	        pAUD->AFE_ADC_ANA_LPGA_CTL  =   LPGA_SRC_IN_MICLIP_LIN | IN_LPGA_ZC_EN;
	        pAUD->AFE_ADC_ANA_RPGA_CTL  =   RPGA_SRC_IN_MICRIP_RIN | IN_RPGA_ZC_EN;
	    }
	    else if (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING){
	        pAUD->AFE_ADC_ANA_LPGA_CTL  =   LPGA_SRC_IN_MICLIP | IN_LPGA_ZC_EN;
	        pAUD->AFE_ADC_ANA_RPGA_CTL  =   RPGA_SRC_IN_MICRIP | IN_RPGA_ZC_EN;
	    }
	    
	    MMPF_Audio_SoftMuteStart();
	    pAUD->AFE_DIG_GAIN_SETTING_CTL &=  ~AFE_LR_MUTE_ENA;
	    pAUD->AFE_DIG_GAIN_MUTE_STEP            =   ADC_PGA_SMOOTH_METHOD;
	     
	    //MMPF_Audio_SetADCFreq(samplerate);
    }
    MMPF_Audio_ConfigHWEQGain(EQ_HSF, DEFAULT_EQ_HSF_GAIN);
    MMPF_Audio_ConfigHWEQGain(EQ_PK3, DEFAULT_EQ_PK3_GAIN);
    MMPF_Audio_ConfigHWEQGain(EQ_PK2, DEFAULT_EQ_PK2_GAIN);
    MMPF_Audio_ConfigHWEQGain(EQ_PK1, DEFAULT_EQ_PK1_GAIN);
    MMPF_Audio_ConfigHWEQGain(EQ_LSF, DEFAULT_EQ_LSF_GAIN);
    
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerOnADC);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_SetPLL
//  Parameter   : 
//      ulSamplerate -- sampling rate
//  Return Value : None
//  Description : Dynamic change PLL for audio DAC.
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_SetPLL(LINUX_SOC_PATH path, MMP_ULONG ulSamplerate,uint8_t reset)
{
    MMP_ERR         err;
    MMPF_PLL_MODE   pll_mode;
    //MMPF_GROUP_SRC  pll_src;
    
    //#if (HDMI_OUTPUT_EN == 0)
    //if (glAudioSamplerate != ulSamplerate) {
    //#endif
    //if(glAudPllRefCount==0)
    //{
        switch(ulSamplerate) {
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
            //RTNA_DBG_Str0("Unsupported audio sample rate %dHz \r\n");
            pr_info("Unsupported audio sample rate %d Hz \r\n",ulSamplerate);
            return MMP_AUDIO_ERR_PARAMETER;
            break;
        }

        //err = MMPF_PLL_GetGroupSrc(MMPF_CLK_GRP_AUDIO, &pll_src);
        //if (err != MMP_ERR_NONE) {
        //    RTNA_DBG_Str0("Get Audio group source failed!\r\n");
        //    return err;
        //}
         #if (CHIP == MCR_V2)
        //err = MMPF_PLL_SetAudioPLL(MMPF_AUDSRC_MCLK, pll_mode);
        err = MMPF_PLL_SetAudioPLL(pll_mode,path,reset);
        #endif

        if (err != MMP_ERR_NONE) {
            RTNA_DBG_Str0("Set Audio PLL frequency failed!\r\n");
            return err;
        }
	
    //glAudPllRefCount++;
    #if 0 //SEANTEST
    pr_info("Audio PLL ref count = %d\r\n",glAudPllRefCount);
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
    AITPS_AUD pAUD = AITC_BASE_AUD;
    AITPS_I2S pI2S = AITC_BASE_I2S0;

    if (samplerate <= 48000) {
        #if (HIGH_SRATE_MODE == DOWN_SAMPLE_TIMES)
        pI2S->AFE_DOWN_SAMPLE_SEL = DOWN_SAMPLE_OFF;
        #elif (HIGH_SRATE_MODE == BYPASS_FILTER_STAGE)
        pAUD->AFE_SAMPLE_RATE_SEL = SRATE_48000Hz_UNDER;
        #endif
    }

    switch(samplerate) {
	case 48000:
        pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_48000HZ);
		break;	
	case 44100:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_44100HZ);
		break;	
	case 32000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_32000HZ);
		break;	
	case 24000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_24000HZ);
		break;	
	case 22050:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_22050HZ);
		break;	
	case 16000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_16000HZ);
		break;	
	case 12000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_12000HZ);
		break;	
	case 11025:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_11025HZ);
		break;
	case 8000:
		pAUD->AFE_SAMPLE_RATE_CTL = DAC_SRATE(SRATE_8000HZ);
		break;	
}

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
    #if (AUDIO_CODEC_DUPLEX_EN == 1)
    //glAudRecSampleRate = samplerate;
    #endif

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
    AITPS_AUD pAUD = AITC_BASE_AUD;

    if (!bPowerDownNow) {
        // not really power down DAC, 
        // keep DAC in power on state to avoid pop-noise again
        return MMP_ERR_NONE;
    }

    pAUD->AFE_POWER_CTL &= ~(DAC_DF_POWER_EN);
    pAUD->AFE_ANA_DAC_POWER_CTL &= ~(DAC_POWER_UP | LINEOUT_POWER_UP);
    
    m_bDACPowerOnState = MMP_FALSE;
    SetDAC_Usage(0);
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerDownDAC);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerDownADC
//  Parameter   : None
//  Return Value : None
//  Description : Power down ADC
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerDownADC(void)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;
    
    return  MMP_ERR_NONE;
    
    #if (CHIP == P_V2)
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING))
        pAUD->AFE_ANA_CTL_4 &= ~(ADC_MIC_BIAS_EN);
    pAUD->AFE_POWER_CTL &= ~(ADC_PGA_POWER_EN | ANALOG_ADC_POWER_EN | ADC_DF_POWER_EN);
    #endif
    
    #if (CHIP == MCR_V2)
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING))
        pAUD->AFE_MICBIAS_DUAL_CHAN = 0;
    pAUD->AFE_POWER_CTL &= ~(ADC_DF_POWER_EN);
    pAUD->AFE_ANA_ADC_POWER_CTL &= ~(ADC_R_POWER_UP | ADC_L_POWER_UP |
                                     ADC_PGA_L_POWER_EN | ADC_PGA_R_POWER_EN);
    #endif

    m_bADCPowerOnState = MMP_FALSE;

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerDownADC);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_PowerDownCodec
//  Parameter   : None
//  Return Value : None
//  Description : Power down ADC & DAC
//------------------------------------------------------------------------------
MMP_ERR MMPF_Audio_PowerDownCodec(void)
{
    AITPS_AUD pAUD = AITC_BASE_AUD;

    if (!m_bDACPowerOnState && !m_bADCPowerOnState)
        return MMP_ERR_NONE;

    #if (CHIP == P_V2)
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING))
        pAUD->AFE_ANA_CTL_4 &= ~(ADC_MIC_BIAS_EN);
    #endif
    #if (CHIP == MCR_V2)
    // MIC bias power down
    if ((gbUseAitADC == AUDIO_IN_AFE_DIFF) || (gbUseAitADC == AUDIO_IN_AFE_DIFF2SING))
        pAUD->AFE_MICBIAS_DUAL_CHAN = 0;
    // PGA, ADC power down
    pAUD->AFE_ANA_ADC_POWER_CTL &= ~(ADC_R_POWER_UP | ADC_L_POWER_UP |
                                     ADC_PGA_L_POWER_EN | ADC_PGA_R_POWER_EN);
    // DAC, LineOut power down
    pAUD->AFE_ANA_DAC_POWER_CTL &= ~(DAC_POWER_UP | LINEOUT_POWER_UP);
    #endif

    pAUD->AFE_POWER_CTL = AFE_POWER_OFF;

    m_bDACPowerOnState = MMP_FALSE;
    m_bADCPowerOnState = MMP_FALSE;

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_PowerDownCodec);

MMP_ERR MMPF_Audio_EnableAFEClock(MMP_BOOL bEnableClk, MMP_ULONG SampleRate,LINUX_SOC_PATH Path)
{
    if(bEnableClk)
    {
    
        if(glAudPllRefCount == 0)
            MMPF_Audio_SetPLL(Path,SampleRate,1);
        else
            MMPF_Audio_SetPLL(Path,SampleRate,0);	
    
        glAudPllRefCount++;
    }
    else
    {
    //if(glAudPllRefCount==0)
    //	MMPF_Audio_SetPLL(Path,SampleRate);
        glAudPllRefCount--;
    } 
    #if 0 //SEANTEST
    pr_info("Audio PLL ref count = %d\r\n",glAudPllRefCount);
    #endif
    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_Audio_EnableAFEClock);

/**------------------------------------------------------------------------------
  Function      : MMPF_Audio_ConfigHWEQGain
  @param        :    None
  Return Value  : None
  Description   : 
------------------------------------------------------------------------------**/
MMP_ERR MMPF_Audio_ConfigHWEQGain(MMP_EQ_BAND band, MMP_EQ_GAIN gainstep)
{
    AITPS_DADC_EXT pDADC_EXT = AITC_BASE_DADC_EXT;
    switch(band) {
    case EQ_LSF:
        pDADC_EXT->DADC_L_EQ_LSF_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_R_EQ_LSF_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_L_EQ_LSF_COEF = gainstep;
        pDADC_EXT->DADC_R_EQ_LSF_COEF = gainstep;
        break;
    case EQ_PK1:
        pDADC_EXT->DADC_L_EQ_PK1_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_R_EQ_PK1_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_L_EQ_PK1_COEF = gainstep;
        pDADC_EXT->DADC_R_EQ_PK1_COEF = gainstep;
        break;
    case EQ_PK2:
        pDADC_EXT->DADC_L_EQ_PK2_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_R_EQ_PK2_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_L_EQ_PK2_COEF = gainstep;
        pDADC_EXT->DADC_R_EQ_PK2_COEF = gainstep;
        break;
    case EQ_PK3:
        pDADC_EXT->DADC_L_EQ_PK3_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_R_EQ_PK3_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_L_EQ_PK3_COEF = gainstep;
        pDADC_EXT->DADC_R_EQ_PK3_COEF = gainstep;
        break;
    case EQ_HSF:
        pDADC_EXT->DADC_L_EQ_HSF_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_R_EQ_HSF_COEF = EQ_BYPASS;
        pDADC_EXT->DADC_L_EQ_HSF_COEF = gainstep;
        pDADC_EXT->DADC_R_EQ_HSF_COEF = gainstep;
        break;                                
    }   
    return MMP_ERR_NONE;     
}

MODULE_LICENSE("GPL");
/** @} */ // end of MMPF_AUDIO
