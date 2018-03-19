//==============================================================================
//
//  File        : mmp_register_audio.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Rogers Chen
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_AUDIO_H_
#define _MMP_REG_AUDIO_H_


#include "hardware.h"
#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

#if (CHIP==VSN_V3)

// ********************************
//   I2S structure (0x8000 7800)
// ********************************
typedef struct _AITS_I2S {
    AIT_REG_B   I2S_FIFO_CPU_INT_EN;                                    // 0x00
    AIT_REG_B   I2S_FIFO_HOST_INT_EN;                                   // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_INT_FIFO_EMPTY          0x0001
        #define AUD_INT_FIFO_FULL           0x0002
        #define AUD_INT_FIFO_REACH_UNRD_TH  0x0004
        #define AUD_INT_FIFO_REACH_UNWR_TH  0x0008
		/*------------------------------------------------------------*/    
    #if (CHIP == VSN_V3)
	AIT_REG_W   						_x02;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
	AIT_REG_B   I2S_FIFO_INT_SR;                                        // 0x02
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_FIFO_SRC_CH0            0x0001
        #define I2S_FIFO_SRC_CH1            0x0002
        #if (CHIP == MERCURY)
        #define I2S_FIFO_SRC_CH2            0x0004
        #endif
        /*------------------------------------------------------------*/
    AIT_REG_B                          	_x03;
    #endif
    AIT_REG_B   I2S_FIFO_SR;                                            // 0x04
    AIT_REG_B   						_x05[3];
    AIT_REG_B   I2S_FIFO_RST;                                           // 0x08
        /*-DEFINE-----------------------------------------------------*/
		#define AUD_FIFO_RST_EN             0x0001
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x09[3];
    AIT_REG_D   I2S_FIFO_DATA;                                          // 0x0C
    AIT_REG_W   I2S_FIFO_RD_TH;                                         // 0x10
    AIT_REG_W   						_x12;
    AIT_REG_W   I2S_FIFO_WR_TH;                                         // 0x14
    AIT_REG_W   						_x16;
    AIT_REG_W   I2S_FIFO_UNRD_CNT;                                      // 0x18
    AIT_REG_W   						_x1A;
    AIT_REG_W   I2S_FIFO_UNWR_CNT;                                      // 0x1C
    AIT_REG_W   						_x1E[5];                         
    AIT_REG_B   I2S_CTL;                                                // 0x28
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_SDO_OUT_EN              0x08
		#define I2S_LRCK_OUT_EN             0x04
		#define I2S_BCK_OUT_EN              0x02
		#define I2S_HCK_CLK_EN              0x01
		#define I2S_ALL_DIS                 0x0
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x29[3];
    AIT_REG_B   I2S_CLK_DIV;                                            // 0x2C
    AIT_REG_B   I2S_CLK_CTL;                                            // 0x2D
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_MCLK_FIX                0x01
		/*------------------------------------------------------------*/
    AIT_REG_W   						_x2E;
    AIT_REG_W   I2S_RATIO_N_M;                                          // 0x30
    AIT_REG_W   						_x32;
    AIT_REG_B   I2S_BIT_CLT;                                            // 0x34
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_OUT_32BITS              0x04
		#define I2S_OUT_24BITS              0x02
		#define I2S_OUT_16BITS              0x01
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x35[3];
    AIT_REG_B   I2S_LRCK_POL;                                           // 0x38
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_LRCK_L_CH_HIGH          0x01
		#define I2S_LRCK_L_CH_LOW           0x00
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x39[3];
    AIT_REG_D   I2S_L_CHNL_DATA;                                        // 0x3C
    AIT_REG_D   I2S_R_CHNL_DATA;                                        // 0x40
    AIT_REG_B   I2S_BIT_ALIGN_OUT;                                      // 0x44
    AIT_REG_B   I2S_BIT_ALIGN_IN;                                       // 0x45
    AIT_REG_W   						_x46;
    AIT_REG_B   I2S_MODE_CTL;                                           // 0x48
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_MCLK_OUT_EN             4
		#define I2S_SLAVE                   0   
		#define I2S_MASTER                  1   
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x49[3];
    AIT_REG_B   I2S_MCLK_CTL;                                           // 0x4C
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_1536_FS                 0x80
        #define I2S_1024_FS                 0x40
        #define I2S_768_FS                  0x20
        #define I2S_512_FS                  0x10
    	#define I2S_384_FS				    0x08
    	#define I2S_256_FS				    0x04
    	#define I2S_192_FS				    0x02
    	#define I2S_128_FS				    0x01
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x4D[3];
    AIT_REG_B   I2S_MSPORT_CTL;                                         // 0x50
    AIT_REG_B   I2S_MSPORT_START_BIT;                                   // 0x51
    AIT_REG_W   						_x52;
    AIT_REG_B   I2S_DATA_OUT_EN;                                        // 0x54
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_SDI_OUT    			    0x02
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x55[3];
    AIT_REG_B   I2S_DATA_IN_SEL;                                        // 0x58
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_SDO_IN     			    0x01
        #define I2S_SDI_IN    			    0x00
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x59[3];
    AIT_REG_B   I2S_CPU_INT_EN;                                         // 0x5C
    AIT_REG_B   I2S_HOST_INT_EN;                                        // 0x5D
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_INT_EN     		        0x01
        #define AUD_INT_DIS    			    0x00
		/*------------------------------------------------------------*/
    AIT_REG_B   I2S_INT_SR;                                             // 0x5E
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_INT_SRC_CH0             0x0001
        #define I2S_INT_SRC_CH1             0x0002
        /*------------------------------------------------------------*/
    AIT_REG_B   						_x5F;
    AIT_REG_B   I2S_OUT_MODE_CTL;                                       // 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define	I2S_STD_MODE			    0x01
        #define	I2S_I2S_MODE			    0x00
        /*------------------------------------------------------------*/
    AIT_REG_B   						_x61[3];
    AIT_REG_W   I2S_MSPORT_FR_CYCLE_CNT;                                // 0x64                             
    AIT_REG_B   I2S_MSPORT_FR_SYNC_CNT;                                 // 0x66
    AIT_REG_B                           _x67;
    AIT_REG_B   I2S_SDO_SHIFT_RST;                                      // 0x68
    AIT_REG_B                           _x69[7];
    AIT_REG_B   I2S_MUX_MODE_CTL;                                       // 0x70
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_MUX_AUTO                0x01
        #define AUD_MUX_CPU                	0x00
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x71[3];
    AIT_REG_B   DUPLEX_PATH_SEL;                                        // 0x74
        /*-DEFINE-----------------------------------------------------*/
        #define AFE_FULL_DUPLEX_I2S1_EN     0x08 //DAC -> AFE FIFO, ADC -> I2S1 FIFO
        #define AFE_FULL_DUPLEX_I2S0_EN     0x04 //DAC -> AFE FIFO, ADC -> I2S0 FIFO
        #define I2S1_FULL_DUPLEX_EN         0x02 //SDO -> I2S1 FIFO, SDI -> AFE FIFO
		#define I2S0_FULL_DUPLEX_EN         0x01 //SDO -> I2S0 FIFO, SDI -> AFE FIFO
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x75[3];
    AIT_REG_B   I2S_PATH_CTL;                                           // 0x78
        /*-DEFINE-----------------------------------------------------*/
        #define ThirdType_SYNC_MODE_EN	    0x40  
        #define I2S1_SDI_LOOPBACK_DAC       0x20 //I2S1 SDI -> DAC without pass FIFO
        #define I2S0_SDI_LOOPBACK_DAC       0x10 //I2S0 SDI -> DAC without pass FIFO
        #define ADC_LOOPBACK_I2S1_SDO       0x02 //ADC -> I2S1 SDO without pass FIFO
        #define ADC_LOOPBACK_I2S0_SDO       0x01 //ADC -> I2S0 SDO without pass FIFO
        /*------------------------------------------------------------*/
    AIT_REG_B   I2S_MISC_CTL;                                           // 0x79
    AIT_REG_B                           _x7A[6];
    AIT_REG_B   I2S_TIMER_EN;                                           // 0x80
    AIT_REG_B   						_x81[3];
    AIT_REG_W   I2S_TIMER_CTL;                                          // 0x84
    AIT_REG_W   						_x86;
    AIT_REG_D   I2S_TIMER_TAGET_VALUE;                                  // 0x88
    AIT_REG_D   I2S_TIMER_CUR_VALUE;                                    // 0x8C
    AIT_REG_B   I2S_TIMER_CPU_INT_EN;                                   // 0x90
    AIT_REG_B   I2S_TIMER_HOST_INT_EN;                                  // 0x91	
    AIT_REG_B   						_x92[2];
    
    AIT_REG_B   I2S0_TIMER_INT_SR;	                                    // 0x94
    #if (CHIP == MCR_V2)
    AIT_REG_B   I2S1_TIMER_INT_SR;	                                    // 0x95
    
    AIT_REG_B   						_x96[2];
    #endif 
    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
	AIT_REG_B							_x95[3];
    #endif
    AIT_REG_B   I2S_TIMER_RST;                                          // 0x98
    
    #if (CHIP == MCR_V2)
    AIT_REG_B                           _x99[7];                        // 0x99~9F
    AIT_REG_D  I2S_TIMER_TARGET_CNT_VALUE_LOW;                          // 0xA0
    AIT_REG_D  I2S_TIMER_TARGET_CNT_VALUE_HIG;                          // 0xA4
    
    AIT_REG_D  I2S_TIMER_CUR_CNT_VALUE_LOW;                             // 0xA8
    AIT_REG_D  I2S_TIMER_CUR_CNT_VALUE_HIG;                             // 0xAC
    AIT_REG_B                           _xB0[16];                       // 0xB0~BF
    #endif
    #if (CHIP == MERCURY)
    AIT_REG_B                           _x99[27];                       // 0x99~BF
    #endif
   
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_D   SPDIF_CHANNEL_SR[6];                                    // 0xC0
    AIT_REG_W   SPDIF_CTL;                                              // 0xD8
        /*-DEFINE-----------------------------------------------------*/
        #define HDMI_TRANS_EN               0x0200
        #define SPDIF_TRANS_EN              0x0100
        /*------------------------------------------------------------*/
    AIT_REG_B   SPDIF_DATA_ALIGN_FMT;                                   // 0xDA
    AIT_REG_B                           _xDB;
    AIT_REG_B   SPDIF_RST;                                              // 0xDC
    AIT_REG_B                           _xDD[3];                        // 0xDD~DF
    AIT_REG_B   HDMI_ECC;                                               // 0xE0
    AIT_REG_B                           _xE1[3];                        // 0xE1~E3
    AIT_REG_B   AFE_SAMPLE_RATE_SEL;                                    // 0xE4
        /*-DEFINE-----------------------------------------------------*/
        #define SRATE_192000Hz              0x02
        #define SRATE_96000Hz               0x01
        #define SRATE_48000Hz_UNDER         0x00
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xE5[3];                        // 0xE5~E7
    AIT_REG_B   AFE_DOWN_SAMPLE_SEL;                                    // 0xE8
        /*-DEFINE-----------------------------------------------------*/
        #define DOWN_SAMPLE_4               0x02
        #define DOWN_SAMPLE_2               0x01
        #define DOWN_SAMPLE_OFF             0x00
        /*------------------------------------------------------------*/
    AIT_REG_B                             _xE9[7];                      // 0xE9~0xEF
    AIT_REG_B  SRAM_LIGHT_SLEEP_MOD_CTL;     	                        // 0xF0
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_LEFT_CHAN_LIGHT_MODE    0x20
        #define AFE_RCHAN_LIGHT_MODE        0x10
        #define AFE_LCHAN_LIGHT_MODE        0x08
        #define AFE_FIFO_LIGHT_MODE         0x04
        #define IS21_FIFO_LIGHT_MODE        0x02
        #define IS20_FIFO_LIGHT_MODE        0x01
        /*------------------------------------------------------------*/
    AIT_REG_B  SRAM_DEEP_SLEEP_MOD_CTL;     	                        // 0xF1
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_LEFT_CHAN_DEEP_MODE     0x20
        #define AFE_RCHAN_DEEP_MODE         0x10
        #define AFE_LCHAN_DEEP_MODE         0x08
        #define AFE_FIFO_DEEP_MODE          0x04
        #define IS21_FIFO_DEEP_MODE         0x02
        #define IS20_FIFO_DEEP_MODE         0x01
        /*------------------------------------------------------------*/
    AIT_REG_B  SRAM_SHUT_DOWN_MOD_CTL;     	                            // 0xF2
       /*-DEFINE-----------------------------------------------------*/
       #define DAC_LEFT_CHAN_STDOWN_MODE    0x20
       #define AFE_RCHAN_STDOWN_MODE        0x10
       #define AFE_LCHAN_STDOWN_MODE        0x08
       #define AFE_FIFO_STDOWN_MODE         0x04
       #define IS21_FIFO_STDOWN_MODE        0x02
       #define IS20_FIFO_STDOWN_MODE        0x01
       /*------------------------------------------------------------*/
    #endif
} AITS_I2S, *AITPS_I2S;
#endif // end of VSN_V3

// **********************************************************
//   Audio AFE structure (0x8000 7F00)
// **********************************************************
#if (CHIP == VSN_V3)
typedef struct _AITS_AUD {
	AIT_REG_B	AFE_FIFO_CPU_INT_EN;									// 0x00
	AIT_REG_B	AFE_FIFO_HOST_INT_EN;									// 0x01
		/*-DEFINE-----------------------------------------------------*/
		#define AFE_INT_FIFO_REACH_UNWR_TH			0x08
		#define AFE_INT_FIFO_REACH_UNRD_TH			0x04
		#define AFE_INT_FIFO_FULL					0x02
		#define AFE_INT_FIFO_EMPTY					0x01
		/*------------------------------------------------------------*/
	AIT_REG_W							_x02;
	AIT_REG_B	AFE_FIFO_CSR;											// 0x04
		/*-DEFINE-----------------------------------------------------*/
		#define FIFO_UNWR_TH						0x08
		#define FIFO_UNRD_TH						0x04
		#define FIFO_FULL							0x02
		#define FIFO_EMPTY							0x01
		/*------------------------------------------------------------*/
	AIT_REG_B							_x05[3];
	AIT_REG_B	AFE_FIFO_RST;											// 0x08
		/*-DEFINE-----------------------------------------------------*/
		#define REST_FIFO	1
		/*------------------------------------------------------------*/
	AIT_REG_B							_x09[3];
	AIT_REG_D	AFE_FIFO_DATA;											// 0x0C
	AIT_REG_W	AFE_FIFO_RD_TH;											// 0x10
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x12;
	AIT_REG_W	AFE_FIFO_WR_TH;											// 0x14
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x16;
	AIT_REG_W	AFE_FIFO_UNRD_CNT;										// 0x18
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x1A;
	AIT_REG_W	AFE_FIFO_UNWR_CNT;										// 0x1C
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x1E;
	AIT_REG_D	AFE_L_CHANNEL_DATA;										// 0x20
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_D	AFE_R_CHANNEL_DATA;										// 0x24
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_CPU_INT_EN;										    // 0x28
		/*-DEFINE-----------------------------------------------------*/
		#define AUD_ADC_INT_EN						0x01
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_HOST_INT_EN;										// 0x29
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_INT_CSR;											// 0x2A
		/*-DEFINE-----------------------------------------------------*/
		#define AFE_INT
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x2B;
	AIT_REG_B	AFE_FIFO_RM;											// 0x2C
		/*-DEFINE-----------------------------------------------------*/
		#define HOST_AFE_ADC_INT_EN					0x01
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_COEF_RM;										// 0x2D
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x2E[17];
	AIT_REG_B	AFE_GBL_PWR_CTL;										// 0x50
		/*-DEFINE-----------------------------------------------------*/
		#define BYPASS_OP							0x40
		#define PWR_UP_ANALOG						0x20
		#define PWR_UP_VREF							0x10
		#define PWR_UP_ADC_DIGITAL_FILTER			0x01
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_CLK_CTL;											// 0x51
		/*-DEFINE-----------------------------------------------------*/
		#define AUD_CODEC_NORMAL_MODE				0x80
		#define ADC_CLK_MODE_USB					0x02
		#define ADC_CLK_INVERT						0x01
		/*------------------------------------------------------------*/
	AIT_REG_W							_0x52;
	AIT_REG_B	AFE_MUX_MODE_CTL;										// 0x54
		/*-DEFINE-----------------------------------------------------*/
		#define AUD_DATA_BIT_20						0x02	//otherwise 16bits
		#define AFE_MUX_AUTO_MODE					0x01
		#define AFE_MUX_CPU_MODE					0x00
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x55;
	AIT_REG_B	SPECIAL_AUD_CODEC_PATH;									// 0x56
		/*-DEFINE-----------------------------------------------------*/
		#define EXT_AUD_CODEC_EN					0x01
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x57;
	AIT_REG_B	AFE_GBL_BIAS_ADJ;									// 0x58
		/*-DEFINE-----------------------------------------------------*/
		#define GBL_BIAS_50							0x00
		#define GBL_BIAS_62_5						0x01
		#define GBL_BIAS_75							0x02
		#define GBL_BIAS_87_5						0x03
		#define GBL_BIAS_100						0x04
		#define GBL_BIAS_112_5						0x05
		#define GBL_BIAS_125						0x06
		#define GBL_BIAS_137_5						0x07
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x59[7];
	AIT_REG_B	AFE_ADC_PWR_CTL;										// 0x60
		/*-DEFINE-----------------------------------------------------*/
		#define ADC_SDM_RCH_POWER_EN				0x08
		#define ADC_SDM_LCH_POWER_EN				0x04
		#define ADC_PGA_RCH_POWER_EN				0x02
		#define ADC_PGA_LCH_POWER_EN				0x01
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_SAMPLE_RATE_CTL;											// 0x61
		/*-DEFINE-----------------------------------------------------*/
		#define ADC_SRATE_MASK                  0x0F
        #define ADC_SRATE(_a)                   (_a)
		#define SRATE_48000HZ                   0x0A
        #define SRATE_44100HZ                   0x09
        #define SRATE_32000HZ                   0x08
        #define SRATE_24000HZ                   0x06
        #define SRATE_22050HZ                   0x05
        #define SRATE_16000HZ                   0x04
        #define SRATE_12000HZ                   0x02
        #define SRATE_11025HZ                   0x01
        #define SRATE_8000HZ                    0x00
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_INPUT_SEL;										// 0x62
		/*-DEFINE-----------------------------------------------------*/
        #define ADC_AUX_IN	    	               0x10
        #define ADC_MIC_IN		                   0x04
        #define ADC_MIC_DIFF2SINGLE				   0x02
        #define ADC_MIC_DIFF					   0x00
        #define ADC_CTL_MASK					   ~(ADC_AUX_IN|ADC_MIC_IN)
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_BOOST_CTL;										// 0x63
		/*-DEFINE-----------------------------------------------------*/
		#define MIC_NO_BOOST						0x00
        #define MIC_BOOST_20DB						0x01
        #define MIC_BOOST_30DB						0x02
        #define MIC_BOOST_40DB						0x03
        #define MIC_LCH_BOOST(_a)					(_a<<2)
        #define MIC_RCH_BOOST(_a)					(_a)
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_HPF_CTL;										// 0x64
		/*-DEFINE-----------------------------------------------------*/
		#define ADC_HPF_EN							0x01
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x65[3];
	AIT_REG_B	AFE_ADC_ANA_LPGA_GAIN;							    // 0x68
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_ANA_RPGA_GAIN;							    // 0x69
		/*-DEFINE-----------------------------------------------------*/
		/*0~1F: -11dB ~ +20dB*/
		
		/*------------------------------------------------------------*/
	AIT_REG_W   AFE_ADC_DIG_GAIN;                                   // 0x6A
		/*-DEFINE-----------------------------------------------------*/
		/*64~87: +0.5dB~+12dB*/
		/*------------------------------------------------------------*/
	AIT_REG_B	FIX_AFE_ADC_OVERFLOW;									// 0x6C
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_DIGITAL_GAIN_MUTE_STEP;							// 0x6D
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_DATA_BIT_WIDTH;									// 0x6E
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B							_0x6F;
	AIT_REG_B	AFE_ADC_BIAS_ADJ;									// 0x70
		/*-DEFINE-----------------------------------------------------*/
		#define ANA_ADC_DISC_OP_MASK            0x0C
        #define ANA_ADC_DISC_OP(_a)             (_a<<2)
        #define ANA_ADC_CONT_OP_MASk            0x03
        #define ANA_ADC_CONT_OP(_a)             (_a)
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_CTL_REG1;									// 0x71
		/*-DEFINE-----------------------------------------------------*/
		#define AFE_ZERO_CROSS_DET				0x10
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_CTL_REG2;									// 0x72
		/*-DEFINE-----------------------------------------------------*/
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_CTL_REG3;									// 0x73
		/*-DEFINE-----------------------------------------------------*/
		
		/*------------------------------------------------------------*/
	AIT_REG_B	AFE_ADC_CTL_REG4;									// 0x74
		/*-DEFINE-----------------------------------------------------*/
		#define ADC_MIC_BIAS_ON                 0x04
		#define ADC_MIC_BIAS_OFF                 0x00		
		#define ADC_MIC_BIAS_VOLTAGE090AVDD     0x03
		#define ADC_MIC_BIAS_VOLTAGE080AVDD     0x02
		#define ADC_MIC_BIAS_VOLTAGE075AVDD     0x01		
		#define ADC_MIC_BIAS_VOLTAGE065AVDD     0x00
		/*------------------------------------------------------------*/
} AITS_AUD, *AITPS_AUD, AITS_AFE,*AITPS_AFE;;
#endif // (CHIP == VSN_V3)


#if (CHIP == MCR_V2)
#pragma pack(1)

// **********************************************************
//   Audio AFE structure (0x8000 7F00)
// **********************************************************
//typedef __packed struct _AUD_FIFO{
typedef struct _AUD_FIFO{
	AIT_REG_B   CPU_INT_EN;                                //  0x40
	AIT_REG_B   HOST_INT_EN;                               //  0x41	
	AIT_REG_B   				_x42[2];
	AIT_REG_B   SR;                                        //  0x44
	#define AFE_INT_FIFO_EMPTY				0x01
	#define AFE_INT_FIFO_FULL				0x02
	#define AFE_INT_FIFO_REACH_UNRD_TH	0x04
	#define AFE_INT_FIFO_REACH_UNWR_TH	0x08	
	AIT_REG_B   				_x45[3];
	AIT_REG_B   RST;                                       //  0x48
	AIT_REG_B   				_x49[3];
	AIT_REG_D   DATA;                                      //  0x4C	
	AIT_REG_W   RD_TH;                                     //  0x50
	AIT_REG_W   				_x52;
	AIT_REG_W   WR_TH;                                     //  0x54
	AIT_REG_W   				_x56;
	AIT_REG_W   UNRD_CNT;                                  //  0x58
	AIT_REG_W   				_x5A;
	AIT_REG_W   UNWR_CNT;                                  //  0x5C
} AUD_FIFO;	

// ********************************
//   Audio structure (0x8000 7800)
//   0x8000 7800 for MERCURY I2S0
//   0x8000 5A00 for MERCURY I2S1
// ********************************
// **********************************************************
//   Audio I2S structure ch0 (0x8000 7800) ch1 (0x8000 5A00)
// **********************************************************
typedef struct _AITS_I2S {
    AIT_REG_B   I2S_FIFO_CPU_INT_EN;                        // 0x00
    AIT_REG_B   I2S_FIFO_HOST_INT_EN;                       // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_INT_FIFO_EMPTY          0x0001
        #define AUD_INT_FIFO_FULL           0x0002
        #define AUD_INT_FIFO_REACH_UNRD_TH  0x0004
        #define AUD_INT_FIFO_REACH_UNWR_TH  0x0008

		/*++MCR_V2 MP ONLY*/
        #define RX_INT_EN_MP(_a)			_a                	
		#define TX_INT_EN_MP(_a)          	_a<<4                
		/*--MCR_V2 MP ONLY*/
		/*------------------------------------------------------------*/    
	AIT_REG_B   I2S_FIFO_INT_SR;                                        // 0x02
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_FIFO_SRC_CH0            0x0001
        #define I2S_FIFO_SRC_CH1            0x0002
        #define I2S_FIFO_SRC_CH2            0x0004
        /*------------------------------------------------------------*/
    AIT_REG_B                          	_x03;
    //__packed union {
    union {
        //__packed struct {
        struct {
            AIT_REG_B   SR;                                             // 0x04
            AIT_REG_B   reserved;                                       // 0x04          
        } SHT;
        //__packed struct {
        struct {
            AIT_REG_B   RX_SR;                                          // 0x05
            AIT_REG_B   TX_SR;                                          // 0x05    
        } MP;       
    }I2S_FIFO_SR;
    AIT_REG_B   						_x06[2];
    AIT_REG_B   I2S_FIFO_RST;                                           // 0x08
        /*-DEFINE-----------------------------------------------------*/
		#define AUD_FIFO_RST_EN             0x0001
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x09[3];
    union {                                                             
        //__packed struct  {
        struct {
            	AIT_REG_D   DATA;       									// 0x0C                                 
		AIT_REG_W	RD_TH;											// 0x10
            	AIT_REG_W                           _x12;
        	AIT_REG_W	WR_TH;											// 0x14
            	AIT_REG_W                           _x16;
		AIT_REG_W	UNRD_CNT;       								// 0x18
		AIT_REG_W							_x1A;       							
		AIT_REG_W	UNWR_CNT;       								// 0x1C
            	AIT_REG_W                           _x1E[5];
        } SHT;
       // __packed struct  {
       struct {
            	AIT_REG_D   	DATA_RX;                                       	// 0x0C 
            	AIT_REG_W	RD_TH_RX;										// 0x10
            	AIT_REG_W                           _x12;
        	AIT_REG_W	WR_TH_RX;										// 0x14
            	AIT_REG_W                           _x16;
		AIT_REG_W	UNRD_CNT_RX;       								// 0x18
		AIT_REG_W							_x1A;       							
		AIT_REG_W	UNWR_CNT_RX;       								// 0x1C
            	AIT_REG_W                           _x1E;
            	AIT_REG_D   	DATA_TX;                            			// 0x20
            	AIT_REG_W   	UNRD_CNT_TX;                                	// 0x24
            	AIT_REG_W   	UNWR_CNT_TX;                                	// 0x26
        } MP;
    } I2S_FIFO;   			                                    
    AIT_REG_B   I2S_CTL;                                                // 0x28
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_SDO_OUT_EN              0x08
		#define I2S_LRCK_OUT_EN             0x04
		#define I2S_BCK_OUT_EN              0x02
		#define I2S_HCK_CLK_EN              0x01
		#define I2S_ALL_DIS                 0x0
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x29;
    AIT_REG_W   I2S_FIFO_RD_TH_TX_MP;                                   // 0x2A	    
    AIT_REG_B   I2S_CLK_DIV;                                            // 0x2C
    AIT_REG_B   I2S_CLK_CTL;                                            // 0x2D
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_MCLK_FIX                0x01
		/*------------------------------------------------------------*/
    AIT_REG_W   I2S_FIFO_WR_TH_TX_MP;									// 0x2E
    AIT_REG_W   I2S_RATIO_N_M;                              // 0x30
    AIT_REG_W                           _x32;
    AIT_REG_B   I2S_BIT_CLT;                                // 0x34
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_OUT_32BITS              0x04
        #define I2S_OUT_24BITS              0x02
		#define I2S_OUT_16BITS              0x01
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x35[3];
    AIT_REG_B   I2S_LRCK_POL;                                           // 0x38
        /*-DEFINE-----------------------------------------------------*/
		#define I2S_LRCK_L_CH_HIGH          0x01
		#define I2S_LRCK_L_CH_LOW           0x00
		/*------------------------------------------------------------*/
    AIT_REG_B	I2S_MULTI_CH_ENA_MP;									// 0x39
		#define MULTI_CH_ENA              	0x01
    AIT_REG_B	I2S_MULTI_CH_BIT_PER_CH_MP;								// 0x3A
	    /*-DEFINE-----------------------------------------------------*/
		#define SEL_20_BITS_ENA              0x01
		#define SEL_24_BITS_ENA              0x02
		#define SEL_32_BITS_ENA              0x04
		#define MULTI_CH_BIT_MASK            0x07
	    /*------------------------------------------------------------*/
    AIT_REG_B	I2S_MULTI_CH_CNT_MP;									// 0x3B
	    /*-DEFINE-----------------------------------------------------*/
		#define SEL_4_CH_ENA              	0x01
		#define SEL_5_CH_ENA              	0x02
		#define SEL_6_CH_ENA              	0x04
		#define SEL_7_CH_ENA              	0x08
		#define SEL_8_CH_ENA              	0x10
		#define MULTI_CH_MASK				0x1F
	    /*------------------------------------------------------------*/
	    
    AIT_REG_D   I2S_L_CHNL_DATA;                                        // 0x3C

    AIT_REG_D   I2S_R_CHNL_DATA;                            // 0x40

    AIT_REG_B   I2S_BIT_ALIGN_OUT;                          // 0x44
    AIT_REG_B   I2S_BIT_ALIGN_IN;                           // 0x45
    AIT_REG_W                           _x46;
    AIT_REG_B   I2S_MODE_CTL;                               // 0x48
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_MCLK_OUT_EN             4
        #define I2S_SLAVE                   0
        #define I2S_MASTER                  1
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x49[3];
    AIT_REG_B   I2S_MCLK_CTL;                                           // 0x4C
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_1536_FS                 0x80
        #define I2S_1024_FS                 0x40
        #define I2S_768_FS                  0x20
        #define I2S_512_FS                  0x10
    	#define I2S_384_FS				    0x08
    	#define I2S_256_FS				    0x04
    	#define I2S_192_FS				    0x02
    	#define I2S_128_FS				    0x01
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x4D[3];
    AIT_REG_B   I2S_MSPORT_CTL;                                         // 0x50
    AIT_REG_B   I2S_MSPORT_START_BIT;                                   // 0x51
    AIT_REG_W   						_x52;
    AIT_REG_B   I2S_DATA_OUT_EN;                            // 0x54
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_SDI_OUT    			    0x02
		/*------------------------------------------------------------*/
    AIT_REG_B   						_x55[3];
    AIT_REG_B   I2S_DATA_IN_SEL;                                        // 0x58
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_SDO_IN     			    0x01
        #define I2S_SDI_IN    			    0x00
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x59[3];
    AIT_REG_B   I2S_CPU_INT_EN;                                         // 0x5C
    AIT_REG_B   I2S_HOST_INT_EN;                                        // 0x5D
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_INT_EN     		        0x01
        #define AUD_INT_DIS    			    0x00
		/*------------------------------------------------------------*/
    AIT_REG_B   I2S_INT_SR;                                             // 0x5E
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_INT_SRC_CH0             0x0001
        #define I2S_INT_SRC_CH1             0x0002
        /*------------------------------------------------------------*/
    AIT_REG_B   						_x5F;
    AIT_REG_B   I2S_OUT_MODE_CTL;                                       // 0x60
        /*-DEFINE-----------------------------------------------------*/
        #define	I2S_STD_MODE			    0x01
        #define	I2S_I2S_MODE			    0x00
        /*------------------------------------------------------------*/
    AIT_REG_B   						_x61[3];
    AIT_REG_W   I2S_MSPORT_FR_CYCLE_CNT;                                // 0x64                             
    AIT_REG_B   I2S_MSPORT_FR_SYNC_CNT;                                 // 0x66
    AIT_REG_B                           _x67;
    AIT_REG_B   I2S_SDO_SHIFT_RST;                                      // 0x68
    AIT_REG_B                           _x69[7];

    AIT_REG_B   I2S_MUX_MODE_CTL;                           // 0x70
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_MUX_AUTO                0x01
        #define AUD_MUX_CPU                 0x00
		/*------------------------------------------------------------*/    
    AIT_REG_B   						_x71[3];
    AIT_REG_B   DUPLEX_PATH_SEL;                                        // 0x74
        /*-DEFINE-----------------------------------------------------*/
        #define AFE_FULL_DUPLEX_I2S1_EN     0x08 //DAC -> AFE FIFO, ADC -> I2S1 FIFO
        #define AFE_FULL_DUPLEX_I2S0_EN     0x04 //DAC -> AFE FIFO, ADC -> I2S0 FIFO
        #define I2S1_FULL_DUPLEX_EN         0x02 //SDO -> I2S1 FIFO, SDI -> AFE FIFO
		#define I2S0_FULL_DUPLEX_EN         0x01 //SDO -> I2S0 FIFO, SDI -> AFE FIFO
		/*------------------------------------------------------------*/
	
	    /*-DEFINE-----------------------------------------------------*/
		/*++MCR_V2 MP ONLY*/
        #define I2S2_FULL_DUPLEX_EN_MP     	0x10 //SDO -> I2S2 Tx FIFO, SDI -> I2S2 Rx FIFO
        #define I2S1_FULL_DUPLEX_EN_MP      0x02 //SDO -> I2S1 Tx FIFO, SDI -> I2S1 Rx FIFO
		#define I2S0_FULL_DUPLEX_EN_MP      0x01 //SDO -> I2S0 Tx FIFO, SDI -> I2S0 Rx FIFO
		/*--MCR_V2 MP ONLY*/
		/*------------------------------------------------------------*/
	AIT_REG_B   						_x75[3];
    AIT_REG_B   I2S_PATH_CTL;                                           // 0x78
        /*-DEFINE-----------------------------------------------------*/
        #define ThirdType_SYNC_MODE_EN	    0x40  
        #define I2S1_SDI_LOOPBACK_DAC       0x20 //I2S1 SDI -> DAC without pass FIFO
        #define I2S0_SDI_LOOPBACK_DAC       0x10 //I2S0 SDI -> DAC without pass FIFO
		/*++MCR_V2 MP ONLY*/
		#define ADC_LOOPBACK_I2S2_SDO_MP    0x04 //ADC -> I2S2 SDO without pass FIFO
		/*--MCR_V2 MP ONLY*/
        #define ADC_LOOPBACK_I2S1_SDO       0x02 //ADC -> I2S1 SDO without pass FIFO
        #define ADC_LOOPBACK_I2S0_SDO       0x01 //ADC -> I2S0 SDO without pass FIFO
        /*------------------------------------------------------------*/
    AIT_REG_B   I2S_MISC_CTL;                                           // 0x79
        /*-DEFINE-----------------------------------------------------*/
		/*++MCR_V2 MP ONLY*/
		#define I2S2_MCLK_AS_SYSCLK_MP			0x10    //AFE use I2S2 MCLK as SYSCLK
		#define I2S1_MCLK_AS_SYSCLK_MP         	0x08    //AFE use I2S1 MCLK as SYSCLK
		#define I2S0_MCLK_AS_SYSCLK_MP         	0x00	//AFE use I2S0 MCLK as SYSCLK
		#define I2S_MCLK_AS_SYSCLK_INV_CTL_MP	0x04	//AFE use inverter I2S MCLK as SYSCLK
		#define I2S_MCLK_AS_SYSCLK_ENA_MP       0x02	//AFE use I2S MCLK as SYSCLK(enable)
		/*--MCR_V2 MP ONLY*/
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x7A[6];
    AIT_REG_B   I2S_TIMER_EN;                                           // 0x80
    AIT_REG_B   						_x81[3];
    AIT_REG_W   I2S_TIMER_CTL;                                          // 0x84
    AIT_REG_W   						_x86;
    AIT_REG_D   I2S_TIMER_TAGET_VALUE;                                  // 0x88
    AIT_REG_D   I2S_TIMER_CUR_VALUE;                                    // 0x8C
    AIT_REG_B   I2S_TIMER_CPU_INT_EN;                                   // 0x90
    AIT_REG_B   I2S_TIMER_HOST_INT_EN;                                  // 0x91	
    AIT_REG_B   						_x92[2];
    
    AIT_REG_B   I2S0_TIMER_INT_SR;	                                    // 0x94
    AIT_REG_B   I2S1_TIMER_INT_SR;	                                    // 0x95
    
    AIT_REG_B   						_x96[2];
    AIT_REG_B   I2S_TIMER_RST;                                          // 0x98
    
    AIT_REG_B                           _x99[7];                        // 0x99~9F
    AIT_REG_D  I2S_TIMER_TARGET_CNT_VALUE_LOW;                          // 0xA0
    AIT_REG_D  I2S_TIMER_TARGET_CNT_VALUE_HIG;                          // 0xA4
    
    AIT_REG_D  I2S_TIMER_CUR_CNT_VALUE_LOW;                             // 0xA8
    AIT_REG_D  I2S_TIMER_CUR_CNT_VALUE_HIG;                             // 0xAC
    AIT_REG_B                           _xB0[16];                       // 0xB0~BF
   
    AIT_REG_D   SPDIF_CHANNEL_SR[6];                        // 0xC0
    AIT_REG_W   SPDIF_CTL;                                  // 0xD8
        /*-DEFINE-----------------------------------------------------*/
        #define HDMI_TRANS_EN               0x0200
        #define SPDIF_TRANS_EN              0x0100
        /*------------------------------------------------------------*/
    AIT_REG_B   SPDIF_DATA_ALIGN_FMT;                                   // 0xDA
    AIT_REG_B                           _xDB;
    AIT_REG_B   SPDIF_RST;                                              // 0xDC
    AIT_REG_B                           _xDD[3];                        // 0xDD~DF
    AIT_REG_B   HDMI_ECC;                                               // 0xE0
    AIT_REG_B                           _xE1[3];                        // 0xE1~E3
    AIT_REG_B   AFE_SAMPLE_RATE_SEL;                                    // 0xE4
        /*-DEFINE-----------------------------------------------------*/
        #define SRATE_192000Hz              0x02
        #define SRATE_96000Hz               0x01
        #define SRATE_48000Hz_UNDER         0x00
        /*------------------------------------------------------------*/
    AIT_REG_B                           _xE5[3];                        // 0xE5~E7
    AIT_REG_B   AFE_DOWN_SAMPLE_SEL;                                    // 0xE8
        /*-DEFINE-----------------------------------------------------*/
        #define DOWN_SAMPLE_4               0x02
        #define DOWN_SAMPLE_2               0x01
        #define DOWN_SAMPLE_OFF             0x00
        /*------------------------------------------------------------*/
    AIT_REG_B                             _xE9[7];                      // 0xE9~0xEF
    AIT_REG_B  SRAM_LIGHT_SLEEP_MOD_CTL;     	                        // 0xF0
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_LEFT_CHAN_LIGHT_MODE    0x20
        #define AFE_RCHAN_LIGHT_MODE        0x10
        #define AFE_LCHAN_LIGHT_MODE        0x08
        #define AFE_FIFO_LIGHT_MODE         0x04
        #define IS21_FIFO_LIGHT_MODE        0x02
        #define IS20_FIFO_LIGHT_MODE        0x01
        /*------------------------------------------------------------*/
    AIT_REG_B  SRAM_DEEP_SLEEP_MOD_CTL;     	                        // 0xF1
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_LEFT_CHAN_DEEP_MODE     0x20
        #define AFE_RCHAN_DEEP_MODE         0x10
        #define AFE_LCHAN_DEEP_MODE         0x08
        #define AFE_FIFO_DEEP_MODE          0x04
        #define IS21_FIFO_DEEP_MODE         0x02
        #define IS20_FIFO_DEEP_MODE         0x01
        /*------------------------------------------------------------*/
    AIT_REG_B  SRAM_SHUT_DOWN_MOD_CTL;     	                            // 0xF2
       /*-DEFINE-----------------------------------------------------*/
       #define DAC_LEFT_CHAN_STDOWN_MODE    0x20
       #define AFE_RCHAN_STDOWN_MODE        0x10
       #define AFE_LCHAN_STDOWN_MODE        0x08
       #define AFE_FIFO_STDOWN_MODE         0x04
       #define IS21_FIFO_STDOWN_MODE        0x02
       #define IS20_FIFO_STDOWN_MODE        0x01
       /*------------------------------------------------------------*/
} AITS_I2S, *AITPS_I2S;

typedef struct _AITS_AUD {
    AIT_REG_B   AFE_POWER_CTL;                                          // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define ANALOG_POWER_EN             0x0020
        #define VREF_POWER_EN               0x0010
        #define ADC_DF_POWER_EN             0x0002
        #define DAC_DF_POWER_EN             0x0001
        #define AFE_POWER_OFF               0x0000
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x01;
    AIT_REG_B   AFE_CLK_CTL;                                            // 0x02
        /*-DEFINE-----------------------------------------------------*/
        #define DMIC_EN                     0x40
        #define CODEC_ADC_64FS_EN           0x20
        #define CODEC_ADC_128FS_EN          0x10
        #define DAC_CLK_256FS_MODE          0x00
        /*now ADC only 256fs mode*/
        #define DAC_CLK_USB_MODE            0x08
        #define DAC_CLK_INV_EN              0x02
        #define ADC_CLK_INV_EN              0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   AFE_SAMPLE_RATE_CTL;                                    // 0x03
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_SRATE_MASK              0xF0
        #define DAC_SRATE(_a)               (_a<<4)
        
        #define ADC_SRATE_MASK              0x0F
        #define ADC_SRATE(_a)               (_a)
        #define SRATE_48000HZ               0x0A
        #define SRATE_44100HZ               0x09
        #define SRATE_32000HZ               0x08
        #define SRATE_24000HZ               0x06
        #define SRATE_22050HZ               0x05
        #define SRATE_16000HZ               0x04
        #define SRATE_12000HZ               0x02
        #define SRATE_11025HZ               0x01
        #define SRATE_8000HZ                0x00
        /*------------------------------------------------------------*/        
    AIT_REG_B                           _x04;      
        
    AIT_REG_B   AFE_DAC_FILTER_CTL;                                     // 0x05
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_DITHER_EN               0x80
        #define DAC_DITHER_SCALE_2_1        0x60
        #define DAC_DITHER_SCALE_1_1        0x40
        #define DAC_DITHER_SCALE_1_2        0x20
        #define DAC_DITHER_SCALE_1_4        0x00
        #define DAC_64_FS                   0x04
        #define DAC_128_FS                  0x02
        #define DAC_256_FS                  0x01
        /*------------------------------------------------------------*/
    AIT_REG_W   AFE_DAC_DIG_GAIN_CTL;                                   // 0x06~0x07
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_R_SOFT_MUTE             0x8000
        #define DAC_R_GAIN_DIRECT_SET       0x4000
        #define DAC_L_SOFT_MUTE             0x0080
        #define DAC_L_GAIN_DIRECT_SET       0x0040
        /*------------------------------------------------------------*/  
    AIT_REG_B  AFE_MICBIAS_DUAL_CHAN;                                   // 0x08
        /*-DEFINE-----------------------------------------------------*/
        #define ADC_MIC_BIAS_VOLT_MASK      	0xFF
        #define ADC_MIC_BIAS_R_POWER_UP     	0x40
        #define ADC_MIC_BIAS_R_VOLT_0D6     	0x00
        #define ADC_MIC_BIAS_R_VOLT_0D65    	0x10
        #define ADC_MIC_BIAS_R_VOLT_0D75    	0x20
        #define ADC_MIC_BIAS_R_VOLT_0D85    	0x30

		/*++ MCP_V2 MP only*/
		#define ADC_MIC_BIAS_R_VOLT_0D65_MP    	0x00
		#define ADC_MIC_BIAS_R_VOLT_0D75_MP    	0x10
		#define ADC_MIC_BIAS_R_VOLT_0D85_MP    	0x20
		#define ADC_MIC_BIAS_R_VOLT_0D95_MP    	0x30
		/*-- MCP_V2 MP only*/
		
        #define ADC_MIC_BIAS_L_POWER_UP     	0x04
        #define ADC_MIC_BIAS_L_VOLT_0D6     	0x00
        #define ADC_MIC_BIAS_L_VOLT_0D65    	0x01
        #define ADC_MIC_BIAS_L_VOLT_0D75    	0x02
		#define ADC_MIC_BIAS_L_VOLT_0D85    	0x03
		
		/*++ MCP_V2 MP only*/
		#define ADC_MIC_BIAS_L_VOLT_0D65_MP   	0x00
		#define ADC_MIC_BIAS_L_VOLT_0D75_MP   	0x01
		#define ADC_MIC_BIAS_L_VOLT_0D85_MP   	0x02
        #define ADC_MIC_BIAS_L_VOLT_0D95_MP   	0x03
		/*-- MCP_V2 MP only*/
        /*------------------------------------------------------------*/
    AIT_REG_B  AFE_ANA_ADC_POWER_CTL;                                   // 0x09    
        /*-DEFINE-----------------------------------------------------*/
        #define ADC_R_POWER_UP              0x08
        #define ADC_L_POWER_UP              0x04
        #define ADC_PGA_R_POWER_EN          0x02
        #define ADC_PGA_L_POWER_EN          0x01
        /*------------------------------------------------------------*/
    AIT_REG_B  AFE_ADC_ANA_LPGA_CTL;                                    // 0x0A    
        /*-DEFINE-----------------------------------------------------*/
        #define LPGA_ZC_DIS                0x00
        /*++MCP_V2 SHT ONLY*/
        #define IN_LPGA_ZC_EN              0x08
        #define OUT_LPGA_ZC_EN             0x10
        /*--MCP_V2 SHT ONLY*/
		/*++MCP_V2 MP ONLY*/        
		#define OUT_LPGA_ZC_EN_MP          0x08
		/*--MCP_V2 MP ONLY*/
        #define LPGA_SRC_IN_MICLIP_LIN     0x00
        #define LPGA_SRC_IN_MICLIP         0x02
        #define LPGA_SRC_IN_AUXL           0x04
        #define LPGA_SRC_IN_MASK           0x06
        #define LPGA_MUTE_CTL              0x01
		/*------------------------------------------------------------*/
    AIT_REG_B  AFE_ADC_ANA_LPGA_GAIN;                                   // 0x0B
    AIT_REG_B  AFE_ADC_ANA_RPGA_CTL;                                    // 0x0C   
         /*-DEFINE-----------------------------------------------------*/
         #define RPGA_ZC_DIS                0x00
         /*++MCP_V2 SHT ONLY*/
         #define IN_RPGA_ZC_EN              0x08
         #define OUT_RPGA_ZC_EN             0x10
         /*--MCP_V2 SHT ONLY*/
         /*++MCP_V2 MP ONLY*/        
		 #define OUT_RPGA_ZC_EN_MP          0x08
         /*--MCP_V2 MP ONLY*/
         #define RPGA_SRC_IN_MICRIP_RIN     0x00
         #define RPGA_SRC_IN_MICRIP         0x02
         #define RPGA_SRC_IN_AUXR           0x04
         #define RPGA_SRC_IN_MASK           0x06
         #define RPGA_MUTE_CTL              0x01
         /*------------------------------------------------------------*/
    AIT_REG_B  AFE_ADC_ANA_RPGA_GAIN;                                   // 0x0D
    
    AIT_REG_B  AFE_LADC_DITH_AND_TEST;                                  // 0x0E
    AIT_REG_B  AFE_RADC_DITH_AND_TEST;                                  // 0x0F
    
    AIT_REG_D  AFE_L_CHNL_DATA;                                         // 0x10
    AIT_REG_D  AFE_R_CHNL_DATA;                                         // 0x14
    AIT_REG_B  AFE_CPU_INT_EN;                                          // 0x18
        /*-DEFINE-----------------------------------------------------*/
        #define AUD_L_TARGET_GAIN_REACH_INT_EN              0x08
        #define AUD_R_TARGET_GAIN_REACH_INT_EN              0x04
        #define AUD_ADC_INT_EN                              0x02
        #define AUD_DAC_INT_EN                              0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   AFE_HOST_INT_EN;                                        // 0x19
    
    AIT_REG_W   AFE_ADC_DIG_GAIN;                                       // 0x1A~0x1B
         /*-DEFINE-----------------------------------------------------*/
         #define ADC_DIG_GAIN_MUTE          0x0000
         #define ADC_DIG_GAIN_0DB           0x5151
         /*------------------------------------------------------------*/
    AIT_REG_B   AFE_ANA_DAC_POWER_CTL;                                  // 0x1C
        /*-DEFINE-----------------------------------------------------*/
        #define LINEOUT_POWER_UP            0x02
        #define DAC_POWER_UP                0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   AFE_DAC_LOUT_VOL;                                       // 0x1D
        /*-DEFINE-----------------------------------------------------*/
        /* VOLUME setting bit range : 1~5*/
        #define   LOUT_DB2BITS(db)			((((0-db) * 2) / 3) << 1)
        #define   LOUT_ANA_GAIN_MUTE        0x3E
        #define   LOUT_ANA_GAIN_MASK        0x3E
        #define   LOUT_ZC_GAIN_CTL_ACT      0x01   /*Active line_out gain control by zero crossing*/
        /*------------------------------------------------------------*/
    AIT_REG_B                        _x1E[2];  
    AIT_REG_B   AFE_DIG_GAIN_MUTE_STEP;                                 // 0x20
        /*-DEFINE-----------------------------------------------------*/
        #define ADC_PGA_SMOOTH_METHOD       0x80
        #define ADC_PGA_DIRECT_METHOD       0x40
        #define DAC_PGA_SMOOTH_METHOD       0x20
        #define DAC_PGA_DIRECT_METHOD       0x10
        #define ADC_PGA_UPDATE_DIS          0x08
        #define DAC_PGA_UPDATE_DIS          0x04
        /* DAC gain update period (ramp up/down step size) when mute enable or DAC digital gain not using zero crossing detect */
        #define DAC_DIG_PGA_MUTE_1FSUP      0x00
        #define DAC_DIG_PGA_MUTE_2FSUP      0x01
        #define DAC_DIG_PGA_MUTE_4FSUP      0x02
        #define DAC_DIG_PGA_MUTE_8FSUP      0x03
        /*------------------------------------------------------------*/
    AIT_REG_B   AFE_DAC_DF_BUGFIX;                                      // 0x21
    AIT_REG_B                           _x22;
    AIT_REG_B   AFE_DIG_SOFT_MUTE_SR;                                   // 0x23
    AIT_REG_W   AFE_DAC_DIG_GAIN;                                       // 0x24
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_L_DIG_MUTE              0x00
        #define DAC_DIG_GAIN_MASK           0x7F
        /*------------------------------------------------------------*/
    AIT_REG_B   AFE_OVF_BUGFIX;                                         // 0x26
        /*-DEFINE-----------------------------------------------------*/
        #define DAC_DIG_VOL_OVF_FIX         0x08
        #define DAC_SDM_OVF_FIX             0x04
        /*------------------------------------------------------------*/
    AIT_REG_B                           _x27;
    AIT_REG_B                           _x28[10];

    AIT_REG_B   AFE_LADC_DIG_TIME_OUT[3];                               //  0x32
     
    AIT_REG_B   AFE_LADC_DIG_STEP_TIME[3];                              //  0x35
     
    AIT_REG_B   AFE_RADC_DIG_TIME_OUT[3];                               //  0x38
   
    AIT_REG_B   AFE_RADC_DIG_STEP_TIME[3];                              //  0x3B
       
    AIT_REG_B   AFE_DIG_GAIN_SETTING_CTL;                               //  0x3E
        /*-DEFINE-----------------------------------------------------*/
        #define AFE_DIG_GAIN_SMOOTH_METHOD  0x08
        #define AFE_LR_MUTE_ENA             0x03
        /*------------------------------------------------------------*/ 
    AIT_REG_B                           _x3F;
    //__packed union {
    union {
    		AUD_FIFO SHT;													// 	0x40
		AUD_FIFO DAC_MP;												// 	0x40
	} AFE_FIFO;	//DAC_FIFO
	AIT_REG_B   						_x5E[2];
	AIT_REG_B   AFE_ANA_L_TIME_OUT[3];                                  //  0x60	
	AIT_REG_B   AFE_ANA_L_STEP_TIME[3];                                 //  0x63	
	AIT_REG_B   AFE_ANA_R_TIME_OUT[3];                                  //  0x66		
	AIT_REG_B   AFE_ANA_R_STEP_TIME[3];                                 //  0x69
	AIT_REG_B   AFE_ANA_GAIN_SETTING_CTL;                               //  0x6C
	    /*-DEFINE-----------------------------------------------------*/
	    #define AFE_ANA_GAIN_DIRECT_METHOD          0x20
	    #define AFE_ANA_GAIN_SMOOTH_METHOD          0x10
	    #define TIME_OUT_PULSE_ENA                  0x01
	    /*------------------------------------------------------------*/ 
	AIT_REG_B   						_x6D[3];

	AIT_REG_B   AFE_MUX_MODE_CTL;                                       //  0x70
        /*-DEFINE-----------------------------------------------------*/
        #define I2S_CH2_DATA_24BIT          0x80
        #define I2S_CH2_DATA_20BIT          0x40
        #define I2S_CH2_DATA_16BIT          0x00
        #define I2S_CH2_DATA_MASK           0x3F
        #define I2S_CH1_DATA_24BIT          0x20
        #define I2S_CH1_DATA_20BIT          0x10
        #define I2S_CH1_DATA_16BIT          0x00
        #define I2S_CH1_DATA_MASK           0xCF
        #define I2S_CH0_DATA_24BIT          0x08
        #define I2S_CH0_DATA_20BIT          0x04
        #define I2S_CH0_DATA_16BIT          0x00
        #define I2S_CH0_DATA_MASK           0xF3
        #define AFE_MUX_AUTO_MODE           0x01
		/*------------------------------------------------------------*/ 
    AIT_REG_B   AFE_DATA_BIT_TRANS;                                     // 0x71
        /*-DEFINE-----------------------------------------------------*/
        #define AFE_ADC_DATA_BIT_MASK       0xF0
        #define AFE_ADC_DATA_BIT_SEL(_a)    (_a<<4)
        #define AFE_DAC_DATA_BIT_MASK       0x07
        #define AFE_DAC_DATA_BIT_SEL(_a)    (_a)
        /*------------------------------------------------------------*/ 
    AIT_REG_B   AFE_DMIC_CLK_DATA_CTL;                                  // 0x72
    AIT_REG_B   AFE_ADDA_BIT_MOD;                                       // 0x73
    
    AIT_REG_B   AUDIO_CODEC_PATH_CTL;                                   // 0x74
    AIT_REG_B                           _x75[11];
    //__packed union {
    union {
    		AIT_REG_B  						_x80[30];						
		AUD_FIFO  MP;													// 0x80
	}ADC_FIFO;
    AIT_REG_B                           _x9E[74];
    AIT_REG_B   AFE_ADC_LOOP_CTL;                                       // 0xE8
} AITS_AUD, *AITPS_AUD, AITS_AFE,*AITPS_AFE;

// **********************************************************
//   New Audio Digital Filter structure (0x8000 6500)
// **********************************************************
typedef struct _AITS_DADC_EXT {
    AIT_REG_W   DADC_HPF_MODE_SEL;                                      // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define ADC_HPF_BYPASS              0x0000
        #define ADC_HPF_AUD                 0x0101
        #define ADC_HPF_VOC                 0x0202
        /*------------------------------------------------------------*/
    AIT_REG_W   DADC_HPF_AUD_MODE_COEF;                                 // 0x02
        /*-DEFINE-----------------------------------------------------*/
        #define HPF_AUD_FC_2HZ              0x0000
        #define HPF_AUD_FC_4HZ              0x0101
        #define HPF_AUD_FC_8HZ              0x0202
        #define HPF_AUD_FC_16HZ             0x0303
        /*------------------------------------------------------------*/
    AIT_REG_W   DADC_HPF_VOC_MODE_COEF;                                 // 0x04
        /*-DEFINE-----------------------------------------------------*/
        #define HPF_VOC_FC_2D5HZ            0x0000
        #define HPF_VOC_FC_25HZ             0x0101
        #define HPF_VOC_FC_50HZ             0x0202
        #define HPF_VOC_FC_100HZ            0x0303
        #define HPF_VOC_FC_150HZ            0x0404
        #define HPF_VOC_FC_200HZ            0x0505
        #define HPF_VOC_FC_300HZ            0x0606
        #define HPF_VOC_FC_400HZ            0x0707
        /*------------------------------------------------------------*/
    AIT_REG_B   DADC_L_EQ_LSF_COEF;                                     // 0x06
    AIT_REG_B   DADC_L_EQ_PK1_COEF;                                     // 0x07
    AIT_REG_B   DADC_L_EQ_PK2_COEF;                                     // 0x08
    AIT_REG_B   DADC_L_EQ_PK3_COEF;                                     // 0x09
    AIT_REG_B   DADC_L_EQ_HSF_COEF;                                     // 0x0A
     
    AIT_REG_B   DADC_R_EQ_LSF_COEF;                                     // 0x0B
    AIT_REG_B   DADC_R_EQ_PK1_COEF;                                     // 0x0C
    AIT_REG_B   DADC_R_EQ_PK2_COEF;                                     // 0x0D
    AIT_REG_B   DADC_R_EQ_PK3_COEF;                                     // 0x0E
    AIT_REG_B   DADC_R_EQ_HSF_COEF;                                     // 0x0F

    AIT_REG_B   DADC_L_NF0_B0_COEF_LOW[2];                              // 0x10
    AIT_REG_B   DADC_L_NF0_B0_COEF_HIG;                                 // 0x12
    AIT_REG_B   DADC_L_NF0_B1_COEF_LOW[2];                              // 0x13
    AIT_REG_B   DADC_L_NF0_B1_COEF_HIG;                                 // 0x15
    AIT_REG_B   DADC_L_NF0_B2_COEF_LOW[2];                              // 0x16
    AIT_REG_B   DADC_L_NF0_B2_COEF_HIG;                                 // 0x18

    AIT_REG_B   DADC_L_NF0_A1_COEF_LOW[2];                              // 0x19
    AIT_REG_B   DADC_L_NF0_A1_COEF_HIG;                                 // 0x1B
    AIT_REG_B   DADC_L_NF0_A2_COEF_LOW[2];                              // 0x1C
    AIT_REG_B   DADC_L_NF0_A2_COEF_HIG;                                 // 0x1E

    AIT_REG_B    _x1F;
    AIT_REG_B   DADC_L_NF1_B0_COEF_LOW[2];                              // 0x20
    AIT_REG_B   DADC_L_NF1_B0_COEF_HIG;                                 // 0x22
    AIT_REG_B   DADC_L_NF1_B1_COEF_LOW[2];                              // 0x23
    AIT_REG_B   DADC_L_NF1_B1_COEF_HIG;                                 // 0x25
    AIT_REG_B   DADC_L_NF1_B2_COEF_LOW[2];                              // 0x26
    AIT_REG_B   DADC_L_NF1_B2_COEF_HIG;                                 // 0x28

    AIT_REG_B   DADC_L_NF1_A1_COEF_LOW[2];                              // 0x29
    AIT_REG_B   DADC_L_NF1_A1_COEF_HIG;                                 // 0x2B
    AIT_REG_B   DADC_L_NF1_A2_COEF_LOW[2];                              // 0x2C
    AIT_REG_B   DADC_L_NF1_A2_COEF_HIG;                                 // 0x2E

    AIT_REG_B    _x2F;
    AIT_REG_B   DADC_L_NF2_B0_COEF_LOW[2];                              // 0x30
    AIT_REG_B   DADC_L_NF2_B0_COEF_HIG;                                 // 0x32
    AIT_REG_B   DADC_L_NF2_B1_COEF_LOW[2];                              // 0x33
    AIT_REG_B   DADC_L_NF2_B1_COEF_HIG;                                 // 0x35
    AIT_REG_B   DADC_L_NF2_B2_COEF_LOW[2];                              // 0x36
    AIT_REG_B   DADC_L_NF2_B2_COEF_HIG;                                 // 0x38

    AIT_REG_B   DADC_L_NF2_A1_COEF_LOW[2];                              // 0x39
    AIT_REG_B   DADC_L_NF2_A1_COEF_HIG;                                 // 0x3B
    AIT_REG_B   DADC_L_NF2_A2_COEF_LOW[2];                              // 0x3C
    AIT_REG_B   DADC_L_NF2_A2_COEF_HIG;                                 // 0x3E
    AIT_REG_B    _x3F;

    AIT_REG_B   DADC_L_NF3_B0_COEF_LOW[2];                              // 0x40
    AIT_REG_B   DADC_L_NF3_B0_COEF_HIG;                                 // 0x42
    AIT_REG_B   DADC_L_NF3_B1_COEF_LOW[2];                              // 0x43
    AIT_REG_B   DADC_L_NF3_B1_COEF_HIG;                                 // 0x45
    AIT_REG_B   DADC_L_NF3_B2_COEF_LOW[2];                              // 0x46
    AIT_REG_B   DADC_L_NF3_B2_COEF_HIG;                                 // 0x48

    AIT_REG_B   DADC_L_NF3_A1_COEF_LOW[2];                              // 0x49
    AIT_REG_B   DADC_L_NF3_A1_COEF_HIG;                                 // 0x4B
    AIT_REG_B   DADC_L_NF3_A2_COEF_LOW[2];                              // 0x4C
    AIT_REG_B   DADC_L_NF3_A2_COEF_HIG;                                 // 0x4E

    AIT_REG_B    _x4F;

    AIT_REG_B   DADC_L_NF4_B0_COEF_LOW[2];                              // 0x50
    AIT_REG_B   DADC_L_NF4_B0_COEF_HIG;                                 // 0x52
    AIT_REG_B   DADC_L_NF4_B1_COEF_LOW[2];                              // 0x53
    AIT_REG_B   DADC_L_NF4_B1_COEF_HIG;                                 // 0x55
    AIT_REG_B   DADC_L_NF4_B2_COEF_LOW[2];                              // 0x56
    AIT_REG_B   DADC_L_NF4_B2_COEF_HIG;                                 // 0x58

    AIT_REG_B   DADC_L_NF4_A1_COEF_LOW[2];                              // 0x59
    AIT_REG_B   DADC_L_NF4_A1_COEF_HIG;                                 // 0x5B
    AIT_REG_B   DADC_L_NF4_A2_COEF_LOW[2];                              // 0x5C
    AIT_REG_B   DADC_L_NF4_A2_COEF_HIG;                                 // 0x5E

    AIT_REG_B    _x5F;

    AIT_REG_B   DADC_R_NF0_B0_COEF_LOW[2];                              // 0x60
    AIT_REG_B   DADC_R_NF0_B0_COEF_HIG;                                 // 0x62
    AIT_REG_B   DADC_R_NF0_B1_COEF_LOW[2];                              // 0x63
    AIT_REG_B   DADC_R_NF0_B1_COEF_HIG;                                 // 0x65
    AIT_REG_B   DADC_R_NF0_B2_COEF_LOW[2];                              // 0x66
    AIT_REG_B   DADC_R_NF0_B2_COEF_HIG;                                 // 0x68

    AIT_REG_B   DADC_R_NF0_A1_COEF_LOW[2];                              // 0x69
    AIT_REG_B   DADC_R_NF0_A1_COEF_HIG;                                 // 0x6B
    AIT_REG_B   DADC_R_NF0_A2_COEF_LOW[2];                              // 0x6C
    AIT_REG_B   DADC_R_NF0_A2_COEF_HIG;                                 // 0x6E
    AIT_REG_B    _x6F;

    AIT_REG_B   DADC_R_NF1_B0_COEF_LOW[2];                              // 0x70
    AIT_REG_B   DADC_R_NF1_B0_COEF_HIG;                                 // 0x72
    AIT_REG_B   DADC_R_NF1_B1_COEF_LOW[2];                              // 0x73
    AIT_REG_B   DADC_R_NF1_B1_COEF_HIG;                                 // 0x75
    AIT_REG_B   DADC_R_NF1_B2_COEF_LOW[2];                              // 0x76
    AIT_REG_B   DADC_R_NF1_B2_COEF_HIG;                                 // 0x78

    AIT_REG_B   DADC_R_NF1_A1_COEF_LOW[2];                              // 0x79
    AIT_REG_B   DADC_R_NF1_A1_COEF_HIG;                                 // 0x7B
    AIT_REG_B   DADC_R_NF1_A2_COEF_LOW[2];                              // 0x7C
    AIT_REG_B   DADC_R_NF1_A2_COEF_HIG;                                 // 0x7E
    AIT_REG_B    _x7F;

    AIT_REG_B   DADC_R_NF2_B0_COEF_LOW[2];                              // 0x80
    AIT_REG_B   DADC_R_NF2_B0_COEF_HIG;                                 // 0x82
    AIT_REG_B   DADC_R_NF2_B1_COEF_LOW[2];                              // 0x83
    AIT_REG_B   DADC_R_NF2_B1_COEF_HIG;                                 // 0x85
    AIT_REG_B   DADC_R_NF2_B2_COEF_LOW[2];                              // 0x86
    AIT_REG_B   DADC_R_NF2_B2_COEF_HIG;                                 // 0x88

    AIT_REG_B   DADC_R_NF2_A1_COEF_LOW[2];                              // 0x89
    AIT_REG_B   DADC_R_NF2_A1_COEF_HIG;                                 // 0x8B
    AIT_REG_B   DADC_R_NF2_A2_COEF_LOW[2];                              // 0x8C
    AIT_REG_B   DADC_R_NF2_A2_COEF_HIG;                                 // 0x8E
    AIT_REG_B    _x8F;

    AIT_REG_B   DADC_R_NF3_B0_COEF_LOW[2];                              // 0x90
    AIT_REG_B   DADC_R_NF3_B0_COEF_HIG;                                 // 0x92
    AIT_REG_B   DADC_R_NF3_B1_COEF_LOW[2];                              // 0x93
    AIT_REG_B   DADC_R_NF3_B1_COEF_HIG;                                 // 0x95
    AIT_REG_B   DADC_R_NF3_B2_COEF_LOW[2];                              // 0x96
    AIT_REG_B   DADC_R_NF3_B2_COEF_HIG;                                 // 0x98

    AIT_REG_B   DADC_R_NF3_A1_COEF_LOW[2];                              // 0x99
    AIT_REG_B   DADC_R_NF3_A1_COEF_HIG;                                 // 0x9B
    AIT_REG_B   DADC_R_NF3_A2_COEF_LOW[2];                              // 0x9C
    AIT_REG_B   DADC_R_NF3_A2_COEF_HIG;                                 // 0x9E
    AIT_REG_B    _x9F;

    AIT_REG_B   DADC_R_NF4_B0_COEF_LOW[2];                              // 0xA0
    AIT_REG_B   DADC_R_NF4_B0_COEF_HIG;                                 // 0xA2
    AIT_REG_B   DADC_R_NF4_B1_COEF_LOW[2];                              // 0xA3
    AIT_REG_B   DADC_R_NF4_B1_COEF_HIG;                                 // 0xA5
    AIT_REG_B   DADC_R_NF4_B2_COEF_LOW[2];                              // 0xA6
    AIT_REG_B   DADC_R_NF4_B2_COEF_HIG;                                 // 0xA8

    AIT_REG_B   DADC_R_NF4_A1_COEF_LOW[2];                              // 0xA9
    AIT_REG_B   DADC_R_NF4_A1_COEF_HIG;                                 // 0xAB
    AIT_REG_B   DADC_R_NF4_A2_COEF_LOW[2];                              // 0xAC
    AIT_REG_B   DADC_R_NF4_A2_COEF_HIG;                                 // 0xAE
    AIT_REG_B    _xAF;
     
} AITS_DADC_EXT,*AITPS_DADC_EXT;

#pragma pack()

#endif //(CHIP == MCR_V2)

#define I2S_REG_OFFSET(reg) offsetof(AITS_I2S,reg)
#define AFE_REG_OFFSET(reg) offsetof(AITS_AFE,reg)

////////////////////////////////////
// Register definition
//

#define FIFO_CPU_INT_EN_OFFSET 0
//#define CPU_INT_EN 

//aud_writeb(base)	__raw_writeb(base+reg)

/* Register access macros */
#define i2sreg_readb(base, reg)		    0//__raw_readb(base + I2S_REG_OFFSET(reg))
#define i2sreg_writeb(base, reg, value)	//__raw_writeb((value), base + I2S_REG_OFFSET(reg))

#define afereg_readb(base, reg)		    __raw_readb(base + AFE_REG_OFFSET(reg))
#define afereg_writeb(base, reg, value)	__raw_writeb((value), base + AFE_REG_OFFSET(reg))
#define afereg_readw(base, reg)		    __raw_readw(base + AFE_REG_OFFSET(reg))
#define afereg_writew(base, reg, value)	__raw_writew((value), base + AFE_REG_OFFSET(reg))

//#ifdef CHIP==MCR_V2
//#define AFE_FIFO_REG_OFFSET(reg) offsetof(AITS_AFE.AFE_FIFO.DAC_MP,reg)
#if (CHIP==MCR_V2)
#if (SYSTEM_CORE_ID==CHIP_CORE_ID_MCR_V2_MP)
#define afe_dac_fifo_reg(base, reg)   ( ((volatile AITS_AFE*)base)->AFE_FIFO.DAC_MP.reg )
#define afe_adc_fifo_reg(base, reg)   ( ((volatile AITS_AFE*)base)->ADC_FIFO.MP.reg )
#elif (SYSTEM_CORE_ID==CHIP_CORE_ID_MCR_V2_SHT)
#define afe_dac_fifo_reg(base, reg)   ( ((volatile AITS_AFE*)base)->AFE_FIFO.SHT.reg )
#define afe_adc_fifo_reg(base, reg)   ( ((volatile AITS_AFE*)base)->AFE_FIFO.SHT.reg )
#endif
#endif 

//I2S
#if (CHIP==MCR_V2)
#define i2s_reg(base, reg)   ( ((volatile AITS_I2S*)base)->reg)
#define i2s_fifo_status(base, reg)   ( ((volatile AITS_I2S*)base)->I2S_FIFO_SR.MP.reg)
#define i2s_fifo_reg(base, reg)   ( ((volatile AITS_I2S*)base)->I2S_FIFO.MP.reg)
#endif

//#define afe_dac_fifo_reg_writeb(base, reg, value)	( ( ((volatile AITS_AFE*)base)->AFE_FIFO.DAC_MP.reg ) = value )//__raw_writeb((value), base + AFE_FIFO_REG_OFFSET(reg))
//#endif 

/// @}

#endif // _MMP_REG_AUDIO_H_
