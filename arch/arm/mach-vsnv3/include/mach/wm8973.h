#ifndef _WM8973_H
#define _WM8973_H

//nclude "ait_config.h"
//#include <mach/mmp_err.h>
#include <mach/mmpf_typedef.h>

#define	I2C_PORT_MODE		(0x80000050)
#define	I2C_PORT_DATA		(0x80000051)


#define I2C_SCL             (0x1<<1)
#define I2C_SDA             (0x1<<2)

#define I2C_DELAY           100

#define I2C_HIGH            1
#define I2C_LOW             0

#define I2C_INPUT           0x0
#define I2C_OUTPUT          0x1

#define I2C_SET_OUT         0x0
#define I2C_CLEAR_OUT       0x1

#define WM8973_L_INPUT_VOLUME                       0x00//0xc3 for 0dB and each step 0.5dB
#define WM8973_R_INPUT_VOLUME                       0x01
#define WM8973_LOUT1_VOLUME                         0x02
#define WM8973_RLOU1_VOLUME                         0x03
#define WM8973_ADC_DAC_CONTROL                      0x05
#define WM8973_AUDIO_INTERFACE                      0x07
    #define BIT_CLK_INVERT                          (1<<7)
    #define ENABLE_MASTER_MODE                      (1<<6)
    #define INVERT_LRCK_POL                         (1<<4)
    #define AUDIO_DATA_LENGTH_16                    (0 << 2)
    #define AUDIO_DATA_LENGTH_20                    (1 << 2)
    #define AUDIO_DATA_LENGTH_24                    (2 << 2)
    #define AUDIO_DATA_LENGTH_32                    (3 << 2)
    #define DSP_FORMAT                              (3)
    #define I2S_FORMAT                              (2)
    #define LEFT_JUSTIFIED_FORMAT                   (1)
    #define RIGHT_JUSTIFIED_FORMAT                  (0)
#define WM8973_SAMPLE_RATE                          0x08
#define WM8973_L_DAC_VOLUME                         0x0A
#define WM8973_R_DAC_VOLUME                         0x0B
#define WM8973_BASS_CONTROL                         0x0C
#define WM8973_TREBLE_CONTROL                       0x0D
#define WM8973_RESET                                0x0F
#define WM8973_3D_CONTROL                           0x10
#define WM8973_ALC1                                 0x11
#define WM8973_ALC2                                 0x12
#define WM8973_ALC3                                 0x13
#define WM8973_NOISE_GATE                           0x14
#define WM8973_L_ADC_VOLUME                         0x15
#define WM8973_R_ADC_VOLUME                         0x16
#define WM8973_ADDITIONAL_CONTROL1                  0x17
#define WM8973_ADDITIONAL_CONTROL2                  0x18
#define WM8973_PWR_MGMT1                            0x19//for ADC                
#define WM8973_PWR_MGMT2                            0x1A//for DAC    
#define WM8973_ADDITIONAL_CONTROL3                  0x1B             
#define WM8973_ADC_INPUT_MODE                       0x1F         
#define WM8973_ADCL_SIGNAL_PATH                     0x20           
#define WM8973_ADCR_SIGNAL_PATH                     0x21           
#define WM8973_L_OUT_MIX1                           0x22     
#define WM8973_L_OUT_MIX2                           0x23     
#define WM8973_R_OUT_MIX1                           0x24     
#define WM8973_R_OUT_MIX2                           0x25     
#define WM8973_MONO_OUT_MIX1                        0x26        
#define WM8973_MONO_OUT_MIX2                        0x27        
#define WM8973_LOUT2_VOLUME                         0x28      
#define WM8973_ROUT2_VOLUME                         0x29       
#define WM8973_MONOOUT_VOLUME                       0x2A      

//#define WM8973_I2C_DEV_ADDR						0x1B //CSB High
#define WM8973_I2C_DEV_ADDR						0x1A //CSB Low

#define	AUDIO_INIT_MICIN 	0x01   ///<Initialize the CODEC mic input
#define	AUDIO_INIT_LINEIN	0x02  ///<Initialize the CODEC line input
#define	AUDIO_INIT_IN		0x03      ///<Initialize the CODEC both line in and mic in
#define	AUDIO_INIT_OUT		0x04     ///<Initialize the CODEC output
#define	AUDIO_INIT_CUS		0x08     ///<Initialize the CODEC custom IO, reserved for special audio I/O
#define	AUDIO_UNINIT_MICIN	0x10 ///<Uninitialize the CODEC mic input
#define	AUDIO_UNINIT_LINEIN	0x20///<Uninitialize the CODEC line input
#define	AUDIO_UNINIT_IN		0x30    ///<Uninitialize the CODEC both line in and mic in
#define	AUDIO_UNINIT_OUT	0x40   ///<Uninitialize the CODEC output
#define	AUDIO_UNINIT_CUS	0x80    ///<Uninitialize the CODEC custom IO, reserved for special audio I/O
#define	AUDIO_INIT_OUT_LINEIN	0x100///<Initialize the CODEC output and line input

typedef enum _AUDIO_EXTCODEC_PATH
{
    AUDIO_EXT_SPK_OUT = 0,     ///< speaker output path
    AUDIO_EXT_HP_OUT,          ///< headphone output path
    AUDIO_EXT_MIC_IN,          ///< mic in path
    AUDIO_EXT_AUX_IN,          ///< AUX in path
    AUDIO_EXT_MIC_BYPASS_SPK,  ///< mic in bypass to speaker out
    AUDIO_EXT_MIC_BYPASS_HP,   ///< mic in bypass to headphone out
    AUDIO_EXT_AUX_BYPASS_SPK,  ///< AUX in bypass to speaker out
    AUDIO_EXT_AUX_BYPASS_HP,   ///< AUX in bypass to headphone out
    AUDIO_EXT_MIC_IN_SPK_OUT,  ///< mic in and speaker out
    AUDIO_EXT_MIC_IN_HP_OUT,   ///< mic in and headphone out
    AUDIO_EXT_AUX_IN_SPK_OUT,  ///< AUX in and speaker out
    AUDIO_EXT_AUX_IN_HP_OUT,   ///< AUX in and headphone out
    AUDIO_EXT_PATH_MAX
} AUDIO_EXTCODEC_PATH;

extern void	InitExtDac(void);
extern void	PowerUpExtDac(void);
extern void	PowerDownExtDac(void);
extern void	InitExtMicIn(void);
extern void	PowerUpExtMicIn(void);
extern void	PowerDownExtMicIn(void);
extern void	InitExtLineIn(void);
extern void	PowerUpExtLineIn(void);
extern void	PowerDownExtLineIn(void);
extern void PowerUpExtLineInDac(void);
extern void	InitExtLineInDac(void);
extern MMP_BOOL	audioInitializer(MMP_USHORT cmd);

extern MMP_ERR MMPC_AudioExtCodec_SetPath(MMP_ULONG path);
extern MMP_ERR MMPC_AudioExtCodec_Init(MMP_UBYTE I2Smode, MMP_ULONG samplerate, MMP_ULONG ulFixedMclkFreq);
extern MMP_ERR MMPC_AudioExtCodec_Uninit(void);
extern MMP_ERR MMPC_AudioExtCodec_SetSampleRate(MMP_ULONG ulSamplerate);
extern MMP_ERR MMPC_AudioExtCodec_SetMute(MMP_BOOL bMute);
extern MMP_ERR MMPC_AudioExtCodec_SetVolume(MMP_ULONG level);
#endif