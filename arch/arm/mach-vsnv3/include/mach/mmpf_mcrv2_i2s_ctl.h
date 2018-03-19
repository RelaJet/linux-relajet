#ifndef	MMPF_I2S_CTL_H
#define	MMPF_I2S_CTL_H

typedef enum I2C_CH_
{ 
	I2S_CH0=0,
	I2S_CH1,
	I2S_CH2
}I2C_CH;

typedef enum _MMPF_AUDIO_I2S_FORMAT
{
    MMPF_AUDIO_I2S_MASTER = 0,      ///< master standard mode, 16bits
    MMPF_AUDIO_I2S_SLAVE            ///< slave i2s mode, 20bits
} MMPF_AUDIO_I2S_FORMAT;

typedef enum _MMPF_AUDIO_I2S_MCLK_MODE
{
    MMPF_AUDIO_I2S_MCLK_MODE_NONE = 0,
    MMPF_AUDIO_I2S_128FS_MODE,              ///< 128*fs clock output
    MMPF_AUDIO_I2S_192FS_MODE,              ///< 192*fs clock output
    MMPF_AUDIO_I2S_256FS_MODE,              ///< 256*fs clock output
    MMPF_AUDIO_I2S_384FS_MODE,              ///< 384*fs clock output  
    #if (CHIP == MCR_V2)
    MMPF_AUDIO_I2S_512FS_MODE,              ///< 512*fs clock output
    MMPF_AUDIO_I2S_768FS_MODE,              ///< 768*fs clock output
    MMPF_AUDIO_I2S_1024FS_MODE,             ///< 1024*fs clock output
    MMPF_AUDIO_I2S_1536FS_MODE,             ///< 1536*fs clock output
    #endif
    MMPF_AUDIO_I2S_FIX_256FS_MODE,  ///< fixed 256*fs clock output
    MMPF_AUDIO_I2S_USB_MODE,        ///< fixed clock output
    MMPF_AUDIO_I2S_MAX_MCLK_MODE
} MMPF_AUDIO_I2S_MCLK_MODE;

extern void MMPF_I2S_PADSET(MMP_UBYTE ch, MMP_BOOL pad);
extern void MMPF_I2S_SetMclkMode(MMP_UBYTE ch, MMP_ULONG mclk_mode);
extern void MMPF_I2S_SetUsbModeFreq(MMP_UBYTE ch, MMP_ULONG mclk_freq);
extern void MMPF_I2S_GetMclkMode(MMP_UBYTE ch, MMPF_AUDIO_I2S_MCLK_MODE *mclk_mode);
extern void MMPF_I2S_SetMclkFreq(MMP_UBYTE ch, MMP_ULONG fs);
extern void MMPF_I2S_EnableDataOutput(MMP_UBYTE ch, MMP_BOOL bEnable);
#endif
