#ifndef MMPF_AUDIO_CTL_H
#define MMPF_AUDIO_CTL_H

#include "config_fw.h"
#include "mmpf_pll.h"

#define AUDIO_NO_ERROR      0x00
#define AUDIO_PARAM_ERROR   0x01
#define AUDIO_NOT_SUPPORT   0x02

typedef enum _MMPF_AUDIO_LINEIN_CHANNEL
{
    MMPF_AUDIO_LINEIN_DUAL = 0x0,		///< dual channel line in
    MMPF_AUDIO_LINEIN_R = 0x1,			///< Right channel line in
    MMPF_AUDIO_LINEIN_L = 0x2,			///< Left channel line in
    MMPF_AUDIO_LINEIN_SWAP = 0x3		///< dual channel, L and R swap
} MMPF_AUDIO_LINEIN_CHANNEL;

//Audio path
#define AUDIO_IN_AFE_SING           0x01 ///< audio in using internal adc: (AUXL/AUXR)
#define AUDIO_IN_AFE_DIFF           0x02 ///< audio in using internal adc: (LIP/LIN/RIP/RIN)
#define AUDIO_IN_AFE_DIFF2SING      0x04 ///< audio in using internal adc: (LIP/RIP)
#define AUDIO_OUT_AFE_HP            0x00 // mcrv2 has no head-phone out 0x01 ///< audio out using internal dac: HP_OUT (LOUT/ROUT)
#define AUDIO_OUT_AFE_HP_INVT       0x02 ///< audio out using internal dac: HP_OUT Inverter (LOUT/ROUT)
#define AUDIO_OUT_AFE_LINE          0x04 ///< audio out using internal dac: LINE_OUT (LOUT2/ROUT2)
#define AUDIO_OUT_AFE_LINE_INVT     0x08 ///< audio out using internal dac: LINE_OUT Inverter (LOUT2/ROUT2)
#define AUDIO_AFE_MASK              0x0F ///< audio using internal dac

#define AUDIO_INOUT_I2S0            0x10 ///< audio in/out using i2s channel 0
#if (CHIP == P_V2)
#define AUDIO_I2S_MASK              0x10 ///< audio using i2s
#endif
#if (CHIP == MCR_V2)
#define AUDIO_INOUT_I2S1            0x20 ///< audio in/out using i2s channel 1
#define AUDIO_I2S_MASK              0x30 ///< audio using i2s
#endif
#define I2S_PATH(ch)                (1 << (ch + 4)) //0x10, 0x20


#define AUDIO_BYPASS_DISABLE 		0x00 ///< audio bypass disable
#define AUDIO_BYPASS_LL 			0x01 ///< audio bypass L-in to L-out
#define AUDIO_BYPASS_LR 			0x02 ///< audio bypass L-in to R-out
#define AUDIO_BYPASS_RL 			0x04 ///< audio bypass R-in to L-out
#define AUDIO_BYPASS_RR 			0x08 ///< audio bypass R-in to R-out

#define AUDIO_FIFO_DEPTH    512

typedef enum _MMPF_AUDIO_DATA_FLOW
{
    AFE_FIFO_TO_DAC = 0x1,  ///< AFE FIFO to AFE DAC
    ADC_TO_AFE_FIFO,        ///< AFE ADC to AFE FIFO
    ADC_TO_I2S0_FIFO,       ///< AFE ADC to I2S0 FIFO
    I2S0_FIFO_TO_SDO,       ///< I2S0 FIFO to I2S output
    SDI0_TO_AFE_FIFO,       ///< I2S0 input to AFE FIFO
    SDI_TO_I2S0_FIFO,       ///< I2S input to I2S0 FIFO
    #if (CHIP == MCR_V2)
    ADC_TO_I2S1_FIFO,       ///< AFE ADC to I2S1 FIFO
    I2S1_FIFO_TO_SDO,       ///< I2S1 FIFO to I2S output
    I2S2_FIFO_TO_SDO,       ///< I2S2 FIFO to I2S output
    SDI1_TO_AFE_FIFO,      	///< I2S1 input to AFE FIFO
    SDI_TO_I2S1_FIFO,      	///< I2S input to I2S1 FIFO
    /*++MCR_V2 MP ONLY*/
    I2S2_FULL_DUPLEX,    	///< I2S input to I2S2 FIFO
    I2S1_FULL_DUPLEX,    	///< I2S input to I2S2 FIFO
    I2S0_FULL_DUPLEX,   	///< I2S input to I2S2 FIFO
    AFE_FULL_DUPLEX,       ///< I2S input to I2S2 FIFO
    SDI_TO_I2S2_RX_FIFO,    ///< I2S input to I2S2 FIFO
    SDI_TO_I2S1_RX_FIFO,    ///< I2S input to I2S1 FIFO
    SDI_TO_I2S0_RX_FIFO,    ///< I2S input to I2S0 FIFO
    I2S2_TX_FIFO_TO_SDO,    ///< I2S output to I2S2 FIFO
    I2S1_TX_FIFO_TO_SDO,    ///< I2S output to I2S1 FIFO
    I2S0_TX_FIFO_TO_SDO,    ///< I2S output to I2S0 FIFO
    ADC_TO_AFE_RX_FIFO,     ///< ADC input to Rx FIFO
    AFE_TX_FIFO_TO_DAC,     ///< DAC output to Tx FIFO
    /*--MCR_V2 MP ONLY*/
    #endif
    MMPF_AUDIO_MAX_FLOW
} MMPF_AUDIO_DATA_FLOW;

//===================//
// Decoder           //
//===================//
#define AMR                 (0x0)       //AMR > 0 can switch to AMR demo mode
#define MP3                 (0x1)
#define AAC                 (0x2)
#define MIDI                (0x3)
#define AACPLUS             (0x4)
#define WMA                 (0x5)
#define WMAPRO              (0x7)
#define OGG                 (0x8)
#define RA                  (0x9)
#define FLAC				(0xA)
#define MP12                (0xB)
#define WAVE                (0x10)

#define AMR_MR475   0
#define AMR_MR515   1

#define AMR_OUTPUT_BUFFER_COUNT 9
#define AAC_OUTPUT_BUFFER_COUNT 6

#define AUDIO_NO_SMOOTH_OP  0x00
#define AUDIO_SMOOTH_UP     0x01
#define AUDIO_SMOOTH_DOWN   0x02

#define A8_AUDIO_STATUS_RECORD  0x8000
#define A8_AUDIO_STATUS_PLAY    0x0000
#define A8_AUDIO_STATUS_START   0x0001
#define A8_AUDIO_STATUS_PAUSE   0x0002
#define A8_AUDIO_STATUS_RESUME  0x0003
#define A8_AUDIO_STATUS_STOP    0x0004

#define MIDI_FILE_MODE      0
#define MIDI_STREAM_MODE    1

#define I2S_DUPLEX              (1)
#define AFE_DUPLEX              (2)

#define PLAY_OP_START           (0x0100)
#define PLAY_OP_STOP            (0x0200)
#define PLAY_OP_PAUSE           (0x0300)
#define PLAY_OP_RESUME          (0x0400)


typedef enum _MMPF_AUDIO_EDIT_POINT
{
    MMPF_AUDIO_EDIT_POINT_ST = 0,
    MMPF_AUDIO_EDIT_POINT_ED
} MMPF_AUDIO_EDIT_POINT;



//Audio Record Event
#define EVENT_FIFO_OVER_THRESHOLD       (0x0002)
#define EVENT_STOP_ENCODER              (0x0004)

//Audio Play Event
#define EVENT_DECODE_AACPLUS            (0x0001)
#define EVENT_DECODE_WMA                (0x0002)
#define EVENT_DECODE_WAV                (0x0008)
#define EVENT_DECODE_AMR                (0x0010)
#define EVENT_DECODE_MP3                (0x0020)
#define EVENT_DECODE_AAC                (0x0040)
#define EVENT_DECODE_MIDI               (0x0080)
#define EVENT_DECODE_OGG                (0x0400)
#define EVENT_DECODE_RA             	(0x0800)
#define EVENT_SBC_TRIGGER_INT           (0x1000)
#define EVENT_WAV_TRIGGER_INT           (0x2000)
#define EVENT_SBC_FILLBUF_INT           (0x4000)
#define EVENT_DECODE_AC3                (0x8000)
#define EVENT_DECODE_FLAC				(0x0100)

typedef enum _MMPF_AUDIO_PLAY_FORMAT
{
    MP3_PLAY_MODE = 0,
    MIDI_PLAY_MODE = 1,
    AMR_PLAY_MODE = 2,
    WMA_PLAY_MODE = 3,
    AAC_PLAY_MODE = 4,
    AACPLUS_PLAY_MODE = 7,
    OGG_PLAY_MODE = 9,
	VIDEO_AMR_PLAY_MODE = 10,	
	VIDEO_AAC_PLAY_MODE = 11,
	RA_PLAY_MODE = 12,
	WAV_PLAY_MODE = 13,
	VIDEO_MP3_PLAY_MODE = 14,
	AC3_PLAY_MODE = 14,
	VIDEO_AC3_PLAY_MODE = 16
} MMPF_AUDIO_PLAY_FORMAT;

#define MP3_PLAY_MODE           (0)
#define MIDI_PLAY_MODE          (1)
#define AMR_PLAY_MODE           (2)
#define WMA_PLAY_MODE           (3)
#define AAC_PLAY_MODE           (4)
#define AACPLUS_PLAY_MODE       (7)
#define OGG_PLAY_MODE           (9)
#define VIDEO_AMR_PLAY_MODE     (10)
#define VIDEO_AAC_PLAY_MODE     (11)
#define RA_PLAY_MODE            (12)
#define WAV_PLAY_MODE           (13)
#define VIDEO_MP3_PLAY_MODE     (14)
#define AC3_PLAY_MODE           (15)
#define VIDEO_AC3_PLAY_MODE     (16)
#define VIDEO_RA_PLAY_MODE		(17)
#define VIDEO_WMA_PLAY_MODE     (18)
#define VIDEO_WAV_PLAY_MODE     (19)
#define FLAC_PLAY_MODE			(20)

#define VIDEO_AMR_REC_MODE          (0)
#define VIDEO_AAC_REC_MODE          (1)
#define AMR_REC_MODE                (2)
#define AAC_REC_MODE                (3)
#define MP3_REC_MODE                (4)
#define WAV_REC_MODE                (5)
#define VIDEO_ADPCM_REC_MODE        (6)
#define VIDEO_MP3_REC_MODE          (7)
//////////// 20141006 //////////

typedef enum 
{
    I2S0_PATH_NUM,                 //  (0)//Before SRC
    I2S1_PATH_NUM,
    I2S2_PATH_NUM,                 //  (1)//Before SRC
    AFE_PATH_NUM,                  //  (2)
    AFE_OUT_PATH_NUM,                  //  (2)
    AUDIO_INPUT_PATH_NUM
}AUDIO_INPUT_PATH;
typedef enum //After SRC
{
    SRC_I2S0_PATH_NUM = AUDIO_INPUT_PATH_NUM,                 //  (3)
    SRC_I2S1_PATH_NUM,
    SRC_I2S2_PATH_NUM,                 //  (4)
    SRC_AFE_PATH_NUM,				// 5
    SRC_SKYPE_48K_TO_24K_PATH_NUM, // 6
    SRC_SKYPE_16K_TO_48K_PATH_NUM, // 7
    ALL_AUDIO_BUFFER_NUM
}AUDIO_SRC_PATH;
#define AFE_PATH         (AFE_PATH_NUM) 
#define AFE_OUT_PATH     (AFE_OUT_PATH_NUM)              
#define I2S0_PATH         (I2S0_PATH_NUM)                
#define I2S1_PATH        (I2S1_PATH_NUM)//(I2S1_PATH_NUM)   
#define I2S2_PATH        (I2S2_PATH_NUM) 
#define I2S_SRC_PATH     (SRC_I2S0_PATH_NUM)
#define I2S1_SRC_PATH    (SRC_I2S1_PATH_NUM)//(SRC_I2S1_PATH_NUM)
#define I2S2_SRC_PATH    (SRC_I2S2_PATH_NUM)   
#define AFE_SRC_PATH    (SRC_AFE_PATH_NUM) 
#define AUDIO_INTPUT_NUM (ALL_AUDIO_BUFFER_NUM)

#define AFE_OUT     0xFF//0x0003
#define AFE_IN      AFE_PATH
#define I2S_OUT     0xFE//0x0004
#define I2S_IN      I2S0_PATH
#define I2S0_IN     I2S0_PATH
#define I2S1_IN     I2S1_PATH
#define I2S2_IN     I2S2_PATH
#define I2S0_SRC    I2S_SRC_PATH
#define I2S1_SRC    I2S1_SRC_PATH
#define I2S2_SRC    I2S2_SRC_PATH
#define AFE_SRC     AFE_SRC_PATH
#define SKYPE_1st_SRC   SRC_SKYPE_16K_TO_48K_PATH_NUM
#define SKYPE_2st_SRC   SRC_SKYPE_48K_TO_24K_PATH_NUM
#define AUDIO_MAX_FILE_SIZE  (256) 

//////////////////////////////////// 

// Number of I2S channels
#define I2S_CH_INVALID              (0xFF)

#define AUDIO_MAX_FILE_SIZE  (256)

#define MIXER_FIFO_WRITE_THRESHOLD    	128
#define MP3_I2S_FIFO_WRITE_THRESHOLD    128
#define WMA_I2S_FIFO_WRITE_THRESHOLD    128
#define OGG_I2S_FIFO_WRITE_THRESHOLD    128
#define	AAC_I2S_FIFO_WRITE_THRESHOLD	128
#define AC3_I2S_FIFO_WRITE_THRESHOLD    128
#define AMR_I2S_FIFO_WRITE_THRESHOLD    80
#define RA_I2S_FIFO_WRITE_THRESHOLD     128
#define WAV_I2S_FIFO_WRITE_THRESHOLD    64
#define FLAC_I2S_FIFO_WRITE_THRESHOLD	128

//=========================================//
//  Audio Play Memory Mode Handshake Buffer//
//=========================================//
#define AUDIO_PLAY_R_HPTR_OFST                  (0)
#define AUDIO_PLAY_R_PTR_OFST                   (4)
#define AUDIO_PLAY_FINISH_SEEK_W                (8)
#define AUDIO_PLAY_START_SEEK_W                 (10)
#define AUDIO_PLAY_FILE_SEEK_OFFSET_L_W         (12)
#define AUDIO_PLAY_FILE_SEEK_OFFSET_H_W         (14)
#define AUDIO_PLAY_W_HPTR_OFST                  (16)
#define AUDIO_PLAY_W_PTR_OFST                   (20)

//===========================================//
//  Audio Record Memory Mode Handshake Buffer//
//===========================================//
#define AUDIO_REC_WRITE_HIGH_PTR_OFFSET_D    (0)
#define AUDIO_REC_WRITE_PTR_OFFSET_D         (4)
#define AUDIO_REC_READ_HIGH_PTR_OFFSET_D     (8)
#define AUDIO_REC_READ_PTR_OFFSET_D          (12)

//===========================================//
//          ADC error saturation prevention  //
//===========================================//

#if (CHIP == P_V2)
#define ADC_ESP_SUPPORT                     (0)
#endif
#if (CHIP == MCR_V2)
#define ADC_ESP_SUPPORT                     (0) // set to 0 only, fix by HW
#endif
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define ADC_ESP_SUPPORT                     (0) // set to 0 only, fix by HW
#endif


#if (ADC_ESP_SUPPORT)
#define MAX_DELTA                           (32768)
#define AUD_SAMPLE_ESP(in, last, delta)     {                               \
                                                delta = in - last;          \
                                			    if (delta < -MAX_DELTA)     \
                                			        last = MAX_DELTA - 1;   \
                                			    else if (delta > MAX_DELTA) \
                                			        last = -MAX_DELTA;      \
                                			    else                        \
                                			        last = in;              \
                                            }
#endif

//===========================================//
//  PMP Project CallBack funtion 			 //
//===========================================//
typedef void AudioPlayerCallBackFunc(void *VidContext, MMP_ULONG flag1, MMP_ULONG flag2);
#if (GAPLESS_EN == 1)
typedef void GaplessNextFileCallBackFunc(MMP_ULONG handle);
#endif
#define         AUDIO_EVENT_EOF     (0x00000001)

typedef struct _AudioCallBackInfo {
	AudioPlayerCallBackFunc *callBackFunction;
	void     *context;
} AudioCallBackInfo;

typedef struct mp4AudioSpecificConfig
{
    /* Audio Specific Info */
    unsigned char objectTypeIndex;
    unsigned char samplingFrequencyIndex;
    unsigned long samplingFrequency;
    unsigned char channelsConfiguration;

    /* GA Specific Info */
    unsigned char frameLengthFlag;
    unsigned short sample256_time;

} mp4AudioSpecificConfig;

typedef struct {
    short rd_index;
    short wr_index;
    unsigned int total_rd;
    unsigned int total_wr;
}AUDIO_DEC_OUTBUF_HANDLE;

typedef struct {
    short rd_index;
    short wr_index;
    unsigned int total_rd;
    unsigned int total_wr;
}AUDIO_ENC_INBUF_HANDLE;

typedef struct _MMPF_AUDIO_RINGBUF {
    MMP_ULONG ulBufStart;
    MMP_ULONG ulBufSize;
    MMP_ULONG ulReadPtr;
    MMP_ULONG ulWritePtr;
    MMP_ULONG ulReadPtrHigh;
    MMP_ULONG ulWritePtrHigh;
} MMPF_AUDIO_RINGBUF;

typedef struct _MMPF_AUDIO_INTIME_HANDLE {
    MMP_ULONG ulTimeCnt;           // in ms
    MMP_ULONG ulSampleCnt;         // modular samplerate
} MMPF_AUDIO_INTIME_HANDLE;

typedef enum {
    MMPF_AUDIO_GAPLESS_SEEK     = 0x0001,
    MMPF_AUDIO_GAPLESS_PAUSE    = 0x0002,
    MMPF_AUDIO_GAPLESS_STOP     = 0x0004,
    MMPF_AUDIO_GAPLESS_OP_ALL   = 0x0007
} AUDIO_GAPLESS_OP;

extern void MMPF_PlayMidiDec(void);
extern void MMPF_StopMidiDec(void);
extern void MMPF_PauseMidiDec(void);
extern void MMPF_ResumeMidiDec(void);
extern void MMPF_InitMidiDec(unsigned short mode);
extern void MMPF_WriteMidiStream(unsigned char count);
extern int MMPF_GetMidiFileInfo(unsigned short *total_time);
extern unsigned int MMPF_GetMidiCurTime(void);
MMP_ERR     MMPF_Audio_SetAmrEncodeMode(unsigned short mode);
MMP_ERR     MMPF_Audio_SetStreamLength(unsigned int stream_length);
MMP_ERR     MMPF_SetAudioPlayReadPtr(void);
MMP_ERR     MMPF_Audio_SetupRender(MMP_USHORT usFifoThreshold, MMP_ULONG ulSamplerate);
MMP_ERR     MMPF_Audio_EnableRender(void);
MMP_ERR     MMPF_Audio_DisableRender(void);
MMP_ERR     MMPF_Audio_EnableCapture(MMP_USHORT usFifoThreshold, MMP_ULONG ulSamplerate);
MMP_ERR     MMPF_Audio_DisableCapture(void);
MMP_ERR     MMPF_Audio_InitializeCodec(MMPF_AUDIO_DATA_FLOW path, MMP_ULONG samprate);
MMP_ERR     MMPF_SetAudioRecWritePtr(void);
MMP_ERR     MMPF_SetAudioEncWritePtr(void);
MMP_ERR     MMPF_Audio_SetSpectrumBuf(MMP_ULONG buf_addr);
MMP_ERR     MMPF_UpdateAudioPlayReadPtr(void);
MMP_ERR     MMPF_UpdateAudioPlayWritePtr(void);
MMP_ERR     MMPF_UpdateAudioRecReadPtr(void);
MMP_ERR     MMPF_Audio_SetRecIntThreshold(MMP_USHORT threshold);
MMP_ERR     MMPF_Audio_SetPlayIntThreshold(MMP_USHORT threshold);
MMP_ERR     MMPF_Audio_SetVoiceInPath(MMP_UBYTE path);
MMP_ERR     MMPF_Audio_SetVoiceOutPath(MMP_UBYTE path);
MMP_ERR     MMPF_Audio_PowerOnDAC(MMP_ULONG samplerate);
MMP_ERR	     MMPF_Audio_PowerOnDAC_Fast(MMP_ULONG samplerate);
MMP_ERR     MMPF_Audio_PowerOnADC(MMP_ULONG samplerate);

MMP_ERR     MMPF_Audio_SetADCMute(MMP_BOOL enable);
MMP_ERR MMPF_Audio_SetDACMute(void);

//MMP_ERR     MMPF_Audio_SetPLL(MMP_UBYTE ubPath, MMP_ULONG ulSamplerate);
MMP_ERR     MMPF_Audio_SetPLL(LINUX_SOC_PATH path, MMP_ULONG ulSamplerate,uint8_t reset);
MMP_ERR     MMPF_Audio_SetDACFreq(MMP_ULONG samplerate);
MMP_ERR     MMPF_Audio_SetADCFreq(MMP_ULONG samplerate);
MMP_ERR     MMPF_Audio_PowerDownDAC(MMP_BOOL bPowerDownNow);
MMP_ERR     MMPF_Audio_PowerDownADC(void);
MMP_ERR     MMPF_Audio_PowerDownCodec(void);
//MMP_ERR	    MMPF_Audio_EnableAFEClock(MMP_BOOL bEnableClk, MMP_ULONG SampleRate);
MMP_ERR	    MMPF_Audio_EnableAFEClock(MMP_BOOL bEnableClk, MMP_ULONG SampleRate,LINUX_SOC_PATH Path);
MMP_ERR     MMPF_PostPlayAudioOp(void);
#if (VMP3_P_EN)
MMP_ERR     MMPF_Audio_UpdateVMP3Time(MMP_ULONG ulTime);
#endif
#if (VAC3_P_EN)
MMP_ERR     MMPF_Audio_UpdateVAC3Time(MMP_ULONG ulTime);
#endif

MMP_ERR     MMPF_Audio_ResetAfeFifo(void);
MMP_ERR		MMPF_Audio_SetMux(MMPF_AUDIO_DATA_FLOW path, MMP_BOOL bEnable);
MMP_ERR     MMPF_Audio_SetDuplexPath(MMPF_AUDIO_DATA_FLOW path, MMP_BOOL bEnable);
MMP_ERR     MMPF_Audio_SetBypassPath(MMP_UBYTE path);
MMP_ERR  	MMPF_Audio_SetLineInChannel(MMPF_AUDIO_LINEIN_CHANNEL lineinchannel);
MMP_ERR     MMPF_Audio_SetPlayFormat(MMP_USHORT mode);
MMP_ERR     MMPF_Audio_SetEncodeFormat(MMP_USHORT mode);
MMP_ERR     MMPF_Audio_SetPlayFileName(MMP_ULONG address);
MMP_ERR  	MMPF_Audio_SetPlayBuffer(MMP_ULONG ulBufStart, MMP_ULONG ulBufSize);
MMP_ERR     MMPF_Audio_SetPlayPath(MMP_UBYTE path);
MMP_ERR		MMPF_Audio_SetPlayHandshakeBuf(MMP_ULONG ulBufStart);
MMP_ERR		MMPF_Audio_GetPlayState(MMP_USHORT* usState);
MMP_ERR     MMPF_Audio_SetDACDigitalGain(MMP_UBYTE gain);
MMP_ERR     MMPF_Audio_SetDACAnalogGain(MMP_UBYTE gain);
MMP_ERR  	MMPF_Audio_SetADCDigitalGain(MMP_UBYTE gain);
MMP_ERR     MMPF_Audio_SetADCAnalogGain(MMP_UBYTE gain, MMP_BOOL boostup);
MMP_ERR		MMPF_Audio_InitializePlayFIFO(MMP_USHORT usPath, MMP_USHORT usThreshold);
MMP_ERR 	MMPF_Audio_SetRecordHeadMuteTime(MMP_ULONG ulMilliSec);
MMP_ERR 	MMPF_Audio_SetRecordTailCutTime(MMP_ULONG ulMilliSec);
MMP_ERR 	MMPF_Audio_EnableDummyRecord(MMP_BOOL bEnable, MMP_UBYTE ubFlag);
MMP_ERR     MMPF_Audio_SetEncodeFormat(MMP_USHORT mode);
MMP_ERR     MMPF_Audio_SetEncodeFileName(MMP_ULONG address);
MMP_ERR  	MMPF_Audio_SetEncodeBuffer(MMP_ULONG ulBufStart, MMP_ULONG ulBufSize);
MMP_ERR     MMPF_Audio_SetEncodePath(MMP_UBYTE path);
MMP_ERR		MMPF_Audio_SetEncodeHandshakeBuf(MMP_ULONG ulBufStart);
MMP_ERR  	MMPF_Audio_SetEncodeLength(MMP_ULONG ulFileLimit);
MMP_ERR		MMPF_Audio_InitializeEncodeFIFO(MMP_USHORT usPath, MMP_USHORT usThreshold);
MMP_ERR 	MMPF_Audio_GetRecordTime(MMP_ULONG *ulMilliSec);
MMP_ERR     MMPF_Audio_SetRecordTimeHandler(MMP_ULONG ulTime, MMP_ULONG ulSampleCnt);

MMP_ERR  	MMPF_Audio_SetEditBuffer(MMP_ULONG ulBufStart, MMP_ULONG ulBufSize);
MMP_ERR     MMPF_Audio_SetEditFileName(MMP_ULONG address);
MMP_ERR 	MMPF_Audio_SetEditPoint(MMPF_AUDIO_EDIT_POINT point);
MMP_ERR  	MMPF_Audio_TrnasferPlayDataToEditBuf(MMP_ULONG ulStartOffset, MMP_ULONG ulSize);
MMP_ERR  	MMPF_Audio_TrnasferEditDataToCard(void);
MMP_ERR  	MMPF_Audio_CloseEditFile(void);

MMP_ERR     MMPF_AUDIO_UpdateRecWrPtr(MMP_ULONG ulSize);
MMP_ERR     MMPF_AUDIO_SetRecWrPtrWrap(MMP_ULONG ulWrap);
MMP_ERR     MMPF_AUDIO_GetRecWrPtr(MMP_ULONG *ulPtr, MMP_ULONG *ulWrap);

#if (GAPLESS_EN == 1)
MMP_ERR     MMPF_Audio_SetGaplessEnable(MMP_BOOL bEnable, GaplessNextFileCallBackFunc *cb, MMP_ULONG param);
#endif
MMP_ERR     MMPF_Audio_GaplessTriggerCB(void);
MMP_ERR     MMPF_Audio_SetGraphicEQEnable(MMP_BOOL enable);
MMP_ERR     MMPF_Audio_SetGraphicEQBand(MMP_SHORT usFreq, MMP_SHORT usQrange, MMP_SHORT usGain);

MMP_ERR     MMPF_Audio_DummyAudio1CodeStart(void);
MMP_ERR     MMPF_Audio_DummyAudio2CodeStart(void);


#endif
