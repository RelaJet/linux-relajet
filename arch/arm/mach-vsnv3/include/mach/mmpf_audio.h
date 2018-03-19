#ifndef MMPF_AUDIO_H
#define MMPF_AUDIO_H

/** @addtogroup MMPF_AUDIO
@{
*/

typedef enum _MMPF_AUDIO_DATAPATH
{
    MMPF_AUDIO_DATAPATH_MEM = 0,///< memory mode
    MMPF_AUDIO_DATAPATH_CARD,   ///< card mode
    MMPF_AUDIO_DATAPATH_MMP_MEM ///< MMP memory mode
} MMPF_AUDIO_DATAPATH;

extern void MMPF_SetAMRDecodeOp(MMP_USHORT op);
extern void MMPF_SetAMREncodeOp(MMP_USHORT op);
extern void MMPF_SetMP3DecodeOp(MMP_USHORT op, MMP_ULONG *parameter);
extern void MMPF_SetAACPLUSDecodeOp(MMP_USHORT op);
extern void MMPF_SetFLACDecodeOp(MMP_USHORT op);
extern void MMPF_SetAACEncodeOp(MMP_USHORT op);
extern void MMPF_SetMP3EncodeOp(MMP_USHORT op);
extern void MMPF_SetWAVEncodeOp(MMP_USHORT op);
extern void MMPF_SetWMADecodeOp(MMP_USHORT op,MMP_ULONG *parameter);
extern void MMPF_SetWMAProDecodeOp(MMP_USHORT op,MMP_ULONG *parameter);
extern void MMPF_SetRADecodeOp(MMP_USHORT op,MMP_ULONG *parameter);
extern void MMPF_SetMIDIDecodeOp(MMP_USHORT op);
extern void MMPF_SetPCMDecodeOp(MMP_USHORT op);
extern void MMPF_SetOGGDecodeOp(MMP_USHORT op,MMP_ULONG *parameter);
extern void MMPF_SetWAVDecodeOp(MMP_USHORT op, MMP_ULONG *parameter);
extern void TestDram(void);
extern void MMPF_StartPcmLoop(void);


#endif

/// @}
