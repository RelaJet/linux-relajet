//==============================================================================
//
//  File        : aitu_ringbuf.h
//  Description : Header File of Generic Ring buffer Utility
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================

#ifndef _CPUB_RINGBUF_H_
#define _CPUB_RINGBUF_H_

/** @addtogroup AIT_UTILITY
 *  @{
 */

//==============================================================================
//
//                              INCLUDE FILES
//
//==============================================================================
#include <mach/mmpf_typedef.h>


//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================

/*
 * Ring buffer error codes
 */
#define RINGBUF_SUCCESS         (0)
#define RINGBUF_ERR_NULL_HANDLE (-1)
#define RINGBUF_ERR_OVERFLOW    (-2)
#define RINGBUF_ERR_UNDERFLOW   (-3)

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

//==============================================================================
//
//                              STRUCTURE
//
//==============================================================================

/*
 * Ring buffer pointer
 */
typedef struct {
    MMP_ULONG   rd;         ///< read pointer
    MMP_ULONG   wr;         ///< write pointer
    MMP_ULONG   rd_wrap;    ///< read pointer wrap count
    MMP_ULONG   wr_wrap;    ///< write pointer wrap count
} AUTL_RINGBUF_PTR;

/*
 * Ring buffer handler
 */
typedef struct _AUTL_RINGBUF{
    void            *buf;   ///< ring buffer address
    void            *buf_phy ; // phy buffer for CPU-b
    MMP_ULONG        size;  ///< ring buffer size
    AUTL_RINGBUF_PTR ptr;   ///< ring buffer pointers
} AUTL_RINGBUF;


typedef enum _RINGBUF_ID
{
	BUFID_IN = 0 ,
	BUFID_OUT ,
	AES_IN,
	AES_OUT,
	BUFID1_IN ,
	BUFID1_OUT ,
	BUFID_MAX
} RINGBUF_ID ;

typedef struct ring_bufctl_s
{
    void *dma_inbuf_virt_addr , *dma_outbuf_virt_addr ;
    dma_addr_t dma_inbuf_phy_addr, dma_outbuf_phy_addr;
    int  dma_inbuf_buffer_size,dma_outbuf_buffer_size  ;
    AUTL_RINGBUF  in_buf_h,out_buf_h ;    
} ring_bufctl_t ;

//==============================================================================
//
//                              FUNCTION PROTOTYPE
//
//==============================================================================

int AUTL_RingBuf_Init(AUTL_RINGBUF *ring, char *buf, char *buf_phy,MMP_ULONG size);
int AUTL_RingBuf_Fork(AUTL_RINGBUF *src, AUTL_RINGBUF *dst);
int AUTL_RingBuf_StrictCommitRead(AUTL_RINGBUF *ring, MMP_ULONG size);
int AUTL_RingBuf_StrictCommitWrite(AUTL_RINGBUF *ring, MMP_ULONG size);
int AUTL_RingBuf_CommitRead(AUTL_RINGBUF *ring, MMP_ULONG size);
int AUTL_RingBuf_CommitWrite(AUTL_RINGBUF *ring, MMP_ULONG size);
int AUTL_RingBuf_SpaceAvailable(AUTL_RINGBUF *ring, MMP_ULONG *space);
int AUTL_RingBuf_DataAvailable(AUTL_RINGBUF *ring, MMP_ULONG *data_size);
MMP_BOOL AUTL_RingBuf_Empty(AUTL_RINGBUF *ring);
MMP_BOOL AUTL_RingBuf_Full(AUTL_RINGBUF *ring);
void *AUTL_RingBuf_CurWrPtr(AUTL_RINGBUF *ring,MMP_ULONG testsize);


AUTL_RINGBUF *MMPF_Audio_GetEncBufHandle(RINGBUF_ID id) ;
void MMPF_Audio_InitBufHandle(RINGBUF_ID id,AUTL_RINGBUF *handle,char *sBufAddr,char *sBufAddrPhy,MMP_ULONG ulBufSize);
char *MMPF_Audio_EncBufDataStart(RINGBUF_ID id,MMP_BOOL virt);
void MMPF_Audio_AdvanceBufReadPtr(RINGBUF_ID id,MMP_ULONG ulSize) ;
void MMPF_Audio_AdvanceBufWritePtr(RINGBUF_ID id,MMP_ULONG ulSize);
MMP_ULONG MMPF_Audio_BufDataAvailable(RINGBUF_ID id);
MMP_ULONG MMPF_Audio_BufFreeSpace(RINGBUF_ID id);
MMP_BOOL MMPF_Audio_BufFull(RINGBUF_ID id);

void MMPF_Audio_ReadBuf(RINGBUF_ID id,char *target, MMP_ULONG sample_bytes);
void MMPF_Audio_WriteBuf(RINGBUF_ID id,char *data, MMP_ULONG sample_bytes) ;
void MMPF_Audio_SetEncBufHandle(RINGBUF_ID id,AUTL_RINGBUF *handle);
void MMPF_Audio_ReadBufToUser(RINGBUF_ID id,char *user, /*char *src, */MMP_ULONG sample_bytes);
void MMPF_Audio_WriteBufFromUser(RINGBUF_ID id,char *user,  MMP_ULONG sample_bytes);

/// @}

#endif //_AITU_RINGBUF_H_
