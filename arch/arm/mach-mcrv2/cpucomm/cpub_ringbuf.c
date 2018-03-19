//==============================================================================
//
//  File        : aitu_ringbuf.c
//  Description : Generic Ring buffer Utility
//  Author      : Alterman
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
#include <linux/export.h>
#include <asm/uaccess.h>
#include <mach/os_wrap.h>
#include <mach/cpucomm/cpucomm_if.h>
#include "cpub_ringbuf.h"

static AUTL_RINGBUF     *pAudBufHandle[BUFID_MAX];

//==============================================================================
//
//                              LOCAL FUNCTIONS
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : _Advance_ReadPtr
//------------------------------------------------------------------------------
/** 
    @brief  Advance the read pointer of ring buffer. The wrap count may
            increase 1 if wrapping.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return none
*/
static void _Advance_ReadPtr(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    CPU_LOCK_INIT();
    //pr_info("adv.rd0 : (0x%08x,0x%08x),size=%d\n",ring->buf + ring->ptr.rd ,ring->buf_phy + ring->ptr.rd , size);
    CPU_LOCK();
    ring->ptr.rd += size;
    if (ring->ptr.rd >= ring->size) {
        ring->ptr.rd -= ring->size;
        ring->ptr.rd_wrap++;
    }
    CPU_UNLOCK();
    //pr_info("adv.rd1 : (0x%08x,0x%08x),size=%d\n",ring->buf + ring->ptr.rd ,ring->buf_phy + ring->ptr.rd , size);
}
//------------------------------------------------------------------------------
//  Function    : _Advance_WritePtr
//------------------------------------------------------------------------------
/** 
    @brief  Advance the write pointer of ring buffer. The wrap count may
            increase 1 if wrapping.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return none
*/
static void _Advance_WritePtr(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    CPU_LOCK_INIT();

    CPU_LOCK();
    ring->ptr.wr += size;
    if (ring->ptr.wr >= ring->size) {
        ring->ptr.wr -= ring->size;
        ring->ptr.wr_wrap++;
    }
    CPU_UNLOCK();
}
//------------------------------------------------------------------------------
//  Function    : _Copy_CurrentPtr
//------------------------------------------------------------------------------
/** 
    @brief  Get all of current pointers of ring buffer.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return none
*/
static void _Copy_CurrentPtr(AUTL_RINGBUF *ring, AUTL_RINGBUF_PTR *ptr)
{
    CPU_LOCK_INIT();

    /* Copy all of pointers */
    CPU_LOCK();
    *ptr = ring->ptr;
    CPU_UNLOCK();
}

//==============================================================================
//
//                              PUBLIC FUNCTIONS
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_Init
//------------------------------------------------------------------------------
/** 
    @brief  Initialize the ring buffer with the specified buffer address, buffer
            size, and user data. Also, reset all read/write pointers to 0.
    @param[in] ring : ring buffer handler
    @param[in] buf  : buffer address
    @param[in] size : buffer size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_Init(AUTL_RINGBUF *ring, char *buf,char *buf_phy, MMP_ULONG size)
{
    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    ring->buf       = buf;
    ring->buf_phy   = buf_phy ;
    ring->size      = size;

    /* Reset ring buffer pointers */
    ring->ptr.rd        = 0;
    ring->ptr.wr        = 0;
    ring->ptr.rd_wrap   = 0;
    ring->ptr.wr_wrap   = 0;
    pr_info("ringbuf_s : 0x%08x,size:%d\n",buf,size);
    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_Init);
//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_Fork
//------------------------------------------------------------------------------
/** 
    @brief  Fork a new ring buffer handler from exist one.
    @param[in] src  : one existed ring buffer
    @param[in] dst  : ring buffer to be forked
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_Fork(AUTL_RINGBUF *src, AUTL_RINGBUF *dst)
{
    if (!src || !dst)
        return RINGBUF_ERR_NULL_HANDLE;
    dst->buf_phy = dst->buf_phy ;
    dst->buf   = src->buf;
    dst->size  = src->size;

    /* Copy current ring buffer pointers */
    _Copy_CurrentPtr(src, &(dst->ptr));

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_Fork);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_StrictCommitRead
//------------------------------------------------------------------------------
/** 
    @brief  Advance the read pointer of ring buffer. The wrap count may
            increase 1 if wrapping. Before advancing, this function will check
            if size is reasonable.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_StrictCommitRead(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    MMP_ULONG data_size;

    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    AUTL_RingBuf_DataAvailable(ring, &data_size);
    if (size > data_size)
        return RINGBUF_ERR_UNDERFLOW;

    _Advance_ReadPtr(ring, size);

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_StrictCommitRead);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_StrictCommitWrite
//------------------------------------------------------------------------------
/** 
    @brief  Advance the write pointer of ring buffer. The wrap count may
            increase 1 if wrapping. Before advancing, this function will check
            if size is reasonable.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_StrictCommitWrite(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    MMP_ULONG space;

    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    AUTL_RingBuf_SpaceAvailable(ring, &space);
    if (size > space)
        return RINGBUF_ERR_OVERFLOW;

    _Advance_WritePtr(ring, size);

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_StrictCommitWrite);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_CommitRead
//------------------------------------------------------------------------------
/** 
    @brief  Advance the read pointer of ring buffer. The wrap count may
            increase 1 if wrapping. If you have already make sure the size is
            reasonable, you can use this function, instread of the strict one.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_CommitRead(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    _Advance_ReadPtr(ring, size);

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_CommitRead);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_CommitWrite
//------------------------------------------------------------------------------
/** 
    @brief  Advance the write pointer of ring buffer. The wrap count may
            increase 1 if wrapping. If you have already make sure the size is
            reasonable, you can use this function, instread of the strict one.
    @param[in] ring : ring buffer handler
    @param[in] size : advance size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_CommitWrite(AUTL_RINGBUF *ring, MMP_ULONG size)
{
    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    _Advance_WritePtr(ring, size);

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_CommitWrite);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_SpaceAvailable
//------------------------------------------------------------------------------
/** 
    @brief  Get how many free space available in the ring buffer.
    @param[in] ring   : ring buffer handler
    @param[out] space : return the available(unwritten) space
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_SpaceAvailable(AUTL_RINGBUF *ring, MMP_ULONG *space)
{
    AUTL_RINGBUF_PTR ptr;

    *space = 0;

    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    /* Copy current ring buffer pointers */
    _Copy_CurrentPtr(ring, &ptr);

    if (ptr.rd_wrap == ptr.wr_wrap) {
        if (ptr.wr >= ptr.rd)
            *space = ring->size - ptr.wr + ptr.rd;
        else
            return RINGBUF_ERR_UNDERFLOW;
    }
    else if (ptr.rd_wrap < ptr.wr_wrap) {
        if (ptr.rd >= ptr.wr)
            *space = ptr.rd - ptr.wr;
        else
            return RINGBUF_ERR_OVERFLOW;
    }
    else {
        return RINGBUF_ERR_UNDERFLOW;
    }

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_SpaceAvailable);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_DataAvailable
//------------------------------------------------------------------------------
/** 
    @brief  Get how many data available in the ring buffer.
    @param[in] ring       : ring buffer handler
    @param[out] data_size : return the available(written) data size
    @return 0 for success, others for failed.
*/
int AUTL_RingBuf_DataAvailable(AUTL_RINGBUF *ring, MMP_ULONG *data_size)
{
    AUTL_RINGBUF_PTR ptr;

    *data_size = 0;

    if (!ring)
        return RINGBUF_ERR_NULL_HANDLE;

    /* Copy current ring buffer pointers */
    _Copy_CurrentPtr(ring, &ptr);

    if (ptr.rd_wrap == ptr.wr_wrap) {
        if (ptr.wr >= ptr.rd)
            *data_size = ptr.wr - ptr.rd;
        else
            return RINGBUF_ERR_UNDERFLOW;
    }
    else if (ptr.rd_wrap < ptr.wr_wrap) {
        if (ptr.rd >= ptr.wr)
            *data_size = ring->size - ptr.rd + ptr.wr;
        else
            return RINGBUF_ERR_OVERFLOW;
    }
    else {
        return RINGBUF_ERR_UNDERFLOW;
    }

    return RINGBUF_SUCCESS;
}
EXPORT_SYMBOL(AUTL_RingBuf_DataAvailable);



void *AUTL_RingBuf_CurWrPtr(AUTL_RINGBUF *ring,MMP_ULONG testsize)
{
    int ret ;
    MMP_ULONG space ;
    AUTL_RINGBUF_PTR ptr;
    ret = AUTL_RingBuf_SpaceAvailable(ring,&space) ;
    pr_info("ring : 0x%08x,in size : %d,space : %d,ret:%d\n",(u32)ring,testsize,space,ret);
    if(testsize > space) {
        return 0 ;
    }
    _Copy_CurrentPtr(ring, &ptr);
    
    return (void *)(ring->buf + ptr.wr );
}
EXPORT_SYMBOL(AUTL_RingBuf_CurWrPtr);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_Empty
//------------------------------------------------------------------------------
/** 
    @brief  Check if ring buffer empty.
    @param[in] ring : ring buffer handler
    @return MMP_TRUE if ring buffer empty, MMP_FALSE for non-empty.
*/
MMP_BOOL AUTL_RingBuf_Empty(AUTL_RINGBUF *ring)
{
    AUTL_RINGBUF_PTR ptr;

    if (!ring)
        return MMP_FALSE;

    /* Copy current ring buffer pointers */
    _Copy_CurrentPtr(ring, &ptr);

    if ((ptr.rd == ptr.wr) && (ptr.rd_wrap == ptr.wr_wrap))
        return MMP_TRUE;

    return MMP_FALSE;
}
EXPORT_SYMBOL(AUTL_RingBuf_Empty);

//------------------------------------------------------------------------------
//  Function    : AUTL_RingBuf_Full
//------------------------------------------------------------------------------
/** 
    @brief  Check if ring buffer full.
    @param[in] ring : ring buffer handler
    @return MMP_TRUE if ring buffer full, MMP_FALSE if there's still free space.
*/
MMP_BOOL AUTL_RingBuf_Full(AUTL_RINGBUF *ring)
{
    AUTL_RINGBUF_PTR ptr;

    if (!ring)
        return MMP_FALSE;

    /* Copy current ring buffer pointers */
    _Copy_CurrentPtr(ring, &ptr);

    if ((ptr.rd == ptr.wr) && ((ptr.rd_wrap + 1) == ptr.wr_wrap))
        return MMP_TRUE;

    return MMP_FALSE;
}
EXPORT_SYMBOL(AUTL_RingBuf_Full);


void MMPF_Audio_SetEncBufHandle(RINGBUF_ID id,AUTL_RINGBUF *handle)
{
    pAudBufHandle[id] = handle ;
    //printc("RingBuf Id[%d] : 0x%08x,size : %d\r\n",id,handle->buf_phy,handle->size);
}
EXPORT_SYMBOL(MMPF_Audio_SetEncBufHandle);


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_GetEncInBufHandle
//  Description : Get the encode input ring buffer handler
//------------------------------------------------------------------------------
AUTL_RINGBUF *MMPF_Audio_GetEncBufHandle(RINGBUF_ID id)
{
    return pAudBufHandle[id] ;
}
EXPORT_SYMBOL(MMPF_Audio_GetEncBufHandle);
//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InitInBufHandle
//  Description : Initialize the encode input ring buffer handler
//------------------------------------------------------------------------------
void MMPF_Audio_InitBufHandle(RINGBUF_ID id,AUTL_RINGBUF *handle,char *sBufAddr, char *sBufAddrPhy,MMP_ULONG ulBufSize)
{
    MMPF_Audio_SetEncBufHandle(id,handle);
    AUTL_RingBuf_Init(MMPF_Audio_GetEncBufHandle(id), sBufAddr, sBufAddrPhy,ulBufSize);
}
EXPORT_SYMBOL(MMPF_Audio_InitBufHandle);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_EncInBufDataStart
//  Description : Get the start addr of encode input ring buffer to read data
//------------------------------------------------------------------------------
char *MMPF_Audio_EncBufDataStart(RINGBUF_ID id,MMP_BOOL virt)
{
    CPU_LOCK_INIT();
    char *data_addr;
    AUTL_RINGBUF *ringbuf = MMPF_Audio_GetEncBufHandle(id) ;
    CPU_LOCK();
    if(virt) {
      data_addr =  (void *) ((char *)ringbuf->buf + ringbuf->ptr.rd);
    } 
    else {
      data_addr =  (void *) ((char *)ringbuf->buf_phy + ringbuf->ptr.rd);
    }
    CPU_UNLOCK();
    return data_addr;
}
EXPORT_SYMBOL(MMPF_Audio_EncBufDataStart);


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_AdvanceInBufReadPtr
//  Description : Advance the read pointer of encode input ring buffer
//------------------------------------------------------------------------------
void MMPF_Audio_AdvanceBufReadPtr(RINGBUF_ID id,MMP_ULONG ulSizeInBytes)
{
    AUTL_RingBuf_CommitRead(MMPF_Audio_GetEncBufHandle(id), ulSizeInBytes);
}
EXPORT_SYMBOL(MMPF_Audio_AdvanceBufReadPtr);


//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_AdvanceInBufWritePtr
//  Description : Advance the write pointer of encode input ring buffer
//------------------------------------------------------------------------------
void MMPF_Audio_AdvanceBufWritePtr(RINGBUF_ID id,MMP_ULONG ulSizeInBytes)
{
    AUTL_RingBuf_CommitWrite(MMPF_Audio_GetEncBufHandle(id), ulSizeInBytes);
}
EXPORT_SYMBOL(MMPF_Audio_AdvanceBufWritePtr);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InBufDataAvailable
//  Description : Get the available size of data in encode input ring buffer
//------------------------------------------------------------------------------
MMP_ULONG MMPF_Audio_BufDataAvailable(RINGBUF_ID id)
{
    MMP_ULONG size = 0;

    AUTL_RingBuf_DataAvailable(MMPF_Audio_GetEncBufHandle(id), &size);

    return size;
}
EXPORT_SYMBOL(MMPF_Audio_BufDataAvailable);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InBufFreeSpace
//  Description : Get the available free space in encode input ring buffer
//------------------------------------------------------------------------------
MMP_ULONG MMPF_Audio_BufFreeSpace(RINGBUF_ID id)
{
    MMP_ULONG size = 0;

    AUTL_RingBuf_SpaceAvailable(MMPF_Audio_GetEncBufHandle(id), &size);

    return size;
}
EXPORT_SYMBOL(MMPF_Audio_BufFreeSpace);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_InBufFull
//  Description : Check if encode input ring buffer full
//------------------------------------------------------------------------------
MMP_BOOL MMPF_Audio_BufFull(RINGBUF_ID id)
{
    return AUTL_RingBuf_Full(MMPF_Audio_GetEncBufHandle(id));
}
EXPORT_SYMBOL(MMPF_Audio_BufFull);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_ReadInBuf
//  Description : Read encode input ring buffer to target addr
//------------------------------------------------------------------------------
void MMPF_Audio_ReadBuf(RINGBUF_ID id,char *target, MMP_ULONG sample_bytes)
{
    CPU_LOCK_INIT();
    char *ring_data;
    MMP_ULONG  ring_end_ofst;
    AUTL_RINGBUF *ringbuf = MMPF_Audio_GetEncBufHandle(id) ;
    //MMP_ULONG sample_bytes = samples * sizeof(short) ;
    
    CPU_LOCK();
    ring_data = (char *)ringbuf->buf + ringbuf->ptr.rd;
    ring_end_ofst = ringbuf->size - ringbuf->ptr.rd;
    CPU_UNLOCK();

    if (ring_end_ofst >= /* samples*/sample_bytes) {
        memcpy(target, ring_data, /* samples << 1*/sample_bytes);
    }
    else {
        memcpy(target, ring_data, /*ring_end_ofst << 1*/ring_end_ofst);
        sample_bytes  -= ring_end_ofst;
        target   += ring_end_ofst;
        ring_data = (char *)ringbuf->buf;
        memcpy(target, ring_data, /*samples << 1*/sample_bytes);
    }
}
EXPORT_SYMBOL(MMPF_Audio_ReadBuf);

//------------------------------------------------------------------------------
//  Function    : MMPF_Audio_WriteInBuf
//  Description : Write PCM to encode input ring buffer
//------------------------------------------------------------------------------
void MMPF_Audio_WriteBuf(RINGBUF_ID id,char *data, MMP_ULONG sample_bytes)
{
    CPU_LOCK_INIT();
    char *ring_wr;
    MMP_ULONG  ring_end_ofst;
    AUTL_RINGBUF *ringbuf = MMPF_Audio_GetEncBufHandle(id) ;
    
    //MMP_ULONG sample_bytes = samples * sizeof(short) ;
    
    CPU_LOCK();
    ring_wr = (char *)ringbuf->buf +ringbuf->ptr.wr;
    ring_end_ofst =ringbuf->size -ringbuf->ptr.wr;
    CPU_UNLOCK();
    
    if (ring_end_ofst >= /*samples*/sample_bytes ) {
        memcpy(ring_wr, data, /*samples << 1*/sample_bytes);
    }
    else {
        memcpy(ring_wr, data, /*ring_end_ofst << 1*/ring_end_ofst );
        sample_bytes -= ring_end_ofst;
        data    += ring_end_ofst;
        ring_wr  = (char *)ringbuf->buf;
        memcpy(ring_wr, data, /*samples << 1*/sample_bytes);
    }
}
EXPORT_SYMBOL(MMPF_Audio_WriteBuf);
/*
read data from ring buffer to user space, and advance buffer pointer
*/
void MMPF_Audio_ReadBufToUser(RINGBUF_ID id,char *user,/* char *src, */MMP_ULONG sample_bytes)
{
    CPU_LOCK_INIT();
    MMP_ULONG size = sample_bytes; 
    char *ring_data;
    MMP_ULONG  ring_end_ofst;
    AUTL_RINGBUF *ringbuf = MMPF_Audio_GetEncBufHandle(id) ;
    //MMP_ULONG sample_bytes = samples * sizeof(short) ;
    CPU_LOCK();
    ring_data = (char *)ringbuf->buf + ringbuf->ptr.rd;
    ring_end_ofst = ringbuf->size - ringbuf->ptr.rd;
    CPU_UNLOCK();
    
//    if(ring_data != src) {
//      pr_info("[bad aac rd pointer]aac.src : 0x%08x,head : 0x%08x,bytes:%d\n",src,ring_data,sample_bytes);
//    }
     
    if (ring_end_ofst >= /* samples*/sample_bytes) {
        //memcpy(target, ring_data, /* samples << 1*/sample_bytes);
        copy_to_user( user , ring_data , sample_bytes );
        //pr_info("d0 : 0x%08x\n",ring_data);
    }
    else {
        //memcpy(target, ring_data, /*ring_end_ofst << 1*/ring_end_ofst);
        //pr_info("d1 : 0x%08x\n",ring_data);
        copy_to_user( user , ring_data , ring_end_ofst );
        sample_bytes  -= ring_end_ofst;
        user   += ring_end_ofst;
        ring_data = (char *)ringbuf->buf;
        //memcpy(target, ring_data, /*samples << 1*/sample_bytes);
        //pr_info("d2 : 0x%08x\n",ring_data);
        copy_to_user(user,ring_data,sample_bytes );
    }
    MMPF_Audio_AdvanceBufReadPtr(id,size);
}
EXPORT_SYMBOL(MMPF_Audio_ReadBufToUser);


void MMPF_Audio_WriteBufFromUser(RINGBUF_ID id,char *user,  MMP_ULONG sample_bytes)
{
    CPU_LOCK_INIT();
    char *ring_wr;
    MMP_ULONG size = sample_bytes; 
    MMP_ULONG  ring_end_ofst;
    AUTL_RINGBUF *ringbuf = MMPF_Audio_GetEncBufHandle(id) ;
    
    //MMP_ULONG sample_bytes = samples * sizeof(short) ;
    
    CPU_LOCK();
    ring_wr = (char *)ringbuf->buf +ringbuf->ptr.wr;
    ring_end_ofst =ringbuf->size -ringbuf->ptr.wr;
    CPU_UNLOCK();
   
    if (ring_end_ofst >= sample_bytes ) {
        copy_from_user(ring_wr, user , sample_bytes ) ;
    }
    else {
        copy_from_user(ring_wr,user,ring_end_ofst );
        sample_bytes -= ring_end_ofst;
        user    += ring_end_ofst;
        ring_wr  = (char *)ringbuf->buf;
        copy_from_user(ring_wr, user, sample_bytes);
    }
    MMPF_Audio_AdvanceBufWritePtr(id,size);
}
EXPORT_SYMBOL(MMPF_Audio_WriteBufFromUser);
