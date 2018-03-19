#ifndef AIT_CAM_IPC_H_
#define AIT_CAM_IPC_H_
#include <linux/interrupt.h>
#include <mach/mcrv2_afe.h>
//#include "ait_cam_common.h"

#define ALSA_MIC        (0)
#define ALSA_SPK        (1)
#define ALSA_SRC_END    (2)

#define AFE_OWNER_IS_RTOS   (0)
#define AFE_OWNER_IS_LINUX  (1)

typedef enum
{
    IPC_ALSA_SETOWNER,
    IPC_ALSA_POWERON,
    IPC_ALSA_POWEROFF,
    IPC_ALSA_MUTE,
    IPC_ALSA_GAIN,
    IPC_ALSA_RW,
    IPC_ALSA_ISR,
    IPC_ALSA_TRIGGER_START,
    IPC_ALSA_TRIGGER_STOP,
    IPC_ALSA_SET_AEC,
    IPC_ALSA_NOT_SUPPORT
    
} aitalsa_ipc_cmd ;

typedef packed_struct  _asla_audio_ring_ptr {
    unsigned long   rd;         ///< read pointer
    unsigned long   wr;         ///< write pointer
    unsigned long   rd_wrap;    ///< read pointer wrap count
    unsigned long   wr_wrap;    ///< write pointer wrap count
} asla_audio_ring_ptr ;

typedef packed_struct _alsa_audio_buffer {
    void            *buf;   ///< ring buffer address
    void            *buf_phy ;
    unsigned long   size;  ///< ring buffer size
    asla_audio_ring_ptr ptr;   ///< ring buffer pointers
} alsa_audio_buffer;

typedef packed_struct _alsa_ring_bufctl
{
    void *dma_inbuf_virt_addr;
    void *dma_outbuf_virt_addr ;
    unsigned long dma_inbuf_phy_addr;
    unsigned long dma_outbuf_phy_addr;
    int  dma_inbuf_buffer_size;
    int  dma_outbuf_buffer_size  ;
    alsa_audio_buffer  in_buf_h,out_buf_h ;    
} alsa_ring_bufctl  ;

typedef packed_struct  _alsa_audio_info
{
    int aud_path ;
    int sample_rate ;
    int channels ;
    int irq_threshold ;
} alsa_audio_info ;

typedef packed_struct  _alsa_ipc_info
{
    alsa_audio_info  aud_info ;
    alsa_ring_bufctl *pcm_buffer;
    /*
    int           pcm_buf_size  ;
    unsigned long pcm_virt_addr ;
    unsigned long pcm_phy_addr  ;
    */
} alsa_ipc_info;

int alsa_ipc_setowner(int aud_path,int owner);
int alsa_ipc_getowner(int aud_path);
int alsa_ipc_poweron( alsa_ipc_info *ipc_info,unsigned long virt_addr,unsigned long phy_addr,int buf_size) ;
int alsa_ipc_poweroff(int aud_path) ;
int alsa_ipc_mute(int aud_path,int mute);
int alsa_ipc_setgain(int aud_path,unsigned short dgain,unsigned short again);
int alsa_ipc_register(void) ;
int alsa_ipc_unregister(void);
int alsa_ipc_is_register(void);
int alsa_ipc_init_buffer(int aud_path,void *virt_addr, void *phy_addr,unsigned long size) ;
int alsa_ipc_rd_mic_data(unsigned long bytes);
int alsa_ipc_wr_spk_data(unsigned long bytes);
int alsa_ipc_reset_spk_rd_data(void);

int alsa_ipc_reset_spk_wr_data(unsigned long bytes);
int alsa_ipc_register_irq(int aud_path,unsigned int irq, irq_handler_t handler,void *dev_id );
int alsa_ipc_resize_ringbuf(int aud_path,unsigned long bytes);
int alsa_ipc_trigger_start(int path, int period);
int alsa_ipc_trigger_stop(int path);
int alsa_ipc_trigger_stop_noack(int path);
int alsa_ipc_set_aec( AUDIO_ENH_MODE aec_mode );
#endif