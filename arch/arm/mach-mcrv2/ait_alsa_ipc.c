#include <linux/module.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <mach/mmpf_typedef.h>
//#include <mach/mmp_cpucomm.h>
#include <mach/cpucomm/cpucomm_api.h>
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/ait_cpu_sharemem.h>
#include <mach/mcrv2_afe.h>

#include <mach/ait_alsa_ipc.h>
extern  MMP_BOOL     m_bDACPowerOnState ;
extern  MMP_BOOL     m_bADCPowerOnState ;
//extern  AUDIO_IPC_MODE audio_ipc_mode = AUDIO_IPC_EN ;


static void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma) ;
static void cpub_free(int size,void *va,dma_addr_t h_dma) ;

#define ALSA_GAIN(d,a)  ((( a & 0xffff) << 16 ) | (d&0xffff) )
#if 1 
#define MS_FUNC(who,statement)  statement
#else
#define MS_FUNC(who,statement)				\
do {										              \
	MMP_ULONG64 t1,t2 ;                   \
	MMPF_OS_GetTimestamp(&t1,MMPF_OS_TIME_UNIT_JIFFIES);    \
	statement ;								                            \
	MMPF_OS_GetTimestamp(&t2,MMPF_OS_TIME_UNIT_JIFFIES);		\
	pr_info("%s time : %d\r\n",who,t2 - t1 );\
} while (0)
#endif
#define ALSA_IPC_TIMEOUT		(1000)
#define COM_OBJ_SZ    sizeof(struct cpu_comm_transfer_data )
#define COM_DMA(addr)             (unsigned long)addr                                

#define COM_SEND()          do {                \
                                MS_FUNC("s:",err = CpuComm_DataSend( cpucmm_id, (MMP_UBYTE*)data, COM_OBJ_SZ,ALSA_IPC_TIMEOUT ) );         \
                                if(! err ) {	\
                                	MS_FUNC("r:",err = CpuComm_DataReceive( cpucmm_id, (MMP_UBYTE*)data,  COM_OBJ_SZ,ALSA_IPC_TIMEOUT,0) );   \
                                } \
                            } while(0)
                            
#define COM_PAYLOAD_ALLOC(size) do {    \
                                    payload = cpub_malloc(size,GFP_KERNEL,&h_dma);    \
                                    if(!payload) {          \
                                        return -ENOMEM ;    \
                                    }                       \
                                } while (0)

#define COM_PAYLOAD_FREE(size) cpub_free(size,payload,h_dma )

struct alsa_ipc_irq_t
{
  unsigned int irq;
  irq_handler_t handler ;
  void *dev ;  
} ;

static int alsa_ipc_afe_owner[] = {AFE_OWNER_IS_RTOS,AFE_OWNER_IS_RTOS } ;


static int cmd_seq_cnt ;
static CPU_COMM_ID cpucmm_id = -1;


static struct alsa_ipc_irq_t alsa_ipc_irq[2] ;
static struct dma_pool *cpub_dma_pool = 0 ;
static alsa_ring_bufctl *pcm_buffer[2] ;
static dma_addr_t       pcm_buffer_phy[2];

static int debug = 0;

#define DEF_DMA_POOL_BLK_SIZE (64)
static int cpub_create_pool(void)
{
    cpub_dma_pool = dma_pool_create("cpub_pool", 0,DEF_DMA_POOL_BLK_SIZE, 16, 0);
    if(!cpub_dma_pool) {
        return -ENOMEM ;
    }
    return 0 ;
}

static int cpub_delete_pool(void)
{
    if(cpub_dma_pool) {
        dma_pool_destroy(cpub_dma_pool);        
    }    
    return 0 ;
}

static void *cpub_malloc(int size,gfp_t flags, dma_addr_t *h_dma)
{
    //dma_addr_t h_dma ;
    if(!cpub_dma_pool) {
        return (void *) 0 ;    
    }
    return dma_pool_alloc(cpub_dma_pool,flags,h_dma);
}

static void cpub_free(int size,void *va,dma_addr_t h_dma)
{
    dma_pool_free(cpub_dma_pool,va,h_dma);
}

int alsa_ipc_setowner(int path,int owner)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  pr_info("alsa_ipc_setowner\n");
  if( alsa_ipc_register() < 0 ) {
      alsa_ipc_afe_owner[path] = AFE_OWNER_IS_LINUX ;
      return 0;
  }
  
	data->command 	= IPC_ALSA_SETOWNER;
	data->phy_addr	= path;
	data->size			= owner;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  alsa_ipc_afe_owner[path] = AFE_OWNER_IS_LINUX ;
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_setowner result:%lu\n", data->result);
	  return -EBUSY ; 
	}
	alsa_ipc_afe_owner[path] = owner ;
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_setowner);

int alsa_ipc_getowner(int path)
{
  return alsa_ipc_afe_owner[path] ;
}
EXPORT_SYMBOL(alsa_ipc_getowner);

int alsa_ipc_poweron( alsa_ipc_info *ipc_info,unsigned long virt_addr,unsigned long phy_addr,int buf_size) 
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
	void *payload ;
	dma_addr_t h_dma ;
	alsa_ipc_info *ipc_info_send ;
	int aud_path ;
	if(debug) {
    pr_info("alsa_ipc_poweron : path : %d, samplerate : %d\n",ipc_info->aud_info.aud_path,ipc_info->aud_info.sample_rate);
    pr_info("alsa_ipc_poweron : buf(virt,byte):(0x%08lx,0x%08lx),size:%d\n",virt_addr,phy_addr,buf_size);
  }
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  aud_path = ipc_info->aud_info.aud_path ;
  if(!pcm_buffer[aud_path]) {
    return -ENOMEM;
  }
  COM_PAYLOAD_ALLOC(sizeof(alsa_ipc_info)) ;
  if(!payload) {
    return -ENOMEM ;  
  }
  /*
  pcm_buffer->dma_inbuf_virt_addr   = (void *)virt_addr ;
	pcm_buffer->dma_outbuf_virt_addr  = (void *)virt_addr;
	pcm_buffer->dma_inbuf_phy_addr    = (unsigned long)phy_addr;
	pcm_buffer->dma_outbuf_phy_addr   = (unsigned long)phy_addr;
	pcm_buffer->dma_inbuf_buffer_size = buf_size;
	pcm_buffer->dma_outbuf_buffer_size= buf_size;
  */
  alsa_ipc_init_buffer( aud_path,(char *)virt_addr,(char *)phy_addr,buf_size );
  
  ipc_info->pcm_buffer = pcm_buffer[aud_path] ;
  
  ipc_info_send = (alsa_ipc_info *)payload ;
  *ipc_info_send = *ipc_info ;
  ipc_info_send->pcm_buffer = (alsa_ring_bufctl *)pcm_buffer_phy[aud_path] ;
  if(debug)
    pr_info("#ipc_info:0x%08x,pcm_buffer phy:0x%08x\n",ipc_info_send,ipc_info_send->pcm_buffer);
  
	data->command 	= IPC_ALSA_POWERON;
	data->phy_addr	= COM_DMA(h_dma);
	data->size			= sizeof(alsa_ipc_info);
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	COM_PAYLOAD_FREE(sizeof(alsa_ipc_info)) ;
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("ipc_streamon result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	if(aud_path==ALSA_MIC) {
	  m_bADCPowerOnState = MMP_TRUE ;
  }
  else {
    m_bDACPowerOnState = MMP_TRUE;
  }
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_poweron);


int alsa_ipc_poweroff( int aud_path ) 
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  pr_info("alsa_ipc_poweroff\n");
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  
	data->command 	= IPC_ALSA_POWEROFF;
	data->phy_addr	= aud_path;
	data->size			= 0;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_poweroff result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	if(aud_path==ALSA_MIC) {
	  m_bADCPowerOnState = MMP_FALSE;
  }
  else {
	  m_bDACPowerOnState = MMP_FALSE;
  
  }
	return 0;	 
}
EXPORT_SYMBOL(alsa_ipc_poweroff);


int alsa_ipc_trigger_start(int path, int period)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  //pr_info("alsa_ipc_trigger_start\n");
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  
	data->command 	= IPC_ALSA_TRIGGER_START;
	data->phy_addr	= path;
	data->size			= period;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_trigger_start result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	return 0;	 
}
EXPORT_SYMBOL(alsa_ipc_trigger_start);

int alsa_ipc_trigger_stop(int path)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  pr_info("alsa_ipc_trigger_stop\n");
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  
	data->command 	= IPC_ALSA_TRIGGER_STOP;
	data->phy_addr	= path;
	data->size			= 0;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_trigger_stop result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_trigger_stop);

static void alsa_ipc_cmd_noack(int aud_path ,aitalsa_ipc_cmd cmd)
{
    struct cpu_share_mem_slot *slot;
    int channel = 1 ;//get_noack_channel(CPU_COMM_ID_ALSA, 1);

    while(1) {
        if (get_slot(channel, &slot) == 0) {
            slot->dev_id        = CPU_COMM_ID_ALSA;
            slot->command       = cmd;
            slot->data_phy_addr = 0;
            slot->size          = 0;
            slot->send_parm[0]  = aud_path;
            slot->send_parm[1]  = 0;
            slot->send_parm[2]  = 0;
            slot->send_parm[3]  = 0;
            slot->ack_func      = 0;
            send_slot(channel, slot);
            break;
        }
        else {
        	pr_info("%s can't get slot\r\n", __func__);
        }
    }
}

int alsa_ipc_trigger_stop_noack(int path)
{
  alsa_ipc_cmd_noack(path , IPC_ALSA_TRIGGER_STOP);  
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_trigger_stop_noack);


int alsa_ipc_mute(int path,int mute) 
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  //pr_info("alsa_ipc_mute[%d]=%d\n",path,mute);
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  
	data->command 	= IPC_ALSA_MUTE;
	data->phy_addr	= path;
	data->size			= mute;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_mute result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_mute);

int alsa_ipc_setgain(int aud_path,unsigned short dgain,unsigned short again)
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  //pr_info("alsa_ipc_setgain\n");
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
	data->command 	= IPC_ALSA_GAIN;
	data->phy_addr	= aud_path;
	data->size			= ALSA_GAIN(dgain,again);
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_setgain result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_setgain);

static  unsigned int alsa_ipc_frame_done(void *slot)
{ 
  int aud_path ;
  int samples = 0 ;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  struct cpu_share_mem_slot *r_slot = (struct cpu_share_mem_slot *)slot ;
  if(! r_slot ) {
    pr_info("#unknow slot isr\n");
    return 0;  
  }
  aud_path= (int)r_slot->send_parm[0];
  samples = (int)r_slot->send_parm[1] ;
  if(r_slot->dev_id==CPU_COMM_ID_ALSA) {
    switch (r_slot->command) 
    {
      case IPC_ALSA_ISR:
      alsa_ipc_irq[aud_path].handler(alsa_ipc_irq[aud_path].irq,alsa_ipc_irq[aud_path].dev) ;
      break ;
    }
  }
  return 0;
}



int alsa_ipc_init_buffer(int aud_path,void *virt_addr, void *phy_addr,unsigned long bytes)
{
  
  if (!pcm_buffer[aud_path])
      return -1;
  pcm_buffer[aud_path]->dma_inbuf_virt_addr   = (void *)virt_addr ;
	pcm_buffer[aud_path]->dma_outbuf_virt_addr  = (void *)virt_addr;
	pcm_buffer[aud_path]->dma_inbuf_phy_addr    = (unsigned long)phy_addr;
	pcm_buffer[aud_path]->dma_outbuf_phy_addr   = (unsigned long)phy_addr;
	pcm_buffer[aud_path]->dma_inbuf_buffer_size = bytes;
	pcm_buffer[aud_path]->dma_outbuf_buffer_size= bytes;
 
  pcm_buffer[aud_path]->in_buf_h.buf       = (void *)virt_addr;
  pcm_buffer[aud_path]->in_buf_h.buf_phy   = (void *)phy_addr ;
  pcm_buffer[aud_path]->in_buf_h.size      = bytes;
  
  /* Reset ring buffer pointers */
  pcm_buffer[aud_path]->in_buf_h.ptr.rd   = 0;
  pcm_buffer[aud_path]->in_buf_h.ptr.wr        = 0;
  pcm_buffer[aud_path]->in_buf_h.ptr.rd_wrap   = 0;
  pcm_buffer[aud_path]->in_buf_h.ptr.wr_wrap   = 0;
  
  if(debug) {
    pr_info("ringbuf_s : [0x%08x,0x%08x],size:%d (ring:0x%08x)\n",virt_addr,phy_addr,bytes,pcm_buffer[aud_path]);
    pr_info("dma_inbuf_virt_addr:0x%08x\r\n" ,pcm_buffer[aud_path]->dma_inbuf_virt_addr );
    pr_info("dma_outbuf_virt_addr:0x%08x\r\n",pcm_buffer[aud_path]->dma_outbuf_virt_addr);
    pr_info("dma_inbuf_phy_addr:0x%08x\r\n"  ,pcm_buffer[aud_path]->dma_inbuf_phy_addr  );
    pr_info("dma_outbuf_phy_addr:0x%08x\r\n" ,pcm_buffer[aud_path]->dma_outbuf_phy_addr );
    pr_info("in_buf_h:0x%08x\r\n"  ,&pcm_buffer[aud_path]->in_buf_h  );
    pr_info("out_buf_h:0x%08x\r\n" ,&pcm_buffer[aud_path]->out_buf_h );
  }
  return 0;
}
EXPORT_SYMBOL(alsa_ipc_init_buffer);

int alsa_ipc_resize_ringbuf(int aud_path,unsigned long bytes)
{
  pcm_buffer[aud_path]->in_buf_h.size      = bytes; 
  pr_info("resize dmabuf[%d] size:%d bytes\n",aud_path,bytes );
  
  return 0;
}
EXPORT_SYMBOL(alsa_ipc_resize_ringbuf);

/*
here only use in_buf_h
*/
int alsa_ipc_rd_mic_data(unsigned long bytes)
{
    CPU_LOCK_INIT();
    alsa_ring_bufctl *ring = (alsa_ring_bufctl *)pcm_buffer[ALSA_MIC] ;
    if (!ring)
      return -1;

    CPU_LOCK();
    ring->in_buf_h.ptr.rd += bytes;
    if (ring->in_buf_h.ptr.rd >= ring->in_buf_h.size) {
        ring->in_buf_h.ptr.rd -= ring->in_buf_h.size;
        ring->in_buf_h.ptr.rd_wrap++;
    }
    CPU_UNLOCK();
    //pr_info("wr(0x%08x,0x%08x),size:%d\r\n",(u32)ring->in_buf_h.buf_phy + ring->in_buf_h.ptr.wr,(u32)ring->in_buf_h.buf + ring->in_buf_h.ptr.wr,bytes);
    //pr_info("rd(0x%08x,0x%08x),size:%d\r\n",(u32)ring->in_buf_h.buf_phy + ring->in_buf_h.ptr.rd,(u32)ring->in_buf_h.buf + ring->in_buf_h.ptr.rd,bytes);

    return 0;
}
EXPORT_SYMBOL(alsa_ipc_rd_mic_data);

/*
here only use out_buf_h
*/
int alsa_ipc_wr_spk_data(unsigned long bytes)
{
    CPU_LOCK_INIT();
    alsa_ring_bufctl *ring = (alsa_ring_bufctl *)pcm_buffer[ALSA_SPK] ;
    if (!ring)
      return -1;

    CPU_LOCK();
    ring->in_buf_h.ptr.wr+= bytes;
    if (ring->in_buf_h.ptr.wr >= ring->in_buf_h.size) {
        ring->in_buf_h.ptr.wr -= ring->in_buf_h.size;
        ring->in_buf_h.ptr.wr_wrap++;
    }
    CPU_UNLOCK();
    //pr_info("wr(0x%08x,0x%08x),size:%d\r\n",(u32)ring->in_buf_h.buf_phy + ring->in_buf_h.ptr.wr,(u32)ring->in_buf_h.buf + ring->in_buf_h.ptr.wr,bytes);
    //pr_info("rd(0x%08x,0x%08x),size:%d\r\n",(u32)ring->in_buf_h.buf_phy + ring->in_buf_h.ptr.rd,(u32)ring->in_buf_h.buf + ring->in_buf_h.ptr.rd,bytes);
    //pr_info("[%d]\n",bytes);
    return 0;
}
EXPORT_SYMBOL(alsa_ipc_wr_spk_data);

int alsa_ipc_reset_spk_rd_data(void)
{
    CPU_LOCK_INIT();
    alsa_ring_bufctl *ring = (alsa_ring_bufctl *)pcm_buffer[ALSA_SPK] ;
    if (!ring)
      return -1;

    CPU_LOCK();
    ring->in_buf_h.ptr.rd = 0;
    ring->in_buf_h.ptr.rd_wrap = 0;
    CPU_UNLOCK();
    return 0;
}
EXPORT_SYMBOL(alsa_ipc_reset_spk_rd_data);

int alsa_ipc_reset_spk_wr_data(unsigned long bytes)
{
    CPU_LOCK_INIT();
    alsa_ring_bufctl *ring = (alsa_ring_bufctl *)pcm_buffer[ALSA_SPK] ;
    if (!ring)
      return -1;
    CPU_LOCK();
    ring->in_buf_h.ptr.wr_wrap = 0 ;
    alsa_ipc_wr_spk_data(bytes);
    CPU_UNLOCK();
    pr_info("[SEAN] : spk in %d bytes\n",bytes );
    return 0;
  
}
EXPORT_SYMBOL(alsa_ipc_reset_spk_wr_data);


int alsa_ipc_register_irq(int aud_path,unsigned int irq, irq_handler_t handler,void *dev_id )
{
    if(!handler || !dev_id) {
      return -EINVAL;
    }
		alsa_ipc_irq[aud_path].irq = irq ;
		alsa_ipc_irq[aud_path].handler = handler ;
		alsa_ipc_irq[aud_path].dev = dev_id ;
		if(debug) {
		  pr_info("  alsa_ipc_register_irq,path:%d,irq:%d,handler:0x%08x,0x%08x\n",aud_path,irq,(u32)handler,(u32)dev_id);
	  }
		return 0;
}
EXPORT_SYMBOL(alsa_ipc_register_irq);

int alsa_ipc_set_aec( AUDIO_ENH_MODE aec_mode )
{
  CPU_COMM_ERR err ;
	struct cpu_comm_transfer_data __data,*data=&__data ;
  pr_info("alsa_ipc_set_aec:%d\n",aec_mode);
  if( alsa_ipc_register() < 0 ) {
      return 0;
  }
  
	data->command 	= IPC_ALSA_SET_AEC ;
	data->phy_addr	= aec_mode;
	data->size			= 0;
	data->seq			= cmd_seq_cnt++;
	data->flag		= CPUCOMM_FLAG_CMDSND; 
	COM_SEND();
	if (err ) {
	  return -EINVAL ;
	}
	if(data->result) {
	  pr_info("alsa_ipc_set_aec result:%lu\n", data->result);
	  return -EINVAL ; 
	}
	return 0;	  
}
EXPORT_SYMBOL(alsa_ipc_set_aec);

int alsa_ipc_is_register(void)
{
	if(cpucmm_id != ( (CPU_COMM_ID)-1) ) {
	  return 0;  
	}
  return -1 ;
}
EXPORT_SYMBOL(alsa_ipc_is_register);

int alsa_ipc_register(void)
{
	CPU_COMM_ERR ret = 0;
	if(cpucmm_id != ( (CPU_COMM_ID)-1) ) {
	  //pr_info("#alsa ipc already register\n");
	  return 0;  
	}
	
	if(cpub_create_pool() < 0) {
		pr_err("alsa_ipc : create cpub dma pool failed\n");
		return -ENOMEM;  
	}
	pcm_buffer[ALSA_MIC] = cpub_malloc( sizeof(alsa_ring_bufctl) ,GFP_KERNEL, &pcm_buffer_phy[ALSA_MIC]); 
  if(!pcm_buffer[ALSA_MIC]) {
	  return -ENOMEM ;
  }
	pcm_buffer[ALSA_SPK] = cpub_malloc( sizeof(alsa_ring_bufctl) ,GFP_KERNEL, &pcm_buffer_phy[ALSA_SPK]); 
  if(!pcm_buffer[ALSA_SPK]) {
	  return -ENOMEM ;
  }


	ret = CpuComm_RegisterEntry(CPU_COMM_ID_ALSA, CPU_COMM_TYPE_DATA);
	if(ret==CPU_COMM_ERR_NONE) {
		cpucmm_id = CPU_COMM_ID_ALSA;
		ret = CpuComm_RegisterISRService( CPU_COMM_ID_ALSA,alsa_ipc_frame_done);
		pr_info("#Enable ALSA ISR Service ret:%d\n",ret);
		
      
		return 0 ;
	}	
	else {
	  cpucmm_id = (CPU_COMM_ID)-1 ;  
	  // free memory
	  alsa_ipc_unregister();
	}
	return -EINVAL;
}
EXPORT_SYMBOL(alsa_ipc_register);

int alsa_ipc_unregister(void)
{
  if(cpucmm_id != ( (CPU_COMM_ID)-1)) {
	  CpuComm_UnregisterEntry(cpucmm_id);
  }
  if(pcm_buffer[ALSA_MIC]) {
    cpub_free( sizeof(alsa_ring_bufctl),pcm_buffer[ALSA_MIC],pcm_buffer_phy[ALSA_MIC] )  ;
    pcm_buffer[ALSA_MIC] = 0 ;
  }
  if(pcm_buffer[ALSA_SPK]) {
    cpub_free( sizeof(alsa_ring_bufctl),pcm_buffer[ALSA_SPK],pcm_buffer_phy[ALSA_SPK] )  ;
    pcm_buffer[ALSA_SPK] = 0 ;
  }

  cpub_delete_pool();
	return 0;
}
EXPORT_SYMBOL(alsa_ipc_unregister);

MODULE_LICENSE("GPL");