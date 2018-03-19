#ifndef __CPU_COMM_ID_H__
#define __CPU_COMM_ID_H__

typedef enum _CPU_COMM_ID {
  CPU_COMM_ID_SYSFLAG = 0,
  CPU_COMM_ID_UART,
  CPU_COMM_ID_TESTA2B ,    
  CPU_COMM_ID_MD,  
  CPU_COMM_ID_AEC,
  CPU_COMM_ID_AAC,
  CPU_COMM_ID_AES,
  CPU_COMM_ID_V4L2,
  CPU_COMM_ID_ALSA,
  CPU_COMM_ID_CMD,
  // this is the end of ID enumeration
  CPUCOMM_ID_MAX_NUM,

  // illeagle ID
  CPU_COMM_ID_ILLEGAL=0xEE,// fixed value cpu-a and cpu-b
  CPU_COMM_ID_CPU_B_NOT_READY   = 0xFF
  
} CPU_COMM_ID;

#endif // __CPU_COMM_ID_H__

