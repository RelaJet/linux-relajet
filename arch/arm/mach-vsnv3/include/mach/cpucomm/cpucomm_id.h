#ifndef __CPU_COMM_ID_H__
#define __CPU_COMM_ID_H__

typedef enum _CPU_COMM_ID {
  CPU_COMM_ID_SYSFLAG = 0,
  CPU_COMM_ID_UART,
  CPU_COMM_ID_TESTA2B ,    
  CPU_COMM_ID_MD,  
  

  // this is the end of ID enumeration
  CPUCOMM_ID_MAX_NUM,

  // illeagle ID
  CPU_COMM_ID_ILLEGAL,
  CPU_COMM_ID_CPU_B_NOT_READY   = 0xFF
} CPU_COMM_ID;

#endif // __CPU_COMM_ID_H__

