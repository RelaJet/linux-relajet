#ifndef _AIT_AES_H
#define _AIT_AES_H

#include <linux/ioctl.h>
#include "aes.h"



#define AESIOC_MAGIC 0xF1

#define	AESIOCS_INIT							_IOW(AESIOC_MAGIC, AES_INIT, struct AES_init_s)
#define	AESIOCS_RUN							_IOW(AESIOC_MAGIC, AES_ENCODE, struct AES_proc_info_s)

#endif

