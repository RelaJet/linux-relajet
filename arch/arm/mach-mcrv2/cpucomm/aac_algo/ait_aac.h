#ifndef _AIT_AAC_H
#define _AIT_AAC_H

#include <linux/ioctl.h>
#include "aac.h"



#define AACIOC_MAGIC 0xF0

#define	AACIOCS_INIT							_IOW(AACIOC_MAGIC, AAC_INIT, struct AAC_init_s)
#define	AACIOCSG_RUN							_IOWR(AACIOC_MAGIC, AAC_ENCODE, struct AAC_proc_info_s)

#endif

