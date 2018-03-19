#ifndef _DEVONE_IOCTL_H
#define _DEVONE_IOCTL_H

#include <linux/ioctl.h>
#include "md.h"
#include "tamper.h"


#define MDIOC_MAGIC 0xEF

#define	MDIOCS_INIT		_IOW(MDIOC_MAGIC, MD_INIT, struct MD_init_s)
#define	MDIOCG_VERSION		_IOR(MDIOC_MAGIC, MD_GET_VERSION, int)
#define	MDIOCSG_RUN		_IOWR(MDIOC_MAGIC, MD_RUN, struct MD_proc_info_s)
#define	MDIOCS_WINDOW		_IOW(MDIOC_MAGIC, MD_SET_WINDOW, struct MD_detect_window_s)
#define	MDIOCG_WINDOW_SIZE	_IOR(MDIOC_MAGIC, MD_GET_WINDOW_SIZE, struct MD_detect_window_s)
#define	MDIOCS_WINDOW_PARA_IN	_IOW(MDIOC_MAGIC, MD_SET_WINDOW_PARA_IN, struct MD_window_parameter_in_s)
#define	MDIOCG_WINDOW_PARA_IN	_IOR(MDIOC_MAGIC, MD_GET_WINDOW_PARA_IN, struct MD_window_parameter_in_s)
#define	MDIOCSG_WINDOW_PARA_OUT	_IOWR(MDIOC_MAGIC, MD_GET_WINDOW_PARA_OUT, struct MD_window_parameter_out_s)
#define	MDIOCSG_BUFFER_INFO	_IOWR(MDIOC_MAGIC, MD_GET_BUFFER_INFO, struct MD_buffer_info_s)
#define	MDIOCS_MD_SUSPEND	_IOW(MDIOC_MAGIC, MD_SUSPEND, struct MD_suspend_s)
#define	MDIOCG_HISTGRAM		_IOR(MDIOC_MAGIC, MD_HISTGRAM, struct MD_motion_info_s)

//TD IOC
#define TDIOCS_INIT             _IOW(MDIOC_MAGIC, TD_INIT, struct TD_init_s)
#define TDIOCG_VERSION	        _IOR(MDIOC_MAGIC, TD_GET_VERSION, int)	
#define TDIOCSG_RUN             _IOW(MDIOC_MAGIC, TD_RUN, struct TD_run_s)
#define TDIOCS_WINDOW           _IOW(MDIOC_MAGIC, TD_SET_WINDOW, struct  TD_detect_window_s)
#define TDIOCG_WINDOW_SIZE      _IOR(MDIOC_MAGIC, TD_GET_WINDOW_SIZE, struct  TD_detect_window_size_s)
#define TDIOCS_PARA_IN          _IOW(MDIOC_MAGIC, TD_SET_PARA_IN, int)
#define TDIOCSG_WINDOW_RESULT   _IOWR(MDIOC_MAGIC, TD_GET_WINDOW_RESULT, struct  TD_window_result_s)
#define TDIOCSG_BUFFER_SIZE     _IOWR(MDIOC_MAGIC, TD_GET_BUFFER_SIZE, struct  TD_buffer_size_s)
#define TDIOCS_WINDOW_EN        _IOW(MDIOC_MAGIC, TD_SET_WINDOW_EN, struct  TD_set_window_enable_s)
#define TDIOCG_WINDOW_EN        _IOR(MDIOC_MAGIC, TD_GET_WINDOW_EN, struct  TD_get_window_enable_s)
#define TDIOCS_RELEASE          _IOW(MDIOC_MAGIC, TD_RELEASE, int)	

#endif

