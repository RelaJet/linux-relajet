/*
 * Driver for MD alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-vsnv3/cpucomm/cpub_md.c which has following copyrights:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef CPUB_MD_CPUCOMM_H
#define CPUB_MD_CPUCOMM_H
#include "md.h"
#include "tamper.h"

extern void enable_md_event(int en);
extern int cpub_md_enable_event(int enable);
extern int cpub_md_set_window(struct MD_detect_window_s* pWin);
extern int cpub_md_set_para(struct MD_window_parameter_in_s* para);
extern int cpub_md_get_windowsize(struct MD_detect_window_s* win);
extern int cpub_md_get_window_para(struct MD_window_parameter_in_s* para);
extern int cpub_md_get_buffer_info(struct MD_buffer_info_s* info);
extern int cpub_md_get_version(unsigned int * version);
extern int cpub_md_get_result(struct MD_window_parameter_out_s* para);
extern int cpub_md_suspend(struct MD_suspend_s* suspndinfo);
extern int cpub_md_run(void *frame,struct MD_motion_info_s *md_info);
extern int cpub_md_init(struct MD_init_s* pInit);
extern int cpub_md_register(int id);
extern int cpub_md_unregister(void);
extern int cpub_md_histgram(struct MD_motion_info_s *md_info);


//TD
extern int cpub_td_init(struct TD_init_s* pInit);
extern int cpub_td_get_version(unsigned int* version); 
extern int cpub_td_run(void *frame,int* td_result); 
extern int cpub_td_set_window(struct TD_detect_window_s* size_para);
extern int cpub_td_get_window_size(struct TD_detect_window_size_s* size_para);
extern int cpub_td_set_para_in(struct Tamper_params_in_s* para_in);
extern int cpub_td_get_window_result(struct TD_window_result_s *td_win_result);
extern int cpub_td_get_buffer_size(struct TD_buffer_size_s* buf_size);
extern int cpub_td_set_windows_en(struct TD_set_window_enable_s* win_en);
extern int cpub_td_get_windows_en(struct TD_get_window_enable_s* win_en); 
extern int cpub_td_release(void);	


#endif
