/*
 * md.h
 *
 *  Created on: 2014/1/8
 *      Author: chris
 *  Modify on : 2015/03/18
 *      Author: Vincent
 *      Description:  Remove pointer in structure for Linux+RTOS porting.
 *
 *
 */

#ifndef MD_HPP_
#define MD_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#define AIT_MD_LIB_VER	(20141120)

typedef enum
{
    MD_INIT,
    MD_GET_VERSION,	
    MD_RUN,
    MD_SET_WINDOW,
    MD_GET_WINDOW_SIZE,
    MD_SET_WINDOW_PARA_IN,
    MD_GET_WINDOW_PARA_IN,
    MD_GET_WINDOW_PARA_OUT,
    MD_GET_BUFFER_INFO,
    MD_SUSPEND,
    MD_HISTGRAM,
    TD_INIT, //TD command
    TD_GET_VERSION,	
    TD_RUN,
    TD_SET_WINDOW,
    TD_GET_WINDOW_SIZE,
    TD_SET_PARA_IN,
    TD_GET_WINDOW_RESULT,
    TD_GET_BUFFER_SIZE,
    TD_SET_WINDOW_EN,
    TD_GET_WINDOW_EN,
    TD_RELEASE,	
    
    // Event from CPU-B definition
    MD_EVENT_CHANGES = 100 , // motion <-> non-motion changes
   
} MD_command_in_t;

typedef struct MD_params_in_s
{
	//(0: disable, 1: enable)
	unsigned char enable;
	//(0 ~ 99)
    unsigned char size_perct_thd_min;
    //(1 ~ 100), must be larger than size_perct_thd_min
    unsigned char size_perct_thd_max;
    //(10, 20, 30, ..., 100), 100 is the most sensitive
    unsigned char sensitivity;
    //(1000 ~ 30000)
    unsigned short learn_rate;
} MD_params_in_t;

typedef struct MD_params_out_s
{
    unsigned char md_result;
    unsigned int obj_cnt;
} MD_params_out_t;

typedef struct MD_init_s
{
	unsigned char* working_buf_ptr;
	int working_buf_len;
	unsigned short width;
	unsigned short height;
	unsigned char color;
} MD_init_t;

typedef struct MD_detect_window_s
{
	unsigned short lt_x;
	unsigned short lt_y;
	unsigned short rb_x;
	unsigned short rb_y;
	unsigned char w_div;
	unsigned char h_div;
} MD_detect_window_t;

typedef struct MD_window_parameter_in_s
{
	unsigned char w_num;
	unsigned char h_num; 
	MD_params_in_t param;
} MD_window_parameter_in_t;

typedef struct MD_window_parameter_out_s
{
	unsigned char w_num;
	unsigned char h_num; 
	MD_params_out_t param;
} MD_window_parameter_out_t;

typedef struct MD_buffer_info_s
{
	unsigned short width;
	unsigned short height;
	unsigned char color;
	unsigned char w_div;
	unsigned char h_div;
	unsigned long return_size;	
} MD_buffer_info_t;

typedef struct MD_suspend_s
{
	unsigned char md_suspend_enable;
	unsigned char md_suspend_duration;
	unsigned char md_suspend_threshold;
} MD_suspend_t;

typedef struct MD_motion_info_s
{
	unsigned short obj_cnt ;
	unsigned short obj_axis[256] ; // funny ~
} MD_motion_info_t ;

typedef struct MD_proc_info_s
{
    unsigned char *frame ;
    MD_motion_info_t result ;  // return result   
} MD_proc_info_t ;

extern unsigned int MD_GetLibVersion(unsigned int* ver);
extern int MD_init(unsigned char* working_buf_ptr, int working_buf_len, unsigned short width, unsigned short height, unsigned char color);
extern int MD_run(unsigned char* _ucImage,struct MD_motion_info_s *md_info);
extern int MD_set_detect_window(unsigned short lt_x, unsigned short lt_y, unsigned short rb_x, unsigned short rb_y, unsigned char w_div, unsigned char h_div);
extern int MD_get_detect_window_size(unsigned short* st_x, unsigned short* st_y, unsigned short* div_w, unsigned short* div_h);
extern int MD_set_window_params_in(unsigned char w_num, unsigned char h_num, MD_params_in_t* param);
extern int MD_get_window_params_in(unsigned char w_num, unsigned char h_num, MD_params_in_t* param);
extern int MD_get_window_params_out(unsigned char w_num, unsigned char h_num, MD_params_out_t* param);
extern int MD_get_buffer_info(unsigned short width, unsigned short height, unsigned char color, unsigned char w_div, unsigned char h_div);
extern void MD_suspend(unsigned char md_suspend_enable, unsigned char md_suspend_duration, unsigned char md_suspend_threshold);
extern void MD_printk(char *fmt, ...);
extern void MD_set_time_ms(unsigned int time_diff);
extern int MD_histgram(struct MD_motion_info_s *md_info);

#ifdef __cplusplus
}
#endif

#endif /* MD_HPP_ */
