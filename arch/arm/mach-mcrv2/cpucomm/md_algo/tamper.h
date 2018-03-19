/*
 * tamper.h
 *
 *  Created on: 2014/3/20
 *      Author: chris
 */

#ifndef TAMPER_H_
#define TAMPER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Tamper_params_in_s
{
	unsigned char thd_tamper; // multiply 10
	unsigned char tamper_blk_thd;
	unsigned char min_duration; // frame
	unsigned char alpha; // multiply 10
	unsigned short M; // frame
} Tamper_params_in_t;

typedef struct Tamper_params_out_s
{
	signed char flag_tamper;
	unsigned int cnt;
	unsigned int ref_cnt;
} Tamper_params_out_t;

typedef struct TD_init_s
{
	unsigned char* working_buf_ptr;
	int working_buf_len;
	unsigned short width;
	unsigned short height;
	unsigned char color;
} TD_init_t;

typedef struct TD_detect_window_s
{
	unsigned short lt_x;
	unsigned short lt_y;
	unsigned short rb_x;
	unsigned short rb_y;
	unsigned char w_div;
	unsigned char h_div;
} TD_detect_window_t;

typedef struct TD_detect_window_size_s
{
	unsigned short* st_x;
 	unsigned short* st_y;
	unsigned short* div_w;
 	unsigned short* div_h;
	int return_size;
} TD_detect_window_size_t;

typedef struct TD_buffer_size_s
{
	unsigned short width;
	unsigned short height;
	unsigned char color;
	unsigned char w_div;
	unsigned char h_div;
	int return_size;	
} TD_buffer_size_t;

typedef struct TD_set_window_enable_s
{
	unsigned char w_num;
	unsigned char h_num;
	unsigned char en;
} TD_set_window_enable_t ;

typedef struct TD_get_window_enable_s
{
	unsigned char w_num;
	unsigned char h_num;
	unsigned char *en;
} TD_get_window_enable_t ;

typedef struct TD_window_result_s
{
	unsigned char w_num;
 	unsigned char h_num;
 	Tamper_params_out_t param_out;
} TD_window_result_t ;

typedef struct TD_run_s
{
    unsigned char *frame ;
    int td_result ;  // return result   
} TD_run_t ;

extern unsigned int IvaTD_GetLibVersion(unsigned int* ver);
extern int IvaTD_Init(unsigned char* working_buf_ptr, int working_buf_len, unsigned short width, unsigned short height, unsigned char color);
extern int IvaTD_Run(unsigned char* _ucImage);
extern int IvaTD_SetDetectWindow(unsigned short lt_x, unsigned short lt_y, unsigned short rb_x, unsigned short rb_y, unsigned char w_div, unsigned char h_div);
extern int IvaTD_GetDetectWindowSize(unsigned short* st_x, unsigned short* st_y, unsigned short* div_w, unsigned short* div_h);
extern int IvaTD_GetBufferSize(unsigned short width, unsigned short height, unsigned char color, unsigned char w_div, unsigned char h_div);
extern int IvaTD_GetWindowResult(unsigned char w_num, unsigned char h_num, Tamper_params_out_t* param_out);
extern int IvaTD_SetWindowEnable(unsigned char w_num, unsigned char h_num, unsigned char en);
extern int IvaTD_GetWindowEnable(unsigned char w_num, unsigned char h_num, unsigned char *en);
extern int IvaTD_SetParamsIn(Tamper_params_in_t* param);
extern void IvaTD_Release();


#ifdef __cplusplus
}
#endif

#endif /* TAMPER_H_ */
