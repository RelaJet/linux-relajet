/*
 * aac.h
 *
 *  Created on: 2015/09/14,based on md.h
 *      Author: sean
 *
 */

#ifndef AAC_HPP_
#define AAC_HPP_

#ifdef __cplusplus
extern "C" {
#endif
//#include "../cpub_ringbuf.h"
#define AIT_AAC_LIB_VER	(20150911)

typedef enum {
  ADTS_HEADER_ON = 0 ,
  AAC_ACK_OFF,
  AAC_OPT_MAX  
} AAC_OPTIONS ;

typedef struct _aac_options_s
{
  // bit flag
  unsigned long options ;     
} aac_options_t ;

typedef enum
{
    AAC_INIT,
    AAC_ENCODE
} AAC_command_in_t;


typedef struct AAC_init_s
{
	unsigned short channel ;
	unsigned short samplerate ;
	unsigned long bitrate ;
	void *priv ;
} AAC_init_t ;

typedef struct AAC_proc_info_s
{
    unsigned char *out_frame; // out buffer
    unsigned char *frame ; //in buffer
    unsigned short samples ;
} AAC_proc_info_t ;

extern int AAC_init(unsigned short channel,unsigned short samplerate, unsigned short bitrate);
extern int AAC_run(unsigned char* _ucImage,unsigned short samples);

#ifdef __cplusplus
}
#endif

#endif /* MD_HPP_ */
