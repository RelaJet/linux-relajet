/*
 * aac.h
 *
 *  Created on: 2015/09/14,based on aac.h
 *      Author: sean
 *
 */

#ifndef AES_HPP_
#define AES_HPP_

#ifdef __cplusplus
extern "C" {
#endif
//#include "../cpub_ringbuf.h"
#define AIT_AES_LIB_VER	(20151001)

typedef enum
{
    AES_INIT,
    AES_ENCODE
} AES_command_in_t;


typedef struct AES_init_s
{
	void *priv ;
} AES_init_t ;

typedef struct AES_proc_info_s
{
    unsigned char *in; 
    unsigned char *out ;
    int len ; 
    //AES_KEY  *key  ;
    void    *key ;
    unsigned char *iv ;    
} AES_proc_info_t ;

extern int AES_init(void);
extern int AES_run(unsigned char *aes_frame,unsigned short samples);

#ifdef __cplusplus
}
#endif

#endif /* MD_HPP_ */
