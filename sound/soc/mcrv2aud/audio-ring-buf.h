#ifndef _AUDIO_RING_BUF_H_
#define _AUDIO_RING_BUF_H_

typedef struct  audio_ring_buffer_
{
	int read_ptr;
	int write_ptr;
	int buf_size_in_sample;
	short *data;
	int count;
}audio_ring_buffer;

int audio_ring_buf_init(audio_ring_buffer* ring_buf,int buf_size);
void audio_ring_buf_release(audio_ring_buffer* ring_buf);
int audio_ring_buf_write(audio_ring_buffer* ring_buf, short* src, int src_size);
int audio_ring_buf_read(audio_ring_buffer* ring_buf, short* dest, int dest_size);
int audio_ring_get_count(audio_ring_buffer* ring_buf);

#endif 
