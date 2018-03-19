#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
//#include <linux/string.h>
#include "audio-ring-buf.h"

int audio_ring_buf_init(audio_ring_buffer* ring_buf,int buf_size)
{
	ring_buf->write_ptr = 0;
	ring_buf->read_ptr = 0;
	ring_buf->buf_size_in_sample = buf_size;
	ring_buf->data = kmalloc(buf_size*sizeof(*ring_buf->data),GFP_KERNEL);
	ring_buf->count = 0;
}
EXPORT_SYMBOL(audio_ring_buf_init);

void audio_ring_buf_release(audio_ring_buffer* ring_buf)
{
	if(ring_buf->data)
	{
		kfree(ring_buf->data);
		ring_buf->data = 0;
	}
}


int audio_ring_buf_write(audio_ring_buffer* ring_buf, short* src, int src_size)
{
	int n;
	int n_to_write;
	//unsigned long flag;
	//spinlock_t lock;
	
	//spin_lock_init(&lock);
	//spin_lock_irqsave(&lock, flag);
	
	src_size = min(ring_buf->buf_size_in_sample-ring_buf->count,src_size);
	n_to_write = min(ring_buf->buf_size_in_sample - ring_buf->write_ptr,src_size);
	memcpy(&ring_buf->data[ring_buf->write_ptr],src,n_to_write*sizeof(short));
	n = n_to_write;
	ring_buf->write_ptr += n_to_write;
	
	if(src_size-n_to_write>0)
	{
		ring_buf->write_ptr = 0;
		n_to_write = src_size-n_to_write;
		memcpy(&ring_buf->data[ring_buf->write_ptr],&src[n],n_to_write*sizeof(short));
		n += n_to_write;
		ring_buf->write_ptr += n_to_write;		
	}

	ring_buf->count += src_size;
	//spin_unlock_irqrestore(&lock, flag);
	return n_to_write;
}
EXPORT_SYMBOL(audio_ring_buf_write);

int audio_ring_buf_read(audio_ring_buffer* ring_buf, short* dest, int dest_size)
{
	int n;
	int n_to_read;
	//unsigned long flag;
	//spinlock_t lock;
	
	//spin_lock_init(&lock);
	//spin_lock_irqsave(&lock, flag);
	
	dest_size = min(ring_buf->buf_size_in_sample-ring_buf->count,dest_size);
	n_to_read = min(ring_buf->buf_size_in_sample - ring_buf->read_ptr,dest_size);
	
	memcpy(dest, &ring_buf->data[ring_buf->read_ptr], n_to_read*sizeof(short));
	n = n_to_read;
	ring_buf->read_ptr += n_to_read;
		
	if(dest_size-n_to_read>0)
	{
		ring_buf->read_ptr = 0;
		n_to_read=dest_size-n_to_read;
		memcpy(&dest[n], &ring_buf->data[ring_buf->read_ptr], n_to_read*sizeof(short));
		n += n_to_read;
		ring_buf->read_ptr += n_to_read;		
	}

	ring_buf->count -= dest_size;
	//spin_unlock_irqrestore(&lock, flag);
	return dest_size;
}
EXPORT_SYMBOL(audio_ring_buf_read);

void audio_ring_reset(audio_ring_buffer* ring_buf)
{
	ring_buf->count = 0;
	ring_buf->read_ptr = 0;
	ring_buf->write_ptr = 0;
}

inline int audio_ring_get_count(audio_ring_buffer* ring_buf)
{
	return ring_buf->count;
}
EXPORT_SYMBOL(audio_ring_get_count);