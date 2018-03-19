#ifndef __AIT_FB_H__
#define __AIT_FB_H__

enum FB_LAYER_INDEX_TYPE {
    FB_VIDEO_LAYER   = 0x0,
    FB_DUMMY_LAYER   = 0x1,      /* RGB0 + YC2RGB */
    FB_VIDEO1_LAYER  = 0x2,      /* RGB0 + YC2RGB */
    FB_RGB_LAYER0    = 0x3,
    FB_DEFAULT_LAYER = 0x4,      /* frame buffer default layer */
};

enum FB_LAYER_ORDER_INDEX_TYPE {
    FB_ORDER_VIDEO0      = 0x0,
    FB_ORDER_VIDEO1,
    //FB_ORDER_RGB_VIDEO1,
    FB_ORDER_RGB0,
    FB_ORDER_DEF,
};

enum FB_LAYER_FORMAT_TYPE {
    /* for RGB Layer */
    FB_RGB_FORMAT_RGB565     = 0x0,
    FB_RGB_FORMAT_RGBA4444,
    FB_RGB_FORMAT_RGBA5551,
    FB_RGB_FORMAT_RGB888,
    FB_RGB_FORMAT_RGBA8888,
    FB_RGB_FORMAT_GRAY,
    FB_RGB_FORMAT_INDEX,
    FB_RGB_FORMAT_YUV422i    = 0x8,
    /* for Video Layer */
    FB_VIDEO_FORMAT_YUV420p  = 0xA,
    FB_VIDEO_FORMAT_YUV422p  = 0xB,
    FB_VIDEO_FORMAT_YUV444p  = 0xC,
    FB_VIDEO_FORMAT_YUV422v  = 0xD,
    FB_RGB_FORMAT_YUV420i    = 0xE,
};

enum FB_DISPLAY_DEV_SET_TYPE {
    FB_DISPLAY_DEV_OFF      = 0x0,
    FB_DISPLAY_DEV_ON       = 0x1,
    FB_DISPLAY_DEV_DISABLE  = 0x2,
    FB_DISPLAY_DEV_ENABLE   = 0x3,
};

struct fb_ioc_yuv_address_info {
    unsigned int Y;                     /* RGB base address */
    unsigned int U;
    unsigned int V;
    
    int is_user_addr;                   /* if used user space address , MUST set '1' */
};

struct fb_ioc_resolution_info {
    unsigned sx;
    unsigned sy;
    unsigned ex;
    unsigned ey;
    unsigned width;
    unsigned height;
};

struct fb_ioc_layer_info {
    int index;
    int format;
    struct fb_ioc_resolution_info src;
    struct fb_ioc_resolution_info dst;
    struct fb_ioc_yuv_address_info addr;
    int scale;
};

struct fb_ioc_address_info {
    int index;
    int format;
    struct fb_ioc_resolution_info pos;
    struct fb_ioc_yuv_address_info addr;
};

struct fb_ioc_position_info {
    int index;
    struct fb_ioc_resolution_info dst;
};

struct fb_ioc_order_info {
    int first;
    int second;
    int third;
    int fourth;
    //int fifth;
};

struct fb_ioc_alpha_info {
    int layer_order_index;              /* first:0 ~ fifth:4 */
    unsigned int alpha;                 /* 0x00 ~ 0xFF */
};

struct fb_ioc_chromakey_info {
    int layer_order_index;
    int format;
    unsigned int color;
};

#define FB_IOC_MAGIC                     'F'
#define FB_IOC_REQUEST_LAYER             _IOW(FB_IOC_MAGIC, 0x00, struct fb_ioc_layer_info)
#define FB_IOC_RELEASE_LAYER             _IOW(FB_IOC_MAGIC, 0x01, int)
#define FB_IOC_SET_LAYER_ADDRESS         _IOW(FB_IOC_MAGIC, 0x04, struct fb_ioc_address_info)
#define FB_IOC_SET_LAYER_POSITION        _IOW(FB_IOC_MAGIC, 0x06, struct fb_ioc_position_info)
/* default order: video0 -> video1 -> RGB0 -> RGB1 -> FB_DEF */
#define FB_IOC_SET_LAYER_ORDER           _IOW(FB_IOC_MAGIC, 0x10, struct fb_ioc_order_info)
#define FB_IOC_SET_LAYER_ALPHABLEND      _IOW(FB_IOC_MAGIC, 0x12, struct fb_ioc_alpha_info)
#define FB_IOC_GET_LAYER_ALPHABLEND      _IOW(FB_IOC_MAGIC, 0x13, int)                                    /* int : layer_order_index */
#define FB_IOC_SET_LAYER_CHROMAKEY       _IOW(FB_IOC_MAGIC, 0x14, struct fb_ioc_chromakey_info)

#define FB_IOC_SET_DISPLAY_DEV           _IOW(FB_IOC_MAGIC, 0x20, int)

#endif
