#ifndef __AIT_FB_FOPS_H__
#define __AIT_FB_FOPS_H__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <video/ait_fb.h>
#include <mach/mmpf_display.h>
#include <mach/mmpf_graphics.h>
#include <mach/lcd_common.h>

struct ait_fb_display_device_info {
    const char *name;
    //unsigned short width;
    //unsigned short height;

    bool boot_on;

    int (*init)(struct device *dev);
    int (*enable)(struct device *dev);
    int (*disable)(struct device *dev);
    int (*close)(struct device *dev);
    int (*test)(unsigned int phys, unsigned int virt);

    MMPF_PANEL_ATTR *panel;
};

struct ait_fb_platform_data_info {
    struct {
        unsigned char format;
        unsigned short width;
        unsigned short height;
        unsigned char rot_angle;
    }fb;

    struct ait_fb_display_device_info *disp;
};

struct ait_fb_layer_info {
    bool enable;
    int index;
    //struct du_resolution_info pos;
    //struct du_image_info image;

    bool en_alpha_unit;
    unsigned char alpha;
    unsigned int chromakey;

    MMPF_GRAPHICS_BUF_ATTR bufAttr;
    MMPF_DISPLAY_DISP_ATTR dispAttr;
};


struct ait_fb_info {
	struct device *dev;
	spinlock_t lock;
	
	struct ait_fb_layer_info layers[MMPF_DISPLAY_WIN_MAX];
	struct ait_fb_platform_data_info *plat;
};

static inline int _checkIoctlData(unsigned int cmd, unsigned long arg)
{
    int ret = 0;

	if (_IOC_TYPE(cmd) != FB_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (ret)
		return -EFAULT;

    return 0;
}

int fb_init_frambuffer_info(struct fb_info *fbinfo, struct ait_fb_platform_data_info *data);
int fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbinfo);
int fb_set_par(struct fb_info *fbinfo);
int fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *fbinfo);
int fb_mmap(struct fb_info *fbinfo, struct vm_area_struct *vma);
void fb_free_frame_buffer_info(struct fb_info *fbinfo);

int fb_create_mix_layer(struct fb_info *fbinfo, struct fb_ioc_layer_info *layer);
int fb_free_mix_layer(struct fb_info *fbinfo, int layer_index);
int fb_set_mix_layer_address(struct fb_info *fbinfo, struct fb_ioc_address_info *addr);
int fb_set_mix_layer_position(struct fb_info *fbinfo, struct fb_ioc_position_info *pos);
int fb_set_mix_layer_order(struct fb_info *fbinfo, struct fb_ioc_order_info *order);
int fb_set_mix_layer_alphablend(struct fb_info *fbinfo, struct fb_ioc_alpha_info *alpha);
int fb_get_mix_layer_alphablend(struct fb_info *fbinfo, int order);
int fb_set_mix_layer_chromakey(struct fb_info *fbinfo, struct fb_ioc_chromakey_info *chroma);

int fb_setup_display_screen_block(struct fb_info *fbinfo, int target, int trig_src, unsigned int base_address, unsigned char burst_size);
int fb_update_display_screen_block(struct fb_info *fbinfo);
int fb_close_display_screen_block(struct fb_info *fbinfo);

int fb_refresh_display_screen_block(struct fb_info *fbinfo);


#endif
