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
#include <asm/dev-tool.h>
#include <video/ait_fb_fops.h>

static unsigned char calc_bit_per_pixel(unsigned char format)
{
    unsigned char bpp = 0;

    switch (format) {
    case DU_IMAGE_FORMAT_Gray:
    case DU_IMAGE_FORMAT_Index:
    case DU_IMAGE_FORMAT_YUV420P:
    case DU_IMAGE_FORMAT_YUV422P:
    case DU_IMAGE_FORMAT_YUV444P:
        bpp = 8;    break;
    case DU_IMAGE_FORMAT_RGB565:
    case DU_IMAGE_FORMAT_RGBA4444:
    case DU_IMAGE_FORMAT_RGBA5551:
    case DU_IMAGE_FORMAT_YUV420I:
    case DU_IMAGE_FORMAT_YUV422I:
    case DU_IMAGE_FORMAT_YUV422V:
    case DU_IMAGE_FORMAT_YUV422IR:
        bpp = 16;   break;
    case DU_IMAGE_FORMAT_RGB888:
    case DU_IMAGE_FORMAT_YUV444I:
        bpp = 32;   break;
    case DU_IMAGE_FORMAT_RGBA8888:
        bpp = 64;   break;
    }

    return bpp;
}

static __inline void _set_RGBA_bitfield(struct fb_var_screeninfo *var)
{
    switch (var->bits_per_pixel) {
    case 1:
    case 2:
    case 4:
    case 8:
        var->red.length 	= var->bits_per_pixel;
        var->green.length 	= var->bits_per_pixel;
        var->blue.length 	= var->bits_per_pixel;
        var->transp.length 	= 0;
        
        var->red.offset 	= 0;
        var->green.offset 	= 0;
        var->blue.offset 	= 0;
        var->transp.offset 	= 0;
        break;
    case 16:
        /* 16bpp : RGB565 */
        var->red.length 	= 5;
        var->green.length 	= 6;
        var->blue.length 	= 5;
        var->transp.length 	= 0;
        
        var->red.offset 	= var->green.length + var->blue.length;
        var->green.offset 	= var->blue.length;
        var->blue.offset 	= 0;
        var->transp.offset 	= 0;
        break;
    case 24:
    default:
        /* 24bpp : RGB888 */
        var->red.length 	= 8;
        var->green.length 	= 8;
        var->blue.length 	= 8;
        var->transp.length 	= 0;
        
        var->red.offset 	= var->green.length + var->blue.length;
        var->green.offset 	= var->blue.length;
        var->blue.offset 	= 0;
        var->transp.offset 	= 0;
        break;
    }
}

static __inline int _allocate_video_memory_map(struct fb_info *fbinfo)
{
    struct ait_fb_info *fbi = fbinfo->par;

    fbinfo->screen_base = dma_alloc_writecombine(fbi->dev, fbinfo->fix.smem_len, (dma_addr_t *)&fbinfo->fix.smem_start, GFP_KERNEL);
    if (fbinfo->screen_base == NULL) {
		dev_err(fbi->dev, "fail to dma_alloc_writecombine, not enough dma area\n");
		return -ENOMEM;
    }
    fbinfo->screen_size = fbinfo->fix.smem_len;
	
    dev_info(fbi->dev, "map_video_memory: dma=0x%lx cpu=%p size=0x%lx\n", fbinfo->fix.smem_start, fbinfo->screen_base, fbinfo->screen_size);

    return 0;
}

int fb_init_frambuffer_info(struct fb_info *fbinfo, struct ait_fb_platform_data_info *data)
{
    struct ait_fb_info *fbi;
    int ret;

    if (!fbinfo || !data) { return -EINVAL; }
    fbi = fbinfo->par;

    fbinfo->flags 		= FBINFO_FLAG_DEFAULT;
    /* set frame buffer variable information */
    fbinfo->var.xres 		= data->fb.width;
    fbinfo->var.xres_virtual    = data->fb.width;
    fbinfo->var.width           = -1;
    fbinfo->var.yres 		= data->fb.height;
    fbinfo->var.yres_virtual 	= data->fb.height;
    fbinfo->var.height		= -1;
    fbinfo->var.bits_per_pixel 	= calc_bit_per_pixel(data->fb.format);
    fbinfo->var.nonstd	        = 0;
    fbinfo->var.activate	= FB_ACTIVATE_NOW;
    fbinfo->var.accel_flags     = 0;
    fbinfo->var.vmode	        = FB_VMODE_NONINTERLACED;
    _set_RGBA_bitfield(&fbinfo->var);
    
    /* set frame buffer fixed information */
    strcpy(fbinfo->fix.id, data->disp->name);
    fbinfo->fix.type 		= FB_TYPE_PACKED_PIXELS;
    fbinfo->fix.visual 	    	= FB_VISUAL_TRUECOLOR;
    fbinfo->fix.type_aux        = 0;
    fbinfo->fix.xpanstep        = 0;
    fbinfo->fix.ypanstep	= 0;
    fbinfo->fix.ywrapstep	= 0;
    fbinfo->fix.line_length	= data->fb.width * (fbinfo->var.bits_per_pixel >> 3);
    fbinfo->fix.accel	        = FB_ACCEL_NONE;
    fbinfo->fix.smem_len	= PAGE_ALIGN(data->fb.width * data->fb.height * (fbinfo->var.bits_per_pixel >> 3));

    ret = _allocate_video_memory_map(fbinfo);
    if (ret) { return ret; }

    fbinfo->pseudo_palette	= kmalloc(sizeof(unsigned int) * 16, GFP_KERNEL);
    if (fbinfo->pseudo_palette == NULL) {
		dev_err(fbi->dev, "fail to kmalloc, not enough memory\n");
		return -ENOMEM;
    }

    spin_lock_init(&fbi->lock);

    return 0;
}
EXPORT_SYMBOL(fb_init_frambuffer_info);

int fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbinfo)
{
	struct ait_fb_info *fbi = fbinfo->par;

	_set_RGBA_bitfield(var);

	return 0;
}
EXPORT_SYMBOL(fb_check_var);

int fb_set_par(struct fb_info *fbinfo)
{
	struct ait_fb_info *fbi = fbinfo->par;
	
	switch (fbinfo->var.bits_per_pixel) {
	case 16:
		fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
		fbinfo->fix.visual = FB_VISUAL_MONO01;
		break;
	default:
		fbinfo->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}

	fbinfo->fix.line_length = fbinfo->var.xres_virtual * (fbinfo->var.bits_per_pixel >> 3);

	return 0;
}
EXPORT_SYMBOL(fb_set_par);

static __inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{	
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

int fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *fbinfo)
{
	struct ait_fb_info *fbi = fbinfo->par;
	unsigned int val;
	unsigned int *pal = fbinfo->pseudo_palette;

	switch (fbinfo->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			val  = chan_to_field(red,   &fbinfo->var.red);
			val |= chan_to_field(green, &fbinfo->var.green);
			val |= chan_to_field(blue,  &fbinfo->var.blue);

			pal[regno] = val;
		}
		break;
	case FB_VISUAL_PSEUDOCOLOR:
		/* not supported */
	default:
		/* unknown type */
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL(fb_setcolreg);

int fb_mmap(struct fb_info *fbinfo, struct vm_area_struct *vma)
{
	struct ait_fb_info *fbi = fbinfo->par;
	unsigned long len, off = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	len = fbinfo->fix.smem_len;
	if (off > len) {
        return -EINVAL;
    }

	ret = dma_mmap_writecombine(fbi->dev, vma, fbinfo->screen_base, fbinfo->fix.smem_start, fbinfo->fix.smem_len);
	if (ret < 0) {
		dev_err(fbi->dev, "fail to dma_mmap_writecombine (ret=%d)\n", ret); 
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(fb_mmap);

void fb_free_frame_buffer_info(struct fb_info *fbinfo)
{
    struct ait_fb_info *fbi = fbinfo->par;
    
    kfree(fbinfo->pseudo_palette);
    dma_free_writecombine(fbi->dev, fbinfo->fix.smem_len, fbinfo->screen_base, fbinfo->fix.smem_start);
}
EXPORT_SYMBOL(fb_free_frame_buffer_info);

static int _get_image_phys_addr_info(MMPF_GRAPHICS_BUF_ATTR  *bufAttr, struct fb_ioc_yuv_address_info *src, int format, unsigned int size)
{
    unsigned long phys_addr;

    if (src->is_user_addr && format >= FB_RGB_FORMAT_YUV422i) {
		phys_addr = user_to_phys(src->Y);
		
			bufAttr->ulBaseAddr = phys_addr;
            switch (format) {
            case FB_VIDEO_FORMAT_YUV420p:
				bufAttr->ulBaseUAddr = bufAttr->ulBaseAddr + (size);
				bufAttr->ulBaseVAddr = bufAttr->ulBaseUAddr + (size) / 4;
                break;
            case FB_VIDEO_FORMAT_YUV422p:
            case FB_VIDEO_FORMAT_YUV422v:
				bufAttr->ulBaseUAddr = bufAttr->ulBaseAddr + (size);
				bufAttr->ulBaseVAddr = bufAttr->ulBaseUAddr + (size) / 2;
                break;
            case FB_VIDEO_FORMAT_YUV444p:
				bufAttr->ulBaseUAddr = bufAttr->ulBaseAddr + (size);
				bufAttr->ulBaseVAddr = bufAttr->ulBaseUAddr + (size);
                break;
            }
		
    } else {
        bufAttr->ulBaseAddr = src->Y;
		bufAttr->ulBaseUAddr = src->U;
		bufAttr->ulBaseVAddr = src->V;
    }

    return 0;
}

/** 
 *  _detect_matched_layer_slot - detect matched layer slot
 *  @ mix - image control info structure point
 **/ 
static int _detect_matched_layer_slot(struct ait_fb_info *fbi, int index)
{
	int layer = -1;

	
	switch(index)
	{
		case FB_VIDEO_LAYER:	// main video layer
			layer = MMPF_DISPLAY_WIN_OVERLAY;
			break;
		case FB_DUMMY_LAYER: // pip video layer
		case FB_VIDEO1_LAYER: // pip video layer
			layer = MMPF_DISPLAY_WIN_PIP;
			break;
		case FB_RGB_LAYER0:		// rgb & osd layer
			layer = MMPF_DISPLAY_WIN_OSD;
			break;
		case FB_DEFAULT_LAYER: // frame buffer default layer
			layer = MMPF_DISPLAY_WIN_MAIN;
			break;
	}

    return layer;
}

int fb_free_mix_layer(struct fb_info *fbinfo, int layer_index)
{
	struct ait_fb_info *fbi = fbinfo->par;
	unsigned long flags;
	int index;

	index = _detect_matched_layer_slot(fbi, layer_index);
	if (index < 0) {
		dev_err(fbi->dev, "fail to create mix layer, can't detect usable layer slot, (ret=%d)", index);
		return index;	
	}

	local_irq_save(flags);
	memset(&fbi->layers[index], 0x0, sizeof(struct ait_fb_layer_info));
	MMPF_Display_SetWinActive(index, 0);
	fbi->layers[index].enable = 0;
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(fb_free_mix_layer);

static int _convert_order_to_prio(int order)
{
	MMPF_DISPLAY_WINID prio = 0;

	switch (order)
	{
		case FB_ORDER_VIDEO0:
			prio = MMPF_DISPLAY_WIN_OVERLAY;
			break;
		case FB_ORDER_VIDEO1:
			prio = MMPF_DISPLAY_WIN_PIP;
			break;
		case FB_ORDER_RGB0:
			prio = MMPF_DISPLAY_WIN_OSD;
			break;
		case FB_ORDER_DEF:
			prio = MMPF_DISPLAY_WIN_MAIN;
			break;
	}

	return prio;
}

int fb_set_mix_layer_order(struct fb_info *fbinfo, struct fb_ioc_order_info *order)
{
	struct ait_fb_info *fbi = fbinfo->par;
	unsigned long flags;
	MMPF_DISPLAY_WINID prio1, prio2, prio3, prio4;

	local_irq_save(flags);
	
	prio1 = _convert_order_to_prio(order->first);
	prio2 = _convert_order_to_prio(order->second);
	prio3 = _convert_order_to_prio(order->third);
	prio4 = _convert_order_to_prio(order->fourth);
	
	MMPF_Display_SetWinPriority(prio1, prio2, prio3, prio4);
	
	local_irq_restore(flags);
	
	//PRINTF(" <<%s>> prio1: %d   prio2: %d  prio3: %d  prio4: %d \n", __func__, prio1, prio2, prio3, prio4);
	return 0;
}
EXPORT_SYMBOL(fb_set_mix_layer_order);

static void _link_address_info_to_buf_attr(MMPF_GRAPHICS_BUF_ATTR *bufAttr, struct fb_ioc_address_info *addr, unsigned char rot)
{
#if 0 //djkim.rot
	if ((rot == DU_STRP_ROT_90) || (rot == DU_STRP_ROT_270)) {
	    bufAttr->usWidth = addr->pos.height;
	    bufAttr->usHeight = addr->pos.width;
	} else {
	    bufAttr->usWidth = addr->pos.width;
	    bufAttr->usHeight = addr->pos.height;
	}
#else
	bufAttr->usWidth = addr->pos.width;
	bufAttr->usHeight = addr->pos.height;
#endif

	switch (addr->format) 
	{
	    case FB_RGB_FORMAT_RGB565:
	        bufAttr->usLineOffset = ALIGN32(bufAttr->usWidth*2);
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_16;
	        break;
	    case FB_RGB_FORMAT_RGB888:
	        bufAttr->usLineOffset = bufAttr->usWidth*3;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_24;
	        break;
	    case FB_VIDEO_FORMAT_YUV422p:
	        bufAttr->usLineOffset = bufAttr->usWidth*2;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV422;
	        break;
	    case FB_VIDEO_FORMAT_YUV420p:
	        bufAttr->usLineOffset = bufAttr->usWidth;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV420;
	        break;
	    case FB_RGB_FORMAT_YUV420i:
	        bufAttr->usLineOffset = bufAttr->usWidth;
            bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV420_INTERLEAVE;
	        break;
    }
	_get_image_phys_addr_info(bufAttr, &addr->addr, addr->format, addr->pos.width * addr->pos.height);

	//PRINTF(" <<%s - 1>> W:%d H:%d Line:%d Depth:%d Y:0x%x U:0x%x V:0x%x \n", __func__, bufAttr->usWidth, bufAttr->usHeight, bufAttr->usLineOffset, bufAttr->colordepth, bufAttr->ulBaseAddr, bufAttr->ulBaseUAddr, bufAttr->ulBaseVAddr);
}

static void _link_address_info_to_disp_attr(MMPF_DISPLAY_DISP_ATTR *dispAttr, struct fb_ioc_address_info *addr, unsigned char rot)
{		
#if 0
	dispAttr->usStartX = 0;
	dispAttr->usStartY = 0;   
	dispAttr->usDisplayOffsetX = (addr->pos.width > ulRotateW) ? (addr->pos.width - ulRotateW) >> 1 : dispAttr.usStartX;
	dispAttr->usDisplayOffsetY = (addr->pos.height > ulRotateH) ? (addr->pos.height - ulRotateH) >> 1 : dispAttr.usStartY;   
	dispAttr->bMirror = 0;
#else
	dispAttr->usStartX = 0;
	dispAttr->usStartY = 0;
	dispAttr->usDisplayOffsetX = addr->pos.sx;
	dispAttr->usDisplayOffsetY = addr->pos.sy;	 
	dispAttr->bMirror = 0;
#endif

	switch (rot) 
	{
		case DU_STRP_ROT_0:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_NO_ROTATE;
			dispAttr->usDisplayWidth	= addr->pos.width;
			dispAttr->usDisplayHeight	= addr->pos.height;
			break;
		case DU_STRP_ROT_90:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_90;
			dispAttr->usDisplayWidth	= addr->pos.height;
			dispAttr->usDisplayHeight	= addr->pos.width;
			break;
		case DU_STRP_ROT_180:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_180;
			dispAttr->usDisplayWidth	= addr->pos.width;
			dispAttr->usDisplayHeight	= addr->pos.height;
			break;
		case DU_STRP_ROT_270:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_270;
			dispAttr->usDisplayWidth	= addr->pos.height;
			dispAttr->usDisplayHeight	= addr->pos.width;
			break;
	}

	//PRINTF(" <<%s - 1>> X:%d, Y:%d, W:%d, H:%d ROT:%d \n", __func__, dispAttr->usDisplayOffsetX, dispAttr->usDisplayOffsetY, dispAttr->usDisplayWidth, dispAttr->usDisplayHeight, dispAttr->rotatetype);
}

int fb_set_mix_layer_address(struct fb_info *fbinfo, struct fb_ioc_address_info *addr)
{
	struct ait_fb_info *fbi = fbinfo->par;
	unsigned long flags;
	int index;
	
	index = _detect_matched_layer_slot(fbi, addr->index);
	if (index < 0) {
		dev_err(fbi->dev, "fail to set address of mix layer, can't detect matched layer slot, (index=%d)", addr->index);
		return index;
	}
	
	local_irq_save(flags);
	_link_address_info_to_buf_attr(&fbi->layers[index].bufAttr, addr, fbi->plat->fb.rot_angle);
	MMPF_Display_UpdateWinAddr(index, fbi->layers[index].bufAttr.ulBaseAddr, fbi->layers[index].bufAttr.ulBaseUAddr, fbi->layers[index].bufAttr.ulBaseVAddr);	
	local_irq_restore(flags);
	
	return 0;
}
EXPORT_SYMBOL(fb_set_mix_layer_address);

static void _link_layer_info_to_buf_attr(MMPF_GRAPHICS_BUF_ATTR *bufAttr, struct fb_ioc_layer_info *layer, unsigned char rot)
{
#if 0 //djkim.rot
	if ((rot == DU_STRP_ROT_90) || (rot == DU_STRP_ROT_270)) {
	    bufAttr->usWidth = layer->src.height;
	    bufAttr->usHeight = layer->src.width;
	} else {
	    bufAttr->usWidth = layer->src.width;
	    bufAttr->usHeight = layer->src.height;
	}
#else
	bufAttr->usWidth = layer->src.width;
	bufAttr->usHeight = layer->src.height;
#endif

	switch (layer->format) 
	{
	    case FB_RGB_FORMAT_RGB565:
	        bufAttr->usLineOffset = ALIGN32(bufAttr->usWidth*2);
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_16;
	        break;
	    case FB_RGB_FORMAT_RGB888:
	        bufAttr->usLineOffset = bufAttr->usWidth*3;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_24;
	        break;
	    case FB_VIDEO_FORMAT_YUV422p:
	        bufAttr->usLineOffset = bufAttr->usWidth*2;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV422;
	        break;
	    case FB_VIDEO_FORMAT_YUV420p:
	        bufAttr->usLineOffset = bufAttr->usWidth;
	        bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV420;
	        break;
	    case FB_RGB_FORMAT_YUV420i:
	        bufAttr->usLineOffset = bufAttr->usWidth;
            bufAttr->colordepth = MMPF_GRAPHICS_COLORDEPTH_YUV420_INTERLEAVE;
	        break;
    }
	_get_image_phys_addr_info(bufAttr, &layer->addr, layer->format, layer->src.width * layer->src.height);

	//PRINTF(" <<%s - 1>> W:%d H:%d Line:%d Depth:%d Y:0x%x U:0x%x V:0x%x \n", __func__, bufAttr->usWidth, bufAttr->usHeight, bufAttr->usLineOffset, bufAttr->colordepth, bufAttr->ulBaseAddr, bufAttr->ulBaseUAddr, bufAttr->ulBaseVAddr);
}

static void _link_layer_info_to_disp_attr(MMPF_DISPLAY_DISP_ATTR *dispAttr, struct fb_ioc_layer_info *layer, unsigned char rot)
{	
#if 0 //djkim.2015.03.18
	dispAttr->usStartX = layer->dst.sx;
	dispAttr->usStartY = layer->dst.sy;
	dispAttr->usDisplayOffsetX = dispAttr->usStartX;
	dispAttr->usDisplayOffsetY = dispAttr->usStartY;	 
	dispAttr->bMirror = 0;
#else
	dispAttr->usStartX = 0;
	dispAttr->usStartY = 0;
	dispAttr->usDisplayOffsetX = layer->dst.sx;
	dispAttr->usDisplayOffsetY = layer->dst.sy;	 
	dispAttr->bMirror = 0;
#endif

	switch (rot) 
	{
		case DU_STRP_ROT_0:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_NO_ROTATE;
			dispAttr->usDisplayWidth	= layer->dst.width;
			dispAttr->usDisplayHeight	= layer->dst.height;
			break;
		case DU_STRP_ROT_90:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_90;
			dispAttr->usDisplayWidth	= layer->dst.height;
			dispAttr->usDisplayHeight	= layer->dst.width;
			break;
		case DU_STRP_ROT_180:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_180;
			dispAttr->usDisplayWidth	= layer->dst.width;
			dispAttr->usDisplayHeight	= layer->dst.height;
			break;
		case DU_STRP_ROT_270:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_270;
			dispAttr->usDisplayWidth	= layer->dst.height;
			dispAttr->usDisplayHeight	= layer->dst.width;
			break;
	}

	//PRINTF(" <<%s - 1>> X:%d, Y:%d, W:%d, H:%d ROT:%d \n", __func__, dispAttr->usDisplayOffsetX, dispAttr->usDisplayOffsetY, dispAttr->usDisplayWidth, dispAttr->usDisplayHeight, dispAttr->rotatetype);
}

static void _link_res_info_to_disp_attr(MMPF_DISPLAY_DISP_ATTR *dispAttr, struct fb_ioc_resolution_info *res, unsigned char rot)
{
	switch (rot) 
	{
		case DU_STRP_ROT_0:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_NO_ROTATE;
			dispAttr->usDisplayWidth	= res->width;
			dispAttr->usDisplayHeight	= res->height;
			break;
		case DU_STRP_ROT_90:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_90;
			dispAttr->usDisplayWidth	= res->height;
			dispAttr->usDisplayHeight	= res->width;
			break;
		case DU_STRP_ROT_180:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_180;
			dispAttr->usDisplayWidth	= res->width;
			dispAttr->usDisplayHeight	= res->height;
			break;
		case DU_STRP_ROT_270:
			dispAttr->rotatetype = MMPF_DISPLAY_ROTATE_RIGHT_270;
			dispAttr->usDisplayWidth	= res->height;
			dispAttr->usDisplayHeight	= res->width;
			break;
	}

	//PRINTF(" <<%s - 1>> X:%d, Y:%d, W:%d, H:%d ROT:%d \n", __func__, dispAttr->usDisplayOffsetX, dispAttr->usDisplayOffsetY, dispAttr->usDisplayWidth, dispAttr->usDisplayHeight, dispAttr->rotatetype);
}

int fb_create_mix_layer(struct fb_info *fbinfo, struct fb_ioc_layer_info *layer)
{
	struct ait_fb_info *fbi = fbinfo->par;
	MMPF_SCAL_FIT_RANGE fitrange;
	unsigned long flags;
	int index;

	index = _detect_matched_layer_slot(fbi, layer->index);
	if (index < 0) {
		dev_err(fbi->dev, "fail to create mix layer, can't detect usable layer slot, (ret=%d)", index);
		return index;	
	}
	
	if (fbi->layers[index].enable) {		
		dev_err(fbi->dev, "fail to create mix layer, already used, (ret=%d)", index);
		return index;
	}

	local_irq_save(flags);
	memset(&fbi->layers[index], 0x0, sizeof(struct ait_fb_layer_info));	
	_link_layer_info_to_buf_attr(&fbi->layers[index].bufAttr, layer, fbi->plat->fb.rot_angle);
#if 0 //djkim.2015.03.18
	_link_layer_info_to_disp_attr(&fbi->layers[index].dispAttr, layer, fbi->plat->fb.rot_angle);
#else
	fbi->layers[index].dispAttr.usStartX = 0;
	fbi->layers[index].dispAttr.usStartY = 0;
	fbi->layers[index].dispAttr.usDisplayOffsetX = layer->dst.sx;
	fbi->layers[index].dispAttr.usDisplayOffsetY = layer->dst.sy;	 
	fbi->layers[index].dispAttr.bMirror = 0;

	if (layer->scale)
		_link_res_info_to_disp_attr(&fbi->layers[index].dispAttr, &layer->src, fbi->plat->fb.rot_angle);
	else
		_link_res_info_to_disp_attr(&fbi->layers[index].dispAttr, &layer->dst, fbi->plat->fb.rot_angle);
#endif

	MMPF_Display_BindBufToWin(&fbi->layers[index].bufAttr, index);
	MMPF_Display_SetWinToDisplay(index, &fbi->layers[index].dispAttr);

#if 0 //djkim.2015.03.18
	fitrange.fitmode		= MMPF_SCAL_FITMODE_OUT;
	fitrange.ulFitResol 	= 64;
	fitrange.ulInWidth		= layer->dst.width;
	fitrange.ulInHeight 	= layer->dst.height;
	fitrange.ulInGrabX		= 1;
	fitrange.ulInGrabY		= 1;
	fitrange.ulInGrabW		= fitrange.ulInWidth;
	fitrange.ulInGrabH		= fitrange.ulInHeight;	
	fitrange.ulOutWidth 	= layer->dst.width;
	fitrange.ulOutHeight	= layer->dst.height;	
	MMPF_Display_SetWinScaling(index, MMP_FALSE, MMP_TRUE, &fitrange, NULL);
#else
	if (layer->scale) {
		fitrange.fitmode		= MMPF_SCAL_FITMODE_OUT;
		fitrange.ulFitResol 	= 64;
		fitrange.ulInWidth		= layer->src.width;
		fitrange.ulInHeight 	= layer->src.height;
		fitrange.ulInGrabX		= 1;
		fitrange.ulInGrabY		= 1;
		fitrange.ulInGrabW		= fitrange.ulInWidth;
		fitrange.ulInGrabH		= fitrange.ulInHeight;	
		fitrange.ulOutWidth 	= layer->dst.width;
		fitrange.ulOutHeight	= layer->dst.height;

		//PRINTF("<<SCALE>> %d %d %d %d \n", fitrange.ulInWidth, fitrange.ulInHeight, fitrange.ulOutWidth, fitrange.ulOutHeight);
		MMPF_Display_SetWinScaling(index, MMP_FALSE, MMP_TRUE, &fitrange, NULL);
	}
#endif

	MMPF_Display_SetWinActive(index, 1);
	fbi->layers[index].enable = 1;
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(fb_create_mix_layer);

int fb_refresh_display_screen_block(struct fb_info *fbinfo)
{
	struct ait_fb_info *fbi = fbinfo->par;
	int index;
	
	for (index = 0; index < MMPF_DISPLAY_WIN_MAX; index++)
	{
		if (fbi->layers[index].enable)
		{	
			MMPF_Display_SetDisplayRefresh(MMPF_DISPLAY_PRM_CTL);
			break;
		}
	}
	
	return 0;
}
EXPORT_SYMBOL(fb_refresh_display_screen_block);
