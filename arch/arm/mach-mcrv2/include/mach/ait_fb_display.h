/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#ifndef __AIT_FB_DISPLAY_H__
#define __AIT_FB_DISPLAY_H__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/ioctl.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_display.h>
#include <mach/mmp_reg_pad.h>
#include <mach/mmpf_system.h>

#define DEVICE_NAME_FB			"ait_fb_display"

//display output type
#define DIS_OUT_TYPE_NTSC		0
#define DIS_OUT_TYPE_PAL		1

//display windows
#define DIS_WIN_MAIN			(1 << 0)
#define DIS_WIN_PIP			(1 << 1)
#define DIS_WIN_OVLY			(1 << 2)
#define DIS_WIN_ICON			(1 << 3)
 
struct ait_fb_display_info {
	struct fb_info		fb;
	struct platform_device	*dev;
	AITPS_DSPY		fb_reg;
	AITPS_TV		tv_reg;
	unsigned int 		dis_output_type;
	unsigned int 		dis_output_format;
	unsigned int		dis_w_enable;
	unsigned int 		dis_w_format;
	unsigned int 		dis_w_pixel_size;
	struct timer_list	timer;
};

#define AITFBIOPUT_LAYERENABLE	_IOW('F', 0x80, int)
#define AITFBIOPUT_TPENABLE	_IOW('F', 0x81, int)

#endif

