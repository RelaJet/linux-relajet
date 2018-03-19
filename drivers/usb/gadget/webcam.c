/*
 *	webcam.c -- USB webcam gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/video.h>

#include "f_uvc.h"
   
/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "composite_uvc.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

/* --------------------------------------------------------------------------
 * Device descriptor
 */
////////////////////////////////////////////////////////////
bool mAudioInited = false;
#include "u_uac1.h"
#include "u_uac1.c"
#include "f_uac1.c"

static int webcam_audio_frequency = 0;
module_param_named(audio_freq, webcam_audio_frequency, int, S_IRUGO);
MODULE_PARM_DESC(audio_freq, "Audio frequency");
/////////////////////////////////////////////////////////////

#include "uvc_queue.c"
#include "uvc_video.c"
#include "uvc_v4l2.c"
#include "f_uvc.c"

bool mSdCardOk = false;
#include "f_mass_storage.c"

#define ENABLE_MS 0


#define WEBCAM_VENDOR_ID		0x1d6b	/* Linux Foundation */
#if ENABLE_MS
#define WEBCAM_PRODUCT_ID		0x0104	/* Multi */
#else
#define WEBCAM_PRODUCT_ID		0x0102	/* Webcam A/V gadget */ //ok
#endif
#define WEBCAM_DEVICE_BCD		0x0010	/* 0.10 */

static char webcam_vendor_label[] = "Linux Foundation";
static char webcam_product_label[] = "Webcam A/V gadget";
static char webcam_config_label[] = "Audio/Video";

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_DESCRIPTION_IDX		2

static struct usb_string webcam_strings[] = {
	[STRING_MANUFACTURER_IDX].s = webcam_vendor_label,
	[STRING_PRODUCT_IDX].s = webcam_product_label,
	[STRING_DESCRIPTION_IDX].s = webcam_config_label,
	{  }
};

static struct usb_gadget_strings webcam_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = webcam_strings,
};

static struct usb_gadget_strings *webcam_device_strings[] = {
	&webcam_stringtab,
	NULL,
};

static struct usb_device_descriptor webcam_device_descriptor = {
	.bLength		= USB_DT_DEVICE_SIZE,
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB			= cpu_to_le16(0x0200),
	.bDeviceClass		= USB_CLASS_MISC,
	.bDeviceSubClass	= 0x02,
	.bDeviceProtocol	= 0x01,
	.bMaxPacketSize0	= 0, /* dynamic */
	.idVendor		= cpu_to_le16(WEBCAM_VENDOR_ID),
	.idProduct		= cpu_to_le16(WEBCAM_PRODUCT_ID),
	.bcdDevice		= cpu_to_le16(WEBCAM_DEVICE_BCD),
	.iManufacturer		= 0, /* dynamic */
	.iProduct		= 0, /* dynamic */
	.iSerialNumber		= 0, /* dynamic */
	.bNumConfigurations	= 0, /* dynamic */
};

DECLARE_UVC_HEADER_DESCRIPTOR(1);

static const struct UVC_HEADER_DESCRIPTOR(1) uvc_control_header = {
	.bLength		= UVC_DT_HEADER_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_HEADER,
	.bcdUVC			= cpu_to_le16(0x0100),
	.wTotalLength		= 0, /* dynamic */
	.dwClockFrequency	= cpu_to_le32(48000000),
	.bInCollection		= 0, /* dynamic */
	.baInterfaceNr[0]	= 0, /* dynamic */
};

static const struct uvc_camera_terminal_descriptor uvc_camera_terminal = {
	.bLength		= UVC_DT_CAMERA_TERMINAL_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_INPUT_TERMINAL,
	.bTerminalID		= 1,
	.wTerminalType		= cpu_to_le16(0x0201),
	.bAssocTerminal		= 0,
	.iTerminal		= 0,
	.wObjectiveFocalLengthMin	= cpu_to_le16(0),
	.wObjectiveFocalLengthMax	= cpu_to_le16(0),
	.wOcularFocalLength		= cpu_to_le16(0),
	.bControlSize		= 3,
	.bmControls[0]		= 2,
	.bmControls[1]		= 0,
	.bmControls[2]		= 0,
}; 

static const struct uvc_processing_unit_descriptor uvc_processing = {
	.bLength		= UVC_DT_PROCESSING_UNIT_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_PROCESSING_UNIT,
	.bUnitID		= 2,
	.bSourceID		= 1,
	.wMaxMultiplier		= cpu_to_le16(16*1024),
	.bControlSize		= 2,
	.bmControls[0]		= 1,
	.bmControls[1]		= 0,
	.iProcessing		= 0,
};

//
//Jfy_check: Interface extension Unit Descriptor
#if 0
 /* 3.7.2.6. Extension Unit Descriptor */
 struct uvc_extension_unit_descriptor {
         __u8  bLength;
         __u8  bDescriptorType;
         __u8  bDescriptorSubType;
         __u8  bUnitID;
         __u8  guidExtensionCode[16];
         __u8  bNumControls;
         __u8  bNrInPins;
         __u8  baSourceID[0];
         __u8  bControlSize;
         __u8  bmControls[0];
         __u8  iExtension;
 } __attribute__((__packed__));
 
 #define UVC_DT_EXTENSION_UNIT_SIZE(p, n)                (24+(p)+(n))
 
 #define UVC_EXTENSION_UNIT_DESCRIPTOR(p, n) \
         uvc_extension_unit_descriptor_##p_##n
 
 #define DECLARE_UVC_EXTENSION_UNIT_DESCRIPTOR(p, n)     \
 struct UVC_EXTENSION_UNIT_DESCRIPTOR(p, n) {            \
         __u8  bLength;                                  \
         __u8  bDescriptorType;                          \
         __u8  bDescriptorSubType;                       \
         __u8  bUnitID;                                  \
         __u8  guidExtensionCode[16];                    \
         __u8  bNumControls;                             \
         __u8  bNrInPins;                                \
         __u8  baSourceID[p];                            \
         __u8  bControlSize;                             \
         __u8  bmControls[n];                            \
         __u8  iExtension;                               \
 } __attribute__ ((packed))
 
#endif

#define HIBYTE(v1)      ((MMP_UBYTE)((v1)>>8))
#define LOBYTE(v1)      ((MMP_UBYTE)((v1)&0xff))

#define MSSBYTE(v1)     ((MMP_UBYTE)((v1)>>24))
#define HISBYTE(v1)     ((MMP_UBYTE)((v1)>>16))
#define LOSBYTE(v1)     ((MMP_UBYTE)((v1)>>8))
#define LESBYTE(v1)     ((MMP_UBYTE)((v1)&0xff))  

#define UUID(L1,S1,S2,D1,D2) \
LESBYTE(L1),\
LOSBYTE(L1),\
HISBYTE(L1),\
MSSBYTE(L1),\
LOBYTE(S1),\
HIBYTE(S1),\
LOBYTE(S2),\
HIBYTE(S2),\
MSSBYTE(D1),\
HISBYTE(D1),\
LOSBYTE(D1),\
LESBYTE(D1),\
MSSBYTE(D2),\
HISBYTE(D2),\
LOSBYTE(D2),\
LESBYTE(D2)

#define UVC_EU1_GUID    UUID(0x23E49ED0,0x1178,0x4f31,0xAE52D2FB,0x8A8D3B48)

DECLARE_UVC_EXTENSION_UNIT_DESCRIPTOR(1,2);

static const struct UVC_EXTENSION_UNIT_DESCRIPTOR(1, 2) uvc_extension = {
         .bLength=UVC_DT_EXTENSION_UNIT_SIZE(1, 2),                                  
         .bDescriptorType=0x24,                          
         .bDescriptorSubType=0x06,                       
         .bUnitID=0x08,      //                            
         .guidExtensionCode=UVC_EU1_GUID,                    
         .bNumControls=0x0B,                             
         .bNrInPins=1,                                
         .baSourceID=0x02,                            
         .bControlSize=0x02,
	 .bmControls={0x3F,0x00},
//       .bmControls[0]=0x3F,
//	 .bmControls[1]=0x00,                           
         .iExtension=0x00,                               

};


static const struct uvc_output_terminal_descriptor uvc_output_terminal = {
	.bLength		= UVC_DT_OUTPUT_TERMINAL_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_OUTPUT_TERMINAL,
	.bTerminalID		= 3,
	.wTerminalType		= cpu_to_le16(0x0101),
	.bAssocTerminal		= 0,
	.bSourceID		= 2,
	.iTerminal		= 0,
};

DECLARE_UVC_INPUT_HEADER_DESCRIPTOR(1, 2);

static const struct UVC_INPUT_HEADER_DESCRIPTOR(1, 2) uvc_input_header = {
	.bLength		= UVC_DT_INPUT_HEADER_SIZE(1, 2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_INPUT_HEADER,
	.bNumFormats		= 2,
	.wTotalLength		= 0, /* dynamic */
	.bEndpointAddress	= 0, /* dynamic */
	.bmInfo			= 0,
	.bTerminalLink		= 3,
	.bStillCaptureMethod	= 0,
	.bTriggerSupport	= 0,
	.bTriggerUsage		= 0,
	.bControlSize		= 1,
	.bmaControls[0][0]	= 0,
	.bmaControls[1][0]	= 4,
};

#if 1
static const struct uvc_format_uncompressed uvc_format_yuv = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 2,
	.guidFormat		=
		{ 'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel		= 16,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

DECLARE_UVC_FRAME_UNCOMPRESSED(1);
DECLARE_UVC_FRAME_UNCOMPRESSED(3);

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuv_90p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(180),
	.wHeight		= cpu_to_le16(90),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuv_120p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(160),
	.wHeight		= cpu_to_le16(120),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuv_240p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(320),
	.wHeight		= cpu_to_le16(240),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};



static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuv_360p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 4,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuv_480p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 5,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(480),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(1) uvc_frame_yuv_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 6,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 1,
	.dwFrameInterval[0]	= cpu_to_le32(5000000),
};
#endif

static const struct uvc_format_mjpeg uvc_format_mjpg = {
	.bLength		= UVC_DT_FORMAT_MJPEG_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_MJPEG,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 4,
	.bmFlags		= 0,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

DECLARE_UVC_FRAME_MJPEG(1);
DECLARE_UVC_FRAME_MJPEG(3);


static const struct UVC_FRAME_MJPEG(1) uvc_frame_mjpg_360p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(480),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 1,
	.dwFrameInterval[0]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_MJPEG(1) uvc_frame_mjpg_480p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(480),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 1,
	.dwFrameInterval[0]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_MJPEG(1) uvc_frame_mjpg_720p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 1,
	.dwFrameInterval[0]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_MJPEG(1) uvc_frame_mjpg_1080p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 4,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 1,
	.dwFrameInterval[0]	= cpu_to_le32(5000000),
};

static const struct uvc_color_matching_descriptor uvc_color_matching = {
	.bLength		= UVC_DT_COLOR_MATCHING_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_COLORFORMAT,
	.bColorPrimaries	= 1,
	.bTransferCharacteristics	= 1,
	.bMatrixCoefficients	= 4,
};

static const struct uvc_format_framebase uvc_format_h264 = {
	.bLength		= UVC_DT_FORMAT_FRAMEBASE_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_FRAME_BASED,
	.bFormatIndex		= 2,
	.bNumFrameDescriptors	= 4,
	.guidFormat		=
		{ 'H',  '2',  '6',  '4', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel = 16,
//	.bmFlags		= 0,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
	.bVariableSize = 1
};

DECLARE_UVC_FRAME_FRAMEBASE(1);
DECLARE_UVC_FRAME_FRAMEBASE(3);

static const struct UVC_FRAME_FRAMEBASE(3) uvc_frame_h264_360p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(480),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	//.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

static const struct UVC_FRAME_FRAMEBASE(3) uvc_frame_h264_480p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(480),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	//.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASE(3) uvc_frame_h264_720p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	//.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASE(3) uvc_frame_h264_1080p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 4,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	//.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct uvc_descriptor_header * const uvc_control_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_control_header,
	(const struct uvc_descriptor_header *) &uvc_camera_terminal,
	(const struct uvc_descriptor_header *) &uvc_processing,
	(const struct uvc_descriptor_header *) &uvc_extension,
	(const struct uvc_descriptor_header *) &uvc_output_terminal,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_fs_streaming_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_input_header,
	(const struct uvc_descriptor_header *) &uvc_format_yuv,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_90p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_120p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_240p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_360p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_480p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_720p,
/*
	(const struct uvc_descriptor_header *) &uvc_format_mjpg,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p,
*/
	(const struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_hs_streaming_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_input_header,

	(const struct uvc_descriptor_header *) &uvc_format_yuv,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_90p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_120p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_240p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_360p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_480p,
	//(const struct uvc_descriptor_header *) &uvc_frame_yuv_720p,
/*
	(const struct uvc_descriptor_header *) &uvc_format_mjpg,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p,
	(const struct uvc_descriptor_header *) &uvc_color_matching,
	(const struct uvc_descriptor_header *) &uvc_format_h264,
	(const struct uvc_descriptor_header *) &uvc_frame_h264_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_h264_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_h264_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_h264_1080p,
*/
	(const struct uvc_descriptor_header *) &uvc_color_matching,
//	(const struct uvc_descriptor_header *) &uvc_format_h264,
//	(const struct uvc_descriptor_header *) &uvc_frame_h264_360p,
//	(const struct uvc_descriptor_header *) &uvc_frame_h264_720p,
//	(const struct uvc_descriptor_header *) &uvc_color_matching,
	
	NULL,
};

/****************************** Configurations ******************************/
static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/*
	 * REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};


static struct fsg_module_parameters fsg_mod_data = { .stall = 1 };
FSG_MODULE_PARAMETERS(/* no prefix */, fsg_mod_data);

static struct fsg_common fsg_common;

/* --------------------------------------------------------------------------
 * USB configuration
 */

static int __init
webcam_config_bind(struct usb_configuration *c)
{
	int ret;


	ret = uvc_bind_config(c, uvc_control_cls, uvc_fs_streaming_cls,
			       uvc_hs_streaming_cls);
	if (ret < 0)
		printk("webcam: webcam_config_bind, uvc_bind_config fail\n");		

#if ENABLE_MS
	if(true == mSdCardOk)
	{
		ret = fsg_bind_config(c->cdev, c, &fsg_common);
		if (ret < 0)
			printk("ms: webcam_config_bind fsg_bind_config fail\n");
		printk("ms: webcam_config_bind 222, ret: %d\n", ret);
	}
#endif

	ret = audio_bind_config(c);
	if (ret < 0)
		printk("webcam: webcam_config_bind, audio_bind_config fail\n");		
	return ret;
}

static struct usb_configuration webcam_config_driver = {
	.label			= webcam_config_label,
	.bConfigurationValue	= 1,
	.iConfiguration		= 0, /* dynamic */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower		= CONFIG_USB_GADGET_VBUS_DRAW / 2,
};

static int /* __init_or_exit */
webcam_unbind(struct usb_composite_dev *cdev)
{
	return 0;
}

static int __init
webcam_bind(struct usb_composite_dev *cdev)
{
	int ret;
	int gcnum;
	struct usb_gadget	*gadget = cdev->gadget;


	
#if ENABLE_MS
	struct file			*filp = NULL;
	filp = filp_open("/dev/mmcblk0p1", O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(filp)) {
		printk("webcam: unable to open backing file: /dev/mmcblk0p1\n");
		//return PTR_ERR(filp);
		mSdCardOk = false;
		webcam_device_descriptor.idProduct		= cpu_to_le16(0x102);
	}
	else
	{
		filp_close(filp, current->files);
		mSdCardOk = true;
		
		/* set up mass storage function */
		fsg_common_from_params(&fsg_common, cdev, &fsg_mod_data);
		printk("ms: webcam_bind 333\n");
		/* set bcdDevice */
		gcnum = usb_gadget_controller_number(gadget);
		if (gcnum >= 0) {
			webcam_device_descriptor.bcdDevice = cpu_to_le16(0x0300 | gcnum);
		} else {
			WARNING(cdev, "controller '%s' not recognized; trying %s\n",
									gadget->name,
									webcam_config_driver.label);
			webcam_device_descriptor.bcdDevice =
									cpu_to_le16(0x0300 | 0x0099);
		}
		printk("ms: webcam_bind 444-111, gcnum: %d, %x\n", gcnum, webcam_device_descriptor.bcdDevice);
	}
#endif	
	
	/* Allocate string descriptor numbers ... note that string contents
	 * can be overridden by the composite_dev glue.
	 */
	if ((ret = usb_string_id(cdev)) < 0)
		goto error;
	webcam_strings[STRING_MANUFACTURER_IDX].id = ret;
	webcam_device_descriptor.iManufacturer = ret;

	if ((ret = usb_string_id(cdev)) < 0)
		goto error;
	webcam_strings[STRING_PRODUCT_IDX].id = ret;
	webcam_device_descriptor.iProduct = ret;

	if ((ret = usb_string_id(cdev)) < 0)
		goto error;
	webcam_strings[STRING_DESCRIPTION_IDX].id = ret;
	webcam_config_driver.iConfiguration = ret;

	/* Register our configuration. */
	if ((ret = usb_add_config(cdev, &webcam_config_driver,
					webcam_config_bind)) < 0)
		goto error;

	INFO(cdev, "Webcam Audio/Video Gadget\n");
#if ENABLE_MS
	if(true == mSdCardOk)
		fsg_common_put(&fsg_common);
#endif
	return 0;

error:
	webcam_unbind(cdev);
#if ENABLE_MS
	fsg_common_put(&fsg_common);
#endif
	return ret;
}

/* --------------------------------------------------------------------------
 * Driver
 */

static struct usb_composite_driver webcam_driver = {
	.name		= "g_webcam",
	.dev		= &webcam_device_descriptor,
	.strings	= webcam_device_strings,
	.iProduct	= "Webcam Video/Audio Gadget",
	.max_speed	= USB_SPEED_HIGH,
	.unbind		= webcam_unbind,
};

static int __init
webcam_init(void)
{
	return usb_composite_probe(&webcam_driver, webcam_bind);
}

static void __exit
webcam_cleanup(void)
{
	usb_composite_unregister(&webcam_driver);
}

//module_init(webcam_init);
late_initcall(webcam_init);
module_exit(webcam_cleanup);

MODULE_AUTHOR("Laurent Pinchart");
MODULE_DESCRIPTION("Webcam Video/Audio Gadget");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");

