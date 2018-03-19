#ifndef __MMPF_GRAPHICS_H__
#define __MMPF_GRAPHICS_H__

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define GRAPHICS_SEM_TIMEOUT	(0x100)

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_GRAPHICS_COLORDEPTH
{
    MMPF_GRAPHICS_COLORDEPTH_8 		= 1,
    MMPF_GRAPHICS_COLORDEPTH_16 	= 2,
    MMPF_GRAPHICS_COLORDEPTH_24 	= 3,
    MMPF_GRAPHICS_COLORDEPTH_YUV422 = 4,
	MMPF_GRAPHICS_COLORDEPTH_YUV420 = 5,
	MMPF_GRAPHICS_COLORDEPTH_32 	= 6,
	MMPF_GRAPHICS_COLORDEPTH_YUV420_INTERLEAVE = 7,
	MMPF_GRAPHICS_COLORDEPTH_UNSUPPORT
} MMPF_GRAPHICS_COLORDEPTH;

typedef enum _MMPF_GRAPHICS_ROTATE_TYPE
{
    MMPF_GRAPHICS_ROTATE_NO_ROTATE = 0,
    MMPF_GRAPHICS_ROTATE_RIGHT_90,
    MMPF_GRAPHICS_ROTATE_RIGHT_180,
    MMPF_GRAPHICS_ROTATE_RIGHT_270
} MMPF_GRAPHICS_ROTATE_TYPE;

typedef enum _MMPF_GRAPHICS_KEYCOLOR
{
    MMPF_GRAPHICS_FG_COLOR = 0,
    MMPF_GRAPHICS_BG_COLOR
} MMPF_GRAPHICS_KEYCOLOR;

typedef enum _MMPF_GRAPHICS_RECTFILLTYPE 
{
    MMPF_GRAPHICS_SOLID_FILL = 0,
    MMPF_GRAPHICS_LINE_FILL
} MMPF_GRAPHICS_RECTFILLTYPE;

typedef enum _MMPF_GRAPHICS_DELAY_TYPE
{
    MMPF_GRAPHICS_DELAY_CHK_SCA_BUSY = 0,
    MMPF_GRAPHICS_DELAY_CHK_LINE_END
} MMPF_GRAPHICS_DELAY_TYPE;

typedef enum _MMPF_GRAPHICS_ROP
{
    MMPF_GRAPHICS_ROP_BLACKNESS     = 0,
    MMPF_GRAPHICS_ROP_NOTSRCERASE   = 0x01,	// ~(S+D)
    MMPF_GRAPHICS_ROP_NOTSRCCOPY    = 0x03,	// ~S
    MMPF_GRAPHICS_ROP_SRCERASE      = 0x04,	// S.~D
    MMPF_GRAPHICS_ROP_DSTINVERT     = 0x05,	// ~D
    MMPF_GRAPHICS_ROP_SRCINVERT     = 0x06,	// S^D
    MMPF_GRAPHICS_ROP_SRCAND        = 0x08,	// S.D
    MMPF_GRAPHICS_ROP_MERGEPAINT    = 0x0B,	// ~S+D
    MMPF_GRAPHICS_ROP_SRCCOPY       = 0x0C,	// S
    MMPF_GRAPHICS_ROP_SRCPAINT      = 0x0E,	// S+D
    MMPF_GRAPHICS_ROP_WHITENESS     = 0x0F
} MMPF_GRAPHICS_ROP;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef struct _MMPF_GRAPHICS_BUF_ATTR {
    MMP_USHORT                      usWidth;
    MMP_USHORT                      usHeight;
    MMP_USHORT                      usLineOffset;
    MMPF_GRAPHICS_COLORDEPTH        colordepth;
	MMP_ULONG                       ulBaseAddr;
    MMP_ULONG                       ulBaseUAddr;
    MMP_ULONG                       ulBaseVAddr;
} MMPF_GRAPHICS_BUF_ATTR;

typedef struct _MMPF_GRAPHICS_RECT {
    MMP_USHORT          			usLeft;
    MMP_USHORT          			usTop;
    MMP_USHORT          			usWidth;
    MMP_USHORT          			usHeight;
} MMPF_GRAPHICS_RECT;

typedef struct _MMPF_GRAPHICS_DRAWRECT_ATTR {
    MMPF_GRAPHICS_RECTFILLTYPE 		type;
    MMP_BOOL            			bUseRect;
    MMP_USHORT          			usWidth;        // Buffer Width
    MMP_USHORT          			usHeight;       // Buffer Height
    MMP_USHORT          			usLineOfst;     // Buffer LineOffset
	MMP_ULONG           			ulBaseAddr;     // Buffer Base Address
    MMPF_GRAPHICS_COLORDEPTH 		colordepth;
    MMP_ULONG           			ulColor;
    MMP_ULONG           			ulPenSize;
    MMPF_GRAPHICS_ROP   			ropcode;
} MMPF_GRAPHICS_DRAWRECT_ATTR;



//==============================================================================
//
//                              IBC
//
//==============================================================================

typedef enum _MMPF_IBC_LINK_TYPE
{
	MMPF_IBC_LINK_NONE 		= 0x00,
	MMPF_IBC_LINK_DISPLAY	= 0x01,
	MMPF_IBC_LINK_VIDEO		= 0x02,
	MMPF_IBC_LINK_ROTATE	= 0x04,
	MMPF_IBC_LINK_FDTC      = 0x08,
	MMPF_IBC_LINK_GRAPHIC   = 0x10,
	MMPF_IBC_LINK_USB		= 0x20,
	MMPF_IBC_LINK_LDC		= 0x40,
	MMPF_IBC_LINK_MDTC		= 0x80,
	MMPF_IBC_LINK_MASK		= 0xFF
} MMPF_IBC_LINK_TYPE;

typedef enum _MMPF_IBC_COLOR
{
    MMPF_IBC_COLOR_RGB565       = 0,
    MMPF_IBC_COLOR_YUV422       = 1,
    MMPF_IBC_COLOR_RGB888       = 2,
    MMPF_IBC_COLOR_I420         = 3,
    MMPF_IBC_COLOR_YUV420_LUMA_ONLY = 4,
    MMPF_IBC_COLOR_NV12         = 5,
    MMPF_IBC_COLOR_NV21         = 6,
    MMPF_IBC_COLOR_M420_CBCR    = 7,
    MMPF_IBC_COLOR_M420_CRCB    = 8,
    MMPF_IBC_COLOR_YUV422_YUYV  = 9,
    MMPF_IBC_COLOR_YUV422_UYVY  = 10,
    MMPF_IBC_COLOR_YUV422_YVYU  = 11,
    MMPF_IBC_COLOR_YUV422_VYUY  = 12,
    MMPF_IBC_COLOR_YUV444_2_YUV422_YUYV = 13,
    MMPF_IBC_COLOR_YUV444_2_YUV422_YVYU = 14,
    MMPF_IBC_COLOR_YUV444_2_YUV422_UYVY = 15,
    MMPF_IBC_COLOR_YUV444_2_YUV422_VYUY = 16,
    MMPF_IBC_COLOR_MAX_NUM    
} MMPF_IBC_COLOR;

typedef enum _MMPF_IBC_SRC_COLOR
{
    MMPF_IBC_SRC_COLOR_RGB565 = 0,
    MMPF_IBC_SRC_COLOR_RGB888,
    MMPF_IBC_SRC_COLOR_YUV444,
    MMPF_IBC_SRC_COLOR_YUV422
} MMPF_IBC_SRC_COLOR;

typedef enum _MMPF_IBC_FX
{
    MMPF_IBC_FX_TOFB = 0,
    MMPF_IBC_FX_JPG, 
    MMPF_IBC_FX_RING_BUF,
    MMPF_IBC_FX_H264
} MMPF_IBC_FX;

typedef enum _MMPF_IBC_PIPEID
{
    MMPF_IBC_PIPE_0 = 0,
    MMPF_IBC_PIPE_1,
    MMPF_IBC_PIPE_2,
    MMPF_IBC_PIPE_3,
    MMPF_IBC_PIPE_4,
    MMPF_IBC_PIPE_MAX
} MMPF_IBC_PIPEID;

typedef enum _MMPF_IBC_EVENT {
    MMPF_IBC_EVENT_FRM_ST 		= 0,
    MMPF_IBC_EVENT_FRM_RDY 		= 1,
    MMPF_IBC_EVENT_FRM_END 		= 2,
    MMPF_IBC_EVENT_FRM_PRERDY 	= 3,
    MMPF_IBC_EVENT_MAX 			= 4
} MMPF_IBC_EVENT;



#endif

