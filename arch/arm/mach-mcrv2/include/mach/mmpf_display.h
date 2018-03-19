#ifndef __MMPF_DISPLAY_H__
#define __MMPF_DISPLAY_H__

#include <mach/mmpf_graphics.h>

//==============================================================================
//
//                              DU DEFINE
//
//==============================================================================

struct ait_display_platform_data_info {
	unsigned int start;
	unsigned int end;
	unsigned int irq;
    unsigned int reserved;
};

/*  index for image description  */
enum {
    DU_IMAGE_FORMAT_RGB565          = 0x0,
    DU_IMAGE_FORMAT_RGBA4444,
    DU_IMAGE_FORMAT_RGBA5551,
    DU_IMAGE_FORMAT_RGB888,
    DU_IMAGE_FORMAT_RGBA8888,
    DU_IMAGE_FORMAT_Gray,
    DU_IMAGE_FORMAT_Index,
    DU_IMAGE_FORMAT_YUV420I,
    DU_IMAGE_FORMAT_YUV422I,
    DU_IMAGE_FORMAT_YUV444I,
    DU_IMAGE_FORMAT_YUV420P,
    DU_IMAGE_FORMAT_YUV422P,
    DU_IMAGE_FORMAT_YUV444P,
    DU_IMAGE_FORMAT_YUV422V,
    DU_IMAGE_FORMAT_YUV422IR,
};

enum {
	DU_STRP_ROT_0					= 0x0,
	DU_STRP_V_MIRROR,
	DU_STRP_H_MIRROR,
	DU_STRP_ROT_180,
	DU_STRP_ROT_90_V_MIRROR,
	DU_STRP_ROT_270,
	DU_STRP_ROT_90,
	DU_STRP_ROT_90_H_MIRROR,
};

/* type define for image description */
struct du_resolution_info {
    unsigned short sx;
    unsigned short ex;
    unsigned short sy;
    unsigned short ey;
    unsigned short width;
    unsigned short height;
};

struct du_image_address_info {
    unsigned long base_Y;
    unsigned long U;
    unsigned long V;
};

struct du_image_info {
    unsigned char format;
    struct du_resolution_info res;
    struct du_image_address_info addr;

    unsigned char pxl_swap;
    unsigned char rgb_swap;
    unsigned char c_swap;
};

//==============================================================================
//
//                              MMMPF DEFINE
//
//==============================================================================

typedef enum _MMPF_DISPLAY_CONTROLLER
{
    MMPF_DISPLAY_PRM_CTL = 0,
    MMPF_DISPLAY_SCD_CTL,
    MMPF_DISPLAY_CTL_MAX
} MMPF_DISPLAY_CONTROLLER;

typedef enum _MMPF_DISPLAY_OUTPUTDEV
{
	MMPF_DISPLAY_NONE = 0,
    MMPF_DISPLAY_P_LCD,
    MMPF_DISPLAY_S_LCD,
    MMPF_DISPLAY_P_LCD_FLM,
    MMPF_DISPLAY_RGB_LCD,
    MMPF_DISPLAY_TV,
    MMPF_DISPLAY_HDMI,
    MMPF_DISPLAY_CCIR
} MMPF_DISPLAY_OUTPUTDEV;

typedef enum _MMPF_LCD_COLORDEPTH
{
    MMPF_LCD_COLORDEPTH_16 = 0,  ///< Panel color is 16bit
    MMPF_LCD_COLORDEPTH_18,      ///< Panel color is 18bit
    MMPF_LCD_COLORDEPTH_24       ///< Panel color is 24bit
} MMPF_LCD_COLORDEPTH;

typedef enum _MMPF_DISPLAY_LCD_CSSEL
{
    MMPF_DISPLAY_CSSEL_1 = 0,
    MMPF_DISPLAY_CSSEL_2
} MMPF_DISPLAY_LCD_CSSEL;

typedef enum _MMPF_TV_TYPE {
    MMPF_TV_TYPE_NTSC = 0,
    MMPF_TV_TYPE_PAL
} MMPF_TV_TYPE;

typedef enum _MMPF_DISPLAY_TVPLLTYPE
{

    MMPF_DISPLAY_TVPLLTYPE_PLL0 = 0x00,
    MMPF_DISPLAY_TVPLLTYPE_PLL1,
    MMPF_DISPLAY_TVPLLTYPE_PLL2,
    MMPF_DISPLAY_TVPLLTYPE_EXT,
    MMPF_DISPLAY_TVPLLTYPE_PLL3,
    MMPF_DISPLAY_TVPLLTYPE_PLL4,
    MMPF_DISPLAY_TVPLLTYPE_PLL5
} MMPF_DISPLAY_TVPLLTYPE;

typedef enum _MMPF_DISPLAY_HDMIOUTPUTMODE
{
    MMPF_DISPLAY_HDMIOUTPUT_UNSUPPORT 		= 0,
    MMPF_DISPLAY_HDMIOUTPUT_USERDEF 		= 0x7FFFFFFF,
    MMPF_DISPLAY_HDMIOUTPUT_640X480P 		= 0x00000001,
    MMPF_DISPLAY_HDMIOUTPUT_720X480P 		= 0x00000002,
    MMPF_DISPLAY_HDMIOUTPUT_720X576P 		= 0x00000004,
    MMPF_DISPLAY_HDMIOUTPUT_1280X720P 		= 0x00000008,
    MMPF_DISPLAY_HDMIOUTPUT_1280X720P_50FPS = 0x00000010,
    MMPF_DISPLAY_HDMIOUTPUT_1920X1080P 		= 0x00000020,
    MMPF_DISPLAY_HDMIOUTPUT_1920X1080P_30FPS = 0x00000040,
    MMPF_DISPLAY_HDMIOUTPUT_1920X1080I 		= 0x00000080
} MMPF_DISPLAY_HDMIOUTPUTMODE;

typedef enum _MMPF_DISPLAY_HDMIPLLTYPE
{

    MMPF_DISPLAY_HDMIPLLTYPE_PLL0 = 0x00,
    MMPF_DISPLAY_HDMIPLLTYPE_PLL1,
    MMPF_DISPLAY_HDMIPLLTYPE_PLL2,
    MMPF_DISPLAY_HDMIPLLTYPE_EXT,
    MMPF_DISPLAY_HDMIPLLTYPE_PLL3,
    MMPF_DISPLAY_HDMIPLLTYPE_PLL4,
    MMPF_DISPLAY_HDMIPLLTYPE_PLL5
} MMPF_DISPLAY_HDMIPLLTYPE;

typedef enum _MMPF_DISPLAY_WINCOLORDEPTH
{
    MMPF_DISPLAY_WINCOLORDEPTH_4 = 0,
    MMPF_DISPLAY_WINCOLORDEPTH_8,
    MMPF_DISPLAY_WINCOLORDEPTH_16,
    MMPF_DISPLAY_WINCOLORDEPTH_24,
    MMPF_DISPLAY_WINCOLORDEPTH_YUV420,
    MMPF_DISPLAY_WINCOLORDEPTH_YUV422,
    MMPF_DISPLAY_WINCOLORDEPTH_32,
    MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE
} MMPF_DISPLAY_WINCOLORDEPTH;

typedef enum _MMPF_DISPLAY_WINID
{

    MMPF_DISPLAY_WIN_MAIN       = 0,
    MMPF_DISPLAY_WIN_PIP        = 1,
    MMPF_DISPLAY_WIN_OSD        = 3,
    MMPF_DISPLAY_WIN_OVERLAY    = 4,
    MMPF_DISPLAY_WIN_MAX        = 5
} MMPF_DISPLAY_WINID;

typedef enum _MMPF_DISPLAY_ROTATE_TYPE
{
    MMPF_DISPLAY_ROTATE_NO_ROTATE = 0,
    MMPF_DISPLAY_ROTATE_RIGHT_90,
    MMPF_DISPLAY_ROTATE_RIGHT_180,
    MMPF_DISPLAY_ROTATE_RIGHT_270,
    MMPF_DISPLAY_ROTATE_MAX
} MMPF_DISPLAY_ROTATE_TYPE;

typedef enum _MMPF_DISPLAY_COLORMODE
{
    MMPF_DISPLAY_COLOR_RGB565 = 0,
    MMPF_DISPLAY_COLOR_RGB888,
    MMPF_DISPLAY_COLOR_YUV422,
    MMPF_DISPLAY_COLOR_YUV420,
    MMPF_DISPLAY_COLOR_YUV420_INTERLEAVE
} MMPF_DISPLAY_COLORMODE;

typedef enum _MMPF_DISPLAY_SEMITP_FUNC
{
    MMPF_DISPLAY_SEMITP_AVG = (0x00 << 2),
    MMPF_DISPLAY_SEMITP_AND = (0x01 << 2),
    MMPF_DISPLAY_SEMITP_OR  = (0x02 << 2),
    MMPF_DISPLAY_SEMITP_INV = (0x03 << 2)
} MMPF_DISPLAY_SEMITP_FUNC;

typedef enum _MMPF_DISPLAY_TV_SYNCMODE
{
	MMPF_DISPLAY_TV_FRAMESYNC = 0,
	MMPF_DISPLAY_TV_FIELDSYNC
} MMPF_DISPLAY_TV_SYNCMODE;

typedef enum _MMPF_DISPLAY_DUPLICATE {
    MMPF_DISPLAY_DUPLICATE_1X = 0,
    MMPF_DISPLAY_DUPLICATE_2X,
    MMPF_DISPLAY_DUPLICATE_4X
} MMPF_DISPLAY_DUPLICATE;

typedef enum _MMPF_DISPLAY_ALPHA_FMT
{
    MMPF_DISPLAY_ALPHA_RGBA = 0,
    MMPF_DISPLAY_ALPHA_ARGB 
} MMPF_DISPLAY_ALPHA_FMT;

typedef enum _MMPF_ICON_ALPHA_DEPTH
{
    MMPF_ICO_FORMAT_RGB565   = 0x00,
    MMPF_ICO_FORMAT_ARGB3454 = 0x04,
    MMPF_ICO_FORMAT_ARGB4444 = 0x08,
    MMPF_ICO_FORMAT_ARGB8888 = 0x0C
} MMPF_ICON_ALPHA_DEPTH;

#if (CHIP == MCR_V2)
typedef enum _MMPF_DISPLAY_RGB_IF 
{
    MMPF_RGB_IF1 = 0,
    MMPF_RGB_IF2,
    MMPF_RGB_IF_BOTH
} MMPF_DISPLAY_RGB_IF;
#endif

//==============================================================================
//
//                              SCALER
//
//==============================================================================

typedef enum _MMPF_SCAL_FIT_MODE
{
    MMPF_SCAL_FITMODE_OUT = 0,
    MMPF_SCAL_FITMODE_IN,
    MMPF_SCAL_FITMODE_OPTIMAL,
    MMPF_SCAL_FITMODE_NUM
} MMPF_SCAL_FIT_MODE;

typedef enum _MMPF_SCAL_SCALER_TYPE
{
    MMPF_SCAL_TYPE_SCALER = 0,
    MMPF_SCAL_TYPE_BAYERSCALER,
    MMPF_SCAL_TYPE_NUM
} MMPF_SCAL_SCALER_TYPE;

typedef struct _MMPF_SCAL_FIT_RANGE {
    MMPF_SCAL_FIT_MODE  	fitmode;
    MMPF_SCAL_SCALER_TYPE	scalerType;
    MMP_ULONG 	ulFitResol;
    MMP_ULONG  	ulInWidth;
    MMP_ULONG 	ulInHeight;
    MMP_ULONG  	ulOutWidth;
    MMP_ULONG 	ulOutHeight;

    MMP_ULONG 	ulInGrabX;
    MMP_ULONG	ulInGrabY;
    MMP_ULONG 	ulInGrabW;
    MMP_ULONG	ulInGrabH;
    
    MMP_ULONG 	ulDummyInPixelX;
    MMP_ULONG	ulDummyInPixelY;
    MMP_ULONG	ulDummyOutPixelX;	// Used in bayer scaler
    MMP_ULONG	ulDummyOutPixelY;	// Used in bayer scaler
   
    MMP_ULONG	ulInputRatioH;
    MMP_ULONG	ulInputRatioV;
    MMP_ULONG	ulOutputRatioH;
    MMP_ULONG	ulOutputRatioV;
} MMPF_SCAL_FIT_RANGE;

typedef struct _MMPF_SCAL_GRAB_CTRL {
    MMP_ULONG	ulOutStX;
    MMP_ULONG 	ulOutStY;
    MMP_ULONG	ulOutEdX;
    MMP_ULONG 	ulOutEdY;

    MMP_ULONG 	ulScaleN;
    MMP_ULONG 	ulScaleM;
    MMP_ULONG	ulScaleXN;
    MMP_ULONG 	ulScaleXM;
    MMP_ULONG  	ulScaleYN;
    MMP_ULONG 	ulScaleYM; 
} MMPF_SCAL_GRAB_CTRL;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef struct _MMPF_DISPLAY_LCDATTRIBUTE {
    MMP_USHORT          		usWidth;
    MMP_USHORT          		usHeight;
    MMPF_LCD_COLORDEPTH			colordepth;
    MMP_ULONG					ulBgColor;
    MMP_BOOL            		bFLMType;
} MMPF_DISPLAY_LCDATTRIBUTE;

typedef struct _MMPF_DISPLAY_TVATTRIBUTE {
    MMPF_TV_TYPE        		tvtype;
    MMP_USHORT          		usStartX;
    MMP_USHORT          		usStartY;
    MMP_USHORT          		usDisplayWidth;
    MMP_USHORT          		usDisplayHeight;
    MMP_ULONG					ulDspyBgColor;
    MMP_UBYTE					ubTvBgYColor;
    MMP_UBYTE					ubTvBgUColor;
    MMP_UBYTE					ubTvBgVColor;
    MMPF_DISPLAY_TVPLLTYPE  	PLLType;
} MMPF_DISPLAY_TVATTRIBUTE;

typedef struct _MMPF_DISPLAY_HDMIATTRIBUTE {
    MMP_UBYTE  					ubColorType;
    MMP_USHORT 					usDisplayWidth;
    MMP_USHORT 					usDisplayHeight;
    MMP_ULONG  					ulDspyBgColor;
    MMP_UBYTE  					ubOutputMode;
    MMPF_DISPLAY_HDMIPLLTYPE 	PLLType;
} MMPF_DISPLAY_HDMIATTRIBUTE;

typedef struct _MMPF_DISPLAY_WIN_ATTR {
    MMP_USHORT          		usWidth;
    MMP_USHORT          		usHeight;
    MMP_USHORT          		usLineOffset;
    MMPF_DISPLAY_WINCOLORDEPTH 	colordepth;
    MMP_ULONG           		ulBaseAddr;
    MMP_ULONG           		ulBaseUAddr;
    MMP_ULONG          	 		ulBaseVAddr;
} MMPF_DISPLAY_WIN_ATTR;

typedef struct _MMPF_DISPLAY_WBACKATTRIBUTE {
    MMP_USHORT          		usWidth;
    MMP_USHORT          		usHeight;
    MMP_USHORT          		usStartX;
    MMP_USHORT          		usStartY;
    MMP_ULONG           		ulBaseAddr;
    MMPF_DISPLAY_WINCOLORDEPTH	colordepth;
    MMP_BOOL            		bWriteBackOnly;
} MMPF_DISPLAY_WBACKATTRIBUTE;

typedef struct _MMPF_DISPLAY_DISP_ATTR {
    MMP_USHORT                  usStartX;           // Desired start offset X within the window buffer
    MMP_USHORT                  usStartY;           // Desired start offset Y within the window buffer
    MMP_USHORT                  usDisplayWidth;     // Desired display width within the window buffer
    MMP_USHORT                  usDisplayHeight;    // Desired display height within the window buffer
    MMP_USHORT                  usDisplayOffsetX;   // Window offset X relative to device boundary
    MMP_USHORT                  usDisplayOffsetY;   // Window offset Y relative to device boundary
    MMP_BOOL                    bMirror;
    MMPF_DISPLAY_ROTATE_TYPE    rotatetype;
} MMPF_DISPLAY_DISP_ATTR;

typedef struct _MMPF_DISPLAY_RECT {
    MMP_USHORT 					usLeft;
    MMP_USHORT 					usTop;
    MMP_USHORT 					usWidth;
    MMP_USHORT 					usHeight;
} MMPF_DISPLAY_RECT;


//==============================================================================
//
//                              FUNCTION
//
//==============================================================================
MMP_ERR MMPF_Display_BindBufToWin(MMPF_GRAPHICS_BUF_ATTR *bufAttr, MMPF_DISPLAY_WINID winID);
MMP_ERR MMPF_Display_SetWinToDisplay(MMPF_DISPLAY_WINID winID, MMPF_DISPLAY_DISP_ATTR *dispAttr);
MMP_ERR MMPF_Display_SetWinActive(MMPF_DISPLAY_WINID winID, MMP_BOOL bEnable);
MMP_ERR MMPF_Display_SetDisplayRefresh(MMPF_DISPLAY_CONTROLLER controller);
MMP_ERR MMPF_Display_SetRGBLCDOutput(MMPF_DISPLAY_CONTROLLER controller, MMPF_DISPLAY_LCDATTRIBUTE *lcdAttr, MMPF_DISPLAY_RGB_IF rgbIf);
MMP_ERR MMPF_Display_UpdateWinAddr(MMPF_DISPLAY_WINID winID, MMP_ULONG ulBaseAddr, MMP_ULONG ulBaseUAddr, MMP_ULONG ulBaseVAddr);
MMP_ERR MMPF_RGBLCD_WaitFrame(MMP_UBYTE ubFrameCount, MMPF_DISPLAY_RGB_IF rgbIf);
MMP_ERR MMPF_Display_SetWinPriority(MMPF_DISPLAY_WINID prio1, MMPF_DISPLAY_WINID prio2, MMPF_DISPLAY_WINID prio3, MMPF_DISPLAY_WINID prio4);
MMP_ERR MMPF_Display_SetWinScaling(MMPF_DISPLAY_WINID winID, MMP_BOOL bUserDefine, MMP_BOOL bKeepAspectRatio, MMPF_SCAL_FIT_RANGE *fitrange, MMPF_SCAL_GRAB_CTRL *grabctl);


#endif
