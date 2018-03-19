#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/board.h>
#include <mach/lcd_common.h>
#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_display.h>
#include <mach/mmpf_pio.h>
#include <mach/mmpf_system.h>
#include <mach/mmpf_graphics.h>

struct ait_display_drv_info {
    struct device *dev;
    struct resource *mem;
    void __iomem *io;
};
static struct ait_display_drv_info *dspy;

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define OSD_FLM_SYNC_SUPPORT    (1)

#define	FLM_GPIO_NUM	        (8)  //TBD

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define WIN_OPR_OFFSET(_w)	((_w - MMPF_DISPLAY_WIN_PIP) * 0x100)

//==============================================================================
//
//                              GLOBAL VARIABLE
//
//==============================================================================
#if (WIFI_PORT == 1)
unsigned int *_kLCD_Preview_Y = 0;
unsigned int *_kLCD_Preview_U = 0;
unsigned int *_kLCD_Preview_V = 0;
#endif

#if 0//ndef defined(CONFIG_VIDEO_AIT_CAMERA)

MMP_BOOL   	gbFrameExposureDone = MMP_FALSE;			///< For FLM panel use.
MMP_UBYTE	gbExposureDoneFrame[MMPF_IBC_PIPE_MAX];	 	///< preview buffer index.
MMP_UBYTE   gbCurIBCBuf[MMPF_IBC_PIPE_MAX];				///< preview buffer index.

MMP_UBYTE   gbPreviewBufferCount[MMPF_IBC_PIPE_MAX]; 	///< preview buffer no. of ibc pipes
MMP_ULONG   glPreviewBufAddr[MMPF_IBC_PIPE_MAX][4];	 	///< preview Y buffer of ibc pipes. 
MMP_ULONG   glPreviewUBufAddr[MMPF_IBC_PIPE_MAX][4];	///< preview U buffer of ibc pipes. 
MMP_ULONG   glPreviewVBufAddr[MMPF_IBC_PIPE_MAX][4];	///< preview V buffer of ibc pipes. 

MMP_USHORT  gsPreviewBufWidth;                      	// width of preview buffer
MMP_USHORT  gsPreviewBufHeight;                     	// height of preview buffer

MMP_ULONG   glRotateBufAddr[MMPF_IBC_PIPE_MAX];     	// dst Y buffer of rotate DMA
MMP_ULONG   glRotateUBufAddr[MMPF_IBC_PIPE_MAX];    	// dst U buffer of rotate DMA
MMP_ULONG   glRotateVBufAddr[MMPF_IBC_PIPE_MAX];    	// dst V buffer of rotate DMA
MMP_UBYTE   gbRotateBufferCount;                    	// dst buffer count of rotate DMA
MMP_USHORT  gsIBCtoDMAPipe = 0;              			// keep which pipe links IBC with rotate DMA
MMP_UBYTE   gbRotateDoneBufIdx = 0;                 	// keep which buffer has the rotated frame
MMP_UBYTE   gbRotateCurBufIdx = 0;                  	// keep which buffer is to be a dst buffer in turn

MMP_BOOL	m_bPreviewPathActive[MMPF_IBC_PIPE_MAX] = {MMP_FALSE, MMP_FALSE};
MMP_USHORT	m_usPreviewLinkGraphic = 0;

MMPF_IBC_LINK_TYPE			gIBCLinkType[MMPF_IBC_PIPE_MAX];
MMPF_DISPLAY_OUTPUTDEV		gPreviewDev[MMPF_IBC_PIPE_MAX];
MMPF_DISPLAY_WINID			gPreviewWinID[MMPF_IBC_PIPE_MAX];

MMPF_OS_SEMID 	m_StartPreviewFrameEndSem;
MMP_BOOL 		m_bFirstSensorInitialize = MMP_TRUE;
#endif
/** @breif m_bReceiveStopPreviewSig for preview, created by MMPF_Sensor_Task()
MMPF_Display_IbcISR() will release m_PreviewControlSem depending on m_bReceiveStopPreviewSig.
MMPF_Display_StartPreview() set m_bReceiveStopPreviewSig MMP_FALSE in order to open preview.
MMPF_Display_StopPreview()	set m_bReceiveStopPreviewSig MMP_TRUE in order to close preview.
*/
MMP_BOOL		m_bReceiveStopPreviewSig[MMPF_IBC_PIPE_MAX] = {MMP_FALSE, MMP_FALSE};

MMPF_OS_SEMID	m_PreviewControlSem;
MMPF_OS_SEMID	m_ISPOperationDoneSem;
MMP_BOOL		m_bStartPreviewFrameEndSig;
MMP_BOOL		m_bPreviewClosedSig = MMP_FALSE;

/**	@brief	VIF frame start interrupt send this flag when it close vi output.
			ISP frame end interrupt will check that 
			does it have to release m_ISPOperationDoneSem, when it get this flag.*/
MMP_BOOL		m_bWaitISPOperationDoneSig = MMP_FALSE;

/**	@brief 	when this flag is true means we want to do the ISP related movement.
			when this flag is false means we dont want to do the ISP related movement.*/
//MMP_BOOL		m_bISP3AStatus = MMP_FALSE;

#if (OSD_FLM_SYNC_SUPPORT)
MMP_BOOL     			m_bOSDSetRefresh = MMP_FALSE;
MMPF_DISPLAY_CONTROLLER m_OSDController;
static MMPF_OS_SEMID    m_FLMRefreshSemID = 0xFFF;
#endif

#if (SUPPORT_LDC_RECD)
MMP_UBYTE 		m_ubSwiPipeLbCnt 		= 0;
MMP_UBYTE  		m_ubLbSrcPipe 			= MMPF_IBC_PIPE_0;
MMP_BOOL		m_bRetriggerGRA			= MMP_FALSE;
#endif
MMP_UBYTE		m_ub3gpPathMode			= 0;


/**
@brief The window attributes. Include width/height/base address ...etc
@details The window attribute describe how the data is put in the memory
       Each window (MAIN/OVERLAY/PIP..) have it's own attribute
       MMPF APIs will save the attribute or use the attribute directly if needed
*/
static  MMPF_DISPLAY_WIN_ATTR			m_winInfo[MMPF_DISPLAY_WIN_MAX];

/**
@brief The window display attribute. 
@details The window display attribute describe how a window is shown on a panel.
       The displayed width/hieght/startX/startY can be just partial of total window
       defined in window attribute.
       Each window (MAIN/Overlay/PIP...) have it's own display attribute.
       MMPF APIs will save the attribute or use the attribute directly if needed
*/
static  MMPF_DISPLAY_DISP_ATTR 			m_winDisplayInfo[MMPF_DISPLAY_WIN_MAX];

/**
@brief The type of display hardware 
@details The array have the hardware type of main and sub display hardware.
       It could be P-LCD, S-LCD, RGB-LCD or TV
*/
static  MMPF_DISPLAY_OUTPUTDEV  		m_displayOutputPanel[MMPF_DISPLAY_CTL_MAX];

static MMP_UBYTE                        m_winSemiEn[MMPF_DISPLAY_WIN_MAX];
static MMP_UBYTE                        m_winAlphaEn[MMPF_DISPLAY_WIN_MAX];
MMP_BOOL                         		m_bHdmiRgbIF1 = MMP_TRUE;

/**
@brief The status of preview display 
@details The status of any pipe set to preview display.
       It could be MMPF_IBC_LINK_DISPLAY or MMPF_IBC_LINK_ROTATE.
*/
static MMP_UBYTE						m_PrevwDspySts = MMP_FALSE;


//==============================================================================
//
//                              ETC
//
//==============================================================================
MMP_BOOL	m_bEnableTVInterlace = MMP_FALSE;
#define WAIT_DISPLAY_REFRESH_DONE 	(1)




//------------------------------------------------------------------------------
//  Function    : MMPF_Display_GetWinActive
//  Description : 
//------------------------------------------------------------------------------
/** @brief The function get the window active state

The function return the activa or non-active status of the the window from the input parameter, winID. 
The function can be used for dual panels.

@param[in] winID the window ID
@param[out] bActive active status of specific window ID
@return It reports the active status of the winID
*/
MMP_ERR MMPF_Display_GetWinActive(MMPF_DISPLAY_WINID winID, MMP_BOOL *bActive)
{
    AITPS_DSPY	pDSPY = AITC_BASE_DSPY;
    MMP_SHORT	offset = WIN_OPR_OFFSET(winID);
    
	#if (MCR_V2_UNDER_DBG)
 	if (winID == MMPF_DISPLAY_WIN_PIP) {
		if ((*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->DSPY_PIP_CTL) + offset)) & WIN_EN)
			*bActive = MMP_TRUE;
    	else
			*bActive = MMP_FALSE;
    }
    else {
        *bActive = MMP_FALSE;
    }
    #else
	if ((*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->DSPY_PIP_CTL) + offset)) & WIN_EN)
		*bActive = MMP_TRUE;
	else
		*bActive = MMP_FALSE;
    #endif
	
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_BindBufToWin
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set attribute of a window

The function validates and binds buffer attributes such as buffer width, height and color depth to the
specified window by setting them into LCD controller registers. The successful binding means that the
buffer attributes can be applied to the specified window. The system maintains internal window
structure. The function can be used for dual panels.

@param[in] bufattribute the struct of buffer attribute which contains W, H, line offeset, color depth, and buffer address.
@param[in] winID for the window ID
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_BindBufToWin(MMPF_GRAPHICS_BUF_ATTR *bufAttr, MMPF_DISPLAY_WINID winID)
{
    DSPY_DECL;
    MMP_SHORT   offset;
    MMP_BOOL	active;
	
    if (winID == MMPF_DISPLAY_WIN_OSD) {
		MMPF_Display_GetWinActive(winID, &active);
        if (active == MMP_TRUE) {
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & ~(SCD_DSPY_REG_READY));
        }        
    }
    else {
		MMPF_Display_GetWinActive(winID, &active);
        if (active == MMP_TRUE) {
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(PRM_DSPY_REG_READY));
        }        
    }
            
    if(winID == MMPF_DISPLAY_WIN_PIP) {
        m_bEnableTVInterlace = MMP_FALSE;
    }
    
    switch (winID) {
    case MMPF_DISPLAY_WIN_MAIN:
    case MMPF_DISPLAY_WIN_OVERLAY:
    case MMPF_DISPLAY_WIN_OSD:

		if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_8)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_8;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_16)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_16;    
        else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_24)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_24;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV422)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV422;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV420)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV420;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV420_INTERLEAVE)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_32)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_32;
		else
            return MMP_DISPLAY_ERR_PARAMETER;
		break;
    case MMPF_DISPLAY_WIN_PIP:
		if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_16)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_16;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_24)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_24;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV422)
            m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV422;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV420)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV420;
		else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_YUV420_INTERLEAVE)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE;
	    else if (bufAttr->colordepth == MMPF_GRAPHICS_COLORDEPTH_32)
			m_winInfo[winID].colordepth = MMPF_DISPLAY_WINCOLORDEPTH_32;
		else	
			return MMP_DISPLAY_ERR_PARAMETER;
        break;
    default:
        return MMP_DISPLAY_ERR_PARAMETER;
    }

    switch (winID) {
    case MMPF_DISPLAY_WIN_MAIN:
    case MMPF_DISPLAY_WIN_PIP:
    case MMPF_DISPLAY_WIN_OVERLAY:
    
		m_winInfo[winID].ulBaseAddr     = bufAttr->ulBaseAddr;
		m_winInfo[winID].usWidth        = bufAttr->usWidth;
		m_winInfo[winID].usHeight       = bufAttr->usHeight;
		m_winInfo[winID].usLineOffset   = bufAttr->usLineOffset;
    
        offset = WIN_OPR_OFFSET(winID);

		DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_ADDR_ST, offset, m_winInfo[winID].ulBaseAddr);

        if (1)//(HDMI_PREVIEW) //EROY CHECK
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & WIN_YUV420_INTERLEAVE_UV); // ------------>> djkim.check
        else
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_YUV420_INTERLEAVE_MASK));

        if (winID == MMPF_DISPLAY_WIN_PIP)
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_PIP_CTL) & ~(WIN_YUV_SCALUP_EN));

        switch (m_winInfo[winID].colordepth) {
        case MMPF_DISPLAY_WINCOLORDEPTH_8:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_8BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_16:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_16BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_24:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_24BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_32:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_32BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV422:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_YUV422);
            
			#if (CHIP == MCR_V2) //EROY CHECK
			if (winID == MMPF_DISPLAY_WIN_PIP ||
			    winID == MMPF_DISPLAY_WIN_OVERLAY) {
				DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM);
			}
			#endif
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV420:
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_YUV420);

       		m_winInfo[winID].ulBaseUAddr = bufAttr->ulBaseUAddr;
       		m_winInfo[winID].ulBaseVAddr = bufAttr->ulBaseVAddr;

	    	DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_U_ADDR_ST, offset, bufAttr->ulBaseUAddr);
    		DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_V_ADDR_ST, offset, bufAttr->ulBaseVAddr);
    	    break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE: 
			DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_YUV420_INTERLEAVE_UV);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT, offset, WIN_YUV420);
            
       		m_winInfo[winID].ulBaseUAddr = bufAttr->ulBaseUAddr;
       		m_winInfo[winID].ulBaseVAddr = bufAttr->ulBaseVAddr;

	    	DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_U_ADDR_ST, offset, bufAttr->ulBaseUAddr);
    		DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_V_ADDR_ST, offset, bufAttr->ulBaseVAddr);
    	    break;
    	}
        break;
    case MMPF_DISPLAY_WIN_OSD:
        m_winInfo[winID].ulBaseAddr 	= bufAttr->ulBaseAddr;
		m_winInfo[winID].usWidth 		= bufAttr->usWidth;
		m_winInfo[winID].usHeight 		= bufAttr->usHeight;
		m_winInfo[winID].usLineOffset 	= bufAttr->usLineOffset;
		
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_ADDR_ST, m_winInfo[winID].ulBaseAddr);
        DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & ~(SCD_SRC_FMT_MASK));
        
		switch (m_winInfo[winID].colordepth) {
		case MMPF_DISPLAY_WINCOLORDEPTH_8:
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_8BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_16:
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_16BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_24:
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_24BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_32:
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_32BPP);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV422:
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_YUV422);
            break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV420:
            m_winInfo[winID].ulBaseUAddr = bufAttr->ulBaseUAddr;
            m_winInfo[winID].ulBaseVAddr = bufAttr->ulBaseVAddr;
            
            DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_U_ADDR_ST, bufAttr->ulBaseUAddr);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_V_ADDR_ST, bufAttr->ulBaseVAddr);
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_YUV420);
    	    break;
        case MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE:
            m_winInfo[winID].ulBaseUAddr = bufAttr->ulBaseUAddr;        
            
            DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_U_ADDR_ST, bufAttr->ulBaseUAddr);
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_OSD_CTL) | WIN_YUV420_INTERLEAVE_UV);
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT, WIN_YUV420);;            
    	    break;
        }
        break;
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetWinToDisplay
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set how a window will be show on the panel

The function validates and binds the window display attributes for display device . For example, the
display dimension, display location and display rotation type can be set from the specified display
attributes. The function can be used for dual panels.

@param[in] winID the window ID
@param[in] dispAttr the display attributes
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_SetWinToDisplay(MMPF_DISPLAY_WINID winID, MMPF_DISPLAY_DISP_ATTR *dispAttr)
{
    MMP_SHORT   offset = 0;
    MMP_ULONG   ulStartOffset = 0;
    MMP_ULONG   ulStartOffsetUV = 0;
    MMP_UBYTE   pixel_byte_cnt = 0;
    MMP_BOOL	active = MMP_FALSE;
    DSPY_DECL;
	
    if (winID == MMPF_DISPLAY_WIN_OSD){
        MMPF_Display_GetWinActive(winID, &active);        
        if (active == MMP_TRUE) {
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & ~(SCD_DSPY_REG_READY));
        }
    }
    else {
		MMPF_Display_GetWinActive(winID, &active);        
        if (active == MMP_TRUE) {
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(PRM_DSPY_REG_READY));
        }        
    }

   switch (m_winInfo[winID].colordepth) {
	case MMPF_DISPLAY_WINCOLORDEPTH_4:
		break;
    case MMPF_DISPLAY_WINCOLORDEPTH_8:
	case MMPF_DISPLAY_WINCOLORDEPTH_YUV420:
	case MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE:
        pixel_byte_cnt = 1;
	    break;
    case MMPF_DISPLAY_WINCOLORDEPTH_16:
	case MMPF_DISPLAY_WINCOLORDEPTH_YUV422:
        pixel_byte_cnt = 2;
	    break;
	case MMPF_DISPLAY_WINCOLORDEPTH_24:
        pixel_byte_cnt = 3;
    	break;
    case MMPF_DISPLAY_WINCOLORDEPTH_32:
        pixel_byte_cnt = 4;
    	break;
    }

	m_winDisplayInfo[winID] = *dispAttr;
	
    switch (winID) {
	case MMPF_DISPLAY_WIN_OSD:
	    if (dispAttr->rotatetype == MMPF_DISPLAY_ROTATE_NO_ROTATE &&
	        dispAttr->bMirror == MMP_FALSE) {
	        
	        DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_OFST_ROW, m_winInfo[winID].usLineOffset);
	        
	    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_X, dispAttr->usDisplayOffsetX);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_Y, dispAttr->usDisplayOffsetY);
	    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_W, dispAttr->usDisplayWidth);
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_H, dispAttr->usDisplayHeight);

    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_PIXL_CNT, (MMP_ULONG)dispAttr->usDisplayWidth * (MMP_ULONG)dispAttr->usDisplayHeight);
        }
        else {
			return MMP_DISPLAY_ERR_NOT_SUPPORT; 					
		}
		break;

    case MMPF_DISPLAY_WIN_MAIN:
    case MMPF_DISPLAY_WIN_PIP:
    case MMPF_DISPLAY_WIN_OVERLAY:
        offset = WIN_OPR_OFFSET(winID);
        
        if (dispAttr->usStartX > m_winInfo[winID].usWidth ||
			dispAttr->usStartY > m_winInfo[winID].usHeight) {
            return MMP_DISPLAY_ERR_PARAMETER;
		}
		
        switch (dispAttr->rotatetype) {
        case MMPF_DISPLAY_ROTATE_NO_ROTATE:
            if ((dispAttr->usStartX + dispAttr->usDisplayWidth) > m_winInfo[winID].usWidth ||
                (dispAttr->usStartY + dispAttr->usDisplayHeight) > m_winInfo[winID].usHeight)
            {
                return MMP_DISPLAY_ERR_PARAMETER;
            }
            
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, m_winInfo[winID].usLineOffset);
            
            if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, m_winInfo[winID].usLineOffset >> 1);
            }
            else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, m_winInfo[winID].usLineOffset);
            }

            if (dispAttr->bMirror) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);

                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt);
                
                ulStartOffset = dispAttr->usStartY * m_winInfo[winID].usLineOffset
                         	 + (dispAttr->usStartX + dispAttr->usDisplayWidth - 1) * pixel_byte_cnt;
				
				//EROY CHECK all mirror setting
                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt);
                    
                    ulStartOffsetUV = dispAttr->usStartY * (m_winInfo[winID].usLineOffset >> 1)
                                   + (dispAttr->usStartX + (dispAttr->usDisplayWidth >> 1) - 1) * pixel_byte_cnt;
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt * 2);
                    
                    ulStartOffsetUV = dispAttr->usStartY * (m_winInfo[winID].usLineOffset)
                                	+ (dispAttr->usStartX + (dispAttr->usDisplayWidth >> 1) - 1) * pixel_byte_cnt * 2;
                }
            }
            else 
            {
            	/* Set Grab / Rotate burst attribute */
				switch (m_winInfo[winID].colordepth) {
		        case MMPF_DISPLAY_WINCOLORDEPTH_4:
		        
		            if (dispAttr->usDisplayWidth & 0x7 ||
		                dispAttr->usDisplayWidth < 64) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
		            }
		            else if (dispAttr->usStartX == 0 && 
                             dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                             (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			             m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth &&
	    	        		 !(m_winInfo[winID].usWidth & 0x3F)) {
						//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
		    			DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
					}
					else {
						//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
    					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN));			 
					}				
					break;		        		
		        case MMPF_DISPLAY_WINCOLORDEPTH_8:
		        
		            if (dispAttr->usDisplayWidth & 0x3 ||
		                dispAttr->usDisplayWidth < 32 || 
		                (!(dispAttr->usDisplayWidth & 0x3) && m_winInfo[winID].usLineOffset & 0x03)) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
		            }
		            else {
		                if (dispAttr->usStartX == 0 && 
                            dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                            (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			            m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth &&
			        		!(m_winInfo[winID].usWidth & 0x1F)) {

							//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
    		    			DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
						}
    					else {
							//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
							DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
        					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN)); 								 
				    	}	
					}
					break;		        		
				case MMPF_DISPLAY_WINCOLORDEPTH_YUV420:
				
					if (dispAttr->usDisplayWidth & 0x1) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
					} 
					else
					{
		                if (dispAttr->usStartX == 0 && 
                            dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                            (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			            m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth &&
			        		!(m_winInfo[winID].usWidth & 0x1F)) {

							//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
    		    			DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
						}
    					else {
							//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
							DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
        					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN));								 
				    	}	
					}
                    break;
                case MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE:
                
					if (dispAttr->usDisplayWidth & 0x1) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
					}
					//else
					{
	                    if (dispAttr->usStartX == 0 && 
	                        dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
	                        (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
	                        m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth &&
	                    	!(m_winInfo[winID].usWidth & 0x1F)) {

	                    	//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
	                    	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
	                    }
	                    else {
	                    	
	                        //PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
	                        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
	                        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN)); 
	                    }
                    }
                    break;                  
                case MMPF_DISPLAY_WINCOLORDEPTH_16:
                case MMPF_DISPLAY_WINCOLORDEPTH_YUV422:
	    		    
		            if (dispAttr->usDisplayWidth & 0x1) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
		            }
					//else
                    {
		                if (dispAttr->usStartX == 0 && 
                            dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                            (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			            m_winInfo[winID].usLineOffset == (m_winInfo[winID].usWidth << 1) &&
			        		!(m_winInfo[winID].usWidth & 0x0F)) {

							//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
    		    			DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
	                    }
    					else {
							//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
        		            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN));
							DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
	                    }
					}
					break;		        		
				case MMPF_DISPLAY_WINCOLORDEPTH_24:
				    
                    if (m_winInfo[winID].usLineOffset & 0x1) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
				    }    
                    else 
                    {
		                if (dispAttr->usStartX == 0 && 
                            dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                            (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			            m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth * 3 &&
			        		!((m_winInfo[winID].usWidth * 3) & 0x1F)) {

							//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
    		                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
            		    }
					    else {
			    			//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
				    		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
    				        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN));
		                }
			    	}
					break;
				case MMPF_DISPLAY_WINCOLORDEPTH_32:
				    
                    if (m_winInfo[winID].usLineOffset & 0x1) {
						//PRINTF("winID 0x%X : Non-Burst Mode\r\n", winID);
						DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);
				    }
				    //else
                    {
		                if (dispAttr->usStartX == 0 && 
                            dispAttr->usDisplayWidth == m_winInfo[winID].usWidth &&
                            (m_winInfo[winID].ulBaseAddr & 0x1F) == 0 &&
    			            m_winInfo[winID].usLineOffset == m_winInfo[winID].usWidth * 3 &&
			        		!((m_winInfo[winID].usWidth * 3) & 0x1F)) {

							//PRINTF("winID 0x%X : Burst + Non-Grab Mode\r\n", winID);
    		                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN));
            		    }
					    else {

			    			//PRINTF("winID 0x%X : Burst + Grab Mode\r\n", winID);
				    		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_SRC_GRAB_EN);
    				        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_ROTATE_EN));
		                }
			    	}		      
					break;		        		
				}

                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, pixel_byte_cnt);
                
                ulStartOffset = dispAttr->usStartY * m_winInfo[winID].usLineOffset
                           	  + dispAttr->usStartX * pixel_byte_cnt;

                if( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, pixel_byte_cnt);

   					ulStartOffsetUV = (dispAttr->usStartY >> 1) * (m_winInfo[winID].usLineOffset >> 1) 
                                 	+ (dispAttr->usStartX >> 1) * pixel_byte_cnt;
                }
                else if( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, pixel_byte_cnt*2);
   
   					ulStartOffsetUV = (dispAttr->usStartY >> 1) * (m_winInfo[winID].usLineOffset) 
                        		 	+ (dispAttr->usStartX) * pixel_byte_cnt;
                }
                
                // TV related setting, non-scaler path
				DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset, (DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset) & 0xC0) | WIN_V_1X | WIN_H_1X );
                DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_TV_EVEN_FIELD_ST, offset, m_winInfo[winID].usLineOffset);
            } 			                       
            break;
        case MMPF_DISPLAY_ROTATE_RIGHT_90: //djkim
			
			// The usDisplayHeight means the rotated height.
            if ((dispAttr->usStartX + dispAttr->usDisplayHeight) > m_winInfo[winID].usWidth ||
                (dispAttr->usStartY + dispAttr->usDisplayWidth) > m_winInfo[winID].usHeight) {
                return MMP_DISPLAY_ERR_PARAMETER;
			}
			
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, WIN_OFST_NEG | m_winInfo[winID].usLineOffset);
            
            if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 ) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | (m_winInfo[winID].usLineOffset >> 1));
            }
            else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE ) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | (m_winInfo[winID].usLineOffset));
            }
            
            if (dispAttr->bMirror) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt);
                
                ulStartOffset = (dispAttr->usStartY + dispAttr->usDisplayWidth - 1)
                    			* m_winInfo[winID].usLineOffset
                    			+ (dispAttr->usStartX + dispAttr->usDisplayHeight - 1)
                    			* pixel_byte_cnt;
                
                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt);

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayWidth >> 1) - 1)
                        			* (m_winInfo[winID].usLineOffset >> 1)
                        			+ ((dispAttr->usStartX >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        			* pixel_byte_cnt;
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt * 2 );

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayWidth >> 1) - 1)
                        			* (m_winInfo[winID].usLineOffset)
                        			+ ((dispAttr->usStartX >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        			* pixel_byte_cnt * 2;
                }
            }
            else {
            
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, pixel_byte_cnt);
                
                ulStartOffset = (dispAttr->usStartY + dispAttr->usDisplayWidth - 1)
                    			* m_winInfo[winID].usLineOffset
                    			+ dispAttr->usStartX * pixel_byte_cnt;

                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, pixel_byte_cnt);

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayWidth >> 1) - 1)
                       	 			* (m_winInfo[winID].usLineOffset >> 1)
                        			+ (dispAttr->usStartX >> 1) * pixel_byte_cnt;
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, pixel_byte_cnt*2);

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayWidth >> 1) - 1)
                        			* (m_winInfo[winID].usLineOffset)
                        			+ (dispAttr->usStartX) * pixel_byte_cnt;
                }
            }           
            break;
        case MMPF_DISPLAY_ROTATE_RIGHT_180:
            if ((dispAttr->usStartX + dispAttr->usDisplayWidth) > m_winInfo[winID].usWidth ||
                (dispAttr->usStartY + dispAttr->usDisplayHeight) > m_winInfo[winID].usHeight) {
                return MMP_DISPLAY_ERR_PARAMETER;
			}

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, WIN_OFST_NEG | m_winInfo[winID].usLineOffset);
            
            if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 ) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | (m_winInfo[winID].usLineOffset >> 1));
            }
            else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE ) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | (m_winInfo[winID].usLineOffset));
            }

            if (dispAttr->bMirror) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, pixel_byte_cnt);
                
                ulStartOffset = (dispAttr->usStartY + dispAttr->usDisplayHeight - 1)
                    			* m_winInfo[winID].usLineOffset
                    			+ (dispAttr->usStartX * pixel_byte_cnt);

                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, pixel_byte_cnt); 

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        			* (m_winInfo[winID].usLineOffset >> 1)
                        			+ ((dispAttr->usStartX >> 1) * pixel_byte_cnt);
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, pixel_byte_cnt*2); 

                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        			* (m_winInfo[winID].usLineOffset)
                        			+ ((dispAttr->usStartX) * pixel_byte_cnt);
                }
            }
            else 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt);
                
                ulStartOffset = (dispAttr->usStartY + dispAttr->usDisplayHeight - 1)
                    			* m_winInfo[winID].usLineOffset
                    			+ (dispAttr->usStartX + dispAttr->usDisplayWidth - 1)
                    			* pixel_byte_cnt;

                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt);
 
                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        		* (m_winInfo[winID].usLineOffset >> 1)
                        		+ ((dispAttr->usStartX >> 1) + (dispAttr->usDisplayWidth >> 1) - 1)
                       	 		* pixel_byte_cnt;
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, WIN_OFST_NEG | pixel_byte_cnt*2);
 
                    ulStartOffsetUV = ((dispAttr->usStartY >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        		* (m_winInfo[winID].usLineOffset)
                        		+ ((dispAttr->usStartX) + (dispAttr->usDisplayWidth - 1))
                        		* pixel_byte_cnt;
                }
            }
            
            break;
        case MMPF_DISPLAY_ROTATE_RIGHT_270:

            if ((dispAttr->usStartX + dispAttr->usDisplayHeight) > m_winInfo[winID].usWidth ||
                (dispAttr->usStartY + dispAttr->usDisplayWidth) > m_winInfo[winID].usHeight) {
                return MMP_DISPLAY_ERR_PARAMETER;
			}

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_ROTATE_EN);

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, m_winInfo[winID].usLineOffset);
            
            if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, m_winInfo[winID].usLineOffset >> 1);
            }
            else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_PIXL, offset, m_winInfo[winID].usLineOffset);
            }
            
            if (dispAttr->bMirror) 
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, pixel_byte_cnt);
                
                ulStartOffset = dispAttr->usStartY * m_winInfo[winID].usLineOffset
                   		 	 + (dispAttr->usStartX * pixel_byte_cnt);

                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, pixel_byte_cnt);

                    ulStartOffsetUV = (dispAttr->usStartY >> 1) * m_winInfo[winID].usLineOffset >> 1
                        		+ ((dispAttr->usStartX >> 1) * pixel_byte_cnt);
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, pixel_byte_cnt*2);

                    ulStartOffsetUV = (dispAttr->usStartY >> 1) * m_winInfo[winID].usLineOffset
                       	 		 + ((dispAttr->usStartX) * pixel_byte_cnt);
                }
            }
            else
            {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt);
                
                ulStartOffset = dispAttr->usStartY * m_winInfo[winID].usLineOffset
                    			+ (dispAttr->usStartX + dispAttr->usDisplayHeight - 1)
                    			* pixel_byte_cnt;
                
                if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt);

                    ulStartOffsetUV = (dispAttr->usStartY >> 1) * (m_winInfo[winID].usLineOffset >> 1)
                        		+ ((dispAttr->usStartX >> 1) + (dispAttr->usDisplayHeight >> 1) - 1)
                        		* pixel_byte_cnt;
                }
                else if ( m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE )
                {
                    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ROW, offset, WIN_OFST_NEG | pixel_byte_cnt*2);

                    ulStartOffsetUV = (dispAttr->usStartY >> 1) * (m_winInfo[winID].usLineOffset)
                        		+ ((dispAttr->usStartX) + (dispAttr->usDisplayHeight - 1))
                        		* pixel_byte_cnt;
                }
            }
            break;
        }

        DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_OFST_ST, offset, ulStartOffset);

        if ( (m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420) ||
             (m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE) )
        {
            DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_OFST_UV_ST, offset, ulStartOffsetUV);
        }
        
        if (m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_TV) {
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset, dispAttr->usDisplayOffsetX);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset, (dispAttr->usDisplayOffsetY) >> 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset, dispAttr->usDisplayWidth);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset, (dispAttr->usDisplayHeight) >> 1);
            DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_PIXL_CNT, offset, ((MMP_ULONG)dispAttr->usDisplayWidth * (MMP_ULONG)dispAttr->usDisplayHeight) >> 1);
        }
        else {
    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset, dispAttr->usDisplayOffsetX);
	        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset, dispAttr->usDisplayOffsetY);
	        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset, dispAttr->usDisplayWidth);
    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset, dispAttr->usDisplayHeight);
		    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_PIXL_CNT, offset, (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset) * (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset));
        }
        break;
    }

    if (m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_TV) {
        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2,  offset, (MMP_USHORT)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset)| WIN_FIFO_INTERLACE_LATCH );
    }
    else {
        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2,  offset, (MMP_USHORT)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset) & (~WIN_FIFO_INTERLACE_LATCH) );
    }

	// YUV422/YUV420 special case 1:1 mode setting 
	#if (CHIP == MCR_V2)
	if ((winID == MMPF_DISPLAY_WIN_PIP || winID == MMPF_DISPLAY_WIN_OVERLAY) &&
    	(m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV422 || 
    	 m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420 ||
    	 m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE)) 
	{    	
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_N, offset, 1);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M, offset, 1);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_N, offset, 1);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_M, offset, 1);
			
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM); //Note : Can't use bypass mode in MCR_V2
        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, offset) | DSPY_SOUT_RGB_EN);

	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_WT, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_WT, offset) & ~(DSPY_SCAL_WT_AVG));
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_WT, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_WT, offset) & ~(DSPY_SCAL_WT_AVG));
		
        if(m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_TV && m_bEnableTVInterlace) {
        
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_W, offset, dispAttr->usDisplayWidth);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_H, offset, dispAttr->usDisplayHeight >> 1);
         
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ST, offset, 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ED, offset, dispAttr->usDisplayWidth);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, dispAttr->usDisplayHeight >> 1);
			DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, (dispAttr->usDisplayWidth * dispAttr->usDisplayHeight) >> 1);
        } 
        else {
            DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_PIXL_CNT, offset, ((MMP_ULONG)dispAttr->usDisplayWidth * (MMP_ULONG)dispAttr->usDisplayHeight));
            
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_W, offset, dispAttr->usDisplayWidth);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_H, offset, dispAttr->usDisplayHeight);

            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ST, offset, 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ED, offset, dispAttr->usDisplayWidth);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, dispAttr->usDisplayHeight);
    	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, dispAttr->usDisplayWidth * dispAttr->usDisplayHeight);
        }   

        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset, (DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL_2, offset) & ~(WIN_DUP_MASK)) | WIN_V_1X | WIN_H_1X);
    }
	#endif

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RGBLCD_WaitFrame
//  Description : 
//------------------------------------------------------------------------------
/** @brief The function get the command back from LCD panel according to the index sent

The function return the command back from the LCD panel by programming LCD controller registers.
@param[in] ubFrameCount number of frame waiting.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_RGBLCD_WaitFrame(MMP_UBYTE ubFrameCount, MMPF_DISPLAY_RGB_IF rgbIf)
{
	DSPY_DECL;

    if (rgbIf == MMPF_RGB_IF1) {

        m_bHdmiRgbIF1 = MMP_TRUE;

        if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & RGB_IF_EN) {
        
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_INT_HOST_SR, RGB_FRME_CNT_HIT);
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_FRAM_CNT_HOST_INT, ubFrameCount);
            while(!(DSPY_RD_W(AITC_BASE_DSPY->DSPY_INT_HOST_SR) & RGB_FRME_CNT_HIT));

            return MMP_ERR_NONE;
        }
        else {
            return MMP_DISPLAY_ERR_RGBLCD_NOT_ENABLED;
    	}
	}
	else if (rgbIf == MMPF_RGB_IF2)
	{
	    m_bHdmiRgbIF1 = MMP_FALSE;

	    if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & RGB_IF_EN) {
        
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_INT_HOST_SR, RGB_FRME_CNT_HIT);
            DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_FRAM_CNT_HOST_INT, ubFrameCount);
            while(!(DSPY_RD_W(AITC_BASE_DSPY->DSPY_INT_HOST_SR) & RGB_FRME_CNT_HIT));

            return MMP_ERR_NONE;
        }
        else {
            return MMP_DISPLAY_ERR_RGBLCD_NOT_ENABLED;
    	}
	}
	return MMP_DISPLAY_ERR_RGBLCD_NOT_ENABLED;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetWinActive
//  Description : Set window address
//------------------------------------------------------------------------------
MMP_ERR MMPF_Display_SetWinActive(MMPF_DISPLAY_WINID winID, MMP_BOOL bEnable)
{
    AITPS_DSPY	pDSPY 	= AITC_BASE_DSPY;
    MMP_SHORT	offset 	= WIN_OPR_OFFSET(winID);

    if (bEnable) {
        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, (DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_EN) );
    }
    else {
        DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, (DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~WIN_EN) );	
		
		#if 1//EROY CHECK
		if (m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_RGB_LCD ||
   	    	m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_HDMI) {

            MMPF_RGBLCD_WaitFrame(1, MMPF_RGB_IF1);
		}
		#endif
	}

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_StartFLMRefresh
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Display_StartFLMRefresh(void)
{
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_StopFLMRefresh
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_Display_StopFLMRefresh(void)
{
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetFLMRefresh
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_Display_SetFLMRefresh(MMPF_DISPLAY_CONTROLLER ctrl, MMP_BOOL bWait)
{
	return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetDisplayRefresh
//  Description : 
//------------------------------------------------------------------------------
/** @brief Refresh the display device

The function refreshes LCD or TV for the frame data output by programming LCD controller register. The
refresh is activated only when necessary. The function can be used for dual panels. It returns the status
about the refresh.
@param[in] controller  the display controller
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_SetDisplayRefresh(MMPF_DISPLAY_CONTROLLER controller)
{
    DSPY_DECL;
	
    if (m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_NONE)
        return MMP_ERR_NONE;
    
    if (controller == MMPF_DISPLAY_PRM_CTL) {
    
        if (!(DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_PRM_EN)) {
            return MMP_DISPLAY_ERR_PRM_NOT_INITIALIZE;
        }

    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | PRM_DSPY_REG_READY);
    	
        if (m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_TV ||
            m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_RGB_LCD ||
            m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_HDMI) {
	    }
    	else {

    	    if((m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_P_LCD) ||
    	       (m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_S_LCD)) {

        	    #if (WAIT_DISPLAY_REFRESH_DONE)
                if (!(DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_BUSY_STATUS)) {
                    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_FRAME_TX);
                    while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_BUSY_STATUS);
                }
                #else
                while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_BUSY_STATUS);
                DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_FRAME_TX);
                #endif
            } 
            else if ((m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_P_LCD_FLM)) {
            
                #if (WAIT_DISPLAY_REFRESH_DONE)
                MMPF_Display_SetFLMRefresh(controller, MMP_TRUE);
                #else
                MMPF_Display_SetFLMRefresh(controller, MMP_FALSE);
                #endif
            }
        }     
    }
    else {
        if (!(DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_SCD_EN)) {
            return MMP_DISPLAY_ERR_SCD_NOT_INITIALIZE;        
        }
        
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | SCD_DSPY_REG_READY);

        if (m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_TV      ||
            m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_RGB_LCD || 
            m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_HDMI) {

        }
        else {

    	    if((m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_P_LCD) ||
    	       (m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_S_LCD)) {

                #if (WAIT_DISPLAY_REFRESH_DONE)
                DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_FRAME_TX);
                while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_FRAME_TX);
                #else
                while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_FRAME_TX);
                DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_FRAME_TX);
                #endif
            } 
            else if ((m_displayOutputPanel[(MMP_USHORT)controller] == MMPF_DISPLAY_P_LCD_FLM)) {
            
                #if (WAIT_DISPLAY_REFRESH_DONE)
                MMPF_Display_SetFLMRefresh(controller, MMP_TRUE);
                #else
                MMPF_Display_SetFLMRefresh(controller, MMP_FALSE);
                #endif
            }
        }
    }
    
    return MMP_ERR_NONE;       
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetRGBLCDOutput
//  Description : 
//------------------------------------------------------------------------------
/** 
@brief The function set the needed parameter for RGB-LCD (LCD without RAM) 
@param[in] controller  the display controller
@param[in] lcdattribute struct with panel related information
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_SetRGBLCDOutput(MMPF_DISPLAY_CONTROLLER controller, MMPF_DISPLAY_LCDATTRIBUTE *lcdAttr, MMPF_DISPLAY_RGB_IF rgbIf)
{
    DSPY_DECL;
    GBL_DECL;

    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_4, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_4) & ~(LCD_OUT_SEL_MASK));

    GBL_WR_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG, GBL_RD_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG) & ~(GBL_LCD_RGB_SPI_MASK));
    GBL_WR_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG, GBL_LCD_PAD_EN | GBL_LCD_RGB_SPI_PAD0);
    
    if (rgbIf == MMPF_RGB_IF1)
    {
        m_bHdmiRgbIF1 = MMP_TRUE;
        
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_FMT, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_FMT) & ~(DSPY_RGB_SYNC_MODE_DIS));
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & ~(RGB_IF_EN));
        while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & RGB_IF_EN);     
        
        if (controller == MMPF_DISPLAY_PRM_CTL) {

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_W, 				lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_H, 				lcdAttr->usHeight);
    	  	DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_X, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_Y, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_X, 	lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_Y, 	lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_PIXL_CNT, 		lcdAttr->usWidth * lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_BG_COLOR, 		lcdAttr->ulBgColor);
    		
    		if (lcdAttr->colordepth >= MMPF_LCD_COLORDEPTH_18)
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, DSPY_SOUT_RGB_888 | DSPY_SOUT_RGB);
    		else                                    
       	    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, DSPY_SOUT_RGB_565| DSPY_SOUT_DITHER_EN | DSPY_SOUT_RGB);

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(DSPY_PRM_SEL_MASK));
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_PRM_SEL(DSPY_TYPE_RGB_LCD));
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_PRM_EN);

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | PRM_DSPY_REG_READY);
    	}
    	else {
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_W, 			lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_H, 			lcdAttr->usHeight);
    	  	DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_X, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_Y, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_X, 	lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_Y, 	lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_PIXL_CNT, 	lcdAttr->usWidth * lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_BG_COLOR, 	lcdAttr->ulBgColor);

    		if (lcdAttr->colordepth >= MMPF_LCD_COLORDEPTH_18) {
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, SCD_SRC_RGB888);
    		} 
    		else {
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, SCD_SRC_RGB565 | SCD_565_2_888_STUFF_0);
    		}
    	
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(DSPY_SCD_SEL_MASK));
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_SCD_SEL(DSPY_TYPE_RGB_LCD) | DSPY_SCD_EN);
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_SCD_EN);

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | SCD_DSPY_REG_READY);
    	}

	    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) | RGB_IF_EN);
    }
    else if (rgbIf == MMPF_RGB_IF2) {

        m_bHdmiRgbIF1 = MMP_FALSE;

        DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_FMT, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_FMT) & ~(DSPY_RGB_SYNC_MODE_DIS));
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & ~(RGB_IF_EN));
        while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & RGB_IF_EN);

        if (controller == MMPF_DISPLAY_PRM_CTL) {
    	
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_W, 				lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_H, 				lcdAttr->usHeight);
    	  	DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_X, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_Y, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_X, 	lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_Y, 	lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_PIXL_CNT, 		lcdAttr->usWidth * lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_BG_COLOR, 		lcdAttr->ulBgColor);
    		
    		if (lcdAttr->colordepth >= MMPF_LCD_COLORDEPTH_18)
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, DSPY_SOUT_RGB_888 | DSPY_SOUT_RGB);
    		else                                    
       	    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL, DSPY_SOUT_RGB_565| DSPY_SOUT_DITHER_EN | DSPY_SOUT_RGB);

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(DSPY_PRM_SEL_MASK));
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_PRM_SEL(DSPY_TYPE_PRM_RGB2));
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_PRM_EN);
     
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | PRM_DSPY_REG_READY);
    	}
    	else {
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_W, 			lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_H, 			lcdAttr->usHeight);
    	  	DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_X, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_Y, 	0);
      		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_X, 	lcdAttr->usWidth);
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_Y, 	lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_PIXL_CNT, 	lcdAttr->usWidth * lcdAttr->usHeight);
    		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_BG_COLOR, 	lcdAttr->ulBgColor);

    		if (lcdAttr->colordepth >= MMPF_LCD_COLORDEPTH_18) {
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, SCD_SRC_RGB888);
    		} 
    		else {
    			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, SCD_SRC_RGB565 | SCD_565_2_888_STUFF_0);
    		}
    	
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(DSPY_SCD_SEL_MASK));
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_SCD_SEL(DSPY_TYPE_SCD_RGB2));
    	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) | DSPY_SCD_EN);

    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | SCD_DSPY_REG_READY);
    	}

    	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) | RGB_IF_EN);    
    }
    
	m_displayOutputPanel[controller] = MMPF_DISPLAY_RGB_LCD;

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_UpdateWinAddr
//  Description : Set window address
//------------------------------------------------------------------------------
/** 
@brief     Update the base address of a window 
@param[in]  winID       The window that address will be update
@param[in]  ulBaseAddr  New address (address of Y if YUV420)
@param[in]  ulBaseUAddr New address of U if YUV420
@param[in]  ulBaseVAddr new address of V if YUV420
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_UpdateWinAddr(MMPF_DISPLAY_WINID winID, MMP_ULONG ulBaseAddr, MMP_ULONG ulBaseUAddr, MMP_ULONG ulBaseVAddr)
{
	MMP_LONG	offset;
	MMP_BOOL	active;
	DSPY_DECL;
	
    offset = WIN_OPR_OFFSET(winID);
        
	switch (winID) {
	case MMPF_DISPLAY_WIN_MAIN:
	case MMPF_DISPLAY_WIN_PIP:
	case MMPF_DISPLAY_WIN_OVERLAY:
		MMPF_Display_GetWinActive(winID, &active);

        if ( (ulBaseUAddr != 0) && (ulBaseVAddr != 0) ) {
		    if (active == MMP_TRUE)
			    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(PRM_DSPY_REG_READY));
		}
		
		DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_ADDR_ST, offset, ulBaseAddr);
        
		if ( (ulBaseUAddr != 0) && (ulBaseVAddr != 0) ) {
			DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_U_ADDR_ST, offset, ulBaseUAddr);
			DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_V_ADDR_ST, offset, ulBaseVAddr);
		}
        break;
	case MMPF_DISPLAY_WIN_OSD:
		MMPF_Display_GetWinActive(winID, &active);

        if ( (ulBaseUAddr != 0) && (ulBaseVAddr != 0) ) {
		    if (active == MMP_TRUE)
			    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & ~(SCD_DSPY_REG_READY));
		}
		
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_ADDR_ST, ulBaseAddr);
        
		if ( (ulBaseUAddr != 0) && (ulBaseVAddr != 0) ) {
			DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_U_ADDR_ST, ulBaseUAddr);
			DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_V_ADDR_ST, ulBaseVAddr);
		}
        break;
    default:
        return MMP_DISPLAY_ERR_PARAMETER;
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetWinPriority
//  Description :
//------------------------------------------------------------------------------
/** @brief The function sets window display priorities by window ID

The function sets window display priorities by window IDs, the input parameters. The first parameter
gives the highest priority to that window and the fourth parameter gives the lowest priority to it. The
function can be used for dual panels.

@param[in] prio1 window ID for the first priority
@param[in] prio2 window ID for the second priority
@param[in] prio3 window ID for the third priority
@param[in] prio4 window ID for the fourth priority

@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_SetWinPriority(MMPF_DISPLAY_WINID prio1, MMPF_DISPLAY_WINID prio2, MMPF_DISPLAY_WINID prio3, MMPF_DISPLAY_WINID prio4)
{
    DSPY_DECL;

    if(prio1 == MMPF_DISPLAY_WIN_OVERLAY){
        prio1 = OVLY_WIN;
    }
    else if(prio2 == MMPF_DISPLAY_WIN_OVERLAY){
        prio2 = OVLY_WIN;
    }
    else if(prio3 == MMPF_DISPLAY_WIN_OVERLAY){
        prio3 = OVLY_WIN;
    } 
    
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_WIN_PRIO, (prio3 << WIN_3_SHFT) + (prio2 << WIN_2_SHFT) + (prio1 << WIN_1_SHFT) );

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_Display_SetWinScaling
//  Description :
//------------------------------------------------------------------------------
/** @brief The function configures the scaling and the grab operations

The function configures the scaling and the grab operations by programming LCD
controller registers.

@param[in] winID The window ID
@param[in] bKeepAspectRatio If the X and Y dimension would be kept in the same aspect ratio.
@param[in] fitrange The fit range parameters.
@param[out] grabctl The grab control parameters. Assign NULL to ignore the output.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_Display_SetWinScaling(MMPF_DISPLAY_WINID winID, MMP_BOOL bUserDefine, MMP_BOOL bKeepAspectRatio, MMPF_SCAL_FIT_RANGE *fitrange, MMPF_SCAL_GRAB_CTRL *grabctl)
{
    MMP_USHORT  scaleM;
    MMP_USHORT  y_scale, x_scale;
    MMP_USHORT  usStartX, usEndX, usStartY, usEndY;
    MMP_USHORT  unscale_width, unscale_height;
    MMP_BOOL	bReGrab = MMP_FALSE;
    MMP_BOOL	active;
    MMP_SHORT	offset;
	DSPY_DECL;
    
	if ((winID != MMPF_DISPLAY_WIN_PIP) && (winID != MMPF_DISPLAY_WIN_OVERLAY))
        return MMP_DISPLAY_ERR_PARAMETER;
	
	MMPF_Display_GetWinActive(winID, &active);
    
    if (active == MMP_TRUE)
        DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & ~(PRM_DSPY_REG_READY));
	
	offset = WIN_OPR_OFFSET(winID);
	
	if (winID == MMPF_DISPLAY_WIN_PIP ||
	 	winID == MMPF_DISPLAY_WIN_OVERLAY)
	{
		if (bUserDefine == MMP_FALSE) {
		
	    	scaleM = fitrange->ulFitResol;
	    	y_scale = (fitrange->ulOutHeight * scaleM + fitrange->ulInHeight - 1) / fitrange->ulInHeight;
	    	x_scale = (fitrange->ulOutWidth * scaleM + fitrange->ulInWidth - 1) / fitrange->ulInWidth;

	        if ((x_scale == y_scale) && (scaleM == x_scale)) {
	            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM | DSPY_SCAL_WT_DYN);
	            return MMP_ERR_NONE;
	        }
	        else {
	            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM | DSPY_SCAL_WT_DYN);
	        }
	    }
	    else {
	        if ( (grabctl->ulScaleXM == grabctl->ulScaleXN) && (grabctl->ulScaleYM == grabctl->ulScaleYN) ){
	            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM | DSPY_SCAL_WT_DYN);
	            return MMP_ERR_NONE;
	        }
	        else {
	            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL, offset, DSPY_SCAL_NM | DSPY_SCAL_WT_DYN);
	        }
	    }
	}
	
	if (bUserDefine == MMP_FALSE) 
	{
		/* Calculate scaling ratio */
		if ((fitrange->ulOutHeight > fitrange->ulInHeight) ||( fitrange->ulOutWidth > fitrange->ulInWidth)) {
			// Scale up case
			if (fitrange->fitmode == MMPF_SCAL_FITMODE_OUT) {
		    	while ((((fitrange->ulInHeight - 1) * y_scale + scaleM - 1) / scaleM)
		    			< fitrange->ulOutHeight)
	    			y_scale++;
		    	while ((((fitrange->ulInWidth - 1) * x_scale + scaleM - 1) / scaleM)
		    			< fitrange->ulOutWidth)
	    			x_scale++;
			}
			else {
		    	while ((((fitrange->ulInHeight - 1) * y_scale + scaleM - 1) / scaleM)
		    			> fitrange->ulOutHeight)
	    			y_scale--;
		    	while ((((fitrange->ulInWidth - 1) * x_scale + scaleM - 1) / scaleM)
		    			> fitrange->ulOutWidth)
	    			x_scale--;
			}	    		
		}
		else {
			// Scale down case
			if (fitrange->fitmode == MMPF_SCAL_FITMODE_OUT) {
			    while (((fitrange->ulInHeight) * y_scale) < (fitrange->ulOutHeight * scaleM))
	    			y_scale++;
			    while (((fitrange->ulInWidth) * x_scale) < (fitrange->ulOutWidth * scaleM))
		    		x_scale++;
			}
			else {
		    	while (((fitrange->ulInHeight) * y_scale) > (fitrange->ulOutHeight * scaleM))
	    			y_scale--;
		    	while (((fitrange->ulInWidth) * x_scale) > (fitrange->ulOutWidth * scaleM))
	    			x_scale--;
			}	    		
		}
	
		/* Adjust scaling ratio */
 	    if (fitrange->fitmode == MMPF_SCAL_FITMODE_OUT) 
 	    {
    	    if (bKeepAspectRatio == MMP_TRUE) {
				if (x_scale > y_scale) {
					y_scale = x_scale;
				}					
				else {
					x_scale = y_scale;
				}				
			} 
			else {
			    if ((fitrange->ulOutHeight > fitrange->ulInHeight) ||( fitrange->ulOutWidth > fitrange->ulInWidth)) {
			        if(x_scale < scaleM)
			            x_scale = scaleM;
			        if(y_scale < scaleM)
			            y_scale = scaleM;
			    } 
			    else {
			        if(x_scale > scaleM)
			            x_scale = scaleM;
			        if(y_scale > scaleM)
			            y_scale = scaleM;
			    }
			}
    	} 
    	else if (fitrange->fitmode == MMPF_SCAL_FITMODE_IN) 
    	{
        	if (bKeepAspectRatio == MMP_TRUE) {
	        	if (x_scale > y_scale) {
    	        	x_scale = y_scale;
				}
		        else {
    		        y_scale = x_scale;
				}
			} 
			else {
			    if ((fitrange->ulOutHeight > fitrange->ulInHeight) ||( fitrange->ulOutWidth > fitrange->ulInWidth)) {
			        if(x_scale < scaleM)
			            x_scale = scaleM;
			        if(y_scale < scaleM)
			            y_scale = scaleM;
			    } else {
			        if(x_scale > scaleM)
			            x_scale = scaleM;
			        if(y_scale > scaleM)
			            y_scale = scaleM;
			    }
			}				
    	}
    	
    	// EROY CHECK
	    if ((x_scale | scaleM | y_scale | scaleM) > 127) {
	    	PRINTF("Scaling ratio over range\r\n");
    	    return MMP_DISPLAY_ERR_OVERRANGE;
    	}
		
		/* Calculate grab range */
	    if (x_scale > scaleM || y_scale > scaleM) {

    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_YUV_SCALUP_EN);
			
			if (fitrange->fitmode == MMPF_SCAL_FITMODE_OUT) 
			{
		    	usStartX 	= ((fitrange->ulInWidth - 1) * x_scale / scaleM - fitrange->ulOutWidth) / 2 + 1;
		    	usStartY 	= ((fitrange->ulInHeight - 1) * y_scale / scaleM - fitrange->ulOutHeight) / 2 + 1;
			    usEndX 		= usStartX + fitrange->ulOutWidth - 1;
			    usEndY 		= usStartY + fitrange->ulOutHeight - 1;
			}
			else if (fitrange->fitmode == MMPF_SCAL_FITMODE_IN) 
			{
	    	    usStartX 	= 1;
        		usStartY 	= 1;
			    usEndX 		= ((fitrange->ulInWidth - 1) * x_scale + scaleM - 1) / scaleM;
		    	usEndY 		= ((fitrange->ulInHeight - 1) * y_scale + scaleM - 1) / scaleM;
				
				// EROY CHECK
				if (grabctl == NULL) {
					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset) + (fitrange->ulOutWidth - usEndX) / 2);
					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset) + (fitrange->ulOutHeight - usEndY) / 2);
				}					
			}				    
		}
	    else {
	    
    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_YUV_SCALUP_EN));

			if (fitrange->fitmode == MMPF_SCAL_FITMODE_OUT) 
			{
	        	usStartX 	= (fitrange->ulInWidth * x_scale / scaleM - fitrange->ulOutWidth) / 2 + 1;
	        	usStartY 	= (fitrange->ulInHeight * y_scale / scaleM - fitrange->ulOutHeight) / 2 + 1;
			    usEndX 		= usStartX + fitrange->ulOutWidth - 1;
	    		usEndY	 	= usStartY + fitrange->ulOutHeight - 1;
			}
			else if (fitrange->fitmode == MMPF_SCAL_FITMODE_IN) 
			{
		        usStartX 	= 1;
        		usStartY 	= 1;
			    usEndX 		= fitrange->ulInWidth * x_scale / scaleM;
		    	usEndY 		= fitrange->ulInHeight * y_scale / scaleM;

				if (grabctl == NULL) {
					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X, offset) + (fitrange->ulOutWidth - usEndX) / 2);
					DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y, offset) + (fitrange->ulOutHeight - usEndY) / 2);
				}					
			}	    	
		}
		
		if (grabctl != NULL) {
			grabctl->ulScaleN 	= x_scale;
    		grabctl->ulScaleM 	= scaleM;
			grabctl->ulOutStX	= usStartX;
    		grabctl->ulOutEdX	= usEndX;
			grabctl->ulOutStY	= usStartY;
    		grabctl->ulOutEdY 	= usEndY;
		}
	}    	
	else {  //(bUserDefine == MMP_TRUE) 

		x_scale 	= grabctl->ulScaleXN;
		y_scale 	= grabctl->ulScaleYN;
    	scaleM  	= grabctl->ulScaleXM;
		usStartX  	= grabctl->ulOutStX;
		usEndX    	= grabctl->ulOutEdX;
		usStartY  	= grabctl->ulOutStY;
		usEndY    	= grabctl->ulOutEdY;
		
		if (x_scale > scaleM || y_scale > scaleM) {
    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) | WIN_YUV_SCALUP_EN);
		}
	    else {
    	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & ~(WIN_YUV_SCALUP_EN));
		}
	}
	
	/* Set the OPR of the display scaler engine, update window width/height/pixel count */
	#if (CHIP == MCR_V2) //EROY CHECK : MCR_V2 has this bug?
	if (DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset) & WIN_YUV_SCALUP_EN) 
	{ 
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_N, offset, x_scale);
 	 	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M, offset, scaleM);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_N, offset, y_scale);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_M, offset, scaleM);

		scaleM = DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M, offset);
        
        if (bUserDefine == MMP_FALSE) {
            if (grabctl != NULL) {
		    	grabctl->ulScaleN = DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_N, offset);
        		grabctl->ulScaleM = DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M, offset);
			}
        }
	} 
	else {
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_N, offset, x_scale);
 	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M, offset, scaleM);
 	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_N, offset, y_scale);
 	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_M, offset, scaleM);
	}
	#endif
	
	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ST, offset, usStartX);
	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ED, offset, usEndX);
	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, usStartY);
	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, usEndY);

	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset, (usEndX - usStartX + 1));
	DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset, (usEndY - usStartY + 1));

    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset)
				    								 * (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset));		    								 
		
	/* Modify display setting for TV */	//EROY CHECK	    								 
    if (m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_TV) 
    {
		if(m_bEnableTVInterlace) {
		
            if(usStartY != 1) {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, usStartY >> 1);
            }
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, usEndY >> 1);
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset) - DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset) + 1);
            DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset) * (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset));
        }
        else {
            if(DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset) & 0x01) {
                DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, usEndY - 1);
            }
            DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset, DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset) >> 1);
            DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, ((MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W, offset) * (MMP_ULONG)DSPY_RD_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H, offset)) << 1);
        }
    }
    
    /* Calculate ReGrab range */
	if ((m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_TV) ||
		(m_displayOutputPanel[MMPF_DISPLAY_PRM_CTL] == MMPF_DISPLAY_RGB_LCD)) 
	{
	    if (x_scale > scaleM || y_scale > scaleM) {
    	    unscale_width = ((usEndX - usStartX) * scaleM + (x_scale - 1)) / x_scale + 1;
        	unscale_height = ((usEndY - usStartY) * scaleM + (y_scale - 1)) / y_scale + 1;
	    }
    	else {
        	unscale_width = ((usEndX - usStartX + 1) * scaleM + (x_scale - 1)) / x_scale;
	        unscale_height = ((usEndY - usStartY + 1) * scaleM + (y_scale - 1)) / y_scale;
    	}   
	    PRINTF("unscale_width : unscale_height = %d, %d\r\n", unscale_width, unscale_height);

    	if (fitrange->ulInHeight - unscale_height > 5) {
    	
   			bReGrab = MMP_TRUE;
        	
    	    if ( (m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420) ||
    	         (m_winInfo[winID].colordepth == MMPF_DISPLAY_WINCOLORDEPTH_YUV420_INTERLEAVE) )
    	    {
	            m_winDisplayInfo[winID].usStartY += (m_winDisplayInfo[winID].usDisplayHeight - unscale_height) / 2;
	            m_winDisplayInfo[winID].usStartY -= m_winDisplayInfo[winID].usStartY & 0x01;
	            
    	        if (unscale_height & 0x01)
					unscale_height += 1;
            	
            	m_winDisplayInfo[winID].usDisplayHeight = unscale_height;
	        }    
    	    else {
        	    m_winDisplayInfo[winID].usStartY += (MMP_ULONG)((usStartY - 1) * scaleM + (y_scale >> 1)) / y_scale + 1; //EROY CHECK
	            m_winDisplayInfo[winID].usDisplayHeight = unscale_height;
    	    }    

			fitrange->ulInHeight = unscale_height;
	    }
    	if (fitrange->ulInWidth - unscale_width > 5) {
        	
	   		bReGrab = MMP_TRUE;
        	
	        m_winDisplayInfo[winID].usStartX += (m_winDisplayInfo[winID].usDisplayWidth - unscale_width) / 2;
	        m_winDisplayInfo[winID].usStartX -= m_winDisplayInfo[winID].usStartX & 0x01;
	        
    	    if (unscale_width & 0x01)
        	    unscale_width += 1;
			
			m_winDisplayInfo[winID].usDisplayWidth = unscale_width;

			fitrange->ulInWidth = unscale_width;
    	}

	    if (bReGrab) 
	    {
			MMPF_Display_SetWinToDisplay(winID, &(m_winDisplayInfo[winID]));
			
			fitrange->ulFitResol = scaleM;
			
			if (grabctl == NULL) {
			    MMPF_Display_SetWinScaling(winID, MMP_FALSE, bKeepAspectRatio, fitrange, NULL);
			} 
			else {
			    grabctl->ulOutEdX 	= grabctl->ulOutEdX - grabctl->ulOutStX + 1;
			    grabctl->ulOutEdY 	= grabctl->ulOutEdY - grabctl->ulOutStY + 1;
			    grabctl->ulOutStX 	= 1;
			    grabctl->ulOutStY 	= 1;
			    
			    if(!bUserDefine) {
    			    grabctl->ulScaleXN = grabctl->ulScaleN;
    		        grabctl->ulScaleYN = grabctl->ulScaleN;
        	        grabctl->ulScaleXM = grabctl->ulScaleM;
        	        grabctl->ulScaleYM = grabctl->ulScaleM;
    	        }
			    
			    MMPF_Display_SetWinScaling(winID, MMP_TRUE, bKeepAspectRatio, fitrange, grabctl);
			}
		}		
	}	
	    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_LCD_SendIndex
//  Description :
//------------------------------------------------------------------------------
/** @brief  Send specified index to LCD panel

The function sends the specified index to one LCD panel by programming LCD controller register. The
function can be used for dual panels.

@param[in] usIndex the index for the LCD to be sent
@return It reports the status of the operation.
*/
MMP_ERR MMPF_LCD_SendIndex(MMP_ULONG ulIndex)
{
	DSPY_DECL;
	
	if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_PRM_EN) {
	
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_0, ulIndex);
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_IDX_RDY);
		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_IDX_RDY);
	} 
	else if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_SCD_EN) {

	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_LCD_TX_0, ulIndex);
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_IDX_RDY);
	    while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_IDX_RDY);
	}
	else {
		return MMP_DISPLAY_ERR_NON_CONTROLLER_ENABLE;
	}

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_LCD_SendCommand
//  Description :
//------------------------------------------------------------------------------
/** @brief Send the specified command to LCD panel

The function sends the specified command to one LCD panel by programming LCD controller register.
The function can be used for dual panels.

@param[in] ulCommand the command for the LCD to be sent
@return It reports the status of the operation.
*/
MMP_ERR MMPF_LCD_SendCommand(MMP_ULONG ulCommand)
{
	DSPY_DECL;
	
	if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_PRM_EN) {
	
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_1, ulCommand);
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_CMD_RDY);
	   	while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_CMD_RDY);
    } 
    else if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_SCD_EN) {
    
        DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_LCD_TX_1, ulCommand);
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_CMD_RDY);
   	  	while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_CMD_RDY);
   	}
   	else {
		return MMP_DISPLAY_ERR_NON_CONTROLLER_ENABLE;
	}

    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_LCD_SendIndexCommand
//  Description :
//------------------------------------------------------------------------------
/** @brief The function sends a index and a command to LCD panel

The function sends a index and a command to LCD panel by programming LCD controller registers.
The function can be used for the auto transfer of LCD index/command sent before outputting frame
data to LCD panel. The function can be used for dual panels.

@param[in] ulIndex
@param[in] ulCommand
@return It reports the status of the operation.
*/
MMP_ERR MMPF_LCD_SendIndexCommand(MMP_ULONG ulIndex, MMP_ULONG ulCommand)
{
	DSPY_DECL;

	if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_PRM_EN) {
	
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_0, ulIndex);
   		DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_1, ulCommand);

		if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & PLCD_CMD_BURST) {
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | (LCD_CMD_RDY | LCD_IDX_RDY));
    		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_IDX_RDY);
		}
		else {
	    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_IDX_RDY);
	    	while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_IDX_RDY);

		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_CMD_RDY);
    		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_CMD_RDY);
		}
    }
    else if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_SCD_EN) {
    
        DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_LCD_TX_0, ulIndex);
   	   	DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_LCD_TX_1, ulCommand);

	    if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & PLCD_CMD_BURST) {
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | (LCD_CMD_RDY | LCD_IDX_RDY));
    		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_IDX_RDY);
    	}
		else {
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_IDX_RDY);
        	while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_IDX_RDY);
	
	        DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_CMD_RDY);
    		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_CMD_RDY);
    	}
	}
	else {
		return MMP_DISPLAY_ERR_NON_CONTROLLER_ENABLE;
	}

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_LCD_GetIndexCommand
//  Description :
//------------------------------------------------------------------------------
/** @brief The function get the command back from LCD panel according to the index sent

The function return the command back from the LCD panel by programming LCD controller registers.
@param[in] ulIndex
@param[out] ulCommand
@param[in] ulReadCount
@return It reports the status of the operation.
*/
MMP_ERR MMPF_LCD_GetIndexCommand(MMP_ULONG ulIndex, MMP_ULONG *ulCommand, MMP_ULONG ulReadCount)
{
    MMP_ULONG i = 0;
    MMP_BOOL  bBurst = MMP_FALSE;
	DSPY_DECL;
    
	if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & PLCD_CMD_BURST) {
        DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & ~(PLCD_CMD_BURST));
        bBurst = MMP_TRUE;
	}
	
	if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_PRM_EN) {
	
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_0, ulIndex);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | (LCD_IDX_RDY));
		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_IDX_RDY);
        
        for(i = 0; i < ulReadCount; i++) {
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | (LCD_CMD_RDY | LCD_PANEL_READ_EN));
		    while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_CMD_RDY);
		    ulCommand[i] = DSPY_RD_D(AITC_BASE_DSPY->DSPY_PLCD_READ_PORT);
		}
		
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & ~(LCD_PANEL_READ_EN));
	}
	else if (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_2) & DSPY_SCD_EN) {

	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_LCD_TX_0, ulIndex);
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | (LCD_IDX_RDY));
		while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_IDX_RDY);
        
        for(i = 0; i < ulReadCount; i++) {
    		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | (LCD_CMD_RDY | LCD_PANEL_READ_EN));
        	while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_CMD_RDY);
        	ulCommand[i] = DSPY_RD_D(AITC_BASE_DSPY->DSPY_PLCD_READ_PORT);
    	}
    	
    	DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & ~(LCD_PANEL_READ_EN));
	}
	else {

		if(bBurst) {
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) | (PLCD_CMD_BURST));
		}
		return MMP_DISPLAY_ERR_NON_CONTROLLER_ENABLE;
	}
	
	if(bBurst) {
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) | (PLCD_CMD_BURST));
	}
    return MMP_ERR_NONE;
}

static int __init_drv_resource(struct ait_display_platform_data_info *plat)
{
	/* Enable Display Clock */
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_TRUE);

	/* DISPLAY H/W RESET */
	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DSPY, MMP_TRUE);

    return 0;
}

static int ait_display_probe(struct platform_device *pdev)
{
	struct ait_display_platform_data_info *plat;
	AITPS_DSPY pDSPY = AITC_BASE_DSPY;
	int ret;

	if (!pdev || !pdev->dev.platform_data) {
		return -EINVAL;
	}
	plat = pdev->dev.platform_data;
	
	dspy = kmalloc(sizeof(struct ait_display_drv_info), GFP_KERNEL);
    if (dspy == NULL) {
        dev_err(&pdev->dev, "fail to kmalloc for ait_display_drv_info\n");
        return -ENOMEM;
    }
    dspy->dev = &pdev->dev;
    platform_set_drvdata(pdev, dspy);

	__init_drv_resource(plat);
	
	dev_info(dspy->dev, "AIT Display probe done\n");
	return 0;
}

static int ait_display_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s \n", __func__);
	
	return 0;
}

static struct platform_driver ait_display_driver = {
	.probe		= ait_display_probe,
	.remove		= ait_display_remove,
	.driver		= {
		.name	= DEVICE_NAME_FB_DISPLAY,
		.owner	= THIS_MODULE,
	},
	.suspend	= NULL,
	.resume		= NULL,
};

int __init ait_display_init(void)
{
	int ret = platform_driver_register(&ait_display_driver);
	return ret;
}

static void __exit ait_display_exit(void)
{
	platform_driver_unregister(&ait_display_driver);
}

module_init(ait_display_init);
module_exit(ait_display_exit);
