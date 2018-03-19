//==============================================================================
//
//  File        : mmpf_vif.c
//  Description : MMPF_VIF functions
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"
#include "lib_retina.h"
#include "mmp_reg_vif.h"
#if (OS_TYPE == OS_UCOSII)
#include "mmp_reg_color.h"
#endif
#include "mmp_reg_gbl.h"
#include "mmpf_vif.h"
#include "mmpf_system.h"

/** @addtogroup MMPF_VIF
@{
*/
//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================

MMPF_VIF_IN_TYPE m_VIFInterface[MMPF_VIF_MDL_NUM] = {MMPF_VIF_PARALLEL};

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetPIODir
//  Description : This function set VIF PIO direction.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF PIO direction.
 * 
 *  This function set VIF PIO direction.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] mask      : stands for bit mask identifying the PIOs
 * @param[in] bOutput   : stands for input or output data.
 * @return It return the function status.   
 */
MMP_ERR	MMPF_VIF_SetPIODir(MMP_UBYTE ubVid, MMP_UBYTE mask, MMP_BOOL bOutput)
{
    AITPS_VIF   	pVIF    = AITC_BASE_VIF;
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif

	switch(ubVid) 
	{
		case MMPF_VIF_MDL_ID0:
			if(bOutput == MMP_TRUE)
				pVIF->VIF_0_SENSR_SIF_EN |= mask;
			else
				pVIF->VIF_0_SENSR_SIF_EN &= ~mask;
		break;
		case MMPF_VIF_MDL_ID1:
			if(bOutput == MMP_TRUE)
				pVIF->VIF_1_SENSR_SIF_EN |= mask;
			else	
				pVIF->VIF_1_SENSR_SIF_EN &= ~mask;
		break;
		#if (CHIP == MERCURY)
		case MMPF_VIF_MDL_ID2:
			if(bOutput == MMP_TRUE)
				pVIF2->VIF_2_SENSR_SIF_EN |= mask;
			else
				pVIF2->VIF_2_SENSR_SIF_EN &= ~mask;
		break;
		#endif
		default:
		break;    		    	
	}

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetPIOOutput
//  Description : This function set VIF PIO output data.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF PIO output data.
 * 
 *  This function set VIF PIO output data.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] mask      : stands for bit mask identifying the PIOs
 * @param[in] bSetHigh  : stands for output data high/low. 
 * @return It return the function status.   
 */
MMP_ERR	MMPF_VIF_SetPIOOutput(MMP_UBYTE ubVid, MMP_UBYTE mask, MMP_BOOL bSetHigh)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif

	switch(ubVid) 
	{
		case MMPF_VIF_MDL_ID0:
			if(bSetHigh == MMP_TRUE)
				pVIF->VIF_0_SENSR_SIF_DATA |= mask;
			else
				pVIF->VIF_0_SENSR_SIF_DATA &= ~mask;
		break;
		case MMPF_VIF_MDL_ID1:
			if(bSetHigh == MMP_TRUE)
				pVIF->VIF_1_SENSR_SIF_DATA |= mask;
			else	
				pVIF->VIF_1_SENSR_SIF_DATA &= ~mask;
		break;
		#if (CHIP == MERCURY)
		case MMPF_VIF_MDL_ID2:
			if(bSetHigh == MMP_TRUE)
				pVIF2->VIF_2_SENSR_SIF_DATA |= mask;
			else
				pVIF2->VIF_2_SENSR_SIF_DATA &= ~mask;
		break;
		#endif
		default:
		break;    		    	
	}

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_GetPIOOutput
//  Description : This function get VIF PIO data.
//------------------------------------------------------------------------------
/** 
 * @brief This function get VIF PIO output data.
 * 
 *  This function get VIF PIO output data.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] mask      : stands for bit mask identifying the PIOs.
 * @return It return the mask mapping result.  
 */
MMP_BOOL MMPF_VIF_GetPIOOutput(MMP_UBYTE ubVid, MMP_UBYTE mask)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;

	switch(ubVid)
	{
		case MMPF_VIF_MDL_ID0:
			return (pVIF->VIF_0_SENSR_SIF_DATA & mask)?(MMP_TRUE):(MMP_FALSE);
		break;
		case MMPF_VIF_MDL_ID1:
			return (pVIF->VIF_1_SENSR_SIF_DATA & mask)?(MMP_TRUE):(MMP_FALSE);
		break;
		#if (CHIP == MERCURY)
		case MMPF_VIF_MDL_ID2:
			return (pVIF2->VIF_2_SENSR_SIF_DATA & mask) ? MMP_TRUE : MMP_FALSE;
		break;
		#endif
		default:
			return MMP_FALSE;
		break;    		    	
	}
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_RegisterInputInterface
//  Description : This function register VIF input type.
//------------------------------------------------------------------------------
/** 
 * @brief This function register VIF input type.
 * 
 *  This function register VIF input type.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] type      : stands for VIF input type.
 * @return It return the function status.  
 */
MMP_ERR	MMPF_VIF_RegisterInputInterface(MMP_UBYTE ubVid, MMPF_VIF_IN_TYPE type)
{
    if (type == MMPF_VIF_MIPI)
	{
        #if (CHIP == MERCURY)
    	if (ubVid == MMPF_VIF_MDL_ID2)
    	{
    		RTNA_DBG_Str3("VIF2 Not Support MIPI Mode\r\n");
    		return MMP_ERR_NONE;
    	}
    	#endif
		
        RTNA_DBG_Str3("Set MIPI Input\r\n");
    }
    else {
        RTNA_DBG_Str3("Set Parallel Input\r\n");
    }

    m_VIFInterface[ubVid] = type;

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_IsInterfaceEnable
//  Description : This function check VIF input interface enable.
//------------------------------------------------------------------------------
/** 
 * @brief This function check VIF input interface enable.
 * 
 *  This function check VIF input interface enable.
 * @param[in]  ubVid     : stands for VIF module index. 
 * @param[out] bEnable   : stands for input interface enable.
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_IsInterfaceEnable(MMP_UBYTE ubVid, MMP_BOOL *bEnable)
{
    AITPS_VIF   pVIF 	= AITC_BASE_VIF;
    AITPS_MIPI  pMIPI 	= AITC_BASE_MIPI;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
    
    if( (ubVid==MMPF_VIF_MDL_ID2) && (m_VIFInterface[ubVid]==MMPF_VIF_MIPI) )
    	return MMP_ERR_NONE;
    
	#endif

    *bEnable = MMP_FALSE;

    if (m_VIFInterface[ubVid] == MMPF_VIF_MIPI) 
    {	
    	#if (CHIP == MERCURY)
    	if(ubVid==MMPF_VIF_MDL_ID2) 
    	{
    		*bEnable = MMP_FALSE;
    	}
    	else
    	#endif
		{
            if (pMIPI->MIPI_CLK_CFG[ubVid] & MIPI_CSI2_EN)
                *bEnable = MMP_TRUE;
        }
    }
    else 
    {
    	#if (CHIP == MERCURY)
    	if(ubVid==MMPF_VIF_MDL_ID2)
    	{
    		if (pVIF2->VIF_2_IN_EN)
	            *bEnable = MMP_TRUE;
    	}
    	else
    	#endif
    	{
	        if (pVIF->VIF_IN_EN[ubVid] & VIF_IN_ENABLE)
	            *bEnable = MMP_TRUE;
        }
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_EnableInputInterface
//  Description : This function set VIF input interface.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF input interface.
 * 
 *  This function set VIF input interface.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] bEnable   : stands for input interface enable.
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_EnableInputInterface(MMP_UBYTE ubVid, MMP_BOOL bEnable)
{
    AITPS_VIF  	pVIF 	= AITC_BASE_VIF;
    AITPS_MIPI  pMIPI 	= AITC_BASE_MIPI;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;

    if( (ubVid==MMPF_VIF_MDL_ID2) && (m_VIFInterface[ubVid]==MMPF_VIF_MIPI) )
    	return MMP_ERR_NONE;
    	
	#endif  
  
    if (bEnable == MMP_TRUE) 
    {
        if(m_VIFInterface[ubVid] == MMPF_VIF_MIPI)
        {
            pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_CSI2_EN;
        }
        else
        {      
        	#if (CHIP == MERCURY)
        	if(ubVid==MMPF_VIF_MDL_ID2)
        	{
	            pVIF2->VIF_2_OUT_EN 	&= ~(VIF_OUT_ENABLE);
	            pVIF2->VIF_2_IN_EN 		|= VIF_IN_ENABLE;        	
        	}
        	else
        	#endif
			{
	            pVIF->VIF_OUT_EN[ubVid] 	&= ~(VIF_OUT_ENABLE);
	            pVIF->VIF_IN_EN[ubVid] 		|= VIF_IN_ENABLE;
			}
    	}
    }
    else 
    {
        if (m_VIFInterface[ubVid] == MMPF_VIF_MIPI) 
            pMIPI->MIPI_CLK_CFG[ubVid] 	&= ~(MIPI_CSI2_EN);
        else 
        {
        	#if (CHIP == MERCURY)
        	if(ubVid==MMPF_VIF_MDL_ID2)
        		pVIF2->VIF_2_IN_EN &= ~(VIF_IN_ENABLE);
        	else
        	#endif
           		pVIF->VIF_IN_EN[ubVid] &= ~(VIF_IN_ENABLE);
        } 
    }

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_EnableOutput
//  Description : This function set VIF output enable.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF output enable.
 * 
 *  This function set VIF output enable.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] bEnable   : stands for output frame enable.
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_EnableOutput(MMP_UBYTE ubVid, MMP_BOOL bEnable)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;

    if( (ubVid==MMPF_VIF_MDL_ID2) && (m_VIFInterface[ubVid]==MMPF_VIF_MIPI) )
    	return MMP_ERR_NONE;

	#endif
    
    if (bEnable == MMP_TRUE) 
    {
    	#if (CHIP == MERCURY)
    	if(ubVid==MMPF_VIF_MDL_ID2)
    		pVIF2->VIF_2_OUT_EN |= VIF_OUT_ENABLE;
    	else
    	#endif
            pVIF->VIF_OUT_EN[ubVid] |= VIF_OUT_ENABLE;
        
        if (ubVid == 1)
        	pVIF->VIF_RAW_OUT_EN[ubVid] |= VIF_1_TO_ISP;
    }
    else 
    {
    	#if (CHIP == MERCURY)
    	if(ubVid==MMPF_VIF_MDL_ID2)
    		pVIF2->VIF_2_OUT_EN &= ~(VIF_OUT_ENABLE);
    	else
    	#endif
        	pVIF->VIF_OUT_EN[ubVid] &= ~(VIF_OUT_ENABLE);

        if (ubVid == 1)
        	pVIF->VIF_RAW_OUT_EN[ubVid] &= ~(VIF_1_TO_ISP);
    }

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_IsOutputEnable
//  Description : This function check VIF output enable.
//------------------------------------------------------------------------------
/** 
 * @brief This function check VIF output enable.
 * 
 *  This function check VIF output enable.
 * @param[in]  ubVid     : stands for VIF module index. 
 * @param[out] bEnable   : stands for output enable.
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_IsOutputEnable(MMP_UBYTE ubVid, MMP_BOOL *bEnable)
{
    AITPS_VIF pVIF = AITC_BASE_VIF; 
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif 

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_TRUE);
	#if (CHIP == MERCURY)
	if(ubVid==MMPF_VIF_MDL_ID2) {
	    if (pVIF->VIF_2_OUT_EN & VIF_OUT_ENABLE) {
	        *bEnable = MMP_TRUE;
	    }
	    else {
	        *bEnable = MMP_FALSE;
	    }
	}
	else
	#endif
	{
	    if (pVIF->VIF_OUT_EN[ubVid] & VIF_OUT_ENABLE) {
	        *bEnable = MMP_TRUE;
	    }
	    else {
	        *bEnable = MMP_FALSE;
	    }
	}
    
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_FALSE);

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetGrabPosition
//  Description : This function set VIF grab range.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF grab range.
 * 
 *  This function set VIF grab range.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] pGrab     : stands for VIF grab information.
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetGrabPosition(MMP_UBYTE ubVid, MMPF_VIF_GRAB_INFO *pGrab)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif 

	#if (CHIP == MERCURY)
	if(ubVid==MMPF_VIF_MDL_ID2)
	{
	    pVIF2->VIF_2_GRAB.PIXL_ST = pGrab->usStartX;
	    pVIF2->VIF_2_GRAB.PIXL_ED = pGrab->usStartX + pGrab->usGrabWidth - 1;
	    pVIF2->VIF_2_GRAB.LINE_ST = pGrab->usStartY;
	    pVIF2->VIF_2_GRAB.LINE_ED = pGrab->usStartY + pGrab->usGrabHeight - 1;	
	}
	else
	#endif
	{
	    pVIF->VIF_GRAB[ubVid].PIXL_ST = pGrab->usStartX;
	    pVIF->VIF_GRAB[ubVid].PIXL_ED = pGrab->usStartX + pGrab->usGrabWidth - 1;
	    pVIF->VIF_GRAB[ubVid].LINE_ST = pGrab->usStartY;
	    pVIF->VIF_GRAB[ubVid].LINE_ED = pGrab->usStartY + pGrab->usGrabHeight - 1;
	}
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_GetGrabPosition
//  Description : This function get VIF grab position.
//------------------------------------------------------------------------------
/** 
 * @brief This function get VIF grab position.
 * 
 *  This function get VIF grab position.
 * @param[in]  ubVid        : stands for VIF module index. 
 * @param[out] usPixelStart : stands for VIF grab pixel start position.
 * @param[out] usPixelEnd   : stands for VIF grab pixel end position.
 * @param[out] usLineStart  : stands for VIF grab line start position.
 * @param[out] usLineEnd    : stands for VIF grab line end position.   
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_GetGrabPosition(	MMP_UBYTE ubVid,
                                    MMP_USHORT *usPixelStart, MMP_USHORT *usPixelEnd,
									MMP_USHORT *usLineStart, MMP_USHORT *usLineEnd)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif 

	#if (CHIP == MERCURY)
	if (ubVid == MMPF_VIF_MDL_ID2)
	{
	    *usPixelStart   = pVIF2->VIF_2_GRAB.PIXL_ST;
	    *usLineStart    = pVIF2->VIF_2_GRAB.LINE_ST;
	    *usPixelEnd     = pVIF2->VIF_2_GRAB.PIXL_ED;
	    *usLineEnd      = pVIF2->VIF_2_GRAB.LINE_ED;
	}
	else
	#endif
	{
	    *usPixelStart   = pVIF->VIF_GRAB[ubVid].PIXL_ST;
	    *usLineStart    = pVIF->VIF_GRAB[ubVid].LINE_ST;
	    *usPixelEnd     = pVIF->VIF_GRAB[ubVid].PIXL_ED;
	    *usLineEnd 		= pVIF->VIF_GRAB[ubVid].LINE_ED;
	}

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_GetGrabResolution
//  Description : This function get VIF grab width and height.
//------------------------------------------------------------------------------
/** 
 * @brief This function get VIF grab width and height.
 * 
 *  This function get VIF grab width and height.
 * @param[in]  ubVid     : stands for VIF module index. 
 * @param[out] ulWidth   : stands for VIF grab width.
 * @param[out] ulHeight  : stands for VIF grab height. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_GetGrabResolution(MMP_UBYTE ubVid, MMP_ULONG *ulWidth, MMP_ULONG *ulHeight)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
	#if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
	#endif 

	#if (CHIP == MERCURY)
	if (ubVid == MMPF_VIF_MDL_ID2)
	{
    	*ulWidth  = (pVIF2->VIF_2_GRAB.PIXL_ED - pVIF2->VIF_2_GRAB.PIXL_ST + 1);
    	*ulHeight = (pVIF2->VIF_2_GRAB.LINE_ED - pVIF2->VIF_2_GRAB.LINE_ST + 1);	
	}
	else
	#endif
    {
		*ulWidth  = (pVIF->VIF_GRAB[ubVid].PIXL_ED - pVIF->VIF_GRAB[ubVid].PIXL_ST + 1);
		*ulHeight = (pVIF->VIF_GRAB[ubVid].LINE_ED - pVIF->VIF_GRAB[ubVid].LINE_ST + 1);
	}
	
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetInterrupt
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_VIF_SetInterrupt(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_BOOL bEnable)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
    #endif
    AIT_REG_B   *pIntSr, *pIntEn;

    #if (CHIP == MERCURY)
    if (ubVid == MMPF_VIF_MDL_ID2) {
        pIntSr = &pVIF2->VIF_INT_CPU_SR_SNR2;
        pIntEn = &pVIF2->VIF_INT_CPU_EN_SNR2;
    }
    else
    #endif
    {
        pIntSr = &pVIF->VIF_INT_CPU_SR[ubVid];
        pIntEn = &pVIF->VIF_INT_CPU_EN[ubVid];
    }

    if (bEnable) {
        *pIntSr = ubFlag;
        *pIntEn |= ubFlag;
    }
    else {
        *pIntEn &= ~ubFlag;
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_WaitFrameSig
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_VIF_WaitFrameSig(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_USHORT usFrameCount)
{
    MMP_USHORT i;
    AITPS_VIF  pVIF = AITC_BASE_VIF;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
    #endif
    AIT_REG_B   *pIntSr;

    #if (CHIP == MERCURY)
    if (ubVid == MMPF_VIF_MDL_ID2) {
        pIntSr = &pVIF2->VIF_INT_CPU_SR_SNR2;
    }
    else
    #endif
    {
        pIntSr = &pVIF->VIF_INT_CPU_SR[ubVid];
    }

    for (i = 0; i < usFrameCount; i++) {
        *pIntSr = ubFlag;
        while (!(*pIntSr & ubFlag));
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_CheckFrameSig
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_VIF_CheckFrameSig(MMP_UBYTE ubVid, MMP_UBYTE ubFlag, MMP_USHORT usFrameCount)
{
    MMP_USHORT  i;
    MMP_BOOL    bEnable;
    AITPS_VIF   pVIF = AITC_BASE_VIF;
    #if (CHIP == MERCURY)
    AITPS_VIF_SNR2  pVIF2   = AITC_BASE_VIF_SNR2;
    #endif
    AIT_REG_B   *pIntSr;

    #if (CHIP == MERCURY)
    if (ubVid == MMPF_VIF_MDL_ID2) {
        pIntSr = &pVIF2->VIF_INT_CPU_SR_SNR2;
    }
    else
    #endif
    {
        pIntSr = &pVIF->VIF_INT_CPU_SR[ubVid];
    }

    if (ubFlag == VIF_INT_FRM_ST) {
    
        MMPF_VIF_IsInterfaceEnable(ubVid, &bEnable);

        for (i = 0; i < usFrameCount; i++) {          
            if (bEnable) {
                *pIntSr = ubFlag;
                while (!(*pIntSr & ubFlag));
            }
            else {
                MMPF_OS_Sleep(100);
            }
        }
    }
    else{    
        *pIntSr = ubFlag;
        while (!(*pIntSr & ubFlag));
    }

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetFormatAndPath
//  Description : This function set VIF input/output format and path.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF input/output format and path.
 * 
 *  This function set VIF input/output format and path.
 * @param[in] ubVid  : stands for VIF module index. 
 * @param[in] path   : stands for VIF path selection. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetFormatAndPath(MMP_UBYTE ubVid, MMPF_VIF_PATH path)
{
    AITPS_VIF  	pVIF 	= AITC_BASE_VIF;
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AITPS_MIPI  pMIPI 	= AITC_BASE_MIPI;
    #endif
    
    switch(path)
    {
        case MMPF_VIF_PATH1_BAYER_TO_ISP:
            pVIF->VIF_YUV_CTL[ubVid] &= ~(VIF_YUV_EN);
            pVIF->VIF_RAW_OUT_EN[ubVid] &= ~(VIF_2_RAW_EN);
            pVIF->VIF_RAW_OUT_EN[ubVid] |= VIF_2_ISP_EN;
        break;
        case MMPF_VIF_PATH2_YCbCr422_2_YCbCr444_BYPASS_ISP:
            pVIF->VIF_YUV_CTL[ubVid] |= VIF_YUV_EN;
        break;
        case MMPF_VIF_PATH3_YCbCr422_2_BAYER:
            pVIF->VIF_YUV_CTL[ubVid] |= (VIF_YUV_EN | VIF_Y2B_EN);
        break;
        case MMPF_VIF_PATH4_YCbCr422_2_YUV444:
            pVIF->VIF_YUV_CTL[ubVid] |= (VIF_YUV_EN | VIF_YUV_PATH_OUT_YUV);
        break;
        case MMPF_VIF_PATH5_BAYER_TO_RAWPROC:
            pVIF->VIF_YUV_CTL[ubVid] &= ~(VIF_YUV_EN);
            pVIF->VIF_RAW_OUT_EN[ubVid] &= ~(VIF_2_ISP_EN);
            pVIF->VIF_RAW_OUT_EN[ubVid] |= VIF_2_RAW_EN;
        break;
        case MMPF_VIF_PATH6_JPG2RAWPROC:
            pVIF->VIF_YUV_CTL[ubVid] &= ~(VIF_YUV_EN);
            pVIF->VIF_RAW_OUT_EN[ubVid] |= VIF_JPG_2_RAW_EN;        
        break;
        #if (CHIP == MCR_V2) || (CHIP == MERCURY)
        case MMPF_VIF_PATH7_YCbCr422_2_YCbCr420_TO_RAWPROC:
            pVIF->VIF_YUV420_CTL[ubVid] &= ~(VIF_MIPI_YUV420_EN);
            pVIF->VIF_YUV420_CTL[ubVid] |= VIF_PARA_YUV422T0420_EN;
        break;
        case MMPF_VIF_PATH8_VC_DATA:
            pMIPI->MIPI_VC_CTL[ubVid] |= MIPI_VC_EN;
        #endif
        break;                                                       
        default:
        break;
    }
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetFixedDataOut
//  Description : This function set VIF fixed data output.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF fixed data output.
 * 
 *  This function set VIF fixed data output.
 * @param[in] ubVid   : stands for VIF module index. 
 * @param[in] bEnable : stands for VIF fixed data output enable. 
 * @param[in] ubData  : stands for VIF fixed data. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetFixedDataOut(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMP_UBYTE ubData)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    if(bEnable == MMP_TRUE){
    
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_FIXED_OUT_EN;
        pVIF->VIF_FIXED_OUT_DATA[ubVid] = ubData;
    }
    else{    
        pVIF->VIF_SENSR_CTL[ubVid] &= ~(VIF_FIXED_OUT_EN);
        pVIF->VIF_FIXED_OUT_DATA[ubVid] = 0;
    }
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetColorID
//  Description : This function set VIF color ID (Pixel ID, Line ID).
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF color ID (Pixel ID, Line ID).
 * 
 *  This function set VIF color ID (Pixel ID, Line ID).
 * @param[in] ubVid : stands for VIF module index. 
 * @param[in] clrId : stands for VIF color ID index. 
 * @return It return the function status.  
 */
#if (OS_TYPE == OS_UCOSII)
MMP_ERR MMPF_VIF_SetColorID(MMP_UBYTE ubVid, MMPF_VIF_COLOR_ID clrId)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    AITPS_ISP pISP = AITC_BASE_ISP;

	pVIF->VIF_SENSR_CTL[ubVid] &= ~(VIF_COLORID_FORMAT_MASK);
    #if (CHIP == MCR_V2)
    pISP->ISP_COLOR_ID &= ~(ISP_VIF_COLORID_MASK);
    #endif
    
    switch(clrId) {
    case MMPF_VIF_COLORID_00:
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_COLORID_00;
        #if (CHIP == MCR_V2)
        pISP->ISP_COLOR_ID |= ISP_VIF_COLORID_00;
        #endif
        break;
    case MMPF_VIF_COLORID_01:
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_COLORID_01;
        #if (CHIP == MCR_V2)
        pISP->ISP_COLOR_ID |= ISP_VIF_COLORID_01;
        #endif
        break;
    case MMPF_VIF_COLORID_10:
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_COLORID_10;
        #if (CHIP == MCR_V2)
        pISP->ISP_COLOR_ID |= ISP_VIF_COLORID_10;
        #endif
        break;
    case MMPF_VIF_COLORID_11:
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_COLORID_11;
        #if (CHIP == MCR_V2)
        pISP->ISP_COLOR_ID |= ISP_VIF_COLORID_11;
        #endif
        break;
    }

    return MMP_ERR_NONE;
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetFrameSkip
//  Description : This function set VIF frame skip mechanism.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF frame skip mechanism.
 * 
 *  This function set VIF frame skip mechanism.
 * @param[in] ubVid     : stands for VIF module index. 
 * @param[in] bEnable   : stands for VIF frame skip enable. 
 * @param[in] ubSkipIdx : stands for VIF frame skip index. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetFrameSkip(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMP_UBYTE ubSkipIdx)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    if(bEnable == MMP_TRUE){
        pVIF->VIF_FRME_SKIP_EN[ubVid] |= VIF_FRME_SKIP_ENABLE;
        pVIF->VIF_FRME_SKIP_NO[ubVid] = ubSkipIdx;
    }
    else{
        pVIF->VIF_FRME_SKIP_EN[ubVid] &= ~(VIF_FRME_SKIP_ENABLE);
        pVIF->VIF_FRME_SKIP_NO[ubVid] = 0;
    }
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetDownSample
//  Description : This function set VIF down-sample mechanism.
//------------------------------------------------------------------------------
/** 
 * @brief This function set VIF down-sample mechanism.
 * 
 *  This function set VIF down-sample mechanism.
 * @param[in] ubVid    : stands for VIF module index. 
 * @param[in] bEnable  : stands for VIF down-sample enable. 
 * @param[in] ubHratio : stands for VIF horizontal down-sample ratio (0~3). 
 * @param[in] ubVratio : stands for VIF vertical down-sample ratio (0~3).
 * @param[in] bHsmooth : stands for VIF horizontal down-sample average enable. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetDownSample(MMP_UBYTE ubVid, MMP_BOOL bEnable, 
                               MMP_UBYTE ubHratio, MMP_UBYTE ubVratio, MMP_BOOL bHsmooth)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    if(bEnable == MMP_TRUE){
        pVIF->VIF_DNSPL_RATIO_CTL[ubVid] |= (ubHratio & VIF_DNSPL_H_RATIO_MASK);
        pVIF->VIF_DNSPL_RATIO_CTL[ubVid] |= (ubVratio<<2 & VIF_DNSPL_V_RATIO_MASK);
        
        if(bHsmooth)
            pVIF->VIF_DNSPL_RATIO_CTL[ubVid] |= VIF_DNSPL_H_AVG_EN;
        else
            pVIF->VIF_DNSPL_RATIO_CTL[ubVid] &= ~(VIF_DNSPL_H_AVG_EN);     
    }
    else{
        pVIF->VIF_DNSPL_RATIO_CTL[ubVid] = 0;
    }
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_AdjustInputPixel
//  Description : This function adjust indicated component input data value.
//------------------------------------------------------------------------------
/** 
 * @brief This function adjust indicated component input data value.
 * 
 *  This function adjust indicated component input data value.
 * @param[in] ubVid   : stands for VIF module index. 
 * @param[in] bEnable : stands for VIF adjust input data enable. 
 * @param[in] pOffset : stands for pointer to data offset structure. 
 * @return It return the function status.  
 */
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
MMP_ERR MMPF_VIF_AdjustInputPixel(MMP_UBYTE ubVid, MMP_BOOL bEnable, MMPF_VIF_DATA_OFFSET* pOffset)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    if(bEnable == MMP_TRUE){
    
        pVIF->VIF_COMP_ID00_OFFSET[ubVid] &= ~(VIF_COMP_OFFSET_SIGN_MASK);
        
        switch(pOffset->ubCompId){
        
            case MMPF_VIF_COLORID_00:
                pVIF->VIF_COMP_ID00_OFFSET[ubVid] = (pOffset->ubOffVal & VIF_COMP_OFFSET_MASK);
                pVIF->VIF_COMP_ID00_OFFSET[ubVid] |= ((pOffset->bPostive)?(VIF_COMP_OFFSET_SIGN_POS):(VIF_COMP_OFFSET_SIGN_NEG));
            break;
            case MMPF_VIF_COLORID_01:
                pVIF->VIF_COMP_ID01_OFFSET[ubVid] = (pOffset->ubOffVal & VIF_COMP_OFFSET_MASK);
                pVIF->VIF_COMP_ID01_OFFSET[ubVid] |= ((pOffset->bPostive)?(VIF_COMP_OFFSET_SIGN_POS):(VIF_COMP_OFFSET_SIGN_NEG));
            break;
            case MMPF_VIF_COLORID_10:
                pVIF->VIF_COMP_ID10_OFFSET[ubVid] = (pOffset->ubOffVal & VIF_COMP_OFFSET_MASK);
                pVIF->VIF_COMP_ID10_OFFSET[ubVid] |= ((pOffset->bPostive)?(VIF_COMP_OFFSET_SIGN_POS):(VIF_COMP_OFFSET_SIGN_NEG));
            break;            
            case MMPF_VIF_COLORID_11:
                pVIF->VIF_COMP_ID11_OFFSET[ubVid] = (pOffset->ubOffVal & VIF_COMP_OFFSET_MASK);
                pVIF->VIF_COMP_ID11_OFFSET[ubVid] |= ((pOffset->bPostive)?(VIF_COMP_OFFSET_SIGN_POS):(VIF_COMP_OFFSET_SIGN_NEG));
            break;        
        }
    }
    else{
        switch(pOffset->ubCompId){
        
            case MMPF_VIF_COLORID_00:
                pVIF->VIF_COMP_ID00_OFFSET[ubVid] = 0;
            break;
            case MMPF_VIF_COLORID_01:
                pVIF->VIF_COMP_ID01_OFFSET[ubVid] = 0;
            break;
            case MMPF_VIF_COLORID_10:
                pVIF->VIF_COMP_ID10_OFFSET[ubVid] = 0;
            break;            
            case MMPF_VIF_COLORID_11:
                pVIF->VIF_COMP_ID11_OFFSET[ubVid] = 0;
            break;        
        }    
    }
    return MMP_ERR_NONE;  
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetSensorMClockAttr
//  Description : This function set sensor clock relative attribute.
//------------------------------------------------------------------------------
/** 
 * @brief This function set sensor clock relative attribute.
 * 
 *  This function set sensor clock relative attribute.
 * @param[in] ubVid : stands for VIF module index. 
 * @param[in] pAttr : stands for pointer to sensor clock attribute structure. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetSensorMClockAttr(MMP_UBYTE ubVid, MMPF_VIF_MCLK_ATTR* pAttr)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    /* Set sensor main clock output enable */
    if(pAttr->bClkOutEn)
        pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_MCLK_EN;
    else    
        pVIF->VIF_SENSR_MCLK_CTL[ubVid] &= ~(VIF_SENSR_MCLK_EN);
    
    /* Set sensor main clock freqency */
    if(pAttr->ubClkFreqDiv != 0){
        pVIF->VIF_SENSR_CLK_FREQ[ubVid] = VIF_SENSR_CLK_PLL_DIV(pAttr->ubClkFreqDiv);
    }
    else{
        pVIF->VIF_SENSR_CLK_FREQ[ubVid] = VIF_SENSR_CLK_PLL_DIV((pAttr->ulDesiredFreq / pAttr->ulMClkFreq)); //TBD
    }

    /* Set sensor main clock phase */
    pVIF->VIF_SENSR_MCLK_CTL[ubVid] &= ~VIF_SENSR_PHASE_DELAY_MASK;
    
    switch(pAttr->ubClkPhase){
        case MMPF_VIF_SNR_PHASE_DELAY_NONE:
            pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_PHASE_DELAY_NONE;
        break;
        case MMPF_VIF_SNR_PHASE_DELAY_0_5F:
            pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_PHASE_DELAY_0_5F;
        break;
        case MMPF_VIF_SNR_PHASE_DELAY_1_0F:
            pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_PHASE_DELAY_1_0F;
        break;
        case MMPF_VIF_SNR_PHASE_DELAY_1_5F:
            pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_PHASE_DELAY_1_5F;
        break;                
    }
    
    /* Set sensor main clock polarity */
    if(pAttr->ubClkPolarity == MMPF_VIF_SNR_CLK_POLARITY_POS)
        pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_MCLK_POLAR_PST;
    else      
        pVIF->VIF_SENSR_MCLK_CTL[ubVid] |= VIF_SENSR_MCLK_POLAR_NEG;
        
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetParallelTimingAttr
//  Description : This function set parallel sensor timing relative attribute.
//------------------------------------------------------------------------------
/** 
 * @brief This function set parallel sensor timing relative attribute.
 * 
 *  This function set parallel sensor timing relative attribute.
 * @param[in] ubVid : stands for VIF module index. 
 * @param[in] pAttr : stands for pointer to parallel sensor timing attribute structure. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetParallelTimingAttr(MMP_UBYTE ubVid, MMPF_VIF_PARAL_ATTR* pAttr)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
        
    /* Set sensor pixel clock latch timing */
    if(pAttr->ubLatchTiming == MMPF_VIF_SNR_LATCH_POS_EDGE)
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_PCLK_LATCH_PST;
    else
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_PCLK_LATCH_NEG;                

    /* Set H-Sync signal polartiy */
    if(pAttr->ubHsyncPolarity == MMPF_VIF_SNR_CLK_POLARITY_POS)
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_HSYNC_POLAR_PST;
    else    
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_HSYNC_POLAR_NEG;

    /* Set V-Sync signal polartiy */
    if(pAttr->ubVsyncPolarity == MMPF_VIF_SNR_CLK_POLARITY_POS)
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_VSYNC_POLAR_PST;
    else    
        pVIF->VIF_SENSR_CTL[ubVid] |= VIF_VSYNC_POLAR_NEG;
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetSensorMipiAttr
//  Description : This function set sensor MIPI relative attribute.
//------------------------------------------------------------------------------
/** 
 * @brief This function set sensor MIPI relative attribute.
 * 
 *  This function set sensor MIPI relative attribute.
 * @param[in] ubVid : stands for VIF module index. 
 * @param[in] pAttr : stands for pointer to sensor MIPI attribute structure. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetSensorMipiAttr(MMP_UBYTE ubVid, MMPF_MIPI_RX_ATTR* pAttr)
{
    AITPS_MIPI  pMIPI = AITC_BASE_MIPI;
    MMP_UBYTE   i = 0;

    /* Set clock lane delay enable */
    if(pAttr->bClkDelayEn)
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_CLK_DLY_EN;
    else    
        pMIPI->MIPI_CLK_CFG[ubVid] &= ~(MIPI_CLK_DLY_EN);
        
    /* Set clock lane swap enable */
    if(pAttr->bClkLaneSwapEn)
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_CLK_LANE_SWAP_EN;
    else    
        pMIPI->MIPI_CLK_CFG[ubVid] &= ~(MIPI_CLK_LANE_SWAP_EN);
        
    /* Set clock lane delay */
    if(pAttr->bClkDelayEn)
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_CLK_DLY(pAttr->usClkDelay);
    else
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_CLK_DLY(0);
        
    /* Set byte clock latch timing */
    pMIPI->MIPI_CLK_CFG[ubVid] &= ~(MIPI_BCLK_LATCH_EDGE_MASK);
    
    if(pAttr->ubbClkLatchTiming == MMPF_VIF_SNR_LATCH_POS_EDGE)
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_BCLK_LATCH_EDGE_POS;
    else
        pMIPI->MIPI_CLK_CFG[ubVid] |= MIPI_BCLK_LATCH_EDGE_NEG;
    
    /* Set data lane configuration */
    for (i = 0; i< MAX_MIPI_DATA_LANE_NUM; i++)
    {
        if(pAttr->bDataLaneEn[i])
        {
            /* Set data lane enable */
            pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_LANE_EN;

            /* Set data lane delay enable */
            if(pAttr->bDataDelayEn[i])
                pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_DLY_EN;
            else    
                pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] &= ~(MIPI_DAT_DLY_EN);

            /* Set data lane swap enable */
            if(pAttr->bDataLaneSwapEn[i])
                pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_LANE_SWAP_EN;
            else    
                pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] &= ~(MIPI_DAT_LANE_SWAP_EN);
            
            /* Set data lane D-PHY source */
            pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] &= ~(MIPI_DAT_SRC_SEL_MASK);
                
            switch(pAttr->ubDataLaneSrc[i])
            {
                case 0:
                    pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_SRC_PHY_0;
                    break;
                case 1:
                    pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_SRC_PHY_1;
                    break;
                case 2:
                    pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_SRC_PHY_2;
                    break;
                case 3:
                    pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] |= MIPI_DAT_SRC_PHY_3;
                    break;        
            }
            
            /* Set data lane delay */
            if(pAttr->bDataDelayEn[i])
                pMIPI->DATA_LANE[i].MIPI_DATA_DELAY[ubVid] |= MIPI_DATA_DLY(pAttr->usDataDelay[i]);
            else    
                pMIPI->DATA_LANE[i].MIPI_DATA_DELAY[ubVid] &= ~(MIPI_DATA_DLY_MASK);
            
        	/* Set data lane SOT counter */
        	pMIPI->DATA_LANE[i].MIPI_DATA_DELAY[ubVid] |= MIPI_DATA_SOT_CNT(pAttr->ubDataSotCnt[i]);
        	
        }
        else{    
            pMIPI->DATA_LANE[i].MIPI_DATA_CFG[ubVid] &= ~(MIPI_DAT_LANE_EN);
        }
    }
        
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_VIF_SetIGBT
//  Description : This function set IGBT relative attribute.
//------------------------------------------------------------------------------
/** 
 * @brief This function set IGBT relative attribute.
 * 
 *  This function set IGBT relative attribute.
 * @param[in] ubVid : stands for VIF module index. 
 * @param[in] pAttr : stands for pointer to IGBT attribute structure. 
 * @return It return the function status.  
 */
MMP_ERR MMPF_VIF_SetIGBT(MMP_UBYTE ubVid, MMPF_VIF_IGBT_ATTR* pAttr)
{
    AITPS_VIF pVIF = AITC_BASE_VIF;
    
    /* Set IBGT enable */
    if(pAttr->bEnable == MMP_TRUE)
        pVIF->VIF_IGBT_EN[ubVid] |= VIF_IGBT_OUT_EN;
    else
        pVIF->VIF_IGBT_EN[ubVid] &= ~(VIF_IGBT_OUT_EN);               
    
    /* Set IBGT start line number */
    pVIF->VIF_IGBT_LINE_ST[ubVid] = pAttr->usStartLine; 
    
    /* Set IBGT start offset cycle number [Unit:16 x VI clock cycle] */
    pVIF->VIF_IGBT_OFST_ST[ubVid] = pAttr->usStartOfst; 

    /* Set IBGT end cycle number [Unit:16 x VI clock cycle] */
    pVIF->VIF_IGBT_LINE_CYC[ubVid] = pAttr->usEndCycle; 
    
    return MMP_ERR_NONE;
}

/** @}*/ //end of MMPF_VIF
