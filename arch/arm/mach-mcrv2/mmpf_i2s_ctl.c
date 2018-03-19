
#include "linux/module.h"

#include "mmpf_typedef.h"
#include "mmpf_i2s_ctl.h"
#include "mmp_reg_audio.h"

#include "mmpf_pll.h"


#define I2S_OUT_EN 1
//------------------------------------------------------------------------------
//  Function    : MMPF_SetI2SFreq
//  Parameter   :
//          freq--sampling rate
//  Return Value : None
//  Description : Set i2s mclk
//------------------------------------------------------------------------------

/** @addtogroup MMPF_I2S
@{
*/

void MMPF_SetI2SFreq(MMP_ULONG freq)
{
	AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	MMP_USHORT /*mn_value,*/ ratioM, ratioN;
	MMP_ULONG audClk;
	MMP_UBYTE clkDiv/*, cuer_div*/;

    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &audClk);


    /* ********************************************************************* */
    // Wilson@120626 NOTE:
    // The MN value calculation is for general case,
    // you may need to modify MN value to meet the requirement for different CODEC
    // For example:
    // For DA7211, 16k, 32k, 48k sample rate, they are belong to same MCLK group;
    // the freq variable should set to 48k for these sample rates.
    /* ********************************************************************* */

    freq = 16000;//0xB71B; //12M/256

    audClk /= 2;
    audClk = 48000;
    clkDiv =(MMP_UBYTE)((audClk * 1000)/(256*freq));
    ratioM = (MMP_UBYTE)((audClk * 1000)/freq - clkDiv * 256);
    ratioN = 256 - ratioM;

    pAUD->I2S_CLK_CTL |= I2S_MCLK_FIX;
    pAUD->I2S_CLK_DIV = 4;//clkDiv;		//	48M
    pAUD->I2S_RATIO_N_M =  ratioM << 8 | ratioN;
    pAUD->I2S_MCLK_CTL = I2S_256_FS;

	return;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SetI2SMode
//  Parameter   :
//          i2s_mode--
//          lrck_mode--
//          output_bits--
//  Return Value : None
//  Description : Set i2s interface format
//------------------------------------------------------------------------------
#if 0
void	MMPF_SetI2SMode(MMP_USHORT i2s_mode, MMP_USHORT lrck_mode, MMP_USHORT output_bits)
{
	AITPS_I2S pAUD = AITC_BASE_I2S0;

//	pAUD->I2S_OUT_MODE_CTL = i2s_mode;//pAUD->I2S_OUT_MODE_CTL = i2s_mode;

//	pAUD->I2S_LRCK_POL = lrck_mode;
//	pAUD->I2S_BIT_CLT = output_bits;	pAUD->I2S_BIT_CLT = output_bits;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PresetI2S
//  Parameter   :
//          direction--specify i2s in or out
//          mode--specify master or slave mode
//  Return Value : None
//  Description : Set i2s data direction
//------------------------------------------------------------------------------
void	MMPF_PresetI2S(MMP_USHORT direction,MMP_USHORT mode,MMP_USHORT alignment)
{
    AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	#if defined(I2S_SWITCH_1DOT8_MODE) 					
    AITPS_GBL   pGBL = AITC_BASE_GBL;
	#endif    

	//pAUD->I2S_MCLK_CTL = I2S_256_FS;

//	pAUD->I2S_MODE_CTL = (mode | I2S_MCLK_OUT_EN);

	#if defined(I2S_SWITCH_1DOT8_MODE) 		
	x
	    pGBL->GBL_HOST_CTL |=(0x80); 
	#endif

//       pAUD->I2S_DATA_IN_SEL = 0;

//	pAUD->I2S_DATA_OUT_PAD_CTL = 0x1;
	
        //pAUD->I2S_CTL = I2S_SDO_OUT_EN| I2S_HCK_CLK_EN;		

	return ;



    switch (direction) {
    
    #if (I2S_OUT_EN == 1) //wilson: for VSN_V3
	    case I2S_OUT:
		pAUD->I2S_DATA_OUT_PAD_CTL = 0x1;//	        pAUD->I2S_DATA_OUT_EN = 1;
	        pAUD->I2S_CTL = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN | I2S_HCK_CLK_EN);
	        break;
	#endif
    
    case I2S_IN:
    #if 0//PCAM_EN==0
        pAUD->I2S_DATA_IN_SEL = I2S_SDI_IN;
        
        #if (I2S_OUT_EN == 1) //wilson: for VSN_V3
        	pAUD->I2S_DATA_OUT_EN = 0;
		#endif
		
        pAUD->I2S_CTL = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN | I2S_HCK_CLK_EN);//enable sdo,lrck,bck,hck
    #else
//    	RTNA_DBG_Str(0, "set i2s alignment, sdi in\r\n"); 
        pAUD->I2S_DATA_IN_SEL = 0;//I2S_SDO_IN;//0;//I2S_SDI_IN;
        
        #if (I2S_OUT_EN == 1) //wilson: for VSN_V3
		pAUD->I2S_DATA_OUT_PAD_CTL = 0x1;//	        pAUD->I2S_DATA_OUT_EN = 1;		pAUD->I2S_DATA_OUT_PAD_CTL = 0x0;//	        pAUD->I2S_DATA_OUT_EN = 0;
		#endif
		
        pAUD->I2S_CTL = (I2S_SDO_OUT_EN | I2S_LRCK_OUT_EN | I2S_BCK_OUT_EN | I2S_HCK_CLK_EN);//pAUD->I2S_CTL =I2S_SDO_OUT_EN| (I2S_HCK_CLK_EN);//enable hck
    #endif
        break;
    }
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_CloseI2S
//  Parameter   : None
//  Return Value : None
//  Description : Disable i2s
//------------------------------------------------------------------------------
void MMPF_CloseI2S(void)
{
	AITPS_I2S   pAUD    = AITC_BASE_I2S0;
	
	pAUD->I2S_CTL = I2S_ALL_DIS;
}
/** @} */ // end of MMPF_I2S
