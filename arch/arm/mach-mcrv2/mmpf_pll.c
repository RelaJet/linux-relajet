//==============================================================================
//
//  File        : mmpf_pll.c
//  Description : Firmware PLL Control Function
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================
#include <linux/module.h>
#include "includes_fw.h"
#include "mmpf_pll.h"
#include "mmp_reg_gbl.h"
#include "lib_retina.h"
//#include "reg_retina.h"
#include "mmpf_dram.h"
/** @addtogroup MMPF_PLL
@{
*/
//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================
#if (CHIP == VSN_V2)
#define MAX_DPLL_SRC	2
#endif

#if (CHIP == VSN_V3)
#define MAX_DPLL_SRC	3
#endif

#if (CHIP == MERCURY)
#define MAX_DPLL_SRC    5
#endif

#define DPLL_3A
//==============================================================================
//
//                              VARIABLES
//
//==============================================================================

#if (CHIP == VSN_V3) || (CHIP == VSN_V2)

/*ATTENTION2: 	Analog team suggest that PLL1's M valuse as "1" or "2" is the best*/
MMP_UBYTE MMPF_PLL_TABLE_VSN_V2[MMPF_PLL_FREQ_TOTAL_NO][6]=
//Note: "N" for PLL0 should fill the value "N - 1", for PLL1 should fill "N-2", "M" is only used in PLL1
// M,	N,					K(P),					DPLL_F,					AudioDiv, 	PostDiv 
{{0,	100,				PLL0_CLK_DIV(2),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_600MHz
 {0,	 83,				PLL0_CLK_DIV(2),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_498MHz
 {0,	100,				PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_400MHz
 {0,	111,				PLL0_CLK_DIV(4),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_333MHz
 {0,	83,					PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_332MHz
 {0,	100,				PLL0_CLK_DIV(4),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_300MHz
 {0,	89,					PLL0_CLK_DIV(4),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_266MHz
 {1,	PLL1_CLK_N(44),		PLL1_CLK_DIV_1,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_264MHz_PLL1
 {0,	80,					PLL0_CLK_DIV(4),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(1)},   //FREQ_240MHz_PLL0
 {1,	PLL1_CLK_N(40),		PLL1_CLK_DIV_1,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_240MHz_PLL1
 //Post-divider input clock must less than 470MHz, and to take it into consideration, our PostDiv does not 
 //take the "Fixed divid 2" of Global_clk and dram clock(for the clock larger than 240 MHz). So, if the 
 //Global or Dram clock share the same CPU pll source, please double check this.
 
 {0,	108,				PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //MMPF_PLL_FREQ_216MHz_PLL0
  {1,	PLL1_CLK_N(36),		PLL1_CLK_DIV_1,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //MMPF_PLL_FREQ_216MHz_PLL1
 {0,	100,				PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_200MHz_PLL0
 {3,	PLL1_CLK_N(100),	PLL1_CLK_DIV_1,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_200MHz_PLL1
  {0,	96,					PLL0_CLK_DIV(6),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_192MHz_PLL0
 {2,	PLL1_CLK_N(64),		PLL1_CLK_DIV_1,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_192MHz_PLL1
 {0,	112,				PLL0_CLK_DIV(4),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_168MHz
 {0,	83,					PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_166MHz_PLL0
 {3,	PLL1_CLK_N(166),	PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_166MHz_PLL1
 {0,	81,					PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_162MHz
 {0,	78,					PLL0_CLK_DIV(3),		PLL_500_1500_1000MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_156MHz
 {1,	PLL1_CLK_N(48),		PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_144MHz
 {3,	PLL1_CLK_N(133),	PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_133MHz
 {1,	PLL1_CLK_N(44),		PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_132MHz
 {1,	PLL1_CLK_N(40),		PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_120MHz
 {1,	PLL1_CLK_N(32),		PLL1_CLK_DIV_2,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_96MHz
 {1,	PLL1_CLK_N(40),		PLL1_CLK_DIV_4,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_60MHz
 {1,	PLL1_CLK_N(36),		PLL1_CLK_DIV_4,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_54MHz
 {1,	PLL1_CLK_N(32),		PLL1_CLK_DIV_4,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_48MHz
 {1,	PLL1_CLK_N(52),		PLL1_CLK_DIV_8,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_39MHz
 {1,	PLL1_CLK_N(32),		PLL1_CLK_DIV_8,			PLL_300_700_500MHZ, 	0, 			GROUP_CLK_DIV(2)},   //FREQ_24MHz
};

MMP_ULONG        		glGroupFreq[MAX_GROUP_NUM];
#if (OS_TYPE == OS_UCOSII) //temp solution, FIXME
static MMPF_GROUP_SRC   m_group_src[MAX_GROUP_NUM];
static MMPF_PLL_MODE    m_pll_mode = MMPF_PLL_MODE_NUMBER;
static MMPF_PLL_FREQ    m_pll_freq[MMPF_PLL_ID_MAX];
static MMPF_PLL_ID      m_cpu_src = MMPF_PLL_ID_0;
static MMP_ULONG		m_pll_out_clk[MMPF_PLL_ID_MAX] = {0};
#endif

MMP_ULONG               glCPUFreq = 0;
MMP_ULONG               glMemFreq = 0;

#endif //(CHIP == VSN_V3) || (CHIP == VSN_V2)

#if (CHIP == MERCURY)

//notes: mercury EXT_PMCLK_CKL is 24Mhz
const static MMPF_PLL_SETTINGS m_PllSettings[6] = {
    //{M, N, DIV, Frac, VCO, Freq}
    //PLL0 528MHz
    {PLL0_M_DIV_1, PLL0_CLK_N(22), PLL_CLK_DIV_1, 0, PLL_200_500_350MHZ,   528000},
    //PLL1 192MHz
    {PLL1_M_DIV_1, PLL1_CLK_N(16), PLL_CLK_DIV_2, 0, PLL_200_500_350MHZ,   192000},
    //PLL2 432MHz
    {PLL2_M_DIV_1, PLL2_CLK_N(18), PLL_CLK_DIV_1, 0, PLL_200_500_350MHZ,   432000},
    //MCLK
    {0, 0, 0, 0, 0, EXT_PMCLK_CKL},
    //PLL3
    {0, 0, 0, 0, 0, EXT_PMCLK_CKL},
    //{PLL3_CLK_M(98), PLL3_CLK_N(16), NULL, 0x2E6BE, NULL, 4096},
    //PLL4
    {0, 0, 0, 0, 0, EXT_PMCLK_CKL}
    //{PLL4_CLK_M(23), PLL4_CLK_N(192), PLL_VCO4_DIV_2, NULL, NULL, 400000}
};

const static MMPF_SYS_CLK_SRC m_ClockSrc = {
    //PLL source        //clock div
    MMPF_PLL_ID_0,      1,          //CPUA    528MHz
    MMPF_PLL_ID_0,      1,          //CPUB    528MHz
    MMPF_PLL_ID_0,      2,          //Global  264MHz
    MMPF_PLL_ID_1,      1,          //DRAM    192MHz
    MMPF_PLL_ID_PMCLK,  2,          //USB PHY 12MHz
    MMPF_PLL_ID_PMCLK,  1,          //RX BIST 24MHz
    MMPF_PLL_ID_0,      2,          //Sensor  264MHz
    MMPF_PLL_ID_0,      11,         //Audio   48MHz
    #if (SPEED_UP_SCA_CLK)
    MMPF_PLL_ID_0,      1,          //ISP     264MHz
    #else
    MMPF_PLL_ID_0,      2,          //ISP     264MHz
    #endif
    MMPF_PLL_ID_2,      1,          //Bayer   432MHz
    MMPF_PLL_ID_PMCLK,  1           //mipitx  24MHz
};

MMP_ULONG               glCPUAFreq = 0;
MMP_ULONG               glCPUBFreq = 0;

MMP_ULONG               glGroupFreq[MAX_GROUP_NUM];

#endif // (CHIP == MERCURY)


#if (CHIP == MCR_V2)

MMP_ULONG               gCPUFreq = 0;
MMP_ULONG               glCPUAFreq = 0;
MMP_ULONG               glCPUBFreq = 0;
MMP_ULONG               glGroupFreq[MAX_GROUP_NUM];
MMP_UBYTE               m_bAudSampleGroup = 0;

#define GET_PLL_FRAC(r)    (r[0] | (r[1] << 8) | (r[2] << 16))
#define FRAC_RANGE	(256*4)

static MMP_ULONG m_AudioPllFrac = 0;
static MMP_ULONG m_AudioPllFracCur = 0;
static MMP_ULONG m_AudioPllFracMax = 0;
static MMP_ULONG m_AudioPllFracMin = 0;

//notes: mercury MMPF_PLL_SETTINGS EXT_CLK is 24Mhz
const static  MMPF_PLL_SETTINGS m_PllSettings[7] = {
    //{M, N, DIV, Frac, VCO, Freq}
    //PLL0 600Hz
    {DPLL0_M_DIV_1, DPLL0_N(24), DPLL_FVCO_DIV_1, 0, DPLL_384_1008MHZ,  600000},
    //PLL1 480MHz
    {DPLL1_M_DIV_1, DPLL1_N(21), DPLL_FVCO_DIV_1, 0, DPLL_384_1008MHZ,  528000},
    //PLL2 360MHz
    {DPLL2_M_DIV_1, DPLL2_N(14), DPLL_FVCO_DIV_1, 0, DPLL_336_792MHZ,   360000},
    //PMCLK
    {0, 0, 0, 0, 0, EXT_CLK},
    //PLL3 24.576MHz for Audio
    {DPLL3_M(19), DPLL3_N(19), 0, 0, 0, 24576},
    //PLL4 378MHz for TV
    //{DPLL4_M(6), DPLL4_N(24), DPLL4_R(0,0,0), 0, DPLL_336_792MHZ, 400000},
    {DPLL4_M(23), DPLL4_N(188), DPLL4_P_DIV_2, 0, DPLL4_400MHZ, 378000},
    //PLL5 400MHz,for dram
    {DPLL5_OUTPUT_DIV_2, DPLL5_N(16), DPLL5_P_DIV_1, 0, DPLL5_800MHZ, 400000}
};

const static MMPF_SYS_CLK_SRC m_ClockSrc = {
    //PLL source        //clock div
    MMPF_PLL_ID_0,      1,          //CPUA    600MHz
    MMPF_PLL_ID_0,      1,          //CPUB    600MHz
    MMPF_PLL_ID_1,      2,          //Global  264MHz
    #if (DRAM_ID == DRAM_DDR)
    MMPF_PLL_ID_5,      2,          //DRAM    200MHz
    #endif
    #if (DRAM_ID == DRAM_DDR3)
    MMPF_PLL_ID_5,      1,          //DRAM    400MHz
    #endif
    MMPF_PLL_ID_PMCLK,  2,          //USB PHY 12MHz
    MMPF_PLL_ID_PMCLK,  1,          //RX BIST 24MHz
    MMPF_PLL_ID_2,      1,          //Sensor  360MHz
    MMPF_PLL_ID_3,      1,          //Audio   24MHz
    #if (BIND_SENSOR_OV10822)
    MMPF_PLL_ID_1,      1,          //ISP     528MHz
    #elif (BIND_SENSOR_AR0330)
    MMPF_PLL_ID_5,      1,          //ISP     360MHz
    #else
    MMPF_PLL_ID_2,      1,          //ISP     360MHz
    #endif
    #if (BIND_SENSOR_OV10822)
    MMPF_PLL_ID_0,      1,          //Bayer   600MHz
    #else
    MMPF_PLL_ID_1,      1,          //Bayer   528MHz
    #endif
    MMPF_PLL_ID_PMCLK,  1,          //mipitx  24MHz
    MMPF_PLL_ID_4,      2           //hmdi    189MHz
};
#endif //(CHIP == MCR_V2)


//==============================================================================
//
//                              FUNCTION
//
//==============================================================================

#if (OS_TYPE == OS_UCOSII)
#pragma O0
#endif
void MMPF_PLL_WaitCount(MMP_ULONG count)
{
    while ((*(volatile MMP_ULONG *)&count)--);
}
#if (OS_TYPE == OS_UCOSII)
#pragma
#endif


//------------------------------------------------------------------------------
//  Function    : MMPF_PLL_PowerUp
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_PLL_PowerUp(MMPF_PLL_NO PLLNo, MMP_BOOL bPowerUp)
{
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_USHORT      usWaitCycle = 50;

    #if (CHIP == VSN_V2)
    if(bPowerUp) {
        pGBL->GBL_DPLL_PWR &= (~(DPLL_PWR_DOWN_EN << PLLNo));
        MMPF_PLL_WaitCount(usWaitCycle);
    }
    else {
        pGBL->GBL_DPLL_PWR |= (DPLL_PWR_DOWN_EN << PLLNo);
        MMPF_PLL_WaitCount(usWaitCycle);
    }
    #endif

    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
    if(bPowerUp) {
        if((PLLNo == MMPF_PLL_0)||(PLLNo == MMPF_PLL_1)) {
            pGBL->GBL_DPLL01_PWR &= (~(DPLL_PWR_DOWN_EN << PLLNo));
        }
        else { //DPLL2
            pGBL->GBL_DPLL2_PWR &= (~(DPLL_PWR_DOWN_EN));
        }
        MMPF_PLL_WaitCount(usWaitCycle);
    }
    else {
        if((PLLNo == MMPF_PLL_0)||(PLLNo == MMPF_PLL_1)) {
            pGBL->GBL_DPLL01_PWR |= (DPLL_PWR_DOWN_EN << PLLNo);
        }
        else { //DPLL2
            pGBL->GBL_DPLL2_PWR |= (DPLL_PWR_DOWN_EN);
        }
        MMPF_PLL_WaitCount(usWaitCycle);
    }
    #endif

    #if (CHIP == MCR_V2)
    MMP_UBYTE       offset = 0;

    switch(PLLNo){
    case MMPF_PLL_1:
        offset = 0x10;
        break;
    case MMPF_PLL_2:
        offset = 0x20;
        break;
    case MMPF_PLL_3:
        offset = 0x30;
        break;
    case MMPF_PLL_4:
        offset = 0x40;
        break;
    case MMPF_PLL_5:
        offset = 0x50;
        break;

    default:
        break;
    }

    if(bPowerUp){
        *(AIT_REG_B *)((MMP_ULONG)&pGBL->GBL_DPLL0_PWR + offset) &= (~(DPLL_PWR_DOWN));

        /*if((PLLNo == MMPF_PLL_ID_0) || (PLLNo == MMPF_PLL_ID_1)){
            pGBL->GBL_DPLL0_PWR |= DPLL_PWR_UP
        }
        else{
            //DPLL2
            pGBL->GBL_DPLL2_PWR &= (~(DPLL_PWR_DOWN_EN));
        }*/
        MMPF_PLL_WaitCount(usWaitCycle);
    }
    else{
        *(AIT_REG_B *)((MMP_ULONG)&pGBL->GBL_DPLL0_PWR + offset) |= DPLL_PWR_DOWN;
        /*if((PLLNo == MMPF_PLL_ID_0) || (PLLNo == MMPF_PLL_ID_1)){
            pGBL->GBL_DPLL01_PWR |= (DPLL_PWR_DOWN_EN << m_pll_id);
        }
        else{
            //DPLL2
            pGBL->GBL_DPLL2_PWR |= (DPLL_PWR_DOWN_EN);
        }*/
        MMPF_PLL_WaitCount(usWaitCycle);
    }

    return MMP_ERR_NONE;
    #endif

    return MMP_ERR_NONE;
}

/** @brief The function get the CPU frequence.
@param[out] ulCPUFreq 1000*(CPU frequency) (ex: 133MHz -> 133000)
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PLL_GetCPUFreq(MMP_ULONG *ulCPUFreq)
{
    #if (CHIP == VSN_V3)
    *ulCPUFreq = glCPUFreq;
    #endif

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)

    #if (CPU_ID == CPU_A)
    *ulCPUFreq = glCPUAFreq;
    #endif
    #if (CPU_ID == CPU_B)
    *ulCPUFreq = glCPUBFreq;
    #endif

    #endif // #if (CHIP == MCR_V2)

    return MMP_ERR_NONE;
}

/** @brief The function get the Group frequence.
@param[in] ubGroupNum select to get group number (ex: 0 -> group 0)
@param[out] ulGroupFreq 1000*(GX frequency) (ex: 133MHz -> 133000)
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP GroupNum, MMP_ULONG *ulGroupFreq)
{
    #if (OS_TYPE == OS_LINUX) //temp solution, FIXME

    #if (CHIP == VSN_V3)
    switch (GroupNum) {
    case MMPF_CLK_GRP_DRAM:
        *ulGroupFreq = 180000;
        break;
    case MMPF_CLK_GRP_USB:
        *ulGroupFreq = 12000;
        break;
    case MMPF_CLK_GRP_GBL:
    default:
        *ulGroupFreq = 264000;
        break;
    }
    #endif

    #if (CHIP == MERCURY)
    TODO
    #endif

    #if (CHIP == MCR_V2)
	extern MMP_ERR MMPF_PLL_MCRV2_GetGroup0Freq(MMP_ULONG *ulGroupFreq);
    switch (GroupNum) {
    case MMPF_CLK_GRP_DRAM:
        *ulGroupFreq = 200000;
        break;
    case MMPF_CLK_GRP_USB:
        *ulGroupFreq = 12000;
        break;
    case MMPF_CLK_GRP_SENSOR:
        *ulGroupFreq = 360000;
        break;
    case MMPF_CLK_GRP_AUDIO:
        *ulGroupFreq = 24000;
        break;
    case MMPF_CLK_GRP_COLOR:
        *ulGroupFreq = 360000;
        break;
    case MMPF_CLK_GRP_GBL:
    default:
        //*ulGroupFreq = 264000;
        //*ulGroupFreq = 200000;
        //*ulGroupFreq = CONFIG_G0_CLOCK_HZ;
	MMPF_PLL_MCRV2_GetGroup0Freq(ulGroupFreq);
        break;
    }
    #endif

    #endif //(OS_TYPE == OS_LINUX)


    #if (OS_TYPE == OS_UCOSII)
    *ulGroupFreq = glGroupFreq[ubGroupNum];
    #endif

    return MMP_ERR_NONE;
}


#if (OS_TYPE == OS_UCOSII) //temp solution, FIXME
//------------------------------------------------------------------------------
//  Function    : MMPF_PLL_Setting
//  Description : 
//------------------------------------------------------------------------------
#if (CHIP == VSN_V3)
MMP_ERR MMPF_PLL_Setting(MMPF_PLL_MODE target_pll_mode, MMP_BOOL KeepG0)
{
	MMP_BYTE i;
	MMP_USHORT	usWaitCycle = 50;
	MMPF_PLL_SRC	m_pll_src[3], m_pll_src_temp;
	AITPS_GBL pGBL = AITC_BASE_GBL;
	
	if (m_pll_mode == target_pll_mode) {
//        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Used original PLL setting.\r\n");
        return MMP_ERR_NONE;
    }

	/*================Note=================================
	Frequncy more than 156MHz should be assigned to DPLL0
	Frequncy less than 144MHz should be assigned to DPLL1
	=====================================================*/
	switch (target_pll_mode) {
	case MMPF_PLL_96CPU_96G0134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_EXT_CLK;
		m_pll_freq[1] = MMPF_PLL_FREQ_96MHz;
		m_pll_freq[2] = MMPF_PLL_FREQ_EXT_CLK;
		
		m_pll_src[0]   = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1]   = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2]   = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_1;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = MMPF_GUP_SRC_PLL_1;
        m_group_src[4] = MMPF_GUP_SRC_PLL_1;
        m_group_src[5] = MMPF_GUP_SRC_PLL_1;
        m_group_src[6] = MMPF_GUP_SRC_PLL_1;
        
        glCPUFreq = 96000;
        glGroupFreq[0] = 96000;
        glGroupFreq[1] = 96000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = 96000;
        glGroupFreq[4] = 96000;
        glGroupFreq[5] = 96000;
        glGroupFreq[6] = 96000;
		break;
	case MMPF_PLL_144CPU_144G0134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_EXT_CLK;
		m_pll_freq[1] = MMPF_PLL_FREQ_144MHz;
		m_pll_freq[2] = MMPF_PLL_FREQ_EXT_CLK;
		
		m_pll_src[0]   = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1]   = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2]   = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_1;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = MMPF_GUP_SRC_PLL_1;
        m_group_src[4] = MMPF_GUP_SRC_PLL_1;
        m_group_src[5] = MMPF_GUP_SRC_PLL_1;
        m_group_src[6] = MMPF_GUP_SRC_PLL_1;
        
        glCPUFreq = 144000;
        glGroupFreq[0] = 144000;
        glGroupFreq[1] = 144000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = 144000;
        glGroupFreq[4] = 144000;
        glGroupFreq[5] = 144000;
        glGroupFreq[6] = 144000;
		break;
    case MMPF_PLL_332CPU_192G034_166G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_332MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_EXT_CLK;

		m_pll_src[0]   = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1]   = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2]   = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_0;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = MMPF_GUP_SRC_PLL_1;
        m_group_src[4] = MMPF_GUP_SRC_PLL_1;
        m_group_src[5] = MMPF_GUP_SRC_PLL_1;
        m_group_src[6] = MMPF_GUP_SRC_PLL_1;
        
        glCPUFreq = 332000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 166000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = 192000;
        glGroupFreq[4] = 192000;
        glGroupFreq[5] = 192000;
        glGroupFreq[6] = 192000;
		break;
    case MMPF_PLL_400CPU_166G0134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_400MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_216MHz_PLL1;
        m_pll_freq[2] = MMPF_PLL_FREQ_192MHz_PLL1;
        
		m_pll_src[0]   = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1]   = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2]   = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 400000;
        glGroupFreq[0] = 216000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_400CPU_192G034_200G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_400MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_EXT_CLK;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_0;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 400000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 200000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_400CPU_240G034_192G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_400MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_240MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_192MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 400000;
        glGroupFreq[0] = 240000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_498CPU_216G034_192G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_498MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_216MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_192MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 498000;
        glGroupFreq[0] = 216000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_498CPU_264G034_192G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_498MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_264MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_192MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 498000;
        glGroupFreq[0] = 264000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_498CPU_192G034_200G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_498MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_200MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 498000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 200000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_600CPU_192G034_166G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_166MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 166000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_600CPU_192G0134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_EXT_CLK;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	case MMPF_PLL_600CPU_216G0_192G134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_216MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_2;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[1];
        m_group_src[4] = m_group_src[1];
        m_group_src[5] = m_group_src[1];
        m_group_src[6] = m_group_src[1];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 216000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[1];
        glGroupFreq[4] = glGroupFreq[1];
        glGroupFreq[5] = glGroupFreq[1];
        glGroupFreq[6] = glGroupFreq[1];
		break;
	case MMPF_PLL_600CPU_240G0_192G134_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_240MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_2;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[1];
        m_group_src[4] = m_group_src[1];
        m_group_src[5] = m_group_src[1];
        m_group_src[6] = m_group_src[1];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 240000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[1];
        glGroupFreq[4] = glGroupFreq[1];
        glGroupFreq[5] = glGroupFreq[1];
        glGroupFreq[6] = glGroupFreq[1];
		break;
	case MMPF_PLL_600CPU_264G0_192G134_X:
        #if 1
        m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_264MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_2;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[1];
        m_group_src[4] = m_group_src[1];
        m_group_src[5] = m_group_src[1];
        m_group_src[6] = m_group_src[1];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 264000;
        glGroupFreq[1] = 192000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[1];
        glGroupFreq[4] = glGroupFreq[1];
		#if FxxxxN == 1
		glGroupFreq[4] = glGroupFreq[0];
		#endif
        glGroupFreq[5] = glGroupFreq[1];
        glGroupFreq[6] = glGroupFreq[1];
        #else
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_200MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_264MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_2;
        m_group_src[1] = MMPF_GUP_SRC_PLL_1;
        
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 264000;
        glGroupFreq[1] = 200000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
		#if FxxxxN == 1
		glGroupFreq[4] = glGroupFreq[0];
		#endif
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
        #endif
		break;
	case MMPF_PLL_600CPU_192G034_200G1_X:
		m_pll_freq[0] = MMPF_PLL_FREQ_600MHz;
		m_pll_freq[1] = MMPF_PLL_FREQ_192MHz_PLL1;
		m_pll_freq[2] = MMPF_PLL_FREQ_200MHz_PLL1;
		
		m_pll_src[0] = MMPF_PLL_SRC_PMCLK;	
        m_pll_src[1] = MMPF_PLL_SRC_PMCLK;
        m_pll_src[2] = MMPF_PLL_SRC_PMCLK;
        
        m_cpu_src = MMPF_GUP_SRC_PLL_0;
        m_group_src[0] = MMPF_GUP_SRC_PLL_1;
        m_group_src[1] = MMPF_GUP_SRC_PLL_2;
        m_group_src[2] = MMPF_GUP_SRC_PMCLK;
        m_group_src[3] = m_group_src[0];
        m_group_src[4] = m_group_src[0];
        m_group_src[5] = m_group_src[0];
        m_group_src[6] = m_group_src[0];
        
        glCPUFreq = 600000;
        glGroupFreq[0] = 192000;
        glGroupFreq[1] = 200000;
        glGroupFreq[2] = EXT_PMCLK_CKL;
        glGroupFreq[3] = glGroupFreq[0];
        glGroupFreq[4] = glGroupFreq[0];
        glGroupFreq[5] = glGroupFreq[0];
        glGroupFreq[6] = glGroupFreq[0];
		break;
	default:
		return MMP_ERR_NONE;
		break;
	}
	
	glMemFreq =  glGroupFreq[1];
	
	#if defined(ALL_FW)
	RTNA_DBG_Open(((glGroupFreq[0]/1000) >> 1), 115200);
	return MMP_ERR_NONE;
	#endif
	
	
	
	
	//Step 1: Switch to external clock (PMCLK), Bypass DPLL0 & DPLL1
	pGBL->GBL_SYS_CLK_CTL |= USB_CLK_SEL_EXT0;
	pGBL->GBL_CLK_PATH_CTL = (BYPASS_DPLL0 | BYPASS_DPLL1 | BYPASS_DPLL2 | USB_CLK_SEL_EXT1);
	
	MMPF_PLL_WaitCount(usWaitCycle);
	
	//Step 2: Turn off some mudules' clock for de-glitch.
	//pGBL->GBL_CLK_DIS0 = (GBL_CLK_SCAL_DIS | GBL_CLK_JPG_DIS);  	//To enable all CLK
	
	//Step 3: Power-down all DPLL
	for(i = 0x0; i < (MMPF_PLL_ID_MAX - 1); i++) {
		MMPF_PLL_PowerUp((MMPF_PLL_ID)(MMPF_PLL_ID_0 + i), MMP_FALSE);
	}
	
	//Step 4: Adjust each PLL targe frequency and power-up
	if(m_pll_freq[0] != MMPF_PLL_FREQ_EXT_CLK) {
		pGBL->GBL_DPLL0_N = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[0]][1];
		pGBL->GBL_DPLL0_K = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[0]][2]; 
		//Update DPLL0 configuration
		pGBL->GBL_DPLL_CFG3 |= DPLL0_UPDATE_EN;
		m_pll_out_clk[0] = ((EXT_PMCLK_CKL)*pGBL->GBL_DPLL0_N)/(pGBL->GBL_DPLL0_K + 1);
		//RTNA_DBG_PrintLong(0, m_pll_out_clk[0]);
		MMPF_PLL_PowerUp((MMPF_PLL_ID)(MMPF_PLL_ID_0), MMP_TRUE); 	
	}
	
	if(m_pll_freq[1] != MMPF_PLL_FREQ_EXT_CLK) {
		MMP_UBYTE ubCfgDivid = 0x8;
		pGBL->GBL_DPLL1_M = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[1]][0];
		pGBL->GBL_DPLL1_N = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[1]][1];
		pGBL->GBL_DPLL1_CFG01 = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[1]][2];
		ubCfgDivid = (ubCfgDivid >> pGBL->GBL_DPLL1_CFG01);
		
		if(PLL_300_700_500MHZ == MMPF_PLL_TABLE_VSN_V2[m_pll_freq[1]][3]) {
			MMP_ULONG ulPllOutClk = (((EXT_PMCLK_CKL/1000)*(pGBL->GBL_DPLL1_N+2))/pGBL->GBL_DPLL1_M);
			pGBL->GBL_DPLL1_CFG01 |= PLL_300_700_500MHZ;
			
			//Adjust Analog gain settings
			if(ulPllOutClk < 350) {
				pGBL->GBL_DPLL1_CFG2 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
				pGBL->GBL_DPLL1_CFG01 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			}
			else if(ulPllOutClk < 400) {
				pGBL->GBL_DPLL1_CFG2 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
				pGBL->GBL_DPLL1_CFG01 |= PLL_V2I_HIGH_GAIN_EN; //bit 7 set 1
			}
			else if(ulPllOutClk < 500) {
				pGBL->GBL_DPLL1_CFG2 &= (~PLL_V2I_GAIN_ADJUST_DIS); //bit 16 set 0
				pGBL->GBL_DPLL1_CFG01 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			}
			else {
				pGBL->GBL_DPLL1_CFG2 &= (~PLL_V2I_GAIN_ADJUST_DIS); //bit 16 set 0
				pGBL->GBL_DPLL1_CFG01 |= PLL_V2I_HIGH_GAIN_EN; //bit 7 set 1
			}
			
		}
		else {
			pGBL->GBL_DPLL1_CFG2 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
			pGBL->GBL_DPLL1_CFG01 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			//RTNA_DBG_Str(0, "Warning: Bad PLL settings !\r\n");
		}
		pGBL->GBL_DPLL_CFG3 |= DPLL1_UPDATE_EN;
		
		/*RTNA_DBG_PrintLong(0, pGBL->GBL_DPLL1_M);
		RTNA_DBG_PrintLong(0, (pGBL->GBL_DPLL1_N+2));
		RTNA_DBG_PrintLong(0, ubCfgDivid);*/
		m_pll_out_clk[1] = (((EXT_PMCLK_CKL)/pGBL->GBL_DPLL1_M)*(pGBL->GBL_DPLL1_N+2))/ubCfgDivid;
		//RTNA_DBG_PrintLong(0, m_pll_out_clk[1]); 
		MMPF_PLL_PowerUp((MMPF_PLL_ID)(MMPF_PLL_ID_1), MMP_TRUE);  	
	}
	
	if(m_pll_freq[2] != MMPF_PLL_FREQ_EXT_CLK) {
		MMP_UBYTE ubCfgDivid = 0x8;
		pGBL->GBL_DPLL2_M = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[2]][0];
		pGBL->GBL_DPLL2_N = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[2]][1];
		pGBL->GBL_DPLL2_CFG2 = MMPF_PLL_TABLE_VSN_V2[m_pll_freq[2]][2];
		ubCfgDivid = (ubCfgDivid >> pGBL->GBL_DPLL2_CFG2);
		
		if(PLL_300_700_500MHZ == MMPF_PLL_TABLE_VSN_V2[m_pll_freq[2]][3]) {
			MMP_ULONG ulPllOutClk = (((EXT_PMCLK_CKL/1000)*(pGBL->GBL_DPLL2_N+2))/pGBL->GBL_DPLL2_M);
			pGBL->GBL_DPLL2_CFG2 |= PLL_300_700_500MHZ;
			
			//Adjust Analog gain settings
			if(ulPllOutClk < 350) {
				pGBL->GBL_DPLL2_CFG4 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
				pGBL->GBL_DPLL2_CFG2 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			}
			else if(ulPllOutClk < 400) {
				pGBL->GBL_DPLL2_CFG4 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
				pGBL->GBL_DPLL2_CFG2 |= PLL_V2I_HIGH_GAIN_EN; //bit 7 set 1
			}
			else if(ulPllOutClk < 500) {
				pGBL->GBL_DPLL2_CFG4 &= (~PLL_V2I_GAIN_ADJUST_DIS); //bit 16 set 0
				pGBL->GBL_DPLL2_CFG2 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			}
			else {
				pGBL->GBL_DPLL2_CFG4 &= (~PLL_V2I_GAIN_ADJUST_DIS); //bit 16 set 0
				pGBL->GBL_DPLL2_CFG2 |= PLL_V2I_HIGH_GAIN_EN; //bit 7 set 1
			}
			
		}
		else {
			pGBL->GBL_DPLL2_CFG4 |= PLL_V2I_GAIN_ADJUST_DIS; //bit 16 set 1
			pGBL->GBL_DPLL2_CFG2 &= (~PLL_V2I_HIGH_GAIN_EN); //bit 7 set 0
			//RTNA_DBG_Str(0, "Warning: Bad PLL settings !\r\n");
		}
		pGBL->GBL_DPLL_CFG3 |= DPLL2_UPDATE_EN;
		
		/*RTNA_DBG_PrintLong(0, pGBL->GBL_DPLL2_M);
		RTNA_DBG_PrintLong(0, (pGBL->GBL_DPLL2_N+2));
		RTNA_DBG_PrintLong(0, ubCfgDivid);*/
		m_pll_out_clk[2] = (((EXT_PMCLK_CKL)/pGBL->GBL_DPLL2_M)*(pGBL->GBL_DPLL2_N+2))/ubCfgDivid;
		//RTNA_DBG_PrintLong(0, m_pll_out_clk[2]); 
		MMPF_PLL_PowerUp((MMPF_PLL_ID)(MMPF_PLL_ID_2), MMP_TRUE);  	
	}
	
	
	MMPF_PLL_WaitCount(20000); //wait H/W switch time 5ms (WaitCount(12000) = 5ms) (WaitCount(20000) =9.4ms) 
	
	//Step 5: Setup each group's mux and diviver
	//Step 5-1: Group 0 (GBL_CLK)
	//pGBL->GBL_CLK_DIV = (pGBL->GBL_CLK_DIV & PLL_CLK_MUX); //Clean previous diver settings
	m_pll_src_temp = (m_group_src[0] - MMPF_GUP_SRC_PLL_0);
	if(glGroupFreq[0] != EXT_PMCLK_CKL) {	
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			MMP_UBYTE ubPostDiv = 0x0;
			
			if(pGBL->GBL_CLK_SEL & GBL_CLK_SEL_MUX0) { //Use 0x6905 bit 6~7 as source selection
				pGBL->GBL_CLK_DIV = (m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN; //MUX PLL
				pGBL->GBL_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			}
			else {
				pGBL->GBL_CLK_SEL = (m_pll_src_temp << PLL_SEL_PLL_OFFSET1);
				pGBL->GBL_CLK_DIV |= PLL_POST_DIV_EN;
				pGBL->GBL_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			}
			if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[0]) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group0\r\n");
				return 1;
			}
			ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glGroupFreq[0]);
			
			if((ubPostDiv & 0x1) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group0\r\n");
				return 1;
			}
			if(ubPostDiv != 0x2) {
				ubPostDiv = (ubPostDiv >> 1); //GBL_CLK have postDivider (divede 2)
				//Note: We can not turn off the PLL_POST_DIV_EN bit in Group 0
				
				pGBL->GBL_CLK_DIV |= (ubPostDiv - 1);
			}
			//pGBL->GBL_CLK_DIV |= PLL_POST_DIV_EN;
		}
	}
	else {
		pGBL->GBL_CLK_DIV = PLL_CLK_DIV_MCLK;
	}
	
	
	//Step 5-2: Group 1 (DRAM_CLK)
	m_pll_src_temp = (m_group_src[1] - MMPF_GUP_SRC_PLL_0);
	if((glGroupFreq[1] != glGroupFreq[0])) {//DRAM ASYNC mode.
		if(glGroupFreq[1] != EXT_PMCLK_CKL) { 
			pGBL->GBL_DRAM_CLK_DIV = (pGBL->GBL_DRAM_CLK_DIV & PLL_CLK_MUX); //Clean previous diver settings;
			m_pll_src_temp = (m_group_src[1] - MMPF_GUP_SRC_PLL_0);
			if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
				MMP_UBYTE ubPostDiv = 0x0;
				pGBL->GBL_DRAM_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN) ; //MUX PLL
				pGBL->GBL_DRAM_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
				if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[1]) != 0x0) {
					//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group1\r\n");
					return 1;
				}
				
				ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glGroupFreq[1]);
				
				if((ubPostDiv & 0x1) != 0x0) {
					//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group1\r\n");
					return 1;
				}
				if(ubPostDiv != 0x2) {
					ubPostDiv = (ubPostDiv >> 1); //DRAM_CLK have postDivider (divede 2)
					pGBL->GBL_DRAM_CLK_DIV |= (ubPostDiv - 1);
				}
				
				pGBL->GBL_DRAM_CLK_DIV |= PLL_POST_DIV_EN;
			}
		}
		else {
			pGBL->GBL_DRAM_CLK_DIV = PLL_CLK_DIV_MCLK;
		}
		pGBL->GBL_SYS_CLK_CTL |= DRAM_ASYNC_EN;	
	}
	else { //DRAM SYNC mode
		pGBL->GBL_SYS_CLK_CTL &= (~DRAM_ASYNC_EN);
	}
	//pGBL->GBL_DRAM_CLK_DIV |= PLL_POST_DIV_EN;
	//pGBL->GBL_SYS_CLK_CTL |= (DRAM_ASYNC_EN);
	
	//Step 5-3: Group 2 (USBPHY_CLK)
	pGBL->GBL_USBPHY_CLK_DIV = (pGBL->GBL_USBPHY_CLK_DIV & PLL_CLK_MUX); //Clean previous diver settings
	m_pll_src_temp = (m_group_src[2] - MMPF_GUP_SRC_PLL_0);
	if(glGroupFreq[2] != EXT_PMCLK_CKL) {
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			MMP_UBYTE ubPostDiv = 0x0;
			pGBL->GBL_USBPHY_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET)|PLL_POST_DIV_EN) ; //MUX PLL
			pGBL->GBL_USBPHY_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[2]) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group2\r\n");
				return 1;
			}
				
			ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glGroupFreq[2]);
			pGBL->GBL_USBPHY_CLK_DIV = (ubPostDiv - 1);
			
			pGBL->GBL_USBPHY_CLK_DIV |= PLL_POST_DIV_EN;
		}
	}
	else {
		pGBL->GBL_USBPHY_CLK_DIV = PLL_CLK_DIV_MCLK;
	}
	
	//Step 5-4: Group 3 (RX_BIST_CLK)
	if(glGroupFreq[3] != EXT_PMCLK_CKL) {
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			pGBL->GBL_MIPI_RX_CLK_SEL = (m_group_src[3] - MMPF_GUP_SRC_PLL_0); //MUX PLL
		}
	}
	else {
		pGBL->GBL_MIPI_RX_CLK_SEL = (PLL_CLK_DIV_MCLK >> PLL_SEL_PLL_OFFSET);
	}
	
	//Step 5-5: Group 4 (SENSOR_CLK)
	pGBL->GBL_VI_CLK_DIV = (pGBL->GBL_VI_CLK_DIV & PLL_CLK_MUX);
	m_pll_src_temp = (m_group_src[4] - MMPF_GUP_SRC_PLL_0);
	if(glGroupFreq[4] != EXT_PMCLK_CKL) {
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			MMP_UBYTE ubPostDiv = 0x0;
			pGBL->GBL_VI_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN); //MUX PLL
			pGBL->GBL_VI_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[4]) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group4\r\n");
				return 1;
			}
				
			ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glGroupFreq[4]);
			pGBL->GBL_VI_CLK_DIV |= (ubPostDiv - 1);
		}
		pGBL->GBL_VI_CLK_DIV |= PLL_POST_DIV_EN;
	}
	else {
		pGBL->GBL_VI_CLK_DIV = PLL_CLK_DIV_MCLK;
	}
	
	//Step 5-6: Group 5 (AUDIO_CLK)
//	pGBL->GBL_AUDIO_CLK_DIV = (pGBL->GBL_AUDIO_CLK_DIV & PLL_CLK_MUX);
	m_pll_src_temp = (m_group_src[5] - MMPF_GUP_SRC_PLL_0);
	if(glGroupFreq[5] != EXT_PMCLK_CKL) {
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			MMP_UBYTE ubPostDiv = 0x0;
//			pGBL->GBL_AUDIO_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN) ; //MUX PLL
//			pGBL->GBL_AUDIO_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[5]) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group5\r\n");
				return 1;
			}
				
			ubPostDiv = 4;//(m_pll_out_clk[m_pll_src_temp]/glGroupFreq[5]);
//			pGBL->GBL_AUDIO_CLK_DIV |= (ubPostDiv - 1);
		}
//		pGBL->GBL_AUDIO_CLK_DIV |= PLL_POST_DIV_EN;
	}
	else {
//		pGBL->GBL_AUDIO_CLK_DIV = PLL_CLK_DIV_MCLK;
	}
	
	
	//Step 5-7: Group 6 (COLOR_CLK(ISP))
	pGBL->GBL_COLOR_CLK_DIV = (pGBL->GBL_COLOR_CLK_DIV & PLL_CLK_MUX);
	m_pll_src_temp = (m_group_src[6] - MMPF_GUP_SRC_PLL_0);
	if(glGroupFreq[6] != EXT_PMCLK_CKL) {
		if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
			MMP_UBYTE ubPostDiv = 0x0;
			pGBL->GBL_COLOR_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN) ; //MUX PLL
			pGBL->GBL_COLOR_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
			if((m_pll_out_clk[m_pll_src_temp]%glGroupFreq[6]) != 0x0) {
				//RTNA_DBG_Str(0, "!!!!    Error PLL settings @group6\r\n");
				return 1;
			}
				
			ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glGroupFreq[6]);
			pGBL->GBL_COLOR_CLK_DIV |= (ubPostDiv - 1);
		}
		pGBL->GBL_COLOR_CLK_DIV |= PLL_POST_DIV_EN;
	}
	else {
		pGBL->GBL_COLOR_CLK_DIV = PLL_CLK_DIV_MCLK;
	}
	
	//Step 5-8: CPU async
	//pGBL->GBL_CPU_CLK_DIV = (pGBL->GBL_CPU_CLK_DIV & PLL_CLK_MUX);
	m_pll_src_temp = (m_cpu_src - MMPF_GUP_SRC_PLL_0);
	if(glCPUFreq != glGroupFreq[0]) {
		if(glCPUFreq != EXT_PMCLK_CKL) {
			if(m_pll_src_temp < MAX_DPLL_SRC) { //Use one of available PLL
				MMP_UBYTE ubPostDiv = 0x0;
				
				if(pGBL->GBL_CPU_CLK_SEL & CPU_CLK_SEL_MUX0) {
					pGBL->GBL_CPU_CLK_DIV = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET) | PLL_POST_DIV_EN); //MUX PLL
					pGBL->GBL_CPU_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
				}
				else {
					pGBL->GBL_CPU_CLK_SEL = ((m_pll_src_temp << PLL_SEL_PLL_OFFSET1) | PLL_POST_DIV_EN);
					pGBL->GBL_CPU_CLK_DIV |= PLL_POST_DIV_EN;
					pGBL->GBL_CPU_CLK_DIV &= (~PLL_DIVIDE_MASK); //Clean divider factor
				}
				if((m_pll_out_clk[m_pll_src_temp]%glCPUFreq) != 0x0) {
					//RTNA_DBG_Str(0, "!!!!    Error PLL settings @CPU\r\n");
					return 1;
				}
				ubPostDiv = (m_pll_out_clk[m_pll_src_temp]/glCPUFreq);
				//Note: We can not turn off the PLL_POST_DIV_EN bit in CPU clk path
				pGBL->GBL_CPU_CLK_DIV |= (ubPostDiv - 1);
			}
			//pGBL->GBL_CPU_CLK_DIV |= PLL_POST_DIV_EN;
		}	
		else {
			pGBL->GBL_CPU_CLK_DIV = PLL_CLK_DIV_MCLK;
		}
		pGBL->GBL_CPU_CFG |= GBL_CPU_ASYNC_EN;
	}
	
	MMPF_PLL_WaitCount(usWaitCycle);
	
	//Step 6: Switch to defined PLL
	pGBL->GBL_SYS_CLK_CTL &= (~USB_CLK_SEL_EXT0);
	pGBL->GBL_CLK_PATH_CTL &= (~(BYPASS_DPLL0 | BYPASS_DPLL1 | BYPASS_DPLL2 | USB_CLK_SEL_EXT1));
	
	m_pll_mode = target_pll_mode;
//	RTNA_DBG_Open(((glGroupFreq[0]/1000) >> 1), 115200);
    MMPF_PLL_WaitCount(5000);
//	RTNA_DBG_PrintLong(0, m_pll_out_clk[0]);
//	RTNA_DBG_PrintLong(0, m_pll_out_clk[1]);
//	RTNA_DBG_PrintLong(0, m_pll_out_clk[2]);
//    RTNA_DBG_PrintLong(0, glCPUFreq);
	return MMP_ERR_NONE;
}
#endif //(OS_TYPE == OS_UCOSII)

#if (CHIP == MERCURY)
MMP_ERR MMPF_PLL_Setting(void)
{
    #if defined(MBOOT_FW)
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ULONG clock;
    MMP_UBYTE ubTempDpll2K;// for 0x80005D1B workaround
    #endif

    glCPUAFreq = m_PllSettings[m_ClockSrc.cpua].freq /          //CPUA
                    m_ClockSrc.cpua_div;
    glCPUBFreq = m_PllSettings[m_ClockSrc.cpub].freq /          //CPUB
                    m_ClockSrc.cpub_div;
    glGroupFreq[0] = m_PllSettings[m_ClockSrc.global].freq /  //Global
                    m_ClockSrc.global_div;
    glGroupFreq[1] = m_PllSettings[m_ClockSrc.dram].freq /    //DRAM
                    m_ClockSrc.dram_div;

    glGroupFreq[2] = m_PllSettings[m_ClockSrc.usb_phy].freq / //USB PHY
                    m_ClockSrc.usb_phy_div;
    glGroupFreq[3] = m_PllSettings[m_ClockSrc.rx_bist].freq / //RX BIST
                    m_ClockSrc.rx_bist_div;
    glGroupFreq[4] = m_PllSettings[m_ClockSrc.sensor].freq /  //Sensor
                    m_ClockSrc.sensor_div;
    glGroupFreq[5] = m_PllSettings[m_ClockSrc.audio].freq /   //Audio
                    m_ClockSrc.audio_div;
    glGroupFreq[6] = m_PllSettings[m_ClockSrc.isp].freq /     //ISP
                    m_ClockSrc.isp_div;
    glGroupFreq[7] = m_PllSettings[m_ClockSrc.bayer].freq /   //BAYER
                    m_ClockSrc.bayer_div;
    glGroupFreq[8] = m_PllSettings[m_ClockSrc.mipitx].freq /  //MIPI TX
                    m_ClockSrc.mipitx_div;


    #if defined(MBOOT_FW)

    //Step 1: Switch USB to external clock (PMCLK), Bypass PLL0, PLL1 & PLL2
    pGBL->GBL_SYS_CLK_CTL |= USB_CLK_SEL_EXT0;
    pGBL->GBL_CLK_PATH_CTL = USB_CLK_SEL_EXT1 | BYPASS_DPLL0 |
                             BYPASS_DPLL1 | BYPASS_DPLL2 |
                             BYPASS_DPLL3 | BYPASS_DPLL4;
    MMPF_PLL_WaitCount(50);

    //Step 2: Update PLL to the target frequency
    if (m_PllSettings[MMPF_PLL_ID_0].freq != EXT_PMCLK_CKL) {

        if (PLL_200_500_350MHZ == m_PllSettings[MMPF_PLL_ID_0].VCO) {
            //Adjust Analog gain settings
            //below clcok region is (24 / M * (n + 1)). It doesn't include post divider
            switch(m_PllSettings[MMPF_PLL_ID_0].M) {
                case PLL0_M_DIV_1:
                    clock = 24 * (m_PllSettings[MMPF_PLL_ID_0].N + 1);
                    break;
                case PLL0_M_DIV_2:
                    clock = 12 * (m_PllSettings[MMPF_PLL_ID_0].N + 1);
                    break;
                case PLL0_M_DIV_3:
                    clock = 8 * (m_PllSettings[MMPF_PLL_ID_0].N + 1);
                    break;
                case PLL0_M_DIV_4:
                    clock = 6 * (m_PllSettings[MMPF_PLL_ID_0].N + 1);
                    break;
                case PLL0_M_DIV_6:
                    clock = 4 * (m_PllSettings[MMPF_PLL_ID_0].N + 1);
                    break;
            }

            if ((clock >= 350)&&(clock < 400)) {
                pGBL->GBL_DPLL0_CFG7_10 &=~ 0xFFFF;
                pGBL->GBL_DPLL0_CFG7_10 |=  0x2020;//Bias current
            }
            else if (clock < 500) {
                pGBL->GBL_DPLL0_CFG7_10 &=~ 0xFFFF;
                pGBL->GBL_DPLL0_CFG7_10 |=  0x2022;
            }
            else if (clock < 600) {
                pGBL->GBL_DPLL0_CFG7_10 &=~ 0xFFFF;
                pGBL->GBL_DPLL0_CFG7_10 |=  0x2026;
            }
            else {// >600
                pGBL->GBL_DPLL0_CFG7_10 &=~ 0xFFFF;
                pGBL->GBL_DPLL0_CFG7_10 |=  0x2028;
            }

        } else {
            //Bad PLL setting
            while(1);
        }

        pGBL->GBL_DPLL0_M = m_PllSettings[MMPF_PLL_ID_0].M;
        pGBL->GBL_DPLL0_N = m_PllSettings[MMPF_PLL_ID_0].N;
        pGBL->GBL_DPLL0_K = m_PllSettings[MMPF_PLL_ID_0].K;
        pGBL->GBL_DPLL0_K |= m_PllSettings[MMPF_PLL_ID_0].VCO;

        pGBL->GBL_DPLL0_K |= PLL_V2I_HIGH_GAIN_EN;
        pGBL->GBL_DPLL0_CFG6 = PLL_V2I_GAIN_ADJUST_DIS;

        //Update DPLL0 configuration
        pGBL->GBL_DPLL_CFG3 |= DPLL0_UPDATE_EN;
    }

    if (m_PllSettings[MMPF_PLL_ID_1].freq != EXT_PMCLK_CKL) {

        if (PLL_200_500_350MHZ == m_PllSettings[MMPF_PLL_ID_1].VCO) {
            //Adjust Analog gain settings
            //below clcok region is (24 / M * (n + 1)). It doesn't include post divider
            switch(m_PllSettings[MMPF_PLL_ID_1].M) {
                case PLL1_M_DIV_1:
                    clock = 24 * (m_PllSettings[MMPF_PLL_ID_1].N + 1);
                    break;
                case PLL1_M_DIV_2:
                    clock = 12 * (m_PllSettings[MMPF_PLL_ID_1].N + 1);
                    break;
                case PLL1_M_DIV_3:
                    clock = 8 * (m_PllSettings[MMPF_PLL_ID_1].N + 1);
                    break;
                case PLL1_M_DIV_4:
                    clock = 6 * (m_PllSettings[MMPF_PLL_ID_1].N + 1);
                    break;
                case PLL1_M_DIV_6:
                    clock = 4 * (m_PllSettings[MMPF_PLL_ID_1].N + 1);
                    break;
            }

            if ((clock >= 350)&&(clock < 400)) {
                pGBL->GBL_DPLL1_CFG3_6 &=~ 0xFFFF;
                pGBL->GBL_DPLL1_CFG3_6 |= 0x2020;//Bias current
            }
            else if (clock < 500) {
                pGBL->GBL_DPLL1_CFG3_6 &=~ 0xFFFF;
                pGBL->GBL_DPLL1_CFG3_6 |= 0x2022;
            }
            else if (clock < 600) {
                pGBL->GBL_DPLL1_CFG3_6 &=~ 0xFFFF;
                pGBL->GBL_DPLL1_CFG3_6 |= 0x2026;
            }
            else {// >600
                pGBL->GBL_DPLL1_CFG3_6 &=~ 0xFFFF;
                pGBL->GBL_DPLL1_CFG3_6 |= 0x2028;
            }
        } else {
            //Bad PLL setting
            while(1);
        }

        pGBL->GBL_DPLL1_M = m_PllSettings[MMPF_PLL_ID_1].M;
        pGBL->GBL_DPLL1_N = m_PllSettings[MMPF_PLL_ID_1].N;
        pGBL->GBL_DPLL1_K = m_PllSettings[MMPF_PLL_ID_1].K;
        pGBL->GBL_DPLL1_K |= m_PllSettings[MMPF_PLL_ID_1].VCO;

        pGBL->GBL_DPLL1_K |= PLL_V2I_HIGH_GAIN_EN;
        pGBL->GBL_DPLL1_CFG2 = PLL_V2I_GAIN_ADJUST_DIS;

        //Update DPLL1 configuration
        pGBL->GBL_DPLL_CFG3 |= DPLL1_UPDATE_EN;
    }

    if (m_PllSettings[MMPF_PLL_ID_3].freq != EXT_PMCLK_CKL) {
        pGBL->GBL_DPLL3_M = m_PllSettings[MMPF_PLL_ID_3].M;
        //------------------Workaround-----------------------
        //Because of bug, exchange PLL2 and PLL3 initial order
        //pGBL->GBL_SYS_CLK_CTL &=~ 1;
        ubTempDpll2K = pGBL->GBL_DPLL2_K;
        *(volatile MMP_ULONG *)0x80005D10 = (m_PllSettings[MMPF_PLL_ID_3].N << 24) | (*(volatile MMP_ULONG *)0x80005D10 & 0xFFFFFF);
        pGBL->GBL_DPLL2_K = ubTempDpll2K;
        //pGBL->GBL_DPLL_CFG3 |= DPLL2_UPDATE_EN;
        //pGBL->GBL_SYS_CLK_CTL |= 1;
        //------------------Workaround-----------------------

        pGBL->GBL_DPLL3_FRAC_LSB = (MMP_USHORT)m_PllSettings[MMPF_PLL_ID_3].frac;
        pGBL->GBL_DPLL3_FRAC_MSB = (MMP_UBYTE)(m_PllSettings[MMPF_PLL_ID_3].frac >> 16);
        pGBL->GBL_DPLL3_PWR &=~ PLL3_CLK_SRC_MASK;
        pGBL->GBL_DPLL3_PWR |= PLL3_CLK_SRC_MCLK;
        pGBL->GBL_DPLL3_CFG3 = 0;
        pGBL->GBL_DPLL3_CFG4 = 0;
        pGBL->GBL_DPLL3_CFG6_9 &=~ 0xFF;

        //Update DPLL2 configuration
        pGBL->GBL_DPLL_CFG3 |= DPLL3_UPDATE_EN;
    }

    if (m_PllSettings[MMPF_PLL_ID_2].freq != EXT_PMCLK_CKL) {

        if (PLL_200_500_350MHZ == m_PllSettings[MMPF_PLL_ID_2].VCO) {
            //Adjust Analog gain settings
            //below clcok region is (24 / M * (n + 1)). It doesn't include post divider
            switch(m_PllSettings[MMPF_PLL_ID_2].M) {
                case PLL2_M_DIV_1:
                    clock = 24 * (m_PllSettings[MMPF_PLL_ID_2].N + 1);
                    break;
                case PLL2_M_DIV_2:
                    clock = 12 * (m_PllSettings[MMPF_PLL_ID_2].N + 1);
                    break;
                case PLL2_M_DIV_3:
                    clock = 8 * (m_PllSettings[MMPF_PLL_ID_2].N + 1);
                    break;
                case PLL2_M_DIV_4:
                    clock = 6 * (m_PllSettings[MMPF_PLL_ID_2].N + 1);
                    break;
                case PLL2_M_DIV_6:
                    clock = 4 * (m_PllSettings[MMPF_PLL_ID_2].N + 1);
                    break;
            }

            if ((clock >= 350)&&(clock < 400)) {
                pGBL->GBL_DPLL2_CFG6_9 &=~ 0xFFFF;
                pGBL->GBL_DPLL2_CFG6_9 |= 0x2020;//Bias current
            }
            else if (clock < 500) {
                pGBL->GBL_DPLL2_CFG6_9 &=~ 0xFFFF;
                pGBL->GBL_DPLL2_CFG6_9 |= 0x2022;
            }
            else if (clock < 600) {
                pGBL->GBL_DPLL2_CFG6_9 &=~ 0xFFFF;
                pGBL->GBL_DPLL2_CFG6_9 |= 0x2026;
            }
            else {// >600
                pGBL->GBL_DPLL2_CFG6_9 &=~ 0xFFFF;
                pGBL->GBL_DPLL2_CFG6_9 |= 0x2028;
            }
        } else {
            //Bad PLL setting
            while(1);
        }

        pGBL->GBL_DPLL2_M = m_PllSettings[MMPF_PLL_ID_2].M;
        //Because of bug, if write 0x5d11, 0x5d13 also writes to 0x5d1b. Needs to careful
        pGBL->GBL_DPLL2_N = m_PllSettings[MMPF_PLL_ID_2].N;
        pGBL->GBL_DPLL2_K = m_PllSettings[MMPF_PLL_ID_2].K;
        pGBL->GBL_DPLL2_K |= m_PllSettings[MMPF_PLL_ID_2].VCO;

        pGBL->GBL_DPLL2_K |= PLL_V2I_HIGH_GAIN_EN;
        pGBL->GBL_DPLL2_CFG5 = PLL_V2I_GAIN_ADJUST_DIS;

        //Update DPLL2 configuration
        pGBL->GBL_DPLL_CFG3 |= DPLL2_UPDATE_EN;
    }

    if (m_PllSettings[MMPF_PLL_ID_4].freq != EXT_PMCLK_CKL) {
        pGBL->GBL_DPLL4_M = m_PllSettings[MMPF_PLL_ID_4].M;
        pGBL->GBL_DPLL4_N = m_PllSettings[MMPF_PLL_ID_4].N;
        pGBL->GBL_DPLL4_K = m_PllSettings[MMPF_PLL_ID_4].K | 0x10;

        pGBL->GBL_DPLL4_PWR = 0;

        pGBL->GBL_DPLL4_CFG6_7 = 0x1D20;

        //Update DPLL2 configuration
        pGBL->GBL_DPLL_CFG3 |= DPLL4_UPDATE_EN;
    }

    //wait PLL power up time, min 5ns~10ns needed
    MMPF_PLL_WaitCount(20000);

    //Step 4: Setup each group's mux and diviver
    MMPF_PLL_ConfigGroupClkSrc();

    //Step 5: Switch clock by using the target PLLs
    pGBL->GBL_SYS_CLK_CTL &= ~(USB_CLK_SEL_EXT0);
    pGBL->GBL_CLK_PATH_CTL &= ~(USB_CLK_SEL_EXT1 | BYPASS_DPLL0 |
                                BYPASS_DPLL1 | BYPASS_DPLL2 | BYPASS_DPLL3 | BYPASS_DPLL4);

    RTNA_DBG_Open(((glGroupFreq[0]/1000) >> 1), 115200);
    MMPF_PLL_WaitCount(5000);

    #endif

    RTNA_DBG_PrintLong(0, m_PllSettings[0].freq);
    RTNA_DBG_PrintLong(0, m_PllSettings[1].freq);
    RTNA_DBG_PrintLong(0, m_PllSettings[2].freq);
    RTNA_DBG_PrintLong(0, m_PllSettings[3].freq);
    RTNA_DBG_PrintLong(0, m_PllSettings[4].freq);
    RTNA_DBG_PrintLong(0, glCPUAFreq);
    RTNA_DBG_PrintLong(0, glCPUBFreq);

    return MMP_ERR_NONE;
}
#endif // (CHIP == MERCURY)

#if (CHIP == MCR_V2)
MMP_ERR MMPF_PLL_Setting(void)
{
    AITPS_GBL pGBL = AITC_BASE_GBL;

    //MMP_UBYTE ubTempDpll2K;// for 0x80005D1B workaround

    glCPUAFreq = m_PllSettings[m_ClockSrc.cpua].freq /          //CPUA
                    m_ClockSrc.cpua_div;
    glCPUBFreq = m_PllSettings[m_ClockSrc.cpub].freq /          //CPUB
                    m_ClockSrc.cpub_div;
    glGroupFreq[0] = m_PllSettings[m_ClockSrc.global].freq /  //Global
                    m_ClockSrc.global_div;
    glGroupFreq[1] = m_PllSettings[m_ClockSrc.dram].freq /    //DRAM
                    m_ClockSrc.dram_div;
    glGroupFreq[2] = m_PllSettings[m_ClockSrc.usb_phy].freq / //USB PHY
                    m_ClockSrc.usb_phy_div;
    glGroupFreq[3] = m_PllSettings[m_ClockSrc.rx_bist].freq / //RX BIST
                    m_ClockSrc.rx_bist_div;
    glGroupFreq[4] = m_PllSettings[m_ClockSrc.sensor].freq /  //Sensor
                    m_ClockSrc.sensor_div;
    glGroupFreq[5] = m_PllSettings[m_ClockSrc.audio].freq /   //Audio
                    m_ClockSrc.audio_div;
    glGroupFreq[6] = m_PllSettings[m_ClockSrc.isp].freq /     //ISP
                    m_ClockSrc.isp_div;
    glGroupFreq[7] = m_PllSettings[m_ClockSrc.bayer].freq /   //BAYER
                    m_ClockSrc.bayer_div;
    glGroupFreq[8] = m_PllSettings[m_ClockSrc.mipitx].freq /  //MIPI TX
                    m_ClockSrc.mipitx_div;

    #if defined(ALL_FW)
    RTNA_DBG_Open((glGroupFreq[0] >> 1), 115200);
    return MMP_ERR_NONE;
    #endif
    //Step 1: Bypass PLL0, PLL1, PLL2, PLL3, PLL4 & PLL5
    pGBL->GBL_DPLL0_CFG |= DPLL_BYPASS;
    pGBL->GBL_DPLL1_CFG |= DPLL_BYPASS;
    pGBL->GBL_DPLL2_CFG |= DPLL_BYPASS;
    pGBL->GBL_DPLL3_CFG |= DPLL_BYPASS;
    pGBL->GBL_DPLL4_CFG |= DPLL_BYPASS;
    pGBL->GBL_DPLL5_CFG |= DPLL_BYPASS;

    MMPF_PLL_WaitCount(50);

    //Step 2: Update PLL to the target frequency
    if (m_PllSettings[MMPF_PLL_ID_0].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 / M * (N + 1)/P). It doesn't include post divider

        // pre-divider
        pGBL->GBL_DPLL0_M = m_PllSettings[MMPF_PLL_ID_0].M; // 0x5d03
        // loop divider
        pGBL->GBL_DPLL0_N = m_PllSettings[MMPF_PLL_ID_0].N; // 0x5d04
        pGBL->GBL_DPPL0_PARAM[0x00] = 0x00; // 0x5d05
        pGBL->GBL_DPPL0_PARAM[0x01] = 0x00; // 0x5d06
        // post divider
        pGBL->GBL_DPPL0_PARAM[0x02] = m_PllSettings[MMPF_PLL_ID_0].K;  // 0x5d07
        // VCO Range, RSEL
        pGBL->GBL_DPPL0_PARAM[0x03] = m_PllSettings[MMPF_PLL_ID_0].VCO;// 0x5d08
        pGBL->GBL_DPPL0_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d08
        pGBL->GBL_DPPL0_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d08
        // V2I no offset
        pGBL->GBL_DPPL0_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d09
        // maximum KVO
        pGBL->GBL_DPPL0_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d0a
        // charge pump
        pGBL->GBL_DPPL0_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d0b
        pGBL->GBL_DPPL0_PARAM[0x07] = 0x00; // 0x5d0c
        //Update DPLL0 configuration
        pGBL->GBL_DPLL0_CFG |= DPLL_UPDATE_PARAM; // 0x5d00
        MMPF_PLL_PowerUp(MMPF_PLL_ID_0, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_0, 1);
    }

    if (m_PllSettings[MMPF_PLL_ID_1].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 / M * (N + 1)/P). It doesn't include post divider

        // Pre-Divider
        pGBL->GBL_DPLL1_M = m_PllSettings[MMPF_PLL_ID_1].M; // 0x5d13
        // Loop-Divider
        pGBL->GBL_DPLL1_N = m_PllSettings[MMPF_PLL_ID_1].N; // 0x5d14
        pGBL->GBL_DPPL1_PARAM[0x00] = 0x00; // 0x5d15
        pGBL->GBL_DPPL1_PARAM[0x01] = 0x00; // 0x5d16
        // Post-Divider
        pGBL->GBL_DPPL1_PARAM[0x02] = m_PllSettings[MMPF_PLL_ID_1].K;  // 0x5d17
        // VCO Range, RSEL
        pGBL->GBL_DPPL1_PARAM[0x03] = m_PllSettings[MMPF_PLL_ID_1].VCO;// 0x5d18
        pGBL->GBL_DPPL1_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d18
        pGBL->GBL_DPPL1_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d18
        // V2I no offset
        pGBL->GBL_DPPL1_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d19
        // maximum KVO
        pGBL->GBL_DPPL1_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d1a
        // charge pump
        pGBL->GBL_DPPL1_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d1b
        pGBL->GBL_DPPL1_PARAM[0x07] = 0x00; //0x5d1c
        //Update DPLL1 configuration
        pGBL->GBL_DPLL1_CFG |= DPLL_UPDATE_PARAM; // 0x5d10
        MMPF_PLL_PowerUp(MMPF_PLL_ID_1, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_1, 1);
    }

    if (m_PllSettings[MMPF_PLL_ID_2].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 / M * (n + 1)). It doesn't include post divider

        // Pre-Divider
        pGBL->GBL_DPLL2_M = m_PllSettings[MMPF_PLL_ID_2].M; // 0x5d23
        // Loop-Divider
        pGBL->GBL_DPLL2_N = m_PllSettings[MMPF_PLL_ID_2].N; // 0x5d24
        pGBL->GBL_DPPL2_PARAM[0x00] = 0x00; // 0x5d25
        pGBL->GBL_DPPL2_PARAM[0x01] = 0x00; // 0x5d26
        // Post-Divider
        pGBL->GBL_DPPL2_PARAM[0x02] = m_PllSettings[MMPF_PLL_ID_2].K;  // 0x5d27
        // VCO Range, RSEL
        pGBL->GBL_DPPL2_PARAM[0x03] = m_PllSettings[MMPF_PLL_ID_2].VCO;// 0x5d28
        pGBL->GBL_DPPL2_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d28
        pGBL->GBL_DPPL2_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d28
        // V2I no offset
        pGBL->GBL_DPPL2_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d29
        // maximum KVO
        pGBL->GBL_DPPL2_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d2a
        // charge pump
        pGBL->GBL_DPPL2_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d2b
        pGBL->GBL_DPPL2_PARAM[0x07] = 0x00; // 5d2c
        //Update DPLL2 configuration
        pGBL->GBL_DPLL2_CFG |= DPLL_UPDATE_PARAM; // 0x5d20
        MMPF_PLL_PowerUp(MMPF_PLL_ID_2, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_2, 1);
    }

    if (m_PllSettings[MMPF_PLL_ID_3].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 * (1/(M+1)*(N+1).Frac)). It doesn't include post divider

        // Post-Divider
        pGBL->GBL_DPLL3_M = m_PllSettings[MMPF_PLL_ID_3].M; // 0x5d33
        // Loop-Divider
        pGBL->GBL_DPLL3_N = m_PllSettings[MMPF_PLL_ID_3].N; // 0x5d34
        // Fractional Part
        pGBL->GBL_DPPL3_PARAM[0x00] = 0x85; // 0x5d35
        pGBL->GBL_DPPL3_PARAM[0x01] = 0xEB; // 0x5d36
        pGBL->GBL_DPPL3_PARAM[0x02] = 0x01; // 0x5d37
        // PLL Control Setting
        pGBL->GBL_DPPL3_PARAM[0x03] = 0x00; // 0x5d38
        // Enable BOOST CP & MAIN CP DIODE MODE
        pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 0x5d39
        MMPF_PLL_WaitCount(50);
        // Enable BOOST CP & MAIN CP OP MODE
        pGBL->GBL_DPPL3_PARAM[0x04] &= ~DPLL3_MAIN_CP_VP_DIODE; // 0x5d39
        MMPF_PLL_WaitCount(50);
        //Update DPLL3 configuration
        pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 1);
    }

    if (m_PllSettings[MMPF_PLL_ID_4].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 * (1/(M+1)*((N+1)*4)/P). It doesn't include post divider

        // Pre-Divider
        pGBL->GBL_DPLL4_M = m_PllSettings[MMPF_PLL_ID_4].M; // 0x5d43
        // Loop-Divider
        pGBL->GBL_DPLL4_N = m_PllSettings[MMPF_PLL_ID_4].N; // 0x5d44
        // SS frequency divider
        pGBL->GBL_DPPL4_PARAM[0x03] = 0x62; // 0x5d48
        // VCO frequency, CP current
        pGBL->GBL_DPPL4_PARAM[0x04] = DPLL4_800MHZ | DPLL4_CP_5uA; // 0x5d49
        pGBL->GBL_DPPL4_PARAM[0x00] = 0x00; // 0x5d45
        // SS-EN, Frange Setting
        pGBL->GBL_DPPL4_PARAM[0x01] = m_PllSettings[MMPF_PLL_ID_4].K; // 0x5d46
        // SS source select
        pGBL->GBL_DPPL4_PARAM[0x02] = DPLL4_SS_SRCSEL_0d31uA; // 0x5d47
        MMPF_PLL_WaitCount(50);
        // charge pump OP mode
        pGBL->GBL_DPPL4_PARAM[0x04] |= DPLL3_CP_OPMODE; // 0x5d49
        //Update DPLL4 configuration
        pGBL->GBL_DPLL4_CFG |= DPLL_UPDATE_PARAM; // 0x5d40
        MMPF_PLL_PowerUp(MMPF_PLL_ID_4, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_4, 1);

    }

    if (m_PllSettings[MMPF_PLL_ID_5].freq != EXT_CLK) {
        //Adjust Analog gain settings
        //below clcok region is (24 *(((N*x).Frac)/P/O)). It doesn't include post divider

        // Start-UP @ Diode Mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x00; // 0x5d5a
        // PLL Settings
        pGBL->GBL_DPPL5_PARAM[0x04] = DPLL5_DIV2_LOOPDIV_EN; // 5d59
        pGBL->GBL_DPPL5_PARAM[0x03] = DPLL5_CHARGE_PUMP_5uA; // 5d58
        pGBL->GBL_DPPL5_PARAM[0x03] |= DPLL5_KVCO_OFFSET_5uA; // 5d58
        // Post Divider Divide 2
        pGBL->GBL_DPPL5_PARAM[0x02] = m_PllSettings[MMPF_PLL_ID_5].M; // 5d57
        // Fractional Part = 0.6666
        pGBL->GBL_DPPL5_PARAM[0x02] |= 0x02; // 5d57
        pGBL->GBL_DPPL5_PARAM[0x01] = 0xAA; // 5d56
        pGBL->GBL_DPPL5_PARAM[0x00] = 0xAA; // 5d55
        // Loop Divider N part
        pGBL->GBL_DPLL5_N = m_PllSettings[MMPF_PLL_ID_5].N; // 0x5d54
        // Pre-Divider Pass
        pGBL->GBL_DPLL5_P = m_PllSettings[MMPF_PLL_ID_5].K; // 0x5d53
        MMPF_PLL_WaitCount(50);
        // change into  OP mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
        MMPF_PLL_WaitCount(50);
        //Update DPLL5 configuration
        pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
        // PLL bug, must update twice.
        // change into  OP mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
        MMPF_PLL_WaitCount(50);
        //Update DPLL5 configuration
        pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50

        MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 1);

    }

    //wait PLL power up time, min 50us needed
    MMPF_PLL_WaitCount(200);

    //Step 4: Setup each group's mux and diviver
    MMPF_PLL_ConfigGroupClkSrc();

    //Step 5: Switch clock by using the target PLLs
    pGBL->GBL_DPLL0_CFG &= ~DPLL_BYPASS;
    pGBL->GBL_DPLL1_CFG &= ~DPLL_BYPASS;
    pGBL->GBL_DPLL2_CFG &= ~DPLL_BYPASS;
    pGBL->GBL_DPLL3_CFG &= ~DPLL_BYPASS;
    pGBL->GBL_DPLL4_CFG &= ~DPLL_BYPASS;
    pGBL->GBL_DPLL5_CFG &= ~DPLL_BYPASS;

    RTNA_DBG_Open((glGroupFreq[0] >> 1), 115200);
    MMPF_PLL_WaitCount(5000);

    RTNA_DBG_PrintLong(0, glCPUAFreq);

    return MMP_ERR_NONE;
}
#endif //(CHIP == MCR_V2)

#endif //(OS_TYPE == OS_UCOSII)

#if 1

#if (CHIP==MCR_V2)
static MMP_ERR MMPF_PLL_ChangeAudioPLL(MMP_BOOL bEnable, MMPF_PLL_MODE ulSampleRate, MMPF_AUDSRC AudPath)
{
    AITPS_GBL pGBL = AITC_BASE_GBL;
	MMP_ERR   err = MMP_ERR_NONE;
	#ifdef DPLL_3A
	pGBL->GBL_AUD_CLK_SRC = AUD_CLK_SRC_DPLL3;
	#else
	pGBL->GBL_AUD_CLK_SRC = AUD_CLK_SRC_DPLL5;
	#endif
	dbg_printf(0,"change pll : %d,%d,%d\n",bEnable,ulSampleRate,AudPath);
	if(bEnable) {
	    if ((ulSampleRate != MMPF_PLL_AUDIO_44d1K) && 
			(ulSampleRate != MMPF_PLL_AUDIO_22d05K) &&
			(ulSampleRate != MMPF_PLL_AUDIO_11d025K) ) { // 147.456Mhz
			if (AudPath == MMPF_AUDSRC_MCLK) {
			#ifdef DPLL_3A
			  		// Post-Divider
			    pGBL->GBL_DPLL3_M = DPLL3_M(4); // 0x5d33
			    // Loop-Divider
			    pGBL->GBL_DPLL3_N = DPLL3_N(29); // 0x5d34
			    // Fractional Part
			    pGBL->GBL_DPPL3_PARAM[0x00] = 0x47; // 0x5d35
			    pGBL->GBL_DPPL3_PARAM[0x01] = 0xE1; // 0x5d36
			    pGBL->GBL_DPPL3_PARAM[0x02] = 0x02; // 0x5d37
			    pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 20150903@Gason, fix PLL issue.
			#else
		            //AITPS_GBL pGBL = AITC_BASE_GBL;
		            //pGBL->GBL_AUD_CLK_SRC = 0x06;//AUD_CLK_SRC_DPLL5;
		            //pGBL->GBL_DPLL5_CFG |= DPLL_BYPASS;
		            pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_1;
			    //pGBL->GBL_DPPL5_PARAM[0x02] = 1;//pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Post-Divider
			    //pGBL->GBL_DPPL5_PARAM[0x02] = pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Fractional Part
			    pGBL->GBL_DPPL5_PARAM[0x00] = 0xE9;//0x00; // 0x5d55
			    pGBL->GBL_DPPL5_PARAM[0x01] = 0x26;//0x80; // 0x5d56
			    pGBL->GBL_DPPL5_PARAM[0x02] = 0x01;//0x04; // 0x5d57
			    // Loop Divider N part
		            pGBL->GBL_DPLL5_N = 12;//pllSettings[MMPF_PLL_ID_5].N; // 0x5d54	
		            // Pre-Divider Pass
			    pGBL->GBL_DPLL5_P = DPLL5_P_DIV_1;//pllSettings[MMPF_PLL_ID_5].K; // 0x5d53
			    // change into  OP mode
                            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
                            pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
		            MMPF_PLL_WaitCount(50);
		            //pGBL->GBL_AUD_CLK_SRC = 0x06;//AUD_CLK_SRC_DPLL5;
		            //pGBL->GBL_DPLL5_CFG |= DPLL_BYPASS;
		            pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_1;
			    //pGBL->GBL_DPPL5_PARAM[0x02] = 1;//pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Post-Divider
			    //pGBL->GBL_DPPL5_PARAM[0x02] = pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Fractional Part
			    pGBL->GBL_DPPL5_PARAM[0x00] = 0xE9;//0x00; // 0x5d55
			    pGBL->GBL_DPPL5_PARAM[0x01] = 0x26;//0x80; // 0x5d56
			    pGBL->GBL_DPPL5_PARAM[0x02] = 0x01;//0x04; // 0x5d57
			// Loop Divider N part
		            pGBL->GBL_DPLL5_N = 12;//pllSettings[MMPF_PLL_ID_5].N; // 0x5d54	
		        // Pre-Divider Pass
			    pGBL->GBL_DPLL5_P = DPLL5_P_DIV_1;//pllSettings[MMPF_PLL_ID_5].K; // 0x5d53
			// change into  OP mode
		            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
		            pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
		            MMPF_PLL_WaitCount(50);
			#endif
			    
			}
			else if (AudPath == MMPF_AUDSRC_I2S) {
				// Post-Divider
			    pGBL->GBL_DPLL3_M = 0x13; // 0x5d33
			    // Loop-Divider
			    pGBL->GBL_DPLL3_N = 0x9F; // 0x5d34
			    // Fractional Part
			    pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
			    pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
			    pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37
			    pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 20150903@Gason, fix PLL issue.
			    pGBL->GBL_DPLL3_CFG &=~ DPLL_SRC_MASK;
				pGBL->GBL_DPLL3_CFG |= DPLL_SRC_PI2S_MCLK;
			}    
			#ifdef DPLL_3A
		    //Update DPLL5 configuration
/*			 pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30 
			 //MMPF_PLL_PowerUp(MMPF_PLL_3, 0);
		         MMPF_PLL_WaitCount(50);
		         MMPF_PLL_PowerUp(MMPF_PLL_3, 1);
		         MMPF_PLL_WaitCount(50);
		    pGBL->GBL_DPPL3_PARAM[4] &= ~DPLL3_MAIN_CP_VP_DIODE ; // 20150903@Gason, fix PLL issue.
*/
			#else
			 MMPF_PLL_PowerUp(MMPF_PLL_5, 0);
		         MMPF_PLL_WaitCount(50);
		         MMPF_PLL_PowerUp(MMPF_PLL_5, 1);
		         MMPF_PLL_WaitCount(50);
			#endif
	
		    
		} else {
			if (AudPath == MMPF_AUDSRC_MCLK) {
#if 1
			#ifdef DPLL_3A
				// Post-Divider
			    pGBL->GBL_DPLL3_M = DPLL3_M(19); // 0x5d33
			    // Loop-Divider
			    pGBL->GBL_DPLL3_N = DPLL3_N(19-2); // 0x5d34
			    // Fractional Part
			    pGBL->GBL_DPPL3_PARAM[0x00] = 0x95; // 0x5d35
			    pGBL->GBL_DPPL3_PARAM[0x01] = 0x43; // 0x5d36
			    pGBL->GBL_DPPL3_PARAM[0x02] = 0x03; // 0x5d37
			    pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 20150903@Gason, fix PLL issue.
			#else
                            pGBL->GBL_DPPL5_PARAM[0x05] = 0x04; // 0x5d5a
                            // PLL Settings
                            pGBL->GBL_DPPL5_PARAM[0x04] = DPLL5_DIV2_LOOPDIV_EN | DPLL5_SS_DOWN; // 5d59
                            pGBL->GBL_DPPL5_PARAM[0x03] = DPLL5_CHARGE_PUMP_5uA; // 5d58		
                            pGBL->GBL_DPPL5_PARAM[0x03] |= DPLL5_KVCO_OFFSET_10uA; // 5d58
		            pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_1;
			    //pGBL->GBL_DPPL5_PARAM[0x02] = 1;//pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Post-Divider
			    //pGBL->GBL_DPPL5_PARAM[0x02] = pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Fractional Part
			    pGBL->GBL_DPPL5_PARAM[0x00] = 0x4D;//0x40;//0x00; // 0x5d55
			    pGBL->GBL_DPPL5_PARAM[0x01] = 0xF3;//0xD8;//0x80; // 0x5d56
			    pGBL->GBL_DPPL5_PARAM[0x02] = 0x02;//0x30;//0x30//0x04; // 0x5d57
			    // Loop Divider N part
		            pGBL->GBL_DPLL5_N = 67;//3//pllSettings[MMPF_PLL_ID_5].N; // 0x5d54	
		            // Pre-Divider Pass
			    pGBL->GBL_DPLL5_P =DPLL5_P_DIV_4;// DPLL5_P_DIV_4;//pllSettings[MMPF_PLL_ID_5].K; // 0x5d53
			    // change into  OP mode
		            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
		
		            pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
		            MMPF_PLL_WaitCount(50);

                            pGBL->GBL_DPPL5_PARAM[0x05] = 0x04; // 0x5d5a
                            // PLL Settings
                            pGBL->GBL_DPPL5_PARAM[0x04] = DPLL5_DIV2_LOOPDIV_EN | DPLL5_SS_DOWN; // 5d59
                            pGBL->GBL_DPPL5_PARAM[0x03] = DPLL5_CHARGE_PUMP_5uA; // 5d58		
                            pGBL->GBL_DPPL5_PARAM[0x03] |= DPLL5_KVCO_OFFSET_10uA; // 5d58

		            //pGBL->GBL_AUD_CLK_SRC = 0x06;//AUD_CLK_SRC_DPLL5;
		            //pGBL->GBL_DPLL5_CFG |= DPLL_BYPASS;
	                    pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_1;
			    //pGBL->GBL_DPPL5_PARAM[0x02] = 1;//pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Post-Divider
			    //pGBL->GBL_DPPL5_PARAM[0x02] = pllSettings[MMPF_PLL_ID_5].M; // 5d57
			    // Fractional Part
			    pGBL->GBL_DPPL5_PARAM[0x00] = 0x4D;//0x40;//0x40;//0x00; // 0x5d55
			    pGBL->GBL_DPPL5_PARAM[0x01] = 0xF3;//0xD8;//0xD8;//0x80; // 0x5d56
			    pGBL->GBL_DPPL5_PARAM[0x02] = 0x02;//0x30;//0x30;//0x04; // 0x5d57
			    // Loop Divider N part
		            pGBL->GBL_DPLL5_N = 67;//3//pllSettings[MMPF_PLL_ID_5].N; // 0x5d54	
		            // Pre-Divider Pass
			    pGBL->GBL_DPLL5_P = DPLL5_P_DIV_4;//DPLL5_P_DIV_4;//pllSettings[MMPF_PLL_ID_5].K; // 0x5d53
			    // change into  OP mode
		            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
			    pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
		            MMPF_PLL_WaitCount(50);
			#endif
#else
                            // Start-UP @ Diode Mode
                            pGBL->GBL_DPPL5_PARAM[0x05] = 0x04; // 0x5d5a
                            // PLL Settings
                            pGBL->GBL_DPPL5_PARAM[0x04] = DPLL5_DIV2_LOOPDIV_EN | DPLL5_SS_DOWN; // 5d59
                            pGBL->GBL_DPPL5_PARAM[0x03] = DPLL5_CHARGE_PUMP_5uA; // 5d58		
                            pGBL->GBL_DPPL5_PARAM[0x03] |= DPLL5_KVCO_OFFSET_10uA; // 5d58
                            // Post Divider Divide 2
                            pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_2;//pllSettings[MMPF_PLL_ID_5].M; // 5d57
                            pGBL->GBL_DPPL5_PARAM[0x02] |= 0x40; // 5d57 
	                    pGBL->GBL_DPPL5_PARAM[0x01] = 0xD8; // 5d56
	                    pGBL->GBL_DPPL5_PARAM[0x00] = 0x30; // 5d55
                            // Loop Divider N part
                            pGBL->GBL_DPLL5_N = 8; // 0x5d54
                            // Pre-Divider Pass
		            pGBL->GBL_DPLL5_P = DPLL5_P_DIV_1; // 0x5d53
		            MMPF_PLL_WaitCount(50);  
		            /*// change into  OP mode
		            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
		            MMPF_PLL_WaitCount(50);  
		            //Update DPLL5 configuration
		            pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50 
		            // PLL bug, must update twice.*/
		            // change into  OP mode
		            pGBL->GBL_DPPL5_PARAM[0x05] = 0x02;//DPLL5_OUTPUT_DIV_2; // 0x5d5a
		            MMPF_PLL_WaitCount(50);  
		            //Update DPLL5 configuration
		            pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50 
		            MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 0);
	                    MMPF_PLL_WaitCount(50);
	                    MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 1);	
#endif
			}
			else if (AudPath == MMPF_AUDSRC_I2S) {
				#ifdef DPLL_3A
				// Post-Divider
			    pGBL->GBL_DPLL3_M = 0x13; // 0x5d33
			    // Loop-Divider
			    pGBL->GBL_DPLL3_N = 0x92; // 0x5d34
			    // Fractional Part
			    pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
			    pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
			    pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37
			    pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 20150903@Gason, fix PLL issue.
			    pGBL->GBL_DPLL3_CFG &=~ DPLL_SRC_MASK;
			    pGBL->GBL_DPLL3_CFG |= DPLL_SRC_PI2S_MCLK;				
				
				#endif
			}
			#ifdef DPLL_3A	
			//Update DPLL5 configuration
			pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30 
			MMPF_PLL_PowerUp(MMPF_PLL_3, 0);
			MMPF_PLL_WaitCount(50);
			MMPF_PLL_PowerUp(MMPF_PLL_3, 1);
			MMPF_PLL_WaitCount(50);
		        pGBL->GBL_DPPL3_PARAM[4] &= ~DPLL3_MAIN_CP_VP_DIODE ; // 20150903@Gason, fix PLL issue.

			#else
			MMPF_PLL_PowerUp(MMPF_PLL_5, 0);
		        MMPF_PLL_WaitCount(50);
		        MMPF_PLL_PowerUp(MMPF_PLL_5, 1);
		        MMPF_PLL_WaitCount(50);
			#endif
		}   
	}  
	return err ;
}    

#endif
AITPS_GBL g_pGBL = AITC_BASE_GBL;

MMP_ERR MMPF_PLL_SetAudioPLL(MMPF_PLL_MODE ulSampleRate,LINUX_SOC_PATH path,MMP_BYTE reset_pll)
{
#if (CHIP == VSN_V3)
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ERR   err = MMP_ERR_NONE;

    pGBL->GBL_AUDIO_CLK_DIV = GBL_AUDIO_CLK_SRC_DPLL2|GBL_AUDIO_CLK_ON|(GBL_AUDIO_CLK_DIV_MASK&10);

    return err;
#endif
#if (CHIP==MCR_V2)

	AITPS_GBL pGBL = AITC_BASE_GBL;
	MMP_ERR   err = MMP_ERR_NONE;
#ifdef DPLL_3A
	pGBL->GBL_DPLL3_CFG |= DPLL_BYPASS;
#else
	pGBL->GBL_DPLL5_CFG |= DPLL_BYPASS;
#endif	

//DEBUG for pll
//#ifdef DPLL_3A
//	 pGBL->GBL_VIF_CLK_SRC = AUD_CLK_SRC_DPLL3;
//#else
//	 pGBL->GBL_VIF_CLK_SRC = AUD_CLK_SRC_DPLL5;
//#endif
          
 //       if (m_ClockSrc.sensor_div)
 //      pGBL->GBL_VIF_CLK_DIV = GRP_CLK_DIV_EN | 1;//GRP_CLK_DIV(m_ClockSrc.sensor_div);


	if ((ulSampleRate != MMPF_PLL_AUDIO_44d1K) && 
		(ulSampleRate != MMPF_PLL_AUDIO_22d05K) &&
		(ulSampleRate != MMPF_PLL_AUDIO_11d025K) ) { 
		/*dbg_printf(0,"++ 0x5d35 b: %x\r\n",pGBL->GBL_DPPL3_PARAM[0x00]);
		dbg_printf(0,"++ 0x5d36 b: %x\r\n",pGBL->GBL_DPPL3_PARAM[0x01]);
		dbg_printf(0,"++ 0x5d37 b: %x\r\n",pGBL->GBL_DPPL3_PARAM[0x02]);*/
		MMPF_PLL_ChangeAudioPLL(/*(m_bAudSampleGroup!= 0)*/1, ulSampleRate, MMPF_AUDSRC_MCLK);
	#ifdef DPLL_3A               
        pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(6); // 0x5da1
	#else
        pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(24); // 0x5da1
	#endif
		
    //if((ubPath & AUDIO_AFE_MASK)< 0x04 ) {
    if(path==AFE_PATH_ADC) {
        //	RTNA_DBG_Str0("+ADC\r\n");
		    switch (ulSampleRate) { // AFE	    
		    case MMPF_PLL_AUDIO_48K:           
	            pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(4); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(2); // x5da3
	            break;
		    case MMPF_PLL_AUDIO_24K:           
	            pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(8); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(4); // x5da3
		    	break;
		    case MMPF_PLL_AUDIO_12K:
				pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(16); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(8); // x5da3
		    	break;
		    case MMPF_PLL_AUDIO_32K:
		    	pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(6); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(3); // x5da3
		    	break;	
		    case MMPF_PLL_AUDIO_16K:
		    	pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(12); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(6); // x5da3
		    	break;	
		    case MMPF_PLL_AUDIO_8K:
		    	pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(24); // x5da2
	    		pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(12); // x5da3
		    	break;			
		    default:
	            RTNA_DBG_Str0("Invalid PLL frequency\r\n");
	            err = MMP_SYSTEM_ERR_SETPLL;
	            break;
	        }
	        //dbg_printf(0,"AUDCLK_D1_D2 : %x,%x\r\n",pGBL->GBL_AUD_CLK_DIV[0x01],pGBL->GBL_AUD_CLK_DIV[0x02]);
	    }    
      //  if((ubPath & AUDIO_AFE_MASK)>= 0x04 ) {
      else {
        	//RTNA_DBG_Str0("+DAC\r\n");
	        switch (ulSampleRate) { // DAC
		    case MMPF_PLL_AUDIO_48K:           
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(4); // x5da4
	    		pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(2); // x5da5
	            break;
	        case MMPF_PLL_AUDIO_24K:                
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(8); // x5da4
	            pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(4); // x5da5    
		    	break;
		    case MMPF_PLL_AUDIO_12K:
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(16); // x5da4
	            pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(8); // x5da5    
		    	break;
		    case MMPF_PLL_AUDIO_32K:
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(6); // x5da4
	            pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(3); // x5da5    
		    	break;	
		    case MMPF_PLL_AUDIO_16K:
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(12); // x5da4
	            pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(6); // x5da5    
		    	break;	
		    case MMPF_PLL_AUDIO_8K:
	            pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(24); // x5da4
	            pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(12); // x5da5    
		    	break;			
		    default:
	            RTNA_DBG_Str0("Invalid PLL frequency\r\n");
	            err = MMP_SYSTEM_ERR_SETPLL;
	            break;
	        } 
        }
        m_bAudSampleGroup =0;
    	
	}
	else { // 22.5792	
        //pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(6); // 0x5da1	
		MMPF_PLL_ChangeAudioPLL(/*(m_bAudSampleGroup!= 1)*/1, ulSampleRate, MMPF_AUDSRC_MCLK);
	    switch (ulSampleRate) { // AFE
#ifdef DPLL_3A         
	    case MMPF_PLL_AUDIO_44d1K:
	    	pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(2); // 0x5da1
	    	break;
	    case MMPF_PLL_AUDIO_22d05K:
	    	pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(4); // 0x5da1
	    	break;
	    case MMPF_PLL_AUDIO_11d025K:
			pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(8); // 0x5da1
	    	break;		
#else
	    case MMPF_PLL_AUDIO_44d1K:
	    	pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(6); // 0x5da1
	    	break;
	    case MMPF_PLL_AUDIO_22d05K:
	    	pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(12); // 0x5da1
	    	break;
	    case MMPF_PLL_AUDIO_11d025K:
			pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(24); // 0x5da1
	    	break;		
#endif
	    default:
            RTNA_DBG_Str0("Invalid PLL frequency\r\n");
            err = MMP_SYSTEM_ERR_SETPLL;
        }
#ifdef DPLL_3A         
        pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(2); // x5da2
    	pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(1); // x5da3
    	pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(2); // x5da4
    	pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(1); // x5da5
#else
        pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(24/*2*/); // x5da2
    	pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(12/*1*/); // x5da3
    	pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(24/*2*/); // x5da4
    	pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(12/*1*/); // x5da5
#endif    	
    	m_bAudSampleGroup =1;
	}
	#ifdef DPLL_3A
	pGBL->GBL_DPLL3_CFG &= ~DPLL_BYPASS;
	#else
	pGBL->GBL_DPLL5_CFG &= ~DPLL_BYPASS;
	#endif
	
	//dbg_printf(0,"m_AudioPllFrac:[%d]\r\n",m_AudioPllFrac);
	if(!m_AudioPllFrac) {
#ifdef DPLL_3A
    	m_AudioPllFrac = GET_PLL_FRAC(pGBL->GBL_DPPL3_PARAM);    	
#else
	m_AudioPllFrac = GET_PLL_FRAC(pGBL->GBL_DPPL5_PARAM);    	
#endif
    	}
    
    m_AudioPllFracCur = m_AudioPllFrac;
    m_AudioPllFracMax = m_AudioPllFrac + FRAC_RANGE ;//0x4000;
    if(m_AudioPllFrac < FRAC_RANGE) {
    	m_AudioPllFrac = 0;
    } 
    else {
    	m_AudioPllFracMin = m_AudioPllFrac - FRAC_RANGE ;//0x4000;
    }
    // for UAC speaker / mic
    m_AudioPllFracCur = m_AudioPllFracCur ;
#ifdef DPLL_3A
    pGBL->GBL_DPPL3_PARAM[0x00] = m_AudioPllFracCur & 0xFF;
    pGBL->GBL_DPPL3_PARAM[0x01] = (m_AudioPllFracCur >>  8) & 0xFF;
    pGBL->GBL_DPPL3_PARAM[0x02] = (m_AudioPllFracCur >> 16) & 0xFF;

    pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM;
#else
    pGBL->GBL_DPPL5_PARAM[0x00] = m_AudioPllFracCur & 0xFF;
    pGBL->GBL_DPPL5_PARAM[0x01] = (m_AudioPllFracCur >>  8) & 0xFF;
    pGBL->GBL_DPPL5_PARAM[0x02] = (m_AudioPllFracCur >> 16) & 0xFF;

    pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM;
#endif
    #if 0
    dbg_printf(0,"MMPF_PLL_SetAudioPLL:[%d,%d],cur:%d>\r\n",m_AudioPllFracMin,m_AudioPllFracMax,m_AudioPllFracCur);
    dbg_printf(0,"++pGBL->GBL_AUD_CLK_DIV[0x00]:%x\r\n",pGBL->GBL_AUD_CLK_DIV[0x00]);
    dbg_printf(0,"++ 0x5d35 : %x\r\n",pGBL->GBL_DPPL3_PARAM[0x00]);
    dbg_printf(0,"++ 0x5d36 : %x\r\n",pGBL->GBL_DPPL3_PARAM[0x01]);
    dbg_printf(0,"++ 0x5d37 : %x\r\n",pGBL->GBL_DPPL3_PARAM[0x02]);    
    #endif
    
    return err;
    
#endif

}
EXPORT_SYMBOL(MMPF_PLL_SetAudioPLL);

#if 0 
MMP_ERR MMPF_PLL_SetAudioPLL(MMPF_PLL_MODE ulSampleRate)
{
#if (CHIP == VSN_V3)
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ERR   err = MMP_ERR_NONE;

    pGBL->GBL_AUDIO_CLK_DIV = GBL_AUDIO_CLK_SRC_DPLL2|GBL_AUDIO_CLK_ON|(GBL_AUDIO_CLK_DIV_MASK&10);

    return err;
#endif
#if (CHIP == MCR_V2)
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ERR   err = MMP_ERR_NONE;

    pGBL->GBL_DPLL3_CFG |= DPLL_BYPASS;

    if ((ulSampleRate != MMPF_PLL_AUDIO_44d1K) && (ulSampleRate != MMPF_PLL_AUDIO_22d05K) &&
        (ulSampleRate != MMPF_PLL_AUDIO_11d025K) ) { // 24.576
        // Post-Divider
        pGBL->GBL_DPLL3_M = m_PllSettings[MMPF_PLL_ID_3].M; // 0x5d33
        // Loop-Divider
        pGBL->GBL_DPLL3_N = m_PllSettings[MMPF_PLL_ID_3].N; // 0x5d34
        // Fractional Part
        pGBL->GBL_DPPL3_PARAM[0x00] = 0x85; // 0x5d35
        pGBL->GBL_DPPL3_PARAM[0x01] = 0xEB; // 0x5d36

        //Update DPLL5 configuration
        pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 1);
        MMPF_PLL_WaitCount(50);

        switch (ulSampleRate) { // AFE
        case MMPF_PLL_AUDIO_48K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(2); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_24K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(4); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_12K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(8); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_32K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(3); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_16K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(6); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_8K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(12); // 0x5da1
            break;
        default:
            RTNA_DBG_Str0("Invalid PLL frequency\r\n");
            err = MMP_SYSTEM_ERR_SETPLL;
        }
        pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(2); // x5da2
        pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(1); // x5da3
        pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(2); // x5da4
        pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(1); // x5da5
    }
    else { // 22.5792
        // Post-Divider
        pGBL->GBL_DPLL3_M = m_PllSettings[MMPF_PLL_ID_3].M; // 0x5d33
        // Loop-Divider
        pGBL->GBL_DPLL3_N = m_PllSettings[MMPF_PLL_ID_3].N-2; // 0x5d34
        // Fractional Part
        pGBL->GBL_DPPL3_PARAM[0x00] = 0x95; // 0x5d35
        pGBL->GBL_DPPL3_PARAM[0x01] = 0x43; // 0x5d36
        pGBL->GBL_DPPL3_PARAM[0x02] = 0x03; // 0x5d37

        //Update DPLL5 configuration
        pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 1);
        MMPF_PLL_WaitCount(50);

        switch (ulSampleRate) { // AFE
        case MMPF_PLL_AUDIO_44d1K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(2); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_22d05K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(4); // 0x5da1
            break;
        case MMPF_PLL_AUDIO_11d025K:
            pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(8); // 0x5da1
            break;
        default:
            RTNA_DBG_Str0("Invalid PLL frequency\r\n");
            err = MMP_SYSTEM_ERR_SETPLL;
        }
        pGBL->GBL_AUD_CLK_DIV[0x01] = AUD_CLK_DIV(2); // x5da2
        pGBL->GBL_AUD_CLK_DIV[0x02] = AUD_CLK_DIV(1); // x5da3
        pGBL->GBL_AUD_CLK_DIV[0x03] = AUD_CLK_DIV(2); // x5da4
        pGBL->GBL_AUD_CLK_DIV[0x04] = AUD_CLK_DIV(1); // x5da5
    }

    pGBL->GBL_DPLL3_CFG &= ~DPLL_BYPASS;

    return err;
#endif
}
#endif 



#else

//------------------------------------------------------------------------------
//  Function    : MMPF_PLL_SetAudioPLL
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_PLL_SetAudioPLL(MMPF_PLL_MODE pllMode, MMPF_GROUP_SRC src, MMP_UBYTE path)
{
    #if (AUDIO_P_EN)||(AUDIO_R_EN)||(VIDEO_P_EN)||(VIDEO_R_EN)

    AITPS_GBL   pGBL = AITC_BASE_GBL;
    MMP_ERR     err = MMP_ERR_NONE;
    MMP_ULONG   audDivider = 0;
    MMP_ULONG   audGrpClk = 0;
    MMPF_AUDIO_I2S_MCLK_MODE i2sMclkMode;

    if (path & AUDIO_AFE_MASK) 
    {
        //Divide clock to less than 48MHz (can't over 50MHz)
        audDivider = (m_ulGrpFreq[5]/48000 < 64) ? m_ulGrpFreq[5]/48000 : 63;
        pGBL->GBL_CLK_AUD_DIV = audDivider;
    
        if (audDivider)
            audGrpClk = m_ulGrpFreq[5]/audDivider;
        else
            audGrpClk = m_ulGrpFreq[5];
    }
    else 
    {
        MMPF_I2S_GetMclkMode(I2S_CH0, &i2sMclkMode);
        
        if (i2sMclkMode == MMPF_AUDIO_I2S_FIX_256FS_MODE) {
            
            if ((((gbSystemCoreID == CHIP_CORE_ID_PV2) || (gbSystemCoreID == CHIP_CORE_ID_PV2_ECO)) && (src >= MMPF_PLL_NULL))) {
                RTNA_DBG_Str0("Invalid PLL source for I2S fixed 256fs MCLK mode!\r\n");
                return MMP_SYSTEM_ERR_SETPLL;
            }
            
            switch(pllMode) {
            case MMPF_PLL_AUDIO_192K:
            case MMPF_PLL_AUDIO_96K:
                err = MMPF_PLL_PowerUpWithFreq(src, MMPF_PLL_FREQ_98d3040MHz, gCPUFreq/800);
                pGBL->GBL_CLK_AUD_DIV = MMPF_PLL_TABLE_PV2_1[MMPF_PLL_FREQ_98d3040MHz][3];
                m_ulGrpFreq[5] = 98304;
                m_PllFreq[src] = MMPF_PLL_FREQ_98d3040MHz;
                break;
            case MMPF_PLL_AUDIO_128K:
            case MMPF_PLL_AUDIO_64K:
                err = MMPF_PLL_PowerUpWithFreq(src, MMPF_PLL_FREQ_65d5360MHz, gCPUFreq/800);
                pGBL->GBL_CLK_AUD_DIV = MMPF_PLL_TABLE_PV2_1[MMPF_PLL_FREQ_65d5360MHz][3];
                m_ulGrpFreq[5] = 65536;
                m_PllFreq[src] = MMPF_PLL_FREQ_65d5360MHz;
                break;
            case MMPF_PLL_AUDIO_48K:
            case MMPF_PLL_AUDIO_32K:
            case MMPF_PLL_AUDIO_24K:
            case MMPF_PLL_AUDIO_16K:
            case MMPF_PLL_AUDIO_12K:
            case MMPF_PLL_AUDIO_8K:
                err = MMPF_PLL_PowerUpWithFreq(src, MMPF_PLL_FREQ_24d5760MHz, gCPUFreq/800);
                pGBL->GBL_CLK_AUD_DIV = MMPF_PLL_TABLE_PV2_1[MMPF_PLL_FREQ_24d5760MHz][3];
                m_ulGrpFreq[5] = 24576;
                m_PllFreq[src] = MMPF_PLL_FREQ_24d5760MHz;
                break;
            case MMPF_PLL_AUDIO_44d1K:
            case MMPF_PLL_AUDIO_22d05K:
            case MMPF_PLL_AUDIO_11d025K:
                err = MMPF_PLL_PowerUpWithFreq(src, MMPF_PLL_FREQ_22d5792MHz, gCPUFreq/800);
                pGBL->GBL_CLK_AUD_DIV = MMPF_PLL_TABLE_PV2_1[MMPF_PLL_FREQ_22d5792MHz][3];
                m_ulGrpFreq[5] = 22580;
                m_PllFreq[src] = MMPF_PLL_FREQ_22d5792MHz;
                break;
            default:
                RTNA_DBG_Str0("Invalid PLL frequency\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            return err;
        }
    }

    switch(pllMode) {
    case MMPF_PLL_AUDIO_192K:
    case MMPF_PLL_AUDIO_128K:
        if (path & AUDIO_AFE_MASK) {
        
            if ((audGrpClk % 48000) != 0) {
                RTNA_DBG_Str(0, "Warning! Group AUD clock must be a multiple of 48MHz!\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            //Codec clock must be 24MHz
            audDivider = (audGrpClk/24000 < 64) ? audGrpClk/24000 : 63;
            if (audDivider)
                pGBL->GBL_CLK_AUDAFE_DIV = audDivider - 1;
            else
                pGBL->GBL_CLK_AUDAFE_DIV = 0;
                
            #if (HIGH_SRATE_MODE == DOWN_SAMPLE_TIMES)
            //Digital clock must be 48MHz
            audDivider = (audGrpClk/48000 < 64) ? audGrpClk/48000 : 63;
            #elif (HIGH_SRATE_MODE == BYPASS_FILTER_STAGE)
            //Digital clock must be 12MHz
            audDivider = (audGrpClk/12000 < 64) ? audGrpClk/12000 : 63;
            #endif
            pGBL->GBL_CLK_AUDCD_DIV = audDivider;
            break;
        }
        else if ((i2sMclkMode == MMPF_AUDIO_I2S_256FS_MODE) || (i2sMclkMode == MMPF_AUDIO_I2S_USB_MODE)) {
            //Divide clock to less than 48MHz (can't over 50MHz)
            audDivider = (m_ulGrpFreq[5]/48000 < 64) ? m_ulGrpFreq[5]/48000 : 63;
            pGBL->GBL_CLK_AUD_DIV = audDivider;
            break;
        }
        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Set Audio PLL48-32.\r\n");
        break;
    case MMPF_PLL_AUDIO_96K:
    case MMPF_PLL_AUDIO_64K:
        if (path & AUDIO_AFE_MASK) {
        
            if ((audGrpClk % 24000) != 0) {
                RTNA_DBG_Str(0, "Warning! Group AUD clock must be a multiple of 24MHz!\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            //Codec clock must be 24MHz
            audDivider = (audGrpClk/24000 < 64) ? audGrpClk/24000 : 63;
            if (audDivider)
                pGBL->GBL_CLK_AUDAFE_DIV = audDivider - 1;
            else
                pGBL->GBL_CLK_AUDAFE_DIV = 0;
                
            #if (HIGH_SRATE_MODE == DOWN_SAMPLE_TIMES)
            //Digital clock must be 24MHz
            audDivider = (audGrpClk/24000 < 64) ? audGrpClk/24000 : 63;
            #elif (HIGH_SRATE_MODE == BYPASS_FILTER_STAGE)
            //Digital clock must be 12MHz
            audDivider = (audGrpClk/12000 < 64) ? audGrpClk/12000 : 63;
            #endif
            pGBL->GBL_CLK_AUDCD_DIV = audDivider;
            break;
        }
        else if ((i2sMclkMode == MMPF_AUDIO_I2S_256FS_MODE) || (i2sMclkMode == MMPF_AUDIO_I2S_USB_MODE)) {
            //Divide clock to less than 48MHz (can't over 50MHz)
            audDivider = (m_ulGrpFreq[5]/48000 < 64) ? m_ulGrpFreq[5]/48000 : 63;
            pGBL->GBL_CLK_AUD_DIV = audDivider;
            break;
        }
        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Set Audio PLL48-32.\r\n");
        break;
    case MMPF_PLL_AUDIO_48K:
    case MMPF_PLL_AUDIO_44d1K:
    case MMPF_PLL_AUDIO_32K:
        if (path & AUDIO_AFE_MASK) {
        
            if ((audGrpClk % 24000) != 0) {
                RTNA_DBG_Str(0, "Warning! Group AUD clock must be a multiple of 24MHz!\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            //Codec clock must be 24MHz
            audDivider = (audGrpClk/24000 < 64) ? audGrpClk/24000 : 63;
            if (audDivider)
                pGBL->GBL_CLK_AUDAFE_DIV = audDivider - 1;
            else
                pGBL->GBL_CLK_AUDAFE_DIV = 0;
                
            //Digital clock must be 12MHz
            audDivider = (audGrpClk/12000 < 64) ? audGrpClk/12000 : 63;
            pGBL->GBL_CLK_AUDCD_DIV = audDivider;
            break;
        }
        else if ((i2sMclkMode == MMPF_AUDIO_I2S_256FS_MODE) || (i2sMclkMode == MMPF_AUDIO_I2S_USB_MODE)) {
            //Divide clock to less than 48MHz (can't over 50MHz)
            audDivider = (m_ulGrpFreq[5]/48000 < 64) ? m_ulGrpFreq[5]/48000 : 63;
            pGBL->GBL_CLK_AUD_DIV = audDivider;
            break;
        }
        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Set Audio PLL48-32.\r\n");
        break;
    case MMPF_PLL_AUDIO_24K:
    case MMPF_PLL_AUDIO_22d05K:
    case MMPF_PLL_AUDIO_16K:
        if (path & AUDIO_AFE_MASK) {
        
            if ((audGrpClk % 12000) != 0) {
                RTNA_DBG_Str(0, "Warning! Group AUD clock must be a multiple of 12MHz!\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            //Codec clock must be 12MHz
            audDivider = (audGrpClk/12000 < 64) ? audGrpClk/12000 : 63;
            if (audDivider)
                pGBL->GBL_CLK_AUDAFE_DIV = audDivider - 1;
            else
                pGBL->GBL_CLK_AUDAFE_DIV = 0;
                
            //Digital clock must be 6MHz
            audDivider = (audGrpClk/6000 < 64) ? audGrpClk/6000 : 63;
            pGBL->GBL_CLK_AUDCD_DIV = audDivider;
        }
        else if ((i2sMclkMode == MMPF_AUDIO_I2S_256FS_MODE) || (i2sMclkMode == MMPF_AUDIO_I2S_USB_MODE)) {
            //Divide clock to less than 24MHz (can't over 50MHz)
            audDivider = (m_ulGrpFreq[5]/24000 < 64) ? m_ulGrpFreq[5]/24000 : 63;
            pGBL->GBL_CLK_AUD_DIV = audDivider;
        }
        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Set Audio PLL24-16.\r\n");
        break;
    case MMPF_PLL_AUDIO_12K:
    case MMPF_PLL_AUDIO_11d025K:
    case MMPF_PLL_AUDIO_8K:
        if (path & AUDIO_AFE_MASK) {
        
            if ((audGrpClk % 6000) != 0) {
                RTNA_DBG_Str(0, "Warning! Group AUD clock must be a multiple of 6MHz!\r\n");
                err = MMP_SYSTEM_ERR_SETPLL;
            }
            //Codec clock must be 6MHz
            audDivider = (audGrpClk/6000 < 64) ? audGrpClk/6000 : 63;
            if (audDivider)
                pGBL->GBL_CLK_AUDAFE_DIV = audDivider - 1;
            else
                pGBL->GBL_CLK_AUDAFE_DIV = 0;
                
            //Digital clock must be 3MHz
            audDivider = (audGrpClk/3000 < 64) ? audGrpClk/3000 : 63;
            pGBL->GBL_CLK_AUDCD_DIV = audDivider;
        }
        else if ((i2sMclkMode == MMPF_AUDIO_I2S_256FS_MODE) || (i2sMclkMode == MMPF_AUDIO_I2S_USB_MODE)) {
            //Divide clock to less than 24MHz (can't over 50MHz)
            audDivider = (m_ulGrpFreq[5]/24000 < 64) ? m_ulGrpFreq[5]/24000 : 63;
            pGBL->GBL_CLK_AUD_DIV = audDivider;
        }
        RTNA_DBG_Str(0, "MMPF_PLL_Setting: Set Audio PLL12-8.\r\n");
        break;
    default:
        RTNA_DBG_Str0("Invalid PLL frequency\r\n");
        err = MMP_SYSTEM_ERR_SETPLL;
    }

    return err;
    #endif

    return MMP_ERR_NONE;
}
#endif 

#if (OS_TYPE == OS_UCOSII)

#if (CHIP == MCR_V2)
MMP_ERR MMPF_PLL_SetTVCLK(MMPF_PLL_ID PLLSrc)
{
    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_ERR   err = MMP_ERR_NONE;

    switch(PLLSrc) {
    case MMPF_PLL_ID_0:
        // 432MHZ
        // pre-divider
        pGBL->GBL_DPLL0_M = 0x00; // 0x5d03
        // loop divider
        pGBL->GBL_DPLL0_N = 0x11; // 0x5d04
        pGBL->GBL_DPPL0_PARAM[0x00] = 0x00; // 0x5d05
        pGBL->GBL_DPPL0_PARAM[0x01] = 0x00; // 0x5d06
        // post divider
        pGBL->GBL_DPPL0_PARAM[0x02] = 0x0C;  // 0x5d07
        // VCO Range, RSEL
        pGBL->GBL_DPPL0_PARAM[0x03] = DPLL_384_1008MHZ;// 0x5d08
        pGBL->GBL_DPPL0_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d08
        pGBL->GBL_DPPL0_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d08
        // V2I no offset
        pGBL->GBL_DPPL0_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d09
        // maximum KVO
        pGBL->GBL_DPPL0_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d0a
        // charge pump
        pGBL->GBL_DPPL0_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d0b
        pGBL->GBL_DPPL0_PARAM[0x07] = 0x00; // 0x5d0c
        //Update DPLL0 configuration
        pGBL->GBL_DPLL0_CFG |= DPLL_UPDATE_PARAM; // 0x5d00
        MMPF_PLL_PowerUp(MMPF_PLL_ID_0, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_0, 1);

        // tv divide
        pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(0x08);
        break;
    case MMPF_PLL_ID_1:
        // 432MHZ
        // Pre-Divider
        pGBL->GBL_DPLL1_M = 0x00; // 0x5d13
        // Loop-Divider
        pGBL->GBL_DPLL1_N = 0x11; // 0x5d14
        pGBL->GBL_DPPL1_PARAM[0x00] = 0x00; // 0x5d15
        pGBL->GBL_DPPL1_PARAM[0x01] = 0x00; // 0x5d16
        // Post-Divider
        pGBL->GBL_DPPL1_PARAM[0x02] = 0x0C;  // 0x5d17
        // VCO Range, RSEL
        pGBL->GBL_DPPL1_PARAM[0x03] = DPLL_384_1008MHZ;// 0x5d18
        pGBL->GBL_DPPL1_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d18
        pGBL->GBL_DPPL1_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d18
        // V2I no offset
        pGBL->GBL_DPPL1_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d19
        // maximum KVO
        pGBL->GBL_DPPL1_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d1a
        // charge pump
        pGBL->GBL_DPPL1_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d1b
        pGBL->GBL_DPPL1_PARAM[0x07] = 0x00; //0x5d1c
        //Update DPLL1 configuration
        pGBL->GBL_DPLL1_CFG |= DPLL_UPDATE_PARAM; // 0x5d10
        MMPF_PLL_PowerUp(MMPF_PLL_ID_1, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_1, 1);

        // tv divide
        pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(0x08);
        break;
    case MMPF_PLL_ID_2:
        // 432MHZ
        // Pre-Divider
        pGBL->GBL_DPLL2_M = 0x00; // 0x5d23
        // Loop-Divider
        pGBL->GBL_DPLL2_N = 0x11; // 0x5d24
        pGBL->GBL_DPPL2_PARAM[0x00] = 0x00; // 0x5d25
        pGBL->GBL_DPPL2_PARAM[0x01] = 0x00; // 0x5d26
        // Post-Divider
        pGBL->GBL_DPPL2_PARAM[0x02] = 0x0C;  // 0x5d27
        // VCO Range, RSEL
        pGBL->GBL_DPPL2_PARAM[0x03] = DPLL_384_1008MHZ;// 0x5d28
        pGBL->GBL_DPPL2_PARAM[0x03] |= DPLL_V2I_HIGH_GAIN;  // 0x5d28
        pGBL->GBL_DPPL2_PARAM[0x03] |= DPLL_LFS_80 | DPLL_BIAS_OPTION_1000; // 0x5d28
        // V2I no offset
        pGBL->GBL_DPPL2_PARAM[0x04] = DPLL_LOCK_DETECTOR_RANGE_20 | DPLL_V2I_GAIN_ADJUST_DIS; //0x5d29
        // maximum KVO
        pGBL->GBL_DPPL2_PARAM[0x05] = DPLL_BIAS_CTL_VCO_15uA; // 0x5d2a
        // charge pump
        pGBL->GBL_DPPL2_PARAM[0x06] = DPLL_BIAS_CTL_ICP_10uA; // 0x5d2b
        pGBL->GBL_DPPL2_PARAM[0x07] = 0x00; // 5d2c
        //Update DPLL2 configuration
        pGBL->GBL_DPLL2_CFG |= DPLL_UPDATE_PARAM; // 0x5d20
        MMPF_PLL_PowerUp(MMPF_PLL_ID_2, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_2, 1);

        // tv divide
        pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(0x08);
        break;
    case MMPF_PLL_ID_3:
        // 54MHZ
        // Post-Divider
        pGBL->GBL_DPLL3_M = 0x07; // 0x5d33
        // Loop-Divider
        pGBL->GBL_DPLL3_N = 0x11; // 0x5d34
        // Fractional Part
        pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
        pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
        pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37
        // PLL Control Setting
        pGBL->GBL_DPPL3_PARAM[0x03] = DPLL3_FRAC_DIS; // 0x5d38
        // Enable BOOST CP & MAIN CP DIODE MODE
        pGBL->GBL_DPPL3_PARAM[0x04] = DPLL3_BOOST_CP_EN | DPLL3_MAIN_CP_VP_DIODE; // 0x5d39
        MMPF_PLL_WaitCount(50);
        // Enable BOOST CP & MAIN CP OP MODE
        pGBL->GBL_DPPL3_PARAM[0x04] &= ~DPLL3_MAIN_CP_VP_DIODE; // 0x5d39
        MMPF_PLL_WaitCount(50);
        //Update DPLL3 configuration
        pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_3, 1);
        break;
    case MMPF_PLL_ID_4:
        // 378MHZ
        // Pre-Divider
        pGBL->GBL_DPLL4_M = m_PllSettings[MMPF_PLL_ID_4].M; // 0x5d43
        // Loop-Divider
        pGBL->GBL_DPLL4_N = m_PllSettings[MMPF_PLL_ID_4].N; // 0x5d44
        // SS frequency divider
        pGBL->GBL_DPPL4_PARAM[0x03] = 0x62; // 0x5d48
        // VCO frequency, CP current
        pGBL->GBL_DPPL4_PARAM[0x04] = DPLL4_800MHZ | DPLL4_CP_5uA; // 0x5d49
        pGBL->GBL_DPPL4_PARAM[0x00] = 0x00; // 0x5d45
        // SS-EN, Frange Setting
        pGBL->GBL_DPPL4_PARAM[0x01] = m_PllSettings[MMPF_PLL_ID_4].K; // 0x5d46
        // SS source select
        pGBL->GBL_DPPL4_PARAM[0x02] = DPLL4_SS_SRCSEL_0d31uA; // 0x5d47
        MMPF_PLL_WaitCount(50);
        // charge pump OP mode
        pGBL->GBL_DPPL4_PARAM[0x04] |= DPLL3_CP_OPMODE; // 0x5d49
        //Update DPLL4 configuration
        pGBL->GBL_DPLL4_CFG |= DPLL_UPDATE_PARAM; // 0x5d40
        MMPF_PLL_PowerUp(MMPF_PLL_ID_4, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_4, 1);

        // tv divide
        pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(0x07);
        break;
    case MMPF_PLL_ID_5:
        // 378MHZ
        // Start-UP @ Diode Mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x00; // 0x5d5a
        // PLL Settings
        pGBL->GBL_DPPL5_PARAM[0x04] = DPLL5_DIV2_LOOPDIV_EN; // 5d59
        pGBL->GBL_DPPL5_PARAM[0x03] = DPLL5_CHARGE_PUMP_5uA; // 5d58
        pGBL->GBL_DPPL5_PARAM[0x03] |= DPLL5_KVCO_OFFSET_5uA; // 5d58
        // Post Divider Divide 2
        pGBL->GBL_DPPL5_PARAM[0x02] = DPLL5_OUTPUT_DIV_2; // 5d57
        // Fractional Part = 0.6666
        pGBL->GBL_DPPL5_PARAM[0x02] |= 0x03; // 5d57
        pGBL->GBL_DPPL5_PARAM[0x01] = 0x00; // 5d56
        pGBL->GBL_DPPL5_PARAM[0x00] = 0x00; // 5d55
        // Loop Divider N part
        pGBL->GBL_DPLL5_N = 0x0F; // 0x5d54
        // Pre-Divider Pass
        pGBL->GBL_DPLL5_P = DPLL5_P_DIV_1; // 0x5d53
        MMPF_PLL_WaitCount(50);
        // change into  OP mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
        MMPF_PLL_WaitCount(50);
        //Update DPLL5 configuration
        pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50
        // PLL bug, must update twice.
        // change into  OP mode
        pGBL->GBL_DPPL5_PARAM[0x05] = 0x02; // 0x5d5a
        MMPF_PLL_WaitCount(50);
        //Update DPLL5 configuration
        pGBL->GBL_DPLL5_CFG |= DPLL_UPDATE_PARAM; // 0x5d50

        MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 0);
        MMPF_PLL_WaitCount(50);
        MMPF_PLL_PowerUp(MMPF_PLL_ID_5, 1);

        // tv divide
        pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(0x07);
        break;
    default:
        break;
    };
    pGBL->GBL_TV_CLK_SRC = PLLSrc;


    return err;
}
#endif // (CHIP == MCR_V2)

#endif //(OS_TYPE == OS_UCOSII)

/** @brief The function get the Group frequence.
 * @param[out] ulGroupFreq (GX frequency) (ex: 264MHz -> 264000)
 * @return It reports the status of the operation.
 */
//MMP_ULONG MMPF_PLL_GetGroup0Freq(MMP_ULONG MClk)
extern MMP_ERR MMPF_PLL_MCRV2_GetGroup0Freq(MMP_ULONG *ulGroupFreq);
MMP_ULONG MMPF_PLL_GetGroup0Freq()
{
    #if (CHIP == VSN_V3)

    AITPS_GBL pGBL = AITC_BASE_GBL;
    MMP_UBYTE m_pll_src_temp;
    MMP_UBYTE ubCfgDivid = 0x8;
    MMP_UBYTE ubPostDiv = 0x0;
    MMP_ULONG pll_out_clk;

    if(pGBL->GBL_CLK_SEL & GBL_CLK_SEL_MUX0) { //Use 0x6905 bit 6~7 as source selection
        m_pll_src_temp = ((pGBL->GBL_CLK_DIV >> PLL_SEL_PLL_OFFSET) & 0x03);
    }
    else {
        m_pll_src_temp = ((pGBL->GBL_CLK_SEL >> PLL_SEL_PLL_OFFSET1) & 0x03);
    }

    switch (m_pll_src_temp) {
    case 0:
        pll_out_clk = (MClk*pGBL->GBL_DPLL0_N)/(pGBL->GBL_DPLL0_K + 1);
        break;
    case 1:
        ubCfgDivid = (ubCfgDivid >> (pGBL->GBL_DPLL1_CFG01 & 0x07));
        pll_out_clk = (((MClk)/pGBL->GBL_DPLL1_M)*(pGBL->GBL_DPLL1_N+2))/ubCfgDivid;
        break;
    case 2:
        ubCfgDivid = (ubCfgDivid >> (pGBL->GBL_DPLL2_CFG2 & 0x07));
        pll_out_clk = (((MClk)/pGBL->GBL_DPLL2_M)*(pGBL->GBL_DPLL2_N+2))/ubCfgDivid;
        break;
    default:
        ubCfgDivid = (ubCfgDivid >> (pGBL->GBL_DPLL2_CFG2 & 0x07));
        pll_out_clk = (((MClk)/pGBL->GBL_DPLL2_M)*(pGBL->GBL_DPLL2_N+2))/ubCfgDivid;
        break;
    }

    ubPostDiv = (pGBL->GBL_CLK_DIV & 0x1f) + 1;
    ubPostDiv = (ubPostDiv << 1);

    return pll_out_clk/ubPostDiv;
    #endif

    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMP_ULONG freq_khz;

    //MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &freq_khz);
    MMPF_PLL_MCRV2_GetGroup0Freq(&freq_khz);
	
    return freq_khz * 1000;
    #endif
}



static MMP_ERR MMPF_PLL_ChangeAudioPLL_I2S(MMP_BOOL bEnable, MMPF_PLL_MODE ulSampleRate)
{
    AITPS_GBL pGBL = AITC_BASE_GBL;
	MMP_ERR   err = MMP_ERR_NONE;
	
	if(bEnable) {
	    if ((ulSampleRate != MMPF_PLL_AUDIO_44d1K) && 
			(ulSampleRate != MMPF_PLL_AUDIO_22d05K) &&
			(ulSampleRate != MMPF_PLL_AUDIO_11d025K) ) {
			// Post-Divider
			pGBL->GBL_DPLL3_M = (3);//0x13; // 0x5d33
			// Loop-Divider
			pGBL->GBL_DPLL3_N = (15); // 0x5d34 
			// Fractional Part
			pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
			pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
			pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37

			pGBL->GBL_DPLL3_CFG &=~ DPLL_SRC_MASK;
//			pGBL->GBL_DPLL3_CFG |= DPLL_SRC_PI2S_MCLK;

			//Update DPLL5 configuration
			pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30 
			//MMPF_PLL_PowerUp(MMPF_PLL_3, 0);
			MMPF_PLL_WaitCount(50);
			MMPF_PLL_PowerUp(MMPF_PLL_3, 1);
			MMPF_PLL_WaitCount(50);	

		} else {
				// Post-Divider
			pGBL->GBL_DPLL3_M = (3);//0x13; // 0x5d33
			// Loop-Divider
			pGBL->GBL_DPLL3_N = (15); // 0x5d34 
			// Fractional Part
			pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
			pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
			pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37

			pGBL->GBL_DPLL3_CFG &=~ DPLL_SRC_MASK;
//			pGBL->GBL_DPLL3_CFG |= DPLL_SRC_PI2S_MCLK;

			//Update DPLL5 configuration
			pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30 
			//MMPF_PLL_PowerUp(MMPF_PLL_3, 0);
			MMPF_PLL_WaitCount(50);
			MMPF_PLL_PowerUp(MMPF_PLL_3, 1);
			MMPF_PLL_WaitCount(50);
		}   
	}  
	return err;
}    

//------------------------------------------------------------------------------
//  Function    : MMPF_PLL_SetAudioPLL
//  Description :
//------------------------------------------------------------------------------
MMP_ERR MMPF_PLL_SetAudioPLL_I2S( int pllclockrate)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
	MMP_ERR   err = MMP_ERR_NONE;
	AIT_REG_B divM;
	
	pGBL->GBL_AUD_CLK_SRC = AUD_CLK_SRC_DPLL3;
	pGBL->GBL_DPLL3_CFG |= DPLL_BYPASS;
	pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM;


	// Post-Divider
	pGBL->GBL_DPLL3_M = (3);//0x13; // 0x5d33
	// Loop-Divider

	divM = pllclockrate/(24000000/4);
	
	pGBL->GBL_DPLL3_N = divM-1; // 0x5d34 
	// Fractional Part
	pGBL->GBL_DPPL3_PARAM[0x00] = 0x00; // 0x5d35
	pGBL->GBL_DPPL3_PARAM[0x01] = 0x00; // 0x5d36
	pGBL->GBL_DPPL3_PARAM[0x02] = 0x00; // 0x5d37

	pGBL->GBL_DPLL3_CFG &=~ DPLL_SRC_MASK;
//			pGBL->GBL_DPLL3_CFG |= DPLL_SRC_PI2S_MCLK;

	//Update DPLL5 configuration
	pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM; // 0x5d30 
	//MMPF_PLL_PowerUp(MMPF_PLL_3, 0);
	MMPF_PLL_WaitCount(50);
	MMPF_PLL_PowerUp(MMPF_PLL_3, 1);
	MMPF_PLL_WaitCount(50);	

#if 0	
	if ((ulSampleRate != MMPF_PLL_AUDIO_44d1K) && 
		(ulSampleRate != MMPF_PLL_AUDIO_22d05K) &&
		(ulSampleRate != MMPF_PLL_AUDIO_11d025K) ) { 
		MMPF_PLL_ChangeAudioPLL_I2S(1, ulSampleRate);     
		// 147.456Mhz  		         
//		pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(6); // 0x5da1
		pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(1); // 0x5da1
		//  24.576Mhz		

		m_bAudSampleGroup =0;
    	
	}
	else { 		
		MMPF_PLL_ChangeAudioPLL_I2S(1, ulSampleRate);
		// 22.5792      
		pGBL->GBL_AUD_CLK_DIV[0x00] = GRP_CLK_DIV_EN | GRP_CLK_DIV(1); // 0x5da1
		// 11.2896       
		m_bAudSampleGroup = 1;
	}
#endif	
	pGBL->GBL_DPLL3_CFG &= ~DPLL_BYPASS;    
	pGBL->GBL_DPLL3_CFG |= DPLL_UPDATE_PARAM;

	return err;
}
MODULE_LICENSE("GPL");

