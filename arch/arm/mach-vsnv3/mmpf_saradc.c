//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
#include "includes_fw.h"
#include "config_fw.h"
#include "lib_retina.h"
#include "mmp_register.h"
#include "mmp_reg_gbl.h"
#include "mmp_reg_spi.h"
#include "mmp_reg_saradc.h"
#include "mmpf_spi.h"
#include "mmpf_system.h"
#include "mmpf_saradc.h"

//==============================================================================
//
//                              GLOBAL VARIABLE
//
//==============================================================================


#define ALIGN_ADDR(x)  (x & 0xFE)

MMP_UBYTE   saradccmd[4];
MMP_BYTE    DEVIC_ID;
MMPF_SPI_ID m_usSpiID;

MMPF_SPI_ATTRIBUTE spiattribute;
MMPF_SPI_OPERATION saradc_spi_op;

MMP_UBYTE TP_ADC_SRC_SEL     ;
MMP_UBYTE TP_MEAS_MODE       ;
MMP_UBYTE TP_PEN_DOWN        ;
MMP_UBYTE TP_CLK_PD_CFG      ;
MMP_UBYTE TP_RD_REQ_STATUS;
MMP_UBYTE TP_POLLING_STATUS;
MMP_UBYTE TP_AUX_SUB_SEL;

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_InitDeviceID
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_InitDeviceID()
{
    AITPS_SARADC_POR pSARADC   =   AITC_BASE_SARADC_POR; 
      
    pSARADC->SARADC_CTL        =   PSPI2_MISO_FROM_SARADC
                                   |SARADC_MACRO_ADDR_SET_VAL<<SARADC_SET_MACRO_BIT_SHIFT
                                   |SARADC_SPI_EN ;
    pSARADC->SARADC_CTL        |=   0x02;
    
    DEVIC_ID =  0x20 |SARADC_MACRO_ADDR_SET_VAL<<SARADC_SET_MACRO_BIT_SHIFT
                |WRITE_OP ;
    return MMP_ERR_NONE;}


//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_Init
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_Init(MMPF_SARADC_ATTRIBUTE *saradc_attribute)
{   
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    MMPF_SPI_Initialize();   
    m_usSpiID                  =   2; 
    spiattribute.spiMode       =   MMPF_SPI_MASTER_MODE;
    spiattribute.ubWordLength  =   32;
    spiattribute.ubSclkDiv     =   14; /* MP: G0_Fast/(2*(usSclkDiv+1)) G0_Fast = 264 MHz */
    spiattribute.ubPspiDelay   =   0;
    spiattribute.ubPspiWait    =   0;
    spiattribute.padCtl        =   MMPF_SPI_PAD_MAX;
    MMPF_SPI_SetAttributes(m_usSpiID, &spiattribute);
    MMPF_SARADC_InitDeviceID();

    pGBL->GBL_SPI_PAD_CFG &= ~(GBL_PSPI2_PAD_MASK);          
    MMPF_SARADC_SetMode(POLLING_MODE);
    MMPF_SARADC_SetCR(SARADC_DEFAULT_CR);  //fix to 48kHz        
    MMPF_SARADC_EnableTPWait(saradc_attribute->TPWait);        
    MMPF_SARADC_WriteByte(SARADC_TP_CLK_PD_CFG,0xB1);
    MMPF_SARADC_WriteByte(SARADC_TP_PEN_DOWN,0x8C);
    #if (OS_TYPE == OS_UCOSII) // very early called in linux.
    MMPF_OS_Sleep(6);
    #endif
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_GetData
//  Description : don't call this function from ISR, os_sleep func will not work
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_GetData(MMP_USHORT *value)
{  
    MMP_USHORT data;
    MMP_UBYTE   temp;
    MMP_SHORT  grp;
    MMP_SHORT  fix_val;

    MMPF_SARADC_WriteByte(SARADC_TP_POLLING_STATUS,TP_POLL_GET_RESULT);
    
    MMPF_OS_Sleep(2);

    do {
        MMPF_SARADC_ReadByte(SARADC_TP_POLLING_STATUS,&temp);
    } while((temp & TP_MEASURE_RESULT_STATUS));

    MMPF_SARADC_ReadShort(SARADC_TP_RD_DATA0,&data);

    if (HIGH_BYTE(data) & TP_ADC_DATA_VALID) {
    
       if(HIGH_BYTE(data) & TP_ADC_SRC_ID_AUX1){
        grp=1;
       }else if(HIGH_BYTE(data) & TP_ADC_SRC_ID_AUX2){
        grp=2;
       }else if(HIGH_BYTE(data) & TP_ADC_SRC_ID_AUX3){
        grp=3;
       }
       /*
       RTNA_DBG_Str3("AUX GRP=");
       RTNA_DBG_Short3(grp);
       RTNA_DBG_Str3("\r\n");
       */
       /*compensate output value*/
       *value = data & 0x3FF;      
        if(*value <=504) {
            fix_val=(*value)*7/504;
        } else {
        //RTNA_DBG_Short(0,(1023-(*value))*7/518);
            fix_val=1023-(*value);
            fix_val=((fix_val*7/518)<<1)+1;
            fix_val>>=1;
        }
       *value+=fix_val;
       /*
       RTNA_DBG_Short3(*value);
       RTNA_DBG_Str3("\r\n");
       */
       //dbg_printf(3,"AUX GRP=%d,Val:%d\r\n",grp,*value);
    }
    else {
        RTNA_DBG_Str(0, "SARADC data invalid \r\n");
    }

    return MMP_ERR_NONE;
}


MMP_ERR MMPF_SARADC_PowerDown(void)
{
    AITPS_SARADC_POR pSARADC   =   AITC_BASE_SARADC_POR;   
    MMP_UBYTE   temp;
    MMPF_SARADC_ATTRIBUTE saradc_attribute;
    
    saradc_attribute.TPWait = MMP_FALSE;
    MMPF_SARADC_Init(&saradc_attribute);
    
    MMPF_SARADC_ReadByte(SARADC_TP_PEN_DOWN,&temp);
    temp &= ~TP_PEN_ADC_AUX_ADX_EN_MASK;
    MMPF_SARADC_WriteByte(SARADC_TP_PEN_DOWN,temp);
    
    MMPF_SARADC_ReadByte(SARADC_TP_CLK_PD_CFG,&temp);
    temp &= ~SARADC_ENABLE_PEN_ADC_POWER;
    MMPF_SARADC_WriteByte(SARADC_TP_CLK_PD_CFG,temp);
    pSARADC->SARADC_CTL        =   0;
    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_WriteShort
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_WriteShort(MMP_UBYTE addr_aligned, MMP_USHORT write_data)
{   
    MMP_ULONG       txaddr;
    txaddr    = (MMP_ULONG)saradccmd;
    
    if(ALIGN_ADDR(addr_aligned) != addr_aligned){
        RTNA_DBG_Str(0, "SARADC word address not alignment \r\n");    
    }
    DEVIC_ID            &=  ~0x03;
    DEVIC_ID            |=  WRITE_OP;  
    saradccmd[3]        =   DEVIC_ID;
    saradccmd[2]        =   addr_aligned;
    saradccmd[1]        =   HIGH_BYTE(write_data);
    saradccmd[0]        =   LOW_BYTE(write_data);

    MMPF_SPI_Write(m_usSpiID, txaddr, 4);

    return MMP_ERR_NONE;
}

MMP_ERR MMPF_SARADC_WriteByte(MMP_UBYTE addr_aligned, MMP_UBYTE write_data)
{   
    MMP_ULONG       txaddr;
    MMP_USHORT      rvale;
    txaddr    = (MMP_ULONG)saradccmd;
      
    MMPF_SARADC_ReadShort(ALIGN_ADDR(addr_aligned), &rvale);
    if( (addr_aligned & 0xFE) == addr_aligned ){
        rvale &=  0xff00;
        rvale |=  write_data;
    }else{                                                                                      
        rvale &=  0x00ff;
        rvale |=  write_data<<8;
    }
    
    DEVIC_ID            &=  ~0x03;
    DEVIC_ID            |=  WRITE_OP;  
    saradccmd[3]        =   DEVIC_ID;
    saradccmd[2]        =   addr_aligned;
    saradccmd[1]        =   HIGH_BYTE(rvale);
    saradccmd[0]        =   LOW_BYTE(rvale);
    
    MMPF_SPI_Write(m_usSpiID, txaddr, 4);

    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_ReadShort
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_ReadByte(MMP_UBYTE addr_aligned, MMP_UBYTE *readata)
{
    MMP_UBYTE       rxbuf[4];
    MMP_ULONG       txaddr,rxaddr;
    
    txaddr          =   (MMP_ULONG)saradccmd;
    rxaddr          =   (MMP_ULONG)rxbuf;
     
    DEVIC_ID        &=  ~0x03;
    DEVIC_ID        |=  READ_OP; 
    
    saradccmd[3]    =   DEVIC_ID;
    saradccmd[2]    =   ALIGN_ADDR(addr_aligned);
    saradccmd[1]    =   0xFF;       //dummy byte
    saradccmd[0]    =   0xFF;       //dummy byte
        
    MMPF_SPI_Read(m_usSpiID, rxaddr, 4,txaddr, 4);
    if(ALIGN_ADDR(addr_aligned) != addr_aligned){
        *readata=rxbuf[1];
    }else{
        *readata=rxbuf[0];
    }
  
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_ReadShort
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_ReadShort(MMP_UBYTE addr_aligned, MMP_USHORT *readata)
{
    MMP_UBYTE       rxbuf[4];
    MMP_ULONG       txaddr,rxaddr;
    
    txaddr          =   (MMP_ULONG)saradccmd;
    rxaddr          =   (MMP_ULONG)rxbuf;
    
    if(ALIGN_ADDR(addr_aligned) != addr_aligned){
        RTNA_DBG_Str(0, "SARADC word address not alignment \r\n");
    }
    DEVIC_ID        &=  ~0x03;
    DEVIC_ID        |=  READ_OP; 
    
    saradccmd[3]    =   DEVIC_ID;
    saradccmd[2]    =   addr_aligned;
    saradccmd[1]    =   0xFF;       //dummy byte
    saradccmd[0]    =   0xFF;       //dummy byte
    //MMPF_SPI_WriteData(m_usSpiID, txaddr, 4);
    MMPF_SPI_Read(m_usSpiID, rxaddr, 4,txaddr, 4);
    *readata = rxbuf[0] | rxbuf[1]<<8;
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetDelClk
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetDelClk(MMPF_SARADC_DELAY_CLOCK DEL)
{
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    MMP_BYTE    temp;
    
    MMPF_SARADC_ReadShort(ALIGN_ADDR(SARADC_TP_MEASURE_MODE), &rdata);

    if(SARADC_TP_MEASURE_MODE == ALIGN_ADDR(SARADC_TP_MEASURE_MODE)){
        temp  =     HIGH_BYTE(rdata);
        temp &=     ~SARADC_ADC_SET_TIME_MASK;
        temp |=     DEL;  
        wdata =     temp<<8 | LOW_BYTE(rdata);
    }else{
        temp  =     LOW_BYTE(rdata);
        temp &=     ~SARADC_ADC_SET_TIME_MASK;
        temp |=     DEL;
        wdata =   HIGH_BYTE(rdata)<<8 | temp;
    }
    MMPF_SARADC_WriteShort(ALIGN_ADDR(SARADC_TP_MEASURE_MODE), wdata);
 
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetCR
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetCR(MMPF_SARADC_MEA_RATE CR)
{
    
    MMP_UBYTE    temp;
       
    MMPF_SARADC_ReadByte(SARADC_TP_MEASURE_MODE,&temp);
    temp &= ~SARADC_CONTI_MODE_RATE_MASK;
    temp |= CR<<2;
    MMPF_SARADC_WriteByte(SARADC_TP_MEASURE_MODE, temp);
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_EnableTPWait
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_EnableTPWait(MMP_BOOL TPWaitenable)
{
    MMP_UBYTE  rdata;

    MMPF_SARADC_ReadByte(SARADC_TP_MEASURE_MODE, &rdata);   
    if(TPWaitenable){
        rdata |= SARADC_TP_WAIT_EN;
    }else{
        rdata &= ~SARADC_TP_WAIT_EN;
    }       
    MMPF_SARADC_WriteByte(SARADC_TP_MEASURE_MODE, rdata);

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetMeasMode
//  Description : 
//------------------------------------------------------------------------------
/* DEL: The CENB will be inactive to capture the measure result after this delay clocks*/
MMP_ERR MMPF_SARADC_SetMeasMode(MMPF_SARADC_DELAY_CLOCK DEL, MMPF_SARADC_MEA_RATE CR, MMP_BOOL TPWaitenable)
{
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    
    TP_MEAS_MODE &= ~SARADC_ADC_SET_TIME_MASK;
    TP_MEAS_MODE |= DEL;
    
    TP_MEAS_MODE &= SARADC_CONTI_MODE_RATE_MASK;
    TP_MEAS_MODE |= CR<<2;
    
    if(TPWaitenable){
        TP_MEAS_MODE |= SARADC_TP_WAIT_EN;
    }else{
        TP_MEAS_MODE &= ~SARADC_TP_WAIT_EN;
    }
    
    MMPF_SARADC_ReadShort(SARADC_TP_MEASURE_MODE & 0xFE, &rdata);
    if(SARADC_TP_MEASURE_MODE != (SARADC_TP_MEASURE_MODE & 0xFE)){
        wdata =   TP_MEAS_MODE<<8 | LOW_BYTE(rdata);
    }else{
        wdata =   HIGH_BYTE(rdata)<<8 | TP_MEAS_MODE;
    }
    MMPF_SARADC_WriteShort(SARADC_TP_MEASURE_MODE & 0xFE, wdata);
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetChannel
//  Description :  Always one channel be measured at one time. Set another AUX grp channel will mask 
//                 other channel setting, in other word, only support polling mode
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetChannel(MMP_SHORT channel)
{
    MMP_SHORT   val = 0;
      
    switch(channel){
    case 1: 
    case 2:  
    case 3:
        val=1;
        break;
    case 4:
    case 6:
    case 8:
        val=2;
        break;
    case 5:
    case 7:
        val=3;
        break;
    }
    
    MMPF_SARADC_ReadByte(SARADC_TP_AUX_SUB_SEL, &TP_AUX_SUB_SEL);
    MMPF_SARADC_ReadByte(SARADC_TP_ADC_SRC_SEL, &TP_ADC_SRC_SEL);
    TP_ADC_SRC_SEL &= ~0x70;
    TP_AUX_SUB_SEL &= ~(TP_AUX1_SUB_SEL_MASK| TP_AUX2_SUB_SEL_MASK | TP_AUX3_SUB_SEL_MASK);
    switch(channel){
    case 1: 
    case 4:  
    case 5:
        TP_AUX_SUB_SEL  &= ~TP_AUX1_SUB_SEL_MASK;  
        TP_AUX_SUB_SEL  |=  val; 
        TP_ADC_SRC_SEL  |=  SARADC_TP_AUX1_EN;
        break;
    case 2:
    case 6:
    case 7:
        TP_AUX_SUB_SEL  &= ~TP_AUX2_SUB_SEL_MASK;
        TP_AUX_SUB_SEL  |=  val<<2;; 
        TP_ADC_SRC_SEL  |=  SARADC_TP_AUX2_EN;
        break; 
    case 3:
    case 8:
        TP_AUX_SUB_SEL  &= ~TP_AUX3_SUB_SEL_MASK;
        TP_AUX_SUB_SEL  |=  val<<4;;         
        TP_ADC_SRC_SEL  |=  SARADC_TP_AUX3_EN;
        break;    
    }
        
    MMPF_SARADC_WriteByte(SARADC_TP_AUX_SUB_SEL, TP_AUX_SUB_SEL);  
    MMPF_SARADC_WriteByte(SARADC_TP_ADC_SRC_SEL, TP_ADC_SRC_SEL);
    
    //dbg_printf(3,"AUX : %d select\r\n",channel);
    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetMode
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetMode(MMP_SHORT mode)
{
    MMPF_SARADC_ReadByte(SARADC_TP_ADC_SRC_SEL, &TP_ADC_SRC_SEL);
    
    TP_ADC_SRC_SEL &= ~0x80;
    
    if( mode ==  1){
        TP_ADC_SRC_SEL |=  SARADC_TP_POLLING_MODE;
    }else{        
        TP_ADC_SRC_SEL |=  SARADC_TP_CONTI_MODE;
    }
    MMPF_SARADC_WriteByte(SARADC_TP_ADC_SRC_SEL, TP_ADC_SRC_SEL);
             
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetFastRead
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetFastRead(void)
{
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    
    TP_PEN_DOWN &=  ~TP_FAST_READ_EN;
    TP_PEN_DOWN |=  TP_FAST_READ_EN;
    TP_PEN_DOWN |=  TP_PEN_ADC_AUX_ADX_EN_MASK;
    
    MMPF_SARADC_ReadShort(SARADC_TP_PEN_DOWN & 0xFE, &rdata);
    if(SARADC_TP_PEN_DOWN != (SARADC_TP_PEN_DOWN & 0xFE)){
        wdata =   TP_PEN_DOWN<<8 | LOW_BYTE(rdata);
    }else{
        wdata =   HIGH_BYTE(rdata)<<8 | TP_PEN_DOWN;
    }
    MMPF_SARADC_WriteShort(SARADC_TP_PEN_DOWN & 0xFE, wdata); 
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_HandShake_Read
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_HandShake_Read(void)
{
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    TP_RD_REQ_STATUS    |= TP_READ_REQ;
    
    MMPF_SARADC_ReadShort(SARADC_TP_RD_REQ_STATUS & 0xFE, &rdata);
    if(SARADC_TP_RD_REQ_STATUS != (SARADC_TP_RD_REQ_STATUS & 0xFE)){
        wdata =   TP_RD_REQ_STATUS<<8 | LOW_BYTE(rdata);
    }else{
        wdata =   HIGH_BYTE(rdata)<<8 | TP_RD_REQ_STATUS;
    }
    MMPF_SARADC_WriteShort(SARADC_TP_RD_REQ_STATUS & 0xFE, wdata);    
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_SetPoll
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_SetPoll(void)
{   
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    TP_POLLING_STATUS   |= TP_POLL_GET_RESULT;
    
    MMPF_SARADC_ReadShort(SARADC_TP_POLLING_STATUS & 0xFE, &rdata);
    if(SARADC_TP_POLLING_STATUS != (SARADC_TP_POLLING_STATUS & 0xFE)){
        wdata =   TP_POLLING_STATUS<<8 | LOW_BYTE(rdata);
    }else{
        wdata =   HIGH_BYTE(rdata)<<8 | TP_POLLING_STATUS;
    }
    MMPF_SARADC_WriteShort(SARADC_TP_POLLING_STATUS & 0xFE, wdata);        
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SARADC_StartADC
//  Description : 
//------------------------------------------------------------------------------
MMP_ERR MMPF_SARADC_StartADC(void)
{
    MMP_USHORT  rdata;
    MMP_USHORT  wdata;
    TP_CLK_PD_CFG       |= SARADC_ENABLE_PEN_ADC_POWER;
    
    MMPF_SARADC_ReadShort(SARADC_TP_CLK_PD_CFG & 0xFE, &rdata);
    if(SARADC_TP_CLK_PD_CFG != (SARADC_TP_CLK_PD_CFG & 0xFE)){
        wdata =   TP_CLK_PD_CFG<<8 | LOW_BYTE(rdata);
    }else{
        wdata =   HIGH_BYTE(rdata)<<8 | TP_CLK_PD_CFG;
    }
    MMPF_SARADC_WriteShort(SARADC_TP_CLK_PD_CFG & 0xFE, wdata);

    return MMP_ERR_NONE;
}

