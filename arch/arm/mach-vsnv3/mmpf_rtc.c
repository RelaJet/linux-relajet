//==============================================================================
//
//  File        : mmpf_rtc.c
//  Description : RTC Control Interface
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================
/**
 *  @file mmpf_rtc.c
 *  @brief The RTC control functions
 *  @author Ben Lu
 *  @version 1.0
 */

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
#include "includes_fw.h"
#include "lib_retina.h"

#include "mmp_reg_gbl.h"
//#include "mmp_reg_rtc.h"

#include "mmpf_rtc.h"

#if (ADX2002_EN)
#include "controlIC_adx2002.h"
#endif
#if (ADX2003_EN)
#include "controlIC_adx2003.h"
#endif

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================
#if (CHIP == MCR_V2)
#define RTC_BASE_YEAR       (2010)
#define RTC_BASE_MONTH      (1)
#define RTC_BASE_DAY        (1)
#define SEC_PER_DAY         (86400)
#endif

static const MMP_SHORT m_usRTCDaysOfMonAcc[12] = {
    31,  59,  90,  120,  151,  181,  212,  243,  273,  304, 334, 365
};
static const MMP_SHORT m_usRTCDaysOfLeapMonAcc[12] = {
    31,  60,  91,  121,  152,  182,  213,  244,  274,  305, 335, 366
};
static const MMP_BYTE m_usRTCMonOfstAcc[12] = {
    0,   0,   3,    3,    4,    4,    5,    5,    5,    6,   6,   7
};

//==============================================================================
//
//                              VARIABLES
//
//==============================================================================
#if (CHIP == MCR_V2)
static RTC_CallBackFunc *cbRtcCallBack = NULL;
MMPF_RTC_TIME_FORMAT m_stime_shadow={0};
#endif
MMP_ULONG m_ulRtcInSeconds = 0;

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================
#if (CHIP == MCR_V2)
//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_ISOCtl
//  Description :
//------------------------------------------------------------------------------
/** @brief The internal function to send ISO_EN control signal.

The function to enable ISO cell 
@return It reports the status of the operation.
*/
static void MMPF_RTC_ISOCtl(void)
{
    AITPS_RTC  pRTC = AITC_BASE_RTC;

    //Input ISO ctrl sequence , 1->2->3->4->5->2->1
	pRTC->RTC_SEQ = RTC_EN_SEQ_1;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_2;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_3;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_4;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_5;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_2;
	RTNA_WAIT_MS(10);
	pRTC->RTC_SEQ = RTC_EN_SEQ_1;
	RTNA_WAIT_MS(10);

    return;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_SetValid
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set current RTC valid

The function set the RTC status to valid
@return It reports the status of the operation.
*/
static MMP_ERR MMPF_RTC_SetValid(void)
{
#if (CHIP == MCR_V2)
	AITPS_RTC   pRTC = AITC_BASE_RTC;

    //pRTC->DIG2RTC_CTL  |= RTC_SET_VALID | RTC_SET_CNT_VALID;
    pRTC->DIG2RTC_CTL  |= RTC_BYPASS | RTC_EN;
    MMPF_RTC_ISOCtl();

    if (pRTC->RTC2DIG_CTL & RTC_CNT_VALID_ST)
        return MMP_ERR_NONE;

    return MMP_SYSTEM_ERR_HW;
#endif
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_IsLeapYear
//  Description :
//------------------------------------------------------------------------------
static MMP_BOOL MMPF_RTC_IsLeapYear(MMP_LONG year)
{
    return ((year & 0x3) == 0) && ((year % 100) || ((year % 400) == 0));
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_DateToDaysAbs
//  Description :
//------------------------------------------------------------------------------
static MMP_LONG MMPF_RTC_DateToDaysAbs(MMP_LONG year, MMP_LONG mon, MMP_LONG day)
{
    MMP_LONG  leap;
    MMP_LONG  d;

    leap = (MMPF_RTC_IsLeapYear(year) && (mon > 2)) ? (1) : (0);

    year -= 1;

	d = (year*365)+(year/4)-(year/100)+(year/400) +
        (((mon - 1) * 31) - m_usRTCMonOfstAcc[mon - 1] + leap) + day;

    return d;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_DateToDays
//  Description : Return how many days from the base date
//------------------------------------------------------------------------------
static MMP_LONG MMPF_RTC_DateToDays(MMP_LONG year, MMP_LONG mon, MMP_LONG day)
{
    MMP_LONG offset;

    offset = MMPF_RTC_DateToDaysAbs(year, mon, day) -
            MMPF_RTC_DateToDaysAbs(RTC_BASE_YEAR, RTC_BASE_MONTH, RTC_BASE_DAY);

    return offset;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_DaysToDate
//  Description :
//------------------------------------------------------------------------------
static MMP_BOOL MMPF_RTC_DaysToDate(MMP_LONG days, MMPF_RTC_TIME_FORMAT *pTime)
{
    // days from base date => date
    MMP_LONG y = (days * 400 / 146097) + RTC_BASE_YEAR;
    MMP_LONG t, m, d;
    const MMP_SHORT *p_day_of_mon;

    // Get the year
    while(1) {
        if (days < MMPF_RTC_DateToDays(y, 1, 1))
            y -= 1;
        else if (days > MMPF_RTC_DateToDays(y, 12, 31))
            y += 1;
        else
            break;
    }

    t = days - MMPF_RTC_DateToDays(y, 1, 1);

    if (MMPF_RTC_IsLeapYear(y))
        p_day_of_mon = m_usRTCDaysOfLeapMonAcc;
    else
        p_day_of_mon = m_usRTCDaysOfMonAcc;

    for(m = t/31; m < 12; m++)
    {
        if (t < p_day_of_mon[m])
            break;
    }

    d = t - ((m > 0) ? (p_day_of_mon[m-1]) : 0);

    pTime->usYear  = y;
    pTime->usMonth = m + 1;
    pTime->usDay   = d + 1;

    return MMP_TRUE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_DateToSeconds
//  Description :
//------------------------------------------------------------------------------
static MMP_ULONG MMPF_RTC_DateToSeconds(MMPF_RTC_TIME_FORMAT *pTime)
{
    MMP_ULONG days;
    MMP_ULONG sec;
    MMP_ULONG hour;

    hour = (pTime->b_12FormatEn && pTime->ubAmOrPm) ?
                pTime->usHour + 12 : pTime->usHour;

    days    = MMPF_RTC_DateToDays(pTime->usYear, pTime->usMonth, pTime->usDay);
    sec     = (days * SEC_PER_DAY) + (hour * 3600) +
              (pTime->usMinute * 60) + pTime->usSecond;

    return sec;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_DateToSeconds
//  Description :
//------------------------------------------------------------------------------
MMP_BOOL MMPF_RTC_SecondsToDate(MMP_ULONG ulSeconds, MMPF_RTC_TIME_FORMAT *pTime)
{
    MMP_ULONG days;
    MMP_ULONG sec, hour, min;

    sec         = ulSeconds;
    days        = sec / SEC_PER_DAY;

    MMPF_RTC_DaysToDate(days, pTime);

    sec        -= days * SEC_PER_DAY;   // sec  of the day
    min         = sec  / 60;            // min  of the day
    hour        = min  / 60;            // hour of the day
    sec        -= min  * 60;            // sec  of the hour
    min        -= hour * 60;            // min  of the hour

    pTime->usHour       = hour;
    pTime->usMinute     = min;
    pTime->usSecond     = sec;
    pTime->ubAmOrPm     = (hour >= 12) ? 1 : 0;
    pTime->b_12FormatEn = MMP_FALSE;

    return MMP_TRUE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_ValidateTime
//  Description :
//------------------------------------------------------------------------------
/** @brief The function validate the value of RTC time

The function validate the RTC time
@return It reports if the RTC time is valide.
*/
static MMP_BOOL MMPF_RTC_ValidateTime(MMPF_RTC_TIME_FORMAT *pTime)
{
    MMP_LONG    y, m, d;
    const MMP_SHORT *p_day_of_mon;

    y = pTime->usYear;
    m = pTime->usMonth;
    d = pTime->usDay;

    //Check date correction
    if (y < RTC_BASE_YEAR) {
        return MMP_FALSE;
    }
    else if (y == RTC_BASE_YEAR) {
        if (m < RTC_BASE_MONTH)
            return MMP_FALSE;
        else if ((m == RTC_BASE_MONTH) && (d < RTC_BASE_DAY))
            return MMP_FALSE;
    }
    if ((m > 12) || (m < 1) || (d < 0))
        return MMP_FALSE;

    if (MMPF_RTC_IsLeapYear(y))
        p_day_of_mon = m_usRTCDaysOfLeapMonAcc;
    else
        p_day_of_mon = m_usRTCDaysOfMonAcc;

    if (m == 1) {
        if (d > 31)
            return MMP_FALSE;
    }
    else if ((d + p_day_of_mon[m - 2]) > p_day_of_mon[m - 1]) {
        return MMP_FALSE;
    }

    //Check time correction
    if (pTime->b_12FormatEn && (pTime->usHour > 12))
        return MMP_FALSE;
    else if (!pTime->b_12FormatEn && (pTime->usHour > 23))
        return MMP_FALSE;
    else if ((pTime->usMinute > 59) || (pTime->usSecond > 59))
        return MMP_FALSE;

    return MMP_TRUE;
}
#endif //(CHIP == MCR_V2)

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_Initialize
//  Description :
//------------------------------------------------------------------------------
/** @brief The function Initialize the RTC.

The function Initialize the RTC 
@return It reports the status of the operation.
*/
MMP_ERR MMPF_RTC_Initialize(void)
{

    AITPS_GBL       pGBL = AITC_BASE_GBL;
	//AITPS_AIC       pAIC = AITC_BASE_AIC;
	static MMP_BOOL m_bAicEnable = MMP_FALSE;

    if (!m_bAicEnable) {
        m_bAicEnable = MMP_TRUE;

        //RTNA_AIC_Open(pAIC, AIC_SRC_GBL, rtc_isr_a,
        //          AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
        //RTNA_AIC_IRQ_En(pAIC, AIC_SRC_GBL);

        pGBL->GBL_WAKEUP_INT_CPU_SR  = RTC2DIG_INT_MASK;
        pGBL->GBL_WAKEUP_INT_CPU_EN &= ~(RTC2DIG_INT_MASK);
    }
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_IsValid
//  Description :
//------------------------------------------------------------------------------
/** @brief The function check if current RTC is valid

The function verify the RTC status
@return It reports the status of the operation.
*/
MMP_BOOL MMPF_RTC_IsValid(void)
{
	AITPS_RTC   pRTC = AITC_BASE_RTC;

    return (pRTC->RTC2DIG_CTL & RTC_VALID_ST) ? MMP_TRUE : MMP_FALSE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_SetTime
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used for setting time information to RTC.

This function is used for setting time information to RTC.
@param[in] pointer of structure MMPF_RTC_TIME_FORMAT.
@return It reports the status of the operation.
*/
MMP_ERR	MMPF_RTC_SetTime(MMPF_RTC_TIME_FORMAT *pTime)
{

    AITPS_RTC   pRTC = AITC_BASE_RTC;
    MMP_ULONG   base_time;

    if (MMPF_RTC_ValidateTime(pTime) == MMP_FALSE) {
        RTNA_DBG_Str(0, "Set RTC bad time\r\n");
        return MMP_SYSTEM_ERR_FORMAT;
    }

    base_time = MMPF_RTC_DateToSeconds(pTime);

    //Set RTC Base from user
    pRTC->DIG2RTC_CTL  |= RTC_BASE_WR;
    pRTC->RTC_WR_DATA   = base_time;

    //Reset RTC Counter
    pRTC->DIG2RTC_CTL  |= RTC_CNT_RST;

    MMPF_RTC_ISOCtl();

    pRTC->DIG2RTC_CTL  &= ~(RTC_BASE_WR|RTC_CNT_RST);

    //Retry if set RTC failed
    if (MMPF_RTC_SetValid())
        MMPF_RTC_SetTime(pTime);

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_SetTimeInSec
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used for setting time information to RTC.

This function is used for setting time information to RTC.
@param[in] pointer of structure MMPF_RTC_TIME_FORMAT.
@return It reports the status of the operation.
*/
MMP_ERR	MMPF_RTC_SetTimeInSec(MMP_ULONG64 secs)
{

    AITPS_RTC   pRTC = AITC_BASE_RTC;
    MMP_ULONG   base_time;

    base_time = (MMP_ULONG) secs;

    //Set RTC Base from user
    pRTC->DIG2RTC_CTL  |= RTC_BASE_WR;
    pRTC->RTC_WR_DATA   = base_time;

    //Reset RTC Counter
    pRTC->DIG2RTC_CTL  |= RTC_CNT_RST;

    MMPF_RTC_ISOCtl();

    pRTC->DIG2RTC_CTL  &= ~(RTC_BASE_WR|RTC_CNT_RST);

    //Retry if set RTC failed
    if (MMPF_RTC_SetValid())
	return MMP_SYSTEM_ERR_HW;
    else
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_GetTime
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used for getting RTC information.

This function is used for getting RTC time information.
@param[in] pointer of structure MMPF_RTC_TIME_FORMAT.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_RTC_GetTime(MMPF_RTC_TIME_FORMAT *pTime)
{

    AITPS_RTC  pRTC = AITC_BASE_RTC;
    MMP_ULONG  base_sec, run_sec;
    MMP_ULONG64 ullSeconds;

    //Read RTC Base
    pRTC->DIG2RTC_CTL  |= RTC_BASE_RD;
    MMPF_RTC_ISOCtl();
    base_sec = pRTC->RTC_RD_DATA;
    pRTC->DIG2RTC_CTL  &= ~(RTC_BASE_RD);

    //Read RTC Counter
    do {
        pRTC->DIG2RTC_CTL  |= RTC_CNT_RD;
        MMPF_RTC_ISOCtl();
        run_sec = pRTC->RTC_RD_DATA;
        pRTC->DIG2RTC_CTL  &= ~(RTC_CNT_RD);
    } while(pRTC->RTC_RD_DATA & RTC_CNT_BUSY);

    ullSeconds = base_sec + run_sec;
    if (ullSeconds > 0xFFFFFFFF)
        ullSeconds = 0xFFFFFFFF;

    m_ulRtcInSeconds = (MMP_ULONG)ullSeconds;

    MMPF_RTC_SecondsToDate((MMP_ULONG)ullSeconds, pTime);

    memcpy((void *)&m_stime_shadow, (void *)pTime, sizeof(MMPF_RTC_TIME_FORMAT));

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_GetTime_InSeconds
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used for getting RTC time in seconds "right after" MMPF_RTC_GetTime

This function is used for getting RTC time time in seconds "right after" MMPF_RTC_GetTime.
@return It reports the RTC time in seconds
*/
MMP_ULONG64 MMPF_RTC_GetTime_InSeconds(void)
{
    //return m_ulRtcInSeconds;
    AITPS_RTC  pRTC = AITC_BASE_RTC;
    MMP_ULONG  base_sec, run_sec;
    MMP_ULONG64 ullSeconds;

    //Read RTC Base
    pRTC->DIG2RTC_CTL  |= RTC_BASE_RD;
    MMPF_RTC_ISOCtl();
    base_sec = pRTC->RTC_RD_DATA;
    pRTC->DIG2RTC_CTL  &= ~(RTC_BASE_RD);

    //Read RTC Counter
    do {
        pRTC->DIG2RTC_CTL  |= RTC_CNT_RD;
        MMPF_RTC_ISOCtl();
        run_sec = pRTC->RTC_RD_DATA;
        pRTC->DIG2RTC_CTL  &= ~(RTC_CNT_RD);
    } while(pRTC->RTC_RD_DATA & RTC_CNT_BUSY);

    ullSeconds = base_sec + run_sec;
    return ullSeconds;
    //if (ullSeconds > 0xFFFFFFFF)
    //    ullSeconds = 0xFFFFFFFF;

    //return (MMP_ULONG)ullSeconds; 
}

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_ISR
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is the interrupt service routine of RTC alarm.

@param[in] None.
@return None.
*/
void MMPF_RTC_ISR(void)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    AITPS_RTC   pRTC = AITC_BASE_RTC;
    MMP_UBYTE   status;

    status = pGBL->GBL_WAKEUP_INT_CPU_EN & pGBL->GBL_WAKEUP_INT_CPU_SR;
    pGBL->GBL_WAKEUP_INT_CPU_SR = status;

    if (status & RTC2DIG_INT) {
        //Disable Alarm Interrupt
        pGBL->GBL_WAKEUP_INT_CPU_EN &= ~(RTC2DIG_INT);
        //Clear Alarm Interrupt 
        pRTC->DIG2RTC_CTL |= RTC_ALARM_INT_CLR;
        //The ISO sequence taks too long time to execute in ISR,
        //UI flow should handle it by call MMPF_RTC_SetAlarmEnable
        //to clear alarm interrupt.
        //MMPF_RTC_ISOCtl();
        //pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_INT_CLR);

        if (cbRtcCallBack) {
            (*cbRtcCallBack)();
        }
    }
}

/*
 * The time is counted by CHIP's timer, it read from HW RTC at system Start up.
 */
// TODO:
// It is declare at AHC_General.h,
// but it is improper to include that file at this layer!!
#if defined(ALL_FW)
MMP_ERR MMPF_RTC_GetTime_Shadow(MMPF_RTC_TIME_FORMAT *ptr)
{
    memcpy((void *)ptr, (void *)&m_stime_shadow, sizeof(MMPF_RTC_TIME_FORMAT));
	return MMP_ERR_NONE;
}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_RTC_SetAlarmEnable
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used for setting RTC alarm.

This function is used for setting RTC alarm.
@param[in] bEnable : enalbe/disable RTC
@param[in] alarm_time_info : pointer of structure MMPF_RTC_TIME_FORMAT.
@param[in] p_callback : call back function when alarm time expired.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_RTC_SetAlarmEnable(MMP_BOOL bEnable, MMPF_RTC_TIME_FORMAT *pAlarmTime, void *p_callback)
{
    //AITPS_GBL   pGBL = AITC_BASE_GBL;
    AITPS_RTC   pRTC = AITC_BASE_RTC;
    MMP_ULONG   cur_sec, alarm_sec, alarm_sec_ofst;
    MMPF_RTC_TIME_FORMAT curTime;

    if (bEnable) {
        if (MMPF_RTC_ValidateTime(pAlarmTime) == MMP_FALSE)
            goto L_BadAlarmTime;

        MMPF_RTC_GetTime(&curTime);

        //Make sure alarm time is later than current time
        cur_sec     = MMPF_RTC_DateToSeconds(&curTime);
        alarm_sec   = MMPF_RTC_DateToSeconds(pAlarmTime);

        if (alarm_sec <= cur_sec)
            goto L_BadAlarmTime;

        cbRtcCallBack = (RTC_CallBackFunc *)p_callback;
        alarm_sec_ofst = alarm_sec - cur_sec;

        //Disable Alarm first
        pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_INT_EN);
        pRTC->DIG2RTC_CTL |= RTC_ALARM_INT_CLR;
        MMPF_RTC_ISOCtl();
        pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_INT_CLR);

        //Read RTC Counter
        pRTC->DIG2RTC_CTL |= RTC_CNT_RD;
        MMPF_RTC_ISOCtl();
        cur_sec = pRTC->RTC_RD_DATA;
        pRTC->DIG2RTC_CTL &= ~(RTC_CNT_RD);

        //Set Alarm Counter & Enable Alarm
        pRTC->DIG2RTC_CTL |= RTC_ALARM_WR | RTC_ALARM_INT_EN;
        pRTC->RTC_WR_DATA = cur_sec + alarm_sec_ofst;
        MMPF_RTC_ISOCtl();
        pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_WR);

        if (p_callback) {
            // Enable interrupt only if callback registered, MCR_V2 MP only
            //pGBL->GBL_WAKEUP_INT_CPU_SR  = RTC2DIG_INT;
            //pGBL->GBL_WAKEUP_INT_CPU_EN |= RTC2DIG_INT;
        }
    }
    else {
        //pGBL->GBL_WAKEUP_INT_CPU_EN &= ~(RTC2DIG_INT);
        //pGBL->GBL_WAKEUP_INT_CPU_SR  = RTC2DIG_INT;

        cbRtcCallBack = NULL;

        //Disable Alarm
        pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_INT_EN);
        pRTC->DIG2RTC_CTL |= RTC_ALARM_INT_CLR;
        MMPF_RTC_ISOCtl();
        pRTC->DIG2RTC_CTL &= ~(RTC_ALARM_INT_CLR);
    }

    return MMP_ERR_NONE;

L_BadAlarmTime:
    RTNA_DBG_Str(0, "Set RTC bad alarm time\r\n");
    return MMP_SYSTEM_ERR_FORMAT;

	return MMP_ERR_NONE;
}
