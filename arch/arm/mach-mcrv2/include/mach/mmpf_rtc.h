#ifndef _MMPF_RTC_H_
#define _MMPF_RTC_H_
#include "os_wrap.h"

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================
typedef struct _MMPF_RTC_TIME_FORMAT
{
	MMP_USHORT usYear;		
	MMP_USHORT usMonth;
	MMP_USHORT usDay;		//The date of January is 1~31
	MMP_USHORT usDayInWeek;		//Sunday ~ Saturday
	MMP_USHORT usHour;   
	MMP_USHORT usMinute;
	MMP_USHORT usSecond;
	MMP_UBYTE ubAmOrPm;		//am: 0, pm: 1, work only at b_12FormatEn = MMP_TURE
	MMP_BOOL b_12FormatEn;  //for set time, to indacate which format, 0 for 24 Hours, 
							//1 for 12 Hours format
} MMPF_RTC_TIME_FORMAT;


#define TmDataTime MMPF_RTC_TIME_FORMAT
typedef void RTC_CallBackFunc(void);
//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR MMPF_RTC_Initialize(void);
MMP_BOOL MMPF_RTC_IsValid(void);
MMP_BOOL MMPF_RTC_SecondsToDate(MMP_ULONG ulSeconds, MMPF_RTC_TIME_FORMAT *pTime);
MMP_ERR	MMPF_RTC_SetTime(MMPF_RTC_TIME_FORMAT *ptr);
MMP_ERR MMPF_RTC_GetTime(MMPF_RTC_TIME_FORMAT *ptr);
//MMP_ULONG MMPF_RTC_GetTime_InSeconds( void );
MMP_ULONG64 MMPF_RTC_GetTime_InSeconds( void );
/*
 * The time is counted by CHIP's timer, it read from HW RTC at system Start up.
 */
MMP_ERR MMPF_RTC_GetTime_Shadow(MMPF_RTC_TIME_FORMAT *ptr);
MMP_ERR MMPF_RTC_SetAlarmEnable(MMP_BOOL bEnable, MMPF_RTC_TIME_FORMAT *alarm_time_info, void* p_callback);
MMP_ERR	MMPF_RTC_SetTimeInSec(MMP_ULONG64 secs);
#endif