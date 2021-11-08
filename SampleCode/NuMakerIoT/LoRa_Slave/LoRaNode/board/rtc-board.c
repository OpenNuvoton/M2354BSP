/****************************************************************************
 * @file     rtc-board.c
 * @version  V1.10
 * @brief    Target board RTC timer and low power modes management
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "NuMicro.h"
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "lpm-board.h"
#include "rtc-board.h"

// MCU Wake Up Time
#define MIN_ALARM_DELAY                             3 // in ticks

// sub-second number of bits
#define N_PREDIV_S                                  7
// Synchronous prediv
#define PREDIV_S                                    ( ( 1 << N_PREDIV_S ) - 1 )
// Asynchronous prediv
#define PREDIV_A                                    ( 1 << ( 15 - N_PREDIV_S ) ) - 1


// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                               3
#define CONV_NUMER                                  ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )


/*!
 * RTC timer context 
 */
typedef struct
{
	uint32_t Time;         // Reference time in second
	S_RTC_TIME_DATA_T CalendarDateTime;	//Referenct date time in calendar format
	uint32_t sub_second;
}RtcTimerContext_t;

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
static bool McuWakeUpTimeInitialized = false;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * \brief Compensates MCU wakeup time
 */
static int16_t McuWakeUpTimeCal = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/**
  * @brief      Update RTC Alarm Date and Time with sub-second
  *
  * @param[in]  psPt    Specify the time property and alarm date and time. It includes:             \n
  *                     u32Year: Year value, range between 2000 ~ 2099.                             \n
  *                     u32Month: Month value, range between 1 ~ 12.                                \n
  *                     u32Day: Day value, range between 1 ~ 31.                                    \n
  *                     u32DayOfWeek: Day of the week. [RTC_SUNDAY / RTC_MONDAY / RTC_TUESDAY /
  *                                                     RTC_WEDNESDAY / RTC_THURSDAY / RTC_FRIDAY /
  *                                                     RTC_SATURDAY]                               \n
  *                     u32Hour: Hour value, range between 0 ~ 23.                                  \n
  *                     u32Minute: Minute value, range between 0 ~ 59.                              \n
  *                     u32Second: Second value, range between 0 ~ 59.                              \n
  *                     u32TimeScale: [RTC_CLOCK_12 / RTC_CLOCK_24]                                 \n
  *                     u8AmPm: [RTC_AM / RTC_PM]                                                   \n
  *
  * @return     None
  *
  * @details    This API is used to update alarm date and time setting to RTC.
  */
static void RTC_SetAlarmDateAndTimeEx(S_RTC_TIME_DATA_T *psPt, uint32_t u32SubSecond)
{
    uint32_t u32RegCALM, u32RegTALM;

    if (psPt == NULL)
    {
        /* No RTC date/time data */
    }
    else
    {
        /*-----------------------------------------------------------------------------------------------------*/
        /* Set RTC 24/12 hour setting and Day of the Week                                                      */
        /*-----------------------------------------------------------------------------------------------------*/

        if (psPt->u32TimeScale == RTC_CLOCK_12)
        {
            RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

            /*-------------------------------------------------------------------------------------------------*/
            /* Important, range of 12-hour PM mode is 21 up to 32                                               */
            /*-------------------------------------------------------------------------------------------------*/
            if (psPt->u32AmPm == RTC_PM)
            {
                psPt->u32Hour += 20ul;
            }
        }
        else
        {
            RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
        }

        /*-----------------------------------------------------------------------------------------------------*/
        /* Set RTC Alarm Date and Time                                                                         */
        /*-----------------------------------------------------------------------------------------------------*/
        u32RegCALM  = ((psPt->u32Year - RTC_YEAR2000) / 10ul) << 20;
        u32RegCALM |= (((psPt->u32Year - RTC_YEAR2000) % 10ul) << 16);
        u32RegCALM |= ((psPt->u32Month  / 10ul) << 12);
        u32RegCALM |= ((psPt->u32Month  % 10ul) << 8);
        u32RegCALM |= ((psPt->u32Day    / 10ul) << 4);
        u32RegCALM |= (psPt->u32Day    % 10ul);

        u32RegTALM  = ((psPt->u32Hour   / 10ul) << 20);
        u32RegTALM |= ((psPt->u32Hour   % 10ul) << 16);
        u32RegTALM |= ((psPt->u32Minute / 10ul) << 12);
        u32RegTALM |= ((psPt->u32Minute % 10ul) << 8);
        u32RegTALM |= ((psPt->u32Second / 10ul) << 4);
        u32RegTALM |= (psPt->u32Second % 10ul);


        RTC->CALM = (uint32_t)u32RegCALM;
        RTC->TALM = (uint32_t)(u32RegTALM | (u32SubSecond << 24));
    }
}


static uint64_t RtcGetCalendarValue(S_RTC_TIME_DATA_T* psTimeDateTime, uint32_t *pu32Sub_second)
{
    uint64_t calendarValue = 0;
    uint32_t seconds;
    uint32_t sub_second;
	uint32_t correction;
	uint32_t firstRead;

	do
	{
		firstRead = RTC->TIME;
		sub_second = (RTC->TIME  >> 24) & 0x7F;
		RTC_GetDateAndTime(psTimeDateTime);
	}while(firstRead != RTC->TIME);
		
    // Calculte amount of elapsed days since 01/01/2000
    seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * psTimeDateTime->u32Year , 4 );
    correction = ( ( psTimeDateTime->u32Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;
    seconds += ( DIVC( ( psTimeDateTime->u32Month-1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( psTimeDateTime->u32Month - 1 ) * 2 ) ) & 0x03 ) ) );
    seconds += ( psTimeDateTime->u32Day -1 );

    // Convert from days to seconds
    seconds *= SECONDS_IN_1DAY;

    seconds += ( ( uint32_t )psTimeDateTime->u32Second + 
                 ( ( uint32_t )psTimeDateTime->u32Minute * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t )psTimeDateTime->u32Hour * SECONDS_IN_1HOUR ) ) ;

	calendarValue = ( ( ( uint64_t )seconds ) << N_PREDIV_S ) + sub_second;
	*pu32Sub_second = sub_second;
    return( calendarValue );
}


/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = ( uint32_t )RtcGetCalendarValue( &RtcTimerContext.CalendarDateTime, &RtcTimerContext.sub_second );
    return ( uint32_t )RtcTimerContext.Time;
}

void RtcInit( void )
{
	S_RTC_TIME_DATA_T sRTCTimeData;

	/* Enable RTC peripheral clock */
	CLK_EnableModuleClock(RTC_MODULE);
	
	if( RtcInitialized == false )
	{
		/* Time Setting */
		sRTCTimeData.u32Year       = RTC_YEAR2000;
		sRTCTimeData.u32Month      = 1;
		sRTCTimeData.u32Day        = 1;
		sRTCTimeData.u32Hour       = 0;
		sRTCTimeData.u32Minute     = 0;
		sRTCTimeData.u32Second     = 0;
		sRTCTimeData.u32DayOfWeek  = RTC_SATURDAY;
		sRTCTimeData.u32TimeScale  = RTC_CLOCK_24;

		RTC_Open(&sRTCTimeData);
		//Enable sub-second conter
		RTC->CLKFMT |= BIT8;
		//Enable RTC spare function
		RTC_EnableSpareAccess();

		NVIC_SetPriority(RTC_IRQn, 1);
		NVIC_EnableIRQ(RTC_IRQn);
		
		RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
		
		RtcSetTimerContext();
		RtcInitialized = true;
	}
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */

uint32_t RtcGetTimerContext( void )
{
	return RtcTimerContext.Time;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
	return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return ( uint32_t )( ( ( ( uint64_t )milliseconds ) * CONV_DENOM ) / CONV_NUMER );
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
    uint32_t seconds = tick >> N_PREDIV_S;

    tick = tick & PREDIV_S;
    return ( ( seconds * 1000 ) + ( ( tick * 1000 ) >> N_PREDIV_S ) );
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
    uint64_t delayTicks = 0;
    uint64_t refTicks = RtcGetTimerValue( );

    delayTicks = RtcMs2Tick( delay );
    // Wait delay ms
    while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
    {
        __NOP( );
    }
}

uint32_t RtcGetTimerValue( void )
{
	S_RTC_TIME_DATA_T sRTCDateTime;
	uint32_t u32SubSecond;
	
	uint32_t calendarValue = ( uint32_t )RtcGetCalendarValue( &sRTCDateTime, &u32SubSecond );
	return( calendarValue );
}

uint32_t RtcGetTimerElapsedTime( void )
{
	S_RTC_TIME_DATA_T sRTCDateTime;
	uint32_t u32SubSecond;

	uint32_t calendarValue = ( uint32_t )RtcGetCalendarValue( &sRTCDateTime, &u32SubSecond );

	return( ( uint32_t )( calendarValue - RtcTimerContext.Time ) );
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
	S_RTC_TIME_DATA_T sRTCDateTime;
	uint32_t ticks;
	uint32_t u32SubSecond;

	uint64_t calendarValue = RtcGetCalendarValue( &sRTCDateTime, &u32SubSecond );
	uint32_t seconds = ( uint32_t )( calendarValue >> N_PREDIV_S );

	ticks =  ( uint32_t )calendarValue & PREDIV_S;
	*milliseconds = RtcTick2Ms( ticks );
	return seconds;
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
	RTC->SPR[0] = data0;
	RTC->SPR[1] = data1;
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
  *data0 = RTC->SPR[0];
  *data1 = RTC->SPR[1];
}

void RtcProcess( void )
{
    // Not used on this platform.
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    float k = RTC_TEMP_COEFFICIENT;
    float kDev = RTC_TEMP_DEV_COEFFICIENT;
    float t = RTC_TEMP_TURNOVER;
	float tDev = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0f;
    float ppm = 0.0f;

    if( k < 0.0f )
    {
        ppm = ( k - kDev );
    }
    else
    {
        ppm = ( k + kDev );
    }

    interim = ( temperature - ( t - tDev ) );
    ppm *=  interim * interim;

    // Calculate the drift in time
    interim = ( ( float ) period * ppm ) / 1000000.0f;
    // Calculate the resulting time period
    interim += period;
    interim = floor( interim );

    if( interim < 0.0f )
    {
        interim = ( float )period;
    }

    // Calculate the resulting period
    return ( TimerTime_t ) interim;
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    // We don't go in Low Power mode for timeout below MIN_ALARM_DELAY
    if( ( int64_t )( MIN_ALARM_DELAY + McuWakeUpTimeCal ) < ( int64_t )( timeout - RtcGetTimerElapsedTime( ) ) )
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );
    }
    else
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_DISABLE );
    }

    // In case stop mode is required
    if( LpmGetMode( ) == LPM_STOP_MODE )
    {
        timeout = timeout - McuWakeUpTimeCal;
    }
    RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
    // Disable the Alarm A interrupt
	RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);

    // Clear RTC Alarm Flag
	RTC_CLEAR_ALARM_INT_FLAG(RTC);

    // Clear the EXTI's line Flag for RTC Alarm
}

void RtcStartAlarm( uint32_t timeout )
{
	uint16_t rtcAlarmSubSeconds = 0;
	uint16_t rtcAlarmSeconds = 0;
	uint16_t rtcAlarmMinutes = 0;
	uint16_t rtcAlarmHours = 0;
	uint16_t rtcAlarmDays = 0;
	uint16_t rtcAlarmMonth = 0;
	uint16_t rtcAlarmYear = 0;

#if 0	
	S_RTC_TIME_DATA_T sDateTime;
	uint32_t u32SubSecond;

	RtcGetCalendarValue( &sDateTime, &u32SubSecond );	
#else	
	S_RTC_TIME_DATA_T sDateTime = RtcTimerContext.CalendarDateTime;
#endif
	S_RTC_TIME_DATA_T sAlarmDateTime;
    RtcStopAlarm( );

    /*reverse counter */
#if 0
    rtcAlarmSubSeconds =  u32SubSecond;
#else
    rtcAlarmSubSeconds =  RtcTimerContext.sub_second;
#endif
	rtcAlarmSubSeconds += ( timeout & PREDIV_S );
    // convert timeout  to seconds
    timeout >>= N_PREDIV_S;

    // Convert microsecs to RTC format and add to 'Now'
    rtcAlarmDays =  sDateTime.u32Day;
    while( timeout >= TM_SECONDS_IN_1DAY )
    {
        timeout -= TM_SECONDS_IN_1DAY;
        rtcAlarmDays++;
    }

    // Calc hours
    rtcAlarmHours = sDateTime.u32Hour;
    while( timeout >= TM_SECONDS_IN_1HOUR )
    {
		timeout -= TM_SECONDS_IN_1HOUR;
        rtcAlarmHours++;
    }

    // Calc minutes
    rtcAlarmMinutes = sDateTime.u32Minute;
    while( timeout >= TM_SECONDS_IN_1MINUTE )
    {
        timeout -= TM_SECONDS_IN_1MINUTE;
        rtcAlarmMinutes++;
    }

    // Calc seconds
    rtcAlarmSeconds =  sDateTime.u32Second + timeout;

    //***** Correct for modulo********
    while( rtcAlarmSubSeconds >= ( PREDIV_S + 1 ) )
    {
        rtcAlarmSubSeconds -= ( PREDIV_S + 1 );
        rtcAlarmSeconds++;
    }

    while( rtcAlarmSeconds >= TM_SECONDS_IN_1MINUTE )
    { 
        rtcAlarmSeconds -= TM_SECONDS_IN_1MINUTE;
        rtcAlarmMinutes++;
    }

    while( rtcAlarmMinutes >= TM_MINUTES_IN_1HOUR )
    {
        rtcAlarmMinutes -= TM_MINUTES_IN_1HOUR;
        rtcAlarmHours++;
    }

    while( rtcAlarmHours >= TM_HOURS_IN_1DAY )
    {
        rtcAlarmHours -= TM_HOURS_IN_1DAY;
        rtcAlarmDays++;
    }

	rtcAlarmMonth = sDateTime.u32Month;
	
    if( sDateTime.u32Year % 4 == 0 ) 
    {
        if( rtcAlarmDays > DaysInMonthLeapYear[sDateTime.u32Month - 1] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[sDateTime.u32Month - 1];
			rtcAlarmMonth ++;
        }
    }
    else
    {
        if( rtcAlarmDays > DaysInMonth[sDateTime.u32Month - 1] )
        {   
            rtcAlarmDays = rtcAlarmDays % DaysInMonth[sDateTime.u32Month - 1];
			rtcAlarmMonth ++;
        }
    }

	rtcAlarmYear = sDateTime.u32Year;
	
	if(rtcAlarmMonth > 12){
		rtcAlarmYear++;
		rtcAlarmMonth = rtcAlarmMonth % 12;
	}
	
	/* Time Setting */
	sAlarmDateTime.u32Year       = rtcAlarmYear;
	sAlarmDateTime.u32Month      = rtcAlarmMonth;
	sAlarmDateTime.u32Day        = rtcAlarmDays;
	sAlarmDateTime.u32Hour       = rtcAlarmHours;
	sAlarmDateTime.u32Minute     = rtcAlarmMinutes;
	sAlarmDateTime.u32Second     = rtcAlarmSeconds;
	sAlarmDateTime.u32DayOfWeek  = RTC_SATURDAY;
	sAlarmDateTime.u32TimeScale  = RTC_CLOCK_24;
	
	RTC_SetAlarmDateAndTimeEx(&sAlarmDateTime, rtcAlarmSubSeconds);

    // Enable the Alarm interrupt
	RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);	
}

/*!
 * \brief  Alarm A callback.
 *
 * \param [IN] hrtc RTC handle
 */
void HAL_RTC_AlarmAEventCallback( void )
{
    TimerIrqHandler( );
}


/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    // Enable low power at irq
    LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );

    if ((RTC->INTEN & RTC_INTEN_ALMIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_ALMIF_Msk))          /* alarm interrupt occurred */
    {
        RTC_CLEAR_ALARM_INT_FLAG(RTC);
		HAL_RTC_AlarmAEventCallback();
    }
}

void RtcSetMcuWakeUpTime( void )
{
    uint32_t now, hit;
    int16_t mcuWakeUpTime;
	S_RTC_TIME_DATA_T sRTCAlarmDateTime;
	uint32_t u32SubSecond;

    if( ( McuWakeUpTimeInitialized == false ) &&
       ( NVIC_GetPendingIRQ( RTC_IRQn ) == 1 ) )
    {
        /* WARNING: Works ok if now is below 30 days
         *          it is ok since it's done once at first alarm wake-up
         */
        McuWakeUpTimeInitialized = true;
        now = ( uint32_t )RtcGetCalendarValue( &sRTCAlarmDateTime, &u32SubSecond );

		RTC_GetAlarmDateAndTime(&sRTCAlarmDateTime);
		u32SubSecond = (RTC->TALM >> 24) & 0x7F;
		
        hit = sRTCAlarmDateTime.u32Second +
              60 * ( sRTCAlarmDateTime.u32Minute +
              60 * ( sRTCAlarmDateTime.u32Hour +
              24 * ( sRTCAlarmDateTime.u32Day ) ) );

        hit = ( hit << N_PREDIV_S ) + ( u32SubSecond );

        mcuWakeUpTime = ( int16_t )( ( now - hit ) );
        McuWakeUpTimeCal += mcuWakeUpTime;
        //PRINTF( 3, "Cal=%d, %d\n", McuWakeUpTimeCal, mcuWakeUpTime);
    }
}

int16_t RtcGetMcuWakeUpTime( void )
{
    return McuWakeUpTimeCal;
}
