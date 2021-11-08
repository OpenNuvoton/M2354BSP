/****************************************************************************
 * @file     lpm-board.c
 * @version  V1.10
 * @brief    Target board low power modes management
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#include "NuMicro.h"
#include "utilities.h"
#include "lpm-board.h"

static uint32_t StopModeDisable = 0;
static uint32_t OffModeDisable = 0;

void LpmSetOffMode( LpmId_t id, LpmSetMode_t mode )
{
	CRITICAL_SECTION_BEGIN( );

	switch( mode )
	{
		case LPM_DISABLE:
		{
			OffModeDisable |= ( uint32_t )id;
			break;
		}
		case LPM_ENABLE:
		{
			OffModeDisable &= ~( uint32_t )id;
			break;
		}
		default:
		{
			break;
		}
	}

    CRITICAL_SECTION_END( );
    return;
}

void LpmSetStopMode( LpmId_t id, LpmSetMode_t mode )

{
    CRITICAL_SECTION_BEGIN( );
    switch( mode )
    {
        case LPM_DISABLE:
        {
            StopModeDisable |= ( uint32_t )id;
            break;
        }
        case LPM_ENABLE:
        {
			
			
            StopModeDisable &= ~( uint32_t )id;
            break;
        }
        default:
        {
            break;
        }
    }
    CRITICAL_SECTION_END( );

    return;
}

void LpmEnterLowPower( void )
{
    if( StopModeDisable != 0 )
    {
        /*!
        * SLEEP mode is required
        */
        LpmEnterSleepMode( );
        LpmExitSleepMode( );
    }
    else
    { 
        if( OffModeDisable != 0 )
        {
            /*!
            * STOP mode is required
            */
            LpmEnterStopMode( );
            LpmExitStopMode( );
        }
        else
        {
            /*!
            * OFF mode is required
            */
            LpmEnterOffMode( );
            LpmExitOffMode( );
        }
    }
    return;
}

LpmGetMode_t LpmGetMode(void)
{
    LpmGetMode_t mode;

    CRITICAL_SECTION_BEGIN( );

    if( StopModeDisable != 0 )
    {
        mode = LPM_SLEEP_MODE;
    }
    else
    {
        if( OffModeDisable != 0 )
        {
            mode = LPM_STOP_MODE;
        }
        else
        {
            mode = LPM_OFF_MODE;
        }
    }

    CRITICAL_SECTION_END( );
    return mode;
}

#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmEnterSleepMode( void )
{
	CLK_DisableSysTick();
	//For M252, it is idle mode
	/* Unlock protected registers before entering Power-down mode */
	SYS_UnlockReg();
    /* CLK_Idle(): Set Processor Low Power Mode Selection to select system low power mode  */
	/* Set System Power-down Enable Bit to set system to enter Idle mode after executing WFI instruction */
	CLK_Idle();
	
	/* Wake-up */
    SYS_LockReg();
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_LXT, CLK_GetLXTFreq() / 1000);
}

#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmExitSleepMode( void )
{
}

#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmEnterStopMode( void )
{
	CLK_DisableSysTick();

	//For M252, it is power done mode
	SYS_UnlockReg();

    /* To program PWRCTL register, it needs to disable register protection first. */
	CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_PD);

	/* Enable power down wakeup interrupt */
    //CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    //CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;

	/* Ener power down */
    CLK_PowerDown();
	/* Wake-up */
    SYS_LockReg();

	/* Enable system tick */
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_LXT, CLK_GetLXTFreq() / 1000);
}


#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmExitStopMode( void )
{
}

#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmEnterOffMode( void )
{
	//for M252, it is deep power done mode
#if 0
	CLK_DisableSysTick();
	SYS_UnlockReg();
	/* To program PWRCTL register, it needs to disable register protection first. */
	CLK->PMUCTL &= ~(CLK_PMUCTL_PDMSEL_Msk | CLK_PMUCTL_RTCWKEN_Msk);
	CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_DPD | CLK_PMUCTL_RTCWKEN_Msk; // DPD(Deep power dwon) Mode and RTC WK enable

//	CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
//	CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;
	CLK_PowerDown();
	SYS_LockReg();
	/* Enable system tick */
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_LXT, CLK_GetLXTFreq() / 1000);
#else
	printf("System try to enter off mode, but not implement \n");
#endif
}

#if defined (__ARMCC_VERSION)
__attribute__((weak))
#endif
#if defined (__GNUC__) && !defined (__ARMCC_VERSION)
__weak 
#endif
void LpmExitOffMode( void )
{
}



