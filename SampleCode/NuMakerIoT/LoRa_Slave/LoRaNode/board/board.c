/****************************************************************************
 * @file     board.c
 * @version  V1.10
 * @brief    Target board general functions implementation
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"
#include "utilities.h"
#include "gpio.h"
#include "spi.h"
//#include "i2c.h"
#include "board.h"
#include "board-config.h"
#include "lpm-board.h"
#include "rtc-board.h"

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    #include "sx126x-board.h"
#elif defined( LR1110MB1XXS )
    #include "lr1110-board.h"
#elif defined( SX1272MB2DAS)
    #include "sx1272-board.h"
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    #include "sx1276-board.h"
#endif

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

/*!
 * LED GPIO pins objects
 */

Gpio_t Led1;
Gpio_t Led2;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * System tick counter
 */
static volatile uint64_t SystemTickCnt = 0;

/**
 * Set UART0 default multi function pin
 *
 * @param  none
 * @return none
 *
 * @brief  The initialization of uart0 default multiple-function pin.
 */
extern void Uart0DefaultMPF(void);

/*!
 * Enable the external 32K XTAL clock
 */
static void LXT_Enable(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Enable external 32768Hz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin XT32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));
}


static void SystemClockConfig( void )
{
	/* Unlock protected registers */
	SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    //CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select HIRC as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

#else
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select HXT as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));
#endif

    /* Enable external 32768Hz XTAL */
    LXT_Enable();
	
#if 1
	/* Select PCLK as the clock source of SPI3 */
    CLK_SetModuleClock(SPI3_MODULE, CLK_CLKSEL2_SPI3SEL_PCLK0, MODULE_NoMsk);


    /* Enable SPI3 peripheral clock */
    CLK_EnableModuleClock(SPI3_MODULE);

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);
#endif

	/* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
	
	/* Enable system tick */
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_LXT, CLK_GetLXTFreq() / 1000);

	/* Unlock protected registers */
	SYS_LockReg();


    // SysTick_IRQn interrupt configuration
	NVIC_SetPriority( SysTick_IRQn, 0);
}

static void BoardUnusedIoInit( void )
{

}

uint8_t GetBoardPowerSource( void )
{
	return BATTERY_POWER;
}

static void SystemClockReConfig( void )
{

}

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void* context )
{
    RtcSetMcuWakeUpTime( );
    SystemWakeupTimeCalibrated = true;
}

static void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {
        }
    }
}

static void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//	Public APIs related board
//////////////////////////////////////////////////////////////////////////////////////////////////

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

uint8_t BoardGetBatteryLevel( void )
{
	//TODO
    return 0;
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{

    if( McuInitialized == false )
    {
        // LEDs

        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

		SystemClockConfig();

		/* Set GPB multi-function pins for UART0 RXD and TXD (for debug console) */
		Uart0DefaultMPF();
		UART0_Init();

		RtcInit();

		BoardUnusedIoInit( );
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            // Disables OFF mode - Enables lowest power mode (STOP)
            LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
        }
    }
	else
	{
		SystemClockReConfig( );
	}

#if defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiInit( &SX1276.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );
#endif

    if( McuInitialized == false )
    {
#if defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
        SX1276IoDbgInit( );
        SX1276IoTcxoInit( );
#endif
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            CalibrateSystemWakeupTime( );
        }
	}
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

	//Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
#if defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
#endif
}

uint32_t BoardGetRandomSeed( void )
{
	uint32_t u32RTCTime = RtcGetTimerValue();
	uint32_t u32PDID = SYS_ReadPDID();

    return (u32RTCTime ^ u32PDID);
}

void BoardGetUniqueId( uint8_t *id )
{
	uint32_t u32PDID = SYS_ReadPDID();

	id[7] = u32PDID >> 24;
	id[6] = u32PDID >> 16;
	id[5] = u32PDID >> 8;
	id[4] = u32PDID & 0x000000FF;

	id[3] = id[7] ^ id[4];
	id[2] = id[6] ^ id[5];
	id[1] = id[7] ^ id[5];
	id[0] = id[6] ^ id[4];
}

uint16_t BoardBatteryMeasureVolage( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

void SysTick_Handler( void )
{
	SystemTickCnt++;
}

void DelayMsMcu( uint32_t ms )
{
	uint32_t u32CurTick = SystemTickCnt;

	while(SystemTickCnt <= u32CurTick + ms)
	{

	}
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );

    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending 
     * and cortex will not enter low power anyway
     */
    LpmEnterLowPower( );

    __enable_irq( );
}


#ifdef USE_FULL_ASSERT

#include <stdio.h>
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */

void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %u\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}

#endif

