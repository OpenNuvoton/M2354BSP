/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to measure AVDD voltage by EADC.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define VBG_VOLTAGE (1200) /* 1.20V = 1200 mV (Typical band-gap voltage) */
#define ADC_SAMPLE_COUNT 128 /* The last line of GetAVDDCodeByADC() needs to be revised when ADC_SAMPLE_COUNT is changed. */
/* For example, if ADC_SAMPLE_COUNT is changed to 64, then the code need revised to "return (u32Sum >> 6);" */

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_u8ADF;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
uint32_t GetAVDDVoltage(void);
uint32_t GetAVDDCodeByADC(void);
void EADC0_IRQHandler(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDVoltage                                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   AVDD voltage(mV).                                                                                     */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Use Band-gap voltage to calculate AVDD voltage                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult;
    uint64_t  u64MvAVDD;

    /* Calculate Vref by using conversion result of VBG */
    u32ConversionResult = GetAVDDCodeByADC();

    /* u32ConversionResult = VBG * 4096 / Vref, Vref = AVDD */
    /* => AVDD = VBG * 4096 / u32ConversionResult */
    u64MvAVDD = (VBG_VOLTAGE << 12) / (uint64_t)u32ConversionResult;

    printf("Conversion result: 0x%X\n", u32ConversionResult);

    return (uint32_t)u64MvAVDD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDCodeByADC                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   ADC code of AVDD voltage.                                                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Get EADC conversion result of Band-gap voltage.                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDCodeByADC(void)
{
    uint32_t u32Count, u32Sum, u32Data, u32TimeOutCnt;

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 16 external sampling time to 0xFF */
    EADC_SetExtendSampleTime(EADC, 16, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 16 interrupt. */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT16);//Enable sample module 16 interrupt.
    NVIC_EnableIRQ(EADC0_IRQn);

    s_u8ADF = 0;
    u32Sum = 0;

    /* sample times are according to ADC_SAMPLE_COUNT definition */
    for(u32Count = 0; u32Count < ADC_SAMPLE_COUNT; u32Count++)
    {
        /* Delay for band-gap voltage stability */
        CLK_SysTickDelay(100);

        /* Start A/D conversion */
        EADC_START_CONV(EADC, BIT16);

        u32Data = 0;
        /* Wait conversion done */
        u32TimeOutCnt = EADC_TIMEOUT;
        while(s_u8ADF == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EADC conversion done time-out!\n");
                break;
            }
        }
        s_u8ADF = 0;
        /* Get the conversion result */
        u32Data = EADC_GET_CONV_DATA(EADC, 16);
        /* Sum each conversion data */
        u32Sum += u32Data;
    }

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Disable ADC */
    EADC_Close(EADC);

    /* Return the average of ADC_SAMPLE_COUNT samples */
    return (u32Sum >> 7);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    uint32_t u32Flag;

    /* Get ADC conversion finish interrupt flag */
    u32Flag = EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Check ADC conversion finish */
    if(u32Flag & EADC_STATUS2_ADIF0_Msk)
        s_u8ADF = 1;

    /* Clear conversion finish flag */
    EADC_CLR_INT_FLAG(EADC, u32Flag);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32AVDDVoltage;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|   EADC Voltage Detection Sample Code   |\n");
    printf("+----------------------------------------+\n\n");

    printf("In this sample code, software will get voltage value from AVDD.\n");
    printf("Notice that the Vref of EADC is from AVDD.\n\n");

    /* Measure AVDD */
    u32AVDDVoltage = GetAVDDVoltage();
    if(u32AVDDVoltage < 1620 || u32AVDDVoltage > 3600)
        printf("Abnormal AVDD Voltage: %dmV\n", u32AVDDVoltage);
    else
        printf("AVDD Voltage: %dmV\n", u32AVDDVoltage);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    while(1);
}
