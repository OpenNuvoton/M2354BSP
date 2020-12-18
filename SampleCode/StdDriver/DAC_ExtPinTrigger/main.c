/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger DAC conversion by external pin.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"


static const uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                                      4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                                      3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                                      639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                                      238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                                     };

static const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);
static uint32_t g_u32Index = 0;

void DAC_IRQHandler(void);
void SYS_Init(void);



void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC0, 0))
    {

        if(g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
        else
        {
            DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index++]);

            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);
        }
    }
    return;
}

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

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Select EPWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pin for DAC voltage output */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | DAC0_OUT_PB12;

    /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));

    /* Set multi-function pin for DAC conversion trigger */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA10MFP_Msk) | DAC0_ST_PA10;

}

int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("Please connect PC0 with PA10, use PC0 to trigger DAC conversion\n");

    /* Set the falling edge trigger, enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_FALLING_EDGE_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Set PC.0 as output mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);

    while(1)
    {
        /* Toggle PC.0 */
        PC0 = 1;
        CLK_SysTickDelay(100);
        PC0 = 0;
        CLK_SysTickDelay(100);
    }

}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
