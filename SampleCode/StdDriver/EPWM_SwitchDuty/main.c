/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Change duty cycle of output waveform by configured period.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


uint32_t CalNewDutyCMR(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution);
void EPWM0_P0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       EPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 interrupt event
 */
void EPWM0_P0_IRQHandler(void)
{

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

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Select EPWM0 module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pins for EPWM */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE7MFP_Msk)) | EPWM0_CH0_PE7;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE6MFP_Msk)) | EPWM0_CH1_PE6;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE5MFP_Msk)) | EPWM0_CH2_PE5;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE4MFP_Msk)) | EPWM0_CH3_PE4;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/**
 * @brief       Calculate the comparator value of new duty by configured period
 *
 * @param       epwm                  The pointer of the specified EPWM module
 *
 * @param       u32ChannelNum        EPWM channel number. Valid values are between 0~5
 *
 * @param       u32DutyCycle         Target generator duty cycle percentage. Valid range are between 0 ~ u32CycleResolution.
 *                                   If u32CycleResolution is 100, and u32DutyCycle is 10 means 10%, 20 means 20% ...
 *
 * @param       u32CycleResolution   Target generator duty cycle resolution. The value in general is 100.
 *
 * @return      The compatator value by new duty cycle
 */
uint32_t CalNewDutyCMR(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
    return (u32DutyCycle * (EPWM_GET_CNR(epwm, u32ChannelNum) + 1) / u32CycleResolution);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t  u8Option;
    uint32_t u32NewDutyCycle = 0, u32NewCMR = 0;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                                   |\n");
    printf("|                                                                                   |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use EPWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0 channel 0(PE.7)\n");
    printf("\nOutput waveform is 1800Hz and it's duty is 50%%.\n");

    /* EPWM0 channel 0 frequency is 1800Hz, duty 50%, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 1800, 50);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK);

    /* Start EPWM counter */
    EPWM_Start(EPWM0, EPWM_CH_0_MASK);

    while(1)
    {
        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = (uint8_t)getchar();

        if(u8Option == '1')
        {
            u32NewDutyCycle = 100;
        }
        else if(u8Option == '2')
        {
            u32NewDutyCycle = 75;
        }
        else if(u8Option == '3')
        {
            u32NewDutyCycle = 25;
        }
        else if(u8Option == '4')
        {
            u32NewDutyCycle = 0;
        }
        else
        {
            printf("Exit\n");
            break;
        }
        /* Get new comparator value by call CalNewDutyCMR() */
        u32NewCMR = CalNewDutyCMR(EPWM0, 0, u32NewDutyCycle, 100);
        /* Set new comparator value to register */
        EPWM_SET_CMR(EPWM0, 0, u32NewCMR);
    }

    /* Stop EPWM counter */
    EPWM_Stop(EPWM0, EPWM_CH_0_MASK);
    /* Disable output of EPWM0 channel 0 */
    EPWM_DisableOutput(EPWM0, EPWM_CH_0_MASK);

    while(1);

}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
