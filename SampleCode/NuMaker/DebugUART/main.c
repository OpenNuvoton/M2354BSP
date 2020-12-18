/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple demo for NuTiny-M2354 board to show message from UART0 to ICE VCOM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
void UART0_Init(void);

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}



int main()
{
    int32_t i;

    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();

    /*
        In NuMaker board, the NuLink ICE also provides VCOM debug port for debug message.
        Once plug the ICE to PC, there will also be a USB VCOM on PC.
        All message print to UART0 will output to the VCOM.

        NOTE:
            User must use "Hardware Manager" of windows to identify which COM number is used for VCOM.
            Then open it by any serial terminal tool for monitor debug message.

    */

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              Simple Debug Message Demo                           |\n");
    printf("+------------------------------------------------------------------+\n");

    /* Init GPIO for LED toggle */
    PD->MODE = (GPIO_MODE_OUTPUT << 2 * 2) | (GPIO_MODE_OUTPUT << 3 * 2);
    PD2 = 1;
    PD3 = 0;

    i = 0;
    while(1)
    {
        PD2 ^= 1;
        PD3 ^= 1;
        CLK_SysTickLongDelay(200000);
        PD2 ^= 1;
        PD3 ^= 1;
        CLK_SysTickLongDelay(200000);

        printf("Iter: %d\n", i++);
    }


}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
