/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple WiFi demo for NuMaker board.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define WIFI_PORT   UART4    // Used to connect to WIFI module
#define BYPASS_PORT UART0    // Used to bypass WIFI module

#define RST_PIN         PE12
#define IOCTL_INIT      \
    do{ \
        PE->MODE = (GPIO_MODE_OUTPUT << 12*2); \
    }while(0)


void SYS_Init(void);
void DEBUG_PORT_Init(void);
void WIFI_PORT_Init(void);

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

    /* Enable UART4 module clock */
    CLK_EnableModuleClock(UART4_MODULE);

    /* Select UART4 module clock source as HIRC and UART4 module clock divider as 1 */
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HIRC, CLK_CLKDIV4_UART4(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB8_Msk | UART0_TXD_PB9_Msk))) | UART0_RXD_PB8 | UART0_TXD_PB9;

}

void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}


void WIFI_PORT_Init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART4CKEN_Msk;
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART4SEL_Msk)) | CLK_CLKSEL3_UART4SEL_HIRC;

    WIFI_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    WIFI_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

    /* Set multi-function pins for RXD and TXD */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(UART4_RXD_PC6_Msk | UART4_TXD_PC7_Msk))) | UART4_RXD_PC6 | UART4_TXD_PC7;
}



int main()
{
    int32_t i;
    uint8_t au8AT_VER[]="AT+GMR\r\n";

    SYS_UnlockReg();

    SYS_Init();
    DEBUG_PORT_Init();
    WIFI_PORT_Init();

    /*
        The ESP8266 WiFi module is connected to UART4 of M2354.
        In this demo code, COM and UART4(WiFi module) are connected to
        pass through all AT commands from debug port to UART.

        Therefore, user may control ESP8266 WiFi module on the Terminal or by PC tool e.g.
        "ESPlorer" (https://esp8266.ru/esplorer/)

    */

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              ESP8266-12F WiFi Module Demo                        |\n");
    printf("+------------------------------------------------------------------+\n");


    IOCTL_INIT;
    RST_PIN = 0;

    printf("Waiting .");
    CLK_SysTickLongDelay(3000000);

    putchar('.');
    CLK_SysTickLongDelay(1000000);
    RST_PIN = 1;

    /* Waiting for module ready */
    for(i = 0; i < 5; i++)
    {
        CLK_SysTickLongDelay(1000000);
        putchar('.');
    }
    printf(" Done\n");
    WIFI_PORT->FIFO |= UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;

    /* Get AT version */
    UART_Write(WIFI_PORT, au8AT_VER, sizeof(au8AT_VER));

    /* Bypass AT commands from debug port to WiFi port */
    while(1)
    {
        if((WIFI_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            while(BYPASS_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
            BYPASS_PORT->DAT = WIFI_PORT->DAT;
        }

        if((BYPASS_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            while(WIFI_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
            WIFI_PORT->DAT = BYPASS_PORT->DAT;
        }
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
