/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple WiFi demo for NuMaker board.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define WIFI_PORT   UART4    // Used to connect to WIFI module
#define BYPASS_PORT UART0    // Used to byass WIFI module

#define LED_OFF         PD3 // Green LED
#define RST_PIN         PC13
#define FW_UPDATE_OFF   PD12
#define IOCTL_INIT      \
    do{ \
        PD->MODE = (GPIO_MODE_OUTPUT << 3*2) | (GPIO_MODE_OUTPUT << 12*2); \
        PC->MODE = (GPIO_MODE_OUTPUT << 13*2); \
    }while(0)


void SYS_Init(void);
void DEBUG_PORT_Init(void);
void WIFI_PORT_Init(void);

void SYS_Init(void)
{
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set power level to 0 */
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL0);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable SRAM module clock */
    CLK_EnableModuleClock(SRAM1_MODULE);
    CLK_EnableModuleClock(SRAM2_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Select IP clock source */
    CLK->CLKSEL2 = CLK_CLKSEL2_UART0SEL_HIRC;
    CLK->CLKSEL3 = CLK_CLKSEL3_UART4SEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART4CKEN_Msk;


    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

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
    LED_OFF = 1;
    RST_PIN = 0;
    FW_UPDATE_OFF = 1;

    printf("Waiting .");
    CLK_SysTickLongDelay(3000000);

    //FW_UPDATE_OFF = 0; // Set 0 to enable WIFI module firmware update.
    FW_UPDATE_OFF = 1; // Set 1 to Disable WIFI module firmware update.
    
    putchar('.');
    CLK_SysTickLongDelay(1000000);
    LED_OFF = 0;
    RST_PIN = 1;
    
    /* Waiting for module ready */
    for(i=0;i<5;i++)
    {
        CLK_SysTickLongDelay(1000000);
        putchar('.');
    }
    printf(" Done\n");
    WIFI_PORT->FIFO |= UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;


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
