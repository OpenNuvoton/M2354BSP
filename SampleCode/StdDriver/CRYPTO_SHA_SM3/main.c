/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to calculate SHA256 of the data in Flash.
 *
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

//#define printf(...)

void SYS_Init(void);
void DEBUG_PORT_Init(void);
int32_t SM3(uint32_t *pu32Addr, uint32_t size, uint32_t digest[]);

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

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}


void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

static __attribute__((aligned(4))) uint8_t g_au8Test[3] =
{
    0x61, 0x62, 0x63
};


static __attribute__((aligned(4))) uint8_t g_au8Test2[64] =
{
    0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64,
    0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64,
    0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64,
    0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64, 0x61, 0x62, 0x63, 0x64
};


/*-----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t hash[8] = {0};
    int32_t i;
    uint8_t *pu8;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+-----------------------------------+\n");
    printf("|      M2354 Crypto SM3 Demo        |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);

    printf("Input data:\n");
    for(i = 0; i < 3; i++)
    {
        printf("%02x", g_au8Test[i]);
    }
    printf("\n");

    if( SM3((uint32_t *)g_au8Test, 3, hash) < 0 )
        goto lexit;

    pu8 = (uint8_t *)hash;
    printf("\nOutput Hash:\n");
    for(i = 0; i < 32; i++)
    {
        printf("%02x", *pu8++);
    }
    printf("\n");


    printf("The result should be:\n");
    printf("66c7f0f462eeedd9d1f2d46bdc10e4e24167c4875cf2f7a2297da02b8f4ba8e0\n\n");

    printf("Input data2:\n");
    for(i = 0; i < 64; i++)
    {
        printf("%02x", g_au8Test2[i]);
    }
    printf("\n");

    if( SM3((uint32_t *)g_au8Test2, 64, hash) < 0 )
        goto lexit;

    printf("\nOutput Hash:\n");
    pu8 = (uint8_t *)hash;
    for(i = 0; i < 32; i++)
    {
        printf("%02x", *pu8++);
    }
    printf("\n");

    printf("The result should be:\n");
    printf("debe9ff92275b8a138604889c18e5a4d6fdb70e5387e5765293dcba39c0c5732\n\n");

    printf("Demo Finished.\n");

lexit:

    while(1) {}
}
