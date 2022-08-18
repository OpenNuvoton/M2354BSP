/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP ECC P-192 key generation function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          192          /* Select ECC P-192 curve, 192-bits key length */

static char d[]  = "e5ce89a34adddf25ff3bf1ffe6803f57d0220de3118798ea";    /* private key */
static char Qx[] = "8abf7b3ceb2b02438af19543d3e5b1d573fa9ac60085840f";    /* expected answer: public key 1 */
static char Qy[] = "a87f80182dcd56a6a061f81f7da393e7cffd5e0738c6b245";    /* expected answer: public key 2 */

static char gKey1[168], gKey2[168];             /* temporary buffer used to keep output public keys */

void CRPT_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);



void CRPT_IRQHandler(void)
{
    ECC_DriverISR(CRPT);
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


void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Generation Demo     |\n");
    printf("+---------------------------------------------+\n");

    /* Enable ECC interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    /* Generate public key from private key d */
    if(ECC_GeneratePublicKey(CRPT, CURVE_P_192, d, gKey1, gKey2) < 0)
    {
        printf("ECC key generation failed!!\n");
        goto lexit;
    }

    /* Verify public key 1 */
    if(memcmp(Qx, gKey1, KEY_LENGTH / 8))
    {

        printf("Public key 1 [%s] is not matched with expected [%s]!\n", gKey1, Qx);

        if(memcmp(Qx, gKey1, KEY_LENGTH / 8) == 0)
            printf("PASS.\n");
        else
            printf("Error !!\n");


        for(i = 0; i < KEY_LENGTH / 8; i++)
        {
            if(Qx[i] != gKey1[i])
                printf("\n%d - 0x%x 0x%x\n", i, Qx[i], gKey1[i]);
        }
        goto lexit;
    }

    /* Verify public key 2 */
    if(memcmp(Qy, gKey2, KEY_LENGTH / 8))
    {
        printf("Public key 2 [%s] is not matched with expected [%s]!\n", gKey2, Qy);
        goto lexit;
    }

    printf("ECC key compared OK.\n");

lexit:

    while(1);
}

