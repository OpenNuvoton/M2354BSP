/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP ECC P-192 ECDSA signature verification function.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


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

    /* ENable CAN module clock */
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
    char sha_msg[] = "608079423f12421de616b7493ebe551cf4d65b92";      /* SHA-1 hash                                    */
    char Qx[] = "07008ea40b08dbe76432096e80a2494c94982d2d5bcf98e6";   /* public key 1                                  */
    char Qy[] = "76fab681d00b414ea636ba215de26d98c41bd7f2e4d65477";   /* public key 2                                  */
    char R[] = "6994d962bdd0d793ffddf855ec5bf2f91a9698b46258a63e";    /* Expected answer: R of (R,S) digital signature */
    char S[] = "02ba6465a234903744ab02bc8521405b73cf5fc00e1a9f41";    /* Expected answer: S of (R,S) digital signature */

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    /* Enable crypto interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Verification Demo    |\n");
    printf("+---------------------------------------------+\n");

    /* Verify the signature */
    if(ECC_VerifySignature(CRPT, CURVE_P_192, sha_msg, Qx, Qy, R, S) < 0)
    {
        printf("ECC signature verification failed!!\n");
        while(1);
    }

    printf("ECC digital signature verification OK.\n");

    while(1);
}



