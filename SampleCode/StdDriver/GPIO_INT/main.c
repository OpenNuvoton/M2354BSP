/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO interrupt function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"



void GPB_IRQHandler(void);
void GPC_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M2354.s.
 */
void GPB_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB.2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       GPIO PC IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PC default IRQ, declared in startup_M2354.s.
 */
void GPC_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PC.5 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PC, BIT5))
    {
        GPIO_CLR_INT_FLAG(PC, BIT5);
        printf("PC.5 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        u32temp = PC->INTSRC;
        PC->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
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
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    GPIO PB.2 and PC.5 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("PB.2 and PC.5 are used to test interrupt ......\n");

    /* Configure PB.2 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);

    /* Configure PC.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PC, BIT5, GPIO_MODE_QUASI);
    GPIO_EnableInt(PC, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPC_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_SET_DEBOUNCE_TIME(PC, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT2);
    GPIO_ENABLE_DEBOUNCE(PC, BIT5);

    /* Waiting for interrupts */
    while(1);
}
