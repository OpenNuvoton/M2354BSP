/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/* Function prototype declaration */
void SYS_Init(void);
void PowerDownFunction(void);
int IsDebugFifoEmpty(void);
void ACMP01_IRQHandler(void);


int32_t main(void)
{
    uint32_t u32DelayCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(DEBUG_PORT, 115200);

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|             ACMP Sample Code          |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates ACMP0 wake-up function. Using ACMP0_P0 (PA11) as ACMP0\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP0_O (PB7).\n");

    printf("When the voltage of the positive input is greater than the voltage of the negative input,\n");
    printf("the analog comparator outputs logical one; otherwise, it outputs logical zero.\n");
    printf("This chip will be waked up from power down mode when detecting a transition of analog comparator's output.\n");
    printf("Press any key to enter power down mode ...");
    getchar();
    printf("\n");

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV_SRC(ACMP01, ACMP_VREF_CRVSSEL_VDDA);
    /* Select CRV level: VDDA * 9 / 24 */
    ACMP_CRV_SEL(ACMP01, 5);
    /* Configure ACMP0. Enable ACMP0 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    __NOP();
    for(u32DelayCount = 0; u32DelayCount < 100; u32DelayCount++); /* For ACMP setup time */
    __NOP();
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 0);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    PowerDownFunction();
    printf("Wake up by ACMP0!\n");
    while(1);

}

void ACMP01_IRQHandler(void)
{
    printf("\nACMP0 interrupt!\n");
    /* Disable interrupt */
    ACMP_DISABLE_INT(ACMP01, 0);
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 0);
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pin for ACMP0 positive input pin */
    SYS->GPA_MFPH = ACMP0_P0_PA11;

    /* Set multi-function pin for ACMP0 output pin */
    SYS->GPB_MFPL = ACMP0_O_PB7;

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Disable digital input path of analog pin ACMP0_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 7));
}


void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("\nSystem enter power-down mode ... ");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Deep sleep mode is selected */
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);

    __WFI();
}

