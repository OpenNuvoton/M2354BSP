/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show hard fault information when hard fault happened.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/*
 The ARM M23 Generic User Guide lists the following sources for a hard fault:

 All faults result in the HardFault exception being taken or cause lockup if
 they occur in the NMI or HardFault handler. The faults are:
  - execution of an SVC instruction at a priority equal or higher than SVCall
  - execution of a BKPT instruction without a debugger attached
  - a system-generated bus error on a load or store
  - execution of an instruction from an XN memory address
  - execution of an instruction from a location for which the system generates a bus fault
  - a system-generated bus error on a vector fetch execution of an Undefined instruction
  - execution of an instruction when not in Thumb-State as a result of the T-bit being previously cleared to 0
  - an attempted load or store to an unaligned address.

 In this sample code, we will show you some information to debug with hardfault exception.

  - Default Hard Fault Handler

    The default hard fault handler is implement in retarget.c and called Hard_Fault_Handler
    By default, Hard_Fault_Handler will print out the information of r0, r1, r2, r3, r12, lr, pc and psr.

  - Overwrite the default Hard Fault Handler

    The default Hard_Fault_Handler is a weak function.
    User can over write it by implementing their own Hard_Fault_Handler.

 NOTE:
    Because hard fault exception will cause data stacking.
    User must make sure SP is pointing to an valid memory location.
    Otherwise, it may cause system lockup and reset when hard fault.

*/

#define USE_MY_HARDFAULT    1   /* Select to use default hard fault handler or not. 0: Default  1: User define */

void SYS_Init(void);
void UART0_Init(void);
void TMR1_IRQHandler(void);
void ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)__attribute__((noreturn));

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

    /* Enable UART0 and TIMER1 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Select UART0 and TIMER1 module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

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

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}



#if USE_MY_HARDFAULT

/**
  * @brief      User defined hard fault handler
  * @param      stack   A pointer to current stack
  * @return     None
  * @details    This function is an example to show how to implement user's hard fault handler
  *
  */
void ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t exception_num;
    uint32_t r0, r1, r2, r3, r12, pc, psr;
    uint32_t *stack;

    (void)psp;
    stack = (uint32_t *)msp;

    /* Get information from stack */
    r0  = stack[0];
    r1  = stack[1];
    r2  = stack[2];
    r3  = stack[3];
    r12 = stack[4];
    lr  = stack[5];
    pc  = stack[6];
    psr = stack[7];


    /* Check T bit of psr */
    if((psr & (1 << 24)) == 0)
    {
        printf("PSR T bit is 0.\nHard fault caused by changing to ARM mode!\n");
        while(1);
    }

    /* Check hard fault caused by ISR */
    exception_num = psr & xPSR_ISR_Msk;
    if(exception_num > 0)
    {
        /*
        Exception number
            0 = Thread mode
            1 = Reserved
            2 = NMI
            3 = HardFault
            4-10 = Reserved11 = SVCall
            12, 13 = Reserved
            14 = PendSV
            15 = SysTick, if implemented[a]
            16 = IRQ0.
                .
                .
            n+15 = IRQ(n-1)[b]
            (n+16) to 63 = Reserved.
        The number of interrupts, n, is 32
        */

        printf("Hard fault is caused in IRQ #%d\n", exception_num - 16);

        while(1);
    }

    printf("Hard fault location is at 0x%08x\n", pc);
    /*
        If the hard fault location is a memory access instruction, You may debug the load/store issues.

        Memory access faults can be caused by:
            Invalid address - read/write wrong address
            Data alignment issue - Violate alignment rule of Cortex-M processor
            Memory access permission - MPU violations or unprivileged access (Cortex-M0+)
            Bus components or peripheral returned an error response.
    */


    printf("r0  = 0x%x\n", r0);
    printf("r1  = 0x%x\n", r1);
    printf("r2  = 0x%x\n", r2);
    printf("r3  = 0x%x\n", r3);
    printf("r12 = 0x%x\n", r12);
    printf("lr  = 0x%x\n", lr);
    printf("pc  = 0x%x\n", pc);
    printf("psr = 0x%x\n", psr);

    while(1);

}

#endif


void TMR1_IRQHandler(void)
{
    printf("This is exception n = %d\n", TMR1_IRQn);
    M32(0) = 0;
    TIMER1->INTSTS = TIMER_INTSTS_TIF_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    void (*func)(void) = (void (*)(void))0x1000;
    char ch;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    while(1)
    {
        printf("\n\n");
        printf("+----------------------------------------------------+\n");
        printf("|        Hard Fault Handler Sample Code              |\n");
        printf("+----------------------------------------------------+\n");
        printf("| [0] Test Load/Store Hard Fault                     |\n");
        printf("| [1] Test Thumb/ARM mode Hard Fault                 |\n");
        printf("| [2] Test Hard Fault in ISR                         |\n");
        printf("+----------------------------------------------------+\n");
        ch = (char)getchar();

        switch(ch)
        {
            case '0':
                /* Write APROM will cause hard fault exception. (Memory access hard fault) */
                M32(0) = 0;
                break;
            case '1':
                /* Call function with bit0 = 0 will cause hard fault. (Change to ARM mode hard fault) */
                func();
                break;
            case '2':
                /* Generate Timer Interrupt to test hard fault in ISR */
                NVIC_EnableIRQ(TMR1_IRQn);
                TIMER1->CMP = 3;
                TIMER1->CTL = TIMER_CTL_INTEN_Msk | TIMER_CTL_CNTEN_Msk | TIMER_CTL_ACTSTS_Msk | TIMER_ONESHOT_MODE;
                break;
            default:
                break;
        }
    }

}
