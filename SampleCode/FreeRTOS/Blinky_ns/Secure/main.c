/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for TrustZone
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                      /* Device header */
#include "partition_M2354.h"


#define NEXT_BOOT_BASE  0x10040000
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */


/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);
typedef int32_t (*Secure_funcptr)(uint32_t);

void SysTick_Handler(void);

/*----------------------------------------------------------------------------
  NonSecure callable function for NonSecure callback
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t ticks;

    switch(ticks++)
    {
        case   0:
            PB0 = 0;
            PB1 = 1;
            break;
        case 10:
            PB0 = 1;
            PB1 = 0;
            break;
        case 30:
            PB0 = 0;
            PB1 = 0;
            break;

        default:
            if(ticks > 50)
            {
                ticks = 0;
                PB0 = 1;
                PB1 = 1;
            }
    }
}

void SYS_Init(void);
void DEBUG_PORT_Init(void);
void Boot_Init(uint32_t u32BootBase);


/*----------------------------------------------------------------------------
    Boot_Init function is used to jump to next boot code.
 *----------------------------------------------------------------------------*/
void Boot_Init(uint32_t u32BootBase)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = u32BootBase;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((NonSecure_funcptr)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    fp = cmse_nsfptr_create(fp);

    /* Check if the Reset_Handler address is in Non-secure space */
    if(cmse_is_nsfptr(fp) && (((uint32_t)fp & 0xf0000000) == 0x10000000))
    {
        printf("Execute non-secure code ...\n");
        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");
        printf("CPU will halted at non-secure state\n");

        /* Set nonsecure MSP in nonsecure region */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = JUMP_HERE;
        fp = (NonSecure_funcptr)(NON_SECURE_SRAM_BASE + 1);
        fp(0);

        while(1);
    }
}


/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    SYS_UnlockReg();

    SYS_Init();

    /* UART is configured as debug port */
    DEBUG_PORT_Init();

    printf("Secure is running ...\n");

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Init and jump to Non-secure code */
    Boot_Init(NEXT_BOOT_BASE);

    do
    {
        __WFI();
    }
    while(1);

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init for nonsecure code                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_UART1SEL_Msk)) | CLK_CLKSEL2_UART1SEL_HIRC;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_RXD_PA2_Msk)) | UART1_RXD_PA2;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_TXD_PA3_Msk)) | UART1_TXD_PA3;

}

void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
