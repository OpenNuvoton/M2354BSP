/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for TrustZone
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                      /* Device header */
#include "partition_M2354.h"


#define PLL_CLOCK       FREQ_96MHZ


#define NEXT_BOOT_BASE  0x10040000
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */


/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);
typedef int32_t (*Secure_funcptr)(uint32_t);

__NONSECURE_ENTRY
int32_t Secure_func(void);
__NONSECURE_ENTRY
int32_t Secure_LED_On(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_LED_Off(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_LED_On_callback(NonSecure_funcptr *callback);
__NONSECURE_ENTRY
int32_t Secure_LED_Off_callback(NonSecure_funcptr *callback);
int32_t LED_On(void);
int32_t LED_Off(void);
void SysTick_Handler(void);
/*----------------------------------------------------------------------------
  Secure functions exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
int32_t Secure_func(void)
{
    printf("Secure NSC func\n");

    return 1;
}

__NONSECURE_ENTRY
int32_t Secure_LED_On(uint32_t num)
{
	  (void)num;
    printf("Secure LED On call by Non-secure\n");
    PA10 = 0;
    PB0 = 0;
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off(uint32_t num)
{
	  (void)num;
    printf("Secure LED Off call by Non-secure\n");
    PA10 = 1;
    PB0 = 1;
    return 1;
}

/*----------------------------------------------------------------------------
  NonSecure callable function for NonSecure callback
 *----------------------------------------------------------------------------*/

static NonSecure_funcptr pfNonSecure_LED_On = (NonSecure_funcptr)NULL;
static NonSecure_funcptr pfNonSecure_LED_Off = (NonSecure_funcptr)NULL;

__NONSECURE_ENTRY
int32_t Secure_LED_On_callback(NonSecure_funcptr *callback)
{
    pfNonSecure_LED_On = (NonSecure_funcptr)cmse_nsfptr_create(callback);
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off_callback(NonSecure_funcptr *callback)
{
    pfNonSecure_LED_Off = (NonSecure_funcptr)cmse_nsfptr_create(callback);
    return 0;
}

/*----------------------------------------------------------------------------
  Secure LED control function
 *----------------------------------------------------------------------------*/
int32_t LED_On(void)
{
    printf("Secure LED On\n");
    PA11 = 0;
    PB1 = 0;
    return 1;
}

int32_t LED_Off(void)
{
    printf("Secure LED Off\n");
    PA11 = 1;
    PB1 = 1;
    return 1;
}

/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch(u32Ticks++)
    {
        case   100:
            LED_On();
            break;
        case 200:
            LED_Off();
            break;
        case 300:
            if(pfNonSecure_LED_On != NULL)
            {
                pfNonSecure_LED_On(1u);
            }
            break;
        case 500:
            if(pfNonSecure_LED_Off != NULL)
            {
                pfNonSecure_LED_Off(1u);
            }
            break;

        default:
            if(u32Ticks > 600)
            {
                u32Ticks = 0;
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

    /* Init GPIO Port A for secure LED control */
    GPIO_SetMode(PA, BIT11 | BIT10, GPIO_MODE_OUTPUT);

    /* Init GPIO Port B for secure LED control */
    GPIO_SetMode(PB, BIT1 | BIT0, GPIO_MODE_OUTPUT);

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

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_UART0SEL_Msk)) | CLK_CLKSEL2_UART0SEL_HIRC;

    /* Enable SRAM module clock */
    CLK_EnableModuleClock(SRAM0_MODULE); 
    CLK_EnableModuleClock(SRAM1_MODULE); 
    CLK_EnableModuleClock(SRAM2_MODULE);   

    /* Enable GPIO port A~H module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);  
    CLK_EnableModuleClock(GPC_MODULE);  
    CLK_EnableModuleClock(GPD_MODULE);  
    CLK_EnableModuleClock(GPE_MODULE);  
    CLK_EnableModuleClock(GPF_MODULE);  
    CLK_EnableModuleClock(GPG_MODULE);  
    CLK_EnableModuleClock(GPH_MODULE);       
    
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

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
