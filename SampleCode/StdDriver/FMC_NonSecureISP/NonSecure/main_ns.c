/**************************************************************************//**
 * @file     main_ns.c
 * @version  V1.00
 * @brief    Non-secure ISP sample code
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include "NuMicro.h"

#define NS_APROM_TEST_START_ADDR   0x60000
#define NS_APROM_TEST_END_ADDR    (NS_APROM_TEST_START_ADDR+0x10000)


typedef int32_t (*funcptr)(uint32_t);


extern int32_t Secure_func(void);
void App_Init(uint32_t u32BootBase);
void DEBUG_PORT_Init(void);



/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32ISPData, i;
    int32_t i32Ret = 0;

    DEBUG_PORT_Init();

    printf("\n");
    printf("+---------------------------------------------+\n");
    printf("|           Nonsecure is running ...          |\n");
    printf("+---------------------------------------------+\n");

    printf("  Non Secure ISP Test start ...\n\n");

    FMC_ENABLE_ISP();

    u32ISPData = FMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32ISPData);

    u32ISPData = FMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32ISPData);

    for(i = 0; i < 3; i++)
    {
        u32ISPData = FMC_ReadUID((uint8_t)i);
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32ISPData);
    }

    for(i = 0; i < 4; i++)
    {
        u32ISPData = FMC_ReadUCID(i);
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32ISPData);
    }

    FMC_ENABLE_AP_UPDATE();

    for(u32Addr = NS_APROM_TEST_START_ADDR; u32Addr < NS_APROM_TEST_END_ADDR; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        i32Ret = FMC_Erase(u32Addr);

        if(i32Ret != 0)
        {
            printf("[%d] Error:  ISP Fail\n", __LINE__);
            goto lexit;
        }
    }

    printf("\n  Non Secure ISP erase test pass \n");

    for(u32Addr = NS_APROM_TEST_START_ADDR; u32Addr < NS_APROM_TEST_END_ADDR; u32Addr += 4)
    {
        u32ISPData = FMC_Read(u32Addr);

        if((FMC_NS->ISPSTS & FMC_ISPSTS_ISPFF_Msk) == FMC_ISPSTS_ISPFF_Msk)
        {
            printf("[%d] Error:  ISP Fail\n", __LINE__);
            goto lexit;
        }
        if(u32ISPData != 0xFFFFFFFF)
        {
            printf("[%d] ISP read rrror:  address[0x%x]  data[0x%x]  -> should be 0xFFFFFFFF\n", __LINE__, u32Addr, u32ISPData);
            goto lexit;
        }

    }
    printf("  Non Secure ISP erase check pass \n");

    for(u32Addr = NS_APROM_TEST_START_ADDR; u32Addr < NS_APROM_TEST_END_ADDR; u32Addr += 4)
    {
        FMC_Write(u32Addr, u32Addr);

        if((FMC_NS->ISPSTS & FMC_ISPSTS_ISPFF_Msk) == FMC_ISPSTS_ISPFF_Msk)
        {
            printf("[%d] Error:  ISP Fail\n", __LINE__);
            goto lexit;
        }
    }

    printf("  Non Secure ISP program test pass \n");

    for(u32Addr = NS_APROM_TEST_START_ADDR; u32Addr < NS_APROM_TEST_END_ADDR; u32Addr += 4)
    {
        u32ISPData = FMC_Read(u32Addr);

        if((FMC_NS->ISPSTS & FMC_ISPSTS_ISPFF_Msk) == FMC_ISPSTS_ISPFF_Msk)
        {
            printf("[%d] Error:  ISP Fail\n", __LINE__);
            goto lexit;
        }
        if(u32ISPData != u32Addr)
        {
            printf("[%d] ISP read rrror:  address[0x%x]  data[0x%x]  -> should be 0xFFFFFFFF\n", __LINE__, u32Addr, u32ISPData);
            goto lexit;
        }

    }
    printf("  Non Secure ISP program check pass \n");

    printf("\n\n  Non Secure ISP Test OK\n");

lexit:

    while(1);
}



void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void App_Init(uint32_t u32BootBase)
{
    funcptr fp;
    uint32_t u32StackBase;

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((funcptr)(*(((uint32_t *)SCB->VTOR) + 1)));

    /* Check if the stack is in secure SRAM space */
    u32StackBase = M32(u32BootBase);
    if((u32StackBase >= 0x30000000UL) && (u32StackBase < 0x40000000UL))
    {
        printf("Execute non-secure code ...\n");
        /* SCB.VTOR points to the target Secure vector table base address. */
        SCB->VTOR = u32BootBase;

        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");

        while(1);
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
