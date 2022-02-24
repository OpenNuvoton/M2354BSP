/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to read/program embedded flash by ISP function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


static uint32_t  g_auPageBuff[FMC_FLASH_PAGE_SIZE / 4];

void SYS_Init(void);

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

int32_t main(void)
{
    uint32_t i, u32Addr, u32Maddr, u32Err = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\n");
    printf("+-------------------------------------+\n");
    printf("|          Multi-word Program Sample  |\n");
    printf("+-------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    FMC_Open();                        /* Enable FMC ISP function */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM erase/program */

    for(u32Addr = 0x80000; u32Addr < 0x82000; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", u32Addr);

        if(FMC_Erase(u32Addr) < 0)
        {
            printf("    Erase failed!!\n");
            u32Err = 1;
            goto err_out;
        }

        printf("    Program...\n");

        for(u32Maddr = u32Addr; u32Maddr < u32Addr + FMC_FLASH_PAGE_SIZE; u32Maddr += FMC_MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for(i = 0; i < FMC_MULTI_WORD_PROG_LEN; i += 4)
                g_auPageBuff[i / 4] = u32Maddr + i;

            i = (uint32_t)FMC_WriteMultiple(u32Maddr, g_auPageBuff, FMC_MULTI_WORD_PROG_LEN);

            if(i <= 0)
            {
                printf("FMC_WriteMultiple failed: %d\n", i);
                u32Err = 1;
                goto err_out;
            }
            printf("programmed length = %d\n", i);

        }
        printf("    [OK]\n");

        printf("    Verify...");

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
            g_auPageBuff[i / 4] = u32Addr + i;

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
        {
            if(FMC_Read(u32Addr + i) != g_auPageBuff[i / 4])
            {
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", u32Addr + i, g_auPageBuff[i / 4], FMC_Read(u32Addr + i));
                u32Err = 1;
                goto err_out;
            }
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", u32Addr+i);
                u32Err = 1;
                goto err_out;
            }
        }
        printf("[OK]\n");
    }

    printf("\n\nMulti-word program demo done.\n");

err_out:

    if(u32Err) printf("\n\nERROR!\n");
    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


