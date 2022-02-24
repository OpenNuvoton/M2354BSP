/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to select SRAM power mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"




extern int IsDebugFifoEmpty(void);
static volatile uint8_t s_u8IsINTEvent;

void WDT_IRQHandler(void);
void PowerDownFunction(void);
void SYS_Init(void);
void UART0_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{

    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    s_u8IsINTEvent = 1;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
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

    /* Enable UART0, WDT and CRC module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(CRC_MODULE);

    /* Select UART0 and WDT module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

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
    uint32_t au32SRAMCheckSum[18] = {0};
    uint32_t u32SRAMSize = 16384;
    uint32_t u32Idx, u32Addr, u32SRAMStartAddr = 0;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|       SRAM Power Mode Sample Code     |\n");
    printf("+---------------------------------------+\n\n");

    /*
        SRAM power mode can select as normal mode, retention mode and power shut down mode.
        SRAM is able to access only in normal mode and SRAM clock have to be enabled.
        The unused SRAM can be set in power shut down mode, but SRAM data will not kept.
        This sample code will set SRAM bank1 (0x2008000 - 0x20027FFF) in normal mode,
        and set SRAM bank2 (0x20028000 - 0x2003_FFFF) in power shut down mode.
        The SRAM bank2 checksum will be different after setting in power shut down mode.
    */

    /* Unlock protected registers before setting SRAM power mode */
    SYS_UnlockReg();

    /* Select SRAM power mode:
       SRAM bank1 is in normal mode.
       SRAM bank2 is in normal mode.
    */
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM0_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM1_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM2_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM3_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM4_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM5_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM6_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM7_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM0_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM1_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM2_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM3_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM4_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM5_Msk, SYS_SRAMPC1_SRAM_NORMAL);

    /* Calculate SRAM checksum */
    printf("Calculate SRAM checksum before Power-down:\n\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20008000;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < 14; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < (u32SRAMStartAddr + u32SRAMSize); u32Addr += 4)
        {
            CRC_WRITE_DATA(CRC, inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[0]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[1]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[2]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 5 Checksum [0x%08X]\n",   au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 6 Checksum [0x%08X]\n",   au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 7 Checksum [0x%08X]\n\n", au32SRAMCheckSum[7]);
    printf("SRAM Bank2 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[10]);
    printf("SRAM Bank2 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[11]);
    printf("SRAM Bank2 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[12]);
    printf("SRAM Bank2 Region 5 Checksum [0x%08X]\n\n",   au32SRAMCheckSum[13]);

    /* Select SRAM power mode:
       SRAM bank2 is in power shut down mode mode.
    */
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM0_Msk, SYS_SRAMPC0_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM1_Msk, SYS_SRAMPC0_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM2_Msk, SYS_SRAMPC1_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM3_Msk, SYS_SRAMPC1_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM4_Msk, SYS_SRAMPC1_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM5_Msk, SYS_SRAMPC1_SRAM_POWER_SHUT_DOWN);

    /* Enter to Power-down mode and wake-up by WDT interrupt */
    printf("Enter to Power-down mode ... ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(s_u8IsINTEvent == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!");
            break;
        }
    }
    printf("wake-up!\n\n");

    /* Select SRAM power mode:
       SRAM bank2 is in normal mode.
    */
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM0_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM1_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM2_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM3_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM4_Msk, SYS_SRAMPC1_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC1_SRAM2PM5_Msk, SYS_SRAMPC1_SRAM_NORMAL);

    /* Calculate SRAM checksum */
    printf("Calculate SRAM CheckSum after wake-up:\n\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20008000;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < 14; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < u32SRAMStartAddr + u32SRAMSize; u32Addr += 4)
        {
            CRC_WRITE_DATA(CRC, inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[0]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[1]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[2]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 5 Checksum [0x%08X]\n",   au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 6 Checksum [0x%08X]\n",   au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 7 Checksum [0x%08X]\n\n", au32SRAMCheckSum[7]);
    printf("SRAM Bank2 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[10]);
    printf("SRAM Bank2 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[11]);
    printf("SRAM Bank2 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[12]);
    printf("SRAM Bank2 Region 5 Checksum [0x%08X]\n\n", au32SRAMCheckSum[13]);

    while(1);

}
