/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define GPIO_P0_TO_P15      0xFFFF

void SYS_Disable_AnalogPORCircuit(void);
void PowerDownFunction(void);
void GPB_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Function for diasble internal analog POR circuit                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Disable_AnalogPORCircuit(void)
{
    /* Disable POR function */
    SYS->PORCTL1 = 0x5AA5;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_PD);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

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

    /* To check if PB.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
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
    uint32_t u32Config[4];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Disable Tamper function from Config3 */
    FMC_Open();
    FMC_ENABLE_CFG_UPDATE();

    u32Config[0] = FMC_Read(FMC_USER_CONFIG_0);
    u32Config[1] = FMC_Read(FMC_USER_CONFIG_1);
    u32Config[2] = FMC_Read(FMC_USER_CONFIG_2);
    u32Config[3] = FMC_Read(FMC_USER_CONFIG_3);

    if(((u32Config[3] & 0xFFFF0000) != 0x5AA50000))
    {
        u32Config[3] = ((u32Config[3] & (~0xFFFF0000)) | 0x5AA50000);

        FMC_Erase(FMC_USER_CONFIG_0);

        FMC_Write(FMC_USER_CONFIG_0, u32Config[0]);
        FMC_Write(FMC_USER_CONFIG_1, u32Config[1]);
        FMC_Write(FMC_USER_CONFIG_2, u32Config[2]);
        FMC_Write(FMC_USER_CONFIG_3, u32Config[3]);

        SYS_ResetChip();
        while(1);
    }

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code   |\n");
    printf("+-------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Disable Tamper function from config3                                |\n");
    printf("|  3. Configure all GPIO as Quasi-bidirectional Mode                      |\n");
    printf("|  4. Disable LVR                                                         |\n");
    printf("|  5. Disable analog function, e.g. POR module                            |\n");
    printf("|  6. Set main voltage regulator to DCDC mode                             |\n");
    printf("|  7. Disable unused SRAM                                                 |\n");
    printf("|  8. Disable unused clock, e.g. LIRC                                     |\n");
    printf("|  9. Disable unused crypto power switch                                  |\n");
    printf("|  10. Enter to Power-Down                                                |\n");
    printf("|  11. Wait for PB.3 rising-edge interrupt event to wake-up the MCU       |\n");
    printf("+-------------------------------------------------------------------------+\n\n");

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set function pin to GPIO mode expect UART pin to print message */
    SYS->GPA_MFPH = 0;
    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPL = (UART0_RXD_PA6 | UART0_TXD_PA7);
    SYS->GPB_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPF_MFPL = 0;
    SYS->GPF_MFPL = 0;
    SYS->GPG_MFPH = 0;
    SYS->GPG_MFPL = 0;
    SYS->GPH_MFPH = 0;
    SYS->GPH_MFPL = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Disable LVR */
    SYS_DISABLE_LVR();

    /* Turn off internal analog POR circuit */
    SYS_Disable_AnalogPORCircuit();

    /* Set main voltage regulator to DCDC mode */
    SYS_SetPowerRegulator(SYS_PLCTL_MVRS_DCDC);

    /* Disable unused SRAM power except SRAM bank0. SRAM bank0 is used to download code. */
    SYS->SRAMPC0 = 0x2AAAA800;
    SYS->SRAMPC1 = 0x2AAA00AA;

    /* Disable unused clock */
    CLK->PWRCTL &= ~CLK_PWRCTL_LIRCEN_Msk;

    /* Disable unused crypto power switch */
    SYS->PSWCTL = 0;

    /* Enter to Power-down mode */
    printf("Enter to Power-Down ......\n");
    PowerDownFunction();

    /* Waiting for PB.3 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);

}
