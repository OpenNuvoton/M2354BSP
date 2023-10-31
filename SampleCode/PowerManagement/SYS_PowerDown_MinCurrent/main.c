/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> PD
//      <1=> LLPD
//      <2=> FWPD
//      <3=> ULLPD
//      <4=> SPD
//      <6=> DPD
*/
#define SET_PDMSEL    0

/*
// <o0> Voltage Regulator
//      <0=> LDO
//      <1=> DCDC
*/
#define SET_MVR       1

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       1

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


#define GPIO_P0_TO_P15      0xFFFF


void PowerDownFunction(void);
void GPC_IRQHandler(void);
int32_t LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void GpioPinSettingRTC(void);
void SYS_Init(void);
void UART0_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL<<CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
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

    /* To check if PC.0 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PC, BIT0))
    {
        GPIO_CLR_INT_FLAG(PC, BIT0);
        printf("PC.0 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        u32temp = PC->INTSRC;
        PC->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

int32_t LvrSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LVR == 0)
    {
        /* Disable LVR and wait for LVR stable */
        SYS_DISABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (SYS->BODCTL & SYS_BODCTL_STB_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR stable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LVR and wait for LVR stable */
        SYS_ENABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (SYS->BODCTL & SYS_BODCTL_STB_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR stable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
        SYS->PORCTL1 = 0x5AA5;
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
        SYS->PORCTL1 = 0;
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LIRCSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        if( CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LXT == 0)
    {
        /* Disable LXT and wait for LXT stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LXTSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LXT and wait for LXT stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        if( CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t VoltageRegulatorSetting(void)
{
    if(SET_MVR == 0)
    {
        /* Set voltage regulator to LDO mode */
        if(SYS_SetPowerRegulator(SYS_PLCTL_MVRS_LDO)==0)
        {
            printf("Set voltage regulator to LDO mode not finished!\n");
            return -1;
        }
    }
    else
    {
        /* Set voltage regulator to DCDC mode */
        if(SYS_SetPowerRegulator(SYS_PLCTL_MVRS_DCDC)==0)
        {
            printf("Set voltage regulator to DCDC mode not finished!\n");
            return -1;
        }
    }

    return 0;
}

void GpioPinSettingRTC(void)
{
    /* Set PF.4~PF.11 as Quasi mode output high by RTC control */
    RTC->GPIOCTL1 = RTC_GPIOCTL1_DOUT7_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE7_Pos) |
                    RTC_GPIOCTL1_DOUT6_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE6_Pos) |
                    RTC_GPIOCTL1_DOUT5_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE5_Pos) |
                    RTC_GPIOCTL1_DOUT4_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE4_Pos);
    RTC->GPIOCTL0 = RTC_GPIOCTL0_DOUT3_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE3_Pos) |
                    RTC_GPIOCTL0_DOUT2_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE2_Pos) |
                    RTC_GPIOCTL0_DOUT1_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE1_Pos) |
                    RTC_GPIOCTL0_DOUT0_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE0_Pos);
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
    uint32_t u32TimeOutCnt, u32PMUSTS, u32Config[4];

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

    /* Clear SPD/DPD mode wake-up status for entering SPD/DPD mode again */
    u32PMUSTS = CLK->PMUSTS;
    if( u32PMUSTS )
    {
        /* Release I/O hold status for SPD mode */
        CLK->IOPDCTL = 1;

        /* Clear SPD/DPD mode wake-up status */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(CLK->PMUSTS)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for SPD/DPD mode wake-up status is cleared time-out!\n");
                goto lexit;
            }
        }
    }

    /* Check SPD/DPD mode PC.0 falling-edge wake-up event */
    if( u32PMUSTS & (CLK_PMUSTS_PINWK_Msk|CLK_PMUSTS_GPCWK_Msk) )
    {
        printf("System waken-up done.\n\n");
        while(1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PC.0 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Disable Tamper function from config3                          |\n");
    printf("|  3. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  4. Disable analog function, e.g. POR module                      |\n");
    printf("|  5. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  6. Set main voltage regulator to DCDC mode                       |\n");
    printf("|  7. Disable unused crypto power switch                            |\n");
    printf("|  8. Disable unused SRAM                                           |\n");
    printf("|  9. Enter to Power-Down                                           |\n");
    printf("| 10. Wait for PC.0 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /*
        To measure Power-down current:
        On NuMaker-M2354 V1.1 board, remove components, e.g. Nu-Link2-Me, R64, R65 and U7 WIFI Module.
        Remove R16 and then user can measure target chip power consumption by AMMETER connector.
    */

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFPL = UART0_TXD_PA7;
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPF_MFPL = 0;
    SYS->GPF_MFPH = 0;
    SYS->GPG_MFPL = 0;
    SYS->GPG_MFPH = 0;
    SYS->GPH_MFPL = 0;
    SYS->GPH_MFPH = 0;

    /*
        Configure all GPIO as Quasi-bidirectional Mode. They are default output high.

        On NuMaker-M2354 V1.1 board, configure GPIO as input mode pull-down if they have pull-down resistor outside:
        PA.12(USB_VBUS),
        PB.15(USB_VBUS_EN),
        PC.8(URTS_ESP12).
    */

    GPIO_SetMode(PA, 0xEFFF, GPIO_MODE_QUASI);
    GPIO_SetMode(PA, BIT12, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT12, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PB, 0x7FFF, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PB, BIT15, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PC, 0xFEFF, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT8, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PC, BIT8, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* LVR setting */
    if( LvrSetting() < 0 ) goto lexit;

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) goto lexit;

    /* LXT setting */
    if( LxtSetting() < 0 ) goto lexit;

    /* Voltage regulator setting */
    if( VoltageRegulatorSetting() < 0 ) goto lexit;

    /* Disable unused crypto power switch */
    SYS->PSWCTL = 0;

    /*
        Disable unused SRAM power except SRAM bank0. SRAM bank0 is used to execute and download code.
        Set SRAM power mode:
            SRAM bank0 as Retention mode.
            SRAM bank1, SRAM bank2 and peripherals SRAM as Power shut down mode.
    */
    SYS->SRAMPC0 = 0x2AAAA800;
    SYS->SRAMPC1 = 0x2AAA00AA;

    /* Wake-up source configuration */
    if( ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD ) ||
        ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD ) ||
        ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD ) ||
        ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_ULLPD ) )
    {
        /* Configure PC.0 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
        GPIO_EnableInt(PC, 0, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPC_IRQn);
    }
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD )
    {
        /* Enable wake-up pin PC.0 falling edge wake-up at SPD mode */
        CLK_EnableSPDWKPin(2, 0, CLK_SPDWKPIN_FALLING, CLK_SPDWKPIN_DEBOUNCEDIS);

        /* Set PF.4~PF.11 I/O state by RTC control to prevent floating */
        if( CLK->APBCLK0 & CLK_APBCLK0_RTCCKEN_Msk ) GpioPinSettingRTC();
    }
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD )
    {
        /* Enable wake-up pin PC.0 falling edge wake-up at DPD mode. PC.0 would be input mode floating at DPD mode. */
        CLK_EnableDPDWKPin(CLK_DPDWKPIN_FALLING);

        /* Set PF.4~PF.11 I/O state by RTC control to prevent floating */
        if( CLK->APBCLK0 & CLK_APBCLK0_RTCCKEN_Msk ) GpioPinSettingRTC();
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    /* Enter to Power-down mode */
    if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD )         printf("Enter to PD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD )  printf("Enter to LLPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD )  printf("Enter to FWPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_ULLPD ) printf("Enter to ULLPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD )   printf("Enter to SPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD )   printf("Enter to DPD Power-Down ......\n");

    PowerDownFunction();

    /* Waiting for PC.0 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while(1);
}
