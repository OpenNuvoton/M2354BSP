/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Capture the EPWM1 Channel 0 waveform by EPWM1 Channel 2, and use PDMA to transfer captured data.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint16_t g_au16Count[4];
static volatile uint32_t g_u32IsTestOver = 0;

void PDMA0_IRQHandler(void);
int32_t CalPeriodTime(EPWM_T *EPWM, uint32_t u32Ch);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x1)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & 0x2)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x1)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* g_au16Count[4] : Keep the internal counter value when input signal rising / falling  */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(EPWM_T *EPWM, uint32_t u32Ch)
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    (void)EPWM;
    (void)u32Ch;

    g_u32IsTestOver = 0;
    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA interrupt time-out!\n");
            return (-1);
        }
    }

    u16RisingTime = g_au16Count[1];

    u16FallingTime = g_au16Count[0];

    u16HighPeriod = g_au16Count[1] - g_au16Count[2];

    u16LowPeriod = (uint16_t)(0x10000 - g_au16Count[1]);

    u16TotalPeriod = (uint16_t)(0x10000 - g_au16Count[2]);

    printf("\nEPWM generate: \nHigh Period=19199 ~ 19201, Low Period=44799 ~ 44801, Total Period=63999 ~ 64001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 19199) || (u16HighPeriod > 19201) || (u16LowPeriod < 44799) || (u16LowPeriod > 44801) || (u16TotalPeriod < 63999) || (u16TotalPeriod > 64001))
    {
        printf("Capture Test Fail!!\n");
        return (-1);
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
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

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pins for EPWM */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | EPWM1_CH0_PC5;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | EPWM1_CH2_PC3;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use EPWM1 channel 2 to capture the signal from EPWM1 channel 0.\n");
    printf("  And the captured data is transferred by PDMA channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    EPWM1 channel 2(PC.3) <--> EPWM1 channel 0(PC.5)\n\n");
    printf("Use EPWM1 Channel 2(PC.3) to capture the EPWM1 Channel 0(PC.5) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start EPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the EPWM1 Channel 0 as EPWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume EPWM output frequency is 250Hz and duty ratio is 30%, user can calculate EPWM settings by follows.(up counter type)
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           EPWM clock source frequency = PLL = 96000000
           (CNR+1) = EPWM clock source frequency/prescaler/EPWM output frequency
                   = 96000000/6/250 = 64000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 63999
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 19200
           Prescale value is 5 : prescaler= 6
        */

        /* Set EPWM1 channel 0 output configuration */
        EPWM_ConfigOutputChannel(EPWM1, 0, 250, 30);

        /* Enable EPWM Output path for EPWM1 channel 0 */
        EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

        /* Enable Timer for EPWM1 channel 0 */
        EPWM_Start(EPWM1, EPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA peripheral mode form EPWM to memory                                   */
        /*--------------------------------------------------------------------------------------*/
        /* Open Channel 0 */
        PDMA_Open(PDMA0, 0x1);

        /* Transfer width is half word(16 bit) and transfer count is 4 */
        PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 4);

        /* Set source address as EPWM capture channel PDMA register(no increment) and destination address as g_au16Count array(increment) */
        PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&EPWM1->PDMACAP[1], PDMA_SAR_FIX, (uint32_t)&g_au16Count[0], PDMA_DAR_INC);

        /* Select PDMA request source as EPWM RX(EPWM1 channel 2 should be EPWM1 pair 2) */
        PDMA_SetTransferMode(PDMA0, 0, PDMA_EPWM1_P2_RX, FALSE, 0);

        /* Set PDMA as single request type for EPWM */
        PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_4);

        PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
        NVIC_EnableIRQ(PDMA0_IRQn);

        /* Enable PDMA for EPWM1 channel 2 capture function, and set capture order as falling first, */
        /* And select capture mode as both rising and falling to do PDMA transfer. */
        EPWM_EnablePDMA(EPWM1, 2, FALSE, EPWM_CAPTURE_PDMA_RISING_FALLING_LATCH);

        /*--------------------------------------------------------------------------------------*/
        /* Set the EPWM1 channel 2 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 96000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 96000000/6/250 = 64000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           62.5ns = 1/96000000/6
        */

        /* Set EPWM1 channel 2 capture configuration */
        EPWM_ConfigCaptureChannel(EPWM1, 2, 62, 0);

        /* Enable Timer for EPWM1 channel 2 */
        EPWM_Start(EPWM1, EPWM_CH_2_MASK);

        /* Enable Capture Function for EPWM1 channel 2 */
        EPWM_EnableCapture(EPWM1, EPWM_CH_2_MASK);

        /* Enable falling capture reload */
        EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN2_Msk;

        /* Wait until EPWM1 channel 2 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((EPWM1->CNT[2]) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EPWM1 channel 2 Timer start time-out!\n");
                return -1;
            }
        }

        /* Capture the Input Waveform Data */
        if( CalPeriodTime(EPWM1, 2) < 0 )
            return -1;
        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop EPWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set EPWM Timer loaded value(Period) as 0. When EPWM internal counter(CNT) reaches to 0, disable EPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set EPWM1 channel 0 loaded value as 0 */
        EPWM_Stop(EPWM1, EPWM_CH_0_MASK);

        /* Wait until EPWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((EPWM1->CNT[0] & EPWM_CNT0_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EPWM1 channel 0 Timer stop time-out!\n");
                return -1;
            }
        }

        /* Disable Timer for EPWM1 channel 0 */
        EPWM_ForceStop(EPWM1, EPWM_CH_0_MASK);

        /* Disable EPWM Output path for EPWM1 channel 0 */
        EPWM_DisableOutput(EPWM1, EPWM_CH_0_MASK);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop EPWM1 channel 2 (Recommended procedure method 1)                                                      */
        /* Set EPWM Timer loaded value(Period) as 0. When EPWM internal counter(CNT) reaches to 0, disable EPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for EPWM1 channel 2 */
        EPWM_Stop(EPWM1, EPWM_CH_2_MASK);

        /* Wait until EPWM1 channel 2 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((EPWM1->CNT[2] & EPWM_CNT2_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EPWM1 channel 2 current counter reach to 0 time-out!\n");
                return -1;
            }
        }

        /* Disable Timer for EPWM1 channel 2 */
        EPWM_ForceStop(EPWM1, EPWM_CH_2_MASK);

        /* Disable Capture Function and Capture Input path for EPWM1 channel 2 */
        EPWM_DisableCapture(EPWM1, EPWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for EPWM1 channel 2 */
        EPWM_ClearCaptureIntFlag(EPWM1, 2, EPWM_CAPTURE_INT_FALLING_LATCH);

        /* Disable PDMA NVIC */
        NVIC_DisableIRQ(PDMA0_IRQn);

        /* Close PDMA */
        PDMA_Close(PDMA0);
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
