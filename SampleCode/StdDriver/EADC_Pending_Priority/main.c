/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger multiple sample modules and got conversion results in order of priority.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag, g_u32EadcInt2Flag, g_u32EadcInt3Flag;

static uint32_t g_au32IntModule[4];    /* save the sample module number for ADINT0~3 */
static volatile uint32_t g_au32IntSequence[4];  /* save the interrupt sequence for ADINT0~3 */
static volatile uint32_t g_u32IntSequenceIndex;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void EADC0_IRQHandler(void);
void EADC1_IRQHandler(void);
void EADC2_IRQHandler(void);
void EADC3_IRQHandler(void);


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

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set EADC clock divider as 12 */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(12));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pins for EADC channels. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB6MFP_Msk) | SYS_GPB_MFPL_PB6MFP_EADC0_CH6;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB7MFP_Msk) | SYS_GPB_MFPL_PB7MFP_EADC0_CH7;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB8MFP_Msk) | SYS_GPB_MFPH_PB8MFP_EADC0_CH8;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB9MFP_Msk) | SYS_GPB_MFPH_PB9MFP_EADC0_CH9;

    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT9 | BIT8 | BIT7 | BIT6 | BIT0);

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
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint8_t  u8Option;
    int32_t  i32ConversionData, i;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      EADC Pending Priority sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while(1)
    {
        printf("Select test items:\n");
        printf("  [1] Assign interrupt ADINT0~3 to Sample Module 0~3\n");
        printf("  [2] Assign interrupt ADINT3~0 to Sample Module 0~3\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = (uint8_t)getchar();

        if(u8Option == '1')
        {
            g_au32IntModule[0] = 0;  /* Assign ADINT0 to Sample module 0 */
            g_au32IntModule[1] = 1;  /* Assign ADINT1 to Sample module 1 */
            g_au32IntModule[2] = 2;  /* Assign ADINT2 to Sample module 2 */
            g_au32IntModule[3] = 3;  /* Assign ADINT3 to Sample module 3 */
        }
        else if(u8Option == '2')
        {
            g_au32IntModule[0] = 3;  /* Assign ADINT0 to Sample module 3 */
            g_au32IntModule[1] = 2;  /* Assign ADINT1 to Sample module 2 */
            g_au32IntModule[2] = 1;  /* Assign ADINT2 to Sample module 1 */
            g_au32IntModule[3] = 0;  /* Assign ADINT3 to Sample module 0 */
        }
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 6);
        EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 7);
        EADC_ConfigSampleModule(EADC, 2, EADC_SOFTWARE_TRIGGER, 8);
        EADC_ConfigSampleModule(EADC, 3, EADC_SOFTWARE_TRIGGER, 9);

        /* Clear the A/D ADINTx interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << g_au32IntModule[0]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << g_au32IntModule[1]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << g_au32IntModule[2]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << g_au32IntModule[3]);

        NVIC_EnableIRQ(EADC0_IRQn);
        NVIC_EnableIRQ(EADC1_IRQn);
        NVIC_EnableIRQ(EADC2_IRQn);
        NVIC_EnableIRQ(EADC3_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IntSequenceIndex = 0;
        g_u32EadcInt0Flag = 0;
        g_u32EadcInt1Flag = 0;
        g_u32EadcInt2Flag = 0;
        g_u32EadcInt3Flag = 0;

        /* Start EADC conversion for sample module 0 ~ 3 at the same time */
        EADC_START_CONV(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        /* Wait all EADC interrupt (g_u32EadcIntxFlag will be set at EADC_INTx_IRQHandler() function) */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((g_u32EadcInt0Flag == 0) || (g_u32EadcInt1Flag == 0) ||
                (g_u32EadcInt2Flag == 0) || (g_u32EadcInt3Flag == 0))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EADC interrupt time-out!\n");
                return;
            }
        }

        /* Get the conversion result of the sample module */
        printf("The ADINTx interrupt sequence is:\n");

        for(i = 0; i < 4; i++)
        {
            i32ConversionData = EADC_GET_CONV_DATA(EADC, g_au32IntModule[i]);
            printf("ADINT%d: #%d, Module %d, Conversion result: 0x%X (%d)\n", i, g_au32IntSequence[i], g_au32IntModule[i], i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << g_au32IntModule[0]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << g_au32IntModule[1]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << g_au32IntModule[2]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << g_au32IntModule[3]);

        NVIC_DisableIRQ(EADC0_IRQn);
        NVIC_DisableIRQ(EADC1_IRQn);
        NVIC_DisableIRQ(EADC2_IRQn);
        NVIC_DisableIRQ(EADC3_IRQn);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC);
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    g_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Save the interrupt sequence about ADINT0 */
    g_au32IntSequence[0] = g_u32IntSequenceIndex++;
}

void EADC1_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Save the interrupt sequence about ADINT1 */
    g_au32IntSequence[1] = g_u32IntSequenceIndex++;
}

void EADC2_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);

    /* Save the interrupt sequence about ADINT2 */
    g_au32IntSequence[2] = g_u32IntSequenceIndex++;
}

void EADC3_IRQHandler(void)
{
    g_u32EadcInt3Flag = 1;
    /* Clear the A/D ADINT3 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);

    /* Save the interrupt sequence about ADINT3 */
    g_au32IntSequence[3] = g_u32IntSequenceIndex++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);
    NVIC_DisableIRQ(EADC1_IRQn);
    NVIC_DisableIRQ(EADC2_IRQn);
    NVIC_DisableIRQ(EADC3_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
