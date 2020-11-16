/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to print text on RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_ALPHABET_NUM    7
static S_LCD_CFG_T g_LCDCfg = {
    __LIRC,                     /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,           /*!< COM duty */
    LCD_BIAS_LV_1_4,            /*!< Bias level */
    30,                         /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_A_NORMAL, /*!< Waveform type */
    LCD_DISABLE_ALL_INT,        /*!< Interrupt source */
    LCD_LOW_DRIVING_AND_BUF_ON, /*!< Driving mode */
    LCD_VOLTAGE_SOURCE_CP,      /*!< Voltage source */
};
    
void Configure_LCD_Pins(void);
void SYS_Init(void);
void UART_Init(void);


void Configure_LCD_Pins(void)
{
    /*
        Summary of LCD pin usage:
            COM 0~5   : PC.0, PC.1, PC.2, PC.3, PC.4, PC.5
            COM 6~7   : PD.8, PD.9
            SEG 0     : PD.14
            SEG 1~4   : PH.11, PH.10, PH.9, PH.8
            SEG 5~12  : PE.0, PE.1, PE.2, PE.3, PE.4, PE.5, PE.6, PE.7
            SEG 13~14 : PD.6, PD.7
            SEG 15~21 : PG.15, PG.14, PG.13, PG.12, PG.11, PG.10, PG.9
            SEG 22~23 : PE.15, PE.14
            SEG 24~29 : PA.0, PA.1, PA.2, PA.3, PA.4, PA.5
            SEG 30~32 : PE.10, PE.9, PE.8
            SEG 33~36 : PH.7, PH.6, PH.5, PH.4
            SEG 37~39 : PG.4, PG.3, PG.2
    */
    
    /* COM 0~5 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & 
                    ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | 
                      SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (LCD_COM0_PC0 | LCD_COM1_PC1 | LCD_COM2_PC2 | LCD_COM3_PC3 | LCD_COM4_PC4 | LCD_COM5_PC5);
    /* COM 6~7 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD8MFP_Msk | SYS_GPD_MFPH_PD9MFP_Msk)) |
                    (LCD_COM6_PD8 | LCD_COM7_PD9);
    
    /* SEG 0 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~SYS_GPD_MFPH_PD14MFP_Msk) | LCD_SEG0_PD14;
    /* SEG 1~4 */
    SYS->GPH_MFPH = (SYS->GPH_MFPH & ~(SYS_GPH_MFPH_PH11MFP_Msk | SYS_GPH_MFPH_PH10MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk | SYS_GPH_MFPH_PH8MFP_Msk)) |
                    (LCD_SEG1_PH11 | LCD_SEG2_PH10 | LCD_SEG3_PH9 | LCD_SEG4_PH8);
    /* SEG 5~12 */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & 
                    ~(SYS_GPE_MFPL_PE0MFP_Msk | SYS_GPE_MFPL_PE1MFP_Msk | SYS_GPE_MFPL_PE2MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk |
                      SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk | SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk)) |
                    (LCD_SEG5_PE0 | LCD_SEG6_PE1 | LCD_SEG7_PE2 | LCD_SEG8_PE3 |
                     LCD_SEG9_PE4 | LCD_SEG10_PE5 | LCD_SEG11_PE6 | LCD_SEG12_PE7);
    /* SEG 13~14 */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD7MFP_Msk)) | (LCD_SEG13_PD6 | LCD_SEG14_PD7);
    /* SEG 15~21 */
    SYS->GPG_MFPH = (SYS->GPG_MFPH & 
                    ~(SYS_GPG_MFPH_PG15MFP_Msk | SYS_GPG_MFPH_PG14MFP_Msk | SYS_GPG_MFPH_PG13MFP_Msk | SYS_GPG_MFPH_PG12MFP_Msk |
                      SYS_GPG_MFPH_PG11MFP_Msk | SYS_GPG_MFPH_PG10MFP_Msk | SYS_GPG_MFPH_PG9MFP_Msk)) |
                    (LCD_SEG15_PG15 | LCD_SEG16_PG14 | LCD_SEG17_PG13 | LCD_SEG18_PG12 |
                     LCD_SEG19_PG11 | LCD_SEG20_PG10 | LCD_SEG21_PG9);
    /* SEG 22~23 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE15MFP_Msk | SYS_GPE_MFPH_PE14MFP_Msk)) | (LCD_SEG22_PE15 | LCD_SEG23_PE14);
    /* SEG 24~29 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & 
                    ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk |
                      SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk)) |
                    (LCD_SEG24_PA0 | LCD_SEG25_PA1 | LCD_SEG26_PA2 | LCD_SEG27_PA3 | LCD_SEG28_PA4 |LCD_SEG29_PA5);
    /* SEG 30~32 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE10MFP_Msk | SYS_GPE_MFPH_PE9MFP_Msk | SYS_GPE_MFPH_PE8MFP_Msk)) | 
                    (LCD_SEG30_PE10 | LCD_SEG31_PE9 | LCD_SEG32_PE8);
    /* SEG 33~36 */
    SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH7MFP_Msk | SYS_GPH_MFPL_PH6MFP_Msk | SYS_GPH_MFPL_PH5MFP_Msk | SYS_GPH_MFPL_PH4MFP_Msk)) |
                    (LCD_SEG33_PH7 | LCD_SEG34_PH6 | LCD_SEG35_PH5 | LCD_SEG36_PH4);
    /* SEG 37~39 */
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~(SYS_GPG_MFPL_PG4MFP_Msk | SYS_GPG_MFPL_PG3MFP_Msk | SYS_GPG_MFPL_PG2MFP_Msk)) |
                    (LCD_SEG37_PG4 | LCD_SEG38_PG3 | LCD_SEG39_PG2);
}

void SYS_Init(void)
{
    /* Enable all GPIO and SRAM clock */
    CLK->AHBCLK |= (CLK_AHBCLK_TRACECKEN_Msk | CLK_AHBCLK_SRAM0CKEN_Msk | CLK_AHBCLK_SRAM1CKEN_Msk | CLK_AHBCLK_SRAM2CKEN_Msk |
                    CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk | CLK_AHBCLK_GPDCKEN_Msk |
                    CLK_AHBCLK_GPECKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk | CLK_AHBCLK_GPGCKEN_Msk | CLK_AHBCLK_GPHCKEN_Msk);
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(TRACE_CLK_PE12_Msk | TRACE_DATA0_PE11_Msk | TRACE_DATA1_PE10_Msk | TRACE_DATA2_PE9_Msk | TRACE_DATA3_PE8_Msk)) |
                    (TRACE_CLK_PE12 | TRACE_DATA0_PE11 | TRACE_DATA1_PE10 | TRACE_DATA2_PE9 | TRACE_DATA3_PE8);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    
    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Configure UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Configure LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL1_LCDSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32ActiveFPS, idx = 0;
    char input, text[LCD_ALPHABET_NUM] = "";

    /* Unlock protected registers */
    SYS_UnlockReg();
        
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    LCD Print Text Sample Code    |\n");
    printf("+----------------------------------+\n\n");

    printf("LCD configurations:\n");
    printf(" * Clock source is LIRC\n");
    printf(" * 8 COM, 40 SEG and 1/4 Bias\n");
    printf(" * Driving waveform is Type-%c\n", (g_LCDCfg.u32WaveformType==LCD_PCTL_TYPE_Msk)?'B':'A');
    printf(" * Target frame rate is %dHz\n\n", g_LCDCfg.u32Framerate);
    
    /* COM 0~5 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (LCD_COM0_PC0 | LCD_COM1_PC1 | LCD_COM2_PC2 | LCD_COM3_PC3 | LCD_COM4_PC4 | LCD_COM5_PC5);
    /* COM 6~7 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD8MFP_Msk | SYS_GPD_MFPH_PD9MFP_Msk)) |
                    (LCD_COM6_PD8 | LCD_COM7_PD9);
    
    /* Configure LCD multi-function pins */
    Configure_LCD_Pins();
    
    /* Reset LCD module */
    SYS_ResetModule(LCD_RST);
        
    /* LCD Initialize and calculate real frame rate */
    u32ActiveFPS = LCD_Open(&g_LCDCfg);
    printf("Working frame rate is %dHz on Type-%c.\n\n", u32ActiveFPS, (g_LCDCfg.u32WaveformType==LCD_PCTL_TYPE_Msk)?'B':'A');
    
    /* Enable charge pump clock MIRC and output voltage level 2 for 3.0V */
    CLK_EnableXtalRC(CLK_PWRCTL_MIRCEN_Msk);
    CLK_EnableModuleClock(LCDCP_MODULE);
    CLK_SetModuleClock(LCDCP_MODULE, CLK_CLKSEL1_LCDCPSEL_MIRC, 0);
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_2);

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();
    
    while(1)
    {
        idx = 0;
        strcpy(text, "");   // clear buffer

        printf("Input text: ");
        while(1)
        {
            input = (char)getchar();
            printf("%c", input);
            if(input == 0xD) // "ENTER" key to exit current input
                break;
            
            strcat(text, &input);
            idx++;
            if(idx >= LCD_ALPHABET_NUM)
                break;
        }
        printf("\n");
        printf("Show: %s\n\n", text);

        /* Set specified text on LCD */
        LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
