/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the BME680 sensor data on RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"
#include "m2354_bme680.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32RTCINT = 0;

uint32_t Initial_LCD(void);
void SYS_Init(void);
void UART_Init(void);
void RTC_IRQHandler(void);
void RTC_Init(void);


#define LCD_ALPHABET_NUM    7
static S_LCD_CFG_T g_LCDCfg =
{
    __LIRC,                     /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,           /*!< COM duty */
    LCD_BIAS_LV_1_4,            /*!< Bias level */
    60,                         /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_A_NORMAL, /*!< Waveform type */
    LCD_DISABLE_ALL_INT,        /*!< Interrupt source */
    LCD_LOW_DRIVING_AND_BUF_ON, /*!< Driving mode */
    LCD_VOLTAGE_SOURCE_CP,      /*!< Voltage source */
};

static void Configure_LCD_Pins(void)
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
                    (LCD_SEG24_PA0 | LCD_SEG25_PA1 | LCD_SEG26_PA2 | LCD_SEG27_PA3 | LCD_SEG28_PA4 | LCD_SEG29_PA5);
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

uint32_t Initial_LCD(void)
{
    uint32_t frame_rate;
    
    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Configure LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL1_LCDSEL_LIRC, 0);
    
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
    frame_rate = LCD_Open(&g_LCDCfg);

    /* Enable charge pump clock MIRC and output voltage level 3 for 3.2V */
    CLK_EnableXtalRC(CLK_PWRCTL_MIRCEN_Msk);
    CLK_EnableModuleClock(LCDCP_MODULE);
    CLK_SetModuleClock(LCDCP_MODULE, CLK_CLKSEL1_LCDCPSEL_MIRC, 0);
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_3);

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();
    
    return frame_rate;
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
    SET_UART0_RXD_PB8();
    SET_UART0_TXD_PB9();
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

/**
 * @brief       IRQ Handler for RTC Interrupt
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);

        g_u32RTCINT++;
    }
}

void RTC_Init(void)
{
    /* Enable LXT-32KHz */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);
    
    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32LCDFPS;
    char text[LCD_ALPHABET_NUM + 1] = "";
    struct bme680_dev bme680;
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    uint16_t meas_period; /* Get the total measurement duration so as to sleep or wait till the measurement is complete */
    uint32_t pressure, humidity, temperature;
    S_RTC_TIME_DATA_T sInitTime, sReadRTC;
    uint32_t u32TimeData;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();
    
    /* Init RTC */
    RTC_Init();
    
    /* Open RTC and start counting */
    sInitTime.u32Year       = 2021;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 12;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_FRIDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;
    RTC_Open(&sInitTime);

    /* Enable RTC tick interrupt, one RTC tick is 0.5 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_2_SEC);
    
    /* Init LCD for display */
    u32LCDFPS = Initial_LCD();
    
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    BME680 Sensor Data Display On LCD    |\n");
    printf("+-----------------------------------------+\n");
    printf("\tLCD working frame rate is %dHz on Type-%c.\n\n", u32LCDFPS, (g_LCDCfg.u32WaveformType == LCD_PCTL_TYPE_Msk) ? 'B' : 'A');
    
    
    /* Show default RTC time */
    u32TimeData = (sReadRTC.u32Hour * 100) + sReadRTC.u32Minute;
    LCDLIB_PrintNumber(ZONE_TIME_DIGIT, u32TimeData);
    LCDLIB_SetSymbol(SYMBOL_COL_47, 1);
    
    /* Show logo and version */
    LCDLIB_SetSymbol(SYMBOL_NVT_14, 1);
    LCDLIB_PrintNumber(ZONE_VER_DIGIT, 101000);
    LCDLIB_SetSymbol(SYMBOL_VERSION_37, 1);
    LCDLIB_SetSymbol(SYMBOL_P_67, 1);
    LCDLIB_SetSymbol(SYMBOL_P_68, 1);    
        
    /* Show default pressure, hPa */
    sprintf(text, "%4dhPa", 0000);
    LCDLIB_Printf(ZONE_MAIN_DIGIT, text);

    /* Show default humidity, %rH */
    LCDLIB_PrintNumberEx(ZONE_PPM_DIGIT, 00, 2);
    LCDLIB_SetSymbol(SYMBOL_PERCENTAGE_30, 1);

    /* Show default temperature, degC */
    LCDLIB_PrintNumberEx(ZONE_TEMP_DIGIT, 00, 2);
    LCDLIB_SetSymbol(SYMBOL_TEMP_C_35, 1);    

    /* Init BME680 */
	if(m2354_bme680_init(&bme680) != BME680_OK) {
		printf("Initialize BME680 ..... [FAIL]\n");
        
        sprintf(text, "%s", "FAIL 1");
        LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
        goto lexit;
	}
	
	if(m2354_bme680_config(&bme680) != BME680_OK) {
		printf("Config BME680 ..... [FAIL]\n");
        
        sprintf(text, "%s", "FAIL 2");
        LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
        goto lexit;
	}
	
	bme680_get_profile_dur(&meas_period, &bme680);
        
    while(1)
    {
        m2354_delay_ms(meas_period);

        rslt = bme680_get_sensor_data(&data, &bme680);
        if(rslt != BME680_OK)
        {
            sprintf(text, "%s", "FAIL 3");
            LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
            goto lexit;
        }
        
        /* Trigger the next measurement if you would like to read data out continuously */
        if(bme680.power_mode == BME680_FORCED_MODE) 
        {
            rslt = bme680_set_sensor_mode(&bme680);
            if(rslt != BME680_OK)
            {
                sprintf(text, "%s", "FAIL 4");
                LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
                goto lexit;
            }
        }
            
        printf("T: %.2f(degC), P: %.2f(hPa), H: %.2f(%%rH)", 
                (double)(data.temperature / 100.0f),
                (double)(data.pressure / 100.0f), 
                (double)(data.humidity / 1000.0f));                    
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            printf(", G: %d(ohms)", data.gas_resistance);
        //printf("\r\n");
        printf("\r");
                
        pressure    = data.pressure / 100;
        humidity    = data.humidity / 1000;
        temperature = data.temperature / 100;

        
        /* Show current RTC time and BME680 data on LCD */
        if(g_u32RTCINT == 1)
            LCDLIB_SetSymbol(SYMBOL_COL_47, 1);
        else
            LCDLIB_SetSymbol(SYMBOL_COL_47, 0);
        
        while(g_u32RTCINT >= 2)
        {
            g_u32RTCINT = 0;
                    
            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            //printf("\n\t%d/%02d/%02d %02d:%02d:%02d\n", sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            
            /* Show RTC time */
            u32TimeData = (sReadRTC.u32Hour * 100) + sReadRTC.u32Minute;
            LCDLIB_PrintNumber(ZONE_TIME_DIGIT, u32TimeData);
                                       
            /* Show humidity, ? %rH */
            LCDLIB_PrintNumberEx(ZONE_PPM_DIGIT, humidity, 2);

            /* Show temperature, ? degC */
            LCDLIB_PrintNumberEx(ZONE_TEMP_DIGIT, temperature, 2);
            
            /* Show pressure, ? hPa */
            sprintf(text, "%4dhPa", pressure);
            LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
        }
    }

lexit:

    while(1);
}
