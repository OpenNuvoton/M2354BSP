/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A simple LoRa slave demo for NuMaker board.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"
#include <string.h>
#include "sx1276.h"
#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"

#define RF_FREQ_433MHz      434000000
#define RF_FREQ_470MHz      470000000
#define RF_FREQ_780MHz      780000000
#define RF_FREQ_868MHz      868000000
#define RF_FREQ_915MHz      915000000

#define RF_FREQ             RF_FREQ_433MHz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            3000
#define BUFFER_SIZE                                 64 // Define the payload size here

static uint16_t s_au16BufferSize = BUFFER_SIZE;		// RF buffer size
static uint8_t s_au8Buffer[BUFFER_SIZE];			// RF buffer
static uint32_t s_u32RxCount = 1;

States_t State = LOWPOWER;

int8_t g_i8RssiValue = 0;
int8_t g_i8SnrValue = 0;

const uint8_t c_au8SlaveMsg[] = "This message was sent from slave.";

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB8_Msk | UART0_TXD_PB9_Msk))) | UART0_RXD_PB8 | UART0_TXD_PB9;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}


/* Main */
int main( void )
{
    uint8_t i;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                           SPI_LoRa_Slave                         |\n");
    printf("+------------------------------------------------------------------+\n");
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    // Target board initialization
    BoardInitMcu();
    BoardInitPeriph();

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQ);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

    Radio.Rx(RX_TIMEOUT_VALUE);

    while(1)
    {
        // Tick the RTC to execute callback in context of the main loop (in stead of the IRQ)
        TimerProcess();

        switch(State)
        {
        case RX:
            // Send the next message frame
            memcpy(s_au8Buffer, c_au8SlaveMsg, sizeof(c_au8SlaveMsg));
            DelayMs(1);
            Radio.Send(s_au8Buffer, s_au16BufferSize);
            State = LOWPOWER;
            break;
        case TX:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            // Send the next message frame
            memcpy(s_au8Buffer, c_au8SlaveMsg, sizeof(c_au8SlaveMsg));
            DelayMs(1);
            Radio.Send(s_au8Buffer, s_au16BufferSize);
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }
    }
}

void OnTxDone(void)
{
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    s_au16BufferSize = size;
    memcpy(s_au8Buffer, payload, s_au16BufferSize);
    g_i8RssiValue = rssi;
    g_i8SnrValue = snr;
    printf("S %s, RSSI=%d\n", s_au8Buffer, g_i8RssiValue);
    State = RX;
}

void OnTxTimeout(void)
{
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    State = RX_ERROR;
}
