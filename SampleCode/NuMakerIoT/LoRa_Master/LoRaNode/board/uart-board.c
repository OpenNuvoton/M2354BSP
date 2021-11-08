/****************************************************************************
 * @file     uart-board.c
 * @version  V1.10
 * @brief    Target board UART driver implementation
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "utilities.h"
#include "board-config.h"
#include "gpio.h"
#include "uart.h"
#include "uart-board.h"

#define MAX_UART_INST	2

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10

struct nu_uart_modinit_s {
    uint32_t clkidx;
    uint32_t clksrc;
    uint32_t clkdiv;
    uint32_t rsetidx;    
    IRQn_Type irq_n;
    void *var;
};

typedef struct{
	UART_T *uart_base;
	UartMode_t mode;
	Uart_t *obj;
}S_UART_HANDLE;

static const struct nu_uart_modinit_s uart_modinit_tab[] = {
    {UART0_MODULE, CLK_CLKSEL2_UART0SEL_PCLK0, MODULE_NoMsk, UART0_RST, UART0_IRQn, NULL},
    {UART1_MODULE, CLK_CLKSEL2_UART0SEL_PCLK0, MODULE_NoMsk, UART1_RST, UART1_IRQn, NULL},
    {0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

static S_UART_HANDLE s_asUARTHandle[MAX_UART_INST];


void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
	obj->UartId = 0;
	
	if(uartId >= MAX_UART_INST)
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}	

	if(uartId == UART_1)
	{
		//Using UART0 instance for UART_1 id
		s_asUARTHandle[UART_1].uart_base = UART0;

		//Set SPI multple function pin
		GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, UART0_MFP_TX_VALUE );
		GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, UART0_MFP_RX_VALUE );
	}
	else if(uartId == UART_2)
	{
		//Using UART1 instance for UART2_1 id
		s_asUARTHandle[UART_2].uart_base = UART1;

		//Set SPI multple function pin
		GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, UART1_MFP_TX_VALUE );
		GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, UART1_MFP_TX_VALUE );
	}
	else
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}
	
    CRITICAL_SECTION_BEGIN( );
	const struct nu_uart_modinit_s *modinit = &uart_modinit_tab[uartId];

   // Reset this module
    SYS_ResetModule(modinit->rsetidx);

    // Select IP clock source
    CLK_SetModuleClock(modinit->clkidx, modinit->clksrc, modinit->clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(modinit->clkidx);

	obj->UartId = uartId;
	
    CRITICAL_SECTION_END( );
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
	S_UART_HANDLE *psUARTHandle;
	const struct nu_uart_modinit_s *modinit;
	
    if( ( obj == NULL ) || (obj->UartId >= MAX_UART_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
    }

	if( wordLength == UART_9_BIT )
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}
	
	psUARTHandle = &s_asUARTHandle[obj->UartId];
	modinit = &uart_modinit_tab[obj->UartId];
	
	if( mode == TX_ONLY )
	{
		if( obj->FifoTx.Data == NULL )
		{
			assert_param( FAIL );
		}
	}
	else if( mode == RX_ONLY )
	{
		if( obj->FifoRx.Data == NULL )
		{
			assert_param( FAIL );
		}
	}
	else if( mode == RX_TX )
	{
		if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
		{
			assert_param( FAIL );
		}
	}
	else
	{
		assert_param( FAIL );
	}

	uint32_t u32DataWidth;
	uint32_t u32Parity;
	uint32_t  u32StopBits;
	
	psUARTHandle->mode = mode;

	if( wordLength == UART_8_BIT )
	{
		u32DataWidth = UART_WORD_LEN_8;
	}

	switch( stopBits )
	{
	case UART_2_STOP_BIT:
		u32StopBits = UART_STOP_BIT_2;
		break;
	case UART_1_5_STOP_BIT:
		u32StopBits = UART_STOP_BIT_1_5;
		break;
	case UART_1_STOP_BIT:
	default:
		u32StopBits = UART_STOP_BIT_1;
		break;
	}

	if( parity == NO_PARITY )
	{
		u32Parity = UART_PARITY_NONE;
	}
	else if( parity == EVEN_PARITY )
	{
		u32Parity = UART_PARITY_EVEN;
	}
	else
	{
		u32Parity = UART_PARITY_ODD;
	}

	//open uart
	UART_Open(psUARTHandle->uart_base, baudrate);

	//line configure
	UART_SetLineConfig(psUARTHandle->uart_base, baudrate, u32DataWidth, u32Parity, u32StopBits);

	psUARTHandle->obj = obj;

    //UART_SetTimeoutCnt(psUARTHandle->uart_base, 0x10); // Set Rx Time-out counter

    // Set RX FIFO Interrupt Trigger Level
    UART0->FIFO &= ~ UART_FIFO_RFITL_Msk;
    UART0->FIFO |= UART_FIFO_RFITL_1BYTE;

    /* Enable UART RDA/THRE/Time-out interrupt */
    NVIC_EnableIRQ(modinit->irq_n);
    UART_EnableInt(psUARTHandle->uart_base, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

void UartMcuDeInit( Uart_t *obj )
{
	S_UART_HANDLE *psUARTHandle;

    if( ( obj == NULL ) || (obj->UartId >= MAX_UART_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
    }
	
	psUARTHandle = &s_asUARTHandle[obj->UartId];

    UART_Close(psUARTHandle->uart_base);
		
	GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    if( ( obj == NULL ) || (obj->UartId >= MAX_UART_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return 1;
    }

	CRITICAL_SECTION_BEGIN( );

	if( IsFifoFull( &obj->FifoTx ) == false )
	{
		FifoPush( &obj->FifoTx, data );

		CRITICAL_SECTION_END( );
		return 0; // OK
	}
	
	CRITICAL_SECTION_END( );
	return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    if( ( obj == NULL ) || (obj->UartId >= MAX_UART_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return 1;
    }

	CRITICAL_SECTION_BEGIN( );

	if( IsFifoEmpty( &obj->FifoRx ) == false )
	{
		*data = FifoPop( &obj->FifoRx );
		CRITICAL_SECTION_END( );
		return 0;
	}

	CRITICAL_SECTION_END( );
	return 1;
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
{
    uint16_t localSize = 0;

    while( localSize < size )
    {
        if( UartGetChar( obj, buffer + localSize ) == 0 )
        {
            localSize++;
        }
        else
        {
            break;
        }
    }

    *nbReadBytes = localSize;

    if( localSize == 0 )
    {
        return 1; // Empty
    }

    return 0; // OK
}



uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
{
	uint8_t retryCount;
	uint16_t i;

    if( ( obj == NULL ) || (obj->UartId >= MAX_UART_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return 1;
    }

	for( i = 0; i < size; i++ )
	{
		retryCount = 0;
		while( UartPutChar( obj, buffer[i] ) != 0 )
		{
			retryCount++;

			// Exit if something goes terribly wrong
			if( retryCount > TX_BUFFER_RETRY_COUNT )
			{
				return 1; // Error
			}
		}
	}

	return 0; // OK
}





//TX completet callback
void HAL_UART_TxCpltCallback( S_UART_HANDLE *psUARTHandle )
{
	Uart_t *obj = psUARTHandle->obj;
	uint8_t TxData = 0;

    if( IsFifoEmpty( &obj->FifoTx ) == false )
    {
        TxData = FifoPop( &obj->FifoTx );
        //  Write one byte to the transmit data register
		while (UART_IS_TX_FULL(psUARTHandle->uart_base)); /* Wait Tx is not full to transmit data */
		UART_WRITE(psUARTHandle->uart_base , TxData);
    }

    if( obj->IrqNotify != NULL )
    {
        obj->IrqNotify( UART_NOTIFY_TX );
    }
}

//RX completet callback
void HAL_UART_RxCpltCallback(S_UART_HANDLE *psUARTHandle)
{
	Uart_t *obj = psUARTHandle->obj;
	uint8_t RxData = 0;

	/* Get all the input characters */
	while (UART_GET_RX_EMPTY(psUARTHandle->uart_base) == 0)
	{
		/* Get the character from UART Buffer */
		RxData = UART_READ(psUARTHandle->uart_base);

		if( IsFifoFull( &obj->FifoRx ) == false )
		{
			// Read one byte from the receive data register
			FifoPush( &obj->FifoRx, RxData );
		}
	}
		
    if( obj->IrqNotify != NULL )
    {
        obj->IrqNotify( UART_NOTIFY_RX );
    }
}


void HAL_UART_IRQHandler(S_UART_HANDLE *psUARTHandle)
{
    uint32_t u32IntSts = psUARTHandle->uart_base->INTSTS;

	//Receive Data Available Interrupt
    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
		if((psUARTHandle->mode == RX_ONLY) || (psUARTHandle->mode == RX_TX))
			HAL_UART_RxCpltCallback(psUARTHandle);
	}
	
	//RX Time-out Interrupt Indicator
	if (u32IntSts & UART_INTSTS_RXTOINT_Msk)
	{
		//printf("uart rx timeout \n");
	}
	
	//Transmit Holding Register Empty Interrupt Indicator
    if (u32IntSts & UART_INTSTS_THREINT_Msk)
    {
		if((psUARTHandle->mode == TX_ONLY) || (psUARTHandle->mode == RX_TX))
			HAL_UART_TxCpltCallback(psUARTHandle);
	}	
}



void UART0_IRQHandler( void )
{
	HAL_UART_IRQHandler(&s_asUARTHandle[UART_1]);
}

void UART1_IRQHandler( void )
{
	HAL_UART_IRQHandler(&s_asUARTHandle[UART_2]);
}

