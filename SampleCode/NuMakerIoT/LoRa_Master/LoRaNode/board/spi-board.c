/****************************************************************************
 * @file     spi-board.c
 * @version  V1.10
 * @brief    Target board SPI driver implementation
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "utilities.h"
#include "board-config.h"
#include "gpio.h"
#include "spi-board.h"

#define MAX_SPI_INST 1

#define SPI_PHASE_1EDGE		0
#define SPI_PHASE_2EDGE		1

#define SPI_POLARITY_LOW	0
#define SPI_POLARITY_HIGH	1

struct nu_spi_modinit_s {
    uint32_t clkidx;
    uint32_t clksrc;
    uint32_t clkdiv;
    uint32_t rsetidx;    
    IRQn_Type irq_n;
    void *var;
};

typedef struct{
	SPI_T *spi_base;
#if 0
	int dma_usage;
    int         dma_chn_id_tx;
    int         dma_chn_id_rx;
    uint32_t    event;
    uint32_t    hdlr_async;
	struct buffer_s tx_buff; /**< Tx buffer */
    struct buffer_s rx_buff; /**< Rx buffer */
#endif
}S_SPI_HANDLE;

static const struct nu_spi_modinit_s spi_modinit_tab[] = {
    {SPI3_MODULE, CLK_CLKSEL2_SPI3SEL_PCLK0, MODULE_NoMsk, SPI3_RST, SPI3_IRQn, NULL},
    {0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

static S_SPI_HANDLE s_asSPIHandle[MAX_SPI_INST];

/* Synchronous version of SPI_ENABLE()/SPI_DISABLE() macros
 *
 * The SPI peripheral clock is asynchronous with the system clock. In order to make sure the SPI
 * control logic is enabled/disabled, this bit indicates the real status of SPI controller.
 *
 * NOTE: All configurations shall be ready before calling SPI_ENABLE_SYNC().
 * NOTE: Before changing the configurations of SPIx_CTL, SPIx_CLKDIV, SPIx_SSCTL and SPIx_FIFOCTL registers,
 *       user shall clear the SPIEN (SPIx_CTL[0]) and confirm the SPIENSTS (SPIx_STATUS[15]) is 0
 *       (by SPI_DISABLE_SYNC here).
 */
__STATIC_INLINE void SPI_ENABLE_SYNC(SPI_T *spi_base)
{
    if (! (spi_base->CTL & SPI_CTL_SPIEN_Msk)) {
        SPI_ENABLE(spi_base);
    }
    while (! (spi_base->STATUS & SPI_STATUS_SPIENSTS_Msk));
}
__STATIC_INLINE void SPI_DISABLE_SYNC(SPI_T *spi_base)
{
    if (spi_base->CTL & SPI_CTL_SPIEN_Msk) {
        // NOTE: SPI H/W may get out of state without the busy check.
        while (SPI_IS_BUSY(spi_base));
    
        SPI_DISABLE(spi_base);
    }
    while (spi_base->STATUS & SPI_STATUS_SPIENSTS_Msk);
}

__STATIC_INLINE uint16_t nu_get16_le(const uint8_t *pos)
{
	uint16_t val;
	
	val = *pos ++;
	val += (*pos << 8);
	
	return val;
}

__STATIC_INLINE void nu_set16_le(uint8_t *pos, uint16_t val)
{
	*pos ++ = val & 0xFF;
	*pos = val >> 8;
}

__STATIC_INLINE uint32_t nu_get32_le(const uint8_t *pos)
{
	uint32_t val;
	
	val = *pos ++;
	val += (*pos ++ << 8);
	val += (*pos ++ << 16);
	val += (*pos ++ << 24);
	
	return val;
}

__STATIC_INLINE void nu_set32_le(uint8_t *pos, uint32_t val)
{
	*pos ++ = val & 0xFF;
	*pos ++ = (val >> 8) & 0xFF;
	*pos ++ = (val >> 16) & 0xFF;
	*pos = (val >> 24) & 0xFF;
}

static int spi_writeable(S_SPI_HANDLE *psSPIHandle)
{
    // Receive FIFO must not be full to avoid receive FIFO overflow on next transmit/receive
    return (! SPI_GET_TX_FIFO_FULL_FLAG(((SPI_T *) psSPIHandle->spi_base)));
}

static int spi_readable(S_SPI_HANDLE *psSPIHandle)
{
    return ! SPI_GET_RX_FIFO_EMPTY_FLAG(((SPI_T *) psSPIHandle->spi_base));
}

static uint8_t spi_get_data_width(S_SPI_HANDLE *psSPIHandle)
{    
    SPI_T *spi_base = (SPI_T *) psSPIHandle->spi_base;

    uint32_t data_width = ((spi_base->CTL & SPI_CTL_DWIDTH_Msk) >> SPI_CTL_DWIDTH_Pos);
    if (data_width == 0) {
        data_width = 32;
    }

    return data_width;
}

int SPI_MasterWrite(S_SPI_HANDLE *psSPIHandle, int value)
{
    SPI_T *spi_base = (SPI_T *) psSPIHandle->spi_base;

    // NOTE: Data in receive FIFO can be read out via ICE.
    SPI_ENABLE_SYNC(spi_base);

    // Wait for tx buffer empty
    while(! spi_writeable(psSPIHandle));
    SPI_WRITE_TX(spi_base, value);
	 
    // Wait for rx buffer full
    while (! spi_readable(psSPIHandle));
    int value2 = SPI_READ_RX(spi_base);

    /* We don't call SPI_DISABLE_SYNC here for performance. */

    return value2;
}

int SPI_MasterBlockWriteRead(S_SPI_HANDLE *psSPIHandle, const char *tx_buffer, int tx_length,
                           char *rx_buffer, int rx_length, char write_fill) {
    int total = (tx_length > rx_length) ? tx_length : rx_length;

    uint8_t data_width = spi_get_data_width(psSPIHandle);
    uint8_t bytes_per_word = (data_width + 7) / 8;
	int32_t in = 0;
	uint8_t *tx = (uint8_t *)tx_buffer;
	uint8_t *rx = (uint8_t *)rx_buffer;
	int i = 0;
	int32_t fill = 0;
	
	if(bytes_per_word == 4)
		fill = (write_fill << 24) | (write_fill << 16) | (write_fill << 8) | write_fill;
	else if(bytes_per_word == 2)
		fill = (write_fill << 8) | write_fill;
	else
		fill = write_fill;

	while(i < total){
        if (i > tx_length){
			in = SPI_MasterWrite(psSPIHandle, fill);			
		} 
		else{
            switch (bytes_per_word) {
            case 4:
                in = SPI_MasterWrite(psSPIHandle, nu_get32_le(tx));
                tx += 4;
                break;
            case 2:
				in = SPI_MasterWrite(psSPIHandle, nu_get16_le(tx));
                tx += 2;
                break;
            case 1:
                in = SPI_MasterWrite(psSPIHandle, *((uint8_t *) tx));
                tx += 1;
                break;
            }
		}
		

		if((rx) && (i <= rx_length)){
            switch (bytes_per_word) {
            case 4:
                nu_set32_le(rx, in);
                rx += 4;
                break;
            case 2:
                nu_set16_le(rx, in);
                rx += 2;
                break;
            case 1:
                *rx ++ = (uint8_t) in;
                break;
            }
		}

		i += bytes_per_word;
	}

    return total;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	SPI public function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    int mode;
	
	SPI_T *spi_base = (SPI_T *) s_asSPIHandle[obj->SpiId].spi_base;

	if((bits < 8) || (bits > 32))
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}
	
    SPI_DISABLE_SYNC(spi_base);

	if(cpol == SPI_POLARITY_LOW){
		if(cpha == SPI_PHASE_1EDGE)
			mode = 0;
		else
			mode = 1;
	}
	else{
		if(cpha == SPI_PHASE_1EDGE)
			mode = 3;
		else
			mode = 2;
	}

    SPI_Open(spi_base,
		 slave ? SPI_SLAVE : SPI_MASTER,
		 (mode == 0) ? SPI_MODE_0 : (mode == 1) ? SPI_MODE_1 : (mode == 2) ? SPI_MODE_2 : SPI_MODE_3,
		 bits,
		 SPI_GetBusClock(spi_base));
	
	SPI_SET_MSB_FIRST(spi_base);

    if (! slave) {
		// Master
		// Configure SS as low active.
		SPI_EnableAutoSS(spi_base, SPI_SS, SPI_SS_ACTIVE_LOW);
	}
	else{
		// Slave
		// Configure SS as low active.
		spi_base->SSCTL &= ~SPI_SSCTL_SSACTPOL_Msk;
	}
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
	SPI_T *spi_base = (SPI_T *) s_asSPIHandle[obj->SpiId].spi_base;

    SPI_DISABLE_SYNC(spi_base);
    SPI_SetBusClock(spi_base, hz);
}


void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
	obj->SpiId = 0;
	
	if(spiId >= MAX_SPI_INST)
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}	
	
	if(spiId == SPI_1)
	{
		//Using SPI3 instance for SPI_1 id
        s_asSPIHandle[SPI_1].spi_base = SPI3;

		//Set SPI multple function pin
		GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, SPI3_MFP_MOSI_VALUE );
		GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, SPI3_MFP_MISO_VALUE );
		GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, SPI3_MFP_CLK_VALUE );
		GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, SPI3_MFP_NSS_VALUE );
	}
	else
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

    CRITICAL_SECTION_BEGIN( );
	const struct nu_spi_modinit_s *modinit = &spi_modinit_tab[spiId];

   // Reset this module
    SYS_ResetModule(modinit->rsetidx);

    // Select IP clock source
    CLK_SetModuleClock(modinit->clkidx, modinit->clksrc, modinit->clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(modinit->clkidx);

	obj->SpiId = spiId;	

	//Master
	SpiFormat(obj, 8, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 0);
	
    if(nss == NC)
    {
		//disable auto SS
        SPI_DisableAutoSS(s_asSPIHandle[obj->SpiId].spi_base);
    }
 
    //SpiFrequency(obj, 10000000); //10MHz
    SpiFrequency(obj, 4000000); //4MHz
    CRITICAL_SECTION_END( );
}

void SpiDeInit( Spi_t *obj )
{
	SPI_T *spi_base = (SPI_T *) s_asSPIHandle[obj->SpiId].spi_base;
	const struct nu_spi_modinit_s *modinit = &spi_modinit_tab[obj->SpiId];

	SPI_Close(spi_base);

    // Disable IP clock
    CLK_DisableModuleClock(modinit->clkidx);
	
    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    uint8_t rxData = 0;

    if( ( obj == NULL ) || (obj->SpiId >= MAX_SPI_INST) )
    {
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return 0;
    }

    CRITICAL_SECTION_BEGIN( );

	SPI_MasterBlockWriteRead(&s_asSPIHandle[obj->SpiId], (const char *)&outData, 1, (char *)&rxData, 1, 0x0);
    CRITICAL_SECTION_END( );
    return( rxData );
}
