/***************************************************************************//**
 * @file     spi_transfer.h
 * @brief    ISP tool SPI initialization header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SPI_TRANS_H__
#define __SPI_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bSpiDataReady;
extern uint32_t spi_rcvbuf[];

/*-------------------------------------------------------------*/
void SPI_Init(void);

#endif  /* __SPI_TRANS_H__ */
