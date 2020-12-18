/**************************************************************************//**
 * @file     i2c_transfer.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     24, August, 2020
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t g_u8I2cDataReady;
extern uint8_t g_au8I2cRcvBuf[];

/*-------------------------------------------------------------*/
void I2C_Init(void);

#endif  /* __I2C_TRANS_H__ */
