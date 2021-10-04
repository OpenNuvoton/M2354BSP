/**************************************************************************//**
 * @file     m2354_bme680.h
 * @version  V3.00
 * @brief    M2354 BME680 Driver Header File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef	__M2354_BME680_H__
#define __M2354_BME680_H__

#include "NuMicro.h"
#include "bme680.h"

void    m2354_delay_ms(uint32_t period);
int8_t  m2354_bme680_init(struct bme680_dev* dev);
int8_t  m2354_bme680_config(struct bme680_dev* dev);

#endif /* __M2354_BME680_H__ */
