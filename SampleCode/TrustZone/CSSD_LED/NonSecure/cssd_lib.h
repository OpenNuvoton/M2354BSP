/**************************************************************************//**
 * @file     cssd_lib.h
 * @version  V1.00
 * @brief    M2354 Collaborative Secure Software Development Library header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CSSD_LIB_H__
#define __CSSD_LIB_H__

/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern int32_t Secure_PD2_LED_On(uint32_t num);
extern int32_t Secure_PD2_LED_Off(uint32_t num);
extern int32_t Secure_PD1_LED_On(uint32_t num);
extern int32_t Secure_PD1_LED_Off(uint32_t num);
extern int32_t Secure_PD0_LED_On(uint32_t num);
extern int32_t Secure_PD0_LED_Off(uint32_t num);
extern uint32_t GetSystemCoreClock(void);

#endif //__CSSD_LIB_H__
