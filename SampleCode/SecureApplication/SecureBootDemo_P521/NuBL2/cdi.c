/**************************************************************************//**
 * @file     cdi.c
 * @version  V3.00
 * @brief    Calculate CDI value for DICE.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define HMAC_SHA_MODE   HMAC_MODE_SHA512

#if (HMAC_SHA_MODE == HMAC_MODE_SHA512)
# define HMA_SHA_SIZE    64
#elif (HMAC_SHA_MODE == HMAC_MODE_SHA384)
# define HMA_SHA_SIZE    48
#elif (HMAC_SHA_MODE == HMAC_MODE_SHA256)
# define HMA_SHA_SIZE    32
#error "Unsupported"
#endif


/* Swap bytes of a 32 bits word */
#define SWAP32(value)   ((uint32_t)(((value) & 0x000000FF) << 24) | \
                         ((uint32_t)(((value) & 0x0000FF00) <<  8) | \
                         ((uint32_t)(((value) & 0x00FF0000) >>  8) | \
                         ((uint32_t)(((value) & 0xFF000000) >> 24)))))

/**
  * @brief      Perform HMAC calculation
  * @param[in]  key         key buffer address.
  * @param[in]  key_len     key length, must be word alignment.
  * @param[in]  data        data buffer address.
  * @param[in]  len         data length, must be word alignment.
  * @param[out] pbuf        output buffer address, must be word alignment and size must larger than 48 bytes.
  */
static int32_t hmac(uint8_t *key, uint32_t key_len, uint8_t *data, uint32_t len, uint32_t *pbuf)
{
    uint32_t au32DmaBuf[32] = {0};
    int32_t i;
    uint8_t *pu8;
    uint32_t u32TimeOutCnt;

    pu8 = (uint8_t *)&au32DmaBuf[0];
    for(i = 0; i < key_len; i++)
        pu8[i] = key[i];
    for(i = 0; i < len; i++)
        pu8[key_len + i] = data[i];

    CRPT->HMAC_CTL = (HMAC_SHA_MODE << CRPT_HMAC_CTL_OPMODE_Pos) |
                     (SHA_IN_OUT_SWAP << CRPT_HMAC_CTL_OUTSWAP_Pos);
    CRPT->HMAC_KEYCNT = key_len;

    CRPT->HMAC_SADDR = (uint32_t)&au32DmaBuf[0];
    CRPT->HMAC_DMACNT = key_len + len;


    CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMAEN_Msk | CRPT_HMAC_CTL_DMALAST_Msk;
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    /* Waiting for HMAC busy */
    while(CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            /* Timeout */
            return -1;
        }
    }
    
    /* Get result of extract process 'prk' */
    for(i = 0; i < (HMA_SHA_SIZE / 4); i++)
        pbuf[i] = CRPT->HMAC_DGST[i];

    return 0;
}

/**
  * @brief      Perform CDI Calculation
  * @param[in]  salt    salt buffer address, the SHA-512 message digest.
  * @param[in]  ikm     ikm buffer address, the 32 bytes UDS data.
  * @param[in]  info    info buffer address, the 16 bytes info data.
  * @param[out] pbuf    output buffer address, must be word alignment and size must larger than 48 bytes.
  */
int32_t CDI_Cal(uint8_t *salt, uint8_t *ikm, uint8_t *info, uint32_t *pbuf)
{
    int32_t err;

    err = hmac(salt, HMA_SHA_SIZE, ikm, 32, pbuf);
    if(err < 0)
        return err;
    
    hmac((uint8_t *)&pbuf[0], HMA_SHA_SIZE, info, 16, pbuf);
    if(err < 0)
        return err;

    return 0;
}