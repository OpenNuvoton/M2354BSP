/**************************************************************************//**
 * @file     keystore.h
 * @version  V3.00
 * @brief    Key Store Driver Header
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __KEYSTORE_H__
#define __KEYSTORE_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup KS_Driver Key Store Driver
  @{
*/

/** @addtogroup KS_EXPORTED_CONSTANTS Key Store Exported Constants
  @{
*/

#define KS_TOMETAKEY(x)     (((uint32_t)(x) << KS_METADATA_NUMBER_Pos) & KS_METADATA_NUMBER_Msk)
#define KS_TOKEYIDX(x)      (((uint32_t)(x) & KS_METADATA_NUMBER_Msk) >> KS_METADATA_NUMBER_Pos)

typedef enum KSMEM
{
    KS_SRAM  = 0,     /*!< Volatile Memory                                */
    KS_FLASH = 1,    /*!< Non-volatile Memory                            */
    KS_OTP   = 2     /*!< One-Time Programming Memory                    */
} KS_MEM_Type;

#define KS_OP_READ      (0U << KS_CTL_OPMODE_Pos)
#define KS_OP_WRITE     (1U << KS_CTL_OPMODE_Pos)
#define KS_OP_ERASE     (2U << KS_CTL_OPMODE_Pos)
#define KS_OP_ERASE_ALL (3U << KS_CTL_OPMODE_Pos)
#define KS_OP_REVOKE    (4U << KS_CTL_OPMODE_Pos)
#define KS_OP_REMAN     (5U << KS_CTL_OPMODE_Pos)

#define KS_OWNER_AES        (0UL)
#define KS_OWNER_HMAC       (1UL)
#define KS_OWNER_RSA_EXP    (2UL)
#define KS_OWNER_RSA_MID    (3UL)
#define KS_OWNER_ECC        (4UL)
#define KS_OWNER_CPU        (5UL)

#define KS_META_AES     (0UL << KS_METADATA_OWNER_Pos)   /*!< AES Access Only                                */
#define KS_META_HMAC    (1UL << KS_METADATA_OWNER_Pos)   /*!< HMAC Access Only                               */
#define KS_META_RSA_EXP (2UL << KS_METADATA_OWNER_Pos)   /*!< RSA_EXP Access Only                            */
#define KS_META_RSA_MID (3UL << KS_METADATA_OWNER_Pos)   /*!< RSA_MID Access Only                            */
#define KS_META_ECC     (4UL << KS_METADATA_OWNER_Pos)   /*!< ECC Access Only                                */
#define KS_META_CPU     (5UL << KS_METADATA_OWNER_Pos)   /*!< CPU Access Only                                */

#define KS_META_128     ( 0UL << KS_METADATA_SIZE_Pos)  /*!< Key size 128 bits                              */
#define KS_META_163     ( 1UL << KS_METADATA_SIZE_Pos)  /*!< Key size 163 bits                              */
#define KS_META_192     ( 2UL << KS_METADATA_SIZE_Pos)  /*!< Key size 192 bits                              */
#define KS_META_224     ( 3UL << KS_METADATA_SIZE_Pos)  /*!< Key size 224 bits                              */
#define KS_META_233     ( 4UL << KS_METADATA_SIZE_Pos)  /*!< Key size 233 bits                              */
#define KS_META_255     ( 5UL << KS_METADATA_SIZE_Pos)  /*!< Key size 255 bits                              */
#define KS_META_256     ( 6UL << KS_METADATA_SIZE_Pos)  /*!< Key size 256 bits                              */
#define KS_META_283     ( 7UL << KS_METADATA_SIZE_Pos)  /*!< Key size 283 bits                              */
#define KS_META_384     ( 8UL << KS_METADATA_SIZE_Pos)  /*!< Key size 384 bits                              */
#define KS_META_409     ( 9UL << KS_METADATA_SIZE_Pos)  /*!< Key size 409 bits                              */
#define KS_META_512     (10UL << KS_METADATA_SIZE_Pos)  /*!< Key size 512 bits                              */
#define KS_META_521     (11UL << KS_METADATA_SIZE_Pos)  /*!< Key size 521 bits                              */
#define KS_META_571     (12UL << KS_METADATA_SIZE_Pos)  /*!< Key size 571 bits                              */
#define KS_META_1024    (16UL << KS_METADATA_SIZE_Pos)  /*!< Key size 1024 bits                             */
#define KS_META_1536    (17UL << KS_METADATA_SIZE_Pos)  /*!< Key size 1024 bits                             */
#define KS_META_2048    (18UL << KS_METADATA_SIZE_Pos)  /*!< Key size 2048 bits                             */
#define KS_META_3072    (19UL << KS_METADATA_SIZE_Pos)  /*!< Key size 1024 bits                             */
#define KS_META_4096    (20UL << KS_METADATA_SIZE_Pos)  /*!< Key size 4096 bits                             */

#define KS_META_BOOT    ( 1UL << KS_METADATA_BS_Pos)    /*!< Key only used for boot ROM only                */

#define KS_META_READABLE (1UL << KS_METADATA_READABLE_Pos)  /*!< Allow the key to be read by software       */

#define KS_META_PRIV     (1UL << KS_METADATA_PRIV_Pos)  /*!< Privilege key                                  */
#define KS_META_NONPRIV  (0UL << KS_METADATA_PRIV_Pos)  /*!< Non-privilege key                              */

#define KS_META_SECURE    (1UL << KS_METADATA_SEC_Pos)    /*!< Secure key                                     */
#define KS_META_NONSECURE (0UL << KS_METADATA_SEC_Pos)    /*!< Non-secure key                                 */

#define KS_TIMEOUT        (SystemCoreClock)             /*!< KS time-out counter (1 second time-out)        */
#define KS_OK             ( 0L)                         /*!< KS operation OK                                */
#define KS_ERR_FAIL       (-1L)                         /*!< KS operation failed                            */
#define KS_ERR_TIMEOUT    (-2L)                         /*!< KS operation abort due to timeout error        */
#define KS_ERR_INIT       (-3L)                         /*!< KS intital fail                                */
#define KS_ERR_BUSY       (-4L)                         /*!< KS is in busy state                            */
#define KS_ERR_PARAMETER  (-5L)                         /*!< Wrong input parameters                         */

/**
  * @brief      Enable scramble function
  * @details    This function is used to enable scramle function of Key Store.
  */

#define KS_SCRAMBLING()     KS->CTL |= KS_CTL_SCMB_Msk




/**@}*/ /* end of group KS_EXPORTED_CONSTANTS */

/** @addtogroup KS_EXPORTED_FUNCTIONS Key Store Exported Functions
  @{
*/

int32_t KS_Open(void);
int32_t KS_Read(KS_MEM_Type eType, int32_t i32KeyIdx, uint32_t au32Key[], uint32_t u32WordCnt);
int32_t KS_Write(KS_MEM_Type eType, uint32_t u32Meta, const uint32_t au32Key[]);
int32_t KS_WriteOTP(int32_t i32KeyIdx, uint32_t u32Meta, const uint32_t au32Key[]);
int32_t KS_EraseKey(int32_t i32KeyIdx);
int32_t KS_EraseAll(KS_MEM_Type eType);
int32_t KS_RevokeKey(KS_MEM_Type eType, int32_t i32KeyIdx);
uint32_t KS_GetRemainSize(KS_MEM_Type mem);
int32_t KS_ToggleSRAM(void);
uint32_t KS_GetKeyWordCnt(uint32_t u32Meta);
uint32_t KS_GetRemainKeyCount(KS_MEM_Type mem);

/**@}*/ /* end of group KS_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group KS_Driver */

/**@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __KEYSTORE_H__ */


