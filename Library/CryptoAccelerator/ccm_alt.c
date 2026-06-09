/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright The Mbed TLS Contributors
 *  Copyright (c) 2023, Nuvoton Technology Corporation
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */

#include "common.h"

#if defined(MBEDTLS_CCM_C)


#include "mbedtls/ccm.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"
#include "NuMicro.h"

#include <string.h>

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#if defined(MBEDTLS_SELF_TEST) && defined(MBEDTLS_AES_C)
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_SELF_TEST && MBEDTLS_AES_C */
#endif /* MBEDTLS_PLATFORM_C */

void mbedtls_ccm_init( mbedtls_ccm_context *ctx );
int mbedtls_ccm_setkey( mbedtls_ccm_context *ctx,
                        mbedtls_cipher_id_t cipher,
                        const unsigned char *key,
                        unsigned int keybits );
void mbedtls_ccm_free( mbedtls_ccm_context *ctx );
int mbedtls_ccm_starts( mbedtls_ccm_context *ctx,
                        int mode,
                        const unsigned char *iv,
                        size_t iv_len );
int mbedtls_ccm_set_lengths( mbedtls_ccm_context *ctx,
                             size_t total_ad_len,
                             size_t plaintext_len,
                             size_t tag_len );
int mbedtls_ccm_update_ad( mbedtls_ccm_context *ctx,
                           const unsigned char *add,
                           size_t add_len );
int mbedtls_ccm_update( mbedtls_ccm_context *ctx,
                        const unsigned char *input, size_t input_len,
                        unsigned char *output, size_t output_size,
                        size_t *output_len );
int mbedtls_ccm_finish( mbedtls_ccm_context *ctx,
                        unsigned char *tag, size_t tag_len );
int mbedtls_ccm_star_encrypt_and_tag( mbedtls_ccm_context *ctx, size_t length,
                                      const unsigned char *iv, size_t iv_len,
                                      const unsigned char *add, size_t add_len,
                                      const unsigned char *input, unsigned char *output,
                                      unsigned char *tag, size_t tag_len );
int mbedtls_ccm_encrypt_and_tag( mbedtls_ccm_context *ctx, size_t length,
                                 const unsigned char *iv, size_t iv_len,
                                 const unsigned char *add, size_t add_len,
                                 const unsigned char *input, unsigned char *output,
                                 unsigned char *tag, size_t tag_len );
int mbedtls_ccm_star_auth_decrypt( mbedtls_ccm_context *ctx, size_t length,
                                   const unsigned char *iv, size_t iv_len,
                                   const unsigned char *add, size_t add_len,
                                   const unsigned char *input, unsigned char *output,
                                   const unsigned char *tag, size_t tag_len );
int mbedtls_ccm_auth_decrypt( mbedtls_ccm_context *ctx, size_t length,
                              const unsigned char *iv, size_t iv_len,
                              const unsigned char *add, size_t add_len,
                              const unsigned char *input, unsigned char *output,
                              const unsigned char *tag, size_t tag_len );


/*
 * Initialize context
 */
void mbedtls_ccm_init( mbedtls_ccm_context *ctx )
{
    (void)memset( ctx, 0, sizeof( mbedtls_ccm_context ) );
}

int mbedtls_ccm_setkey( mbedtls_ccm_context *ctx,
                        mbedtls_cipher_id_t cipher,
                        const unsigned char *key,
                        unsigned int keybits )
{
    int i;
    (void)cipher;

    /* Store the key to ctx */
    for(i = 0; i < (keybits / 8); i++)
    {
        ctx->key[i] = key[i];
    }
    ctx->keybits = keybits;

    return( 0 );
}

/*
 * Free context
 */
void mbedtls_ccm_free( mbedtls_ccm_context *ctx )
{
    if( ctx == NULL )
        return;
    mbedtls_cipher_free( &ctx->cipher_ctx );
    mbedtls_platform_zeroize( ctx, sizeof( mbedtls_ccm_context ) );
}

#define CCM_STATE__CLEAR                0U
#define CCM_STATE__STARTED              (1U << 0U)
#define CCM_STATE__LENGHTS_SET          (1U << 1U)
#define CCM_STATE__AUTH_DATA_STARTED    (1U << 2U)
#define CCM_STATE__AUTH_DATA_FINISHED   (1U << 3U)
#define CCM_STATE__ERROR                (1U << 4U)

/*
 * Encrypt or decrypt a partial block with CTR
 */
static int mbedtls_ccm_crypt( mbedtls_ccm_context *ctx,
                              size_t offset, size_t use_len,
                              const unsigned char *input,
                              unsigned char *output )
{
    size_t i;
    size_t olen = 0;
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char tmp_buf[16] = {0};

    ret = mbedtls_cipher_update( &ctx->cipher_ctx, ctx->ctr, 16, tmp_buf, &olen );
    if( ret != 0 )
    {
        ctx->state |= CCM_STATE__ERROR;
        mbedtls_platform_zeroize(tmp_buf, sizeof(tmp_buf));
        return ret;
    }

    for( i = 0U; i < use_len; i++ )
    {
        output[i] = input[i] ^ tmp_buf[offset + i];
    }

    mbedtls_platform_zeroize(tmp_buf, sizeof(tmp_buf));
    return ret;
}

static void mbedtls_ccm_clear_state(mbedtls_ccm_context *ctx) {
    ctx->state = CCM_STATE__CLEAR;
    (void)memset( ctx->y, 0, 16);
    (void)memset( ctx->ctr, 0, 16);
}

static int ccm_calculate_first_block_if_ready(mbedtls_ccm_context *ctx)
{
    unsigned char i;
    size_t len_left;

    /* length calulcation can be done only after both
     * mbedtls_ccm_starts() and mbedtls_ccm_set_lengths() have been executed
     */
    if( !((unsigned int)ctx->state & CCM_STATE__STARTED) || !((unsigned int)ctx->state & CCM_STATE__LENGHTS_SET) )
    {
        return 0;
    }

    /* CCM expects non-empty tag.
     * CCM* allows empty tag. For CCM* without tag, ignore plaintext length.
     */
    if( ctx->tag_len == 0U )
    {
        if( (ctx->mode == MBEDTLS_CCM_STAR_ENCRYPT) || (ctx->mode == MBEDTLS_CCM_STAR_DECRYPT) )
        {
            ctx->plaintext_len = 0;
        }
        else
        {
            return( MBEDTLS_ERR_CCM_BAD_INPUT );
        }
    }

    /*
     * First block:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)  - set by: mbedtls_ccm_starts()
     * iv_len+1 .. 15       length
     *
     * With flags as (bits):
     * 7        0
     * 6        add present?
     * 5 .. 3   (t - 2) / 2
     * 2 .. 0   q - 1
     */
    ctx->y[0] |= (unsigned char)( ((ctx->add_len > 0U) ? 1U : 0U) << 6U );
    ctx->y[0] |= (unsigned char)( ( ( ctx->tag_len - 2U ) / 2U ) << 3U );
    ctx->y[0] |= (unsigned char)( ctx->q - 1U );

    len_left = ctx->plaintext_len;
    for( i = 0U; i < ctx->q; i++ )
    {
        ctx->y[15U - i] = (unsigned char)MBEDTLS_BYTE_0( len_left );
        len_left >>= 8U;
    }

    if( len_left > 0U )
    {
        ctx->state |= CCM_STATE__ERROR;
        return( MBEDTLS_ERR_CCM_BAD_INPUT );
    }

    return (0);
}

int mbedtls_ccm_starts( mbedtls_ccm_context *ctx,
                        int mode,
                        const unsigned char *iv,
                        size_t iv_len )
{
    /* Also implies q is within bounds */
    if( (iv_len < 7U) || (iv_len > 13U) )
    {
        return( MBEDTLS_ERR_CCM_BAD_INPUT );
    }

    ctx->mode = mode;
    ctx->q = 16U - 1U - (unsigned char) iv_len;

    /*
     * Prepare counter block for encryption:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)
     * iv_len+1 .. 15       counter (initially 1)
     *
     * With flags as (bits):
     * 7 .. 3   0
     * 2 .. 0   q - 1
     */
    (void)memset( ctx->ctr, 0, 16);
    ctx->ctr[0] = ctx->q - 1U;
    (void)memcpy( ctx->ctr + 1, iv, iv_len );
    (void)memset( ctx->ctr + 1U + iv_len, 0, ctx->q );
    ctx->ctr[15] = 1U;

    /*
     * See ccm_calculate_first_block_if_ready() for block layout description
     */
    (void)memcpy( ctx->y + 1, iv, iv_len );

    ctx->state |= CCM_STATE__STARTED;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_set_lengths( mbedtls_ccm_context *ctx,
                             size_t total_ad_len,
                             size_t plaintext_len,
                             size_t tag_len )
{
    /*
     * Check length requirements: SP800-38C A.1
     * Additional requirement: a < 2^16 - 2^8 to simplify the code.
     * 'length' checked later (when writing it to the first block)
     *
     * Also, loosen the requirements to enable support for CCM* (IEEE 802.15.4).
     */
    if( (tag_len == 2U) || (tag_len > 16U) || ((tag_len % 2U) != 0U) )
    {
        return( MBEDTLS_ERR_CCM_BAD_INPUT );
    }

    if( total_ad_len >= 0xFF00U )
    {
        return( MBEDTLS_ERR_CCM_BAD_INPUT );
    }

    ctx->plaintext_len = plaintext_len;
    ctx->add_len = total_ad_len;
    ctx->tag_len = tag_len;
    ctx->processed = 0;

    ctx->state |= CCM_STATE__LENGHTS_SET;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_update_ad( mbedtls_ccm_context *ctx,
                           const unsigned char *add,
                           size_t add_len )
{
    (void)ctx;
    (void)add;
    (void)add_len;
    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;

}

int mbedtls_ccm_update( mbedtls_ccm_context *ctx,
                        const unsigned char *input, size_t input_len,
                        unsigned char *output, size_t output_size,
                        size_t *output_len )
{
    (void)ctx;
    (void)input;
    (void)input_len;
    (void)output;
    (void)output_size;
    (void)output_len;
    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
}

int mbedtls_ccm_finish( mbedtls_ccm_context *ctx,
                        unsigned char *tag, size_t tag_len )
{
    (void)ctx;
    (void)tag;
    (void)tag_len;
    return (0);
}

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt(mbedtls_ccm_context* ctx, int mode, size_t length,
    const unsigned char* iv, size_t iv_len,
    const unsigned char* add, size_t add_len,
    const unsigned char* input, unsigned char* output,
    unsigned char* tag, size_t tag_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    size_t add_len_aligned;
    size_t length_aligned;
    int32_t i;
    int32_t j;
    uint32_t keySizeOp;
    int32_t timeout;

    ret = mbedtls_ccm_starts(ctx, mode, iv, iv_len);
    if(ret != 0)
    {
        return(ret);
    }

    /* The default value is 1 in mbedtls_ccm_starts but we need 0 */
    ctx->ctr[15] = 0;


    ret = mbedtls_ccm_set_lengths(ctx, add_len, length, tag_len);
    if(ret != 0)
    {
        return(ret);
    }

    /* Check size of input block */
    add_len_aligned = (((add_len + 2U) & 0xfU) != 0U) ? ((((add_len + 2U) & (~0xfU)) + 16U)) : (add_len + 2U);
    length_aligned = ((length & 0xfU) != 0U) ? (((length & (~0xfU)) + 16U)) : (length);
    if((16U + add_len_aligned + length_aligned) > MAX_CCM_BUF)
    {
        return MBEDTLS_ERR_CCM_BAD_INPUT;
    }

    /* Prepare input block for hardware CCM */
    (void)memset(ctx->ccm_buf, 0, MAX_CCM_BUF);
    (void)memcpy(ctx->ccm_buf, ctx->y, 16);

    /* add len in big endian. and support only support 2^16 - 2^8 length */
    ctx->ccm_buf[16] = add_len >> 8;
    ctx->ccm_buf[17] = (uint8_t)(add_len & 0xffU);

    (void)memcpy(ctx->ccm_buf + 16U + 2U, add, add_len);
    (void)memset(ctx->ccm_buf + 16U + 2U + add_len, 0, add_len_aligned - add_len - 2U); // padding 0
    (void)memcpy(ctx->ccm_buf + 16U + add_len_aligned, input, length);
    (void)memset(ctx->ccm_buf + 16U + add_len_aligned + length, 0, length_aligned - length); // padding 0

    /* Init CCM Hardware */

    /* Enable AES interrupt */
    AES_ENABLE_INT(CRPT);

    /* Basic AES CCM Option */
    keySizeOp = (ctx->keybits == 256) ? AES_KEY_SIZE_256 : (ctx->keybits == 192) ? AES_KEY_SIZE_192 : AES_KEY_SIZE_128;

    CRPT->AES_CTL = (mode << CRPT_AES_CTL_ENCRPT_Pos) | (AES_MODE_CCM << CRPT_AES_CTL_OPMODE_Pos) |
        (keySizeOp << CRPT_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos) | CRPT_AES_CTL_KINSWAP_Msk;

    /* Set Key */
    j = 0;
    for(i = 0; i < (int32_t)(ctx->keybits / 8U); i += 4)
    {
        CRPT->AES_KEY[j] = (uint32_t)ctx->key[i] | ((uint32_t)ctx->key[i + 1] << 8U) | ((uint32_t)ctx->key[i + 2] << 16U) | ((uint32_t)ctx->key[i + 3] << 24U);
        j++;
    }

    /* Set Ctr0 */
    j = 0;
    for(i = 0; i < 16; i += 4)
    {
        CRPT->AES_IV[j] = (uint32_t)ctx->ctr[i] | ((uint32_t)ctx->ctr[i + 1] << 8U) | ((uint32_t)ctx->ctr[i + 2] << 16U) | ((uint32_t)ctx->ctr[i + 3] << 24U);
        j++;
    }


    /* Length settings. we assume length < 2^32 here */
    CRPT->AES_GCM_ACNT[0] = (uint32_t)(16U + add_len_aligned);
    CRPT->AES_GCM_ACNT[1] = 0;
    CRPT->AES_GCM_PCNT[0] = length;
    CRPT->AES_GCM_PCNT[1] = 0;

    /* Init DMA */
    CRPT->AES_SADDR = (uint32_t)ctx->ccm_buf;
    CRPT->AES_DADDR = (uint32_t)ctx->out_buf;
    CRPT->AES_CNT = (uint32_t)(16U + add_len_aligned + length_aligned);

    /* Clear flag */
    CRPT->INTSTS = CRPT_INTSTS_AESIF_Msk;
    
    /* Start AES CCM */
    CRPT->AES_CTL |= CRPT_AES_CTL_START_Msk | (CRYPTO_DMA_ONE_SHOT << CRPT_AES_CTL_DMALAST_Pos);

    timeout = SystemCoreClock;
    while((CRPT->INTSTS & CRPT_INTSTS_AESIF_Msk) == 0U)
    {
        /* Check timeout */
        if(timeout-- <= 0)
        {
            return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
        }
    }

    /* output */
    if(output != NULL)
    {
        (void)memcpy(output, ctx->out_buf, length);
    }
    if(tag != NULL)
    {
        (void)memcpy(tag, ctx->out_buf + length_aligned, tag_len);
    }

    mbedtls_ccm_clear_state(ctx);

    return( 0 );
}

/*
 * Authenticated encryption
 */
int mbedtls_ccm_star_encrypt_and_tag( mbedtls_ccm_context *ctx, size_t length,
                         const unsigned char *iv, size_t iv_len,
                         const unsigned char *add, size_t add_len,
                         const unsigned char *input, unsigned char *output,
                         unsigned char *tag, size_t tag_len )
{
    return( ccm_auth_crypt( ctx, MBEDTLS_CCM_STAR_ENCRYPT, length, iv, iv_len,
                            add, add_len, input, output, tag, tag_len ) );
}

int mbedtls_ccm_encrypt_and_tag( mbedtls_ccm_context *ctx, size_t length,
                         const unsigned char *iv, size_t iv_len,
                         const unsigned char *add, size_t add_len,
                         const unsigned char *input, unsigned char *output,
                         unsigned char *tag, size_t tag_len )
{
    return( ccm_auth_crypt( ctx, MBEDTLS_CCM_ENCRYPT, length, iv, iv_len,
                            add, add_len, input, output, tag, tag_len ) );
}

/*
 * Authenticated decryption
 */
static int mbedtls_ccm_compare_tags(const unsigned char *tag1, const unsigned char *tag2, size_t tag_len)
{
    unsigned char i;
    unsigned char diff_u8;
    int diff;

    /* Check tag in "constant-time" */
    diff = 0;
    for( i = 0U; i < (unsigned char)tag_len; i++ )
    {
        diff_u8 = (unsigned char)(tag1[i] ^ tag2[i]);
        diff |= (int)diff_u8;
    }

    if( diff != 0 )
    {
        return( MBEDTLS_ERR_CCM_AUTH_FAILED );
    }

    return( 0 );
}

static int ccm_auth_decrypt( mbedtls_ccm_context *ctx, int mode, size_t length,
                             const unsigned char *iv, size_t iv_len,
                             const unsigned char *add, size_t add_len,
                             const unsigned char *input, unsigned char *output,
                             const unsigned char *tag, size_t tag_len )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char check_tag[16];

    ret = ccm_auth_crypt( ctx, mode, length,
                          iv, iv_len, add, add_len,
                          input, output, check_tag, tag_len );
    if( ret != 0 )
    {
        return( ret );
    }

    ret = mbedtls_ccm_compare_tags( tag, check_tag, tag_len );
    if( ret != 0 )
    {
        mbedtls_platform_zeroize( output, length );
        return( ret );
    }

    return( 0 );
}

int mbedtls_ccm_star_auth_decrypt( mbedtls_ccm_context *ctx, size_t length,
                      const unsigned char *iv, size_t iv_len,
                      const unsigned char *add, size_t add_len,
                      const unsigned char *input, unsigned char *output,
                      const unsigned char *tag, size_t tag_len )
{
    return ccm_auth_decrypt( ctx, MBEDTLS_CCM_STAR_DECRYPT, length,
                             iv, iv_len, add, add_len,
                             input, output, tag, tag_len );
}

int mbedtls_ccm_auth_decrypt( mbedtls_ccm_context *ctx, size_t length,
                      const unsigned char *iv, size_t iv_len,
                      const unsigned char *add, size_t add_len,
                      const unsigned char *input, unsigned char *output,
                      const unsigned char *tag, size_t tag_len )
{
    return ccm_auth_decrypt( ctx, MBEDTLS_CCM_DECRYPT, length,
                             iv, iv_len, add, add_len,
                             input, output, tag, tag_len );
}

#endif /* MBEDTLS_CCM_C */
