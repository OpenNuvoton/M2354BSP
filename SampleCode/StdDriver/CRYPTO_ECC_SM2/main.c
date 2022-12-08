/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show whole ECC SM2 flow. Including private key/public key/Signature generation and
 *           Signature verification.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          256
#define PRNG_KEY_SIZE       PRNG_KEY_SIZE_256
#define CURVE_P_SIZE        CURVE_SM2_256


uint8_t Byte2Char(uint8_t c);
void CRPT_IRQHandler(void);
void dump_buff_hex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

static char e[168];
static char d[168];                         /* private key */
static char Qx[168], Qy[168];               /* temporary buffer used to keep output public keys */
static char k[168];                         /* random integer k form [1, n-1]                */
__ALIGNED(4) static char msg[] = "abc";
static char R[168], S[168];                 /* temporary buffer used to keep digital signature (R,S) pair */


#define ENDIAN(x)   ((((x)>>24)&0xff) | (((x)>>8)&0xff00) | (((x)<<8)&0xff0000) | ((x)<<24))

uint8_t Byte2Char(uint8_t c)
{
    if(c < 10)
        return (c + '0');
    if(c < 16)
        return (c - 10 + 'a');

    return 0;
}


void CRPT_IRQHandler()
{
    ECC_DriverISR(CRPT);
}


void dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}


void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i, j, nbits, m, err;
    uint32_t time;
    uint32_t au32r[(KEY_LENGTH + 31) / 32];
    uint8_t *au8r;
    uint32_t hash[8];


    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|            Crypto SM2 Demo                  |\n");
    printf("+---------------------------------------------+\n");

    nbits = KEY_LENGTH;

    /* hash the message */
    SM3((uint32_t*)msg, 3, hash);

    err = RNG_Open();
    if(err)
        printf("RNG Init FAIL\n");

    au8r = (uint8_t *)&au32r[0];

    sprintf(e, "%08x%08x%08x%08x%08x%08x%08x%08x", ENDIAN(hash[0]), ENDIAN(hash[1]), ENDIAN(hash[2]), ENDIAN(hash[3]), 
    ENDIAN(hash[4]), ENDIAN(hash[5]), ENDIAN(hash[6]), ENDIAN(hash[7]));
    printf("msg         = %s\n", msg);
    printf("e           = %s\n", e);

    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    
    do
    {

        /* Generate random number for private key */
        RNG_Random(au32r, (nbits + 31) / 32);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            d[j++] = Byte2Char(au8r[i] & 0xf);
            d[j++] = Byte2Char(au8r[i] >> 4);
        }
        d[j] = 0; // NULL end


        printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, d))
        {
            break;
        }
        else
        {
            /* Invalid key */
            printf("Current private key is not valid. Need a new one.\n");
        }

    }
    while(1);

    /* Enable SysTick */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Reset SysTick to measure time */
    SysTick->VAL = 0;
    if(ECC_GeneratePublicKey(CRPT, CURVE_P_SIZE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        return -1;
    }
    time = 0xffffff - SysTick->VAL;

    printf("Public Qx = %s\n", Qx);
    printf("Public Qy = %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    /*
        Try to generate signature serveral times with private key and verificate them with the same
        public key.

    */
    for(m = 0; m < 3; m++)
    {
        printf("//-------------------------------------------------------------------------//\n");

        /* Generate random number k */
        //BL_Random(&rng, au8r, nbits / 8);
        RNG_Random(au32r, (nbits + 7) / 8);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }
        k[j] = 0; // NULL End

        printf("  k  = %s\n", k);

        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, k))
        {
            /* Private key check ok */
        }
        else
        {
            /* Invalid key */
            printf("Current k is not valid\n");
            goto lexit;

        }

        SysTick->VAL = 0;
        if(SM2_Sign(CRPT, CURVE_P_SIZE, e, d, k, R, S) < 0)
        {
            printf("ECC signature generation failed!!\n");
            goto lexit;
        }
        time = 0xffffff - SysTick->VAL;

        printf("  R  = %s\n", R);
        printf("  S  = %s\n", S);
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

        SysTick->VAL = 0;
        err = SM2_Verify(CRPT, CURVE_P_SIZE, e, Qx, Qy, R, S);
        time = 0xffffff - SysTick->VAL;
        if(err < 0)
        {
            printf("ECC signature verification failed!!\n");
            goto lexit;
        }
        else
        {
            printf("ECC digital signature verification OK.\n");
        }
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);
    }

    printf("\n\n");

    strcpy(d, "bdca6455a55b9c2722d0f580f7f3c5633cbfcee85517aaa57f119b4b25569b43");
    ECC_GeneratePublicKey(CRPT, CURVE_P_SIZE, d, Qx, Qy);
    printf("d =%s\n", d);
    printf("Qx=%s\n", Qx);
    printf("Qy=%s\n", Qy);

    if(strcmp(Qx, "8a689f2ea87a601cdba2cd46e0862d66deb48ff1c636d068ed1ddbe47201bbdd") ||
            strcmp(Qy, "02be58c5acc94fa3fb82e1cbd220172f1f304bdd89ab7e294a4f672c04eb3de4"))
    {
        printf("Public key check fail!\n");
    }
    else
    {
        printf("public calculation check PASS\n");
    }

lexit:

    while(1);
}



