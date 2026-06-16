/**************************************************************************//**
 * @file     lcdlib.c
 * @version  V3.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD library source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"

/** @addtogroup Library Library
  @{
*/

/** @addtogroup LCDLIB LCD Library
  @{
*/

/** @addtogroup LCDLIB_EXPORTED_FUNCTIONS LCD Library Exported Functions
  @{
*/

/**
 *  @brief Display text on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputStr    Text string to show on display
 *
 *  @return None
 */
void LCDLIB_Printf(uint32_t u32Zone, const char *InputStr)
{
    uint32_t        i;
    uint32_t        index;
    uint32_t        ch;
    uint32_t        len;
    uint16_t        DispData;
    const char      *pStr = InputStr;

    len = strlen(InputStr);

    /* Fill out all characters on display */
    for(index = 0; index < g_LCDZoneInfo[u32Zone].u8LCDDispTableNum; index++)
    {
        if(index < len)
        {
            ch = (uint32_t)*pStr;
        }
        else
        {
            /* Padding with SPACE */
            ch = 0x20;
        }

        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            /* The Main Digit Table is an ASCII table beginning with "SPACE" (hex is 0x20) */
            ch       = ch - 0x20U;
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[ch];
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if((ch >= (uint32_t)'0') && (ch <= (uint32_t)'9'))
        {
            ch = ch - (uint32_t)'0';
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[ch];
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0U;
        }

        for(i = 0U; i < (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t u32Offset;
            uint32_t com;
            uint32_t seg;

            u32Offset = (index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset];
            seg = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset + 1U];

            if((DispData & (1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }

        pStr++;
    }
}

/**
 *  @brief Display unsigned number on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputNum    unsigned number to show on display
 *
 *  @return None
 */
void LCDLIB_PrintNumber(uint32_t u32Zone, uint32_t InputNum)
{
    uint32_t    i;
    uint32_t    index;
    uint32_t    div;

    /* Extract useful digits */
    div = 1U;

    /* Fill out all characters on display */
    index = g_LCDZoneInfo[u32Zone].u8LCDDispTableNum;
    while(index != 0U)
    {
        uint32_t val;
        uint16_t DispData;

        index--;

        val = (InputNum / div) % 10U;
        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            val += 16U; /* The Main Digit Table is an ASCII table beginning with "SPACE" */
        }

        DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[val];

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t u32Offset;
            uint32_t com;
            uint32_t seg;

            u32Offset = (index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset];
            seg = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset + 1U];

            if((DispData & (1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }

        div = div * 10U;
    }
}

/**
 *  @brief Display signed number on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  iInputNum   signed number to show on display
 *  @param[in]  u8DigiCnt   valid digital number count
 *
 *  @return None
 */
void LCDLIB_PrintNumberEx(uint32_t u32Zone, int32_t iInputNum, uint8_t u8DigiCnt)
{
    uint32_t    i;
    uint32_t    index;
    uint32_t    div;
    int32_t     iLocalNum   = iInputNum;
    uint8_t     u8LocalDigi = u8DigiCnt;
    uint8_t     is_negative = 0U;

    /* Extract useful digits */
    div = 1U;

    if(iLocalNum < 0)
    {
        is_negative = 1U;
        iLocalNum = -iLocalNum;
    }

    /* Fill out all characters on display */
    index = g_LCDZoneInfo[u32Zone].u8LCDDispTableNum;
    while(index != 0U)
    {
        uint32_t val;
        uint16_t DispData;

        index--;

        if(u8LocalDigi == 0U)
        {
            continue;
        }
        u8LocalDigi--;

        val = ((uint32_t)iLocalNum / div) % 10U;
        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            val += 16U; /* The Main Digit Table is an ASCII table beginning with "SPACE" */
        }

        DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[val];

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t u32Offset;
            uint32_t com;
            uint32_t seg;

            u32Offset = (index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset];
            seg = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset + 1U];

            if((DispData & (1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }

        div = div * 10U;
    }

    if(u32Zone == ZONE_MAIN_DIGIT)
    {
        LCDLIB_SetSymbol(SYMBOL_MINUS_2, (uint32_t)is_negative);
    }
}

/**
 *  @brief Display character on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  u32Index    the requested display position in zone
 *  @param[in]  u8Ch        Character to show on display
 *
 *  @return None
 */
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch)
{
    if(u32Index <= (uint32_t)g_LCDZoneInfo[u32Zone].u8LCDDispTableNum)
    {
        uint16_t DispData;
        uint32_t i;

        /* For Main Zone */
        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            /* Defined letters currently starts at "SPACE" - 0x20; */
            uint32_t ch = (uint32_t)u8Ch - 0x20U;
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[ch];
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if(((uint32_t)u8Ch >= (uint32_t)'0') && ((uint32_t)u8Ch <= (uint32_t)'9'))
        {
            uint32_t u32ChLocal = (uint32_t)u8Ch - (uint32_t)'0';
            DispData = g_LCDZoneInfo[u32Zone].pu16LCDDispTable[u32ChLocal];
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0;
        }

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            uint32_t u32Offset;
            uint32_t com;
            uint32_t seg;

            u32Offset = (u32Index * (uint32_t)g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2U) + (i * 2U);
            com = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset];
            seg = (uint32_t)g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg[u32Offset + 1U];

            if((DispData & (1U << i)) != 0U)
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1U);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0U);
            }
        }
    }
}

/**
 *  @brief Display symbol on LCD
 *
 *  @param[in]  u32Symbol   the combination of com, seg position
 *  @param[in]  u32OnOff    1: display symbol
 *                          0: not display symbol
 *
 *  @return     None
 */
void LCDLIB_SetSymbol(uint32_t u32Symbol, uint32_t u32OnOff)
{
    uint32_t com;
    uint32_t seg;

    com = (u32Symbol & 0xFU);
    seg = ((u32Symbol & 0xFF0U) >> 4U);

    if(u32OnOff)
    {
        LCD_SetPixel(com, seg, 1U); /* Turn on display */
    }
    else
    {
        LCD_SetPixel(com, seg, 0U); /* Turn off display */
    }
}

/*@}*/ /* end of group LCDLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LCDLIB */

/*@}*/ /* end of group Library */
