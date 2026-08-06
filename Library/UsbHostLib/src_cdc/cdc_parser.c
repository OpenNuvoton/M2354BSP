/**************************************************************************//**
 * @file     cdc_parser.c
 * @version  V1.00
 * @brief    M2354 MCU USB Host CDC Class driver
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "usbh_lib.h"
#include "usbh_cdc.h"

/// @cond HIDDEN_SYMBOLS

static void cdc_read_descriptor_header(const uint8_t *buffer, DESC_HDR_T *header)
{
    header->bLength = buffer[0];
    header->bDescriptorType = buffer[1];
}

static void cdc_read_config_descriptor(const uint8_t *buffer, DESC_CONF_T *config)
{
    config->bLength = buffer[0];
    config->bDescriptorType = buffer[1];
    config->wTotalLength = (uint16_t)(((uint16_t)buffer[2]) | ((uint16_t)buffer[3] << 8U));
}

static void cdc_read_cs_interface_header(const uint8_t *buffer, CDC_IF_HDR_T *cifd)
{
    cifd->bLength = buffer[0];
    cifd->bDescriptorType = buffer[1];
    cifd->bDescriptorSubtype = buffer[2];

    if (cifd->bLength > 3U)
    {
        uint8_t payload_length = cifd->bLength - 3U;

        if (payload_length > (uint8_t)sizeof(cifd->payload))
        {
            payload_length = (uint8_t)sizeof(cifd->payload);
        }
        (void)memcpy(&cifd->payload[0], &buffer[3], (size_t)payload_length);
    }
}

static uint8_t cdc_read_interface_number(const uint8_t *buffer)
{
    return buffer[2];
}

static int  cdc_parse_cs_interface(CDC_DEV_T *cdev, uint8_t *buffer, int size)
{
    int             parsed = 0;
    int             size_tmp = size;

    while (size_tmp > 0)
    {

        while (size_tmp >= (int)sizeof(DESC_HDR_T))
        {
            DESC_HDR_T      header;
            CDC_IF_HDR_T    cifd;

            cdc_read_descriptor_header(&buffer[parsed], &header);

            if (header.bLength < 2U)
            {
                CDC_DBGMSG("Invalid descriptor length of %d\n", header.bLength);
                return -1;
            }

            if (header.bDescriptorType != CDC_CS_INTERFACE)
            {
                return parsed;
            }

            if ((size_tmp < (int)header.bLength) || ((int)header.bLength < (int)CDC_IF_HDR_MANDATORY_LEN))
            {
                CDC_DBGMSG("Invalid CS_INTERFACE descriptor length of %d\n", header.bLength);
                return -1;
            }

            cdc_read_cs_interface_header(&buffer[parsed], &cifd);

            CDC_DBGMSG("CS_INTERFACE: 0x%x, ", cifd.bDescriptorSubtype);

            switch(cifd.bDescriptorSubtype)
            {
                case CDC_DT_HDR_FUNC:
                    CDC_DBGMSG("Header Functional\n");
                    break;
                case CDC_DT_CALL_MANAGE:
                    CDC_DBGMSG("Call Management\n");
                    break;
                case CDC_DT_ABS_CTRL:
                    CDC_DBGMSG("Abstract Control Management\n");
                    break;
                case CDC_DT_LINE_MANAGE:
                    CDC_DBGMSG("Direct Line Management\n");
                    break;
                case CDC_DT_TEL_RINGER:
                    CDC_DBGMSG("Telephone Ringer\n");
                    break;
                case CDC_DT_TEL_OPER_MODES:
                    CDC_DBGMSG("Telephone Operational Modes\n");
                    break;
                case CDC_DT_CALL_LINE_CAP:
                    CDC_DBGMSG("Telephone Call and Line State Reporting Capabilities\n");
                    break;
                case CDC_DT_UNION:
                    CDC_DBGMSG("Union Functional\n");

                    if (cifd.bLength >= 5U)
                    {
                        cdev->ifnum_data = cifd.payload[1];
                    }

                    if (cifd.bLength >= 6U)
                    {
                        CDC_DBGMSG("Union Functional length %d, not supported!\n", cifd.bLength);
                    }
                    break;
                case CDC_DT_COUNTRY_SEL:
                    CDC_DBGMSG("Country Selection\n");
                    break;
                case CDC_DT_USB_TERMINAL:
                    CDC_DBGMSG("USB Terminal\n");
                    break;
                case CDC_DT_NET_CHANNEL:
                    CDC_DBGMSG("Network Channel Terminal\n");
                    break;
                case CDC_DT_PROTO_UNIT:
                    CDC_DBGMSG("Protocol Unit\n");
                    break;
                case CDC_DT_EXTENT_UNIT:
                    CDC_DBGMSG("Extension Unit\n");
                    break;
                case CDC_DT_MULTI_CHANNEL:
                    CDC_DBGMSG("Multi-Channel Management\n");
                    break;
                case CDC_DT_CAPI_CTRL:
                    CDC_DBGMSG("CAPI Control Management\n");
                    break;
                case CDC_DT_ETHERNET_FUNC:
                    CDC_DBGMSG("Ethernet Networking Functional\n");
                    break;
                case CDC_DT_ATM_FUNC:
                    CDC_DBGMSG("ATM Networking Functional\n");
                    break;

                default:
                    CDC_DBGMSG("Unknown Functional Descriptor: 0x%x\n", cifd.bDescriptorSubtype);
                    break;
            }

            parsed += (int)header.bLength;
            size_tmp -= (int)header.bLength;
        }

    }   /* end of while */
    return parsed;
}


int  cdc_config_parser(CDC_DEV_T *cdev)
{
    UDEV_T          *udev = cdev->udev;
    DESC_CONF_T     config;
    DESC_HDR_T      header;
    uint8_t         *bptr;
    int             size;

    cdc_read_config_descriptor(udev->cfd_buff, &config);
    bptr = udev->cfd_buff;
    bptr = &bptr[config.bLength];
    size = (int)config.wTotalLength - (int)config.bLength;

    while(size >= (int)sizeof(DESC_HDR_T))
    {
        cdc_read_descriptor_header(bptr, &header);

        if(((int)header.bLength > size) || (header.bLength < 2U))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header.bLength);
            return USBH_ERR_NOT_SUPPORTED;
        }

        /*
         *  Is the interface descriptor of this CDC device?
         */
        if(header.bDescriptorType == USB_DT_INTERFACE)
        {
            if((header.bLength >= 3U) && (cdc_read_interface_number(bptr) == cdev->iface_cdc->if_num))
            {
                bptr = &bptr[header.bLength];
                size -= (int)header.bLength;
                break;
            }
        }
        bptr = &bptr[header.bLength];
        size -= (int)header.bLength;
    }   /* end of while */

    /*------------------------------------------------------------------*/
    /*  Parsing all follwoing CDC class interface descriptors           */
    /*------------------------------------------------------------------*/

    while(size >= (int)sizeof(DESC_HDR_T))
    {
        cdc_read_descriptor_header(bptr, &header);

        if(((int)header.bLength > size) || (header.bLength < 2U))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header.bLength);
            return USBH_ERR_NOT_SUPPORTED;
        }

        /*
         *  Is a class interface descriptor?
         */
        if(header.bDescriptorType != CDC_CS_INTERFACE)
        {
            break;
        }

        int result = cdc_parse_cs_interface(cdev, bptr, size);
        if(result < 0)
        {
            return result;
        }
        bptr = &bptr[result];
        size -= result;
    }   /* end of while */

    CDC_DBGMSG("CDC ifnum_cdc = %d\n", cdev->iface_cdc->if_num);
    if(cdev->iface_data)
    {
        CDC_DBGMSG("CDC ifnum_data = %d\n", cdev->iface_data->if_num);
    }
    return 0;
}


/// @endcond HIDDEN_SYMBOLS

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

