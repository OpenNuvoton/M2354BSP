/**************************************************************************//**
 * @file     mem_alloc.c
 * @version  V1.10
 * @brief   USB host library memory allocation functions.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"


/// @cond HIDDEN_SYMBOLS

//#define MEM_DEBUG

#if defined(MEM_DEBUG) && (defined(ENABLE_ERROR_MSG) || defined(ENABLE_DEBUG_MSG))
#define mem_debug       (void)usbh_printf
#else
#define mem_debug(...)
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
static uint8_t  _mem_pool[MEM_POOL_UNIT_NUM][MEM_POOL_UNIT_SIZE];
#else
static uint8_t _mem_pool[MEM_POOL_UNIT_NUM][MEM_POOL_UNIT_SIZE] __attribute__((aligned(32)));
#endif
static uint8_t  _unit_used[MEM_POOL_UNIT_NUM];

static volatile int  _usbh_mem_used;
static volatile int  _usbh_max_mem_used;
static volatile int  _mem_pool_used;

static UDEV_T   _udev_pool[USBH_UDEV_POOL_NUM];
static uint8_t  _udev_used[USBH_UDEV_POOL_NUM];
static UTR_T    _utr_pool[USBH_UTR_POOL_NUM];
static uint8_t  _utr_used[USBH_UTR_POOL_NUM];
static IFACE_T  _iface_pool[USBH_IFACE_POOL_NUM];
static uint8_t  _iface_used[USBH_IFACE_POOL_NUM];

typedef union
{
    DESC_CONF_T  conf;
    uint8_t      byte[MAX_DESC_BUFF_SIZE];
} CFD_BUFF_T;

static CFD_BUFF_T  _cfd_buff_pool[USBH_UDEV_POOL_NUM];

#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t  _rpd_buff[USBH_MAX_RPD_BUFF_SIZE];
#pragma data_alignment=4
static uint8_t  _xfer_buff_pool[USBH_XFER_BUFF_NUM][USBH_MAX_XFER_BUFF_SIZE];
#pragma data_alignment=4
static uint8_t  _iso_buff_pool[USBH_ISO_STREAM_NUM][USBH_MAX_ISO_PACKET_SIZE * USBH_ISO_FRAME_PER_STREAM];
#else
static uint8_t  _rpd_buff[USBH_MAX_RPD_BUFF_SIZE] __attribute__((aligned(4)));
static uint8_t  _xfer_buff_pool[USBH_XFER_BUFF_NUM][USBH_MAX_XFER_BUFF_SIZE] __attribute__((aligned(4)));
static uint8_t  _iso_buff_pool[USBH_ISO_STREAM_NUM][USBH_MAX_ISO_PACKET_SIZE * USBH_ISO_FRAME_PER_STREAM] __attribute__((aligned(4)));
#endif
static uint8_t  _cfd_buff_used[USBH_UDEV_POOL_NUM];
static uint8_t  _rpd_buff_used;
static uint8_t  _xfer_buff_used[USBH_XFER_BUFF_NUM];
static uint8_t  _iso_buff_used[USBH_ISO_STREAM_NUM];

UDEV_T * g_udev_list;

static uint8_t  _dev_addr_pool[128];
static volatile int  _device_addr;

/*
 *  The slot flags of every static object pool in this library are updated from both thread
 *  context (enumeration) and interrupt context (transfer completion calls free_utr() from
 *  USBH_IRQHandler), so the test-and-set / clear sequences that guard them must not be
 *  interrupted. These two helpers are the single implementation shared by the core and by
 *  every class driver.
 */
uint32_t usbh_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __set_PRIMASK(1U);
    return primask;
}

void usbh_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/// @endcond HIDDEN_SYMBOLS

/*--------------------------------------------------------------------------*/
/*   Memory alloc/free recording                                            */
/*--------------------------------------------------------------------------*/

/// @cond HIDDEN_SYMBOLS

void usbh_memory_init(void)
{
    if(sizeof(TD_T) > (size_t)MEM_POOL_UNIT_SIZE)
    {
        USB_error("TD_T - MEM_POOL_UNIT_SIZE too small!\n");
        while(1){}
    }

    if(sizeof(ED_T) > (size_t)MEM_POOL_UNIT_SIZE)
    {
        USB_error("ED_T - MEM_POOL_UNIT_SIZE too small!\n");
        while(1) {}
    }

    _usbh_mem_used = 0L;
    _usbh_max_mem_used = 0L;

    (void)memset(_unit_used, 0, sizeof(_unit_used));
    _mem_pool_used = 0;

    (void)memset(_udev_used, 0, sizeof(_udev_used));
    (void)memset(_utr_used, 0, sizeof(_utr_used));
    (void)memset(_iface_used, 0, sizeof(_iface_used));
    (void)memset(_cfd_buff_used, 0, sizeof(_cfd_buff_used));
    (void)memset(_xfer_buff_used, 0, sizeof(_xfer_buff_used));
    (void)memset(_iso_buff_used, 0, sizeof(_iso_buff_used));
    _rpd_buff_used = 0U;

    g_udev_list = NULL;

    (void)memset(_dev_addr_pool, 0, sizeof(_dev_addr_pool));
    _device_addr = 1;
}

uint32_t  usbh_memory_used(void)
{
    USB_debug("USB static memory: %d/%d, tracked allocation: %d\n", _mem_pool_used, MEM_POOL_UNIT_NUM, _usbh_mem_used);
    return _usbh_mem_used;
}

static void  memory_counter(int size)
{
    _usbh_mem_used += size;
    if(_usbh_mem_used > _usbh_max_mem_used)
    {
        _usbh_max_mem_used = _usbh_mem_used;
    }
}

/// @endcond HIDDEN_SYMBOLS

/*--------------------------------------------------------------------------*/
/*   USB byte buffer allocate/free                                          */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate the configuration descriptor buffer of a USB device.
 * @return    Pointer to a zeroed descriptor buffer of MAX_DESC_BUFF_SIZE bytes, or NULL if
 *            none is free.
 */
DESC_CONF_T * usbh_alloc_cfd_buff(void)
{
    CFD_BUFF_T  *buff = NULL;
    uint32_t     primask;
    int          i;

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UDEV_POOL_NUM; i++)
    {
        if(_cfd_buff_used[i] == 0U)
        {
            _cfd_buff_used[i] = 1U;
            buff = &_cfd_buff_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(buff == NULL)
    {
        USB_error("usbh_alloc_cfd_buff failed! All %d configuration descriptor buffers are in use. Raise USBH_MAX_HUB_DEV or USBH_MAX_FUNC_DEV in config.h to support a larger topology.\n", USBH_UDEV_POOL_NUM);
        return NULL;
    }

    (void)memset(buff->byte, 0, sizeof(buff->byte));
    memory_counter(MAX_DESC_BUFF_SIZE);
    return &buff->conf;
}

/**
 * @brief     Free a USB device's configuration descriptor buffer.
 * @param[in] udev  Device whose cfd_buff is to be released. Its cfd_buff field is cleared
 *                  to NULL afterward. NULL udev, or a NULL cfd_buff, is ignored.
 * @return    None.
 */
void usbh_free_cfd_buff(UDEV_T *udev)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if((udev == NULL) || (udev->cfd_buff == NULL))
    {
        return;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UDEV_POOL_NUM; i++)
    {
        if(_cfd_buff_pool[i].byte == udev->cfd_buff)
        {
            was_used = _cfd_buff_used[i];
            _cfd_buff_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_UDEV_POOL_NUM)
    {
        USB_error("usbh_free_cfd_buff: buffer does not belong to _cfd_buff_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("usbh_free_cfd_buff: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-MAX_DESC_BUFF_SIZE);
    udev->cfd_buff = NULL;
}

/**
 * @brief     Allocate the buffer used to fetch and parse a HID report descriptor.
 * @param[in] size  Number of bytes required.
 * @return    Pointer to a zeroed buffer of at least @a size bytes, or NULL if none is free.
 */
uint8_t * usbh_alloc_rpd_buff(int size)
{
    uint8_t  *buff = NULL;
    uint32_t  primask;

    if((size <= 0) || (size > (int)sizeof(_rpd_buff)))
    {
        USB_error("usbh_alloc_rpd_buff failed! Report descriptor needs %d bytes. Raise USBH_MAX_RPD_BUFF_SIZE in config.h to support this device.\n", size);
        return NULL;
    }

    primask = usbh_enter_critical();
    if(_rpd_buff_used == 0U)
    {
        _rpd_buff_used = 1U;
        buff = _rpd_buff;
    }
    usbh_exit_critical(primask);

    if(buff == NULL)
    {
        USB_error("usbh_alloc_rpd_buff failed! The report descriptor buffer is already in use.\n");
        return NULL;
    }

    (void)memset(buff, 0, (size_t)size);
    memory_counter(size);
    return buff;
}

/**
 * @brief     Free a HID report descriptor buffer.
 * @param[in] buff  Buffer returned by usbh_alloc_rpd_buff(). NULL is safely ignored.
 * @param[in] size  Size passed to usbh_alloc_rpd_buff(), used to update the usage counter.
 * @return    None.
 */
void usbh_free_rpd_buff(const uint8_t *buff, int size)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       found = 0;

    if(buff == NULL)
    {
        return;
    }

    primask = usbh_enter_critical();
    if(_rpd_buff == buff)
    {
        found = 1;
        was_used = _rpd_buff_used;
        _rpd_buff_used = 0U;
    }
    usbh_exit_critical(primask);

    if(found == 0)
    {
        USB_error("usbh_free_rpd_buff: buffer does not belong to _rpd_buff! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("usbh_free_rpd_buff: buffer released twice! It may still be in use by another owner.\n");
    }
    else
    {
        /* no action required */
    }
    memory_counter(-size);
}

/**
 * @brief     Allocate the data buffer of an interrupt pipe.
 * @param[in] size  Number of bytes required, at most one endpoint packet.
 * @return    Pointer to a zeroed buffer of at least @a size bytes, or NULL if none is free.
 */
uint8_t * usbh_alloc_xfer_buff(int size)
{
    uint8_t  *buff = NULL;
    uint32_t  primask;
    int       i;

    if((size <= 0) || (size > USBH_MAX_XFER_BUFF_SIZE))
    {
        USB_error("usbh_alloc_xfer_buff failed! Endpoint packet is %d bytes. Raise USBH_MAX_XFER_BUFF_SIZE in config.h to support this endpoint.\n", size);
        return NULL;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_XFER_BUFF_NUM; i++)
    {
        if(_xfer_buff_used[i] == 0U)
        {
            _xfer_buff_used[i] = 1U;
            buff = _xfer_buff_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(buff == NULL)
    {
        USB_error("usbh_alloc_xfer_buff failed! All %d interrupt buffers are in use. Raise USBH_MAX_HID_IFACE in config.h to support more interrupt pipes.\n", USBH_XFER_BUFF_NUM);
        return NULL;
    }

    (void)memset(buff, 0, (size_t)size);
    memory_counter(size);
    return buff;
}

/**
 * @brief     Free an interrupt pipe data buffer.
 * @param[in] buff  Buffer returned by usbh_alloc_xfer_buff(). NULL is safely ignored.
 * @param[in] size  Size passed to usbh_alloc_xfer_buff(), used to update the usage counter.
 * @return    None.
 */
void usbh_free_xfer_buff(const uint8_t *buff, int size)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if(buff == NULL)
    {
        return;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_XFER_BUFF_NUM; i++)
    {
        if(_xfer_buff_pool[i] == buff)
        {
            was_used = _xfer_buff_used[i];
            _xfer_buff_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_XFER_BUFF_NUM)
    {
        USB_error("usbh_free_xfer_buff: buffer does not belong to _xfer_buff_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("usbh_free_xfer_buff: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-size);
}

/**
 * @brief     Allocate the data buffer of an isochronous stream.
 * @param[in] size  Number of bytes required, one packet times the frames buffered per stream.
 * @return    Pointer to a zeroed buffer of at least @a size bytes, or NULL if none is free.
 */
uint8_t * usbh_alloc_iso_buff(int size)
{
    uint8_t  *buff = NULL;
    uint32_t  primask;
    int       i;

    if((size <= 0) || (size > (int)sizeof(_iso_buff_pool[0])))
    {
        USB_error("usbh_alloc_iso_buff failed! Stream needs %d bytes, arena holds %d. Raise USBH_MAX_ISO_PACKET_SIZE in config.h to support this endpoint.\n", size, (int)sizeof(_iso_buff_pool[0]));
        return NULL;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_ISO_STREAM_NUM; i++)
    {
        if(_iso_buff_used[i] == 0U)
        {
            _iso_buff_used[i] = 1U;
            buff = _iso_buff_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(buff == NULL)
    {
        USB_error("usbh_alloc_iso_buff failed! All %d isochronous buffers are in use. Raise USBH_ISO_STREAM_NUM in config.h to support more simultaneous streams.\n", USBH_ISO_STREAM_NUM);
        return NULL;
    }

    (void)memset(buff, 0, (size_t)size);
    memory_counter(size);
    return buff;
}

/**
 * @brief     Free an isochronous stream data buffer.
 * @param[in] buff  Buffer returned by usbh_alloc_iso_buff(). NULL is safely ignored.
 * @param[in] size  Size passed to usbh_alloc_iso_buff(), used to update the usage counter.
 * @return    None.
 */
void usbh_free_iso_buff(const uint8_t *buff, int size)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if(buff == NULL)
    {
        return;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_ISO_STREAM_NUM; i++)
    {
        if(_iso_buff_pool[i] == buff)
        {
            was_used = _iso_buff_used[i];
            _iso_buff_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_ISO_STREAM_NUM)
    {
        USB_error("usbh_free_iso_buff: buffer does not belong to _iso_buff_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("usbh_free_iso_buff: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-size);
}


/*--------------------------------------------------------------------------*/
/*   USB device allocate/free                                               */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate and initialize a new USB device (UDEV_T) object.
 * @return    Pointer to the newly allocated USB device, or NULL if the pool is exhausted.
 *
 */
UDEV_T * alloc_device(void)
{
    UDEV_T   *udev = NULL;
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UDEV_POOL_NUM; i++)
    {
        if(_udev_used[i] == 0U)
        {
            _udev_used[i] = 1U;
            udev = &_udev_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(udev == NULL)
    {
        USB_error("alloc_device failed! All %d UDEV_T slots are in use. Raise USBH_MAX_HUB_DEV or USBH_MAX_FUNC_DEV in config.h to support a larger topology.\n", USBH_UDEV_POOL_NUM);
        return NULL;
    }
    (void)memset(udev, 0, sizeof(*udev));
    memory_counter((int)sizeof(*udev));
    udev->cur_conf = -1;                    /* must! used to identify the first SET CONFIGURATION */
    udev->next = g_udev_list;               /* chain to global device list */
    g_udev_list = udev;
    return udev;
}

/**
 * @brief     Free a USB device object and remove it from the global device list.
 * @param[in] udev  The USB device to be freed. NULL is safely ignored.
 * @return    None.
 */
void free_device(UDEV_T *udev)
{
    UDEV_T  *d;
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if(udev == NULL)
    {
        return;
    }

    usbh_free_cfd_buff(udev);

    /*
     *  Remove it from the global device list
     */
    if(g_udev_list == udev)
    {
        g_udev_list = g_udev_list->next;
    }
    else
    {
        d = g_udev_list;
        while(d != NULL)
        {
            if(d->next == udev)
            {
                d->next = udev->next;
                break;
            }
            d = d->next;
        }
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UDEV_POOL_NUM; i++)
    {
        if(&_udev_pool[i] == udev)
        {
            was_used = _udev_used[i];
            _udev_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_UDEV_POOL_NUM)
    {
        USB_error("free_device: device does not belong to _udev_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("free_device: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-(int)sizeof(*udev));
}

/// @cond HIDDEN_SYMBOLS

int  alloc_dev_address(void)
{
    _device_addr++;

    if(_device_addr >= 128)
    {
        _device_addr = 1;
    }

    while(1)
    {
        if(_dev_addr_pool[_device_addr] == 0U)
        {
            _dev_addr_pool[_device_addr] = 1;
            return _device_addr;
        }
        _device_addr++;
        if(_device_addr >= 128)
        {
            _device_addr = 1;
        }
    }
}

void  free_dev_address(int dev_addr)
{
    if(dev_addr < 128)
    {
        _dev_addr_pool[dev_addr] = 0;
    }
}

/// @endcond HIDDEN_SYMBOLS

/*--------------------------------------------------------------------------*/
/*   UTR (USB Transfer Request) allocate/free                               */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate and initialize a new USB transfer request (UTR_T) object.
 * @param[in] udev  The USB device that owns this transfer request.
 * @return    Pointer to the newly allocated UTR, or NULL if the pool is exhausted.
 *
 */
UTR_T * alloc_utr(UDEV_T *udev)
{
    UTR_T    *utr = NULL;
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UTR_POOL_NUM; i++)
    {
        if(_utr_used[i] == 0U)
        {
            _utr_used[i] = 1U;
            utr = &_utr_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(utr == NULL)
    {
        USB_error("alloc_utr failed! All %d UTR_T slots are in use. Raise USBH_MAX_UTR_PER_DEV or USBH_MAX_FUNC_DEV in config.h to support more concurrent transfers.\n", USBH_UTR_POOL_NUM);
        return NULL;
    }
    memory_counter((int)sizeof(*utr));
    (void)memset(utr, 0, sizeof(*utr));
    utr->udev = udev;
    mem_debug("[ALLOC] [UTR] - 0x%x\n", (int)utr);
    return utr;
}

/**
 * @brief     Free a USB transfer request (UTR_T) object.
 * @param[in] utr  The transfer request to be freed. NULL is safely ignored.
 * @return    None.
 */
void free_utr(const UTR_T *utr)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if(utr == NULL)
    {
        return;
    }

    mem_debug("[FREE] [UTR] - 0x%x\n", (int)utr);

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_UTR_POOL_NUM; i++)
    {
        if(&_utr_pool[i] == utr)
        {
            was_used = _utr_used[i];
            _utr_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_UTR_POOL_NUM)
    {
        USB_error("free_utr: request does not belong to _utr_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("free_utr: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-(int)sizeof(*utr));
}

/*--------------------------------------------------------------------------*/
/*   USB interface allocate/free                                            */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate and zero-initialize a new USB interface (IFACE_T) object.
 * @return    Pointer to the newly allocated interface, or NULL if the pool is exhausted.
 */
IFACE_T * usbh_alloc_iface(void)
{
    IFACE_T  *iface = NULL;
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_IFACE_POOL_NUM; i++)
    {
        if(_iface_used[i] == 0U)
        {
            _iface_used[i] = 1U;
            iface = &_iface_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(iface == NULL)
    {
        USB_error("usbh_alloc_iface failed! All %d IFACE_T slots are in use. Raise USBH_MAX_IFACE_PER_DEV or USBH_MAX_FUNC_DEV in config.h to support more interfaces.\n", USBH_IFACE_POOL_NUM);
        return NULL;
    }
    (void)memset(iface, 0, sizeof(*iface));
    memory_counter((int)sizeof(*iface));
    return iface;
}

/**
 * @brief     Free a USB interface (IFACE_T) object.
 * @param[in] iface  The interface to be freed. NULL is safely ignored.
 * @return    None.
 */
void usbh_free_iface(const IFACE_T *iface)
{
    uint32_t  primask;
    uint8_t   was_used = 0U;
    int       i;

    if(iface == NULL)
    {
        return;
    }

    primask = usbh_enter_critical();
    for(i = 0; i < USBH_IFACE_POOL_NUM; i++)
    {
        if(&_iface_pool[i] == iface)
        {
            was_used = _iface_used[i];
            _iface_used[i] = 0U;
            break;
        }
    }
    usbh_exit_critical(primask);

    if(i >= USBH_IFACE_POOL_NUM)
    {
        USB_error("usbh_free_iface: interface does not belong to _iface_pool! Ignored.\n");
    }
    else if(was_used == 0U)
    {
        USB_error("usbh_free_iface: slot %d released twice! It may still be in use by another owner.\n", i);
    }
    else
    {
        /* no action required */
    }
    memory_counter(-(int)sizeof(*iface));
}

/// @cond HIDDEN_SYMBOLS

/*--------------------------------------------------------------------------*/
/*   OHCI ED allocate/free                                                  */
/*--------------------------------------------------------------------------*/

ED_T * alloc_ohci_ED(void)
{
    ED_T     *ed = NULL;
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if(_unit_used[i] == 0U)
        {
            _unit_used[i] = 1;
            _mem_pool_used++;
            ed = (ED_T *)&_mem_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(ed == NULL)
    {
        USB_error("alloc_ohci_ED failed!\n");
        return NULL;
    }

    (void)memset(ed, 0, sizeof(*ed));
    mem_debug("[ALLOC] [ED] - 0x%x\n", (int)ed);
    return ed;
}

void free_ohci_ED(const ED_T *ed)
{
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if((uint32_t)&_mem_pool[i] == (uint32_t)ed)
        {
            _unit_used[i] = 0;
            _mem_pool_used--;
            break;
        }
    }
    usbh_exit_critical(primask);

#ifdef ENABLE_DEBUG_MSG
    if(i >= MEM_POOL_UNIT_NUM)
    {
        USB_debug("free_ohci_ED - not found! (ignored in case of multiple UTR)\n");
    }
    else
    {
        mem_debug("[FREE]  [ED] - 0x%x\n", (int)ed);
    }
#endif
}

/*--------------------------------------------------------------------------*/
/*   OHCI TD allocate/free                                                  */
/*--------------------------------------------------------------------------*/
TD_T * alloc_ohci_TD(UTR_T *utr)
{
    TD_T     *td = NULL;
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if(_unit_used[i] == 0U)
        {
            _unit_used[i] = 1;
            _mem_pool_used++;
            td = (TD_T *)&_mem_pool[i];
            break;
        }
    }
    usbh_exit_critical(primask);

    if(td == NULL)
    {
        USB_error("alloc_ohci_TD failed!\n");
        return NULL;
    }

    (void)memset(td, 0, sizeof(*td));
    td->utr = utr;
    mem_debug("[ALLOC] [TD] - 0x%x\n", (int)td);
    return td;
}

void free_ohci_TD(const TD_T *td)
{
    uint32_t  primask;
    int       i;

    primask = usbh_enter_critical();
    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if((uint32_t)&_mem_pool[i] == (uint32_t)td)
        {
            _unit_used[i] = 0;
            _mem_pool_used--;
            break;
        }
    }
    usbh_exit_critical(primask);

#ifdef ENABLE_DEBUG_MSG
    if(i >= MEM_POOL_UNIT_NUM)
    {
        USB_error("free_ohci_TD - not found!\n");
    }
    else
    {
        mem_debug("[FREE]  [TD] - 0x%x\n", (int)td);
    }
#endif
}

/// @endcond HIDDEN_SYMBOLS

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

