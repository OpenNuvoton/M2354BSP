/**************************************************************************//**
 * @file     mem_alloc.c
 * @version  V1.10
 * @brief   USB host library memory allocation functions.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"


/// @cond HIDDEN_SYMBOLS

//#define MEM_DEBUG

#ifdef MEM_DEBUG
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


UDEV_T * g_udev_list;

static uint8_t  _dev_addr_pool[128];
static volatile int  _device_addr;

/**
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.3<br>
 * <b>Justification:</b> The USB Host library requires dynamic memory for its system allocator.
 *                       malloc() is intentionally used here as the single, contained heap entry
 *                       point; the deviation is isolated to this wrapper so the rest of the
 *                       library does not call <stdlib.h> allocators directly.
 */
static void *usbh_sys_malloc(size_t size)
{
    /* cppcheck-suppress misra-c2012-21.3 */
    return malloc(size);
}

/**
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.3<br>
 * <b>Justification:</b> Pairs with usbh_sys_malloc(). free() is intentionally used here as the
 *                       single, contained heap release point; the deviation is isolated to this
 *                       wrapper so the rest of the library does not call <stdlib.h> allocators
 *                       directly.
 */
static void usbh_sys_free(void *ptr)
{
    /* cppcheck-suppress misra-c2012-21.3 */
    free(ptr);
}

/*--------------------------------------------------------------------------*/
/*   Memory alloc/free recording                                            */
/*--------------------------------------------------------------------------*/

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

    g_udev_list = NULL;

    (void)memset(_dev_addr_pool, 0, sizeof(_dev_addr_pool));
    _device_addr = 1;
}

uint32_t  usbh_memory_used(void)
{
    USB_debug("USB static memory: %d/%d, heap used: %d\n", _mem_pool_used, MEM_POOL_UNIT_NUM, _usbh_mem_used);
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

/**
 * @brief     Allocate a zero-initialized block from the USB Host heap.
 * @param[in] size  Number of bytes to allocate.
 * @return    Pointer to the allocated memory, or NULL if allocation failed.
 */
void * usbh_alloc_mem(int size)
{
    void  *p;

    p = usbh_sys_malloc((size_t)size);

    if(p == NULL)
    {
        USB_error("usbh_alloc_mem failed! %d\n", size);
        return NULL;
    }

    (void)memset(p, 0, size);
    memory_counter(size);
    return p;
}

/**
 * @brief     Free a memory block previously allocated by usbh_alloc_mem().
 * @param[in] p     Pointer to the memory block to free.
 * @param[in] size  Size of the block in bytes, used to update the usage counter.
 * @return    None.
 */
void usbh_free_mem(void *p, int size)
{
    usbh_sys_free(p);
    memory_counter(-size);
}


/*--------------------------------------------------------------------------*/
/*   USB device allocate/free                                               */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate and initialize a new USB device (UDEV_T) object.
 * @return    Pointer to the newly allocated USB device, or NULL if allocation failed.
 */
UDEV_T * alloc_device(void)
{
    UDEV_T  *udev;

    udev = usbh_sys_malloc(sizeof(*udev));

    if(udev == NULL)
    {
        USB_error("alloc_device failed!\n");
        return NULL;
    }
    (void)memset(udev, 0, sizeof(*udev));
    memory_counter(sizeof(*udev));
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

    if(udev == NULL)
    {
        return;
    }

    if(udev->cfd_buff != NULL)
    {
        usbh_free_mem(udev->cfd_buff, MAX_DESC_BUFF_SIZE);
    }

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

    usbh_sys_free(udev);
    memory_counter(-sizeof(*udev));
}

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

/*--------------------------------------------------------------------------*/
/*   UTR (USB Transfer Request) allocate/free                               */
/*--------------------------------------------------------------------------*/

/**
 * @brief     Allocate and initialize a new USB transfer request (UTR_T) object.
 * @param[in] udev  The USB device that owns this transfer request.
 * @return    Pointer to the newly allocated UTR, or NULL if allocation failed.
 */
UTR_T * alloc_utr(UDEV_T *udev)
{
    UTR_T  *utr;

    utr = usbh_sys_malloc(sizeof(*utr));

    if(utr == NULL)
    {
        USB_error("alloc_utr failed!\n");
        return NULL;
    }
    memory_counter(sizeof(*utr));
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
void free_utr(UTR_T *utr)
{
    if(utr == NULL)
    {
        return;
    }

    mem_debug("[FREE] [UTR] - 0x%x\n", (int)utr);
    usbh_sys_free(utr);
    memory_counter(-(int)sizeof(*utr));
}

/*--------------------------------------------------------------------------*/
/*   OHCI ED allocate/free                                                  */
/*--------------------------------------------------------------------------*/

ED_T * alloc_ohci_ED(void)
{
    int    i;

    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if(_unit_used[i] == 0U)
        {
            ED_T   *ed;

            _unit_used[i] = 1;
            _mem_pool_used++;
            ed = (ED_T *)&_mem_pool[i];
            (void)memset(ed, 0, sizeof(*ed));
            mem_debug("[ALLOC] [ED] - 0x%x\n", (int)ed);
            return ed;
        }
    }
    USB_error("alloc_ohci_ED failed!\n");
    return NULL;
}

void free_ohci_ED(const ED_T *ed)
{
    int      i;

    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if((uint32_t)&_mem_pool[i] == (uint32_t)ed)
        {
            mem_debug("[FREE]  [ED] - 0x%x\n", (int)ed);
            _unit_used[i] = 0;
            _mem_pool_used--;
            return;
        }
    }
    USB_debug("free_ohci_ED - not found! (ignored in case of multiple UTR)\n");
}

/*--------------------------------------------------------------------------*/
/*   OHCI TD allocate/free                                                  */
/*--------------------------------------------------------------------------*/
TD_T * alloc_ohci_TD(UTR_T *utr)
{
    int    i;

    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if(_unit_used[i] == 0U)
        {
            TD_T   *td;

            _unit_used[i] = 1;
            _mem_pool_used++;
            td = (TD_T *)&_mem_pool[i];

            (void)memset(td, 0, sizeof(*td));
            td->utr = utr;
            mem_debug("[ALLOC] [TD] - 0x%x\n", (int)td);
            return td;
        }
    }
    USB_error("alloc_ohci_TD failed!\n");
    return NULL;
}

void free_ohci_TD(const TD_T *td)
{
    int   i;

    for(i = 0; i < MEM_POOL_UNIT_NUM; i++)
    {
        if((uint32_t)&_mem_pool[i] == (uint32_t)td)
        {
            mem_debug("[FREE]  [TD] - 0x%x\n", (int)td);
            _unit_used[i] = 0;
            _mem_pool_used--;
            return;
        }
    }
    USB_error("free_ohci_TD - not found!\n");
}

/// @endcond HIDDEN_SYMBOLS

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

