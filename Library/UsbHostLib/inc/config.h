/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * @brief    This header file defines the configuration of USB Host library.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef  _USBH_CONFIG_H_
#define  _USBH_CONFIG_H_

#include <stdarg.h>

/// @cond HIDDEN_SYMBOLS

/*----------------------------------------------------------------------------------------*/
/*   Hardware settings                                                                    */
/*----------------------------------------------------------------------------------------*/
#define HCLK_MHZ               192          /* used for loop-delay. must be larger than
                                               true HCLK clock MHz                        */

#define ENABLE_OHCI_IRQ()      NVIC_EnableIRQ(USBH_IRQn)
#define DISABLE_OHCI_IRQ()     NVIC_DisableIRQ(USBH_IRQn)

#define ENABLE_OHCI                         /* Enable OHCI host controller                */

#define OHCI_ISO_DELAY         4            /* preserved number frames while scheduling
                                               OHCI isochronous transfer                  */

#define MAX_DESC_BUFF_SIZE     512          /* To hold the configuration descriptor, USB
                                               core will allocate a buffer with this size
                                               for each connected device. USB core does
                                               not release it until device disconnected.  */

/*----------------------------------------------------------------------------------------*/
/*   Memory allocation settings                                                           */
/*----------------------------------------------------------------------------------------*/

#define STATIC_MEMORY_ALLOC    0       /* pre-allocate static memory blocks. No dynamic memory aloocation.
                                          But the maximum number of connected devices and transfers are
                                          limited.  */

#define MAX_UDEV_DRIVER        8       /*!< Maximum number of registered drivers                      */
#define MAX_ALT_PER_IFACE      8       /*!< maximum number of alternative interfaces per interface    */
#define MAX_EP_PER_IFACE       6       /*!< maximum number of endpoints per interface                 */
#define MAX_HUB_DEVICE         8       /*!< Maximum number of hub devices                             */

/* Typed object pools. Every fixed-size USB object (device,
   interface, transfer request, MSC/CDC device, HID report info) is served from its own statically
   allocated, correctly typed array -- no generic-to-typed pointer conversion at the allocation
   site, and no heap use. An exhausted pool returns NULL; every call site reports the failure
   through USB_error and refuses the new device.

   The limits cover the shipped sample applications, not the theoretical worst case, so an
   under-declared topology running out is expected, not a fault: raise the named limit and rebuild
   to support a larger topology.                                                                    */

#define USBH_MAX_HUB_DEV       2       /*!< External hubs attached at once. Must not exceed
                                            MAX_HUB_DEVICE.                                         */
#define USBH_MAX_FUNC_DEV      2       /*!< Function (non-hub) devices attached at once; covers the
                                            shipped samples, all reached through the hub(s) above.  */
#define USBH_MAX_IFACE_PER_DEV 4       /*!< Interfaces claimed per function device; covers the
                                            largest composite, an audio headset with HID controls
                                            (AudioControl, AudioStreaming in/out, HID).             */
#define USBH_MAX_UTR_PER_DEV   6       /*!< Transfer requests outstanding per function device;
                                            covers the same headset: NUM_UTR (2) per streaming
                                            direction (4) plus one interrupt-in and one
                                            interrupt-out request for HID.                          */
#define USBH_MAX_HID_IFACE     4       /*!< HID interfaces present at once; covers the declared
                                            function devices each presenting two HID interfaces
                                            (typical boot keyboard).                                */
#define USBH_MAX_REPORT_PER_HID 8      /*!< Report-info nodes per HID interface: one per Input or
                                            Output main item, plus one per named X/Y/Wheel usage.
                                            Measured in the BSP test set: 6 (wheel mouse), 5 and 3
                                            (keyboard's two interfaces), 6 (headset consumer ctrl). */

/*!< Every attached device owns one UDEV_T; hubs and functions share the pool.                      */
#define USBH_UDEV_POOL_NUM     (USBH_MAX_HUB_DEV + USBH_MAX_FUNC_DEV)

/*!< Per-function requests, plus one status-change request per hub, plus the enumeration control
     request.                                                                                       */
#define USBH_UTR_POOL_NUM      ((USBH_MAX_FUNC_DEV * USBH_MAX_UTR_PER_DEV) + USBH_MAX_HUB_DEV + 1)

/*!< Interfaces of the declared function devices, plus the one interface every hub declares.        */
#define USBH_IFACE_POOL_NUM    ((USBH_MAX_FUNC_DEV * USBH_MAX_IFACE_PER_DEV) + USBH_MAX_HUB_DEV)

/*!< One class object per attached device of that class; bounded by the full function-device count. */
#define USBH_MSC_POOL_NUM      USBH_MAX_FUNC_DEV
#define USBH_CDC_POOL_NUM      USBH_MAX_FUNC_DEV

/*!< Report-info nodes live as long as their owning HID interface, so the bound covers every HID
     interface that may be present.                                                                 */
#define USBH_RPINFO_POOL_NUM   (USBH_MAX_HID_IFACE * USBH_MAX_REPORT_PER_HID)

/* Byte buffers                                                                                     */

#define USBH_MAX_RPD_BUFF_SIZE   512   /*!< Largest HID report descriptor that can be fetched and
                                            parsed (wDescriptorLength + 8 guard bytes).             */
#define USBH_MAX_XFER_BUFF_SIZE  64    /*!< Largest interrupt endpoint packet buffered, in bytes --
                                            the full-speed maximum, so no supported device exceeds
                                            it.                                                     */
#define USBH_XFER_BUFF_NUM       (USBH_MAX_HID_IFACE * 2)
                                       /*!< Interrupt packet buffers: one per open interrupt-in or
                                            interrupt-out pipe per HID interface.                   */
#define USBH_MAX_ISO_PACKET_SIZE 200   /*!< Largest isochronous packet buffered, in bytes. Test-set
                                            headset reports 200 (speaker) / 100 (mic) for 48kHz
                                            stereo 16-bit.                                          */
#define USBH_ISO_FRAME_PER_STREAM 16   /*!< Frames buffered per stream. Must equal IF_PER_UTR (8,
                                            usb.h) x NUM_UTR (2, usbh_uac.h)                        */
#define USBH_ISO_STREAM_NUM      2     /*!< Concurrent isochronous streams; 2 covers a headset's
                                            simultaneous speaker and microphone streaming.          */

/* Host controller hardware transfer descriptors memory pool. ED/TD/ITD of OHCI and QH/QTD of EHCI
   are all allocated from this pool. Allocated unit size is determined by MEM_POOL_UNIT_SIZE.
   May allocate one or more units depend on hardware descriptor type.                                 */

#define MEM_POOL_UNIT_SIZE     64      /*!< A fixed hard coding setting. Do not change it!            */
#define MEM_POOL_UNIT_NUM     256      /*!< Increase this or heap size if memory allocate failed.     */

/*----------------------------------------------------------------------------------------*/
/*   Re-defined staff for various compiler                                                */
/*----------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#define   __inline    inline
#endif


/*----------------------------------------------------------------------------------------*/
/*   Debug settings                                                                       */
/*----------------------------------------------------------------------------------------*/
//#define ENABLE_ERROR_MSG                    /* enable debug messages                      */
//#define ENABLE_DEBUG_MSG                    /* enable debug messages                      */
//#define ENABLE_VERBOSE_DEBUG              /* verbos debug messages                      */
//#define DUMP_DESCRIPTOR                     /* dump descriptors                           */

extern int usbh_printf(const char *fmt, ...); ///< For USB Library Usage: debug print function

#ifdef ENABLE_ERROR_MSG
#define USB_error            (void)usbh_printf
#else
#define USB_error(...)
#endif

#ifdef ENABLE_DEBUG_MSG
#define USB_debug            (void)usbh_printf
#ifdef ENABLE_VERBOSE_DEBUG
#define USB_vdebug         (void)usbh_printf
#else
#define USB_vdebug(...)
#endif
#else
#define USB_debug(...)
#define USB_vdebug(...)
#endif


/// @endcond HIDDEN_SYMBOLS

#endif  /* _USBH_CONFIG_H_ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

