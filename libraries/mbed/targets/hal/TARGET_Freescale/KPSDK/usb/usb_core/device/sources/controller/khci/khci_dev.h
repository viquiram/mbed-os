/**HEADER********************************************************************
*
* Copyright (c) 2009, 2013 Freescale Semiconductor;
* All Rights Reserved
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: khci_dev.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*   This file contains the macros, function prototypes and data structure
*   definitions required by the Full Speed USB Device Controller driver.
*
*END************************************************************************/

#ifndef __khci_dev_h__
#define __khci_dev_h__

#define KHCI_MAX_ENDPOINT             (16)

/***************************************
**
** Prototypes
**/
#ifdef __cplusplus
extern "C" {
#endif

USB_STATUS usb_dci_khci_preinit(_usb_device_handle, _usb_device_handle *);
USB_STATUS usb_dci_khci_init(uint8_t, _usb_device_handle);
USB_STATUS usb_dci_khci_send(_usb_device_handle, XD_STRUCT_PTR);
USB_STATUS usb_dci_khci_recv(_usb_device_handle, XD_STRUCT_PTR);
USB_STATUS usb_dci_khci_cancel(_usb_device_handle, uint8_t, uint8_t);
USB_STATUS usb_dci_khci_set_addr(_usb_device_handle, uint8_t);
USB_STATUS usb_dci_khci_shutdown(_usb_device_handle);
USB_STATUS usb_dci_khci_get_setup_data(_usb_device_handle, uint8_t, uint8_t *);
USB_STATUS usb_dci_khci_assert_resume(_usb_device_handle);
USB_STATUS usb_dci_khci_init_endpoint(_usb_device_handle, XD_STRUCT_PTR);
USB_STATUS usb_dci_khci_stall_endpoint(_usb_device_handle, uint8_t, uint8_t);
USB_STATUS usb_dci_khci_unstall_endpoint(_usb_device_handle, uint8_t, uint8_t);
USB_STATUS usb_dci_khci_deinit_endpoint(_usb_device_handle, uint8_t, uint8_t);
USB_STATUS usb_dci_khci_get_endpoint_status(_usb_device_handle, uint8_t, uint16_t *);
USB_STATUS usb_dci_khci_set_endpoint_status(_usb_device_handle, uint8_t, uint16_t);
USB_STATUS usb_dci_khci_get_transfer_status(_usb_device_handle,uint8_t,uint8_t);
USB_STATUS usb_dci_khci_get_status(_usb_device_handle, uint8_t, uint16_t *);
USB_STATUS usb_dci_khci_set_status(_usb_device_handle, uint8_t, uint16_t);
USB_STATUS usb_dci_khci_alloc_xd(_usb_device_handle, XD_STRUCT_PTR*);


#ifdef __cplusplus
}
#endif

#endif
