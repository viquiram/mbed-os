/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
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
* $FileName: usb_dev.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*  This file contains the declarations specific to the USB Device API
*
*END*********************************************************************/
#ifndef __USB_DEV_H__
#define __USB_DEV_H__ 1

#include "usb_framework.h"
/* Callback function storage structure */
typedef struct service_struct 
{
   uint8_t                          type;
   USB_EVENT_SERVICE                service;
   void*                            arg;
   struct service_struct*           next;
} SERVICE_STRUCT, * SERVICE_STRUCT_PTR;

typedef struct usb_dev_state_struct {
    uint8_t                         occupied;
    uint8_t                         controller_id;       /* Device controller ID */
    _usb_device_handle              controller_handle;
    SERVICE_STRUCT_PTR              service_head_ptr;    /* Head struct address of registered services */
    void*                           usb_dev_interface;
    USB_CLASS_FW_OBJECT_STRUCT      usb_framework; 
} USB_DEV_STATE_STRUCT, * USB_DEV_STATE_STRUCT_PTR;


typedef struct usb_dev_interface_functions_struct
{
   /* The Host/Device init function */
   USB_STATUS (_CODE_PTR_ DEV_PREINIT)(_usb_device_handle, _usb_device_handle *);

   /* The Host/Device init function */
   USB_STATUS (_CODE_PTR_ DEV_INIT)(uint8_t, _usb_device_handle);

   /* The function to send data */
   USB_STATUS (_CODE_PTR_ DEV_SEND)(_usb_device_handle, struct xd_struct *);

   /* The function to receive data */
   USB_STATUS (_CODE_PTR_ DEV_RECV)(_usb_device_handle, struct xd_struct *);
   
   /* The function to cancel the transfer */
   USB_STATUS (_CODE_PTR_ DEV_CANCEL_TRANSFER)(_usb_device_handle, uint8_t, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_INIT_ENDPOINT)(_usb_device_handle, struct xd_struct *);
   
   USB_STATUS (_CODE_PTR_ DEV_DEINIT_ENDPOINT)(_usb_device_handle, uint8_t, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_UNSTALL_ENDPOINT)(_usb_device_handle, uint8_t, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_GET_ENDPOINT_STATUS)(_usb_device_handle, uint8_t, uint16_t*);
   
   USB_STATUS (_CODE_PTR_ DEV_SET_ENDPOINT_STATUS)(_usb_device_handle, uint8_t, uint16_t);

   USB_STATUS (_CODE_PTR_ DEV_GET_TRANSFER_STATUS)(_usb_device_handle, uint8_t, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_SET_ADDRESS)(_usb_device_handle, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_SHUTDOWN)(_usb_device_handle);
   
   USB_STATUS (_CODE_PTR_ DEV_GET_SETUP_DATA)(_usb_device_handle, uint8_t, uint8_t *);
   
   USB_STATUS (_CODE_PTR_ DEV_ASSERT_RESUME)(_usb_device_handle);
   
   USB_STATUS (_CODE_PTR_ DEV_STALL_ENDPOINT)(_usb_device_handle, uint8_t, uint8_t);
   
   USB_STATUS (_CODE_PTR_ DEV_SET_DEVICE_STATUS)(_usb_device_handle, uint8_t, uint16_t);
   
   USB_STATUS (_CODE_PTR_ DEV_GET_DEVICE_STATUS)(_usb_device_handle, uint8_t, uint16_t*);

   USB_STATUS (_CODE_PTR_ DEV_GET_XD)(_usb_device_handle, XD_STRUCT_PTR *);

} USB_DEV_INTERFACE_FUNCTIONS_STRUCT, * USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR;

#ifdef __cplusplus
{
#endif
	



#ifdef __cplusplus
}
#endif

#endif
/* EOF */
