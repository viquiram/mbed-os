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
* $FileName: usb_device_stack_interface.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* 
*
*END************************************************************************/
/* Prototypes */
#ifndef __usb_device_stack_interface_h__
#define __usb_device_stack_interface_h__

#include "usb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Informational Request/Set Types */
#define  USB_STATUS_DEVICE_STATE               (0x01)
#define  USB_STATUS_INTERFACE                  (0x02)
#define  USB_STATUS_ADDRESS                    (0x03)
#define  USB_STATUS_CURRENT_CONFIG             (0x04)
#define  USB_STATUS_SOF_COUNT                  (0x05)
#define  USB_STATUS_DEVICE                     (0x06)
#define  USB_STATUS_TEST_MODE                  (0x07)
#define  USB_STATUS_ENDPOINT                   (0x10)
#define  USB_STATUS_ENDPOINT_NUMBER_MASK       (0x0F)

#define  USB_TEST_MODE_TEST_PACKET             (0x0400)

/* Available service types */
/* Services 0 through 15 are reserved for endpoints */
#define  USB_SERVICE_EP0                       (0x00)
#define  USB_SERVICE_EP1                       (0x01)
#define  USB_SERVICE_EP2                       (0x02)
#define  USB_SERVICE_EP3                       (0x03)
#define  USB_SERVICE_BUS_RESET                 (0x10)
#define  USB_SERVICE_SUSPEND                   (0x11)
//#define  USB_SERVICE_SOF                                   (0x12)
#define  USB_SERVICE_RESUME                    (0x13)
#define  USB_SERVICE_SLEEP                     (0x14)
#define  USB_SERVICE_SPEED_DETECTION           (0x15)
#define  USB_SERVICE_ERROR                     (0x16)
//#define  USB_SERVICE_STALL                                (0x17)
#define  USB_SERVICE_REQUEST                   (0x18)

#define  USB_CONTROL_ENDPOINT                  (0)
#define  USB_SETUP_PKT_SIZE                    (8)/* Setup Packet Size */
#define  USB_UNINITIALIZED_VAL_32              (0xFFFFFFFF)


#define  USB_DEV_EVENT_BUS_RESET               (0)
#define  USB_DEV_EVENT_CONFIG_CHANGED          (1)
#define  USB_DEV_EVENT_INTERFACE_CHANGED       (2)
#define  USB_DEV_EVENT_ENUM_COMPLETE           (3)
#define  USB_DEV_EVENT_SEND_COMPLETE           (4)
#define  USB_DEV_EVENT_DATA_RECEIVED           (5)
#define  USB_DEV_EVENT_ERROR                   (6)
#define  USB_DEV_EVENT_GET_DATA_BUFF           (7)
#define  USB_DEV_EVENT_EP_STALLED              (8)
#define  USB_DEV_EVENT_EP_UNSTALLED            (9)
#define  USB_DEV_EVENT_GET_TRANSFER_SIZE       (0x10)
#define  USB_DEV_EVENT_TYPE_SET_REMOTE_WAKEUP  (0x11)
#define  USB_DEV_EVENT_TYPE_CLR_REMOTE_WAKEUP  (0x12)
#define  USB_DEV_EVENT_TYPE_SET_EP_HALT        (0x13)
#define  USB_DEV_EVENT_TYPE_CLR_EP_HALT        (0x14)

/* Macros for description of class, configuration, interface */
#define  USB_DESC_INTERFACE(index, ep_cnt, ep) \
{ \
    index, \
    { \
        ep_cnt, \
        ep \
    } \
}
#define  USB_DESC_CONFIGURATION(intf_cnt, intf) \
{ \
    intf_cnt, \
    intf \
}
#define  USB_DESC_CLASS(type, config) \
{ \
    type, \
    config, \
}

/* Go through each endpoint in class */
#define  for_each_ep_in_class(epPtr, usbclassPtr, classtype) \
    for(uint32_t class_i = 0; usbclassPtr[class_i].type != USB_CLASS_INVALID; class_i++) \
        if((usbclassPtr[class_i].type == classtype) || (classtype == USB_CLASS_ALL)) \
        for(uint32_t intf_i = 0; intf_i < usbclassPtr[class_i].interfaces.count; intf_i++) \
            for(uint32_t ep_i = 0; \
                ((ep_i < usbclassPtr[class_i].interfaces.interface[intf_i].endpoints.count) && \
                ((epPtr = usbclassPtr[class_i].interfaces.interface[intf_i].endpoints.ep + ep_i) != NULL)); \
                ep_i++)

/* Go through each interface in class */
#define  for_each_if_in_class(ifPtr, usbclassPtr, classtype) \
                for(uint32_t class_i = 0; usbclassPtr[class_i].type != USB_CLASS_INVALID; class_i++) \
                    if((usbclassPtr[class_i].type == classtype) || (classtype == USB_CLASS_ALL)) \
                    for(uint32_t intf_i = 0; ((intf_i < usbclassPtr[class_i].interfaces.count) \
                        && ((ifPtr = usbclassPtr[class_i].interfaces.interface + intf_i) != NULL)); \
                        intf_i++)

typedef enum {
    USB_CLASS_INFO  = 0,
    USB_COMPOSITE_INFO,    
    USB_AUDIO_UNITS,            
    USB_RNDIS_INFO,
    USB_PHDC_QOS_INFO,
    USB_MSC_LBA_INFO,
} ENTITY_TYPE;

typedef enum {
    USB_CLASS_HID  = 0,
    USB_CLASS_CDC,    
    USB_CLASS_MSC,         
    USB_CLASS_AUDIO,          
    USB_CLASS_PHDC,
    USB_CLASS_ALL,
    USB_CLASS_INVALID
} class_type;

typedef struct _USB_EP_STRUCT
{
    uint8_t               ep_num;      /* endpoint number         */
    uint8_t               type;        /* type of endpoint        */
    uint8_t               direction;   /* direction of endpoint   */
    uint32_t              size;        /* buffer size of endpoint */
} USB_EP_STRUCT, * USB_EP_STRUCT_PTR;

/* Strucutre Representing Endpoints and number of endpoints user want*/
typedef struct _USB_ENDPOINTS
{
    uint8_t               count;
    USB_EP_STRUCT*        ep; 
} USB_ENDPOINTS;

/* Strucutre Representing interface*/
typedef struct _USB_IF_STRUCT
{
    uint8_t               index;
    USB_ENDPOINTS         endpoints;
} USB_IF_STRUCT;

/* Strucutre Representing how many interfaces in one class type*/
typedef struct _USB_INTERFACES_STRUCT
{
    uint8_t               count;
    USB_IF_STRUCT*        interface;
} USB_INTERFACES_STRUCT, * USB_INTERFACES_STRUCT_PTR;

/* Strucutre Representing class info*/
typedef struct _USB_CLASS_STRUCT
{
    class_type            type;
    USB_INTERFACES_STRUCT interfaces;
} USB_CLASS_STRUCT, * USB_CLASS_STRUCT_PTR;

/* Strucutre Representing composite info*/
typedef struct _USB_COMPOSITE_INFO_STRUCT
{
    uint8_t               count;
    USB_CLASS_STRUCT*     class;
} USB_COMPOSITE_INFO_STRUCT, * USB_COMPOSITE_INFO_STRUCT_PTR;

/* Common Data Structures */
typedef struct _USB_SETUP_STRUCT
{
    uint8_t               request_type;
    uint8_t               request;
    uint16_t              value;
    uint16_t              index;
    uint16_t              length;
} USB_SETUP_STRUCT, * USB_SETUP_STRUCT_PTR;

/* USB Specs define CONTROL_MAX_PACKET_SIZE for High Speed device as only 64,
   whereas for FS its allowed to be 8, 16, 32 or 64 */
#define CONTROL_MAX_PACKET_SIZE       (64)

#if (HIGH_SPEED_DEVICE && (CONTROL_MAX_PACKET_SIZE != 64))
#error "For High Speed CONTROL_MAX_PACKET_SIZE should be 64"
#endif

typedef struct _USB_EVENT_STRUCT
{
    _usb_device_handle    handle;             /* conttroler device handle*/
    uint8_t               ep_num;             /* endpoint number */
    bool                  setup;              /* is setup packet         */
    bool                  direction;          /* direction of endpoint   */
    uint8_t*              buffer_ptr;         /* void* to buffer       */
    uint32_t              len;                /* the buffer len had been done */
                                              /* special case: 0xFFFFFFFF means transfer cancel
                                                                                                0xFFFFFFFE means tansfer error */
} USB_EVENT_STRUCT, *PTR_USB_EVENT_STRUCT;

/* callback function pointer structure for Application to handle events */
typedef void(_CODE_PTR_ USB_DEVICE_NOFIFY)(uint8_t event, void* val, void* arg);


/* callback function pointer structure to handle USB framework request */
typedef uint8_t (_CODE_PTR_ USB_REQUEST_NOTIFY)(USB_SETUP_STRUCT *,
                                          uint8_t **,
                                          uint32_t*,void* arg);

typedef void(_CODE_PTR_ USB_EVENT_SERVICE)(PTR_USB_EVENT_STRUCT, void*);


typedef struct _usb_desc_request_notify_struct
 {
	uint8_t (_CODE_PTR_ GET_DESC)(uint32_t handle,uint8_t type,uint8_t desc_index,
		uint16_t index,uint8_t * *descriptor,uint32_t *size);  
	uint8_t (_CODE_PTR_ GET_DESC_INTERFACE)(uint32_t handle,uint8_t interface,
		uint8_t * alt_interface);
	uint8_t (_CODE_PTR_ SET_DESC_INTERFACE)(uint32_t handle,uint8_t interface,
		uint8_t alt_interface);
	uint8_t (_CODE_PTR_ SET_CONFIGURATION)(uint32_t handle, uint8_t config);  
	uint8_t (_CODE_PTR_ GET_DESC_ENTITY)(uint32_t handle, ENTITY_TYPE type, uint32_t * object);

 }DESC_REQUEST_NOFIFY_STRUCT, * DESC_REQUEST_NOFIFY_STRUCT_PTR;

/* Structure application request class callback */
typedef struct usb_application_callback_struct
{
    USB_DEVICE_NOFIFY     callback;
    void*                 arg;
}USB_APPLICATION_CALLBACK_STRUCT,* USB_APPLICATION_CALLBACK_STRUCT_PTR ;

/* Structure vendor request class callback */
typedef struct usb_vendor_req_callback_struct
{
    USB_REQUEST_NOTIFY    callback;
    void*                 arg;
}USB_VENDOR_REQ_CALLBACK_STRUCT,* USB_VENDOR_REQ_CALLBACK_STRUCT_PTR ;

extern USB_STATUS usb_device_init(uint8_t, _usb_device_handle * );
extern USB_STATUS usb_device_deinit(_usb_device_handle);
extern USB_STATUS usb_device_recv_data(_usb_device_handle, uint8_t, uint8_t *, uint32_t);
extern USB_STATUS usb_device_send_data(_usb_device_handle, uint8_t, uint8_t *, uint32_t);
extern USB_STATUS usb_device_cancel_transfer(_usb_device_handle, uint8_t, uint8_t);
extern USB_STATUS usb_device_register_service(_usb_device_handle, uint8_t, USB_EVENT_SERVICE, void* arg);
extern USB_STATUS usb_device_unregister_service(_usb_device_handle, uint8_t);
extern USB_STATUS usb_device_assert_resume(_usb_device_handle);
extern USB_STATUS usb_device_init_endpoint(_usb_device_handle, USB_EP_STRUCT_PTR, uint8_t);
extern USB_STATUS usb_device_stall_endpoint(_usb_device_handle, uint8_t, uint8_t);
extern USB_STATUS usb_device_unstall_endpoint(_usb_device_handle, uint8_t, uint8_t);
extern USB_STATUS usb_device_deinit_endpoint(_usb_device_handle, uint8_t, uint8_t);
extern USB_STATUS usb_device_register_application_notify(_usb_device_handle, USB_DEVICE_NOFIFY, void*);
extern USB_STATUS usb_device_register_vendor_class_request_notify(_usb_device_handle, USB_REQUEST_NOTIFY, void*);
extern USB_STATUS usb_device_register_desc_request_notify(_usb_device_handle,DESC_REQUEST_NOFIFY_STRUCT_PTR, void*);

#ifdef __cplusplus
}
#endif

#endif
