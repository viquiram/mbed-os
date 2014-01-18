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
* $FileName: usb_hid.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack HID class layer api header function.
*
*****************************************************************************/

#ifndef _USB_HID_H
#define _USB_HID_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_class_hid.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define HID_IMPLEMENT_QUEUING  0 
 /* class specific requests */

/* for class specific requests */
#define REPORT_SIZE                       (4)
#define CLASS_REQ_DATA_SIZE               (0x01)
#define MAX_HID_DEVICE                    (0x05)
#define HID_MAX_QUEUE_ELEMS               (4) 


/*****************************************************************************
 * Local Functions
 *****************************************************************************/
void USB_Service_Hid(PTR_USB_EVENT_STRUCT event,void* arg);
void USB_Class_Hid_Event(uint8_t event, void* val,void* arg);
uint8_t USB_HID_Requests(USB_SETUP_STRUCT * setup_packet, 
                           uint8_t * *data, 
                           uint32_t *size,void* arg);


 /******************************************************************************
 * Types
 *****************************************************************************/
     
/* structure to hold a request in the endpoint queue */
typedef struct _usb_class_hid_queue 
{
    _usb_device_handle                       handle;
    uint8_t*                                 app_buff; /* buffer to send */
    uint32_t                                 size; /* size of the transfer */
    uint8_t                                  channel; 
}USB_CLASS_HID_QUEUE, *PTR_USB_CLASS_HID_QUEUE;
 
/* USB class hid endpoint data */
typedef struct _usb_class_hid_endpoint 
{
    uint8_t                                  endpoint; /* endpoint num */                    
    uint8_t                                  type;     /* type of endpoint (interrupt, bulk or isochronous) */   
    uint8_t                                  bin_consumer;/* the num of queued elements */
    uint8_t                                  bin_producer;/* the num of de-queued elements */
    USB_CLASS_HID_QUEUE queue[HID_MAX_QUEUE_ELEMS]; /* queue data */  
}USB_CLASS_HID_ENDPOINT;

/* contains the endpoint data for non control endpoints */
typedef struct _usb_class_hid_endpoint_data 
{    
    uint8_t                                  count;  /* num of non control endpoints */     
    USB_CLASS_HID_ENDPOINT*                  ep;       
}USB_CLASS_HID_ENDPOINT_DATA, *PTR_USB_CLASS_HID_ENDPOINT_DATA;

/* Strucutre holding HID class state information*/
typedef struct hid_device_struct
{
  _usb_device_handle                         handle;
  uint32_t                                   user_handle;
  USB_CLASS_HANDLE                           class_handle;
  USB_ENDPOINTS*                             ep_desc_data;
  USB_APPLICATION_CALLBACK_STRUCT            hid_application_callback;
  USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback;
  USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
  DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr;
  USB_CLASS_HID_ENDPOINT_DATA                hid_endpoint_data;
  /* for get/set idle and protocol requests*/
  uint8_t                                    class_request_params[2]; 
}HID_DEVICE_STRUCT, * HID_DEVICE_STRUCT_PTR;
 
 
/******************************************************************************
 * Global Functions
 *****************************************************************************/

#endif

/* EOF */
