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

#ifndef _USB_CLASS_HID_H
#define _USB_CLASS_HID_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_class.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/

 
 /* class specific requests */
#define USB_HID_GET_REPORT_REQUEST     (0x01)
#define USB_HID_GET_IDLE_REQUEST       (0x02)
#define USB_HID_GET_PROTOCOL_REQUEST   (0x03)
#define USB_HID_SET_REPORT_REQUEST     (0x09)
#define USB_HID_SET_IDLE_REQUEST       (0x0A)
#define USB_HID_SET_PROTOCOL_REQUEST   (0x0B)
#define HIGH_BYTE_SHIFT                (8) 
#define MSB_MASK                       (0xFF00)
#define USB_HID_REQUEST_DIR_MASK       (0x08)
#define USB_HID_REQUEST_TYPE_MASK      (0x01)

/*****************************************************************************
 * Local Functions
 *****************************************************************************/
#define USB_REQ_VAL_INVALID				 (0xFFFF)

 /******************************************************************************
 * Types
 *****************************************************************************/
 typedef uint32_t HID_HANDLE; 
 
 /* Structure used to configure HID class by APP*/
 typedef struct hid_config_struct
 {
    USB_APPLICATION_CALLBACK_STRUCT            hid_application_callback;
    USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback;
    USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
    DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr; 
 }HID_CONFIG_STRUCT,* HID_CONFIG_STRUCT_PTR;

 
/******************************************************************************
 * Global Functions
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Init
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param *handle: handle pointer to Identify the controller
 * @param hid_class_callback:   event callback      
 * @param vendor_req_callback:  vendor specific class request callback      
 * @param param_callback:       application params callback      
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 *
 *This function initializes the HID Class layer and layers it is dependent on 
 *
 *****************************************************************************/
extern  USB_STATUS USB_Class_HID_Init
(
	uint8_t               controller_id,
	HID_CONFIG_STRUCT_PTR hid_config_ptr,
	HID_HANDLE *          hidHandle
);

extern USB_STATUS USB_Class_HID_Deinit
(
 HID_HANDLE   hidHandle
);

/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Send_Data
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
extern uint8_t USB_Class_HID_Send_Data
(
    HID_HANDLE         handle,        /* [IN]*/
    uint8_t            ep_num,        /* [IN]*/
    uint8_t*           buff_ptr,      /* [IN] buffer to send */      
    uint32_t           size           /* [IN] length of the transfer */
);

#define USB_HID_Periodic_Task USB_Class_Periodic_Task

#endif

/* EOF */
