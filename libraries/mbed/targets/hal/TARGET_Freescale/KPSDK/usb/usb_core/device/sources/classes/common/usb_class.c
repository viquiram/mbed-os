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
* $FileName: usb_class.c$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack Class module implimentation.
*
*****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"

#include "usb_device_stack_interface.h"

#if USBCFG_DEV_HID || USBCFG_DEV_PHDC || USBCFG_DEV_AUDIO || USBCFG_DEV_CDC || USBCFG_DEV_MSC

#include "usb_class_internal.h"
#include "usb_framework.h"

/*****************************************************************************
 * Constant and Macro's
 *****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/
static USB_CLASS_OBJECT_STRUCT_PTR usb_class_object[USBCFG_DEV_MAX_CLASS_OBJECT];
#if USBCFG_DEV_COMPOSITE
static USB_CLASS_HANDLE s_class_handle = USB_UNINITIALIZED_VAL_32;
#endif
/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes 
 *****************************************************************************/

/*****************************************************************************
 * Local Variables 
 *****************************************************************************/ 
 
 /*****************************************************************************
 * Local Functions - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions
 *****************************************************************************/

 /*************************************************************************//*!
 *
 * @name  USB_Class_Allocate_Handle
 *
 * @brief The funtion reserves entry in device array and returns the index.
 *
 * @param none.
 * @return returns the reserved handle or if no entry found device busy.      
 *
 *****************************************************************************/
static USB_CLASS_HANDLE  USB_Class_Allocate_Handle()
{
    int32_t cnt = 0;
    for (;cnt< USBCFG_DEV_MAX_CLASS_OBJECT;cnt++)
    {
       if (usb_class_object[cnt] == NULL)
        return cnt;
    }
    return USBERR_DEVICE_BUSY;
}

 /*************************************************************************//*!
 *
 * @name  USB_Class_Free_Handle
 *
 * @brief The funtion releases entry in device array .
 *
 * @param handle  index in device array to be released..
 * @return returns and error code or USB_OK.      
 *
 *****************************************************************************/
static int32_t USB_Class_Free_Handle(USB_CLASS_HANDLE handle)
{
    if (handle > USBCFG_DEV_MAX_CLASS_OBJECT)
        return USBERR_ERROR;

    usb_class_object[handle] = NULL;
    return USB_OK;
}
 /*************************************************************************//*!
 *
 * @name  Get_Usb_Class_Object_Ptr
 *
 * @brief The funtion gets the class object pointer from class array .
 *
 * @param handle  index in class object array.
 * @return returns returns pointer to USB Class Object structure..      
 *
 *****************************************************************************/
static USB_CLASS_OBJECT_STRUCT_PTR Get_Usb_Class_Object_Ptr(USB_CLASS_HANDLE handle)
{
     return usb_class_object[handle]; 
}

/**************************************************************************//*!
 *
 * @name  USB_Class_Init
 *
 * @brief The funtion initializes the Class Module 
 *
 * @param handle             :handle to Identify the controller
 * @param class_callback     :event callback      
 * @param other_req_callback :call back for class/vendor specific requests on 
 *                            CONTROL ENDPOINT
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
USB_CLASS_HANDLE USB_Class_Init
(
    _usb_device_handle handle, /* [IN] the USB device controller to initialize */                  
    USB_DEVICE_NOFIFY class_callback,/*[IN]*/
    USB_REQUEST_NOTIFY other_req_callback,/*[IN]*/
    void* user_arg,/*[IN]*/
    DESC_REQUEST_NOFIFY_STRUCT_PTR  desc_callback_ptr/*[IN]*/
 ) 
{
    USB_CLASS_HANDLE  class_handle;
    USB_CLASS_OBJECT_STRUCT_PTR class_object_ptr = NULL;
    
    class_object_ptr = (USB_CLASS_OBJECT_STRUCT_PTR)OS_Mem_alloc_zero(
        sizeof(USB_CLASS_OBJECT_STRUCT));
    if (NULL == class_object_ptr)
    {
        #if _DEBUG
            printf("USB_Class_Init: Memalloc failed\n");
        #endif  
        return USBERR_ALLOC;
    }
    
    class_handle = USB_Class_Allocate_Handle();
    if (USBERR_DEVICE_BUSY == class_handle) 
    {
        OS_Mem_free((void*)class_object_ptr);
        class_object_ptr = NULL;
        return USBERR_DEVICE_BUSY;
    }
        
    class_object_ptr->controller_handle = handle;
    class_object_ptr->class_callback = class_callback;
    class_object_ptr->arg = user_arg;

    usb_device_register_application_notify(handle, class_callback, user_arg);
    usb_device_register_vendor_class_request_notify(handle, other_req_callback, user_arg);
    usb_device_register_desc_request_notify(handle, desc_callback_ptr, user_arg);

    usb_class_object[class_handle] = class_object_ptr;     
#if USBCFG_DEV_COMPOSITE
	/* Suppose only one class handle can be assigned */
	s_class_handle = class_handle;
#endif
    return class_handle;     
}

/**************************************************************************//*!
 *
 * @name  USB_Class_Deinit
 *
 * @brief The funtion initializes the Class Module 
 *
 * @param handle             :handle to Identify the controller
 * @param class_handle       :class handle      
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
uint8_t USB_Class_Deinit
(
    _usb_device_handle handle, /* [IN] the USB device controller to initialize */                  
    USB_CLASS_HANDLE  class_handle
 ) 
{
	USB_CLASS_OBJECT_STRUCT_PTR class_obj_ptr;
    uint8_t error = USB_OK;
	
	class_obj_ptr = usb_class_object[class_handle];
    
    OS_Mem_free((void*)class_obj_ptr);
    
    error = USB_Class_Free_Handle(class_handle);

#if USBCFG_DEV_COMPOSITE
	/* Suppose only one class handle can be assigned */
	s_class_handle = USB_UNINITIALIZED_VAL_32;
#endif
    
    return error;     
}

/**************************************************************************//*!
 *
 * @name  USB_Class_Send_Data
 *
 * @brief The funtion calls the device to send data upon recieving an IN token 
 *
 * @param handle:               handle to Identify the controller
 * @param ep_num:               The endpoint number     
 * @param buff_ptr:             buffer to send
 * @param size:                 length of transfer
 * 
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
uint8_t USB_Class_Send_Data
(
    USB_CLASS_HANDLE handle, /*[IN]*/
    uint8_t ep_num,        /* [IN] the Endpoint number */                  
    uint8_t * buff_ptr,      /* [IN] buffer to send */      
    uint32_t size           /* [IN] length of the transfer */
) 
{
    uint8_t error = USB_OK;
    //uint16_t  state;
    USB_CLASS_OBJECT_STRUCT_PTR class_object_ptr;   
    
    class_object_ptr = Get_Usb_Class_Object_Ptr(handle); 
    if (NULL == class_object_ptr) 
        return USBERR_ERROR;

    error = usb_device_send_data(class_object_ptr->controller_handle,
                ep_num,buff_ptr,size);

    return error;      
}
 
/**************************************************************************//*!
 *
 * @name   USB_Class_Periodic_Task
 *
 * @brief  The funtion calls for periodic tasks 
 *
 * @param  None
 *
 * @return None
 *
 *****************************************************************************/
void USB_Class_Periodic_Task(void) 
{
    #ifdef DELAYED_PROCESSING  
        USB_Framework_Periodic_Task();
    #endif  
}
/**************************************************************************//*!
 *
 * @name  USB_Class_Get_Desc
 *
 * @brief  This function is called in to get the discriptor as specified in cmd.
 *
 * @param handle:           USB class handle. Received from
 *                          USB_Class_Init      
 * @param cmd:              command for USB discriptor to get.
 * @param in_data:          input to the Application functions.
 * @param in_buff           buffer which will contian the discriptors.
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
uint8_t USB_Class_Get_Desc(USB_CLASS_HANDLE handle,/*[IN]*/
int32_t cmd,/*[IN]*/
uint8_t input_data,/*[IN]*/
uint8_t * *in_buf /*[OUT]*/
)
{
   USB_CLASS_OBJECT_STRUCT_PTR class_obj_ptr;
    
   if (handle > USBCFG_DEV_MAX_CLASS_OBJECT)
      return USBERR_ERROR;
   
   class_obj_ptr = Get_Usb_Class_Object_Ptr(handle);
   if (class_obj_ptr == NULL)
        return USBERR_ERROR;
   
   return USB_Framework_GetDesc(class_obj_ptr->usb_fw_handle,cmd,input_data,in_buf);
            
}
/**************************************************************************//*!
 *
 * @name  USB_Class_Set_Desc
 *
 * @brief  This function is called in to Set the discriptor as specified in cmd.
 *
 * @param handle:           USB class handle. Received from
 *                          USB_Class_Init      
 * @param cmd:              command for USB discriptor to get.
 * @param in_data:          input to the Application functions.
 * @param in_buff           buffer which will contian the discriptors.
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
uint8_t USB_Class_Set_Desc(USB_CLASS_HANDLE handle,/*[IN]*/
int32_t cmd,/*[IN]*/
uint8_t input_data,/*[IN]*/
uint8_t * *out_buf /*[IN]*/
)
{
   USB_CLASS_OBJECT_STRUCT_PTR class_obj_ptr;
    
   if (handle > USBCFG_DEV_MAX_CLASS_OBJECT)
      return USBERR_ERROR;
   
   class_obj_ptr = Get_Usb_Class_Object_Ptr(handle);
   if (class_obj_ptr == NULL)
        return USBERR_ERROR;
   
   return USB_Framework_SetDesc(class_obj_ptr->usb_fw_handle,cmd,input_data,out_buf);
            
}
#if USBCFG_DEV_COMPOSITE
/**************************************************************************//*!
 *
 * @name  USB_Class_Get_Class_Handle
 *
 * @brief  This function is called to return class handle.
 *
 * @return value:
 *                        class handle
 *
 *****************************************************************************/
USB_CLASS_HANDLE USB_Class_Get_Class_Handle()
{
    return s_class_handle;
}

/**************************************************************************//*!
 *
 * @name  USB_Class_Get_Ctrler_Handle
 *
 * @brief  This function is called to return controller handle.
 *
 * @return value:
 *                        controller handle
 *
 *****************************************************************************/
_usb_device_handle USB_Class_Get_Ctrler_Handle(USB_CLASS_HANDLE class_handle)
{
    _usb_device_handle ret;
    if(USB_UNINITIALIZED_VAL_32 != class_handle)
        ret = usb_class_object[class_handle]->controller_handle;
    else
        ret = NULL; 
    return ret;
}
#endif

#endif
/* EOF */

