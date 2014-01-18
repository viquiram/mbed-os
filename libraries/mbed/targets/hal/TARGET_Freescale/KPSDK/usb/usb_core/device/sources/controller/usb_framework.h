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
* $FileName: usb_framework.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB Framework module api header function.
*
*****************************************************************************/

#ifndef _USB_FRAMEWORK_H
#define _USB_FRAMEWORK_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define MAX_EXPECTED_CONTROL_OUT_SIZE       (76 + USB_SETUP_PKT_SIZE) 
#define MAX_STRD_REQ            (13)  /* Max value of stadard request */
/* size of data to be returned for various Get Desc calls */
#define DEVICE_STATUS_SIZE      (2)
#define INTERFACE_STATUS_SIZE   (1)
#define CONFIG_SIZE             (1)
#define FRAME_SIZE              (2)
#define ENDP_STATUS_SIZE        (2)
#define USB_SET_REQUEST_MASK    (0x02) 

/******************************************************************************
 * Types
 *****************************************************************************/
typedef uint32_t USB_CLASS_FW_HANDLE;
 
/* Strucutre holding USB state information and handles.*/
typedef struct _usb_class_fw_object
{
   uint16_t                       std_framework_data;
   uint16_t                       assigned_address;
   _usb_device_handle             controller_handle;
   _usb_device_handle             dev_handle;
   USB_SETUP_STRUCT               setup_packet;
   uint8_t*                       ext_req_to_host;
   USB_DEVICE_NOFIFY              device_notify_callback;
   void*                          device_notify_param;
   USB_REQUEST_NOTIFY             request_notify_callback;
   void*                          request_notify_param;
   DESC_REQUEST_NOFIFY_STRUCT_PTR desc_notify_callback;
   void*                          desc_notify_param;
   uint16_t                       cur_config;
}USB_CLASS_FW_OBJECT_STRUCT,* USB_CLASS_FW_OBJECT_STRUCT_PTR;

 /* callback function pointer structure to handle USB framework request */
typedef uint8_t (_CODE_PTR_ USB_FW_REQ_FUNC)(USB_CLASS_FW_OBJECT_STRUCT_PTR usb_fw_ptr,
                                            USB_SETUP_STRUCT * setup_packet, 
                                            uint8_t * *data, 
                                            uint32_t *size); 

/******************************************************************************
 * Global Functions
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Framework_Periodic_Task
 *
 * @brief The funtion is called to respond to any control request
 *
 * @param None
 *
 * @return None       
 *
 *****************************************************************************/
#ifdef DELAYED_PROCESSING
extern void USB_Framework_Periodic_Task(void);
#endif
/**************************************************************************//*!
 *
 * @name  USB_Framework_Reset
 *
 * @brief The funtion resets the framework 
 *
 * @param handle: handle to Identify the controller
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
extern uint8_t USB_Framework_Reset
(
    _usb_device_handle handle /*[IN] the USB device controller to initialize*/ 
);

/**************************************************************************//*!
 *
 * @name  USB_Framework_GetDesc
 *
 * @brief  This function is called in to get the discriptor as specified in cmd.
 *
 * @param handle:           USB framework handle. Received from
 *                          USB_Framework_Init      
 * @param cmd:              command for USB discriptor to get.
 * @param in_data:          input to the Application functions.
 * @param in_buff           buffer which will contian the discriptors.
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
uint8_t USB_Framework_GetDesc(USB_CLASS_FW_HANDLE handle, /*[IN]*/
int32_t cmd,/*[IN]*/
uint8_t in_data,/*[IN]*/
uint8_t * * in_buff/*[OUT]*/
);

/**************************************************************************//*!
 *
 * @name  USB_Framework_SetDesc
 *
 * @brief  This function is called in to get the discriptor as specified in cmd.
 *
 * @param handle:           USB framework handle. Received from
 *                          USB_Framework_Init      
 * @param cmd:              command for USB discriptor to get.
 * @param in_data:          input to the Application functions.
 * @param outBuf           buffer which will contian the discriptors.
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
uint8_t USB_Framework_SetDesc(USB_CLASS_FW_HANDLE handle,/*[IN]*/
int32_t cmd,/*[IN]*/
uint8_t input_data,/*[IN]*/
uint8_t * * outBuf/*[IN]*/
);
/**************************************************************************//*!
 *
 * @name  USB_Framework_Remote_wakeup
 *
 * @brief  This function is called in to get the discriptor as specified in cmd.
 *
 * @param handle:           USB framework handle. Received from
 *                          USB_Framework_Init      
 * @param cmd:              command for USB discriptor to get.
 * @param in_data:          input to the Application functions.
 * @param in_buff           buffer which will contian the discriptors.
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
bool USB_Framework_Remote_wakeup(USB_CLASS_FW_HANDLE handle/*[IN]*/);

#endif

/* EOF */
