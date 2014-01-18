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
* $FileName: usb_class_phdc.h$
* $Version : 3.8.2.0$
* $Date    : Sep-19-2011$
*
* Comments:
*
* @brief The file contains USB stack PHDC class layer api header function.
*
*****************************************************************************/

#ifndef _USB_CLASS_PHDC_H
#define _USB_CLASS_PHDC_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_class.h"
//#include "usb_stack_config.h"
/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define USB_METADATA_SUPPORTED                  (0)
#define USB_DEV_EVENT_META_DATA_PARAMS_CHANGED  (0xF2)
#define USB_DEV_EVENT_FEATURE_CHANGED           (0xF3)
#define USB_Class_PHDC_Periodic_Task USB_Class_Periodic_Task
#if USB_METADATA_SUPPORTED
    #define META_DATA_MSG_PRE_IMPLEMENTED       (1)/*TRUE:1; FALSE:0*/
#else
    #define META_DATA_MSG_PRE_IMPLEMENTED       (0)/*TRUE:1; FALSE:0*/
#endif
/* Exception */
#define USB_REQ_VAL_INVALID             (0xFFFF)
/*****************************************************************************
 * Local Functions
 *****************************************************************************/

 /******************************************************************************
 * Types
 *****************************************************************************/
typedef uint32_t PHDC_HANDLE;
 
typedef struct _usb_class_phdc_rx_buff 
{
    uint8_t*  in_buff;  /* Pointer to input Buffer */
    uint32_t  in_size; /* Length of Input Buffer*/
    uint16_t  out_size; /* Size of Output Buffer */
    uint8_t*  out_buff; /* Pointer to Output Buffer */
    uint16_t  transfer_size;
#if USB_METADATA_SUPPORTED
    bool      meta_data_packet;/* meta data packet flag */
#endif
}USB_CLASS_PHDC_RX_BUFF, *PTR_USB_CLASS_PHDC_RX_BUFF;

/* event structures */ 
typedef struct _usb_app_event_send_complete 
{
    uint8_t   qos;
    uint8_t*  buffer_ptr;
    uint32_t  size;
}USB_APP_EVENT_SEND_COMPLETE, *PTR_USB_APP_EVENT_SEND_COMPLETE;

typedef struct _usb_app_event_data_recieved 
{
    uint8_t   qos;
    uint8_t*  buffer_ptr;
    uint32_t  size;
}USB_APP_EVENT_DATA_RECIEVED, *PTR_USB_APP_EVENT_DATA_RECIEVED;

/* Structures used to configure PHDC class by  APP*/
typedef struct phdc_config_struct
{
    USB_APPLICATION_CALLBACK_STRUCT            phdc_application_callback;
    USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback; 
    USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
    DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr;  
}PHDC_CONFIG_STRUCT,* PHDC_CONFIG_STRUCT_PTR;

/******************************************************************************
 * Global Functions
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Class_PHDC_Init
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param  phdc_config_ptr[IN]  : Phdc configuration structure pointer
 * @return PHDC_HANDLE      : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 * This function initializes the PHDC Class layer and layers it is dependednt on 
 *****************************************************************************/                          
extern USB_STATUS USB_Class_PHDC_Init
(
	uint8_t controller_id,	/*[IN]*/
    PHDC_CONFIG_STRUCT_PTR phdc_config_ptr, /*[IN]*/
    PHDC_HANDLE *  phdcHandle /*[OUT]*/
);

/**************************************************************************//*!
 *
 * @name  USB_Class_PHDC_Deinit
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_PHDC_Deinit   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
extern USB_STATUS USB_Class_PHDC_Deinit
(
 PHDC_HANDLE   phdcHandle
);

/**************************************************************************//*!
 *
 * @name  USB_PHDC_Class_Recv_Data
 *
 * @brief This fucntion is used by Application to receive data through PHDC class
 *
 * @param handle     :   Handle returned by USB_Class_PHDC_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 * This fucntion is used by Application to send data through PHDC class 
 *****************************************************************************/  
extern uint8_t USB_Class_PHDC_Recv_Data
(
    PHDC_HANDLE         handle,
    uint8_t             qos, 
    uint8_t*            buff_ptr,      /* [IN] buffer to send */      
    uint32_t            size           /* [IN] length of the transfer */
);
/**************************************************************************//*!
 *
 * @name  USB_Class_PHDC_Send_Data
 *
 * @brief This fucntion is used by Application to send data through PHDC class
 *
 * @param handle          :   handle returned by USB_Class_PHDC_Init
 * @param meta_data       :   packet is meta data or not
 * @param num_tfr         :   no. of transfers
 * @param qos             :   current qos of the transfer
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 * This fucntion is used by Application to send data through PHDC class 
 *****************************************************************************/  
extern uint8_t USB_Class_PHDC_Send_Data
(
    PHDC_HANDLE      handle,    
    bool             meta_data,    /* opaque meta data in app buffer */
    uint8_t          num_tfr,      /* no. of transfers to follow with given 
                                                            channel--only valid if meta data is 
                                                            true */
    uint8_t          current_qos,  /* qos of the transfers to follow--only 
                                                             valid if meta data is true */
    uint8_t*         app_buff,     /* buffer holding application data */
    uint32_t         size          /* [IN] length of the transfer */
);
                               
#define USB_PHDC_Periodic_Task USB_Class_Periodic_Task

#endif

/* EOF */
