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
* $FileName: usb_cdc.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack CDC class layer api header function.
*
*****************************************************************************/

#ifndef _USB_CDC_H
#define _USB_CDC_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_class.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/
#define CDC_IMPLEMENT_QUEUING            (1)/* 1: TRUE; 0: FALSE*/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define MDLM_SPECIFIC_NOTIF_MASK         (0x5F)

/* other macros */
#define NOTIF_PACKET_SIZE                (0x08)
#define NOTIF_REQUEST_TYPE               (0xA1)
#define PSTN_SUBCLASS_NOTIF_SUPPORT      (1)/*TRUE*/

/* macros for queuing */
#define CDC_MAX_QUEUE_ELEMS              (16)  

#define MAX_CDC_DEVICE                   (0x01)
 
/*****************************************************************************
 * Local Functions
 *****************************************************************************/
void USB_Service_Cdc_Notif(PTR_USB_EVENT_STRUCT event,void* arg);
void USB_Service_Dic_Bulk_In(PTR_USB_EVENT_STRUCT event,void* arg);
void USB_Service_Dic_Bulk_Out(PTR_USB_EVENT_STRUCT event,void* arg);

void USB_Class_CDC_Event(uint8_t event, void* val,void* arg);
uint8_t USB_CDC_Other_Requests(USB_SETUP_STRUCT * setup_packet, uint8_t * *data, uint32_t *size,void* arg);

/******************************************************************************
 * Types
 *****************************************************************************/
typedef enum
{
   USB_EP_COUNT = 0,
   USB_CDC_EP_COUNT,
   USB_INTERFACE_COUNT,
   USB_CDC_INTERFACE_COUNT
}USB_DESC_INFO_T;

/* structure to hold a request in the endpoint queue */
typedef struct _usb_class_cdc_queue 
{
    _usb_device_handle                         handle;
    uint8_t                                    channel;       
    CDC_APP_DATA_STRUCT                        app_data;  
}USB_CLASS_CDC_QUEUE, *PTR_USB_CLASS_CDC_QUEUE;
 
/* USB class cdc endpoint data */
typedef struct _usb_class_cdc_endpoint 
{
    uint8_t                                    endpoint; /* endpoint num */                    
    uint8_t                                    type;     /* type of endpoint (interrupt, bulk or isochronous) */   
    uint8_t                                    bin_consumer;/* the num of queued elements */
    uint8_t                                    bin_producer;/* the num of de-queued elements */
    USB_CLASS_CDC_QUEUE                        queue[CDC_MAX_QUEUE_ELEMS]; /* queue data */  
}USB_CLASS_CDC_ENDPOINT;

typedef struct _cdc_variable_struct
{
    CDC_HANDLE                                 cdc_handle;
    USB_CLASS_HANDLE                           class_handle;
    _usb_device_handle                         controller_handle;
    USB_ENDPOINTS*                             usb_ep_data;
    uint32_t                                   comm_feature_data_size;
    uint8_t                                    cic_send_endpoint;
    uint8_t                                    cic_recv_endpoint;
    uint8_t                                    dic_send_endpoint;
    uint8_t                                    dic_recv_endpoint;
    uint32_t                                   dic_recv_pkt_size;
    uint32_t                                   dic_send_pkt_size;
    uint32_t                                   cic_send_pkt_size;    
    /* Holds the PSTN object*/
    void*                                      pstn_obj_ptr;
    USB_APPLICATION_CALLBACK_STRUCT            cdc_application_callback;
    USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback;
	USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
    /* contains the endpoint info */
    USB_CLASS_CDC_ENDPOINT*                    ep;
    DESC_REQUEST_NOFIFY_STRUCT                 desc_callback;
    uint8_t                                    max_supported_interfaces;
#if USBCFG_DEV_RNDIS_SUPPORT   
    /* rndis specific configuration */
	USB_RNDIS_INFO_STRUCT                      rndis_info;
#endif
    
 }CDC_DEVICE_STRUCT, * CDC_DEVICE_STRUCT_PTR;
 
/******************************************************************************
 * Global Functions of Subclass
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Pstn_Init
 *
 * @brief The funtion initializes the Pstn Module 
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param class_callback:       event callback 
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
uint8_t USB_Pstn_Init
(
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
    USB_APPLICATION_CALLBACK_STRUCT_PTR pstn_cb
); 
/**************************************************************************//*!
 *
 * @name  USB_Pstn_Deinit
 *
 * @brief The funtion initializes the Pstn Module 
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param class_callback:       event callback 
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *
 *****************************************************************************/
uint8_t USB_Pstn_Deinit
(
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr
);
/**************************************************************************//*!
 *
 * @name  PSTN_Get_Line_Coding
 *
 * @brief  This function is called in response to GetLineCoding request
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/ 
extern uint8_t PSTN_Get_Line_Coding(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                   USB_SETUP_STRUCT * setup_packet,
                                   uint8_t * *data, 
                                   uint32_t *size);
                                   
/**************************************************************************//*!
 *
 * @name  PSTN_Set_Line_Coding
 *
 * @brief  This function is called in response to SetLineCoding request
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/ 
extern uint8_t PSTN_Set_Line_Coding(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                   USB_SETUP_STRUCT * setup_packet,
                                   uint8_t * *data, 
                                   uint32_t *size);  
                                   
/**************************************************************************//*!
 *
 * @name  PSTN_Set_Ctrl_Line_State
 *
 * @brief  This function is called in response to Set Control Line State 
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/ 
extern uint8_t PSTN_Set_Ctrl_Line_State(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                       USB_SETUP_STRUCT * setup_packet, 
                                       uint8_t * *data, 
                                       uint32_t *size);
                                       
/**************************************************************************//*!
 *
 * @name  PSTN_Send_Break
 *
 * @brief  This function is called in response to Set Config request
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/ 
 extern uint8_t PSTN_Send_Break(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                              USB_SETUP_STRUCT * setup_packet, 
                              uint8_t * *data, 
                              uint32_t *size);  
                              
/**************************************************************************//*!
 *
 * @name  PSTN_Get_Comm_Feature
 *
 * @brief  This function is called in response to GetCommFeature request
 *
 * @param cdc_obj_ptr :     Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/ 
extern uint8_t PSTN_Get_Comm_Feature(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                    USB_SETUP_STRUCT * setup_packet,
                                    uint8_t * *data, 
                                    uint32_t *size);
                                    
/**************************************************************************//*!
 *
 * @name  PSTN_Set_Comm_Feature
 *
 * @brief  This function is called in response to SetCommFeature request
 *
 * @param cdc_obj_ptr :     Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/ 
extern uint8_t PSTN_Set_Comm_Feature(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                    USB_SETUP_STRUCT * setup_packet,
                                    uint8_t * *data, 
                                    uint32_t *size);   
                                      
/**************************************************************************//*!
 *
 * @name  PSTN_Send_Serial_State
 *
 * @brief  This function is called to send serial state notification
 *
 * @param cdc_obj_ptr :   Pointer to CDC class object.
 *
 * @return NONE
 *****************************************************************************/ 
extern void PSTN_Send_Serial_State(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr) ;

/**************************************************************************//*!
 *
 * @name  PSTN_Response_Available
 *
 * @brief  This function is called to send notification to host that a 
 *         response is available
 *
 * @param cdc_obj_ptr   
 *
 * @return NONE
 *****************************************************************************/ 
extern void PSTN_Response_Available(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr);
#if USBCFG_DEV_RNDIS_SUPPORT
/**************************************************************************//*!
 *
 * @name  PSTN_Rndis_Message_Set
 *
 * @brief  This function is called in response to PSTN_Rndis_Message_Set 
 *
 * @param cdc_obj_ptr :     Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/ 
 extern uint8_t PSTN_Rndis_Message_Set(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                    USB_SETUP_STRUCT * setup_packet,
                                    uint8_t * *data, 
                                    uint32_t *size); 
/**************************************************************************//*!
 *
 * @name  PSTN_Rndis_Message_Get
 *
 * @brief  This function is called in response to PSTN_Rndis_Message_Get
 *
 * @param cdc_obj_ptr :     Pointer to CDC class object.
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned 
 *
 * @return status:       
 *                        USB_OK : Always
 *
 *****************************************************************************/                                          
extern uint8_t PSTN_Rndis_Message_Get(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
                                    USB_SETUP_STRUCT * setup_packet,
                                    uint8_t * *data, 
                                    uint32_t *size);    
                                    

extern void RNDIS_Initialize_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );
extern void RNDIS_Query_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );
extern void RNDIS_Set_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );
extern void RNDIS_Reset_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );
extern void RNDIS_Indicate_Status_Command 
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );      
extern void RNDIS_Keepalive_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,
        uint8_t * *data, 
        uint32_t *size
    );
extern void RNDIS_Halt_Command
    (
        CDC_DEVICE_STRUCT_PTR cdc_obj_ptr
    );                  
            
#endif/*endif USBCFG_DEV_RNDIS_SUPPORT*/

#endif

/* EOF */
