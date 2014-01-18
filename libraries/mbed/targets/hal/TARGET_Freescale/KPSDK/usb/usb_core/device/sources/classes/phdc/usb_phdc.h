/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989 - 2008 ARC International;
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
* $FileName: usb_phdc.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack PHDC class layer api header function.
*
*****************************************************************************/
#ifndef _USB_PHDC_H
#define _USB_PHDC_H 1

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
#define MAX_QOS_BIN_ELEMS                 (4) 
 /* the num of recieve endpoints */ 
#define PHDC_RX_ENDPOINTS                 (1)  
/* the num of transmit endpoints */ 
#define PHDC_TX_ENDPOINTS                 (2)   
#define SET_FEATURE_REQUEST               (3)
#define CLEAR_FEATURE_REQUEST             (1)
#define GET_STATUS_REQUEST                (0)
#define INVALID_VAL                       (0xFF)
#define USB_SET_REQUEST_MASK              (0x02)
#define MAX_PHDC_DEVICE                   (0x01)

/******************************************************************************
 * Types
 *****************************************************************************/
 
/* structure to hold a request in the endpoint QOS bin */

PACKED_STRUCT_BEGIN
struct _usb_class_phdc_qos_bin 
{
    uint8_t*                    app_buff;   /* buffer to send */
    uint32_t                    size;       /* size of the transfer */
    bool                        meta_data;  /* packet is a meta data or not */
    uint8_t                     channel;    /* endpoint num */
    uint8_t                     num_tfr;    /* num of transfers that follow the meta data packet.
                                                                           used only when meta_data is TRUE */                               
    uint8_t                     qos;        /* qos of the transfers that follow the meta data packet */
} 
PACKED_STRUCT_END;

typedef struct _usb_class_phdc_qos_bin USB_CLASS_PHDC_QOS_BIN, *PTR_USB_CLASS_PHDC_QOS_BIN;

/* USB class phdc endpoint data */
typedef struct _usb_class_phdc_tx_endpoint 
{
    uint32_t                    size;               /* from the application */
    USB_CLASS_PHDC_QOS_BIN      qos_bin[MAX_QOS_BIN_ELEMS]; 
    uint8_t                     endpoint;           /* from the application */
    uint8_t                     type;               /* from the application */ 
    uint8_t                     qos;                /* from the application */
    uint8_t                     current_qos;        /* from received meta data */
    uint8_t                     transfers_left;     /* from application meta data */
    uint8_t                     bin_consumer;       /* num of dequeued transfers */
    uint8_t                     bin_producer;       /* num of queued transfers */
}USB_CLASS_PHDC_TX_ENDPOINT;

typedef struct _usb_class_phdc_rx_endpoint 
{
    uint8_t*                    buff_ptr;
    uint32_t                    size;               /* from the application */
    uint16_t                    buffer_size;
    uint8_t                     endpoint;           /* from the application */
    uint8_t                     type;               /* from the application */
    uint8_t                     qos;                /* from the application */
    uint8_t                     current_qos;        /* from received meta data */    
    uint8_t                     transfers_left;     /* from received meta data */
}USB_CLASS_PHDC_RX_ENDPOINT;
  
typedef struct _usb_class_phdc_endpoint_data 
{
    _usb_device_handle          handle;
    USB_CLASS_PHDC_RX_ENDPOINT* ep_rx;  
    USB_CLASS_PHDC_TX_ENDPOINT* ep_tx;
    uint8_t                     count_rx;
    uint8_t                     count_tx;     
}USB_CLASS_PHDC_ENDPOINT_DATA, *PTR_USB_CLASS_PHDC_ENDPOINT_DATA; 

#if USB_METADATA_SUPPORTED
#define METADATA_PREAMBLE_SIGNATURE     (16)
#define METADATA_QOSENCODING_VERSION    (1)
#define METADATA_HEADER_SIZE            (21)
/* structure for meta_data msg preamble */
typedef struct _usb_meta_data_msg_preamble 
{
    char                        signature[METADATA_PREAMBLE_SIGNATURE];
    uint8_t                     num_tfr;
    uint8_t                     version;
    uint8_t                     qos;
    uint8_t                     opaque_data_size;
    uint8_t                     opaque_data[1];
}USB_META_DATA_MSG_PREAMBLE;

typedef struct _usb_app_event_metadata_params 
{
    uint8_t*                    metadata_ptr;
    uint32_t                    size;
    uint8_t                     channel;
    uint8_t                     num_tfr;
    uint8_t                     qos;
}USB_APP_EVENT_METADATA_PARAMS, *PTR_USB_APP_EVENT_METADATA_PARAMS;
#endif

/* Structure Representing PHDC class */
typedef struct _phdc_struct
{
    _usb_device_handle                         handle;
    uint32_t                                   user_handle;
    USB_CLASS_HANDLE                           class_handle;
    USB_ENDPOINTS*                             ep_desc_data;
    USB_APPLICATION_CALLBACK_STRUCT            phdc_application_callback;
    USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback; 
    USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
    DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr;
    /* RAM buffer for configuring next receive */
    uint8_t*                                   service_buff_ptr; 
    USB_CLASS_PHDC_ENDPOINT_DATA               phdc_endpoint_data;
#if META_DATA_MSG_PRE_IMPLEMENTED
    USB_META_DATA_MSG_PREAMBLE                 meta_data_msg_preamble; 
#endif
#if USB_METADATA_SUPPORTED
    /* used to store whether meta-data feature is active or not */
    bool                                       phdc_metadata;
#endif 
    /* used to store a bit map of the active endpoints */
    uint16_t                                   phdc_ep_has_data;
}PHDC_DEVICE_STRUCT, * PHDC_DEVICE_STRUCT_PTR;

/******************************************************************************
 * Global Functions
 *****************************************************************************/
void USB_Class_PHDC_Event( uint8_t event, void* val, void* arg);
uint8_t USB_PHDC_Requests ( USB_SETUP_STRUCT * setup_packet, uint8_t **data, uint32_t *size, void* arg); 
#endif
