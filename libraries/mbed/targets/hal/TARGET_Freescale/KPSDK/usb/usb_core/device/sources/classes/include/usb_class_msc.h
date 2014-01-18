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
* $FileName: usb_class_msc.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief 
*
*****************************************************************************/

#ifndef _USB_CLASS_MSC_H
#define _USB_CLASS_MSC_H 1

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

#define USB_MSC_DEVICE_READ_REQUEST     (0x81)
#define USB_MSC_DEVICE_WRITE_REQUEST    (0x82) 
#define USB_MSC_DEVICE_FORMAT_COMPLETE  (0x83)
#define USB_MSC_DEVICE_REMOVAL_REQUEST  (0x84)
#define USB_MSC_DEVICE_GET_INFO         (0x85)
#define USB_MSC_START_STOP_EJECT_MEDIA  (0x86) 
/* macros for queuing */
 #define MSD_MAX_QUEUE_ELEMS  (4)

 #define USB_REQ_VAL_INVALID             (0xFFFF)
 
typedef uint32_t MSD_HANDLE;

/* structure to hold a request in the endpoint queue */
typedef struct _msc_app_data_struct
{
    uint8_t*                data_ptr;         /* pointer to buffer       */     
    uint32_t                data_size;        /* buffer size of endpoint */
}MSC_APP_DATA_STRUCT;
typedef struct _usb_class_msc_queue 
{
    _usb_device_handle      handle;
    uint8_t                 channel;       
    MSC_APP_DATA_STRUCT     app_data;  
}USB_CLASS_MSC_QUEUE, *PTR_USB_CLASS_MSC_QUEUE;

typedef struct _device_lba_info_struct
{
    uint32_t                total_lba_device_supports;/* lab : LOGICAL ADDRESS BLOCK */ 
    uint32_t                length_of_each_lab_of_device;
    uint8_t                 num_lun_supported; 
}DEVICE_LBA_INFO_STRUCT, * PTR_DEVICE_LBA_INFO_STRUCT;

typedef struct _msd_buffers_info
{
     uint8_t*               msc_lba_send_ptr;
     uint8_t*               msc_lba_recv_ptr;
     uint32_t               msc_lba_send_buff_size;
     uint32_t               msc_lba_recv_buff_size;
}MSD_BUFF_INFO, *PTR_MSD_BUFF_INFO;

/* USB class msc endpoint data */
typedef struct _usb_class_msc_endpoint 
{
    uint8_t                 endpoint; /* endpoint num */                    
    uint8_t                 type;     /* type of endpoint (interrupt, bulk or isochronous) */   
    uint8_t                 bin_consumer;/* the num of queued elements */
    uint8_t                 bin_producer;/* the num of de-queued elements */
    USB_CLASS_MSC_QUEUE     queue[MSD_MAX_QUEUE_ELEMS]; /* queue data */  
}USB_CLASS_MSC_ENDPOINT;

typedef struct _usb_class_msc_endpoint_data
{
	uint8_t                 count;
	USB_CLASS_MSC_ENDPOINT* ep;
}USB_CLASS_MSC_ENDPOINT_DATA,*PTR_USB_CLASS_MSC_ENDPOINT_DATA;

/* MSD Configuration structure to be passed by APP*/
typedef struct _usb_msd_config 
{
    /* SCSI related initialization data. To be moved to SCSI layer.*/
	 
     USB_APPLICATION_CALLBACK_STRUCT            msc_application_callback;
     USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback;
     USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
     DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr; 
}USB_MSD_CONFIG_STRUCT, * USB_MSD_CONFIG_STRUCT_PTR;

typedef struct _lba_app_struct
{
    uint32_t             offset;
    uint32_t             size;
    uint8_t*             buff_ptr;
}LBA_APP_STRUCT, * PTR_LBA_APP_STRUCT;

#define USB_MSC_Periodic_Task USB_Class_Periodic_Task 
/******************************************************************************
 * Global Functions
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Class_MSC_Init
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param msd_config_ptr    : Configuration paramemter strucutre pointer
 *                            passed by APP.
 * @return status       
 *         MSD Handle           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 *
 *This function initializes the MSC Class layer and layers it is dependednt on 
 ******************************************************************************/
extern USB_STATUS USB_Class_MSC_Init
(
		uint8_t controller_id,
		USB_MSD_CONFIG_STRUCT_PTR msd_config_ptr,
		MSD_HANDLE *  msd_handle
); 


/**************************************************************************//*!
 *
 * @name  USB_Class_MSC_Deinit
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param cdc_handle
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 *
 *This function initializes the MSC Class layer and layers it is dependednt on 
 *
 *****************************************************************************/
extern USB_STATUS USB_Class_MSC_Deinit
(
  MSD_HANDLE msd_handle
);

#endif


