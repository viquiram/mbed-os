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
* $FileName: usb_audio_class.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack AUDIO class layer api header function.
*
*****************************************************************************/

#ifndef _USB_AUDIO_H
#define _USB_AUDIO_H 1

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
#define AUDIO_MAX_QUEUE_ELEMS  (4)

/* Code of bmRequest Type */
#define SET_REQUEST_ITF                   (0x21)
#define SET_REQUEST_EP                    (0x22)
#define GET_REQUEST_EP                    (0xA2)
#define GET_REQUEST_ITF                   (0xA1) 
 
 /* class specific requests */
#define AUDIO_CONTROL_INPUT_TERMINAL      (0x02)
#define AUDIO_CONTROL_OUTPUT_TERMINAL     (0x03)
#define AUDIO_CONTROL_FEATURE_UNIT        (0x06)

/* Audio Class Specific Request Codes */
#define REQUEST_CODE_UNDEFINED		      (0x00)
#define SET_CUR						      (0x01)
#define SET_MIN						      (0x02)
#define SET_MAX						      (0x03)
#define SET_RES						      (0x04)


#define GET_CUR						      (0x81)
#define GET_MIN						      (0x82)
#define GET_MAX						      (0x83)
#define GET_RES						      (0x84)

#define GET_STAT					      (0xFF)


/* Terminal control selector codes */
#define TE_CONTROL_UNDEFINED		      (0x00)
#define COPY_PROTECT_CONTROL		      (0x01)

/* feature unit control selector codes */
#define FU_CONTROL_UNDEFINED		      (0x00)
/* Feature Unit Control Selector codes */
#define MUTE_CONTROL                      (0x01)
#define VOLUME_CONTROL                    (0x02)
#define BASS_CONTROL                      (0x03)
#define MID_CONTROL                       (0x04)
#define TREBLE_CONTROL                    (0x05)
#define GRAPHIC_EQUALIZER_CONTROL         (0x06)
#define AUTOMATIC_GAIN_CONTROL            (0x07)
#define DELAY_CONTROL                     (0x08)
#define BASS_BOOST_CONTROL                (0x09)
#define LOUDNESS_CONTROL                  (0x0A)

/* Endpoint control selector codes */
#define EP_CONTROL_DEFINED			      (0x00)
#define SAMPLING_FREQ_CONTROL		      (0x01)
#define PITCH_CONTROL		   		      (0x02)

#define MAX_AUDIO_DEVICE                  (0x01)
  
typedef struct _usb_class_audio_queue 
{
    _usb_device_handle        handle;
    uint8_t                   channel;       
	AUDIO_APP_DATA_STRUCT     app_data;
}USB_CLASS_AUDIO_QUEUE, *PTR_USB_CLASS_AUDIO_QUEUE;
 
/* USB class audio endpoint data */
  
typedef struct _usb_class_audio_endpoint 
{
    uint8_t                   endpoint; /* endpoint num */                    
    uint8_t                   type;     /* type of endpoint (interrupt, bulk or isochronous) */   
    uint8_t                   bin_consumer;/* the num of queued elements */
    uint8_t                   bin_producer;/* the num of de-queued elements */
    uint8_t                   queue_num;	/* the num of queue */
    USB_CLASS_AUDIO_QUEUE     queue[AUDIO_MAX_QUEUE_ELEMS]; /* queue data */  
}USB_CLASS_AUDIO_ENDPOINT;

/* contains the endpoint data for non control endpoints */
typedef struct _usb_class_audio_endpoint_data 
{    
    uint8_t                   count;  /* num of non control endpoints */     
    USB_CLASS_AUDIO_ENDPOINT* ep;       
}USB_CLASS_AUDIO_ENDPOINT_DATA, *PTR_USB_CLASS_AUDIO_ENDPOINT_DATA;
 
/*****************************************************************************
 * Local Functions
 *****************************************************************************/
void USB_Class_Audio_Event(uint8_t event, void* val,void* arg);
uint8_t USB_Audio_Requests(USB_SETUP_STRUCT * setup_packet, uint8_t * *data, uint32_t *size, void* arg);


/******************************************************************************
 Types
****************************************************************************/
/* Strucutre holding AUDIO class state information*/
typedef struct audio_device_struct
{
    AUDIO_HANDLE                               audio_handle;
    USB_CLASS_HANDLE                           class_handle;
    uint32_t                                   user_handle;
    _usb_device_handle                         handle;
    USB_ENDPOINTS*                             usb_ep_data;
    USB_CLASS_AUDIO_UNITS*                     usb_ut_data;
    USB_CLASS_AUDIO_ENDPOINT*                  ep;
    USB_APPLICATION_CALLBACK_STRUCT            audio_application_callback;
    USB_VENDOR_REQ_CALLBACK_STRUCT		       vendor_req_callback;
    USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
    DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr;
    uint8_t                                    current_interface;
}AUDIO_DEVICE_STRUCT, * AUDIO_DEVICE_STRUCT_PTR; 
#endif

/* EOF */
