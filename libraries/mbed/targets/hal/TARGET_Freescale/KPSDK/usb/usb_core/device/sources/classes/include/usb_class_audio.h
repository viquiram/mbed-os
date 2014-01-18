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
* $FileName: usb_audio.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack AUDIO class layer api header function.
*
*****************************************************************************/

#ifndef _USB_CLASS_AUDIO_H
#define _USB_CLASS_AUDIO_H 1


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

/* command */
/* GET CUR COMMAND */
#define GET_CUR_MUTE_CONTROL						(0x8101)
#define GET_CUR_VOLUME_CONTROL						(0x8102)
#define GET_CUR_BASS_CONTROL						(0x8103)
#define GET_CUR_MID_CONTROL							(0x8104)
#define GET_CUR_TREBLE_CONTROL						(0x8105)
#define GET_CUR_GRAPHIC_EQUALIZER_CONTROL			(0x8106)
#define GET_CUR_AUTOMATIC_GAIN_CONTROL				(0x8107)
#define GET_CUR_DELAY_CONTROL						(0x8108)
#define GET_CUR_BASS_BOOST_CONTROL					(0x8109)
#define GET_CUR_LOUDNESS_CONTROL					(0x810A)

/* GET MIN COMMAND */
#define GET_MIN_VOLUME_CONTROL						(0x8202)
#define GET_MIN_BASS_CONTROL						(0x8203)
#define GET_MIN_MID_CONTROL							(0x8204)
#define GET_MIN_TREBLE_CONTROL						(0x8205)
#define GET_MIN_GRAPHIC_EQUALIZER_CONTROL			(0x8206)
#define GET_MIN_DELAY_CONTROL						(0x8208)

/* GET MAX COMMAND */
#define GET_MAX_VOLUME_CONTROL						(0x8302)
#define GET_MAX_BASS_CONTROL						(0x8303)
#define GET_MAX_MID_CONTROL							(0x8304)
#define GET_MAX_TREBLE_CONTROL						(0x8305)
#define GET_MAX_GRAPHIC_EQUALIZER_CONTROL			(0x8306)
#define GET_MAX_DELAY_CONTROL						(0x8308)

/* GET RES COMMAND */
#define GET_RES_VOLUME_CONTROL						(0x8402)
#define GET_RES_BASS_CONTROL						(0x8403)
#define GET_RES_MID_CONTROL							(0x8404)
#define GET_RES_TREBLE_CONTROL						(0x8405)
#define GET_RES_GRAPHIC_EQUALIZER_CONTROL			(0x8406)
#define GET_RES_DELAY_CONTROL						(0x8408)

/* SET CUR COMMAND */
#define SET_CUR_MUTE_CONTROL						(0x0101)
#define SET_CUR_VOLUME_CONTROL						(0x0102)
#define SET_CUR_BASS_CONTROL						(0x0103)
#define SET_CUR_MID_CONTROL							(0x0104)
#define SET_CUR_TREBLE_CONTROL						(0x0105)
#define SET_CUR_GRAPHIC_EQUALIZER_CONTROL			(0x0106)
#define SET_CUR_AUTOMATIC_GAIN_CONTROL				(0x0107)
#define SET_CUR_DELAY_CONTROL						(0x0108)
#define SET_CUR_BASS_BOOST_CONTROL					(0x0109)
#define SET_CUR_LOUDNESS_CONTROL					(0x010A)

/* SET MIN COMMAND */
#define SET_MIN_VOLUME_CONTROL						(0x0202)
#define SET_MIN_BASS_CONTROL						(0x0203)
#define SET_MIN_MID_CONTROL							(0x0204)
#define SET_MIN_TREBLE_CONTROL						(0x0205)
#define SET_MIN_GRAPHIC_EQUALIZER_CONTROL			(0x0206)
#define SET_MIN_DELAY_CONTROL						(0x0208)

/* SET MAX COMMAND */
#define SET_MAX_VOLUME_CONTROL						(0x0302)
#define SET_MAX_BASS_CONTROL						(0x0303)
#define SET_MAX_MID_CONTROL							(0x0304)
#define SET_MAX_TREBLE_CONTROL						(0x0305)
#define SET_MAX_GRAPHIC_EQUALIZER_CONTROL			(0x0306)
#define SET_MAX_DELAY_CONTROL						(0x0308)

/* SET RES COMMAND */
#define SET_RES_VOLUME_CONTROL						(0x0402)
#define SET_RES_BASS_CONTROL						(0x0403)
#define SET_RES_MID_CONTROL							(0x0404)
#define SET_RES_TREBLE_CONTROL						(0x0405)
#define SET_RES_GRAPHIC_EQUALIZER_CONTROL			(0x0406)
#define SET_RES_DELAY_CONTROL						(0x0408)

#define GET_CUR_COPY_PROTECT_CONTROL				(0x810B)

#define GET_CUR_SAMPLING_FREQ_CONTROL				(0x810C)
#define GET_MIN_SAMPLING_FREQ_CONTROL				(0x820C)
#define GET_MAX_SAMPLING_FREQ_CONTROL				(0x830C)
#define GET_RES_SAMPLING_FREQ_CONTROL				(0x840C)

#define GET_CUR_PITCH_CONTROL						(0x810D)
#define GET_MIN_PITCH_CONTROL						(0x820D)
#define GET_MAX_PITCH_CONTROL						(0x830D)
#define GET_RES_PITCH_CONTROL						(0x840D)

#define SET_CUR_COPY_PROTECT_CONTROL				(0x010B)

#define SET_CUR_SAMPLING_FREQ_CONTROL				(0x010C)
#define SET_MIN_SAMPLING_FREQ_CONTROL				(0x020C)
#define SET_MAX_SAMPLING_FREQ_CONTROL				(0x030C)
#define SET_RES_SAMPLING_FREQ_CONTROL				(0x040C)

#define SET_CUR_PITCH_CONTROL						(0x010D)

#define SET_MEM						      (0x05)
#define GET_MEM						      (0x85)

#define USB_REQ_VAL_INVALID				 (0xFFFF)


 /* structure to hold a request in the endpoint queue */
typedef struct _audio_app_data_struct
{
    uint8_t*      data_ptr;    /* pointer to buffer       */     
    uint32_t      data_size;   /* buffer size of endpoint */
}AUDIO_APP_DATA_STRUCT;

typedef struct _USB_AUDIO_UT_STRUCT
{
  uint8_t         unit_id;     /* endpoint number         */
  uint8_t         type;        /* type of endpoint        */
}USB_UT_STRUCT, *USB_UT_STRUCT_PTR;

typedef  struct _USB_CLASS_AUDIO_UNITS 
{
   uint8_t        count;       /* Number of terminal or Ferture Unit End point */  
   USB_UT_STRUCT* epp;         /* Array of terminal or Feature Unit */
}USB_CLASS_AUDIO_UNITS; 

/*****************************************************************************
 * Local Functions
 *****************************************************************************/

 /******************************************************************************
 * Types
 *****************************************************************************/
 typedef uint32_t AUDIO_HANDLE;

 /* Structure used to configure Audio class by APP*/
 typedef struct audio_config_struct
 {
    USB_APPLICATION_CALLBACK_STRUCT                audio_application_callback;
	USB_VENDOR_REQ_CALLBACK_STRUCT 	               vendor_req_callback;
	USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT     class_specific_callback;
	DESC_REQUEST_NOFIFY_STRUCT_PTR	               desc_callback_ptr; 
 }AUDIO_CONFIG_STRUCT,* AUDIO_CONFIG_STRUCT_PTR;
/******************************************************************************
 * Global Functions
 *****************************************************************************/
/**************************************************************************//*!
 *
 * @name  USB_Class_Audio_Init
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
 *This function initializes the Audio Class layer and layers it is dependent on 
 *
 *****************************************************************************/
extern USB_STATUS USB_Class_Audio_Init
(
	uint8_t                 controller_id,
	AUDIO_CONFIG_STRUCT_PTR audio_config_ptr,
	AUDIO_HANDLE*           audioHandle
);

/**************************************************************************//*!
 *
 * @name  USB_Class_Audio_Deinit
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
extern USB_STATUS USB_Class_Audio_Deinit
(
	AUDIO_HANDLE handle
);

/**************************************************************************//*!
 *
 * @name  USB_Class_Audio_Send_Data
 *
 * @brief This function is used to send data to the host
 *
 * @param handle          :   handle returned by USB_Class_Audio_Send_Data
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
extern uint8_t USB_Class_Audio_Send_Data
(
    AUDIO_HANDLE         handle,        /* [IN]*/
    uint8_t              ep_num,        /* [IN]*/
    uint8_t*             buff_ptr,      /* [IN] buffer to send */      
    uint32_t             size           /* [IN] length of the transfer */
);

/**************************************************************************//*!
 *
 * @name  USB_Class_Audio_Recv_Data
 *
 * @brief This function receives Data from Host.
 *
 * @param handle          :   handle returned by USB_Class_Audio_Recv_Data
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
extern uint8_t USB_Class_Audio_Recv_Data
(
    AUDIO_HANDLE         audio_handle,
    uint8_t              ep_num,
    uint8_t*             buff_ptr,      /* [IN] buffer to recv */      
    uint32_t             size           /* [IN] length of the transfer */
);

#endif

/* EOF */
