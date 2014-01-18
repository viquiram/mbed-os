/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010, 2013 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 ******************************************************************************
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
 **************************************************************************//*!
 *
 * @file usb_composite.h
 *
 * @author
 *
 * @version
 *
 * @date
 *
 * @brief The file contains USB stack Video class layer API header function.
 *
 *****************************************************************************/


#ifndef _USB_CLASS_COMPOSITE_H
#define _USB_CLASS_COMPOSITE_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "usb_class.h"


/******************************************************************************
 * Macro's
 *****************************************************************************/
/* Audio class type */ 


 typedef uint32_t COMPOSITE_HANDLE;
/******************************************************************************
 * Types
 *****************************************************************************/

 /* Structure used to configure composite class by APP*/
 typedef struct composite_config_struct
 {
     USB_APPLICATION_CALLBACK_STRUCT            composite_application_callback;
     USB_VENDOR_REQ_CALLBACK_STRUCT             vendor_req_callback;
     USB_CLASS_SPECIFIC_HANDLER_CALLBACK_STRUCT class_specific_callback;
     DESC_REQUEST_NOFIFY_STRUCT_PTR             desc_callback_ptr;
	 class_type                                 type;
 }COMPOSITE_CONFIG_STRUCT,* COMPOSITE_CONFIG_STRUCT_PTR;
 
 typedef struct _COMPOSITE_CALLBACK_STRUCT
 {	 
	 uint8_t                                    count;               /* Number of class support */	 
	 COMPOSITE_CONFIG_STRUCT_PTR                class_app_callback;  /* Array of Endpoints Structures */
 }COMPOSITE_CALLBACK_STRUCT;
/******************************************************************************
 * Global function prototypes
 *****************************************************************************/
extern uint8_t USB_Composite_Init(
    uint8_t                    controller_ID,                /* [IN] Controller ID */
    COMPOSITE_CALLBACK_STRUCT* composite_callback_ptr,       /* [IN] Poiter to class info */
    COMPOSITE_HANDLE*          compositeHandle   
);

extern uint8_t USB_Composite_DeInit(
    COMPOSITE_HANDLE           compositeHandle               /* [IN] Controller ID */
);
extern uint8_t USB_Composite_Get_Class_Handle(COMPOSITE_HANDLE handle, class_type type, void *	class_handle_ptr);
#endif
/* EOF */
