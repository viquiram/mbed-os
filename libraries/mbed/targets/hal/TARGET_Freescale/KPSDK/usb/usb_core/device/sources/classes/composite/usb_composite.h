/**HEADER********************************************************************
* 
* Copyright (c) 2004 -2010, 2013 Freescale Semiconductor;
* All Rights Reserved
*
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
* $FileName: usb_composite.h$
* $Version : 3.8.2.0$
* $Date    : Sep-19-2011$
*
* Comments:
*
* @brief The file contains USB stack Composite class layer api header function.
*
*****************************************************************************/

#ifndef _USB_COMPOSITE_H
#define _USB_COMPOSITE_H 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_class_composite.h"
#include "usb_class_hid.h"
#include "usb_class_audio.h"
#include "usb_class_msc.h"
#include "usb_class_cdc.h"
#include "usb_class_phdc.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/


/*****************************************************************************
 * Local Functions
 *****************************************************************************/


/* Strucutre holding HID class state information*/
typedef struct composite_device_struct
{
	_usb_device_handle            handle;
	uint32_t                      user_handle;
	USB_CLASS_HANDLE              class_handle;
	HID_HANDLE                    hid_handle;
	AUDIO_HANDLE                  audio_handle;
	CDC_HANDLE                    cdc_handle;
	MSD_HANDLE                    msc_handle;
	PHDC_HANDLE                   phdc_handle;	
	COMPOSITE_CONFIG_STRUCT_PTR   class_app_callback;
	USB_COMPOSITE_INFO_STRUCT_PTR class_composite_info;
	/* Number of class support */
	uint8_t                       cl_count;
}COMPOSITE_DEVICE_STRUCT, * COMPOSITE_DEVICE_STRUCT_PTR;

/******************************************************************************
 * Global Functions
 *****************************************************************************/

#endif

/* EOF */
