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
* $FileName: usb_dev.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*  This file contains the main USB device API functions that will be 
*  used by most applications.
*                                                               
*END*********************************************************************/
#include "usb_device_config.h"
#if USBCFG_DEV_KHCI || USBCFG_DEV_EHCI
#include "usb.h"
#include "usb_device_stack_interface.h"
#include "usb_dev.h"


#define USB_DEV_HANDLE_OCCUPIED ((uint8_t)1)
#define USB_DEV_HANDLE_FREE     ((uint8_t)0)
extern int32_t bsp_usb_dev_init(uint8_t controller_id);
extern void USB_Control_Service (void* handle, PTR_USB_EVENT_STRUCT event,void* arg);
extern void USB_Reset_Service(void* handle, PTR_USB_EVENT_STRUCT event, void* arg);
extern void USB_Error_Service(void* handle, PTR_USB_EVENT_STRUCT event, void* arg);
extern void USB_Suspend_Service(void* handle, PTR_USB_EVENT_STRUCT event,void* arg);
extern void USB_Resume_Service(void* handle,PTR_USB_EVENT_STRUCT event,void* arg );

USB_DEV_STATE_STRUCT g_usb_dev[USBCFG_DEV_NUM] = {0};


extern const USB_DEV_INTERFACE_FUNCTIONS_STRUCT _usb_khci_dev_function_table;

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_device_get_handle
*  Returned Value : NULL
*  Comments       :
*        This function is used to get one unused device object
*
*END*-----------------------------------------------------------------*/
USB_DEV_STATE_STRUCT* _usb_device_get_handle(void)
{
    uint8_t i = 0;
    for (; i < USBCFG_DEV_NUM; i++) {
        if (g_usb_dev[i].occupied != USB_DEV_HANDLE_OCCUPIED) {
            OS_Mem_zero(&g_usb_dev[i], sizeof(USB_DEV_STATE_STRUCT));
            g_usb_dev[i].occupied = USB_DEV_HANDLE_OCCUPIED;
            return &g_usb_dev[i];
        }
    }
    return NULL;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_device_release_handle
*  Returned Value : NULL
*  Comments       :
*        This function is used to set one used device object to free
*
*END*-----------------------------------------------------------------*/
void _usb_device_release_handle(USB_DEV_STATE_STRUCT *usb_dev)
{
    usb_dev->occupied = USB_DEV_HANDLE_FREE;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_device_get_DCI
*  Returned Value : NULL
*  Comments       :
*        This function is used to get the device controller's interface table pointer
*
*END*-----------------------------------------------------------------*/
void _usb_device_get_DCI(uint8_t controller_id, USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR* controller_if_ptr)
{
    if(controller_id == 0)
        *controller_if_ptr = (USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)&_usb_khci_dev_function_table;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_device_shutdown
*  Returned Value : USB_OK or error code
*  Comments       :
*        Shutdown an initialized USB device
*
*END*-----------------------------------------------------------------*/
USB_STATUS _usb_device_shutdown
(
    /* [IN] the USB_USB_dev_initialize state structure */
    _usb_device_handle         handle
)
{ 
    USB_STATUS                        error;
    volatile USB_DEV_STATE_STRUCT_PTR usb_dev_ptr;
    void*                           temp;
   
    if (handle == NULL)
    {
        #if _DEBUG
            printf("_usb_device_shutdowna: handle is NULL\n");
        #endif  
        return USBERR_ERROR;
    }
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;

    if ( ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
        usb_dev_ptr->usb_dev_interface)->DEV_SHUTDOWN != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
            usb_dev_ptr->usb_dev_interface)->DEV_SHUTDOWN(usb_dev_ptr->controller_handle);
    }
    else
    {
        #if _DEBUG
            printf("_usb_device_shutdown: DEV_SHUTDOWN is NULL\n");
        #endif  
        return USBERR_ERROR;
    }
       
    /* Free all the Callback function structure memory */
    temp = usb_dev_ptr->service_head_ptr;
    while(temp != NULL)
    {
        OS_Mem_free(temp);
        usb_dev_ptr->service_head_ptr = usb_dev_ptr->service_head_ptr->next;
        temp = usb_dev_ptr->service_head_ptr;
    }

     _usb_device_release_handle(usb_dev_ptr);
    
    return  error;   
} /* EndBody */

/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : usb_device_call_service
* Returned Value : USB_OK or error code
* Comments       :
*     Calls the appropriate service for the specified type, if one is
*     registered. Used internally only.
* 
*END*--------------------------------------------------------------------*/
 USB_STATUS _usb_device_call_service
   (
       /* [IN] Type of service or endpoint */    
      uint8_t                  type,
      
      /* [IN] pointer to event structure  */ 
      PTR_USB_EVENT_STRUCT    event            
   )
{
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    SERVICE_STRUCT *          service_ptr = NULL;

    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)event->handle;
    
    /* Needs mutual exclusion */
    OS_Lock();

    switch (type)
    {
        case USB_SERVICE_EP0:
            USB_Control_Service(&usb_dev_ptr->usb_framework, event, service_ptr->arg);
            break;     
        case USB_SERVICE_BUS_RESET:
            USB_Reset_Service(&usb_dev_ptr->usb_framework, event, service_ptr->arg);
            break;
        case USB_SERVICE_SUSPEND:
            USB_Suspend_Service(&usb_dev_ptr->usb_framework, event, service_ptr->arg);
            break;
        case USB_SERVICE_RESUME:
            USB_Resume_Service(&usb_dev_ptr->usb_framework, event, service_ptr->arg);
            break;
        case USB_SERVICE_ERROR:
            USB_Error_Service(&usb_dev_ptr->usb_framework, event, service_ptr->arg);
            break;
        default:
            break;
    } /* Endswitch */

    /* Search for an existing entry for type */
    for (service_ptr = usb_dev_ptr->service_head_ptr;
         service_ptr;
         service_ptr = service_ptr->next) 
    {
        if (service_ptr->type == type) 
        {   
            service_ptr->service(event,service_ptr->arg);
            OS_Unlock();
            return USB_OK;
        }  
    }

    OS_Unlock();

    return USBERR_CLOSED_SERVICE;
} /* EndBody */


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_set_address
*  Returned Value : USB_OK or error code
*  Comments       :
*        Sets the device address as assigned by the host during enumeration
*
*END*-----------------------------------------------------------------*/
USB_STATUS _usb_device_set_address
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
      
      /* [IN] the USB address to be set in the hardware */
      uint8_t                     address
   )
{ 
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_STATUS                    error;
   
    if (handle == NULL)
    {
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;

    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
      usb_dev_ptr->usb_dev_interface)->DEV_SET_ADDRESS != NULL) 
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
            usb_dev_ptr->usb_dev_interface)->DEV_SET_ADDRESS(usb_dev_ptr->controller_handle, address);
    }
    else
    {
        #ifdef _DEV_DEBUG
            printf("usb_device_set_address: DEV_SET_ADDRESS is NULL\n");                      
        #endif  
        return USBERR_ERROR;
    }

    return error;
}

/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : usb_device_get_status
* Returned Value : USB_OK or error code
* Comments       :
*     Provides API to access the USB internal state.
* 
*END*--------------------------------------------------------------------*/
USB_STATUS _usb_device_get_status
   (
      /* [IN] Handle to the USB device */
      _usb_device_handle   handle,
      
      /* [IN] What to get the error of */
      uint8_t               component,
      
      /* [OUT] The requested error */
      uint16_t*          error
   )
{ /* Body */
    volatile USB_DEV_STATE_STRUCT_PTR usb_dev_ptr;

    if((handle == NULL)||(error == NULL))
    {
        #if _DEBUG
            printf("usb_device_get_status: NULL pointer\n");
        #endif  
        return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
   
    OS_Lock();
    
    if (component & USB_STATUS_ENDPOINT) 
    {
        if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
           usb_dev_ptr->usb_dev_interface)->DEV_GET_ENDPOINT_STATUS != NULL)
        {
            ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
                usb_dev_ptr->usb_dev_interface)->DEV_GET_ENDPOINT_STATUS(usb_dev_ptr->controller_handle,
                (uint8_t)(component),error);
        }
        else
        {
            #if _DEBUG
                printf("usb_device_get_status: DEV_GET_ENDPOINT_STATUS is NULL\n");
            #endif  
            OS_Unlock();
            return USBERR_ERROR;
        }             
    } 
    else 
    {
        if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
           usb_dev_ptr->usb_dev_interface)->DEV_GET_DEVICE_STATUS != NULL)
        {
            ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
                usb_dev_ptr->usb_dev_interface)->DEV_GET_DEVICE_STATUS(usb_dev_ptr->controller_handle,
                (uint8_t)(component),error);
        }
        else
        {
            #if _DEBUG
                printf("usb_device_get_status: DEV_GET_DEVICE_STATUS is NULL\n");
            #endif  
            OS_Unlock();
            return USBERR_ERROR;
        }        
    } 

   OS_Unlock();
   return USB_OK;   
}   
 
/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : usb_device_set_status
* Returned Value : USB_OK or error code
* Comments       :
*     Provides API to set internal state
* 
*END*--------------------------------------------------------------------*/
USB_STATUS _usb_device_set_status
   (
      /* [IN] Handle to the usb device */
      _usb_device_handle   handle,
      
      /* [IN] What to set the error of */
      uint8_t               component,
      
      /* [IN] What to set the error to */
      uint16_t              setting
   )
{
    volatile USB_DEV_STATE_STRUCT_PTR usb_dev_ptr;
    uint8_t error = USB_OK;
      
    if (handle == NULL)
    {
        #if _DEBUG
            printf("usb_device_set_status: handle is NULL\n");
        #endif  
        return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    OS_Lock();
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
       usb_dev_ptr->usb_dev_interface)->DEV_SET_DEVICE_STATUS != NULL)
    {
        ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
            usb_dev_ptr->usb_dev_interface)->DEV_SET_DEVICE_STATUS(usb_dev_ptr->controller_handle,
            (uint8_t)(component),setting);
    }
    else
    {
        #if _DEBUG
            printf("usb_device_set_status: DEV_SET_DEVICE_STATUS is NULL\n");
        #endif  
        OS_Unlock();
        return USBERR_ERROR;
    }        
    
    OS_Unlock();
    return error;   
} /* EndBody */


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_init
*  Returned Value : USB_OK or error code
*  Comments       :
*        Initializes the USB device specific data structures and calls 
*  the low-level device controller chip initialization routine.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_init
   (
      /* [IN] the USB device controller to initialize */
      uint8_t controller_id,

      /* [OUT] the USB_USB_dev_initialize state structure */
      _usb_device_handle *  handle
   )
{
    USB_DEV_STATE_STRUCT_PTR         usb_dev_ptr;
    //uint32_t                          temp, i, j;
    //SCRATCH_STRUCT_PTR               temp_scratch_ptr;
    //USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR call_back_table_ptr;
    USB_STATUS                       error;
    USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR dev_if = NULL;
    USB_CLASS_FW_OBJECT_STRUCT_PTR usb_fw_ptr = NULL;

    OS_Lock();
 
    usb_dev_ptr = _usb_device_get_handle();
 
    if(usb_dev_ptr == NULL){
          /* The interface does not support device functionality */
        OS_Unlock();
        return USBERR_DEVICE_BUSY;
    }
    usb_fw_ptr = &usb_dev_ptr->usb_framework;
    usb_fw_ptr->ext_req_to_host = (uint8_t*)OS_Mem_alloc_zero(MAX_EXPECTED_CONTROL_OUT_SIZE);
    if (NULL == usb_fw_ptr->ext_req_to_host)
    {
        #ifdef _DEV_DEBUG
            printf("ext_req_to_host malloc failed: usb_device_init\n");
        #endif
        OS_Unlock();
        return USBERR_ALLOC;
    }
   
    _usb_device_get_DCI(controller_id, &dev_if);
 
    if(dev_if == NULL)
    {
         _usb_device_release_handle(usb_dev_ptr);
         OS_Unlock();
         return USBERR_DEVICE_NOT_FOUND;
    }
 
    usb_dev_ptr->usb_dev_interface = (void*)dev_if;
    OS_Unlock();
       
    /* Initialize the USB interface. */
    if (dev_if->DEV_PREINIT != NULL)
    {
        error = dev_if->DEV_PREINIT(usb_dev_ptr, (_usb_device_handle *) (&usb_dev_ptr->controller_handle));
    }

    if (usb_dev_ptr->controller_handle == NULL)
    {
        #ifdef _DEV_DEBUG
        printf("1 memalloc failed in usb_device_init\n");
        #endif  
        return USBERR_ALLOC_STATE;
    } /* Endif */
    usb_fw_ptr->controller_handle = usb_dev_ptr->controller_handle;
    usb_fw_ptr->dev_handle = usb_dev_ptr;
    error = bsp_usb_dev_init(controller_id);

    if (error != USB_OK)
    {     
        if (dev_if->DEV_SHUTDOWN != NULL)
        {
            dev_if->DEV_SHUTDOWN(usb_dev_ptr->controller_handle);
        }
        return USBERR_UNKNOWN_ERROR;
    }

    /* Initialize the USB controller chip */
    if (dev_if->DEV_INIT != NULL) 
    {
        error = dev_if->DEV_INIT(controller_id,usb_dev_ptr->controller_handle);     
    }
    else
    {
        #ifdef _DEV_DEBUG
            printf("usb_device_init: DEV_INIT is NULL\n");                   
        #endif  
        return USBERR_ERROR;
    }

    if (error) 
    {
        if (dev_if->DEV_SHUTDOWN != NULL)
        {
            dev_if->DEV_SHUTDOWN(usb_dev_ptr->controller_handle);
        }
        return USBERR_INIT_FAILED;
    } /* Endif */
    
    *handle = usb_dev_ptr;
    return error;
} /* EndBody */


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_deinit
*  Returned Value : USB_OK or error code
*  Comments       :
*  uninitializes the USB device specific data structures and calls 
*  the low-level device controller chip initialization routine.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_deinit
   (
      /* [OUT] the USB_USB_dev_initialize state structure */
      _usb_device_handle  handle
   )
{
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_CLASS_FW_OBJECT_STRUCT_PTR usb_fw_ptr = NULL;
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    usb_fw_ptr = &usb_dev_ptr->usb_framework;
    
    if(NULL != usb_fw_ptr->ext_req_to_host)
    {
        OS_Mem_free(usb_fw_ptr->ext_req_to_host);
    }
    
	_usb_device_shutdown(handle);
    
    _usb_device_release_handle(usb_dev_ptr);
		
	return USB_OK;

} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_init_endpoint
*  Returned Value : USB_OK or error code
*  Comments       :
*     Initializes the endpoint and the data structures associated with the 
*  endpoint
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_init_endpoint
   (
    /* [IN] the USB_USB_dev_initialize state structure */
    _usb_device_handle         handle,

    /* [IN] the endpoint structure, include members such as endpoint number, 
     * endpoint type, endpoint direction and the max packet size 
     */                  
    USB_EP_STRUCT_PTR          ep_ptr, 
    
    /* [IN] After all data is transfered, should we terminate the transfer
     * with a zero length packet if the last packet size == MAX_PACKET_SIZE?
     */
    uint8_t                    flag
   )
{
    USB_STATUS                    error = 0;
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    struct xd_struct              temp_xd;
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
 
    /* Initialize the transfer descriptor */
    temp_xd.ep_num = ep_ptr->ep_num;
    temp_xd.bdirection = ep_ptr->direction;
    temp_xd.wmaxpacketsize = (uint16_t)(ep_ptr->size & 0x0000FFFF);
    temp_xd.ep_type = ep_ptr->type;
    temp_xd.dont_zero_terminate = flag;
    temp_xd.wsofar = 0;
 
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
       usb_dev_ptr->usb_dev_interface)->DEV_INIT_ENDPOINT != NULL) 
    {
         error=((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
           usb_dev_ptr->usb_dev_interface)->DEV_INIT_ENDPOINT(usb_dev_ptr->controller_handle, &temp_xd);
    }
    else
    {
         #ifdef _DEV_DEBUG
             printf("usb_device_init_endpoint: DEV_INIT_ENDPOINT is NULL\n");                     
         #endif  
         return USBERR_ERROR;
    }
    
    return error;
} /* EndBody */

/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : usb_device_register_service
* Returned Value : USB_OK or error code
* Comments       :
*     Registers a callback routine for a specified event or endpoint.
* 
*END*--------------------------------------------------------------------*/
USB_STATUS usb_device_register_service
   (
      /* [IN] Handle to the USB device */
      _usb_device_handle         handle,
      
      /* [IN] type of event or endpoint number to service */
      uint8_t                    type,
      
      /* [IN] Pointer to the service's callback function */
      USB_EVENT_SERVICE          service,
      
      /*[IN] User Argument to be passed to Services when invoked.*/
      void*                      arg
   )
{
    USB_DEV_STATE_STRUCT_PTR   usb_dev_ptr;
    SERVICE_STRUCT_PTR         service_ptr;
    SERVICE_STRUCT_PTR *   search_ptr;
 
    if (handle == NULL)
    {
         return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
 
    /* Needs mutual exclusion */
    OS_Lock();
    
    /* Search for an existing entry for type */
    for (search_ptr = &usb_dev_ptr->service_head_ptr;
       *search_ptr;
       search_ptr = &(*search_ptr)->next) 
    {
        if ((*search_ptr)->type == type) 
        {
            /* Found an existing entry */
            OS_Unlock();
            return USBERR_OPEN_SERVICE;
        } /* Endif */
    } /* Endfor */
    
    /* No existing entry found - create a new one */
    service_ptr = (SERVICE_STRUCT_PTR)OS_Mem_alloc_zero(sizeof(SERVICE_STRUCT));
    
    if (!service_ptr) 
    {
        OS_Unlock();
        #ifdef _DEV_DEBUG
          printf("memalloc failed in usb_device_register_service\n");
        #endif    
        return USBERR_ALLOC;
    } /* Endif */
    
    service_ptr->type = type;
    service_ptr->service = service;
    service_ptr->arg = arg;
    service_ptr->next = NULL;
    *search_ptr = service_ptr;
    
    OS_Unlock();
    return USB_OK;
} /* EndBody */

/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : usb_device_unregister_service
* Returned Value : USB_OK or error code
* Comments       :
*     Unregisters a callback routine for a specified event or endpoint.
* 
*END*--------------------------------------------------------------------*/
USB_STATUS usb_device_unregister_service
   (
      /* [IN] Handle to the USB device */
      _usb_device_handle         handle,

      /* [IN] type of event or endpoint number to service */
      uint8_t                     type
   )
{ /* Body */
    USB_DEV_STATE_STRUCT_PTR   usb_dev_ptr;
    SERVICE_STRUCT_PTR         service_ptr;
    SERVICE_STRUCT_PTR *   search_ptr;
 
    if (handle == NULL)
    {
       return USBERR_ERROR;
    }
 
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    /* Needs mutual exclusion */
    OS_Lock();
    
    /* Search for an existing entry for type */
    for (search_ptr = &usb_dev_ptr->service_head_ptr;
         *search_ptr;
         search_ptr = &(*search_ptr)->next) 
    {
        if ((*search_ptr)->type == type) 
        {
            /* Found an existing entry - delete it */
            break;
        }
    }
    
    /* No existing entry found */
    if (!*search_ptr) 
    {
        OS_Unlock();
        return USBERR_CLOSED_SERVICE;
    }
    
    service_ptr = *search_ptr;
    *search_ptr = service_ptr->next;
 
    OS_Mem_free((void*)service_ptr);
    
    OS_Unlock();
    return USB_OK;
} /* EndBody */

 

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_cancel_transfer
*  Returned Value : USB_OK or error code
*  Comments       :
*        returns the status of the transaction on the specified endpoint.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_cancel_transfer
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                     ep_num,
            
      /* [IN] direction */
      uint8_t                     direction
   )
{ 
    uint8_t                        error = USB_OK;
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    
    OS_Lock();
    
    /* Cancel transfer on the specified endpoint for the specified 
     ** direction 
     */
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
      usb_dev_ptr->usb_dev_interface)->DEV_CANCEL_TRANSFER != NULL)   
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
            usb_dev_ptr->usb_dev_interface)->DEV_CANCEL_TRANSFER(usb_dev_ptr->controller_handle, 
            ep_num, direction);
    }
    else
    {
        #if _DEBUG
            printf("usb_device_cancel_transfer: DEV_CANCEL_TRANSFER is NULL\n");               
        #endif  
        return USBERR_ERROR;
    }

    OS_Unlock();

    return error;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_deinit_endpoint
*  Returned Value : USB_OK or error code
*  Comments       :
*  Disables the endpoint and the data structures associated with the 
*  endpoint
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_deinit_endpoint
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                    ep_num,
            
      /* [IN] Direction */
      uint8_t                    direction
   )
{
    uint8_t         error = 0;
    USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
 
    if (handle == NULL)
    {
        return USBERR_ERROR;
    }
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    OS_Lock();
 
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
       usb_dev_ptr->usb_dev_interface)->DEV_DEINIT_ENDPOINT != NULL) 
    {
         error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
             usb_dev_ptr->usb_dev_interface)->DEV_DEINIT_ENDPOINT(usb_dev_ptr->controller_handle, 
             ep_num, direction);
    }
    else
    {
         #if _DEBUG
             printf("usb_device_deinit_endpoint: DEV_DEINIT_ENDPOINT is NULL\n");                     
         #endif  
         return USBERR_ERROR;
    }
    
    OS_Unlock();
    return error;
} 

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_recv_data
*  Returned Value : USB_OK or error code
*  Comments       :
*        Receives data on a specified endpoint.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_recv_data
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                     ep_num,
            
      /* [IN] buffer to receive data */
      uint8_t *                 buff_ptr,
            
      /* [IN] length of the transfer */
      uint32_t                    size
   )
{
    USB_STATUS                       error = USB_OK;
    XD_STRUCT_PTR                    xd_ptr;
    USB_DEV_STATE_STRUCT_PTR         usb_dev_ptr;
    
    if (handle == NULL)
    {
        return USBERR_ERROR;
    }
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
    
    #if PSP_HAS_DATA_CACHE
       /********************************************************
             If system has a data cache, it is assumed that buffer
             passed to this routine will be aligned on a cache line
             boundry. The following code will invalidate the
             buffer before passing it to hardware driver.   
            ********************************************************/
    if(buff_ptr != NULL)
        OS_dcache_invalidate_mlines((void*)buff_ptr,size);   
    #endif
    
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
         usb_dev_ptr->usb_dev_interface)->DEV_GET_XD != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
            usb_dev_ptr->usb_dev_interface)->DEV_GET_XD(usb_dev_ptr->controller_handle, &xd_ptr);    
    }
    else
    {
        #if _DEBUG
            printf("usb_device_recv_data: DEV_GET_XD is NULL\n");
        #endif  
        return USBERR_ERROR;
    }
     
          
    OS_Lock();
 
    /* Initialize the new transfer descriptor */      
    xd_ptr->ep_num = ep_num;
    xd_ptr->bdirection = USB_RECV;
    xd_ptr->wtotallength = size;
    xd_ptr->wstartaddress = buff_ptr;
    xd_ptr->wsofar = 0;
    
    xd_ptr->bstatus = USB_STATUS_TRANSFER_ACCEPTED;
    
 
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
       usb_dev_ptr->usb_dev_interface)->DEV_RECV != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
                 usb_dev_ptr->usb_dev_interface)->DEV_RECV(usb_dev_ptr->controller_handle, xd_ptr);  
    }
    else
    {
        #if _DEBUG
        printf("usb_device_recv_data: DEV_RECV is NULL\n");                      
        #endif    
        return USBERR_ERROR;
    }
        
    OS_Unlock();
    
    if (error) 
    {
        return USBERR_RX_FAILED;
    } /* Endif */
    
    return error;
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_send_data
*  Returned Value : USB_OK or error code
*  Comments       :
*        Sends data on a specified endpoint.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_send_data
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                     ep_num,
            
      /* [IN] buffer to send */
      uint8_t *                  buff_ptr,
            
      /* [IN] length of the transfer */
      uint32_t                    size
   )
{ /* Body */
    USB_STATUS                       error;
    XD_STRUCT_PTR                    xd_ptr;
    volatile USB_DEV_STATE_STRUCT_PTR usb_dev_ptr;

    if (handle == NULL)
    {
        #if _DEBUG
            printf("usb_device_send_data: handle is NULL\n");
        #endif  
        return USBERR_ERROR;
    }

    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
   
    #if PSP_HAS_DATA_CACHE
        /********************************************************
         If system has a data cache, it is assumed that buffer
         passed to this routine will be aligned on a cache line
         boundry. The following code will flush the
         buffer before passing it to hardware driver.   
         ********************************************************/
        OS_dcache_flush_mlines((void*)buff_ptr,size); 
    #endif

    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
        usb_dev_ptr->usb_dev_interface)->DEV_GET_XD != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
            usb_dev_ptr->usb_dev_interface)->DEV_GET_XD(usb_dev_ptr->controller_handle, &xd_ptr);    
    }
    else
    {
        #if _DEBUG
            printf("usb_device_send_data: DEV_GET_XD is NULL\n");
        #endif  
        return USBERR_ERROR;
    }
    
    OS_Lock();

    /* Initialize the new transfer descriptor */      
    xd_ptr->ep_num = ep_num;
    xd_ptr->bdirection = USB_SEND;
    xd_ptr->wtotallength = size;
    xd_ptr->wstartaddress = buff_ptr;
    xd_ptr->wsofar = 0;

    
    xd_ptr->bstatus = USB_STATUS_TRANSFER_ACCEPTED;
    
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
        usb_dev_ptr->usb_dev_interface)->DEV_SEND != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
            usb_dev_ptr->usb_dev_interface)->DEV_SEND(usb_dev_ptr->controller_handle, xd_ptr);    
    }
    else
    {
        #if _DEBUG
            printf("usb_device_send_data: DEV_SEND is NULL\n");
        #endif  
        return USBERR_ERROR;
    }

    OS_Unlock();
   
    if (error) 
    {
        #if _DEBUG
            printf("usb_device_send_data, transfer failed\n");
        #endif  
        return USBERR_TX_FAILED;
    }
    return error;
} 



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_unstall_endpoint
*  Returned Value : USB_OK or error code
*  Comments       :
*     Unstalls the endpoint in specified direction
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_unstall_endpoint
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                     ep_num,
            
      /* [IN] direction */
      uint8_t                     direction
   )
{
    USB_STATUS                        error = USB_OK;
    volatile USB_DEV_STATE_STRUCT_PTR usb_dev_ptr;
    
    if (handle  == NULL)
    {
       #if _DEBUG
          printf("usb_device_unstall_endpoint: handle is NULL\n");
       #endif    
       return USBERR_ERROR;
    }
    
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
 
    OS_Lock();
    
 
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
       usb_dev_ptr->usb_dev_interface)->DEV_UNSTALL_ENDPOINT != NULL)
    {
        error= ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
        usb_dev_ptr->usb_dev_interface)->DEV_UNSTALL_ENDPOINT(usb_dev_ptr->controller_handle, ep_num, direction);   
    }
    else
    {
        return USBERR_ERROR;
    }
    OS_Unlock();
    return error;
} /* EndBody */



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_stall_endpoint
*  Returned Value : USB_OK or error code
*  Comments       :
*     Stalls the endpoint.
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_stall_endpoint
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
            
      /* [IN] the Endpoint number */
      uint8_t                     ep_num,
            
      /* [IN] direction */
      uint8_t                     direction
   )
{
    USB_STATUS                             error = 0;
    volatile USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    
    if (handle == NULL)
    {
       #if _DEBUG
         printf("usb_device_stall_endpoint: handle is NULL\n");
       #endif    
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
 
    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
    usb_dev_ptr->usb_dev_interface)->DEV_STALL_ENDPOINT
        != NULL)
    {
        error = ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
            usb_dev_ptr->usb_dev_interface)->DEV_STALL_ENDPOINT(usb_dev_ptr->controller_handle, 
            ep_num, direction);
    }
    else
    {
        #if _DEBUG
            printf("usb_device_stall_endpoint: DEV_STALL_ENDPOINT is NULL\n");             
        #endif  
        error = USBERR_ERROR;
    }
    
    return  error;
} 

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_process_resume
*  Returned Value : USB_OK or error code
*  Comments       :
*        Process Resume event
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_assert_resume
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle
   )
{
    volatile USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_STATUS                             error = USB_OK;
     
    if (handle == NULL)
    {
       #if _DEBUG
         printf("usb_device_assert_resume: handle is NULL\n");
       #endif    
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;

    if (((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)
         usb_dev_ptr->usb_dev_interface)->DEV_ASSERT_RESUME != NULL)
    {
        error= ((USB_DEV_INTERFACE_FUNCTIONS_STRUCT_PTR)\
            usb_dev_ptr->usb_dev_interface)->DEV_ASSERT_RESUME(usb_dev_ptr->controller_handle);
    }
    else
    {
        #if _DEBUG
            printf("usb_device_assert_resume: DEV_ASSERT_RESUME is NULL\n");               
        #endif  
        error = USBERR_ERROR;
    }
    
    return error;
} 

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_register_application_notify
*  Returned Value : USB_OK or error code
*  Comments       :
*        Process Resume event
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_register_application_notify
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
      USB_DEVICE_NOFIFY          device_notify_callback,
      void*                    device_notify_param
   )
{
    volatile USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_STATUS                             error = USB_OK;
     
         
    if (handle == NULL)
    {
       #if _DEBUG
       printf("usb_device_assert_resume: handle is NULL\n");
       #endif    
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
 
    usb_dev_ptr->usb_framework.device_notify_callback = device_notify_callback;
    usb_dev_ptr->usb_framework.device_notify_param = device_notify_param;
    return error;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : usb_device_register_vendor_class_request_notify
*  Returned Value : USB_OK or error code
*  Comments       :
*        Process Resume event
*
*END*-----------------------------------------------------------------*/
USB_STATUS usb_device_register_vendor_class_request_notify
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
      USB_REQUEST_NOTIFY         request_notify_callback,
      void*                    request_notify_param
   )
{
    volatile USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_STATUS                             error = USB_OK;
 
    if (handle == NULL)
    {
       #if _DEBUG
       printf("usb_device_assert_resume: handle is NULL\n");
       #endif    
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
     
    usb_dev_ptr->usb_framework.request_notify_callback = request_notify_callback;
    usb_dev_ptr->usb_framework.request_notify_param = request_notify_param;

    return error;
}

USB_STATUS usb_device_register_desc_request_notify
   (
      /* [IN] the USB_USB_dev_initialize state structure */
      _usb_device_handle         handle,
      DESC_REQUEST_NOFIFY_STRUCT_PTR desc_request_notify_callback,
      void*                    desc_request_notify_param
   )
{
    volatile USB_DEV_STATE_STRUCT_PTR      usb_dev_ptr;
    USB_STATUS                             error = USB_OK;
    
    if (handle == NULL)
    {
    #if _DEBUG
       printf("usb_device_register_desc_request_notify\n");
    #endif    
       return USBERR_ERROR;
    }
    usb_dev_ptr = (USB_DEV_STATE_STRUCT_PTR)handle;
     
    usb_dev_ptr->usb_framework.desc_notify_callback = desc_request_notify_callback;
    usb_dev_ptr->usb_framework.desc_notify_param    = desc_request_notify_param;
    
    return error;
}

#endif
