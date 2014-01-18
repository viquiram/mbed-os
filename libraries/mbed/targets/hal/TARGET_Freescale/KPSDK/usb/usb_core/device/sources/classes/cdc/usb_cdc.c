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
* $FileName: usb_cdc.c$
* $Version : 
* $Date    : 
*
*
* @brief The file contains CDC layer implimentation.
*
*****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"


#if USBCFG_DEV_CDC
#include "usb_class_cdc.h"
#include "usb_cdc.h"
#include "usb_cdc_pstn.h"
/*****************************************************************************
 * Constant and Macro's
 *****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/
CDC_DEVICE_STRUCT_PTR   cdc_device_array[MAX_CDC_DEVICE];

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
uint8_t USB_Map_Ep_To_Struct_Index(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr, uint8_t ep_num);  
static CDC_HANDLE  USB_Cdc_Allocate_Handle(void);
static int32_t USB_Cdc_Free_Handle(CDC_HANDLE handle);
static CDC_DEVICE_STRUCT_PTR USB_Cdc_Get_Device_Ptr(CDC_HANDLE handle);
/*****************************************************************************
 * Local Variables - None
 *****************************************************************************/
  /*************************************************************************//*!
 *
 * @name  USB_Cdc_Allocate_Handle
 *
 * @brief The funtion reserves entry in device array and returns the index.
 *
 * @param none.
 * @return returns the reserved handle or if no entry found device busy.      
 *
 *****************************************************************************/
static CDC_HANDLE  USB_Cdc_Allocate_Handle(void)
{
    uint32_t cnt = 0;
    for (;cnt< MAX_CDC_DEVICE;cnt++)
    {
        if (cdc_device_array[cnt] == NULL)
            return cnt;
    }
    return USBERR_DEVICE_BUSY;
}
 /*************************************************************************//*!
 *
 * @name  USB_Cdc_Free_Handle
 *
 * @brief The funtion releases entry in device array .
 *
 * @param handle  index in device array to be released..
 * @return returns and error code or USB_OK.      
 *
 *****************************************************************************/

static int32_t USB_Cdc_Free_Handle(CDC_HANDLE handle)
{
    if (/*(handle < 0) || */ (handle > MAX_CDC_DEVICE))
        return USBERR_ERROR;
    
    OS_Mem_free((void *)cdc_device_array[handle]);
    cdc_device_array[handle] = NULL;
    return USB_OK;
}
 /*************************************************************************//*!
 *
 * @name  USB_Cdc_Get_Device_Ptr
 *
 * @brief The funtion gets the device pointer from device array .
 *
 * @param handle  index in device array.
 * @return returns returns pointer to HID device structure..      
 *
 *****************************************************************************/
static CDC_DEVICE_STRUCT_PTR USB_Cdc_Get_Device_Ptr(CDC_HANDLE handle)
{
    if (/*(handle < 0) || */ (handle > MAX_CDC_DEVICE))
        return NULL;
    
     return cdc_device_array[handle]; 
}
 /*****************************************************************************
 * Local Functions
 *****************************************************************************/
 
 /*************************************************************************//*!
 *
 * @name  USB_Cdc_Get_Desc_Info
 *
 * @brief The funtion gets the info of the descriptors. .
 *
 * @param handle  index in device array.
 * @param type     descriptor type.
 * @param object   store the returned value.
 * @return returns USB_OK if successful.      
 *
 *****************************************************************************/
 static uint8_t USB_Cdc_Get_Desc_Info(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,USB_DESC_INFO_T type, uint32_t * object)
 {
	 USB_CLASS_STRUCT_PTR usbclassPtr;
	 /* Get class info */
#if USBCFG_DEV_COMPOSITE
	 USB_COMPOSITE_INFO_STRUCT_PTR usbcompinfoPtr;
		cdc_obj_ptr->desc_callback.GET_DESC_ENTITY((uint32_t)cdc_obj_ptr->controller_handle,
													USB_COMPOSITE_INFO,
													(uint32_t *)&usbcompinfoPtr);
		usbclassPtr = usbcompinfoPtr->class;
#else
		cdc_obj_ptr->desc_callback.GET_DESC_ENTITY((uint32_t)cdc_obj_ptr->controller_handle,
													USB_CLASS_INFO,
													(uint32_t *)&usbclassPtr);
#endif
	 switch(type)
	 {
	     case USB_EP_COUNT:
	     {
	         USB_IF_STRUCT *if_ptr;
	         uint32_t ep_cnt = 0;
	         for_each_if_in_class(if_ptr, usbclassPtr, USB_CLASS_ALL)
	         {
	             ep_cnt += if_ptr->endpoints.count;
	         }
	         *object = ep_cnt;
	         break;
	     }
	     case USB_CDC_EP_COUNT:
	     {
	         USB_IF_STRUCT *if_ptr;
	         uint32_t ep_cnt = 0;
	         for_each_if_in_class(if_ptr, usbclassPtr, USB_CLASS_CDC)
	         {
	             ep_cnt += if_ptr->endpoints.count;
	         }
	         *object = ep_cnt;
	         break;
	     }
	     case USB_INTERFACE_COUNT:
	     {
	         USB_IF_STRUCT *if_ptr = NULL;
	         uint32_t if_cnt = 0;
             if_ptr = if_ptr;
	         for_each_if_in_class(if_ptr, usbclassPtr, USB_CLASS_ALL)
	         {
	             if_cnt++;
	         }
	         *object = if_cnt;
	         break;
	     }
	     case USB_CDC_INTERFACE_COUNT:
	     {
	         USB_IF_STRUCT *if_ptr = NULL;
	         uint32_t if_cnt = 0;
             if_ptr = if_ptr;
	         for_each_if_in_class(if_ptr, usbclassPtr, USB_CLASS_CDC)
	         {
	             if_cnt++;
	         }
	         *object = if_cnt;
	         break;
	     }
	     default :
	         break;
	 } 
     return USB_OK;
}
/**************************************************************************//*!
 *
 * @name  USB_Map_Ep_To_Struct_Index
 *
 * @brief The funtion maps the endpoint num to the index of the ep data 
 *           structure
 *
 * @param handle          handle to identify the controller
 * @param ep_num          endpoint num
 *
 * @return index          mapped index       
 *
 *****************************************************************************/
uint8_t USB_Map_Ep_To_Struct_Index(CDC_DEVICE_STRUCT_PTR cdc_obj_ptr,uint8_t ep_num) 
{
    uint8_t index = 0;
    uint32_t ep_count;
    
	USB_Cdc_Get_Desc_Info(cdc_obj_ptr, USB_EP_COUNT, &ep_count);
    /* map the endpoint num to the index of the endpoint structure */
    for(index = 0; index < ep_count; index++)
    {
        if(cdc_obj_ptr->ep[index].endpoint == ep_num)
            break;
    }    
    return index;     
}

/**************************************************************************//*!
 *
 * @name  USB_Service_Cdc_Notif
 *
 * @brief The funtion ic callback function of CIC Notification endpoint 
 *
  * @param event
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Service_Cdc_Notif(PTR_USB_EVENT_STRUCT event,void* arg)
{    
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr = (CDC_DEVICE_STRUCT_PTR)arg;
    #if CDC_IMPLEMENT_QUEUING
        uint8_t index;
        uint8_t producer, consumer;
//        USB_ENDPOINTS *usb_ep_data = cdc_obj_ptr->usb_ep_data;
        
        USB_CLASS_CDC_QUEUE queue;
    #endif
    
    UNUSED_ARGUMENT (event)
    
    #if CDC_IMPLEMENT_QUEUING
        /* map the endpoint num to the index of the endpoint structure */
        index = USB_Map_Ep_To_Struct_Index(cdc_obj_ptr, event->ep_num); 

        producer = cdc_obj_ptr->ep[index].bin_producer;
            
        /* if there are no errors de-queue the queue and decrement the no. of 
           transfers left, else send the same data again */
        cdc_obj_ptr->ep[index].bin_consumer++;              
        consumer = cdc_obj_ptr->ep[index].bin_consumer;
            
        if(consumer != producer) 
        {/*if bin is not empty */                           
            queue = cdc_obj_ptr->ep[index].queue[consumer%CDC_MAX_QUEUE_ELEMS];                         
            (void)USB_Class_Send_Data(cdc_obj_ptr->class_handle, queue.channel, 
                queue.app_data.data_ptr, queue.app_data.data_size);
        }        
    #endif
    
    if(cdc_obj_ptr->class_specific_callback.callback != NULL) 
    {
        uint8_t event_type = USB_DEV_EVENT_SEND_COMPLETE;
		cdc_obj_ptr->class_specific_callback.callback(event_type,
				USB_REQ_VAL_INVALID,
				NULL,
				NULL,
				cdc_obj_ptr->class_specific_callback.arg);
    }   
}

/**************************************************************************//*!
 *
 * @name  USB_Service_Dic_Bulk_In
 *
 * @brief The funtion ic callback function of DIC Bulk In Endpoint 
 *
 * @param event
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Service_Dic_Bulk_In(PTR_USB_EVENT_STRUCT event,void* arg)
{
    uint8_t event_type;
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr = (CDC_DEVICE_STRUCT_PTR)arg;

    if(USB_UNINITIALIZED_VAL_32 == event->len) return;
    #if CDC_IMPLEMENT_QUEUING
        uint8_t index;
        uint8_t producer, consumer;
//        USB_ENDPOINTS *usb_ep_data = cdc_obj_ptr->usb_ep_data;
        
        USB_CLASS_CDC_QUEUE queue;

        /* map the endpoint num to the index of the endpoint structure */
        index = USB_Map_Ep_To_Struct_Index(cdc_obj_ptr, event->ep_num); 
        producer = cdc_obj_ptr->ep[index].bin_producer;         
        /* if there are no errors de-queue the queue and decrement the no. of 
           transfers left, else send the same data again */
        cdc_obj_ptr->ep[index].bin_consumer++;                  
        consumer = cdc_obj_ptr->ep[index].bin_consumer;
            
        if(consumer != producer) 
        {/*if bin is not empty */
                            
            queue = cdc_obj_ptr->ep[index].queue[consumer%CDC_MAX_QUEUE_ELEMS];
                            
            (void)USB_Class_Send_Data(cdc_obj_ptr->class_handle, queue.channel, 
                queue.app_data.data_ptr, queue.app_data.data_size);
        }          
    #endif
    
    if(cdc_obj_ptr->class_specific_callback.callback != NULL) 
    {
        event_type = USB_DEV_EVENT_SEND_COMPLETE;
		cdc_obj_ptr->class_specific_callback.callback(event_type,
				USB_REQ_VAL_INVALID,
				&(event->buffer_ptr),
				&(event->len),
				cdc_obj_ptr->class_specific_callback.arg);
    }
}

/**************************************************************************//*!
 *
 * @name  USB_Service_Dic_Bulk_Out
 *
 * @brief The funtion ic callback function of DIC Bulk Out Endpoint 
 *
 * @param event
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Service_Dic_Bulk_Out(PTR_USB_EVENT_STRUCT event,void* arg)
{
    uint8_t event_type; 
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr = (CDC_DEVICE_STRUCT_PTR)arg;
      
    if(USB_UNINITIALIZED_VAL_32 == event->len) return;
    event_type = USB_DEV_EVENT_DATA_RECEIVED;
    if(cdc_obj_ptr->class_specific_callback.callback != NULL) 
    {
	  cdc_obj_ptr->class_specific_callback.callback(event_type,
			  USB_REQ_VAL_INVALID,
			  &(event->buffer_ptr),
			  &(event->len),
			  cdc_obj_ptr->class_specific_callback.arg);
    }
 }

/**************************************************************************//*!
 *
 * @name  USB_Class_CDC_Event
 *
 * @brief The funtion initializes CDC endpoints 
 *
 * @param handle   handle to Identify the controller
 * @param event           pointer to event structure
 * @param val             gives the configuration value 
 *
 * @return None       
 *
 *****************************************************************************/
 void USB_Class_CDC_Event(uint8_t event, void* val,void* arg) 
{  
    USB_CLASS_STRUCT_PTR usbclassPtr;
#if USBCFG_DEV_COMPOSITE
	USB_COMPOSITE_INFO_STRUCT_PTR usbcompinfoPtr;
#endif
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr = NULL;
    cdc_obj_ptr = USB_Cdc_Get_Device_Ptr((CDC_HANDLE)arg);
#if CDC_IMPLEMENT_QUEUING
    uint8_t index;
#endif
    if(event == USB_DEV_EVENT_CONFIG_CHANGED)
    {
        uint32_t ep_count, max_if_count;
        USB_EP_STRUCT_PTR ep_struct_ptr = NULL;
        /* Set configuration according to config number*/
        cdc_obj_ptr->desc_callback.SET_CONFIGURATION((uint32_t)cdc_obj_ptr->controller_handle, (uint8_t)(*(uint16_t*)val));
        /* Get class info */
#if USBCFG_DEV_COMPOSITE
		cdc_obj_ptr->desc_callback.GET_DESC_ENTITY((uint32_t)cdc_obj_ptr->controller_handle,
													USB_COMPOSITE_INFO,
													(uint32_t*)&usbcompinfoPtr);
		usbclassPtr = usbcompinfoPtr->class;
#else
        cdc_obj_ptr->desc_callback.GET_DESC_ENTITY((uint32_t)cdc_obj_ptr->controller_handle,
													USB_CLASS_INFO,
													(uint32_t*)&usbclassPtr);
#endif
        /* Get count of endpoints for a specific configuration */
		USB_Cdc_Get_Desc_Info(cdc_obj_ptr, USB_CDC_EP_COUNT, &ep_count);
        /* Get count of interfaces for a specific configuration */
		USB_Cdc_Get_Desc_Info(cdc_obj_ptr, USB_CDC_INTERFACE_COUNT, &max_if_count);
        cdc_obj_ptr->max_supported_interfaces = max_if_count;
        if(NULL == cdc_obj_ptr->ep)
        {
            cdc_obj_ptr->ep = (USB_CLASS_CDC_ENDPOINT *)OS_Mem_alloc_zero(sizeof(USB_CLASS_CDC_ENDPOINT) * (ep_count));
#if CDC_IMPLEMENT_QUEUING
             index = 0;
             
             for_each_ep_in_class(ep_struct_ptr, usbclassPtr, USB_CLASS_CDC)
             {
                cdc_obj_ptr->ep[index].endpoint = ep_struct_ptr->ep_num;
                cdc_obj_ptr->ep[index].type = ep_struct_ptr->type;
                cdc_obj_ptr->ep[index].bin_consumer = 0x00;
                cdc_obj_ptr->ep[index].bin_producer = 0x00;
                index++;
              }
#endif
        }
        /* intialize all non control endpoints */            
        for_each_ep_in_class(ep_struct_ptr, usbclassPtr, USB_CLASS_CDC)
        {
            (void)usb_device_init_endpoint(cdc_obj_ptr->controller_handle,
             ep_struct_ptr,TRUE);
  
            /* register callback service for Non Control EndPoints */
            switch(ep_struct_ptr->type) 
            {
                case USB_INTERRUPT_PIPE :
                    (void)usb_device_register_service(cdc_obj_ptr->controller_handle,
                        (uint8_t)(USB_SERVICE_EP0+ep_struct_ptr->ep_num),
                        USB_Service_Cdc_Notif,(void *)cdc_obj_ptr);
                    cdc_obj_ptr->cic_recv_endpoint = USB_CONTROL_ENDPOINT;
                    cdc_obj_ptr->cic_send_endpoint = ep_struct_ptr->ep_num;
                    cdc_obj_ptr->cic_send_pkt_size = ep_struct_ptr->size;
                    break;                              
                case USB_BULK_PIPE :
                    if(ep_struct_ptr->direction == USB_RECV) 
                    {
                        (void)usb_device_register_service(cdc_obj_ptr->controller_handle,
                            (uint8_t)(USB_SERVICE_EP0+ep_struct_ptr->ep_num),
                            USB_Service_Dic_Bulk_Out,(void *)cdc_obj_ptr);
                        cdc_obj_ptr->dic_recv_endpoint = ep_struct_ptr->ep_num;
                        cdc_obj_ptr->dic_recv_pkt_size = ep_struct_ptr->size;
                    } 
                    else
                    {
                        (void)usb_device_register_service(cdc_obj_ptr->controller_handle,
                            (uint8_t)(USB_SERVICE_EP0+ep_struct_ptr->ep_num),
                            USB_Service_Dic_Bulk_In,(void *)cdc_obj_ptr);
                        cdc_obj_ptr->dic_send_endpoint = ep_struct_ptr->ep_num;
                        cdc_obj_ptr->dic_send_pkt_size = ep_struct_ptr->size;
                    }
                    break;
                default : break;        
            }
            
        }
	}
	else if(event == USB_DEV_EVENT_ENUM_COMPLETE)
    {
        /* To Do */
    }
    #if USBCFG_DEV_RNDIS_SUPPORT
    else if(event == USB_DEV_EVENT_BUS_RESET)
    {
        uint8_t * data; 
        uint32_t size;
        RNDIS_Reset_Command(cdc_obj_ptr, &data, &size);     
    }
    #endif        
    if(cdc_obj_ptr->cdc_application_callback.callback != NULL) 
    {
        cdc_obj_ptr->cdc_application_callback.callback(event,
            val,cdc_obj_ptr->cdc_application_callback.arg);
    } 
}

/**************************************************************************//*!
 *
 * @name  USB_CDC_Other_Requests
 *
 * @brief The funtion provides flexibilty to add class and vendor specific
 *        requests 
 *
 * @param handle
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned    
 *
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
uint8_t USB_CDC_Other_Requests
(   
    USB_SETUP_STRUCT * setup_packet, 
    uint8_t * *data, 
    uint32_t *size,
    void* arg
) 
{
    uint8_t status;
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr = NULL;
    cdc_obj_ptr = USB_Cdc_Get_Device_Ptr((CDC_HANDLE)arg);
    status = USBERR_INVALID_REQ_TYPE;
    if((setup_packet->request_type & USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_POS) == 
        USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_CLASS) 
    {  /* class request so handle it here */     
        status=USB_OK;
        /* call for class/subclass specific requests */
        switch(setup_packet->request) 
        {
            case SEND_ENCAPSULATED_COMMAND :
                #if USBCFG_DEV_RNDIS_SUPPORT               
                    /* Pass the Remote NDIS Control Message supoported by 
                       802.3 connectionless device to PSTN Layer */
                    status = PSTN_Rndis_Message_Set(cdc_obj_ptr,setup_packet,data,size); 
                #endif
                *size = 0;              
                break;
            case GET_ENCAPSULATED_RESPONSE :
                #if USBCFG_DEV_RNDIS_SUPPORT               
                    /* Get the Remote NDIS Control Message supoported by 
                       802.3 connectionless device from  PSTN Layer */
                    status = PSTN_Rndis_Message_Get(cdc_obj_ptr,setup_packet,data,size); 
                #else
                    *size = 0;
                    /* protocol says to return zero byte data instead of stalling 
                       the command if you don't have data to return */
                    status = USB_OK;                
                #endif
                break; 
            case SET_COMM_FEATURE :
                status = PSTN_Set_Comm_Feature(cdc_obj_ptr,setup_packet,data,size);  
                break;
            case GET_COMM_FEATURE :
                status = PSTN_Get_Comm_Feature(cdc_obj_ptr,setup_packet,data,size); 
                break;
            case CLEAR_COMM_FEATURE : /* Verify this implementation */
                *size = cdc_obj_ptr->comm_feature_data_size;
                **data = 0x00; *(++(*data)) = 0x00;/*clear both feature bytes*/
                status = PSTN_Set_Comm_Feature(cdc_obj_ptr,
                setup_packet,data,size);  
                break; 
            case GET_LINE_CODING :               
                status = PSTN_Get_Line_Coding(cdc_obj_ptr,
                setup_packet,data,size); 
                break;
            case SET_LINE_CODING :               
                status = PSTN_Set_Line_Coding(cdc_obj_ptr,setup_packet,data,size); 
                break;    
            case SET_CONTROL_LINE_STATE : 
                status = PSTN_Set_Ctrl_Line_State(cdc_obj_ptr,
                setup_packet,data,size); 
                break;
            case SEND_BREAK : 
                status = PSTN_Send_Break(cdc_obj_ptr,setup_packet,data,size);
                break;
            default :  *size=0;break;        
        }
    } 
    else if((setup_packet->request_type & USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_POS) == 
        USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_VENDOR) 
    {   /* vendor specific request  */    
        if(cdc_obj_ptr->vendor_req_callback.callback != NULL) 
        {
            status = cdc_obj_ptr->vendor_req_callback.callback(setup_packet,
            data,size,cdc_obj_ptr->vendor_req_callback.arg);
        }
    }
        
    return status;
}


/*****************************************************************************
 * Global Functions
 *****************************************************************************/


/**************************************************************************//*!
 *
 * @name  USB_Class_CDC_Init
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param *cdc_config_ptr[IN]:  This structure contians configuration parameter
 *                              send by APP to configure CDC class.
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 *
 *This function initializes the CDC Class layer and layers it is dependednt on 
 *
 *****************************************************************************/
uint32_t USB_Class_CDC_Init
(
    uint8_t controller_id,
    CDC_CONFIG_STRUCT_PTR cdc_config_ptr,
    CDC_HANDLE *  cdc_handle_ptr
)
{
    uint8_t error = USB_OK;
    CDC_HANDLE cdc_handle;
    CDC_DEVICE_STRUCT_PTR devicePtr = NULL;
    
    if (NULL == (void *)cdc_config_ptr)
        return USBERR_ERROR;
    
    devicePtr = (CDC_DEVICE_STRUCT_PTR)OS_Mem_alloc_zero(sizeof(CDC_DEVICE_STRUCT));
    if (NULL == devicePtr) 
    {
        #if _DEBUG
            printf("memalloc failed in USB_Class_CDC_Init\n");
        #endif  
        return USBERR_ALLOC;    
    }
    
    cdc_handle = USB_Cdc_Allocate_Handle();
    if (USBERR_DEVICE_BUSY == cdc_handle) 
    {
        OS_Mem_free(devicePtr);
        devicePtr = NULL;
        return USBERR_INIT_FAILED;
    }

        /* initialize the Global Variable Structure */
    OS_Mem_zero(devicePtr, sizeof(CDC_DEVICE_STRUCT));

    devicePtr->ep = NULL;
#if USBCFG_DEV_COMPOSITE
    devicePtr->class_handle = USB_Class_Get_Class_Handle();
    devicePtr->controller_handle =(_usb_device_handle)USB_Class_Get_Ctrler_Handle(devicePtr->class_handle);
    if(NULL == devicePtr->controller_handle)
    {
        goto error1;
    }
#else
	/* Initialize the device layer*/
	error = usb_device_init(0, &devicePtr->controller_handle);
	/* +1 is for Control Endpoint */ 
	if(error != USB_OK)
	{
	  goto error1;  
	}

	/* Initialize the generic class functions */
	devicePtr->class_handle = USB_Class_Init(devicePtr->controller_handle,
	USB_Class_CDC_Event,USB_CDC_Other_Requests,(void *)cdc_handle,
	cdc_config_ptr->desc_callback_ptr);

#endif
    /* Initialize the generic class functions */
    #if PSTN_SUBCLASS_NOTIF_SUPPORT
    /* Initialize the pstn subclass functions */
    error = USB_Pstn_Init(devicePtr,&cdc_config_ptr->cdc_application_callback);
    if(error != USB_OK)
    {
      goto error2;  
    }
    #endif
    /* Save the desc callback to ask application for class specific params*/
    OS_Mem_copy(cdc_config_ptr->desc_callback_ptr,
    &devicePtr->desc_callback ,sizeof(DESC_REQUEST_NOFIFY_STRUCT));
    /* save the callback pointer */
    devicePtr->cdc_application_callback.callback = cdc_config_ptr->cdc_application_callback.callback;
    devicePtr->cdc_application_callback.arg = cdc_config_ptr->cdc_application_callback.arg;         
    /* save the callback pointer */
    devicePtr->vendor_req_callback.callback = 
    cdc_config_ptr->vendor_req_callback.callback;
    devicePtr->vendor_req_callback.arg = cdc_config_ptr->vendor_req_callback.arg; 
    /* save the callback pointer */
    devicePtr->class_specific_callback.callback = cdc_config_ptr->class_specific_callback.callback;
    devicePtr->class_specific_callback.arg = cdc_config_ptr->class_specific_callback.arg; 
    devicePtr->comm_feature_data_size =  COMM_FEATURE_DATA_SIZE;
      
    devicePtr->cdc_handle = cdc_handle;
   
   cdc_device_array[cdc_handle] = devicePtr;
   *cdc_handle_ptr = cdc_handle;
#if USBCFG_DEV_RNDIS_SUPPORT
    devicePtr->desc_callback.GET_DESC_ENTITY((uint32_t)devicePtr->controller_handle,
	   USB_RNDIS_INFO,
	   (uint32_t*)&devicePtr->rndis_info);
#endif
   return USB_OK;
 error2:
  /* Implement Denit class and invoke here*/    
 error1: 
     USB_Cdc_Free_Handle(cdc_handle);
     OS_Mem_free(devicePtr);
     devicePtr = NULL;    
    return error;    
}

/**************************************************************************//*!
 *
 * @name  USB_Class_CDC_Deinit
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
 *This function initializes the CDC Class layer and layers it is dependednt on 
 *
 *****************************************************************************/
uint32_t USB_Class_CDC_Deinit
(
  CDC_HANDLE cdc_handle
)
{   

    uint8_t error = USB_OK;
    CDC_DEVICE_STRUCT_PTR devicePtr = NULL;
    devicePtr = USB_Cdc_Get_Device_Ptr(cdc_handle);

    #if PSTN_SUBCLASS_NOTIF_SUPPORT
    /* deinitialize the pstn subclass functions */
    error = USB_Pstn_Deinit(devicePtr);
    #endif
#if !USBCFG_DEV_COMPOSITE
    if(error == USB_OK)
        /* deinitialize the generic class functions */
        error = USB_Class_Deinit(devicePtr->controller_handle,devicePtr->class_handle);
    if(error == USB_OK)
        /* deinitialize the device layer*/
        error = usb_device_deinit(devicePtr->controller_handle);
#endif
    if(NULL != devicePtr->ep)
    {
        OS_Mem_free(devicePtr->ep);
    }

    if(error == USB_OK)
        error = USB_Cdc_Free_Handle(cdc_handle);

    devicePtr = NULL;
    return error;    
}
/**************************************************************************//*!
 *
 * @name  USB_Class_CDC_Send_Data
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_CDC_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK         : When Successfull 
 *         Others         : Errors
 *****************************************************************************/
uint8_t USB_Class_CDC_Send_Data
(
    CDC_HANDLE handle,
    uint8_t ep_num,
    uint8_t * app_buff,
    uint32_t size
) 
{
    #if CDC_IMPLEMENT_QUEUING
        uint8_t index;
        uint8_t producer, consumer; 
//        USB_ENDPOINTS *usb_ep_data; 
    #endif    
        
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr;
    uint8_t status = USB_OK;
    cdc_obj_ptr = USB_Cdc_Get_Device_Ptr(handle);
    if (NULL == cdc_obj_ptr)
      return USBERR_ERROR;
    
    #if CDC_IMPLEMENT_QUEUING
//        usb_ep_data = cdc_obj_ptr->usb_ep_data;
      
        /* map the endpoint num to the index of the endpoint structure */
        index = USB_Map_Ep_To_Struct_Index(cdc_obj_ptr, ep_num); 
                
        producer = cdc_obj_ptr->ep[index].bin_producer;
		OS_Lock();
        consumer = cdc_obj_ptr->ep[index].bin_consumer;
                                
        if(((uint8_t)(producer - consumer)) != (uint8_t)(CDC_MAX_QUEUE_ELEMS))  
        {   /* the bin is not full*/        
            uint8_t queue_num = (uint8_t)(producer % CDC_MAX_QUEUE_ELEMS);
            
            /* put all send request parameters in the endpoint data structure*/
            cdc_obj_ptr->ep[index].queue[queue_num].channel = ep_num;
            cdc_obj_ptr->ep[index].queue[queue_num].app_data.data_ptr = app_buff;
            cdc_obj_ptr->ep[index].queue[queue_num].app_data.data_size = size; 
            cdc_obj_ptr->ep[index].queue[queue_num].handle = 
            cdc_obj_ptr->controller_handle;         
            /* increment producer bin by 1*/       
            cdc_obj_ptr->ep[index].bin_producer = ++producer;

            if((uint8_t)(producer - consumer) == (uint8_t)1)         
            {          
    #endif      
                status = USB_Class_Send_Data(cdc_obj_ptr->class_handle,
                 ep_num, app_buff,size);
    #if CDC_IMPLEMENT_QUEUING
            }
        }
        else /* bin is full */
        {
            status = USBERR_DEVICE_BUSY; 
        }    
		OS_Unlock();
    #endif 
    return status;
}
/**************************************************************************//*!
 *
 * @name  USB_Class_CDC_Recv_Data
 *
 * @brief This functions receives Data from Host.
 *
 * @param handle          :   handle returned by USB_Class_CDC_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK         : When Successfull 
 *         Others         : Errors
 *****************************************************************************/
uint8_t USB_Class_CDC_Recv_Data
(
    CDC_HANDLE          cdc_handle,
    uint8_t              ep_num,
    uint8_t *          buff_ptr,      /* [IN] buffer to recv */      
    uint32_t             size           /* [IN] length of the transfer */
)
{
    CDC_DEVICE_STRUCT_PTR cdc_obj_ptr;
    uint8_t status = USB_OK;

    cdc_obj_ptr = USB_Cdc_Get_Device_Ptr(cdc_handle);
    if (NULL == cdc_obj_ptr)
    {
        return USBERR_ERROR;    
    }
    status = usb_device_recv_data(cdc_obj_ptr->controller_handle,
    ep_num,buff_ptr,size);
    
    return status;
    
}

#endif /*CDC_CONFIG*/
/* EOF */
