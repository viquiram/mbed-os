/**HEADER********************************************************************
* 
* Copyright (c) 2013 Freescale Semiconductor;
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
* $FileName: osadapter_ucos.c$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file includes the implementation of OS adapter based on SDK OSA.
* 
*****************************************************************************/ 
#include "adapter_cfg.h"
#include "adapter_types.h"
#include "adapter_sdk.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"


uint8_t soc_get_usb_vector_number(uint8_t controller_id)
{
    if (controller_id == 0)
        return USB0_IRQn;
    return 0;
}


OS_GPIO_handle OS_Gpio_init(uint32_t id, uint32_t dir, uint32_t value)
{
    return (OS_GPIO_handle)(0x0000FFFF);
}

uint32_t OS_Gpio_set_functionality(OS_GPIO_handle handle, uint32_t function)
{
    return OS_GPIO_OK;
}

uint32_t OS_Gpio_set_value(OS_GPIO_handle handle, uint32_t value)
{
    return OS_GPIO_OK;
}

uint32_t OS_Gpio_deinit(OS_GPIO_handle handle)
{
    return OS_GPIO_OK;
}

uint32_t OS_Task_create(TASK_START pstart, void* param, uint32_t pri, uint32_t stack_size, char* task_name, void* opt)
{
    task_handler_t task_handler;
    void *stack_mem = (void*)0;
#if ((FSL_RTOS_SELECTED == FSL_RTOS_UCOSII) || (FSL_RTOS_SELECTED == FSL_RTOS_UCOSIII))
    stack_mem = mem_allocate(stack_size);
    if (!stack_mem)
    {
        return 0;
    }
#endif
    fsl_rtos_status status = __task_create((task_t)pstart, task_name, stack_size, stack_mem, pri, param, 0, &task_handler);
    
    if (kSuccess == status)
    {
        return (uint32_t)task_handler;
    }
    else
    {
#if ((FSL_RTOS_SELECTED == FSL_RTOS_UCOSII) || (FSL_RTOS_SELECTED == FSL_RTOS_UCOSIII))
        mem_free(stack_mem);
#endif
        return 0;
    }

}

uint32_t OS_Task_delete(uint32_t task_id)
{
    return (kSuccess == task_destroy((task_handler_t)task_id)) ? OS_TASK_OK : OS_TASK_ERROR;
}

/* Event create and destroy */
OS_Event_handle OS_Event_create(uint32_t flag)
{
    event_object_t *p_event = mem_allocate(sizeof(event_object_t));

    if (!p_event)
    {
        return (OS_Event_handle)0;
    }

    if (kSuccess != event_create(p_event, flag ? kEventAutoClr : kEventManualClr))
    {
        mem_free(p_event);
        return (OS_Event_handle)0;
    }
    return (OS_Event_handle)p_event;
}

uint32_t OS_Event_destroy(OS_Event_handle handle)
{
    event_object_t* obj = (event_object_t*)handle;

    if (kSuccess == event_destroy(obj))
    {
        mem_free(handle);
        return OS_EVENT_OK; 
    }
    else
    {
        return OS_EVENT_ERROR;
    }
}

/* Message queue create and destroy */
/*
 * NOTE: The msg_size here is counted by words, not bytes!
 */
OS_MsgQ_handle OS_MsgQ_create(uint32_t max_msg_number, uint32_t msg_size)
{
    OS_MsgQ_handle ret;
#if (FSL_RTOS_SELECTED == FSL_RTOS_UCOSII)
    uint8_t *p_tmp;
    uint32_t size = sizeof(MSGQ_STRUCT_UCOS) + (msg_size*sizeof(int32_t)+sizeof(void*))*max_msg_number;
#elif (FSL_RTOS_SELECTED == FSL_RTOS_MQX)
    uint32_t size = sizeof(LWMSGQ_STRUCT) + (msg_size*sizeof(int32_t))*max_msg_number;
#elif (FSL_RTOS_SELECTED == FSL_RTOS_FREE_RTOS)
    OS_MsgQ_handle handle;
#elif (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    uint8_t *p_tmp;
    uint32_t size = sizeof(msg_queue_t) + sizeof(uint32_t)*max_msg_number*msg_size;
#endif

    msg_queue_t* msgq = (msg_queue_t*)mem_allocate(size);
    
    if (!msgq)
    {
        return (msg_queue_handler_t)0;
    }
    /* initialize the msg_queue_t */
#if (FSL_RTOS_SELECTED == FSL_RTOS_UCOSII)
    p_tmp = (uint8_t*)msgq;
    p_tmp += sizeof(msg_queue_t);
    msgq->msgTbl = (void**)p_tmp;
    p_tmp += max_msg_number*sizeof(void*);
    msgq->msgs = (uint32_t*)p_tmp;
#elif (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    p_tmp = (uint8_t*)msgq;
    p_tmp += sizeof(msg_queue_t);
    msgq->queueMem = (uint32_t*)p_tmp;
#endif
    ret = msg_queue_create(msgq, max_msg_number, msg_size);
    if (!ret)
    {
        mem_free(msgq);
    }
    return ret;
}

uint32_t OS_MsgQ_destroy(OS_MsgQ_handle msgq)
{
#if (FSL_RTOS_SELECTED == FSL_RTOS_UCOSII) || (FSL_RTOS_SELECTED == FSL_RTOS_MQX)
    if (kSuccess == msg_queue_destroy(msgq))
    {
        mem_free(msgq);
        return OS_MSGQ_OK;
    }
    else
    {
        return OS_MSGQ_ERROR;
    }
#elif (FSL_RTOS_SELECTED == FSL_RTOS_FSL_RTOS_FREE_RTOS)
    if (kSuccess == msg_queue_destroy(msgq))
    {
        return OS_MSGQ_OK;
    }
    else
    {
        return OS_MSGQ_ERROR;
    }
#else
    return 0;
#endif
}


/* Mutex create and destroy */
OS_Mutex_handle OS_Mutex_create()
{
#if (FSL_RTOS_SELECTED == FSL_RTOS_FREE_RTOS)
    lock_object_t *p_lock;
    lock_create(p_lock);
    if (p_lock == NULL)
    {
        return (OS_Mutex_handle)0;
    }
#else
    lock_object_t *p_lock = mem_allocate(sizeof(lock_object_t));

    if (!p_lock)
    {
        return (OS_Mutex_handle)0;
    }
    if (kSuccess != lock_create(p_lock))
    {
        mem_free(p_lock);
        return (OS_Mutex_handle)0;
    }
#endif
    return (OS_Mutex_handle)p_lock;
}

uint32_t OS_Mutex_destroy(OS_Mutex_handle handle)
{
    lock_object_t* obj = (lock_object_t*)handle;

    if (kSuccess == lock_destroy(obj))
    {
        mem_free(handle);
        return OS_MUTEX_OK;
    }
    else
    {
        return OS_MUTEX_ERROR;
    }
}

/* Semaphore create and destroy */
OS_Sem_handle OS_Sem_create(int32_t initial_number)
{
    sync_object_t *p_sync = mem_allocate(sizeof(sync_object_t));

    if (!p_sync)
    {
        return (OS_Sem_handle)0;
    }
#if (FSL_RTOS_SELECTED == FSL_RTOS_FREE_RTOS)
    if (kSuccess != sync_create(p_sync))
    {
        mem_free(p_sync);
        return (OS_Sem_handle)0;
    }
#else
    if (kSuccess != sync_create(p_sync, initial_number))
    {
        mem_free(p_sync);
        return (OS_Sem_handle)0;
    }
#endif
    return (OS_Sem_handle)p_sync;
}

uint32_t OS_Sem_destroy(OS_Sem_handle handle)
{
    sync_object_t* obj = (sync_object_t*)handle;
    if (kSuccess == sync_destroy(obj))
    {
        mem_free(handle);
        return OS_SEM_OK;
    }
    else
    {
        return OS_SEM_ERROR;
    }
}


/* Task management */

uint32_t OS_Task_suspend(uint32_t task_id)
{
    return ((kSuccess==task_suspend((task_handler_t)(task_id))) ? OS_TASK_OK : OS_TASK_ERROR);
}

uint32_t OS_Task_resume(uint32_t task_id)  
{
    return ((kSuccess==task_resume((task_handler_t)(task_id))) ? OS_TASK_OK : OS_TASK_ERROR);
}
/* Events */
uint32_t OS_Event_check_bit(event_object_t* handle,uint32_t bitmask) 
{
    return ((uint32_t)event_check_flags((event_object_t*)(handle), (bitmask)));
}

uint32_t OS_Event_set(event_object_t* handle, uint32_t bitmask)    
{
    return ((kSuccess==event_set((event_object_t*)(handle), (bitmask))) ? OS_EVENT_OK : OS_EVENT_ERROR);
}

uint32_t OS_Event_clear(event_object_t* handle, uint32_t bitmask)   
{
    return((kSuccess==event_clear((event_object_t*)(handle), (bitmask))) ? OS_EVENT_OK : OS_EVENT_ERROR);
}

uint32_t OS_Event_wait(event_object_t* handle,uint32_t bitmask,uint32_t flag,uint32_t timeout)         
{
    fsl_rtos_status status = kIdle;
    event_group_t event_bit;
    // flag will always be false, so wait any bits
#if (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    if (0 == timeout) // If timeout is 0 and non-blocking
    {
        if (kFlagSet ==event_check_flags(handle, bitmask))
        {
            return OS_EVENT_OK;
        }
        else
        {
            return OS_EVENT_ERROR;
        }
    }
#else
    if (0 == timeout)
    {
        timeout = kSyncWaitForever;
    }
#endif
    // Block or timeout mode
    do {
        status = event_wait(handle, timeout, &event_bit);
    } while (kIdle == status);

    switch (status)
    {
        case kSuccess:
            return OS_EVENT_OK;
        case kTimeout:
            return OS_EVENT_TIMEOUT;
        default:
            return OS_EVENT_ERROR;
    }
}

/* Semaphore */
uint32_t OS_Sem_post(sync_object_t* handle)     
{
    return ((kSuccess==sync_signal((sync_object_t*)(handle))) ? OS_SEM_OK : OS_SEM_ERROR);
}

uint32_t OS_Sem_wait(sync_object_t* handle, uint32_t timeout)      
{
    fsl_rtos_status status = kIdle;
#if (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    if (0==timeout)
    {
        status = sync_poll(handle);
    }
    else
    {
#else
    if (0==timeout)
    {
        timeout = kSyncWaitForever;
    }
#endif
    do {
        status = sync_wait((sync_object_t*)handle, timeout);
    } while (kIdle == status);
#if (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    }
#endif

    switch (status)
    {
        case kSuccess:
            return OS_SEM_OK;
        case kTimeout:
            return OS_SEM_TIMEOUT;
        default:
            return OS_SEM_ERROR;
    }
}

/* Mutex */
uint32_t OS_Mutex_lock(lock_object_t* handle)     
{
    fsl_rtos_status status = kIdle;

#if (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
    status = lock_poll(handle);
#else
    status = lock_wait(handle, kSyncWaitForever);
#endif

    switch (status)
    {
        case kSuccess:
            return OS_MUTEX_OK;
        default:
            return OS_MUTEX_ERROR;
    }
}

uint32_t OS_Mutex_unlock(lock_object_t* handle)      
{
    return ((kSuccess==lock_release((lock_object_t*)(handle))) ? OS_MUTEX_OK : OS_MUTEX_ERROR);
}

/* Message queue */
uint32_t OS_MsgQ_recv(msg_queue_handler_t msgq, void* msg, uint32_t flag, uint32_t timeout)      
{
    fsl_rtos_status status = kIdle;
    do {
        status = msg_queue_get(msgq, &msg,                               
                              (!(OS_MSGQ_RECEIVE_BLOCK_ON_EMPTY & flag)) ?           
                               0u                                        :           
                              ((0==timeout) ? kSyncWaitForever : timeout));
    } while (kIdle == status);

    switch (status)
    {
        case kSuccess:
            return OS_MSGQ_OK;
        case kTimeout:
            return OS_MSGQ_TIMEOUT;
        default:
            return OS_MSGQ_ERROR;
    }
}

uint32_t OS_MsgQ_send(msg_queue_handler_t msgq, void* msg,uint32_t flag)        
{
    return((kSuccess==msg_queue_put((msg_queue_handler_t)(msgq),                           
                              msg                                 
                              )) ? OS_MSGQ_OK : OS_MSGQ_ERROR);
}

uint32_t OS_MsgQ_Is_Empty(msg_queue_handler_t msgq, void* msg)
{
      return (kSuccess==msg_queue_get(msgq, &msg, 0)) ? 0 : 1;
}


