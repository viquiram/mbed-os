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
* $FileName: osadapter.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains OS adapter layer api header function.
*
*****************************************************************************/

#ifndef _USB_OSADAPTER_H
#define _USB_OSADAPTER_H 1
#include "adapter_cfg.h"
#include "adapter_types.h"

#ifdef __cplusplus
extern "C" {
#endif



#if 0
extern void OS_Lock(void);
extern void OS_Unlock(void);
extern int32_t OS_printf(const char  *fmt_ptr, ...);

extern void * OS_Mem_alloc(uint32_t size);
extern void * OS_Mem_alloc_zero(uint32_t size);
extern void * OS_Mem_alloc_uncached(uint32_t size);
extern void * OS_Mem_alloc_uncached_zero(uint32_t size);
extern uint32_t OS_Mem_free(void*);
extern void OS_Mem_copy(void*, void*, uint32_t);
extern void OS_Mem_zero(void*, uint32_t);

extern OSA_INT_ISR_FPTR OS_install_isr(uint32_t, OSA_INT_ISR_FPTR, void *);
extern uint32_t OS_intr_init(uint32_t, uint32_t, uint32_t, bool);

extern void OS_dcache_invalidate_mlines(void *addr, uint32_t length);
extern void OS_dcache_flush_mlines(void *addr, uint32_t length);

extern void OS_Time_delay(register uint32_t milliseconds);
#endif
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)            /* USB stack running on MQX */
#include "adapter_mqx.h"
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)        /* USB stack running on BM  */
#include "adapter_bm.h"
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)        /* USB stack running on SDK  */
#include "adapter_sdk.h"
#endif

extern uint32_t OS_Task_create(TASK_START pstart, void* task_param, uint32_t pri, uint32_t stack_size, char* task_name, void* opt);
extern uint32_t OS_Task_delete(uint32_t);
extern uint32_t OS_Task_suspend(uint32_t);
extern uint32_t OS_Task_resume(uint32_t);

extern OS_Event_handle OS_Event_create(uint32_t flag);
extern uint32_t OS_Event_destroy(OS_Event_handle event);
extern uint32_t OS_Event_set(OS_Event_handle event, uint32_t bitmask);
extern uint32_t OS_Event_check_bit(OS_Event_handle event, uint32_t bitmask);
//extern uint32_t OS_Event_set_auto_clear(OS_Event_handle event, uint32 bitmask);
extern uint32_t OS_Event_clear(OS_Event_handle event, uint32_t bitmask);
extern uint32_t OS_Event_wait(OS_Event_handle event, uint32_t bitmask, uint32_t flag, uint32_t timeout);
//extern OS_Event_get_value(a,b)      _lwevent_get_value(a,b)
//#define OS_EVENT_WAIT_TIMEOUT            0x01


extern OS_MsgQ_handle OS_MsgQ_create(uint32_t max_msg_number, uint32_t msg_size);
extern uint32_t OS_MsgQ_send(OS_MsgQ_handle, void* msg, uint32_t flag);
extern uint32_t OS_MsgQ_recv(OS_MsgQ_handle, void* msg, uint32_t flag, uint32_t timeout);
extern uint32_t OS_MsgQ_destroy(OS_MsgQ_handle);

extern OS_GPIO_handle OS_Gpio_init(uint32_t, uint32_t, uint32_t);
extern uint32_t OS_Gpio_set_functionality(OS_GPIO_handle handle, uint32_t function);
extern uint32_t OS_Gpio_set_value(OS_GPIO_handle handle, uint32_t value);

extern OS_Mutex_handle OS_Mutex_create(void);
extern uint32_t OS_Mutex_lock(OS_Mutex_handle);
extern uint32_t OS_Mutex_unlock(OS_Mutex_handle);
extern uint32_t OS_Mutex_destroy(OS_Mutex_handle);


extern OS_Sem_handle OS_Sem_create(uint32_t initial_number);
extern uint32_t OS_Sem_post(OS_Sem_handle);
extern uint32_t OS_Sem_wait(OS_Sem_handle sem,uint32_t timeout);
extern uint32_t OS_Sem_destroy(OS_Sem_handle sem);

#ifdef __cplusplus
}
#endif


#endif


