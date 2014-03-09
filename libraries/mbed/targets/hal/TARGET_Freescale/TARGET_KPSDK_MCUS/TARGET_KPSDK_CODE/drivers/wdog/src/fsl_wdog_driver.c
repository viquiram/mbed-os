/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_wdog_driver.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/
extern IRQn_Type wdog_irq_ids[HW_WDOG_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 *******************************************************************************/

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_unlock
 * Description   : Unlock watchdog register written
 * This function is used to unlock the WDOG register written because WDOG register
 * will lock automatically after 256 bus clock. Written while the register is
 * locked has no affect.
 *
 *END*********************************************************************/
static void wdog_unlock(void)
{
    interrupt_disable_global();

    wdog_hal_unlock();

    interrupt_enable_global();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_init 
 * Description   : Initialize watchdog
 * This function is used to initialize the WDOG, after called, the WDOG 
 * will run immediately according to the configure.
 *
 *END*********************************************************************/
void wdog_init(const wdog_user_config_t* init_ptr)
{
    assert(init_ptr);

    if (NULL != init_ptr->wdogCallbackFunc)
    {
        interrupt_register_handler(Watchdog_IRQn,init_ptr->wdogCallbackFunc);
        interrupt_enable(wdog_irq_ids[0]);
    }

    wdog_hal_configure_enabled_in_cpu_debug_mode(init_ptr->cpuDebugModeEnable);
    wdog_hal_configure_enabled_in_cpu_stop_mode(init_ptr->cpuStopModeEnable);
    wdog_hal_configure_enabled_in_cpu_wait_mode(init_ptr->cpuWaitModeEnable);
    wdog_hal_set_clock_source(init_ptr->clockSource);
    wdog_hal_configure_interrupt(NULL != init_ptr->wdogCallbackFunc);
    wdog_hal_configure_window_mode(0 != init_ptr->windowValue);
    wdog_hal_configure_register_update(init_ptr->updateRegisterEnable);

    wdog_unlock();
    wdog_hal_set_timeout_value(init_ptr->timeOutValue);
    wdog_hal_set_window_value(init_ptr->windowValue);
    wdog_hal_set_clock_prescaler(init_ptr->clockPrescaler);
    wdog_hal_clear_interrupt_flag();
    wdog_hal_enable();

}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_shutdown 
 * Description   : Shutdown watchdog
 * This function is used to shutdown the WDOG.
 *
 *END*********************************************************************/
void wdog_shutdown(void)
{
    wdog_unlock();
    wdog_hal_disable();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_is_running 
 * Description   : Get watchdog running status
 * This function is used to get the WDOG running status.
 *
 *END*********************************************************************/
bool wdog_is_running(void)
{
    return wdog_hal_is_enabled();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_refresh 
 * Description   : Refresh watchdog.
 * This function is used to feed the WDOG, it will set the WDOG timer count to zero and 
 * should be called before watchdog timer is timeout, otherwise a RESET will assert.
 *
  *END*********************************************************************/
void wdog_refresh(void)
{
    interrupt_disable_global();

    wdog_hal_refresh();

    interrupt_enable_global();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_reset_chip 
 * Description   : Reset chip by watchdog
 * This function is used to reset chip using WDOG. 
 *
 *END*********************************************************************/
void wdog_reset_chip(void)
{
    wdog_hal_reset_chip();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_clear_reset_count 
 * Description   : Clear watchdog reset count
 * This function is used to set the WDOG reset count to zero, the WDOG_RSTCNT
 * register will only clear on Power On Reset or clear by this function.
 *
 *END*********************************************************************/
void wdog_clear_reset_count(void)
{
    wdog_hal_clear_reset_count();
}

/*FUNCTION****************************************************************
 *
 * Function Name : wdog_get_reset_count 
 * Description   : Get chip reset count that reset by watchdog
 * This function is used to get the WDOG_RSTCNT value.
 *
 *END*********************************************************************/
uint32_t wdog_get_reset_count(void)
{
    return wdog_hal_get_reset_count();
}

/*******************************************************************************
 * EOF
 *******************************************************************************/

