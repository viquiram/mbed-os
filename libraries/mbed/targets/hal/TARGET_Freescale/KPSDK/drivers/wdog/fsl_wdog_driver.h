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
#ifndef __FSL_WDOG_DRIVER_H__
#define __FSL_WDOG_DRIVER_H__

#include "fsl_wdog_hal.h"
#include "fsl_device_registers.h"
#include "fsl_interrupt_manager.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/*! 
 * @addtogroup wdog_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! 
 * @brief Data struct for watchdog initialize
 *
 * This structure is used when initialize the WDOG while wdog_init function is
 * called. It contains all configure of the WDOG.
 */
typedef struct wdogInit {
    wdog_clock_source_t clockSource; /*!< Clock source select*/
    bool updateRegisterEnable; /*!< Update write-once register enable*/
    bool cpuDebugModeEnable; /*!< Enable watchdog ini cpu debug mode*/
    bool cpuWaitModeEnable; /*!< Enable watchdog ini cpu wait mode*/
    bool cpuStopModeEnable; /*!< Enable watchdog ini cpu stop mode*/
    uint32_t windowValue; /*!< Window value*/
    uint32_t timeOutValue; /*!< Timeout value*/
    wdog_clock_prescaler_t clockPrescaler; /*!< Clock prescaler*/
    wdog_isr_callback_t wdogCallbackFunc; /*!< Isr callback function. must in 256 bus cycles.*/
} wdog_init_t;

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! 
 * @name Watchdog Driver
 * @{
 */


/*!
 * @brief Initialize watchdog.
 *
 * This function is used to initialize the WDOG, after called, the WDOG 
 * will run immediately according to the configure.
 *
 * @param init_ptr Watchdog Initialize data structure.
 *
 */
void wdog_init(wdog_init_t* init_ptr);

/*!
 * @brief Shutdown watchdog.
 *
 * This function is used to shotdown the WDOG.
 *
 */
void wdog_shutdown(void);

/*!
 * @brief Refresh watchdog.
 *
 *  This function is used to feed the WDOG, it will set the WDOG timer count to zero and 
 *  should be called before watchdog timer is timeout, otherwise a RESET will assert.
 *
 */
void wdog_refresh(void);

/*!
 * @brief Clear watchdog reset count.
 *
 * This function is used to set the WDOG reset count to zero, the WDOG_RSTCNT
 * register will only clear on Power On Reset or clear by this function.
 *
 */
void wdog_clear_reset_count(void);

/*!
 * @brief Get watchdog running status.
 *
 * This function is used to get the WDOG running status.
 *
 * @return watchdog running status, 0 means not running, 1 means running
 */
bool wdog_is_running(void);

/*!
 * @brief Reset chip by watchdog.
 *
 * This function is used to reset chip using WDOG. 
 *
 */
void wdog_reset_chip(void);

/*!
 * @brief Get chip reset count that reset by watchdog.
 *
 * This function is used to get the WDOG_RSTCNT value.
 *
 * @return Chip reset count that reset by watchdog.
 */
uint32_t wdog_get_reset_count(void);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_WDOG_H__*/
/*******************************************************************************
 * EOF
 *******************************************************************************/

