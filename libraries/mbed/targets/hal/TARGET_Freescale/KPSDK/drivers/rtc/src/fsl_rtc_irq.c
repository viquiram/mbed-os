/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
 
#include "fsl_rtc_driver.h"
#include "fsl_os_abstraction.h"
#include "fsl_interrupt_manager.h"
#include <assert.h>

/*!
 * @addtogroup rtc_irq
 * @{
 */
 
/*******************************************************************************
 * Variables
 ******************************************************************************/

IRQn_Type rtc_irq_ids[FSL_FEATURE_RTC_INTERRUPT_COUNT] = 
{
    RTC_IRQn, RTC_Seconds_IRQn
};

/*!
 * @brief Function table to save RTC isr callback function pointers.
 *
 * Call rtc_register_isr_callback_function to install isr callback functions.
 */
rtc_isr_callback_t rtc_isr_callback_table[FSL_FEATURE_RTC_INTERRUPT_COUNT] = {NULL};
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Implementation of RTC handler named in startup code. */
/*!*/
/*! Passes instance to generic RTC IRQ handler.*/
void RTC_IRQHandler(void);

/*! @brief Implementation of RTC handler named in startup code. */
/*!*/
/*! Passes instance to generic RTC IRQ handler.*/
void RTC_Seconds_IRQHandler(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! @brief RTC IRQ handler with the same name in startup code*/
/*! @brief Implementation of RTC handler named in startup code. */
/*!*/
/*! Passes instance to generic RTC IRQ handler.*/
void RTC_IRQHandler(void)
{
    /* Run callback function if it exists.*/
    if (rtc_isr_callback_table[0])
    {
        (*rtc_isr_callback_table[0])();
    }
}

/*! @brief Implementation of RTC handler named in startup code. */
/*!*/
/*! Passes instance to generic RTC IRQ handler.*/
void RTC_Seconds_IRQHandler(void)
{
    /* Run callback function if it exists.*/
    if (rtc_isr_callback_table[1])
    {
        (*rtc_isr_callback_table[1])();
    }
}

/*! @} */

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_register_isr_callback_function 
 * Description   : Register rtc isr callback function. 
 * System default ISR interfaces are already defined in fsl_rtc_irq.c. Users 
 * can either edit these ISRs or use this function to register a callback
 * function. The default ISR will run the callback function it there is one
 * installed here.

 *END**************************************************************************/
void rtc_register_isr_callback_function(uint8_t rtc_irq_number, rtc_isr_callback_t function)
{
    assert(rtc_irq_number < FSL_FEATURE_RTC_INTERRUPT_COUNT);
    assert(function != NULL);
    
    rtc_isr_callback_table[rtc_irq_number] = function;
    /* Enable RTC interrupt.*/
    interrupt_enable(rtc_irq_ids[rtc_irq_number]);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

