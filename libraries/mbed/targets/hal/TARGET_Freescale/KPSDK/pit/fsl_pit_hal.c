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
 
#include "fsl_pit_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : pit_hal_configure_timer_run_in_debug
 * Description   : Configure timers continue to run or stop in debug mode.
 * In debug mode, the timers will be frozen or not based on the setting of this
 * function. This is intended to aid software development, allowing the developer
 * to halt the processor, investigate the current state of the system (for example,
 * the timer values) and then continue the operation.

 *END**************************************************************************/
void pit_hal_configure_timer_run_in_debug(bool timerRun)
{
    if (!timerRun)
    {
        /* Timers are stopped in debug mode.*/
        PIT->MCR |= PIT_MCR_FRZ_MASK;
    }
    else
    {
        /* Timers continue to run.*/
        PIT->MCR &= (~PIT_MCR_FRZ_MASK);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pit_hal_configure_interrupt
 * Description   : Enable or disable timer interrupt.
 * If enabled, an interrupt will happen when there is a timeout event occurred
 * (Note: NVIC should be called to enable pit interrupt in system level).
 *
 *END**************************************************************************/
void pit_hal_configure_interrupt(uint32_t timer, bool enable)
{
    assert(timer < FSL_FEATURE_PIT_TIMER_COUNT);
    if (!enable)
    {
        /* Disable interrupt.*/
        PIT->CHANNEL[timer].TCTRL &= (~PIT_TCTRL_TIE_MASK);
    }
    else
    {
        /* Generate interrupt when timer counts to 0.*/
        PIT->CHANNEL[timer].TCTRL |= PIT_TCTRL_TIE_MASK;
    }
}

#if FSL_FEATURE_PIT_HAS_CHAIN_MODE
/*FUNCTION**********************************************************************
 *
 * Function Name : pit_hal_configure_timer_chain
 * Description   : Enable or disable chain with previous timer.
 * When a timer has chain mode enabled, it will only count after the previous
 * timer has expired. So if timer n-1 has counted down to 0, counter n will 
 * decrement the value by one. This allows to chain some of the timers together
 * to form a longer timer. The first timer (timer 0) cannot be chained to any
 * other timer.
 *
 *END**************************************************************************/
void pit_hal_configure_timer_chain(uint32_t timer, bool enable)
{
    assert(timer < FSL_FEATURE_PIT_TIMER_COUNT);
    if (!enable)
    {
        /* Doesn't chain timer with previous timer.*/
        PIT->CHANNEL[timer].TCTRL &= (~PIT_TCTRL_CHN_MASK);
    }
    else
    {
        /* Chained current timer with previous timer.*/
        PIT->CHANNEL[timer].TCTRL |= PIT_TCTRL_CHN_MASK;
    }
}
#endif /* FSL_FEATURE_PIT_HAS_CHAIN_MODE */

#if FSL_FEATURE_PIT_HAS_LIFETIME_TIMER
/*FUNCTION**********************************************************************
 *
 * Function Name : pit_hal_read_lifetime_timer_count
 * Description   : Read current lifefime counter value.
 * Lifetime timer is 64-bit timer which chains timer 0 and timer 1 together. 
 * So, timer 0 and 1 should by chained by calling pit_hal_configure_timer_chain
 * before using this timer. The period of lifetime timer equals to "period of
 * timer 0 * period of timer 1". For the 64-bit value, higher 32-bit will have
 * the value of timer 1, and lower 32-bit have the value of timer 0.
*
 *END**************************************************************************/
uint64_t pit_hal_read_lifetime_timer_count(void)
{
    uint32_t valueH = 0U, valueL = 0U;
    
    /* LTMR64H should be read before LTMR64L */
    valueH = PIT->LTMR64H;
    valueL = PIT->LTMR64L;
    return (((uint64_t)valueH << 32U) + (uint64_t)(valueL));
}
#endif /* FSL_FEATURE_PIT_HAS_LIFETIME_TIMER*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

