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
#if !defined(__FSL_RTC_DRIVER_H__)
#define __FSL_RTC_DRIVER_H__

#include <stdint.h>
#include "fsl_rtc_hal.h"

/*!
 * @addtogroup rtc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief Number of interrupts.*/
#define FSL_FEATURE_RTC_INTERRUPT_COUNT (2)

/*!
 * @brief Structure is used to hold the time in a simple "date" format.
 * @warning If you violate the ranges, undefined behavior results.
 */
typedef struct RtcDatetimeStruct
{
   uint16_t year;    /*!< Range from 200 to 2099.*/
   uint16_t month;   /*!< Range from 1 to 12.*/
   uint16_t day;     /*!< Range from 1 to 31 (depending on month).*/
   uint16_t hour;    /*!< Range from 0 to 23.*/
   uint16_t minute;  /*!< Range from 0 to 59.*/
   uint8_t second;   /*!< Range from 0 to 59.*/

} rtc_datetime_t;

/*! @brief Time representation: hours, minutes, second and total seconds*/
typedef struct RtcTimeStruct
{
   uint16_t hour;    /*!< Range from 0 to 23.*/
   uint16_t minute;  /*!< Range from 0 to 59.*/
   uint8_t second;   /*!< Range from 0 to 59.*/

} rtc_time_t;

/*!
 * @brief RTC timer configuration structure
 */
typedef struct RtcUserConfig
{
  rtc_hal_init_config_t * general_config; /*!< Pointer to a structure most of the configurations*/
                                          /*!  are found. Set to NULL to skip related configuration.*/
                                          /*!  See the 'rtc_hal_init_config' definition for details.*/
  rtc_datetime_t * start_at_datetime;     /*!< Initial datetime. Set to NULL to skip.*/
  bool start_counter;                     /*!< Set to true to start the real time counter. Will*/
                                          /*!  Ignored if pointer to start_datetime is NULL.*/
} rtc_user_config_t;

/*! @brief RTC ISR callback function typedef */
typedef void (*rtc_isr_callback_t)(void);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialize and Shutdown
 * @{
 */

/*!
 * @brief      Initializes the Real Time Clock module.
 * @param      Config [in] initialization configuration details.
 * @return     True: success; false: datetime format is invalid or the counter
 *             indicated to start but it was already started.
 */
bool rtc_init(const rtc_user_config_t * config);

/*! @brief      disable RTC module clock gate control. */
void rtc_shutdown(void);
/* @} */

/*!
 * @name RTC interrupt configuration and status 
 * @{
 */

/*!
 * @brief      Enables/disables RTC interrupts.
 * @param      bitfields [in]  set/clear respective bitfields to enabled/disabled
 *                              interrupts. \n
 *                        [out] resulting interrupt enable state. \n
 *              Valid bitfields: \n
 *   TSIE:    Time Seconds Interrupt Enable \n
 *   TAIE:    Time Alarm Interrupt Enable \n
 *   TOIE:    Time Overflow Interrupt Enable \n
 *   TIIE:    Time Invalid Interrupt Enable \n
 * \n
 * For MCUs that have the Wakeup Pin only: \n
 *   WPON:    Wakeup Pin On (see the corresponding MCU's reference manual)\n
 * \n
 * For MCUs that have the Monotonic Counter only: \n
 *   MOIE:    Monotonic Overflow Interrupt Enable \n
 */
void rtc_configure_int(hw_rtc_ier_t * bitfields);

/*!
 * @brief      Returns the RTC interrupt status flags.
 * @param      int_status_flags [out] pointer to where to store bitfield with
 *             an actual RTC interrupt flags.\n
 *             ONLY valid bitfields: \n
 *                TIF: Time Invalid Flag \n
 *                TOF: Time Overflow Flag \n
 *                TAF: Time Alarm Flag \n
 *                MOF: Monotonic Overflow Flag (Not in all devices) \n
 *             Note: other bitfields are not to be taken in account.
 */
void rtc_get_int_status(hw_rtc_sr_t * int_status_flags);

/* @} */

/*!
 * @name RTC datetime set and get 
 * @{
 */
 
/*! 
 * @brief      Sets the RTC date and time according to the given time structure,
 *             if indicated in the corresponding parameter. After, the time
 *             counter is started.        
 * @param      datetime [in] pointer to a structure where the date and time
 *             details are stored.
 * @param	   start_after_set true: the RTC oscillator will be enabled and
 *             the counter will start.
 *             False: otherwise.
 * @return     True: success; false: error, datetime format incorrect, datetime
 *             format is invalid or unable to set the counter value because it is
 *             already enabled.
 */
bool rtc_set_datetime(const rtc_datetime_t * datetime, bool start_after_set);

/*!
 * @brief      Gets the actual RTC time and stores it in the given time structure.
 * @param      datetime [out] pointer to structure where the date and time details will be
 *             stored.
 */
void rtc_get_datetime(rtc_datetime_t * datetime);

/* @} */

/*!
 * @name RTC alarm set and get 
 * @{
 */
 
/*!
 * @brief      Sets the RTC alarm.
 * @param      date [in] pointer to structure where the alarm date and time
 *             details will be stored.
 * @return     true: success; false: error.
 */
bool rtc_set_alarm(const rtc_datetime_t * date);

/*!
 * @brief      Returns the RTC alarm time.
 * @param      date [out] pointer to structure where the alarm date and time
 *             details will be stored.
 * @return     True: success; false: error.
 */
bool rtc_get_alarm(rtc_datetime_t * date);

/* @} */

/*!
 * @name RTC time counter start and stop 
 * @{
 */
 
/*! @brief      Enables the RTC oscillator and starts time counter.*/
void rtc_start_time_counter(void);

/*! @brief      Halts running time counter.*/
void rtc_stop_time_counter(void);

/* @} */

#if FSL_FEATURE_RTC_HAS_MONOTONIC
/*!
 * @name Increments monotonic counter
 * @{
 */
 
/*!
 * @brief      Increments monotonic counter by one.
 * @return     True: increment successful; False: error invalid time found
 *             because of a tamper source enabled is detected and any write to
 *             the tamper time seconds counter is done.
 */
bool rtc_increment_monotonic(void);
#endif

/*!
 * @name Copy time data 
 * @{
 */
 
/*!
 * @brief      Utility to copy time data from datetime structure to a time
 *             structure.
 * @param      datetime        [in] data origin
 * @param      time            [out] data destination
 */
void rtc_cp_datetime_time(const rtc_datetime_t * datetime, rtc_time_t * time);

/*!
 * @brief      Utility to copy time data from time structure to a datetime.
 *             structure.
 * @param      time            [in] data origin
 * @param      datetime        [out] data destination
 */
void rtc_cp_time_datetime(const rtc_time_t * time, rtc_datetime_t * datetime);

/* @} */

/*!
 * @name ISR Callback Function 
 * @{
 */

/*!
 * @brief Register RTC ISR callback function. 
 *
 * System default ISR interfaces are already defined in the fsl_rtc_irq.c. Users 
 * can either edit these ISRs or use this function to register a callback
 * function. The default ISR runs the callback function if one is 
 * installed.
 *
 * @param rtc_irq_number    RTC interrupt number.
 * @param function Pointer to the RTC ISR callback function.
 */
void rtc_register_isr_callback_function(uint8_t rtc_irq_number, rtc_isr_callback_t function);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_RTC_DRIVER_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

