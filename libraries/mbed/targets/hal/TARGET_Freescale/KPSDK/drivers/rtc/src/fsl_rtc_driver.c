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

#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"

/*!
 * @addtogroup rtc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

static void convert_seconds_to_datetime(const uint32_t * seconds,
  rtc_datetime_t * datetime);

static void convert_datetime_to_seconds(const rtc_datetime_t * datetime,
  uint32_t * seconds);

static bool has_datetime_correct_format(const rtc_datetime_t * datetime);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of month length (in days) for the Un-leap-year*/
static const uint8_t ULY[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U,
    31U,30U,31U};

/* Table of month length (in days) for the Leap-year*/
static const uint8_t  LY[] = {0U, 31U, 29U, 31U, 30U, 31U, 30U, 31U, 31U, 30U,
    31U,30U,31U};

/* Number of days from begin of the Leap-year*/
static const uint16_t MONTH_DAYS[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U,
    212U,243U,273U,304U,334U};

extern IRQn_Type rtc_irq_ids[FSL_FEATURE_RTC_INTERRUPT_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_init
 * Description   : initializes the Real Time Clock module
 * This function will initialize the Real Time Clock module, install ISR for
 * general interrupt and seconds interrupt as needed.
 *
 *END**************************************************************************/
bool rtc_init(const rtc_init_config_t * config)
{
    bool result = false;

    /* Protect against null pointers*/
    if(NULL == config)
    {
        /* assuming result variable was initialized to false.*/
        return result;
    }

    /* Enable clock gate to RTC module */
    clock_manager_set_gate(kClockModuleRTC, 0U, true);

    if(config->general_config)
    {
        /* Initialize the general configuration for RTC module.*/
        rtc_hal_init(config->general_config);
    }

    result = true;

    if(config->start_at_datetime)
    {
        /* will return false if datetime format is invalid or the counter was*/
        /* already started.*/
        result = rtc_set_datetime(config->start_at_datetime, config->start_counter);
    }
  
    return result;  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_shutdown 
 * Description   : Disable RTC module clock gate control
 * This function will disable clock gate to RTC module.
 * 
 *END**************************************************************************/
void rtc_shutdown(void)
{
    /* Disable clock gate to RTC module */
    clock_manager_set_gate(kClockModuleRTC, 0U, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_configure_int
 * Description   : installs the each second ISR for the RTC senconds interuupt.
 * This function will install the each second ISR for the RTC senconds interuupt
 * and enable the interrupt.
 *
 *END**************************************************************************/
void rtc_configure_int(hw_rtc_ier_t * bitfields)
{
  /* Protect against null pointers*/
  if(NULL == bitfields)
  {
    return;
  }

  /* Enable RTC interrupts*/
  rtc_hal_config_interrupts(bitfields);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_get_int_status
 * Description   : returns the RTC interrupt's status flags.
 * This function will return the RTC interrupt's status flags.
 *
 *END**************************************************************************/
void rtc_get_int_status(hw_rtc_sr_t * int_status_flags)
{
    /* Protect against null pointers*/
    if(NULL == int_status_flags)
    {
        return;
    }

    int_status_flags->U = HW_RTC_SR_RD;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_set_datetime
 * Description   : sets the RTC date and time according to the given time struct.
 * This function will set the RTC date and time according to the given time
 * struct, if start_after_set is true, the RTC oscillator will be enabled and
 * the counter will start.
 *
 *END**************************************************************************/
bool rtc_set_datetime(const rtc_datetime_t * datetime, bool start_after_set)
{
    bool result = false;

    /* Protect against null pointers*/
    if(NULL == datetime)
    {
        return result;
    }

    uint32_t seconds;

    result = has_datetime_correct_format(datetime);

    if(result)
    {
        convert_datetime_to_seconds(datetime, &seconds);
        /* Disable counter*/
        rtc_hal_counter_enable(false);

        /* Create scope for prescale variable*/
        { 
            uint16_t prescale = 0;
            /* clear prescaler*/
            result = rtc_hal_set_prescaler(&prescale);
        }

        if(result)
        {
            /* Set seconds counter*/
            result = rtc_hal_set_seconds(&seconds);
        }

        if(result && start_after_set)
        {
            /* Enable RTC oscillator since it is required to start the counter*/
            rtc_hal_config_oscillator(true);

            /* jgsp: After enabling the oscillator, wait the oscillator startup */
            /* time before setting SR[TCE] to allow time for the oscillator clock*/
            /* output to stabilize. */
            /* Start counter*/
            rtc_hal_counter_enable(true);
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_get_datetime
 * Description   : gets the actual RTC time and stores it in the given time struct.
 * This function will get the actual RTC time and stores it in the given time
 * struct.
 *
 *END**************************************************************************/
void rtc_get_datetime(rtc_datetime_t * datetime)
{
    /* Protect against null pointers*/
    if(NULL == datetime)
    {
        return;
    }

    uint32_t seconds = 0;

    rtc_hal_get_seconds(&seconds);

    convert_seconds_to_datetime(&seconds, datetime);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_get_alarm
 * Description   : returns the RTC alarm time.
 * This function will first get alarm time in seconds, then convert the seconds to
 * date time.
 *
 *END**************************************************************************/
bool rtc_get_alarm(rtc_datetime_t * date)
{
    bool result = false;

    /* Protect against null pointers*/
    if(NULL == date)
    {
        return result;
    } 

    uint32_t seconds = 0;

    /* Get alarm in seconds  */
    rtc_hal_get_alarm(&seconds);

    convert_seconds_to_datetime(&seconds, date);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_set_alarm
 * Description   : sets the RTC alarm.
 * This function will first check if the date time has correct format. If yes,
 * convert the date time to seconds, and set the alarm in seconds.
 *
 *END**************************************************************************/
bool rtc_set_alarm(const rtc_datetime_t * date)
{
    bool result;
    uint32_t seconds;

    result = has_datetime_correct_format(date);

    if(result)
    {
        convert_datetime_to_seconds(date, &seconds);
        /* set alarm in seconds*/
        rtc_hal_set_alarm(&seconds);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_start_time_counter
 * Description   : enables the RTC oscillator and starts time counter.
 * This function will first enables the RTC oscillator. After enabling the
 * oscillator, wait the oscillator startup time to allow time for the oscillator
 * clock output to stabilize, then enable time counter.
 *
 *END**************************************************************************/
void rtc_start_time_counter(void)
{
    /* Enable RTC oscillator since it is required to start the counter*/
    rtc_hal_config_oscillator(true);

    /* jgsp: After enabling the oscillator, wait the oscillator startup*/
    /* time before setting SR[TCE] to allow time for the oscillator clock*/
    /* output to stabilize. */
    /* Enable counter*/
    rtc_hal_counter_enable(true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_stop_time_counter
 * Description   : stops running time counter.
 * This function will stop running time counter.
 *
 *END**************************************************************************/
void rtc_stop_time_counter(void)
{
    /* Disable counter*/
    rtc_hal_counter_enable(false);
}

#if FSL_FEATURE_RTC_HAS_MONOTONIC
/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_increment_monotonic
 * Description   : increments monotonic counter by one.
 * This function will increment monotonic counter by one.
 *
 *END**************************************************************************/
bool rtc_increment_monotonic(void)
{
    return rtc_hal_monotonic_counter_increment();
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_cp_datetime_time
 * Description   : copys time data from datetime structure to a time structure.
 * This function will copy time data from datetime structure to a time structure.
 *
 *END**************************************************************************/
void rtc_cp_datetime_time(const rtc_datetime_t * datetime, rtc_time_t * time)
{
    /* Protect against null pointers*/
    if((NULL == datetime) || (NULL == time))
    {
        return;
    }

    time->hour = datetime->hour;
    time->minute = datetime->minute;
    time->second = datetime->second;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : rtc_cp_time_datetime
 * Description   : copys time data from time structure to a datetime structure.
 * This function will copy time data from time structure to a datetime structure.
 *
 *END**************************************************************************/
void rtc_cp_time_datetime(const rtc_time_t * time, rtc_datetime_t * datetime)
{
    /* Protect against null pointers*/
    if((NULL == time) || (NULL == datetime))
    {
        return;
    }

    datetime->hour = time->hour;
    datetime->minute = time->minute;
    datetime->second = time->second;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : convert_seconds_to_datetime
 * Description   : converts time data from seconds to a datetime structure.
 * This function will convert time data from seconds to a datetime structure.
 *
 *END**************************************************************************/
static void convert_seconds_to_datetime(const uint32_t * seconds,
  rtc_datetime_t * datetime)
{
    uint32_t x;
    uint32_t Seconds, Days;

    /* Start from 1970-01-01*/
    Seconds = *seconds;
    /* days*/
    Days = Seconds / 86400U;
    /* seconds left*/
    Seconds = Seconds % 86400U;
    /* hours*/
    datetime->hour = Seconds / 3600U;
    /* seconds left*/
    Seconds = Seconds % 3600u;
    /* minutes*/
    datetime->minute = Seconds / 60U;
    /* seconds*/
    datetime->second = Seconds % 60U;
    /* year*/
    datetime->year = (4U * (Days / ((4U * 365U) + 1U))) + 1970;
    /* Days left*/
    Days = Days % ((4U * 365U) + 1U);
    /* 59*/
    if (Days == ((0U * 365U) + 59U))
    {
        datetime->day = 29U;
        datetime->month = 2U;
        return;
    }
    else if (Days > ((0U * 365U) + 59U))
    {
        Days--;
    }
    else { }

    x =  Days / 365U;
    datetime->year += x;
    Days -= x * 365U;
    for (x=1U; x <= 12U; x++)
    {
        if (Days < ULY[x])
        {
            datetime->month = x;
            break;
        }
        else
        {
            Days -= ULY[x];
        }
    }

    datetime->day = Days + 1U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : has_datetime_correct_format
 * Description   : checks if the datetime is in correct format.
 * This function will check if the given datetime is in the correct format.
 *
 *END**************************************************************************/
static bool has_datetime_correct_format(const rtc_datetime_t * datetime)
{
    bool result = false;

    /* Test correctness of given parameters*/
    if ((datetime->year < 1970U) || (datetime->year > 2038U) || (datetime->month > 12U) ||
      (datetime->month == 0U) || (datetime->day > 31U) || (datetime->day == 0U))
    {
        /* If not correct then error*/
        result = false;
    }
    else
    {
        result = true;
    }

    /* Is given year un-leap-one?*/
    if ( result && (datetime->year & 3U))
    {
        /* Does the obtained number of days exceed number of days in the appropriate month & year?*/
        if (ULY[datetime->month] < datetime->day)
        {
            /* If yes (incorrect datetime inserted) then error*/
            result = false;
        }
    }
    else /* Is given year leap-one?*/
    {
        /* Does the obtained number of days exceed number of days in the appropriate month & year?*/
        if (result && (LY[datetime->month] < datetime->day))
        {
            /* if yes (incorrect date inserted) then error*/
            result = false;
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : convert_datetime_to_seconds
 * Description   : converts time data from datetime to seconds.
 * This function will convert time data from datetime to seconds.
 *
 *END**************************************************************************/
static void convert_datetime_to_seconds(const rtc_datetime_t * datetime,
  uint32_t * seconds)
{
    /* Compute number of days from 1970 till given year*/
    *seconds = ((datetime->year - 1970U) * 365U) + (((datetime->year - 1970U) + 3U) / 4U);
    /* Add number of days till given month*/
    *seconds += MONTH_DAYS[datetime->month];
    /* Add days in given month*/
    *seconds += datetime->day;
    /* For un-leap year or month <= 2, decrement day counter*/
    if ((datetime->year & 3U) || (datetime->month <= 2U))
    {
        (*seconds)--;
    }

    *seconds = ((*seconds) * 86400U) + (datetime->hour * 3600U) + (datetime->minute * 60U) +
               datetime->second;
    (*seconds)++;
}

/*! @}*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

