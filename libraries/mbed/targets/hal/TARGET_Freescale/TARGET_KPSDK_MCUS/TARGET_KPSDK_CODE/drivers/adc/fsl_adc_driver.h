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

#ifndef __FSL_ADC_DRIVER_H__
#define __FSL_ADC_DRIVER_H__

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "fsl_port_hal.h"
#include "fsl_adc_hal.h"

/*!
 * @addtogroup adc_driver
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Defines the calibration parameter structure.
 *
 * This structure keeps the calibration parameter after executing the
 * auto-calibration or filled by indicated ones.
 */
typedef struct adcCalibrationParam
{
    uint32_t PG; /*!< The value for PG register */
    uint32_t MG; /*!< The value for MG register */
} adc_calibration_param_t;

/*!
 * @brief Defines the ADC basic configuration structure.
 *
 * This structure is used when initializing the ADC device associated with
 * adc_init(). It contains the basic feature configuration which are
 * necessary.
 */
typedef struct AdcUserConfig
{
    adc_clock_source_mode_t clockSourceMode; /*!< Selection of ADC clock source */
    adc_clock_divider_mode_t clockSourceDividerMode; /*!< Selection of ADC clock divider */
    adc_resolution_mode_t resolutionMode; /*!< Selection of ADC resolution */
    adc_reference_voltage_mode_t referenceVoltageMode; /*!< Selection of ref voltage source */
    bool isContinuousEnabled;   /*!< Switcher to enable continuous conversion */
} adc_user_config_t;

/*!
 * @brief Defines the ADC extended configuration structure.
 *
 * This structure is used when initializing the ADC device associated with
 * adc_init_extend(). It contains the advanced feature configuration when
 * necessary.
 */
typedef struct adcExtendConfig
{
    bool isLowPowerEnabled;    /*!< Switcher to enable the low power mode*/
    bool isLongSampleEnabled;  /*!< Switcher to enable the long sample mode*/
    adc_long_sample_mode_t hwLongSampleMode;  /*!< Selection of long sample mode*/
    bool isHighSpeedEnabled;  /*!< Switcher to enable high speed sample mode*/
    bool isAsynClockEnabled;  /*!< Switcher to enable internal asynchronous clock at initialization*/
    bool isHwTriggerEnabled;  /*!< Switcher to enable hardware trigger*/
    bool isHwCompareEnabled;  /*!< Switcher to enable hardware compare*/
    bool isHwCompareGreaterEnabled; /*!< Switcher to enable greater compare*/
    bool isHwCompareRangeEnabled;  /*!< Switcher to enable range compare*/
    uint32_t hwCompareValue1;   /*!< Low limit in hardware compare*/
    uint32_t hwCompareValue2;   /*!< High limit in hardware compare*/
    bool isHwAverageEnabled;    /*!< Switcher to enable hardware average*/
    adc_hw_average_mode_t hwAverageSampleMode;  /*!< Selection of hardware average time*/
    bool isDmaEnabled;  /*! < Switcher to enable DMA support*/
} adc_extend_config_t;

/*!
 * @brief Defines the channel configuration structure.
 *
 * This structure is used when setting the conversion channel associated with
 * adc_start_conversion(), adc_stop_conversion(), adc_is_conversion_completed()
 * and adc_get_conversion_value(). It contains all the information that can
 * identify an ADC channel.
 */
typedef struct adcChannelConfig
{
    /*
     * Corresponding to the ADCH bits of a 5-bit field , channel ID  selects
     * one of the input channels. The ADC  is turned off when the channel
     * select bits are all set as ADCH = 11111(kAdcChannelDisable). See  the
     * type definition of adc_channel_mode_t.
     */
    adc_channel_mode_t channelId; /*!< Channel number*/
    bool isDifferentialEnabled;   /*!< The switcher to enable differential channel*/
    bool isInterruptEnabled;  /*!< The switcher to enable interrupt when conversion is completed*/
    adc_group_mux_mode_t muxSelect; /*!< Selection mux to group A(0) or group B(1)*/
} adc_channel_config_t;

#if defined(__cplusplus)
extern "C" {
#endif
  
/*!
 * @brief Defines the ADC ISR callback function.
 *
 * This type defines the prototype of ADC ISR callback function that can be
 * registered inside the ISR.
 */
typedef void (*adc_isr_callback_t)(void);

/*!
 * @brief Gets the parameters for calibration.
 *
 * This function is used to get the calibration parameters in auto-calibrate mode.
 * Execute this function to obtain the parameter for the
 * calibration during the initialization. This process may be time consuming.
 *
 * @param instance ADC instance ID.
 * @param paramPtr The pointer to a empty calibration parameter structure.
 * @return The execution status.
 */
adc_status_t adc_get_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr);

/*!
 * @brief Sets the parameters for calibration.
 *
 * This function is used to set the calibration parameters. The
 * parameters can be generated from the auto-calibration by the
 * adc_get_calibration_param() or created by manually indicated parameters.
 *
 * @param instance ADC instance ID.
 * @param paramPtr The pointer to a filled calibration parameter structure.
 * @return The execution status.
 */
adc_status_t adc_set_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr);

/*!
 * @brief Executes the auto calibration.
 *
 * This function is used to execute the auto calibration.
 * Recommended configuration has been accepted to fetch calibration parameters
 * for highest accuracy. The calibration offset  is returned to the application
 * for further use. After the auto calibration process, the initialization function
 * should be called explicitly to update the configuration according to the
 * application.
 *
 * @param instance ADC instance ID.
 * @param paramPtr The pointer to an empty calibration parameter structure.
 * It is  filled with the calibration offset value after the function is called.
 * @return The execution status.
 */
adc_status_t adc_auto_calibration(uint32_t instance, adc_calibration_param_t *paramPtr);

/*!
 * @brief Initializes the ADC with the basic configuration.
 *
 * This function ensures that the basic operations of ADC  function correctly.
 * This function should be called when an application does not  require complex features.
 *
 *
 * @param instance ADC instance ID.
 * @param cfgPtr The pointer to basic configuration structure.
 * @return The execution status.
 */
adc_status_t adc_init(uint32_t instance, adc_user_config_t *cfgPtr);

/*!
 * @brief Initializes the ADC with the extended configurations when necessary.
 *
 * This function  provides advanced features according
 * when an application requires  complex configurations. They are:
 * low power mode, long sample mode, high speed mode, asynchronous work mode,
 * hardware trigger, hardware compare, and hardware average.
 *
 * @param instance ADC instance ID.
 * @param extendCfgPtr The pointer to extend configuration structure.
 * @return The execution status.
 */
adc_status_t adc_init_extend(uint32_t instance, adc_extend_config_t *extendCfgPtr);

/*!
 * @brief Shuts down the ADC.
 *
 * Shutting down the ADC  cuts off the clock to the indicated ADC device.
 *
 * @param instance ADC instance ID.
 */
void adc_shutdown(uint32_t instance);

/*!
 * @brief Starts the conversion from the indicated channel.
 *
 * Triggers the indicated channel conversion in a single conversion mode. This
 * function should be called when each time the conversion is triggered. In a
 * continuous conversion mode, this function can be called only once at the
 * beginning of conversion. The ADC  executes the conversion periodically
 * and automatically.
 *
 * @param instance ADC instance ID.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return The execution status.
 */
adc_status_t adc_start_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Stops the conversion.
 *
 * Stops the ADC conversion. This function  sets  ADC to a "NULL"
 * channel, which stops ADC conversion from any channel. It is a different function
 * than the adc_shutdown().
 *
 * @param instance ADC instance ID.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return The execution status.
 */
adc_status_t adc_stop_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Checks whether the conversion is completed.
 *
 * Checks whether the current conversion is completed. Because there are multiple
 * channels sharing the same converter, the status is used to indicate the
 * converter status.
 *
 * @param instance ADC instance ID.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return True if the event is asserted.
 */
bool adc_is_conversion_completed(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Gets the value after the conversion.
 *
 * The value  comes from the value register that may be eventually processed
 * according to the application. When using polling mode, the value is obtained
 * after the conversion is completed. When using the interrupt mode, the value
 * comes from the buffer that is updated by the ADC ISR.
 *
 * @param instance ADC instance ID.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return the value of conversion.
 */
uint32_t adc_get_conversion_value(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Registers the custom callback function of the ADC ISR.
 *
 * Callback provides a friendly API for application to program the ISR. A special
 * function  needs to be executed at the moment conversion is completed and can
 * be inserted to the ISR by calling the function registered  by the user.
 *
 * @param instance ADC instance ID.
 * @param func The pointer to user indicating callback function.
 */
void adc_register_user_callback_isr(uint32_t instance, adc_isr_callback_t func);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_ADC_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
