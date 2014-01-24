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
 * @brief Define the parameter structure for calibration
 * 
 * This structure is used to keep the calibration parameter after executing the 
 * auto-calibration or filled by indicated ones.
 */
typedef struct adcCalibrationParam
{
    uint32_t PG; /*!< The value for PG register */
    uint32_t MG; /*!< The value for MG register */
} adc_calibration_param_t;   
   
/*! 
 * @brief Define ADC's extend configuration structure
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
    bool isAsynClockEnabled;  /*!< Switcher to enable internal asynchonous clock at initialization*/
    bool isHwTriggerEnabled;  /*!< Switcher to enable hardware trigger*/
    bool isHwCompareEnabled;  /*!< Switcher to enable hardware compare*/
    bool isHwCompareGreaterEnabled; /*!< Switcher to enable greater compare*/
    bool isHwCompareRangeEnabled;  /*!< Switcher to enable range compare*/
    uint32_t hwCompareValue1;   /*!< Low limit in hardware compare*/
    uint32_t hwCompareValue2;   /*!< High limit in hardware compare*/
    bool isHwAverageEnabled;    /*!< Switcher to enable hareware average*/
    adc_hw_average_mode_t hwAverageSampleMode;  /*!< Selection of hardware average time*/
    bool isDmaEnabled;  /*! < Switcher to enable DMA support*/
} adc_extend_config_t;

/*! 
 * @brief Define the channel's configuration structure
 * 
 * This structure is used when setting the convertion channel associated with
 * adc_start_conversion(), adc_stop_conversion(), adc_is_conversion_completed()
 * and adc_get_conversion_value(). It contains all the information that can
 * identify an ADC channel.
 */
typedef struct adcChannelConfig
{
    /* 
     * Corresponding to the ADCH bits of a 5-bit field , chennelId  here selects 
     * one of the input channels. The ADC will be turned off when the channel 
     * select bits are all set as ADCH = 11111(kAdcChannelDisable). See to the 
     * type definition of adc_channel_mode_t.
     */
    adc_channel_mode_t channelId; /*!< Channel number*/
    bool isDifferentialEnabled;   /*!< The switcher to enable differential channel*/
    bool isInterruptEnabled;  /*!< The switcher to enable interrupt when conversion is completed*/
    adc_group_mux_mode_t muxSelect; /*!< Selection mux to group A(0) or group B(1)*/
} adc_channel_config_t;

/*! 
 * @brief Define ADC's basic configuration structure
 * 
 * This structure is used when initializing the ADC device associated with 
 * adc_init(). It contains the basic feature configuration which are 
 * necessary.
 */
typedef struct adcConfig
{
    adc_clock_source_mode_t clockSourceMode; /*!< the selection of ADC clock source, see to adc_clock_source_mode_t's type definition */
    adc_clock_divider_mode_t clockSourceDividerMode; /*!< the selection of ADC clock divider, see to adc_clock_divider_mode_t's type defnition */
    adc_resolution_mode_t resolutionMode; /*!< the selection of ADC resolution, see to adc_resolution_mode_t's type definition */
    adc_reference_voltage_mode_t referenceVoltageMode; /*!< the selection of ref voltage source, see to adc_reference_voltage_mode_t's type definition*/
    bool isContinuousEnabled;   /*!< switcher to enable continuous conversion*/
    adc_calibration_param_t *calibrationParam; /*!< NULL to enable auto-calibration or indicate mannually when initializing */
} adc_config_t;

#if defined(__cplusplus)
extern "C" {
#endif
  
/*! 
 * @brief Define ADC ISR callback function.
 *
 * This type define the phototype of ADC ISR'callback function that can be 
 * registered inside the ISR.
 */
typedef void (*adc_isr_callback_t)(void);

/*!
 * @brief Get the parameters for calibration.
 *
 * This function is to get the calibration parameters in auto-calibrate mode.
 * It is recommended to execute this fucntion to obtain the parameter for 
 * calibration during the initialzation, even though it will take a few time.
 * 
 * @param instance ADC instance id.
 * @param paramPtr The pointer to a empty calibration parameter structure.
 * @return The execution status.
 */
adc_status_t adc_get_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr);

/*!
 * @brief Set the parameters for calibration.
 * 
 * This function is to set the calibration parameters when necessary. The 
 * parameters could be generated from the auto-calibration by 
 * adc_get_calibration_param() or created by mannually indicated ones.
 * 
 * @param instance ADC instance id.
 * @param paramPtr The pointer to a filled calibration parameter structure.
 * @return The execution status.
 */
adc_status_t adc_set_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr);
/*!
 * @brief Initialize the ADC with the basic configuration.
 *
 * This function is the necessary one that can make sure ADC can work at least 
 * in the basic way. If application do not need more complex features, just call
 * this fucntion will be simple.
 * 
 * @param instance ADC instance id.
 * @param cfgPtr The pointer to basic configuration structure.
 * @return The execution status.
 */
adc_status_t adc_init(uint32_t instance, adc_config_t *cfgPtr);

/*!
 * @brief Initialize the ADC with the extend configuration when necessory.
 *
 * This function is an optional one that can provide advanced features according
 * to application when more complex configurations are needed. They are:
 * low power mode, long sample mode, high speed mode, asynchonous's work mode,
 * hardware trigger, hardware compare and hardwrare average.
 *
 * @param instance ADC instance id.
 * @param extendCfgPtr The pointer to extend configuration structure.
 * @return The execution status.
 */
adc_status_t adc_init_extend(uint32_t instance, adc_extend_config_t *extendCfgPtr);

/*!
 * @brief Shutdown the ADC.
 *
 * Shutdown the ADC will cut off the clock to the indicated ADC device.
 *
 * @param instance ADC instance id.
 */
void adc_shutdown(uint32_t instance);

/*!
 * @brief Start the conversion from indicated channel.
 *
 * Trigger the indicated channel's conversion. In single conversion mode, this
 * function should be called every time when triggering the conversion. In
 * continuous conversion mode, this function can be called only once at the 
 * beginning of conversion, and then the ADC will execute conversion periodically
 * and automatically.
 *
 * @param instance ADC instance id.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return The execution status.
 */
adc_status_t adc_start_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Stop the conversion.
 *
 * Stop the ADC's conversion. In fact, this function will set ADC to a "NULL"  
 * channel. And then, the ADC will not convert from any channel. It is different
 * from adc_shutdown() while ADC is still alive.
 *
 * @param instance ADC instance id.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return The execution status.
 */
adc_status_t adc_stop_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Check whether the conversion is completed.
 *
 * Check whether the current conversion is completed. As there are multiple 
 * channels sharing the same conventer, the status is used to indicated the 
 * converter's situation.
 *
 * @param instance ADC instance id.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return True if the event is asserted.
 */
bool adc_is_conversion_completed(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Get the value after the conversion.
 *
 * The value just comes from value register that may be eventually processed 
 * according to application. when use polling mode, the value will be obtain 
 * after the converions is completed. when use interrupt mode, the value will 
 * come from the buffer that is updated by ADC ISR.
 *
 * @param instance ADC instance id.
 * @param channelCfgPtr The pointer to channel configuration structure.
 * @return the value of conversion. 
 */
uint32_t adc_get_conversion_value(uint32_t instance, adc_channel_config_t *channelCfgPtr);

/*!
 * @brief Privated to register the custom callback funcion of ADC ISR.
 *
 * Callback provides a friendly API for application to program ISR, when special
 * function piece need to be executed at the moment conversion is completed, 
 * they can be inserted to ISR by call the registering function by user. 
 *
 * @param instance ADC instance id.
 * @param func The pointer to user indicated callback function.
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
