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
#ifndef __FSL_GPIO_HAL_H__
#define __FSL_GPIO_HAL_H__
 
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_gpio_features.h"
#include "fsl_device_registers.h"
 
/*!
 * @addtogroup gpio_hal
 * @{
 */

/*!
 * @file fsl_gpio_hal.h
 *
 * @brief GPIO hardware driver configuration. Use these functions to set GPIO input/output, 
 * set output logic or get input logic. Check gpio header file for instance numbers. Each 
 * gpio instance has 32 pins with number from 0 to 31.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief gpio direction definition.*/
typedef enum _gpio_pin_direction {
    kGpioDigitalInput  = 0, /*!< Set current pin as digital input*/
    kGpioDigitalOutput = 1  /*!< Set current pin as digital output*/
} gpio_pin_direction_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Set individual gpio pin to general input or output.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number. 
 * @param direction  gpio directions. 
 *        - 0: set to input
 *        - 1: set to output
 */
static inline void gpio_hal_set_pin_direction(uint32_t instance, uint32_t pin,
                                              gpio_pin_direction_t direction)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);

    if (kGpioDigitalOutput == direction)
    {
        HW_GPIO_PDDR_WR(instance, ((uint32_t)1 << pin) | HW_GPIO_PDDR_RD(instance));  
    }
    else
    {
        HW_GPIO_PDDR_WR(instance, (~((uint32_t)1 << pin)) & HW_GPIO_PDDR_RD(instance));
    }
}

/*!
 * @brief Set gpio port pins to general input or output.
 *
 * This functions will operate all 32 pins of port.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param direction  gpio directions.
 *        - 0: set to input
 *        - 1: set to output
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline void gpio_hal_set_port_direction(uint32_t instance, uint32_t direction)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    HW_GPIO_PDDR_SET(instance, direction);
}

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get current direction of individual gpio pin.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number. 
 * @return gpio directions.
 *        - 0: corresponding pin is set to input.
 *        - 1: corresponding pin is set to output.
 */
static inline uint32_t gpio_hal_get_pin_direction(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    return (HW_GPIO_PDDR_RD(instance) >> pin) & 1U;
} 

/*!
 * @brief Get gpio port pins direction.
 *
 * This functions will get all 32-pin directions as a 32-bit integer.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @return gpio directions. Each bit represents one pin. For each bit:
 *        - 0: corresponding pin is set to input.
 *        - 1: corresponding pin is set to output.
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline uint32_t gpio_hal_get_port_direction(uint32_t instance)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);    
    return HW_GPIO_PDDR_RD(instance);
} 

/* @} */

/*!
 * @name Output Operation
 * @{
 */

/*!
 * @brief Set output level of individual gpio pin to logic 1 or 0.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number. 
 * @param output  pin output logic level.
 */
void gpio_hal_write_pin_output(uint32_t instance, uint32_t pin, uint32_t output);

/*!
 * @brief Set output level of individual gpio pin to logic 1.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number.
 */
static inline void gpio_hal_set_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_GPIO_PSOR_WR(instance, 1U << pin);
}

/*!
 * @brief Clear output level of individual gpio pin to logic 0.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number.
 */
static inline void gpio_hal_clear_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_GPIO_PCOR_WR(instance, 1U << pin);
}

/*!
 * @brief Reverse current output logic of individual gpio pin.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number. 
 */
static inline void gpio_hal_toggle_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_GPIO_PTOR_WR(instance, 1U << pin);
}

/*!
 * @brief Set output of gpio port to specific logic value.
 *
 * This function will operate all 32 pins of port.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc).  
 * @param portOutput  data to configure gpio output. Each bit represents one pin. For each bit:
 *        - 0: set logic level 0 to pin.
 *        - 1: set logic level 1 to pin.
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline void gpio_hal_write_port_output(uint32_t instance, uint32_t portOutput)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);    
    HW_GPIO_PDOR_WR(instance, portOutput);
}

/* @} */

/*!
 * @name Input Operation
 * @{
 */

/*!
 * @brief Read current input value of individual gpio pin.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @param pin  gpio port pin number. 
 * @return gpio port input value.
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 */
static inline uint32_t gpio_hal_read_pin_input(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    return (HW_GPIO_PDIR_RD(instance) >> pin) & 1U;
}

/*!
 * @brief Read current input value of specific gpio port.
 *
 * This function will get all 32-pin input as a 32-bit integer.
 * 
 * @param instance  gpio instance number(HW_GPIOA, HW_GPIOB, HW_GPIOC, etc). 
 * @return gpio port input data. Each bit represents one pin. For each bit:
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 *         - LSB: pin 0
 *         - MSB: pin 31
 */
static inline uint32_t gpio_hal_read_port_input(uint32_t instance)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);    
    return HW_GPIO_PDIR_RD(instance);
}

/* @} */

/*!
 * @name FGPIO Operation
 *
 * @note FGPIO (Fast GPIO) are only available in few MCUs. FGPIO and GPIO share the same
 *       peripheral but use different registers. FGPIO is closer to the core than regular GPIO,
 *       so it's faster to read and write.
 * @{
 */

#if FSL_FEATURE_GPIO_HAS_FAST_GPIO

/*!
 * @name Output Operation
 * @{
 */

/*!
 * @brief Set output level of individual fgpio pin to logic 1.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @param pin  fgpio port pin number. 
 */
static inline void fgpio_hal_set_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_FGPIO_PSOR_WR(instance, 1U << pin);
}

/*!
 * @brief Clear output level of individual fgpio pin to logic 0.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @param pin  fgpio port pin number. 
 */
static inline void fgpio_hal_clear_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_FGPIO_PCOR_WR(instance, 1U << pin);
}

/*!
 * @brief Reverse current output logic of individual fgpio pin.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @param pin  fgpio port pin number. 
 */
static inline void fgpio_hal_toggle_pin_output(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    HW_FGPIO_PTOR_WR(instance, 1U << pin);
}

/*!
 * @brief Set output of fgpio port to specific logic value. 
 *
 * This function will affect all 32 pins of port.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @param portOutput  data to configure gpio output. Each bit represents one pin. For each bit:
 *        - 0: set logic level 0 to pin.
 *        - 1: set logic level 1 to pin.
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline void fgpio_hal_write_port_output(uint32_t instance, uint32_t portOutput)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);    
    HW_FGPIO_PDOR_WR(instance, portOutput);
}

/* @} */

/*!
 * @name Input Operation
 * @{ 
 */

/*!
 * @brief Get current input value of individual fgpio pin.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @param pin  fgpio port pin number. 
 * @return fgpio port input data.
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 */
static inline uint32_t fgpio_hal_read_pin_input(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);
    assert(pin < 32);
    return (HW_FGPIO_PDIR_RD(instance) >> pin) & 1U;
}

/*!
 * @brief Get current input value of specific fgpio port.
 *
 * This function will get all 32-pin input as a 32-bit integer.
 * 
 * @param instance  gpio instance number(HW_FPTA, HW_FPTB, HW_FPTC, etc). 
 * @return fgpio port input data. Each bit represents one pin. For each bit:
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 *         - LSB: pin 0
 *         - MSB: pin 31
 */
static inline uint32_t fgpio_hal_read_port_input(uint32_t instance)
{
    assert(instance < HW_GPIO_INSTANCE_COUNT);    
    return HW_FGPIO_PDIR_RD(instance);
}

/* @} */

#endif /* FSL_FEATURE_GPIO_HAS_FAST_GPIO*/

#if defined(__cplusplus)
}
#endif
 
/*! @} */
 
#endif /* __FSL_GPIO_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

