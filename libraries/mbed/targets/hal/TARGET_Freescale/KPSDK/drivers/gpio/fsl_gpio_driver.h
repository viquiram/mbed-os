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
#ifndef __FSL_GPIO_DRIVER_H__
#define __FSL_GPIO_DRIVER_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"

/*!
 * @addtogroup gpio_driver
 * @{
 */

/*!
 * @file
 *
 * This gpio driver uses virtual gpio name rather than actual port and pin number.
 * By using virtual name, each pin name is self-explanatory.
 * To use this gpio driver, an enum and a struct must be predefined in user's
 * application files. These two variables will save all gpio pin information used
 * in your project.
 *
 * Here is an example demonstrating how to define these variables.
   @code
   // This is the enum to define virtual gpio pin names.
   // These members will be used by "uint32_t pinName" in gpio_output_pin_t
   // and gpio_input_pin_t. Usually defined in a header file.
   enum _gpio_pins
   {
       kGpioLED1  = 0x0, // Orange LED.
       kGpioLED2  = 0x1, // Yellow LED.
       kGpioLED3  = 0x2, // Breen LED.
       kGpioLED4  = 0x3, // Red LED.
   };

   // This is the array which maps the virtual pin names in
   // enum _gpio_pins to actual gpio pins. Must define in a source file.
   const uint8_t gpioPinLookupTable[][2] = {
       {HW_GPIOA, 5},  // kGpioLED1
       {HW_GPIOA, 16}, // kGpioLED2
       {HW_GPIOA, 17}, // kGpioLED3
       {HW_GPIOB, 8},  // kGpioLED4
   };
   @endcode
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief macro to indicate end of pin configuration structure.*/
#define GPIO_PINS_OUT_OF_RANGE (0xFFFFFFFFU)
#define GPIO_PORT_SHIFT 12

/*!
 * @brief gpio pin lookup table defined by user.
 *
 * This array is a n*2 array to save actual port and pin number. Must be declared
 * exactly as "const uint8_t gpioPinLookupTable[][2]" in user's file.
 * Column 1 are port instances (eg, HW_GPIOA), column 2 are pin numbers between
 * 0 to 31 in corresponding port instance.
 */
extern const uint8_t gpioPinLookupTable[][2];

/*!
 * @brief gpio input pin configuration structure.
 *
 * Although every pin is configurable for these features, valid configurations
 * highly depend on specific Soc. Users should check related reference manual to
 * make sure specific feature is valid in individual pin. Configuration of
 * unavailable features is harmless, just will not take effect.
 */
typedef struct gpioInputPinConfig {
    bool isPullEnable;                  /*!< Enable or disable pull. */
    port_pull_t pullSelect;             /*!< Select internal pull(up/down) resistor.*/
    bool isPassiveFilterEnabled;        /*!< Enable or disable passive filter.*/
    #if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
    /* Digital filter clock source and width should be pre-configured using port hal.*/
    bool isDigitalFilterEnabled;        /*!< Enable or disable digital filter.*/
    #endif
    port_interrupt_config_t interrupt;  /*!< Select interrupt/DMA request.*/
} gpio_input_pin_config_t;

/*!
 * @brief gpio output pin configuration structure.
 *
 * Although every pin is configurable for these features, valid configurations
 * highly depend on specific Soc. Users should check related reference manual to
 * make sure specific feature is valid in individual pin. Configuration of
 * unavailable features is harmless, just will not take effect.
 */
typedef struct gpioOutputPinConfig {
    uint32_t outputLogic;               /*!< Set default output logic.*/
    port_slew_rate_t slewRate;          /*! Select fast/slow slew rate.*/
    port_drive_strength_t driveStrength;/*!< Select low/high drive strength.*/
    #if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
    bool isOpenDrainEnabled;            /*!< Enable or disable open drain.*/
    #endif
} gpio_output_pin_config_t;

/*!
 * @brief gpio input pin structure.
 *
 * Although pinName is defined as uint32_t type, values assigned to pinName
 * should be the enum names defined in enum _gpio_pins.
 */
typedef struct gpioInputPin {
    uint32_t pinName;               /*!< Virtual pin name from enum defined by user.*/
    gpio_input_pin_config_t config; /*!< Input pin configuration structure.*/
} gpio_input_pin_t;

/*!
 * @brief gpio output pin structure.
 *
 * Although pinName is defined as uint32_t type, values assigned to pinName
 * should be the enum names defined in enum _gpio_pins.
 */
typedef struct gpioOutputPin {
    uint32_t pinName;               /*!< Virtual pin name from enum defined by user.*/
    gpio_output_pin_config_t config;/*!< Input pin configuration structure.*/
} gpio_output_pin_t;

typedef struct gpioInputOutputPin {
    uint32_t pinName;                    /*!< Virtual pin name from enum defined by user.*/
    gpio_input_pin_config_t in_config;   /*!< Input pin configuration structure.*/
    gpio_output_pin_config_t out_config; /*!< Input pin configuration structure.*/
    bool isOutput;               /*!< Input/Output */
} gpio_input_output_pin_t;

/*! @brief gpio ISR callback function*/
typedef void (*gpio_isr_callback_t)(void);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Init
 * @{
 */

/*!
 * @brief Initialize all GPIO pins used by board.
 *
 * To initialize the GPIO driver, two arrays similar with gpio_input_pin_t
 * inputPin[] and gpio_output_pin_t outputPin[] should be defined in user's file.
 * Then call gpio_init() and pass into these two arrays.
 *
 * Here is an example demonstrating how to define an input pin array:
   @code
   // Configure kGpioPTA2 as digital input.
   gpio_input_pin_t inputPin[] = {
     {
         .pinName = kGpioPTA2,
         .config.isPullEnable = false,
         .config.pullSelect = kPortPullDown,
         .config.isPassiveFilterEnabled = false,
         .config.interrupt = kPortIntDisabled,
     },
     {
        // Note: This pinName must be defined here to indicate end of array.
        .pinName = GPIO_PINS_OUT_OF_RANGE,
     }
   };
   @endcode
 *
 * @param inputPins input gpio pins pointer.
 * @param outputPins output gpio pins pointer.
 */
void sdk_gpio_init(const gpio_input_pin_t * inputPins, const gpio_output_pin_t * outputPins);

void sdk_gpio_input_pin_init(const gpio_input_pin_t *inputPin);

void sdk_gpio_output_pin_init(const gpio_output_pin_t *outputPin);

void sdk_gpio_inout_pin_init(const gpio_input_output_pin_t *pin);

void sdk_gpio_set_pin_direction(uint32_t pinName, gpio_pin_direction_t direction);

/* @} */

/*!
 * @name Pin Direction
 * @{
 */

/*!
 * @brief Get current direction of individual gpio pin.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 * @return gpio directions.
 *         - 0: corresponding pin is set as digital input.
 *         - 1: corresponding pin is set as digital output.
 */
uint32_t sdk_gpio_get_pin_direction(uint32_t pinName);

/* @} */

/*!
 * @name Output Operations
 * @{
 */

/*!
 * @brief Set output level of individual gpio pin to logic 1 or 0.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 * @param output  pin output logic level.
 *        - 0: corresponding pin output low logic level.
 *        - Non-0: corresponding pin output high logic level.
 */
void sdk_gpio_write_pin_output(uint32_t pinName, uint32_t output);

/*!
 * @brief Set output level of individual gpio pin to logic 1.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 */
void sdk_gpio_set_pin_output(uint32_t pinName);

/*!
 * @brief Set output level of individual gpio pin to logic 0.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 */
void sdk_gpio_clear_pin_output(uint32_t pinName);

/*!
 * @brief Reverse current output logic of individual gpio pin.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 */
void sdk_gpio_toggle_pin_output(uint32_t pinName);

/* @} */

/*!
 * @name Input Operations
 * @{
 */

/*!
 * @brief Read current input value of individual gpio pin.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 * @return gpio port input value.
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 */
uint32_t sdk_gpio_read_pin_input(uint32_t pinName);

#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
/*!
 * @brief Enable or disable digital filter in one single port.
 *
 * Each bit of the 32-bit register represents one pin.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 * @param isDigitalFilterEnabled  digital filter enable/disable.
 *        - false: digital filter is disabled on the corresponding pin.
 *        - true : digital filter is enabled on the corresponding pin.
 */
void sdk_gpio_configure_digital_filter(uint32_t pinName, bool isDigitalFilterEnabled);
#endif

/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Clear individual gpio pin interrupt status flag.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 */
void sdk_gpio_clear_pin_interrupt_flag(uint32_t pinName);

/*!
 * @brief Register gpio isr callback function.
 *
 * @param pinName gpio pin name defined by user in gpio pin enum list.
 * @param function Pointer to gpio isr callback function.
 */
void sdk_gpio_register_isr_callback_function(uint32_t pinName, gpio_isr_callback_t function);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_GPIO_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

