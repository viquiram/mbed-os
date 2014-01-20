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
    software without specific prior written permission.
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

#include "fsl_gpio_driver.h"
#include "fsl_clock_manager.h"
#include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
uint32_t gpioPinCount;
extern IRQn_Type gpio_irq_ids[HW_PORT_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_init
 * Description   : Initialize all GPIO pins used by board.
 * To initialize the GPIO driver, two arrays similar with gpio_input_pin_t
 * inputPin[] and gpio_output_pin_t outputPin[] should be defined in user's file.
 * Then simply call gpio_init() and pass into these two arrays.
 *
 *END**************************************************************************/
void sdk_gpio_init(const gpio_input_pin_t * inputPins, const gpio_output_pin_t * outputPins)
{
    if (inputPins)
    {
        /* Initialize input pins.*/
        while (inputPins->pinName != GPIO_PINS_OUT_OF_RANGE)
        {
            sdk_gpio_input_pin_init(inputPins);
            inputPins++;
        }
    }

    if (outputPins)
    {
        /* Initialize output pins.*/
        while (outputPins->pinName != GPIO_PINS_OUT_OF_RANGE)
        {
            sdk_gpio_output_pin_init(outputPins);
            outputPins++;
        }
    }
}

static void sdk_gpio_input_pin_config(const gpio_input_pin_config_t *inputConfig,
                                      uint32_t instance, uint32_t pin)
{
    /* Un-gate port clock*/
    clock_manager_set_gate(kClockModulePORT, instance, true);

    /* Set current pin as digital input.*/
    gpio_hal_set_pin_direction(instance, pin, kGpioDigitalInput);

    /* Configure gpio input features. */
    port_hal_configure_pull(instance, pin, inputConfig->isPullEnable);
    port_hal_pull_select(instance, pin, inputConfig->pullSelect);
    port_hal_configure_passive_filter(instance, pin,
            inputConfig->isPassiveFilterEnabled);
#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
    port_hal_configure_digital_filter(instance, pin,
            inputConfig->isDigitalFilterEnabled);
#endif
    port_hal_configure_pin_interrupt(instance, pin, inputConfig->interrupt);

    /* Configure NVIC */
    if ((inputConfig->interrupt) && (gpio_irq_ids[instance]))
    {
        /* Enable GPIO interrupt.*/
        NVIC_EnableIRQ(gpio_irq_ids[instance]);
    }
    port_hal_mux_control(instance, pin, kPortMuxAsGpio);
}

void sdk_gpio_input_pin_init(const gpio_input_pin_t *inputPin)
{
    /* Get actual port and pin number.*/
    uint32_t gpioInstance = inputPin->pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = inputPin->pinName & 0xFF;

    sdk_gpio_input_pin_config(&inputPin->config, gpioInstance, pin);
}

static void sdk_gpio_output_pin_config(const gpio_output_pin_config_t *outputConfig,
                                       uint32_t instance, uint32_t pin)
{
    /* Un-gate port clock*/
    clock_manager_set_gate(kClockModulePORT, instance, true);

    /* Set current pin as digital output.*/
    gpio_hal_set_pin_direction(instance, pin, kGpioDigitalOutput);

    /* Configure gpio output features. */
    gpio_hal_write_pin_output(instance, pin, outputConfig->outputLogic);
    port_hal_configure_slew_rate(instance, pin, outputConfig->slewRate);
    port_hal_configure_drive_strength(instance, pin, outputConfig->driveStrength);
#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
    port_hal_configure_open_drain(instance, pin, outputConfig->isOpenDrainEnabled);
#endif
    port_hal_mux_control(instance, pin, kPortMuxAsGpio);
}

void sdk_gpio_output_pin_init(const gpio_output_pin_t *outputPin)
{
    /* Get actual port and pin number.*/
    uint32_t gpioInstance = outputPin->pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = outputPin->pinName & 0xFF;

    sdk_gpio_output_pin_config(&outputPin->config, gpioInstance, pin);
}

void sdk_gpio_inout_pin_init(const gpio_input_output_pin_t *inoutPin)
{
    /* Get actual port and pin number.*/
    uint32_t gpioInstance = inoutPin->pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = inoutPin->pinName & 0xFF;

    if (inoutPin->isOutput)
    {
        sdk_gpio_output_pin_config(&inoutPin->out_config, gpioInstance, pin);
    }
    else
    {
        sdk_gpio_input_pin_config(&inoutPin->in_config, gpioInstance, pin);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_get_pin_direction
 * Description   : Get current direction of individual gpio pin.
 *
 *END**************************************************************************/
uint32_t sdk_gpio_get_pin_direction(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    return gpio_hal_get_pin_direction(gpioInstance, pin);
}

void sdk_gpio_set_pin_direction(uint32_t pinName, gpio_pin_direction_t direction)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    gpio_hal_set_pin_direction(gpioInstance, pin, direction);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_write_pin_output
 * Description   : Set output level of individual gpio pin to logic 1 or 0.
 *
 *END**************************************************************************/
void sdk_gpio_write_pin_output(uint32_t pinName, uint32_t output)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    gpio_hal_write_pin_output(gpioInstance, pin, output);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_set_pin_output
 * Description   : Set output level of individual gpio pin to logic 1.
 *
 *END**************************************************************************/
void sdk_gpio_set_pin_output(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    gpio_hal_set_pin_output(gpioInstance, pin);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_clear_pin_output
 * Description   : Set output level of individual gpio pin to logic 0.
 *
 *END**************************************************************************/
void sdk_gpio_clear_pin_output(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    gpio_hal_clear_pin_output(gpioInstance, pin);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_toggle_pin_output
 * Description   : Reverse current output logic of individual gpio pin.
 *
 *END**************************************************************************/
void sdk_gpio_toggle_pin_output(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    gpio_hal_toggle_pin_output(gpioInstance, pin);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_read_pin_input
 * Description   : Read current input value of individual gpio pin.
 *
 *END**************************************************************************/
uint32_t sdk_gpio_read_pin_input(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    return gpio_hal_read_pin_input(gpioInstance, pin);
}

#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_configure_digital_filter
 * Description   : Enable or disable digital filter in one single port.
 *
 *END**************************************************************************/
void sdk_gpio_configure_digital_filter(uint32_t pinName, bool isDigitalFilterEnabled)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    port_hal_configure_digital_filter(gpioInstance, pin, isDigitalFilterEnabled);
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : gpio_clear_pin_interrupt_flag
 * Description   : Clear individual gpio pin interrupt status flag.
 *
 *END**************************************************************************/
void sdk_gpio_clear_pin_interrupt_flag(uint32_t pinName)
{
    uint32_t gpioInstance = pinName  >> GPIO_PORT_SHIFT;
    uint32_t pin = pinName & 0xFF;

    port_hal_clear_pin_interrupt_flag(gpioInstance, pin);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

