/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "gpio_api.h"
#include "pinmap.h"
#include "fsl_gpio_driver.h"

uint32_t gpio_set(PinName pin) {
    uint32_t instance = pin >> GPIO_PORT_SHIFT;
    uint32_t pin_num = pin & 0xFF;

    port_hal_mux_control(instance, pin_num, kPortMuxAsGpio);
    return 1 << pin_num;
}

void gpio_init(gpio_t *obj, PinName pin, PinDirection direction) {
    if (pin == NC)
        return;

    if (direction) {
        gpio_output_pin_t output = {0};

        obj->pinName = pin;
        output.pinName = pin;
        output.config.outputLogic = 0;
        output.config.slewRate = kPortSlowSlewRate;
        output.config.driveStrength = kPortLowDriveStrength,
        sdk_gpio_output_pin_init((const gpio_output_pin_t *)&output);
    } else {
        gpio_input_pin_t input = {0};
        obj->pinName = pin;
        input.pinName = pin;
        input.config.isPullEnable = true;
        input.config.pullSelect = kPortPullUp;
        input.config.isPassiveFilterEnabled = false;
        input.config.interrupt = kPortIntDisabled;
        sdk_gpio_input_pin_init((const gpio_input_pin_t *)&input);
    }
}

void gpio_mode(gpio_t *obj, PinMode mode) {
    uint32_t instance = obj->pinName >> GPIO_PORT_SHIFT;
    uint32_t pin = obj->pinName & 0xFF;

    switch (mode) {
        case PullNone:
            port_hal_configure_pull(instance, pin, 0);
            port_hal_pull_select(instance, pin, kPortPullDown);
            break;
        case PullDown:
            port_hal_configure_pull(instance, pin, 1);
            port_hal_pull_select(instance, pin, kPortPullDown);
            break;
        case PullUp:
            port_hal_configure_pull(instance, pin, 1);
            port_hal_pull_select(instance, pin, kPortPullUp);
            break;
        default:
            break;
    }
}

void gpio_dir(gpio_t *obj, PinDirection direction) {
    switch (direction) {
        case PIN_INPUT:
            sdk_gpio_set_pin_direction(obj->pinName, kGpioDigitalInput);
            break;
        case PIN_OUTPUT:
            sdk_gpio_set_pin_direction(obj->pinName, kGpioDigitalOutput);
            break;
    }
}
