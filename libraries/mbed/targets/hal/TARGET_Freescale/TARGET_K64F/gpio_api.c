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
    return 1;
}

void gpio_init(gpio_t *obj, PinName pin, PinDirection direction) {
    if (direction) {
        obj->pinName = pin;
        obj->out_config.outputLogic = 0;
        obj->out_config.slewRate = kPortSlowSlewRate;
        obj->out_config.driveStrength = kPortLowDriveStrength,
        obj->isOutput = true;
        sdk_gpio_inout_pin_init((const gpio_input_output_pin_t *)obj);
    } else {
        obj->pinName = pin;
        obj->in_config.isPullEnable = true;
        obj->in_config.pullSelect = kPortPullUp;
        obj->in_config.isPassiveFilterEnabled = false;
        obj->in_config.interrupt = kPortIntDisabled;
        obj->isOutput = false;
        sdk_gpio_inout_pin_init((const gpio_input_output_pin_t *)obj);
    }
}

void gpio_mode(gpio_t *obj, PinMode mode) {
    uint32_t instance = obj->pinName >> GPIO_PORT_SHIFT;
    uint32_t pin = obj->pinName & 0xFF;

    /* TODO: in KPSDK driver should be set pull */
    switch (mode) {
        case PullNone:
            obj->in_config.isPullEnable = false;
            obj->in_config.pullSelect = kPortPullDown;
            port_hal_configure_pull(instance, pin, 0);
            port_hal_pull_select(instance, pin, kPortPullDown);
            break;
        case PullDown:
            obj->in_config.isPullEnable = true;
            obj->in_config.pullSelect = kPortPullUp;
            port_hal_configure_pull(instance, pin, 1);
            port_hal_pull_select(instance, pin, kPortPullDown);
            break;
        case PullUp:
            obj->in_config.isPullEnable = true;
            obj->in_config.pullSelect = kPortPullDown;
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
            obj->isOutput = false;
            sdk_gpio_set_pin_direction(obj->pinName, kGpioDigitalInput);
            break;
        case PIN_OUTPUT:
            obj->isOutput = true;
            sdk_gpio_set_pin_direction(obj->pinName, kGpioDigitalOutput);
            break;
    }
}
