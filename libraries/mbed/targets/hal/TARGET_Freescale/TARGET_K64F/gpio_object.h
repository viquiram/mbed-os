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
#ifndef MBED_GPIO_OBJECT_H
#define MBED_GPIO_OBJECT_H

#include "fsl_gpio_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pinName;
} gpio_t;

static inline void gpio_write(gpio_t *obj, int value) {
    sdk_gpio_write_pin_output(obj->pinName, (uint32_t)value);
}

static inline int gpio_read(gpio_t *obj) {
    return (int)sdk_gpio_read_pin_input(obj->pinName);
}

#ifdef __cplusplus
}
#endif

#endif
