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
#ifndef MPS2_I2C_API_H
#define MPS2_I2C_API_H

#include "device.h"

#if DEVICE_I2C

#ifdef __cplusplus
extern "C" {
#endif

int mps2_i2c_ts_init			(void);
void mps2_i2c_aaci_init		(void);


#ifdef __cplusplus
}
#endif

#endif

#endif
