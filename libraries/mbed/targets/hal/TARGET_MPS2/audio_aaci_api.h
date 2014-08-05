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
#ifndef MBED_AACI_API_H
#define MBED_AACI_API_H

#include "device.h"

#if DEVICE_AACI

#include "AAIC_I2S_MPS2.h"
#include "AAIC_I2C_MPS2.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct aaci_s aaci_t;


// Loopback test limits
#define MINSIGNAL          0xC000                                  // Allow for gain errors
#define MAXSIGNAL          0x0200                                  // Wideband noise level
#define RECLENGTH          48000                                   // 1 Second recording (48kHz), see note in AACI_Recording()
#define AACI_TIMEOUT       1000                                    // Timeout for reading FIFOs (10mS)
#define TONETIME           500                                     // Loop back test tone time

// External function types

/*  Function: audio_aaci_init(void)
 *   Purpose: initialisation of audio
 *
 * Arguments: None
 *   Returns:
 */
unsigned char 	audio_aaci_init					(aaci_t *obj);
void 						audio_aaci_out_sine			(void);
int 						audio_aaci_read					(void);
void 						audio_aaci_write				(int data);
void 						audio_aaci_volume				(int volume);
unsigned char 	audio_i2c_read 					(unsigned char reg_addr, unsigned char sadr);
void 						audio_i2c_write					(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);



#ifdef __cplusplus
}
#endif

#endif

#endif
