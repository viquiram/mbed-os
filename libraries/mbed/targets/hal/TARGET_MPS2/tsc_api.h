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
#ifndef MBED_TSC_API_H
#define MBED_TSC_API_H

#include "device.h"

// External Variables from flyer.c
//extern const unsigned short flyerData[];
// External Variables from intro.c
extern const unsigned short introData[];

#if DEVICE_TSC

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tsc_s tsc_t;
 
// Types
#undef FALSE
#undef TRUE
#define FALSE   0
#define TRUE    1

// TSC I2C controller
#define TI2C_ADDR          0x82
#define TI2C_CID           0x0811

// TSC I2C controller registers
#define TI2C_CRID          0x00

// AACI I2C interface
#define TI2C_CONTROL       ((volatile unsigned int *)0x40022000)
#define TI2C_CONTROLS      ((volatile unsigned int *)0x40022000)
#define TI2C_CONTROLC      ((volatile unsigned int *)0x40022004)

#define SDA                1 << 1
#define SCL                1 << 0

// TSSPCPSR Clock prescale register
#define TSSPCPSR_DFLT      0x0002      // Clock prescale (use with SCR)

// TSC defaults
#define TSC_XOFF           20          // X offset
#define TSC_YOFF           20          // Y offset
#define TSC_MAXVAL         37000       // 0x0FFF * 10 with TSC to LCD scaling

// System Control registers
#define BRD_COUNTER     ((volatile unsigned int *)(0x40028018))

// External function types

// Initialise TSC
int tsc_init(tsc_t *obj, PinName sda, PinName scl);
// TSC read pen down, X,Y
unsigned int apTSC_READXY(unsigned int *PENX, unsigned int *PENY);

// External function types for the TSC I2C
unsigned int TSC_read (unsigned char reg_addr, unsigned char sadr, unsigned char bytes);
unsigned int TSC_read_x (void);
unsigned int TSC_read_y (void);
void         TSC_write(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);
void         TSC_clear(void);
// TSC read pen down, X,Y
unsigned int TSC_readxy(unsigned int *PENX, unsigned int *PENY);


#ifdef __cplusplus
}
#endif

#endif

#endif
