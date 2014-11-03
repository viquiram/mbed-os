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
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

#define PORT_SHIFT  5

typedef enum {
    // MPS2 EXP Pin Names
    EXP0 = 0,
    EXP1 = 1,
    EXP2 = 2,
    EXP3 = 3,
    EXP4 = 4,
    EXP5 = 5,
    EXP6 = 6,
    EXP7 = 7,
    EXP8 = 8,
    EXP9 = 9,
    EXP10 = 10,
    EXP11 = 11,
    EXP12 = 12,
    EXP13 = 13,
    EXP14 = 14,
    EXP15 = 15,
    EXP16 = 16,
    EXP17 = 17,
    EXP18 = 18,
    EXP19 = 19,
    EXP20 = 20,
    EXP21 = 21,
    EXP22 = 22,
    EXP23 = 23,
    EXP24 = 24,
    EXP25 = 25,
		
    EXP26 = 26,
    EXP27 = 27,
    EXP28 = 28,
    EXP29 = 29,
    EXP30 = 30,
    EXP31 = 31,
    EXP32 = 32, // NOT CONNECTED
    EXP33 = 33, // NOT CONNECTED
    EXP34 = 34, // NOT CONNECTED
    EXP35 = 35, // NOT CONNECTED
    EXP36 = 36, // NOT CONNECTED
    EXP37 = 37, // NOT CONNECTED
    EXP38 = 38, // NOT CONNECTED
    EXP39 = 39, // NOT CONNECTED
    EXP40 = 40, // NOT CONNECTED
    EXP41 = 41, // NOT CONNECTED
    EXP42 = 42, // NOT CONNECTED
    EXP43 = 43, // NOT CONNECTED
    EXP44 = 44, // NOT CONNECTED
    EXP45 = 45, // NOT CONNECTED
    EXP46 = 46, // NOT CONNECTED
    EXP47 = 47, // NOT CONNECTED
    EXP48 = 48, // NOT CONNECTED
    EXP49 = 49, // NOT CONNECTED
    EXP50 = 50, // NOT CONNECTED
    EXP51 = 51, // NOT CONNECTED
		
// Other mbed Pin Names

		//LEDs on mps2
		//user leds
    USERLED1 = 100,
    USERLED2 = 101,
		//user switches
		USERSW1  = 110,
		USERSW2  = 111,
		
		//mcc leds
		LED1 = 200,
		LED2 = 201,
		LED3 = 202,
		LED4 = 203,
		LED5 = 204,
		LED6 = 205,
		LED7 = 206,
    LED8 = 207,
		
		//MCC Switches
		SW1 = 210,
		SW2 = 211,
		SW3 = 212,
		SW4 = 213,
		SW5 = 214,
		SW6 = 215,
		SW7 = 216,
		SW8 = 217,
		
		//MPS2 SPI header pins j21
		MOSI_SPI = 300,
		MISO_SPI = 301,
		SCLK_SPI = 302,
		SSEL_SPI = 303,
		
		//MPS2 CLCD SPI
		CLCD_MOSI = 304,
		CLCD_MISO = 305,
		CLCD_SCLK = 306,
		CLCD_SSEL = 307,
		
		//MPS2 Uart
		USBTX  = 400,
    USBRX  = 401,
		UART_TX1 = 402,
    UART_RX1 = 403,
		UART_TX2 = 404,
    UART_RX2 = 405,
		
		//MPS2 I2C touchscreen and audio
		TSC_SDA = 500,
		TSC_SCL = 501,
		AUD_SDA = 502,
		AUD_SCL = 503,
//		SDA3 = 504,
//		SCL3 = 505,
		
    // Not connected
    NC = (int)0xFFFFFFFF,
} PinName;

//typedef enum {
//    CHANNEL0 = FLEX_INT0_IRQn,
//    CHANNEL1 = FLEX_INT1_IRQn,
//    CHANNEL2 = FLEX_INT2_IRQn,
//    CHANNEL3 = FLEX_INT3_IRQn,
//    CHANNEL4 = FLEX_INT4_IRQn,
//    CHANNEL5 = FLEX_INT5_IRQn,
//    CHANNEL6 = FLEX_INT6_IRQn,
//    CHANNEL7 = FLEX_INT7_IRQn
//} Channel;

typedef enum {
    PullUp = 2,
    PullDown = 1,
    PullNone = 0,
    Repeater = 3,
    OpenDrain = 4,
    PullDefault = PullDown
} PinMode;

#ifdef __cplusplus
}
#endif

#endif

/*

    UART_TX = P0_19,
    UART_RX = P0_18,
*/

/*		ULED_0 = 52,
		ULED_1 = 53,
		ULED_2 = 54,
		ULED_3 = 55,
		ULED_4 = 56,
		ULED_5 = 57,
		ULED_6 = 58,
		ULED_7 = 59,


// EXP0 connector 
//   _____________________________
//  | 2                         34| 
//  | . . . . . . . . . . . . . . |
//  | . . . . . . . . . . . . . . |
//  |_1___________________________|
    p0_1 = EXP0,		// GPIO0
    p0_2 = EXP14,   // GPIO0
    p0_3 = EXP1,    // GPIO0
    p0_4 = EXP15,   // GPIO0
    p0_5 = EXP2,    // GPIO0
    p0_6 = EXP16,   // GPIO1
    p0_7 = EXP3,    // GPIO0
    p0_8 = EXP17,   // GPIO1
    p0_9 = EXP4,    // GPIO0
    p0_10 = EXP18,  // GPIO1
										// p11 = GND
										// p12 = GND
		p0_13 = EXP5,   // GPIO0
    p0_14 = EXP19,  // GPIO1
    p0_15 = EXP6,   // GPIO0
    p0_16 = EXP20,  // GPIO1
    p0_17 = EXP7,   // GPIO0
    p0_18 = EXP21,  // GPIO1
    p0_19 = EXP8,   // GPIO0
    p0_20 = EXP22,  // GPIO1
    p0_21 = EXP9,   // GPIO0
    p0_22 = EXP23,  // GPIO1
    p0_23 = EXP10,  // GPIO0
 										// p24 = GND
    p0_25 = EXP11,  // GPIO0
										// p26 = 3V_EXT
										// p27 = 3V_EXT
 										// p28 = GND
 										// p29 = GND
										// p30 = 3V_EXT
    p0_31 = EXP12,  // GPIO0
    p0_32 = EXP24,  // GPIO1
    p0_33 = EXP13,  // GPIO0
    p0_34 = EXP25,  // GPIO1

// EXP1 connector
//   _____________________________
//  | 2                         34| 
//  | . . . . . . . . . . . . . . |
//  | . . . . . . . . . . . . . . |
//  |_1___________________________|

    p1_1 = EXP26,		// GPIO1
    p1_2 = EXP40,   // NOT CONNECTED
    p1_3 = EXP27,   // GPIO1
    p1_4 = EXP41,   // NOT CONNECTED
    p1_5 = EXP28,   // GPIO1
    p1_6 = EXP42,   // NOT CONNECTED
    p1_7 = EXP29,   // GPIO1
    p1_8 = EXP43,   // NOT CONNECTED
    p1_9 = EXP30,   // GPIO1
    p1_10 = EXP44,  // NOT CONNECTED
										// p11 = GND
										// p12 = GND
		p1_13 = EXP31,  // GPIO1
    p1_14 = EXP45,  // NOT CONNECTED
    p1_15 = EXP32,  // NOT CONNECTED
    p1_16 = EXP46,  // NOT CONNECTED
    p1_17 = EXP33,  // NOT CONNECTED
    p1_18 = EXP47,  // NOT CONNECTED
    p1_19 = EXP34,  // NOT CONNECTED
    p1_20 = EXP48,  // NOT CONNECTED
    p1_21 = EXP35,  // NOT CONNECTED
    p1_22 = EXP49,  // NOT CONNECTED
    p1_23 = EXP36,  // NOT CONNECTED
 									  // p24 = GND
    p1_25 = EXP37,  // NOT CONNECTED
									  // p26 = 3V_EXT
									  // p27 = 3V_EXT
 									  // p28 = GND
 									  // p29 = GND
									  // p30 = 3V_EXT
    p1_31 = EXP38,  // NOT CONNECTED
    p1_32 = EXP50,  // NOT CONNECTED
    p1_33 = EXP39,  // NOT CONNECTED
    p1_34 = EXP51,  // NOT CONNECTED

    */
		
