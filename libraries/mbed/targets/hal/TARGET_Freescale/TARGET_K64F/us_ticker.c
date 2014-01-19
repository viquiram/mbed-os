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
#include <stddef.h>
#include "us_ticker_api.h"
#include "PeripheralNames.h"

static void pit_init(void);
static void lptmr_init(void);

static int us_ticker_inited = 0;

void us_ticker_init(void) {

}

static uint32_t pit_us_ticker_counter = 0;

void pit0_isr(void) {

}

uint32_t us_ticker_read() {
}
/******************************************************************************
 * Timer for us timing.
 ******************************************************************************/
static void pit_init(void) {

}

/******************************************************************************
 * Timer Event
 *
 * It schedules interrupts at given (32bit)us interval of time.
 * It is implemented used the 16bit Low Power Timer that remains powered in all
 * power modes.
 ******************************************************************************/
static void lptmr_isr(void);

static void lptmr_init(void) {

}

void us_ticker_disable_interrupt(void) {

}

void us_ticker_clear_interrupt(void) {
    // we already clear interrupt in lptmr_isr
}

static uint32_t us_ticker_int_counter = 0;
static uint16_t us_ticker_int_remainder = 0;

static void lptmr_set(unsigned short count) {
    /* Reset */

}

static void lptmr_isr(void) {

}

void us_ticker_set_interrupt(unsigned int timestamp) {

}
