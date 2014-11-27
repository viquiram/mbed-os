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
//#include "CMSDK_driver.h"

#define US_TICKER_TIMER      CMSDK_DUALTIMER2
#define US_TICKER_TIMER_IRQn DUALTIMER_IRQn

int us_ticker_inited = 0;

void us_ticker_init(void) {
    if (us_ticker_inited) return;
    us_ticker_inited = 1;
    
	//CMSDK_timer_EnableIRQ(US_TICKER_TIMER);
	
	//CMSDK_timer_SetReload(US_TICKER_TIMER, 0x19);
	
    US_TICKER_TIMER->TimerControl = 0x82; // enable = 0x80, enable in periodic mode 0xC0, reset = 0
	
		US_TICKER_TIMER->TimerLoad = 0xFFFFFFFF;
    
    NVIC_SetVector(US_TICKER_TIMER_IRQn, (uint32_t)us_ticker_irq_handler);
    NVIC_EnableIRQ(US_TICKER_TIMER_IRQn);
}

uint32_t us_ticker_read() {
    if (!us_ticker_inited)
        us_ticker_init();
    int return_value = (US_TICKER_TIMER->TimerLoad - US_TICKER_TIMER->TimerValue)/25;
    return return_value;
}

void us_ticker_set_interrupt(unsigned int timestamp) {
    int delta = (int)(timestamp - us_ticker_read());
    if (delta <= 0) {
        // This event was in the past:
        us_ticker_irq_handler();
        return;
    }
		// enable interrupt
    
		//CMSDK_timer_EnableIRQ(US_TICKER_TIMER);
	
		US_TICKER_TIMER->TimerControl |= 0x20;
}

void us_ticker_disable_interrupt(void) {
    
		//CMSDK_timer_DisableIRQ(US_TICKER_TIMER);
	
		US_TICKER_TIMER->TimerControl &= 0x82;
}

void us_ticker_clear_interrupt(void) {
    US_TICKER_TIMER->TimerIntClr = 0x1;
}
