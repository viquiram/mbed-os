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

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif
	
typedef struct {
    PinName  pin;
    uint32_t mask;
		uint32_t pin_number;
	
    __IO uint32_t *reg_dir;
		__IO uint32_t *reg_dirclr;
    __IO uint32_t *reg_data;
//    __IO uint32_t *pin_number;
    __I uint32_t *reg_in;
} gpio_t;

static inline void gpio_write(gpio_t *obj, int value) {
				int pin_value = obj->pin_number;
    if (value == 1){
			if(pin_value <=15){
				*obj->reg_data |= (0x1 << pin_value); //gpio0
			}else if((pin_value >=16) && (pin_value <=31)){
				*obj->reg_data |= (0x1 << (pin_value-16)); //gpio1
			}else if((pin_value >=32) && (pin_value <=47)){
				*obj->reg_data |= (0x1 << (pin_value-32)); //gpio2
			}else if((pin_value >=48) && (pin_value <=51)){
				*obj->reg_data |= (0x1 << (pin_value-48)); //gpio3
			}else if(pin_value == 100 || pin_value == 101){ 
				*obj->reg_data |= (0x1 << (pin_value-100)); //user leds
			}else if((pin_value >=200) && (pin_value <=207)){ 
				*obj->reg_data |= (0x1 << (pin_value-200)); //mcc leds
			}else if(pin_value == 303){ 
				*obj->reg_data |= (0x1 << (pin_value-302)); //spi nSS
			}else if(pin_value == 307){ 
				*obj->reg_data |= (0x1 << (pin_value-307)); //clcd CS
			}
		} else if (value == 0){
			if(pin_value <=15){
				*obj->reg_data &= ~(0x1 << pin_value); //gpio0
			}else if((pin_value >=16) && (pin_value <=31)){
				*obj->reg_data &= ~(0x1 << (pin_value-16)); //gpio1
			}else if((pin_value >=32) && (pin_value <=47)){
				*obj->reg_data &= ~(0x1 << (pin_value-32)); //gpio2
			}else if((pin_value >=48) && (pin_value <=51)){
				*obj->reg_data &= ~(0x1 << (pin_value-48)); //gpio3
			}else if(pin_value == 100 || pin_value == 101){
				*obj->reg_data &= ~(0x1 << (pin_value-100)); //user leds
			}else if((pin_value >=200) && (pin_value <=207)){
				*obj->reg_data &= ~(0x1 << (pin_value-200)); //mcc leds
			}else if(pin_value == 303){
				*obj->reg_data &= ~(0x1 << (pin_value-302)); //spi nSS
			}else if(pin_value == 307){
				*obj->reg_data &= ~(0x1 << (pin_value-307)); //clcd CS
			}
		}
}

static inline int gpio_read(gpio_t *obj) {
    return ((*obj->reg_in & obj->mask) ? 1 : 0);
}

#ifdef __cplusplus
}
#endif

#endif
