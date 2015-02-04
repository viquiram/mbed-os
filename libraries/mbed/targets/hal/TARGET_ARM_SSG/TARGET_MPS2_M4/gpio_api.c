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

// function to enable the GPIO pin
uint32_t gpio_set(PinName pin) {
//    pin_function(pin, 0);
//    return (1 << ((int)pin & 0xF));
	return (1);
}

//function to initialise the gpio pin
// this links the board control bits for each pin 
// with the object created for the pin
void gpio_init(gpio_t *obj, PinName pin) {
		if(pin == NC){ return;}
		else {
		int pin_value = 0;
		obj->pin = pin;
		if(pin <=15){
			pin_value = pin;
		}else if (pin >= 16 && pin <= 31){
			pin_value = pin-16;
		}else if (pin >= 32 && pin <= 47){
			pin_value = pin-32;
		}else if (pin >= 48 && pin <= 51){
			pin_value = pin-48;
		}else if (pin == 100 || pin == 101){
			pin_value = pin-100;
		}else if (pin == 110 || pin == 111){
			pin_value = pin-110;
		}else if (pin >= 200 && pin <= 207){
			pin_value = pin-200;
		}else if (pin >= 210 && pin <= 217){
			pin_value = pin-210;
		}else if (pin == 303){
			pin_value = pin-302;
		}else if (pin == 307){
			pin_value = pin-307;
		}
		
		obj->mask = 0x1 << pin_value;
		obj->pin_number = pin;
		if(pin <=15) {
			//CMSDK_GPIO_TypeDef *port_reg = (CMSDK_GPIO_TypeDef *) (((int)pin) & ~0xF); //+ pin_number);
			obj->reg_data = &CMSDK_GPIO0->DATAOUT ;
//			obj->reg_clr = &port_reg->DATAOUT;
			obj->reg_in  = 		&CMSDK_GPIO0->DATA ;
			obj->reg_dir = 		&CMSDK_GPIO0->OUTENABLESET ;
			obj->reg_dirclr = &CMSDK_GPIO0->OUTENABLECLR ;
		} else if (pin >= 16 && pin <= 31){
			obj->reg_data = &CMSDK_GPIO1->DATAOUT ;
//			obj->reg_clr = &port_reg->DATAOUT;
			obj->reg_in  = 		&CMSDK_GPIO1->DATA ;
			obj->reg_dir = 		&CMSDK_GPIO1->OUTENABLESET ;
			obj->reg_dirclr = &CMSDK_GPIO1->OUTENABLECLR ;
		} else if (pin >= 32 && pin <= 47){
			obj->reg_data = &CMSDK_GPIO2->DATAOUT;
//			obj->reg_clr = &port_reg->DATAOUT;
			obj->reg_in  = 		&CMSDK_GPIO2->DATA;
			obj->reg_dir = 		&CMSDK_GPIO2->OUTENABLESET ;
			obj->reg_dirclr = &CMSDK_GPIO2->OUTENABLECLR ;
		}	else if (pin >= 48 && pin <= 51){
			obj->reg_data = &CMSDK_GPIO3->DATAOUT;
//			obj->reg_clr = &port_reg->DATAOUT;
			obj->reg_in  = 		&CMSDK_GPIO3->DATA;
			obj->reg_dir = 		&CMSDK_GPIO3->OUTENABLESET ;
			obj->reg_dirclr = &CMSDK_GPIO3->OUTENABLECLR ;
		} else if (pin == 100 || pin == 101){
			obj->reg_data = &MPS2_FPGAIO->LED; //user leds
		} else if (pin == 110 || pin == 111){
			obj->reg_data = &MPS2_FPGAIO->BUTTON; //user leds
		}else if (pin >= 200 && pin <= 207){
			obj->reg_data = &MPS2_SCC->LEDS; //mcc leds
		}else if (pin >= 210 && pin <= 217){
			obj->reg_in = &MPS2_SCC->SWITCHES; //mcc switches
		}else if (pin == 303 || pin == 307){
			obj->reg_data = &MPS2_FPGAIO->MISC; //mcc switches
		}
	}
}

void gpio_mode(gpio_t *obj, PinMode mode) {
		pin_mode(obj->pin, mode);
}

void gpio_dir(gpio_t *obj, PinDirection direction) {
		if((obj->pin >= 100 && obj->pin <= 109)
				||(obj->pin >= 200 && obj->pin <= 207))
			{return;}
		switch (direction) {
				case PIN_INPUT : *obj->reg_dirclr = obj->mask;  break;
				case PIN_OUTPUT: *obj->reg_dir |= obj->mask; break;
		}
}
