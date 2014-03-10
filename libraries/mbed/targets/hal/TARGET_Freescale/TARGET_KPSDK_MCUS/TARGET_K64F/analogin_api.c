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
#include "analogin_api.h"

#include "cmsis.h"
#include "pinmap.h"
#include "error.h"
#include "fsl_adc_driver.h"
#include "PeripheralNames.h"
 
static const PinMap PinMap_ADC[] = {
    {PTC0, ADC0_SE14, 0},
    {NC,   NC,        0}
};

void analogin_init(analogin_t *obj, PinName pin) {
    obj->adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
    adc_user_config_t module_config;
    uint32_t instance = obj->adc >> ADC_SHIFT;

    module_config.clockSourceMode = kAdcClockSourceBusClk;
    module_config.clockSourceDividerMode = kAdcClockDivider8;
    module_config.resolutionMode = kAdcSingleDiff16;
    module_config.referenceVoltageMode = kAdcVoltageVref;
    module_config.isContinuousEnabled = false;

    adc_init(instance, &module_config);

    obj->channel_cfg.channelId =(adc_channel_mode_t)(obj->adc & 0xF);
    obj->channel_cfg.isDifferentialEnabled = false;
    obj->channel_cfg.isInterruptEnabled = false;
    obj->channel_cfg.muxSelect =  kAdcChannelMuxB;
    adc_start_conversion(instance, &obj->channel_cfg);

}

uint16_t analogin_read_u16(analogin_t *obj) {
    adc_start_conversion(obj->adc >> ADC_SHIFT, &obj->channel_cfg);
    return (uint16_t)adc_get_conversion_value(obj->adc >> ADC_SHIFT, &obj->channel_cfg);
}

float analogin_read(analogin_t *obj) {
    uint16_t value = analogin_read_u16(obj);
    return (float)value * (1.0f / (float)0xFFFF);
}

