/*
* Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "fsl_adc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Internal type defination
 ******************************************************************************/

/* define ADC ISR's context that keep the current configuration */
typedef struct adc_context
{
    adc_isr_callback_t 	userCallback; /* user registed callback isr function */
    uint32_t            muxSelect;     /* selection of ping-pang group */
    uint32_t            value;        /* the latest value of conversion */
} adc_context_t;

/*******************************************************************************
 * Internal Variables
 ******************************************************************************/

/* ADC's IRQ list */
extern IRQn_Type adc_irq_ids[HW_ADC_INSTANCE_COUNT];

/* Keep the cortext for active converter */
static adc_context_t adc_context_internal[HW_ADC_INSTANCE_COUNT];

/*******************************************************************************
 * Internal Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_get_irq_id_internal
 * Description  : return the responding IRQn id  related to ADC instance.
 *
 *END**************************************************************************/
static IRQn_Type adc_get_irq_id_internal(uint32_t instance)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);
    return adc_irq_ids[instance];
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_isr_internal
 * Description  : process ADC's ISR internal that update the conversion value
 * into the buffer and call the user callback if registered.
 *
 *END**************************************************************************/
void adc_isr_internal(uint32_t instance)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);
    /* move the conversion value to buffer in context variable */
    adc_context_internal[instance].value =
        adc_hal_get_conversion_value(instance, adc_context_internal[instance].muxSelect);
    /* call the user-defined ADC isr which stored in context array */
    if (adc_context_internal[instance].userCallback)
    {
        (*(adc_context_internal[instance].userCallback))(); 
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_get_calibration_param
 * Description  : This function is to get the calibration parameters in 
 * auto-calibrate mode. It is recommended to execute this fucntion to obtain 
 * the parameter for calibration during the initialzation, even though it will 
 * take a few time.
 *
 *END**************************************************************************/
adc_status_t adc_get_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr)
{
    if (kStatus_ADC_Fail ==  adc_hal_start_calibration(instance))
    {
        return kStatus_ADC_Fail;
    }
    
    paramPtr->PG = adc_hal_get_calibration_PG(instance);
    paramPtr->MG = adc_hal_get_calibration_MG(instance);
    /* clear R register after calibration */
    adc_hal_get_conversion_value(instance, 0U);
    
    adc_hal_end_calibration(instance);
    
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_set_calibration_param
 * Description  : This function is to set the calibration parameters when 
 * necessary. The parameters could be generated from the auto-calibration by 
 * adc_get_calibration_param() or created by mannually indicated ones.
 *
 *END**************************************************************************/
adc_status_t adc_set_calibration_param(uint32_t instance, adc_calibration_param_t *paramPtr)
{
    if (paramPtr == NULL)
    {
        return kStatus_ADC_InvalidArgument;
    }
    
    adc_hal_set_calibration_PG(instance, paramPtr->PG);
    adc_hal_set_calibration_MG(instance, paramPtr->MG);
    
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_init
 * Description  : initialize the conventer with basic configuration that will
 * let ADC work in the basic mode.
 *
 *END**************************************************************************/
adc_status_t adc_init(uint32_t instance, adc_config_t *cfgPtr)
{
    adc_calibration_param_t calParam;
    
    /* check the input parameters */
    if (cfgPtr == NULL)
    {
        return kStatus_ADC_InvalidArgument;
    }
    
    /* Enable the ADC's clock from peripheral bus */
    clock_manager_set_gate(kClockModuleADC, instance, true);

    /* Set the clock configuration */
    adc_hal_set_clock_source_mode(instance, cfgPtr->clockSourceMode);  
    adc_hal_set_clock_divider_mode(instance, cfgPtr->clockSourceDividerMode);
    
    /* Set the reference voltage */
    adc_hal_set_reference_voltage_mode(instance, cfgPtr->referenceVoltageMode);
    
    /* Set the sample resolution */
    adc_hal_set_resolution_mode(instance, cfgPtr->resolutionMode);
    
    /* Set the continuous mode */
    adc_hal_configure_continuous_conversion(instance, cfgPtr->isContinuousEnabled);
    
    /* 
     * Start calibrating when necessary. If the calibration parameter is not 
     * indicated, auto-calibration will be executed. Otherwise, the indicated 
     * parameter will be used to do calibration.
     */
    if (cfgPtr->calibrationParam == NULL)
    {
        if (kStatus_ADC_Success != adc_get_calibration_param(instance, &calParam))
        {
            return kStatus_ADC_Fail;
        }
        adc_set_calibration_param(instance, &calParam);
    }
    else
    {
        adc_set_calibration_param(instance, cfgPtr->calibrationParam);
    }
    
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_init_extend
 * Description  : initialize the conventer with extend configuration that will
 * let ADC work in the advanced mode.
 *
 *END**************************************************************************/
adc_status_t adc_init_extend(uint32_t instance, adc_extend_config_t *extendCfgPtr)
{
    /* check the input parameters */
    if (extendCfgPtr == NULL)
    {
        return kStatus_ADC_InvalidArgument;
    }
	
    /* conversion speed mode */
    adc_hal_configure_high_speed(instance, extendCfgPtr->isHighSpeedEnabled);
    
    /* long sample mode */
    if (extendCfgPtr->isLongSampleEnabled)
    {
        adc_hal_set_long_sample_mode(instance, extendCfgPtr->hwLongSampleMode);
    }
    adc_hal_configure_long_sample(instance, extendCfgPtr->isLongSampleEnabled);
    
    /* low power mode */
    adc_hal_configure_low_power(instance, extendCfgPtr->isLowPowerEnabled);
    
    /* enable the asynchronic clock before converison */
    adc_hal_configure_asynchronous_clock(instance, extendCfgPtr->isAsynClockEnabled);
    
    /* is hardware trigger need after the software setting(trigger) */
    adc_hal_configure_hw_trigger(instance, extendCfgPtr->isHwTriggerEnabled);
    
    /* hardware average mode */
    if (extendCfgPtr->isHwAverageEnabled)
    {
        adc_hal_set_hw_average_mode(instance, extendCfgPtr->hwAverageSampleMode);
    }
    adc_hal_configure_hw_average(instance, extendCfgPtr->isHwAverageEnabled);
	
    /* hardware comparation mode */
    if (extendCfgPtr->isHwCompareEnabled)
    {
        adc_hal_set_hw_compare_value1(instance, extendCfgPtr->hwCompareValue1);
        adc_hal_set_hw_compare_value2(instance, extendCfgPtr->hwCompareValue2);    
        adc_hal_configure_hw_compare_greater(instance, 
                                extendCfgPtr->isHwCompareGreaterEnabled);
        adc_hal_configure_hw_compare_in_range(instance, 
                                extendCfgPtr->isHwCompareRangeEnabled);
    }
    adc_hal_configure_hw_compare(instance, extendCfgPtr->isHwCompareEnabled);

    /* Set the DMA configuration */
    adc_hal_configure_dma(instance, extendCfgPtr->isDmaEnabled);
  
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_shutdown
 * Description  : shutdown the ADC by cut off its clock supply.
 *
 *END**************************************************************************/
void adc_shutdown(uint32_t instance)
{   
    /* Disable the ADC's clock from peripheral bus */
    clock_manager_set_gate(kClockModuleADC, instance, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_start_conversion
 * Description  : start the conversion from indicated channel.
 *
 *END**************************************************************************/
adc_status_t adc_start_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr)
{
    /* check the input parameters */
    if (channelCfgPtr == NULL)
    {
        return kStatus_ADC_InvalidArgument;
    }
    /* disable the conversion during configure the interrupt */
    adc_hal_disable(instance, channelCfgPtr->muxSelect);

    /* set the group mux */
    adc_hal_set_group_mux(instance, channelCfgPtr->muxSelect);
    
    /* register ADC isr's context to be current configuration */
    if (channelCfgPtr->isInterruptEnabled)
    {
        adc_context_internal[instance].muxSelect = channelCfgPtr->muxSelect;
        interrupt_enable(adc_get_irq_id_internal(instance));
        adc_hal_configure_interrupt(instance, channelCfgPtr->muxSelect, true);
    }
    else
    {
        adc_hal_configure_interrupt(instance, channelCfgPtr->muxSelect, false);
        interrupt_disable(adc_get_irq_id_internal(instance));
    } /* end_if */
    
    /* start the conversion in indicated channel */
    adc_hal_enable(instance, channelCfgPtr->muxSelect,
        channelCfgPtr->channelId, channelCfgPtr->isDifferentialEnabled);
  
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_stop_conversion
 * Description  : stop the conversion from indicated channel.
 *
 *END**************************************************************************/
adc_status_t adc_stop_conversion(uint32_t instance, adc_channel_config_t *channelCfgPtr)
{
    /* check the input parameters */
    if (channelCfgPtr == NULL)
    {
        return kStatus_ADC_InvalidArgument;
    }
    adc_hal_disable(instance, channelCfgPtr->muxSelect);
    adc_hal_configure_interrupt(instance, channelCfgPtr->muxSelect, false);
    
    return kStatus_ADC_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_is_conversion_completed
 * Description  : check whether the conversion is complete.
 *
 *END**************************************************************************/
bool adc_is_conversion_completed(uint32_t instance, adc_channel_config_t *channelCfgPtr)
{
    /* check the input parameters */
    if (channelCfgPtr == NULL)
    {
        return false;
    }
    return adc_hal_is_conversion_completed(instance, channelCfgPtr->muxSelect);
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_get_conversion_value
 * Description: return the value of conversion. The value just comes from value
 * register that may be eventually processed according to application.
 * when use polling mode, the value will be obtain after the converions is 
 * completed. when use interrupt mode, the value will come from the buffer
 * that is updated by ADC ISR.
 *
 *END**************************************************************************/
uint32_t adc_get_conversion_value(uint32_t instance, adc_channel_config_t *channelCfgPtr)
{
    /* check the input parameters */
    if ((instance >= HW_ADC_INSTANCE_COUNT) || (channelCfgPtr == NULL))
    {
        return 0U;
    }
    
    if (!(channelCfgPtr->isInterruptEnabled)) /* Polling mode */
    {
        while (!adc_hal_is_conversion_completed(instance, channelCfgPtr->muxSelect))
        {}
        adc_context_internal[instance].value = 
                adc_hal_get_conversion_value(instance, channelCfgPtr->muxSelect);
    }
    /* else the value will be updated by driver's internal isr */
    return adc_context_internal[instance].value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: adc_register_user_callback_isr
 * Description  : private to register the user callback funcion into ADC ISR.
 *
 *END**************************************************************************/
void adc_register_user_callback_isr(uint32_t instance, adc_isr_callback_t func)
{
    adc_context_internal[instance].userCallback = func;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
