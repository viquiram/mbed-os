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

#include <string.h>
#include "fsl_smc_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : smc_set_power_mode
 * Description   : Config the power mode
 * This function will configure the power mode control for any run, stop and
 * stop submode if needed. It will also configure the power options for specific
 * power mode. Application should follow the proper procedure to configure and 
 * switch power mode between the different run and stop mode. Refer to reference
 * manual for the proper procedure and supported power mode that can be configured
 * and switch between each other. Refert to smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to configure through the hal driver individaully. Refer to hal driver
 * header for details. 
 * 
 *END**************************************************************************/
smc_manager_error_code_t smc_set_power_mode(const smc_power_mode_config_t *powerModeConfig)
{
    uint8_t currentStat;
    volatile unsigned int dummyread;
    smc_stop_mode_t stopMode;
    smc_run_mode_t runMode;
    power_mode_stat_t modeStat;
    power_modes_t powerModeName = powerModeConfig->powerModeName;

    /* verify the power mode name*/
    assert(powerModeName < kPowerModeMax);

#if  FSL_FEATURE_SMC_HAS_LPWUI     
    /* check lpwui option*/
    if (powerModeConfig->lpwuiOption)
    {
       /* check current stat*/
        currentStat = smc_hal_get_power_mode_stat();

        /* if not in VLPR stat, could not set to RUN*/
        if (currentStat == kStatRun)
        {
            smc_hal_config_lpwui_option(powerModeConfig->lpwuiOptionValue);
        }
    }
#endif
    
    /* branch based on power mode name*/
    switch (powerModeName)
    {
    case kPowerModeRun:
    case kPowerModeVlpr:
        if (powerModeName == kPowerModeRun)
        {
            /* mode setting for normal RUN*/
            runMode = kSmcRun;
            modeStat = kStatVlpr;
        }
        else
        {
            /* mode setting for VLPR*/
            runMode = kSmcVlpr;
            modeStat = kStatRun;
        }
        
        /* check current stat*/
        currentStat = smc_hal_get_power_mode_stat();

        /* if not in VLPR stat, could not set to RUN*/
        if (currentStat != modeStat)
        {
            return kSmcManagerFailed;
        }
        else
        {
            /* set power mode to normal RUN or VLPR*/
            smc_hal_power_mode_config_run(runMode);
            return kSmcManagerSuccess;
        }
        break;
    case kPowerModeWait:
    case kPowerModeVlpw:
        if (powerModeName == kPowerModeWait)
        {
            /* mode setting for normal RUN*/
            runMode = kSmcRun;
            modeStat = kStatRun;
        }
        else
        {
            /* mode setting for VLPR*/
            runMode = kSmcVlpr;
            modeStat = kStatVlpr;
        }

        /* check current stat*/
        currentStat = smc_hal_get_power_mode_stat();

        if (currentStat != modeStat)
        {
            /* if not in the mode, return error*/
            return kSmcManagerFailed;
        }
        else
        {
            /* set power mode to normal RUN or VLPR mode first*/
            smc_hal_power_mode_config_run(runMode);
        }

        /* Clear the SLEEPDEEP bit to disable deep sleep mode - enter wait mode*/
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
        __WFI();
        
        return kSmcManagerSuccess;
        break;

    case kPowerModeStop:
    case kPowerModeVlps:
    case kPowerModeLls:
        if (powerModeName == kPowerModeStop)
        {
            stopMode = kSmcStop;
        }
        else if (powerModeName == kPowerModeVlps)
        {
            stopMode = kSmcVlps;
        }
        else
        {
            stopMode = kSmcLls;
        }

        /* set power mode to specified STOP mode*/
        smc_hal_power_mode_config_stop(stopMode);

#if FSL_FEATURE_SMC_HAS_LLS_SUBMODE
        if (powerModeName == kPowerModeLls) 
        {
            /* further set the stop sub mode configuration*/
            smc_hal_power_mode_config_stop_submode(powerModeConfig->stopSubMode);
        }
#endif

        /* wait for write to complete to SMC before stopping core  */
        dummyread = smc_hal_get_power_mode_stat();
        dummyread = dummyread + 1;

        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP)*/
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        __WFI();

        return kSmcManagerSuccess;
        break;

    case kPowerModeVlls:
        /* set power mode to specified STOP mode*/
        smc_hal_power_mode_config_stop(kSmcVlls);

        /* further set the stop sub mode configuration*/
        smc_hal_power_mode_config_stop_submode(powerModeConfig->stopSubMode);

        /* check if Vlls0 option needs configuration*/
        if (powerModeConfig->stopSubMode == kSmcStopSub0)
        {
#if FSL_FEATURE_SMC_HAS_PORPO              
            if (powerModeConfig->porOption)
            {
                smc_hal_config_por_power_option(powerModeConfig->porOptionValue);
            }
#endif                
        }

        /* wait for write to complete to SMC before stopping core  */
        dummyread = smc_hal_get_power_mode_stat();
        dummyread = dummyread + 1;

        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP)*/
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        __WFI();

        return kSmcManagerSuccess;
        break;
    default:
        break;
    }
    
    return kSmcManagerNoSuchModeName;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

