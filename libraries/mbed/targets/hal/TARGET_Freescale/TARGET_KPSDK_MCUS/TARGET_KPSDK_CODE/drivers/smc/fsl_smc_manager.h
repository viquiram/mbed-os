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

#if !defined(__FSL_SMC_MANAGER_H__)
#define __FSL_SMC_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_smc_features.h"
#include "fsl_smc_hal.h"

/*! @addtogroup smc_manager*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Power Modes */
typedef enum _power_modes {
    kPowerModeRun,
    kPowerModeWait,
    kPowerModeStop,
    kPowerModeVlpr,
    kPowerModeVlpw,
    kPowerModeVlps,
    kPowerModeLls,
    kPowerModeVlls,
    kPowerModeMax
} power_modes_t;

/*!
 * @brief Error code definition for the system mode controller manager APIs.
 */
typedef enum _smc_manager_error_code {
    kSmcManagerSuccess,                           /*!< Success */
    kSmcManagerNoSuchModeName,                    /*!< Cannot find the mode name specified*/
    kSmcManagerAlreadyInTheState,                 /*!< Already in the required state*/
    kSmcManagerFailed                             /*!< Unknown error, operation failed*/
} smc_manager_error_code_t;

/*! @brief Power mode control configuration used for calling the smc_set_power_mode API. */
typedef struct _smc_power_mode_config {
    power_modes_t       powerModeName;      /*!< Power mode(enum), see power_modes_t */
    smc_stop_submode_t  stopSubMode;        /*!< Stop submode(enum), see smc_stop_submode_t */
    bool                lpwuiOption;        /*!< If LPWUI option is needed */
    smc_lpwui_option_t  lpwuiOptionValue;   /*!< LPWUI option(enum), see smc_lpwui_option_t */
    bool                porOption;          /*!< If POR option is needed */
    smc_por_option_t    porOptionValue;     /*!< POR option(enum), see smc_por_option_t */
} smc_power_mode_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Power Mode Configuration*/
/*@{*/

/*!
 * @brief Configures the power mode
 *
 * This function configures the power mode control for both run, stop, and
 * stop sub mode if needed. Also it configures the power options for a specific
 * power mode. An application should follow the proper procedure to configure and 
 * switch power modes between  different run and stop modes. For proper procedures and supported power modes, see an appropriate chip reference
 * manual. See the smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to be individually configured through the HAL driver. See the HAL driver
 * header file for details.
 *
 * @param powerModeConfig Power mode configuration structure smc_power_mode_config_t 
 */
smc_manager_error_code_t smc_set_power_mode(const smc_power_mode_config_t *powerModeConfig);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_SMC_MANAGER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

