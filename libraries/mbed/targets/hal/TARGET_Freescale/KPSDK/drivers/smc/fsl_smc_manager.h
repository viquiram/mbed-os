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
 * @brief error code definition for system mode controller manager APIs
 */
typedef enum _smc_manager_error_code {
    kSmcManagerSuccess,                           /*!< success */
    kSmcManagerNoSuchModeName,                    /*!< cannot find the mode name specified*/
    kSmcManagerAlreadyInTheState,                 /*!< already in the required state*/
    kSmcManagerFailed                             /*!< unknown error, operation failed*/
} smc_manager_error_code_t;

/*! @brief power mode control configuration, used for calling smc_set_power_mode API */
typedef struct _smc_power_mode_config {
    power_modes_t       powerModeName;      /*!< power mode(enum), see power_modes_t */
    smc_stop_submode_t  stopSubMode;        /*!< stop submode(enum), see smc_stop_submode_t */
    bool                lpwuiOption;        /*!< if LPWUI option is needed */
    smc_lpwui_option_t  lpwuiOptionValue;   /*!< LPWUI option(enum), see smc_lpwui_option_t */
    bool                porOption;          /*!< if POR option is needed */
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
 * @brief Config the power mode
 *
 * This function will configure the power mode control for both run, stop and
 * stop submode if needed. Also it will configure the power options for specific
 * power mode. Application should follow the proper procedure to configure and 
 * switch power mode between the different run and stop mode. Refer to reference
 * manual for the proper procedure and supported power mode that can be configured
 * and switch between each other. Refert to smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to configure through the hal driver individaully. Refer to hal driver
 * header for details.
 *
 * @param powerModeConfig Power mode config structure smc_power_mode_config_t 
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

