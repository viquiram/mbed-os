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

#if !defined(__FSL_SMC_HAL_H__)
#define __FSL_SMC_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_smc_features.h"

/*! @addtogroup smc_hal*/
/*! @{*/

/*! @file fsl_smc_hal.h */

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @brief Power Modes in PMSTAT*/
typedef enum _power_mode_stat {
    kStatRun    = 0x01,             /*!< 0000_0001 - Current power mode is RUN*/
    kStatStop   = 0x02,             /*!< 0000_0010 - Current power mode is STOP*/
    kStatVlpr   = 0x04,             /*!< 0000_0100 - Current power mode is VLPR*/
    kStatVlpw   = 0x08,             /*!< 0000_1000 - Current power mode is VLPW*/
    kStatVlps   = 0x10,             /*!< 0001_0000 - Current power mode is VLPS*/
    kStatLls    = 0x20,             /*!< 0010_0000 - Current power mode is LLS*/
    kStatVlls   = 0x40,             /*!< 0100_0000 - Current power mode is VLLS*/
    kStatHsrun  = 0x80              /*!< 1000_0000 - Current power mode is HSRUN*/
} power_mode_stat_t;

/*! @brief Power Modes Protection*/
typedef enum _power_modes_protect {
    kAllowHsrun,                    /*!< Allow High Speed Run mode*/
    kAllowVlp,                      /*!< Allow Very-Low-Power Modes*/
    kAllowLls,                      /*!< Allow Low-Leakage Stop Mode*/
    kAllowVlls,                     /*!< Allow Very-Low-Leakage Stop Mode*/
    kAllowMax
} power_modes_protect_t;

/*!
 * @brief run mode definition
 */
typedef enum _smc_run_mode {
    kSmcRun,                                /*!< normal RUN mode*/
    kSmcReservedRun,
    kSmcVlpr,                               /*!< Very-Low-Power RUN mode*/
    kSmcHsrun                               /*!< High Speed Run mode (HSRUN)*/
} smc_run_mode_t;

/*!
 * @brief stop mode definition
 */
typedef enum _smc_stop_mode {
    kSmcStop,                               /*!< Normal STOP mode*/
    kSmcReservedStop1,                      /*!< Reserved*/
    kSmcVlps,                               /*!< Very-Low-Power STOP mode*/
    kSmcLls,                                /*!< Low-Leakage Stop mode*/
    kSmcVlls                                /*!< Very-Low-Leakage Stop mode*/
} smc_stop_mode_t;

/*!
 * @brief vlls/lls stop sub mode definition
 */
typedef enum _smc_stop_submode {
    kSmcStopSub0,                               
    kSmcStopSub1,                               
    kSmcStopSub2,                               
    kSmcStopSub3                                
} smc_stop_submode_t;

/*! @brief Low Power Wake Up on Interrupt option*/
typedef enum _smc_lpwui_option {
    kSmcLpwuiEnabled,                        /*!< Low Power Wake Up on Interrupt enabled*/
    kSmcLpwuiDisabled                        /*!< Low Power Wake Up on Interrupt disabled*/
} smc_lpwui_option_t;

/*! @brief Partial STOP opton*/
typedef enum _smc_pstop_option {
    kSmcPstopStop,                          /*!< STOP - Normal Stop mode*/
    kSmcPstopStop1,                         /*!< Partial Stop with both system and bus clocks disabled*/
    kSmcPstopStop2,                         /*!< Partial Stop with system clock disabled and bus clock enabled*/
    kSmcPstopReserved,
} smc_pstop_option_t;

/*! @brief POR opton*/
typedef enum _smc_por_option {
    kSmcPorEnabled,                        /*!< POR detect circuit is enabled in VLLS0*/
    kSmcPorDisabled                        /*!< POR detect circuit is disabled in VLLS0*/
} smc_por_option_t;

/*! @brief LPO power opton*/
typedef enum _smc_lpo_option {
    kSmcLpoEnabled,                        /*!< LPO clock is enabled in LLS/VLLSx*/
    kSmcLpoDisabled                        /*!< LPO clock is disabled in LLS/VLLSx*/
} smc_lpo_option_t;

/*! @brief power mode control options*/
typedef enum _smc_power_options {
    kSmcOptionLpwui,                        /*!< Low Power Wake Up on Interrupt*/
    kSmcOptionPropo                         /*!< POR option*/
} smc_power_options_t;

/*! @brief power mode protection configuration*/
typedef struct _smc_power_mode_protection_config {
    bool                vlpProt;            /*!< vlp protect*/
    bool                llsProt;            /*!< lls protect */
    bool                vllsProt;           /*!< vlls protect*/
} smc_power_mode_protection_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name system mode controller APIs*/
/*@{*/

/*!
 * @brief Config all power mode protection settings
 *
 * This function will configure the power mode protection settings for
 * supported power mode on the specified chip family. The availabe power modes
 * are defined in smc_power_mode_protection_config_t. Application should provide
 * the protect settings for all supported power mode on the chip and aslo this
 * should be done at early system level init stage. Refer to reference manual
 * for details. This register can only write once after power reset. So either
 * use this function or use the individual set function if you only have single
 * option to set.
 * 
 * @param protectConfig Configurations for the supported power mode protect settings
 *                      - See smc_power_mode_protection_config_t for details.
 */
void smc_hal_config_power_mode_protection(smc_power_mode_protection_config_t *protectConfig);

/*!
 * @brief Config the individual power mode protection setting
 *
 * This function will only configure the power mode protection settings for
 * a specified power mode on the specified chip family. The availabe power modes
 * are defined in smc_power_mode_protection_config_t. Refer to reference manual
 * for details. This register can only write once after power reset.
 *
 * @param protect Power mode to set for protection
 * @param allow   Allow or not allow the power mode protection
 */
void smc_hal_set_power_mode_protection(power_modes_protect_t protect, bool allow);

/*!
 * @brief Get the current power mode protection setting
 *
 * This function will get the current power mode protection settings for
 * a specified power mode.
 *
 * @param protect Power mode to set for protection
 * @return state  Status of the protection setting
 *                - true: Allowed
 *                - false: Not allowed
*/
bool smc_hal_get_power_mode_protection(power_modes_protect_t protect);

/*!
 * @brief Config the RUN mode control setting
 *
 * This function will set the run mode settings. For example, normal run mode,
 * very lower power run mode, etc. Refer to smc_run_mode_t for supported run
 * mode on the chip family. Refer to reference manual for details about the 
 * run mode.
 *
 * @param runMode Run mode setting defined in smc_run_mode_t
 */
void smc_hal_power_mode_config_run(smc_run_mode_t runMode);

/*!
 * @brief Get the current RUN mode config
 *
 * This function will get the run mode settings. Refer to smc_run_mode_t 
 * for supported run mode on the chip family. Refer to reference manual for 
 * details about the run mode.
 *
 * @return setting Run mode config setting
*/
smc_run_mode_t smc_hal_power_mode_get_run_config(void);

/*!
 * @brief Config the STOP mode control setting
 *
 * This function will set the stop mode settings. For example, normal stop mode,
 * very lower power stop mode, etc. Refer to smc_stop_mode_t for supported stop
 * mode on the chip family. Refer to reference manual for details about the 
 * stop mode.
 *
 * @param stopMode Stop mode defined in smc_stop_mode_t
 */
void smc_hal_power_mode_config_stop(smc_stop_mode_t stopMode);

/*!
 * @brief Get the current STOP mode control setting
 *
 * This function will get the stop mode settings. For example, normal stop mode,
 * very lower power stop mode, etc. Refer to smc_stop_mode_t for supported stop
 * mode on the chip family. Refer to reference manual for details about the 
 * stop mode.
 *
 * @return setting Current stop mode config setting
*/
smc_stop_mode_t smc_hal_power_mode_get_stop_config(void);

/*!
 * @brief Config the stop sub mode control setting 
 *
 * This function will set the stop submode settings. Some of the stop mode will
 * further have submode supported. Refer to smc_stop_submode_t for supported
 * stop submode and Refer to reference manual for details about the submode
 * for specific stop mode.
 *
 * @param stopSubMode Stop submode setting defined in smc_stop_submode_t
 *
 * @return none
 */
void smc_hal_power_mode_config_stop_submode(smc_stop_submode_t stopSubMode);

/*!
 * @brief Get the current stop submode config 
 *
 * This function will get the stop submode settings. Some of the stop mode will
 * further have submode supported. Refer to smc_stop_submode_t for supported
 * stop submode and Refer to reference manual for details about the submode
 * for specific stop mode.
 *
 * @return setting Current stop submode setting
*/
smc_stop_submode_t smc_hal_power_mode_get_stop_submode_config(void);

#if FSL_FEATURE_SMC_HAS_PORPO
/*!
 * @brief Config the POR (power-on-reset) option 
 *
 * This function will set the POR power option setting. It controls whether the
 * POR detect circuit (for brown-out detection) is enabled in certain stop mode.
 * The setting will be either enable or disable the above feature when POR 
 * happened. Refer to reference manual for details.
 *
 * @param option POR option setting refer to smc_por_option_t
 */
void smc_hal_config_por_power_option(smc_por_option_t option);

/*!
 * @brief Get the config of POR option 
 *
 * This function will set the POR power option setting. See config function
 * header for details.
 *
 * @return option Current POR option setting
*/
smc_por_option_t smc_hal_get_por_power_config(void);
#endif

#if FSL_FEATURE_SMC_HAS_PSTOPO
/*!
 * @brief Config the PSTOPOR (Partial Stop Option)
 *
 * This function will set the PSTOPOR option. It controls whether a Partial 
 * Stop mode is entered when STOPM=STOP. When entering a Partial Stop mode from
 * RUN mode, the PMC, MCG and flash remain fully powered, allowing the device 
 * to wakeup almost instantaneously at the expense of higher power consumption.
 * In PSTOP2, only system clocks are gated allowing peripherals running on bus
 * clock to remain fully functional. In PSTOP1, both system and bus clocks are
 * gated. Refer to smc_pstop_option_t for supported options. Refer to reference
 * manual for details.
 *
 * @param option PSTOPOR option setting defined in smc_pstop_option_t
 */
void smc_hal_config_pstop_power_option(smc_pstop_option_t option);

/*!
 * @brief Get the config of PSTOPO option 
 *
 * This function will get the current PSTOPOR option setting. Refer to config
 * function for more details.
 *
 * @return option Current PSTOPOR option setting
*/
smc_por_option_t smc_hal_get_pstop_power_config(void);
#endif

#if FSL_FEATURE_SMC_HAS_LPOPO
/*!
 * @brief Config the LPO option setting 
 *
 * This function will set the LPO option setting. It controls whether the 1kHZ
 * LPO clock is enabled in certain lower power stop modes. Refer to 
 * smc_lpo_option_t for supported options and refer to reference manual for 
 * details about this option.
 *
 * @param option LPO option setting defined in smc_lpo_option_t
 */
void smc_hal_config_lpo_power_option(smc_lpo_option_t option);

/*!
 * @brief Get the config of LPO option 
 *
 * This function will get the current LPO option setting. Refer to config 
 * function for details.
 *
 * @return option Current LPO option setting
*/
smc_por_option_t smc_hal_get_lpo_power_config(void);
#endif

#if FSL_FEATURE_SMC_HAS_LPWUI
/*!
 * @brief Config the LPWUI (Low Power Wake Up on interrup) option
 *
 * This function will set the LPWUI option. It will cause the system to exit
 * to normal RUN mode when any active interrupt occurs while in a certain lower
 * power mode. Refer to smc_lpwui_option_t for supported options and refer to 
 * reference manual for more details about this option.
 *
 * @param option LPWUI option setting defined in smc_lpwui_option_t
 */
void smc_hal_config_lpwui_option(smc_lpwui_option_t option);

/*!
 * @brief Get the current LPWUI option
 *
 * This function will get the LPWUI option. Refer to config function for more
 * details.
 *
 * @return setting Current LPWAUI option setting
*/
smc_lpwui_option_t smc_hal_get_lpwui_config(void);
#endif

/*!
 * @brief Get the current power mode stat
 *
 * This function will return the current power mode stat. Once application is 
 * switching the power mode, it should always check the stat to make sure it 
 * runs into the specified mode or not. Also application will need to check 
 * this mode before switching to certain mode. The system will require that
 * only certain mode could switch to other specific mode. Refer to the 
 * reference manual for details. Refer to _power_mode_stat for the meaning
 * of the power stat
 *
 * @return stat  Current power mode stat
*/
uint8_t smc_hal_get_power_mode_stat(void);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_SMC_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

