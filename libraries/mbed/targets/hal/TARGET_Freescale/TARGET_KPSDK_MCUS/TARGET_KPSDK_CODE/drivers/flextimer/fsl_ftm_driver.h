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
#ifndef __FSL_FTM_DRIVER_H__
#define __FSL_FTM_DRIVER_H__

#include "fsl_ftm_hal.h"

/*!
 * @addtogroup ftm_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Channel information */
typedef struct FtmChannelInfo
{
    ftm_config_mode_t   mode;                   /*!< FlexTimer operation mode */
    ftm_edge_mode_t     edge_mode;              /*!< capture mode */
    bool                isChannelEnabled;
    bool                isSoftwareOutputCTRL;
    bool                isSoftwareOutput;       /*!< 1:output high 0:output low*/
    bool                isChannlePinEnabled;
    bool                isChannelInterrupt;
    bool                isChannelDMA;
    uint8_t             inputCaptureFilterVal;
}ftm_channel_info_t;

/*! @brief Combines channel information.*/
typedef struct FtmCombinedChannelSetting
{
    bool   isComplementaryMode;
    bool   isFaultCTRLEnabled;
    bool   isSyncEnabled;
    bool   isDeadTimeEnabled;
    bool   isInvertingEnabled;
}ftm_combined_channel_info_t;

/*! @brief Internal driver state information grouped by naming. User  needs to set the relevant ones.*/
typedef struct FtmUserConfig {
    uint8_t instance;                /*!< name FTM instance FTM0, FTM1, FTM2, FTM3 */

    ftm_clock_source_t  clockSrc;
    ftm_clock_ps_t      clockPS;
    uint16_t            counterInitVal;
    uint16_t            counterMod;
    uint32_t            frequencyHZ;
    ftm_deadtime_ps_t   deadtimePrescaler;
    uint8_t             uDeadTimeCount;
    uint8_t             uNumOfOverflows;
    bool      isFTMMode;
    uint8_t   BDMMode;
    bool      isGlobalTimeBase;
    bool      isGlobalTimeOutput;
    bool      isWriteProtection;

    ftm_combined_channel_info_t  combinedChanSetting[HW_FTM_CHANNEL_PAIR_COUNT];

    bool isEnhancedSyncMode;
    bool isHWTriggerMode;
    bool isCNTINSync;
    bool isINVCTRLSync;
    bool isSWOCCTRLsync;
    bool isSWTriggermode;
    bool isOutmaskSync;
    bool isMinLoadingPoint;
    bool isMaxLoadingPoint;

    bool isExternalCINTTrigger;
    bool isExternalChan0Trigger;
    bool isExternalChan1Trigger;
    bool isExternalChan2Trigger;
    bool isExternalChan3Trigger;
    bool isExternalChan4Trigger;
    bool isExternalChan5Trigger;

    bool isFaultCTRLMode;
    bool isFaultInputFilter;
    bool isFaultInput0Filter;
    bool isFaultInput0Polarity;
    bool isFaultInput1Filter;
    bool isFaultInput1Polarity;
    bool isFaultInput2Filter;
    bool isFaultInput2Polarity;
    bool isFaultInput3Filter;
    bool isFaultInput3Polarity;

    ftm_channel_info_t  channleInfo[HW_FTM_CHANNEL_COUNT];

    bool enableFaultPin0;
    bool enableFaultPin1;
    bool enableFaultPin2;
    bool enableFaultPin3;

    uint8_t externalClockPin;

    bool isHardwareTrigger0;
    bool isHardwareTrigger1;
    bool isHardwareTrigger2;

    uint8_t interruptPriority;
    uint8_t interruptInstance;
    bool isInterruptRequest;
    bool isTimerOverFlowInterrupt;
    bool isFaultInterrupt;

    bool isInitChannelOutput;
    bool isLoadEnable;
    bool isChannel0LoadSel;
    bool isChannel1LoadSel;
    bool isChannel2LoadSel;
    bool isChannel3LoadSel;
    bool isChannel4LoadSel;
    bool isChannel5LoadSel;
    bool isChannel6LoadSel;
    bool isChannel7LoadSel;

    /* ftm_config_mode_t   mode;*/
    /* ftm_edge_mode_t     edge_mode    */

} ftm_user_config_t;

/*!
 * @brief FlexTimer driver PWM parameter
 *
 * Setting uPulseHigPercentage = 1, uPulseLowPulsePercentage = 1
 * means sin wave equal to low and high width.
 */
typedef struct FtmDriverPwmParam
{
  uint32_t uFrequencyHZ;           /*!< PWM period */
  uint32_t uPulseHighPercentage;   /*!< High pulse width period */
  uint32_t uPulseLowPercentage;    /*!< Low pulse width period*/
  uint16_t uCnV;                   /*!< In combined mode, the n channel count value, the duty cycle=|Cn+1V -CnV| */
}ftm_pwm_param_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the FTM driver.
 *
 * @param instance The FTM peripheral instance number.
 * @param info The FTM user config structure.
 */
void ftm_init(uint8_t instance, ftm_user_config_t *info);

/*!
 * @brief Shuts down the FTM driver.
 *
 * @param state The FTM user config structure.
 */
void ftm_shutdown(ftm_user_config_t *state);

/*!
 * @brief Starts channel PWM.
 * @param info The FTM user config structure.
 * @param channel The channel or channel pair number(combined mode).
 */
void ftm_pwm_start( ftm_user_config_t *info, uint8_t channel);

/*!
 * @brief Stops channel PWM.
 *
 * @param info The FTM user config structure.
 * @param channel The channel or channel pair number(combined mode).
 */
void ftm_pwm_stop( ftm_user_config_t *info,uint8_t channel);

/*!
 * @brief Configures duty cycle and frequency of the channel PWM.
 * @param info The FTM user config structure.
 * @param channel The channel or channel pair number(combined mode).
 *                For channel pair, channel= 0(0,1),1(2,3),2(4,5),3(6,7)
 * @param param FTM driver PWM parameter to configure PWM options
 */
void ftm_pwm_configure(ftm_user_config_t *info,uint8_t channel, ftm_pwm_param_t *param);

/*Other API functions are for input capture, output compare, dual edge capture, and quadrature. */
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_FTM_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
