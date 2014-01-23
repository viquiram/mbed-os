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

#include "fsl_ftm_driver.h"
#include "fsl_clock_manager.h"
/*#include "fsl_ftm_shared_irqs.h"*/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*See fsl_ftm_driver.h for documentation of this function.*/
void ftm_init(uint8_t instance, ftm_driver_info_t * info)
{
    uint32_t channel_pair_index, channel = 0;

    info->instance = instance;
    /* clock setting initialization*/
    if(info->clockSrc == kClock_source_FTM_SystemClk)
    {
       clock_manager_set_gate(kClockModuleFTM, instance, true);
    }
    ftm_hal_set_clock_source(instance,info->clockSrc);
    ftm_hal_set_clock_ps(instance,info->clockPS);
    ftm_hal_set_counter_init_val(instance, info->counterInitVal);
    ftm_hal_set_mod(instance, info->counterMod);
    ftm_hal_set_deadtime_prescale(instance, info->deadtimePrescaler);
    ftm_hal_set_deadtime_count(instance, info->uDeadTimeCount);
    ftm_hal_set_tof_frequency(instance, info->uNumOfOverflows);
    ftm_hal_enable_global_time_base_output(instance, info->isGlobalTimeOutput);
    ftm_hal_enable_global_time_base(instance, info->isGlobalTimeBase);
    ftm_hal_enable_write_protection(instance, info->isWriteProtection);

    /*combined channel setting*/
    for(channel_pair_index = 0; channel_pair_index < HW_FTM_CHANNEL_PAIR_COUNT; channel_pair_index++)
    {
        ftm_hal_enable_dual_channel_comp(instance, channel, info->combinedChanSetting[channel_pair_index].isComplementaryMode);
        ftm_hal_enable_dual_channel_fault(instance, channel, info->combinedChanSetting[channel_pair_index].isFaultCTRLEnabled);
        ftm_hal_enable_dual_channel_pwm_sync(instance, channel, info->combinedChanSetting[channel_pair_index].isSyncEnabled);
        ftm_hal_enable_dual_channel_deadtime(instance, channel, info->combinedChanSetting[channel_pair_index].isDeadTimeEnabled);
        ftm_hal_enable_dual_channel_invert(instance, channel, info->combinedChanSetting[channel_pair_index].isInvertingEnabled);
        channel = channel + 2;
    }
    /*synchronization settings*/
    if(info->isEnhancedSyncMode)
    {
        ftm_hal_enable_enhanced_pwm_sync_mdoe(instance, info->isEnhancedSyncMode);
        ftm_hal_enable_hardware_sync_software_output_ctrl(instance, info->isHWTriggerMode);
        if(info->isHWTriggerMode)
        {
            ftm_hal_enable_hardware_sync_software_output_ctrl(instance, info->isSWOCCTRLsync);
            ftm_hal_enable_hardware_sync_invert_ctrl(instance, info->isINVCTRLSync);
            ftm_hal_enable_hardware_sync_output_mask(instance, info->isOutmaskSync);
            ftm_hal_enable_hardware_sync_counter(instance, info->isCNTINSync);
        }
        ftm_hal_enable_software_trigger(instance, info->isSWTriggermode);
        if(info->isSWTriggermode)
        {
            ftm_hal_enable_software_sync_swoctrl(instance, info->isSWOCCTRLsync);
            ftm_hal_enable_software_sync_invert_ctrl(instance, info->isINVCTRLSync);
            ftm_hal_enable_software_sync_output_mask(instance, info->isOutmaskSync);
            ftm_hal_enable_software_sync_counter(instance, info->isCNTINSync);
        }
        ftm_hal_enable_max_loading(instance, info->isMaxLoadingPoint);
        ftm_hal_enable_min_loading(instance, info->isMinLoadingPoint);
    }

    /*external trigger settings initialization*/
    if(info->isExternalCINTTrigger)
    {
        ftm_hal_enable_channel_trigger(instance, 0, info->isExternalChan0Trigger);
        ftm_hal_enable_channel_trigger(instance, 1, info->isExternalChan1Trigger);
        ftm_hal_enable_channel_trigger(instance, 2, info->isExternalChan2Trigger);
        ftm_hal_enable_channel_trigger(instance, 3, info->isExternalChan3Trigger);
        ftm_hal_enable_channel_trigger(instance, 4, info->isExternalChan4Trigger);
        ftm_hal_enable_channel_trigger(instance, 5, info->isExternalChan5Trigger);
    }

    /*Pin control setting initialization*/
    /*fault pin input*/
    ftm_hal_enable_channel_fault_input(instance, HW_CHAN0, info->enableFaultPin0);
    ftm_hal_enable_channel_fault_input(instance, HW_CHAN1, info->enableFaultPin1);
    ftm_hal_enable_channel_fault_input(instance, HW_CHAN2, info->enableFaultPin2);
    ftm_hal_enable_channel_fault_input(instance, HW_CHAN3, info->enableFaultPin3);
    /*hardware trigger pin initialization*/
    ftm_hal_set_hardware_trigger(instance, 0, info->isHardwareTrigger0);
    ftm_hal_set_hardware_trigger(instance, 1, info->isHardwareTrigger1);
    ftm_hal_set_hardware_trigger(instance, 2, info->isHardwareTrigger2);

    /*channle info initialization*/
    uint8_t chan = ((instance==1) || (instance == 2))?2:HW_FTM_CHANNEL_COUNT;
    for(channel = 0; channel < chan; channel++)
    {
      if(info->channleInfo[channel].isChannelEnabled)
      {
          ftm_hal_set_channel_MSnBA_mode(instance, channel, info->channleInfo[channel].mode);
          ftm_hal_set_channel_edge_level(instance, channel, (info->channleInfo[channel].edge_mode.ftm_pwm_edge_mode)?1:2);
          ftm_hal_enable_channle_dma(instance, channel, info->channleInfo[channel].isChannelDMA);
          if(info->channleInfo[channel].isChannelInterrupt)
          {
              ftm_hal_enable_channel_interrupt(instance, channel);
          }
          else
          {
              ftm_hal_disable_channel_interrupt(instance, channel);
          }
          ftm_hal_enable_channel_software_ctrl(instance, channel, info->channleInfo[channel].isSoftwareOutputCTRL);
          ftm_hal_set_channel_software_ctrl_val(instance, channel, info->channleInfo[channel].isSoftwareOutput);
          if(channel < HW_CHAN4)
          {
              ftm_hal_set_channel_input_capture_filter(instance, channel, info->channleInfo[channel].inputCaptureFilterVal);
          }
      }
    }
    /*interrupt initialization*/
    if(info->isTimerOverFlowInterrupt)
    {
      ftm_hal_enable_timer_overflow_interrupt(instance);
    }
    else
    {
      ftm_hal_disable_timer_overflow_interrupt(instance);
    }

    if(info->isFaultInterrupt)
    {
       ftm_hal_enable_fault_interrupt(instance);
    }
    else
    {
      ftm_hal_disable_fault_interrupt(instance);
    }

    /*channel output and loading select initialization*/
    /*channel output control init is in channel init section*/
     if(info->isLoadEnable)
     {
         ftm_hal_enable_pwm_load(instance, info->isLoadEnable);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN0, info->isChannel0LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN1, info->isChannel1LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN2, info->isChannel2LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN3, info->isChannel3LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN4, info->isChannel4LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN5, info->isChannel5LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN6, info->isChannel6LoadSel);
         ftm_hal_enable_pwm_load_matching_channel(instance, HW_CHAN7, info->isChannel7LoadSel);
     }
}

/*See fsl_ftm_driver.h for documentation of this function.*/
void ftm_shutdown(ftm_driver_info_t *state)
{
    ftm_hal_reset(state->instance);
    /* disable clock for FTM.*/
    clock_manager_set_gate(kClockModuleFTM, state->instance, false);
}

/*See fsl_ftm_driver.h for documentation of this function.*/
void ftm_pwm_start( ftm_driver_info_t *info, uint8_t channel)
{
    assert((info->channleInfo[channel].mode == kFtmEdgeAlignedPWM) ||
           (info->channleInfo[channel].mode == kFtmCenterAlignedPWM) ||
            (info->channleInfo[channel].mode == kFtmCombinedPWM));
    if((info->instance==1) || (info->instance == 2))
      assert(channel < HW_CHAN2);
    /*set clock source again after updating mod, cnv registers*/
     ftm_hal_set_clock_source(info->instance,info->clockSrc);
    if(info->combinedChanSetting[channel].isComplementaryMode)
    {
        info->combinedChanSetting[channel].isInvertingEnabled = true;
    }
    ftm_hal_enable_dual_channel_comp(info->instance, channel, info->combinedChanSetting[channel].isComplementaryMode);
    ftm_hal_enable_dual_channel_invert(info->instance, channel, info->combinedChanSetting[channel].isInvertingEnabled);
    ftm_config_t config =
    {
        .mode= info->channleInfo[channel].mode,
        .edge_mode = info->channleInfo[channel].edge_mode,
        .channel = channel,
    };
     ftm_hal_enable_pwm_mode(info->instance, &config);
}

/*See fsl_ftm_driver.h for documentation of this function.*/
void ftm_pwm_stop( ftm_driver_info_t *info,uint8_t channel)
{
    assert((info->channleInfo[channel].mode == kFtmEdgeAlignedPWM) ||
           (info->channleInfo[channel].mode == kFtmCenterAlignedPWM) ||
            (info->channleInfo[channel].mode == kFtmCombinedPWM));
    if((info->instance==1) || (info->instance == 2))
    {
      assert(channel < HW_CHAN2);
    }
    ftm_config_t config =
    {
        .mode= info->channleInfo[channel].mode,
        .edge_mode = info->channleInfo[channel].edge_mode,
        .channel = channel,
    };
     ftm_hal_disable_pwm_mode(info->instance, &config);
}

/*See fsl_ftm_driver.h for documentation of this function.*/
void ftm_pwm_configure(ftm_driver_info_t *info,uint8_t channel, ftm_pwm_param_t *param)
{

  uint16_t uMod, uCnv;

  if((info->instance==1) || (info->instance == 2))
  {
     assert(channel < HW_CHAN2);
  }
  /*to disable clock source to update mod, CnV and Cn+1V register*/
  ftm_hal_set_clock_source(info->instance,kClock_source_FTM_None);
  /*based on Ref manual, in PWM mode CNTIN is to be set 0*/
  info->counterInitVal = 0;
  uint32_t uFTMhz;
  if(info->clockSrc == kClock_source_FTM_SystemClk)
  {
#if FSL_FEATURE_FTM_BUS_CLOCK
      clock_manager_get_frequency(kBusClock, &uFTMhz);
#else
      clock_manager_get_frequency(kSystemClock, &uFTMhz);
#endif
      uFTMhz = uFTMhz/(1<<(info->clockPS));
  }
  else
  {
      uFTMhz = info->frequencyHZ/(1<<(info->clockPS));
  }

  switch(info->channleInfo[channel].mode)
  {
  case kFtmEdgeAlignedPWM:
     uMod = uFTMhz/(param->uFrequencyHZ) + info->counterInitVal - 1;
     if((info->channleInfo[channel]).edge_mode.ftm_pwm_edge_mode == kFtmHighTrue)
     {
         uCnv = uMod*param->uPulseHighPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage)+ info->counterInitVal;
     }
     else
     {
         uCnv = uMod*param->uPulseLowPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage) + info->counterInitVal;
     }
     ftm_hal_set_counter(info->instance, 2);
     ftm_hal_set_mod(info->instance, uMod);
     ftm_hal_set_counter_init_val(info->instance, info->counterInitVal);
     ftm_hal_set_channel_count_value(info->instance, channel, uCnv);
     assert(ftm_hal_get_mod(info->instance) == uMod);
     break;
  case kFtmCenterAlignedPWM:
     uMod = uFTMhz/(param->uFrequencyHZ*2) + info->counterInitVal;
     if((info->channleInfo[channel]).edge_mode.ftm_pwm_edge_mode == kFtmHighTrue)
     {
         uCnv = uMod*param->uPulseHighPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage)+ info->counterInitVal;
     }
     else
     {
         uCnv = uMod*param->uPulseLowPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage) + info->counterInitVal;
     }
     ftm_hal_set_counter(info->instance, 0);
     ftm_hal_set_mod(info->instance, uMod);
     ftm_hal_set_counter_init_val(info->instance, info->counterInitVal);
     ftm_hal_set_channel_count_value(info->instance, channel, uCnv);
     break;
  case kFtmCombinedPWM:
    /*for instance 1 and instance 2, only one channel pair*/
     if((info->instance==1) || (info->instance == 2))
     {
        assert(channel == 0);
     }
     uMod = uFTMhz/(param->uFrequencyHZ) + info->counterInitVal - 1;
     if((info->channleInfo[channel]).edge_mode.ftm_pwm_edge_mode == kFtmHighTrue)
     {
         uCnv = uMod*param->uPulseHighPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage);
     }
     else
     {
         uCnv = uMod*param->uPulseLowPercentage/(param->uPulseHighPercentage+param->uPulseLowPercentage);
     }
     ftm_hal_enable_dual_channel_combine(info->instance, channel, true);
     ftm_hal_set_counter(info->instance, 0);
     ftm_hal_set_mod(info->instance, uMod);
     ftm_hal_set_counter_init_val(info->instance, info->counterInitVal);
     ftm_hal_set_channel_count_value(info->instance, (2*channel), param->uCnV);
     ftm_hal_set_channel_count_value(info->instance, (2*channel+1), uCnv+param->uCnV);
     break;
  default:
    assert(0);
    break;
  }

}