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


#ifndef __FSL_SAI_DRIVER_H__
#define __FSL_SAI_DRIVER_H__

#include "fsl_sai_hal.h"
#include "fsl_soundcard.h"

/*!
 * @addtogroup sai_driver
 * @{ 
 */

/*! @file */

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize sai module.
 *
 * This initialize is not the configuration, it only initialize the controller structure.
 * The function would be called by snd_init().
 * @param param Controller structure pointer of sai module.
 * @return Return kStatus_Success while the initialize success and kStatus_fail if failed.
 */
snd_status_t sai_init(void *param, audio_status_t *w_status, audio_status_t *r_status);

/*!
 * @brief De-initialize sai module.
 *
 * This function would be called by snd_deinit().
 * @param param Controller structure pointer of sai module.
 * @return Return kStatus_Success while the process success and kStatus_fail if failed.
 */
snd_status_t sai_deinit(void *param);

/*!
 * @brief Configure the sai module.
 *
 * Configure the paramater before playback/record, for example the bus mode, synchronous mode and so on.
 * @param param Controller structure pointer of sai module.
 * @param SAI configuration strcture pointer.
 * @return Return kStatus_Success while the process success and kStatus_fail if failed.
 */
snd_status_t sai_configure(void *param, void *device_config, bool direction);

/*!
 * @brief Configure the PCM data format.
 *
 * The function would configure mainly audio sample rate, data bits and channel number.
 * This function is called by snd_configure_data_format().
 * @param param Controller structure pointer of sai module.
 * @param format PCM data format structure pointer.
 * @param direction 1 means to configure tx, 0 means configure rx.
 * @return Return kStatus_Success while the process success and kStatus_fail if failed.
 */
snd_status_t sai_configure_data_format(void *param, audio_data_format_t *format, bool direction);

/*!
 * @brief Start to read data from codec. Mainly enable RE bit of RCSR.
 * @param param Controller structure pointer of sai module.
 */
 void sai_start_read_data(void *param);

/*!
 * @brief Start to write data to codec. Mainly enable TE bit of TCSR.
 * @param param Controller structure pointer of sai module.
 */
void sai_start_write_data(void *param);

/*!
 * @brief Stop read data form codec, mainly to disable dma or interrupt request bit.
 *
 * This function provides the method to pause receiving data.  
 * @param param Controller structure pointer of sai module.
 */
static inline void sai_stop_read_data(void *param)
{
    assert (param);
    audio_controller_t *ctrl = (audio_controller_t *)param;	
    sai_hal_disable_rx(ctrl->instance);
#if USEDMA
    sai_hal_disable_rx_dma(ctrl->instance, kSaiDmaReqFIFORequest);
    sai_hal_disable_rx_dma(ctrl->instance, kSaiDmaReqFIFOWarning);
    sai_hal_disable_rx_interrupt(ctrl->instance, kSaiIntrequestFIFOError);
#else	
    sai_hal_disable_rx_interrupt(ctrl->instance,kSaiIntrequestFIFORequest);
    sai_hal_disable_rx_interrupt(ctrl->instance,kSaiIntrequestFIFOWarning);
    sai_hal_disable_rx_interrupt(ctrl->instance,kSaiIntrequestFIFOError);	
#endif	
}

/*!
 * @brief Stop write data to codec, mainly to disable dma or interrupt request bit.
 *
 * This function provides the method to pause writing data.  
 * @param param Controller structure pointer of sai module.
 */
static inline void sai_stop_write_data(void *param)
{
    assert (param);
    audio_controller_t *ctrl = (audio_controller_t *)param;	
    sai_hal_disable_tx(ctrl->instance);
#if USEDMA
    sai_hal_disable_tx_dma(ctrl->instance, kSaiDmaReqFIFORequest);
    sai_hal_disable_tx_dma(ctrl->instance, kSaiDmaReqFIFOWarning);
    sai_hal_disable_tx_interrupt(ctrl->instance, kSaiIntrequestFIFOError);
#else	
    sai_hal_disable_tx_interrupt(ctrl->instance,kSaiIntrequestFIFORequest);
    sai_hal_disable_tx_interrupt(ctrl->instance,kSaiIntrequestFIFOWarning);	
#endif	
}

/*!
 * @brief Clear the word start flag and FIFO error flag.
 * @param param Controller structure pointer of sai module.
 */
static inline void sai_clear_tx_status(void *param)
{
    assert (param);
    audio_controller_t *ctrl = (audio_controller_t *)param;
    sai_hal_clear_tx_state_flag(ctrl->instance, kSaiStateFlagWordStart);
    sai_hal_clear_tx_state_flag(ctrl->instance, kSaiStateFlagFIFOError);
}

/*!
 * @brief Clear the word start flag and FIFO error flag.
 * @param param Controller structure pointer of sai module.
 */
static inline void sai_clear_rx_status(void *param)
{
     assert (param);
     audio_controller_t *ctrl = (audio_controller_t *)param;
     sai_hal_clear_rx_state_flag(ctrl->instance, kSaiStateFlagWordStart);
     sai_hal_clear_rx_state_flag(ctrl->instance, kSaiStateFlagFIFOError);
}

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif/* __FSL_SAI_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

