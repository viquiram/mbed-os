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

#ifndef __FSL_SOUNDCARD_H__
#define __FSL_SOUNDCARD_H__

#include "fsl_os_abstraction.h"
#include "fsl_edma_driver.h"
#include "fsl_sai_driver.h"
#include "fsl_sgtl5000_driver.h"

/*!
 * @addtogroup soundcard
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define AUDIO_CONTROLLER    AUDIO_CONTROLLER_SAI /*!< Define audio controller sai */
#define AUDIO_CONTROLLER_SAI    1

#define AUDIO_CODEC AUDIO_CODEC_SGTL5000 /*!< Define audio codec sgtl5000 */
#define AUDIO_CODEC_SGTL5000    1

#define AUDIO_BUFFER_BLOCK_SIZE     128 /*!< Buffer block size setting */
#define AUDIO_BUFFER_BLOCK              2 /*!< Buffer block number setting */
#define AUDIO_BUFFER_SIZE         (AUDIO_BUFFER_BLOCK * AUDIO_BUFFER_BLOCK_SIZE)

#define AUDIO_TX    1 /*!< Audio transfer direction Tx */
#define AUDIO_RX    0 /*!< Audio transfer direction Rx */


#if AUDIO_CONTROLLER == AUDIO_CONTROLLER_SAI
typedef  sai_handler_t ctrl_handler_t;
typedef sai_config_t ctrl_config_t;
typedef sai_data_format_t ctrl_data_format_t;
typedef sai_callback_t ctrl_callback_t;
typedef sai_status_t ctrl_status_t;
#endif

#if AUDIO_CODEC == AUDIO_CODEC_SGTL5000
typedef sgtl_handler_t codec_handler_t;
typedef sgtl_status_t codec_status_t;
#endif

/*! @brief Soundcard return status */
typedef enum _snd_status
{
    kStatus_SND_Success = 0U, /*!< Execute successfully*/
    kStatus_SND_Fail = 1U, /*!< Execute fail */
    kStatus_SND_DmaFail = 2U, /*!< Dma operation fail */
    kStatus_SND_CtrlFail = 3U, /*!< Audio controller operation fail */
    kStatus_SND_CodecFail = 4U, /*!< Audio codec operation fail */
    kStatus_SND_BufferAllocateFail = 5U /*!< Buffer allocate failure */
} snd_status_t;

/*!
 * @brief Soundcard status includes the information which the application can see
 * This structure is the interface between the driver and the application, application can get the 
 * information about where and when to input/output data.
 */
 typedef struct SoundcardState
{
    uint32_t size; /*!< The size of a block */
    uint32_t empty_block; /*!< How many blocks are empty */
    uint32_t full_block; /*!< How many blocks are full */
    uint8_t *input_address; /*!< The input address */
    uint8_t *output_address; /*!< The output address */
    sync_object_t sem; /*!< The sems tells application when to copy data */
} snd_state_t;

/*!
 * @brief The operations of an audio controller, for example SAI, SSI and so on. 
 * The operation includes the basic initialize, configure, send, receive and so on.
 */
typedef struct AudioControllerOperation
{
    ctrl_status_t (*ctrl_init)(ctrl_handler_t *param, ctrl_config_t * config);/*!< Initialize the controller. */
    
    ctrl_status_t (*ctrl_deinit)(ctrl_handler_t  *param);/*!< Deinit the controller. */
    
    ctrl_status_t (*ctrl_config_data_format)(ctrl_handler_t  *param, ctrl_data_format_t *format);/*!< Configure audio data format. */
    
    void (*ctrl_start_write)(ctrl_handler_t  *param);/*!< Used in strat transfer or resume transfer*/

    void (*ctrl_start_read)(ctrl_handler_t  *param);/*!< Used in start receive or resume receive */
    
    void (*ctrl_stop_write)(ctrl_handler_t  *param);/*!< Used in stop transfer */

    void (*ctrl_stop_read)(ctrl_handler_t  *param);/*!< Used in stop receive */

    uint32_t (*ctrl_send_data)(ctrl_handler_t  *param, uint8_t *addr, uint32_t len); /*!< Send data function*/

    uint32_t (*ctrl_receive_data)(ctrl_handler_t  *param, uint8_t *addr, uint32_t len); /*!< Receive data*/

    void (*ctrl_register_callback)(ctrl_handler_t  *param, ctrl_callback_t callback, void *callback_param); /*!< Register callback function. */

    uint32_t *(*ctrl_get_fifo_address)(ctrl_handler_t  *param); /*!< Get fifo address */

} audio_ctrl_operation_t;

/*! @brief Audio codec operation structure. */
typedef struct AudioCodecOperation
{
    codec_status_t (*codec_init)(codec_handler_t *param, void *config); /*!< Codec initialize function*/
    codec_status_t (*codec_deinit)(codec_handler_t  *param); /*!< Codec deinit function */
    codec_status_t (*codec_config_data_format)(codec_handler_t  *param, 
        uint32_t mclk, uint32_t sample_rate, uint8_t bits ); /*!< Configure data format. */
} audio_codec_operation_t;


/*! @brief The definitation of the audio device which may be a controller. */
typedef struct AudioController
{
    void *handler;
#if USEDMA
    edma_channel_t       dma_channel;/*!< Which dma channel it uses */
    dma_request_source_t dma_source; /*!< Dma request source. */
    edma_software_tcd_t  stcd[5];    /*!< Temporily assign 5 tcds for configuration. */
#endif
    audio_ctrl_operation_t *ops;/*!< Operations including the initialize, configure etc*/
} audio_controller_t;

/*! @brief The codec structure. */
typedef struct AudioCodec
{  
    void *handler;/*!< Codec instance */
    audio_codec_operation_t *ops;/*!< operations. */
}audio_codec_t;

/*!
 * @brief A sound card includes the audio controller and a codec. 
 */
typedef struct Soundcard
{
    audio_controller_t controller;/*!< Controller. */ 
    audio_codec_t codec;/*!< Codec. */
} sound_card_t;

extern audio_ctrl_operation_t g_sai_ops;
extern audio_codec_operation_t g_sgtl_ops;
/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the sound card. The function would initialize the controller and the codec.
 *
 * The function would initialize the generic layer structure, for example the buffer and the
 * status structure, then the function would call the initialize functions of controller and codec.
 * @param card Soundcard pointer.
 * @param ctrl_config The configuration structure of audio controller.
 * @param codec_config The configuration structure of audio codec.
 * @param direction Tx or Rx, can be AUDIO_TX or AUDIO_RX.
 * @return Return kStatus_SND_Success while the initialize success and kStatus_SND_fail if failed.
 */
snd_status_t snd_init(sound_card_t *card, void * ctrl_config, void * codec_config, bool direction);

/*!
 * @brief Deinit the sound card instance.
 *
 * It would call the codec and controller deinit function and free the buffer controlled by
 * sound card.The function should be used at the end of the application, if just pause the
 * playback/record, you shouldn't use the function, you should use snd_stop_tx/snd_stop_rx
 * instead.
 * @param card Soundcard pointer.
 * @param direction Tx or Rx, can be AUDIO_TX or AUDIO_RX. 
 * @return Return kStatus_SND_Success while the initialize success and kStatus_SND_fail if failed.
 */
snd_status_t snd_deinit(sound_card_t *card, bool direction);

/*!
 * @brief Configure the audio data format running in sound card.
 *
 * This function can make the application change the data format during the run time.
 * But this fucntion cannot be called while TCSR.TE or RCSR.RE is enable.
 * This function can change the sample rate, bit depth(i.e. 16-bit).
 * @param card Soundcard pointer.
 * @param format Data format using in sound card.
 * @param direction Tx or Rx, can be AUDIO_TX or AUDIO_RX.
 * @return Return kStatus_SND_Success while the initialize success and kStatus_SND_fail if failed.
 */
snd_status_t snd_data_format_configure(sound_card_t *card, ctrl_data_format_t *format, bool direction);

/*!
 * @brief Update the status of sai tx.
 *
 * This function should be called after application copied data into buffer provided
 * by soundcard. Soundcard would not help users to copy data to internal buffer,this
 * operation should be done by applications. Soundcard would provide an interface
 * snd_get_status() for appliaction to get the inforamtion about the internal buffer
 * status, including the starting address and empty blocks and so on.
 * @param card Soundcard pointer
 * @param len Data size of data to write.
 * @return The size which have been written.
 */
uint32_t snd_update_tx_status(sound_card_t *card, uint32_t len);

/*!
 * @brief Update status of sai rx.
 *
 * This function should be called after application copied data from the buffer provided
 * by soundcard. Soundcard would not help apllications to copy data out from internal
 * buffer. This operation should be done by applications. Souncard would provide an
 * interface snd_get_status() for applications to get the inforamtion about the internal
 * buffer status, including the starting address and full blocks and so on.
 * @param card Soundcard pointer.
 * @param len Data size of data to read.
 * @return The data size which have been read.
 */
uint32_t snd_update_rx_status(sound_card_t *card, uint32_t len);

/*!
 * @brief Get the status of sai.
 *
 * Every time the application want to write/read data from the internal buffer, application
 * should call this function to get the status of the internal buffer. This function 
 * would copy data to the @param status, from the structure, the user can get the information
 * about where to write/read data and how much data can be read/write.
 * @param card Soundcard pointer.
 * @param status Pointer of audio_status_t structure.
 * @param direction Tx or rx, can be AUDIO_TX or AUDIO_RX.
 */
void snd_get_status(snd_state_t *status, bool direction);

/*!
 * @brief Start soundcard TX process.
 *
 * This function is used to start the tx process of soundcard. This function would enable
 * dma/interrupt request source and enable tx and bit clock of tx. Note that this
 * function can be used both the beginning of sai transfer and also resume the transfer.
 * @param card Soundcard pointer.
 */
void snd_start_tx(sound_card_t *card);

/*!
 * @brief Start soundcard RX process.
 *
 * This function is used to start the rx process of soundcard. This function is the same
 * as the snd_start_tx, it would be used in the begining of soundcard rx transfer and also
 * resume rx.
 * @param card Soundcard pointer.
 */
void snd_start_rx(sound_card_t *card);

/*!
 * @brief Stop soundcard tx process.
 *
 * This function is used to stop the trnasfer of soundcard tx. Note that this function
 * would not close audio controller, it just disable the dma/interrupt request source.
 * So this function can be used as the pause of audio play.
 * @param card Soundcard pointer.
 */
static inline void snd_stop_tx(sound_card_t *card)
{
    audio_controller_t *ctrl = &card->controller;
    ctrl->ops->ctrl_stop_write(ctrl->handler); 
}

/*!
 * @brief Stop soundcard rx process.
 *
 * This fucntion is the same as snd_stop_tx, it is for stopping rx process. This
 * function is also disable the dma/interrupt source.
 * @param card Soundcard pointer.
 */
static inline void snd_stop_rx(sound_card_t *card)
{
     audio_controller_t *ctrl = &card->controller;
     ctrl->ops->ctrl_stop_read(ctrl->handler); 
}

/*!
 * @brief Wait the semaphore of write/read data from internal buffer.
 *
 * Application should call this function before write/read data from soundcard buffer.
 * Before application write data to soundcard buffer, the buffer must have free space
 * for the new data, or it would cause data loss. This function would wait the
 * semaphore which represents there is free space in soundcard buffer.
 * The same as read data from soundcacrd buffer, there must have effective data in the
 * buffer, this function can wait for that semaphore.
 * @param direction Tx or rx, can be AUDIO_TX or AUDIO_RX. 
 */
void snd_wait_event(bool direction);


#if defined(__cplusplus)
extern "C" }
#endif

/*! @} */

#endif  /* __FSL_SOUNDCARD_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/

