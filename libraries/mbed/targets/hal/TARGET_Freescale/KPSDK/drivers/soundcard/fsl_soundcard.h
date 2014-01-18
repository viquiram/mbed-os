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
#include "fsl_sai_hal.h"

/*!
 * @addtogroup soundcard
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The macro defined the sai buffer size and period */
#define AUDIO_BUFFER_BLOCK_SIZE     64
#define AUDIO_BUFFER_BLOCK              2
#define AUDIO_BUFFER_SIZE         (AUDIO_BUFFER_BLOCK * AUDIO_BUFFER_BLOCK_SIZE)

#define USEDMA 1/*!< Use DMA mode or interrupt mode. */

/*! @brief Define the status of audio */
typedef enum _snd_status
{
    kStatus_SND_Success = 0,
    kStatus_SND_DmaFail = 1,
    kStatus_SND_Fail = 2
} snd_status_t;

/*! @brief Define the PCM data format */
typedef struct PCM_Audio_Format{

    uint32_t sample_rate;/*!< Sample rate of the PCM file */
    uint32_t mclk;/*!< Master clock frequency */
    uint8_t  bits;/*!< How many bits in a word, it can be 8, 10, 16, 24,32 */
    uint8_t  words;/*!< How many word in a frame */

} audio_data_format_t;

/*! @brief The description structure for the sai tx/rx module. */
typedef struct I2S_config
{
    uint32_t			count; /*!< Used to count the transferred data number. */
    sai_sync_mode_t	sync_mode;/*!< sychronous or asychronous. */
    sai_bus_t			bus_type;/*!< I2S left, I2S right or I2S type. */
    sai_master_slave_t	slave_master;/*!< Mater or slave. */
    sai_bclk_source_t	bclk_source;/*!< Bit clock from master clock or other modules. */
    audio_data_format_t format; /*!< The transferring audio data format. */
    uint8_t			watermark;/*!< Watermark to trigger a dma request or interrupt request. */
    uint8_t			channel;/*!< Which FIFO used to transfer. */
    /* dma define */
#if USEDMA
    edma_channel_t		 dma_channel;/*!< Which dma channel it uses */
    dma_request_source_t dma_source; /*!< Dma request source. */
    edma_software_tcd_t  stcd[5];    /*!< Temporily assign 4 tcds for configuration. */
#endif
}  i2s_config_t;

/*!
 * @brief SAI module description structure. An SAI module includes a tx and a rx module, tx and rx can 
 * work at the same time, and both use a master clock.
 */
typedef struct I2S_device_config
{
    i2s_config_t *tx_config;/*!< Tx configuration info. */
    i2s_config_t *rx_config;/*!< Rx configuration info. */    
    sai_mclk_source_t	mclk_source;/*!< Master clock source. */
    bool				mclk_divide_enable;/*!< Enable the divide of master clock to generate bit clock. */
    uint32_t			mclk;/*!< Mater clock frequency. */
}  i2s_device_config_t;

/*!
 * @brief Soundcard status includes the information which the application can see
 * This structure is the interface between the driver and the application, application can get the 
 * information about where and when to input/output data.
 */
 typedef struct Audio_Status
{
    uint32_t size; /*!< The size of a block */
    uint32_t empty_block; /*!< How many blocks are empty */
    uint32_t full_block; /*!< How many blocks are full */
    uint8_t *input_address; /*!< The input address */
    uint8_t *output_address; /*!< The output address */
    sync_object_t sem; /*!< The sems tells application when to copy data */
} audio_status_t;

/*!
 * @brief The operations of an audio controller, for example SAI, SSI and so on. 
 * The operation includes the basic initialize, configure, reand and write function.
 */
typedef struct audio_controller_operation
{
    snd_status_t (*ctrl_init)(void *param, audio_status_t *w_status, audio_status_t *r_status);/*!< Initialize the controller. */
    
    snd_status_t (*ctrl_deinit)(void *param);/*!< Deinit the controller. */
    
    snd_status_t (*ctrl_config)(void *param, void *config, bool direction);/*!< Configure the basic bus information. */
    
    snd_status_t (*ctrl_config_data_format)(void *param, audio_data_format_t *format, bool direction);/*!< Configure audio data format. */
    
    void (*ctrl_start_write)(void *param);/*!< Used in strat transfer or resume transfer*/

    void (*ctrl_start_read)(void *param);/*!< Used in start receive or resume receive */
    
    void (*ctrl_stop_write)(void *param);/*!< Used in stop transfer */

    void (*ctrl_stop_read)(void *param);/*!< Used in stop receive */

    void (*ctrl_clear_tx_status)(void *param);

    void (*ctrl_clear_rx_status)(void *param);
#if USEDMA
    void(*ctrl_tx_callback)(void *param, dma_channel_status_t status);
    void(*ctrl_rx_callback)(void *param, dma_channel_status_t status);
#else
    void(*ctrl_tx_callback)(void *param);
    void(*ctrl_rx_callback)(void *param);
#endif
} audio_ctrl_operation_t;

/*! @brief Audio codec operation structure. */
typedef struct audio_codec_operation
{
    snd_status_t (*codec_init)(void *param);
    snd_status_t (*codec_deinit)(void *param);
    snd_status_t (*codec_config)(void *param, void *config);
    snd_status_t (*codec_config_data_format)(void *param, audio_data_format_t *format);
} audio_codec_operation_t;

/*! @brief The definitation of the audio device which may be a controller. */
typedef struct Audio_Controller
{
    uint8_t* name;/*!< Name of controller */
    uint8_t instance;/*!< Controller instance */
    void* config; /*!< Audio device config structure including tx and rx configure */
    audio_ctrl_operation_t *ops;/*!< Operations including the initialize, configure etc*/
} audio_controller_t;

/*! @brief The codec structure. */
typedef struct Audio_Codec
{  
    uint8_t instance;/*!< Codec instance */
    void *config;/*!< The configuration inforamtion for codec device. */
    audio_codec_operation_t *ops;/*!< operations. */
}audio_codec_t;

/*!
 * @brief A sound card includes the audio controller and a codec. 
 */
typedef struct Soundcard
{
    uint8_t *name;/*!< Soundcard name. */
    audio_controller_t *controller;/*!< Controller. */ 
    audio_codec_t *codec;/*!< Codec. */
    bool isrun;
} sound_card_t;

/* @brief Alerady defined controller or codec instances. */
extern audio_controller_t g_sai0;
extern audio_controller_t g_sai1;
extern audio_codec_t g_sgtl5000;

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
 * @param card: Sound card pointer.
 * @return Return kStatus_Success while the initialize success and kStatus_fail if failed.
 */
snd_status_t snd_init(sound_card_t *card);

/*!
 * @brief Deinit the sound card instance.
 *
 * It would call the codec and controller deinit function and free the buffer controlled by
 * sound card.The function should be used at the end of the application, if just pause the
 * playback/record, you shouldn't use the function. 
 * @param card: Sound card pointer.
 * @return Return kStatus_Success while the initialize success and kStatus_fail if failed.
 */
snd_status_t snd_deinit(sound_card_t *card);

/*!
 * @brief Configure the sound card. This is the static configurations.
 *
 * This configure function is the config before running, it can be seen as an static
 * configure. This configure would configure the protocol or the slave/master mode
 * and so on. The data foramt (i.e. Sample rate) is not configured in this function,
 * it is in snd_data_format_configure().
 * @param card: Sound card pointer.
 * @param ctrl_config Controller configuration structure.
 * @param codec_config Codec configuration structure.
 * @param direction Configure rx or tx. 1 represents tx and 0 represents rx.
 * @return Return kStatus_Success while the initialize success and kStatus_fail if failed.
 */
snd_status_t snd_configure(sound_card_t *card, void *ctrl_config, void *codec_config, bool direction);

/*!
 * @brief Configure the audio data format play in sound card.
 *
 * This function can make the application change the data format during the run time.
 * This function can change the sample rate, bit depth(i.e. 16-bit).
 * @param card: Sound card pointer.
 * @param format Data format using in sound card.
 * @param direction Configure rx or tx. 1 represents tx and 0 represents rx.
 * @return Return kStatus_Success while the initialize success and kStatus_fail if failed.
 */
snd_status_t snd_data_format_configure(sound_card_t *card, audio_data_format_t *format, bool direction);

/*!
 * @brief Update the status of sai tx.
 *
 * The write operation would maintain the buffer state structure and also call
 * the write operation for the controller.The device function would use a callback
 * function to let the sound card operate the buffer state structure.
 * Notice: The function would not copy the data to sai buffer, this process need
 * the application do, and the driver can provide the status structure in card->tx_status,
 * in which tell the application the the start address of the buffer application
 * can write in (tx_status->input_address).
 * @param card Sound card pointer
 * @param len Data size of data to write.
 * @return The size which have been written.
 */
uint32_t snd_update_tx_status(sound_card_t *card, uint32_t len);

/*!
 * @brief Update status of sai rx.
 *
 * The sound card read function would call the controller operations.
 * The write operation would maintain the buffer state structure and also call the write operation for the controller.
 * The device function would use a callback function to let the sound card operate the buffer state structure.
 * Notice: The function would not copy the data from the buffer to data, this process need the application do, and the driver can
 * provide the status structure in card->rx_status, in which tell the application the the start address of the buffer application
 * can write in (tx_status->output_address).
 * @param card Sound card pointer.
 * @param len Data size of data to read.
 * @return The data size which have been read.
 */
uint32_t snd_update_rx_status(sound_card_t *card, uint32_t len);

/*!
 * @brief Tx Callback function to maintain the controller buffer. While the controller finished the transfer of a block,
 * it would callback.
 * @param param Audio controller structure.
 */
void snd_tx_callback(void *param);

/*!
 * @brief Rx Callback function to maintain the controller buffer. While the controller finished the transfer of a block,
 * it would callback.
 * @param param Audio controller structure.
 */
void snd_rx_callback(void *param);

#if USEDMA
/*!
 * @brief Tx Callback function to maintain the controller buffer in dma mode. While the controller finished the transfer of a block,
 * it would callback.
 * @param param Audio controller structure.
 */
static inline void snd_tx_dma_callback(void *param, dma_channel_status_t status)
{
    snd_tx_callback(param);
}

/*!
 * @brief Rx Callback function to maintain the controller buffer in dma mode. While the controller finished the transfer of a block,
 * it would callback.
 * @param param Audio controller structure.
 */
static inline void snd_rx_dma_callback(void *param, dma_channel_status_t status)
{
    snd_rx_callback(param);
}
#endif

/*!
 * @brief Get the status of sai.
 *
 * Every time the application want to write/read data from the sai buffer, application
 * should call this function to get the status of the sai. This function would copy
 * data to the @param status, from the structure, the user can get the information
 * about where to write/read data and how much data can be read/write.
 * @param card Soundcard pointer.
 * @param status Pointer of audio_status_t structure.
 * @param direction In tx or rx, true represents tx and false represents rx.
 */
void snd_get_status(audio_status_t *status, bool direction);

/*!
 * @brief Start sai TX process.
 *
 * This function is used to start the tx process of sai. This function would enable
 * dma/interrupt request source and enable tx and bit clock of tx. Note that this
 * function can be used both the beginning of sai transfer and also resume the transfer.
 */
static inline void snd_start_tx(sound_card_t *card)
{
    audio_controller_t *ctrl = card->controller;
    ctrl->ops->ctrl_start_write(ctrl);
}

/*!
 * @brief Start sai RX process.
 *
 * This function is used to start the rx process of sai. This function is the same
 * as the snd_start_tx, it would be used in the begining of sai transfer and also
 * resume rx.
 */
 static inline void snd_start_rx(sound_card_t *card)
{
    audio_controller_t *ctrl = card->controller;
    ctrl->ops->ctrl_start_read(ctrl);
}

/*!
 * @brief Stop sai tx process.
 *
 * This function is used to stop the trnasfer of sai tx. Note that this function
 * would not reset the inner logic of sai driver, it just disable the dma/interrupt
 * request source. So this function can be used as the pause of audio play.
 */
static inline void snd_stop_tx(sound_card_t *card)
{
    audio_controller_t *ctrl = card->controller;
    ctrl->ops->ctrl_stop_write(ctrl); 
}

/*!
 * @brief Stop sai rx process.
 *
 * This fucntion is the same as snd_stop_tx, it is for stopping rx process. This
 * function is also disable the dma/interrupt source.
 */
static inline void snd_stop_rx(sound_card_t *card)
{
     audio_controller_t *ctrl = card->controller;
     ctrl->ops->ctrl_stop_read(ctrl); 
}

/*!
 * @brief Wait the semaphore of write/read data from sai buffer.
 *
 * Application should call this function before write/read data from sai buffer.
 * Before application write data to sai buffer, sai buffer must have free space
 * for the new data, or it would cause data loss. This function would wait the
 * semaphore which represents there is free space in sai buffer.
 * The same as read data from sai buffer, there must have effective data in sai
 * buffer, this function can wait for that semaphore.
 * @param direction Tx or Rx, 1 represents Tx and 0 represents Rx. 
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

