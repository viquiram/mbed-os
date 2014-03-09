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


#include "fsl_soundcard.h"
#include <assert.h>
#include <string.h>
#include "fsl_os_abstraction.h"

/*******************************************************************************
 *Definitation
 ******************************************************************************/

#if USEDMA
void snd_tx_dma_callback(void *param, edma_channel_status_t status);
void snd_rx_dma_callback(void *param, edma_channel_status_t status);
#else
void snd_tx_callback(void *param);
void snd_rx_callback(void *param);
#endif

/* The instance for sai operation structure. */
audio_ctrl_operation_t g_sai_ops =
{
    sai_init,
    sai_deinit,
    sai_configure_data_format,
    sai_start_write_data,
    sai_start_read_data,
    sai_stop_write_data,
    sai_stop_read_data,
    sai_send_data,
    sai_receive_data,
    sai_register_callback,
    sai_get_fifo_address
};

/* Instance of codec operation for sgtl5000. */
audio_codec_operation_t g_sgtl_ops =
{
    sgtl_init,
    sgtl_deinit,
    sgtl_configure_data_format
};

/*******************************************************************************
 *Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : snd_init
* Description	: Initialize the soundcard.
*  The soundcard includes a controller and a codec.
*END**************************************************************************/
snd_status_t snd_init(sound_card_t * card, void * ctrl_config, void * codec_config)
{
    audio_controller_t *ctrl = &card->controller;
    audio_codec_t *codec = &card->codec;
    /* Allocate space for buffer */
    audio_buffer_t *buffer = &card->buffer;
    /* Buffer size and block settings */
    buffer->blocks = AUDIO_BUFFER_BLOCK;
    buffer->size = AUDIO_BUFFER_BLOCK_SIZE;
    buffer->buff = (uint8_t *)mem_allocate_zero(buffer->size * buffer->blocks);
    if(!buffer->buff)
    {
        return kStatus_SND_BufferAllocateFail;
    }
    buffer->input_curbuff = buffer->buff;
    buffer->output_curbuff = buffer->buff;
    /* Initialize the status structure */
    buffer->empty_block = buffer->blocks;
    buffer->full_block = 0;
#if USEDMA
    edma_request_channel(kEdmaAnyChannel, ctrl->dma_source, &ctrl->dma_channel);
#endif
    if(card->direction == AUDIO_TX)
    {
        sync_create(&buffer->sem, buffer->blocks); 
#if USEDMA
        edma_register_callback(&ctrl->dma_channel, snd_tx_dma_callback, (void *)card);
#endif
    }
    else
    {
        sync_create(&buffer->sem, 0);
#if USEDMA
        edma_register_callback(&ctrl->dma_channel, snd_rx_dma_callback, (void *)card);
#endif
     }
    /* Initialize audio controller and codec */
    ctrl->ops->ctrl_init((void *)(ctrl->handler), ctrl_config);
    codec->ops->codec_init((void *)codec->handler, codec_config);
    return kStatus_SND_Success;
}

/*FUNCTION*********************************************************************
*
* Function Name : snd_deinit
* Description	: Deinit the soundcard.
*  The soundcard includes a controller and a codec.
*END**************************************************************************/
snd_status_t snd_deinit(sound_card_t *card)
{
    audio_controller_t *ctrl = &card->controller;
    audio_codec_t *codec = &card->codec;
    audio_buffer_t *buffer = &card->buffer;
    /* Call the deinit function of the ctrl and codec. */
    ctrl->ops->ctrl_deinit((void *)(ctrl->handler));
    codec->ops->codec_deinit((void *)codec->handler);
#if USEDMA
    /* Deinit the dma resource */
    edma_stop_channel(&ctrl->dma_channel);
    edma_free_channel(&ctrl->dma_channel);
#endif
    sync_destroy(&buffer->sem);
    /* Free the tx and rx buffer. */
    mem_free(buffer->buff);
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_data_format_configure
* Description	: Configure the audio file format in the soundcard.
*  The soundcard includes a controller and a codec. The audio format includes 
*  sample rate, bit length and so on.
*END**************************************************************************/
snd_status_t snd_data_format_configure(sound_card_t *card, ctrl_data_format_t *format)
{
    audio_controller_t *ctrl = &card->controller;
    audio_codec_t *codec = &card->codec;

    ctrl->ops->ctrl_config_data_format(ctrl->handler, format);
    codec->ops->codec_config_data_format(codec->handler, format->mclk, format->sample_rate, 
                                                                    format->bits);
    /* Configure dma */
#if USEDMA
    audio_buffer_t *buffer = &card->buffer;
    if(card->direction == AUDIO_TX)
    {
        uint8_t sample_size = format->bits/8;
        if((sample_size == 3) || (format->bits & 0x7))
        {
            sample_size = 4;
        }
        uint32_t * desAddr = ctrl->ops->ctrl_get_fifo_address(ctrl->handler);
        edma_software_tcd_t * stcd_temp = (edma_software_tcd_t *)(((uint32_t )ctrl->stcd + 32) & 0xFFFFFFE0);
        edma_config_loop(
                stcd_temp, &ctrl->dma_channel,kEdmaMemoryToPeripheral,
                (uint32_t)buffer->buff, (uint32_t)desAddr, sample_size,
                (sample_size * (SAI_FIFO_LEN - format->watermark)), 
                AUDIO_BUFFER_SIZE, AUDIO_BUFFER_BLOCK);
        edma_start_channel(&ctrl->dma_channel);
    }
    else
    {
        uint8_t sample_size = format->bits/8;
        if((sample_size == 3) || (format->bits & 0x7))
        {
            sample_size = 4;
        }
        uint32_t * desAddr = ctrl->ops->ctrl_get_fifo_address(ctrl->handler);
        edma_software_tcd_t * stcd_temp = (edma_software_tcd_t *)(((uint32_t )ctrl->stcd + 32) & 0xFFFFFFE0);
        edma_config_loop(
                stcd_temp, &ctrl->dma_channel,kEdmaPeripheralToMemory,
                 (uint32_t)desAddr, (uint32_t)buffer->buff, sample_size,
                (sample_size *format->watermark), AUDIO_BUFFER_SIZE, AUDIO_BUFFER_BLOCK);
        edma_start_channel(&ctrl->dma_channel);
    }
#else
    if(card->direction == AUDIO_TX)
    {
        ctrl->ops->ctrl_register_callback(ctrl->handler, snd_tx_callback, card);
    }
    else
    {
        ctrl->ops->ctrl_register_callback(ctrl->handler, snd_rx_callback, card);
    }
#endif

    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_update_tx_status
* Description	: Write data from audio controller to audio codec.
* The function would maintain the buffer and call the audio controller interface
* to transfer the data to the codec. The audio controller would using callback 
* function to communicate.
*END**************************************************************************/
uint32_t snd_update_tx_status(sound_card_t * card, uint32_t len)
{
    audio_buffer_t * buffer = &card->buffer;
    uint32_t input_blocks = len/buffer->size;
    /* Update the buffer information */
    buffer->requested += len;
    buffer->queued += len;
    
    if(buffer->input_index + input_blocks < buffer->blocks)
    {
        buffer->input_index += input_blocks;
    }
    else
    {
        buffer->input_index = input_blocks - (buffer->blocks - 1 - buffer->input_index) - 1;
    }
    buffer->input_curbuff = buffer->buff + buffer->input_index * buffer->size;
    buffer->empty_block -= input_blocks;
    buffer->full_block += input_blocks;
    /* If sai is not enable, enable the sai */
    if (buffer->first_io)
    {
        buffer->first_io = false;
        snd_start_tx(card);
    }
    return len;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_tx_callback
* Description	: Callback function to tell that audio controller have finished 
* a period data.
* The function would update the buffer status information.
*END**************************************************************************/
void snd_tx_callback(void *param)
{
    sound_card_t *card = (sound_card_t *)param;
    audio_controller_t * ctrl = &card->controller;
    audio_buffer_t *buffer = &card->buffer;
    if(buffer->queued == 0)
    {
        return;
    }
    buffer->processed += buffer->size;
    buffer->queued -= buffer->size;

    /* Change the current buffer */
    if (buffer->output_index == buffer->blocks - 1)
    {
        buffer->output_curbuff = buffer->buff;
        buffer->output_index = 0;
    }
    else
    {
        buffer->output_index ++;
        buffer->output_curbuff += buffer->size;
    }
    /* Update the status */
    buffer->empty_block += 1;
    buffer->full_block -= 1;
    /* Judge if need to close the SAI transfer. */
    if (buffer->input_index == buffer->output_index)
    {
        ctrl->ops->ctrl_stop_write(ctrl->handler);
        buffer->buffer_error ++;
        buffer->first_io = true;
    }
#if !USEDMA
    ctrl->ops->ctrl_send_data(ctrl->handler, buffer->output_curbuff, buffer->size);
#endif
    /* post the sync */
    sync_signal_from_isr(&buffer->sem);
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_update_rx_status
* Description	: Read data from audio codec to audio controller.
* The function would maintain the buffer and call the audio controller interface
* to get data from the codec. The audio controller would using callback function
* to communicate.
*END**************************************************************************/
uint32_t snd_update_rx_status(sound_card_t *card,  uint32_t len)
{
    audio_buffer_t *buffer = &card->buffer;
    uint8_t output_blocks = len/buffer->size;
    /* Update inner information */
    buffer->requested += len;
    buffer->processed += len;
    buffer->queued -= len;
    /* Switch the buffer */
    if(buffer->output_index + output_blocks < buffer->blocks)
    {
        buffer->output_index += output_blocks;
    }
    else
    {
        buffer->output_index = output_blocks - (buffer->blocks - 1 - buffer->output_index) - 1;
    }
    buffer->output_curbuff = buffer->buff + buffer->output_index * buffer->size;
    buffer->full_block -= output_blocks;
    buffer->empty_block += output_blocks;
    /* The first time should start the progress. */
    if (buffer->first_io)
    {
        buffer->first_io = false;
        snd_start_rx(card);
    }
    return len;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_tx_callback
* Description	: Callback function to tell that audio controller have finished
*  a period data.
* The function would update the buffer status information.
*END**************************************************************************/
void snd_rx_callback(void *param)
{
    sound_card_t *card = (sound_card_t *)param;
    audio_controller_t *ctrl = &card->controller;
    audio_buffer_t *buffer = &card->buffer;
    buffer->queued += buffer->size;
    /* Change the current buffer. */
    if (buffer->input_index == buffer->blocks - 1)
    {  
        buffer->input_curbuff = buffer->buff;
        buffer->input_index = 0;
    }
    else
    {
        buffer->input_index ++;
        buffer->input_curbuff += buffer->size;
    }
    buffer->empty_block -= 1;
    buffer->full_block += 1;
    /* Judge if need to close the SAI transfer, while the buffer is full, 
     * we need to close the SAI */
    if (buffer->input_index == buffer->output_index)
    {
        ctrl->ops->ctrl_stop_read(ctrl->handler);
        buffer->buffer_error ++;
        buffer->first_io = true;
    }
#if !USEDMA
    ctrl->ops->ctrl_receive_data(ctrl->handler, buffer->input_curbuff, buffer->size);
#endif
    sync_signal_from_isr(&buffer->sem);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_get_status
 * Description   : Format the audio data to 32bits length complement before into FIFO
 *  
 *END**************************************************************************/
 void snd_get_status(snd_state_t *status, sound_card_t *card)
{
    audio_buffer_t *buffer = &card->buffer;
    status->size = buffer->size;
    status->empty_block = buffer->empty_block;
    status->full_block = buffer->full_block;
    status->input_address = buffer->input_curbuff;
    status->output_address = buffer->output_curbuff;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_wait_event
 * Description   : This function is used for appliaction to wait for the semaphore
 * to copy data in/out the sai buffer.
 *END**************************************************************************/
void snd_wait_event(sound_card_t *card)
{
    fsl_rtos_status syncStatus;
    audio_buffer_t *buffer = &card->buffer;
    do
    {
        syncStatus = sync_wait(&buffer->sem, kSyncWaitForever);
    }while(syncStatus == kIdle);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_start_tx
 * Description   : This function is used to start tx transfer.
 *END**************************************************************************/
void snd_start_tx(sound_card_t *card)
{
    audio_controller_t *ctrl = &card->controller;
#if !USEDMA
    audio_buffer_t *buffer = &card->buffer;
    ctrl->ops->ctrl_send_data(ctrl->handler, buffer->output_curbuff, buffer->size);
#endif
    ctrl->ops->ctrl_start_write(ctrl->handler);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_start_rx
 * Description   : This function is used to start rx transfer.
 *END**************************************************************************/
void snd_start_rx(sound_card_t *card)
{
    audio_controller_t *ctrl = &card->controller;
#if !USEDMA
    audio_buffer_t *buffer = &card->buffer;
    ctrl->ops->ctrl_receive_data(ctrl->handler, buffer->input_curbuff, buffer->size);
#endif
    ctrl->ops->ctrl_start_read(ctrl->handler);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_tx_dma_callback
 * Description   : This function is as the tx callback function registered to dma module.
 *END**************************************************************************/
void snd_tx_dma_callback(void *param, edma_channel_status_t status)
{
    snd_tx_callback(param);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_rx_dma_callback
 * Description   : This function is as the rx callback function registered to dma module.
 *END**************************************************************************/
void snd_rx_dma_callback(void *param, edma_channel_status_t status)
{
    snd_rx_callback(param);    
}

/*******************************************************************************
 *EOF
 ******************************************************************************/

