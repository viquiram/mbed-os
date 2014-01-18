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

/* The states for the ring buffer */
typedef struct Audio_Buffer{

    uint8_t	*buff;/* Buffer address */
    
    uint8_t	blocks;/* Block number of the buffer. */
    uint16_t	size;/* The size of a block */
    
    uint32_t	requested;/* The request data number to transfer */
    uint32_t	queued;/* Data which is in buffer, but not processed */
    uint32_t	processed;/* Data which is have been put into FIFO, 
                             this is used to judge if the sai is underrun */
    
    uint8_t	input_index;
    uint8_t	output_index;
    uint8_t*	input_curbuff;
    uint8_t*	output_curbuff;

    audio_status_t status;

    bool first_io;/* Means the first time the transfer */

    uint32_t fifo_error;
    uint32_t buffer_error;

}audio_buffer_t;

/* Two global buffers, the buffer can only be seen by the soundcard level. */
static audio_buffer_t *tx_buffer;
static audio_buffer_t *rx_buffer;

/*******************************************************************************
 *Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : snd_init_buffer
* Description	: Initialize the tx buffer and rx buffer, allocate space for them.
*  
*END**************************************************************************/
void snd_init_buffer(void)
{
    /* tx buffer */
    tx_buffer = (audio_buffer_t *)mem_allocate_zero(sizeof(audio_buffer_t));
    tx_buffer->blocks = AUDIO_BUFFER_BLOCK;
    tx_buffer->size = AUDIO_BUFFER_BLOCK_SIZE;
    tx_buffer->first_io = true;
    tx_buffer->buff = (uint8_t *)mem_allocate_zero((uint16_t)tx_buffer->size * tx_buffer->blocks);
    tx_buffer->input_curbuff = tx_buffer->buff;
    tx_buffer->output_curbuff = tx_buffer->buff;

    /* rx buffer */
    rx_buffer = (audio_buffer_t *)mem_allocate_zero(sizeof(audio_buffer_t));
    rx_buffer->blocks = AUDIO_BUFFER_BLOCK;
    rx_buffer->size = AUDIO_BUFFER_BLOCK_SIZE;
    rx_buffer->first_io = true;
    rx_buffer->buff = (uint8_t *)mem_allocate_zero((uint16_t)rx_buffer->blocks * rx_buffer->size);
    rx_buffer->input_curbuff = rx_buffer->buff;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_init
* Description	: Initialize the soundcard.
*  The soundcard includes a controller and a codec.
*END**************************************************************************/
snd_status_t snd_init(sound_card_t * card)
{
    /* Initialize the global buffer */
    snd_init_buffer();
    /* Initialize the controller and codec */
    audio_controller_t *ctrl = card->controller;
    audio_codec_t *codec = card->codec;
    audio_status_t *tx_status = &tx_buffer->status;
    audio_status_t *rx_status = &rx_buffer->status;
    /* Initialize the status staucture */
    tx_status->size = AUDIO_BUFFER_BLOCK_SIZE;
    tx_status->empty_block = AUDIO_BUFFER_BLOCK;
    tx_status->full_block = 0;
    tx_status->input_address = tx_buffer->buff;
    tx_status->output_address = tx_buffer->buff;
    sync_create(&tx_status->sem, AUDIO_BUFFER_BLOCK);
    rx_status->size = AUDIO_BUFFER_BLOCK_SIZE;
    rx_status->empty_block = AUDIO_BUFFER_BLOCK;
    rx_status->full_block = 0;
    rx_status->input_address = rx_buffer->buff;
    rx_status->output_address = rx_buffer->buff;

    sync_create(&rx_status->sem, 0);
    /* Transfer the callback function */
#if USEDMA
    ctrl->ops->ctrl_tx_callback = snd_tx_dma_callback;
    ctrl->ops->ctrl_rx_callback = snd_rx_dma_callback;
#else
    ctrl->ops->ctrl_tx_callback = snd_tx_callback;
    ctrl->ops->ctrl_rx_callback = snd_rx_callback;
#endif
    /* Initialize audio controller and codec */
    if(kStatus_SND_Success != ctrl->ops->ctrl_init((void *)ctrl, tx_status, rx_status))
    {
        return kStatus_SND_Fail;
    }
    if(kStatus_SND_Success != codec->ops->codec_init((void *)codec))
    {
        return kStatus_SND_Fail;
    }  
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
    audio_controller_t *ctrl = card->controller;
    audio_codec_t *codec = card->codec;
    /* Call the deinit function of the ctrl and codec. */
    ctrl->ops->ctrl_deinit((void *)ctrl);
    codec->ops->codec_deinit((void *)codec);
    /* Free the tx and rx buffer. */
    free(tx_buffer->buff);
    free(rx_buffer->buff);
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_configure
* Description	: Configure the soundcard before run.
*  The soundcard includes a controller and a codec.
*END**************************************************************************/
snd_status_t snd_configure(sound_card_t *card, void *ctrl_config, void *codec_config, bool direction)
{  
    card->controller->ops->ctrl_config((void *)card->controller,ctrl_config, direction);
    /* The codec static configuration is done in sgtl_init */
    card->codec->ops->codec_config((void *)card->codec,codec_config);
    
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : snd_data_format_configure
* Description	: Configure the audio file format in the soundcard.
*  The soundcard includes a controller and a codec. The audio format includes 
*  sample rate, bit length and so on.
*END**************************************************************************/
snd_status_t snd_data_format_configure(sound_card_t *card, audio_data_format_t *format, bool direction)
{
    audio_controller_t *ctrl = card->controller;
    audio_codec_t *codec = card->codec;

    ctrl->ops->ctrl_config_data_format((void *)ctrl, format, direction);
    codec->ops->codec_config_data_format((void *)codec, format);

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
    audio_buffer_t * buffer = tx_buffer;
    audio_controller_t *ctrl = card->controller;
    audio_status_t *status = &tx_buffer->status;
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

    /*Update the status */
    status->input_address = buffer->input_curbuff;
    status->empty_block -= input_blocks;
    status->full_block += input_blocks;
    /* If sai is not enable, enable the sai */
    if (buffer->first_io)
    {
        buffer->first_io = false;
        ctrl->ops->ctrl_start_write((void *)ctrl);
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
    audio_controller_t *ctrl = (audio_controller_t *)param;
    audio_status_t *status = &tx_buffer->status;
    if(tx_buffer->queued == 0)
    {
        return;
    }
    tx_buffer->processed += tx_buffer->size;
    tx_buffer->queued -= tx_buffer->size;

    /* Change the current buffer */
    if (tx_buffer->output_index == tx_buffer->blocks - 1)
    {
        tx_buffer->output_curbuff = tx_buffer->buff;
        tx_buffer->output_index = 0;
    }
    else
    {
        tx_buffer->output_index ++;
        tx_buffer->output_curbuff += tx_buffer->size;
    }
    /* Update the status */
    status->empty_block += 1;
    status->full_block -= 1;
    status->output_address = tx_buffer->output_curbuff;
    /* Judge if need to close the SAI transfer. */
    if (tx_buffer->input_index == tx_buffer->output_index)
    {
        ctrl->ops->ctrl_stop_write(param);
        tx_buffer->buffer_error ++;
        tx_buffer->first_io = true;
    }
    /* clear the status */
    ctrl->ops->ctrl_clear_tx_status(param);
    /* post the sync */
    sync_signal_from_isr(&status->sem);
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
    audio_controller_t *ctrl = card->controller;
    audio_status_t *status = &rx_buffer->status;
    audio_buffer_t *buffer = rx_buffer;
    uint8_t output_blocks = len/status->size;
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

    status->full_block -= output_blocks;
    status->empty_block += output_blocks;
    status->output_address = buffer->output_curbuff;
    /* The first time should start the progress. */
    if (buffer->first_io)
    {
        buffer->first_io = false;
        ctrl->ops->ctrl_start_read((void *)ctrl);
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
    audio_controller_t *ctrl = (audio_controller_t *)param;
    audio_status_t *status = &rx_buffer->status;
    rx_buffer->queued += rx_buffer->size;	
    /* Change the current buffer. */
    if (rx_buffer->input_index == rx_buffer->blocks - 1)
    {  
        rx_buffer->input_curbuff = rx_buffer->buff;
        rx_buffer->input_index = 0;
    }
    else
    {
        rx_buffer->input_index ++;
        rx_buffer->input_curbuff += rx_buffer->size;
    }
    /* Update the status */
    status->input_address = rx_buffer->input_curbuff;
    status->empty_block -= 1;
    status->full_block += 1;
    /* Judge if need to close the SAI transfer, while the buffer is full, 
     * we need to close the SAI */
    if (rx_buffer->input_index == rx_buffer->output_index)
    {
        ctrl->ops->ctrl_stop_read(param);
        rx_buffer->buffer_error ++;
        rx_buffer->first_io = true;
    }
    ctrl->ops->ctrl_clear_rx_status(param);
    sync_signal_from_isr(&status->sem);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_get_status
 * Description   : Format the audio data to 32bits length complement before into FIFO
 *  
 *END**************************************************************************/
 void snd_get_status(audio_status_t *status, bool direction)
{	
    if(direction)
    {
        *status = tx_buffer->status;
    }
    else
    {
        *status = rx_buffer->status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : snd_wait_event
 * Description   : This function is used for appliaction to wait for the semaphore
 * to copy data in/out the sai buffer.
 *END**************************************************************************/
void snd_wait_event(bool direction)
{
    fsl_rtos_status syncStatus;
    audio_status_t *status;
    if(direction)
    {
        status = &tx_buffer->status;
    }
    else
    {
        status = &rx_buffer->status;
    }
    do
    {
        syncStatus = sync_wait(&status->sem, kSyncWaitForever);
    }while(syncStatus == kIdle);
}

/*******************************************************************************
 *EOF
 ******************************************************************************/

