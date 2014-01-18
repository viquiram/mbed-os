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

#include "fsl_sai_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"


/*******************************************************************************
 *Definitation
 ******************************************************************************/

#define SAI_TX_VECTOR_PRIORITY (2)
#define SAI_RX_VECTOR_PRIORITY (2)

extern IRQn_Type sai_irq_ids[HW_ADC_INSTANCE_COUNT][2];

/* The instance for sai operation structure. */
audio_ctrl_operation_t g_sai_ops =
{
    sai_init,
    sai_deinit,
    sai_configure,
    sai_configure_data_format,
    sai_start_write_data,
    sai_start_read_data,
    sai_stop_write_data,
    sai_stop_read_data,
    sai_clear_tx_status,
    sai_clear_rx_status,
    NULL,/* The callback function should be transferred from the generic layer */
    NULL
};

/* Instance for sai0. */
audio_controller_t g_sai0 = 
{
    .name = "sai0",
    .instance = 0,
    .config = NULL,
    .ops = &g_sai_ops
};

#if defined (K70F12_SERIES)
/* Instance for sai1. */
audio_controller_t g_sai1 = 
{
    .name = "sai1",
    .instance = 1,
    .config = NULL,
    .ops = &g_sai_ops
};
#endif

static audio_status_t tx_status;
static audio_status_t rx_status;
static uint8_t *tx_address_start;
static uint8_t *rx_address_start;
static uint8_t *tx_address_end;
static uint8_t *rx_address_end;
#if !USEDMA
static uint32_t tx_count;
static uint32_t rx_count;
#endif
audio_controller_t g_sai[HW_I2S_INSTANCE_COUNT];

/*******************************************************************************
 * Internal API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* The interrupt handle function of sai. */
#if USEDMA
void sai_tx_error_handle(uint32_t instance);
void sai_rx_error_handle(uint32_t instance);
#else
void sai_tx_interrupt_handle(uint32_t instance);
void sai_rx_interrupt_handle(uint32_t instance);
#endif


#if defined(__cplusplus)
}
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
 
/*FUNCTION**********************************************************************
 *
 * Function Name : sai_init
 * Description   : Initialize the sai module, and install the interrupt handle.
 *
 *END**************************************************************************/
snd_status_t sai_init(void *param, audio_status_t *w_status, audio_status_t *r_status)
{
    audio_controller_t *ctrl = (audio_controller_t *)param;
    /* Open clock gate for sai instance */
    clock_manager_set_gate(kClockModuleSAI, ctrl->instance, true);
    /* Initialize the g_sai*/
    g_sai[0] = g_sai0;
#if defined (K70F12_SERIES)
    g_sai[1] = g_sai1;
#endif
    /* Call the HAL function */
    sai_hal_init(ctrl->instance);
#if USEDMA
    edma_init();
#endif
    /* Init the status */
    tx_status = *w_status;
    rx_status = *r_status;
    tx_address_start = tx_status.output_address;
    rx_address_start = rx_status.input_address;
    tx_address_end = tx_status.output_address + tx_status.size * tx_status.empty_block;
    rx_address_end = rx_status.input_address + rx_status.size * rx_status.empty_block;
    /* Initialize the irq */
    NVIC_SetPriority(sai_irq_ids[ctrl->instance][0], SAI_TX_VECTOR_PRIORITY);
    NVIC_EnableIRQ(sai_irq_ids[ctrl->instance][0]);
    
    NVIC_SetPriority(sai_irq_ids[ctrl->instance][1], SAI_RX_VECTOR_PRIORITY);
    NVIC_EnableIRQ(sai_irq_ids[ctrl->instance][1]);
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_deinit
 * Description   :Deinit the sai module, free the resources.
 * 
 *END**************************************************************************/
snd_status_t sai_deinit(void *param)
{
    audio_controller_t *ctrl = (audio_controller_t *)param;
    sai_hal_disable_tx(ctrl->instance);
    sai_hal_reset_tx(ctrl->instance,kSaiResetTypeSoftware);
    sai_hal_disable_rx(ctrl->instance);
    sai_hal_reset_rx(ctrl->instance,kSaiResetTypeSoftware);
#if USEDMA
    i2s_device_config_t *config = ctrl->config;
    i2s_config_t *tx_config = config->tx_config;
    i2s_config_t *rx_config = config->rx_config;	
    edma_stop_channel(&tx_config->dma_channel);
    edma_free_channel(&tx_config->dma_channel);
    edma_stop_channel(&rx_config->dma_channel); 	
    edma_free_channel(&rx_config->dma_channel);
#endif
    clock_manager_set_gate(kClockModuleSAI, ctrl->instance, false);
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_configure
 * Description   :Configure the sai register before sai start.
 * 
 *END**************************************************************************/
snd_status_t sai_configure(void *param, void *device_config, bool direction)
{
    audio_controller_t *ctrl = (audio_controller_t *)param;
    ctrl->config = device_config;
    i2s_device_config_t * config = (i2s_device_config_t *)device_config;
    i2s_config_t *tx_config = config->tx_config;
    i2s_config_t *rx_config = config->rx_config;
    /* Set the master clock source */
    sai_hal_set_mclk_source(ctrl->instance, config->mclk_source);
    
    if (direction)
    {
        sai_hal_set_tx_mode(ctrl->instance, kSaiRunModeDebug);
        sai_hal_set_tx_sync_mode(ctrl->instance,tx_config->sync_mode);
        sai_hal_set_tx_master_slave(ctrl->instance,tx_config->slave_master);
        sai_hal_set_tx_bus(ctrl->instance,tx_config->bus_type);
        sai_hal_set_tx_watermark(ctrl->instance,tx_config->watermark);
        sai_hal_set_tx_fifo_channel(ctrl->instance,tx_config->channel);
    }
    else
    {
        sai_hal_set_rx_mode(ctrl->instance, kSaiRunModeDebug);
        sai_hal_set_rx_sync_mode(ctrl->instance,rx_config->sync_mode);
        sai_hal_set_rx_master_slave(ctrl->instance,rx_config->slave_master);
        sai_hal_set_rx_bus(ctrl->instance,rx_config->bus_type);
        sai_hal_set_rx_watermark(ctrl->instance,rx_config->watermark);
        sai_hal_set_rx_fifo_channel(ctrl->instance,rx_config->channel);
    }
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_configure_data_format
 * Description   :Configure audio format information.
 * The audio format information includes the sample rate, data length and so on.
 *END**************************************************************************/
snd_status_t sai_configure_data_format(void *param, audio_data_format_t *format, bool direction)
{
    uint32_t frequency = 0;
    audio_controller_t *ctrl = (audio_controller_t *)param;
    i2s_device_config_t *device_config = (i2s_device_config_t *)ctrl->config;
    i2s_config_t *tx_config = device_config->tx_config;
    i2s_config_t *rx_config = device_config->rx_config;
    uint32_t bclk = format->sample_rate * format->bits * format->words;
    format->mclk = format->sample_rate * 384;
    uint8_t divider = format->mclk/bclk;
     /* Get the clock source frequency */
     clock_manager_get_frequency(kCoreClock, &frequency);
    /* Configure master clock */
    sai_hal_set_mclk_divider(ctrl->instance, format->mclk, frequency);
    if (direction)
    {
        /* Modify the tx_config */
        tx_config->format = *format;	
        /* Master clock and bit clock setting */
        sai_hal_set_tx_bclk_source(ctrl->instance,kSaiBclkSourceMclkDiv);
        sai_hal_set_tx_blck_divider(ctrl->instance,divider);
        sai_hal_set_tx_sync_width(ctrl->instance,format->bits);
        /* Frmae size and word size setting */
        sai_hal_set_tx_frame_size(ctrl->instance,format->words);
        sai_hal_set_tx_word_size(ctrl->instance,format->bits);
        sai_hal_set_tx_word_zero_size(ctrl->instance,format->bits);
        sai_hal_set_tx_word_start_index(ctrl->instance,0);
        sai_hal_set_tx_fbt(ctrl->instance,format->bits - 1);
        /* The chennl number configuration */
        if (format->words == 1)
        {
            sai_hal_set_tx_word_mask(ctrl->instance, 0x2);
        }
        else
        {
            sai_hal_set_tx_word_mask(ctrl->instance, 0x0);
        }
#if USEDMA
        uint8_t sample_size = format->bits/8;
        if((sample_size == 3) || (format->bits & 0x7))
        {
            sample_size = 4;
        }
        edma_request_channel(kDmaAnyChannel, tx_config->dma_source, &tx_config->dma_channel);
        edma_register_callback(&tx_config->dma_channel, ctrl->ops->ctrl_tx_callback, (void *)ctrl);
        uint32_t * desAddr = sai_hal_get_fifo_address(ctrl->instance,kSaiIOModeTransmit,tx_config->channel);
        edma_software_tcd_t * stcd_temp = (edma_software_tcd_t *)(((uint32_t )tx_config->stcd + 32) & 0xFFFFFFE0);
        edma_config_loop(
                stcd_temp, &tx_config->dma_channel,kDmaMemoryToPeripheral,
                (uint32_t)tx_address_start, (uint32_t)desAddr, sample_size,
                (sample_size * (SAI_FIFO_LEN - tx_config->watermark)), 
                tx_address_end - tx_address_start, AUDIO_BUFFER_BLOCK);
        edma_start_channel(&tx_config->dma_channel);
#endif
    }
    else
    {
        rx_config->format = *format;
        /* Bit clock setting */
        sai_hal_set_rx_bclk_source(ctrl->instance,kSaiBclkSourceMclkDiv);
        sai_hal_set_rx_blck_divider(ctrl->instance,divider);
        sai_hal_set_rx_sync_width(ctrl->instance,format->bits);
        
        /* Frmae size and word size setting */
        sai_hal_set_rx_frame_size(ctrl->instance,format->words);
        sai_hal_set_rx_word_size(ctrl->instance,format->bits);
        sai_hal_set_rx_word_zero_size(ctrl->instance,format->bits);
        sai_hal_set_rx_word_start_index(ctrl->instance,0);
        sai_hal_set_rx_fbt(ctrl->instance,format->bits - 1);
        /* The chennl number configuration */
        if (format->words == 1)
        {
            sai_hal_set_rx_word_mask(ctrl->instance, 0x2);
        }
        else
        {
            sai_hal_set_rx_word_mask(ctrl->instance, 0x0);
        }
#if USEDMA
        uint8_t sample_size = format->bits/8;
        if((sample_size == 3) || (format->bits & 0x7))
        {
            sample_size = 4;
        }
        edma_request_channel(kDmaAnyChannel, rx_config->dma_source, &rx_config->dma_channel);
        edma_register_callback(&rx_config->dma_channel, ctrl->ops->ctrl_rx_callback, (void *)ctrl); 	
        uint32_t * srcAddr = sai_hal_get_fifo_address(ctrl->instance,kSaiIOModeReceive,rx_config->channel); 
        edma_software_tcd_t * stcd_temp = (edma_software_tcd_t *)(((uint32_t )rx_config->stcd + 32) & 0xFFFFFFE0);
        edma_config_loop(stcd_temp, &rx_config->dma_channel,kDmaPeripheralToMemory,\
                        (uint32_t)srcAddr, (uint32_t)rx_address_start, sample_size,
                         sample_size * rx_config->watermark, rx_address_end -rx_address_start,
                         AUDIO_BUFFER_BLOCK);
        edma_start_channel(&rx_config->dma_channel);
#endif
    }
    return kStatus_SND_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_start_write_data
 * Description   : Start the writing process.
 * 
 *END**************************************************************************/
void sai_start_write_data(void *param)
{
    audio_controller_t *ctrl = (audio_controller_t *)param;
    i2s_device_config_t *device_config = (i2s_device_config_t *)ctrl->config;
    i2s_config_t *config = device_config->tx_config;
#if USEDMA
    sai_hal_enable_tx_dma(ctrl->instance, kSaiDmaReqFIFORequest);
    sai_hal_enable_tx_dma(ctrl->instance, kSaiDmaReqFIFOWarning);
    /* Enable the FIFO error interrupt */
    sai_hal_enable_tx_interrupt(ctrl->instance, kSaiIntrequestFIFOError);
#else
    sai_hal_enable_tx_interrupt(ctrl->instance,kSaiIntrequestFIFORequest);
#endif
    sai_hal_enable_tx(ctrl->instance);
    /* If the sync mode is synchronous, it will need Rx enable bit clock */
    if(config->sync_mode == kSaiModeSync)
    {
        sai_hal_enable_rx(ctrl->instance);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_start_read_data
 * Description   : Start the reading process.
 * 
 *END**************************************************************************/
void sai_start_read_data(void *param)
{
    audio_controller_t *ctrl = (audio_controller_t *)param;
    i2s_device_config_t *device_config = (i2s_device_config_t *)ctrl->config;
    i2s_config_t *config = device_config->rx_config;
#if USEDMA
    sai_hal_enable_rx_dma(ctrl->instance, kSaiDmaReqFIFORequest);
    sai_hal_enable_rx_dma(ctrl->instance, kSaiDmaReqFIFOWarning);
    /* Enable the FIFO error interrupt */
    sai_hal_enable_rx_interrupt(ctrl->instance, kSaiIntrequestFIFOError);	
#else
    sai_hal_enable_rx_interrupt(ctrl->instance,kSaiIntrequestFIFORequest);
#endif
    sai_hal_enable_rx(ctrl->instance);
    /* If the sync mode is synchronous, it will need Tx enable bit clock */
    if(config->sync_mode == kSaiModeSync)
    {
        sai_hal_enable_tx(ctrl->instance);
    }
}

#if USEDMA
/*FUNCTION**********************************************************************
 *
 * Function Name : sai_tx_error_handle
 * Description   : The interrupt handle of tx FIFO error.
 * 
 *END**************************************************************************/
void sai_tx_error_handle(uint32_t instance)
{
    /* Clear FIFO error */
    sai_hal_reset_tx(instance, kSaiResetTypeFIFO);
    sai_hal_clear_tx_state_flag(instance, kSaiStateFlagFIFOError);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_rx_error_handle
 * Description   : The interrupt handle of rx FIFO error.
 * 
 *END**************************************************************************/
void sai_rx_error_handle(uint32_t instance)
{
    sai_hal_reset_rx(instance, kSaiResetTypeFIFO);
    sai_hal_clear_rx_state_flag(instance, kSaiStateFlagFIFOError);	
}

#else
/*FUNCTION**********************************************************************
 *
 * Function Name : sai_tx_interrupt_handle
 * Description   : The interrupt handle of tx FIFO request or FIFO warning.
 * The interrupt handle is used to transfer data from sai buffer to sai fifo. 
 *END**************************************************************************/
void sai_tx_interrupt_handle(uint32_t instance)
{
    audio_controller_t *sai = &g_sai[instance];
    i2s_device_config_t *device_config = sai->config;
    i2s_config_t *config = device_config->tx_config;
    audio_data_format_t *format = &config->format;
    uint8_t data_size = 0;
    uint8_t i = 0, j = 0;
    uint8_t space = FSL_FEATURE_I2S_FIFO_COUNT - config->watermark;
    uint32_t data = 0, temp = 0;	

    data_size = format->bits/8;
    if((data_size == 3) || (format->bits & 0x7))
    {
        data_size = 4;
    }

    /* Judge if FIFO error */
    if(sai_hal_fifo_error_is_detected(instance, kSaiIOModeTransmit))
    {
        sai_hal_reset_tx(instance, kSaiResetTypeFIFO);
    }
    /*Judge if the data need to transmit is less than space */
    if(space > ( AUDIO_BUFFER_BLOCK_SIZE - tx_count)/data_size)
    {
        space = (AUDIO_BUFFER_BLOCK_SIZE - tx_count)/data_size;
    }
    /* If normal, copy the data from sai buffer to FIFO */
    for(i = 0; i < space; i++)
    {
        for(j = 0; j < data_size; j ++)
        {
            temp = (uint32_t)(*tx_status.output_address);
            data |= (temp << (8U * j));
            tx_status.output_address ++;
        }
        sai_hal_transmit_data(instance,config->channel,(uint32_t )data);
        tx_count += data_size;
    }
    /* clear the WSF */
    sai_hal_clear_tx_state_flag(instance, kSaiStateFlagWordStart);
    sai_hal_clear_tx_state_flag(instance, kSaiStateFlagFIFOError);
    /* If a block is finished, just callback */
    if(tx_count >= AUDIO_BUFFER_BLOCK_SIZE)
    {
        tx_count = 0;
        if(tx_status.output_address >= tx_address_end)
        {
            tx_status.output_address = tx_address_start;
        }
        sai->ops->ctrl_tx_callback((void *)sai);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_rx_interrupt_handle
 * Description   : The interrupt handle of rx FIFO request or FIFO warning.
 * The interrupt handle is used to transfer data from sai fifo to sai buffer. 
 *END**************************************************************************/
void sai_rx_interrupt_handle(uint32_t instance)
{
    audio_controller_t *sai = &g_sai[instance];
    i2s_device_config_t *device_config = sai->config; 
    i2s_config_t *config = device_config->rx_config;
    audio_data_format_t *format = &config->format;
    uint8_t i = 0, j = 0;
    uint8_t data_size = 0;
    uint32_t data = 0;
    uint8_t space = config->watermark;

    data_size = format->bits/8;
    if((data_size == 3) || (format->bits & 0x7))
    {
        data_size = 4;
    }
    /* Judge if FIFO error */
    if (sai_hal_fifo_error_is_detected(instance, kSaiIOModeReceive))
    {
        sai_hal_reset_rx(instance, kSaiResetTypeFIFO);
    }
    /*Judge if the data need to transmit is less than space */
    if(space > ( AUDIO_BUFFER_BLOCK_SIZE - rx_count)/data_size)
    {
        space = (AUDIO_BUFFER_BLOCK_SIZE - rx_count)/data_size;
    }
    /* Read data from FIFO to the buffer */
    for (i = 0; i < space; i ++)
    {
        sai_hal_receive_data(instance,config->channel,&data);
        for(j = 0; j < data_size; j ++)
        {
            *rx_status.input_address = (data >> (8U * j)) & 0xFF;
            rx_status.input_address ++;         
        }
        rx_count += data_size;
    }
    //sai_hal_clear_rx_state_flag(instance, kSaiStateFlagWordStart);
    sai_hal_clear_rx_state_flag(instance, kSaiStateFlagFIFOError);	
    /* If need to callback the function */
    if (rx_count >= AUDIO_BUFFER_BLOCK_SIZE)
    {
        rx_count = 0;
        if(rx_status.input_address >= rx_address_end)
        {
            rx_status.input_address = rx_address_start;
        }
        g_sai_ops.ctrl_rx_callback((void *)sai);
    }
}
#endif

/*******************************************************************************
 *EOF
 ******************************************************************************/
