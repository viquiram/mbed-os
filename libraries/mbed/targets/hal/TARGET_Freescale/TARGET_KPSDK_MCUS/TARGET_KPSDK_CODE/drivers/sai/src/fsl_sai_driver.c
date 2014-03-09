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

typedef struct sai_internal_status
{
    sai_data_format_t format;
    uint8_t * address;
    uint32_t len;
    uint32_t count;
    sai_callback_t  callback;
    void * callback_param;
    sai_sync_mode_t sync_mode;
} sai_internal_status_t;

extern IRQn_Type sai_irq_ids[HW_I2S_INSTANCE_COUNT][2];
static sai_handler_t * volatile sai_handler_ids[HW_I2S_INSTANCE_COUNT][2];
static volatile sai_internal_status_t sai_status_ids[HW_I2S_INSTANCE_COUNT][2];

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
sai_status_t sai_init(sai_handler_t *handler, sai_user_config_t * config)
{
    /* Open clock gate for sai instance */
    clock_manager_set_gate(kClockModuleSAI, handler->instance, true);
    /*Check if the device is busy */
    if(sai_handler_ids[handler->instance][handler->direction] != NULL)
    {
        return kStatus_SAI_DeviceBusy;
    }
    sai_handler_ids[handler->instance][handler->direction] = handler;
    sai_status_ids[handler->instance][handler->direction].sync_mode = config->sync_mode;
    /* Mclk source select */
    sai_hal_set_mclk_source(handler->instance, config->mclk_source);
    /* Tx register initialize */
    if(handler->direction)
    {
        sai_hal_set_tx_sync_mode(handler->instance,config->sync_mode);
        sai_hal_set_tx_master_slave(handler->instance,config->slave_master);
        sai_hal_set_tx_bus(handler->instance,config->bus_type);
        sai_hal_set_tx_fifo_channel(handler->instance,config->channel);
        interrupt_enable(sai_irq_ids[handler->instance][1]);
    }
    /* Rx register initialize */
    else
    {
        sai_hal_set_rx_sync_mode(handler->instance, config->sync_mode);
        sai_hal_set_rx_master_slave(handler->instance, config->slave_master);
        sai_hal_set_rx_bus(handler->instance, config->bus_type);
        sai_hal_set_rx_fifo_channel(handler->instance, config->channel); 
        interrupt_enable(sai_irq_ids[handler->instance][0]);
    }
    return kStatus_SAI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_deinit
 * Description   :Deinit the sai module, free the resources.
 * 
 *END**************************************************************************/
sai_status_t sai_deinit(sai_handler_t *handler)
{
    if(handler->direction)
    {
        sai_stop_write_data(handler);
        sai_hal_disable_tx(handler->instance);
        sai_hal_reset_tx(handler->instance,kSaiResetTypeSoftware);
        sai_hal_clear_tx_state_flag(handler->instance, kSaiStateFlagSoftReset);
    }
    else
    {
        sai_stop_read_data(handler);
        sai_hal_disable_rx(handler->instance);
        sai_hal_reset_rx(handler->instance,kSaiResetTypeSoftware);
        sai_hal_clear_rx_state_flag(handler->instance, kSaiStateFlagSoftReset);
    }
    sai_handler_ids[handler->instance][handler->direction] = NULL;
    /* Check if need to close the clock gate */
    if  ((sai_handler_ids[handler->instance][0] == NULL) && (sai_handler_ids[handler->instance][1] == NULL))
    {
        clock_manager_set_gate(kClockModuleSAI, handler->instance, false);
    }
    return kStatus_SAI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_configure_data_format
 * Description   :Configure audio format information.
 * The audio format information includes the sample rate, data length and so on.
 *END**************************************************************************/
sai_status_t sai_configure_data_format(sai_handler_t *handler, sai_data_format_t *format)
{
    uint32_t frequency = 0;
    uint32_t bclk = format->sample_rate * format->bits * 2;
    uint8_t divider = format->mclk/bclk;
     /* Get the clock source frequency */
     clock_manager_get_frequency(kCoreClock, &frequency);
    /* Configure master clock */
    sai_hal_set_mclk_divider(handler->instance, format->mclk, frequency);
    if (handler->direction)
    {
        /* Modify the tx_config */
        sai_status_ids[handler->instance][1].format = *format;	
        /* Master clock and bit clock setting */
        sai_hal_set_tx_bclk_source(handler->instance,kSaiBclkSourceMclkDiv);
        sai_hal_set_tx_blck_divider(handler->instance,divider);
        sai_hal_set_tx_sync_width(handler->instance,format->bits);
        /* Frmae size and word size setting */
        sai_hal_set_tx_frame_size(handler->instance, 2);
        sai_hal_set_tx_word_size(handler->instance,format->bits);
        sai_hal_set_tx_word_zero_size(handler->instance,format->bits);
        sai_hal_set_tx_word_start_index(handler->instance,0);
        sai_hal_set_tx_fbt(handler->instance,format->bits - 1);
        sai_hal_set_tx_watermark(handler->instance,format->watermark);
        /* The chennl number configuration */
        if (format->words == 1)
        {
            sai_hal_set_tx_word_mask(handler->instance, 0x2);
        }
        else
        {
            sai_hal_set_tx_word_mask(handler->instance, 0x0);
        }
    }
    else
    {
        sai_status_ids[handler->instance][0].format = *format;
        /* Bit clock setting */
        sai_hal_set_rx_bclk_source(handler->instance,kSaiBclkSourceMclkDiv);
        sai_hal_set_rx_blck_divider(handler->instance,divider);
        sai_hal_set_rx_sync_width(handler->instance,format->bits);
        
        /* Frmae size and word size setting */
        sai_hal_set_rx_frame_size(handler->instance,2);
        sai_hal_set_rx_word_size(handler->instance,format->bits);
        sai_hal_set_rx_word_zero_size(handler->instance,format->bits);
        sai_hal_set_rx_word_start_index(handler->instance,0);
        sai_hal_set_rx_fbt(handler->instance,format->bits - 1);
        sai_hal_set_rx_watermark(handler->instance,format->watermark);
        /* The chennl number configuration */
        if (format->words == 1)
        {
            sai_hal_set_rx_word_mask(handler->instance, 0x2);
        }
        else
        {
            sai_hal_set_rx_word_mask(handler->instance, 0x0);
        }
    }
    return kStatus_SAI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_start_write_data
 * Description   : Start the writing process.
 * 
 *END**************************************************************************/
void sai_start_write_data(sai_handler_t *handler)
{
#if USEDMA
    sai_hal_enable_tx_dma(handler->instance, kSaiDmaReqFIFORequest);
    sai_hal_enable_tx_dma(handler->instance, kSaiDmaReqFIFOWarning);
    /* Enable the FIFO error interrupt */
    sai_hal_enable_tx_interrupt(handler->instance, kSaiIntrequestFIFOError);
#else
    sai_hal_enable_tx_interrupt(handler->instance,kSaiIntrequestFIFORequest);
#endif
    sai_hal_enable_tx(handler->instance);
    /* If the sync mode is synchronous, it will need Rx enable bit clock */
    if(sai_status_ids[handler->instance][1].sync_mode == kSaiModeSync)
    {
        sai_hal_enable_rx(handler->instance);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_start_read_data
 * Description   : Start the reading process.
 * 
 *END**************************************************************************/
void sai_start_read_data(sai_handler_t *handler)
{
#if USEDMA
    sai_hal_enable_rx_dma(handler->instance, kSaiDmaReqFIFORequest);
    sai_hal_enable_rx_dma(handler->instance, kSaiDmaReqFIFOWarning);
    /* Enable the FIFO error interrupt */
    sai_hal_enable_rx_interrupt(handler->instance, kSaiIntrequestFIFOError);	
#else
    sai_hal_enable_rx_interrupt(handler->instance,kSaiIntrequestFIFORequest);
#endif
    sai_hal_enable_rx(handler->instance);
    /* If the sync mode is synchronous, it will need Tx enable bit clock */
    if(sai_status_ids[handler->instance][0].sync_mode == kSaiModeSync)
    {
        sai_hal_enable_tx(handler->instance);
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
    uint8_t data_size = 0;
    uint8_t i = 0, j = 0;
    sai_handler_t *handler = sai_handler_ids[instance][1];
    sai_data_format_t format = sai_status_ids[instance][1].format;
    uint8_t space = FSL_FEATURE_I2S_FIFO_COUNT - format.watermark;
    uint32_t data = 0, temp = 0;
    uint32_t len = sai_status_ids[instance][1].len;
    uint32_t count = sai_status_ids[instance][1].count;

    data_size = format.bits/8;
    if((data_size == 3) || (format.bits & 0x7))
    {
        data_size = 4;
    }

    /* Judge if FIFO error */
    if(sai_hal_fifo_error_is_detected(instance, kSaiIOModeTransmit))
    {
        sai_hal_reset_tx(instance, kSaiResetTypeFIFO);
    }
    /*Judge if the data need to transmit is less than space */
    if(space > (len -count)/data_size)
    {
        space = (len -count)/data_size;
    }
    /* If normal, copy the data from sai buffer to FIFO */
    for(i = 0; i < space; i++)
    {
        for(j = 0; j < data_size; j ++)
        {
            temp = (uint32_t)(*sai_status_ids[instance][1].address);
            data |= (temp << (8U * j));
            sai_status_ids[instance][1].address ++;
        }
        sai_hal_transmit_data(instance, handler->fifo_channel, (uint32_t )data);
        sai_status_ids[instance][1].count += data_size;
        data = 0;
    }
    sai_hal_clear_tx_state_flag(instance, kSaiStateFlagFIFOError);
    /* If a block is finished, just callback */
    count = sai_status_ids[instance][1].count;
    if(count == len)
    {
        void * callback_param = sai_status_ids[instance][1].callback_param;
        sai_status_ids[instance][1].count = 0;
        (sai_status_ids[instance][1].callback)(callback_param);
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
    uint8_t i = 0, j = 0;
    uint8_t data_size = 0;
    uint32_t data = 0;
    sai_handler_t *handler = sai_handler_ids[instance][0];
    sai_data_format_t format = sai_status_ids[instance][0].format;
    uint8_t space = format.watermark;
    uint32_t len = sai_status_ids[instance][0].len;
    uint32_t count = sai_status_ids[instance][0].count;

    data_size = format.bits/8;
    if((data_size == 3) || (format.bits & 0x7))
    {
        data_size = 4;
    }
    /* Judge if FIFO error */
    if (sai_hal_fifo_error_is_detected(instance, kSaiIOModeReceive))
    {
        sai_hal_reset_rx(instance, kSaiResetTypeFIFO);
    }
    /*Judge if the data need to transmit is less than space */
    if(space > (len - count)/data_size)
    {
        space = (len -count)/data_size;
    }
    /* Read data from FIFO to the buffer */
    for (i = 0; i < space; i ++)
    {
        sai_hal_receive_data(instance, handler->fifo_channel, &data);
        for(j = 0; j < data_size; j ++)
        {
            *sai_status_ids[instance][0].address = (data >> (8U * j)) & 0xFF;
            sai_status_ids[instance][0].address ++;
        }
        sai_status_ids[instance][0].count += data_size;
    }
    sai_hal_clear_rx_state_flag(instance, kSaiStateFlagFIFOError);	
    /* If need to callback the function */
    count = sai_status_ids[instance][0].count;
    if (count == len)
    {
        void *callback_param = sai_status_ids[instance][0].callback_param;
        sai_status_ids[instance][0].count = 0;
        (sai_status_ids[instance][0].callback)(callback_param);
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_register_callback
 * Description   : The function would register the callback function to tell sai
 * driver what to do after the transfer. 
 *END**************************************************************************/
void sai_register_callback(sai_handler_t *handler, sai_callback_t callback, void *callback_param)
{
    sai_status_ids[handler->instance][handler->direction].callback = callback;
    sai_status_ids[handler->instance][handler->direction].callback_param = callback_param;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_send_data
 * Description   : The function would tell sai driver to start send a period of
 * data to sai tx fifo.
 *END**************************************************************************/
uint32_t sai_send_data(sai_handler_t *handler, uint8_t *addr, uint32_t len)
{
    sai_status_ids[handler->instance][1].len = len;
    sai_status_ids[handler->instance][1].address= addr;
    return len;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sai_receive_data
 * Description   : The function would tell sai driver to start receive a period of
 * data from sai rx fifo.
 *END**************************************************************************/
uint32_t sai_receive_data(sai_handler_t *handler, uint8_t *addr, uint32_t len)
{
    sai_status_ids[handler->instance][0].len = len;
    sai_status_ids[handler->instance][0].address= addr;
    return len;
}
/*******************************************************************************
 *EOF
 ******************************************************************************/
