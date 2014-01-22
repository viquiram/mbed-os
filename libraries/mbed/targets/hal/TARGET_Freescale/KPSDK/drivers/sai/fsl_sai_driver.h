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

/*!
 * @addtogroup sai_driver
 * @{ 
 */

/*! @file */

#define USEDMA 1/*!< Use DMA mode or interrupt mode. */

/*! @brief Sai callback function */
typedef void (*sai_callback_t)(void *parameter);

/*! @brief Status structure for sai */
typedef enum _sai_status
{
    kStatus_SAI_Success = 0U,
    kStatus_SAI_Fail = 1U,
    kStatus_SAI_DeviceBusy = 2U
} sai_status_t;

/*! @brief The description structure for the sai tx/rx module. */
typedef struct SaiConfig
{
    sai_mclk_source_t	mclk_source;/*!< Master clock source. */
    bool				mclk_divide_enable;/*!< Enable the divide of master clock to generate bit clock. */
    sai_sync_mode_t	sync_mode;/*!< sychronous or asychronous. */
    sai_bus_t			bus_type;/*!< I2S left, I2S right or I2S type. */
    sai_master_slave_t	slave_master;/*!< Mater or slave. */
    sai_bclk_source_t	bclk_source;/*!< Bit clock from master clock or other modules. */
    uint8_t			channel;/*!< Which FIFO used to transfer. */
}  sai_config_t;

/*! @brief Define the PCM data format */
typedef struct SaiAudioDataFormat
{
    uint32_t sample_rate;/*!< Sample rate of the PCM file */
    uint32_t mclk;/*!< Master clock frequency */
    uint8_t  bits;/*!< How many bits in a word */
    uint8_t  words;/*!< How many word in a frame */
    uint8_t watermark; /*!< When to send interrupt/dma request*/
} sai_data_format_t;

/*! @brief The sai handler structure. The structure is used in the runtime of sai */
typedef struct SaiHandler
{
    uint8_t instance; /*!< Sai instance */
    bool direction; /*!< RX or TX */
    uint8_t fifo_channel; /*!< Which data channel used */     
} sai_handler_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize sai module.
 *
 * This initialize function would initialize sai registers according to the configuration
 * structure. This function would initialize the basic settings for sai, including
 * some board relevant settings.
 * Notice: This fucntion would not initialize a whole sai instance. This function
 * just initialize Tx or Rx according to the value in handler.
 * @param handler Sai handler structure pointer.
 * @param config The configuration structure of sai.
 * @return Return kStatus_SAI_Success while the initialize success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_init(sai_handler_t *handler, sai_config_t * config);

/*!
 * @brief De-initialize sai module.
 *
 * This function would close the sai device. It would not close the whole sai instance,
 * just close Tx or Rx according to the value in hanlder.
 * @param handler Sai handler structure pointer of sai module.
 * @return Return kStatus_SAI_Success while the process success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_deinit(sai_handler_t *handler);

/*!
 * @brief Configure the PCM data format.
 *
 * The function would configure mainly audio sample rate, data bits and channel number.
 * @param handler Sai handler structure pointer of sai module.
 * @param format PCM data format structure pointer.
 * @return Return kStatus_SAI_Success while the process success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_configure_data_format(sai_handler_t *handler, sai_data_format_t *format);

/*!
 * @brief Start to read data from fifo.
 *
 * The function would enable interrupt/dma request source and enable transmit channel.
 * @param handler Sai handler structure pointer of sai module.
 */
void sai_start_read_data(sai_handler_t *handler);

/*!
 * @brief Start to write data to fifo.
 *
 * The function would enable interrupt/dma request source and enable receive channel.
 * @param handler Sai handler structure pointer of sai module.
 */
void sai_start_write_data(sai_handler_t *handler);

/*!
 * @brief Stop read data form fifo, mainly to disable dma or interrupt request bit.
 *
 * This function provides the method to pause receiving data.  
 * @param handler Sai handler structure pointer of sai module.
 */
static inline void sai_stop_read_data(sai_handler_t *handler)
{
    //sai_hal_disable_rx(handler->instance);
#if USEDMA
    sai_hal_disable_rx_dma(handler->instance, kSaiDmaReqFIFORequest);
    sai_hal_disable_rx_dma(handler->instance, kSaiDmaReqFIFOWarning);
    sai_hal_disable_rx_interrupt(handler->instance, kSaiIntrequestFIFOError);
#else	
    sai_hal_disable_rx_interrupt(handler->instance,kSaiIntrequestFIFORequest);
    sai_hal_disable_rx_interrupt(handler->instance,kSaiIntrequestFIFOWarning);
    sai_hal_disable_rx_interrupt(handler->instance,kSaiIntrequestFIFOError);	
#endif	
}

/*!
 * @brief Stop write data to fifo, mainly to disable dma or interrupt request bit.
 *
 * This function provides the method to pause writing data.  
 * @param handler Sai handler structure pointer of sai module.
 */
static inline void sai_stop_write_data(sai_handler_t *handler)
{
    //sai_hal_disable_tx(handler->instance);
#if USEDMA
    sai_hal_disable_tx_dma(handler->instance, kSaiDmaReqFIFORequest);
    sai_hal_disable_tx_dma(handler->instance, kSaiDmaReqFIFOWarning);
    sai_hal_disable_tx_interrupt(handler->instance, kSaiIntrequestFIFOError);
#else	
    sai_hal_disable_tx_interrupt(handler->instance,kSaiIntrequestFIFORequest);
    sai_hal_disable_tx_interrupt(handler->instance,kSaiIntrequestFIFOWarning);	
#endif	
}

/*!
 * @brief Clear Tx FIFO error flag.
 * @param handler Sai handler structure pointer of sai module.
 */
static inline void sai_clear_tx_status(sai_handler_t *handler)
{
    sai_hal_clear_tx_state_flag(handler->instance, kSaiStateFlagFIFOError);
}

/*!
 * @brief Clear Rx FIFO error flag.
 * @param handler Sai handler structure pointer of sai module.
 */
static inline void sai_clear_rx_status(sai_handler_t *handler)
{
    sai_hal_clear_rx_state_flag(handler->instance, kSaiStateFlagFIFOError);
}

/*!
 * @brief Get the fifo address of the data channel.
 *
 * This function mainly used for dma settings, which dma
 * configuration needs the source/destination address of sai.
 * @param handler Sai handler structure pointer of sai module.
 * @return Return the address of the data channel fifo.
 */
static inline uint32_t* sai_get_fifo_address(sai_handler_t *handler)
{
    if(handler->direction)
    {
        return sai_hal_get_fifo_address(handler->instance, kSaiIOModeTransmit, handler->fifo_channel);
    }
    else
    {
        return sai_hal_get_fifo_address(handler->instance, kSaiIOModeReceive, handler->fifo_channel);
    }
}

/*!
 * @brief Send a certain length data.
 *
 * This function would send the data to tx fifo. This function would 
 * start the transfer, and while finished the transfer, it would call the callback
 * function registered by users.
 * @param handler Sai handler structure pointer of sai module.
 * @param addr Addrerss of the data which need to be transfer.
 * @param len The data length which need to be sent.
 * @return Return the length been sent.
 */
uint32_t sai_send_data(sai_handler_t *handler, uint8_t *addr, uint32_t len);

/*!
 * @brief Receive a certain length data.
 *
 * This function would receive data from rx fifo. This function would 
 * start the transfer, and while finished the transfer, it would call the callback
 * function registered by users.
 * @param handler Sai handler structure pointer of sai module.
 * @param addr Addrerss of the data which need to be transfer.
 * @param len The data length which need to be received.
 * @return Return the length received.
 */
uint32_t sai_receive_data(sai_handler_t *handler, uint8_t *addr, uint32_t len);

/*!
 * @brief Register the callback function after a transfer.
 *
 * This function would tell sai which function need to be called after a 
 * period length transfer. 
 * @param handler Sai handler structure pointer of sai module.
 * @param callback Callback function defined by users.
 * @param callback_param The parameter of the callback function.
 */
void sai_register_callback(sai_handler_t *handler, sai_callback_t callback, void *callback_param);


#if defined(__cplusplus)
}
#endif

/*! @} */

#endif/* __FSL_SAI_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

