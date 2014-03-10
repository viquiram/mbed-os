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

/*! @brief SAI callback function */
typedef void (*sai_callback_t)(void *parameter);

/*! @brief Status structure for SAI */
typedef enum _sai_status
{
    kStatus_SAI_Success = 0U,
    kStatus_SAI_Fail = 1U,
    kStatus_SAI_DeviceBusy = 2U
} sai_status_t;

/*! @brief The description structure for the SAI TX/RX module. */
typedef struct SaiUserConfig
{
    sai_mclk_source_t   mclk_source;/*!< Master clock source. */
    bool                mclk_divide_enable;/*!< Enable the divide of master clock to generate bit clock. */
    sai_sync_mode_t     sync_mode;/*!< Synchronous or asynchronous. */
    sai_bus_t           bus_type;/*!< I2S left, I2S right or I2S type. */
    sai_master_slave_t  slave_master;/*!< Master or slave. */
    sai_bclk_source_t   bclk_source;/*!< Bit clock from master clock or other modules. */
    uint8_t             channel;/*!< Which FIFO is used to transfer. */
}  sai_user_config_t;

/*! @brief Defines the PCM data format */
typedef struct SaiAudioDataFormat
{
    uint32_t sample_rate;/*!< Sample rate of the PCM file */
    uint32_t mclk;/*!< Master clock frequency */
    uint8_t  bits;/*!< How many bits in a word */
    uint8_t  words;/*!< How many word in a frame */
    uint8_t watermark; /*!< When to send interrupt/DMA request*/
} sai_data_format_t;

/*! @brief The SAI handler structure. The structure is used in the SAI runtime. */
typedef struct SaiHandler
{
    uint8_t instance; /*!< SAI instance */
    bool direction; /*!< RX or TX */
    uint8_t fifo_channel; /*!< Which data channel is used */     
} sai_handler_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the SAI module.
 *
 * This  function  initializes the SAI registers according to the configuration
 * structure. This function  initializes the basic settings for SAI, including
 * some board relevant settings.
 * Notice: This function does not initialize an entire SAI instance. This function
 * only initializes the TX or RX according to the value in the handler.
 * @param handler SAI handler structure pointer.
 * @param config The configuration structure of SAI.
 * @return Return kStatus_SAI_Success while the initialize success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_init(sai_handler_t *handler, sai_user_config_t * config);

/*!
 * @brief De-initializes the SAI module.
 *
 * This function  closes the SAI device. It does not close the entire SAI instance.
 * It only closes the TX or RX according to the value in the handler.
 * @param handler SAI handler structure pointer of the SAI module.
 * @return Return kStatus_SAI_Success while the process success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_deinit(sai_handler_t *handler);

/*!
 * @brief Configures the PCM data format.
 *
 * The function  mainly configures  an audio sample rate, data bits and a channel number.
 * @param handler SAI handler structure pointer of the SAI module.
 * @param format PCM data format structure pointer.
 * @return Return kStatus_SAI_Success while the process success and kStatus_SAI_Fail if failed.
 */
sai_status_t sai_configure_data_format(sai_handler_t *handler, sai_data_format_t *format);

/*!
 * @brief Starts  reading data from FIFO.
 *
 * The function  enables the interrupt/DMA request source and enables the transmit channel.
 * @param handler SAI handler structure pointer of the SAI module.
 */
void sai_start_read_data(sai_handler_t *handler);

/*!
 * @brief Starts  writing data to FIFO.
 *
 * The function  enables the interrupt/DMA request source and enables the receive channel.
 * @param handler SAI handler structure pointer of the SAI module.
 */
void sai_start_write_data(sai_handler_t *handler);

/*!
 * @brief Stops reading data from FIFO, mainly to disable the DMA or the interrupt request bit.
 *
 * This function provides the method to pause receiving data.  
 * @param handler SAI handler structure pointer of the SAI module.
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
 * @brief Stop write data to FIFO, mainly to disable the DMA or the interrupt request bit.
 *
 * This function provides the method to pause writing data.  
 * @param handler SAI handler structure pointer of the SAI module.
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
 * @brief Clears the TX FIFO error flag.
 * @param handler SAI handler structure pointer of the SAI module.
 */
static inline void sai_clear_tx_status(sai_handler_t *handler)
{
    sai_hal_clear_tx_state_flag(handler->instance, kSaiStateFlagFIFOError);
}

/*!
 * @brief Clears the RX FIFO error flag.
 * @param handler SAI handler structure pointer of the SAI module.
 */
static inline void sai_clear_rx_status(sai_handler_t *handler)
{
    sai_hal_clear_rx_state_flag(handler->instance, kSaiStateFlagFIFOError);
}

/*!
 * @brief Gets the FIFO address of the data channel.
 *
 * This function is mainly used for the DMA settings, which the DMA
 * configuration needs for the source/destination address of SAI.
 * @param handler SAI handler structure pointer of the SAI module.
 * @return Returns the address of the data channel FIFO.
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
 * @brief Sends a certain length data.
 *
 * This function  sends the data to the TX FIFO. This function  
 * starts the transfer, and while finishing the transfer, it  calls the callback
 * function registered by users.
 * @param handler SAI handler structure pointer of the SAI module.
 * @param addr Address of the data which needs to be transferred.
 * @param len The data length which need to be sent.
 * @return Returns the length which was sent.
 */
uint32_t sai_send_data(sai_handler_t *handler, uint8_t *addr, uint32_t len);

/*!
 * @brief Receives a certain length data.
 *
 * This function  receives the data from the RX FIFO. This function  
 * starts the transfer, and while finishing the transfer, it  calls the callback
 * function registered by users.
 * @param handler SAI handler structure pointer of the SAI module.
 * @param addr Address of the data which needs to be transferred.
 * @param len The data length which needs to be received.
 * @return Returns the length received.
 */
uint32_t sai_receive_data(sai_handler_t *handler, uint8_t *addr, uint32_t len);

/*!
 * @brief Registers the callback function after a transfer.
 *
 * This function  tells SAI which function needs to be called after a 
 * period length transfer. 
 * @param handler SAI handler structure pointer of the SAI module.
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

