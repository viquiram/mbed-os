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

#ifndef __FSL_SAI_HAL_H__
#define __FSL_SAI_HAL_H__


#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_sai_features.h"


/*!
 * @addtogroup sai_hal
 * @{
 */
 
/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Define the bit limits of in a word*/
#define SAI_BIT_MIN	8
#define SAI_BIT_MAX	32

/* Define the limits of word number per frame */
#define SAI_WORD_MAX	FSL_FEATURE_I2S_MAX_WORDS_PER_FRAME

/* Define the max div and fract value for master clock divider. */
#define SAI_FRACT_MAX	256
#define SAI_DIV_MAX		4096

/* Define the max value for watermark setting. */
#define SAI_WATERMARK_MAX   FSL_FEATURE_I2S_FIFO_COUNT
#define SAI_FIFO_LEN		FSL_FEATURE_I2S_FIFO_COUNT

/*! @brief Define the bus type of sai */
typedef enum _sai_bus
{
    kSaiBusI2SLeft = 0x0,/*!< Use I2S left aligned format */
    kSaiBusI2SRight = 0x1,/*!< Use I2S right aligned format */
    kSaiBusI2SType = 0x2,/*!< Use I2S format */ 
 } sai_bus_t;

/*! @brief Transmit or receive data, or reand and write at the same time */
typedef enum _sai_io_mode
{
    kSaiIOModeTransmit = 0x0,/*!< Write data to FIFO */
    kSaiIOModeReceive = 0x1,/*!< Read data from FIFO */
    kSaiIOModeDuplex = 0x2/*!< Read data and write data at the same time */
} sai_io_mode_t;

/*! @brief Master or slave mode */
typedef enum _sai_master_slave
{
    kSaiMaster = 0x0,/*!< Master mode */
    kSaiSlave = 0x1/*!< Slave mode */
} sai_master_slave_t;

/*! @brief Synchronous or asynchronous mode */
typedef enum _sai_sync_mode
{
    kSaiModeAsync = 0x0,/*!< Asynchronous mode */
    kSaiModeSync = 0x1,/*!< Synchronous mode (with receiver or transmit) */
    kSaiModeSyncWithOtherTx = 0x2,/*!< Synchronous with another sai transmit */
    kSaiModeSyncWithOtherRx = 0x3/*!< Synchronous with another sai receiver */
} sai_sync_mode_t;

/*! @brief Mater clock source */
typedef enum _sai_mclk_source
{
    kSaiMclkSourceSysclk = 0x0,/*!< Master clock from system clock */
    kSaiMclkSourceExtal = 0x1,/*!< Master clock from extal */
    kSaiMclkSourceAltclk = 0x2,/*!< Master clock from alt */
    kSaiMclkSourcePllout = 0x3/*!< Master clcok from pll */ 
} sai_mclk_source_t;

/*! @brief Bit clock source */
typedef enum _sai_bclk_source
{
    kSaiBclkSourceBusclk = 0x0,/*!< Bit clock using bus clock */
    kSaiBclkSourceMclkDiv = 0x1,/*!< Bit clock using master clock divider */
    kSaiBclkSourceOtherSai0 = 0x2,/*!< Bit clock from other sai device */
    kSaiBclkSourceOtherSai1 = 0x3/*!< Bit clock from other sai device */
} sai_bclk_source_t;

/*! @brief The state flag of the sai */
typedef enum _sai_interrupt_request
{
    kSaiIntrequestWordStart = 0x0,/*!< Word start flag, means the first word in a frame detected */
    kSaiIntrequestSyncError = 0x1,/*!< Sync error flag, means the sync error is detected */
    kSaiIntrequestFIFOWarning = 0x2,/*!< FIFO warning flag, means the FIFO is empty */
    kSaiIntrequestFIFOError = 0x3,/*!< FIFO error flag */
    kSaiIntrequestFIFORequest = 0x4/*!< FIFO request, means reached watermark */
} sai_interrupt_request_t;


/*! @brief The dma request sources */
typedef enum _sai_dma_type
{
    kSaiDmaReqFIFOWarning = 0x0,/*!< FIFO warning caused dma request */
    kSaiDmaReqFIFORequest = 0x1/*!< FIFO request caused dma request */
} sai_dma_request_t;

/*! @brief The state flag of the sai */
typedef enum _sai_state_flag
{
    kSaiStateFlagWordStart = 0x0,/*!< Word start flag, means the first word in a frame detected. */
    kSaiStateFlagSyncError = 0x1,/*!< Sync error flag, means the sync error is detected */
    kSaiStateFlagFIFOError = 0x2,/*!< FIFO error flag */
} sai_state_flag_t;

/*! @brief The reset type */
typedef enum _sai_reset
{
    kSaiResetTypeSoftware = 0x0,/*!< Software reset, reset the logic state */
    kSaiResetTypeFIFO = 0x1/*!< FIFO reset, reset the FIFO read and write pointer */
} sai_reset_type_t;

/*
 * @brief The running mode of sai
 * The mode includes normal mode, debug mode, stop mode
 */
typedef enum _sai_running_mode
{
    kSaiRunModeDebug = 0x0,/*!< In debug mode */ 
    kSaiRunModeStop = 0x1/*!< In stop mode */
} sai_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief  Initialize sai device.
 *
 * This initialize just reset sai module by setting SR bit of TCSR and RCSR register.
 * Note that the function would write 0 to every control registers.
 * @param instance The sai peripheral instance number.
 */
void sai_hal_init(uint8_t instance);

/*!
 * @brief Set the bus protocol relevant settings for tx.
 *
 * The bus mode means which protocol sai uses, it can be I2S left, right and so on. Each protocol
 * would have different configure on bit clock and frame sync.
 * @param instance The sai peripheral instance number.
 * @param bus_mode The protocol selection, it can be I2S left aligned, I2S right aligned and etc.
 */
void sai_hal_set_tx_bus(uint8_t instance, sai_bus_t bus_mode);

/*!
 * @brief Set the bus protocol relevant settings for rx.
 *
 * The bus mode means which protocol sai uses, it can be I2S left, right and so on. Each protocol
 * would have different configure on bit clock and frame sync.
 * @param instance The sai peripheral instance number.
 * @param bus_mode The protocol selection, it can be I2S left aligned, I2S right aligned and etc.
 */
void sai_hal_set_rx_bus(uint8_t instance, sai_bus_t bus_mode);

/*!
 * @brief Set the master clock source.
 *
 * The source of the clock can be 4 types: PLL_OUT, ALT_CLK, EXTAL, SYS_CLK.
 * This function would set the clock source for sai master clock source.
 * Master clock is used to produce the bit clock for the data transfer, the 
 * @param instance The sai peripheral instance number.
 * @param source Mater clock source
 */
static inline void sai_hal_set_mclk_source(uint8_t instance, sai_mclk_source_t source)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_MCR_MICS(instance,source);
}

/*!
 * @brief Set the divider of master clock.
 *
 * Using the divider to get the master clock frequency wanted from the source. 
 * mclk = clk_source * fract/divide. The input is the master clock need and the sorce clock frequency.
 * The master clock is decided by sample rate and the multi-clock number.
 * @param instance The sai peripheral instance number.
 * @param mclk Master clock frequency needed.
 * @param src_clk The source clock frequency.
 */
void sai_hal_set_mclk_divider(uint8_t instance, uint32_t mclk, uint32_t src_clk);

/*!
 * @brief Set bit clock source of tx, it would generated by master clock, bus clock and other devices.
 *
 * The function would set the source of the bit clock, the bit clock can not only be produced by master
 * clock, but also can from the bus clock or other SAI tx/rx. Tx and Rx in an SAI can use the same bit 
 * clock from tx or rx.
 * @param instance The sai peripheral instance number.
 * @param source Bit clock source.
 */
static inline void sai_hal_set_tx_bclk_source(uint8_t instance, sai_bclk_source_t source)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_TCR2_MSEL(instance,source);
}

/*!
 * @brief Set bit clock source of rx, it would generated by master clock, bus clock and other devices.
 *
 * The function would set the source of rx bit clock, the bit clock can not only be produced by master
 * clock, but also can from the bus clock or other SAI tx/rx. Tx and Rx in an SAI can use the same bit 
 * clock from tx or rx.
 * @param instance The sai peripheral instance number.
 * @param source Bit clock source.
 */
static inline void sai_hal_set_rx_bclk_source(uint8_t instance, sai_bclk_source_t source)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_RCR2_MSEL(instance,source);
}

/*!
 * @brief Set the bit clock divider value of tx. 
 *
 * bclk = mclk / divider. At the same time, bclk = sample_rate * channel * bits. means the 
 * how much time needed to transfer one bit.
 * Notice: The function would be called while the bit clock source is the master clock.
 * @param instance The sai peripheral instance number.
 * @param div The divide number of bit clock.
 */
static inline void sai_hal_set_tx_blck_divider(uint8_t instance, uint32_t divider)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(divider);
    BW_I2S_TCR2_DIV(instance,divider/2 -1);
}

/*!
 * @brief Set the bit clock divider value of tx.
 *
 * bclk = mclk / divider. At the same time, bclk = sample_rate * channel * bits. means the 
 * how much time needed to transfer one bit.
 * Notice: The function would be called while the bit clock source is the master clock.
 * @param instance The sai peripheral instance number.
 * @param div The divide number of bit clock.
 */
static inline void sai_hal_set_rx_blck_divider(uint8_t instance, uint32_t divider)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(divider);
    BW_I2S_RCR2_DIV(instance,divider/2 -1);
}

/*!
 * @brief Set the frame size for tx. 
 *
 * The frame size means how many words in a frame. In the usual case, for example 2-channel
 * audio data, the frame size is 2, means 2 words in a frame.
 * @param instance The sai peripheral instance number.
 * @param size Words number in a frame.
 */
static inline void sai_hal_set_tx_frame_size(uint8_t instance, uint8_t size)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(size <= SAI_WORD_MAX);
    BW_I2S_TCR4_FRSZ(instance,size-1);
}

/*!
 * @brief Set the frame size for rx.
 *
 * The frame size means how many words in a frame. In the usual case, for example 2-channel
 * audio data, the frame size is 2, means 2 words in a frame.
 * @param instance The sai peripheral instance number.
 * @param size Words number in a frame.
 */
static inline void sai_hal_set_rx_frame_size(uint8_t instance, uint8_t size)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(size <= SAI_WORD_MAX);
    BW_I2S_RCR4_FRSZ(instance,size-1);
}

/*!
 * @brief Set the word size for tx.
 *
 * The word size means the quantization level of audio file. 
 * Generally, there are 8bit, 16bit, 24bit, 32bit format which sai would all support.
 * @param instance The sai peripheral instance number.
 * @param bits How many bits in a word.
*/
static inline void sai_hal_set_tx_word_size(uint8_t instance, uint8_t bits)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert((bits >= SAI_BIT_MIN) && (bits <= SAI_BIT_MAX));
    BW_I2S_TCR5_WNW(instance,bits-1);
}

/*!
 * @brief Set the word size for rx.
 *
 * The word size means the quantization level of audio file. 
 * Generally, there are 8bit, 16bit, 24bit, 32bit format which sai would all support.
 * @param instance The sai peripheral instance number.
 * @param bits How many bits in a word.
 */
static inline void sai_hal_set_rx_word_size(uint8_t instance, uint8_t bits)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert((bits >= SAI_BIT_MIN) && (bits <= SAI_BIT_MAX));
    BW_I2S_RCR5_WNW(instance,bits-1);
}

/*!
 * @brief Set the size of the first word of the frame for tx.
 *
 * In I2S protocol, the first word is the same as other word, but in some protocol,
 * for example, AC'97, the first word is not the same with others. This function would 
 * set the length of first word which actually the same with others in most situations.
 * @param instance The sai peripheral instance number.
 * @param size The length of frame head word.
 */
static inline void sai_hal_set_tx_word_zero_size(uint8_t instance, uint8_t size)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert((size >= SAI_BIT_MIN) && (size <= SAI_BIT_MAX));
    BW_I2S_TCR5_W0W(instance,size-1);
}

/*!
 * @brief Set the size of the first word of the frame for rx.
 *
 * In I2S protocol, the first word is the same as other word, but in some protocol,
 * for example, AC'97, the first word is not the same with others. This function would 
 * set the length of first word which actually the same with others in most situations.
 * @param instance The sai peripheral instance number.
 * @param size The length of frame head word.
 */
static inline void sai_hal_set_rx_word_zero_size(uint8_t instance, uint8_t size)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert((size >= SAI_BIT_MIN) && (size <= SAI_BIT_MAX));
    BW_I2S_RCR5_W0W(instance,size-1);
}

/*!
 * @brief Set the sync width for tx.
 *
 * A sync means the number of bit clocks of a frame, the sync width cannot longer than the 
 * length of the first word of the frame.
 * @param instance The sai peripheral instance number.
 * @param width How many bit clock in a sync.
 */
static inline void sai_hal_set_tx_sync_width(uint8_t instance, uint8_t width)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(width <= SAI_BIT_MAX);
    BW_I2S_TCR4_SYWD(instance,width-1);
}

/*!
 * @brief Set the sync width for rx.
 *
 * A sync means the number of bit clocks of a frame, the sync width cannot longer than the 
 * length of the first word of the frame.
 * @param instance The sai peripheral instance number.
 * @param width How many bit clock in a sync.
 */
static inline void sai_hal_set_rx_sync_width(uint8_t instance, uint8_t width)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(width <= SAI_BIT_MAX);
    BW_I2S_RCR4_SYWD(instance,width-1);
}

/*!
 * @brief Set the watermark value for tx FIFO.
 *
 * Watermark means while the value in tx FIFO is less or equal to it , it would generate an interrupt 
 * request or dma request. The watermark value can not more than the depth of FIFO.
 * @param instance The sai peripheral instance number.
 * @param watermark Watermark value of a FIFO.
 */
static inline void sai_hal_set_tx_watermark(uint8_t instance, uint8_t watermark)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(watermark < SAI_WATERMARK_MAX);
    BW_I2S_TCR1_TFW(instance,watermark);
}

/*!
 * @brief Set the watermark value for rx FIFO.
 *
 * Watermark means while the value in rx FIFO is larger or equal to it , it would generate an interrupt 
 * request or dma request. The watermark value can not more than the depth of FIFO.
 * @param instance The sai peripheral instance number.
 * @param watermark Watermark value of a FIFO.
 */
static inline void sai_hal_set_rx_watermark(uint8_t instance, uint8_t watermark)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(watermark < SAI_WATERMARK_MAX);
    BW_I2S_RCR1_RFW(instance,watermark);
}

/*!
 * @brief Set master or slave mode of tx.
 *
 * The function would set the mode of tx, if it is in master or slave mode. Master mose would provide its
 * own clock and slave mode would use extern clock.
 * @param instance The sai peripheral instance number.
 * @param master_slave_mode Mater or slave mode.
 */
void sai_hal_set_tx_master_slave(uint8_t instance, sai_master_slave_t master_slave_mode);

/*!
 * @brief Set master or slave mode of rx.
 *
 * The function would set the mode of rx, if it is in master or slave mode. Master mose would provide its
 * own clock and slave mode would use extern clock.
 * @param instance The sai peripheral instance number.
 * @param master_slave_mode Mater or slave mode.
 */
void sai_hal_set_rx_master_slave(uint8_t instance, sai_master_slave_t master_slave_mode);

/*!
 * @brief Transmit mode setting. 
 *
 * The mode can be asynchronous mode, synchronous, synchronous with other sai device.
 * When configured for a synchronous mode of operation, the receiver must be configured for asynchronous operation.
 * @param instance The sai peripheral instance number.
 * @param sync_mode Synchronouns mode or Asychronous mode.
 */
void sai_hal_set_tx_sync_mode(uint8_t instance, sai_sync_mode_t sync_mode);

/*!
 * @brief Receive mode setting.
 *
 * The mode can be asynchronous mode, synchronous, synchronous with other sai device.
 * When configured for a synchronous mode of operation, the receiver must be configured for asynchronous operation.
 * @param instance The sai peripheral instance number.
 * @param sync_mode Synchronouns mode or Asychronous mode.
 */
void sai_hal_set_rx_sync_mode(uint8_t instance, sai_sync_mode_t sync_mode);

/*!
 * @brief Get read pointer of FIFO.
 *
 * It would be used to judge if the fifo is full or empty and know how much sapce of FIFO.
 * If read_ptr == write_ptr, it means the fifo is empty, and while the bit of read_ptr and write_ptr are
 * equal except for the MSB, it means the fifo is full.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @param fifo_channel FIFO channel selected.
 * @return FIFO read pointer value.
 */
uint8_t sai_hal_get_fifo_read_pointer(uint8_t instance, sai_io_mode_t io_mode, uint8_t fifo_channel);

/*!
 * @brief Get read pointer of FIFO.
 *
 * It would be used to judge if the fifo is full or empty and know how much sapce of FIFO.
 * If read_ptr == write_ptr, it means the fifo is empty, and while the bit of read_ptr and write_ptr are
 * equal except for the MSB, it means the fifo is full.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @param fifo_channel FIFO channel selected.
 * @return FIFO write pointer value
 */
uint8_t sai_hal_get_fifo_write_pointer(uint8_t instance, sai_io_mode_t io_mode,uint8_t fifo_channel);

/*!
 * @brief Get TDR/RDR register address
 *
 * This function is used in dma trnasfer, as it needs to know the dest/src address of dma transfer.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @param fifo_channel FIFO channel selected.
 * @return TDR register or RDR register address
 */
uint32_t* sai_hal_get_fifo_address(uint8_t instance, sai_io_mode_t io_mode, uint8_t fifo_channel);

/*!
 * @brief Enable tx transmit.
 *
 * Enables the transmitter. This function would enable both the bit clock and the transfer channel.
 * @param instance The sai peripheral instance number.
 */
static inline void sai_hal_enable_tx(uint8_t instance)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_TCSR_BCE(instance,1);
    BW_I2S_TCSR_TE(instance,1);
}

/*!
 * @brief Enable rx receive.
 *
 * Enables the receiver. This function would enable both the bit clock and the receive channel.
 * @param instance The sai peripheral instance number.
 */
static inline void sai_hal_enable_rx(uint8_t instance)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_RCSR_BCE(instance,1);	
    BW_I2S_RCSR_RE(instance,1);
}

/*!
 * @brief Disable tx transmit.
 *
 * Disables the transmitter. This function would disable both the bit clock and the transfer channel.
 * When software clears this field, the transmitter remains enabled, and this bit remains set, until 
 * the end of the current frame.
 * @param instance The sai peripheral instance number.
 */
static inline void sai_hal_disable_tx(uint8_t instance)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_TCSR_TE(instance,0);
    BW_I2S_TCSR_BCE(instance,0);
}

/*!
 * @brief Disable rx receive.
 *
 * Disables the receiver. This function would disable both the bit clock and the transfer channel.
 * When software clears this field, the receiver remains enabled, and this bit remains set, until 
 * the end of the current frame.
 * @param instance The sai peripheral instance number.
 */
static inline void sai_hal_disable_rx(uint8_t instance)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_RCSR_RE(instance,0);
    BW_I2S_RCSR_BCE(instance,0);
}

/*!
 * @brief Enable tx interrupt from different interrupt sources.
 *
 * The interrupt source can be : Word start flag, Sync error flag, FIFO error flag, FIFO warning flag, FIFO request flag.
 * This function would set which flag would cause an interrupt request, these flag means different meanings. 
 * @param instance The sai peripheral instance number.
 * @param source Sai interrupt request source.
 */
void sai_hal_enable_tx_interrupt(uint8_t instance, sai_interrupt_request_t source);

/*!
 * @brief Enable rx interrupt from different sources.
 *
 * The interrupt source can be : Word start flag, Sync error flag, FIFO error flag, FIFO warning flag, FIFO request flag.
 * This function would set which flag would cause an interrupt request, these flag means different meanings. 
 * @param instance The sai peripheral instance number.
 * @param source Sai interrupt request source.
 */
void sai_hal_enable_rx_interrupt(uint8_t instance, sai_interrupt_request_t source);

/*!
 * @brief Disable tx interrupts from different interrupt sources.
 *
 * This function would disable the interrupt requests from interrupt request source of sai.
 * @param instance The sai peripheral instance number.
 * @param source Sai interrupt request source.
 */
void sai_hal_disable_tx_interrupt(uint8_t instance, sai_interrupt_request_t source);

/*!
 * @brief Disable rx interrupts from different interrupt sources.
 *
 * This function would disable the interrupt requests from interrupt request source of sai.
 * @param instance The sai peripheral instance number.
 * @param source Sai interrupt request source.
 */
void sai_hal_disable_rx_interrupt(uint8_t instance, sai_interrupt_request_t source);

/*!
 * @brief Enable tx dma request from different sources.
 *
 * The dma sources can be: FIFO warning and FIFO request.
 * This function would enable the dma request from different dma request source.
 * @param instance The sai peripheral instance number.
 * @param source Sai dma request source.
 */
void sai_hal_enable_tx_dma(uint8_t instance, sai_dma_request_t request);

/*!
 * @brief Enable rx dma request from different sources.
 *
 * The dma sources can be: FIFO warning and FIFO request.
 * This function would enable the dma request from different dma request source.
 * @param instance The sai peripheral instance number.
 * @param source Sai dma request source.
 */
void sai_hal_enable_rx_dma(uint8_t instance, sai_dma_request_t request);

/*!
 * @brief  Disable tx dma request from different sources.
 *
 * The function would disable the dma request of tx in sai. DMA request can from FIFO warning or FIFO
 * request which means FIFO is empty or reach the watermark.
 * @param instance The sai peripheral instance number.
 * @param source Sai dma request source.
 */
void sai_hal_disable_tx_dma(uint8_t instance, sai_dma_request_t request);

/*!
 * @brief  Disable rx dma request from different sources.
 *
 * The function would disable the dma request of tx in sai. DMA request can from FIFO warning or FIFO
 * request which means FIFO is empty or reach the watermark.
 * @param instance The sai peripheral instance number.
 * @param source Sai dma request source.
 */
void sai_hal_disable_rx_dma(uint8_t instance, sai_dma_request_t request);

/*!
 * @brief Clear the state flags for tx.
 *
 * The function is used to clear the flags manualy. It can clear word start, fifo warning, fifo error,
 * fifo request flag.
 * @param instance The sai peripheral instance number.
 * @param flag Sai state flag type. The flag can be word start, sync error, FIFO error/warning.
 */
void sai_hal_clear_tx_state_flag(uint8_t instance, sai_state_flag_t flag);

/*!
 * @brief Clear the state flags for rx.
 *
 * The function is used to clear the flags manualy. It can clear word start, fifo warning, fifo error,
 * fifo request flag.
 * @param instance The sai peripheral instance number.
 * @param flag Sai state flag type. The flag can be word start, sync error, FIFO error/warning.
 */
void sai_hal_clear_rx_state_flag(uint8_t instance, sai_state_flag_t flag);

/*!
 * @brief Rest tx.
 *
 * There are two kinds of reset: Software reset and FIFO reset.
 * Software reset:resets all transmitter internal logic, including the bit clock generation, status flags and FIFO pointers. It does not reset the
 * configuration registers.
 * FIFO reset: synchronizes the FIFO write pointer to the same value as the FIFO read pointer. This empties the FIFO contents and is to be used
 * after the Transmit FIFO Error Flag is set, and before the FIFO is re-initialized and the Error Flag is cleared.
 * @param instance The sai peripheral instance number.
 * @param mode Sai reset type.
 */
void sai_hal_reset_tx(uint8_t instance, sai_reset_type_t mode);

/*!
 * @brief Reset rx.
 * @param instance The sai peripheral instance number.
 * @param mode Sai reset type.
 */
void sai_hal_reset_rx(uint8_t instance, sai_reset_type_t mode);

/*!
 * @brief Set the mask word of the frame in tx.
 *
 * Each bit number represent the mask word index. For example, 0 represents mask the 0th word, 3 represents mask 0th and 1st word.
 * The TMR register can be different from frame to frame. If you want a mono audio, you can just set the mask to 0/1.
 * @param instance The sai peripheral instance number.
 * @param mask Which bits need to be masked in a frame.
 */
static inline void sai_hal_set_tx_word_mask(uint8_t instance, uint32_t mask)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    HW_I2S_TMR_WR(instance,mask);
}

/*!
 * @brief Set the mask word of the frame in rx.
 * @param instance The sai peripheral instance number.
 * @param mask Which bits need to be masked in a frame.
 */
static inline void sai_hal_set_rx_word_mask(uint8_t instance, uint32_t mask)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    HW_I2S_RMR_WR(instance,mask);
}

/*!
 * @brief Set the FIFO channel of tx.
 *
 * An sai instance includes a tx and a rx, each have several channels according to 
 * different platforms, a channel means a path for audio data input/output.
 * @param instance The sai peripheral instance number.
 * @param fifo_channel FIFO channel number.
 */
static inline void sai_hal_set_tx_fifo_channel(uint8_t instance, uint8_t fifo_channel)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_TCR3_TCE(instance,fifo_channel + 1);
}

/*!
 * @brief Set the FIFO channel of rx.
 * @param instance The sai peripheral instance number.
 * @param fifo_channel FIFO channel number.
 */
static inline void sai_hal_set_rx_fifo_channel(uint8_t instance, uint8_t fifo_channel)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    BW_I2S_RCR3_RCE(instance,fifo_channel + 1);
}

/*!
 * @brief Set the running mode. There are debug mode, stop mode and normal mode.
 *
 * This function can set the working mode of the sai instance, stop mode would always been
 * used in low power case, and the debug mode would disable SAI after complete the current 
 * transmit/receive.
 * @param instance The sai peripheral instance number.
 * @param mode Sai running mode.
 */
void sai_hal_set_tx_mode(uint8_t instance, sai_mode_t mode);

/*!
 * @brief Set the running mode of rx.
 * @param instance The sai peripheral instance number.
 * @param mode Sai running mode.
 */
void sai_hal_set_rx_mode(uint8_t instance, sai_mode_t mode);

/*!
 * @brief Set tx bit clock swap.
 *
 * While set in asynchronous mode, the transmitter is clocked by the receiver bit clock. When set in
 * synchronous mode, the transmitter is clocked by the transmitter bit clock, but uses the receiver frame
 * sync. This bit has no effect when synchronous with another SAI peripheral.
 * @param instance The sai peripheral instance number.
 * @param ifswap If swap bit clock.
 */
void sai_hal_set_tx_bclk_swap(uint8_t instance, bool ifswap);

/*!
 * @brief Set rx bit clock swap.
 *
 * When set in asynchronous mode, the receiver is clocked by the transmitter bit clock. When set in
 * synchronous mode, the receiver is clocked by the receiver bit clock, but uses the transmitter frame sync.
 * This bit has no effect when synchronous with another SAI peripheral.
 * @param instance The sai peripheral instance number.
 * @param ifswap If swap bit clock.
 */
void sai_hal_set_rx_bclk_swap(uint8_t instance, bool ifswap);

/*!
 * @brief Configure which word the start of word flag is set.
 * @param instance The sai peripheral instance number.
 * @param index Which word would trigger word start flag.
 */
static inline void sai_hal_set_tx_word_start_index(uint8_t instance, uint8_t index)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(index <= SAI_WORD_MAX);
    BW_I2S_TCR3_WDFL(instance,index);
}

/*!
 * @brief Configure which word the start of word flag is set.
 * @param instance The sai peripheral instance number.
 * @param index Which word would trigger word start flag.
 */
static inline void sai_hal_set_rx_word_start_index(uint8_t instance, uint8_t index)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(index <= SAI_WORD_MAX);
    BW_I2S_RCR3_WDFL(instance,index);
}

/*!
 * @brief Set the index in fifo for the first bit data .
 *
 * As the FIFO is 32-bit in SAI, not all audio data is 32-bit, mostly they are 16-bit and so on.
 * In this situation, the codec need to know from which bit of the fifo is the valid audio data.
 * @param instance The sai peripheral instance number.
 * @param index First bit shifted in FIFO.
 */
static inline void sai_hal_set_tx_fbt(uint8_t instance, uint8_t index)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(index <= SAI_BIT_MAX);
    BW_I2S_TCR5_FBT(instance,index);
}

/*!
 * @brief Set the index in fifo for the first bit data.
 * @param instance The sai peripheral instance number.
 * @param index First bit shifted in FIFO.
 */
static inline void sai_hal_set_rx_fbt(uint8_t instance, uint8_t index)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(index <= SAI_BIT_MAX);
    BW_I2S_RCR5_FBT(instance,index);
}

/*!
 * @brief Flag to see if the master clock divider is re-divided.
 * @param instance The sai peripheral instance number.
 * @return True if the divider updated otherwise false.
 */
bool sai_hal_mclk_divider_is_update(uint8_t instance);

/*!
 * @brief Word start is detected.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @return True if detect word start otherwise false.
 */
bool sai_hal_word_start_is_detected(uint8_t instance, sai_io_mode_t io_mode);

/*!
 * @brief Sync error is detected.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @return True if detect sync error otherwise false.
 */
bool sai_hal_sync_error_is_detected(uint8_t instance, sai_io_mode_t io_mode);

/*!
 * @brief FIFO warning is detected.
 *
 * FIFO warning means the fifo is empty in tx. While in tx, fifo warning means that
 * the fifo is empty and it needs data.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @return True if detect fifo warning otherwise false.
 */
bool sai_hal_fifo_warning_is_detected(uint8_t instance, sai_io_mode_t io_mode);

/*!
 * @brief FIFO error is detected.
 *
 * FIFO error means the fifo have no data and the codec is stil transferring data.
 * While in rx, fifo error means the data is still in but the fifo is full.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @return True if detect fifo error otherwise false.
 */
bool sai_hal_fifo_error_is_detected(uint8_t instance, sai_io_mode_t io_mode);

/*!
 * @brief FIFO request is detected.
 *
 * FIFO request means the data in fifo is less than watermark in tx and more than watermark while in rx.
 * @param instance The sai peripheral instance number.
 * @param io_mode Transmit or receive data.
 * @return True if detect fifo request otherwise false.
 */
bool sai_hal_fifo_request_is_detected(uint8_t instance, sai_io_mode_t io_mode);

/*!
 * @brief Receive data from FIFO.
 * @param instance The sai peripheral instance number.
 * @param rx_channel FIFO channel of rx.
 * @param data Pointer to the address to be written in.
 */
static inline void sai_hal_receive_data(uint8_t instance, uint8_t rx_channel, uint32_t *data)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(rx_channel < FSL_FEATURE_I2S_CHANNEL_COUNT);
    assert(data);
    
    *data = HW_I2S_RDRn_RD(instance, rx_channel);
}

/*!
 * @brief Transmit data to the FIFO.
 * @param instance The sai peripheral instance number.
 * @param tx_channel FIFO channel of tx.
 * @param data Data value which need to be written into FIFO.
 */
static inline void sai_hal_transmit_data(uint8_t instance, uint8_t tx_channel, uint32_t data)
{
    assert(instance < HW_I2S_INSTANCE_COUNT);
    assert(tx_channel < FSL_FEATURE_I2S_CHANNEL_COUNT);
    
    HW_I2S_TDRn_WR(instance,tx_channel,data);
}

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_SAI_HAL_H__ */
/*******************************************************************************
* EOF
*******************************************************************************/

