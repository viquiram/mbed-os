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
#if !defined(__FSL_EDMA_DRIVER_H__)
#define __FSL_EDMA_DRIVER_H__

#include <stdint.h>
#include "fsl_edma_request.h"
#include "fsl_edma_hal.h"
#include "fsl_dmamux_hal.h"

/*!
 * @addtogroup edma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Status of the DMA hardware descriptor
 *
 * Defines the status of the DMA descriptor. The status of the descriptor is not limited to
 * these two states. It can also be any the value between 0~0xFFFFFFFF for
 * left bytes of a specified descriptor.
 */
typedef enum _edma_descriptor_internal_status {
    kEdmaDescriptorDone = 0U,               /*!< The descriptor is finished. */
    kEdmaDescriptorPrepared = 0xFFFFFFFFU   /*!< The descriptor is either consumed or not consumed. */
} edma_descriptor_internal_status_t;

/*! @brief Alignment for eDMA TCD start address. */
typedef enum _edma_tcd_alignment {
    kEdmaTcdAlignment = 0x20U,        /*!< Alignment of the eDMA TCD. */
    kEdmaTcdAlignmentMask = 0x1FU     /*!< Alignment mask of the eDMA TCD. */
} edma_tcd_alignment_t;

/*!
 * @brief A constant for the DMA allocation status.
 *
 * A structure used for the dma_request_channel.
 */
typedef enum _edma_channel_type {
    kEdmaInvalidChannel = 0xFFU, /*!< Macros indicate the failure of the channel request. */
    kEdmaAnyChannel = 0xFEU      /*!< Macros used when requesting channel.  */
                                /*!< kEdmaAnyChannel means a request of a dynamic channel allocation. */
} edma_channel_type_t;

/*!
 * @brief Channel status for an eDMA channel.
 *
 * A structure describing the DMA channel status. The user can get the status from the channel callback
 * function.
 */
typedef enum _edma_channel_status {
    kEdmaIdle,                  /*!< DMA channel is idle. */
    kEdmaNormal,                    /*!< DMA channel is occupied. */
    kEdmaError                  /*!< An error occurs in the DMA channel. */
} edma_channel_status_t;

/*! @brief A type for the DMA transfer. */
typedef enum _edma_transfer_type {
    kEdmaPeripheralToMemory,        /*!< Transfer from peripheral to memory */
    kEdmaMemoryToPeripheral,        /*!< Transfer from memory to peripheral */
    kEdmaMemoryToMemory,            /*!< Transfer from memory to memory */
    kEdmaPeripheralToPeripheral  /*!< Transfer from peripheral to peripheral */
} edma_transfer_type_t;

/*!
 * @brief Definition for the DMA channel callback function.
 *
 * Prototype for the callback function registered in the DMA driver.
 */
typedef void (*edma_callback_t)(void *parameter, edma_channel_status_t status);

/*! @brief Data structure for configuring a discrete memory transfer. */
typedef struct EdmaScatterList {
    uint32_t address;           /*!< Address of buffer. */
    uint32_t length;            /*!< Length of buffer. */
} edma_scatter_list_t;

/*! @brief Data structure for the DMA channel. */
typedef struct EdmaChannel {
    uint8_t module;                 /*!< eDMA physical module indicator */
    uint8_t channel;                /*!< eDMA physical channel indicator */
    uint8_t dmamuxModule;           /*!< DMA Mux module indicator. */
    uint8_t dmamuxChannel;          /*!< DMA Mux channel indicator. */
    edma_callback_t callback;       /*!< Callback for the eDMA channel. */
    void *parameter;                /*!< Parameter for the callback function. */
    volatile edma_channel_status_t status;   /*!< Channel status. */
    uint8_t tcdNumber;              /*!< Length of the hardware descriptor chain. */
    volatile uint8_t tcdWrite;               /*!< Indicator for updates of the user descriptor. */
    volatile uint8_t tcdRead;                /*!< Indicator for the consuming of the DMA controller. */
    uint32_t tcdLeftBytes;          /*!< Left bytes to be transferred for a current TCD. */
    volatile bool tcdUnderflow;              /*!< Flag indicating whether the user failed to feed a descriptor in
                                         time. */
} edma_channel_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
  * @name eDMA Peripheral Driver
  * @{
  */

/*!
 * @brief Initializes eDMA.
 *
 */
edma_status_t edma_init(void);

/*!
 * @brief De-initilizes eDMA.
 *
 */
edma_status_t edma_shutdown(void);

/*!
 * @brief Registers the callback function and a parameter.
 *
 * The user register callback function and parameter for a specified eDMA channel. When channel
 * interrupt or channel error occurs, the callback function is called and a parameter along
 * with user parameter is provided to indicate the channel status.
 * 
 * @param chn Handler for eDMA channel.
 * @param callback Callback function.
 * @param para A parameter for callback functions.
 *
 */
void edma_register_callback(edma_channel_t *chn, edma_callback_t callback, void *para);

/*!
 * @brief Gets the status of the eDMA channel descriptor chain.
 *
 * This function indicates the status of descriptor chains. The user needs to provide the memory
 * space to store the descriptor status. A parameter descriptorStatus is the pointer for the
 * descriptorStatus. If the descriptorStaus can't point to a valid memory space and the
 * length of the memory is not enough to store the descriptor status, an error occurs inside the DMA
 * driver. Every descriptor needs a uint32_t to store the status.
 * If the return descriptorStatus[n] is equal to the kEdmaDescriptorPrepared, the descriptor
 * is either consuming or it is already prepared but not consumed. Any other value indicates the left
 * bytes to be transferred for this descriptor. If the value is equal to 0, it means the descriptor
 * is already finished. The user can update the memory for this descriptor safely.
 * To get a precise status of the descriptor, the user can first stop the channel. The ongoing descriptor
 * is updated with a value indicating bytes to be transferred for this descriptor. If not,
 * it indicates that this descriptor is ongoing with a value of
 * kEdmaDescriptorPrepared.
 *
 * @param chn Handler for eDMA channel.
 * @param descriptorStatus Status of descriptors.
 *
 */
edma_status_t edma_get_descriptor_status(edma_channel_t *chn, uint32_t *descriptorStatus);

/*!
 * @brief Requests an eDMA channel.
 *
 * This function provides two ways to allocate a DMA channel: static allocation and dynamic allocation.
 * To allocate a channel dynamically, the user should set the channel parameter with the value of 
 * kDmaAnyChannel. The driver searches  all available channels and assigns the first 
 * channel to the user.
 * To allocate the channel statically, the user should set the channel parameter with the value of a specified
 * channel. If the channel is available, the driver assigns the channel to the user.
 * Note that the user must provide the handler memory for the DMA channel. The driver initializes
 * the handler and configures the handler memory.
 *
 * @param channel eDMA channel number. If a channel is assigned with a valid channel number,
 * the DMA driver tries to assign a specified channel to the user. If a channel is assigned with a
 * kDmaAnyChannel, the DMA driver searches all available channels and assigns the first channel to the user.
 * @param source eDMA hardware request.
 * @param chan Memory pointing to eDMA channel. The user must ensure that the handler memory is valid and
 * that it will not be released or changed by another code before the channel is freed.
 *
 * @return If the channel allocation is successful, the return value indicates the requested channel. If
 * not, the driver  returns a  kDmaInvalidChannel value to indicate that the requested operation has failed.
 */
uint32_t edma_request_channel(uint32_t channel, dma_request_source_t source, edma_channel_t *chan);

/*!
 * @brief Frees eDMA channel hardware and software resource.
 *
 * This function frees the relevant software and hardware resources. The request and the freeing operation need to 
 * be called in a pair. 
 *
 * @param chn Memory pointing to the eDMA channel
 *
 */
edma_status_t edma_free_channel(edma_channel_t *chn);

/*!
 * @brief Starts an eDMA channel.
 *
 * Starts an eDMA channel. The driver starts an eDMA channel by enabling the DMA request. The software start
 * bit is not used in the eDMA peripheral driver.
 *
 * @param chn Memory pointing to the eDMA channel.
 *
 */
edma_status_t edma_start_channel(edma_channel_t *chn);

/*!
 * @brief Stops an eDMA channel.
 *
 * This function stops an eDMA channel and updates the descriptor chain status. By calling the edma_get_descriptor_status() function,
 * the ongoing left bytes of the descriptor are updated and the user can simultaneously get the status
 * of all descriptors.
 *
 * @param chn Memory pointing to the eDMA channel
 */
edma_status_t edma_stop_channel(edma_channel_t *chn);

/*!
 * @brief Updates the status of a particular descriptor in the eDMA.
 *
 * This function is designed for the loop descriptor chain. When descriptor is chained in loop
 * mode, the DMA constantly consumes descriptors. At the same time, the user may need to update
 * the content belonging to a consumed descriptor. This function is used to update the descriptor 
 * state from "CONSUMED" to "TO BE CONSUMED". In this case, the DMA driver can work out whether the 
 * underflow happens on the loop descriptor chain. 
 * This function can only update descriptors one-by-one in a sequence but not a specified
 * descriptor.
 *
 * @param chn Memory pointing to the eDMA channel
 */
edma_status_t edma_update_descriptor(edma_channel_t *chn);

/*!
 * @brief Configures the DMA transfer in a scatter-gather mode.
 *
 * This function configures descriptors in a loop chain. The user passes a block of memory into this
 * function. The memory is divided into "period" sub blocks. The DMA driver  configures "period"
 * descriptors. Each descriptor stands for a sub block. The DMA driver  transfers data from the 1st
 * descriptor to the last descriptor. Then, the DMA driver  wraps to the first descriptor to continue
 * the loop. The interrupt handler is called on every finish of a descriptor. The user can find out whether a descriptor is in the process of being transferred,
 * is already transferred, or to be transferred by calling the edma_get_descriptor_status() function in the interrupt handler or any
 * other task context. At the same
 * time, the user can call the edma_update_descriptor() function to tell the DMA driver that the content belonging to 
 * a descriptor is already updated and the DMA needs to count it as and underflow next time it
 * loops to this descriptor.
 *
 * @param chn Memory pointing to the eDMA channel
 * @param stcd Memory pointing to software TCDs. The user must prepare this memory block. The required
 * memory size is equal to a "period" * size of(edma_software_tcd_t). At the same time, the "stcd"
 * must align with 32 bytes. If not, an error occurs in the eDMA driver.
 * @param srcAddr A source register address or a start memory address.
 * @param destAddr A destination register address or a start memory address.
 * @param size Size to be transferred on every DMA write/read. Source/Dest share the same write/read
 * size.
 * @param watermark size write/read for every trigger of the DMA request. 
 * @param length Total length of Memory.
 * @period A number of the descriptor that is configured for this transfer configuration.
 */
edma_status_t edma_config_loop(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t srcAddr, uint32_t destAddr, uint32_t size,
                            uint32_t watermark, uint32_t length, uint8_t period);

/*!
 * @brief Configures the DMA transfer in scatter-gather mode.
 *
 * This function configure descriptors into sing-end chain. User passed blocks of memory into 
 * this function. Interrupt will only be triggered on the last memory block is finished. The
 * information of memory blocks are passed by edma_scatter_list_t data structure which can tell
 * the memory address and length.
 * DMA driver will configure descriptor for every memory block and transfer descriptor from the 
 * first one to the last one and then stop.
 *
 * @param chn Memory pointing to eDMA channel.
 * @param stcd Memory pointing to software TCDs. The user must prepare this memory block. The required
 * memory size is equal to the "number" * size of(edma_software_tcd_t). At the same time, the "stcd"
 * must align with 32 bytes. If not, an error occurs in the eDMA driver.
 * @param type Transfer type.
 * @param srcScatterList Data structure storing the address and length to be transferred for source
 * memory blocks. If source memory is peripheral, the length is not used.
 * @param destScatterList Data structure storing the address and length to be transferred for dest
 * memory blocks. If in the memory-to-memory transfer mode, the user must ensure that the length of the dest
 * scatter gather list is equal to the source scatter gather list. If the dest memory is a peripheral
 * register, the length is not used.
 * @param size Size to be transferred on each DMA write/read. Source/Dest share the same write/read
 * size.
 * @param watermark Size write/read for each trigger of the DMA request. 
 * @prame number A number of memory block contained in the scatter gather list.
 */
edma_status_t edma_config_scatter_gather(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t size, uint32_t watermark,
                            edma_scatter_list_t *srcScatterList, edma_scatter_list_t *destScatterList,
                            uint8_t number);

/*!
 * @brief eDMA IRQ Handler
 *
 */
void EDMA_IRQ_HANDLER(uint32_t channel);

/*!
 * @brief eDMA ERROR IRQ Handler
 *
 */
void DMA_ERR_IRQHandler(uint32_t instance);
/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_EDMA_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

