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
#include <stdlib.h>
#include <string.h>
#include "fsl_edma_request.h"
#include "fsl_edma_hal.h"
#include "fsl_dmamux_hal.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"
#include "fsl_os_abstraction.h"

/*!
 * @addtogroup edma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Status of DMA hardware descriptor
 *
 * Define the status of DMA descriptor. The status of descriptor is not limited to
 * these 2 states. It can also be any the value between 0~0xFFFFFFFF to stand for
 * left bytes for specified descriptor.
 */
typedef enum _edma_descriptor_internal_status {
    kEdmaDescriptorDone = 0U,				/*!< The descriptor is already finished */
    kEdmaDescriptorPrepared = 0xFFFFFFFFU	/*!< The descriptor is being consumed or not consumed. */
} edma_descriptor_internal_status_t;

/*! @brief Alignment for eDMA TCD start address. */
typedef enum _edma_tcd_alignment {
    kEdmaTcdAlignment = 0x20U,        /*!< Alignment of eDMA TCD. */
    kEdmaTcdAlignmentMask = 0x1FU     /*!< Alignment mask of eDMA TCD. */
} edma_tcd_alignment_t;

/*!
 * @brief Constant for status of DMA allocation.
 *
 * Structure used for dma_request_channel.
 */
typedef enum _edma_channel_type {
    kEdmaInvalidChannel = 0xFFU, /*!< Macros indicating the failure of channel request. */
    kEdmaAnyChannel = 0xFEU      /*!< Macros used when requesting channel.  */
                                /*!< kEdmaAnyChannel means a request of dynamically channel allocation. */
} edma_channel_type_t;

/*!
 * @brief Channel status for eDMA channel.
 *
 * Structure describing the status of DMA channel. User can get the status from channel callback
 * function.
 */
typedef enum _edma_channel_status {
	kEdmaIdle,				    /*!< DMA channel is idle. */
	kEdmaNormal,				    /*!< DMA channel is occupied. */
	kEdmaError				    /*!< Error happens in DMA channel. */
} edma_channel_status_t;

/*! @brief Type for DMA transfer. */
typedef enum _edma_transfer_type {
	kEdmaPeripheralToMemory,	    /*!< Transfer from peripheral to memory */
	kEdmaMemoryToPeripheral,	    /*!< Transfer from memory to peripheral */
	kEdmaMemoryToMemory,		    /*!< Transfer from memory to memory */
	kEdmaPeripheralToPeripheral  /*!< Transfer from peripheral to peripheral */
} edma_transfer_type_t;

/*!
 * @brief Definition for DMA channel's callback function.
 *
 * Prototype for callback function registered into DMA driver.
 */
typedef void (*edma_callback_t)(void *parameter, edma_channel_status_t status);

/*! @brief Data structure for configuring discrete memory transfer. */
typedef struct EdmaScatterList {
	uint32_t address;			/*!< Address of buffer. */
	uint32_t length;		    /*!< Lenght of buffer. */
} edma_scatter_list_t;

/*! @brief data struct for DMA channel. */
typedef struct EdmaChannel {
    uint8_t module;					/*!< eDMA physical Module indicator. */
    uint8_t channel;				/*!< eDMA physical channel indicator. */
    uint8_t dmamuxModule;			/*!< DMA mux module indicator. */
    uint8_t dmamuxChannel;			/*!< DMA mux channel indicator. */

    edma_callback_t callback;		/*!< Callback for edma channel. */
    void *parameter;				/*!< Parameter for callback function. */

    edma_channel_status_t status;	/*!< Channel status. */
    uint8_t tcdNumber;				/*!< Length of hardware descriptor chain. */
    uint8_t tcdWrite;               /*!< Indicator for user's descriptor updating. */
    uint8_t tcdRead;                /*!< Indicator for DMA controller's consuming. */ 
    uint32_t tcdLeftBytes;          /*!< Left bytes to be transferred for current tcd. */
    bool tcdUnderflow;              /*!< Flag telling whether user failed to feed descriptor in
                                         time. */
} edma_channel_t;

/*! @brief data struct for eDMA device */
typedef struct EdmaDevice {
    edma_channel_t *edmaChan[FSL_FEATURE_DMA_DMAMUX_CHANNELS];	/*!< Data pointer array for
                                                                     eDMA channel. */
    sync_object_t sema;	                                        /*!< Semaphore for eDMA driver. */
} edma_device_t;


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
 * @brief Initialize eDMA.
 *
 */
edma_status_t edma_init(void);

/*!
 * @brief Deinitilize eDMA.
 *
 */
edma_status_t edma_shutdown(void);

/*!
 * @brief Register callback function and parameter.
 *
 * User register callback function and parameter for specified eDMA channel. When channel
 * interrupt or channel error happens. The callback will be called and a parameter along
 * with user parameter will be provided to tell the channel status.
 * 
 * @param chn Handler for eDMA channel.
 * @param callback Callback function.
 * @param para parameter for callback functions.
 *
 */
void edma_register_callback(edma_channel_t *chn, edma_callback_t callback, void *para);

/*!
 * @brief Get the status of eDMA channel descriptor chain.
 *
 * This function can tell user the status of descriptor chains. User need to provide the memory
 * space to store the descriptor status. parameter descriptorStatus is the pointer pointing to 
 * descriptorStatus. If the descriptorStaus is not able to point to a valid memory space and the
 * length of memory is not enough to store the descriptor status. Error will happen inside DMA
 * driver. Every descriptor need a uint32_t to store the status.
 * If the return descriptorStatus[n] is equal = kEdmaDescriptorPrepared, it means the descriptor
 * is consuming or it is already prepared but not consumed. For other value, it means the left
 * bytes to be transferred for this descriptor. If the value is equal to 0, it means the descriptor
 * is already finished. User can update the memory belong to this descriptor safely.
 * To get a precise status of descriptor, user can first stop the channel. The ongoing descriptor
 * will be updated with a value indicating bytes to be transferred for this descriptor. If not,
 * its descriptor will only tell that this descriptor is ongoing with a value of
 * kEdmaDescriptorPrepared.
 *
 * @param chn Handler for eDMA channel.
 * @param descriptorStatus Status of descriptors.
 *
 */
edma_status_t edma_get_descriptor_status(edma_channel_t *chn, uint32_t *descriptorStatus);

/*!
 * @brief Request an eDMA channel.
 *
 * This function provide to ways to allocate a DMA channel. The first way is statically allocation.
 * The second way is dynamically allocation.
 * To allocate a channel dynamically, User need to set the channel parameter with the value of 
 * kDmaAnyChannel. Driver would search into all available free channel and assign the first searched
 * channel to user.
 * For the statically allocation, user need to set the channel parameter with the value of specified
 * channel. If the channel is available, Driver will assign the channel for user.
 * Notes: User is responsible to provide the handler memory for DMA channel. Driver will initialize
 * the handler and configure the handler memory.
 *
 * @param channel eDMA channel number. If channel is assigned with a valid channel number,
 * DMA driver will try to assign specified channel to user. If channel is assigned with
 * kDmaAnyChannel, DMA driver will search all available channels and assign the first channel to User.
 * @param source eDMA hardware request.
 * @param chan Memory pointing to eDMA channel. User need to assure the handler memory is valid and
 * it would not be released or changed by other codes before channel free operation.
 *
 * @return If channel allocation is successfully, the return value tell the requested channel. If
 * not, driver will return a value kDmaInvalidChannel to tell request operation is failed.
 */
uint32_t edma_request_channel(uint32_t channel, dma_request_source_t source, edma_channel_t *chan);

/*!
 * @brief Free eDMA channel's hardware and software resource.
 *
 * This function free relevant software and hardware resource. Request and Free operation need to 
 * be called in pair. 
 *
 * @param chn Memory pointing to eDMA channel.
 *
 */
edma_status_t edma_free_channel(edma_channel_t *chn);

/*!
 * @brief Start an eDMA channel.
 *
 * Start an eDMA channel. Driver start an eDMA channel by enable the DMA request. Software start
 * bit is not used in this eDMA Peripheral driver.
 *
 * @param chn Memory pointing to eDMA channel.
 *
 */
edma_status_t edma_start_channel(edma_channel_t *chn);

/*!
 * @brief Stop an eDMA channel.
 *
 * This function stop a eDMA channel and update the descriptor chain status. At the same time, the
 * ongoing descriptor's left bytes to be transferred will be updated and user can get the status
 * of all descriptor by calling edma_get_descriptor_status().
 *
 * @param chn Memory pointing to eDMA channel.
 */
edma_status_t edma_stop_channel(edma_channel_t *chn);

/*!
 * @brief Update the status of particular descriptor in eDMA
 *
 * This function is designed for the loop descriptor chain. When descriptor is chained in loop
 * mode, DMA will consume descriptors again and again. At the same time, user may need to update
 * the content belong to consumed descriptor. This function is used to update the descriptor 
 * state from "CONSUMED" to "TO BE CONSUMED". In this case, DMA driver can work out whether 
 * underflow happens on the loop descriptor chain. 
 * This function can only update descriptor one-by-one in sequence but not for specified
 * descriptor.
 *
 * @param chn Memory pointing to eDMA channel.
 */
edma_status_t edma_update_descriptor(edma_channel_t *chn);

/*!
 * @brief Configure DMA transfer in scatter-gather mode.
 *
 * This function configure descriptors into loop chain. User passed a block of memory into this
 * function. This memory are divided into "period" sub blocks. DMA driver will configure "period"
 * descriptors. Each descriptor stands for a sub block. DMA driver will transfer data from 1st
 * descriptor to the end of descriptor. Then DMA driver will wrap to the first descriptor to continue
 * the loop. Interrupt handler is called on every finish of descriptor. In the interrupt handler or
 * other task context, user can get to know whether an specified descriptor are being transferred,
 * already transferred, or to be transferred by calling edma_get_descriptor_status(). At the same
 * time, User can call edma_update_descriptor() to tell DMA driver that the content belong to 
 * some descriptor is already updated thus DMA need count it as and underflow while DMA next time
 * loop to this descriptor.
 *
 * @param chn Memory pointing to eDMA channel.
 * @param stcd Memory pointing to software tcds. User must prepare this memory block. The required
 * memory size is equal to "period" * size of(edma_software_tcd_t). At the same time, the "stcd"
 * must align with 32 bytes. If not, error will happen on eDMA driver.
 * @param srcAddr Source register address or start memory address.
 * @param destAddr Destination register address or start memory address.
 * @param size size to be transferred on every DMA write/read. Source/Dest share the same write/read
 * size.
 * @param watermark size write/read for every trigger of DMA request. 
 * @param length Total length of Memory.
 * @period number of descriptor will be configured for this transfer configuration.
 */
edma_status_t edma_config_loop(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t srcAddr, uint32_t destAddr, uint32_t size,
                            uint32_t watermark, uint32_t length, uint8_t period);

/*!
 * @brief Configure DMA transfer in scatter-gather mode.
 *
 * This function configure descriptors into sing-end chain. User passed blocks of memory into 
 * this function. Interrupt will only be triggered on the last memory block is finished. The
 * information of memory blocks are passed by edma_scatter_list_t data structure which can tell
 * the memory address and length.
 * DMA driver will configure descriptor for every memory block and transfer descriptor from the 
 * first one to the last one and then stop.
 *
 * @param chn Memory pointing to eDMA channel.
 * @param stcd Memory pointing to software tcds. User must prepare this memory block. The required
 * memory size is equal to "number" * size of(edma_software_tcd_t). At the same time, the "stcd"
 * must align with 32 bytes. If not, error will happen on eDMA driver.
 * @param type transfer type.
 * @param srcScatterList data structure storing the address and length to be transferred for source
 * memory blocks. If source memory is peripheral, length is not used.
 * @param destScatterList data structure storing the address and length to be transferred for dest
 * memory blocks. If in memory to memory transfer mode, User need to assure the length of dest
 * scatter gather list are equal to source scatter gather list. If dest memory is peripheral
 * register, length is not used.
 * @param size size to be transferred on every DMA write/read. Source/Dest share the same write/read
 * size.
 * @param watermark size write/read for every trigger of DMA request. 
 * @prame number number of memory block contained in the scatter gather list.
 */
edma_status_t edma_config_scatter_gather(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t size, uint32_t watermark,
                            edma_scatter_list_t *srcScatterList, edma_scatter_list_t *destScatterList,
                            uint8_t number);

/*!
 * @brief eDMA IRQ handler.
 *
 */
void EDMA_IRQ_HANDLER(uint32_t channel);

/*!
 * @brief eDMA ERROR IRQ Handler.
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

