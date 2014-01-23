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
#ifndef __EDMA_HAL_H__
#define __EDMA_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_edma_features.h"
#include "fsl_device_registers.h"
#include <assert.h>

/*!
 * @addtogroup edma_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Edma status */
typedef enum _edma_status
{
    kStatus_EDMA_Success = 0U,
    kStatus_EDMA_InvalidArgument = 1U,  /*!< Parameter is not available for current configuration */
    kStatus_EDMA_Fail = 2U              /*!< Function operation is failed */
} edma_status_t;

/*! @brief Edma TCD control configuration */
typedef union EdmaTCDControl {
    struct {
        uint16_t reserve1 : 1;
        uint16_t majorInterrupt : 1;        /*!< Interrupt after major loop complete. */
        uint16_t halfInterrupt : 1;         /*!< Interrupt after half of major loop complete. */
        uint16_t disabledDmaRequest : 1;    /*!< Disabled dma request after major loop complete. */
        uint16_t enabledScatterGather : 1;  /*!< Enable scatter/gather processing. */
        uint16_t enableMajorLink : 1;       /*!< Enabled major link after major loop complete. */
        uint16_t reserve2 : 1;
        uint16_t reserve3 : 1;
        uint16_t majorLinkChannel : 4;      /*!< Major link channel number.*/
        uint16_t reserve4 : 2;
        uint16_t bandwidthControl : 2;      /*!< Bandwidth control configuration. */
    } U;
    uint16_t B;
} edma_tcd_control_t;

/*! @brief Edma TCD Minor loop mapping configuration. */
typedef struct EdmaMinorLoopOffset {
    bool isEnableSourceMinorloop;
    bool isEnableDestMinorloop;
    uint32_t offset;
} edma_minorloop_offset_config_t;

/*! @brief Priorty limitation of Edma channel */
typedef enum _edma_channel_priority {
    kEdmaChannelPriority = 16
} edma_channel_priority_t;

/*! @brief Edma modulo configuration. */
typedef enum _edma_modulo {
    kEdmaModuloDisable = 0x0U,
    kEdmaModulo2bytes = 0x1U,
    kEdmaModulo4bytes = 0x2U,
    kEdmaModulo8bytes = 0x3U,
    kEdmaModulo16bytes = 0x4U,
    kEdmaModulo32bytes = 0x5U,
    kEdmaModulo64bytes = 0x6U,
    kEdmaModulo128bytes = 0x7U,
    kEdmaModulo256bytes = 0x8U,
    kEdmaModulo512bytes = 0x9U,
    kEdmaModulo1Kbytes = 0xaU,
    kEdmaModulo2Kbytes = 0xbU,
    kEdmaModulo4Kbytes = 0xcU,
    kEdmaModulo8Kbytes = 0xdU,
    kEdmaModulo16Kbytes = 0xeU,
    kEdmaModulo32Kbytes = 0xfU,
    kEdmaModulo64Kbytes = 0x10U,
    kEdmaModulo128Kbytes = 0x11U,
    kEdmaModulo256Kbytes = 0x12U,
    kEdmaModulo512Kbytes = 0x13U,
    kEdmaModulo1Mbytes = 0x14U,
    kEdmaModulo2Mbytes = 0x15U,
    kEdmaModulo4Mbytes = 0x16U,
    kEdmaModulo8Mbytes = 0x17U,
    kEdmaModulo16Mbytes = 0x18U,
    kEdmaModulo32Mbytes = 0x19U,
    kEdmaModulo64Mbytes = 0x1aU,
    kEdmaModulo128Mbytes = 0x1bU,
    kEdmaModulo256Mbytes = 0x1cU,
    kEdmaModulo512Mbytes = 0x1dU,
    kEdmaModulo1Gbytes = 0x1eU,
    kEdmaModulo2Gbytes = 0x1fU
} edma_modulo_t;

/*! @brief Edma transfer size configuration. */
typedef enum _edma_transfer_size {
    kEdmaTransferSize1bytes = 0x0U,
    kEdmaTransferSize2bytes = 0x1U,
    kEdmaTransferSize4bytes = 0x2U,
    kEdmaTransferSize16bytes = 0x4U,
    kEdmaTransferSize32bytes = 0x5U
} edma_transfer_size_t;

/*! @brief Error status of Edma module. */
typedef union EdmaErrorStatusAll {
    struct {
        uint32_t destinationBusError : 1;               /*!< Bus error on dest address. */
        uint32_t sourceBusError : 1;                    /*!< Bus error on Src address. */
        uint32_t scatterOrGatherConfigurationError : 1; /*!< Error on Scatter/Gather address. */
        uint32_t nbyteOrCiterConfigurationError : 1;    /*!< NBYTES/CITER configuration error. */
        uint32_t destinationOffsetError : 1;            /*!< Destination offset error. */
        uint32_t destinationAddressError : 1;           /*!< Destination address error. */
        uint32_t sourceOffsetError : 1;                 /*!< Source offset error. */
        uint32_t sourceAddressError : 1;                /*!< Source adderss error. */
        uint32_t errorChannel : 5;                      /*!< Error channel number of cancelled
                                                             channel number. */
        uint32_t _reserved1 : 1;
        uint32_t channelPriorityError : 1;              /*!< Channel priority error. */
        uint32_t groupPriorityError : 1;                /*!< Group priority error. */
        uint32_t transferCancelledError : 1;            /*!< Transfer cancelled. */
        uint32_t _reserved0 : 14;
        uint32_t orOfAllError : 1;                      /*!< Logical OR of all ERR status bits. */
    } U;
    uint32_t B;
} edma_error_status_all_t;

/*! @brief Bandwidth control configuration. */
typedef enum _edma_bandwidth_configuration {
    kEdmaBandwidthStallNone = 0,    /*!< No eDMA engine stalls. */
    kEdmaBandwidthStall4Cycle = 2,  /*!< eDMA engine stalls for 4 cycles after each r/w. */
    kEdmaBandwidthStall8Cycle = 3   /*!< eDMA engine stalls for 4 cycles after each r/w. */
} edma_bandwidth_configuration_t;

/*! @brief Edma TCD. */
typedef struct EdmaSoftwareTcd {
    uint32_t SADDR;
    uint16_t SOFF;
    uint16_t ATTR;
    union {
        uint32_t NBYTES_MLNO;
        uint32_t NBYTES_MLOFFNO;
        uint32_t NBYTES_MLOFFYES;
    };
    uint32_t SLAST;
    uint32_t DADDR;
    uint16_t DOFF;
    union {
        uint16_t CITER_ELINKNO;
        uint16_t CITER_ELINKYES;
    };
    uint32_t DLAST_SGA;
    uint16_t CSR;
    union {
        uint16_t BITER_ELINKNO;
        uint16_t BITER_ELINKYES;
    };
} edma_software_tcd_t;

/*! @brief Edma group priority. */
typedef enum _edma_group_priority {
    kEdmaGroup0Priority0Group1Priority1,
    kEdmaGroup0Priority1Group1Priority0
} edma_group_priority_t;

/*! @brief DMA configuration structure. */
typedef struct EdmaConfiguration {
    bool isEnableMinorLoopping;         /*!< Enabled minor loop mapping. */
    bool isEnableContinuousMode;        /*!< Enabled continuous mode. */
	bool isHaltOnError;                 /*!< Halt if error happens. */
    bool isEnableRoundrobinArbitration; /*!< Enabled roundrobin or fixed priority arbitration. */
    bool isEnableDebug;                /*!< Enabled Debug mode. */
#if (FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
    edma_group_priority_t groupPriority;
    bool isEnableGroupRoundrobinArbitration;
#endif
} edma_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! 
 * @name EDMA hal common configuration. 
 * @{
 */

/*!
 * @brief Init edma module.
 *
 * The function would configure the edma module with the corresponding global configuration. These
 * configuration is for all channels in this moduel. 
 *
 * @param module Edma module.
 * @param init Init data structure.
 */
void edma_hal_init(uint32_t instance, edma_config_t *init);

/*!
 * @brief Cancel remaining data transfer. Stop the executing channel force the minor loop
 * to finish.The Cancel takes effect after the last write of the current read/write sequence.
 * The CX clears itself after the cancel has been honored. This cancel retires the channel 
 * normally as if the mino loop was completed.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_cancel_transfer(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_CX(instance, 1U);
    while (BR_DMA_CR_CX(instance))
    {}
}

/*!
 * @brief Cancel remaining data transfer. Stop the executing channel and force the minor loop to
 * finish. The cancel takes effect after the last write of the current read/write sequence. The
 * ECX bit clears itself after the cancel is honored. In addition to cancelling the transfer, ECX
 * treats the cancel as an error condition. 
 *
 * @param instance Edma module.
 */
static inline void edma_hal_error_cancel_transfer(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_ECX(instance, 1U);
    while(BR_DMA_CR_ECX(instance))
    {}
}

/*!
 * @brief Enable/Disable the minor loop mapping.
 *
 * If enabled, NBYTES is redefined to include individual enable fields. and the NBYTES field. The
 * individual enable fields allow the minor loop offset to be applied to the source address, the 
 * destination address, or both. The NBYTES field is reduced when either offset is enabled.
 *
 * @param instance Edma module.
 * @param isEnabled Enable or disable.
 */
static inline void edma_hal_set_minor_loop_mapping(uint32_t instance, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_EMLM(instance, isEnabled);
}


#if (FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
/*!
 * @brief Configure group priority.
 *
 * @param module Edma module.
 * @param isContinuous Whether the minor loop's finished would trigger itself.
 */
static inline void edma_hal_set_group_priority(uint32_t instance, edma_group_priority_t groupPriority)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);

    if (groupPriority == kEdmaGroup0Priority0Group1Priority1)
    {
        BW_DMA_CR_GRP0PRI(instance, 0U);
        BW_DMA_CR_GRP1PRI(instance, 1U);
    }
    else
    {
        BW_DMA_CR_GRP0PRI(instance, 1U);
        BW_DMA_CR_GRP1PRI(instance, 0U);

    }
}

/*!
 * @brief Fixed priority arbitration is used for group selection.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_set_fixed_priority_group_arbitration(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_ERGA(instance, 0U);
}

/*!
 * @brief Round robin arbitration is used for group selection.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_set_roundrobin_group_arbitration(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_ERGA(instance, 1U);
}

#endif

/*!
 * @brief Configure the continuous mode.If set, a minor loop channel link made to itself does not
 * go through channel arbitration before being activated again.Upon minor loop completion, the
 * channel activates again if that channel has a minor loop channel link enabled and the link
 * channel is itself. 
 *
 * @param module Edma module.
 * @param isContinuous Whether the minor loop's finished would trigger itself.
 */
static inline void edma_hal_set_continuous_mode(uint32_t instance, bool isContinuous)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_CLM(instance, isContinuous);
}

/*!
 * @brief Halt DMA Operations.
 *
 * Stall the start of any new channels. Executing channels are allowed to complete. 
 *
 * @param instance Edma module.
 */
static inline void edma_hal_halt(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_HALT(instance, 1U);
}

/*!
 * @brief Clear the halt bit.
 *
 * If a previous edma channel is halted, clear operaion would resume it back to executing.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_clear_halt(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_HALT(instance, 0U);
}

/*!
 * @brief Halt edma module when error happens.
 *
 * Any error causes the HALT bit to set. subsequently, all service requests are ignored until the
 * HALT bit is cleared.
 *
 * @param instance Edma module.
 * @param isHaltOnError halt or not halt when error happens. 
 */
static inline void edma_hal_set_halt_on_error(uint32_t instance, bool isHaltOnError)
{	
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_HOE(instance, isHaltOnError);
}

/*!
 * @brief Fixed priority arbitration is used for channel selection.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_set_fixed_priority_channel_arbitration(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_ERCA(instance, 0U);
}

/*!
 * @brief Round robin arbitration is used for channel selection.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_set_roundrobin_channel_arbitration(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_ERCA(instance, 1U);
}

/*!
 * @brief Enable/Disable edma DEBUG mode.
 *
 * When in debug mode, the DMA stalls the start of a new 
 * channel. Execulting channels are allowed to complete. Channel execution resumes when the system
 * when the system exits debug mode or the EDBG bit is cleared. 
 *
 * @param instance Edma module.
 */
static inline void edma_hal_set_debug_mode(uint32_t instance, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CR_EDBG(instance, isEnabled);
}

/*!
 * @brief Get the error status of edma module. The detailed reason is listed along with the error
 * channel.
 *
 * @param instance Edma module.
 * @return Detailed information of error type in edma module.
 */
static inline uint32_t edma_hal_get_error_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    return HW_DMA_ES_RD(instance);
}

/*!
 * @brief Disable interrupt when error happens on any of channel in the edma module.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_disable_all_enabled_error_interrupt(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    BW_DMA_CEEI_CAEE(instance, 1U);
}

/*!
 * @brief Enable interrupt when error happens on any channel in edma module.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_enable_all_channel_error_interrupt(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);

    BW_DMA_SEEI_SAEE(instance, 1U);
}

/*!
 * @brief Disable dma request for all edma channels.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_disable_all_channel_dma_request(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_CERQ_WR(instance, DMA_CERQ_CAER_MASK);
}

/*!
 * @brief Enable dma request for all edma channels.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_enable_all_channel_dma_request(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_SERQ_WR(instance, DMA_SERQ_SAER_MASK);
}

/*!
 * @brief Clear done status for all edma channels.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_clear_all_channel_done_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_CDNE_WR(instance, DMA_CDNE_CADN_MASK);
}

/*!
 * @brief Trigger all channel's start bits.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_trigger_all_channel_start_bit(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_SSRT_WR(instance, DMA_SSRT_SAST_MASK);
}

/*!
 * @brief Clear error status for all edma channels.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_clear_all_channel_error_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_CERR_WR(instance, DMA_CERR_CAEI_MASK);
}

/*!
 * @brief Clear interrupt request for all edma channles.
 *
 * @param instance Edma module.
 */
static inline void edma_hal_clear_all_channel_interrupt_request(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    HW_DMA_CINT_WR(instance, DMA_CINT_CAIR_MASK);
}

/*!
 * @brief Get the interrupt status for all edma channels.
 *
 * @param instance Edma module.
 * @return 32 bit data. Every bit stands for a edma channle. bit 0 stands for channel 0...
 */
static inline uint32_t edma_hal_get_all_channel_interrupt_request_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    return HW_DMA_INT_RD(instance);
}

/*!
 * @brief Get the channel error status for all edma channels.
 *
 * @param instance Edma module.
 * @return 32 bit data. every bit stands for a edma channle. bit 0 stands for channel 0...
 */
static inline uint32_t edma_hal_get_all_channel_error_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    return HW_DMA_ERR_RD(instance);
}

/*!
 * @brief Get the status of dma request for all dma channel.
 *
 * @param instance Edma module.
 * @return 32 bit data. every bit stands for a edma channle. bit 0 stands for channel 0...
 */
static inline uint32_t edma_hal_get_all_channel_dma_request_status(uint32_t instance)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    return HW_DMA_HRS_RD(instance);
}

/* @} */

/*! 
 * @name EDMA hal channel configuration. 
 * @{
 */

/*!
 * @brief Check if the channel dma request is enabled.
 *
 * Check whether the specified channel's dma request is enabled.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 *
 * @return True stands for enabled. False stands for disbled.
 */
static inline bool edma_hal_check_dma_request_enable_status(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);

    return ((HW_DMA_ERQ_RD(instance)>>channel) & 1U); 
}

/*!
 * @brief Disable interrupt when error happens on edma channel.
 *
 * Disable error interrupt for eDMA module.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_disable_error_interrupt(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_CEEI_WR(instance, DMA_CEEI_CEEI(channel));
}

/*!
 * @brief Enable interrupt when error happens on edma channel 
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_enable_error_interrupt(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_SEEI_WR(instance, DMA_SEEI_SEEI(channel));
}

/*!
 * @brief Disable dma request for edma channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_disable_dma_request(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_CERQ_WR(instance, DMA_CERQ_CERQ(channel)); 
}

/*!
 * @brief Enable dma request for specified edma channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_enable_dma_request(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_SERQ_WR(instance, DMA_SERQ_SERQ(channel));
}

/*!
 * @brief Clear done status for edma channel.
 *
 * DONE status of dma channel would be cleared. If the scatter/gather state is
 * enabled, the DONE status in CSR maybe cleared but the global DONE statue is
 * still set. This function is to clear the global done state.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_clear_done_status(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_CDNE_WR(instance, DMA_CDNE_CDNE(channel));
}

/*!
 * @brief Start edma channel manually.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_trigger_start_bit(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_SSRT_WR(instance, DMA_SSRT_SSRT(channel));
}

/*!  * @brief Clear error status for edma channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_clear_error_status(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_CERR_WR(instance, DMA_CERR_CERR(channel));
}

/*!
 * @brief Clear interrupt request for edma channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_clear_interrupt_request(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    HW_DMA_CINT_WR(instance, DMA_CINT_CINT(channel));
}

#if (FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT > 0x0U)
/*!
 * @brief Enable/Disable asynchronous request in stop mode.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_set_asynchronous_request_in_stop_mode(
                    uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    if(isEnabled) 
    {
        HW_DMA_EARS_SET(instance, 1U << channel);
    }
    else
    {
        HW_DMA_EARS_CLR(instance, 1U << channel);
    }
}
#endif

/*!
 * @brief Configure the preemp feature for edma channel.
 *
 * If it is disabled, DMA channel can't suspend a lower priority channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param preempt configuration mode for preempt.
 */
static inline void edma_hal_set_channel_preemp_ability(
                uint32_t instance, uint32_t channel, bool isDisabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_DCHPRIn_DPA(instance, HW_DMA_DCHPRIn_CHANNEL(channel), isDisabled);
}

/*!
 * @brief Configure the preemp feature for edma channel.
 *
 * If enabled, channel can be temporarily suspended by a higher priority channel.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param preempt configuration mode for preempt.
 */
static inline void edma_hal_set_channel_preemption_ability(uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_DCHPRIn_ECP(instance, HW_DMA_DCHPRIn_CHANNEL(channel), isEnabled);
}

/*!
 * @brief Configure edma channel priority.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param priority Priority of dma channel. different channel should have differnt priority inside a
 * group.
 */
static inline void edma_hal_set_channel_priority(
                uint32_t instance, uint32_t channel, uint32_t priority)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);

    BW_DMA_DCHPRIn_CHPRI(instance, HW_DMA_DCHPRIn_CHANNEL(channel), priority);
}

/* @} */

/*! 
 * @name EDMA hal hardware TCD configruation. 
 * @{
 */

/*!
 * @brief Configure source address for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param address memory address pointing to the source data.
 */
static inline void edma_hal_htcd_configure_source_address(
                uint32_t instance, uint32_t channel, uint32_t address)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_SADDR_SADDR(instance, channel, address);
}

/*!
 * @brief Configure source address signed offset for hardware TCD.
 *
 * Sign-extended offset applied to the current source address to form the next-state value as each
 * source read is complete.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param offset signed-offset.
 */
static inline void edma_hal_htcd_configure_source_offset(
                uint32_t instance, uint32_t channel, int16_t offset)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_SOFF_SOFF(instance, channel, offset);
}

/*!
 * @brief Configure source modulo for hardware TCD.
 *
 * The value defines a specific address range specified to be the value after SADDR + SOFF
 * calculation is performed on the original register value. Setting this field provides the ability
 * to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 * queue should start at a 0-modulo-size address and the SMOD field should be set to the appropriate
 * value for the queue, freezing the desired number of upper address bits. The value programmed into
 * this field specifies the number of lower address bits allowed to change. For a circular queue
 * application, the SOFF is typically set to the ransfer size to implement post-increment addressing
 * with SMOD function constraining the addresses to a 0-modulo-size range.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param modulo enum type for allowed modulo.
 */
static inline void edma_hal_htcd_configure_source_modulo(
                uint32_t instance, uint32_t channel, edma_modulo_t modulo)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_ATTR_SMOD(instance, channel, modulo);
}

/*!
 * @brief Configure source data transfersize for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param size enum type for transfer size.
 */
static inline void edma_hal_htcd_configure_source_transfersize(
                uint32_t instance, uint32_t channel, edma_transfer_size_t size)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_ATTR_SSIZE(instance, channel, size);
}

/*!
 * @brief Configure destination modulo for hardware TCD.
 *
 * The value defines a specific address range specified to be the value after DADDR + DOFF
 * calculation is performed on the original register value. Setting this field provides the ability
 * to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 * queue should start at a 0-modulo-size address and the SMOD field should be set to the appropriate
 * value for the queue, freezing the desired number of upper address bits. The value programmed into
 * this field specifies the number of lower address bits allowed to change. For a circular queue
 * application, the SOFF is typically set to the ransfer size to implement post-increment addressing
 * with DMOD function constraining the addresses to a 0-modulo-size range.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param modulo enum type for allowed modulo.
 */
static inline void edma_hal_htcd_configure_dest_modulo(
                uint32_t instance, uint32_t channel, edma_modulo_t modulo) 
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_ATTR_DMOD(instance, channel, modulo);
}

/*!
 * @brief Configure dest data transfersize for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param size enum type for transfer size.
 */
static inline void edma_hal_htcd_configure_dest_transfersize(
                uint32_t instance, uint32_t channel, edma_transfer_size_t size)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_ATTR_DSIZE(instance, channel, size);
}

/*!
 * @brief Configure nbytes if minor loop mapping is diabled the for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_htcd_configure_nbytes_minorloop_disabled(
                uint32_t instance, uint32_t channel, uint32_t nbytes)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_NBYTES_MLNO_NBYTES(instance, channel, nbytes); 
}

/*!
 * @brief Configure nbytes if minor loop mapping is enabled and offset is disabled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_htcd_configure_nbytes_minorloop_enabled_offset_disabled(
                uint32_t instance, uint32_t channel, uint32_t nbytes)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_NBYTES_MLOFFNO_NBYTES(instance, channel, nbytes);
}

/*!
 * @brief Configure nbytes if minor loop mapping is enabled and offset is enabled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_htcd_configure_nbytes_minorloop_enabled_offset_enabled(
                uint32_t instance, uint32_t channel, uint32_t nbytes)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_NBYTES_MLOFFYES_NBYTES(instance, channel, nbytes);
}

/*!
 * @brief Get nbytes configuration data.
 *
 * This function would firstly decide if the minor loop mapping is enabled or if the source/dest
 * minor loop mapping is enabled. And then the nbytes would be returned accordingly.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return nbytes configuration.
 */
uint32_t edma_hal_htcd_get_nbytes_configuration(uint32_t instance, uint32_t channel);


/*!
 * @brief Configure minorloop offset for hardware TCD.
 *
 * Configure both the enable bits and offset value. If neither source nor dest offset is enabled,
 * offset would not be configured.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param config Configuration data structure for minorloop offset.
 */
static inline void edma_hal_htcd_configure_minorloop_offset(
        uint32_t instance, uint32_t channel, edma_minorloop_offset_config_t config)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_NBYTES_MLOFFYES_SMLOE(instance, channel, config.isEnableSourceMinorloop);
    BW_DMA_TCDn_NBYTES_MLOFFYES_DMLOE(instance, channel, config.isEnableDestMinorloop);
    if ((config.isEnableSourceMinorloop == true) || (config.isEnableDestMinorloop == true))
    {
        BW_DMA_TCDn_NBYTES_MLOFFYES_MLOFF(instance, channel, config.offset);
    }
}

/*!
 * @brief Configure last source address adjustment for hardware TCD.
 *
 * Adjustment value added to the source address at the completion of the major iteration count. This
 * value can be applied to restore the source address to the intial value, or adjust the address to
 * reference the next data structure.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param size adjustment value.
 */
static inline void edma_hal_htcd_configure_source_last_adjustment(
                uint32_t instance, uint32_t channel, int32_t size)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_SLAST_SLAST(instance, channel, size);
}

/*!
 * @brief Configure dest address for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param address memory address pointing to destination data.
 */
static inline void edma_hal_htcd_configure_dest_address(
                uint32_t instance, uint32_t channel, uint32_t address)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_DADDR_DADDR(instance, channel, address);
}

/*!
 * @brief Configure dest address signed offset for hardware TCD.
 *
 * Sign-extended offset applied to the current source address to form the next-state value as each
 * dest write is complete.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param offset signed-offset. 
 */
static inline void edma_hal_htcd_configure_dest_offset(
                uint32_t instance, uint32_t channel, int16_t offset)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_DOFF_DOFF(instance, channel, offset);
}

/*!
 * @brief Configure last source address adjustment or the memory address for the next transfer
 * control for hardware TCD.
 *
 * If scatter/gather feature is eanbled(edma_hal_htcd_set_scatter_gather_process()):
 *
 * This address points to the beginning of a 0-modulo-32 byte region containing the next transfer
 * control descriptor to be loaded into this channel. This channel reload is performed as the major
 * iteration count completes. The scatter/gather address must be 0-modulo-32-byte, else a
 * configuration error is reported.
 *
 * else:
 *
 * Adjustment value added to the source address at the completion of the major iteration count. This
 * value can be applied to restore the source address to the intial value, or adjust the address to
 * reference the next data structure.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param size adjustment value.
 */
static inline void edma_hal_htcd_configure_dest_last_adjustment_or_scatter_address(
        uint32_t instance, uint32_t channel, uint32_t address)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_DLASTSGA_DLASTSGA(instance, channel, address);
}

/*!
 * @brief Configure bandwidth for hardware TCD.
 *
 * Throttles the amount of bus bandwidth consumed by the eDMA. In general, as the eDMA processes the
 * minor loop, it continuously generates read/write sequences until the minor count is exhausted.
 * This field forces the eDMA to stall after the completion of each read/write access to control the
 * bus request bandwidth seen by the crossbar switch.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param bandwidth enum type for bandwidth control.
 */
static inline void edma_hal_htcd_configure_bandwidth(
        uint32_t instance, uint32_t channel, edma_bandwidth_configuration_t bandwidth)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_BWC(instance, channel, bandwidth);
}

/*!
 * @brief Configure major link channel number for hardware TCD.
 *
 * If majorlink is enabled, after the major loop counter is exhausted, the eDMA engine initiates a
 * channel service request at the channel defined by these six bits by setting that channel's start
 * bits.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param majorchannel channel number for major link.
 */
static inline void edma_hal_htcd_configure_majorlink_channel(
        uint32_t instance, uint32_t channel, uint32_t majorchannel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_MAJORLINKCH(instance, channel, majorchannel);
}

/*!
 * @brief Get the major link channel for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return major link channel number. 
 */
static inline uint32_t edma_hal_htcd_get_majorlink_channel( 
        uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    return BR_DMA_TCDn_CSR_MAJORLINKCH(instance, channel);
}

/*!
 * @brief Enable/Disable major link channel feature for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnabled Enable/Disable.
 */
static inline void edma_hal_htcd_set_majorlink(uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_MAJORELINK(instance, channel, isEnabled);
}

/*!
 * @brief Enable/Disable scatter/gather feature for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnabled Enable/Disable.
 */
static inline void edma_hal_htcd_set_scatter_gather_process(
        uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_ESG(instance, channel, isEnabled);
}

/*!
 * @brief Check if scatter/gather feature is enabled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return True stand for enabled. False stands for disabled. 
 */
static inline bool edma_hal_htcd_is_gather_scatter_enabled(
        uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    return BR_DMA_TCDn_CSR_ESG(instance, channel);

}

/*!
 * @brief Disable/Enable dma request after major loop complete for hardware TCD.
 *
 * If disabled, the eDMA hardware automatically clears the corresponding dma request when the
 * current major iteration count reaches zero.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isDisabled Disable/Enable.
 */
static inline void edma_hal_htcd_set_disable_dma_request_after_tcd_done(
        uint32_t instance, uint32_t channel, bool isDisabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_DREQ(instance, channel, isDisabled);
}  

/*!
 * @brief Enable/Disable half compelte interrupt for hardware TCD.
 *
 * If set, the channel generates an interrupt request by setting the appropriate bit in the
 * interrupt register when the current major iteration count reaches the halfway point. Specially,
 * the comparison performed by the eDMA engine is (CITER == (BITER >> 1)). This half way point
 * interrupt request is provided to support double-buffered schemes or other types of data movement
 * where the processor needs an early indication of the transfer's process.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_htcd_set_half_complete_interrupt(
        uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_INTHALF(instance, channel, isEnabled);
}

/*!
 * @brief Enable/Disable interrupt after major loop complete for hardware TCD.
 *
 * If enabled, the channel generates an interrupt request by setting the appropriate bit in the 
 * interrupt register when the current major iteration count reaches zero.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_htcd_set_complete_interrupt(
        uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_INTMAJOR(instance, channel, isEnabled);
}

/*!
 * @brief Trigger start bits for hardware TCD.
 *
 * The eDMA hardware automatically clears this flag after the channel begins execution.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 */
static inline void edma_hal_htcd_trigger_channel_start(
        uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CSR_START(instance, channel, 1); 
}

/*!
 * @brief Check if channel is running for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return True stands for running. False stands for not. 
 */
static inline bool edma_hal_htcd_is_channel_active(
        uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    return BR_DMA_TCDn_CSR_ACTIVE(instance, channel); 
}

/*!
 * @brief Check if major loop is exhausted for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return True stands for running. False stands for not. 
 */
static inline bool edma_hal_htcd_is_channel_done(
        uint32_t instance, uint32_t channel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    return BR_DMA_TCDn_CSR_DONE(instance, channel);
}

/*!
 * @brief Enable/Disable channel link after minor loop for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_htcd_set_minor_link(
        uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_BITER_ELINKYES_ELINK(instance, channel, isEnabled);
}

/*!
 * @brief Enable/Disable channel link after minor loop in current register for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_htcd_set_current_minor_link(
        uint32_t instance, uint32_t channel, bool isEnabled)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CITER_ELINKYES_ELINK(instance, channel, isEnabled);
}

/*!
 * @brief Configure minor loop link channel for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param minorchannel minor loop link channel.
 */
static inline void edma_hal_htcd_configure_minor_link_channel(
        uint32_t instance, uint32_t channel, uint32_t minorchannel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_BITER_ELINKYES_LINKCH(instance, channel, minorchannel);
}

/*!
 * @brief Configure minor loop link channel in current register for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param minorchannel minor loop link channel.
 */
static inline void edma_hal_htcd_configure_current_minor_link_channel(
        uint32_t instance, uint32_t channel, uint32_t minorchannel)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CITER_ELINKYES_LINKCH(instance, channel, minorchannel);
}

/*!
 * @brief Configure major count if minor loop channel link is disbled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param count major loop count
 */
static inline void edma_hal_htcd_configure_majorcount_minorlink_disabled(
        uint32_t instance, uint32_t channel, uint32_t count)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_BITER_ELINKNO_BITER(instance, channel, count);
}

/*!
 * @brief Configure current major count if minor loop channel link is disbled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param count major loop count
 */
static inline void edma_hal_htcd_configure_current_majorcount_minorlink_disabled(
        uint32_t instance, uint32_t channel, uint32_t count)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CITER_ELINKNO_CITER(instance, channel, count);
}

/*!
 * @brief Configure major count if minor loop channel link is enabled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param count major loop count
 */
static inline void edma_hal_htcd_configure_majorcount_minorlink_enabled(
        uint32_t instance, uint32_t channel, uint32_t count)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_BITER_ELINKYES_BITER(instance, channel, count);
}

/*!
 * @brief Configure current major count if minor loop channel link is enabled for hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param count major loop count
 */
static inline void edma_hal_htcd_configure_current_majorcount_minorlink_enabled(
        uint32_t instance, uint32_t channel, uint32_t count)
{
    assert(instance < HW_DMA_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_DMA_MODULE_CHANNEL);
    BW_DMA_TCDn_CITER_ELINKYES_CITER(instance, channel, count);
}

/*!
 * @brief Get current major loop count.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return current major loop count.
 */
uint32_t edma_hal_htcd_get_current_major_count(uint32_t instance, uint32_t channel);

/*!
 * @brief Get begin major loop count.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return begin major loop count.
 */
uint32_t edma_hal_htcd_get_begin_major_count(uint32_t instance, uint32_t channel);

/*!
 * @brief Get the bytes number not be transferred for this hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return data bytes to be transferred. 
 */
uint32_t edma_hal_htcd_get_unfinished_bytes(uint32_t instance, uint32_t channel);

/*!
 * @brief Get the bytes number already be transferred for this hardware TCD.
 *
 * @param instance Edma module.
 * @param channel Edma channel.
 * @return data bytes to be transferred. 
 */
uint32_t edma_hal_htcd_get_finished_bytes(uint32_t instance, uint32_t channel);
/* @} */

/*! 
 * @name EDMA hal software TCD configruation. 
 * @{
 */

/*!
 * @brief Configure source address for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param address memory address pointing to the source data.
 */
static inline void edma_hal_stcd_configure_source_address(
        edma_software_tcd_t *stcd, uint32_t address)
{
    assert(stcd);
    stcd->SADDR = DMA_SADDR_SADDR(address); 
}

/*!
 * @brief Configure source address for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param address memory address pointing to the source data.
 */
static inline void edma_hal_stcd_configure_source_offset(
                edma_software_tcd_t *stcd, uint32_t offset)
{
    assert(stcd);
    stcd->SOFF = DMA_SOFF_SOFF(offset);
}

/*!
 * @brief Configure source modulo for software TCD.
 *
 * The value defines a specific address range specified to be the value after SADDR + SOFF
 * calculation is performed on the original register value. Setting this field provides the ability
 * to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 * queue should start at a 0-modulo-size address and the SMOD field should be set to the appropriate
 * value for the queue, freezing the desired number of upper address bits. The value programmed into
 * this field specifies the number of lower address bits allowed to change. For a circular queue
 * application, the SOFF is typically set to the ransfer size to implement post-increment addressing
 * with SMOD function constraining the addresses to a 0-modulo-size range.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param modulo enum type for allowed modulo.
 */
static inline void edma_hal_stcd_configure_source_modulo(
                edma_software_tcd_t *stcd, edma_modulo_t modulo)
{
    assert(stcd);
    stcd->ATTR = (stcd->ATTR & ~DMA_ATTR_SMOD_MASK) | DMA_ATTR_SMOD(modulo);
}

/*!
 * @brief Configure source data transfersize for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param size enum type for transfer size.
 */
static inline void edma_hal_stcd_configure_source_transfersize(
                edma_software_tcd_t *stcd, edma_transfer_size_t size)
{
    assert(stcd);
    stcd->ATTR = (stcd->ATTR & ~DMA_ATTR_SSIZE_MASK) | DMA_ATTR_SSIZE(size);
}

/*!
 * @brief Configure destination modulo for software TCD.
 *
 * The value defines a specific address range specified to be the value after DADDR + DOFF
 * calculation is performed on the original register value. Setting this field provides the ability
 * to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 * queue should start at a 0-modulo-size address and the SMOD field should be set to the appropriate
 * value for the queue, freezing the desired number of upper address bits. The value programmed into
 * this field specifies the number of lower address bits allowed to change. For a circular queue
 * application, the SOFF is typically set to the ransfer size to implement post-increment addressing
 * with DMOD function constraining the addresses to a 0-modulo-size range.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param modulo enum type for allowed modulo.
 */
static inline void edma_hal_stcd_configure_dest_modulo(
                edma_software_tcd_t *stcd, edma_modulo_t modulo) 
{
    assert(stcd);
    stcd->ATTR = (stcd->ATTR & ~DMA_ATTR_DMOD_MASK) | DMA_ATTR_DMOD(modulo);
}

/*!
 * @brief Configure dest data transfersize for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param size enum type for transfer size.
 */
static inline void edma_hal_stcd_configure_dest_transfersize(
                edma_software_tcd_t *stcd, edma_transfer_size_t size)
{
    assert(stcd);
    stcd->ATTR = (stcd->ATTR & ~DMA_ATTR_DSIZE_MASK) | DMA_ATTR_DSIZE(size);
}

/*!
 * @brief Configure nbytes if minor loop mapping is diabled the for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_stcd_configure_nbytes_minorloop_disabled(
                edma_software_tcd_t *stcd, uint32_t nbytes)
{
    assert(stcd);
    stcd->NBYTES_MLNO =
        (stcd->NBYTES_MLNO & ~DMA_NBYTES_MLNO_NBYTES_MASK) | DMA_NBYTES_MLNO_NBYTES(nbytes);
}

/*!
 * @brief Configure nbytes if minor loop mapping is enabled and offset is disabled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_stcd_configure_nbytes_minorloop_enabled_offset_disabled(
                edma_software_tcd_t *stcd, uint32_t nbytes)
{
    assert(stcd);
    stcd->NBYTES_MLOFFNO =
        (stcd->NBYTES_MLOFFNO & ~DMA_NBYTES_MLOFFNO_NBYTES_MASK) | DMA_NBYTES_MLOFFNO_NBYTES(nbytes);
}

/*!
 * @brief Configure nbytes if minor loop mapping is enabled and offset is enabled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param nbytes Number of bytes to be transferred in each service request of the channel.
 */
static inline void edma_hal_stcd_configure_nbytes_minorloop_enabled_offset_enabled(
                edma_software_tcd_t *stcd, uint32_t nbytes)
{
    assert(stcd);
    stcd->NBYTES_MLOFFYES =
        (stcd->NBYTES_MLOFFYES & ~DMA_NBYTES_MLOFFYES_NBYTES_MASK) | DMA_NBYTES_MLOFFYES_NBYTES(nbytes);
}

/*!
 * @brief Configure minorloop offset for software TCD.
 *
 * Configure both the enable bits and offset value. If neither source nor dest offset is enabled,
 * offset would not be configured.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param config Configuration data structure for minorloop offset.
 */
static inline void edma_hal_stcd_configure_minorloop_offset(
        edma_software_tcd_t *stcd, edma_minorloop_offset_config_t *config)
{
    assert(stcd);
    stcd->NBYTES_MLOFFYES =
        (stcd->NBYTES_MLOFFYES & ~(DMA_NBYTES_MLOFFYES_SMLOE_MASK | DMA_NBYTES_MLOFFYES_DMLOE_MASK)) |
        (((uint32_t)config->isEnableSourceMinorloop << DMA_NBYTES_MLOFFYES_SMLOE_SHIFT) |
         ((uint32_t)config->isEnableDestMinorloop << DMA_NBYTES_MLOFFYES_DMLOE_SHIFT));

    if ((config->isEnableSourceMinorloop == true) || (config->isEnableDestMinorloop == true))
    {
        stcd->NBYTES_MLOFFYES =
            (stcd->NBYTES_MLOFFYES & ~DMA_NBYTES_MLOFFYES_MLOFF_MASK) |
            DMA_NBYTES_MLOFFYES_MLOFF(config->offset);
    }
}

/*!
 * @brief Configure last source address adjustment for software TCD.
 *
 * Adjustment value added to the source address at the completion of the major iteration count. This
 * value can be applied to restore the source address to the intial value, or adjust the address to
 * reference the next data structure.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param size adjustment value.
 */
static inline void edma_hal_stcd_configure_source_last_adjustment(
                edma_software_tcd_t *stcd, int32_t size)
{
    assert(stcd);
    stcd->SLAST = (stcd->SLAST & ~DMA_SLAST_SLAST_MASK) | DMA_SLAST_SLAST(size);
}

/*!
 * @brief Configure dest address for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param address memory address pointing to destination data.
 */
static inline void edma_hal_stcd_configure_dest_address(
                edma_software_tcd_t *stcd, uint32_t address)
{
    assert(stcd);
    stcd->DADDR = DMA_DADDR_DADDR(address); 
}

/*!
 * @brief Configure dest address signed offset for software TCD.
 *
 * Sign-extended offset applied to the current source address to form the next-state value as each
 * dest write is complete.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param offset signed-offset. 
 */
static inline void edma_hal_stcd_configure_dest_offset(
                edma_software_tcd_t *stcd, uint32_t offset)
{
    assert(stcd);
    stcd->DOFF = DMA_DOFF_DOFF(offset);
}

/*!
 * @brief Configure last source address adjustment or the memory address for the next transfer
 * control for software TCD.
 *
 * If scatter/gather feature is eanbled(edma_hal_htcd_set_scatter_gather_process()):
 *
 * This address points to the beginning of a 0-modulo-32 byte region containing the next transfer
 * control descriptor to be loaded into this channel. This channel reload is performed as the major
 * iteration count completes. The scatter/gather address must be 0-modulo-32-byte, else a
 * configuration error is reported.
 *
 * else:
 *
 * Adjustment value added to the source address at the completion of the major iteration count. This
 * value can be applied to restore the source address to the intial value, or adjust the address to
 * reference the next data structure.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param size adjustment value.
 */
static inline void edma_hal_stcd_configure_dest_last_adjustment_or_scatter_address(
        edma_software_tcd_t *stcd, uint32_t address)
{
    assert(stcd);
    stcd->DLAST_SGA = DMA_DLAST_SGA_DLASTSGA(address);
}

/*!
 * @brief Configure bandwidth for software TCD.
 *
 * Throttles the amount of bus bandwidth consumed by the eDMA. In general, as the eDMA processes the
 * minor loop, it continuously generates read/write sequences until the minor count is exhausted.
 * This field forces the eDMA to stall after the completion of each read/write access to control the
 * bus request bandwidth seen by the crossbar switch.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param bandwidth enum type for bandwidth control.
 */
static inline void edma_hal_stcd_configure_bandwidth(
        edma_software_tcd_t *stcd, edma_bandwidth_configuration_t bandwidth)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_BWC_MASK) | DMA_CSR_BWC(bandwidth);
}

/*!
 * @brief Configure major link channel number for software TCD.
 *
 * If majorlink is enabled, after the major loop counter is exhausted, the eDMA engine initiates a
 * channel service request at the channel defined by these six bits by setting that channel's start
 * bits.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param majorchannel channel number for major link.
 */
static inline void edma_hal_stcd_configure_majorlink_channel(
        edma_software_tcd_t *stcd, uint32_t majorchannel)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_MAJORLINKCH_MASK) | DMA_CSR_MAJORLINKCH(majorchannel);
}

/*!
 * @brief Enable/Disable major link channel feature for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnabled Enable/Disable.
 */
static inline void edma_hal_stcd_set_majorlink(edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_MAJORELINK_MASK) |
        ((uint32_t)isEnabled << DMA_CSR_MAJORELINK_SHIFT);
}

/*!
 * @brief Enable/Disable scatter/gather feature for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnabled Enable/Disable.
 */
static inline void edma_hal_stcd_set_scatter_gather_process(
        edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_ESG_MASK) | ((uint32_t)isEnabled << DMA_CSR_ESG_SHIFT);
}

/*!
 * @brief Disable/Enable dma request after major loop complete for software TCD.
 *
 * If disabled, the eDMA hardware automatically clears the corresponding dma request when the
 * current major iteration count reaches zero.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isDisabled Disable/Enable.
 */
static inline void edma_hal_stcd_set_disable_dma_request_after_tcd_done(
        edma_software_tcd_t *stcd, bool isDisabled)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_DREQ_MASK) | ((uint32_t)isDisabled << DMA_CSR_DREQ_SHIFT);
}  

/*!
 * @brief Enable/Disable half compelte interrupt for software TCD.
 *
 * If set, the channel generates an interrupt request by setting the appropriate bit in the
 * interrupt register when the current major iteration count reaches the halfway point. Specially,
 * the comparison performed by the eDMA engine is (CITER == (BITER >> 1)). This half way point
 * interrupt request is provided to support double-buffered schemes or other types of data movement
 * where the processor needs an early indication of the transfer's process.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_stcd_set_half_complete_interrupt(
        edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_INTHALF_MASK) | ((uint32_t)isEnabled << DMA_CSR_INTHALF_SHIFT);
}

/*!
 * @brief Enable/Disable interrupt after major loop complete for software TCD.
 *
 * If enabled, the channel generates an interrupt request by setting the appropriate bit in the 
 * interrupt register when the current major iteration count reaches zero.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_stcd_set_complete_interrupt(
        edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->CSR = (stcd->CSR & ~DMA_CSR_INTMAJOR_MASK) | ((uint32_t)isEnabled << DMA_CSR_INTMAJOR_SHIFT);
}

/*!
 * @brief Set trigger start bits for software TCD.
 *
 */
static inline void edma_hal_stcd_trigger_channel_start(
        edma_software_tcd_t *stcd)
{
    assert(stcd);
    stcd->CSR |= DMA_CSR_START_MASK;
}

/*!
 * @brief Enable/Disable channel link after minor loop for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_stcd_set_minor_link(
        edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->BITER_ELINKYES = (stcd->BITER_ELINKYES & ~DMA_BITER_ELINKYES_ELINK_MASK) |
                            ((uint32_t)isEnabled << DMA_BITER_ELINKYES_ELINK_SHIFT);
}

/*!
 * @brief Enable/Disable current channel link after minor loop for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param isEnable Enable/Disable.
 */
static inline void edma_hal_stcd_set_current_minor_link(
        edma_software_tcd_t *stcd, bool isEnabled)
{
    assert(stcd);
    stcd->CITER_ELINKYES = (stcd->CITER_ELINKYES & ~DMA_CITER_ELINKYES_ELINK_MASK) |
                            ((uint32_t)isEnabled << DMA_CITER_ELINKYES_ELINK_SHIFT);
}

/*!
 * @brief Configure minor loop link channel for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param minorchannel minor loop link channel.
 */
static inline void edma_hal_stcd_configure_minor_link_channel(
        edma_software_tcd_t *stcd, uint32_t minorchannel)
{
    assert(stcd);
    stcd->BITER_ELINKYES = (stcd->BITER_ELINKYES & ~DMA_BITER_ELINKYES_LINKCH_MASK) |
                            DMA_BITER_ELINKYES_LINKCH(minorchannel);
}

/*!
 * @brief Configure minor loop link channel for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param minorchannel minor loop link channel.
 */
static inline void edma_hal_stcd_configure_current_minor_link_channel(
        edma_software_tcd_t *stcd, uint32_t minorchannel)
{
    assert(stcd);
    stcd->CITER_ELINKYES = (stcd->CITER_ELINKYES & ~DMA_CITER_ELINKYES_LINKCH_MASK) |
                            DMA_CITER_ELINKYES_LINKCH(minorchannel);
}

/*!
 * @brief Configure major count if minor loop channel link is disbled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param count major loop count
 */
static inline void edma_hal_stcd_configure_majorcount_minorlink_disabled(
        edma_software_tcd_t *stcd, uint32_t count)
{
    assert(stcd);
    stcd->BITER_ELINKNO = (stcd->BITER_ELINKNO & ~DMA_BITER_ELINKNO_BITER_MASK) |
                            DMA_BITER_ELINKNO_BITER(count);
}

/*!
 * @brief Configure current major count if minor loop channel link is disbled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param count major loop count
 */
static inline void edma_hal_stcd_configure_current_majorcount_minorlink_disabled(
        edma_software_tcd_t *stcd, uint32_t count)
{
    assert(stcd);
    stcd->CITER_ELINKNO = (stcd->CITER_ELINKNO & ~DMA_CITER_ELINKNO_CITER_MASK) |
                            DMA_CITER_ELINKNO_CITER(count);
}

/*!
 * @brief Configure major count if minor loop channel link is enabled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param count major loop count
 */
static inline void edma_hal_stcd_configure_majorcount_minorlink_enabled(
        edma_software_tcd_t *stcd, uint32_t count)
{
    assert(stcd);
    stcd->BITER_ELINKYES = (stcd->BITER_ELINKYES & ~DMA_BITER_ELINKYES_BITER_MASK) |
                            DMA_BITER_ELINKYES_BITER(count);
}

/*!
 * @brief Configure currernt major count if minor loop channel link is enabled for software TCD.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param count major loop count
 */
static inline void edma_hal_stcd_configure_current_majorcount_minorlink_enabled(
        edma_software_tcd_t *stcd, uint32_t count)
{
    assert(stcd);
    stcd->CITER_ELINKYES = (stcd->CITER_ELINKYES & ~DMA_CITER_ELINKYES_CITER_MASK) |
                            DMA_CITER_ELINKYES_CITER(count);
}

/*!
 * @brief Copy software tcd configuration to hardware tcd.
 *
 * @param stcd memory pointing to edma software tcd.
 * @param instance Edma module.
 * @param channel Edma channel.
 * @param stcd memory pointing to software tcd.
 */
void edma_hal_stcd_push_to_htcd(uint32_t instance, uint32_t channel, edma_software_tcd_t *stcd);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __EDMA_HAL_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/

