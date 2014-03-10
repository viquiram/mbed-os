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

#include <stdlib.h>
#include <string.h>
#include "fsl_edma_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"

/*******************************************************************************
 * Variabled
 ******************************************************************************/

/*! @brief Interrupt data structure. */
extern const IRQn_Type edma_irq_ids[HW_DMA_INSTANCE_COUNT][FSL_FEATURE_DMA_MODULE_CHANNEL];
extern const IRQn_Type edma_error_irq_ids[HW_DMA_INSTANCE_COUNT];

/*! @brief Data structure for the eDMA device */
typedef struct EdmaDevice {
    edma_channel_t * volatile edmaChan[FSL_FEATURE_DMA_DMAMUX_CHANNELS];  /*!< Data pointer array for the
                                                                     eDMA channel */
    sync_object_t sema;                                         /*!< A semaphore for the eDMA driver. */
} edma_device_t;

/*! @brief EDMA global structure to maintain EDMA resource */
edma_device_t g_edma;

/*******************************************************************************
 * PROTOTYPES 
 ******************************************************************************/
void edma_update_descriptor_internal(edma_channel_t *chn);
edma_status_t edma_claim_channel(uint32_t channel,dma_request_source_t source, edma_channel_t *chn);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : edma_init
 * Description   : Initialize EDMA.
 *
 *END**************************************************************************/
edma_status_t edma_init(void)
{
    uint32_t i, j;
    IRQn_Type irqNumber;
    edma_config_t configuration;

    memset(&g_edma, 0, sizeof(edma_device_t));

    /* Init synchronizaiton object for the access control of edma data structure. */
    sync_create(&g_edma.sema, 1);

    /* Configure the edma module. */
    configuration.isEnableMinorLoopping = false;
    configuration.isEnableContinuousMode = false;
    configuration.isHaltOnError = true;
    configuration.isEnableRoundrobinArbitration = true;
    configuration.isEnableDebug = false;
#if (FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
    configuration.groupPriority = kEdmaGroup0Priority0Group1Priority1;
    configuration.isEnableGroupRoundrobinArbitration = true;
#endif

    for (i = 0; i < HW_DMA_INSTANCE_COUNT; i++)
    {
        /* Enable clock gate of eDMA module. */
        if (clock_manager_set_gate(kClockModuleDMA, i, true) != kClockManagerSuccess)
        {
            return kStatus_EDMA_Fail;
        }

        /* Init eDMA module in hardware level. */
        edma_hal_init(i, &configuration);

        /* Enable the error interrupt for eDMA module. */
        irqNumber = edma_error_irq_ids[i];
        interrupt_enable(irqNumber);

        /* Register all edma channl interrupt handler into vector table. */
        for (j = 0; j < FSL_FEATURE_DMA_MODULE_CHANNEL; j++)
        {
            irqNumber = edma_irq_ids[i][j];
            interrupt_enable(irqNumber);
        }
    }

    for (i = 0; i < HW_DMAMUX_INSTANCE_COUNT; i++)
    {
        /* Enable dmamux clock gate */
        if (clock_manager_set_gate(kClockModuleDMAMUX, i, true) != kClockManagerSuccess)
        {
            return kStatus_EDMA_Fail;
        }

        /* Init dmamux module in hardware level */
        dmamux_hal_init(i);
    }

    return kStatus_EDMA_Success;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_shutdown
 * Description   : Deinitilize EDMA.
 *
 *END**************************************************************************/
edma_status_t edma_shutdown(void)
{
    uint32_t i, j;

    /* Release all edma channel. */
    for (i = 0; i < HW_DMA_INSTANCE_COUNT; i++)
    {
        for (j = i * FSL_FEATURE_DMA_MODULE_CHANNEL; j < (i + 1) * FSL_FEATURE_DMA_MODULE_CHANNEL; j++)
        {
            if (g_edma.edmaChan[j])
            {
                edma_free_channel(g_edma.edmaChan[j]);
            }
        }

        /* Disable edma clock gate. */
        clock_manager_set_gate(kClockModuleDMA, i, false);
    }

    /* Disable dmamux clock gate. */
    for (i = 0; i < HW_DMAMUX_INSTANCE_COUNT; i++)
    {
        clock_manager_set_gate(kClockModuleDMAMUX, i, false);
    }

    sync_destroy(&g_edma.sema);

    return kStatus_EDMA_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_register_callback
 * Description   : Register callback function and parameter.
 *
 *END**************************************************************************/
void edma_register_callback(edma_channel_t *chn, edma_callback_t callback, void *para)
{
    chn->callback = callback;
    chn->parameter = para;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_get_descriptor_statu
 * Description   : Get the status of edma channel descritpor chain.
 *
 *END**************************************************************************/
edma_status_t edma_get_descriptor_status(edma_channel_t *chn, uint32_t *descriptorStatus)
{
    uint8_t i;

    /* All descritpor is set to kEdmaDescriptorDone(0).*/
    memset(descriptorStatus, 0, chn->tcdNumber * sizeof(uint32_t));

    if (chn->tcdNumber == chn->tcdRead)
    {
        return kStatus_EDMA_Success;
    }
    else if (chn->tcdUnderflow)
    {
        descriptorStatus[chn->tcdRead] = chn->tcdLeftBytes;
        return kStatus_EDMA_Success;
    }
    else
    {
        i = chn->tcdRead;
        descriptorStatus[i] = chn->tcdLeftBytes;

        i = (i + 1) % chn->tcdNumber;
        for (; i != chn->tcdWrite;i = (i + 1) % chn->tcdNumber) 
        {
            descriptorStatus[i] = kEdmaDescriptorPrepared;
        }
        return kStatus_EDMA_Success;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_request_channel
 * Description   : Request a edma channel.
 *
 *END**************************************************************************/
uint32_t edma_request_channel(uint32_t channel, dma_request_source_t source, edma_channel_t *chan)
{

    /*Check if dynamically allocation is requested */
    if (channel == kEdmaAnyChannel)
    {
        uint32_t i = 0, j;
        uint32_t map;
        map = ((uint32_t)source >> 8);
       
        while (map != 0)
        {
            if (map & (0x1U << i))
            {
                for (j = i * FSL_FEATURE_DMAMUX_MODULE_CHANNEL; j < (i + 1) * FSL_FEATURE_DMAMUX_MODULE_CHANNEL; j++) 
                {
                    if (!g_edma.edmaChan[j])
                    {
                        edma_claim_channel(j, source, chan);
                        return j;
                    }
                }
            
            }
            map &= ~(0x1U << i);
            i++;
        }
        
        /* No available channel. */
        return kEdmaInvalidChannel;
    }

    /* static allocation */
    if (!g_edma.edmaChan[channel])
    {
        edma_claim_channel(channel, source, chan);
        return channel;
    }

    return kEdmaInvalidChannel;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_claim_channel
 * Description   : Claim an edma channel.
 *
 *END**************************************************************************/
edma_status_t edma_claim_channel(uint32_t channel,dma_request_source_t source, edma_channel_t *chn)
{
    fsl_rtos_status syncStatus;
    uint8_t src = (uint32_t)source & 0xFF;

    memset(chn, 0, sizeof(edma_channel_t));

    do
    {
        syncStatus = sync_wait(&g_edma.sema, kSyncWaitForever);
    }while(syncStatus == kIdle);
    
    g_edma.edmaChan[channel] = chn;
    sync_signal(&g_edma.sema);

    /* Init the edma channel context to reset status. */
    chn->channel = channel % FSL_FEATURE_DMA_MODULE_CHANNEL;
    chn->dmamuxChannel = channel % FSL_FEATURE_DMAMUX_MODULE_CHANNEL;
    chn->dmamuxModule = channel / FSL_FEATURE_DMAMUX_MODULE_CHANNEL;
    chn->module = channel / FSL_FEATURE_DMA_MODULE_CHANNEL;

    chn->status = kEdmaNormal;
    chn->tcdLeftBytes = kEdmaDescriptorPrepared;

    /* Enable error interrupt for this channel */
    edma_hal_enable_error_interrupt(chn->module, chn->channel);

    /* Configure the DMAMUX for edma channel */
    dmamux_hal_disable_channel(chn->dmamuxModule, chn->dmamuxChannel);
    dmamux_hal_set_trigger_source(
           chn->dmamuxModule, chn->dmamuxChannel, src%(uint8_t)kDmamuxDmaRequestSource);
    dmamux_hal_enable_channel(chn->dmamuxModule, chn->dmamuxChannel);

    return kStatus_EDMA_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_free_channel
 * Description   : Free eDMA channel's hardware and software resource.
 *
 *END**************************************************************************/
edma_status_t edma_free_channel(edma_channel_t *chn)
{
    fsl_rtos_status syncStatus;
    uint32_t channel;

    channel = chn->module * FSL_FEATURE_DMA_MODULE_CHANNEL + chn->channel;

    /* Stop edma channel. */
    edma_stop_channel(chn);

    if (g_edma.edmaChan[channel])
    {
        do
        {
            syncStatus = sync_wait(&g_edma.sema, kSyncWaitForever);
        }while(syncStatus == kIdle);

        g_edma.edmaChan[channel] = NULL;

        sync_signal(&g_edma.sema);
    }

    memset(chn, 0x1, sizeof(edma_channel_t)); 

    return kStatus_EDMA_Success;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_start_channel
 * Description   : Start an EDMA channel.
 *
 *END**************************************************************************/
edma_status_t edma_start_channel(edma_channel_t *chn)
{
    chn->tcdLeftBytes = kEdmaDescriptorPrepared;
    edma_hal_enable_dma_request(chn->module, chn->channel);
    return kStatus_EDMA_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_stop_channel
 * Description   : Stop a eDMA channel.
 *
 *END**************************************************************************/
edma_status_t edma_stop_channel(edma_channel_t *chn)
{
    edma_hal_disable_dma_request(chn->module, chn->channel);

    /* Update the tcd Status according current status.*/
    edma_update_descriptor_internal(chn);

    /* Get the unfinished bytes for current tcd.*/
    chn->tcdLeftBytes = edma_hal_htcd_get_unfinished_bytes(chn->module, chn->channel);

    return kStatus_EDMA_Success;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : edma_update_descriptor 
 * Description   : Update edma descriptor status.
 *
 *END**************************************************************************/
edma_status_t edma_update_descriptor(edma_channel_t *chn)
{
    chn->tcdUnderflow = false;
    chn->tcdWrite = (chn->tcdWrite + 1) % chn->tcdNumber;
    return kStatus_EDMA_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_update_descriptor_internal
 * Description   : Update the status of TCD for internal TCD management.
 *
 *END**************************************************************************/
void edma_update_descriptor_internal(edma_channel_t *chn)
{
    uint32_t current_index;

    do {
        current_index = edma_hal_htcd_get_majorlink_channel(chn->module, chn->channel);

        chn->tcdRead = current_index;
        
        /* Check if the channel is disabled by checking whether the dma request is disabled.
           In this DMA driver, only the end of chain would disable the dma request. */
        if (!edma_hal_check_dma_request_enable_status(chn->module, chn->channel))
        {
            /* Set DMA read pointer to number to tell that all DMA chain is finished.
             * This status is specailly for the scatter list status.*/
            chn->tcdRead = chn->tcdNumber;
            return;
        }

        /* Do with the underrun case.*/
        uint8_t read, write;
        write = chn->tcdWrite;
        read = chn->tcdRead;

        if (read == write)
        {
            chn->tcdUnderflow = true;
            chn->tcdWrite = (chn->tcdWrite + 1) % chn->tcdNumber;
        }

    } while(edma_hal_htcd_get_majorlink_channel(chn->module, chn->channel) != current_index);
}

#if (FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_IRQ_HANDLER
 * Description   : EDMA IRQ handler.
 *
 *END**************************************************************************/
void EDMA_IRQ_HANDLER(uint32_t channel)
{
    edma_channel_t *chn = g_edma.edmaChan[channel];

    if (!chn)
    {
        return;
    }

    edma_update_descriptor_internal(chn);
    edma_hal_clear_done_status(chn->module, chn->channel);
    edma_hal_clear_interrupt_request(chn->module, chn->channel);

    if (chn->callback)
    {
        chn->callback(chn->parameter, chn->status);
    }
}
#else
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_IRQ_HANDLER
 * Description   : EDMA IRQ handler.This handler is for EDMA module in which
 * 				   channel n share the irq number with channel (n + 16)
 *
 *END**************************************************************************/
void EDMA_IRQ_HANDLER(uint32_t channel)
{
    edma_channel_t *chn;
    uint32_t interrupt;
    uint32_t channelInt = channel;

    interrupt =
        edma_hal_get_all_channel_interrupt_request_status(channel /FSL_FEATURE_DMA_MODULE_CHANNEL) &
                        ((1U << (channelInt % FSL_FEATURE_DMA_MODULE_CHANNEL)) |
                        (1U << ((channelInt + 16) % FSL_FEATURE_DMA_MODULE_CHANNEL))) ;

    while (interrupt)
    {
        chn = g_edma.edmaChan[channelInt];
		interrupt &= ~(0x1U << channelInt);
        channelInt += 16U;
        if (!chn)
        {
            continue;
        }

        edma_update_descriptor_internal(chn);
        edma_hal_clear_done_status(chn->module, chn->channel);
        edma_hal_clear_interrupt_request(chn->module, chn->channel);

        if (chn->callback)
        {
            chn->callback(chn->parameter, chn->status);
        }
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : DMA_ERR_IRQHandler
 * Description   : EDMA error handler
 *
 *END**************************************************************************/
void DMA_ERR_IRQHandler(uint32_t instance)
{
    uint32_t j;
    edma_channel_t *chn;
    uint32_t error;

    error = edma_hal_get_all_channel_error_status(instance);
    if (error)
    {
        for (j = 0; j < FSL_FEATURE_DMA_MODULE_CHANNEL; j++)
        {
            if ((error >> j) & 0x1U)
            {
                edma_hal_disable_dma_request(instance, j);
                chn = g_edma.edmaChan[instance * FSL_FEATURE_DMA_MODULE_CHANNEL + j];
                if (chn)
                {
                    /* Disable error channel interrupt. */
                    edma_hal_clear_done_status(chn->module, chn->channel);
                    edma_hal_clear_interrupt_request(chn->module, chn->channel);
                    chn->status = kEdmaError;
                    if (chn->callback)
                    {
                        chn->callback(chn->parameter, chn->status);
                    }
                    edma_hal_clear_error_status(chn->module, chn->channel);
                    chn->status = kEdmaNormal;
                }
            }
        }
        edma_hal_clear_halt(instance);
    }
}
/*FUNCTION**********************************************************************
 *
 * Function Name : edma_config_loop
 * Description   : User friendly interface to configure loop descritptor chain.
 *
 *END**************************************************************************/
edma_status_t edma_config_loop(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t srcAddr, uint32_t destAddr, uint32_t size,
                            uint32_t watermark, uint32_t length, uint8_t period)
{
    uint8_t i;
    edma_transfer_size_t transfersize;

    chn->tcdNumber = period;
    memset(stcd, 0, period * sizeof(edma_software_tcd_t));

    switch(size)
    {
        case 1:
            transfersize = kEdmaTransferSize1bytes;
            break;
        case 2:
            transfersize = kEdmaTransferSize2bytes;
            break;
        case 4:
            transfersize = kEdmaTransferSize4bytes;
            break;
        case 16:
            transfersize = kEdmaTransferSize16bytes;
            break;
        case 32:
            transfersize = kEdmaTransferSize32bytes;
            break;
        default:
            return kStatus_EDMA_InvalidArgument;
    }

    /* Configure the software TCD one by one.*/
    for(i = 0; i < period; i++)
    {
        edma_hal_stcd_configure_nbytes_minorloop_disabled(&stcd[i],watermark);
        edma_hal_stcd_configure_dest_last_adjustment_or_scatter_address(

                                    &stcd[i], (uint32_t)&stcd[(i + 1)%chn->tcdNumber]);
        edma_hal_stcd_configure_majorlink_channel(&stcd[i], i);
        edma_hal_stcd_set_scatter_gather_process(&stcd[i], true);
        edma_hal_stcd_set_complete_interrupt(&stcd[i], true);
        edma_hal_stcd_configure_majorcount_minorlink_disabled(
                        &stcd[i], length/(period * watermark));
        edma_hal_stcd_configure_current_majorcount_minorlink_disabled(
                        &stcd[i], length/(period * watermark));
        switch (type)
        {
            case kEdmaPeripheralToMemory:
                /* Configure Source Read. */
                edma_hal_stcd_configure_source_address(&stcd[i], srcAddr);
                edma_hal_stcd_configure_source_offset(&stcd[i], 0);
                edma_hal_stcd_configure_source_transfersize(&stcd[i], transfersize);

                /* Configure Dest Write. */
                edma_hal_stcd_configure_dest_address(
                        &stcd[i], destAddr + i * (length/period));
                edma_hal_stcd_configure_dest_offset(&stcd[i], size); 
                edma_hal_stcd_configure_dest_transfersize(&stcd[i], transfersize);
                
                break;
            case kEdmaMemoryToPeripheral:
                /* Configure Source Read. */
                edma_hal_stcd_configure_source_address(&stcd[i], srcAddr + i * (length/period));
                edma_hal_stcd_configure_source_offset(&stcd[i], size);
                edma_hal_stcd_configure_source_transfersize(&stcd[i], transfersize);

                /* Configure Dest Write. */
                edma_hal_stcd_configure_dest_address(&stcd[i], destAddr);
                edma_hal_stcd_configure_dest_offset(&stcd[i], 0); 
                edma_hal_stcd_configure_dest_transfersize(&stcd[i], transfersize);
                
                break;
            case kEdmaMemoryToMemory:
                /* Configure Source Read. */
                edma_hal_stcd_configure_source_address(&stcd[i], srcAddr + i * (length/period));
                edma_hal_stcd_configure_source_offset(&stcd[i], size);
                edma_hal_stcd_configure_source_transfersize(&stcd[i], transfersize);

                /* Configure Dest Write. */
                edma_hal_stcd_configure_dest_address(
                        &stcd[i], destAddr + i * (length/period));
                edma_hal_stcd_configure_dest_offset(&stcd[i], size); 
                edma_hal_stcd_configure_dest_transfersize(&stcd[i], transfersize);
                
                break;
            default:
                return kStatus_EDMA_InvalidArgument;
        }
    }

    edma_hal_stcd_push_to_htcd(chn->module, chn->channel, &stcd[0]);

    return kStatus_EDMA_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : edma_config_scatter_gather
 * Description   : User friendly interface to configure single end descritptor chain.
 *
 *END**************************************************************************/
edma_status_t edma_config_scatter_gather(
                            edma_software_tcd_t *stcd, edma_channel_t *chn, edma_transfer_type_t type,
                            uint32_t size, uint32_t watermark,
                            edma_scatter_list_t *srcScatterList, edma_scatter_list_t *destScatterList,
                            uint8_t number)
{
    uint8_t i;
    edma_transfer_size_t transfersize;

    chn->tcdNumber = number;

    memset(stcd, 0, number * sizeof(edma_software_tcd_t));

    switch(size)
    {
        case 1:
            transfersize = kEdmaTransferSize1bytes;
            break;
        case 2:
            transfersize = kEdmaTransferSize2bytes;
            break;
        case 4:
            transfersize = kEdmaTransferSize4bytes;
            break;
        case 16:
            transfersize = kEdmaTransferSize16bytes;
            break;
        case 32:
            transfersize = kEdmaTransferSize32bytes;
            break;
        default:
            return kStatus_EDMA_InvalidArgument;
    }

    if (number == 1)
    {
        edma_software_tcd_t initTcd;

        memset(&initTcd, 0, sizeof(edma_software_tcd_t));

        /* Clear Hardware TCD to init state. */
        edma_hal_stcd_push_to_htcd(chn->module, chn->channel, &initTcd);

        /*The software tcd would not be introduced into this case. */
        edma_hal_htcd_configure_nbytes_minorloop_disabled(chn->module, chn->channel,watermark);
        edma_hal_htcd_configure_majorcount_minorlink_disabled(
                        chn->module, chn->channel, srcScatterList[0].length/watermark);
        edma_hal_htcd_configure_current_majorcount_minorlink_disabled(
                        chn->module, chn->channel, srcScatterList[0].length/watermark);
        edma_hal_htcd_configure_source_transfersize(chn->module, chn->channel, transfersize);
        edma_hal_htcd_configure_source_address(chn->module, chn->channel, srcScatterList[0].address);
        edma_hal_htcd_configure_dest_transfersize(chn->module, chn->channel, transfersize);
        edma_hal_htcd_configure_dest_address( chn->module, chn->channel, destScatterList[0].address);
        edma_hal_htcd_set_disable_dma_request_after_tcd_done( chn->module, chn->channel, true);
        edma_hal_htcd_set_complete_interrupt(chn->module, chn->channel, true);
        edma_hal_htcd_set_scatter_gather_process(chn->module, chn->channel, false);
        switch (type)
        {
            case kEdmaPeripheralToMemory:
                /* Configure Source Read. */
                edma_hal_htcd_configure_source_offset(chn->module, chn->channel, 0);

                /* Configure Dest Write. */
                edma_hal_htcd_configure_dest_offset(chn->module, chn->channel, size); 
                break;
            case kEdmaMemoryToPeripheral:
                /* Configure Source Read. */
                edma_hal_htcd_configure_source_offset(chn->module, chn->channel, size);

                /* Configure Dest Write. */
                edma_hal_htcd_configure_dest_offset(chn->module, chn->channel, 0); 
                break;
            case kEdmaMemoryToMemory:
                /* Configure Source Read. */
                edma_hal_htcd_configure_source_offset(chn->module, chn->channel, size);

                /* Configure Dest Write. */
                edma_hal_htcd_configure_dest_offset(chn->module, chn->channel, size); 
                break;
            default:
                return kStatus_EDMA_InvalidArgument;
        }

        return kStatus_EDMA_Success;
    }
    else
    {
        for (i = 0; i < number; i++)
        {
            edma_hal_stcd_configure_nbytes_minorloop_disabled(&stcd[i],watermark);
            edma_hal_stcd_configure_majorcount_minorlink_disabled(
                            &stcd[i], srcScatterList[i].length/watermark);
            edma_hal_stcd_configure_current_majorcount_minorlink_disabled(
                            &stcd[i], srcScatterList[i].length/watermark);
            edma_hal_stcd_configure_source_transfersize(&stcd[i], transfersize);
            edma_hal_stcd_configure_source_address(
                                        &stcd[i], srcScatterList[i].address);
            edma_hal_stcd_configure_dest_transfersize(&stcd[i], transfersize);
            edma_hal_stcd_configure_dest_address(
                                        &stcd[i], destScatterList[i].address);

            if (i == (number - 1))
            {
                /*If it is the last descriptor, disable the request and set the interrupt. */
                edma_hal_stcd_set_disable_dma_request_after_tcd_done(
                        &stcd[i], true);
                edma_hal_stcd_set_complete_interrupt(&stcd[i], true);
                edma_hal_stcd_set_scatter_gather_process(&stcd[i], false);
            }
            else
            {
                edma_hal_stcd_set_scatter_gather_process(&stcd[i], true);
                edma_hal_stcd_configure_dest_last_adjustment_or_scatter_address(
                                            &stcd[i], (uint32_t)&stcd[i + 1]);
            }
            
            switch (type)
            {
                case kEdmaPeripheralToMemory:
                    /* Configure Source Read. */
                    edma_hal_stcd_configure_source_offset(&stcd[i], 0);

                    /* Configure Dest Write. */
                    edma_hal_stcd_configure_dest_offset(&stcd[i], size); 
                    break;
                case kEdmaMemoryToPeripheral:
                    /* Configure Source Read. */
                    edma_hal_stcd_configure_source_offset(&stcd[i], size);

                    /* Configure Dest Write. */
                    edma_hal_stcd_configure_dest_offset(&stcd[i], 0); 
                    break;
                case kEdmaMemoryToMemory:
                    /* Configure Source Read. */
                    edma_hal_stcd_configure_source_offset(&stcd[i], size);

                    /* Configure Dest Write. */
                    edma_hal_stcd_configure_dest_offset(&stcd[i], size); 
                    break;
                default:
                    return kStatus_EDMA_InvalidArgument;
            }
        }
        edma_hal_stcd_push_to_htcd(chn->module, chn->channel, &stcd[0]);

        return kStatus_EDMA_Success;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

