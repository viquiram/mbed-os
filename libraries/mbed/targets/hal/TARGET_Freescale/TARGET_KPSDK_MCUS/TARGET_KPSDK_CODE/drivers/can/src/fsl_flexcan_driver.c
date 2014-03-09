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

#include "fsl_flexcan_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

bool int_mb;
bool int_fifo;
uint32_t rx_mb_idx;
sync_object_t irqSync;

/*! The following table is based on the assumption that BSP_CANPE_CLOCK is defined. The CAN bit*/
/*! timing parameters are calculated by using the method outlined in AN1798, section 4.1.*/
/*! A maximum time for PROP_SEG will be used, the remaining TQ will be split equally between PSEG1*/
/*! and PSEG2, if PSEG2 >=2.*/
/*! RJW is set to the minimum of 4 or PSEG1.*/
/*! The table contains bit_rate (Hz), propseg, pseg1, pseg2, pre_divider, and rjw.*/
#if defined(K70F12_SERIES) || defined(K64F12_SERIES)
const flexcan_bitrate_table_t bit_rate_table[] = {
    { kFlexCanBitrate_125k, 6, 7, 7, 19, 3},  /* 125 kHz */
    { kFlexCanBitrate_250k, 6, 7, 7,  9, 3},  /* 250 kHz */
    { kFlexCanBitrate_500k, 6, 7, 7,  4, 3},  /* 500 kHz */
    { kFlexCanBitrate_750k, 6, 5, 5,  3, 3},  /* 750 kHz */
    { kFlexCanBitrate_1M,   6, 5, 5,  2, 3},  /* 1   MHz */
};
#endif

extern IRQn_Type flexcan_irq_ids[HW_CAN_INSTANCE_COUNT][FSL_CAN_INTERRUPT_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_set_bitrate
 * Description   : Set FlexCAN baudrate.
 * This function will set up all the time segment values. Those time segment
 * values are from the table bit_rate_table and based on the baudrate passed in.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_set_bitrate(uint8_t instance, flexcan_bitrate_t bitrate)
{
    uint32_t num_bitrate_table;
    uint32_t i;
    flexcan_time_segment_t time_seg;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Find the time segments from the table bit_rate_table*/
    num_bitrate_table = sizeof(bit_rate_table)/sizeof(bit_rate_table[0]);
    for (i = 0; i < num_bitrate_table; i++)
    {
        if (bit_rate_table[i].bit_rate == bitrate)
        {
            time_seg.propseg = bit_rate_table[i].propseg;
            time_seg.pseg1 = bit_rate_table[i].pseg1;
            time_seg.pseg2 = bit_rate_table[i].pseg2;
            time_seg.pre_divider = bit_rate_table[i].pre_divider;
            time_seg.rjw = bit_rate_table[i].rjw;
            break;
        }
    }

    if (i == num_bitrate_table)
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }

    /* Set time segments*/
    flexcan_hal_set_time_segments(instance, &time_seg);

    return kStatus_FLEXCAN_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_get_bitrate
 * Description   : Get FlexCAN baudrate.
 * This function will be based on all the time segment values and find out the
 * baudrate from the table bit_rate_table.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_get_bitrate(uint8_t instance, flexcan_bitrate_t *bitrate)
{
    uint32_t i;
    flexcan_time_segment_t time_seg;
    uint32_t num_bitrate_table;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Get the time segments*/
    flexcan_hal_get_time_segments(instance, &time_seg);

    /* Find out the corresponding bit rate in the table bit_rate_table*/
    num_bitrate_table = sizeof(bit_rate_table)/sizeof(bit_rate_table[0]);
    for (i = 0; i < num_bitrate_table; i++)
    {
        if (bit_rate_table[i].pre_divider == time_seg.pre_divider)
        {
            if (bit_rate_table[i].pseg2 == time_seg.pseg2)
            {
                *bitrate = bit_rate_table[i].bit_rate;
                return kStatus_FLEXCAN_Success;
            }
        }
    }

    return kStatus_FLEXCAN_InvalidArgument;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_set_mask_type
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 *END**************************************************************************/
void flexcan_set_mask_type(uint8_t instance, flexcan_rx_mask_type_t type)
{
    flexcan_hal_set_mask_type(instance, type);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_set_rx_fifo_global_mask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_set_rx_fifo_global_mask(
    uint8_t instance,
    flexcan_mb_id_type_t id_type,
    uint32_t mask)
{
    assert(instance < HW_CAN_INSTANCE_COUNT);

    if (id_type == kFlexCanMbId_Std)
    {
        /* Set standard global mask for RX FIOF*/
        flexcan_hal_set_rx_fifo_global_std_mask(instance, mask);
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended global mask for RX FIFO*/
        flexcan_hal_set_rx_fifo_global_ext_mask(instance, mask);
    }
    else
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }

    return kStatus_FLEXCAN_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_set_rx_mb_global_mask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask
 * or the 29-bit extended mask.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_set_rx_mb_global_mask(
    uint8_t instance,
    flexcan_mb_id_type_t id_type,
    uint32_t mask)
{
    assert(instance < HW_CAN_INSTANCE_COUNT);

    if (id_type == kFlexCanMbId_Std)
    {
        /* Set standard global mask for RX MB*/
        flexcan_hal_set_rx_mb_global_std_mask(instance, mask);
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended global mask for RX MB*/
        flexcan_hal_set_rx_mb_global_ext_mask(instance, mask);
    }
    else
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }

    return kStatus_FLEXCAN_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_set_rx_individual_mask
 * Description   : Set Rx individual mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_set_rx_individual_mask(
    uint8_t instance,
    const flexcan_user_config_t * data,
    flexcan_mb_id_type_t id_type,
    uint32_t mb_idx,
    uint32_t mask)
{
    assert(instance < HW_CAN_INSTANCE_COUNT);

    if (id_type == kFlexCanMbId_Std)
    {
        /* Set standard individual mask*/
        return flexcan_hal_set_rx_individual_std_mask(instance, data, mb_idx, mask);
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended individual mask*/
        return flexcan_hal_set_rx_individual_ext_mask(instance, data, mb_idx, mask);
    }
    else
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_init
 * Description   : Initialize FlexCAN driver.
 * This function will select a source clock, reset FlexCAN module, set maximum
 * number of message buffers, initialize all message buffers as inactive, enable
 * RX FIFO if needed, mask all mask bits, disable all MB interrupts, enable
 * FlexCAN normal mode, and enable all the error interrupts if needed.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_init(
   uint8_t instance,
   const flexcan_user_config_t *data,
   bool enable_err_interrupts)
{
    flexcan_status_t result;
    uint32_t i;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Enable clock gate to FlexCAN module */
    clock_manager_set_gate(kClockModuleFLEXCAN, instance, true);

    /* Select a source clock for FlexCAN*/
    result = flexcan_hal_select_clk(instance, kFlexCanClkSource_Ipbus);
    if (result)
    {
        return result;
    }

    /* Initialize FLEXCAN device */
    result = flexcan_hal_init(instance, data);
    if (result)
    {
        return result;
    }

    /* Select mode */
    result = flexcan_hal_enable_operation_mode(instance, kFlexCanNormalMode);
    if (result)
    {
        return result;
    }

    /* Enable error interrupts */
    if(enable_err_interrupts)
    {
        flexcan_hal_enable_error_interrupt(instance);

        /* Init the interrupt sync object.*/
        sync_create(&irqSync, 0);

        /* configure and enable the FlexCAN instance IRQ */
        for (i = 0; i < 4; i++)
        {
            /* Enable FlexCAN interrupt.*/
            interrupt_enable(flexcan_irq_ids[instance][i]);
        }
    }

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_tx_mb_config
 * Description   : Configure a Tx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Tx buffer as INACTIVE, and enable the
 * Message Buffer interrupt.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_tx_mb_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *tx_info,
    uint32_t msg_id)
{
    flexcan_status_t result;
    flexcan_mb_code_status_tx_t cs;
    assert(instance < HW_CAN_INSTANCE_COUNT);
    assert(data);

    /* Initialize transmit mb*/
    cs.data_length = tx_info->data_length;
    cs.msg_id_type = tx_info->msg_id_type;
    cs.code = kFlexCanTX_Inactive;
    result = flexcan_hal_set_mb_tx(instance, data, mb_idx, &cs, msg_id, NULL);
    if (result)
    {
        return result;
    }

    /* Enable message buffer interrupt*/
    return flexcan_hal_enable_mb_interrupt(instance, data, mb_idx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_send
 * Description   : Set up FlexCAN Message buffer for transmitting data.
 * This function will set the MB CODE field as DATA for Tx buffer. Then this
 * function will copy user's buffer into the message buffer data area, and wait
 * for the Message Buffer interrupt.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_send(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    uint32_t num_bytes,
    uint8_t *mb_data)
{
    flexcan_status_t result;
    uint32_t num_byte_left;
    uint32_t total_byte_data_transmitted;
    flexcan_mb_code_status_tx_t cs;
    assert(instance < HW_CAN_INSTANCE_COUNT);
    assert(data);

    /* If the byte count of the data for transmitting is zero, then return immediately.*/
    if (num_bytes == 0)
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }

    /* Start transmitting data*/
    num_byte_left = num_bytes;
    total_byte_data_transmitted = 0;
    cs.data_length = tx_info->data_length;
    cs.msg_id_type = tx_info->msg_id_type;
    
    while (num_byte_left)
    {
        if (num_byte_left < cs.data_length)
        {
            cs.data_length = num_byte_left;
        }

        /* Set up FlexCAN message buffer for transmitting data*/
        cs.code = kFlexCanTX_Data;
        result = flexcan_hal_set_mb_tx(instance, data, mb_idx, &cs, msg_id,
                                       (mb_data + total_byte_data_transmitted));

        if(result == kStatus_FLEXCAN_Success)
        {
            fsl_rtos_status syncStatus;
          
            do
            {
                syncStatus = sync_wait(&irqSync, 1);
            }while(syncStatus == kIdle);
            
            /* Wait for the interrupt*/
            if (syncStatus != kSuccess)
            {
                return kStatus_FLEXCAN_TimeOut;
            }
        }
        else
        {
            return result;
        }

        total_byte_data_transmitted += cs.data_length;
        num_byte_left -= cs.data_length;
    }

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_rx_mb_config
 * Description   : Configure a Rx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Rx message buffer as NOT_USED, enable
 * the Message Buffer interrupt, configure the message buffer code for Rx
 * message buffer as INACTIVE, copy user's buffer into the message buffer data
 * area, and configure the message buffer code for Rx message buffer as EMPTY.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_rx_mb_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *rx_info,
    uint32_t msg_id)
{
    flexcan_status_t result;
    flexcan_mb_code_status_rx_t cs;
    assert(instance < HW_CAN_INSTANCE_COUNT);
    assert(data);

    rx_mb_idx = mb_idx;
    cs.data_length = rx_info->data_length;
    cs.msg_id_type = rx_info->msg_id_type;

    /* Initialize rx mb*/
    cs.code = kFlexCanRX_NotUsed;
    result = flexcan_hal_set_mb_rx(instance, data, mb_idx, &cs, msg_id);
    if (result)
    {
         return result;
    }

    /* Enable MB interrupt*/
    result = flexcan_hal_enable_mb_interrupt(instance, data, mb_idx);
    if (result)
    {
         return result;
    }

    /* Initialize receive MB*/
    cs.code = kFlexCanRX_Inactive;
    result = flexcan_hal_set_mb_rx(instance, data, mb_idx, &cs, msg_id);
    if (result)
    {
         return result;
    }

    /* Set up FlexCAN message buffer fields for receiving data*/
    cs.code = kFlexCanRX_Empty;
    return flexcan_hal_set_mb_rx(instance, data, mb_idx, &cs, msg_id);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_rx_fifo_config
 * Description   : Confgure RX FIFO ID filter table elements.
 * This function will confgure RX FIFO ID filter table elements, and enable RX
 * FIFO interrupts.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_rx_fifo_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    flexcan_rx_fifo_id_element_format_t id_format,
    flexcan_id_table_t *id_filter_table)
{
    flexcan_status_t result;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Initialize rx fifo*/
    result = flexcan_hal_set_rx_fifo(instance, data, id_format, id_filter_table);
    if(result)
    {
         return result;
    }

    /* Enable RX FIFO interrupts*/
    for (uint8_t i = 5; i <= 7; i++)
    {
        result = flexcan_hal_enable_mb_interrupt(instance, data, i);
        if(result)
        {
             return result;
        }
    }
    
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_start_receive
 * Description   : Start receive data after a Rx MB interrupt occurs.
 * This function will lock Rx MB after a Rx MB interrupt occurs, get the Rx MB
 * field values, and unlock the Rx MB.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_start_receive(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    uint32_t receiveDataCount,
    bool *is_rx_mb_data,
    bool *is_rx_fifo_data,
    flexcan_mb_t *rx_mb,
    flexcan_mb_t *rx_fifo)
{
    flexcan_status_t result;
    uint32_t bit_mask = 0;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Find out bit mask. Each set bit represents an event bit to wait for*/
    if (data->is_rx_mb_needed)
    {
        bit_mask |= 1U << mb_idx;
    }

    if (data->is_rx_fifo_needed)
    {
        bit_mask |= 1 << 5;
    }
    
    if (bit_mask == 0)
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }

    while (receiveDataCount)
    {
        fsl_rtos_status syncStatus;  
      
        int_mb = false;
        int_fifo = false;

        do
        {
            syncStatus = sync_wait(&irqSync, 1);
        }while(syncStatus == kIdle);
        
        /* Wait for the interrupt*/
        if (syncStatus != kSuccess)
        {
            return kStatus_FLEXCAN_TimeOut;
        }

        if (int_mb)
        {
            *is_rx_mb_data = true;

            /* Lock RX MB*/
            result = flexcan_hal_lock_rx_mb(instance, data, mb_idx);
            if(result)
            {
                return result;
            }

            /* Get RX MB field values*/
            result = flexcan_hal_get_mb(instance, data, mb_idx, rx_mb);
            if(result)
            {
                return result;
            }
        }

        if (int_fifo)
        {
            *is_rx_fifo_data = true;

            /* Lock RX FIFO*/
            result = flexcan_hal_lock_rx_mb(instance, data, 0);
            if(result)
            {
                return result;
            }

            /* Get RX FIFO field values*/
            result = flexcan_hal_read_fifo(instance, rx_fifo);
            if(result)
            {
                return result;
            }
        }

        /* Unlock RX message buffer and RX FIFO*/
        flexcan_hal_unlock_rx_mb(instance);

        receiveDataCount--;
    }

   return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_receive
 * Description   : Configure Rx FIFO fields and Rx MB fields for receiving data.
 * This function will configure RX FIFO fields if Rx FIFO needed, configure RX
 * MB fields if Rx MB needed, and start receiving data.
 *
 *END**************************************************************************/
flexcan_status_t flexcan_receive(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *rx_info,
    uint32_t msg_id,
    flexcan_rx_fifo_id_element_format_t id_format,
    flexcan_id_table_t *id_filter_table,
    uint32_t receiveDataCount,
    flexcan_mb_t *rx_mb,
    flexcan_mb_t *rx_fifo)
{
    flexcan_status_t result;
    bool is_rx_mb_data = false;
    bool is_rx_fifo_data = false;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    int_mb = false;
    int_fifo = false;

    if (data->is_rx_fifo_needed)
    {
        /* Configure RX FIFO fields*/
        result = flexcan_rx_fifo_config(instance, data, id_format, id_filter_table);
        if (result)
        {
            return result;
        }
    }

    if (data->is_rx_mb_needed)
    {
        /* Configure RX MB fields*/
        result = flexcan_rx_mb_config(instance, data, mb_idx, rx_info, msg_id);
        if (result)
        {
            return result;
        }
    }

    /* Start receiving data*/
    return (flexcan_start_receive(instance, data, mb_idx, receiveDataCount,
                                  &is_rx_mb_data, &is_rx_fifo_data, rx_mb, rx_fifo));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : flexcan_shutdown
 * Description   : Shutdown a FlexCAN module.
 * This function will disable all FlexCAN interrupts, and disable the FlexCAN.
 *
 *END**************************************************************************/
uint32_t flexcan_shutdown(uint8_t instance)
{
    uint32_t i;
    assert(instance < HW_CAN_INSTANCE_COUNT);

    /* Disable the FlexCAN instance IRQs */
    for (i = 0; i < 4; i++)
    {
        /* Disable FlexCAN interrupts.*/
        interrupt_disable(flexcan_irq_ids[instance][i]);
    }

    /* Disable FlexCAN.*/
    flexcan_hal_disable(instance);

    /* Disable clock gate to FlexCAN module */
    return clock_manager_set_gate(kClockModuleFLEXCAN, instance, false);
}

