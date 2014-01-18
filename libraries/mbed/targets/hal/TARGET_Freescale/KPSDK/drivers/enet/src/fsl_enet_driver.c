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

#include "fsl_enet_driver.h"
#include "fsl_enet_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_phy_driver.h"
#include <string.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Define ENET's IRQ list */
extern IRQn_Type enet_irq_ids[HW_PIT_INSTANCE_COUNT][FSL_FEATURE_ENET_INTERRUPT_COUNT];
extern uint8_t enetIntMap[kEnetIntNum];

/*! @brief Define global value for ISR input parameter*/
void *enetIfHandle;

/*! @brief Define interupt priority*/
#define ENET_TX_PRIORITY     4
#define ENET_RX_PRIORITY     4
#define ENET_TS_PRIORITY     3	

/*! @brief Define external osc clk frequency*/
#define ENET_OSCCLK_OUTCLK_SRC   50000000  

#if FSL_FEATURE_ENET_SUPPORT_PTP
/*! @brief Define ptp mastertimer information*/
enet_ptp_master_time_data_t g_ptpMasterTime;
#endif

/*! @brief Define MAC driver API structure and for application of stack adaptor layer*/
const enet_mac_api_t g_enetMacApi = 
{
    enet_mac_init,
    enet_mac_close,
    enet_mac_send,
    enet_mac_receive,
    enet_mii_read,
    enet_mii_write,
    enet_mac_add_multicast_group,
    enet_mac_leave_multicast_group,
#if FSL_FEATURE_ENET_SUPPORT_PTP
    enet_ptp_ioctl,
#endif
};
/*******************************************************************************
 * Code
 ******************************************************************************/
#if FSL_FEATURE_ENET_SUPPORT_PTP
/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_ring_init
 * Return Value: The execution status.
 * Description:Initialize ENET ptp(Precision Time Synchronization Protocol) 
 * data buffer ring. ptp data buffer is used to store ptp message context and
 * the timestamp of the ptp message.
 *
 *END*********************************************************************/
uint32_t enet_ptp_ring_init(enet_mac_ptp_ts_ring_t *ptpTsRingPtr)
{
    /* Check input parameter*/
    if(!ptpTsRingPtr)
    {
        return kStatus_ENET_InvalidInput;
    }
	
    /* Allocate memory for enet private ptp ring*/
    ptpTsRingPtr->ptpTsDataPtr = (enet_mac_ptp_ts_data_t *)mem_allocate(ptpTsRingPtr->size * sizeof(enet_mac_ptp_ts_data_t));
    if(!ptpTsRingPtr->ptpTsDataPtr)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }

    ptpTsRingPtr->front = 0;
    ptpTsRingPtr->end = 0;

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_init
 * Return Value: The execution status.
 * Description:Initialize the ENET private ptp(Precision Time Synchronization Protocol)
 * data structure with basic configuration. All ptp data are stored there.
 *
 *END*********************************************************************/
uint32_t enet_ptp_init(enet_dev_if_t *enetIfPtr)
{
    /* Check the input parameters */
    if(!enetIfPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Allocate memory for private ptp buffer*/
    enetIfPtr->macContextPtr->privatePtpPtr = mem_allocate(sizeof(enet_private_ptp_buffer_t));
    if(!enetIfPtr->macContextPtr->privatePtpPtr)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }

    /* Initialize some ring parameter*/
    enetIfPtr->macContextPtr->privatePtpPtr->rxTimeStamp.size = enetIfPtr->macCfgPtr->ptpRingBufferNumber;
    enetIfPtr->macContextPtr->privatePtpPtr->txTimeStamp.size = enetIfPtr->macCfgPtr->ptpRingBufferNumber;

    /* Initialize required ring buffers*/
    enet_ptp_ring_init(&(enetIfPtr->macContextPtr->privatePtpPtr->rxTimeStamp));
    enet_ptp_ring_init(&(enetIfPtr->macContextPtr->privatePtpPtr->txTimeStamp));

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_start
 * Return Value: The execution status.
 * Description: Start the ENET ptp(Precision Time Synchronization Protocol)
 * timer. After this, the timer is ready for 1588 synchronization.  
 *
 *END*********************************************************************/
uint32_t enet_ptp_start(uint32_t instance, bool isSlaveEnabled)
{
    enet_config_ptp_timer_t ptpCfg;
	
    /* Check if this is the master ptp timer*/
    if(isSlaveEnabled)
    {
        g_ptpMasterTime.MasterPtpInstance = instance;
    }

    /* Initialzie timer configuration struct*/
    ptpCfg.isSlaveEnabled = isSlaveEnabled;
    ptpCfg.period = kEnetPtpAtperVaule;
    ptpCfg.clockIncease = ptpCfg.period / ENET_OSCCLK_OUTCLK_SRC;
	
    /* Restart ptp timer*/
    enet_hal_restart_ptp_timer(instance);
    /* Initialize ptp timer */
    enet_hal_init_ptp_timer(instance, &ptpCfg);
    /* Start ptp timer*/
    enet_hal_start_ptp_timer(instance, true);

#if FSL_FEATURE_ENET_PTP_TIMER_CHANNEL_INTERRUPT
    /* Initialize timer channel for timestamp interrupt for old silicon*/
    uint32_t compareValue = kEnetPtpAtperVaule - ptpCfg.clockIncease;
    enet_hal_set_timer_channel_compare(instance, ENET_TIMER_CHANNEL_NUM, compareValue);
    enet_hal_init_timer_channel(instance, ENET_TIMER_CHANNEL_NUM, kEnetChannelToggleCompare);
    enet_hal_set_timer_channel_compare(instance, ENET_TIMER_CHANNEL_NUM, compareValue);
#endif
	
    /* Enable master ptp timer interrupt */
    if(!ptpCfg.isSlaveEnabled)
    {
        enet_hal_config_interrupt(instance, kEnetTsTimerInterrupt, true);
        NVIC_SetPriority(enet_irq_ids[instance][enetIntMap[kEnetTstimerInt]], ENET_TS_PRIORITY);
        interrupt_enable(enet_irq_ids[instance][enetIntMap[kEnetTstimerInt]]);
    }
	
    return kStatus_ENET_Success;
}
/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_quick_parse
 * Return Value: The execution status.
 * Description: Quickly parse the packet and set the ptp message flag if 
 * this is a ptp message. It is called by enet transmit interface.
 *
 *END*********************************************************************/
uint32_t enet_ptp_quick_parse(uint8_t *packet,bool *isPtpMsg)
{
    uint8_t *buffer = packet;

    /* Set default value false*/
    *isPtpMsg = false;

    /* Check for vlan frame*/
    if(*(uint16_t *)(buffer+kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocol8021QVlan))
    {
        buffer += (sizeof(enet_8021vlan_header_t) - sizeof(enet_etherent_header_t));
    }

    if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIeee8023))
    {
        if(*(uint8_t *)(buffer + kEnetPtpEtherMsgTypeOffset) <= kEnetPtpEventMsgType)
        {
            *isPtpMsg = true;  
        }
    }
    else if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIpv4))
    {
        if((*(uint8_t *)(buffer + kEnetPtpIpVersionOffset) >> 4 ) == kEnetPacketIpv4Version)
        {
            if(((*(uint16_t *)(buffer + kEnetPtpUdpPortOffset)) == HTONS(kEnetPtpEventPort))&&
				(*(uint8_t *)(buffer + kEnetPtpUdpProtocolOffset) == kEnetPacketUdpVersion))
            {
                *isPtpMsg = true;  
            }
        }
    }
    else if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIpv6))
    {
        if((*(uint8_t *)(buffer + kEnetPtpIpVersionOffset) >> 4 ) == kEnetPacketIpv6Version)
        {
            if(((*(uint16_t *)(buffer + kEnetPtpIpv6UdpPortOffset)) == HTONS(kEnetPtpEventPort))&&
            (*(uint8_t *)(buffer + kEnetPtpIpv6UdpProtocolOffset) == kEnetPacketUdpVersion))
            {
                *isPtpMsg = true;  
            }
        }
    }

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_parse
 * Return Value: The execution status.
 * Description: Parse the message and store the ptp message information if 
 * it is a ptp message. this is called by the tx/rx store timestamp interface. 
 *
 *END*********************************************************************/
uint32_t enet_ptp_parse(uint8_t *packet, enet_mac_ptp_ts_data_t *ptpTsPtr, bool *isPtpMsg)
{
    uint8_t *buffer = packet;

    /* Set default value false*/
    *isPtpMsg = false;
 
    /* Check for vlan frame*/
    if(*(uint16_t *)(buffer+kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocol8021QVlan))
    {
        buffer += (sizeof(enet_8021vlan_header_t) - sizeof(enet_etherent_header_t));
    }

    if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIeee8023))
    {
        if(*(uint8_t *)(buffer + kEnetPtpEtherMsgTypeOffset) <= kEnetPtpEventMsgType)
        {
            /* It's a ptpv2 message and store the ptp header information*/
            ptpTsPtr->version = (*(uint8_t *)(buffer + kEnetPtpEtherVersionOffset))&0x0F;
            ptpTsPtr->messageType = (*(uint8_t *)(buffer + kEnetPtpEtherMsgTypeOffset))& 0x0F;
            ptpTsPtr->sequenceId = HTONS(*(uint16_t *)(buffer + kEnetPtpEtherSequenceIdOffset));
            memcpy((void *)&ptpTsPtr->sourcePortId[0],(void *)(buffer + kEnetPtpEtherClockIdOffset),kEnetPtpSourcePortIdLen);
            /* Set the ptp message flag*/
            *isPtpMsg = true;  
        }
    }
    else if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIpv4))
    {
        if((*(uint8_t *)(buffer + kEnetPtpIpVersionOffset) >> 4 ) == kEnetPacketIpv4Version)
        {
            if(((*(uint16_t *)(buffer + kEnetPtpUdpPortOffset)) == HTONS(kEnetPtpEventPort))&&
            (*(uint8_t *)(buffer + kEnetPtpUdpProtocolOffset) == kEnetPacketUdpVersion))
            {
                /* It's a IPV4 ptp message and store the ptp header information*/
                ptpTsPtr->version = (*(uint8_t *)(buffer + kEnetPtpUdpVersionoffset))&0x0F;
                ptpTsPtr->messageType = (*(uint8_t *)(buffer + kEnetPtpUdpMsgTypeOffset))& 0x0F;
                ptpTsPtr->sequenceId = HTONS(*(uint16_t *)(buffer + kEnetPtpUdpSequenIdOffset));
                memcpy(( void *)&ptpTsPtr->sourcePortId[0],( void *)(buffer + kEnetPtpUdpClockIdOffset),kEnetPtpSourcePortIdLen);
                /* Set the ptp message flag*/
                *isPtpMsg = true;  
            }
        }
    }
    else if(*(uint16_t *)(buffer + kEnetPtpEtherPktTypeOffset) == HTONS(kEnetProtocolIpv6))
    {
        if((*(uint8_t *)(buffer + kEnetPtpIpVersionOffset) >> 4 ) == kEnetPacketIpv6Version)
        {
            if(((*(uint16_t *)(buffer + kEnetPtpIpv6UdpPortOffset)) == HTONS(kEnetPtpEventPort))&&
              (*(uint8_t *)(buffer + kEnetPtpIpv6UdpProtocolOffset) == kEnetPacketUdpVersion))
            {
                /* It's a IPV6 ptp message and store the ptp header information*/
                ptpTsPtr->version = (*(uint8_t *)(buffer + kEnetPtpIpv6UdpVersionOffset))&0x0F;
                ptpTsPtr->messageType = (*(uint8_t *)(buffer + kEnetPtpIpv6UdpMsgTypeOffset))& 0x0F;
                ptpTsPtr->sequenceId = HTONS(*(uint16_t *)(buffer + kEnetPtpIpv6UdpSequenceIdOffset));
                memcpy(( void *)&ptpTsPtr->sourcePortId[0],( void *)(buffer + kEnetPtpIpv6UdpClockIdOffset),kEnetPtpSourcePortIdLen);
                /* Set the ptp message flag*/
                *isPtpMsg = true;  
            }
        }
    }

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_get_time
 * Return Value: The execution status.
 * Description: Get current enet ptp time. 
 * This interface is use by 1588 stack to get the current value from the ptp timer
 * through ioctl interface.
 *END*********************************************************************/
uint32_t enet_ptp_get_time(enet_mac_ptp_time_t *ptpTimerPtr)
{
    /* Check input parameters*/
    if(!ptpTimerPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Interrupt disable*/
    interrupt_disable_global();

    /* Get the current value of the master time*/
    ptpTimerPtr->second = g_ptpMasterTime.second;
    enet_hal_set_timer_capture(g_ptpMasterTime.MasterPtpInstance);
    ptpTimerPtr->nanosecond = enet_hal_get_current_time(g_ptpMasterTime.MasterPtpInstance);

    /* Enable interrupt*/
    interrupt_enable_global();

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_set_time
 * Return Value: The execution status.
 * Description: Set enet ptp time. 
 * This interface is use by 1588 stack to set the current ptp timer
 * through ioctl interface.
 *END*********************************************************************/
uint32_t enet_ptp_set_time(enet_mac_ptp_time_t *ptpTimerPtr)
{
    /* Check input parameters*/
    if(!ptpTimerPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Disable interrupt*/
    interrupt_disable_global();
    /* Set ptp timer*/
    g_ptpMasterTime.second = ptpTimerPtr->second;
    enet_hal_set_current_time(g_ptpMasterTime.MasterPtpInstance, ptpTimerPtr->nanosecond);

    /* Enable interrupt*/
    interrupt_enable_global();
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_correction_time
 * Return Value: The execution status.
 * Description: Adjust enet ptp timer. 
 * This interface is mainly the adjust algorithm for ptp timer synchronize.
 * this function is used to adjust ptp timer to synchronize with master timer.
 *END*********************************************************************/
uint32_t enet_ptp_correction_time(uint32_t instance, int32_t drift)
{
    uint32_t clockFreq,clockIncrease,adjIncrease,corrPeriod,corrIncrease,count;
    uint32_t gapMax = 0xFFFFFFFF,gapTemp,adjPeriod = 1;

    /* Get the ptp clock source frequency*/
    if(clock_manager_get_frequency_by_source(kClockTimeSrc, &clockFreq) != kClockManagerSuccess)
    {
         return kStatus_ENET_GetClockFreqFail;
    }
	
    /* Calculate clock period of the ptp timer*/
    clockIncrease = kEnetPtpAtperVaule / clockFreq ;
	
    if(drift != 0)
    {
        if(abs(drift) >= clockFreq)
        {
            /* Drift is greater than the 1588 clock frequency the correction should be done
			every tick of the timer*/   
            corrPeriod = 1;	
            corrIncrease = (uint32_t)(abs(drift)/clockFreq);
        }
        else
        {
            /* Drift is smaller than the 1588 clock frequency*/
            if(abs(drift) > (clockFreq / clockIncrease))
            {
                adjIncrease = clockIncrease / kEnetBaseIncreaseUnit;
            }
            else if(abs(drift) > (clockFreq / (2*kEnetBaseIncreaseUnit*clockIncrease)))
            {
                adjIncrease = clockIncrease / (2*kEnetBaseIncreaseUnit);
                adjPeriod =  kEnetBaseIncreaseUnit;
            }
            else
            {
                adjIncrease = clockIncrease / (2*kEnetBaseIncreaseUnit);
                adjPeriod = 2*kEnetBaseIncreaseUnit;
            }
            for(count = 1; count < adjIncrease; count++)
            {
                gapTemp = (clockFreq * adjPeriod * count) % abs(drift);
                if(!gapTemp)
                {
                    corrIncrease = count;
                    corrPeriod = (uint32_t)((clockFreq * adjPeriod * count) / abs(drift));
                    break;
                }
                else if(gapTemp < gapMax)
                { 
                    corrIncrease = count;
                    corrPeriod = (uint32_t)((clockFreq * adjPeriod * count) / abs(drift));
                    gapMax = gapTemp;
                }
            }
        }
        /* Calculate the clock correction increase value*/
        if(drift < 0)
        {
            corrIncrease = clockIncrease - corrIncrease;
        }
        else
        {
            corrIncrease = clockIncrease + corrIncrease;
        }
        /* Adjust the ptp timer*/	
        enet_hal_adjust_ptp_timer(instance, corrPeriod, corrIncrease);
    }
    else
    {
        /* Adjust the ptp timer*/
        enet_hal_adjust_ptp_timer(instance, 0, clockIncrease);
    }
   
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_store_tx_timestamp
 * Return Value: The execution status.
 * Description: Store the transmit ptp timestamp. 
 * This interface is to store transmit ptp timestamp and is called by transmit function.
 *END*********************************************************************/
uint32_t enet_ptp_store_tx_timestamp(enet_private_ptp_buffer_t *ptpBuffer,void *bdPtr)
{
    bool isPtpMsg,ptpTimerWrap;
    enet_mac_ptp_ts_data_t ptpTsData;
    enet_mac_ptp_time_t ptpTimerPtr;
    uint8_t * bdBufferPtr;
    uint32_t result = kStatus_ENET_Success;
		
    /* Check input parameter*/
    if(!ptpBuffer)
    {
        return kStatus_ENET_InvalidInput;
    }
  
    /* Parse the message packet to check if there is a ptp message*/	
    bdBufferPtr = enet_hal_get_bd_buffer(bdPtr);
    result = enet_ptp_parse(bdBufferPtr, &ptpTsData, &isPtpMsg);
    if(result != kStatus_ENET_Success)
    {
        return result;
    }
	
    /* Store transmit timestamp of the ptp message*/
    if(isPtpMsg)
    {
        /* Get transmit timestamp nanosecond*/
        ptpTsData.timeStamp.nanosecond = enet_hal_get_bd_timestamp(bdPtr);

        interrupt_disable_global();

        /* Get current ptp timer nanosecond value*/
        enet_ptp_get_time(&ptpTimerPtr);

        /* Get ptp timer wrap event*/
        ptpTimerWrap = enet_hal_get_interrupt_status(g_ptpMasterTime.MasterPtpInstance, kEnetTsTimerInterrupt);

        /* Get transmit timestamp second*/
        if((ptpTimerPtr.nanosecond > ptpTsData.timeStamp.nanosecond) || 
        ((ptpTimerPtr.nanosecond < ptpTsData.timeStamp.nanosecond) && ptpTimerWrap))
        {
            ptpTsData.timeStamp.second = g_ptpMasterTime.second;
        }
        else 
        {
            ptpTsData.timeStamp.second = g_ptpMasterTime.second - 1;
        }
     
        interrupt_enable_global();

        /* Add the new timestamp data into the ptp ring buffer*/
        result = enet_ptp_ring_update(&(ptpBuffer->txTimeStamp), &ptpTsData);
    }

    return result;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_store_rx_timestamp
 * Return Value: The execution status.
 * Description: Store the receive ptp packet timestamp. 
 * This interface is to store receive ptp packet timestamp and is called by receive function.
 *END*********************************************************************/
uint32_t enet_ptp_store_rx_timestamp(enet_private_ptp_buffer_t *ptpBuffer, uint8_t *packet, void *bdPtr)
{
    enet_mac_ptp_ts_data_t ptpTsData;
    enet_mac_ptp_time_t ptpTimerPtr;
    bool  isPtpMsg = false, ptpTimerWrap;
    uint32_t result;
   	
    /* Check input parameter*/
    if((!ptpBuffer) || (!packet) || (!bdPtr))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Check if the message is a ptp message */
    result = enet_ptp_parse(packet, &ptpTsData, &isPtpMsg);
    if(result != kStatus_ENET_Success)
    {
        return result;
    }

    /* Store the receive timestamp of the ptp message*/
    if(isPtpMsg)
    {
        /* Get the timestamp from the bd buffer*/
        ptpTsData.timeStamp.nanosecond = enet_hal_get_bd_timestamp(bdPtr);

        interrupt_disable_global();

        /* Get current ptp timer nanosecond value*/
        enet_ptp_get_time(&ptpTimerPtr);

        /* Get ptp timer wrap event*/
        ptpTimerWrap = enet_hal_get_interrupt_status(g_ptpMasterTime.MasterPtpInstance,
            kEnetTsTimerInterrupt);

        /* Get transmit timestamp second*/
        if((ptpTimerPtr.nanosecond > ptpTsData.timeStamp.nanosecond) || 
         ((ptpTimerPtr.nanosecond < ptpTsData.timeStamp.nanosecond) && ptpTimerWrap))
        {
            ptpTsData.timeStamp.second = g_ptpMasterTime.second;
        }
        else 
        {
            ptpTsData.timeStamp.second = g_ptpMasterTime.second - 1;
        }	
        interrupt_enable_global();
			
        /* Add the new timestamp data into the ptp ring buffer*/
        result = enet_ptp_ring_update(&(ptpBuffer->rxTimeStamp), &ptpTsData);

    }
        
    return result;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_l2queue_init
 * Return Value: The execution status.
 * Description: Initialize buffer queue for ptp layer2 etherent packets. 
 * This interface is to initialize the layer2 etherent frame buffer queue.
 *END*********************************************************************/
uint32_t enet_ptp_l2queue_init(enet_private_ptp_buffer_t *ptpBuffer)
{
    uint32_t index;
    enet_ptp_l2queue_t *ptpL2QuePtr;

    /* Check input parameters*/
    if(!ptpBuffer)
    {
        return kStatus_ENET_InvalidInput;
    }

    ptpL2QuePtr = (enet_ptp_l2queue_t *)mem_allocate(sizeof(enet_ptp_l2queue_t));
    if(!ptpL2QuePtr)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }
    ptpBuffer->l2QueuePtr = ptpL2QuePtr;
 	
    /* Initialize the queue*/
    ptpL2QuePtr->writeIdex = 0;
    ptpL2QuePtr->readIdx = 0;
    for(index = 0; index < kEnetPtpL2bufferNumber; index++)
    {
        ptpL2QuePtr->l2Packet[index].length = 0;
    }
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_service_l2packet
 * Return Value: The execution status.
 * Description: Add the ptp layer2 etherent packet to the queue.
 * This interface is the call back for ethernet 1588 layer2 packets to 
 * add queue for ptp layer2 etherent packets. 
 *END*********************************************************************/
uint32_t enet_ptp_service_l2packet(enet_dev_if_t * enetIfPtr, uint8_t *packet, uint16_t length)
{
    enet_ptp_l2queue_t *ptpQuePtr;
	
    /* Check input parameter*/
    if((!enetIfPtr) || (!packet))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Check the l2queue buffer*/
    ptpQuePtr = enetIfPtr->macContextPtr->privatePtpPtr->l2QueuePtr;
    if(!ptpQuePtr)
    {
        return kStatus_ENET_Layer2QueueNull;
    }
	
    /* Check if the queue is full*/
    if(ptpQuePtr->l2Packet[ptpQuePtr->writeIdex].length != 0 )
    {
        return kStatus_ENET_Layer2BufferFull;
    }

    /* Store the packet*/
    ptpQuePtr->l2Packet[ptpQuePtr->writeIdex].length = length;
    memcpy((void *)packet, (void *)ptpQuePtr->l2Packet[ptpQuePtr->writeIdex].packet, length);

    /* Increse the index to the next one*/
    ptpQuePtr->writeIdex = (ptpQuePtr->writeIdex + 1) % kEnetPtpL2bufferNumber;

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_send_l2packet
 * Return Value: The execution status.
 * Description: Send the ptp layer2 etherent packet to the net.
 * This interface is used to send the ptp layer2 ethernet packet and 
 * this interface is called by 1588 stack. 
 *END*********************************************************************/
uint32_t enet_ptp_send_l2packet(enet_dev_if_t * enetIfPtr, void *paramPtr)
{
    enet_etherent_header_t *tempBuffer;
    uint32_t result;
    
    /* Check input parameters*/
    if((!enetIfPtr) || (!paramPtr))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Allocate memory*/
    tempBuffer = (enet_etherent_header_t *)mem_allocate(sizeof(enet_etherent_header_t) + ((enet_ptp_l2_ethernet_t *)paramPtr)->length);
    if(!tempBuffer)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }

    /* Add etherent header*/
    memcpy(&tempBuffer->destAddr[0], &(((enet_ptp_l2_ethernet_t *)paramPtr)->hwAddr[0]), kEnetMacAddrLen);
    memcpy(&tempBuffer->sourceAddr[0], &enetIfPtr->macCfgPtr->macAddr[0], kEnetMacAddrLen);
    tempBuffer->type = HTONS(kEnetProtocolIeee8023);

    /* Copy the real data*/
    memcpy((void *)(((enet_ptp_l2_ethernet_t *)paramPtr)->ptpMsg),(void *)(tempBuffer + sizeof(enet_etherent_header_t)),((enet_ptp_l2_ethernet_t *)paramPtr)->length);

    /* Check transmit packets*/
    if((((enet_ptp_l2_ethernet_t *)paramPtr)->length + sizeof(enet_etherent_header_t)) > enetIfPtr->maxFrameSize)
    {
       enetIfPtr->stats.statsTxLarge++;
       enetIfPtr->stats.statsTxDiscard++;
       return kStatus_ENET_Layer2OverLarge;
    }

    /* Send packet to the device*/
    result = enetIfPtr->macApiPtr->enet_mac_send(enetIfPtr, (uint8_t *)tempBuffer,
        (((enet_ptp_l2_ethernet_t *)paramPtr)->length + sizeof(enet_etherent_header_t)));
    if(result != kStatus_ENET_Success)
    {
       mem_free(tempBuffer);
       return result;
    }

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_receive_l2packet
 * Return Value: The execution status.
 * Description: Receive the ptp layer2 etherent packet to the net.
 * This interface is used to receive the ptp layer2 ethernet packet and 
 * this interface is called by 1588 stack. 
 *END*********************************************************************/
uint32_t enet_ptp_receive_l2packet(enet_dev_if_t * enetIfPtr,void *paramPtr)
{
    enet_private_ptp_buffer_t *ptpBuffer;
    uint32_t result = kStatus_ENET_Success;
    uint16_t len;
   
    /* Check input parameters*/
    if((!enetIfPtr) || (!paramPtr) || (!enetIfPtr->macContextPtr->privatePtpPtr))
    {
        return kStatus_ENET_InvalidInput;
    }

    ptpBuffer = enetIfPtr->macContextPtr->privatePtpPtr;

    /* Disable interrupt to avoid buffer data overwrite*/
    interrupt_disable_global();
	
    /* Check if the queue is full*/
    if(ptpBuffer->l2QueuePtr->readIdx == ptpBuffer->l2QueuePtr->writeIdex)
    {
        result = kStatus_ENET_Layer2BufferFull;
    }
    else
    {
        /* Data process*/
        len = ptpBuffer->l2QueuePtr->l2Packet[ptpBuffer->l2QueuePtr->readIdx].length;
        memcpy((void *)ptpBuffer->l2QueuePtr->l2Packet[ptpBuffer->l2QueuePtr->readIdx].packet, 
           (void *)((enet_ptp_l2_ethernet_t *)paramPtr)->ptpMsg, len);

        /* Clear the queue parameter*/
        ptpBuffer->l2QueuePtr->l2Packet[ptpBuffer->l2QueuePtr->readIdx].length = 0;
        ptpBuffer->l2QueuePtr->readIdx = 
            (ptpBuffer->l2QueuePtr->readIdx + 1)% kEnetPtpL2bufferNumber;		
    }
	
    interrupt_enable_global();
    memcpy(&(((enet_ptp_l2_ethernet_t *)paramPtr)->hwAddr[0]),&(enetIfPtr->macCfgPtr->macAddr[0]), kEnetMacAddrLen);
    
    return result;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_ioctl
 * Return Value: The execution status.
 * Description: The function provides the handler for 1588 stack to do ptp ioctl.
 * This interface provides ioctl for 1588 stack to get or set timestamp and do ptp  
 * version2 packets process. Additional user specified driver functionality may be 
 * added if necessary.
 *END*********************************************************************/

uint32_t enet_ptp_ioctl(enet_dev_if_t * enetIfPtr, uint32_t commandId, void *inOutPtr)
{
   
    uint32_t result = kStatus_ENET_Success;
    enet_private_ptp_buffer_t *buffer;
    enet_mac_ptp_time_t ptpTimer;

    /*Check input parameters*/
    if(!enetIfPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /*Check private PTP buffer*/
    buffer =  enetIfPtr->macContextPtr->privatePtpPtr;
    if(!buffer)
    {
        return kStatus_ENET_InvalidInput;
    }
	
    switch(commandId)
    {
        case kEnetPtpGetRxTimestamp:
            /* Get receive timestamp*/
            result = enet_ptp_ring_search(&buffer->rxTimeStamp,
                (enet_mac_ptp_ts_data_t *)inOutPtr); 
            break;
        case kEnetPtpGetTxTimestamp:
            /* Get transmit timestamp*/
            result = enet_ptp_ring_search(&buffer->txTimeStamp,
                (enet_mac_ptp_ts_data_t *)inOutPtr);
            break;
        case kEnetPtpGetCurrentTime:
            /* Get current time*/
            result = enet_ptp_get_time(&ptpTimer);
            inOutPtr = (enet_mac_ptp_time_t *)&ptpTimer;
            break;
        case kEnetPtpSetCurrentTime:
            /* Set current time*/
            ptpTimer.second = ((enet_mac_ptp_time_t *)inOutPtr)->second;
            ptpTimer.nanosecond = ((enet_mac_ptp_time_t *)inOutPtr)->nanosecond;
            result = enet_ptp_set_time(&ptpTimer);
            break;
        case kEnetPtpFlushTimestamp:
            /* Reset receive and transmit buffer*/
            buffer->rxTimeStamp.end = 0;
            buffer->rxTimeStamp.front = 0;
            buffer->rxTimeStamp.size = kEnetPtpL2bufferNumber;
            buffer->txTimeStamp.end = 0;
            buffer->txTimeStamp.front = 0;
            buffer->txTimeStamp.size = kEnetPtpL2bufferNumber;			
            break;
        case kEnetPtpCorrectTime:
            /* Adjust time*/
            result = enet_ptp_correction_time(enetIfPtr->deviceNumber, 
                ((enet_ptp_drift_t *)inOutPtr)->drift);
            break;
        case kEnetPtpSendEthernetPtpV2:
            /* Send layer2 packet*/
            result = enet_ptp_send_l2packet(enetIfPtr, inOutPtr);
            break;
        case kEnetPtpReceiveEthernetPtpV2:
            /* Receive layer2 packet*/
            result = enet_ptp_receive_l2packet(enetIfPtr, inOutPtr);
            break;
        default:
            result = kStatus_ENET_UnknownCommand;
            break;
    }
    return result;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_stop
 * Return Value: The execution status.
 * Description:Stop enet ptp timer. 
 *
 *END*********************************************************************/
uint32_t enet_ptp_stop(uint32_t instance)
{	
    /* Disable ptp timer*/
    enet_hal_start_ptp_timer(instance, false);
    enet_hal_restart_ptp_timer(instance);
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_ring_is_full
 * Return Value: true if the ptp ring is full, false if not.
 * Description: Calcuate the number of used ring buffers to see if the 
 * ring buffer queue is full. 
 *
 *END*********************************************************************/
bool enet_ptp_ring_is_full(enet_mac_ptp_ts_ring_t *ptpTsRingPtr)
{
    uint32_t availBuffer = 0;
			
    if(ptpTsRingPtr->end > ptpTsRingPtr->front)
    {
        availBuffer =  ptpTsRingPtr->end - ptpTsRingPtr->front;       
    }
    else if(ptpTsRingPtr->end < ptpTsRingPtr->front)
    {
        availBuffer = ptpTsRingPtr->size - (ptpTsRingPtr->front - ptpTsRingPtr->end);
    }

    if(availBuffer == (ptpTsRingPtr->size - 1))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_ring_update
 * Return Value: The execution status.
 * Description: Update the ring buffers. 
 *
 *END*********************************************************************/
uint32_t enet_ptp_ring_update(enet_mac_ptp_ts_ring_t *ptpTsRingPtr, enet_mac_ptp_ts_data_t *data)
{
    /* Check input parameter*/
    if((!ptpTsRingPtr) || (!data))
    {
        return kStatus_ENET_InvalidInput;
    }
	
    /* Return if the buffers ring is full*/
    if(enet_ptp_ring_is_full(ptpTsRingPtr))
    {
        return kStatus_ENET_PtpringBufferFull;
    }
	
    /* Copy the new data into the buffer*/
    memcpy((ptpTsRingPtr->ptpTsDataPtr + ptpTsRingPtr->end), data, 
        sizeof(enet_mac_ptp_ts_data_t));

    /* Increse the buffer pointer to the next empty one*/
    ptpTsRingPtr->end = enet_ptp_ring_index(ptpTsRingPtr->size, ptpTsRingPtr->end, 1);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_ring_search
 * Return Value: The execution status.
 * Description: Search the element in the ring buffers with the message 
 * sequence Id, Clock Id, ptp message version etc. 
 *
 *END*********************************************************************/
uint32_t enet_ptp_ring_search(enet_mac_ptp_ts_ring_t *ptpTsRingPtr, enet_mac_ptp_ts_data_t *data)
{
    uint32_t index,size;
	
    /* Check input parameter*/
    if((!ptpTsRingPtr) || (!data))
    {
        return kStatus_ENET_InvalidInput;
    } 
	
    /* Check the ring buffer*/
    if(ptpTsRingPtr->front == ptpTsRingPtr->end)
    {
        return kStatus_ENET_PtpringBufferEmpty;
    }
	
    /* Serach the element in the ring buffer*/
    index = ptpTsRingPtr->front;
    size = ptpTsRingPtr->size;
    while(index != ptpTsRingPtr->end)
    {
        if(((ptpTsRingPtr->ptpTsDataPtr + index)->sequenceId == data->sequenceId)&&
        (!memcmp((( void *)&(ptpTsRingPtr->ptpTsDataPtr + index)->sourcePortId[0]),
             ( void *)&data->sourcePortId[0],kEnetPtpSourcePortIdLen))&&
        ((ptpTsRingPtr->ptpTsDataPtr + index)->version == data->version)&&
        ((ptpTsRingPtr->ptpTsDataPtr + index)->messageType == data->messageType))
        {
            break;
        }

        /* Increase the ptp ring index*/
        index = enet_ptp_ring_index(size, index, 1);
    }

    if(index == ptpTsRingPtr->end)
    {
        /* Check if buffers is full*/
        if(enet_ptp_ring_is_full(ptpTsRingPtr))
        {
            /* Drop one in the front*/
            ptpTsRingPtr->front = enet_ptp_ring_index(size, index, 1);
        }
        return kStatus_ENET_PtpringBufferFull;
	}

    /* Get the right timestamp of the required ptp message*/
    data->timeStamp.second = (ptpTsRingPtr->ptpTsDataPtr + index)->timeStamp.second;
    data->timeStamp.nanosecond = 
       (ptpTsRingPtr->ptpTsDataPtr + index)->timeStamp.nanosecond;

    /* Increase the index*/
    ptpTsRingPtr->front = enet_ptp_ring_index(size, index, 1);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_ptp_deinit
 * Return Value: The execution status.
 * Description: Free enet ptp data buffers. 
 *
 *END*********************************************************************/
uint32_t enet_ptp_deinit(enet_mac_context_t *enetContextPtr)
{
    /* Check the input parameters*/
    if(!enetContextPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Free timestamp data buffer*/
    mem_free(enetContextPtr->privatePtpPtr);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_ts_isr
 * Description: ENET timer isr.
 * This interface is the ptp timer interrupt handler.
 *END*********************************************************************/
void enet_mac_ts_isr(void *enetIfPtr)
{
    uint32_t number;
	
    /*Check input parameter*/
    if(!enetIfPtr)
    {
        return;
    }

    number = ((enet_dev_if_t *)enetIfPtr)->deviceNumber;
	
    /*Get interrupt status*/
    if(enet_hal_get_interrupt_status(number, kEnetTsTimerInterrupt))
    {
#if FSL_FEATURE_ENET_PTP_TIMER_CHANNEL_INTERRUPT
        uint32_t frequency;
        /* Get current clock frequency*/
        clock_manager_get_frequency_by_source(kClockTimeSrc, &frequency);
        enet_hal_set_timer_channel_compare(number, ENET_TIMER_CHANNEL_NUM, 
            (kEnetPtpAtperVaule - kEnetPtpAtperVaule/frequency));
        enet_hal_clear_timer_channel_flag(number, ENET_TIMER_CHANNEL_NUM);
#else
        /*Clear interrupt events*/
        enet_hal_clear_interrupt(number, kEnetTsTimerInterrupt);
#endif
        /* Increase timer second counter*/
        g_ptpMasterTime.second++;
    }
}

#endif

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mii_read
 * Return Value: The execution status.
 * Description: Read function.
 * This interface read data over the (R)MII bus from the specified phy register,
 * This function is called by all phy intrefaces.
 *END*********************************************************************/
uint32_t enet_mii_read(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t *dataPtr)
{
    uint32_t  counter;
	
    /* Check the input parameters*/
    if(!dataPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Check if the mii is enabled*/
    if(!enet_hal_is_mii_enabled(instance))
    {
        return kStatus_ENET_Miiuninitialized;
    }

    /* Clear the MII interrupt event*/
    enet_hal_clear_interrupt(instance, kEnetMiiInterrupt);

    /* Read command operation*/
    enet_hal_set_mii_command(instance, phyAddr, phyReg, kEnetReadValidFrame, 0);

    /* Poll for MII complete*/
    for(counter = 0; counter < kEnetMaxTimeout; counter++)
    {
        if(enet_hal_get_interrupt_status(instance, kEnetMiiInterrupt))
        {
            break;
        }	
    }

    /* Check for timeout*/
    if(counter == kEnetMaxTimeout)
    {
        return kStatus_ENET_TimeOut;
    }

    /* Get data from mii register*/
    *dataPtr = enet_hal_get_mii_data(instance);

    /* Clear MII intrrupt event*/
    enet_hal_clear_interrupt(instance, kEnetMiiInterrupt);
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mii_write
 * Return Value: The execution status.
 * Description: Write function.
 * This interface write data over the (R)MII bus to the specified Phy register, This is   
 * This function is called by all phy intrefaces.
 *END*********************************************************************/
uint32_t enet_mii_write(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t data)
{
    uint32_t counter;

    /* Check if the mii is enabled*/
    if(!enet_hal_is_mii_enabled(instance))
    {
        return kStatus_ENET_Miiuninitialized;
    }

    /* Clear the MII interrupt event*/
    enet_hal_clear_interrupt(instance, kEnetMiiInterrupt);

    /* Read command operation*/
    enet_hal_set_mii_command(instance, phyAddr, phyReg, kEnetWriteValidFrame, data);

    /* Poll for MII complete*/
    for(counter = 0; counter < kEnetMaxTimeout; counter++)
    {
        if(enet_hal_get_interrupt_status(instance, kEnetMiiInterrupt))
        {
            break;
        }		
    }

    /* Check for timeout*/
    if(counter == kEnetMaxTimeout)
    {
        return kStatus_ENET_TimeOut;
    }

    /* Clear MII intrrupt event*/
    enet_hal_clear_interrupt(instance, kEnetMiiInterrupt);
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_bd_init
 * Return Value: The execution status.
 * Description:Initialize the ENET receive and transmit buffer descriptor
 * This function prepare all of the transmit and receive buffer descriptors
 * for the enet module. it is called by the enet mac initialze interface.
 *END*********************************************************************/
uint32_t enet_mac_bd_init(enet_dev_if_t * enetIfPtr)
{
    void  *bdPtr, *bdTempPtr;
    uint16_t bdNumber;
    bool isLastBd = false;
    uint8_t *bufferPtr,*bufferTempPtr, counter;
    uint32_t bdSize,rxBuffer,rxBufferAlign,txBufferAlign,txLargeBuffer;
	
    /* Check the input parameters*/
    if (enetIfPtr == NULL)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* The receive buffer number should be at least equal to bd numbers*/
    if( enetIfPtr->macCfgPtr->rxBdNumber > enetIfPtr->macCfgPtr->rxBufferNumber)
    {
        return kStatus_ENET_InvalidInput;
    }
	
    /* Allocate buffer for enet mac context*/
    enetIfPtr->macContextPtr = 
        (enet_mac_context_t *)mem_allocate_zero(sizeof(enet_mac_context_t));
    if(!enetIfPtr->macContextPtr)
    {
        return kStatus_ENET_InvalidInput;
    }
 
    /* Get bd size*/
    bdSize = enet_hal_get_bd_size();
    enetIfPtr->macContextPtr->bufferdescSize = bdSize;

	/* Initialize the bd status*/
	enetIfPtr->macContextPtr->isRxFull = false;
	enetIfPtr->macContextPtr->isTxFull = false;
	
    /* Allocate enet receive buffer descriptors*/
    bdPtr = (void *)mem_allocate_zero((bdSize * enetIfPtr->macCfgPtr->rxBdNumber) + 
        enetIfPtr->macCfgPtr->bdAlignment);
    if(!bdPtr)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }
    /* Initialize receive bd base address and current address*/
    enetIfPtr->macContextPtr->rxBdBasePtr = (void *)ENET_ALIGN(((uint32_t)bdPtr),
        enetIfPtr->macCfgPtr->bdAlignment);
    enetIfPtr->macContextPtr->rxBdCurPtr = enetIfPtr->macContextPtr->rxBdBasePtr;
	enetIfPtr->macContextPtr->rxBdDirtyPtr = enetIfPtr->macContextPtr->rxBdBasePtr;
		
    /* Allocate enet transmit buffer descriptors*/
    bdPtr = (void *)mem_allocate_zero((bdSize * enetIfPtr->macCfgPtr->txBdNumber + 
        enetIfPtr->macCfgPtr->bdAlignment));
    if(!bdPtr)
    {
        return kStatus_ENET_MemoryAllocateFail;
    }
    /* Initialize transmit bd base address and current address etc*/
    enetIfPtr->macContextPtr->txBdBasePtr = (void *)ENET_ALIGN((uint32_t)bdPtr,
        enetIfPtr->macCfgPtr->bdAlignment);
    enetIfPtr->macContextPtr->txBdCurPtr = enetIfPtr->macContextPtr->txBdBasePtr;
    enetIfPtr->macContextPtr->txBdDirtyPtr = enetIfPtr->macContextPtr->txBdBasePtr;
	
    /* Allocate the transmit and receivce date buffers*/
    rxBufferAlign = ENET_ALIGN(enetIfPtr->macCfgPtr->rxBufferSize,
        enetIfPtr->macCfgPtr->rxBufferAlignment);
    enetIfPtr->macContextPtr->rxBufferSizeAligned = rxBufferAlign;
    rxBuffer = enetIfPtr->macCfgPtr->rxBufferNumber *rxBufferAlign + 
        enetIfPtr->macCfgPtr->rxBufferAlignment;
    bufferPtr = mem_allocate_zero(rxBuffer);
    if(!bufferPtr)
    {
         return kStatus_ENET_MemoryAllocateFail;
    }
    bufferPtr = (uint8_t *)ENET_ALIGN((uint32_t)bufferPtr, enetIfPtr->macCfgPtr->rxBufferAlignment);
    enetIfPtr->macContextPtr->rxBufferPtr = NULL;
    for(counter = 0; counter < enetIfPtr->macCfgPtr->rxBufferNumber; counter++)
    {
        enet_mac_enqueue_buffer((void **)&enetIfPtr->macContextPtr->rxBufferPtr, 
            bufferPtr);
        bufferPtr += rxBufferAlign;
    }

    /*Initialize the large receive buffer*/
    rxBufferAlign = ENET_ALIGN(enetIfPtr->maxFrameSize,
        enetIfPtr->macCfgPtr->rxBufferAlignment);
    rxBuffer = enetIfPtr->macCfgPtr->rxLargeBufferNumber *rxBufferAlign + 
        enetIfPtr->macCfgPtr->rxBufferAlignment;
    bufferPtr = mem_allocate_zero(rxBuffer);
    if(!bufferPtr)
    {
         return kStatus_ENET_MemoryAllocateFail;
    }
	bufferPtr = (uint8_t *)ENET_ALIGN((uint32_t)bufferPtr, enetIfPtr->macCfgPtr->rxBufferAlignment);
    enetIfPtr->macContextPtr->rxLargeBufferPtr = NULL;
    for(counter = 0; counter < enetIfPtr->macCfgPtr->rxLargeBufferNumber; counter++)
    {
        enet_mac_enqueue_buffer((void **)&enetIfPtr->macContextPtr->rxLargeBufferPtr, 
            bufferPtr);
        bufferPtr += rxBufferAlign;
    }

    txBufferAlign = ENET_ALIGN(enetIfPtr->maxFrameSize,
        enetIfPtr->macCfgPtr->txBufferAlignment);
    txLargeBuffer = enetIfPtr->macCfgPtr->txLargeBufferNumber * txBufferAlign;
    bufferPtr = mem_allocate_zero(txLargeBuffer);
    if(!bufferPtr)
    {
         return kStatus_ENET_MemoryAllocateFail;
    }
    bufferPtr = (uint8_t *)ENET_ALIGN((uint32_t)bufferPtr, enetIfPtr->macCfgPtr->txBufferAlignment);
    enetIfPtr->macContextPtr->txBufferPtr = NULL;
    for(counter = 0; counter < enetIfPtr->macCfgPtr->txLargeBufferNumber; counter++)
    {
        enet_mac_enqueue_buffer((void **)&enetIfPtr->macContextPtr->txBufferPtr, 
            bufferPtr);
        bufferPtr += txBufferAlign;
    }
    	
    /* Initialize transmit and receive buffer descriptor ring address*/
    enet_hal_init_bd_address(enetIfPtr->deviceNumber, 
        (uint32_t)(enetIfPtr->macContextPtr->rxBdBasePtr), 
        (uint32_t)(enetIfPtr->macContextPtr->txBdBasePtr));

    /* Initialize the receive buffer descriptor ring*/
    for( bdNumber = 0; bdNumber < enetIfPtr->macCfgPtr->rxBdNumber; bdNumber++)
    {
        if(bdNumber == enetIfPtr->macCfgPtr->rxBdNumber-1)
        {
           isLastBd = true;
        }
        bdTempPtr = enetIfPtr->macContextPtr->rxBdBasePtr + bdNumber * bdSize;
        bufferTempPtr = enet_mac_dequeue_buffer((void **)&enetIfPtr->macContextPtr->rxBufferPtr);
		if(!bufferTempPtr)
        {
            return kStatus_ENET_MemoryAllocateFail;
        }
        enet_hal_init_rxbds(bdTempPtr,bufferTempPtr,isLastBd);
    }
	
    isLastBd = false;
    /* Initialize transmit buffer descriptor ring*/
    for(bdNumber = 0; bdNumber < enetIfPtr->macCfgPtr->txBdNumber;  bdNumber++)
    {
        if(bdNumber == enetIfPtr->macCfgPtr->txBdNumber-1)
        {
            isLastBd = true;
        }
        bdTempPtr = enetIfPtr->macContextPtr->txBdBasePtr + bdNumber * bdSize;
        enet_hal_init_txbds(bdTempPtr,isLastBd);
    }
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_fifo_accelerator_init
 * Return Value: The execution status.
 * Description:Initialize the enet accelerator with the basic configuration.
 *END*********************************************************************/
uint32_t enet_mac_fifo_accelerator_init(enet_dev_if_t * enetIfPtr)
{
    enet_config_rx_fifo_t rxFifo;
    enet_config_tx_fifo_t txFifo;
    uint32_t bufferSize;	
    /* Check the input parameters*/
    if (!enetIfPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Configure tx/rx accelerator*/
    if(enetIfPtr->macCfgPtr->isRxAccelEnabled)
    {
        enet_hal_config_rx_accelerator(enetIfPtr->deviceNumber, 
           (enet_config_rx_accelerator_t *)&(enetIfPtr->macCfgPtr->rxAcceler));
        if((enetIfPtr->macCfgPtr->rxAcceler.isIpcheckEnabled) || (enetIfPtr->macCfgPtr->rxAcceler.isProtocolCheckEnabled))
        {
            rxFifo.rxFull = 0;
        }
    }
    if(enetIfPtr->macCfgPtr->isTxAccelEnabled)
    {
        enet_hal_config_tx_accelerator(enetIfPtr->deviceNumber, 
            (enet_config_tx_accelerator_t *)&(enetIfPtr->macCfgPtr->txAcceler));
        if((enetIfPtr->macCfgPtr->txAcceler.isIpCheckEnabled) || (enetIfPtr->macCfgPtr->txAcceler.isProtocolCheckEnabled))
        {
            txFifo.isStoreForwardEnabled = 1;
        }
    }
    if(enetIfPtr->macCfgPtr->isStoreAndFwEnabled)
    {
          rxFifo.rxFull = 0;
          txFifo.isStoreForwardEnabled = 1;
    }

    /* Configure tx/rx FIFO with defult value*/
    rxFifo.rxAlmostEmpty = 4;
    rxFifo.rxAlmostFull = 4;
    txFifo.txAlmostEmpty = 4;
    txFifo.txAlmostFull = 8;
    enet_hal_config_rx_fifo(enetIfPtr->deviceNumber, &rxFifo);
    enet_hal_config_tx_fifo(enetIfPtr->deviceNumber, &txFifo); 

    /* Configure receive buffer size*/
    bufferSize = ENET_ALIGN(enetIfPtr->macCfgPtr->rxBufferSize,
        enetIfPtr->macCfgPtr->rxBufferAlignment);    
    if(enetIfPtr->macCfgPtr->isVlanEnabled)
    {
        enetIfPtr->maxFrameSize = kEnetMaxFrameVlanSize;
        enet_hal_set_rx_max_size(enetIfPtr->deviceNumber, bufferSize,kEnetMaxFrameVlanSize);
    }
    else
    {   
        enetIfPtr->maxFrameSize = kEnetMaxFrameSize;
        enet_hal_set_rx_max_size(enetIfPtr->deviceNumber, bufferSize,kEnetMaxFrameSize); 
    }

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_mii_init
 * Return Value: The execution status.
 * Description:Initialize the enet mac rmii/mii interface.
 *END*********************************************************************/
uint32_t enet_mac_mii_init(enet_dev_if_t * enetIfPtr)
{
    uint32_t frequency;
	
    /* Check the input parameters*/
    if (enetIfPtr == NULL)
    {
        return kStatus_ENET_InvalidInput;
    }   

    /* Configure rmii/mii interface*/
    enet_hal_config_rmii(enetIfPtr->deviceNumber, enetIfPtr->macCfgPtr->rmiiCfgMode, 
        enetIfPtr->macCfgPtr->speed, enetIfPtr->macCfgPtr->duplex, false, 
        enetIfPtr->macCfgPtr->isLoopEnabled);

    /* Configure mii speed*/
    clock_manager_get_frequency(kSystemClock, &frequency);
    enet_hal_config_mii(enetIfPtr->deviceNumber, (frequency/(2 * enetIfPtr->macCfgPtr->miiClock) + 1), false);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_init
 * Return Value: The execution status.
 * Description:Initialize the enet device with the basic configuration
 * When enet is used, this function need to be called by the NET initialize 
 * interface.
 *END*********************************************************************/
uint32_t enet_mac_init(enet_dev_if_t * enetIfPtr)
{   
    uint32_t timeOut = 0;
    uint32_t devNumber, result = 0; 
	
    /* Check the input parameters*/
    if (enetIfPtr == NULL)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Get device number and check the parameter*/
    devNumber = enetIfPtr->deviceNumber;

    /* Store the global enet structure for isr input parameters for instance 0*/
    if(!devNumber)
    {
        enetIfHandle = enetIfPtr;
    }
	
    /* Turn on enet module clock gate */
    clock_manager_set_gate(kClockModuleENET, 0U, true);

    /* Reset enet mac*/
    enet_hal_reset_ethernet(devNumber);
    while((!enet_hal_is_reset_completed(devNumber)) && (timeOut < kEnetMaxTimeout))
    {
        time_delay(1);
        timeOut++;
    }

    /* Check out if timeout*/
    if(timeOut == kEnetMaxTimeout)
    {
        return kStatus_ENET_TimeOut;
    }
	
    /* Disable all enet mac interrupt and Clear all interrupt events*/
    enet_hal_config_interrupt(devNumber, kEnetAllInterrupt, false);
    enet_hal_clear_interrupt(devNumber, kEnetAllInterrupt);
	
    /* Program this station's physical address*/
    enet_hal_set_mac_address(devNumber, enetIfPtr->macCfgPtr->macAddr);

    /* Clear group and individul hash register*/
    enet_hal_set_group_hashtable(devNumber, 0, kEnetSpecialAddressInit);
    enet_hal_set_individual_hashtable(devNumber, 0, kEnetSpecialAddressInit);

    /* Set promiscuous */
    enet_hal_config_promiscuous(devNumber, enetIfPtr->macCfgPtr->isPromiscEnabled);
	
    /* Clear mib zero counters*/
    enet_hal_clear_mib(devNumber, true);

    /* Initizlize fifo and accelerator*/
    enet_mac_fifo_accelerator_init(enetIfPtr);
		
    /* Initialize buffer descriptor ring*/
    result = enet_mac_bd_init(enetIfPtr);
    if(result != kStatus_ENET_Success)
    {
        return result;
    }
		
    /* Initialize rmii/mii interface*/
    result = enet_mac_mii_init(enetIfPtr);
    if(result != kStatus_ENET_Success)
    {
        return result;
    }
	
    /* Initialize phy*/
    if(enetIfPtr->macCfgPtr->isPhyAutoDiscover)
    {
        ((enet_phy_api_t *)(enetIfPtr->phyApiPtr))->phy_auto_discover(enetIfPtr);
    }
    ((enet_phy_api_t *)(enetIfPtr->phyApiPtr))->phy_init(enetIfPtr);

#if FSL_FEATURE_ENET_SUPPORT_PTP
    /* Intialize ptp timer feature*/
    enet_ptp_init(enetIfPtr);
    enet_ptp_start(devNumber, enetIfPtr->macCfgPtr->isSlaveModeEnabled);
#endif	
    /* Enable ethenet rx and tx interrupt*/
    enet_hal_config_interrupt(devNumber, (kEnetTxByteInterrupt | kEnetRxFrameInterrupt), true);
    NVIC_SetPriority(enet_irq_ids[devNumber][enetIntMap[kEnetRxfInt]], ENET_RX_PRIORITY);
    NVIC_SetPriority(enet_irq_ids[devNumber][enetIntMap[kEnetTxbInt]], ENET_TX_PRIORITY);
    interrupt_enable(enet_irq_ids[devNumber][enetIntMap[kEnetRxfInt]]);
    interrupt_enable(enet_irq_ids[devNumber][enetIntMap[kEnetTxbInt]]);
    
    /* Enbale ethernet module*/
    enet_hal_config_ethernet(devNumber, true, true);

    /* Active Receive buffer descriptor must be done after module enable*/
    enet_hal_active_rxbd(enetIfPtr->deviceNumber);
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_close
 * Return Value: The execution status.
 * Description: Close enet device.
 * This function is used to shutdown ENET device.
 *END*********************************************************************/
uint32_t enet_mac_close(enet_dev_if_t * enetIfPtr)
{
    uint32_t count;
	
    /*Check input parameter*/
    if(!enetIfPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Reset enet module and disble enet module*/
    enet_hal_reset_ethernet(enetIfPtr->deviceNumber);

    /* Disable all interrupts and clear all interrupt events*/
    enet_hal_config_interrupt(enetIfPtr->deviceNumber, kEnetAllInterrupt, false);
    enet_hal_clear_interrupt(enetIfPtr->deviceNumber, kEnetAllInterrupt);
   
    /* Disable irq*/
    for(count=0; count < FSL_FEATURE_ENET_INTERRUPT_COUNT; count++)
    {
        interrupt_disable(enet_irq_ids[enetIfPtr->deviceNumber][count]);
    }
 
    /*free context*/
    mem_free(enetIfPtr->macContextPtr->rxBdBasePtr);
    mem_free(enetIfPtr->macContextPtr->rxBufferPtr);
    mem_free(enetIfPtr->macContextPtr->txBdBasePtr);
    mem_free(enetIfPtr->macContextPtr->txBufferPtr);
#if FSL_FEATURE_ENET_SUPPORT_PTP
    enet_ptp_deinit(enetIfPtr->macContextPtr);
#endif
    mem_free(enetIfPtr->macContextPtr);
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_update_rxbd
 * Return Value: The execution status.
 * Description: ENET receive buffer descriptor update.
 * This interface is used to update the used receive buffer descriptor ring to
 * make sure the used bds will be correctly used again. It will clean 
 * the status region and set the control region of the used receive buffer 
 * descriptor. If the isBufferUpdate flag is set the data buffer in the
 * buffer descriptor will be updated
 *END*********************************************************************/
uint32_t enet_mac_update_rxbd(enet_dev_if_t * enetIfPtr, bool isBufferUpdate)
{	
    uint8_t *buffer;

    while((enetIfPtr->macContextPtr->rxBdDirtyPtr != enetIfPtr->macContextPtr->rxBdCurPtr) ||
		(enetIfPtr->macContextPtr->isRxFull))
    {

        if(isBufferUpdate)
        {
            buffer = enet_mac_dequeue_buffer((void **)&enetIfPtr->macContextPtr->rxBufferPtr); 
            if(!buffer)
            {
                return kStatus_ENET_NoRxBufferLeft;
            }
            enet_hal_update_rxbds(enetIfPtr->macContextPtr->rxBdDirtyPtr, buffer, true);
        }
        else
        {
            /* the first buffer on the queue need to be covered*/
            enet_hal_update_rxbds(enetIfPtr->macContextPtr->rxBdDirtyPtr, buffer, false);
        }

        /* Increase the buffer descritpr to the next one*/
        if(enet_hal_get_rxbd_control(enetIfPtr->macContextPtr->rxBdDirtyPtr) & kEnetRxBdWrap)
        {
   	        enetIfPtr->macContextPtr->rxBdDirtyPtr = enetIfPtr->macContextPtr->rxBdBasePtr;
        }
        else
        {
   	        enetIfPtr->macContextPtr->rxBdDirtyPtr +=  enetIfPtr->macContextPtr->bufferdescSize;
        }

        enetIfPtr->macContextPtr->isRxFull = false;

        /* Active the receive buffer descriptor*/
        enet_hal_active_rxbd(enetIfPtr->deviceNumber);
    }
    return kStatus_ENET_Success;
}

#if ENET_ENABLE_DETAIL_STATS
/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_tx_error_stats
 * Return Value: .
 * Description: ENET frame receive stats process.
 * This interface is used to process packet error statistic 
 * in the last buffer descriptor of each frame.
 *END*********************************************************************/
void enet_mac_tx_error_stats(enet_dev_if_t * enetIfPtr,void *curBd)
{
    uint16_t dataCtl;

    /* Get extended control regions of the transmit buffer descriptor*/
    dataCtl = enet_hal_get_txbd_control_extend(curBd);
    if(dataCtl & kEnetTxBdTxErr)
    {

        /* Transmit error*/
        enetIfPtr->stats.statsTxError++;

        if(dataCtl & kEnetTxBdExcessCollisionErr)
        {
            /* Transmit excess collision*/
            enetIfPtr->stats.statsTxExcessCollision++;
        }
		
        if(dataCtl & kEnetTxBdLatecollisionErr)
        {   
            /* Transmit late collision*/
            enetIfPtr->stats.statsTxLateCollision++;
        }
        if(dataCtl & kEnetTxBdTxUnderFlowErr)
        {
     	    /* Transmit underflow*/
            enetIfPtr->stats.statsTxUnderFlow++;
        }
        if(dataCtl & kEnetTxBdOverFlowErr)
        {
            /* Transmit overflow*/
            enetIfPtr->stats.statsTxOverFlow++;
        }
    }
	 
    enetIfPtr->stats.statsTxTotal++;
}
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_rx_error_stats
 * Return Value: true if the frame is error else false.
 * Description: ENET frame receive stats process.
 * This interface is used to process packet statistic in the last buffer
 * descriptor of each frame.
 *END*********************************************************************/
bool enet_mac_rx_error_stats(enet_dev_if_t * enetIfPtr, uint32_t data)
{
    uint32_t status;

    /* The last bd in the frame check the stauts of the received frame*/
    status = kEnetRxBdLengthViolation | kEnetRxBdOverRun | kEnetRxBdNoOctet | kEnetRxBdCrc | kEnetRxBdCollision;
    if(data & status)
    {
#if ENET_ENABLE_DETAIL_STATS         
        /* Discard error packets*/
        enetIfPtr->stats.statsRxError++;
        enetIfPtr->stats.statsRxDiscard++;

        /* Receive error*/
        if((data & kEnetRxBdOverRun) != 0)
        {
            /* Receive over run*/
            enetIfPtr->stats.statsRxOverRun++;
        }
	    else if((data & kEnetRxBdLengthViolation) != 0)
        {
            /* Receive length greater than max frame*/
            enetIfPtr->stats.statsRxLengthGreater++;
        }
        else if( (data & kEnetRxBdNoOctet) != 0)
        {
            /* Receive non-octet aligned frame*/
            enetIfPtr->stats.statsRxAlign++;
    	}
    	else if((data & kEnetRxBdCrc) != 0)
        {
            /* Receive crc error*/
            enetIfPtr->stats.statsRxFcs++;
        }
        else if( (data & kEnetRxBdCollision) != 0)
        { 
            /* late collision frame discard*/
            enetIfPtr->stats.statsRxCollision++;
        }
#endif
        return true;
    }
    else
    {
#if ENET_ENABLE_DETAIL_STATS 
        /* Add the right packets*/            
        enetIfPtr->stats.statsRxTotal++;
#endif
        return false;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_receive
 * Return Value: The execution status.
 * Description: ENET frame receive function.
 * This interface receive the frame from ENET deviece and returns the address 
 * of the received data.
 *END*********************************************************************/
uint32_t enet_mac_receive(enet_dev_if_t * enetIfPtr, enet_mac_packet_buffer_t *packBuffer)
{
    void *curBd;
    bool isLastFrame = true;
    uint8_t packetFrag = 0;
    uint16_t totalLen = 0;
    uint32_t controlStatus;
   
    /* Check input parameters*/
    if((!enetIfPtr->macContextPtr) || (!packBuffer))
    {
        return kStatus_ENET_InvalidInput;
    }

	/* Check if the bd is full*/
	if(!enetIfPtr->macContextPtr->isRxFull)
	{
        /* Check the current buffer descriptor address*/
        curBd = enetIfPtr->macContextPtr->rxBdCurPtr;
        controlStatus = enet_hal_get_rxbd_control(curBd);
        if((controlStatus & kEnetRxBdEmpty) != 0)
        {
            return kStatus_ENET_RxbdEmpty;
        }
    
    	/* Increase current buffer descriptor to the next one*/
        if(enet_hal_get_rxbd_control(curBd) & kEnetRxBdWrap)
        {
            enetIfPtr->macContextPtr->rxBdCurPtr = enetIfPtr->macContextPtr->rxBdBasePtr;
        }
        else
        {
            enetIfPtr->macContextPtr->rxBdCurPtr +=  enetIfPtr->macContextPtr->bufferdescSize;
        }
    
        if(enetIfPtr->macContextPtr->rxBdCurPtr == enetIfPtr->macContextPtr->rxBdDirtyPtr)
        {
            enetIfPtr->macContextPtr->isRxFull = true;
        }
    	
        /* Discard packets with truncate error*/
        if((controlStatus & kEnetRxBdTrunc) != 0 )
        {
#if ENET_ENABLE_DETAIL_STATS 
            enetIfPtr->stats.statsRxTruncate++;
            enetIfPtr->stats.statsRxDiscard++;
#endif
            enet_mac_update_rxbd(enetIfPtr, false);
            return kStatus_ENET_RxbdTrunc;
        }
    	
        if((controlStatus & kEnetRxBdLast) != 0)
        {
            /*This is valid frame */
            isLastFrame = true;
    		
            /* The last bd in the frame check the stauts of the received frame*/
            if(!enet_mac_rx_error_stats(enetIfPtr, controlStatus))
            {
                packBuffer[0].data = enet_hal_get_bd_buffer(curBd);
                packBuffer[0].length = enet_hal_get_bd_length(curBd) - kEnetFrameFcsLen;
                packBuffer[1].length = 0;
#if FSL_FEATURE_ENET_SUPPORT_PTP
                enet_ptp_store_rx_timestamp(enetIfPtr->macContextPtr->privatePtpPtr, 
                    packBuffer[0].data, curBd);
#endif
                /* Update receive buffer descriptor*/
                enet_mac_update_rxbd(enetIfPtr, true);
                return kStatus_ENET_Success;
            }
            else
            {
                enet_mac_update_rxbd(enetIfPtr, false);
                return kStatus_ENET_RxbdError;
            }
        }
    	else
        {
            /* Store the fragments of a frame on serveral buffer descriptors*/
            isLastFrame = false;
            packBuffer[packetFrag].data = enet_hal_get_bd_buffer(curBd);
            packBuffer[packetFrag].length = enetIfPtr->macContextPtr->rxBufferSizeAligned;
    		totalLen = packBuffer[packetFrag].length;
            packetFrag ++;
        }
	}
	else
	{
#if ENET_ENABLE_DETAIL_STATS 
        enetIfPtr->stats.statsRxDiscard++;
#endif
	    enet_mac_update_rxbd(enetIfPtr, false);
        return kStatus_ENET_RxBdFull;
	}

    /*process the frame stored on serveral bds*/
    while(!isLastFrame)
    {
        if(!enetIfPtr->macContextPtr->isRxFull)
        {
    		/* Get the current buffer descriptor address*/
            curBd = enetIfPtr->macContextPtr->rxBdCurPtr;
            controlStatus = enet_hal_get_rxbd_control(curBd);
            if((controlStatus & kEnetRxBdEmpty) != 0)
            {
                return kStatus_ENET_RxbdEmpty;
            }
    
    	    /* Increase current buffer descriptor to the next one*/
    	    if(enet_hal_get_rxbd_control(curBd) & kEnetRxBdWrap)
    	    {
                enetIfPtr->macContextPtr->rxBdCurPtr = enetIfPtr->macContextPtr->rxBdBasePtr;
    	    }
    	    else
    	    {
                enetIfPtr->macContextPtr->rxBdCurPtr +=  enetIfPtr->macContextPtr->bufferdescSize;
    	    }
			
            if(enetIfPtr->macContextPtr->rxBdCurPtr == enetIfPtr->macContextPtr->rxBdDirtyPtr)
            {
                enetIfPtr->macContextPtr->isRxFull = true;
            }
	
            /* Discard packets with truncate error*/
            if((controlStatus & kEnetRxBdTrunc) != 0)
            {
#if ENET_ENABLE_DETAIL_STATS 
                enetIfPtr->stats.statsRxTruncate++;
                enetIfPtr->stats.statsRxDiscard++;
#endif
                enet_mac_update_rxbd(enetIfPtr, false);
                return kStatus_ENET_RxbdTrunc;
            }
    
            if((controlStatus & kEnetRxBdLast) != 0)
            {
                /*This is the last bd in a frame*/     
                isLastFrame = true;
    
                /* The last bd in the frame check the stauts of the received frame*/
                if(enet_mac_rx_error_stats(enetIfPtr, controlStatus))
                {
                    enet_mac_update_rxbd(enetIfPtr, false);
                    return kStatus_ENET_RxbdError;
                }
                else
                {
                    packBuffer[packetFrag].data = enet_hal_get_bd_buffer(curBd);
                    packBuffer[packetFrag].length = enet_hal_get_bd_length(curBd) - 
    				    (kEnetFrameFcsLen + totalLen);
                    packBuffer[packetFrag + 1].length = 0;
                    totalLen = 0;
                    /* Delivery the last part data to the packet*/				
#if FSL_FEATURE_ENET_SUPPORT_PTP
                    enet_ptp_store_rx_timestamp(enetIfPtr->macContextPtr->privatePtpPtr, 
                        packBuffer[0].data, curBd);
#endif
                    /* Update receive buffer descriptor*/
                    enet_mac_update_rxbd(enetIfPtr, true);
                    return kStatus_ENET_Success;
                }
            }
            else
            {
                isLastFrame = false;
                packBuffer[packetFrag].data = enet_hal_get_bd_buffer(curBd);
                packBuffer[packetFrag].length = enetIfPtr->macContextPtr->rxBufferSizeAligned;
                totalLen += packBuffer[packetFrag].length; 
                packetFrag ++;
                /* Check a frame with total bd numbers */
                if(packetFrag == kEnetMaxFrameBdNumbers - 1)
                {
                    return kStatus_ENET_SmallBdSize;
                }
            }  
        }
        else
        {
#if ENET_ENABLE_DETAIL_STATS 
            enetIfPtr->stats.statsRxDiscard++;
#endif
            enet_mac_update_rxbd(enetIfPtr, false);
            return kStatus_ENET_RxBdFull;
        }
    }

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_send
 * Return Value: The execution status.
 * Description: ENET frame send function.
 * This interface send the frame to enet device.
 *END*********************************************************************/
uint32_t enet_mac_send(enet_dev_if_t * enetIfPtr, uint8_t *packet, uint32_t size)
{
    void *curBd;

    bool isPtpMsg = false;

    /* Check input parameters*/
    if((!enetIfPtr) || (!packet))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Get the current buffer descriptor address*/
    curBd = enetIfPtr->macContextPtr->txBdCurPtr;
    if(!curBd)
    {
        return kStatus_ENET_RxbdInvalid;
    }

#if FSL_FEATURE_ENET_SUPPORT_PTP
    /* Check if this is ptp message*/
    enet_ptp_quick_parse(packet, &isPtpMsg);
#endif

    /* Packet the transmit frame to the buffer descriptor*/
    enet_hal_update_txbds(curBd, packet, size, isPtpMsg);

    /* Upate the buffer address*/
    if(enet_hal_get_txbd_control(enetIfPtr->macContextPtr->txBdCurPtr) & kEnetTxBdWrap)
    {
        enetIfPtr->macContextPtr->txBdCurPtr = enetIfPtr->macContextPtr->txBdBasePtr;
    }
    else
    {
        enetIfPtr->macContextPtr->txBdCurPtr +=  enetIfPtr->macContextPtr->bufferdescSize;
    }

    /* Return if the transmit buffer ring is full*/
    if(enetIfPtr->macContextPtr->txBdCurPtr == enetIfPtr->macContextPtr->txBdDirtyPtr)
    {
         enet_hal_active_txbd(enetIfPtr->deviceNumber);
         enetIfPtr->macContextPtr->isTxFull = true;
    	 return kStatus_ENET_TxbdFull;
    }
	
    enetIfPtr->macContextPtr->isTxFull = false;

    /* Active the receive buffer descriptor*/
    enet_hal_active_txbd(enetIfPtr->deviceNumber);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_tx_cleanup
 * Return Value: The execution status.
 * Description: First, store transmit frame error statistic and ptp timestamp 
 * of transmitted packets. second, cleanup the used transmit buffer descriptors.
 * If the ptp 1588 feature is open, this interface will do capture 1588 timestamp. 
 * It is called by transmit interrupt handler.
 *END*********************************************************************/
uint32_t enet_mac_tx_cleanup(enet_dev_if_t * enetIfPtr)
{
    uint8_t *curBd, *packet;
    uint16_t dataCtl;

    /* Check if the packet is ok*/
    curBd = enetIfPtr->macContextPtr->txBdDirtyPtr;
    if(!curBd)
    {
        return kStatus_ENET_TxbdNull;
    }

    packet = enet_hal_get_bd_buffer(curBd);

    while( packet != NULL)
    {
        /* Get the control status data*/
        dataCtl = enet_hal_get_txbd_control(curBd);

        /* If the buffer descriptor has not been processed just break out*/
        if(dataCtl & kEnetTxBdReady)
        {
            break;
        }
      
        /* If the transmit buffer descriptor is full, just break out*/
        if((curBd == enetIfPtr->macContextPtr->txBdCurPtr) && 
                (enetIfPtr->macContextPtr->isTxFull))
        {
            /* Active the receive buffer descriptor*/
            enet_hal_active_txbd(enetIfPtr->deviceNumber);
            return kStatus_ENET_TxbdFull;
        }

        /* If the transmit buffer descriptor is ready, store packet statistic*/
        if( dataCtl & kEnetTxBdLast)
        {
#if ENET_ENABLE_DETAIL_STATS
            enet_mac_tx_error_stats(enetIfPtr,curBd);
#endif
#if FSL_FEATURE_ENET_SUPPORT_PTP
            /* Do ptp timestamp store*/
            if(enet_hal_get_txbd_timestamp_flag(curBd))
            {
                enet_ptp_store_tx_timestamp(enetIfPtr->macContextPtr->privatePtpPtr, 
                    curBd);
            }
#endif
            /* Enqueue buffer to the buffer queue*/  
            enet_mac_enqueue_buffer((void **)&enetIfPtr->macContextPtr->txBufferPtr, 
                packet);

            /* Clear the buffer descriptor buffer address*/
            enet_hal_clear_txbds(curBd);
        }

        /* Upate the buffer address*/
        if(dataCtl & kEnetTxBdWrap)
        {
            enetIfPtr->macContextPtr->txBdDirtyPtr = enetIfPtr->macContextPtr->txBdBasePtr;
        }
        else
        {
            enetIfPtr->macContextPtr->txBdDirtyPtr += enetIfPtr->macContextPtr->bufferdescSize;
        }

        curBd = enetIfPtr->macContextPtr->txBdDirtyPtr;
        if(curBd == NULL)
        {
            /* Active the receive buffer descriptor*/
            enet_hal_active_txbd(enetIfPtr->deviceNumber);
            return kStatus_ENET_TxbdNull;
        }
		
        packet = enet_hal_get_bd_buffer(curBd);
    }
    
    /* Active the receive buffer descriptor*/
    enet_hal_active_txbd(enetIfPtr->deviceNumber);

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_rx_isr
 * Description: ENET receive isr.
 * This interface is the receive interrupt handler.
 *END*********************************************************************/
void enet_mac_rx_isr(void *enetIfPtr)
{
    event_group_t flag = 0x1;

    /*Check input parameter*/
    if(!enetIfPtr)
    {
        return;
    }
    /* Get interrupt status.*/
    while(enet_hal_get_interrupt_status(((enet_dev_if_t *)enetIfPtr)->deviceNumber, (kEnetRxFrameInterrupt | kEnetRxByteInterrupt)))
    {
        /*Clear interrupt*/
        enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, 
            (kEnetRxFrameInterrupt | kEnetRxByteInterrupt));
        /* Release sync signal*/
        event_set(&((enet_dev_if_t *)enetIfPtr)->enetReceiveSync, flag);
    }
    
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_tx_isr
 * Description: ENET transmit isr.
 * This interface is the transmit interrupt handler.
 *END*********************************************************************/
void enet_mac_tx_isr(void *enetIfPtr)
{
	
    /*Check input parameter*/
    if(!enetIfPtr)
    {
        return;
    }
    /* Get interrupt status.*/
    while(enet_hal_get_interrupt_status(((enet_dev_if_t *)enetIfPtr)->deviceNumber, 
              (kEnetTxFrameInterrupt | kEnetTxByteInterrupt)))
    {
        /*Clear interrupt*/
        enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, 
            (kEnetTxFrameInterrupt | kEnetTxByteInterrupt));

        /*Cleanup the transmit buffers*/
        enet_mac_tx_cleanup((enet_dev_if_t *)enetIfPtr);
    }   
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_calculate_crc32
 * Description: Calculate crc-32.
 * This function is called by the enet_mac_add_multicast_group and 
 * enet_mac_leave_multicast_group.
 *END*********************************************************************/
void enet_mac_calculate_crc32(enetMacAddr address, uint32_t *crcValue)
{
    uint32_t crc = kEnetCrcData, count1,count2;	

    /* Calculate the CRC-32 polynomial on the multicast group address*/
    for(count1 = 0; count1 < kEnetMacAddrLen; count1++)
    {
        uint8_t c = address[count1];
        for(count2 = 0; count2 < kEnetCrcOffset; count2++)
        {
            if((c ^ crc)& 1U)
            {
                crc >>= 1U;
                c >>= 1U;
                crc ^= 0xEDB88320L;
            }
            else
            {
               crc >>= 1U;
               c >>= 1U;
            }
        }
    }
	
    *crcValue = crc;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_add_multicast_group
 * Return Value: The execution status.
 * Description: ADD enet to the specific multicast group.
 * This function is used to add enet device to specific multicast
 * group and it is called by the upper TCP/IP stack.
 *END*********************************************************************/
 uint32_t enet_mac_add_multicast_group(uint32_t instance, enet_multicast_group_t *multiGroupPtr, enetMacAddr address)
{
    uint32_t crcValue;
	
    /* Check input parameters*/
    if((!multiGroupPtr) || (!address))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Calculate the CRC-32 polynomial on the multicast group address*/
    enet_mac_calculate_crc32(address, &crcValue);

    /* Set the hash table*/
    enet_hal_set_group_hashtable(instance, crcValue, kEnetSpecialAddressEnable);

    /* Store the hash value in the right address stucture*/
    multiGroupPtr->hash = (crcValue >>= 26U) & kEnetCrcMask1;
	
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_leave_multicast_group
 * Return Value: The execution status.
 * Description: ENET Leave specific multicast group.
 * This function is used to remove enet device from specific multicast
 * group and it is called by the upper TCP/IP stack.
 *END*********************************************************************/
 uint32_t enet_mac_leave_multicast_group(uint32_t instance, enet_multicast_group_t *multiGroupPtr, enetMacAddr address)
{
    uint32_t crcValue;
	
    /* Check input parameters*/
    if((!multiGroupPtr))
    {
        return kStatus_ENET_InvalidInput;
    }
	
    /* Calculate the CRC-32 polynomial on the multicast group address*/
    enet_mac_calculate_crc32(address, &crcValue);

    /* Set the hash table*/
    enet_hal_set_group_hashtable(instance,crcValue, kEnetSpecialAddressDisable);
    
    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_enqueue_buffer
 * Return Value: 
 * Description: ENET mac enqueue buffers.
 * This function is used to enqueue buffers to buffer queue.
 *END*********************************************************************/
void enet_mac_enqueue_buffer( void **queue, void *buffer)
{
    *((void **)buffer) = *queue;
    *queue = buffer;
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_mac_dequeue_buffer
 * Return Value: 
 * Description: ENET mac dequeue buffers.
 * This function is used to dequeue a buffer from buffer queue.
 *END*********************************************************************/
void *enet_mac_dequeue_buffer( void **queue)
{
    void *buffer = *queue;

    if (buffer) 
    {
        *queue = *((void **)buffer);
    }

    return buffer;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

