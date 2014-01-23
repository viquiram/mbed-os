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
#include "fsl_phy_driver.h"
#include "fsl_enet_rtcs_adapter.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include <string.h>
/*******************************************************************************
 * Variables
 ******************************************************************************/
static enet_dev_if_t enetDevIf[HW_ENET_INSTANCE_COUNT];
enet_mac_config_t g_enetMacCfg[HW_ENET_INSTANCE_COUNT] = 
{
{
    kEnetMaxFrameSize,  /*!< ENET receive buffer size*/
    ENET_RX_LARGE_BUFFER_NUM,  /*!< ENET receive large buffer number*/
    ENET_RX_BUFFER_ALIGNMENT,  /*!< ENET receive buffer alignment*/
    ENET_TX_BUFFER_ALIGNMENT,  /*!< ENET transmit buffer alignment*/
    ENET_RX_RING_LEN,        /*!< ENET receive bd number*/
    ENET_TX_RING_LEN,        /*!< ENET transmit bd number*/
    ENET_BD_ALIGNMENT,         /*!< ENET bd alignment*/
    {0},                /*!< ENET mac address*/
    kEnetCfgRmii,       /*!< ENET rmii interface*/
    kEnetCfgSpeed100M,  /*!< ENET rmii 100M*/
    kEnetCfgFullDuplex, /*!< ENET rmii Full- duplex*/
    false,              /*!< ENET no loop*/
    false,              /*!< ENET enable promiscuous*/
    false,              /*!< ENET txaccelerator enabled*/
    false,              /*!< ENET rxaccelerator enabled*/
    false,              /*!< ENET store and forward*/
    {true, true, true, false, false},  /*!< ENET rxaccelerator config*/
    {true, true, false},          /*!< ENET txaccelerator config*/
    false,              /*!< vlan frame support*/
    true,               /*!< PHY auto discover*/
    ENET_MII_CLOCK,     /*!< ENET MDC clock*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    ENET_PTP_RING_BUFFER_NUM,   /*!< ptp ring buffer number */
    false,
#endif
},
};

enet_phy_config_t g_enetPhyCfg[HW_ENET_INSTANCE_COUNT] =
{{0, false }};

/* ENET error describe list*/
static const char * ENET_errlist[ENETERR_MAX - ENETERR_MIN + 1] = {
    "Invalid device number",                         /* ENETERR_INVALID_DEVICE     */
    "Device already initialized",                    /* ENETERR_INIT_DEVICE        */
    "Couldn't allocate state (out of memory)",       /* ENETERR_ALLOC_CFG          */
    "Couldn't allocate PCBs (out of memory)",        /* ENETERR_ALLOC_PCB          */
    "Couldn't allocate buffer descriptors",          /* ENETERR_ALLOC_BD           */
    "Couldn't install Ethernet notifier",            /* ENETERR_INSTALL_ISR        */
    "Some PCBs are still in use",                    /* ENETERR_FREE_PCB           */
    "Out of memory",                                 /* ENETERR_ALLOC_ECB          */
    "Protocol already open",                         /* ENETERR_OPEN_PROT          */
    "Not an open protocol",                          /* ENETERR_CLOSE_PROT         */
    "Packet too short",                              /* ENETERR_SEND_SHORT         */
    "Packet too long",                               /* ENETERR_SEND_LONG          */
    "Not a multicast address",                       /* ENETERR_JOIN_MULTICAST     */
    "Out of memory",                                 /* ENETERR_ALLOC_MCB          */
    "Not a joined group",                            /* ENETERR_LEAVE_GROUP        */
    "Transmit ring full",                             /* ENETERR_SEND_FULL          */
    "IP Table full of IP pairs",                     /* ENETERR_IP_TABLE_FULL      */
    "Generic alloc failed",                          /* ENETERR_ALLOC              */
    "Device failed to initialize",                   /* ENETERR_INIT_FAILED        */
    "Device read or write timeout",                  /* ENETERR_DEVICE_TIMEOUT     */
    "Couldn't allocate buffers (out of memory)",     /* ENETERR_ALLOC_BUFFERS      */
    "Couldn't allocate MAC context (out of memory)", /* ENETERR_ALLOC_MAC_CONTEXT  */
    "Couldn't allocate TX buffer (out of memory)",   /* ENETERR_NO_TX_BUFFER       */
    "Invalid initialization parameter",              /* ENETERR_INVALID_INIT_PARAM */
    "Shutdown failed, device in use",                /* ENETERR_DEVICE_IN_USE      */
    "Device already initialized",                    /* ENETERR_INITIALIZED_DEVICE */
    "Setting of ESSID in progress",                  /* ENETERR_INPROGRESS         */
    "1588 driver lwevent creation failed",           /* ENETERR_1588_LWEVENT       */
    "Invalid mode for this ethernet driver",         /* ENETERR_INVALID_MODE       */
    "Invalid option for this ethernet driver"        /* ENETERR_INVALID_OPTION     */
};

pcb_queue  packbuffer[HW_ENET_INSTANCE_COUNT];
bool frameIsCollected = false;
#if !ENET_RECEIVE_ALL_INTERRUPT
    task_handler_t revHandle;
#endif
#if BSPCFG_ENABLE_ENET_STATS
ENET_STATS enetStats;
#endif
FSL_RTOS_TASK_DEFINE(ENET_receive, ENET_TASK_STACK_SIZE, "receive", false);

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void IPE_recv_IP(PCB_PTR pcb, void *handle);
extern void IPE_recv_ARP(PCB_PTR pcb, void *handle);
#if RTCSCFG_ENABLE_IP6 
extern void IP6E_recv_IP(PCB_PTR pcb, void *handle);
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_free
 * Description: ENET packet buffer free. 
 *
 *END*********************************************************************/
static void ENET_free(PCB_PTR packet)
{
    enet_dev_if_t *param;
    uint8_t count;

    /* Check input Parameter*/
    if (packet == NULL)
    {
        return;
    }

    param = (enet_dev_if_t *)packet->PRIVATE;
    count = param->deviceNumber;
    if (packet->FRAG[0].FRAGMENT != NULL)
    {
        *(uint32_t *)packet->FRAG[0].FRAGMENT = 0;
        if (frameIsCollected)
        {
            enet_mac_enqueue_buffer((void **)&param->macContextPtr->rxLargeBufferPtr, packet->FRAG[0].FRAGMENT);

        }
#if !ENET_RECEIVE_ALL_INTERRUPT	
        else
        {  
            enet_mac_enqueue_buffer((void **)&param->macContextPtr->rxBufferPtr, 
                packet->FRAG[0].FRAGMENT);
        }
#endif        
        /* Clear fragment in the packet*/
        packet->FRAG[0].FRAGMENT = NULL;
    }
		
    /* Add the used PCB buffer into the PCB buffer queue*/
    QUEUEADD(packbuffer[count].pcbHead, packbuffer[count].pcbTail, packet);

}

#if ENET_RECEIVE_ALL_INTERRUPT
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_find_receiver
 * Description: Find ENET tcp/ip upper layer functions. 
 *
 *END*********************************************************************/
uint32_t ENET_find_receiver(void *enetPtr, enet_mac_packet_buffer_t *packetBuffer)
{
    uint8_t *packet, counter;
    uint16_t type, length = 0;
    PCB *pcbPtr;
    uint16_t *typePtr;
    ENET_ECB_STRUCT *ecbPtr;
    bool packetInvalid = ENET_ERROR;
    enet_dev_if_t *enetDevifPtr = (enet_dev_if_t *)enetPtr;

    /* Collect the frame first*/
    if(!packetBuffer[1].length)
    {
        packet = packetBuffer[0].data;  /* the frame with only one bd */
        length = packetBuffer[0].length;
        frameIsCollected = false;
    }
    else
    {
        /* Dequeue a large  buffer */
        packet = enet_mac_dequeue_buffer((void **)&enetDevifPtr->macContextPtr->rxLargeBufferPtr);
        if(packet!=NULL)
        {
            for(counter = 0; counter < kEnetMaxFrameBdNumbers ; counter ++)
            {
                if(packetBuffer[counter].length != 0)
                {
                    memcpy(packet + length, packetBuffer[counter].data, packetBuffer[counter].length);
                    length += packetBuffer[counter].length; 
	            }
   
            }
        }
        else
        {
#if ENET_ENABLE_DETAIL_STATS 
            enetDevifPtr->stats.statsRxDiscard ++;
#endif
            return kStatus_ENET_LargeBufferFull;
        }
        frameIsCollected = true;
    }
	
    /* Process the received frame*/
    typePtr = &((enet_ethernet_header_t *)packet)->type;
    type = NTOHS((*typePtr));
    if(type == ENETPROT_8021Q) 
    {
        typePtr = &((enet_8021vlan_header_t *)packet)->type;
        type = NTOHS((*typePtr));
    }
    if(type <= kEnetMaxFrameDateSize)
    {
        enet_8022_header_ptr llcPtr = (enet_8022_header_ptr)(typePtr + 2);
        type = HTONS(llcPtr->type);
    }

    for(ecbPtr = (ENET_ECB_STRUCT *)enetDevifPtr->enetNetifService; ecbPtr; ecbPtr = ecbPtr->NEXT)
    {
        if(ecbPtr->TYPE == type)
        {
            packetInvalid = ENET_OK;
            /* Collect frame to PCB structure for upper layer process*/
            QUEUEGET(packbuffer[enetDevifPtr->deviceNumber].pcbHead,packbuffer[enetDevifPtr->deviceNumber].pcbTail, pcbPtr);
            if(pcbPtr)
            {
                pcbPtr->FRAG[0].LENGTH = length;
                pcbPtr->FRAG[0].FRAGMENT = packet;
                pcbPtr->FRAG[1].LENGTH = 0;
                pcbPtr->PRIVATE = enetDevifPtr;
                ecbPtr->SERVICE(pcbPtr, ecbPtr->PRIVATE);
           }
        }
    }

    return packetInvalid;
}
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_initialize
 * Return Value: The execution status.
 * Description:Initialize the ENET device. 
 *
 *END*********************************************************************/
uint32_t ENET_initialize(uint32_t device, _enet_address address,uint32_t flag, _enet_handle *handle)
{
    enet_dev_if_t * enetIfPtr;
    uint32_t result;
    uint8_t count;
    PCB2 *pcbbuffer;

    if (device > HW_ENET_INSTANCE_COUNT)
    {
        return ENETERR_INVALID_INIT_PARAM;
    }
	
    /* Check the device status*/
    if (enetDevIf[device].isInitialized)
    {
        return ENETERR_INITIALIZED_DEVICE;
    }

    /* Initialize device*/
    enetIfPtr = (enet_dev_if_t *)&enetDevIf[device];
    if (HW_ENET_INSTANCE_COUNT == device)
    {
        enetIfPtr->next = NULL;
    }   

    enetIfPtr->deviceNumber = device;
    enetIfPtr->macCfgPtr = &g_enetMacCfg[device];
    enetIfPtr->phyCfgPtr = &g_enetPhyCfg[device];
    enetIfPtr->macApiPtr = &g_enetMacApi;
    enetIfPtr->phyApiPtr = (void *)&g_enetPhyApi;
#if ENET_RECEIVE_ALL_INTERRUPT
    enetIfPtr->enetNetifcall = ENET_find_receiver;
#endif
    memcpy(enetIfPtr->macCfgPtr->macAddr, address, kEnetMacAddrLen);
	
    /* Initialize PCB buffer*/
    pcbbuffer = (PCB2 *)mem_allocate_zero(PCB_MINIMUM_SIZE * ENET_PCB_NUM);
    for (count = 0; count < ENET_PCB_NUM; count++)
    {
        QUEUEADD(packbuffer[device].pcbHead, packbuffer[device].pcbTail, (PCB *)pcbbuffer);        
        pcbbuffer->FRAG[1].LENGTH = 0;
        pcbbuffer->FRAG[1].FRAGMENT = NULL;
        pcbbuffer->FREE = ENET_free;
        pcbbuffer ++;
    }

    /* Create sync signal*/
    lock_create(&enetIfPtr->enetContextSync);	
	
#if !ENET_RECEIVE_ALL_INTERRUPT
    event_create(&enetIfPtr->enetReceiveSync, kEventAutoClr);	
    /* Create receive task*/
    task_create(ENET_receive, ENET_RECEIVE_TASK_PRIO, enetIfPtr, &revHandle);
#endif	
    /* Initialize ENET device*/
    result = enetIfPtr->macApiPtr->enet_mac_init(enetIfPtr);
    if (result == kStatus_ENET_Success)
    {
        *handle = enetIfPtr;
        enetIfPtr->isInitialized = true;
        return ENET_OK;
    }
    else
    {

        lock_destroy(&enetIfPtr->enetContextSync);
#if !ENET_RECEIVE_ALL_INTERRUPT
        task_destroy(revHandle);
        event_destroy(&enetIfPtr->enetReceiveSync);
#endif
        *handle = NULL;
        return ENET_ERROR;
    }
}
#if !ENET_RECEIVE_ALL_INTERRUPT
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_open
 * Return Value: The execution status.
 * Description: Open the ENET device, This interface is used to add the private
 * address to the ENET device structure. 
 *
 *END*********************************************************************/
uint32_t ENET_open(_enet_handle  handle, uint16_t type, void (* service)(PCB_PTR, void *), void *private)
{
    enet_dev_if_t * enetIfPtr;

    /* Check input parameter*/
    if ((!handle) || (!private))
    {
        return ENETERR_INVALID_DEVICE;
    }
   
    enetIfPtr = (enet_dev_if_t *)handle;
    lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);
    /*add the upper layer netiIF structure into the device structure*/
    enetIfPtr->netIfPtr = private;
    lock_release(&enetIfPtr->enetContextSync);

    return ENET_OK;
}
#else
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_open
 * Return Value: The execution status.
 * Description: Open the ENET device, This interface is used to add the private
 * address to the enet device structure. 
 *
 *END*********************************************************************/
uint32_t ENET_open(_enet_handle  handle, uint16_t type, void (* service)(PCB_PTR, void *), void *private)
{
    enet_dev_if_t * enetIfPtr;
    ENET_ECB_STRUCT_PTR ecbPtr, *searchPtr;
	
    /* Check input parameter*/
    if ((!handle) || (!private))
    {
        return ENETERR_INVALID_DEVICE;
    }
   
    enetIfPtr = (enet_dev_if_t *)handle;
    lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);
    for (searchPtr = (ENET_ECB_STRUCT_PTR *)(&enetIfPtr->enetNetifService); *searchPtr; searchPtr = &(*searchPtr)->NEXT)
    {
        if ((*searchPtr)->TYPE == type)
        {
            lock_release(&enetIfPtr->enetContextSync);
            return ENETERR_OPEN_PROT;
        }
    }

    ecbPtr = (ENET_ECB_STRUCT_PTR)mem_allocate_zero(sizeof(ENET_ECB_STRUCT));
    if (!ecbPtr)
    {
        lock_release(&enetIfPtr->enetContextSync);
        return ENETERR_ALLOC_ECB;
    }

    ecbPtr->TYPE = type;
    ecbPtr->SERVICE = service;
    ecbPtr->PRIVATE = private;
    ecbPtr->NEXT = NULL;
    *searchPtr = ecbPtr;

    lock_release(&enetIfPtr->enetContextSync);

    return ENET_OK;
}
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_shutdown
 * Return Value: The execution status.
 * Description: Stop the ENET device. 
 *
 *END*********************************************************************/
uint32_t ENET_shutdown(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr;
    uint32_t result;
   
    /* Check the input parameter*/
    if (!handle)
    {
        return ENETERR_INVALID_DEVICE;
    }

    enetIfPtr = (enet_dev_if_t *)handle;
	
#if ENET_RECEIVE_ALL_INTERRUPT
    /* Make sure upper layers have closed the device*/
    if (enetIfPtr->enetNetifService)
    {
        return ENETERR_DEVICE_IN_USE;
    }
#endif
    /* Close the ENET device*/
    result = enetIfPtr->macApiPtr->enet_mac_close(enetIfPtr);
    if (result == kStatus_ENET_Success)
    {
        lock_destroy(&enetIfPtr->enetContextSync);
#if !ENET_RECEIVE_ALL_INTERRUPT
        task_destroy(revHandle);
        event_destroy(&enetIfPtr->enetReceiveSync);
#endif
        mem_free((void *)enetIfPtr);
        return ENET_ERROR;
    }

    return ENET_OK;
}

#if !ENET_RECEIVE_ALL_INTERRUPT
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_receive
 * Description:  Net Receive interface. 
 *
 *END*********************************************************************/
static void ENET_receive(void *param) 
{
    uint8_t *packet;
    uint16_t length = 0, type, counter = 0;
    uint32_t result;
    PCB *pcbPtr;
    uint16_t *typePtr;
    event_group_t flag = 0x1;
    enet_mac_packet_buffer_t packetBuffer[kEnetMaxFrameBdNumbers];

    /* Check input parameter*/
    if (!param)
    {
        return ;
    }
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)param;
   
    while(1)
    {
        memset(&packetBuffer[0], 0, kEnetMaxFrameBdNumbers * sizeof(enet_mac_packet_buffer_t));   
        /* Receive frame*/
        result = enetIfPtr->macApiPtr->enet_mac_receive(enetIfPtr, &packetBuffer[0]);
        if ((result == kStatus_ENET_RxbdEmpty) || (result == kStatus_ENET_InvalidInput))
        { 
            event_wait(&enetIfPtr->enetReceiveSync, kSyncWaitForever, &flag);
        }

        /* Process with the right packets*/
        if (packetBuffer[0].data != NULL)
        {
            /* Collect the frame first*/
            if (!packetBuffer[1].length)
            {
                packet = packetBuffer[0].data;  /* the frame with only one bd */
                length = packetBuffer[0].length;
                frameIsCollected = false;
            }
            else
           {
                /* Dequeue a large  buffer */
                packet = enet_mac_dequeue_buffer((void **)&enetIfPtr->macContextPtr->rxLargeBufferPtr);
                if (packet!=NULL)
                {
                    length = 0;
                    for (counter = 0; counter < kEnetMaxFrameBdNumbers ; counter ++)
                    {
                        if (packetBuffer[counter].length != 0)
                        {
                            memcpy(packet + length, packetBuffer[counter].data, packetBuffer[counter].length);
                            length += packetBuffer[counter].length; 

                            *(uint32_t *)packetBuffer[counter].data = 0;
                            enet_mac_enqueue_buffer((void **)&enetIfPtr->macContextPtr->rxBufferPtr, packetBuffer[counter].data);
                        }
			
                    }

                }
                else
                {
#if ENET_ENABLE_DETAIL_STATS 
                    enetIfPtr->stats.statsRxMissed++;
#endif
                }
                frameIsCollected = true;
        }
    
            /* Process the received frame*/
            typePtr = &((enet_ethernet_header_t *)packet)->type;
            type = NTOHS((*typePtr));
            if (type == ENETPROT_8021Q) 
            {
                typePtr = &((enet_8021vlan_header_t *)packet)->type;
                type = NTOHS((*typePtr));
            }
            if (type <= kEnetMaxFrameDateSize)
            {
                enet_8022_header_ptr llcPtr = (enet_8022_header_ptr)(typePtr + 2);
                type = HTONS(llcPtr->type);
            }
    
            /* Collect frame to PCB structure for upper layer process*/
            QUEUEGET(packbuffer[enetIfPtr->deviceNumber].pcbHead, packbuffer[enetIfPtr->deviceNumber].pcbTail, pcbPtr);
            if (pcbPtr)
            {
                pcbPtr->FRAG[0].LENGTH = length;
                pcbPtr->FRAG[0].FRAGMENT = packet;
                pcbPtr->FRAG[1].LENGTH = 0;
                pcbPtr->PRIVATE = (void *)enetIfPtr;
				
                switch (type)
                {
                    case ENETPROT_IP:
                    IPE_recv_IP((PCB *)pcbPtr,enetIfPtr->netIfPtr);
                        break;
                    case ENETPROT_ARP:
                    IPE_recv_ARP((PCB *)pcbPtr,enetIfPtr->netIfPtr);
                        break;
#if RTCSCFG_ENABLE_IP6                  
                    case ENETPROT_IP6:
                    IP6E_recv_IP((PCB *)pcbPtr,enetIfPtr->netIfPtr);
                       break;
#endif
                    case ENETPROT_ETHERNET:
#if FSL_FEATURE_ENET_SUPPORT_PTP
                    enet_ptp_service_l2packet(enetIfPtr, packet, length);
#endif
                        break;
                    default:
                    PCB_free((PCB *)pcbPtr);
                        break;	
                }
            }
            else
            {
#if ENET_ENABLE_DETAIL_STATS 
                enetIfPtr->stats.statsRxMissed++;
#endif
            }
        }
    }
}
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_send
 * Return Value: The execution status.
 * Description:  Net send interface. this is called by tcp/ip stack. 
 *
 *END*********************************************************************/
uint32_t ENET_send(_enet_handle handle, PCB_PTR packet, uint32_t type, _enet_address dest, uint32_t flags)	
{
    uint8_t headerLen;
    PCB_FRAGMENT *fragPtr;
    uint16_t size = 0, lenTemp = 0;
    enet_dev_if_t *enetIfPtr;
    enet_ethernet_header_t *packetPtr;
    uint8_t *frame;

    /*Check out*/
    if ((!handle) || (!packet))
    {
        return  ENETERR_INVALID_INIT_PARAM;
    }

    enetIfPtr = (enet_dev_if_t *)handle;
    /* Default frame header size*/
    headerLen = sizeof(enet_ethernet_header_t);

    /* Check the frame length*/
    for (fragPtr = packet->FRAG; fragPtr->LENGTH; fragPtr++)
    {
        size += fragPtr->LENGTH;
    }
    if (size > enetIfPtr->maxFrameSize)
    {
#if ENET_ENABLE_DETAIL_STATS 
        enetIfPtr->stats.statsTxLarge++;
#endif
        return ENETERR_SEND_LARGE;
    }

    /*Add MAC hardware address*/
    packetPtr = (enet_ethernet_header_t *)packet->FRAG[0].FRAGMENT;
    htone(packetPtr->destAddr, dest);
    htone(packetPtr->sourceAddr, enetIfPtr->macCfgPtr->macAddr);
    packetPtr->type = HTONS(type);
    if (flags & ENET_OPT_8021QTAG)
    {
        enet_8021vlan_header_t *vlanHeadPtr = (enet_8021vlan_header_t *)packetPtr;
        vlanHeadPtr->tpidtag = HTONS(ENETPROT_8021Q);
        vlanHeadPtr->othertag = HTONS((ENET_GETOPT_8021QPRIO(flags) << 13));
        vlanHeadPtr->type = HTONS(type);
        headerLen = sizeof(enet_8021vlan_header_t);  
        packet->FRAG[0].LENGTH = headerLen;
    }

    if (flags & ENET_OPT_8023)
    {
        enet_8022_header_ptr lcPtr = (enet_8022_header_ptr)(packetPtr->type + 2);
        packetPtr->type = HTONS(size - headerLen);
        lcPtr->dsap[0] = 0xAA;
        lcPtr->ssap[0] = 0xAA;
        lcPtr->command[0] = 0x03;
        lcPtr->oui[0] = 0x00;
        lcPtr->oui[1] = 0x00;
        lcPtr->oui[2] = 0x00;
        lcPtr->type = HTONS(type);
        packet->FRAG[0].LENGTH = packet->FRAG[0].LENGTH+ sizeof(enet_8022_header_t);
    }

    /* Allocate for a frame */
    frame = enet_mac_dequeue_buffer((void * *)&enetIfPtr->macContextPtr->txBufferPtr);
    if ( frame == NULL)
    {
#if ENET_ENABLE_DETAIL_STATS 
        enetIfPtr->stats.statsTxMissed++;
#endif
        return ENETERR_ALLOC;
    }
    /* Send a whole frame with a signal buffer*/
    for (fragPtr = packet->FRAG; fragPtr->LENGTH; fragPtr++)
    {
        memcpy(frame + lenTemp, fragPtr->FRAGMENT, fragPtr->LENGTH);
        lenTemp += fragPtr->LENGTH;
    }
 
    /* Free the PCB buffer*/
    PCB_free(packet);

    enetIfPtr->macApiPtr->enet_mac_send(enetIfPtr, frame, size);

    return ENET_OK;	
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_address
 * Return Value: The execution status.
 * Description:Get the ENET address of an initialized device 
 *
 *END*********************************************************************/
uint32_t ENET_get_address(_enet_handle handle, _enet_address address)
{
    /* Check input param*/
    if (!handle)
    {
        return ENETERR_INVALID_INIT_PARAM;
    }

    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)handle;

    memcpy(address, enetIfPtr->macCfgPtr->macAddr, kEnetMacAddrLen);

    return ENET_OK;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_mac_address
 * Return Value: The execution status.
 * Description:Get the ENET address of an uninitialized device 
 *
 *END*********************************************************************/
uint32_t ENET_get_mac_address(uint32_t device, uint32_t value, _enet_address address)
{
    enetMacAddr g_enetAddress = ENET_DEFAULT_MAC_ADD;

    address[0] = g_enetAddress[0];
    address[1] = g_enetAddress[1];
    address[2] = g_enetAddress[2];
    address[3] = (value & 0xFF00000U)>>16;
    address[4] = (value & 0xFF00U) >> 8;
    address[5] = (value & 0xFFU);

    return ENET_OK;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_join
 * Return Value: The execution status.
 * Description: Join a multicast group. 
 *
 *END*********************************************************************/
uint32_t ENET_join(_enet_handle handle, uint16_t type, _enet_address address)
{
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)handle;
    enet_multicast_group_t *enetMultiGroupPtr;
	
    /* Make sure it's a multicast group*/
    if (!(address[0] & 1U))
    {
       return ENETERR_JOIN_MULTICAST;
    }
	
    lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);

    if (!enetIfPtr->multiGroupPtr)
    {
        enetIfPtr->multiGroupPtr = mem_allocate(sizeof(enet_multicast_group_t));
        if (enetIfPtr->multiGroupPtr == NULL)
        {
            lock_release(&enetIfPtr->enetContextSync);
            return ENETERR_ALLOC;
        }
        memcpy(enetIfPtr->multiGroupPtr->groupAdddr, address, kEnetMacAddrLen); 
        enetIfPtr->macApiPtr->enet_add_multicast_group(enetIfPtr->deviceNumber, enetIfPtr->multiGroupPtr, address);
        enetIfPtr->multiGroupPtr->next = NULL;
        enetIfPtr->multiGroupPtr->prv = NULL;
    }
    else
    {
        /* Check if we had add the multicast group*/
        enetMultiGroupPtr = enetIfPtr->multiGroupPtr;
        while (enetMultiGroupPtr != NULL)
        {
            if (!memcmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
            {
                lock_release(&enetIfPtr->enetContextSync);
            	return ENETERR_INITIALIZED_MULTICAST;
            }
            if (enetMultiGroupPtr->next == NULL)
            {
                break;
            }
            enetMultiGroupPtr =  enetMultiGroupPtr->next;		
        }

        /* Add this multicast group*/
        enetMultiGroupPtr->next = mem_allocate_zero(sizeof(enet_multicast_group_t));
        if (enetMultiGroupPtr->next == NULL)
        {
            lock_release(&enetIfPtr->enetContextSync);
            return ENETERR_ALLOC;
        }
        memcpy(enetMultiGroupPtr->next->groupAdddr, address, kEnetMacAddrLen);
        enetIfPtr->macApiPtr->enet_add_multicast_group(enetIfPtr->deviceNumber, enetMultiGroupPtr->next, address);
        enetMultiGroupPtr->next->next = NULL;
        enetMultiGroupPtr->next->prv = enetMultiGroupPtr;
    }
	
    lock_release(&enetIfPtr->enetContextSync);
    return ENET_OK;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_leave
 * Return Value: The execution status.
 * Description: Leave a multicast group. 
 *
 *END*********************************************************************/
uint32_t ENET_leave(_enet_handle handle, uint16_t type, _enet_address address)
{
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)handle;
    enet_multicast_group_t *enetMultiGroupPtr, *enetTempPtr;
	
    /* Make sure it's a multicast group*/
    if (!(address[0] & 1U))
    {
       return ENETERR_JOIN_MULTICAST;
    }
	
    lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);
	
    if (!enetIfPtr->multiGroupPtr)
    {
        lock_release(&enetIfPtr->enetContextSync);
        return ENETERR_NULL_MULTICAST;
    }
	
    /* Check if we had add the multicast group*/
    for (enetMultiGroupPtr = enetIfPtr->multiGroupPtr; enetMultiGroupPtr != NULL;enetMultiGroupPtr = enetMultiGroupPtr->next )
    {
        if (!memcmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
        {
            enetIfPtr->macApiPtr->enet_leave_multicast_group(enetIfPtr->deviceNumber, enetMultiGroupPtr, address);
            memset(enetMultiGroupPtr->groupAdddr, 0, kEnetMacAddrLen);
            enetTempPtr = enetMultiGroupPtr->prv;
            if (enetTempPtr != NULL)
            {
                enetTempPtr->next = enetMultiGroupPtr->next;
            }		 
            if (enetMultiGroupPtr->next != NULL)
            {
                enetMultiGroupPtr->next->prv = enetTempPtr;
            }	 
            mem_free((void *)enetMultiGroupPtr);
            break;
        }
    }

    lock_release(&enetIfPtr->enetContextSync);
	
    return ENET_OK;
}

#if BSPCFG_ENABLE_ENET_STATS
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_stats
 * Return Value: The execution status.
 * Description: Get ENET packets statistic. 
 *
 *END*********************************************************************/
ENET_STATS_PTR ENET_get_stats(_enet_handle handle)
{	
#if ENET_ENABLE_DETAIL_STATS 
    /* Common stats*/
    memcpy(&enetStats.COMMON, &((enet_dev_if_t *)handle)->stats, sizeof(enetStats.COMMON));
    /* Detail stats*/
    enetStats.ST_RX_ALIGN = ((enet_dev_if_t *)handle)->stats.statsRxAlign;
    enetStats.ST_RX_FCS = ((enet_dev_if_t *)handle)->stats.statsRxFcs;
    enetStats.ST_RX_GIANT = ((enet_dev_if_t *)handle)->stats.statsRxLengthGreater;
    enetStats.ST_RX_LATECOLL = ((enet_dev_if_t *)handle)->stats.statsRxCollision;
    enetStats.ST_RX_OVERRUN = ((enet_dev_if_t *)handle)->stats.statsRxOverRun;
    enetStats.ST_RX_RUNT = ((enet_dev_if_t *)handle)->stats.statsRxTruncate;
    enetStats.ST_TX_EXCESSCOLL = ((enet_dev_if_t *)handle)->stats.statsTxExcessCollision;
    enetStats.ST_TX_UNDERRUN = ((enet_dev_if_t *)handle)->stats.statsTxUnderFlow;
    enetStats.ST_TX_LATECOLL = ((enet_dev_if_t *)handle)->stats.statsTxLateCollision;

    enetStats.ST_TX_COPY_LARGE = ((enet_dev_if_t *)handle)->stats.statsTxLarge;
#endif	
    return (ENET_STATS_PTR)&enetStats;
}
#endif
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_link_status
 * Return Value: True if link is up else false.
 * Description: Get ENET link status.
 * If ENET is link up return true else false. 
 *
 *END*********************************************************************/
bool ENET_link_status(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)handle;
    bool status = false;
	
    ((enet_phy_api_t *)(enetIfPtr->phyApiPtr))->phy_get_link_status(enetIfPtr, &status);

    return status;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_speed
 * Return Value: The link speed.
 * Description: Get ENET link speed. 
 *
 *END*********************************************************************/
uint32_t ENET_get_speed(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr; 
    enet_phy_speed_t status;
    uint32_t result;

    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }

    enetIfPtr = (enet_dev_if_t *)handle;
    result = ((enet_phy_api_t *)(enetIfPtr->phyApiPtr))->phy_get_link_speed(enetIfPtr, &status);
    if (result != kStatus_ENET_Success)
    {
        return result;
    }
    else
    {
        if (status == (uint32_t)kEnetSpeed100M)
        {
            return 100;
        }
        else if (status == (uint32_t)kEnetSpeed10M)
        {
            return 10;
        }
    }

    return ENET_ERROR; 
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_MTU
 * Return Value: The value of MTU.
 * Description: Get ENET MTU. 
 *
 *END*********************************************************************/
uint32_t ENET_get_MTU(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr;

    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }
	
    enetIfPtr = (enet_dev_if_t *)handle;
    if (!enetIfPtr->maxFrameSize)
    {
        return kEnetMaxFrameDateSize;
    }

    if (enetIfPtr->macCfgPtr->isVlanEnabled)
    {
        return enetIfPtr->maxFrameSize - sizeof(enet_ethernet_header_t) - kEnetFrameFcsLen;
    }
    else
    {
       return enetIfPtr->maxFrameSize - sizeof(enet_8021vlan_header_t) - kEnetFrameFcsLen;
    }   
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_phy_register
 * Return Value: The number of registers .
 * Description: Read required ENET PHY registers. 
 *
 *END*********************************************************************/
bool ENET_phy_registers(_enet_handle handle, uint32_t numRegs, uint32_t *regPtr)
{
    uint32_t counter;
    enet_dev_if_t *enetIfPtr;
		
    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }

    enetIfPtr = (enet_dev_if_t *)handle;
    if (!enetIfPtr->maxFrameSize)
    {
        return kEnetMaxFrameDateSize;
    }
	

    for (counter = 0; counter < numRegs; counter++)
    {
        *regPtr = 0;
        if (enetIfPtr->macApiPtr->enet_mii_read(enetIfPtr->deviceNumber, enetIfPtr->phyCfgPtr->phyAddr, counter, regPtr) != kStatus_ENET_Success)
        {
            return false;
        }
        regPtr ++;
    }
    return true;
}
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_next_device_handle
 * Return Value: The device structure address .
 * Description: Get the next device structure address. 
 *
 *END*********************************************************************/
_enet_handle ENET_get_next_device_handle(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr;

    /* Check input parameter*/
    if (handle == NULL)
    {
        return NULL;
    }

    enetIfPtr = (enet_dev_if_t *)handle;

    return (void *)enetIfPtr->next;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_get_options
 * Return Value: The device structure address.
 * Description: Get device option.
 *
 *END*********************************************************************/
uint32_t ENET_get_options(_enet_handle handle)
{
    enet_dev_if_t * enetIfPtr;
    uint32_t option = 0;
	
    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }

    enetIfPtr = (enet_dev_if_t *)handle;

    if(enetIfPtr->macCfgPtr->isRxAccelEnabled)
    {
        if (enetIfPtr->macCfgPtr->rxAcceler.isIpcheckEnabled)
        {
            option |= ENET_OPTION_HW_RX_IP_CHECKSUM;
        }
        if (enetIfPtr->macCfgPtr->rxAcceler.isProtocolCheckEnabled)	
        {
            option |= ENET_OPTION_HW_RX_PROTOCOL_CHECKSUM;
        }
        if (enetIfPtr->macCfgPtr->rxAcceler.isMacCheckEnabled)
        {
            option |= ENET_OPTION_HW_RX_MAC_ERR;     
        }
    }
    if(enetIfPtr->macCfgPtr->isTxAccelEnabled)
    {
        if (enetIfPtr->macCfgPtr->txAcceler.isIpCheckEnabled)
        {
            option |= ENET_OPTION_HW_TX_IP_CHECKSUM;
        }
        if (enetIfPtr->macCfgPtr->txAcceler.isProtocolCheckEnabled)	
        {
            option |= ENET_OPTION_HW_TX_PROTOCOL_CHECKSUM;
        }
    }
    	
    return option;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_close
 * Return Value: The execution status.
 * Description: Unregisters a protocol type on an Ethernet channel. 
 *
 *END*********************************************************************/
uint32_t ENET_close(_enet_handle handle, uint16_t type)
{
#if !ENET_RECEIVE_ALL_INTERRUPT

    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }
#else
    enet_dev_if_t *enetIfPtr;
    ENET_ECB_STRUCT_PTR ecbPtr, *searchPtr;

    /* Check input parameter*/
    if (handle == NULL)
    {
        return ENETERR_INVALID_DEVICE;
    }
    enetIfPtr = (enet_dev_if_t *)handle;
    lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);
    for (searchPtr = (ENET_ECB_STRUCT_PTR *)&enetIfPtr->enetNetifService;
        *searchPtr; searchPtr = &(*searchPtr)->NEXT)
    {
        if ((*searchPtr)->TYPE == type)
        {
            break;
        }
    }

    if (!*searchPtr)
    {    
        lock_release(&enetIfPtr->enetContextSync);
        return ENETERR_CLOSE_PROT;
    }

    ecbPtr = *searchPtr;
    *searchPtr = ecbPtr->NEXT;

    lock_release(&enetIfPtr->enetContextSync);
    mem_free(ecbPtr);

#endif

    return ENET_OK;
}
/*FUNCTION****************************************************************
 *
 * Function Name: ENET_mediactl
 * Return Value: The execution status.
 * Description: ENET mediactl interface. 
 *
 *END*********************************************************************/
uint32_t ENET_mediactl(_enet_handle handle, uint32_t commandId, void *inOutParam)
{
    return ENET_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name : ENET_strerror
*  Returned Value: pointer to error string
*  Description: Describe an ENET error code
*
*END*-----------------------------------------------------------------*/
const char * ENET_strerror(uint32_t  error)
{ 
    if (error == ENET_OK) 
    {
        return "OK";
    } 
    if ((error < ENETERR_MIN) || (error > ENETERR_MAX))
    {
        return "Unknown error";
    } 
    return ENET_errlist[error - ENETERR_MIN];
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
