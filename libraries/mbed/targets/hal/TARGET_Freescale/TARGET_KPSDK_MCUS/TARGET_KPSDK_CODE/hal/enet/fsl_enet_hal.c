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
 
#include "fsl_enet_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_set_mac_address
 * Description: Set ENET mac physical address.
 * 
 *END*********************************************************************/
void enet_hal_set_mac_address(uint32_t instance, enetMacAddr hwAddr)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    uint32_t address;
	
    address = (uint32_t)(((uint32_t)hwAddr[0] << 24U)|((uint32_t)hwAddr[1] << 16U)|((uint32_t)hwAddr[2] << 8U)| (uint32_t)hwAddr[3]) ;
    HW_ENET_PALR_SET(instance,address);             /* Set low physical address */
    address = (uint32_t)(((uint32_t)hwAddr[4] << 24U)|((uint32_t)hwAddr[5] << 16U)) ;
    HW_ENET_PAUR_SET(instance,address);             /* Set high physical address */
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_set_group_hashtable
 * Description: Set multicast group address hash value to the mac register
 * To join the multicast group address.
 *END*********************************************************************/
void enet_hal_set_group_hashtable(uint32_t instance, uint32_t crcValue, enet_special_address_filter_t mode)
{
    assert(instance < HW_ENET_INSTANCE_COUNT); 
	
    switch (mode)
    {
        case kEnetSpecialAddressInit:           /* Clear group address register on ENET initialize */
            HW_ENET_GALR_SET(instance,0);
            HW_ENET_GAUR_SET(instance,0);			
            break;
        case kEnetSpecialAddressEnable:         /* Enable a multicast group address*/
            if (!((crcValue >> 31) & 1U))
            {
                HW_ENET_GALR_SET(instance,(1U << ((crcValue >> 26) & kEnetHashValMask))); 
            }
            else
            {
                HW_ENET_GAUR_SET(instance,(1U << ((crcValue >> 26) & kEnetHashValMask)));
            }
            break;
        case kEnetSpecialAddressDisable:       /* Disable a multicast group address*/
            if (!((crcValue >> 31) & 1U))
            {
                HW_ENET_GALR_CLR(instance,(1U << ((crcValue >> 26) & kEnetHashValMask)));
            }
            else
            {
                HW_ENET_GAUR_CLR(instance,(1U << ((crcValue>>26) & kEnetHashValMask))); 
            }
        break;
        default:
        break;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_set_individual_hashtable 
 * Description: Set a specific unicast address hash value to the mac register
 * To receive frames with the individual destination address.  
 *END*********************************************************************/
void enet_hal_set_individual_hashtable(uint32_t instance, uint32_t crcValue, enet_special_address_filter_t mode)
{
    assert(instance < HW_ENET_INSTANCE_COUNT); 
	
    switch (mode)
    {
        case kEnetSpecialAddressInit:         /* Clear individual address register on ENET initialize */
            HW_ENET_IALR_SET(instance,0);
            HW_ENET_IAUR_SET(instance,0);			
            break;
        case kEnetSpecialAddressEnable:        /* Enable a special address*/
            if (((crcValue >>31) & 1U) == 0)
            {
                HW_ENET_IALR_SET(instance,(1U << ((crcValue>>26)& kEnetHashValMask))); 
            }
            else
            {
                HW_ENET_IAUR_SET(instance,(1U << ((crcValue>>26)& kEnetHashValMask)));
            }
            break;
        case kEnetSpecialAddressDisable:     /* Disable a special address*/
            if (((crcValue >>31) & 1U) == 0)
            {
                HW_ENET_IALR_CLR(instance,(1U << ((crcValue>>26)& kEnetHashValMask)));
            }
            else
            {
                HW_ENET_IAUR_CLR(instance,(1U << ((crcValue>>26)& kEnetHashValMask))); 
            }	
            break;
        default:
            break;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_tx_fifo
 * Description: Configure ENET transmit FIFO.  
 *END*********************************************************************/
void enet_hal_config_tx_fifo(uint32_t instance, enet_config_tx_fifo_t *thresholdCfg)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    assert(thresholdCfg);

    BW_ENET_TFWR_STRFWD(instance,thresholdCfg->isStoreForwardEnabled);   /* Set store and forward mode*/
    BW_ENET_TFWR_TFWR(instance,thresholdCfg->txFifoWrite);               /* Set transmit FIFO write	region*/
    BW_ENET_TSEM_TX_SECTION_EMPTY(instance,thresholdCfg->txEmpty);       /* Set transmit FIFO empty threshold*/
    BW_ENET_TAEM_TX_ALMOST_EMPTY(instance,thresholdCfg->txAlmostEmpty);  /* Set transmit FIFO almost empty threshold*/
    BW_ENET_TAFL_TX_ALMOST_FULL(instance,thresholdCfg->txAlmostFull);    /* Set transmit FIFO almost full threshold*/
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_rx_fifo
 * Description: Configure ENET receive FIFO.  
 *END*********************************************************************/
void enet_hal_config_rx_fifo(uint32_t instance,enet_config_rx_fifo_t *thresholdCfg )
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    assert(thresholdCfg);

    BW_ENET_RSFL_RX_SECTION_FULL(instance,thresholdCfg->rxFull);        /* Set receive FIFO full threshold*/
    BW_ENET_RSEM_RX_SECTION_EMPTY(instance,thresholdCfg->rxEmpty);      /* Set receive FIFO empty threshold*/
    BW_ENET_RAEM_RX_ALMOST_EMPTY(instance,thresholdCfg->rxAlmostEmpty); /* Set receive FIFO almost empty threshold*/
    BW_ENET_RAFL_RX_ALMOST_FULL(instance,thresholdCfg->rxAlmostFull);   /* Set receive FIFO almost full threshold*/    
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_init_bd_address 
 * Description: Initialize the start address of buffer descriptors.  
 *END*********************************************************************/
void enet_hal_init_bd_address(uint32_t instance, uint32_t rxBdAddr, uint32_t txBdAddr)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
	
    HW_ENET_RDSR_SET(instance,rxBdAddr);   /* Initialize receive buffer descriptor start address*/
    HW_ENET_TDSR_SET(instance,txBdAddr);   /* Initialize transmit buffer descriptor start address*/
#if SYSTEM_LITTLE_ENDIAN && !FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY
    BW_ENET_ECR_DBSWP(instance,1);         /* buffer descriptor byte swapping for little-endian system and endianness configurable IP*/
#endif
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_init_rxbds
 * Description: Initialize ENET receive buffer descriptors.
 *END*********************************************************************/
void enet_hal_init_rxbds(void *rxBds, uint8_t *buffer, bool isLastBd)
{
    assert(rxBds);
    assert(buffer);

    ((enet_bd_struct_t *)rxBds)->buffer = (uint8_t *)NTOHL((uint32_t)buffer); /* Set data buffer address */
    ((enet_bd_struct_t *)rxBds)->length = 0;    /* Initialize data length*/

    /*The last buffer descriptor should be set with the wrap flag*/
    if (isLastBd)
    {    
        ((enet_bd_struct_t *)rxBds)->control |= kEnetRxBdWrap; 
    }
    ((enet_bd_struct_t *)rxBds)->control |= kEnetRxBdEmpty;   /* Initialize bd with empty bit*/
    ((enet_bd_struct_t *)rxBds)->controlExtend1 |= kEnetRxBdIntrrupt;/* Enable receive interrupt*/
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_init_txbds
 * Description: Initialize ENET transmit buffer descriptors.
 *END*********************************************************************/
void enet_hal_init_txbds(void *txBds, bool isLastBd)
{
    assert(txBds);
	
    ((enet_bd_struct_t *)txBds)->length = 0;   /* Initialize  data length*/

    /*The last buffer descriptor should be set with the wrap flag*/
    if (isLastBd)
    {
        ((enet_bd_struct_t *)txBds)->control |= kEnetTxBdWrap;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_update_rxbds
 * Description: Update ENET receive buffer descriptors.
 *END*********************************************************************/
void enet_hal_update_rxbds(void *rxBds, uint8_t *data, bool isbufferUpdate)
{
    assert(rxBds);

    if (isbufferUpdate)
    {
        ((enet_bd_struct_t *)rxBds)->buffer = (uint8_t *)HTONL((uint32_t)data);
    }
    ((enet_bd_struct_t *)rxBds)->control &= kEnetRxBdWrap;  /* Clear status*/
    ((enet_bd_struct_t *)rxBds)->control |= kEnetRxBdEmpty;   /* Set rx bd empty*/
    ((enet_bd_struct_t *)rxBds)->controlExtend1 |= kEnetRxBdIntrrupt;/* Enable interrupt*/
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_update_txbds
 * Description: Update ENET transmit buffer descriptors.
 *END*********************************************************************/
void enet_hal_update_txbds(void *txBds,uint8_t *buffer, uint16_t length, bool isTxtsCfged)
{
    assert(txBds);
    assert(buffer);

    ((enet_bd_struct_t *)txBds)->length = HTONS(length); /* Set data length*/
    ((enet_bd_struct_t *)txBds)->buffer = (uint8_t *)HTONL((uint32_t)buffer); /* Set data buffer*/
    ((enet_bd_struct_t *)txBds)->control |= kEnetTxBdLast | kEnetTxBdTransmitCrc | kEnetTxBdReady;/* set control */
    if (isTxtsCfged)
    {
         /* Set receive and timestamp interrupt*/
        ((enet_bd_struct_t *)txBds)->controlExtend1 |= (kEnetTxBdTxInterrupt | kEnetTxBdTimeStamp);	
    }
    else
    {
        /* Set receive interrupt*/
        ((enet_bd_struct_t *)txBds)->controlExtend1 |= kEnetTxBdTxInterrupt;	
    }   
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_get_rxbd_control
 * Description: Get receive buffer descriptor control and status region.
 *END*********************************************************************/
uint16_t enet_hal_get_rxbd_control(void *curBd)
{
    assert(curBd);

    return ((enet_bd_struct_t *)curBd)->control;	
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_get_txbd_control
 * Description: Get ENET transmit buffer descriptor control and status data.
 *END*********************************************************************/
uint16_t enet_hal_get_txbd_control(void *curBd)
{
    assert(curBd);

    return ((enet_bd_struct_t *)curBd)->control;	
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_get_rxbd_control_extend
 * Description: Get ENET receive buffer descriptor extended control region.
 *END*********************************************************************/
bool enet_hal_get_rxbd_control_extend(void *curBd,enet_rx_bd_control_extend_t controlRegion)
{
    assert(curBd);

#if SYSTEM_LITTLE_ENDIAN && FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY 
    if (((uint16_t)controlRegion > kEnetRxBdCtlJudge1) && ((uint16_t)controlRegion < kEnetRxBdCtlJudge2))                
    {
        return ((((enet_bd_struct_t *)curBd)->controlExtend0 & controlRegion) != 0); /* Control extended0 region*/
    }
    else
    {
        return ((((enet_bd_struct_t *)curBd)->controlExtend1 & controlRegion) != 0); /* Control extended1 region*/
    }	
#else
    if( (uint16_t)controlRegion < kEnetRxBdCtlJudge1)                 
    {
        return ((((enet_bd_struct_t *)curBd)->controlExtend0 & controlRegion) != 0); /* Control extended0 region*/
    }
    else
    {
        return ((((enet_bd_struct_t *)curBd)->controlExtend1 & controlRegion) != 0);/* Control extended1 region*/
    }
#endif
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_get_txbd_control_extend
 * Description: Get ENET transmit buffer descriptor extended control region.
 *END*********************************************************************/
uint16_t enet_hal_get_txbd_control_extend(void *curBd)
{
    assert(curBd);

    return ((enet_bd_struct_t *)curBd)->controlExtend0;	
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_get_txbd_timestamp_flag
 * Description: Get ENET transmit buffer descriptor timestamp region.
 *END*********************************************************************/
bool enet_hal_get_txbd_timestamp_flag(void *curBd)
{
    assert(curBd);

    return ((((enet_bd_struct_t *)curBd)->controlExtend1 & kEnetTxBdTimeStamp) != 0);	
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_rmii
 * Description: Configure (R)MII mode.
 *END*********************************************************************/
void enet_hal_config_rmii(uint32_t instance, enet_config_rmii_t mode, enet_config_speed_t speed, enet_config_duplex_t duplex, bool isRxOnTxDisabled,  bool isLoopEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_RCR_MII_MODE(instance,1);             /* Set mii mode */
    BW_ENET_RCR_RMII_MODE(instance,mode);
    BW_ENET_RCR_RMII_10T(instance,speed);         /* Set speed mode	*/
    BW_ENET_TCR_FDEN(instance,duplex);            /* Set duplex mode*/
    if ((!duplex) && isRxOnTxDisabled)
    {
        BW_ENET_RCR_DRT(instance,1);              /* Disable receive on transmit*/
    }
	
    if (mode == kEnetCfgMii)                 /* Set internal loop only for mii mode*/
    {             
        BW_ENET_RCR_LOOP(instance,isLoopEnabled);
    }
    else
    {
        BW_ENET_RCR_LOOP(instance, 0);    /* Clear internal loop for rmii mode*/
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_set_mii_command
 * Description: Set MII command.
 *END*********************************************************************/
void enet_hal_set_mii_command(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, enet_mii_operation_t operation, uint32_t data)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    uint32_t mmfrValue = 0 ;
	
    mmfrValue = BF_ENET_MMFR_ST(1)| BF_ENET_MMFR_OP(operation)| BF_ENET_MMFR_PA(phyAddr) | BF_ENET_MMFR_RA(phyReg)| BF_ENET_MMFR_TA(2) | (data&0xFFFF); /* mii command*/
    HW_ENET_MMFR_WR(instance,mmfrValue);
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_etherent
 * Description: Enable or disable normal Ethernet mode and enhanced mode.
 *END*********************************************************************/
void enet_hal_config_ethernet(uint32_t instance, bool isEnhanced, bool isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
		
    BW_ENET_ECR_ETHEREN(instance,isEnabled);     /* Enable/Disable Ethernet module*/
    if (isEnhanced)
    {
        BW_ENET_ECR_EN1588(instance,isEnabled);	 /* Enable/Disable enhanced frame feature*/
    }
}	

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_interrupt
 * Description: Enable or disable different Ethernet interrupts.
 *END*********************************************************************/
void enet_hal_config_interrupt(uint32_t instance, uint32_t source, bool isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    if (isEnabled)
    {
        HW_ENET_EIMR_SET(instance,source);                     /* Enable interrupt */
    }
    else
    {
        HW_ENET_EIMR_CLR(instance,source);                     /* Disable interrupt*/
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_tx_acceleator
 * Description: Configure Ethernet transmit accelerator features.
 *END*********************************************************************/
void enet_hal_config_tx_accelerator(uint32_t instance, enet_config_tx_accelerator_t *txCfgPtr)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    assert(txCfgPtr);
	
    HW_ENET_TACC_WR(instance,0);                                    /* Clear all*/
    BW_ENET_TACC_IPCHK(instance,txCfgPtr->isIpCheckEnabled);        /* Insert ipheader checksum */
    BW_ENET_TACC_PROCHK(instance,txCfgPtr->isProtocolCheckEnabled); /* Insert protocol checksum*/
    BW_ENET_TACC_SHIFT16(instance,txCfgPtr->isShift16Enabled);      /* Set tx fifo shift-16*/
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_config_rx_acceleator
 * Description: Configure Ethernet receive accelerator features.
 *END*********************************************************************/
void enet_hal_config_rx_accelerator(uint32_t instance, enet_config_rx_accelerator_t *rxCfgPtr)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    assert(rxCfgPtr);

    HW_ENET_RACC_WR(instance,0);                                         /* Clear all*/
    BW_ENET_RACC_IPDIS(instance,rxCfgPtr->isIpcheckEnabled);             /* Set ipchecksum feild*/
    BW_ENET_RACC_PRODIS(instance,rxCfgPtr->isProtocolCheckEnabled);      /* Set protocol feild*/
    BW_ENET_RACC_LINEDIS(instance,rxCfgPtr->isMacCheckEnabled);         /* Set maccheck feild*/
    BW_ENET_RACC_SHIFT16(instance,rxCfgPtr->isShift16Enabled);           /* Set rx fifo shift feild*/
    BW_ENET_RACC_PADREM(instance,rxCfgPtr->isPadRemoveEnabled);          /* Set rx padding remove feild*/
}

/*FUNCTION****************************************************************
 *
 * Function Name: enet_hal_init_ptp_timer
 * Description: Initialize Ethernet ptp timer.
 *END*********************************************************************/
void enet_hal_init_ptp_timer(uint32_t instance,enet_config_ptp_timer_t *ptpCfgPtr)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    assert(ptpCfgPtr);
	
    BW_ENET_ATCR_SLAVE(instance,ptpCfgPtr->isSlaveEnabled);    /* Set ptp timer slave/master mode*/
    if (!ptpCfgPtr->isSlaveEnabled)
    {
        BW_ENET_ATINC_INC(instance,ptpCfgPtr->clockIncease);   /* Set increase value for ptp timer*/
        HW_ENET_ATPER_SET(instance,ptpCfgPtr->period);         /* Set wrap time for ptp timer*/
        /* set periodical event and the event signal output assertion*/
        HW_ENET_ATCR_SET(instance,((1U << BP_ENET_ATCR_PEREN)|(1U << BP_ENET_ATCR_PINPER)));    
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

