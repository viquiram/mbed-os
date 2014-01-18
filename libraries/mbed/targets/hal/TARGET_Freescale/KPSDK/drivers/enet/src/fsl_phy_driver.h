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
#ifndef __FSL_PHY_DRIVER_H__
#define __FSL_PHY_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_enet_driver.h"
/*! 
 * @addtogroup phy_driver
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Define phy return status */
typedef enum _phy_status
{
    kStatus_PHY_Success = 0, /*!< Success*/
    kStatus_PHY_InvaildInput = 1, /*!< Invalid phy input parameter*/
    kStatus_PHY_TimeOut = 2,  /*!< PHY timeout*/
    kStatus_PHY_Fail = 3  /*!< Phy Fail*/	
}phy_status_t;

/*! @brief Define ENET's  time out*/
typedef enum _phy_timeout
{
    kPhyTimeout = 0x10000, /*!< enet reset timeout*/
}phy_timeout_t;

/*! @brief Define PHY register*/
typedef enum _enet_phy_register
{
    kEnetPhyCR = 0, /*!< phy control register */
    kEnetPhySR = 1, /*!< phy status register*/
    kEnetPhyId1 = 2, /*!< phy identification register 1*/
    kEnetPhyId2 = 3, /*!< phy identification register 2*/
    kEnetPhyCt2 = 0x1f /*!< phy control2 register*/
}enet_phy_register_t;

/*! @brief Define the control flag*/
typedef enum _enet_phy_control
{
    kEnetPhyAutoNeg = 0x1000,/*!< enet phy auto negotiation control*/
    kEnetPhySpeed = 0x2000, /*! enet phy speed control*/
    kEnetPhyLoop = 0x4000, /*!< enet phy loop control*/
    kEnetPhyReset = 0x8000, /*!< enet phy reset control*/
    kEnetPhy10HalfDuplex = 0x4, /*!< enet phy 10M half duplex*/
    kEnetPhy100HalfDuplex = 0x8,/*!< enet phy 100M half duplex*/
    kEnetPhy10FullDuplex = 0x14,/*!< enet phy 10M full duplex*/
    kEnetPhy100FullDuplex = 0x18/*!< enet phy 100M full duplex*/
}enet_phy_control_t;

/*! @brief Define phy link speed */
typedef enum _enet_phy_speed
{
    kEnetSpeed10M = 0,   /*!< enet phy 10M speed*/
    kEnetSpeed100M = 1  /*!< enet phy 100M speed*/
}enet_phy_speed_t;

/*! @brief Define phy link duplex*/
typedef enum _enet_phy_duplex
{
    kEnetHalfDuplex = 0, /*!< enet phy half duplex*/
    kEnetFullDuplex = 1  /*!< enet phy full duplex*/
}enet_phy_duplex_t;

/*! @brief Define the phy status*/
typedef enum _enet_phy_status
{
    kEnetPhyLinkStatus = 0x4,  /*!< enet phy link status bit*/
    kEnetPhyAutoNegAble = 0x08, /*!< enet phy auto negotiation ability*/
    kEnetPhyAutoNegComplete = 0x20, /*!< enet phy auto negotiation complete*/
    kEnetPhySpeedDulpexMask = 0x1c /*!< enet phy speed mask on status register 2*/
}enet_phy_status_t;

/*! @brief Define basic PHY application*/
typedef struct enet_phy_api
{
    uint32_t (* phy_auto_discover)(enet_dev_if_t * enetIfPtr);/*!< phy auto discover*/
    uint32_t (* phy_init)(enet_dev_if_t * enetIfPtr);/*!< phy initialize*/
    uint32_t (* phy_get_link_speed)(enet_dev_if_t * enetIfPtr, enet_phy_speed_t *speed);/*!<  get phy speed*/
    uint32_t (* phy_get_link_status)(enet_dev_if_t * enetIfPtr, bool *status);/*! get phy link status*/
    uint32_t (* phy_get_link_duplex)(enet_dev_if_t * enetIfPtr, enet_phy_duplex_t *duplex);/*!< get phy link duplex*/
}enet_phy_api_t;

/*******************************************************************************
 * Global variables
 ******************************************************************************/
extern const enet_phy_api_t g_enetPhyApi;

/*******************************************************************************
 * API 
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! 
  * @name PHY Driver
  * @{
  */

/*!
 * @brief Initialize PHY.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t phy_init(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Phy address auto discover.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t phy_auto_discover(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Get phy link speed.
 *
 * @param enetIfPtr The enet context structure.
 * @param status The link speed of phy.
 * @return The execution status.
 */
uint32_t phy_get_link_speed(enet_dev_if_t * enetIfPtr, enet_phy_speed_t *status);

/*!
 * @brief Get phy link status.
 *
 * @param enetIfPtr The enet context structure.
 * @param status The link on or down status of phy.
 * @return The execution status.
 */
uint32_t phy_get_link_status(enet_dev_if_t * enetIfPtr, bool *status);

/*!
 * @brief Get phy link duplex.
 *
 * @param enetIfPtr The enet context structure.
 * @param status The link duplex status of phy.
 * @return The execution status.
 */
uint32_t phy_get_link_duplex(enet_dev_if_t * enetIfPtr, enet_phy_duplex_t *status);

/* @} */

#if defined(__cplusplus)
extern }
#endif

/*! @}*/

#endif /* __FSL_PHY_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

