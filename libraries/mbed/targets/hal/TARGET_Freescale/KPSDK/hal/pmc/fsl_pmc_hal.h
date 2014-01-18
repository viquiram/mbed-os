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
#if !defined(__FSL_PMC_HAL_H__)
#define __FSL_PMC_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_pmc_features.h"

/*! @addtogroup pmc_hal*/
/*! @{*/

/*! @file fsl_pmc_hal.h */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Low-Voltage Warning Voltage Select*/
typedef enum _pmc_lvwv_select {
    kPmcLvwvLowTrip,            /*!< Low trip point selected (VLVW = VLVW1)*/
    kPmcLvwvMid1Trip,           /*!< Mid 1 trip point selected (VLVW = VLVW2)*/
    kPmcLvwvMid2Trip,           /*!< Mid 2 trip point selected (VLVW = VLVW3)*/
    kPmcLvwvHighTrip            /*!< High trip point selected (VLVW = VLVW4)*/
} pmc_lvwv_select_t;

/*! @brief Low-Voltage Detect Voltage Select*/
typedef enum _pmc_lvdv_select {
    kPmcLvdvLowTrip,            /*!< Low trip point selected (V LVD = V LVDL )*/
    kPmcLvdvHighTrip,           /*!< High trip point selected (V LVD = V LVDH )*/
} pmc_lvdv_select_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Power Management Controller Ccontrol APIs*/
/*@{*/


/*!
 * @brief Low-Voltage Detect Interrupt Enable
 *
 * This function will enable the the interrupt for low voltage detection. When
 * enabled, if the LVDF (Low Voltage Detect Flag) is set, a hardware interrupt
 * will happen to the system.
 *
 */
static inline void pmc_hal_enable_low_voltage_detect_interrupt(void)
{
    BW_PMC_LVDSC1_LVDIE(1);
}

/*!
 * @brief Low-Voltage Detect Interrupt Disable (use polling)
 *
 * This function will disable the the interrupt for low voltage detection. When
 * disabled, application can only check the low voltage through polling the LVDF
 * (Low Voltage Detect Flag).
 *
 */
static inline void pmc_hal_disable_low_voltage_detect_interrupt(void)
{
    BW_PMC_LVDSC1_LVDIE(0);
}

/*!
 * @brief Low-Voltage Detect Hardware Reset Enable (write once)
 *
 * This function will enable the the hardware reset for low voltage detection. 
 * When enabled, if the LVDF (Low Voltage Detect Flag) is set, a hardware reset
 * will happen. This setting is a write once only and additional writes are 
 * ignored.
 *
 */
static inline void pmc_hal_enable_low_voltage_detect_reset(void)
{
    BW_PMC_LVDSC1_LVDRE(1);
}

/*!
 * @brief Low-Voltage Detect Hardware Reset Disable
 *
 * This function will disable the the hardware reset for low voltage detection. 
 * When disabled, if the LVDF (Low Voltage Detect Flag) is set, a hardware reset
 * will not happen. This setting is a write once only and additional writes are 
 * ignored.
 *
 */
static inline void pmc_hal_disable_low_voltage_detect_reset(void)
{
    BW_PMC_LVDSC1_LVDRE(0);
}

/*!
 * @brief Low-Voltage Detect Acknowledge
 *
 * This function is used to acknowledge low voltage detection errors (write 1 to
 * clear LVDF)
 *
 */
static inline void pmc_hal_low_voltage_detect_ack(void)
{
    BW_PMC_LVDSC1_LVDACK(1);
}

/*!
 * @brief Low-Voltage Detect Flag Read
 *
 * This function will read the current LVDF status. If returns 1, it means low
 * voltage event is detected.
 *
 * @return status Current low voltage detect flag
 *                - true: Low-Voltage detected
 *                - false: Low-Voltage not detected
 */
static inline bool pmc_hal_get_low_voltage_detect_flag(void)
{
    return BR_PMC_LVDSC1_LVDF;
}

/*!
 * @brief Set Low-Voltage Detect Voltage Select
 *
 * This function will set the low voltage detect voltage select. It will set
 * the low voltage detect trip point voltage (Vlvd). Application can select
 * either low trip or high trip point. Refer to reference manual for details.
 *
 * @param select Voltage select setting defined in pmc_lvdv_select_t
 */
static inline void pmc_hal_set_low_voltage_detect_voltage_select(pmc_lvdv_select_t select)
{
    BW_PMC_LVDSC1_LVDV(select);
}

/*!
 * @brief Get Low-Voltage Detect Voltage Select
 *
 * This function will get the low voltage detect voltage select. It will get
 * the low voltage detect trip point voltage (Vlvd). Application can select
 * either low trip or high trip point. Refer to reference manual for details.
 *
 * @return select Current voltage select setting
 */
static inline pmc_lvdv_select_t pmc_hal_get_low_voltage_detect_voltage_select(void)
{
    return (pmc_lvdv_select_t)BR_PMC_LVDSC1_LVDV;
}


/*!
 * @brief Low-Voltage Warning Interrupt Enable
 *
 * This function will enable the the interrupt for low voltage warning 
 * detection. When enabled, if the LVWF (Low Voltage Warning Flag) is set, 
 * a hardware interrupt will happen to the system.
 *
 */
static inline void pmc_hal_enable_low_voltage_warning_interrupt(void)
{
    BW_PMC_LVDSC2_LVWIE(1);
}

/*!
 * @brief Low-Voltage Warning Interrupt Disable (use polling)
 *
 * This function will disable the the interrupt for low voltage warning 
 * detection. When disabled, if the LVWF (Low Voltage Warning Flag) is set, 
 * a hardware interrupt will not happen to the system.
 *
 */
static inline void pmc_hal_disable_low_voltage_warning_interrupt(void)
{
    BW_PMC_LVDSC2_LVWIE(0);
}

/*!
 * @brief Low-Voltage Warning Acknowledge
 * 
 * This function is used to acknowledge low voltage warning errors (write 1 to
 * clear LVWF).
 *
 */
static inline void pmc_hal_low_voltage_warning_ack(void)
{
    BW_PMC_LVDSC2_LVWACK(1);
}

/*!
 * @brief Low-Voltage Warning Flag Read
 *
 * This function is used to poll the current LVWF status. When returns 1, it 
 * indicates a low-voltage warning event. LVWF is set when V Supply transitions
 * below the trip point or after reset and V Supply is already below V LVW.
 *
 * @return status Current LVWF status
                  - true: Low-Voltage Warning Flag is set
                  - falase: no Low-Voltage Warning happen
 */
static inline bool pmc_hal_get_low_voltage_warning_flag(void)
{
    return BR_PMC_LVDSC2_LVWF;
}

/*!
 * @brief Set Low-Voltage Warning Voltage Select
 *
 * This function will set the low voltage warning voltage select. It will set
 * the low voltage warning trip point voltage (Vlvw). Application can select
 * either low, mid1, mid2 and high trip point. Refer to reference manual for 
 * details. Also refer to pmc_lvwv_select_t for supported settings
 * 
 * @param select Low voltage warning select setting
 */
static inline void pmc_hal_set_low_voltage_warning_voltage_select(pmc_lvwv_select_t select)
{
    BW_PMC_LVDSC2_LVWV(select);
}

/*!
 * @brief Get Low-Voltage Warning Voltage Select
 *
 * This function will get the low voltage warning voltage select. It will get
 * the low voltage warning trip point voltage (Vlvw). Refer to pmc_lvwv_select_t
 * for detail supported settings.
 *
 * @return select Current low voltage warning select setting
 */
static inline pmc_lvwv_select_t pmc_hal_get_low_voltage_warning_voltage_select(void)
{
    return (pmc_lvwv_select_t)BR_PMC_LVDSC2_LVWV;
}

#if FSL_FEATURE_SMC_HAS_BGEN
/*!
 * @brief Enable Bandgap in VLPx Operation
 *
 * This function will enable bandgap in lower power modes of operation (VLPx, 
 * LLS, and VLLSx). When on-chip peripherals require the bandgap voltage 
 * reference in low power modes of operation, set BGEN to continue to enable
 * the bandgap operation
 *
 */
static inline void pmc_hal_enable_bandgap_in_low_power_mode(void)
{
    BW_PMC_REGSC_BGEN(1);
}

/*!
 * @brief Disable Bandgap in VLPx Operation
 *
 * This function will disable bandgap in lower power modes of operation (VLPx, 
 * LLS, and VLLSx). When the bandgap voltage reference is not needed in low 
 * power modes, disable BGEN to avoid excess power consumption.
 *
 */
static inline void pmc_hal_disable_bandgap_in_low_power_mode(void)
{
    BW_PMC_REGSC_BGEN(0);
}
#endif

/*!
 * @brief Enable Bandgap Buffer
 *
 * This function will enable the Bandgap buffer.
 *
 */
static inline void pmc_hal_enable_bandgap_buffer(void)
{
    BW_PMC_REGSC_BGBE(1);
}

/*!
 * @brief Disable Bandgap Buffer
 *
 * This function will disable the Bandgap buffer.
 *
 */
static inline void pmc_hal_disable_bandgap_buffer(void)
{
    BW_PMC_REGSC_BGBE(0);
}

/*!
 * @brief Get Acknowledge Isolation
 *
 * This function will read the Acknowledge Isolation setting that indicates 
 * whether certain peripherals and the I/O pads are in a latched state as 
 * a result of having been in a VLLS mode. 
 *
 * @return value Ack isolation. 
 *               0 - Peripherals and I/O pads are in normal run state
 *               1 - Certain peripherals and I/O pads are in an isolated and
 *                   latched state
 */
static inline uint8_t pmc_hal_get_ack_isolation(void)
{
    return BR_PMC_REGSC_ACKISO;
}

/*!
 * @brief Clear Acknowledge Isolation
 *
 * This function will clear the Ack Isolation flag. Writing one to this setting
 * when it is set releases the I/O pads and certain peripherals to their normal
 * run mode state.
 *
 */
static inline void pmc_hal_clear_ack_isolation(void)
{
    BW_PMC_REGSC_ACKISO(1);
}

/*!
 * @brief Get Regulator regulation status
 *
 * This function will return the regulator in run regulation status. It provides
 * the current status of the internal voltage regulator.
 *
 * @return value Regulation status. 
 *               0 - Regulator is in stop regulation or in transition to/from it
 *               1 - Regulator is in run regulation
 *
 */
static inline uint8_t pmc_hal_get_regulator_status(void)
{
    return BR_PMC_REGSC_REGONS;
}

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_PMC_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

