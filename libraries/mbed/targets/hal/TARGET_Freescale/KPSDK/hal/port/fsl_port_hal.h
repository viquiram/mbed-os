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
#ifndef __FSL_PORT_HAL_H__
#define __FSL_PORT_HAL_H__
 
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_port_features.h"
#include "fsl_device_registers.h"
 
/*!
 * @addtogroup port_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief internal resistor pull feature selection.*/
typedef enum _port_pull {
    kPortPullDown = 0,  /*!< internal pulldown resistor is enabled.*/
    kPortPullUp   = 1   /*!< internal pullup resistor is enabled.*/
} port_pull_t;

/*! @brief slew rate selection.*/
typedef enum _port_slew_rate {
    kPortFastSlewRate = 0,  /*!< fast slew rate is configured.*/
    kPortSlowSlewRate = 1   /*!< slow slew rate is configured.*/
} port_slew_rate_t;

/*! @brief configure drive strength.*/
typedef enum _port_drive_strength {
    kPortLowDriveStrength  = 0, /*!< low drive strength is configured.*/
    kPortHighDriveStrength = 1  /*!< high drive strength is configured.*/
} port_drive_strength_t;

/*! @brief pin mux selection.*/
typedef enum _port_mux {
    kPortPinDisabled = 0,   /*!< corresponding pin is disabled as analog.*/
    kPortMuxAsGpio   = 1,   /*!< corresponding pin is configured as GPIO.*/
    kPortMuxAlt2     = 2,   /*!< chip-specific*/
    kPortMuxAlt3     = 3,   /*!< chip-specific*/
    kPortMuxAlt4     = 4,   /*!< chip-specific*/
    kPortMuxAlt5     = 5,   /*!< chip-specific*/
    kPortMuxAlt6     = 6,   /*!< chip-specific*/
    kPortMuxAlt7     = 7    /*!< chip-specific*/
} port_mux_t;

/*! @brief digital filter clock source selection.*/
#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
typedef enum _port_digital_filter_clock_source {
    kPortBusClock = 0,  /*!< digital filters are clocked by the bus clock.*/
    kPortLPOClock = 1   /*!< digital filters are clocked by the 1 kHz LPO clock.*/
} port_digital_filter_clock_source_t;
#endif

/*! @brief configure interrupt generation condition.*/
typedef enum _port_interrupt_config {
    kPortIntDisabled    = 0x0,  /*!< interrupt/DMA request is disabled.*/
    kPortDmaRisingEdge  = 0x1,  /*!< DMA request on rising edge.*/
    kPortDmaFallingEdge = 0x2,  /*!< DMA request on falling edge.*/
    kPortDmaEitherEdge  = 0x3,  /*!< DMA request on either edge.*/
    kPortIntLogicZero   = 0x8,  /*!< Interrupt when logic zero. */
    kPortIntRisingEdge  = 0x9,  /*!< Interrupt on rising edge. */
    kPortIntFallingEdge = 0xA,  /*!< Interrupt on falling edge. */
    kPortIntEitherEdge  = 0xB,  /*!< Interrupt on either edge. */
    kPortIntLogicOne    = 0xC   /*!< Interrupt when logic one. */
} port_interrupt_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Select internal resistor as pulldown or pullup.
 * 
 * Pull configuration is valid in all digital pin muxing modes.
 *
 * @param instance  port instance number. 
 * @param pin       port pin number. 
 * @param pullSelect  internal resistor pull feature selection.
 *        - kPortPullDown: internal pulldown resistor is enabled.
 *        - kPortPullUp  : internal pullup resistor is enabled.
 */
static inline void port_hal_pull_select(uint32_t instance, 
                                        uint32_t pin, 
                                        port_pull_t pullSelect)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_PS(instance, pin, pullSelect);
}

/*!
 * @brief Enable or disable internal pull resistor.
 *
 * @param instance  port instance number. 
 * @param pin       port pin number. 
 * @param isPullEnabled  internal pull resistor enable or disable .
 *        - true : internal pull resistor is enabled.
 *        - false: internal pull resistor is disabled.
 */
static inline void port_hal_configure_pull(uint32_t instance, uint32_t pin, bool isPullEnabled)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);    
    BW_PORT_PCRn_PE(instance, pin, isPullEnabled);
}

/*!
 * @brief Configure fast/slow slew rate if the pin is used as digital output.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param rateSelect  slew rate selection.
 *        - kPortFastSlewRate: fast slew rate is configured.
 *        - kPortSlowSlewRate: slow slew rate is configured.
 */
static inline void port_hal_configure_slew_rate(uint32_t instance, 
                                                uint32_t pin, 
                                                port_slew_rate_t rateSelect)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_SRE(instance, pin, rateSelect);
}

/*!
 * @brief Configure passive filter if the pin is used as digital input.
 * 
 * If enabled, a low pass filter (10 MHz to 30 MHz bandwidth) will be enabled
 * on the digital input path. Disable the Passive Input Filter when supporting
 * high speed interfaces (> 2 MHz) on the pin.
 *
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param isPassiveFilterEnabled  passive filter configuration.
 *        - false: passive filter is disabled.
 *        - true : passive filter is enabled.
 */
static inline void port_hal_configure_passive_filter(uint32_t instance, 
                                                     uint32_t pin, 
                                                     bool isPassiveFilterEnabled)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_PFE(instance, pin, isPassiveFilterEnabled);
}

#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
/*!
 * @brief Enable or disable open drain.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param isOpenDrainEnabled  enable open drain or not.
 *        - false: Open Drain output is disabled on the corresponding pin.
 *        - true : Open Drain output is disabled on the corresponding pin.
 */
static inline void port_hal_configure_open_drain(uint32_t instance, 
                                                 uint32_t pin, 
                                                 bool isOpenDrainEnabled)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_ODE(instance, pin, isOpenDrainEnabled);
}
#endif /*FSL_FEATURE_PORT_HAS_OPEN_DRAIN*/

/*!
 * @brief Configure drive strength if the pin is used as digital output.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param driveSelect  drive strength selection.
 *        - kLowDriveStrength : low drive strength is configured.
 *        - kHighDriveStrength: high drive strength is configured.
 */
static inline void port_hal_configure_drive_strength(uint32_t instance, 
                                                     uint32_t pin, 
                                                     port_drive_strength_t driveSelect)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_DSE(instance, pin, driveSelect);
}

/*!
 * @brief Configure pin muxing.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param mux  pin muxing slot selection.
 *        - kPinDisabled: Pin disabled.
 *        - kMuxAsGpio  : Set as GPIO.
 *        - others      : chip-specific.
 */
static inline void port_hal_mux_control(uint32_t instance, uint32_t pin, port_mux_t mux)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_MUX(instance, pin, mux);
}
 
#if FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK
/*!
 * @brief Lock or unlock pin control register bits[15:0].
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param isPinLockEnabled  lock pin control register or not.
 *        - false: pin control register bit[15:0] are not locked.
 *        - true : pin control register bit[15:0] are locked, cannot be updated till system reset.
 */
static inline void port_hal_configure_pin_control_lock(uint32_t instance, 
                                                       uint32_t pin, 
                                                       bool isPinLockEnabled)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);    
    BW_PORT_PCRn_LK(instance, pin, isPinLockEnabled);
}
#endif /* FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK*/

#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
/*!
 * @brief Enable or disable digital filter in one single port.
 *        Each bit of the 32-bit register represents one pin.
 *  
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param isDigitalFilterEnabled  digital filter enable/disable.
 *        - false: digital filter is disabled on the corresponding pin.
 *        - true : digital filter is enabled on the corresponding pin.
 */
static inline void port_hal_configure_digital_filter(uint32_t instance, 
                                                     uint32_t pin,
                                                     bool isDigitalFilterEnabled)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    HW_PORT_DFER_SET(instance, (uint32_t)isDigitalFilterEnabled << pin);
}

/*!
 * @brief Configure clock source for digital input filters. Changing the filter clock source should
 *        only be done after disabling all enabled filters. Every pin in one port uses the same
 *        clock source.
 *
 * @param instance  port instance number. 
 * @param clockSource  chose which clock source to use for current port.
 *        - kBusClock: digital filters are clocked by the bus clock.
 *        - kLPOClock: digital filters are clocked by the 1 kHz LPO clock.
 */
static inline void port_hal_configure_digital_filter_clock(uint32_t instance, 
                                                    port_digital_filter_clock_source_t clockSource)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    HW_PORT_DFCR_WR(instance, clockSource);
}

/*!
 * @brief Configure the maximum size of the glitches(in clock cycles) the digital filter absorbs 
 *        for enabled digital filters. Glitches that are longer than this register setting 
 *        (in clock cycles) will pass through the digital filter, while glitches that are equal
 *        to or less than this register setting (in clock cycles) will be filtered. Changing the
 *        filter length should only be done after disabling all enabled filters.
 *
 * @param instance  port instance number. 
 * @param width  configure digital filter width (should be less than 5 bits). 
 */
static inline void port_hal_configure_digital_filter_width(uint32_t instance, uint8_t width)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    HW_PORT_DFWR_WR(instance, width);
}
#endif /* FSL_FEATURE_PORT_HAS_DIGITAL_FILTER*/

/*!
 * @brief Configure low half of pin control register for the same settings, 
 *        this function operates pin 0 -15 of one specific port.
 * 
 * @param instance  port instance number. 
 * @param pinSelect  update corresponding pin control register or not. For specific bit:
 *        - 0: corresponding low half of pin control register won't be updated according config.
 *        - 1: corresponding low half of pin control register will be updated according config.
 * @param config  value would be written to low half port control register bits[15:0]. 
 */
void port_hal_global_pin_control_low(uint32_t instance, uint16_t lowPinSelect, uint16_t config);

/*!
 * @brief Configure high half of pin control register for the same settings, 
 *        this function operates pin 16 -31 of one specific port.
 * 
 * @param instance  port instance number. 
 * @param pinSelect  update corresponding pin control register or not. For specific bit:
 *        - 0: corresponding high half of pin control register won't be updated according config.
 *        - 1: corresponding high half of pin control register will be updated according config.
 * @param config  value would be written to high half port control register bits[15:0]. 
 */
void port_hal_global_pin_control_high(uint32_t instance, uint16_t highPinSelect, uint16_t config);

/*@}*/

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Configure port pin interrupt/DMA request.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @param intConfig  interrupt configuration.
 *        - kIntDisabled   : interrupt/DMA request disabled.
 *        - kDmaRisingEdge : DMA request on rising edge.
 *        - kDmaFallingEdge: DMA request on falling edge.
 *        - kDmaEitherEdge : DMA request on either edge.
 *        - KIntLogicZero  : Interrupt when logic zero. 
 *        - KIntRisingEdge : Interrupt on rising edge. 
 *        - KIntFallingEdge: Interrupt on falling edge. 
 *        - KIntEitherEdge : Interrupt on either edge. 
 *        - KIntLogicOne   : Interrupt when logic one. 
 */
static inline void port_hal_configure_pin_interrupt(uint32_t instance, 
                                                    uint32_t pin, 
                                                    port_interrupt_config_t intConfig)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);
    BW_PORT_PCRn_IRQC(instance, pin, intConfig);
}

/*!
 * @brief Read individual pin interrupt status flag.
 * 
 * If pin is configured to generate a DMA request then the corresponding flag
 * will be cleared automatically at the completion of the requested DMA transfer,
 * otherwise the flag remains set until a logic one is written to that flag. 
 * If configured for a level sensitive interrupt that remains asserted then flag
 * will set again immediately.
 *
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 * @return current pin interrupt status flag.
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 */
static inline bool port_hal_read_pin_interrupt_flag(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);    
    return BR_PORT_PCRn_ISF(instance, pin);
}

/*!
 * @brief Clear individual pin interrupt status flag.
 * 
 * @param instance  port instance number. 
 * @param pin  port pin number. 
 */
static inline void port_hal_clear_pin_interrupt_flag(uint32_t instance, uint32_t pin)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);
    assert(pin < 32);    
    BW_PORT_PCRn_ISF(instance, pin, 1U);
}

/*!
 * @brief Read entire port interrupt status flag.
 * 
 * @param instance  port instance number. 
 * @return all 32 pin interrupt status flags. For specific bit:
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 */
static inline uint32_t port_hal_read_port_interrupt_flag(uint32_t instance)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);    
    return HW_PORT_ISFR_RD(instance);
}

/*!
 * @brief Clear entire port interrupt status flag.
 * 
 * @param instance  port instance number. 
 */
static inline void port_hal_clear_port_interrupt_flag(uint32_t instance)
{
    assert(instance < HW_PORT_INSTANCE_COUNT);    
    HW_PORT_ISFR_WR(instance, ~0U);
}

/*@}*/

#if defined(__cplusplus)
}
#endif
 
/*! @}*/
 
#endif /* __FSL_PORT_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

