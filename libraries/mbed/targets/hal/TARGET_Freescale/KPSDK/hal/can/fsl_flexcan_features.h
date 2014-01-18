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
#if !defined(__FSL_CAN_FEATURES_H__)
#define __FSL_CAN_FEATURES_H__

#if defined(CPU_MK10DN512VLK10) || defined(CPU_MK10DN512VLL10) || defined(CPU_MK10DX128VLQ10) || defined(CPU_MK10DX256VLQ10) || \
    defined(CPU_MK10DN512VLQ10) || defined(CPU_MK10DN512VMB10) || defined(CPU_MK10DN512VMC10) || defined(CPU_MK10DX128VMD10) || \
    defined(CPU_MK10DX256VMD10) || defined(CPU_MK10DN512VMD10) || defined(CPU_MK20DN512VLK10) || defined(CPU_MK20DN512VLL10) || \
    defined(CPU_MK20DX128VLQ10) || defined(CPU_MK20DX256VLQ10) || defined(CPU_MK20DN512VLQ10) || defined(CPU_MK20DN512VMB10) || \
    defined(CPU_MK20DX256VMC10) || defined(CPU_MK20DN512VMC10) || defined(CPU_MK20DX128VMD10) || defined(CPU_MK20DX256VMD10) || \
    defined(CPU_MK20DN512VMD10) || defined(CPU_MK30DN512VLK10) || defined(CPU_MK30DN512VLL10) || defined(CPU_MK30DX128VLQ10) || \
    defined(CPU_MK30DX256VLQ10) || defined(CPU_MK30DN512VLQ10) || defined(CPU_MK30DN512VMB10) || defined(CPU_MK30DN512VMC10) || \
    defined(CPU_MK30DX128VMD10) || defined(CPU_MK30DX256VMD10) || defined(CPU_MK30DN512VMD10) || defined(CPU_MK40DN512VLK10) || \
    defined(CPU_MK40DN512VLL10) || defined(CPU_MK40DX128VLQ10) || defined(CPU_MK40DX256VLQ10) || defined(CPU_MK40DN512VLQ10) || \
    defined(CPU_MK40DN512VMB10) || defined(CPU_MK40DN512VMC10) || defined(CPU_MK40DX128VMD10) || defined(CPU_MK40DX256VMD10) || \
    defined(CPU_MK40DN512VMD10) || defined(CPU_MK60DN256VLL10) || defined(CPU_MK60DX256VLL10) || defined(CPU_MK60DN512VLL10) || \
    defined(CPU_MK60DN256VLQ10) || defined(CPU_MK60DX256VLQ10) || defined(CPU_MK60DN512VLQ10) || defined(CPU_MK60DN256VMC10) || \
    defined(CPU_MK60DX256VMC10) || defined(CPU_MK60DN512VMC10) || defined(CPU_MK60DN256VMD10) || defined(CPU_MK60DX256VMD10) || \
    defined(CPU_MK60DN512VMD10)
    /* @brief Message buffer size*/
    #define FSL_FEATURE_CAN_HAS_MESSAGE_BUFFER_MAX_NUMBER (16)
    /* @brief Has doze mode support (register bit field MCR[DOZE]).*/
    #define FSL_FEATURE_CAN_HAS_DOZE_MODE_SUPPORT (0)
    /* @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]).*/
    #define FSL_FEATURE_CAN_HAS_GLITCH_FILTER (1)
    /* @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2).*/
    #define FSL_FEATURE_CAN_HAS_EXTENDED_FLAG_REGISTER (0)
    /* @brief Has separate message buffer 0 interrupt flag (register bit field IFLAG1[BUF0I]).*/
    #define FSL_FEATURE_CAN_HAS_SEPARATE_BUFFER_0_FLAG (0)
    /* @brief Number of interrupts.*/
    #define FSL_FEATURE_CAN_INTERRUPT_COUNT (4)
#elif defined(CPU_MK10DX64VLH7) || defined(CPU_MK10DX128VLH7) || defined(CPU_MK10DX256VLH7) || defined(CPU_MK10DX64VLK7) || \
    defined(CPU_MK10DX128VLK7) || defined(CPU_MK10DX256VLK7) || defined(CPU_MK10DX128VLL7) || defined(CPU_MK10DX256VLL7) || \
    defined(CPU_MK10DX64VMB7) || defined(CPU_MK10DX128VMB7) || defined(CPU_MK10DX256VMB7) || defined(CPU_MK10DX128VML7) || \
    defined(CPU_MK10DX256VML7) || defined(CPU_MK10FN1M0VLQ12) || defined(CPU_MK10FX512VLQ12) || defined(CPU_MK10FN1M0VMD12) || \
    defined(CPU_MK10FX512VMD12) || defined(CPU_MK20DX64VLH7) || defined(CPU_MK20DX128VLH7) || defined(CPU_MK20DX256VLH7) || \
    defined(CPU_MK20DX64VLK7) || defined(CPU_MK20DX128VLK7) || defined(CPU_MK20DX256VLK7) || defined(CPU_MK20DX128VLL7) || \
    defined(CPU_MK20DX256VLL7) || defined(CPU_MK20DX64VMB7) || defined(CPU_MK20DX128VMB7) || defined(CPU_MK20DX256VMB7) || \
    defined(CPU_MK20DX128VML7) || defined(CPU_MK20DX256VML7) || defined(CPU_MK20FN1M0VLQ12) || defined(CPU_MK20FX512VLQ12) || \
    defined(CPU_MK20FN1M0VMD12) || defined(CPU_MK20FX512VMD12) || defined(CPU_MK30DX64VLH7) || defined(CPU_MK30DX128VLH7) || \
    defined(CPU_MK30DX256VLH7) || defined(CPU_MK30DX64VLK7) || defined(CPU_MK30DX128VLK7) || defined(CPU_MK30DX256VLK7) || \
    defined(CPU_MK30DX128VLL7) || defined(CPU_MK30DX256VLL7) || defined(CPU_MK30DX64VMB7) || defined(CPU_MK30DX128VMB7) || \
    defined(CPU_MK30DX256VMB7) || defined(CPU_MK30DX128VML7) || defined(CPU_MK30DX256VML7) || defined(CPU_MK40DX64VLH7) || \
    defined(CPU_MK40DX128VLH7) || defined(CPU_MK40DX256VLH7) || defined(CPU_MK40DX64VLK7) || defined(CPU_MK40DX128VLK7) || \
    defined(CPU_MK40DX256VLK7) || defined(CPU_MK40DX128VLL7) || defined(CPU_MK40DX256VLL7) || defined(CPU_MK40DX64VMB7) || \
    defined(CPU_MK40DX128VMB7) || defined(CPU_MK40DX256VMB7) || defined(CPU_MK40DX128VML7) || defined(CPU_MK40DX256VML7) || \
    defined(CPU_MK60FN1M0VLQ12) || defined(CPU_MK60FX512VLQ12) || defined(CPU_MK60FN1M0VLQ15) || defined(CPU_MK60FX512VLQ15) || \
    defined(CPU_MK60FN1M0VMD12) || defined(CPU_MK60FX512VMD12) || defined(CPU_MK60FN1M0VMD15) || defined(CPU_MK60FX512VMD15) || \
    defined(CPU_MK61FN1M0VMD12) || defined(CPU_MK61FX512VMD12) || defined(CPU_MK61FN1M0VMD15) || defined(CPU_MK61FX512VMD15) || \
    defined(CPU_MK61FN1M0VMD12WS) || defined(CPU_MK61FX512VMD12WS) || defined(CPU_MK61FN1M0VMD15WS) || defined(CPU_MK61FX512VMD15WS) || \
    defined(CPU_MK61FN1M0VMF12) || defined(CPU_MK61FX512VMF12) || defined(CPU_MK61FN1M0VMF15) || defined(CPU_MK61FX512VMF15) || \
    defined(CPU_MK61FN1M0VMJ12) || defined(CPU_MK61FX512VMJ12) || defined(CPU_MK61FN1M0VMJ15) || defined(CPU_MK61FX512VMJ15) || \
    defined(CPU_MK61FN1M0VMJ12WS) || defined(CPU_MK61FX512VMJ12WS) || defined(CPU_MK61FN1M0VMJ15WS) || defined(CPU_MK61FX512VMJ15WS) || \
    defined(CPU_MK70FN1M0VMF12) || defined(CPU_MK70FX512VMF12) || defined(CPU_MK70FN1M0VMF15) || defined(CPU_MK70FX512VMF15) || \
    defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12) || defined(CPU_MK70FN1M0VMJ15) || defined(CPU_MK70FX512VMJ15) || \
    defined(CPU_MK70FN1M0VMJ12WS) || defined(CPU_MK70FX512VMJ12WS) || defined(CPU_MK70FN1M0VMJ15WS) || defined(CPU_MK70FX512VMJ15WS)
    /* @brief Message buffer size*/
    #define FSL_FEATURE_CAN_HAS_MESSAGE_BUFFER_MAX_NUMBER (16)
    /* @brief Has doze mode support (register bit field MCR[DOZE]).*/
    #define FSL_FEATURE_CAN_HAS_DOZE_MODE_SUPPORT (0)
    /* @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]).*/
    #define FSL_FEATURE_CAN_HAS_GLITCH_FILTER (0)
    /* @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2).*/
    #define FSL_FEATURE_CAN_HAS_EXTENDED_FLAG_REGISTER (1)
    /* @brief Has separate message buffer 0 interrupt flag (register bit field IFLAG1[BUF0I]).*/
    #define FSL_FEATURE_CAN_HAS_SEPARATE_BUFFER_0_FLAG (0)
    /* @brief Number of interrupts.*/
    #define FSL_FEATURE_CAN_INTERRUPT_COUNT (4)
#elif defined(CPU_MK10DN512ZVLK10) || defined(CPU_MK10DN512ZVLL10) || defined(CPU_MK10DN512ZVLQ10) || defined(CPU_MK10DX256ZVLQ10) || \
    defined(CPU_MK10DX128ZVLQ10) || defined(CPU_MK10DN512ZVMB10) || defined(CPU_MK10DN512ZVMC10) || defined(CPU_MK10DN512ZVMD10) || \
    defined(CPU_MK10DX256ZVMD10) || defined(CPU_MK10DX128ZVMD10) || defined(CPU_MK20DN512ZVLK10) || defined(CPU_MK20DX256ZVLK10) || \
    defined(CPU_MK20DN512ZVLL10) || defined(CPU_MK20DX256ZVLL10) || defined(CPU_MK20DN512ZVLQ10) || defined(CPU_MK20DX256ZVLQ10) || \
    defined(CPU_MK20DX128ZVLQ10) || defined(CPU_MK20DN512ZVMB10) || defined(CPU_MK20DX256ZVMB10) || defined(CPU_MK20DN512ZVMC10) || \
    defined(CPU_MK20DX256ZVMC10) || defined(CPU_MK20DN512ZVMD10) || defined(CPU_MK20DX256ZVMD10) || defined(CPU_MK20DX128ZVMD10) || \
    defined(CPU_MK30DN512ZVLK10) || defined(CPU_MK30DN512ZVLL10) || defined(CPU_MK30DN512ZVLQ10) || defined(CPU_MK30DX256ZVLQ10) || \
    defined(CPU_MK30DX128ZVLQ10) || defined(CPU_MK30DN512ZVMB10) || defined(CPU_MK30DN512ZVMC10) || defined(CPU_MK30DN512ZVMD10) || \
    defined(CPU_MK30DX256ZVMD10) || defined(CPU_MK30DX128ZVMD10) || defined(CPU_MK40DN512ZVLK10) || defined(CPU_MK40DN512ZVLL10) || \
    defined(CPU_MK40DN512ZVLQ10) || defined(CPU_MK40DX256ZVLQ10) || defined(CPU_MK40DX128ZVLQ10) || defined(CPU_MK40DN512ZVMB10) || \
    defined(CPU_MK40DN512ZVMC10) || defined(CPU_MK40DN512ZVMD10) || defined(CPU_MK40DX256ZVMD10) || defined(CPU_MK40DX128ZVMD10) || \
    defined(CPU_MK60DN512ZVLL10) || defined(CPU_MK60DX256ZVLL10) || defined(CPU_MK60DN256ZVLL10) || defined(CPU_MK60DN512ZVLQ10) || \
    defined(CPU_MK60DX256ZVLQ10) || defined(CPU_MK60DN256ZVLQ10) || defined(CPU_MK60DN512ZVMC10) || defined(CPU_MK60DX256ZVMC10) || \
    defined(CPU_MK60DN256ZVMC10) || defined(CPU_MK60DN512ZVMD10) || defined(CPU_MK60DX256ZVMD10) || defined(CPU_MK60DN256ZVMD10)
    /* @brief Message buffer size*/
    #define FSL_FEATURE_CAN_HAS_MESSAGE_BUFFER_MAX_NUMBER (16)
    /* @brief Has doze mode support (register bit field MCR[DOZE]).*/
    #define FSL_FEATURE_CAN_HAS_DOZE_MODE_SUPPORT (1)
    /* @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]).*/
    #define FSL_FEATURE_CAN_HAS_GLITCH_FILTER (0)
    /* @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2).*/
    #define FSL_FEATURE_CAN_HAS_EXTENDED_FLAG_REGISTER (1)
    /* @brief Has separate message buffer 0 interrupt flag (register bit field IFLAG1[BUF0I]).*/
    #define FSL_FEATURE_CAN_HAS_SEPARATE_BUFFER_0_FLAG (0)
    /* @brief Number of interrupts.*/
    #define FSL_FEATURE_CAN_INTERRUPT_COUNT (4)
#elif defined(CPU_MK21FX512VLQ12) || defined(CPU_MK21FN1M0VLQ12) || defined(CPU_MK21FX512VLQ12WS) || defined(CPU_MK21FN1M0VLQ12WS) || \
    defined(CPU_MK21FX512VMC12) || defined(CPU_MK21FN1M0VMC12) || defined(CPU_MK21FX512VMC12WS) || defined(CPU_MK21FN1M0VMC12WS) || \
    defined(CPU_MK21FX512VMD12) || defined(CPU_MK21FN1M0VMD12) || defined(CPU_MK21FX512VMD12WS) || defined(CPU_MK21FN1M0VMD12WS) || \
    defined(CPU_MK22FX512VLH12) || defined(CPU_MK22FN1M0VLH12) || defined(CPU_MK22FX512VLK12) || defined(CPU_MK22FN1M0VLK12) || \
    defined(CPU_MK22FX512VLL12) || defined(CPU_MK22FN1M0VLL12) || defined(CPU_MK22FX512VLQ12) || defined(CPU_MK22FN1M0VLQ12) || \
    defined(CPU_MK22FX512VMC12) || defined(CPU_MK22FN1M0VMC12) || defined(CPU_MK22FX512VMD12) || defined(CPU_MK22FN1M0VMD12) || \
    defined(CPU_MK63FN1M0VMD12) || defined(CPU_MK63FN1M0VMD12WS) || defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK64FX512VMD12)
    /* @brief Message buffer size*/
    #define FSL_FEATURE_CAN_HAS_MESSAGE_BUFFER_MAX_NUMBER (16)
    /* @brief Has doze mode support (register bit field MCR[DOZE]).*/
    #define FSL_FEATURE_CAN_HAS_DOZE_MODE_SUPPORT (0)
    /* @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]).*/
    #define FSL_FEATURE_CAN_HAS_GLITCH_FILTER (1)
    /* @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2).*/
    #define FSL_FEATURE_CAN_HAS_EXTENDED_FLAG_REGISTER (0)
    /* @brief Has separate message buffer 0 interrupt flag (register bit field IFLAG1[BUF0I]).*/
    #define FSL_FEATURE_CAN_HAS_SEPARATE_BUFFER_0_FLAG (1)
    /* @brief Number of interrupts.*/
    #define FSL_FEATURE_CAN_INTERRUPT_COUNT (4)
#else
    #error "No valid CPU defined!"
#endif

#endif /* __FSL_CAN_FEATURES_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

