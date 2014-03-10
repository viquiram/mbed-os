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
#include "fsl_os_abstraction.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined (K64F12_SERIES)
IRQn_Type flexcan_irq_ids[HW_CAN_INSTANCE_COUNT][FSL_CAN_INTERRUPT_COUNT] = 
{
    {CAN0_ORed_Message_buffer_IRQn, CAN0_Bus_Off_IRQn, CAN0_Error_IRQn, CAN0_Wake_Up_IRQn}
};
#elif defined (K70F12_SERIES)
IRQn_Type flexcan_irq_ids[HW_CAN_INSTANCE_COUNT][FSL_CAN_INTERRUPT_COUNT] = 
{
    {CAN0_ORed_Message_buffer_IRQn, CAN0_Bus_Off_IRQn, CAN0_Error_IRQn, CAN0_Wake_Up_IRQn},
    {CAN1_ORed_Message_buffer_IRQn, CAN1_Bus_Off_IRQn, CAN1_Error_IRQn, CAN1_Wake_Up_IRQn}
};
#endif

extern bool int_mb;
extern bool int_fifo;
extern uint32_t rx_mb_idx;
extern sync_object_t irqSync;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Interrupt handler for a FlexCAN instance.
 *
 * @param   instance    The FlexCAN instance number.
 */
void flexcan_irq_handler(uint8_t instance);

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_ORed_Message_buffer_IRQHandler(void);

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_ORed_Message_buffer_IRQHandler(void);

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Bus_Off_IRQHandler(void);

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Bus_Off_IRQHandler(void);

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Error_IRQHandler(void);

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Error_IRQHandler(void);

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Wake_Up_IRQHandler(void);

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Wake_Up_IRQHandler(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Interrupt handler for a FlexCAN instance.
 *
 * @param   instance    The FlexCAN instance number.
 */
void flexcan_irq_handler(uint8_t instance)
{
    volatile uint32_t             tmp_reg;
    uint32_t temp;

    /* Get the interrupt flag*/
    tmp_reg = (flexcan_hal_get_all_mb_int_flags(instance)) & CAN_IMASK1_BUFLM_MASK;

    /* Check Tx/Rx interrupt flag and clear the interrupt*/
    if(tmp_reg){
        if (tmp_reg & 0x20)
        {
            int_fifo = true;
        }
        
        temp = 1 << rx_mb_idx;

        if (tmp_reg & temp)
        {
            int_mb = true;
        }

        /* Clear the interrupt and unlock message buffer*/
        sync_signal_from_isr(&irqSync);
        flexcan_hal_clear_mb_int_flag(instance, tmp_reg);
        flexcan_hal_unlock_rx_mb(instance);
   }

    /* Clear all other interrupts in ERRSTAT register (Error, Busoff, Wakeup)*/
    flexcan_hal_clear_err_interrupt_status(instance);

    return;
}

/*! @brief CAN IRQ handler with the same name in startup code*/
#if defined (K64F12_SERIES)
/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Bus_Off_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Error_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Wake_Up_IRQHandler(void)
{
    flexcan_irq_handler(0);
}
#elif defined (K70F12_SERIES)
/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    flexcan_irq_handler(1);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Bus_Off_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Bus_Off_IRQHandler(void)
{
    flexcan_irq_handler(1);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Error_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Error_IRQHandler(void)
{
    flexcan_irq_handler(1);
}

/*! @brief Implementation of CAN0 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN0_Wake_Up_IRQHandler(void)
{
    flexcan_irq_handler(0);
}

/*! @brief Implementation of CAN1 handler named in startup code. */
/*!*/
/*! Passes instance to generic FlexCAN IRQ handler.*/
void CAN1_Wake_Up_IRQHandler(void)
{
    flexcan_irq_handler(1);
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/

