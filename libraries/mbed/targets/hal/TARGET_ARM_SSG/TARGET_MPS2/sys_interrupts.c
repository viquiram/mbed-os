/*!
** \file    interrupts.c
** \brief   Interrupt handler
** \date    Copyright (c) 1996-2004 ARM Limited. All rights reserved.
*/

/*
** Revision $Revision: 25763 $
** Checkin $Date: 2009-02-13 14:30:12 +0000 (Fri, 13 Feb 2009) $
*/
#include "common.h"

typedef unsigned long  u32;
typedef unsigned long  const uc32;  /* Read Only */
typedef volatile unsigned long  vu32;
typedef volatile unsigned long  const vuc32;  /* Read Only */

/*------------------------ SystemTick ----------------------------------------*/
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

/*------------------------ Nested Vectored Interrupt Controller --------------*/
typedef struct
{
  vu32 ISER[2];
  u32 RESERVED0[30];
  vu32 ICER[2];
  u32 RSERVED1[30];
  vu32 ISPR[2];
  u32 RESERVED2[30];
  vu32 ICPR[2];
  u32 RESERVED3[30];
  vu32 IABR[2];
  u32 RESERVED4[62];
  vu32 IPR[11];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

/* System Control Space memory map */
#define SCS_BASE              ((u32)0xE000E000)

#define SysTick_BASE          (SCS_BASE + 0x0010)
#define NVIC_BASE             (SCS_BASE + 0x0100)
#define SCB_BASE              (SCS_BASE + 0x0D00)

#define SysTick             ((SysTick_TypeDef *) SysTick_BASE)
#define NVIC                ((NVIC_TypeDef *) NVIC_BASE)
#define SCB                 ((SCB_TypeDef *) SCB_BASE)  

#define BITS_PER_WORD           32
#define NWORD(bitnum)           (bitnum / 32)
#define BIT(bitnum)             (1 << (bitnum % BITS_PER_WORD))

/*!
** \brief   Generic interrupt enable routine
**
** \param   number      Interrupt number
** \param   routine     User routine to handle interrupt
**
** \return  
*/
int _irq_enable (int number, void (*routine)(void))
{
    unsigned int * nvic_vecttbl = (unsigned int *)0xE000ED08;

    HW_REG((*nvic_vecttbl + 0x40) , (0x4 * number)) = (unsigned int) routine; // set up interrupt handler

    if(number < 32)
        NVIC->ISER[0] = 1 << (number & 0x1F);  // enable interrupt
    else
        NVIC->ISER[1] = 1 << (number & 0x1F);  // enable interrupt
    
    return TRUE;
}

/*!
** \brief   Generic interrupt disable routine
**
** \param   number      Interrupt number
**
** \return  
*/
int _irq_disable (int number)
{
    if(number < 32)
        NVIC->ICER[0] = 1 << (number & 0x1F);  // disable interrupt
    else
        NVIC->ICER[1] = 1 << (number & 0x1F);  // disable interrupt

    return TRUE;
}

// Checks whether interrupt for this peripheral is pending. 
int NVIC_check_peripheral(char * const periphName, const int IRQNo, const int asserted)
{
    printf("");
    if(!periphName) {
        debug("Error: Null peripheral name.\n");
        return TRUE;
    }
    
    // Checking whether asserted
    if(asserted) 
    {
//        if(!(NVIC->ICPR[NWORD(IRQNo)] & BIT(IRQNo))) 
        if(!(NVIC->ICPR[0] & (1 << (IRQNo & 0x1F)))) 
        {
            printf("Error: NVIC Pending Status for %s is incorrect. "
                "Read: %#08x, Expected: %#08x\n",periphName, 
                NVIC->ICPR[0] & (1 << (IRQNo & 0x1F)), (1 << (IRQNo & 0x1F)));
//                NVIC->ICPR[NWORD(IRQNo)] & BIT(IRQNo), BIT(IRQNo));
            return TRUE;
        }
        else 
        {
            // its correct
            return FALSE;
        }
    }
    else if(!asserted) 
    {
        if(NVIC->ICPR[0] & (1 << (IRQNo & 0x1F))) 
//        if(NVIC->ICPR[NWORD(IRQNo)] & BIT(IRQNo)) 
        {
            printf("Error: NVIC Pending Status for %s is incorrect. "
                "Read: %#08x, Expected: %#08x\n",periphName,
                NVIC->ICPR[0] & (1 << (IRQNo & 0x1F)), (1 << (IRQNo & 0x1F)));
            return TRUE;
        }
        else 
        {
            // its correct
            return FALSE;
        }
    }
    // Should never come here.
    return TRUE;
}

void NVIC_clear_pending(const int IRQNo)
{
    NVIC->ICPR[0] = (1 << (IRQNo & 0x1F));    
//    NVIC->ICPR[NWORD(IRQNo)] = BIT(IRQNo);    
}

void NVIC_clear_all(void)
{
    NVIC->ICPR[0] = 0xFFFFFFFF;    
}

