/*!
** \file    interrupts.h
** \brief   Interrupt handler
** \date    Copyright (c) 1996-2004 ARM Limited. All rights reserved.
*/

/*
** Revision $Revision: 25763 $
** Checkin $Date: 2009-02-13 14:30:12 +0000 (Fri, 13 Feb 2009) $
*/

#ifndef _SYS_INTERRUPTS_H
#define _SYS_INTERRUPTS_H

#include "sys_msg.h"
#include "sys_macros.h"
#include "sys_hardware.h"
#include "sys_platform.h"

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
  vuc32 CPUID;      // 0x00 -
  vu32 ICSR;        // 0x04 -
  vu32 VTOR;        // 0x08 -
  vu32 AIRCR;       // 0x0C -
  vu32 SCR;         // 0x10 -
  vu32 CCR;         // 0x14 -
  vu32 SHPR[3];     // (0x18 - 0x20) -
  vu32 SHCSR;       // 0x24 - System Handler Control and Status Register
  vu32 CFSR;        // 0x28 - Usage Fault Status Register (upper half word)
                    //      - Bus Fault Status Register (second byte)
                    //      - Memory Management Fault Status register (lowest byte)
  vu32 HFSR;        // 0x2C - Hard Fault Status Register
  vu32 DFSR;        // 0x30
  vu32 MMFAR;       // 0x34 - Memory Management Fault Address Register
  vu32 BFAR;        // 0x38 - Bus Fault Address Register
  vu32 AFSR;        // 0x3C
} SCB_TypeDef;

/* System Control Space memory map */
#define SCS_BASE              ((u32)0xE000E000)

#define SysTick_BASE          (SCS_BASE + 0x0010)
#define NVIC_BASE             (SCS_BASE + 0x0100)
#define SCB_BASE              (SCS_BASE + 0x0D00)

#define SysTick             ((SysTick_TypeDef *) SysTick_BASE)
#define NVIC                ((NVIC_TypeDef *) NVIC_BASE)
#define SCB                 ((SCB_TypeDef *) SCB_BASE)  

typedef enum
{
    USG_FAULT = 18,
    BUS_FAULT = 17,
    MEM_FAULT = 16,    
}
_exception_t;


msg_t _fault_enable (_exception_t number);
msg_t _fault_disable (_exception_t number);

void SetupBusFault_handler(void);
void SetupUsageFault_handler(void);

#endif
