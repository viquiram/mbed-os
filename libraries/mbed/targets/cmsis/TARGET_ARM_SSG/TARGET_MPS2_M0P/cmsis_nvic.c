/* mbed Microcontroller Library - cmsis_nvic for MPS2
 * Copyright (c) 2009-2011 ARM Limited. All rights reserved.
 *
 * CMSIS-style functionality to support dynamic vectors
 */ 
#include "cmsis_nvic.h"

#define NVIC_RAM_VECTOR_ADDRESS   (0x20000000)  // Location of vectors in RAM
#define NVIC_FLASH_VECTOR_ADDRESS (0x00000000)  // Initial vector position in flash

void NVIC_SetVector(IRQn_Type IRQn, uint32_t vector) {
   // int i;
    // Space for dynamic vectors, initialised to allocate in R/W
    static volatile uint32_t* vectors = (uint32_t*)NVIC_RAM_VECTOR_ADDRESS;

    // Set the vector 
    vectors[IRQn + 16] = vector; 
}

uint32_t NVIC_GetVector(IRQn_Type IRQn) {
    // We can always read vectors at 0x0, as the addresses are remapped
    uint32_t *vectors = (uint32_t*)0; 

    // Return the vector
    return vectors[IRQn + 16];
}
