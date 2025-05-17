/*
 * nvic.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "nvic.h"

#include "nvic_registers.h"
#include "scb_registers.h"
#include "types.h"

/*** NVIC linker generated symbols ***/

extern uint32_t __Vectors;

/*** NVIC local macros ***/

#define NVIC_PRIORITY_MIN   3

/*** NVIC functions ***/

/*******************************************************************/
void NVIC_init(void) {
    // Init vector table address.
    SCB->VTOR = (uint32_t) &__Vectors;
}

/*******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Enable interrupt.
    NVIC->ISER = (0b1 << irq_index);
errors:
    return;
}

/*******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Disable interrupt.
    NVIC->ICER = (0b1 << irq_index);
errors:
    return;
}

/*******************************************************************/
void NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Clamp parameter.
    if (priority > NVIC_PRIORITY_MIN) {
        priority = NVIC_PRIORITY_MIN;
    }
    // Reset bits.
    NVIC->IPR[irq_index >> 2] &= ~(0xFF << ((irq_index % 4) << 3));
    // Set priority.
    NVIC->IPR[irq_index >> 2] |= ((priority << 6) << ((irq_index % 4) << 3));
errors:
    return;
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
