/*
 * nvic.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "nvic.h"

#include "nvic_reg.h"
#include "scb_reg.h"
#include "types.h"

/*** NVIC local macros ***/

#define NVIC_PRIORITY_MIN   3

/*** NVIC local global variables ***/

extern uint32_t __Vectors;

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
    NVIC->IPR[(irq_index >> 2)] &= ~(0xFF << (8 * (irq_index % 4)));
    // Set priority.
    NVIC->IPR[(irq_index >> 2)] |= ((priority << 6) << (8 * (irq_index % 4)));
errors:
    return;
}
