/*
 * fault.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "pwr.h"

/*******************************************************************/
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
