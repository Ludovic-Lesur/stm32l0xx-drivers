/*
 * flash.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "flash.h"

#include "flash_registers.h"
#include "types.h"

/*** FLASH local macros ***/

#define FLASH_WAIT_STATES_MAX   1
#define FLASH_TIMEOUT_COUNT     100000

/*** FLASH functions ***/

/*******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint32_t loop_count = 0;
    // Check parameter.
    if (wait_states > FLASH_WAIT_STATES_MAX) {
        status = FLASH_ERROR_LATENCY;
        goto errors;
    }
    // Configure number of wait states.
    FLASH->ACR &= ~(0b1 << 0); // Reset bit.
    FLASH->ACR |= wait_states; // Set latency.
    // Wait until configuration is done.
    while (((FLASH->ACR) & (0b1 << 0)) != wait_states) {
        loop_count++;
        if (loop_count > FLASH_TIMEOUT_COUNT) {
            status = FLASH_ERROR_TIMEOUT;
            goto errors;
        }
    }
errors:
    return status;
}
