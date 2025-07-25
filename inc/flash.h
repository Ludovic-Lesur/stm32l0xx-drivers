/*
 * flash.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __FLASH_H__
#define __FLASH_H__

#include "error.h"
#include "types.h"

/*** FLASH structures ***/

/*!******************************************************************
 * \enum FLASH_status_t
 * \brief FLASH driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    FLASH_SUCCESS = 0,
    FLASH_ERROR_LATENCY,
    FLASH_ERROR_TIMEOUT,
    // Last base value.
    FLASH_ERROR_BASE_LAST = ERROR_BASE_STEP
} FLASH_status_t;

/*** FLASH functions ***/

/*!******************************************************************
 * \fn FLASH_status_t FLASH_set_latency(uint8_t wait_states)
 * \brief Set FLASH latency.
 * \param[in]   wait_states: Number of wait states to set.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states);

/*******************************************************************/
#define FLASH_exit_error(base) { ERROR_check_exit(flash_status, FLASH_SUCCESS, base) }

/*******************************************************************/
#define FLASH_stack_error(base) { ERROR_check_stack(flash_status, FLASH_SUCCESS, base) }

/*******************************************************************/
#define FLASH_stack_exit_error(base, code) { ERROR_check_stack_exit(flash_status, FLASH_SUCCESS, base, code) }

#endif /* __FLASH_H__ */

#endif /* STM32L0XX_DRIVERS_DISABLE */
