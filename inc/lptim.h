/*
 * lptim.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __LPTIM_H__
#define __LPTIM_H__

#include "types.h"

/*** LPTIM structures ***/

/*!******************************************************************
 * \enum LPTIM_status_t
 * \brief LPTIM driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LPTIM_SUCCESS = 0,
	LPTIM_ERROR_DELAY_UNDERFLOW,
	LPTIM_ERROR_DELAY_OVERFLOW,
	LPTIM_ERROR_DELAY_MODE,
	LPTIM_ERROR_ARR_TIMEOUT,
	LPTIM_ERROR_CLOCK_SOURCE,
	// Last base value.
	LPTIM_ERROR_BASE_LAST = 0x0100
} LPTIM_status_t;

/*!******************************************************************
 * \enum LPTIM_delay_mode_t
 * \brief LPTIM delay waiting modes.
 *******************************************************************/
typedef enum {
	LPTIM_DELAY_MODE_ACTIVE = 0,
	LPTIM_DELAY_MODE_SLEEP,
	LPTIM_DELAY_MODE_STOP,
	LPTIM_DELAY_MODE_LAST
} LPTIM_delay_mode_t;

/*** LPTIM functions ***/

/*!******************************************************************
 * \fn LPTIM_status_t LPTIM_init(uint8_t nvic_priority)
 * \brief Init LPTIM peripheral for delay operation.
 * \param[in]  	nvic_priority: Interrupt priority.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPTIM_init(uint8_t nvic_priority);

/*!******************************************************************
 * \fn LPTIM_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode)
 * \brief Delay function.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[in]	delay_mode: Delay waiting mode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPTIM_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode);

/*******************************************************************/
#define LPTIM_exit_error(base) { if (lptim_status != LPTIM_SUCCESS) { status = (base + lptim_status); goto errors; } }

/*******************************************************************/
#define LPTIM_stack_error(base) { if (lptim_status != LPTIM_SUCCESS) { ERROR_stack_add(base + lptim_status); } }

/*******************************************************************/
#define LPTIM_stack_exit_error(base, code) { if (lptim_status != LPTIM_SUCCESS) { ERROR_stack_add(base + lptim_status); status = code; goto errors; } }

#endif /* __LPTIM_H__ */
