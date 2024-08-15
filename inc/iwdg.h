/*
 * iwdg.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __IWDG_H__
#define __IWDG_H__

#include "types.h"

/*** IWDG macros ***/

// Based on worst case 56kHz LSI clock frequency, minimum IWDG period is 18 seconds.
// Adding 3 second margin to perform reload operation, the maximum free delay is limited to 15 seconds.
#define IWDG_FREE_DELAY_SECONDS_MAX		15

/*** IWDG structures ***/

/*!******************************************************************
 * \enum IWDG_status_t
 * \brief IWDG driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	IWDG_SUCCESS = 0,
	IWDG_ERROR_TIMEOUT,
	// Last base value.
	IWDG_ERROR_BASE_LAST = 0x0100
} IWDG_status_t;

/*** IWDG functions ***/

/*!******************************************************************
 * \fn IWDG_status_t IWDG_init(void)
 * \brief Start independent watchdog.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
IWDG_status_t IWDG_init(void);

/*!******************************************************************
 * \fn void IWDG_reload(void)
 * \brief Refresh independent watchdog.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
void IWDG_reload(void);

/*******************************************************************/
#define IWDG_exit_error(base) { if (iwdg_status != IWDG_SUCCESS) { status = (base + iwdg_status); goto errors; } }

/*******************************************************************/
#define IWDG_stack_error(base) { if (iwdg_status != IWDG_SUCCESS) { ERROR_stack_add(base + iwdg_status); } }

/*******************************************************************/
#define IWDG_stack_exit_error(base, code) { if (iwdg_status != IWDG_SUCCESS) { ERROR_stack_add(base + iwdg_status); status = code; goto errors; } }

#endif /* __IWDG_H__ */
