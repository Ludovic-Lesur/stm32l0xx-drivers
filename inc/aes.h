/*
 * aes.h
 *
 *  Created on: 01 aug. 2018
 *      Author: Ludo
 */

#ifndef __AES_H__
#define __AES_H__

#include "error.h"
#include "types.h"

/*** AES structures ***/

/*!******************************************************************
 * \enum AES_status_t
 * \brief AES driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    AES_SUCCESS = 0,
    AES_ERROR_UNINITIALIZED,
    AES_ERROR_NULL_PARAMETER,
    AES_ERROR_TIMEOUT,
    // Last base value.
    AES_ERROR_BASE_LAST = ERROR_BASE_STEP
} AES_status_t;

/*** AES functions ***/

/*!******************************************************************
 * \fn void AES_init(void)
 * \brief Init AES peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void AES_init(void);

/*!******************************************************************
 * \fn void AES_de_init(void)
 * \brief Release AES peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void AES_de_init(void);

/*!******************************************************************
 * \fn AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* key)
 * \brief Compute AES-128.
 * \param[in]   data_in: Input data.
 * \param[in]   key: AES key.
 * \param[out]  data_out: Output data.
 * \retval      Function execution status.
 *******************************************************************/
AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* key);

/*******************************************************************/
#define AES_exit_error(base) { ERROR_check_exit(aes_status, AES_SUCCESS, base) }

/*******************************************************************/
#define AES_stack_error(base) { ERROR_check_stack(aes_status, AES_SUCCESS, base) }

/*******************************************************************/
#define AES_stack_exit_error(base, code) { ERROR_check_stack_exit(aes_status, AES_SUCCESS, base, code) }

#endif /* __AES_H__ */
