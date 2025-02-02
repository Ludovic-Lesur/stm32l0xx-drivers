/*
 * nvm.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __NVM_H__
#define __NVM_H__

#include "types.h"

/*** NVM structures ***/

/*!******************************************************************
 * \enum NVM_status_t
 * \brief NVM driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    NVM_SUCCESS = 0,
    NVM_ERROR_NULL_PARAMETER,
    NVM_ERROR_OVERFLOW,
    NVM_ERROR_ADDRESS,
    NVM_ERROR_UNLOCK,
    NVM_ERROR_LOCK,
    NVM_ERROR_READ,
    NVM_ERROR_WRITE,
    // Last base value.
    NVM_ERROR_BASE_LAST = 0x0100
} NVM_status_t;

/*** NVM functions ***/

/*!******************************************************************
 * \fn NVM_status_t NVM_read_byte(uint32_t address, uint8_t* data)
 * \brief Read byte in NVM.
 * \param[in]   address: Relative address to read (starting from 0).
 * \param[out]  data: Pointer to byte that will contain the read value.
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_read_byte(uint32_t address, uint8_t* data);

/*!******************************************************************
 * \fn NVM_status_t NVM_write_byte(uint32_t address, uint8_t data)
 * \brief Write byte in NVM.
 * \param[in]   address: Relative address to write (starting from 0).
 * \param[out]  data: Byte to write.
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_write_byte(uint32_t address, uint8_t data);

/*******************************************************************/
#define NVM_exit_error(base) { ERROR_check_exit(nvm_status, NVM_SUCCESS, base) }

/*******************************************************************/
#define NVM_stack_error(base) { ERROR_check_stack(nvm_status, NVM_SUCCESS, base) }

/*******************************************************************/
#define NVM_stack_exit_error(base, code) { ERROR_check_stack_exit(nvm_status, NVM_SUCCESS, base, code) }

#endif /* __NVM_H__ */
