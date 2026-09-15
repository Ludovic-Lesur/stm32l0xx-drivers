/*
 * nvm.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __NVM_H__
#define __NVM_H__

#include "error.h"
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
    NVM_ERROR_DATA_TYPE,
    NVM_ERROR_DATA_SIZE,
    NVM_ERROR_ADDRESS,
    NVM_ERROR_UNLOCK_READY,
    NVM_ERROR_UNLOCK_SEQUENCE,
    NVM_ERROR_READ_READY,
    NVM_ERROR_WRITE_READY,
    NVM_ERROR_WRITE_OPERATION,
    NVM_ERROR_WRITE_COMPLETION,
    NVM_ERROR_WRITE_VERIFY,
    // Last base value.
    NVM_ERROR_BASE_LAST = ERROR_BASE_STEP
} NVM_status_t;

/*!******************************************************************
 * \enum NVM_data_type_t
 * \brief NVM data types list.
 *******************************************************************/
typedef enum {
    NVM_DATA_TYPE_BYTE = 0,
    NVM_DATA_TYPE_SHORT,
    NVM_DATA_TYPE_LONG,
    NVM_DATA_TYPE_LAST
} NVM_data_type_t;

/*** NVM functions ***/

/*!******************************************************************
 * \fn NVM_status_t NVM_read_byte(uint32_t address, uint8_t* data)
 * \brief Read byte in NVM.
 * \param[in]   address: Relative address to read (starting from 0).
 * \param[out]  data: Pointer to byte that will contain the read value.
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_read(uint32_t address, void* data, uint8_t data_size, NVM_data_type_t data_type);

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

#endif /* STM32L0XX_DRIVERS_DISABLE */
