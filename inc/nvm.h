/*
 * nvm.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __NVM_H__
#define __NVM_H__

#include "flash_reg.h"
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
	NVM_ERROR_ADDRESS,
	NVM_ERROR_UNLOCK,
	NVM_ERROR_LOCK,
	NVM_ERROR_READ,
	NVM_ERROR_WRITE,
	// Last base value.
	NVM_ERROR_BASE_LAST = 0x0100
} NVM_status_t;

/*!******************************************************************
 * \enum NVM_address_t
 * \brief NVM address range.
 *******************************************************************/
typedef enum {
	NVM_ADDRESS_FIRST = 0,
	NVM_ADDRESS_LAST = (EEPROM_SIZE_BYTES - 1)
} NVM_address_t;

/*** NVM functions ***/

/*!******************************************************************
 * \fn NVM_status_t NVM_read_byte(NVM_address_t address, uint8_t* data)
 * \brief Read byte in NVM.
 * \param[in]  	address: Address to read.
 * \param[out] 	data: Pointer to byte that will contain the read value.
 * \retval		Function execution status.
 *******************************************************************/
NVM_status_t NVM_read_byte(NVM_address_t address, uint8_t* data);

/*!******************************************************************
 * \fn NVM_status_t NVM_write_byte(NVM_address_t address, uint8_t data)
 * \brief Write byte in NVM.
 * \param[in]  	address: Address to write.
 * \param[out] 	data: Byte to write.
 * \retval		Function execution status.
 *******************************************************************/
NVM_status_t NVM_write_byte(NVM_address_t address, uint8_t data);

/*******************************************************************/
#define NVM_exit_error(base) { if (nvm_status != NVM_SUCCESS) { status = (base + nvm_status); goto errors; } }

/*******************************************************************/
#define NVM_stack_error(base) { if (nvm_status != NVM_SUCCESS) { ERROR_stack_add(base + nvm_status); } }

/*******************************************************************/
#define NVM_stack_exit_error(base, code) { if (nvm_status != NVM_SUCCESS) { ERROR_stack_add(base + nvm_status); status = code; goto errors; } }

#endif /* __NVM_H__ */
