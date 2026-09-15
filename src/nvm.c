/*
 * nvm.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "nvm.h"

#include "flash_registers.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "types.h"

/*** NVM linker generated symbols ***/

extern uint32_t __eeprom_address__;
extern uint32_t __eeprom_size_bytes__;

/*** NVM local macros ***/

#define NVM_EEPROM_ADDRESS      ((uint32_t) &__eeprom_address__)
#define NVM_EEPROM_SIZE_BYTES   ((uint32_t) &__eeprom_size_bytes__)

#define NVM_ERROR_FLAGS_MASK    0x00032F00

#define NVM_TIMEOUT_COUNT       1000000

/*** NVM local functions ***/

/*******************************************************************/
#define _NVM_check_parameters(void) { \
    /* Check parameters */ \
    if (address >= NVM_EEPROM_SIZE_BYTES) { \
        status = NVM_ERROR_ADDRESS; \
        goto end; \
    } \
    if (data == NULL) { \
        status = NVM_ERROR_NULL_PARAMETER; \
        goto end; \
    } \
    if (data_size == 0) { \
        status = NVM_ERROR_DATA_SIZE; \
        goto end; \
    } \
    if (data_type >= NVM_DATA_TYPE_LAST) { \
        status = NVM_ERROR_DATA_TYPE; \
        goto end; \
    } \
}

/*******************************************************************/
#define _NVM_read(type) { \
    /* Read data */ \
    ((type*) data)[idx] = *((type*) (absolute_address)); \
}

/*******************************************************************/
#define _NVM_write_verify(type) { \
    /* Write data */ \
    (*((type*) (absolute_address))) = ((type*) data)[idx]; \
    /* Wait the end of operation */ \
    status = _NVM_check_busy(NVM_ERROR_WRITE_COMPLETION); \
    if (status != NVM_SUCCESS) goto errors; \
    /* Verify write operation */ \
    if ((*((type*) (absolute_address))) != ((type*) data)[idx]) { \
        status = NVM_ERROR_WRITE_VERIFY; \
        goto errors; \
    } \
}

/*******************************************************************/
static NVM_status_t _NVM_check_busy(NVM_status_t timeout_error_code) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    uint32_t loop_count = 0;
    // Check no write/erase operation is running.
    while (((FLASH->SR) & (0b1 << 0)) != 0) {
        // Wait till BSY='1' or timeout.
        loop_count++;
        if (loop_count > NVM_TIMEOUT_COUNT) {
            // Exit with error.
            status = timeout_error_code;
            goto errors;
        }
    }
    // Check end of operation flag.
    if ((FLASH->SR & (0b1 << 1)) != 0) {
        // Clear flag.
        FLASH->SR = (0b1 << 1);
    }
    // Check error flags.
    if (((FLASH->SR) & NVM_ERROR_FLAGS_MASK) != 0) {
        // Clear flags and return error.
        FLASH->SR = NVM_ERROR_FLAGS_MASK;
        // Exit with error.
        status = NVM_ERROR_WRITE_OPERATION;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
static NVM_status_t _NVM_unlock(void) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    // Check memory is ready.
    status = _NVM_check_busy(NVM_ERROR_UNLOCK_READY);
    if (status != NVM_SUCCESS) goto errors;
    // Check the memory is not already unlocked.
    if (((FLASH->PECR) & (0b1 << 0)) != 0) {
        // Perform unlock sequence.
        FLASH->PEKEYR = 0x89ABCDEF;
        FLASH->PEKEYR = 0x02030405;
    }
    // Check if unlock sequence completed successfully.
    if (((FLASH->PECR) & (0b1 << 0)) != 0) {
        status = NVM_ERROR_UNLOCK_SEQUENCE;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
static void _NVM_lock(void) {
    // Lock sequence.
    FLASH->PECR |= (0b1 << 0);
}

/*** NVM functions ***/

/*******************************************************************/
NVM_status_t NVM_read(uint32_t address, void* data, uint8_t data_size, NVM_data_type_t data_type) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    uint32_t absolute_base_address = (NVM_EEPROM_ADDRESS + address);
    uint32_t absolute_address = 0;
    uint8_t global_interrupts = NVIC_get_global_interrupts();
    uint32_t idx = 0;
    // Check parameters.
    _NVM_check_parameters();
    // Disable all interrupts.
    NVIC_set_global_interrupts(0);
    // Enable peripheral.
    RCC->AHBENR |= (0b1 << 8); // MIFEN='1'.
    // Data loop.
    for (idx = 0; idx < data_size; idx++) {
        // Check there is no pending operation.
        status = _NVM_check_busy(NVM_ERROR_READ_READY);
        if (status != NVM_SUCCESS) goto errors;
        // Compute absolute address.
        absolute_address = (absolute_base_address + (idx << data_type));
        // Read data.
        switch (data_type) {
        case NVM_DATA_TYPE_BYTE:
            _NVM_read(uint8_t);
            break;
        case NVM_DATA_TYPE_SHORT:
            _NVM_read(uint16_t);
            break;
        case NVM_DATA_TYPE_LONG:
            _NVM_read(uint32_t);
            break;
        default:
            status = NVM_ERROR_DATA_TYPE;
            goto errors;
        }
    }
errors:
    // Disable peripheral.
    RCC->AHBENR &= ~(0b1 << 8); // MIFEN='0'.
    // Restore interrupts.
    NVIC_set_global_interrupts(global_interrupts);
end:
    return status;
}

/*******************************************************************/
NVM_status_t NVM_write(uint32_t address, void* data, uint8_t data_size, NVM_data_type_t data_type) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    uint32_t absolute_base_address = (NVM_EEPROM_ADDRESS + address);
    uint32_t absolute_address = 0;
    uint8_t global_interrupts = NVIC_get_global_interrupts();
    uint16_t idx = 0;
    // Check parameters.
    _NVM_check_parameters();
    // Disable all interrupts.
    NVIC_set_global_interrupts(0);
    // Enable peripheral.
    RCC->AHBENR |= (0b1 << 8); // MIFEN='1'.
    // Unlock memory.
    status = _NVM_unlock();
    if (status != NVM_SUCCESS) goto errors;
    // Data loop.
    for (idx = 0; idx < data_size; idx++) {
        // Check there is no pending operation.
        status = _NVM_check_busy(NVM_ERROR_WRITE_READY);
        if (status != NVM_SUCCESS) goto errors;
        // Compute absolute address.
        absolute_address = (absolute_base_address + (idx << data_type));
        // Write data.
        switch (data_type) {
        case NVM_DATA_TYPE_BYTE:
            _NVM_write_verify(uint8_t);
            break;
        case NVM_DATA_TYPE_SHORT:
            _NVM_write_verify(uint16_t);
            break;
        case NVM_DATA_TYPE_LONG:
            _NVM_write_verify(uint32_t);
            break;
        default:
            status = NVM_ERROR_DATA_TYPE;
            goto errors;
        }
    }
errors:
    // Lock memory.
    _NVM_lock();
    // Disable peripheral.
    RCC->AHBENR &= ~(0b1 << 8); // MIFEN='0'.
    // Restore interrupts.
    NVIC_set_global_interrupts(global_interrupts);
end:
    return status;
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
